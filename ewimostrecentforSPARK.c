#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/resets.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include "ff.h"
#include "diskio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include <math.h>
#include "sd_audio.h"
 
// --- FORWARD DECLARATIONS ---
void init_spi_sdcard(void);
void enable_sdcard(void);
void disable_sdcard(void);
void init_sdcard_io(void);
void init_ad5693(void);
void ad5693_write(uint16_t sample);
void meter_out(void);
void dac_write_scaled(uint16_t sample);
void button_callback(uint gpio, uint32_t events);
void init_buttons(void);
void init_mouthpiece(void);
void meter_init(void);
 
// --- CONFIGURATION ---
#define I2C_PORT        i2c1
#define TAIL_TRIM 50
#define AD5693_ADDR     0x4C
#define CMD_WRITE_UPDATE 0x30
#define THRESHOLD_ON    250
#define THRESHOLD_OFF   250
#define ADC_OFFSET      250
#define ADC_MAX         4095
#define NUM_BUTTONS     10
#define CROSSFADE_LEN 512

#define CROSSFADE_LEN     512
#define LOOP_START_OFFSET 1000 // Skip the initial attack/silence
#define LOOP_END_OFFSET   1000 // Trim the trailing decay/silence
const uint BUTTON_PINS[NUM_BUTTONS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
 
// --- BUTTON STATE ---
volatile uint16_t button_state    = 0;
volatile uint16_t button_pressed  = 0;
volatile uint16_t button_released = 0;

static volatile float breath_scale = 0.0f;

//tremelo
static volatile float tremolo_depth = 0.0f;
static volatile float tremolo_phase = 0.0f;
#define TREMOLO_STEP 0.2f
#define TREMOLO_RATE 6.0f
static int trem_counter = 0;
static float trem_gain = 1.0f;
 
// LED meter pins
#define LED_PIN_1   32
#define LED_PIN_2   33
#define LED_PIN_3   34
#define LED_PIN_MAX 35
 
// SD SPI pins
#define SD_SPI_PORT spi0
#define SD_PIN_MISO 16
#define SD_PIN_CS   17
#define SD_PIN_SCK  18
#define SD_PIN_MOSI 19

//display
#define DISP_CS    41
#define DISP_SCK   42
#define DISP_MOSI  43
#define DISP_RESET 20
#define DISP_DC    21
 
volatile uint32_t button_pending_time[NUM_BUTTONS] = {0};
volatile bool     button_pending[NUM_BUTTONS]       = {false};
uint32_t          last_action_time[NUM_BUTTONS]     = {0};
 
// --- MULTI-VOICE AUDIO ---
// At 44100 Hz 16-bit mono, 8192 samples = ~186 ms per voice loop.
// Adjust MAX_SAMPLES up/down based on how much RAM you can spare.
// 4 voices × 8192 samples × 2 bytes = 64 KB total.
#define MAX_SAMPLES  8192
#define NUM_VOICES   4
 
static int16_t voice_buf[NUM_VOICES][MAX_SAMPLES];
static int     voice_samples[NUM_VOICES] = {0};
static int     active_voice = 0;
 
// Names of WAV files on the SD card, one per voice
const char *voice_files[NUM_VOICES] = {
    "sine.wav",
    "clarinet.wav",
    "flute.wav",
    "oboe.wav"
};
 
// --- PLAYBACK STATE ---
static float sample_index = 0.0f;
static float pitch_mult   = 1.0f;
 
struct repeating_timer audio_timer;
volatile bool playing = false;
 
volatile bool update_meter = false;
uint32_t      meter_tick   = 0;
 
// -----------------------------
// BUTTON HANDLING
// -----------------------------
void check_buttons(void) {
    uint32_t now      = time_us_32();
    uint32_t all_pins = gpio_get_all();
 
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (button_pending[i]) {
            if (now - button_pending_time[i] >= 1000) {   // 1 ms debounce
                bool is_pressed_now = (all_pins & (1u << BUTTON_PINS[i]));
                bool last_state     = (button_state & (1u << i));
 
                if (is_pressed_now != last_state) {
                    if (now - last_action_time[i] >= 10000) { // 10 ms lockout
                        if (is_pressed_now) {
                            button_state   |=  (1u << i);
                            button_pressed |=  (1u << i);
                        } else {
                            button_state    &= ~(1u << i);
                            button_released |=  (1u << i);
                        }
                        last_action_time[i] = now;
                    }
                }
                button_pending[i] = false;
            }
        }
    }
}
 
void button_callback(uint gpio, uint32_t events) {
    int i = gpio - 2;
    if (i >= 0 && i < NUM_BUTTONS) {
        button_pending[i]      = true;
        button_pending_time[i] = time_us_32();
    }
}
 
void init_buttons(void) {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        gpio_init(BUTTON_PINS[i]);
        gpio_set_dir(BUTTON_PINS[i], GPIO_IN);
        gpio_set_irq_enabled_with_callback(
            BUTTON_PINS[i],
            GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
            true,
            &button_callback
        );
    }
}
 
// -----------------------------
// DAC / VOLUME SCALING
// -----------------------------
void dac_write_scaled(uint16_t sample) {
    int32_t  s      = (int32_t)sample - 0x8000;
    int32_t  scaled = (int32_t)(s * breath_scale);
    uint16_t out    = (uint16_t)(scaled + 0x8000);
    uint8_t  tx[3]  = {CMD_WRITE_UPDATE, (uint8_t)(out >> 8), (uint8_t)(out & 0xFF)};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false); 
 
    // float scale = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
    // if (scale > 1.0f) scale = 1.0f;
    // if (scale < 0.0f) scale = 0.0f;
    // scale = sqrtf(scale);
 
    // int32_t  signed_sample = (int32_t)sample - 0x8000;
    // int32_t  scaled        = (int32_t)(signed_sample * scale);
    // uint16_t output        = (uint16_t)(scaled + 0x8000);
 
    // uint8_t tx[3] = {CMD_WRITE_UPDATE, (uint8_t)(output >> 8), (uint8_t)(output & 0xFF)};
    // i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
}
 
// -----------------------------
// PITCH: BUTTONS → SEMITONES
// -----------------------------
// static float pitch_multiplier_from_semitones(int semitones) {
//     return powf(2.0f, semitones / 12.0f);
// }
 
// Upper nibble of button_state encodes the valve fingering.
// Semitone offsets are relative to Bb (the base sample frequency).
static int semitones_from_valves(uint8_t state) {
    uint8_t valves = (state >> 4) & 0x0F;
    switch (valves) {
        case 0b0000: return 0;  // Bb  (1 semitone below base so powf gives exact Bb)
        case 0b0111: return  1;  // B
        case 0b0101: return  2;  // C
        case 0b0110: return  3;  // Db
        case 0b0011: return  4;  // D
        case 0b0001: return  5;  // Eb
        case 0b0010: return  6;  // E
        case 0b1000: return  7;  // F
        case 0b1110: return  8;  // Gb
        case 0b1011: return  9;  // G
        case 0b1001: return  10;  // Ab
        case 0b1010: return 11;  // A
        default:     return 0;  // Bb fallback
    }
}
 
static void update_pitch_from_buttons(void) {
    uint8_t state    = button_state;
    int     semitones = semitones_from_valves(state);
 
    // Lower nibble: octave shift buttons (GP2–GP5)
    int octave_shift = 0;
    if (state & (1u << 0)) octave_shift = 12;   // GP2 → +1 octave
    if (state & (1u << 1)) octave_shift = 24;   // GP3 → +2 octaves
    if (state & (1u << 2)) octave_shift = 36;   // GP4 → +3 octaves
    if (state & (1u << 3)) octave_shift = 48;   // GP5 → +4 octaves
 
    semitones += octave_shift;
    pitch_mult = pitch_multiplier_from_semitones(semitones);
}
 
// -----------------------------
// VOICE SWITCHING
// -----------------------------
// static void switch_voice(int v) {
//     if (v < 0 || v >= NUM_VOICES) return;
//     if (voice_samples[v] < 2) return;
//     active_voice = v;
//     sample_index = 0.0f;   // reset phase cleanly on voice change
// }
 
// -----------------------------
// TIMER CALLBACK: AUDIO OUTPUT
// -----------------------------
bool timer_callback(struct repeating_timer *t) {
if (++meter_tick >= 100) {
        meter_tick   = 0;
        update_meter = true;
    }

    if (adc_hw->result < THRESHOLD_OFF) {
        playing = false;
        tremolo_depth = 0.0f;
        tremolo_phase = 0.0f;
        uint8_t tx[3] = {CMD_WRITE_UPDATE, 0x80, 0x00};
        i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
        return true;

    }

    if (!playing) return true;

    int n = voice_samples[active_voice];
    if (n < 2) return true;

    // 1. Establish boundaries to skip silence/attack/decay
    int loop_start = LOOP_START_OFFSET;
    int loop_end   = n - LOOP_END_OFFSET;
    int current_crossfade = CROSSFADE_LEN;

    // Safety check: if the loaded WAV is too short, fall back to safe values
    if (loop_end <= loop_start + current_crossfade) {
        loop_start = 0;
        loop_end   = n;
        current_crossfade = n / 4; 
    }

    int i0 = (int)sample_index;
    
    // Ensure playhead starts inside our valid loop region
    if (i0 < loop_start) {
        sample_index = (float)loop_start;
        i0 = loop_start;
    }

    int i1 = i0 + 1;
    if (i1 >= loop_end) i1 = loop_start;

    float frac = sample_index - (float)i0;
    
    // Base Interpolation
    float s = voice_buf[active_voice][i0] * (1.0f - frac)
            + voice_buf[active_voice][i1] * frac;

    // 2. Crossfade Logic
    int crossfade_start = loop_end - current_crossfade;
    if (i0 >= crossfade_start) {
        // Calculate crossfade progression (0.0 to 1.0)
        float cf = (float)(i0 - crossfade_start) / (float)current_crossfade; 
        
        // Find the equivalent position at the START of the loop
        int j0 = loop_start + (i0 - crossfade_start);
        int j1 = j0 + 1;
        if (j1 >= loop_end) j1 = loop_start;
        
        float s_loop = voice_buf[active_voice][j0] * (1.0f - frac)
                     + voice_buf[active_voice][j1] * frac;
                     
        // Blend out the tail (s), and blend in the start (s_loop)
        s = s * (1.0f - cf) + s_loop * cf; 
    }

    uint16_t out = (uint16_t)((int16_t)s + 0x8000);

    //implemeting tremelo
    if (++trem_counter >= 44) {
    trem_counter = 0;
    tremolo_phase += TREMOLO_RATE / 1000.0f;
    if (tremolo_phase > 1.0f) tremolo_phase -= 1.0f;
    trem_gain = 1.0f - tremolo_depth * (tremolo_phase < 0.5f ? tremolo_phase * 2.0f : (1.0f - tremolo_phase) * 2.0f);
    }
    int32_t tremmed = (int32_t)(((int32_t)out - 0x8000) * trem_gain) + 0x8000;
    if (tremmed > 0xFFFF) tremmed = 0xFFFF;
    if (tremmed < 0) tremmed = 0;
    out = (uint16_t)tremmed;


    dac_write_scaled(out);

    sample_index += pitch_mult;

    // 3. The Wrap-Around (The Crucial Math Fix)
    // Because we successfully blended the audio to sound like `loop_start + current_crossfade`, 
    // we MUST jump the playhead to that exact spot to prevent a time-skip pop.
    if (sample_index >= (float)loop_end) {
        sample_index = sample_index - (float)loop_end + (float)loop_start + (float)current_crossfade;
    }

    return true;

    // if (++meter_tick >= 100) {
    //     meter_tick   = 0;
    //     update_meter = true;
    // }

    // if (adc_hw->result < THRESHOLD_OFF) {
    //     playing = false;
    //     uint8_t tx[3] = {CMD_WRITE_UPDATE, 0x80, 0x00};
    //     i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
    //     return true;
    // }

    // if (!playing) return true;

    // int n = voice_samples[active_voice];
    // if (n < 2) return true;

    // int i0 = (int)sample_index;
    // int i1 = i0 + 1;
    // if (i1 >= n) i1 = 0;

    // float frac = sample_index - (float)i0;
    // float s    = voice_buf[active_voice][i0] * (1.0f - frac)
    //            + voice_buf[active_voice][i1] * frac;

    // uint16_t out = (uint16_t)((int16_t)s + 0x8000);
    // dac_write_scaled(out);

    // sample_index += pitch_mult;
    // if (sample_index >= (float)n) sample_index -= (float)n;

    // return true;

    // if (++meter_tick >= 100) {
    //     meter_tick   = 0;
    //     update_meter = true;
    //     // meter_out() moved to main loop — never do work here
    // }

    // if (adc_hw->result < THRESHOLD_OFF) {
    //     playing = false;
    //     uint8_t tx[3] = {CMD_WRITE_UPDATE, 0x80, 0x00};
    //     i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
    //     return true;
    // }

    // if (!playing) return true;

    // int n = voice_samples[active_voice];
    // if (n < 2) return true;

    // int   i0   = (int)sample_index % n;
    // int   i1   = (i0 + 1) % n;
    // float frac = sample_index - (float)(int)sample_index;

    // float s  = voice_buf[active_voice][i0] * (1.0f - frac)
    //          + voice_buf[active_voice][i1] * frac;

    // uint16_t out = (uint16_t)((int16_t)s + 0x8000);
    // dac_write_scaled(out);

    // sample_index += pitch_mult;
    // if (sample_index >= (float)n) sample_index -= (float)n;

    // return true;
    // if (++meter_tick >= 100) { meter_tick = 0; update_meter = true; }

    // if (adc_hw->result < THRESHOLD_OFF) {
    //     playing = false;
    //     uint8_t tx[3] = {CMD_WRITE_UPDATE, 0x80, 0x00};
    //     i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
    //     return true;
    // }

    // if (!playing) return true;

    // int n = voice_samples[active_voice];
    // if (n < 2) return true;

    // int   i0   = (int)sample_index % n;
    // int   i1   = (i0 + 1) % n;
    // float frac = sample_index - (float)(int)sample_index;

    // float s = voice_buf[active_voice][i0] * (1.0f - frac)
    //         + voice_buf[active_voice][i1] * frac;

    // // Crossfade: when near the end of the buffer, blend with the start
    // int crossfade_start = n - CROSSFADE_LEN;
    // if (i0 >= crossfade_start) {
    //     float cf = (float)(i0 - crossfade_start) / (float)CROSSFADE_LEN; // 0→1

    //     int   j0     = i0 - crossfade_start;   // equivalent position at start of buffer
    //     int   j1     = j0 + 1;
    //     float s_loop = voice_buf[active_voice][j0] * (1.0f - frac)
    //                  + voice_buf[active_voice][j1] * frac;

    //     s = s * (1.0f - cf) + s_loop * cf;  // blend end→start
    // }

    // uint16_t out = (uint16_t)((int16_t)s + 0x8000);
    // dac_write_scaled(out);

    // sample_index += pitch_mult;
    // if (sample_index >= (float)n) sample_index -= (float)n;

    // if (update_meter) { update_meter = false; meter_out(); }
    // return true;
}
 
// -----------------------------
// SD / I2C / ADC / METER INIT
// -----------------------------
void sdcard_io_high_speed(void) {
    spi_set_baudrate(spi0, 12 * 1000 * 1000);
}
 
void init_ad5693(void) {
    i2c_init(I2C_PORT, 1000 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    sleep_ms(10);
 
    uint8_t reset_cmd[3] = {0x60, 0x00, 0x00};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, reset_cmd, 3, false);
    sleep_ms(10);
 
    uint8_t config[3] = {0x40, 0x00, 0x00};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, config, 3, false);
    sleep_ms(10);
}
 
void ad5693_write(uint16_t sample) {
    uint8_t buf[3];
    buf[0] = CMD_WRITE_UPDATE;
    buf[1] = (uint8_t)(sample >> 8);
    buf[2] = (uint8_t)(sample & 0xFF);
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, buf, 3, false);
}
 
void init_spi_sdcard(void) {
    gpio_init(16); gpio_init(17); gpio_init(18); gpio_init(19);
 
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);
 
    spi_init(spi0, 100 * 1000);
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
 
    spi_set_baudrate(spi0, 20 * 1000 * 1000);
 
    printf("SPI pins configured.\n");
}
 
void disable_sdcard(void) { gpio_put(17, true); }
void enable_sdcard(void)  { gpio_put(17, 0);    }
 
void init_sdcard_io(void) {
    init_spi_sdcard();
}
 
void init_mouthpiece(void) {
    adc_init();
    adc_gpio_init(45);
    adc_select_input(5);
    adc_run(1);
}
 
void meter_init(void) {
    const uint MAX = 4095;
 
    gpio_init(32); gpio_init(33); gpio_init(34); gpio_init(35);
 
    gpio_set_function(32, GPIO_FUNC_PWM);
    gpio_set_function(33, GPIO_FUNC_PWM);
    gpio_set_function(34, GPIO_FUNC_PWM);
    gpio_set_function(35, GPIO_FUNC_SIO);
 
    gpio_set_dir(32, true);
    gpio_set_dir(33, true);
    gpio_set_dir(34, true);
    gpio_set_dir(35, true);
 
    uint slice1 = pwm_gpio_to_slice_num(32);
    uint slice2 = pwm_gpio_to_slice_num(33);
    uint slice3 = pwm_gpio_to_slice_num(34);
 
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, MAX);
    pwm_init(slice1, &config, true);
    pwm_init(slice2, &config, true);
    pwm_init(slice3, &config, true);
}
 
void meter_out(void) {
    const uint led_pins[3] = {32, 33, 34};
    const uint MAX = 4095;
 
    float level;
    if      (adc_hw->result >= 4095) level = 1.0f;
    else if (adc_hw->result <= 0)    level = 0.0f;
    else                             level = (float)(adc_hw->result) / 4095.0f;
 
    float active_led = level * 3.0f;
 
    for (int i = 0; i < 3; i++) {
        float brightness = 0.0f;
        if      (active_led >= (float)(i + 1)) brightness = 1.0f;
        else if (active_led <= 0.15f)          brightness = 0.0f;
        else if (active_led >  (float)i)       brightness = active_led - (float)i;
 
        pwm_set_gpio_level(led_pins[i], (uint16_t)(brightness * MAX));
    }
 
    gpio_put(35, adc_hw->result == 4095);
}
 
void disp_cmd(uint8_t cmd) {
    gpio_put(DISP_DC, 0);
    gpio_put(DISP_CS, 0);
    spi_write_blocking(spi1, &cmd, 1);
    gpio_put(DISP_CS, 1);
}

void disp_data(uint8_t data) {
    gpio_put(DISP_DC, 1);
    gpio_put(DISP_CS, 0);
    spi_write_blocking(spi1, &data, 1);
    gpio_put(DISP_CS, 1);
}

void disp_fill(uint16_t color) {
    disp_cmd(0x2A); disp_data(0); disp_data(0); disp_data(0); disp_data(239);
    disp_cmd(0x2B); disp_data(0); disp_data(0); disp_data(1); disp_data(63);
    disp_cmd(0x2C);
    gpio_put(DISP_DC, 1);
    gpio_put(DISP_CS, 0);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (int i = 0; i < 240 * 320; i++) {
        spi_write_blocking(spi1, &hi, 1);
        spi_write_blocking(spi1, &lo, 1);
    }
    gpio_put(DISP_CS, 1);
}

void disp_pixel(int x, int y, uint16_t color) {
    disp_cmd(0x2A); disp_data(x>>8); disp_data(x&0xFF); disp_data(x>>8); disp_data(x&0xFF);
    disp_cmd(0x2B); disp_data(y>>8); disp_data(y&0xFF); disp_data(y>>8); disp_data(y&0xFF);
    disp_cmd(0x2C);
    gpio_put(DISP_DC, 1);
    gpio_put(DISP_CS, 0);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    spi_write_blocking(spi1, &hi, 1);
    spi_write_blocking(spi1, &lo, 1);
    gpio_put(DISP_CS, 1);
}

// Draw a thick pixel block (for big text)
void disp_block(int x, int y, int size, uint16_t color) {
    for (int dy = 0; dy < size; dy++)
        for (int dx = 0; dx < size; dx++)
            disp_pixel(x+dx, y+dy, color);
}

// Big chunky letters for E, W, I
void disp_letter_E(int x, int y, uint16_t color) {
    int s = 8;
    for (int i = 0; i < 5; i++) disp_block(x, y+i*s, s, color);      // left bar
    for (int i = 0; i < 4; i++) disp_block(x+i*s, y, s, color);       // top
    for (int i = 0; i < 3; i++) disp_block(x+i*s, y+2*s, s, color);   // middle
    for (int i = 0; i < 4; i++) disp_block(x+i*s, y+4*s, s, color);   // bottom
}

void disp_letter_W(int x, int y, uint16_t color) {
    int s = 8;
    int w = 4*s;  // total width
    int h = 4*s;  // total height
    for (int i = 0; i < 5; i++) disp_block(x+w,     y+h-i*s, s, color);  // left bar
    for (int i = 0; i < 5; i++) disp_block(x+w-4*s, y+h-i*s, s, color);  // right bar
    for (int i = 2; i < 5; i++) disp_block(x+w-2*s, y+h-i*s, s, color);  // middle
    disp_block(x+w-s,   y+h-3*s, s, color);
    disp_block(x+w-3*s, y+h-3*s, s, color);
}


void disp_letter_I(int x, int y, uint16_t color) {
    int s = 8;
    for (int i = 0; i < 4; i++) disp_block(x+i*s, y, s, color);       // top
    for (int i = 0; i < 5; i++) disp_block(x+s, y+i*s, s, color);     // middle
    for (int i = 0; i < 4; i++) disp_block(x+i*s, y+4*s, s, color);   // bottom
}

void init_display() {
    spi_init(spi1, 20 * 1000 * 1000);
    gpio_set_function(DISP_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(DISP_MOSI, GPIO_FUNC_SPI);

    gpio_init(DISP_CS);    gpio_set_dir(DISP_CS,    GPIO_OUT); gpio_put(DISP_CS,    1);
    gpio_init(DISP_DC);    gpio_set_dir(DISP_DC,    GPIO_OUT); gpio_put(DISP_DC,    1);
    gpio_init(DISP_RESET); gpio_set_dir(DISP_RESET, GPIO_OUT); gpio_put(DISP_RESET, 1);

    gpio_put(DISP_RESET, 0); sleep_ms(10);
    gpio_put(DISP_RESET, 1); sleep_ms(120);

    disp_cmd(0x01); sleep_ms(5);   // software reset
    disp_cmd(0x11); sleep_ms(120); // sleep out
    disp_cmd(0x3A); disp_data(0x55); // 16-bit color
    disp_cmd(0x29);                // display on

    disp_fill(0x0000);  // black background

    // Draw EWI centered
    disp_letter_E(20,  150, 0xFFFF);
    disp_letter_W(90,  150, 0xFFFF);
    disp_letter_I(180, 150, 0xFFFF);
}

// -----------------------------
// FAT TIME (for FatFs)
// -----------------------------
DWORD get_fattime(void) {
    return ((DWORD)(2026 - 1980) << 25) |
           ((DWORD)1  << 21) |
           ((DWORD)1  << 16) |
           ((DWORD)0  << 11) |
           ((DWORD)0  <<  5) |
           ((DWORD)0  >>  1);
}
 
// -----------------------------
// MAIN
// -----------------------------
int main(void) {
    stdio_init_all();
    sleep_ms(500);

    init_display();

    init_mouthpiece();
    meter_init();
    init_buttons();
    init_ad5693();
    init_sdcard_io();

    if (!sd_audio_init()) {
        printf("sd_audio_init failed\n");
        while (1) tight_loop_contents();
    }

    for (int v = 0; v < NUM_VOICES; v++) {
        if (!sd_audio_open(voice_files[v])) {
            printf("Warning: failed to open %s, voice %d disabled\n", voice_files[v], v);
            voice_samples[v] = 0;
            continue;
        }

        wav_info_t info = sd_audio_get_info();
        printf("Voice %d — %s: %lu Hz, %u bits\n",
               v, voice_files[v],
               (unsigned long)info.sample_rate,
               (unsigned)info.bits_per_sample);

        UINT bytes_read = sd_audio_read(
            (uint8_t*)voice_buf[v],
            MAX_SAMPLES * sizeof(int16_t)
        );
        voice_samples[v] = (int)(bytes_read / sizeof(int16_t));
        voice_samples[v] -= TAIL_TRIM;   // <-- add here
        printf("  Loaded %d samples (%.1f ms)\n",
               voice_samples[v],
               1000.0f * voice_samples[v] / (float)info.sample_rate);
    }

    active_voice = 0;
    for (int v = 0; v < NUM_VOICES; v++) {
        if (voice_samples[v] >= 2) { active_voice = v; break; }
    }

    if (voice_samples[active_voice] < 2) {
        printf("No valid voices loaded — halting\n");
        while (1) tight_loop_contents();
    }

    if (!sd_audio_open(voice_files[0])) {
        printf("Cannot re-open voice 0 for rate info\n");
        while (1) tight_loop_contents();
    }
    wav_info_t info0 = sd_audio_get_info();
    int64_t period_us = -(int64_t)(1000000.0f / (float)info0.sample_rate);
    add_repeating_timer_us(period_us, timer_callback, NULL, &audio_timer);

    playing = true;

    while (1) {
        check_buttons();
        update_pitch_from_buttons();

        // Pre-compute breath scale here — keeps sqrtf out of ISR
        float raw = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
        if (raw > 1.0f) raw = 1.0f;
        if (raw < 0.0f) raw = 0.0f;
        breath_scale = sqrtf(raw);

        if (update_meter) {
            update_meter = false;
            meter_out();
        }

        for (int i = 0; i < 8; i++) {
            if (button_pressed & (1u << i)) {
                button_pressed &= ~(1u << i);
                printf("Button %d pressed\n", i);
            }
            if (button_released & (1u << i)) {
                button_released &= ~(1u << i);
                printf("Button %d released\n", i);
            }
        }

        if (button_pressed & (1 << 8)) {
        button_pressed &= ~(1 << 8);
        tremolo_depth += TREMOLO_STEP;
        if (tremolo_depth > 1.0f) tremolo_depth = 1.0f;
        printf("Tremolo up: %.1f\n", tremolo_depth);
    }

    if (button_pressed & (1 << 9)) {
        button_pressed &= ~(1 << 9);
        tremolo_depth -= TREMOLO_STEP;
        if (tremolo_depth < 0.0f) tremolo_depth = 0.0f;
        printf("Tremolo down: %.1f\n", tremolo_depth);
    }

        if      (adc_hw->result > THRESHOLD_ON  && !playing) playing = true;
        else if (adc_hw->result < THRESHOLD_OFF &&  playing) playing = false;

        tight_loop_contents();
    }

    return 0;

    // stdio_init_all();
    // sleep_ms(500);

    // init_mouthpiece();
    // meter_init();
    // init_buttons();
    // init_ad5693();
    // init_sdcard_io();

    // if (!sd_audio_init()) {
    //     printf("sd_audio_init failed\n");
    //     while (1) tight_loop_contents();
    // }

    // for (int v = 0; v < NUM_VOICES; v++) {
    //     if (!sd_audio_open(voice_files[v])) {
    //         printf("Warning: failed to open %s, voice %d disabled\n", voice_files[v], v);
    //         voice_samples[v] = 0;
    //         continue;
    //     }

    //     wav_info_t info = sd_audio_get_info();
    //     printf("Voice %d — %s: %lu Hz, %u bits\n",
    //            v, voice_files[v],
    //            (unsigned long)info.sample_rate,
    //            (unsigned)info.bits_per_sample);

    //     UINT bytes_read = sd_audio_read(
    //         (uint8_t*)voice_buf[v],
    //         MAX_SAMPLES * sizeof(int16_t)
    //     );
    //     voice_samples[v] = (int)(bytes_read / sizeof(int16_t));
    //     printf("  Loaded %d samples (%.1f ms)\n",
    //            voice_samples[v],
    //            1000.0f * voice_samples[v] / (float)info.sample_rate);
    // }

    // active_voice = 0;
    // for (int v = 0; v < NUM_VOICES; v++) {
    //     if (voice_samples[v] >= 2) { active_voice = v; break; }
    // }

    // if (voice_samples[active_voice] < 2) {
    //     printf("No valid voices loaded — halting\n");
    //     while (1) tight_loop_contents();
    // }

    // if (!sd_audio_open(voice_files[0])) {
    //     printf("Cannot re-open voice 0 for rate info\n");
    //     while (1) tight_loop_contents();
    // }
    // wav_info_t info0 = sd_audio_get_info();
    // int64_t period_us = -(int64_t)(1000000.0f / (float)info0.sample_rate);
    // add_repeating_timer_us(period_us, timer_callback, NULL, &audio_timer);

    // playing = true;

    // while (1) {
    //     check_buttons();
    //     update_pitch_from_buttons();

    //     // Meter update moved here — safe, not in ISR
    //     if (update_meter) {
    //         update_meter = false;
    //         meter_out();
    //     }

    //     for (int i = 0; i < NUM_BUTTONS; i++) {
    //         if (button_pressed & (1u << i)) {
    //             button_pressed &= ~(1u << i);
    //             printf("Button %d pressed\n", i);
    //         }
    //         if (button_released & (1u << i)) {
    //             button_released &= ~(1u << i);
    //             printf("Button %d released\n", i);
    //         }
    //     }

    //     if      (adc_hw->result > THRESHOLD_ON  && !playing) playing = true;
    //     else if (adc_hw->result < THRESHOLD_OFF &&  playing) playing = false;

    //     tight_loop_contents();
    // }

    // return 0;

    // stdio_init_all();
    // sleep_ms(500);
 
    // init_mouthpiece();
    // meter_init();
    // init_buttons();
    // init_ad5693();
    // init_sdcard_io();
 
    // if (!sd_audio_init()) {
    //     printf("sd_audio_init failed\n");
    //     while (1) tight_loop_contents();
    // }
 
    // // --- Load all voices into RAM at startup ---
    // for (int v = 0; v < NUM_VOICES; v++) {
    //     if (!sd_audio_open(voice_files[v])) {
    //         printf("Warning: failed to open %s, voice %d disabled\n", voice_files[v], v);
    //         voice_samples[v] = 0;
    //         continue;
    //     }
 
    //     wav_info_t info = sd_audio_get_info();
    //     printf("Voice %d — %s: %lu Hz, %u bits\n",
    //            v, voice_files[v],
    //            (unsigned long)info.sample_rate,
    //            (unsigned)info.bits_per_sample);
 
    //     UINT bytes_read = sd_audio_read(
    //         (uint8_t*)voice_buf[v],
    //         MAX_SAMPLES * sizeof(int16_t)
    //     );
    //     voice_samples[v] = (int)(bytes_read / sizeof(int16_t));
    //     printf("  Loaded %d samples (%.1f ms)\n",
    //            voice_samples[v],
    //            1000.0f * voice_samples[v] / (float)info.sample_rate);
    // }
 
    // // Default to voice 0; find the first valid voice if 0 failed
    // active_voice = 0;
    // for (int v = 0; v < NUM_VOICES; v++) {
    //     if (voice_samples[v] >= 2) { active_voice = v; break; }
    // }
 
    // if (voice_samples[active_voice] < 2) {
    //     printf("No valid voices loaded — halting\n");
    //     while (1) tight_loop_contents();
    // }
 
    // // Timer period is based on voice 0's sample rate.
    // // If your voices have different sample rates, store per-voice and reload here.
    // if (!sd_audio_open(voice_files[0])) {
    //     printf("Cannot re-open voice 0 for rate info\n");
    //     while (1) tight_loop_contents();
    // }
    // wav_info_t info0 = sd_audio_get_info();
    // int64_t period_us = -(int64_t)(1000000.0f / (float)info0.sample_rate);
    // add_repeating_timer_us(period_us, timer_callback, NULL, &audio_timer);
 
    // playing = true;
 
    // while (1) {
    //     check_buttons();
    //     update_pitch_from_buttons();
 
    //     // Print button events
    //     for (int i = 0; i < NUM_BUTTONS; i++) {
    //         if (button_pressed & (1u << i)) {
    //             button_pressed &= ~(1u << i);
    //             printf("Button %d pressed\n", i);
    //         }
    //         if (button_released & (1u << i)) {
    //             button_released &= ~(1u << i);
    //             printf("Button %d released\n", i);
    //         }
    //     }
 
    //     // Breath gate
    //     if      (adc_hw->result > THRESHOLD_ON  && !playing) playing = true;
    //     else if (adc_hw->result < THRESHOLD_OFF &&  playing) playing = false;
 
    //     tight_loop_contents();
    // }
 
    // return 0;
}

// #include "pico/stdlib.h"
// #include "hardware/timer.h"
// #include "hardware/irq.h"
// #include "hardware/adc.h"
// #include "hardware/resets.h"
// #include "hardware/dma.h"
// #include "hardware/pwm.h"
// #include "hardware/gpio.h"
// #include <stdio.h>
// #include "ff.h"
// #include "diskio.h"
// #include "hardware/i2c.h"
// #include "hardware/spi.h"
// #include <math.h>
// #include "sd_audio.h"

// // --- FORWARD DECLARATIONS ---
// void init_spi_sdcard(void);
// void enable_sdcard(void);
// void disable_sdcard(void);
// void init_sdcard_io(void);
// void init_ad5693(void);
// void ad5693_write(uint16_t sample);
// void meter_out(void);
// void dac_write_scaled(uint16_t sample);
// void button_callback(uint gpio, uint32_t events);
// void init_buttons(void);
// void init_mouthpiece(void);
// void meter_init(void);

// // --- CONFIGURATION ---
// #define I2C_PORT        i2c1
// #define AD5693_ADDR     0x4C // Default address (check your ADDR pin)
// #define CMD_WRITE_UPDATE 0x30
// #define THRESHOLD_ON   250
// #define THRESHOLD_OFF  250
// #define ADC_OFFSET     250
// #define ADC_MAX        4095
// #define NUM_BUTTONS 8
// const uint BUTTON_PINS[NUM_BUTTONS] = {2, 3, 4, 5, 6, 7, 8, 9};

// // --- BUTTON STATE ---
// volatile uint8_t button_state    = 0;  // bitmask of current state
// volatile uint8_t button_pressed  = 0;  // newly pressed
// volatile uint8_t button_released = 0;  // newly released

// // LED meter pins
// #define LED_PIN_1 32
// #define LED_PIN_2 33
// #define LED_PIN_3 34
// #define LED_PIN_MAX 35

// // SD SPI pins
// #define SD_SPI_PORT spi0
// #define SD_PIN_MISO 16
// #define SD_PIN_CS   17
// #define SD_PIN_SCK  18
// #define SD_PIN_MOSI 19

// volatile uint32_t button_pending_time[NUM_BUTTONS] = {0};
// volatile bool button_pending[NUM_BUTTONS] = {false};
// uint32_t last_action_time[NUM_BUTTONS] = {0};

// // --- AUDIO / PITCH STATE ---
// #define STREAM_BUFFER_SAMPLES 256

// static int16_t audio_buf[2][STREAM_BUFFER_SAMPLES];
// static volatile int play_buf = 0;
// static volatile int ready_buf = 1;
// static volatile int play_samples = 0;
// static volatile bool buf_ready = false;

// static float sample_index = 0.0f;
// static float base_step    = 1.0f;   // 1.0 = normal speed
// static float pitch_mult   = 1.0f;   // from buttons

// struct repeating_timer audio_timer;
// volatile bool playing = false;

// volatile bool update_meter = false;
// uint32_t meter_tick = 0;

// // -----------------------------
// // BUTTON HANDLING
// // -----------------------------
// void check_buttons() {
//     uint32_t now = time_us_32();
//     uint32_t all_pins = gpio_get_all();

//     for (int i = 0; i < NUM_BUTTONS; i++) {
//         if (button_pending[i]) {
//             if (now - button_pending_time[i] >= 1000) { // 1 ms debounce
//                 bool is_pressed_now = (all_pins & (1 << BUTTON_PINS[i]));
//                 bool last_state = (button_state & (1 << i));

//                 if (is_pressed_now != last_state) {
//                     if (now - last_action_time[i] >= 10000) { // 10 ms lockout
//                         if (is_pressed_now) {
//                             button_state   |=  (1 << i);
//                             button_pressed |=  (1 << i);
//                         } else {
//                             button_state   &= ~(1 << i);
//                             button_released |= (1 << i);
//                         }
//                         last_action_time[i] = now;
//                     }
//                 }
//                 button_pending[i] = false;
//             }
//         }
//     }
// }

// void button_callback(uint gpio, uint32_t events) {
//     int i = gpio - 2;
//     if (i >= 0 && i < NUM_BUTTONS) {
//         button_pending[i] = true;
//         button_pending_time[i] = time_us_32();
//     }
// }

// void init_buttons() {
//     for (int i = 0; i < NUM_BUTTONS; i++) {
//         gpio_init(BUTTON_PINS[i]);
//         gpio_set_dir(BUTTON_PINS[i], GPIO_IN);
//         // buttons are pulled down in hardware
//         gpio_set_irq_enabled_with_callback(
//             BUTTON_PINS[i],
//             GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
//             true,
//             &button_callback
//         );
//     }
// }

// // -----------------------------
// // DAC / VOLUME SCALING
// // -----------------------------
// void dac_write_scaled(uint16_t sample) {
//     float scale = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
//     if (scale > 1.0f) scale = 1.0f;
//     if (scale < 0.0f) scale = 0.0f;
//     scale = sqrtf(scale);

//     int32_t signed_sample = (int32_t)sample - 0x8000;
//     int32_t scaled = (int32_t)(signed_sample * scale);
//     uint16_t output = (uint16_t)(scaled + 0x8000);

//     uint8_t tx[3] = {CMD_WRITE_UPDATE, (uint8_t)(output >> 8), (uint8_t)(output & 0xFF)};
//     i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
// }

// // -----------------------------
// // PITCH: BUTTONS → MULTIPLIER
// // -----------------------------
// static int semitones_from_valves(uint8_t state) {
//     // bits 0,1,2 = valves 1,2,3
//     uint8_t valves = (state >> 4) & 0x0F;
//     switch (valves) { // buttons are 4 (not 3 buttons valves) // strating with a frequency and multiplying up from there
//         case 0b0000: return -1;  // Bb
//         case 0b0111: return  0;  // B
//         case 0b0101: return  1;  // C
//         case 0b0110: return  2;  // Db
//         case 0b0011: return  3;  // D
//         case 0b0001: return  4;  // Eb
//         case 0b0010: return  5;  // E
//         case 0b1000: return  6;  // F
//         case 0b1110: return  7;  // Gb
//         case 0b1011: return  8;  // G
//         case 0b1001: return  9;  // Ab
//         case 0b1010: return 10;  // A
//         default:     return -1;  // Bb fallback
//     }
// }

// // static void update_pitch_from_buttons(void) {
// //     uint8_t state = button_state;

// //     int semitones = semitones_from_valves(state);

// //     int partial = (state & (1 << 3)) ? 1 : 0;
// //     int octave_up = (state & (1 << 4)) ? 1 : 0;
// //     int octave_down = (state & (1 << 5)) ? 1 : 0;

// //     if (partial) {
// //         semitones += 7; // example: perfect fifth
// //     }

// //     int octave = octave_up - octave_down;
// //     semitones += octave * 12;

// //     pitch_mult = pitch_multiplier_from_semitones(semitones);
// // }
// static void update_pitch_from_buttons(void) {
//     uint8_t state = button_state;

//     int semitones = semitones_from_valves(state);

//     // uint8_t octaves = state & 0x0F;  // lower 4 bits
//     // int octave_count = 
//     //     ((octaves & 0b0001) ? 1 : 0) +
//     //     ((octaves & 0b0010) ? 1 : 0) +
//     //     ((octaves & 0b0100) ? 1 : 0) +
//     //     ((octaves & 0b1000) ? 1 : 0);

//     // semitones += octave_count * 12;

//     int octave_shift = 0;

//     if (state & (1 << 0)) octave_shift = 12;   // GP2
//     if (state & (1 << 1)) octave_shift = 24;   // GP3
//     if (state & (1 << 2)) octave_shift = 36;   // GP4
//     if (state & (1 << 3)) octave_shift = 48;   // GP5

//     semitones += octave_shift;

//     pitch_mult = pitch_multiplier_from_semitones(semitones);
// }

// // -----------------------------
// // STREAM BUFFER REFILL USING sd_audio
// // -----------------------------
// static bool refill_stream_buffer(void) {
//      // Fill the non-playing buffer
//     int fill_buf = (play_buf == 0) ? 1 : 0;

//     UINT bytes_read = sd_audio_read(
//         (uint8_t*)audio_buf[fill_buf],
//         STREAM_BUFFER_SAMPLES * sizeof(int16_t)
//     );

//     int samples = bytes_read / sizeof(int16_t);
//     if (samples > 1) {
//         // Publish as ready
//         ready_buf   = fill_buf;
//         play_samples = samples;
//         buf_ready   = true;
//         return true;
//     }

//     return false;
// }

// // -----------------------------
// // TIMER CALLBACK: AUDIO OUTPUT
// // -----------------------------
// bool timer_callback(struct repeating_timer *t) {
//     if (++meter_tick >= 100) {
//         meter_tick = 0;
//         update_meter = true;
//     }

//     // Breath off → silence
//     if (adc_hw->result < THRESHOLD_OFF) {
//         playing = false;
//         uint8_t tx[3] = {CMD_WRITE_UPDATE, 0x80, 0x00};
//         i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
//         return true;
//     }

//     if (!playing) return true;

//     // If a new buffer is ready, swap to it
//     if (buf_ready) {
//         play_buf    = ready_buf;
//         sample_index = 0.0f;
//         buf_ready   = false;
//     }

//     if (play_samples < 2) {
//         // Nothing valid to play
//         return true;
//     }

//     int i0 = (int)sample_index;
//     int i1 = i0 + 1;
//     if (i1 >= play_samples) {
//         i0 = 0;
//         i1 = 1;
//         sample_index = 0.0f;
//     }

//     float frac = sample_index - (float)i0;
//     int16_t s0 = audio_buf[play_buf][i0];
//     int16_t s1 = audio_buf[play_buf][i1];
//     float interp = s0 * (1.0f - frac) + s1 * frac;
//     int16_t signed_out = (int16_t)interp;
//     uint16_t out = (uint16_t)(signed_out + 0x8000); // signed → unsigned

//     dac_write_scaled(out);

//     sample_index += base_step * pitch_mult;
//     if (sample_index >= (float)(play_samples - 1)) {
//         sample_index -= (float)(play_samples - 1);
//     }

//     if (update_meter) {
//         update_meter = false;
//         meter_out();
//     }

//     return true;
// }

// // -----------------------------
// // SD / I2C / ADC / METER INIT
// // -----------------------------
// void sdcard_io_high_speed(void) {
//     spi_set_baudrate(spi0, 12 * 1000 * 1000);
// }

// void init_ad5693() {
//     i2c_init(I2C_PORT, 1000 * 1000);
//     gpio_set_function(14, GPIO_FUNC_I2C);
//     gpio_set_function(15, GPIO_FUNC_I2C);
//     sleep_ms(10);

//     uint8_t reset_cmd[3] = {0x60, 0x00, 0x00};
//     i2c_write_blocking(I2C_PORT, AD5693_ADDR, reset_cmd, 3, false);
//     sleep_ms(10);

//     uint8_t config[3] = {0x40, 0x00, 0x00};
//     i2c_write_blocking(I2C_PORT, AD5693_ADDR, config, 3, false);
//     sleep_ms(10);
// }

// void ad5693_write(uint16_t sample) {
//     uint8_t buf[3];
//     buf[0] = CMD_WRITE_UPDATE;
//     buf[1] = (uint8_t)(sample >> 8);
//     buf[2] = (uint8_t)(sample & 0xFF);
//     i2c_write_blocking(I2C_PORT, AD5693_ADDR, buf, 3, false);
// }

// void init_spi_sdcard() {
//     gpio_init(16); gpio_init(17); gpio_init(18); gpio_init(19);

//     gpio_set_dir(17, GPIO_OUT);
//     gpio_put(17, 1);

//     spi_init(spi0, 100 * 1000);
//     gpio_set_function(16, GPIO_FUNC_SPI);
//     gpio_set_function(18, GPIO_FUNC_SPI);
//     gpio_set_function(19, GPIO_FUNC_SPI);

//     spi_set_baudrate(spi0, 20 * 1000 * 1000);

//     printf("SPI Pins Configured.\n");
// }

// void disable_sdcard() {
//     gpio_put(17, true); // CS high
// }

// void enable_sdcard() {
//     gpio_put(17, 0); // CS low
// }

// void init_sdcard_io() {
//     init_spi_sdcard();
// }

// void init_mouthpiece() {
//     adc_init();
//     adc_gpio_init(45);
//     adc_select_input(5);
//     adc_run(1);
// }

// void meter_init() {
//     const uint MAX = 4095;

//     gpio_init(32);
//     gpio_init(33);
//     gpio_init(34);
//     gpio_init(35);

//     gpio_set_function(32, GPIO_FUNC_PWM);
//     gpio_set_function(33, GPIO_FUNC_PWM);
//     gpio_set_function(34, GPIO_FUNC_PWM);
//     gpio_set_function(35, GPIO_FUNC_SIO);

//     gpio_set_dir(32, true);
//     gpio_set_dir(33, true);
//     gpio_set_dir(34, true);
//     gpio_set_dir(35, true);

//     uint slice1 = pwm_gpio_to_slice_num(32);
//     uint slice2 = pwm_gpio_to_slice_num(33);
//     uint slice3 = pwm_gpio_to_slice_num(34);
//     pwm_config config = pwm_get_default_config();
//     pwm_config_set_wrap(&config, MAX);
//     pwm_init(slice1, &config, true);
//     pwm_init(slice2, &config, true);
//     pwm_init(slice3, &config, true);
// }

// void meter_out() {
//     const uint led_pins[3] = {32, 33, 34};
//     const uint MAX = 4095;
//     float level;

//     if (adc_hw->result >= 4095)      level = 1.0f;
//     else if (adc_hw->result <= 0)    level = 0.0f;
//     else                             level = (float)(adc_hw->result) / 4095.0f;

//     float active_led = level * 3.0f;

//     for (int i = 0; i < 3; i++) {
//         float brightness = 0.0f;

//         if (active_led >= (float)(i + 1)) {
//             brightness = 1.0f;
//         } else if (active_led <= 0.15f) {
//             brightness = 0.0f;
//         } else if (active_led > (float)i) {
//             brightness = active_led - (float)i;
//         } else {
//             brightness = 0.0f;
//         }

//         pwm_set_gpio_level(led_pins[i], (uint16_t)(brightness * MAX));
//     }

//     if (adc_hw->result == 4095) gpio_put(35, true);
//     else                        gpio_put(35, false);
// }

// // -----------------------------
// // FAT TIME (for FatFs)
// // -----------------------------
// DWORD get_fattime (void) {
//     return ((DWORD)(2026 - 1980) << 25) |
//            ((DWORD)1 << 21)             |
//            ((DWORD)1 << 16)             |
//            ((DWORD)0 << 11)             |
//            ((DWORD)0 << 5)              |
//            ((DWORD)0 >> 1);
// }

// // -----------------------------
// // MAIN
// // -----------------------------
// int main() {
//     stdio_init_all();
//     sleep_ms(500);

//     init_mouthpiece();
//     meter_init();
//     init_buttons();
//     init_ad5693();
//     init_sdcard_io();

//     if (!sd_audio_init()) {
//         printf("sd_audio_init failed\n");
//         while (1) tight_loop_contents();
//     }

//     if (!sd_audio_open("sine.wav")) {
//         printf("Failed to open sine.wav\n");
//         while (1) tight_loop_contents();
//     }

//     wav_info_t info = sd_audio_get_info();
//     printf("WAV: %lu Hz, %u bits\n",
//            (unsigned long)info.sample_rate,
//            (unsigned)info.bits_per_sample);

//     if (!refill_stream_buffer()) {
//         printf("Failed to fill stream buffer\n");
//         while (1) tight_loop_contents();
//     }

//     int64_t period_us = - (int64_t)(1000000.0f / (float)info.sample_rate);
//     add_repeating_timer_us(period_us, timer_callback, NULL, &audio_timer);

//     playing = true;

//     while (1) {
//         check_buttons();
//         update_pitch_from_buttons();

//         if (!buf_ready) {
//             refill_stream_buffer();
//         }

//         for (int i = 0; i < NUM_BUTTONS; i++) {
//             if (button_pressed & (1 << i)) {
//                 button_pressed &= ~(1 << i);
//                 printf("Button %d pressed\n", i);
//             }
//             if (button_released & (1 << i)) {
//                 button_released &= ~(1 << i);
//                 printf("Button %d released\n", i);
//             }
//         }

//         if (adc_hw->result > THRESHOLD_ON && !playing) {
//             playing = true;
//         } else if (adc_hw->result < THRESHOLD_OFF && playing) {
//             playing = false;
//         }

//         tight_loop_contents();
//     }

//     return 0;
// }