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
#define AD5693_ADDR     0x4C // Default address (check your ADDR pin)
#define CMD_WRITE_UPDATE 0x30
#define THRESHOLD_ON   250
#define THRESHOLD_OFF  250
#define ADC_OFFSET     250
#define ADC_MAX        4095
#define NUM_BUTTONS 8
const uint BUTTON_PINS[NUM_BUTTONS] = {2, 3, 4, 5, 6, 7, 8, 9};

// --- BUTTON STATE ---
volatile uint8_t button_state    = 0;  // bitmask of current state
volatile uint8_t button_pressed  = 0;  // newly pressed
volatile uint8_t button_released = 0;  // newly released

// LED meter pins
#define LED_PIN_1 32
#define LED_PIN_2 33
#define LED_PIN_3 34
#define LED_PIN_MAX 35

// SD SPI pins
#define SD_SPI_PORT spi0
#define SD_PIN_MISO 16
#define SD_PIN_CS   17
#define SD_PIN_SCK  18
#define SD_PIN_MOSI 19

volatile uint32_t button_pending_time[NUM_BUTTONS] = {0};
volatile bool button_pending[NUM_BUTTONS] = {false};
uint32_t last_action_time[NUM_BUTTONS] = {0};

// --- AUDIO / PITCH STATE ---
#define STREAM_BUFFER_SAMPLES 256

static int16_t audio_buf[2][STREAM_BUFFER_SAMPLES];
static volatile int play_buf = 0;
static volatile int ready_buf = 1;
static volatile int play_samples = 0;
static volatile bool buf_ready = false;

static float sample_index = 0.0f;
static float base_step    = 1.0f;   // 1.0 = normal speed
static float pitch_mult   = 1.0f;   // from buttons

struct repeating_timer audio_timer;
volatile bool playing = false;

volatile bool update_meter = false;
uint32_t meter_tick = 0;

// -----------------------------
// BUTTON HANDLING
// -----------------------------
void check_buttons() {
    uint32_t now = time_us_32();
    uint32_t all_pins = gpio_get_all();

    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (button_pending[i]) {
            if (now - button_pending_time[i] >= 1000) { // 1 ms debounce
                bool is_pressed_now = (all_pins & (1 << BUTTON_PINS[i]));
                bool last_state = (button_state & (1 << i));

                if (is_pressed_now != last_state) {
                    if (now - last_action_time[i] >= 10000) { // 10 ms lockout
                        if (is_pressed_now) {
                            button_state   |=  (1 << i);
                            button_pressed |=  (1 << i);
                        } else {
                            button_state   &= ~(1 << i);
                            button_released |= (1 << i);
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
        button_pending[i] = true;
        button_pending_time[i] = time_us_32();
    }
}

void init_buttons() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        gpio_init(BUTTON_PINS[i]);
        gpio_set_dir(BUTTON_PINS[i], GPIO_IN);
        // buttons are pulled down in hardware
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
    float scale = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;
    scale = sqrtf(scale);

    int32_t signed_sample = (int32_t)sample - 0x8000;
    int32_t scaled = (int32_t)(signed_sample * scale);
    uint16_t output = (uint16_t)(scaled + 0x8000);

    uint8_t tx[3] = {CMD_WRITE_UPDATE, (uint8_t)(output >> 8), (uint8_t)(output & 0xFF)};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
}

// -----------------------------
// PITCH: BUTTONS → MULTIPLIER
// -----------------------------
static int semitones_from_valves(uint8_t state) {
    // bits 0,1,2 = valves 1,2,3
    uint8_t valves = (state >> 4) & 0x0F;
    switch (valves) { // buttons are 4 (not 3 buttons valves) // strating with a frequency and multiplying up from there
        case 0b0000: return 2;   // base note (raised)
        case 0b0001: return 4;
        case 0b0010: return 5;
        case 0b0011: return 7;
        case 0b0100: return 9;
        case 0b0101: return 11;
        case 0b0110: return 12;
        case 0b0111: return 14;
        case 0b1000: return 16;
        case 0b1001: return 17;
        case 0b1010: return 19;
        case 0b1011: return 21;
        case 0b1100: return 23;
        case 0b1101: return 24;
        case 0b1110: return 26;
        case 0b1111: return 28;   // instead of 26, keeps scale rising
        default:    return 2;
    }
}

// static void update_pitch_from_buttons(void) {
//     uint8_t state = button_state;

//     int semitones = semitones_from_valves(state);

//     int partial = (state & (1 << 3)) ? 1 : 0;
//     int octave_up = (state & (1 << 4)) ? 1 : 0;
//     int octave_down = (state & (1 << 5)) ? 1 : 0;

//     if (partial) {
//         semitones += 7; // example: perfect fifth
//     }

//     int octave = octave_up - octave_down;
//     semitones += octave * 12;

//     pitch_mult = pitch_multiplier_from_semitones(semitones);
// }
static void update_pitch_from_buttons(void) {
    uint8_t state = button_state;

    int semitones = semitones_from_valves(state);

    // uint8_t octaves = state & 0x0F;  // lower 4 bits
    // int octave_count = 
    //     ((octaves & 0b0001) ? 1 : 0) +
    //     ((octaves & 0b0010) ? 1 : 0) +
    //     ((octaves & 0b0100) ? 1 : 0) +
    //     ((octaves & 0b1000) ? 1 : 0);

    // semitones += octave_count * 12;

    int octave_shift = 0;

    if (state & (1 << 0)) octave_shift = 12;   // GP2
    if (state & (1 << 1)) octave_shift = 24;   // GP3
    if (state & (1 << 2)) octave_shift = 36;   // GP4
    if (state & (1 << 3)) octave_shift = 48;   // GP5

    semitones += octave_shift;

    pitch_mult = pitch_multiplier_from_semitones(semitones);
}

// -----------------------------
// STREAM BUFFER REFILL USING sd_audio
// -----------------------------
static bool refill_stream_buffer(void) {
     // Fill the non-playing buffer
    int fill_buf = (play_buf == 0) ? 1 : 0;

    UINT bytes_read = sd_audio_read(
        (uint8_t*)audio_buf[fill_buf],
        STREAM_BUFFER_SAMPLES * sizeof(int16_t)
    );

    int samples = bytes_read / sizeof(int16_t);
    if (samples > 1) {
        // Publish as ready
        ready_buf   = fill_buf;
        play_samples = samples;
        buf_ready   = true;
        return true;
    }

    return false;
}

// -----------------------------
// TIMER CALLBACK: AUDIO OUTPUT
// -----------------------------
bool timer_callback(struct repeating_timer *t) {
    if (++meter_tick >= 100) {
        meter_tick = 0;
        update_meter = true;
    }

    // Breath off → silence
    if (adc_hw->result < THRESHOLD_OFF) {
        playing = false;
        uint8_t tx[3] = {CMD_WRITE_UPDATE, 0x80, 0x00};
        i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
        return true;
    }

    if (!playing) return true;

    // If a new buffer is ready, swap to it
    if (buf_ready) {
        play_buf    = ready_buf;
        sample_index = 0.0f;
        buf_ready   = false;
    }

    if (play_samples < 2) {
        // Nothing valid to play
        return true;
    }

    int i0 = (int)sample_index;
    int i1 = i0 + 1;
    if (i1 >= play_samples) {
        i0 = 0;
        i1 = 1;
        sample_index = 0.0f;
    }

    float frac = sample_index - (float)i0;
    int16_t s0 = audio_buf[play_buf][i0];
    int16_t s1 = audio_buf[play_buf][i1];
    float interp = s0 * (1.0f - frac) + s1 * frac;
    int16_t signed_out = (int16_t)interp;
    uint16_t out = (uint16_t)(signed_out + 0x8000); // signed → unsigned

    dac_write_scaled(out);

    sample_index += base_step * pitch_mult;
    if (sample_index >= (float)(play_samples - 1)) {
        sample_index -= (float)(play_samples - 1);
    }

    if (update_meter) {
        update_meter = false;
        meter_out();
    }

    return true;
}

// -----------------------------
// SD / I2C / ADC / METER INIT
// -----------------------------
void sdcard_io_high_speed(void) {
    spi_set_baudrate(spi0, 12 * 1000 * 1000);
}

void init_ad5693() {
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

void init_spi_sdcard() {
    gpio_init(16); gpio_init(17); gpio_init(18); gpio_init(19);

    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);

    spi_init(spi0, 100 * 1000);
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);

    spi_set_baudrate(spi0, 20 * 1000 * 1000);

    printf("SPI Pins Configured.\n");
}

void disable_sdcard() {
    gpio_put(17, true); // CS high
}

void enable_sdcard() {
    gpio_put(17, 0); // CS low
}

void init_sdcard_io() {
    init_spi_sdcard();
}

void init_mouthpiece() {
    adc_init();
    adc_gpio_init(45);
    adc_select_input(5);
    adc_run(1);
}

void meter_init() {
    const uint MAX = 4095;

    gpio_init(32);
    gpio_init(33);
    gpio_init(34);
    gpio_init(35);

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

void meter_out() {
    const uint led_pins[3] = {32, 33, 34};
    const uint MAX = 4095;
    float level;

    if (adc_hw->result >= 4095)      level = 1.0f;
    else if (adc_hw->result <= 0)    level = 0.0f;
    else                             level = (float)(adc_hw->result) / 4095.0f;

    float active_led = level * 3.0f;

    for (int i = 0; i < 3; i++) {
        float brightness = 0.0f;

        if (active_led >= (float)(i + 1)) {
            brightness = 1.0f;
        } else if (active_led <= 0.15f) {
            brightness = 0.0f;
        } else if (active_led > (float)i) {
            brightness = active_led - (float)i;
        } else {
            brightness = 0.0f;
        }

        pwm_set_gpio_level(led_pins[i], (uint16_t)(brightness * MAX));
    }

    if (adc_hw->result == 4095) gpio_put(35, true);
    else                        gpio_put(35, false);
}

// -----------------------------
// FAT TIME (for FatFs)
// -----------------------------
DWORD get_fattime (void) {
    return ((DWORD)(2026 - 1980) << 25) |
           ((DWORD)1 << 21)             |
           ((DWORD)1 << 16)             |
           ((DWORD)0 << 11)             |
           ((DWORD)0 << 5)              |
           ((DWORD)0 >> 1);
}

// -----------------------------
// MAIN
// -----------------------------
int main() {
    stdio_init_all();
    sleep_ms(500);

    init_mouthpiece();
    meter_init();
    init_buttons();
    init_ad5693();
    init_sdcard_io();

    if (!sd_audio_init()) {
        printf("sd_audio_init failed\n");
        while (1) tight_loop_contents();
    }

    if (!sd_audio_open("sine.wav")) {
        printf("Failed to open sine.wav\n");
        while (1) tight_loop_contents();
    }

    wav_info_t info = sd_audio_get_info();
    printf("WAV: %lu Hz, %u bits\n",
           (unsigned long)info.sample_rate,
           (unsigned)info.bits_per_sample);

    if (!refill_stream_buffer()) {
        printf("Failed to fill stream buffer\n");
        while (1) tight_loop_contents();
    }

    int64_t period_us = - (int64_t)(1000000.0f / (float)info.sample_rate);
    add_repeating_timer_us(period_us, timer_callback, NULL, &audio_timer);

    playing = true;

    while (1) {
        check_buttons();
        update_pitch_from_buttons();

        if (!buf_ready) {
            refill_stream_buffer();
        }

        for (int i = 0; i < NUM_BUTTONS; i++) {
            if (button_pressed & (1 << i)) {
                button_pressed &= ~(1 << i);
                printf("Button %d pressed\n", i);
            }
            if (button_released & (1 << i)) {
                button_released &= ~(1 << i);
                printf("Button %d released\n", i);
            }
        }

        if (adc_hw->result > THRESHOLD_ON && !playing) {
            playing = true;
        } else if (adc_hw->result < THRESHOLD_OFF && playing) {
            playing = false;
        }

        tight_loop_contents();
    }

    return 0;
}
