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

void init_spi_sdcard(void);
void enable_sdcard(void);
void disable_sdcard(void);
void init_sdcard_io(void);
void ad5693_write(uint16_t sample);
void meter_out(void);
void dac_write_scaled(uint16_t sample);
void play_wav_timer(const char *filename);
void button_callback(uint gpio, uint32_t events);


// --- CONFIGURATION ---
#define I2C_PORT i2c1
#define AD5693_ADDR 0x4C // Default address (check your ADDR pin)
#define BUF_SIZE 1024    // Larger buffer for 16-bit data
#define CMD_WRITE_UPDATE 0x30
#define THRESHOLD_ON  250
#define THRESHOLD_OFF 250
#define ADC_OFFSET 250
#define ADC_MAX    4095
#define NUM_BUTTONS 8
const uint BUTTON_PINS[NUM_BUTTONS] = {2, 3, 4, 5, 6, 7, 8, 9};

volatile uint8_t button_state = 0;      // bitmask of current state
volatile uint8_t button_pressed = 0;    // bitmask of newly pressed (cleared after read)
volatile uint8_t button_released = 0; 

// AD5693 Commands
#define AUDIO_BUF_SIZE 2048
uint16_t audio_buffer[AUDIO_BUF_SIZE];
volatile int read_ptr = 0;
volatile int write_ptr = 0;
volatile bool playing = false;
struct repeating_timer audio_timer;
volatile bool sample_ready = false;  // <-- ADD
volatile uint16_t next_sample = 0;   // <-- ADD
volatile bool update_meter = false;
uint32_t meter_tick = 0;
volatile uint32_t button_pending_time[NUM_BUTTONS] = {0};
volatile bool button_pending[NUM_BUTTONS] = {false};

uint32_t last_action_time[NUM_BUTTONS] = {0};

void check_buttons() {
    uint32_t now = time_us_32();
    uint32_t all_pins = gpio_get_all();

    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (button_pending[i]) {
            // 1. Wait for debounce (1ms)
            if (now - button_pending_time[i] >= 1000) {
                
                // 2. Determine physical state (Assuming Hardware Pull-Down)
                // Pin is HIGH (1) when pressed, LOW (0) when idle
                bool is_pressed_now = (all_pins & (1 << BUTTON_PINS[i]));

                // 3. ONLY proceed if the state has actually changed
                bool last_state = (button_state & (1 << i));
                
                if (is_pressed_now != last_state) {
                    // 4. NOW check lockout. 
                    // We don't clear button_pending until we actually process or reject based on time.
                    if (now - last_action_time[i] >= 100000) {
                        if (is_pressed_now) {
                            button_state   |= (1 << i);
                            button_pressed |= (1 << i);
                        } else {
                            button_state    &= ~(1 << i);
                            button_released |= (1 << i);
                        }
                        last_action_time[i] = now;
                    }
                }
                
                // Always clear the flag after the 1ms debounce window
                // so the interrupt can set it again.
                button_pending[i] = false;
            }
        }
    }
}

void button_callback(uint gpio, uint32_t events) {
    // Instead of a loop, if your pins are sequential 2-9:
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
        //buttons are pulled down in hardware
        //gpio_pull_up(BUTTON_PINS[i]);  // buttons should connect pin to GND when pressed
        gpio_set_irq_enabled_with_callback(
            BUTTON_PINS[i],
            GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
            true,
            &button_callback
        );
    }
}

void dac_write_scaled(uint16_t sample) {
    float scale = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;
    scale = sqrtf(scale);

    int32_t signed_sample = (int32_t)sample - 0x8000;
    int32_t scaled = (int32_t)(signed_sample * scale);
    uint16_t output = (uint16_t)(scaled + 0x8000);

    uint8_t tx[3] = {0x30, (uint8_t)(output >> 8), (uint8_t)(output & 0xFF)};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
}

// This function runs exactly every 62us (for 16kHz)
bool timer_callback(struct repeating_timer *t) {
    if (++meter_tick >= 100) {
        meter_tick = 0;
        update_meter = true;
    }    
    
    if (adc_hw->result < THRESHOLD_OFF) {
        playing = false;
        sample_ready = false;
        // Write silence directly from ISR — acceptable since it's a single write
        uint8_t tx[3] = {0x30, 0x80, 0x00};
        i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
        return true;
    }

    if (playing && read_ptr != write_ptr) {
        next_sample = audio_buffer[read_ptr];
        read_ptr = (read_ptr + 1) % AUDIO_BUF_SIZE;
        sample_ready = true;
    }
    return true;
}

void play_wav_from_file(FIL *fil, uint16_t bytes_per_frame, uint32_t timer_us) {
    uint8_t raw_bytes[512];
    UINT br;

    read_ptr = 0;
    write_ptr = 0;
    playing = false;
    sample_ready = false;

    // Pre-fill buffer
    while (f_read(fil, raw_bytes, 512, &br) == FR_OK && br > 0) {
        for (uint i = 0; i + bytes_per_frame <= br; i += bytes_per_frame) {
            uint16_t dac_val = ((uint16_t)raw_bytes[i] | ((uint16_t)raw_bytes[i+1] << 8)) ^ 0x8000;
            audio_buffer[write_ptr] = dac_val;
            write_ptr = (write_ptr + 1) % AUDIO_BUF_SIZE;
        }
        int count = (write_ptr - read_ptr + AUDIO_BUF_SIZE) % AUDIO_BUF_SIZE;
        if (count >= AUDIO_BUF_SIZE / 2) break;
    }

    add_repeating_timer_us(timer_us, timer_callback, NULL, &audio_timer);
    playing = true;

    bool file_done = false;

    while (true) {
        // Timer callback sets playing=false when ADC drops — catch it here
        if (!playing) {
            cancel_repeating_timer(&audio_timer);
            read_ptr = 0;
            write_ptr = 0;
            return;
        }

        // Send sample to DAC
        if (sample_ready) {
            sample_ready = false;
            dac_write_scaled(next_sample);

            if (update_meter) {
            update_meter = false;
            meter_out();
}

            for (int i = 0; i < NUM_BUTTONS; i++) 
            {
                if (button_pressed & (1 << i)) {
                    button_pressed &= ~(1 << i);
                    printf("Button %d pressed\n", i);
                }
                if (button_released & (1 << i)) {
                    button_released &= ~(1 << i);
                    printf("Button %d released\n", i);
                }
            }

            float scale = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
            if (scale > 1.0f) scale = 1.0f;
            if (scale < 0.0f) scale = 0.0f;

            int32_t signed_sample = (int32_t)next_sample - 0x8000;
            int32_t scaled = (int32_t)(signed_sample * scale);
            uint16_t output = (uint16_t)(scaled + 0x8000);

            uint8_t tx[3] = {0x30, (uint8_t)(output >> 8), (uint8_t)(output & 0xFF)};
            i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
        }

        // Feed ring buffer from SD
        if (!file_done && ((write_ptr + 1) % AUDIO_BUF_SIZE) != read_ptr) {
            if (f_read(fil, raw_bytes, 512, &br) == FR_OK && br > 0) {
                for (uint i = 0; i + bytes_per_frame <= br; i += bytes_per_frame) {
                    uint16_t dac_val = ((uint16_t)raw_bytes[i] | ((uint16_t)raw_bytes[i+1] << 8)) ^ 0x8000;
                    while (((write_ptr + 1) % AUDIO_BUF_SIZE) == read_ptr) {
                            if (sample_ready) {
                                sample_ready = false;

                                float scale = (float)(adc_hw->result - ADC_OFFSET) / (float)(ADC_MAX - ADC_OFFSET);
                                if (scale > 1.0f) scale = 1.0f;
                                if (scale < 0.0f) scale = 0.0f;
                                int32_t signed_sample = (int32_t)next_sample - 0x8000;
                                int32_t scaled = (int32_t)(signed_sample * scale);
                                uint16_t output = (uint16_t)(scaled + 0x8000);

                                uint8_t tx[3] = {0x30, (uint8_t)(output >> 8), (uint8_t)(output & 0xFF)};
                                i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
                            }
                        // Check here too since this inner loop can spin a while
                        if (!playing) {
                            cancel_repeating_timer(&audio_timer);
                            read_ptr = 0;
                            write_ptr = 0;
                            return;
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
                    }
                    audio_buffer[write_ptr] = dac_val;
                    write_ptr = (write_ptr + 1) % AUDIO_BUF_SIZE;
                }
            } else {
                file_done = true;
            }
        }

        if (file_done && read_ptr == write_ptr) break;
    }

    // Natural end
    playing = false;
    cancel_repeating_timer(&audio_timer);
    uint8_t tx[3] = {0x30, 0x80, 0x00};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);
}

void stop_audio() {
    playing = false;
    read_ptr = 0;
    write_ptr = 0;
    sample_ready = false;
    // Write silence to DAC
    uint8_t tx[3] = {0x30, 0x80, 0x00};
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, tx, 3, false);

    // Clear all LEDs explicitly
    pwm_set_gpio_level(32, 0);
    pwm_set_gpio_level(33, 0);
    pwm_set_gpio_level(34, 0);
    gpio_put(35, false);
}

void sdcard_io_high_speed(void)
{
    uint actual_baud = spi_set_baudrate(spi0, 12 * 1000 * 1000);
}

void init_ad5693() 
{
    i2c_init(I2C_PORT, 1000 * 1000); 
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    sleep_ms(10);

    // SOFTWARE RESET COMMAND
    uint8_t reset_cmd[3] = {0x60, 0x00, 0x00}; 
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, reset_cmd, 3, false);
    sleep_ms(10);

    // Fixed Config: No stray 1s in reserved bits
    uint8_t config[3] = {0x40, 0x00, 0x00}; 
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, config, 3, false);
    sleep_ms(10);

}

void ad5693_write(uint16_t sample) {
    uint8_t buf[3];
    buf[0] = CMD_WRITE_UPDATE;
    buf[1] = (uint8_t)(sample >> 8);   // MSB
    buf[2] = (uint8_t)(sample & 0xFF); // LSB
    i2c_write_blocking(I2C_PORT, AD5693_ADDR, buf, 3, false);
}

void init_spi_sdcard()
{
// 1. Manually reset the pins to ensure no leftover states
    gpio_init(16); gpio_init(17); gpio_init(18); gpio_init(19);
    
    // 2. Set CS high immediately so the card isn't "listening" yet
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);

    // 3. Initialize SPI at a very slow speed
    spi_init(spi0, 100 * 1000); // 100kHz
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
    
    // 4. Strong Pull-up on MISO (Pin 16)
    //gpio_pull_up(16); 

    spi_set_baudrate(spi0, 20 * 1000 * 1000); // 20MHz
    
    printf("SPI Pins Configured.\n");
}

void disable_sdcard()
{
    gpio_put(17, true); //CS high

}

void enable_sdcard()
{
    gpio_put(17, 0); //CS low

}

void init_sdcard_io()
{
    init_spi_sdcard();
}

void init_mouthpiece()
{
    //adc config
    adc_init();
    adc_gpio_init(45);
    adc_select_input(5);
    adc_run(1);
}

void meter_init()
{
    const uint MAX = 4095; //scale to adc output
    
    //pin init
    gpio_init(32);
    gpio_init(33);
    gpio_init(34);
    gpio_init(35);

    //set func
    gpio_set_function(32, GPIO_FUNC_PWM);
    gpio_set_function(33, GPIO_FUNC_PWM);
    gpio_set_function(34, GPIO_FUNC_PWM);
    gpio_set_function(35, GPIO_FUNC_SIO);

    //dir
    gpio_set_dir(32, true);
    gpio_set_dir(33, true);
    gpio_set_dir(34, true);
    gpio_set_dir(35, true);

    //pwm control and slices
    uint slice1 = pwm_gpio_to_slice_num(32);
    uint slice2 = pwm_gpio_to_slice_num(33);
    uint slice3 = pwm_gpio_to_slice_num(34);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, MAX);
    pwm_init(slice1, &config, true);
    pwm_init(slice2, &config, true);
    pwm_init(slice3, &config, true);
}

void meter_out()
{
    const uint led_pins[3] = {32,33,34}; //leds
    const uint MAX = 4095; //scale to adc output
    float level;
    
    //scaling adc output from 0 - 1 float
    if (adc_hw->result >= 4095)
    {
        level = 1.0f;
    }
    else if (adc_hw->result <= 0)
    {
        level = 0.0f;
    }
    else
    {
        level = (float) (adc_hw->result)/4095;
    }

    float active_led = level * 3; //scale to number of LEDs
    
    //LED output
    for (int i = 0; i<3; i++)
    {
        float brightness = 0.0f; //init

        if(active_led >= (float)(i+1)) //fully light up
        {
            brightness = 1.0f;
        }
        else if (active_led <= 0.15f) //reference pressure offset
        {   
            brightness = 0.0f;
        }
        else if (active_led > (float)(i)) //partially light up
        {
            brightness = active_led - (float)(i);
        }
        else //off
        {
            brightness = 0.0f;
        }

        pwm_set_gpio_level(led_pins[i], (uint16_t)(brightness*MAX)); 
    }
    
    //max val output
    if (adc_hw->result == 4095)
    {
        gpio_put(35, true);
    }
    else
    {
        gpio_put(35, false);
    }
}

DWORD get_fattime (void) {
    return ((DWORD)(2026 - 1980) << 25) | // Year
           ((DWORD)1 << 21)              | // Month
           ((DWORD)1 << 16)              | // Day
           ((DWORD)0 << 11)              | // Hour
           ((DWORD)0 << 5)               | // Min
           ((DWORD)0 >> 1);                // Sec
}

void sd_op()
{
     // Force I2C to 1MHz immediately
    i2c_init(I2C_PORT, 1000 * 1000); 
    init_ad5693();
    
    // Force SPI to 25MHz (Fast as the SD card can go)
    init_spi_sdcard();
    spi_set_baudrate(spi0, 25 * 1000 * 1000); 

    FATFS fs;
    if (f_mount(&fs, "", 1) == FR_OK) {
        // Try the ultra_fast version first. 
        // If it's TOO FAST, your hardware is finally fixed.
        // If it's STILL SLOW, your I2C bus is physically capped by 
        // wire length or pull-up resistors.
        play_wav_timer("sine.wav"); 
    }
}

int main()
{
    stdio_init_all();
    init_mouthpiece();
    meter_init();
    init_buttons();

    // Init hardware ONCE
    i2c_init(I2C_PORT, 1000 * 1000);
    init_ad5693();
    init_spi_sdcard();
    spi_set_baudrate(spi0, 25 * 1000 * 1000);
    

    // Mount ONCE
    FATFS fs;
    if (f_mount(&fs, "", 1) != FR_OK) {
        printf("SD mount failed\n");
        while(1);
    }

    // Parse WAV header ONCE
    FIL fil;
    uint8_t header[44];
    UINT br;
    if (f_open(&fil, "sine.wav", FA_READ) != FR_OK) {
        printf("File open failed\n");
        while(1);
    }
    f_read(&fil, header, 44, &br);

    uint16_t num_channels   = (uint16_t)header[22] | ((uint16_t)header[23] << 8);
    uint32_t sample_rate    = (uint32_t)header[24] | ((uint32_t)header[25] << 8)
                            | ((uint32_t)header[26] << 16) | ((uint32_t)header[27] << 24);
    uint16_t bit_depth      = (uint16_t)header[34] | ((uint16_t)header[35] << 8);
    uint16_t bytes_per_frame = num_channels * (bit_depth / 8);
    uint32_t timer_us       = (uint32_t)((1000000.0f / (float)sample_rate) + 0.5f);

    uint32_t data_start = 44; // Standard WAV, adjust if your header differs

    for (;;) {
        check_buttons();
        
        if(!playing)
        { 
            meter_out();

            for (int i = 0; i < NUM_BUTTONS; i++) {
                if (button_pressed & (1 << i)) {
                button_pressed &= ~(1 << i); // clear flag after reading
                // handle press of button i
                printf("Button %d pressed\n", i);
            }
            if (button_released & (1 << i)) {
                button_released &= ~(1 << i);
                // handle release of button i
                printf("Button %d released\n", i);
            }

            if (button_state & (1 << 0)) {
            // button 0 is being held right now
            }

            if (adc_hw->result > THRESHOLD_ON && !playing) {
                f_lseek(&fil, data_start);
                play_wav_from_file(&fil, bytes_per_frame, timer_us);
            }
    }

        }
        
        if (adc_hw->result > THRESHOLD_ON && !playing) {
            // Rewind to start of audio data
            f_lseek(&fil, data_start);
            play_wav_from_file(&fil, bytes_per_frame, timer_us);
        }
    }
}

