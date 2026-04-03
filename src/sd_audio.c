#include "sd_audio.h"
#include "ff.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define AUDIO_BUFFER_SIZE 512  // bytes
//b flat blow in 
//next note up is a BYTE
// right 4 buttons is valves
// left right buttons are for the octaves
//  og frequency, multipkly once based on what valves are pressed
// gpio_get
// based on which ones are high, youmultiply it by a certain amount and you take that prodct and mutliply it by which octaves is pressed 
// opening note should be the lowest it should go, we should not go lower than opening note
// every octave button should bring it up another octave, not lower(*2,*3,*4,*5 from the original button)

// -----------------------------
// Internal state
// -----------------------------
static FATFS fs;
static FIL file;
static wav_info_t wav_info;

static uint8_t audio_buffer[AUDIO_BUFFER_SIZE];
static UINT buffer_bytes = 0;
static UINT buffer_pos = 0;

// Forward declaration
static bool refill_buffer(void);

// -----------------------------
// SD + FATFS INITIALIZATION
// -----------------------------
bool sd_audio_init(void) {
    FRESULT fr = f_mount(&fs, "", 1); // before you can read files, the system needs to connect to and recognize the SD card filesystem
    return (fr == FR_OK);
}

// -----------------------------
// OPEN WAV FILE + PARSE HEADER
// -----------------------------
bool sd_audio_open(const char *filename) {
    FRESULT fr = f_open(&file, filename, FA_READ); // handles the file and can start reading from it
    if (fr != FR_OK) return false; // reminder: FR_OK = 0,				/* (0) Succeeded */

    uint8_t header[44]; // making a buffer to store the WAV header. array of 44 bytes
    UINT br; // variable that stores how many bytes were actually read

    // Read standard 44-byte WAV header
    fr = f_read(&file, header, 44, &br);
    if (fr != FR_OK || br != 44) return false;

    // Parse WAV header
    wav_info.sample_rate     = *(uint32_t*)&header[24]; // extracting sample rate from header
    wav_info.bits_per_sample = *(uint16_t*)&header[34]; // extracting bits per sample
    wav_info.data_start      = 44;
    wav_info.data_size       = *(uint32_t*)&header[40]; // extracting data size

    // moves file pointer to start of the audio data
    f_lseek(&file, wav_info.data_start);

    // reset streaming(internal) buffer
    buffer_bytes = 0;
    buffer_pos   = 0;

    return refill_buffer(); // called to load the first 512 bytes
}

// -----------------------------
// REFILL INTERNAL BUFFER
// -----------------------------
static bool refill_buffer(void) {
    FRESULT fr;
    UINT br;

    fr = f_read(&file, audio_buffer, AUDIO_BUFFER_SIZE, &br); // reads 512 bytes from the SD card into audio_buffer
    if (fr != FR_OK) return false;

    // loop the audio if the end is reached
    if (br == 0) {
        f_lseek(&file, wav_info.data_start);
        fr = f_read(&file, audio_buffer, AUDIO_BUFFER_SIZE, &br);
        if (fr != FR_OK) return false;
    }

    buffer_bytes = br;
    buffer_pos   = 0;
    return (buffer_bytes > 0);
}

// -----------------------------
// PUBLIC READ FUNCTION
// -----------------------------
// gives the next chunk of audio samples
UINT sd_audio_read(uint8_t *buffer, UINT bytes_to_read) {
    UINT total_copied = 0;

    while (total_copied < bytes_to_read) {
        if (buffer_pos >= buffer_bytes) {
            if (!refill_buffer()) break;
        }

        UINT to_copy = buffer_bytes - buffer_pos;
        if (to_copy > (bytes_to_read - total_copied)) {
            to_copy = bytes_to_read - total_copied;
        }

        memcpy(buffer + total_copied, audio_buffer + buffer_pos, to_copy);
        buffer_pos   += to_copy;
        total_copied += to_copy;
    }

    return total_copied;
}

// -----------------------------
// CLOSE FILE
// -----------------------------
void sd_audio_close(void) {
    f_close(&file);
}

// -----------------------------
// GET WAV METADATA
// -----------------------------
//Returns the parsed WAV metadata so your EWI knows: sample rate, bits per sample, and data size
wav_info_t sd_audio_get_info(void) {
    return wav_info;
}

// -----------------------------
// PITCHING HELPERS
// -----------------------------
float pitch_multiplier_from_semitones(int semitones) {
    return powf(2.0f, semitones / 12.0f); // this converts semitone offsets into playback speed multipliers
}
