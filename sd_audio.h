#ifndef SD_AUDIO_H
#define SD_AUDIO_H

#include "ff.h"
#include <stdint.h>
#include <stdbool.h>

// WAV metadata structure
typedef struct {
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint32_t data_start;
    uint32_t data_size;
} wav_info_t;

// Public API
bool sd_audio_init(void);
bool sd_audio_open(const char *filename);
UINT sd_audio_read(uint8_t *buffer, UINT bytes_to_read);
void sd_audio_close(void);

// WAV metadata getter
wav_info_t sd_audio_get_info(void);

// Pitching helpers
float pitch_multiplier_from_semitones(int semitones);
float pitch_multiplier_from_buttons(int valve_state, int partial, int octave);

#endif
