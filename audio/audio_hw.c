/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "audio_hw_smdkv310"
#define LOG_NDEBUG 0

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include <cutils/log.h>

#include <hardware/hardware.h>
#include <system/audio.h>
#include <hardware/audio.h>
#include "hwdep.h"

/* MC1N2 Mixer Controls */
#define MIXER_MASTER_PLAYBACK_SWITCH            "Master Playback Switch"
#define MIXER_MASTER_PLAYBACK_VOLUME            "Master Playback Volume"
#define MIXER_HEADPHONE_PLAYBACK_SWITCH         "Headphone Playback Switch"
#define MIXER_HEADPHONE_PLAYBACK_VOLUME         "Headphone Playback Volume"
#define MIXER_LINE1_PLAYBACK_SWITCH             "Line 1 Playback Switch"
#define MIXER_LINE1_PLAYBACK_VOLUME             "Line 1 Playback Volume"
#define MIXER_LINE2_PLAYBACK_SWITCH             "Line 2 Playback Switch"
#define MIXER_LINE2_PLAYBACK_VOLUME             "Line 2 Playback Volume"
#define MIXER_LINE_BYPASS_PLAYBACK_SWITCH       "Line Bypass Playback Switch"
#define MIXER_LINE_BYPASS_PLAYBACK_VOLUME       "Line Bypass Playback Volume"
#define MIXER_MIC1_BYPASS_PLAYBACK_SWITCH       "Mic 1 Bypass Playback Switch"
#define MIXER_MIC1_BYPASS_PLAYBACK_VOLUME       "Mic 1 Bypass Playback Volume"
#define MIXER_MIC1_GAIN_VOLUME                  "Mic 1 Gain Volume"
#define MIXER_MIC2_BYPASS_PLAYBACK_SWITCH       "Mic 2 Bypass Playback Switch"
#define MIXER_MIC2_BYPASS_PLAYBACK_VOLUME       "Mic 2 Bypass Playback Volume"
#define MIXER_MIC2_GAIN_VOLUME                  "Mic 2 Gain Volume"
#define MIXER_MIC3_BYPASS_PLAYBACK_SWITCH       "Mic 3 Bypass Playback Switch"
#define MIXER_MIC3_BYPASS_PLAYBACK_VOLUME       "Mic 3 Bypass Playback Volume"
#define MIXER_MIC3_GAIN_VOLUME                  "Mic 3 Gain Volume"
#define MIXER_ADCL_PDM_SEL                      "ADC PDM SEL"
#define MIXER_ADCL_LINE_MIXMODE                 "ADCL LINE MIXMODE"
#define MIXER_ADCL_MIXER_LINE_SWITCH            "ADCL MIXER Line Switch"
#define MIXER_ADCL_MIXER_MIC1_SWITCH            "ADCL MIXER Mic1 Switch"
#define MIXER_ADCL_MIXER_MIC2_SWITCH            "ADCL MIXER Mic2 Switch"
#define MIXER_ADCL_MIXER_MIC3_SWITCH            "ADCL MIXER Mic3 Switch"
#define MIXER_ADCR_LINE_MIXMODE                 "ADCR LINE MIXMODE"
#define MIXER_ADCR_MIXER_LINE_SWITCH            "ADCR MIXER Line Switch"
#define MIXER_ADCR_MIXER_MIC1_SWITCH            "ADCR MIXER Mic1 Switch"
#define MIXER_ADCR_MIXER_MIC2_SWITCH            "ADCR MIXER Mic2 Switch"
#define MIXER_ADCR_MIXER_MIC3_SWITCH            "ADCR MIXER Mic3 Switch"
#define MIXER_AD_ATT_SWITCH                     "AD ATT Switch"
#define MIXER_AD_ATT_VOLUME                     "AD ATT Volume"
#define MIXER_AD_ANALOG_SWITCH                  "AD Analog Switch"
#define MIXER_AD_ANALOG_VOLUME                  "AD Analog Volume"
#define MIXER_AD_DIGITAL_SWITCH                 "AD Digital Switch"
#define MIXER_AD_DIGITAL_VOLUME                 "AD Digital Volume"
#define MIXER_AE_PARAMETER_SEL                  "AE PARAMETER SEL"
#define MIXER_AE_SRC                            "AE SRC"
#define MIXER_AENG6_SWITCH                      "AENG6 Switch"
#define MIXER_AENG6_VOLUME                      "AENG6 Volume"
#define MIXER_CODEC_STATUS                      "Codec Status"
#define MIXER_DAC_PLAYBACK_SWITCH               "DAC Playback Switch"
#define MIXER_DAC_PLAYBACK_VOLUME               "DAC Playback Volume"
#define MIXER_DACMAIN_SRC                       "DACMAIN SRC"
#define MIXER_DACVOICE_SRC                      "DACVOICE SRC"
#define MIXER_DIGITAL_MIXER_ADC_SWITCH          "DIGITAL MIXER Adc Switch"
#define MIXER_DIGITAL_MIXER_DIR0_SWITCH         "DIGITAL MIXER Dir0 Switch"
#define MIXER_DIGITAL_MIXER_DIR1_SWITCH         "DIGITAL MIXER Dir1 Switch"
#define MIXER_DIGITAL_MIXER_DIR2_SWITCH         "DIGITAL MIXER Dir2 Switch"
#define MIXER_DIR0_ATT_SWITCH                   "DIR#0 ATT Switch"
#define MIXER_DIR0_ATT_VOLUME                   "DIR#0 ATT Volume"
#define MIXER_DIR0_SWITCH                       "DIR#0 Switch"
#define MIXER_DIR0_VOLUME                       "DIR#0 Volume"
#define MIXER_DIR1_ATT_SWITCH                   "DIR#1 ATT Switch"
#define MIXER_DIR1_ATT_VOLUME                   "DIR#1 ATT Volume"
#define MIXER_DIR1_SWITCH                       "DIR#1 Switch"
#define MIXER_DIR1_VOLUME                       "DIR#1 Volume"
#define MIXER_DIR2_ATT_SWITCH                   "DIR#2 ATT Switch"
#define MIXER_DIR2_ATT_VOLUME                   "DIR#2 ATT Volume"
#define MIXER_DIR2_SWITCH                       "DIR#2 Switch"
#define MIXER_DIR2_VOLUME                       "DIR#2 Volume"
#define MIXER_DIT0_CAPTURE_SWITCH               "DIT#0 Capture Switch"
#define MIXER_DIT0_CAPTURE_VOLUME               "DIT#0 Capture Volume"
#define MIXER_DIT1_CAPTURE_SWITCH               "DIT#1 Capture Switch"
#define MIXER_DIT1_CAPTURE_VOLUME               "DIT#1 Capture Volume"
#define MIXER_DIT2_CAPTURE_SWITCH               "DIT#2 Capture Switch"
#define MIXER_DIT2_CAPTURE_VOLUME               "DIT#2 Capture Volume"
#define MIXER_DIT0_SRC                          "DIT0 SRC"
#define MIXER_DIT1_SRC                          "DIT1 SRC"
#define MIXER_DIT2_SRC                          "DIT2 SRC"
#define MIXER_HP_GAIN_PLAYBACK_VOLUME           "HP Gain Playback Volume"
#define MIXER_HPL_DAC_MIXMODE                   "HPL DAC MIXMODE"
#define MIXER_HPL_LINE_MIXMODE                  "HPL LINE MIXMODE"
#define MIXER_HPL_MIXER_DAC_SWITCH              "HPL MIXER Dac Switch"
#define MIXER_HPL_MIXER_LINE_SWITCH             "HPL MIXER Line Switch"
#define MIXER_HPL_MIXER_MIC1_SWITCH             "HPL MIXER Mic1 Switch"
#define MIXER_HPL_MIXER_MIC2_SWITCH             "HPL MIXER Mic2 Switch"
#define MIXER_HPL_MIXER_MIC3_SWITCH             "HPL MIXER Mic3 Switch"
#define MIXER_HPR_MIXER_DACR_SWITCH             "HPR MIXER DacR Switch"
#define MIXER_HPR_MIXER_LINER_SWITCH            "HPR MIXER LineR Switch"
#define MIXER_HPR_MIXER_MIC1_SWITCH             "HPR MIXER Mic1 Switch"
#define MIXER_HPR_MIXER_MIC2_SWITCH             "HPR MIXER Mic2 Switch"
#define MIXER_HPR_MIXER_MIC3_SWITCH             "HPR MIXER Mic3 Switch"
#define MIXER_LINEOUT1L_DAC_MIXMODE             "LINEOUT1L DAC MIXMODE"
#define MIXER_LINEOUT1L_LINE_MIXMODE            "LINEOUT1L LINE MIXMODE"
#define MIXER_LINEOUT1L_MIXER_DAC_SWITCH        "LINEOUT1L MIXER Dac Switch"
#define MIXER_LINEOUT1L_MIXER_LINE_SWITCH       "LINEOUT1L MIXER Line Switch"
#define MIXER_LINEOUT1L_MIXER_MIC1_SWITCH       "LINEOUT1L MIXER Mic1 Switch"
#define MIXER_LINEOUT1L_MIXER_MIC2_SWITCH       "LINEOUT1L MIXER Mic2 Switch"
#define MIXER_LINEOUT1L_MIXER_MIC3_SWITCH       "LINEOUT1L MIXER Mic3 Switch"
#define MIXER_LINEOUT1R_MIXER_DACR_SWITCH       "LINEOUT1R MIXER DacR Switch"
#define MIXER_LINEOUT1R_MIXER_LINER_SWITCH      "LINEOUT1R MIXER LineR Switch"
#define MIXER_LINEOUT1R_MIXER_MIC1_SWITCH       "LINEOUT1R MIXER Mic1 Switch"
#define MIXER_LINEOUT1R_MIXER_MIC2_SWITCH       "LINEOUT1R MIXER Mic2 Switch"
#define MIXER_LINEOUT1R_MIXER_MIC3_SWITCH       "LINEOUT1R MIXER Mic3 Switch"
#define MIXER_LINEOUT2L_DAC_MIXMODE             "LINEOUT2L DAC MIXMODE"
#define MIXER_LINEOUT2L_LINE_MIXMODE            "LINEOUT2L LINE MIXMODE"
#define MIXER_LINEOUT2L_MIXER_DAC_SWITCH        "LINEOUT2L MIXER Dac Switch"
#define MIXER_LINEOUT2L_MIXER_LINE_SWITCH       "LINEOUT2L MIXER Line Switch"
#define MIXER_LINEOUT2L_MIXER_MIC1_SWITCH       "LINEOUT2L MIXER Mic1 Switch"
#define MIXER_LINEOUT2L_MIXER_MIC2_SWITCH       "LINEOUT2L MIXER Mic2 Switch"
#define MIXER_LINEOUT2L_MIXER_MIC3_SWITCH       "LINEOUT2L MIXER Mic3 Switch"
#define MIXER_LINEOUT2R_MIXER_DACR_SWITCH       "LINEOUT2R MIXER DacR Switch"
#define MIXER_LINEOUT2R_MIXER_LINER_SWITCH      "LINEOUT2R MIXER LineR Switch"
#define MIXER_LINEOUT2R_MIXER_MIC1_SWITCH       "LINEOUT2R MIXER Mic1 Switch"
#define MIXER_LINEOUT2R_MIXER_MIC2_SWITCH       "LINEOUT2R MIXER Mic2 Switch"
#define MIXER_LINEOUT2R_MIXER_MIC3_SWITCH       "LINEOUT2R MIXER Mic3 Switch"
#define MIXER_MB1_SWITCH                        "MB1 Switch"
#define MIXER_MB2_SWITCH                        "MB2 Switch"
#define MIXER_MB3_SWITCH                        "MB3 Switch"
#define MIXER_PDM_SWITCH                        "PDM Switch"
#define MIXER_PDM_VOLUME                        "PDM Volume"
#define MIXER_RC_MIXER_DACL_SWITCH              "RC MIXER DacL Switch"
#define MIXER_RC_MIXER_DACR_SWITCH              "RC MIXER DacR Switch"
#define MIXER_RC_MIXER_LINEMONO_SWITCH          "RC MIXER LineMono Switch"
#define MIXER_RC_MIXER_MIC1_SWITCH              "RC MIXER Mic1 Switch"
#define MIXER_RC_MIXER_MIC2_SWITCH              "RC MIXER Mic2 Switch"
#define MIXER_RC_MIXER_MIC3_SWITCH              "RC MIXER Mic3 Switch"
#define MIXER_RECEIVER_PLAYBACK_SWITCH          "Receiver Playback Switch"
#define MIXER_RECEIVER_PLAYBACK_VOLUME          "Receiver Playback Volume"
#define MIXER_SPL_DAC_MIXMODE                   "SPL DAC MIXMODE"
#define MIXER_SPL_LINE_MIXMODE                  "SPL LINE MIXMODE"
#define MIXER_SPL_MIXER_DAC_SWITCH              "SPL MIXER Dac Switch"
#define MIXER_SPL_MIXER_LINE_SWITCH             "SPL MIXER Line Switch"
#define MIXER_SPR_DAC_MIXMODE                   "SPR DAC MIXMODE"
#define MIXER_SPR_LINE_MIXMODE                  "SPR LINE MIXMODE"
#define MIXER_SPR_MIXER_DAC_SWITCH              "SPR MIXER Dac Switch"
#define MIXER_SPR_MIXER_LINE_SWITCH             "SPR MIXER Line Switch"
#define MIXER_SIDE_TONE_PLAYBACK_SWITCH         "Side Tone Playback Switch"
#define MIXER_SIDE_TONE_PLAYBACK_VOLUME         "Side Tone Playback Volume"
#define MIXER_SPEAKER_PLAYBACK_SWITCH           "Speaker Playback Switch"
#define MIXER_SPEAKER_PLAYBACK_VOLUME           "Speaker Playback Volume"
#define MIXER_VOICE_PLAYBACK_SWITCH             "Voice Playback Switch"
#define MIXER_VOICE_PLAYBACK_VOLUME             "Voice Playback Volume"


#define DEFAULT_OUT_SAMPLING_RATE 44100


struct stub_audio_device {
    struct audio_hw_device device;
};

struct stub_stream_out {
    struct audio_stream_out stream;
};

struct stub_stream_in {
    struct audio_stream_in stream;
};

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    return 44100;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return 0;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    return 4096;
}

static uint32_t out_get_channels(const struct audio_stream *stream)
{
    return AUDIO_CHANNEL_OUT_STEREO;
}

static int out_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, int format)
{
    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    return 0;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    return 0;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    return strdup("");
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    return 0;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    return 0;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    /* XXX: fake timing for audio output */
    usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
           out_get_sample_rate(&stream->common));
    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    return -EINVAL;
}

static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    return 8000;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return 0;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    return 320;
}

static uint32_t in_get_channels(const struct audio_stream *stream)
{
    return AUDIO_CHANNEL_IN_MONO;
}

static int in_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int in_set_format(struct audio_stream *stream, int format)
{
    return 0;
}

static int in_standby(struct audio_stream *stream)
{
    return 0;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    return 0;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    /* XXX: fake timing for audio input */
    usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
           in_get_sample_rate(&stream->common));
    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   uint32_t devices, int *format,
                                   uint32_t *channels, uint32_t *sample_rate,
                                   struct audio_stream_out **stream_out)
{
    struct stub_audio_device *ladev = (struct stub_audio_device *)dev;
    struct stub_stream_out *out;
    int ret;

    out = (struct stub_stream_out *)calloc(1, sizeof(struct stub_stream_out));
    if (!out)
        return -ENOMEM;

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;

    *stream_out = &out->stream;
    return 0;

err_open:
    free(out);
    *stream_out = NULL;
    return ret;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    free(stream);
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    return -ENOSYS;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    return NULL;
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, int mode)
{
    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    return -ENOSYS;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    return -ENOSYS;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
                                         uint32_t sample_rate, int format,
                                         int channel_count)
{
    return 320;
}

static int adev_open_input_stream(struct audio_hw_device *dev, uint32_t devices,
                                  int *format, uint32_t *channels,
                                  uint32_t *sample_rate,
                                  audio_in_acoustics_t acoustics,
                                  struct audio_stream_in **stream_in)
{
    struct stub_audio_device *ladev = (struct stub_audio_device *)dev;
    struct stub_stream_in *in;
    int ret;

    in = (struct stub_stream_in *)calloc(1, sizeof(struct stub_stream_in));
    if (!in)
        return -ENOMEM;

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

    *stream_in = &in->stream;
    return 0;

err_open:
    free(in);
    *stream_in = NULL;
    return ret;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
                                   struct audio_stream_in *in)
{
    return;
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    return 0;
}

static int adev_close(hw_device_t *device)
{
    free(device);
    return 0;
}

static uint32_t adev_get_supported_devices(const struct audio_hw_device *dev)
{
    return (/* OUT */
            AUDIO_DEVICE_OUT_EARPIECE |
            AUDIO_DEVICE_OUT_SPEAKER |
            AUDIO_DEVICE_OUT_WIRED_HEADSET |
            AUDIO_DEVICE_OUT_WIRED_HEADPHONE |
            AUDIO_DEVICE_OUT_AUX_DIGITAL |
            AUDIO_DEVICE_OUT_ANLG_DOCK_HEADSET |
            AUDIO_DEVICE_OUT_DGTL_DOCK_HEADSET |
            AUDIO_DEVICE_OUT_ALL_SCO |
            AUDIO_DEVICE_OUT_DEFAULT |
            /* IN */
            AUDIO_DEVICE_IN_COMMUNICATION |
            AUDIO_DEVICE_IN_AMBIENT |
            AUDIO_DEVICE_IN_BUILTIN_MIC |
            AUDIO_DEVICE_IN_WIRED_HEADSET |
            AUDIO_DEVICE_IN_AUX_DIGITAL |
            AUDIO_DEVICE_IN_BACK_MIC |
            AUDIO_DEVICE_IN_ALL_SCO |
            AUDIO_DEVICE_IN_DEFAULT);
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    struct stub_audio_device *adev;
    int ret;

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct stub_audio_device));
    if (!adev)
        return -ENOMEM;

    adev->device.common.tag = HARDWARE_DEVICE_TAG;
    adev->device.common.version = 0;
    adev->device.common.module = (struct hw_module_t *) module;
    adev->device.common.close = adev_close;

    adev->device.get_supported_devices = adev_get_supported_devices;
    adev->device.init_check = adev_init_check;
    adev->device.set_voice_volume = adev_set_voice_volume;
    adev->device.set_master_volume = adev_set_master_volume;
    adev->device.set_mode = adev_set_mode;
    adev->device.set_mic_mute = adev_set_mic_mute;
    adev->device.get_mic_mute = adev_get_mic_mute;
    adev->device.set_parameters = adev_set_parameters;
    adev->device.get_parameters = adev_get_parameters;
    adev->device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->device.open_output_stream = adev_open_output_stream;
    adev->device.close_output_stream = adev_close_output_stream;
    adev->device.open_input_stream = adev_open_input_stream;
    adev->device.close_input_stream = adev_close_input_stream;
    adev->device.dump = adev_dump;

    *device = &adev->device.common;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "MC1N2 audio HW HAL",
        .author = "The CyanogenMod Project",
        .methods = &hal_module_methods,
    },
};
