/*
 * Copyright (C) 2011 The Android Open Source Project
 * Copyright (C) 2011 The CyanogenMod Project
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

#define LOG_TAG "AudioHardware"
#define LOG_NDEBUG 0

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>
#include <stdlib.h>

#include <cutils/log.h>
#include <cutils/str_parms.h>
#include <cutils/properties.h>

#include <hardware/hardware.h>
#include <system/audio.h>
#include <hardware/audio.h>

#include <tinyalsa/asoundlib.h>
#include <audio_utils/resampler.h>
#include <audio_utils/echo_reference.h>
#include <hardware/audio_effect.h>
#include <audio_effects/effect_aec.h>

#include "audio_hw.h"
#include "hwdep.h"
#include "ril_interface.h"

/* Common defines */
#define DEFAULT_OUT_SAMPLING_RATE 44100

/* ALSA cards for MC1N2 */
#define CARD_MC1N2_DEFAULT      0

struct audio_device {
    struct audio_hw_device device;
    
    pthread_mutex_t lock;
    struct mixer *mixer;
    struct mixer_ctls mixer_ctls;

    /* RIL */
    struct ril_handle ril;
};

struct stream_out {
    struct audio_stream_out stream;
};

struct stream_in {
    struct audio_stream_in stream;
};

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return 44100;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return 4096;
}

static uint32_t out_get_channels(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return AUDIO_CHANNEL_OUT_STEREO;
}

static int out_get_format(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, int format)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    LOGD("%s called.\n", __func__ );
    return strdup("");
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    LOGD("%s called.\n", __func__ );
    /* XXX: fake timing for audio output */
    usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
           out_get_sample_rate(&stream->common));
    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    LOGD("%s called.\n", __func__ );
    return -EINVAL;
}

static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return 8000;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return 320;
}

static uint32_t in_get_channels(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return AUDIO_CHANNEL_IN_MONO;
}

static int in_get_format(const struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int in_set_format(struct audio_stream *stream, int format)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int in_standby(struct audio_stream *stream)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    LOGD("%s called.\n", __func__ );
    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    LOGD("%s called.\n", __func__ );
    /* XXX: fake timing for audio input */
    usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
           in_get_sample_rate(&stream->common));
    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    LOGD("%s called.\n", __func__ );
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
    LOGD("%s called.\n", __func__ );
    struct audio_device *ladev = (struct audio_device *)dev;
    struct stream_out *out;
    int ret;

    out = (struct stream_out *)calloc(1, sizeof(struct stream_out));
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
    LOGD("%s called.\n", __func__ );
    free(stream);
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    LOGD("%s called.\n", __func__ );
    return -ENOSYS;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    LOGD("%s called.\n", __func__ );
    return NULL;
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    LOGD("%s called.\n", __func__ );
    // connectRILDIfRequired()
    return -ENOSYS;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    LOGD("%s called.\n", __func__ );
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, int mode)
{
    LOGD("%s called.\n", __func__ );
    // connectRILDIfRequired()
    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    LOGD("%s called.\n", __func__ );
    return -ENOSYS;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    LOGD("%s called.\n", __func__ );
    return -ENOSYS;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
                                         uint32_t sample_rate, int format,
                                         int channel_count)
{
    LOGD("%s called.\n", __func__ );
    return 320;
}

static int adev_open_input_stream(struct audio_hw_device *dev, uint32_t devices,
                                  int *format, uint32_t *channels,
                                  uint32_t *sample_rate,
                                  audio_in_acoustics_t acoustics,
                                  struct audio_stream_in **stream_in)
{
    LOGD("%s called.\n", __func__ );
    struct audio_device *ladev = (struct audio_device *)dev;
    struct stream_in *in;
    int ret;

    in = (struct stream_in *)calloc(1, sizeof(struct stream_in));
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
    LOGD("%s called.\n", __func__ );
    return;
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    LOGD("%s called.\n", __func__ );
    return 0;
}

static int adev_close(hw_device_t *device)
{
    LOGD("%s called.\n", __func__ );

    struct audio_device *adev = (struct audio_device *)device;

    /* RIL */
    ril_close(&adev->ril);

    free(device);
    return 0;
}

static uint32_t adev_get_supported_devices(const struct audio_hw_device *dev)
{
    LOGD("%s called.\n", __func__ );
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

/*
*   RIL
*/



static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    LOGD("%s called.\n", __func__ );
    struct audio_device *adev;
    int ret;

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
    {
        LOGE("%s: audio interface named %s not found.\n", __func__, name);
        return -EINVAL;
    }

    adev = calloc(1, sizeof(struct audio_device));
    if (!adev)
    {
        LOGE("%s: couldn't allocate memory.\n", __func__);
        return -ENOMEM;
    }

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

    // open mixer
    adev->mixer = mixer_open(0);
    if (!adev->mixer) {
        free(adev);
        LOGE("Unable to open the mixer, aborting.");
        return -EINVAL;
    } else {
        LOGD("Mixer opened.");
    }

    // get mixer controls
    adev->mixer_ctls.master_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MASTER_PLAYBACK_SWITCH);
    adev->mixer_ctls.master_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MASTER_PLAYBACK_VOLUME);
    adev->mixer_ctls.headphone_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HEADPHONE_PLAYBACK_SWITCH);
    adev->mixer_ctls.headphone_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_HEADPHONE_PLAYBACK_VOLUME);
    adev->mixer_ctls.line1_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINE1_PLAYBACK_SWITCH);
    adev->mixer_ctls.line1_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_LINE1_PLAYBACK_VOLUME);
    adev->mixer_ctls.line2_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINE2_PLAYBACK_SWITCH);
    adev->mixer_ctls.line2_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_LINE2_PLAYBACK_VOLUME);
    adev->mixer_ctls.line_bypass_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINE_BYPASS_PLAYBACK_SWITCH);
    adev->mixer_ctls.line_bypass_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_LINE_BYPASS_PLAYBACK_VOLUME);
    adev->mixer_ctls.mic1_bypass_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC1_BYPASS_PLAYBACK_SWITCH);
    adev->mixer_ctls.mic1_bypass_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC1_BYPASS_PLAYBACK_VOLUME);
    adev->mixer_ctls.mic1_gain_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC1_GAIN_VOLUME);
    adev->mixer_ctls.mic2_bypass_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC2_BYPASS_PLAYBACK_SWITCH);
    adev->mixer_ctls.mic2_bypass_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC2_BYPASS_PLAYBACK_VOLUME);
    adev->mixer_ctls.mic2_gain_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC2_GAIN_VOLUME);
    adev->mixer_ctls.mic3_bypass_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC3_BYPASS_PLAYBACK_SWITCH);
    adev->mixer_ctls.mic3_bypass_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC3_BYPASS_PLAYBACK_VOLUME);
    adev->mixer_ctls.mic3_gain_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_MIC3_GAIN_VOLUME);
    adev->mixer_ctls.adc_pdm_sel = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCL_PDM_SEL);
    adev->mixer_ctls.adcl_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCL_LINE_MIXMODE);
    adev->mixer_ctls.adcl_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCL_MIXER_LINE_SWITCH);
    adev->mixer_ctls.adcl_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCL_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.adcl_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCL_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.adcl_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCL_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.adcr_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCR_LINE_MIXMODE);
    adev->mixer_ctls.adcr_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCR_MIXER_LINE_SWITCH);
    adev->mixer_ctls.adcr_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCR_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.adcr_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCR_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.adcr_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_ADCR_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.ad_att_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_AD_ATT_SWITCH);
    adev->mixer_ctls.ad_att_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_AD_ATT_VOLUME);
    adev->mixer_ctls.ad_analog_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_AD_ANALOG_SWITCH);
    adev->mixer_ctls.ad_analog_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_AD_ANALOG_VOLUME);
    adev->mixer_ctls.ad_digital_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_AD_DIGITAL_SWITCH);
    adev->mixer_ctls.ad_digital_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_AD_DIGITAL_VOLUME);
    adev->mixer_ctls.ae_parameter_sel = mixer_get_ctl_by_name(adev->mixer,MIXER_AE_PARAMETER_SEL);
    adev->mixer_ctls.ae_src = mixer_get_ctl_by_name(adev->mixer,MIXER_AE_SRC);
    adev->mixer_ctls.aeng6_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_AENG6_SWITCH);
    adev->mixer_ctls.aeng6_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_AENG6_VOLUME);
    adev->mixer_ctls.codec_status = mixer_get_ctl_by_name(adev->mixer,MIXER_CODEC_STATUS);
    adev->mixer_ctls.dac_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DAC_PLAYBACK_VOLUME);
    adev->mixer_ctls.dac_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DAC_PLAYBACK_SWITCH);
    adev->mixer_ctls.dac_main_src = mixer_get_ctl_by_name(adev->mixer,MIXER_DACMAIN_SRC);
    adev->mixer_ctls.dac_voice_src = mixer_get_ctl_by_name(adev->mixer,MIXER_DACVOICE_SRC);
    adev->mixer_ctls.digital_mixer_adc_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIGITAL_MIXER_ADC_SWITCH);
    adev->mixer_ctls.digital_mixer_dir0_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIGITAL_MIXER_DIR0_SWITCH);
    adev->mixer_ctls.digital_mixer_dir1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIGITAL_MIXER_DIR1_SWITCH);
    adev->mixer_ctls.digital_mixer_dir2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIGITAL_MIXER_DIR2_SWITCH );
    adev->mixer_ctls.mixer_dir0_att_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR0_ATT_SWITCH);
    adev->mixer_ctls.mixer_dir0_att_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR0_ATT_VOLUME);
    adev->mixer_ctls.mixer_dir0_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR0_SWITCH);
    adev->mixer_ctls.mixer_dir0_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR0_VOLUME);
    adev->mixer_ctls.mixer_dir1_att_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR1_ATT_SWITCH);
    adev->mixer_ctls.mixer_dir1_att_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR1_ATT_VOLUME);
    adev->mixer_ctls.mixer_dir1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR1_SWITCH);
    adev->mixer_ctls.mixer_dir1_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR1_VOLUME);
    adev->mixer_ctls.mixer_dir2_att_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR2_ATT_SWITCH);
    adev->mixer_ctls.mixer_dir2_att_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR2_ATT_VOLUME);
    adev->mixer_ctls.mixer_dir2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR2_SWITCH);
    adev->mixer_ctls.mixer_dir2_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIR2_VOLUME);
    adev->mixer_ctls.mixer_dit0_capture_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT0_CAPTURE_SWITCH);
    adev->mixer_ctls.mixer_dit0_capture_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT0_CAPTURE_VOLUME);
    adev->mixer_ctls.mixer_dit1_capture_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT1_CAPTURE_SWITCH);
    adev->mixer_ctls.mixer_dit1_capture_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT1_CAPTURE_VOLUME);
    adev->mixer_ctls.mixer_dit2_capture_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT2_CAPTURE_SWITCH);
    adev->mixer_ctls.mixer_dit2_capture_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT2_CAPTURE_VOLUME);
    adev->mixer_ctls.mixer_dit0_src = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT0_SRC);
    adev->mixer_ctls.mixer_dit1_src = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT1_SRC);
    adev->mixer_ctls.mixer_dit2_src = mixer_get_ctl_by_name(adev->mixer,MIXER_DIT2_SRC);
    adev->mixer_ctls.mixer_hp_gain_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_HP_GAIN_PLAYBACK_VOLUME);
    adev->mixer_ctls.mixer_hpl_dac_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_DAC_MIXMODE);
    adev->mixer_ctls.mixer_hpl_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_LINE_MIXMODE);
    adev->mixer_ctls.mixer_hpl_mixer_dac_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_MIXER_DAC_SWITCH);
    adev->mixer_ctls.mixer_hpl_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_MIXER_LINE_SWITCH);
    adev->mixer_ctls.mixer_hpl_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_hpl_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_hpl_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPL_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_hpr_mixer_dacr_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPR_MIXER_DACR_SWITCH);
    adev->mixer_ctls.mixer_hpr_mixer_liner_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPR_MIXER_LINER_SWITCH);
    adev->mixer_ctls.mixer_hpr_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPR_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_hpr_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPR_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_hpr_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_HPR_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_lineout1l_dac_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_DAC_MIXMODE);
    adev->mixer_ctls.mixer_lineout1l_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_LINE_MIXMODE);
    adev->mixer_ctls.mixer_lineout1l_mixer_dac_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_MIXER_DAC_SWITCH);
    adev->mixer_ctls.mixer_lineout1l_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_MIXER_LINE_SWITCH);
    adev->mixer_ctls.mixer_lineout1l_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_lineout1l_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_lineout1l_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1L_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_lineout1r_mixer_dacr_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1R_MIXER_DACR_SWITCH);
    adev->mixer_ctls.mixer_lineout1r_mixer_liner_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1R_MIXER_LINER_SWITCH);
    adev->mixer_ctls.mixer_lineout1r_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1R_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_lineout1r_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1R_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_lineout1r_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT1R_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_lineout2l_dac_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_DAC_MIXMODE);
    adev->mixer_ctls.mixer_lineout2l_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_LINE_MIXMODE);
    adev->mixer_ctls.mixer_lineout2l_mixer_dac_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_MIXER_DAC_SWITCH);
    adev->mixer_ctls.mixer_lineout2l_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_MIXER_LINE_SWITCH);
    adev->mixer_ctls.mixer_lineout2l_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_lineout2l_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_lineout2l_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2L_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_lineout2r_mixer_dacr_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2R_MIXER_DACR_SWITCH);
    adev->mixer_ctls.mixer_lineout2r_mixer_liner_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2R_MIXER_LINER_SWITCH);
    adev->mixer_ctls.mixer_lineout2r_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2R_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_lineout2r_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2R_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_lineout2r_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_LINEOUT2R_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_mb1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MB1_SWITCH);
    adev->mixer_ctls.mixer_mb2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MB2_SWITCH);
    adev->mixer_ctls.mixer_mb3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_MB3_SWITCH);
    adev->mixer_ctls.mixer_pdm_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_PDM_SWITCH);
    adev->mixer_ctls.mixer_pdm_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_PDM_VOLUME);
    adev->mixer_ctls.mixer_rc_mixer_dacl_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RC_MIXER_DACL_SWITCH);
    adev->mixer_ctls.mixer_rc_mixer_dacr_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RC_MIXER_DACR_SWITCH);
    adev->mixer_ctls.mixer_rc_mixer_linemono_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RC_MIXER_LINEMONO_SWITCH);
    adev->mixer_ctls.mixer_rc_mixer_mic1_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RC_MIXER_MIC1_SWITCH);
    adev->mixer_ctls.mixer_rc_mixer_mic2_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RC_MIXER_MIC2_SWITCH);
    adev->mixer_ctls.mixer_rc_mixer_mic3_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RC_MIXER_MIC3_SWITCH);
    adev->mixer_ctls.mixer_receiver_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_RECEIVER_PLAYBACK_SWITCH);
    adev->mixer_ctls.mixer_receiver_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_RECEIVER_PLAYBACK_VOLUME);
    adev->mixer_ctls.mixer_spl_dac_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_SPL_DAC_MIXMODE);
    adev->mixer_ctls.mixer_spl_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_SPL_LINE_MIXMODE);
    adev->mixer_ctls.mixer_spl_mixer_dac_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_SPL_MIXER_DAC_SWITCH);
    adev->mixer_ctls.mixer_spl_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_SPL_MIXER_LINE_SWITCH);
    adev->mixer_ctls.mixer_spr_dac_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_SPR_DAC_MIXMODE);
    adev->mixer_ctls.mixer_spr_line_mixmode = mixer_get_ctl_by_name(adev->mixer,MIXER_SPR_LINE_MIXMODE);
    adev->mixer_ctls.mixer_spr_mixer_dac_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_SPR_MIXER_DAC_SWITCH);
    adev->mixer_ctls.mixer_spr_mixer_line_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_SPR_MIXER_LINE_SWITCH);
    adev->mixer_ctls.mixer_side_tone_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_SIDE_TONE_PLAYBACK_SWITCH);
    adev->mixer_ctls.mixer_side_tone_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_SIDE_TONE_PLAYBACK_VOLUME);
    adev->mixer_ctls.mixer_speaker_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_SPEAKER_PLAYBACK_SWITCH);
    adev->mixer_ctls.mixer_speaker_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_SPEAKER_PLAYBACK_VOLUME);
    adev->mixer_ctls.mixer_voice_playback_switch = mixer_get_ctl_by_name(adev->mixer,MIXER_VOICE_PLAYBACK_SWITCH);
    adev->mixer_ctls.mixer_voice_playback_volume = mixer_get_ctl_by_name(adev->mixer,MIXER_VOICE_PLAYBACK_VOLUME);

    // locate all mixer controlls
    if (!adev->mixer_ctls.master_playback_switch || !adev->mixer_ctls.master_playback_volume || !adev->mixer_ctls.headphone_playback_switch || 
        !adev->mixer_ctls.headphone_playback_volume || !adev->mixer_ctls.line1_playback_switch || !adev->mixer_ctls.line1_playback_volume || 
        !adev->mixer_ctls.line2_playback_switch || !adev->mixer_ctls.line2_playback_volume || !adev->mixer_ctls.line_bypass_playback_switch || 
        !adev->mixer_ctls.line_bypass_playback_volume || !adev->mixer_ctls.mic1_bypass_playback_switch || !adev->mixer_ctls.mic1_bypass_playback_volume || 
        !adev->mixer_ctls.mic1_gain_volume || !adev->mixer_ctls.mic2_bypass_playback_switch || !adev->mixer_ctls.mic2_bypass_playback_volume || 
        !adev->mixer_ctls.mic2_gain_volume || !adev->mixer_ctls.mic3_bypass_playback_switch || !adev->mixer_ctls.mic3_bypass_playback_volume || 
        !adev->mixer_ctls.mic3_gain_volume || !adev->mixer_ctls.adc_pdm_sel || !adev->mixer_ctls.adcl_line_mixmode || 
        !adev->mixer_ctls.adcl_mixer_line_switch || !adev->mixer_ctls.adcl_mixer_mic1_switch || !adev->mixer_ctls.adcl_mixer_mic2_switch || 
        !adev->mixer_ctls.adcl_mixer_mic3_switch || !adev->mixer_ctls.adcr_line_mixmode || !adev->mixer_ctls.adcr_mixer_line_switch || 
        !adev->mixer_ctls.adcr_mixer_mic1_switch || !adev->mixer_ctls.adcr_mixer_mic2_switch || !adev->mixer_ctls.adcr_mixer_mic3_switch || 
        !adev->mixer_ctls.ad_att_switch || !adev->mixer_ctls.ad_att_volume || !adev->mixer_ctls.ad_analog_switch || 
        !adev->mixer_ctls.ad_analog_volume || !adev->mixer_ctls.ad_digital_switch || !adev->mixer_ctls.ad_digital_volume || 
        !adev->mixer_ctls.ae_parameter_sel || !adev->mixer_ctls.ae_src || !adev->mixer_ctls.aeng6_switch || 
        !adev->mixer_ctls.aeng6_volume || !adev->mixer_ctls.codec_status || !adev->mixer_ctls.dac_playback_volume ||
        !adev->mixer_ctls.dac_playback_switch || !adev->mixer_ctls.dac_main_src || !adev->mixer_ctls.dac_voice_src || 
        !adev->mixer_ctls.digital_mixer_adc_switch || !adev->mixer_ctls.digital_mixer_dir0_switch || !adev->mixer_ctls.digital_mixer_dir1_switch || 
        !adev->mixer_ctls.digital_mixer_dir2_switch || !adev->mixer_ctls.mixer_dir0_att_switch || !adev->mixer_ctls.mixer_dir0_att_volume || 
        !adev->mixer_ctls.mixer_dir0_switch || !adev->mixer_ctls.mixer_dir0_volume || !adev->mixer_ctls.mixer_dir1_att_switch || 
        !adev->mixer_ctls.mixer_dir1_att_volume || !adev->mixer_ctls.mixer_dir1_switch || !adev->mixer_ctls.mixer_dir1_volume || 
        !adev->mixer_ctls.mixer_dir2_att_switch || !adev->mixer_ctls.mixer_dir2_att_volume || !adev->mixer_ctls.mixer_dir2_switch || 
        !adev->mixer_ctls.mixer_dir2_volume || !adev->mixer_ctls.mixer_dit0_capture_switch || !adev->mixer_ctls.mixer_dit0_capture_volume || 
        !adev->mixer_ctls.mixer_dit1_capture_switch || !adev->mixer_ctls.mixer_dit1_capture_volume || !adev->mixer_ctls.mixer_dit2_capture_switch || 
        !adev->mixer_ctls.mixer_dit2_capture_volume || !adev->mixer_ctls.mixer_dit0_src || !adev->mixer_ctls.mixer_dit1_src || 
        !adev->mixer_ctls.mixer_dit2_src || !adev->mixer_ctls.mixer_hp_gain_playback_volume || !adev->mixer_ctls.mixer_hpl_dac_mixmode || 
        !adev->mixer_ctls.mixer_hpl_line_mixmode || !adev->mixer_ctls.mixer_hpl_mixer_dac_switch || !adev->mixer_ctls.mixer_hpl_mixer_line_switch || 
        !adev->mixer_ctls.mixer_hpl_mixer_mic1_switch || !adev->mixer_ctls.mixer_hpl_mixer_mic2_switch || !adev->mixer_ctls.mixer_hpl_mixer_mic3_switch || 
        !adev->mixer_ctls.mixer_hpr_mixer_dacr_switch || !adev->mixer_ctls.mixer_hpr_mixer_liner_switch || !adev->mixer_ctls.mixer_hpr_mixer_mic1_switch || 
        !adev->mixer_ctls.mixer_hpr_mixer_mic2_switch || !adev->mixer_ctls.mixer_hpr_mixer_mic3_switch || !adev->mixer_ctls.mixer_lineout1l_dac_mixmode || 
        !adev->mixer_ctls.mixer_lineout1l_line_mixmode || !adev->mixer_ctls.mixer_lineout1l_mixer_dac_switch || !adev->mixer_ctls.mixer_lineout1l_mixer_line_switch || 
        !adev->mixer_ctls.mixer_lineout1l_mixer_mic1_switch || !adev->mixer_ctls.mixer_lineout1l_mixer_mic2_switch || !adev->mixer_ctls.mixer_lineout1l_mixer_mic3_switch || 
        !adev->mixer_ctls.mixer_lineout1r_mixer_dacr_switch || !adev->mixer_ctls.mixer_lineout1r_mixer_liner_switch || !adev->mixer_ctls.mixer_lineout1r_mixer_mic1_switch || 
        !adev->mixer_ctls.mixer_lineout1r_mixer_mic2_switch || !adev->mixer_ctls.mixer_lineout1r_mixer_mic3_switch || !adev->mixer_ctls.mixer_lineout2l_dac_mixmode || 
        !adev->mixer_ctls.mixer_lineout2l_line_mixmode || !adev->mixer_ctls.mixer_lineout2l_mixer_dac_switch || !adev->mixer_ctls.mixer_lineout2l_mixer_line_switch || 
        !adev->mixer_ctls.mixer_lineout2l_mixer_mic1_switch || !adev->mixer_ctls.mixer_lineout2l_mixer_mic2_switch || !adev->mixer_ctls.mixer_lineout2l_mixer_mic3_switch || 
        !adev->mixer_ctls.mixer_lineout2r_mixer_dacr_switch || !adev->mixer_ctls.mixer_lineout2r_mixer_liner_switch || !adev->mixer_ctls.mixer_lineout2r_mixer_mic1_switch || 
        !adev->mixer_ctls.mixer_lineout2r_mixer_mic2_switch || !adev->mixer_ctls.mixer_lineout2r_mixer_mic3_switch || !adev->mixer_ctls.mixer_mb1_switch || 
        !adev->mixer_ctls.mixer_mb2_switch || !adev->mixer_ctls.mixer_mb3_switch || !adev->mixer_ctls.mixer_pdm_switch || 
        !adev->mixer_ctls.mixer_pdm_volume || !adev->mixer_ctls.mixer_rc_mixer_dacl_switch || !adev->mixer_ctls.mixer_rc_mixer_dacr_switch || 
        !adev->mixer_ctls.mixer_rc_mixer_linemono_switch || !adev->mixer_ctls.mixer_rc_mixer_mic1_switch || !adev->mixer_ctls.mixer_rc_mixer_mic2_switch || 
        !adev->mixer_ctls.mixer_rc_mixer_mic3_switch || !adev->mixer_ctls.mixer_receiver_playback_switch || !adev->mixer_ctls.mixer_receiver_playback_volume || 
        !adev->mixer_ctls.mixer_spl_dac_mixmode || !adev->mixer_ctls.mixer_spl_line_mixmode || !adev->mixer_ctls.mixer_spl_mixer_dac_switch || 
        !adev->mixer_ctls.mixer_spl_mixer_line_switch || !adev->mixer_ctls.mixer_spr_dac_mixmode || !adev->mixer_ctls.mixer_spr_line_mixmode || 
        !adev->mixer_ctls.mixer_spr_mixer_dac_switch || !adev->mixer_ctls.mixer_spr_mixer_line_switch || !adev->mixer_ctls.mixer_side_tone_playback_switch || 
        !adev->mixer_ctls.mixer_side_tone_playback_volume || !adev->mixer_ctls.mixer_speaker_playback_switch || !adev->mixer_ctls.mixer_speaker_playback_volume || 
        !adev->mixer_ctls.mixer_voice_playback_switch || !adev->mixer_ctls.mixer_voice_playback_volume)
    {
        mixer_close(adev->mixer);
        free(adev);
        LOGE("Unable to locate all mixer controls, aborting.");
        return -EINVAL;
    } else {
        LOGD("Located all mixer controls.");
    }

    pthread_mutex_lock(&adev->lock);

    /* RIL */
    ril_open(&adev->ril);
    pthread_mutex_unlock(&adev->lock);

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
