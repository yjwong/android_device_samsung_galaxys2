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

struct mixer_ctls
{
    struct mixer_ctl *master_playback_switch;
    struct mixer_ctl *master_playback_volume;
    struct mixer_ctl *headphone_playback_switch;
    struct mixer_ctl *headphone_playback_volume;
    struct mixer_ctl *line1_playback_switch;
    struct mixer_ctl *line1_playback_volume;
    struct mixer_ctl *line2_playback_switch;
    struct mixer_ctl *line2_playback_volume;
    struct mixer_ctl *line_bypass_playback_switch;
    struct mixer_ctl *line_bypass_playback_volume;
    struct mixer_ctl *mic1_bypass_playback_switch;
    struct mixer_ctl *mic1_bypass_playback_volume;
    struct mixer_ctl *mic1_gain_volume;
    struct mixer_ctl *mic2_bypass_playback_switch;
    struct mixer_ctl *mic2_bypass_playback_volume;
    struct mixer_ctl *mic2_gain_volume;
    struct mixer_ctl *mic3_bypass_playback_switch;
    struct mixer_ctl *mic3_bypass_playback_volume;
    struct mixer_ctl *mic3_gain_volume;
    struct mixer_ctl *adc_pdm_sel;
    struct mixer_ctl *adcl_line_mixmode;
    struct mixer_ctl *adcl_mixer_line_switch;
    struct mixer_ctl *adcl_mixer_mic1_switch;
    struct mixer_ctl *adcl_mixer_mic2_switch;
    struct mixer_ctl *adcl_mixer_mic3_switch;
    struct mixer_ctl *adcr_line_mixmode;
    struct mixer_ctl *adcr_mixer_line_switch;
    struct mixer_ctl *adcr_mixer_mic1_switch;
    struct mixer_ctl *adcr_mixer_mic2_switch;
    struct mixer_ctl *adcr_mixer_mic3_switch;
    struct mixer_ctl *ad_att_switch;
    struct mixer_ctl *ad_att_volume;
    struct mixer_ctl *ad_analog_switch;
    struct mixer_ctl *ad_analog_volume;
    struct mixer_ctl *ad_digital_switch;
    struct mixer_ctl *ad_digital_volume;
    struct mixer_ctl *ae_parameter_sel;
    struct mixer_ctl *ae_src;
    struct mixer_ctl *aeng6_switch;
    struct mixer_ctl *aeng6_volume;
    struct mixer_ctl *codec_status;
    struct mixer_ctl *dac_playback_switch;
    struct mixer_ctl *dac_playback_volume;
    struct mixer_ctl *dac_main_src;
    struct mixer_ctl *dac_voice_src;
    struct mixer_ctl *digital_mixer_adc_switch;
    struct mixer_ctl *digital_mixer_dir0_switch;
    struct mixer_ctl *digital_mixer_dir1_switch;
    struct mixer_ctl *digital_mixer_dir2_switch;
    struct mixer_ctl *mixer_dir0_att_switch;
    struct mixer_ctl *mixer_dir0_att_volume;
    struct mixer_ctl *mixer_dir0_switch;
    struct mixer_ctl *mixer_dir0_volume;
    struct mixer_ctl *mixer_dir1_att_switch;
    struct mixer_ctl *mixer_dir1_att_volume;
    struct mixer_ctl *mixer_dir1_switch;
    struct mixer_ctl *mixer_dir1_volume;
    struct mixer_ctl *mixer_dir2_att_switch;
    struct mixer_ctl *mixer_dir2_att_volume;
    struct mixer_ctl *mixer_dir2_switch;
    struct mixer_ctl *mixer_dir2_volume;
    struct mixer_ctl *mixer_dit0_capture_switch;
    struct mixer_ctl *mixer_dit0_capture_volume;
    struct mixer_ctl *mixer_dit1_capture_switch;
    struct mixer_ctl *mixer_dit1_capture_volume;
    struct mixer_ctl *mixer_dit2_capture_switch;
    struct mixer_ctl *mixer_dit2_capture_volume;
    struct mixer_ctl *mixer_dit0_src;
    struct mixer_ctl *mixer_dit1_src;
    struct mixer_ctl *mixer_dit2_src;
    struct mixer_ctl *mixer_hp_gain_playback_volume;
    struct mixer_ctl *mixer_hpl_dac_mixmode;
    struct mixer_ctl *mixer_hpl_line_mixmode;
    struct mixer_ctl *mixer_hpl_mixer_dac_switch;
    struct mixer_ctl *mixer_hpl_mixer_line_switch;
    struct mixer_ctl *mixer_hpl_mixer_mic1_switch;
    struct mixer_ctl *mixer_hpl_mixer_mic2_switch;
    struct mixer_ctl *mixer_hpl_mixer_mic3_switch;
    struct mixer_ctl *mixer_hpr_mixer_dacr_switch;
    struct mixer_ctl *mixer_hpr_mixer_liner_switch;
    struct mixer_ctl *mixer_hpr_mixer_mic1_switch;
    struct mixer_ctl *mixer_hpr_mixer_mic2_switch;
    struct mixer_ctl *mixer_hpr_mixer_mic3_switch;
    struct mixer_ctl *mixer_lineout1l_dac_mixmode;
    struct mixer_ctl *mixer_lineout1l_line_mixmode;
    struct mixer_ctl *mixer_lineout1l_mixer_dac_switch;
    struct mixer_ctl *mixer_lineout1l_mixer_line_switch;
    struct mixer_ctl *mixer_lineout1l_mixer_mic1_switch;
    struct mixer_ctl *mixer_lineout1l_mixer_mic2_switch;
    struct mixer_ctl *mixer_lineout1l_mixer_mic3_switch;
    struct mixer_ctl *mixer_lineout1r_mixer_dacr_switch;
    struct mixer_ctl *mixer_lineout1r_mixer_liner_switch;
    struct mixer_ctl *mixer_lineout1r_mixer_mic1_switch;
    struct mixer_ctl *mixer_lineout1r_mixer_mic2_switch;
    struct mixer_ctl *mixer_lineout1r_mixer_mic3_switch;
    struct mixer_ctl *mixer_lineout2l_dac_mixmode;
    struct mixer_ctl *mixer_lineout2l_line_mixmode;
    struct mixer_ctl *mixer_lineout2l_mixer_dac_switch;
    struct mixer_ctl *mixer_lineout2l_mixer_line_switch;
    struct mixer_ctl *mixer_lineout2l_mixer_mic1_switch;
    struct mixer_ctl *mixer_lineout2l_mixer_mic2_switch;
    struct mixer_ctl *mixer_lineout2l_mixer_mic3_switch;
    struct mixer_ctl *mixer_lineout2r_mixer_dacr_switch;
    struct mixer_ctl *mixer_lineout2r_mixer_liner_switch;
    struct mixer_ctl *mixer_lineout2r_mixer_mic1_switch;
    struct mixer_ctl *mixer_lineout2r_mixer_mic2_switch;
    struct mixer_ctl *mixer_lineout2r_mixer_mic3_switch;
    struct mixer_ctl *mixer_mb1_switch;
    struct mixer_ctl *mixer_mb2_switch;
    struct mixer_ctl *mixer_mb3_switch;
    struct mixer_ctl *mixer_pdm_switch;
    struct mixer_ctl *mixer_pdm_volume;
    struct mixer_ctl *mixer_rc_mixer_dacl_switch;
    struct mixer_ctl *mixer_rc_mixer_dacr_switch;
    struct mixer_ctl *mixer_rc_mixer_linemono_switch;
    struct mixer_ctl *mixer_rc_mixer_mic1_switch;
    struct mixer_ctl *mixer_rc_mixer_mic2_switch;
    struct mixer_ctl *mixer_rc_mixer_mic3_switch;
    struct mixer_ctl *mixer_receiver_playback_switch;
    struct mixer_ctl *mixer_receiver_playback_volume;
    struct mixer_ctl *mixer_spl_dac_mixmode;
    struct mixer_ctl *mixer_spl_line_mixmode;
    struct mixer_ctl *mixer_spl_mixer_dac_switch;
    struct mixer_ctl *mixer_spl_mixer_line_switch;
    struct mixer_ctl *mixer_spr_dac_mixmode;
    struct mixer_ctl *mixer_spr_line_mixmode;
    struct mixer_ctl *mixer_spr_mixer_dac_switch;
    struct mixer_ctl *mixer_spr_mixer_line_switch;
    struct mixer_ctl *mixer_side_tone_playback_switch;
    struct mixer_ctl *mixer_side_tone_playback_volume;
    struct mixer_ctl *mixer_speaker_playback_switch;
    struct mixer_ctl *mixer_speaker_playback_volume;
    struct mixer_ctl *mixer_voice_playback_switch;
    struct mixer_ctl *mixer_voice_playback_volume;
};
