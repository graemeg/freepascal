{*****************************************************************************}
{
    This file is part of the Free Pascal's "Free Components Library".
    Copyright (c) 2014 by Mazen NEIFER of the Free Pascal development team
    and was adapted from wavopenal.pas copyright (c) 2010 Dmitry Boyarintsev.

    RIFF/WAVE sound file basic types and constants.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
unit fpwavformat;

{$mode objfpc}{$H+}

interface

const
  AUDIO_CHUNK_ID_RIFF = 'RIFF';
  AUDIO_CHUNK_ID_WAVE = 'WAVE';
  AUDIO_CHUNK_ID_fmt  = 'fmt ';
  AUDIO_CHUNK_ID_data = 'data';
  AUDIO_FORMAT_PCM = 1;
  
  { WAVE form wFormatTag IDs }
  WAVE_FORMAT_UNKNOWN                    = $0000; { Microsoft Corporation }
  WAVE_FORMAT_PCM                        = $0001; { Microsoft Corporation }
  WAVE_FORMAT_ADPCM                      = $0002; { Microsoft Corporation }
  WAVE_FORMAT_IEEE_FLOAT                 = $0003; { Microsoft Corporation }
  WAVE_FORMAT_VSELP                      = $0004; { Compaq Computer Corp. }
  WAVE_FORMAT_IBM_CVSD                   = $0005; { IBM Corporation }
  WAVE_FORMAT_ALAW                       = $0006; { Microsoft Corporation }
  WAVE_FORMAT_MULAW                      = $0007; { Microsoft Corporation }
  WAVE_FORMAT_DTS                        = $0008; { Microsoft Corporation }
  WAVE_FORMAT_DRM                        = $0009; { Microsoft Corporation }
  WAVE_FORMAT_WMAVOICE9                  = $000A; { Microsoft Corporation }
  WAVE_FORMAT_WMAVOICE10                 = $000B; { Microsoft Corporation }
  WAVE_FORMAT_OKI_ADPCM                  = $0010; { OKI }
  WAVE_FORMAT_DVI_ADPCM                  = $0011; { Intel Corporation }
  WAVE_FORMAT_IMA_ADPCM                  = (WAVE_FORMAT_DVI_ADPCM); {  Intel Corporation }
  WAVE_FORMAT_MEDIASPACE_ADPCM           = $0012; { Videologic }
  WAVE_FORMAT_SIERRA_ADPCM               = $0013; { Sierra Semiconductor Corp }
  WAVE_FORMAT_G723_ADPCM                 = $0014; { Antex Electronics Corporation }
  WAVE_FORMAT_DIGISTD                    = $0015; { DSP Solutions, Inc. }
  WAVE_FORMAT_DIGIFIX                    = $0016; { DSP Solutions, Inc. }
  WAVE_FORMAT_DIALOGIC_OKI_ADPCM         = $0017; { Dialogic Corporation }
  WAVE_FORMAT_MEDIAVISION_ADPCM          = $0018; { Media Vision, Inc. }
  WAVE_FORMAT_CU_CODEC                   = $0019; { Hewlett-Packard Company }
  WAVE_FORMAT_HP_DYN_VOICE               = $001A; { Hewlett-Packard Company }
  WAVE_FORMAT_YAMAHA_ADPCM               = $0020; { Yamaha Corporation of America }
  WAVE_FORMAT_SONARC                     = $0021; { Speech Compression }
  WAVE_FORMAT_DSPGROUP_TRUESPEECH        = $0022; { DSP Group, Inc }
  WAVE_FORMAT_ECHOSC1                    = $0023; { Echo Speech Corporation }
  WAVE_FORMAT_AUDIOFILE_AF36             = $0024; { Virtual Music, Inc. }
  WAVE_FORMAT_APTX                       = $0025; { Audio Processing Technology }
  WAVE_FORMAT_AUDIOFILE_AF10             = $0026; { Virtual Music, Inc. }
  WAVE_FORMAT_PROSODY_1612               = $0027; { Aculab plc }
  WAVE_FORMAT_LRC                        = $0028; { Merging Technologies S.A. }
  WAVE_FORMAT_DOLBY_AC2                  = $0030; { Dolby Laboratories }
  WAVE_FORMAT_GSM610                     = $0031; { Microsoft Corporation }
  WAVE_FORMAT_MSNAUDIO                   = $0032; { Microsoft Corporation }
  WAVE_FORMAT_ANTEX_ADPCME               = $0033; { Antex Electronics Corporation }
  WAVE_FORMAT_CONTROL_RES_VQLPC          = $0034; { Control Resources Limited }
  WAVE_FORMAT_DIGIREAL                   = $0035; { DSP Solutions, Inc. }
  WAVE_FORMAT_DIGIADPCM                  = $0036; { DSP Solutions, Inc. }
  WAVE_FORMAT_CONTROL_RES_CR10           = $0037; { Control Resources Limited }
  WAVE_FORMAT_NMS_VBXADPCM               = $0038; { Natural MicroSystems }
  WAVE_FORMAT_CS_IMAADPCM                = $0039; { Crystal Semiconductor IMA ADPCM }
  WAVE_FORMAT_ECHOSC3                    = $003A; { Echo Speech Corporation }
  WAVE_FORMAT_ROCKWELL_ADPCM             = $003B; { Rockwell International }
  WAVE_FORMAT_ROCKWELL_DIGITALK          = $003C; { Rockwell International }
  WAVE_FORMAT_XEBEC                      = $003D; { Xebec Multimedia Solutions Limited }
  WAVE_FORMAT_G721_ADPCM                 = $0040; { Antex Electronics Corporation }
  WAVE_FORMAT_G728_CELP                  = $0041; { Antex Electronics Corporation }
  WAVE_FORMAT_MSG723                     = $0042; { Microsoft Corporation }
  WAVE_FORMAT_INTEL_G723_1               = $0043; { Intel Corp. }
  WAVE_FORMAT_INTEL_G729                 = $0044; { Intel Corp. }
  WAVE_FORMAT_SHARP_G726                 = $0045; { Sharp }
  WAVE_FORMAT_MPEG                       = $0050; { Microsoft Corporation }
  WAVE_FORMAT_RT24                       = $0052; { InSoft, Inc. }
  WAVE_FORMAT_PAC                        = $0053; { InSoft, Inc. }
  WAVE_FORMAT_MPEGLAYER3                 = $0055; { ISO/MPEG Layer3 Format Tag }
  WAVE_FORMAT_LUCENT_G723                = $0059; { Lucent Technologies }
  WAVE_FORMAT_CIRRUS                     = $0060; { Cirrus Logic }
  WAVE_FORMAT_ESPCM                      = $0061; { ESS Technology }
  WAVE_FORMAT_VOXWARE                    = $0062; { Voxware Inc }
  WAVE_FORMAT_CANOPUS_ATRAC              = $0063; { Canopus, co., Ltd. }
  WAVE_FORMAT_G726_ADPCM                 = $0064; { APICOM }
  WAVE_FORMAT_G722_ADPCM                 = $0065; { APICOM }
  WAVE_FORMAT_DSAT                       = $0066; { Microsoft Corporation }
  WAVE_FORMAT_DSAT_DISPLAY               = $0067; { Microsoft Corporation }
  WAVE_FORMAT_VOXWARE_BYTE_ALIGNED       = $0069; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_AC8                = $0070; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_AC10               = $0071; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_AC16               = $0072; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_AC20               = $0073; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_RT24               = $0074; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_RT29               = $0075; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_RT29HW             = $0076; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_VR12               = $0077; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_VR18               = $0078; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_TQ40               = $0079; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_SC3                = $007A; { Voxware Inc }
  WAVE_FORMAT_VOXWARE_SC3_1              = $007B; { Voxware Inc }
  WAVE_FORMAT_SOFTSOUND                  = $0080; { Softsound, Ltd. }
  WAVE_FORMAT_VOXWARE_TQ60               = $0081; { Voxware Inc }
  WAVE_FORMAT_MSRT24                     = $0082; { Microsoft Corporation }
  WAVE_FORMAT_G729A                      = $0083; { AT&T Labs, Inc. }
  WAVE_FORMAT_MVI_MVI2                   = $0084; { Motion Pixels }
  WAVE_FORMAT_DF_G726                    = $0085; { DataFusion Systems (Pty) (Ltd) }
  WAVE_FORMAT_DF_GSM610                  = $0086; { DataFusion Systems (Pty) (Ltd) }
  WAVE_FORMAT_ISIAUDIO                   = $0088; { Iterated Systems, Inc. }
  WAVE_FORMAT_ONLIVE                     = $0089; { OnLive! Technologies, Inc. }
  WAVE_FORMAT_MULTITUDE_FT_SX20          = $008A; { Multitude Inc. }
  WAVE_FORMAT_INFOCOM_ITS_G721_ADPCM     = $008B; { Infocom }
  WAVE_FORMAT_CONVEDIA_G729              = $008C; { Convedia Corp. }
  WAVE_FORMAT_CONGRUENCY                 = $008D; { Congruency Inc. }
  WAVE_FORMAT_SBC24                      = $0091; { Siemens Business Communications Sys }
  WAVE_FORMAT_DOLBY_AC3_SPDIF            = $0092; { Sonic Foundry }
  WAVE_FORMAT_MEDIASONIC_G723            = $0093; { MediaSonic }
  WAVE_FORMAT_PROSODY_8KBPS              = $0094; { Aculab plc }
  WAVE_FORMAT_ZYXEL_ADPCM                = $0097; { ZyXEL Communications, Inc. }
  WAVE_FORMAT_PHILIPS_LPCBB              = $0098; { Philips Speech Processing }
  WAVE_FORMAT_PACKED                     = $0099; { Studer Professional Audio AG }
  WAVE_FORMAT_MALDEN_PHONYTALK           = $00A0; { Malden Electronics Ltd. }
  WAVE_FORMAT_RACAL_RECORDER_GSM         = $00A1; { Racal recorders }
  WAVE_FORMAT_RACAL_RECORDER_G720_A      = $00A2; { Racal recorders }
  WAVE_FORMAT_RACAL_RECORDER_G723_1      = $00A3; { Racal recorders }
  WAVE_FORMAT_RACAL_RECORDER_TETRA_ACELP = $00A4; { Racal recorders }
  WAVE_FORMAT_NEC_AAC                    = $00B0; { NEC Corp. }
  WAVE_FORMAT_RAW_AAC1                   = $00FF; { For Raw AAC, with format block AudioSpecificConfig() (as defined by MPEG-4), that follows WAVEFORMATEX }
  WAVE_FORMAT_RHETOREX_ADPCM             = $0100; { Rhetorex Inc. }
  WAVE_FORMAT_IRAT                       = $0101; { BeCubed Software Inc. }
  WAVE_FORMAT_VIVO_G723                  = $0111; { Vivo Software }
  WAVE_FORMAT_VIVO_SIREN                 = $0112; { Vivo Software }
  WAVE_FORMAT_PHILIPS_CELP               = $0120; { Philips Speech Processing }
  WAVE_FORMAT_PHILIPS_GRUNDIG            = $0121; { Philips Speech Processing }
  WAVE_FORMAT_DIGITAL_G723               = $0123; { Digital Equipment Corporation }
  WAVE_FORMAT_SANYO_LD_ADPCM             = $0125; { Sanyo Electric Co., Ltd. }
  WAVE_FORMAT_SIPROLAB_ACEPLNET          = $0130; { Sipro Lab Telecom Inc. }
  WAVE_FORMAT_SIPROLAB_ACELP4800         = $0131; { Sipro Lab Telecom Inc. }
  WAVE_FORMAT_SIPROLAB_ACELP8V3          = $0132; { Sipro Lab Telecom Inc. }
  WAVE_FORMAT_SIPROLAB_G729              = $0133; { Sipro Lab Telecom Inc. }
  WAVE_FORMAT_SIPROLAB_G729A             = $0134; { Sipro Lab Telecom Inc. }
  WAVE_FORMAT_SIPROLAB_KELVIN            = $0135; { Sipro Lab Telecom Inc. }
  WAVE_FORMAT_VOICEAGE_AMR               = $0136; { VoiceAge Corp. }
  WAVE_FORMAT_G726ADPCM                  = $0140; { Dictaphone Corporation }
  WAVE_FORMAT_DICTAPHONE_CELP68          = $0141; { Dictaphone Corporation }
  WAVE_FORMAT_DICTAPHONE_CELP54          = $0142; { Dictaphone Corporation }
  WAVE_FORMAT_QUALCOMM_PUREVOICE         = $0150; { Qualcomm, Inc. }
  WAVE_FORMAT_QUALCOMM_HALFRATE          = $0151; { Qualcomm, Inc. }
  WAVE_FORMAT_TUBGSM                     = $0155; { Ring Zero Systems, Inc. }
  WAVE_FORMAT_MSAUDIO1                   = $0160; { Microsoft Corporation }
  WAVE_FORMAT_WMAUDIO2                   = $0161; { Microsoft Corporation }
  WAVE_FORMAT_WMAUDIO3                   = $0162; { Microsoft Corporation }
  WAVE_FORMAT_WMAUDIO_LOSSLESS           = $0163; { Microsoft Corporation }
  WAVE_FORMAT_WMASPDIF                   = $0164; { Microsoft Corporation }
  WAVE_FORMAT_UNISYS_NAP_ADPCM           = $0170; { Unisys Corp. }
  WAVE_FORMAT_UNISYS_NAP_ULAW            = $0171; { Unisys Corp. }
  WAVE_FORMAT_UNISYS_NAP_ALAW            = $0172; { Unisys Corp. }
  WAVE_FORMAT_UNISYS_NAP_16K             = $0173; { Unisys Corp. }
  WAVE_FORMAT_SYCOM_ACM_SYC008           = $0174; { SyCom Technologies }
  WAVE_FORMAT_SYCOM_ACM_SYC701_G726L     = $0175; { SyCom Technologies }
  WAVE_FORMAT_SYCOM_ACM_SYC701_CELP54    = $0176; { SyCom Technologies }
  WAVE_FORMAT_SYCOM_ACM_SYC701_CELP68    = $0177; { SyCom Technologies }
  WAVE_FORMAT_KNOWLEDGE_ADVENTURE_ADPCM  = $0178; { Knowledge Adventure, Inc. }
  WAVE_FORMAT_FRAUNHOFER_IIS_MPEG2_AAC   = $0180; { Fraunhofer IIS }
  WAVE_FORMAT_DTS_DS                     = $0190; { Digital Theatre Systems, Inc. }
  WAVE_FORMAT_CREATIVE_ADPCM             = $0200; { Creative Labs, Inc }
  WAVE_FORMAT_CREATIVE_FASTSPEECH8       = $0202; { Creative Labs, Inc }
  WAVE_FORMAT_CREATIVE_FASTSPEECH10      = $0203; { Creative Labs, Inc }
  WAVE_FORMAT_UHER_ADPCM                 = $0210; { UHER informatic GmbH }
  WAVE_FORMAT_ULEAD_DV_AUDIO             = $0215; { Ulead Systems, Inc. }
  WAVE_FORMAT_ULEAD_DV_AUDIO_1           = $0216; { Ulead Systems, Inc. }
  WAVE_FORMAT_QUARTERDECK                = $0220; { Quarterdeck Corporation }
  WAVE_FORMAT_ILINK_VC                   = $0230; { I-link Worldwide }
  WAVE_FORMAT_RAW_SPORT                  = $0240; { Aureal Semiconductor }
  WAVE_FORMAT_ESST_AC3                   = $0241; { ESS Technology, Inc. }
  WAVE_FORMAT_GENERIC_PASSTHRU           = $0249;
  WAVE_FORMAT_IPI_HSX                    = $0250; { Interactive Products, Inc. }
  WAVE_FORMAT_IPI_RPELP                  = $0251; { Interactive Products, Inc. }
  WAVE_FORMAT_CS2                        = $0260; { Consistent Software }
  WAVE_FORMAT_SONY_SCX                   = $0270; { Sony Corp. }
  WAVE_FORMAT_SONY_SCY                   = $0271; { Sony Corp. }
  WAVE_FORMAT_SONY_ATRAC3                = $0272; { Sony Corp. }
  WAVE_FORMAT_SONY_SPC                   = $0273; { Sony Corp. }
  WAVE_FORMAT_TELUM_AUDIO                = $0280; { Telum Inc. }
  WAVE_FORMAT_TELUM_IA_AUDIO             = $0281; { Telum Inc. }
  WAVE_FORMAT_NORCOM_VOICE_SYSTEMS_ADPCM = $0285; { Norcom Electronics Corp. }
  WAVE_FORMAT_FM_TOWNS_SND               = $0300; { Fujitsu Corp. }
  WAVE_FORMAT_MICRONAS                   = $0350; { Micronas Semiconductors, Inc. }
  WAVE_FORMAT_MICRONAS_CELP833           = $0351; { Micronas Semiconductors, Inc. }
  WAVE_FORMAT_BTV_DIGITAL                = $0400; { Brooktree Corporation }
  WAVE_FORMAT_INTEL_MUSIC_CODER          = $0401; { Intel Corp. }
  WAVE_FORMAT_INDEO_AUDIO                = $0402; { Ligos }
  WAVE_FORMAT_QDESIGN_MUSIC              = $0450; { QDesign Corporation }
  WAVE_FORMAT_ON2_VP7_AUDIO              = $0500; { On2 Technologies }
  WAVE_FORMAT_ON2_VP6_AUDIO              = $0501; { On2 Technologies }
  WAVE_FORMAT_VME_VMPCM                  = $0680; { AT&T Labs, Inc. }
  WAVE_FORMAT_TPC                        = $0681; { AT&T Labs, Inc. }
  WAVE_FORMAT_LIGHTWAVE_LOSSLESS         = $08AE; { Clearjump }
  WAVE_FORMAT_OLIGSM                     = $1000; { Ing C. Olivetti & C., S.p.A. }
  WAVE_FORMAT_OLIADPCM                   = $1001; { Ing C. Olivetti & C., S.p.A. }
  WAVE_FORMAT_OLICELP                    = $1002; { Ing C. Olivetti & C., S.p.A. }
  WAVE_FORMAT_OLISBC                     = $1003; { Ing C. Olivetti & C., S.p.A. }
  WAVE_FORMAT_OLIOPR                     = $1004; { Ing C. Olivetti & C., S.p.A. }
  WAVE_FORMAT_LH_CODEC                   = $1100; { Lernout & Hauspie }
  WAVE_FORMAT_LH_CODEC_CELP              = $1101; { Lernout & Hauspie }
  WAVE_FORMAT_LH_CODEC_SBC8              = $1102; { Lernout & Hauspie }
  WAVE_FORMAT_LH_CODEC_SBC12             = $1103; { Lernout & Hauspie }
  WAVE_FORMAT_LH_CODEC_SBC16             = $1104; { Lernout & Hauspie }
  WAVE_FORMAT_NORRIS                     = $1400; { Norris Communications, Inc. }
  WAVE_FORMAT_ISIAUDIO_2                 = $1401; { ISIAudio }
  WAVE_FORMAT_SOUNDSPACE_MUSICOMPRESS    = $1500; { AT&T Labs, Inc. }
  WAVE_FORMAT_MPEG_ADTS_AAC              = $1600; { Microsoft Corporation }
  WAVE_FORMAT_MPEG_RAW_AAC               = $1601; { Microsoft Corporation }
  WAVE_FORMAT_MPEG_LOAS                  = $1602; { Microsoft Corporation (MPEG-4 Audio Transport Streams (LOAS/LATM) }
  WAVE_FORMAT_NOKIA_MPEG_ADTS_AAC        = $1608; { Microsoft Corporation }
  WAVE_FORMAT_NOKIA_MPEG_RAW_AAC         = $1609; { Microsoft Corporation }
  WAVE_FORMAT_VODAFONE_MPEG_ADTS_AAC     = $160A; { Microsoft Corporation }
  WAVE_FORMAT_VODAFONE_MPEG_RAW_AAC      = $160B; { Microsoft Corporation }
  WAVE_FORMAT_MPEG_HEAAC                 = $1610; { Microsoft Corporation (MPEG-2 AAC or MPEG-4 HE-AAC v1/v2 streams with any payload (ADTS, ADIF, LOAS/LATM, RAW). Format block includes MP4 AudioSpecificConfig() -- see HEAACWAVEFORMAT below }
  WAVE_FORMAT_VOXWARE_RT24_SPEECH        = $181C; { Voxware Inc. }
  WAVE_FORMAT_SONICFOUNDRY_LOSSLESS      = $1971; { Sonic Foundry }
  WAVE_FORMAT_INNINGS_TELECOM_ADPCM      = $1979; { Innings Telecom Inc. }
  WAVE_FORMAT_LUCENT_SX8300P             = $1C07; { Lucent Technologies }
  WAVE_FORMAT_LUCENT_SX5363S             = $1C0C; { Lucent Technologies }
  WAVE_FORMAT_CUSEEME                    = $1F03; { CUSeeMe }
  WAVE_FORMAT_NTCSOFT_ALF2CM_ACM         = $1FC4; { NTCSoft }
  WAVE_FORMAT_DVM                        = $2000; { FAST Multimedia AG }
  WAVE_FORMAT_DTS2                       = $2001;
  WAVE_FORMAT_MAKEAVIS                   = $3313;
  WAVE_FORMAT_DIVIO_MPEG4_AAC            = $4143; { Divio, Inc. }
  WAVE_FORMAT_NOKIA_ADAPTIVE_MULTIRATE   = $4201; { Nokia }
  WAVE_FORMAT_DIVIO_G726                 = $4243; { Divio, Inc. }
  WAVE_FORMAT_LEAD_SPEECH                = $434C; { LEAD Technologies }
  WAVE_FORMAT_LEAD_VORBIS                = $564C; { LEAD Technologies }
  WAVE_FORMAT_WAVPACK_AUDIO              = $5756; { xiph.org }
  WAVE_FORMAT_ALAC                       = $6C61; { Apple Lossless }
  WAVE_FORMAT_OGG_VORBIS_MODE_1          = $674F; { Ogg Vorbis }
  WAVE_FORMAT_OGG_VORBIS_MODE_2          = $6750; { Ogg Vorbis }
  WAVE_FORMAT_OGG_VORBIS_MODE_3          = $6751; { Ogg Vorbis }
  WAVE_FORMAT_OGG_VORBIS_MODE_1_PLUS     = $676F; { Ogg Vorbis }
  WAVE_FORMAT_OGG_VORBIS_MODE_2_PLUS     = $6770; { Ogg Vorbis }
  WAVE_FORMAT_OGG_VORBIS_MODE_3_PLUS     = $6771; { Ogg Vorbis }
  WAVE_FORMAT_3COM_NBX                   = $7000; { 3COM Corp. }
  WAVE_FORMAT_OPUS                       = $704F; { Opus }
  WAVE_FORMAT_FAAD_AAC                   = $706D;
  WAVE_FORMAT_AMR_NB                     = $7361; { AMR Narrowband }
  WAVE_FORMAT_AMR_WB                     = $7362; { AMR Wideband }
  WAVE_FORMAT_AMR_WP                     = $7363; { AMR Wideband Plus }
  WAVE_FORMAT_GSM_AMR_CBR                = $7A21; { GSMA/3GPP }
  WAVE_FORMAT_GSM_AMR_VBR_SID            = $7A22; { GSMA/3GPP }
  WAVE_FORMAT_COMVERSE_INFOSYS_G723_1    = $A100; { Comverse Infosys }
  WAVE_FORMAT_COMVERSE_INFOSYS_AVQSBC    = $A101; { Comverse Infosys }
  WAVE_FORMAT_COMVERSE_INFOSYS_SBC       = $A102; { Comverse Infosys }
  WAVE_FORMAT_SYMBOL_G729_A              = $A103; { Symbol Technologies }
  WAVE_FORMAT_VOICEAGE_AMR_WB            = $A104; { VoiceAge Corp. }
  WAVE_FORMAT_INGENIENT_G726             = $A105; { Ingenient Technologies, Inc. }
  WAVE_FORMAT_MPEG4_AAC                  = $A106; { ISO/MPEG-4 }
  WAVE_FORMAT_ENCORE_G726                = $A107; { Encore Software }
  WAVE_FORMAT_ZOLL_ASAO                  = $A108; { ZOLL Medical Corp. }
  WAVE_FORMAT_SPEEX_VOICE                = $A109; { xiph.org }
  WAVE_FORMAT_VIANIX_MASC                = $A10A; { Vianix LLC }
  WAVE_FORMAT_WM9_SPECTRUM_ANALYZER      = $A10B; { Microsoft }
  WAVE_FORMAT_WMF_SPECTRUM_ANAYZER       = $A10C; { Microsoft }
  WAVE_FORMAT_GSM_610                    = $A10D;
  WAVE_FORMAT_GSM_620                    = $A10E;
  WAVE_FORMAT_GSM_660                    = $A10F;
  WAVE_FORMAT_GSM_690                    = $A110;
  WAVE_FORMAT_GSM_ADAPTIVE_MULTIRATE_WB  = $A111;
  WAVE_FORMAT_POLYCOM_G722               = $A112; { Polycom }
  WAVE_FORMAT_POLYCOM_G728               = $A113; { Polycom }
  WAVE_FORMAT_POLYCOM_G729_A             = $A114; { Polycom }
  WAVE_FORMAT_POLYCOM_SIREN              = $A115; { Polycom }
  WAVE_FORMAT_GLOBAL_IP_ILBC             = $A116; { Global IP }
  WAVE_FORMAT_RADIOTIME_TIME_SHIFT_RADIO = $A117; { RadioTime }
  WAVE_FORMAT_NICE_ACA                   = $A118; { Nice Systems }
  WAVE_FORMAT_NICE_ADPCM                 = $A119; { Nice Systems }
  WAVE_FORMAT_VOCORD_G721                = $A11A; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_G726                = $A11B; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_G722_1              = $A11C; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_G728                = $A11D; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_G729                = $A11E; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_G729_A              = $A11F; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_G723_1              = $A120; { Vocord Telecom }
  WAVE_FORMAT_VOCORD_LBC                 = $A121; { Vocord Telecom }
  WAVE_FORMAT_NICE_G728                  = $A122; { Nice Systems }
  WAVE_FORMAT_FRACE_TELECOM_G729         = $A123; { France Telecom }
  WAVE_FORMAT_CODIAN                     = $A124; { CODIAN }
  WAVE_FORMAT_FLAC                       = $F1AC; { flac.sourceforge.net }
  WAVE_FORMAT_EXTENSIBLE                 = $FFFE; { Microsoft }

type
  TChunkID = array [0..3] of char;
  TChunkHeader = packed record
    ID: TChunkID;
    Size: UInt32;
  end;
  TRiffHeader = packed record
    ChunkHeader: TChunkHeader;
    Format: TChunkID;
  end;
  TWaveFormat = packed record
    ChunkHeader: TChunkHeader;
    Format: UInt16;
    Channels: UInt16;
    SampleRate: UInt32;
    ByteRate: UInt32;
    BlockAlign: UInt16;
    BitsPerSample: UInt16;
  end;

implementation

end.

