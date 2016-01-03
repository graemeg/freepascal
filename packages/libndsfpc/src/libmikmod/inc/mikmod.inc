(*	MikMod sound library
	(c) 1998, 1999, 2000 Miodrag Vallat and others - see file AUTHORS
	for complete list.

	This library is free software; you can redistribute it and/or modify
	it under the terms of the GNU Library General Public License as
	published by the Free Software Foundation; either version 2 of
	the License, or (at your option) any later version.
 
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Library General Public License for more details.
 
	You should have received a copy of the GNU Library General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
	02111-1307, USA.
*)


{$ifdef NDS_INTERFACE}
(**** CONST *******************************************************************)
const
  LIBMIKMOD_VERSION_MAJOR = 3;
  LIBMIKMOD_VERSION_MINOR = 1;
  LIBMIKMOD_REVISION      = 10;

  LIBMIKMOD_VERSION = (
                        (LIBMIKMOD_VERSION_MAJOR shl 16) or
                        (LIBMIKMOD_VERSION_MINOR shl 8) or
                        (LIBMIKMOD_REVISION)
                      );

  SFX_CRITICAL    = 1;

  SF_16BITS       = $0001;
  SF_STEREO       = $0002;
  SF_SIGNED       = $0004;
  SF_BIG_ENDIAN   = $0008;
  SF_DELTA        = $0010;
  SF_ITPACKED		  = $0020;

  SF_FORMATMASK	  = $003F;

  SF_LOOP         = $0100;
  SF_BIDI         = $0200;
  SF_REVERSE      = $0400;
  SF_SUSTAIN      = $0800;

  SF_PLAYBACKMASK	= $0C00;

  SF_OWNPAN            = $1000;
  SF_UST_LOOP          = $2000;

  SF_EXTRAPLAYBACKMASK = $3000;

  PAN_LEFT      = 0;
  PAN_HALFLEFT  = 64;
  PAN_CENTER    = 128;
  PAN_HALFRIGHT = 192;
  PAN_RIGHT     = 255;
  PAN_SURROUND  = 512; 

  INSTNOTES = 120;

  ENVPOINTS = 32;

  UF_MAXCHAN = 64;

  UF_XMPERIODS  = $0001; 
  UF_LINEAR     = $0002; 
  UF_INST       = $0004; 
  UF_NNA        = $0008; 
  UF_S3MSLIDES  = $0010; 
  UF_BGSLIDES   = $0020; 
  UF_HIGHBPM    = $0040; 
  UF_NOWRAP     = $0080; 
  UF_ARPMEM     = $0100; 
  UF_FT2QUIRKS  = $0200;
  UF_PANNING    = $0400;
  
  MUTE_EXCLUSIVE = 32000;
  MUTE_INCLUSIVE = 32001;

  DMODE_16BITS     = $0001; 
  DMODE_STEREO     = $0002; 
  DMODE_SOFT_SNDFX = $0004; 
  DMODE_SOFT_MUSIC = $0008; 
  DMODE_HQMIXER    = $0010; 
  DMODE_SURROUND   = $0100; 
  DMODE_INTERP     = $0200;
  DMODE_REVERSE    = $0400;

  OCTAVE            = 12;

  UF_MAXMACRO       = $10;
  UF_MAXFILTER      = $100;

  FILT_CUT          = $80;
  FILT_RESONANT     = $81;

  IF_OWNPAN         = 1;
  IF_PITCHPAN       = 2;

  EF_ON             = 1;
  EF_SUSTAIN        = 2;
  EF_LOOP           = 4;
  EF_VOLENV         = 8;

  NNA_CUT           = 0;
  NNA_CONTINUE      = 1;
  NNA_OFF           = 2;
  NNA_FADE          = 3;

  NNA_MASK          = 3;

  DCT_OFF           = 0;
  DCT_NOTE          = 1;
  DCT_SAMPLE        = 2;
  DCT_INST          = 3;

  DCA_CUT           = 0;
  DCA_OFF           = 1;
  DCA_FADE          = 2;

  KEY_KICK          = 0;
  KEY_OFF           = 1;
  KEY_FADE          = 2;
  KEY_KILL          = (KEY_OFF or KEY_FADE);

  KICK_ABSENT       = 0;
  KICK_NOTE         = 1;
  KICK_KEYOFF       = 2;
  KICK_ENV          = 4;

  AV_IT             = 1;

  POS_NONE          = (-2);             

  LAST_PATTERN      = word(-1);         

  S3MIT_OLDSTYLE = 1;	
  S3MIT_IT       = 2;	
  S3MIT_SCREAM   = 4;	

  MAXSAMPLEHANDLES  = 384;


(****** Types *****************************************************************)
type
  SBYTE   = cschar;
  PSBYTE  = ^cschar;
  UBYTE   = cuchar;
  PUBYTE  = ^cuchar;
  SWORD   = csshort;
  PSWORD  = ^csshort;
  UWORD   = cushort;
  PUWORD  = ^cushort;
  SLONG   = csint;
  PSLONG  = ^csint;
  ULONG   = cuint;
  PULONG  = ^cuint;
  BOOL    = cint;

  MikMod_Errors = (
    MMERR_OPENING_FILE = 1,
    MMERR_OUT_OF_MEMORY,
    MMERR_DYNAMIC_LINKING,

    MMERR_SAMPLE_TOO_BIG,
    MMERR_OUT_OF_HANDLES,
    MMERR_UNKNOWN_WAVE_TYPE,

    MMERR_LOADING_PATTERN,
    MMERR_LOADING_TRACK,
    MMERR_LOADING_HEADER,
    MMERR_LOADING_SAMPLEINFO,
    MMERR_NOT_A_MODULE,
    MMERR_NOT_A_STREAM,
    MMERR_MED_SYNTHSAMPLES,
    MMERR_ITPACK_INVALID_DATA,

    MMERR_DETECTING_DEVICE,
    MMERR_INVALID_DEVICE,
    MMERR_INITIALIZING_MIXER,
    MMERR_OPENING_AUDIO,
    MMERR_8BIT_ONLY,
    MMERR_16BIT_ONLY,
    MMERR_STEREO_ONLY,
    MMERR_ULAW,
    MMERR_NON_BLOCK,

    MMERR_AF_AUDIO_PORT,

    MMERR_AIX_CONFIG_INIT,
    MMERR_AIX_CONFIG_CONTROL,
    MMERR_AIX_CONFIG_START,

    MMERR_GUS_SETTINGS,
    MMERR_GUS_RESET,
    MMERR_GUS_TIMER,

    MMERR_HP_SETSAMPLESIZE,
    MMERR_HP_SETSPEED,
    MMERR_HP_CHANNELS,
    MMERR_HP_AUDIO_OUTPUT,
    MMERR_HP_AUDIO_DESC,
    MMERR_HP_BUFFERSIZE,

    MMERR_OSS_SETFRAGMENT,
    MMERR_OSS_SETSAMPLESIZE,
    MMERR_OSS_SETSTEREO,
    MMERR_OSS_SETSPEED,

    MMERR_SGI_SPEED,
    MMERR_SGI_16BIT,
    MMERR_SGI_8BIT,
    MMERR_SGI_STEREO,
    MMERR_SGI_MONO,

    MMERR_SUN_INIT,

    MMERR_OS2_MIXSETUP,
    MMERR_OS2_SEMAPHORE,
    MMERR_OS2_TIMER,
    MMERR_OS2_THREAD,

    MMERR_DS_PRIORITY,
    MMERR_DS_BUFFER,
    MMERR_DS_FORMAT,
    MMERR_DS_NOTIFY,
    MMERR_DS_EVENT,
    MMERR_DS_THREAD,
    MMERR_DS_UPDATE,

    MMERR_WINMM_HANDLE,
    MMERR_WINMM_ALLOCATED,
    MMERR_WINMM_DEVICEID,
    MMERR_WINMM_FORMAT,
    MMERR_WINMM_UNKNOWN,

    MMERR_MAC_SPEED,
    MMERR_MAC_START,

    MMERR_MAX
  );
    
  SAMPLE = packed record 
    panning: csshort;
    speed: culong;
    volume: cuchar;
    inflags: cushort;
    flags: cushort;  
    length: cuint;   
    loopstart: cuint;
    loopend: cuint;  
    susbegin: cuint; 
    susend: cuint;   
    globvol: cuchar;
    vibflags: cuchar;
    vibtype: cuchar; 
    vibsweep: cuchar;
    vibdepth: cuchar;
    vibrate: cuchar;
    samplename: pcchar; 
    avibpos: cushort; 
    divfactor: cuchar;
    seekpos: cuint;   
    handle: csshort;  
  end;
  TSample = SAMPLE;
  PSample = ^SAMPLE;

  ENVPT = packed record 
    pos: csshort;
    val: csshort;
  end;
  TENVPT = ENVPT;
  PENVPT = ^ENVPT;

  INSTRUMENT = packed record
    insname: pcchar;
    flags: cuchar;
    samplenumber: array [0..INSTNOTES - 1] of cushort;
    samplenote: array [0..INSTNOTES - 1] of cuchar;
    nnatype: cuchar;
    dca: cuchar;
    dct: cuchar;
    globvol: cuchar;
    volfade: cushort;
    panning: csshort;
    pitpansep: cuchar;
    pitpancenter: cuchar;
    rvolvar: cuchar;
    rpanvar: cuchar;
    volflg: cuchar;
    volpts: cuchar;
    volsusbeg: cuchar;
    volsusend: cuchar;
    volbeg: cuchar;
    volend: cuchar;
    volenv: array [0..ENVPOINTS - 1] of ENVPT;
    panflg: cuchar;
    panpts: cuchar;
    pansusbeg: cuchar;
    pansusend: cuchar;
    panbeg: cuchar;
    panend: cuchar;
    panenv: array [0..ENVPOINTS - 1] of ENVPT;
    pitflg: cuchar;
    pitpts: cuchar;
    pitsusbeg: cuchar;
    pitsusend: cuchar;
    pitbeg: cuchar;
    pitend: cuchar;
    pitenv: array [0..ENVPOINTS - 1] of ENVPT;
  end;
  TInstrument = INSTRUMENT;
  PInstrument = ^INSTRUMENT;

  MP_CHANNEL = packed record
    i: PInstrument;
    s: PSample;
    sample: cuchar;
    note: cuchar;
    outvolume: csshort;
    chanvol: cschar;
    fadevol: cushort;
    panning: csshort;
    kick: cuchar;
    period: cushort;
    nna: cuchar;
    volflg: cuchar;
    panflg: cuchar;
    pitflg: cuchar;
    keyoff: cuchar;
    handle: csshort;
    notedelay: cuchar;
    start: csint;
  end;
  TMPChannel = MP_CHANNEL;
  PMPChannel = ^MP_CHANNEL;
  
  ENVPR = packed record
    flg: cuchar;
    pts: cuchar;
    susbeg: cuchar;
    susend: cuchar;
    beg: cuchar;
    _end: cuchar;
    p: csshort;
    a: cushort;
    b: cushort;
    env: PENVPT;
  end;
  TENVPR = ENVPR;
  PENVPR = ^ENVPR;


  PMP_VOICE = ^MP_VOICE;

  MP_CONTROL = packed record
    main: MP_CHANNEL;
    slave: PMP_VOICE;
    slavechn: cuchar;
    muted: cuchar;
    ultoffset: cushort;
    anote: cuchar;
    oldnote: cuchar;
    ownper: csshort;
    ownvol: csshort;
    dca: cuchar;
    dct: cuchar;
    row: pcuchar;
    retrig: cschar;
    speed: cuint;
    volume: csshort;
    tmpvolume: csshort;
    tmpperiod: cushort;
    wantedperiod: cushort;
    arpmem: cuchar;
    pansspd: cuchar;
    slidespeed: cushort;
    portspeed: cushort;
    s3mtremor: cuchar;
    s3mtronof: cuchar;
    s3mvolslide: cuchar;
    sliding: cschar;
    s3mrtgspeed: cuchar;
    s3mrtgslide: cuchar;
    glissando: cuchar;
    wavecontrol: cuchar;
    vibpos: cschar;
    vibspd: cuchar;
    vibdepth: cuchar;
    trmpos: cschar;
    trmspd: cuchar;
    trmdepth: cuchar;
    fslideupspd: cuchar;
    fslidednspd: cuchar;
    fportupspd: cuchar;
    fportdnspd: cuchar;
    ffportupspd: cuchar;
    ffportdnspd: cuchar;
    hioffset: cuint;
    soffset: cushort;
    sseffect: cuchar;
    ssdata: cuchar;
    chanvolslide: cuchar;
    panbwave: cuchar;
    panbpos: cuchar;
    panbspd: cschar;
    panbdepth: cuchar;
    newsamp: cushort;
    voleffect: cuchar;
    voldata: cuchar;
    pat_reppos: csshort;
    pat_repcnt: cushort;
  end;
  TMPControl = MP_CONTROL;
  PMPControl = ^MP_CONTROL;

  MP_VOICE = packed record
    main: MP_CHANNEL;
    venv: ENVPR;
    penv: ENVPR;
    cenv: ENVPR;
    avibpos: cushort;
    aswppos: cushort;
    totalvol: cuint;
    mflag: cint;
    masterchn: csshort;
    masterperiod: cushort;
    master: PMPControl;
  end;
  TMPVoice = MP_VOICE;
  PMPVoice = ^MP_VOICE;

  ppcuchar = ^pcuchar;

  MODULE = packed record
    songname: pcchar;
    modtype: pcchar;
    comment: pcchar;
    flags: cushort;
    numchn: cuchar;
    numvoices: cuchar;
    numpos: cushort;
    numpat: cushort;
    numins: cushort;
    numsmp: cushort;
    instruments: PInstrument;
    samples: PSample;
    realchn: cuchar;
    totalchn: cuchar;
    reppos: cushort;
    initspeed: cuchar;
    inittempo: cushort;
    initvolume: cuchar;
    panning: array [0..UF_MAXCHAN - 1] of cushort;
    chanvol: array [0..UF_MAXCHAN - 1] of cuchar;
    bpm: cushort;
    sngspd: cushort;
    volume: csshort;
    extspd: cint;
    panflag: cint;
    wrap: cint;
    loop: cint;
    fadeout: cint;
    patpos: cushort;
    sngpos: csshort;
    sngtime: cuint;
    relspd: csshort;
    numtrk: cushort;
    tracks: ppcuchar;
    patterns: pcushort;
    pattrows: pcushort;
    positions: pcushort;
    forbid: cint;
    numrow: cushort;
    vbtick: cushort;
    sngremainder: cushort;
    control: PMPControl;
    voice: PMPVoice;
    globalslide: cuchar;
    pat_repcrazy: cuchar;
    patbrk: cushort;
    patdly: cuchar;
    patdly2: cuchar;
    posjmp: csshort;
    bpmlimit: cushort;
  end;
  TModule = MODULE;
  PModule = ^MODULE;

  drvenum1 = (
    MD_MUSIC = 0,
    MD_SNDFX
  );

  drvenum2 = (
    MD_HARDWARE = 0,
    MD_SOFTWARE
  );

  PMReader = ^MREADER;
  TSeek1 = function(value1: PMReader; value2: clong; value3: cint): pcint;
  PSeek1 = ^TSeek1; 
  TTell1 = function(value: PMReader): pclong;
  PTell1 = ^TTell1;
  TRead = function(value1: PMReader; value2: pointer; value3: cint): pcint;
  PRead = ^TRead;
  TGet = function(value: PMReader): pcint;
  PGet = ^TGet;
  TEof = function(value: PMReader): pcint;
  PEof = ^TEof;

  MREADER = packed record
    Seek: TSeek1;
    Tell: TTell1;
    Read: TRead;
    Get: TGet;
    Eof: TEof;
  end;

  PMWriter = ^MWRITER;
  TSeek2 = function(value1: PMWriter; value2: clong; value3: cint): pcint;
  PSeek2 = ^TSeek2;
  TTell2 = function(value: PMWriter): pclong;
  PTell2 = ^TTell2;
  TWrite = function(value1: PMWriter; value2: pointer; value3: cint): pcint;
  PWrite = ^TWrite;
  TPut = function(value1: PMWriter; value2: cint): pcint;
  PPut = ^TPut;
  
  MWRITER = packed record
    Seek: TSeek2;
    Tell: TTell2;
    Write: TWrite;
    Put: TPut;
  end;
  TMWriter = MWRITER;

  PSampLoad = ^SAMPLOAD;
  PPSampLoad  = ^PSampLoad;
  SAMPLOAD = packed record
    next: PSampLoad;
    length: cuint;
    loopstart: cuint;
    loopend: cuint;
    infmt, outfmt: cushort;
    scalefactor: cint;
    sample: PSample;
    reader: PMReader;
  end;
  TSampLoad = SAMPLOAD;

  TMDCommandLine = procedure(value: pcchar);
  PMDCommandLine = ^TMDCommandLine;
  TMDIsPresent = function(): cint;
  PMDIsPresent = ^TMDIsPresent;
  TMDSampleLoad = function (value1: PSAMPLOAD; value2: cint): csshort;
  PMDSampleLoad = ^TMDSampleLoad;  
  TMDSampleUnload = procedure(value: csshort);
  PMDSampleUnload = ^TMDSampleUnload;
  TMDFreeSampleSpace = function(value: cint): cuint;
  PMDFreeSampleSpace = ^TMDFreeSampleSpace;
  TMDRealSampleLength = function(value1: cint; value2: PSAMPLE): cuint;
  PMDRealSampleLength = ^TMDRealSampleLength;
  TMDInit = function(): cint;
  PMDInit = ^TMDInit;
  TMDExit = procedure();
  PMDExit = ^TMDExit;
  TMDReset = function(): cint;
  PMDReset = ^TMDReset;
  TMDSetNumVoices = function(): cint;
  PMDSetNumVoices = ^TMDSetNumVoices;
  TMDPlayStart = function(): cint;
  PMDPlayStart = ^TMDPlayStart;
  TMDPlayStop = procedure();
  PMDPlayStop = ^TMDPlayStop;
  TMDUpdate = procedure();
  PMDUpdate = ^TMDUpdate;
  TMDPause = procedure();
  PMDPause = ^TMDPause;
  TMDVoiceSetVolume = procedure(value1: cuchar; value2: cushort);
  PMDVoiceSetVolume = ^TMDVoiceSetVolume;
  TMDVoiceGetVolume = function(value: cuchar): cushort;
  PMDVoiceGetVolume = ^TMDVoiceGetVolume;
  TMDVoiceSetFrequency = procedure(value1: cuchar; value2: cuint);
  PMDVoiceSetFrequency = ^TMDVoiceSetFrequency;
  TMDVoiceGetFrequency = function(value: cuchar): cuint;
  PMDVoiceGetFrequency = ^TMDVoiceGetFrequency;
  TMDVoiceSetPanning = procedure(value1: cuchar; value2: cuint);
  PMDVoiceSetPanning = ^TMDVoiceSetPanning;
  TMDVoiceGetPanning = function(value: cuchar): cuint;
  PMDVoiceGetPanning = ^TMDVoiceGetPanning;
  TMDVoicePlay = procedure(value1: cuchar; value2: csshort; value3, value4, value5, value6: cuint; value7: cushort);
  PMDVoicePlay = ^TMDVoicePlay;
  TMDVoiceStop = procedure(value: cuchar);
  PMDVoiceStop = ^TMDVoiceStop;
  TMDVoiceStopped = function(value: cuchar): cint;
  PMDVoiceStopped = ^TMDVoiceStopped;
  TMDVoiceGetPosition = function(value: cuchar): csint;
  PMDVoiceGetPosition = ^TMDVoiceGetPosition;
  TMDVoiceRealVolume = function(value: cuchar): cuint;
  PMDVoiceRealVolume = ^TMDVoiceRealVolume;

  PMDriver = ^MDRIVER;
  MDRIVER = packed record
    next: PMdriver;
    Name: pcchar;
    Version: pcchar;
    HardVoiceLimit: cuchar;
    SoftVoiceLimit: cuchar;
    Alias: pcchar;
    CommandLine: TMDCommandLine;
    IsPresent: TMDIsPresent;
    SampleLoad: TMDSampleLoad;
    SampleUnload: TMDSampleUnload;
    FreeSampleSpace: TMDFreeSampleSpace;
    RealSampleLength: TMDRealSampleLength;
    Init: TMDInit;
    Exit: TMDExit;
    Reset: TMDReset;
    SetNumVoices: TMDSetNumVoices;
    PlayStart: TMDPlayStart;
    PlayStop: TMDPlayStop;
    Update: TMDUpdate;
    Pause: TMDPause;
    VoiceSetVolume: TMDVoiceSetVolume;
    VoiceGetVolume: TMDVoiceGetVolume;
    VoiceSetFrequency: TMDVoiceSetFrequency;
    VoiceGetFrequency: TMDVoiceGetFrequency;
    VoiceSetPanning: TMDVoiceSetPanning;
    VoiceGetPanning: TMDVoiceGetPanning;
    VoicePlay: TMDVoicePlay;
    VoiceStop: TMDVoiceStop;
    VoiceStopped: TMDVoiceStopped;
    VoiceGetPosition: TMDVoiceGetPosition;
    VoiceRealVolume: TMDVoiceRealVolume;
  end;
  TMDriver = MDRIVER;

  MikMod_player = procedure();
  TMikModPlayer = MikMod_player;  
  MikMod_player_t = ^MikMod_player;
  PMikModPlayer = ^MikMod_player;

  uni_fx = (
    UNI_NOTE = 1,
    UNI_INSTRUMENT,
    UNI_PTEFFECT0,                      // arpeggio 
    UNI_PTEFFECT1,                      // porta up 
    UNI_PTEFFECT2,                      // porta down 
    UNI_PTEFFECT3,                      // porta to note 
    UNI_PTEFFECT4,                      // vibrato 
    UNI_PTEFFECT5,                      // dual effect 3+A 
    UNI_PTEFFECT6,                      // dual effect 4+A 
    UNI_PTEFFECT7,                      // tremolo 
    UNI_PTEFFECT8,                      // pan 
    UNI_PTEFFECT9,                      // sample offset 
    UNI_PTEFFECTA,                      // volume slide 
    UNI_PTEFFECTB,                      // pattern jump 
    UNI_PTEFFECTC,                      // set volume 
    UNI_PTEFFECTD,                      // pattern break 
    UNI_PTEFFECTE,                      // extended effects 
    UNI_PTEFFECTF,                      // set speed 
    UNI_S3MEFFECTA,                     // set speed 
    UNI_S3MEFFECTD,                     // volume slide 
    UNI_S3MEFFECTE,                     // porta down 
    UNI_S3MEFFECTF,                     // porta up 
    UNI_S3MEFFECTI,                     // tremor 
    UNI_S3MEFFECTQ,                     // retrig 
    UNI_S3MEFFECTR,                     // tremolo 
    UNI_S3MEFFECTT,                     // set tempo 
    UNI_S3MEFFECTU,                     // fine vibrato 
    UNI_KEYOFF,                         // note off 
    UNI_KEYFADE,                        // note fade 
    UNI_VOLEFFECTS,                     // volume column effects 
    UNI_XMEFFECT4,                      // vibrato 
    UNI_XMEFFECT6,                      // dual effect 4+A 
    UNI_XMEFFECTA,                      // volume slide 
    UNI_XMEFFECTE1,                     // fine porta up 
    UNI_XMEFFECTE2,                     // fine porta down 
    UNI_XMEFFECTEA,                     // fine volume slide up 
    UNI_XMEFFECTEB,                     // fine volume slide down 
    UNI_XMEFFECTG,                      // set global volume 
    UNI_XMEFFECTH,                      // global volume slide 
    UNI_XMEFFECTL,                      // set envelope position 
    UNI_XMEFFECTP,                      // pan slide 
    UNI_XMEFFECTX1,                     // extra fine porta up 
    UNI_XMEFFECTX2,                     // extra fine porta down 
    UNI_ITEFFECTG,                      // porta to note 
    UNI_ITEFFECTH,                      // vibrato 
    UNI_ITEFFECTI,                      // tremor (xy not incremented) 
    UNI_ITEFFECTM,                      // set channel volume 
    UNI_ITEFFECTN,                      // slide / fineslide channel volume 
    UNI_ITEFFECTP,                      // slide / fineslide channel panning 
    UNI_ITEFFECTT,                      // slide tempo 
    UNI_ITEFFECTU,                      // fine vibrato 
    UNI_ITEFFECTW,                      // slide / fineslide global volume 
    UNI_ITEFFECTY,                      // panbrello 
    UNI_ITEFFECTZ,                      // resonant filters 
    UNI_ITEFFECTS0,
    UNI_ULTEFFECT9,                     // Sample fine offset 
    UNI_MEDSPEED,
    UNI_MEDEFFECTF1,                    // play note twice 
    UNI_MEDEFFECTF2,                    // delay note 
    UNI_MEDEFFECTF3,                    // play note three times 
    UNI_OKTARP,                         // arpeggio 
    UNI_LAST
  );

  extsound_fx = (
    SS_GLISSANDO = 1,
    SS_FINETUNE,
    SS_VIBWAVE,
    SS_TREMWAVE,
    SS_PANWAVE,
    SS_FRAMEDELAY,
    SS_S7EFFECTS,
    SS_PANNING,
    SS_SURROUND,
    SS_HIOFFSET,
    SS_PATLOOP,
    SS_NOTECUT,
    SS_NOTEDELAY,
    SS_PATDELAY
  );

  itcol_fx = (
    VOL_VOLUME = 1,
    VOL_PANNING,
    VOL_VOLSLIDE,
    VOL_PITCHSLIDEDN,
    VOL_PITCHSLIDEUP,
    VOL_PORTAMENTO,
    VOL_VIBRATO
  );

  FILTER = packed record
    filter, inf: cuchar;
  end;
  TFilter = FILTER;
  PFilter = ^FILTER;

  TMLInit = function(): cint;
  PMLInit = ^TMLInit;
  TMLTest = function(): cint;
  PMLTest = ^TMLTest;
  TMLLoad = function(value: cint): cint;
  PMLLoad = ^TMLLoad;
  TMLCleanup = procedure();
  PMLCleanup = ^TMLCleanup;
  TMLLoadTitle = function(): pcchar;
  PMLLoadTitle = ^TMLLoadTitle;

  PMLoader = ^MLOADER;
  MLOADER = packed record
    next: PMLoader;
    _type: pcchar;
    version: pcchar;
    Init: TMLInit;
    Test: TMLTest;
    Load: TMLLoad;
    Cleanup: TMLCleanup;
    LoadTitle: TMLLoadTitle;
  end;
  TMLoader = MLOADER;

  MikMod_handler = procedure;
  MikMod_handler_t = ^MikMod_handler;
  TMikModHandler = MikMod_handler;
  PMikModHandler = ^TMikModHandler;


var
  _mm_errno         : MikMod_Errors;
  _mm_critical      : cint;
  _mm_errorhandler  : MikMod_handler_t; cvar; external;  

  unioperands: array [0.. integer(UNI_LAST) - 1] of cushort; cvar; external; 

  MikMod_errno: cint; cvar; external;
  MikMod_critical: cint; cvar; external;

  load_669: MLOADER; cvar; external; (* 669 and Extended-669 (by Tran/Renaissance) *)
  load_amf: MLOADER; cvar; external; (* DMP Advanced Module Format (by Otto Chrons) *)
  load_dsm: MLOADER; cvar; external; (* DSIK internal module format *)
  load_far: MLOADER; cvar; external; (* Farandole Composer (by Daniel Potter) *)
  load_gdm: MLOADER; cvar; external; (* General DigiMusic (by Edward Schlunder) *)
  load_it: MLOADER; cvar; external;  (* Impulse Tracker (by Jeffrey Lim) *)
  load_imf: MLOADER; cvar; external; (* Imago Orpheus (by Lutz Roeder) *)
  load_med: MLOADER; cvar; external; (* Amiga MED modules (by Teijo Kinnunen) *)
  load_m15: MLOADER; cvar; external; (* Soundtracker 15-instrument *)
  load_mod: MLOADER; cvar; external; (* Standard 31-instrument Module loader *)
  load_mtm: MLOADER; cvar; external; (* Multi-Tracker Module (by Renaissance) *)
  load_okt: MLOADER; cvar; external; (* Amiga Oktalyzer *)
  load_stm: MLOADER; cvar; external; (* ScreamTracker 2 (by Future Crew) *)
  load_stx: MLOADER; cvar; external; (* STMIK 0.2 (by Future Crew) *)
  load_s3m: MLOADER; cvar; external; (* ScreamTracker 3 (by Future Crew) *)
  load_ult: MLOADER; cvar; external; (* UltraTracker (by MAS) *)
  load_uni: MLOADER; cvar; external; (* MikMod and APlayer internal module format *)
  load_xm: MLOADER; cvar; external;  (* FastTracker 2 (by Triton) *)

  md_volume: cuchar; cvar; external;      (* global sound volume (0-128) *)
  md_musicvolume: cuchar; cvar; external;  (* volume of song *)
  md_sndfxvolume: cuchar; cvar; external;  (* volume of sound effects *)
  md_reverb: cuchar; cvar; external;       (* 0 = none;  15 = chaos *)
  md_pansep: cuchar; cvar; external;       (* 0 = mono;  128 == 100% (full left/right) *)

  md_device: cushort; cvar; external;      (* device *)
  md_mixfreq: cushort; cvar; external;     (* mixing frequency *)
  md_mode: cushort; cvar; external;        (* mode. See DMODE_? flags above *)

  md_driver: PMDRIVER; cvar; external;     (* Current driver in use. *)

  drv_nos: MDRIVER; cvar; external;       (* no sound *)
  drv_nds_sw: MDRIVER; cvar; external;    (* Nintendo DS Software driver *)
  drv_nds_hw: MDRIVER; cvar; external;    (* Nintendo DS Hardware driver *)


{$ifdef ARM9}
var
  md_bpm: cushort; cvar;
procedure MikMod9_SendCommand(command: cuint); cdecl; {forward;}external; 
function Player_LoadMemory(const buffer: pointer; size: cint; maxchan: cint; curious: cint): PModule; cdecl; external;
{$endif ARM9}

{$ifdef ARM7}
procedure MikMod7_ProcessCommand(command: cuint); cdecl; external;
{$endif ARM7}


function MikMod_GetVersion(): clong; cdecl; external;

function _mm_malloc(value: cint): pointer; cdecl; external;
function _mm_calloc(value1, value2: cint): pointer; cdecl; external;

procedure _mm_free(var p: pointer); inline;

function Player_Init(value: PModule): cint; cdecl; external;
procedure Player_Exit(value: PModule); cdecl; external;
procedure Player_HandleTick(); cdecl; external;

function _mm_new_file_reader(fp: P_FILE): PMReader; cdecl; external;
procedure _mm_delete_file_reader(value: PMReader); cdecl; external;

function _mm_new_file_writer(fp: P_FILE): PMWriter; cdecl; external;
procedure _mm_delete_file_writer(value: PMWriter); cdecl; external;

function _mm_FileExists(fname: pcchar): cint; cdecl; external;




procedure _mm_iobase_setcur(value: PMReader); cdecl; external;
procedure _mm_iobase_revert(); cdecl; external;
function _mm_fopen(value1, value2: pcchar): P_FILE; cdecl; external;
function _mm_fclose(value: P_FILE): cint; cdecl; external;
procedure _mm_write_string(value1: pcchar; value2: PMWriter); cdecl; external;
function _mm_read_string (value1: pcchar; value2: cint; value3: PMReader): cint; cdecl; external;

function _mm_read_M_SWORD(value: PMReader): csshort; cdecl; external;
function _mm_read_I_SWORD(value: PMReader): csshort; cdecl; external;
function _mm_read_M_UWORD(value: PMReader): cushort; cdecl; external;
function _mm_read_I_UWORD(value: PMReader): cushort; cdecl; external;

function _mm_read_M_SLONG(value: PMReader): csint; cdecl; external;
function _mm_read_I_SLONG(value: PMReader): csint; cdecl; external;
function _mm_read_M_ULONG(value: PMReader): cuint; cdecl; external;
function _mm_read_I_ULONG(value: PMReader): cuint; cdecl; external;

function _mm_read_M_SWORDS(value1: pcsshort; value2: cint; value3: PMReader): cint; cdecl; external;
function _mm_read_I_SWORDS(value1: pcsshort; value2: cint; value3: PMReader): cint; cdecl; external;
function _mm_read_M_UWORDS(value1: pcushort; value2: cint; value3: PMReader): cint; cdecl; external;
function _mm_read_I_UWORDS(value1: pcushort; value2: cint; value3: PMReader): cint; cdecl; external;

function _mm_read_M_SLONGS(value1: pcsint; value2: cint; value3: PMReader): cint; cdecl; external;
function _mm_read_I_SLONGS(value1: pcsint; value2: cint; value3: PMReader): cint; cdecl; external;
function _mm_read_M_ULONGS(value1: pcuint; value2: cint; value3: PMReader): cint; cdecl; external;
function _mm_read_I_ULONGS(value1: pcuint; value2: cint; value3: PMReader): cint; cdecl; external;

procedure _mm_write_M_SWORD(value1: csshort; value2: PMWriter); cdecl; external;
procedure _mm_write_I_SWORD(value1: csshort; value2: PMWriter); cdecl; external;
procedure _mm_write_M_UWORD(value1: cushort; value2: PMWriter); cdecl; external;
procedure _mm_write_I_UWORD(value1: cushort; value2: PMWriter); cdecl; external;

procedure _mm_write_M_SLONG(value1: csint; value2: PMWriter); cdecl; external;
procedure _mm_write_I_SLONG(value1: csint; value2: PMWriter); cdecl; external;
procedure _mm_write_M_ULONG(value1: cuint; value2: PMWriter); cdecl; external;
procedure _mm_write_I_ULONG(value1: cuint; value2: PMWriter); cdecl; external;

procedure _mm_write_M_SWORDS(value1: pcsshort; value2: cint; value3: PMWriter); cdecl; external;
procedure _mm_write_I_SWORDS(value1: pcsshort; value2: cint; value3: PMWriter); cdecl; external;
procedure _mm_write_M_UWORDS(value1: pcushort; value2: cint; value3: PMWriter); cdecl; external;
procedure _mm_write_I_UWORDS(value1: pcushort; value2: cint; value3: PMWriter); cdecl; external;

procedure _mm_write_M_SLONGS(value1: pcsint; value2: cint; value3: PMWriter); cdecl; external;
procedure _mm_write_I_SLONGS(value1: pcsint; value2: cint; value3: PMWriter); cdecl; external;
procedure _mm_write_M_ULONGS(value1: pcuint; value2: cint; value3: PMWriter); cdecl; external;
procedure _mm_write_I_ULONGS(value1: pcuint; value2: cint; value3: PMWriter); cdecl; external;


function MikMod_strerror(value: cint): pcchar; cdecl; external;
function MikMod_RegisterErrorHandler(value: MikMod_handler_t): MikMod_handler_t; cdecl; external;

procedure MikMod_RegisterAllDrivers(); cdecl; external;

function MikMod_InfoDriver(): pcchar; cdecl; external;
procedure MikMod_RegisterDriver(value: PMDriver); cdecl; external;
function MikMod_DriverFromAlias(value: pcint): cint; cdecl; external;

function MikMod_Init(value: string): cint; cdecl; external;
procedure MikMod_Exit(); cdecl; external;
function MikMod_Reset(value: pcint): cint; cdecl; external;
function MikMod_SetNumVoices(value1, value2: cint): cint; cdecl; external;
function MikMod_Active(): cint; cdecl; external;
function MikMod_EnableOutput(): cint; cdecl; external;
procedure MikMod_DisableOutput(); cdecl; external;
procedure MikMod_Update(); cdecl; external;

function MikMod_InitThreads(): cint; cdecl; external;
procedure MikMod_Lock(); cdecl; external;
procedure MikMod_Unlock(); cdecl; external;

function Sample_Load(value: pcchar): PSAMPLE; cdecl; external;
function Sample_LoadFP(value: P_FILE): PSAMPLE; cdecl; external;
function Sample_LoadGeneric(value: PMREADER): PSAMPLE; cdecl; external;
procedure Sample_Free(value: PSAMPLE); cdecl; external;
function Sample_Play(value1: PSAMPLE; value2: cuint; value3: cuchar): cschar; cdecl; external;

procedure Voice_SetVolume(value1: cschar; value2: cushort); cdecl; external;
function Voice_GetVolume(value: cschar): cushort; cdecl; external;
procedure Voice_SetFrequency(value1: cschar; value2: cuint); cdecl; external;
function Voice_GetFrequency(value: cschar): cuint; cdecl; external;
procedure Voice_SetPanning(value1: cschar; value2: cuint); cdecl; external;
function Voice_GetPanning(value: cschar): cuint; cdecl; external;
procedure Voice_Play(value1: cschar; value2: PSAMPLE; value3: cuint); cdecl; external;
procedure Voice_Stop(value: cschar); cdecl; external;
function Voice_Stopped(value: cschar): cint; cdecl; external;
function Voice_GetPosition(value: cschar): csint; cdecl; external;
function Voice_RealVolume(value: cschar): cuint; cdecl; external;
 
function MikMod_InfoLoader(): pcchar; cdecl; external;
procedure MikMod_RegisterAllLoaders(); cdecl; external;
procedure MikMod_RegisterLoader(value: PMLOADER); cdecl; external;

function Player_Load(value1: pcchar; value2: cint; value3: cint): PMODULE; cdecl; external;
function Player_LoadFP(value1: P_FILE; value2: cint; value3: cint): PMODULE; cdecl; external;
function Player_LoadGeneric(value1: PMREADER; value2: cint; value3: cint): PMODULE; cdecl; external;
function Player_LoadTitle(value: pcchar): pcchar; cdecl; external;
function Player_LoadTitleFP(value: P_FILE): pcchar; cdecl; external;
procedure Player_Free(value: PMODULE); cdecl; external;
procedure Player_Start(value: PMODULE); cdecl; external;
function Player_Active(): cint; cdecl; external;
procedure Player_Stop(); cdecl; external;
procedure Player_TogglePause(); cdecl; external;
function Player_Paused(): cint; cdecl; external;
procedure Player_NextPosition(); cdecl; external;
procedure Player_PrevPosition(); cdecl; external;
procedure Player_SetPosition(value: cushort); cdecl; external;
function Player_Muted(value: cuchar): cint; cdecl; external;
procedure Player_SetVolume(value: csshort); cdecl; external;
function Player_GetModule(): PMODULE; cdecl; external;
procedure Player_SetSpeed(value: cushort); cdecl; external;
procedure Player_SetTempo(value: cushort); cdecl; external;
procedure Player_Unmute(value: csint; args: array of const); cdecl; external;
procedure Player_Mute(value: csint; args: array of const); cdecl; external;
procedure Player_ToggleMute(value: csint; args: array of const); cdecl; external;
function Player_GetChannelVoice(value: cuchar): cint; cdecl; external;
function Player_GetChannelPeriod(value: cuchar): cushort; cdecl; external;

function MikMod_RegisterPlayer(value: MikMod_player_t): MikMod_player_t; cdecl; external;

function VC_Init(): cint; cdecl; external;
procedure VC_Exit(); cdecl; external;
function VC_SetNumVoices(): cint; cdecl; external;
function VC_SampleSpace(value: cint): cuint; cdecl; external;
function VC_SampleLength(value1: cint; value2: PSAMPLE): cuint; cdecl; external;

function VC_PlayStart(): cint; cdecl; external;
procedure VC_PlayStop(); cdecl; external;

function VC_SampleLoad(value1: PSAMPLOAD; value2: cint): csshort; cdecl; external;
procedure VC_SampleUnload(value: csshort); cdecl; external;

function VC_WriteBytes(value1: pcschar; value2: cuint): cuint; cdecl; external;
function VC_SilenceBytes(value1: pcschar; value2: cuint): cuint; cdecl; external;

procedure VC_VoiceSetVolume(value1: cuchar; value2: cushort); cdecl; external;
function VC_VoiceGetVolume(value: cuchar): cushort; cdecl; external;
procedure VC_VoiceSetFrequency(value1: cuchar; value2: cuint); cdecl; external;
function VC_VoiceGetFrequency(value: cuchar): cuint; cdecl; external;
procedure VC_VoiceSetPanning(value1: cuchar; value2: cuint); cdecl; external;
function VC_VoiceGetPanning(value: cuchar): cuint; cdecl; external;
procedure VC_VoicePlay(value1: cuchar; value2: csshort; value3, value4, value5, value6: cuint; value7: cushort); cdecl; external;

procedure VC_VoiceStop(value: cuchar); cdecl; external;
function VC_VoiceStopped(value: cuchar): cint; cdecl; external;
function VC_VoiceGetPosition(value: cuchar): csint; cdecl; external;
function VC_VoiceRealVolume(value: cuchar): cuint; cdecl; external;


(****** NDS part **************************************************************) 
const
  NDS_HW_MAXSAMPLES = 128;
  NDS_HW_MAXVOICES = 16;

  NDS_HW_CHANGE_START     = (1 shl 0);
  NDS_HW_CHANGE_STOP      = (1 shl 1);
  NDS_HW_CHANGE_VOLUME    = (1 shl 2);
  NDS_HW_CHANGE_FREQUENCY = (1 shl 3);
  NDS_HW_CHANGE_PANNING   = (1 shl 4);

type
  NDS_SW_IPC = packed record
    buffer: pcschar;
    prevTimer: cint;
    writeCursor: cint;
    readCursor: cint;
    bufferSize: cint;
    sampleRate: cint;
    format: cint;
    playing: cint;
  end;

  NDS_HW_VOICE = packed record
    handle: csshort;
    start: cuint;
    length: cuint;
    loopstart: cuint;
    loopend: cuint;
    flags: cushort;
    volume: cushort;
    frequency: cuint;
    panning: cuint;
    playing: cint;
    changes: cuchar;
  end;

  NDS_HW_IPC = packed record
    samples: array [0..NDS_HW_MAXSAMPLES - 1] of pointer;
    voices: array [0..NDS_HW_MAXVOICES - 1] of NDS_HW_VOICE;
  end;

  NDS_COMMAND = (
    NDS_SW_CMD_INIT = 1,
    NDS_SW_CMD_START,
    NDS_SW_CMD_UPDATE,
    NDS_SW_CMD_STOP,
    NDS_SW_CMD_EXIT,
    NDS_HW_CMD_INIT,
    NDS_HW_CMD_EXIT,
    NDS_HW_CMD_PLAY_START,
    NDS_HW_CMD_PLAY_STOP,
    NDS_HW_CMD_RESET,
    NDS_HW_CMD_VOICE_UPDATE
  );
  
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
{$ifdef ARM9}
//procedure MikMod9_SendCommand(command: cuint); cdecl; forward; 
{$endif ARM9}

procedure _mm_free(var p: pointer); inline;
begin
  if p <> nil then
    FreeMem(p);
  p := nil;
end;
{$endif NDS_IMPLEMENTATION}
