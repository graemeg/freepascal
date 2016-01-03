@ 00h
@ extern void SoftReset(void);
@
      .GLOBAL  SoftReset
      .THUMB_FUNC
SoftReset:
  swi   1
  swi   0
 
@--------------------------------------------------
@ 01h
@ extern void RegisterRamReset(u32 flags);
 
      .GLOBAL  RegisterRamReset
      .THUMB_FUNC
RegisterRamReset:
  swi   1
  bx    lr
 
@--------------------------------------------------
@ 02h
@ extern void Halt(void);
 
      .GLOBAL  Halt
      .THUMB_FUNC
Halt:       
  swi   2
  bx    lr
 
@--------------------------------------------------
@ 03h
@ extern void Stop(void);
 
      .GLOBAL  Stop
      .THUMB_FUNC
Stop:       swi      3
            bx    lr
 
@--------------------------------------------------
@ 04h
@ extern void IntrWait(u32 flagClear, u32 irq);
 
      .GLOBAL  IntrWait
      .THUMB_FUNC
IntrWait:      swi      4
            bx    lr
 
@--------------------------------------------------
@ 05h
@ extern void VBlankIntrWait(void);
 
      .GLOBAL  VBlankIntrWait
      .THUMB_FUNC
VBlankIntrWait:
            swi      5
            bx    lr
 
@--------------------------------------------------
@ 06h
@ extern s32 Divi(s32 numerator, s32 denominator); 
 
      .GLOBAL  Divi
      .THUMB_FUNC
Divi:       swi      6
            bx    lr
 
 
@--------------------------------------------------
@ 07h
@ extern s32 DiviArm(s32 denominator, s32 numerator);   
 
      .GLOBAL  DiviArm
      .THUMB_FUNC
DiviArm:    swi      7
            bx    lr
 
@--------------------------------------------------
@ 08h
@ extern u32 Sqrt(u32 value);
 
      .GLOBAL  Sqrt
      .THUMB_FUNC
Sqrt:       swi      8
            bx    lr
 
 
@--------------------------------------------------
@ 09h
@ extern s16 ArcTan(s16 ang);
 
      .GLOBAL  ArcTan
      .THUMB_FUNC
ArcTan:        swi      9
            bx    lr
 
@--------------------------------------------------
@ 0Ah
@ extern u16 ArcTan2(s16 x, s16 y);
 
      .GLOBAL  ArcTan2
      .THUMB_FUNC
ArcTan2:    swi      10
            bx    lr
 
@--------------------------------------------------
@ 0Bh
@ extern void CpuSet(void *source, void *dest, u32 mode);
 
      .GLOBAL  CpuSet
      .THUMB_FUNC
CpuSet:        swi      11
            bx    lr
 
@--------------------------------------------------
@ 0Ch
@ extern void CpuFastSet(void *source, void *dest, u32 mode);
 
      .GLOBAL  CpuFastSet
      .THUMB_FUNC
CpuFastSet:    swi      12
            bx    lr
 
@--------------------------------------------------
@ 0Eh
@ extern void BgAffineSet(BgAffineSrcData *source, BgAffineDestData *dest, s32 num);
 
      .GLOBAL  BgAffineSet
      .THUMB_FUNC
BgAffineSet:   swi      14
            bx    lr
 
@--------------------------------------------------
@ 0Fh
@ extern void ObjAffineSet(ObjAffineSrcData *source, void *dest, s32 num, s32 offset);
 
      .GLOBAL  ObjAffineSet
      .THUMB_FUNC
ObjAffineSet:  swi      15
            bx    lr
 
@--------------------------------------------------
@ 10h
@ extern void BitUnPack(void  *source, void *dest, BUP* bup);
 
      .GLOBAL  BitUnPack
      .THUMB_FUNC
BitUnPack:     swi      16
            bx    lr
 
@--------------------------------------------------
@ 11h
@ extern void LZ77UnCompWram(void *source, void *dest);
 
      .GLOBAL  LZ77UnCompWram
      .THUMB_FUNC
LZ77UnCompWram:
            swi      17
            bx    lr
 
@--------------------------------------------------
@ 12h
@ extern void LZ77UnCompVram(void *source, void *dest);
 
      .GLOBAL  LZ77UnCompVram
      .THUMB_FUNC
LZ77UnCompVram:
            swi      18
            bx    lr
 
@--------------------------------------------------
@ 13h
@ extern void HuffUnComp(void *source, void *dest);
 
      .GLOBAL  HuffUnComp
      .THUMB_FUNC
HuffUnComp:    swi      19
            bx    lr
 
@--------------------------------------------------
@ 14h
@ extern void RLUnCompWram(void *source, void *dest);
 
      .GLOBAL  RLUnCompWram
      .THUMB_FUNC
RLUnCompWram:  swi      20
            bx    lr
 
@--------------------------------------------------
@ 15h
@ extern void RLUnCompVram(void *source, void *dest);
 
      .GLOBAL  RLUnCompVram
      .THUMB_FUNC
RLUnCompVram:  swi      21
            bx    lr
 
@--------------------------------------------------
@ 16h
@ extern void Diff8bitUnFilterWram(void *source, void *dest);
 
      .GLOBAL  Diff8bitUnFilterWram
      .THUMB_FUNC
Diff8bitUnFilterWram:
            swi      22
            bx    lr
 
@--------------------------------------------------
@ 17h
@ extern void Diff8bitUnFilterVram(void *source, void *dest);
 
      .GLOBAL  Diff8bitUnFilterVram
      .THUMB_FUNC
Diff8bitUnFilterVram:
            swi      23
            bx    lr
 
@--------------------------------------------------
@ 18h
@ extern void Diff16bitUnFilter(void *source, void *dest);
 
      .GLOBAL  Diff16bitUnFilter
      .THUMB_FUNC
Diff16bitUnFilter:
            swi      24
            bx    lr
 
 
@--------------------------------------------------
@ 19h
@ extern void SoundBias(void);
 
      .GLOBAL  SoundBias
      .THUMB_FUNC
SoundBiasSet:
            swi      25
            bx    lr
 
@--------------------------------------------------
@ 1Ah
@ extern void SoundDriverInit(void);
 
      .GLOBAL  SoundDriverInit
      .THUMB_FUNC
SoundDriverInit:
            swi      26
            bx    lr
 
 
@--------------------------------------------------
@ 1Bh
@ extern void SoundDriverMode(u32 mode);
 
      .GLOBAL  SoundDriverMode
      .THUMB_FUNC
SoundDriverMode:
            swi      27
            bx    lr
 
@--------------------------------------------------
@ 1Ch
@ extern void SoundDriverMain(void);
 
      .GLOBAL  SoundDriverMain
      .THUMB_FUNC
SoundDriverMain:
            swi      28
            bx    lr
 
@--------------------------------------------------
@ 1Dh
@ extern void SoundDriverVSync(void);
 
      .GLOBAL  SoundDriverVSync
      .THUMB_FUNC
SoundDriverVSync:
            swi      29
            bx    lr
 
@--------------------------------------------------
@ 1Eh
@ extern void SoundChannelClear(void);
 
      .GLOBAL  SoundChannelClear
      .THUMB_FUNC
SoundChannelClear:
            swi      30
            bx    lr
 
 
@--------------------------------------------------
@ 1Fh
@ extern void  MidiKey2Freq(void);
 
      .GLOBAL  MidiKey2Freq
      .THUMB_FUNC
MidiKey2Freq:
            swi      31
            bx    lr
 
@--------------------------------------------------
@ 25h
@ extern int MultiBoot(void)
 
      .GLOBAL  MultiBoot
      .THUMB_FUNC
MultiBoot:
            swi      37
            bx    lr
 
@--------------------------------------------------
@ 28h
@ extern void SoundDriverVSyncOff(void);
 
      .GLOBAL  SoundDriverVSyncOff
      .THUMB_FUNC
SoundDriverVSyncOff:
            swi      40
            bx    lr
 
@--------------------------------------------------
@ 29h
@ extern void SoundDriverVSyncOn(void);
 
      .GLOBAL  SoundDriverVSyncOn
      .THUMB_FUNC
SoundDriverVSyncOn:
            swi      41
            bx    lr
  .END
