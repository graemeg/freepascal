{
    This file is part of the Free Pascal Sinclair QL support package.
    Copyright (c) 2021 by Norman Dunbar

    Include file to define the system variables record, SMSQ naming

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{

{ This file holds the system variables for an SMSQ system and uses
  the SMSQ naming convention. ND. }

Const
  { System Identifier constants }
  sysid_ql = $D2540000;    { QL (QDOS) system variable identifier }
  sysid_at = $53324154;    { SMS Atari system variable identifier }
  sysid_sq = $534D5351;    { SMSQ identifier }
  sysid_th = $DC010000;    { Thor (ARGOS) system variable identifier }

  { Display Type constants }
  dtype_monitor = 0;
  dtype_tv_625 = 1;
  dtype_tv_525 = 2;

  { Machine Identifier constants }
  mtype_st = 0;
  mtype_mega_st_rtc = 2;
  mtype_stacy = 4;
  mtype_ste = 6;
  mtype_mega_ste = 8;
  mtype_gold_card = $0A;
  mtype_super_gold_card = $0C;
  mtype_falcon = $10;
  mtype_q40_q60 = $11;
  mtype_smsqmulator = $14;
  mtype_tt = $18;
  mtype_qxl = $1C;
  mtype_qpc = $1E;

Type
  SystemVariables = record
    sys_idnt: Longint;  { system variables identifier }

{
  The following variables are the pointers which define the 
  current state of the memory map.
}
    sys_chpb: Pointer;  { Common HeaP Base }
    sys_chpf: Pointer;  { Common HeaP Free space pointer }
    sys_fsbb: Pointer;  { Filing system Slave Block area Base }
    sys_sbab: Pointer;  { 'QL S*Basic' Area Base }
    sys_tpab: Pointer;  { Transient Program Area Base }
    sys_tpaf: Pointer;  { Transient Program Area Free space pointer }
    sys_rpab: Pointer;  { Resident Procedure Area Base }
    sys_ramt: Pointer;  { RAM Top (+1) }
    sys_mxfr: Pointer;  { Maximum return from free memory call [SMS] }
    sys_rtc:  Longint;  { Real time (seconds) [SMS] }
    sys_rtcf: Word;     { Real time fractional, count down [SMS] }
    sys_rand: Word;     { RANDom number }

    sys_pict: Word;     { Polling Interupt CounT }
    sys_dtyp: Byte;     { Display TYPe (0=normal, 1=TV 625, 2=TV 525) }
    sys_dfrz: Byte;     { Display FRoZen (T or F) }
    sys_qlmr: Byte;     { QL Master chip Register value (Copy of MC_STAT) }
    sys_qlir: Byte;     { QL Interrupt Register value (Copy of PC_INTR) }
    sys_rshd: Byte;     { True to reschedule [SMS] }
    sys_nnnr: Byte;     { Network Node NumbeR }

{
  The following system variables are pointers to the list of
  tasks and drivers.
}

    sys_exil: Pointer;  { EXternal Interrupt action List }
    sys_poll: Pointer;  { POLled action List }
    sys_shdl: Pointer;  { ScHeDuler loop action List }
    sys_iodl: Pointer;  { IO Driver List }
    sys_fsdl: Pointer;  { Filing System Driver List }
    sys_ckyq: Pointer;  { Current Keyboard Queue }
    sys_ertb: Pointer;  { Exception Redirection Table Base }

{
  The following system variables are pointers to the resource
  management tables. The slave block tables have 8 byte 
  entries, whilst the others have 4 byte entries.
}

    { First the slave block tables }

    sys_sbrp: Pointer;  { Slave Block Running Pointer }
    sys_sbtb: Pointer;  { Slave Block Table Base }
    sys_sbtt: Pointer;  { Slave Block Table Top }

    { Then job stuff }

    sys_jbtg: Word;     { Next JoB TaG }
    sys_jbtp: Word;     { Highest JoB in table (ToP one) }
    sys_jbpt: Pointer;  { Current JoB PoinTer }
    sys_jbtb: Pointer;  { JoB Table Base }
    sys_jbtt: Pointer;  { JoB Table Top }

    { Then channel stuff. (Not SuperBASIC channels) }

    sys_chtg: Word;     { Next CHannel TaG }
    sys_chtp: Word;     { Highest CHannel in table (ToP one) }
    sys_chpt: Pointer;  { Last checked CHannel PoinTer }
    sys_chtb: Pointer;  { CHannel Table Base }
    sys_chtt: Pointer;  { CHannel Table Top }

    { Then odds and sods }

    sys_frbl: Pointer;  { FRee Block List (to return to common heap) [SMS] }
    sys_tsdd: Byte;     { Thor flag [THOR] }
    sys_unused_85: array [0..2] of Byte; { Unused, offset = $85, $86 and $87 }
{
  The following variables contain information about how to 
  treat the keyboard, and about other aspects of the
  IPC and serial port communications.
}

    sys_caps: Word;     { CAPS lock (0 if off, $FF00 if on) }
    sys_lchr: Word;     { Last CHaRacter (for auto-repeat) }
    sys_rdel: Word;     { Repeat DELay (20ms units) }
    sys_rtim: Word;     { Repeat TIMe (20ms units) }
    sys_rcnt: Word;     { Repeat CouNTer (decremented every 20ms) }
    sys_swtc: Word;     { SWiTch Keyboard queues Character }
    sys_unused_94: array [0..1] of Byte;  { Unused, offset = $94 and $95 }
    sys_qlbp: Word;     { QL BeePing (0 if off, $FF00 if on) }
    sys_ser1: Pointer;  { Receive channel 1 queue address [QL] }
    sys_ser2: Pointer;  { Receive channel 2 queue address [QL] }
    sys_tmod: Byte;     { ZX8302 transmit mode (includes baudrate) (copy of PC_TCTRL) [QL] }
    sys_ptyp: Byte;     { Processor TYPe $00=68000/8, $30=68030 etc. [SMSQ] }
    sys_csub: Pointer;  { Subroutine to jump to on Caps Lock }

    { Sys_stmo only on QL. Sys_dmiu/sys_mtyp on everything else. }
    // sys_stmo: Word;     { Serial xmit timeout [QL] }

    sys_dmiu: Byte;     { DMA in use? [SMS2, ST, SMSQ] }
    sys_mtyp: Byte;     { Machine TYPe [SMSQ] }

    { Sys_stmv on  QL Only. Redefined by other systems by sys_polf }
    //sys_stmv: Word;   { Value of serial timeout }
                        {  Formula = (1200/baud+1, i.e. 
                                     11 = 75   bps, 
                                      5 = 300  bps, 
                                      3 = 600  bps, 
                                      2 = 1200 bps, 
                                      1 = 2400 bps+) [QL] }

    sys_polf: Word;     { Polling frequency [SMSQ] }
    sys_cfst: Word;     { Flashing cursor status }

    { Filing system defaults }

    sys_prgd: Pointer;  { Pointer to PRoGram Default [EXT][SMSQ] }
    sys_datd: Pointer;  { Pointer to DATa Default [EXT][SMSQ] }
    sys_dstd: Pointer;  { Pointer to DeSTination Default [EXT][SMSQ] }
    sys_thgl: Pointer;  { Pointer to THinG List [EXT][SMSQ] }
    sys_psf:  Pointer;  { Primary stack frame pointer [SMSQ] }
    sys_200i: Byte;     { 200 Hz in service/interrupt 2 in service [SMSQ] }
    sys_50i:  Byte;     { 50 Hz in service [SMSQ] }
    sys_10i:  Byte;     { 10 Hz in service [SMSQ] }
    sys_plrq: Byte;     { Poll requested (-ve for request) [SMSQ] }

    sys_clnk: Pointer;  { Pointer to console linkage [SMSQ] }
    sys_castat: Byte;   { -1 cache on, +1 instruction cache temp off [SMSQ] }
    sys_casup: Byte;    { Cache suppressed timer [SMSQ] }
    sys_iopr: Word;     { I/O priority [SMSQ] }
    sys_cbas: Pointer;  { Current basic (copy of sys_jbpt) [SMSQ] }
    sys_fpu:  Array [0..15] of Byte; { FPU stuff? [SMSQ] }
    sys_prtc: Byte;     {  Set if real time clock protected [SMSQ] }
    sys_pmem: Byte;     { Memory protection level [SMSQ, ST] }
    sys_slug: Word;     { Slug level [SMSQ] }
    sys_klock: Byte;    { Key lock [SMSQ/E] }
    sys_unused_e5: Byte; { Not used, offset = $E5 }
    sys_mtick: Word;    { Mini tick counter [SMSQ/E] }
    sys_klnk: Pointer;  { Pointer to keyboard linkage [SMSQ/E] }
    sys_unused_ec: Array [1..2] of Byte; { Not used, offset = $EC and $ED }

    { Fixed filing system working area [QL] }

    sys_mdrn: Byte;     { Which MDV drive is running? }
    sys_mdct: Byte;     { MDV run-up run-down counter [QL] }
    sys_mdid: Array [0..7] of Byte; { Drive ID*4 of each microdrive [QL] }
    sys_mdst: Array [0..7] of Byte; { MDV Status: 0=no pending ops [QL] }
    sys_fsdd: Array [0..15] of Pointer; { Long Pointers to Filing System Drive Definitions }

    sys_fsch: Pointer;  { Linked list of Filing System CHannel blocks }
    sys_xact: Byte;     { Set if TRANSLATE active [QDOS V1.10+, SMSQ, not SMS2] }
    sys_unused_145: Byte; { Unused, offset = $0145 }
    sys_xtab: Pointer;  { Pointer to TRANSLATE table [QDOS V1.10+, SMSQ, not SMS2] }
    sys_erms: Pointer;  { Pointer to (QDOS) error message table [QDOS V1.10+, SMSQ, not SMS2] }

    sys_mstab: Pointer; { Pointer to (SMSQ) message table [SMSQ]. }
                        { This is a pointer to a 256 long word table of
                          pointers to message groups. All undefined 
                          message groups have a zero pointer. }
    sys_unused_152: Array [0..1] of Byte; { Unused, Offset = $0152 and $0153 }

    { Offset $0154 is a table of 4 long words used by Taskmaster
      but one which is also used by TURBO. I've used the most 
      likely case here, Taskmaster is pretty much defunct.
      (Famous last words?) }
    //sys_taskm: Array [0..3] of Longint; { 4 Long Used by Taskmaster }
    sys_unused_154: Array [0..2] of Longint; { First 3 Taskmaster longs }
    sys_turbo: Longint; { Used by Turbo }

    sys_qsound: Longint; { Used by QSound }
    sys_ldmlst: Pointer; { Language dependent module list [SMSQ] }
    sys_lang: Word;      { Current language [SMSQ] }
    sys_vers: Longint;   { Operating system version [SMSQ] }
    sys_rthg: Byte;      { Use RECENT Thing (<>0 if yes) [SMSQ/E 3.24+] }
    sys_xdly: Byte;      { Suspend delay after executing another job [SMSQ/E 3.13+] }
    sys_ouch: Byte;      { Ouch flag (currently used to activate SGC debug) [SMSQ] }

  end; 

  pSystemVariables = ^SystemVariables;

