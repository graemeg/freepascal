{
    This file is part of the Free Pascal Sinclair QL support package.
    Copyright (c) 2021 by Norman Dunbar

    Include file to define the system variables record, QDOS naming

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}

{ This file holds the system variables for a QDOS system and uses
  the QDOS (original) naming convention. ND. }

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

Type
  SystemVariables = record
    sv_ident: Longint;  { $00 - system variables identifier }

{
  The following variables are the pointers which define the 
  current state of the memory map.
}
    sv_cheap: Pointer;  { $04 - Start of Common Heap area }
    sv_chpfr: Pointer; { $08 - First free space in Common Heap }
    sv_free: Pointer;  { $0C - Start of free area }
    sv_basic: Pointer; { $10 - Start of SuperBasic Area }
    sv_trnsp: Pointer; { $14 - Start of Transient Program Area }
    sv_trnfr: Pointer; { $18 - First free space in Transient Program Area }
    sv_respr: Pointer; { $1C - Resident Procedure Area Base }
    sv_ramt: Pointer;  { $20 - RAM Top (+1) }
    sv_unused_24: Array [0..9] of Byte; { $24 - $2D, Unused }
    sv_rand: Word;     { $2E - Random number }
    sv_pollm: Word;    { $30 - Number of missed poll interrupts }
    sv_tvmod: Byte;    { $32 - Display Type (0=normal, 1=TV PAL, 2=TV NSTC), <>2 = TV PAL }
    sv_scrst: Byte;    { $33 - Display Frozen (0=active, <>0 = frozen) }
    sv_mcsta: Byte;    { $34 - Copy of TV Register (MC_STAT) }
    sv_pcint: Byte;    { $35 - Copy of Interrupt Register (PC_INTR) }
    sv_unused_36: Byte; { $36 - Unused }
    sv_netnr: Byte;    { $37 - Network node number }

    sv_i2lst: Pointer; { $38 - Start of External Interrupt List }
    sv_plist: Pointer; { $3C - Start of Polled Tasks List }
    sv_shlst: Pointer; { $40 - Start of Scheduler Tasks List }
    sv_drlst: Pointer; { $44 - Start of simple Device Driver List }
    sv_ddlst: Pointer; { $48 - Start of Directory Driver List }
    sv_keyq: Pointer;  { $4C - Current Keyboard Queue, 0 if none }
    sv_trapv: Pointer; { $50 - Exception Redirection Table, 0 if none }

    sv_btpnt: Pointer; { $54 - Most recent slave block entry }
    sv_btbas: Pointer; { $58 - Start of Slave Block Table }
    sv_bttop: Pointer; { $5C - End of Slave Block Table }

    sv_jbtag: Word;    { $60   Current value of Job Tag }
    sv_jbmax: Word;    { $62 - Highest Job Number so far }
    sv_jbpnt: Pointer; { $64 - Current Job Table entry }
    sv_jbbas: Pointer; { $68 - Job Table Base }
    sv_jbtop: Pointer; { $6C - Job Table Top }

    sv_chtag: Word;    { $70 - Current value of Channel Tag }
    sv_chmax: Word;    { $72 - Highest Channel Number so far }
    sv_chpnt: Pointer; { $74 - Last channel checked for I/O }
    sv_chbas: Pointer; { $78 - Start of Channel Table }
    sv_chtop: Pointer; { $7C - End of Channel Table }

    sv_unused_80: Array [0..7] of Byte; { $80-$87, Unused }
    sv_caps: Word;     { $88 - CAPS lock (0 if off, $FF00 if on) }
    sv_arbuf: Word;    { $8A - Last Character (for auto-repeat) }
                       { = $00xx if ALT not pressed
                         = $xxFF if ALT pressed }
    sv_ardel: Word;    { $8C - Repeat Delay (20ms units) }
    sv_arfrq: Word;    { $8E - Repeat Frequency (20ms units) }
    sv_arcnt: Word;    { $90 - Repeat Counter (decremented every 20ms) }
    sv_cqch: Word;     { $92 - Change keyboard queue character }
    sv_wp: Word;       { $94 - Should be MDV write protect status, but not implemented }
    sv_sound: Word;    { $96 - Beeping? (0 if off, $FF00 if on) }
    sv_ser1c: Pointer; { $98 - Address of SER1 input queue }
    sv_ser2c: Pointer; { $9C - Address of SER2 input queue }
    sv_tmode: Byte;    { $A0 - ULA transmit mode }
                       { Bits 0-2: Baud rate number
                         Bit 3   : 0 = SER1, 1 = SER2
                         Bit 4   : MDV running }
    sv_ptyp: Byte;     { $A1 - Processor Type $00=68000/8, $30=68030 etc. [SMSQ] }
    sv_csub: Pointer;  { $A2 - Subroutine to jump to on Caps Lock }
    sv_timo: Word;     { $A6 - Counter for timing serial output }
    sv_timov: Word;    { $A8 - Initial value of sv_timo }
                        {  Formula = (1200/baud+1, i.e. 
                                     11 = 75   bps, 
                                      5 = 300  bps, 
                                      3 = 600  bps, 
                                      2 = 1200 bps, 
                                      1 = 2400 bps+) [QL] }
    sv_fstat: Word;    { $AA - Cursor flash counter }

{ 
  The original QL had 66 unused bytes between $AC and $ED inclusive. Over time
  these have been used by Toolkit 2, for example, for default devices etc. These
  are defined with the SMS names as QDOS didn't have them. 
}

    sv_prgd: Pointer;  { $AC - Pointer to Program Default device }
    sv_datd: Pointer;  { $B0 - Pointer to Data Default device }
    sv_dstd: Pointer;  { $B4 - Pointer to Destination Default device }
    sv_thgl: Pointer;  { $B8 - Pointer to ThinG List }
    sv_unused_bc: Array [0..49] of Byte; { $BC-$ED Unused }
    sv_mdrun: Byte;    { $EE - Which MDV drive is running? }
    sv_mdcnt: Byte;    { $EF - MDV run-up run-down counter }
    sv_mddid: Array [0..7] of Byte; { $F0 - Drive ID*4 of each microdrive [QL] }
    sv_mdsta: Array [0..7] of Byte; { $F8 - MDV Status: 0=no pending ops [QL] }
    sv_fsdef: Array [0..15] of Pointer; { $100 Long Pointers to File System Drive Definitions }
    sv_unused_140: Array [0..3] of Byte; { $140 - Unused }
    sv_xact: Byte;     { $144 - Set if TRANSLATE active [QDOS V1.10+, SMSQ, not SMS2] }
    sv_unused_145: Byte; { $145 - Unused }
    sv_xtab: Pointer;  { $146 - Pointer to TRANSLATE table [QDOS V1.10+, SMSQ, not SMS2] }
    sv_erms: Pointer;  { $14A -Pointer to (QDOS) error message table [QDOS V1.10+, SMSQ, not SMS2] }
    sv_unused_14e: Array [0..5] of Byte; { $014E-$0153, Unused }

{
  Offset $0154 is a table of 4 long words used by Taskmaster
  but one which is also used by TURBO. I've used the most 
  likely case here, Taskmaster is pretty much defunct.
  (Famous last words?) 
}

    //sv_taskm: Array [0..3] of Longint; {$154 - 4 Long Used by Taskmaster }
    sv_unused_154: Array [0..2] of Longint; { First 3 Taskmaster longs }
    sv_turbo: Longint; { $160 - Used by Turbo }
  end;

  pSystemVariables = ^SystemVariables;
