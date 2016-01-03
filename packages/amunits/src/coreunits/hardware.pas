{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    registers and bits in the Complex Interface Adapter (CIA) chip
}

unit hardware;

INTERFACE
uses exec;

{
 * ciaa is on an ODD Pointer (e.g. the low Byte) -- $bfe001
 * ciab is on an EVEN Pointer (e.g. the high Byte) -- $bfd000
 *
 * do this to get the definitions:
 *    extern struct CIA ciaa, ciab;
}

Type

    pCIA = ^tCIA;
    tCIA = record
    ciapra      : Byte;
    pad0        : Array [0..254] of Byte;
    ciaprb      : Byte;
    pad1        : Array [0..254] of Byte;
    ciaddra     : Byte;
    pad2        : Array [0..254] of Byte;
    ciaddrb     : Byte;
    pad3        : Array [0..254] of Byte;
    ciatalo     : Byte;
    pad4        : Array [0..254] of Byte;
    ciatahi     : Byte;
    pad5        : Array [0..254] of Byte;
    ciatblo     : Byte;
    pad6        : Array [0..254] of Byte;
    ciatbhi     : Byte;
    pad7        : Array [0..254] of Byte;
    ciatodlow   : Byte;
    pad8        : Array [0..254] of Byte;
    ciatodmid   : Byte;
    pad9        : Array [0..254] of Byte;
    ciatodhi    : Byte;
    pad10       : Array [0..254] of Byte;
    unusedreg   : Byte;
    pad11       : Array [0..254] of Byte;
    ciasdr      : Byte;
    pad12       : Array [0..254] of Byte;
    ciaicr      : Byte;
    pad13       : Array [0..254] of Byte;
    ciacra      : Byte;
    pad14       : Array [0..254] of Byte;
    ciacrb      : Byte;
    end;


Const

{ interrupt control register bit numbers }

    CIAICRB_TA      = 0;
    CIAICRB_TB      = 1;
    CIAICRB_ALRM    = 2;
    CIAICRB_SP      = 3;
    CIAICRB_FLG     = 4;
    CIAICRB_IR      = 7;
    CIAICRB_SETCLR  = 7;

{ control register A bit numbers }

    CIACRAB_START   = 0;
    CIACRAB_PBON    = 1;
    CIACRAB_OUTMODE     = 2;
    CIACRAB_RUNMODE     = 3;
    CIACRAB_LOAD    = 4;
    CIACRAB_INMODE  = 5;
    CIACRAB_SPMODE  = 6;
    CIACRAB_TODIN   = 7;

{ control register B bit numbers }

    CIACRBB_START   = 0;
    CIACRBB_PBON    = 1;
    CIACRBB_OUTMODE     = 2;
    CIACRBB_RUNMODE     = 3;
    CIACRBB_LOAD    = 4;
    CIACRBB_INMODE0     = 5;
    CIACRBB_INMODE1     = 6;
    CIACRBB_ALARM   = 7;

{ interrupt control register masks }

    CIAICRF_TA      = $01;
    CIAICRF_TB      = $02;
    CIAICRF_ALRM    = $04;
    CIAICRF_SP      = $08;
    CIAICRF_FLG     = $10;
    CIAICRF_IR      = $80;
    CIAICRF_SETCLR  = $80;

{ control register A register masks }

    CIACRAF_START   = $01;
    CIACRAF_PBON    = $02;
    CIACRAF_OUTMODE = $04;
    CIACRAF_RUNMODE = $08;
    CIACRAF_LOAD    = $10;
    CIACRAF_INMODE  = $20;
    CIACRAF_SPMODE  = $40;
    CIACRAF_TODIN   = $80;

{ control register B register masks }

    CIACRBF_START   = $01;
    CIACRBF_PBON    = $02;
    CIACRBF_OUTMODE = $04;
    CIACRBF_RUNMODE = $08;
    CIACRBF_LOAD    = $10;
    CIACRBF_INMODE0 = $20;
    CIACRBF_INMODE1 = $40;
    CIACRBF_ALARM   = $80;

{ control register B INMODE masks }

    CIACRBF_IN_PHI2 = 0;
    CIACRBF_IN_CNT  = CIACRBF_INMODE0;
    CIACRBF_IN_TA   = CIACRBF_INMODE1;
    CIACRBF_IN_CNT_TA   = CIACRBF_INMODE0 + CIACRBF_INMODE1;

{
 * Port definitions -- what each bit in a cia peripheral register is tied to
 }

{ ciaa port A (0xbfe001) }

    CIAB_GAMEPORT1  = 7;    { gameport 1, pin 6 (fire button*) }
    CIAB_GAMEPORT0  = 6;    { gameport 0, pin 6 (fire button*) }
    CIAB_DSKRDY     = 5;    { disk ready* }
    CIAB_DSKTRACK0  = 4;    { disk on track 00* }
    CIAB_DSKPROT    = 3;    { disk write protect* }
    CIAB_DSKCHANGE  = 2;    { disk change* }
    CIAB_LED        = 1;    { led light control (0==>bright) }
    CIAB_OVERLAY    = 0;    { memory overlay bit }

{ ciaa port B (0xbfe101) -- parallel port }

{ ciab port A (0xbfd000) -- serial and printer control }

    CIAB_COMDTR     = 7;    { serial Data Terminal Ready* }
    CIAB_COMRTS     = 6;    { serial Request to Send* }
    CIAB_COMCD      = 5;    { serial Carrier Detect* }
    CIAB_COMCTS     = 4;    { serial Clear to Send* }
    CIAB_COMDSR     = 3;    { serial Data Set Ready* }
    CIAB_PRTRSEL    = 2;    { printer SELECT }
    CIAB_PRTRPOUT   = 1;    { printer paper out }
    CIAB_PRTRBUSY   = 0;    { printer busy }

{ ciab port B (0xbfd100) -- disk control }

    CIAB_DSKMOTOR   = 7;    { disk motorr* }
    CIAB_DSKSEL3    = 6;    { disk select unit 3* }
    CIAB_DSKSEL2    = 5;    { disk select unit 2* }
    CIAB_DSKSEL1    = 4;    { disk select unit 1* }
    CIAB_DSKSEL0    = 3;    { disk select unit 0* }
    CIAB_DSKSIDE    = 2;    { disk side select* }
    CIAB_DSKDIREC   = 1;    { disk direction of seek* }
    CIAB_DSKSTEP    = 0;    { disk step heads* }

{ ciaa port A (0xbfe001) }

    CIAF_GAMEPORT1  = 128;
    CIAF_GAMEPORT0  = 64;
    CIAF_DSKRDY     = 32;
    CIAF_DSKTRACK0  = 16;
    CIAF_DSKPROT    = 8;
    CIAF_DSKCHANGE  = 4;
    CIAF_LED        = 2;
    CIAF_OVERLAY    = 1;

{ ciaa port B (0xbfe101) -- parallel port }

{ ciab port A (0xbfd000) -- serial and printer control }

    CIAF_COMDTR     = 128;
    CIAF_COMRTS     = 64;
    CIAF_COMCD      = 32;
    CIAF_COMCTS     = 16;
    CIAF_COMDSR     = 8;
    CIAF_PRTRSEL    = 4;
    CIAF_PRTRPOUT   = 2;
    CIAF_PRTRBUSY   = 1;

{ ciab port B (0xbfd100) -- disk control }

    CIAF_DSKMOTOR   = 128;
    CIAF_DSKSEL3    = 64;
    CIAF_DSKSEL2    = 32;
    CIAF_DSKSEL1    = 16;
    CIAF_DSKSEL0    = 8;
    CIAF_DSKSIDE    = 4;
    CIAF_DSKDIREC   = 2;
    CIAF_DSKSTEP    = 1;


Type

    pAudChannel = ^tAudChannel;
    tAudChannel = record
        ac_ptr          : Pointer;      { ptr to start of waveform data }
        ac_len          : Word;         { length of waveform in words }
        ac_per          : Word;         { sample period }
        ac_vol          : Word;         { volume }
        ac_dat          : Word;         { sample pair }
        ac_pad          : Array [0..1] of Word;        { unused }
    end;

    pSpriteDef = ^tSpriteDef;
    tSpriteDef = record
        pos             : Word;
        ctl             : Word;
        dataa           : Word;
        datab           : Word;
    end;

   pCustom = ^tCustom;
   tCustom = record
        bltddat         : Word;
        dmaconr         : Word;
        vposr           : Word;
        vhposr          : Word;
        dskdatr         : Word;
        joy0dat         : Word;
        joy1dat         : Word;
        clxdat          : Word;
        adkconr         : Word;
        pot0dat         : Word;
        pot1dat         : Word;
        potinp          : Word;
        serdatr         : Word;
        dskbytr         : Word;
        intenar         : Word;
        intreqr         : Word;
        dskpt           : Pointer;
        dsklen          : Word;
        dskdat          : Word;
        refptr          : Word;
        vposw           : Word;
        vhposw          : Word;
        copcon          : Word;
        serdat          : Word;
        serper          : Word;
        potgo           : Word;
        joytest         : Word;
        strequ          : Word;
        strvbl          : Word;
        strhor          : Word;
        strlong         : Word;
        bltcon0         : Word;
        bltcon1         : Word;
        bltafwm         : Word;
        bltalwm         : Word;
        bltcpt          : Pointer;
        bltbpt          : Pointer;
        bltapt          : Pointer;
        bltdpt          : Pointer;
        bltsize         : Word;
        pad2d           : Byte;
        bltcon0l        : Byte;
        bltsizv         : Word;
        bltsizh         : Word;
        bltcmod         : Word;
        bltbmod         : Word;
        bltamod         : Word;
        bltdmod         : Word;
        pad34           : Array [0..3] of Word;
        bltcdat         : Word;
        bltbdat         : Word;
        bltadat         : Word;
        pad3b           : Array [0..2] of Word;
        deniseid        : Word;
        dsksync         : Word;
        cop1lc          : Longint;
        cop2lc          : Longint;
        copjmp1         : Word;
        copjmp2         : Word;
        copins          : Word;
        diwstrt         : Word;
        diwstop         : Word;
        ddfstrt         : Word;
        ddfstop         : Word;
        dmacon          : Word;
        clxcon          : Word;
        intena          : Word;
        intreq          : Word;
        adkcon          : Word;
        aud             : Array [0..3] of tAudChannel;
        bplpt           : Array [0..7] of Pointer;
        bplcon0         : Word;
        bplcon1         : Word;
        bplcon2         : Word;
        bplcon3         : Word;
        bpl1mod         : Word;
        bpl2mod         : Word;
        bplcon4         : Word;
        clxcon2         : Word;
        bpldat          : Array [0..7] of Word;
        sprpt           : Array [0..7] of Pointer;
        spr             : Array [0..7] of tSpriteDef;
        color           : Array [0..31] of Word;
        htotal          : Word;
        hsstop          : Word;
        hbstrt          : Word;
        hbstop          : Word;
        vtotal          : Word;
        vsstop          : Word;
        vbstrt          : Word;
        vbstop          : Word;
        sprhstrt        : Word;
        sprhstop        : Word;
        bplhstrt        : Word;
        bplhstop        : Word;
        hhposw          : Word;
        hhposr          : Word;
        beamcon0        : Word;
        hsstrt          : Word;
        vsstrt          : Word;
        hcenter         : Word;
        diwhigh         : Word;
        padf3           : Array [0..10] of Word;
        fmode           : Word;
    end;

CONST
{    defines for beamcon register }
  VARVBLANK     =  $1000;  {    Variable vertical blank enable }
  LOLDIS        =  $0800;  {    long line disable }
  CSCBLANKEN    =  $0400;  {    redirect composite sync }
  VARVSYNC      =  $0200;  {    Variable vertical sync enable }
  VARHSYNC      =  $0100;  {    Variable horizontal sync enable }
  VARBEAM       =  $0080;  {    variable beam counter enable }
  DISPLAYDUAL   =  $0040;  {    use UHRES pointer AND standard pointers }
  DISPLAYPAL    =  $0020;  {    set decodes to generate PAL display }
  VARCSYNC      =  $0010;  {    Variable composite sync enable }
  CSBLANK       =  $0008;  {    Composite blank out to CSY* pin }
  CSYNCTRUE     =  $0004;  {    composite sync TRUE signal }
  VSYNCTRUE     =  $0002;  {    vertical sync TRUE }
  HSYNCTRUE     =  $0001;  {    horizontal sync TRUE }

{    new defines for bplcon0 }
  USE_BPLCON3   =  1;

{    new defines for bplcon2 }
  BPLCON2_ZDCTEN        =  1024; {    colormapped genlock bit }
  BPLCON2_ZDBPEN        =  2048; {    use bitplane as genlock bits }
  BPLCON2_ZDBPSEL0      =  4096; {    three bits to select one }
  BPLCON2_ZDBPSEL1      =  8192; {    of 8 bitplanes in }
  BPLCON2_ZDBPSEL2      =  16384; {    ZDBPEN genlock mode }

{    defines for bplcon3 register }
  BPLCON3_EXTBLNKEN     =  1;  {    external blank enable }
  BPLCON3_EXTBLKZD      =  2;  {    external blank ored into trnsprncy }
  BPLCON3_ZDCLKEN       =  4;  {    zd pin outputs a 14mhz clock}
  BPLCON3_BRDNTRAN      =  16;  {    border is opaque }
  BPLCON3_BRDNBLNK      =  32;  {    border is opaque }


Const

    ADKB_SETCLR     = 15;   { standard set/clear bit }
    ADKB_PRECOMP1   = 14;   { two bits of precompensation }
    ADKB_PRECOMP0   = 13;
    ADKB_MFMPREC    = 12;   { use mfm style precompensation }
    ADKB_UARTBRK    = 11;   { force uart output to zero }
    ADKB_WORDSYNC   = 10;   { enable DSKSYNC register matching }
    ADKB_MSBSYNC    = 9;    { (Apple GCR Only) sync on MSB for reading }
    ADKB_FAST       = 8;    { 1 -> 2 us/bit (mfm), 2 -> 4 us/bit (gcr) }
    ADKB_USE3PN     = 7;    { use aud chan 3 to modulate period of ?? }
    ADKB_USE2P3     = 6;    { use aud chan 2 to modulate period of 3 }
    ADKB_USE1P2     = 5;    { use aud chan 1 to modulate period of 2 }
    ADKB_USE0P1     = 4;    { use aud chan 0 to modulate period of 1 }
    ADKB_USE3VN     = 3;    { use aud chan 3 to modulate volume of ?? }
    ADKB_USE2V3     = 2;    { use aud chan 2 to modulate volume of 3 }
    ADKB_USE1V2     = 1;    { use aud chan 1 to modulate volume of 2 }
    ADKB_USE0V1     = 0;    { use aud chan 0 to modulate volume of 1 }

    ADKF_SETCLR     = $8000;
    ADKF_PRECOMP1   = $4000;
    ADKF_PRECOMP0   = $2000;
    ADKF_MFMPREC    = $1000;
    ADKF_UARTBRK    = $0800;
    ADKF_WORDSYNC   = $0400;
    ADKF_MSBSYNC    = $0200;
    ADKF_FAST       = $0100;
    ADKF_USE3PN     = $0080;
    ADKF_USE2P3     = $0040;
    ADKF_USE1P2     = $0020;
    ADKF_USE0P1     = $0010;
    ADKF_USE3VN     = $0008;
    ADKF_USE2V3     = $0004;
    ADKF_USE1V2     = $0002;
    ADKF_USE0V1     = $0001;

    ADKF_PRE000NS   = 0;            { 000 ns of precomp }
    ADKF_PRE140NS   = ADKF_PRECOMP0;    { 140 ns of precomp }
    ADKF_PRE280NS   = ADKF_PRECOMP1;    { 280 ns of precomp }
    ADKF_PRE560NS   = ADKF_PRECOMP0 + ADKF_PRECOMP1; { 560 ns of precomp }


Const

    HSIZEBITS           = 6;
    VSIZEBITS           = 16 - HSIZEBITS;
    HSIZEMASK           = $3F;                  { 2^6 - 1 }
    VSIZEMASK           = $3FF;                 { 2^10 - 1 }

    MAXBYTESPERROW      = 128;

{ definitions for blitter control register 0 }

    ABC         = $80;
    ABNC        = $40;
    ANBC        = $20;
    ANBNC       = $10;
    NABC        = $08;
    NABNC       = $04;
    NANBC       = $02;
    NANBNC      = $01;

{ some commonly used operations }

    A_OR_B      = ABC + ANBC + NABC + ABNC + ANBNC + NABNC;
    A_OR_C      = ABC + NABC + ABNC + ANBC + NANBC + ANBNC;
    A_XOR_C     = NABC + ABNC + NANBC + ANBNC;
    A_TO_D      = ABC + ANBC + ABNC + ANBNC;

    BC0B_DEST   = 8;
    BC0B_SRCC   = 9;
    BC0B_SRCB   = 10;
    BC0B_SRCA   = 11;
    BC0F_DEST   = $100;
    BC0F_SRCC   = $200;
    BC0F_SRCB   = $400;
    BC0F_SRCA   = $800;

    BC1F_DESC   = 2;            { blitter descend direction }

    DEST        = $100;
    SRCC        = $200;
    SRCB        = $400;
    SRCA        = $800;

    ASHIFTSHIFT = 12;           { bits to right align ashift value }
    BSHIFTSHIFT = 12;           { bits to right align bshift value }

{ definations for blitter control register 1 }

    LINEMODE    = $01;
    FILL_OR     = $08;
    FILL_XOR    = $10;
    FILL_CARRYIN = $04;
    ONEDOT      = $02;          { one dot per horizontal line }
    OVFLAG      = $20;
    SIGNFLAG    = $40;
    BLITREVERSE = $02;

    SUD         = $10;
    SUL         = $08;
    AUL         = $04;

    OCTANT8     = 24;
    OCTANT7     = 4;
    OCTANT6     = 12;
    OCTANT5     = 28;
    OCTANT4     = 20;
    OCTANT3     = 8;
    OCTANT2     = 0;
    OCTANT1     = 16;

 type
    pbltnode = ^tbltnode;
    tbltnode = record
       n         : pbltnode;
       _function : Pointer;
       stat      : Byte;
       blitsize  : smallint;
       beamsync  : smallint;
       cleanup   : Pointer;
    end;

Const

{ write definitions for dmaconw }

    DMAF_SETCLR     = $8000;
    DMAF_AUDIO      = $000F;    { 4 bit mask }
    DMAF_AUD0       = $0001;
    DMAF_AUD1       = $0002;
    DMAF_AUD2       = $0004;
    DMAF_AUD3       = $0008;
    DMAF_DISK       = $0010;
    DMAF_SPRITE     = $0020;
    DMAF_BLITTER    = $0040;
    DMAF_COPPER     = $0080;
    DMAF_RASTER     = $0100;
    DMAF_MASTER     = $0200;
    DMAF_BLITHOG    = $0400;
    DMAF_ALL        = $01FF;    { all dma channels }

{ read definitions for dmaconr }
{ bits 0-8 correspnd to dmaconw definitions }

    DMAF_BLTDONE    = $4000;
    DMAF_BLTNZERO   = $2000;

    DMAB_SETCLR     = 15;
    DMAB_AUD0       = 0;
    DMAB_AUD1       = 1;
    DMAB_AUD2       = 2;
    DMAB_AUD3       = 3;
    DMAB_DISK       = 4;
    DMAB_SPRITE     = 5;
    DMAB_BLITTER    = 6;
    DMAB_COPPER     = 7;
    DMAB_RASTER     = 8;
    DMAB_MASTER     = 9;
    DMAB_BLITHOG    = 10;
    DMAB_BLTDONE    = 14;
    DMAB_BLTNZERO   = 13;


Const

    INTB_SETCLR     = 15;   { Set/Clear control bit. Determines if bits }
                { written with a 1 get set or cleared. Bits }
                { written with a zero are allways unchanged }
    INTB_INTEN      = 14;   { Master interrupt (enable only ) }
    INTB_EXTER      = 13;   { External interrupt }
    INTB_DSKSYNC    = 12;   { Disk re-SYNChronized }
    INTB_RBF        = 11;   { serial port Receive Buffer Full }
    INTB_AUD3       = 10;   { Audio channel 3 block finished }
    INTB_AUD2       = 9;    { Audio channel 2 block finished }
    INTB_AUD1       = 8;    { Audio channel 1 block finished }
    INTB_AUD0       = 7;    { Audio channel 0 block finished }
    INTB_BLIT       = 6;    { Blitter finished }
    INTB_VERTB      = 5;    { start of Vertical Blank }
    INTB_COPER      = 4;    { Coprocessor }
    INTB_PORTS      = 3;    { I/O Ports and timers }
    INTB_SOFTINT    = 2;    { software interrupt request }
    INTB_DSKBLK     = 1;    { Disk Block done }
    INTB_TBE        = 0;    { serial port Transmit Buffer Empty }


    INTF_SETCLR     = $8000;
    INTF_INTEN      = $4000;
    INTF_EXTER      = $2000;
    INTF_DSKSYNC    = $1000;
    INTF_RBF        = $0800;
    INTF_AUD3       = $0400;
    INTF_AUD2       = $0200;
    INTF_AUD1       = $0100;
    INTF_AUD0       = $0080;
    INTF_BLIT       = $0040;
    INTF_VERTB      = $0020;
    INTF_COPER      = $0010;
    INTF_PORTS      = $0008;
    INTF_SOFTINT    = $0004;
    INTF_DSKBLK     = $0002;
    INTF_TBE        = $0001;

IMPLEMENTATION

end.
