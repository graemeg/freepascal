{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2000 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:
    Added overlay functions for Pchar->Strings, functions
    and procedures. Now you can mix PChar and Strings e.g
    OpenLibrary('whatis.library',37). No need to cast to
    a PChar.
    12 Oct 1998.

    More missing functions added.
    Added BitMask,
    IsListEmpty and
    IsMsgPortEmpty.
    22 Aug 2000.

    Bug found in
    AllocSignal,
    OpenDevice,
    SetTaskPri,
    WaitIO
    and DoIO.
    The stubs for this functions push a long for result,
    the result was defined as a shortint. If you tried
    to use any of those functions you get a big crash.
    Just changed the result to a longint.
    06 Sep 2000.

    Fixed the above functions so that they return a
    shortint as they should. Made some changes to the
    stub.
    20 Sep 2000.

    Put together exec.pp and exec.inc.
    04 Feb 2003.

    Update for AmigaOS 3.9.
    Added some consts and a record.
    Functions added.
         PROCEDURE NewMinList
         FUNCTION AVL_AddNode
         FUNCTION AVL_RemNodeByAddress
         FUNCTION AVL_RemNodeByKey
         FUNCTION AVL_FindNode
         FUNCTION AVL_FindPrevNodeByAddress
         FUNCTION AVL_FindPrevNodeByKey
         FUNCTION AVL_FindNextNodeByAddress
         FUNCTION AVL_FindNextNodeByKey
         FUNCTION AVL_FindFirstNode
         FUNCTION AVL_FindLastNode

    05 Feb 2003.

    Changed integer > smallint.
    Retyped ULONG to longword
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

UNIT EXEC;


INTERFACE

{
    History:

    Added BOOL = smallint, some libraries need that define
    (read triton, wizard)
    25 Oct 1998

    Added UWORD, WORDBITS, LONGBITS, PLONGBITS,
          UBYTE, PULONG, PAPTR, PLONG.
          For use with MUI.
    17 Jul 2000.

    nils.sjoholm@mailbox.swipnet.se
}

TYPE

       STRPTR   = PChar;
       ULONG    = Longword;
       LONG     = longint;
       APTR     = Pointer;
       BPTR     = Longint;
       BSTR     = Longint;
       BOOL     = smallint;
       UWORD    = Word;
       WORDBITS = Word;
       LONGBITS = longword;
       PLONGBITS = ^LONGBITS;
       UBYTE    = Byte;
       PULONG   = ^longword;
       PAPTR    = ^APTR;
       PLONG    = ^LONG;
       psmallint = ^smallint;

const
       {There is a problem with boolean
       vaules in taglists, just use this
       for now instead}
       ltrue  : longint = 1;
       lfalse : longint = 0;

TYPE

{ *  List Node Structure.  Each member in a list starts with a Node * }

  pNode = ^tNode;
  tNode =  Record
    ln_Succ,                { * Pointer to next (successor) * }
    ln_Pred  : pNode;       { * Pointer to previous (predecessor) * }
    ln_Type  : Byte;
    ln_Pri   : Shortint;        { * Priority, for sorting * }
    ln_Name  : STRPTR;      { * ID string, null terminated * }
  End;  { * Note: smallint aligned * }


{ * minimal node -- no type checking possible * }

  pMinNode = ^tMinNode;
  tMinNode =  Record
    mln_Succ,
    mln_Pred  : pMinNode;
  End;



{ *
** Note: Newly initialized IORequests, and software interrupt structures
** used with Cause(), should have type NT_UNKNOWN.  The OS will assign a type
** when they are first used.
* }

{ *----- Node Types for LN_TYPE -----* }

Const

  NT_UNKNOWN      =  0;
  NT_TASK     =  1;  { * Exec task * }
  NT_INTERRUPT    =  2;
  NT_DEVICE   =  3;
  NT_MSGPORT      =  4;
  NT_MESSAGE      =  5;  { * Indicates message currently pending * }
  NT_FREEMSG      =  6;
  NT_REPLYMSG     =  7;  { * Message has been replied * }
  NT_RESOURCE     =  8;
  NT_LIBRARY      =  9;
  NT_MEMORY   = 10;
  NT_SOFTINT      = 11;  { * Internal flag used by SoftInits * }
  NT_FONT     = 12;
  NT_PROCESS      = 13;  { * AmigaDOS Process * }
  NT_SEMAPHORE    = 14;
  NT_SIGNALSEM    = 15;  { * signal semaphores * }
  NT_BOOTNODE     = 16;
  NT_KICKMEM      = 17;
  NT_GRAPHICS     = 18;
  NT_DEATHMESSAGE = 19;

  NT_USER     = 254;  { * User node types work down from here * }
  NT_EXTENDED     = 255;

{
    This file defines Exec system lists, which are used to link
    various things.  Exec provides several routines to handle list
    processing (defined at the bottom of this file), so you can
    use these routines to save yourself the trouble of writing a list
    package.
}


Type

{ normal, full featured list }

    pList = ^tList;
    tList =  record
    lh_Head     : pNode;
    lh_Tail     : pNode;
    lh_TailPred : pNode;
    lh_Type     : Byte;
    l_pad       : Byte;
    end;

{ minimum list -- no type checking possible }

    pMinList = ^tMinList;
    tMinList =  record
    mlh_Head        : pMinNode;
    mlh_Tail        : pMinNode;
    mlh_TailPred    : pMinNode;
    end;



{ ********************************************************************
*
*  Format of the alert error number:
*
*    +-+-------------+----------------+--------------------------------+
*    |D|  SubSysId   |  General Error |    SubSystem Specific Error    |
*    +-+-------------+----------------+--------------------------------+
*     1    7 bits          8 bits                  16 bits
*
*                    D:  DeadEnd alert
*             SubSysId:  indicates ROM subsystem number.
*        General Error:  roughly indicates what the error was
*       Specific Error:  indicates more detail
*********************************************************************}

const
{*********************************************************************
*
*  Hardware/CPU specific alerts:  They may show without the 8 at the
*  front of the number.  These are CPU/68000 specific.  See 68$0
*  programmer's manuals for more details.
*
*********************************************************************}
    ACPU_BusErr     = $80000002;      { Hardware bus fault/access error }
    ACPU_AddressErr = $80000003;      { Illegal address access (ie: odd) }
    ACPU_InstErr    = $80000004;      { Illegal instruction }
    ACPU_DivZero    = $80000005;      { Divide by zero }
    ACPU_CHK        = $80000006;      { Check instruction error }
    ACPU_TRAPV      = $80000007;      { TrapV instruction error }
    ACPU_PrivErr    = $80000008;      { Privilege violation error }
    ACPU_Trace      = $80000009;      { Trace error }
    ACPU_LineA      = $8000000A;      { Line 1010 Emulator error }
    ACPU_LineF      = $8000000B;      { Line 1111 Emulator error }
    ACPU_Format     = $8000000E;      { Stack frame format error }
    ACPU_Spurious   = $80000018;      { Spurious interrupt error }
    ACPU_AutoVec1   = $80000019;      { AutoVector Level 1 interrupt error }
    ACPU_AutoVec2   = $8000001A;      { AutoVector Level 2 interrupt error }
    ACPU_AutoVec3   = $8000001B;      { AutoVector Level 3 interrupt error }
    ACPU_AutoVec4   = $8000001C;      { AutoVector Level 4 interrupt error }
    ACPU_AutoVec5   = $8000001D;      { AutoVector Level 5 interrupt error }
    ACPU_AutoVec6   = $8000001E;      { AutoVector Level 6 interrupt error }
    ACPU_AutoVec7   = $8000001F;      { AutoVector Level 7 interrupt error }


{ ********************************************************************
*
*  General Alerts
*
*  For example: timer.device cannot open math.library would be $05038015
*
*       Alert(AN_TimerDev|AG_OpenLib|AO_MathLib);
*
********************************************************************}


CONST

{ ------ alert types }
  AT_DeadEnd    = $80000000;
  AT_Recovery   = $00000000;


{ ------ general purpose alert codes }
  AG_NoMemory   = $00010000;
  AG_MakeLib    = $00020000;
  AG_OpenLib    = $00030000;
  AG_OpenDev    = $00040000;
  AG_OpenRes    = $00050000;
  AG_IOError    = $00060000;
  AG_NoSignal   = $00070000;
  AG_BadParm    = $00080000;
  AG_CloseLib   = $00090000;    { usually too many closes }
  AG_CloseDev   = $000A0000;    { or a mismatched close }
  AG_ProcCreate = $000B0000;    { Process creation failed }


{ ------ alert objects: }
  AO_ExecLib      = $00008001;
  AO_GraphicsLib  = $00008002;
  AO_LayersLib    = $00008003;
  AO_Intuition    = $00008004;
  AO_MathLib      = $00008005;
  AO_DOSLib       = $00008007;
  AO_RAMLib       = $00008008;
  AO_IconLib      = $00008009;
  AO_ExpansionLib = $0000800A;
  AO_DiskfontLib  = $0000800B;
  AO_UtilityLib   = $0000800C;
  AO_KeyMapLib    = $0000800D;

  AO_AudioDev     = $00008010;
  AO_ConsoleDev   = $00008011;
  AO_GamePortDev  = $00008012;
  AO_KeyboardDev  = $00008013;
  AO_TrackDiskDev = $00008014;
  AO_TimerDev     = $00008015;

  AO_CIARsrc    = $00008020;
  AO_DiskRsrc   = $00008021;
  AO_MiscRsrc   = $00008022;

  AO_BootStrap  = $00008030;
  AO_Workbench  = $00008031;
  AO_DiskCopy   = $00008032;
  AO_GadTools   = $00008033;
  AO_Unknown    = $00008035;



{ ********************************************************************
*
*   Specific Alerts:
*
********************************************************************}

{ ------ exec.library }

  AN_ExecLib    = $01000000;
  AN_ExcptVect  = $01000001; {  68000 exception vector checksum (obs.) }
  AN_BaseChkSum = $01000002; {  Execbase checksum (obs.) }
  AN_LibChkSum  = $01000003; {  Library checksum failure }

  AN_MemCorrupt = $81000005; {  Corrupt memory list detected in FreeMem }
  AN_IntrMem    = $81000006; {  No memory for interrupt servers }
  AN_InitAPtr   = $01000007; {  InitStruct() of an APTR source (obs.) }
  AN_SemCorrupt = $01000008; {  A semaphore is in an illegal state
                                      at ReleaseSempahore() }
  AN_FreeTwice    = $01000009; {  Freeing memory already freed }
  AN_BogusExcpt   = $8100000A; {  illegal 68k exception taken (obs.) }
  AN_IOUsedTwice  = $0100000B; {  Attempt to reuse active IORequest }
  AN_MemoryInsane = $0100000C; {  Sanity check on memory list failed
                                      during AvailMem(MEMF_LARGEST) }
  AN_IOAfterClose = $0100000D; {  IO attempted on closed IORequest }
  AN_StackProbe   = $0100000E; {  Stack appears to extend out of range }
  AN_BadFreeAddr  = $0100000F; {  Memory header not located. [ Usually an
                                  invalid address passed to FreeMem() ] }
  AN_BadSemaphore = $01000010; { An attempt was made to use the old
                                      message semaphores. }

{ ------ graphics.library }

  AN_GraphicsLib  = $02000000;
  AN_GfxNoMem     = $82010000;  {  graphics out of memory }
  AN_GfxNoMemMspc = $82010001;  {  MonitorSpec alloc, no memory }
  AN_LongFrame    = $82010006;  {  long frame, no memory }
  AN_ShortFrame   = $82010007;  {  short frame, no memory }
  AN_TextTmpRas   = $02010009;  {  text, no memory for TmpRas }
  AN_BltBitMap    = $8201000A;  {  BltBitMap, no memory }
  AN_RegionMemory = $8201000B;  {  regions, memory not available }
  AN_MakeVPort    = $82010030;  {  MakeVPort, no memory }
  AN_GfxNewError  = $0200000C;
  AN_GfxFreeError = $0200000D;

  AN_GfxNoLCM     = $82011234;  {  emergency memory not available }

  AN_ObsoleteFont = $02000401;  {  unsupported font description used }

{ ------ layers.library }

  AN_LayersLib    = $03000000;
  AN_LayersNoMem  = $83010000;  {  layers out of memory }

{ ------ intuition.library }
  AN_Intuition    = $04000000;
  AN_GadgetType   = $84000001;  {  unknown gadget type }
  AN_BadGadget    = $04000001;  {  Recovery form of AN_GadgetType }
  AN_CreatePort   = $84010002;  {  create port, no memory }
  AN_ItemAlloc    = $04010003;  {  item plane alloc, no memory }
  AN_SubAlloc     = $04010004;  {  sub alloc, no memory }
  AN_PlaneAlloc   = $84010005;  {  plane alloc, no memory }
  AN_ItemBoxTop   = $84000006;  {  item box top < RelZero }
  AN_OpenScreen   = $84010007;  {  open screen, no memory }
  AN_OpenScrnRast = $84010008;  {  open screen, raster alloc, no memory }
  AN_SysScrnType  = $84000009;  {  open sys screen, unknown type }
  AN_AddSWGadget  = $8401000A;  {  add SW gadgets, no memory }
  AN_OpenWindow   = $8401000B;  {  open window, no memory }
  AN_BadState     = $8400000C;  {  Bad State Return entering Intuition }
  AN_BadMessage   = $8400000D;  {  Bad Message received by IDCMP }
  AN_WeirdEcho    = $8400000E;  {  Weird echo causing incomprehension }
  AN_NoConsole    = $8400000F;  {  couldn't open the Console Device }
  AN_NoISem       = $04000010;  { Intuition skipped obtaining a sem }
  AN_ISemOrder    = $04000011;  { Intuition obtained a sem in bad order }

{ ------ math.library }

  AN_MathLib      = $05000000;

{ ------ dos.library }

  AN_DOSLib       = $07000000;
  AN_StartMem     = $07010001; {  no memory at startup }
  AN_EndTask      = $07000002; {  EndTask didn't }
  AN_QPktFail     = $07000003; {  Qpkt failure }
  AN_AsyncPkt     = $07000004; {  Unexpected packet received }
  AN_FreeVec      = $07000005; {  Freevec failed }
  AN_DiskBlkSeq   = $07000006; {  Disk block sequence error }
  AN_BitMap       = $07000007; {  Bitmap corrupt }
  AN_KeyFree      = $07000008; {  Key already free }
  AN_BadChkSum    = $07000009; {  Invalid checksum }
  AN_DiskError    = $0700000A; {  Disk Error }
  AN_KeyRange     = $0700000B; {  Key out of range }
  AN_BadOverlay   = $0700000C; {  Bad overlay }
  AN_BadInitFunc  = $0700000D; {  Invalid init packet for cli/shell }
  AN_FileReclosed = $0700000E; {  A filehandle was closed more than once }

{ ------ ramlib.library }

  AN_RAMLib       = $08000000;
  AN_BadSegList   = $08000001;  {  no overlays in library seglists }

{ ------ icon.library }

  AN_IconLib      = $09000000;

{ ------ expansion.library }

  AN_ExpansionLib       = $0A000000;
  AN_BadExpansionFree   = $0A000001; {  freeed free region }

{ ------ diskfont.library }

  AN_DiskfontLib        = $0B000000;

{ ------ audio.device }

  AN_AudioDev   = $10000000;

{ ------ console.device }

  AN_ConsoleDev = $11000000;
  AN_NoWindow   = $11000001;    {  Console can't open initial window }

{ ------ gameport.device }

  AN_GamePortDev        = $12000000;

{ ------ keyboard.device }

  AN_KeyboardDev        = $13000000;

{ ------ trackdisk.device }

  AN_TrackDiskDev = $14000000;
  AN_TDCalibSeek  = $14000001;  {  calibrate: seek error }
  AN_TDDelay      = $14000002;  {  delay: error on timer wait }

{ ------ timer.device }

  AN_TimerDev     = $15000000;
  AN_TMBadReq     = $15000001; {  bad request }
  AN_TMBadSupply  = $15000002; {  power supply -- no 50/60Hz ticks }

{ ------ cia.resource }

  AN_CIARsrc      = $20000000;

{ ------ disk.resource }

  AN_DiskRsrc   = $21000000;
  AN_DRHasDisk  = $21000001;    {  get unit: already has disk }
  AN_DRIntNoAct = $21000002;    {  interrupt: no active unit }

{ ------ misc.resource }

  AN_MiscRsrc   = $22000000;

{ ------ bootstrap }

  AN_BootStrap  = $30000000;
  AN_BootError  = $30000001;    {  boot code returned an error }

{ ------ Workbench }

  AN_Workbench          = $31000000;
  AN_NoFonts            = $B1000001;
  AN_WBBadStartupMsg1   = $31000001;
  AN_WBBadStartupMsg2   = $31000002;
  AN_WBBadIOMsg         = $31000003;

  AN_WBReLayoutToolMenu          = $B1010009;

{ ------ DiskCopy }

  AN_DiskCopy   = $32000000;

{ ------ toolkit for Intuition }

  AN_GadTools   = $33000000;

{ ------ System utility library }

  AN_UtilityLib = $34000000;

{ ------ For use by any application that needs it }

  AN_Unknown    = $35000000;



CONST

  IOERR_OPENFAIL   = -1;    {  device/unit failed to open  }
  IOERR_ABORTED    = -2;    {  request terminated early [after AbortIO()]  }
  IOERR_NOCMD      = -3;    {  command not supported by device  }
  IOERR_BADLENGTH  = -4;    {  not a valid length (usually IO_LENGTH)  }
  IOERR_BADADDRESS = -5;    {  invalid address (misaligned or bad range)  }
  IOERR_UNITBUSY   = -6;    {  device opens ok, but requested unit is busy  }
  IOERR_SELFTEST   = -7;    {  hardware failed self-test  }



type
    pResident = ^tResident;
    tResident =  record
    rt_MatchWord  : Word;        { smallint to match on (ILLEGAL)  }
    rt_MatchTag   : pResident;    { pointer to the above        }
    rt_EndSkip    : Pointer;      { address to continue scan    }
    rt_Flags      : Byte;        { various tag flags           }
    rt_Version    : Byte;        { release version number      }
    rt_Type       : Byte;        { type of module (NT_mumble)  }
    rt_Pri        : Shortint;         { initialization priority     }
    rt_Name       : STRPTR;       { pointer to node name        }
    rt_IdString   : STRPTR;       { pointer to ident string     }
    rt_Init       : Pointer;      { pointer to init code        }
    end;

const


    RTC_MATCHWORD   = $4AFC;

    RTF_AUTOINIT    = $80;
    RTF_AFTERDOS    = $04;
    RTF_SINGLETASK  = $02;
    RTF_COLDSTART   = $01;


{ Compatibility: }

    RTM_WHEN        = $03;
    RTW_COLDSTART   = $01;
    RTW_NEVER       = $00;



TYPE

{ ****** MemChunk **************************************************** }

  pMemChunk = ^tMemChunk;
  tMemChunk =  Record
    mc_Next  : pMemChunk;       { * pointer to next chunk * }
    mc_Bytes : ULONG;           { * chunk byte size     * }
  End;


{ ****** MemHeader *************************************************** }

  pMemHeader = ^tMemHeader;
  tMemHeader =  Record
    mh_Node       : tNode;
    mh_Attributes : Word;       { * characteristics of this region * }
    mh_First      : pMemChunk;   { * first free region          * }
    mh_Lower,                    { * lower memory bound         * }
    mh_Upper      : Pointer;     { * upper memory bound+1       * }
    mh_Free       : Ulong;       { * total number of free bytes * }
  End;


{ ****** MemEntry **************************************************** }

  pMemEntry = ^tMemEntry;
  tMemEntry =  record
           me_Un : record
                case longint of
                   0 : ( meu_Reqs : ULONG );
                   1 : ( meu_Addr : APTR );
                end;
            me_Length : ULONG;
         end;

{ ****** MemList ***************************************************** }

{ * Note: sizeof(struct MemList) includes the size of the first MemEntry! * }

  pMemList = ^tMemList;
  tMemList =  Record
    ml_Node       : tNode;
    ml_NumEntries : Word;      { * number of entries in this struct * }
    ml_ME         : Array [0..0] of tMemEntry;    { * the first entry * }
  End;

{ *----- Memory Requirement Types ---------------------------* }
{ *----- See the AllocMem() documentation for details--------* }

Const

   MEMF_ANY      = %000000000000000000000000;   { * Any type of memory will do * }
   MEMF_PUBLIC   = %000000000000000000000001;
   MEMF_CHIP     = %000000000000000000000010;
   MEMF_FAST     = %000000000000000000000100;
   MEMF_LOCAL    = %000000000000000100000000;
   MEMF_24BITDMA = %000000000000001000000000;   { * DMAable memory within 24 bits of address * }
   MEMF_KICK     = %000000000000010000000000;   { Memory that can be used for KickTags }

   MEMF_CLEAR    = %000000010000000000000000;
   MEMF_LARGEST  = %000000100000000000000000;
   MEMF_REVERSE  = %000001000000000000000000;
   MEMF_TOTAL    = %000010000000000000000000;   { * AvailMem: return total size of memory * }
   MEMF_NO_EXPUNGE = $80000000;   {AllocMem: Do not cause expunge on failure }

   MEM_BLOCKSIZE = 8;
   MEM_BLOCKMASK = MEM_BLOCKSIZE-1;

Type
{***** MemHandlerData *********************************************}
{ Note:  This structure is *READ ONLY* and only EXEC can create it!}

 pMemHandlerData = ^tMemHandlerData;
 tMemHandlerData =  Record
        memh_RequestSize,       { Requested allocation size }
        memh_RequestFlags,      { Requested allocation flags }
        memh_Flags  : ULONG;    { Flags (see below) }
 end;

const
    MEMHF_RECYCLE  = 1; { 0==First time, 1==recycle }

{***** Low Memory handler return values **************************}
    MEM_DID_NOTHING = 0;     { Nothing we could do... }
    MEM_ALL_DONE    = -1;    { We did all we could do }
    MEM_TRY_AGAIN   = 1;     { We did some, try the allocation again }


type
    pInterrupt = ^tInterrupt;
    tInterrupt =  record
        is_Node : tNode;
        is_Data : Pointer;      { Server data segment }
        is_Code : Pointer;      { Server code entry }
    end;

    pIntVector = ^tIntVector;
    tIntVector =  record          { For EXEC use ONLY! }
        iv_Data : Pointer;
        iv_Code : Pointer;
        iv_Node : pNode;
    end;

    pSoftIntList = ^tSoftIntList;
    tSoftIntList =  record        { For EXEC use ONLY! }
        sh_List : tList;
        sh_Pad  : Word;
    end;

const
    SIH_PRIMASK = $F0;

{ this is a fake INT definition, used only for AddIntServer and the like }

    INTB_NMI    = 15;
    INTF_NMI    = $0080;

{
    Every Amiga Task has one of these Task structures associated with it.
    To find yours, use FindTask(Nil).  AmigaDOS processes tack a few more
    values on to the end of this structure, which is the difference between
    Tasks and Processes.
}

type

    pTask = ^tTask;
    tTask =  record
        tc_Node         : tNode;
        tc_Flags        : Byte;
        tc_State        : Byte;
        tc_IDNestCnt    : Shortint;         { intr disabled nesting         }
        tc_TDNestCnt    : Shortint;         { task disabled nesting         }
        tc_SigAlloc     : ULONG;        { sigs allocated                }
        tc_SigWait      : ULONG;        { sigs we are waiting for       }
        tc_SigRecvd     : ULONG;        { sigs we have received         }
        tc_SigExcept    : ULONG;        { sigs we will take excepts for }
        tc_TrapAlloc    : Word;        { traps allocated               }
        tc_TrapAble     : Word;        { traps enabled                 }
        tc_ExceptData   : Pointer;      { points to except data         }
        tc_ExceptCode   : Pointer;      { points to except code         }
        tc_TrapData     : Pointer;      { points to trap data           }
        tc_TrapCode     : Pointer;      { points to trap code           }
        tc_SPReg        : Pointer;      { stack pointer                 }
        tc_SPLower      : Pointer;      { stack lower bound             }
        tc_SPUpper      : Pointer;      { stack upper bound + 2         }
        tc_Switch       : Pointer;      { task losing CPU               }
        tc_Launch       : Pointer;      { task getting CPU              }
        tc_MemEntry     : tList;        { allocated memory              }
        tc_UserData     : Pointer;      { per task data                 }
    end;

{
 * Stack swap structure as passed to StackSwap()
 }
  pStackSwapStruct = ^tStackSwapStruct;
  tStackSwapStruct =  Record
        stk_Lower       : Pointer;      { Lowest byte of stack }
        stk_Upper       : ULONG;        { Upper end of stack (size + Lowest) }
        stk_Pointer     : Pointer;      { Stack pointer at switch point }
  end;



{----- Flag Bits ------------------------------------------}

const

    TB_PROCTIME         = 0;
    TB_ETASK            = 3;
    TB_STACKCHK         = 4;
    TB_EXCEPT           = 5;
    TB_SWITCH           = 6;
    TB_LAUNCH           = 7;

    TF_PROCTIME         = 1;
    TF_ETASK            = 8;
    TF_STACKCHK         = 16;
    TF_EXCEPT           = 32;
    TF_SWITCH           = 64;
    TF_LAUNCH           = 128;

{----- Task States ----------------------------------------}

    TS_INVALID          = 0;
    TS_ADDED            = 1;
    TS_RUN              = 2;
    TS_READY            = 3;
    TS_WAIT             = 4;
    TS_EXCEPT           = 5;
    TS_REMOVED          = 6;

{----- Predefined Signals -------------------------------------}

    SIGB_ABORT          = 0;
    SIGB_CHILD          = 1;
    SIGB_BLIT           = 4;
    SIGB_SINGLE         = 4;
    SIGB_INTUITION      = 5;
    SIGB_DOS            = 8;

    SIGF_ABORT          = 1;
    SIGF_CHILD          = 2;
    SIGF_BLIT           = 16;
    SIGF_SINGLE         = 16;
    SIGF_INTUITION      = 32;
    SIGF_DOS            = 256;



{
    This file defines ports and messages, which are used for inter-
    task communications using the routines defined toward the
    bottom of this file.
}

type

{****** MsgPort *****************************************************}

    pMsgPort = ^tMsgPort;
    tMsgPort = record
    mp_Node     : tNode;
    mp_Flags    : Byte;
    mp_SigBit   : Byte;     { signal bit number    }
    mp_SigTask  : Pointer;   { task to be signalled (TaskPtr) }
    mp_MsgList  : tList;     { message linked list  }
    end;

{****** Message *****************************************************}

    pMessage = ^tMessage;
    tMessage =  record
    mn_Node       : tNode;
    mn_ReplyPort  : pMsgPort;   { message reply port }
    mn_Length     : Word;      { message len in bytes }
    end;



{ mp_Flags: Port arrival actions (PutMsg) }

CONST

  PF_ACTION = 3;    { * Mask * }
  PA_SIGNAL = 0;    { * Signal task in mp_SigTask * }
  PA_SOFTINT    = 1;    { * Signal SoftInt in mp_SoftInt/mp_SigTask * }
  PA_IGNORE = 2;    { * Ignore arrival * }


        { Semaphore }
type
    pSemaphore = ^tSemaphore;
    tSemaphore =  record
        sm_MsgPort : tMsgPort;
        sm_Bids    : smallint;
    end;

{  This is the structure used to request a signal semaphore }

    pSemaphoreRequest = ^tSemaphoreRequest;
    tSemaphoreRequest =  record
        sr_Link    : tMinNode;
        sr_Waiter  : pTask;
    end;

{ The actual semaphore itself }

    pSignalSemaphore = ^tSignalSemaphore;
    tSignalSemaphore =  record
        ss_Link         : tNode;
        ss_NestCount    : smallint;
        ss_WaitQueue    : tMinList;
        ss_MultipleLink : tSemaphoreRequest;
        ss_Owner        : pTask;
        ss_QueueCount   : smallint;
    end;


{  ***** Semaphore procure message (for use in V39 Procure/Vacate *** }


 pSemaphoreMessage = ^tSemaphoreMessage;
 tSemaphoreMessage =  Record
   ssm_Message   : tMessage;
   ssm_Semaphore : pSignalSemaphore;
 end;

const
 SM_SHARED      = 1;
 SM_EXCLUSIVE   = 0;


CONST

{ ------ Special Constants --------------------------------------- }
  LIB_VECTSIZE  =  6;   {  Each library entry takes 6 bytes  }
  LIB_RESERVED  =  4;   {  Exec reserves the first 4 vectors  }
  LIB_BASE  = (-LIB_VECTSIZE);
  LIB_USERDEF   = (LIB_BASE-(LIB_RESERVED*LIB_VECTSIZE));
  LIB_NONSTD    = (LIB_USERDEF);

{ ------ Standard Functions -------------------------------------- }
  LIB_OPEN  =  -6;
  LIB_CLOSE = -12;
  LIB_EXPUNGE   = -18;
  LIB_EXTFUNC   = -24;  {  for future expansion  }

TYPE

{ ------ Library Base Structure ---------------------------------- }
{  Also used for Devices and some Resources  }

    pLibrary = ^tLibrary;
    tLibrary =  record
        lib_Node     : tNode;
        lib_Flags,
        lib_pad      : Byte;
        lib_NegSize,            {  number of bytes before library  }
        lib_PosSize,            {  number of bytes after library  }
        lib_Version,            {  major  }
        lib_Revision : Word;   {  minor  }
        lib_IdString : STRPTR;  {  ASCII identification  }
        lib_Sum      : ULONG;   {  the checksum itself  }
        lib_OpenCnt  : Word;   {  number of current opens  }
    end;                {  * Warning: size is not a longword multiple ! * }

CONST

{  lib_Flags bit definitions (all others are system reserved)  }

  LIBF_SUMMING = %00000001; {  we are currently checksumming  }
  LIBF_CHANGED = %00000010; {  we have just changed the lib  }
  LIBF_SUMUSED = %00000100; {  set if we should bother to sum  }
  LIBF_DELEXP  = %00001000; {  delayed expunge  }

{
    This file defines the constants and types required to use
    Amiga device IO routines, which are also defined here.
}


TYPE

{***** Device *****************************************************}
  pDevice = ^tDevice;
  tDevice =  record
    dd_Library : tLibrary;
  end;

{***** Unit *******************************************************}
  pUnit = ^tUnit;
  tUnit = record
      unit_MsgPort : tMsgPort;     { queue for unprocessed messages }
                    { instance of msgport is recommended }
      unit_flags,
      unit_pad     : Byte;
      unit_OpenCnt : Word;       { number of active opens }
  end;

Const
  UNITF_ACTIVE  = %00000001;
  UNITF_INTASK  = %00000010;

type

    pIORequest = ^tIORequest;
    tIORequest =  record
    io_Message  : tMessage;
    io_Device   : pDevice;      { device node pointer  }
    io_Unit     : pUnit;        { unit (driver private)}
    io_Command  : Word;        { device command }
    io_Flags    : Byte;
    io_Error    : Shortint;         { error or warning num }
    end;

    pIOStdReq = ^tIOStdReq;
    tIOStdReq =  record
    io_Message  : tMessage;
    io_Device   : pDevice;      { device node pointer  }
    io_Unit     : pUnit;        { unit (driver private)}
    io_Command  : Word;        { device command }
    io_Flags    : Byte;
    io_Error    : Shortint;         { error or warning num }
    io_Actual   : ULONG;        { actual number of bytes transferred }
    io_Length   : ULONG;        { requested number bytes transferred}
    io_Data     : Pointer;      { points to data area }
    io_Offset   : ULONG;        { offset for block structured devices }
    end;


{ library vector offsets for device reserved vectors }

const
    DEV_BEGINIO = -30;
    DEV_ABORTIO = -36;

{ io_Flags defined bits }

    IOB_QUICK   = 0;
    IOF_QUICK   = 1;

    CMD_INVALID = 0;
    CMD_RESET   = 1;
    CMD_READ    = 2;
    CMD_WRITE   = 3;
    CMD_UPDATE  = 4;
    CMD_CLEAR   = 5;
    CMD_STOP    = 6;
    CMD_START   = 7;
    CMD_FLUSH   = 8;

    CMD_NONSTD  = 9;




{  Definition of the Exec library base structure (pointed to by location 4).
** Most fields are not to be viewed or modified by user programs.  Use
** extreme caution.
 }

type

pExecBase = ^tExecBase;
tExecBase =  Record
        LibNode    : tLibrary;   {  Standard library node  }

{ ******* Static System Variables ******* }

        SoftVer      : Word;   {  kickstart release number (obs.)  }
        LowMemChkSum : smallint;    {  checksum of 68000 trap vectors  }
        ChkBase      : ULONG;   {  system base pointer complement  }
        ColdCapture,            {  coldstart soft capture vector  }
        CoolCapture,            {  coolstart soft capture vector  }
        WarmCapture,            {  warmstart soft capture vector  }
        SysStkUpper,            {  system stack base   (upper bound)  }
        SysStkLower  : Pointer; {  top of system stack (lower bound)  }
        MaxLocMem    : ULONG;   {  top of chip memory  }
        DebugEntry,             {  global debugger entry point  }
        DebugData,              {  global debugger data segment  }
        AlertData,              {  alert data segment  }
        MaxExtMem    : Pointer; {  top of extended mem, or null if none  }

        ChkSum       : Word;   {  for all of the above (minus 2)  }

{ ***** Interrupt Related ************************************** }

        IntVects     : Array[0..15] of tIntVector;

{ ***** Dynamic System Variables ************************************ }

        ThisTask     : pTask;   {  pointer to current task (readable)  }

        IdleCount,              {  idle counter  }
        DispCount    : ULONG;   {  dispatch counter  }
        Quantum,                {  time slice quantum  }
        Elapsed,                {  current quantum ticks  }
        SysFlags     : Word;   {  misc internal system flags  }
        IDNestCnt,              {  interrupt disable nesting count  }
        TDNestCnt    : Shortint;    {  task disable nesting count  }

        AttnFlags,              {  special attention flags (readable)  }
        AttnResched  : Word;   {  rescheduling attention  }
        ResModules,             {  resident module array pointer  }
        TaskTrapCode,
        TaskExceptCode,
        TaskExitCode : Pointer;
        TaskSigAlloc : ULONG;
        TaskTrapAlloc: Word;


{ ***** System Lists (private!) ******************************* }

        MemList,
        ResourceList,
        DeviceList,
        IntrList,
        LibList,
        PortList,
        TaskReady,
        TaskWait     : tList;

        SoftInts     : Array[0..4] of tSoftIntList;

{ ***** Other Globals ****************************************** }

        LastAlert    : Array[0..3] of LONG;

        {  these next two variables are provided to allow
        ** system developers to have a rough idea of the
        ** period of two externally controlled signals --
        ** the time between vertical blank interrupts and the
        ** external line rate (which is counted by CIA A's
        ** "time of day" clock).  In general these values
        ** will be 50 or 60, and may or may not track each
        ** other.  These values replace the obsolete AFB_PAL
        ** and AFB_50HZ flags.
         }

        VBlankFrequency,                {  (readable)  }
        PowerSupplyFrequency : Byte;   {  (readable)  }

        SemaphoreList    : tList;

        {  these next two are to be able to kickstart into user ram.
        ** KickMemPtr holds a singly linked list of MemLists which
        ** will be removed from the memory list via AllocAbs.  If
        ** all the AllocAbs's succeeded, then the KickTagPtr will
        ** be added to the rom tag list.
         }

        KickMemPtr,             {  ptr to queue of mem lists  }
        KickTagPtr,             {  ptr to rom tag queue  }
        KickCheckSum : Pointer; {  checksum for mem and tags  }

{ ***** V36 Exec additions start here ************************************* }

        ex_Pad0           : Word;
        ex_Reserved0      : ULONG;
        ex_RamLibPrivate  : Pointer;

        {  The next ULONG contains the system "E" clock frequency,
        ** expressed in Hertz.  The E clock is used as a timebase for
        ** the Amiga's 8520 I/O chips. (E is connected to "02").
        ** Typical values are 715909 for NTSC, or 709379 for PAL.
         }

        ex_EClockFrequency,         {  (readable)  }
        ex_CacheControl,            {  Private to CacheControl calls  }
        ex_TaskID         : ULONG;  {  Next available task ID  }

        ex_Reserved1      : Array[0..4] of ULONG;

        ex_MMULock        : Pointer;    {  private  }

        ex_Reserved2      : Array[0..2] of ULONG;
{***** V39 Exec additions start here *************************************}

        { The following list and data element are used
         * for V39 exec's low memory handler...
         }
        ex_MemHandlers    : tMinList; { The handler list }
        ex_MemHandler     : Pointer;          { Private! handler pointer }
end;


{ ***** Bit defines for AttnFlags (see above) ***************************** }

{   Processors and Co-processors:  }

CONST

  AFB_68010     = 0;    {  also set for 68020  }
  AFB_68020     = 1;    {  also set for 68030  }
  AFB_68030     = 2;    {  also set for 68040  }
  AFB_68040     = 3;
  AFB_68881     = 4;    {  also set for 68882  }
  AFB_68882     = 5;
  AFB_FPU40     = 6;    {  Set if 68040 FPU }
  AFB_68060     = 7;

  AFF_68010     = %00000001;
  AFF_68020     = %00000010;
  AFF_68030     = %00000100;
  AFF_68040     = %00001000;
  AFF_68881     = %00010000;
  AFF_68882     = %00100000;
  AFF_FPU40     = %01000000;
  AFF_68060     = (1 shl 7);

{    AFB_RESERVED8 = %000100000000;  }
{    AFB_RESERVED9 = %001000000000;  }


{ ***** Selected flag definitions for Cache manipulation calls ********* }

  CACRF_EnableI       = %0000000000000001;  { Enable instruction cache  }
  CACRF_FreezeI       = %0000000000000010;  { Freeze instruction cache  }
  CACRF_ClearI        = %0000000000001000;  { Clear instruction cache   }
  CACRF_IBE           = %0000000000010000;  { Instruction burst enable  }
  CACRF_EnableD       = %0000000100000000;  { 68030 Enable data cache   }
  CACRF_FreezeD       = %0000001000000000;  { 68030 Freeze data cache   }
  CACRF_ClearD        = %0000100000000000;  { 68030 Clear data cache    }
  CACRF_DBE           = %0001000000000000;  { 68030 Data burst enable   }
  CACRF_WriteAllocate = %0010000000000000;  { 68030 Write-Allocate mode
                                              (must always be set!)     }
  CACRF_EnableE       = 1073741824;  { Master enable for external caches }
                                     { External caches should track the }
                                     { state of the internal caches }
                                     { such that they do not cache anything }
                                     { that the internal cache turned off }
                                     { for. }

  CACRF_CopyBack      = $80000000;  { Master enable for copyback caches }

  DMA_Continue        = 2;      { Continuation flag for CachePreDMA }
  DMA_NoModify        = 4;      { Set if DMA does not update memory }
  DMA_ReadFromRAM     = 8;      { Set if DMA goes *FROM* RAM to device }


{ Don't even think about the contents of this structure. Just embed it
 * and reference it
 *}
  type
     PAVLNode = ^tAVLNode;
     tAVLNode = record
          reserved : array[0..3] of ULONG;
       end;
     ppAVLNode = ^pAVLNode;


       PAVLNODECOMP = ^AVLNODECOMP;
       AVLNODECOMP = APTR;

       PAVLKEYCOMP = ^AVLKEYCOMP;
       AVLKEYCOMP = APTR;



PROCEDURE AbortIO(ioRequest : pIORequest);
PROCEDURE AddDevice(device : pDevice);
PROCEDURE AddHead(list : pList; node : pNode);
PROCEDURE AddIntServer(intNumber : LONGINT; interrupt_ : pInterrupt);
PROCEDURE AddLibrary(lib : pLibrary);
PROCEDURE AddMemHandler(memhand : pInterrupt);
PROCEDURE AddMemList(size : ULONG; attributes : ULONG; pri : LONGINT; base : POINTER; const name : pCHAR);
PROCEDURE AddPort(port : pMsgPort);
PROCEDURE AddResource(resource : POINTER);
PROCEDURE AddSemaphore(sigSem : pSignalSemaphore);
PROCEDURE AddTail(list : pList; node : pNode);
FUNCTION AddTask(task : pTask;const initPC : POINTER;const finalPC : POINTER) : POINTER;
PROCEDURE Alert(alertNum : ULONG);
FUNCTION AllocAbs(byteSize : ULONG; location : POINTER) : POINTER;
FUNCTION Allocate(freeList : pMemHeader; byteSize : ULONG) : POINTER;
FUNCTION AllocEntry(entry : pMemList) : pMemList;
FUNCTION AllocMem(byteSize : ULONG; requirements : ULONG) : POINTER;
FUNCTION AllocPooled(poolHeader : POINTER; memSize : ULONG) : POINTER;
FUNCTION AllocSignal(signalNum : LONGINT) : shortint;
FUNCTION AllocTrap(trapNum : LONGINT) : LONGINT;
FUNCTION AllocVec(byteSize : ULONG; requirements : ULONG) : POINTER;
FUNCTION AttemptSemaphore(sigSem : pSignalSemaphore) : BOOLEAN;
FUNCTION AttemptSemaphoreShared(sigSem : pSignalSemaphore) : ULONG;
FUNCTION AvailMem(requirements : ULONG) : ULONG;
PROCEDURE CacheClearE(address : POINTER; length : ULONG; caches : ULONG);
PROCEDURE CacheClearU;
FUNCTION CacheControl(cacheBits : ULONG; cacheMask : ULONG) : ULONG;
PROCEDURE CachePostDMA(const address : POINTER; VAR length : ULONG; flags : ULONG);
FUNCTION CachePreDMA(const address : POINTER; VAR length : ULONG; flags : ULONG) : POINTER;
PROCEDURE Cause(interrupt_ : pInterrupt);
FUNCTION CheckIO(ioRequest : pIORequest) : pIORequest;
PROCEDURE ChildFree(tid : POINTER);
PROCEDURE ChildOrphan(tid : POINTER);
PROCEDURE ChildStatus(tid : POINTER);
PROCEDURE ChildWait(tid : POINTER);
PROCEDURE CloseDevice(ioRequest : pIORequest);
PROCEDURE CloseLibrary(lib : pLibrary);
PROCEDURE ColdReboot;
PROCEDURE CopyMem(const source : POINTER; dest : POINTER; size : ULONG);
PROCEDURE CopyMemQuick(const source : POINTER; dest : POINTER; size : ULONG);
FUNCTION CreateIORequest(const port : pMsgPort; size : ULONG) : POINTER;
FUNCTION CreateMsgPort : pMsgPort;
FUNCTION CreatePool(requirements : ULONG; puddleSize : ULONG; threshSize : ULONG) : POINTER;
PROCEDURE Deallocate(freeList : pMemHeader; memoryBlock : POINTER; byteSize : ULONG);
PROCEDURE Debug(flags : ULONG);
PROCEDURE DeleteIORequest(iorequest : POINTER);
PROCEDURE DeleteMsgPort(port : pMsgPort);
PROCEDURE DeletePool(poolHeader : POINTER);
PROCEDURE Disable;
FUNCTION DoIO(ioRequest : pIORequest) : shortint;
PROCEDURE Enable;
PROCEDURE Enqueue(list : pList; node : pNode);
PROCEDURE ExecFreeMem(memoryBlock : POINTER; byteSize : ULONG);
PROCEDURE ExecInsert(list : pList; node : pNode; pred : pNode);
FUNCTION FindName(list : pList; const name : pCHAR) : pNode;
FUNCTION FindPort(const name : pCHAR) : pMsgPort;
FUNCTION FindResident(const name : pCHAR) : pResident;
FUNCTION FindSemaphore(const sigSem : pCHAR) : pSignalSemaphore;
FUNCTION FindTask(const name : pCHAR) : pTask;
PROCEDURE Forbid;
PROCEDURE FreeEntry(entry : pMemList);
PROCEDURE FreePooled(poolHeader : POINTER; memory : POINTER; memSize : ULONG);
PROCEDURE FreeSignal(signalNum : LONGINT);
PROCEDURE FreeTrap(trapNum : LONGINT);
PROCEDURE FreeVec(memoryBlock : POINTER);
FUNCTION GetCC : ULONG;
FUNCTION GetMsg(port : pMsgPort) : pMessage;
PROCEDURE InitCode(startClass : ULONG; version : ULONG);
FUNCTION InitResident(const resident_ : pResident; segList : ULONG) : POINTER;
PROCEDURE InitSemaphore(sigSem : pSignalSemaphore);
PROCEDURE InitStruct(const initTable : POINTER; memory : POINTER; size : ULONG);
PROCEDURE MakeFunctions(const target : POINTER;const functionArray : POINTER;const funcDispBase :pointer);
FUNCTION MakeLibrary(const  funcInit : POINTER;const  structInit : POINTER; libInit : tPROCEDURE;dataSize : ULONG; segList : ULONG) : pLibrary;
FUNCTION ObtainQuickVector(interruptCode : POINTER) : ULONG;
PROCEDURE ObtainSemaphore(sigSem : pSignalSemaphore);
PROCEDURE ObtainSemaphoreList(sigSem : pList);
PROCEDURE ObtainSemaphoreShared(sigSem : pSignalSemaphore);
FUNCTION OldOpenLibrary(const libName : pCHAR) : pLibrary;
FUNCTION OpenDevice(const devName : pCHAR; unite : ULONG; ioRequest : pIORequest; flags : ULONG) : shortint;
FUNCTION OpenLibrary(const libName : pCHAR; version : ULONG) : pLibrary;
FUNCTION OpenResource(const resName : pCHAR) : POINTER;
PROCEDURE Permit;
FUNCTION Procure(sigSem : pSignalSemaphore; bidMsg : pSemaphoreMessage) : BOOLEAN;
PROCEDURE PutMsg(port : pMsgPort; message : pMessage);
function RawDoFmt(const formatString : pCHAR;const dataStream : POINTER; putChProc : tPROCEDURE; putChData : POINTER): pointer;
PROCEDURE ReleaseSemaphore(sigSem : pSignalSemaphore);
PROCEDURE ReleaseSemaphoreList(sigSem : pList);
PROCEDURE RemDevice(device : pDevice);
FUNCTION RemHead(list : pList) : pNode;
PROCEDURE RemIntServer(intNumber : LONGINT; interrupt_ : pInterrupt);
PROCEDURE RemLibrary(lib : pLibrary);
PROCEDURE RemMemHandler(memhand : pInterrupt);
PROCEDURE Remove(node : pNode);
PROCEDURE RemPort(port : pMsgPort);
PROCEDURE RemResource(resource : POINTER);
PROCEDURE RemSemaphore(sigSem : pSignalSemaphore);
FUNCTION RemTail(list : pList) : pNode;
PROCEDURE RemTask(task : pTask);
PROCEDURE ReplyMsg(message : pMessage);
PROCEDURE SendIO(ioRequest : pIORequest);
FUNCTION SetExcept(newSignals : ULONG; signalSet : ULONG) : ULONG;
FUNCTION SetFunction(lib : pLibrary; funcOffset : LONGINT; newFunction : tPROCEDURE) : POINTER;
FUNCTION SetIntVector(intNumber : LONGINT;const interrupt_ : pInterrupt) : pInterrupt;
FUNCTION SetSignal(newSignals : ULONG; signalSet : ULONG) : ULONG;
FUNCTION SetSR(newSR : ULONG; mask : ULONG) : ULONG;
FUNCTION SetTaskPri(task : pTask; priority : LONGINT) : shortint;
PROCEDURE Signal(task : pTask; signalSet : ULONG);
PROCEDURE StackSwap(newStack : pStackSwapStruct);
PROCEDURE SumKickData;
PROCEDURE SumLibrary(lib : pLibrary);
FUNCTION SuperState : POINTER;
FUNCTION Supervisor(userFunction : tPROCEDURE) : ULONG;
FUNCTION TypeOfMem(const address : POINTER) : ULONG;
PROCEDURE UserState(sysStack : POINTER);
PROCEDURE Vacate(sigSem : pSignalSemaphore; bidMsg : pSemaphoreMessage);
FUNCTION Wait(signalSet : ULONG) : ULONG;
FUNCTION WaitIO(ioRequest : pIORequest) : shortint;
FUNCTION WaitPort(port : pMsgPort) : pMessage;

PROCEDURE NewMinList(minlist : pMinList);
FUNCTION AVL_AddNode(root : ppAVLNode; node : pAVLNode; func : POINTER) : pAVLNode;
FUNCTION AVL_RemNodeByAddress(root : ppAVLNode; node : pAVLNode) : pAVLNode;
FUNCTION AVL_RemNodeByKey(root : ppAVLNode; key : POINTER; func : POINTER) : pAVLNode;
FUNCTION AVL_FindNode(CONST root : pAVLNode; key : POINTER; func : POINTER) : pAVLNode;
FUNCTION AVL_FindPrevNodeByAddress(CONST node : pAVLNode) : pAVLNode;
FUNCTION AVL_FindPrevNodeByKey(CONST root : pAVLNode; key : POINTER; func : POINTER) : pAVLNode;
FUNCTION AVL_FindNextNodeByAddress(CONST node : pAVLNode) : pAVLNode;
FUNCTION AVL_FindNextNodeByKey(CONST root : pAVLNode; key : POINTER; func : POINTER) : pAVLNode;
FUNCTION AVL_FindFirstNode(CONST root : pAVLNode) : pAVLNode;
FUNCTION AVL_FindLastNode(CONST root : pAVLNode) : pAVLNode;

PROCEDURE AddMemList(size : ULONG; attributes : ULONG; pri : LONGINT; base : POINTER; const name : String);
FUNCTION FindName(list : pList; const name : String) : pNode;
FUNCTION FindPort(const name : String) : pMsgPort;
FUNCTION FindResident(const name : String) : pResident;
FUNCTION FindSemaphore(const sigSem : String) : pSignalSemaphore;
FUNCTION FindTask(const name : String) : pTask;
FUNCTION OldOpenLibrary(const libName : String) : pLibrary;
FUNCTION OpenDevice(const devName : String; unite : ULONG; ioRequest : pIORequest;flags : ULONG) : shortint;
FUNCTION OpenLibrary(const libName : String; version : ULONG) : pLibrary;
FUNCTION OpenResource(const resName : String) : POINTER;
function RawDoFmt(const formatString : String;const dataStream : POINTER; putChProc :tPROCEDURE; putChData : POINTER): pointer;

function BitMask(no :shortint): longint;
function IsListEmpty( list : pList): boolean;
function IsMsgPortEmpty( mp : pMsgPort): boolean;

IMPLEMENTATION

uses pastoc;

function BitMask(no :shortint): longint;
begin
   BitMask := 1 shl no;
end;

function IsListEmpty( list : pList): boolean;
begin
     IsListEmpty := list^.lh_TailPred = pnode(list);
end;

function IsMsgPortEmpty( mp : pMsgPort): boolean;
begin
     with mp^ do
         IsMsgPortEmpty := mp_MsgList.lh_TailPred = pNode(@mp_MsgList);
end;

PROCEDURE AbortIO(ioRequest : pIORequest);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ioRequest,A1
    MOVEA.L _ExecBase,A6
    JSR -480(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddDevice(device : pDevice);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L device,A1
    MOVEA.L _ExecBase,A6
    JSR -432(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddHead(list : pList; node : pNode);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L node,A1
    MOVEA.L _ExecBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddIntServer(intNumber : LONGINT; interrupt_ : pInterrupt);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  intNumber,D0
    MOVEA.L interrupt_,A1
    MOVEA.L _ExecBase,A6
    JSR -168(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddLibrary(lib : pLibrary);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lib,A1
    MOVEA.L _ExecBase,A6
    JSR -396(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddMemHandler(memhand : pInterrupt);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L memhand,A1
    MOVEA.L _ExecBase,A6
    JSR -774(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddMemList(size : ULONG; attributes : ULONG; pri : LONGINT; base : POINTER; const name : pCHAR);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  size,D0
    MOVE.L  attributes,D1
    MOVE.L  pri,D2
    MOVEA.L base,A0
    MOVEA.L name,A1
    MOVEA.L _ExecBase,A6
    JSR -618(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddPort(port : pMsgPort);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A1
    MOVEA.L _ExecBase,A6
    JSR -354(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddResource(resource : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L resource,A1
    MOVEA.L _ExecBase,A6
    JSR -486(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddSemaphore(sigSem : pSignalSemaphore);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A1
    MOVEA.L _ExecBase,A6
    JSR -600(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AddTail(list : pList; node : pNode);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L node,A1
    MOVEA.L _ExecBase,A6
    JSR -246(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AddTask(task : pTask;const initPC : POINTER;const finalPC : POINTER) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L task,A1
    MOVEA.L initPC,A2
    MOVEA.L finalPC,A3
    MOVEA.L _ExecBase,A6
    JSR -282(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE Alert(alertNum : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  alertNum,D7
    MOVEA.L _ExecBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AllocAbs(byteSize : ULONG; location : POINTER) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  byteSize,D0
    MOVEA.L location,A1
    MOVEA.L _ExecBase,A6
    JSR -204(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION Allocate(freeList : pMemHeader; byteSize : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L freeList,A0
    MOVE.L  byteSize,D0
    MOVEA.L _ExecBase,A6
    JSR -186(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocEntry(entry : pMemList) : pMemList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L entry,A0
    MOVEA.L _ExecBase,A6
    JSR -222(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocMem(byteSize : ULONG; requirements : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  byteSize,D0
    MOVE.L  requirements,D1
    MOVEA.L _ExecBase,A6
    JSR -198(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocPooled(poolHeader : POINTER; memSize : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L poolHeader,A0
    MOVE.L  memSize,D0
    MOVEA.L _ExecBase,A6
    JSR -708(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocSignal(signalNum : LONGINT) : shortint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  signalNum,D0
    MOVEA.L _ExecBase,A6
    JSR -330(A6)
    MOVEA.L (A7)+,A6
    MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AllocTrap(trapNum : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  trapNum,D0
    MOVEA.L _ExecBase,A6
    JSR -342(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocVec(byteSize : ULONG; requirements : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  byteSize,D0
    MOVE.L  requirements,D1
    MOVEA.L _ExecBase,A6
    JSR -684(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AttemptSemaphore(sigSem : pSignalSemaphore) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -576(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AttemptSemaphoreShared(sigSem : pSignalSemaphore) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -720(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AvailMem(requirements : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  requirements,D1
    MOVEA.L _ExecBase,A6
    JSR -216(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE CacheClearE(address : POINTER; length : ULONG; caches : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L address,A0
    MOVE.L  length,D0
    MOVE.L  caches,D1
    MOVEA.L _ExecBase,A6
    JSR -642(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CacheClearU;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -636(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CacheControl(cacheBits : ULONG; cacheMask : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  cacheBits,D0
    MOVE.L  cacheMask,D1
    MOVEA.L _ExecBase,A6
    JSR -648(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE CachePostDMA(const address : POINTER; VAR length : ULONG; flags : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L address,A0
    MOVEA.L length,A1
    MOVE.L  flags,D0
    MOVEA.L _ExecBase,A6
    JSR -768(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CachePreDMA(const address : POINTER; VAR length : ULONG; flags : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L address,A0
    MOVEA.L length,A1
    MOVE.L  flags,D0
    MOVEA.L _ExecBase,A6
    JSR -762(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE Cause(interrupt_ : pInterrupt);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L interrupt_,A1
    MOVEA.L _ExecBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CheckIO(ioRequest : pIORequest) : pIORequest;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ioRequest,A1
    MOVEA.L _ExecBase,A6
    JSR -468(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ChildFree(tid : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  tid,D0
    MOVEA.L _ExecBase,A6
    JSR -738(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ChildOrphan(tid : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  tid,D0
    MOVEA.L _ExecBase,A6
    JSR -744(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ChildStatus(tid : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  tid,D0
    MOVEA.L _ExecBase,A6
    JSR -750(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ChildWait(tid : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  tid,D0
    MOVEA.L _ExecBase,A6
    JSR -756(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CloseDevice(ioRequest : pIORequest);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ioRequest,A1
    MOVEA.L _ExecBase,A6
    JSR -450(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CloseLibrary(lib : pLibrary);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lib,A1
    MOVEA.L _ExecBase,A6
    JSR -414(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ColdReboot;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -726(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CopyMem(const source : POINTER; dest : POINTER; size : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L source,A0
    MOVEA.L dest,A1
    MOVE.L  size,D0
    MOVEA.L _ExecBase,A6
    JSR -624(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE CopyMemQuick(const source : POINTER; dest : POINTER; size : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L source,A0
    MOVEA.L dest,A1
    MOVE.L  size,D0
    MOVEA.L _ExecBase,A6
    JSR -630(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CreateIORequest(const port : pMsgPort; size : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A0
    MOVE.L  size,D0
    MOVEA.L _ExecBase,A6
    JSR -654(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateMsgPort : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -666(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreatePool(requirements : ULONG; puddleSize : ULONG; threshSize : ULONG) :
POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  requirements,D0
    MOVE.L  puddleSize,D1
    MOVE.L  threshSize,D2
    MOVEA.L _ExecBase,A6
    JSR -696(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE Deallocate(freeList : pMemHeader; memoryBlock : POINTER; byteSize : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L freeList,A0
    MOVEA.L memoryBlock,A1
    MOVE.L  byteSize,D0
    MOVEA.L _ExecBase,A6
    JSR -192(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE Debug(flags : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  flags,D0
    MOVEA.L _ExecBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteIORequest(iorequest : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L iorequest,A0
    MOVEA.L _ExecBase,A6
    JSR -660(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteMsgPort(port : pMsgPort);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A0
    MOVEA.L _ExecBase,A6
    JSR -672(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeletePool(poolHeader : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L poolHeader,A0
    MOVEA.L _ExecBase,A6
    JSR -702(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE Disable;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DoIO(ioRequest : pIORequest) : shortint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ioRequest,A1
    MOVEA.L _ExecBase,A6
    JSR -456(A6)
    MOVEA.L (A7)+,A6
    MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE Enable;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE Enqueue(list : pList; node : pNode);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L node,A1
    MOVEA.L _ExecBase,A6
    JSR -270(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ExecFreeMem(memoryBlock : POINTER; byteSize : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L memoryBlock,A1
    MOVE.L  byteSize,D0
    MOVEA.L _ExecBase,A6
    JSR -210(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ExecInsert(list : pList; node : pNode; pred : pNode);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L node,A1
    MOVEA.L pred,A2
    MOVEA.L _ExecBase,A6
    JSR -234(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FindName(list : pList; const name : pCHAR) : pNode;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L name,A1
    MOVEA.L _ExecBase,A6
    JSR -276(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindPort(const name : pCHAR) : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A1
    MOVEA.L _ExecBase,A6
    JSR -390(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindResident(const name : pCHAR) : pResident;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A1
    MOVEA.L _ExecBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindSemaphore(const sigSem : pCHAR) : pSignalSemaphore;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A1
    MOVEA.L _ExecBase,A6
    JSR -594(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindTask(const name : pCHAR) : pTask;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A1
    MOVEA.L _ExecBase,A6
    JSR -294(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE Forbid;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeEntry(entry : pMemList);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L entry,A0
    MOVEA.L _ExecBase,A6
    JSR -228(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreePooled(poolHeader : POINTER; memory : POINTER; memSize : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L poolHeader,A0
    MOVEA.L memory,A1
    MOVE.L  memSize,D0
    MOVEA.L _ExecBase,A6
    JSR -714(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeSignal(signalNum : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  signalNum,D0
    MOVEA.L _ExecBase,A6
    JSR -336(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeTrap(trapNum : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  trapNum,D0
    MOVEA.L _ExecBase,A6
    JSR -348(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeVec(memoryBlock : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L memoryBlock,A1
    MOVEA.L _ExecBase,A6
    JSR -690(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetCC : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -528(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetMsg(port : pMsgPort) : pMessage;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A0
    MOVEA.L _ExecBase,A6
    JSR -372(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE InitCode(startClass : ULONG; version : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  startClass,D0
    MOVE.L  version,D1
    MOVEA.L _ExecBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION InitResident(const resident_ : pResident; segList : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L resident_,A1
    MOVE.L  segList,D1
    MOVEA.L _ExecBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE InitSemaphore(sigSem : pSignalSemaphore);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -558(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE InitStruct(const initTable : POINTER; memory : POINTER; size : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L initTable,A1
    MOVEA.L memory,A2
    MOVE.L  size,D0
    MOVEA.L _ExecBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MakeFunctions(const target : POINTER;const functionArray : POINTER;const funcDispBase :pointer);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L target,A0
    MOVEA.L functionArray,A1
    MOVEA.L funcDispBase,A2
    MOVEA.L _ExecBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MakeLibrary(const funcInit : POINTER;const structInit : POINTER; libInit : tPROCEDURE; dataSize : ULONG; segList : ULONG) : pLibrary;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L funcInit,A0
    MOVEA.L structInit,A1
    MOVEA.L libInit,A2
    MOVE.L  dataSize,D0
    MOVE.L  segList,D1
    MOVEA.L _ExecBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ObtainQuickVector(interruptCode : POINTER) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L interruptCode,A0
    MOVEA.L _ExecBase,A6
    JSR -786(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ObtainSemaphore(sigSem : pSignalSemaphore);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -564(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ObtainSemaphoreList(sigSem : pList);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -582(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ObtainSemaphoreShared(sigSem : pSignalSemaphore);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -678(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION OldOpenLibrary(const libName : pCHAR) : pLibrary;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L libName,A1
    MOVEA.L _ExecBase,A6
    JSR -408(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenDevice(const devName : pCHAR; unite : ULONG; ioRequest : pIORequest;
flags : ULONG) : shortint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L devName,A0
    MOVE.L  unite,D0
    MOVEA.L ioRequest,A1
    MOVE.L  flags,D1
    MOVEA.L _ExecBase,A6
    JSR -444(A6)
    MOVEA.L (A7)+,A6
    MOVE.B  D0,@RESULT
  END;
END;

FUNCTION OpenLibrary(const libName : pCHAR; version : ULONG) : pLibrary;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L libName,A1
    MOVE.L  version,D0
    MOVEA.L _ExecBase,A6
    JSR -552(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenResource(const resName : pCHAR) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L resName,A1
    MOVEA.L _ExecBase,A6
    JSR -498(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE Permit;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION Procure(sigSem : pSignalSemaphore; bidMsg : pSemaphoreMessage) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L bidMsg,A1
    MOVEA.L _ExecBase,A6
    JSR -540(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE PutMsg(port : pMsgPort; message : pMessage);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A0
    MOVEA.L message,A1
    MOVEA.L _ExecBase,A6
    JSR -366(A6)
    MOVEA.L (A7)+,A6
  END;
END;

function RawDoFmt(const formatString : pCHAR;const dataStream : POINTER; putChProc : tPROCEDURE; putChData : POINTER): pointer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L formatString,A0
    MOVEA.L dataStream,A1
    MOVEA.L putChProc,A2
    MOVEA.L putChData,A3
    MOVEA.L _ExecBase,A6
    JSR -522(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ReleaseSemaphore(sigSem : pSignalSemaphore);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -570(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ReleaseSemaphoreList(sigSem : pList);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L _ExecBase,A6
    JSR -588(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemDevice(device : pDevice);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L device,A1
    MOVEA.L _ExecBase,A6
    JSR -438(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RemHead(list : pList) : pNode;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L _ExecBase,A6
    JSR -258(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RemIntServer(intNumber : LONGINT; interrupt_ : pInterrupt);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  intNumber,D0
    MOVEA.L interrupt_,A1
    MOVEA.L _ExecBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemLibrary(lib : pLibrary);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lib,A1
    MOVEA.L _ExecBase,A6
    JSR -402(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemMemHandler(memhand : pInterrupt);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L memhand,A1
    MOVEA.L _ExecBase,A6
    JSR -780(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE Remove(node : pNode);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L node,A1
    MOVEA.L _ExecBase,A6
    JSR -252(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemPort(port : pMsgPort);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A1
    MOVEA.L _ExecBase,A6
    JSR -360(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemResource(resource : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L resource,A1
    MOVEA.L _ExecBase,A6
    JSR -492(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemSemaphore(sigSem : pSignalSemaphore);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A1
    MOVEA.L _ExecBase,A6
    JSR -606(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RemTail(list : pList) : pNode;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L list,A0
    MOVEA.L _ExecBase,A6
    JSR -264(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RemTask(task : pTask);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L task,A1
    MOVEA.L _ExecBase,A6
    JSR -288(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ReplyMsg(message : pMessage);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L message,A1
    MOVEA.L _ExecBase,A6
    JSR -378(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SendIO(ioRequest : pIORequest);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ioRequest,A1
    MOVEA.L _ExecBase,A6
    JSR -462(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetExcept(newSignals : ULONG; signalSet : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  newSignals,D0
    MOVE.L  signalSet,D1
    MOVEA.L _ExecBase,A6
    JSR -312(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetFunction(lib : pLibrary; funcOffset : LONGINT; newFunction : tPROCEDURE) :
POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lib,A1
    MOVEA.L funcOffset,A0
    MOVE.L  newFunction,D0
    MOVEA.L _ExecBase,A6
    JSR -420(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetIntVector(intNumber : LONGINT;const interrupt_ : pInterrupt) : pInterrupt;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  intNumber,D0
    MOVEA.L interrupt_,A1
    MOVEA.L _ExecBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetSignal(newSignals : ULONG; signalSet : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  newSignals,D0
    MOVE.L  signalSet,D1
    MOVEA.L _ExecBase,A6
    JSR -306(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetSR(newSR : ULONG; mask : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  newSR,D0
    MOVE.L  mask,D1
    MOVEA.L _ExecBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetTaskPri(task : pTask; priority : LONGINT) : shortint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L task,A1
    MOVE.L  priority,D0
    MOVEA.L _ExecBase,A6
    JSR -300(A6)
    MOVEA.L (A7)+,A6
    MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE Signal(task : pTask; signalSet : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L task,A1
    MOVE.L  signalSet,D0
    MOVEA.L _ExecBase,A6
    JSR -324(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE StackSwap(newStack : pStackSwapStruct);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L newStack,A0
    MOVEA.L _ExecBase,A6
    JSR -732(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SumKickData;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -612(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SumLibrary(lib : pLibrary);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lib,A1
    MOVEA.L _ExecBase,A6
    JSR -426(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SuperState : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _ExecBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION Supervisor(userFunction : tPROCEDURE) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L userFunction,A5
    MOVEA.L _ExecBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TypeOfMem(const address : POINTER) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L address,A1
    MOVEA.L _ExecBase,A6
    JSR -534(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE UserState(sysStack : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  sysStack,D0
    MOVEA.L _ExecBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE Vacate(sigSem : pSignalSemaphore; bidMsg : pSemaphoreMessage);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L sigSem,A0
    MOVEA.L bidMsg,A1
    MOVEA.L _ExecBase,A6
    JSR -546(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION Wait(signalSet : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  signalSet,D0
    MOVEA.L _ExecBase,A6
    JSR -318(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WaitIO(ioRequest : pIORequest) : shortint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ioRequest,A1
    MOVEA.L _ExecBase,A6
    JSR -474(A6)
    MOVEA.L (A7)+,A6
    MOVE.B  D0,@RESULT
  END;
END;

FUNCTION WaitPort(port : pMsgPort) : pMessage;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A0
    MOVEA.L _ExecBase,A6
    JSR -384(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE NewMinList(minlist : pMinList);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L minlist,A0
        MOVEA.L _ExecBase,A6
        JSR     -828(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AVL_AddNode(root : ppAVLNode; node : pAVLNode; func : POINTER) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L node,A1
        MOVEA.L func,A2
        MOVEA.L _ExecBase,A6
        JSR     -852(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_RemNodeByAddress(root : ppAVLNode; node : pAVLNode) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L node,A1
        MOVEA.L _ExecBase,A6
        JSR     -858(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_RemNodeByKey(root : ppAVLNode; key : POINTER; func : POINTER) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L key,A1
        MOVEA.L func,A2
        MOVEA.L _ExecBase,A6
        JSR     -864(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindNode(CONST root : pAVLNode; key : POINTER; func : POINTER) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L key,A1
        MOVEA.L func,A2
        MOVEA.L _ExecBase,A6
        JSR     -870(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindPrevNodeByAddress(CONST node : pAVLNode) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L node,A0
        MOVEA.L _ExecBase,A6
        JSR     -876(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindPrevNodeByKey(CONST root : pAVLNode; key : POINTER; func : POINTER) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L key,A1
        MOVEA.L func,A2
        MOVEA.L _ExecBase,A6
        JSR     -882(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindNextNodeByAddress(CONST node : pAVLNode) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L node,A0
        MOVEA.L _ExecBase,A6
        JSR     -888(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindNextNodeByKey(CONST root : pAVLNode; key : POINTER; func : POINTER) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L key,A1
        MOVEA.L func,A2
        MOVEA.L _ExecBase,A6
        JSR     -894(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindFirstNode(CONST root : pAVLNode) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L _ExecBase,A6
        JSR     -900(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AVL_FindLastNode(CONST root : pAVLNode) : pAVLNode;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L root,A0
        MOVEA.L _ExecBase,A6
        JSR     -906(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;


PROCEDURE AddMemList(size : ULONG; attributes : ULONG; pri : LONGINT; base : POINTER; const name : String);
BEGIN
    AddMemList(size,attributes,pri,base,pas2c(name));
END;
FUNCTION FindName(list : pList; const name : String) : pNode;
BEGIN
    FindName := FindName(list,pas2c(name));
END;
FUNCTION FindPort(const name : String) : pMsgPort;
BEGIN
    FindPort := FindPort(pas2c(name));
END;
FUNCTION FindResident(const name : String) : pResident;
BEGIN
    FindResident := FindResident(pas2c(name));
END;
FUNCTION FindSemaphore(const sigSem : String) : pSignalSemaphore;
BEGIN
    FindSemaphore := FindSemaphore(pas2c(sigSem));
END;
FUNCTION FindTask(const name : String) : pTask;
BEGIN
    FindTask := FindTask(pas2c(name));
END;
FUNCTION OldOpenLibrary(const libName : String) : pLibrary;
BEGIN
    OldOpenLibrary := OldOpenLibrary(pas2c(libName));
END;
FUNCTION OpenDevice(const devName : String; unite : ULONG; ioRequest : pIORequest;
flags : ULONG) : shortint;
BEGIN
    OpenDevice := OpenDevice(pas2c(devName),unite,ioRequest,flags);
END;
FUNCTION OpenLibrary(const libName : String; version : ULONG) : pLibrary;
BEGIN
    OpenLibrary := OpenLibrary(pas2c(libName),version);
END;
FUNCTION OpenResource(const resName : String) : POINTER;
BEGIN
    OpenResource := OpenResource(pas2c(resName));
END;
function RawDoFmt(const formatString : String;const dataStream : POINTER; putChProc : tPROCEDURE; putChData : POINTER): pointer;
BEGIN
    RawDoFmt := RawDoFmt(pas2c(formatString),dataStream,putChProc,putChData);
END;

END. (* UNIT EXEC *)





