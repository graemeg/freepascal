{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Free Pascal development team

    exec.library functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{

missing:
  MemHeaderExt (difficult struct)
  ETask Substructure in TTask

defines:
  AROS_NEED_LONG_ALIGN    = ????
  AROS_FLAVOUR_BINCOMPAT  = AROS_BINCOMPAT (Bincompat mode)
  AROS_MORPHOS_COMPATIBLE = ????
}


unit Exec;

{$PACKRECORDS C}

interface

type
  APTR         = Pointer;
  CONST_APTR   = Pointer;
  LONG         = LongInt;
  ULONG        = LongWord;
  QUAD         = Int64;
  UQUAD        = QWord;
  IPTR         = NativeUInt;
  SIPTR        = NativeInt;
  PIPTR        = ^IPTR;
  STRPTR       = PChar;
  CONST_STRPTR = PChar;
  BPTR         = Pointer;
  BSTR         = Pointer;
  BOOL         = SmallInt;
  UWORD        = Word;
  WORDBITS     = Word;
  LONGBITS     = LongWord;
  PLONGBITS    = ^LONGBITS;
  UBYTE        = Byte;
  PULONG       = PLongWord;
  PAPTR        = ^APTR;
  PLONG        = PLongInt;
  PaSmallInt   = PSmallInt;


// TagItem moved from Utilities to exec because needed for records

{ This data type may propagate through the system for more general use.
  In the meantime, it is used as a general mechanism of extensible data
  arrays for parameter specification and property inquiry (coming soon
  to a display controller near you).
  In practice, an array (or chain of arrays) of TagItems is used.}
type
 Tag = LongWord;
 PTag = ^Tag;

 PTagItem = ^TTagItem;
 TTagItem = record
   ti_Tag: Tag;
   ti_Data: IPTR;
 end;
 PPTagItem = ^PTagItem;

const
  TAG_USER = DWord(1 shl 31);  // differentiates user tags from system tags
// END of part from utility move

const
// There is a problem with Boolean vaules in taglists, just use this for now instead
  LTrue : LongInt = 1;
  LFalse: LongInt = 0;

type
// List Node Structure.  Each member in a list starts with a Node
  PNode = ^TNode;

{$ifdef AROS_BINCOMPAT}
  TNode =  record
    ln_Succ,                // Pointer to next (successor)
    ln_Pred : PNode;       // Pointer to previous (predecessor)
    ln_Type : Byte;
    ln_Pri  : ShortInt;    // Priority, for sorting
    ln_Name : STRPTR;      // ID string, null terminated
  end;  // Note: smallint aligned
{$else}
  TNode =  Record
    ln_Succ,                // Pointer to next (successor)
    ln_Pred : PNode;       // Pointer to previous (predecessor)
    ln_Name : STRPTR;      // ID string, null terminated
    ln_Type : Byte;
    ln_Pri  : ShortInt;    // Priority, for sorting
  End;  // Note: smallint aligned
{$endif}

// minimal node -- no type checking possible

  PMinNode = ^TMinNode;
  TMinNode =  Record
    mln_Succ,
    mln_Pred : PMinNode;
  End;

{
 Note: Newly initialized IORequests, and software interrupt structures
 used with Cause(), should have type NT_UNKNOWN.  The OS will assign a type
 when they are first used.
 }

// ----- Node Types for LN_TYPE -----

Const

  NT_UNKNOWN      =  0; // Unknown Node
  NT_TASK         =  1; // Exec task
  NT_INTERRUPT    =  2; // Interrupt
  NT_DEVICE       =  3; // Device
  NT_MSGPORT      =  4; // Message Port
  NT_MESSAGE      =  5; // Indicates message currently pending
  NT_FREEMSG      =  6;
  NT_REPLYMSG     =  7; // Message has been replied
  NT_RESOURCE     =  8;
  NT_LIBRARY      =  9;
  NT_MEMORY       = 10;
  NT_SOFTINT      = 11; // Internal flag used by SoftInits
  NT_FONT         = 12;
  NT_PROCESS      = 13; // AmigaDOS Process
  NT_SEMAPHORE    = 14;
  NT_SIGNALSEM    = 15; // signal semaphores
  NT_BOOTNODE     = 16;
  NT_KICKMEM      = 17;
  NT_GRAPHICS     = 18;
  NT_DEATHMESSAGE = 19;
  NT_HIDD         = 20; // AROS Specific

  NT_USER         = 254; // User node types work down from here
  NT_EXTENDED     = 255;

{ This file defines Exec system lists, which are used to link
  various things.  Exec provides several routines to handle list
  processing (defined at the bottom of this file), so you can
  use these routines to save yourself the trouble of writing a list
  package.}
type
// Normal, full featured list
  PList = ^TList;
  TList = record
    lh_Head: PNode;
    lh_Tail: PNode;
    lh_TailPred: PNode;
    lh_Type: Byte;
    l_pad: Byte;
  end;

// minimum list -- no type checking possible
  PMinList = ^TMinList;
  TMinList =  record
    mlh_Head: PMinNode;
    mlh_Tail: PMinNode;
    mlh_TailPred: PMinNode;
  end;

{********************************************************************
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

{ ********************************************************************
*
*  General Alerts
*
*  For example: timer.device cannot open math.library would be $05038015
*
*       Alert(AN_TimerDev|AG_OpenLib|AO_MathLib);
*
********************************************************************}

const
// General Types
  AT_DeadEnd    = $80000000; // Alert that crashes
  AT_Recovery   = $00000000; // Alert that returns

// General purpose alert codes }
  AG_NoMemory   = $00010000;
  AG_MakeLib    = $00020000;
  AG_OpenLib    = $00030000;
  AG_OpenDev    = $00040000;
  AG_OpenRes    = $00050000;
  AG_IOError    = $00060000;
  AG_NoSignal   = $00070000;
  AG_BadParm    = $00080000;
  AG_CloseLib   = $00090000; // usually too many closes
  AG_CloseDev   = $000A0000; // or a mismatched close
  AG_ProcCreate = $000B0000; // Process creation failed

{*********************************************************************
*
*  Hardware/CPU specific alerts:  They may show without the 8 at the
*  front of the number.  These are CPU/68000 specific.  See 68$0
*  programmer's manuals for more details.
*
*********************************************************************}
  ACPU_BusErr     = $80000002; // Hardware bus fault/access error
  ACPU_AddressErr = $80000003; // Illegal address access (ie: odd)
  ACPU_InstErr    = $80000004; // Illegal instruction
  ACPU_DivZero    = $80000005; // Divide by zero
  ACPU_CHK        = $80000006; // Check instruction error
  ACPU_TRAPV      = $80000007; // TrapV instruction error
  ACPU_PrivErr    = $80000008; // Privilege violation error
  ACPU_Trace      = $80000009; // Trace error
  ACPU_LineA      = $8000000A; // Line 1010 Emulator error
  ACPU_LineF      = $8000000B; // Line 1111 Emulator error
  ACPU_Format     = $8000000E; // Stack frame format error
  ACPU_Spurious   = $80000018; // Spurious interrupt error
  ACPU_AutoVec1   = $80000019; // AutoVector Level 1 interrupt error
  ACPU_AutoVec2   = $8000001A; // AutoVector Level 2 interrupt error
  ACPU_AutoVec3   = $8000001B; // AutoVector Level 3 interrupt error
  ACPU_AutoVec4   = $8000001C; // AutoVector Level 4 interrupt error
  ACPU_AutoVec5   = $8000001D; // AutoVector Level 5 interrupt error
  ACPU_AutoVec6   = $8000001E; // AutoVector Level 6 interrupt error
  ACPU_AutoVec7   = $8000001F; // AutoVector Level 7 interrupt error

// alert libraries
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
// alert devices
  AO_AudioDev     = $00008010;
  AO_ConsoleDev   = $00008011;
  AO_GamePortDev  = $00008012;
  AO_KeyboardDev  = $00008013;
  AO_TrackDiskDev = $00008014;
  AO_TimerDev     = $00008015;
// alert resources
  AO_CIARsrc    = $00008020;
  AO_DiskRsrc   = $00008021;
  AO_MiscRsrc   = $00008022;
// alert other
  AO_BootStrap  = $00008030;
  AO_Workbench  = $00008031;
  AO_DiskCopy   = $00008032;
  AO_GadTools   = $00008033;
  AO_Unknown    = $00008035;
// AROS Additions, start at  $40
  AO_ArosLib    = $00008040;
  AO_OOPLib     = $00008041;
  AO_HiddLib    = $00008042;

{********************************************************************
*
*   Specific Alerts:
*
********************************************************************}
// exec.library
  AN_ExecLib      = $01000000;
  AN_ExcptVect    = $01000001; // 68000 exception vector checksum (obs.)
  AN_BaseChkSum   = $01000002; //  Execbase checksum (obs.)
  AN_LibChkSum    = $01000003; // Library checksum failure
  AN_MemCorrupt   = $81000005; // Corrupt memory list detected in FreeMem
  AN_IntrMem      = $81000006; // No memory for interrupt servers
  AN_InitAPtr     = $01000007; // InitStruct() of an APTR source (obs.)
  AN_SemCorrupt   = $01000008; // A semaphore is in an illegal state at ReleaseSempahore()
  AN_FreeTwice    = $01000009; // Freeing memory already freed
  AN_BogusExcpt   = $8100000A; // illegal 68k exception taken (obs.)
  AN_IOUsedTwice  = $0100000B; // Attempt to reuse active IORequest
  AN_MemoryInsane = $0100000C; // Sanity check on memory list failed during AvailMem(MEMF_LARGEST)
  AN_IOAfterClose = $0100000D; // IO attempted on closed IORequest
  AN_StackProbe   = $0100000E; // Stack appears to extend out of range
  AN_BadFreeAddr  = $0100000F; // Memory header not located. [ Usually an invalid address passed to FreeMem() ]
  AN_BadSemaphore = $01000010; // An attempt was made to use the old message semaphores.

// dos.library
  AN_DOSLib       = $07000000;
  AN_StartMem     = $07010001; // no memory at startup
  AN_EndTask      = $07000002; // EndTask didn't
  AN_QPktFail     = $07000003; // Qpkt failure
  AN_AsyncPkt     = $07000004; // Unexpected packet received
  AN_FreeVec      = $07000005; // Freevec failed
  AN_DiskBlkSeq   = $07000006; // Disk block sequence error
  AN_BitMap       = $07000007; // Bitmap corrupt
  AN_KeyFree      = $07000008; // Key already free
  AN_BadChkSum    = $07000009; // Invalid checksum
  AN_DiskError    = $0700000A; // Disk Error
  AN_KeyRange     = $0700000B; // Key out of range
  AN_BadOverlay   = $0700000C; // Bad overlay
  AN_BadInitFunc  = $0700000D; // Invalid init packet for cli/shell
  AN_FileReclosed = $0700000E; // A filehandle was closed more than once

// graphics.library
  AN_GraphicsLib  = $02000000;
  AN_GfxNoMem     = $82010000; // graphics out of memory
  AN_GfxNoMemMspc = $82010001; // MonitorSpec alloc, no memory
  AN_LongFrame    = $82010006; // long frame, no memory
  AN_ShortFrame   = $82010007; // short frame, no memory
  AN_TextTmpRas   = $02010009; // text, no memory for TmpRas
  AN_BltBitMap    = $8201000A; //  BltBitMap, no memory
  AN_RegionMemory = $8201000B; //  regions, memory not available
  AN_MakeVPort    = $82010030; //  MakeVPort, no memory
  AN_GfxNewError  = $0200000C;
  AN_GfxFreeError = $0200000D;
  AN_GfxNoLCM     = $82011234; // emergency memory not available
  AN_ObsoleteFont = $02000401; // unsupported font description used

// intuition.library
  AN_Intuition    = $04000000;
  AN_GadgetType   = $84000001; // unknown gadget type
  AN_BadGadget    = $04000001; // Recovery form of AN_GadgetType
  AN_CreatePort   = $84010002; // create port, no memory
  AN_ItemAlloc    = $04010003; // item plane alloc, no memory
  AN_SubAlloc     = $04010004; // sub alloc, no memory
  AN_PlaneAlloc   = $84010005; // plane alloc, no memory
  AN_ItemBoxTop   = $84000006; // item box top < RelZero
  AN_OpenScreen   = $84010007; // open screen, no memory
  AN_OpenScrnRast = $84010008; // open screen, raster alloc, no memory
  AN_SysScrnType  = $84000009; // open sys screen, unknown type
  AN_AddSWGadget  = $8401000A; // add SW gadgets, no memory
  AN_OpenWindow   = $8401000B; // open window, no memory
  AN_BadState     = $8400000C; // Bad State Return entering Intuition
  AN_BadMessage   = $8400000D; // Bad Message received by IDCMP
  AN_WeirdEcho    = $8400000E; // Weird echo causing incomprehension
  AN_NoConsole    = $8400000F; // couldn't open the Console Device
  AN_NoISem       = $04000010; // Intuition skipped obtaining a sem
  AN_ISemOrder    = $04000011; // Intuition obtained a sem in bad order

// System utility library
  AN_UtilityLib   = $34000000;

// layers.library
  AN_LayersLib    = $03000000;
  AN_LayersNoMem  = $83010000; // layers out of memory

// math.library
  AN_MathLib      = $05000000;

// expansion.library
  AN_ExpansionLib     = $0A000000;
  AN_BadExpansionFree = $0A000001; // freeed free region

// ramlib.library
  AN_RAMLib       = $08000000;
  AN_BadSegList   = $08000001; // no overlays in library seglists

// diskfont.library
  AN_DiskfontLib  = $0B000000;

// icon.library
  AN_IconLib      = $09000000;

// Gadtools.library toolkit for Intuition
  AN_GadTools     = $33000000;

// audio.device
  AN_AudioDev     = $10000000;

// console.device
  AN_ConsoleDev   = $11000000;
  AN_NoWindow     = $11000001; // Console can't open initial window

// gameport.device
  AN_GamePortDev  = $12000000;

// keyboard.device
  AN_KeyboardDev  = $13000000;

// ------ trackdisk.device
  AN_TrackDiskDev = $14000000;
  AN_TDCalibSeek  = $14000001; // calibrate: seek error
  AN_TDDelay      = $14000002; // delay: error on timer wait

// timer.device
  AN_TimerDev     = $15000000;
  AN_TMBadReq     = $15000001; // bad request
  AN_TMBadSupply  = $15000002; // power supply -- no 50/60Hz ticks

// cia.resource
  AN_CIARsrc      = $20000000;

// disk.resource
  AN_DiskRsrc     = $21000000;
  AN_DRHasDisk    = $21000001; // get unit: already has disk
  AN_DRIntNoAct   = $21000002; // interrupt: no active unit

// misc.resource
  AN_MiscRsrc     = $22000000;

// bootstrap
  AN_BootStrap    = $30000000;
  AN_BootError    = $30000001; // boot code returned an error

// Workbench
  AN_Workbench          = $31000000;
  AN_NoFonts            = $B1000001;
  AN_WBBadStartupMsg1   = $31000001;
  AN_WBBadStartupMsg2   = $31000002;
  AN_WBBadIOMsg         = $31000003;
  AN_WBReLayoutToolMenu = $B1010009;

// DiskCopy
  AN_DiskCopy     = $32000000;

// For use by any application that needs it
  AN_Unknown      = $35000000;

// AROS Additions
  AN_Aros         = $40000000;
  AN_OOP          = $41000000;

// Hidd Subsystem
  AN_Hidd         = $42000000;
  AN_HiddNoRoot   = $C2000001;

const
  IOERR_OPENFAIL   = -1; // device/unit failed to open
  IOERR_ABORTED    = -2; // request terminated early [after AbortIO()]
  IOERR_NOCMD      = -3; // command not supported by device
  IOERR_BADLENGTH  = -4; // not a valid length (usually IO_LENGTH)
  IOERR_BADADDRESS = -5; // invalid address (misaligned or bad range)
  IOERR_UNITBUSY   = -6; // device opens ok, but requested unit is busy
  IOERR_SELFTEST   = -7; // hardware failed self-test

type
  PResident = ^TResident;
  TResident = record
    rt_MatchWord: Word;       // smallint to match on (ILLEGAL)
    rt_MatchTag: PResident;   // pointer to the above
    rt_EndSkip: APTR;         // address to continue scan
    rt_Flags: Byte;           // various tag flags
    rt_Version: Byte;         // release version number
    rt_Type: Byte;            // type of module (NT_mumble)
    rt_Pri: ShortInt;         // initialization priority
    rt_Name: CONST_STRPTR;    // pointer to node name
    rt_IdString: CONST_STRPTR;// pointer to ident string
    rt_Init: APTR;            // pointer to init code
    rt_Revision: Word;        // Extension taken over from MorphOS. Only valid if RTF_EXTENDED is set
    rt_Tags: PTagItem;         // PTagItem
  end;

const
  RTC_MATCHWORD   = $4AFC;

  RTF_COLDSTART   = $01;
  RTF_SINGLETASK  = $02;
  RTF_AFTERDOS    = $04;
  RTF_AUTOINIT    = $80;

  RTF_EXTENDED    = $40; // MorphOS extension: extended structure fields are valid

// Compatibility:

  RTW_NEVER       = $00;
  RTW_COLDSTART   = $01;

  RTT_STARTUP = TAG_USER + $04AF1234;

type
//****** MemChunk ****************************************************
  PMemChunk = ^TMemChunk;
  TMemChunk = record
    mc_Next: PMemChunk; // pointer to next chunk
    mc_Bytes: IPTR;    // chunk byte size
  end;

//****** MemHeader ***************************************************
  PMemHeader = ^TMemHeader;
  TMemHeader = record
    mh_Node: TNode;
    mh_Attributes: Word; // characteristics of this region
    mh_First: PMemChunk; // first free region
    mh_Lower,            // lower memory bound
    mh_Upper: APTR;      // upper memory bound+1
    mh_Free: IPTR;       // total number of free bytes
  end;

//****** MemEntry ****************************************************
  PMemEntry = ^TMemEntry;
  TMemEntry = record
    me_Un: record
      case LongInt of
        0: (meu_Reqs: ULONG);
        1: (meu_Addr: APTR);
      end;
    me_Length: IPTR;
  end;

//****** MemList *****************************************************
// Note: sizeof(TMemList) includes the size of the first MemEntry
  PMemList = ^TMemList;
  TMemList = record
    ml_Node: TNode;
    ml_NumEntries: Word;              // number of entries in this struct
    ml_ME: array [0..0] of TMemEntry; // the first entry
  End;

// Memory Requirement Types
//   See the AllocMem() documentation for details
const
  MEMF_ANY           = $0; // Any type of memory will do
  MEMF_PUBLIC        = 1 shl  0;
  MEMF_CHIP          = 1 shl  1;
  MEMF_FAST          = 1 shl  2;
  MEMF_EXECUTABLE    = 1 shl  4; // AmigaOS 4 Compatible
  MEMF_LOCAL         = 1 shl  8;
  MEMF_24BITDMA      = 1 shl  9; // DMAable memory within 24 bits of address
  MEMF_KICK          = 1 shl 10; // Memory that can be used for KickTags
  MEMF_31BIT         = 1 shl 12; // Low address space (<2GB). Effective only on 64 bit machines.
  MEMF_CLEAR         = 1 shl 16; // Explicitly clear memory after allocation
  MEMF_LARGEST       = 1 shl 17;
  MEMF_REVERSE       = 1 shl 18;
  MEMF_TOTAL         = 1 shl 19; // AvailMem: return total size of memory
  MEMF_HWALIGNED     = 1 shl 20; // For AllocMem() - align address and size to physical page boundary
  MEMF_SEM_PROTECTED = 1 shl 20; // For CreatePool() - add semaphore protection to the pool
  MEMF_NO_EXPUNGE    = 1 shl 31; // AllocMem: Do not cause expunge on failure

  MEM_BLOCKSIZE = 8;
  MEM_BLOCKMASK = MEM_BLOCKSIZE-1;

{$ifdef CPU64}
  MEMF_PHYSICAL_MASK = MEMF_PUBLIC or MEMF_CHIP or MEMF_FAST or MEMF_LOCAL or MEMF_24BITDMA or MEMF_KICK or MEMF_31BIT;
{$else}
  MEMF_PHYSICAL_MASK = MEMF_PUBLIC or MEMF_CHIP or MEMF_FAST or MEMF_LOCAL or MEMF_24BITDMA or MEMF_KICK;
{$endif}

type
//***** MemHandlerData *********************************************
// Note:  This structure is *READ ONLY* and only EXEC can create it!
 PMemHandlerData = ^TMemHandlerData;
 TMemHandlerData = record
   memh_RequestSize: IPTR;  // Requested allocation size
   memh_RequestFlags,       // Requested allocation flags
   memh_Flags: ULONG;       // Flags (see below)
 end;

const
  MEMHF_RECYCLE   =  1; // 0 = First time, 1 = recycle
//***** Low Memory handler return values **************************
  MEM_ALL_DONE    = -1; // We did all we could do
  MEM_DID_NOTHING =  0; // Nothing we could do...
  MEM_TRY_AGAIN   =  1; // We did some, try the allocation again

type
  PInterrupt = ^TInterrupt;
  TInterrupt = record
    is_Node: TNode;
    is_Data: APTR;    // Server data segment
    is_Code: Pointer; // Server code entry
  end;

// PRIVATE
  PIntVector = ^TIntVector;
  TIntVector = record      // For EXEC use ONLY!
    iv_Data: APTR;
    iv_Code: Pointer;
    iv_Node: PNode;
  end;

// PRIVATE
  PSoftIntList = ^TSoftIntList;
  TSoftIntList = record    // For EXEC use ONLY!
    sh_List: TList;
    sh_Pad : Word;
  end;

const
  SIH_PRIMASK = $F0;
// this is a fake INT definition, used only for AddIntServer and the like
  INTB_NMI    = 15;
  INTF_NMI    = 1 shl 15;
{ Offset of kernel interrupt vectors.
  Usage:
    AddIntServer(INTB_KERNEL + irq, irq_handler);
    RemIntServer(INTB_KERNEL + irq, irq_handler); }
  INTB_KERNEL = 16;

{ This file defines ports and messages, which are used for inter-
  task communications using the routines defined toward the
  bottom of this file.}
type
//****** MsgPort *****************************************************
  PMsgPort = ^TMsgPort;
  TMsgPort = record
    mp_Node: TNode;
    mp_Flags: Byte;
    mp_SigBit: Byte;     { signal bit number    }
    mp_SigTask: Pointer;   { task to be signalled (TaskPtr) }
    mp_MsgList: TList;     { message linked list  }
  end;

//****** Message *****************************************************
  PMessage = ^TMessage;
  TMessage = record
    mn_Node: TNode;
    mn_ReplyPort: PMsgPort; // message reply port
    mn_Length: Word;        // message len in bytes (include the size of the Message structure in the length)
  end;

//****** MagicMessage ***********************************************
  PMagicMessage = ^TMagicMessage;
  TMagicMessage = record
    mn_Node: TNode;
    mn_ReplyPort: PMsgPort; // message reply port
    mn_Length: Word;        // total message length, in bytes (include the size of the Message structure in the length)
    mn_Magic: ULONG;        // can be used to figure out the message sender
    mn_Version: ULONG;      // version can be used to extend a message in later versions
  end;

{ definition for entry Magic in Messages
  Magic is introduced to prevent Multiple Ports, for example if you´r using
  ScreenNotifications and DecorNotifications you must have two Ports as long
  as you cannot figure out which Messsage ist posted. With Magic this is no
  problem.}
const
  MAGIC_DECORATOR    = $8000001;
  MAGIC_SCREENNOTIFY = $8000002;

{   Every Amiga Task has one of these Task structures associated with it.
    To find yours, use FindTask(Nil).  AmigaDOS processes tack a few more
    values on to the end of this structure, which is the difference between
    Tasks and Processes.}
type
  PTask = ^TTask;
  TTask = record
    tc_Node: TNode;
    tc_Flags: Byte;
    tc_State: Byte;
    tc_IDNestCnt: Shortint; // intr disabled nesting
    tc_TDNestCnt: Shortint; // task disabled nesting
    tc_SigAlloc: ULONG;     // sigs allocated
    tc_SigWait: ULONG;      // sigs we are waiting for
    tc_SigRecvd: ULONG;     // sigs we have received
    tc_SigExcept: ULONG;    // sigs we will take excepts for
    case boolean of
      True:(
        tc_TrapAlloc: Word;     // traps allocated
        tc_TrapAble: Word;);      // traps enabled
      False:(
        tc_ETask: Pointer;      // Valid if TF_ETask is set
        tc_ExceptData: APTR;    // points to except data
        tc_ExceptCode: APTR;    // points to except code
        tc_TrapData: APTR;      // points to trap data
        tc_TrapCode: APTR;      // points to trap code
        tc_SPReg: APTR;         // stack pointer
        tc_SPLower: APTR;       // stack lower bound
        tc_SPUpper: APTR;       // stack upper bound + 2
        tc_Switch: Pointer;     // task losing CPU
        tc_Launch: Pointer;     // task getting CPU
        tc_MemEntry: TList;     // allocated memory
        tc_UserData: APTR;      // per task data
      );
  end;

// Stack swap structure as passed to StackSwap()
  PStackSwapStruct = ^TStackSwapStruct;
  TStackSwapStruct = record
    stk_Lower: APTR;   // Lowest byte of stack
    stk_Upper: APTR;   // Upper end of stack (size + Lowest)
    stk_Pointer: APTR; // Stack pointer at switch point
  end;

  PStackSwapArgs = ^TStackSwapArgs;
  TStackSwapArgs = record
    Args: array[0..7] of IPTR;
  end;

const
//----- Flag Bits ------------------------------------------
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

//----- Task States (tc_State) ---------------------------------
  TS_INVALID          = 0;
  TS_ADDED            = 1;
  TS_RUN              = 2;
  TS_READY            = 3;
  TS_WAIT             = 4;
  TS_EXCEPT           = 5;
  TS_REMOVED          = 6;

//----- Predefined Signals -------------------------------------
  SIGB_ABORT          = 0;
  SIGB_CHILD          = 1;
  SIGB_BLIT           = 4; // Note: same as SIGB_SINGLE
  SIGB_SINGLE         = 4; // Note: same as SIGB_BLIT
  SIGB_INTUITION      = 5;
  SIGB_NET            = 7;
  SIGB_DOS            = 8;

  SIGF_ABORT          = 1;
  SIGF_CHILD          = 2;
  SIGF_BLIT           = 16;
  SIGF_SINGLE         = 16;
  SIGF_INTUITION      = 32;
  SIGF_NET            = 128;
  SIGF_DOS            = 256;

type
{$ifdef AROS_MORPHOS_COMPATIBLE}
  TETask = record
    Message: TMessage;
    Parent: PTask;      // Pointer to task
    UniqueID: ULONG;
    Children: TMinList; // List of children
    TrapAlloc: Word;
    TrapAble: Word;
    Result1: ULONG;     // First result
    Result2: APTR;      // Result data pointer (AllocVec)
    MsgPort: TMsgPort;
    MemPool: Pointer;
    Reserved: array[0..1] of Pointer;
    RegFrame: Pointer;
    // Internal fields follow
  end;
{$else}
// Extended Task structure
  TETask = record
    et_Message: TMessage;
    et_Parent: APTR;       // Pointer to parent task
    et_UniqueID: ULONG;
    et_Children: TMinList; // List of children
    et_TrapAlloc: Word;
    et_TrapAble: Word;
    et_Result1: ULONG;     // First result
    et_Result2: APTR;      // Result data pointer (AllocVec)
    et_TaskMsgPort: TMsgPort;
    et_Compatibility: array[0..3] of APTR;   // Reserve this space for compiled software to access iet_startup and iet_acpd
    et_MemPool: Pointer;              // Task's private memory pool
{$ifdef aros}
    et_Reserved: array[0..0] of IPTR; // MorphOS Private
    et_TaskStorage: Pointer;          // Task Storage Slots
{$else}
    et_Reserved: array[0..1] of IPTR; // MorphOS Private
{$endif}
    et_RegFrame: Pointer;
    // Internal fields follow
  end;
{$endif}

const
// Return codes from new child functions
  CHILD_NOTNEW   = 1; // function not called from a new style task
  CHILD_NOTFOUND = 2; // Child not found
  CHILD_EXITED   = 3; // Child has exited
  CHILD_ACTIVE   = 4; // Child is currently active and running

  TASKTAG_Dummy        = TAG_USER + $100000;
  TASKTAG_ERROR        = TASKTAG_Dummy + 0;
  TASKTAG_CODETYPE     = TASKTAG_Dummy + 1;
  TASKTAG_PC           = TASKTAG_Dummy + 2;
  TASKTAG_FINALPC      = TASKTAG_Dummy + 3;
  TASKTAG_STACKSIZE    = TASKTAG_Dummy + 4;
  TASKTAG_NAME         = TASKTAG_Dummy + 6;
  TASKTAG_USERDATA     = TASKTAG_Dummy + 7;
  TASKTAG_PRI          = TASKTAG_Dummy + 8;
  TASKTAG_POOLPUDDLE   = TASKTAG_Dummy + 9;
  TASKTAG_POOLTHRESH   = TASKTAG_Dummy + 10;
  TASKTAG_ARG1         = TASKTAG_Dummy + 16;
  TASKTAG_ARG2         = TASKTAG_Dummy + 17;
  TASKTAG_ARG3         = TASKTAG_Dummy + 18;
  TASKTAG_ARG4         = TASKTAG_Dummy + 19;
  TASKTAG_ARG5         = TASKTAG_Dummy + 20;
  TASKTAG_ARG6         = TASKTAG_Dummy + 21;
  TASKTAG_ARG7         = TASKTAG_Dummy + 22;
  TASKTAG_ARG8         = TASKTAG_Dummy + 23;
  TASKTAG_STARTUPMSG   = TASKTAG_Dummy + 24;
  TASKTAG_TASKMSGPORT  = TASKTAG_Dummy + 25;
  TASKTAG_FLAGS        = TASKTAG_Dummy + 26;
  TASKTAG_TCBEXTRASIZE = TASKTAG_Dummy + 28;

  TASKERROR_OK = 0;
  TASKERROR_NOMEMORY = 1;
// Actions for ShutdownA()
  SD_ACTION_POWEROFF   = 0;
  SD_ACTION_COLDREBOOT = 1;

// mp_Flags: Port arrival actions (PutMsg)
const
  PF_ACTION   = 7; // Mask
  PA_SIGNAL   = 0; // Signal task in mp_SigTask
  PA_SOFTINT  = 1; // Signal SoftInt in mp_SoftInt/mp_SigTask
  PA_IGNORE   = 2; // Ignore arrival
  PA_CALL     = 3; // Call function in mp_SigTask. This was never
                   //   documented on AmigaOS and was never defined
                   //   but would work for mp_Flags = 3
  PA_FASTCALL = 4; // AROS extension. Like PA_SOFTINT, calls an
                   //   Interrupt in mp_SoftInt, but passes the
                   //   message as the third argument without doesn't
                   //   add it to the message list and so doesn't
                   //   require any locking, task switching or
                   //   Disable()/Enable() pairs */
// Semaphore
type
// This is the structure used to request a signal semaphore
  PSemaphoreRequest = ^TSemaphoreRequest;
  TSemaphoreRequest = record
    sr_Link: TMinNode;
    sr_Waiter: PTask;
  end;

// The actual semaphore itself
  PSignalSemaphore = ^TSignalSemaphore;
  TSignalSemaphore = record
    ss_Link: TNode;
    ss_NestCount: SmallInt;
    ss_WaitQueue: TMinList;
    ss_MultipleLink: TSemaphoreRequest;
    ss_Owner: PTask;
    ss_QueueCount: SmallInt;
  end;

// Semaphore procure message for Procure/Vacate
  PSemaphoreMessage = ^TSemaphoreMessage;
  TSemaphoreMessage = record
    ssm_Message: TMessage;
    ssm_Semaphore: PSignalSemaphore;
  end;

{ not in aros?
  PSemaphore = ^TSemaphore;
  TSemaphore = record
    sm_MsgPort: TMsgPort;
    sm_Bids: SmallInt;
  end;
}
const
  SM_SHARED    = 1;
  SM_EXCLUSIVE = 0;

//------ Special Constants ---------------------------------------
  LIB_RESERVED =  4;   // Exec reserves the first 4 vectors
  LIB_VECTSIZE =  6;   // Each library entry takes 6 bytes
  LIB_BASE     = (-LIB_VECTSIZE);
  LIB_USERDEF  = (LIB_BASE-(LIB_RESERVED*LIB_VECTSIZE));
  LIB_NONSTD   = (LIB_USERDEF);
//------ Standard functions --------------------------------------
  LIB_OPEN     = LIB_BASE * 1;
  LIB_CLOSE    = LIB_BASE * 2;
  LIB_EXPUNGE  = LIB_BASE * 3;
  LIB_EXTFUNC  = LIB_BASE * 4;  // for future expansion
type
//------ Library Base Structure ----------------------------------
// Also used for Devices and some Resources
  PLibrary = ^TLibrary;
  TLibrary = record
    lib_Node: TNode;
    lib_Flags,
    lib_pad: Byte;
    lib_NegSize,          // number of bytes before library
    lib_PosSize,          // number of bytes after library
    lib_Version,          // major
    lib_Revision: Word;   // minor
{$ifdef AROS_NEED_LONG_ALIGN}
    lib_pad1: Word;
{$endif}
    lib_IdString: STRPTR; // ASCII identification
    lib_Sum: ULONG;       // the checksum itself
    lib_OpenCnt: Word;    // number of current opens
{$ifdef AROS_NEED_LONG_ALIGN}
    lib_pad2: Word;
{$endif}

  end; // Warning: size is not a longword multiple!

const
// lib_Flags bit definitions (all others are system reserved)
  LIBF_SUMMING = 1 shl 0; // we are currently checksumming
  LIBF_CHANGED = 1 shl 1; // we have just changed the lib
  LIBF_SUMUSED = 1 shl 2; // set if we should bother to sum
  LIBF_DELEXP  = 1 shl 3; // delayed expunge

// ID numbers for Exec/TaggedOpenLibrary
  TAGGEDOPEN_GRAPHICS   = 1;
  TAGGEDOPEN_LAYERS     = 2;
  TAGGEDOPEN_INTUITION  = 3;
  TAGGEDOPEN_DOS        = 4;
  TAGGEDOPEN_ICON       = 5;
  TAGGEDOPEN_EXPANSION  = 6;
  TAGGEDOPEN_UTILITY    = 7;
  TAGGEDOPEN_KEYMAP     = 8;
  TAGGEDOPEN_GADTOOLS   = 9;
  TAGGEDOPEN_WORKBENCH  = 10;

{ This file defines the constants and types required to use
  Amiga device IO routines, which are also defined here.}
type
//***** Device ****************************************************
  PDevice = ^TDevice;
  TDevice =  record
    dd_Library: TLibrary;
  end;

//***** Unit ******************************************************
  PUnit = ^TUnit;
  TUnit = record
    unit_MsgPort: TMsgPort; // queue for unprocessed messages instance of msgport is recommended
    unit_flags,
    unit_pad    : Byte;
    unit_OpenCnt: Word;     // number of active opens
  end;

const
  UNITF_ACTIVE  = 1 shl 0;
  UNITF_INTASK  = 1 shl 1;

type
  PIORequest = ^TIORequest;
  TIORequest = record
    io_Message: TMessage;
    io_Device: PDevice;  // device node pointer
    io_Unit: PUnit;      // unit (driver private)
    io_Command: Word;    // device command
    io_Flags: Byte;
    io_Error: ShortInt;  // error or warning num
  end;

  PIOStdReq = ^TIOStdReq;
  TIOStdReq = record
    io_Message: TMessage;
    io_Device: PDevice;   // device node pointe
    io_Unit: PUnit;       // unit (driver private)
    io_Command: Word;     // device command
    io_Flags: Byte;
    io_Error: ShortInt;   // error or warning num
    io_Actual: ULONG;     // actual number of bytes transferred
    io_Length: ULONG;     // requested number bytes transferred
    io_Data: APTR;        // points to data area
    io_Offset: ULONG;     // offset for block structured devices
  end;

// library vector offsets for device reserved vectors
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
  PArosSupportBase = ^TArosSupportBase;
  TArosSupportBase = record
    StdOut: Pointer;
    kPrintfPtr: Pointer;
    rkPrintfPtr: Pointer;
    vkPrintfPtr: Pointer;
    DebugConfig: Pointer;
  end;

  PExecBase = ^TExecBase;
  TExecBase = record
// Standard Library Structure
    LibNode: TLibrary; // Standard library node
{ ******* Static System Variables ******* }
    SoftVer: Word;          // kickstart release number (obs.)
    LowMemChkSum: SmallInt; // checksum of 68000 trap vectors
    ChkBase: IPTR;          // system base pointer complement
    ColdCapture,            // coldstart soft capture vector
    CoolCapture,            // coolstart soft capture vector
    WarmCapture,            // warmstart soft capture vector
    SysStkUpper,            // system stack base   (upper bound)
    SysStkLower: APTR;      // top of system stack (lower bound)
    MaxLocMem: IPTR;        // top of chip memory
    DebugEntry,             // global debugger entry point
    DebugData,              //  global debugger data segment
    AlertData,              //  alert data segment
    MaxExtMem: APTR;        //  top of extended mem, or null if none
    ChkSum: Word;           // for all of the above (minus 2)
//***** Interrupt Related *****************************************
    IntVects: array[0..15] of TIntVector;
//***** Dynamic System Variables **********************************
    ThisTask: PTask;        // pointer to current task (readable)
    IdleCount,              // idle counter
    DispCount: ULONG;       // dispatch coutner
    Quantum,                // time slice quantum
    Elapsed,                // current quantum ticks
    SysFlags: Word;         // misc internal system flags
    IDNestCnt,              // interrupt disable nesting count
    TDNestCnt: Shortint;    // task disable nesting count
    AttnFlags,              // special attention flags (readable)
    AttnResched : Word;    // rescheduling attention
    ResModules,             // resident module array pointer
    TaskTrapCode,
    TaskExceptCode,
    TaskExitCode: APTR;
    TaskSigAlloc: ULONG;
    TaskTrapAlloc: Word;
//***** System Lists (private!) *******************************
    MemList,
    ResourceList,
    DeviceList,
    IntrList,
    LibList,
    PortList,
    TaskReady,
    TaskWait: TList;
    SoftInts: array[0..4] of TSoftIntList;
//***** Other Globals ******************************************
    LastAlert: array[0..3] of LONG;
  {  these next two variables are provided to allow
  ** system developers to have a rough idea of the
  ** period of two externally controlled signals --
  ** the time between vertical blank interrupts and the
  ** external line rate (which is counted by CIA A's
  ** "time of day" clock).  In general these values
  ** will be 50 or 60, and may or may not track each
  ** other.  These values replace the obsolete AFB_PAL
  ** and AFB_50HZ flags.}
    VBlankFrequency,             // (readable)
    PowerSupplyFrequency: Byte;  // (readable)
    SemaphoreList: TList;
  {  these next two are to be able to kickstart into user ram.
  ** KickMemPtr holds a singly linked list of MemLists which
  ** will be removed from the memory list via AllocAbs.  If
  ** all the AllocAbs's succeeded, then the KickTagPtr will
  ** be added to the rom tag list.}
    KickMemPtr,             // ptr to queue of mem lists
    KickTagPtr,             // ptr to rom tag queue
    KickCheckSum: APTR;     // checksum for mem and tags
//***** Miscellaneous Stuff *************************************
    ex_Pad0: Word;
    ex_LaunchPoint: IPTR;
    ex_RamLibPrivate: APTR;
  {  The next ULONG contains the system "E" clock frequency,
  ** expressed in Hertz.  The E clock is used as a timebase for
  ** the Amiga's 8520 I/O chips. (E is connected to "02").
  ** Typical values are 715909 for NTSC, or 709379 for PAL.}
    ex_EClockFrequency,         // (readable)
    ex_CacheControl,            // Private to CacheControl calls
    ex_TaskID: ULONG;           // Next available task ID
    ex_Reserved1: array[0..4] of ULONG;
    ex_MMULock: Pointer;        // private
    ex_Reserved2: array[0..1] of ULONG;
    ex_DebugFlags: ULONG;
    { The following list and data element are used
      exec's low memory handler...}
    ex_MemHandlers   : tMinList; // The handler list
    ex_MemHandler    : Pointer;  // Private! handler pointer
//***** Additional AROS fields **********************************
    DebugArosBase    : PArosSupportBase;
  end;

{ ***** Bit defines for AttnFlags (see above) ***************************** }
{   Processors and Co-processors:  }
const
  AFB_68010     = 0;    {  also set for 68020  }
  AFB_68020     = 1;    {  also set for 68030  }
  AFB_68030     = 2;    {  also set for 68040  }
  AFB_68040     = 3;
  AFB_68881     = 4;    {  also set for 68882  }
  AFB_68882     = 5;
  AFB_FPU40     = 6;    {  Set if 68040 FPU }
  AFB_68060     = 7;
  AFB_ADDR32    = 14; // AROS extension, CPU has 32-bit addressing
  AFB_PRIVATE   = 15; // see below

  AFF_68010     = 1 shl 0;
  AFF_68020     = 1 shl 1;
  AFF_68030     = 1 shl 2;
  AFF_68040     = 1 shl 3;
  AFF_68881     = 1 shl 4;
  AFF_68882     = 1 shl 5;
  AFF_FPU40     = 1 shl 6;
  AFF_68060     = 1 shl 7;
  AFF_ADDR32    = 1 shl 14;
  AFF_PRIVATE   = 1 shl 15;
{*
* AFB_PRIVATE is actually FPU presence flag with architecture-specific meaning:
* m68k   - Set if any FPU type detected. AmigaOS-compatible, however considered private.
* ARM    - Set if VFP is present. Considered public.
* Others - not used.
*}
  AFB_FPU       = AFB_PRIVATE;
  AFF_FPU       = AFF_PRIVATE ;

//* SysFlags. Private and AROS-specific. */
  SFF_SoftInt     = 1 shl 5;  // There is a software interrupt pending
  SFF_QuantumOver = 1 shl 13; // Task's time slice is over

// AttnResched. AmigaOS(tm)-compatible, but private.
  ARF_AttnSwitch  = 1 shl 7;  // Delayed task switch pending

{ ***** Selected flag definitions for Cache manipulation calls ********* }

  CACRF_EnableI       = 1 shl 0;  // Enable instruction cache
  CACRF_FreezeI       = 1 shl 1;  // Freeze instruction cache
  CACRF_ClearI        = 1 shl 3;  // Clear instruction cache
  CACRF_IBE           = 1 shl 4;  // Instruction burst enable
  CACRF_EnableD       = 1 shl 8;  // 68030 Enable data cache
  CACRF_FreezeD       = 1 shl 9;  // 68030 Freeze data cache
  CACRF_ClearD        = 1 shl 11; // 68030 Clear data cache
  CACRF_DBE           = 1 shl 12; // 68030 Data burst enable
  CACRF_WriteAllocate = 1 shl 13; // 68030 Write-Allocate mode (must always be set!)
  CACRF_InvalidateD   = 1 shl 15;
  CACRF_EnableE       = 1 shl 30;
  CACRF_CopyBack      = 1 shl 31;

  DMA_Continue        = 1 shl 1;  // Continuation flag for CachePreDMA
  DMA_NoModify        = 1 shl 2;  // Set if DMA does not update memory
  DMA_ReadFromRAM     = 1 shl 3;  // Set if DMA goes *FROM* RAM to device

{*
 * Runtime debug output flags, MorphOS-compatible.
 * Most of them are reserved for now.
 *}
  EXECDEBUGF_INITRESIDENT     = $00000001;  // Single resident initialization
  EXECDEBUGF_INITCODE         = $00000002;  // Kickstart initialization
  EXECDEBUGF_FINDRESIDENT     = $00000004;  // Resident search
  EXECDEBUGF_CREATELIBRARY    = $00000010;  // Library creation
  EXECDEBUGF_SETfunction      = $00000020;  // Library function patching
  EXECDEBUGF_NEWSETfunction   = $00000040;
  EXECDEBUGF_CHIPRAM          = $00000080;
  EXECDEBUGF_ADDTASK          = $00000100;  // Task creation
  EXECDEBUGF_REMTASK          = $00000200;  // Task removal
  EXECDEBUGF_GETTASKATTR      = $00000400;
  EXECDEBUGF_SETTASKATTR      = $00000800;
  EXECDEBUGF_EXCEPTHANDLER    = $00001000;
  EXECDEBUGF_ADDDOSNODE       = $00002000;
  EXECDEBUGF_PCI              = $00004000;
  EXECDEBUGF_RAMLIB           = $00008000;
  EXECDEBUGF_NOLOGSERVER      = $00010000;
  EXECDEBUGF_NOLOGWINDOW      = $00020000;
  EXECDEBUGF_LOGFILE          = $00040000;
  EXECDEBUGF_LOGKPRINTF       = $00080000;
  EXECDEBUGF_PERMMEMTRACK     = $00100000;
  EXECDEBUGF_MEMTRACK         = $00200000;
  EXECDEBUGF_CYBERGUARDDEADLY = $00400000;
  EXECDEBUGF_LOGEXTENDED      = $00800000;
  EXECDEBUGF_LOADSEG          = $01000000;
  EXECDEBUGF_UNLOADSEG        = $02000000;
  EXECDEBUGF_PPCSTART         = $04000000;
  EXECDEBUGF_CGXDEBUG         = $08000000;
  EXECDEBUGF_INVZEROPAGE      = $10000000;
  EXECDEBUGF_INIT             = $40000000; // Generic system startup
  EXECDEBUGF_LOG              = $80000000;

// The base node in an AVL tree.  Embed this within your object a-la exec ListNode
type
  PPAVLNode = ^PAVLNode;
  PAVLNode = ^TAVLNode;
  TAVLNode = record
    avl_link: array[0..1] of PAVLNode;
    avl_parent: PAVLNode;
    avl_balance: LONG;
  end;
  AVLKey = Pointer;

  PAVLNODECOMP = ^AVLNODECOMP;
  AVLNODECOMP = APTR;
  PAVLKEYCOMP = ^AVLKEYCOMP;
  AVLKEYCOMP = APTR;


const
// Magic constants for RawDoFmt() anv VNewRawDoFmt() to be given as PutChProc
  RAWFMTFUNC_STRING = 0; // Output to string given in PutChData
  RAWFMTFUNC_SERIAL = 1; // Output to debug log (usually serial port)
  RAWFMTFUNC_COUNT  = 2; // Just count characters, PutChData is a pointer to the counter (ULONG *)

var
  ExecBase: PExecBase absolute AOS_ExecBase;

// function headers
function Supervisor(UserFunction: TProcedure): ULONG; syscall AOS_ExecBase 5;
procedure Reschedule(Task: PTask); syscall AOS_ExecBase 8;
procedure ExecException; syscall AOS_ExecBase 11;
procedure InitCode(StartClass: ULONG; Version: ULONG); syscall AOS_ExecBase 12;
procedure InitStruct(const InitTable: APTR; Memory: APTR; Size: ULONG); syscall AOS_ExecBase 13;
function MakeLibrary(const FuncInit: APTR; const StructInit: APTR; LibInit: TProcedure; DataSize: ULONG; SegList: ULONG): PLibrary; syscall AOS_ExecBase 14;
procedure MakeFunctions(const Target: APTR; const FunctionArray: CONST_APTR; const FuncDispBase: CONST_APTR); syscall AOS_ExecBase 15;
function FindResident(const Name: PChar): PResident; syscall AOS_ExecBase 16;
function InitResident(const Resident_: PResident; SegList: ULONG): PResident; syscall AOS_ExecBase 17;
procedure Alert(AlertNum: ULONG); syscall AOS_ExecBase 18;
procedure Debug(Flags: ULONG); syscall AOS_ExecBase 19;
procedure Disable; syscall AOS_ExecBase 20;
procedure Enable; syscall AOS_ExecBase 21;
procedure Forbid; syscall AOS_ExecBase 22;
procedure Permit; syscall AOS_ExecBase 23;
function SetSR(NewSR: ULONG; Mask: ULONG): ULONG; syscall AOS_ExecBase 24;
function SuperState: APTR; syscall AOS_ExecBase 25;
procedure UserState(SysStack: APTR); syscall AOS_ExecBase 26;
function SetIntVector(IntNumber: LongInt; const Interrupt_: PInterrupt): PInterrupt; syscall AOS_ExecBase 27;
procedure AddIntServer(IntNumber: ULONG; Interrupt_: PInterrupt); syscall AOS_ExecBase 28;
procedure RemIntServer(IntNumber: ULONG; Interrupt_: PInterrupt); syscall AOS_ExecBase 29;
procedure Cause(Interrupt_: PInterrupt); syscall AOS_ExecBase 30;
function Allocate(FreeList: PMemHeader; ByteSize: ULONG): PMemHeader; syscall AOS_ExecBase 31;
procedure Deallocate(FreeList: PMemHeader; MemoryBlock: APTR; ByteSize: ULONG); syscall AOS_ExecBase 32;
function ExecAllocMem(ByteSize: ULONG; Requirements: ULONG): APTR; syscall AOS_ExecBase 33;
function AllocAbs(ByteSize: ULONG; Location: APTR): APTR; syscall AOS_ExecBase 34;
procedure ExecFreeMem(MemoryBlock: APTR; ByteSize: ULONG); syscall AOS_ExecBase 35;
function AvailMem(Requirements: ULONG): ULONG; syscall AOS_ExecBase 36;
function AllocEntry(Entry: PMemList): PMemList; syscall AOS_ExecBase 37;
procedure FreeEntry(Entry: PMemList); syscall AOS_ExecBase 38;
procedure ExecInsert(List: PList; Node: PNode; Pred: PNode); syscall AOS_ExecBase 39;
procedure AddHead(List: PList; Node: PNode); syscall AOS_ExecBase 40;
procedure AddTail(List: PList; Node: PNode); syscall AOS_ExecBase 41;
procedure Remove(Node: PNode); syscall AOS_ExecBase 42;
function RemHead(List: PList): PNode; syscall AOS_ExecBase 43;
function RemTail(List: PList): PNode; syscall AOS_ExecBase 44;
procedure Enqueue(List: PList; Node: PNode); syscall AOS_ExecBase 45;
function FindName(List: PList; const Name: PChar): PNode; syscall AOS_ExecBase 46;
function AddTask(Task: PTask; const InitialPC: APTR; const FinalPC: APTR): PTask; syscall AOS_ExecBase 47;  deprecated;
procedure RemTask(Task: PTask); syscall AOS_ExecBase 48;
function FindTask(const Name: STRPTR): PTask; syscall AOS_ExecBase 49;
function SetTaskPri(Task: PTask; Priority: LongInt): ShortInt; syscall AOS_ExecBase 50;
function SetSignal(NewSignals: ULONG; SignalSet: ULONG): ULONG; syscall AOS_ExecBase 51;
function SetExcept(NewSignals: ULONG; SignalSet: ULONG): ULONG; syscall AOS_ExecBase 52;
function Wait(SignalSet: ULONG): ULONG; syscall AOS_ExecBase 53;
procedure Signal(Task: PTask; SignalSet: ULONG); syscall AOS_ExecBase 54;
function AllocSignal(SignalNum: LongInt): ShortInt; syscall AOS_ExecBase 55;
procedure FreeSignal(SignalNum: LongInt); syscall AOS_ExecBase 56;
function AllocTrap(TrapNum: LongInt): LongInt; syscall AOS_ExecBase 57;
procedure FreeTrap(TrapNum: LongInt); syscall AOS_ExecBase 58;
procedure AddPort(Port: PMsgPort); syscall AOS_ExecBase 59;
procedure RemPort(Port: PMsgPort); syscall AOS_ExecBase 60;
procedure PutMsg(Port: PMsgPort; Message: PMessage); syscall AOS_ExecBase 61;
function GetMsg(Port: PMsgPort): PMessage; syscall AOS_ExecBase 62;
procedure ReplyMsg(Message: PMessage); syscall AOS_ExecBase 63;
function WaitPort(Port: PMsgPort): PMessage; syscall AOS_ExecBase 64;
function FindPort(const Name: STRPTR): PMsgPort; syscall AOS_ExecBase 65;
procedure AddLibrary(Library_: PLibrary); syscall AOS_ExecBase 66;
procedure RemLibrary(Library_: PLibrary); syscall AOS_ExecBase 67;
function OldOpenLibrary(const LibName: STRPTR): PLibrary; syscall AOS_ExecBase 68; deprecated;
procedure CloseLibrary(Library_: PLibrary); syscall AOS_ExecBase 69;
function SetFunction(Library_: PLibrary; FuncOffset: LongInt; NewFunction: TProcedure): APTR; syscall AOS_ExecBase 70;
procedure SumLibrary(Library_: PLibrary); syscall AOS_ExecBase 71;
procedure AddDevice(Device: PDevice); syscall AOS_ExecBase 72;
procedure RemDevice(Device: PDevice); syscall AOS_ExecBase 73;
function OpenDevice(const DevName: STRPTR; UnitNumber: ULONG; IORequest: PIORequest; Flags: ULONG): LongInt; syscall AOS_ExecBase 74;
procedure CloseDevice(IORequest: PIORequest); syscall AOS_ExecBase 75;
function DoIO(IORequest: PIORequest): LongInt; syscall AOS_ExecBase 76;
procedure SendIO(IORequest: PIORequest); syscall AOS_ExecBase 77;
function CheckIO(IORequest: PIORequest): PIORequest; syscall AOS_ExecBase 78;
function WaitIO(IORequest: PIORequest): LongInt; syscall AOS_ExecBase 79;
function AbortIO(IORequest: PIORequest): LongInt; syscall AOS_ExecBase 80;
procedure AddResource(Resource: APTR); syscall AOS_ExecBase 81;
procedure RemResource(Resource: APTR); syscall AOS_ExecBase 82;
function OpenResource(const ResName: STRPTR): APTR; syscall AOS_ExecBase 83;
procedure RawIOInit; syscall AOS_ExecBase 84;
function RawMayGetChar: LongInt; syscall AOS_ExecBase 85;
procedure RawMayPutChar(Cha: Byte); syscall AOS_ExecBase 86;
function RawDoFmt(const FormatString: STRPTR; const DataStream: APTR; PutChProc: TProcedure; PutChData: APTR): APTR; syscall AOS_ExecBase 87;
function GetCC: Word; syscall AOS_ExecBase 88;
function TypeOfMem(const Address: APTR): ULONG; syscall AOS_ExecBase 89;
function Procure(SigSem: PSignalSemaphore; BidMsg: PSemaphoreMessage): ULONG; syscall AOS_ExecBase 90;
procedure Vacate(SigSem: PSignalSemaphore; BidMsg: PSemaphoreMessage); syscall AOS_ExecBase 91;
function OpenLibrary(const LibName: STRPTR; Version: ULONG): PLibrary; syscall AOS_ExecBase 92;
procedure InitSemaphore(SigSem: PSignalSemaphore); syscall AOS_ExecBase 93;
procedure ObtainSemaphore(SigSem: PSignalSemaphore); syscall AOS_ExecBase 94;
procedure ReleaseSemaphore(SigSem: PSignalSemaphore); syscall AOS_ExecBase 95;
function AttemptSemaphore(SigSem: PSignalSemaphore): ULONG; syscall AOS_ExecBase 96;
procedure ObtainSemaphoreList(SigSem: PList); syscall AOS_ExecBase 97;
procedure ReleaseSemaphoreList(SigSem: PList); syscall AOS_ExecBase 98;
function FindSemaphore(const SigSem: STRPTR): PSignalSemaphore; syscall AOS_ExecBase 99;
procedure AddSemaphore(SigSem: PSignalSemaphore); syscall AOS_ExecBase 100;
procedure RemSemaphore(SigSem: PSignalSemaphore); syscall AOS_ExecBase 101;
procedure SumKickData; syscall AOS_ExecBase 102;
procedure AddMemList(Size: ULONG; Attributes: ULONG; Pri: LongInt; Base: APTR; const Name: STRPTR); syscall AOS_ExecBase 103;
procedure CopyMem(const Source: APTR; Dest: APTR; Size: ULONG); syscall AOS_ExecBase 104;
procedure CopyMemQuick(const Source: APTR; Dest: APTR; Size: ULONG); syscall AOS_ExecBase 105;
procedure CacheClearU;syscall AOS_ExecBase 106;
procedure CacheClearE(Address: APTR; Length: ULONG; Caches: ULONG); syscall AOS_ExecBase 107;
function CacheControl(CacheBits: ULONG; CacheMask: ULONG): ULONG; syscall AOS_ExecBase 108;
function CreateIORequest(const IOReplyPort: PMsgPort; Size: ULONG): APTR; syscall AOS_ExecBase 109;
procedure DeleteIORequest(IORequest: APTR); syscall AOS_ExecBase 110;
function CreateMsgPort: PMsgPort; syscall AOS_ExecBase 111;
procedure DeleteMsgPort(Port: PMsgPort); syscall AOS_ExecBase 112;
procedure ObtainSemaphoreShared(SigSem: PSignalSemaphore); syscall AOS_ExecBase 113;
function AllocVec(ByteSize: ULONG; Requirements: ULONG): APTR; syscall AOS_ExecBase 114;
procedure FreeVec(MemoryBlock: APTR); syscall AOS_ExecBase 115;
function CreatePool(Requirements: ULONG; PuddleSize: ULONG; ThreshSize: ULONG): APTR; syscall AOS_ExecBase 116;
procedure DeletePool(PoolHeader: APTR); syscall AOS_ExecBase 117;
function AllocPooled(PoolHeader: APTR; MemSize: ULONG): APTR; syscall AOS_ExecBase 118;
procedure FreePooled(PoolHeader: APTR; Memory: APTR; MemSize: ULONG); syscall AOS_ExecBase 119;
function AttemptSemaphoreShared(SigSem: PSignalSemaphore): ULONG; syscall AOS_ExecBase 120;
procedure ColdReboot; syscall AOS_ExecBase 121;
procedure StackSwap(NewStack: PStackSwapStruct); syscall AOS_ExecBase 122; deprecated;
procedure ChildFree(Tid: ULONG); syscall AOS_ExecBase 123;
function ChildOrphan(Tid: ULONG): ULONG; syscall AOS_ExecBase 124;
function ChildStatus(Tid: ULONG): ULONG; syscall AOS_ExecBase 125;
function ChildWait(Tid: ULONG): IPTR; syscall AOS_ExecBase 126;
function CachePreDMA(const Address: APTR; var Length: ULONG; Flags: ULONG): APTR; syscall AOS_ExecBase 127;
procedure CachePostDMA(const Address: APTR; var Length: ULONG; Flags: ULONG); syscall AOS_ExecBase 128;
procedure AddMemHandler(MemHandler: PInterrupt); syscall AOS_ExecBase 129;
procedure RemMemHandler(MemHandler: PInterrupt); syscall AOS_ExecBase 130;
function ObtainQuickVector(InterruptCode: APTR): ULONG; syscall AOS_ExecBase 131;
function NewStackSwap(NewStack: PStackSwapStruct; Function_: APTR; Args: PStackSwapArgs): IPTR; syscall AOS_ExecBase 134;
function TaggedOpenLibrary(Tag: LongInt): APTR; syscall AOS_ExecBase 135;
function ReadGayle: ULONG; syscall AOS_ExecBase 136;
function VNewRawDoFmt(const FormatString: STRPTR; PutChProc: TProcedure; PutChData: APTR; VaListStream: PChar): STRPTR; syscall AOS_ExecBase 137;
function NewAddTaskA(TagList: PTagItem): APTR; syscall AOS_ExecBase 153;
function AddResetCallback(ResetCallback: PInterrupt): LongBool; syscall AOS_ExecBase 167;
procedure RemResetCallback(ResetCallback: PInterrupt); syscall AOS_ExecBase 168;
function ShutdownA(Action: ULONG): ULONG; syscall AOS_ExecBase 173;


{$ifdef AROS_ABIv0}
function PrepareContext(Task: PTask; EntryPoint: APTR; FallBack: APTR; TagList: PTagItem): LongBool; syscall AOS_ExecBase 6;
procedure Switch; syscall AOS_ExecBase 9;
procedure Dispatch; syscall AOS_ExecBase 10;
function AVL_AddNode(Root: PPAVLNode; Node: PAVLNode; Func: PAVLNODECOMP): PAVLNode; syscall AOS_ExecBase 139;
function AVL_RemNodeByAddress(Root: PPAVLNode; Node: PAVLNode): PAVLNode; syscall AOS_ExecBase 140;
function AVL_RemNodeByKey(Root: PPAVLNode; Key: AVLKey; Func: PAVLNODECOMP): PAVLNode; syscall AOS_ExecBase 141;
function AVL_FindNode(Root: PPAVLNode; Key: AVLKey; Func: PAVLNODECOMP): PAVLNode; syscall AOS_ExecBase 142;
function AVL_FindPrevNodeByAddress(Node: PAVLNode): PAVLNode; syscall AOS_ExecBase 143;
function AVL_FindPrevNodeByKey(Node: PAVLNode; Key: AVLKey): PAVLNode; syscall AOS_ExecBase 144;
function AVL_FindNextNodeByAddress(Node: PAVLNode): PAVLNode; syscall AOS_ExecBase 145;
function AVL_FindNextNodeByKey(Node: PAVLNode; Key: AVLKey): PAVLNode; syscall AOS_ExecBase 146;
function AVL_FindFirstNode(Root: PAVLNode): PAVLNode; syscall AOS_ExecBase 147;
function AVL_FindLastNode(Root: PAVLNode): PAVLNode; syscall AOS_ExecBase 148;
function NewAddTask(Task: PTask; InitialPC: APTR; FinalPC: APTR; TagList: PTagItem): APTR; syscall AOS_ExecBase 152;
function AllocVecPooled(Pool: APTR; Size: ULONG): APTR; syscall AOS_ExecBase 149;
procedure FreeVecPooled(Pool: APTR; Memory: APTR); syscall AOS_ExecBase 150;
function NewAllocEntry(Entry: PMemList; var Return_Entry: PMemList; var Return_Flags: ULONG): LongBool; syscall AOS_ExecBase 151;
{$endif}

{$ifdef AROS_ABIv1}
procedure NewMinList(Ml: PMinList); syscall AOS_ExecBase 138;
function AVL_AddNode(Root: PPAVLNode; Node: PAVLNode; Func: PAVLNODECOMP): PAVLNode; syscall AOS_ExecBase 142;
function AVL_RemNodeByAddress(Root: PPAVLNode; Node: PAVLNode): PAVLNode; syscall AOS_ExecBase 143;
function AVL_RemNodeByKey(Root: PPAVLNode; Key: AVLKey; Func: PAVLNODECOMP): PAVLNode; syscall AOS_ExecBase 144;
function AVL_FindNode(Root: PPAVLNode; Key: AVLKey; Func: PAVLNODECOMP): PAVLNode; syscall AOS_ExecBase 145;
function AVL_FindPrevNodeByAddress(Node: PAVLNode): PAVLNode; syscall AOS_ExecBase 146;
function AVL_FindPrevNodeByKey(Node: PAVLNode; Key: AVLKey): PAVLNode; syscall AOS_ExecBase 147;
function AVL_FindNextNodeByAddress(Node: PAVLNode): PAVLNode; syscall AOS_ExecBase 148;
function AVL_FindNextNodeByKey(Node: PAVLNode; Key: AVLKey): PAVLNode; syscall AOS_ExecBase 149;
function AVL_FindFirstNode(Root: PAVLNode): PAVLNode; syscall AOS_ExecBase 150;
function AVL_FindLastNode(Root: PAVLNode): PAVLNode; syscall AOS_ExecBase 151;
function FindTaskByPID(ProcessID: LongWord): PTask; syscall AOS_ExecBase 166;
function AllocVecPooled(Pool: APTR; Size: ULONG): APTR; syscall AOS_ExecBase 169;
procedure FreeVecPooled(Pool: APTR; Memory: APTR); syscall AOS_ExecBase 170;
function NewAllocEntry(Entry: PMemList; var Return_Entry: PMemList; var Return_Flags: ULONG): LongBool; syscall AOS_ExecBase 174;
function NewAddTask(Task: PTask; InitialPC: APTR; FinalPC: APTR; TagList: PTagItem): APTR; syscall AOS_ExecBase 176;
function AllocTaskStorage: LongInt; syscall AOS_ExecBase 180;
procedure FreeTaskStorage(Slot: LongInt); syscall AOS_ExecBase 181;
function SaveTaskStorage: APTR; syscall AOS_ExecBase 182;
procedure RestoreTaskStorage(Id: APTR); syscall AOS_ExecBase 183;
function SetTaskStorageSlot(Id: LongInt; Value: IPTR): LongBool; syscall AOS_ExecBase 184;
function GetTaskStorageSlot(Id: LongInt): IPTR; syscall AOS_ExecBase 185;
function GetParentTaskStorageSlot(Id: LongInt): IPTR; syscall AOS_ExecBase 186;
{$endif}

function BitMask(no :ShortInt): LongInt;
// C Macros
procedure SetNodeName(Node: PNode; Name: PChar);
function GetNodeName(Node: PNode): PChar;

procedure NewList(List: PList);
function GetHead(List: PList): PNode; inline;
function GetTail(List: PList): PNode; inline;
function GetSucc(Node: PNode): PNode; inline;
function GetPred(Node: PNode): PNode; inline;
function ListLength(List: PList): Integer;

function IsListEmpty(List: PList): Boolean;

function IsMinListEmpty(List: PMinList): Boolean;
function IsMsgPortEmpty( mp: pMsgPort): Boolean;

type
  TNodeProcedure = procedure(Node:PNode); // Procedure for ForEachNode;

procedure ForEachNode(List:PList; NodeProc: TNodeProcedure);
procedure ForEachNodeSafe(List:PList; NodeProc: TNodeProcedure);

function CreateExtIO(const Mp: PMsgPort; Size: Integer): PIORequest;
procedure DeleteExtIO(ioReq: PIORequest);

implementation

// C Macros
procedure SetNodeName(Node: PNode; Name: PChar); inline;
begin
  if Assigned(Node) then
    Node^.ln_Name := Name;
end;

function GetNodeName(Node: PNode): PChar; inline;
begin
  if Assigned(Node) then
    GetNodeName := Node^.ln_Name;
end;

procedure NewList(List: PList); inline;
begin
  if Assigned(List) then
  begin
    List^.lh_TailPred := PNode(List);
    List^.lh_Tail := nil;
    List^.lh_Head := @List^.lh_Tail;
  end;
end;

function GetHead(List: PList): PNode; inline;
begin
  GetHead := nil;
  if Assigned(List) then
  begin
    if Assigned(List^.lh_Head^.ln_Succ) then
      GetHead := List^.lh_Head;
  end;
end;

function GetTail(List: PList): PNode; inline;
begin
  GetTail := nil;
  if Assigned(List) then
  begin
    if Assigned(List^.lh_TailPred^.ln_Pred) then
      GetTail := List^.lh_TailPred;
  end;
end;

function GetSucc(Node: PNode): PNode; inline;
begin
  GetSucc := nil;
  if Assigned(Node) then
    if Assigned(Node^.ln_Succ) then
      if Assigned(Node^.ln_Succ^.ln_Succ) then
        GetSucc := Node^.ln_Succ;
end;

function GetPred(Node: PNode): PNode; inline;
begin
  GetPred := nil;
  if Assigned(Node) then
    if Assigned(Node^.ln_Pred) then
      if Assigned(Node^.ln_Pred^.ln_Pred) then
        GetPred := Node^.ln_Pred;
end;

procedure ForEachNode(List:PList; NodeProc: TNodeProcedure);
var
  i: Integer;
  Node: PNode;
begin
  if not Assigned(List) or not Assigned(NodeProc) then
    Exit;
  Node := GetHead(List);
  for i := 0 to ListLength(List) do
  begin
    if not Assigned(Node) then
      Exit;
    NodeProc(Node);
    Node := GetSucc(Node);
  end;
end;

procedure ForEachNodeSafe(List:PList; NodeProc: TNodeProcedure);
var
  i: Integer;
  Node: PNode;
  NextNode: PNode;
begin
  if not Assigned(List) or not Assigned(NodeProc) then
    Exit;
  Node := GetHead(List);
  if not Assigned(Node) then
    Exit;
  while Assigned(Node) do
  begin
    if not Assigned(Node) then
      Exit;
    NextNode := GetSucc(Node);
    NodeProc(Node);
    Node := NextNode;
  end;
end;

function ListLength(List: PList): Integer;
var
  Current: PNode;
  Next: PNode;
begin
  Current := List^.lh_Head;
  Next := Current^.ln_Succ;
  ListLength := 0;
  if Assigned(Current) and Assigned(Next) then
    while Next = Current^.ln_Succ do
    begin
      Current := Next;
      Next := Current^.ln_Succ;
      Inc(ListLength);
      if (not Assigned(Current)) or (not Assigned(Next)) then
        Exit;
    end;
end;

function IsListEmpty(List: PList): Boolean; inline;
begin
  IsListEmpty := True;
  if Assigned(List) then
    IsListEmpty := List^.lh_TailPred = PNode(List);
end;

function IsMinListEmpty(List: PMinList): Boolean;
begin
  IsMinListEmpty := True;
  if Assigned(List) then
    IsMinListEmpty := List^.mlh_TailPred = PMinNode(List);
end;

function IsMsgPortEmpty(Mp: PMsgPort): Boolean;
begin
  IsMsgPortEmpty := True;
  if Assigned(mp) then
    IsMsgPortEmpty := mp^.mp_MsgList.lh_TailPred = PNode(@(mp^.mp_MsgList));
end;

function BitMask(no :ShortInt): LongInt;
begin
   BitMask := 1 shl no;
end;

function CreateExtIO(const Mp: PMsgPort; Size: Integer): PIORequest;
begin
  CreateExtIO := nil;
  if not Assigned(mp) then
    Exit;
  CreateExtIO := System.AllocMem(Size);
  if Assigned(CreateExtIO) then
  begin
    CreateExtIO^.io_Message.mn_Node.ln_Type := NT_REPLYMSG;
    CreateExtIO^.io_Message.mn_ReplyPort := Mp;
    CreateExtIO^.io_Message.mn_Length := Size;
  end;
end;

procedure DeleteExtIO(ioReq: PIORequest);
begin
  if Assigned(ioReq) then
  begin
    ioReq^.io_Message.mn_Node.ln_Type := Byte(-1);
    ioReq^.io_Device := Pointer(-1);
    ioReq^.io_Unit := Pointer(-1);
    System.FreeMem(ioReq);
  end;
end;

end.





