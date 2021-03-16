{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by Free Pascal development team

    exec.library functions for Amiga OS 4.x

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit exec;

{$PACKRECORDS 2}

// TODO: ExtMem.h - still missing ExtMemIFace discuss how to (open all possible interfaces)
//                  or make an other unit?
//       Initializers.h - Macros needed?

interface

type
  STRPTR   = PChar;
  PSTRPTR  = PPChar;
  ULONG    = Longword;
  LONG     = LongInt;
  APTR     = Pointer;
  BPTR     = LongInt; // Long word (BCPL) pointer
  BSTR     = LongInt; // Long word pointer to BCPL string
  BOOL     = SmallInt;
  UWORD    = Word;
  WORDBITS = Word;
  LONGBITS = LongWord;
  PLONGBITS = ^LONGBITS;
  UBYTE    = Byte;
  PULONG   = ^LongWord;
  PAPTR    = ^APTR;
  PLONG    = ^LONG;

type
// List Node Structure.  Each member in a list starts with a Node
  PNode = ^TNode;
  TNode =  record
    ln_Succ,           // Pointer to next (successor)
    ln_Pred: PNode;    // Pointer to previous (predecessor)
    ln_Type: Byte;
    ln_Pri: Shortint;  // Priority, for sorting
    ln_Name: STRPTR;   // ID string, null terminated
  end;  // Note: smallint aligned

// minimal node -- no type checking possible
  PMinNode = ^TMinNode;
  TMinNode =  record
    mln_Succ,
    mln_Pred: PMinNode;
  end;

// Note: Newly initialized IORequests, and software interrupt structures
// used with Cause(), should have type NT_UNKNOWN.  The OS will assign a type
// when they are first used.
// ----- Node Types for LN_TYPE -----
const
  NT_UNKNOWN      =  0;
  NT_TASK         =  1;  // Exec task
  NT_INTERRUPT    =  2;
  NT_DEVICE       =  3;
  NT_MSGPORT      =  4;
  NT_MESSAGE      =  5;  // Indicates message currently pending
  NT_FREEMSG      =  6;
  NT_REPLYMSG     =  7;  // Message has been replied
  NT_RESOURCE     =  8;
  NT_LIBRARY      =  9;
  NT_MEMORY       = 10;
  NT_SOFTINT      = 11;  // Internal flag used by SoftInits
  NT_FONT         = 12;
  NT_PROCESS      = 13;  // AmigaDOS Process
  NT_SEMAPHORE    = 14;
  NT_SIGNALSEM    = 15;  // signal semaphores
  NT_BOOTNODE     = 16;
  NT_KICKMEM      = 17;
  NT_GRAPHICS     = 18;
  NT_DEATHMESSAGE = 19;

  // New additions in V50
  NT_EXTINTERRUPT =  20;  // Native interrupt
  NT_EXTSOFTINT   =  21;  // Native soft interrupt
  NT_VMAREA       =  22;  // Internal use only
  NT_VMAREA_PROXY =  23;  // Internal use only
  NT_CLASS        =  24;  // Class
  NT_INTERFACE    =  25;  // Interface

  // New additions in V51
  NT_KMEMCACHE    =  26;  // Internal use only
  NT_RESERVED1    =  27;

  // New additions in V53
  NT_FILESYSTEM   =  28;  // For new style Vector-Port based Filesystems

  // New additions in V54
  NT_PAGE         =  40;

  NT_ELFHANDLE    =  41;
  NT_SOLIBHANDLE  =  42;

  NT_USER         = 254;  // User node types work down from here
  NT_EXTENDED     = 255;


// START consts and types from utility needed here

const
// differentiates user tags from control tags
 TAG_USER = $80000000;    // differentiates user tags from system tags
type
  Tag = LongWord;
  PTag = ^Tag;

  PTagItem = ^TTagItem;
  TTagItem = record
    ti_Tag: Tag;        // identifies the type of data
    ti_Data: LongWord;  // type-specific data
  end;
  PPTagItem = ^PTagItem;

  PHook = ^THook;
  THook = record
    h_MinNode: TMinNode;
    h_Entry: Pointer;    // assembler entry point
    h_SubEntry: Pointer; // often HLL entry point
    h_Data: Pointer;     // owner specific
  end;

// END consts and types from utility needed here


const
  {There is a problem with boolean
  vaules in taglists, just use this
  for now instead}
  LTrue : LongInt = 1;
  LFalse: LongInt = 0;

{
    This file defines Exec system lists, which are used to link
    various things.  Exec provides several routines to handle list
    processing (defined at the bottom of this file), so you can
    use these routines to save yourself the trouble of writing a list
    package.
}


type
// Full featured list header.
  PList = ^TList;
  TList =  record
    lh_Head: PNode;
    lh_Tail: PNode;
    lh_TailPred: PNode;
    lh_Type: Byte;
    l_pad: Byte;
  end; // word aligned

// Minimal List Header - no type checking
  PMinList = ^TMinList;
  TMinList =  record
    mlh_Head: PMinNode;
    mlh_Tail: PMinNode;
    mlh_TailPred: PMinNode;
  end; // LongWord aligned


{ ********************************************************************
*
*  Format of the alert error number:
*
*    +-+-------------+----------------+--------------------------------+
*    |D|  SubSysId   |  General Error |    SubSystem Specific Error    |
*    +-+-------------+----------------+--------------------------------+
*     1    7 bits          8 bits                  16 bits
*
*                    D:  Deadend alert
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


const

// ------ alert types
  AT_Deadend    = $80000000;
  AT_Recovery   = $00000000;

// ------ general purpose alert codes
  AG_NoMemory   = $00010000;
  AG_MakeLib    = $00020000;
  AG_OpenLib    = $00030000;
  AG_Opendev    = $00040000;
  AG_OpenRes    = $00050000;
  AG_IOError    = $00060000;
  AG_NoSignal   = $00070000;
  AG_BadParm    = $00080000;
  AG_CloseLib   = $00090000; // usually too many closes
  AG_CloseDev   = $000A0000; // or a mismatched close
  AG_ProcCreate = $000B0000; // Process creation failed
  AG_Obsolete   = $000C0000; // Obsolete feature used */

// ------ alert objects:
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
  AO_NewlibLib    = $0000800E;

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

// ------ exec.library

  AN_ExecLib     = $01000000;
  AN_ExcptVect   = $01000001; //  68000 exception vector checksum (obs.)
  AN_BaseChkSum  = $01000002; //  Execbase checksum (obs.)
  AN_LibChkSum   = $01000003; // Library checksum failure
  AN_IFaceChkSum = $01000004; // Interface checksum failure

  AN_MemCorrupt = $81000005; //  Corrupt memory list detected in FreeMem
  AN_IntrMem    = $81000006; //  No memory for interrupt servers
  AN_InitAPtr   = $01000007; //  InitStruct() of an APTR source (obs.)
  AN_SemCorrupt = $01000008; //  A semaphore is in an illegal state at ReleaseSempahore()

  AN_FreeTwice    = $01000009; //  Freeing memory already freed
  AN_BogusExcpt   = $8100000A; //  illegal 68k exception taken (obs.)
  AN_IOUsedTwice  = $0100000B; //  Attempt to reuse active IORequest
  AN_MemoryInsane = $0100000C; //  Sanity check on memory list failed during AvailMem(MEMF_LARGEST)
  AN_IOAfterClose = $0100000D; //  IO attempted on closed IORequest
  AN_StackProbe   = $0100000E; //  Stack appears to extend out of range
  AN_BadFreeAddr  = $0100000F; //  Memory header not located. [ Usually an invalid address passed to FreeMem() ]
  AN_BadSemaphore = $01000010; // An attempt was made to use the old message semaphores.
  AN_BadMemory    = $01000011; // A bad memory request was made (Realloc on non-allocated memory)
  AN_BadHook      = $01000012; // An uninitialized hook was called
// ------ graphics.library
  AN_GraphicsLib  = $02000000;
  AN_GfxNoMem     = $82010000;  //  graphics out of memory
  AN_GfxNoMemMspc = $82010001;  //  MonitorSpec alloc, no memory
  AN_LongFrame    = $82010006;  //  long frame, no memory
  AN_ShortFrame   = $82010007;  //  short frame, no memory
  AN_TextTmpRas   = $02010009;  //  text, no memory for TmpRas
  AN_BltBitMap    = $8201000A;  //  BltBitMap, no memory
  AN_RegionMemory = $8201000B;  //  regions, memory not available
  AN_MakeVPort    = $82010030;  //  MakeVPort, no memory
  AN_GfxNewError  = $0200000C;
  AN_GfxFreeError = $0200000D;

  AN_GfxNoLCM     = $82011234;  //  emergency memory not available

  AN_ObsoleteFont = $02000401;  //  unsupported font description used

// ------ layers.library

  AN_LayersLib    = $03000000;
  AN_LayersNoMem  = $83010000;  //  layers out of memory

// ------ intuition.library
  AN_Intuition    = $04000000;
  AN_GadgetType   = $84000001;  //  unknown gadget type
  AN_BadGadget    = $04000001;  //  Recovery form of AN_GadgetType
  AN_CreatePort   = $84010002;  //  create port, no memory
  AN_ItemAlloc    = $04010003;  //  item plane alloc, no memory
  AN_SubAlloc     = $04010004;  //  sub alloc, no memory
  AN_PlaneAlloc   = $84010005;  //  plane alloc, no memory
  AN_ItemBoxTop   = $84000006;  //  item box top < RelZero
  AN_OpenScreen   = $84010007;  //  open screen, no memory
  AN_OpenScrnRast = $84010008;  //  open screen, raster alloc, no memory
  AN_SysScrnType  = $84000009;  //  open sys screen, unknown type
  AN_AddSWGadget  = $8401000A;  //  add SW gadgets, no memory
  AN_OpenWindow   = $8401000B;  //  open window, no memory
  AN_BadState     = $8400000C;  //  Bad State Return entering Intuition
  AN_BadMessage   = $8400000D;  //  Bad Message received by IDCMP
  AN_WeirdEcho    = $8400000E;  //  Weird echo causing incomprehension
  AN_NoConsole    = $8400000F;  //  couldn't open the Console Device
  AN_NoISem       = $04000010;  // Intuition skipped obtaining a sem
  AN_ISemOrder    = $04000011;  // Intuition obtained a sem in bad order

// ------ math.library
  AN_MathLib      = $05000000;

// ------ dos.library
  AN_DOSLib       = $07000000;
  AN_StartMem     = $07010001; //  no memory at startup
  AN_endTask      = $07000002; //  endTask didn't
  AN_QPktFail     = $07000003; //  Qpkt failure
  AN_AsyncPkt     = $07000004; //  Unexpected packet received
  AN_FreeVec      = $07000005; //  Freevec failed
  AN_DiskBlkSeq   = $07000006; //  Disk block sequence error
  AN_BitMap       = $07000007; //  Bitmap corrupt
  AN_KeyFree      = $07000008; //  Key already free
  AN_BadChkSum    = $07000009; //  Invalid checksum
  AN_DiskError    = $0700000A; //  Disk Error
  AN_KeyRange     = $0700000B; //  Key out of range
  AN_BadOverlay   = $0700000C; //  Bad overlay
  AN_BadInitFunc  = $0700000D; //  Invalid init packet for cli/shell
  AN_FileReclosed = $0700000E; //  A filehandle was closed more than once
  AN_NoBootNode   = $0700000F; //  No bootnode found in eb_MountList

// ------ ramlib.library
  AN_RAMLib       = $08000000;
  AN_BadSegList = $08000001; // no overlays in library seglists

// ------ icon.library
  AN_IconLib = $09000000;

// ------ expansion.library
  AN_ExpansionLib     = $0A000000;
  AN_BadExpansionFree = $0A000001; // freeed free region

// ------ diskfont.library
  AN_DiskfontLib = $0B000000;

//------ newlib.library
  AN_NewlibLib  = $0E000000;

// ------ audio.device
  AN_AudioDev   = $10000000;

// ------ console.device
  AN_ConsoleDev = $11000000;
  AN_NoWindow   = $11000001; // Console can't open initial window

// ------ gameport.device
  AN_GamePortDev = $12000000;

// ------ keyboard.device
  AN_KeyboardDev        = $13000000;

// ------ trackdisk.device
  AN_TrackDiskDev = $14000000;
  AN_TDCalibSeek  = $14000001;  // calibrate: seek error
  AN_TDDelay      = $14000002;  // delay: error on timer wait

// ------ timer.device
  AN_TimerDev     = $15000000;
  AN_TMBadReq     = $15000001; // bad request
  AN_TMBadSupply  = $15000002; // power supply -- no 50/60Hz ticks

//------ cybppc.device
  AN_CybppcDev    = $16000000;
  AN_CybppcNoTerm = $16000001; // no termination
  AN_CybppcNoWide = $16000002; // no wide termination

// ------ cia.resource
  AN_CIARsrc      = $20000000;

// ------ disk.resource
  AN_DiskRsrc   = $21000000;
  AN_DRHasDisk  = $21000001; // get unit: already has disk
  AN_DRIntNoAct = $21000002; // interrupt: no active unit

// ------ misc.resource
  AN_MiscRsrc   = $22000000;

// ------ bootstrap
  AN_BootStrap  = $30000000;
  AN_BootError  = $30000001; // boot code returned an error

// ------ Workbench
  AN_Workbench          = $31000000;
  AN_NoFonts            = $B1000001;
  AN_WBBadStartupMsg1   = $31000001;
  AN_WBBadStartupMsg2   = $31000002;
  AN_WBBadIOMsg         = $31000003; // Hacker code?
  AN_WBReLayoutToolMenu = $B1010009; // GadTools broke?

// ------ DiskCopy
  AN_DiskCopy   = $32000000;

// ------ toolkit for Intuition
  AN_GadTools   = $33000000;

// ------ System utility library
  AN_UtilityLib = $34000000;

// ------ For use by any application that needs it
  AN_Unknown    = $35000000;


type
  PResident = ^TResident;
  TResident = record
    rt_MatchWord: Word;     // smallint to match on (ILLEGAL)
    rt_MatchTag: PResident; // pointer to the above
    rt_EndSkip: APTR;       // address to continue scan
    rt_Flags: Byte;         // various tag flags
    rt_Version: Byte;       // release version number
    rt_Type: Byte;          // type of module (NT_mumble)
    rt_Pri: Shortint;       // initialization priority
    rt_Name: STRPTR;        // pointer to node name
    rt_IdString: STRPTR;    // pointer to ident string
    rt_Init: APTR;          // pointer to init code
  end;

const
  RTC_MATCHWORD   = $4AFC; // The 68000 "ILLEGAL" instruction

// enResidentFlags
  RTF_AUTOINIT   = 1 shl 7; // rt_Init points to data structure
  RTF_NATIVE     = 1 shl 5; // rt_Init points to a native function (otherwise, 68k is assumed)
  RTF_AFTERDOS   = 1 shl 2;
  RTF_SINGLETASK = 1 shl 1;
  RTF_COLDSTART  = 1 shl 0;

type
// ****** MemChunk *****************************************************

  PMemChunk = ^TMemChunk;
  TMemChunk = record
    mc_Next: PMemChunk;  // pointer to next chunk
    mc_Bytes: LongWord;  // chunk byte size
  end;

// ****** MemHeader ****************************************************

  PMemHeader = ^TMemHeader;
  TMemHeader = record
    mh_Node: TNode;
    mh_Attributes: Word; // characteristics of this region
    mh_First: PMemChunk; // first free region
    mh_Lower: APTR;      // lower memory bound
    mh_Upper: APTR;      // upper memory bound + 1
    mh_Free: LongWord;   // total number of free bytes
  end;

// ****** MemEntry *****************************************************

  PMemEntry = ^TMemEntry;
  TMemEntry = record
    me_Un: record
      case LongInt of
         0: (meu_Reqs: LongWord);
         1: (meu_Addr: APTR);
      end;
    me_Length: LongWord;
  end;

// ****** MemList ******************************************************
// Note: sizeof(struct MemList) includes the size of the first MemEntry!
  PMemList = ^TMemList;
  TMemList = record
    ml_Node: TNode;
    ml_NumEntries: Word;             // number of entries in this struct
    ml_ME: array[0..0] of TMemEntry; // the first entry
  end;

// ----- Memory Requirement Types ---------------------------
// ----- See the AllocMem() documentation for details--------

const
  MEMF_ANY           = 0;        // Any type of memory will do
  MEMF_PUBLIC        = 1 shl  0;
  MEMF_CHIP          = 1 shl  1;
  MEMF_FAST          = 1 shl  2;
  MEMF_VIRTUAL       = 1 shl  3; // Memory that is mapped/paged
  MEMF_EXECUTABLE    = 1 shl  4; // Memory that contains executable code
  MEMF_LOCAL         = 1 shl  8; // Memory that does not go away at RESET
  MEMF_24BITDMA      = 1 shl  9; // DMAable memory within 24 bits of address
  MEMF_KICK          = 1 shl 10; // Memory that can be used for KickTags
  MEMF_PRIVATE       = 1 shl 11; // Memory that is only _visible_ to the  allocator task
  MEMF_SHARED        = 1 shl 12; // Memory that is visible and accessible  to all tasks

  MEMF_CLEAR         = 1 shl 16; // AllocMem: NL out area before return
  MEMF_LARGEST       = 1 shl 17; // AvailMem: return the largest chunk size
  MEMF_REVERSE       = 1 shl 18; // AllocMem: allocate from the top down
  MEMF_TOTAL         = 1 shl 19; // AvailMem: return total size of memory
  MEMF_HWALIGNED     = 1 shl 20; // AllocMem: Allocate aligned to hardware  page size
  MEMF_DELAYED       = 1 shl 21; // AllocMem: Delay physical memory mapping
  MEMF_CORE_RESIDENT = 1 shl 22; // Availmem: return only memory that is not paged out
  MEMF_NO_EXPUNGE    = 1 shl 31; // AllocMem: Do not cause expunge on failure

  MEM_BLOCKSIZE = 16;
  MEM_BLOCKMASK = MEM_BLOCKSIZE - 1;

type
// ***** MemHandlerData ************************************************
// Note:  This structure is *READ ONLY* and only EXEC can create it!
  PMemHandlerData = ^TMemHandlerData;
  TMemHandlerData = record
    memh_RequestSize: LongWord;  // Requested allocation size
    memh_RequestFlags: LongWord; // Requested allocation flags
    memh_Flags: LongWord;        // Flags (see below)
  end;

const
  MEMHF_RECYCLE  = 1 shl 0; // 0 = First time, 1 = recycle

// ***** Low Memory handler return values ******************************
  MEM_DID_NOTHING = 0;  // Nothing we could do...
  MEM_ALL_DONE    = -1; // We did all we could do
  MEM_TRY_AGAIN   = 1;  // We did some, try the allocation again

// ****** Memory attributes ********************************************
//enMemAttrs
  MEMATTRF_WRITETHROUGH     = 1 shl  0; // Stores in this area update cache  and memory
  MEMATTRF_CACHEINHIBIT     = 1 shl  1; // Caches are inhibited in this  area
  MEMATTRF_COHERENT         = 1 shl  2; // Coherency required, stores to  same region will be serialized
  MEMATTRF_GUARDED          = 1 shl  3; // Ensure in-order execute of memory accesses
  MEMATTRF_REFERENCED       = 1 shl  4; // Page containing memory location has been referenced (used)
  MEMATTRF_CHANGED          = 1 shl  5; // Page containing memory location has been changed

    // The following are mutually exclusive
  MEMATTRF_SUPER_RW         = 0 shl  6;
  MEMATTRF_SUPER_RW_USER_RO = 1 shl  6;
  MEMATTRF_SUPER_RW_USER_RW = 2 shl  6;
  MEMATTRF_SUPER_RO_USER_RO = 3 shl  6;
  MEMATTRF_RW_MASK          = 3 shl  6;

  MEMATTRF_EXECUTE          = 1 shl  9; // CPU can execute instructions from this memory

  MEMATTRF_NOT_MAPPED       = 1 shl 10; // Special flag: The memory is not mapped at all. This flag is only used as return value of GetMemoryAttr
  MEMATTRF_RESERVED1        = 1 shl 11; // Used by the system
  MEMATTRF_RESERVED2        = 1 shl 12; // _NEVER_ use these
  MEMATTRF_RESERVER3        = 1 shl 13;

// Short forms for common cases
  MEMATTRF_READ_WRITE = MEMATTRF_SUPER_RW_USER_RW;
  MEMATTRF_READ_ONLY  = MEMATTRF_SUPER_RO_USER_RO;

// ****** GetMemoryAttrs flags *****************************************
// enGetMemoryAttrsFlags
  GMAF_REPORT_CR = 1 shl 0;

// ****** AllocSysObject flags *****************************************
// enAllocSysObjectFlags
  ASOF_NOTRACK = 1 shl 1; // Used internally to indicate no tracking of object

// ****** Trackable ****************************************************
// Warning: Do NOT allocate one of those yourself!
type
  PTrackable = ^TTrackable;
  TTrackable = record
    Destructor_: THook;
    Object_: APTR;
    Flags: LongWord;
    HashChain: PTrackable;
  end; // Long word aligned

const
// enTrackableFlags
  TRACKF_COMPACT = 1 shl 0; // Trackable was allocated by "constructor"

// ****** DMA Scatter list *********************************************
type
  PDMAEntry = ^TDMAEntry;
  TDMAEntry = record
    PhysicalAddress: APTR; // Physically mapped address
    BlockLength: LongWord; // Length of the block in physical memory
  end;

// ****** Named memory scan message ************************************
  PSNMMessage = ^TSNMMessage;
  TSNMMessage = record
    Namespace: STRPTR;
    Name: STRPTR;
    Memory: APTR;
    Size: LongWord;
    Flags: LongWord;
  end;

const
  SNMF_NAMESPACES_ONLY = 1 shl 0;

type
  PInterrupt = ^TInterrupt;
  TInterrupt =  record
    is_Node: TNode;
    is_Data: APTR;       // Server data segment
    is_Code: TProcedure; // Server code entry
  end;

  PIntVector = ^TIntVector;
  TIntVector =  record   // For EXEC use ONLY!
    iv_Data: APTR;
    iv_Code: TProcedure;
    iv_Node: PNode;
  end;

  PSoftIntList = ^TSoftIntList;
  TSoftIntList =  record // For EXEC use ONLY!
    sh_List: TList;
    sh_Pad: Word;
  end;

  PExceptionContext = ^TExceptionContext;
  TExceptionContext = record
    Flags: LongWord;               // Flags, describing the context (READ-ONLY)
    Traptype: LongWord;            // Type of trap (READ-ONLY)
    msr: LongWord;                 // Machine state
    ip: LongWord;                  // Return instruction pointer
    gpr: array[0..31] of LongWord; // r0 - r31
    cr: LongWord;                  // Condition code register
    xer: LongWord;                 // Extended exception register
    ctr: LongWord;                 // Count register
    lr: LongWord;                  // Link register
    dsisr: LongWord;               // DSI status register. Only set when valid
    dar: LongWord;                 // Data address register. Only set when valid
    fpr: array[0..31] of double;   // Floating point registers
    fpscr: QWord;                  // Floating point control and status register
    // The following are only used on AltiVec
    vscr: array[0..15] of Byte;    // AltiVec vector status and control register
    vr: array[0..511] of Byte;     // AltiVec vector register storage
    vrsave: LongWord;              // AltiVec VRSAVE register
  end;

const
// Flags for ExceptionContext
// enECFlags
  ECF_FULL_GPRS = 1 shl 0; // Set if all register have been saved
                           // If this flag is not set; gpr[14] through
                           // gpr[31] are invalid
  ECF_FPU       = 1 shl 1; // Set if the FPU registers have been saved
  ECF_FULL_FPU  = 1 shl 2; // Set if all FPU registers have been saved
  ECF_VECTOR    = 1 shl 3; // Set if vector registers have been saved
  ECF_VRSAVE    = 1 shl 4; // Set if VRSAVE reflects state of vector registers saved

// Flags for ReadTaskContext/WriteTaskContext
// enRTCFlags
  RTCF_SPECIAL  = 1 shl 0;
  RTCF_STATE    = 1 shl 1;
  RTCF_GENERAL  = 1 shl 2;
  RTCF_FPU      = 1 shl 3;
  RTCF_VECTOR   = 1 shl 4;
  RTCF_INFO     = 1 shl 5;

  SIH_PRIMASK = $F0;
//**********************************************************************
// this is a fake INT definition, used only for AddIntServer and the like
  INTB_NMI    = 15;
  INTF_NMI    = 1 shl INTB_NMI;

//  These are used with AddIntServer/SetIntVector to install global
// trap handlers and with SetTaskTrap to install local task traps
// Note: Use of these global trap handlers should be
// restricted to system and debugger use. You should normally
// use the task's local trap handler.

// enTrapNumbers
  TRAPNUM_BUS_ERROR              = $01000000; // Bus error exception/machine check
  TRAPNUM_DATA_SEGMENT_VIOLATION = $02000000; // Data segment violation
  TRAPNUM_INST_SEGMENT_VIOLATION = $03000000; // Instruction segment violation
  TRAPNUM_ALIGNMENT              = $04000000; // Alignment violation
  TRAPNUM_ILLEGAL_INSTRUCTION    = $05000000; // Illegal instruction
  TRAPNUM_PRIVILEGE_VIOLATION    = $06000000; // Privilege violation
  TRAPNUM_TRAP                   = $07000000; // Trap instruction
  TRAPNUM_FPU                    = $08000000; // Floating point related (FPU disabled; imprecise)
  TRAPNUM_TRACE                  = $09000000; // Single step trace exception
  TRAPNUM_DATA_BREAKPOINT        = $0a000000; // Data breakpoint
  TRAPNUM_INST_BREAKPOINT        = $0b000000; // Instruction breakpoint
  TRAPNUM_PERFORMANCE            = $0c000000; // Performance monitor (System use only)
  TRAPNUM_THERMAL                = $0d000000; // Thermal management (System use only)
  TRAPNUM_RESERVED1              = $0e000000; // reserved
  TRAPNUM_ALTIVEC_ASSIST         = $0f000000; // AltiVec Assist
  TRAPNUM_SMI                    = $10000000; // System Management interrupt

  TRAPNUM_NUMTRAPS               = 16;          // Number of hardware traps

// Every Amiga Task has one of these Task structures associated with it.
// To find yours, use FindTask(Nil).  AmigaDOS processes tack a few more
// values on to the end of this structure, which is the difference between
// Tasks and Processes.
type
  PTask = ^TTask;
  TTask = record
    tc_Node: TNode;
    tc_Flags: Byte;
    tc_State: Byte;
    tc_IDNestCnt: ShortInt; // intr disabled nesting
    tc_TDNestCnt: ShortInt; // task disabled nesting
    tc_SigAlloc: LongWord;  // sigs allocated
    tc_SigWait: LongWord;   // sigs we are waiting for
    tc_SigRecvd: LongWord;  // sigs we have received
    tc_SigExcept: LongWord; // sigs we will take excepts for
      //The following field used to be this tc_TrapAlloc, tc_TrapAble, It was replaced by a pointer to an private extended task structure.
    tc_ETask: APTR;
    tc_ExceptData: APTR;    // points to except data
    tc_ExceptCode: APTR;    // points to except code
    tc_TrapData: APTR;      // points to trap data
    tc_TrapCode: APTR;      // points to trap code
    tc_SPReg: APTR;         // stack APTR
    tc_SPLower: APTR;       // stack lower bound
    tc_SPUpper: APTR;       // stack upper bound + 2
    tc_Switch: TProcedure;  // task losing CPU
    tc_Launch: TProcedure;  // task getting CPU
    tc_MemEntry: TList;     // allocated memory
    tc_UserData: APTR;      // per task data
  end;

// Stack swap structure as passed to StackSwap()
  PStackSwapStruct = ^TStackSwapStruct;
  TStackSwapStruct = record
    stk_Lower: APTR;     // Lowest byte of stack
    stk_Upper: LongWord; // Upper end of stack (size + Lowest)
    stk_Pointer: APTR;   // Stack pointer at switch point
  end;

// ----- Flag Bits ------------------------------------------
const
// enTaskFlagsBits
  TB_PROCTIME         = 0;
  TB_ETASK            = 3;
  TB_STACKCHK         = 4;
  TB_EXCEPT           = 5;
  TB_SWITCH           = 6;
  TB_LAUNCH           = 7;
// enTaskFlags
  TF_PROCTIME         = 1;
  TF_ETASK            = 8;
  TF_STACKCHK         = 16;
  TF_EXCEPT           = 32;
  TF_SWITCH           = 64;
  TF_LAUNCH           = 128;

// ----- Task States ----------------------------------------
// enTaskState
  TS_INVALID          = 0;
  TS_ADDED            = 1;
  TS_RUN              = 2;
  TS_READY            = 3;
  TS_WAIT             = 4;
  TS_EXCEPT           = 5;
  TS_REMOVED          = 6;
  TS_CRASHED          = 7;
  TS_SUSPENDED        = 8;

// ----- Predefined Signals -------------------------------------
// enTaskSignalBits
  SIGB_ABORT          = 0;
  SIGB_CHILD          = 1;
  SIGB_BLIT           = 4;
  SIGB_SINGLE         = 4;
  SIGB_INTUITION      = 5;
  SIGB_NET            = 7;
  SIGB_DOS            = 8;

// enTaskSignal
  SIGF_ABORT     = 1 shl SIGB_ABORT;
  SIGF_CHILD     = 1 shl SIGB_CHILD;
  SIGF_BLIT      = 1 shl SIGB_BLIT;
  SIGF_SINGLE    = 1 shl SIGB_SINGLE;
  SIGF_INTUITION = 1 shl SIGB_INTUITION;
  SIGF_NET       = 1 shl SIGB_NET;
  SIGF_DOS       = 1 shl SIGB_DOS;

// enSuspendBits
  STB_CRASHED = 0;
  STB_REMOVED = 1;

// enSuspendFlags
  STF_CRASHED = 1 shl STB_CRASHED;
  STF_REMOVED = 1 shl STB_REMOVED;

// This file defines ports and messages, which are used for inter-
// task communications using the routines defined toward the
// bottom of this file.
type
// ****** MsgPort ******************************************************
  PMsgPort = ^TMsgPort;
  TMsgPort = record
    mp_Node: TNode;
    mp_Flags: Byte;
    mp_SigBit: Byte;   // signal bit number
    mp_SigTask: APTR;  // task to be signalled (TaskPtr)
    mp_MsgList: TList; // message linked list
  end;

// ****** Message ******************************************************

  PMessage = ^TMessage;
  TMessage =  record
    mn_Node: TNode;
    mn_ReplyPort: PMsgPort; // message reply port
    mn_Length: Word;        // total message length, in bytes, (include the size of the Message structure in the length)
   end;

// mp_Flags: Port arrival actions (PutMsg)
const
// enMsgPortFlags
  PF_ACTION  = 3;        // Mask
  PF_SIGALLOC = 1 shl 7; // Internal use

// enMsgPortActions
  PA_SIGNAL  = 0; // Signal task in mp_SigTask
  PA_SOFTINT = 1; // Signal SoftInt in mp_SoftInt/mp_SigTask
  PA_IGNORE  = 2; // Ignore arrival


// ****** SignalSemaphore **********************************************
type
// Private structure used by ObtainSemaphore()
  PSemaphoreRequest = ^TSemaphoreRequest;
  TSemaphoreRequest = record
    sr_Link: TMinNode;
    sr_Waiter: PTask;
  end;

// Signal Semaphore data structure
  PSignalSemaphore = ^TSignalSemaphore;
  TSignalSemaphore = record
      ss_Link: TNode;
      ss_NestCount: SmallInt;
      ss_WaitQueue: TMinList;
      ss_MultipleLink: TSemaphoreRequest;
      ss_Owner: PTask;
      ss_QueueCount: SmallInt;
  end;

// ***** Semaphore procure message (for use in V39 Procure/Vacate ***
  PSemaphoreMessage = ^TSemaphoreMessage;
  TSemaphoreMessage = record
    ssm_Message: TMessage;
    ssm_Semaphore: PSignalSemaphore;
  end;

const
// enSemaphoreRequestType
 SM_SHARED      = 1;
 SM_EXCLUSIVE   = 0;

const
// ------ Special constants ---------------------------------------
// Note: This only applies to "legacy" 68k-based functions
  LIB_VECTSIZE  =  6;   //  Each library entry takes 6 bytes
  LIB_RESERVED  =  4;   //  Exec reserves the first 4 vectors
  LIB_BASE      = -LIB_VECTSIZE;
  LIB_USERDEF   = LIB_BASE - (LIB_RESERVED * LIB_VECTSIZE);
  LIB_NONSTD    = LIB_USERDEF;

// ------ Standard functions --------------------------------------
// Note: This only applies to "legacy" 68k-based functions
  LIB_OPEN    =  -6;
  LIB_CLOSE   = -12;
  LIB_EXPUNGE = -18;
  LIB_EXTFUNC = -24;  // for future expansion

type
// ------ Library Base Structure ----------------------------------
//  Also used for Devices and some Resources

  PLibrary = ^TLibrary;
  TLibrary =  record
    lib_Node: TNode;
    lib_Flags: Byte;
    lib_ABIVersion: Byte; // ABI exported by library
    lib_NegSize: Word;    // number of bytes before library
    lib_PosSize: Word;    // number of bytes after library
    lib_Version: Word;    // major
    lib_Revision: Word;   // minor
    lib_IdString: STRPTR; // ASCII identification
    lib_Sum: LongWord;    // the checksum itself  }
    lib_OpenCnt: Word;    // number of current opens  }
  end;   // Warning: size is not a longword multiple!

const

// lib_ABIVersion definitions
// enABIVersion
  LIBABI_68K    = 0; // A 68k library (pre OS4)
  LIBABI_MIFACE = 1; // V50 multi interface library

// lib_Flags bit definitions (all others are system reserved)
  LIBF_SUMMING = 1 shl 0; // we are currently checksumming
  LIBF_CHANGED = 1 shl 1; // we have just changed the lib
  LIBF_SUMUSED = 1 shl 2; // set if we should bother to sum
  LIBF_DELEXP  = 1 shl 3; // delayed expunge
  LIBF_EXP0CNT = 1 shl 4;


// This file defines the constants and types required to use
// Amiga device IO routines, which are also defined here.

type

//***** Device *********************************************************
  PDevice = ^TDevice;
  TDevice =  record
    dd_Library: TLibrary;
  end;

//***** Unit ***********************************************************
  PUnit = ^TUnit;
  TUnit = record
      unit_MsgPort: TMsgPort; // queue for unprocessed messages instance of msgport is recommended
      unit_flags: Byte;
      unit_pad: Byte;
      unit_OpenCnt: Word;     // number of active opens
  end;

const
  UNITF_ACTIVE  = 1 shl 0;
  UNITF_INTASK  = 1 shl 1;

type
  PIORequest = ^TIORequest;
  TIORequest =  record
    io_Message: TMessage;
    io_Device: PDevice; // device node pointer
    io_Unit: PUnit;     // unit (driver private)
    io_Command: Word;   // device command
    io_Flags: Byte;
    io_Error: ShortInt; // error or warning num
  end;

  PIOStdReq = ^TIOStdReq;
  TIOStdReq =  record
    io_Message: TMessage;
    io_Device: PDevice;  // device node pointer
    io_Unit: PUnit;      // unit (driver private)
    io_Command: Word;    // device command
    io_Flags: Byte;
    io_Error: ShortInt;  // error or warning num
    io_Actual: LongWord; // actual number of bytes transferred
    io_Length: LongWord; // requested number bytes transferred
    io_Data: APTR;       // points to data area
    io_Offset: LongWord; // offset for block structured devices
  end;


// library vector offsets for device reserved vectors }
const
// enum enDeviceLibraryReserved
  DEV_beginIO = -30;
  DEV_ABORTIO = -36;

// io_Flags defined bits

  IOB_QUICK   = 0;
  IOF_QUICK   = 1 shl IOB_QUICK;

// enDefaultDeviceCommands
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

{$PACKRECORDS C}
//**********************************************************************
// The interface is the new way for exec libraries.
// Basically, the interface is embedded in a table of
// function pointers similar to the old library jump table.
//
// FIXME: Add some more documentation
type
  TInterfaceData = record
    Link: TNode;             // Node for linking several interfaces
    LibBase: PLibrary;       // Library this interface belongs to

    RefCount: LongWord;      // Reference count
    Version: LongWord;       // Version number of the interface
    Flags: LongWord;         // Various flags (see below)
    CheckSum: LongWord;      // Checksum of the interface
    PositiveSize: LongWord;  // Size of the function pointer part, (with this struct)
    NegativeSize: LongWord;  // Size of the data area
    IExecPrivate: APTR;      // Private copy of IExec
    EnvironmentVector: APTR; // Base address for base relative code
    Reserved3: LongWord;
    Reserved4: LongWord;
  end;

  TInterface = record
    Data: TInterfaceData;
    // some functions, calling convention?
  end;
  PInterface = ^TInterface;
{$PACKRECORDS 2}

const
// Flags for the Flags field in interfaces and as flags parameter for GetInterface
// enInterfaceFlags
  IFLF_NONE          = $0000; // No flags
  IFLF_PROTECTED     = $0001; // This interface can't be SetMethod'd
  IFLF_NOT_NATIVE    = $0002; // Interface is 68k
  IFLF_PRIVATE       = $0004; // Interface is a private, non-shareable instance
  IFLF_CHANGED       = $0008; // Interface has been changed, ready for re-summing
  IFLF_UNMODIFIED    = $0010; // Interface is unmodified. This flag will be set
                               // if the interface is created, and reset as soon
                               // as someone uses SetMethod on it.
  IFLF_CLONED        = $0020; // Interface was created by Clone method and will
                               // have to be freed via Expunge(). Interface
                               // implementors must set this bit in Clone().
  IFLF_CLONE_EXPUNGE = $0040;  // Call Expunge() on cloned interface when the
                               // reference count reaches zero. (V53.31)


//  Definition of the Exec library base structure (pointed to by location 4).
// Most fields are not to be viewed or modified by user programs.  Use
// extreme caution.

type
  PExecBase = ^TExecBase;
  TExecBase =  Record
    LibNode: TLibrary; //  Standard library node

// ******* Static System Variables *************************************

    SoftVer: Word;          // kickstart release number (obs.)
    LowMemChkSum: SmallInt; // checksum of 68000 trap vectors
    ChkBase: LongWord;      // system base pointer complement
    ColdCapture: APTR;      //  coldstart soft capture vector
    CoolCapture: APTR;      //  coolstart soft capture vector
    WarmCapture: APTR;      //  warmstart soft capture vector
    SysStkUpper: APTR;      //  system stack base   (upper bound)
    SysStkLower: APTR;      //  top of system stack (lower bound)
    MaxLocMem: LongWord;    //  top of chip memory
    DebugEntry: APTR;       //  global debugger entry point
    DebugData: APTR;        //  global debugger data segment
    AlertData: APTR;        //  alert data segment
    MaxExtMem: APTR;        //  top of extended mem, or nil if none

    ChkSum: Word;           //  for all of the above (minus 2)

// ****** Interrupt Related ********************************************

    IntVects: array[0..15] of TIntVector;

// ****** Dynamic System Variables *************************************

    ThisTask: PTask;     // pointer to current task (readable)

    IdleCount: LongWord; //  idle counter
    DispCount: LongWord; //  dispatch counter
    Quantum: Word;       //  time slice quantum
    Elapsed: Word;       //  current quantum ticks
    SysFlags: Word;      //  misc internal system flags
    IDNestCnt: ShortInt; //  interrupt disable nesting count
    TDNestCnt: ShortInt; //  task disable nesting count

    AttnFlags: Word;     //  special attention flags (readable)

    AttnResched: Word;   //  rescheduling attention
    ResModules: APTR;    //  resident module array pointer
    TaskTrapCode: APTR;
    TaskExceptCode: APTR;
    TaskExitCode: APTR;
    TaskSigAlloc: LongWord;
    TaskTrapAlloc: Word;


// ****** System Lists (private!) **************************************

    MemList: TList;
    ResourceList: TList;
    DeviceList: TList;
    IntrList: TList;
    LibList: TList;
    PortList: TList;
    TaskReady: TList;
    TaskWait: TList;

    SoftInts: array[0..4] of TSoftIntList;

// ****** Other Globals ************************************************

    LastAlert: array[0..3] of LongInt;

    // these next two variables are provided to allow
    // system developers to have a rough idea of the
    // period of two externally controlled signals --
    // the time between vertical blank interrupts and the
    // external line rate (which is counted by CIA A's
    // "time of day" clock).  In general these values
    // will be 50 or 60, and may or may not track each
    // other.  These values replace the obsolete AFB_PAL
    // and AFB_50HZ flags.
    VBlankFrequency: Byte;      //  (readable)
    PowerSupplyFrequency: Byte; //  (readable)

    SemaphoreList: TList;

    // these next two are to be able to kickstart into user ram.
    // KickMemPtr holds a singly linked list of MemLists which
    // will be removed from the memory list via AllocAbs.  If
    // all the AllocAbs's succeeded, then the KickTagPtr will
    // be added to the rom tag list.
    KickMemPtr: APTR;    //  ptr to queue of mem lists
    KickTagPtr: APTR;    //  ptr to rom tag queue
    KickCheckSum: APTR;  //  checksum for mem and tags

// ****** V36 Exec additions start here ********************************

    ex_Pad0: Word;            // Private internal use
    ex_LaunchPoint: LongWord; // Private to Launch/Switch
    ex_RamLibPrivate: APTR;

    //  The next LongWord contains the system "E" clock frequency,
    // expressed in Hertz.  The E clock is used as a timebase for
    // the Amiga's 8520 I/O chips. (E is connected to "02").
    // Typical values are 715909 for NTSC, or 709379 for PAL.
    ex_EClockFrequency: LongWord; //  (readable)
    ex_CacheControl: LongWord;    //  Private to CacheControl calls
    ex_TaskID: LongWord;          //  Next available task ID

    ex_Reserved1: array[0..4] of LongWord;

    ex_MMULock: APTR;    //  private

    ex_Reserved2: array[0..2] of LongWord;

// ****** V39 Exec additions start here ********************************

    // The following list and data element are used
    //for V39 exec's low memory handler...
    ex_MemHandlers: TMinList; // The handler list
    ex_MemHandler: APTR;      // Private! handler pointer

// ****** V50 Exec additions start here ********************************

    MainInterface: PInterface; // ExecLibrary's primary interface
    Private01: APTR;
    Private02: LongWord;
    Private03: APTR;
    Private04: APTR;
    Private05: APTR;
    Private06: TList;
    Private07: APTR;
    EmuWS: APTR;          // Emulator Workspace. Legacy libraries might access this field
  // Yes, there are more additions, but you don't need to know what it is
  end;

// ***** Bit defines for AttnFlags (see above) *************************

// Processors and Co-processors:
// enAttnFlagBits
const
  AFB_68010     = 0; // also set for 68020
  AFB_68020     = 1; // also set for 68030
  AFB_68030     = 2; // also set for 68040
  AFB_68040     = 3; // also set for 68060
  AFB_68881     = 4; // also set for 68882
  AFB_68882     = 5;
  AFB_FPU40     = 6; // Set if 68040 FPU
  AFB_68060     = 7;

  // The following flags are new to V50
  AFB_603     =  8;
  AFB_604     =  9;
  AFB_750     = 10;
  AFB_7400    = 11;
  AFB_ALTIVEC = 12;
  AFB_4XX     = 13;
  AFB_OTHER   = 14;

// The AFB_FPU40 bit is set when a working 68040 FPU
// is in the system.  If this bit is set and both the
// AFB_68881 and AFB_68882 bits are not set, then the 68040
// math emulation code has not been loaded and only 68040
// FPU instructions are available.  This bit is valid *ONLY*
// if the AFB_68040 bit is set.
// Likewise, AFB_ALTIVEC identifies an existing AltiVec unit.

  AFB_PRIVATE = 15; // Just what it says

// enAttnFlags
  AFF_68010   = 1 shl  0;
  AFF_68020   = 1 shl  1;
  AFF_68030   = 1 shl  2;
  AFF_68040   = 1 shl  3;
  AFF_68881   = 1 shl  4;
  AFF_68882   = 1 shl  5;
  AFF_FPU40   = 1 shl  6;
  AFF_68060   = 1 shl  7;

  AFF_603     = 1 shl  8;
  AFF_604     = 1 shl  9;
  AFF_750     = 1 shl 10;
  AFF_7400    = 1 shl 11;
  AFF_ALTIVEC = 1 shl 12;
  AFF_4XX     = 1 shl 13;
  AFF_OTHER   = 1 shl 14;

  AFF_PRIVATE = 1 shl 15;


//***** Selected flag definitions for Cache manipulation calls *********
  CACRF_EnableI       = 1 shl  0; // Enable instruction cache
  CACRF_FreezeI       = 1 shl  1; // Freeze instruction cache
  CACRF_ClearI        = 1 shl  3; // Clear instruction cache
  CACRF_IBE           = 1 shl  4; // Instruction burst enable
  CACRF_EnableD       = 1 shl  8; // Enable data cache
  CACRF_FreezeD       = 1 shl  9; // Freeze data cache
  CACRF_ClearD        = 1 shl 11; // Clear data cache (flush to memory first)
  CACRF_DBE           = 1 shl 12; // 68030 Data burst enable
  CACRF_WriteAllocate = 1 shl 13; // 68030 Write-Allocate mode (must always be set!)
  CACRF_InvalidateD   = 1 shl 15; // Invalidate data cache (no writeback)
  CACRF_EnableE       = 1 shl 30; // Master enable for external caches.
                                        // External caches should track the
                                        // state of the internal caches
                                        // such that they do not cache anything
                                        // that the internal cache turned off
                                        // for.
  CACRF_CopyBack      = 1 shl 31; // Master enable for copyback caches

// enDMAFlags
  DMA_Continue     = 1 shl 1;      // Continuation flag for CachePreDMA
  DMAF_Continue    = DMA_Continue;
  DMA_NoModify     = 1 shl 2;      // Set if DMA does not update memory
  DMAF_NoModify    = DMA_NoModify;
  DMA_ReadFromRAM  = 1 shl 3;      // Set if DMA goes *FROM* RAM to device
  DMAF_ReadFromRAM = DMA_ReadFromRAM;

// The only fixed address in the Amiga memory space.
  AbsExecBase: PExecBase = Pointer(4);

// Don't even think about the contents of this structure. Just embed it and reference it
type
  PAVLNode = ^TAVLNode;
  TAVLNode = record
    Reserved: array[0..3] of LongWord;
  end;
  PPAVLNode = ^PAVLNode;

  TAVLKey = Pointer;

  PAVLNODECOMP = ^AVLNODECOMP;
  AVLNODECOMP = APTR;

  PAVLKEYCOMP = ^AVLKEYCOMP;
  AVLKEYCOMP = APTR;

const
// Minimum size of the buffers to receive disassembled opcodes
// and operands including the NIL string terminator.
  LEN_DISASSEMBLE_OPCODE_STRING   = 15;
  LEN_DISASSEMBLE_OPERANDS_STRING = 40;

type
// The StackFrameMsg is used when traversing a Task's stack.
  TStackFrameMsg = record
    StructSize: LongWord;    // Size of the data structure
    State: LongWord;         // State of the stack frame
    MemoryAddress: APTR;     // Memory address being pointed to
    StackPointer: PLongWord; // The stack pointer itself
  end;

const
//enStackFrameMsgState
  STACK_FRAME_DECODED               = 1;  // Decoded stack frame
  STACK_FRAME_INVALID_BACKCHAIN_PTR = 2;  // Invalid backchain pointer
  STACK_FRAME_TRASHED_MEMORY_LOOP   = 3;  // Memory loop caused by trashed memory
  STACK_FRAME_BACKCHAIN_PTR_LOOP    = 4;   // Backchain pointer loops

// Each DebugSymbol corresponds to some memory address.
type
  TDebugSymbol = record
    StructSize: LongWord;       // Size of the data structure
    Type_: LongWord;            // Type of debug symbol

    Name: STRPTR;               // Module name (may be NULL)
    Offset: LongWord;           // Offset into the module
    SegmentNumber: LongWord;    // DOS segment number
    SegmentOffset: LongWord;    // DOS segment offset

    SourceFileName: STRPTR;     // Source code file name (may be NULL)
    SourceLineNumber: LongWord; // Source code line number (may be zero)
    SourceFunctionName: STRPTR; // Source code function name (may be NULL)
    SourceBaseName: STRPTR;     // Source code base name (may be NULL)
  end;

const
// enDebugSymbolType
  DEBUG_SYMBOL_68K_MODULE    = 1; // 68K kernel module
  DEBUG_SYMBOL_NATIVE_MODULE = 2; // Kernel data module
  DEBUG_SYMBOL_KERNEL_MODULE = 3; // Kernel module
  DEBUG_SYMBOL_MODULE        = 4; // Module
  DEBUG_SYMBOL_MODULE_STABS  = 5; // Module with stabs debug

type
// Opaque datatype for the result of StartDebugOutputNotify.
  TDebugOutputNotify = record
  end;

// he task that should be signaled with the given
// signal mask when the debug output buffer was updated.
const
  SDONA_Task = TAG_USER + $1021000; // PTask
// The signal mask that shall be used
  SDONA_SignalMask = TAG_USER + $1021001; // LongWord

type
//**********************************************************************
// The following structure can be used to switch from 68k into PPC code,
// in general its use is discouraging but necessary in a few places.
  TEmuTrapfunction = procedure(Reg68K: PLongWord);

  TEmuTrap = record
    Instruction: LongWord;      // TRAPINST, see below
    Type_: Byte;                // TRAPTYPE or TRAPTYPENR
    Function_: TEmutrapFunction; // PPC function address also see "enRegConst" below but watch out byteoffsets!
  end;

const
  TRAPINST   = $4ef80000;  // jmp.w 0, indicate switch
  TRAPTYPE   = $0004;      // type of this trap (result in r3/d0)
  TRAPTYPENR = $0005;      // same as above but no return value

//**********************************************************************

// enRegConst
  REG68K_D0  =   0;
  REG68K_D1  =   4;
  REG68K_D2  =   8;
  REG68K_D3  =  12;
  REG68K_D4  =  16;
  REG68K_D5  =  20;
  REG68K_D6  =  24;
  REG68K_D7  =  28;
  REG68K_A0  =  32;
  REG68K_A1  =  36;
  REG68K_A2  =  40;
  REG68K_A3  =  44;
  REG68K_A4  =  48;
  REG68K_A5  =  52;
  REG68K_A6  =  56;
  REG68K_A7  =  60;

  REG68K_FP0 =  64;
  REG68K_FP1 =  72;
  REG68K_FP2 =  80;
  REG68K_FP3 =  88;
  REG68K_FP4 =  96;
  REG68K_FP5 = 104;
  REG68K_FP6 = 112;
  REG68K_FP7 = 120;

//**********************************************************************
// Tag Items for Emulate() system call

  ET_RegisterD0     = TAG_USER +  1;
  ET_RegisterD1     = TAG_USER +  2;
  ET_RegisterD2     = TAG_USER +  3;
  ET_RegisterD3     = TAG_USER +  4;
  ET_RegisterD4     = TAG_USER +  5;
  ET_RegisterD5     = TAG_USER +  6;
  ET_RegisterD6     = TAG_USER +  7;
  ET_RegisterD7     = TAG_USER +  8;

  ET_RegisterA0     = TAG_USER +  9;
  ET_RegisterA1     = TAG_USER + 10;
  ET_RegisterA2     = TAG_USER + 11;
  ET_RegisterA3     = TAG_USER + 12;
  ET_RegisterA4     = TAG_USER + 13;
  ET_RegisterA5     = TAG_USER + 14;
  ET_RegisterA6     = TAG_USER + 15;
  ET_RegisterA7     = TAG_USER + 16;

  ET_NoJIT          = TAG_USER + 17;

  ET_FPRegisters    = TAG_USER + 18;
  ET_FPRegisterMask = TAG_USER + 19;

  ET_SuperState     = TAG_USER + 20;

  ET_Offset         = TAG_USER + 21;

  ET_StackPtr       = TAG_USER + 22;

  ET_SaveRegs       = TAG_USER + 23;
  ET_SaveParamRegs  = TAG_USER + 24;

//**********************************************************************
// enEmulateFPFlags

  EFPF_FP0 = 1;
  EFPF_FP1 = 1 shl 1;
  EFPF_FP2 = 1 shl 2;
  EFPF_FP3 = 1 shl 3;
  EFPF_FP4 = 1 shl 4;
  EFPF_FP5 = 1 shl 5;
  EFPF_FP6 = 1 shl 6;
  EFPF_FP7 = 1 shl 7;

//**********************************************************************
// enDeviceIOErrors

  IOERR_SUCCESS    =  0; // no error
  IOERR_OPENFAIL   = -1; // device/unit failed to open
  IOERR_ABORTED    = -2; // request terminated early [after AbortIO()]
  IOERR_NOCMD      = -3; // command not supported by device
  IOERR_BADLENGTH  = -4; // not a valid length (usually IO_LENGTH)
  IOERR_BADADDRESS = -5; // invalid address (misaligned or bad range)
  IOERR_UNITBUSY   = -6; // device opens ok, but requested unit is busy
  IOERR_SELFTEST   = -7; // hardware failed self-test

//**********************************************************************
//Tag items used by AllocSysObject

  ASO_NoTrack         = TAG_USER +  1; // Don't track this object (i.e. do not free automatically;
  ASO_MemoryOvr       = TAG_USER +  2; // Memory type override
// IORequest
  ASOIOR_Size         = TAG_USER + 10; // Size of the object
  ASOIOR_ReplyPort    = TAG_USER + 11; // ReplyPort to use
  ASOIOR_Duplicate    = TAG_USER + 12; // Source IO request to duplicate
// Hook
  ASOHOOK_Size        = TAG_USER + 10; // Size of the object
  ASOHOOK_Entry       = TAG_USER + 11; // The hook's entry
  ASOHOOK_Subentry    = TAG_USER + 12; // The hook's subentry field
  ASOHOOK_Data        = TAG_USER + 13; // The hook's user data
// Interrupt
  ASOINTR_Size        = TAG_USER + 10; // Size of the object
  ASOINTR_Code        = TAG_USER + 11; // Code pointer
  ASOINTR_Data        = TAG_USER + 12; // Data pointer
  ASOINTR_SoftInt     = TAG_USER + 13; // Interrupt is used by Cause(;
// List
  ASOLIST_Size        = TAG_USER + 10; // Size of the object
  ASOLIST_Type        = TAG_USER + 11; // Type to set for the list
  ASOLIST_Min         = TAG_USER + 12; // Create a minlist
// DMAEntry array
  ASODMAE_Size        = TAG_USER + 10; // Raw size of the object
  ASODMAE_NumEntries  = TAG_USER + 11; // Number of entries
// List node
  ASONODE_Size        = TAG_USER + 10; // Size of the node
  ASONODE_Min         = TAG_USER + 11; // Make it a MinNode
  ASONODE_Type        = TAG_USER + 12; // Node's type
  ASONODE_Pri         = TAG_USER + 13; // Node's priority
  ASONODE_Name        = TAG_USER + 14; // Pointer to a node string
// Message port
  ASOPORT_Size        = TAG_USER + 10; // Size of the object
  ASOPORT_AllocSig    = TAG_USER + 11; // Allocate a signal
  ASOPORT_Action      = TAG_USER + 12; // Action at message arrival (see ports.h, enMsgPortActions;
  ASOPORT_Pri         = TAG_USER + 13; // Priority used when the port is added to a list
  ASOPORT_Name        = TAG_USER + 14; // Name for the port
  ASOPORT_Signal      = TAG_USER + 15; // Preallocted signal number
  ASOPORT_Target      = TAG_USER + 16; // MsgPort's target, either a task, or a softint
  ASOPORT_Public      = TAG_USER + 17; // Make the port public
  ASOPORT_CopyName    = TAG_USER + 18; // Copy the name string
// Message
  ASOMSG_Size         = TAG_USER + 10; // Size of the object
  ASOMSG_ReplyPort    = TAG_USER + 11; // Reply port
  ASOMSG_Length       = TAG_USER + 12; // Message length
  ASOMSG_Name         = TAG_USER + 13; // Name to put into the node
// Signal semaphore
  ASOSEM_Size         = TAG_USER + 10; // Size of the object
  ASOSEM_Name         = TAG_USER + 11; // Name
  ASOSEM_Pri          = TAG_USER + 12; // Node's priority (only used on public semaphores;
  ASOSEM_Public       = TAG_USER + 13; // Semaphore is public and will be added to the system
  ASOSEM_CopyName     = TAG_USER + 14; // Copy the name string
// TagItem array
  ASOTAGS_Size        = TAG_USER + 10; // Raw size of the object, i.e. in bytes
  ASOTAGS_NumEntries  = TAG_USER + 11; // Number of tagitems
// Memory Pool
  ASOPOOL_MFlags      = TAG_USER + 10; // Memory flags/requirements for this pool
  ASOPOOL_Puddle      = TAG_USER + 11; // Size of each puddle
  ASOPOOL_Threshold   = TAG_USER + 12; // Largest alloction size that goes into the puddle
  ASOPOOL_Protected   = TAG_USER + 13; // Protect pool with a semaphore
  ASOPOOL_Name        = TAG_USER + 14; // Name for the pool (for informational purpose only;
  ASOPOOL_CopyName    = TAG_USER + 15; // Copy the name string
  ASOPOOL_LockMem     = TAG_USER + 16; // Lock memory allocated
// Item Pool
  ASOITEM_MFlags      = TAG_USER + 10; // Memory flags for the pool
  ASOITEM_ItemSize    = TAG_USER + 11; // Size of individual items
  ASOITEM_BatchSize   = TAG_USER + 12; // Amount of items to be allocated in one batch
  ASOITEM_MaxSize     = TAG_USER + 13; // Maximum amount of items allowed in pool
  ASOITEM_GCPolicy    = TAG_USER + 14; // Garbage collection policy
  ASOITEM_GCParameter = TAG_USER + 15; // Garbage collection parameter
  ASOITEM_Constructor = TAG_USER + 16; // Constructor hook
  ASOITEM_Destructor  = TAG_USER + 17; // Destructor hook
  ASOITEM_Protected   = TAG_USER + 18; // Protect pool with a semaphore
// Mutex
  ASOMUTEX_Recursive  = TAG_USER + 10; // Make mutex recursive
// ExtMem
  ASOEXTMEM_Size            = TAG_USER + 10; // Size of extended memory area
  ASOEXTMEM_AllocationPolicy  = TAG_USER + 11; // Allocation policy

// Item pool GCPolicy types
// enItemPoolGCPolicy
  ITEMGC_NONE       = 0;
  ITEMGC_AFTERCOUNT = 1;

// Ext Memory allocation policies
// enExtMemAllocationPolicy
  EXTMEMPOLICY_IMMEDIATE  = 0;
  EXTMEMPOLICY_DELAYED    = 1;
  EXTMEMPOLICY_ACCESS     = 2;

// Kernel memory cache
  ASOKMEM_Name        = TAG_USER + 10; // Name of the cache
  ASOKMEM_Type        = TAG_USER + 11; // Type, private or shared
  ASOKMEM_Size        = TAG_USER + 12; // Object size
  ASOKMEM_Alignment   = TAG_USER + 13; // Object Alignment
  ASOKMEM_Constructor = TAG_USER + 14; // Constructor
  ASOKMEM_Destructor  = TAG_USER + 15; // Destructor
  ASOKMEM_Colored     = TAG_USER + 16; // Use cache slab coloring
  ASOKMEM_Compact     = TAG_USER + 17; // Force cache to be compact, even if this means the slab internal
                                       // fragmentation will be above the threshold
// Kernel memory cache type
// enKMemCacheType
  KMEMT_PRIVATE     = 0;
  KMEMT_SHARED      = 1;

// Resource map
  ASORMAP_Base        = TAG_USER + 10; // Resource range base
  ASORMAP_Size        = TAG_USER + 11; // Resource range size
  ASORMAP_Quantum     = TAG_USER + 12; // Minimal quantum for one single allocation

  ASORMAP_CacheMax    = TAG_USER + 13; // Maximum size for quantum caches. All allocations smaller or equal
                                       // to this size will come from quantum caches

// Tags for AllocVecTagList (V51)
  AVT_Type              = TAG_USER + 1;
  AVT_Contiguous        = TAG_USER + 2;
  AVT_Lock              = TAG_USER + 3;
  AVT_Alignment         = TAG_USER + 4;
  AVT_PhysicalAlignment = TAG_USER + 5;
  AVT_Clear             = TAG_USER + 6;
  AVT_ClearWithValue    = TAG_USER + 6;
  AVT_ClearValue        = TAG_USER + 6;
  AVT_Wait              = TAG_USER + 7;
  AVT_NoExpunge         = TAG_USER + 8;

// Tags for AllocNamedMemory (V51)
  ANMT_CheckSum       = TAG_USER + 1;
  ANMT_Error          = TAG_USER + 2;

// Possible values for ANMT_Error
// enAllocNamedMemoryErrors
  ANMERROR_NOERROR        = 0;
  ANMERROR_NOMEMORY       = 1;
  ANMERROR_DUPLICATENAME  = 2;
  ANMERROR_PARAMETER      = 3;

// Tags for GetCPUInfo
  GCIT_NumberOfCPUs   = TAG_USER +  1;
  GCIT_Family         = TAG_USER +  2;
  GCIT_Model          = TAG_USER +  3;
  GCIT_ModelString    = TAG_USER +  4;
  GCIT_Version        = TAG_USER +  5;
  GCIT_VersionString  = TAG_USER +  6;
  GCIT_FrontsideSpeed = TAG_USER +  7;
  GCIT_ProcessorSpeed = TAG_USER +  8;
  GCIT_L1CacheSize    = TAG_USER +  9;
  GCIT_L2CacheSize    = TAG_USER + 10;
  GCIT_L3CacheSize    = TAG_USER + 11;
  GCIT_VectorUnit     = TAG_USER + 12;
  GCIT_Extensions     = TAG_USER + 13;
  GCIT_CacheLineSize  = TAG_USER + 14;
  GCIT_CPUPageSize    = TAG_USER + 15;
  GCIT_ExecPageSize   = TAG_USER + 16;
  GCIT_TimeBaseSpeed  = TAG_USER + 17;

// Family codes
//enCPUFamiliy
  CPUFAMILY_UNKNOWN = 0;
  CPUFAMILY_60X     = 1;
  CPUFAMILY_7X0     = 2;
  CPUFAMILY_74XX    = 3;
  CPUFAMILY_4XX     = 4;
  CPUFAMILY_PA6T    = 5;
  CPUFAMILY_E300    = 6;
  CPUFAMILY_E5500   = 7;

// Model codes
// enCPUModel
  CPUTYPE_UNKNOWN        =  0;
  CPUTYPE_PPC603E        =  1;
  CPUTYPE_PPC604E        =  2;
  CPUTYPE_PPC750CXE      =  3;
  CPUTYPE_PPC750FX       =  4;
  CPUTYPE_PPC750GX       =  5;
  CPUTYPE_PPC7410        =  6;
  CPUTYPE_PPC74XX_VGER   =  7;
  CPUTYPE_PPC74XX_APOLLO =  8;
  CPUTYPE_PPC405LP       =  9;
  CPUTYPE_PPC405EP       = 10;
  CPUTYPE_PPC405GP       = 11;
  CPUTYPE_PPC405GPR      = 12;
  CPUTYPE_PPC440EP       = 13;
  CPUTYPE_PPC440GP       = 14;
  CPUTYPE_PPC440GX       = 15;
  CPUTYPE_PPC440SX       = 16;
  CPUTYPE_PPC440SP       = 17;
  CPUTYPE_PA6T_1682M     = 18;
  CPUTYPE_PPC460EX       = 19;
  CPUTYPE_PPC5121E     = 20;
  CPUTYPE_P50XX      = 21;

// Vector unit types
// enVectorUnitType
  VECTORTYPE_NONE    = 0;
  VECTORTYPE_ALTIVEC = 1;
  // Same as AltiVec    VECTORTYPE_VMX     = 2

//**********************************************************************
// Object types for AllocSysObject

// enAllocSysObjectTypes
  ASOT_IOREQUEST =  0; // IORequest
  ASOT_HOOK      =  1; // Hook
  ASOT_INTERRUPT =  2; // Interrupt structure
  ASOT_LIST      =  3; // List and MinList
  ASOT_DMAENTRY  =  4; // DMAEntry array
  ASOT_NODE      =  5; // List node and MinNode
  ASOT_PORT      =  6; // Message port
  ASOT_MESSAGE   =  7; // Exec Message
  ASOT_SEMAPHORE =  8; // Signal Semaphore
  ASOT_TAGLIST   =  9; // TagItem list
  ASOT_MEMPOOL   = 10; // Memory pool
  ASOT_ITEMPOOL  = 11; // Item pool
  ASOT_RMAP      = 12; // Resource map
  ASOT_MUTEX     = 13; // Mutex
  ASOT_EXTMEM    = 14; // Extended memory

//**********************************************************************
// Tag items for CreateLibrary
  CLT_Vector68K     = TAG_USER +  1;
  CLT_InitData      = TAG_USER +  2;
  CLT_InitFunc      = TAG_USER +  3;
  CLT_Seglist       = TAG_USER +  8;
  CLT_Interfaces    = TAG_USER +  9;
  CLT_DataSize      = TAG_USER + 10;
  CLT_Legacy        = TAG_USER + 11;
  CLT_NoLegacyIFace = TAG_USER + 12;

//**********************************************************************
// Message types for debugger hook
// enDebugMessage
  DBHMT_EXCEPTION     = 1;
  DBHMT_REMTASK       = 2;
  DBHMT_OPENLIB       = 3;
  DBHMT_CLOSELIB      = 4;
  DBHMT_ADDTASK       = 5;
  DBHMT_SHAREDOBJECTOPEN  = 6;
  DBHMT_SHAREDOBJECTCLOSE = 7;

//**********************************************************************
// Tags for AddTask/CreateTask
  AT_Param1         = TAG_USER + 1;
  AT_Param2         = TAG_USER + 2;
  AT_Param3         = TAG_USER + 3;
  AT_Param4         = TAG_USER + 4;
  AT_Param5         = TAG_USER + 5;
  AT_Param6         = TAG_USER + 6;
  AT_Param7         = TAG_USER + 7;
  AT_Param8         = TAG_USER + 8;
  AT_Child          = TAG_USER + 9;

  CT_LockStack      = TAG_USER + 20;

//**********************************************************************
// Tags for NewStackRun
  NSR_Dummy         = TAG_USER + 500;    // Offset to avoid Emulate(; tag collision.
  NSR_StackSize     = NSR_Dummy + 1;     // Initial stack size
  NSR_MinStackSize  = NSR_Dummy + 2;     // Absolute minimal stack size
  NSR_LockStack     = NSR_Dummy + 3;     // Lock the stack in memory to prevent paging
  NSR_Arg1          = NSR_Dummy + 10;    // Parameters passed to the function
  NSR_Arg2          = NSR_Dummy + 11;
  NSR_Arg3          = NSR_Dummy + 12;
  NSR_Arg4          = NSR_Dummy + 13;
  NSR_Arg5          = NSR_Dummy + 14;
  NSR_Arg6          = NSR_Dummy + 15;

//**********************************************************************

var
  ExecBase: PExecBase absolute AOS_ExecBase;

function ExecObtain(): LongWord; syscall IExec 60;
function ExecRelease(): LongWord; syscall IExec 64;
procedure ExecExpunge(); syscall IExec 68;
function ExecClone(): PInterface; syscall IExec 72;
procedure AddHead(List: PList; Node: PNode); syscall IExec 76;
procedure AddMemHandler(MemHand: PInterrupt); syscall IExec 80;
procedure AddMemList(Size: LongWord; Attributes: LongWord; Pri: LongInt; Base: APTR; const Name: PChar); syscall IExec 84;
procedure AddTail(List: PList; Node: PNode); syscall IExec 88;
function AllocAbs(ByteSize: LongWord; Location: APTR): APTR; syscall IExec 92;
function Allocate(FreeList: PMemHeader; ByteSize: LongWord): APTR; syscall IExec 96;
function AllocEntry(Entry: PMemList): PMemList; syscall IExec 100;
function ExecAllocMem(ByteSize: LongWord; Requirements: LongWord): APTR; syscall IExec 104;
function AllocPooled(PoolHeader: APTR; MemSize: LongWord): APTR; syscall IExec 108;
function AllocVec(ByteSize: LongWord; Requirements: LongWord): APTR; syscall IExec 112;
function AllocVecPooled(PoolHeader: APTR; Size: LongWord): APTR; syscall IExec 116;
function AvailMem(Requirements: LongWord): LongWord; syscall IExec 120;
procedure CopyMem(const Source: APTR; Dest: APTR; Size: LongWord); syscall IExec 124;
procedure CopyMemQuick(const Source: APTR; Dest: APTR; Size: LongWord); syscall IExec 128;
function CreatePool(MemFlags: LongWord; PuddleSize: LongWord; ThreshSize: LongWord): APTR; syscall IExec 132;
procedure Deallocate(MemHeader: PMemHeader; MemoryBlock: APTR; ByteSize: LongWord); syscall IExec 136;
procedure DeletePool(PoolHeader: APTR); syscall IExec 140;
procedure Enqueue(List: PList; Node: PNode); syscall IExec 144;
function FindName(Start: PList; const Name: PChar): PNode; syscall IExec 148;
function FindIName(Start: PList; const Name: PChar): PNode; syscall IExec 152;
procedure Forbid(); syscall IExec 156;
procedure FreeEntry(MemList: PMemList); syscall IExec 160;
procedure ExecFreeMem(MemoryBlock: APTR; ByteSize: LongWord); syscall IExec 164;
procedure FreePooled(PoolHeader: APTR; Memory: APTR; MemSize: LongWord); syscall IExec 168;
procedure FreeVec(MemoryBlock: APTR); syscall IExec 172;
procedure FreeVecPooled(PoolHeader: APTR; MemoryBlock: APTR); syscall IExec 176;
procedure InitData(const InitTab: APTR; Memory: APTR; Size: LongWord); syscall IExec 180;
procedure InitStruct(const InitTab: APTR; Memory: APTR; Size: LongWord); syscall IExec 184;
procedure ExecInsert(List: PList; Node: PNode; ListNode: PNode); syscall IExec 188;
function MakeInterface(Lib: PLibrary; const TagList: PTagItem): PInterface; syscall IExec 192;
// 196 MakeInterfaceTags varargs version of MakeInterface
procedure Permit(); syscall IExec 200;
function RawDoFmt(const FormatString: STRPTR; const DataStream: APTR; putChProc: TProcedure; PutChData: APTR): APTR; syscall IExec 204;
function RemHead(List: PList): PNode; syscall IExec 208;
procedure RemMemHandler(MemHandler: PInterrupt); syscall IExec 212;
procedure Remove(Node: PNode); syscall IExec 216;
function RemTail(List: PList): PNode; syscall IExec 220;
function TypeOfMem(const address: APTR): LongWord; syscall IExec 224;
function InitResident(const Resident_: PResident; SegList: LongWord): APTR; syscall IExec 228;
procedure InitCode(StartClass: LongWord; Version: LongWord); syscall IExec 232;
function SumKickData(): LongWord; syscall IExec 236;
function AddTask(Task: PTask; const InitPC: APTR; const FinalPC: APTR): APTR; syscall IExec 240;
// 244 AddTaskTags varargs version of AddTask
procedure Disable(); syscall IExec 248;
procedure Enable(); syscall IExec 252;
procedure Reschedule(); syscall IExec 256;
function FindTask(const Name: STRPTR): PTask; syscall IExec 260;
procedure RemTask(Task: PTask); syscall IExec 264;
function SetTaskPri(Task: PTask; Priority: LongInt): LongInt; syscall IExec 268;
procedure StackSwap(NewStack: PStackSwapStruct); syscall IExec 272;
function AllocSignal(SignalNum: LongInt): ShortInt; syscall IExec 276;
procedure FreeSignal(SignalNum: LongInt); syscall IExec 280;
function SetExcept(NewSignals: LongWord; SignalMask: LongWord): LongWord; syscall IExec 284;
function SetSignal(NewSignals: LongWord; SignalMask: LongWord): LongWord; syscall IExec 288;
procedure Signal(Task: PTask; Signals: LongWord); syscall IExec 292;
function Wait(SignalSet: LongWord): LongWord; syscall IExec 296;
procedure AddPort(Port: PMsgPort); syscall IExec 300;
function CreatePort(const Name: STRPTR; Pri: LongInt): PMsgPort; syscall IExec 304;
function CreateMsgPort(): PMsgPort; syscall IExec 308;
procedure DeletePort(Port: PMsgPort); syscall IExec 312;
procedure DeleteMsgPort(Port: PMsgPort); syscall IExec 316;
function FindPort(const Name: STRPTR): PMsgPort; syscall IExec 320;
function GetMsg(Port: PMsgPort): PMessage; syscall IExec 324;
procedure PutMsg(Port: PMsgPort; Message: PMessage); syscall IExec 328;
procedure RemPort(Port: PMsgPort); syscall IExec 332;
procedure ReplyMsg(Message: PMessage); syscall IExec 336;
function WaitPort(Port: PMsgPort): PMessage; syscall IExec 340;
procedure Cause(Interrupt_: PInterrupt); syscall IExec 344;
procedure AddSemaphore(SigSem: PSignalSemaphore); syscall IExec 348;
function AttemptSemaphore(SigSem: PSignalSemaphore): LongBool; syscall IExec 352;
function AttemptSemaphoreShared(SigSem: PSignalSemaphore): LongBool; syscall IExec 356;
function FindSemaphore(const Name: STRPTR): PSignalSemaphore; syscall IExec 360;
procedure InitSemaphore(SigSem: PSignalSemaphore); syscall IExec 364;
procedure ObtainSemaphore(SigSem: PSignalSemaphore); syscall IExec 368;
procedure ObtainSemaphoreList(SigSem: PList); syscall IExec 372;
procedure ObtainSemaphoreShared(SigSem: PSignalSemaphore); syscall IExec 376;
function Procure(SigSem: PSignalSemaphore; BidMessage: PSemaphoreMessage): LongBool; syscall IExec 380;
procedure ReleaseSemaphore(SigSem: PSignalSemaphore); syscall IExec 384;
procedure ReleaseSemaphoreList(SigSem: PList); syscall IExec 388;
procedure RemSemaphore(SigSem: PSignalSemaphore); syscall IExec 392;
procedure Vacate(SigSem: PSignalSemaphore; BidMessage: PSemaphoreMessage); syscall IExec 396;
function CreateTask(const Name: STRPTR; Pri: LongInt; const InitPC: APTR; StackSize: LongWord; const TagList: PTagItem): PTask; syscall IExec 400;
// 404 CreateTaskTags varargs version of CreateTask
procedure DeleteTask(Task: PTask); syscall IExec 408;
procedure SumLibrary(Lib: PLibrary); syscall IExec 412;
function CreateLibrary(const TagList: PTagItem): PLibrary; syscall IExec 416;
// 420 CreateLibraryTags varargs version of CreateLibrary
function OpenLibrary(const Name: STRPTR; Version: LongWord): PLibrary; syscall IExec 424;
function CloseLibrary(Lib: PLibrary): APTR; syscall IExec 428;
procedure AddLibrary(Lib: PLibrary); syscall IExec 432;
function RemLibrary(Lib: PLibrary): LongWord; syscall IExec 436;
procedure AddDevice(Device: PDevice); syscall IExec 440;
function RemDevice(Device: PDevice): LongWord; syscall IExec 444;
function GetInterface(Lib: PLibrary; const Name: STRPTR; Version: LongWord; const TagList: PTagItem): PInterface; syscall IExec 448;
// 452 GetInterfaceTags varargs version of GetInterface
procedure DropInterface(Interface_: PInterface); syscall IExec 456;
procedure AddInterface(Lib: PLibrary; Interface_: PInterface); syscall IExec 460;
procedure RemInterface(Interface_: PInterface); syscall IExec 464;
procedure SumInterface(Interface_: PInterface); syscall IExec 468;
function FindResident(const Name: STRPTR): PResident; syscall IExec 472;
function SetMethod(Interface_: PInterface; FuncOffset: LongInt; const NewFunc: APTR): APTR; syscall IExec 476;
procedure DeleteInterface(Interface_: PInterface); syscall IExec 480;
procedure DeleteLibrary(Lib: PLibrary); syscall IExec 484;
function Setfunction(Lib: PLibrary; FuncOffset: LongInt; const Newfunction: APTR): APTR; syscall IExec 488;
procedure CacheClearE(Address: APTR; Length: LongWord; Caches: LongWord); syscall IExec 492;
procedure CacheClearU(); syscall IExec 496;
function Makefunctions(Target: APTR; const FunctionArray: APTR; const FuncDispBase: APTR): LongWord; syscall IExec 500;
function OpenDevice(const DevName: STRPTR; UnitNumber: LongWord; IORequest: PIORequest; Flags: LongWord): LongInt; syscall IExec 504;
procedure CloseDevice(IORequest: PIORequest); syscall IExec 508;
function CreateIORequest(const IOReplyPort: PMsgPort; Size: LongWord): PIORequest; syscall IExec 512;
procedure DeleteIORequest(IORequest: PIORequest); syscall IExec 516;
procedure AbortIO(IORequest: PIORequest); syscall IExec 520;
function CheckIO(IORequest: PIORequest): PIORequest; syscall IExec 524;
function DoIO(IORequest: PIORequest): LongInt; syscall IExec 528;
procedure SendIO(IORequest: PIORequest); syscall IExec 532;
procedure BeginIO(IORequest: PIORequest); syscall IExec 536;
procedure WaitIO(IORequest: PIORequest); syscall IExec 540;
procedure AddResource(Resource: APTR); syscall IExec 544;
procedure RemResource(Resource: APTR); syscall IExec 548;
function OpenResource(const ResName: STRPTR): APTR; syscall IExec 552;
procedure AddIntServer(IntNumber: LongWord; Interrupt_: PInterrupt); syscall IExec 556;
procedure RemIntServer(IntNumber: LongWord; Interrupt_: PInterrupt); syscall IExec 560;
function SetIntVector(IntNumber: LongWord;const Interrupt_: PInterrupt): PInterrupt; syscall IExec 564;
function ObtainQuickVector(InterruptCode: APTR): LongWord; syscall IExec 568;
procedure Alert(AlertNum: LongWord); syscall IExec 572;
function SuperState(): APTR; syscall IExec 576;
procedure UserState(SysStack: APTR); syscall IExec 580;
function Supervisor(UserFunc: APTR): LongWord; syscall IExec 584;
function SetTaskTrap(TrapNum: LongWord; const TrapCode: APTR; const TrapData: APTR): LongBool; syscall IExec 588;
function AllocTrap(TrapNum: LongWord): LongInt; syscall IExec 592;
procedure FreeTrap(TrapNum: LongWord); syscall IExec 596;
function GetCC: LongWord; syscall IExec 600;
function SetSR(NewSR: LongWord; Mask: LongWord): LongWord; syscall IExec 604;
function AVL_AddNode(Root: PPAVLNode; Node: PAVLNode; Func: Pointer): PAVLNode; syscall IExec 608;
function AVL_FindFirstNode(const Root: PAVLNode): PAVLNode; syscall IExec 612;
function AVL_FindLastNode(const Root: PAVLNode): PAVLNode; syscall IExec 616;
function AVL_FindNextNodeByAddress(const Node: PAVLNode): PAVLNode; syscall IExec 620;
function AVL_FindNextNodeByKey(const Root: PAVLNode; Key: APTR; Func: APTR): PAVLNode; syscall IExec 624;
function AVL_FindNode(const Root: PAVLNode; Key: APTR; Func: APTR): PAVLNode; syscall IExec 628;
function AVL_FindPrevNodeByAddress(const Root: PAVLNode): PAVLNode; syscall IExec 632;
function AVL_FindPrevNodeByKey(const Root: PAVLNode; Key: APTR; Func: APTR): PAVLNode; syscall IExec 636;
function AVL_RemNodeByAddress(Root: PPAVLNode; Node: PAVLNode): PAVLNode; syscall IExec 640;
function AVL_RemNodeByKey(Root: PPAVLNode; Key: APTR; Func: APTR): PAVLNode; syscall IExec 644;
function CacheControl(CacheBits: LongWord; CacheMask: LongWord): LongWord; syscall IExec 648;
function LockMem(BaseAddress: APTR; Size: LongWord): LongBool; syscall IExec 652;
procedure UnLockMem(BaseAddress: APTR; Size: LongWord); syscall IExec 656;
function ReallocVec(MemBlock: APTR; NewSize: LongWord; Flags: LongWord): LongWord; syscall IExec 660;
function CachePreDMA(const VAddr: APTR; var Length: LongWord; Flags: LongWord): APTR; syscall IExec 664;
procedure CachePostDMA(const VAddr: APTR; var Length: LongWord; Flags: LongWord); syscall IExec 668;
function StartDMA(const StartAddr: APTR; BlockSize: LongWord; Flags: LongWord): LongWord; syscall IExec 672;
function EndDMA(const StartAddr: APTR; BlockSize: LongWord; Flags: LongWord): LongWord; syscall IExec 676;
procedure GetDMAList(const StartAddr: APTR; BlockSize: LongWord; Flags: LongWord; DMAList: Pointer); syscall IExec 680;
function AddTrackable(UsingTask: PTask; Obj: APTR; DestFunc: PHook): PTrackable; syscall IExec 684;
function FindTrackable(UsingTask: PTask; Obj: APTR): PTrackable; syscall IExec 688;
function RemTrackable(UsingTask: PTask; Obj: APTR; Trackable: PTrackable): PTrackable; syscall IExec 692;
procedure DeleteTrackable(Trackable: PTrackable); syscall IExec 696;
function AllocSysObject(Type_: LongWord; const Tags: PTagItem): APTR; syscall IExec 700;
// 704 AllocSysObjectTags
procedure FreeSysObject(Type_: LongWord; Obj: APTR); syscall IExec 708;
procedure SuspendTask(WhichTask: PTask; Flags: LongWord); syscall IExec 712;
procedure RestartTask(WhichTask: PTask; Flags: LongWord); syscall IExec 716;
procedure MoveList(DestinationList: PList; SourceList: PList); syscall IExec 720;
procedure NewList(List: PList); syscall IExec 724;
procedure NewMinList(List: PMinList); syscall IExec 728;
procedure ColdReboot(); syscall IExec 732;
function MakeLibrary(const Vectors: APTR;const Structure: APTR; const Init: APTR; DataSize: LongWord; SegList: APTR): PLibrary; syscall IExec 736;
function Emulate(const InitPC: APTR; const TagList: PTagItem): LongWord; syscall IExec 740;
// 744 EmulateTags
// 748 DebugPrintF
function IsNative(const Code: APTR): LongBool; syscall IExec 752;
function RawMayGetChar(): LongInt;syscall IExec 756;
procedure RawPutChar(c: Char); syscall IExec 760;
procedure GetCPUInfo(const TagList: PTagItem); syscall IExec 764;
// 768 GetCPUInfoTags
function OwnerOfMem(const Address: APTR): PTask; syscall IExec 772;
function AddResetCallback(ResetCallBack: PInterrupt): LongBool; syscall IExec 776;
function RemResetCallback(ResetCallBack: PInterrupt): LongBool; syscall IExec 780;
function ItemPoolAlloc(ItemPool: APTR): APTR; syscall IExec 784;
procedure ItemPoolFree(ItemPool: APTR); syscall IExec 788;
procedure ItemPoolGC(ItemPool: APTR); syscall IExec 792;
function ItemPoolControl(ItemPool: APTR; const TagList: PTagItem): LongWord; syscall IExec 796;
// 800 ItemPoolControlTags
procedure ItemPoolFlush(ItemPool: APTR); syscall IExec 804;
function GetHead(List: PList): PNode; syscall IExec 808;
function GetTail(List: PList): PNode; syscall IExec 812;
function GetSucc(Node: PNode): PNode; syscall IExec 816;
function GetPred(Node: PNode): PNode; syscall IExec 820;
procedure IceColdReboot(); syscall IExec 824;
function KMemCacheCreate(VMA: APTR; const Name: STRPTR; Size: LongWord; Align: LongWord; const Constructor_: APTR; const Destructor_: APTR; Flag: LongWord; UserData: APTR): APTR; syscall IExec 828;
procedure KMemCacheDestroy(Cache: APTR); syscall IExec 832;
function KMemCacheAlloc(Cache: APTR; Flags: LongWord): APTR; syscall IExec 836;
procedure KMemCacheFree(Cache: APTR; Obj: APTR); syscall IExec 840;
procedure KMemCacheGrow(Cache: APTR; Flags: LongWord); syscall IExec 840;
function KMemCacheReap(Cache: APTR; NumPages: LongWord; Flags: LongWord): LongWord; syscall IExec 848;
function KMemCacheFind(const Name: STRPTR): APTR; syscall IExec 852;
function PageGet(VMA: APTR; NumPages: LongWord; Alignment: LongWord): APTR; syscall IExec 856;
function PageMap(PageList: APTR; NumPages: LongWord; Attrs: LongWord; Flags: LongWord): LongBool; syscall IExec 860;
procedure PageUnmap(PageList: APTR; NumPages: LongWord); syscall IExec 864;
procedure PageDrop(PageList: APTR; NumPages: LongWord); syscall IExec 868;
function PageFindByVA(AddressSpace: APTR; VirtualAddress: APTR): APTR; syscall IExec 872;
procedure PageHash(Page: APTR); syscall IExec 876;
procedure PageUnhash(Page: APTR); syscall IExec 880;
function PageBackendAlloc(Order: LongWord; Flags: LongWord): APTR; syscall IExec 884;
procedure PageBackendFree(Addr: APTR; Order: LongWord); syscall IExec 888;
procedure PageBackendLock(); syscall IExec 892;
procedure PageBackendUnlock(); syscall IExec 896;
// 900
// 904
// 908
function RMapAlloc(Map: APTR; Size: LongWord; Flags: LongWord): APTR; syscall IExec 912;
procedure RMapFree(Map: APTR; Addr: APTR; Size: LongWord); syscall IExec 916;
function VMAreaInit(AddrSpace: APTR; Shared: LongBool; Area: APTR; const Tags: PTagItem): LongBool; syscall IExec 920;
// 924 VMAreaInitTags
procedure VMAreaTerm(Area: APTR); syscall IExec 928;
function AllocVecTagList(Size: LongWord; const Tags: PTagItem): APTR; syscall IExec 932;
// 936 AllocVecTags
function PageMapAligned(PageList: Pointer; NumPages: LongWord; Alignment: LongWord; Attrs: LongWord; Flags: LongWord): LongBool; syscall IExec 940;
function RMapExtAlloc(Map: APTR; Size: LongWord; Alignment: LongWord; Flags: LongWord): APTR; syscall IExec 944;
procedure RMapExtFree(Map: APTR; Addr: APTR; Size: LongWord); syscall IExec 948;
procedure PageBackendAllocAligned(Order: LongWord; Alignment: LongWord; Flags: LongWord); syscall IExec 952;
function AllocNamedMemory(ByteSize: LongWord; const Space: STRPTR; const Name: STRPTR; TagList: PTagITem): APTR; syscall IExec 956;
// 960 AllocNamedMemoryTags
function FreeNamedMemory(const Space: STRPTR; const Name: STRPTR): LongBool; syscall IExec 964;
function FindNamedMemory(const Space: STRPTR; const Name: STRPTR): Pointer; syscall IExec 968;
procedure UpdateNamedMemory(const Space: STRPTR; const Name: STRPTR); syscall IExec 972;
function LockNamedMemory(const Space: STRPTR; const Name: STRPTR): Pointer; syscall IExec 976;
function AttemptNamedMemory(const Space: STRPTR; const Name: STRPTR): Pointer; syscall IExec 980;
procedure UnlockNamedMemory(const Space: STRPTR; const Name: STRPTR); syscall IExec 984;
function ScanNamedMemory(SCHook: PHook; Flags: LongWord; User: APTR): LongWord; syscall IExec 988;
function AllocTaskMemEntry(MemList: PMemList): PMemList; syscall IExec 992;
function PagerFindPageOut(NFlags: LongWord): APTR; syscall IExec 996;
function PagerPageIn(PPage: APTR; NFlags: LongWord): LongBool; syscall IExec 1000;
function PagerPageOut(PPage: APTR; NFlags: LongWord; PReceiver: APTR): LongBool; syscall IExec 1004;
function PagerAssignPager(PPage: APTR; NFlags: LongWord): LongBool; syscall IExec 1008;
procedure MutexObtain(Mutex: APTR); syscall IExec 1012;
function MutexAttempt(Mutex: APTR): LongBool; syscall IExec 1016;
procedure MutexRelease(Mutex: APTR); syscall IExec 1020;
function MutexAttemptWithSignal(Mutex: APTR; SigSet: LongWord): LongWord; syscall IExec 1024;
function NewStackRun(InitPC: APTR; const TagList: PTagItem): LongInt; syscall IExec 1028;
// 1032 NewStackRunTags

function BitMask(No: ShortInt): LongInt; inline;
function IsListEmpty(List: PList): Boolean; inline;
function IsMinListEmpty(List: PMinList): Boolean; inline;
function IsMsgPortEmpty(mp: PMsgPort): Boolean; inline;
procedure NewListType(var List: PList; NType: Byte); inline;

function CreateExtIO(const Mp: PMsgPort; Size: Integer): PIORequest;
procedure DeleteExtIO(ioReq: PIORequest);

implementation

function BitMask(No: ShortInt): LongInt; inline;
begin
  BitMask := 1 shl No;
end;

function IsListEmpty(List: PList): Boolean; inline;
begin
  IsListEmpty := List^.lh_TailPred = PNode(List);
end;

function IsMinListEmpty(List: PMinList): Boolean; inline;
begin
  IsMinListEmpty := List^.mlh_TailPred = PMinNode(List);
end;

function IsMsgPortEmpty(Mp: PMsgPort): Boolean; inline;
begin
  with Mp^ do
    IsMsgPortEmpty := mp_MsgList.lh_TailPred = PNode(@mp_MsgList);
end;

procedure NewListType(var List: PList; NType: Byte); inline;
begin
  NewList(List);
  List^.lh_Type := NType;
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
