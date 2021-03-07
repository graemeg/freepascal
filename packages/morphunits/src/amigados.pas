{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    dos.library interface unit for MorphOS/PowerPC

    MorphOS port was done on a free Pegasos II/G4 machine
    provided by Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$INLINE ON}
{$PACKRECORDS 2}

unit AmigaDOS;

interface

uses Exec, Timer;

var
  DosBase: Pointer;


{ * dos global definitions (V50)
  *********************************************************************
  * }


const
  DOSNAME = 'dos.library';

const
  DOSTRUE  = -1;
  DOSFALSE =  0;

const
  MODE_OLDFILE   = 1005;
  MODE_NEWFILE   = 1006;
  MODE_READWRITE = 1004;

const
  OFFSET_BEGINNING = -1;
  OFFSET_CURRENT   =  0;
  OFFSET_END       =  1;
  OFFSET_BEGINING  =  OFFSET_BEGINNING; { * Typo fix * }

const
  BITSPERBYTE  = 8;
  BYTESPERLONG = 4;
  BITSPERLONG  = 32;
  _MAXINT      = $7FFFFFFF;
  _MININT      = $80000000;

const
  SHARED_LOCK    = -2;
  ACCESS_READ    = -2;
  EXCLUSIVE_LOCK = -1;
  ACCESS_WRITE   = -1;


type
  PDateStamp = ^TDateStamp;
  TDateStamp = packed record
    ds_Days  : LongInt;
    ds_Minute: LongInt;
    ds_Tick  : LongInt;
  end;

const
  TICKS_PER_SECOND = 50;


type
  PFileInfoBlock = ^TFileInfoBlock;
  TFileInfoBlock = packed record
    fib_DiskKey     : LongInt;
    fib_DirEntryType: LongInt;
    fib_FileName    : Array[0..107] Of Char;
    fib_Protection  : LongInt;
    fib_EntryType   : LongInt;
    fib_Size        : LongInt;
    fib_NumBlocks   : LongInt;
    fib_Date        : TDateStamp;
    fib_Comment     : Array[0..79] Of Char;

    fib_OwnerUID    : Word;
    fib_OwnerGID    : Word;
    case SmallInt of
      0: (
        fib_Reserved    : Array[0..31] Of Char;
        );
      1: (
        // 64bit DOS extensions V51 filled by Examine64, ExNext64, ExamineFH64 with ACTION_EXAMINE_OBJECT64
        fib_Size64: QWord;
        fib_NumBlocks64: QWord;
      );
  end;

const
  FIBB_OTR_READ    = 15;
  FIBB_OTR_WRITE   = 14;
  FIBB_OTR_EXECUTE = 13;
  FIBB_OTR_DELETE  = 12;
  FIBB_GRP_READ    = 11;
  FIBB_GRP_WRITE   = 10;
  FIBB_GRP_EXECUTE = 9;
  FIBB_GRP_DELETE  = 8;

  FIBB_SCRIPT      = 6;
  FIBB_PURE        = 5;
  FIBB_ARCHIVE     = 4;
  FIBB_READ        = 3;
  FIBB_WRITE       = 2;
  FIBB_EXECUTE     = 1;
  FIBB_DELETE      = 0;

const
  FIBF_OTR_READ    = 1 Shl FIBB_OTR_READ;
  FIBF_OTR_WRITE   = 1 Shl FIBB_OTR_WRITE;
  FIBF_OTR_EXECUTE = 1 Shl FIBB_OTR_EXECUTE;
  FIBF_OTR_DELETE  = 1 Shl FIBB_OTR_DELETE;
  FIBF_GRP_READ    = 1 Shl FIBB_GRP_READ;
  FIBF_GRP_WRITE   = 1 Shl FIBB_GRP_WRITE;
  FIBF_GRP_EXECUTE = 1 Shl FIBB_GRP_EXECUTE;
  FIBF_GRP_DELETE  = 1 Shl FIBB_GRP_DELETE;

  FIBF_SCRIPT      = 1 Shl FIBB_SCRIPT;
  FIBF_PURE        = 1 Shl FIBB_PURE;
  FIBF_ARCHIVE     = 1 Shl FIBB_ARCHIVE;
  FIBF_READ        = 1 Shl FIBB_READ;
  FIBF_WRITE       = 1 Shl FIBB_WRITE;
  FIBF_EXECUTE     = 1 Shl FIBB_EXECUTE;
  FIBF_DELETE      = 1 Shl FIBB_DELETE;

const
  FAULT_MAX = 82;

type
  BPTR = LongInt;
  BSTR = LongInt;

type
  PInfoData = ^TInfoData;
  TInfoData = packed record
    id_NumSoftErrors: LongInt;
    id_UnitNumber   : LongInt;
    id_DiskState    : LongInt;
    id_NumBlocks    : LongInt;
    id_NumBlocksUsed: LongInt;
    id_BytesPerBlock: LongInt;
    id_DiskType     : LongInt;
    id_VolumeNode   : BPTR;
    id_InUse        : LongInt;
  end;


const
  ID_WRITE_PROTECTED = 80;
  ID_VALIDATING      = 81;
  ID_VALIDATED       = 82;

  ID_NO_DISK_PRESENT  = -1;
  ID_UNREADABLE_DISK  = $42414400;
  ID_DOS_DISK         = $444F5300;
  ID_FFS_DISK         = $444F5301;
  ID_INTER_DOS_DISK   = $444F5302;
  ID_INTER_FFS_DISK   = $444F5303;
  ID_FASTDIR_DOS_DISK = $444F5304;
  ID_FASTDIR_FFS_DISK = $444F5305;
  ID_LNFS_DOS_DISK    = $444F5306;
  ID_LNFS_FFS_DISK    = $444F5307;
  ID_NOT_REALLY_DOS   = $4E444F53;
  ID_KICKSTART_DISK   = $4B49434B;
  ID_MSDOS_DISK       = $4d534400;
  ID_CDFS_DISK        = $43444653; // CDFS - Built-in CDROM
  ID_CDROM_ISO_DISK   = $43443031; // CD01
  ID_CDROM_HSF_DISK   = $43443030; // CD00
  ID_CDROM_CDDA_DISK  = $43444441; // CDDA
  ID_SFS_DISK         = $53465300; // SFS#0

const
  ERROR_NO_FREE_STORE            = 103;
  ERROR_TASK_TABLE_FULL          = 105;
  ERROR_BAD_TEMPLATE             = 114;
  ERROR_BAD_NUMBER               = 115;
  ERROR_REQUIRED_ARG_MISSING     = 116;
  ERROR_KEY_NEEDS_ARG            = 117;
  ERROR_TOO_MANY_ARGS            = 118;
  ERROR_UNMATCHED_QUOTES         = 119;
  ERROR_LINE_TOO_LONG            = 120;
  ERROR_FILE_NOT_OBJECT          = 121;
  ERROR_INVALID_RESIDENT_LIBRARY = 122;
  ERROR_NO_DEFAULT_DIR           = 201;
  ERROR_OBJECT_IN_USE            = 202;
  ERROR_OBJECT_EXISTS            = 203;
  ERROR_DIR_NOT_FOUND            = 204;
  ERROR_OBJECT_NOT_FOUND         = 205;
  ERROR_BAD_STREAM_NAME          = 206;
  ERROR_OBJECT_TOO_LARGE         = 207;
  ERROR_ACTION_NOT_KNOWN         = 209;
  ERROR_INVALID_COMPONENT_NAME   = 210;
  ERROR_INVALID_LOCK             = 211;
  ERROR_OBJECT_WRONG_TYPE        = 212;
  ERROR_DISK_NOT_VALIDATED       = 213;
  ERROR_DISK_WRITE_PROTECTED     = 214;
  ERROR_RENAME_ACROSS_DEVICES    = 215;
  ERROR_DIRECTORY_NOT_EMPTY      = 216;
  ERROR_TOO_MANY_LEVELS          = 217;
  ERROR_DEVICE_NOT_MOUNTED       = 218;
  ERROR_SEEK_ERROR               = 219;
  ERROR_COMMENT_TOO_BIG          = 220;
  ERROR_DISK_FULL                = 221;
  ERROR_DELETE_PROTECTED         = 222;
  ERROR_WRITE_PROTECTED          = 223;
  ERROR_READ_PROTECTED           = 224;
  ERROR_NOT_A_DOS_DISK           = 225;
  ERROR_NO_DISK                  = 226;
  ERROR_NO_MORE_ENTRIES          = 232;

  ERROR_IS_SOFT_LINK             = 233;
  ERROR_OBJECT_LINKED            = 234;
  ERROR_BAD_HUNK                 = 235;
  ERROR_NOT_IMPLEMENTED          = 236;
  ERROR_RECORD_NOT_LOCKED        = 240;
  ERROR_LOCK_COLLISION           = 241;
  ERROR_LOCK_TIMEOUT             = 242;
  ERROR_UNLOCK_ERROR             = 243;

const
  RETURN_OK    = 0;
  RETURN_WARN  = 5;
  RETURN_ERROR = 10;
  RETURN_FAIL  = 20;

const
  SIGBREAKB_CTRL_C = 12;
  SIGBREAKB_CTRL_D = 13;
  SIGBREAKB_CTRL_E = 14;
  SIGBREAKB_CTRL_F = 15;

  SIGBREAKF_CTRL_C = 1 Shl SIGBREAKB_CTRL_C;
  SIGBREAKF_CTRL_D = 1 Shl SIGBREAKB_CTRL_D;
  SIGBREAKF_CTRL_E = 1 Shl SIGBREAKB_CTRL_E;
  SIGBREAKF_CTRL_F = 1 Shl SIGBREAKB_CTRL_F;

const
  LOCK_DIFFERENT    = -1;
  LOCK_SAME         =  0;
  LOCK_SAME_VOLUME  =  1;
  LOCK_SAME_HANDLER =  LOCK_SAME_VOLUME;

const
  CHANGE_LOCK = 0;
  CHANGE_FH   = 1;

const
  LINK_HARD  = 0;
  LINK_SOFT  = 1;

const
  ITEM_EQUAL    = -2;
  ITEM_ERROR    = -1;
  ITEM_NOTHING  =  0;
  ITEM_UNQUOTED =  1;
  ITEM_QUOTED   =  2;

// AllocDosObject(), FreeDosObject() Types
const
  DOS_FILEHANDLE   = 0;
  DOS_EXALLCONTROL = 1;
  DOS_FIB          = 2;
  DOS_STDPKT       = 3;
  DOS_CLI          = 4;
  DOS_RDARGS       = 5;
  // V51
  DOS_DEVICENODE   = 6;
  DOS_FSCONTEXT    = 7;
  DOS_VOLUMENODE   = 8;
  DOS_ASSIGNNODE   = 9;

{ * dos date/time definitions
  *********************************************************************
  * }


type
  { * Required to avoid conflict with default types * }
  _PDateTime = ^_TDateTime;
  _TDateTime = packed record
    dat_Stamp  : TDateStamp;
    dat_Format : Byte;
    dat_Flags  : Byte;
    dat_StrDay : PChar;
    dat_StrDate: PChar;
    dat_StrTime: PChar;
  end;

const
  LEN_DATSTRING  = 16;

const
  DTB_SUBST  = 0;
  DTF_SUBST  = 1 Shl DTB_SUBST;
  DTB_FUTURE = 1;
  DTF_FUTURE = 1 Shl DTB_FUTURE;

const
  FORMAT_DOS = 0;
  FORMAT_INT = 1;
  FORMAT_USA = 2;
  FORMAT_CDN = 3;
  FORMAT_MAX = FORMAT_CDN;
  FORMAT_DEF = 4;



{ * dos extended structures definitions
  *********************************************************************
  * }


type
  PProcess = ^TProcess;
  TProcess = packed record
    pr_Task          : TTask;
    pr_MsgPort       : TMsgPort;
    pr_Pad           : Word;
    pr_SegList       : BPTR;
    pr_StackSize     : LongInt;  { 68k stacksize! }
    pr_GlobVec       : APTR;
    pr_TaskNum       : LongInt;
    pr_StackBase     : BPTR;
    pr_Result2       : LongInt;
    pr_CurrentDir    : BPTR;
    pr_CIS           : BPTR;
    pr_COS           : BPTR;
    pr_ConsoleTask   : APTR;
    pr_FileSystemTask: APTR;
    pr_CLI           : BPTR;
    pr_ReturnAddr    : APTR;
    pr_PktWait       : APTR;
    pr_WindowPtr     : APTR;
    pr_HomeDir       : BPTR;
    pr_Flags         : LongInt;
    pr_ExitCode      : Pointer;  { Procedure }
    pr_ExitData      : LongInt;
    pr_Arguments     : STRPTR;
    pr_LocalVars     : TMinList;
    pr_ShellPrivate  : LongWord;
    pr_CES           : BPTR;
  end;

const
  PRB_FREESEGLIST = 0;
  PRF_FREESEGLIST = 1 Shl PRB_FREESEGLIST;

  PRB_FREECURRDIR = 1;
  PRF_FREECURRDIR = 1 Shl PRB_FREECURRDIR;

  PRB_FREECLI     = 2;
  PRF_FREECLI     = 1 Shl PRB_FREECLI;

  PRB_CLOSEINPUT  = 3;
  PRF_CLOSEINPUT  = 1 Shl PRB_CLOSEINPUT;

  PRB_CLOSEOUTPUT = 4;
  PRF_CLOSEOUTPUT = 1 Shl PRB_CLOSEOUTPUT;

  PRB_FREEARGS    = 5;
  PRF_FREEARGS    = 1 Shl PRB_FREEARGS;

  PRB_CLOSEERROR  = 6;
  PRF_CLOSEERROR  = 1 Shl PRB_CLOSEERROR;


type
  PFileHandle = ^TFileHandle;
  TFileHandle = packed record
    case Byte of
      0: (
        fh_Link      : LongWord;
        fh_Port      : LongInt;
        );
      1: (
        fh_Flags      : LongWord;
        fh_Interactive: LongInt;
        fh_Type       : PMsgPort;
        fh_Buf        : LongInt;
        fh_Pos        : LongInt;
        fh_End        : LongInt;
        fh_Func1      : LongInt;
        fh_Func2      : LongInt;
        fh_Func3      : LongInt;
        fh_Arg1       : LongInt;
        fh_Arg2       : LongInt;
        { *** V50 MorphOS *** }
        fh_BufSize    : LongInt;
        fh_OrigBuf    : LongInt;
      );
  end;

type
  PDOSPacket = ^TDOSPacket;
  TDOSPacket = packed record
    dp_Link: PMessage;
    dp_Port: PMsgPort;
    case Byte of
    0 : ( dp_Action : Longint;
          dp_Status : Longint;
          dp_Status2: Longint;
          dp_BufAddr: Longint;
        );
    1 : ( dp_Type: Longint;
          dp_Res1: Longint;
          dp_Res2: Longint;
          dp_Arg1: Longint;
          dp_Arg2: Longint;
          dp_Arg3: Longint;
          dp_Arg4: Longint;
          dp_Arg5: Longint;
          dp_Arg6: Longint;
          dp_Arg7: Longint;
        );
  end;

type
  PStandardPacket = ^TStandardPacket;
  TStandardPacket = packed record
    sp_Msg: TMessage;
    sp_Pkt: TDOSPacket;
  end;


const
  ACTION_NIL            = 0;
  ACTION_STARTUP        = 0;
  ACTION_GET_BLOCK      = 2; { *** OBSOLETE *** }
  ACTION_SET_MAP        = 4;
  ACTION_DIE            = 5;
  ACTION_EVENT          = 6;
  ACTION_CURRENT_VOLUME = 7;
  ACTION_LOCATE_OBJECT  = 8;
  ACTION_RENAME_DISK    = 9;
  ACTION_WRITE          = $57;  // 'W'
  ACTION_READ           = $52;  // 'R'
  ACTION_FREE_LOCK      = 15;
  ACTION_DELETE_OBJECT  = 16;
  ACTION_RENAME_OBJECT  = 17;
  ACTION_MORE_CACHE     = 18;
  ACTION_COPY_DIR       = 19;
  ACTION_WAIT_CHAR      = 20;
  ACTION_SET_PROTECT    = 21;
  ACTION_CREATE_DIR     = 22;
  ACTION_EXAMINE_OBJECT = 23;
  ACTION_EXAMINE_NEXT   = 24;
  ACTION_DISK_INFO      = 25;
  ACTION_INFO           = 26;
  ACTION_FLUSH          = 27;
  ACTION_SET_COMMENT    = 28;
  ACTION_PARENT         = 29;
  ACTION_TIMER          = 30;
  ACTION_INHIBIT        = 31;
  ACTION_DISK_TYPE      = 32;
  ACTION_DISK_CHANGE    = 33;
  ACTION_SET_DATE       = 34;

  ACTION_SAME_LOCK      = 40;

  ACTION_SCREEN_MODE    = 994;

  ACTION_CHANGE_SIGNAL  = 995;

  ACTION_READ_RETURN     = 1001;
  ACTION_WRITE_RETURN    = 1002;
  ACTION_SEEK            = 1008;
  ACTION_FINDUPDATE      = 1004;
  ACTION_FINDINPUT       = 1005;
  ACTION_FINDOUTPUT      = 1006;
  ACTION_END             = 1007;

  ACTION_FORMAT          = 1020;
  ACTION_MAKE_LINK       = 1021;

  ACTION_SET_FILE_SIZE   = 1022;
  ACTION_WRITE_PROTECT   = 1023;

  ACTION_READ_LINK       = 1024;
  ACTION_FH_FROM_LOCK    = 1026;
  ACTION_IS_FILESYSTEM   = 1027;
  ACTION_CHANGE_MODE     = 1028;

  ACTION_COPY_DIR_FH     = 1030;
  ACTION_PARENT_FH       = 1031;
  ACTION_EXAMINE_ALL     = 1033;
  ACTION_EXAMINE_FH      = 1034;

  ACTION_EXAMINE_ALL_END = 1035;
  ACTION_SET_OWNER       = 1036;

  ACTION_LOCK_RECORD     = 2008;
  ACTION_FREE_RECORD     = 2009;

  ACTION_ADD_NOTIFY      = 4097;
  ACTION_REMOVE_NOTIFY   = 4098;

  ACTION_SERIALIZE_DISK  = 4200;

  ACTION_GET_DISK_FSSM   = 4201;
  ACTION_FREE_DISK_FSSM  = 4202;

  // 64bit DOS extensions - V51
  ACTION_SEEK64           = 26400; // dp_Arg1 - LongInt fh_Arg1, dp_Arg2 - PInt64 position, dp_Arg3 - LongInt mode, dp_Arg4 - PInt64 oldposition
  ACTION_SET_FILE_SIZE64  = 26401; // dp_Arg1 - LongInt fh_Arg1, dp_Arg2 - PInt64 position, dp_Arg3 - LongInt mode, dp_Arg4 - PInt64 newsize
  ACTION_LOCK_RECORD64    = 26402; // dp_Arg1 - LongInt fh_Arg1, dp_Arg2 - PQWord offset, dp_Arg3 - PQWord length, dp_Arg4 - LongWord mode, dp_Arg5 - LongWord timeout
  ACTION_FREE_RECORD64    = 26403; // dp_Arg1 - LongInt fh_Arg1, dp_Arg2 - PQWord offset, dp_Arg3 - PQWord length
  ACTION_RESERVED_1       = 26404; // reserved, do not use
  ACTION_RESERVED_2       = 26405; // reserved, do not use
  ACTION_NEW_READ_LINK    = 26406; // dp_Arg1 - BPTR lock, lock on directory that dp_Arg2 is relative to, dp_Arg2 - UBYTE *name, path and name of link (relative to dp_Arg1), dp_Arg3 - PByte buffer, dp_Arg4 - LongInt buffersize
  ACTION_QUERY_ATTR       = 26407; // dp_Arg1 - LongInt attr, which attribute you want to know about, dp_Arg2 - Pointer storage, memory to hold the return value, dp_Arg3 - LongInt storagesize, size of storage reserved for,
  ACTION_EXAMINE_OBJECT64 = 26408; // dp_Arg1 - BPTR to lock to examine, dp_Arg2 - BPTR to TFileInfoBlock
  ACTION_EXAMINE_NEXT64   = 26409; // dp_Arg1 - BPTR to lock to examine, dp_Arg2 - BPTR to TFileInfoBlock
  ACTION_EXAMINE_FH64     = 26410; // dp_Arg1 - LongInt fh_Arg1, dp_Arg2 - BPTR to TFileInfoBlock

  FQA_MaxFileNameLength   = 0; // LongInt - Return the maximum length of a file name (in characters), excluding terminating '\0' char.
  FQA_MaxVolumeNameLength = 1; // LongInt - Return the maximum length of the volume name (in characters), excluding terminating '\0' char
  FQA_MaxFileSize         = 2; // Int64 - Returns maximum size of the file the filesystem supports.
  FQA_IsCaseSensitive     = 3; // LongInt - If the filesystem names are case sensitive, this attribute must return DOSTrue.
  FQA_DeviceType          = 4; // LongInt - Return the type of the medium the filesystem is using, if known. Value is one of DG_*
  FQA_ReservedAttr1       = 5; // reserved - do not use
  FQA_NumBlocks           = 6; // Int64 - Return the total number of blocks on the filesystem.
  FQA_NumBlocksUsed       = 7; // Int64 - Return the total number of used blocks on the filesystem.

// Packets to get and set filesystem options runtime
  ACTION_GET_PREFS_TEMPLATE = 26500; // arg1 - STRPTR dest_buff, arg2 - LongInt buffer_size
  ACTION_GET_CURRENT_PREFS  = 26501; // arg1 - STRPTR dest_buff, arg2 - LongInt buffer_size
  ACTION_SET_PREFS          = 26502; // arg1 - STRPTR prefs_string

type
  PErrorString = ^TErrorString;
  TErrorString = packed record
    estr_Nums: PLongInt;
    estr_Strings: PByte;
  end;

type
  PRootNode = ^TRootNode;
  TRootNode = packed record
    rn_TaskArray         : BPTR;
    rn_ConsoleSegment    : BPTR;
    rn_Time              : TDateStamp;
    rn_RestartSeg        : LongInt;
    rn_Info              : BPTR;
    rn_FileHandlerSegment: BPTR;
    rn_CliList           : TMinList;
    rn_BootProc          : PMsgPort;
    rn_ShellSegment      : BPTR;
    rn_Flags             : LongInt;
  end;

type
  PDOSLibrary = ^TDOSLibrary;
  TDOSLibrary = packed record
    dl_Lib          : TLibrary;
    dl_Root         : PRootNode;
    dl_GU           : APTR;
    dl_A2           : LongInt;
    dl_A5           : LongInt;
    dl_A6           : LongInt;
    dl_Errors       : PErrorString;
    dl_TimeReq      : PTimeRequest;
    dl_UtilityBase  : PLibrary;
    dl_IntuitionBase: PLibrary;
  end;


const
  RNB_WILDSTAR = 24;
  RNF_WILDSTAR = 1 Shl RNB_WILDSTAR;

  RNB_PRIVATE1 = 1;
  RNF_PRIVATE1 = 1 Shl RNB_PRIVATE1;


type
  PCliProcList = ^TCliProcList;
  TCliProcList = packed record
    cpl_Node : TMinNode;
    cpl_First: LongInt;
    cpl_Array: Array[0..0] Of PMsgPort;
  end;

type
  PDOSInfo = ^TDOSInfo;
  TDOSInfo = packed record
    case Byte of
    0 : ( di_ResList: BPTR;
        );
    1 : ( di_McName    : BPTR;
          di_DevInfo   : BPTR;
          di_Devices   : BPTR;
          di_Handlers  : BPTR;
          di_NetHand   : APTR;
          di_DevLock   : TSignalSemaphore;
          di_EntryLock : TSignalSemaphore;
          di_DeleteLock: TSignalSemaphore;
        );
  end;

type
  PSegment = ^TSegment;
  TSegment = packed record
    seg_Next : BPTR;
    seg_UC   : LongInt;
    seg_Seg  : BPTR;
    seg_Name : Array[0..3] Of Byte;
    { * seg_Name continues * }
  end;


const
  CMD_SYSTEM    = -1;
  CMD_INTERNAL  = -2;
  CMD_NOTLOADED = -998;
  CMD_DISABLED  = -999;


type
  PCommandLineInterface = ^TCommandLineInterface;
  TCommandLineInterface = packed record
    cli_Result2       : LongInt;
    cli_SetName       : BSTR;
    cli_CommandDir    : BPTR;
    cli_ReturnCode    : LongInt;
    cli_CommandName   : BSTR;
    cli_FailLevel     : LongInt;
    cli_Prompt        : BSTR;
    cli_StandardInput : BPTR;
    cli_CurrentInput  : BPTR;
    cli_CommandFile   : BSTR;
    cli_Interactive   : LongInt;
    cli_Background    : LongInt;
    cli_CurrentOutput : BPTR;
    cli_DefaultStack  : LongInt;
    cli_StandardOutput: BPTR;
    cli_Module        : BPTR;
  end;

type
  PDeviceList = ^TDeviceList;
  TDeviceList = packed record
    dl_Next      : BPTR;
    dl_Type      : LongInt;
    dl_Task      : PMsgPort;
    dl_Lock      : BPTR;
    dl_VolumeDate: TDateStamp;
    dl_LockList  : BPTR;
    dl_DiskType  : LongInt;
    dl_unused    : LongInt;
    dl_Name      : BSTR;
  end;

type
  PDevInfo = ^TDevInfo;
  TDevInfo = packed record
    dvi_Next     : BPTR;
    dvi_Type     : LongInt;
    dvi_Task     : Pointer;
    dvi_Lock     : BPTR;
    dvi_Handler  : BSTR;
    dvi_StackSize: LongInt;
    dvi_Priority : LongInt;
    dvi_Startup  : LongInt;
    dvi_SegList  : BPTR;
    dvi_GlobVec  : BPTR;
    dvi_Name     : BSTR;
  end;

type
  PAssignList = ^TAssignList;
  TAssignList = packed record
    al_Next: PAssignList;
    al_Lock: BPTR;
  end;

type
  PDOSList = ^TDOSList;
  TDOSList = packed record
    dol_Next: BPTR;
    dol_Type: LongInt;
    dol_Task: PMsgPort;
    dol_Lock: BPTR;
    case Byte of
    0: ( dol_handler : record
           dol_Handler  : BSTR;
           dol_StackSize: LongInt;
           dol_Priority : LongInt;
           dol_Startup  : LongWord;
           dol_SegList  : BPTR;
           dol_GlobVec  : BPTR;
         end;
       );
    1: ( dol_volume : record
           dol_VolumeDate: TDateStamp;
           dol_LockList  : BPTR;
           dol_DiskType  : LongInt;
         end;
       );
    2: ( dol_assign : record
           dol_AssignName: PChar;
           dol_List      : PAssignList;
         end;
       );
    3: ( dol_Misc: array[0..23] of Byte;
         dol_Name: BPTR;
       );
  end;


const
  DLT_DEVICE     = 0;
  DLT_DIRECTORY  = 1;
  DLT_VOLUME     = 2;
  DLT_LATE       = 3;
  DLT_NONBINDING = 4;
  DLT_PRIVATE    = -1;
// V51 (MorphOS)
  DLT_FSCONTEXT  = 5;

type
  PDevProc = ^TDevProc;
  TDevProc = packed record
    dvp_Port   : PMsgPort;
    dvp_Lock   : BPTR;
    dvp_Flags  : LongWord;
    dvp_DevNode: PDOSList;
  end;


const
  DVPB_UNLOCK = 0;
  DVPF_UNLOCK = 1 Shl DVPB_UNLOCK;

  DVPB_ASSIGN = 1;
  DVPF_ASSIGN = 1 Shl DVPB_ASSIGN;
  // V51 (MorphOS)
  DVPB_FSCONTEXT = 2;
  DVPF_FSCONTEXT = 1 Shl DVPB_FSCONTEXT;

const
  LDB_READ    = 0;
  LDF_READ    = 1 Shl LDB_READ;

  LDB_WRITE   = 1;
  LDF_WRITE   = 1 Shl LDB_WRITE;

  LDB_DEVICES = 2;
  LDF_DEVICES = 1 Shl LDB_DEVICES;

  LDB_VOLUMES = 3;
  LDF_VOLUMES = 1 Shl LDB_VOLUMES;

  LDB_ASSIGNS = 4;
  LDF_ASSIGNS = 1 Shl LDB_ASSIGNS;

  LDB_ENTRY   = 5;
  LDF_ENTRY   = 1 Shl LDB_ENTRY;

  LDB_DELETE  = 6;
  LDF_DELETE  = 1 Shl LDB_DELETE;

  LDF_ALL     = LDF_DEVICES Or LDF_VOLUMES Or LDF_ASSIGNS;


type
  PFileLock = ^TFileLock;
  TFileLock = packed record
    fl_Link  : BPTR;
    fl_Key   : LongInt;
    fl_Access: LongInt;
    fl_Task  : PMsgPort;
    fl_Volume: BPTR;
  end;


const
  REPORT_STREAM = 0;
  REPORT_TASK   = 1;
  REPORT_LOCK   = 2;
  REPORT_VOLUME = 3;
  REPORT_INSERT = 4;

const
  ABORT_DISK_ERROR = 296;
  ABORT_BUSY       = 288;

const
  RUN_EXECUTE       = -1;
  RUN_SYSTEM        = -2;
  RUN_SYSTEM_ASYNCH = -3;

const
  ST_ROOT      = 1;
  ST_USERDIR   = 2;
  ST_SOFTLINK  = 3;
  ST_LINKDIR   = 4;
  ST_FILE      = -3;
  ST_LINKFILE  = -4;
  ST_PIPEFILE  = -5;

type
  TCLIDataItem = record
    cdi_CLINum: LongInt;              // CLI number of the CLI
    cdi_DefaultStack: LongInt;        // cli_DefaultStack of the CLI
    cdi_GlobVec: LongInt;             // pr_GlobVec[0] of the CLI
    cdi_Future: LongWord;             // For future expansion, 0 for now
    cdi_Pri: ShortInt;                // CLI priority
    cdi_Flags: Byte;                  // If bit 0 is set cdi_command is valid
    cdi_Command: array[0..0] of Char; // 0-terminated command being executed
  end;
  PCLIDataItem = ^TCLIDataItem;

  TCLIData = record
    cd_NumCLIs: LongWord; // Number of entries in cd_cli array
    cd_CLI: array[0..100] of PCLIDataItem; // the entries (could be more than 101 though)
  end;
  PCLIData = ^TCLIData;



{ * dos asl definitions
  *********************************************************************
  * }


type
  PAChain = ^TAChain;
  TAChain = packed record
    an_Child : PAChain;
    an_Parent: PAChain;
    an_Lock  : BPTR;
    an_Info  : TFileInfoBlock;
    an_Flags : ShortInt;
    an_String: Array[0..0] Of Char;
    { * an_String continues * }
  end;

type
  PAnchorPath = ^TAnchorPath;
  TAnchorPath = packed record
    case Byte of
    0 : ( ap_First: PAChain;
          ap_Last : PAChain;
          ap_BreakBits_1 : LongInt; // just as placeholder
          ap_FoundBreak_1: LongInt; // just as placeholder
          ap_Length      : ShortInt;
          ap_Extended    : ShortInt;
        );
    1 : ( ap_Base      : PAChain;
          ap_Current   : PAChain;
          ap_BreakBits : LongInt;
          ap_FoundBreak: LongInt;
          ap_Flags     : ShortInt;
          ap_Reserved  : ShortInt;
          ap_Strlen    : SmallInt;
          ap_Info      : TFileInfoBlock;
          ap_Buf       : Array[0..1] of Char;
          { * an_Buf continues * }
        );
  end;


const
  APB_DOWILD       = 0;
  APF_DOWILD       = 1 Shl APB_DOWILD;

  APB_ITSWILD      = 1;
  APF_ITSWILD      = 1 Shl APB_ITSWILD;

  APB_DODIR        = 2;
  APF_DODIR        = 1 Shl APB_DODIR;

  APB_DIDDIR       = 3;
  APF_DIDDIR       = 1 Shl APB_DIDDIR;

  APB_NOMEMERR     = 4;
  APF_NOMEMERR     = 1 Shl APB_NOMEMERR;

  APB_DODOT        = 5;
  APF_DODOT        = 1 Shl APB_DODOT;

  APB_DirChanged   = 6;
  APF_DirChanged   = 1 Shl APB_DirChanged;

  APB_FollowHLinks = 7;
  APF_FollowHLinks = 1 Shl APB_FollowHLinks;

const
  APSB_EXTENDED        = 15;
  APSF_EXTENDED        = 1 Shl APSB_EXTENDED;

// Following are ap_Extended flags, only enabled when ap_StrLen has
//   APSF_EXTENDED set. Added in dos.library 50.67.

// Return literal softlink names instead of the link destination
  APEB_LiteralSLinks   = 1;
  APEF_LiteralSLinks   = 1 shl APEB_LiteralSLinks;

// Don't follow softlink directories, implies APEF_LiteralSLinks
  APEB_DontFollowSLinks = 2;
  APEF_DontFollowSLinks = 1 shl APEB_DontFollowSLinks;

  APEB_FutureExtension = 7;
  APEF_FutureExtension = 1 Shl APEB_FutureExtension;

const
  DDB_PatternBit  = 0;
  DDF_PatternBit  = 1 Shl DDB_PatternBit;

  DDB_ExaminedBit = 1;
  DDF_ExaminedBit = 1 Shl DDB_ExaminedBit;

  DDB_Completed   = 2;
  DDF_Completed   = 1 Shl DDB_Completed;

  DDB_AllBit      = 3;
  DDF_AllBit      = 1 Shl DDB_AllBit;

  DDB_Single      = 4;
  DDF_Single      = 1 Shl DDB_Single;

const
  P_ANY      = $80;
  P_SINGLE   = $81;
  P_ORSTART  = $82;
  P_ORNEXT   = $83;
  P_OREND    = $84;
  P_NOT      = $85;
  P_NOTEND   = $86;
  P_NOTCLASS = $87;
  P_CLASS    = $88;
  P_REPBEG   = $89;
  P_REPEND   = $8A;
  P_STOP     = $8B;

const
  COMPLEX_BIT = 1;
  EXAMINE_BIT = 2;

const
  ERROR_BUFFER_OVERFLOW = 303;
  ERROR_BREAK           = 304;
  ERROR_NOT_EXECUTABLE  = 305;


{ * dos hunk definitions
  *********************************************************************
  * }

const
  HUNK_UNIT         = 999;
  HUNK_NAME         = 1000;
  HUNK_CODE         = 1001;
  HUNK_DATA         = 1002;
  HUNK_BSS          = 1003;

  HUNK_RELOC32      = 1004;
  HUNK_ABSRELOC32   = HUNK_RELOC32;

  HUNK_RELOC16      = 1005;
  HUNK_RELRELOC16   = HUNK_RELOC16;

  HUNK_RELOC8       = 1006;
  HUNK_RELRELOC8    = HUNK_RELOC8;

  HUNK_EXT          = 1007;
  HUNK_SYMBOL       = 1008;
  HUNK_DEBUG        = 1009;
  HUNK_END          = 1010;
  HUNK_HEADER       = 1011;

  HUNK_OVERLAY      = 1013;
  HUNK_BREAK        = 1014;

  HUNK_DREL32       = 1015;
  HUNK_DREL16       = 1016;
  HUNK_DREL8        = 1017;

  HUNK_LIB          = 1018;
  HUNK_INDEX        = 1019;

  HUNK_RELOC32SHORT = 1020;

  HUNK_RELRELOC32   = 1021;
  HUNK_ABSRELOC16   = 1022;

const
  HUNKB_ADVISORY = 29;
  HUNKB_CHIP     = 30;
  HUNKB_FAST     = 31;

  HUNKF_ADVISORY = 1 Shl HUNKB_ADVISORY;
  HUNKF_CHIP     = 1 Shl HUNKB_CHIP;
  HUNKF_FAST     = 1 Shl HUNKB_FAST;

const
  EXT_SYMB      = 0;
  EXT_DEF       = 1;
  EXT_ABS       = 2;
  EXT_RES       = 3;

  EXT_REF32     = 129;
  EXT_ABSREF32  = EXT_REF32;

  EXT_COMMON    = 130;
  EXT_ABSCOMMON = EXT_COMMON;

  EXT_REF16     = 131;
  EXT_RELREF16  = EXT_REF16;

  EXT_REF8      = 132;
  EXT_RELREF8   = EXT_REF8;

  EXT_DEXT32    = 133;
  EXT_DEXT16    = 134;
  EXT_DEXT8     = 135;

  EXT_RELREF32  = 136;
  EXT_RELCOMMON = 137;

  EXT_ABSREF16  = 138;

  EXT_ABSREF8   = 139;


{ * dos ExAll definitions
  *********************************************************************
  * }

const
  ED_NAME       = 1;
  ED_TYPE       = 2;
  ED_SIZE       = 3;
  ED_PROTECTION = 4;
  ED_DATE       = 5;
  ED_COMMENT    = 6;
  ED_OWNER      = 7;
  // 64bit DOS extensions - V51
  // The ExAllData ed_Size64 field is filled. For files larger than 2^31-1 bytes, ed_Size is 0.
  ED_SIZE64     = 8;

type
  PExAllData = ^TExAllData;
  TExAllData = packed record
    ed_Next    : PExAllData;
    ed_Name    : PChar;
    ed_Type    : LongInt;
    ed_Size    : LongWord;
    ed_Prot    : LongWord;
    ed_Days    : LongWord;
    ed_Mins    : LongWord;
    ed_Ticks   : LongWord;
    ed_Comment : PChar;
    ed_OwnerUID: Word;
    ed_OwnerGID: Word;
    ed_Size64: QWord;
  end;

type
  PExAllControl = ^TExAllControl;
  TexAllControl = packed record
    eac_Entries    : LongWord;
    eac_LastKey    : LongWord;
    eac_MatchString: PChar;
    eac_MatchFunc  : PHook;

  end;



{ * dos record definitions
  *********************************************************************
  * }


const
  REC_EXCLUSIVE       = 0;
  REC_EXCLUSIVE_IMMED = 1;
  REC_SHARED          = 2;
  REC_SHARED_IMMED    = 3;


type
  PRecordLock = ^TRecordLock;
  TRecordLock = packed record
    rec_FH: BPTR;
    rec_Offset: LongWord;
    rec_Length: LongWord;
    rec_Mode: LongWord;
  end;

  // 64bit DOS extensions - V51
  PRecordLock64 = ^TRecordLock64;
  TRecordLock64 = packed record
    rec_FH: BPTR;
    rec_Offset: QWord;
    rec_Length: QWord;
    rec_Mode: LongWord;
  end;


{ * dos tag definitions (V50)
  *********************************************************************
  * }


const
  SYS_Dummy       = TAG_USER + 32;
  SYS_Input       = SYS_Dummy + 1;
  SYS_Output      = SYS_Dummy + 2;
  SYS_Asynch      = SYS_Dummy + 3;
  SYS_UserShell   = SYS_Dummy + 4;
  SYS_CustomShell = SYS_Dummy + 5;

  { *** V50 *** }
  SYS_FilterTags  = SYS_Dummy + 6;   { * filters the tags passed down to CreateNewProc(), default: TRUE * }

const
  NP_Dummy         = TAG_USER + 1000;
  NP_Seglist       = NP_Dummy + 1;
  NP_FreeSeglist   = NP_Dummy + 2;
  NP_Entry         = NP_Dummy + 3;
  NP_Input         = NP_Dummy + 4;
  NP_Output        = NP_Dummy + 5;
  NP_CloseInput    = NP_Dummy + 6;
  NP_CloseOutput   = NP_Dummy + 7;
  NP_Error         = NP_Dummy + 8;
  NP_CloseError    = NP_Dummy + 9;
  NP_CurrentDir    = NP_Dummy + 10;
  NP_StackSize     = NP_Dummy + 11;
  NP_Name          = NP_Dummy + 12;
  NP_Priority      = NP_Dummy + 13;
  NP_ConsoleTask   = NP_Dummy + 14;
  NP_WindowPtr     = NP_Dummy + 15;
  NP_HomeDir       = NP_Dummy + 16;
  NP_CopyVars      = NP_Dummy + 17;
  NP_Cli           = NP_Dummy + 18;
  NP_Path          = NP_Dummy + 19;
  NP_CommandName   = NP_Dummy + 20;
  NP_Arguments     = NP_Dummy + 21;

  NP_NotifyOnDeath = NP_Dummy + 22;
  NP_Synchronous   = NP_Dummy + 23;
  NP_ExitCode      = NP_Dummy + 24;
  NP_ExitData      = NP_Dummy + 25;

  { *** V50 *** }
  NP_SeglistArray  = NP_Dummy + 26;
  NP_UserData      = NP_Dummy + 27;
  NP_StartupMsg    = NP_Dummy + 28;  { * PMessage, ReplyMsg'd at exit * }
  NP_TaskMsgPort   = NP_Dummy + 29;  { * ^PMsgPort, create MsgPort, automagic delete * }

  NP_CodeType      = NP_Dummy + 100;
  NP_PPC_Arg1      = NP_Dummy + 101;
  NP_PPC_Arg2      = NP_Dummy + 102;
  NP_PPC_Arg3      = NP_Dummy + 103;
  NP_PPC_Arg4      = NP_Dummy + 104;
  NP_PPC_Arg5      = NP_Dummy + 105;
  NP_PPC_Arg6      = NP_Dummy + 106;
  NP_PPC_Arg7      = NP_Dummy + 107;
  NP_PPC_Arg8      = NP_Dummy + 108;
  NP_PPCStackSize  = NP_Dummy + 109;

const
  ADO_Dummy       = TAG_USER + 2000;
  ADO_FH_Mode     = ADO_Dummy + 1;

  ADO_DirLen      = ADO_Dummy + 2;
  ADO_CommNameLen = ADO_Dummy + 3;
  ADO_CommFileLen = ADO_Dummy + 4;
  ADO_PromptLen   = ADO_Dummy + 5;

  { *** V50 *** }
  ADDS_Dummy       = TAG_USER + 3000;
  ADDS_Name        = ADDS_Dummy + 1;  // Segment name
  ADDS_Seglist     = ADDS_Dummy + 2;  // Seglist for this segment
  ADDS_Filename    = ADDS_Dummy + 3;  // Name of the file to load when needed. Ignored if Seglist is given.
  ADDS_Type        = ADDS_Dummy + 4;  // Segment type

  ADO_CLI_Dir          = ADO_Dummy + 20; // BSTR fixed
  ADO_CLI_CommName     = ADO_Dummy + 21; // BSTR fixed
  ADO_CLI_CommFile     = ADO_Dummy + 22; // BSTR fixed
  ADO_CLI_Prompt       = ADO_Dummy + 23; // BSTR fixed
  ADO_CLI_Result2      = ADO_Dummy + 24; // LongInt
  ADO_CLI_ReturnCode   = ADO_Dummy + 25; // LongInt
  ADO_CLI_FailLevel    = ADO_Dummy + 26; // LongInt
  ADO_CLI_Interactive  = ADO_Dummy + 27; // LongInt
  ADO_CLI_Background   = ADO_Dummy + 28; // LongInt
  ADO_CLI_DefaultStack = ADO_Dummy + 29; // LongInt

// DOS_DEVICENODE tags
  ADO_DN_Name          = ADO_Dummy + 30; // BSTR
  ADO_DN_MsgPort       = ADO_Dummy + 31; // PMsgPort
  ADO_DN_Lock          = ADO_Dummy + 32; // BPTR
  ADO_DN_Handler       = ADO_Dummy + 33; // BSTR
  ADO_DN_Seglist       = ADO_Dummy + 34; // BPTR
  ADO_DN_StackSize     = ADO_Dummy + 35; // LongWord Default: 4096
  ADO_DN_Priority      = ADO_Dummy + 36; // LongInt
  ADO_DN_Startup       = ADO_Dummy + 37; // BSTR
  ADO_DN_GlobalVec     = ADO_Dummy + 38; // BPTR

// Only for new devicenodes created by AllocDosObject
  ADO_DN_SerialID      = ADO_Dummy + 39; // BPTR
// Value < $100 as startup argument
  ADO_DN_StartupValue  = ADO_Dummy + 40; // LongWord

// New = dos.lib 51.40+; tags
  ADO_DN_Flags         = ADO_Dummy + 41; // LongWord
  ADO_DN_Status        = ADO_Dummy + 42; // LongWord
  ADO_DN_ExitNotifyMsg = ADO_Dummy + 43; // PMsg

// Run handler only once, do not reload after exit
  DNF_STARTONCE     = 1 shl 0;
// Free handler seglist after handler exit
  DNF_UNLOADSEGLIST = 1 shl 1;
// Remove doslist entry after handler exit
  DNF_REMDOSLIST    = 1 shl 2;

// Only useful for GetDosObjectAttr
  ADO_FSSM             = ADO_Dummy + 50; // PFileSysStartupMsg
  ADO_FSSM_Device      = ADO_Dummy + 51; // BSTR
  ADO_FSSM_Unit        = ADO_Dummy + 52; // LongWord
  ADO_FSSM_Flags       = ADO_Dummy + 53; // LongWord

// Only useful for GetDosObjectAttr
  ADO_DE               = ADO_Dummy + 60; // PDosEnvec
  ADO_DE_TableSize     = ADO_Dummy + 61; // LongWord Default: DE_BOOTBLOCKS
  ADO_DE_SizeBlock     = ADO_Dummy + 62; // LongWord Default: 512
  ADO_DE_SecOrg        = ADO_Dummy + 63; // LongWord
  ADO_DE_NumHeads      = ADO_Dummy + 64; // LongWord
  ADO_DE_Surfaces      = ADO_DE_NumHeads;
  ADO_DE_SecsPerBlk    = ADO_Dummy + 65; // LongWord Default: 1
  ADO_DE_BlksPerTrack  = ADO_Dummy + 66; // LongWord
  ADO_DE_ReservedBlks  = ADO_Dummy + 67; // LongWord
  ADO_DE_PreFac        = ADO_Dummy + 68; // LongWord
  ADO_DE_PreAlloc      = ADO_DE_PreFac;
  ADO_DE_Interleave    = ADO_Dummy + 69; // LongWord
  ADO_DE_LowCyl        = ADO_Dummy + 70; // LongWord
  ADO_DE_UpperCyl      = ADO_Dummy + 71; // LongWord
  ADO_DE_HighCyl       = ADO_DE_UpperCyl;
  ADO_DE_NumBuffers    = ADO_Dummy + 72; // LongWord
  ADO_DE_BufMemType    = ADO_Dummy + 73; // LongWord
  ADO_DE_MaxTransfer   = ADO_Dummy + 74; // LongWord Default: $ffffffff
  ADO_DE_Mask          = ADO_Dummy + 75; // LongWord Default: $ffffffff
  ADO_DE_BootPri       = ADO_Dummy + 76; // LongInt
  ADO_DE_DosType       = ADO_Dummy + 77; // LongWord
  ADO_DE_Baud          = ADO_Dummy + 78; // LongWord
  ADO_DE_Control       = ADO_Dummy + 79; // BSTR
  ADO_DE_BootBlocks    = ADO_Dummy + 80; // LongWord

// Used by AllocDosObject
  ADO_FS_DosType       = ADO_Dummy + 90; // LongWord

  ADO_VN_Name          = ADO_Dummy + 100; // BSTR
  ADO_VN_MsgPort       = ADO_Dummy + 101; // PMsgPort
  ADO_VN_Lock          = ADO_Dummy + 102; // BPTR
  ADO_VN_LockList      = ADO_Dummy + 103; // PLock
  ADO_VN_Date          = ADO_Dummy + 104; // PDateStamp
  ADO_VN_DiskType      = ADO_Dummy + 105; // LongWord


  ADO_AN_Name          = ADO_Dummy + 120; // BSTR
  ADO_AN_MsgPort       = ADO_Dummy + 121; // PMsgPort
  ADO_AN_Lock          = ADO_Dummy + 122; // BPTR
  ADO_AN_Type          = ADO_Dummy + 123; // LongWord
  ADO_AN_AssignName    = ADO_Dummy + 124; // PChar
  ADO_AN_AssignList    = ADO_Dummy + 125; // PAssignList

// GetDosObjectAttr uses this to store variable size entries.
type
  TDosAttrBuffer = record
    dab_Ptr: APTR;
    dab_Len: LongWord;
  end;
  PDosAttrBuffer = ^TDosAttrBuffer;

const
  FNDS_Dummy      = TAG_USER + 3100;
  FNDS_Name       = FNDS_Dummy + 1;  { * Segment name * }
  FNDS_From       = FNDS_Dummy + 2;  { * Segment to start from * }
  FNDS_System     = FNDS_Dummy + 3;  { * Look for a system segment ? * }
  FNDS_Load       = FNDS_Dummy + 4;  { * Load the seglist if needed ? (Default: TRUE) * }

  // V51
  FSCONTEXTTAG_Dummy = TAG_USER + 3200;

// Pass a custom handler seglist. This handler MUST be able to handle the fscontext filesystem extension

  FSCONTEXTTAG_SEGLIST            = FSCONTEXTTAG_Dummy + $1;
  FSCONTEXTTAG_PRIORITY           = FSCONTEXTTAG_Dummy + $2;
  FSCONTEXTTAG_STACKSIZE          = FSCONTEXTTAG_Dummy + $3;
  FSCONTEXTTAG_STARTUPSTRING      = FSCONTEXTTAG_Dummy + $4;
  FSCONTEXTTAG_STARTUPVALUE       = FSCONTEXTTAG_Dummy + $5;
  FSCONTEXTTAG_FSSM               = FSCONTEXTTAG_Dummy + $6;
  // A matching filesystem is searched through query.library
  FSCONTEXTTAG_DOSTYPE            = FSCONTEXTTAG_Dummy + $7;

  FSCONTEXTINFOTAG_Dummy          = TAG_USER + 3300;
  FSCONTEXTINFOTAG_NAME           = FSCONTEXTINFOTAG_Dummy + $1;

  SEGLISTTAG_Dummy                = TAG_USER + 3400;

  SEGLISTTAG_OBJDATA              = SEGLISTTAG_Dummy + $1; // return the ObjData object when it exists or nil.
  // V51.52
  SEGLISTTAG_SEGLISTTYPE          = SEGLISTTAG_Dummy + $2; // return the seglist type, one of SEGLISTTYPE_*.
  SEGLISTTAG_DOS_SEGINDEX         = SEGLISTTAG_Dummy + $3; // specify that segment index is the hunk number, starting from 0.
  SEGLISTTAG_ELF_SEGINDEX         = SEGLISTTAG_Dummy + $4; // specify that the segment index is the elf segment number, starting from 1.
  SEGLISTTAG_SEGSTART             = SEGLISTTAG_Dummy + $5; // return segment start address for segment specified by either SEGLISTTAG_DOS_SEGINDEX or SEGLISTTAG_ELF_SEGINDEX.
                                                           // note that SEGLISTTAG_ELF_SEGINDEX segments might return a nil pointer, so be prepared for this.
  SEGLISTTAG_SEGSIZE              = SEGLISTTAG_Dummy + $6; // return segment data size for segment specified by either SEGLISTTAG_DOS_SEGINDEX or SEGLISTTAG_ELF_SEGINDEX.
  // V51.54
  SEGLISTTAG_ELF_SEGTYPE          = SEGLISTTAG_Dummy + $7; // return ELF segment type (ELF SHT_*). Only applicable for ELF.
  SEGLISTTAG_ELF_SEGOFFSET        = SEGLISTTAG_Dummy + $8; // return ELF segment file offset. Only applicable for ELF.
  SEGLISTTAG_ELF_SEGFLAGS         = SEGLISTTAG_Dummy + $9; // return ELF segment flags. Meaning depends on segment type. Refer to ELF documentation for details. Only applicable for ELF.
  SEGLISTTAG_ELF_SEGADDRALIGN     = SEGLISTTAG_Dummy + $a; // return ELF segment alignment. 0 and 1 mean unaligned. Only applicable for ELF.
  SEGLISTTAG_ELF_SEGNAME          = SEGLISTTAG_Dummy + $b; // return ELF segname name. Only applicable for ELF.


  // for tag SEGLISTTAG_SEGLISTTYPE
  SEGLISTTYPE_ELF                 = 1;
  SEGLISTTYPE_POWERUP             = 2;
  SEGLISTTYPE_AMIGA               = 3;

  // QueryCLIDataTagList tags (V51.51)
  CLIDATATAG_Dummy = TAG_USER + 3500;
  CLIDATATAG_CLINumber   = CLIDATATAG_Dummy + $1; // Return only CLI matching the given CLI number (returns 0 or 1 entries)
  CLIDATATAG_CommandName = CLIDATATAG_Dummy + $2; // Return only CLIs matching the given command (0 to n entries possible)
  CLIDATATAG_Sorted      = CLIDATATAG_Dummy + $3; // When ti_Data is TRUE, return results sorted by CLI number(default to FALSE)


{ * dos stdio definitions
  *********************************************************************
  * }

const
 BUF_LINE = 0;
 BUF_FULL = 1;
 BUF_NONE = 2;

const
 ENDSTREAMCH = -1;



{ * dos env-var definitions
  *********************************************************************
  * }


type
  PLocalVar = ^TLocalVar;
  TLocalVar = packed record
    lv_Node : TNode;
    lv_Flags: Word;
    lv_Value: PChar;
    lv_Len  : LongWord;
  end;


const
  LV_VAR   = 0;
  LV_ALIAS = 1;

const
  LVB_IGNORE         = 7;
  LVF_IGNORE         = 1 Shl LVB_IGNORE;

  GVB_GLOBAL_ONLY    = 8;
  GVF_GLOBAL_ONLY    = 1 Shl GVB_GLOBAL_ONLY;

  GVB_LOCAL_ONLY     = 9;
  GVF_LOCAL_ONLY     = 1 Shl GVB_LOCAL_ONLY;

  GVB_BINARY_VAR     = 10;
  GVF_BINARY_VAR     = 1 Shl GVB_BINARY_VAR;

  GVB_DONT_NULL_TERM = 11;
  GVF_DONT_NULL_TERM = 1 Shl GVB_DONT_NULL_TERM;

  GVB_SAVE_VAR       = 12;
  GVF_SAVE_VAR       = 1 Shl GVB_SAVE_VAR;



{ * dos ReadArgs definitions
  *********************************************************************
  * }


type
  PCSource = ^TCSource;
  TCSource = packed record
    CS_Buffer: PChar;
    CS_Length: LongInt;
    CS_CurChr: LongInt;
  end;

type
  PRDArgs = ^TRDArgs;
  TRDArgs = packed record
    RDA_Source : TCSource;
    RDA_DAList : LongInt;
    RDA_Buffer : PChar;
    RDA_BufSiz : LongInt;
    RDA_ExtHelp: PChar;
    RDA_Flags  : LongInt;
  end;


const
  RDAB_STDIN    = 0;
  RDAF_STDIN    = 1 Shl RDAB_STDIN;

  RDAB_NOALLOC  = 1;
  RDAF_NOALLOC  = 1 Shl RDAB_NOALLOC;

  RDAB_NOPROMPT = 2;
  RDAF_NOPROMPT = 1 Shl RDAB_NOPROMPT;

const
  MAX_TEMPLATE_ITEMS = 100;
  MAX_MULTIARGS      = 128;



{ * dos filehandler definitions
  *********************************************************************
  * }


type
  PDosEnvec = ^TDosEnvec;
  TDosEnvec = packed record
    de_TableSize     : LongWord;
    de_SizeBlock     : LongWord;
    de_SecOrg        : LongWord;
    de_Surfaces      : LongWord;
    de_SectorPerBlock: LongWord;
    de_BlocksPerTrack: LongWord;
    de_Reserved      : LongWord;
    de_PreAlloc      : LongWord;
    de_Interleave    : LongWord;
    de_LowCyl        : LongWord;
    de_HighCyl       : LongWord;
    de_NumBuffers    : LongWord;
    de_BufMemType    : LongWord;
    de_MaxTransfer   : LongWord;
    de_Mask          : LongWord;
    de_BootPri       : LongInt;
    de_DosType       : LongWord;
    de_Baud          : LongWord;
    de_Control       : LongWord;
    de_BootBlocks    : LongWord;
  end;


const
  DE_TABLESIZE    = 0;
  DE_SIZEBLOCK    = 1;
  DE_SECORG       = 2;
  DE_NUMHEADS     = 3;
  DE_SECSPERBLK   = 4;
  DE_BLKSPERTRACK = 5;
  DE_RESERVEDBLKS = 6;
  DE_PREFAC       = 7;
  DE_INTERLEAVE   = 8;
  DE_LOWCYL       = 9;
  DE_UPPERCYL     = 10;
  DE_NUMBUFFERS   = 11;
  DE_MEMBUFTYPE   = 12;
  DE_BUFMEMTYPE   = 12;
  DE_MAXTRANSFER  = 13;
  DE_MASK         = 14;
  DE_BOOTPRI      = 15;
  DE_DOSTYPE      = 16;
  DE_BAUD         = 17;
  DE_CONTROL      = 18;
  DE_BOOTBLOCKS   = 19;


type
  PFileSysStartupMsg = ^TFileSysStartupMsg;
  TFileSysStartupMsg = packed record
    fssm_Unit   : LongWord;
    fssm_Device : BSTR;
    fssm_Environ: BPTR;
    fssm_Flags  : LongWord;
  end;

type
  PDeviceNode = ^TDeviceNode;
  TDeviceNode = packed record
    dn_Next     : BPTR;
    dn_Type     : LongWord;
    dn_Task     : PMsgPort;
    dn_Lock     : BPTR;
    dn_Handler  : BSTR;
    dn_StackSize: LongWord;
    dn_Priority : LongInt;
    dn_Startup  : BPTR;
    dn_SegList  : BPTR;
    dn_GlobalVec: BPTR;
    dn_Name     : BSTR;
  end;



{ * dos notification definitions
  *********************************************************************
  * }


const
  NOTIFY_CLASS = $40000000;
  NOTIFY_CODE  = $1234;


type
  PNotifyRequest = ^TNotifyRequest;
  TNotifyRequest = packed record
    nr_Name    : PChar;
    nr_FullName: PChar;
    nr_UserData: LongWord;
    nr_Flags   : LongWord;
    nr_stuff : record
      case Byte of
      0 : ( nr_Msg : record
              nr_Port: PMsgPort;
            end );
      1 : ( nr_Signal : record
              nr_Task     : PTask;
              nr_SignalNum: Byte;
              nr_pad      : Array[0..2] Of Byte;
            end );
    end;
    nr_Reserved: Array[0..3] Of LongWord;
    nr_MsgCount: LongWord;
    nr_Handler : PMsgPort;
  end;

type
  PNotifyMessage = ^TNotifyMessage;
  TNotifyMessage = packed record
    nm_ExecMessage: TMessage;
    nm_Class      : LongWord;
    nm_Code       : Word;
    nm_NReq       : PNotifyRequest;
    nm_DoNotTouch : LongWord;
    nm_DoNotTouch2: LongWord;
  end;


const
  NRB_SEND_MESSAGE   = 0;
  NRB_SEND_SIGNAL    = 1;
  NRB_WAIT_REPLY     = 3;
  NRB_NOTIFY_INITIAL = 4;

  NRB_MAGIC          = 31;

const
  NRF_SEND_MESSAGE   = 1 Shl NRB_SEND_MESSAGE;
  NRF_SEND_SIGNAL    = 1 Shl NRB_SEND_SIGNAL;
  NRF_WAIT_REPLY     = 1 Shl NRB_WAIT_REPLY;
  NRF_NOTIFY_INITIAL = 1 Shl NRB_NOTIFY_INITIAL;

  NRF_MAGIC          = 1 Shl NRB_MAGIC;

const
  NR_HANDLER_FLAGS = $ffff0000;



{ * dos.library segtracker include
  *********************************************************************
  * }


const
  SEG_SEM = 'SegTracker';


type
  PSegSem = ^TSegSem;
  TSegSem = packed record
    seg_Semaphore: TSignalSemaphore;
    seg_Find     : Procedure; { Name = seg_Find(REG(a0, ULONG Address), REG(a1, ULONG *SegNum), REG(a2, ULONG *Offset)) }
    seg_List     : TMinList;
  end;

type
  PSegArray = ^TSegArray;
  TSegArray = packed record
    seg_Address: Cardinal;
    seg_Size   : Cardinal;
  end;

type
  PSegNode = ^TSegNode;
  TSegNode = packed record
    seg_Node : TMinNode;
    seg_Name : PChar;
    seg_Array: Array[0..0] Of TSegArray;
  end;


{.$include doslibd.inc}
{.$include doslibf.inc}

{ dos.library functions }

function dosOpen(fname     : PChar   location 'd1';
              accessMode: LongInt location 'd2'): BPTR;
SysCall MOS_DOSBase 30;

function dosClose(fileh: BPTR location 'd1'): LongBool;
SysCall MOS_DOSBase 36;

function dosRead(fileh : BPTR location 'd1';
                 buffer: Pointer location 'd2';
                 length: LongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 42;

function dosWrite(fileh : BPTR location 'd1';
                  buffer: Pointer location 'd2';
                  length: LongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 48;

function dosInput: BPTR;
SysCall MOS_DOSBase 54;

function dosOutput: BPTR;
SysCall MOS_DOSBase 60;

function dosSeek(fileh   : BPTR location 'd1';
                 position: LongInt location 'd2';
                 posmode : LongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 66;

function dosDeleteFile(fname: PChar location 'd1'): LongBool;
SysCall MOS_DOSBase 72;

function dosRename(oldName: PChar location 'd1';
                   newName: PChar location 'd2'): LongInt;
SysCall MOS_DOSBase 78;

function Lock(lname     : PChar   location 'd1';
              accessMode: LongInt location 'd2'): BPTR;
SysCall MOS_DOSBase 84;

procedure Unlock(lock: BPTR location 'd1');
SysCall MOS_DOSBase 90;

function DupLock(lock: BPTR location 'd1'): LongInt;
SysCall MOS_DOSBase 096;

function Examine(lock         : BPTR        location 'd1';
                 fileInfoBlock: PFileInfoBlock location 'd2'): LongInt;
SysCall MOS_DOSBase 102;

function ExNext(lock         : BPTR        location 'd1';
                fileInfoBlock: PFileInfoBlock location 'd2'): LongInt;
SysCall MOS_DOSBase 108;

function Info(lock          : BPTR   location 'd1';
              parameterBlock: PInfoData location 'd2'): LongInt;
SysCall MOS_DOSBase 114;

function dosCreateDir(dname: PChar location 'd1'): LongInt;
SysCall MOS_DOSBase 120;

function CurrentDir(lock: BPTR location 'd1'): BPTR;
SysCall MOS_DOSBase 126;

function IoErr: LongInt;
SysCall MOS_DOSBase 132;

function CreateProc(name     : PChar   location 'd1';
                    pri      : LongInt location 'd2';
                    segList  : BPTR location 'd3';
                    stackSize: LongInt location 'd4'): PMsgPort;
SysCall MOS_DOSBase 138;

procedure dosExit(returnCode: LongInt location 'd1');
SysCall MOS_DOSBase 144;

function LoadSeg(name: PChar location 'd1'): BPTR;
SysCall MOS_DOSBase 150;

procedure UnLoadSeg(seglist: BPTR location 'd1');
SysCall MOS_DOSBase 156;

function DeviceProc(name: PChar location 'd1'): PMsgPort;
SysCall MOS_DOSBase 174;

function SetComment(name   : PChar location 'd1';
                    comment: PChar location 'd2'): LongBool;
SysCall MOS_DOSBase 180;

function SetProtection(name: PChar   location 'd1';
                       mask: LongInt location 'd2'): LongInt;
SysCall MOS_DOSBase 186;

function DateStamp(date: PDateStamp location 'd1'): PDateStamp;
SysCall MOS_DOSBase 192;

procedure DOSDelay(timeout: LongInt location 'd1');
SysCall MOS_DOSBase 198;

function WaitForChar(file1  : LongInt location 'd1';
                     timeout: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 204;

function ParentDir(lock: BPTR location 'd1'): BPTR;
SysCall MOS_DOSBase 210;

function IsInteractive(file1: BPTR location 'd1'): LongBool;
SysCall MOS_DOSBase 216;

function Execute(string1: PChar   location 'd1';
                 file1  : BPTR location 'd2';
                 file2  : BPTR location 'd3'): LongBool;
SysCall MOS_DOSBase 222;

function AllocDosObject(type1: Cardinal location 'd1';
                        tags : PTagItem location 'd2'): Pointer;
SysCall MOS_DOSBase 228;

function AllocDosObjectTagList(type1: Cardinal location 'd1';
                               tags : PTagItem location 'd2'): Pointer;
SysCall MOS_DOSBase 228;

procedure FreeDosObject(type1: Cardinal location 'd1';
                        ptr  : Pointer  location 'd2');
SysCall MOS_DOSBase 234;

function DoPkt(port  : PMsgPort location 'd1';
               action: LongInt  location 'd2';
               arg1  : LongInt  location 'd3';
               arg2  : LongInt  location 'd4';
               arg3  : LongInt  location 'd5';
               arg4  : LongInt  location 'd6';
               arg5  : LongInt  location 'd7'): LongInt;
SysCall MOS_DOSBase 240;

function DoPkt0(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2'): LongInt;
SysCall MOS_DOSBase 240;

function DoPkt1(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3'): LongInt;
SysCall MOS_DOSBase 240;

function DoPkt2(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3';
                arg2  : LongInt  location 'd4'): LongInt;
SysCall MOS_DOSBase 240;

function DoPkt3(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3';
                arg2  : LongInt  location 'd4';
                arg3  : LongInt  location 'd5'): LongInt;
SysCall MOS_DOSBase 240;

function DoPkt4(port  : PMsgPort location 'd1';
                action: LongInt  location 'd2';
                arg1  : LongInt  location 'd3';
                arg2  : LongInt  location 'd4';
                arg3  : LongInt  location 'd5';
                arg4  : LongInt  location 'd6'): LongInt;
SysCall MOS_DOSBase 240;

procedure SendPkt(dp       : PDosPacket location 'd1';
                  port     : PMsgPort   location 'd2';
                  replyport: PMsgPort   location 'd3');
SysCall MOS_DOSBase 246;

function WaitPkt: PDosPacket;
SysCall MOS_DOSBase 252;

procedure ReplyPkt(dp  : PDosPacket location 'd1';
                   res1: LongInt    location 'd2';
                   res2: LongInt    location 'd3');
SysCall MOS_DOSBase 258;

procedure AbortPkt(port: PMsgPort   location 'd1';
                   pkt : PDosPacket location 'd2');
SysCall MOS_DOSBase 264;

function LockRecord(fh     : BPTR  location 'd1';
                    offset : Cardinal location 'd2';
                    length : Cardinal location 'd3';
                    mode   : Cardinal location 'd4';
                    timeout: Cardinal location 'd5'): LongBool;
SysCall MOS_DOSBase 270;

function LockRecords(recArray: PRecordLock location 'd1';
                     timeout : Cardinal    location 'd2'): LongBool;
SysCall MOS_DOSBase 276;

function UnLockRecord(fh    : BPTR  location 'd1';
                      offset: Cardinal location 'd2';
                      length: Cardinal location 'd3'): LongBool;
SysCall MOS_DOSBase 282;

function UnLockRecords(recArray: PRecordLock location 'd1'): LongBool;
SysCall MOS_DOSBase 288;

function SelectInput(fh: BPTR location 'd1'): LongInt;
SysCall MOS_DOSBase 294;

function SelectOutput(fh: BPTR location 'd1'): LongInt;
SysCall MOS_DOSBase 300;

function FGetC(fh: BPTR location 'd1'): LongInt;
SysCall MOS_DOSBase 306;

function FPutC(fh: BPTR location 'd1';
               ch: LongInt location 'd2'): LongInt;
SysCall MOS_DOSBase 312;

function UnGetC(fh       : BPTR location 'd1';
                character: LongInt location 'd2'): LongInt;
SysCall MOS_DOSBase 318;

function FRead(fh      : BPTR  location 'd1';
               block   : Pointer  location 'd2';
               blocklen: Cardinal location 'd3';
               number  : Cardinal location 'd4'): LongInt;
SysCall MOS_DOSBase 324;

function FWrite(fh      : BPTR  location 'd1';
                block   : Pointer  location 'd2';
                blocklen: Cardinal location 'd3';
                number  : Cardinal location 'd4'): LongInt;
SysCall MOS_DOSBase 330;

function FGets(fh    : BPTR  location 'd1';
               buf   : PChar    location 'd2';
               buflen: Cardinal location 'd3'): PChar;
SysCall MOS_DOSBase 336;

function FPuts(fh : BPTR location 'd1';
               str: PChar   location 'd2'): LongInt;
SysCall MOS_DOSBase 342;

procedure VFWritef(fh      : BPTR location 'd1';
                   format  : PChar   location 'd2';
                   argarray: Pointer location 'd3');
SysCall MOS_DOSBase 348;

function VFPrintf(fh      : BPTR location 'd1';
                  format  : PChar   location 'd2';
                  argarray: PLongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 354;

function dosFlush(fh: BPTR location 'd1'): LongInt;
SysCall MOS_DOSBase 360;

function SetVBuf(fh   : BPTR location 'd1';
                 buff : PChar   location 'd2';
                 type1: LongInt location 'd3';
                 size : LongInt location 'd4'): LongInt;
SysCall MOS_DOSBase 366;

function DupLockFromFH(fh: BPTR location 'd1'): BPTR;
SysCall MOS_DOSBase 372;

function OpenFromLock(lock: BPTR location 'd1'): BPTR;
SysCall MOS_DOSBase 378;

function ParentOfFH(fh: BPTR location 'd1'): BPTR;
SysCall MOS_DOSBase 384;

function ExamineFH(fh : BPTR        location 'd1';
                   fib: PFileInfoBlock location 'd2'): LongBool;
SysCall MOS_DOSBase 390;

function SetFileDate(name: PChar      location 'd1';
                     date: PDateStamp location 'd2'): LongBool;
SysCall MOS_DOSBase 396;

function NameFromLock(lock  : BPTR location 'd1';
                      buffer: PChar   location 'd2';
                      len   : LongInt location 'd3'): LongBool;
SysCall MOS_DOSBase 402;

function NameFromFH(fh    : BPTR location 'd1';
                    buffer: PChar   location 'd2';
                    len   : LongInt location 'd3'): LongBool;
SysCall MOS_DOSBase 408;

function SplitName(name     : PChar    location 'd1';
                   separator: Cardinal location 'd2';
                   buf      : PChar    location 'd3';
                   oldpos   : LongInt  location 'd4';
                   size     : LongInt  location 'd5'): SmallInt;
SysCall MOS_DOSBase 414;

function SameLock(lock1: BPTR location 'd1';
                  lock2: BPTR location 'd2'): LongInt;
SysCall MOS_DOSBase 420;

function SetMode(fh  : BPTR location 'd1';
                 mode: LongInt location 'd2'): LongInt;
SysCall MOS_DOSBase 426;

function ExAll(lock   : BPTR       location 'd1';
               buffer : PExAllData    location 'd2';
               size   : LongInt       location 'd3';
               data   : LongInt       location 'd4';
               control: PExAllControl location 'd5'): LongBool;
SysCall MOS_DOSBase 432;

function ReadLink(port  : PMsgPort location 'd1';
                  lock  : BPTR  location 'd2';
                  path  : PChar    location 'd3';
                  buffer: PChar    location 'd4';
                  size  : Cardinal location 'd5'): LongBool;
SysCall MOS_DOSBase 438;

function MakeLink(name: PChar   location 'd1';
                  dest: LongInt location 'd2';
                  soft: LongInt location 'd3'): LongBool;
SysCall MOS_DOSBase 444;

function ChangeMode(type1  : LongInt location 'd1';
                    fh     : BPTR location 'd2';
                    newmode: LongInt location 'd3'): LongBool;
SysCall MOS_DOSBase 450;

function SetFileSize(fh  : BPTR location 'd1';
                     pos : LongInt location 'd2';
                     mode: LongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 456;

function SetIoErr(result: LongInt location 'd1'): LongInt;
SysCall MOS_DOSBase 462;

function Fault(code  : LongInt location 'd1';
               header: PChar   location 'd2';
               buffer: PChar   location 'd3';
               len   : LongInt location 'd4'): LongBool;
SysCall MOS_DOSBase 468;

function PrintFault(code  : LongInt location 'd1';
                    header: PChar   location 'd2'): LongBool;
SysCall MOS_DOSBase 474;

function ErrorReport(code  : LongInt  location 'd1';
                     type1 : LongInt  location 'd2';
                     arg1  : Cardinal location 'd3';
                     device: PMsgPort location 'd4'): LongBool;
SysCall MOS_DOSBase 480;

function Cli: PCommandLineInterface;
SysCall MOS_DOSBase 492;

function CreateNewProc(tags: PTagItem location 'd1'): PProcess;
SysCall MOS_DOSBase 498;

function CreateNewProcTagList(tags: PTagItem location 'd1'): PProcess;
SysCall MOS_DOSBase 498;

function RunCommand(seg     : BPTR location 'd1';
                    stack   : LongInt location 'd2';
                    paramptr: PChar   location 'd3';
                    paramlen: LongInt location 'd4'): LongInt;
SysCall MOS_DOSBase 504;

function GetConsoleTask: PMsgPort;
SysCall MOS_DOSBase 510;

function SetConsoleTask(task: PMsgPort location 'd1'): PMsgPort;
SysCall MOS_DOSBase 516;

function GetFileSysTask: PMsgPort;
SysCall MOS_DOSBase 522;

function SetFileSysTask(task: PMsgPort location 'd1'): PMsgPort;
SysCall MOS_DOSBase 528;

function GetArgStr: PChar;
SysCall MOS_DOSBase 534;

function SetArgStr(str: PChar location 'd1'): LongBool;
SysCall MOS_DOSBase 540;

function FindCliProc(num: Cardinal location 'd1'): PProcess;
SysCall MOS_DOSBase 546;

function MaxCli: Cardinal;
SysCall MOS_DOSBase 552;

function SetCurrentDirName(name: PChar location 'd1'): LongBool;
SysCall MOS_DOSBase 558;

function GetCurrentDirName(buf: PChar   location 'd1';
                           len: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 564;

function SetProgramName(name: PChar location 'd1'): LongBool;
SysCall MOS_DOSBase 570;

function GetProgramName(buf: PChar   location 'd1';
                        len: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 576;

function SetPrompt(name: PChar location 'd1'): LongBool;
SysCall MOS_DOSBase 582;

function GetPrompt(buf: PChar   location 'd1';
                   len: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 588;

function SetProgramDir(lock: BPTR location 'd1'): LongInt;
SysCall MOS_DOSBase 594;

function GetProgramDir: BPTR;
SysCall MOS_DOSBase 600;

function SystemTagList(command: PChar    location 'd1';
                       tags   : PTagItem location 'd2'): LongInt;
SysCall MOS_DOSBase 606;

function dosSystem(command: PChar    location 'd1';
                   tags   : PTagItem location 'd2'): LongInt;
SysCall MOS_DOSBase 606;

function AssignLock(name: PChar   location 'd1';
                    lock: BPTR location 'd2'): LongBool;
SysCall MOS_DOSBase 612;

function AssignLate(name: PChar location 'd1';
                    path: PChar location 'd2'): LongBool;
SysCall MOS_DOSBase 618;

function AssignPath(name: PChar location 'd1';
                    path: PChar location 'd2'): LongBool;
SysCall MOS_DOSBase 624;

function AssignAdd(name: PChar   location 'd1';
                   lock: BPTR location 'd2'): LongBool;
SysCall MOS_DOSBase 630;

function RemAssignList(name: PChar   location 'd1';
                       lock: BPTR location 'd2'): LongBool;
SysCall MOS_DOSBase 636;

function GetDeviceProc(name: PChar    location 'd1';
                       dp  : PDevProc location 'd2'): PDevProc;
SysCall MOS_DOSBase 642;

procedure FreeDeviceProc(dp: PDevProc location 'd1');
SysCall MOS_DOSBase 648;

function LockDosList(flags: Cardinal location 'd1'): PDosList;
SysCall MOS_DOSBase 654;

procedure UnLockDosList(flags: Cardinal location 'd1');
SysCall MOS_DOSBase 660;

function AttemptLockDosList(flags: Cardinal location 'd1'): PDosList;
SysCall MOS_DOSBase 666;

function RemDosEntry(dlist: PDosList location 'd1'): LongBool;
SysCall MOS_DOSBase 672;

function AddDosEntry(dlist: PDosList location 'd1'): LongInt;
SysCall MOS_DOSBase 678;

function FindDosEntry(dlist: PDosList location 'd1';
                      name : PChar    location 'd2';
                      flags: Cardinal location 'd3'): PDosList;
SysCall MOS_DOSBase 684;

function NextDosEntry(dlist: PDosList location 'd1';
                      flags: Cardinal location 'd2'): PDosList;
SysCall MOS_DOSBase 690;

function MakeDosEntry(name : PChar   location 'd1';
                      type1: LongInt location 'd2'): PDosList;
SysCall MOS_DOSBase 696;

procedure FreeDosEntry(dlist: PDosList location 'd1');
SysCall MOS_DOSBase 702;

function IsFileSystem(name: PChar location 'd1'): LongBool;
SysCall MOS_DOSBase 708;

function Format(filesystem: PChar    location 'd1';
                volumename: PChar    location 'd2';
                dostype   : Cardinal location 'd3'): LongBool;
SysCall MOS_DOSBase 714;

function Relabel(drive  : PChar location 'd1';
                 newname: PChar location 'd2'): LongBool;
SysCall MOS_DOSBase 720;

function Inhibit(name : PChar   location 'd1';
                 onoff: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 726;

function AddBuffers(name  : PChar   location 'd1';
                    number: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 732;

function CompareDates(date1: PDateStamp location 'd1';
                      date2: PDateStamp location 'd2'): LongInt;
SysCall MOS_DOSBase 738;

function DOSDateToStr(datetime: _PDateTime location 'd1'): LongBool;
SysCall MOS_DOSBase 744;

function DOSStrToDate(datetime: _PDateTime location 'd1'): LongBool;
SysCall MOS_DOSBase 750;

function InternalLoadSeg(fh           : BPTR location 'd0';
                         table        : LongInt location 'a0';
                         var funcarray: LongInt location 'a1';
                         var stack    : LongInt location 'a2'): LongInt;
SysCall MOS_DOSBase 756;

function NewLoadSeg(file1: PChar    location 'd1';
                    tags : PTagItem location 'd2'): LongInt;
SysCall MOS_DOSBase 768;

function NewLoadSegTagList(file1: PChar    location 'd1';
                           tags : PTagItem location 'd2'): LongInt;
SysCall MOS_DOSBase 768;

function AddSegment(name  : PChar   location 'd1';
                    seg   : BPTR location 'd2';
                    system: LongInt location 'd3'): LongBool;
SysCall MOS_DOSBase 774;

function FindSegment(name  : PChar    location 'd1';
                     seg   : PSegment location 'd2';
                     system: LongInt  location 'd3'): PSegment;
SysCall MOS_DOSBase 780;

function RemSegment(seg: PSegment location 'd1'): LongBool;
SysCall MOS_DOSBase 786;

function CheckSignal(mask: LongInt location 'd1'): LongInt;
SysCall MOS_DOSBase 792;

function ReadArgs(arg_template: PChar   location 'd1';
                  var array1  : LongInt location 'd2';
                  args        : PRDArgs location 'd3'): PRDArgs;
SysCall MOS_DOSBase 798;

function FindArg(keyword     : PChar location 'd1';
                 arg_template: PChar location 'd2'): LongInt;
SysCall MOS_DOSBase 804;

function ReadItem(name    : PChar    location 'd1';
                  maxchars: LongInt  location 'd2';
                  cSource : PCSource location 'd3'): LongInt;
SysCall MOS_DOSBase 810;

function StrToLong(string1  : PChar   location 'd1';
                   var value: LongInt location 'd2'): LongInt;
SysCall MOS_DOSBase 816;

function MatchFirst(pat   : PChar       location 'd1';
                    anchor: PAnchorPath location 'd2'): LongInt;
SysCall MOS_DOSBase 822;

function MatchNext(anchor: PAnchorPath location 'd1'): LongInt;
SysCall MOS_DOSBase 828;

procedure MatchEnd(anchor: PAnchorPath location 'd1');
SysCall MOS_DOSBase 834;

function ParsePattern(pat   : PChar   location 'd1';
                      buf   : PChar   location 'd2';
                      buflen: LongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 840;

function MatchPattern(pat: PChar location 'd1';
                      str: PChar location 'd2'): LongBool;
SysCall MOS_DOSBase 846;

procedure FreeArgs(args: pRDArgs location 'd1');
SysCall MOS_DOSBase 858;

function FilePart(path: PChar location 'd1'): PChar;
SysCall MOS_DOSBase 870;

function PathPart(path: PChar location 'd1'): PChar;
SysCall MOS_DOSBase 876;

function AddPart(dirname: PChar    location 'd1';
                filename: PChar    location 'd2';
                size    : Cardinal location 'd3'): LongBool;
SysCall MOS_DOSBase 882;

function StartNotify(notify: PNotifyRequest location 'd1'): LongBool;
SysCall MOS_DOSBase 888;

procedure EndNotify(notify: PNotifyRequest location 'd1');
SysCall MOS_DOSBase 894;

function SetVar(name  : PChar   location 'd1';
                buffer: PChar   location 'd2';
                size  : LongInt location 'd3';
                flags : LongInt location 'd4'): LongBool;
SysCall MOS_DOSBase 900;

function GetVar(name  : PChar   location 'd1';
                buffer: PChar   location 'd2';
                size  : LongInt location 'd3';
                flags : LongInt location 'd4'): LongInt;
SysCall MOS_DOSBase 906;

function DeleteVar(name : PChar    location 'd1';
                   flags: Cardinal location 'd2'): LongBool;
SysCall MOS_DOSBase 912;

function FindVar(name : PChar    location 'd1';
                 type1: Cardinal location 'd2'): PLocalVar;
SysCall MOS_DOSBase 918;

function CliInitNewcli(dp: PDosPacket location 'a0'): LongInt;
SysCall MOS_DOSBase 930;

function CliInitRun(dp: PDosPacket location 'a0'): LongInt;
SysCall MOS_DOSBase 936;

function WriteChars(buf   : PChar    location 'd1';
                    buflen: Cardinal location 'd2'): LongInt;
SysCall MOS_DOSBase 942;

function PutStr(str: PChar location 'd1'): LongInt;
SysCall MOS_DOSBase 948;

function VPrintf(format  : PChar   location 'd1';
                 argarray: Pointer location 'd2'): LongInt;
SysCall MOS_DOSBase 954;

function ParsePatternNoCase(pat   : PChar   location 'd1';
                            buf   : PChar   location 'd2';
                            buflen: LongInt location 'd3'): LongInt;
SysCall MOS_DOSBase 966;

function MatchPatternNoCase(pat: PChar location 'd1';
                            str: PChar location 'd2'): LongBool;
SysCall MOS_DOSBase 972;

function SameDevice(lock1: BPTR location 'd1';
                    lock2: BPTR location 'd2'): LongBool;
SysCall MOS_DOSBase 984;

procedure ExAllEnd(lock   : LongInt       location 'd1';
                   buffer : PExAllData    location 'd2';
                   size   : LongInt       location 'd3';
                   data   : LongInt       location 'd4';
                   control: PExAllControl location 'd5');
SysCall MOS_DOSBase 990;

function SetOwner(name      : PChar   location 'd1';
                  owner_info: LongInt location 'd2'): LongBool;
SysCall MOS_DOSBase 996;

function AddSegmentTagList(tags: PTagItem location 'a0'): LongInt;
SysCall MOS_DOSBase 1002;

function FindSegmentTagList(tags: PTagItem location 'a0'): PSegment;
SysCall MOS_DOSBase 1008;

// MorphOS Specific

function Seek64(fh: BPTR; Position: Int64; Mode: LongInt): Int64; syscall BaseSysV MOS_DOSBase 1066;
function SetFileSize64(fh: BPTR; Pos: Int64; Mode: LongInt): Int64; syscall BaseSysV MOS_DOSBase 1072;
function LockRecord64(fh: BPTR; Offset, Length: QWord; Mode, TimeOut: LongWord): LongInt; syscall BaseSysV MOS_DOSBase 1078;
function LockRecords64(RecArray: TRecordLock64; TimeOut: LongWord): LongInt; syscall BaseSysV MOS_DOSBase 1084;
function UnLockRecord64(Fh: BPTR; Offset, Length: QWord): LongInt; syscall BaseSysV MOS_DOSBase 1090;
function UnLockRecords64(RecArray: TRecordLock64): LongInt; syscall BaseSysV MOS_DOSBase 1096;
function NewReadLink(Port: PMsgPort; Lock: BPTR; Path: STRPTR; Buffer: PByte; Size: LongInt): LongInt; syscall BaseSysV MOS_DOSBase 1114;
function GetFileSysAttr(DeviceName: STRPTR; Attr: LongInt; Storage: APTR; StorageSize: LongInt): LongInt; syscall BaseSysV MOS_DOSBase 1120;
function GetSegListAttr(SegList: BPTR; Attr: LongInt; Storage: APTR; StorageSize: LongInt): LongInt; syscall BaseSysV MOS_DOSBase 1126;
function SetDosObjectAttr(Type_: LongWord; Ptr: APTR; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1132;
function GetDosObjectAttr(Type_: LongWord; Ptr: APTR; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1138;
function Examine64(Lock: BPTR; Fib: PFileInfoBlock; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1144;
function Examine64TagList(Lock: BPTR; Fib: PFileInfoBlock; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1144;
function ExNext64(Lock: BPTR; Fib: PFileInfoBlock; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1150;
function ExNext64TagList(Lock: BPTR; Fib: PFileInfoBlock; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1150;
function ExamineFH64(Fh: BPTR; Fib: PFileInfoBlock; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1156;
function ExamineFH64TagList(Fh: BPTR; Fib: PFileInfoBlock; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1156;
// V51.51
procedure ReleaseCLINumber(CLINum: LongInt); syscall BaseSysV MOS_DOSBase 1162;
function QueryCLIDataTagList(Tags: PTagItem): PCLIData; syscall BaseSysV MOS_DOSBase 1168;
procedure FreeCLIData(Data: PCLIData); syscall BaseSysV MOS_DOSBase 1174;
// V51.52
function GetSegListAttrTagList(SegList: BPTR; Attr: LongInt; Storage: APTR; StorageSize: LongInt; Tags: PTagItem): LongInt; syscall BaseSysV MOS_DOSBase 1180;


{ * dos global definitions (V50)
  *********************************************************************
  * }

function BADDR(x: BPTR): Pointer; Inline;
function MKBADDR(x: Pointer): BPTR; Inline;


{ * dos stdio definitions
  *********************************************************************
  * }

function ReadChar: LongInt; Inline;
function WriteChar(ch: Char): LongInt; Inline;
function UnReadChar(ch: Char): LongInt; Inline;
function ReadChars(buf: Pointer; num: LongInt): LongInt; Inline;
function dosReadLn(buf: PChar; num: LongInt): PChar; Inline;
function dosWriteStr(str: PChar): LongInt; Inline;
procedure VWritef(format: PChar; argv: Pointer); Inline;


{ * calls with tags workarounds (should be removed later)
  *********************************************************************
  * }

function CreateNewProcTags(tags: array of dword): PProcess; Inline;
function AllocDosObjectTags(type1: Cardinal; Tags: array of DWord): Pointer; inline;
function SystemTags(command: PChar; Tags: array of DWord): LongInt; Inline;

function SetDosObjectAttrTagList(Type_: LongWord; Ptr: APTR; const Tags: array of PtrUInt): LongInt; inline;
function GetDosObjectAttrTagList(Type_: LongWord; Ptr: APTR; const Tags: array of PtrUInt): LongInt; inline;
function Examine64Tags(Lock: BPTR; Fib: PFileInfoBlock; const Tags: array of PtrUInt): LongInt; inline;
function ExNext64Tags(Lock: BPTR; Fib: PFileInfoBlock; const Tags: array of PtrUInt): LongInt; inline;
function ExamineFH64Tags(Fh: BPTR; Fib: PFileInfoBlock; const Tags: array of PtrUInt): LongInt; inline;

function QueryCLIDataTags(const Tags: array of PtrUInt): PCLIData; inline;
function GetSegListAttrTags(SegList: BPTR; Attr: LongInt; Storage: APTR; StorageSize: LongInt; const Tags: array of PtrUInt): LongInt; inline;

implementation


{ * dos stdio definitions
  *********************************************************************
  * }

function ReadChar: LongInt; Inline;
begin
  ReadChar:=FGetC(dosInput);
end;

function WriteChar(ch: Char): LongInt; Inline;
begin
  WriteChar:=FPutC(dosOutput,Byte(ch));
end;

function UnReadChar(ch: Char): LongInt; Inline;
begin
  UnReadChar:=UnGetC(dosInput,Byte(ch));
end;

function ReadChars(buf: Pointer; num: LongInt): LongInt; Inline;
begin
  ReadChars:=FRead(dosInput,buf,1,num);
end;

function dosReadLn(buf: PChar; num: LongInt): PChar; Inline;
begin
  dosReadLn:=FGets(dosInput,buf,num);
end;

function dosWriteStr(str: PChar): LongInt; Inline;
begin
  dosWriteStr:=FPuts(dosOutput,str);
end;

procedure VWritef(format: PChar; argv: Pointer); Inline;
begin
  VFWritef(dosOutput,format,argv);
end;



{ * dos global definitions (V50)
  *********************************************************************
  * }


function BADDR(x: BPTR): Pointer; Inline;
begin
 BADDR:=Pointer(x Shl 2);
end;

function MKBADDR(x: Pointer): BPTR; Inline;
begin
 MKBADDR:=LongInt(PtrUInt(x)) Shr 2;
end;



{ * calls with tags workarounds (should be removed later)
  *********************************************************************
  * }

function CreateNewProcTags(tags: array of DWord): PProcess; Inline;
begin
  CreateNewProcTags:=CreateNewProc(@tags);
end;

function AllocDosObjectTags(type1: Cardinal; Tags: array of DWord): Pointer; inline;
begin
  AllocDosObjectTags := AllocDosObject(type1, @Tags);
end;

function SystemTags(command: PChar; Tags: array of DWord): LongInt;
begin
  SystemTags := SystemTagList(Command, @Tags);
end;

function SetDosObjectAttrTagList(Type_: LongWord; Ptr: APTR; const Tags: array of PtrUInt): LongInt;
begin
  SetDosObjectAttrTagList := SetDosObjectAttr(Type_, Ptr, @Tags);
end;

function GetDosObjectAttrTagList(Type_: LongWord; Ptr: APTR; const Tags: array of PtrUInt): LongInt;
begin
  GetDosObjectAttrTagList := GetDosObjectAttr(Type_, Ptr, @Tags);
end;

function Examine64Tags(Lock: BPTR; Fib: PFileInfoBlock; const Tags: array of PtrUInt): LongInt;
begin
  Examine64Tags := Examine64(Lock, Fib, @Tags);
end;

function ExNext64Tags(Lock: BPTR; Fib: PFileInfoBlock; const Tags: array of PtrUInt): LongInt;
begin
  ExNext64Tags := ExNext64(Lock, Fib, @Tags);
end;

function ExamineFH64Tags(Fh: BPTR; Fib: PFileInfoBlock; const Tags: array of PtrUInt): LongInt;
begin
  ExamineFH64Tags := ExamineFH64(Fh, Fib, @Tags);
end;

function QueryCLIDataTags(const Tags: array of PtrUInt): PCLIData;
begin
  QueryCLIDataTags := QueryCLIDataTagList(@Tags);
end;

function GetSegListAttrTags(SegList: BPTR; Attr: LongInt; Storage: APTR; StorageSize: LongInt; const Tags: array of PtrUInt): LongInt; inline;
begin
  GetSegListAttrTags := GetSegListAttrTagList(SegList, Attr, Storage, StorageSize, @Tags);
end;


begin
  DosBase:=MOS_DOSBase;
end.
