{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
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
    14 Jul 2000.

    Changed ReadArgs, removed the var for the second arg.
    Changed DOSRename from longint to a boolean.
    Aug 04 2000.

    Added functions and procedures with array of const.
    For use with fpc 1.0.7

    You have to use systemvartags. Check out that unit.
    09 Nov 2002.

    Added the define use_amiga_smartlink.
    13 Jan 2003.

    Update for AmigaOS 3.9.
    Added some const.
    26 Jan 2003.

    Changed integer > smallint.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}


{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit amigados;

INTERFACE

uses exec, utility, timer;

Const

{ Predefined Amiga DOS global constants }

    DOSTRUE     = -1;
    DOSFALSE    =  0;

{ Mode parameter to Open() }

    MODE_OLDFILE        = 1005;         { Open existing file read/write
                                          positioned at beginning of file. }
    MODE_NEWFILE        = 1006;         { Open freshly created file (delete
                                          old file) read/write }
    MODE_READWRITE      = 1004;         { Open old file w/exclusive lock }

{ Relative position to Seek() }

    OFFSET_BEGINNING    = -1;           { relative to Begining Of File }
    OFFSET_CURRENT      = 0;            { relative to Current file position }
    OFFSET_END          = 1;            { relative to End Of File }

    BITSPERBYTE         = 8;
    BYTESPERLONG        = 4;
    BITSPERLONG         = 32;
    MAXINT              = $7FFFFFFF;
    MININT              = $80000000;

{ Passed as type to Lock() }

    SHARED_LOCK         = -2;           { File is readable by others }
    ACCESS_READ         = -2;           { Synonym }
    EXCLUSIVE_LOCK      = -1;           { No other access allowed }
    ACCESS_WRITE        = -1;           { Synonym }

Type

    FileHandle  = BPTR;
    FileLock    = BPTR;

    pDateStamp = ^tDateStamp;
    tDateStamp = record
        ds_Days         : Longint;      { Number of days since Jan. 1, 1978 }
        ds_Minute       : Longint;      { Number of minutes past midnight }
        ds_Tick         : Longint;      { Number of ticks past minute }
    end;

Const

    TICKS_PER_SECOND    = 50;           { Number of ticks in one second }

{$PACKRECORDS 4}
Type

{ Returned by Examine() and ExInfo(), must be on a 4 byte boundary }

    pFileInfoBlock = ^tFileInfoBlock;
    tFileInfoBlock = record
        fib_DiskKey      : Longint;
        fib_DirEntryType : Longint;
                        { Type of Directory. If < 0, then a plain file.
                          If > 0 a directory }
        fib_FileName     : Array [0..107] of Char;
                        { Null terminated. Max 30 chars used for now }
        fib_Protection   : Longint;
                        { bit mask of protection, rwxd are 3-0. }
        fib_EntryType    : Longint;
        fib_Size         : Longint;      { Number of bytes in file }
        fib_NumBlocks    : Longint;      { Number of blocks in file }
        fib_Date         : tDateStamp;   { Date file last changed }
        fib_Comment      : Array [0..79] of Char;
                        { Null terminated comment associated with file }
        fib_OwnerUID     : Word;
        fib_OwnerGID     : Word;
        fib_Reserved     : Array [0..31] of Char;
    end;

Const

{ FIB stands for FileInfoBlock }

{* FIBB are bit definitions, FIBF are field definitions *}
{* Regular RWED bits are 0 == allowed. *}
{* NOTE: GRP and OTR RWED permissions are 0 == not allowed! *}
{* Group and Other permissions are not directly handled by the filesystem *}

    FIBB_OTR_READ       = 15;   {* Other: file is readable *}
    FIBB_OTR_WRITE      = 14;   {* Other: file is writable *}
    FIBB_OTR_EXECUTE    = 13;   {* Other: file is executable *}
    FIBB_OTR_DELETE     = 12;   {* Other: prevent file from being deleted *}
    FIBB_GRP_READ       = 11;   {* Group: file is readable *}
    FIBB_GRP_WRITE      = 10;   {* Group: file is writable *}
    FIBB_GRP_EXECUTE    = 9;    {* Group: file is executable *}
    FIBB_GRP_DELETE     = 8;    {* Group: prevent file from being deleted *}

    FIBB_SCRIPT         = 6;    { program is a script (execute) file }
    FIBB_PURE           = 5;    { program is reentrant and rexecutable}
    FIBB_ARCHIVE        = 4;    { cleared whenever file is changed }
    FIBB_READ           = 3;    { ignored by old filesystem }
    FIBB_WRITE          = 2;    { ignored by old filesystem }
    FIBB_EXECUTE        = 1;    { ignored by system, used by Shell }
    FIBB_DELETE         = 0;    { prevent file from being deleted }

    FIBF_OTR_READ      = (1 shl FIBB_OTR_READ);
    FIBF_OTR_WRITE     = (1 shl FIBB_OTR_WRITE);
    FIBF_OTR_EXECUTE   = (1 shl FIBB_OTR_EXECUTE);
    FIBF_OTR_DELETE    = (1 shl FIBB_OTR_DELETE);
    FIBF_GRP_READ      = (1 shl FIBB_GRP_READ);
    FIBF_GRP_WRITE     = (1 shl FIBB_GRP_WRITE);
    FIBF_GRP_EXECUTE   = (1 shl FIBB_GRP_EXECUTE);
    FIBF_GRP_DELETE    = (1 shl FIBB_GRP_DELETE);

    FIBF_SCRIPT         = 64;
    FIBF_PURE           = 32;
    FIBF_ARCHIVE        = 16;
    FIBF_READ           = 8;
    FIBF_WRITE          = 4;
    FIBF_EXECUTE        = 2;
    FIBF_DELETE         = 1;

{* Standard maximum length for an error string from fault.  However, most *}
{* error strings should be kept under 60 characters if possible.  Don't   *}
{* forget space for the header you pass in. *}

    FAULT_MAX  = 82;

{* All BCPL data must be long Integer aligned.  BCPL pointers are the long Integer
 *  address (i.e byte address divided by 4 (>>2)) *}

{* BCPL strings have a length in the first byte and then the characters.
 * For example:  s[0]=3 s[1]=S s[2]=Y s[3]=S                 *}


Type

{ returned by Info(), must be on a 4 byte boundary }

    pInfoData = ^tInfoData;
    tInfoData = record
        id_NumSoftErrors        : Longint;      { number of soft errors on disk }
        id_UnitNumber           : Longint;      { Which unit disk is (was) mounted on }
        id_DiskState            : Longint;      { See defines below }
        id_NumBlocks            : Longint;      { Number of blocks on disk }
        id_NumBlocksUsed        : Longint;      { Number of block in use }
        id_BytesPerBlock        : Longint;
        id_DiskType             : Longint;      { Disk Type code }
        id_VolumeNode           : BPTR;         { BCPL pointer to volume node }
        id_InUse                : Longint;      { Flag, zero if not in use }
    end;

{$PACKRECORDS NORMAL}

Const

{ ID stands for InfoData }

        { Disk states }

    ID_WRITE_PROTECTED  = 80;   { Disk is write protected }
    ID_VALIDATING       = 81;   { Disk is currently being validated }
    ID_VALIDATED        = 82;   { Disk is consistent and writeable }

CONST
 ID_NO_DISK_PRESENT     = -1;
 ID_UNREADABLE_DISK     = $42414400;   { 'BAD\0' }
 ID_DOS_DISK            = $444F5300;   { 'DOS\0' }
 ID_FFS_DISK            = $444F5301;   { 'DOS\1' }
 ID_NOT_REALLY_DOS      = $4E444F53;   { 'NDOS'  }
 ID_KICKSTART_DISK      = $4B49434B;   { 'KICK'  }
 ID_MSDOS_DISK          = $4d534400;   { 'MSD\0' }

{ Errors from IoErr(), etc. }
 ERROR_NO_FREE_STORE              = 103;
 ERROR_TASK_TABLE_FULL            = 105;
 ERROR_BAD_TEMPLATE               = 114;
 ERROR_BAD_NUMBER                 = 115;
 ERROR_REQUIRED_ARG_MISSING       = 116;
 ERROR_KEY_NEEDS_ARG              = 117;
 ERROR_TOO_MANY_ARGS              = 118;
 ERROR_UNMATCHED_QUOTES           = 119;
 ERROR_LINE_TOO_LONG              = 120;
 ERROR_FILE_NOT_OBJECT            = 121;
 ERROR_INVALID_RESIDENT_LIBRARY   = 122;
 ERROR_NO_DEFAULT_DIR             = 201;
 ERROR_OBJECT_IN_USE              = 202;
 ERROR_OBJECT_EXISTS              = 203;
 ERROR_DIR_NOT_FOUND              = 204;
 ERROR_OBJECT_NOT_FOUND           = 205;
 ERROR_BAD_STREAM_NAME            = 206;
 ERROR_OBJECT_TOO_LARGE           = 207;
 ERROR_ACTION_NOT_KNOWN           = 209;
 ERROR_INVALID_COMPONENT_NAME     = 210;
 ERROR_INVALID_LOCK               = 211;
 ERROR_OBJECT_WRONG_TYPE          = 212;
 ERROR_DISK_NOT_VALIDATED         = 213;
 ERROR_DISK_WRITE_PROTECTED       = 214;
 ERROR_RENAME_ACROSS_DEVICES      = 215;
 ERROR_DIRECTORY_NOT_EMPTY        = 216;
 ERROR_TOO_MANY_LEVELS            = 217;
 ERROR_DEVICE_NOT_MOUNTED         = 218;
 ERROR_SEEK_ERROR                 = 219;
 ERROR_COMMENT_TOO_BIG            = 220;
 ERROR_DISK_FULL                  = 221;
 ERROR_DELETE_PROTECTED           = 222;
 ERROR_WRITE_PROTECTED            = 223;
 ERROR_READ_PROTECTED             = 224;
 ERROR_NOT_A_DOS_DISK             = 225;
 ERROR_NO_DISK                    = 226;
 ERROR_NO_MORE_ENTRIES            = 232;
{ added for 1.4 }
 ERROR_IS_SOFT_LINK               = 233;
 ERROR_OBJECT_LINKED              = 234;
 ERROR_BAD_HUNK                   = 235;
 ERROR_NOT_IMPLEMENTED            = 236;
 ERROR_RECORD_NOT_LOCKED          = 240;
 ERROR_LOCK_COLLISION             = 241;
 ERROR_LOCK_TIMEOUT               = 242;
 ERROR_UNLOCK_ERROR               = 243;

{ error codes 303-305 are defined in dosasl.h }

{ These are the return codes used by convention by AmigaDOS commands }
{ See FAILAT and IF for relvance to EXECUTE files                    }
 RETURN_OK                        =   0;  { No problems, success }
 RETURN_WARN                      =   5;  { A warning only }
 RETURN_ERROR                     =  10;  { Something wrong }
 RETURN_FAIL                      =  20;  { Complete or severe failure}

{ Bit numbers that signal you that a user has issued a break }
 SIGBREAKB_CTRL_C   = 12;
 SIGBREAKB_CTRL_D   = 13;
 SIGBREAKB_CTRL_E   = 14;
 SIGBREAKB_CTRL_F   = 15;

{ Bit fields that signal you that a user has issued a break }
{ for example:  if (SetSignal(0,0) & SIGBREAKF_CTRL_C) cleanup_and_exit(); }
 SIGBREAKF_CTRL_C   = 4096;
 SIGBREAKF_CTRL_D   = 8192;
 SIGBREAKF_CTRL_E   = 16384;
 SIGBREAKF_CTRL_F   = 32768;

{ Values returned by SameLock() }
 LOCK_SAME             =  0;
 LOCK_SAME_HANDLER     =  1;       { actually same volume }
 LOCK_DIFFERENT        =  -1;

{ types for ChangeMode() }
 CHANGE_LOCK    = 0;
 CHANGE_FH      = 1;

{ Values for MakeLink() }
 LINK_HARD      = 0;
 LINK_SOFT      = 1;       { softlinks are not fully supported yet }

{ values returned by  }
 ITEM_EQUAL     = -2;              { "=" Symbol }
 ITEM_ERROR     = -1;              { error }
 ITEM_NOTHING   = 0;               { *N, ;, endstreamch }
 ITEM_UNQUOTED  = 1;               { unquoted item }
 ITEM_QUOTED    = 2;               { quoted item }

{ types for AllocDosObject/FreeDosObject }
 DOS_FILEHANDLE        =  0;       { few people should use this }
 DOS_EXALLCONTROL      =  1;       { Must be used to allocate this! }
 DOS_FIB               =  2;       { useful }
 DOS_STDPKT            =  3;       { for doing packet-level I/O }
 DOS_CLI               =  4;       { for shell-writers, etc }
 DOS_RDARGS            =  5;       { for ReadArgs if you pass it in }


{
 *      Data structures and equates used by the V1.4 DOS functions
 * StrtoDate() and DatetoStr()
 }

{--------- String/Date structures etc }
Type
       pDateTime = ^tDateTime;
       tDateTime = record
        dat_Stamp   : tDateStamp;      { DOS DateStamp }
        dat_Format,                    { controls appearance of dat_StrDate }
        dat_Flags   : Byte;           { see BITDEF's below }
        dat_StrDay,                    { day of the week string }
        dat_StrDate,                   { date string }
        dat_StrTime : STRPTR;          { time string }
       END;

{ You need this much room for each of the DateTime strings: }
CONST
 LEN_DATSTRING =  16;

{      flags for dat_Flags }

 DTB_SUBST      = 0;               { substitute Today, Tomorrow, etc. }
 DTF_SUBST      = 1;
 DTB_FUTURE     = 1;               { day of the week is in future }
 DTF_FUTURE     = 2;

{
 *      date format values
 }

 FORMAT_DOS     = 0;               { dd-mmm-yy }
 FORMAT_INT     = 1;               { yy-mm-dd  }
 FORMAT_USA     = 2;               { mm-dd-yy  }
 FORMAT_CDN     = 3;               { dd-mm-yy  }
 FORMAT_MAX     = FORMAT_CDN;
 FORMAT_DEF     = 4;            { use default format, as defined
                                           by locale; if locale not
                                           available, use FORMAT_DOS
                                           instead }

{**********************************************************************
************************ PATTERN MATCHING ******************************
************************************************************************

* structure expected by MatchFirst, MatchNext.
* Allocate this structure and initialize it as follows:
*
* Set ap_BreakBits to the signal bits (CDEF) that you want to take a
* break on, or NULL, if you don't want to convenience the user.
*
* If you want to have the FULL PATH NAME of the files you found,
* allocate a buffer at the END of this structure, and put the size of
* it into ap_Strlen.  If you don't want the full path name, make sure
* you set ap_Strlen to zero.  In this case, the name of the file, and stats
* are available in the ap_Info, as per usual.
*
* Then call MatchFirst() and then afterwards, MatchNext() with this structure.
* You should check the return value each time (see below) and take the
* appropriate action, ultimately calling MatchEnd() when there are
* no more files and you are done.  You can tell when you are done by
* checking for the normal AmigaDOS return code ERROR_NO_MORE_ENTRIES.
*
}

Type
       pAChain = ^tAChain;
       tAChain = record
        an_Child,
        an_Parent   : pAChain;
        an_Lock     : BPTR;
        an_Info     : tFileInfoBlock;
        an_Flags    : Shortint;
        an_String   : Array[0..0] of Char;   { FIX!! }
       END;

       pAnchorPath = ^tAnchorPath;
       tAnchorPath = record
        case smallint of
        0 : (
        ap_First      : pAChain;
        ap_Last       : pAChain;
        );
        1 : (
        ap_Base,                    { pointer to first anchor }
        ap_Current    : pAChain;    { pointer to last anchor }
        ap_BreakBits,               { Bits we want to break on }
        ap_FoundBreak : Longint;    { Bits we broke on. Also returns ERROR_BREAK }
        ap_Flags      : Shortint;       { New use for extra Integer. }
        ap_Reserved   : Shortint;
        ap_Strlen     : smallint;       { This is what ap_Length used to be }
        ap_Info       : tFileInfoBlock;
        ap_Buf        : Array[0..0] of Char;     { Buffer for path name, allocated by user !! }
        { FIX! }
        );
       END;


CONST
    APB_DOWILD    =  0;       { User option ALL }
    APF_DOWILD    =  1;

    APB_ITSWILD   =  1;       { Set by MatchFirst, used by MatchNext  }
    APF_ITSWILD   =  2;       { Application can test APB_ITSWILD, too }
                                { (means that there's a wildcard        }
                                { in the pattern after calling          }
                                { MatchFirst).                          }

    APB_DODIR     =  2;       { Bit is SET IF a DIR node should be }
    APF_DODIR     =  4;       { entered. Application can RESET this }
                                { bit after MatchFirst/MatchNext to AVOID }
                                { entering a dir. }

    APB_DIDDIR    =  3;       { Bit is SET for an "expired" dir node. }
    APF_DIDDIR    =  8;

    APB_NOMEMERR  =  4;       { Set on memory error }
    APF_NOMEMERR  =  16;

    APB_DODOT     =  5;       { IF set, allow conversion of '.' to }
    APF_DODOT     =  32;      { CurrentDir }

    APB_DirChanged  = 6;       { ap_Current->an_Lock changed }
    APF_DirChanged  = 64;      { since last MatchNext call }


    DDB_PatternBit  = 0;
    DDF_PatternBit  = 1;
    DDB_ExaminedBit = 1;
    DDF_ExaminedBit = 2;
    DDB_Completed   = 2;
    DDF_Completed   = 4;
    DDB_AllBit      = 3;
    DDF_AllBit      = 8;
    DDB_Single      = 4;
    DDF_Single      = 16;

{
 * Constants used by wildcard routines, these are the pre-parsed tokens
 * referred to by pattern match.  It is not necessary for you to do
 * anything about these, MatchFirst() MatchNext() handle all these for you.
 }

    P_ANY         =  $80;    { Token for '*' or '#?  }
    P_SINGLE      =  $81;    { Token for '?' }
    P_ORSTART     =  $82;    { Token for '(' }
    P_ORNEXT      =  $83;    { Token for '|' }
    P_OREND       =  $84;    { Token for ')' }
    P_NOT         =  $85;    { Token for '~' }
    P_NOTEND      =  $86;    { Token for }
    P_NOTCLASS    =  $87;    { Token for '^' }
    P_CLASS       =  $88;    { Token for '[]' }
    P_REPBEG      =  $89;    { Token for '[' }
    P_REPEND      =  $8A;    { Token for ']' }
    P_STOP        =  $8B;    { token to force end of evaluation }

{ Values for an_Status, NOTE: These are the actual bit numbers }

    COMPLEX_BIT   =  1;       { Parsing complex pattern }
    EXAMINE_BIT   =  2;       { Searching directory }

{
 * Returns from MatchFirst(), MatchNext()
 * You can also get dos error returns, such as ERROR_NO_MORE_ENTRIES,
 * these are in the dos.h file.
 }

    ERROR_BUFFER_OVERFLOW  = 303;     { User OR internal buffer overflow }
    ERROR_BREAK            = 304;     { A break character was received }
    ERROR_NOT_EXECUTABLE   = 305;     { A file has E bit cleared }

{   hunk types }
     HUNK_UNIT      = 999 ;
     HUNK_NAME      = 1000;
     HUNK_CODE      = 1001;
     HUNK_DATA      = 1002;
     HUNK_BSS       = 1003;
     HUNK_RELOC32   = 1004;
     HUNK_RELOC16   = 1005;
     HUNK_RELOC8    = 1006;
     HUNK_EXT       = 1007;
     HUNK_SYMBOL    = 1008;
     HUNK_DEBUG     = 1009;
     HUNK_END       = 1010;
     HUNK_HEADER    = 1011;

     HUNK_OVERLAY   = 1013;
     HUNK_BREAK     = 1014;

     HUNK_DREL32    = 1015;
     HUNK_DREL16    = 1016;
     HUNK_DREL8     = 1017;

     HUNK_LIB       = 1018;
     HUNK_INDEX     = 1019;

{   hunk_ext sub-types }
     EXT_SYMB       = 0  ;     {   symbol table }
     EXT_DEF        = 1  ;     {   relocatable definition }
     EXT_ABS        = 2  ;     {   Absolute definition }
     EXT_RES        = 3  ;     {   no longer supported }
     EXT_REF32      = 129;     {   32 bit reference to symbol }
     EXT_COMMON     = 130;     {   32 bit reference to COMMON block }
     EXT_REF16      = 131;     {   16 bit reference to symbol }
     EXT_REF8       = 132;     {    8 bit reference to symbol }
     EXT_DEXT32     = 133;     {   32 bit data releative reference }
     EXT_DEXT16     = 134;     {   16 bit data releative reference }
     EXT_DEXT8      = 135;     {    8 bit data releative reference }


Type

{ All DOS processes have this structure }
{ Create and Device Proc returns pointer to the MsgPort in this structure }
{ dev_proc = Address(smallint(DeviceProc()) - SizeOf(Task)) }

    pProcess = ^tProcess;
    tProcess = record
        pr_Task         : tTask;
        pr_MsgPort      : tMsgPort;     { This is BPTR address from DOS functions  }
        pr_Pad          : smallint;         { Remaining variables on 4 byte boundaries }
        pr_SegList      : BPTR;         { Array of seg lists used by this process  }
        pr_StackSize    : Longint;      { Size of process stack in bytes            }
        pr_GlobVec      : Pointer;      { Global vector for this process (BCPL)    }
        pr_TaskNum      : Longint;      { CLI task number of zero if not a CLI      }
        pr_StackBase    : BPTR;         { Ptr to high memory end of process stack  }
        pr_Result2      : Longint;      { Value of secondary result from last call }
        pr_CurrentDir   : BPTR;         { Lock associated with current directory   }
        pr_CIS          : BPTR;         { Current CLI Input Stream                  }
        pr_COS          : BPTR;         { Current CLI Output Stream                 }
        pr_ConsoleTask  : Pointer;      { Console handler process for current window}
        pr_FileSystemTask : Pointer;    { File handler process for current drive   }
        pr_CLI          : BPTR;         { pointer to ConsoleLineInterpreter         }
        pr_ReturnAddr   : Pointer;      { pointer to previous stack frame           }
        pr_PktWait      : Pointer;      { Function to be called when awaiting msg  }
        pr_WindowPtr    : Pointer;      { Window for error printing }
        { following definitions are new with 2.0 }
        pr_HomeDir      : BPTR;         { Home directory of executing program      }
        pr_Flags        : Longint;      { flags telling dos about process          }
        pr_ExitCode     : Pointer;      { code to call on exit of program OR NULL  }
        pr_ExitData     : Longint;      { Passed as an argument to pr_ExitCode.    }
        pr_Arguments    : STRPTR;       { Arguments passed to the process at start }
        pr_LocalVars    : tMinList;      { Local environment variables             }
        pr_ShellPrivate : ULONG;        { for the use of the current shell         }
        pr_CES          : BPTR;         { Error stream - IF NULL, use pr_COS       }
    end;

{
 * Flags for pr_Flags
 }
CONST
 PRB_FREESEGLIST       =  0 ;
 PRF_FREESEGLIST       =  1 ;
 PRB_FREECURRDIR       =  1 ;
 PRF_FREECURRDIR       =  2 ;
 PRB_FREECLI           =  2 ;
 PRF_FREECLI           =  4 ;
 PRB_CLOSEINPUT        =  3 ;
 PRF_CLOSEINPUT        =  8 ;
 PRB_CLOSEOUTPUT       =  4 ;
 PRF_CLOSEOUTPUT       =  16;
 PRB_FREEARGS          =  5 ;
 PRF_FREEARGS          =  32;


{ The long smallint address (BPTR) of this structure is returned by
 * Open() and other routines that return a file.  You need only worry
 * about this struct to do async io's via PutMsg() instead of
 * standard file system calls }

Type

    pFileHandle = ^tFileHandle;
    tFileHandle = record
        fh_Link         : pMessage;   { EXEC message        }
        fh_Port         : pMsgPort;   { Reply port for the packet }
        fh_Type         : pMsgPort;   { Port to do PutMsg() to
                                          Address is negative if a plain file }
        fh_Buf          : Longint;
        fh_Pos          : Longint;
        fh_End          : Longint;
        fh_Func1        : Longint;
        fh_Func2        : Longint;
        fh_Func3        : Longint;
        fh_Arg1         : Longint;
        fh_Arg2         : Longint;
    end;

{ This is the extension to EXEC Messages used by DOS }

    pDosPacket = ^tDosPacket;
    tDosPacket = record
        dp_Link : pMessage;     { EXEC message        }
        dp_Port : pMsgPort;     { Reply port for the packet }
                                { Must be filled in each send. }
        case smallint of
        0 : (
        dp_Action : Longint;
        dp_Status : Longint;
        dp_Status2 : Longint;
        dp_BufAddr : Longint;
        );
        1 : (
        dp_Type : Longint;      { See ACTION_... below and
                                * 'R' means Read, 'W' means Write to the
                                * file system }
        dp_Res1 : Longint;      { For file system calls this is the result
                                * that would have been returned by the
                                * function, e.g. Write ('W') returns actual
                                * length written }
        dp_Res2 : Longint;      { For file system calls this is what would
                                * have been returned by IoErr() }
        dp_Arg1 : Longint;
        dp_Arg2 : Longint;
        dp_Arg3 : Longint;
        dp_Arg4 : Longint;
        dp_Arg5 : Longint;
        dp_Arg6 : Longint;
        dp_Arg7 : Longint;
        );
    end;


{ A Packet does not require the Message to be before it in memory, but
 * for convenience it is useful to associate the two.
 * Also see the function init_std_pkt for initializing this structure }

    pStandardPacket = ^tStandardPacket;
    tStandardPacket = record
        sp_Msg          : tMessage;
        sp_Pkt          : tDosPacket;
    end;


Const

{ Packet types }
    ACTION_NIL                  = 0;
    ACTION_GET_BLOCK            = 2;    { OBSOLETE }
    ACTION_SET_MAP              = 4;
    ACTION_DIE                  = 5;
    ACTION_EVENT                = 6;
    ACTION_CURRENT_VOLUME       = 7;
    ACTION_LOCATE_OBJECT        = 8;
    ACTION_RENAME_DISK          = 9;
    ACTION_WRITE                = $57;  { 'W' }
    ACTION_READ                 = $52;  { 'R' }
    ACTION_FREE_LOCK            = 15;
    ACTION_DELETE_OBJECT        = 16;
    ACTION_RENAME_OBJECT        = 17;
    ACTION_MORE_CACHE           = 18;
    ACTION_COPY_DIR             = 19;
    ACTION_WAIT_CHAR            = 20;
    ACTION_SET_PROTECT          = 21;
    ACTION_CREATE_DIR           = 22;
    ACTION_EXAMINE_OBJECT       = 23;
    ACTION_EXAMINE_NEXT         = 24;
    ACTION_DISK_INFO            = 25;
    ACTION_INFO                 = 26;
    ACTION_FLUSH                = 27;
    ACTION_SET_COMMENT          = 28;
    ACTION_PARENT               = 29;
    ACTION_TIMER                = 30;
    ACTION_INHIBIT              = 31;
    ACTION_DISK_TYPE            = 32;
    ACTION_DISK_CHANGE          = 33;
    ACTION_SET_DATE             = 34;

    ACTION_SCREEN_MODE          = 994;

    ACTION_READ_RETURN          = 1001;
    ACTION_WRITE_RETURN         = 1002;
    ACTION_SEEK                 = 1008;
    ACTION_FINDUPDATE           = 1004;
    ACTION_FINDINPUT            = 1005;
    ACTION_FINDOUTPUT           = 1006;
    ACTION_END                  = 1007;
    ACTION_TRUNCATE             = 1022; { fast file system only }
    ACTION_WRITE_PROTECT        = 1023; { fast file system only }

{ new 2.0 packets }
    ACTION_SAME_LOCK       = 40;
    ACTION_CHANGE_SIGNAL   = 995;
    ACTION_FORMAT          = 1020;
    ACTION_MAKE_LINK       = 1021;
{}
{}
    ACTION_READ_LINK       = 1024;
    ACTION_FH_FROM_LOCK    = 1026;
    ACTION_IS_FILESYSTEM   = 1027;
    ACTION_CHANGE_MODE     = 1028;
{}
    ACTION_COPY_DIR_FH     = 1030;
    ACTION_PARENT_FH       = 1031;
    ACTION_EXAMINE_ALL     = 1033;
    ACTION_EXAMINE_FH      = 1034;

    ACTION_LOCK_RECORD     = 2008;
    ACTION_FREE_RECORD     = 2009;

    ACTION_ADD_NOTIFY      = 4097;
    ACTION_REMOVE_NOTIFY   = 4098;

    {* Added in V39: *}
    ACTION_EXAMINE_ALL_END  = 1035;
    ACTION_SET_OWNER        = 1036;

{* Tell a file system to serialize the current volume. This is typically
 * done by changing the creation date of the disk. This packet does not take
 * any arguments.  NOTE: be prepared to handle failure of this packet for
 * V37 ROM filesystems.
 *}

    ACTION_SERIALIZE_DISK  = 4200;

{
 * A structure for holding error messages - stored as array with error == 0
 * for the last entry.
 }
Type
       pErrorString = ^tErrorString;
       tErrorString = record
        estr_Nums     : Pointer;
        estr_Strings  : Pointer;
       END;


{ DOS library node structure.
 * This is the data at positive offsets from the library node.
 * Negative offsets from the node is the jump table to DOS functions
 * node = (struct DosLibrary *) OpenLibrary( "dos.library" .. )      }

Type

    pDosLibrary = ^tDosLibrary;
    tDosLibrary = record
        dl_lib          : tLibrary;
        dl_Root         : Pointer;      { Pointer to RootNode, described below }
        dl_GV           : Pointer;      { Pointer to BCPL global vector       }
        dl_A2           : Longint;      { Private register dump of DOS        }
        dl_A5           : Longint;
        dl_A6           : Longint;
        dl_Errors       : pErrorString; { pointer to array of error msgs }
        dl_TimeReq      : pTimeRequest; { private pointer to timer request }
        dl_UtilityBase  : pLibrary;     { private ptr to utility library }
        dl_IntuitionBase : pLibrary;
    end;

    pRootNode = ^tRootNode;
    tRootNode = record
        rn_TaskArray    : BPTR;         { [0] is max number of CLI's
                                          [1] is APTR to process id of CLI 1
                                          [n] is APTR to process id of CLI n }
        rn_ConsoleSegment : BPTR;       { SegList for the CLI }
        rn_Time          : tDateStamp;  { Current time }
        rn_RestartSeg   : Longint;      { SegList for the disk validator process }
        rn_Info         : BPTR;         { Pointer ot the Info structure }
        rn_FileHandlerSegment : BPTR;   { segment for a file handler }
        rn_CliList      : tMinList;     { new list of all CLI processes }
                                        { the first cpl_Array is also rn_TaskArray }
        rn_BootProc     : pMsgPort;     { private ptr to msgport of boot fs      }
        rn_ShellSegment : BPTR;         { seglist for Shell (for NewShell)         }
        rn_Flags        : Longint;      { dos flags }
    end;

CONST
 RNB_WILDSTAR   = 24;
 RNF_WILDSTAR   = 16777216;
 RNB_PRIVATE1   = 1;       { private for dos }
 RNF_PRIVATE1   = 2;

Type
    pDosInfo = ^tDosInfo;
    tDosInfo = record
        case smallint of
        0 : (
        di_ResList : BPTR;
        );
        1 : (
        di_McName       : BPTR;          { Network name of this machine; currently 0 }
        di_DevInfo      : BPTR;          { Device List }
        di_Devices      : BPTR;          { Currently zero }
        di_Handlers     : BPTR;          { Currently zero }
        di_NetHand      : Pointer;       { Network handler processid; currently zero }
        di_DevLock,                      { do NOT access directly! }
        di_EntryLock,                    { do NOT access directly! }
        di_DeleteLock   : tSignalSemaphore; { do NOT access directly! }
        );
    end;

{ ONLY to be allocated by DOS! }

       pCliProcList = ^tCliProcList;
       tCliProcList = record
        cpl_Node   : tMinNode;
        cpl_First  : Longint;      { number of first entry in array }
        cpl_Array  : Array[0..0] of pMsgPort;
                             { [0] is max number of CLI's in this entry (n)
                              * [1] is CPTR to process id of CLI cpl_First
                              * [n] is CPTR to process id of CLI cpl_First+n-1
                              }
       END;

{ structure for the Dos resident list.  Do NOT allocate these, use       }
{ AddSegment(), and heed the warnings in the autodocs!                   }

Type
       pSegment = ^tSegment;
       tSegment = record
        seg_Next  : BPTR;
        seg_UC    : Longint;
        seg_Seg   : BPTR;
        seg_Name  : Array[0..3] of Char;      { actually the first 4 chars of BSTR name }
       END;

CONST
 CMD_SYSTEM    =  -1;
 CMD_INTERNAL  =  -2;
 CMD_DISABLED  =  -999;


{ DOS Processes started from the CLI via RUN or NEWCLI have this additional
 * set to data associated with them }
Type
    pCommandLineInterface = ^tCommandLineInterface;
    tCommandLineInterface = record
        cli_Result2        : Longint;      { Value of IoErr from last command }
        cli_SetName        : BSTR;         { Name of current directory }
        cli_CommandDir     : BPTR;         { Lock associated with command directory }
        cli_ReturnCode     : Longint;      { Return code from last command }
        cli_CommandName    : BSTR;         { Name of current command }
        cli_FailLevel      : Longint;      { Fail level (set by FAILAT) }
        cli_Prompt         : BSTR;         { Current prompt (set by PROMPT) }
        cli_StandardInput  : BPTR;         { Default (terminal) CLI input }
        cli_CurrentInput   : BPTR;         { Current CLI input }
        cli_CommandFile    : BSTR;         { Name of EXECUTE command file }
        cli_Interactive    : Longint;      { Boolean; True if prompts required }
        cli_Background     : Longint;      { Boolean; True if CLI created by RUN }
        cli_CurrentOutput  : BPTR;         { Current CLI output }
        cli_DefaultStack   : Longint;      { Stack size to be obtained in long words }
        cli_StandardOutput : BPTR;         { Default (terminal) CLI output }
        cli_Module         : BPTR;         { SegList of currently loaded command }
    end;

{ This structure can take on different values depending on whether it is
 * a device, an assigned directory, or a volume.  Below is the structure
 * reflecting volumes only.  Following that is the structure representing
 * only devices.
 }

{ structure representing a volume }

    pDeviceList = ^tDeviceList;
    tDeviceList = record
        dl_Next         : BPTR;         { bptr to next device list }
        dl_Type         : Longint;      { see DLT below }
        dl_Task         : pMsgPort;     { ptr to handler task }
        dl_Lock         : BPTR;         { not for volumes }
        dl_VolumeDate   : tDateStamp;   { creation date }
        dl_LockList     : BPTR;         { outstanding locks }
        dl_DiskType     : Longint;      { 'DOS', etc }
        dl_unused       : Longint;
        dl_Name         : BSTR;         { bptr to bcpl name }
    end;

{ device structure (same as the DeviceNode structure in filehandler.h) }

    pDevInfo = ^tDevInfo;
    tDevInfo = record
        dvi_Next        : BPTR;
        dvi_Type        : Longint;
        dvi_Task        : Pointer;
        dvi_Lock        : BPTR;
        dvi_Handler     : BSTR;
        dvi_StackSize   : Longint;
        dvi_Priority    : Longint;
        dvi_Startup     : Longint;
        dvi_SegList     : BPTR;
        dvi_GlobVec     : BSTR;
        dvi_Name        : BSTR;
    end;

{    structure used for multi-directory assigns. AllocVec()ed. }

       pAssignList = ^tAssignList;
       tAssignList = record
        al_Next : pAssignList;
        al_Lock : BPTR;
       END;


{ combined structure for devices, assigned directories, volumes }

   pDosList = ^tDosList;
   tDosList = record
    dol_Next            : BPTR;           {    bptr to next device on list }
    dol_Type            : Longint;        {    see DLT below }
    dol_Task            : pMsgPort;       {    ptr to handler task }
    dol_Lock            : BPTR;
    case smallint of
    0 : (
        dol_Handler : record
          dol_Handler    : BSTR;      {    file name to load IF seglist is null }
          dol_StackSize,              {    stacksize to use when starting process }
          dol_Priority,               {    task priority when starting process }
          dol_Startup    : Longint;   {    startup msg: FileSysStartupMsg for disks }
          dol_SegList,                {    already loaded code for new task }
          dol_GlobVec    : BPTR;      {    BCPL global vector to use when starting
                                 * a process. -1 indicates a C/Assembler
                                 * program. }
        end;
    );
    1 : (
        dol_Volume       : record
          dol_VolumeDate : tDateStamp; {    creation date }
          dol_LockList   : BPTR;       {    outstanding locks }
          dol_DiskType   : Longint;    {    'DOS', etc }
        END;
    );
    2 : (
        dol_assign       :  record
          dol_AssignName : STRPTR;        {    name for non-OR-late-binding assign }
          dol_List       : pAssignList;   {    for multi-directory assigns (regular) }
        END;
    dol_Name            : BSTR;           {    bptr to bcpl name }
    );
   END;

Const

{ definitions for dl_Type }

    DLT_DEVICE          = 0;
    DLT_DIRECTORY       = 1;
    DLT_VOLUME          = 2;
    DLT_LATE            = 3;       {    late-binding assign }
    DLT_NONBINDING      = 4;       {    non-binding assign }
    DLT_PRIVATE         = -1;      {    for internal use only }

{    structure return by GetDeviceProc() }
Type

       pDevProc = ^tDevProc;
       tDevProc = record
        dvp_Port        : pMsgPort;
        dvp_Lock        : BPTR;
        dvp_Flags       : Longint;
        dvp_DevNode     : pDosList;    {    DON'T TOUCH OR USE! }
       END;

CONST
{    definitions for dvp_Flags }
     DVPB_UNLOCK   =  0;
     DVPF_UNLOCK   =  1;
     DVPB_ASSIGN   =  1;
     DVPF_ASSIGN   =  2;

{    Flags to be passed to LockDosList(), etc }
     LDB_DEVICES   =  2;
     LDF_DEVICES   =  4;
     LDB_VOLUMES   =  3;
     LDF_VOLUMES   =  8;
     LDB_ASSIGNS   =  4;
     LDF_ASSIGNS   =  16;
     LDB_ENTRY     =  5;
     LDF_ENTRY     =  32;
     LDB_DELETE    =  6;
     LDF_DELETE    =  64;

{    you MUST specify one of LDF_READ or LDF_WRITE }
     LDB_READ      =  0;
     LDF_READ      =  1;
     LDB_WRITE     =  1;
     LDF_WRITE     =  2;

{    actually all but LDF_ENTRY (which is used for internal locking) }
     LDF_ALL       =  (LDF_DEVICES+LDF_VOLUMES+LDF_ASSIGNS);

{    error report types for ErrorReport() }
     REPORT_STREAM          = 0;       {    a stream }
     REPORT_TASK            = 1;       {    a process - unused }
     REPORT_LOCK            = 2;       {    a lock }
     REPORT_VOLUME          = 3;       {    a volume node }
     REPORT_INSERT          = 4;       {    please insert volume }

{    Special error codes for ErrorReport() }
     ABORT_DISK_ERROR       = 296;     {    Read/write error }
     ABORT_BUSY             = 288;     {    You MUST replace... }

{    types for initial packets to shells from run/newcli/execute/system. }
{    For shell-writers only }
     RUN_EXECUTE           =  -1;
     RUN_SYSTEM            =  -2;
     RUN_SYSTEM_ASYNCH     =  -3;

{    Types for fib_DirEntryType.  NOTE that both USERDIR and ROOT are      }
{    directories, and that directory/file checks should use <0 and >=0.    }
{    This is not necessarily exhaustive!  Some handlers may use other      }
{    values as needed, though <0 and >=0 should remain as supported as     }
{    possible.                                                             }
     ST_ROOT       =  1 ;
     ST_USERDIR    =  2 ;
     ST_SOFTLINK   =  3 ;      {    looks like dir, but may point to a file! }
     ST_LINKDIR    =  4 ;      {    hard link to dir }
     ST_FILE       =  -3;      {    must be negative for FIB! }
     ST_LINKFILE   =  -4;      {    hard link to file }
     ST_PIPEFILE   =  -5;      {    for pipes that support ExamineFH   }

Type

{ a lock structure, as returned by Lock() or DupLock() }

    pFileLock = ^tFileLock;
    tFileLock = record
        fl_Link         : BPTR;         { bcpl pointer to next lock }
        fl_Key          : Longint;      { disk block number }
        fl_Access       : Longint;      { exclusive or shared }
        fl_Task         : pMsgPort;     { handler task's port }
        fl_Volume       : BPTR;         { bptr to a DeviceList }
    end;


{  NOTE: V37 dos.library, when doing ExAll() emulation, and V37 filesystems  }
{  will return an error if passed ED_OWNER.  If you get ERROR_BAD_NUMBER,    }
{  retry with ED_COMMENT to get everything but owner info.  All filesystems  }
{  supporting ExAll() must support through ED_COMMENT, and must check Type   }
{  and return ERROR_BAD_NUMBER if they don't support the type.               }

{   values that can be passed for what data you want from ExAll() }
{   each higher value includes those below it (numerically)       }
{   you MUST chose one of these values }
CONST
     ED_NAME        = 1;
     ED_TYPE        = 2;
     ED_SIZE        = 3;
     ED_PROTECTION  = 4;
     ED_DATE        = 5;
     ED_COMMENT     = 6;
     ED_OWNER       = 7;
{
 *   Structure in which exall results are returned in.  Note that only the
 *   fields asked for will exist!
 }
Type
       pExAllData = ^tExAllData;
       tExAllData = record
        ed_Next     : pExAllData;
        ed_Name     : STRPTR;
        ed_Type,
        ed_Size,
        ed_Prot,
        ed_Days,
        ed_Mins,
        ed_Ticks    : ULONG;
        ed_Comment  : STRPTR;     {   strings will be after last used field }
        ed_OwnerUID,              { new for V39 }
        ed_OwnerGID : Word;
       END;

{
 *   Control structure passed to ExAll.  Unused fields MUST be initialized to
 *   0, expecially eac_LastKey.
 *
 *   eac_MatchFunc is a hook (see utility.library documentation for usage)
 *   It should return true if the entry is to returned, false if it is to be
 *   ignored.
 *
 *   This structure MUST be allocated by AllocDosObject()!
 }

       pExAllControl = ^tExAllControl;
       tExAllControl = record
        eac_Entries,                 {   number of entries returned in buffer      }
        eac_LastKey     : ULONG;     {   Don't touch inbetween linked ExAll calls! }
        eac_MatchString : STRPTR;    {   wildcard string for pattern match OR NULL }
        eac_MatchFunc   : pHook;     {   optional private wildcard FUNCTION     }
       END;



{ The disk "environment" is a longword array that describes the
 * disk geometry.  It is variable sized, with the length at the beginning.
 * Here are the constants for a standard geometry.
}

Type

    pDosEnvec = ^tDosEnvec;
    tDosEnvec = record
        de_TableSize      : ULONG;      { Size of Environment vector }
        de_SizeBlock      : ULONG;      { in longwords: standard value is 128 }
        de_SecOrg         : ULONG;      { not used; must be 0 }
        de_Surfaces       : ULONG;      { # of heads (surfaces). drive specific }
        de_SectorPerBlock : ULONG;      { not used; must be 1 }
        de_BlocksPerTrack : ULONG;      { blocks per track. drive specific }
        de_Reserved       : ULONG;      { DOS reserved blocks at start of partition. }
        de_PreAlloc       : ULONG;      { DOS reserved blocks at end of partition }
        de_Interleave     : ULONG;      { usually 0 }
        de_LowCyl         : ULONG;      { starting cylinder. typically 0 }
        de_HighCyl        : ULONG;      { max cylinder. drive specific }
        de_NumBuffers     : ULONG;      { Initial # DOS of buffers.  }
        de_BufMemType     : ULONG;      { type of mem to allocate for buffers }
        de_MaxTransfer    : ULONG;      { Max number of bytes to transfer at a time }
        de_Mask           : ULONG;      { Address Mask to block out certain memory }
        de_BootPri        : Longint;    { Boot priority for autoboot }
        de_DosType        : ULONG;      { ASCII (HEX) string showing filesystem type;
                                        * 0X444F5300 is old filesystem,
                                        * 0X444F5301 is fast file system }
        de_Baud           : ULONG;      {     Baud rate for serial handler }
        de_Control        : ULONG;      {     Control smallint for handler/filesystem }
        de_BootBlocks     : ULONG;      {     Number of blocks containing boot code }
    end;

Const

{ these are the offsets into the array }

    DE_TABLESIZE        = 0;    { standard value is 11 }
    DE_SIZEBLOCK        = 1;    { in longwords: standard value is 128 }
    DE_SECORG           = 2;    { not used; must be 0 }
    DE_NUMHEADS         = 3;    { # of heads (surfaces). drive specific }
    DE_SECSPERBLK       = 4;    { not used; must be 1 }
    DE_BLKSPERTRACK     = 5;    { blocks per track. drive specific }
    DE_RESERVEDBLKS     = 6;    { unavailable blocks at start.   usually 2 }
    DE_PREFAC           = 7;    { not used; must be 0 }
    DE_INTERLEAVE       = 8;    { usually 0 }
    DE_LOWCYL           = 9;    { starting cylinder. typically 0 }
    DE_UPPERCYL         = 10;   { max cylinder.  drive specific }
    DE_NUMBUFFERS       = 11;   { starting # of buffers.  typically 5 }
    DE_MEMBUFTYPE       = 12;   { type of mem to allocate for buffers. }
    DE_BUFMEMTYPE       = 12;   { same as above, better name
                                 * 1 is public, 3 is chip, 5 is fast }
    DE_MAXTRANSFER      = 13;   { Max number bytes to transfer at a time }
    DE_MASK             = 14;   { Address Mask to block out certain memory }
    DE_BOOTPRI          = 15;   { Boot priority for autoboot }
    DE_DOSTYPE          = 16;   { ASCII (HEX) string showing filesystem type;
                                 * 0X444F5300 is old filesystem,
                                 * 0X444F5301 is fast file system }
    DE_BAUD             = 17;   {     Baud rate for serial handler }
    DE_CONTROL          = 18;   {     Control smallint for handler/filesystem }
    DE_BOOTBLOCKS       = 19;   {     Number of blocks containing boot code }


{ The file system startup message is linked into a device node's startup
** field.  It contains a pointer to the above environment, plus the
** information needed to do an exec OpenDevice().
}

Type

    pFileSysStartupMsg = ^tFileSysStartupMsg;
    tFileSysStartupMsg = record
        fssm_Unit       : ULONG;        { exec unit number for this device }
        fssm_Device     : BSTR;         { null terminated bstring to the device name }
        fssm_Environ    : BPTR;         { ptr to environment table (see above) }
        fssm_Flags      : ULONG;        { flags for OpenDevice() }
    end;


{ The include file "libraries/dosextens.h" has a DeviceList structure.
 * The "device list" can have one of three different things linked onto
 * it.  Dosextens defines the structure for a volume.  DLT_DIRECTORY
 * is for an assigned directory.  The following structure is for
 * a dos "device" (DLT_DEVICE).
}

    pDeviceNode = ^tDeviceNode;
    tDeviceNode = record
        dn_Next         : BPTR;         { singly linked list }
        dn_Type         : ULONG;        { always 0 for dos "devices" }
        dn_Task         : pMsgPort;     { standard dos "task" field.  If this is
                                         * null when the node is accesses, a task
                                         * will be started up }
        dn_Lock         : BPTR;         { not used for devices -- leave null }
        dn_Handler      : BSTR;         { filename to loadseg (if seglist is null) }
        dn_StackSize    : ULONG;        { stacksize to use when starting task }
        dn_Priority     : Longint;      { task priority when starting task }
        dn_Startup      : BPTR;         { startup msg: FileSysStartupMsg for disks }
        dn_SegList      : BPTR;         { code to run to start new task (if necessary).
                                         * if null then dn_Handler will be loaded. }
        dn_GlobalVec    : BPTR; { BCPL global vector to use when starting
                                 * a task.  -1 means that dn_SegList is not
                                 * for a bcpl program, so the dos won't
                                 * try and construct one.  0 tell the
                                 * dos that you obey BCPL linkage rules,
                                 * and that it should construct a global
                                 * vector for you.
                                 }
        dn_Name         : BSTR;         { the node name, e.g. '\3','D','F','3' }
    end;

CONST
{     use of Class and code is discouraged for the time being - we might want to
   change things }
{     --- NotifyMessage Class ------------------------------------------------ }
     NOTIFY_CLASS  =  $40000000;

{     --- NotifyMessage Codes ------------------------------------------------ }
     NOTIFY_CODE   =  $1234;


{     Sent to the application if SEND_MESSAGE is specified.                    }

Type
{     Do not modify or reuse the notifyrequest while active.                   }
{     note: the first LONG of nr_Data has the length transfered                }


       pNotifyRequest = ^tNotifyRequest;
       tNotifyRequest = record
            nr_Name : pchar;
            nr_FullName : pchar;
            nr_UserData : ULONG;
            nr_Flags : ULONG;
            nr_stuff : record
                case smallint of
                   0 : ( nr_Msg : record
                        nr_Port : pMsgPort;
                     end );
                   1 : ( nr_Signal : record
                        nr_Task : pTask;
                        nr_SignalNum : BYTE;
                        nr_pad : array[0..2] of BYTE;
                     end );
                end;
            nr_Reserved : array[0..3] of ULONG;
            nr_MsgCount : ULONG;
            nr_Handler : pMsgPort;
         end;

   pNotifyMessage = ^tNotifyMessage;
   tNotifyMessage = record
    nm_ExecMessage : tMessage;
    nm_Class       : ULONG;
    nm_Code        : Word;
    nm_NReq        : pNotifyRequest;     {     don't modify the request! }
    nm_DoNotTouch,                       {     like it says!  For use by handlers }
    nm_DoNotTouch2 : ULONG;            {     ditto }
   END;


CONST
{     --- NotifyRequest Flags ------------------------------------------------ }
     NRF_SEND_MESSAGE      =  1 ;
     NRF_SEND_SIGNAL       =  2 ;
     NRF_WAIT_REPLY        =  8 ;
     NRF_NOTIFY_INITIAL    =  16;

{     do NOT set or remove NRF_MAGIC!  Only for use by handlers! }
     NRF_MAGIC             = $80000000;

{     bit numbers }
     NRB_SEND_MESSAGE      =  0;
     NRB_SEND_SIGNAL       =  1;
     NRB_WAIT_REPLY        =  3;
     NRB_NOTIFY_INITIAL    =  4;

     NRB_MAGIC             =  31;

{     Flags reserved for private use by the handler: }
     NR_HANDLER_FLAGS      =  $ffff0000;

{   *********************************************************************
 *
 * The CSource data structure defines the input source for "ReadItem()"
 * as well as the ReadArgs call.  It is a publicly defined structure
 * which may be used by applications which use code that follows the
 * conventions defined for access.
 *
 * When passed to the dos.library functions, the value passed as
 * struct *CSource is defined as follows:
 *      if ( CSource == 0)      Use buffered IO "ReadChar()" as data source
 *      else                    Use CSource for input character stream
 *
 * The following two pseudo-code routines define how the CSource structure
 * is used:
 *
 * long CS_ReadChar( struct CSource *CSource )
 *
 *      if ( CSource == 0 )     return ReadChar();
 *      if ( CSource->CurChr >= CSource->Length )       return ENDSTREAMCHAR;
 *      return CSource->Buffer[ CSource->CurChr++ ];
 *
 *
 * BOOL CS_UnReadChar( struct CSource *CSource )
 *
 *      if ( CSource == 0 )     return UnReadChar();
 *      if ( CSource->CurChr <= 0 )     return FALSE;
 *      CSource->CurChr--;
 *      return TRUE;
 *
 *
 * To initialize a struct CSource, you set CSource->CS_Buffer to
 * a string which is used as the data source, and set CS_Length to
 * the number of characters in the string.  Normally CS_CurChr should
 * be initialized to ZERO, or left as it was from prior use as
 * a CSource.
 *
 *********************************************************************}

Type
       pCSource = ^tCSource;
       tCSource = record
        CS_Buffer  : STRPTR;
        CS_Length,
        CS_CurChr  : Longint;
       END;

{   *********************************************************************
 *
 * The RDArgs data structure is the input parameter passed to the DOS
 * ReadArgs() function call.
 *
 * The RDA_Source structure is a CSource as defined above;
 * if RDA_Source.CS_Buffer is non-null, RDA_Source is used as the input
 * character stream to parse, else the input comes from the buffered STDIN
 * calls ReadChar/UnReadChar.
 *
 * RDA_DAList is a private address which is used internally to track
 * allocations which are freed by FreeArgs().  This MUST be initialized
 * to NULL prior to the first call to ReadArgs().
 *
 * The RDA_Buffer and RDA_BufSiz fields allow the application to supply
 * a fixed-size buffer in which to store the parsed data.  This allows
 * the application to pre-allocate a buffer rather than requiring buffer
 * space to be allocated.  If either RDA_Buffer or RDA_BufSiz is NULL,
 * the application has not supplied a buffer.
 *
 * RDA_ExtHelp is a text string which will be displayed instead of the
 * template string, if the user is prompted for input.
 *
 * RDA_Flags bits control how ReadArgs() works.  The flag bits are
 * defined below.  Defaults are initialized to ZERO.
 *
 *********************************************************************}

       pRDArgs = ^tRDArgs;
       tRDArgs = record
        RDA_Source  : tCSource;     {    Select input source }
        RDA_DAList  : Longint;      {    PRIVATE. }
        RDA_Buffer  : STRPTR;       {    Optional string parsing space. }
        RDA_BufSiz  : Longint;      {    Size of RDA_Buffer (0..n) }
        RDA_ExtHelp : STRPTR;       {    Optional extended help }
        RDA_Flags   : Longint;      {    Flags for any required control }
       END;

CONST
       RDAB_STDIN     = 0;       {    Use "STDIN" rather than "COMMAND LINE" }
       RDAF_STDIN     = 1;
       RDAB_NOALLOC   = 1;       {    If set, do not allocate extra string space.}
       RDAF_NOALLOC   = 2;
       RDAB_NOPROMPT  = 2;       {    Disable reprompting for string input. }
       RDAF_NOPROMPT  = 4;

{   *********************************************************************
 * Maximum number of template keywords which can be in a template passed
 * to ReadArgs(). IMPLEMENTOR NOTE - must be a multiple of 4.
 *********************************************************************}
       MAX_TEMPLATE_ITEMS     = 100;

{   *********************************************************************
 * Maximum number of MULTIARG items returned by ReadArgs(), before
 * an ERROR_LINE_TOO_LONG.  These two limitations are due to stack
 * usage.  Applications should allow "a lot" of stack to use ReadArgs().
 *********************************************************************}
       MAX_MULTIARGS          = 128;

CONST
{     Modes for LockRecord/LockRecords() }
       REC_EXCLUSIVE          = 0;
       REC_EXCLUSIVE_IMMED    = 1;
       REC_SHARED             = 2;
       REC_SHARED_IMMED       = 3;

{     struct to be passed to LockRecords()/UnLockRecords() }

Type
       pRecordLock = ^tRecordLock;
       tRecordLock = record
        rec_FH    : BPTR;         {     filehandle }
        rec_Offset,               {     offset in file }
        rec_Length,               {     length of file to be locked }
        rec_Mode  : ULONG;        {     Type of lock }
       END;


{      the structure in the pr_LocalVars list }
{      Do NOT allocate yourself, use SetVar()!!! This structure may grow in }
{      future releases!  The list should be left in alphabetical order, and }
{      may have multiple entries with the same name but different types.    }
Type
       pLocalVar = ^tLocalVar;
       tLocalVar = record
        lv_Node  : tNode;
        lv_Flags : Word;
        lv_Value : STRPTR;
        lv_Len   : ULONG;
       END;

{
 * The lv_Flags bits are available to the application.  The unused
 * lv_Node.ln_Pri bits are reserved for system use.
 }

CONST
{      bit definitions for lv_Node.ln_Type: }
       LV_VAR               =   0;       {      an variable }
       LV_ALIAS             =   1;       {      an alias }
{      to be or'ed into type: }
       LVB_IGNORE           =   7;       {      ignore this entry on GetVar, etc }
       LVF_IGNORE           =   $80;

{      definitions of flags passed to GetVar()/SetVar()/DeleteVar() }
{      bit defs to be OR'ed with the type: }
{      item will be treated as a single line of text unless BINARY_VAR is used }
       GVB_GLOBAL_ONLY       =  8   ;
       GVF_GLOBAL_ONLY       =  $100;
       GVB_LOCAL_ONLY        =  9   ;
       GVF_LOCAL_ONLY        =  $200;
       GVB_BINARY_VAR        =  10  ;            {      treat variable as binary }
       GVF_BINARY_VAR        =  $400;
       GVB_DONT_NULL_TERM    =  11;            { only with GVF_BINARY_VAR }
       GVF_DONT_NULL_TERM    =  $800;

{ this is only supported in >= V39 dos.  V37 dos ignores this. }
{ this causes SetVar to affect ENVARC: as well as ENV:.        }
      GVB_SAVE_VAR           = 12 ;     { only with GVF_GLOBAL_VAR }
      GVF_SAVE_VAR           = $1000 ;


CONST
{   ***************************************************************************}
{    definitions for the System() call }

    SYS_Dummy      = (TAG_USER + 32);
    SYS_Input      = (SYS_Dummy + 1);
                                {    specifies the input filehandle  }
    SYS_Output     = (SYS_Dummy + 2);
                                {    specifies the output filehandle }
    SYS_Asynch     = (SYS_Dummy + 3);
                                {    run asynch, close input/output on exit(!) }
    SYS_UserShell  = (SYS_Dummy + 4);
                                {    send to user shell instead of boot shell }
    SYS_CustomShell= (SYS_Dummy + 5);
                                {    send to a specific shell (data is name) }
{         SYS_Error, }


{   ***************************************************************************}
{    definitions for the CreateNewProc() call }
{    you MUST specify one of NP_Seglist or NP_Entry.  All else is optional. }

    NP_Dummy       = (TAG_USER + 1000);
    NP_Seglist     = (NP_Dummy + 1);
                                {    seglist of code to run for the process  }
    NP_FreeSeglist = (NP_Dummy + 2);
                                {    free seglist on exit - only valid for   }
                                {    for NP_Seglist.  Default is TRUE.       }
    NP_Entry       = (NP_Dummy + 3);
                                {    entry point to run - mutually exclusive }
                                {    with NP_Seglist! }
    NP_Input       = (NP_Dummy + 4);
                                {    filehandle - default is Open("NIL:"...) }
    NP_Output      = (NP_Dummy + 5);
                                {    filehandle - default is Open("NIL:"...) }
    NP_CloseInput  = (NP_Dummy + 6);
                                {    close input filehandle on exit          }
                                {    default TRUE                            }
    NP_CloseOutput = (NP_Dummy + 7);
                                {    close output filehandle on exit         }
                                {    default TRUE                            }
    NP_Error       = (NP_Dummy + 8);
                                {    filehandle - default is Open("NIL:"...) }
    NP_CloseError  = (NP_Dummy + 9);
                                {    close error filehandle on exit          }
                                {    default TRUE                            }
    NP_CurrentDir  = (NP_Dummy + 10);
                                {    lock - default is parent's current dir  }
    NP_StackSize   = (NP_Dummy + 11);
                                {    stacksize for process - default 4000    }
    NP_Name        = (NP_Dummy + 12);
                                {    name for process - default "New Process"}
    NP_Priority    = (NP_Dummy + 13);
                                {    priority - default same as parent       }
    NP_ConsoleTask = (NP_Dummy + 14);
                                {    consoletask - default same as parent    }
    NP_WindowPtr   = (NP_Dummy + 15);
                                {    window ptr - default is same as parent  }
    NP_HomeDir     = (NP_Dummy + 16);
                                {    home directory - default curr home dir  }
    NP_CopyVars    = (NP_Dummy + 17);
                                {    boolean to copy local vars-default TRUE }
    NP_Cli         = (NP_Dummy + 18);
                                {    create cli structure - default FALSE    }
    NP_Path        = (NP_Dummy + 19);
                                {    path - default is copy of parents path  }
                                {    only valid if a cli process!    }
    NP_CommandName = (NP_Dummy + 20);
                                {    commandname - valid only for CLI        }
    NP_Arguments   = (NP_Dummy + 21);
                                {    cstring of arguments - passed with str  }
                                {    in a0, length in d0.  (copied and freed }
                                {    on exit.  Default is empty string.      }
                                {    NOTE: not operational until 2.04 - see  }
                                {    BIX/TechNotes for more info/workarounds }
                                {    NOTE: in 2.0, it DIDN'T pass "" - the   }
                                {    registers were random.                  }
{    FIX! should this be only for cli's? }
    NP_NotifyOnDeath = (NP_Dummy + 22);
                                {    notify parent on death - default FALSE  }
                                {    Not functional yet. }
    NP_Synchronous   = (NP_Dummy + 23);
                                {    don't return until process finishes -   }
                                {    default FALSE.                          }
                                {    Not functional yet. }
    NP_ExitCode      = (NP_Dummy + 24);
                                {    code to be called on process exit       }
    NP_ExitData      = (NP_Dummy + 25);
                                {    optional argument for NP_EndCode rtn -  }
                                {    default NULL                            }


{   ***************************************************************************}
{    tags for AllocDosObject }

    ADO_Dummy        = (TAG_USER + 2000);
    ADO_FH_Mode      = (ADO_Dummy + 1);
                                {    for type DOS_FILEHANDLE only            }
                                {    sets up FH for mode specified.
                                   This can make a big difference for buffered
                                   files.                                  }
        {    The following are for DOS_CLI }
        {    If you do not specify these, dos will use it's preferred values }
        {    which may change from release to release.  The BPTRs to these   }
        {    will be set up correctly for you.  Everything will be zero,     }
        {    except cli_FailLevel (10) and cli_Background (DOSTRUE).         }
        {    NOTE: you may also use these 4 tags with CreateNewProc.         }

    ADO_DirLen     = (ADO_Dummy + 2);
                                {    size in bytes for current dir buffer    }
    ADO_CommNameLen= (ADO_Dummy + 3);
                                {    size in bytes for command name buffer   }
    ADO_CommFileLen= (ADO_Dummy + 4);
                                {    size in bytes for command file buffer   }
    ADO_PromptLen  = (ADO_Dummy + 5);
                                {    size in bytes for the prompt buffer     }

{   ***************************************************************************}
{    tags for NewLoadSeg }
{    no tags are defined yet for NewLoadSeg }


PROCEDURE AbortPkt(port : pMsgPort; pkt : pDosPacket);
FUNCTION AddBuffers(const name : pCHAR; number : LONGINT) : BOOLEAN;
FUNCTION AddDosEntry(dlist : pDosList) : BOOLEAN;
FUNCTION AddPart(dirname : pCHAR;const filename : pCHAR; size : ULONG) : BOOLEAN;
FUNCTION AddSegment(const name : pCHAR; seg : LONGINT; system : LONGINT) : BOOLEAN;
FUNCTION AllocDosObject(type_ : ULONG;const tags : pTagItem) : POINTER;
FUNCTION AllocDosObjectTagList(type_ : ULONG;const tags : pTagItem) : POINTER;
FUNCTION AssignAdd(const name : pCHAR; lock : LONGINT) : BOOLEAN;
FUNCTION AssignLate(const name : pCHAR;const path : pCHAR) : BOOLEAN;
FUNCTION AssignLock(const name : pCHAR; lock : LONGINT) : BOOLEAN;
FUNCTION AssignPath(const name : pCHAR;const path : pCHAR) : BOOLEAN;
FUNCTION AttemptLockDosList(flags : ULONG) : pDosList;
FUNCTION ChangeMode(type_ : LONGINT; fh : LONGINT; newmode : LONGINT) : BOOLEAN;
FUNCTION CheckSignal(mask : LONGINT) : LONGINT;
FUNCTION Cli : pCommandLineInterface;
FUNCTION CliInitNewcli(dp : pDosPacket) : LONGINT;
FUNCTION CliInitRun(dp : pDosPacket) : LONGINT;
FUNCTION CompareDates(const date1 : pDateStamp;const date2 : pDateStamp) : LONGINT;
FUNCTION CreateDir(const name : pCHAR) : LONGINT;
FUNCTION CreateNewProc(const tags : pTagItem) : pProcess;
FUNCTION CreateNewProcTagList(const tags : pTagItem) : pProcess;
FUNCTION CreateProc(const name : pCHAR; pri : LONGINT; segList : LONGINT; stackSize : LONGINT) : pMsgPort;
FUNCTION CurrentDir(lock : LONGINT) : LONGINT;
PROCEDURE DateStamp(date : pDateStamp);
FUNCTION DateToStr(datetime : pDateTime) : BOOLEAN;
FUNCTION DeleteFile(const name : pCHAR) : BOOLEAN;
FUNCTION DeleteVar(const name : pCHAR; flags : ULONG) : BOOLEAN;
FUNCTION DeviceProc(const name : pCHAR) : pMsgPort;
FUNCTION DoPkt(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT; arg3 : LONGINT; arg4 : LONGINT; arg5 : LONGINT) : LONGINT;
FUNCTION DoPkt0(port : pMsgPort; action : LONGINT) : LONGINT;
FUNCTION DoPkt1(port : pMsgPort; action : LONGINT; arg1 : LONGINT) : LONGINT;
FUNCTION DoPkt2(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT) : LONGINT;
FUNCTION DoPkt3(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT; arg3 : LONGINT) : LONGINT;
FUNCTION DoPkt4(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT; arg3 : LONGINT; arg4 : LONGINT) : LONGINT;
PROCEDURE DOSClose(file_ : LONGINT);
PROCEDURE DOSDelay(timeout : LONGINT);
PROCEDURE DOSExit(returnCode : LONGINT);
FUNCTION DOSFlush(fh : LONGINT) : BOOLEAN;
FUNCTION DOSInput : LONGINT;
FUNCTION DOSOpen(const name : pCHAR; accessMode : LONGINT) : LONGINT;
FUNCTION DOSOutput : LONGINT;
FUNCTION DOSRead(file_ : LONGINT; buffer : POINTER; length : LONGINT) : LONGINT;
FUNCTION DOSRename(const oldName : pCHAR;const newName : pCHAR) : Boolean;
FUNCTION DOSSeek(file_ : LONGINT; position : LONGINT; offset : LONGINT) : LONGINT;
FUNCTION DOSWrite(file_ : LONGINT; buffer : POINTER; length : LONGINT) : LONGINT;
FUNCTION DupLock(lock : LONGINT) : LONGINT;
FUNCTION DupLockFromFH(fh : LONGINT) : LONGINT;
PROCEDURE EndNotify(notify : pNotifyRequest);
FUNCTION ErrorReport(code : LONGINT; type_ : LONGINT; arg1 : ULONG; device : pMsgPort) : BOOLEAN;
FUNCTION ExAll(lock : LONGINT; buffer : pExAllData; size : LONGINT; data : LONGINT; control : pExAllControl) : BOOLEAN;
PROCEDURE ExAllEnd(lock : LONGINT; buffer : pExAllData; size : LONGINT; data : LONGINT; control : pExAllControl);
FUNCTION Examine(lock : LONGINT; fileInfoBlock : pFileInfoBlock) : BOOLEAN;
FUNCTION ExamineFH(fh : LONGINT; fib : pFileInfoBlock) : BOOLEAN;
FUNCTION Execute(const string_ : pCHAR; file_ : LONGINT; file2 : LONGINT) : BOOLEAN;
FUNCTION ExNext(lock : LONGINT; fileInfoBlock : pFileInfoBlock) : BOOLEAN;
FUNCTION Fault(code : LONGINT; header : pCHAR; buffer : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION FGetC(fh : LONGINT) : LONGINT;
FUNCTION FGets(fh : LONGINT; buf : pCHAR; buflen : ULONG) : pCHAR;
FUNCTION FilePart(const path : pCHAR) : pCHAR;
FUNCTION FindArg(const keyword : pCHAR;const arg_template : pCHAR) : LONGINT;
FUNCTION FindCliProc(num : ULONG) : pProcess;
FUNCTION FindDosEntry(const dlist : pDosList;const name : pCHAR; flags : ULONG) : pDosList;
FUNCTION FindSegment(const name : pCHAR;const seg : pSegment; system : LONGINT) : pSegment;
FUNCTION FindVar(const name : pCHAR; type_ : ULONG) : pLocalVar;
FUNCTION Format(const filesystem : pCHAR;const volumename : pCHAR; dostype : ULONG) : BOOLEAN;
FUNCTION FPutC(fh : LONGINT; ch : LONGINT) : LONGINT;
FUNCTION FPuts(fh : LONGINT;const str : pCHAR) : BOOLEAN;
FUNCTION FRead(fh : LONGINT; block : POINTER; blocklen : ULONG; number : ULONG) : LONGINT;
PROCEDURE FreeArgs(args : pRDArgs);
PROCEDURE FreeDeviceProc(dp : pDevProc);
PROCEDURE FreeDosEntry(dlist : pDosList);
PROCEDURE FreeDosObject(type_ : ULONG; ptr : POINTER);
FUNCTION FWrite(fh : LONGINT; block : POINTER; blocklen : ULONG; number : ULONG) : LONGINT;
FUNCTION GetArgStr : pCHAR;
FUNCTION GetConsoleTask : pMsgPort;
FUNCTION GetCurrentDirName(buf : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION GetDeviceProc(const name : pCHAR; dp : pDevProc) : pDevProc;
FUNCTION GetFileSysTask : pMsgPort;
FUNCTION GetProgramDir : LONGINT;
FUNCTION GetProgramName(buf : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION GetPrompt(buf : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION GetVar(const name : pCHAR; buffer : pCHAR; size : LONGINT; flags : LONGINT) : LONGINT;
FUNCTION Info(lock : LONGINT; parameterBlock : pInfoData) : BOOLEAN;
FUNCTION Inhibit(const name : pCHAR; onoff : LONGINT) : BOOLEAN;
FUNCTION InternalLoadSeg(fh : LONGINT; table : LONGINT;const funcarray : pLONGINT; VAR stack : LONGINT) : LONGINT;
FUNCTION InternalUnLoadSeg(seglist : LONGINT; freefunc : tPROCEDURE) : BOOLEAN;
FUNCTION IoErr : LONGINT;
FUNCTION IsFileSystem(const name : pCHAR) : BOOLEAN;
FUNCTION IsInteractive(file_ : LONGINT) : BOOLEAN;
FUNCTION LoadSeg(const name : pCHAR) : LONGINT;
FUNCTION Lock(const name : pCHAR; type_ : LONGINT) : LONGINT;
FUNCTION LockDosList(flags : ULONG) : pDosList;
FUNCTION LockRecord(fh : LONGINT; offset : ULONG; length : ULONG; mode : ULONG; timeout : ULONG) : BOOLEAN;
FUNCTION LockRecords(recArray : pRecordLock; timeout : ULONG) : BOOLEAN;
FUNCTION MakeDosEntry(const name : pCHAR; type_ : LONGINT) : pDosList;
FUNCTION MakeLink(const name : pCHAR; dest : LONGINT; soft : LONGINT) : BOOLEAN;
PROCEDURE MatchEnd(anchor : pAnchorPath);
FUNCTION MatchFirst(const pat : pCHAR; anchor : pAnchorPath) : LONGINT;
FUNCTION MatchNext(anchor : pAnchorPath) : LONGINT;
FUNCTION MatchPattern(const pat : pCHAR; str : pCHAR) : BOOLEAN;
FUNCTION MatchPatternNoCase(const pat : pCHAR; str : pCHAR) : BOOLEAN;
FUNCTION MaxCli : ULONG;
FUNCTION NameFromFH(fh : LONGINT; buffer : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION NameFromLock(lock : LONGINT; buffer : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION NewLoadSeg(const file_ : pCHAR;const tags : pTagItem) : LONGINT;
FUNCTION NewLoadSegTagList(const file_ : pCHAR;const tags : pTagItem) : LONGINT;
FUNCTION NextDosEntry(const dlist : pDosList; flags : ULONG) : pDosList;
FUNCTION OpenFromLock(lock : LONGINT) : LONGINT;
FUNCTION ParentDir(lock : LONGINT) : LONGINT;
FUNCTION ParentOfFH(fh : LONGINT) : LONGINT;
FUNCTION ParsePattern(const pat : pCHAR; buf : pCHAR; buflen : LONGINT) : LONGINT;
FUNCTION ParsePatternNoCase(const pat : pCHAR; buf : pCHAR; buflen : LONGINT) : LONGINT;
FUNCTION PathPart(const path : pCHAR) : pCHAR;
FUNCTION PrintFault(code : LONGINT;const header : pCHAR) : BOOLEAN;
FUNCTION PutStr(const str : pCHAR) : BOOLEAN;
FUNCTION ReadArgs(const arg_template : pCHAR; arra : pLONGINT; args : pRDArgs) : pRDArgs;
FUNCTION ReadItem(const name : pCHAR; maxchars : LONGINT; cSource : pCSource) : LONGINT;
FUNCTION ReadLink(port : pMsgPort; lock : LONGINT;const path : pCHAR; buffer : pCHAR; size : ULONG) : BOOLEAN;
FUNCTION Relabel(const drive : pCHAR;const newname : pCHAR) : BOOLEAN;
FUNCTION RemAssignList(const name : pCHAR; lock : LONGINT) : BOOLEAN;
FUNCTION RemDosEntry(dlist : pDosList) : BOOLEAN;
FUNCTION RemSegment(seg : pSegment) : BOOLEAN;
PROCEDURE ReplyPkt(dp : pDosPacket; res1 : LONGINT; res2 : LONGINT);
FUNCTION RunCommand(seg : LONGINT; stack : LONGINT;const paramptr : pCHAR; paramlen : LONGINT) : LONGINT;
FUNCTION SameDevice(lock1 : LONGINT; lock2 : LONGINT) : BOOLEAN;
FUNCTION SameLock(lock1 : LONGINT; lock2 : LONGINT) : LONGINT;
FUNCTION SelectInput(fh : LONGINT) : LONGINT;
FUNCTION SelectOutput(fh : LONGINT) : LONGINT;
PROCEDURE SendPkt(dp : pDosPacket; port : pMsgPort; replyport : pMsgPort);
FUNCTION SetArgStr(const string_ : pCHAR) : BOOLEAN;
FUNCTION SetComment(const name : pCHAR;const comment : pCHAR) : BOOLEAN;
FUNCTION SetConsoleTask(const task : pMsgPort) : pMsgPort;
FUNCTION SetCurrentDirName(const name : pCHAR) : BOOLEAN;
FUNCTION SetFileDate(const name : pCHAR; date : pDateStamp) : BOOLEAN;
FUNCTION SetFileSize(fh : LONGINT; pos : LONGINT; mode : LONGINT) : BOOLEAN;
FUNCTION SetFileSysTask(const task : pMsgPort) : pMsgPort;
FUNCTION SetIoErr(result : LONGINT) : LONGINT;
FUNCTION SetMode(fh : LONGINT; mode : LONGINT) : BOOLEAN;
FUNCTION SetOwner(const name : pCHAR; owner_info : LONGINT) : BOOLEAN;
FUNCTION SetProgramDir(lock : LONGINT) : LONGINT;
FUNCTION SetProgramName(const name : pCHAR) : BOOLEAN;
FUNCTION SetPrompt(const name : pCHAR) : BOOLEAN;
FUNCTION SetProtection(const name : pCHAR; protect : LONGINT) : BOOLEAN;
FUNCTION SetVar(const name : pCHAR; buffer : pCHAR; size : LONGINT; flags : LONGINT) : BOOLEAN;
FUNCTION SetVBuf(fh : LONGINT; buff : pCHAR; type_ : LONGINT; size : LONGINT) : BOOLEAN;
FUNCTION SplitName(const name : pCHAR; seperator : ULONG; buf : pCHAR; oldpos : LONGINT; size : LONGINT) : smallint;
FUNCTION StartNotify(notify : pNotifyRequest) : BOOLEAN;
FUNCTION StrToDate(datetime : pDateTime) : BOOLEAN;
FUNCTION StrToLong(const string_ : pCHAR; VAR value : LONGINT) : LONGINT;
FUNCTION SystemTagList(const command : pCHAR;const tags : pTagItem) : LONGINT;
FUNCTION DOSSystem(const command : pCHAR;const tags : pTagItem) : LONGINT;
FUNCTION UnGetC(fh : LONGINT; character : LONGINT) : LONGINT;
PROCEDURE UnLoadSeg(seglist : LONGINT);
PROCEDURE UnLock(lock : LONGINT);
PROCEDURE UnLockDosList(flags : ULONG);
FUNCTION UnLockRecord(fh : LONGINT; offset : ULONG; length : ULONG) : BOOLEAN;
FUNCTION UnLockRecords(recArray : pRecordLock) : BOOLEAN;
FUNCTION VFPrintf(fh : LONGINT;const format : pCHAR;const argarray : POINTER) : LONGINT;
PROCEDURE VFWritef(fh : LONGINT;const format : pCHAR;const argarray : pLONGINT);
FUNCTION VPrintf(const format : pCHAR; const argarray : POINTER) : LONGINT;
FUNCTION WaitForChar(file_ : LONGINT; timeout : LONGINT) : BOOLEAN;
FUNCTION WaitPkt : pDosPacket;
FUNCTION WriteChars(const buf : pCHAR; buflen : ULONG) : LONGINT;

FUNCTION BADDR(bval :BPTR): POINTER;
FUNCTION MKBADDR(adr: Pointer): BPTR;

{ overlay function and procedures}

FUNCTION AddBuffers(const name : string; number : LONGINT) : BOOLEAN;
FUNCTION AddPart(dirname : string;const filename : pCHAR; size : ULONG) : BOOLEAN;
FUNCTION AddPart(dirname : pCHAR;const filename : string; size : ULONG) : BOOLEAN;
FUNCTION AddPart(dirname : string;const filename : string; size : ULONG) : BOOLEAN;
FUNCTION AssignAdd(const name : string; lock : LONGINT) : BOOLEAN;
FUNCTION AssignLate(const name : string;const path : pCHAR) : BOOLEAN;
FUNCTION AssignLate(const name : pChar;const path : string) : BOOLEAN;
FUNCTION AssignLate(const name : string;const path : string) : BOOLEAN;
FUNCTION AssignLock(const name : string; lock : LONGINT) : BOOLEAN;
FUNCTION AssignPath(const name : string; const path : pCHAR) : BOOLEAN;
FUNCTION AssignPath(const name : pCHAR;const path : string) : BOOLEAN;
FUNCTION AssignPath(const name : string;const path : string) : BOOLEAN;
FUNCTION CreateDir(const name : string) : LONGINT;
FUNCTION CreateProc(const name : string; pri : LONGINT; segList : LONGINT; stackSize : LONGINT) : pMsgPort;
FUNCTION DeleteFile(const name : string) : BOOLEAN;
FUNCTION DeleteVar(const name : string; flags : ULONG) : BOOLEAN;
FUNCTION DeviceProc(const name : string) : pMsgPort;
FUNCTION DOSOpen(const name : string; accessMode : LONGINT) : LONGINT;
FUNCTION DOSRename(const oldName : string;const newName : pChar) : boolean;
FUNCTION DOSRename(const oldName : pCHAR;const newName : string) : Boolean;
FUNCTION DOSRename(const oldName : string;const newName : string) : Boolean;
FUNCTION Execute(const string_ : string; file_ : LONGINT; file2 : LONGINT) : BOOLEAN;
FUNCTION Fault(code : LONGINT; header : string; buffer : pCHAR; len : LONGINT) : BOOLEAN;
FUNCTION FilePart(const path : string) : pCHAR;
FUNCTION FindArg(const keyword : string;const arg_template : pCHAR) : LONGINT;
FUNCTION FindArg(const keyword : pCHAR;const arg_template : string) : LONGINT;
FUNCTION FindArg(const keyword : string;const arg_template : string) : LONGINT;
FUNCTION FindDosEntry(const dlist : pDosList;const name : string; flags : ULONG) : pDosList;
FUNCTION FindSegment(const name : string;const seg : pSegment; system : LONGINT) : pSegment;
FUNCTION FindVar(const name : string; type_ : ULONG) : pLocalVar;
FUNCTION Format(const filesystem : string;const volumename : pCHAR; dostype : ULONG) : BOOLEAN;
FUNCTION Format(const filesystem : pCHAR;const volumename : string; dostype : ULONG) : BOOLEAN;
FUNCTION Format(const filesystem : string;const volumename : string; dostype : ULONG) : BOOLEAN;
FUNCTION FPuts(fh : LONGINT;const str : string) : BOOLEAN;
FUNCTION GetDeviceProc(const name : string; dp : pDevProc) : pDevProc;
FUNCTION GetVar(const name : string; buffer : pCHAR; size : LONGINT; flags : LONGINT) : LONGINT;
FUNCTION Inhibit(const name : string; onoff : LONGINT) : BOOLEAN;
FUNCTION IsFileSystem(const name : string) : BOOLEAN;
FUNCTION LoadSeg(const name : string) : LONGINT;
FUNCTION Lock(const name : string; type_ : LONGINT) : LONGINT;
FUNCTION MakeDosEntry(const name : string; type_ : LONGINT) : pDosList;
FUNCTION MakeLink(const name : string; dest : LONGINT; soft : LONGINT) : BOOLEAN;
FUNCTION MatchFirst(const pat : string; anchor : pAnchorPath) : LONGINT;
FUNCTION MatchPattern(const pat : string; str : pCHAR) : BOOLEAN;
FUNCTION MatchPattern(const pat : pCHAR; str : string) : BOOLEAN;
FUNCTION MatchPattern(const pat : string; str : string) : BOOLEAN;
FUNCTION MatchPatternNoCase(const pat : string; str : pCHAR) : BOOLEAN;
FUNCTION MatchPatternNoCase(const pat : pCHAR; str : string) : BOOLEAN;
FUNCTION MatchPatternNoCase(const pat : string; str : string) : BOOLEAN;
FUNCTION NewLoadSeg(const file_ : string;const tags : pTagItem) : LONGINT;
FUNCTION NewLoadSegTagList(const file_ : string;const tags : pTagItem) : LONGINT;
FUNCTION PathPart(const path : string) : pCHAR;
FUNCTION PrintFault(code : LONGINT;const header : string) : BOOLEAN;
FUNCTION PutStr(const str : string) : BOOLEAN;
FUNCTION ReadArgs(const arg_template : string; arra : pLONGINT; args : pRDArgs) : pRDArgs;
FUNCTION ReadItem(const name : string; maxchars : LONGINT; cSource : pCSource) : LONGINT;
FUNCTION ReadLink(port : pMsgPort; lock : LONGINT;const path : string; buffer : pCHAR; size : ULONG) : BOOLEAN;
FUNCTION Relabel(const drive : string;const newname : pCHAR) : BOOLEAN;
FUNCTION Relabel(const drive : pCHAR;const newname : string) : BOOLEAN;
FUNCTION Relabel(const drive : string;const newname : string) : BOOLEAN;
FUNCTION RemAssignList(const name : string; lock : LONGINT) : BOOLEAN;
FUNCTION RunCommand(seg : LONGINT; stack : LONGINT;const paramptr : string; paramlen : LONGINT) : LONGINT;
FUNCTION SetArgStr(const string_ : string) : BOOLEAN;
FUNCTION SetComment(const name : string;const comment : pCHAR) : BOOLEAN;
FUNCTION SetComment(const name : pCHAR;const comment : string) : BOOLEAN;
FUNCTION SetComment(const name : string;const comment : string) : BOOLEAN;
FUNCTION SetCurrentDirName(const name : string) : BOOLEAN;
FUNCTION SetFileDate(const name : string; date : pDateStamp) : BOOLEAN;
FUNCTION SetOwner(const name : string; owner_info : LONGINT) : BOOLEAN;
FUNCTION SetProgramName(const name : string) : BOOLEAN;
FUNCTION SetPrompt(const name : string) : BOOLEAN;
FUNCTION SetProtection(const name : string; protect : LONGINT) : BOOLEAN;
FUNCTION SetVar(const name : string; buffer : pCHAR; size : LONGINT; flags : LONGINT) : BOOLEAN;
FUNCTION SplitName(const name : string; seperator : ULONG; buf : pCHAR; oldpos : LONGINT; size : LONGINT) : smallint;
FUNCTION StrToLong(const string_ : string; VAR value : LONGINT) : LONGINT;
FUNCTION SystemTagList(const command : string;const tags : pTagItem) : LONGINT;
FUNCTION DOSSystem(const command : string;const tags : pTagItem) : LONGINT;

IMPLEMENTATION

uses pastoc;



FUNCTION BADDR(bval : BPTR): POINTER;
BEGIN
    BADDR := POINTER( bval shl 2);
END;

FUNCTION MKBADDR(adr : POINTER): BPTR;
BEGIN
    MKBADDR := BPTR( LONGINT(adr) shr 2);
END;

PROCEDURE AbortPkt(port : pMsgPort; pkt : pDosPacket);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  pkt,D2
    MOVEA.L _DOSBase,A6
    JSR -264(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AddBuffers(const name : pCHAR; number : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  number,D2
    MOVEA.L _DOSBase,A6
    JSR -732(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AddDosEntry(dlist : pDosList) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dlist,D1
    MOVEA.L _DOSBase,A6
    JSR -678(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AddPart(dirname : pCHAR;const filename : pCHAR; size : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dirname,D1
    MOVE.L  filename,D2
    MOVE.L  size,D3
    MOVEA.L _DOSBase,A6
    JSR -882(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AddSegment(const name : pCHAR; seg : LONGINT; system : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  seg,D2
    MOVE.L  system,D3
    MOVEA.L _DOSBase,A6
    JSR -774(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AllocDosObject(type_ : ULONG;const tags : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  type_,D1
    MOVE.L  tags,D2
    MOVEA.L _DOSBase,A6
    JSR -228(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocDosObjectTagList(type_ : ULONG;const tags : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  type_,D1
    MOVE.L  tags,D2
    MOVEA.L _DOSBase,A6
    JSR -228(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AssignAdd(const name : pCHAR; lock : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  lock,D2
    MOVEA.L _DOSBase,A6
    JSR -630(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AssignLate(const name : pCHAR;const path : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  path,D2
    MOVEA.L _DOSBase,A6
    JSR -618(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AssignLock(const name : pCHAR; lock : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  lock,D2
    MOVEA.L _DOSBase,A6
    JSR -612(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AssignPath(const name : pCHAR;const path : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  path,D2
    MOVEA.L _DOSBase,A6
    JSR -624(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AttemptLockDosList(flags : ULONG) : pDosList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  flags,D1
    MOVEA.L _DOSBase,A6
    JSR -666(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ChangeMode(type_ : LONGINT; fh : LONGINT; newmode : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  type_,D1
    MOVE.L  fh,D2
    MOVE.L  newmode,D3
    MOVEA.L _DOSBase,A6
    JSR -450(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION CheckSignal(mask : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  mask,D1
    MOVEA.L _DOSBase,A6
    JSR -792(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION Cli : pCommandLineInterface;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -492(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CliInitNewcli(dp : pDosPacket) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dp,A0
    MOVEA.L _DOSBase,A6
    JSR -930(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CliInitRun(dp : pDosPacket) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dp,A0
    MOVEA.L _DOSBase,A6
    JSR -936(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CompareDates(const date1 : pDateStamp;const date2 : pDateStamp) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  date1,D1
    MOVE.L  date2,D2
    MOVEA.L _DOSBase,A6
    JSR -738(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateDir(const name : pCHAR) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateNewProc(const tags : pTagItem) : pProcess;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  tags,D1
    MOVEA.L _DOSBase,A6
    JSR -498(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateNewProcTagList(const tags : pTagItem) : pProcess;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  tags,D1
    MOVEA.L _DOSBase,A6
    JSR -498(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateProc(const name : pCHAR; pri : LONGINT; segList : LONGINT; stackSize : LONGINT) : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  pri,D2
    MOVE.L  segList,D3
    MOVE.L  stackSize,D4
    MOVEA.L _DOSBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CurrentDir(lock : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVEA.L _DOSBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DateStamp(date : pDateStamp);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  date,D1
    MOVEA.L _DOSBase,A6
    JSR -192(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DateToStr(datetime : pDateTime) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  datetime,D1
    MOVEA.L _DOSBase,A6
    JSR -744(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION DeleteFile(const name : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION DeleteVar(const name : pCHAR; flags : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  flags,D2
    MOVEA.L _DOSBase,A6
    JSR -912(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION DeviceProc(const name : pCHAR) : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoPkt(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT; arg3 : LONGINT; arg4 : LONGINT; arg5 : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  action,D2
    MOVE.L  arg1,D3
    MOVE.L  arg2,D4
    MOVE.L  arg3,D5
    MOVE.L  arg4,D6
    MOVE.L  arg5,D7
    MOVEA.L _DOSBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoPkt0(port : pMsgPort; action : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  action,D2
    MOVEA.L _DOSBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoPkt1(port : pMsgPort; action : LONGINT; arg1 : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  action,D2
    MOVE.L  arg1,D3
    MOVEA.L _DOSBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoPkt2(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  action,D2
    MOVE.L  arg1,D3
    MOVE.L  arg2,D4
    MOVEA.L _DOSBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoPkt3(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT; arg3 : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  action,D2
    MOVE.L  arg1,D3
    MOVE.L  arg2,D4
    MOVE.L  arg3,D5
    MOVEA.L _DOSBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoPkt4(port : pMsgPort; action : LONGINT; arg1 : LONGINT; arg2 : LONGINT; arg3 : LONGINT; arg4 : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  action,D2
    MOVE.L  arg1,D3
    MOVE.L  arg2,D4
    MOVE.L  arg3,D5
    MOVE.L  arg4,D6
    MOVEA.L _DOSBase,A6
    JSR -240(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DOSClose(file_ : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVEA.L _DOSBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DOSDelay(timeout : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  timeout,D1
    MOVEA.L _DOSBase,A6
    JSR -198(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DOSExit(returnCode : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  returnCode,D1
    MOVEA.L _DOSBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DOSFlush(fh : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVEA.L _DOSBase,A6
    JSR -360(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION DOSInput : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DOSOpen(const name : pCHAR; accessMode : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  accessMode,D2
    MOVEA.L _DOSBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DOSOutput : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DOSRead(file_ : LONGINT; buffer : POINTER; length : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVE.L  buffer,D2
    MOVE.L  length,D3
    MOVEA.L _DOSBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DOSRename(const oldName : pCHAR;const newName : pCHAR) : Boolean;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  oldName,D1
    MOVE.L  newName,D2
    MOVEA.L _DOSBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION DOSSeek(file_ : LONGINT; position : LONGINT; offset : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVE.L  position,D2
    MOVE.L  offset,D3
    MOVEA.L _DOSBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DOSWrite(file_ : LONGINT; buffer : POINTER; length : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVE.L  buffer,D2
    MOVE.L  length,D3
    MOVEA.L _DOSBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DupLock(lock : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVEA.L _DOSBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DupLockFromFH(fh : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVEA.L _DOSBase,A6
    JSR -372(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE EndNotify(notify : pNotifyRequest);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  notify,D1
    MOVEA.L _DOSBase,A6
    JSR -894(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ErrorReport(code : LONGINT; type_ : LONGINT; arg1 : ULONG; device : pMsgPort) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  code,D1
    MOVE.L  type_,D2
    MOVE.L  arg1,D3
    MOVE.L  device,D4
    MOVEA.L _DOSBase,A6
    JSR -480(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION ExAll(lock : LONGINT; buffer : pExAllData; size : LONGINT; data : LONGINT; control : pExAllControl) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVE.L  buffer,D2
    MOVE.L  size,D3
    MOVE.L  data,D4
    MOVE.L  control,D5
    MOVEA.L _DOSBase,A6
    JSR -432(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE ExAllEnd(lock : LONGINT; buffer : pExAllData; size : LONGINT; data : LONGINT; control : pExAllControl);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVE.L  buffer,D2
    MOVE.L  size,D3
    MOVE.L  data,D4
    MOVE.L  control,D5
    MOVEA.L _DOSBase,A6
    JSR -990(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION Examine(lock : LONGINT; fileInfoBlock : pFileInfoBlock) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVE.L  fileInfoBlock,D2
    MOVEA.L _DOSBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION ExamineFH(fh : LONGINT; fib : pFileInfoBlock) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  fib,D2
    MOVEA.L _DOSBase,A6
    JSR -390(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION Execute(const string_ : pCHAR; file_ : LONGINT; file2 : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  string_,D1
    MOVE.L  file_,D2
    MOVE.L  file2,D3
    MOVEA.L _DOSBase,A6
    JSR -222(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION ExNext(lock : LONGINT; fileInfoBlock : pFileInfoBlock) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVE.L  fileInfoBlock,D2
    MOVEA.L _DOSBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION Fault(code : LONGINT; header : pCHAR; buffer : pCHAR; len : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  code,D1
    MOVE.L  header,D2
    MOVE.L  buffer,D3
    MOVE.L  len,D4
    MOVEA.L _DOSBase,A6
    JSR -468(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION FGetC(fh : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVEA.L _DOSBase,A6
    JSR -306(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FGets(fh : LONGINT; buf : pCHAR; buflen : ULONG) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  buf,D2
    MOVE.L  buflen,D3
    MOVEA.L _DOSBase,A6
    JSR -336(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FilePart(const path : pCHAR) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  path,D1
    MOVEA.L _DOSBase,A6
    JSR -870(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindArg(const keyword : pCHAR;const arg_template : pCHAR) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  keyword,D1
    MOVE.L  arg_template,D2
    MOVEA.L _DOSBase,A6
    JSR -804(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindCliProc(num : ULONG) : pProcess;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  num,D1
    MOVEA.L _DOSBase,A6
    JSR -546(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindDosEntry(const dlist : pDosList;const name : pCHAR; flags : ULONG) : pDosList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dlist,D1
    MOVE.L  name,D2
    MOVE.L  flags,D3
    MOVEA.L _DOSBase,A6
    JSR -684(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindSegment(const name : pCHAR;const seg : pSegment; system : LONGINT) : pSegment;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  seg,D2
    MOVE.L  system,D3
    MOVEA.L _DOSBase,A6
    JSR -780(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindVar(const name : pCHAR; type_ : ULONG) : pLocalVar;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  type_,D2
    MOVEA.L _DOSBase,A6
    JSR -918(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION Format(const filesystem : pCHAR;const volumename : pCHAR; dostype : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  filesystem,D1
    MOVE.L  volumename,D2
    MOVE.L  dostype,D3
    MOVEA.L _DOSBase,A6
    JSR -714(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION FPutC(fh : LONGINT; ch : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  ch,D2
    MOVEA.L _DOSBase,A6
    JSR -312(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FPuts(fh : LONGINT;const str : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  str,D2
    MOVEA.L _DOSBase,A6
    JSR -342(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION FRead(fh : LONGINT; block : POINTER; blocklen : ULONG; number : ULONG) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  block,D2
    MOVE.L  blocklen,D3
    MOVE.L  number,D4
    MOVEA.L _DOSBase,A6
    JSR -324(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FreeArgs(args : pRDArgs);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  args,D1
    MOVEA.L _DOSBase,A6
    JSR -858(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeDeviceProc(dp : pDevProc);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dp,D1
    MOVEA.L _DOSBase,A6
    JSR -648(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeDosEntry(dlist : pDosList);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dlist,D1
    MOVEA.L _DOSBase,A6
    JSR -702(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeDosObject(type_ : ULONG; ptr : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  type_,D1
    MOVE.L  ptr,D2
    MOVEA.L _DOSBase,A6
    JSR -234(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FWrite(fh : LONGINT; block : POINTER; blocklen : ULONG; number : ULONG) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  block,D2
    MOVE.L  blocklen,D3
    MOVE.L  number,D4
    MOVEA.L _DOSBase,A6
    JSR -330(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetArgStr : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -534(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetConsoleTask : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -510(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetCurrentDirName(buf : pCHAR; len : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  buf,D1
    MOVE.L  len,D2
    MOVEA.L _DOSBase,A6
    JSR -564(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION GetDeviceProc(const name : pCHAR; dp : pDevProc) : pDevProc;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  dp,D2
    MOVEA.L _DOSBase,A6
    JSR -642(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetFileSysTask : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -522(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetProgramDir : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -600(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetProgramName(buf : pCHAR; len : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  buf,D1
    MOVE.L  len,D2
    MOVEA.L _DOSBase,A6
    JSR -576(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION GetPrompt(buf : pCHAR; len : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  buf,D1
    MOVE.L  len,D2
    MOVEA.L _DOSBase,A6
    JSR -588(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION GetVar(const name : pCHAR; buffer : pCHAR; size : LONGINT; flags : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  buffer,D2
    MOVE.L  size,D3
    MOVE.L  flags,D4
    MOVEA.L _DOSBase,A6
    JSR -906(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION Info(lock : LONGINT; parameterBlock : pInfoData) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVE.L  parameterBlock,D2
    MOVEA.L _DOSBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION Inhibit(const name : pCHAR; onoff : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  onoff,D2
    MOVEA.L _DOSBase,A6
    JSR -726(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION InternalLoadSeg(fh : LONGINT; table : LONGINT;const funcarray : pLONGINT; VAR stack : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D0
    MOVEA.L table,A0
    MOVEA.L funcarray,A1
    MOVEA.L stack,A2
    MOVEA.L _DOSBase,A6
    JSR -756(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION InternalUnLoadSeg(seglist : LONGINT; freefunc : tPROCEDURE) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  seglist,D1
    MOVEA.L freefunc,A1
    MOVEA.L _DOSBase,A6
    JSR -762(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IoErr : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IsFileSystem(const name : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -708(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsInteractive(file_ : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVEA.L _DOSBase,A6
    JSR -216(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LoadSeg(const name : pCHAR) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION Lock(const name : pCHAR; type_ : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  type_,D2
    MOVEA.L _DOSBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LockDosList(flags : ULONG) : pDosList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  flags,D1
    MOVEA.L _DOSBase,A6
    JSR -654(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LockRecord(fh : LONGINT; offset : ULONG; length : ULONG; mode : ULONG; timeout : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  offset,D2
    MOVE.L  length,D3
    MOVE.L  mode,D4
    MOVE.L  timeout,D5
    MOVEA.L _DOSBase,A6
    JSR -270(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LockRecords(recArray : pRecordLock; timeout : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  recArray,D1
    MOVE.L  timeout,D2
    MOVEA.L _DOSBase,A6
    JSR -276(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MakeDosEntry(const name : pCHAR; type_ : LONGINT) : pDosList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  type_,D2
    MOVEA.L _DOSBase,A6
    JSR -696(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MakeLink(const name : pCHAR; dest : LONGINT; soft : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  dest,D2
    MOVE.L  soft,D3
    MOVEA.L _DOSBase,A6
    JSR -444(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE MatchEnd(anchor : pAnchorPath);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  anchor,D1
    MOVEA.L _DOSBase,A6
    JSR -834(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MatchFirst(const pat : pCHAR; anchor : pAnchorPath) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  pat,D1
    MOVE.L  anchor,D2
    MOVEA.L _DOSBase,A6
    JSR -822(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MatchNext(anchor : pAnchorPath) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  anchor,D1
    MOVEA.L _DOSBase,A6
    JSR -828(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MatchPattern(const pat : pCHAR; str : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  pat,D1
    MOVE.L  str,D2
    MOVEA.L _DOSBase,A6
    JSR -846(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MatchPatternNoCase(const pat : pCHAR; str : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  pat,D1
    MOVE.L  str,D2
    MOVEA.L _DOSBase,A6
    JSR -972(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MaxCli : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -552(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NameFromFH(fh : LONGINT; buffer : pCHAR; len : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  buffer,D2
    MOVE.L  len,D3
    MOVEA.L _DOSBase,A6
    JSR -408(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION NameFromLock(lock : LONGINT; buffer : pCHAR; len : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVE.L  buffer,D2
    MOVE.L  len,D3
    MOVEA.L _DOSBase,A6
    JSR -402(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION NewLoadSeg(const file_ : pCHAR;const tags : pTagItem) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVE.L  tags,D2
    MOVEA.L _DOSBase,A6
    JSR -768(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NewLoadSegTagList(const file_ : pCHAR;const tags : pTagItem) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVE.L  tags,D2
    MOVEA.L _DOSBase,A6
    JSR -768(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NextDosEntry(const dlist : pDosList; flags : ULONG) : pDosList;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dlist,D1
    MOVE.L  flags,D2
    MOVEA.L _DOSBase,A6
    JSR -690(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenFromLock(lock : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVEA.L _DOSBase,A6
    JSR -378(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ParentDir(lock : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVEA.L _DOSBase,A6
    JSR -210(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ParentOfFH(fh : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVEA.L _DOSBase,A6
    JSR -384(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ParsePattern(const pat : pCHAR; buf : pCHAR; buflen : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  pat,D1
    MOVE.L  buf,D2
    MOVE.L  buflen,D3
    MOVEA.L _DOSBase,A6
    JSR -840(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ParsePatternNoCase(const pat : pCHAR; buf : pCHAR; buflen : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  pat,D1
    MOVE.L  buf,D2
    MOVE.L  buflen,D3
    MOVEA.L _DOSBase,A6
    JSR -966(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PathPart(const path : pCHAR) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  path,D1
    MOVEA.L _DOSBase,A6
    JSR -876(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PrintFault(code : LONGINT;const header : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  code,D1
    MOVE.L  header,D2
    MOVEA.L _DOSBase,A6
    JSR -474(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PutStr(const str : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  str,D1
    MOVEA.L _DOSBase,A6
    JSR -948(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION ReadArgs(const arg_template : pCHAR; arra : pLONGINT; args : pRDArgs) : pRDArgs;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  arg_template,D1
    MOVE.L  arra,D2
    MOVE.L  args,D3
    MOVEA.L _DOSBase,A6
    JSR -798(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ReadItem(const name : pCHAR; maxchars : LONGINT; cSource : pCSource) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  maxchars,D2
    MOVE.L  cSource,D3
    MOVEA.L _DOSBase,A6
    JSR -810(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ReadLink(port : pMsgPort; lock : LONGINT;const path : pCHAR; buffer : pCHAR; size : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D1
    MOVE.L  lock,D2
    MOVE.L  path,D3
    MOVE.L  buffer,D4
    MOVE.L  size,D5
    MOVEA.L _DOSBase,A6
    JSR -438(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION Relabel(const drive : pCHAR;const newname : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  drive,D1
    MOVE.L  newname,D2
    MOVEA.L _DOSBase,A6
    JSR -720(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION RemAssignList(const name : pCHAR; lock : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  lock,D2
    MOVEA.L _DOSBase,A6
    JSR -636(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION RemDosEntry(dlist : pDosList) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dlist,D1
    MOVEA.L _DOSBase,A6
    JSR -672(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION RemSegment(seg : pSegment) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  seg,D1
    MOVEA.L _DOSBase,A6
    JSR -786(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE ReplyPkt(dp : pDosPacket; res1 : LONGINT; res2 : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dp,D1
    MOVE.L  res1,D2
    MOVE.L  res2,D3
    MOVEA.L _DOSBase,A6
    JSR -258(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RunCommand(seg : LONGINT; stack : LONGINT;const paramptr : pCHAR; paramlen : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  seg,D1
    MOVE.L  stack,D2
    MOVE.L  paramptr,D3
    MOVE.L  paramlen,D4
    MOVEA.L _DOSBase,A6
    JSR -504(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SameDevice(lock1 : LONGINT; lock2 : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock1,D1
    MOVE.L  lock2,D2
    MOVEA.L _DOSBase,A6
    JSR -984(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SameLock(lock1 : LONGINT; lock2 : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock1,D1
    MOVE.L  lock2,D2
    MOVEA.L _DOSBase,A6
    JSR -420(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SelectInput(fh : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVEA.L _DOSBase,A6
    JSR -294(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SelectOutput(fh : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVEA.L _DOSBase,A6
    JSR -300(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SendPkt(dp : pDosPacket; port : pMsgPort; replyport : pMsgPort);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  dp,D1
    MOVE.L  port,D2
    MOVE.L  replyport,D3
    MOVEA.L _DOSBase,A6
    JSR -246(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetArgStr(const string_ : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  string_,D1
    MOVEA.L _DOSBase,A6
    JSR -540(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetComment(const name : pCHAR;const comment : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  comment,D2
    MOVEA.L _DOSBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetConsoleTask(const task : pMsgPort) : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  task,D1
    MOVEA.L _DOSBase,A6
    JSR -516(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetCurrentDirName(const name : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -558(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetFileDate(const name : pCHAR; date : pDateStamp) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  date,D2
    MOVEA.L _DOSBase,A6
    JSR -396(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetFileSize(fh : LONGINT; pos : LONGINT; mode : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  pos,D2
    MOVE.L  mode,D3
    MOVEA.L _DOSBase,A6
    JSR -456(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetFileSysTask(const task : pMsgPort) : pMsgPort;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  task,D1
    MOVEA.L _DOSBase,A6
    JSR -528(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetIoErr(result : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  result,D1
    MOVEA.L _DOSBase,A6
    JSR -462(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetMode(fh : LONGINT; mode : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  mode,D2
    MOVEA.L _DOSBase,A6
    JSR -426(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetOwner(const name : pCHAR; owner_info : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  owner_info,D2
    MOVEA.L _DOSBase,A6
    JSR -996(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetProgramDir(lock : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVEA.L _DOSBase,A6
    JSR -594(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetProgramName(const name : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -570(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetPrompt(const name : pCHAR) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVEA.L _DOSBase,A6
    JSR -582(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetProtection(const name : pCHAR; protect : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  protect,D2
    MOVEA.L _DOSBase,A6
    JSR -186(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetVar(const name : pCHAR; buffer : pCHAR; size : LONGINT; flags : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  buffer,D2
    MOVE.L  size,D3
    MOVE.L  flags,D4
    MOVEA.L _DOSBase,A6
    JSR -900(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SetVBuf(fh : LONGINT; buff : pCHAR; type_ : LONGINT; size : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  buff,D2
    MOVE.L  type_,D3
    MOVE.L  size,D4
    MOVEA.L _DOSBase,A6
    JSR -366(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION SplitName(const name : pCHAR; seperator : ULONG; buf : pCHAR; oldpos : LONGINT; size : LONGINT) : smallint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D1
    MOVE.L  seperator,D2
    MOVE.L  buf,D3
    MOVE.L  oldpos,D4
    MOVE.L  size,D5
    MOVEA.L _DOSBase,A6
    JSR -414(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION StartNotify(notify : pNotifyRequest) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  notify,D1
    MOVEA.L _DOSBase,A6
    JSR -888(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION StrToDate(datetime : pDateTime) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  datetime,D1
    MOVEA.L _DOSBase,A6
    JSR -750(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION StrToLong(const string_ : pCHAR; VAR value : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  string_,D1
    MOVE.L  value,D2
    MOVEA.L _DOSBase,A6
    JSR -816(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SystemTagList(const command : pCHAR;const tags : pTagItem) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  command,D1
    MOVE.L  tags,D2
    MOVEA.L _DOSBase,A6
    JSR -606(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DOSSystem(const command : pCHAR;const tags : pTagItem) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  command,D1
    MOVE.L  tags,D2
    MOVEA.L _DOSBase,A6
    JSR -606(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION UnGetC(fh : LONGINT; character : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  character,D2
    MOVEA.L _DOSBase,A6
    JSR -318(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE UnLoadSeg(seglist : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  seglist,D1
    MOVEA.L _DOSBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnLock(lock : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lock,D1
    MOVEA.L _DOSBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnLockDosList(flags : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  flags,D1
    MOVEA.L _DOSBase,A6
    JSR -660(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION UnLockRecord(fh : LONGINT; offset : ULONG; length : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  offset,D2
    MOVE.L  length,D3
    MOVEA.L _DOSBase,A6
    JSR -282(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION UnLockRecords(recArray : pRecordLock) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  recArray,D1
    MOVEA.L _DOSBase,A6
    JSR -288(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION VFPrintf(fh : LONGINT;const format : pCHAR;const argarray : POINTER) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  format,D2
    MOVE.L  argarray,D3
    MOVEA.L _DOSBase,A6
    JSR -354(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE VFWritef(fh : LONGINT;const format : pCHAR;const argarray : pLONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  fh,D1
    MOVE.L  format,D2
    MOVE.L  argarray,D3
    MOVEA.L _DOSBase,A6
    JSR -348(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION VPrintf(const format : pCHAR; const argarray : POINTER) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  format,D1
    MOVE.L  argarray,D2
    MOVEA.L _DOSBase,A6
    JSR -954(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WaitForChar(file_ : LONGINT; timeout : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  file_,D1
    MOVE.L  timeout,D2
    MOVEA.L _DOSBase,A6
    JSR -204(A6)
    MOVEA.L (A7)+,A6
    TST.L   D0
    BEQ.B   @end
    MOVEQ   #1,D0
    @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION WaitPkt : pDosPacket;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L _DOSBase,A6
    JSR -252(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WriteChars(const buf : pCHAR; buflen : ULONG) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  buf,D1
    MOVE.L  buflen,D2
    MOVEA.L _DOSBase,A6
    JSR -942(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;


FUNCTION AddBuffers(const name : string; number : LONGINT) : BOOLEAN;
begin
     AddBuffers := AddBuffers(pas2c(name), number);
end;

FUNCTION AddPart(dirname : string;const filename : pCHAR; size : ULONG) : BOOLEAN;
begin
     AddPart := AddPart(pas2c(dirname),filename,size);
end;

FUNCTION AddPart(dirname : pCHAR;const filename : string; size : ULONG) : BOOLEAN;
begin
     AddPart := AddPart(dirname,pas2c(filename),size);
end;

FUNCTION AddPart(dirname : string;const filename : string; size : ULONG) : BOOLEAN;
begin
     AddPart := AddPart(pas2c(dirname),pas2c(filename),size);
end;

FUNCTION AssignAdd(const name : string; lock : LONGINT) : BOOLEAN;
begin
     AssignAdd := AssignAdd(pas2c(name),lock);
end;

FUNCTION AssignLate(const name : string;const path : pCHAR) : BOOLEAN;
begin
     AssignLate := AssignLate(pas2c(name),path);
end;

FUNCTION AssignLate(const name : pChar;const  path : string) : BOOLEAN;
begin
     AssignLate := AssignLate(name,pas2c(path));
end;

FUNCTION AssignLate(const name : string;const path : string) : BOOLEAN;
begin
     AssignLate := AssignLate(pas2c(name),pas2c(path));
end;

FUNCTION AssignLock(const name : string; lock : LONGINT) : BOOLEAN;
begin
     AssignLock := AssignLock(pas2c(name),lock);
end;

FUNCTION AssignPath(const name : string;const path : pCHAR) : BOOLEAN;
begin
     AssignPath := AssignPath(pas2c(name),path);
end;

FUNCTION AssignPath(const name : pCHAR;const path : string) : BOOLEAN;
begin
     AssignPath := AssignPath(name,pas2c(path));
end;

FUNCTION AssignPath(const name : string;const path : string) : BOOLEAN;
begin
     AssignPath := AssignPath(pas2c(name),pas2c(path));
end;

FUNCTION CreateDir(const name : string) : LONGINT;
begin
     CreateDir := CreateDir(pas2c(name));
end;

FUNCTION CreateProc(const name : string; pri : LONGINT; segList : LONGINT; stackSize : LONGINT) : pMsgPort;
begin
     CreateProc := CreateProc(pas2c(name),pri,segList,stackSize);
end;

FUNCTION DeleteFile(const name : string) : BOOLEAN;
begin
     DeleteFile := DeleteFile(pas2c(name));
end;

FUNCTION DeleteVar(const name : string; flags : ULONG) : BOOLEAN;
begin
     DeleteVar := DeleteVar(pas2c(name),flags);
end;

FUNCTION DeviceProc(const name : string) : pMsgPort;
begin
     Deviceproc := DeviceProc(pas2c(name));
end;

FUNCTION DOSOpen(const name : string; accessMode : LONGINT) : LONGINT;
begin
     DOSOpen := DOSOpen(pas2c(name),accessMode);
end;

FUNCTION DOSRename(const oldName : string;const newName : pCHAR) : Boolean;
begin
     DOSRename := DOSRename(pas2c(oldName),newName);
end;

FUNCTION DOSRename(const oldName : pCHAR;const newName : string) : Boolean;
begin
     DOSRename := DOSRename(oldName,pas2c(newName));
end;

FUNCTION DOSRename(const oldName : string;const newName : string) : Boolean;
begin
     DOSRename := DOSRename(pas2c(oldName),pas2c(newName));
end;

FUNCTION Execute(const string_ : string; file_ : LONGINT; file2 : LONGINT) : BOOLEAN;
begin
     Execute := Execute(pas2c(string_),file_ ,file2);
end;

FUNCTION Fault(code : LONGINT; header : string; buffer : pCHAR; len : LONGINT) : BOOLEAN;
begin
    Fault := Fault(code,pas2c(header),buffer,len);
end;

FUNCTION FilePart(const path : string) : pCHAR;
begin
    FilePart := FilePart(pas2c(path));
end;

FUNCTION FindArg(const keyword : string;const arg_template : pCHAR) : LONGINT;
begin
    FindArg := FindArg(pas2c(keyword),arg_template);
end;

FUNCTION FindArg(const keyword : pCHAR;const arg_template : string) : LONGINT;
begin
    FindArg := FindArg(keyword,pas2c(arg_template));
end;

FUNCTION FindArg(const keyword : string;const arg_template : string) : LONGINT;
begin
    FindArg := FindArg(pas2c(keyword),pas2c(arg_template));
end;

FUNCTION FindDosEntry(const dlist : pDosList;const name : string; flags : ULONG) : pDosList;
begin
    FindDosEntry := FindDosEntry(dlist,pas2c(name),flags);
end;

FUNCTION FindSegment(const name : string;const seg : pSegment; system : LONGINT) : pSegment;
begin
    FindSegment := FindSegment(pas2c(name),seg,system);
end;

FUNCTION FindVar(const name : string; type_ : ULONG) : pLocalVar;
begin
    FindVar := FindVar(pas2c(name),type_);
end;

FUNCTION Format(const filesystem : string;const volumename : pCHAR; dostype : ULONG) : BOOLEAN;
begin
    Format := Format(pas2c(filesystem),volumename,dostype);
end;

FUNCTION Format(const filesystem : pCHAR;const volumename : string; dostype : ULONG) : BOOLEAN;
begin
    Format := Format(filesystem,pas2c(volumename),dostype);
end;

FUNCTION Format(const filesystem : string;const volumename : string; dostype : ULONG) : BOOLEAN;
begin
    Format := Format(pas2c(filesystem),pas2c(volumename),dostype);
end;

FUNCTION FPuts(fh : LONGINT;const str : string) : BOOLEAN;
begin
    FPuts := FPuts(fh,pas2c(str));
end;

FUNCTION GetDeviceProc(const name : string; dp : pDevProc) : pDevProc;
begin
    GetDeviceProc := GetDeviceProc(pas2c(name),dp);
end;

FUNCTION GetVar(const name : string; buffer : pCHAR; size : LONGINT; flags : LONGINT) : LONGINT;
begin
    GetVar := GetVar(pas2c(name),buffer,size,flags);
end;

FUNCTION Inhibit(const name : string; onoff : LONGINT) : BOOLEAN;
begin
    Inhibit := Inhibit(pas2c(name),onoff);
end;

FUNCTION IsFileSystem(const name : string) : BOOLEAN;
begin
    IsFileSystem := IsFileSystem(pas2c(name));
end;

FUNCTION LoadSeg(const name : string) : LONGINT;
begin
    LoadSeg := LoadSeg(pas2c(name));
end;

FUNCTION Lock(const name : string; type_ : LONGINT) : LONGINT;
begin
    Lock := Lock(pas2c(name),type_);
end;

FUNCTION MakeDosEntry(const name : string; type_ : LONGINT) : pDosList;
begin
    MakeDosEntry := MakeDosEntry(pas2c(name),type_);
end;

FUNCTION MakeLink(const name : string; dest : LONGINT; soft : LONGINT) : BOOLEAN;
begin
    MakeLink := MakeLink(pas2c(name),dest,soft);
end;

FUNCTION MatchFirst(const pat : string; anchor : pAnchorPath) : LONGINT;
begin
    MatchFirst := MatchFirst(pas2c(pat),anchor);
end;

FUNCTION MatchPattern(const pat : string; str : pCHAR) : BOOLEAN;
begin
    MatchPattern := MatchPattern(pas2c(pat),str);
end;

FUNCTION MatchPattern(const pat : pCHAR; str : string) : BOOLEAN;
begin
    MatchPattern := MatchPattern(pat,pas2c(str));
end;

FUNCTION MatchPattern(const pat : string; str : string) : BOOLEAN;
begin
    MatchPattern := MatchPattern(pas2c(pat),pas2c(str));
end;

FUNCTION MatchPatternNoCase(const pat : string; str : pCHAR) : BOOLEAN;
begin
    MatchPatternNoCase := MatchPatternNoCase(pas2c(pat),str);
end;

FUNCTION MatchPatternNoCase(const pat : pCHAR; str : string) : BOOLEAN;
begin
    MatchPatternNoCase := MatchPatternNoCase(pat,pas2c(str));
end;

FUNCTION MatchPatternNoCase(const pat : string; str : string) : BOOLEAN;
begin
    MatchPatternNoCase := MatchPatternNoCase(pas2c(pat),pas2c(str));
end;

FUNCTION NewLoadSeg(const file_ : string;const tags : pTagItem) : LONGINT;
begin
    NewLoadSeg := NewLoadSeg(pas2c(file_),tags);
end;

FUNCTION NewLoadSegTagList(const file_ : string;const tags : pTagItem) : LONGINT;
begin
    NewLoadSegTagList := NewLoadSegTagList(pas2c(file_),tags);
end;

FUNCTION PathPart(const path : string) : pCHAR;
begin
    PathPart := PathPart(pas2c(path));
end;

FUNCTION PrintFault(code : LONGINT;const header : string) : BOOLEAN;
begin
    PrintFault := PrintFault(code,pas2c(header));
end;

FUNCTION PutStr(const str : string) : BOOLEAN;
begin
    PutStr := PutStr(pas2c(str));
end;

FUNCTION ReadArgs(const arg_template : string; arra : pLONGINT; args : pRDArgs) : pRDArgs;
begin
    ReadArgs := ReadArgs(pas2c(arg_template),arra,args);
end;

FUNCTION ReadItem(const name : string; maxchars : LONGINT; cSource : pCSource) : LONGINT;
begin
    ReadItem := ReadItem(pas2c(name),maxchars,cSource);
end;

FUNCTION ReadLink(port : pMsgPort; lock : LONGINT;const path : string; buffer : pCHAR; size : ULONG) : BOOLEAN;
begin
    ReadLink := ReadLink(port,lock,pas2c(path),buffer,size);
end;

FUNCTION Relabel(const drive : string;const newname : pCHAR) : BOOLEAN;
begin
    Relabel := Relabel(pas2c(drive),newname);
end;

FUNCTION Relabel(const drive : pCHAR;const newname : string) : BOOLEAN;
begin
    Relabel := Relabel(drive,pas2c(newname));
end;

FUNCTION Relabel(const drive : string;const newname : string) : BOOLEAN;
begin
    Relabel := Relabel(pas2c(drive),pas2c(newname));
end;

FUNCTION RemAssignList(const name : string; lock : LONGINT) : BOOLEAN;
begin
    RemAssignList := RemAssignList(pas2c(name),lock);
end;

FUNCTION RunCommand(seg : LONGINT; stack : LONGINT;const paramptr : string; paramlen : LONGINT) : LONGINT;
begin
    RunCommand := RunCommand(seg,stack,pas2c(paramptr),paramlen);
end;

FUNCTION SetArgStr(const string_ : string) : BOOLEAN;
begin
    SetArgStr := SetArgStr(pas2c(string_));
end;

FUNCTION SetComment(const name : string;const comment : pCHAR) : BOOLEAN;
begin
    SetComment := SetComment(pas2c(name),comment);
end;

FUNCTION SetComment(const name : pCHAR;const comment : string) : BOOLEAN;
begin
    SetComment := SetComment(name,pas2c(comment));
end;

FUNCTION SetComment(const name : string;const comment : string) : BOOLEAN;
begin
    SetComment := SetComment(pas2c(name),pas2c(comment));
end;

FUNCTION SetCurrentDirName(const name : string) : BOOLEAN;
begin
     SetCurrentDirName := SetCurrentDirName(pas2c(name));
end;

FUNCTION SetFileDate(const name : string; date : pDateStamp) : BOOLEAN;
begin
     SetFileDate := SetFileDate(pas2c(name),date);
end;

FUNCTION SetOwner(const name : string; owner_info : LONGINT) : BOOLEAN;
begin
     SetOwner := SetOwner(pas2c(name),owner_info);
end;

FUNCTION SetProgramName(const name : string) : BOOLEAN;
begin
     SetProgramName := SetProgramName(pas2c(name));
end;

FUNCTION SetPrompt(const name : string) : BOOLEAN;
begin
     SetPrompt := SetPrompt(pas2c(name));
end;

FUNCTION SetProtection(const name : string; protect : LONGINT) : BOOLEAN;
begin
     SetProtection := SetProtection(pas2c(name),protect);
end;

FUNCTION SetVar(const name : string; buffer : pCHAR; size : LONGINT; flags : LONGINT) : BOOLEAN;
begin
     SetVar := SetVar(pas2c(name),buffer,size,flags);
end;

FUNCTION SplitName(const name : string; seperator : ULONG; buf : pCHAR; oldpos : LONGINT; size : LONGINT) : smallint;
begin
     SplitName := SplitName(pas2c(name), seperator,buf,oldpos,size);
end;

FUNCTION StrToLong(const string_ : string; VAR value : LONGINT) : LONGINT;
begin
     StrToLong := StrToLong(pas2c(string_),value);
end;

FUNCTION SystemTagList(const command : string;const tags : pTagItem) : LONGINT;
begin
     SystemTagList := SystemTagList(pas2c(command),tags);
end;

FUNCTION DOSSystem(const command : string;const tags : pTagItem) : LONGINT;
begin
     DOSSystem := DOSSystem(pas2c(command),tags);
end;


END. (* UNIT DOS *)


