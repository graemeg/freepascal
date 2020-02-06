{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Free Pascal development team

    dos.library functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
 BSTR Funktions

 defines:
 AROS_FAST_BPTR: BPTR is a pointer or a 2 shifted Pointer
   -> atm its standard ABIv0, for v1 needs a ifdef
}

{$define AROS_FAST_BPTR}

unit amigados;

interface

uses
  exec, utility, timer;

{$PACKRECORDS C}

const
{ Predefined Amiga DOS global constants }
  DOSTRUE     = -1;
  DOSFALSE    =  0;
  TICKS_PER_SECOND = 50;   { Number of ticks in one second }

  // Still to TEST
  BITSPERBYTE         = 8;
  BYTESPERLONG        = 4;
  BITSPERLONG         = 32;

type
  FileHandle  = BPTR;
  FileLock    = BPTR;

{* All BCPL data must be long Integer aligned.  BCPL pointers are the long Integer
 *  address (i.e byte address divided by 4 (>>2)) *}

{* BCPL strings have a length in the first byte and then the characters.
 * For example:  s[0]=3 s[1]=S s[2]=Y s[3]=S                 *}



const
// DOS functions will return this when they reach EOF. */
  ENDSTREAMCH = -1;
// Buffering types for SetVBuf().
  BUF_LINE    = 0; // Flush at the end of lines '\n'.
  BUF_FULL    = 1; // Flush only when buffer is full.
  BUF_NONE    = 2; // Do not buffer, read and write immediatly.

type
  PDateStamp = ^TDateStamp;
  TDateStamp = record
    ds_Days: Longint;        { Number of days since Jan. 1, 1978 }
    ds_Minute: Longint;      { Number of minutes past midnight }
    ds_Tick: Longint;        { Number of ticks past minute }
  end;

const
{ The maximum length of filenames in AmigaOS. You should not depend on
  this value, as it may change in future versions.}
  MAXFILENAMELENGTH = 108;

{ The maximum length of comments in AmigaOS. You should not depend on
  this value, as it may change in future versions.}
  MAXCOMMENTLENGTH = 80;

type
{ Returned by Examine() and ExInfo(), must be on a 4 byte boundary
  Structure used to describe a directory entry. Note that not all fields
  are supported by all filesystems. This structure should be allocated
  with AllocDosObject(). }
  PFileInfoBlock = ^TFileInfoBlock;
  TFileInfoBlock = record
    fib_DiskKey: IPTR;
    fib_DirEntryType: LongInt;                              // type of Directory. If < 0, then a plain file. If > 0 a directory
    fib_FileName: array [0..MAXFILENAMELENGTH - 1] of Char; // Null terminated. Max 30 chars used for now
    fib_Protection: LongInt;                                // bit mask of protection, rwxd are 3-0.
    fib_EntryType: LongInt;
    fib_Size: LongInt;                                      // Number of bytes in file
    fib_NumBlocks: LongInt;                                 // Number of blocks in file
    fib_Date: TDateStamp;                                   // Date file last changed
    fib_Comment: array [0..MAXCOMMENTLENGTH - 1] of Char;   // Null terminated comment associated with file
    fib_OwnerUID: Word;                                     // UserID of fileowner.
    fib_OwnerGID: Word;                                     // GroupID of fileowner.
    fib_Reserved: array [0..31] of Char;                    // PRIVATE
  end;

const
{ FIB stands for TFileInfoBlock (fib_Protection)}

{ FIBB are bit definitions, FIBF are field definitions
  Regular RWED bits are 0 == allowed.
  NOTE: GRP and OTR RWED permissions are 0 == not allowed!
  Group and Other permissions are not directly handled by the filesystem}
  FIBB_DELETE         = 0;  // prevent file from being deleted }
  FIBB_EXECUTE        = 1;  // ignored by system, used by Shell }
  FIBB_WRITE          = 2;  // ignored by old filesystem }
  FIBB_READ           = 3;  // ignored by old filesystem }
  FIBB_ARCHIVE        = 4;  // cleared whenever file is changed }
  FIBB_PURE           = 5;  // program is reentrant and rexecutable}
  FIBB_SCRIPT         = 6;  // program is a script (execute) file }
// group flags
  FIBB_GRP_DELETE     = 8;  // Group: prevent file from being deleted *}
  FIBB_GRP_EXECUTE    = 9;  // Group: file is executable *}
  FIBB_GRP_WRITE      = 10; // Group: file is writable *}
  FIBB_GRP_READ       = 11; // Group: file is readable *}
// other
  FIBB_OTR_DELETE     = 12; // Other: prevent file from being deleted *}
  FIBB_OTR_EXECUTE    = 13; // Other: file is executable *}
  FIBB_OTR_WRITE      = 14; // Other: file is writable *}
  FIBB_OTR_READ       = 15; // Other: file is readable *}
// Values
  FIBF_DELETE         = (1 shl FIBB_DELETE);
  FIBF_EXECUTE        = (1 shl FIBB_EXECUTE);
  FIBF_WRITE          = (1 shl FIBB_WRITE);
  FIBF_READ           = (1 shl FIBB_READ);
  FIBF_ARCHIVE        = (1 shl FIBB_ARCHIVE);
  FIBF_PURE           = (1 shl FIBB_PURE);
  FIBF_SCRIPT         = (1 shl FIBB_SCRIPT);
// Group Values
  FIBF_GRP_DELETE    = (1 shl FIBB_GRP_DELETE);
  FIBF_GRP_EXECUTE   = (1 shl FIBB_GRP_EXECUTE);
  FIBF_GRP_WRITE     = (1 shl FIBB_GRP_WRITE);
  FIBF_GRP_READ      = (1 shl FIBB_GRP_READ);
// Other Values
  FIBF_OTR_DELETE    = (1 shl FIBB_OTR_DELETE);
  FIBF_OTR_EXECUTE   = (1 shl FIBB_OTR_EXECUTE);
  FIBF_OTR_WRITE     = (1 shl FIBB_OTR_WRITE);
  FIBF_OTR_READ      = (1 shl FIBB_OTR_READ);

// Devices
type
{ returned by Info(), must be on a 4 byte boundary }
  PInfoData = ^TInfoData;
  TInfoData = record
    id_NumSoftErrors: LongInt; // Number of soft errors on disk
    id_UnitNumber: LongInt;    // Which unit disk is (was) mounted on
    id_DiskState: LongInt;     // Dtate of Volume See defines below
    id_NumBlocks: LongInt;     // Number of blocks on device
    id_NumBlocksUsed: LongInt; // Number of block in use
    id_BytesPerBlock: LongInt; // Bytes per Block
    id_DiskType: LongInt;      // Type of Disk
    id_VolumeNode: BPTR;       // BCPL pointer to volume node
    id_InUse: IPTR;            // Flag, zero if not in use
  end;

  {$PACKRECORDS NORMAL}

const
  { ID stands for InfoData }
{ Disk states }
  ID_WRITE_PROTECTED  = 80;   // Disk is write protected
  ID_VALIDATING       = 81;   // Disk is currently being validated
  ID_VALIDATED        = 82;   // Disk is consistent and writeable
{ Filesystem types as used for id_DiskType. These are multi-character
  constants of identifier strings. They are self-descriptive.}
  ID_NO_DISK_PRESENT     = -1;
  ID_UNREADABLE_DISK     = $42414400; // 'BAD#0'
  ID_DOS_DISK            = $444F5300; // 'DOS#0'
  ID_FFS_DISK            = $444F5301; // 'DOS#1'
  ID_INTER_DOS_DISK      = $444F5302; // 'DOS#2'
  ID_INTER_FFS_DISK      = $444F5303; // 'DOS#3'
  ID_FASTDIR_DOS_DISK    = $444F5304; // 'DOS#4'
  ID_FASTDIR_FFS_DISK    = $444F5305; // 'DOS#5'
  ID_NOT_REALLY_DOS      = $4E444F53; // 'NDOS'
  ID_KICKSTART_DISK      = $4B49434B; // 'KICK'
  ID_MSDOS_DISK          = $4d534400; // 'MSD#0'
  ID_SFS_BE_DISK         = $53465300; // 'SFS#0'
  ID_SFS_LE_DISK         = $73667300; // 'sfs#0'
{ These are the return codes used by convention by AmigaDOS commands
  See FAILAT and IF for relvance to EXECUTE files}
    // No Problem, success
  RETURN_OK              =  0;
  { Program succeeded, but there was something not quite right.
    This value may also be used to express a boolean state
    (RETURN_WARN meaning TRUE, RETURN_OK meaning FALSE).}
  RETURN_WARN            =  5;
  { Program succeeded partly. This may be returned, if the user aborts a
    program or some external data were wrong.}
  RETURN_ERROR           = 10; // Something wrong
  { Program execution failed. Normally used, if some system resources could
    not be allocated.}
  RETURN_FAIL            = 20; // Complete or severe failure

{ Secondary errors codes as used for IoErr(), SetIoErr() and in
  Process^.pr_Result2. The term 'object' refers to files of all kinds
  (ie plain files, directories, links, etc).}

  {This is used, if something went wrong, but it is unknown what exactly
   went wrong. This is especially useful for emulation devices, when the
   underlying system returned an error that the emulation side does not
   know.}
  ERROR_UNKNOWN                  = 100;
//General system errors
  ERROR_NO_FREE_STORE            = 103; // Out of memory.
  ERROR_TASK_TABLE_FULL          = 105; // Too many tasks are already running.
//Errors concerning ReadArgs().
  ERROR_BAD_TEMPLATE             = 114; // Supplied template is broken
  ERROR_BAD_NUMBER               = 115; { A supplied argument that was expected to be numeric, was not numeric.
                                          This is also returned by some functions to expresss that a supplied
                                          number is out of range (ie to express application internal errors).}
  ERROR_REQUIRED_ARG_MISSING     = 116; // An argument that has to be supplied (ie signed with the '/A' flag) was not supplied.
  ERROR_KEY_NEEDS_ARG            = 117; // Keyword was specified, but not its contents.
  ERROR_TOO_MANY_ARGS            = 118; // There were more arguments than the template needs.
  ERROR_UNMATCHED_QUOTES         = 119; // An odd number of quotation marks was supplied.
  ERROR_LINE_TOO_LONG            = 120; { Either the command-line was longer than hardcoded line length limit or the
                                          maximum number of multiple arguments (flag '/M') was exceeded. This can
                                          also indicate that some argument is too long or a supplied buffer is too small.}
// Errors in files.
  ERROR_FILE_NOT_OBJECT          = 121; // You tried to execute a file that is not an executable.
  ERROR_INVALID_RESIDENT_LIBRARY = 122; // A library or device could not be opened or that library or device is broken.
  ERROR_NO_DEFAULT_DIR           = 201;
  ERROR_OBJECT_IN_USE            = 202; // The accessed object is already in use (eg locked) by another task.
  ERROR_OBJECT_EXISTS            = 203; // You tried to overwrite an object.
  ERROR_DIR_NOT_FOUND            = 204; // The given directory or the path of a given object does not exist.
  ERROR_OBJECT_NOT_FOUND         = 205; // The given object does not exist.
// Miscellaneous errors.
  ERROR_BAD_STREAM_NAME          = 206;
  ERROR_OBJECT_TOO_LARGE         = 207; { The given object is too large for the operation to be made. Object is
                                          this context are for example components of path-names.}
  ERROR_ACTION_NOT_KNOWN         = 209; { This is usually used to indicate that a filesystem does not support a
                                          certain action, but may generally also be used by functions.}
  ERROR_INVALID_COMPONENT_NAME   = 210; // A path component was invalid (eg there were multiple colons in a path name
  ERROR_INVALID_LOCK             = 211;
  ERROR_OBJECT_WRONG_TYPE        = 212; { You tried to perform an action on an object, which this kind of object
                                          does not support (eg makedir on a file).}
  ERROR_DISK_NOT_VALIDATED       = 213; // Writing failed, because the volume is not validated.
  ERROR_DISK_WRITE_PROTECTED     = 214; // Writing failed, because the volume is write-protected.
  ERROR_RENAME_ACROSS_DEVICES    = 215; { You tried to move/rename a file across different devices. Rename does only
                                          work on the same device, as only the inode-data has to be changed to
                                          perform that action.}
  ERROR_DIRECTORY_NOT_EMPTY      = 216; // You tried to delete a directory that still contains some files. Delete these files first.
  ERROR_TOO_MANY_LEVELS          = 217; // A recursive directory search could not be performed, because the stack was too small.
  ERROR_DEVICE_NOT_MOUNTED       = 218; // You tried to access a device that is currently not mounted.
  ERROR_SEEK_ERROR               = 219; // An error occurred, while executing DosSeek().
  ERROR_COMMENT_TOO_BIG          = 220; // The supplied file comment was longer than the hardcoded length limit for file comments.
  ERROR_DISK_FULL                = 221; // A write-operation could not be performed, because the volume has no space left.
  ERROR_DELETE_PROTECTED         = 222; // You tried to delete a delete-protected object.
  ERROR_WRITE_PROTECTED          = 223; { You tried to write to a write-protected object. This does not mean that
                                          the volume, you wanted to write to, is write-protected!}
  ERROR_READ_PROTECTED           = 224; // You tried to read a read-protected object.
  ERROR_NOT_A_DOS_DISK           = 225; // Accessed disk is unreadable.
  ERROR_NO_DISK                  = 226; // You tried to perform an action on a device that has no volume mounted (eg. an empty disk drive).
  ERROR_NO_MORE_ENTRIES          = 232; { This does not indicate an error, but is returned by several functions to
                                          indicate that the last entry of a list was reached.}
  ERROR_IS_SOFT_LINK              = 233; { Given action can not be performed on a given object, because it is a
                                           soft-link. This is usually only used by filesystem handlers and is catched
                                           by dos. Applications should not see this.}
  ERROR_OBJECT_LINKED             = 234; // Given action can not be performed on a given object, because it is a link.
  ERROR_BAD_HUNK                  = 235; // There was a bad hunk in a file that was to load.
  ERROR_NOT_IMPLEMENTED           = 236; { Indicates that a function does not implement a certain functionality.
                                           There are more special error conditions (ERROR_BAD_NUMBER and
                                           ERROR_ACTION_NOT_KNOWN), which should be preferred, if applicable.}
  ERROR_RECORD_NOT_LOCKED         = 240; // You tried to access a record that was not locked.
  ERROR_LOCK_COLLISION            = 241; // Somebody already locked a part of the record, you wanted to lock.
  ERROR_LOCK_TIMEOUT              = 242; // LockRecord() timed out.
  ERROR_UNLOCK_ERROR              = 243; // An error occurred, while unlocking a record.

{ more error codes are defined in dosasl.h and filesystem.h }

{ Maximum length of strings got from Fault(). Note that they should be
   shorter than 60 characters. }
  FAULT_MAX  = 82;

{ Signals that are set, if the user presses the corresponding keys on
   the controlling terminal. They may also be sent by using Signal().
   For more information see <exec/tasks.h>. }
  SIGBREAKB_CTRL_C   = 12; // CTRL-c, usually meaning program abortion.
  SIGBREAKB_CTRL_D   = 13; // CTRL-d
  SIGBREAKB_CTRL_E   = 14; // CTRL-e, usually meaning that the application should iconify itself.
  SIGBREAKB_CTRL_F   = 15; // CTRL-f, usually meaning that the application should uniconify itself.
{ Bit fields that signal you that a user has issued a break
  for example:  if (SetSignal(0,0) and SIGBREAKF_CTRL_C) then cleanup_and_exit();}
  SIGBREAKF_CTRL_C   = 1 shl SIGBREAKB_CTRL_C;
  SIGBREAKF_CTRL_D   = 1 shl SIGBREAKB_CTRL_D;
  SIGBREAKF_CTRL_E   = 1 shl SIGBREAKB_CTRL_E;
  SIGBREAKF_CTRL_F   = 1 shl SIGBREAKB_CTRL_F;

{ Mode parameter to Open() }
  MODE_OLDFILE   = 1005; // Open existing file read/write positioned at beginning of file.
  MODE_NEWFILE   = 1006; // Open freshly created file (delete old file) read/write
  MODE_READWRITE = 1004; // An old file is opened. If it does not exist, a new one is created.

{ Passed as type to Lock() }
  SHARED_LOCK    = -2;             // Non-exclusive lock, other tasks may lock this file as well.
  ACCESS_READ    = SHARED_LOCK;    //   This is used for read-only operations.
  EXCLUSIVE_LOCK = -1;             // Exclusive lock, other tasks may not lock this file.
  ACCESS_WRITE   = EXCLUSIVE_LOCK; // This is used for write operations.

{ Values returned by SameLock() }
  LOCK_SAME         =  0;
  LOCK_SAME_HANDLER =  1; // actually same volume
  LOCK_DIFFERENT    = -1;

{ Values for MakeLink() }
  LINK_HARD =   0;
  LINK_SOFT =   1;

{ Relative position to Seek() }
  OFFSET_BEGINNING = -1; // relative to Begining Of File
  OFFSET_CURRENT   =  0; // relative to Current file position
  OFFSET_END       =  1; // relative to End Of File

{ Limits of the "Integer" type already defined in FPC... but ok}
  MAXINT = $7FFFFFFF;
  MININT = $80000000;

{ types for ChangeMode() }
  CHANGE_LOCK = 0;
  CHANGE_FH = 1;

{ values returned by  ReadItem() }
  ITEM_EQUAL    = -2; // '=' Symbol
  ITEM_ERROR    = -1; // error
  ITEM_NOTHING  =  0; // *N, ;, endstreamch
  ITEM_UNQUOTED =  1; // unquoted item
  ITEM_QUOTED   =  2; // quoted item

{ types for AllocDosObject/FreeDosObject }
  DOS_FILEHANDLE   =  0; // PFileHandle
  DOS_EXALLCONTROL =  1; // PExAllControl
  DOS_FIB          =  2; // PFileInfoBlock
  DOS_STDPKT       =  3; // PDosPacket
  DOS_CLI          =  4; // PCommandLineInterface
  DOS_RDARGS       =  5; // PRDArgs


{ Data structures and equates used by the V1.4 DOS functions StrtoDate() and DatetoStr() }
{--------- String/Date structures etc }
type
  _PDateTime = ^_TDateTime;
  _TDateTime = record
    dat_Stamp: TDateStamp; // DOS DateStamp
    dat_Format,            // controls appearance of dat_StrDate
    dat_Flags: Byte;       // see BITDEF's below
    dat_StrDay,            // day of the week string
    dat_StrDate,           // date string
    dat_StrTime: STRPTR;   // time string
  end;

{ You need this much room for each of the DateTime strings: }
const
  LEN_DATSTRING =  16;

{ date format values }
 FORMAT_DOS     = 0;          // DOS internal format, dd-mmm-yy
 FORMAT_INT     = 1;          // International format, yy-mm-dd
 FORMAT_USA     = 2;          // US-American format, mm-dd-yy  }
 FORMAT_CDN     = 3;          // Canadian format, dd-mm-yy  }
 FORMAT_MAX     = FORMAT_CDN;
 FORMAT_DEF     = 4;          { use default format, as defined by locale; if locale not
                                available, use FORMAT_DOS instead }

{ flags for dat_Flags }

 DTB_SUBST      = 0; // Substitute Today, Tomorrow, etc. if possible.
 DTF_SUBST      = 1 shl DTB_SUBST;
 DTB_FUTURE     = 1; // Day of the week is in future.
 DTF_FUTURE     = 1 shl DTB_FUTURE;

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

type
  // PRIVATE structure, which describes an anchor for matching functions.
  PAChain = ^TAChain;
  TAChain = record
    an_Child,           // The next anchor
    an_Parent: PAChain; // The last anchor
    an_Lock: BPTR;      // Lock of this anchor
    an_Info: TFileInfoBlock; // fib Discribing this anchor
    an_Flags: ShortInt;      // se below
    an_String: array[0..0] of Char;
  end;
const
// an_Flags
  DDB_PatternBit  = 0;
  DDB_ExaminedBit = 1;
  DDB_Completed   = 2;
  DDB_AllBit      = 3;
  DDB_Single      = 4;
  DDF_PatternBit  = 1 shl DDB_PatternBit;
  DDF_ExaminedBit = 1 shl DDB_ExaminedBit;
  DDF_Completed   = 1 shl DDB_Completed;
  DDF_AllBit      = 1 shl DDB_AllBit;
  DDF_Single      = 1 shl DDB_Single;

type
  PAnchorPath = ^TAnchorPath;
  TAnchorPath = record
  case SmallInt of
    0 :(
      ap_First      : PAChain;
      ap_Last       : PAChain;
    );
    1 :(
      ap_Base,                             // pointer to first anchor
      ap_Current    : PAChain;             // pointer to last anchor
      ap_BreakBits,                        // Signal bits that caused the function to break.
      ap_FoundBreak : LongInt;             // Bits we broke on. Also returns ERROR_BREAK
      ap_Flags      : Shortint;            // see below
      ap_Reserved   : Shortint;            // Private
      ap_Strlen     : SmallInt;            // Size of ap_Buf (see below). This may be zero.
      ap_Info       : TFileInfoBlock;      // describes any files found by matching-functions.
      ap_Buf        : Array[0..0] of Char; // Buffer for path name, allocated by user!!
    );
  end;

const
  APB_DOWILD       = 0; // Please check for wildcards in supplied string.
  APB_ITSWILD      = 1; // There is actually a wildcard in the supplied string. READ-ONLY
  APB_DODIR        = 2; { Set, if a directory is to be entered.
                          Applications may clear this bit to prohibit the
                          matching-functions from entering a directory. }
  APB_DIDDIR       = 3; // Set, if directory was already searched. READ-ONLY
  APB_NOMEMERR     = 4; // Set, if function was out of memory. READ-ONLY
  APB_DODOT        = 5; // '.' may refer to the current directory (unix-style).
  APB_DirChanged   = 6; // Directory changed since last call.
  APB_FollowHLinks = 7; // Follow hardlinks, too.

  APF_DOWILD       = 1 shl APB_DOWILD;
  APF_ITSWILD      = 1 shl APB_ITSWILD;
  APF_DODIR        = 1 shl APB_DODIR;
  APF_DIDDIR       = 1 shl APB_DIDDIR;
  APF_NOMEMERR     = 1 shl APB_NOMEMERR;
  APF_DODOT        = 1 shl APB_DODOT;
  APF_DirChanged   = 1 shl APB_DirChanged;
  APF_FollowHLinks = 1 shl APB_FollowHLinks;

{ Predefined tokens for wildcards. The characters are replaced by these
  tokens in the tokenized string returned by the ParsePattern() function
  family.}
  P_ANY         =  $80; // Matches everything '#?' and '*'
  P_SINGLE      =  $81; // Any character '?'
  P_ORSTART     =  $82; // Opening parenthesis for OR'ing '('
  P_ORNEXT      =  $83; // Field delimiter for OR'ing '|'
  P_OREND       =  $84; // Closing parenthesis for OR'ing ')'
  P_NOT         =  $85; // Inversion '~'
  P_NOTEND      =  $86; // Inversion end
  P_NOTCLASS    =  $87; // Inversion class '^'
  P_CLASS       =  $88; // Class '[]'
  P_REPBEG      =  $89; // Beginning of repetition '['
  P_REPEND      =  $8A; // End of repetition ']'
  P_STOP        =  $8B; // token to force end of evaluation

{ Values for an_Status, NOTE: These are the actual bit numbers }
  COMPLEX_BIT   =  1; // Parsing complex pattern
  EXAMINE_BIT   =  2; // Searching directory

{ Returns from MatchFirst(), MatchNext() more see before}
  ERROR_BUFFER_OVERFLOW  = 303; // User OR internal buffer overflow
  ERROR_BREAK            = 304; // A break character was received
  ERROR_NOT_EXECUTABLE   = 305; // A file has E bit cleared

{ hunk types }
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
  HUNK_RELOC32SHORT = 1020;
  HUNK_RELRELOC32 = 1021;
  HUNK_ABSRELOC16 = 1022;

{ hunk_ext sub-types }
  EXT_SYMB       = 0;   // symbol table
  EXT_DEF        = 1;   // relocatable definition
  EXT_ABS        = 2;   // Absolute hunks
  EXT_REF32      = 129; // 32bit absolute reference to symbol
  EXT_ABSREF32   = 129;
  EXT_COMMON     = 130; // 32bit absolute reference to common block
  EXT_ABSCOMMON  = 130;
  EXT_REF16      = 131; // 16bit relative reference to symbol
  EXT_RELREF16   = 131;
  EXT_REF8       = 132; // 8bit relative reference to symbol
  EXT_RELREF8    = 132;
  EXT_DEXT32     = 133; // 32 bit data releative reference
  EXT_DEXT16     = 134; // 16 bit data releative reference
  EXT_DEXT8      = 135; // 8 bit data releative reference
  EXT_RELREF32   = 136; // 32bit relative reference to symbol
  EXT_RELCOMMON  = 137; // 32bit relative reference to common block
  EXT_ABSREF16   = 138;
  EXT_ABSREF8    = 139;

{ Hunk flags }
  HUNKB_ADVISORY = 29; // Hunk is ignored, if unknown to loader.
  HUNKB_CHIP     = 30;
  HUNKB_FAST     = 31;
  HUNKF_ADVISORY = 1 shl HUNKB_ADVISORY;
  HUNKF_CHIP     = 1 shl HUNKB_CHIP;
  HUNKF_FAST     = 1 shl HUNKB_FAST;

type
  PDosInfo = ^TDosInfo;
  TDosInfo = record
    di_McName: BPTR;   // Network name of this machine; currently nil
    di_DevInfo: BPTR;  // Device List
    di_Devices: BPTR;  // Reserved
    di_Handlers: BPTR; // Reserved
    di_NetHand: BPTR;  // Reserved (actually resident segment list)
    di_DevLock,        // do NOT access directly!
    di_EntryLock,      // do NOT access directly!
    di_DeleteLock: TSignalSemaphore; // do NOT access directly!
  end;

{ All DOS processes have this structure }
{ Create and Device Proc returns pointer to the MsgPort in this structure }
{ dev_proc = Address(SmallInt(DeviceProc()) - SizeOf(Task)) }

  TExitProcedure = procedure(Arg: IPTR); cdecl;

  PProcess = ^TProcess;
  TProcess = record
    pr_Task: TTask;
    pr_MsgPort: TMsgPort;        // This is BPTR address from DOS functions
    pr_Pad: SmallInt;            // Remaining variables on 4 byte boundaries
    pr_SegList: BPTR;            // Array of seg lists used by this process
    pr_StackSize: LongInt;       // Size of process stack in bytes
    pr_GlobVec: APTR;            // Global vector for this process (BCPL)
    pr_TaskNum: LongInt;         // CLI task number of zero if not a CLI
    pr_StackBase: BPTR;          // Ptr to high memory end of process stack
    pr_Result2: STRPTR;          // Value of secondary result from last call
    pr_CurrentDir: BPTR;         // Lock associated with current directory
    pr_CIS: BPTR;                // Current CLI Input Stream
    pr_COS: BPTR;                // Current CLI Output Stream
    pr_ConsoleTask: APTR;        // Console handler process for current window
    pr_FileSystemTask: APTR;     // File handler process for current drive
    pr_CLI: BPTR;                // pointer to ConsoleLineInterpreter
    pr_ReturnAddr: APTR;         // pointer to previous stack frame
    pr_PktWait: APTR;            // Function to be called when awaiting msg
    pr_WindowPtr: APTR;          // Window for error printing
    pr_HomeDir: BPTR;            // Home directory of executing program
    pr_Flags: LongInt;           // flags telling dos about process
    pr_ExitCode: TExitProcedure; // code to call on exit of program OR nil
    pr_ExitData: IPTR;           // Passed as an argument to pr_ExitCode.
    pr_Arguments: STRPTR;        // Arguments passed to the process at start
    pr_LocalVars: TMinList;      // Local environment variables
    pr_ShellPrivate: ULONG;      // for the use of the current shell
    pr_CES: BPTR;                // Error stream - IF NULL, use pr_COS
  end;

{ Flags for pr_Flags. (all PRIVATE) They mainly descibe what happens if the process
  exits, i.e. which resources the process should clean itself. The flags
  are self-explaining.}
const
  PRB_FREESEGLIST     = 0;
  PRB_FREECURRDIR     = 1;
  PRB_FREECLI         = 2;
  PRB_CLOSEINPUT      = 3;
  PRB_CLOSEOUTPUT     = 4;
  PRB_FREEARGS        = 5;
  PRB_CLOSEERROR      = 6;
  PRB_SYNCHRONOUS     = 7;
  PRB_WAITINGFORCHILD = 8; // This one is subject to change!
  PRB_NOTIFYONDEATH   = 9;

  PRF_FREESEGLIST     = 1 shl PRB_FREESEGLIST;
  PRF_FREECURRDIR     = 1 shl PRB_FREECURRDIR;
  PRF_FREECLI         = 1 shl PRB_FREECLI;
  PRF_CLOSEINPUT      = 1 shl PRB_CLOSEINPUT;
  PRF_CLOSEOUTPUT     = 1 shl PRB_CLOSEOUTPUT;
  PRF_FREEARGS        = 1 shl PRB_FREEARGS;
  PRF_CLOSEERROR      = 1 shl PRB_CLOSEERROR;
  PRF_SYNCHRONOUS     = 1 shl PRB_SYNCHRONOUS;
  PRF_WAITINGFORCHILD = 1 shl PRB_WAITINGFORCHILD; // This one is subject to change!
  PRF_NOTIFYONDEATH   = 1 shl PRB_NOTIFYONDEATH;

{ The long SmallInt address (BPTR) of this structure is returned by
  Open() and other routines that return a file.  You need only worry
  about this struct to do async io's via PutMsg() instead of
  standard file system calls }
type
  PFileHandle = ^TFileHandle;
  TFileHandle = record
    fh_Flags: ULONG;   { EXEC message        }
    fh_Port: PMsgPort;   { Reply port for the packet }
    fh_Type: PMsgPort;   { Port to do PutMsg() to Address is negative if a plain file }

    fh_Buf: BPTR;
    fh_Pos: LongInt;
    fh_End: LongInt;

    fh_Func1: LongInt;
    fh_Func2: LongInt;
    fh_Func3: LongInt;
    fh_Arg1:  LongInt;
    fh_Arg2: APTR;
    fh_Size: ULONG; // Size of buffered io buffer
    fh_Buf2: BPTR;  // Always the same as fh_Buf
  end;

{ This is the extension to EXEC Messages used by DOS }

  PDosPacket = ^TDosPacket;
  TDosPacket = record
    dp_Link : PMessage;     // Pointer to a standard exec message.
    dp_Port : PMsgPort;     // Reply-Port of that packet. Must be filled in each send.
    case SmallInt of
    0 : (
      dp_Action : LongInt;
      dp_Status : LongInt;
      dp_Status2 : LongInt;
      dp_BufAddr : LongInt;
    );
    1 : (
      dp_Type : LongInt;      { See ACTION_... below and
                              * 'R' means Read, 'W' means Write to the
                              * file system }
      dp_Res1 : LongInt;      { For file system calls this is the result
                              * that would have been returned by the
                              * function, e.g. Write ('W') returns actual
                              * length written }
      dp_Res2 : LongInt;      { For file system calls this is what would
                              * have been returned by IoErr() }
      dp_Arg1 : LongInt;
      dp_Arg2 : LongInt;
      dp_Arg3 : LongInt;
      dp_Arg4 : LongInt;
      dp_Arg5 : LongInt;
      dp_Arg6 : LongInt;
      dp_Arg7 : LongInt;
    );
  end;

const
{ Packet types dp_Type }
  ACTION_NIL                  = 0;
  ACTION_STARTUP              = 0;
  ACTION_GET_BLOCK            = 2;    // OBSOLETE
  ACTION_SET_MAP              = 4;
  ACTION_DIE                  = 5;
  ACTION_EVENT                = 6;
  ACTION_CURRENT_VOLUME       = 7;
  ACTION_LOCATE_OBJECT        = 8;
  ACTION_RENAME_DISK          = 9;
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
  ACTION_SAME_LOCK            = 40;

  ACTION_WRITE                = $57;  // 'W'
  ACTION_READ                 = $52;  // 'R'

  ACTION_SCREEN_MODE          = 994;
  ACTION_CHANGE_SIGNAL        = 995;
  ACTION_READ_RETURN          = 1001;
  ACTION_WRITE_RETURN         = 1002;
  ACTION_FINDUPDATE           = 1004;
  ACTION_FINDINPUT            = 1005;
  ACTION_FINDOUTPUT           = 1006;
  ACTION_END                  = 1007;
  ACTION_SEEK                 = 1008;
  ACTION_FORMAT               = 1020;
  ACTION_MAKE_LINK            = 1021;
  ACTION_SET_FILE_SIZE        = 1022;
  ACTION_WRITE_PROTECT        = 1023;
  ACTION_READ_LINK            = 1024;
  ACTION_FH_FROM_LOCK         = 1026;
  ACTION_IS_FILESYSTEM        = 1027;
  ACTION_CHANGE_MODE          = 1028;
  {}
  ACTION_COPY_DIR_FH          = 1030;
  ACTION_PARENT_FH            = 1031;
  ACTION_EXAMINE_ALL          = 1033;
  ACTION_EXAMINE_FH           = 1034;
  ACTION_EXAMINE_ALL_END      = 1035;
  ACTION_SET_OWNER            = 1036;

  ACTION_LOCK_RECORD          = 2008;
  ACTION_FREE_RECORD          = 2009;

  ACTION_ADD_NOTIFY           = 4097;
  ACTION_REMOVE_NOTIFY        = 4098;
{ Tell a file system to serialize the current volume. This is typically
  done by changing the creation date of the disk. This packet does not take
  any arguments.}
  ACTION_SERIALIZE_DISK       = 4200;

type
{ A Packet does not require the Message to be before it in memory, but
  for convenience it is useful to associate the two.
  Also see the function init_std_pkt for initializing this structure }
  PStandardPacket = ^TStandardPacket;
  TStandardPacket = record
    sp_Msg: TMessage;
    sp_Pkt: TDosPacket;
  end;

const
{ types for initial packets to shells from run/newcli/execute/system.
  NOTE: AROS doesn't use startup packets. This will ONLY make a difference
         for shell writers...}
  RUN_EXECUTE           =  -1;
  RUN_SYSTEM            =  -2;
  RUN_SYSTEM_ASYNCH     =  -3;

type
{ ONLY to be allocated by DOS! }
  PCliProcList = ^TCliProcList;
  TCliProcList = record
    cpl_Node: TMinNode;
    cpl_First: LongInt;      { number of first entry in array }
    cpl_Array: array[0..0] of PMsgPort;
                         { [0] is max number of CLI's in this entry (n)
                           [1] is CPTR to process id of CLI cpl_First
                           [n] is CPTR to process id of CLI cpl_First+n-1}
  end;

{ structure for the Dos resident list.  Do NOT allocate these, use       }
{ AddSegment(), and heed the warnings in the autodocs!                   }
type
  PSegment = ^TSegment;
  TSegment = record
    seg_Next: BPTR;    // Pointer to next segment.
    seg_UC: LongInt;   // Usage count/type
    seg_Seg: BPTR;     // Actual Segment
    seg_Name: array[0..3] of Char;  // actually the first 4 chars of BSTR name }
  end;

const
  CMD_SYSTEM    =  -1;
  CMD_INTERNAL  =  -2;
  CMD_DISABLED  =  -999;

{ DOS Processes started from the CLI via RUN or NEWCLI have this additional
 * set to data associated with them }
type
  PCommandLineInterface = ^TCommandLineInterface;
  TCommandLineInterface = record
    cli_Result2: LongInt;      // Value of IoErr from last command
    cli_SetName: BSTR;         // Name of current directory
    cli_CommandDir: BPTR;      // Lock associated with command directory
    cli_ReturnCode: LongInt;   // Return code from last command
    cli_CommandName: BSTR;     // Name of current command
    cli_FailLevel: LongInt;    // Fail level (set by FAILAT)
    cli_Prompt: BSTR;          // Current prompt (set by PROMPT)
    cli_StandardInput: BPTR;   // Default (terminal) CLI input
    cli_CurrentInput: BPTR;    // Current CLI input
    cli_CommandFile: BSTR;     // Name of EXECUTE command file
    cli_Interactive: LongInt;  // Boolean; True if prompts required
    cli_Background: LongInt;   // Boolean; True if CLI created by RUN
    cli_CurrentOutput: BPTR;   // Current CLI output
    cli_DefaultStack: LongInt; // Stack size to be obtained in long words
    cli_StandardOutput: BPTR;  // Default (terminal) CLI output
    cli_Module: BPTR;          // SegList of currently loaded command
{$ifdef aros}
    cli_StandardError: BPTR;   // Standard/Default Error file. PFileLock
{$endif}
  end;

const
// CLI_DEFAULTSTACK_UNIT * cli_DefaultStack = stack in bytes
  CLI_DEFAULTSTACK_UNIT = SizeOf(IPTR);

type
{$ifdef aros}
  PDosListAROSExt = ^TDosListAROSExt;
  TDosListAROSExt = record
    dol_DevName: STRPTR;
    dol_Device: PDevice;
    dol_Unit: PUnit;
  end;
{$endif}
{ This structure can take on different values depending on whether it is
 * a device, an assigned directory, or a volume.  Below is the structure
 * reflecting volumes only.  Following that is the structure representing
 * only devices.
 }

{ structure representing a volume }

  PDeviceList = ^TDeviceList;
  TDeviceList = record
    dl_Next: BPTR;       // bptr to next device list
    dl_Type: LongInt;    // see DLT below
    dl_Task: PMsgPort;   // ptr to handler task
    dl_Lock: BPTR;       // not for volumes
    dl_VolumeDate: TDateStamp; // creation date
    dl_LockList: BPTR;    // outstanding locks
    dl_DiskType: LongInt; // 'DOS', etc
    dl_unused: BPTR;
    dl_Name: BSTR;        // bptr to bcpl name
  end;

{ device structure (same as the DeviceNode structure in filehandler.h) }
  PDevInfo = ^TDevInfo;
  TDevInfo = record
    dvi_Next: BPTR;
    dvi_Type: LongInt;
    dvi_Task: PMsgPort;
    dvi_Lock: BPTR;
    dvi_Handler: BSTR;
    dvi_StackSize: LongInt;
    dvi_Priority: LongInt;
    dvi_Startup: BPTR;
{$ifdef aros}
    dvi_NoAROS4: array[0..1] of BPTR;
{$else}
    dvi_SegList: BPTR;
    dvi_GlobVec: BSTR;
{$endif}
    dvi_Name: BSTR;
  end;

const
{ Dos list scanning and locking modes as used in LockDosList()
  Specify either LDF_READ, if you want a non-exclusive lock, or LDF_WRITE,
  if you want an exclusive lock (i.e. if you want to modify the list).}
  LDB_READ      =  0;
  LDB_WRITE     =  1;
// Specify which list(s) to lock.
  LDB_DEVICES   =  2;
  LDB_VOLUMES   =  3;
  LDB_ASSIGNS   =  4;
  LDB_ENTRY     =  5;
  LDB_DELETE    =  6;

  LDF_READ      =  1 shl LDB_READ;
  LDF_WRITE     =  1 shl LDB_WRITE;
  LDF_DEVICES   =  1 shl LDB_DEVICES;
  LDF_VOLUMES   =  1 shl LDB_VOLUMES;
  LDF_ASSIGNS   =  1 shl LDB_ASSIGNS;
  LDF_ENTRY     =  1 shl LDB_ENTRY;
  LDF_DELETE    =  1 shl LDB_DELETE;
{ actually all but LDF_ENTRY (which is used for internal locking) }
  LDF_ALL       =  (LDF_DEVICES or LDF_VOLUMES or LDF_ASSIGNS);

type
{ Used for assigns that point to multiple directories. }

  PAssignList = ^TAssignList;
  TAssignList = record
    al_Next: PAssignList; // Pointer to next assign node.
    al_Lock: BPTR;        // (struct FileLock *) Lock of on of the directories.
  end;

{ combined structure for devices, assigned directories, volumes }
  PDosList = ^TDosList;
  TDosList = record
    dol_Next: BPTR;           {    bptr to next device on list }
    dol_Type: LongInt;        {    see DLT below }
    dol_Task: PMsgPort;       {    ptr to handler task }
    dol_Lock: BPTR;
    case SmallInt of
      0 :(
        dol_Handler : record
          dol_Handler: BSTR;      {    file name to load IF seglist is null }
          dol_StackSize,              {    stacksize to use when starting process }
          dol_Priority: LongInt;               {    task priority when starting process }
          dol_Startup: BPTR;   {    startup msg: FileSysStartupMsg for disks }
          dol_SegList,                {    already loaded code for new task }
          dol_GlobVec: BPTR;      {    BCPL global vector to use when starting }
        end;
      );
      1 :(
        dol_Volume: record
          dol_VolumeDate: TDateStamp; {    creation date }
          dol_LockList: BPTR;       {    outstanding locks }
          dol_DiskType: LongInt;    {    'DOS', etc }
          dol_Unused: BPTR
        end;
      );
      2 :(
        dol_assign:  record
          dol_AssignName: STRPTR;        {    name for non-OR-late-binding assign }
          dol_List: PAssignList;   {    for multi-directory assigns (regular) }
        end;
      );
      3 :(
        {$ifdef CPU64}
        dol_Misc: array[0..39] of Byte;
        {$else}
        dol_Misc: array[0..23] of Byte;
        {$endif}
        dol_Name: BSTR;           {    bptr to bcpl name }
      );
    end;


const

{ definitions for dl_Type }
  DLT_DEVICE          = 0;
  DLT_DIRECTORY       = 1;
  DLT_VOLUME          = 2;
  DLT_LATE            = 3;  // late-binding assign
  DLT_NONBINDING      = 4;  // non-binding assign
  DLT_PRIVATE         = -1; // for internal use only

{ structure return by GetDeviceProc() }
type
  PDevProc = ^TDevProc;
  TDevProc = record
    dvp_Port: PMsgPort;
    dvp_Lock: BPTR;         // PFileLock
    dvp_Flags: LongInt;     // see below (DVPF_*)
    dvp_DevNode: PDosList;  // Private
  end;

const
{ definitions for dvp_Flags }
  DVPB_UNLOCK   =  0;
  DVPB_ASSIGN   =  1;
  DVPF_UNLOCK   =  1 shl DVPB_UNLOCK;
  DVPF_ASSIGN   =  1 shl DVPB_ASSIGN;

{ Types for fib_DirEntryType.  NOTE that both USERDIR and ROOT are
  directories, and that directory/file checks should use <0 and >=0.
  This is not necessarily exhaustive!  Some handlers may use other
  values as needed, though <0 and >=0 should remain as supported as
  possible.}
  ST_ROOT       =  1 ; // Root directory of filesystem
  ST_USERDIR    =  2 ; // Normal directory
  ST_SOFTLINK   =  3 ; // Soft link (may be a file or directory)
  ST_LINKDIR    =  4 ; // Hard link to a directory
  ST_FILE       =  -3; // Plain file
  ST_LINKFILE   =  -4; // Hard link to a file
  ST_PIPEFILE   =  -5; // File is a pipe

type
{ a lock structure, as returned by Lock() or DupLock() }
  PFileLock = ^TFileLock;
  TFileLock = record
    fl_Link: BPTR;      // bcpl pointer to next lock
    fl_Key: IPTR;       // disk block number
    fl_Access: LongInt; // exclusive or shared
    fl_Task: PMsgPort;  // handler task's port
    fl_Volume: BPTR;    // bptr to a DeviceList
  end;


{ Structure (as used in ExAll()), containing information about a file. This
  structure is only as long as it need to be. If is for example ED_SIZE was
  specified, when calling ExAll(), this structure only consists of the fields
  ed_Name through ed_Size. Therefore you can use the ED_ definitions below
  as longword offsets into this structure.}
type
  PExAllData = ^TExAllData;
  TExAllData = record
    ed_Next: PExAllData;
    ed_Name: PChar;        // Name of the file
    ed_Type: LongInt;      // Type of File
    ed_Size,               // Size of File
    ed_Prot,               // Protection Bits
{ The following three fields are de facto an embedded datestamp
  structure (see <dos/dos.h>), which describes the last modification date.}
    ed_Days,
    ed_Mins,
    ed_Ticks    : ULONG;

    ed_Comment: PChar;     // The file comment
    ed_OwnerUID,           // The owner ID
    ed_OwnerGID : Word;    // the group-owner ID
  end;

{ Type argument for ExAll(). Each number includes the information of all
  lower numbers, too. If you specify for example ED_SIZE, you will get
  information about name, type and the size of a file. Note that all
  filehandlers must handle all types up to ED_OWNER. If they do not support
  a type, they must return ERROR_WRONG_NUMBER. Currently
  that means, if a value higher than ED_OWNER is specified, filehandlers
  must fail with this error.}
const
  ED_NAME        = 1; // Filename.
  ED_TYPE        = 2; // Type of file.
  ED_SIZE        = 3; // Size of file.
  ED_PROTECTION  = 4; // Protection bits.
  ED_DATE        = 5; // Last modification date.
  ED_COMMENT     = 6; // Addtional file comment.
  ED_OWNER       = 7; // Owner information.

{ Structure as used for controlling ExAll(). Allocate this structure by using
  AllocDosObject(DOS_EXALLCONTROL,...) only. All fields must be initialized
  to 0, before using this structure. (AllocDosObject() does that for you.)
  After calling ExAll() the first time, this structure is READ-ONLY. }
type
  PExAllControl = ^TExAllControl;
  TExAllControl = record
    eac_Entries: ULONG;     // number of entries returned in buffer
    eac_LastKey: IPTR;      // Don't touch inbetween linked ExAll calls!
    eac_MatchString: PChar; // wildcard string for pattern match OR nil
    eac_MatchFunc: PHook;   // optional private wildcard FUNCTION
  end;

{ The disk "environment" is a longword array that describes the
 * disk geometry.  It is variable sized, with the length at the beginning.
 * Here are the constants for a standard geometry.}
type
  PDosEnvec = ^TDosEnvec;
  TDosEnvec = record
    de_TableSize: IPTR;      // Size of this structure. Must be at least 11 (DE_NUMBUFFERS).
    de_SizeBlock: IPTR;      // Size in longwords of a block on the disk.
    de_SecOrg: IPTR;         // Unused. Must be 0 for now.
    de_Surfaces: IPTR;       // Number of heads/surfaces in drive.
    de_SectorPerBlock: IPTR; // Unused. Must be 1 for now.
    de_BlocksPerTrack: IPTR; // blocks per track. drive specific
    de_Reserved: IPTR;       // DOS reserved blocks at start of partition.
    de_PreAlloc: IPTR;       // DOS reserved blocks at end of partition
    de_Interleave: IPTR;     // usually 0
    de_LowCyl: IPTR;         // starting cylinder. typically 0
    de_HighCyl: IPTR;        // max cylinder. drive specific
    de_NumBuffers: IPTR;     // Initial # DOS of buffers.
    de_BufMemType: IPTR;     // type of mem to allocate for buffers
    de_MaxTransfer: IPTR;    // Max number of bytes to transfer at a time
    de_Mask: IPTR;           // Address Mask to block out certain memory
    de_BootPri: LongInt;     // Boot priority for autoboot
    de_DosType: IPTR;        // ASCII (HEX) string showing filesystem type
    de_Baud: IPTR;           // Baud rate for serial handler
    de_Control: IPTR;        // Control SmallInt for handler/filesystem
    de_BootBlocks: IPTR;     // Number of blocks containing boot code
  end;

const
{ The following constants are longword offsets, which point into a filehandler
  structure (like the one above). For more information about the meaning
  of these constants see the structure above. }
  DE_TABLESIZE        = 0;    // standard value is 11 }
  DE_SIZEBLOCK        = 1;    // in longwords: standard value is 128 }
  DE_SECORG           = 2;    // not used; must be 0 }
  DE_NUMHEADS         = 3;    // # of heads (surfaces). drive specific }
  DE_SECSPERBLK       = 4;    // not used; must be 1 }
  DE_BLKSPERTRACK     = 5;    // blocks per track. drive specific }
  DE_RESERVEDBLKS     = 6;    // unavailable blocks at start.   usually 2 }
  DE_PREFAC           = 7;    // not used; must be 0 }
  DE_INTERLEAVE       = 8;    // usually 0 }
  DE_LOWCYL           = 9;    // starting cylinder. typically 0 }
  DE_UPPERCYL         = 10;   // max cylinder.  drive specific }
  DE_NUMBUFFERS       = 11;   // starting # of buffers.  typically 5 }
  DE_MEMBUFTYPE       = 12;   // type of mem to allocate for buffers. }
  DE_BUFMEMTYPE       = 12;   // same as above, 1 is public, 3 is chip, 5 is fast }
  DE_MAXTRANSFER      = 13;   // Max number bytes to transfer at a time }
  DE_MASK             = 14;   // Address Mask to block out certain memory }
  DE_BOOTPRI          = 15;   // Boot priority for autoboot }
  DE_DOSTYPE          = 16;   // ASCII (HEX) string showing filesystem type;
  DE_BAUD             = 17;   // Baud rate for serial handler }
  DE_CONTROL          = 18;   // Control SmallInt for handler/filesystem }
  DE_BOOTBLOCKS       = 19;   // Number of blocks containing boot code }


{ This is the message that is passed to a file handler during startup
  in the DeviceNode->dn_Startup field. It is not used in AROS DOS handlers
  as they are now Device based, and the information is passed in during
  OpenDevice(), however this needs to be stored for late opening
  handlers.}
type
  PFileSysStartupMsg = ^TFileSysStartupMsg;
  TFileSysStartupMsg = record
    fssm_Unit: IPTR;      // exec unit number for this device
    fssm_Device: BSTR;    // null terminated bstring to the device name
    fssm_Environ: BPTR;   // ptr to environment table (see above)
    fssm_Flags: ULONG;    // flags for OpenDevice()
  end;


{ This is an unwound version of the DosList structure.
  This is the version for a DOS "device" DLT_DEVICE.
  It is essentially the same structure as DevInfo.
  For AROS this is notably different, as filehandlers are no longer
  DOS tasks (ie Processes), some of the fields here have no purpose
  and are ignored. The only fields retained are the dn_Next, dn_Type,
  dn_Startup and dn_Handler fields.}
  PDeviceNode = ^TDeviceNode;
  TDeviceNode = record
    dn_Next: BPTR;         { singly linked list }
    dn_Type: ULONG;        { always 0 for dos "devices" }
    dn_Task: PMsgPort;     { standard dos "task" field.  If this is
                                     * null when the node is accesses, a task
                                     * will be started up }
    dn_Lock: BPTR;         { not used for devices -- leave null }
    dn_Handler: BSTR;         { filename to loadseg (if seglist is null) }
    dn_StackSize: ULONG;        { stacksize to use when starting task }
    dn_Priority: LongInt;      { task priority when starting task }
    dn_Startup: BPTR;         { startup msg: FileSysStartupMsg for disks }
    dn_SegList: BPTR;         { code to run to start new task (if necessary).
                                     * if null then dn_Handler will be loaded. }
    dn_GlobalVec: BPTR; { BCPL global vector to use when starting
                             * a task.  -1 means that dn_SegList is not
                             * for a bcpl program, so the dos won't
                             * try and construct one.  0 tell the
                             * dos that you obey BCPL linkage rules,
                             * and that it should construct a global
                             * vector for you.
                             }
    dn_Name: BSTR;         { the node name, e.g. '\3','D','F','3' }
  end;

type
{  General notification structure as passed to StartNotify() and EndNotify().
   After passing it to StartNotify() the first time, this structure becomes
   READ-ONLY! }

  PNotifyRequest = ^TNotifyRequest;
  TNotifyRequest = record
    nr_Name: STRPTR;        // Name of the watched file.
    nr_FullName: STRPTR;    // Fully qualified name of the watched file. This is READ-ONLY!
    nr_UserData: IPTR;      // Fill in with your own data.
    nr_Flags: LongWord;     // Flags: (NRB_*)
                                // The following case specified the way to notify the application, if
    nr_stuff: record            // the watched file changes. IF NRF_SEND_MESSAGE is set, nr_Msg is used,
    case SmallInt of            // when NRF_SEND_SIGNAL is set, nr_Signal is used.
       0: ( nr_Msg: record
            nr_Port: PMsgPort; // Port to send message to.
         end );
       1 : ( nr_Signal: record
            nr_Task: pTask;              // Task to notify.
            nr_SignalNum: Byte;          // Signal number to set.
            nr_pad: array[0..2] of Byte; // PRIVATE
         end );
    end;
    nr_Reserved: array[0..3] of LongWord; // PRIVATE! Set to 0 for now.
    nr_MsgCount: LongWord;                // Number of unreplied messages.
    nr_Handler: PMsgPort;                 // Filesystem task/device. Used by EndNotify()
    end;

   PNotifyMessage = ^TNotifyMessage;
   TNotifyMessage = record
    nm_ExecMessage: TMessage;
    nm_Class: LongWord;          // Class: NOTIFY_CLASS
    nm_Code: Word;               // Code: NOTIFY_CODE
    nm_NReq: PNotifyRequest;     // The notify structure that was passed to StartNotify(). Read-Only
    nm_DoNotTouch,               // like it says!  For use by handlers
    nm_DoNotTouch2 : LongWord;   // dito
   end;


const
// The two following flags specify by which means the watching task is to be notified.
  NRB_SEND_MESSAGE      =  0; // Send a message to the specified message port.
  NRB_SEND_SIGNAL       =  1; // Set a signal of the specified task.
  NRB_WAIT_REPLY        =  3; // Wait for a reply by the application before going on with watching?
  NRB_NOTIFY_INITIAL    =  4; // Notify if the file/directory exists when the notification request is posted

// Flag Values for TNotifyRequest.nr_Flags
  NRF_SEND_MESSAGE      =  1 shl NRB_SEND_MESSAGE;
  NRF_SEND_SIGNAL       =  1 shl NRB_SEND_SIGNAL;
  NRF_WAIT_REPLY        =  1 shl NRB_WAIT_REPLY;
  NRF_NOTIFY_INITIAL    =  1 shl NRB_NOTIFY_INITIAL;

// The following flags are for use by handlers only!
  NR_HANDLER_FLAGS      =  $ffff0000;
  NRB_MAGIC             =  31;
  NRF_MAGIC             =  1 shl NRB_MAGIC;

// nm_Class. Do not use, yet.
  NOTIFY_CLASS  =  $40000000;

//nm_Code. Do not use, yet.
   NOTIFY_CODE   =  $1234;

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

type
// This structure emulates an input stream by using a buffer.
  PCSource = ^TCSource;
  TCSource = record
    CS_Buffer: PChar;   // The buffer, which contains the stream. In most cases this may be nil,
                        // in which case the current input stream is used.
    CS_Length,
    CS_CurChr: LongInt;
  end;

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
{ The main structure used for ReadArgs(). It contains everything needed for
   ReadArgs() handling. Allocate this structure with AllocDosObject().}
  PRDArgs = ^TRDArgs;
  TRDArgs = record
    RDA_Source: TCSource;    // Select input source use this structure as source
                             // for parsing, otherwise use DosInput() as source.
    RDA_DAList: IPTR;        // PRIVATE. Must be initialized to 0
        {The next two fields allow an application to supply a buffer to be parsed
       to ReadArgs(). If either of these fields is 0, ReadArgs() allocates this
       buffer itself.}
    RDA_Buffer: PChar;       // Pointer to buffer. May be nil.
    RDA_BufSiz: LongInt;     // Size of the supplied RDA_Buffer. May be 0.
    RDA_ExtHelp: PChar;      // Additional help, if user requests it, by supplying '?' as argument.
    RDA_Flags: LongInt;      // Flags for any required control (RDAF_?)
  end;

const
  RDAB_STDIN     = 0; // Use Input() instead of the supplied command line.
  RDAB_NOALLOC   = 1; // If set, do not allocate extra string space.
  RDAB_NOPROMPT  = 2; // Do not prompt for input.

  RDAF_STDIN     = 1 shl RDAB_STDIN;
  RDAF_NOALLOC   = 1 shl RDAB_NOALLOC;
  RDAF_NOPROMPT  = 1 shl RDAB_NOPROMPT;

{ Maximum number of template keywords which can be in a template passed
  to ReadArgs(). IMPLEMENTOR NOTE - must be a multiple of 4.}
  MAX_TEMPLATE_ITEMS = 100;

{ Maximum number of MULTIARG items (/M) returned by ReadArgs(), before
  an ERROR_LINE_TOO_LONG.  These two limitations are due to stack
  usage.  Applications should allow "a lot" of stack to use ReadArgs().
  This may change in the future}
  MAX_MULTIARGS          = 128;

const
{  LockRecord() and LockRecords() locking modes. EXCLUSIVE modes mean that
   nobody else is allowed to lock a specific record, which is allowed, when
   locking with SHARED mode. When using IMMED modes, the timeout is ignored. }
  REC_EXCLUSIVE          = 0;
  REC_EXCLUSIVE_IMMED    = 1;
  REC_SHARED             = 2;
  REC_SHARED_IMMED       = 3;

// Structure as passed to LockRecords() and UnLockRecords().
type
  PRecordLock = ^TRecordLock;
  TRecordLock = record
    rec_FH    : BPTR;     // PFileHandle The file to get the current record from.
    rec_Offset,           // The offset, the current record should start.
    rec_Length,           // The length of the current record.
    rec_Mode  : LongWord; // The mode od locking (REC_*).
  end;

{ This structure describes a local variable. The list is normally held in
  Process^.pr_LocalVars. Note that this structure is READ-ONLY!
  Allocate it with SetVar(). }
type
  PLocalVar = ^TLocalVar;
  TLocalVar = record
    lv_Node: TNode;     // Standard node structure as defined in Exec
    lv_Flags: Word;
    lv_Value: STRPTR;   // The contents of the variable.
    lv_Len: LongWord;   // The length of the contents.
  end;

{ The lv_Flags bits are available to the application.  The unused
  lv_Node.ln_Pri bits are reserved for system use.}
const
// bit definitions for lv_Node.ln_Type:
  LV_VAR     = 0; // This is a variable.
  LV_ALIAS   = 1; // This is an alias.
{ This flag may be or'ed into lv_Node.ln_Type. It means that dos.library
  should ignore this entry.}
  LVB_IGNORE = 7;    // ignore this entry on GetVar, etc
  LVF_IGNORE = 1 shl LVB_IGNORE;

{ definitions of flags passed to GetVar()/SetVar()/DeleteVar()
  bit defs to be OR'ed with the type:
  item will be treated as a single line of text unless BINARY_VAR is used }
  GVB_GLOBAL_ONLY    =  8;      //The variable is not to be used locally.
  GVB_LOCAL_ONLY     =  9;      // The variable is not to be used globally.
  GVB_BINARY_VAR     = 10;     // The variable is a binary variable. lv_Value points to binary data.
  GVB_DONT_NULL_TERM = 11;     // lv_Value is not null-terminated. This is only allowed, if GVB_BINARY_VAR is also set.
  GVB_SAVE_VAR       = 12;    // This flag tells dos to save the variable to ENVARC: too.
  GVF_GLOBAL_ONLY    = 1 shl GVB_GLOBAL_ONLY;
  GVF_LOCAL_ONLY     = 1 shl GVB_LOCAL_ONLY;
  GVF_BINARY_VAR     = 1 shl GVB_BINARY_VAR;
  GVF_DONT_NULL_TERM = 1 shl GVB_DONT_NULL_TERM;
  GVF_SAVE_VAR       = 1 shl GVB_SAVE_VAR;

const
{   ***************************************************************************}
{    definitions for the System() call }

  SYS_Dummy       = (TAG_USER + 32);
  SYS_Input       = (SYS_Dummy + 1); // specifies the input filehandle
  SYS_Output      = (SYS_Dummy + 2); // specifies the output filehandle
  SYS_Asynch      = (SYS_Dummy + 3); // run asynch, close input/output on exit(!)
  SYS_UserShell   = (SYS_Dummy + 4); // send to user shell instead of boot shell
  SYS_CustomShell = (SYS_Dummy + 5); // send to a specific shell (data is name)
{$ifdef aros}
  SYS_Error       = (SYS_Dummy + 10); // (BPTR/struct FileHandle *) Output filehandle.
  SYS_ScriptInput = (SYS_Dummy + 11); // Filehandle to script to execute
  SYS_Background  = (SYS_Dummy + 12); // (BOOL) The shell is run as a "background shell
  SYS_CliNumPtr   = (SYS_Dummy + 13); // (LONG *) ti_Data to store the cli number

{ This is not a Tag its a TAG Item  Use this together with SYS_Input, SYS_Output and
  SYS_Error, to tell SystemTagList to *duplicate* the respective caller's streams.}
  SYS_DupStream   = 1;
{$endif}


{ Tags for CreateNewProc(). All tags, where no default is stated, the default
  is inherited from the parent process. Additionally you may use tags for
  AllocDosObject(DOS_CLI, ...).}
  NP_Dummy       = (TAG_USER + 1000);
  NP_Seglist     = (NP_Dummy + 1); // seglist of code to run for the process
  NP_FreeSeglist = (NP_Dummy + 2); // free seglist on exit - only valid for for NP_Seglist.  Default is True.
  NP_Entry       = (NP_Dummy + 3); // entry point to run, mutually exclusive with NP_Seglist!
  NP_Input       = (NP_Dummy + 4); // filehandle - default is Open("NIL:"...)
  NP_Output      = (NP_Dummy + 5); // filehandle - default is Open("NIL:"...)
  NP_CloseInput  = (NP_Dummy + 6); // close input filehandle on exit default True
  NP_CloseOutput = (NP_Dummy + 7); // close output filehandle on exit default True
  NP_Error       = (NP_Dummy + 8); // filehandle - default is Open("NIL:"...)
  NP_CloseError  = (NP_Dummy + 9); // close error filehandle on exit default True
  NP_CurrentDir  = (NP_Dummy + 10);// lock - default is parent's current dir
  NP_StackSize   = (NP_Dummy + 11);// stacksize for process - default 4000
  NP_Name        = (NP_Dummy + 12);// name for process - default "New Process"
  NP_Priority    = (NP_Dummy + 13);// priority - default same as parent
  NP_ConsoleTask = (NP_Dummy + 14);// consoletask - default same as parent
  NP_WindowPtr   = (NP_Dummy + 15);// window ptr - default is same as parent
  NP_HomeDir     = (NP_Dummy + 16);// home directory - default curr home dir
  NP_CopyVars    = (NP_Dummy + 17);// boolean to copy local vars-default True
  NP_Cli         = (NP_Dummy + 18);// create cli structure - default FALSE
  NP_Path        = (NP_Dummy + 19);// path - default is copy of parents path only valid if a cli process!
  NP_CommandName = (NP_Dummy + 20);// commandname - valid only for CLI
  NP_Arguments   = (NP_Dummy + 21);// If this tag is used, NP_Input must not be NULL.
//The following two tags do not work, yet.
  NP_NotifyOnDeath = (NP_Dummy + 22); // (BOOL) Notify parent, when process exits? (Default: FALSE)
  NP_Synchronous   = (NP_Dummy + 23); // (BOOL) Wait until called process returns. (Default: FALSE)

  NP_ExitCode      = (NP_Dummy + 24);// (APTR) Code that is to be called, when process exits. (Default: NULL)
  NP_ExitData      = (NP_Dummy + 25);// (APTR) Optional data for NP_ExitCode. (Default: NULL)
{$ifdef aros}
  NP_UserData      = (NP_Dummy + 26); //(IPTR) User dependant data. Do with it, what you want to. (Default: NULL)
{$endif}

{ Tags for AllocDosObject }
  ADO_Dummy        = (TAG_USER + 2000);
  ADO_FH_Mode      = (ADO_Dummy + 1); // Sets up FH to the specified mode.
  ADO_DirLen       = (ADO_Dummy + 2); // size in bytes for current dir buffer
  ADO_CommNameLen  = (ADO_Dummy + 3); // size in bytes for command name buffer
  ADO_CommFileLen  = (ADO_Dummy + 4); // size in bytes for command file buffer
  ADO_PromptLen    = (ADO_Dummy + 5); // size in bytes for the prompt buffer

type
  PRootNode = ^TRootNode;
  TRootNode = record
    rn_TaskArray: BPTR;          // Pointer to the SegList for CLIs.
    rn_ConsoleSegment: BPTR;     // SegList for the CLI
    rn_Time: TDateStamp;         // Current time
    rn_RestartSeg: APTR;         // SegList for the disk validator process
    rn_Info: BPTR;               // Pointer ot the Info structure
    rn_FileHandlerSegment: BPTR; // segment for a file handler
    rn_CliList: TMinList;        // List of all CLI processe (CliProcList)
    rn_BootProc: PMsgPort;       // private ptr to msgport of boot fs
    rn_ShellSegment: BPTR;       // seglist for Shell (for NewShell)
    rn_Flags: LongInt;           // dos flags
    rn_RootLock: TSignalSemaphore; // RootNode arbitrator
  end;

{ Structure that is linked into the rootnode's rn_CliList. Completely
   private, of course! ... and it's not compatible to AmigaOS.}
  PCLIInfo = ^TCLIInfo;
  TCLIInfo = record
    ci_Node: TNode;
    ci_Process: PProcess;
  end;

{ A structure for holding error messages - stored as array with error == 0
  for the last entry.}
  PErrorString = ^TErrorString;
  TErrorString = record
    estr_Nums: PLongInt;
    estr_Strings: STRPTR;
  end;

const
{ error report types for ErrorReport() }
  REPORT_STREAM = 0; // a stream
  REPORT_TASK   = 1; // a process - unused
  REPORT_LOCK   = 2; // a lock
  REPORT_VOLUME = 3; // a volume node
  REPORT_INSERT = 4; // please insert volume

{ Special error codes for ErrorReport() }
  ABORT_DISK_ERROR = 296; //    Read/write error
  ABORT_BUSY       = 288; //    You MUST replace...


type
// This is how the base of dos.library looks like.
  PDosLibrary = ^TDosLibrary;
  TDosLibrary = record
    dl_lib: TLibrary;
    dl_Root: PRootNode;      // Pointer to RootNode, described below }

    dl_GV: APTR;             // Pointer to BCPL global vector       }
    dl_A2: LongInt;          // Private register dump of DOS        }
    dl_A5: LongInt;
    dl_A6: LongInt;

    dl_Errors: PErrorString;    // pointer to array of error msgs
    dl_TimeReq: PTimeRequest;   // private pointer to timer request
    dl_UtilityBase  : PLibrary; // private ptr to utility library
    dl_IntuitionBase : PLibrary;
  end;

const
  RNB_WILDSTAR   = 24;
  RNF_WILDSTAR   = 1 shl RNB_WILDSTAR;
  RNB_PRIVATE1   = 1;                  // private for dos
  RNF_PRIVATE1   = 1 shl RNB_PRIVATE1;

{   ***************************************************************************}
{    tags for NewLoadSeg }
{    no tags are defined yet for NewLoadSeg }

{$ifdef aros}

type
// FSA_Open, Returns a new filehandle. The file may be newly created (depending on io_FileMode)
  PIFS_OPEN = ^TIFS_OPEN;
  TIFS_OPEN = record
    io_FileName: STRPTR;   // File to open.
    io_FileMode: LongWord; // Filemode (FMF_*)
  end;
// Reads from a filehandle into a buffer.
  PIFS_READ_WRITE = ^TIFS_READ_WRITE;
  TIFS_READ_WRITE = record
    io_Buffer: PChar;    // The buffer for the data to read/write.
    io_Length: LongInt;  // The length of the buffer. This is filled by the filesystem handler
  end;                   // with the number of bytes actually read/written.
// This action does exactly the same as the function Seek().
  PIFS_SEEK = ^TIFS_SEEK;
  TIFS_SEEK = record
    io_Offset: QWord;     // Offset from position, specified as mode. This is filled by the
                          // filehandler with the old position in the file.
    io_SeekMode: LongInt; // Seek mode (OFFSET_*)
  end;
{ Waits for a character to arrive at the filehandle. This is not used for
   plain files, but for queues only. Optionally a maximum time to wait may be
   specified.}
  PIFS_WAIT_CHAR = ^TIFS_WAIT_CHAR;
  TIFS_WAIT_CHAR = record
    io_Timeout: LongInt;  // Maximum time (in microseconds) to wait for a character.
    io_Success: LongBool; // This is set to False by the filehandler if no character arrived in
  end;                    // time. Otherwise it is set to True.
{ Applies a new mode to a file. If you supply io_Mask with a value of 0,
   no changes are made and you can just read the resulting io_FileMode.}
  PIFS_FILE_MODE = ^TIFS_FILE_MODE;
  TIFS_FILE_MODE = record
    io_FileMode: LongWord; // The new mode to apply to the filehandle. See below for definitions.
                           //  The filehandler fills this with the old mode bits.
    io_Mask: LongWord;     // This mask defines which flags are to be changed.
  end;
{ This action can be used to query if a filehandle is interactive, i.e. if it
   is a terminal or not.}
  PIFS_IS_INTERACTIVE = ^TIFS_IS_INTERACTIVE;
  TIFS_IS_INTERACTIVE = record
    io_IsInteractive: LongBool; // This boolean is filled by the filehandler. It is set to TRUE if the
                                //  filehandle is interactive, otherwise it is set to FALSE.
  end;
// Compares two locks for equality.
  PIFS_SAME_LOCK = ^TIFS_SAME_LOCK;
  TIFS_SAME_LOCK = record
    io_Lock: array[0..1] of APTR; // The two locks to compare.
    io_Same: LongInt;             // This is set to one of LOCK_DIFFERENT or LOCK_SAME
  end;
// Examines a filehandle, giving various information about it.
  PIFS_EXAMINE = ^TIFS_EXAMINE;
  TIFS_EXAMINE = record
    io_ead: PExAllData;  // ExAllData structure buffer to be filled by the filehandler.
    io_Size: LongInt;    // Size of the buffer.
    io_Mode: LongInt;    // With which kind of information shall the buffer be filled with?
  end;                   // see ED_* definitions for more information.
  PIFS_EXAMINE_NEXT = ^TIFS_EXAMINE_NEXT;
  TIFS_EXAMINE_NEXT = record
    io_fib: PFileInfoBlock; // FileInfoBlock structure buffer to be used and filled by the filehandler.
  end;
{ Works exactly like FSA_EXAMINE with the exeption that multiple files may be
   examined, i.e. the filehandle must be a directory.}
  PIFS_EXAMINE_ALL = ^TIFS_EXAMINE_ALL;
  TIFS_EXAMINE_ALL = record
    io_ead: PExAllData;
    io_eac: PExallControl;
    io_Size: LongInt;
    io_Mode: LongInt;
  end;
{ Works exactly like FSA_OPEN, but you can additionally specify protection
   bits to be applied to new files.}
  PIFS_OPEN_FILE = ^TIFS_OPEN_FILE;
  TIFS_OPEN_FILE = record
    io_Filename: STRPTR;     // File to open.
    io_FileMode: LongWord;   // see below.
    io_Protection: LongWord; // The protection bits
  end;
// Creates a new directory. The filehandle of that new directory is returned.
  PIFS_CREATE_DIR = ^TIFS_CREATE_DIR;
  TIFS_CREATE_DIR = record
    io_FileName: STRPTR;     // Name of directory to create.
    io_Protection: LongWord; // The protection bits.
  end;
// Creates a hard link (i.e. gives one file/directory a second name).
  PIFS_CREATE_HARDLINK = ^TIFS_CREATE_HARDLINK;
  TIFS_CREATE_HARDLINK = record
    io_Filename: STRPTR;  // The filename of the link to create.
    io_OldFile: APTR;     // Filehandle of the file to link to.
  end;
// Creates a soft link (i.e. a file is created that references another by its name).
  PIFS_CREATE_SOFTLINK = ^TIFS_CREATE_SOFTLINK;
  TIFS_CREATE_SOFTLINK = record
    io_Filename: STRPTR;  // The filename of the link to create.
    io_Reference: STRPTR; // The name of the file to link to.
  end;
// Renames a file. To the old and the new name, the current directory is applied to.
  PIFS_RENAME = ^TIFS_RENAME;
  TIFS_RENAME = record
    io_Filename: STRPTR;  // The old filename.
    io_NewName: STRPTR;   // The new filename.
  end;
// Resolves the full path name of the file a softlink filehandle points to.
  PIFS_READ_SOFTLINK = ^TIFS_READ_SOFTLINK;
  TIFS_READ_SOFTLINK = record
    io_Filename: STRPTR; // file name which returned ERROR_IS_SOFT_LINK
    io_Buffer: STRPTR;   { The buffer to fill with the pathname. If this buffer is too small, the
                            filesystem handler is supposed to return ERROR_LINE_TOO_LONG.}
    io_Size: LongWord;  // The size of the buffer pointed to by io_Buffer.
  end;
// Deletes an object on the volume.
  PIFS_DELETE_OBJECT = ^TIFS_DELETE_OBJECT;
  TIFS_DELETE_OBJECT = record
    io_Filename: STRPTR;  // The name of the file to delete.
  end;
// Sets a filecomment for a file.
  PIFS_SET_COMMENT = ^TIFS_SET_COMMENT;
  TIFS_SET_COMMENT = record
    io_Filename: STRPTR;  // The name of the file to be commented.
    io_Comment: STRPTR;   // The new filecomment. May be nil, in which case the current filecomment is deleted.
  end;
// Sets the protection bits of a file.
  PIFS_SET_PROTECT = ^TIFS_SET_PROTECT;
  TIFS_SET_PROTECT = record
    io_Filename: STRPTR;     // The file to change.
    io_Protection: LongWord; // The new protection bits.
  end;
// Sets the ownership of a file.
  PIFS_SET_OWNER = ^TIFS_SET_OWNER;
  TIFS_SET_OWNER = record
    io_Filename: STRPTR; // The file to change.
    io_UID: Word;        // The new owner.
    io_GID: Word;        // The new group owner.
  end;
{ Sets the last modification date/time of the filename given as first
   argument. The date/time is given as standard DateStamp structure}
  PIFS_SET_DATE = ^TIFS_SET_DATE;
  TIFS_SET_DATE = record
    io_Filename: STRPTR;  // The file to change
    io_Date: TDateStamp;  // The new date
  end;
// Check if a filesystem is in fact a FILEsystem, i.e. can contain different files.
  PIFS_IS_FILESYSTEM = ^TIFS_IS_FILESYSTEM;
  TIFS_IS_FILESYSTEM = record
    io_IsFilesystem: LongBool; // This is set to True by the filesystem handler if it is a filesystem
  end;                         // and set to False if it is not.
{ Changes the number of buffers for the filesystem. The current number of
   buffers is returned. The size of the buffers is filesystem-dependent.}
  PIFS_MORE_CACHE = ^TIFS_MORE_CACHE;
  TIFS_MORE_CACHE = record
    io_NumBuffers: LongInt; // Number of buffers to add. May be negative to reduce number of buffers.
  end;                      // This is to be set to the current number of buffers on success.
// Formats a volume, i.e. erases all data on it.
  PIFS_FORMAT = ^TIFS_FORMAT;
  TIFS_FORMAT = record
    io_VolumeName: STRPTR; // New name for the volume.
    io_DosType: LongWord;  // New type for the volume. Filesystem specific.
  end;
{ Resets/reads the mount-mode of the volume passed in as io_Unit. The first
   and second arguments work exactly like FSA_FILE_MODE, but the third
   argument can contain a password, if MMF_LOCKED is set.}
  PIFS_MOUNT_MODE = ^TIFS_MOUNT_MODE;
  TIFS_MOUNT_MODE = record
    io_MountMode: LongWord; // The new mode to apply to the volume. See below for definitions.
                            // The filehandler fills this with the old mode bits.
    io_Mask: LongWord;      // This mask defines which flags are to be changed.
    io_Password: STRPTR;    // A password, which is needed if MMF_LOCKED is set.
  end;
  PIFS_INHIBIT = ^TIFS_INHIBIT;
  TIFS_INHIBIT = record
    io_Inhibit: LongBool;
  end;
  PIFS_NOTIFY = ^TIFS_NOTIFY;
  TIFS_NOTIFY = record
    io_FileName: STRPTR;    // Needed for synchronous operation
    io_NotificationRequest: PNotifyRequest;
  end;
  PIFS_INFO = ^TIFS_INFO;
  TIFS_INFO = record
    io_Info: PInfoData;
  end;
  PIFS_CHANGE_SIGNAL = ^TIFS_CHANGE_SIGNAL;
  TIFS_CHANGE_SIGNAL = record
    io_Task: PTask;
  end;
  PIFS_RECORD = ^TIFS_RECORD;
  TIFS_RECORD = record
    io_Offset: QWord;
    io_Size: LongInt;
    io_RecordMode: LongWord;
    io_Timeout: LongWord;
  end;
  PIFS_PARENT_DIR = ^TIFS_PARENT_DIR;
  TIFS_PARENT_DIR = record
    io_DirName: PChar; // This will contain the return value of the parent directory, or
  end;                 // nil if we are at the root directory already
// Allows us to change a console between raw and cooked mode.
  PIFS_CONSOLE_MODE = ^TIFS_CONSOLE_MODE;
  TIFS_CONSOLE_MODE = record
    io_ConsoleMode: LongInt;  // (FCM_*)
  end;
  PIFS_RELABEL = ^TIFS_RELABEL;
  TIFS_RELABEL = record
    io_NewName: STRPTR;
    io_Result: LongBool;
  end;
{ FSA_PIPE: create a pair of handles connected to each other
  This opens a "file" (which will usually be a pipe device) and returns two
  handles such that writing data to the writer will result in that data
  appearing on the reader. Both handles must be closed for the underlying
  file to be closed. If a NULL/empty path is supplied, an unnamed pipe will
  be created, which will be destroyed once both handles are closed.
  The read handle is returned in io_Unit.}
  PIFS_PIPE = ^TIFS_PIPE;
  TIFS_PIPE = record
    io_FileName: STRPTR;
    io_Writer: PUnit;
  end;
const
  FSA_OPEN            =  1; // Returns a new filehandle. The file may be newly created (depending on io_FileMode) TIFS_OPEN
  FAS_CLOSE           =  2; // Closes an opened filehandle. Takes no extra arguments.
  FSA_READ            =  3; // Reads from a filehandle into a buffer. TIFS_READ_WRITE
  FSA_WRITE           =  4; // Writes the contents of a buffer into a filehandle. Uses TIFS_READ_WRITE.
  FSA_SEEK            =  5; // This action does exactly the same as the function Seek(). TIFS_SEEK
  FSA_SET_FILE_SIZE   =  6; // Sets the size of filehandle. Uses TIFS_SEEK (see above) as argument array.
  FSA_WAIT_CHAR       =  7; // Waits for a character to arrive at the filehandle. (TIFS_WAIT_CHAR)
  FSA_FILE_MODE       =  8; // Applies a new mode to a file.
  FSA_IS_INTERACTIVE  =  9; // Query if a filehandle is interactive
  FSA_SAME_LOCK       = 10; // Compares two locks for equality.
  FSA_EXAMINE         = 11; // Examines a filehandle, giving various information about it.
  FSA_EXAMINE_NEXT    = 12; // Examine next file
  FSA_EXAMINE_ALL     = 13; // Works exactly like FSA_EXAMINE on directories
  FSA_EXAMINE_ALL_END = 14; { This has to be called if FSA_EXAMINE_ALL is stopped before all examined
                              files were returned. It takes no arguments except the filehandle in io_Unit.}
  FSA_OPEN_FILE       = 15; // Works exactly like FSA_OPEN but with special protection bits
  FSA_CREATE_DIR      = 16; // Creates a new directory. The filehandle of that new directory is returned.
  FSA_CREATE_HARDLINK = 17; // Creates a hard link (i.e. gives one file/directory a second name).
  FSA_CREATE_SOFTLINK = 18; // Creates a soft link (i.e. a file is created that references another by its name).
  FSA_RENAME          = 19; // Renames a file. To the old and the new name, the current directory is applied to.
  FSA_READ_SOFTLINK   = 20; // Resolves the full path name of the file a softlink filehandle points to.
  FSA_DELETE_OBJECT   = 21; // Deletes an object on the volume.
  FSA_SET_COMMENT     = 22; // Sets a filecomment for a file.
  FSA_SET_PROTECT     = 23; // Sets the ownership of a file.
  FSA_SET_OWNER       = 24; // Sets the last modification date/time
  FSA_SET_DATE        = 25; // Set file date
  FSA_IS_FILESYSTEM   = 26; // Check if a filesystem is in fact a FILEsystem, i.e. can contain different files.
  FSA_MORE_CACHE      = 27; // Changes the number of buffers for the filesystem.
  FSA_FORMAT          = 28; // Formats a volume, i.e. erases all data on it.
  FSA_MOUNT_MODE      = 29; // Resets/reads the mount-mode
  //FSA_SERIALIZE_DISK = 30; // currently not supported
  // FSA_FLUSH         = 31; // currently not supported
  FSA_INHIBIT         = 32;
  //FSA_WRITE_PROTECT   = 33; // currently not supported
  //FSA_DISK_CHANGE     = 34; // currently not supported
  FSA_ADD_NOTIFY      = 35;
  FSA_REMOVE_NOTIFY   = 36;
  FSA_DISK_INFO       = 37;
  FSA_CHANGE_SIGNAL   = 38;
  FSA_LOCK_RECORD     = 39;
  FSA_UNLOCK_RECORD   = 40;
  FSA_PARENT_DIR      = 41;
  FSA_PARENT_DIR_POST = 42;
  FSA_CONSOLE_MODE    = 43; // Allows us to change a console between raw and cooked mode.
  FSA_RELABEL         = 44;
  FSA_PIPE            = 45; // create a pair of handles connected to each other
// io_ConsoleMode
  FCM_COOKED = 0;
  FCM_RAW    = 1 shl 0;
  FCM_NOECHO = 1 shl 1;
{ io_FileMode for FSA_OPEN, FSA_OPEN_FILE and FSA_FILE_MODE. These are flags
   and may be OR'ed. Note that not all filesystems support all flags.}
  FMF_LOCK     = 1 shl 0; // Lock exclusively.
  FMF_EXECUTE  = 1 shl 1; // Open for executing.
// At least one of the following two flags must be specified. Otherwise expect strange things to happen.
  FMF_WRITE    = 1 shl 2; // Open for writing.
  FMF_READ     = 1 shl 3; // Open for reading.
  FMF_CREATE   = 1 shl 4; // Create file if it doesn't exist.
  FMF_CLEAR    = 1 shl 5; // Truncate file on open.
  FMF_RAW      = 1 shl 6; // Switch cooked to raw and vice versa.
  FMF_NONBLOCK = 1 shl 7; // Don't block Open() in case it would and return an error in case Write()/Read()  would block
  FMF_APPEND   = 1 shl 8; // Every write will happen always at the end of the file
  FMF_AMIGADOS = (1 shl 9) or (1 shl 31); // Identifies the old AmigaDOS modes:
                                          // - bit 9 is the first bit set in the MODE_#? modes
                                          // - bit 31 is the first bit set in ACCESS_#? modes
  FMF_MODE_OLDFILE   = FMF_AMIGADOS or FMF_WRITE or FMF_READ;
  FMF_MODE_READWRITE = FMF_MODE_OLDFILE or FMF_CREATE;
  FMF_MODE_NEWFILE   = FMF_MODE_READWRITE or FMF_LOCK or FMF_CLEAR;
// io_MountMode for FSA_MOUNT_MODE. These are flags and may be OR'ed.
  MMF_READ        = 1 shl 0; // Mounted for reading.
  MMF_WRITE       = 1 shl 1; // Mounted for writing.
  MMF_READ_CACHE  = 1 shl 2; // Read cache enabled.
  MMF_WRITE_CACHE = 1 shl 3; // Write cache enabled.
  MMF_OFFLINE     = 1 shl 4; // Filesystem currently does not use the device.
  MMF_LOCKED      = 1 shl 5; // Mount mode is password protected.
{ This structure is an extended TIORequest. It is used for
   requesting actions from AROS filesystem handlers.
   Note that this structure may grow in the future. Do not depend on its size!
   You may use sizeof(TIOFileSys) nevertheless if you are reserving
   memory for a TIOFileSys as the size of it will never shrink.}
type
  PIOFileSys = ^TIOFileSys;
  TIOFileSys = record
    IOFS: TIORequest; // Standard I/O request.
    io_DosError: LongInt; // Dos error code.
    io_PacketEmulation: PDosPacket; // Private
    io_DirPos: IPTR; // Handler-private key to current directory position
    //* This union contains all the data needed for the various actions. */
    io_Union: record
      case Smallint of
        0: (io_OpenDevice : record
            io_DeviceName: STRPTR; // Name of the device to open. */
            io_Unit: IPTR;         // Number of unit to open. */
            io_Environ: ^IPTR;     // Pointer to environment array.
            io_DosName: STRPTR;    // The name with which the
                                   //    filesystem is being mounted
                                   //    (the mount point, one might
                                   //    say)
            io_DeviceNode: PDeviceNode; // The DOS entry for this
                                        // filesystem. Packet-based
                                        // filesystems expect to receive
                                        // this along with the
                                        // startup message
            end;
        );
        1: (io_NamedFile: record
          io_Filename: STRPTR;
          end;
         );
        2: (
          io_OPEN: TIFS_OPEN;                       // FSA_OPEN
          io_READ_WRITE: TIFS_READ_WRITE;           // FSA_READ, FSA_WRITE
          io_SEEK: TIFS_SEEK;                       // FSA_SEEK
          io_WAIT_CHAR: TIFS_WAIT_CHAR;             // FSA_WAIT_CHAR
          io_FILE_MODE: TIFS_FILE_MODE;             // FSA_FILE_MODE */
          io_IS_INTERACTIVE: TIFS_IS_INTERACTIVE;   // FSA_IS_INTERACTIVE */
          io_SAME_LOCK: TIFS_SAME_LOCK;             // FSA_SAME_LOCK */
          io_EXAMINE: TIFS_EXAMINE;                 // FSA_EXAMINE */
          io_EXAMINE_ALL: TIFS_EXAMINE_ALL;         // FSA_EXAMINE_ALL */
          io_EXAMINE_NEXT: TIFS_EXAMINE_NEXT;       // FSA_EXAMINE_NEXT */
          io_OPEN_FILE: TIFS_OPEN_FILE;             // FSA_OPEN_FILE */
          io_CREATE_DIR: TIFS_CREATE_DIR;           // FSA_CREATE_DIR */
          io_CREATE_HARDLINK: TIFS_CREATE_HARDLINK; // FSA_CREATE_HARDLINK */
          io_CREATE_SOFTLINK: TIFS_CREATE_SOFTLINK; // FSA_CREATE_SOFTLINK */
          io_RENAME: TIFS_RENAME;                   // FSA_RENAME */
          io_READ_SOFTLINK: TIFS_READ_SOFTLINK;     // FSA_READ_SOFTLINK */
          io_DELETE_OBJECT: TIFS_DELETE_OBJECT;     // FSA_DELETE_OBJECT */
          io_SET_COMMENT: TIFS_SET_COMMENT;         // FSA_SET_COMMENT */
          io_SET_PROTECT: TIFS_SET_PROTECT;         // FSA_SET_PROTECT */
          io_SET_OWNER: TIFS_SET_OWNER;             // FSA_SET_OWNER */
          io_SET_DATE: TIFS_SET_DATE;               // FSA_SET_DATE */
          io_IS_FILESYSTEM: TIFS_IS_FILESYSTEM;     // FSA_IS_FILESYSTEM */
          io_MORE_CACHE: TIFS_MORE_CACHE;           // FSA_MORE_CACHE */
          io_FORMAT: TIFS_FORMAT;                   // FSA_FORMAT */
          io_MOUNT_MODE: TIFS_MOUNT_MODE;           // FSA_MOUNT_MODE */
          io_INHIBIT: TIFS_INHIBIT;                 // FSA_INHIBIT */
          io_PARENT_DIR: TIFS_PARENT_DIR;           // FSA_PARENT_DIR */
          io_CONSOLE_MODE: TIFS_CONSOLE_MODE;       // FSA_CONSOLE_MODE */
          io_RELABEL: TIFS_RELABEL;                 // FSA_RELABEL */
          io_NOTIFY: TIFS_NOTIFY;                   // FSA_ADD_NOTIFY
          io_INFO: TIFS_INFO;                       // FSA_INFO
          io_RECORD: TIFS_RECORD;                   // FSA_LOCK_RECORD
          io_CHANGE_SIGNAL: TIFS_CHANGE_SIGNAL;     // FSA_CHANGE_SIGNAL
          io_PIPE: TIFS_PIPE;                       // FSA_PIPE
    );
    end;
end;

const
  ERROR_BROKEN_PIPE = 400;  // An attempt to write on a pipe without any reader has been made
  ERROR_WOULD_BLOCK = 401;  // A Read() or a Write() on a file opened with the FMF_NONBLOCK flag would block
  ERROR_INTERRUPTED = 402;  // The I/O file operation has been interrupted for some reason

// elf.h

type
  //*
  //* Define one of ELF_64BIT or ELF_32BIT in your code if you want to enforce specific
  //* version of ELF structures. Otherwize it fails back to your native machine's size.
  //*
  {$IFDEF ELF_64BIT}
  {$define elf_ptr_t}
  elf_ptr_t             = UQUAD;
  elf_uintptr_t         = UQUAD;
  elf_intptr_t          = QUAD;
  {$ENDIF}

  {$IFDEF ELF_32BIT}
  {$define elf_ptr_t}
  elf_ptr_t             = ULONG;
  elf_uintptr_t         = ULONG;
  elf_intptr_t          = LONG;
  {$ENDIF}

  {$IFNDEF elf_ptr_t}
  elf_ptr_t             = APTR;
  elf_uintptr_t         = IPTR;
  elf_intptr_t          = SIPTR;
  {$ENDIF}


Const
  SHT_PROGBITS          =  1;
  SHT_SYMTAB            =  2;
  SHT_STRTAB            =  3;
  SHT_RELA              =  4;
  SHT_NOBITS            =  8;
  SHT_REL               =  9;
  SHT_SYMTAB_SHNDX      = 18;
  SHT_ARM_ATTRIBUTES    = $70000003;

  ET_REL                =  1;
  ET_EXEC               =  2;

  EM_386                =  3;
  EM_68K                =  4;
  EM_PPC                = 20;
  EM_ARM                = 40;
  EM_X86_64             = 62;     //* AMD x86-64 */

  R_386_NONE            = 0;
  R_386_32              = 1;
  R_386_PC32            = 2;

  //* AMD x86-64 relocations.  */
  R_X86_64_NONE         =  0;      //* No reloc */
  R_X86_64_64           =  1;      //* Direct 64 bit  */
  R_X86_64_PC32         =  2;      //* PC relative 32 bit signed */
  R_X86_64_32           = 10;
  R_X86_64_32S          = 11;

  R_68K_NONE            = 0;
  R_68K_32              = 1;
  R_68K_16              = 2;
  R_68K_8               = 3;
  R_68K_PC32            = 4;
  R_68K_PC16            = 5;
  R_68K_PC8             = 6;

  R_PPC_NONE            =   0;
  R_PPC_ADDR32          =   1;
  R_PPC_ADDR16_LO       =   4;
  R_PPC_ADDR16_HA       =   6;
  R_PPC_REL24           =  10;
  R_PPC_REL32           =  26;
  R_PPC_REL16_LO        = 250;
  R_PPC_REL16_HA        = 252;

  R_ARM_NONE            =  0;
  R_ARM_PC24            =  1;
  R_ARM_ABS32           =  2;
  R_ARM_CALL            = 28;
  R_ARM_JUMP24          = 29;
  R_ARM_TARGET1         = 38;
  R_ARM_V4BX            = 40;
  R_ARM_TARGET2         = 41;
  R_ARM_PREL31          = 42;
  R_ARM_MOVW_ABS_NC     = 43;
  R_ARM_MOVT_ABS        = 44;
  R_ARM_THM_CALL        = 10;
  R_ARM_THM_JUMP24      = 30;
  R_ARM_THM_MOVW_ABS_NC = 47;
  R_ARM_THM_MOVT_ABS    = 48;

  STT_NOTYPE            =  0;
  STT_OBJECT            =  1;
  STT_FUNC              =  2;
  STT_SECTION           =  3;
  STT_FILE              =  4;
  STT_LOPROC            = 13;
  STT_HIPROC            = 15;

  STB_LOCAL             =  0;
  STB_GLOBAL            =  1;
  STB_WEAK              =  2;
  STB_LOOS              = 10;
  STB_GNU_UNIQUE        = 10;
  STB_HIOS              = 12;
  STB_LOPROC            = 13;
  STB_HIPROC            = 15;

  SHN_UNDEF             = 0;
  SHN_LORESERVE         = $ff00;
  SHN_ABS               = $fff1;
  SHN_COMMON            = $fff2;
  SHN_XINDEX            = $ffff;
  SHN_HIRESERVE         = $ffff;

  SHF_WRITE             = (1 shl 0);
  SHF_ALLOC             = (1 shl 1);
  SHF_EXECINSTR         = (1 shl 2);

  //  ELF_ST_TYPE(i)    ((i) & 0x0F)

  EI_VERSION            =  6;
  EV_CURRENT            =  1;

  EI_DATA               =  5;
  ELFDATA2LSB           =  1;
  ELFDATA2MSB           =  2;

  EI_CLASS              =  4;
  ELFCLASS32            =  1;
  ELFCLASS64            =  2;             //* 64-bit objects */

  EI_OSABI              =  7;
  EI_ABIVERSION         =  8;

  ELFOSABI_AROS         = 15;

  PF_X                  = (1 shl 0);

  ATTR_VERSION_CURRENT  = $41;

type
  PElfHeader = ^TELFHeader;
  TElfHeader = record
    Ident:     array [0..16-1] of Byte;
    Type_:     Word;
    Machine:   Word;
    Version:   LongWord;
    Entry:     elf_ptr_t;
    PhOff:     elf_uintptr_t;
    ShOff:     elf_uintptr_t;
    Flags:     LongWord;
    EhSize:    Word;
    PhentSize: Word;
    PhNum:     Word;
    ShentSize: Word;
    ShNum:     Word;
    ShStrndx:  Word;
  end;

  PSHeader  = ^TSHeader;
  TSHeader = record
    Name:      LongWord;
    Type_:     LongWord;
    Flags:     elf_uintptr_t ;
    Addr:      elf_ptr_t     ;
    Offset:    elf_uintptr_t ;
    Size:      elf_uintptr_t ;
    Link:      LongWord;
    Info:      LongWord;
    AddrAlign: elf_uintptr_t ;
    EntSize:   elf_uintptr_t ;
  end;

  {$DEFINE PT_LOAD}

{$IFDEF ELF_64BIT}
  TPHeader = record
    Type_:  LongWord;
    Flags:  LongWord;
    Offset: elf_uintptr_t;
    VAddr:  elf_ptr_t;
    PAddr:  elf_ptr_t;
    Filesz: elf_uintptr_t;
    Memsz:  elf_uintptr_t;
    Align:  elf_uintptr_t;
  end;

  TSymbol = record
    Name:    LongWord; // Offset of the name string in the string table
    Info:    Byte;     // What kind of symbol is this ? (global, variable, etc)
    Other:   Byte;     // undefined
    ShIndex: Word;     // In which section is the symbol defined ?
    Value:   elf_uintptr_t ; // Varies; eg. the offset of the symbol in its hunk
    Size:    elf_uintptr_t ; // How much memory does the symbol occupy
  end;

  // 209 #define ELF_R_SYM(i)          (ULONG)((i) >> 32)
  // 210 #define ELF_R_TYPE(i)         (ULONG)((i) & 0xffffffffULL)
  // 211 #define ELF_R_INFO(sym, type) (((UQUAD)(sym) << 32) + (type))

{$ELSE ELF_64BIT}
  TPHeader = record
    Type_:  LongWord;
    Offset: LongWord;
    VAddr:  elf_ptr_t;
    PAddr:  elf_ptr_t;
    Filesz: LongWord;
    Memsz:  LongWord;
    Flags:  LongWord;
    Align:  LongWord;
  end;

  TSymbol = record
    Name:    LongWord;       // Offset of the name string in the string table
    Value:   elf_uintptr_t;  // Varies; eg. the offset of the symbol in its hunk
    Size:    elf_uintptr_t;  // How much memory does the symbol occupy
    Info:    Byte;           // What kind of symbol is this ? (global, variable, etc)
    Other:   Byte;           // undefined
    ShIndex: Word;           // In which section is the symbol defined?
  end;

  // 237 #define ELF_R_SYM(val)        ((val) >> 8)
  // 238 #define ELF_R_TYPE(val)       ((val) & 0xff)
  // 239 #define ELF_R_INFO(sym, type) (((sym) << 8) + ((type) & 0xff))
{$ENDIF}


  // 243 #define ELF_S_BIND(val)         ((val) >> 4)
  // 244 #define ELF_S_TYPE(val)         ((val) & 0xF)
  // 245 #define ELF_S_INFO(bind, type)  (((bind) << 4) + ((type) & 0xF))

  TRel = record
    Offset: elf_uintptr_t;     // Address of the relocation relative to the section it refers to
    Info:   elf_uintptr_t;     // Type of the relocation
  end;

  TRelA = record
    Offset: elf_uintptr_t;     // Address of the relocation relative to the section it refers to
    Info:   elf_uintptr_t;     // Type of the relocation
    Addend: elf_uintptr_t;     // Constant addend used to compute value
  end;


  (*
 260 /* Note: the conversion below is not in line with ELF specification and is fixed in GNU binutils since 2008
 261  * See: https://sourceware.org/bugzilla/show_bug.cgi?id=5900
 262  */
 263 /* convert section header number to array index */
 264 /*#define SHINDEX(n) \
 265     ((n) < SHN_LORESERVE ? (n) : ((n) <= SHN_HIRESERVE ? 0 : (n) - (SHN_HIRESERVE + 1 - SHN_LORESERVE)))*/
 266
 267 /* convert section header array index to section number */
 268 /*#define SHNUM(i) \
 269     ((i) < SHN_LORESERVE ? (i) : (i) + (SHN_HIRESERVE + 1 - SHN_LORESERVE))*/
 270
 271 /* ARM-specific attributes section definitions follow */
 272
 273 #define
  *)

  TAttrs_Section = record
    Size:   LongWord;
    Vendor: array[0..0] of char;   // NULL-terminated name
  end;                             // Vendor-specific subsections follow

  TAttrs_SubSection = packed record
    Tag: Byte;
    Size: LongWord;
  end;

const
  Tag_File                 = 1;
  Tag_Section              = 2;
  Tag_Symbol               = 3;
  Tag_CPU_raw_name         = 4;
  Tag_CPU_name             = 5;
  Tag_CPU_arch             = 6;
  Tag_FP_arch              = 10;
  Tag_compatibility        = 32;
  Tag_also_compatible_with = 65;
  Tag_conformance          = 67;

  // Tag_CPU_arch values
  ELF_CPU_PREv4    = 0;
  ELF_CPU_ARMv4    = 1;
  ELF_CPU_ARMv4T   = 2;
  ELF_CPU_ARMv5T   = 3;
  ELF_CPU_ARMv5TE  = 4;
  ELF_CPU_ARMv5TEJ = 5;
  ELF_CPU_ARMv6    = 6;
  ELF_CPU_ARMv6KZ  = 7;
  ELF_CPU_ARMv6T2  = 8;
  ELF_CPU_ARMv6K   = 9;
  ELF_CPU_ARMv7    = 10;
  ELF_CPU_ARM_v6M  = 11;
  ELF_CPU_ARMv6SM  = 12;
  ELF_CPU_ARMv7EM  = 13;

  //* Tag_FP_arch values */
  ELF_FP_None     = 0;
  ELF_FP_v1       = 1;
  ELF_FP_v2       = 2;
  ELF_FP_v3       = 3;
  ELF_FP_v3_Short = 4;
  ELF_FP_v4       = 5;
  ELF_FP_v4_Short = 6;

{$endif}

function DOSOpen(const Name: STRPTR; AccessMode: LongInt): BPTR; syscall AOS_DOSBase 5;
function DOSClose(File_: BPTR): LongBool; syscall AOS_DOSBase 6;
function DOSRead(File_: BPTR; Buffer: APTR; Length: LongInt): LongInt; syscall AOS_DOSBase 7;
function DOSWrite(File_: BPTR; Buffer: APTR; Length: LongInt): LongInt;  syscall AOS_DOSBase 8;
function DOSInput: BPTR; syscall AOS_DOSBase 9;
function DOSOutput : BPTR; syscall AOS_DOSBase 10;
function DOSSeek(File_: BPTR; Position: LongInt; Mode: LongInt): LongInt; syscall AOS_DOSBase 11;
function DOSDeleteFile(const Name: STRPTR): LongBool; syscall AOS_DOSBase 12;
function DOSRename(const OldName: STRPTR; const NewName: STRPTR): LongInt; syscall AOS_DOSBase 13;
function Lock(const Name: STRPTR; AccessMode: LongInt): BPTR; syscall AOS_DOSBase 14;
function UnLock(Lock: BPTR): LongBool; syscall AOS_DOSBase 15;
function DupLock(Lock: BPTR): BPTR; syscall AOS_DOSBase 16;
function Examine(Lock: BPTR; FileInfoBlock: PFileInfoBlock): LongInt; syscall AOS_DOSBase 17;
function ExNext(Lock: BPTR; FileInfoBlock: PFileInfoBlock): LongInt; syscall AOS_DOSBase 18;
function Info(Lock: BPTR; ParameterBlock: PInfoData): LongInt; syscall AOS_DOSBase 19;
function DOSCreateDir(const Name: STRPTR): BPTR; syscall AOS_DOSBase 20;
function CurrentDir(Lock: BPTR): BPTR; syscall AOS_DOSBase 21;
function IoErr: LongInt; syscall AOS_DOSBase 22;
function CreateProc(const Name: STRPTR; Pri: LongInt; SegList: BPTR; StackSize: LongInt): PMsgPort; syscall AOS_DOSBase 23;
procedure DOSExit(ReturnCode: LongInt); syscall AOS_DOSBase 24;
function LoadSeg(const Name: STRPTR): BPTR; syscall AOS_DOSBase 25;
procedure UnLoadSeg(Seglist: BPTR); syscall AOS_DOSBase 26;
function DeviceProc(const Name: STRPTR): PMsgPort; syscall AOS_DOSBase 29;
function SetComment(const Name: STRPTR; const Comment: STRPTR): LongInt; syscall AOS_DOSBase 30;
function SetProtection(const Name: STRPTR; Protect: LongWord): LongInt; syscall AOS_DOSBase 31;
function DateStamp(Date: PDateStamp): PDateStamp; syscall AOS_DOSBase 32;
procedure DOSDelay(TimeOut: LongWord); syscall AOS_DOSBase 33;
function WaitForChar(File_: BPTR; TimeOut: LongInt): LongInt; syscall AOS_DOSBase 34;
function ParentDir(Lock: BPTR): BPTR; syscall AOS_DOSBase 35;
function IsInteractive(File_: BPTR): LongInt; syscall AOS_DOSBase 36;
function Execute(const String_: STRPTR; Input: BPTR; Output: BPTR): LongInt; syscall AOS_DOSBase 37;
function AllocDosObject(Type_: LongWord; const Tags: PTagItem): APTR; syscall AOS_DOSBase 38;
function AllocDosObjectTagList(Type_: LongWord; const Tags: PTagItem): APTR; syscall AOS_DOSBase 38;
procedure FreeDosObject(Type_: LongWord; Ptr: APTR); syscall AOS_DOSBase 39;
function DoPkt(Port: PMsgPort; Action: LongInt; Arg1, Arg2, Arg3, Arg4, Arg5: LongInt): LongInt; syscall AOS_DOSBase 40;
procedure SendPkt(Dp: PDosPacket; Port: PMsgPort; ReplyPort: PMsgPort); syscall AOS_DOSBase 41;
function WaitPkt: PDosPacket; syscall AOS_DOSBase 42;
procedure ReplyPkt(Dp: PDosPacket; Res1: LongInt; Res2: LongInt); syscall AOS_DOSBase 43;
procedure AbortPkt(Port: PMsgPort; Pkt: PDosPacket); syscall AOS_DOSBase 44;
function LockRecord(Fh: BPTR; Offset: LongWord; Length: LongWord; Mode: LongWord; Timeout: LongWord): LongBool; syscall AOS_DOSBase 45;
function LockRecords(RecArray: PRecordLock; TimeOut: LongWord): LongBool; syscall AOS_DOSBase 46;
function UnLockRecord(Fh: BPTR; Offset: LongWord; Length: LongWord): LongBool; syscall AOS_DOSBase 47;
function UnLockRecords(RecArray: PRecordLock): LongBool; syscall AOS_DOSBase 48;
function SelectInput(Fh: BPTR): BPTR; syscall AOS_DOSBase 49;
function SelectOutput(Fh: BPTR): BPTR; syscall AOS_DOSBase 50;
function FGetC(File_: BPTR): LongInt; syscall AOS_DOSBase 51;
function FPutC(File_: BPTR; Character: LongInt): LongInt; syscall AOS_DOSBase 52;
function UnGetC(File_: BPTR; Character: LongInt): LongInt; syscall AOS_DOSBase 53;
function FRead(Fh: BPTR; Block: APTR; Blocklen: LongWord; Number: LongWord): LongInt; syscall AOS_DOSBase 54;
function FWrite(Fh: BPTR; Block: APTR; Blocklen: LongWord; NumBlocks: LongWord): LongInt; syscall AOS_DOSBase 55;
function FGets(Fh: BPTR; Buf: STRPTR; BufLen: LongWord): STRPTR; syscall AOS_DOSBase 56;
function FPuts(File_: BPTR; const String_: STRPTR): LongInt; syscall AOS_DOSBase 57;
function VFWritef(Fh: BPTR; const Fmt: STRPTR; const ArgArray: PLongInt): LongInt; syscall AOS_DOSBase 58;
function VFPrintf(Fh: BPTR; const format: STRPTR; const ArgArray: PLongInt): LongInt; syscall AOS_DOSBase 59;
function DOSFlush(File_: BPTR): LongInt; syscall AOS_DOSBase 60;
function SetVBuf(File_: BPTR; Buff: STRPTR; Type_: LongInt; Size: LongInt): LongInt; syscall AOS_DOSBase 61;
function DupLockFromFH(Lock: BPTR): BPTR; syscall AOS_DOSBase 62;
function OpenFromLock(Lock: BPTR): BPTR; syscall AOS_DOSBase 63;
function ParentOfFH(Fh: BPTR): BPTR; syscall AOS_DOSBase 64;
function ExamineFH(Fh: BPTR; Fib: PFileInfoBlock): LongBool; syscall AOS_DOSBase 65;
function SetFileDate(const Name: STRPTR; Date: PDateStamp): LongBool; syscall AOS_DOSBase 66;
function NameFromLock(Lock: BPTR; Buffer: STRPTR; Length: LongInt): LongBool; syscall AOS_DOSBase 67;
function NameFromFH(Fh: BPTR; Buffer: STRPTR; Length: LongInt): LongBool; syscall AOS_DOSBase 68;
function SplitName(const Name: STRPTR; Seperator: LongWord; Buf: STRPTR; OldPos: LongInt; Size: LongInt): LongInt; syscall AOS_DOSBase 69;
function SameLock(Lock1: BPTR; Lock2: BPTR): LongInt; syscall AOS_DOSBase 70;
function SetMode(Fh: BPTR; Mode: LongInt): LongBool; syscall AOS_DOSBase 71;
function ExAll(Lock: BPTR; Buffer: PExAllData; Size: LongInt; Data: LongInt; Control: PExAllControl): LongBool; syscall AOS_DOSBase 72;
function ReadLink(Port: PMsgPort; Lock: LongInt; const Path: STRPTR; Buffer: STRPTR; Size: LongWord): LongInt; syscall AOS_DOSBase 73;
function MakeLink(const Name: STRPTR; Dest: APTR; Soft: LongInt): LongInt; syscall AOS_DOSBase 74;
function ChangeMode(Type_: LongWord; Object_: BPTR; NewMode: LongWord): LongBool; syscall AOS_DOSBase 75;
function SetFileSize(File_: BPTR; Offset: LongInt; Mode: LongInt): LongInt; syscall AOS_DOSBase 76;
function SetIoErr(Result_: LongInt): LongInt; syscall AOS_DOSBase 77;
function Fault(Code: LongInt; Header: STRPTR; Buffer: STRPTR; Len: LongInt): LongBool; syscall AOS_DOSBase 78;
function PrintFault(Code: LongInt; const Header: STRPTR): LongBool; syscall AOS_DOSBase 79;
function ErrorReport(Code: LongInt; Type_: LongInt; Arg1: IPTR; Device: PMsgPort): LongBool; syscall AOS_DOSBase 80;
function DisplayError(FormstStr: STRPTR; Flags: LongWord; Args: APTR): LongInt; syscall AOS_DOSBase 81;
function Cli: PCommandLineInterface; syscall AOS_DOSBase 82;
function CreateNewProc(const Tags: PTagItem): PProcess; syscall AOS_DOSBase 83;
function CreateNewProcTagList(const Tags: PTagItem): PProcess; syscall AOS_DOSBase 83;
function RunCommand(SegList: BPTR; StackSize: LongWord; const ArgPtr: STRPTR; ArgSize: LongWord): LongInt; syscall AOS_DOSBase 84;
function GetConsoleTask: PMsgPort; syscall AOS_DOSBase 85;
function SetConsoleTask(const Handler: PMsgPort): PMsgPort; syscall AOS_DOSBase 86;
function GetFileSysTask: PMsgPort; syscall AOS_DOSBase 87;
function SetFileSysTask(const Task: PMsgPort): PMsgPort; syscall AOS_DOSBase 88;
function GetArgStr: STRPTR; syscall AOS_DOSBase 89;
function SetArgStr(const String_: STRPTR): STRPTR; syscall AOS_DOSBase 90;
function FindCliProc(Num: LongWord): PProcess; syscall AOS_DOSBase 91;
function MaxCli: LongWord; syscall AOS_DOSBase 92;
function SetCurrentDirName(const Name: STRPTR): LongBool; syscall AOS_DOSBase 93;
function GetCurrentDirName(Buf: STRPTR; Len: LongInt): LongBool; syscall AOS_DOSBase 94;
function SetProgramName(const Name: STRPTR): LongBool; syscall AOS_DOSBase 95;
function GetProgramName(Buf: STRPTR; Len: LongInt): LongBool; syscall AOS_DOSBase 96;
function SetPrompt(const Name: STRPTR): LongBool; syscall AOS_DOSBase 97;
function GetPrompt(Buf: STRPTR; Len: LongInt): LongBool; syscall AOS_DOSBase 98;
function SetProgramDir(Lock: BPTR): BPTR; syscall AOS_DOSBase 99;
function GetProgramDir: BPTR; syscall AOS_DOSBase 100;
function SystemTagList(const Command: STRPTR; const Tags: PTagItem): LongInt; syscall AOS_DOSBase 101;
function DOSSystem(const Command: STRPTR; const Tags: PTagItem): LongInt; syscall AOS_DOSBase 101;
function AssignLock(const Name: STRPTR; Lock: BPTR): LongInt; syscall AOS_DOSBase 102;
function AssignLate(const Name: STRPTR; const Path: STRPTR): LongBool; syscall AOS_DOSBase 103;
function AssignPath(const Name: STRPTR; const Path: STRPTR): LongBool; syscall AOS_DOSBase 104;
function AssignAdd(const Name: STRPTR; Lock: BPTR): LongBool; syscall AOS_DOSBase 105;
function RemAssignList(const Name: STRPTR; Lock: BPTR): LongInt; syscall AOS_DOSBase 106;
function GetDeviceProc(const Name: STRPTR; Dp: PDevProc): PDevProc; syscall AOS_DOSBase 107;
procedure FreeDeviceProc(Dp: PDevProc); syscall AOS_DOSBase 108;
function LockDosList(Flags: LongWord): PDosList; syscall AOS_DOSBase 109;
procedure UnLockDosList(Flags: LongWord); syscall AOS_DOSBase 110;
function AttemptLockDosList(Flags: LongWord): PDosList; syscall AOS_DOSBase 111;
function RemDosEntry(DList: PDosList): LongInt; syscall AOS_DOSBase 112;
function AddDosEntry(DList: PDosList): LongInt; syscall AOS_DOSBase 113;
function FindDosEntry(const DList: PDosList; const Name: STRPTR; Flags: LongWord): PDosList; syscall AOS_DOSBase 114;
function NextDosEntry(const DList: PDosList; Flags: LongWord): PDosList; syscall AOS_DOSBase 115;
function MakeDosEntry(const Name: STRPTR; Type_: LongInt): PDosList; syscall AOS_DOSBase 116;
procedure FreeDosEntry(DList: PDosList); syscall AOS_DOSBase 117;
function IsFileSystem(const Name: STRPTR): LongBool; syscall AOS_DOSBase 118;
function DosFormat(const DeviceName: STRPTR; const VolumeName: STRPTR; DosType: LongWord): LongBool; syscall AOS_DOSBase 119;
function Relabel(const Drive: STRPTR; const NewName: STRPTR): LongInt; syscall AOS_DOSBase 120;
function Inhibit(const Name: STRPTR; OnOff: LongInt): LongInt; syscall AOS_DOSBase 121;
function AddBuffers(const DeviceName: STRPTR; NumbBuffers: LongInt): LongBool; syscall AOS_DOSBase 122;
function CompareDates(const Date1: PDateStamp; const Date2: PDateStamp): LongInt; syscall AOS_DOSBase 123;
function DOSDateToStr(Datetime: _PDateTime): LongBool; syscall AOS_DOSBase 124;
function DOSStrToDate(DateTime: _PDateTime): LongBool; syscall AOS_DOSBase 125;
function InternalLoadSeg(Fh: BPTR; Table: BPTR; const FuncArray: PLongInt; var Stack: LongInt): BPTR; syscall AOS_DOSBase 126;
function InternalUnLoadSeg(SegList: BPTR; FreeFunc: TProcedure): LongBool; syscall AOS_DOSBase 127;
function NewLoadSeg(const File_: STRPTR; const Tags: PTagItem): BPTR; syscall AOS_DOSBase 128;
function NewLoadSegTagList(const File_: STRPTR; const Tags: PTagItem): BPTR; syscall AOS_DOSBase 128;
function AddSegment(const Name: STRPTR; Seg: BPTR; Type_: LongInt): LongBool; syscall AOS_DOSBase 129;
function FindSegment(const Name: STRPTR; const Seg: PSegment; System: LongBool): PSegment; syscall AOS_DOSBase 130;
function RemSegment(Seg: PSegment): LongInt; syscall AOS_DOSBase 131;
function CheckSignal(Mask: LongInt): LongInt; syscall AOS_DOSBase 132;
function ReadArgs(const Template: STRPTR; Array_: PIPTR; RdArgs: PRDArgs): PRDArgs; syscall AOS_DOSBase 133;
function FindArg(const Template: STRPTR; const KeyWord: STRPTR): LongInt; syscall AOS_DOSBase 134;
function ReadItem(const Buffer: STRPTR; MaxChars: LongInt; CSource: PCSource): LongInt; syscall AOS_DOSBase 135;
function StrToLong(const String_: STRPTR; var Value: LongInt): LongInt; syscall AOS_DOSBase 136;
function MatchFirst(const Pat: STRPTR; AP: PAnchorPath): LongInt; syscall AOS_DOSBase 137;
function MatchNext(AP: PAnchorPath): LongInt; syscall AOS_DOSBase 138;
procedure MatchEnd(AP: PAnchorPath); syscall AOS_DOSBase 139;
function ParsePattern(const Source: STRPTR; Dest: STRPTR; DestLength: LongInt): LongInt; syscall AOS_DOSBase 140;
function MatchPattern(const Pat: STRPTR; Str: STRPTR): LongBool; syscall AOS_DOSBase 141;
procedure FreeArgs(Args: PRDArgs); syscall AOS_DOSBase 143;
function FilePart(const Path: STRPTR): STRPTR; syscall AOS_DOSBase 145;
function PathPart(const Path: STRPTR): STRPTR; syscall AOS_DOSBase 146;
function AddPart(DirName: STRPTR; const FileName: STRPTR; Size: LongWord): LongBool; syscall AOS_DOSBase 147;
function StartNotify(Notify: PNotifyRequest): LongBool; syscall AOS_DOSBase 148;
procedure EndNotify(Notify: PNotifyRequest); syscall AOS_DOSBase 149;
function SetVar(const Name: STRPTR; Buffer: PChar; Size: LongInt; Flags: LongInt): LongBool;  syscall AOS_DOSBase 150;
function GetVar(const Name: STRPTR; Buffer: STRPTR; Size: LongInt; Flags: LongInt): LongInt; syscall AOS_DOSBase 151;
function DeleteVar(const Name: STRPTR; Flags: LongWord): LongInt; syscall AOS_DOSBase 152;
function FindVar(const Name: STRPTR; Type_: LongWord): PLocalVar; syscall AOS_DOSBase 153;
function CliInit(Dp: PDosPacket): IPTR; syscall AOS_DOSBase 154;
function CliInitNewcli(Dp: PDosPacket): IPTR; syscall AOS_DOSBase 155;
function CliInitRun(Dp: PDosPacket): IPTR; syscall AOS_DOSBase 156;
function WriteChars(const Buf: STRPTR; BufLen: LongWord): LongInt; syscall AOS_DOSBase 157;
function PutStr(const String_: STRPTR): LongInt; syscall AOS_DOSBase 158;
function VPrintf(const Format: STRPTR; var ArgArray: IPTR): LongInt; syscall AOS_DOSBase 159;
function ParsePatternNoCase(const Source: STRPTR; Dest: STRPTR; DestLen: LongInt): LongInt; syscall AOS_DOSBase 161;
function MatchPatternNoCase(const Pat: STRPTR; Str: STRPTR): LongBool; syscall AOS_DOSBase 162;
function DosGetString(StringNum: LongInt): STRPTR; syscall AOS_DOSBase 163;
function SameDevice(Lock1: BPTR; Lock2: BPTR): LongBool; syscall AOS_DOSBase 164;
procedure ExAllEnd(Lock: BPTR; Buffer: PExAllData; Size: LongInt; Data: LongInt; Control: PExAllControl); syscall AOS_DOSBase 165;
function SetOwner(const Name: STRPTR; Owner_Info: LongWord): LongBool; syscall AOS_DOSBase 166;
function ScanVars(Hook: PHook; Flags: LongWord; UserData: APTR): LongInt; syscall AOS_DOSBase 167;

{$ifdef AROS_ABIv0}
function RunHandler(DevNode: PDeviceNode; Path: PChar): PMsgPort; syscall AOS_DOSBase 27;
function DosError(): BPTR; syscall AOS_DOSBase 142;
function SelectError(Fh: BPTR): BPTR; syscall AOS_DOSBase 144;
function Pipe(const Name: STRPTR; var Reader: BPTR; var Writer: BPTR): LongInt; syscall AOS_DOSBase 160;
{$endif}

{$ifdef AROS_ABIv1}
function DosError(): BPTR;
{$endif}

function ReadChar(): LongInt;
function WriteChar(c: LongInt): LongInt;
function UnReadChar(c: LongInt): LongInt;

// Special functions for var args
function AllocDosObjectTags(const Type_: LongWord; const Tags: array of PtrUInt): APTR;
function CreateNewProcTags(const Tags: array of PtrUInt): PProcess;
function NewLoadSegTags(const File_: STRPTR; const Tags: array of PtrUInt): BPTR;
function SystemTags(const Command: STRPTR; const Tags: array of PtrUInt): LongInt;

// elf.h

function ELF_ST_TYPE(i: LongWord): LongWord;
{$ifdef ELF_64BIT}
function ELF_R_SYM(i: QWord): QWord;
function ELF_R_TYPE(i: QWord): QWord;
function ELF_R_INFO(Sym: QWord; Type_: QWord): QWord;
{$else}
function ELF_R_SYM(i: LongWord): LongWord;
function ELF_R_TYPE(i: LongWord): LongWord;
function ELF_R_INFO(Sym: LongWord; Type_: LongWord): LongWord;
{$endif}

const
  BNULL = nil;

function MKBADDR(a: APTR): BPTR;
function BADDR(a: BPTR): APTR;

implementation

function ELF_ST_TYPE(i: LongWord): LongWord; inline;
begin
  ELF_ST_TYPE := i and $0F;
end;

{$ifdef ELF_64BIT}
  function ELF_R_SYM(i: QWord): QWord; inline;
  begin
    ELF_R_SYM := i shr 32;
  end;

  function ELF_R_TYPE(i: QWord): QWord; inline;
  begin
    ELF_R_TYPE := i and $ffffffff;
  end;

  function ELF_R_INFO(Sym: QWord; Type_: QWord): QWord; inline;
  begin
    ELF_R_INFO := Sym shl 32 + Type_;
  end;
{$else}
  function ELF_R_SYM(i: LongWord): LongWord; inline;
  begin
    ELF_R_SYM := i shr 8;
  end;

  function ELF_R_TYPE(i: LongWord): LongWord; inline;
  begin
    ELF_R_TYPE := i and $ff;
  end;

  function ELF_R_INFO(Sym: LongWord; Type_: LongWord): LongWord; inline;
  begin
    ELF_R_INFO := Sym shl 8 + (Type_ and $ff);
  end;
{$endif}

function AllocDosObjectTags(const Type_: LongWord; const Tags: array of PtrUInt): APTR; inline;
begin
  AllocDosObjectTags := AllocDosObject(Type_, @Tags);
end;

function CreateNewProcTags(const Tags: array of PtrUInt): PProcess; inline;
begin
  CreateNewProcTags := CreateNewProc(@Tags);
end;

function NewLoadSegTags(const File_: STRPTR; const Tags: array of PtrUInt): BPTR; inline;
begin
  NewLoadSegTags := NewLoadSeg(File_, @Tags);
end;

function SystemTags(const Command: STRPTR; const Tags: array of PtrUInt): LongInt; inline;
begin
  SystemTags := SystemTagList(Command, @Tags);
end;

function MKBADDR(a: APTR): BPTR; inline;
begin
  {$ifdef AROS_FAST_BPTR}
  MKBADDR := a;
  {$else}
  MKBADDR := APTR((IPTR(a)) shr 2);
  {$endif}
end;

function BADDR(a: BPTR): APTR; inline;
begin
  {$ifdef AROS_FAST_BPTR}
  BADDR := a;
  {$else}
  BADDR := BPTR((IPTR(a)) shl 2);
  {$endif}
end;

function ReadChar(): LongInt; inline;
begin
  ReadChar := FGetC(DosInput());
end;

function WriteChar(c: LongInt): LongInt; inline;
begin
  WriteChar := FPutC(DosOutput(), c);
end;

function UnReadChar(c: LongInt): LongInt; inline;
begin
  UnReadChar := UnGetC(DosInput(),c);
end;

{$ifdef AROS_ABIv1}
function DosError(): BPTR;
var
  P: PProcess;
begin
  DosError := nil;
  P := PProcess(FindTask(nil));
  if Assigned(P) then
    DosError := P^.pr_CES;
  if DosError = nil then
    DosError := DosOutput();
end;
{$endif}

end. (* UNIT DOS *)


