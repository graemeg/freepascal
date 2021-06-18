{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by Free Pascal development team

    dos.library functions for Amiga OS 4.x

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$PACKRECORDS 2}

unit amigados;

interface

uses
  exec, utility, timer;

const
// Predefined Amiga DOS global constants
  DOSTRUE     = -1;
  DOSFALSE    =  0;

  BITSPERBYTE  = 8;
  BYTESPERLONG = 4;
  BITSPERLONG  = 32;
  MAXINT       = $7FFFFFFF;
  MININT       = $80000000;

// Mode parameter to Open(), all files are positioned at beginning of file.
  MODE_OLDFILE   = 1005; // Open existing file read/write positioned at beginning of file.
  MODE_NEWFILE   = 1006; // Open freshly created file (delete old file) read/write
  MODE_READWRITE = 1004; // Open old file w/exclusive lock

// Relative position to Seek(deprecated) and ChangeFilePosition()
  OFFSET_BEGINNING = -1; // relative to Begining Of File
  OFFSET_CURRENT   = 0;  // relative to Current file position
  OFFSET_END       = 1;  // relative to End Of File

// Passed as 'accessMode' to Lock()
  SHARED_LOCK    = -2; // File is readable by others
  ACCESS_READ    = -2; //   Somewhat misleading old synonym
  EXCLUSIVE_LOCK = -1; // No other access allowed
  ACCESS_WRITE   = -1; //   Somewhat misleading old synonym

// *********************************************************************
// Flags for the bitmasks returned by LockTags() when using LK_InfoMask tag.*/
// These provide detailed information on the string being resolved.  V53.57 */

  LOCKB_SOFTLINK_MIDPART = 8;  // One or more soft links were found in the middle of the path
                               // but none of these formed the last component of the path.
  LOCKB_SOFTLINK_ENDPART = 9;  // One soft link exists as the last component of the path.
  LOCKB_DEVICE           = 16; // The supplied path is a device relative specification.
  LOCKB_VOLUME           = 17; // The supplied path is a volume relative specification.
  LOCKB_ASSIGNMENT       = 18; // The supplied path is an assignment relative specification.
  LOCKB_PROGDIR          = 19; // The supplied path is a PROGDIR: relative specification.
  LOCKB_CURRDIR          = 20; // The supplied path is a CURRDIR: relative specification.
  LOCKB_ROOTDIR          = 21; // The supplied path is a root directory relative specification.
  LOCKB_FILESYS          = 24; // The supplied path returned the root of the default filesystem.

// Field definitions of the above.
  LOCKF_SOFTLINK_MIDPART = 1 shl LOCKB_SOFTLINK_MIDPART;
  LOCKF_SOFTLINK_ENDPART = 1 shl LOCKB_SOFTLINK_ENDPART;
  LOCKF_DEVICE           = 1 shl LOCKB_DEVICE;
  LOCKF_VOLUME           = 1 shl LOCKB_VOLUME;
  LOCKF_ASSIGNMENT       = 1 shl LOCKB_ASSIGNMENT;
  LOCKF_PROGDIR          = 1 shl LOCKB_PROGDIR;
  LOCKF_CURRDIR          = 1 shl LOCKB_CURRDIR;
  LOCKF_ROOTDIR          = 1 shl LOCKB_ROOTDIR;
  LOCKF_FILESYS          = 1 shl LOCKB_FILESYS;

// *********************************************************************
// Result flags for WaitForData().
  WFDB_READ  = 0; // Stream has data ready to be read
  WFDB_WRITE = 1; // Stream is ready to accept more data to be written.

  WFDF_READ  = 1 shl WFDB_READ;
  WFDF_WRITE = 1 shl WFDB_WRITE;

// *********************************************************************
// NotifyProcListChange(), NotifyDosListChange(), termination signal value.
  NPLC_END = $FFFFFFFF;
  NDLC_END = $FFFFFFFF;

// *********************************************************************
// SetBlockingMode() mode values which define whether read/write access to a
// stream will block until a request is satisfied or will return immediately.
  SBM_BLOCKING     = 1;
  SBM_NON_BLOCKING = 2;

// *********************************************************************
// GetPID() function, this flag determins which PID is returned. 53.58
  GPID_PROCESS = 1;
  GPID_PARENT  = 2;

// *********************************************************************
// Length constants specifically for new vector-port based filesystems.
// Older packet filesystem designs may not support these lengths,
// so use FileSystemAttr() to find out filesystem specific values.
  MAX_VP_FILENAME   = 255;
  MAX_VP_COMMENT    = 255;
  MAX_VP_SOFTLINK_TARGET = 4000;

type
  FileHandle = BPTR;
  FileLock   = BPTR;

// *********************************************************************
  PDateStamp = ^TDateStamp;
  TDateStamp = record
    ds_Days: LongInt;   // Number of days since Jan. 1, 1978
    ds_Minute: LongInt; // Number of minutes past midnight
    ds_Tick: LongInt;   // Number of ticks past minute
  end;

const
  TICKS_PER_SECOND = 50; // Fifty ticks = one second One tick = twenty milliseconds

// *********************************************************************
// V52 Struct ExamineData, returned by ExamineObject() and ExamineDir().
// This structure supports unlimited string lengths, 64 bit file sizes,
// automatic resolution of link targets and extended auxilliary information.
//
// NOTE; Applications DO NOT allocate these, the filesystem does it.
//       Also, the filesystem itself must always allocate these using
//       AllocDosObject(), otherwise they will simply not work.
type
  // (RO) = Read Only, these fields must not be modified by the application.
  PExamineData = ^TExamineData;
  TExamineData = record
    EXDnode: TMinNode;     // MinList node.
    EXDinfo: LongWord;     // General purpose user data.

    // These are private members exclusively for the FS & DOS.
    FSPrivate: LongWord;   // FILESYSTEM ONLY - Private usage.
    Reserved0: LongWord;   // DOS RESERVED - Private usage.
    DOSPrivate: APTR;      // DOSLIB ONLY - Private usage.

    // The following are public fields.
    StructSize: LongWord;  // (RO) Full size of the structure.
    Type_: LongWord;       // Entry type; use EXD_IS_ macros below.
    FileSize: Int64;       // Size of a file, otherwise set to -1.
    Date: TDateStamp;      // Days, Mins, Ticks.
    RefCount: LongWord;    // Object hardlink references.    [Note 1]
    ObjectID: QWord;       // Unique object identifier.      [Note 2]
    Name: STRPTR;          // (RO) Nul-terminated object name.
    NameSize: LongWord;    // (RO) Allocated size of the name buffer.
    Comment: STRPTR;       // (RO) Nul-terminated comment string.
    CommentSize: LongWord; // (RO) Allocated size of comment buffer.
    Link: STRPTR;          // (RO) Nul-terminated target name string.
    LinkSize: LongWord;    // (RO) Allocated size of target buffer.
    Protection: LongWord;  // Protection status bitmask.
    OwnerUID: LongWord;    // Owner info. [Note 3]
    OwnerGID: LongWord;    // Group info. [Note 3]
    Reserved2: LongWord;   // DOS reserved field.
    Reserved3: LongWord;   // DOS reserved field.
    Reserved4: LongWord;   // DOS reserved field.
  end;
{
 [Note 1] If possible, this field should be supported by filesystems that
          support hardlinks, it is used to indicate the hardlink reference
          count for objects.  A value of one indicates that the object has
          no additional references other than itself, a value of two will
          indicate that it has one additional hardlink reference,
          a value of three means it has two hardlink references, etc...
          All hardlinks themselves always show a value of 1 in this field
          when supported, otherwise this field MUST be set to zero for
          all objects, if the filesystem does not support it.
          It will be undefined if the caller fails to specify the
          EXF_REFCOUNT bit in the EX_DataFields mask.

 [Note 2] This field is the mandatory unique object identfier for all
          objects on this volume, the value used here may be any arbitrary
          value as long as it is unique from all other objects on this
          volume, it may use the object node address, the disk header block
          number or other such values. (Note: this field is 64 bits wide.)
          This field will be undefined if you fail to specify the
          EXF_OBJECTID bit in the EX_DataFields mask.

 [Note 3] Historically, these fields only used the low 16 bits.
          For extensibility, the size of these are now 32 bit fields.
          However, for interoperability, it is recommended that values
          greater than 65535 be avoided where possible so reading only
          the low 16 bits results in the same value for legacy applications.
          These fields will be undefined if you fail to specify the
          EXF_OWNER bit in the EX_DataFields mask.
          (See extended information relating to the UID/GID fields below.)}

// *********************************************************************
// Bit mask field values that are passed in the EX_DataFields tag,
// this determines what data is returned from the ExamineDir() function.
//
// The data value returned in the respective fields will be undefined if
// the mask bit is not specified, NEVER assume otherwise.
const
  EXF_DATE       = 1 shl 0;
  EXF_REFCOUNT   = 1 shl 1;
  EXF_NAME       = 1 shl 2;
  EXF_COMMENT    = 1 shl 3;
  EXF_LINK       = 1 shl 4;
  EXF_PROTECTION = 1 shl 5;
  EXF_OWNER      = 1 shl 6;  // both Group and User ID
  EXF_TYPE       = 1 shl 7;  // MUST be specified for EXD_IS macros
  EXF_SIZE       = 1 shl 8;
  EXF_OBJECTID   = 1 shl 9;

  EXF_spare1     = 1 shl 10; // reserved for future EXF_ALL feature
  EXF_spare2     = 1 shl 11; // reserved for future EXF_ALL feature
  EXF_spare3     = 1 shl 12; // reserved for future EXF_ALL feature
  EXF_spare4     = 1 shl 13; // reserved for future EXF_ALL feature
  EXF_spare5     = 1 shl 14; // reserved for future EXF_ALL feature
  EXF_spare6     = 1 shl 15; // reserved for future EXF_ALL feature

  EXF_ALL        = $FFFF;    // Return all of the information above

// *********************************************************************
// Values for ExamineData->Type for checking what kind of object it describes.
// The lower 8 bits form the unique descriptor value, the upper bits are used
// as a bitmask. The FSO_TYPE_MASK is used to filter the descriptor value.
// NOTE: You MUST specify EXF_TYPE in the EX_DataFields tag for these macros
//       to work otherwise random values will be returned in the 'Type' field.

  FSO_TYPE_MASK      = $FF; // mask for object type descriptors

  FSO_TYPE_SOFTLINK  = 0;   // Object is a softlink
  FSO_TYPE_FILE      = 1;   // Object is a file
  FSO_TYPE_DIRECTORY = 2;   // Object is a directory
  FSO_TYPE_PIPE      = 3;   // Object is a pipe stream
  FSO_TYPE_SOCKET    = 4;   // Object is a socket stream

  FSO_TYPE_INVALID   = $FE; // Object is unidentifiable or corrupt, you SHOULD never actually see this,
                            // but the value is reserved here for use by the handler AND DOS to indicate internal errors.
                            // You MAY also possibly see this value if one were to read the 'Type' member without
                            // setting the EXF_TYPE bit in EX_DataFields.
  FSOF_LINK    = 1 shl 8; // Bit set for FFS style hard & soft links
  FSOF_ALTLINK = 1 shl 9; // Reserved bit for alternate style links, these are currently unimplemented.

// Information relating to the ExamineData; OwnerUID and OwnerGID members.
  DOS_OWNER_ROOT = 65535;
  DOS_OWNER_NONE = 0;

// *********************************************************************
// Definitions for the ExamineData "Protection" bitfield.
  EXDB_OTR_READ        = 15; // Other: file is readable
  EXDB_OTR_WRITE       = 14; // Other: file is writable
  EXDB_OTR_EXECUTE     = 13; // Other: file is executable
  EXDB_OTR_DELETE      = 12; // Other: file is deleted
  EXDB_GRP_READ        = 11; // Group: file is readable
  EXDB_GRP_WRITE       = 10; // Group: file is writable
  EXDB_GRP_EXECUTE     =  9; // Group: file is executable
  EXDB_GRP_DELETE      =  8; // Group: file is deleteable

  EXDB_HOLD             = 7; // (H) hold loaded program in cli resident list (v50)
  EXDB_SCRIPT           = 6; // (S) program is a script (execute) file
  EXDB_PURE             = 5; // (P) program is reentrant and reexecutable
  EXDB_ARCHIVE          = 4; // (A) cleared whenever file is changed
  EXDB_NO_READ          = 3; // (R) NOT readable,
  EXDB_NO_WRITE         = 2; // (W) NOT writable,
  EXDB_NO_EXECUTE       = 1; // (E) NOT executable, Used by Shell only
  EXDB_NO_DELETE        = 0; // (D) NOT deletable,

// Masks for above bits
  EXDF_OTR_READ    = 1 shl EXDB_OTR_READ;
  EXDF_OTR_WRITE   = 1 shl EXDB_OTR_WRITE;
  EXDF_OTR_EXECUTE = 1 shl EXDB_OTR_EXECUTE;
  EXDF_OTR_DELETE  = 1 shl EXDB_OTR_DELETE;
  EXDF_GRP_READ    = 1 shl EXDB_GRP_READ;
  EXDF_GRP_WRITE   = 1 shl EXDB_GRP_WRITE;
  EXDF_GRP_EXECUTE = 1 shl EXDB_GRP_EXECUTE;
  EXDF_GRP_DELETE  = 1 shl EXDB_GRP_DELETE;

  EXDF_HOLD        = 1 shl EXDB_HOLD;
  EXDF_SCRIPT      = 1 shl EXDB_SCRIPT;
  EXDF_PURE        = 1 shl EXDB_PURE;
  EXDF_ARCHIVE     = 1 shl EXDB_ARCHIVE;
  EXDF_NO_READ     = 1 shl EXDB_NO_READ;
  EXDF_NO_WRITE    = 1 shl EXDB_NO_WRITE;
  EXDF_NO_EXECUTE  = 1 shl EXDB_NO_EXECUTE;
  EXDF_NO_DELETE   = 1 shl EXDB_NO_DELETE;

  EXDF_DEFAULT = 0;

{$PACKRECORDS 4}
type
// Returned by Examine() and ExInfo(), must be on a 4 byte boundary
// This structure is obsolete, due to 4 gig file size limits, short string
// length issues and lack of extensibility.
// Software should migrate to using the new ExamineDir() and the ExamineObject()
// functions which use the struct ExamineData.
//
// This structure is here because it is nested within other structures and for
// legacy compatibility reasons, so this definition can't be removed at this time.
//
// NOTE: The fib_DOSReserved[8] area is ABSOLUTELY DOS PRIVATE !! - Do not access it.
//       DOS uses all of this space for context information when emulating the
//       old V40 functions that require this structure.

  PFileInfoBlock = ^TFileInfoBlock;
  TFileInfoBlock = record
    fib_DiskKey: LongWord;     // -- FILESYSTEM PRIVATE !!
    fib_DirEntryType: LongInt; // Use FIB_IS_ macros to identify object.
    fib_FileName: array[0..107] of Char; // Null terminated.
    fib_Protection: LongWord;  // Bit mask of protection, rwxd are 3-0.
    fib_Obsolete: LongInt;     // obsolete use fib_DirEntryType instead
    fib_Size: LongWord;        // Byte size of file, only good to 4 gig.
    fib_NumBlocks: LongWord;   // Number of blocks in file
    fib_Date: TDateStamp;      // Date file last changed
    fib_Comment: array[0..79] of Char; // Null terminated comment string.
    // Note: the following two fields are not supported by all filesystems.
    //   Should be 0 if not supported
    fib_OwnerUID: Word;        //
    fib_OwnerGID: Word;        //
    fib_Reserved: array [0..7] of Pointer; // DOS Private do not access this
  end; // FileInfoBlock - 260 bytes

const

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

// Standard maximum length for an error string from fault.  However, most
// error strings should be kept under 60 characters if possible.  Don't
// forget space for the header you pass in. *}
  FAULT_MAX = 82;

type
// For the old Info() function, the structure must be on a 4 byte boundary.
// The new V51+ GetDiskInfo() doesn't require special structure alignment.
// Just use AllocDosObject(DOS_INFO,0); to get the right one every time.
  PInfoData = ^TInfoData;
  TInfoData = record
    id_NumSoftErrors: LongInt;  // number of soft errors on disk
    id_UnitNumber: LongInt;     // Which unit disk is (was) mounted on
    id_DiskState: LongInt;      // See defines below
    id_NumBlocks: LongWord;     // Number of blocks on disk
    id_NumBlocksUsed: LongWord; // Number of block in use
    id_BytesPerBlock: LongWord;
    id_DiskType: LongInt;       // Disk Type code
    id_VolumeNode: BPTR;        // BCPL pointer to volume node
    id_InUse: LongInt;          // Flag, zero if not in use
  end;
{$PACKRECORDS 2}

const
// InfoData (ID stands for InfoData)
// Disk states }
  ID_WRITE_PROTECTED  = 80; // Disk is write protected
  ID_VALIDATING       = 81; // Disk is currently being validated
  ID_VALIDATED        = 82; // Disk is consistent and writeable
// Disk types for; id_DiskType
  ID_NO_DISK_PRESENT     = -1;
  ID_UNREADABLE_DISK     = $42414400; // 'BAD\0'
  ID_BUSY_DISK           = $42555359; // 'BUSY'
  ID_SWAP_DISK           = $53574150; // 'SWAP'
  ID_NOT_REALLY_DOS      = $4E444F53; // 'NDOS'
  ID_KICKSTART_DISK      = $4B49434B; // 'KICK'
  ID_CDFS_DISK           = ((Ord('C') shl 24) or (Ord('D') shl 16) or (Ord('0') shl 8) or $01); // CDFileSystem 'CD01'

  ID_DOS_DISK            = $444F5300; // 'DOS\0'
  ID_FFS_DISK            = $444F5301; // 'DOS\1'
  ID_INTER_DOS_DISK      = $444F5302; // 'DOS\2'
  ID_INTER_FFS_DISK      = $444F5303; // 'DOS\3'
  ID_FASTDIR_DOS_DISK    = $444F5304; // 'DOS\4'
  ID_FASTDIR_FFS_DISK    = $444F5305; // 'DOS\5'
  ID_LONGNAME_DOS_DISK   = $444F5306; // 'DOS\6'
  ID_LONGNAME_FFS_DISK   = $444F5307; // 'DOS\7'

// V53 disk type used for new vector-port based filesystems like RAM: ENV: APPDIR:
  ID_VP255_DOS_DISK     = $444F5309;  // 'DOS\9'
  ID_NGFS_DISK          = $4E474653;  //  NGFS

// Special 'disk type' signatures returned by con-handler
  ID_CON                = $434F4E00;  // 'CON\0'
  ID_RAWCON             = $52415700;  // 'RAW\0'

// Other known disk types
  ID_MSDOS_DISK    = (Ord('M') shl 24) or (Ord('S') shl 16) or (Ord('D') shl 8) or $00;
  ID_MSDOS_DISK_HD = (Ord('M') shl 24) or (Ord('S') shl 16) or (Ord('H') shl 8) or $00;
  ID_MSDOS_DISK_DS = (Ord('M') shl 24) or (Ord('D') shl 16) or (Ord('D') shl 8) or $00;

// FAT disk types
  ID_FAT00_DISK = (Ord('F') shl 24) or (Ord('A') shl 16) or (Ord('T') shl 8) or $00;
  ID_FAT01_DISK = (Ord('F') shl 24) or (Ord('A') shl 16) or (Ord('T') shl 8) or $01;
  ID_FAT32_DISK = (Ord('F') shl 24) or (Ord('A') shl 16) or (Ord('T') shl 8) or $32;

// FUSE filesystems
  ID_BOX0_DISK        = $424F5800; //  BOX\0 - BoxFileSystem
  ID_EXFAT_DISK       = $46415458; //  FATX  - exFATFileSystem
  ID_EXT2_DISK        = $45585402; //  EXT\2 - FuseExt2FileSystem
  ID_HFS_DISK         = $48465300; //  HFS\0 - FuseHFS
  ID_NTFS_DISK        = $4e544653; //  NTFS  - NTFileSystem3G




// Errors from IoErr(), etc.
  ERROR_INVALID_PACKET_MESSAGE     = 101;
  ERROR_WRONG_PACKET_RETURNED      = 102;
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
// added for 1.4
  ERROR_IS_SOFT_LINK               = 233;
  ERROR_OBJECT_LINKED              = 234;
  ERROR_BAD_HUNK                   = 235;
  ERROR_NOT_IMPLEMENTED            = 236;
  ERROR_RECORD_NOT_LOCKED          = 240;
  ERROR_LOCK_COLLISION             = 241;
  ERROR_LOCK_TIMEOUT               = 242;
  ERROR_UNLOCK_ERROR               = 243;
//
  ERROR_BUFFER_OVERFLOW            = 303; // User or internal buffer overflow
  ERROR_BREAK                      = 304; // A break character was received
  ERROR_NOT_EXECUTABLE             = 305; // A file has E bit cleared
  ERROR_IS_PIPE                    = 306; // Operation is not permitted on pipes
  ERROR_BROKEN_PIPE                = 307; // No more data can be read from or written to this pipe
  ERROR_WOULD_BLOCK                = 308; // Operation cannot complete immediately, as requested
  ERROR_BAD_SIGNAL_BIT             = 309; // An invalid signal bit number was specified
//
  ERROR_UNSUPPORTED_HARDWARE       = 350; // The hardware found is unsupported by software
// Special error codes that can be set by MountDevice().
  ERROR_INVALID_DEVICE_TYPE        = 401; // Device to be mounted is neither handler nor file system.
  ERROR_INVALID_DEVICE_NAME        = 402; // Name of device to be mounted does contain more colon characters than necessary, or in the wrong place
  ERROR_DEVICE_NAME_TOO_LONG       = 403; // Name of device to be mounted is longer than 255 characters
  ERROR_INVALID_SECTOR_SIZE        = 404; // Sector size is not a multiple of four
  ERROR_CONTROL_STRING_TOO_LONG    = 405; // Control string for device to be mounted is longer than 255 characters
  ERROR_EXEC_DEVICE_NAME_TOO_LONG  = 406; // Exec device driver name for device to be mounted is longer than 255 characters
  ERROR_HANDLER_STRING_TOO_LONG    = 407; // Name of handler to be mounted is longer than 255 characters
  ERROR_STARTUP_STRING_TOO_LONG    = 408; // Startup string for device to be mounted is longer than 255 characters
  ERROR_SIZE_MISSING               = 409; // Number of surfaces, sector size, number of sectors per block or number of sectors per track is zero
  ERROR_DEVICE_NAME_MISSING        = 410; // No exec device driver name was given
  ERROR_INVALID_NUMBER_OF_CYLINDERS = 411; // Number of cylinders to use is negative
  ERROR_HANDLER_CANNOT_BE_STARTED   = 412; // No handler name, segment list or port address was given
  ERROR_INVALID_GLOBAL_VECTOR       = 413; // Global vector must be one of -2 or -1
  ERROR_INVALID_TASK_PRIORITY       = 414; // The task priority is not in the range -128..127

// *********************************************************************
// These are the return codes used by convention by AmigaDOS commands.
// See FAILAT and IF commands for relevance to EXECUTE files.
// See RunCommand() and CreateNewProc() for relevance to all DOS processes.
  RETURN_OK    = 0;  // No problems, success
  RETURN_WARN  = 5;  // A warning only
  RETURN_ERROR = 10; // Something wrong
  RETURN_FAIL  = 20; // Complete or severe failure

// Bit numbers that signal that a user has issued a break
  SIGBREAKB_CTRL_C = 12;
  SIGBREAKB_CTRL_D = 13;
  SIGBREAKB_CTRL_E = 14;
  SIGBREAKB_CTRL_F = 15;
// Bit fields that signal that a user has issued a break, See CheckSignal() for examples.
  SIGBREAKF_CTRL_C = 1 shl SIGBREAKB_CTRL_C;
  SIGBREAKF_CTRL_D = 1 shl SIGBREAKB_CTRL_D;
  SIGBREAKF_CTRL_E = 1 shl SIGBREAKB_CTRL_E;
  SIGBREAKF_CTRL_F = 1 shl SIGBREAKB_CTRL_F;
// Values returned by SameLock()
  LOCK_DIFFERENT    =  -1;
  LOCK_SAME         =  0;
  LOCK_SAME_HANDLER =  1; // locks are on same volume
// types for ChangeMode()
  CHANGE_LOCK = 0;
  CHANGE_FH   = 1;
// Mode parameter for DevNameFromLock() and DevNameFromFH()
  DN_DEVICEONLY = 0;
  DN_FULLPATH   = 1;
  DN_ROOTPATH   = 2;
// Mode flags for GetProcSegList() (v51.88)
  GPSLF_CLI = 1 shl 1;
  GPSLF_SEG = 1 shl 2;
  GPSLF_ENT = 1 shl 3;
  GPSLF_RUN = 1 shl 4;
// Values for MakeLink() - DO NOT use random values, new link types will be added in the future that require other values to be used.
  LINK_HARD = 0;
  LINK_SOFT = 1;
// Values returned by ReadItem (and ReadLineItem V50)
  ITEM_EQUAL    = -2; // "=" Symbol
  ITEM_ERROR    = -1; // error
  ITEM_NOTHING  = 0;  // *N, ;, endstreamch
  ITEM_UNQUOTED = 1;  // unquoted item
  ITEM_QUOTED   = 2;  // quoted item
// Types for AllocDosObject/FreeDosObject
  DOS_FILEHANDLE        = 0;  // Very few people should use this
  DOS_EXALLCONTROL      = 1;  // Must be used to allocate this! - Obsolete struct
  DOS_FIB               = 2;  // may be useful - not really, it's obsolete now
  DOS_STDPKT            = 3;  // for doing packet-level I/O
  DOS_CLI               = 4;  // for shell-writers, etc
  DOS_RDARGS            = 5;  // for ReadArgs if you pass it in
  DOS_ANCHORPATH        = 6;  // for MatchFirst/MatchNext (V50)
  DOS_INFODATA          = 7;  // for Info() aligned InfoData (V50)
  DOS_NOTIFYREQUEST     = 8;  // for Notify request functions (V51)
  DOS_LOCK              = 9;  // for filesystem lock allocations (V51)
  DOS_FREADLINEDATA     = 10; // for FReadLine() structure. (V51)
  DOS_EXAMINEDATA       = 11; // for filesystem ExamineData allocations.(V51)
  DOS_DOSLIST           = 12; // for struct DosList allocations. (V52.16)
  DOS_VOLUMELIST        = 13; // for struct list with attached nodes. (V53.52)
  DOS_FSVECTORPORT      = 14; // for filesystem vector port creation. (53.80)

type
// The following is used by the new V51 tag;  NP_NotifyOnDeathMessage
// See CreateNewProc() autodoc for more details.
  PDeathMessage = ^TDeathMessage;
  TDeathMessage = record
    dm_Msg: PMessage;       // Embedded exec message structure.
    dm_ReturnCode: LongInt; // Primary process return code, set by DOS
    dm_Result2: LongInt;    // The value returned from IoErr(),set by DOS
  end;

// The following is the data structure to be used for FReadLine()
// which is allocated by the AllocDosObject() function ONLY.
  PFReadLineData = ^TFReadLineData;
  TFReadLineData = record
    frld_Line: STRPTR;                        // NUL-terminated string
    frld_LineLength: LongWord;                // what strlen(frld_Line) would return
    frld_DosPrivate: array[0..7] of LongWord; // -- internal dos.library use only
  end;

// For use by the V1.4+ DOS functions StrtoDate() and DatetoStr()
type
  _PDateTime = ^_TDateTime;
  _TDateTime = record
    dat_Stamp: TDateStamp; // DOS DateStamp
    dat_Format: Byte;      // controls appearance of dat_StrDate
    dat_Flags: Byte;       // see BITDEF's below
    dat_StrDay: STRPTR;    // day of the week string
    dat_StrDate: STRPTR;   // date string
    dat_StrTime: STRPTR;   // time string
  end;

// You need this much room for each of the DateTime strings:
CONST
  LEN_DATSTRING = 16;

// Flags for dat_Flags
   DTB_SUBST  = 0; // substitute Today, Tomorrow, etc.
   DTB_FUTURE = 1; // day of the week is in future
   DTF_SUBST  = 1 shl DTB_SUBST;
   DTF_FUTURE = 1 shl DTB_FUTURE;


// date format values
  FORMAT_DOS = 0; // dd-mmm-yy
  FORMAT_INT = 1; // yy-mm-dd
  FORMAT_USA = 2; // mm-dd-yy
  FORMAT_CDN = 3; // dd-mm-yy
  FORMAT_DEF = 4; // use default format, as defined by locale; if locale not
                      // available, use FORMAT_DOS instead
  FORMAT_ISO = 5; // yyyy-mm-dd (ISO 8601) Requires locale V48 or dos V50.36 if locale not available
  FORMAT_MAX = FORMAT_ISO;

// *********************************************************************
// This structure is mostly DOS private for MatchXXX() directory scanner.
Type
  PAChain = ^TAChain;
  TAChain = record
    an_Child: PAChain;
    an_Parent: PAChain;
    an_Lock: BPTR;
    // The remaining members are strictly DOS private.
    an_Info: TFileInfoBlock;
    an_Flags: Shortint;
    an_ExData: PExamineData;
    an_DevProc: Pointer; // real type PDevProc;
    an_String: Array[0..0] of Char;
  end;

// Obsolete definition ==ONLY== for legacy reference, pre V50.76 DOS.
//  This is what DOS will expect when NOT allocated by AllocDosObject().

  PAnchorPathOld = ^TAnchorPathOld;
  TAnchorPathOld = record
    case smallint of
    0 : (
      ap_First: PAChain;
      ap_Last: PAChain;
     );
    1 : (
      ap_Base: PAChain;            // pointer to first anchor
      ap_Current: PAChain;         // pointer to last anchor
      ap_BreakBits,                // Bits we want to break on
      ap_FoundBreak: LongWord;     // Bits we broke on. Also returns ERROR_BREAK
      ap_Flags: Shortint;          // New use for extra word.
      ap_Reserved: Shortint;
      ap_Strlen: Word;             // This is what ap_Length used to be
      ap_Info: TFileInfoBlock;
      ap_Buf: array[0..0] of Char; // Buffer for path name, allocated by user !!
     );
  end;

// ************************ PATTERN MATCHING ***************************
{
  Structure expected by MatchFirst, MatchNext.
  Allocate this structure ONLY with AllocDosObject() from DOS 50.76+
  and initialize the ADO_Flags with the appropriate bits as follows:

  Set ADO_Mask, (ap_BreakBits) to the signal bitmask (^CDEF) that you want to
  take a break on, or 0L, (default) if you don't want to convenience the user.

  If you want to have the FULL PATH NAME of the files you found,
  allocate an additional buffer space using the tag ADO_Strlen, this will
  place the buffer in ap_Buffer and the size into ap_Strlen.

  If you don't need the full path name, DO NOT specify the ADO_Strlen tag,
  this will by default, set ap_Strlen to zero, for no additional buffer space.
  In this case, the name of the file, and other stats are available in the
  ap_ExData structure if not NULL, (or old FIB ap_Info struct for legacy apps),
  Note that the ap_ExData pointer was NULL prior to V54, so you MUST check the
  pointer before access, when operating with previous dos.library releases.
  Always use ap_ExData in preference to the old ap_Info data, because only
  ap_ExData supports 64 bit file sizes and long names > 107 bytes.

  Then call MatchFirst() and then afterwards, MatchNext() with this structure.
  You should check the return value each time (see below) and take the
  appropriate action, ultimately calling MatchEnd() when there are
  no more files and you are done.  You can tell when you are done by
  checking for the normal AmigaDOS return code ERROR_NO_MORE_ENTRIES.

************************************************************************
WARNING:  You MUST allocate these with AllocDosObject() from DOS 50.76+
        MatchXXX() will simply not work if you do not heed this warning.
************************************************************************}

  PAnchorPath = ^TAnchorPath;
  TAnchorPath = record       // redefined for dos 50.76+ (2004)
    ap_Magic: LongInt;       // -- PRIVATE - DOS compatibility
    case smallint of
    0 : (
      ap_First: PAChain;
      ap_Last: PAChain;
     );
    1 : (
      ap_Base: PAChain;        // Ptr to first anchor
      ap_Current: PAChain;     // Ptr to current anchor
      ap_BreakBits: LongWord;  // Bits we want to break on
      ap_FoundBreak: LongWord; // Bits we broke on.
      ap_Flags: LongWord;      // The flags bitfield.
      ap_ExData: PExamineData; // Ptr to ExamineData (or nil)
      ap_CTXPrivate: APTR;     // -- PRIVATE, DOS use only.
      ap_Reserved: array[0..1] of LongWord;  // Future use, currently 0
      ap_Strlen: LongWord;     // Strlen : Size of the buffer -1
      ap_Buffer: STRPTR;       // Full name, (see ADO_Strlen)
      ap_Info: PFileInfoBlock; // The old FileInfoBlock space
      ap_Private1: LongWord;   // --PRIVATE, DOS use only.
      ap_Private2: LongWord;   // --PRIVATE, DOS use only.
    );
  end;

// Flags for AnchorPath->ap_Flags.

const
  APB_DOWILD       = 0; // Unused
  APB_ITSWILD      = 1; // Set by MatchFirst, used by MatchNext. Application can test APB_ITSWILD, too
                        // (means that there's a wildcard in the pattern after calling MatchFirst).
  APB_DODIR        = 2; // Bit is SET if a DIR node should be entered. Application can RESET this
                        // bit after MatchFirst/MatchNext to AVOID entering a dir.
  APB_DIDDIR       = 3; // Bit is SET for an "expired" dir node.
  APB_NOMEMERR     = 4; // Set on memory error
  APB_DODOT        = 5; // Unused
  APB_DirChanged   = 6; // ap_Current->an_Lock changed since last MatchNext call
  APB_FollowHLinks = 7; // follow hardlinks on DODIR - defaults to not following hardlinks on a DODIR.
  APB_MultiAssigns = 8; // Set this bit via AllocDosObject() to allow Multi-Assign scanning to be enabled.
                        // ( NOTE: ONLY AVAILABLE FROM DOS 50.76+ )
  APF_DOWILD       = 1 shl APB_DOWILD;
  APF_ITSWILD      = 1 shl APB_ITSWILD;
  APF_DODIR        = 1 shl APB_DODIR;
  APF_DIDDIR       = 1 shl APB_DIDDIR;
  APF_NOMEMERR     = 1 shl APB_NOMEMERR;
  APF_DODOT        = 1 shl APB_DODOT;
  APF_DirChanged   = 1 shl APB_DirChanged;
  APF_FollowHLinks = 1 shl APB_FollowHLinks;
  APF_MultiAssigns = 1 shl APB_MultiAssigns; // New for V50, See AllocDosObject()

// Flags for AChain->an_Flags; these are private to DOS!


  DDB_PatternBit  = 0;
  DDB_ExaminedBit = 1;
  DDB_Completed   = 2;
  DDB_AllBit      = 3;
  DDB_Assign      = 4;
  DDB_Device      = 5;

  DDF_Device      = 1 shl DDB_Device;
  DDF_Assign      = 1 shl DDB_Assign;
  DDF_AllBit      = 1 shl DDB_AllBit;
  DDF_Completed   = 1 shl DDB_Completed;
  DDF_ExaminedBit = 1 shl DDB_ExaminedBit;
  DDF_PatternBit  = 1 shl DDB_PatternBit;

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


type
// All DOS processes have this structure
  PProcess = ^TProcess;
  TProcess = record
    // original definitions from version 1.0 and earlier betas (v30 - 1984)
    pr_Task: TTask;         // The exec task structure for this process
    pr_MsgPort: TMsgPort;   // This process' message port structure
    pr_Size: Word;          // Size of struct from V51.08+, previously 0
    pr_SegArray: BPTR;      // -- PRIVATE - BPTR to internal process segarray[].
    pr_StackSize: LongWord; // The running process stack size, in bytes
    pr_gv: LongInt;         // -- OBSOLETE BCPL value - do not access this.
    pr_CliNum: LongInt;     // CLI number, for DOS allocated cli processes.
    pr_sb: BPTR;            // -- OBSOLETE BCPL value - do not access this.
    pr_Result2: LongInt;    // Secondary result from last call, for IoErr()
    pr_CurrentDir: BPTR;    // Lock associated with current directory
    pr_CIS: BPTR;           // Current CLI Input Stream
    pr_COS: BPTR;           // Current CLI Output Stream
    pr_ConsoleTask: PMsgPort;     // Console handler process for current window
    pr_FileSystemTask : PMsgPort; // File handler process for current drive
    pr_CLI: BPTR;           // BCPL Pointer to struct CommandLineInterface
    pr_ra: APTR;            // -- OBSOLETE - do not access this.
    pr_PktWait: APTR;       // Function to be called when awaiting packet
    pr_WindowPtr: APTR;     // Window for posting error requesters

    // following definitions are new with 2.0 (v36 - 1990)
    pr_ProgramDir: BPTR;       // Program's home directory lock
    pr_Flags: LongWord;        // Flags telling dos about this process
    pr_ExitCode: APTR;         // Code to call on exit of process or nil
    pr_ExitData: LongInt;      // Passed as an argument to pr_ExitCode.
    pr_Arguments: STRPTR;      // Arguments passed to the process at start
    pr_LocalVars: TMinList;    // Local environment variables
    pr_ShellPrivate: LongWord; // -- PRIVATE - Internal shell use only.
    pr_CES: BPTR;              // Error stream - if nil, use pr_COS

    // following definitions are new from 4.0 (v50 - Apr,2004)
    pr_PrData: APTR;          // -- PRIVATE - internal dos access ONLY.
    pr_CurrentSeg: BPTR;      // Seglist of current running code.(READ ONLY)
    pr_EmulPrivate: LongWord; // -- PRIVATE - Flags for 68k JIT Emulator.
    pr_68kPrivate: LongWord;  // -- PRIVATE - 68K control opts, OS access only
    pr_ParentID: LongWord;    // ID# of parent process, 0 if a task. (READ ONLY)
    pr_ProcessID: LongWord;   // ID# for this process. (READ ONLY)
    pr_Reserved4: LongWord;   // -- reserved for dos expansion - leave alone
    pr_OGLContextData: APTR;  // -- PRIVATE - ogl access only.
    pr_Reserved5: APTR;       // -- reserved for dos expansion - leave alone.
    pr_CLibData: APTR;        // -- PRIVATE - clib/newlib use, copied by CNP()
    pr_Reserved6: APTR;       // -- reserved for dos expansion - leave alone

    // following definitions are new from (v51 - Jan,2006)
    pr_RestoreList: TMinList;  // -- PRIVATE - used by DOS for process cleanup()
    pr_DeathSigTask: APTR;     // Process/task to signal when the process ends
    pr_DeathSigBit: LongWord;  // Signal bit number for pr_DeathSigTask
    pr_DeathMessage: APTR;     // DeathMessage to ReplyMsg() to when process ends
    pr_EntryCode: APTR;        // Code to call on startup of process, or nil
    pr_EntryData: LongInt;     // Passed as an argument to pr_EntryCode func.
    pr_FinalCode: APTR;        // Code to call on exit of the process, or nil
    pr_FinalData: LongInt;     // Passed as an argument to pr_FinalCode func.
    pr_DLNotifyData: LongWord; // -- PRIVATE - dos NotifyDosListChange() data.
    pr_PLNotifyData: LongWord; // -- PRIVATE - dos NotifyProcListChange() data.

    // following definitions are new from (v52 - Dec,2006)
    pr_UID: LongWord;      // -- PRIVATE - dos Set/GetOwnerInfo() use only.
    pr_GID: LongWord;      // -- PRIVATE - dos Set/GetOwnerInfo() use only.
    pr_Reserved3: LongInt; // -- reserved for dos expansion - leave alone
    pr_Reserved2: LongInt; // -- reserved for dos expansion - leave alone
    pr_Reserved1: LongInt; // -- reserved for dos expansion - leave alone
    pr_Reserved0: LongInt; // -- reserved for dos expansion - leave alone
  end; // Process - 344 bytes

const
// Flags for Process->pr_Flags for all DOS processes.
  PRB_FREESEGLIST       = 0;  // NP_FreeSegList,TRUE sets this bit.
  PRB_FREECURRDIR       = 1;  // NP_CurrentDir will be UnLocked if set
  PRB_FREECLI           = 2;  // NP_Cli will be freed if this is set.
  PRB_CLOSEINPUT        = 3;  // NP_CloseInput,TRUE sets this bit.
  PRB_CLOSEOUTPUT       = 4;  // NP_CloseOutput,TRUE sets this bit.
  PRB_FREEARGS_OBSOLETE = 5;  // (V50) obsolete, args copy now on task memlist.
  PRB_CLOSEERROR        = 6;  // NP_CloseError,TRUE sets this bit. (V51)
  PRB_LOCKSTACK         = 7;  // NP_LockStack,TRUE sets this bit. (V52.10)
  PRB_spare08           = 8;
  PRB_spare09           = 9;
  PRB_spare10           = 10;
  PRB_spare11           = 11;
  PRB_spare12           = 12;
  PRB_spare13           = 13;
  PRB_spare14           = 14;
  PRB_spare15           = 15;

  PRB_spare16           = 16;
  PRB_spare17           = 17;
  PRB_spare18           = 18;
  PRB_spare19           = 19;
  PRB_spare20           = 20;
  PRB_spare21           = 21;
  PRB_spare22           = 22;
  PRB_spare23           = 23;
  PRB_CHILDPROCESS      = 24; // (V50) NP_Child,TRUE sets this for a dependant child.
  PRB_HADCHILDREN       = 25; // (V51) Gets set if this process had created any children.
  PRB_HASDLNOTIFY       = 26; // (V51) Set when doslist change signal notify is enabled.
  PRB_HASPLNOTIFY       = 27; // (V51) Set when process list change signal notify is on.
  PRB_SERVERPROCESS     = 28; // (V53) Set when process is one of the internal dos servers
  PRB_HANDLERPROCESS    = 29; // (V52) Set to indicate if this is a handler/filesystem process
  PRB_SHELLPROCESS      = 30; // (V51) Set to indicate if this is a shell handler process
  PRB_EXTENDED_FLAGS    = 31; // (V51) Reserved for internal dos flags expansion use only.
// Mask definitions for above bits
  PRF_FREESEGLIST        = 1 shl PRB_FREESEGLIST;
  PRF_FREECURRDIR        = 1 shl PRB_FREECURRDIR;
  PRF_FREECLI            = 1 shl PRB_FREECLI;
  PRF_CLOSEINPUT         = 1 shl PRB_CLOSEINPUT;
  PRF_CLOSEOUTPUT        = 1 shl PRB_CLOSEOUTPUT;
  PRF_FREEARGS_OBSOLETE  = 1 shl PRB_FREEARGS_OBSOLETE;
  PRF_CLOSEERROR         = 1 shl PRB_CLOSEERROR;
  PRF_LOCKSTACK          = 1 shl PRB_LOCKSTACK;

  PRF_CHILDPROCESS       = 1 shl PRB_CHILDPROCESS;
  PRF_HADCHILDREN        = 1 shl PRB_HADCHILDREN;
  PRF_HASDLNOTIFY        = 1 shl PRB_HASDLNOTIFY;
  PRF_HASPLNOTIFY        = 1 shl PRB_HASPLNOTIFY;
  PRF_SERVERPROCESS      = 1 shl PRB_SERVERPROCESS;
  PRF_HANDLERPROCESS     = 1 shl PRB_HANDLERPROCESS;
  PRF_SHELLPROCESS       = 1 shl PRB_SHELLPROCESS;
  PRF_EXTENDED_FLAGS     = 1 shl PRB_EXTENDED_FLAGS;

// The long word address (BPTR) of this structure is returned by
// Open() and other routines that return a filehandle.
Type
  PFileHandle = ^TFileHandle;
  TFileHandle = record
    fh_StructSize: Word;     // Size of DOS structure allocation.
    fh_Flags: Word;          // --Private DOS use only.
    fh_Interactive: LongInt; // Boolean; True if interactive handle
    fh_MsgPort: PMsgPort;    // MsgPort of the filesystem/handler.
    fh_Buf: BPTR;            // --Private Bufferered stream members.
    fh_Pos: LongInt;
    fh_End: LongInt;
    fh_Func1: APTR;          // --Private function pointers to the
    fh_Func2: APTR;          //   DOS i/o routines.
    fh_Func3: APTR;
    fh_Arg1: BPTR;           // --Private, packet handler use.
    fh_Arg2: APTR;           // --Private, FSVP handler use.
    fh_OpenerPID: LongWord;  // The process ID of the opener. V52.16
    fh_Reserved2: LongInt;   // public expansion, leave alone.
    fh_Reserved1: LongInt;   // public expansion, leave alone.
    fh_Priv: array[0..17] of LongInt; // --Private, start of the remainder of many more private DOS members.
  end; // FileHandle - 128 bytes

// This is the standard extension to EXEC Messages used by DOS
  PDosPacket = ^TDosPacket;
  TDosPacket = record
    dp_Link: PMessage;  // EXEC message
    dp_Port: PMsgPort;  // Reply port for the packet Must be filled in each send.
    case smallint of
     0:(
      dp_Action : LongInt;
      dp_Status : LongInt;
      dp_Status2 : LongInt;
      dp_BufAddr : LongInt;
     );
     1:(
      dp_Type : LongInt;      // See ACTION_... below
      dp_Res1 : LongInt;      // For file system calls this is the result that would have been returned by the
                              // function, e.g. Write() returns actual length written
      dp_Res2 : LongInt;      // For file system calls this is what would have been returned by IoErr()
      dp_Arg1 : LongInt;
      dp_Arg2 : LongInt;
      dp_Arg3 : LongInt;
      dp_Arg4 : LongInt;
      dp_Arg5 : LongInt;
      dp_Arg6 : LongInt;
      dp_Arg7 : LongInt;
     );
  end; // DosPacket - 48 bytes

// A Packet does not require the Message to be before it in memory, but for convenience it is useful to associate the two.
  PStandardPacket = ^TStandardPacket;
  TStandardPacket = record
    sp_Msg: TMessage;
    sp_Pkt: TDosPacket;
  end;

{$PACKRECORDS 4}
// This is the extended 64 bit style EXEC Messages used by DOS. Only dp_Type packets between 8000-8999 range use this structure.
  PDosPacket64 = ^TDosPacket64;
  TDosPacket64 = record
    dp_Link: PMessage;  // EXEC message
    dp_Port: PMsgPort;  // Reply port for the packet Must be filled in each send.
    case smallint of
     0:(
      dp_Action: LongInt;
      dp_pad: LongInt;
      dp_Status2: LongInt;
      dp_Status: Int64;
      dp_BufAddr: LongInt;
     );
     1:(
      dp_Type: LongInt; // See ACTION_... below
      dp_Res0: LongInt; // Special compatibility field. [See below]
      dp_Res2: LongInt; // This is returned for IoErr()
      dp_Res1: Int64;   // This is the 64 bit primary result
      dp_Arg1: LongInt; // 32 bit argument
      dp_Arg2: Int64;   // 64 bit argument
      dp_Arg3: LongInt; // 32 bit argument
      dp_Arg4: LongInt; // 32 bit argument
      dp_Arg5: Int64;   // 64 bit argument
     );
  end; // DosPacket64 - 64 bytes
  PStandardPacket64 = ^TStandardPacket64;
  TStandardPacket64 = record
    sp_Msg: TMessage;
    sp_Pkt: TDosPacket64;
  end;

{$PACKRECORDS 2}
const
// The DosPacket64 dp_Res0 member initialisation value.
  DP64_INIT       = -3;
// Packet types
  ACTION_NIL                  = 0;
  ACTION_STARTUP              = 0;
  ACTION_EVENT                = 6; // INTERNAL for CDFS and CrossDos commodity
  ACTION_CURRENT_VOLUME       = 7; // DEPRECATED from DOS 53.100+
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
  ACTION_EXAMINE_OBJECT       = 23; // DEPRECATED
  ACTION_EXAMINE_NEXT         = 24; // DEPRECATED
  ACTION_DISK_INFO            = 25;
  ACTION_INFO                 = 26;
  ACTION_FLUSH                = 27;
  ACTION_SET_COMMENT          = 28;
  ACTION_PARENT               = 29;
  ACTION_TIMER                = 30; // INTERNAL
  ACTION_INHIBIT              = 31;
  ACTION_SET_DATE             = 34;
  ACTION_SAME_LOCK            = 40;
  ACTION_SAME_FH              = 53; // Added 53.71

  ACTION_READ                 = 82; // 'R'
  ACTION_WRITE                = 87; // 'W'

  ACTION_SINGLE_CHARACTER_MODE = 994;
  ACTION_CHANGE_SIGNAL         = 995;
// Internal packets used by con-handler
  ACTION_READ_RETURN          = 1001; // INTERNAL
  ACTION_WRITE_RETURN         = 1002; // INTERNAL
  ACTION_INT_WRITE_RETURN     = 1003; // INTERNAL
  ACTION_FINDUPDATE           = 1004; // aka MODE_READWRITE
  ACTION_FINDINPUT            = 1005; // aka MODE_OLDFILE
  ACTION_FINDOUTPUT           = 1006; // aka MODE_NEWFILE
  ACTION_END                  = 1007;
  ACTION_SEEK                 = 1008; // DEPRECATED
  ACTION_ICONIFY              = 1009; // INTERNAL

  ACTION_FORMAT         = 1020;
  ACTION_MAKE_LINK      = 1021;
  ACTION_WRITE_PROTECT  = 1023;
  ACTION_READ_SOFT_LINK = 1024;
  ACTION_FH_FROM_LOCK   = 1026;
  ACTION_IS_FILESYSTEM  = 1027;
  ACTION_CHANGE_MODE    = 1028;
  ACTION_COPY_LOCK_FH   = 1030;
  ACTION_PARENT_FH      = 1031;

// Added V52.18 - SetOwnerInfo() - replaces old SetOwner() packet 1036.
  ACTION_SET_OWNER_INFO = 1037;
// Internal packets used by datatypes.library
  ACTION_NEWMEMFILE     = 1039; // INTERNAL
  ACTION_NEWMEMLOCK     = 1040; // INTERNAL
// New packet types for V50
  ACTION_WAIT_FOR_DATA     = 1998;
  ACTION_SET_BLOCKING_MODE = 1999;

// PACKETS 2050-2999 are reserved for use by third party applications.

  ACTION_SHUTDOWN = 3000; // Similar functionality to ACTION_DIE
  ACTION_COLLECT  = 3001; // controls the relinquishing of active filesystem objects
// The following are the ACTION_COLLECT object identifiers (in dp_Arg1) to identify the object that is placed in dp_Arg2.
  ID_COLLECT_LOCK         = 1;
  ID_COLLECT_FILEHANDLE   = 2;
  ID_COLLECT_NOTIFICATION = 3;

  ACTION_FILESYSTEM_ATTR = 3005; // Obtain or Set filesystem specific attributes; FileSystemAttr()  51.46
  ACTION_OBTAIN_CON_INFO = 3006; // Release console information, as obtained through ACTION_OBTAIN_CON_INFO; you must pass the same parameters as with ACTION_OBTAIN_CON_INFO.
// Packets for ExamineObject() and ExamineDir() functions. 51.104
  ACTION_EXAMINEDATA     = 3030;
  ACTION_EXAMINEDATA_FH  = 3031; // added @ 52.30
  ACTION_EXAMINEDATA_DIR = 3040;
// DOS will emulate notifications from 52.33+ if you do not support these
  ACTION_ADD_NOTIFY     = 4097; // optional
  ACTION_REMOVE_NOTIFY  = 4098; // optional
  ACTION_SERIALIZE_DISK = 4200; // Tell a file system to serialize the current volume.
  ACTION_GET_DISK_FSSM  = 4201; // DEPRECATED, Obtain a disk's geometry
  ACTION_FREE_DISK_FSSM = 4202; // DEPRECATED, Obtain a disk's geometry

// New Packet to control dos.library long path handling code. (@ v53.23)
  ACTION_INHIBIT_DOS_LONGPATH_HANDLING = 5323;

// 64 Bit ACTION types, ONLY the 8000 series use struct DosPacket64.  51.62
  ACTION_CHANGE_FILE_POSITION64 = 8001;
  ACTION_GET_FILE_POSITION64    = 8002;
  ACTION_CHANGE_FILE_SIZE64     = 8003;
  ACTION_GET_FILE_SIZE64        = 8004;
  ACTION_LOCK_RECORD64          = 8010; // added 53.86
  ACTION_FREE_RECORD64          = 8011; // added 53.86

// 64 bit PACKETS 8500-8999 are reserved for use by third party applications.

// The following were used to convert UID and GID to names and visa-versa
  ACTION_USERNAME_TO_UID  = 20000;
  ACTION_GROUPNAME_TO_GID = 20001;
  ACTION_UID_TO_USERINFO  = 20002;
  ACTION_GID_TO_GROUPINFO = 20003;

type
// Data structure used by ACTION_OBTAIN_CON_INFO/ACTION_RELEASE_CON_INFO
  PConsoleWindowData = ^TConsoleWindowData;
  TConsoleWindowData = record
    Reserved: array[0..3] of LongWord; // For use by the console handler
    ConsoleWindow: APTR;   // real type: PWindow - Pointer to console window; this may be nil for console handlers
                           // which are not bound to a window, or if that window is currently closed.
    ConsoleIO: PIORequest; // Pointer to console.device I/O request; this may be nil for console handlers which are not
                           // bound to a window, or if that window is currently closed.
    ConsoleType: LongWord; // Identifies the type of console, e.g. ID_RAWCON or ID_CON
    DOSPrivate: array[0..1] of APTR; // Private fields for use by DOS only
  end;

// The following are used by the new V50 Address Tracking functions
  PAddressAndSize = ^TAddressAndSize;
  TAddressAndSize = record
    aas_Address: LongWord;
    aas_Size: LongWord;
  end;

  PFindTrackedAddressMsg = ^TFindTrackedAddressMsg;
  TFindTrackedAddressMsg = record
    ftam_Size: LongInt;
    ftam_Name: STRPTR;
    ftam_Date: TDateStamp;
    ftam_SegList: BPTR;
    ftam_SegmentNumber: LongWord;
    ftam_SegmentOffset: LongWord;
    ftam_ExtraInfo: APTR;
    ftam_ExtraInfoSize: LongInt;
    // -- 40 bytes @ V50
    ftam_AAS: TAddressAndSize;
    // -- 48 bytes @ 53.118
  end;

// The following is used by the new V50 pattern functions
  PCapturedExpression = ^TCapturedExpression;
  TCapturedExpression = record
    cape_Next: PCapturedExpression;
    cape_Match: STRPTR;
    cape_Start: STRPTR;
    cape_End: STRPTR;
    cape_Pool: APTR;   // private
  end;

// The public structure for PseudoSegLists which are used by RunCommand(), LoadSeg(), Create[new]Proc(), AllocSegList(), etc..
  PPseudoSegList = ^TPseudoSegList;
  TPseudoSegList = record
    ps_Next: BPTR;                // Pointer to next segment. Or 0.
    ps_Jump: LongWord;            // Internal compatibility magic.
    ps_Entry: APTR;               // The function entry pointer.
    ps_Ikey: LongWord;            // Identification key.
    ps_DosPrivate: LongWord;      // DOS Private use only.
    ps_Reserved1: LongWord;       // Block alignment - reserved.
    ps_Reserved2: LongWord;       // Block alignment - reserved.
    ps_Data: array[0..3] of Byte; // First 4 bytes of data area.
  end;

// This value is always initialised in the ps_Jump member. (PPC & 68K)
const
  PSJUMP_MAGIC   = $4e714ef9; // NOP,JMP - 68k compatibility
// These are the currently used values for use in the ps_Ikey member.
  PSIKEY_INIT    = 0; // Set this value for normal native, data or 68k types.  UnLoadSeg() will perform the normal memory freeing on these.
  PSIKEY_NOFREE  = $80000000; // Set this value if your seglist must not be UnLoadSeg()'ed. Not for internal components, see PSIKEY_NFSYS below.
  PSIKEY_ELF32   = $7F454C46; //7F 'E' 'L' 'F' This is set EXCLUSIVELY by LoadSeg() for ELF32 executables.
  PSIKEY_NFSYS   = $FFFFFFFF;

type
// A structure ONLY for the Dos resident list.  Do NOT allocate these, use AddSegment() and heed the warnings in the autodocs!
  PDosResidentSeg = ^TDosResidentSeg;
  TDosResidentSeg = record
    seg_Next: BPTR;   // BCPL pointer to next DosResidentSeg, or zero
    seg_UC: LongInt;  // Use Count
    seg_Seg: BPTR;    // BCPL pointer to seglist of command.
    seg_Name: array[0..3] of Char; // First 4 chars of BCPL style formatted name
  end;
const
  CMD_SYSTEM   = -1;
  CMD_INTERNAL = -2;
  CMD_DISABLED = -999;



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
        dl_A2           : LongInt;      { Private register dump of DOS        }
        dl_A5           : LongInt;
        dl_A6           : LongInt;
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
        rn_RestartSeg   : LongInt;      { SegList for the disk validator process }
        rn_Info         : BPTR;         { Pointer ot the Info structure }
        rn_FileHandlerSegment : BPTR;   { segment for a file handler }
        rn_CliList      : tMinList;     { new list of all CLI processes }
                                        { the first cpl_Array is also rn_TaskArray }
        rn_BootProc     : PMsgPort;     { private ptr to msgport of boot fs      }
        rn_ShellSegment : BPTR;         { seglist for Shell (for NewShell)         }
        rn_Flags        : LongInt;      { dos flags }
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
        cpl_First  : LongInt;      { number of first entry in array }
        cpl_Array  : Array[0..0] of PMsgPort;
                             { [0] is max number of CLI's in this entry (n)
                              * [1] is CPTR to process id of CLI cpl_First
                              * [n] is CPTR to process id of CLI cpl_First+n-1
                              }
       END;

// DOS Processes started from the CLI via RUN or NEWCLI have this additional set of data associated with them
Type
  PCommandLineInterface = ^TCommandLineInterface;
  TCommandLineInterface = record
    cli_Result2: LongInt;       // Value of IoErr from last command
    cli_CurrentDirName: BSTR;   // Name of current directory
    cli_PathList: BPTR;         // Lock associated with command directory
    cli_ReturnCode: LongInt;    // Return code from last command
    cli_CommandName: BSTR;      // Name of current command
    cli_FailLevel: LongInt;     // Fail level (set by FAILAT)
    cli_Prompt: BSTR;           // Current prompt (set by PROMPT)
    cli_StandardInput: BPTR;    // Default (terminal) CLI input stream
    cli_CurrentInput: BPTR;     // Current CLI input stream
    cli_CommandFile: BSTR;      // BSTR name of EXECUTE command file
    cli_Interactive: LongInt;   // Boolean; Truth if prompts required
    cli_Background: LongInt;    // Boolean; Truth if CLI created by 'RUN'
    cli_CurrentOutput: BPTR;    // Current CLI output
    cli_DefaultStack: LongWord; // Stack size to be obtained in long words
    cli_StandardOutput: BPTR;   // Default (terminal) CLI output
    cli_Module: BPTR;           // SegList of currently loaded command
  end;

  // VOLUME structure; vn_Type==DLT_VOLUME
  PVolumeNode = ^TVolumeNode;
  TVolumeNode = record
    vn_Next: BPTR;             // BPTR to next entry in the chain
    vn_Type: LongInt;          // always DLT_VOLUME for dos "volumes"
    vn_Port: PMsgPort;         // msg port for the handler process
    vn_Reserved2: LongInt;     // reserved for use by DOS
    vn_VolumeDate: TDateStamp; // creation date
    vn_LockList: BPTR;         // unused, leave as 0
    vn_DOSType: LongInt;       // ie; 'DOS\7' - 32 bit hex dostype = vn_DiskType
    vn_FSPrivate: LongInt;     // private filesystem use
    vn_Name: BSTR;             // bstr name
    vn_StructSize: LongInt;    // FULL size of this structure
    vn_Reserved: array[0..3] of LongInt; // DOS expansion
  end;

// DEVICE structure; dn_Type==DLT_DEVICE
  PDeviceNode = ^TDeviceNode;
  TDeviceNode = record
    dn_Next: BPTR;          // BPTR to next entry in the chain
    dn_Type: LongInt;       // always DLT_DEVICE for dos "devices"
    dn_Port: PMsgPort;      // msg port to the handler process.
    dn_Reserved1: LongInt;  // reserved for use by DOS
    dn_Handler: BSTR;       // BSTR name to loadseg if seglist==0
    dn_StackSize: LongInt;  // stacksize for the handler process
    dn_Priority: LongInt;   // task priority when starting task
    dn_Startup: BPTR;       // startup msg / FileSysStartupMsg
    dn_SegList: BPTR;       // code to run handler process. if nil then dn_Handler will be loaded
    dn_GlobalVec: LongInt;  // Global vector locking method key to use when starting the handler proc.
                            // A value of -1 is used for standard C startup locking method. Only values -1 & -2 are valid now.
    dn_Name: BSTR;          // BSTR device node name.
    dn_StructSize: LongInt; // FULL size of this structure.
    dn_Reserved: array[0..3] of LongInt; // DOS expansion space.
  end;

  PMultiAssign = ^TMultiAssign; // For normal DLT_LOCK and DLT_NONBINDING(v54) multi-assigns
  TMultiAssign = record
    ma_Next: PMultiAssign;      // next MultiAssign in chain, or nil
    dat: record
      ma_lock: BPTR;   // for DLT_LOCK multi-assigns
      ma_name: STRPTR; // for DLT_NONBINDING multi-assigns (v54)
    end;
  end;


  // ASSIGN struct; DLT_LOCK, DLT_LATE, DLT_NONBINDING
  PAssignNode = ^TAssignNode;
  TAssignNode = record
    an_Next: BPTR;                      // BPTR to next entry in the chain
    an_Type: LongInt;                   // DLT_LOCK,DLT_LATE,DLT_NONBINDING
    an_Port: PMsgPort;                  // Ptr to handler process port for DLT_LOCK types.
    an_Lock: BPTR;                      // Primary lock for DLT_LOCK assignments
    an_AssignName: STRPTR;              // Primary name for non or late-binding assigns
    an_MultiAssignList: PMultiAssign;   // Chain of DLT_LOCK multi-assigns
    an_NBMultiAssignList: PMultiAssign; // Chain of DLT_NONBINDING multi-assigns
    an_Unused: array[0..2] of LongInt;  // not currently used, leave as 0
    an_Name: BSTR;                      // BSTR assignment node name
    an_StructSize: LongInt;             // FULL allocated size of this structure.
    an_Reserved: array[0..3] of LongInt;// DOS reserved expansion space.
  end;

       pAssignList = ^tAssignList;
       tAssignList = record
        al_Next : pAssignList;
        al_Lock : BPTR;
       END;

// COMBINED structure for all types.
  PDOSList = ^TDOSList;
  TDOSList = record
      dol_Next: DWord;    { BPTR }
      dol_Type: LongInt;
      dol_Task: PMsgPort;
      dol_Lock: DWord;    { BPTR }
      case Byte of
      0: ( dol_handler : record
             dol_Handler  : DWord;    { BSTR }
             dol_StackSize: LongInt;
             dol_Priority : LongInt;
             dol_Startup  : DWord;
             dol_SegList  : DWord;    { BPTR }
             dol_GlobVec  : DWord;    { BPTR }
           end;
         );
      1: ( dol_volume : record
             dol_VolumeDate: TDateStamp;
             dol_LockList  : DWord;   { BPTR }
             dol_DiskType  : LongInt;
           end;
         );
      2: ( dol_assign : record
             dol_AssignName: PChar;
             dol_List      : PAssignList;
           end;
         );
      3: ( dol_Misc: array[0..23] of Byte;
           dol_Name: DWord;    { BPTR }
         );
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
        dl_Type         : LongInt;      { see DLT below }
        dl_Task         : PMsgPort;     { ptr to handler task }
        dl_Lock         : BPTR;         { not for volumes }
        dl_VolumeDate   : tDateStamp;   { creation date }
        dl_LockList     : BPTR;         { outstanding locks }
        dl_DiskType     : LongInt;      { 'DOS', etc }
        dl_unused       : LongInt;
        dl_Name         : BSTR;         { bptr to bcpl name }
    end;

{ device structure (same as the DeviceNode structure in filehandler.h) }

    pDevInfo = ^tDevInfo;
    tDevInfo = record
        dvi_Next        : BPTR;
        dvi_Type        : LongInt;
        dvi_Task        : Pointer;
        dvi_Lock        : BPTR;
        dvi_Handler     : BSTR;
        dvi_StackSize   : LongInt;
        dvi_Priority    : LongInt;
        dvi_Startup     : LongInt;
        dvi_SegList     : BPTR;
        dvi_GlobVec     : BSTR;
        dvi_Name        : BSTR;
    end;

{    structure used for multi-directory assigns. AllocVec()ed. }






const
// definitions for dl_Type
  DLT_DEVICE     = 0;
  DLT_LOCK       = 1;  // normal assign
  DLT_VOLUME     = 2;
  DLT_LATE       = 3;  // late-binding assign
  DLT_NONBINDING = 4;  // non-binding assign
  DLT_PRIVATE    = -1; // for internal dos use only

// Flags for Doslist functions
  LDB_READ    = 0;
  LDB_WRITE   = 1;
  LDB_DEVICES = 2;
  LDB_VOLUMES = 3;
  LDB_ASSIGNS = 4;
  LDB_ENTRY   = 5;
  LDB_DELETE  = 6;

// You MUST specify one of LDF_READ or LDF_WRITE
  LDF_READ  = 1 shl LDB_READ;
  LDF_WRITE = 1 shl LDB_WRITE;

// Flags to be passed to LockDosList(), etc
  LDF_DEVICES = 1 shl LDB_DEVICES;
  LDF_VOLUMES = 1 shl LDB_VOLUMES;
  LDF_ASSIGNS = 1 shl LDB_ASSIGNS;
  LDF_ENTRY   = 1 shl LDB_ENTRY;   // internal
  LDF_DELETE  = 1 shl LDB_DELETE;  // internal

// Actually all but the internal locking flags
  LDF_ALL = LDF_DEVICES or LDF_VOLUMES or LDF_ASSIGNS;

// Mode types for NonBlockingModifyDosEntry()   (v51.30)

  NBM_ADDDOSENTRY     = 1 shl 0;
  NBM_REMDOSENTRY     = 1 shl 1;
  NBM_RENAMEDOSENTRY  = 1 shl 2;
  NBM_CHANGESIGNAL    = 1 shl 3;
  NBM_REMFREEDOSENTRY = 1 shl 4; // added 53.67
  NBM_DISKINSERTED    = 1 shl 5; // added 53.73
  NBM_DISKREMOVED     = 1 shl 6; // added 53.73

// A filesystem lock structure, as returned by Lock(), DupLock(), etc.. private
type
  PLock = ^TLock;
  TLock = record
    fl_Link: BPTR;      // PRIVATE - filesystem use only.
    fl_Key: LongInt;    // PRIVATE - filesystem use only.
    fl_Access: LongInt; // PRIVATE - filesystem use only.
    fl_Port: PMsgPort;  // Handler process message port
    fl_Volume: BPTR;    // BPTR to DLT_VOLUME DosList entry
      // -- V51 additions --
    fl_FSPrivate1: APTR;     // PRIVATE - filesystem use only.    */
    fl_FSPrivate2: APTR;     // PRIVATE - filesystem use only.    */
    fl_DOSType: LongWord;    // The DOSType of the filesystem, initialised by AllocDosObject(). */
    fl_StructSize: LongWord; // Full DOS allocated struct size, initialised by AllocDosObject(). */
      // -- V53 additions --
    fl_DosPrivate: LongInt;              // PRIVATE - dos library use only.
    fl_Reserved: array[0..1] of LongInt; // RESERVED for expansion. V53
  end;// Filesystem private data usually extends past the end of this definition.

type
// The Public structure return by GetDeviceProc(), GetDeviceProcFlags()
  PDevProc = ^TDevProc;
  TDevProc = record
    dvp_Port: PMsgPort;  // Handler message port
    dvp_Lock: BPTR;      // Reference lock (or nil)
    dvp_Flags: LongWord; // Flags fields.
  end; // DOS private data extends past the end of this definition.

const
// Definitions for TDevProc.dvp_Flags;
  DVPB_UNLOCK      = 0; // For DOS internal use.
  DVPB_MULTIASSIGN = 1; // The Lock refers to part of a multi-assignment
// The following flags were added at V53.56 to provide additional functionality for LockTags() and other internal subsystems.
  DVPB_DEVICE      = 16; // The supplied path is a device relative specification.
  DVPB_VOLUME      = 17; // The supplied path is a volume relative specification.
  DVPB_ASSIGNMENT  = 18; // The supplied path is an assignment relative specification.
  DVPB_PROGDIR     = 19; // The supplied path is a PROGDIR: relative specification.
  DVPB_CURRDIR     = 20; // The supplied path is a CURRDIR: relative specification.
  DVPB_ROOTDIR     = 21; // The supplied path is a root directory relative specification.
  DVPB_CONSOLE     = 23; // The supplied path is a CONSOLE: handler specification.
  DVPB_FILESYS     = 24; // The supplied path returned the root of the default filesystem.
// Field definitions for above bits
  DVPF_UNLOCK      = 1 shl DVPB_UNLOCK;
  DVPF_MULTIASSIGN = 1 shl DVPB_MULTIASSIGN;
  DVPF_DEVICE      = 1 shl DVPB_DEVICE;
  DVPF_VOLUME      = 1 shl DVPB_VOLUME;
  DVPF_ASSIGNMENT  = 1 shl DVPB_ASSIGNMENT;
  DVPF_PROGDIR     = 1 shl DVPB_PROGDIR;
  DVPF_CURRDIR     = 1 shl DVPB_CURRDIR;
  DVPF_ROOTDIR     = 1 shl DVPB_ROOTDIR;
  DVPF_CONSOLE     = 1 shl DVPB_CONSOLE;
  DVPF_FILESYS     = 1 shl DVPB_FILESYS;

// Error report types for ErrorReport()
  REPORT_STREAM = 0; // a stream
  REPORT_TASK   = 1; // a process - currently unused
  REPORT_LOCK   = 2; // a lock
  REPORT_VOLUME = 3; // a volume node
  REPORT_INSERT = 4; // "please insert volume..."

// Special error codes for ErrorReport()
  ABORT_DISK_ERROR = 296; // Read/write error
  ABORT_BUSY       = 288; // "You MUST replace..."

// Types for initial packets to shells from run/newcli/execute/system. For shell-writers only
  RUN_EXECUTE       = -1;
  RUN_SYSTEM        = -2;
  RUN_SYSTEM_ASYNCH = -3;

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
        fl_Key          : LongInt;      { disk block number }
        fl_Access       : LongInt;      { exclusive or shared }
        fl_Task         : PMsgPort;     { handler task's port }
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

// The ExAll() function and associated structures have been deprecated for V50.
// Use ExamineDir()/ExamineObject() from V50 onwards.

// Structure in which exall results are returned in.
//  Note that only the fields asked for will exist!
type
  PExAllData = ^TExAllData;
  TExAllData = record
    ed_Next: PExAllData;
    ed_Name: STRPTR;
    ed_Type: LongInt;
    ed_Size: LongWord;  // only good to 4gig
    ed_Prot: LongWord;
    ed_Days: LongWord;
    ed_Mins: LongWord;
    ed_Ticks: LongWord;
    ed_Comment: STRPTR; // strings will be after last used field
    ed_OwnerUID: Word;  // new for V39
    ed_OwnerGID: Word;
  end;

{ Control structure passed to ExAll.  Unused fields MUST be initialized to 0, expecially eac_LastKey.
  eac_MatchFunc is a hook (see utility.library documentation for usage)
  It should return true if the entry is to returned, false if it is to be ignored.
  This structure MUST be allocated by AllocDosObject()!}
  PExAllControl = ^TExAllControl;
  TExAllControl = record
    eac_Entries: LongWord;   // number of entries returned in buffer
    eac_LastKey: LongWord;   // Don't touch inbetween linked ExAll calls!
    eac_MatchString: STRPTR; // wildcard string for pattern match OR nil
    eac_MatchFunc: PHook;    // optional private wildcard function
  end;

type
  pDosEnvec = ^tDosEnvec;
  tDosEnvec = record
    de_TableSize: LongWord;      // Size of Environment vector
    de_SizeBlock: LongWord;      // in longwords: standard value is 128
    de_SecOrg: LongWord;         // not used; must be 0
    de_Surfaces: LongWord;       // # of heads (surfaces). drive specific
    de_SectorPerBlock: LongWord; // not used; must be 1
    de_BlocksPerTrack: LongWord; // blocks per track. drive specific
    de_Reserved: LongWord;       // DOS reserved blocks at start of partition.
    de_PreAlloc: LongWord;       // DOS reserved blocks at end of partition
    de_Interleave: LongWord;     // usually 0
    de_LowCyl: LongWord;         // starting cylinder. typically 0
    de_HighCyl: LongWord;        // max cylinder. drive specific
    de_NumBuffers: LongWord;     // Initial # DOS of buffers.
    de_BufMemType: LongWord;     // type of mem to allocate for buffers
    de_MaxTransfer: LongWord;    // Max number of bytes to transfer at a time
    de_Mask: LongWord;           // Address Mask to block out certain memory
    de_BootPri: LongInt;         // Boot priority for autoboot
    de_DosType: LongWord;        // ASCII (HEX) string showing filesystem type; $444F5300 is old filesystem, $444F5301 is fast file system
    de_Baud: LongWord;           // Baud rate for serial handler
    de_Control: LongWord;        // Control smallint for handler/filesystem
    de_BootBlocks: LongWord;     // Number of blocks containing boot code
  end;

const
// The following are the offsets when DosEnvec was accessed as an array of longwords; DE_TABLESIZE is set to the number of fields in the array, minus 1,
//  that is, NOT including the DE_TABLESIZE field itself. So if de_DosType was the last valid entry, de_TableSize = DE_DOSTYPE.

  DE_TABLESIZE        = 0; // minimum value is 11 (includes NumBuffers)
  DE_SIZEBLOCK        = 1; // in longwords: standard value is 128
  DE_SECORG           = 2; // not used; must be 0
  DE_NUMHEADS         = 3; // # of heads (surfaces). drive specific
  DE_SECSPERBLK       = 4; // not used; must be 1
  DE_BLKSPERTRACK     = 5; // sectors per track. drive specific
  DE_RESERVEDBLKS     = 6; // unavailable blocks at start. usually 2
  DE_PREFAC           = 7; // not used; must be 0
  DE_INTERLEAVE       = 8; // usually 0
  DE_LOWCYL           = 9; // starting cylinder. typically 0
  DE_UPPERCYL         = 10; // max cylinder.  drive specific
  DE_NUMBUFFERS       = 11; // starting # of buffers.  typically 5
  DE_MEMBUFTYPE       = 12; // type of mem to allocate for buffers.
  DE_BUFMEMTYPE       = 12; //    1 is public, 3 is chip, 5 is fast
  DE_MAXTRANSFER      = 13; // Max number bytes to transfer at a time
  DE_MASK             = 14; // Address Mask to block out certain memory
  DE_BOOTPRI          = 15; // Boot priority for autoboot
  DE_DOSTYPE          = 16; // ASCII (HEX) string showing filesystem type; $444F5300 is old filesystem, $444F5301 is fast file system
  DE_BAUD             = 17; // Baud rate for serial handler
  DE_CONTROL          = 18; // Control word for handler/filesystem
  DE_BOOTBLOCKS       = 19; // Number of blocks containing boot code

// The file system startup message is linked into a device node's startup
// field.  It contains a pointer to the above environment, plus the
// information needed to do an exec OpenDevice().
type
  PFileSysStartupMsg = ^TFileSysStartupMsg;
  TFileSysStartupMsg = record
    fssm_Unit: LongWord;  // exec unit number for this device
    fssm_Device: BSTR;    // null terminated bstring to the device name
    fssm_Environ: BPTR;   // ptr to environment table (see above)
    fssm_Flags: LongWord; // flags for OpenDevice()
  end;

// The public portion of the structure used for the new V50 functions;
// GetDiskFileSystemData() and FreeDiskFileSystemData().
// Do not depend on the size of this structure to stay constant.
  PFileSystemData = ^TFileSystemData;
  TFileSystemData = record
    fsd_Size: LongWord;                  // the size of this structure
    fsd_FileSystemName: STRPTR;
    fsd_DeviceName: STRPTR;
    fsd_DeviceUnit: LongWord;
    fsd_DeviceFlags: LongWord;
    fsd_Environment: PDosEnvec;          // WARNING: See (Note 1) below
    fsd_DosPrivate: array[0..3] of APTR; // this really is dos private !
    fsd_Reserved: array[0..3] of APTR;
  end;
const
  // The new V53 struct FileSystemVectorPort includes follow.
  FS_VECTORPORT_VERSION = 53;

type
  PFileSystemVectors = ^TFileSystemVectors;
  TFileSystemVectors = record
    StructSize: LongWord; // Filesystem must initialise this to; sizeof(struct FileSystemVectors)
    Version: LongWord;    // Filesystem must initialise this to; FS_VECTORPORT_VERSION
    FSPrivate: APTR;      // Private field for exclusive use by the filesystem, this is generally used to point to the filesystems private global data structure to
                          // make it accessible from the vector-port functions. Optionally, the filesystem may just initialise this to zero and then after the AllocDosObject() call, place your required data in there.
    Reserved: array[0..2] of LongWord; // Filesystems initialise these to 0 - reserved expansion
    DOSPrivate: APTR;     // Filesystems initialise this  to 0 - private DOS usage
    // function links
    DOSEmulatePacket: Pointer;
    FSLock,
    FSUnLock,
    FSDupLock,
    FSCreateDir,
    FSParentDir,
    FSDupLockFromFH,
    FSOpenFromLock,
    FSParentOfFH,
    FSOpen,
    FSClose,
    FSDelete,
    FSRead,
    FSWrite,
    FSFlush,
    FSChangeFilePosition,
    FSChangeFileSize,
    FSGetFilePosition,
    FSGetFileSize,
    FSChangeLockMode,
    FSChangeFileMode,
    FSSetDate,
    FSSetProtection,
    FSSetComment,
    FSSetGroup,
    FSSetOwner,
    FSRename,
    FSCreateSoftLink,
    FSCreateHardLink,
    FSReadSoftLink,
    FSSameLock,
    FSSameFile,
    FSFileSystemAttr,
    FSVolumeInfoData,
    FSDeviceInfoData,
    FSReserved1,
    FSReserved2,
    FSExamineLock,
    FSExamineFile,
    FSExamineDir,
    FSInhibit,
    FSWriteProtect,
    FSFormat,
    FSSerialize,
    FSRelabel,
    FSReserved3,
    FSAddNotify,
    FSRemoveNotify,
    FSLockRecord,
    FSUnLockRecord: Pointer;
    //=== End of V53 vectors, - New V54 vectors to be added after this line ===
    End_Marker: LongInt; // This must always be at vector table end, set to -1
  end;

  PFileSystemVectorPort = ^TFileSystemVectorPort;
  TFileSystemVectorPort = record
    MP: TMsgPort;
    FSV: TFileSystemVectors;
  end;

  FSVP = TFileSystemVectorPort; // shortcut

const
 // The type of device to mount
  MDT_FileSystem = 0; // A file system, which is associated with a block storage device
  MDT_Handler    = 1; // Any other kind which does not require a block storage device
// Control tags which describe the device MountDevice() should mount
  MD_Dummy           = TAG_USER + 4000;
  MD_SectorSize      = MD_Dummy + 1;  // Sector size in bytes (ULONG); must be a multiple of 4
  MD_Surfaces        = MD_Dummy + 2;  // Number of surfaces the file system should use (ULONG)
  MD_SectorsPerBlock = MD_Dummy + 3;  // Number of sectors that make up a data block (ULONG)
  MD_SectorsPerTrack = MD_Dummy + 4;  // Number of sectors that make up a track (ULONG)
  MD_Reserved        = MD_Dummy + 5;  // Number of sectors at the beginning of the partition which should not be touched by the file system (ULONG)
  MD_PreAlloc        = MD_Dummy + 6;  // Number of sectors at the end of the partition which should not be touched by the file system (ULONG)
  MD_LowCyl          = MD_Dummy + 7;  // Lowest cylinder number used by the file system (ULONG)
  MD_HighCyl         = MD_Dummy + 8;  // Highest cylinder number used by the file system (ULONG)
  MD_NumBuffers      = MD_Dummy + 9;  // Number of data buffers the file system is to use (ULONG)
  MD_BufMemType      = MD_Dummy + 10; // The type of memory to use for data buffers (ULONG)
  MD_MaxTransfer     = MD_Dummy + 11; // Maximum number of bytes the device driver can transfer in a single step (ULONG)
  MD_Mask            = MD_Dummy + 12; // Bit mask which covers the address range which the device driver can access (ULONG)
  MD_DOSType         = MD_Dummy + 13;  // File system signature, e.g. ID_DOS_DISK (ULONG)
  MD_Baud            = MD_Dummy + 14;  // Transmission speed (bits/second) to be used by the handler (ULONG)
  MD_Control         = MD_Dummy + 15;  // Control information for the handler/file system (STRPTR).
  MD_Device          = MD_Dummy + 16;  // Name of the exec device driver this file system is to use (STRPTR).
  MD_Unit            = MD_Dummy + 17;  // Exec device driver unit number to be used by this file system (ULONG).
  MD_Flags           = MD_Dummy + 18;  // Flags to use when the file system opens the exec device driver (ULONG).
  MD_StackSize       = MD_Dummy + 19;  // Size of the stack to allocate for the file system (ULONG).
  MD_Priority        = MD_Dummy + 20;  // Priority to start the file system process with (LONG).
  MD_GlobVec         = MD_Dummy + 21;  // Global vector number (LONG).
  MD_StartupNumber   = MD_Dummy + 22;  // The number to store as the file system startup data (LONG).
  MD_StartupString   = MD_Dummy + 23;  // The string to store as the file system startup data (STRPTR).
  MD_IgnoreFSR       = MD_Dummy + 24;  // Whether the file system parameters should be initialized from the FileSystem.resource or not (BOOL).
  MD_Activate        = MD_Dummy + 25;  // Whether the file system should be activated immediately after it has been mounted (BOOL).
  MD_Handler         = MD_Dummy + 26;  // Name of the handler which implements the file system (STRPTR).
  MD_SegList         = MD_Dummy + 27;  // The segment list which refers to the code which implements the file system (BPTR).
  MD_Port            = MD_Dummy + 28;  // The port which implements the file system (struct MsgPort *).
  MD_Entry           = MD_Dummy + 29;  // The function which implements the file system (VOID (*)(VOID)).


// Flags for DismountDevice()
  DMDF_KEEPDEVICE     =  1 shl 0;
  DMDF_REMOVEDEVICE   =  1 shl 1;
  DMDF_FORCE_DISMOUNT =  1 shl 2;

const
// --- NotifyMessage Class
  NOTIFY_CLASS  =  $40000000;
// --- NotifyMessage Codes
  NOTIFY_CODE   =  $1234;


// Sent to the application if SEND_MESSAGE is specified.

type
  PNotifyRequest = ^tNotifyRequest;
  tNotifyRequest = record
    nr_Name : STRPTR;       // The name of object for notification
    nr_FullName: STRPTR;   // PRIVATE: set by dos - don't touch
    nr_UserData: LongWord; // For the applications use
    nr_Flags: LongWord;    // Notify method flags NRF_xxx
    nr_stuff: record
    case smallint of
    0: (nr_Msg: record
        nr_Port: PMsgPort;              // for SEND_MESSAGE
        nr_MsgPad: array[0..3] of Byte; // Reserved
     end);
    1: (nr_Signal : record
        nr_Task : PTask;                // for SEND_SIGNAL
        nr_SignalNum : Byte;            // for SEND_SIGNAL
        nr_SigPad: array[0..2] of Byte; // reserved
     end);
     0: (nr_CallHook: record
        nr_Hook: PHook;                  // for CALL_HOOK
        nr_HookPad: array[0..3] of Byte; // Reserved
     end);
    end;
    nr_Reserved : array[0..1] of LongWord; // Reserved - leave as 0 for now
    // internal use by dos and handlers
    nr_DosPrivate:LongWord;                // PRIVATE: DOS use only. -  V51.30
    nr_FSPrivate: APTR;                    // PRIVATE: FS/Handler use only. - V51.30
    nr_MsgCount: LongWord;                 // PRIVATE: # of outstanding msgs
    nr_Handler: PMsgPort;                  // PRIVATE: handler sent to (for EndNotify)
    nr_Expansion: array[0..3] of LongWord; // expansion space - added V51.30
  end;

  PNotifyMessage = ^TNotifyMessage;
  TNotifyMessage = record
    nm_ExecMessage: TMessage;
    nm_Class: LongWord;
    nm_Code: Word;
    nm_NReq: PNotifyRequest;  // Don't modify the request while active !
    nm_DoNotTouch: LongWord;  // Like it says! For use by handlers
    nm_DoNotTouch2: LongWord; // ditto
  end;

const
// Flag bit numbers
  NRB_SEND_MESSAGE      = 0;
  NRB_SEND_SIGNAL       = 1;
  NRB_WAIT_REPLY        = 3;
  NRB_NOTIFY_INITIAL    = 4;
  NRB_CALL_HOOK         = 5;
  NRB_DOS_NOTIFY_ONLY   = 12;
  NRB_MAGIC             = 31; // PRIVATE - Handler use only.

// --- NotifyRequest Flags ------------------------------------------------
// Notify Methods - the following three are mutually exclusive
  NRF_SEND_MESSAGE      = 1 shl NRB_SEND_MESSAGE;
  NRF_SEND_SIGNAL       = 1 shl NRB_SEND_SIGNAL;
  NRF_CALL_HOOK         = 1 shl NRB_CALL_HOOK;
// Do not queue messages for NRF_SEND_MESSAGE method
  NRF_WAIT_REPLY        = 1 shl NRB_WAIT_REPLY;
// Always send an initial notification on setup
  NRF_NOTIFY_INITIAL    = 1 shl NRB_NOTIFY_INITIAL;
// New flag for V52.33+ DOS, specifying this flag prevents the use  of filesystem supported notification, even if it is available,
// it forces usage of the dos.library notification function instead. The dos.library notification function supports all three methods with both file and directory monitoring.
  NRF_DOS_NOTIFY_ONLY   = 1 shl NRB_DOS_NOTIFY_ONLY;

// PRIVATE - Handler use only. Do NOT set or remove NRF_MAGIC yourself.
  NRF_MAGIC             = 1 shl NRB_MAGIC;

// PRIVATE - Bitmask of flags that are reserved for use by the handler:
  NR_HANDLER_MASK       = $ffff0000;
  NR_HANDLER_FLAGS      = NR_HANDLER_MASK;  // old source compatibility

type
// --- NotifyHook data
  PNotifyHookMsg = ^TNotifyHookMsg;
  TNotifyHookMsg = record
    nhm_Size: LongInt;   // Size of data structure
    nhm_Action: LongInt; // What happened (see below)
    nhm_Name: STRPTR;    // The name of the object
  end;

const
// nhm_Action types
  NHM_ACTION_INITIAL = -1; // Initial invocation
  NHM_ACTION_ADD     = 0;  // Object was added
  NHM_ACTION_CHANGE  = 1;  // Object has changed
  NHM_ACTION_DELETE  = 2;  // Object was removed

// A shell search path list component. Do not allocate this yourself!
type
  PPathNode = ^TPathNode;
  TPathNode = record
    pn_Next: BPTR; // Pointer to next path node
    pn_Lock: BPTR; // Directory lock
  end;

// Parameters for use with the AddPathNode() function. Where to add the new node?
const
  ADDCMDPATHNODE_HEAD = 0;
  ADDCMDPATHNODE_TAIL = 1;

// The message passed to hook invoked by the SearchCmdPathList() function.
type
  PSearchCmdPathListMsg = ^TSearchCmdPathListMsg;
  TSearchCmdPathListMsg = record
    splm_Size: LongInt;
    splm_Lock: BPTR;
    splm_Name: STRPTR;
  end;

{*********************************************************************
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
  PCSource = ^TCSource;
  TCSource = record
    CS_Buffer: STRPTR;
    CS_Length: LongInt;
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

  PRDArgs = ^TRDArgs;
  TRDArgs = record
    RDA_Source: TCSource; // Select input source
    RDA_DAList: LongInt;  // PRIVATE.
    RDA_Buffer: STRPTR;   // Optional string parsing space.
    RDA_BufSiz: LongInt;  // Size of RDA_Buffer (0..n)
    RDA_ExtHelp: STRPTR;  // Optional extended help
    RDA_Flags: LongInt;   // Flags for any required control
  end;

const
  RDAB_STDIN     = 0; // Use "STDIN" rather than "COMMAND LINE"
  RDAB_NOALLOC   = 1; // If set, do not allocate extra string space.
  RDAB_NOPROMPT  = 2; // Disable reprompting for string input.
  RDAF_STDIN     = 1 shl RDAB_STDIN;
  RDAF_NOALLOC   = 1 shl RDAB_NOALLOC;
  RDAF_NOPROMPT  = 1 shl RDAB_NOPROMPT;

// Modes for LockRecord/LockRecords()
  REC_EXCLUSIVE       = 0;
  REC_EXCLUSIVE_IMMED = 1;
  REC_SHARED          = 2;
  REC_SHARED_IMMED    = 3;

  RECF_DOS_METHOD_ONLY = 1 shl 16;
  REC_MODE_MASK        = $FF;

// Struct to be passed to LockRecords()/UnLockRecords() v53.86+
// Note: the rec_Size member MUST be set in each struct or it won't work.
type
  PRecordLock = ^TRecordLock;
  TRecordLock = record
    rec_Size: LongWord;   // This MUST be set to sizeof(struct RecordLock)
    rec_FH: BPTR;         // filehandle
    rec_Offset: LongWord; // offset in file
    rec_Length: LongWord; // length of file record to be locked
    rec_Mode: LongWord;   // Type of lock
  end;

// The structure in the pr_LocalVars list. Do NOT allocate yourself, use SetVar()!!! This structure may grow in
// future releases!  The list should be left in alphabetical order, and may have multiple entries with the same name but different types.
  PLocalVar = ^TLocalVar;
  TLocalVar = record
    lv_Node: TNode;
    lv_Flags: Word;
    lv_Value: STRPTR;
    lv_Len: LongWord;
  end;
// The LocalVar->lv_Node.ln_Name points to the name buffer.
// The LocalVar->lv_Node.ln_Type is used by the system.
// The LocalVar->lv_Node.ln_Pri is reserved for system use.
const
// LocalVar type identifiers:
  LV_VAR     =   0; // a variable
  LV_ALIAS   =   1; // an alias
// bit definitions for LocalVar type:
  LVB_IGNORE =   7;   // ignore this entry on GetVar, FindVar() etc
  LV_TYPE_MASK = $FF; // the low 8 flags bits hold the local var type

// bit definitions of flags passed to GetVar()/SetVar()/DeleteVar(), ScanVars() these bit defs are OR'ed with the type. Item will be treated as a single line of text unless BINARY_VAR is specified
  GVB_GLOBAL_ONLY       = 8;  // only use global vars, no local vars will be accessed.
  GVB_LOCAL_ONLY        = 9;  // only use local vars, no global vars will be accessed.
  GVB_BINARY_VAR        = 10; // treat as binary var, don't truncate at a newline or carriage return char.
  GVB_DONT_NULL_TERM    = 11; // can only be used with GVF_BINARY_VAR do not add a nul-terminator 0 byte at the end of data.
  GVB_SAVE_VAR          = 12; // only works with GVF_GLOBAL_VAR this is only supported in >= V39 dos.  V37 dos ignores this. this causes SetVar to affect ENVARC: as well as ENV:
  GVB_SCAN_ENVARC       = 13; // only usefull with GLOBAL vars scan envarc: instead of env: for use in ScanVars() - v50
  GVB_SCAN_LEVEL        = 14; // for ScanVars() - v53 scan single directory level do not go into further sub directories. - v53
  GVB_SCAN_TOPLEVEL     = GVB_SCAN_LEVEL;
  GVB_SCAN_STARTDIR     = 15;

  LVF_IGNORE              = 1 shl LVB_IGNORE;
  GVF_GLOBAL_ONLY         = 1 shl GVB_GLOBAL_ONLY;
  GVF_LOCAL_ONLY          = 1 shl GVB_LOCAL_ONLY;
  GVF_BINARY_VAR          = 1 shl GVB_BINARY_VAR;
  GVF_DONT_NULL_TERM      = 1 shl GVB_DONT_NULL_TERM;
  GVF_SAVE_VAR            = 1 shl GVB_SAVE_VAR;
  GVF_SCAN_ENVARC         = 1 shl GVB_SCAN_ENVARC;
  GVF_SCAN_LEVEL          = 1 shl GVB_SCAN_LEVEL;
  GVF_SCAN_STARTDIR       = 1 shl GVB_SCAN_STARTDIR;

const
//definitions for the (V51.96) GetDiskInfo() call.
  GDI_Dummy           = TAG_USER + 500;
  GDI_StringNameInput = GDI_Dummy + 1;  // STRPTR - pointer to a string identifier for the handler.
  GDI_FileHandleInput = GDI_Dummy + 2;  // BPTR - BCPL pointer to a FileHandle to identify the handler
  GDI_LockInput       = GDI_Dummy + 3;  // BPTR - BCPL pointer to a Lock to identify the handler.
  GDI_FileLockInput   = GDI_LockInput;
  GDI_MsgPortInput    = GDI_Dummy + 4;  // PMsgPort - Pointer to the handlers message port
  GDI_VolumeRequired  = GDI_Dummy + 10; // LongBool - True for ACTION_INFO, FALSE for ACTION_DISK_INFO
  GDI_InfoData        = GDI_Dummy + 11; // PInfoData - Pointer to an InfoData structure
// definitions for the (V53.59) ObtainConsoleDataTags() call.
  OCD_Dummy           = TAG_USER + 600;
  OCD_FileHandleInput = OCD_Dummy + 1; // BPTR - BCPL pointer to a FileHandle for a console window.
  OCD_MsgPortInput    = OCD_Dummy + 2; // PMsgPort - Pointer to a console message port.
// definitions for the (V50) GetSegListInfo() call
  GSLI_Dummy         = TAG_USER + 4000;
  GSLI_NATIVE        = GSLI_Dummy + 1;  // PPPseudoSegList - if PPC executable. this only means it's native and executable, it does NOT imply any particular type.
  GSLI_Data          = GSLI_Dummy + 2;  // PAPTR - data, if a data-only pseudoseglist.
  GSLI_68KPS         = GSLI_Dummy + 3;  // PPPseudoSeglist - if 68K pseudoseglist
  GSLI_ElfHandle     = GSLI_Dummy + 4;  // PPElf32_Handle
  GSLI_68KHUNK       = GSLI_Dummy + 5;  // PBPTR - seglist, if old 68K HUNK style seglist.
  GSLI_68KOVLAY      = GSLI_Dummy + 6;  // PBPTR - seglist, if old 68K OVERLAY seglist.
  GSLI_68KFileSize   = GSLI_Dummy + 7;  // PLongWord - size of the 68K binary load file. 51.58
  GSLI_68KFileXsum   = GSLI_Dummy + 8;  // PLongWord - 32 bit xsum of the 68K binary load file. 51.58
  GSLI_HeaderSize    = GSLI_Dummy + 9;  // PLongWord - byte size of the DOS allocated segment header. 51.99
  GSLI_SegmentSize   = GSLI_Dummy + 10; // PLongWord - byte size of the entire segment allocation.  52.13
  GSLI_VersionString = GSLI_Dummy + 11; // STRPTR - returns a "$VER:" version string in a seglist. 52.18
  GSLI_ResidentVersionString = GSLI_Dummy + 12; // STRPTR - returns Resident->rt_IDString version string. 53.21
  GSLI_ResidentStruct = GSLI_Dummy + 13; // PResident - returns a pointer to a struct Resident. 53.70
// definitions for the (V50) AddSegmentTagList() call
  AS_Dummy       = TAG_USER + 3000;
  AS_SegmentList = AS_Dummy + 1; // Segment list, as returned by the LoadSeg() function
  AS_Entry       = AS_Dummy + 2; // Address of a function which implements the command
// definitions for the (V53) LockTagList() call
  LK_Dummy       = TAG_USER + 5750;
  LK_Name        = LK_Dummy + 1; // STRPTR - Name of object to lock.
  LK_Mode        = LK_Dummy + 2; // LongInt - Access mode; SHARED_LOCK or EXCLUSIVE_LOCK
  LK_InfoMask    = LK_Dummy + 3; // PLongWord - pointer to info mask storage area.
  LK_SoftLinks   = LK_Dummy + 4; // PLongWord - pointer to softlink count storage area.
  LK_ResolveSL   = LK_Dummy + 5; // LongInt - Boolean switch to control softlink resolution.
  LK_ResolveMA   = LK_Dummy + 6; // LongInt - Boolean switch to control multi-assign resolution. 54.21

// definitions for the DosSystem() call.
  SYS_Dummy              = TAG_USER + 32;
  SYS_CommandRunMode     = SYS_Dummy + 0; // (LongBool) PRIVATE Internal SHELL tag to identify "run" command.
  SYS_Input              = SYS_Dummy + 1; // (BPTR) specifies the input filehandle
  SYS_Output             = SYS_Dummy + 2; // (BPTR) specifies the output filehandle
  SYS_Asynch             = SYS_Dummy + 3; // (LongBool) run asynch, close input/output on exit(!)
  SYS_UserShell          = SYS_Dummy + 4; // (LongBool) send to user shell instead of boot shell
  SYS_CustomShell        = SYS_Dummy + 5; // (STRPTR) send to a specific shell (data is name)
  SYS_Error              = SYS_Dummy + 6; // (BPTR) specifies the error output filehandle (New for V50)
  SYS_ExecuteInputStream = SYS_Dummy + 7; // (LongBool) instead of reading the 'command' string parameter, reads commands from the input filehandle instead. (V53.45)

//definitions for the CreateNewProc() call; you MUST specify one of NP_Seglist or NP_Entry. All others are optional.
  NP_Dummy       = TAG_USER + 1000;
  NP_Seglist     = NP_Dummy + 1;  // seglist of code to run for the process
  NP_FreeSeglist = NP_Dummy + 2;  // free seglist on exit - only valid for for NP_Seglist. Default is FALSE.
  NP_Entry       = NP_Dummy + 3;  // entry point to run
  NP_Input       = NP_Dummy + 4;  // filehandle - default is DosOpen('NIL:'...)
  NP_Output      = NP_Dummy + 5;  // filehandle - default is DosOpen('NIL:'...)
  NP_CloseInput  = NP_Dummy + 6;  // close input filehandle on exit default TRUE
  NP_CloseOutput = NP_Dummy + 7;  // close output filehandle on exit default TRUE
  NP_Error       = NP_Dummy + 8;  // filehandle - default is nil V50
  NP_CloseError  = NP_Dummy + 9;  // close error filehandle on exit default FALSE V50
  NP_CurrentDir  = NP_Dummy + 10; // lock - default is parent's current dir
  NP_StackSize   = NP_Dummy + 11; // stacksize for process - default 4000
  NP_Name        = NP_Dummy + 12; // name for process - default 'New Process'
  NP_Priority    = NP_Dummy + 13; // priority - default same as parent
  NP_ConsolePort = NP_Dummy + 14; // consoletask - old compatibility  - default same as parent
  NP_ConsoleTask = NP_ConsolePort;
  NP_WindowPtr   = NP_Dummy + 15; // window ptr - default is same as parent
  NP_ProgramDir  = NP_Dummy + 16; // home program directory - default curr progdir
  NP_CopyVars    = NP_Dummy + 17; // Boolean to copy local vars - default TRUE
  NP_Cli         = NP_Dummy + 18; // create cli structure - default FALSE
  NP_Path        = NP_Dummy + 19; // path - default is copy of parents path only valid if a cli process!
  NP_CommandName = NP_Dummy + 20; // commandname - valid only for CLI
  NP_Arguments   = NP_Dummy + 21; // cstring of arguments
  NP_unused1     = NP_Dummy + 22; // never implemented
  NP_unused2     = NP_Dummy + 23; // INTERNALLY - used
  NP_ExitCode    = NP_Dummy + 24; // code to be called on process exit, just before cleanup
  NP_ExitData    = NP_Dummy + 25; // optional argument for NP_ExitCode function - default 0
  NP_UserData    = NP_Dummy + 26; // optional value to install into TTask.tc_UserData.
  NP_Child       = NP_Dummy + 27; // LongBool flag to nominate this new process as a dependant child of the parent. (V50)
  NP_NotifyOnDeathMessage = NP_Dummy + 28; // PDeathMessage -- (V51.53) Specify an initialised death message structure to ReplyMsg() to,
                                           // upon death of this process. Defaults to none.
  NP_NotifyOnDeathSigTask = NP_Dummy + 29; // PTask -- (V51.53) Specify the task or process to signal upon death of this process.
                                           // Specify NULL for the parent of this child process. Defaults to no signalling, if this tag is not specified
  NP_NotifyOnDeathSignalBit = NP_Dummy + 30; // A value 0-31 for the signal bit number to send to the task NP_NotifyOnDeathSigTask, upon death of this process. Defaults to SIGB_CHILD if not specified.
  NP_LocalVars   = NP_Dummy + 31; // STRPTR * -- (V51.70) Paired array of string pointers representing a list of local variables to add to the new process, array must be
                                  // arranged as;  Name1,Value1, Name2,Value2, Name3,Value3, nil; Default nil
  NP_EntryCode   = NP_Dummy + 32; // code to be called on process startup, just before internalRunCommand()
  NP_EntryData   = NP_Dummy + 33; // optional argument for NP_EntryCode function - default 0
  NP_FinalCode   = NP_Dummy + 34; // code to be called on process exit, just before cleanup
  NP_FinalData   = NP_Dummy + 35; // optional argument for NP_FinalCode function - default 0
  NP_LocalAlias  = NP_Dummy + 36; // PSTRPTR -- (added V52.5) Paired array of string pointers to represent a list of local aliases to add to the new process,
                                  // array must be arranged as; Alias1,Value1, Alias2,Value2, Alias3,Value3, nil; Default nil
  NP_LockStack   = NP_Dummy + 37; // Boolean tag to cause exec to lock the stack in place, specifying TRUE will prevent the stack frame being swapped out. Default FALSE   (added V52.10)
  NP_OwnerUID    = NP_Dummy + 38; // LongWord -- (added 52.18) Specify the pr_UID value for the new process.
  NP_OwnerGID    = NP_Dummy + 39; // LongWord -- (added 52.18) Specify the pr_GID value for the new process.

// tags for AllocDosObject()
  ADO_Dummy        = TAG_USER + 2000;
  ADO_TermCharStr  = ADO_Dummy + 0;    // Added for FReadLine() V53.19
// Obsolete tags +1...+5 have removed
  ADO_AddedSize    = ADO_Dummy + 6; // Size of additional buffer to allocate over and above the base structure size required by the flag DOS_xxxxx .  (V50)
  ADO_Strlen       = ADO_AddedSize;
  ADO_Flags        = ADO_Dummy + 7; // Generic 32 bit flags for those allocations that require initialisation of a specific flags field. (V50)
  ADO_DOSType      = ADO_Flags;     // alias V51
  ADO_TermChar     = ADO_Flags;     // alias V51
  ADO_Type         = ADO_Flags;     // alias V52
  ADO_Vectors      = ADO_Flags;     // alias V53
  ADO_Mask         = ADO_Dummy + 8; // Generic 32 bit mask for those allocations that require initialisation of a specific bitmask. (V50)
  ADO_AddColon     = ADO_Mask;      // alias V53
  ADO_Name         = ADO_Dummy + 9; // Generic STRPTR Name. (V52)
  ADO_NotifyName   = ADO_Name;      // alias V52 Name of the volume, directory or file to monitor and produce notification messages for. (V51)
  ADO_NotifyUserData     = ADO_Dummy + 10; // User data to be stored in a NotifyRequest. (V51)
  ADO_NotifyMethod       = ADO_Dummy + 11; // Notification method; must be one of NRF_SEND_MESSAGE, NRF_SEND_SIGNAL or NRF_CALL_HOOK. (V51)
  ADO_NotifyPort         = ADO_Dummy + 12; // MsgPort to send notification messages to; this is used for the NRF_SEND_MESSAGE notification method. (V51)
  ADO_NotifyTask         = ADO_Dummy + 13; // Task to send a notification signal to; this is used for the NRF_SEND_SIGNAL notification method. (V51)
  ADO_NotifySignalNumber = ADO_Dummy + 14; // The signal number (0..31) to use when sending a notification signal; this is used for the NRF_SEND_SIGNAL notification method. (V51)
  ADO_NotifyHook         = ADO_Dummy + 15; // The hook to call when a notification is required; this is used for the NRF_CALL_HOOK method. (V51)
  ADO_NotifyWaitReply    = ADO_Dummy + 16; // Selects if the further notification messages should be sent unless the last message has been replied. (V51)
  ADO_NotifyInitial      = ADO_Dummy + 17; // Selects if the notification should be sent immediately after it was requested. (V51)
  ADO_Size               = ADO_Dummy + 18; // Specify allocation size that is larger than the default. (V51)
  ADO_DOSMethodOnly      = ADO_Dummy + 19; // Force the dos.library to service the notify request, and exclude the filesystem routines even if they are available. (V52.33)
  ADO_ExamineData_NameSize = ADO_Dummy + 20; // Selects the size of the name buffer for DOS_EXAMINEDATA. (V51)
  ADO_ExamineData_CommentSize = ADO_Dummy + 21; // Selects the size of the comment buffer for DOS_EXAMINEDATA (V51)
  ADO_ExamineData_LinkSize = ADO_Dummy + 22; // Selects the size of the link name buffer for DOS_EXAMINEDATA. (V51)
  ADO_ExamineDir_Context = ADO_Dummy + 23; // Used by filesystems and DOS to pass an ExamineContext. (V51.104)

// Tags for DosControl()
  DC_Dummy     = TAG_USER + 5000;
  DC_WildStarW = DC_Dummy + 1; // (LongInt Boolean) -- Write to the default WildStar switch. Default is FALSE.
  DC_WildStarR = DC_Dummy + 2; // (PLongInt) -- Obtain the state of WildStar switch.
  DC_FHBufferW = DC_Dummy + 3; // (LongInt) -- Writes the size in bytes for all FileHandle buffers. The default value for V50 DOS is 8000 bytes.
  DC_FHBufferR = DC_Dummy + 4; // (PLongInt) -- Obtain the size in bytes for FH Buffers.
  DC_WarnPostTimeW = DC_Dummy + 5;  // (LongInt) -- Write the number of seconds to post warning for. Default posting time is currently 120 seconds.
  DC_WarnPostTimeR = DC_Dummy + 6;  // (PLongInt) -- Obtain the value for WarnPostTime.
  DC_WarnWaitTimeW = DC_Dummy + 7;  // (LongInt) -- Write the number of seconds to wait between warnings. Default wait time is currently 5 seconds.
  DC_WarnWaitTimeR = DC_Dummy + 8;  // (PLongInt) -- Obtain the value for WarnWaitTime.
  DC_MinProcStackW = DC_Dummy + 9;  // (LongInt) -- Write the number of bytes to enforce as min proc stack
  DC_MinProcStackR = DC_Dummy + 10; // (PLongInt) -- Obtain the value for MinProcStack.
  DC_AssignMountW  = DC_Dummy + 11; // (LongInt) -- Write to the default AssignMount switch.
  DC_AssignMountR  = DC_Dummy + 12; // (PLongInt) -- Obtain the state of the AssignMount switch.
  DC_BootCliFontSizeW = DC_Dummy + 13; // (LongInt) -- Write the boot cli font size. TOPAZ_SIXTY or TOPAZ_EIGHTY
  DC_BootCliFontSizeR = DC_Dummy + 14; // (PLongInt) -- Obtain the size of the boot cli font size.
  DC_Reload68KApplistW = DC_Dummy + 15; // (LongInt) -- DOSTRUE will cause DEVS:applications.dos config file to reload from disk. 51.58
  DC_unused1R          = DC_Dummy + 16; // pad tag for DC_Reload68KApplistR slot
  DC_BootCliCloseGadgetW = DC_Dummy + 17; // (LongInt) -- DOSTRUE will cause the boot cli to have a close gadget. 52.8
  DC_BootCliCloseGadgetR = DC_Dummy + 18; // (PLongInt) -- Obtain the state of the boot cli close gadget switch. 52.8
  DC_LocaleBaseW = DC_Dummy + 19; // (PLocaleBase) -- Write the locale base pointer for DOS to access. V54
  DC_LocaleBaseR = DC_Dummy + 20; // (PPLocaleBase) -- Obtain the locale base pointer value from DOS. V54

// Tags for ReadLineItem()
  RLI_Dummy           = TAG_USER + 5500;
  RLI_FileHandleInput = RLI_Dummy + 1;  // (BPTR) -- Read with FGetC() from this filehandle. Default is FGetC(Input());
  RLI_CSourceInput    = RLI_Dummy + 2;  // (PCSource) -- Read from this CSource stream. Default is FGetC(Input());
  RLI_EscapeChars     = RLI_Dummy + 10; // (STRPTR) -- Nul-terminated string of one or more characters to be recognised as escape characters.
                                        // This may be an empty string '' or nil, for none. Defaults to '*'.
  RLI_MultiLine       = RLI_Dummy + 11; // (LongInt; boolean) -- Enable Multi-line processing. Defaults to FALSE.
  RLI_CommentChars    = RLI_Dummy + 12; // (STRPTR) -- Nul-terminated string of one or more characters to be used as a comment marker.
                                        // This may be an empty string '' or nil, for none. Defaults to ''.
  RLI_Substitute_N    = RLI_Dummy + 13; // (LongInt; boolean) -- Substitute quoted escaped 'N' to the hex char $0a.
                                        // Only works when RLI_EscapeChars is not "" or nil. Defaults to TRUE.
  RLI_Substitute_E    = RLI_Dummy + 14; // (LongInt; boolean) -- Substitute quoted escaped 'E' to the hex
                                        // char $1b. Only works when RLI_EscapeChars is not '' or nil. Defaults to TRUE.
// Tags for SetFileHandleAttr()
  FH_Dummy      = TAG_USER + 6000;

  FH_BufferSize = FH_Dummy + 1;
  FH_UserBuffer = FH_Dummy + 2;
  FH_BufferMode = FH_Dummy + 3;
  FH_Locking    = FH_Dummy + 4;
  FH_EndStream  = FH_Dummy + 5;

// Tags for SetOwnerInfo(), GetOwnerInfo().
  OI_Dummy           = TAG_USER + 6200;

  OI_StringNameInput = OI_Dummy + 1;  // (STRPTR) -- Pointer to a string identifying a filesystem object. This tag is mutually exclusive with OI_Process.
  OI_StringName      = OI_StringNameInput;
  OI_ProcessInput    = OI_Dummy + 2;  // (PProcess) -- Pointer to a process structure, or nil for this process, to identify a process. This tag is mutually exclusive with OI_StringName.
  OI_Process         = OI_ProcessInput;
  OI_OwnerUID        = OI_Dummy + 10; // GetOwnerInfo() -- Pointer to a LongInt sized storage area. Returns the value of the objects UID field.
                                      // SetOwnerInfo() -- LongInt value. Sets the value of an objects UID field.
  OI_OwnerGID        = OI_Dummy + 11; // GetOwnerInfo() -- Pointer to a LongInt sized storage area. Returns the value of the objects GID field.
                                      // SetOwnerInfo() -- LongInt value. Sets the value of an objects GID field.
// Tags for TimedDosRequester()
  TDR_Dummy      = TAG_USER + 3500;

  TDR_IDCMP_Ptr  = TDR_Dummy + 1; // (PLongInt) -- [OPTIONAL] Pointer to a longword initialised with the IDCMP flags that you want to terminate the requester on. (Default nil)
  TDR_Timeout    = TDR_Dummy + 2; // (LongInt) -- [OPTIONAL] Value in seconds to wait before requester will close without intervention.  A timeout will only occur with values > 0.
                                 // It is recommended that timouts less than 10 seconds be avoided whenever possible, to be user-friendly. (Default; 0)
  TDR_Window     = TDR_Dummy + 3; // (PWindow) -- [OPTIONAL] A pointer to an (over-riding) reference window pointer. Normally, you would not need to specify this tag.
  TDR_EasyStruct = TDR_Dummy + 4; // (PEasyStruct) [OPTIONAL] -- Pointer to your own private struct EasyStruct that has already been initialised.   (Default; internally allocated)
  TDR_FormatString = TDR_Dummy + 5; // (STRPTR) [OPTIONAL] -- The format string using RawDoFmt style specifiers that match the number of argument array entries.
                                    // If this tag is present, it will override a formatstring that may have already been supplied with a private TDR_EasyStruct.
  TDR_TitleString  = TDR_Dummy + 6; // (STRPTR) [OPTIONAL] -- The title string for the requester. If this tag is present, it will override a title string that may have already been supplied with a private TDR_EasyStruct.
  TDR_GadgetString = TDR_Dummy + 7; // (STRPTR) [OPTIONAL] -- The string spec for the gadgets/text for the requester. If this tag is present, it will override a Gadget string that may have been supplied with a private TDR_EasyStruct.
  TDR_ImageType    = TDR_Dummy + 8; // (LongInt) -- The visual style of this request. This argument has currently no effect before IPrefs is running, but should always
                                    // be specified to indicate the style of request this is directedfor.   (Default; TDRIMAGE_DEFAULT) The values for this tag are defined below.
  TDRIMAGE_DEFAULT    = 0;
  TDRIMAGE_INFO       = 1;
  TDRIMAGE_WARNING    = 2;
  TDRIMAGE_ERROR      = 3;
  TDRIMAGE_QUESTION   = 4;
  TDRIMAGE_INSERTDISK = 5;

  TDR_ArgArray = TDR_Dummy + 9;  // (APTR) [OPTIONAL] -- A pointer to a completely initialised argument array with the number of entries that correspond to the RawDoFmt() format specifiers.
                                 // Do not use this tag if you want to specify up to the first 10 arguments individually with the following TDR_Arg# tags.
  TDR_Inactive = TDR_Dummy + 10; // (LongInt; Boolean) -- Boolean value to specify the requester should not be activated to avoid its stealing the users input focus.
                                 // You should set this to TRUE when the gadgets of your requester contain keyboard shortcuts and its possible that the user is
                                 // currently typing text in another application (otherwise the user could answer the requester by accident). This tag has
                                 // currently no effect before IPrefs is running, keyboard shortcuts also won't work before either.  (Default; FALSE)
  TDR_CharSet  = TDR_Dummy + 11; // (LongWord) -- Character set for the requester's text and gadgets. Defaults to 0, meaning the charset of the screen font which
                                 // will be the current system default charset in most cases. This tag has currently no effect before IPrefs is running.
                                 // The tag value is based on the IANA MIBenum value for charsets. See the autodoc for diskfont.library/ObtainCharsetInfo().
                                 // TDR_CharSet should be set by localized applications that want to display localized requesters, they tell locale.library in
                                 // OpenCatalog() that it shall not convert the catalog charset to current system default charset and use the value of
                                 // Catalog->cat_CodeSet for the TDR_CharSet.    (Default; 0)
  TDR_NonBlocking = TDR_Dummy + 12; // (LongInt; Boolean) -- [OPTIONAL] switch to prevent this function from attempting any disk based I/O whatsoever.
                                    // This makes the function safe to call at any time from handlers and filesystems and in some cases performing dospacket I/O from a task context.
                                    // This tag will effectively bypass the use of any reaction components that may cause blocking issues and re-apply the original dos.library
                                    // code that only calls the intuition library EasyRequest() function, as if it was never patched by "IPrefs". (Default; FALSE)
  TDR_Arg1  = TDR_Dummy+20;
  TDR_Arg2  = TDR_Dummy+21;
  TDR_Arg3  = TDR_Dummy+22;
  TDR_Arg4  = TDR_Dummy+23;
  TDR_Arg5  = TDR_Dummy+24;
  TDR_Arg6  = TDR_Dummy+25;
  TDR_Arg7  = TDR_Dummy+26;
  TDR_Arg8  = TDR_Dummy+27;
  TDR_Arg9  = TDR_Dummy+28;
  TDR_Arg10 = TDR_Dummy+29;

// Tags for FileSystemAttr()
  FSA_Dummy                    = TAG_USER + 9000;
  FSA_StringNameInput          = FSA_Dummy + 1; // (STRPTR) -- Identify the filesystem by this name reference.
  FSA_FileHandleInput          = FSA_Dummy + 2; // (BPTR) -- Identify the filesystem by this FileHandle.
  FSA_LockInput                = FSA_Dummy + 3; // (BPTR) -- Identify the filesystem by this Lock.
  FSA_FileLockInput            = FSA_LockInput;
  FSA_MsgPortInput             = FSA_Dummy + 4;  // (PMsgPort)  Identify the filesystem by this message port. NOTE: This tag was added in dos.library 51.96
  FSA_MaxFileNameLengthR       = FSA_Dummy + 10; // (PLongWord) -- Obtain the maximum filename length. (including \0)
  FSA_MaxFileNameLengthW       = FSA_Dummy + 11; // (LongWord) -- Set the maximum filename length. (including \0)
  FSA_VersionNumberR           = FSA_Dummy + 12; // (PLongWord) -- Obtain the version/rev number for the filesystem.
  FSA_DOSTypeR                 = FSA_Dummy + 13; // (PLongWord) -- Obtain the dostype identifier for the filesystem.
  FSA_ActivityFlushTimeoutR    = FSA_Dummy + 14; // (PLongWord) -- Obtain the mS (1000th second) before a flush while active occurs.
  FSA_ActivityFlushTimeoutW    = FSA_Dummy + 15; // (LongWord) -- Set the mS (1000th second) before a flush while active occurs.
  FSA_InactivityFlushTimeoutR  = FSA_Dummy + 16; // (PLongWord) -- Obtain the mS (1000th second) before a flush when inactive occurs.
  FSA_InactivityFlushTimeoutW  = FSA_Dummy + 17; // (LongWord) -- Set the mS (1000th second) before a flush when inactive occurs.
  FSA_MaxRecycledEntriesR      = FSA_Dummy + 18; // (PLongWord) -- Obtain the number of recycled entries supported.
  FSA_MaxRecycledEntriesW      = FSA_Dummy + 19; // (LongWord) -- Set the number of recycled entries supported.
  FSA_HasRecycledEntriesR      = FSA_Dummy + 20; // (PLongWord) -- Obtain boolean value if the filesystem supports recycled entries.
  FSA_VersionStringR           = FSA_Dummy + 21; // (STRPTR) -- Obtain a copy of the version string for the filesystem.
  FSA_VersionStringR_BufSize   = FSA_Dummy + 22; // (LongWord) -- Sub tag to specify the length of the space provided for FSA_VersionStringR.
  FSA_VersionStringR_Len       = FSA_VersionStringR_BufSize;  // temp compatibility

// Common tags for ExamineObject() and ExamineDir().
  EX_Dummy           = TAG_USER + 6500;
  EX_StringNameInput = EX_Dummy + 1; // (STRPTR) -- Pointer to a nul-terminated string name, to specify the filesystem object required. This may be relative to the current directory an assignment
  EX_StringName      = EX_Dummy + 1; //   or an absolute path.  DOS will internally perform a Lock() on the string provided, it will also Unlock() it again for you. Failure will occur if the object is already exclusively locked.

  EX_FileHandleInput = EX_Dummy + 2; // (BPTR) -- BCPL pointer to a FileHandle. Identify the parent directory from the stream pointed to by an open FileHandle.  DOS determins the parent directory of the
  EX_FileHandle      = EX_Dummy + 2; //   file by calling ParentOfFH() internally. The internal lock is UnLock'ed when the caller invokes ReleaseDirContext().

  EX_LockInput       = EX_Dummy + 3; // (BPTR) -- BCPL pointer to a Lock.
  EX_FileLockInput   = EX_Dummy + 3; //   Identify the filesystem object by this associated Lock.
  EX_DirLockInput    = EX_Dummy + 3; //   This lock is always passed directly throught to the filesystem.
  EX_FileLock        = EX_Dummy + 3;
  EX_DirLock         = EX_Dummy + 3;

 //The following are specific only for use with ObtainDirContext() when preparing a context for ExamineDir()
   EX_DataFields     = EX_Dummy + 10; // (LongWord) -- A bitmask to indicate which additional items of data are to be filled in your ExamineData structure, by the filesystem.
   EX_MatchFunc      = EX_Dummy + 11; // (PHook) -- Pointer to a custom Hook structure which has the h_Entry function pointer set to a filtering function. (Default = nil) for the internal filter to be used instead.
   EX_MatchString    = EX_Dummy + 12; // (STRPTR) -- A pointer to a pre-parsed pattern string. (Default=NULL) If this field is NULL then all entries will be returned from the internal filter function.
   EX_DoCurrentDir   = EX_Dummy + 13; // (BOOLEAN) -- Set TRUE if you wish DOS to make the reference to the specified directory the current directory.
                                      // This will allow for recursive directory scans to nominate the sub-directory by name, without having to specifying the full path. (Default=FALSE)
   EX_ResetContext   = EX_Dummy + 14; // (APTR) -- Pointer to a previous context for which a change of any parameter/s are required.  By passing in a previous context via this tag, the resources already allocated for the old context and
                                      // ExamineData nodes will be reused.  All necessary filesystem context information will also be reset to allow a new scan to begin again. (Default = nil)

function DosObtain(): LongWord; syscall IDos 60;
function DosRelease(): LongWord; syscall IDos 64;
procedure DosExpunge(); syscall IDos 68;
function DosClone(): PInterface; syscall IDos 72;
function DosOpen(const Name: STRPTR; AccessMode: LongInt): BPTR; syscall IDos 76;
procedure DOSClose(File_: BPTR); syscall IDos 80;
function DosRead(File_: BPTR; Buffer: APTR; Length: LongInt): LongInt; syscall IDos 84;
function DosWrite(File_: BPTR; Buffer: APTR; Length: LongInt): LongInt; syscall IDos 88;
function DosInput: BPTR; syscall IDos 92;
function DosOutput: BPTR; syscall IDos 96;
function DosSeek(File_: BPTR; Position: LongInt; Offset: LongInt): LongInt; syscall IDos 100;
function DosDeleteFile(const Name: STRPTR) : LongBool; overload; syscall IDos 104;  // compatibility to older platforms
function DosDelete(const Name: STRPTR) : LongBool; overload; syscall IDos 104;
function DosRename(const OldName: STRPTR; const NewName: STRPTR): LongBool; syscall IDos 108;
function Lock(const Name: STRPTR; Type_: LongInt): BPTR; syscall IDos 112;
procedure UnLock(Lock: BPTR); syscall IDos 116;
function DupLock(Lock: BPTR): BPTR; syscall IDos 120;
function Examine(Lock: BPTR; FileInfoBlock: PFileInfoBlock): LongBool; syscall IDos 124;
function ExNext(Lock: BPTR; FileInfoBlock: PFileInfoBlock): LongBool; syscall IDos 128;
function Info(Lock: BPTR; ParameterBlock: PInfoData): LongBool; syscall IDos 132;
function CreateDir(const Name: STRPTR) : BPTR; syscall IDos 136;
function CurrentDir(Lock: BPTR): BPTR; syscall IDos 140; // compatibility to older platforms
function SetCurrentDir(Lock: BPTR): BPTR; syscall IDos 140;
function IoErr: LongInt; syscall IDos 144;
function CreateProc(const Name: STRPTR; Pri: LongInt; SegList: BPTR; StackSize: LongInt): PMsgPort; syscall IDos 148;
// obsolete - procedure DosExit(ReturnCode: LongInt); syscall IDos 152;
// obsolete - function LoadSeg(const Name: STRPTR): LongInt; syscall IDos 156;
procedure UnLoadSeg(Seglist: BPTR); syscall IDos 160;
// private - function DoPkt32(Port: PMsgPort; Action, Arg1, Arg2, Arg3, Arg4, Arg5, ): LongInt; syscall IDos 164;
function LoadSeg(const Name: STRPTR): LongInt; syscall IDos 168;
function DeviceProc(const Name: STRPTR): PMsgPort; syscall IDos 172;
function SetComment(const Name: STRPTR; const Comment: STRPTR): LongBool; syscall IDos 176;
function SetProtection(const Name: STRPTR; Protect: LongInt): LongBool; syscall IDos 180;
function DateStamp(Date: PDateStamp): PDateStamp; syscall IDos 184;
procedure DosDelay(TimeOut: LongInt); syscall IDos 188;
function WaitForChar(File_: BPTR; TimeOut: LongInt): LongBool; syscall IDos 192;
function ParentDir(Lock: BPTR): BPTR; syscall IDos 196;
function IsInteractive(File_: BPTR): LongBool; syscall IDos 200;
function Execute(const String_: STRPTR; File_: BPTR; File2: BPTR): LongBool; syscall IDos 204;
function AllocDosObject(Type_: LongWord; const Tags: PTagItem): APTR; syscall IDos 208;
function AllocDosObjectTagList(Type_: LongWord; const Tags: PTagItem): APTR; syscall IDos 212;
// 216 AllocDosObjectTags
procedure FreeDosObject(Type_: LongWord; Ptr: APTR); syscall IDos 220;
function DoPkt(Port: PMsgPort; Action, Arg1, Arg2, Arg3, Arg4, Arg5: LongInt): LongInt; syscall IDos 224;
function DoPkt0(Port: PMsgPort; Action: LongInt): LongInt; syscall IDos 228;
function DoPkt1(Port: PMsgPort; Action, Arg1: LongInt): LongInt; syscall IDos 232;
function DoPkt2(Port: PMsgPort; Action, Arg1, Arg2: LongInt): LongInt; syscall IDos 236;
function DoPkt3(Port: PMsgPort; Action, Arg1, Arg2, Arg3: LongInt): LongInt; syscall IDos 240;
function DoPkt4(Port: PMsgPort; Action, Arg1, Arg2, Arg3, Arg4: LongInt): LongInt; syscall IDos 244;
procedure SendPkt(Dp: PDosPacket; Port: PMsgPort; ReplyPort: PMsgPort); syscall IDos 248;
function WaitPkt: PDosPacket; syscall IDos 252;
procedure ReplyPkt(Dp: PDosPacket; Res1, Res2: LongInt); syscall IDos 256;
// 260 Reserved
// obsolete LockRecord 264
function LockRecords(RecArray: PRecordLock; TimeOut: LongWord): LongBool; syscall IDos 268;
// obsolete UnLockRecord 272
function UnLockRecords(RecArray: PRecordLock): LongBool; syscall IDos 276;
function SelectInput(Fh: BPTR): BPTR; syscall IDos 280;
function SelectOutput(Fh: BPTR): BPTR; syscall IDos 284;
function FGetC(Fh: BPTR): LongInt; syscall IDos 288;
function FPutC(Fh: BPTR; Ch: LongInt): LongInt; syscall IDos 292;
function UnGetC(Fh: BPTR; Character: LongInt): LongInt; syscall IDos 296;
function FRead(Fh: BPTR; Block: APTR; BlockLen: LongWord; Number: LongWord): LongWord; syscall IDos 300;
function FWrite(Fh: BPTR; Block: APTR; BlockLen: LongWord; Number: LongWord): LongWord; syscall IDos 304;
function FGets(Fh: BPTR; Buf: STRPTR; BufLen: LongWord): STRPTR; syscall IDos 308;
function FPuts(Fh: BPTR; const Str: STRPTR): LongInt; syscall IDos 312;
// obsolete VFWritef 316
// obsolete FWritef 320
function VFPrintf(Fh: BPTR; const Format: STRPTR; const ArgArray: APTR): LongInt; syscall IDos 324;
// 328 FPrintf
function DosFlush(Fh: BPTR): LongBool; syscall IDos 332; // compatibility to older platforms
function FFlush(Fh: BPTR): LongBool; syscall IDos 332;
function SetVBuf(Fh: BPTR; Buff: STRPTR; Type_: LongInt; Size: LongInt): LongBool; syscall IDos 336;
function DupLockFromFH(Fh: BPTR): BPTR; syscall IDos 340;
function OpenFromLock(Lock: BPTR): BPTR; syscall IDos 344;
function ParentOfFH(Fh: BPTR): BPTR; syscall IDos 348;
function ExamineFH(Fh: BPTR; Fib: PFileInfoBlock): LongBool; syscall IDos 352;
function SetDate(const Name: STRPTR; const Date: PDateStamp): LongBool; syscall IDos 356;
function NameFromLock(Lock: BPTR; Buffer: STRPTR; Len: LongInt): LongBool; syscall IDos 360;
function NameFromFH(Fh: BPTR; Buffer: STRPTR; Len: LongInt): LongBool; syscall IDos 364;
function SplitName(const Name: STRPTR; Seperator: LongWord; Buf: STRPTR; OldPos: LongInt; Size: LongInt): LongInt; syscall IDos 368;
function SameLock(Lock1, Lock2: BPTR): LongInt; syscall IDos 372;
function SetMode(Fh: BPTR; Mode: LongInt): LongBool; syscall IDos 376;
function ExAll(Lock: BPTR; Buffer: PExAllData; Size, Data: LongInt; Control: PExAllControl): LongBool; syscall IDos 380;
function ReadSoftLink(Port: PMsgPort; Lock: BPTR; const Path: STRPTR; Size: LongWord): LongInt; syscall IDos 384;
function MakeLink(const Name: STRPTR; Dest: APTR; soft: LongInt): LongBool; syscall IDos 388;
function ChangeMode(Type_ : LongInt; Fh: BPTR; NewMode: LongInt): LongBool; syscall IDos 392;
function SetFileSize(Fh: BPTR; Pos: LongInt; Mode: LongInt): LongBool; syscall IDos 396;
function SetIoErr(Result: LongInt): LongInt; syscall IDos 400;
function Fault(Code: LongInt; const Header: STRPTR; Buffer: STRPTR; Len: LongInt): LongBool; syscall IDos 404;
function PrintFault(Code: LongInt; const Header: STRPTR): LongBool; syscall IDos 408;
function ErrorReport(Code, Type_: LongInt; Arg1: LongWord; Device: PMsgPort): LongBool; syscall IDos 412;
// private Requester 416
function Cli: PCommandLineInterface; syscall IDos 420;
function CreateNewProc(const Tags: PTagItem): PProcess; syscall IDos 424;
function CreateNewProcTagList(const Tags: PTagItem): PProcess; syscall IDos 428;
// 432 CreateNewProcTags
function RunCommand(Seg: BPTR; Stack: LongWord; const ParamPtr: STRPTR; ParamLen: LongInt): LongInt; syscall IDos 436;
function GetConsolePort(): PMsgPort; syscall IDos 440;
function SetConsolePort(const Port: PMsgPort): PMsgPort; syscall IDos 444;
function GetFileSysPort(): PMsgPort; syscall IDos 448;
function SetFileSysPort(const Port: PMsgPort): PMsgPort; syscall IDos 452;
function GetArgStr: STRPTR; syscall IDos 456;
function SetArgStr(const String_: STRPTR): STRPTR; syscall IDos 460;
function FindCliProc(Num: LongWord): PProcess; syscall IDos 464;
function MaxCli: LongWord; syscall IDos 468;
function SetCliCurrentDirName(const Name: STRPTR): LongInt; syscall IDos 472;
function GetCliCurrentDirName(Buf: STRPTR; Len: LongInt): LongInt; syscall IDos 476;
function SetCliProgramName(const Name: STRPTR): LongInt; syscall IDos 480;
function GetCliProgramName(Buf: STRPTR; Len: LongInt): LongInt; syscall IDos 484;
function SetCliPrompt(const Name: STRPTR): LongInt; syscall IDos 488;
function GetCliPrompt(Buf: STRPTR; Len: LongInt): LongInt; syscall IDos 492;
function SetProgramDir(Lock: BPTR): BPTR; syscall IDos 496;
function GetProgramDir(): BPTR; syscall IDos 500;
function SystemTagList(const Command: STRPTR; const Tags: PTagItem): LongInt; syscall IDos 504;
function DosSystem(const Command: STRPTR; const Tags: PTagItem): LongInt; syscall IDos 508;
// 512 SystemTags
function AssignLock(const Name: STRPTR; Lock: BPTR): LongBool; syscall IDos 516;
function AssignLate(const Name: STRPTR; const Path: STRPTR): LongBool; syscall IDos 520;
function AssignPath(const Name: STRPTR; const Path: STRPTR) : LongBool; syscall IDos 524;
function AssignAdd(const Name: STRPTR; Lock: BPTR): LongBool; syscall IDos 528;
function RemAssignList(const Name: STRPTR; Lock: BPTR): LongBool; syscall IDos 532;
function GetDeviceProc(const Name: STRPTR; Dp: PDevProc): pDevProc; syscall IDos 536;
procedure FreeDeviceProc(Dp: PDevProc); syscall IDos 540;
function LockDosList(Flags: LongWord): PDosList; syscall IDos 544;
procedure UnLockDosList(Flags: LongWord); syscall IDos 548;
function AttemptLockDosList(Flags: LongWord): PDosList; syscall IDos 552;
function RemDosEntry(DList: PDosList): LongBool; syscall IDos 556;
function AddDosEntry(DList: PDosList): LongBool; syscall IDos 560;
function FindDosEntry(const DList: PDosList; const Name: STRPTR; Flags: LongWord): PDosList; syscall IDos 564;
function NextDosEntry(const DList: PDosList; Flags: LongWord) : PDosList; syscall IDos 568;
function MakeDosEntry(const Name: STRPTR; Type_: LongInt): PDosList; syscall IDos 572;
procedure FreeDosEntry(DList: PDosList); syscall IDos 576;
function IsFileSystem(const Name: STRPTR): LongBool; syscall IDos 580;
function Format(const Filesystem: STRPTR; const VolumeName: STRPTR; DosType: LongWord): LongBool; syscall IDos 584;
function Relabel(const Drive: STRPTR; const NewName: STRPTR): LongBool; syscall IDos 588;
function Inhibit(const Name: STRPTR; OnOff: LongInt): LongBool; syscall IDos 592;
function AddBuffers(const Name: STRPTR; Number: LongInt): LongBool; syscall IDos 596;
function CompareDates(const Date1: PDateStamp; const Date2: PDateStamp): LongInt; syscall IDos 600;
function DosDateToStr(DateTime: _PDateTime): LongBool; syscall IDos 604;
function DosStrToDate(DateTime: _PDateTime): LongBool; syscall IDos 608;
// obsolete InternalLoadSeg 612
// obsolete InternalUnLoadSeg 616
// obsolete NewLoadSeg 620
// obsolete NewLoadSegTagList 624
// obsolete NewLoadSegTags 628
function AddSegment(const Name: STRPTR; Seg: BPTR; Type_: LongInt): LongBool; syscall IDos 632;
function FindSegment(const Name: STRPTR; const Seg: PDosResidentSeg; Sys: LongInt): PDosResidentSeg; syscall IDos 636;
function RemSegment(Seg: PDosResidentSeg): LongBool; syscall IDos 640;
function CheckSignal(Mask: LongWord): LongWord; syscall IDos 644;
function ReadArgs(const ArgTemplate: STRPTR; Array_: PLongInt; Args: PRDArgs): PRDArgs; syscall IDos 648;
function FindArg(const KeyWord: STRPTR; const ArgTemplate: STRPTR): LongInt; syscall IDos 652;
function ReadItem(const Name: STRPTR; MaxChars: LongInt; CSource: PCSource): LongInt; syscall IDos 656;
function StrToLong(const String_: STRPTR; var Value: LongInt): LongInt; syscall IDos 660;
function MatchFirst(const Pat: STRPTR; Anchor: PAnchorPath): LongInt; syscall IDos 664;
function MatchNext(Anchor: PAnchorPath): LongInt; syscall IDos 668;
procedure MatchEnd(Anchor: PAnchorPath); syscall IDos 672;
function ParsePattern(const Pat: STRPTR; Buf: STRPTR; BufLen: LongInt): LongInt; syscall IDos 676;
function MatchPattern(const Pat: STRPTR; Str: STRPTR): LongBool; syscall IDos 680;
// 684 reserved
procedure FreeArgs(Args: PRDArgs); syscall IDos 688;
// 692 reserved
function FilePart(const Path: STRPTR): STRPTR; syscall IDos 696;
function PathPart(const Path: STRPTR): STRPTR; syscall IDos 700;
function AddPart(DestDirName: STRPTR; const FileName: STRPTR; Size: LongWord): LongBool; syscall IDos 704;
function StartNotify(Notify: PNotifyRequest): LongBool; syscall IDos 708;
procedure EndNotify(Notify: PNotifyRequest); syscall IDos 712;
function SetVar(const Name: STRPTR; Buffer: STRPTR; Size: LongInt; Flags: LongWord): LongBool; syscall IDos 716;
function GetVar(const Name: STRPTR; Buffer: STRPTR; Size: LongInt; Flags: LongInt): LongInt; syscall IDos 720;
function DeleteVar(const Name: STRPTR; Flags: LongWord): LongBool; syscall IDos 724;
function FindVar(const Name: STRPTR; Type_: LongWord): PLocalVar; syscall IDos 728;
// private CLIInit 732
function CliInitNewcli(Dp: PDosPacket): LongInt; syscall IDos 736;
function CliInitRun(Dp: PDosPacket): LongInt; syscall IDos 740;
function WriteChars(const Buf: STRPTR; BufLen: LongWord): LongInt; syscall IDos 744;
function PutStr(const Str: STRPTR): LongInt; syscall IDos 748;
function VPrintf(const Format: STRPTR; const ArgArray: PLongInt): LongInt; syscall IDos 752;
// 756 Printf
// 760 reserved
function ParsePatternNoCase(const Pat: STRPTR; Buf: STRPTR; BufLen: LongInt): LongInt; syscall IDos 764;
function MatchPatternNoCase(const Pat: STRPTR; Str: STRPTR): LongBool; syscall IDos 768;
// private DosGetString 772
function SameDevice(Lock1, Lock2: BPTR): LongBool; syscall IDos 776;
procedure ExAllEnd(Lock: BPTR; Buffer: PExAllData; Size, data: LongInt; Control: PExAllControl); syscall IDos 780;
function SetOwner(const Name: STRPTR; OwnerInfo: LongInt): LongBool; syscall IDos 784;
function GetEntryData(): LongInt; syscall IDos 788;
function ReadLineItem(Buffer: STRPTR; MaxChars: LongInt; TagList: PTagItem): LongInt; syscall IDos 792;
// 796 ReadLineItemTags
// private InternalRunCommand 800
function GetCurrentDir(): BPTR; syscall IDos 804;
function NonBlockingModifyDosEntry(Dl: PDosList; Mode: LongInt; Arg1, Arg2: APTR): LongInt; syscall IDos 808;
function SecondsToDateStamp(Seconds: LongWord; Ds: PDateStamp): PDateStamp; syscall IDos 812;
function DateStampToSeconds(const Ds: PDateStamp): LongWord; syscall IDos 816;
function FixDateStamp(Ds: PDateStamp): LongInt; syscall IDos 820;
function AddDates(To_: PDateStamp; const From_: PDateStamp): LongInt; syscall IDos 824;
function SubtractDates(To_: PDateStamp; const From_: PDateStamp): LongInt; syscall IDos 828;
function AddSegmentTagList(const Name: STRPTR; Type_: LongInt; const Tags: PTagItem): LongInt; syscall IDos 832;
function ParseCapturePattern(const Pat: STRPTR; Dst: STRPTR; Length: LongInt; CaseSen: LongInt): LongInt; syscall IDos 836;
function CapturePattern(const Pat: STRPTR; const Str: STRPTR; CaseSen: LongInt; var Cap: PCapturedExpression): LongInt; syscall IDos 840;
procedure ReleaseCapturedExpressions(First: PCapturedExpression); syscall IDos 844;
procedure FindTrackedAddress(const Address: APTR; Hook: PHook); syscall IDos 848;
function TrackAddressList(const Name: STRPTR; Segment: BPTR; const ExtraInfo: APTR; ExtraInfoSize: LongInt; const Aas: PAddressAndSize; NumPairs: LongInt): LongInt; syscall IDos 852;
function TrackSegmentList(const Name: STRPTR; Segment: BPTR; const ExtraInfo: APTR; ExtraInfoSize: LongInt): LongInt; syscall IDos 856;
procedure UnTrackAddress(Address: APTR); syscall IDos 860;
procedure UnTrackSegmentList(Segment: BPTR); syscall IDos 864;
function GetExitData(): LongInt; syscall IDos 868;
function PutErrStr(const str: STRPTR): LongInt; syscall IDos 872;
function ErrorOutput(): BPTR; syscall IDos 876;
function SelectErrorOutput(Fh: BPTR): BPTR; syscall IDos 880;
function MountDevice(const Name: STRPTR; Type_: LongInt; Tags: PTagItem): LongInt; syscall IDos 884;
// 888 MountDeviceTags
function SetProcWindow(const Win: APTR): APTR; syscall IDos 892;
function FindSegmentStackSize(Segment: BPTR): LongWord; syscall IDos 896;
function CalculateSegmentChecksum(Segment: BPTR): LongWord; syscall IDos 900;
function AllocSegList(const Entry: APTR; const Data: APTR; DataLen: LongWord; IdentKey: LongWord): BPTR; syscall IDos 904;
function GetSegListInfo(SegList: BPTR; const Tags: PTagItem): LongInt; syscall IDos 908;
// 912 GetSegListInfoTags
function AddSegListTail(BSegList: BPTR; BSegNew: BPTR): LongInt; syscall IDos 916;
function DevNameFromLock(Lock: BPTR; Buffer: STRPTR; BufLen: LongInt; Mode: LongInt): LongInt; syscall IDos 920;
function GetProcMsgPort(const Proc: PProcess): PMsgPort; syscall IDos 924;
function WaitForData(Stream: BPTR; DataDirection: LongInt; TimeOut: LongInt): LongInt; syscall IDos 928;
function SetBlockingMode(Stream: BPTR; NewBlocking: LongInt): LongInt; syscall IDos 932;
function SetCurrentCmdPathList(Pn: PPathNode): PPathNode; syscall IDos 936;
function AllocateCmdPathList(FirstLock: BPTR): PPathNode; syscall IDos 940;
procedure FreeCmdPathList(Pn: PPathNode); syscall IDos 944;
function RemoveCmdPathNode(Pn: PPathNode; Lock: BPTR): PPathNode; syscall IDos 948;
function AddCmdPathNode(Pn: PPathNode; Lock: BPTR; Where: LongInt): PPathNode; syscall IDos 952;
function SearchCmdPathList(Pn: PPathNode; h: PHook; const Name: STRPTR; const Tags: PTagItem): LongInt; syscall IDos 956;
// 960 SearchCmdPathListTags
function ScanVars(Hook: PHook; Flags: LongWord; const UserData: APTR): LongInt; syscall IDos 964;
function GetProcSegList(const Proc: PProcess; Flags: LongWord): BPTR; syscall IDos 968;
function HexToLong(const String_: STRPTR; ValuePtr: PLongWord): LongInt; syscall IDos 972;
function GetDeviceProcFlags(const Name: STRPTR; Odp: PDevProc; Flags: LongWord): PDevProc; syscall IDos 976;
function DosControl(const Tags: PTagItem): LongInt; syscall IDos 980;
// 984 DosControlTags
function CreateDirTree(const Name: STRPTR): BPTR; syscall IDos 988;
function NotifyVar(const Name: STRPTR; Hook: PHook; Flags: LongInt; const UserData: APTR): LongInt; syscall IDos 992;
function GetDiskFileSystemData(const Name: STRPTR): PFileSystemData; syscall IDos 996;
procedure FreeDiskFileSystemData(Fsd: PFileSystemData); syscall IDos 1000;
function FOpen(const Name: STRPTR; Mode: LongInt; BufSize: LongInt): BPTR; syscall IDos 1004;
function FClose(Scb: BPTR): LongInt; syscall IDos 1008;
function FOpenFromLock(Lock: BPTR; BufSize: LongInt): BPTR; syscall IDos 1012;
// 1016 reserved
function TimedDosRequester(const Tags: PTagITem): LongInt; syscall IDos 1020;
// 1024 TimedDosRequesterTags
function RenameDosEntry(const DList: PDosList; const NewName: STRPTR): LongInt; syscall IDos 1028;
function DismountDevice(const Name: STRPTR; Flags: LongWord; Reserved: APTR): LongInt; syscall IDos 1032;
function DupFileHandle(Scb: BPTR): BPTR; syscall IDos 1036;
function DevNameFromFH(Scb: BPTR; Buffer: STRPTR; BufLen: LongInt; Mode: LongInt): LongInt; syscall IDos 1040;
function AssignAddToList(const Name: STRPTR; Lock: BPTR; EndPos: LongInt): LongInt; syscall IDos 1044;
function SetFileHandleAttr(Fh: BPTR; const Tags: PTagItem): LongInt; syscall IDos 1048;
// 1052 SetFileHandleAttrTags
function FileSystemAttr(const Tags: PTagItem): LongInt; syscall IDos 1056;
// 1060 FileSystemAttrTags
function FReadLine(Fh: BPTR; Frld: PFReadLineData): LongInt; syscall IDos 1064;
function CopyStringBSTRToC(BSrc: BSTR; Dst: STRPTR; Size: LongWord): LongWord; syscall IDos 1068;
function CopyStringCToBSTR(const Src: STRPTR; BDest: BSTR; Size: LongWord): LongWord; syscall IDos 1072;
function GetFilePosition(Fh: BPTR): Int64; syscall IDos 1076;
function ChangeFilePosition(Fh: BPTR; Position: Int64; Offset: LongInt): LongInt; syscall IDos 1080;
function ChangeFileSize(Fh: BPTR; Pos: Int64; Mode: LongInt): LongInt; syscall IDos 1084;
function GetFileSize(Fh: BPTR): Int64; syscall IDos 1088;
// 1092 reserved
// 1096 Private DoPkt64
// 1100 reserved
// 1104 reserved
// 1108 reserved
function ProcessScan(Hook: PHook; const UserData: APTR; Reserved: LongWord): LongInt; syscall IDos 1112;
function NotifyDosListChange(Process: PProcess; SignalNum, Reserved: LongWord): LongInt; syscall IDos 1116;
function NotifyProcListChange(Process: PProcess; SignalNum, Reserved: LongWord): LongInt; syscall IDos 1120;
function GetDiskInfo(const Tags: PTagItem): LongInt; syscall IDos 1124;
// 1128 GetDiskInfoTags
function WriteProtectVolume(const Name: STRPTR; OnOff: LongInt; PassKey: LongWord; Reserved: LongWord): LongInt; syscall IDos 1132;
function ExamineObject(const CTags: PTagItem): PExamineData; syscall IDos 1136;
// 1140 ExamineObjectTags
function ExamineDir(Context: APTR): PExamineData; syscall IDos 1144;
function ObtainDirContext(const CTags: PTagItem): APTR; syscall IDos 1148;
// 1152 ObtainDirContextTags
procedure ReleaseDirContext(Contx: APTR); syscall IDos 1156;
function GetOwnerInfo(const Tags: PTagItem): APTR; syscall IDos 1160;
// 1164 GetOwnerInfoTags
function SetOwnerInfo(const Tags: PTagItem): LongInt; syscall IDos 1168;
// 1172 SetOwnerInfoTags
function LockTagList(const Tags: PTagItem): BPTR; syscall IDos 1176;
// 1180 LockTags
function GetPID(Process: PProcess): LongWord; syscall IDos 1184;
function FlushVolume(const Name: STRPTR): LongInt; syscall IDos 1188;
// 1192 obsolete ConsoleData
// 1196 obsolete ConsoleDataTags
procedure ReleaseConsoleData(Data: PConsoleWindowData); syscall IDos 1200;
function Serialize(const Name: STRPTR): LongInt; syscall IDos 1204;
function NameFromPort(Port: PMsgPort; Buffer: STRPTR; BufLen: LongInt; AddColon: LongInt): LongInt; syscall IDos 1208;
function DevNameFromPort(Port: PMsgPort; Buffer: STRPTR; BufLen: LongInt; AddColon: LongInt): LongInt; syscall IDos 1212;
function SameFH(Scb1, Scb2: BPTR): LongInt; syscall IDos 1216;
function LockRecord(Fh: BPTR; Offset, Length: Int64; Mode: LongWord; TimeOut: LongWord): LongBool; syscall IDos 1220;
function UnLockRecord(Fh: BPTR; Offset, Length: Int64): LongBool; syscall IDos 1224;
function IsFileSystemPort(Port: PMsgPort): LongBool; syscall IDos 1228;
function InhibitPort(Port: PMsgPort; State: LongInt): LongBool; syscall IDos 1232;
function FormatPort(Port: PMsgPort; const VolumeName: STRPTR; DosType: LongInt): LongBool; syscall IDos 1236;
function SerializePort(Port: PMsgPort): LongBool; syscall IDos 1240;
function FlushVolumePort(Port: PMsgPort): LongBool; syscall IDos 1244;
function FileHandleScan(const Hook: PHook; const UserData: APTR; Reserved: LongWord): LongInt; syscall IDos 1248;
function GetFileSystemVectorPort(Port: PMsgPort; MinVersion: LongWord): PFileSystemVectorPort; syscall IDos 1252;
function ResolvePath(Port: PMsgPort; const Path: STRPTR; InLock: BPTR; Out_: STRPTR; OutLen: LongWord; var OutDir: BPTR; var SLCount: LongWord): LongInt; syscall IDos 1256;
function WriteProtectVolumePort(Port: PMsgPort; OnOff: LongInt; PassKey: LongWord; Reserved: LongInt): LongInt; syscall IDos 1260;
// 1264 - 1280 Reserved


function BADDR(bval :BPTR): POINTER;
function MKBADDR(adr: Pointer): BPTR;

// var args version
function AllocDosObjectTags(type_ : LongWord; Const argv : Array of PtrUInt) : POINTER;
function CreateNewProcTags(Const argv : Array of PtrUInt) : pProcess;
function SystemTags(command : PChar; Const argv : Array of PtrUInt) : LongInt;



implementation


function BADDR(bval : BPTR): POINTER; inline;
BEGIN
  BADDR := POINTER( bval shl 2);
END;

function MKBADDR(adr : POINTER): BPTR; inline;
BEGIN
  MKBADDR := BPTR(LongWord(adr) shr 2);
END;

function AllocDosObjectTags(type_ : LongWord; Const argv : Array of PtrUInt) : POINTER;
begin
  AllocDosObjectTags := AllocDosObjectTagList(type_, @argv);
end;

function CreateNewProcTags(Const argv : Array of PtrUInt) : pProcess;
begin
  CreateNewProcTags := CreateNewProcTagList(@argv);
end;

function SystemTags(command : PChar; Const argv : Array of PtrUInt) : LongInt;
begin
  SystemTags := SystemTagList(command, @argv);
end;

end.


