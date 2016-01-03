{%MainUnit ndk.pas}
{
    Native Development Kit for Native NT

    This file is part of the Free Pascal run time library.
    This unit contains types used for IO functions.
    Copyright (c) 2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  //
  // NtCreateFile OpenType Flags
  //
  FILE_SUPERSEDE                          = $00000000;
  FILE_OPEN                               = $00000001;
  FILE_CREATE                             = $00000002;
  FILE_OPEN_IF                            = $00000003;
  FILE_OVERWRITE                          = $00000004;
  FILE_OVERWRITE_IF                       = $00000005;
  FILE_MAXIMUM_DISPOSITION                = $00000005;

  //
  // NtCreateFile Flags
  //
  FILE_DIRECTORY_FILE                     = $00000001;
  FILE_WRITE_THROUGH                      = $00000002;
  FILE_SEQUENTIAL_ONLY                    = $00000004;
  FILE_NO_INTERMEDIATE_BUFFERING          = $00000008;
  FILE_SYNCHRONOUS_IO_ALERT               = $00000010;
  FILE_SYNCHRONOUS_IO_NONALERT            = $00000020;
  FILE_NON_DIRECTORY_FILE                 = $00000040;
  FILE_CREATE_TREE_CONNECTION             = $00000080;
  FILE_COMPLETE_IF_OPLOCKED               = $00000100;
  FILE_NO_EA_KNOWLEDGE                    = $00000200;
  FILE_OPEN_REMOTE_INSTANCE               = $00000400;
  FILE_RANDOM_ACCESS                      = $00000800;
  FILE_DELETE_ON_CLOSE                    = $00001000;
  FILE_OPEN_BY_FILE_ID                    = $00002000;
  FILE_OPEN_FOR_BACKUP_INTENT             = $00004000;
  FILE_NO_COMPRESSION                     = $00008000;
  FILE_RESERVE_OPFILTER                   = $00100000;
  FILE_OPEN_REPARSE_POINT                 = $00200000;
  FILE_OPEN_NO_RECALL                     = $00400000;
  FILE_OPEN_FOR_FREE_SPACE_QUERY          = $00800000;

type
  //
  // File Information Classes for NtQueryInformationFile
  //
  _FILE_INFORMATION_CLASS = (
    FileDirectoryInformation = 1,
    FileFullDirectoryInformation,
    FileBothDirectoryInformation,
    FileBasicInformation,
    FileStandardInformation,
    FileInternalInformation,
    FileEaInformation,
    FileAccessInformation,
    FileNameInformation,
    FileRenameInformation,
    FileLinkInformation,
    FileNamesInformation,
    FileDispositionInformation,
    FilePositionInformation,
    FileFullEaInformation,
    FileModeInformation,
    FileAlignmentInformation,
    FileAllInformation,
    FileAllocationInformation,
    FileEndOfFileInformation,
    FileAlternateNameInformation,
    FileStreamInformation,
    FilePipeInformation,
    FilePipeLocalInformation,
    FilePipeRemoteInformation,
    FileMailslotQueryInformation,
    FileMailslotSetInformation,
    FileCompressionInformation,
    FileObjectIdInformation,
    FileCompletionInformation,
    FileMoveClusterInformation,
    FileQuotaInformation,
    FileReparsePointInformation,
    FileNetworkOpenInformation,
    FileAttributeTagInformation,
    FileTrackingInformation,
    FileIdBothDirectoryInformation,
    FileIdFullDirectoryInformation,
    FileValidDataLengthInformation,
    FileShortNameInformation,
    FileMaximumInformation
  );
  FILE_INFORMATION_CLASS = _FILE_INFORMATION_CLASS;
  PFILE_INFORMATION_CLASS = ^FILE_INFORMATION_CLASS;

  //
  // I/O Status Block
  //
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _IO_STATUS_BLOCK = record
    union1: record
=======
  _IO_STATUS_BLOCK = packed record
    union1: packed record
>>>>>>> graemeg/cpstrnew
=======
  _IO_STATUS_BLOCK = packed record
    union1: packed record
>>>>>>> graemeg/cpstrnew
=======
  _IO_STATUS_BLOCK = packed record
    union1: packed record
>>>>>>> graemeg/cpstrnew
=======
  _IO_STATUS_BLOCK = packed record
    union1: packed record
>>>>>>> origin/cpstrnew
      case Boolean of
        True: (Status: NTSTATUS);
        False: (Pointer: PVOID);
    end;
    Information: ULONG_PTR;
  end;
  IO_STATUS_BLOCK = _IO_STATUS_BLOCK;
  PIO_STATUS_BLOCK = ^IO_STATUS_BLOCK;

  //
  // File Information structures for NtQueryInformationFile
  //
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_BASIC_INFORMATION = record
=======
  _FILE_BASIC_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_BASIC_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_BASIC_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_BASIC_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    CreationTime: LARGE_INTEGER;
    LastAccessTime: LARGE_INTEGER;
    LastWriteTime: LARGE_INTEGER;
    ChangeTime: LARGE_INTEGER;
    FileAttributes: ULONG;
  end;
  FILE_BASIC_INFORMATION = _FILE_BASIC_INFORMATION;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  PFILE_BASIC_INFORMATION = ^FILE_BASIC_INFORMATION;

  _FILE_STANDARD_INFORMATION = record
=======
  PFILE_BASIC_INFORMATION = FILE_BASIC_INFORMATION;

  _FILE_STANDARD_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  PFILE_BASIC_INFORMATION = FILE_BASIC_INFORMATION;

  _FILE_STANDARD_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  PFILE_BASIC_INFORMATION = FILE_BASIC_INFORMATION;

  _FILE_STANDARD_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  PFILE_BASIC_INFORMATION = FILE_BASIC_INFORMATION;

  _FILE_STANDARD_INFORMATION = packed record
>>>>>>> origin/cpstrnew
      AllocationSize: LARGE_INTEGER;
      EndOfFile: LARGE_INTEGER;
      NumberOfLinks: ULONG;
      DeletePending: NT_BOOLEAN;
      Directory: NT_BOOLEAN;
  end;
  FILE_STANDARD_INFORMATION = _FILE_STANDARD_INFORMATION;
  PFILE_STANDARD_INFORMATION = ^FILE_STANDARD_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_NETWORK_OPEN_INFORMATION = record
=======
  _FILE_NETWORK_OPEN_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_NETWORK_OPEN_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_NETWORK_OPEN_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_NETWORK_OPEN_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    CreationTime: LARGE_INTEGER;
    LastAccessTime: LARGE_INTEGER;
    LastWriteTime: LARGE_INTEGER;
    ChangeTime: LARGE_INTEGER;
    AllocationSize: LARGE_INTEGER;
    EndOfFile: LARGE_INTEGER;
    FileAttributes: ULONG;
  end;
  FILE_NETWORK_OPEN_INFORMATION = _FILE_NETWORK_OPEN_INFORMATION;
  PFILE_NETWORK_OPEN_INFORMATION = ^FILE_NETWORK_OPEN_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_POSITION_INFORMATION = record
=======
  _FILE_POSITION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_POSITION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_POSITION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_POSITION_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    CurrentByteOffset: LARGE_INTEGER;
  end;
  FILE_POSITION_INFORMATION = _FILE_POSITION_INFORMATION;
  PFILE_POSITION_INFORMATION = ^FILE_POSITION_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_DISPOSITION_INFORMATION = record
=======
  _FILE_DISPOSITION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_DISPOSITION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_DISPOSITION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_DISPOSITION_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    DeleteFile: NT_BOOLEAN;
  end;
  FILE_DISPOSITION_INFORMATION = _FILE_DISPOSITION_INFORMATION;
  PFILE_DISPOSITION_INFORMATION = ^FILE_DISPOSITION_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_RENAME_INFORMATION = record
=======
  _FILE_RENAME_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_RENAME_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_RENAME_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_RENAME_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    ReplaceIfExists: NT_BOOLEAN;
    RootDirectory: HANDLE;
    FileNameLength: ULONG;
    FileName: array[0..0] of WCHAR;
  end;
  FILE_RENAME_INFORMATION = _FILE_RENAME_INFORMATION;
  PFILE_RENAME_INFORMATION = ^FILE_RENAME_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_BOTH_DIR_INFORMATION = record
    NextEntryOffset: ULONG;
    FileIndex: ULONG;
    CreationTime: LARGE_INTEGER;
    LastAccessTime: LARGE_INTEGER;
    LastWriteTime: LARGE_INTEGER;
    ChangeTime: LARGE_INTEGER;
    EndOfFile: LARGE_INTEGER;
    AllocationSize: LARGE_INTEGER;
    FileAttributes: ULONG;
    FileNameLength: ULONG;
    EaSize: ULONG;
    ShortNameLength: CCHAR;
    ShortName: array[0..11] of WCHAR;
    FileName: array[0..0] of WCHAR;
  end;
  FILE_BOTH_DIR_INFORMATION = _FILE_BOTH_DIR_INFORMATION;
  PFILE_BOTH_DIR_INFORMATION = ^FILE_BOTH_DIR_INFORMATION;

  _FILE_NAME_INFORMATION = record
    FileNameLength: ULONG;
    FileName: array[0..0] of WCHAR;
  end;
  FILE_NAME_INFORMATION = _FILE_NAME_INFORMATION;
  PFILE_NAME_INFORMATION = ^FILE_NAME_INFORMATION;

  _FILE_ALLOCATION_INFORMATION = record
=======
  _FILE_ALLOCATION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_ALLOCATION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_ALLOCATION_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_ALLOCATION_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    AllocationSize: LARGE_INTEGER;
  end;
  FILE_ALLOCATION_INFORMATION = _FILE_ALLOCATION_INFORMATION;
  PFILE_ALLOCATION_INFORMATION = ^FILE_ALLOCATION_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_END_OF_FILE_INFORMATION = record
=======
  _FILE_END_OF_FILE_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_END_OF_FILE_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_END_OF_FILE_INFORMATION = packed record
>>>>>>> graemeg/cpstrnew
=======
  _FILE_END_OF_FILE_INFORMATION = packed record
>>>>>>> origin/cpstrnew
    EndOfFile: LARGE_INTEGER;
  end;
  FILE_END_OF_FILE_INFORMATION = _FILE_END_OF_FILE_INFORMATION;
  PFILE_END_OF_FILE_INFORMATION = ^FILE_END_OF_FILE_INFORMATION;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  _FILE_DIRECTORY_INFORMATION = record
    NextEntryOffset: ULONG;
    FileIndex: ULONG;
    CreationTime: LARGE_INTEGER;
    LastAccessTime: LARGE_INTEGER;
    LastWriteTime: LARGE_INTEGER;
    ChangeTime: LARGE_INTEGER;
    EndOfFile: LARGE_INTEGER;
    AllocationSize: LARGE_INTEGER;
    FileAttributes: ULONG;
    FileNameLength: ULONG;
    FileName: array[0..0] of WCHAR;
  end;
  FILE_DIRECTORY_INFORMATION = _FILE_DIRECTORY_INFORMATION;
  PFILE_DIRECTORY_INFORMATION = ^FILE_DIRECTORY_INFORMATION;

  //
  // APC Callback for NtCreateFile
  //
  PIO_APC_ROUTINE = procedure(
    ApcContext: PVOID;
    IoStatusBlock: PIO_STATUS_BLOCK;
    Reserved: ULONG); stdcall;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

