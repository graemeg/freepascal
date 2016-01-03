{%MainUnit ndk.pas}
{
    Native Development Kit for Native NT

    This file is part of the Free Pascal run time library.
    This unit contains IO functions.
    Copyright (c) 2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function NtCreateFile(
  FileHandle: PHANDLE;
  DesiredAccess: ACCESS_MASK;
  ObjectAttributes: POBJECT_ATTRIBUTES;
  IoStatusBlock: PIO_STATUS_BLOCK;
  AllocationSize: PLARGE_INTEGER; { optional }
  FileAttributes: ULONG;
  ShareAccess: ULONG;
  CreateDisposition: ULONG;
  CreateOptions: ULONG;
  EaBuffer: PVOID; { optional }
  EaLength: ULONG
): NTSTATUS; external ntdll;

function NtOpenFile(
  FileHandle: PHANDLE;
  DesiredAccess: ACCESS_MASK;
  ObjectAttributes: POBJECT_ATTRIBUTES;
  IoStatusBlock: PIO_STATUS_BLOCK;
  ShareAccess: ULONG;
  OpenOptions: ULONG
): NTSTATUS; external ntdll;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NtQueryDirectoryFile(
  FileHandle: HANDLE;
  Event: HANDLE; {OPTIONAL}
  ApcRoutine: PIO_APC_ROUTINE; {OPTIONAL}
  ApcContext: PVOID; {OPTIONAL}
  IoStatusBlock: PIO_STATUS_BLOCK;
  FileInformation: PVOID;
  Length: ULONG;
  FileInformationClass: FILE_INFORMATION_CLASS;
  ReturnSingleEntry: NT_BOOLEAN;
  FileName: PUNICODE_STRING; {OPTIONAL}
  RestartScan: NT_BOOLEAN
): NTSTATUS; external ntdll;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function NtQueryFullAttributesFile(
  ObjectAttributes: POBJECT_ATTRIBUTES;
  FileInformation: PFILE_NETWORK_OPEN_INFORMATION
): NTSTATUS; external ntdll;

function NtQueryInformationFile(
  FileHandle: HANDLE;
  IoStatusBlock: PIO_STATUS_BLOCK;
  FileInformation: PVOID;
  Length: ULONG;
  FileInformationClass: FILE_INFORMATION_CLASS
): NTSTATUS; external ntdll;

function NtReadFile(
  FileHandle: HANDLE;
  Event: HANDLE; { optional }
  UserApcRoutine: Pointer; //PIO_APC_ROUTINE; { optional }
  UserApcContext: PVOID; { optional }
  IoStatusBlock: PIO_STATUS_BLOCK;
  Buffer: PVOID;
  BufferLength: ULONG;
  ByteOffset: PLARGE_INTEGER; { optional }
  Key: PULONG { optional }
): NTSTATUS; external ntdll;

function NtSetInformationFile(
  FileHandle: HANDLE;
  IoStatusBlock: PIO_STATUS_BLOCK;
  FileInformation: PVOID;
  Length: ULONG;
  FileInformationClass: FILE_INFORMATION_CLASS
): NTSTATUS; external ntdll;

function NtWriteFile(
  FileHandle: HANDLE;
  Event: HANDLE; { optional }
  ApcRoutine: Pointer; //PIO_APC_ROUTINE; { optional }
  ApcContext: PVOID; { optional }
  IoStatusBlock: PIO_STATUS_BLOCK;
  Buffer: PVOID;
  Length: ULONG;
  ByteOffset: PLARGE_INTEGER; { optional? }
  Key: PULONG { optional }
): NTSTATUS; external ntdll;

