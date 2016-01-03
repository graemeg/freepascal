{%MainUnit ndk.pas}
{
    Native Development Kit for Native NT

    This file is part of the Free Pascal run time library.
    This unit contains functions for use with the Object Manager.
    Copyright (c) 2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function NtOpenDirectoryObject(
  FileHandle: PHANDLE;
  DesiredAccess: ACCESS_MASK;
  ObjectAttributes: POBJECT_ATTRIBUTES
): NTSTATUS; external ntdll;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NtQueryDirectoryObject(
  DirectoryHandle: HANDLE;
  Buffer: PVOID;
  BufferLength: ULONG;
  ReturnSingleEntry: NT_BOOLEAN;
  RestartScan: NT_BOOLEAN;
  Context: PULONG;
  ReturnLength: PULONG {OPTIONAL}
): NTSTATUS; external ntdll;

function NtQueryObject(
  ObjectHandle: HANDLE;
  ObjectInformationClass: OBJECT_INFORMATION_CLASS;
  ObjectInformation: PVOID;
  Length: ULONG;
  ResultLength: PULONG {OPTIONAL}
): NTSTATUS; external ntdll;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function NtWaitForSingleObject(
  _Object: HANDLE;
  Alertable: NT_BOOLEAN;
  Time: PLARGE_INTEGER
): NTSTATUS; external ntdll;

