{
    Native Development Kit for Native NT

    This file is part of the Free Pascal run time library.
    Copyright (c) 2009-2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit NDK;

interface

const
{$ifdef kmode}
  ntdll = 'ntoskrnl.exe';
{$else}
  ntdll = 'ntdll.dll';
{$endif}

{$calling stdcall}

{$include ntdef.inc}
{$include winnt.inc}

{$include ntstatus.inc}

{$include umtypes.inc}
{$include iotypes.inc}
{$include rtltypes.inc}
{$include ketypes.inc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$include obtypes.inc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$include pstypes.inc}
{$include peb_teb.inc}

{$include rtlfuncs.inc}
{$include iofuncs.inc}
{$include obfuncs.inc}

function NtClose(Handle: HANDLE): NTSTATUS; stdcall; external ntdll;

function NtDelayExecution(aAlertable: NT_BOOLEAN; aInterval: PLARGE_INTEGER): NTSTATUS; stdcall; external ntdll;
function NtDisplayString(aString: PUNICODE_STRING): NTSTATUS; stdcall; external ntdll;

function LdrGetProcedureAddress(hModule: HANDLE; psName: PUNICODE_STRING; dwOrdinal: LongWord; var pProcedure: PVOID): NTSTATUS; stdcall; external ntdll;
function LdrLoadDll(pwPath : PWord; pdwFlags : LongWord; pusPath : PUNICODE_STRING; var phModule : HANDLE): NTSTATUS; stdcall; external ntdll;
function LdrUnloadDll(hModule: HANDLE): NTSTATUS; stdcall; external ntdll;

implementation

function SharedUserData: PKUSER_SHARED_DATA; register;
begin
  { this is a pointer to a page that is mapped into every process by the kernel
  }
  SharedUserData := PKUSER_SHARED_DATA(USER_SHARED_DATA);
end;

procedure InitializeObjectAttributes(var aObjectAttr: OBJECT_ATTRIBUTES;
    aName: PUNICODE_STRING; aAttributes: ULONG; aRootDir: HANDLE;
    aSecurity: Pointer {PSECURITY_DESCRIPTOR}); register;
begin
  with aObjectAttr do begin
    Length := SizeOf(OBJECT_ATTRIBUTES);
    RootDirectory := aRootDir;
    Attributes := aAttributes;
    ObjectName := aName;
    SecurityDescriptor := aSecurity;
    SecurityQualityOfService := Nil;
  end;
end;

function NT_SUCCESS(Status: NTSTATUS): Boolean; register;
begin
  NT_SUCCESS := Status >= 0;
end;

function NT_INFORMATION(Status: NTSTATUS): Boolean; register;
begin
  NT_INFORMATION := ULONG(Status) shr 30 = 1;
end;

function NT_WARNING(Status: NTSTATUS): Boolean; register;
begin
  NT_WARNING := ULONG(Status) shr 30 = 2;
end;

function NT_ERROR(Status: NTSTATUS): Boolean; register;
begin
  NT_ERROR := ULONG(Status) shr 30 = 3;
end;

end.

