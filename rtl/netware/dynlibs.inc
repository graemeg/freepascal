{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2011 by the Free Pascal development team

    Implements OS dependent part for loading of dynamic libraries.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Uses nwserv;

Function SysLoadLibraryA(const Name : RawByteString) : TlibHandle;
var args : array[0..1] of PAnsiChar;
begin
  args[0] := PAnsiChar(Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef readinterface}

{ ---------------------------------------------------------------------
    Interface declarations
  ---------------------------------------------------------------------}

Type
  TLibHandle = System.THandle;

Const
  NilHandle = 0;
// these are for easier crossplatform construction of dll names in dynloading libs.
  SharedSuffix = 'nlm';

{$else}

{ ---------------------------------------------------------------------
    Implementation section
  ---------------------------------------------------------------------}

Uses nwserv;

Function LoadLibrary(const Name : AnsiString) : TlibHandle;
var args : array[0..1] of pchar;
begin
  args[0] := pchar(Name);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  args[1] := nil;
  Result:=spawnvp(P_NOWAIT,@args,nil);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function SysLoadLibraryU(const Name: UnicodeString) : TLibHandle;
begin
  Result := SysLoadLibraryA(ToSingleByteFileSystemEncodedFileName(Name));
end;

Function SysGetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;
=======
Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/cpstrnew
=======
Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/cpstrnew
=======
Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/cpstrnew
=======
Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> origin/cpstrnew

begin
  Result:=ImportSymbol(GetNlmHandle, pchar(ProcName));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function SysUnloadLibrary(Lib : TLibHandle) : Boolean;
=======
Function UnloadLibrary(Lib : TLibHandle) : Boolean;
>>>>>>> graemeg/cpstrnew
=======
Function UnloadLibrary(Lib : TLibHandle) : Boolean;
>>>>>>> graemeg/cpstrnew
=======
Function UnloadLibrary(Lib : TLibHandle) : Boolean;
>>>>>>> graemeg/cpstrnew
=======
Function UnloadLibrary(Lib : TLibHandle) : Boolean;
>>>>>>> origin/cpstrnew

begin
  Result:=false;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Function SysGetLoadErrorStr: string;

begin  
  Result:='';
end;


const
  SysDynLibsManager: TDynLibsManager = (
    LoadLibraryU: @SysLoadLibraryU;
    LoadLibraryA: @SysLoadLibraryA;
    GetProcAddress: @SysGetProcedureAddress;
    GetProcAddressOrdinal: Nil;
    UnloadLibrary: @SysUnloadLibrary;
    GetLoadErrorStr: @SysGetLoadErrorStr;
  );

procedure InitDynLibs;
begin
  SetDynLibsManager(SysDynLibsManager);
end;
=======
{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$endif}
>>>>>>> origin/cpstrnew

