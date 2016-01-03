{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by the Free Pascal development team

    Implement OS-dependent part of dynamic library loading.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
uses libc;
=======
{$ifdef readinterface}

{ ---------------------------------------------------------------------
    Interface declarations
  ---------------------------------------------------------------------}

Type
  TLibHandle = Pointer;

Const
  NilHandle = Nil;
// these are for easier crossplatform construction of dll names in dynloading libs.
  SharedSuffix = 'so';

{$else}
>>>>>>> graemeg/cpstrnew

Function SysLoadLibraryA(const Name : RawByteString) : TLibHandle;

begin
  Result:=dlopen(PAnsiChar(Name),RTLD_LAZY);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function SysLoadLibraryU(const Name: UnicodeString) : TLibHandle;
=======
Function LoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> graemeg/cpstrnew
=======
Function LoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> graemeg/cpstrnew
=======
Function LoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> graemeg/cpstrnew
=======
Function LoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> origin/cpstrnew

begin
  Result:=SysLoadLibraryA(ToSingleByteFileSystemEncodedFileName(Name));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
  Result:=dlsym(lib,pchar(ProcName));
end;

Function SysUnloadLibrary(Lib : TLibHandle) : Boolean;

begin
  Result:=dlClose(Lib)=0;
end;

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

