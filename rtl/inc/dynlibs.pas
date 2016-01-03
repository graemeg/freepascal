{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Implements OS-independent loading of dynamic libraries.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$IFDEF FPC}
{$MODE OBJFPC}
{$ENDIF}

unit dynlibs;

interface

Type
  TLibHandle = System.TLibHandle;

Const
  NilHandle = System.NilHandle;
  SharedSuffix = System.SharedSuffix;

Function SafeLoadLibrary(const Name : RawByteString) : TLibHandle; inline;
Function LoadLibrary(const Name : RawByteString) : TLibHandle; inline;
Function SafeLoadLibrary(const Name : UnicodeString) : TLibHandle; inline;
Function LoadLibrary(const Name : UnicodeString) : TLibHandle; inline;

<<<<<<< HEAD
Function GetProcedureAddress(Lib : TlibHandle; const ProcName : AnsiString) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif}; inline;
Function GetProcedureAddress(Lib : TLibHandle; Ordinal: TOrdinalEntry) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif}; inline;
Function UnloadLibrary(Lib : TLibHandle) : Boolean; inline;
Function GetLoadErrorStr: string; inline;

// Kylix/Delphi compability

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function FreeLibrary(Lib : TLibHandle) : Boolean; inline;
Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif}; inline;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function SafeLoadLibrary(const Name : AnsiString) : TLibHandle;
Function LoadLibrary(const Name : AnsiString) : TLibHandle;
Function GetProcedureAddress(Lib : TlibHandle; const ProcName : AnsiString) : Pointer;
=======
=======
>>>>>>> origin/fixes_2_2
Function SafeLoadLibrary(Name : AnsiString) : TLibHandle;
Function LoadLibrary(Name : AnsiString) : TLibHandle;
Function GetProcedureAddress(Lib : TlibHandle; ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/fixes_2_2
Function UnloadLibrary(Lib : TLibHandle) : Boolean;

// Kylix/Delphi compability

Function FreeLibrary(Lib : TLibHandle) : Boolean;
Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : Pointer;
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

Type
  HModule = TLibHandle; 

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
// these are for easier crossplatform construction of dll names in dynloading libs.
Const
 {$ifdef Windows}
  SharedSuffix  = 'dll';
 {$else}
   {$ifdef Darwin}
     SharedSuffix = 'dylib';
   {$else}
     {$ifdef OS2}
       SharedSuffix = 'dll';
     {$else}
       SharedSuffix = 'so';  
     {$endif}
   {$endif}
 {$endif}      
      
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Implementation


{ Should define a procedure InitDynLibs which sets up the DynLibs manager; optionally a
  DoneDynLibs can be defined which is called during finalization }
{$i dynlibs.inc}

Function SafeLoadLibrary(const Name : RawByteString) : TLibHandle;
begin
  Result:=System.SafeLoadLibrary(Name);
end;

Function LoadLibrary(const Name : RawByteString) : TLibHandle;
begin
  Result:=System.LoadLibrary(Name);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function SafeLoadLibrary(const Name : UnicodeString) : TLibHandle;
begin
  Result:=System.SafeLoadLibrary(Name);
end;
=======
Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/cpstrnew
=======
Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/cpstrnew

Function LoadLibrary(const Name : UnicodeString) : TLibHandle;
begin
  Result:=System.LoadLibrary(Name);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
Function SafeLoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> graemeg/cpstrnew
=======
Function SafeLoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> graemeg/cpstrnew

Function GetProcedureAddress(Lib : TLibHandle; const ProcName: AnsiString) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif};
begin
  Result:=System.GetProcedureAddress(Lib, ProcName);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function GetProcedureAddress(Lib : TLibHandle; Ordinal : TOrdinalEntry) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif};
begin
  Result:=System.GetProcedureAddress(Lib, Ordinal);
end;
=======
Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/cpstrnew
=======
Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : Pointer;
>>>>>>> origin/cpstrnew

Function UnloadLibrary(Lib : TLibHandle) : Boolean;
begin
  Result:=System.UnloadLibrary(Lib);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function GetLoadErrorStr: String;
begin
  Result:=System.GetLoadErrorStr;
end;
=======
Function SafeLoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> graemeg/cpstrnew
=======
Function SafeLoadLibrary(const Name : AnsiString) : TLibHandle;
>>>>>>> origin/cpstrnew

Function FreeLibrary(Lib : TLibHandle) : Boolean;

begin
  Result:=System.FreeLibrary(lib);
end;

Function GetProcAddress(Lib : TlibHandle; const ProcName : AnsiString) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif};

begin
  Result:=System.GetProcedureAddress(Lib,Procname);
end;

<<<<<<< HEAD
<<<<<<< HEAD
initialization
  InitDynLibs;
finalization
{$if declared(DoneDynLibs)}
  DoneDynLibs;
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
Function SafeLoadLibrary(Name : AnsiString) : TLibHandle;

{$ifdef i386}
 var w : word;
{$endif}


Begin
{$ifdef i386}
  w:=get8087cw;
{$endif}
 result:=loadlibrary(name);

{$ifdef i386}
  set8087cw(w);
{$endif}
End;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
