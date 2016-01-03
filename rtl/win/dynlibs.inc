{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Implements OS dependent part for loading of dynamic libraries.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
procedure InitDynLibs;
=======

{$ifdef readinterface}

{ ---------------------------------------------------------------------
    Interface declarations
  ---------------------------------------------------------------------}

Type
  TLibHandle = System.THandle;

Const
  NilHandle = 0;
// these are for easier crossplatform construction of dll names in dynloading libs.
  SharedSuffix = 'dll';

{$else}

{ ---------------------------------------------------------------------
    Implementation section
  ---------------------------------------------------------------------}

Uses windows;

Function LoadLibrary(const Name : AnsiString) : TlibHandle;

>>>>>>> graemeg/cpstrnew
begin
  { nothing to do here since Windows DynLibs is done in System }
end;
<<<<<<< HEAD
=======

Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;

begin
  Result:=Windows.GetProcAddress(Lib,PChar(ProcName));
end;

Function UnloadLibrary(Lib : TLibHandle) : Boolean;

begin
  Result:=Windows.FreeLibrary(Lib);
end;

{$endif}

>>>>>>> graemeg/cpstrnew
