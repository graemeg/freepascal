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
<<<<<<< HEAD
<<<<<<< HEAD
procedure InitDynLibs;
=======
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef readinterface}

{ ---------------------------------------------------------------------
    Interface declarations
  ---------------------------------------------------------------------}

Type
  TLibHandle = Longint;

Const
  NilHandle = 0;
<<<<<<< HEAD
<<<<<<< HEAD
// these are for easier crossplatform construction of dll names in dynloading libs.
  SharedSuffix = 'dll';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$else}

{ ---------------------------------------------------------------------
    Implementation section
  ---------------------------------------------------------------------}

Uses windows;

<<<<<<< HEAD
<<<<<<< HEAD
Function LoadLibrary(const Name : AnsiString) : TlibHandle;
=======
Function LoadLibrary(Name : AnsiString) : TlibHandle;
>>>>>>> graemeg/fixes_2_2
=======
Function LoadLibrary(Name : AnsiString) : TlibHandle;
>>>>>>> origin/fixes_2_2
var
  ws: PWideChar;
begin
  ws:=StringToPWideChar(Name);
  Result:=Windows.LoadLibrary(ws);
  FreeMem(ws);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : Pointer;
=======
Function GetProcedureAddress(Lib : TLibHandle; ProcName : AnsiString) : Pointer;
>>>>>>> graemeg/fixes_2_2
=======
Function GetProcedureAddress(Lib : TLibHandle; ProcName : AnsiString) : Pointer;
>>>>>>> origin/fixes_2_2
var
  ws: PWideChar;
begin
  ws:=StringToPWideChar(ProcName);
  Result:=Windows.GetProcAddress(Lib, ws);
  FreeMem(ws);
end;

Function UnloadLibrary(Lib : TLibHandle) : Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
begin
  { nothing to do here since Windows DynLibs is done in System }
end;

=======
=======
>>>>>>> origin/fixes_2_2
begin
  Result:=Windows.FreeLibrary(Lib);
end;

{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
