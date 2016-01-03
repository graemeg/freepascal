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

type
 TLibHandle = longint;

const
 NilHandle = 0;
// these are for easier crossplatform construction of dll names in dynloading libs.
 SharedSuffix  = 'dll';

{$else}

{ ---------------------------------------------------------------------
    Implementation section
  ---------------------------------------------------------------------}

uses
 DosCalls;

function LoadLibrary (const Name: AnsiString): TLibHandle;
var
 ErrPath: array [0..259] of char;
 Handle: longint;
>>>>>>> graemeg/cpstrnew
begin
  { nothing to do here since OS/2 DynLibs is done in System }
end;
<<<<<<< HEAD
=======

function GetProcedureAddress (Lib: TLibHandle; const ProcName: AnsiString): pointer;
var
 P: pointer;
begin
 if DosQueryProcAddr (Lib, 0, PChar (ProcName), P) = 0 then Result := P
                                                            else Result := nil;
end;

function UnloadLibrary (Lib: TLibHandle): boolean;
begin
 Result := DosFreeModule (Lib) = 0;
end;

{$endif}

>>>>>>> graemeg/cpstrnew
