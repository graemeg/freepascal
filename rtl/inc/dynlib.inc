{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2015 by the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ---------------------------------------------------------------------
  OS - Independent declarations.
  ---------------------------------------------------------------------}

Var
  CurrentDLM : TDynLibsManager;

{$ifndef FPCRTL_FILESYSTEM_TWO_BYTE_API}
Function DoSafeLoadLibrary(const Name : RawByteString) : TLibHandle;
{$else not FPCRTL_FILESYSTEM_TWO_BYTE_API}
Function DoSafeLoadLibrary(const Name : UnicodeString) : TLibHandle;
{$endif not FPCRTL_FILESYSTEM_TWO_BYTE_API}
{$if defined(cpui386) or defined(cpux86_64)}
  var
    fpucw : Word;
    ssecw : DWord;
{$endif}
  begin
    try
{$if defined(cpui386) or defined(cpux86_64)}
      fpucw:=Get8087CW;
{$ifdef cpui386}
      if has_sse_support then
{$endif cpui386}
        ssecw:=GetMXCSR;
{$endif}
{$ifndef FPCRTL_FILESYSTEM_TWO_BYTE_API}
      Result:=CurrentDLM.LoadLibraryA(Name);
{$else FPCRTL_FILESYSTEM_TWO_BYTE_API}
      Result:=CurrentDLM.LoadLibraryU(Name);
{$endif FPCRTL_FILESYSTEM_TWO_BYTE_API}
      finally
{$if defined(cpui386) or defined(cpux86_64)}
      Set8087CW(fpucw);
{$ifdef cpui386}
      if has_sse_support then
{$endif cpui386}
        SetMXCSR(ssecw);
{$endif}
    end;
  end;

Function LoadLibrary(const Name : RawByteString) : TLibHandle;
begin
  Result:=CurrentDLM.LoadLibraryA(Name);
end;

Function LoadLibrary(const Name: UnicodeString) : TLibHandle;
begin
  Result:=CurrentDLM.LoadLibraryU(Name);
end;

{$ifndef FPCRTL_FILESYSTEM_TWO_BYTE_API}

Function SafeLoadLibrary(const Name: RawByteString) : TLibHandle;
begin
  Result:=DoSafeLoadLibrary(Name);
end;

Function SafeLoadLibrary(const Name: UnicodeString) : TLibHandle;
begin
  Result:=DoSafeLoadLibrary(ToSingleByteFileSystemEncodedFileName(Name));
end;

{$else not FPCRTL_FILESYSTEM_TWO_BYTE_API}

Function SafeLoadLibrary(const Name: RawByteString) : TLibHandle;
begin
  Result:=DoSafeLoadLibrary(UnicodeString(Name));
end;

Function SafeLoadLibrary(const Name: UnicodeString) : TLibHandle;
begin
  Result:=DoSafeLoadLibrary(Name);
end;

{$endif not FPCRTL_FILESYSTEM_TWO_BYTE_API}

Function GetProcedureAddress(Lib : TLibHandle; const ProcName : AnsiString) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif};
begin
  Result:=CurrentDLM.GetProcAddress(Lib, ProcName);
end;


Function GetProcedureAddress(Lib : TLibHandle; Ordinal : TOrdinalEntry) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif};
begin
  if Assigned(CurrentDLM.GetProcAddressOrdinal) then
    Result:=CurrentDLM.GetProcAddressOrdinal(Lib, Ordinal)
  else
    Result:=Nil;
end;


Function UnloadLibrary(Lib : TLibHandle) : Boolean;
begin
  Result:=CurrentDLM.UnloadLibrary(lib);
end;


function GetLoadErrorStr: String;
begin
  Result:=CurrentDLM.GetLoadErrorStr();
end;


Function FreeLibrary(Lib : TLibHandle) : Boolean;
begin
  Result:=UnloadLibrary(lib);
end;


Function GetProcAddress(Lib : TLibHandle; const ProcName : AnsiString) : {$ifdef cpui8086}FarPointer{$else}Pointer{$endif};
begin
  Result:=GetProcedureAddress(Lib,Procname);
end;

Procedure GetDynLibsManager (Out Manager : TDynLibsManager);
begin
  Manager:=CurrentDLM;
end;

Procedure SetDynLibsManager (Const New : TDynLibsManager);
begin
  CurrentDLM:=New;
end;


Procedure SetDynLibsManager (Const New : TDynLibsManager; Out Old: TDynLibsManager);
begin
  Old:=CurrentDLM;
  CurrentDLM:=New;
end;

{ ---------------------------------------------------------------------
    DynLibsManager which gives run-time error. Use if no thread support.
  ---------------------------------------------------------------------}

{$ifndef DISABLE_NO_DYNLIBS_MANAGER}

{ resourcestrings are not supported by the system unit,
  they are in the objpas unit and not available for fpc/tp modes }
const
  SNoDynLibs = 'This binary has no dynamic library support compiled in.';
  SRecompileWithDynLibs = 'Recompile the application with a dynamic-library-driver in the program uses clause before other units using dynamic libraries.';

Procedure NoDynLibsError; noreturn;
begin
{$ifndef EMBEDDED}
{$ifdef FPC_HAS_FEATURE_CONSOLEIO}
  If IsConsole then
    begin
    Writeln(StdErr,SNoDynLibs);
    Writeln(StdErr,SRecompileWithDynLibs);
    end;
{$endif FPC_HAS_FEATURE_CONSOLEIO}
{$endif EMBEDDED}
  RunError(235)
end;

Function NoLoadLibraryU(const Name: UnicodeString): TLibHandle; noreturn;
begin
  NoDynLibsError;
end;

Function NoLoadLibraryA(const Name: RawByteString): TLibHandle; noreturn;
begin
  NoDynLibsError;
end;

function NoGetProcAddress(Lib: TLibHandle; const Proc: AnsiString): {$ifdef cpui8086}FarPointer{$else}Pointer{$endif}; noreturn;
begin
  NoDynLibsError;
end;

function NoGetProcAddressOrdinal(Lib: TLibHandle; Ordinal: TOrdinalEntry): {$ifdef cpui8086}FarPointer{$else}Pointer{$endif}; noreturn;
begin
  NoDynLibsError;
end;

function NoGetLoadErrorStr: String; noreturn;
begin
  NoDynLibsError;
end;

function NoUnloadLibrary(Lib: TLibHandle): Boolean; noreturn;
begin
  NoDynLibsError;
end;

const
  NoDynLibsManager: TDynLibsManager = (
    LoadLibraryU: @NoLoadLibraryU;
    LoadLibraryA: @NoLoadLibraryA;
    GetProcAddress: @NoGetProcAddress;
    GetProcAddressOrdinal: @NoGetProcAddressOrdinal;
    UnloadLibrary: @NoUnloadLibrary;
    GetLoadErrorStr: @NoGetLoadErrorStr;
  );

procedure SetNoDynLibsManager;
begin
  SetDynLibsManager(NoDynLibsManager);
end;

procedure InitSystemDynLibs;
begin
  SetNoDynLibsManager;
end;

{$endif DISABLE_NO_DYNLIBS_MANAGER}
