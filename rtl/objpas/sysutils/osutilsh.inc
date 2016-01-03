{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Type TExecuteFlags = Set of ( ExecInheritsHandles);

{ OS handling utilities }

{$ifdef HAS_OSERROR}
Function GetLastOSError : Integer;
{$endif}
Procedure RaiseLastOSError;overload;
Procedure RaiseLastOSError(LastError: Integer);overload;

Function GetEnvironmentVariable(Const EnvVar : AnsiString) : AnsiString;
Function GetEnvironmentVariable(Const EnvVar : UnicodeString) : UnicodeString;
Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};
Function GetEnvironmentVariableCount : Integer;
{$IFDEF HAS_SLEEP}
procedure Sleep(milliseconds: Cardinal);
{$ENDIF}
function ExecuteProcess(Const Path: AnsiString; Const ComLine: AnsiString;Flags:TExecuteFlags=[]):integer;
function ExecuteProcess(Const Path: AnsiString; Const ComLine: Array of AnsiString;Flags:TExecuteFlags=[]):integer;

Function GetTempDir(Global : Boolean) : String;
Function GetTempDir : String;
Function GetTempFileName(Const Dir,Prefix : String) : String;
Function GetTempFileName : String;
{ For windows compatibility }
function GetTempFileName(Dir,Prefix: PChar; uUnique: DWORD; TempFileName: PChar):DWORD;
Function GetAppConfigDir(Global : Boolean) : String;
Function GetAppConfigFile(Global : Boolean) : String;
Function GetAppConfigFile(Global : Boolean; SubDir : Boolean) : String;
Function GetUserDir : String;
Function VendorName : String;
Function ApplicationName : String;

Const
  ConfigExtension : String = '.cfg';
  SysConfigDir    : String = '';

Type
  TGetVendorNameEvent  = Function : String;
  TGetAppNameEvent  = Function : String;
  TGetTempDirEvent  = Function (Global : Boolean) : String;
  TGetTempFileEvent = Function (Const Dir,Prefix : String) : String;

Type
  TEventType = (etCustom,etInfo,etWarning,etError,etDebug);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TEventTypes = Set of TEventType;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

Var
  OnGetVendorName      : TGetVendorNameEvent;
  OnGetApplicationName : TGetAppNameEvent;
  OnGetTempDir         : TGetTempDirEvent;
  OnGetTempFile        : TGetTempFileEvent;


