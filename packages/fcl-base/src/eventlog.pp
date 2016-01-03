{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team

    Cross-platform event logging facility.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$h+}
unit eventlog;

interface

uses SysUtils,Classes;

Type
  TEventLog = Class;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  TEventType = (etCustom,etInfo,etWarning,etError,etDebug);
>>>>>>> graemeg/fixes_2_2
=======
  TEventType = (etCustom,etInfo,etWarning,etError,etDebug);
>>>>>>> origin/fixes_2_2
  TLogType = (ltSystem,ltFile);
  TLogCodeEvent = Procedure (Sender : TObject; Var Code : DWord) of Object;
  TLogCategoryEvent = Procedure (Sender : TObject; Var Code : Word) of Object;

  TEventLog = Class(TComponent)
  Private
<<<<<<< HEAD
<<<<<<< HEAD
    fAppendContent : Boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FEventIDOffset : DWord;
    FLogHandle : Pointer;
    FStream : TFileStream;
    FActive: Boolean;
    FRaiseExceptionOnError: Boolean;
    FIdentification: String;
    FDefaultEventType: TEventType;
    FLogtype: TLogType;
    FFileName: String;
    FTimeStampFormat: String;
    FCustomLogType: Word;
    FOnGetCustomCategory : TLogCategoryEvent;
    FOnGetCustomEventID : TLogCodeEvent;
    FOnGetCustomEvent : TLogCodeEvent;
<<<<<<< HEAD
<<<<<<< HEAD
    FPaused : Boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SetActive(const Value: Boolean);
    procedure SetIdentification(const Value: String);
    procedure SetlogType(const Value: TLogType);
    procedure ActivateLog;
    procedure DeActivateLog;
    procedure ActivateFileLog;
    procedure SetFileName(const Value: String);
    procedure ActivateSystemLog;
    function DefaultFileName: String;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure WriteFileLog(EventType : TEventType; const Msg: String);
    procedure WriteSystemLog(EventType: TEventType; const Msg: String);
=======
    procedure WriteFileLog(EventType : TEventType; Msg: String);
    procedure WriteSystemLog(EventType: TEventType; Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
    procedure WriteFileLog(EventType : TEventType; Msg: String);
    procedure WriteSystemLog(EventType: TEventType; Msg: String);
>>>>>>> origin/fixes_2_2
    procedure DeActivateFileLog;
    procedure DeActivateSystemLog;
    procedure CheckIdentification;
    Procedure DoGetCustomEventID(Var Code : DWord);
    Procedure DoGetCustomEventCategory(Var Code : Word);
    Procedure DoGetCustomEvent(Var Code : DWord);
  Protected
    Procedure CheckInactive;
    Procedure EnsureActive;
    function MapTypeToEvent(EventType: TEventType): DWord;
    Function MapTypeToCategory(EventType : TEventType) : Word;
    Function MapTypeToEventID(EventType : TEventType) : DWord;
  Public
    Destructor Destroy; override;
    Function EventTypeToString(E : TEventType) : String;
    Function RegisterMessageFile(AFileName : String) : Boolean; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    Function UnRegisterMessageFile : Boolean; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Pause;
    Procedure Resume;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Procedure Log (EventType : TEventType; const Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Log (EventType : TEventType; const Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Log (const Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Log (const Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Warning (const Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Warning (const Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Error (const Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Error (const Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Debug (const Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Debug (const Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Info (const Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Info (const Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
  Published
    Property AppendContent : Boolean Read fAppendContent Write fAppendContent;
=======
=======
>>>>>>> origin/fixes_2_2
    Procedure Log (EventType : TEventType; Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Log (EventType : TEventType; Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Log (Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Log (Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Warning (Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Warning (Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Error (Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Error (Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Debug (Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Debug (Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
    Procedure Info (Msg : String); {$ifndef fpc }Overload;{$endif}
    Procedure Info (Fmt : String; Args : Array of const); {$ifndef fpc }Overload;{$endif}
  Published
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Property Identification : String Read FIdentification Write SetIdentification;
    Property LogType : TLogType Read Flogtype Write SetlogType;
    Property Active : Boolean Read FActive write SetActive;
    Property RaiseExceptionOnError : Boolean Read FRaiseExceptionOnError Write FRaiseExceptionOnError;
    Property DefaultEventType : TEventType Read FDEfaultEventType Write FDefaultEventType;
    Property FileName : String Read FFileName Write SetFileName;
    Property TimeStampFormat : String Read FTimeStampFormat Write FTimeStampFormat;
    Property CustomLogType : Word Read FCustomLogType Write FCustomLogType;
    Property EventIDOffset : DWord Read FEventIDOffset Write FEventIDOffset;
    Property OnGetCustomCategory : TLogCategoryEvent Read FOnGetCustomCategory Write FOnGetCustomCategory;
    Property OnGetCustomEventID : TLogCodeEvent Read FOnGetCustomEventID Write FOnGetCustomEventID;
    Property OnGetCustomEvent : TLogCodeEvent Read FOnGetCustomEvent Write FOnGetCustomEvent;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Paused : Boolean Read FPaused Write FPaused;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  End;

  ELogError = Class(Exception);

Resourcestring

  SLogInfo      = 'Info';
  SLogWarning   = 'Warning';
  SLogError     = 'Error';
  SLogDebug     = 'Debug';
  SLogCustom    = 'Custom (%d)';
  SErrLogFailedMsg = 'Failed to log entry (Error: %s)';
<<<<<<< HEAD
<<<<<<< HEAD

implementation

{$i eventlog.inc}
(* File based dummy implementation is used for all platforms not providing
   specific implementation of eventlog.inc for the particular platform. *)
=======
=======
>>>>>>> origin/fixes_2_2
  
implementation

{$i eventlog.inc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ TEventLog }

Resourcestring
  SErrOperationNotAllowed = 'Operation not allowed when eventlog is active.';

procedure TEventLog.CheckInactive;
begin
  If Active then
    Raise ELogError.Create(SErrOperationNotAllowed);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Debug(const Fmt: String; Args: array of const);
=======
procedure TEventLog.Debug(Fmt: String; Args: array of const);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Debug(Fmt: String; Args: array of const);
>>>>>>> origin/fixes_2_2
begin
   Debug(Format(Fmt,Args));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Debug(const Msg: String);
=======
procedure TEventLog.Debug(Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Debug(Msg: String);
>>>>>>> origin/fixes_2_2
begin
  Log(etDebug,Msg);
end;

procedure TEventLog.EnsureActive;
begin
  If Not Active then
    Active:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Procedure TEventLog.Pause;

begin
  Paused:=True;
end;


Procedure TEventLog.Resume;
begin
  Paused:=False;
end;


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TEventLog.Error(const Fmt: String; Args: array of const);
=======
procedure TEventLog.Error(Fmt: String; Args: array of const);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Error(Fmt: String; Args: array of const);
>>>>>>> origin/fixes_2_2
begin
  Error(Format(Fmt,Args));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Error(const Msg: String);
=======
procedure TEventLog.Error(Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Error(Msg: String);
>>>>>>> origin/fixes_2_2
begin
  Log(etError,Msg);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Info(const Fmt: String; Args: array of const);
=======
procedure TEventLog.Info(Fmt: String; Args: array of const);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Info(Fmt: String; Args: array of const);
>>>>>>> origin/fixes_2_2
begin
  Info(Format(Fmt,Args));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Info(const Msg: String);
=======
procedure TEventLog.Info(Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Info(Msg: String);
>>>>>>> origin/fixes_2_2
begin
  Log(etInfo,Msg);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Log(const Msg: String);
=======
procedure TEventLog.Log(Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Log(Msg: String);
>>>>>>> origin/fixes_2_2
begin
  Log(DefaultEventType,msg);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Log(EventType: TEventType; const Fmt: String;
=======
procedure TEventLog.Log(EventType: TEventType; Fmt: String;
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Log(EventType: TEventType; Fmt: String;
>>>>>>> origin/fixes_2_2
  Args: array of const);
begin
  Log(EventType,Format(Fmt,Args));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Log(EventType: TEventType; const Msg: String);
begin
  If Paused then 
    exit;
=======
procedure TEventLog.Log(EventType: TEventType; Msg: String);
begin
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Log(EventType: TEventType; Msg: String);
begin
>>>>>>> origin/fixes_2_2
  EnsureActive;
  Case FlogType of
    ltFile   : WriteFileLog(EventType,Msg);
    ltSystem : WriteSystemLog(EventType,Msg);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.WriteFileLog(EventType : TEventType; const Msg : String);
=======
procedure TEventLog.WriteFileLog(EventType : TEventType; Msg : String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.WriteFileLog(EventType : TEventType; Msg : String);
>>>>>>> origin/fixes_2_2

Var
  S,TS,T : String;

begin
  If FTimeStampFormat='' then
    FTimeStampFormat:='yyyy-mm-dd hh:nn:ss.zzz';
  TS:=FormatDateTime(FTimeStampFormat,Now);
  T:=EventTypeToString(EventType);
  S:=Format('%s [%s %s] %s%s',[Identification,TS,T,Msg,LineEnding]);
  try
    FStream.WriteBuffer(S[1],Length(S));
    S:='';
  except
    On E : Exception do
      S:=E.Message;
<<<<<<< HEAD
<<<<<<< HEAD
  end;
=======
  end;  
>>>>>>> graemeg/fixes_2_2
=======
  end;  
>>>>>>> origin/fixes_2_2
  If (S<>'') and RaiseExceptionOnError then
    Raise ELogError.CreateFmt(SErrLogFailedMsg,[S]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Log(const Fmt: String; Args: array of const);
=======
procedure TEventLog.Log(Fmt: String; Args: array of const);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Log(Fmt: String; Args: array of const);
>>>>>>> origin/fixes_2_2
begin
  Log(Format(Fmt,Args));
end;

procedure TEventLog.SetActive(const Value: Boolean);
begin
  If Value<>FActive then
    begin
    If Value then
      ActivateLog
    else
      DeActivateLog;
    FActive:=Value;
    end;
end;

Procedure TEventLog.ActivateLog;

begin
  Case FLogType of
    ltFile : ActivateFileLog;
    ltSystem : ActivateSystemLog;
  end;
end;

Procedure TEventLog.DeActivateLog;

begin
  Case FLogType of
    ltFile : DeActivateFileLog;
    ltSystem : DeActivateSystemLog;
  end;
end;

Procedure TEventLog.ActivateFileLog;
<<<<<<< HEAD
<<<<<<< HEAD
var
  fFileFlags : Word;
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
begin
  If (FFileName='') then
    FFileName:=DefaultFileName;
  // This will raise an exception if the file cannot be opened for writing !
<<<<<<< HEAD
<<<<<<< HEAD
  if fAppendContent and FileExists(FFileName) then
    fFileFlags := fmOpenWrite
  else
    fFileFlags := fmCreate;

  fFileFlags := fFileFlags or fmShareDenyWrite;
  FStream:=TFileStream.Create(FFileName,fFileFlags);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if fAppendContent then
    FStream.Seek(0,soFromEnd);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  FStream:=TFileStream.Create(FFileName,fmCreate or fmShareDenyWrite);
>>>>>>> graemeg/fixes_2_2
=======
  FStream:=TFileStream.Create(FFileName,fmCreate or fmShareDenyWrite);
>>>>>>> origin/fixes_2_2
end;

Procedure TEventLog.DeActivateFileLog;

begin
  FStream.Free;
  FStream:=Nil;
end;


procedure TEventLog.SetIdentification(const Value: String);
begin
  FIdentification := Value;
end;

procedure TEventLog.SetlogType(const Value: TLogType);
begin
  CheckInactive;
  Flogtype := Value;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Warning(const Fmt: String; Args: array of const);
=======
procedure TEventLog.Warning(Fmt: String; Args: array of const);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Warning(Fmt: String; Args: array of const);
>>>>>>> origin/fixes_2_2
begin
  Warning(Format(Fmt,Args));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventLog.Warning(const Msg: String);
=======
procedure TEventLog.Warning(Msg: String);
>>>>>>> graemeg/fixes_2_2
=======
procedure TEventLog.Warning(Msg: String);
>>>>>>> origin/fixes_2_2
begin
  Log(etWarning,Msg);
end;

procedure TEventLog.SetFileName(const Value: String);
begin
  CheckInactive;
  FFileName := Value;
end;

Procedure TEventLog.CheckIdentification;

begin
  If (Identification='') then
    Identification:=ChangeFileExt(ExtractFileName(Paramstr(0)),'');
end;

Function TEventLog.EventTypeToString(E : TEventType) : String;

begin
  Case E of
    etInfo    : Result:=SLogInfo;
    etWarning : Result:=SLogWarning;
    etError   : Result:=SLogError;
    etDebug   : Result:=SLogDebug;
    etCustom  : Result:=Format(SLogCustom,[CustomLogType]);
  end;
end;

Procedure TEventLog.DoGetCustomEventID(Var Code : DWord);

begin
  If Assigned(FOnGetCustomEventID) then
    FOnGetCustomEventID(Self,Code);
end;

Procedure TEventLog.DoGetCustomEventCategory(Var Code : Word);

begin
  If Assigned(FOnGetCustomCategory) then
    FOnGetCustomCategory(Self,Code);
end;

Procedure TEventLog.DoGetCustomEvent(Var Code : DWord);

begin
  If Assigned(FOnGetCustomEvent) then
    FOnGetCustomEvent(Self,Code);
end;


destructor TEventLog.Destroy;
begin
  Active:=False;
  inherited;
end;

end.
