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
  TLogType = (ltSystem,ltFile,ltStdOut,ltStdErr);
  TLogCodeEvent = Procedure (Sender : TObject; Var Code : DWord) of Object;
  TLogCategoryEvent = Procedure (Sender : TObject; Var Code : Word) of Object;
  TLogMessageEvent = Procedure (Sender : TObject; EventType : TEventType; Const Msg : String) of Object;

  TEventLog = Class(TComponent)
  Private
    fAppendContent : Boolean;
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
    FOnLogMessage: TLogMessageEvent;
    FPaused : Boolean;
    procedure SetActive(const Value: Boolean);
    procedure SetIdentification(const Value: String);
    procedure SetlogType(const Value: TLogType);
    procedure ActivateLog;
    procedure DeActivateLog;
    procedure ActivateFileLog;
    procedure SetFileName(const Value: String);
    procedure ActivateIOLog;
    procedure ActivateSystemLog;
    function DefaultFileName: String;
    function FormatLogMessage(EventType : TEventType; const Msg: String): String;
    procedure WriteFileLog(EventType : TEventType; const Msg: String);
    procedure WriteSystemLog(EventType: TEventType; const Msg: String);
    procedure WriteIOLog(EventType: TEventType; const Msg: String; var OutFile: TextFile);
    procedure DeActivateFileLog;
    procedure DeActivateSystemLog;
    procedure CheckIdentification;
    Procedure DoGetCustomEventID(Var Code : DWord);
    Procedure DoGetCustomEventCategory(Var Code : Word);
    Procedure DoGetCustomEvent(Var Code : DWord);
    Procedure DoLogMessage(EventType : TEventType; const Msg: String);
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
    Function UnRegisterMessageFile : Boolean; virtual;
    Procedure Pause;
    Procedure Resume;
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
    Property OnLogMessage : TLogMessageEvent read FOnLogMessage write FOnLogMessage;
    Property Paused : Boolean Read FPaused Write FPaused;
  End;

  ELogError = Class(Exception);

Resourcestring

  SLogInfo      = 'Info';
  SLogWarning   = 'Warning';
  SLogError     = 'Error';
  SLogDebug     = 'Debug';
  SLogCustom    = 'Custom (%d)';
  SErrLogFailedMsg = 'Failed to log entry (Error: %s)';
  SErrLogOpenStdOut = 'Standard Output not available for logging';
  SErrLogOpenStdErr = 'Standard Error not available for logging';

implementation

{$i eventlog.inc}
(* File based dummy implementation is used for all platforms not providing
   specific implementation of eventlog.inc for the particular platform. *)

{ TEventLog }

Resourcestring
  SErrOperationNotAllowed = 'Operation not allowed when eventlog is active.';

procedure TEventLog.CheckInactive;
begin
  If Active then
    Raise ELogError.Create(SErrOperationNotAllowed);
end;

procedure TEventLog.Debug(const Fmt: String; Args: array of const);
begin
   Debug(Format(Fmt,Args));
end;

procedure TEventLog.Debug(const Msg: String);
begin
  Log(etDebug,Msg);
end;

procedure TEventLog.EnsureActive;
begin
  If Not Active then
    Active:=True;
end;


Procedure TEventLog.Pause;

begin
  Paused:=True;
end;


Procedure TEventLog.Resume;
begin
  Paused:=False;
end;


procedure TEventLog.Error(const Fmt: String; Args: array of const);
begin
  Error(Format(Fmt,Args));
end;

procedure TEventLog.Error(const Msg: String);
begin
  Log(etError,Msg);
end;

procedure TEventLog.Info(const Fmt: String; Args: array of const);
begin
  Info(Format(Fmt,Args));
end;

procedure TEventLog.Info(const Msg: String);
begin
  Log(etInfo,Msg);
end;

procedure TEventLog.Log(const Msg: String);
begin
  Log(DefaultEventType,msg);
end;

procedure TEventLog.Log(EventType: TEventType; const Fmt: String;
  Args: array of const);
begin
  Log(EventType,Format(Fmt,Args));
end;

procedure TEventLog.Log(EventType: TEventType; const Msg: String);
begin
  If Paused then 
    exit;
  EnsureActive;
  Case FlogType of
    ltFile   : WriteFileLog(EventType,Msg);
    ltSystem : WriteSystemLog(EventType,Msg);
    ltStdOut : WriteIOLog(EventType,Msg,StdOut);
    ltStdErr : WriteIOLog(EventType,Msg,StdErr);
  end;
  DoLogMessage(EventType, Msg);
end;

function TEventLog.FormatLogMessage(EventType : TEventType; const Msg: String): String;
Var
  TS,T : String;

begin
  If FTimeStampFormat='' then
    FTimeStampFormat:='yyyy-mm-dd hh:nn:ss.zzz';
  TS:=FormatDateTime(FTimeStampFormat,Now);
  T:=EventTypeToString(EventType);
  Result:=Format('%s [%s %s] %s',[Identification,TS,T,Msg]);
end;

procedure TEventLog.WriteFileLog(EventType : TEventType; const Msg : String);

Var
  S : String;

begin
  S:=FormatLogMessage(EventType, Msg)+LineEnding;
  try
    FStream.WriteBuffer(S[1],Length(S));
    S:='';
  except
    On E : Exception do
      S:=E.Message;
  end;
  If (S<>'') and RaiseExceptionOnError then
    Raise ELogError.CreateFmt(SErrLogFailedMsg,[S]);
end;

procedure TEventLog.WriteIOLog(EventType: TEventType; const Msg: String; var OutFile: TextFile);
begin
  Writeln(OutFile,FormatLogMessage(EventType,Msg));
end;

procedure TEventLog.Log(const Fmt: String; Args: array of const);
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
    ltStdOut,
    ltStdErr : ActivateIOLog;
  end;
end;

Procedure TEventLog.DeActivateLog;

begin
  Case FLogType of
    ltFile : DeActivateFileLog;
    ltSystem : DeActivateSystemLog;
    { nothing to do here }
    ltStdOut,ltStdErr : ;
  end;
end;

Procedure TEventLog.ActivateFileLog;
var
  fFileFlags : Word;
begin
  If (FFileName='') then
    FFileName:=DefaultFileName;
  // This will raise an exception if the file cannot be opened for writing !
  if fAppendContent and FileExists(FFileName) then
    fFileFlags := fmOpenWrite
  else
    fFileFlags := fmCreate;

  fFileFlags := fFileFlags or fmShareDenyWrite;
  FStream:=TFileStream.Create(FFileName,fFileFlags);
  if fAppendContent then
    FStream.Seek(0,soFromEnd);
end;

Procedure TEventLog.ActivateIOLog;

var
  errmsg: String;
  m: LongInt;

begin
  if FLogtype = ltStdOut then begin
    m := TextRec(StdOut).Mode;
    errmsg := SErrLogOpenStdOut;
  end else begin
    m := TextRec(StdErr).Mode;
    errmsg := SErrLogOpenStdErr;
  end;
  if (m <> fmOutput) and (m <> fmAppend) then
    raise ELogError.Create(errmsg);
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

procedure TEventLog.Warning(const Fmt: String; Args: array of const);
begin
  Warning(Format(Fmt,Args));
end;

procedure TEventLog.Warning(const Msg: String);
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

Procedure TEventLog.DoLogMessage(EventType : TEventType; const Msg: String);

begin
  If Assigned(FOnLogMessage) then
    FOnLogMessage(Self,EventType,Msg);
end;


destructor TEventLog.Destroy;
begin
  Active:=False;
  inherited;
end;

end.
