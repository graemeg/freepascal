{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Type
   {$ifdef processunicodestring}
   TProcessString = Unicodestring;
   TprocessChar   = UnicodeChar;

   { TProcessStrings }

   TProcessStrings = Class(TPersistent)
                private
                   name   : array of unicodestring;
                   function getcount: Integer;
                   function getname( index: integer): Unicodestring;
                public
                   procedure AssignTo(Dest: TPersistent); override;
                   procedure add(const s : Unicodestring);
                   procedure Clear;
                   procedure Delete(i:integer);

                   property Names[ index:integer]:Unicodestring read getname; default;
                   property Count : Integer read getcount;
   end;
   TProcessStringList = TProcessStrings;
   {$else}
   TprocessChar   = {Ansi}Char;
   TProcessString = {Ansi}String;
   TProcessStrings = TStrings;
   TProcessStringList = TStringList;
   {$endif}

  { TProcess }

  TProcessnamemacro = Class (TComponent)
  Private
    FOnRunCommandEvent: TOnRunCommandEvent;
    FProcessOptions : TProcessOptions;
    FRunCommandSleepTime: Integer;
    FStartupOptions : TStartupOptions;
    FFillAttribute : Cardinal;
    FApplicationName : TProcessString;
    FConsoleTitle : TProcessString;
    FCommandLine : TProcessString;
    FCurrentDirectory : TProcessString;
    FDesktop : String;
    FEnvironment : TProcessStrings;
    FExecutable : TProcessString;
    FParameters : TProcessStrings;
    FShowWindow : TShowWindowOptions;
    FInherithandles : Boolean;
    {$ifdef UNIX}
    FForkEvent : TProcessForkEvent;
    {$endif UNIX}
    FProcessPriority : TProcessPriority;
    dwXCountchars,
    dwXSize,
    dwYsize,
    dwx,
    dwYcountChars,
    dwy : Cardinal;
    FXTermProgram: String;
    FPipeBufferSize : cardinal;
    Procedure FreeStreams;
    Function  GetExitStatus : Integer;
    Function  GetExitCode : Integer;
    Function  GetRunning : Boolean;
    Function  GetWindowRect : TRect;
    procedure SetCommandLine(const AValue: TProcessString); deprecated;
    procedure SetParameters(const AValue: TProcessStrings);
    Procedure SetWindowRect (Value : TRect);
    Procedure SetShowWindow (Value : TShowWindowOptions);
    Procedure SetWindowColumns (Value : Cardinal);
    Procedure SetWindowHeight (Value : Cardinal);
    Procedure SetWindowLeft (Value : Cardinal);
    Procedure SetWindowRows (Value : Cardinal);
    Procedure SetWindowTop (Value : Cardinal);
    Procedure SetWindowWidth (Value : Cardinal);
    procedure SetApplicationName(const Value: TProcessString);
    procedure SetProcessOptions(const Value: TProcessOptions);
    procedure SetActive(const Value: Boolean);
    procedure SetEnvironment(const Value: TProcessStrings);
    Procedure ConvertCommandLine;
    function  PeekExitStatus: Boolean;
    Procedure IntOnIdleSleep(Sender,Context : TObject;Status:TRunCommandEventCode;const Message:String);
  Protected
    FRunning : Boolean;
    FExitCode : Cardinal;
    FInputStream  : TOutputPipeStream;
    FOutputStream : TInputPipeStream;
    FStderrStream : TInputPipeStream;
    FProcessID : Integer;
    FThreadID : Integer;
    FProcessHandle : Thandle;
    FThreadHandle : Thandle;
    procedure CloseProcessHandles; virtual;
    Procedure CreateStreams(InHandle,OutHandle,ErrHandle : Longint);virtual;
    procedure FreeStream(var AStream: THandleStream);
    procedure Loaded; override;
  Public
    Constructor Create (AOwner : TComponent);override;
    Destructor Destroy; override;
    Procedure Execute; virtual;
    procedure CloseInput; virtual;
    procedure CloseOutput; virtual;
    procedure CloseStderr; virtual;
    Function Resume : Integer; virtual;
    Function Suspend : Integer; virtual;
    Function Terminate (AExitCode : Integer): Boolean; virtual;
    Function WaitOnExit : Boolean;
    Function WaitOnExit(Timeout : DWord) : Boolean;
    function ReadInputStream(p:TInputPipeStream;var BytesRead:integer;var DataLength:integer;var Data:string;MaxLoops:integer=10):boolean; virtual;
    function ReadInputStream(p:TInputPipeStream;data:TStream;MaxLoops:integer=10):boolean; virtual;
    function RunCommandLoop(out outputstring:string;out stderrstring:string; out anexitstatus:integer):integer; virtual;

    Property WindowRect : Trect Read GetWindowRect Write SetWindowRect;
    Property Handle : THandle Read FProcessHandle;
    Property ProcessHandle : THandle Read FProcessHandle;
    Property ThreadHandle : THandle Read FThreadHandle;
    Property ProcessID : Integer Read FProcessID;
    Property ThreadID : Integer Read FThreadID;
    Property Input  : TOutputPipeStream Read FInputStream;
    Property Output : TInputPipeStream  Read FOutputStream;
    Property Stderr : TinputPipeStream  Read FStderrStream;
    Property ExitStatus : Integer Read GetExitStatus;
    Property ExitCode : Integer Read GetExitCode;
    Property InheritHandles : Boolean Read FInheritHandles Write FInheritHandles;
    Property OnRunCommandEvent : TOnRunCommandEvent Read FOnRunCommandEvent Write FOnRunCommandEvent;
    Property RunCommandSleepTime : Integer read FRunCommandSleepTime write FRunCommandSleepTime;
    {$ifdef UNIX}
    property OnForkEvent : TProcessForkEvent Read FForkEvent Write FForkEvent;
    {$endif UNIX}
  Published
    property PipeBufferSize : cardinal read FPipeBufferSize write FPipeBufferSize default 1024;
    Property Active : Boolean Read GetRunning Write SetActive;
    Property ApplicationName : TProcessString Read FApplicationName Write SetApplicationName; deprecated;
    Property CommandLine : TProcessString Read FCommandLine Write SetCommandLine ; deprecated;
    Property Executable : TProcessString Read FExecutable Write FExecutable;
    Property Parameters : TProcessStrings Read FParameters Write SetParameters;
    Property ConsoleTitle : TProcessString Read FConsoleTitle Write FConsoleTitle;
    Property CurrentDirectory : TProcessString Read FCurrentDirectory Write FCurrentDirectory;
    Property Desktop : String Read FDesktop Write FDesktop;
    Property Environment : TProcessStrings Read FEnvironment Write SetEnvironment;
    Property Options : TProcessOptions Read FProcessOptions Write SetProcessOptions;
    Property Priority : TProcessPriority Read FProcessPriority Write FProcessPriority;
    Property StartupOptions : TStartupOptions Read FStartupOptions Write FStartupOptions;
    Property Running : Boolean Read GetRunning;
    Property ShowWindow : TShowWindowOptions Read FShowWindow Write SetShowWindow;
    Property WindowColumns : Cardinal Read dwXCountChars Write SetWindowColumns;
    Property WindowHeight : Cardinal Read dwYSize Write SetWindowHeight;
    Property WindowLeft : Cardinal Read dwX Write SetWindowLeft;
    Property WindowRows : Cardinal Read dwYCountChars Write SetWindowRows;
    Property WindowTop : Cardinal Read dwY Write SetWindowTop ;
    Property WindowWidth : Cardinal Read dwXSize Write SetWindowWidth;
    Property FillAttribute : Cardinal read FFillAttribute Write FFillAttribute;
    Property XTermProgram : String Read FXTermProgram Write FXTermProgram;
  end;

  TProcessClass = Class of TProcessnamemacro;

Procedure CommandToList(S : TProcessString; List : TProcessStrings);

{$ifdef unix}
Var
  TryTerminals : Array of string;
  XTermProgram : String;
  Function DetectXTerm : String;
{$endif unix}

function RunCommandIndir(const curdir:TProcessString;const exename:TProcessString;const commands:array of TProcessString;out outputstring:string; out exitstatus:integer; Options : TProcessOptions = [];SWOptions:TShowWindowOptions=swoNone):integer;
function RunCommandIndir(const curdir:TProcessString;const exename:TProcessString;const commands:array of TProcessString;out outputstring:string; Options : TProcessOptions = [];SWOptions:TShowWindowOptions=swoNone):boolean;
function RunCommand(const exename:TProcessString;const commands:array of TProcessString;out outputstring:string; Options : TProcessOptions = [];SWOptions:TShowWindowOptions=swoNone):boolean;

function RunCommandInDir(const curdir,cmdline:TProcessString;out outputstring:string):boolean; deprecated;
function RunCommand(const cmdline:TProcessString;out outputstring:string):boolean; deprecated;

// Allows override of the class instantiated for RunCommand*.

var DefaultTProcess : TProcessClass = TProcessnamemacro;

implementation

{$i process.inc}

Procedure CommandToList(S : TProcessString; List : TProcessStrings);

  Function GetNextWord : TProcessString;

  Const
    WhiteSpace = [' ',#9,#10,#13];
    Literals = ['"',''''];

  Var
    Wstart,wend : Integer;
    InLiteral : Boolean;
    LastLiteral : TProcessChar;

  begin
    WStart:=1;
    While (WStart<=Length(S)) and charinset(S[WStart],WhiteSpace) do
      Inc(WStart);
    WEnd:=WStart;
    InLiteral:=False;
    LastLiteral:=#0;
    While (Wend<=Length(S)) and (Not charinset(S[Wend],WhiteSpace) or InLiteral) do
      begin
      if charinset(S[Wend],Literals) then
        If InLiteral then
          InLiteral:=Not (S[Wend]=LastLiteral)
        else
          begin
          InLiteral:=True;
          LastLiteral:=S[Wend];
          end;
       inc(wend);
       end;

     Result:=Copy(S,WStart,WEnd-WStart);

     if  (Length(Result) > 0)
     and (Result[1] = Result[Length(Result)]) // if 1st char = last char and..
     and (Result[1] in Literals) then // it's one of the literals, then
       Result:=Copy(Result, 2, Length(Result) - 2); //delete the 2 (but not others in it)

     While (WEnd<=Length(S)) and (S[Wend] in WhiteSpace) do
       inc(Wend);
     Delete(S,1,WEnd-1);

  end;

Var
  W : TProcessString;

begin
  While Length(S)>0 do
    begin
    W:=GetNextWord;
    If (W<>'') then
      List.Add(W);
    end;
end;

Constructor TProcessnamemacro.Create (AOwner : TComponent);
begin
  Inherited;
  FProcessPriority:=ppNormal;
  FShowWindow:=swoNone;
  FInheritHandles:=True;
  {$ifdef UNIX}
  FForkEvent:=nil;
  {$endif UNIX}
  FPipeBufferSize := 1024;
  FEnvironment:=TProcessStringList.Create;
  FParameters:=TProcessStringList.Create;
  FRunCommandSleepTime:=100;
  FOnRunCommandEvent:=@IntOnIdleSleep;
end;

Destructor TProcessnamemacro.Destroy;

begin
  FParameters.Free;
  FEnvironment.Free;
  FreeStreams;
  CloseProcessHandles;
  Inherited Destroy;
end;

Procedure TProcessnamemacro.FreeStreams;
begin
  If FStderrStream<>FOutputStream then
    FreeStream(THandleStream(FStderrStream));
  FreeStream(THandleStream(FOutputStream));
  FreeStream(THandleStream(FInputStream));
end;


Function TProcessnamemacro.GetExitStatus : Integer;

begin
  GetRunning;
  Result:=FExitCode;
end;

{$IFNDEF OS_HASEXITCODE}
Function TProcessnamemacro.GetExitCode : Integer;

begin
  if Not Running then
    Result:=GetExitStatus
  else
    Result:=0
end;
{$ENDIF}

Function TProcessnamemacro.GetRunning : Boolean;

begin
  IF FRunning then
    FRunning:=Not PeekExitStatus;
  Result:=FRunning;
end;


Procedure TProcessnamemacro.CreateStreams(InHandle,OutHandle,ErrHandle : Longint);

begin
  FreeStreams;
  FInputStream:=TOutputPipeStream.Create (InHandle);
  FOutputStream:=TInputPipeStream.Create (OutHandle);
  if Not (poStderrToOutput in FProcessOptions) then
    FStderrStream:=TInputPipeStream.Create(ErrHandle);
end;

procedure TProcessnamemacro.FreeStream(var AStream: THandleStream);
begin
  if AStream = nil then exit;
  FreeAndNil(AStream);
end;

procedure TProcessnamemacro.Loaded;
begin
  inherited Loaded;
  If (csDesigning in ComponentState) and (FCommandLine<>'') then
    ConvertCommandLine;
end;

procedure TProcessnamemacro.CloseInput;
begin
  FreeStream(THandleStream(FInputStream));
end;

procedure TProcessnamemacro.CloseOutput;
begin
  FreeStream(THandleStream(FOutputStream));
end;

procedure TProcessnamemacro.CloseStderr;
begin
  FreeStream(THandleStream(FStderrStream));
end;

Procedure TProcessnamemacro.SetWindowColumns (Value : Cardinal);

begin
  if Value<>0 then
    Include(FStartupOptions,suoUseCountChars);
  dwXCountChars:=Value;
end;


Procedure TProcessnamemacro.SetWindowHeight (Value : Cardinal);

begin
  if Value<>0 then
    include(FStartupOptions,suoUsePosition);
  dwYSize:=Value;
end;

Procedure TProcessnamemacro.SetWindowLeft (Value : Cardinal);

begin
  if Value<>0 then
    Include(FStartupOptions,suoUseSize);
  dwx:=Value;
end;

Procedure TProcessnamemacro.SetWindowTop (Value : Cardinal);

begin
  if Value<>0 then
    Include(FStartupOptions,suoUsePosition);
  dwy:=Value;
end;

Procedure TProcessnamemacro.SetWindowWidth (Value : Cardinal);
begin
  If (Value<>0) then
    Include(FStartupOptions,suoUseSize);
  dwXSize:=Value;
end;

Function TProcessnamemacro.GetWindowRect : TRect;
begin
  With Result do
    begin
    Left:=dwx;
    Right:=dwx+dwxSize;
    Top:=dwy;
    Bottom:=dwy+dwysize;
    end;
end;

procedure TProcessnamemacro.SetCommandLine(const AValue: TProcessString);
begin
  if FCommandLine=AValue then exit;
  FCommandLine:=AValue;
  If Not (csLoading in ComponentState) then
    ConvertCommandLine;
end;

procedure TProcessnamemacro.SetParameters(const AValue: TProcessStrings);
begin
  FParameters.Assign(AValue);
end;

Procedure TProcessnamemacro.SetWindowRect (Value : Trect);
begin
  Include(FStartupOptions,suoUseSize);
  Include(FStartupOptions,suoUsePosition);
  With Value do
    begin
    dwx:=Left;
    dwxSize:=Right-Left;
    dwy:=Top;
    dwySize:=Bottom-top;
    end;
end;


Procedure TProcessnamemacro.SetWindowRows (Value : Cardinal);

begin
  if Value<>0 then
    Include(FStartupOptions,suoUseCountChars);
  dwYCountChars:=Value;
end;

procedure TProcessnamemacro.SetApplicationName(const Value: TProcessString);
begin
  FApplicationName := Value;
  If (csDesigning in ComponentState) and
     (FCommandLine='') then
    FCommandLine:=Value;
end;

procedure TProcessnamemacro.SetProcessOptions(const Value: TProcessOptions);
begin
  FProcessOptions := Value;
  If poNewConsole in FProcessOptions then
    Exclude(FProcessOptions,poNoConsole);
  if poRunSuspended in FProcessOptions then
    Exclude(FProcessOptions,poWaitOnExit);
end;

procedure TProcessnamemacro.SetActive(const Value: Boolean);
begin
  if (Value<>GetRunning) then
    If Value then
      Execute
    else
      Terminate(0);
end;

procedure TProcessnamemacro.SetEnvironment(const Value: TProcessStrings);
begin
  FEnvironment.Assign(Value);
end;

procedure TProcessnamemacro.ConvertCommandLine;
begin
  FParameters.Clear;
  CommandToList(FCommandLine,FParameters);
  If FParameters.Count>0 then
    begin
    Executable:=FParameters[0];
    FParameters.Delete(0);
    end;
end;

Const
  READ_BYTES = 65536; // not too small to avoid fragmentation when reading large files.

function TProcessnamemacro.ReadInputStream(p:TInputPipeStream;var BytesRead:integer;var DataLength:integer;var data:string;MaxLoops:integer=10):boolean;
var Available, NumBytes: integer;
begin
    Available:=P.NumBytesAvailable;
    result:=Available>0;
    if not result then
     exit;
    while (available > 0) and (MaxLoops>0) do
      begin
        if (BytesRead + available > DataLength) then
          begin
            DataLength:=BytesRead + max(READ_BYTES,available);
            Setlength(Data,DataLength);
          end;
        NumBytes := p.Read(data[1+BytesRead], Available);
        if NumBytes > 0 then
          Inc(BytesRead, NumBytes);
        Available:=P.NumBytesAvailable;
        dec(MaxLoops);
      end;
end;

function TProcessnamemacro.ReadInputStream(p:TInputPipeStream;data:TStream;MaxLoops:integer=10):boolean;
const
  BufSize = 4096;
var
  Buffer: array[0..BufSize - 1] of byte;
  ReadBytes: integer;
  Available : integer;
begin
  Available:=P.NumBytesAvailable;
  result:=Available>0;
  if not result then
    Exit;
  while (available > 0) and (MaxLoops>0) do
  begin
    ReadBytes := Output.Read({%H-}Buffer, min(BufSize,Available));
    data.Write(Buffer, ReadBytes);
    Available:=P.NumBytesAvailable;
    dec(MaxLoops);
  end;
end;

procedure TProcessnamemacro.IntOnIdleSleep(Sender,Context : TObject;status:TRunCommandEventCode;const message:string);
begin
  if status=RunCommandIdle then
    sleep(FRunCommandSleepTime);
end;

// helperfunction that does the bulk of the work.
// We need to also collect stderr output in order to avoid
// lock out if the stderr pipe is full.
function TProcessnamemacro.RunCommandLoop(out outputstring:string;
                            out stderrstring:string; out anexitstatus:integer):integer;
var
    bytesread : integer;
    outputlength, stderrlength : integer;
    stderrbytesread : integer;
    gotoutput,gotoutputstderr : boolean;
begin
  result:=-1;
    try
    Options := Options + [poUsePipes];
    bytesread:=0;
    outputlength:=0;
    stderrbytesread:=0;
    stderrlength:=0;
    Execute;
    while Running do
      begin
        // Only call ReadFromStream if Data from corresponding stream
        // is already available, otherwise, on  linux, the read call
        // is blocking, and thus it is not possible to be sure to handle
        // big data amounts bboth on output and stderr pipes. PM.
        gotoutput:=ReadInputStream(output,BytesRead,OutputLength,OutputString,1);
        // The check for assigned(P.stderr) is mainly here so that
        // if we use poStderrToOutput in p.Options, we do not access invalid memory.
        gotoutputstderr:=false;
        if assigned(stderr) then
            gotoutputstderr:=ReadInputStream(StdErr,StdErrBytesRead,StdErrLength,StdErrString,1);
 
        if (porunidle in options) and not gotoutput and not gotoutputstderr and Assigned(FOnRunCommandEvent) Then
          FOnRunCommandEvent(self,Nil,RunCommandIdle,'');
      end;
    // Get left output after end of execution
    ReadInputStream(output,BytesRead,OutputLength,OutputString,250);
    setlength(outputstring,BytesRead);
    if assigned(stderr) then
      ReadInputStream(StdErr,StdErrBytesRead,StdErrLength,StdErrString,250);
    setlength(stderrstring,StderrBytesRead);
    anexitstatus:=exitstatus;
    result:=0; // we came to here, document that.
    if Assigned(FOnRunCommandEvent) then          // allow external apps to react to that and finish GUI
      FOnRunCommandEvent(self,Nil,RunCommandFinished,'');

    except
      on e : Exception do
         begin
           result:=1;
           setlength(outputstring,BytesRead);
           setlength(stderrstring,StderrBytesRead);
           if Assigned(FOnRunCommandEvent) then
             FOnRunCommandEvent(self,Nil,RunCommandException,e.Message);
         end;
     end;
end;

{ Functions without StderrString }

Const
  ForbiddenOptions = [poRunSuspended,poWaitOnExit];

function RunCommandIndir(const curdir:TProcessString;const exename:TProcessString;const commands:array of TProcessString;out outputstring:string;out exitstatus:integer; Options : TProcessOptions = [];SWOptions:TShowWindowOptions=swoNone):integer;
Var
    p : TProcessnamemacro;
    i : integer;
    ErrorString : String;
begin
  p:=DefaultTProcess.create(nil);
  if Options<>[] then
    P.Options:=Options - ForbiddenOptions;
  P.ShowWindow:=SwOptions;
  p.Executable:=exename;
  if curdir<>'' then
    p.CurrentDirectory:=curdir;
  if high(commands)>=0 then
   for i:=low(commands) to high(commands) do
     p.Parameters.add(commands[i]);
  try
    result:=p.RunCommandLoop(outputstring,errorstring,exitstatus);
  finally
    p.free;
  end;
end;

function RunCommandInDir(const curdir,cmdline:TProcessString;out outputstring:string):boolean; deprecated;
Var
    p : TProcessnamemacro;
    exitstatus : integer;
    ErrorString : String;
begin
  p:=DefaultTProcess.create(nil);
  p.setcommandline(cmdline);
  if curdir<>'' then
    p.CurrentDirectory:=curdir;
  try
    result:=p.RunCommandLoop(outputstring,errorstring,exitstatus)=0;
  finally
    p.free;
  end;
  if exitstatus<>0 then result:=false;
end;

function RunCommandIndir(const curdir:TProcessString;const exename:TProcessString;const commands:array of TProcessString;out outputstring:string; Options : TProcessOptions = [];SWOptions:TShowWindowOptions=swoNone):boolean;
Var
    p : TProcessnamemacro;
    i,
    exitstatus : integer;
    ErrorString : String;
begin
  p:=DefaultTProcess.create(nil);
  if Options<>[] then
    P.Options:=Options - ForbiddenOptions;
  P.ShowWindow:=SwOptions;
  p.Executable:=exename;
  if curdir<>'' then
    p.CurrentDirectory:=curdir;
  if high(commands)>=0 then
   for i:=low(commands) to high(commands) do
     p.Parameters.add(commands[i]);
  try
    result:=p.RunCommandLoop(outputstring,errorstring,exitstatus)=0;
  finally
    p.free;
  end;
  if exitstatus<>0 then result:=false;
end;

function RunCommand(const cmdline:TProcessString;out outputstring:String):boolean; deprecated;
Var
    p : TProcessnamemacro;
    exitstatus : integer;
    ErrorString : String;
begin
  p:=DefaultTProcess.create(nil);
  p.setcommandline(cmdline);
  try
    result:=p.RunCommandLoop(outputstring,errorstring,exitstatus)=0;
  finally
    p.free;
  end;
  if exitstatus<>0 then result:=false;
end;

function RunCommand(const exename:TProcessString;const commands:array of TProcessString;out outputstring:string; Options : TProcessOptions = [];SWOptions:TShowWindowOptions=swoNone):boolean;
Var
    p : TProcessnamemacro;
    i,
    exitstatus : integer;
    ErrorString : String;
begin
  p:=DefaultTProcess.create(nil);
  if Options<>[] then
    P.Options:=Options - ForbiddenOptions;
  P.ShowWindow:=SwOptions;
  p.Executable:=exename;
  if high(commands)>=0 then
   for i:=low(commands) to high(commands) do
     p.Parameters.add(commands[i]);
  try
    result:=p.RunCommandLoop(outputstring,errorstring,exitstatus)=0;
  finally
    p.free;
  end;
  if exitstatus<>0 then result:=false;
end;

{$ifdef processunicodestring}
// dummy subset of tstrings.
{ TProcessStrings }

function TProcessStrings.getname( index: integer): Unicodestring;
begin
  if index<length(name) then
     result:=name[index]
  else
     result:='';
end;

function TProcessStrings.getcount: Integer;
begin
  result:=length(name);
end;

procedure TProcessStrings.AssignTo(Dest: TPersistent);
var i : integer;
begin
  inherited assign(dest);
  if dest is TStrings then
    begin
      setlength(name,tstrings(dest).count);
      for i:=0 to length(name)-1 do
        name[i]:=tstrings(dest)[i];
    end;
  if dest is tprocessstrings then
     name:=copy(tprocessstrings(dest).name);
end;

procedure TProcessStrings.add(const s: Unicodestring);
var len : integer;
begin
  len:=length(name);
  setlength(name, len+1);
  name[len]:=s;
end;

procedure TProcessStrings.Clear;
begin
 setlength(name,0);
end;

procedure TProcessStrings.Delete(i: integer);
var len,j : integer;
begin
  len:=length(name);
  if len=0 then exit;
  if (i<>len-1) and (len<>1) then
     begin
       for j:=i+1 to len-1 do
         name[j-1]:=name[j];
       setlength(name,len-1)
     end
  else
    setlength(name,len-1)
end;
{$endif}

end.
