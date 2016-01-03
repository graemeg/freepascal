{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team

    CustomApplication class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit CustApp;

Interface

<<<<<<< HEAD
<<<<<<< HEAD
uses SysUtils,Classes,singleinstance;

Type
  TStringArray = Array of string;
  TExceptionEvent = Procedure (Sender : TObject; E : Exception) Of Object;
  TEventLogTypes = Set of TEventType;

  { TCustomApplication }

  TCustomApplication = Class(TComponent)
  Private
    FEventLogFilter: TEventLogTypes;
    FOnException: TExceptionEvent;
    FSingleInstance: TBaseSingleInstance;
    FSingleInstanceClass: TBaseSingleInstanceClass; // set before FSingleInstance is created
    FSingleInstanceEnabled: Boolean; // set before Initialize is called
=======
=======
>>>>>>> origin/fixes_2_2
uses SysUtils,Classes;

Type
  TExceptionEvent = Procedure (Sender : TObject; E : Exception) Of Object;

  TCustomApplication = Class(TComponent)
  Private
    FOnException: TExceptionEvent;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FTerminated : Boolean;
    FHelpFile,
    FTitle : String;
    FOptionChar : Char;
    FCaseSensitiveOptions : Boolean;
    FStopOnException : Boolean;
    function GetEnvironmentVar(VarName : String): String;
    function GetExeName: string;
    Function GetLocation : String;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetSingleInstance: TBaseSingleInstance;
    procedure SetSingleInstanceClass(
      const ASingleInstanceClass: TBaseSingleInstanceClass);
    function GetTitle: string;
  Protected
    function GetOptionAtIndex(AIndex: Integer; IsLong: Boolean): String;
=======
    function GetTitle: string;
  Protected
>>>>>>> graemeg/fixes_2_2
=======
    function GetTitle: string;
  Protected
>>>>>>> origin/fixes_2_2
    procedure SetTitle(const AValue: string); Virtual;
    Function GetConsoleApplication : boolean; Virtual;
    Procedure DoRun; Virtual;
    Function GetParams(Index : Integer) : String;virtual;
    function GetParamCount: Integer;Virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure DoLog(EventType : TEventType; const Msg : String);  virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    // Some Delphi methods.
    procedure HandleException(Sender: TObject); virtual;
    procedure Initialize; virtual;
    procedure Run;
    procedure ShowException(E: Exception);virtual;
    procedure Terminate; virtual;
    // Extra methods.
<<<<<<< HEAD
<<<<<<< HEAD
    function FindOptionIndex(Const S : String; Var Longopt : Boolean; StartAt : Integer = -1) : Integer;
    Function GetOptionValue(Const S : String) : String;
    Function GetOptionValue(Const C: Char; Const S : String) : String;
    Function GetOptionValues(Const C: Char; Const S : String) : TStringArray;
    Function HasOption(Const S : String) : Boolean;
    Function HasOption(Const C : Char; Const S : String) : Boolean;
    Function CheckOptions(Const ShortOptions : String; Const Longopts : TStrings; Opts,NonOpts : TStrings; AllErrors : Boolean = False) : String;
    Function CheckOptions(Const ShortOptions : String; Const Longopts : Array of string; Opts,NonOpts : TStrings; AllErrors : Boolean = False) : String;
    Function CheckOptions(Const ShortOptions : String; Const Longopts : TStrings; AllErrors : Boolean = False) : String;
    Function CheckOptions(Const ShortOptions : String; Const LongOpts : Array of string; AllErrors : Boolean = False) : String;
    Function CheckOptions(Const ShortOptions : String; Const LongOpts : String; AllErrors : Boolean = False) : String;
    Function GetNonOptions(Const ShortOptions : String; Const Longopts : Array of string) : TStringArray;
    Procedure GetNonOptions(Const ShortOptions : String; Const Longopts : Array of string; NonOptions : TStrings);
    Procedure GetEnvironmentList(List : TStrings;NamesOnly : Boolean);
    Procedure GetEnvironmentList(List : TStrings);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Log(EventType : TEventType; const Msg : String);
    Procedure Log(EventType : TEventType; const Fmt : String; const Args : array of const);
=======
    Procedure Log(EventType : TEventType; const Msg : String); virtual;
>>>>>>> graemeg/cpstrnew
=======
    Procedure Log(EventType : TEventType; const Msg : String); virtual;
>>>>>>> graemeg/cpstrnew
=======
    Procedure Log(EventType : TEventType; const Msg : String); virtual;
>>>>>>> graemeg/cpstrnew
=======
    Procedure Log(EventType : TEventType; const Msg : String); virtual;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    function FindOptionIndex(Const S : String; Var Longopt : Boolean) : Integer;
    Function GetOptionValue(Const S : String) : String;
    Function GetOptionValue(Const C: Char; Const S : String) : String;
    Function HasOption(Const S : String) : Boolean;
    Function HasOption(Const C : Char; Const S : String) : Boolean;
    Function CheckOptions(Const ShortOptions : String; Const Longopts : TStrings; Opts,NonOpts : TStrings) : String;
    Function CheckOptions(Const ShortOptions : String; Const Longopts : TStrings) : String;
    Function CheckOptions(Const ShortOptions : String; Const LongOpts : Array of string) : String;
    Function CheckOptions(Const ShortOptions : String; Const LongOpts : String) : String;
    Procedure GetEnvironmentList(List : TStrings;NamesOnly : Boolean);
    Procedure GetEnvironmentList(List : TStrings);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Delphi properties
    property ExeName: string read GetExeName;
    property HelpFile: string read FHelpFile write FHelpFile;
    property Terminated: Boolean read FTerminated;
    property Title: string read FTitle write SetTitle;
    property OnException: TExceptionEvent read FOnException write FOnException;
    // Extra properties
    Property ConsoleApplication : Boolean Read GetConsoleApplication;
    Property Location : String Read GetLocation;
    Property Params [Index : integer] : String Read GetParams;
    Property ParamCount : Integer Read GetParamCount;
    Property EnvironmentVariable[envName : String] : String Read GetEnvironmentVar;
    Property OptionChar : Char Read FoptionChar Write FOptionChar;
    Property CaseSensitiveOptions : Boolean Read FCaseSensitiveOptions Write FCaseSensitiveOptions;
    Property StopOnException : Boolean Read FStopOnException Write FStopOnException;
<<<<<<< HEAD
<<<<<<< HEAD
    Property EventLogFilter : TEventLogTypes Read FEventLogFilter Write FEventLogFilter;
    Property SingleInstance: TBaseSingleInstance read GetSingleInstance;
    Property SingleInstanceClass: TBaseSingleInstanceClass read FSingleInstanceClass write SetSingleInstanceClass;
    Property SingleInstanceEnabled: Boolean read FSingleInstanceEnabled write FSingleInstanceEnabled;
  end;

var CustomApplication : TCustomApplication = nil;

Implementation

{$ifdef darwin}
uses
  MacOSAll;
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
  end;

Implementation

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ TCustomApplication }

function TCustomApplication.GetExeName: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$if defined(darwin)}
var
  mainBundle: CFBundleRef;
  executableUrl: CFURLRef;
  executableFSPath: CFStringRef;
  utf16len: ptrint;
  error: boolean;
begin
  error:=false;
  { Get main bundle. This even works most of the time for command line
    applications
  }
  mainbundle:=CFBundleGetMainBundle;
  if assigned(mainbundle) then
    begin
      { get the URL pointing to the executable of the bundle }
      executableUrl:=CFBundleCopyExecutableURL(mainBundle);
      if assigned(executableUrl) then
        begin
          { convert the url to a POSIX path }
          executableFSPath:=CFURLCopyFileSystemPath(executableUrl,kCFURLPOSIXPathStyle);
          CFRelease(executableUrl);
          { convert to UTF-8 -- this is not really clean since in theory the
            ansi-encoding could be different, but
              a) all file i/o routines on Darwin expect utf-8-encoded strings
              b) there is no easy way to convert the Unix LANG encoding
                 setting to an equivalent CoreFoundation encoding
          }
          utf16len:=CFStringGetLength(executableFSPath);
          // +1 for extra terminating #0 in the worst case, so the pos below
          // will always find the #0
          setlength(result,utf16len*3+1);
          if CFStringGetCString(executableFSPath,@result[1],length(result),kCFStringEncodingUTF8) then
            { truncate to actual length, #0 cannot appear in a file path }
            setlength(result,pos(#0,result)-1)
          else
            error:=true;
          CFRelease(executableFSPath);
        end
      else
        error:=true;
    end
  else
    error:=true;
  if error then
    { can't do better than this }
    Result:=Paramstr(0);
end;
{$else darwin}
begin
  Result:=Paramstr(0);
end;
{$endif darwin}
=======
begin
  Result:=Paramstr(0);
end;
>>>>>>> graemeg/fixes_2_2
=======
begin
  Result:=Paramstr(0);
end;
>>>>>>> origin/fixes_2_2

Procedure SysGetEnvironmentList(List : TStrings;NamesOnly : Boolean);

var
   s : string;
   i,l,j,count : longint;

begin
  count:=GetEnvironmentVariableCount;
  if count>0 then
    for j:=1 to count  do
     begin
       s:=GetEnvironmentString(j);
       l:=Length(s);
       If NamesOnly then
          begin
            I:=pos('=',s);
            If (I>0) then
              S:=Copy(S,1,I-1);
          end;
       List.Add(S);
    end;
end;

function TCustomApplication.GetEnvironmentVar(VarName : String): String;
begin
  Result:=GetEnvironmentVariable(VarName);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomApplication.GetEnvironmentList(List: TStrings;
  NamesOnly: Boolean);
=======
Procedure TCustomApplication.GetEnvironmentList(List : TStrings;NamesOnly : Boolean);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TCustomApplication.GetEnvironmentList(List : TStrings;NamesOnly : Boolean);
>>>>>>> origin/fixes_2_2

begin
  // Routine must be in custapp.inc
  SysGetEnvironmentList(List,NamesOnly);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomApplication.GetEnvironmentList(List: TStrings);
=======
Procedure TCustomApplication.GetEnvironmentList(List : TStrings);
>>>>>>> graemeg/fixes_2_2
=======
Procedure TCustomApplication.GetEnvironmentList(List : TStrings);
>>>>>>> origin/fixes_2_2

begin
  GetEnvironmentList(List,False);
end;

function TCustomApplication.GetLocation: String;
begin
  Result:=ExtractFilePath(GetExeName);
end;

function TCustomApplication.GetParamCount: Integer;
begin
   Result:=System.ParamCount;
end;

function TCustomApplication.GetTitle: string;
begin
  Result:=FTitle;
end;

function TCustomApplication.GetParams(Index: Integer): String;
begin
  Result:=ParamStr(Index);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.GetSingleInstance: TBaseSingleInstance;
begin
  if FSingleInstance = nil then
    begin
    if FSingleInstanceClass=Nil then
      Raise ESingleInstance.Create('No single instance provider class set! Include a single-instance class unit such as advsingleinstance');
    FSingleInstance := FSingleInstanceClass.Create(Self);
    end;
  Result := FSingleInstance;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TCustomApplication.SetTitle(const AValue: string);
begin
  FTitle:=AValue;
end;

function TCustomApplication.GetConsoleApplication: boolean;
begin
  Result:=IsConsole;
end;

procedure TCustomApplication.DoRun;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if Assigned(FSingleInstance) then
    if FSingleInstance.IsServer then
      FSingleInstance.ServerCheckMessages;

  // Override in descendent classes.
end;

procedure TCustomApplication.DoLog(EventType: TEventType; const Msg: String);

begin
  // Do nothing, override in descendants
end;

procedure TCustomApplication.Log(EventType: TEventType; const Msg: String);

begin
  If (FEventLogFilter=[]) or (EventType in FEventLogFilter) then
    DoLog(EventType,Msg);
end;

procedure TCustomApplication.Log(EventType: TEventType; const Fmt: String;
  const Args: array of const);
begin
  try
    Log(EventType, Format(Fmt, Args));
  except
    On E : Exception do
      Log(etError,Format('Error formatting message "%s" with %d arguments: %s',[Fmt,Length(Args),E.Message]));
  end  
end;

Procedure TCustomApplication.Log(EventType : TEventType; const Msg : String);

begin
  // Do nothing. Override in descendent classes.
end;

Procedure TCustomApplication.Log(EventType : TEventType; const Msg : String);

begin
  // Do nothing. Override in descendent classes.
end;

Procedure TCustomApplication.Log(EventType : TEventType; const Msg : String);

begin
  // Do nothing. Override in descendent classes.
end;

Procedure TCustomApplication.Log(EventType : TEventType; const Msg : String);

begin
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // Do nothing. Override in descendent classes.
end;

constructor TCustomApplication.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FOptionChar:='-';
  FCaseSensitiveOptions:=True;
  FStopOnException:=False;
<<<<<<< HEAD
<<<<<<< HEAD
  FSingleInstanceClass := DefaultSingleInstanceClass;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TCustomApplication.Destroy;
begin
  inherited Destroy;
end;

procedure TCustomApplication.HandleException(Sender: TObject);
begin
  If Not (ExceptObject is Exception) then
    SysUtils.showexception(ExceptObject,ExceptAddr)
  else
    begin
    If Not Assigned(FOnexception) then
      ShowException(Exception(ExceptObject))
    else
      FOnException(Sender,Exception(ExceptObject));
    end;
  If FStopOnException then
    FTerminated:=True;
end;


procedure TCustomApplication.Initialize;
begin
  FTerminated:=False;
<<<<<<< HEAD
<<<<<<< HEAD
  if FSingleInstanceEnabled then
  begin
    case SingleInstance.Start of
      siClient:
      begin
        SingleInstance.ClientPostParams;
        FTerminated:=True;
      end;
      siNotResponding:
        FTerminated:=True;
    end;
  end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TCustomApplication.Run;

begin
  Repeat
    Try
      DoRun;
    except
      HandleException(Self);
    end;
  Until FTerminated;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TCustomApplication.SetSingleInstanceClass(
  const ASingleInstanceClass: TBaseSingleInstanceClass);
begin
  Assert((FSingleInstance = nil) and (ASingleInstanceClass <> nil));
  FSingleInstanceClass := ASingleInstanceClass;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TCustomApplication.ShowException(E: Exception);

begin
  Sysutils.ShowException(E,ExceptAddr)
end;

procedure TCustomApplication.Terminate;
begin
  FTerminated:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.GetOptionAtIndex(AIndex : Integer; IsLong: Boolean): String;

Var
  P : Integer;
  O : String;

begin
  Result:='';
  If (AIndex=-1) then
    Exit;
  If IsLong then
    begin // Long options have form --option=value
    O:=Params[AIndex];
    P:=Pos('=',O);
   If (P=0) then
      P:=Length(O);
    Delete(O,1,P);
    Result:=O;
    end
  else
    begin // short options have form '-o value'
    If (AIndex<ParamCount) then
      if (Copy(Params[AIndex+1],1,1)<>'-') then
        Result:=Params[AIndex+1];
    end;
  end;


function TCustomApplication.GetOptionValue(const S: String): String;
=======
function TCustomApplication.GetOptionValue(Const S: String): String;
>>>>>>> graemeg/fixes_2_2
=======
function TCustomApplication.GetOptionValue(Const S: String): String;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetoptionValue(#255,S);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.GetOptionValue(const C: Char; const S: String
  ): String;

Var
  B : Boolean;
  I : integer;
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomApplication.GetOptionValue(Const C: Char; Const S: String): String;

Var
  B : Boolean;
  I,P : integer;
  O : String;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result:='';
  I:=FindOptionIndex(C,B);
  If (I=-1) then
<<<<<<< HEAD
<<<<<<< HEAD
    I:=FindOptionIndex(S,B);
  If I<>-1 then
    Result:=GetOptionAtIndex(I,B);
end;

function TCustomApplication.GetOptionValues(const C: Char; const S: String): TStringArray;

Var
  I,Cnt : Integer;
  B : Boolean;

begin
  SetLength(Result,ParamCount);
  Cnt:=0;
  Repeat
    I:=FindOptionIndex(C,B,I);
    If I<>-1 then
      begin
      Inc(Cnt);
      Dec(I);
      end;
  Until I=-1;
  Repeat
    I:=FindOptionIndex(S,B,I);
    If I<>-1 then
      begin
      Inc(Cnt);
      Dec(I);
      end;
  Until I=-1;
  SetLength(Result,Cnt);
  Cnt:=0;
  I:=-1;
  Repeat
    I:=FindOptionIndex(C,B,I);
    If (I<>-1) then
      begin
      Result[Cnt]:=GetOptionAtIndex(I,False);
      Inc(Cnt);
      Dec(i);
      end;
  Until (I=-1);
  I:=-1;
  Repeat
    I:=FindOptionIndex(S,B,I);
    If I<>-1 then
      begin
      Result[Cnt]:=GetOptionAtIndex(I,True);
      Inc(Cnt);
      Dec(i);
      end;
  Until (I=-1);
end;

function TCustomApplication.HasOption(const S: String): Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    I:=FindoptionIndex(S,B);
  If (I<>-1) then
    begin
    If B then
      begin // Long options have form --option=value
      O:=Params[I];
      P:=Pos('=',O);
      If (P=0) then
        P:=Length(O);
      Delete(O,1,P);
      Result:=O;
      end
    else
      begin // short options have form '-o value'
      If (I<ParamCount) then
        Result:=Params[I+1];
      end;
    end;
end;

function TCustomApplication.HasOption(Const S: String): Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Var
  B : Boolean;

begin
  Result:=FindOptionIndex(S,B)<>-1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.FindOptionIndex(const S: String;
  var Longopt: Boolean; StartAt : Integer = -1): Integer;
=======
function TCustomApplication.FindOptionIndex(Const S : String; Var Longopt : Boolean) : Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TCustomApplication.FindOptionIndex(Const S : String; Var Longopt : Boolean) : Integer;
>>>>>>> origin/fixes_2_2

Var
  SO,O : String;
  I,P : Integer;

begin
  If Not CaseSensitiveOptions then
    SO:=UpperCase(S)
  else
    SO:=S;
  Result:=-1;
<<<<<<< HEAD
<<<<<<< HEAD
  I:=StartAt;
  if (I=-1) then
    I:=ParamCount;
  While (Result=-1) and (I>0) do
    begin
    O:=Params[i];
    // - must be seen as an option value
    If (Length(O)>1) and (O[1]=FOptionChar) then
=======
=======
>>>>>>> origin/fixes_2_2
  I:=ParamCount;
  While (Result=-1) and (I>0) do
    begin
    O:=Params[i];
    If (Length(O)>0) and (O[1]=FOptionChar) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
      Delete(O,1,1);
      LongOpt:=(Length(O)>0) and (O[1]=FOptionChar);
      If LongOpt then
        begin
        Delete(O,1,1);
        P:=Pos('=',O);
        If (P<>0) then
          O:=Copy(O,1,P-1);
        end;
      If Not CaseSensitiveOptions then
        O:=UpperCase(O);
      If (O=SO) then
        Result:=i;
      end;
    Dec(i);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.HasOption(const C: Char; const S: String): Boolean;
=======
function TCustomApplication.HasOption(Const C: Char; Const S: String): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TCustomApplication.HasOption(Const C: Char; Const S: String): Boolean;
>>>>>>> origin/fixes_2_2

Var
  B : Boolean;

begin
  Result:=(FindOptionIndex(C,B)<>-1) or (FindOptionIndex(S,B)<>-1);
end;


<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.CheckOptions(const ShortOptions: String;
  const Longopts: TStrings; AllErrors: Boolean): String;

begin
  Result:=CheckOptions(ShortOptions,LongOpts,Nil,Nil,AllErrors);
=======
=======
>>>>>>> origin/fixes_2_2
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const Longopts : TStrings) : String;

begin
  Result:=CheckOptions(ShortOptions,LongOpts,Nil,Nil);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

ResourceString
  SErrInvalidOption = 'Invalid option at position %d: "%s"';
  SErrNoOptionAllowed = 'Option at position %d does not allow an argument: %s';
  SErrOptionNeeded = 'Option at position %d needs an argument : %s';

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.CheckOptions(const ShortOptions: String;
  const Longopts: TStrings; Opts, NonOpts: TStrings; AllErrors: Boolean
  ): String;
=======
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const Longopts : TStrings; Opts,NonOpts : TStrings) : String;
>>>>>>> graemeg/fixes_2_2
=======
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const Longopts : TStrings; Opts,NonOpts : TStrings) : String;
>>>>>>> origin/fixes_2_2

Var
  I,J,L,P : Integer;
  O,OV,SO : String;
<<<<<<< HEAD
<<<<<<< HEAD
  UsedArg,HaveArg : Boolean;
=======
  HaveArg : Boolean;
>>>>>>> graemeg/fixes_2_2
=======
  HaveArg : Boolean;
>>>>>>> origin/fixes_2_2

  Function FindLongOpt(S : String) : boolean;

  Var
    I : integer;

  begin
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=Assigned(LongOpts);
    if Not Result then
      exit;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    If CaseSensitiveOptions then
      begin
      I:=LongOpts.Count-1;
      While (I>=0) and (LongOpts[i]<>S) do
        Dec(i);
      end
    else
      begin
      S:=UpperCase(S);
      I:=LongOpts.Count-1;
      While (I>=0) and (UpperCase(LongOpts[i])<>S) do
        Dec(i);
      end;
    Result:=(I<>-1);
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  Procedure AddToResult(Const Msg : string);

  begin
    If (Result<>'') then
      Result:=Result+sLineBreak;
    Result:=Result+Msg;
  end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  If CaseSensitiveOptions then
    SO:=Shortoptions
  else
    SO:=LowerCase(Shortoptions);
  Result:='';
  I:=1;
<<<<<<< HEAD
<<<<<<< HEAD
  While (I<=ParamCount) and ((Result='') or AllErrors) do
=======
  While (I<=ParamCount) and (Result='') do
>>>>>>> graemeg/fixes_2_2
=======
  While (I<=ParamCount) and (Result='') do
>>>>>>> origin/fixes_2_2
    begin
    O:=Paramstr(I);
    If (Length(O)=0) or (O[1]<>FOptionChar) then
      begin
      If Assigned(NonOpts) then
        NonOpts.Add(O)
      end
    else
      begin
      If (Length(O)<2) then
<<<<<<< HEAD
<<<<<<< HEAD
        AddToResult(Format(SErrInvalidOption,[i,O]))
=======
        Result:=Format(SErrInvalidOption,[i,O])
>>>>>>> graemeg/fixes_2_2
=======
        Result:=Format(SErrInvalidOption,[i,O])
>>>>>>> origin/fixes_2_2
      else
        begin
        HaveArg:=False;
        OV:='';
        // Long option ?
        If (O[2]=FOptionChar) then
          begin
          Delete(O,1,2);
          J:=Pos('=',O);
          If J<>0 then
            begin
            HaveArg:=true;
            OV:=O;
            Delete(OV,1,J);
            O:=Copy(O,1,J-1);
            end;
          // Switch Option
          If FindLongopt(O) then
            begin
            If HaveArg then
<<<<<<< HEAD
<<<<<<< HEAD
              AddToResult(Format(SErrNoOptionAllowed,[I,O]))
=======
              Result:=Format(SErrNoOptionAllowed,[I,O])
>>>>>>> graemeg/fixes_2_2
=======
              Result:=Format(SErrNoOptionAllowed,[I,O])
>>>>>>> origin/fixes_2_2
            end
          else
            begin // Required argument
            If FindLongOpt(O+':') then
              begin
              If Not HaveArg then
<<<<<<< HEAD
<<<<<<< HEAD
                AddToResult(Format(SErrOptionNeeded,[I,O]));
=======
                Result:=Format(SErrOptionNeeded,[I,O]);
>>>>>>> graemeg/fixes_2_2
=======
                Result:=Format(SErrOptionNeeded,[I,O]);
>>>>>>> origin/fixes_2_2
              end
            else
              begin // Optional Argument.
              If not FindLongOpt(O+'::') then
<<<<<<< HEAD
<<<<<<< HEAD
                AddToResult(Format(SErrInvalidOption,[I,O]));
=======
                Result:=Format(SErrInvalidOption,[I,O]);
>>>>>>> graemeg/fixes_2_2
=======
                Result:=Format(SErrInvalidOption,[I,O]);
>>>>>>> origin/fixes_2_2
              end;
            end;
          end
        else // Short Option.
          begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          HaveArg:=(I<ParamCount) and (Length(ParamStr(I+1))>0) and (ParamStr(I+1)[1]<>FOptionChar);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          UsedArg:=False;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
          HaveArg:=(I<ParamCount) and (Length(ParamStr(I+1))>0) and (ParamStr(I+1)[i]<>FOptionChar);
>>>>>>> graemeg/fixes_2_2
=======
          HaveArg:=(I<ParamCount) and (Length(ParamStr(I+1))>0) and (ParamStr(I+1)[i]<>FOptionChar);
>>>>>>> origin/fixes_2_2
=======
          HaveArg:=(I<ParamCount) and (Length(ParamStr(I+1))>0) and (ParamStr(I+1)[1]<>FOptionChar);
>>>>>>> origin/fixes_2.4
          If HaveArg then
            OV:=Paramstr(I+1);
          If Not CaseSensitiveOptions then
            O:=LowerCase(O);
          L:=Length(O);
          J:=2;
<<<<<<< HEAD
<<<<<<< HEAD
          While ((Result='') or AllErrors) and (J<=L) do
            begin
            P:=Pos(O[J],ShortOptions);
            If (P=0) or (O[j]=':') then
              AddToResult(Format(SErrInvalidOption,[I,O[J]]))
=======
=======
>>>>>>> origin/fixes_2_2
          While (result='') and (J<=L) do
            begin
            P:=Pos(O[J],ShortOptions);
            If (P=0) or (O[j]=':') then
              Result:=Format(SErrInvalidOption,[I,O[J]])
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            else
              begin
              If (P<Length(ShortOptions)) and (Shortoptions[P+1]=':') then
                begin
                // Required argument
                If ((P+1)=Length(ShortOptions)) or (Shortoptions[P+2]<>':') Then
                  If (J<L) or not haveArg then // Must be last in multi-opt !!
<<<<<<< HEAD
<<<<<<< HEAD
                    AddToResult(Format(SErrOptionNeeded,[I,O[J]]));
                O:=O[j]; // O is added to arguments.
                UsedArg:=True;
=======
                    Result:=Format(SErrOptionNeeded,[I,O[J]]);
                O:=O[j]; // O is added to arguments.
>>>>>>> graemeg/fixes_2_2
=======
                    Result:=Format(SErrOptionNeeded,[I,O[J]]);
                O:=O[j]; // O is added to arguments.
>>>>>>> origin/fixes_2_2
                end;
              end;
            Inc(J);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          If HaveArg and UsedArg then
=======
          If HaveArg then
>>>>>>> graemeg/fixes_2_2
=======
          If HaveArg then
>>>>>>> origin/fixes_2_2
            begin
            Inc(I); // Skip argument.
            O:=O[Length(O)]; // O is added to arguments !
            end;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
        If HaveArg and ((Result='') or AllErrors) then
=======
        If HaveArg and (Result='') then
>>>>>>> graemeg/fixes_2_2
=======
        If HaveArg and (Result='') then
>>>>>>> origin/fixes_2_2
          If Assigned(Opts) then
            Opts.Add(O+'='+OV);
        end;
      end;
    Inc(I);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.CheckOptions(const ShortOptions: String;
  const Longopts: array of string; Opts, NonOpts: TStrings; AllErrors: Boolean
  ): String;
Var
  L : TStringList;
  I : Integer;

begin
  L:=TStringList.Create;
  Try
    For I:=0 to High(LongOpts) do
      L.Add(LongOpts[i]);
    Result:=CheckOptions(ShortOptions,L,Opts,NonOpts,AllErrors);
  Finally
    L.Free;
  end;
end;

function TCustomApplication.CheckOptions(const ShortOptions: String;
  const LongOpts: array of string; AllErrors: Boolean): String;
=======
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const LongOpts : Array of string) : String;
>>>>>>> graemeg/fixes_2_2
=======
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const LongOpts : Array of string) : String;
>>>>>>> origin/fixes_2_2

Var
  L : TStringList;
  I : Integer;

begin
  L:=TStringList.Create;
  Try
    For I:=0 to High(LongOpts) do
      L.Add(LongOpts[i]);
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=CheckOptions(ShortOptions,L,AllErrors);
=======
    Result:=CheckOptions(ShortOptions,L);
>>>>>>> graemeg/fixes_2_2
=======
    Result:=CheckOptions(ShortOptions,L);
>>>>>>> origin/fixes_2_2
  Finally
    L.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.CheckOptions(const ShortOptions: String;
  const LongOpts: String; AllErrors: Boolean): String;
=======
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const LongOpts : String) : String;
>>>>>>> graemeg/fixes_2_2
=======
Function TCustomApplication.CheckOptions(Const ShortOptions : String; Const LongOpts : String) : String;
>>>>>>> origin/fixes_2_2

Const
  SepChars = ' '#10#13#9;

Var
  L : TStringList;
  Len,I,J : Integer;

begin
  L:=TStringList.Create;
  Try
    I:=1;
    Len:=Length(LongOpts);
    While I<=Len do
      begin
      While Isdelimiter(SepChars,LongOpts,I) do
        Inc(I);
      J:=I;
      While (J<=Len) and Not IsDelimiter(SepChars,LongOpts,J) do
        Inc(J);
      If (I<=J) then
        L.Add(Copy(LongOpts,I,(J-I)));
      I:=J+1;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=CheckOptions(Shortoptions,L,AllErrors);
=======
    Result:=CheckOptions(Shortoptions,L);
>>>>>>> graemeg/fixes_2_2
=======
    Result:=CheckOptions(Shortoptions,L);
>>>>>>> origin/fixes_2_2
  Finally
    L.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TCustomApplication.GetNonOptions(const ShortOptions: String;
  const Longopts: array of string): TStringArray;

Var
  NO : TStrings;
  I : Integer;

begin
  No:=TStringList.Create;
  try
    GetNonOptions(ShortOptions,LongOpts,No);
    SetLength(Result,NO.Count);
    For I:=0 to NO.Count-1 do
      Result[I]:=NO[i];
  finally
    NO.Free;
  end;
end;

procedure TCustomApplication.GetNonOptions(const ShortOptions: String;
  const Longopts: array of string; NonOptions: TStrings);

Var
  S : String;

begin
  S:=CheckOptions(ShortOptions,LongOpts,Nil,NonOptions,true);
  if (S<>'') then
    Raise EListError.Create(S);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
