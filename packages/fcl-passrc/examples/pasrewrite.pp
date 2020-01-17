program pasrewrite;

{$mode objfpc}
{$H+}

uses SysUtils, inifiles, strutils, Classes, Pscanner,PParser, PasTree, paswrite, custapp, iostream;

//# types the parser needs

type
  { We have to override abstract TPasTreeContainer methods.
    See utils/fpdoc/dglobals.pp for an implementation of TFPDocEngine,
    a "real" engine. }
  TSimpleEngine = class(TPasTreeContainer)
  public
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
      override;
    function FindElement(const AName: String): TPasElement; override;
  end;

  { TPasRewriteApplication }

  TPasRewriteApplication  = Class(TCustomApplication)
  Private
    FHeaderFile : String;
    FDefines : TStrings;
    FLineNumberWidth,
    FIndentSize : Integer;
    FOptions : TPasWriterOptions;
    FForwardClasses,
    FExtraUnits,
    cmdl,
    ConfigFile,
    filename,
    TargetOS,
    TargetCPU : string;
    function GetModule: TPasModule;
    procedure PrintUsage(S: String);
    procedure ReadConfig(const aFileName: String);
    procedure ReadConfig(const aIni: TIniFile);
    procedure WriteModule(M: TPasModule);
  Protected
    function ParseOptions : Boolean;
    Procedure DoRun; override;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
  end;

{ TSimpleEngine }

function TSimpleEngine.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
begin
  Result := AClass.Create(AName, AParent);
  Result.Visibility := AVisibility;
  Result.SourceFilename := ASourceFilename;
  Result.SourceLinenumber := ASourceLinenumber;
end;

function TSimpleEngine.FindElement(const AName: String): TPasElement;
begin
  { dummy implementation, see TFPDocEngine.FindElement for a real example }
  Result := nil;
end;

{ TPasRewriteApplication }

procedure TPasRewriteApplication.PrintUsage(S : String);

begin
  if S<>'' then
    Writeln('Error : ',S);
  writeln('usage: pasrewrite options');
  writeln;
  writeln('Where options is one or more of');
  writeln('-s --os=OS            Set OS, one of WINDOWS, LINUX (default), FREEBSD, NETBSD,');
  writeln('                           SUNOS, BEOS, QNX, GO32V2');
  writeln('-u --cpu=CPU          Set CPU = i386 (default), x86_64');
  writeln('-x --extra=units      Comma-separated list of extra units to be added to uses list.');
  writeln('-i --input=cmd        Is the commandline for the parser');
  writeln('-o --output=file      Output file name. If not specified, standard output is assumed ');
  Writeln('-t --indent=N         Number of characters for indent (default 2)');
  Writeln('-c --config=filename  Read ini file with configuration');
  Writeln('-H --header=filename  Add file header using contents of file "filename"');
  Writeln('--no-implementation   Skip generation of executeable code');
  Writeln('--no-externalclass    Skip generation of external classes (write as regular class)');
  Writeln('--no-externalvar      Skip generation of external variables (write as regular variables)');
  Writeln('--no-externalfunction Skip generation of external functions (write as regular functions)');
  Writeln('-f --forwardclasses[=list]');
  Writeln('                      Generate forward definitions for list of classes. If empty, for all classes.');
  Writeln('-n --add-linenumber   Add linenumber to comment in front of each line');
  Writeln('-N --add-sourcelinenumber   Add source linenumber to comment in front of each line');
  Writeln('-w --linenumberwidth  Number of digits to pad line numbers (default 4)');
  ExitCode:=Ord(S<>'');
end;

function TPasRewriteApplication.ParseOptions : Boolean;

Var
  S : String;

begin
  TargetOS:='linux';
  TargetCPU:='i386';
  FIndentSize:=-1;
  FOptions:=[];
  Result:=False;
  S:=CheckOptions('d:w:fhs:u:i:o:nNt:c:x:',['help','os:','cpu:','input:','output:','indent:','define',
                                    'no-implementation','no-externalclass',
                                    'no-externalvar','add-linenumber','add-sourcelinenumber',
                                    'no-externalfunction','extra:','forwardclasses::',
                                    'config:','linenumberwidth']);
  if (S<>'') or HasOption('h','help') then
     begin
     PrintUsage(S);
     Exit;
     end;
  // Standard options
  cmdl:=GetOptionValue('i','input');
  FileName:=GetOptionValue('o','output');
  FHeaderFile:=GetOptionValue('H','header');;
  if HasOption('s','os') then
    TargetOS:=GetOPtionValue('s','os');
  if HasOption('u','cpu') then
    TargetCPU:=GetOptionValue('u','cpu');
  ConfigFile:=GetOptionValue('c','config');
  FExtraUnits:=GetOptionValue('x','extra');
  // Options
  if Hasoption('w','linenumberwidth') then
    FLineNumberWidth:=StrToIntDef(GetOptionValue('w','linenumberwidth'),-1);
  if Hasoption('n','add-linenumber') then
    Include(Foptions,woAddLineNumber);
  if Hasoption('N','add-sourcelinenumber') then
    Include(Foptions,woAddSourceLineNumber);
  if Hasoption('no-implementation') then
    Include(Foptions,woNoImplementation);
  if Hasoption('no-externalclass') then
    Include(Foptions,woNoExternalClass);
  if Hasoption('no-externalvar') then
    Include(Foptions,woNoExternalVar);
  if Hasoption('no-externalfunction') then
    Include(Foptions,woNoExternalFunc);
  If HasOption('d','define') then
    for S in GetOptionValues('d','define') do
      FDefines.Add(S);
  if Hasoption('f','forwardclasses') then
    begin
    Include(Foptions,woForwardClasses);
    FForwardClasses:=GetOptionValue('f','forwardclasses');
    end;
  // Indent
  if HasOption('t','indent') then
    FIndentSize:=StrToIntDef(GetOptionValue('t','indent'),-1);
  if (FHeaderFile<>'') and Not FileExists(FheaderFile) then
    begin
    PrintUsage(Format('Header file "%s"does not exist',[FHeaderFile]));
    Exit;
    end;
  //  Check options
  Result:=(Cmdl<>'') ;
  If Not Result then
    PrintUsage('Need input');
end;

{ TPasRewriteApplication }



Function TPasRewriteApplication.GetModule : TPasModule;

Var
  SE : TSimpleEngine;
  FileResolver: TFileResolver;
  InputFileName : string;
  Parser: TPasParser;
  Start, CurPos: PChar;
  Scanner: TPascalScanner;

  procedure ProcessCmdLinePart;

  var
    l: Integer;
    s: String;
  begin
    l := CurPos - Start;
    SetLength(s, l);
    if l > 0 then
      Move(Start^, s[1], l)
    else
      exit;
    if (s[1] = '-') and (length(s)>1) then
    begin
      case s[2] of
        'd': // -d define
          Scanner.AddDefine(UpperCase(Copy(s, 3, Length(s))));
        'u': // -u undefine
          Scanner.RemoveDefine(UpperCase(Copy(s, 3, Length(s))));
        'F': // -F
          if (length(s)>2) and (s[3] = 'i') then // -Fi include path
            FileResolver.AddIncludePath(Copy(s, 4, Length(s)));
        'I': // -I include path
          FileResolver.AddIncludePath(Copy(s, 3, Length(s)));
        'S': // -S mode
          if  (length(s)>2) then
            begin
            l:=3;
            While L<=Length(S) do
              begin
              case S[l] of
                'c' : Scanner.Options:=Scanner.Options+[po_cassignments];
                'd' : Scanner.SetCompilerMode('DELPHI');
                '2' : Scanner.SetCompilerMode('OBJFPC');
                'h' : ; // do nothing
              end;
              inc(l);
              end;
            end;
        'M' :
           begin
           delete(S,1,2);
           Scanner.SetCompilerMode(S);
           end;
      end;
    end else
      if InputFilename <> '' then
        raise Exception.Create(SErrMultipleSourceFiles)
      else
        InputFilename := s;
  end;


var
  s: String;

begin
  try
    Result := nil;
    FileResolver := nil;
    Scanner := nil;
    Parser := nil;
    SE:=TSimpleEngine.Create;
    try
      FileResolver := TFileResolver.Create;
      FileResolver.UseStreams:=True;
      Scanner := TPascalScanner.Create(FileResolver);
      Scanner.Options:=[po_keepclassforward,po_AsmWhole];
      SCanner.LogEvents:=SE.ScannerLogEvents;
      SCanner.OnLog:=SE.Onlog;
      Scanner.AddDefine('FPK');
      Scanner.AddDefine('FPC');
      For S in FDefines do
        Scanner.AddDefine(S);
      // TargetOS
      s := UpperCase(TargetOS);
      Scanner.AddDefine(s);
      if s = 'LINUX' then
        Scanner.AddDefine('UNIX')
      else if s = 'FREEBSD' then
      begin
        Scanner.AddDefine('BSD');
        Scanner.AddDefine('UNIX');
      end else if s = 'NETBSD' then
      begin
        Scanner.AddDefine('BSD');
        Scanner.AddDefine('UNIX');
      end else if s = 'SUNOS' then
      begin
        Scanner.AddDefine('SOLARIS');
        Scanner.AddDefine('UNIX');
      end else if s = 'GO32V2' then
        Scanner.AddDefine('DPMI')
      else if s = 'BEOS' then
        Scanner.AddDefine('UNIX')
      else if s = 'QNX' then
        Scanner.AddDefine('UNIX')
      else if s = 'AROS' then
        Scanner.AddDefine('HASAMIGA')
      else if s = 'MORPHOS' then
        Scanner.AddDefine('HASAMIGA')
      else if s = 'AMIGA' then
        Scanner.AddDefine('HASAMIGA');

      // TargetCPU
      s := UpperCase(TargetCPU);
      Scanner.AddDefine('CPU'+s);
      if (s='X86_64') then
        Scanner.AddDefine('CPU64')
      else
        Scanner.AddDefine('CPU32');
      Parser := TPasParser.Create(Scanner, FileResolver, SE);
      InputFilename := '';
      Parser.LogEvents:=SE.ParserLogEvents;
      Parser.OnLog:=SE.Onlog;

      if cmdl<>'' then
        begin
          Start := @cmdl[1];
          CurPos := Start;
          while CurPos[0] <> #0 do
          begin
            if CurPos[0] = ' ' then
            begin
              ProcessCmdLinePart;
              Start := CurPos + 1;
            end;
            Inc(CurPos);
          end;
          ProcessCmdLinePart;
        end;

      if InputFilename = '' then
        raise Exception.Create(SErrNoSourceGiven);
      FileResolver.AddIncludePath(ExtractFilePath(InputFileName));
      Scanner.OpenFile(InputFilename);
      Parser.Options:=Parser.Options+[po_AsmWhole,po_KeepClassForward];
      Parser.ParseMain(Result);
    finally
      Parser.Free;
      Scanner.Free;
      FileResolver.Free;
      SE.Free;
    end;
  except
    on E : EParserError do
      begin
      writeln(E.message,' line:',E .row,' column:', E .column,' file:',E.filename);
      end;
    on Ex : Exception do
      begin
      Writeln(Ex.Message);
      end;
  end;
end;

procedure TPasRewriteApplication.ReadConfig(const aFileName: String);

Var
  ini : TMemIniFile;

begin
  ini:=TMemIniFile.Create(AFileName);
  try
    ReadConfig(Ini);
  finally
    Ini.Free;
  end;
end;

procedure TPasRewriteApplication.ReadConfig(const aIni: TIniFile);

Const
  DelChars = [',',' '];

Var
  O : TPaswriterOptions;
  W,S : String;
  I : Integer;


begin
  O:=[];
  With aIni do
    begin
    TargetOS:=ReadString('config','targetos',TargetOS);
    TargetCPU:=ReadString('config','targetcpu',TargetCPU);
    S:=ReadString('config','options','');
    if (S<>'') then
      For I:=1 to WordCount(S,DelChars) do
        begin
        W:=LowerCase(ExtractWord(I,S,DelChars));
        Case w of
         'noimplementation': Include(O,woNoImplementation);
         'noexternalclass' : Include(O,woNoExternalClass);
         'noexternalvar' : Include(O,woNoExternalVar);
         'noexternalfunction' : Include(O,woNoExternalFunc);
         'forwardclasses' : Include(O,woForwardClasses);
         'addlinenumber': Include(O,woAddLineNumber);
         'addsourcelinenumber': Include(O,woAddSourceLineNumber);
        end;
        end;
    FOptions:=O;
    cmdl:=ReadString('config','input',cmdl);
    Self.filename:=ReadString('config','output',Self.filename);
    FIndentSize:=ReadInteger('config','indentsize',FIndentSize);
    FLineNumberWidth:=ReadInteger('config','linenumberwidth',FLineNumberWidth);
    FExtraUnits:=ReadString('config','extra',FExtraUnits);
    FForwardClasses:=ReadString('config','forwardclasses',FForwardClasses);
    S:=ReadString('config','defines','');
    if (S<>'') then
      For I:=1 to WordCount(S,DelChars) do
        FDefines.Add(UpperCase(ExtractWord(I,S,DelChars)));
    if (FForwardClasses<>'') then
      Include(O,woForwardClasses);
    end;
end;

procedure TPasRewriteApplication.WriteModule(M : TPAsModule);

Var
  F,H : TStream;
  W : TPasWriter;

begin
  W:=Nil;
  if FileName='' then
    F:=TIOStream.Create(iosOutPut)
  else
    F:=TFileStream.Create(FileName,fmCreate);
  try
     if (FHeaderFile<>'') then
       begin
       H:=TFileStream.Create(FHeaderFile,fmOpenRead or fmShareDenyWrite);
       try
         F.CopyFrom(H,H.Size);
       finally
         H.Free;
       end;
       end;
     W:=TPasWriter.Create(F);
     W.Options:=FOptions;
     W.ExtraUnits:=FExtraUnits;
     if FIndentSize<>-1 then
       W.IndentSize:=FIndentSize;
     if FLineNumberWidth>0 then
       W.LineNumberWidth:=FLineNumberWidth;
     W.ForwardClasses.CommaText:=FForwardClasses;
     W.WriteModule(M);
  finally
    W.Free;
    F.Free;
  end;
end;

procedure TPasRewriteApplication.DoRun;

Var
  M: TPasModule;

begin
  Terminate;
  TargetOS:='linux';
  TargetCPU:='i386';
  If not ParseOptions then
    exit;
  If (ConfigFile<>'') then
    ReadConfig(ConfigFile);
  M:=GetModule;
  if M=Nil then
    exit;
  try
    WriteModule(M);
  finally
    M.Free;
  end;
end;

constructor TPasRewriteApplication.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDefines:=TStringList.Create;

end;

destructor TPasRewriteApplication.Destroy;
begin
  FreeAndNil(FDefines);
  inherited Destroy;
end;

Var
  Application : TPasRewriteApplication;

begin
  Application:=TPasRewriteApplication.Create(Nil);
  Application.Initialize;
  Application.Run;
  Application.Free;
end.
