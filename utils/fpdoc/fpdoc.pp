{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org
    2005-2012 by
      various FPC contributors

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}


program FPDoc;

uses
 {$ifdef Unix}
  CThreads,
  cwstring,
{$endif}
  SysUtils, Classes, Gettext, custapp,
  dGlobals,  // Global definitions, constants.
  fpdocclasstree, // Class tree builder
  dwriter,   // TFPDocWriter definition.
  dwlinear,  // Linear (abstract) writer
  dw_LaTeX,  // TLaTex writer
  dw_XML,    // XML writer
  dw_dxml,   // Delphi XML doc.
  dw_HTML,   // HTML writer
  dw_ipflin, // IPF writer (new linear output)
  dw_man,    // Man page writer
  dw_linrtf, // linear RTF writer
  dw_txt,    // TXT writer
  fpdocproj, mkfpdoc;


Type

  { TFPDocApplication }

  TFPDocApplication = Class(TCustomApplication)
  private
    FCreator : TFPDocCreator;
    FPackage : TFPDocPackage;
    FDryRun,
    FProjectFile : Boolean;
    FWriteProjectFile : String;
  Protected
    procedure OutputLog(Sender: TObject; const Msg: String);
    procedure ParseCommandLine;
    procedure ParseOption(const S: String);
    Procedure Usage(AnExitCode : Byte);
    Procedure DoRun; override;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Function SelectedPackage : TFPDocPackage;
  end;


Procedure TFPDocApplication.Usage(AnExitCode : Byte);

Var
  I,P : Integer;
  S : String;
  L : TStringList;
  C : TFPDocWriterClass;
  Backend : String;

begin
  Writeln(Format(SCmdLineHelp,[ExtractFileName(Paramstr(0))]));
  Writeln(SUsageOption008);
  Writeln(SUsageOption009);
  Writeln(SUsageOption010);
  Writeln(SUsageOption020);
  Writeln(SUsageOption030);
  Writeln(SUsageOption035);
  Writeln(SUsageOption040);
  Writeln(SUsageOption050);
  Writeln(SUsageOption060);
  Writeln(SUsageOption070);
  Writeln(SUsageOption080);
  Writeln(SUsageOption090);
  Writeln(SUsageOption100);
  Writeln(SUsageOption110);
  Writeln(SUsageOption120);
  Writeln(SUsageOption130);
  Writeln(SUsageOption140);
  Writeln(SUsageOption145);
  Writeln(SUsageOption150);
  Writeln(SUsageOption160);
  Writeln(SUsageOption170);
  Writeln(SUsageOption180);
  Writeln(SUsageOption190);
  Writeln(SUsageOption200);
  Writeln(SUsageOption210);
  Writeln(SUsageOption215);
  Writeln(SUsageOption215A);
  Writeln(SUsageOption220);
  Writeln(SUsageOption230);
  Writeln(SUsageOption240);
  Writeln(SUsageOption250);
  Writeln(SUsageOption260);
  Writeln(SUsageOption270);
  Writeln(SUsageOption280);
  Writeln(SUsageOption290);
  Writeln(SUsageOption300);
  Writeln(SUsageOption310);
  Writeln(SUsageOption320);
  L:=TStringList.Create;
  Try
    Backend:=FCreator.OPtions.Backend;
    If (Backend='') then
      begin
      Writeln;
      Writeln(SUsageFormats);
      EnumWriters(L);
      For I:=0 to L.Count-1 do
        begin
        S:=L[i];
        P:=Pos('=',S);
        Writeln(Format(' %s - %s',[Copy(S,1,P-1)+Space(10-p),Copy(S,P+1,Length(S))]));
        end;
      Writeln(SUsageBackendHelp);
      end
    else
      begin
      Writeln;
      Writeln(Format(SUsageFormatSpecific,[Lowercase(backend)]));
      C:=GetWriterClass(Backend);
      C.Usage(L);
      If L.Count>0 then
        For I:=0 to (L.Count-1) div 2 do
          begin
          S:=L[i*2];
          Writeln(Format('%s %s',[S+Space(30-Length(S)),L[(i*2)+1]]));
          end;
      end;
  Finally
    L.Free;
  end;
  Halt(AnExitCode);
end;

destructor TFPDocApplication.Destroy;

begin
  FreeAndNil(FCreator);
  Inherited;
end;

function TFPDocApplication.SelectedPackage: TFPDocPackage;
var
  i:integer;
begin
  Result:=FPackage;
  if (FPackage=Nil) or (FPackage.Name='') then
    begin
    Writeln(SNeedPackageName);
    if FCreator.Packages.Count>0 then
      begin
      if (FCreator.Packages[0].Name<>'') then
        Writeln(SAvailablePackages);
      for i:=0 to FCreator.Packages.Count-1 do
        begin
        Writeln(FCreator.Packages[i].Name);
        end;
      end;
    Usage(1);
    end;
end;

procedure TFPDocApplication.OutputLog(Sender: TObject; const Msg: String);
begin
  Writeln(StdErr,Msg);
end;

procedure TFPDocApplication.ParseCommandLine;

Const
  SOptProject = '--project=';
  SOptPackage = '--package=';
  SOptMacro = '--macro=';

  Function ProjectOpt(Const s : string) : boolean;

  begin
    Result:=(Copy(s,1,3)='-p=') or (Copy(s,1,Length(SOptProject))=SOptProject) or (Copy(s,1,Length(SOptMacro))=SOptMacro);
  end;

  Function PackageOpt(Const s : string) : boolean;

  begin
    Result:=((Copy(s,1,3)='-a=') or (Copy(s,1,Length(SOptPackage))=SOptPackage));
  end;

var
  i : Integer;
  s : string;

begin

  // Check project
  for i := 1 to ParamCount do
    begin
    s:=ParamStr(I);
    If ProjectOpt(S) then
      ParseOption(s);
    end;
  If (FCreator.Packages.Count=1) then
    FPackage:=FCreator.Packages[0]
  else if (FCreator.Options.DefaultPackageName<>'') then
    Fpackage:=FCreator.Packages.FindPackage(FCreator.Options.DefaultPackageName);
  If FCreator.Project.Packages.Count=0 then
    begin // Add default package if none defined
    FPackage:=FCreator.Packages.Add as TFPDocPackage;
    end;
  // Check package
  for i := 1 to ParamCount do
    begin
    s:=ParamStr(I);
    If PackageOpt(S) then
      ParseOption(s);
    end;
  for i := 1 to ParamCount do
    begin
    s:=ParamStr(I);
    If Not (ProjectOpt(s) or PackageOpt(S)) then
      ParseOption(s);
    end;
  SelectedPackage; // Will print error if none available.
  // Set defaults
  if FCreator.Options.BackEnd='' then
    FCreator.Options.BackEnd:='html';
  if SelectedPackage.Output='' then
    SelectedPackage.Output:=SelectedPackage.Name;
end;

procedure TFPDocApplication.ParseOption(Const S : String);

  procedure AddDirToFileList(List: TStrings; const ADirName, AMask: String);

  Var
    Info : TSearchRec;
    D : String;

  begin
    if (ADirName<>'') and not DirectoryExists(ADirName) then
       OutputLog(Self,'Directory '+ADirName+' does not exist')
    else
      begin
      if (ADirName='.') or (ADirName='') then
        D:=''
      else
        D:=IncludeTrailingPathDelimiter(ADirName);
      If (FindFirst(D+AMask,0,Info)=0) then
        try
          Repeat
            If (Info.Attr and faDirectory)=0 then
              List.Add(D+Info.name);
          Until FindNext(Info)<>0;
        finally
          FindClose(Info);
        end;
      end;
  end;

  procedure AddToFileList(List: TStrings; const FileName: String);
  var
    f: Text;
    s: String;
  begin
    if Copy(FileName, 1, 1) = '@' then
    begin
      AssignFile(f, Copy(FileName, 2, Length(FileName)));
      Reset(f);
      while not EOF(f) do
      begin
        ReadLn(f, s);
        List.Add(s);
      end;
      Close(f);
    end else
      List.Add(FileName);
  end;

var
  i: Integer;
  Cmd, Arg: String;

begin
  if (s = '-h') or (s = '--help') then
    Usage(0)
  else if s = '--hide-protected' then
    FCreator.Options.HideProtected := True
  else if s = '--warn-no-node' then
    FCreator.Options.WarnNoNode := True
  else if s = '--show-private' then
    FCreator.Options.ShowPrivate := True
  else if s = '--stop-on-parser-error' then
    FCreator.Options.StopOnParseError := True
  else if s = '--dont-trim' then
    FCreator.Options.DontTrim := True
  else
    begin
    i := Pos('=', s);
    if i > 0 then
      begin
      Cmd := Copy(s, 1, i - 1);
      Arg := Copy(s, i + 1, Length(s));
      end
    else
      begin
      Cmd := s;
      SetLength(Arg, 0);
      end;
    if (Cmd = '--project') or (Cmd='-p') then
      begin
      FProjectFile:=True;
      FCreator.LoadProjectFile(Arg);
      end
    else if (Cmd = '--descr') then
      AddToFileList(SelectedPackage.Descriptions, Arg)
    else if (Cmd = '--descr-dir') then
      AddDirToFileList(SelectedPackage.Descriptions, Arg, '*.xml')
    else if (Cmd = '--base-descr-dir') then
      FCreator.BaseDescrDir:=Arg
    else if (Cmd = '--macro') then
      begin
      If Pos('=',Arg)=0 then
        WriteLn(StdErr, Format(SCmdLineErrInvalidMacro, [Arg]));
      FCreator.ProjectMacros.Add(Arg);
      end
    else if (Cmd = '-f') or (Cmd = '--format') then
      begin
      Arg:=UpperCase(Arg);
      If FindWriterClass(Arg)=-1 then
        WriteLn(StdErr, Format(SCmdLineInvalidFormat, [Arg]))
      else
        FCreator.Options.BackEnd:=Arg;
      end
    else if (Cmd = '-l') or (Cmd = '--lang') then
      FCreator.Options.Language := Arg
    else if (Cmd = '-i') or (Cmd = '--input') then
      AddToFileList(SelectedPackage.Inputs, Arg)
    else if (Cmd = '--base-input-dir') then
      FCreator.BaseInputDir:=Arg
    else if (Cmd = '--input-dir') then
      begin
      AddDirToFileList(SelectedPackage.Inputs, Arg,'*.pp');
      AddDirToFileList(SelectedPackage.Inputs, Arg,'*.pas');
      end
    else if (Cmd = '-o') or (Cmd = '--output') then
      SelectedPackage.Output := Arg
    else if (Cmd = '-v') or (Cmd = '--verbose') then
      FCreator.Verbose:=true
    else if (Cmd = '-n') or (Cmd = '--dry-run') then
      FDryRun:=True
    else if (Cmd = '-t') or (Cmd = '--emit-notes') then
      FCreator.Options.EmitNotes := True
    else if Cmd = '--content' then
      SelectedPackage.ContentFile := Arg
    else if Cmd = '--import' then
      SelectedPackage.Imports.Add(Arg)
    else if Cmd = '--package' then
      begin
      If FProjectFile then
        FPackage:=FCreator.Packages.FindPackage(Arg)
      else
        FPackage.Name:=Arg;
      end
    else if Cmd = '--ostarget' then
      FCreator.Options.OSTarget := Arg
    else if Cmd = '--cputarget' then
      FCreator.Options.CPUTarget := Arg
    else if Cmd = '--mo-dir' then
      FCreator.Options.modir := Arg
    else if Cmd = '--parse-impl' then
      FCreator.Options.InterfaceOnly:=false
    else if Cmd = '--write-project' then
      FWriteProjectFile:=Arg
    else
      begin
      FCreator.Options.BackendOptions.Add(Cmd);
      FCreator.Options.BackendOptions.Add(Arg);
      end;
    end;
end;

Procedure TFPDocApplication.DoRun;

begin
   ExceptionExitCode:=1;
  try
  {$IFDEF Unix}
    gettext.TranslateResourceStrings('/usr/local/share/locale/%s/LC_MESSAGES/fpdoc.mo');
  {$ELSE}
    gettext.TranslateResourceStrings('intl/fpdoc.%s.mo');
  {$ENDIF}
    WriteLn(STitle);
    WriteLn(Format(SVersion, [DefFPCVersion, DefFPCDate]));
    WriteLn(SCopyright1);
    WriteLn(SCopyright2);
    WriteLn;
    ParseCommandLine;
    if (FWriteProjectFile<>'') then
      FCreator.CreateProjectFile(FWriteProjectFile)
    else
      FCreator.CreateDocumentation(FPackage,FDryRun);
    WriteLn(SDone);
    Terminate;
  except
    ExitCode:=1;
    Raise;
  end;
end;

constructor TFPDocApplication.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  StopOnException:=true;
  FCreator:=TFPDocCreator.Create(Self);
  FCreator.OnLog:=@OutputLog;
end;

begin
  With TFPDocApplication.Create(Nil) do
    try
      Run;
    finally
      Free;
    end;
end.
