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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 {$ifdef Unix}
  CThreads,
  cwstring,
{$endif}
  SysUtils, Classes, Gettext, custapp,
  dGlobals,  // Global definitions, constants.
  fpdocclasstree, // Class tree builder
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SysUtils, Classes, Gettext, DOM, XMLWrite, PasTree, PParser, custapp,
  dGlobals,  // GLobal definitions, constants.
>>>>>>> graemeg/cpstrnew
=======
  SysUtils, Classes, Gettext, DOM, XMLWrite, PasTree, PParser,
  dGlobals,  // GLobal definitions, constants.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  dwriter,   // TFPDocWriter definition.
  dwlinear,  // Linear (abstract) writer
  dw_LaTeX,  // TLaTex writer
  dw_XML,    // XML writer
<<<<<<< HEAD
<<<<<<< HEAD
  dw_dxml,   // Delphi XML doc.
  dw_HTML,   // HTML writer
  dw_ipflin, // IPF writer (new linear output)
  dw_man,    // Man page writer
  dw_linrtf, // linear RTF writer
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  dw_txt, fpdocproj, fpdocxmlopts;    // TXT writer
=======
=======
>>>>>>> origin/fixes_2_2
  dw_HTML,   // HTML writer
  dw_ipf,    // IPF writer
  dw_man,    // Man page writer
  dw_linrtf, // lineair RTF writer
  dw_txt;    // TXT writer
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

const
  DefOSTarget    = {$I %FPCTARGETOS%};
  DefCPUTarget   = {$I %FPCTARGETCPU%};
  DefFPCVersion  = {$I %FPCVERSION%};
  DefFPCDate     = {$I %FPCDATE%};
<<<<<<< HEAD

<<<<<<< HEAD
Type

  { TFPDocAplication }

  TFPDocAplication = Class(TCustomApplication)
  private
    FProject : TFPDocProject;
    FProjectFile : Boolean;
    FPackage : TFPDocPackage;
  Protected
    procedure ParseCommandLine;
    procedure Parseoption(const S: String);
    Procedure Usage(AnExitCode : Byte);
    procedure CreateDocumentation(APackage : TFPDocPackage; Options : TEngineOptions);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    Procedure DoRun; override;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Function SelectedPackage : TFPDocPackage;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


Procedure TFPDocApplication.Usage(AnExitCode : Byte);
=======


Procedure TFPDocAplication.Usage(AnExitCode : Byte);
>>>>>>> graemeg/cpstrnew
=======
=======
var
  Backend : String;
  BackendOptions : TStrings;
  InputFiles, DescrFiles: TStringList;
  PackageName, DocLang, ContentFile : String;
  Engine: TFPDocEngine;
  StopOnParserError : Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

=======
>>>>>>> origin/fixes_2_2

Procedure TFPDocAplication.Usage(AnExitCode : Byte);
>>>>>>> graemeg/cpstrnew
=======


Procedure TFPDocAplication.Usage(AnExitCode : Byte);
>>>>>>> graemeg/cpstrnew
=======

Type

  { TFPDocAplication }

  TFPDocAplication = Class(TCustomApplication)
  private
    FProject : TFPDocProject;
    FProjectFile : Boolean;
    FPackage : TFPDocPackage;
  Protected
    procedure ParseCommandLine;
    procedure Parseoption(const S: String);
    Procedure Usage(AnExitCode : Byte);
    procedure CreateDocumentation(APackage : TFPDocPackage; Options : TEngineOptions);
    Procedure DoRun; override;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Function SelectedPackage : TFPDocPackage;
  end;


Procedure TFPDocAplication.Usage(AnExitCode : Byte);
>>>>>>> origin/cpstrnew

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
  Writeln(SUsageOption150);
  Writeln(SUsageOption160);
  Writeln(SUsageOption170);
  Writeln(SUsageOption180);
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln(SUsageOption190);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln(SUsageOption200);
  Writeln(SUsageOption210);
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
=======
>>>>>>> origin/fixes_2_2
  L:=TStringList.Create;
  Try
    Backend:=FCreator.OPtions.Backend;
=======
=======
>>>>>>> graemeg/fixes_2_2
  L:=TStringList.Create;
  Try
    Backend:=FProject.OPtions.Backend;
>>>>>>> graemeg/cpstrnew
=======
  L:=TStringList.Create;
  Try
    Backend:=FProject.OPtions.Backend;
>>>>>>> graemeg/cpstrnew
=======
  L:=TStringList.Create;
  Try
    Backend:=FProject.OPtions.Backend;
>>>>>>> graemeg/cpstrnew
=======
  L:=TStringList.Create;
  Try
    Backend:=FProject.OPtions.Backend;
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
destructor TFPDocApplication.Destroy;

begin
  FreeAndNil(FCreator);
  Inherited;
end;

function TFPDocApplication.SelectedPackage: TFPDocPackage;
var
  i:integer;
=======
destructor TFPDocAplication.Destroy;

begin
  FreeAndNil(FProject);
  Inherited;
end;

function TFPDocAplication.SelectedPackage: TFPDocPackage;
>>>>>>> graemeg/cpstrnew
begin
<<<<<<< HEAD
  Result:=FPackage;
  if (FPackage=Nil) or (FPackage.Name='') then
    begin
    Writeln(SNeedPackageName);
<<<<<<< HEAD
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
=======
  InputFiles := TStringList.Create;
  DescrFiles := TStringList.Create;
  BackendOptions := TStringList.Create;
  Engine := TFPDocEngine.Create;
  StopOnParserError:=False;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
end;

procedure TFPDocApplication.OutputLog(Sender: TObject; const Msg: String);
begin
  Writeln(StdErr,Msg);
end;

procedure TFPDocApplication.ParseCommandLine;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
destructor TFPDocAplication.Destroy;

begin
  FreeAndNil(FProject);
  Inherited;
=======
>>>>>>> origin/fixes_2_2
end;

function TFPDocAplication.SelectedPackage: TFPDocPackage;
begin
  Result:=FPackage;
  if (FPackage=Nil) or (FPackage.Name='') then
    begin
    Writeln(SNeedPackageName);
    Usage(1);
    end;
<<<<<<< HEAD
end;


procedure TFPDocAplication.ParseCommandLine;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    Usage(1);
    end;
=======
>>>>>>> origin/cpstrnew
end;


procedure TFPDocAplication.ParseCommandLine;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  Function ProjectOpt(Const s : string) : boolean;

  begin
    Result:=(Copy(s,1,3)='-p=') or (Copy(s,1,10)='--project=');
  end;

  Function PackageOpt(Const s : string) : boolean;

  begin
    Result:=((Copy(s,1,3)='-a=') or (Copy(s,1,10)='--package='));
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    If (FCreator.Packages.Count=1) then
      FPackage:=FCreator.Packages[0]
    else if (FCreator.Options.DefaultPackageName<>'') then
      Fpackage:=FCreator.Packages.FindPackage(FCreator.Options.DefaultPackageName);
    end;
  If FCreator.Project.Packages.Count=0 then
    begin // Add default package if none defined
    FPackage:=FCreator.Packages.Add as TFPDocPackage;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    If (FProject.Packages.Count=1) then
      FPackage:=FProject.Packages[0]
    else if (FProject.Options.DefaultPackageName<>'') then
      Fpackage:=FProject.Packages.FindPackage(FProject.Options.DefaultPackageName);
    end;
  If FProject.Packages.Count=0 then
    begin
    FPackage:=FProject.Packages.Add as  TFPDocPackage;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SelectedPackage; // Will print error if none available.
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

=======
  if (FPackage=Nil) or (FPackage.Name='') then
    begin
    Writeln(SNeedPackageName);
    Usage(1);
    end;
end;

procedure TFPDocAplication.Parseoption(Const S : String);

>>>>>>> graemeg/cpstrnew
=======
  if (FPackage=Nil) or (FPackage.Name='') then
    begin
    Writeln(SNeedPackageName);
    Usage(1);
    end;
end;

procedure TFPDocAplication.Parseoption(Const S : String);

>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
  if (FPackage=Nil) or (FPackage.Name='') then
    begin
    Writeln(SNeedPackageName);
    Usage(1);
    end;
end;

procedure TFPDocAplication.Parseoption(Const S : String);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FCreator.Options.HideProtected := True
  else if s = '--warn-no-node' then
    FCreator.Options.WarnNoNode := True
  else if s = '--show-private' then
<<<<<<< HEAD
    FCreator.Options.ShowPrivate := True
  else if s = '--stop-on-parser-error' then
    FCreator.Options.StopOnParseError := True
  else if s = '--dont-trim' then
    FCreator.Options.DontTrim := True
=======
    FProject.Options.HideProtected := True
  else if s = '--warn-no-node' then
    FProject.Options.WarnNoNode := True
  else if s = '--show-private' then
    FProject.Options.ShowPrivate := False
  else if s = '--stop-on-parser-error' then
    FProject.Options.StopOnParseError := True
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
    FProject.Options.HideProtected := True
  else if s = '--warn-no-node' then
    FProject.Options.WarnNoNode := True
  else if s = '--show-private' then
    FProject.Options.ShowPrivate := False
  else if s = '--stop-on-parser-error' then
    FProject.Options.StopOnParseError := True
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    FProject.Options.HideProtected := True
  else if s = '--warn-no-node' then
    FProject.Options.WarnNoNode := True
  else if s = '--show-private' then
    FProject.Options.ShowPrivate := False
  else if s = '--stop-on-parser-error' then
    FProject.Options.StopOnParseError := True
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
    Engine.HidePrivate := False
  else if s = '--stop-on-parser-error' then
    StopOnParserError := True
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FCreator.LoadProjectFile(Arg);
      end
    else if (Cmd = '--descr') then
      AddToFileList(SelectedPackage.Descriptions, Arg)
    else if (Cmd = '--descr-dir') then
      AddDirToFileList(SelectedPackage.Descriptions, Arg, '*.xml')
    else if (Cmd = '--base-descr-dir') then
      FCreator.BaseDescrDir:=Arg
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      With TXMLFPDocOptions.Create(self) do
        try
          LoadOptionsFromFile(FProject,Arg);
        finally
          Free;
        end;
      end
    else if (Cmd = '--descr') then
      AddToFileList(SelectedPackage.Descriptions, Arg)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    else if (Cmd = '-f') or (Cmd = '--format') then
      begin
      Arg:=UpperCase(Arg);
      If FindWriterClass(Arg)=-1 then
        WriteLn(StdErr, Format(SCmdLineInvalidFormat, [Arg]))
      else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/cpstrnew
        FProject.Options.BackEnd:=Arg;
      end
    else if (Cmd = '-l') or (Cmd = '--lang') then
      FProject.Options.Language := Arg
    else if (Cmd = '-i') or (Cmd = '--input') then
      AddToFileList(SelectedPackage.Inputs, Arg)
    else if (Cmd = '-o') or (Cmd = '--output') then
      SelectedPackage.Output := Arg
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
        FProject.Options.BackEnd:=Arg;
      end
    else if (Cmd = '-l') or (Cmd = '--lang') then
      FProject.Options.Language := Arg
    else if (Cmd = '-i') or (Cmd = '--input') then
      AddToFileList(SelectedPackage.Inputs, Arg)
    else if (Cmd = '-o') or (Cmd = '--output') then
      SelectedPackage.Output := Arg
>>>>>>> graemeg/cpstrnew
=======
        FProject.Options.BackEnd:=Arg;
      end
    else if (Cmd = '-l') or (Cmd = '--lang') then
      FProject.Options.Language := Arg
    else if (Cmd = '-i') or (Cmd = '--input') then
      AddToFileList(SelectedPackage.Inputs, Arg)
    else if (Cmd = '-o') or (Cmd = '--output') then
      SelectedPackage.Output := Arg
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    else if Cmd = '--content' then
      SelectedPackage.ContentFile := Arg
    else if Cmd = '--import' then
      SelectedPackage.Imports.Add(Arg)
    else if Cmd = '--package' then
      begin
      If FProjectFile then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FPackage:=FCreator.Packages.FindPackage(Arg)
=======
        FPackage:=FProject.Packages.FindPackage(Arg)
>>>>>>> graemeg/cpstrnew
=======
        FPackage:=FProject.Packages.FindPackage(Arg)
>>>>>>> graemeg/cpstrnew
=======
        FPackage:=FProject.Packages.FindPackage(Arg)
>>>>>>> graemeg/cpstrnew
=======
        FPackage:=FProject.Packages.FindPackage(Arg)
>>>>>>> origin/cpstrnew
      else
        FPackage.Name:=Arg;
      end
    else if Cmd = '--ostarget' then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FCreator.Options.OSTarget := Arg
    else if Cmd = '--cputarget' then
<<<<<<< HEAD
      FCreator.Options.CPUTarget := Arg
    else if Cmd = '--mo-dir' then
      FCreator.Options.modir := Arg
    else if Cmd = '--parse-impl' then
      FCreator.Options.InterfaceOnly:=false
    else if Cmd = '--write-project' then
      FWriteProjectFile:=Arg
=======
      CPUTarget := Arg
    else if Cmd = '--mo-dir' then
      modir := Arg
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
    else
      begin
      FCreator.Options.BackendOptions.Add(Cmd);
      FCreator.Options.BackendOptions.Add(Arg);
=======
      FProject.Options.OSTarget := Arg
    else if Cmd = '--cputarget' then
      FProject.Options.CPUTarget := Arg
    else if Cmd = '--mo-dir' then
      FProject.Options.modir := Arg
    else if Cmd = '--parse-impl' then
      FProject.Options.InterfaceOnly:=false
    else
      begin
      FProject.Options.BackendOptions.Add(Cmd);
      FProject.Options.BackendOptions.Add(Arg);
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
      FProject.Options.OSTarget := Arg
    else if Cmd = '--cputarget' then
      FProject.Options.CPUTarget := Arg
    else if Cmd = '--mo-dir' then
      FProject.Options.modir := Arg
    else if Cmd = '--parse-impl' then
      FProject.Options.InterfaceOnly:=false
    else
      begin
      FProject.Options.BackendOptions.Add(Cmd);
      FProject.Options.BackendOptions.Add(Arg);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
      FProject.Options.OSTarget := Arg
    else if Cmd = '--cputarget' then
      FProject.Options.CPUTarget := Arg
    else if Cmd = '--mo-dir' then
      FProject.Options.modir := Arg
    else if Cmd = '--parse-impl' then
      FProject.Options.InterfaceOnly:=false
=======
>>>>>>> origin/fixes_2_2
    else
      begin
      FProject.Options.BackendOptions.Add(Cmd);
      FProject.Options.BackendOptions.Add(Arg);
>>>>>>> origin/cpstrnew
      end;
    end;
  // Set defaults
  if FCreator.Options.BackEnd='' then
    FCreator.Options.BackEnd:='html';
  if SelectedPackage.Output='' then
    SelectedPackage.Output:=SelectedPackage.Name;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPDocApplication.DoRun;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure TFPDocAplication.CreateDocumentation(APackage : TFPDocPackage; Options : TEngineOptions);

var
  i,j: Integer;
  WriterClass : TFPDocWriterClass;
  Writer : TFPDocWriter;
  Engine : TFPDocEngine;
  Cmd,Arg : String;

begin
<<<<<<< HEAD
  Engine:=TFPDocEngine.Create;
  try
    For J:=0 to Apackage.Imports.Count-1 do
      begin
      Arg:=Apackage.Imports[j];
      i := Pos(',', Arg);
      Engine.ReadContentFile(Copy(Arg,1,i-1),Copy(Arg,i+1,Length(Arg)));
      end;
    for i := 0 to APackage.Descriptions.Count - 1 do
      Engine.AddDocFile(APackage.Descriptions[i]);
    Engine.SetPackageName(APackage.Name);
    Engine.Output:=APackage.Output;
    Engine.HideProtected:=Options.HideProtected;
    Engine.HidePrivate:=Not Options.ShowPrivate;
    if Length(Options.Language) > 0 then
      TranslateDocStrings(Options.Language);
    for i := 0 to Fpackage.Inputs.Count - 1 do
      try
        ParseSource(Engine, APackage.Inputs[i], Options.OSTarget, Options.CPUTarget);
      except
        on e: EParserError do
          If Options.StopOnParseError then
            Raise
          else
            WriteLn(StdErr, Format('%s(%d,%d): %s',
                    [e.Filename, e.Row, e.Column, e.Message]));
      end;
    WriterClass:=GetWriterClass(Options.Backend);
    Writer:=WriterClass.Create(Engine.Package,Engine);
    With Writer do
      Try
        If Options.BackendOptions.Count>0 then
          for I:=0 to ((Options.BackendOptions.Count-1) div 2) do
            begin
            Cmd:=Options.BackendOptions[I*2];
            Arg:=Options.BackendOptions[I*2+1];
            If not InterPretOption(Cmd,Arg) then
              WriteLn(StdErr, Format(SCmdLineInvalidOption,[Cmd+'='+Arg]));
            end;
        WriteDoc;
      Finally
        Free;
      end;
    if Length(FPackage.ContentFile) > 0 then
      Engine.WriteContentFile(FPackage.ContentFile);
  finally
    FreeAndNil(Engine);
  end;
=======
  for i := 0 to DescrFiles.Count - 1 do
    Engine.AddDocFile(DescrFiles[i]);
  Engine.SetPackageName(PackageName);
  if Length(DocLang) > 0 then
    TranslateDocStrings(DocLang);
  for i := 0 to InputFiles.Count - 1 do
    try
      ParseSource(Engine, InputFiles[i], OSTarget, CPUTarget);
    except
      on e: EParserError do
        If StopOnParserError then
          Raise
        else 
          WriteLn(StdErr, Format('%s(%d,%d): %s',
                  [e.Filename, e.Row, e.Column, e.Message]));
    end;
  WriterClass:=GetWriterClass(Backend);
  Writer:=WriterClass.Create(Engine.Package,Engine);
  With Writer do
    Try
      If BackendOptions.Count>0 then
        for I:=0 to ((BackendOptions.Count-1) div 2) do
          If not InterPretOption(BackendOptions[I*2],BackendOptions[I*2+1]) then
            WriteLn(StdErr, Format(SCmdLineInvalidOption,[BackendOptions[I*2]+' '+BackendOptions[I*2+1]]));
      WriteDoc;
    Finally
      Free;
    end;
  if Length(ContentFile) > 0 then
    Engine.WriteContentFile(ContentFile);
>>>>>>> graemeg/fixes_2_2
end;


Procedure TFPDocAplication.DoRun;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
{$IFDEF Unix}
  gettext.TranslateResourceStrings('/usr/local/share/locale/%s/LC_MESSAGES/fpdoc.mo');
{$ELSE}
  gettext.TranslateResourceStrings('intl/fpdoc.%s.mo');
{$ENDIF}
  WriteLn(STitle);
  WriteLn(Format(SVersion, [DefFPCVersion, DefFPCDate]));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  WriteLn(SCopyright1);
  WriteLn(SCopyright2);
  WriteLn;
  ParseCommandLine;
  if (FWriteProjectFile<>'') then
    FCreator.CreateProjectFile(FWriteProjectFile)
  else
    FCreator.CreateDocumentation(FPackage,FDryRun);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  WriteLn(SCopyright);
  WriteLn;
  ParseCommandLine;
  CreateDocumentation(FPackage,FProject.Options);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  WriteLn(SDone);
  Terminate;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TFPDocApplication.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  StopOnException:=true;
  FCreator:=TFPDocCreator.Create(Self);
  FCreator.OnLog:=@OutputLog;
end;

begin
  With TFPDocApplication.Create(Nil) do
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TFPDocAplication.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  StopOnException:=true;
  FProject:=TFPDOCproject.Create(Nil);
  FProject.Options.StopOnParseError:=False;
  FProject.Options.CPUTarget:=DefCPUTarget;
  FProject.Options.OSTarget:=DefOSTarget;
end;

begin
  With TFPDocAplication.Create(Nil) do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    try
      Run;
    finally
      Free;
    end;
end.
