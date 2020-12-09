unit mkfpdoc;

{$mode objfpc}{$H+}
{$WARN 5024 off : Parameter "$1" not used}
interface

uses
  Classes, SysUtils, dglobals, DOM, fpdocxmlopts, dwriter, pscanner, pparser, fpdocproj;

const
  DefOSTarget    = {$I %FPCTARGETOS%};
  DefCPUTarget   = {$I %FPCTARGETCPU%};
  DefFPCVersion  = {$I %FPCVERSION%};
  DefFPCDate     = {$I %FPCDATE%};
{$IFDEF FPC_BIG_ENDIAN}
  DefEndianNess = 'FPC_BIG_ENDIAN';
{$ELSE}
  DefEndianNess = 'FPC_LITTLE_ENDIAN';
{$ENDIF}

Type

  { TFPDocCreator }

  TFPDocCreator = Class(TComponent)
  Private
    FBaseDescrDir: String;
    FBaseInputDir: String;
    FCurPackage : TFPDocPackage;
    FProcessedUnits : TStrings;
    FOnLog: TPasParserLogHandler;
    FPParserLogEvents: TPParserLogEvents;
    FProject : TFPDocProject;
    FProjectMacros: TStrings;
    FScannerLogEvents: TPScannerLogEvents;
    FVerbose: Boolean;
    function GetOptions: TEngineOptions;
    function GetPackages: TFPDocPackages;
    procedure SetBaseDescrDir(AValue: String);
    procedure SetBaseInputDir(AValue: String);
    procedure SetProjectMacros(AValue: TStrings);
  Protected
    Function FixInputFile(Const AFileName : String) : String;
    Function FixDescrFile(Const AFileName : String) : String;
    Procedure DoBeforeEmitNote(Sender : TObject; Note : TDomElement; Var EmitNote : Boolean); virtual;
    procedure HandleOnParseUnit(Sender: TObject; const AUnitName: String; out AInputFile, OSTarget, CPUTarget: String);
    procedure SetVerbose(AValue: Boolean); virtual;
    Procedure DoLog(Const Msg : String);
    procedure DoLog(Const Fmt : String; Args : Array of Const);
    Procedure DoLogSender(Sender : TObject; Const Msg : String);
    // Create documetation by specified Writer class
    procedure CreateOutput(APackage: TFPDocPackage; Engine: TFPDocEngine); virtual;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure CreateDocumentation(APackage : TFPDocPackage; ParseOnly : Boolean); virtual; //Writes out documentation in selected format
    Procedure CreateProjectFile(Const AFileName : string); //Writes out project file with the chosen options
    Procedure LoadProjectFile(Const AFileName: string);
    Property Project : TFPDocProject Read FProject;
    Property ScannerLogEvents : TPScannerLogEvents Read FScannerLogEvents Write FScannerLogEvents;
    Property ParserLogEvents : TPParserLogEvents Read FPParserLogEvents Write FPParserLogEvents;
    Property Verbose : Boolean Read FVerbose Write SetVerbose;
    Property OnLog : TPasParserLogHandler Read FOnLog Write FOnLog;
    // Easy access
    Property Options : TEngineOptions Read GetOptions;
    Property Packages : TFPDocPackages Read GetPackages;
    // When set, they will be prepended to non-absolute filenames.
    Property BaseInputDir : String Read FBaseInputDir Write SetBaseInputDir;
    Property BaseDescrDir : String Read FBaseDescrDir Write SetBaseDescrDir;
    // Macros used when loading the project file
    Property ProjectMacros : TStrings Read FProjectMacros Write SetProjectMacros;
  end;

implementation


{ TFPDocCreator }

procedure TFPDocCreator.SetVerbose(AValue: Boolean);
begin
  if FVerbose=AValue then Exit;
  FVerbose:=AValue;
  if FVerbose then
    begin
    ScannerLogEvents:=[sleFile];
    ParserLogEvents:=[];
    end
  else
    begin
    ScannerLogEvents:=[];
    ParserLogEvents:=[];
    end;
end;

procedure TFPDocCreator.DoLog(const Msg: String);
begin
  If Assigned(OnLog) then
    OnLog(Self,Msg);
end;

procedure TFPDocCreator.DoLog(const Fmt: String; Args: array of const);
begin
  DoLog(Format(Fmt,Args));
end;

procedure TFPDocCreator.DoLogSender ( Sender: TObject; const Msg: String ) ;
begin
  if Assigned(Sender) then
    DoLog(Format('%s - Sender: %s', [Msg, Sender.ClassName]))
  else
    DoLog(Msg);
end;

procedure TFPDocCreator.HandleOnParseUnit(Sender: TObject;
  const AUnitName: String; out AInputFile, OSTarget, CPUTarget: String);

Var
  I : Integer;
  S,un,opts : String;

begin
  AInputFile:='';
  OSTarget:='';
  CPUTarget:='';
  if Assigned(FCurPackage) then
    begin
    I:=0;
    While (AInputFIle='') and (I<FCurPackage.Inputs.Count) do
       begin
       S:=FCurPackage.Inputs[i];
       SplitInputFIleOption(S,UN,Opts);
       if CompareText(ChangeFileExt(ExtractFileName(Un),''),AUnitName)=0 then
         begin
         AInputFile:=FixInputFile(UN)+' '+Opts+' -d'+Options.EndianNess;
         OSTarget:=FProject.Options.OSTarget;
         CPUTarget:=FProject.Options.CPUTarget;
         FProcessedUnits.Add(UN);
         end;
       Inc(I);
       end;
   end;
end;

function TFPDocCreator.GetOptions: TEngineOptions;
begin
  Result:=FProject.Options;
end;

function TFPDocCreator.GetPackages: TFPDocPackages;
begin
  Result:=FProject.Packages;
end;

function TFPDocCreator.FixInputFile(const AFileName: String): String;
begin
  Result:=AFileName;
  If Result='' then exit;
  if (ExtractFileDrive(Result)='') and (Result[1]<>PathDelim) then
    Result:=BaseInputDir+Result;
end;

function TFPDocCreator.FixDescrFile(const AFileName: String): String;
begin
  Result:=AFileName;
  If Result='' then exit;
  if (ExtractFileDrive(Result)='') and (Result[1]<>PathDelim) then
    Result:=BaseDescrDir+Result;
end;

procedure TFPDocCreator.SetBaseDescrDir(AValue: String);
begin
  if FBaseDescrDir=AValue then Exit;
  FBaseDescrDir:=AValue;
  If FBaseDescrDir<>'' then
    FBaseDescrDir:=IncludeTrailingPathDelimiter(FBaseDescrDir);
end;

procedure TFPDocCreator.SetBaseInputDir(AValue: String);
begin
  if FBaseInputDir=AValue then Exit;
  FBaseInputDir:=AValue;
  If FBaseInputDir<>'' then
    FBaseInputDir:=IncludeTrailingPathDelimiter(FBaseInputDir);
end;

procedure TFPDocCreator.SetProjectMacros(AValue: TStrings);
begin
  if FProjectMacros=AValue then Exit;
  FProjectMacros.Assign(AValue);
end;

procedure TFPDocCreator.DoBeforeEmitNote(Sender: TObject; Note: TDomElement;
  var EmitNote: Boolean);
begin
  EmitNote:=True;
end;

constructor TFPDocCreator.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FProject:=TFPDocProject.Create(Self);
  FProject.Options.StopOnParseError:=False;
  FProject.Options.CPUTarget:=DefCPUTarget;
  FProject.Options.OSTarget:=DefOSTarget;
  FProject.Options.EndianNess:=DefEndianNess;
  FProcessedUnits:=TStringList.Create;
  FProjectMacros:=TStringList.Create;
end;

destructor TFPDocCreator.Destroy;
begin
  FreeAndNil(FProcessedUnits);
  FreeAndNil(FProject);
  FreeAndNil(FProjectMacros);
  inherited Destroy;
end;

procedure TFPDocCreator.CreateOutput(APackage: TFPDocPackage;Engine : TFPDocEngine);

Var
  WriterClass : TFPDocWriterClass;
  Writer : TFPDocWriter;
  I : Integer;
  Cmd,Arg : String;

begin
  // Now is used the specified writer
  WriterClass:=GetWriterClass(Options.Backend);
  // ALL CONTENT CREATED HERE
  Writer:=WriterClass.Create(Engine.Package,Engine);
  With Writer do
    Try
      If FVerbose then
        DoLog('Writing documentation');
      OnLog:=Self.OnLog;
      BeforeEmitNote:=@self.DoBeforeEmitNote;
      EmitNotes:=Options.EmitNotes;
      If Options.BackendOptions.Count>0 then
        for I:=0 to ((Options.BackendOptions.Count-1) div 2) do
          begin
          Cmd:=Options.BackendOptions[I*2];
          Arg:=Options.BackendOptions[I*2+1];
          If not InterPretOption(Cmd,Arg) then
            DoLog(SCmdLineInvalidOption,[Cmd+'='+Arg]);
          end;
      // Output created Documentation
      WriteDoc;
    Finally
      Free;
    end;
  // Output content files
  if FVerbose then
    DoLog('Content file : '+APackage.ContentFile);
  if Length(APackage.ContentFile) > 0 then
    Engine.WriteContentFile(APackage.ContentFile);
end;

procedure TFPDocCreator.CreateDocumentation(APackage: TFPDocPackage;
  ParseOnly: Boolean);

var
  i,j: Integer;
  Engine : TFPDocEngine;
  Cmd,Arg : String;
  WriterClass: TFPDocWriterClass;

begin
  Cmd:='';
  FCurPackage:=APackage;
  Engine:=TFPDocEngine.Create;
  Engine.OnLog:= @DoLogSender;
  try
    // get documentation Writer html, latex, and other
    WriterClass:=GetWriterClass(Options.Backend);
    For J:=0 to Apackage.Imports.Count-1 do
      begin
      Arg:=Apackage.Imports[j];
      // conversion import FilePathes
      WriterClass.SplitImport(Arg,Cmd);
      // create tree of imported objects
      Engine.ReadContentFile(Arg, Cmd);
      end;
    for i := 0 to APackage.Descriptions.Count - 1 do
      Engine.AddDocFile(FixDescrFile(APackage.Descriptions[i]),Options.donttrim);
    // set engine options
    Engine.SetPackageName(APackage.Name);
    Engine.Output:=APackage.Output;
    Engine.OnLog:=Self.OnLog;
    Engine.ScannerLogEvents:=Self.ScannerLogEvents;
    Engine.ParserLogEvents:=Self.ParserLogEvents;
    Engine.HideProtected:=Options.HideProtected;
    Engine.HidePrivate:=Not Options.ShowPrivate;
    Engine.OnParseUnit:=@HandleOnParseUnit;
    Engine.WarnNoNode:=Options.WarnNoNode;
    if Length(Options.Language) > 0 then
      TranslateDocStrings(Options.Language);
    // scan the input source files
    for i := 0 to APackage.Inputs.Count - 1 do
      try
        // get options from input packages
        SplitInputFileOption(APackage.Inputs[i],Cmd,Arg);
        arg:=Arg+' -d'+Options.EndianNess;
        // make absolute filepath
        Cmd:=FixInputFile(Cmd);
        if FProcessedUnits.IndexOf(Cmd)=-1 then
          begin
          FProcessedUnits.Add(Cmd);

          // Parce sources for OS Target
          //WriteLn(Format('Parsing unit: %s', [ExtractFilenameOnly(Cmd)]));
          ParseSource(Engine,Cmd+' '+Arg, Options.OSTarget, Options.CPUTarget,[poUseStreams]);
          end;
      except
        on E: EParserError do
          If Options.StopOnParseError then
            Raise
          else
            begin
            DoLog('Error: %s(%d,%d): %s',[E.Filename, E.Row, E.Column, E.Message]);
            DoLog('Ignoring error, continuing with next unit (if any).');
            end;
      end;
    if Not ParseOnly then
      begin
      Engine.StartDocumenting;
      // Create documentation
      CreateOutput(APackage,Engine);
      end;
  finally
    FreeAndNil(Engine);
    FCurPackage:=Nil;
  end;
end;

procedure TFPDocCreator.CreateProjectFile(const AFileName: string);
begin
  With TXMLFPDocOptions.Create(Self) do
  try
    SaveOptionsToFile(FProject,AFileName);
  finally
    Free;
  end;
end;

procedure TFPDocCreator.LoadProjectFile(const AFileName: string);
begin
  With TXMLFPDocOptions.Create(self) do
    try
      if (ProjectMacros.Count>0) then
        LoadOptionsFromFile(FProject,AFileName,ProjectMacros)
      else
        LoadOptionsFromFile(FProject,AFileName,Nil);
    finally
      Free;
    end;
end;

end.

