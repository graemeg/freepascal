{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018  Michael Van Canneyt

    Pascal to Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

  Abstract:
    FileSystem aware compiler descendent with support for PCU files.
}
unit Pas2JSPCUCompiler;

{$mode objfpc}{$H+}

{$I pas2js_defines.inc}

{$IF defined(VerboseUnitQueue) or defined(VerbosePCUFiler)}
{$DEFINE ReallyVerbose}
{$ENDIF}

interface

uses
  SysUtils, Classes,
  jstree,
  PasTree, PScanner, PasResolveEval,
  FPPas2Js,
  Pas2jsCompiler, Pas2JSFS, Pas2JSFSCompiler, Pas2JsFiler,
  Pas2jsLogger, Pas2jsFileUtils, FPPJsSrcMap;

Type

  { TFilerPCUSupport }

  TFilerPCUSupport = Class(TPCUSupport)
  Private
    FPCUFormat: TPas2JSPrecompileFormat;
    FPrecompileInitialFlags: TPCUInitialFlags;
    FPCUReader: TPCUCustomReader;
    FPCUReaderStream: TStream;
    function OnPCUConverterIsElementUsed(Sender: TObject; El: TPasElement): boolean;
    function OnPCUConverterIsTypeInfoUsed(Sender: TObject; El: TPasElement): boolean;
    function OnWriterIsElementUsed(Sender: TObject; El: TPasElement): boolean;
    procedure OnFilerGetSrc(Sender: TObject; aFilename: string; out p: PChar; out Count: integer);
  Public
    constructor Create(aCompilerFile: TPas2JSCompilerFile; aFormat: TPas2JSPrecompileFormat); reintroduce;
    destructor Destroy; override;
    function Compiler: TPas2JSCompiler;
    function HandleException(E: Exception): Boolean; override;
    function FindPCU(const UseUnitName: string): string; override;
    function FindPCU(const UseUnitName: string; out aFormat: TPas2JSPrecompileFormat): string;
    function HasReader: Boolean; override;
    function ReadContinue: Boolean; override;
    function ReadCanContinue: Boolean; override;
    procedure SetInitialCompileFlags; override;
    procedure WritePCU; override;
    procedure CreatePCUReader; override;
    procedure ReadUnit; override;
    property PrecompileInitialFlags: TPCUInitialFlags read FPrecompileInitialFlags;
  end;

  { TPas2jsPCUCompilerFile }

  TPas2jsPCUCompilerFile = Class(TPas2jsCompilerFile)
    function CreatePCUSupport: TPCUSupport; override;
  end;

  { TPas2jsPCUCompiler }

  TPas2jsPCUCompiler = Class(TPas2JSFSCompiler)
  Private
    FPrecompileFormat: TPas2JSPrecompileFormat;
  Protected
    function CreateJSMapper: TPas2JSMapper; override;
    procedure WritePrecompiledFormats; override;
    function CreateCompilerFile(const PasFileName, PCUFilename: String): TPas2jsCompilerFile; override;
    procedure HandleOptionPCUFormat(Value: string); override;
    property PrecompileFormat: TPas2JSPrecompileFormat read FPrecompileFormat;
  end;

implementation

{$IFDEF HASPAS2JSFILER}

{ ---------------------------------------------------------------------
  TFilerPCUSupport
  ---------------------------------------------------------------------}

{ TFilerPCUSupport }

constructor TFilerPCUSupport.Create(aCompilerFile: TPas2JSCompilerFile; aFormat: TPas2JSPrecompileFormat);
begin
  Inherited Create(aCompilerFile);
  FPCUFormat:=AFormat;
  if FPCUFormat=nil then
    RaiseInternalError(20181207143653,aCompilerFile.UnitFilename);
  FPrecompileInitialFlags:=TPCUInitialFlags.Create;
end;

destructor TFilerPCUSupport.Destroy;
begin
  FreeAndNil(FPrecompileInitialFlags);
  FreeAndNil(FPCUReader);
  FreeAndNil(FPCUReaderStream);
  inherited Destroy;
end;

function TFilerPCUSupport.Compiler: TPas2JSCompiler;
begin
  Result:=MyFile.Compiler;
end;

function TFilerPCUSupport.HandleException(E: Exception): Boolean;

begin
  Result:=False;
  if E is EPas2JsReadError then
    begin
    Result:=True;
    if EPas2JsReadError(E).Owner is TPCUCustomReader then
      MyFile.Log.Log(mtError,E.Message,0,MyFile.PCUFilename)
    else
      MyFile.Log.Log(mtError,E.Message);
    Compiler.Terminate(ExitCodePCUError);
    end
  else if (E is EPas2JsWriteError) then
    begin
    MyFile.Log.Log(mtFatal,E.ClassName+':'+E.Message);
    Compiler.Terminate(ExitCodeErrorInternal);
    Result:=True;
    end
end;

function TFilerPCUSupport.FindPCU(const UseUnitName: string): string;

var
  aPCUFormat: TPas2JSPrecompileFormat;
begin
  Result:=FindPCU(UseUnitName,aPCUFormat);
  if (Result<>'') and (FPCUFormat<>aPCUFormat) then
    RaiseInternalError(20181207143826,UseUnitName);
end;

function TFilerPCUSupport.HasReader: Boolean;
begin
  Result:=Assigned(FPCUReader);
end;

function TFilerPCUSupport.ReadContinue: Boolean;
begin
  Result:=FPCUReader.ReadContinue;
end;

function TFilerPCUSupport.ReadCanContinue: Boolean;
begin
  Result:=FPCUReader.ReadCanContinue;
end;

procedure TFilerPCUSupport.SetInitialCompileFlags;
begin
  PrecompileInitialFlags.ParserOptions:=MyFile.Parser.Options;
  PrecompileInitialFlags.ModeSwitches:=MyFile.Scanner.CurrentModeSwitches;
  PrecompileInitialFlags.BoolSwitches:=MyFile.Scanner.CurrentBoolSwitches;
  PrecompileInitialFlags.ConverterOptions:=MyFile.GetInitialConverterOptions+PCUMinConverterOptions;
  PrecompileInitialFlags.TargetPlatform:=Compiler.TargetPlatform;
  PrecompileInitialFlags.TargetProcessor:=Compiler.TargetProcessor;
end;

procedure TFilerPCUSupport.CreatePCUReader;
var
  aFile: TPas2jsFile;
  s: String;
begin
  if MyFile.PCUFilename='' then
    RaiseInternalError(20180312144742,MyFile.PCUFilename);
  if FPCUReader<>nil then
    RaiseInternalError(20180312142938,GetObjName(FPCUReader));
  if FPCUFormat=nil then
    RaiseInternalError(20180312142954,'');
  FPCUReader:=FPCUFormat.ReaderClass.Create;
  FPCUReader.SourceFilename:=ExtractFileName(MyFile.PCUFilename);
  FPCUReader.PCUFilename:=MyFile.PCUFilename;

  if MyFile.ShowDebug then
    MyFile.Log.LogMsg(nParsingFile,[QuoteStr(MyFile.PCUFilename)]);
  aFile:=Compiler.FS.LoadFile(MyFile.PCUFilename,true);
  if aFile=nil then
    RaiseInternalError(20180312145941,MyFile.PCUFilename);
  FPCUReaderStream:=TMemoryStream.Create;
  s:=aFile.Source;
  //writeln('TPas2jsCompilerFile.CreatePCUReader ',PCUFilename,'-----START-----');
  //writeln(s);
  //writeln('TPas2jsCompilerFile.CreatePCUReader ',PCUFilename,'-----END-------');
  if s<>'' then
  begin
    FPCUReaderStream.Write(s[1],length(s));
    FPCUReaderStream.Position:=0;
  end;
end;

procedure TFilerPCUSupport.ReadUnit;
begin
  FPCUReader.ReadPCU(MyFile.PascalResolver,FPCUReaderStream);
  SetPasModule(MyFile.PascalResolver.RootElement);
  SetReaderState(prsCanContinue);
end;

function TFilerPCUSupport.FindPCU(const UseUnitName: string;
  out aFormat: TPas2JSPrecompileFormat): string;

  function SearchInDir(DirPath: string): boolean;
  var
    i: Integer;
    CurFormat: TPas2JSPrecompileFormat;
    Filename: String;
  begin
    if DirPath='' then exit(false);
    DirPath:=IncludeTrailingPathDelimiter(DirPath);
    for i:=0 to PrecompileFormats.Count-1 do
    begin
      CurFormat:=PrecompileFormats[i];
      if not CurFormat.Enabled then continue;
      Filename:=DirPath+UseUnitName+'.'+CurFormat.Ext;
      if Compiler.FS.PCUExists(Filename) then
      begin
        FindPCU:=Filename;
        aFormat:=CurFormat;
        exit(true);
      end;
    end;
    Result:=false;
  end;

var
  L: TstringList;
  i: Integer;

begin
  Result:='';
  aFormat:=nil;
  L:=TStringList.Create;
  try
    Compiler.FS.GetPCUDirs(L,MyFile.FileResolver.BaseDirectory);
    for i:=0 to L.Count-1 do
      if SearchInDir(L[i]) then exit;
  finally
    L.Free;
  end;
end;

function TFilerPCUSupport.OnWriterIsElementUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  Result:=MyFile.UseAnalyzer.IsUsed(El);
end;

procedure TFilerPCUSupport.WritePCU;

Const
  AllowCompressed =
{$IFDEF DisablePCUCompressed}false{$ELSE}true{$ENDIF};

var
  Writer: TPCUWriter;
  ms: TMemoryStream;
  DestDir: String;
  JS: TJSElement;
  FN: String;

begin
  if FPCUFormat=Nil then
    exit; // Don't write
  if MyFile.PasModule.ClassType<>TPasModule then
  begin
    {$IFDEF REALLYVERBOSE}
    writeln('TPas2jsCompilerFile.WritePCU not a unit: ',MyFile.PasFilename,' skip');
    {$ENDIF}
    exit;
  end;

  if (MyFile.PCUFilename<>'') or (FPCUReader<>nil) then
  begin
    {$IFDEF REALLYVERBOSE}
    writeln('TPas2jsCompilerFile.WritePCU already precompiled "',MyFile.PCUFilename,'" Reader=',GetObjName(FPCUReader));
    {$ENDIF}
    exit;
  end;

  // Determine output filename
  FN:=ExtractFilenameOnly(MyFile.PasFilename)+'.'+FPCUFormat.Ext;
  if Compiler.FS.UnitOutputPath<>'' then
    FN:=Compiler.FS.UnitOutputPath+FN
  else
    FN:=ExtractFilePath(MyFile.PasFilename)+FN;
  // Set as our filename
  SetPCUFilename(FN);
  {$IFDEF REALLYVERBOSE}
  writeln('TPas2jsCompilerFile.WritePCU precompiling ',MyFile.PCUFilename);
  {$ENDIF}

  JS:=nil;
  ms:=TMemoryStream.Create;
  Writer:=FPCUFormat.WriterClass.Create;
  try
    Writer.GUID:=Compiler.PrecompileGUID;
    Writer.OnGetSrc:=@OnFilerGetSrc;
    Writer.OnIsElementUsed:=@OnWriterIsElementUsed;

    // create JavaScript for procs, initialization, finalization
    MyFile.CreateConverter;
    MyFile.Converter.Options:=MyFile.Converter.Options+PCUMinConverterOptions;
    MyFile.Converter.OnIsElementUsed:=@OnPCUConverterIsElementUsed;
    MyFile.Converter.OnIsTypeInfoUsed:=@OnPCUConverterIsTypeInfoUsed;
    JS:=MyFile.Converter.ConvertPasElement(MyFile.PasModule,MyFile.PascalResolver);
    MyFile.PCUSupport.SetInitialCompileFlags;
    {$IFDEF REALLYVERBOSE}
    writeln('TPas2jsCompilerFile.WritePCU create pcu ... ',MyFile.PCUFilename);
    {$ENDIF}
    Writer.WritePCU(MyFile.PascalResolver,MyFile.Converter,
              PrecompileInitialFlags,ms,AllowCompressed);
    {$IFDEF REALLYVERBOSE}
    writeln('TPas2jsCompilerFile.WritePCU precompiled ',MyFile.PCUFilename);
    {$ENDIF}

    MyFile.Log.LogMsg(nWritingFile,[QuoteStr(Compiler.FS.FormatPath(MyFile.PCUFilename))],'',0,0,
               not (coShowLineNumbers in Compiler.Options));

    // check output directory
    DestDir:=ChompPathDelim(ExtractFilePath(MyFile.PCUFilename));
    if (DestDir<>'') and not Compiler.FS.DirectoryExists(DestDir) then
    begin
      {$IFDEF REALLYVERBOSE}
      writeln('TPas2jsCompilerFile.WritePCU output dir not found "',DestDir,'"');
      {$ENDIF}
      MyFile.Log.LogMsg(nOutputDirectoryNotFound,[QuoteStr(Compiler.FS.FormatPath(DestDir))]);
      Compiler.Terminate(ExitCodeFileNotFound);
    end;
    if Compiler.FS.DirectoryExists(MyFile.PCUFilename) then
    begin
      {$IFDEF REALLYVERBOSE}
      writeln('TPas2jsCompilerFile.WritePCU file is folder "',DestDir,'"');
      {$ENDIF}
      MyFile.Log.LogMsg(nFileIsFolder,[QuoteStr(Compiler.FS.FormatPath(MyFile.PCUFilename))]);
      Compiler.Terminate(ExitCodeWriteError);
    end;

    ms.Position:=0;
    Compiler.FS.SaveToFile(ms,MyFile.PCUFilename);
    {$IFDEF REALLYVERBOSE}
    writeln('TPas2jsCompilerFile.WritePCU written ',MyFile.PCUFilename);
    {$ENDIF}
  finally
    JS.Free;
    Writer.Free;
    ms.Free;
  end;
end;

procedure TFilerPCUSupport.OnFilerGetSrc(Sender: TObject; aFilename: string;
  out p: PChar; out Count: integer);
var
  SrcFile: TPas2jsFile;
begin
  if Sender=nil then
    RaiseInternalError(20180311135558,aFilename);
  SrcFile:=MyFile.Compiler.FS.LoadFile(aFilename);
  if SrcFile=nil then
    RaiseInternalError(20180311135329,aFilename);
  p:=PChar(SrcFile.Source);
  Count:=length(SrcFile.Source);
end;

function TFilerPCUSupport.OnPCUConverterIsElementUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  if (coKeepNotUsedPrivates in MyFile.Compiler.Options) then
    Result:=true
  else
    Result:=MyFile.UseAnalyzer.IsUsed(El);
end;

function TFilerPCUSupport.OnPCUConverterIsTypeInfoUsed(Sender: TObject;
  El: TPasElement): boolean;
begin
  if Sender=nil then ;
  if El=nil then ;
  // PCU does not need precompiled typeinfo
  Result:=false;
end;

{ TPas2jsPCUCompiler }

function TPas2jsPCUCompiler.CreateJSMapper: TPas2JSMapper;
begin
  Result:=inherited CreateJSMapper;
  if PrecompileFormat<>nil then
    Result.PCUExt:='.'+PrecompileFormat.Ext;
end;

procedure TPas2jsPCUCompiler.WritePrecompiledFormats;
Var
  I: Integer;
begin
  if PrecompileFormats.Count>0 then
  begin
    if PrecompileFormats.Count>1 then
    begin
      writeHelpLine('   -JU<x>: Create precompiled units in format x.');
      for i:=0 to PrecompileFormats.Count-1 do
        with PrecompileFormats[i] do
          writeHelpLine('     -JU'+Ext+':  '+Description);
      writeHelpLine('     -JU-: Disable prior -JU<x> option. Do not create precompiled units.');
    end else
    begin
      with PrecompileFormats[0] do
        writeHelpLine('   -JU'+Ext+': Create precompiled units using '+Description);
      writeHelpLine('   -JU-  : Disable prior -JU<x> option. Do not create precompiled units.');
    end;
  end;
end;

function TPas2jsPCUCompiler.CreateCompilerFile(const PasFileName,
  PCUFilename: String): TPas2jsCompilerFile;
begin
  Result:=TPas2JSPCUCompilerFile.Create(Self,PasFileName,PCUFilename);
end;

procedure TPas2jsPCUCompiler.HandleOptionPCUFormat(Value: string);
Var
  Found: Boolean;
  I: integer;
  PF: TPas2JSPrecompileFormat;
begin
  Found:=false;
  for i:=0 to PrecompileFormats.Count-1 do
  begin
    PF:=PrecompileFormats[i];
    if not SameText(Value,PF.Ext) then continue;
    FPrecompileFormat:=PrecompileFormats[i];
    Options:=Options+[coPrecompile];
    Found:=true;
  end;
  if not Found then
    ParamFatal('invalid precompile output format (-JU) "'+Value+'"');
end;

{ TPas2jsPCUCompilerFile }

function TPas2jsPCUCompilerFile.CreatePCUSupport: TPCUSupport;
Var
  PF: TPas2JSPrecompileFormat;
begin
  // Note that if no format was preset, no files will be written
  PF:=(Compiler as TPas2jsPCUCompiler).FPrecompileFormat;
  if (PF=nil) and (PrecompileFormats.Count>0) then
    PF:=PrecompileFormats[0];
  if PF<>Nil then
    Result:=TFilerPCUSupport.Create(Self,PF)
  else
    Result:=Nil;
end;
{$ENDIF}

end.

