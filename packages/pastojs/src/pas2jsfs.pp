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
    FileSystem abstraction layer for compiler.
    Has only abstract classes with no actual implementation, so it does not actually
    interacts with the filesystem.
    See Pas2JSFileCache for an actual implementation.
}
unit Pas2JSFS;

{$mode objfpc}{$H+}
{$I pas2js_defines.inc}

interface

uses
  // No filesystem-dependent units here !
  Classes, SysUtils, PScanner, fpjson;

const // Messages
  nUsingPath = 104; sUsingPath = 'Using %s: "%s"';
  nFolderNotFound = 105; sFolderNotFound = '%s not found: %s';

  nIncludeSearch = 201; sIncludeSearch = 'Include file search: %s';
  nUnitSearch = 202; sUnitSearch = 'Unitsearch: %s';
  nSearchingFileFound = 203; sSearchingFileFound = 'Searching file: %s... found';
  nSearchingFileNotFound = 204; sSearchingFileNotFound = 'Searching file: %s... not found';
  nDuplicateFileFound = 205; sDuplicateFileFound = 'Duplicate file found: "%s" and "%s"';
  nCustomJSFileNotFound = 206; sCustomJSFileNotFound = 'custom JS file not found: "%s"';

Type
  // Forward definitions
  EPas2jsFS = Class(Exception);
  TPas2jsFile = class;
  TSourceLineReader = class;
  TPas2jsFSResolver = class;
  TPas2JSFS = Class;

  { TSourceLineReader }

  TSourceLineReader = class(TLineReader)
  private
    FIsEOF: boolean;
    FLineNumber: integer;
    FSource: string;
    FSrcPos: integer;
  Protected
    Procedure IncLineNumber; virtual;
    property Source: string read FSource;
    property SrcPos: integer read FSrcPos;
  public
    Constructor Create(Const aFileName, aSource: String); overload;
    function IsEOF: Boolean; override;
    function ReadLine: string; override;
    property LineNumber: integer read FLineNumber;
  end;

  TP2jsFSOption = (
    caoShowFullFilenames,
    caoShowTriedUsedFiles,
    caoSearchLikeFPC,
    caoStrictFileCase
    );
  TP2jsFSOptions = set of TP2jsFSOption;
  TKeyCompareType = (kcFilename,kcUnitName);

  { TPas2JSFS }

  TPas2JSFS = Class
  Private
    FOptions: TP2jsFSOptions;
    FReadLineCounter: SizeInt;
    FDefaultOutputPath: string;
    FUnitOutputPath: string;
    procedure SetOptionFromIndex(AIndex: Integer; AValue: boolean);
    procedure SetDefaultOutputPath(AValue: string);
    procedure SetUnitOutputPath(AValue: string);
  Protected
    // Not to be overridden
    procedure SetOption(Flag: TP2jsFSOption; Enable: boolean);
    Function OptionIsSet(Index: Integer):  Boolean;
  Protected
    // Protected Abstract. Must be overridden
    function FindSourceFileName(const aFilename: string): String; virtual; abstract;
  Public
    // Public Abstract. Must be overridden
    function FindResourceFileName(const aFilename, ModuleDir: string): String; virtual; abstract;
    function FindIncludeFileName(const aFilename, SrcDir, ModuleDir: string; Mode: TModeSwitch): String; virtual; abstract;
    function LoadFile(Filename: string; Binary: boolean = false): TPas2jsFile; virtual; abstract;
    Function FileExists(Const aFileName: String): Boolean; virtual; abstract;
    function FindUnitJSFileName(const aUnitFilename: string): String; virtual; abstract;
    function FindCustomJSFileName(const aFilename: string): String; virtual; abstract;
    function FindUnitFileName(const aUnitname, InFilename, ModuleDir: string; out IsForeign: boolean): String; virtual; abstract;
    procedure SaveToFile(ms: TFPJSStream; Filename: string); virtual; abstract;
    function PCUExists(var aFileName: string): Boolean; virtual;
    procedure GetPCUDirs(aList: TStrings; const aBaseDir: String); virtual;
  Public
    // Public, may be overridden
    Function SameFileName(Const File1,File2: String): Boolean; virtual;
    Function File1IsNewer(Const File1,File2: String): Boolean; virtual;
    function ExpandDirectory(const Filename: string): string; virtual;
    function ExpandFileName(const Filename: string): string; virtual;
    function ExpandExecutable(const Filename: string): string; virtual;
    Function FormatPath(Const aFileName: string): String; virtual;
    Function DirectoryExists(Const aDirectory: string): boolean; virtual;
    function TryCreateRelativePath(const Filename, BaseDirectory: String;
      UsePointDirectory, AlwaysRequireSharedBaseFolder: boolean; out RelPath: String): Boolean; virtual;
    procedure DeleteDuplicateFiles(List: TStrings); virtual;
    function IndexOfFile(FileList: TStrings; aFilename: string; Start: integer = 0): integer; virtual;// -1 if not found
    Procedure WriteFoldersAndSearchPaths; virtual;
    function CreateResolver: TPas2jsFSResolver; virtual;
    // On success, return '', On error, return error message.
    Function AddForeignUnitPath(Const aValue: String; FromCmdLine: Boolean): String; virtual;
    Function HandleOptionPaths(C: Char; aValue: String; FromCmdLine: Boolean): String; virtual;
  Public
    Constructor Create; virtual;
    Procedure Reset; virtual;
    Procedure IncReadLineCounter;
    property ReadLineCounter: SizeInt read FReadLineCounter write FReadLineCounter;
    property Options: TP2jsFSOptions read FOptions write FOptions;
    property ShowFullPaths: boolean Index 0 Read OptionIsSet Write SetOptionFromIndex;
    property ShowTriedUsedFiles: boolean Index 1 read OptionIsSet Write SetOptionFromIndex;
    property SearchLikeFPC: boolean index 2 read OptionIsSet Write SetOptionFromIndex;
    Property StrictFileCase: Boolean Index 3 Read OptionIsSet Write SetOptionFromIndex;
    property MainOutputPath: string read FDefaultOutputPath write SetDefaultOutputPath; // includes trailing pathdelim
    property UnitOutputPath: string read FUnitOutputPath write SetUnitOutputPath; // includes trailing pathdelim
  end;

  { TPas2jsFile }

  TPas2jsFile = class
  private
    FFilename: string;
    FFS: TPas2JSFS;
    FSource: string;
  Protected
    Procedure SetSource(aSource: String);
  public
    constructor Create(aFS: TPas2jsFS; const aFilename: string);
    function CreateLineReader(RaiseOnError: boolean): TSourceLineReader; virtual; abstract;
    function Load(RaiseOnError: boolean; Binary: boolean): boolean; virtual; abstract;
    property Source: string read FSource; // UTF-8 without BOM or Binary
    Property FS: TPas2JSFS Read FFS;
    property Filename: string read FFilename;
  end;

  { TPas2jsFSResolver }

  TPas2jsFSResolver = class({$IFDEF HASFILESYSTEM}TFileResolver{$ELSE}TBaseFileResolver{$ENDIF})
  private
    FFS: TPas2jsFS;
  public
    constructor Create(aFS: TPas2jsFS); reintroduce;
    // Redirect all calls to FS.
    function FindResourceFileName(const aFilename: string): String; override;
    function FindIncludeFileName(const aFilename: string): String; override;
    function FindIncludeFile(const aFilename: string): TLineReader; override;
    function FindSourceFile(const aFilename: string): TLineReader; override;
    property FS: TPas2jsFS read FFS;
  end;


Const
  p2jsfcoCaption: array[TP2jsFSOption] of string = (
      // only used by experts, no need for resourcestrings
      'Show full filenames',
      'Show tried/used files',
      'Search files like FPC',
      'Strict file case'
      );
    // 'Combine all JavaScript into main file',
    EncodingBinary = 'Binary';

  DefaultPas2jsFSOptions = [];

implementation

// No filesystem-dependent units here !

{ TPas2JSFS }

procedure TPas2JSFS.SetOptionFromIndex(AIndex: Integer; AValue: boolean);
begin
  SetOption(TP2jsFSOption(aIndex),aValue);
end;

procedure TPas2JSFS.SetOption(Flag: TP2jsFSOption; Enable: boolean);
begin
  if Enable then
    Include(FOptions,Flag)
  else
    Exclude(FOptions,Flag);
end;

function TPas2JSFS.OptionIsSet(Index: Integer): Boolean;
begin
  Result:=TP2jsFSOption(Index) in FOptions;
end;

function TPas2JSFS.PCUExists(var aFileName: string): Boolean;
begin
  Result:=Self.FileExists(aFileName);
end;

procedure TPas2JSFS.GetPCUDirs(aList: TStrings; const aBaseDir: String);
begin
  if UnitOutputPath<>'' then
    aList.Add(UnitOutputPath);
  aList.Add(aBaseDir);
end;

function TPas2JSFS.SameFileName(const File1, File2: String): Boolean;
begin
  Result:=CompareText(File1,File2)=0;
end;

function TPas2JSFS.File1IsNewer(const File1, File2: String): Boolean;
begin
  Result:=False;
  if File1=File2 then ;
end;

function TPas2JSFS.ExpandDirectory(const Filename: string): string;
begin
  Result:=FileName;
end;

function TPas2JSFS.ExpandFileName(const Filename: string): string;
begin
  Result:=FileName;
end;

function TPas2JSFS.ExpandExecutable(const Filename: string): string;
begin
  Result:=FileName
end;

function TPas2JSFS.FormatPath(const aFileName: string): String;
begin
  Result:=aFileName;
end;

function TPas2JSFS.DirectoryExists(const aDirectory: string): boolean;
begin
  Result:=aDirectory='';
end;

function TPas2JSFS.TryCreateRelativePath(const Filename, BaseDirectory: String;
  UsePointDirectory, AlwaysRequireSharedBaseFolder: boolean; out RelPath: String
  ): Boolean;
begin
  Result:=True;
  RelPath:=FileName;
  if (BaseDirectory='') or UsePointDirectory or AlwaysRequireSharedBaseFolder then ;
end;

procedure TPas2JSFS.DeleteDuplicateFiles(List: TStrings);
var
  i, j: Integer;
begin
  for i:=0 to List.Count-2 do
    for j:=List.Count-1 downto i+1 do
      if SameFileName(List[i],List[j]) then
        List.Delete(j);
end;

function TPas2JSFS.IndexOfFile(FileList: TStrings; aFilename: string;
  Start: integer): integer;
var
  i: Integer;
begin
  if FileList<>nil then
    for i:=Start to FileList.Count-1 do
      if SameFileName(FileList[i],aFilename) then exit(i);
  Result:=-1;
end;

procedure TPas2JSFS.WriteFoldersAndSearchPaths;
begin
  // Do nothing
end;

function TPas2JSFS.CreateResolver: TPas2jsFSResolver;
begin
  Result:=TPas2jsFSResolver.Create(Self);
end;

function TPas2JSFS.AddForeignUnitPath(const aValue: String; FromCmdLine: Boolean): String;
begin
  Result:='';
  if (aValue='') or FromCmdLine then ;
end;

function TPas2JSFS.HandleOptionPaths(C: Char; aValue: String; FromCmdLine: Boolean): String;
begin
  Result:='Invalid parameter: -F'+C+aValue;
  if FromCmdLine then ;
end;

constructor TPas2JSFS.Create;
begin
  FOptions:=DefaultPas2jsFSOptions;
end;

procedure TPas2JSFS.Reset;
begin
  FReadLineCounter:=0;
  FUnitOutputPath:='';
  FDefaultOutputPath:='';
end;

procedure TPas2JSFS.IncReadLineCounter;
begin
  Inc(FReadLineCounter);
end;

procedure TPas2JSFS.SetDefaultOutputPath(AValue: string);
begin
  AValue:=ExpandDirectory(AValue);
  if FDefaultOutputPath=AValue then Exit;
  FDefaultOutputPath:=AValue;
end;

procedure TPas2JSFS.SetUnitOutputPath(AValue: string);

begin
  AValue:=ExpandDirectory(AValue);
  if FUnitOutputPath=AValue then Exit;
  FUnitOutputPath:=AValue;
end;


{ TPas2jsFile }

procedure TPas2jsFile.SetSource(aSource: String);
begin
  FSource:=ASource;
end;

constructor TPas2jsFile.Create(aFS: TPas2jsFS; const aFilename: string);
begin
  FFS:=aFS;
  FFileName:=aFileName;
end;

procedure TSourceLineReader.IncLineNumber;
begin
  inc(FLineNumber);
end;

Constructor TSourceLineReader.Create(Const aFileName, aSource: String);
begin
  Inherited Create(aFileName);
  FSource:=aSource;
  FSrcPos:=1;
  FIsEOF:=FSource='';
end;

function TSourceLineReader.IsEOF: Boolean;
begin
  Result:=FIsEOF;
end;

function TSourceLineReader.ReadLine: string;
var
  S: string;
  p, SrcLen: integer;

  procedure GetLine;
  var
    l: SizeInt;
  begin
    l:=p-FSrcPos;
    Result:=copy(S,FSrcPos,l);
    FSrcPos:=p;
    IncLineNumber;
    //writeln('GetLine "',Result,'"');
  end;

begin
  if FIsEOF then exit('');
  S:=Source;
  SrcLen:=length(S);
  p:=FSrcPos;
  while p<=SrcLen do
    case S[p] of
    #10,#13:
      begin
        GetLine;
        inc(p);
        if (p<=SrcLen) and (S[p] in [#10,#13]) and (S[p]<>S[p-1]) then
          inc(p);
        if p>SrcLen then
          FIsEOF:=true;
        FSrcPos:=p;
        exit;
      end;
    else
      inc(p);
    end;
  FIsEOF:=true;
  GetLine;
end;


function TPas2jsFSResolver.FindIncludeFile(const aFilename: string): TLineReader;
var
  Filename: String;
begin
  Result:=nil;
  Filename:=FS.FindIncludeFileName(aFilename,BaseDirectory,ModuleDirectory,Mode);
  if Filename='' then exit;
  try
    Result:=FindSourceFile(Filename);
  except
    // error is shown in the scanner, which has the context information
  end;
end;

constructor TPas2jsFSResolver.Create(aFS: TPas2jsFS);
begin
  Inherited Create;
  FFS:=aFS;
end;

function TPas2jsFSResolver.FindResourceFileName(const aFilename: string): String;
begin
  Result:=FS.FindResourceFileName(aFilename,BaseDirectory);
end;

function TPas2jsFSResolver.FindIncludeFileName(const aFilename: string): String;

begin
  Result:=FS.FindIncludeFileName(aFilename,BaseDirectory,ModuleDirectory,Mode);
end;


function TPas2jsFSResolver.FindSourceFile(const aFilename: string): TLineReader;

var
  CurFilename: String;

begin
  CurFilename:=FS.FindSourceFileName(aFileName);
  Result:=FS.LoadFile(CurFilename).CreateLineReader(false);
end;



end.

