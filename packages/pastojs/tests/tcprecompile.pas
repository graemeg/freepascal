{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018 by Michael Van Canneyt

    Unit tests for Pascal-to-Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

 Examples:
    ./testpas2js --suite=TTestCLI_Precompile
    ./testpas2js --suite=TTestModule.TestEmptyUnit
}
unit TCPrecompile;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,
  fpcunit, testregistry, Pas2jsFileUtils, Pas2JsFiler, Pas2jsCompiler,
  TCUnitSearch, TCModules;

type

  { TCustomTestCLI_Precompile }

  TCustomTestCLI_Precompile = class(TCustomTestCLI)
  private
    FPCUFormat: TPas2JSPrecompileFormat;
    FUnitOutputDir: string;
  protected
    procedure SetUp; override;
    procedure CheckPrecompile(MainFile, UnitPaths: string;
      SharedParams: TStringList = nil;
      FirstRunParams: TStringList = nil;
      SecondRunParams: TStringList = nil; ExpExitCode: integer = 0);
    function GetJSFilename(ModuleName: string): string; virtual;
  public
    constructor Create; override;
    property PCUFormat: TPas2JSPrecompileFormat read FPCUFormat write FPCUFormat;
    property UnitOutputDir: string read FUnitOutputDir write FUnitOutputDir;
  end;

  { TTestCLI_Precompile }

  TTestCLI_Precompile = class(TCustomTestCLI_Precompile)
  published
    procedure TestPCU_EmptyUnit;
    procedure TestPCU_UnitWithoutImplementation;
    procedure TestPCU_UTF8BOM;
    procedure TestPCU_ParamNS;
    procedure TestPCU_Overloads;
    procedure TestPCU_Overloads_MDelphi_ModeObjFPC;
    procedure TestPCU_UnitCycle;
    procedure TestPCU_Class_Forward;
    procedure TestPCU_Class_Constructor;
    procedure TestPCU_Class_ClassConstructor;
    procedure TestPCU_ClassInterface;
    procedure TestPCU_EnumNames;
    procedure TestPCU_Namespace;
    procedure TestPCU_CheckVersionMain;
    procedure TestPCU_CheckVersionMain2;
    procedure TestPCU_CheckVersionSystem;
  end;

function LinesToList(const Lines: array of string): TStringList;

implementation

function LinesToList(const Lines: array of string): TStringList;
var
  i: Integer;
begin
  Result:=TStringList.Create;
  for i:=Low(Lines) to High(Lines) do Result.Add(Lines[i]);
end;

{ TCustomTestCLI_Precompile }

procedure TCustomTestCLI_Precompile.SetUp;
begin
  inherited SetUp;
  UnitOutputDir:='units';
end;

procedure TCustomTestCLI_Precompile.CheckPrecompile(MainFile,
  UnitPaths: string; SharedParams: TStringList; FirstRunParams: TStringList;
  SecondRunParams: TStringList; ExpExitCode: integer);
var
  JSFilename, OrigSrc, NewSrc, s: String;
  JSFile: TCLIFile;
begin
  try
    AddDir(UnitOutputDir);
    // compile, create  .pcu files
    {$IFDEF VerbosePCUFiler}
    writeln('TTestCLI_Precompile.CheckPrecompile create pcu files=========================');
    {$ENDIF}
    Params.Clear;
    Params.Add('-Jminclude');
    Params.Add('-Jc');
    if SharedParams<>nil then
      Params.AddStrings(SharedParams);
    if FirstRunParams<>nil then
      Params.AddStrings(FirstRunParams);
    Compile([MainFile,'-Fu'+UnitPaths,'-JU'+PCUFormat.Ext,'-FU'+UnitOutputDir]);
    AssertFileExists(UnitOutputDir+'/system.'+PCUFormat.Ext);
    JSFilename:=GetJSFilename(MainFile);
    AssertFileExists(JSFilename);
    JSFile:=FindFile(JSFilename);
    OrigSrc:=JSFile.Source;
    // compile, using .pcu files
    //for i:=0 to FileCount-1 do
    //  writeln('TCustomTestCLI_Precompile.CheckPrecompile ',i,' ',Files[i].Filename);
    {$IFDEF VerbosePCUFiler}
    writeln('TTestCLI_Precompile.CheckPrecompile compile using pcu files==================');
    {$ENDIF}
    JSFile.Source:='';
    Compiler.Reset;
    Params.Clear;
    Params.Add('-Jminclude');
    Params.Add('-Jc');
    if SharedParams<>nil then
      Params.AddStrings(SharedParams);
    if SecondRunParams<>nil then
      Params.AddStrings(SecondRunParams);
    Compile([MainFile,'-FU'+UnitOutputDir],ExpExitCode);
    if ExpExitCode=0 then
      begin
      NewSrc:=JSFile.Source;
      //writeln('TCustomTestCLI_Precompile.CheckPrecompile ',NewSrc);
      if not CheckSrcDiff(OrigSrc,NewSrc,s) then
        begin
        WriteSources;
        writeln('TCustomTestCLI_Precompile.CheckPrecompile OrigSrc==================');
        writeln(OrigSrc);
        writeln('TCustomTestCLI_Precompile.CheckPrecompile NewSrc==================');
        writeln(NewSrc);
        Fail('test1.js: '+s);
        end;
      end;
  finally
    SharedParams.Free;
    FirstRunParams.Free;
    SecondRunParams.Free;
  end;
end;

function TCustomTestCLI_Precompile.GetJSFilename(ModuleName: string): string;
begin
  Result:=UnitOutputDir+PathDelim+ExtractFilenameOnly(ModuleName)+'.js';
end;

constructor TCustomTestCLI_Precompile.Create;
begin
  inherited Create;
  FPCUFormat:=PrecompileFormats[0];
end;

{ TTestCLI_Precompile }

procedure TTestCLI_Precompile.TestPCU_EmptyUnit;
begin
  AddUnit('src/system.pp',[''],['']);
  AddFile('test1.pas',[
    'begin',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_UnitWithoutImplementation;
begin
  AddUnit('src/system.pp',[''],['']);
  AddFile('src/unit1.pas',
    'unit unit1;'+LineEnding
    +'interface'+LineEnding
    +'end.'+LineEnding);
  AddFile('src/unit2.pas',
    'unit unit2;'+LineEnding
    +'interface'+LineEnding
    +'uses unit1;'+LineEnding
    +'end.'+LineEnding);
  AddFile('test1.pas',[
    'uses unit2;',
    'begin',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_UTF8BOM;
var
  aFile: TCLIFile;
begin
  aFile:=AddUnit('src/system.pp',
    ['var',
    '  s: string = ''aaaäö'';',
    '  s2: string = ''😊'';', // 1F60A
    ''],
    ['']);
  aFile.Source:=UTF8BOM+aFile.Source;
  aFile:=AddFile('test1.pas',[
    'begin',
    '  s:=''ö😊'';',
    'end.']);
  aFile.Source:=UTF8BOM+aFile.Source;
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_ParamNS;
begin
  AddUnit('src/system.pp',[''],['']);
  AddUnit('src/foo.unit1.pp',['var i: longint;'],['']);
  AddFile('test1.pas',[
    'uses unit1;',
    'begin',
    '  i:=3;',
    'end.']);
  CheckPrecompile('test1.pas','src',LinesToList(['-FNfoo']));
end;

procedure TTestCLI_Precompile.TestPCU_Overloads;
begin
  AddUnit('src/system.pp',['type integer = longint;'],['']);
  AddUnit('src/unit1.pp',
  ['var i: integer;',
   'procedure DoIt(j: integer); overload;',
   'procedure DoIt(b: boolean);'],
  ['procedure DoIt(j: integer);',
   'begin',
   '  i:=j;',
   'end;',
   'procedure DoIt(b: boolean);',
   'begin',
   '  i:=3;',
   'end;']);
  AddUnit('src/unit2.pp',
  ['uses unit1;',
  'procedure DoIt(s: string); overload;'],
  ['procedure DoIt(s: string);',
   'begin',
   '  unit1.i:=length(s);',
   'end;']);
  AddFile('test1.pas',[
    'uses unit1, unit2;',
    'procedure DoIt(d: double); overload;',
    'begin',
    '  unit1.i:=4;',
    'end;',
    'begin',
    '  DoIt(3);',
    '  DoIt(''abc'');',
    '  DoIt(true);',
    '  DoIt(3.3);',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_Overloads_MDelphi_ModeObjFPC;
var
  SharedParams: TStringList;
begin
  AddUnit('src/system.pp',[
  'type',
  '  integer = longint;',
  '  TDateTime = type double;'],
  ['']);
  AddFile('src/unit1.pp',
    LinesToStr([
    'unit unit1;',
    '{$mode objfpc}',
    'interface',
    'function DoIt(i: integer): TDateTime;', // no overload needed in ObjFPC
    'function DoIt(i, j: integer): TDateTime;',
    'implementation',
    'function DoIt(i: integer): TDateTime;',
    'begin',
    '  Result:=i;',
    'end;',
    'function DoIt(i, j: integer): TDateTime;',
    'begin',
    '  Result:=i+j;',
    'end;',
    'end.']));
  AddFile('test1.pas',[
    'uses unit1;',
    'var d: TDateTime;',
    'begin',
    '  d:=DoIt(3);',
    '  d:=DoIt(4,5);',
    'end.']);
  SharedParams:=TStringList.Create;
  SharedParams.Add('-MDelphi');
  CheckPrecompile('test1.pas','src',SharedParams);
end;

procedure TTestCLI_Precompile.TestPCU_UnitCycle;
begin
  AddUnit('src/system.pp',['type integer = longint;'],['']);
  AddUnit('src/unit1.pp',
  ['var i: integer;',
   'procedure Do1(j: integer);'],
  ['uses unit2;',
   'procedure Do1(j: integer);',
   'begin',
   '  Do2(j);',
   'end;']);
  AddUnit('src/unit2.pp',
  ['uses unit1;',
   'procedure Do2(j: integer);'],
  ['procedure Do2(j: integer);',
   'begin',
   '  unit1.i:=j;',
   'end;']);
  AddFile('test1.pas',[
    'uses unit1;',
    'begin',
    '  Do1(3);',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_Class_Forward;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddUnit('src/unit1.pp',
  ['type',
   '  TClass = class of TObject;',
   '  TBirdClass = class of TBird;',
   '  TObject = class',
   '    FBirdClass: TBirdClass;',
   '    constructor Create;',
   '    constructor Create(Id: integer);',
   '    property BirdClass: TBirdClass read FBirdClass;',
   '  end;',
   '  TBird = class',
   '    constructor Create(d: double); overload;',
   '  end;',
   ''],
  ['constructor TObject.Create; begin end;',
   'constructor TObject.Create(Id: integer); begin end;',
   'constructor TBird.Create(d: double); begin end;']);
  AddFile('test1.pas',[
    'uses unit1;',
    'var',
    '  b: TBird;',
    '  c: TClass;',
    'begin',
    '  c:=TObject;',
    '  c:=TBird;',
    '  c:=b.BirdClass;',
    '  b:=TBird.Create;',
    '  b:=TBird.Create(1);',
    '  b:=TBird.Create(3.3);',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_Class_Constructor;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddUnit('src/unit1.pp',[
    'type',
    '  TObject = class',
    '    constructor Create;',
    '  end;',
    '  TBird = class',
    '    constructor Create; reintroduce;',
    '  end;',
    '  TCow = class',
    '    constructor Create; reintroduce;',
    '  end;',
    ''],[
    'constructor TObject.Create; begin end;',
    'constructor TBird.Create; begin end;',
    'constructor TCow.Create; begin end;',
    '']);
  AddUnit('src/unit2.pp',[
    'uses unit1;',
    'procedure DoIt;',
    ''],[
    'procedure DoIt;',
    'begin',
    '  TBird.Create;',
    '  TCow.Create;',
    'end;',
    '']);
  AddFile('test1.pas',[
    'uses unit2;',
    'begin',
    '  DoIt;',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_Class_ClassConstructor;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddUnit('src/unit1.pp',[
    'type',
    '  TObject = class',
    '    constructor Create;',
    '  end;',
    '  TBird = class',
    '    class constructor InitBird;',
    '  end;',
    ''],[
    'constructor TObject.Create; begin end;',
    'class constructor TBird.InitBird;',
    'begin',
    '  exit;',
    'end;',
    '']);
  AddUnit('src/unit2.pp',[
    'uses unit1;',
    'procedure DoIt;',
    ''],[
    'procedure DoIt;',
    'begin',
    '  TBird.Create;',
    'end;',
    '']);
  AddFile('test1.pas',[
    'uses unit2;',
    'begin',
    '  DoIt;',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_ClassInterface;
begin
  AddUnit('src/system.pp',[
    '{$interfaces corba}',
    'type',
    '  integer = longint;',
    '  IUnknown = interface',
    '  end;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddUnit('src/unit1.pp',[
    'type',
    '  IIntf = interface',
    '    function GetItems(Index: longint): longint;',
    '    procedure SetItems(Index: longint; Value: longint);',
    '    property Items[Index: longint]: longint read GetItems write SetItems; default;',
    '  end;',
    ''],[
    '']);
  AddUnit('src/unit2.pp',[
    'uses unit1;',
    'type',
    '  IAlias = IIntf;',
    '  TObject = class end;',
    '  TBird = class(IIntf)',
    '  strict private',
    '    function IIntf.GetItems = FetchItems;',
    '    function FetchItems(Index: longint): longint; virtual; abstract;',
    '    procedure SetItems(Index: longint; Value: longint); virtual; abstract;',
    '  end;',
    ''],[
    '']);
  AddUnit('src/unit3.pp',[
    'uses unit2;',
    'type',
    '  TEagle = class(TBird)',
    '    function FetchItems(Index: longint): longint; override;',
    '    procedure SetItems(Index: longint; Value: longint); override;',
    '  end;',
    '  TFlying = class(IAlias)',
    '  strict private',
    '    FEagle: TEagle;',
    '    property Eagle: TEagle read FEagle implements IAlias;',
    '  public',
    '    constructor Create;',
    '  end;',
    ''],[
    'function TEagle.FetchItems(Index: longint): longint; begin end;',
    'procedure TEagle.SetItems(Index: longint; Value: longint); begin end;',
    'constructor TFlying.Create;',
    'begin',
    '  FEagle:=nil;',
    'end;',
    '']);
  AddFile('test1.pas',[
    'uses unit2, unit3;',
    'type IAlias2 = IAlias;',
    'var',
    '  f: TFlying;',
    '  i: IAlias2;',
    'begin',
    '  f:=TFlying.Create;',
    '  i:=f;',
    '  i[2]:=i[3];',
    'end.']);
  CheckPrecompile('test1.pas','src');
end;

procedure TTestCLI_Precompile.TestPCU_EnumNames;
var
  SharedParams: TStringList;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;',
    '  TObject = class end;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddUnit('src/unit1.pp',
  ['type',
  '  TEnum = (red,green,blue);',
   '  TBird = class ',
   '  private',
   '    Color: TEnum;',
   '  public',
   '    procedure Fly;',
   '    procedure Run;',
   '  end;',
   ''],
  ['procedure TBird.Fly;',
   'begin',
   '  Color:=blue;',
   'end;',
   'procedure TBird.Run;',
   'begin',
   '  Color:=green;',
   'end;']);
  AddFile('test1.pas',[
    'uses unit1;',
    'var b: TBird;',
    'begin',
    '  b.Fly();',
    '  b.Run();',
    'end.']);
  SharedParams:=TStringList.Create;
  SharedParams.Add('-OoEnumNumbers-');
  CheckPrecompile('test1.pas','src',SharedParams);
end;

procedure TTestCLI_Precompile.TestPCU_Namespace;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddUnit('src/Web.Unit1.pp',[
    'var i: integer;',
    ''],[
    '']);
  AddUnit('src/Unit2.pp',[
    'uses WEB.uNit1;',
    'procedure DoIt;',
    ''],[
    'procedure DoIt;',
    'begin',
    '  writeln(i);',
    'end;',
    '']);
  AddFile('test1.pas',[
    'uses unIT2;',
    'begin',
    '  DoIt;',
    'end.']);
  CheckPrecompile('test1.pas','src');
  AssertFileExists(UnitOutputDir+'/Unit2.'+PCUFormat.Ext);
  AssertFileExists(UnitOutputDir+'/Web.Unit1.'+PCUFormat.Ext);
end;

procedure TTestCLI_Precompile.TestPCU_CheckVersionMain;
var
  aFile: TCLIFile;
  s, JSFilename, ExpectedSrc: string;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;'],
    ['']);
  AddFile('test1.pas',[
    'begin',
    'end.']);
  CheckPrecompile('test1.pas','src',LinesToList(['-JoCheckVersion=Main','-Jm-','-Jc-']));
  JSFilename:=GetJSFilename('test1.js');
  aFile:=FindFile(JSFilename);
  AssertNotNull('File not found '+JSFilename,aFile);
  ExpectedSrc:=LinesToStr([
    UTF8BOM+'rtl.module("program",["system"],function () {',
    '  "use strict";',
    '  var $mod = this;',
    '  $mod.$main = function () {',
    '    rtl.checkVersion('+IntToStr((VersionMajor*100+VersionMinor)*100+VersionRelease)+');',
    '  };',
    '});']);
  if not CheckSrcDiff(ExpectedSrc,aFile.Source,s) then
    Fail('TTestCLI_Precompile.TestPCU_CheckVersionMain src diff: '+s);
end;

procedure TTestCLI_Precompile.TestPCU_CheckVersionMain2;
var
  aFile: TCLIFile;
  s, JSFilename, ExpectedSrc: string;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;',
    'procedure Writeln; varargs;'],
    ['procedure Writeln; begin end;']);
  AddFile('test1.pas',[
    'begin',
    '  Writeln;',
    'end.']);
  CheckPrecompile('test1.pas','src',LinesToList(['-JoCheckVersion=Main','-Jm-','-Jc-']));
  JSFilename:=GetJSFilename('test1.js');
  aFile:=FindFile(JSFilename);
  AssertNotNull('File not found '+JSFilename,aFile);
  ExpectedSrc:=LinesToStr([
    UTF8BOM+'rtl.module("program",["system"],function () {',
    '  "use strict";',
    '  var $mod = this;',
    '  $mod.$main = function () {',
    '    rtl.checkVersion('+IntToStr((VersionMajor*100+VersionMinor)*100+VersionRelease)+');',
    '    pas.system.Writeln();',
    '  };',
    '});']);
  if not CheckSrcDiff(ExpectedSrc,aFile.Source,s) then
    Fail('TTestCLI_Precompile.TestPCU_CheckVersionMain src diff: '+s);
end;

procedure TTestCLI_Precompile.TestPCU_CheckVersionSystem;
var
  aFile: TCLIFile;
  s, JSFilename, ExpectedSrc, VerStr: string;
begin
  AddUnit('src/system.pp',[
    'type integer = longint;'],
    ['']);
  AddFile('test1.pas',[
    'begin',
    'end.']);
  CheckPrecompile('test1.pas','src',LinesToList(['-JoCheckVersion=system','-Jm-','-Jc-']));
  JSFilename:=GetJSFilename('system.js');
  aFile:=FindFile(JSFilename);
  AssertNotNull('File not found '+JSFilename,aFile);
  writeln('TTestCLI_Precompile.TestPCU_CheckVersionMain ',aFile.Source);
  VerStr:=IntToStr((VersionMajor*100+VersionMinor)*100+VersionRelease);
  ExpectedSrc:=LinesToStr([
    UTF8BOM+'rtl.module("system",[],function () {',
    '  "use strict";',
    '  rtl.checkVersion('+VerStr+');',
    '  var $mod = this;',
    '});']);
  if not CheckSrcDiff(ExpectedSrc,aFile.Source,s) then
    Fail('TTestCLI_Precompile.TestPCU_CheckVersionMain src diff: '+s);
end;

Initialization
  RegisterTests([TTestCLI_Precompile]);
  RegisterPCUFormat;
end.

