unit TCGenerics;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry,
  TCModules, FPPas2Js;

type

  { TTestGenerics }

  TTestGenerics = class(TCustomTestModule)
  Published
    // generic record
    Procedure TestGen_RecordEmpty;

    // generic class
    Procedure TestGen_ClassEmpty;
    Procedure TestGen_Class_EmptyMethod;
    Procedure TestGen_Class_TList;
    Procedure TestGen_ClassAncestor;
    Procedure TestGen_TypeInfo;
    // ToDo: TBird, TBird<T>, TBird<S,T>
    // ToDo: rename local const T

    // generic external class
    procedure TestGen_ExtClass_Array;

    // statements
    Procedure TestGen_InlineSpec_Constructor;
    Procedure TestGen_CallUnitImplProc;
    Procedure TestGen_IntAssignTemplVar;
    // ToDo: TBird<word>(o).field:=3;

    // generic helper
    // ToDo: helper for gen array: TArray<word>.Fly(aword);

    // generic functions
    // ToDo: Fly<word>(3);
    // ToDo: TestGenProc_ProcT
    // ToDo: inference Fly(3);
  end;

implementation

{ TTestGenerics }

procedure TTestGenerics.TestGen_RecordEmpty;
begin
  StartProgram(false);
  Add([
  'type',
  '  generic TRecA<T> = record',
  '  end;',
  'var a,b: specialize TRecA<word>;',
  'begin',
  '  if a=b then ;']);
  ConvertProgram;
  CheckSource('TestGen_RecordEmpty',
    LinesToStr([ // statements
    'rtl.recNewT($mod, "TRecA$G1", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '});',
    'this.a = $mod.TRecA$G1.$new();',
    'this.b = $mod.TRecA$G1.$new();',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.a.$eq($mod.b)) ;'
    ]));
end;

procedure TTestGenerics.TestGen_ClassEmpty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '  end;',
  'var a,b: specialize TBird<word>;',
  'begin',
  '  if a=b then ;']);
  ConvertProgram;
  CheckSource('TestGen_ClassEmpty',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass($mod, "TBird$G1", $mod.TObject, function () {',
    '});',
    'this.a = null;',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.a === $mod.b) ;'
    ]));
end;

procedure TTestGenerics.TestGen_Class_EmptyMethod;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '    function Fly(w: T): T;',
  '  end;',
  'function TBird.Fly(w: T): T;',
  'begin',
  'end;',
  'var a: specialize TBird<word>;',
  'begin',
  '  if a.Fly(3)=4 then ;']);
  ConvertProgram;
  CheckSource('TestGen_Class_EmptyMethod',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass($mod, "TBird$G1", $mod.TObject, function () {',
    '  this.Fly = function (w) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.a = null;',
    '']),
    LinesToStr([ // $mod.$main
    '  if ($mod.a.Fly(3) === 4) ;'
    ]));
end;

procedure TTestGenerics.TestGen_Class_TList;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TObject = class end;',
  '  generic TList<T> = class',
  '  strict private',
  '    FItems: array of T;',
  '    function GetItems(Index: longint): T;',
  '    procedure SetItems(Index: longint; Value: T);',
  '  public',
  '    procedure Alter(w: T);',
  '    property Items[Index: longint]: T read GetItems write SetItems; default;',
  '  end;',
  '  TWordList = specialize TList<word>;',
  'function TList.GetItems(Index: longint): T;',
  'begin',
  '  Result:=FItems[Index];',
  'end;',
  'procedure TList.SetItems(Index: longint; Value: T);',
  'begin',
  '  FItems[Index]:=Value;',
  'end;',
  'procedure TList.Alter(w: T);',
  'begin',
  '  SetLength(FItems,length(FItems)+1);',
  '  Insert(w,FItems,2);',
  '  Delete(FItems,2,3);',
  'end;',
  'var l: TWordList;',
  '  w: word;',
  'begin',
  '  l[1]:=w;',
  '  w:=l[2];',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_TList',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass($mod, "TList$G1", $mod.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FItems = [];',
    '  };',
    '  this.$final = function () {',
    '    this.FItems = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.GetItems = function (Index) {',
    '    var Result = 0;',
    '    Result = this.FItems[Index];',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '    this.FItems[Index] = Value;',
    '  };',
    '  this.Alter = function (w) {',
    '    this.FItems = rtl.arraySetLength(this.FItems, 0, rtl.length(this.FItems) + 1);',
    '    this.FItems.splice(2, 0, w);',
    '    this.FItems.splice(2, 3);',
    '  };',
    '});',
    'this.l = null;',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.l.SetItems(1, $mod.w);',
    '$mod.w = $mod.l.GetItems(2);',
    '']));
end;

procedure TTestGenerics.TestGen_ClassAncestor;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '  end;',
  '  generic TEagle<T> = class(specialize TBird<T>)',
  '  end;',
  'var a: specialize TEagle<word>;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ClassAncestor',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass($mod, "TBird$G2", $mod.TObject, function () {',
    '});',
    'rtl.createClass($mod, "TEagle$G1", $mod.TBird$G2, function () {',
    '});',
    'this.a = null;',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_TypeInfo;
begin
  Converter.Options:=Converter.Options-[coNoTypeInfo];
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '  published',
  '    m: T;',
  '  end;',
  '  TEagle = specialize TBird<word>;',
  'var',
  '  b: specialize TBird<word>;',
  '  p: pointer;',
  'begin',
  '  p:=typeinfo(TEagle);',
  '  p:=typeinfo(b);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_TypeInfo',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass($mod, "TBird$G1", $mod.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.m = 0;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("m", rtl.word);',
    '});',
    'this.b = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TBird$G1"];',
    '$mod.p = $mod.b.$rtti;',
    '']));
end;

procedure TTestGenerics.TestGen_ExtClass_Array;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  '{$ModeSwitch externalclass}',
  'type',
  '  NativeInt = longint;',
  '  TJSGenArray<T> = Class external name ''Array''',
  '  private',
  '    function GetElements(Index: NativeInt): T; external name ''[]'';',
  '    procedure SetElements(Index: NativeInt; const AValue: T); external name ''[]'';',
  '  public',
  '    type TSelfType = TJSGenArray<T>;',
  '  public',
  '    FLength : NativeInt; external name ''length'';',
  '    constructor new; overload;',
  '    constructor new(aLength : NativeInt); overload;',
  '    class function _of() : TSelfType; varargs; external name ''of'';',
  '    function fill(aValue : T) : TSelfType; overload;',
  '    function fill(aValue : T; aStartIndex : NativeInt) : TSelfType; overload;',
  '    function fill(aValue : T; aStartIndex,aEndIndex : NativeInt) : TSelfType; overload;',
  '    property Length : NativeInt Read FLength Write FLength;',
  '    property Elements[Index: NativeInt]: T read GetElements write SetElements; default;',
  '  end;',
  '  TJSWordArray = TJSGenArray<word>;',
  'var',
  '  wa: TJSWordArray;',
  '  w: word;',
  'begin',
  '  wa:=TJSWordArray.new;',
  '  wa:=TJSWordArray.new(3);',
  '  wa:=TJSWordArray._of(4,5);',
  '  wa:=wa.fill(7);',
  '  wa:=wa.fill(7,8,9);',
  '  w:=wa.length;',
  '  wa.length:=10;',
  '  wa[11]:=w;',
  '  w:=wa[12];',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ExtClass_Array',
    LinesToStr([ // statements
    'this.wa = null;',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.wa = new Array();',
    '$mod.wa = new Array(3);',
    '$mod.wa = Array.of(4, 5);',
    '$mod.wa = $mod.wa.fill(7);',
    '$mod.wa = $mod.wa.fill(7, 8, 9);',
    '$mod.w = $mod.wa.length;',
    '$mod.wa.length = 10;',
    '$mod.wa[11] = $mod.w;',
    '$mod.w = $mod.wa[12];',
    '']));
end;

procedure TTestGenerics.TestGen_InlineSpec_Constructor;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TObject = class',
  '  public',
  '    constructor Create;',
  '  end;',
  '  generic TBird<T> = class',
  '  end;',
  'constructor TObject.Create; begin end;',
  'var b: specialize TBird<word>;',
  'begin',
  '  b:=specialize TBird<word>.Create;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_InlineSpec_Constructor',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass($mod, "TBird$G1", $mod.TObject, function () {',
    '});',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b = $mod.TBird$G1.$create("Create");',
    '']));
end;

procedure TTestGenerics.TestGen_CallUnitImplProc;
begin
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
    'type',
    '  generic TBird<T> = class',
    '    procedure Fly;',
    '  end;',
    'var b: specialize TBird<boolean>;',
    '']),
  LinesToStr([
    'procedure DoIt;',
    'var b: specialize TBird<word>;',
    'begin',
    '  b:=specialize TBird<word>.Create;',
    '  b.Fly;',
    'end;',
    'procedure TBird.Fly;',
    'begin',
    '  DoIt;',
    'end;',
    '']));
  StartProgram(true,[supTObject]);
  Add('uses UnitA;');
  Add('begin');
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  rtl.createClass($mod, "TBird$G1", pas.system.TObject, function () {',
    '    this.Fly = function () {',
    '      $impl.DoIt();',
    '    };',
    '  });',
    '  rtl.createClass($mod, "TBird$G2", pas.system.TObject, function () {',
    '    this.Fly = function () {',
    '      $impl.DoIt();',
    '    };',
    '  });',
    '  this.b = null;',
    '}, null, function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  $impl.DoIt = function () {',
    '    var b = null;',
    '    b = $mod.TBird$G2.$create("Create");',
    '    b.Fly();',
    '  };',
    '});',
    '']));
end;

procedure TTestGenerics.TestGen_IntAssignTemplVar;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '    m: T;',
  '    procedure Fly;',
  '  end;',
  'var b: specialize TBird<word>;',
  'procedure TBird.Fly;',
  'var i: nativeint;',
  'begin',
  '  i:=m;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_IntAssignTemplVar',
    LinesToStr([ // statements
    'rtl.createClass($mod, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass($mod, "TBird$G1", $mod.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.m = 0;',
    '  };',
    '  this.Fly = function () {',
    '    var i = 0;',
    '    i = this.m;',
    '  };',
    '});',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

Initialization
  RegisterTests([TTestGenerics]);
end.

