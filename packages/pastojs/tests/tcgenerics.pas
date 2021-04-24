unit TCGenerics;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry,
  TCModules, FPPas2Js, PScanner, PasResolveEval;

type

  { TTestGenerics }

  TTestGenerics = class(TCustomTestModule)
  Published
    // generic record
    Procedure TestGen_RecordEmpty;
    Procedure TestGen_Record_ClassProc;
    Procedure TestGen_Record_ClassVarRecord_Program;
    Procedure TestGen_Record_ClassVarRecord_UnitImpl;
    Procedure TestGen_Record_RTTI_UnitImpl;
    procedure TestGen_Record_Delay_UsedByImplUses;
    // ToDo: type alias type as parameter, TBird = type word;

    // generic class
    Procedure TestGen_ClassEmpty;
    Procedure TestGen_Class_EmptyMethod;
    Procedure TestGen_Class_TList;
    Procedure TestGen_Class_TCustomList; // ToDo: with Self do Result:=Method()
    Procedure TestGen_ClassAncestor;
    Procedure TestGen_Class_TypeInfo;
    Procedure TestGen_Class_TypeOverload; // ToDo TBird, TBird<T>, TBird<S,T>
    Procedure TestGen_Class_ClassProperty;
    Procedure TestGen_Class_ClassProc;
    //Procedure TestGen_Record_ReferGenClass_DelphiFail; TBird<T> = class x:TBird; end;
    Procedure TestGen_Class_ClassConstructor;
    Procedure TestGen_Class_TypeCastSpecializesWarn;
    Procedure TestGen_Class_TypeCastSpecializesJSValueNoWarn;
    procedure TestGen_Class_OverloadsInUnit;
    procedure TestGen_ClassForward_CircleRTTI;
    procedure TestGen_Class_Nested_RTTI;
    Procedure TestGen_Class_ClassVarRecord_UnitImpl;

    // generic external class
    procedure TestGen_ExtClass_VarArgsOfType;
    procedure TestGen_ExtClass_Array;
    procedure TestGen_ExtClass_GenJSValueAssign;
    procedure TestGen_ExtClass_AliasMemberType;
    Procedure TestGen_ExtClass_RTTI;
    procedure TestGen_ExtClass_UnitImplRec;

    // class interfaces
    procedure TestGen_ClassInterface_Corba;
    procedure TestGen_ClassInterface_InterfacedObject;
    procedure TestGen_ClassInterface_COM_RTTI;
    procedure TestGen_ClassInterface_Helper;
    procedure TestGen_ClassInterface_DelayedInitSpec;

    // statements
    Procedure TestGen_InlineSpec_Constructor;
    Procedure TestGen_CallUnitImplProc;
    Procedure TestGen_IntAssignTemplVar;
    Procedure TestGen_TypeCastDotField;
    Procedure TestGen_Except;

    // generic helper
    procedure TestGen_HelperForArray;

    // generic functions
    procedure TestGenProc_Function_ObjFPC;
    procedure TestGenProc_Function_Delphi;
    procedure TestGenProc_Overload;
    procedure TestGenProc_Forward;
    procedure TestGenProc_Infer_OverloadForward;
    procedure TestGenProc_TypeInfo;
    procedure TestGenProc_Infer_Widen;
    procedure TestGenProc_Infer_PassAsArg;
    procedure TestGenProc_AnonymousProc;
    // ToDo: FuncName:= instead of Result:=

    // generic methods
    procedure TestGenMethod_ImplicitSpec_ObjFPC;
    procedure TestGenMethod_Delphi;

    // generic array
    procedure TestGen_Array_OtherUnit;
    procedure TestGen_ArrayOfUnitImplRec;
    procedure TestGen_Array_TypecastJSValueResultToArg;

    // generic procedure type
    procedure TestGen_ProcType_ProcLocal;
    procedure TestGen_ProcType_Local_RTTI_Fail;
    procedure TestGen_ProcType_ParamUnitImpl;
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
    'rtl.recNewT(this, "TRecA$G1", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '});',
    'this.a = this.TRecA$G1.$new();',
    'this.b = this.TRecA$G1.$new();',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.a.$eq($mod.b)) ;'
    ]));
end;

procedure TTestGenerics.TestGen_Record_ClassProc;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  generic TPoint<T> = record',
  '    class var x: T;',
  '    class procedure Fly; static;',
  '  end;',
  'class procedure Tpoint.Fly;',
  'begin',
  '  x:=x+3;',
  '  tpoint.x:=tpoint.x+4;',
  '  Fly;',
  '  tpoint.Fly;',
  'end;',
  'var p: specialize TPoint<word>;',
  'begin',
  '  p.x:=p.x+10;',
  '  p.Fly;',
  '  p.Fly();',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Record_ClassProc',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint$G1", function () {',
    '  this.x = 0;',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.Fly = function () {',
    '    $mod.TPoint$G1.x = $mod.TPoint$G1.x + 3;',
    '    $mod.TPoint$G1.x = $mod.TPoint$G1.x + 4;',
    '    $mod.TPoint$G1.Fly();',
    '    $mod.TPoint$G1.Fly();',
    '  };',
    '}, true);',
    'this.p = this.TPoint$G1.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TPoint$G1.x = $mod.p.x + 10;',
    '$mod.TPoint$G1.Fly();',
    '$mod.TPoint$G1.Fly();',
    '']));
end;

procedure TTestGenerics.TestGen_Record_ClassVarRecord_Program;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  generic TAnt<T> = record',
  '    class var x: T;',
  '  end;',
  '  TBird = record',
  '    b: word;',
  '  end;',
  'var f: specialize TAnt<TBird>;',
  'begin',
  '  f.x.b:=f.x.b+10;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Record_ClassVarRecord_Program',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TBird", function () {',
    '  this.b = 0;',
    '  this.$eq = function (b) {',
    '    return this.b === b.b;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.b = s.b;',
    '    return this;',
    '  };',
    '});',
    'rtl.recNewT(this, "TAnt$G1", function () {',
    '  this.x = $mod.TBird.$new();',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '}, true);',
    'this.f = this.TAnt$G1.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.f.x.b = $mod.f.x.b + 10;',
    '']));
end;

procedure TTestGenerics.TestGen_Record_ClassVarRecord_UnitImpl;
begin
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  generic TAnt<T> = record',
  '    class var x: T;',
  '    class var a: array[1..2] of T;',
  '  end;',
  '']),
  LinesToStr([
  'type',
  '  TBird = record',
  '    b: word;',
  '  end;',
  'var f: specialize TAnt<TBird>;',
  'begin',
  '  f.x.b:=f.x.b+10;',
  '']));
  Add([
  'uses UnitA;',
  'begin',
  'end.']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  rtl.recNewT(this, "TAnt$G1", function () {',
    '    this.$initSpec = function () {',
    '      this.x = $impl.TBird.$new();',
    '      this.a = rtl.arraySetLength(null, $impl.TBird, 2);',
    '    };',
    '    this.a$a$clone = function (a) {',
    '      var r = [];',
    '      for (var i = 0; i < 2; i++) r.push($impl.TBird.$clone(a[i]));',
    '      return r;',
    '    };',
    '    this.$eq = function (b) {',
    '      return true;',
    '    };',
    '    this.$assign = function (s) {',
    '    return this;',
    '    };',
    '  }, true);',
    '  $mod.$implcode = function () {',
    '    rtl.recNewT($impl, "TBird", function () {',
    '      this.b = 0;',
    '      this.$eq = function (b) {',
    '        return this.b === b.b;',
    '      };',
    '      this.$assign = function (s) {',
    '        this.b = s.b;',
    '        return this;',
    '      };',
    '    });',
    '    $impl.f = $mod.TAnt$G1.$new();',
    '  };',
    '  $mod.$init = function () {',
    '    $impl.f.x.b = $impl.f.x.b + 10;',
    '  };',
    '}, []);']));
  CheckSource('TestGen_Record_ClassVarRecord_UnitImpl',
    LinesToStr([ // statements
    '$mod.$implcode = function () {',
    '  pas.UnitA.TAnt$G1.$initSpec();',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_Record_RTTI_UnitImpl;
begin
  WithTypeInfo:=true;
  StartUnit(true);
  Add([
  'interface',
  '{$modeswitch AdvancedRecords}',
  'type',
  '  generic TAnt<T> = record',
  '    class var x: T;',
  //'    class var a,b: array of T;',
  '  end;',
  'implementation',
  'type',
  '  TBird = record',
  '    b: word;',
  '  end;',
  'var f: specialize TAnt<TBird>;',
  '  p: pointer;',
  'begin',
  '  p:=typeinfo(f);',
  '']);
  ConvertUnit;
  CheckSource('TestGen_Record_RTTI_UnitImpl',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'rtl.recNewT(this, "TAnt$G1", function () {',
    '  var $r = $mod.$rtti.$Record("TAnt<Test1.TBird>", {});',
    '  this.$initSpec = function () {',
    '    this.x = $impl.TBird.$new();',
    '    $r.addField("x", $mod.$rtti["TBird"]);',
    '  };',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '}, true);',
    '']),
    LinesToStr([ // $mod.$init
    '$impl.p = $mod.$rtti["TAnt<Test1.TBird>"];',
    '']),
    LinesToStr([ // statements
    'rtl.recNewT($impl, "TBird", function () {',
    '  this.b = 0;',
    '  this.$eq = function (b) {',
    '    return this.b === b.b;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.b = s.b;',
    '    return this;',
    '  };',
    '  var $r = $mod.$rtti.$Record("TBird", {});',
    '  $r.addField("b", rtl.word);',
    '});',
    '$impl.f = $mod.TAnt$G1.$new();',
    '$impl.p = null;',
    '']));
end;

procedure TTestGenerics.TestGen_Record_Delay_UsedByImplUses;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
    '{$modeswitch AdvancedRecords}',
    'type',
    '  generic TBird<T> = record',
    '    class var a: T;',
    '  end;',
    '']),
  LinesToStr([
    '']));
  AddModuleWithIntfImplSrc('UnitB.pas',
  LinesToStr([
    'procedure Fly;',
    '']),
  LinesToStr([
    'uses UnitA;',
    'type',
    '  TFox = record',
    '    B: word;',
    '  end;',
    'procedure Fly;',
    'var Bird: specialize TBird<TFox>;',
    'begin',
    '  if typeinfo(Bird)<>nil then ;',
    '  Bird.a:=Bird.a;',
    'end;',
    '']));
  Add([
  'uses UnitB;',
  'begin',
  '  Fly;']);
  ConvertProgram;
  CheckSource('TestGen_Record_Delay_UsedByImplUses',
    LinesToStr([ // statements
    '$mod.$implcode = function () {',
    '  pas.UnitA.TBird$G1.$initSpec();',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    'pas.UnitB.Fly();'
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
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '}, "TBird<System.Word>");',
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
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.Fly = function (w) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '}, "TBird<System.Word>");',
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
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TList$G1", this.TObject, function () {',
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
    '    this.FItems = rtl.arrayInsert(w, this.FItems, 2);',
    '    this.FItems.splice(2, 3);',
    '  };',
    '}, "TList<System.Word>");',
    'this.l = null;',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.l.SetItems(1, $mod.w);',
    '$mod.w = $mod.l.GetItems(2);',
    '']));
end;

procedure TTestGenerics.TestGen_Class_TCustomList;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TCustomList<T> = class',
  '  public',
  '    function PrepareAddingItem: word; virtual;',
  '  end;',
  '  TList<T> = class(TCustomList<T>)',
  '  public',
  '    function Add: word;',
  '  end;',
  '  TWordList = TList<word>;',
  'function TCustomList<T>.PrepareAddingItem: word;',
  'begin',
  'end;',
  'function TList<T>.Add: word;',
  'begin',
  '  Result:=PrepareAddingItem;',
  '  Result:=Self.PrepareAddingItem;',
  //'  with Self do Result:=PrepareAddingItem;',
  'end;',
  'var l: TWordList;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_TCustomList',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TCustomList$G2", this.TObject, function () {',
    '  this.PrepareAddingItem = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '}, "TCustomList<System.Word>");',
    'rtl.createClass(this, "TList$G1", this.TCustomList$G2, function () {',
    '  this.Add = function () {',
    '    var Result = 0;',
    '    Result = this.PrepareAddingItem();',
    '    Result = this.PrepareAddingItem();',
    '    return Result;',
    '  };',
    '}, "TList<System.Word>");',
    'this.l = null;',
    '']),
    LinesToStr([ // $mod.$main
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
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G2", this.TObject, function () {',
    '}, "TBird<System.Word>");',
    'rtl.createClass(this, "TEagle$G1", this.TBird$G2, function () {',
    '}, "TEagle<System.Word>");',
    'this.a = null;',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_Class_TypeInfo;
begin
  WithTypeInfo:=true;
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
  CheckSource('TestGen_Class_TypeInfo',
    LinesToStr([ // statements
    '$mod.$rtti.$Class("TBird<System.Word>");',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.m = 0;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("m", rtl.word);',
    '}, "TBird<System.Word>");',
    'this.b = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TBird<System.Word>"];',
    '$mod.p = $mod.b.$rtti;',
    '']));
end;

procedure TTestGenerics.TestGen_Class_TypeOverload;
begin
  exit;// ToDo

  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TBird = word;',
  '  TBird<T> = class',
  '    m: T;',
  '  end;',
  '  TEagle = TBird<word>;',
  'var',
  '  b: TBird<word>;',
  '  e: TEagle;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_TypeOverload',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_Class_ClassProperty;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TBird<T> = class',
  '  private',
  '    class var fSize: T;',
  '  public',
  '    class property Size: T read fSize write fSize;',
  '  end;',
  '  TEagle = TBird<word>;',
  'begin',
  '  TBird<word>.Size:=3+TBird<word>.Size;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_ClassProperty',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.fSize = 0;',
    '}, "TBird<System.Word>");',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TBird$G1.fSize = 3 + $mod.TBird$G1.fSize;',
    '']));
end;

procedure TTestGenerics.TestGen_Class_ClassProc;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TPoint<T> = class',
  '    class var x: T;',
  '    class procedure Fly; static;',
  '    class procedure Run;',
  '  end;',
  'class procedure Tpoint.Fly;',
  'begin',
  '  x:=x+3;',
  '  tpoint.x:=tpoint.x+4;',
  '  Fly;',
  '  tpoint.Fly;',
  '  Run;',
  '  tpoint.Run;',
  'end;',
  'class procedure TPoint.Run;',
  'begin',
  '  x:=x+5;',
  '  tpoint.x:=tpoint.x+6;',
  '  Fly;',
  '  tpoint.Fly;',
  '  Run;',
  '  tpoint.Run;',
  'end;',
  'var p: specialize TPoint<word>;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_ClassProc',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TPoint$G1", this.TObject, function () {',
    '  this.x = 0;',
    '  this.Fly = function () {',
    '    $mod.TPoint$G1.x = $mod.TPoint$G1.x + 3;',
    '    $mod.TPoint$G1.x = $mod.TPoint$G1.x + 4;',
    '    $mod.TPoint$G1.Fly();',
    '    $mod.TPoint$G1.Fly();',
    '    $mod.TPoint$G1.Run();',
    '    $mod.TPoint$G1.Run();',
    '  };',
    '  this.Run = function () {',
    '    $mod.TPoint$G1.x = this.x + 5;',
    '    $mod.TPoint$G1.x = $mod.TPoint$G1.x + 6;',
    '    this.Fly();',
    '    this.Fly();',
    '    this.Run();',
    '    $mod.TPoint$G1.Run();',
    '  };',
    '}, "TPoint<System.Word>");',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_Class_ClassConstructor;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TPoint<T> = class',
  '    class var x: T;',
  '    class procedure Fly; static;',
  '    class constructor Init;',
  '  end;',
  'var count: word;',
  'class procedure Tpoint.Fly;',
  'begin',
  'end;',
  'class constructor tpoint.init;',
  'begin',
  '  count:=count+1;',
  '  x:=3;',
  '  tpoint.x:=4;',
  '  fly;',
  '  tpoint.fly;',
  'end;',
  'var',
  '  r: specialize TPoint<word>;',
  '  s: specialize TPoint<smallint>;',
  'begin',
  '  r.x:=10;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_ClassConstructor',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.count = 0;',
    'rtl.createClass(this, "TPoint$G1", this.TObject, function () {',
    '  this.x = 0;',
    '  this.Fly = function () {',
    '  };',
    '}, "TPoint<System.Word>");',
    'this.r = null;',
    'rtl.createClass(this, "TPoint$G2", this.TObject, function () {',
    '  this.x = 0;',
    '  this.Fly = function () {',
    '  };',
    '}, "TPoint<System.SmallInt>");',
    'this.s = null;',
    '']),
    LinesToStr([ // $mod.$main
    '(function () {',
    '  $mod.count = $mod.count + 1;',
    '  $mod.TPoint$G1.x = 3;',
    '  $mod.TPoint$G1.x = 4;',
    '  $mod.TPoint$G1.Fly();',
    '  $mod.TPoint$G1.Fly();',
    '})();',
    '(function () {',
    '  $mod.count = $mod.count + 1;',
    '  $mod.TPoint$G2.x = 3;',
    '  $mod.TPoint$G2.x = 4;',
    '  $mod.TPoint$G2.Fly();',
    '  $mod.TPoint$G2.Fly();',
    '})();',
    '$mod.TPoint$G1.x = 10;',
    '']));
end;

procedure TTestGenerics.TestGen_Class_TypeCastSpecializesWarn;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TBird<T> = class F: T; end;',
  '  TBirdWord = TBird<Word>;',
  '  TBirdChar = TBird<Char>;',
  'var',
  '  w: TBirdWord;',
  '  c: TBirdChar;',
  'begin',
  '  w:=TBirdWord(c);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_TypeCastSpecializesWarn',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.F = 0;',
    '  };',
    '}, "TBird<System.Word>");',
    'rtl.createClass(this, "TBird$G2", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.F = "";',
    '  };',
    '}, "TBird<System.Char>");',
    'this.w = null;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.w = $mod.c;',
    '']));
  CheckHint(mtWarning,nClassTypesAreNotRelatedXY,'Class types "TBird<System.Char>" and "TBird<System.Word>" are not related');
  CheckResolverUnexpectedHints();
end;

procedure TTestGenerics.TestGen_Class_TypeCastSpecializesJSValueNoWarn;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TBird<T> = class F: T; end;',
  '  TBirdWord = TBird<Word>;',
  '  TBirdAny = TBird<JSValue>;',
  'var',
  '  w: TBirdWord;',
  '  a: TBirdAny;',
  'begin',
  '  w:=TBirdWord(a);',
  '  a:=TBirdAny(w);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Class_TypeCastSpecializesJSValueNoWarn',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.F = 0;',
    '  };',
    '}, "TBird<System.Word>");',
    'rtl.createClass(this, "TBird$G2", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.F = undefined;',
    '  };',
    '}, "TBird<System.JSValue>");',
    'this.w = null;',
    'this.a = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.w = $mod.a;',
    '$mod.a = $mod.w;',
    '']));
  CheckResolverUnexpectedHints();
end;

procedure TTestGenerics.TestGen_Class_OverloadsInUnit;
begin
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
    'type',
    '  generic TBird<T> = class',
    '  const c = 13;',
    '    constructor Create(w: T);',
    '    constructor Create(b: boolean);',
    '  end;',
    '']),
  LinesToStr([
    'constructor TBird.Create(w: T);',
    'const c = 14;',
    'begin',
    'end;',
    'constructor TBird.Create(b: boolean);',
    'const c = 15;',
    'begin',
    'end;',
    '']));
  Add([
  'uses UnitA;',
  'type',
  '  TWordBird = specialize TBird<word>;',
  '  TDoubleBird = specialize TBird<double>;',
  'var',
  '  wb: TWordBird;',
  '  db: TDoubleBird;',
  'begin',
  '  wb:=TWordBird.Create(3);',
  '  wb:=TWordBird.Create(true);',
  '  db:=TDoubleBird.Create(1.3);',
  '  db:=TDoubleBird.Create(true);',
  '']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  rtl.createClass(this, "TBird$G1", pas.system.TObject, function () {',
    '    this.c = 13;',
    '    var c$1 = 14;',
    '    this.Create$1 = function (w) {',
    '      return this;',
    '    };',
    '    var c$2 = 15;',
    '    this.Create$2 = function (b) {',
    '      return this;',
    '    };',
    '  }, "TBird<System.Word>");',
    '  rtl.createClass(this, "TBird$G2", pas.system.TObject, function () {',
    '    this.c = 13;',
    '    var c$1 = 14;',
    '    this.Create$1 = function (w) {',
    '      return this;',
    '    };',
    '    var c$2 = 15;',
    '    this.Create$2 = function (b) {',
    '      return this;',
    '    };',
    '  }, "TBird<System.Double>");',
    '});',
    '']));
  CheckSource('TestGen_Class_OverloadsInUnit',
    LinesToStr([ // statements
    'this.wb = null;',
    'this.db = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.wb = pas.UnitA.TBird$G1.$create("Create$1", [3]);',
    '$mod.wb = pas.UnitA.TBird$G1.$create("Create$2", [true]);',
    '$mod.db = pas.UnitA.TBird$G2.$create("Create$1", [1.3]);',
    '$mod.db = pas.UnitA.TBird$G2.$create("Create$2", [true]);',
    '']));
end;

procedure TTestGenerics.TestGen_ClassForward_CircleRTTI;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TObject = class end;',
  '  {$M+}',
  '  TPersistent = class end;',
  '  {$M-}',
  '  generic TAnt<T> = class;',
  '  generic TFish<U> = class(TPersistent)',
  '    private type AliasU = U;',
  '  published',
  '    a: specialize TAnt<AliasU>;',
  '  end;',
  '  generic TAnt<T> = class(TPersistent)',
  '    private type AliasT = T;',
  '  published',
  '    f: specialize TFish<AliasT>;',
  '  end;',
  'var',
  '  WordFish: specialize TFish<word>;',
  '  p: pointer;',
  'begin',
  '  p:=typeinfo(specialize TAnt<word>);',
  '  p:=typeinfo(specialize TFish<word>);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ClassForward_CircleRTTI',
    LinesToStr([ // statements
    '$mod.$rtti.$Class("TAnt<System.Word>");',
    '$mod.$rtti.$Class("TFish<System.Word>");',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TPersistent", this.TObject, function () {',
    '});',
    'rtl.createClass(this, "TAnt$G2", this.TPersistent, function () {',
    '  this.$init = function () {',
    '    $mod.TPersistent.$init.call(this);',
    '    this.f = null;',
    '  };',
    '  this.$final = function () {',
    '    this.f = undefined;',
    '    $mod.TPersistent.$final.call(this);',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("f", $mod.$rtti["TFish<System.Word>"]);',
    '}, "TAnt<System.Word>");',
    'rtl.createClass(this, "TFish$G2", this.TPersistent, function () {',
    '  this.$init = function () {',
    '    $mod.TPersistent.$init.call(this);',
    '    this.a = null;',
    '  };',
    '  this.$final = function () {',
    '    this.a = undefined;',
    '    $mod.TPersistent.$final.call(this);',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("a", $mod.$rtti["TAnt<System.Word>"]);',
    '}, "TFish<System.Word>");',
    'this.WordFish = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TAnt<System.Word>"];',
    '$mod.p = $mod.$rtti["TFish<System.Word>"];',
    '']));
end;

procedure TTestGenerics.TestGen_Class_Nested_RTTI;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  'type',
  '  generic TAnt<T> = class',
  '  type',
  '    TLeg = class',
  '    published',
  '      Size: T;',
  '    end;',
  '  end;',
  '  TBoolAnt = specialize TAnt<boolean>;',
  '']),
  LinesToStr([
  '']));
  Add([
  'uses UnitA;',
  'var',
  '  BoolLeg: TBoolAnt.TLeg;',
  'begin',
  '  if typeinfo(TBoolAnt.TLeg)=nil then ;',
  '']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  $mod.$rtti.$Class("TAnt<System.Boolean>");',
    '  rtl.createClass(this, "TAnt$G1", pas.system.TObject, function () {',
    '    rtl.createClass(this, "TLeg", pas.system.TObject, function () {',
    '      this.$init = function () {',
    '        pas.system.TObject.$init.call(this);',
    '        this.Size = false;',
    '      };',
    '      var $r = this.$rtti;',
    '      $r.addField("Size", rtl.boolean);',
    '    }, "TAnt<System.Boolean>.TLeg");',
    '  }, "TAnt<System.Boolean>");',
    '});']));
  CheckSource('TestGen_Class_Nested_RTTI',
    LinesToStr([ // statements
    'this.BoolLeg = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if (pas.UnitA.$rtti["TAnt<System.Boolean>.TLeg"] === null) ;',
    '']));
end;

procedure TTestGenerics.TestGen_Class_ClassVarRecord_UnitImpl;
begin
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  'type',
  '  generic TAnt<T> = class',
  '  public',
  '    class var x: T;',
  '    class var a: array[1..2] of T;',
  '  end;',
  '']),
  LinesToStr([
  'type',
  '  TBird = record',
  '    b: word;',
  '  end;',
  'var f: specialize TAnt<TBird>;',
  'begin',
  '  f.x.b:=f.x.b+10;',
  '']));
  Add([
  'uses UnitA;',
  'begin',
  'end.']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  rtl.createClass(this, "TAnt$G1", pas.system.TObject, function () {',
    '    this.$initSpec = function () {',
    '      this.x = $impl.TBird.$new();',
    '      this.a = rtl.arraySetLength(null, $impl.TBird, 2);',
    '    };',
    '    this.a$a$clone = function (a) {',
    '      var r = [];',
    '      for (var i = 0; i < 2; i++) r.push($impl.TBird.$clone(a[i]));',
    '      return r;',
    '    };',
    '  }, "TAnt<UnitA.TBird>");',
    '  $mod.$implcode = function () {',
    '    rtl.recNewT($impl, "TBird", function () {',
    '      this.b = 0;',
    '      this.$eq = function (b) {',
    '        return this.b === b.b;',
    '      };',
    '      this.$assign = function (s) {',
    '        this.b = s.b;',
    '        return this;',
    '      };',
    '    });',
    '    $impl.f = null;',
    '  };',
    '  $mod.$init = function () {',
    '    $impl.f.x.b = $impl.f.x.b + 10;',
    '  };',
    '}, []);',
    '']));
  CheckSource('TestGen_Class_ClassVarRecord_UnitImpl',
    LinesToStr([ // statements
    '$mod.$implcode = function () {',
    '  pas.UnitA.TAnt$G1.$initSpec();',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_ExtClass_VarArgsOfType;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''Object''',
  '  end;',
  '  generic TGJSSet<T> = class external name ''Set''',
  '    constructor new(aElement1: T); varargs of T; overload;',
  '    function bind(thisArg: TJSObject): T; varargs of T;',
  '  end;',
  '  TJSWordSet = specialize TGJSSet<word>;',
  'var',
  '  s: TJSWordSet;',
  '  w: word;',
  'begin',
  '  s:=TJSWordSet.new(3);',
  '  s:=TJSWordSet.new(3,5);',
  '  w:=s.bind(nil);',
  '  w:=s.bind(nil,6);',
  '  w:=s.bind(nil,7,8);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ExtClass_VarArgsOfType',
    LinesToStr([ // statements
    'this.s = null;',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.s = new Set(3);',
    '$mod.s = new Set(3, 5);',
    '$mod.w = $mod.s.bind(null);',
    '$mod.w = $mod.s.bind(null, 6);',
    '$mod.w = $mod.s.bind(null, 7, 8);',
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

procedure TTestGenerics.TestGen_ExtClass_GenJSValueAssign;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  '{$modeswitch externalclass}',
  'type',
  '  TExt<T> = class external name ''Ext''',
  '    F: T;',
  '  end;',
  '  TExtWord = TExt<Word>;',
  '  TExtAny = TExt<JSValue>;',
  'procedure Run(e: TExtAny);',
  'begin end;',
  'var',
  '  w: TExtWord;',
  '  a: TExtAny;',
  'begin',
  '  a:=w;',
  '  Run(w);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ExtClass_GenJSValueAssign',
    LinesToStr([ // statements
    'this.Run = function (e) {',
    '};',
    'this.w = null;',
    'this.a = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.a = $mod.w;',
    '$mod.Run($mod.w);',
    '']));
  CheckResolverUnexpectedHints();
end;

procedure TTestGenerics.TestGen_ExtClass_AliasMemberType;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  generic TExt<T> = class external name ''Ext''',
  '  public type TRun = reference to function(a: T): T;',
  '  end;',
  '  TExtWord = specialize TExt<word>;',
  '  TExtWordRun = TExtWord.TRun;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ExtClass_AliasMemberType',
    LinesToStr([ // statements
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_ExtClass_RTTI;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  generic TGJSSET<T> = class external name ''SET''',
  '    A: T;',
  '  end;',
  '  TJSSet = specialize TGJSSET<JSValue>;',
  '  TJSSetEventProc = reference to procedure(value : JSValue; key: NativeInt; set_: TJSSet);',
  'var p: Pointer;',
  'begin',
  '  p:=typeinfo(TJSSetEventProc);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ExtClass_RTTI',
    LinesToStr([ // statements
    'this.$rtti.$ExtClass("TGJSSET<System.JSValue>", {',
    '  jsclass: "SET"',
    '});',
    'this.$rtti.$RefToProcVar("TJSSetEventProc", {',
    '  procsig: rtl.newTIProcSig([["value", rtl.jsvalue], ["key", rtl.nativeint], ["set_", this.$rtti["TGJSSET<System.JSValue>"]]])',
    '});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TJSSetEventProc"];',
    '']));
end;

procedure TTestGenerics.TestGen_ExtClass_UnitImplRec;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  generic TAnt<T> = class external name ''SET''',
  '    x: T;',
  '  end;',
  '']),
  LinesToStr([
  'type',
  '  TBird = record',
  '    b: word;',
  '  end;',
  'var',
  '  f: specialize TAnt<TBird>;',
  'begin',
  '  f.x.b:=f.x.b+10;',
  '']));
  Add([
  'uses UnitA;',
  'begin',
  'end.']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  this.$rtti.$ExtClass("TAnt<UnitA.TBird>", {',
    '    jsclass: "SET"',
    '  });',
    '  $mod.$implcode = function () {',
    '    rtl.recNewT($impl, "TBird", function () {',
    '      this.b = 0;',
    '      this.$eq = function (b) {',
    '        return this.b === b.b;',
    '      };',
    '      this.$assign = function (s) {',
    '        this.b = s.b;',
    '        return this;',
    '      };',
    '      var $r = $mod.$rtti.$Record("TBird", {});',
    '      $r.addField("b", rtl.word);',
    '    });',
    '    $impl.f = null;',
    '  };',
    '  $mod.$init = function () {',
    '    $impl.f.x.b = $impl.f.x.b + 10;',
    '  };',
    '}, []);']));
  CheckSource('TestGen_Class_ClassVarRecord_UnitImpl',
    LinesToStr([ // statements
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_ClassInterface_Corba;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface;',
  '  IUnknown = interface',
  '    [''{00000000-0000-0000-C000-000000000046}'']',
  '  end;',
  '  IInterface = IUnknown;',
  '  generic IBird<T> = interface(IInterface)',
  '    function GetSize: T;',
  '    procedure SetSize(i: T);',
  '    property Size: T read GetSize write SetSize;',
  '    procedure DoIt(i: T);',
  '  end;',
  '  TObject = class',
  '  end;',
  '  generic TBird<T> = class(TObject,specialize IBird<T>)',
  '    function GetSize: T; virtual; abstract;',
  '    procedure SetSize(i: T); virtual; abstract;',
  '    procedure DoIt(i: T); virtual; abstract;',
  '  end;',
  '  IWordBird = specialize IBird<Word>;',
  '  TWordBird = specialize TBird<Word>;',
  'var',
  '  BirdIntf: IWordBird;',
  'begin',
  '  BirdIntf.Size:=BirdIntf.Size;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ClassInterface_Corba',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{00000000-0000-0000-C000-000000000046}", [], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createInterface(',
    '  this,',
    '  "IBird$G2",',
    '  "{33AB51C6-6240-3BDF-B4B0-D48A593EAB0A}",',
    '  ["GetSize", "SetSize", "DoIt"],',
    '  this.IUnknown,',
    '  "IBird<System.Word>"',
    ');',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird$G2);',
    '}, "TBird<System.Word>");',
    'this.BirdIntf = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.BirdIntf.SetSize($mod.BirdIntf.GetSize());',
    '']));
end;

procedure TTestGenerics.TestGen_ClassInterface_InterfacedObject;
begin
  StartProgram(true,[supTInterfacedObject]);
  Add([
  '{$mode delphi}',
  'type',
  '  IComparer<T> = interface [''{505778ED-F783-4456-9691-32F419CC5E18}'']',
  '    function Compare(const Left, Right: T): Integer; overload;',
  '  end;',
  '  TComparer<T> = class(TInterfacedObject, IComparer<T>)',
  '    function Compare(const Left, Right: T): Integer;',
  '  end;',
  'function TComparer<T>.Compare(const Left, Right: T): Integer; begin end;',
  'var',
  '  aComparer : IComparer<Integer>;',
  'begin',
  '  aComparer:=TComparer<Integer>.Create;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ClassInterface_InterfacedObject',
    LinesToStr([ // statements
    'rtl.createInterface(',
    '  this,',
    '  "IComparer$G2",',
    '  "{505778ED-F783-4456-9691-32F419CC5E18}",',
    '  ["Compare"],',
    '  pas.system.IUnknown,',
    '  "IComparer<System.Longint>"',
    ');',
    'this.aComparer = null;',
    'rtl.createClass(this, "TComparer$G1", pas.system.TInterfacedObject, function () {',
    '  this.Compare = function (Left, Right) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  rtl.addIntf(this, $mod.IComparer$G2);',
    '  rtl.addIntf(this, pas.system.IUnknown);',
    '}, "TComparer<System.Longint>");',
    '']),
    LinesToStr([ // $mod.$main
    'rtl.setIntfP($mod, "aComparer", rtl.queryIntfT($mod.TComparer$G1.$create("Create"), $mod.IComparer$G2), true);',
    '']));
end;

procedure TTestGenerics.TestGen_ClassInterface_COM_RTTI;
begin
  StartProgram(true,[supTInterfacedObject]);
  Add([
  '{$mode delphi}',
  'type',
  '  TBird = class',
  '    function Fly<T: IInterface>: T;',
  '  end;',
  '  IAnt = interface',
  '    procedure InterfaceProc;',
  '  end;',
  'function TBird.Fly<T>: T;',
  'begin',
  '  if TypeInfo(T)=nil then ;',
  'end;',
  'var Bird: TBird;',
  '  Ant: IAnt;',
  'begin',
  '  Ant := Bird.Fly<IAnt>;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ClassInterface_COM_RTTI',
    LinesToStr([ // statements
    'rtl.createClass(this, "TBird", pas.system.TObject, function () {',
    '  this.Fly$G1 = function () {',
    '    var Result = null;',
    '    if ($mod.$rtti["IAnt"] === null) ;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createInterface(this, "IAnt", "{B9D0FF27-A446-3A1B-AA85-F167837AA297}", ["InterfaceProc"], pas.system.IUnknown);',
    'this.Bird = null;',
    'this.Ant = null;',
    '']),
    LinesToStr([ // $mod.$main
    'rtl.setIntfP($mod, "Ant", $mod.Bird.Fly$G1(), true);',
    '']));
end;

procedure TTestGenerics.TestGen_ClassInterface_Helper;
begin
  StartProgram(true,[supTInterfacedObject]);
  Add([
  '{$mode objfpc}',
  '{$ModeSwitch typehelpers}',
  'type',
  '  IAnt = interface',
  '    procedure InterfaceProc;',
  '  end;',
  '  TBird = type helper for IAnt',
  '    generic procedure Fly<T>(a: T);',
  '  end;',
  'generic procedure TBird.Fly<T>(a: T);',
  'begin',
  'end;',
  'var ',
  '  Ant: IAnt;',
  'begin',
  '  Ant.specialize Fly<word>(3);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_ClassInterface_COM_RTTI',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IAnt", "{B9D0FF27-A446-3A1B-AA85-F167837AA297}", ["InterfaceProc"], pas.system.IUnknown);',
    'rtl.createHelper(this, "TBird", null, function () {',
    '  this.Fly$G1 = function (a) {',
    '  };',
    '});',
    'this.Ant = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TBird.Fly$G1.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.Ant;',
    '    },',
    '  set: function (v) {',
    '      rtl.setIntfP(this.p, "Ant", v);',
    '    }',
    '}, 3);',
    '']));
end;

procedure TTestGenerics.TestGen_ClassInterface_DelayedInitSpec;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject,supTInterfacedObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  '{$mode delphi}',
  'type',
  '  TAnt<T> = interface',
  '    procedure Run(x: T);',
  '  end;',
  '']),
  LinesToStr([
  '']));
  Add([
  '{$mode delphi}',
  'uses UnitA;',
  'type',
  '  TArrWord = array of word;',
  '  TMyIntf = TAnt<TArrWord>;',
  '  TBird = class(TInterfacedObject,TMyIntf)',
  '    procedure Run(a: TArrWord); external name ''Run'';',
  '  end;',
  'var',
  '  i: TMyIntf;',
  'begin',
  '  i:=TBird.Create;',
  '  i.Run([3,4]);',
  'end.']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  $mod.$rtti.$Interface("TAnt<test1.TArrWord>");',
    '  rtl.createInterface(',
    '    this,',
    '    "TAnt$G1",',
    '    "{B145F21B-2696-32D5-87A5-F16C037A2D45}",',
    '    ["Run"],',
    '    pas.system.IUnknown,',
    '    function () {',
    '      this.$initSpec = function () {',
    '        var $r = this.$rtti;',
    '        $r.addMethod("Run", 0, [["x", pas.program.$rtti["TArrWord"]]]);',
    '      };',
    '    },',
    '    "TAnt<test1.TArrWord>"',
    '  );',
    '});']));
  CheckSource('TestGen_ClassInterface_DelayedInitSpec',
    LinesToStr([ // statements
    'this.$rtti.$DynArray("TArrWord", {',
    '  eltype: rtl.word',
    '});',
    'rtl.createClass(this, "TBird", pas.system.TInterfacedObject, function () {',
    '  rtl.addIntf(this, pas.UnitA.TAnt$G1);',
    '  rtl.addIntf(this, pas.system.IUnknown);',
    '});',
    'this.i = null;',
    '$mod.$implcode = function () {',
    '  pas.UnitA.TAnt$G1.$initSpec();',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    'rtl.setIntfP($mod, "i", rtl.queryIntfT($mod.TBird.$create("Create"), pas.UnitA.TAnt$G1), true);',
    '$mod.i.Run([3, 4]);',
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
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '}, "TBird<System.Word>");',
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
    '  rtl.createClass(this, "TBird$G1", pas.system.TObject, function () {',
    '    this.Fly = function () {',
    '      $impl.DoIt();',
    '    };',
    '  }, "TBird<System.Boolean>");',
    '  this.b = null;',
    '  rtl.createClass(this, "TBird$G2", pas.system.TObject, function () {',
    '    this.Fly = function () {',
    '      $impl.DoIt();',
    '    };',
    '  }, "TBird<System.Word>");',
    '  $mod.$implcode = function () {',
    '    $impl.DoIt = function () {',
    '      var b = null;',
    '      b = $mod.TBird$G2.$create("Create");',
    '      b.Fly();',
    '    };',
    '  };',
    '}, []);',
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
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.m = 0;',
    '  };',
    '  this.Fly = function () {',
    '    var i = 0;',
    '    i = this.m;',
    '  };',
    '}, "TBird<System.Word>");',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_TypeCastDotField;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '    Field: T;',
  '    procedure Fly;',
  '  end;',
  'var',
  '  o: TObject;',
  '  b: specialize TBird<word>;',
  'procedure TBird.Fly;',
  'begin',
  '  specialize TBird<word>(o).Field:=3;',
  '  if 4=specialize TBird<word>(o).Field then ;',
  'end;',
  'begin',
  '  specialize TBird<word>(o).Field:=5;',
  '  if 6=specialize TBird<word>(o).Field then ;',
  '']);
  ConvertProgram;
  CheckSource('TestGen_TypeCastDotField',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.o = null;',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.Field = 0;',
    '  };',
    '  this.Fly = function () {',
    '    $mod.o.Field = 3;',
    '    if (4 === $mod.o.Field) ;',
    '  };',
    '}, "TBird<System.Word>");',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.Field = 5;',
    'if (6 === $mod.o.Field) ;',
    '']));
end;

procedure TTestGenerics.TestGen_Except;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  generic TBird<T> = class',
  '    Field: T;',
  '    procedure Fly;',
  '  end;',
  '  Exception = class',
  '  end;',
  '  generic EBird<T> = class(Exception)',
  '    Id: T;',
  '  end;',
  'var',
  '  b: specialize TBird<word>;',
  'procedure TBird.Fly;',
  'begin',
  '  try',
  '  except',
  '    on E: Exception do Fly;',
  '    on EBird: specialize EBird<word> do EBird.Id:=3;',
  '  else',
  '    Fly;',
  '  end;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestGen_Except',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "Exception", this.TObject, function () {',
    '});',
    'rtl.createClass(this, "TBird$G1", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.Field = 0;',
    '  };',
    '  this.Fly = function () {',
    '    try {} catch ($e) {',
    '      if ($mod.Exception.isPrototypeOf($e)) {',
    '        var E = $e;',
    '        this.Fly();',
    '      } else if ($mod.EBird$G1.isPrototypeOf($e)) {',
    '        var EBird = $e;',
    '        EBird.Id = 3;',
    '      } else {',
    '        this.Fly();',
    '      }',
    '    };',
    '  };',
    '}, "TBird<System.Word>");',
    'this.b = null;',
    'rtl.createClass(this, "EBird$G1", this.Exception, function () {',
    '  this.$init = function () {',
    '    $mod.Exception.$init.call(this);',
    '    this.Id = 0;',
    '  };',
    '}, "EBird<System.Word>");',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_HelperForArray;
begin
  StartProgram(false);
  Add([
  '{$ModeSwitch typehelpers}',
  'type',
  '  generic TArr<T> = array[1..2] of T;',
  '  TWordArrHelper = type helper for specialize TArr<word>',
  '    procedure Fly(w: word);',
  '  end;',
  'procedure TWordArrHelper.Fly(w: word);',
  'begin',
  'end;',
  'var',
  '  a: specialize TArr<word>;',
  'begin',
  '  a.Fly(3);',
  '']);
  ConvertProgram;
  CheckSource('TestGen_HelperForArray',
    LinesToStr([ // statements
    'rtl.createHelper(this, "TWordArrHelper", null, function () {',
    '  this.Fly = function (w) {',
    '  };',
    '});',
    'this.a = rtl.arraySetLength(null, 0, 2);',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TWordArrHelper.Fly.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.a;',
    '    },',
    '  set: function (v) {',
    '      this.p.a = v;',
    '    }',
    '}, 3);',
    '']));
end;

procedure TTestGenerics.TestGenProc_Function_ObjFPC;
begin
  StartProgram(false);
  Add([
  'generic function Run<T>(a: T): T;',
  'var i: T;',
  'begin',
  '  a:=i;',
  '  Result:=a;',
  'end;',
  'var w: word;',
  'begin',
  '  w:=specialize Run<word>(3);',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_Function_ObjFPC',
    LinesToStr([ // statements
    'this.w = 0;',
    'this.Run$G1 = function (a) {',
    '  var Result = 0;',
    '  var i = 0;',
    '  a = i;',
    '  Result = a;',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.w = $mod.Run$G1(3);',
    '']));
end;

procedure TTestGenerics.TestGenProc_Function_Delphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'function Run<T>(a: T): T;',
  'var i: T;',
  'begin',
  '  a:=i;',
  '  Result:=a;',
  'end;',
  'var w: word;',
  'begin',
  '  w:=Run<word>(3);',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_Function_Delphi',
    LinesToStr([ // statements
    'this.w = 0;',
    'this.Run$G1 = function (a) {',
    '  var Result = 0;',
    '  var i = 0;',
    '  a = i;',
    '  Result = a;',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.w = $mod.Run$G1(3);',
    '']));
end;

procedure TTestGenerics.TestGenProc_Overload;
begin
  StartProgram(false);
  Add([
  'generic procedure DoIt<T>(a: T; w: word); overload;',
  'begin',
  'end;',
  'generic procedure DoIt<T>(a: T; b: boolean); overload;',
  'begin',
  'end;',
  'begin',
  '  specialize DoIt<word>(3,4);',
  '  specialize DoIt<boolean>(false,5);',
  '  specialize DoIt<word>(6,true);',
  '  specialize DoIt<double>(7.3,true);',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_Overload',
    LinesToStr([ // statements
    'this.DoIt$G1 = function (a, w) {',
    '};',
    'this.DoIt$G2 = function (a, w) {',
    '};',
    'this.DoIt$1G1 = function (a, b) {',
    '};',
    'this.DoIt$1G2 = function (a, b) {',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt$G1(3, 4);',
    '$mod.DoIt$G2(false, 5);',
    '$mod.DoIt$1G1(6, true);',
    '$mod.DoIt$1G2(7.3, true);',
    '']));
end;

procedure TTestGenerics.TestGenProc_Forward;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'procedure Run<S>(a: S; b: boolean); forward;',
  'procedure Run<S>(a: S; b: boolean);',
  'begin',
  '  Run<word>(1,true);',
  'end;',
  'begin',
  '  Run(1.3,true);',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_infer_OverloadForward',
    LinesToStr([ // statements
    'this.Run$G1 = function (a, b) {',
    '  $mod.Run$G1(1, true);',
    '};',
    'this.Run$G2 = function (a, b) {',
    '  $mod.Run$G1(1, true);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Run$G2(1.3, true);',
    '']));
end;

procedure TTestGenerics.TestGenProc_Infer_OverloadForward;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'procedure {#A}Run<S>(a: S; b: boolean); forward; overload;',
  'procedure {#B}Run<T>(a: T; w: word); forward; overload;',
  'procedure {#C}Run<U>(a: U; b: U); forward; overload;',
  'procedure {#A2}Run<S>(a: S; b: boolean); overload;',
  'begin',
  '  {@A}Run(1,true);', // non generic take precedence
  '  {@B}Run(2,word(3));', // non generic take precedence
  '  {@C}Run(''foo'',''bar'');',
  'end;',
  'procedure {#B2}Run<T>(a: T; w: word); overload;',
  'begin',
  'end;',
  'procedure {#C2}Run<U>(a: U; b: U); overload;',
  'begin',
  'end;',
  'begin',
  '  {@A}Run(1,true);', // non generic take precedence
  '  {@B}Run(2,word(3));', // non generic take precedence
  '  {@C}Run(''foo'',''bar'');',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_infer_OverloadForward',
    LinesToStr([ // statements
    'this.Run$G1 = function (a, b) {',
    '  $mod.Run$G1(1, true);',
    '  $mod.Run$1G1(2, 3);',
    '  $mod.Run$2G1("foo", "bar");',
    '};',
    'this.Run$1G1 = function (a, w) {',
    '};',
    'this.Run$2G1 = function (a, b) {',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Run$G1(1, true);',
    '$mod.Run$1G1(2, 3);',
    '$mod.Run$2G1("foo", "bar");',
    '']));
end;

procedure TTestGenerics.TestGenProc_TypeInfo;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$modeswitch implicitfunctionspecialization}',
  'generic procedure Run<S>(a: S);',
  'var',
  '  p: TTypeInfo;',
  'begin',
  '  p:=TypeInfo(S);',
  '  p:=TypeInfo(a);',
  'end;',
  'begin',
  '  Run(word(3));',
  '  Run(''foo'');',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_TypeInfo',
    LinesToStr([ // statements
    'this.Run$G1 = function (a) {',
    '  var p = null;',
    '  p = rtl.word;',
    '  p = rtl.word;',
    '};',
    'this.Run$G2 = function (a) {',
    '  var p = null;',
    '  p = rtl.string;',
    '  p = rtl.string;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Run$G1(3);',
    '$mod.Run$G2("foo");',
    '']));
end;

procedure TTestGenerics.TestGenProc_Infer_Widen;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'procedure Run<S>(a: S; b: S);',
  'begin',
  'end;',
  'begin',
  '  Run(word(1),longint(2));',
  '  Run(byte(2),smallint(2));',
  '  Run(longword(3),longint(2));',
  '  Run(nativeint(4),longint(2));',
  '  Run(nativeint(5),nativeuint(2));',
  '  Run(''a'',''foo'');',
  '  Run(''bar'',''c'');',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_Infer_Widen',
    LinesToStr([ // statements
    'this.Run$G1 = function (a, b) {',
    '};',
    'this.Run$G2 = function (a, b) {',
    '};',
    'this.Run$G3 = function (a, b) {',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Run$G1(1, 2);',
    '$mod.Run$G1(2, 2);',
    '$mod.Run$G2(3, 2);',
    '$mod.Run$G2(4, 2);',
    '$mod.Run$G2(5, 2);',
    '$mod.Run$G3("a", "foo");',
    '$mod.Run$G3("bar", "c");',
    '']));
end;

procedure TTestGenerics.TestGenProc_Infer_PassAsArg;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'function Run<T>(a: T): T;',
  'var b: T;',
  'begin',
  '  Run(Run<word>(3));',
  '  Run(Run(word(4)));',
  'end;',
  'begin',
  '  Run(Run<word>(5));',
  '  Run(Run(word(6)));',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_Infer_PassAsArg',
    LinesToStr([ // statements
    'this.Run$G1 = function (a) {',
    '  var Result = 0;',
    '  var b = 0;',
    '  $mod.Run$G1($mod.Run$G1(3));',
    '  $mod.Run$G1($mod.Run$G1(4));',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Run$G1($mod.Run$G1(5));',
    '$mod.Run$G1($mod.Run$G1(6));',
    '']));
end;

procedure TTestGenerics.TestGenProc_AnonymousProc;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TProc = reference to procedure;',
  '  TFunc = reference to function(Value: JSValue): JSValue;',
  'function Run<T>(a: T; p: TProc): T;',
  'var b: T;',
  '  f: TFunc;',
  'begin',
  '  Result:=Run(a,procedure()begin end);',
  '  f:=function(b: JSValue): JSValue begin end;',
  '  f:=function(b: JSValue): JSValue',
  '      function Sub(c: JSValue): JSValue;',
  '      begin',
  '        Result:=c;',
  '      end;',
  '    begin',
  '      Result:=Sub(b);',
  '    end;',
  'end;',
  'begin',
  '  Run<word>(3,procedure() begin end);',
  '']);
  ConvertProgram;
  CheckSource('TestGenProc_AnonymousProc',
    LinesToStr([ // statements
    'this.Run$G1 = function (a, p) {',
    '  var Result = 0;',
    '  var b = 0;',
    '  var f = null;',
    '  Result = $mod.Run$G1(a, function () {',
    '  });',
    '  f = function (b) {',
    '    var Result = undefined;',
    '    return Result;',
    '  };',
    '  f = function (b) {',
    '    var Result = undefined;',
    '    function Sub(c) {',
    '      var Result = undefined;',
    '      Result = c;',
    '      return Result;',
    '    };',
    '    Result = Sub(b);',
    '    return Result;',
    '  };',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Run$G1(3, function () {',
    '});',
    '']));
end;

procedure TTestGenerics.TestGenMethod_ImplicitSpec_ObjFPC;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$ModeSwitch implicitfunctionspecialization}',
  'type',
  '  TObject = class',
  '    generic procedure {#A}Run<S>(a: S; b: boolean); overload;',
  '    generic procedure {#B}Run<T>(a: T; w: word); overload;',
  '    generic procedure {#C}Run<U>(a: U; b: U); overload;',
  '  end; ',
  'generic procedure {#A2}TObject.Run<S>(a: S; b: boolean); overload;',
  'begin',
  '  {@A}Run(1,true);', // non generic take precedence
  '  {@B}Run(2,word(3));', // non generic take precedence
  '  {@C}Run(''foo'',''bar'');',
  'end;',
  'generic procedure {#B2}TObject.Run<T>(a: T; w: word); overload;',
  'begin',
  'end;',
  'generic procedure {#C2}TObject.Run<U>(a: U; b: U); overload;',
  'begin',
  'end;',
  'var o: TObject;',
  'begin',
  '  o.{@A}Run(1,true);', // non generic take precedence
  '  o.{@B}Run(2,word(3));', // non generic take precedence
  '  o.{@C}Run(''foo'',''bar'');',
  '']);
  ConvertProgram;
  CheckSource('TestGenMethod_ImplicitSpec_ObjFPC',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Run$G1 = function (a, b) {',
    '    this.Run$G1(1, true);',
    '    this.Run$1G1(2, 3);',
    '    this.Run$2G1("foo", "bar");',
    '  };',
    '  this.Run$1G1 = function (a, w) {',
    '  };',
    '  this.Run$2G1 = function (a, b) {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.Run$G1(1, true);',
    '$mod.o.Run$1G1(2, 3);',
    '$mod.o.Run$2G1("foo", "bar");',
    '']));
end;

procedure TTestGenerics.TestGenMethod_Delphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class',
  '    procedure Run<S>;',
  '  end; ',
  'procedure TObject.Run<S>;',
  'begin',
  'end;',
  'var o: TObject;',
  'begin',
  '  o.Run<word>;',
  '  o.Run<word>();',
  '  with o do begin',
  '    Run<word>;',
  '    Run<word>();',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestGenMethod_Delphi',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Run$G1 = function () {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.Run$G1();',
    '$mod.o.Run$G1();',
    'var $with = $mod.o;',
    '$with.Run$G1();',
    '$with.Run$G1();',
    '']));
end;

procedure TTestGenerics.TestGen_Array_OtherUnit;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  'type',
  '  generic TDyn<T> = array of T;',
  '  generic TStatic<T> = array[1..2] of T;',
  '']),
  '');
  AddModuleWithIntfImplSrc('UnitB.pas',
  LinesToStr([
  'uses UnitA;',
  'type',
  '  TAnt = class end;',
  '  TAntArray = specialize TDyn<TAnt>;',
  'procedure Run;',
  '']),
  LinesToStr([
  'procedure Run;',
  'begin',
  '  if typeinfo(TAntArray)=nil then ;',
  'end;',
  '']));
  Add([
  'uses UnitB;',
  'begin',
  '  Run;',
  '']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  this.$rtti.$DynArray("TDyn<UnitB.TAnt>", {});',
    '});']));
  CheckUnit('UnitB.pas',
    LinesToStr([ // statements
    'rtl.module("UnitB", ["system", "UnitA"], function () {',
    '  var $mod = this;',
    '  rtl.createClass(this, "TAnt", pas.system.TObject, function () {',
    '  });',
    '  this.Run = function () {',
    '    if (pas.UnitA.$rtti["TDyn<UnitB.TAnt>"] === null) ;',
    '  };',
    '});']));
  CheckSource('TestGen_Array_OtherUnit',
    LinesToStr([ // statements
    '$mod.$implcode = function () {',
    '  pas.UnitA.$rtti["TDyn<UnitB.TAnt>"].eltype = pas.UnitB.$rtti["TAnt"];',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '  pas.UnitB.Run();',
    '']));
end;

procedure TTestGenerics.TestGen_ArrayOfUnitImplRec;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  'type',
  '  generic TDyn<T> = array of T;',
  '  generic TStatic<T> = array[1..2] of T;',
  '']),
  LinesToStr([
  'type',
  '  TBird = record',
  '    b: word;',
  '  end;',
  '  TAnt = class end;',
  '  TAntArray = specialize TDyn<TAnt>;',
  'var',
  '  d: specialize TDyn<TBird>;',
  '  s: specialize TStatic<TBird>;',
  '  p: pointer;',
  'begin',
  '  d[0].b:=s[1].b;',
  '  s:=s;',
  '  p:=typeinfo(TAntArray);',
  '']));
  Add([
  'uses UnitA;',
  'begin',
  '']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  this.$rtti.$DynArray("TDyn<UnitA.TAnt>", {});',
    '  this.$rtti.$DynArray("TDyn<UnitA.TBird>", {});',
    '  this.TStatic$G1$clone = function (a) {',
    '    var r = [];',
    '    for (var i = 0; i < 2; i++) r.push($impl.TBird.$clone(a[i]));',
    '    return r;',
    '  };',
    '  this.$rtti.$StaticArray("TStatic<UnitA.TBird>", {',
    '    dims: [2]',
    '  });',
    '  $mod.$implcode = function () {',
    '    rtl.recNewT($impl, "TBird", function () {',
    '      this.b = 0;',
    '      this.$eq = function (b) {',
    '        return this.b === b.b;',
    '      };',
    '      this.$assign = function (s) {',
    '        this.b = s.b;',
    '        return this;',
    '      };',
    '      var $r = $mod.$rtti.$Record("TBird", {});',
    '      $r.addField("b", rtl.word);',
    '    });',
    '    rtl.createClass($impl, "TAnt", pas.system.TObject, function () {',
    '    });',
    '    $impl.d = [];',
    '    $impl.s = rtl.arraySetLength(null, $impl.TBird, 2);',
    '    $impl.p = null;',
    '  };',
    '  $mod.$init = function () {',
    '    $impl.d[0].b = $impl.s[0].b;',
    '    $impl.s = $mod.TStatic$G1$clone($impl.s);',
    '    $impl.p = $mod.$rtti["TDyn<UnitA.TAnt>"];',
    '  };',
    '}, []);']));
  CheckSource('TestGen_ArrayOfUnitImplRec',
    LinesToStr([ // statements
    '$mod.$implcode = function () {',
    '  pas.UnitA.$rtti["TDyn<UnitA.TAnt>"].eltype = pas.UnitA.$rtti["TAnt"];',
    '  pas.UnitA.$rtti["TDyn<UnitA.TBird>"].eltype = pas.UnitA.$rtti["TBird"];',
    '  pas.UnitA.$rtti["TStatic<UnitA.TBird>"].eltype = pas.UnitA.$rtti["TBird"];',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_Array_TypecastJSValueResultToArg;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TArray<T> = array of T;',
  '  TFunc = function: JSValue of object;',
  '  TObject = class',
  '    f: TFunc;',
  '    function Run: jsvalue; virtual; abstract;',
  '  end;',
  'procedure Sit(Arr: TArray<TObject>);',
  'begin',
  'end;',
  'procedure Fly(o: TObject);',
  'begin',
  '  Sit(TArray<TObject>(o.f()));',
  '  Sit(TArray<TObject>(o.Run));',
  '  Sit(TArray<TObject>(o.Run()));',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestGen_Array_TypecastJSValueResultToArg',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.f = null;',
    '  };',
    '  this.$final = function () {',
    '    this.f = undefined;',
    '  };',
    '});',
    'this.Sit = function (Arr) {',
    '};',
    'this.Fly = function (o) {',
    '  $mod.Sit(o.f());',
    '  $mod.Sit(o.Run());',
    '  $mod.Sit(o.Run());',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_ProcType_ProcLocal;
begin
  StartProgram(false);
  Add([
  'procedure Fly(w: word);',
  'begin',
  'end;',
  'procedure Run(w: word);',
  'type generic TProc<T> = procedure(a: T);',
  'var p: specialize TProc<word>;',
  'begin',
  '  p:=@Fly;',
  '  p(w);',
  'end;',
  'begin',
  'end.']);
  ConvertProgram;
  CheckSource('TestGen_ProcType_ProcLocal',
    LinesToStr([ // statements
    'this.Fly = function (w) {',
    '};',
    'this.Run = function (w) {',
    '  var p = null;',
    '  p = $mod.Fly;',
    '  p(w);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestGenerics.TestGen_ProcType_Local_RTTI_Fail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'procedure Fly(w: word);',
  'begin',
  'end;',
  'procedure Run(w: word);',
  'type generic TProc<T> = procedure(a: T);',
  'var',
  '  p: specialize TProc<word>;',
  '  t: Pointer;',
  'begin',
  '  p:=@Fly;',
  '  p(w);',
  '  t:=typeinfo(p);',
  'end;',
  'begin',
  'end.']);
  SetExpectedPasResolverError(sSymbolCannotBePublished,nSymbolCannotBePublished);
  ConvertProgram;
end;

procedure TTestGenerics.TestGen_ProcType_ParamUnitImpl;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTObject]);
  AddModuleWithIntfImplSrc('UnitA.pas',
  LinesToStr([
  'type',
  '  generic TAnt<T> = function(const a: T): T;',
  '']),
  LinesToStr([
  'type',
  '  TBird = record',
  '    b: word;',
  '  end;',
  'var',
  '  f: specialize TAnt<TBird>;',
  '  b: TBird;',
  '  p: pointer;',
  'begin',
  '  b:=f(b);',
  '  p:=typeinfo(f);',
  '']));
  Add([
  'uses UnitA;',
  'begin',
  'end.']);
  ConvertProgram;
  CheckUnit('UnitA.pas',
    LinesToStr([ // statements
    'rtl.module("UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  var $impl = $mod.$impl;',
    '  this.$rtti.$ProcVar("TAnt<UnitA.TBird>", {',
    '    init: function () {',
    '      this.procsig = rtl.newTIProcSig([["a", $mod.$rtti["TBird"], 2]], $mod.$rtti["TBird"]);',
    '    }',
    '  });',
    '  $mod.$implcode = function () {',
    '    rtl.recNewT($impl, "TBird", function () {',
    '      this.b = 0;',
    '      this.$eq = function (b) {',
    '        return this.b === b.b;',
    '      };',
    '      this.$assign = function (s) {',
    '        this.b = s.b;',
    '        return this;',
    '      };',
    '      var $r = $mod.$rtti.$Record("TBird", {});',
    '      $r.addField("b", rtl.word);',
    '    });',
    '    $impl.f = null;',
    '    $impl.b = $impl.TBird.$new();',
    '    $impl.p = null;',
    '  };',
    '  $mod.$init = function () {',
    '    $impl.b.$assign($impl.f($impl.b));',
    '    $impl.p = $mod.$rtti["TAnt<UnitA.TBird>"];',
    '  };',
    '}, []);']));
  CheckSource('TestGen_Class_ClassVarRecord_UnitImpl',
    LinesToStr([ // statements
    '$mod.$implcode = function () {',
    '  pas.UnitA.$rtti["TAnt<UnitA.TBird>"].init();',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

Initialization
  RegisterTests([TTestGenerics]);
end.

