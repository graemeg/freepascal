unit tcgenerics;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, pastree, testregistry, pscanner, tctypeparser;

Type

  { TTestGenerics - for resolver see unit tcresolvegenerics }

  TTestGenerics = Class(TBaseTestTypeParser)
  Published
    // generic types
    Procedure TestObjectGenerics;
    Procedure TestRecordGenerics;
    Procedure TestArrayGenerics;
    Procedure TestArrayGenericsDelphi;
    Procedure TestProcTypeGenerics;
    Procedure TestDeclarationDelphi;
    Procedure TestDeclarationFPC;
    Procedure TestDeclarationFPCNoSpaces;
    Procedure TestMethodImplementation;

    // generic constraints
    Procedure TestGenericConstraint;
    Procedure TestGenericInterfaceConstraint;
    Procedure TestDeclarationConstraint;

    // specialize type
    Procedure TestSpecializationDelphi;
    Procedure TestDeclarationDelphiSpecialize;
    Procedure TestInlineSpecializationInArgument;
    Procedure TestSpecializeNested;
    Procedure TestInlineSpecializeInStatement;
    Procedure TestInlineSpecializeInStatementDelphi;

    // generic functions
    Procedure TestGenericFunction_Program;
    Procedure TestGenericFunction_Unit;

    // generic method
    Procedure TestGenericMethod_Program;
  end;

implementation

procedure TTestGenerics.TestObjectGenerics;
begin
  Add([
    'Type',
    'Generic TSomeClass<T> = Object',
    '  b : T;',
    'end;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestRecordGenerics;
begin
  Add([
    'Type',
    '  Generic TSome<T> = Record',
    '    b : T;',
    '  end;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestArrayGenerics;
begin
  Add([
    'Type',
    '  Generic TSome<T> = array of T;',
    '  Generic TStatic<R,T> = array[R] of T;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestArrayGenericsDelphi;
begin
  Add([
    '{$mode delphi}',
    'Type',
    '  TSome<T> = array of T;',
    '  TStatic<R,T> = array[R] of T;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestProcTypeGenerics;
begin
  Add([
    'Type',
    '  Generic TSome<T> = procedure(v: T);',
    '  Generic TFunc<R,T> = function(b: R): T;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestDeclarationDelphi;
Var
  T : TPasClassType;
begin
  Scanner.CurrentModeSwitches:=[msDelphi]+Scanner.CurrentModeSwitches ;
  Source.Add('Type');
  Source.Add('  TSomeClass<T,T2> = Class(TObject)');
  Source.Add('    b : T;');
  Source.Add('    b2 : T2;');
  Source.Add('    FItems: ^TArray<T>;');
  Source.Add('  type');
  Source.Add('    TDictionaryEnumerator = TDictionary<T, TEmptyRecord>.TKeyEnumerator;');
  Source.Add('  end;');
  ParseDeclarations;
  AssertNotNull('have generic definition',Declarations.Classes);
  AssertEquals('have generic definition',1,Declarations.Classes.Count);
  AssertEquals('Pascal class',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  T:=TPasClassType(Declarations.Classes[0]);
  AssertNotNull('have generic templates',T.GenericTemplateTypes);
  AssertEquals('2 template types',2,T.GenericTemplateTypes.Count);
  AssertSame('Parent 0 is class',T,TPasElement(T.GenericTemplateTypes[0]).Parent);
  AssertSame('Parent 1 is class',T,TPasElement(T.GenericTemplateTypes[1]).Parent);
end;

procedure TTestGenerics.TestDeclarationFPC;
Var
  T : TPasClassType;
begin
  Scanner.CurrentModeSwitches:=[msDelphi]+Scanner.CurrentModeSwitches;
  Source.Add('Type');
  Source.Add('  TSomeClass<T;T2> = Class(TObject)');
  Source.Add('    b : T;');
  Source.Add('    b2 : T2;');
  Source.Add('  end;');
  ParseDeclarations;
  AssertNotNull('have generic definition',Declarations.Classes);
  AssertEquals('have generic definition',1,Declarations.Classes.Count);
  AssertEquals('Pascal class',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  T:=TPasClassType(Declarations.Classes[0]);
  AssertNotNull('have generic templates',T.GenericTemplateTypes);
  AssertEquals('2 template types',2,T.GenericTemplateTypes.Count);
  AssertSame('Parent 0 is class',T,TPasElement(T.GenericTemplateTypes[0]).Parent);
  AssertSame('Parent 1 is class',T,TPasElement(T.GenericTemplateTypes[1]).Parent);
end;

procedure TTestGenerics.TestDeclarationFPCNoSpaces;
Var
  T : TPasClassType;
begin
  Scanner.CurrentModeSwitches:=[msDelphi]+Scanner.CurrentModeSwitches;
  Source.Add('Type');
  Source.Add('  TSomeClass<T;T2>=Class(TObject)');
  Source.Add('    b : T;');
  Source.Add('    b2 : T2;');
  Source.Add('  end;');
  ParseDeclarations;
  AssertNotNull('have generic definition',Declarations.Classes);
  AssertEquals('have generic definition',1,Declarations.Classes.Count);
  AssertEquals('Pascal class',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  T:=TPasClassType(Declarations.Classes[0]);
  AssertNotNull('have generic templates',T.GenericTemplateTypes);
  AssertEquals('2 template types',2,T.GenericTemplateTypes.Count);
  AssertSame('Parent 0 is class',T,TPasElement(T.GenericTemplateTypes[0]).Parent);
  AssertSame('Parent 1 is class',T,TPasElement(T.GenericTemplateTypes[1]).Parent);
end;

procedure TTestGenerics.TestMethodImplementation;
begin
  With source do
    begin
    Add('unit afile;');
    Add('{$MODE DELPHI}');
    Add('interface');
    Add('type');
    Add('  TTest<T> =  object');
    Add('    procedure foo(v:T);');
    Add('    procedure bar<Y>(v:T);');
    Add('  type');
    Add('    TSub = class');
    Add('      procedure DoIt<Y>(v:T);');
    Add('    end;');
    Add('  end;');
    Add('implementation');
    Add('procedure TTest<T>.foo;');
    Add('begin');
    Add('end;');
    Add('procedure TTest<T>.bar<Y>;');
    Add('begin');
    Add('end;');
    Add('procedure TTest<T>.TSub.DoIt<Y>;');
    Add('begin');
    Add('end;');
    end;
  ParseModule;
end;

procedure TTestGenerics.TestGenericConstraint;
begin
  Add([
    'Type',
    'Generic TSomeClass<T: TObject> = class',
    '  b : T;',
    'end;',
    'Generic TBird<T: class> = class',
    '  c : specialize TBird<T>;',
    'end;',
    'Generic TEagle<T: record> = class',
    'end;',
    'Generic TEagle<T: constructor> = class',
    'end;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestGenericInterfaceConstraint;
begin
  Add([
    'Type',
    'TIntfA = interface end;',
    'TIntfB = interface end;',
    'TBird = class(TInterfacedObject,TIntfA,TIntfB) end;',
    'Generic TAnt<T: TIntfA, TIntfB> = class',
    '  b: T;',
    '  c: specialize TAnt<T>;',
    'end;',
    'Generic TFly<T: TIntfA, TIntfB; S> = class',
    '  b: S;',
    '  c: specialize TFly<T>;',
    'end;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestDeclarationConstraint;
Var
  T : TPasClassType;
begin
  Scanner.CurrentModeSwitches:=[msDelphi]+Scanner.CurrentModeSwitches ;
  Source.Add('Type');
  Source.Add('  TSomeClass<T: T2> = Class(TObject)');
  Source.Add('    b : T;');
  Source.Add('  end;');
  ParseDeclarations;
  AssertNotNull('have generic definition',Declarations.Classes);
  AssertEquals('have generic definition',1,Declarations.Classes.Count);
  AssertEquals('Pascal class',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  T:=TPasClassType(Declarations.Classes[0]);
  AssertNotNull('have generic templates',T.GenericTemplateTypes);
  AssertEquals('1 template types',1,T.GenericTemplateTypes.Count);
  AssertSame('Parent 0 is class',T,TPasElement(T.GenericTemplateTypes[0]).Parent);
  AssertEquals('Type constraint is recorded','T2',TPasGenericTemplateType(T.GenericTemplateTypes[0]).TypeConstraint);
end;

procedure TTestGenerics.TestSpecializationDelphi;
begin
  Add('{$mode delphi}');
  ParseType('TFPGList<integer>',TPasSpecializeType,'');
end;

procedure TTestGenerics.TestDeclarationDelphiSpecialize;
Var
  T : TPasClassType;
begin
  Scanner.CurrentModeSwitches:=[msDelphi]+Scanner.CurrentModeSwitches ;
  Source.Add('Type');
  Source.Add('  TSomeClass<T,T2> = Class(TSomeGeneric<Integer,Integer>)');
  Source.Add('    b : T;');
  Source.Add('    b2 : T2;');
  Source.Add('  end;');
  ParseDeclarations;
  AssertNotNull('have generic definition',Declarations.Classes);
  AssertEquals('have generic definition',1,Declarations.Classes.Count);
  AssertEquals('Pascal class',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  T:=TPasClassType(Declarations.Classes[0]);
  AssertEquals('Name is correct','TSomeClass',T.Name);
  AssertNotNull('have generic templates',T.GenericTemplateTypes);
  AssertEquals('2 template types',2,T.GenericTemplateTypes.Count);
  AssertSame('Parent 0 is class',T,TPasElement(T.GenericTemplateTypes[0]).Parent);
  AssertSame('Parent 1 is class',T,TPasElement(T.GenericTemplateTypes[1]).Parent);
end;

procedure TTestGenerics.TestInlineSpecializationInArgument;
begin
  With source do
    begin
    Add('unit afile;');
    Add('{$MODE DELPHI}');
    Add('interface');
    Add('type');
    Add('  TFoo=class');
    Add('    procedure foo(var Node:TSomeGeneric<TBoundingBox>;const index:Integer);');
    Add('  end;');
    Add('implementation');
    end;
  ParseModule;
end;

procedure TTestGenerics.TestSpecializeNested;
begin
  Add([
    'Type',
    '  generic TSomeClass<A,B> = class(specialize TOther<A,specialize TAnother<B>>) end;',
    '']);
  ParseDeclarations;
end;

procedure TTestGenerics.TestInlineSpecializeInStatement;
begin
  Add([
  '{$mode objfpc}',
  'begin',
  '  vec:=specialize TVector<double>.create;',
  '  t:=specialize a<b>;',
  //'  t:=specialize a<b.specialize c<d,e.f>>;',
  //'  t:=a.specialize b<c>;',
  '  t:=specialize a<b>.c;',
  '']);
  ParseModule;
end;

procedure TTestGenerics.TestInlineSpecializeInStatementDelphi;
begin
  Add([
  '{$mode delphi}',
  'begin',
  '  vec:=TVector<double>.create;',
  '  b:=a<b;',
  '  t:=a<b.c<d,e.f>>;',
  '  t:=a.b<c>;',
  '  t:=a<b>.c;',
  // forbidden:'  t:=a<b<c>.d>;',
  '']);
  ParseModule;
end;

procedure TTestGenerics.TestGenericFunction_Program;
begin
  Add([
  'generic function IfThen<T>(val:boolean;const iftrue:T; const iffalse:T) :T; inline; overload;',
  'begin',
  'end;',
  'begin',
  '  specialize IfThen<word>(true,2,3);',
  '']);
  ParseModule;
end;

procedure TTestGenerics.TestGenericFunction_Unit;
begin
  Add([
  'unit afile;',
  'interface',
  'generic function Get<T>(val: T) :T;',
  'implementation',
  'generic function Get<T>(val: T) :T;',
  'begin',
  'end;',
  'initialization',
  '  specialize GetIt<word>(2);',
  '']);
  ParseModule;
end;

procedure TTestGenerics.TestGenericMethod_Program;
begin
  Add([
  '{$mode objfpc}',
  'type',
  '  TObject = class',
  '    generic function Get<T>(val: T) :T;',
  '  type TBird = word;',
  '  generic procedure Fly<T>;',
  '  const C = 1;',
  '  generic procedure Run<T>;',
  '  end;',
  'generic function TObject.Get<T>(val: T) :T;',
  'begin',
  'end;',
  'begin',
  '  TObject.specialize GetIt<word>(2);',
  '']);
  ParseModule;
end;

initialization
  RegisterTest(TTestGenerics);
end.

