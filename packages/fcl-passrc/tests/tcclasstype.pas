unit tcclasstype;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, pscanner, pparser, pastree, testregistry, tctypeparser;

type

  { TTestClassType }
  TClassDeclType = (cdtClass,cdtObjCClass,cdtObjCCategory);
  TTestClassType = Class(TBaseTestTypeParser)
  Private
    FDecl : TStrings;
    FClass : TPasClassType;
    FMember1: TPasElement;
    FParent : String;
    FEnded,
    FStarted: Boolean;
    procedure AssertGenericClass(C: TPasClassType);
    procedure AssertSpecializedClass(C: TPasSpecializeType);
    function GetC(AIndex: Integer): TPasConst;
    function GetF1: TPasVariable;
    function GetM(AIndex : Integer): TPasElement;
    function GetMM(AIndex : Integer): TPasProcedure;
    function GetMF1: TPasFunction;
    function GetP1: TPasProperty;
    function GetP2: TPasProperty;
    function GetT(AIndex : Integer) : TPasType;
  protected
    Procedure StartClass (AncestorName : String = 'TObject'; InterfaceList : String = ''; aClassType : TClassDeclType = cdtClass);
    Procedure StartExternalClass (AParent : String; AExternalName,AExternalNameSpace : String );
    Procedure StartClassHelper (ForType : String = 'TOriginal'; AParent : String = 'TObject');
    Procedure StartInterface (AParent : String = 'IInterface'; UUID : String = ''; Disp : Boolean = False; UseObjcClass : Boolean = False; UseExternal : Boolean = False);
    Procedure StartRecordHelper (ForType : String = 'TOriginal'; AParent : String = 'TObject');
    Procedure StartVisibility(A : TPasMemberVisibility);
    Procedure EndClass(AEnd : String = 'end');
    Procedure AddMember(S : String);
    Procedure ParseClass;
    Procedure ParseClassFail(Msg: string; MsgNumber: integer);
    Procedure DoParseClass(FromSpecial : Boolean = False);
    procedure SetUp; override;
    procedure TearDown; override;
    procedure DefaultMethod;
    Procedure AssertParserError(Const Msg : String);
    Procedure AssertVisibility(V : TPasMemberVisibility = visDefault; Member : TPasElement = Nil);
    procedure AssertMemberType(AType : TClass; Member : TPaselement = Nil);
    procedure AssertMemberName(AName : string; Member : TPaselement = Nil);
    Procedure AssertProperty(P : TPasProperty; AVisibility : TPasMemberVisibility;AName,ARead,AWrite,AStored,AImplements : String; AArgCount : Integer; ADefault,ANodefault : Boolean);
    Property TheClass : TPasClassType Read FClass;
    Property Members[AIndex : Integer] : TPasElement Read GetM;
    Property Member1 : TPasElement Read FMember1;
    Property Field1 : TPasVariable Read GetF1;
    Property Method1 : TPasProcedure Index 0 Read GetMM;
    Property Method2 : TPasProcedure Index 1 Read GetMM;
    Property Method3 : TPasProcedure index 2 Read GetMM;
    Property FunctionMethod1 : TPasFunction Read GetMF1;
    Property Property1 : TPasProperty Read GetP1;
    Property Property2 : TPasProperty Read GetP2;
    Property Type1 : TPasType Index 0 Read GetT;
    Property Type2 : TPasType Index 1 Read GetT;
    Property Const1 : TPasConst Index 0 Read GetC;
    Property Const2 : TPasConst Index 1 Read GetC;
  published
    procedure TestEmpty;
    procedure TestEmptyComment;
    procedure TestEmptyDeprecated;
    procedure TestEmptyEnd;
    procedure TestEmptyEndNoParent;
    procedure TestEmptyObjC;
    procedure TestEmptyObjCCategory;
    Procedure TestOneInterface;
    Procedure TestTwoInterfaces;
    procedure TestOneSpecializedClass;
    procedure TestOneSpecializedClassInterface;
    Procedure TestOneField;
    Procedure TestOneFieldComment;
    procedure TestOneFieldStatic;
    Procedure TestOneHelperField;
    Procedure TestOneVarField;
    Procedure TestOneClassField;
    Procedure TestOneFieldVisibility;
    Procedure TestOneFieldDeprecated;
    Procedure TestTwoFields;
    Procedure TestTwoFieldsB;
    Procedure TestTwoVarFieldsB;
    procedure TestNoVarFields;
    procedure TestVarClassFunction;
    procedure TestClassVarClassFunction;
    Procedure TestTwoFieldsVisibility;
    Procedure TestConstProtectedEnd;
    Procedure TestTypeProtectedEnd;
    Procedure TestVarProtectedEnd;
    procedure TestHintFieldDeprecated;
    procedure TestHintFieldPlatform;
    procedure TestHintFieldExperimental;
    procedure TestHintFieldLibraryError;
    procedure TestHintFieldUninmplemented;
    Procedure TestOneVarFieldExternalName;
    procedure TestOneVarFieldExternalNameSemicolon;
    Procedure TestMethodSimple;
    Procedure TestMethodSimpleComment;
    Procedure TestMethodWithDotFails;
    Procedure TestMethodWithDotOK;
    Procedure TestMethodFunctionWithDotOK;
    Procedure TestNoSemicolon;
    Procedure TestClassMethodSimple;
    Procedure TestClassMethodSimpleComment;
    Procedure TestConstructor;
    Procedure TestClassConstructor;
    Procedure TestDestructor;
    Procedure TestClassDestructor;
    Procedure TestFunctionMethodSimple;
    Procedure TestClassFunctionMethodSimple;
    Procedure TestMethodOneArg;
    Procedure TestMethodVirtual;
    Procedure TestMethodVirtualSemicolon;
    Procedure TestMethodVirtualAbstract;
    procedure TestMethodVirtualAbstractFinal;
    Procedure TestMethodOverride;
    procedure TestMethodDynamic;
    procedure TestMethodReintroduce;
    procedure TestMethodInline;
    Procedure TestMethodVisibility;
    Procedure TestMethodSVisibility;
    Procedure TestMethodOverloadVisibility;
    Procedure TestMethodHint;
    Procedure TestMethodVirtualHint;
    Procedure TestIntegerMessageMethod;
    Procedure TestStringMessageMethod;
    Procedure Test2Methods;
    Procedure Test2MethodsDifferentVisibility;
    Procedure TestPropertyRedeclare;
    Procedure TestPropertyRedeclareComment;
    Procedure TestPropertyRedeclareDefault;
    Procedure TestPropertyReadOnly;
    Procedure TestPropertyReadWrite;
    Procedure TestPropertyWriteOnly;
    Procedure TestPropertyDefault;
    Procedure TestPropertyNoDefault;
    Procedure TestPropertyIndex;
    Procedure TestPropertyStored;
    Procedure TestPropertyStoredFalse;
    Procedure TestPropertyFullyQualifiedType;
    Procedure TestPropertyArrayReadOnly;
    Procedure TestPropertyArrayReadWrite;
    Procedure TestPropertyArrayReadOnlyDefault;
    Procedure TestPropertyArrayReadWriteDefault;
    Procedure TestPropertyArrayMultiDimReadOnly;
    Procedure TestPropertyImplements;
    Procedure TestPropertyImplementsFullyQualifiedName;
    Procedure TestPropertyReadFromRecordField;
    procedure TestPropertyReadFromArrayField;
    procedure TestPropertyReadWriteFromRecordField;
    procedure TestPropertyDeprecated;
    procedure TestPropertyDeprecatedMessage;
    Procedure TestExternalClass;
    Procedure TestExternalClassNoNameSpace;
    Procedure TestExternalClassNoNameKeyWord;
    Procedure TestExternalClassNoName;
    Procedure TestLocalSimpleType;
    Procedure TestLocalSimpleTypes;
    Procedure TestLocalSimpleConst;
    Procedure TestLocalSimpleConsts;
    Procedure TestClassTypeAttributes;
    Procedure TestClassConstAttributes;
    procedure TestClassHelperEmpty;
    procedure TestClassHelperParentedEmpty;
    procedure TestClassHelperOneMethod;
    procedure TestInterfaceEmpty;
    procedure TestObjcProtocolEmpty;
    procedure TestObjcProtocolEmptyExternal;
    procedure TestObjcProtocolMultiParent;
    procedure TestObjcProtocolOptional;
    procedure TestObjcProtocolRequired;
    procedure TestInterfaceDisp;
    procedure TestInterfaceParentedEmpty;
    procedure TestInterfaceOneMethod;
    procedure TestInterfaceDispIDMethod;
    procedure TestInterfaceDispIDMethod2;
    procedure TestInterfaceProperty;
    procedure TestInterfaceDispProperty;
    procedure TestInterfaceDispPropertyReadOnly;
    procedure TestInterfaceNoConstructor;
    procedure TestInterfaceNoDestructor;
    procedure TestInterfaceNoFields;
    procedure TestInterfaceUUID;
    procedure TestInterfaceUUIDParentedEmpty;
    procedure TestInterfaceUUIDOneMethod;
    procedure TestRecordHelperEmpty;
    procedure TestRecordHelperParentedEmpty;
    procedure TestRecordHelperOneMethod;
  end;

implementation

{ TTestClassType }

function TTestClassType.GetM(AIndex : Integer): TPasElement;
begin
  AssertNotNull('Have class',TheClass);
  if (AIndex>=TheClass.Members.Count) then
    Fail('No member '+IntToStr(AIndex));
  AssertNotNull('Have member'+IntToStr(AIndex),TheClass.Members[AIndex]);
  If Not (TObject(TheClass.Members[AIndex]) is TPasElement) then
    Fail('Member '+IntTostr(AIndex)+' is not a Tpaselement');
  Result:=TPasElement(TheClass.Members[AIndex])
end;

function TTestClassType.GetMM(AIndex: Integer): TPasProcedure;
begin
  AssertNotNull('Have member '+IntToStr(AIndex),Members[AIndex]);
  AssertEquals('Member is method '+IntToStr(AIndex),TPasProcedure,Members[Aindex].ClassType);
  Result:=TPasProcedure(Members[Aindex]);
end;

function TTestClassType.GetMF1: TPasFunction;
begin
  AssertNotNull('Have 1 member',Member1);
  AssertEquals('Member 1 is function method',TPasFunction,Member1.ClassType);
  Result:=TPasFunction(Member1);
end;

function TTestClassType.GetP1: TPasProperty;
begin
  AssertNotNull('Have 1 member',Member1);
  AssertEquals('Member 1 is property',TPasProperty,Member1.ClassType);
  Result:=TPasProperty(Member1);
end;

function TTestClassType.GetP2: TPasProperty;
begin
  AssertNotNull('Have 2 members',Members[1]);
  AssertEquals('Member 1 is property',TPasProperty,Members[1].ClassType);
  Result:=TPasProperty(Members[1]);
end;

function TTestClassType.GetT(AIndex: Integer): TPasType;
begin
  AssertNotNull('Have member '+IntToStr(AIndex),Members[AIndex]);
  if not (Members[AIndex] is TPasType) then
    Fail('Member '+IntToStr(AIndex)+' is not a type');
  Result:=TPasType(Members[AIndex]);
end;

function TTestClassType.GetF1: TPasVariable;
begin
  AssertNotNull('Have 1 member',Member1);
  AssertEquals('Member 1 is field',TPasVariable,Member1.ClassType);
  Result:=TPasVariable(Member1);
end;

function TTestClassType.GetC(AIndex: Integer): TPasConst;
begin
  AssertNotNull('Have member '+IntToStr(AIndex),Members[AIndex]);
  if not (Members[AIndex] is TPasConst) then
    Fail('Member '+IntToStr(AIndex)+' is not a const');
  Result:=TPasConst(Members[AIndex]);
end;

procedure TTestClassType.StartClass(AncestorName: String; InterfaceList: String = ''; aClassType : TClassDeclType = cdtClass);

Var
  S : String;
begin
  if FStarted then
    Fail('TTestClassType.StartClass already started');
  FStarted:=True;
  case aClassType of
  cdtObjCClass:
    begin
    FDecl.Add('{$modeswitch objectivec1}');
    S:='TMyClass = ObjCClass';
    end;
  cdtObjCCategory:
    begin
    FDecl.Add('{$modeswitch objectivec1}');
    S:='TMyClass = ObjCCategory(aParent)';
    end;
  else
    S:='TMyClass = Class';
  end;
  if (AncestorName<>'') then
    begin
    S:=S+'('+AncestorName;
    if (InterfaceList<>'') then
      S:=S+','+InterfaceList;
    S:=S+')';
    end;
  FDecl.Add(S);
  FParent:=AncestorName;
end;

procedure TTestClassType.StartExternalClass(AParent: String; AExternalName,
  AExternalNameSpace: String);

Var
  S : String;

begin
  FStarted:=True;
  S:=Format('TMyClass = Class external ''%s'' name ''%s'' ',[AExternalNameSpace,AExternalName]);
  if (AParent<>'') then
    S:=S+'('+AParent+')';
  FDecl.Add(S);
  FParent:=AParent;
end;

procedure TTestClassType.StartClassHelper(ForType: String; AParent: String);
Var
  S : String;
begin
  FStarted:=True;
  S:='TMyClass = Class Helper';
  if (AParent<>'') then
    begin
    S:=S+'('+AParent;
    S:=S+')';
    end;
  S:=S+' for '+ForType;
  FDecl.Add(S);
  FParent:=AParent;
end;

procedure TTestClassType.StartInterface(AParent: String; UUID: String;
  Disp: Boolean = False; UseObjcClass : Boolean = False; UseExternal : Boolean = False);
Var
  S : String;
begin
  FStarted:=True;
  if UseObjCClass then
    begin
    FDecl.Add('{$modeswitch objectivec1}');
    S:='TMyClass = objcprotocol';
    if UseExternal then
      S:=S+' external name ''abc'' ';
    end
  else if Disp then
    S:='TMyClass = DispInterface'
  else
    S:='TMyClass = Interface';
  if (AParent<>'') then
    S:=S+' ('+AParent+')';
  if (UUID<>'') then
    S:=S+' ['''+UUID+''']';
  FDecl.Add(S);
  FParent:=AParent;
end;

procedure TTestClassType.StartRecordHelper(ForType: String; AParent: String);
Var
  S : String;
begin
  FStarted:=True;
  S:='TMyClass = Record Helper';
  if (AParent<>'') then
    begin
    S:=S+'('+AParent;
    S:=S+')';
    end;
  S:=S+' for '+ForType;
  FDecl.Add(S);
  FParent:=AParent;
end;

procedure TTestClassType.StartVisibility(A: TPasMemberVisibility);
begin
  if not FStarted then
    StartClass;
  FDecl.Add('  '+VisibilityNames[A]);
end;

procedure TTestClassType.EndClass(AEnd: String);
begin
  if FEnded then exit;
  if not FStarted then
    StartClass;
  FEnded:=True;
  if (AEnd<>'') then
    FDecl.Add('  '+AEnd);
end;

procedure TTestClassType.AddMember(S: String);
begin
  if Not FStarted then
    StartClass;
  FDecl.Add('    '+S+';');
end;

procedure TTestClassType.ParseClass;

begin
  DoParseClass(False);
end;

procedure TTestClassType.ParseClassFail(Msg: string; MsgNumber: integer);
var
  ok: Boolean;
begin
  ok:=false;
  try
    ParseClass;
  except
    on E: EParserError do
      begin
      AssertEquals('Expected {'+Msg+'}, but got msg {'+Parser.LastMsg+'}',MsgNumber,Parser.LastMsgNumber);
      ok:=true;
      end;
  end;
  AssertEquals('Missing parser error {'+Msg+'} ('+IntToStr(MsgNumber)+')',true,ok);
end;

procedure TTestClassType.DoParseClass(FromSpecial: Boolean);
var
  AncestorType: TPasType;
begin
  EndClass;
  Add('Type');
  if AddComment then
    begin
    Add('// A comment');
    Engine.NeedComments:=True;
    end;
  Add('  '+TrimRight(FDecl.Text)+';');
  ParseDeclarations;
  AssertEquals('One class type definition',1,Declarations.Classes.Count);
  AssertEquals('First declaration is type definition.',TPasClassType,TObject(Declarations.Classes[0]).ClassType);
  FClass:=TObject(Declarations.Classes[0]) as TPasClassType;
  TheType:=FClass; // So assertcomment can get to it
  if (FParent<>'') then
     begin
     AssertNotNull('Have parent class',TheClass.AncestorType);
     if FromSpecial then
       begin
       AncestorType:=TheClass.AncestorType;
       if AncestorType is TPasSpecializeType then
         begin
         AncestorType:=TPasSpecializeType(AncestorType).DestType;
         AssertEquals('Parent class',TPasUnresolvedTypeRef,AncestorType.ClassType);
         end
       else
         AssertEquals('Parent class',TPasClassType,AncestorType.ClassType);
       end
     else
       begin
       AssertEquals('Parent class',TPasUnresolvedTypeRef,TheClass.AncestorType.ClassType);
       AssertEquals('Parent class name',FParent,TPasUnresolvedTypeRef(TheClass.AncestorType).Name);
       end;
     end;
  if (TheClass.ObjKind<>okInterface) then
    AssertNull('No interface, No GUID',TheClass.GUIDExpr);
  if (Not (TheClass.ObjKind in [okClassHelper,okRecordHelper])) then
    AssertNull('No helperfortype if not helper',TheClass.HelperForType);
  if TheClass.Members.Count>0 then
    FMember1:=TObject(TheClass.Members[0]) as TPaselement;
end;

procedure TTestClassType.SetUp;
begin
  inherited SetUp;
  FDecl:=TStringList.Create;
  FClass:=Nil;
  FParent:='';
  FStarted:=False;
end;

procedure TTestClassType.TearDown;
begin
  FClass:=Nil;
  FreeAndNil(FDecl);
  inherited TearDown;
end;

procedure TTestClassType.AssertVisibility(V: TPasMemberVisibility;
  Member: TPasElement);
begin
  If Member=Nil then
    Member:=Member1;
  AssertNotNull('Cannot get visibility of null member',Member);
  AssertEquals('Visibility of '+Member.Name,V,Member.Visibility);
end;

procedure TTestClassType.AssertMemberType(AType: TClass; Member: TPaselement);
begin
  If Member=Nil then
    Member:=Member1;
  AssertEquals('Member '+Member.Name+' type',AType,Member.ClassType);
end;

procedure TTestClassType.AssertMemberName(AName: string; Member: TPaselement);
begin
  If Member=Nil then
    Member:=Member1;
  AssertEquals('Member name ',AName,Member.Name)
end;

procedure TTestClassType.AssertProperty(P: TPasProperty;
  AVisibility: TPasMemberVisibility; AName, ARead, AWrite, AStored,
  AImplements: String; AArgCount: Integer; ADefault, ANodefault: Boolean);
begin
  AssertEquals('Property Name',AName,P.Name);
  AssertEquals(P.Name+': Visibility',AVisibility,P.Visibility);
  Assertequals(P.Name+': No args',AArgCount,P.Args.Count);
  Assertequals(P.Name+': Read accessor',ARead,P.ReadAccessorName);
  Assertequals(P.Name+': Write accessor',AWrite,P.WriteAccessorName);
  Assertequals(P.Name+': implements name',AImplements,P.ImplementsName);
  Assertequals(P.Name+': stored accessor name',AStored,P.StoredAccessorName);
  Assertequals(P.Name+': default',ADefault,P.IsDefault);
  Assertequals(P.Name+': nodefault',ANodefault,P.IsNoDefault);
end;

procedure TTestClassType.TestEmpty;
begin
  EndClass('');
  ParseClass;
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestEmptyComment;
begin
  AddComment:=True;
  TestEmpty;
  AssertComment;
end;

procedure TTestClassType.TestEmptyDeprecated;
begin
  EndClass('end deprecated');
  ParseClass;
  AssertEquals('No members',0,TheClass.Members.Count);
  HaveHint(hDeprecated,Theclass.Hints);
end;

procedure TTestClassType.TestEmptyEnd;
begin
  ParseClass;
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestEmptyEndNoParent;
begin
  StartClass('','');
  ParseClass;
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestEmptyObjC;
begin
  StartClass('','',cdtObjCClass);
  ParseClass;
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
end;

procedure TTestClassType.TestEmptyObjCCategory;
begin
  StartClass('','',cdtObjCCategory);
  ParseClass;
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertEquals('Is interface',okObjcCategory,TheClass.ObjKind);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
end;

procedure TTestClassType.TestOneInterface;
begin
  StartClass('TObject','ISomething');
  ParseClass;
  AssertEquals('Have 1 interface',1,TheClass.Interfaces.Count);
  AssertNotNull('Correct class',TheClass.Interfaces[0]);
  AssertEquals('Correct class',TPasUnresolvedTypeRef,TObject(TheClass.Interfaces[0]).ClassType);
  AssertEquals('Interface name','ISomething',TPasUnresolvedTypeRef(TheClass.Interfaces[0]).Name);
end;

procedure TTestClassType.TestTwoInterfaces;
begin
  StartClass('TObject','ISomething, ISomethingElse');
  ParseClass;
  AssertEquals('Have 2 interface',2,TheClass.Interfaces.Count);
  AssertNotNull('Correct class',TheClass.Interfaces[0]);
  AssertEquals('Correct class',TPasUnresolvedTypeRef,TObject(TheClass.Interfaces[0]).ClassType);
  AssertEquals('Interface name','ISomething',TPasUnresolvedTypeRef(TheClass.Interfaces[0]).Name);
  AssertNotNull('Correct class',TheClass.Interfaces[1]);
  AssertEquals('Correct class',TPasUnresolvedTypeRef,TObject(TheClass.Interfaces[1]).ClassType);
  AssertEquals('Interface name','ISomethingElse',TPasUnresolvedTypeRef(TheClass.Interfaces[1]).Name);
end;

procedure TTestClassType.AssertGenericClass(C : TPasClassType);

begin
  AssertEquals('Parent class name is empty','',C.Name);
  AssertNotNull('Have ancestor type',C.AncestorType);
  AssertEquals('Have ancestor type name','TMyList',C.AncestorType.Name);
  AssertNotNull('Have generic template types',C.GenericTemplateTypes);
  AssertEquals('Have generic template types',1,C.GenericTemplateTypes.Count);
  AssertEquals('Class name ',TPasGenericTemplateType,TObject(C.GenericTemplateTypes[0]).ClassType);
  AssertEquals('Have generic template types','Integer',TPasElement(C.GenericTemplateTypes[0]).Name);
end;

procedure TTestClassType.AssertSpecializedClass(C: TPasSpecializeType);
begin
  AssertEquals('Parent class name is empty','',C.Name);
  AssertNotNull('Have dest type',C.DestType);
  AssertEquals('Have dest type name','TMyList',C.DestType.Name);
  AssertNotNull('Have param types',C.Params);
  AssertEquals('Have one param type',1,C.Params.Count);
  AssertNotNull('First Param ',C.Params[0]);
  AssertEquals('First Param unresolvedtype',TPasUnresolvedTypeRef,TObject(C.Params[0]).ClassType);
  AssertEquals('Has specialize param integer','Integer',TPasUnresolvedTypeRef(C.Params[0]).Name);
end;

procedure TTestClassType.TestOneSpecializedClass;

Var
  C : TPasSpecializeType;

begin
  StartClass('Specialize TMyList<Integer>','');
  DoParseClass(True);
  C:=TPasSpecializeType(TheClass.AncestorType);
  AssertSpecializedClass(C);
end;

procedure TTestClassType.TestOneSpecializedClassInterface;
Var
  C : TPasSpecializeType;

begin
  StartClass('Specialize TMyList<Integer>','ISomething');
  DoParseClass(True);
  C:=TPasSpecializeType(TheClass.AncestorType);
  AssertSpecializedClass(C);
  AssertEquals('Have 1 interface',1,TheClass.Interfaces.Count);
  AssertNotNull('Correct class',TheClass.Interfaces[0]);
  AssertEquals('Correct class',TPasUnresolvedTypeRef,TObject(TheClass.Interfaces[0]).ClassType);
  AssertEquals('Interface name','ISomething',TPasUnresolvedTypeRef(TheClass.Interfaces[0]).Name);
end;

procedure TTestClassType.TestOneField;
begin
  AddMember('a : integer');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('a');
  AssertVisibility;
end;

procedure TTestClassType.TestOneFieldStatic;
begin
  AddMember('a : integer; static');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('a');
  AssertVisibility;
  AssertTrue('Have static field',vmStatic in TPasVariable(Field1).VarModifiers);
end;

procedure TTestClassType.TestOneHelperField;
begin
  AddMember('helper : integer');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('helper');
  AssertVisibility;
end;

procedure TTestClassType.TestOneFieldComment;
begin
  AddComment:=true;
  AddMember('{c}a : integer');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertEquals('field comment','c'+sLineBreak,Field1.DocComment);
  AssertVisibility;
end;

procedure TTestClassType.TestOneVarField;
begin
  StartVisibility(visPublished);
  FDecl.Add('var');
  AddMember('a : integer');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('a');
  AssertVisibility(visPublished);
end;

procedure TTestClassType.TestOneClassField;
begin
  StartVisibility(visPublished);
  FDecl.Add('class var');
  AddMember('a : integer');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('a');
  AssertVisibility(visPublished);
  if not (vmClass in Field1.VarModifiers) then
     Fail('Field is not a class field');
end;

procedure TTestClassType.TestOneFieldVisibility;
begin
  StartVisibility(visPublished);
  AddMember('a : integer');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('a');
  AssertVisibility(visPublished);
end;

procedure TTestClassType.TestOneFieldDeprecated;
begin
  AddMember('a : integer deprecated');
  ParseClass;
  AssertNotNull('Have 1 field',Field1);
  AssertMemberName('a');
  HaveHint(hDeprecated,Member1.Hints);
  AssertVisibility;
end;

procedure TTestClassType.TestTwoFields;
begin
  AddMember('a : integer');
  AddMember('b : integer');
  ParseClass;
  AssertEquals('2 members',2,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('a');
  AssertVisibility;
  AssertNotNull('Have field',Members[1]);
  AssertMemberName('b',Members[1]);
  AssertMemberType(TPasVariable,Members[1]);
  AssertVisibility(visDefault,Members[1]);
end;

procedure TTestClassType.TestTwoFieldsB;
begin
  AddMember('a,b : integer');
  ParseClass;
  AssertEquals('2 members',2,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('a');
  AssertVisibility;
  AssertNotNull('Have field',Members[1]);
  AssertMemberName('b',Members[1]);
  AssertMemberType(TPasVariable,Members[1]);
  AssertVisibility(visDefault,Members[1]);
end;

procedure TTestClassType.TestTwoVarFieldsB;
begin
  StartVisibility(visPublic);
  FDecl.Add('var');
  AddMember('a,b : integer');
  ParseClass;
  AssertEquals('2 members',2,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('a');
  AssertVisibility(vispublic);
  AssertNotNull('Have field',Members[1]);
  AssertMemberName('b',Members[1]);
  AssertMemberType(TPasVariable,Members[1]);
  AssertVisibility(visPublic,Members[1]);
end;

procedure TTestClassType.TestNoVarFields;

begin
  StartVisibility(visPublic);
  FDecl.Add('var');
  AddMember('Function b : integer');
  ParseClass;
  AssertEquals('member count',1,TheClass.members.Count);
  AssertNotNull('Have function',Members[0]);
  AssertMemberName('b',Members[0]);
  AssertMemberType(TPasFunction,Members[0]);
  AssertVisibility(visPublic,Members[0]);
end;

procedure TTestClassType.TestVarClassFunction;
begin
  StartVisibility(visPublic);
  FDecl.Add('var');
  AddMember('class Function b : integer');
  ParseClass;
  AssertEquals('member count',1,TheClass.members.Count);
  AssertNotNull('Have function',Members[0]);
  AssertMemberName('b',Members[0]);
  AssertMemberType(TPasClassFunction,Members[0]);
  AssertVisibility(visPublic,Members[0]);
end;

procedure TTestClassType.TestClassVarClassFunction;
begin
  StartVisibility(visPublic);
  FDecl.Add('class var');
  AddMember('class Function b : integer');
  ParseClass;
  AssertEquals('member count',1,TheClass.members.Count);
  AssertNotNull('Have function',Members[0]);
  AssertMemberName('b',Members[0]);
  AssertMemberType(TPasClassFunction,Members[0]);
  AssertVisibility(visPublic,Members[0]);
end;

procedure TTestClassType.TestTwoFieldsVisibility;
begin
  StartVisibility(visPublic);
  AddMember('a,b : integer');
  ParseClass;
  AssertEquals('2 members',2,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('a');
  AssertVisibility(vispublic);
  AssertNotNull('Have field',Members[1]);
  AssertMemberName('b',Members[1]);
  AssertMemberType(TPasVariable,Members[1]);
  AssertVisibility(visPublic,Members[1]);
end;

procedure TTestClassType.TestConstProtectedEnd;
begin
  // After bug report 25720
   StartVisibility(visPrivate);
   AddMember('fmy : Integer');
   StartVisibility(visProtected);
   AddMember('fmy : Integer');
   FDecl.Add('protected const');
   FDecl.Add('cconst = 10;');
   StartVisibility(visProtected);
   AddMember('I : Integer');
   ParseClass;
end;

procedure TTestClassType.TestTypeProtectedEnd;
begin
  // After bug report 25720
   StartVisibility(visPrivate);
   AddMember('fmy : Integer');
   StartVisibility(visProtected);
   AddMember('fmy : Integer');
   FDecl.Add('protected type');
   FDecl.Add('mytype = integer;');
   StartVisibility(visProtected);
   AddMember('I : Integer');
   ParseClass;
end;

procedure TTestClassType.TestVarProtectedEnd;
begin
  // After bug report 25720
   StartVisibility(visPrivate);
   AddMember('fmy : Integer');
   StartVisibility(visProtected);
   AddMember('fmy : Integer');
   FDecl.Add('protected var');
   FDecl.Add('mytype : integer;');
   StartVisibility(visProtected);
   AddMember('I : Integer');
   ParseClass;
end;

procedure TTestClassType.TestHintFieldDeprecated;
begin
  AddMember('deprecated : integer');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('deprecated');
end;

procedure TTestClassType.TestHintFieldPlatform;
begin
  AddMember('platform : integer');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('platform');
end;

procedure TTestClassType.TestHintFieldLibraryError;
begin
  AddMember('library: integer');
  AssertException(EParserError,@ParseClass);
end;

procedure TTestClassType.TestHintFieldExperimental;
begin
  AddMember('experimental: integer');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('experimental');
end;

procedure TTestClassType.TestHintFieldUninmplemented;
begin
  AddMember('unimplemented: integer');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('unimplemented');
end;

procedure TTestClassType.TestOneVarFieldExternalName;
begin
  Parser.CurrentModeswitches:=Parser.CurrentModeswitches+[msExternalClass];
  StartExternalClass('','myname','');
  AddMember('unimplemented: integer external name ''uni''');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('unimplemented');
end;

procedure TTestClassType.TestOneVarFieldExternalNameSemicolon;
begin
  Parser.CurrentModeswitches:=Parser.CurrentModeswitches+[msExternalClass];
  StartExternalClass('','myname','');
  AddMember('unimplemented: integer; external name ''uni''');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertNotNull('Have field',Field1);
  AssertMemberName('unimplemented');
end;

procedure TTestClassType.TestMethodSimple;
begin
  AddMember('Procedure DoSomething');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertNotNull('Have method',Method1);
  AssertMemberName('DoSomething');
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertNotNull('Method proc type',Method1.ProcType);
  AssertEquals('No arguments',0,Method1.ProcType.Args.Count)
end;

procedure TTestClassType.TestMethodSimpleComment;
begin
  AddComment:=True;
  AddMember('{c} Procedure DoSomething');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertNotNull('Have method',Method1);
  AssertMemberName('DoSomething');
  AssertEquals('Comment','c'+sLineBreak,Method1.DocComment);
end;

procedure TTestClassType.TestMethodWithDotFails;
begin
  AddMember('Procedure DoSomething.Stupid');
  ParseClassFail('Expected ";"',nParserExpectTokenError);
end;

procedure TTestClassType.TestMethodWithDotOK;

begin
  AddMember('Procedure DoSomething.Stupid=me');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('1 method resolution procedure',TPasMethodResolution,members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertNotNull('1 method resolution procedure',TPasMethodResolution(members[0]).ImplementationProc);
end;

procedure TTestClassType.TestMethodFunctionWithDotOK;
begin
  AddMember('Function DoSomething.Stupid=me');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('1 method resolution procedure',TPasMethodResolution,members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertNotNull('1 method resolution procedure',TPasMethodResolution(members[0]).ImplementationProc);
end;

procedure TTestClassType.TestNoSemicolon;
begin
  StartClass;
  fDecl.Add('Y : String');
  ParseClass;
end;

procedure TTestClassType.TestClassMethodSimple;

begin
  AddMember('Class Procedure DoSomething');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('1 class procedure',TPasClassProcedure,members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertMemberName('DoSomething');
  AssertEquals('No modifiers',[],TPasClassProcedure(Members[0]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasClassProcedure(Members[0]).ProcType.CallingConvention);
  AssertNotNull('Method proc type',TPasClassProcedure(Members[0]).ProcType);
  AssertEquals('No arguments',0,TPasClassProcedure(Members[0]).ProcType.Args.Count)
end;

procedure TTestClassType.TestClassMethodSimpleComment;

begin
  AddComment:=True;
  AddMember('{c} Class Procedure DoSomething');
  ParseClass;
  AssertEquals('Comment','c'+sLineBreak,Members[0].DocComment);
end;

procedure TTestClassType.TestConstructor;
begin
  AddMember('Constructor Create');
  ParseClass;
  AssertEquals('1 members',1,TheClass.Members.Count);
  AssertEquals('1 class procedure',TPasConstructor,Members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertMemberName('Create');
  AssertEquals('No modifiers',[],TPasConstructor(Members[0]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasConstructor(Members[0]).ProcType.CallingConvention);
  AssertNotNull('Method proc type',TPasConstructor(Members[0]).ProcType);
  AssertEquals('No arguments',0,TPasConstructor(Members[0]).ProcType.Args.Count)
end;

procedure TTestClassType.TestClassConstructor;
begin
  AddMember('Class Constructor Create');
  ParseClass;
  AssertEquals('1 members',1,TheClass.Members.Count);
  AssertEquals('1 class procedure',TPasClassConstructor,Members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertMemberName('Create');
  AssertEquals('No modifiers',[],TPasClassConstructor(Members[0]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasClassConstructor(Members[0]).ProcType.CallingConvention);
  AssertNotNull('Method proc type',TPasClassConstructor(Members[0]).ProcType);
  AssertEquals('No arguments',0,TPasClassConstructor(Members[0]).ProcType.Args.Count)
end;

procedure TTestClassType.TestDestructor;
begin
  AddMember('Destructor Destroy');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('1 class procedure',TPasDestructor,members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertMemberName('Destroy');
  AssertEquals('No modifiers',[],TPasDestructor(Members[0]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasDestructor(Members[0]).ProcType.CallingConvention);
  AssertNotNull('Method proc type',TPasDestructor(Members[0]).ProcType);
  AssertEquals('No arguments',0,TPasDestructor(Members[0]).ProcType.Args.Count)
end;

procedure TTestClassType.TestClassDestructor;
begin
  AddMember('Class Destructor Destroy');
  ParseClass;
  AssertEquals('1 members',1,TheClass.Members.Count);
  AssertEquals('1 class procedure',TPasClassDestructor,Members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertMemberName('Destroy');
  AssertEquals('No modifiers',[],TPasClassDestructor(Members[0]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasClassDestructor(Members[0]).ProcType.CallingConvention);
  AssertNotNull('Method proc type',TPasClassDestructor(Members[0]).ProcType);
  AssertEquals('No arguments',0,TPasClassDestructor(Members[0]).ProcType.Args.Count)
end;

procedure TTestClassType.TestFunctionMethodSimple;
begin
  AddMember('Function DoSomething : integer');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('Default visibility',visDefault,FunctionMethod1.Visibility);
  AssertNotNull('Have method',Member1);
  AssertMemberName('DoSomething');
  AssertEquals('No modifiers',[],functionMethod1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, functionMethod1.ProcType.CallingConvention);
  AssertNotNull('Method proc type',functionMethod1.ProcType);
  AssertEquals('No arguments',0,functionMethod1.ProcType.Args.Count)
end;

procedure TTestClassType.TestClassFunctionMethodSimple;
begin
  AddMember('Class Function DoSomething : integer');
  ParseClass;
  AssertEquals('1 members',1,TheClass.members.Count);
  AssertEquals('1 class procedure',TPasClassFunction,members[0].ClassType);
  AssertEquals('Default visibility',visDefault,Members[0].Visibility);
  AssertMemberName('DoSomething');
  AssertEquals('No modifiers',[],TPasClassFunction(members[0]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasClassFunction(members[0]).ProcType.CallingConvention);
  AssertNotNull('Method proc type',TPasClassFunction(members[0]).ProcType);
  AssertEquals('No arguments',0,TPasClassFunction(members[0]).ProcType.Args.Count)
end;

procedure TTestClassType.DefaultMethod;

begin
  if TheClass.members.Count<1 then
    Fail('No members for method');
  AssertNotNull('Have method',Method1);
  AssertNotNull('Method proc type',Method1.ProcType);
    AssertMemberName('DoSomething');
  AssertEquals('1 argument',1,Method1.ProcType.Args.Count) ;
  AssertEquals('Argument name','A',TPasVariable(Method1.ProcType.Args[0]).Name);
end;

procedure TTestClassType.AssertParserError(const Msg: String);
begin
  AssertException(Msg,EParserError,@ParseClass)
end;

procedure TTestClassType.TestMethodOneArg;
begin
  AddMember('Procedure DoSomething(A : Integer)');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodVirtual;
begin
  AddMember('Procedure DoSomething(A : Integer) virtual');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Virtual modifiers',[pmVirtual],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodVirtualSemicolon;
begin
  AddMember('Procedure DoSomething(A : Integer); virtual');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Virtual modifiers',[pmVirtual],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodVirtualAbstract;
begin
  AddMember('Procedure DoSomething(A : Integer) virtual abstract');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Virtual, abstract modifiers',[pmVirtual,pmAbstract],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodVirtualAbstractFinal;
begin
  AddMember('Procedure DoSomething(A : Integer) virtual; abstract; final');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Virtual, abstract modifiers',[pmVirtual,pmAbstract,pmFinal],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;


procedure TTestClassType.TestMethodOverride;
begin
  AddMember('Procedure DoSomething(A : Integer) override');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Override modifiers',[pmoverride],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodReintroduce;
begin
  AddMember('Procedure DoSomething(A : Integer) ReIntroduce');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Reintroduce modifiers',[pmReintroduce],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodDynamic;
begin
  AddMember('Procedure DoSomething(A : Integer) dynamic');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Dynamic modifiers',[pmDynamic],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodInline;
begin
  AddMember('Procedure DoSomething(A : Integer) inline');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Inline modifiers',[pmInline],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodVisibility;
begin
  StartVisibility(visPublic);
  AddMember('Procedure DoSomething(A : Integer)');
  ParseClass;
  DefaultMethod;
  AssertEquals('Public visibility',visPublic,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodSVisibility;
begin
  AddMember('Procedure DoSomething(A : Integer)');
  StartVisibility(visPublic);
  AddMember('Procedure DoSomethingB(A : Integer)');
  ParseClass;
  DefaultMethod;
  AssertEquals('First Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertNotNull('Have method 2',Method2);
  AssertEquals('Second Default visibility',visPublic,Method2.Visibility);
  AssertNotNull('Method proc type',Method2.ProcType);
  AssertMemberName('DoSomethingB',Method2);
  AssertEquals('1 argument',1,Method2.ProcType.Args.Count) ;
  AssertEquals('Argument name','A',TPasVariable(Method2.ProcType.Args[0]).Name);
end;

procedure TTestClassType.TestMethodOverloadVisibility;
begin
  AddMember('Procedure DoSomething(A : Integer)');
  StartVisibility(visPublic);
  AddMember('Procedure DoSomething(A : String)');
  ParseClass;
  AssertNotNull('Have member 1',Member1);
  AssertEquals('Overload',TPasOverloadedProc,Member1.ClassType);
  AssertEquals('Default visibility',visDefault,Member1.Visibility);
end;

procedure TTestClassType.TestMethodHint;
begin
  AddMember('Procedure DoSomething(A : Integer) deprecated');
  ParseClass;
  DefaultMethod;
  HaveHint(hDeprecated,Member1.Hints);
  HaveHint(hDeprecated,Method1.ProcType.Hints);
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestMethodVirtualHint;
begin
  AddMember('Procedure DoSomething(A : Integer) virtual; deprecated');
  ParseClass;
  DefaultMethod;
  HaveHint(hDeprecated,Member1.Hints);
  HaveHint(hDeprecated,Method1.ProcType.Hints);
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('virtual modifiers',[pmVirtual],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestIntegerMessageMethod;
begin
  AddMember('Procedure DoSomething(A : Integer) message 123');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('message modifier',[pmMessage],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertEquals('Message name','123',Method1.MessageName);
end;

procedure TTestClassType.TestStringMessageMethod;
begin
  AddMember('Procedure DoSomething(A : Integer) message ''aha''');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('message modifiers',[pmMessage],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertEquals('Message name','''aha''',Method1.MessageName);
end;

procedure TTestClassType.Test2Methods;
begin
  AddMember('Procedure DoSomething(A : Integer) virtual');
  AddMember('Procedure DoSomethingElse');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Virtual modifiers',[pmVirtual],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertEquals('Default visibility',visDefault,Members[1].Visibility);
  AssertEquals('Default visibility',TPasProcedure,Members[1].ClassType);
  AssertEquals('Virtual modifiers',[],TPasProcedure(Members[1]).Modifiers);
  AssertEquals('Default calling convention',ccDefault, TPasProcedure(Members[1]).ProcType.CallingConvention);
end;

procedure TTestClassType.Test2MethodsDifferentVisibility;
begin
  AddMember('Procedure DoSomething(A : Integer) virtual');
  StartVisibility(visPublic);
  AddMember('Procedure DoSomethingElse');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('Virtual modifiers',[pmVirtual],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertEquals('2 Public visibility',visPublic,Members[1].Visibility);
  AssertEquals('2 Default visibility',TPasProcedure,Members[1].ClassType);
  AssertEquals('2 No modifiers',[],TPasProcedure(Members[1]).Modifiers);
  AssertEquals('2 Default calling convention',ccDefault, TPasProcedure(Members[1]).ProcType.CallingConvention);

end;

procedure TTestClassType.TestPropertyRedeclare;
begin
  StartVisibility(visPublished);
  AddMember('Property Something');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','','','','',0,False,False);
  AssertNull('No type',Property1.VarType);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyRedeclareComment;
begin
  StartVisibility(visPublished);
  AddComment:=True;
  AddMember('{p} Property Something');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','','','','',0,False,False);
  AssertEquals('comment','p'+sLineBreak,Property1.DocComment);
end;

procedure TTestClassType.TestPropertyRedeclareDefault;
begin
  StartVisibility(visPublic);
  AddMember('Property Something; default');
  ParseClass;
  AssertProperty(Property1,visPublic,'Something','','','','',0,True,False);
  AssertNull('No type',Property1.VarType);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No default value','',Property1.DefaultValue);
  // Actually, already tested in AssertProperty
  AssertEquals('Is default property',True, Property1.IsDefault);
end;

procedure TTestClassType.TestPropertyReadOnly;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Read FSomething');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyReadWrite;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Read FSomething Write FSomething');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','FSomething','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyWriteOnly;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Write FSomething');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','','FSomething','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyDefault;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Read FSomething Write FSomething default 1');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','FSomething','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertExpression('Default expression',Property1.DefaultExpr,pekNumber,'1');
  Assertequals('Default value','1',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyNoDefault;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Read FSomething Write FSomething nodefault');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','FSomething','','',0,False,True);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyIndex;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Index 2 Read GetF Write SetF');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','GetF','SetF','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertExpression('Index expression',Property1.IndexExpr,pekNumber,'2');
  Assertequals('index','2',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyStored;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Read GetF Write SetF Stored CheckStored');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','GetF','SetF','CheckStored','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyStoredFalse;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : integer Read GetF Write SetF Stored False');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','GetF','SetF','False','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyFullyQualifiedType;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : unita.typeb Read FSomething');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','unita.typeb',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyArrayReadOnly;
Var
  A : TPasArgument;
begin
  StartVisibility(visPublished);
  AddMember('Property Somethings[AIndex : Integer] : integer Read GetF');
  ParseClass;
  AssertProperty(Property1,visPublished,'Somethings','GetF','','','',1,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertEquals('Argument class',TPasArgument,TObject(Property1.Args[0]).ClassType);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
  // Argument
  A:=TPasArgument(Property1.Args[0]);
  AssertEquals('Argument name','AIndex',A.Name);
  AssertNotNull('Argument class', A.ArgType);
  AssertEquals('Argument class type',TPasUnresolvedTypeRef,A.ArgType.ClassType);
  AssertEquals('Argument class type name','Integer',A.ArgType.Name);
end;

procedure TTestClassType.TestPropertyArrayReadWrite;
Var
  A : TPasArgument;
begin
  StartVisibility(visPublished);
  AddMember('Property Somethings[AIndex : Integer] : integer Read GetF Write SetF');
  ParseClass;
  AssertProperty(Property1,visPublished,'Somethings','GetF','SetF','','',1,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
  // Argument
  AssertEquals('Argument class',TPasArgument,TObject(Property1.Args[0]).ClassType);
  A:=TPasArgument(Property1.Args[0]);
  AssertEquals('Argument name','AIndex',A.Name);
  AssertNotNull('Argument class', A.ArgType);
  AssertEquals('Argument class type',TPasUnresolvedTypeRef,A.ArgType.ClassType);
  AssertEquals('Argument class type name','Integer',A.ArgType.Name);
end;

procedure TTestClassType.TestPropertyArrayReadOnlyDefault;

Var
  A : TPasArgument;
begin
  StartVisibility(visPublished);
  AddMember('Property Somethings[AIndex : Integer] : integer Read GetF; default');
  ParseClass;
  AssertProperty(Property1,visPublished,'Somethings','GetF','','','',1,True,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
  // Argument
  AssertEquals('Argument class',TPasArgument,TObject(Property1.Args[0]).ClassType);
  A:=TPasArgument(Property1.Args[0]);
  AssertEquals('Argument name','AIndex',A.Name);
  AssertNotNull('Argument class', A.ArgType);
  AssertEquals('Argument class type',TPasUnresolvedTypeRef,A.ArgType.ClassType);
  AssertEquals('Argument class type name','Integer',A.ArgType.Name);
end;

procedure TTestClassType.TestPropertyArrayReadWriteDefault;
Var
  A : TPasArgument;
begin
  StartVisibility(visPublished);
  AddMember('Property Somethings[AIndex : Integer] : integer Read GetF Write SetF; default');
  ParseClass;
  AssertProperty(Property1,visPublished,'Somethings','GetF','SetF','','',1,True,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
  // Argument
  AssertEquals('Argument class',TPasArgument,TObject(Property1.Args[0]).ClassType);
  A:=TPasArgument(Property1.Args[0]);
  AssertEquals('Argument name','AIndex',A.Name);
  AssertNotNull('Argument class', A.ArgType);
  AssertEquals('Argument class type',TPasUnresolvedTypeRef,A.ArgType.ClassType);
  AssertEquals('Argument class type name','Integer',A.ArgType.Name);
end;

procedure TTestClassType.TestPropertyArrayMultiDimReadOnly;
Var
  A : TPasArgument;
begin
  StartVisibility(visPublished);
  AddMember('Property Somethings[ACol : Integer; ARow : Integer] : integer Read GetF; default');
  ParseClass;
  AssertProperty(Property1,visPublished,'Somethings','GetF','','','',2,True,False);
  AssertEquals('Published property',vispublished,Property1.Visibility);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  AssertNull('No Index expression',Property1.IndexExpr);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Default expression',Property1.DefaultExpr);
  Assertequals('No Default value','',Property1.DefaultValue);
  // Argument 1
  AssertEquals('Argument 1 class',TPasArgument,TObject(Property1.Args[0]).ClassType);
  A:=TPasArgument(Property1.Args[0]);
  AssertEquals('Argument 1name','ACol',A.Name);
  AssertNotNull('Argument  1class', A.ArgType);
  AssertEquals('Argument 1 class type',TPasUnresolvedTypeRef,A.ArgType.ClassType);
  AssertEquals('Argument 1 class type name','Integer',A.ArgType.Name);
  // Argument 2
  AssertEquals('Argument 2 class',TPasArgument,TObject(Property1.Args[1]).ClassType);
  A:=TPasArgument(Property1.Args[1]);
  AssertEquals('Argument 2 name','ARow',A.Name);
  AssertNotNull('Argument 2 class', A.ArgType);
  AssertEquals('Argument 2 class type',TPasUnresolvedTypeRef,A.ArgType.ClassType);
  AssertEquals('Argument 2 class type name','Integer',A.ArgType.Name);
end;

procedure TTestClassType.TestPropertyImplements;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : AInterface Read FSomething Implements ISomeInterface');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','','','ISomeInterface',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','AInterface',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);

end;

procedure TTestClassType.TestPropertyDeprecated;

begin
  StartVisibility(visPublished);
  AddMember('Property Something : AInterface Read FSomething; deprecated');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','AInterface',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);
  AssertTrue('Deprecated',[hDeprecated]=Property1.Hints);
end;

procedure TTestClassType.TestPropertyDeprecatedMessage;

begin
  StartVisibility(visPublished);
  AddMember('Property Something : AInterface Read FSomething; deprecated ''this is no longer used'' ');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','AInterface',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);
  AssertTrue('Deprecated',[hDeprecated]=Property1.Hints);
end;

procedure TTestClassType.TestPropertyImplementsFullyQualifiedName;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : AInterface Read FSomething Implements UnitB.ISomeInterface');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FSomething','','','UnitB.ISomeInterface',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','AInterface',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyReadFromRecordField;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : Integer Read FPoint.X');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FPoint.X','','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyReadFromArrayField;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : Integer Read FPoint.W[x].y.Z');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FPoint.W[x].y.Z','','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestPropertyReadWriteFromRecordField;
begin
  StartVisibility(visPublished);
  AddMember('Property Something : Integer Read FPoint.X Write FPoint.X');
  ParseClass;
  AssertProperty(Property1,visPublished,'Something','FPoint.X','FPoint.X','','',0,False,False);
  AssertNotNull('Have type',Property1.VarType);
  AssertEquals('Property type class type',TPasUnresolvedTypeRef,Property1.vartype.ClassType);
  AssertEquals('Property type name','Integer',Property1.vartype.name);
  Assertequals('No index','',Property1.IndexValue);
  AssertNull('No Index expression',Property1.IndexExpr);
  AssertNull('No default expression',Property1.DefaultExpr);
  Assertequals('Default value','',Property1.DefaultValue);
end;

procedure TTestClassType.TestExternalClass;
begin
  StartExternalClass('','myname','mynamespace');
  Parser.CurrentModeswitches:=[msObjfpc,msexternalClass];
  ParseClass;
  AssertTrue('External class ',TheClass.IsExternal);
  AssertEquals('External name space','mynamespace',TheClass.ExternalNameSpace);
  AssertEquals('External name ','myname',TheClass.ExternalName);
end;

procedure TTestClassType.TestExternalClassNoNameSpace;
begin
  FStarted:=True;
  Parser.CurrentModeswitches:=[msObjfpc,msexternalClass];
  FDecl.add('TMyClass = Class external name ''me'' ');
  ParseClass;
  AssertTrue('External class ',TheClass.IsExternal);
  AssertEquals('External name space','',TheClass.ExternalNameSpace);
  AssertEquals('External name ','me',TheClass.ExternalName);
end;

procedure TTestClassType.TestExternalClassNoNameKeyWord;
begin
  FStarted:=True;
  Parser.CurrentModeswitches:=[msObjfpc,msexternalClass];
  FDecl.add('TMyClass = Class external ''name'' ''me'' ');
  AssertException('No name keyword raises error',EParserError,@ParseClass);

end;

procedure TTestClassType.TestExternalClassNoName;
begin
  FStarted:=True;
  Parser.CurrentModeswitches:=[msObjfpc,msexternalClass];
  FDecl.add('TMyClass = Class external ''name'' name ');
  AssertException('No name  raises error',EParserError,@ParseClass);

end;

procedure TTestClassType.TestLocalSimpleType;
begin
  StartVisibility(visPublic);
  FDecl.add('Type');
  AddMember('TDirection = (left,right)');
  AddMember('Procedure Something');
  ParseClass;
  AssertEquals('Local Enumeration type',TPasEnumType, Type1.ClassType);
  AssertEquals('Visibility is correct',VisPublic, Type1.Visibility);
  AssertEquals('Type name','TDirection', Type1.Name);
  AssertSame('Type parent is class',TheClass, Type1.Parent);
  AssertNotNull('Member 2 is procedure',Method2);
  AssertEquals('method name','Something', Method2.Name);
end;

procedure TTestClassType.TestLocalSimpleTypes;
begin
  StartVisibility(visPublic);
  FDecl.add('Type');
  AddMember('TDirection = (left,right)');
  AddMember('TVerticalDirection = (up,down)');
  AddMember('Procedure Something');
  ParseClass;
  AssertEquals('Local Enumeration type',TPasEnumType, Type1.ClassType);
  AssertEquals('Visibility is correct',VisPublic, Type1.Visibility);
  AssertEquals('Type name','TDirection', Type1.Name);
  AssertSame('Type parent is class',TheClass, Type1.Parent);
  AssertEquals('Local Enumeration type',TPasEnumType, Type2.ClassType);
  AssertEquals('Visibility is correct',VisPublic, Type2.Visibility);
  AssertEquals('Type name','TVerticalDirection', Type2.Name);
  AssertSame('Type parent is class',TheClass, Type2.Parent);
  AssertNotNull('Member 2 is procedure',Method3);
  AssertEquals('method name','Something', Method3.Name);
end;

procedure TTestClassType.TestLocalSimpleConst;
begin
  StartVisibility(visPublic);
  FDecl.add('Const');
  AddMember(' A = 23');
  AddMember('Procedure Something');
  ParseClass;
  AssertEquals('Local const value',TPasConst, Const1.ClassType);
  AssertEquals('Visibility is correct',VisPublic, Const1.Visibility);
  AssertEquals('Const name','A', Const1.Name);
  AssertExpression('Const value',Const1.Expr,pekNUmber,'23');
  AssertSame('Const parent is class',TheClass, Const1.Parent);
  AssertNotNull('Member 2 is procedure',Method2);
  AssertEquals('method name','Something', Method2.Name);
end;

procedure TTestClassType.TestLocalSimpleConsts;
begin
  StartVisibility(visPublic);
  FDecl.add('Const');
  AddMember(' A = 23');
  AddMember(' B = 45');
  AddMember('Procedure Something');
  ParseClass;
  // Const A
  AssertEquals('Local const value',TPasConst, Const1.ClassType);
  AssertEquals('Visibility is correct',VisPublic, Const1.Visibility);
  AssertEquals('Const name','A', Const1.Name);
  AssertExpression('Const value',Const1.Expr,pekNUmber,'23');
  AssertSame('Type parent is class',TheClass, Const1.Parent);
  // Const B
  AssertEquals('Local const value',TPasConst, Const2.ClassType);
  AssertEquals('Visibility is correct',VisPublic, Const2.Visibility);
  AssertEquals('Const name','B', Const2.Name);
  AssertExpression('Const value',Const2.Expr,pekNUmber,'45');
  AssertSame('Type parent is class',TheClass, Const2.Parent);
  AssertNotNull('Member 3 is procedure',Method3);
  AssertEquals('method name','Something', Method3.Name);
end;

procedure TTestClassType.TestClassTypeAttributes;
begin
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '  [Black]',
  '  type',
  '    [Red]',
  '    [White]',
  '    TWord = word;',
  '    [Blue]',
  '    [Green]',
  '    TChar = char;',
  '  end;',
  '']);
  ParseDeclarations;
end;

procedure TTestClassType.TestClassConstAttributes;
begin
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '  [Black]',
  '  const',
  '    [Red]',
  '    [White]',
  '    A = 1;',
  '    [Blue]',
  '    [Green]',
  '    B = 2;',
  '  end;',
  '']);
  ParseDeclarations;
end;

procedure TTestClassType.TestClassHelperEmpty;
begin
  StartClassHelper('TOriginal','');
  EndClass();
  ParseClass;
  AssertEquals('Is class helper',okClassHelper,TheClass.ObjKind);
  AssertNotNull('Have helper original',TheClass.HelperForType);
  AssertEquals('Have helper original alias',TPasUnresolvedTypeRef,TheClass.HelperForType.CLassType);
  AssertEquals('Helper original alias name','TOriginal',TheClass.HelperForType.Name);
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestClassHelperParentedEmpty;
begin
  StartClassHelper('TOriginal','TOtherHelper');
  EndClass();
  ParseClass;
  AssertEquals('Is class helper',okClassHelper,TheClass.ObjKind);
  AssertNotNull('Have helper original',TheClass.HelperForType);
  AssertEquals('Have helper original alias',TPasUnresolvedTypeRef,TheClass.HelperForType.CLassType);
  AssertEquals('Helper original alias name','TOriginal',TheClass.HelperForType.Name);
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestClassHelperOneMethod;
begin
  StartClassHelper('TOriginal','');
  AddMember('Procedure DoSomething(A : Integer)');
  ParseClass;
  AssertEquals('Is class helper',okClassHelper,TheClass.ObjKind);
  AssertNotNull('Have helper original',TheClass.HelperForType);
  AssertEquals('Have helper original alias',TPasUnresolvedTypeRef,TheClass.HelperForType.CLassType);
  AssertEquals('Helper original alias name','TOriginal',TheClass.HelperForType.Name);
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestInterfaceEmpty;
begin
  StartInterface('','');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestObjcProtocolEmpty;
begin
  StartInterface('','',False,True);
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okObjcProtocol,TheClass.ObjKind);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestObjcProtocolEmptyExternal;
begin
  StartInterface('','',False,True,true);
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okObjcProtocol,TheClass.ObjKind);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestObjcProtocolMultiParent;
begin
  StartInterface('A, B','',False,True,true);
  FParent:='A';
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okObjcProtocol,TheClass.ObjKind);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
  AssertEquals('Have 1 interface',1,TheClass.Interfaces.Count);
  AssertNotNull('Correct class',TheClass.Interfaces[0]);
  AssertEquals('Correct class',TPasUnresolvedTypeRef,TObject(TheClass.Interfaces[0]).ClassType);
  AssertEquals('Interface name','B',TPasUnresolvedTypeRef(TheClass.Interfaces[0]).Name);
end;

procedure TTestClassType.TestObjcProtocolOptional;
begin
  StartInterface('','',False,True);
  FDecl.Add('    optional');
  AddMember('Procedure DoSomething(A : Integer)');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okObjcProtocol,TheClass.ObjKind);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
  AssertEquals('No members',1,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestObjcProtocolRequired;
begin
  StartInterface('','',False,True);
  FDecl.Add('    required');
  AddMember('Procedure DoSomething(A : Integer)');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okObjcProtocol,TheClass.ObjKind);
  AssertTrue('Is objectivec',TheClass.IsObjCClass);
  AssertEquals('No members',1,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestInterfaceDisp;

begin
  StartInterface('','',true);
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okDispInterface,TheClass.ObjKind);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestInterfaceParentedEmpty;
begin
  StartInterface('IInterface','');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestInterfaceOneMethod;
begin
  StartInterface('IInterface','');
  AddMember('Procedure DoSomething(A : Integer)');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertNull('No UUID',TheClass.GUIDExpr);
end;

procedure TTestClassType.TestInterfaceDispIDMethod;

begin
  StartInterface('IInterface','');
  AddMember('Procedure DoSomething(A : Integer) dispid 12');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('dispid modifier',[pmDispID],Method1.Modifiers);
  AssertNotNull('dispid expression',Method1.DispIDExpr);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestInterfaceDispIDMethod2;
begin
  StartInterface('IInterface','');
  AddMember('Procedure DoSomething(A : Integer); dispid 12');
  ParseClass;
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('dispid modifier',[pmDispID],Method1.Modifiers);
  AssertNotNull('dispid expression',Method1.DispIDExpr);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

procedure TTestClassType.TestInterfaceProperty;
begin
  StartInterface('IInterface','');
  AddMember('Function GetS : Integer');
  AddMember('Property S : Integer Read GetS');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  if TheClass.members.Count<1 then
    Fail('No members for method');
  AssertNotNull('Have method',FunctionMethod1);
  AssertNotNull('Method proc type',FunctionMethod1.ProcType);
  AssertMemberName('GetS');
  AssertEquals('0 arguments',0,FunctionMethod1.ProcType.Args.Count) ;
  AssertEquals('Default visibility',visDefault,FunctionMethod1.Visibility);
  AssertEquals('No modifiers',[],FunctionMethod1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, FunctionMethod1.ProcType.CallingConvention);
  AssertNull('No UUID',TheClass.GUIDExpr);
  AssertNotNull('Have property',Property2);
  AssertMemberName('S',Property2);
end;

procedure TTestClassType.TestInterfaceDispProperty;
begin
  StartInterface('IInterface','',True);
  AddMember('Property S : Integer DispID 1');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okDispInterface,TheClass.ObjKind);
  if TheClass.members.Count<1 then
    Fail('No members for method');
  AssertNotNull('Have property',Property1);
  AssertMemberName('S',Property1);
  AssertNotNull('Have property dispID',Property1.DispIDExpr);
  AssertEquals('Have number',pekNumber,Property1.DispIDExpr.Kind);
  AssertEquals('Have number','1', (Property1.DispIDExpr as TPrimitiveExpr).Value);
end;

procedure TTestClassType.TestInterfaceDispPropertyReadOnly;
begin
  StartInterface('IInterface','',True);
  AddMember('Property S : Integer readonly DispID 1');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okDispInterface,TheClass.ObjKind);
  if TheClass.members.Count<1 then
    Fail('No members for method');
  AssertNotNull('Have property',Property1);
  AssertMemberName('S',Property1);
  AssertNotNull('Have property dispID',Property1.DispIDExpr);
  AssertTrue('DispID property is readonly',Property1.DispIDReadOnly);
  AssertEquals('Have number',pekNumber,Property1.DispIDExpr.Kind);
  AssertEquals('Have number','1', (Property1.DispIDExpr as TPrimitiveExpr).Value);
end;

procedure TTestClassType.TestInterfaceNoConstructor;
begin
  StartInterface('','');
  AddMember('Constructor DoSomething(A : Integer)');
  AssertParserError('No constructor in interface');
end;

procedure TTestClassType.TestInterfaceNoDestructor;
begin
  StartInterface('','');
  AddMember('Destructor DoSomething(A : Integer)');
  AssertParserError('No destructor in interface');
end;

procedure TTestClassType.TestInterfaceNoFields;
begin
  StartInterface('','');
  AddMember('AField : Integer');
  AssertParserError('No fields in interface');
end;

procedure TTestClassType.TestInterfaceUUID;
begin
  StartInterface('','123');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertExpression('UUID',TheClass.GUIDExpr,pekString,'''123''');
end;

procedure TTestClassType.TestInterfaceUUIDParentedEmpty;
begin
  StartInterface('IInterface','123');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  AssertEquals('No members',0,TheClass.Members.Count);
  AssertExpression('UUID',TheClass.GUIDExpr,pekString,'''123''');
end;

procedure TTestClassType.TestInterfaceUUIDOneMethod;
begin
  StartInterface('IInterface','123');
  AddMember('Procedure DoSomething(A : Integer)');
  EndClass();
  ParseClass;
  AssertEquals('Is interface',okInterface,TheClass.ObjKind);
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
  AssertExpression('UUID',TheClass.GUIDExpr,pekString,'''123''');
end;

procedure TTestClassType.TestRecordHelperEmpty;
begin
  StartRecordHelper('TOriginal','');
  ParseClass;
  AssertEquals('Is Record helper',okRecordHelper,TheClass.ObjKind);
  AssertNotNull('Have helper original',TheClass.HelperForType);
  AssertEquals('Have helper original alias',TPasUnresolvedTypeRef,TheClass.HelperForType.ClassType);
  AssertEquals('Helper original alias name','TOriginal',TheClass.HelperForType.Name);
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestRecordHelperParentedEmpty;
begin
  StartRecordHelper('TOriginal','TOtherHelper');
  ParseClass;
  AssertEquals('Is Record helper',okRecordHelper,TheClass.ObjKind);
  AssertNotNull('Have helper original',TheClass.HelperForType);
  AssertEquals('Have helper original alias',TPasUnresolvedTypeRef,TheClass.HelperForType.ClassType);
  AssertEquals('Helper original alias name','TOriginal',TheClass.HelperForType.Name);
  AssertEquals('No members',0,TheClass.Members.Count);
end;

procedure TTestClassType.TestRecordHelperOneMethod;
begin
  StartRecordHelper('TOriginal','');
  AddMember('Procedure DoSomething(A : Integer)');
  ParseClass;
  AssertEquals('Is Record helper',okRecordHelper,TheClass.ObjKind);
  AssertNotNull('Have helper original',TheClass.HelperForType);
  AssertEquals('Have helper original alias',TPasUnresolvedTypeRef,TheClass.HelperForType.ClassType);
  AssertEquals('Helper original alias name','TOriginal',TheClass.HelperForType.Name);
  DefaultMethod;
  AssertEquals('Default visibility',visDefault,Method1.Visibility);
  AssertEquals('No modifiers',[],Method1.Modifiers);
  AssertEquals('Default calling convention',ccDefault, Method1.ProcType.CallingConvention);
end;

initialization

  RegisterTest(TTestClassType);
end.

