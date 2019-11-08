{
    This file is part of the Free Component Library

    Pascal parse tree classes
    Copyright (c) 2017  Mattias Gaertner, mattias@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
Abstract:
  After running TPasResolver, run this to
  - create a list of used declararion, either in a module or a whole program.
  - emit hints about unused declarations
  - and warnings about uninitialized variables.

Working:
- mark used elements of a module, starting from all accessible elements
- Hint: 'Unit "%s" not used in %s'
- Hint: 'Parameter "%s" not used'
- Hint: 'Local variable "%s" not used'
- Hint: 'Value parameter "%s" is assigned but never used'
- Hint: 'Local variable "%s" is assigned but never used'
- Hint: 'Local %s "%s" not used'
- Hint: 'Private field "%s" is never used'
- Hint: 'Private field "%s" is assigned but never used'
- Hint: 'Private method "%s" is never used'
- Hint: 'Private type "%s" never used'
- Hint: 'Private const "%s" never used'
- Hint: 'Private property "%s" never used'
- Hint: 'Function result does not seem to be set'
- TPasArgument: compute the effective Access
- calls: use the effective Access of arguments
}
unit PasUseAnalyzer;

{$mode objfpc}{$H+}
{$inline on}

{$ifdef fpc}
  {$define UsePChar}
  {$define HasInt64}
{$endif}

interface

uses
  {$ifdef pas2js}
  js,
  {$else}
  AVL_Tree,
  {$endif}
  Classes, SysUtils, Types, contnrs,
  PasTree, PScanner, PasResolveEval, PasResolver;

const
  // non fpc hints
  nPAParameterInOverrideNotUsed = 4501;
  sPAParameterInOverrideNotUsed = 'Parameter "%s" not used';
  // fpc hints: use same IDs as fpc
  nPAUnitNotUsed = 5023;
  sPAUnitNotUsed = 'Unit "%s" not used in %s';
  nPAParameterNotUsed = 5024;
  sPAParameterNotUsed = 'Parameter "%s" not used';
  nPALocalVariableNotUsed = 5025;
  sPALocalVariableNotUsed = 'Local variable "%s" not used';
  nPAValueParameterIsAssignedButNeverUsed = 5026;
  sPAValueParameterIsAssignedButNeverUsed = 'Value parameter "%s" is assigned but never used';
  nPALocalVariableIsAssignedButNeverUsed = 5027;
  sPALocalVariableIsAssignedButNeverUsed = 'Local variable "%s" is assigned but never used';
  nPALocalXYNotUsed = 5028;
  sPALocalXYNotUsed = 'Local %s "%s" not used';
  nPAPrivateFieldIsNeverUsed = 5029;
  sPAPrivateFieldIsNeverUsed = 'Private field "%s" is never used';
  nPAPrivateFieldIsAssignedButNeverUsed = 5030;
  sPAPrivateFieldIsAssignedButNeverUsed = 'Private field "%s" is assigned but never used';
  nPAPrivateMethodIsNeverUsed = 5031;
  sPAPrivateMethodIsNeverUsed = 'Private method "%s" is never used';
  nPAFunctionResultDoesNotSeemToBeSet = 5033;
  sPAFunctionResultDoesNotSeemToBeSet  = 'Function result does not seem to be set';
  nPAPrivateTypeXNeverUsed = 5071;
  sPAPrivateTypeXNeverUsed = 'Private type "%s" never used';
  nPAPrivateConstXNeverUsed = 5072;
  sPAPrivateConstXNeverUsed = 'Private const "%s" never used';
  nPAPrivatePropertyXNeverUsed = 5073;
  sPAPrivatePropertyXNeverUsed = 'Private property "%s" never used';

type
  EPasAnalyzer = class(EPasResolve);

  { TPAMessage }

  TPAMessage = class
  private
    FRefCount: integer;
  public
    Id: TMaxPrecInt;
    MsgType: TMessageType;
    MsgNumber: integer;
    MsgText: string;
    MsgPattern: String;
    Args: TMessageArgs;
    PosEl: TPasElement;
    Filename: string;
    Row, Col: integer;
    constructor Create;
    procedure AddRef;
    procedure Release;
    property RefCount: integer read FRefCount;
  end;

  TPAMessageEvent = procedure(Sender: TObject; Msg: TPAMessage) of object;

  TPAIdentifierAccess = (
    paiaNone,
    paiaRead,
    paiaWrite,
    paiaReadWrite,
    paiaWriteRead
    );

  { TPAElement }

  TPAElement = class
  private
    FElement: TPasElement;
    procedure SetElement(AValue: TPasElement);
  public
    Access: TPAIdentifierAccess;
    destructor Destroy; override;
    property Element: TPasElement read FElement write SetElement;
  end;
  TPAElementClass = class of TPAElement;

  { TPAOverrideList
    used for
    - a method and its overrides
    - an interface method and its implementations
    - an interface and its delegations (property implements) }

  TPAOverrideList = class
  private
    FElement: TPasElement;
    FOverrides: TFPList; // list of TPasElement
    function GetOverrides(Index: integer): TPasElement; inline;
    procedure SetElement(AValue: TPasElement);
  public
    constructor Create;
    destructor Destroy; override;
    procedure Add(OverrideEl: TPasElement);
    property Element: TPasElement read FElement write SetElement;
    function Count: integer;
    function IndexOf(OverrideEl: TPasElement): integer; inline;
    property Overrides[Index: integer]: TPasElement read GetOverrides; default;
  end;

  {$ifdef pas2js}
  TPASItemToNameProc = function(Item: Pointer): String;
  {$endif}

  { TPasAnalyzerKeySet - set of items, each item has a key, no duplicate keys }

  TPasAnalyzerKeySet = class
  private
    {$ifdef pas2js}
    FItems: TJSObject;
    FCount: integer;
    FItemToName: TPASItemToNameProc;
    FKeyToName: TPASItemToNameProc;
    {$else}
    FTree: TAVLTree; // tree of pointers, sorted for keys given by OnItemToKey, no duplicate keys
    FCompareKeyWithData: TListSortCompare;
    {$endif}
  public
    {$ifdef pas2js}
    constructor Create(const OnItemToName, OnKeyToName: TPASItemToNameProc); reintroduce;
    {$else}
    constructor Create(const OnCompareMethod: TListSortCompare;
      const OnCompareKeyWithData: TListSortCompare);
    {$endif}
    destructor Destroy; override;
    procedure Clear;
    procedure FreeItems;
    procedure Add(Item: Pointer; CheckDuplicates: boolean = true);
    procedure Remove(Item: Pointer);
    function ContainsItem(Item: Pointer): boolean;
    function ContainsKey(Key: Pointer): boolean;
    function FindItem(Item: Pointer): Pointer;
    function FindKey(Key: Pointer): Pointer;
    function Count: integer;
    function GetList: TFPList; // list of items
  end;

  TPasAnalyzerOption = (
    paoOnlyExports, // default: use all class members accessible from outside (protected, but not private)
    paoImplReferences // collect references of top lvl proc implementations, initializationa dn finalization sections
    );
  TPasAnalyzerOptions = set of TPasAnalyzerOption;

  TPAUseMode = (
    paumElement, // Mark element. Do not descend into children.
    paumAllPasUsable, // Mark element and descend into children and mark non private identifiers
    paumAllExports, // Do not mark element. Descend into children and mark exports.
    paumTypeInfo // Mark element and its type and descend into children and mark published identifiers
    );
  TPAUseModes = set of TPAUseMode;
const
  PAUseModeToPSRefAccess: array[TPAUseMode] of TPSRefAccess = (
    psraRead,
    psraRead,
    psraRead,
    psraTypeInfo
    );

type
  TPAOtherCheckedEl = (
    pocClassConstructor
    );

  { TPasAnalyzer }

  TPasAnalyzer = class
  private
    FModeChecked: array[TPAUseMode] of TPasAnalyzerKeySet; // tree of TElement
    FOtherChecked: array[TPAOtherCheckedEl] of TPasAnalyzerKeySet; // tree of TElement
    FOnMessage: TPAMessageEvent;
    FOptions: TPasAnalyzerOptions;
    FOverrideLists: TPasAnalyzerKeySet; // tree of TPAOverrideList sorted for Element
    FResolver: TPasResolver;
    FScopeModule: TPasModule;
    FUsedElements: TPasAnalyzerKeySet; // tree of TPAElement sorted for Element
    procedure UseElType(El: TPasElement; aType: TPasType; Mode: TPAUseMode); inline;
    function AddOverride(OverriddenEl, OverrideEl: TPasElement): boolean;
    procedure SetOptions(AValue: TPasAnalyzerOptions);
    procedure UpdateAccess(IsWrite: Boolean; IsRead: Boolean; Usage: TPAElement);
    procedure OnUseScopeRef(Data, DeclScope: pointer);
  protected
    procedure RaiseInconsistency(const Id: TMaxPrecInt; Msg: string);
    procedure RaiseNotSupported(const Id: TMaxPrecInt; El: TPasElement; const Msg: string = '');
    function FindTopImplScope(El: TPasElement): TPasScope;
    // mark used elements
    function Add(El: TPasElement; CheckDuplicate: boolean = true;
      aClass: TPAElementClass = nil): TPAElement;
    function PAElementExists(El: TPasElement): boolean; inline;
    procedure CreateTree; virtual;
    function MarkElementAsUsed(El: TPasElement; aClass: TPAElementClass = nil): boolean; // true if new
    function ElementVisited(El: TPasElement; Mode: TPAUseMode): boolean; overload;
    function ElementVisited(El: TPasElement; OtherCheck: TPAOtherCheckedEl): boolean; overload;
    procedure MarkImplScopeRef(El, RefEl: TPasElement; Access: TPSRefAccess);
    function CanSkipGenericType(El: TPasGenericType): boolean;
    function CanSkipGenericProc(DeclProc: TPasProcedure): boolean;
    procedure UseElement(El: TPasElement; Access: TResolvedRefAccess;
      UseFull: boolean); virtual;
    procedure UseTypeInfo(El: TPasElement); virtual;
    procedure UseAttributes(El: TPasElement); virtual;
    function UseModule(aModule: TPasModule; Mode: TPAUseMode): boolean; virtual;
    procedure UseSection(Section: TPasSection; Mode: TPAUseMode); virtual;
    procedure UseImplBlock(Block: TPasImplBlock; Mark: boolean); virtual;
    procedure UseImplElement(El: TPasImplElement); virtual;
    procedure UseExpr(El: TPasExpr); virtual;
    procedure UseExprRef(El: TPasElement; Expr: TPasExpr;
      Access: TResolvedRefAccess; UseFull: boolean); virtual;
    procedure UseInheritedExpr(El: TInheritedExpr); virtual;
    procedure UseInlineSpecializeExpr(El: TInlineSpecializeExpr); virtual;
    procedure UseScopeReferences(Refs: TPasScopeReferences); virtual;
    procedure UseProcedure(Proc: TPasProcedure); virtual;
    procedure UseProcedureType(ProcType: TPasProcedureType); virtual;
    procedure UseType(El: TPasType; Mode: TPAUseMode); virtual;
    procedure UseClassOrRecType(El: TPasMembersType; Mode: TPAUseMode); virtual;
    procedure UseClassConstructor(El: TPasMembersType); virtual;
    procedure UseSpecializeType(El: TPasSpecializeType; Mode: TPAUseMode); virtual;
    procedure UseVariable(El: TPasVariable; Access: TResolvedRefAccess;
      UseFull: boolean); virtual;
    procedure UseResourcestring(El: TPasResString); virtual;
    procedure UseArgument(El: TPasArgument; Access: TResolvedRefAccess); virtual;
    procedure UseResultElement(El: TPasResultElement; Access: TResolvedRefAccess); virtual;
    // create hints for a unit, program or library
    procedure EmitElementHints(El: TPasElement); virtual;
    procedure EmitSectionHints(Section: TPasSection); virtual;
    procedure EmitDeclarationsHints(El: TPasDeclarations); virtual;
    procedure EmitTypeHints(El: TPasType); virtual;
    procedure EmitVariableHints(El: TPasVariable); virtual;
    procedure EmitProcedureHints(El: TPasProcedure); virtual;
    procedure EmitFunctionResultHints(El: TPasFunction); virtual;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Clear;
    procedure AnalyzeModule(aModule: TPasModule);
    procedure AnalyzeWholeProgram(aStartModule: TPasProgram);
    procedure EmitModuleHints(aModule: TPasModule); virtual;
    function FindElement(El: TPasElement): TPAElement; inline;
    function FindUsedElement(El: TPasElement): TPAElement;
    // utility
    function IsUsed(El: TPasElement): boolean; // valid after calling Analyze*
    function IsTypeInfoUsed(El: TPasElement): boolean; // valid after calling Analyze*
    function IsModuleInternal(El: TPasElement): boolean;
    function IsExport(El: TPasElement): boolean;
    function IsIdentifier(El: TPasElement): boolean;
    function IsImplBlockEmpty(El: TPasImplBlock): boolean;
    function IsSpecializedGenericType(El: TPasElement): boolean;
    procedure EmitMessage(Id: TMaxPrecInt; MsgType: TMessageType;
      MsgNumber: integer; Fmt: String;
      const Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
      PosEl: TPasElement);
    procedure EmitMessage(Msg: TPAMessage);
    class function GetWarnIdentifierNumbers(Identifier: string;
      out MsgNumbers: TIntegerDynArray): boolean; virtual;
    function GetUsedElements: TFPList; virtual; // list of TPAElement
    property OnMessage: TPAMessageEvent read FOnMessage write FOnMessage;
    property Options: TPasAnalyzerOptions read FOptions write SetOptions;
    property Resolver: TPasResolver read FResolver write FResolver;
    property ScopeModule: TPasModule read FScopeModule write FScopeModule;
  end;

{$ifdef pas2js}
function PasElementToHashName(Item: Pointer): String;
function PAElement_ElToHashName(Item: Pointer): String;
function PAOverrideList_ElToHashName(Item: Pointer): String;
{$else}
function ComparePAElements(Identifier1, Identifier2: Pointer): integer;
function CompareElementWithPAElement(El, Id: Pointer): integer;
function ComparePAOverrideLists(List1, List2: Pointer): integer;
function CompareElementWithPAOverrideList(El, List: Pointer): integer;
{$endif}
function GetElModName(El: TPasElement): string;
function dbgs(a: TPAIdentifierAccess): string; overload;

implementation

{$ifdef pas2js}
function PasElementToHashName(Item: Pointer): String;
var
  El: TPasElement absolute Item;
begin
  Result:=string(jsvalue(El.PasElementId));
end;

function PAElement_ElToHashName(Item: Pointer): String;
var
  El: TPAElement absolute Item;
begin
  Result:=string(jsvalue(El.Element.PasElementId));
end;

function PAOverrideList_ElToHashName(Item: Pointer): String;
var
  List: TPAOverrideList absolute Item;
begin
  Result:=string(jsvalue(List.Element.PasElementId));
end;
{$else}
function ComparePointer(Data1, Data2: Pointer): integer;
begin
  if Data1>Data2 then Result:=-1
  else if Data1<Data2 then Result:=1
  else Result:=0;
end;

function ComparePAElements(Identifier1, Identifier2: Pointer): integer;
var
  Item1: TPAElement absolute Identifier1;
  Item2: TPAElement absolute Identifier2;
begin
  Result:=ComparePointer(Item1.Element,Item2.Element);
end;

function CompareElementWithPAElement(El, Id: Pointer): integer;
var
  Identifier: TPAElement absolute Id;
begin
  Result:=ComparePointer(El,Identifier.Element);
end;

function ComparePAOverrideLists(List1, List2: Pointer): integer;
var
  Item1: TPAOverrideList absolute List1;
  Item2: TPAOverrideList absolute List2;
begin
  Result:=ComparePointer(Item1.Element,Item2.Element);
end;

function CompareElementWithPAOverrideList(El, List: Pointer): integer;
var
  OvList: TPAOverrideList absolute List;
begin
  Result:=ComparePointer(El,OvList.Element);
end;
{$endif}

function GetElModName(El: TPasElement): string;
var
  aModule: TPasModule;
begin
  if El=nil then exit('nil');
  Result:=El.PathName+':'+El.ClassName;
  aModule:=El.GetModule;
  if aModule=El then exit;
  if aModule=nil then
    Result:='NilModule.'+Result;
end;

function dbgs(a: TPAIdentifierAccess): string;
begin
  str(a,Result);
end;

{ TPasAnalyzerKeySet }

{$ifdef pas2js}
constructor TPasAnalyzerKeySet.Create(const OnItemToName,
  OnKeyToName: TPASItemToNameProc);
begin
  FItemToName:=OnItemToName;
  FKeyToName:=OnKeyToName;
  FItems:=TJSObject.new;
end;
{$else}
constructor TPasAnalyzerKeySet.Create(const OnCompareMethod: TListSortCompare;
  const OnCompareKeyWithData: TListSortCompare);
begin
  FTree:=TAVLTree.Create(OnCompareMethod);
  FCompareKeyWithData:=OnCompareKeyWithData;
end;
{$endif}

destructor TPasAnalyzerKeySet.Destroy;
begin
  {$ifdef pas2js}
  FItems:=nil;
  {$else}
  FreeAndNil(FTree);
  {$endif}
  inherited Destroy;
end;

procedure TPasAnalyzerKeySet.Clear;
begin
  {$ifdef pas2js}
  FItems:=TJSObject.new;
  FCount:=0;
  {$else}
  FTree.Clear;
  {$endif}
end;

procedure TPasAnalyzerKeySet.FreeItems;
{$ifdef pas2js}
var
  List: TStringDynArray;
  i: Integer;
begin
  List:=TJSObject.getOwnPropertyNames(FItems);
  for i:=0 to length(List)-1 do
    TObject(FItems[List[i]]).Destroy;
  FItems:=TJSObject.new;
  FCount:=0;
end;
{$else}
begin
  FTree.FreeAndClear;
end;
{$endif}

procedure TPasAnalyzerKeySet.Add(Item: Pointer; CheckDuplicates: boolean);
begin
  if CheckDuplicates {$IFDEF VerbosePasAnalyzer}or true{$endif} then
    if ContainsItem(Item) then
      raise Exception.Create('[20181101151755] TPasAnalyzerSet.Add duplicate');
  {$ifdef pas2js}
  FItems['%'+FItemToName(Item)]:=Item;
  inc(FCount);
  {$else}
  FTree.Add(Item);
  {$endif}
  {$ifdef VerbosePasAnalyzer}
  if not ContainsItem(Item) then
    raise Exception.Create('[20181101151811] TPasAnalyzerSet.Add failed');
  {$endif}
end;

procedure TPasAnalyzerKeySet.Remove(Item: Pointer);
{$ifdef pas2js}
var
  aName: string;
begin
  aName:='%'+FItemToName(Item);
  if not FItems.hasOwnProperty(aName) then exit;
  JSDelete(FItems,aName);
  dec(FCount);
end;
{$else}
begin
  FTree.Remove(Item);
end;
{$endif}

function TPasAnalyzerKeySet.ContainsItem(Item: Pointer): boolean;
begin
  {$ifdef pas2js}
  Result:=FItems.hasOwnProperty('%'+FItemToName(Item));
  {$else}
  Result:=FTree.Find(Item)<>nil;
  {$endif}
end;

function TPasAnalyzerKeySet.ContainsKey(Key: Pointer): boolean;
begin
  {$ifdef pas2js}
  Result:=FItems.hasOwnProperty('%'+FKeyToName(Key));
  {$else}
  Result:=FTree.FindKey(Key,FCompareKeyWithData)<>nil;
  {$endif}
end;

function TPasAnalyzerKeySet.FindItem(Item: Pointer): Pointer;
{$ifdef pas2js}
var
  aName: string;
begin
  aName:='%'+FItemToName(Item);
  if not FItems.hasOwnProperty(aName) then
    exit(nil)
  else
    Result:=Pointer(FItems[aName]);
end;
{$else}
var
  Node: TAVLTreeNode;
begin
  Node:=FTree.Find(Item);
  if Node<>nil then
    Result:=Node.Data
  else
    Result:=nil;
end;
{$endif}

function TPasAnalyzerKeySet.FindKey(Key: Pointer): Pointer;
{$ifdef pas2js}
var
  aName: string;
begin
  aName:='%'+FKeyToName(Key);
  if not FItems.hasOwnProperty(aName) then
    exit(nil)
  else
    Result:=Pointer(FItems[aName]);
end;
{$else}
var
  Node: TAVLTreeNode;
begin
  Node:=FTree.FindKey(Key,FCompareKeyWithData);
  if Node<>nil then
    Result:=Node.Data
  else
    Result:=nil;
end;
{$endif}

function TPasAnalyzerKeySet.Count: integer;
begin
  {$ifdef pas2js}
  Result:=FCount;
  {$else}
  Result:=FTree.Count;
  {$endif}
end;

function TPasAnalyzerKeySet.GetList: TFPList;
{$ifdef pas2js}
var
  List: TStringDynArray;
  i: Integer;
begin
  List:=TJSObject.getOwnPropertyNames(FItems);
  Result:=TFPList.Create;
  for i:=0 to length(List)-1 do
    Result.Add(FItems[List[i]]);
end;
{$else}
var
  Node: TAVLTreeNode;
begin
  Result:=TFPList.Create;
  Node:=FTree.FindLowest;
  while Node<>nil do
    begin
    Result.Add(Node.Data);
    Node:=FTree.FindSuccessor(Node);
    end;
end;
{$endif}

{ TPAMessage }

constructor TPAMessage.Create;
begin
  FRefCount:=1;
end;

procedure TPAMessage.AddRef;
begin
  inc(FRefCount);
end;

procedure TPAMessage.Release;
begin
  if FRefCount=0 then
    raise Exception.Create('');
  dec(FRefCount);
  if FRefCount=0 then
    {$ifdef pas2js}
    Destroy;
    {$else}
    Free;
    {$endif}
end;

{ TPAOverrideList }

// inline
function TPAOverrideList.GetOverrides(Index: integer): TPasElement;
begin
  Result:=TPasElement(FOverrides[Index]);
end;

// inline
function TPAOverrideList.IndexOf(OverrideEl: TPasElement): integer;
begin
  Result:=FOverrides.IndexOf(OverrideEl);
end;

procedure TPAOverrideList.SetElement(AValue: TPasElement);
begin
  if FElement=AValue then Exit;
  if FElement<>nil then
    FElement.Release{$IFDEF CheckPasTreeRefCount}('TPAOverrideList.Element'){$ENDIF};
  FElement:=AValue;
  if FElement<>nil then
    FElement.AddRef{$IFDEF CheckPasTreeRefCount}('TPAOverrideList.Element'){$ENDIF};
end;

constructor TPAOverrideList.Create;
begin
  FOverrides:=TFPList.Create;
end;

destructor TPAOverrideList.Destroy;
var
  i: Integer;
begin
  for i:=0 to FOverrides.Count-1 do
    TPasElement(FOverrides[i]).Release{$IFDEF CheckPasTreeRefCount}('TPAOverrideList.Overrides'){$ENDIF};
  FreeAndNil(FOverrides);
  Element:=nil;
  inherited Destroy;
end;

procedure TPAOverrideList.Add(OverrideEl: TPasElement);
begin
  FOverrides.Add(OverrideEl);
  OverrideEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPAOverrideList.Overrides'){$ENDIF};
end;

function TPAOverrideList.Count: integer;
begin
  Result:=FOverrides.Count;
end;

{ TPAElement }

procedure TPAElement.SetElement(AValue: TPasElement);
begin
  if FElement=AValue then Exit;
  if FElement<>nil then
    FElement.Release{$IFDEF CheckPasTreeRefCount}('TPAElement.Element'){$ENDIF};
  FElement:=AValue;
  if FElement<>nil then
    FElement.AddRef{$IFDEF CheckPasTreeRefCount}('TPAElement.Element'){$ENDIF};
end;

destructor TPAElement.Destroy;
begin
  Element:=nil;
  inherited Destroy;
end;

{ TPasAnalyzer }

// inline
function TPasAnalyzer.PAElementExists(El: TPasElement): boolean;
begin
  Result:=FUsedElements.ContainsKey(El);
end;

// inline
procedure TPasAnalyzer.UseElType(El: TPasElement; aType: TPasType;
  Mode: TPAUseMode);
begin
  if aType=nil then exit;
  MarkImplScopeRef(El,aType,PAUseModeToPSRefAccess[Mode]);
  UseType(aType,Mode);
end;

// inline
function TPasAnalyzer.FindElement(El: TPasElement): TPAElement;
begin
  Result:=TPAElement(FUsedElements.FindKey(El));
end;

procedure TPasAnalyzer.SetOptions(AValue: TPasAnalyzerOptions);
begin
  if FOptions=AValue then Exit;
  FOptions:=AValue;
end;

function TPasAnalyzer.AddOverride(OverriddenEl, OverrideEl: TPasElement): boolean;
// OverrideEl overrides OverriddenEl
// returns true if new override
var
  Item: TPAOverrideList;
  OverriddenPAEl: TPAElement;
  TypeEl: TPasType;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.AddOverride OverriddenEl=',GetElModName(OverriddenEl),' OverrideEl=',GetElModName(OverrideEl));
  {$ENDIF}
  Item:=TPAOverrideList(FOverrideLists.FindKey(OverriddenEl));
  if Item=nil then
    begin
    Item:=TPAOverrideList.Create;
    Item.Element:=OverriddenEl;
    FOverrideLists.Add(Item,false);
    end
  else
    begin
    if Item.IndexOf(OverrideEl)>=0 then
      exit(false);
    end;
  // new override
  Item.Add(OverrideEl);
  Result:=true;

  OverriddenPAEl:=FindElement(OverriddenEl);
  if OverriddenPAEl<>nil then
    begin
    // OverriddenEl was already used -> use OverrideEl
    if OverrideEl.ClassType=TPasProperty then
      begin
      if OverriddenEl is TPasType then
        begin
        TypeEl:=Resolver.ResolveAliasTypeEl(TPasType(OverriddenEl));
        if (TypeEl.ClassType=TPasClassType)
            and (TPasClassType(TypeEl).ObjKind=okInterface) then
          begin
          // interface was already used -> use delegation / property implements
          UseVariable(TPasProperty(OverrideEl),rraRead,false);
          exit;
          end;
        end;
      RaiseNotSupported(20180328221736,OverrideEl,GetElModName(OverriddenEl));
      end
    else
      UseElement(OverrideEl,rraNone,true);
    end;
end;

procedure TPasAnalyzer.UpdateAccess(IsWrite: Boolean; IsRead: Boolean;
  Usage: TPAElement);
begin
  if IsRead then
    case Usage.Access of
      paiaNone: Usage.Access:=paiaRead;
      paiaRead: ;
      paiaWrite: Usage.Access:=paiaWriteRead;
      paiaReadWrite: ;
      paiaWriteRead: ;
      else RaiseInconsistency(20170311183122, '');
    end;
  if IsWrite then
    case Usage.Access of
      paiaNone: Usage.Access:=paiaWrite;
      paiaRead: Usage.Access:=paiaReadWrite;
      paiaWrite: ;
      paiaReadWrite: ;
      paiaWriteRead: ;
      else RaiseInconsistency(20170311183127, '');
    end;
end;

procedure TPasAnalyzer.OnUseScopeRef(Data, DeclScope: pointer);
var
  Ref: TPasScopeReference absolute data;
  Scope: TPasScope absolute DeclScope;
begin
  if Scope=nil then ;
  while Ref<>nil do
    begin
    case Ref.Access of
      psraNone: ;
      psraRead: UseElement(Ref.Element,rraRead,false);
      psraWrite: UseElement(Ref.Element,rraAssign,false);
      psraReadWrite: UseElement(Ref.Element,rraReadAndAssign,false);
      psraWriteRead:
        begin
        UseElement(Ref.Element,rraAssign,false);
        UseElement(Ref.Element,rraRead,false);
        end;
      psraTypeInfo: UseTypeInfo(Ref.Element);
    else
      RaiseNotSupported(20180228191928,Ref.Element,dbgs(Ref.Access));
    end;
    Ref:=Ref.NextSameName;
    end;
end;

procedure TPasAnalyzer.RaiseInconsistency(const Id: TMaxPrecInt; Msg: string);
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.RaiseInconsistency ['+IntToStr(Id)+']: '+Msg);
  {$ENDIF}
  raise EPasAnalyzer.Create('['+IntToStr(Id)+']: '+Msg);
end;

procedure TPasAnalyzer.RaiseNotSupported(const Id: TMaxPrecInt; El: TPasElement;
  const Msg: string);
var
  s: String;
  E: EPasAnalyzer;
begin
  s:='['+IntToStr(Id)+']: Element='+GetObjPath(El);
  if Msg<>'' then S:=S+' '+Msg;
  E:=EPasAnalyzer.Create(s);
  E.PasElement:=El;
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.RaiseNotSupported ',E.Message);
  {$ENDIF}
  raise E;
end;

function TPasAnalyzer.FindTopImplScope(El: TPasElement): TPasScope;
var
  ProcScope: TPasProcedureScope;
  C: TClass;
  ImplProc: TPasProcedure;
begin
  Result:=nil;
  while El<>nil do
    begin
    C:=El.ClassType;
    if C.InheritsFrom(TPasProcedure) then
      begin
      ProcScope:=TPasProcedureScope(El.CustomData);
      if ProcScope.DeclarationProc<>nil then
        ProcScope:=TPasProcedureScope(ProcScope.DeclarationProc.CustomData);
      ImplProc:=ProcScope.ImplProc;
      if ImplProc=nil then
        ImplProc:=TPasProcedure(ProcScope.Element);
      if ImplProc.Body<>nil then
        // has implementation, not an external proc
        Result:=ProcScope;
      end
    else if (C=TInitializationSection)
        or (C=TFinalizationSection) then
      Result:=TPasInitialFinalizationScope(El.CustomData);
    El:=El.Parent;
    end;
end;

function TPasAnalyzer.Add(El: TPasElement; CheckDuplicate: boolean;
  aClass: TPAElementClass): TPAElement;
begin
  if El=nil then
    RaiseInconsistency(20170308093407,'');
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.Add ',GetElModName(El),' New=',not PAElementExists(El){$IFDEF Pas2js},' ID=[',El.PasElementId,']'{$ENDIF});
  {$ENDIF}
  {$IFDEF VerbosePasAnalyzer}CheckDuplicate:=true;{$ENDIF}
  if CheckDuplicate and PAElementExists(El) then
    RaiseInconsistency(20170304201318,'');
  if aClass=nil then
    aClass:=TPAElement;
  Result:=aClass.Create;
  Result.Element:=El;
  FUsedElements.Add(Result);
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.Add END ',GetElModName(El),' Success=',PAElementExists(El),' '{$Ifdef pas2js},El.PasElementId{$endif});
  {$ENDIF}
end;

procedure TPasAnalyzer.CreateTree;
begin
  FUsedElements:=TPasAnalyzerKeySet.Create(
    {$ifdef pas2js}
    @PAElement_ElToHashName,@PasElementToHashName
    {$else}
    @ComparePAElements,@CompareElementWithPAElement
    {$endif});
end;

function TPasAnalyzer.MarkElementAsUsed(El: TPasElement; aClass: TPAElementClass
  ): boolean;

  function MarkModule(CurModule: TPasModule): boolean;
  begin
    if PAElementExists(CurModule) then
      exit(false);
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.MarkElement.MarkModule mark "',GetElModName(CurModule),'"');
    {$ENDIF}
    Add(CurModule);
    Result:=true;
  end;

var
  CurModule: TPasModule;
begin
  if El=nil then exit(false);
  CurModule:=El.GetModule;
  if CurModule=nil then
    begin
    if El.ClassType=TPasUnresolvedSymbolRef then
      exit(false);
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.MarkElement GetModule failed for El=',GetElModName(El),' El.Parent=',GetElModName(El.Parent));
    {$ENDIF}
    RaiseInconsistency(20170308093540,GetElModName(El));
    end;
  if (ScopeModule<>nil) then
    begin
    // single module analysis
    if (CurModule<>ScopeModule) then
      begin
      // element from another unit
      // -> mark unit as used and do not descend deeper
      MarkModule(CurModule);
      exit(false);
      end;
    end;

  // mark element
  if PAElementExists(El) then exit(false);
  Add(El,false,aClass);
  Result:=true;

  if ScopeModule=nil then
    begin
    // whole program analysis
    if IsIdentifier(El) then
      // an identifier of this unit is used -> mark unit
      if MarkModule(CurModule) then
        UseModule(CurModule,paumElement);
    end;
end;

function TPasAnalyzer.ElementVisited(El: TPasElement; Mode: TPAUseMode
  ): boolean;
begin
  if El=nil then
    exit(true);
  if FModeChecked[Mode].ContainsItem(El) then exit(true);
  Result:=false;
  FModeChecked[Mode].Add(El,false);
end;

function TPasAnalyzer.ElementVisited(El: TPasElement;
  OtherCheck: TPAOtherCheckedEl): boolean;
begin
  if El=nil then
    exit(true);
  if FOtherChecked[OtherCheck].ContainsItem(El) then exit(true);
  Result:=false;
  FOtherChecked[OtherCheck].Add(El,false);
end;

procedure TPasAnalyzer.MarkImplScopeRef(El, RefEl: TPasElement;
  Access: TPSRefAccess);

  procedure CheckImplRef;
  // check if El inside a proc, initialization or finalization
  // and if RefEl is outside
  var
    ElImplScope, RefElImplScope: TPasScope;
  begin
    ElImplScope:=FindTopImplScope(El);
    if ElImplScope=nil then exit;
    RefElImplScope:=FindTopImplScope(RefEl);
    if RefElImplScope=ElImplScope then exit;

    if (RefEl.Name='') and not (RefEl is TInterfaceSection) then
      exit; // reference to anonymous type -> not needed
    if ElImplScope is TPasProcedureScope then
      TPasProcedureScope(ElImplScope).AddReference(RefEl,Access)
    else if ElImplScope is TPasInitialFinalizationScope then
      TPasInitialFinalizationScope(ElImplScope).AddReference(RefEl,Access)
    else
      RaiseInconsistency(20180302142933,GetObjName(ElImplScope));
  end;

begin
  if RefEl=nil then exit;
  if RefEl.Parent=El then exit; // same scope
  if paoImplReferences in Options then
    CheckImplRef;
end;

function TPasAnalyzer.CanSkipGenericType(El: TPasGenericType): boolean;

  procedure RaiseHalfSpecialized;
  var
    GenScope: TPasGenericScope;
    Item: TPRSpecializedItem;
  begin
    if (El.GenericTemplateTypes<>nil) and (El.GenericTemplateTypes.Count>0) then
      RaiseNotSupported(20190817151437,El);
    if not (El.CustomData is TPasGenericScope) then
      RaiseNotSupported(20190826141320,El,GetObjName(El.CustomData));
    GenScope:=TPasGenericScope(El.CustomData);
    Item:=GenScope.SpecializedFromItem;
    if Item=nil then
      RaiseNotSupported(20190826141352,El);
    if Item.SpecializedEl=nil then
      RaiseNotSupported(20190826141516,El);
    if Item.FirstSpecialize=nil then
      RaiseNotSupported(20190826141649,El);
    RaiseNotSupported(20190826141540,El,'SpecializedAt:'+GetObjPath(Item.FirstSpecialize)+' '+Resolver.GetElementSourcePosStr(Item.FirstSpecialize));
  end;

begin
  Result:=false;
  if ScopeModule=nil then
    begin
    // analyze whole program
    if not Resolver.IsFullySpecialized(El) then
      RaiseHalfSpecialized;
    end
  else
    begin
    // analyze a module
    if (El.GenericTemplateTypes<>nil) and (El.GenericTemplateTypes.Count>0) then
      // generic template -> analyze
    else if not Resolver.IsFullySpecialized(El) then
      // half specialized -> skip
      exit(true);
    end;
end;

function TPasAnalyzer.CanSkipGenericProc(DeclProc: TPasProcedure): boolean;

  procedure RaiseHalfSpecialized;
  var
    Templates: TFPList;
    ProcScope: TPasProcedureScope;
    Item: TPRSpecializedItem;
  begin
    Templates:=Resolver.GetProcTemplateTypes(DeclProc);
    if (Templates<>nil) and (Templates.Count>0) then
      RaiseNotSupported(20191016132828,DeclProc);
    if not (DeclProc.CustomData is TPasProcedureScope) then
      RaiseNotSupported(20191016132836,DeclProc,GetObjName(DeclProc.CustomData));
    ProcScope:=TPasProcedureScope(DeclProc.CustomData);
    Item:=ProcScope.SpecializedFromItem;
    if Item=nil then
      RaiseNotSupported(20191016133013,DeclProc);
    if Item.SpecializedEl=nil then
      RaiseNotSupported(20191016133017,DeclProc);
    if Item.FirstSpecialize=nil then
      RaiseNotSupported(20191016133019,DeclProc);
    RaiseNotSupported(20191016133022,DeclProc,'SpecializedAt:'+GetObjPath(Item.FirstSpecialize)+' '+Resolver.GetElementSourcePosStr(Item.FirstSpecialize));
  end;

var
  Templates: TFPList;
begin
  Result:=false;
  if ScopeModule=nil then
    begin
    // analyze whole program
    if not Resolver.IsFullySpecialized(DeclProc) then
      RaiseHalfSpecialized;
    end
  else
    begin
    // analyze a module
    Templates:=Resolver.GetProcTemplateTypes(DeclProc);
    if (Templates<>nil) and (Templates.Count>0) then
      // generic template -> analyze
    else if not Resolver.IsFullySpecialized(DeclProc) then
      // half specialized -> skip
      exit(true);
    end;
end;

procedure TPasAnalyzer.UseElement(El: TPasElement; Access: TResolvedRefAccess;
  UseFull: boolean);
var
  C: TClass;
begin
  if El=nil then exit;
  C:=El.ClassType;
  if C.InheritsFrom(TPasType) then
    UseType(TPasType(El),paumElement)
  else if C.InheritsFrom(TPasVariable) then
    UseVariable(TPasVariable(El),Access,UseFull)
  else if C=TPasArgument then
    UseArgument(TPasArgument(El),Access)
  else if C=TPasResultElement then
    UseResultElement(TPasResultElement(El),Access)
  else if C=TPasResString then
    UseResourcestring(TPasResString(El))
  else if C.InheritsFrom(TPasProcedure) then
    UseProcedure(TPasProcedure(El))
  else if C.InheritsFrom(TPasExpr) then
    UseExpr(TPasExpr(El))
  else if C=TPasEnumValue then
    UseExpr(TPasEnumValue(El).Value)
  else if C=TPasMethodResolution then
    // nothing to do
  else if (C.InheritsFrom(TPasModule)) or (C=TPasUsesUnit) then
    // e.g. unitname.identifier -> the module is used by the identifier
  else
    RaiseNotSupported(20170307090947,El);
  repeat
    El:=El.Parent;
    if not (El is TPasType) then break;
    UseType(TPasType(El),paumElement);
    //MarkElementAsUsed(El);
    //if El is TPasMembersType then
    //  UseClassConstructor(TPasMembersType(El));
  until false;
end;

procedure TPasAnalyzer.UseTypeInfo(El: TPasElement);
// mark typeinfo, do not mark code

  procedure UseSubEl(SubEl: TPasElement); inline;
  begin
    if SubEl=nil then exit;
    MarkImplScopeRef(El,SubEl,psraTypeInfo);
    UseTypeInfo(SubEl);
  end;

var
  C: TClass;
  Members, Args: TFPList;
  i: Integer;
  Member: TPasElement;
  MemberResolved: TPasResolverResult;
  Prop: TPasProperty;
  ProcType: TPasProcedureType;
  ClassEl: TPasClassType;
  ArrType: TPasArrayType;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UsePublished START ',GetObjName(El));
  {$ENDIF}
  if ElementVisited(El,paumTypeInfo) then exit;

  C:=El.ClassType;
  if C=TPasUnresolvedSymbolRef then
  else if (C=TPasVariable) or (C=TPasConst) then
    UseSubEl(TPasVariable(El).VarType)
  else if (C=TPasArgument) then
    UseSubEl(TPasArgument(El).ArgType)
  else if C=TPasProperty then
    begin
    // published property
    Prop:=TPasProperty(El);
    Args:=Resolver.GetPasPropertyArgs(Prop);
    for i:=0 to Args.Count-1 do
      UseSubEl(TPasArgument(Args[i]).ArgType);
    UseSubEl(Resolver.GetPasPropertyType(Prop));
    UseElement(Resolver.GetPasPropertyGetter(Prop),rraRead,false);
    UseElement(Resolver.GetPasPropertySetter(Prop),rraRead,false);
    UseElement(Resolver.GetPasPropertyIndex(Prop),rraRead,false);
    // stored and defaultvalue are only used when published -> mark as used
    UseElement(Resolver.GetPasPropertyStoredExpr(Prop),rraRead,false);
    UseElement(Resolver.GetPasPropertyDefaultExpr(Prop),rraRead,false);
    end
  else if (C=TPasAliasType) or (C=TPasTypeAliasType) then
    UseSubEl(TPasAliasType(El).DestType)
  else if C=TPasEnumType then
  else if C=TPasSetType then
    UseSubEl(TPasSetType(El).EnumType)
  else if C=TPasRangeType then
  else if C=TPasArrayType then
    begin
    ArrType:=TPasArrayType(El);
    if CanSkipGenericType(ArrType) then exit;
    UseSubEl(ArrType.ElType);
    for i:=0 to length(ArrType.Ranges)-1 do
      begin
      Member:=ArrType.Ranges[i];
      Resolver.ComputeElement(Member,MemberResolved,[rcConstant]);
      UseSubEl(MemberResolved.HiTypeEl);
      end;
    end
  else if C=TPasPointerType then
    UseSubEl(TPasPointerType(El).DestType)
  else if C=TPasClassType then
    begin
    ClassEl:=TPasClassType(El);
    if CanSkipGenericType(ClassEl) then exit;
    if ClassEl.ObjKind=okInterface then
      begin
      // mark all used members
      Members:=ClassEl.Members;
      for i:=0 to Members.Count-1 do
        begin
        Member:=TPasElement(Members[i]);
        if Member.ClassType=TPasAttributes then
          continue;
        if IsUsed(Member) then
          UseTypeInfo(Member);
        end;
      end;
    end
  else if C=TPasClassOfType then
  else if C=TPasRecordType then
    begin
    // published record: use all members
    if CanSkipGenericType(TPasRecordType(El)) then exit;
    Members:=TPasRecordType(El).Members;
    for i:=0 to Members.Count-1 do
      begin
      Member:=TPasElement(Members[i]);
      if Member.ClassType=TPasAttributes then
        continue; // attributes are never used directly
      UseSubEl(Member);
      end;
    end
  else if C.InheritsFrom(TPasProcedure) then
    UseSubEl(TPasProcedure(El).ProcType)
  else if C.InheritsFrom(TPasProcedureType) then
    begin
    ProcType:=TPasProcedureType(El);
    if CanSkipGenericType(ProcType) then exit;
    for i:=0 to ProcType.Args.Count-1 do
      UseSubEl(TPasArgument(ProcType.Args[i]).ArgType);
    if El is TPasFunctionType then
      UseSubEl(TPasFunctionType(El).ResultEl.ResultType);
    end
  else if C=TPasSpecializeType then
    UseSubEl(TPasSpecializeType(El).DestType)
  else if C=TPasGenericTemplateType then
    begin
    if ScopeModule=nil then
      RaiseNotSupported(20190817110226,El);
    end
  else
    begin
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.UsePublished ',GetObjName(El));
    {$ENDIF}
    RaiseNotSupported(20170414153904,El);
    end;

  UseElement(El,rraNone,true);

  UseAttributes(El);
end;

procedure TPasAnalyzer.UseAttributes(El: TPasElement);
var
  Calls: TPasExprArray;
  i: Integer;
begin
  Calls:=Resolver.GetAttributeCallsEl(El);
  for i:=0 to length(Calls)-1 do
    UseExpr(Calls[i]);
end;

function TPasAnalyzer.UseModule(aModule: TPasModule; Mode: TPAUseMode): boolean;

  procedure UseInitFinal(ImplBlock: TPasImplBlock);
  var
    Scope: TPasInitialFinalizationScope;
  begin
    if ImplBlock=nil then exit;
    Scope:=TPasInitialFinalizationScope(ImplBlock.CustomData);
    UseScopeReferences(Scope.References);
    if (Scope.References=nil) and IsImplBlockEmpty(ImplBlock) then exit;
    // this module has an initialization section -> mark module
    if not PAElementExists(aModule) then
      Add(aModule);
    UseImplBlock(ImplBlock,true);
  end;

var
  ModScope: TPasModuleScope;
begin
  if ElementVisited(aModule,Mode) then exit(false);
  Result:=true;

  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseModule ',GetElModName(aModule),' Mode=',Mode{$IFDEF pas2js},' ',aModule.PasElementId{$ENDIF});
  {$ENDIF}
  if Mode in [paumAllExports,paumAllPasUsable] then
    begin
    if aModule is TPasProgram then
      UseSection(TPasProgram(aModule).ProgramSection,Mode)
    else if aModule is TPasLibrary then
      UseSection(TPasLibrary(aModule).LibrarySection,Mode)
    else
      begin
      // unit
      UseSection(aModule.InterfaceSection,Mode);
      // Note: implementation can not be used directly from outside
      end;
    end;
  UseInitFinal(aModule.InitializationSection);
  UseInitFinal(aModule.FinalizationSection);
  ModScope:=aModule.CustomData as TPasModuleScope;
  if ModScope.RangeErrorClass<>nil then
    UseClassOrRecType(ModScope.RangeErrorClass,paumElement);
  if ModScope.RangeErrorConstructor<>nil then
    UseProcedure(ModScope.RangeErrorConstructor);
  // no need to use here ModScope.AssertClass, it is used by Assert
  // no need to use here ModScope.AssertMsgConstructor
  // no need to use here ModScope.AssertDefConstructor
  // no need to use here ModScope.SystemTVarRec

  if Mode=paumElement then
    // e.g. a reference: unitname.identifier
    if not PAElementExists(aModule) then
      Add(aModule);
end;

procedure TPasAnalyzer.UseSection(Section: TPasSection; Mode: TPAUseMode);
// called by UseModule
var
  i: Integer;
  UsedModule: TPasModule;
  Decl: TPasElement;
  OnlyExports: Boolean;
  UsesClause: TPasUsesClause;
  C: TClass;
begin
  // Section is TProgramSection, TLibrarySection, TInterfaceSection, TImplementationSection
  if Mode=paumElement then
    RaiseInconsistency(20170317172721,'');
  if ElementVisited(Section,Mode) then exit;

  OnlyExports:=Mode=paumAllExports;

  if Mode=paumAllPasUsable then
    MarkElementAsUsed(Section);
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseSection ',GetElModName(Section),' Mode=',Mode);
  {$ENDIF}

  // used units
  UsesClause:=Section.UsesClause;
  for i:=0 to length(UsesClause)-1 do
    begin
    if UsesClause[i].Module is TPasModule then
      begin
      UsedModule:=TPasModule(UsesClause[i].Module);
      if ScopeModule=nil then
        // whole program analysis
        UseModule(UsedModule,paumAllExports)
      else
        begin
        // unit analysis
        if IsImplBlockEmpty(UsedModule.InitializationSection)
            and IsImplBlockEmpty(UsedModule.FinalizationSection) then
          continue;
        if not PAElementExists(UsedModule) then
          Add(UsedModule);
        UseImplBlock(UsedModule.InitializationSection,true);
        UseImplBlock(UsedModule.FinalizationSection,true);
        end;
      end;
    end;

  // section declarations
  for i:=0 to Section.Declarations.Count-1 do
    begin
    Decl:=TPasElement(Section.Declarations[i]);
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.UseSection ',Section.ClassName,' Decl=',GetElModName(Decl),' Mode=',Mode);
    {$ENDIF}
    C:=Decl.ClassType;
    // Note: no MarkImplScopeRef needed, because all Decl are in the same scope
    if C.InheritsFrom(TPasProcedure) then
      begin
      if OnlyExports and ([pmExport,pmPublic]*TPasProcedure(Decl).Modifiers=[]) then
        continue;
      UseProcedure(TPasProcedure(Decl))
      end
    else if C.InheritsFrom(TPasType) then
      UseType(TPasType(Decl),Mode)
    else if C.InheritsFrom(TPasVariable) then
      begin
      if OnlyExports and ([vmExport,vmPublic]*TPasVariable(Decl).VarModifiers=[]) then
        continue;
      UseVariable(TPasVariable(Decl),rraNone,true);
      end
    else if C=TPasResString then
      begin
      if OnlyExports then continue;
      UseResourcestring(TPasResString(Decl));
      end
    else if C=TPasAttributes then
      // attributes are never used directly
    else
      RaiseNotSupported(20170306165213,Decl);
    end;
end;

procedure TPasAnalyzer.UseImplBlock(Block: TPasImplBlock; Mark: boolean);
var
  i: Integer;
  El: TPasElement;
begin
  if Block=nil then exit;
  if Mark and not MarkElementAsUsed(Block) then exit;
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseImplBlock ',GetElModName(Block),' Elements=',Block.Elements.Count);
  {$ENDIF}
  for i:=0 to Block.Elements.Count-1 do
    begin
    El:=TPasElement(Block.Elements[i]);
    if El is TPasImplElement then
      UseImplElement(TPasImplElement(El))
    else
      RaiseNotSupported(20170306195110,El);
    end;
end;

procedure TPasAnalyzer.UseImplElement(El: TPasImplElement);
var
  C: TClass;
  ForLoop: TPasImplForLoop;
  CaseOf: TPasImplCaseOf;
  i, j: Integer;
  CaseSt: TPasImplCaseStatement;
  WithDo: TPasImplWithDo;
  SubEl, ParentEl: TPasElement;
  ForScope: TPasForLoopScope;
begin
  // do not mark
  if El=nil then exit;
  C:=El.ClassType;
  if C=TPasImplBlock then
    // impl block
    UseImplBlock(TPasImplBlock(El),false)
  else if C=TPasImplSimple then
    // simple expression
    UseExpr(TPasImplSimple(El).expr)
  else if C=TPasImplAssign then
    // a:=b
    begin
    UseExpr(TPasImplAssign(El).left);
    UseExpr(TPasImplAssign(El).right);
    end
  else if C=TPasImplAsmStatement then
    // asm..end
  else if C=TPasImplBeginBlock then
    // begin..end
    UseImplBlock(TPasImplBeginBlock(El),false)
  else if C=TPasImplCaseOf then
    begin
    // case-of
    CaseOf:=TPasImplCaseOf(El);
    UseExpr(CaseOf.CaseExpr);
    for i:=0 to CaseOf.Elements.Count-1 do
      begin
      SubEl:=TPasElement(CaseOf.Elements[i]);
      if SubEl.ClassType=TPasImplCaseStatement then
        begin
        CaseSt:=TPasImplCaseStatement(SubEl);
        for j:=0 to CaseSt.Expressions.Count-1 do
          UseExpr(TObject(CaseSt.Expressions[j]) as TPasExpr);
        UseImplElement(CaseSt.Body);
        end
      else if SubEl.ClassType=TPasImplCaseElse then
        UseImplBlock(TPasImplCaseElse(SubEl),false)
      else
        RaiseNotSupported(20170307195329,SubEl);
      end;
    end
  else if C=TPasImplForLoop then
    begin
    // for-loop
    ForLoop:=TPasImplForLoop(El);
    UseExpr(ForLoop.VariableName);
    UseExpr(ForLoop.StartExpr);
    UseExpr(ForLoop.EndExpr);
    ForScope:=ForLoop.CustomData as TPasForLoopScope;
    MarkImplScopeRef(ForLoop,ForScope.GetEnumerator,psraRead);
    UseProcedure(ForScope.GetEnumerator);
    MarkImplScopeRef(ForLoop,ForScope.MoveNext,psraRead);
    UseProcedure(ForScope.MoveNext);
    MarkImplScopeRef(ForLoop,ForScope.Current,psraRead);
    UseVariable(ForScope.Current,rraRead,false);
    UseImplElement(ForLoop.Body);
    end
  else if C=TPasImplIfElse then
    begin
    // if-then-else
    UseExpr(TPasImplIfElse(El).ConditionExpr);
    UseImplElement(TPasImplIfElse(El).IfBranch);
    UseImplElement(TPasImplIfElse(El).ElseBranch);
    end
  else if C=TPasImplCommand then
    // used for if-then <empty> -> nothing to do
  else if C=TPasImplLabelMark then
    // label mark
  else if C=TPasImplRepeatUntil then
    begin
    // repeat-until
    UseImplBlock(TPasImplRepeatUntil(El),false);
    UseExpr(TPasImplRepeatUntil(El).ConditionExpr);
    end
  else if C=TPasImplWhileDo then
    begin
    // while-do
    UseExpr(TPasImplWhileDo(El).ConditionExpr);
    UseImplBlock(TPasImplWhileDo(El),false);
    end
  else if C=TPasImplWithDo then
    begin
    // with-do
    WithDo:=TPasImplWithDo(El);
    for i:=0 to WithDo.Expressions.Count-1 do
      UseExpr(TObject(WithDo.Expressions[i]) as TPasExpr);
    UseImplBlock(WithDo,false);
    end
  else if C=TPasImplExceptOn then
    begin
    // except-on
    // Note: VarEl is marked when actually used
    UseElType(El,TPasImplExceptOn(El).TypeEl,paumElement);
    UseImplElement(TPasImplExceptOn(El).Body);
    end
  else if C=TPasImplRaise then
    begin
    // raise
    if TPasImplRaise(El).ExceptObject<>nil then
      UseExpr(TPasImplRaise(El).ExceptObject)
    else
      begin
      // raise; -> mark On E:
      ParentEl:=El.Parent;
      while ParentEl<>nil do
        begin
        if ParentEl is TPasImplExceptOn then
          begin
          UseVariable(TPasVariable(TPasImplExceptOn(ParentEl).VarEl),rraRead,false);
          break;
          end;
        ParentEl:=ParentEl.Parent;
        end;
      end;
    UseExpr(TPasImplRaise(El).ExceptAddr);
    end
  else if C=TPasImplTry then
    begin
    // try..finally/except..else..end
    UseImplBlock(TPasImplTry(El),false);
    UseImplBlock(TPasImplTry(El).FinallyExcept,false);
    UseImplBlock(TPasImplTry(El).ElseBranch,false);
    end
  else
    RaiseNotSupported(20170307162715,El);
end;

procedure TPasAnalyzer.UseExpr(El: TPasExpr);

  procedure UseSystemExit;
  var
    Params: TPasExprArray;
    SubEl: TPasElement;
    Proc: TPasProcedure;
    ProcScope: TPasProcedureScope;
    ParentParams: TPRParentParams;
  begin
    Resolver.GetParamsOfNameExpr(El,ParentParams);
    if ParentParams.Params=nil then exit;
    Params:=ParentParams.Params.Params;
    if length(Params)<1 then
      exit;
    SubEl:=El.Parent;
    while (SubEl<>nil) and not (SubEl is TPasProcedure) do
      SubEl:=SubEl.Parent;
    if SubEl=nil then exit;
    Proc:=TPasProcedure(SubEl);
    if not (Proc.ProcType is TPasFunctionType) then
      RaiseNotSupported(20190825203504,El);
    ProcScope:=Proc.CustomData as TPasProcedureScope;
    if ProcScope.DeclarationProc<>nil then
      Proc:=ProcScope.DeclarationProc;
    SubEl:=TPasFunctionType(Proc.ProcType).ResultEl;
    UseElement(SubEl,rraAssign,false);
  end;

  procedure UseBuilInFuncTypeInfo;
  var
    ParentParams: TPRParentParams;
    ParamResolved: TPasResolverResult;
    SubEl: TPasElement;
    Params: TPasExprArray;
  begin
    Resolver.GetParamsOfNameExpr(El,ParentParams);
    if ParentParams.Params=nil then
      RaiseNotSupported(20190225150136,El);
    Params:=ParentParams.Params.Params;
    if length(Params)<>1 then
      RaiseNotSupported(20180226144217,El.Parent);
    Resolver.ComputeElement(Params[0],ParamResolved,[rcNoImplicitProc]);
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.UseExpr typeinfo ',GetResolverResultDbg(ParamResolved));
    {$ENDIF}
    if ParamResolved.IdentEl=nil then
      RaiseNotSupported(20180628155107,Params[0]);
    if (ParamResolved.IdentEl is TPasProcedure)
        and (TPasProcedure(ParamResolved.IdentEl).ProcType is TPasFunctionType) then
      begin
      SubEl:=TPasFunctionType(TPasProcedure(ParamResolved.IdentEl).ProcType).ResultEl.ResultType;
      MarkImplScopeRef(El,SubEl,psraTypeInfo);
      UseTypeInfo(SubEl);
      end
    else
      begin
      SubEl:=ParamResolved.IdentEl;
      MarkImplScopeRef(El,SubEl,psraTypeInfo);
      UseTypeInfo(SubEl);
      end;
    // the parameter is not used otherwise
  end;

var
  Ref: TResolvedReference;
  C: TClass;
  Params: TPasExprArray;
  i: Integer;
  BuiltInProc: TResElDataBuiltInProc;
  Decl: TPasElement;
  ModScope: TPasModuleScope;
  Access: TResolvedRefAccess;
begin
  if El=nil then exit;
  // Note: expression itself is not marked, but it can reference identifiers

  Ref:=nil;
  if El.CustomData is TResolvedReference then
    begin
    // this is a reference -> mark target
    Ref:=TResolvedReference(El.CustomData);
    Decl:=Ref.Declaration;
    Access:=Ref.Access;
    MarkImplScopeRef(El,Decl,ResolvedToPSRefAccess[Access]);
    UseElement(Decl,Access,false);

    if Ref.Context<>nil then
      begin
      if Ref.Context.ClassType=TResolvedRefCtxAttrProc then
        UseProcedure(TResolvedRefCtxAttrProc(Ref.Context).Proc);
      end;

    if Resolver.IsNameExpr(El) then
      begin
      if Ref.WithExprScope<>nil then
        begin
        if Ref.WithExprScope.ClassRecScope is TPasRecordScope then
          begin
          // a record member was accessed -> access the record too
          UseExprRef(El,Ref.WithExprScope.Expr,Access,false);
          exit;
          end;
        end;
      if (Decl is TPasVariable)
          and (El.Parent is TBinaryExpr)
          and (TBinaryExpr(El.Parent).right=El) then
        begin
        if ((Decl.Parent is TPasRecordType)
              or (Decl.Parent is TPasVariant)) then
          begin
          // a record member was accessed -> access the record with same Access
          UseExprRef(El.Parent,TBinaryExpr(El.Parent).left,Access,false);
          end;
        end;
      end;

    if Decl is TPasUnresolvedSymbolRef then
      begin
      if Decl.CustomData is TResElDataBuiltInProc then
        begin
        BuiltInProc:=TResElDataBuiltInProc(Decl.CustomData);
        case BuiltInProc.BuiltIn of
        bfExit:
          begin
          UseSystemExit;
          exit;
          end;
        bfTypeInfo:
          begin
          UseBuilInFuncTypeInfo;
          exit;
          end;
        bfAssert:
          begin
          ModScope:=Resolver.RootElement.CustomData as TPasModuleScope;
          if ModScope.AssertClass<>nil then
            UseElType(El,ModScope.AssertClass,paumElement);
          end;
        end;

        end;
      end;

    end;
  UseExpr(El.format1);
  UseExpr(El.format2);
  C:=El.ClassType;
  if (C=TPrimitiveExpr)
      or (C=TBoolConstExpr)
      or (C=TNilExpr) then
    // ok
  else if C=TBinaryExpr then
    begin
    UseExpr(TBinaryExpr(El).left);
    UseExpr(TBinaryExpr(El).right);
    end
  else if C=TUnaryExpr then
    UseExpr(TUnaryExpr(El).Operand)
  else if C=TParamsExpr then
    begin
    UseExpr(TParamsExpr(El).Value);
    Params:=TParamsExpr(El).Params;
    for i:=0 to length(Params)-1 do
      UseExpr(Params[i]);
    end
  else if C=TArrayValues then
    begin
    Params:=TArrayValues(El).Values;
    for i:=0 to length(Params)-1 do
      UseExpr(Params[i]);
    end
  else if C=TRecordValues then
    begin
    for i:=0 to length(TRecordValues(El).Fields)-1 do
      UseExpr(TRecordValues(El).Fields[i].ValueExp);
    end
  else if C=TInheritedExpr then
    UseInheritedExpr(TInheritedExpr(El))
  else if C=TProcedureExpr then
    UseProcedure(TProcedureExpr(El).Proc)
  else if C=TInlineSpecializeExpr then
    UseInlineSpecializeExpr(TInlineSpecializeExpr(El))
  else
    RaiseNotSupported(20170307085444,El);
end;

procedure TPasAnalyzer.UseExprRef(El: TPasElement; Expr: TPasExpr;
  Access: TResolvedRefAccess; UseFull: boolean);
var
  Ref: TResolvedReference;
  C: TClass;
  Bin: TBinaryExpr;
  Params: TParamsExpr;
  ValueResolved: TPasResolverResult;
  Unary: TUnaryExpr;
begin
  C:=Expr.ClassType;
  if C=TBinaryExpr then
    begin
    Bin:=TBinaryExpr(Expr);
    if Bin.OpCode in [eopSubIdent,eopNone] then
      UseExprRef(El,Bin.right,Access,UseFull);
    end
  else if C=TParamsExpr then
    begin
    Params:=TParamsExpr(Expr);
    case Params.Kind of
    pekFuncParams:
      if Resolver.IsTypeCast(Params) then
        UseExprRef(El,Params.Params[0],Access,UseFull)
      else
        UseExprRef(El,Params.Value,Access,UseFull);
    pekArrayParams:
      begin
      Resolver.ComputeElement(Params.Value,ValueResolved,[]);
      if not Resolver.IsDynArray(ValueResolved.LoTypeEl) then
        UseExprRef(El,Params.Value,Access,UseFull);
      end;
    pekSet: ;
    else
      RaiseNotSupported(20170403173817,Params);
    end;
    end
  else if (C=TPrimitiveExpr) and (TPrimitiveExpr(Expr).Kind=pekIdent) then
    begin
    if (Expr.CustomData is TResolvedReference) then
      begin
      Ref:=TResolvedReference(Expr.CustomData);
      MarkImplScopeRef(El,Ref.Declaration,ResolvedToPSRefAccess[Access]);
      UseElement(Ref.Declaration,Access,UseFull);
      end;
    end
  else if C=TUnaryExpr then
    begin
    Unary:=TUnaryExpr(Expr);
    if Unary.OpCode in [eopAdd,eopSubtract,eopAddress,eopDeref,eopMemAddress] then
      UseExprRef(El,Unary.Operand,rraRead,false)
    else
      RaiseNotSupported(20181015193334,Expr,OpcodeStrings[Unary.OpCode]);
    end
  else if (Access=rraRead)
      and ((C=TPrimitiveExpr) // Kind<>pekIdent
        or (C=TNilExpr)
        or (C=TBoolConstExpr)
        or (C=TUnaryExpr)) then
    // ok
  else
    begin
    {$IFDEF VerbosePasResolver}
    writeln('TPasResolver.UseExprRef Expr=',GetObjName(Expr),' Access=',Access,' Declaration="',Expr.GetDeclaration(false),'"');
    {$ENDIF}
    RaiseNotSupported(20170306102159,Expr);
    end;
end;

procedure TPasAnalyzer.UseInheritedExpr(El: TInheritedExpr);
var
  P: TPasElement;
  ProcScope: TPasProcedureScope;
  Proc: TPasProcedure;
  Args: TFPList;
  i: Integer;
  Arg: TPasArgument;
begin
  if (El.Parent.ClassType=TBinaryExpr)
  and (TBinaryExpr(El.Parent).OpCode=eopNone) then
    // 'inherited Proc...;'
    exit;
  // 'inherited;'
  P:=El.Parent;
  while not P.InheritsFrom(TPasProcedure) do
    P:=P.Parent;
  ProcScope:=TPasProcedure(P).CustomData as TPasProcedureScope;
  if ProcScope.DeclarationProc<>nil then
    Proc:=ProcScope.DeclarationProc
  else
    Proc:=TPasProcedure(P);
  Args:=Proc.ProcType.Args;
  for i:=0 to Args.Count-1 do
    begin
    Arg:=TPasArgument(Args[i]);
    case Arg.Access of
    argDefault,argConst,argConstRef: UseArgument(Arg,rraRead);
    argVar: UseArgument(Arg,rraVarParam);
    argOut: UseArgument(Arg,rraOutParam);
    else
      RaiseNotSupported(20171107175406,Arg);
    end;
    end;
end;

procedure TPasAnalyzer.UseInlineSpecializeExpr(El: TInlineSpecializeExpr);
var
  i: Integer;
begin
  for i:=0 to El.Params.Count-1 do
    UseType(TPasType(El.Params[i]),paumElement);
  UseExpr(El.NameExpr);
end;

procedure TPasAnalyzer.UseScopeReferences(Refs: TPasScopeReferences);
begin
  if Refs=nil then exit;
  Refs.References.ForEachCall(@OnUseScopeRef,Refs.Scope);
end;

procedure TPasAnalyzer.UseProcedure(Proc: TPasProcedure);

  procedure UseOverrides(CurProc: TPasProcedure);
  var
    OverrideList: TPAOverrideList;
    i: Integer;
    OverrideProc: TPasProcedure;
  begin
    OverrideList:=TPAOverrideList(FOverrideLists.FindKey(CurProc));
    if OverrideList=nil then exit;
    // Note: while traversing the OverrideList it may grow
    i:=0;
    while i<OverrideList.Count do
      begin
      OverrideProc:=TObject(OverrideList.Overrides[i]) as TPasProcedure;
      UseProcedure(OverrideProc);
      inc(i);
      end;
  end;

var
  ProcScope: TPasProcedureScope;
  ImplProc: TPasProcedure;
  ClassScope: TPasClassScope;
  Name: String;
  Identifier: TPasIdentifier;
  El: TPasElement;
  ClassEl: TPasClassType;
begin
  if Proc=nil then exit;
  // use declaration, not implementation
  ProcScope:=Proc.CustomData as TPasProcedureScope;
  if ProcScope.DeclarationProc<>nil then
    exit; // skip implementation, Note:PasResolver always refers the declaration
  if CanSkipGenericProc(Proc) then exit;

  if not MarkElementAsUsed(Proc) then exit;
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseProcedure ',GetElModName(Proc));
  {$ENDIF}
  if Proc.Parent is TPasMembersType then
    UseClassOrRecType(TPasMembersType(Proc.Parent),paumElement);

  UseScopeReferences(ProcScope.References);

  UseProcedureType(Proc.ProcType);

  ImplProc:=Proc;
  if ProcScope.ImplProc<>nil then
    ImplProc:=ProcScope.ImplProc;
  if ImplProc.Body<>nil then
    UseImplBlock(ImplProc.Body.Body,false);

  if Proc.IsOverride and (ProcScope.OverriddenProc<>nil) then
    AddOverride(ProcScope.OverriddenProc,Proc);

  // mark overrides
  if ([pmOverride,pmVirtual]*Proc.Modifiers<>[])
      or ((Proc.Parent.ClassType=TPasClassType)
        and (TPasClassType(Proc.Parent).ObjKind=okInterface)) then
    UseOverrides(Proc);

  if Proc.Parent is TPasClassType then
    begin
    ClassScope:=TPasClassScope(Proc.Parent.CustomData);
    ClassEl:=TPasClassType(ClassScope.Element);
    if (ClassEl.ObjKind=okInterface) and IsTypeInfoUsed(ClassEl) then
      UseTypeInfo(Proc);
    if (Proc.ClassType=TPasConstructor) or (Proc.ClassType=TPasDestructor) then
      begin
      if ClassScope.AncestorScope=nil then
        begin
        // root class constructor -> mark AfterConstruction
        if Proc.ClassType=TPasConstructor then
          Name:='AfterConstruction'
        else
          Name:='BeforeDestruction';
        Identifier:=ClassScope.FindLocalIdentifier(Name);
        while Identifier<>nil do
          begin
          El:=Identifier.Element;
          if (El.ClassType=TPasProcedure)
              and (TPasProcedure(El).ProcType.Args.Count=0) then
            begin
            UseProcedure(TPasProcedure(El));
            break;
            end;
          Identifier:=Identifier.NextSameIdentifier;
          end;
        end;
      end;
    end;
end;

procedure TPasAnalyzer.UseProcedureType(ProcType: TPasProcedureType);
var
  i: Integer;
  Arg: TPasArgument;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseProcedureType ',GetElModName(ProcType));
  {$ENDIF}
  if not MarkElementAsUsed(ProcType) then exit;
  if CanSkipGenericType(ProcType) then
    RaiseNotSupported(20190817151651,ProcType);

  for i:=0 to ProcType.Args.Count-1 do
    begin
    Arg:=TPasArgument(ProcType.Args[i]);
    // Note: the arguments themselves are marked when used in code
    // mark argument type and default value
    UseElType(ProcType,Arg.ArgType,paumElement);
    UseExpr(Arg.ValueExpr);
    end;
  if ProcType is TPasFunctionType then
    UseElType(ProcType,TPasFunctionType(ProcType).ResultEl.ResultType,paumElement);
end;

procedure TPasAnalyzer.UseType(El: TPasType; Mode: TPAUseMode);
var
  C: TClass;
  i: Integer;
  ArrType: TPasArrayType;
begin
  if El=nil then exit;

  C:=El.ClassType;
  if Mode=paumAllExports then
    begin
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.UseType searching exports in ',GetElModName(El),' ...');
    {$ENDIF}
    if (C=TPasRecordType) or (C=TPasClassType) then
      UseClassOrRecType(TPasMembersType(El),Mode);
    end
  else
    begin
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.UseType using ',GetElModName(El),' Mode=',Mode);
    {$ENDIF}
    if C=TPasUnresolvedSymbolRef then
      begin
      if (El.CustomData is TResElDataBaseType)
          or (El.CustomData is TResElDataBuiltInProc) then
      else
        RaiseNotSupported(20170307101353,El);
      end
    else if (C=TPasAliasType)
        or (C=TPasTypeAliasType)
        or (C=TPasClassOfType) then
      begin
      if not MarkElementAsUsed(El) then exit;
      UseElType(El,TPasAliasType(El).DestType,Mode);
      if C=TPasTypeAliasType then
        UseExpr(TPasTypeAliasType(El).Expr);
      end
    else if C=TPasArrayType then
      begin
      ArrType:=TPasArrayType(El);
      if CanSkipGenericType(ArrType) then exit;
      if not MarkElementAsUsed(ArrType) then exit;
      for i:=0 to length(ArrType.Ranges)-1 do
        UseExpr(ArrType.Ranges[i]);
      UseElType(El,ArrType.ElType,Mode);
      end
    else if (C=TPasRecordType) or (C=TPasClassType) then
      UseClassOrRecType(TPasMembersType(El),Mode)
    else if C=TPasEnumType then
      begin
      if not MarkElementAsUsed(El) then exit;
      for i:=0 to TPasEnumType(El).Values.Count-1 do
        UseElement(TPasEnumValue(TPasEnumType(El).Values[i]),rraRead,false);
      end
    else if C=TPasPointerType then
      begin
      if not MarkElementAsUsed(El) then exit;
      UseElType(El,TPasPointerType(El).DestType,Mode);
      end
    else if C=TPasRangeType then
      begin
      if not MarkElementAsUsed(El) then exit;
      UseExpr(TPasRangeType(El).RangeExpr);
      end
    else if C=TPasSetType then
      begin
      if not MarkElementAsUsed(El) then exit;
      UseElType(El,TPasSetType(El).EnumType,Mode);
      end
    else if C.InheritsFrom(TPasProcedureType) then
      begin
      if CanSkipGenericType(TPasProcedureType(El)) then exit;
      UseProcedureType(TPasProcedureType(El));
      end
    else if C=TPasSpecializeType then
      UseSpecializeType(TPasSpecializeType(El),Mode)
    else if C=TPasGenericTemplateType then
    else
      RaiseNotSupported(20170306170315,El);

    if Mode=paumAllPasUsable then
      UseTypeInfo(El);
    end;
end;

procedure TPasAnalyzer.UseClassOrRecType(El: TPasMembersType; Mode: TPAUseMode);
// called by UseType

  procedure UseDelegations;
  var
    OverrideList: TPAOverrideList;
    i: Integer;
    Prop: TPasProperty;
  begin
    OverrideList:=TPAOverrideList(FOverrideLists.FindKey(El));
    if OverrideList=nil then exit;
    // Note: while traversing the OverrideList it may grow
    i:=0;
    while i<OverrideList.Count do
      begin
      Prop:=TObject(OverrideList.Overrides[i]) as TPasProperty;
      UseVariable(Prop,rraRead,false);
      inc(i);
      end;
  end;

  procedure MarkAllInterfaceImplementations(Scope: TPasClassScope);
  var
    i, j: Integer;
    o: TObject;
    Map: TPasClassIntfMap;
  begin
    if Scope.Interfaces=nil then exit;
    for i:=0 to Scope.Interfaces.Count-1 do
      begin
      o:=TObject(Scope.Interfaces[i]);
      if o is TPasProperty then
        UseVariable(TPasProperty(o),rraRead,false)
      else if o is TPasClassIntfMap then
        begin
        Map:=TPasClassIntfMap(o);
        repeat
          if Map.Intf<>nil then
            UseClassOrRecType(TPasClassType(Map.Intf),paumElement);
          if Map.Procs<>nil then
            for j:=0 to Map.Procs.Count-1 do
              UseProcedure(TPasProcedure(Map.Procs[j]));
          Map:=Map.AncestorMap;
        until Map=nil;
        end
      else
        RaiseNotSupported(20180405190114,El,GetObjName(o));
      end;
  end;

var
  i: Integer;
  Member: TPasElement;
  AllPublished, FirstTime, IsCOMInterfaceRoot: Boolean;
  ProcScope: TPasProcedureScope;
  ClassScope: TPasClassScope;
  Ref: TResolvedReference;
  j: Integer;
  List, ProcList: TFPList;
  o: TObject;
  Map: TPasClassIntfMap;
  ImplProc, IntfProc, Proc: TPasProcedure;
  aClass: TPasClassType;
begin
  FirstTime:=true;
  case Mode of
  paumAllExports: exit;
  paumAllPasUsable:
    begin
    if CanSkipGenericType(El) then exit;
    if MarkElementAsUsed(El) then
      ElementVisited(El,Mode)
    else
      begin
      if ElementVisited(El,Mode) then exit;
      // this class has been used (e.g. paumElement), which marked ancestors
      // and published members
      // -> now mark all members paumAllPasUsable
      FirstTime:=false;
      end;
    end;
  paumElement:
    begin
    if CanSkipGenericType(El) then exit;
    if not MarkElementAsUsed(El) then exit;
    end
  else
    RaiseInconsistency(20170414152143,IntToStr(ord(Mode)));
  end;
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseClassOrRecType ',GetElModName(El),' ',Mode,' First=',FirstTime);
  {$ENDIF}
  aClass:=nil;
  ClassScope:=nil;
  IsCOMInterfaceRoot:=false;

  if El.ClassType=TPasClassType then
    begin
    aClass:=TPasClassType(El);
    if aClass.IsForward then
      begin
      Ref:=aClass.CustomData as TResolvedReference;
      UseClassOrRecType(Ref.Declaration as TPasClassType,Mode);
      exit;
      end;

    ClassScope:=aClass.CustomData as TPasClassScope;

    if FirstTime then
      begin
      UseElType(El,ClassScope.DirectAncestor,paumElement);
      UseElType(El,aClass.HelperForType,paumElement);
      UseExpr(aClass.GUIDExpr);
      // aClass.Interfaces: using a class does not use automatically the interfaces
      if aClass.ObjKind=okInterface then
        begin
        UseDelegations;
        if (aClass.InterfaceType=citCom) and (aClass.AncestorType=nil) then
          IsCOMInterfaceRoot:=true;
        end;
      if (aClass.ObjKind=okClass) and (ScopeModule<>nil)
          and (ClassScope.Interfaces<>nil) then
        // when checking a single unit, mark all method+properties implementing the interfaces
        MarkAllInterfaceImplementations(ClassScope);
      end;
    end
  else if El is TPasRecordType then
    begin
    if (Mode<>paumAllPasUsable) and Resolver.IsTGUID(TPasRecordType(El)) then
      for i:=0 to El.Members.Count-1 do
        begin
        Member:=TPasElement(El.Members[i]);
        if Member is TPasVariable then
          UseVariable(TPasVariable(Member),rraNone,true);
        end;
    end
  else
    RaiseNotSupported(20181229103139,El);

  // members
  AllPublished:=(Mode<>paumAllExports);
  for i:=0 to El.Members.Count-1 do
    begin
    Member:=TPasElement(El.Members[i]);
    if FirstTime and (Member is TPasProcedure) then
      begin
      Proc:=TPasProcedure(Member);
      ProcScope:=Member.CustomData as TPasProcedureScope;
      if Proc.IsOverride and (ProcScope.OverriddenProc<>nil) then
        begin
        // this is an override
        AddOverride(ProcScope.OverriddenProc,Member);
        if ScopeModule<>nil then
          begin
          // when analyzing a single module, all overrides are assumed to be called
          UseProcedure(Proc);
          continue;
          end;
        end
      else if (Proc.ClassType=TPasClassConstructor)
          or (Proc.ClassType=TPasClassDestructor) then
        begin
        UseProcedure(Proc);
        continue;
        end;
      if IsCOMInterfaceRoot then
        begin
        case lowercase(Member.Name) of
        'queryinterface':
          if (Proc.ProcType.Args.Count=2) then
            begin
            UseProcedure(Proc);
            continue;
            end;
        '_addref':
          if Proc.ProcType.Args.Count=0 then
            begin
            UseProcedure(Proc);
            continue;
            end;
        '_release':
          if Proc.ProcType.Args.Count=0 then
            begin
            UseProcedure(Proc);
            continue;
            end;
        end;
        //writeln('TPasAnalyzer.UseClassType ',El.FullName,' ',Mode,' ',Member.Name);
        end;
      if Proc.MessageExpr<>nil then
        begin
        UseProcedure(Proc);
        continue;
        end;
      end
    else if Member.ClassType=TPasAttributes then
      continue; // attributes are never used directly

    if AllPublished and (Member.Visibility=visPublished) then
      begin
      // include published
      if not FirstTime then continue;
      UseTypeInfo(Member);
      end
    else if Mode=paumElement then
      continue
    else if IsModuleInternal(Member) then
      // private or strict private
      continue
    else if (Mode=paumAllPasUsable) and FirstTime
        and ((Member.ClassType=TPasProperty) or (Member is TPasType)) then
      begin
      // non private property can be used by typeinfo by descendants in other units
      UseTypeInfo(Member);
      end
    else
      ; // else: class/record is in unit interface, mark all non private members
    UseElement(Member,rraNone,true);
    end;

  if FirstTime and (ClassScope<>nil) then
    begin
    // method resolution
    List:=ClassScope.Interfaces;
    if List<>nil then
      for i:=0 to List.Count-1 do
        begin
        o:=TObject(List[i]);
        if o is TPasProperty then
          begin
          // interface delegation
          // Note: This class is used. When the intftype is used, this delegation is used.
          AddOverride(TPasType(aClass.Interfaces[i]),TPasProperty(o));
          end
        else if o is TPasClassIntfMap then
          begin
          Map:=TPasClassIntfMap(o);
          while Map<>nil do
            begin
            ProcList:=Map.Procs;
            if ProcList<>nil then
              for j:=0 to ProcList.Count-1 do
                begin
                ImplProc:=TPasProcedure(ProcList[j]);
                if ImplProc=nil then continue;
                IntfProc:=TObject(Map.Intf.Members[j]) as TPasProcedure;
                // This class is used. When the interface method is used, this method is used.
                AddOverride(IntfProc,ImplProc);
                end;
            Map:=Map.AncestorMap;
            end;
          end
        else
          RaiseNotSupported(20180328224632,aClass,GetObjName(o));
        end;
    end;
end;

procedure TPasAnalyzer.UseClassConstructor(El: TPasMembersType);
var
  i: Integer;
  Member: TPasElement;
begin
  if ElementVisited(El,pocClassConstructor) then exit;
  for i:=0 to El.Members.Count-1 do
    begin
    Member:=TPasElement(El.Members[i]);
    if (Member.ClassType=TPasClassConstructor) or (Member.ClassType=TPasClassDestructor) then
      UseProcedure(TPasProcedure(Member));
    end;
end;

procedure TPasAnalyzer.UseSpecializeType(El: TPasSpecializeType;
  Mode: TPAUseMode);
var
  Param: TPasElement;
  i: Integer;
begin
  if not MarkElementAsUsed(El) then exit;
  // El.DestType is TPasGenericType, which is never be used
  if El.CustomData is TPasSpecializeTypeData then
    UseElType(El,TPasSpecializeTypeData(El.CustomData).SpecializedType,Mode);
  for i:=0 to El.Params.Count-1 do
    begin
    Param:=TPasElement(El.Params[i]);
    if Param is TPasGenericTemplateType then continue;
    UseElement(Param,rraRead,false);
    end;
end;

procedure TPasAnalyzer.UseVariable(El: TPasVariable;
  Access: TResolvedRefAccess; UseFull: boolean);
var
  Usage: TPAElement;
  UseRead, UseWrite: boolean;

  procedure UpdateVarAccess(IsRead, IsWrite: boolean);
  begin
    if IsRead then
      case Usage.Access of
        paiaNone: begin Usage.Access:=paiaRead; UseRead:=true; end;
        paiaRead: ;
        paiaWrite: begin Usage.Access:=paiaWriteRead; UseRead:=true; end;
        paiaReadWrite: ;
        paiaWriteRead: ;
        else RaiseInconsistency(20170311182420,'');
      end;
    if IsWrite then
      case Usage.Access of
        paiaNone: begin Usage.Access:=paiaWrite; UseWrite:=true; end;
        paiaRead: begin Usage.Access:=paiaReadWrite; UseWrite:=true; end;
        paiaWrite: ;
        paiaReadWrite: ;
        paiaWriteRead: ;
        else RaiseInconsistency(20170311182536,'');
      end;
  end;

var
  Prop: TPasProperty;
  i: Integer;
  IsRead, IsWrite, CanRead, CanWrite: Boolean;
  ClassEl: TPasClassType;
begin
  if El=nil then exit;
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.UseVariable ',GetElModName(El),' ',Access,' Full=',UseFull);
  {$ENDIF}

  if El.ClassType=TPasProperty then
    begin
    Prop:=TPasProperty(El);
    if Prop.Parent is TPasClassType then
      begin
      ClassEl:=TPasClassType(Prop.Parent);
      if (ClassEl.ObjKind=okInterface) and IsTypeInfoUsed(ClassEl) then
        begin
        UseFull:=true;
        UseTypeInfo(Prop);
        end;
      end;
    end
  else
    Prop:=nil;

  IsRead:=false;
  IsWrite:=false;
  if UseFull then
    if (Prop<>nil) then
      begin
      CanRead:=Resolver.GetPasPropertyGetter(Prop)<>nil;
      CanWrite:=Resolver.GetPasPropertySetter(Prop)<>nil;
      if CanRead then
        begin
        if CanWrite then
          Access:=rraReadAndAssign
        else
          Access:=rraRead;
        end
      else
        if CanWrite then
          Access:=rraAssign
        else
          Access:=rraNone;
      end
    else
      Access:=rraRead;
  case Access of
    rraNone: ;
    rraRead: IsRead:=true;
    rraAssign: IsWrite:=true;
    rraReadAndAssign,
    rraVarParam,
    rraOutParam: begin IsRead:=true; IsWrite:=true; end;
    rraParamToUnknownProc: RaiseInconsistency(20170307153439,'');
  else
    RaiseInconsistency(20170308120949,'');
  end;

  UseRead:=false;
  UseWrite:=false;
  if MarkElementAsUsed(El) then
    begin
    // first access of this variable
    Usage:=FindElement(El);
    // first set flags
    if El.Expr<>nil then
      Usage.Access:=paiaWrite;
    UpdateVarAccess(IsRead,IsWrite);
    // then use recursively
    UseElType(El,El.VarType,paumElement);
    UseExpr(El.Expr);
    UseExpr(El.LibraryName);
    UseExpr(El.ExportName);
    UseExpr(El.AbsoluteExpr);
    if Prop<>nil then
      begin
      for i:=0 to Prop.Args.Count-1 do
        UseElType(Prop,TPasArgument(Prop.Args[i]).ArgType,paumElement);
      UseExpr(Prop.IndexExpr);
      // ToDo: UseExpr(Prop.DispIDExpr);
      // see UseTypeInfo: Prop.StoredAccessor, Prop.DefaultExpr
      end;
    end
  else
    begin
    Usage:=FindElement(El);
    if Usage=nil then
      exit; // element outside of scope
    // var is accessed another time

    // first update flags
    UpdateVarAccess(IsRead,IsWrite);
    end;
  // then use recursively
  if Prop<>nil then
    begin
    {$IFDEF VerbosePasAnalyzer}
    writeln('TPasAnalyzer.UseVariable Property=',Prop.FullName,
      ' Ancestor=',GetElModName(Resolver.GetPasPropertyAncestor(Prop)),
      ' UseRead=',UseRead,',Acc=',GetElModName(Resolver.GetPasPropertyGetter(Prop)),
      ' UseWrite=',UseWrite,',Acc=',GetElModName(Resolver.GetPasPropertySetter(Prop)),
      '');
    {$ENDIF}
    if UseRead then
      UseElement(Resolver.GetPasPropertyGetter(Prop),rraRead,false);
    if UseWrite then
      UseElement(Resolver.GetPasPropertySetter(Prop),rraAssign,false);
    end;
end;

procedure TPasAnalyzer.UseResourcestring(El: TPasResString);
begin
  if not MarkElementAsUsed(El) then exit;
  UseExpr(El.Expr);
end;

procedure TPasAnalyzer.UseArgument(El: TPasArgument; Access: TResolvedRefAccess
  );
var
  Usage: TPAElement;
  IsRead, IsWrite: Boolean;
begin
  IsRead:=false;
  IsWrite:=false;
  case Access of
    rraNone: ;
    rraRead: IsRead:=true;
    rraAssign: IsWrite:=true;
    rraReadAndAssign,
    rraVarParam,
    rraOutParam: begin IsRead:=true; IsWrite:=true; end;
    rraParamToUnknownProc: RaiseInconsistency(20170308121031,'');
  else
    RaiseInconsistency(20170308121037,'');
  end;
  if MarkElementAsUsed(El) then
    begin
    // first time
    Usage:=FindElement(El);
    end
  else
    begin
    // used again
    Usage:=FindElement(El);
    if Usage=nil then
      RaiseNotSupported(20170308121928,El);
    end;
  UpdateAccess(IsWrite, IsRead, Usage);
end;

procedure TPasAnalyzer.UseResultElement(El: TPasResultElement;
  Access: TResolvedRefAccess);
var
  IsRead, IsWrite: Boolean;
  Usage: TPAElement;
begin
  IsRead:=false;
  IsWrite:=false;
  case Access of
    rraNone: ;
    rraRead: IsRead:=true;
    rraAssign: IsWrite:=true;
    rraReadAndAssign,
    rraVarParam,
    rraOutParam: begin IsRead:=true; IsWrite:=true; end;
    rraParamToUnknownProc: RaiseInconsistency(20170308122319,'');
  else
    RaiseInconsistency(20170308122324,'');
  end;
  if MarkElementAsUsed(El) then
    begin
    // first time
    Usage:=FindElement(El);
    end
  else
    begin
    // used again
    Usage:=FindElement(El);
    if Usage=nil then
      RaiseNotSupported(20170308122333,El);
    end;
  UpdateAccess(IsWrite, IsRead, Usage);
end;

procedure TPasAnalyzer.EmitElementHints(El: TPasElement);
var
  C: TClass;
begin
  if El=nil then exit;

  C:=El.ClassType;
  if C.InheritsFrom(TPasVariable) then
    EmitVariableHints(TPasVariable(El))
  else if C.InheritsFrom(TPasType) then
    EmitTypeHints(TPasType(El))
  else if C.InheritsFrom(TPasProcedure) then
    EmitProcedureHints(TPasProcedure(El))
  else if C=TPasMethodResolution then
  else
    RaiseInconsistency(20170312093126,'');
end;

procedure TPasAnalyzer.EmitSectionHints(Section: TPasSection);
var
  i: Integer;
  UsedModule, aModule: TPasModule;
  UsesClause: TPasUsesClause;
  Use: TPasUsesUnit;
  PosEl: TPasElement;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitSectionHints ',GetElModName(Section));
  {$ENDIF}
  // initialization, program or library sections
  aModule:=Section.GetModule;
  UsesClause:=Section.UsesClause;
  for i:=0 to length(UsesClause)-1 do
    begin
    Use:=UsesClause[i];
    if Use.Module is TPasModule then
      begin
      UsedModule:=TPasModule(Use.Module);
      if CompareText(UsedModule.Name,'system')=0 then continue;
      if not PAElementExists(UsedModule) then
        begin
        PosEl:=Use.Expr;
        if PosEl=nil then PosEl:=Use;
        EmitMessage(20170311191725,mtHint,nPAUnitNotUsed,sPAUnitNotUsed,
          [UsedModule.Name,aModule.Name],PosEl);
        end;
      end;
    end;

  EmitDeclarationsHints(Section);
end;

procedure TPasAnalyzer.EmitDeclarationsHints(El: TPasDeclarations);
var
  i: Integer;
  Decl: TPasElement;
  Usage: TPAElement;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitDeclarationsHints ',GetElModName(El));
  {$ENDIF}
  for i:=0 to El.Declarations.Count-1 do
    begin
    Decl:=TPasElement(El.Declarations[i]);
    if Decl is TPasVariable then
      EmitVariableHints(TPasVariable(Decl))
    else if Decl is TPasType then
      EmitTypeHints(TPasType(Decl))
    else if Decl is TPasProcedure then
      EmitProcedureHints(TPasProcedure(Decl))
    else if Decl.ClassType=TPasAttributes then
      // no hints
    else
      begin
      Usage:=FindElement(Decl);
      if Usage=nil then
        begin
        // declaration was never used
        if IsSpecializedGenericType(Decl) then
          continue;
        EmitMessage(20170311231734,mtHint,nPALocalXYNotUsed,
          sPALocalXYNotUsed,[Decl.ElementTypeName,Decl.Name],Decl);
        end;
      end;
    end;
end;

procedure TPasAnalyzer.EmitTypeHints(El: TPasType);
var
  C: TClass;
  Usage: TPAElement;
  i: Integer;
  Member, SpecEl: TPasElement;
  Members: TFPList;
  GenScope: TPasGenericScope;
  SpecializedItems: TObjectList;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitTypeHints ',GetElModName(El));
  {$ENDIF}
  Usage:=FindElement(El);
  if Usage=nil then
    begin
    // the whole type was never used
    if IsSpecializedGenericType(El) then
      exit; // no hints for not used specializations
    if (El.CustomData is TPasGenericScope) then
      begin
      GenScope:=TPasGenericScope(El.CustomData);
      SpecializedItems:=GenScope.SpecializedItems;
      if SpecializedItems<>nil then
        for i:=0 to SpecializedItems.Count-1 do
          begin
          SpecEl:=TPRSpecializedItem(SpecializedItems[i]).SpecializedEl;
          if FindElement(SpecEl)<>nil then
            exit; // a specialization of this generic type is used
          end;
      end;

    if (El.Visibility in [visPrivate,visStrictPrivate]) then
      EmitMessage(20170312000020,mtHint,nPAPrivateTypeXNeverUsed,
        sPAPrivateTypeXNeverUsed,[El.FullName],El)
    else
      begin
      if (El is TPasClassType) and (TPasClassType(El).ObjKind=okInterface) then
        exit;

      EmitMessage(20170312000025,mtHint,nPALocalXYNotUsed,
        sPALocalXYNotUsed,[El.ElementTypeName,GetElementNameAndParams(El)],El);
      end;
    exit;
    end;
  // emit hints for sub elements
  Members:=nil;
  C:=El.ClassType;
  if C=TPasRecordType then
    Members:=TPasRecordType(El).Members
  else if C=TPasClassType then
    begin
    if TPasClassType(El).IsForward then exit;
    Members:=TPasClassType(El).Members;
    end;
  if Members<>nil then
    for i:=0 to Members.Count-1 do
      begin
      Member:=TPasElement(Members[i]);
      if Member.ClassType=TPasAttributes then continue;
      EmitElementHints(Member);
      end;
end;

procedure TPasAnalyzer.EmitVariableHints(El: TPasVariable);
var
  Usage: TPAElement;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitVariableHints ',GetElModName(El));
  {$ENDIF}
  Usage:=FindElement(El);
  if Usage=nil then
    begin
    // not used
    if El.Visibility in [visPrivate,visStrictPrivate] then
      begin
      if El.ClassType=TPasConst then
        EmitMessage(20170311234602,mtHint,nPAPrivateConstXNeverUsed,
          sPAPrivateConstXNeverUsed,[El.FullName],El)
      else if El.ClassType=TPasProperty then
        EmitMessage(20170311234634,mtHint,nPAPrivatePropertyXNeverUsed,
          sPAPrivatePropertyXNeverUsed,[El.FullName],El)
      else
        EmitMessage(20170311231412,mtHint,nPAPrivateFieldIsNeverUsed,
          sPAPrivateFieldIsNeverUsed,[El.FullName],El);
      end
    else if El.ClassType=TPasVariable then
      EmitMessage(20170311234201,mtHint,nPALocalVariableNotUsed,
        sPALocalVariableNotUsed,[El.Name],El)
    else
      EmitMessage(20170314221334,mtHint,nPALocalXYNotUsed,
        sPALocalXYNotUsed,[El.ElementTypeName,El.Name],El);
    end
  else if Usage.Access=paiaWrite then
    begin
    // write without read
    if (vmExternal in El.VarModifiers)
        or ((El.Parent is TPasClassType) and TPasClassType(El.Parent).IsExternal) then
      exit;
    if El.Visibility in [visPrivate,visStrictPrivate] then
      EmitMessage(20170311234159,mtHint,nPAPrivateFieldIsAssignedButNeverUsed,
        sPAPrivateFieldIsAssignedButNeverUsed,[El.FullName],El)
    else
      EmitMessage(20170311233825,mtHint,nPALocalVariableIsAssignedButNeverUsed,
        sPALocalVariableIsAssignedButNeverUsed,[El.Name],El);
    end;
end;

procedure TPasAnalyzer.EmitProcedureHints(El: TPasProcedure);
var
  Args: TFPList;
  i: Integer;
  Arg: TPasArgument;
  Usage: TPAElement;
  ProcScope: TPasProcedureScope;
  DeclProc, ImplProc: TPasProcedure;
  SpecializedItems: TObjectList;
  SpecEl: TPasElement;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitProcedureHints ',GetElModName(El));
  {$ENDIF}
  ProcScope:=El.CustomData as TPasProcedureScope;
  if ProcScope.DeclarationProc=nil then
    DeclProc:=El
  else
    DeclProc:=ProcScope.DeclarationProc;
  if ProcScope.ImplProc=nil then
    ImplProc:=El
  else
    ImplProc:=ProcScope.ImplProc;
  if (ProcScope.ClassRecScope<>nil)
      and (ProcScope.ClassRecScope.SpecializedFromItem<>nil) then
    exit; // specialized proc

  if not PAElementExists(DeclProc) then
    begin
    // procedure never used
    if ProcScope.DeclarationProc<>nil then
      exit;

    if ProcScope.SpecializedFromItem<>nil then
      exit; // no hint for not used specialized procedure
    SpecializedItems:=ProcScope.SpecializedItems;
    if SpecializedItems<>nil then
      for i:=0 to SpecializedItems.Count-1 do
        begin
        SpecEl:=TPRSpecializedItem(SpecializedItems[i]).SpecializedEl;
        if FindElement(SpecEl)<>nil then
          exit; // a specialization of this generic procedure is used
        end;

    if El.Visibility in [visPrivate,visStrictPrivate] then
      EmitMessage(20170312093348,mtHint,nPAPrivateMethodIsNeverUsed,
        sPAPrivateMethodIsNeverUsed,[El.FullName],El)
    else
      EmitMessage(20170312093418,mtHint,nPALocalXYNotUsed,
        sPALocalXYNotUsed,[El.ElementTypeName,El.Name],El);
    exit;
    end;

  // procedure was used

  if [pmAbstract,pmAssembler,pmExternal]*DeclProc.Modifiers<>[] then exit;
  if [pmAssembler]*ImplProc.Modifiers<>[] then exit;
  if El.Parent is TPasClassType then
    begin
    if TPasClassType(El.Parent).ObjKind=okInterface then exit;
    end;

  if ProcScope.DeclarationProc=nil then
    begin
    // check parameters
    Args:=El.ProcType.Args;
    for i:=0 to Args.Count-1 do
      begin
      Arg:=TPasArgument(Args[i]);
      Usage:=FindElement(Arg);
      if (Usage=nil) or (Usage.Access=paiaNone) then
        begin
        // parameter was never used
        if (Arg.Parent is TPasProcedureType) and (Arg.Parent.Parent is TPasProcedure)
            and ([pmVirtual,pmOverride]*TPasProcedure(Arg.Parent.Parent).Modifiers<>[]) then
          EmitMessage(20180625153623,mtHint,nPAParameterInOverrideNotUsed,
            sPAParameterInOverrideNotUsed,[Arg.Name],Arg)
        else
          EmitMessage(20170312094401,mtHint,nPAParameterNotUsed,
            sPAParameterNotUsed,[Arg.Name],Arg);
        end
      else
        begin
        // parameter was used
        if (Usage.Access=paiaWrite) and not (Arg.Access in [argOut,argVar]) then
          EmitMessage(20170312095348,mtHint,nPAValueParameterIsAssignedButNeverUsed,
            sPAValueParameterIsAssignedButNeverUsed,[Arg.Name],Arg);
        end;
      end;
    // check result
    if (El.ProcType is TPasFunctionType) then
      EmitFunctionResultHints(TPasFunction(El));
    end;

  if El.Body<>nil then
    begin
    // check declarations
    EmitDeclarationsHints(El.Body);
    // ToDo: emit hints for statements
    end;
end;

procedure TPasAnalyzer.EmitFunctionResultHints(El: TPasFunction);
var
  FuncType: TPasFunctionType;
  Usage: TPAElement;
  TypeEl: TPasType;
  Members: TFPList;
  i: Integer;
  Member: TPasElement;
  HasFields: Boolean;
  PosEl: TPasResultElement;
  ProcScope: TPasProcedureScope;
begin
  FuncType:=El.FuncType;
  Usage:=FindElement(FuncType.ResultEl);
  if (Usage=nil) or (Usage.Access in [paiaNone,paiaRead]) then
    begin
    // result was never set
    TypeEl:=Resolver.ResolveAliasType(FuncType.ResultEl.ResultType);
    if TypeEl is TPasRecordType then
      begin
      Members:=TPasRecordType(TypeEl).Members;
      HasFields:=false;
      for i:=0 to Members.Count-1 do
        begin
        Member:=TPasElement(Members[i]);
        if Member.ClassType=TPasVariable then
          begin
          HasFields:=true;
          break;
          end;
        end;
      if not HasFields then
        // empty record -> no hint
        exit;
      end;
    PosEl:=FuncType.ResultEl;
    ProcScope:=El.CustomData as TPasProcedureScope;
    if (ProcScope.ImplProc<>nil)
        and (TPasFunction(ProcScope.ImplProc).FuncType.ResultEl<>nil) then
      PosEl:=TPasFunction(ProcScope.ImplProc).FuncType.ResultEl;
    EmitMessage(20170313214038,mtHint,nPAFunctionResultDoesNotSeemToBeSet,
      sPAFunctionResultDoesNotSeemToBeSet,[],PosEl)
    end
  else
    begin
    // result was used
    end;
end;

constructor TPasAnalyzer.Create;
var
  m: TPAUseMode;
  oc: TPAOtherCheckedEl;
begin
  CreateTree;
  for m in TPAUseMode do
    FModeChecked[m]:=TPasAnalyzerKeySet.Create(
      {$ifdef pas2js}
      @PasElementToHashName
      {$else}
      @ComparePointer
      {$endif}
      ,nil
      );
  for oc in TPAOtherCheckedEl do
    FOtherChecked[oc]:=TPasAnalyzerKeySet.Create(
      {$ifdef pas2js}
      @PasElementToHashName
      {$else}
      @ComparePointer
      {$endif}
      ,nil
      );
  FOverrideLists:=TPasAnalyzerKeySet.Create(
    {$ifdef pas2js}
    @PAOverrideList_ElToHashName,@PasElementToHashName
    {$else}
    @ComparePAOverrideLists,@CompareElementWithPAOverrideList
    {$endif});
end;

destructor TPasAnalyzer.Destroy;
var
  m: TPAUseMode;
  oc: TPAOtherCheckedEl;
begin
  Clear;
  FreeAndNil(FOverrideLists);
  FreeAndNil(FUsedElements);
  for m in TPAUseMode do
    FreeAndNil(FModeChecked[m]);
  for oc in TPAOtherCheckedEl do
    FreeAndNil(FOtherChecked[oc]);
  inherited Destroy;
end;

procedure TPasAnalyzer.Clear;
var
  m: TPAUseMode;
  oc: TPAOtherCheckedEl;
begin
  FOverrideLists.FreeItems;
  FUsedElements.FreeItems;
  for m in TPAUseMode do
    FModeChecked[m].Clear;
  for oc in TPAOtherCheckedEl do
    FOtherChecked[oc].Clear;
end;

procedure TPasAnalyzer.AnalyzeModule(aModule: TPasModule);
var
  Mode: TPAUseMode;
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.AnalyzeModule START ',GetElModName(aModule));
  {$ENDIF}
  if Resolver=nil then
    RaiseInconsistency(20170314223032,'TPasAnalyzer.AnalyzeModule missing Resolver');
  if FUsedElements.Count>0 then
    RaiseInconsistency(20170315153243,'');
  ScopeModule:=aModule;
  if (aModule is TPasProgram) or (aModule is TPasLibrary) then
    Mode:=paumAllExports
  else
    Mode:=paumAllPasUsable;
  UseModule(aModule,Mode);
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.AnalyzeModule END ',GetElModName(aModule));
  {$ENDIF}
end;

procedure TPasAnalyzer.AnalyzeWholeProgram(aStartModule: TPasProgram);
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.AnalyzeWholeProgram START ',GetElModName(aStartModule));
  {$ENDIF}
  if Resolver=nil then
    RaiseInconsistency(20170315153201,'TPasAnalyzer.AnalyzeWholeProgram missing Resolver');
  if FUsedElements.Count>0 then
    RaiseInconsistency(20170315153252,'');
  ScopeModule:=nil;
  UseModule(aStartModule,paumAllExports);
  MarkElementAsUsed(aStartModule); // always mark the start
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.AnalyzeWholeProgram END ',GetElModName(aStartModule));
  {$ENDIF}
end;

procedure TPasAnalyzer.EmitModuleHints(aModule: TPasModule);
begin
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitModuleHints ',GetElModName(aModule));
  {$ENDIF}
  if aModule.ClassType=TPasProgram then
    EmitSectionHints(TPasProgram(aModule).ProgramSection)
  else if aModule.ClassType=TPasLibrary then
    EmitSectionHints(TPasLibrary(aModule).LibrarySection)
  else
    begin
    // unit
    EmitSectionHints(aModule.InterfaceSection);
    EmitSectionHints(aModule.ImplementationSection);
    end;
  //EmitBlockHints(aModule.InitializationSection);
  //EmitBlockHints(aModule.FinalizationSection);
end;

function TPasAnalyzer.FindUsedElement(El: TPasElement): TPAElement;
var
  ProcScope: TPasProcedureScope;
begin
  if not IsIdentifier(El) then exit(nil);
  if El is TPasProcedure then
    begin
    ProcScope:=El.CustomData as TPasProcedureScope;
    if (ProcScope<>nil) and (ProcScope.DeclarationProc<>nil) then
      El:=ProcScope.DeclarationProc;
    end;
  Result:=FindElement(El);
end;

function TPasAnalyzer.IsUsed(El: TPasElement): boolean;
begin
  Result:=FindUsedElement(El)<>nil;
end;

function TPasAnalyzer.IsTypeInfoUsed(El: TPasElement): boolean;
begin
  Result:=FModeChecked[paumTypeInfo].ContainsItem(El);
end;

function TPasAnalyzer.IsModuleInternal(El: TPasElement): boolean;
begin
  if El=nil then
    exit(true);
  if El.ClassType=TInterfaceSection then
    exit(false);
  if IsExport(El) then exit(false);
  case El.Visibility of
  visPrivate,visStrictPrivate: exit(true);
  visPublished: exit(false);
  end;
  Result:=IsModuleInternal(El.Parent);
end;

function TPasAnalyzer.IsExport(El: TPasElement): boolean;
begin
  if El is TPasVariable then
    Result:=[vmExport,vmPublic]*TPasVariable(El).VarModifiers<>[]
  else if El is TPasProcedure then
    Result:=[pmExport,pmPublic]*TPasProcedure(El).Modifiers<>[]
  else
    Result:=false;
end;

function TPasAnalyzer.IsIdentifier(El: TPasElement): boolean;
var
  C: TClass;
begin
  C:=El.ClassType;
  Result:=C.InheritsFrom(TPasType)
      or C.InheritsFrom(TPasVariable)
      or C.InheritsFrom(TPasProcedure)
      or C.InheritsFrom(TPasModule)
      or (C=TPasArgument)
      or (C=TPasResString);
end;

function TPasAnalyzer.IsImplBlockEmpty(El: TPasImplBlock): boolean;
begin
  Result:=true;
  if (El=nil) or (El.Elements.Count=0) then exit;
  Result:=false;
end;

function TPasAnalyzer.IsSpecializedGenericType(El: TPasElement): boolean;
var
  GenScope: TPasGenericScope;
begin
  if El is TPasGenericType then
    begin
    GenScope:=El.CustomData as TPasGenericScope;
    if (GenScope<>nil) and (GenScope.SpecializedFromItem<>nil) then
      exit(true);
    end;
  Result:=false;
end;

procedure TPasAnalyzer.EmitMessage(Id: TMaxPrecInt; MsgType: TMessageType;
  MsgNumber: integer; Fmt: String;
  const Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
  PosEl: TPasElement);
var
  Msg: TPAMessage;
  El: TPasElement;
  ProcScope: TPasProcedureScope;
  ModScope: TPasModuleScope;
  Scanner: TPascalScanner;
  State: TWarnMsgState;
begin
  {$IFDEF VerbosePasAnalyzer}
  //writeln('TPasAnalyzer.EmitMessage [',Id,'] ',MsgType,': (',MsgNumber,') Fmt={',Fmt,'} PosEl='+GetElModName(PosEl));
  {$ENDIF}
  if MsgType>=mtWarning then
    begin
    El:=PosEl;
    while El<>nil do
      begin
      if El is TPasProcedure then
        begin
        ProcScope:=El.CustomData as TPasProcedureScope;
        if ProcScope.ImplProc<>nil then
          ProcScope:=ProcScope.ImplProc.CustomData as TPasProcedureScope;
        case MsgType of
        mtHint: if not (bsHints in ProcScope.BoolSwitches) then exit;
        mtNote: if not (bsNotes in ProcScope.BoolSwitches) then exit;
        mtWarning: if not (bsWarnings in ProcScope.BoolSwitches) then exit;
        end;
        break;
        end
      else if El is TPasModule then
        begin
        ModScope:=TPasModule(El).CustomData as TPasModuleScope;
        case MsgType of
        mtHint: if not (bsHints in ModScope.BoolSwitches) then exit;
        mtNote: if not (bsNotes in ModScope.BoolSwitches) then exit;
        mtWarning: if not (bsWarnings in ModScope.BoolSwitches) then exit;
        end;
        break;
        end;
      El:=El.Parent;
      end;
    if (Resolver<>nil) and (Resolver.CurrentParser<>nil) then
      begin
      Scanner:=Resolver.CurrentParser.Scanner;
      if Scanner<>nil then
        begin
        State:=Scanner.WarnMsgState[MsgNumber];
        case State of
        wmsOff:
          begin
          {$IFDEF VerbosePasAnalyzer}
          writeln('TPasAnalyzer.EmitMessage ignoring [',Id,'] ',MsgType,': (',MsgNumber,') Fmt={',Fmt,'} PosEl='+GetElModName(PosEl));
          {$ENDIF}
          exit;
          end;
        wmsError:
          MsgType:=mtError;
        end;
        end;
      end;
    end;
  Msg:=TPAMessage.Create;
  Msg.Id:=Id;
  Msg.MsgType:=MsgType;
  Msg.MsgNumber:=MsgNumber;
  Msg.MsgPattern:=Fmt;
  Msg.MsgText:=SafeFormat(Fmt,Args);
  CreateMsgArgs(Msg.Args,Args);
  Msg.PosEl:=PosEl;
  Msg.Filename:=PosEl.SourceFilename;
  Resolver.UnmangleSourceLineNumber(PosEl.SourceLinenumber,Msg.Row,Msg.Col);
  EmitMessage(Msg);
end;

procedure TPasAnalyzer.EmitMessage(Msg: TPAMessage);
begin
  if not Assigned(OnMessage) then
    begin
    Msg.Release;
    exit;
    end;
  {$IFDEF VerbosePasAnalyzer}
  writeln('TPasAnalyzer.EmitMessage [',Msg.Id,'] ',Msg.MsgType,': (',Msg.MsgNumber,') "',Msg.MsgText,'" at ',Resolver.GetElementSourcePosStr(Msg.PosEl),' ScopeModule=',GetObjName(ScopeModule));
  {$ENDIF}
  try
    OnMessage(Self,Msg);
  finally
    Msg.Release;
  end;
end;

class function TPasAnalyzer.GetWarnIdentifierNumbers(Identifier: string; out
  MsgNumbers: TIntegerDynArray): boolean;

  procedure SetNumber(Number: integer);
  begin
    {$IF FPC_FULLVERSION>=30101}
    MsgNumbers:=[Number];
    {$ELSE}
    Setlength(MsgNumbers,1);
    MsgNumbers[0]:=Number;
    {$ENDIF}
  end;

begin
  if Identifier='' then exit(false);
  if Identifier[1] in ['0'..'9'] then exit(false);

  Result:=true;
  case UpperCase(Identifier) of
  // Delphi+FPC
  'NO_RETVAL': SetNumber(nPAFunctionResultDoesNotSeemToBeSet); // Function result is not set.
  else
    Result:=false;
  end;
end;

function TPasAnalyzer.GetUsedElements: TFPList;
begin
  Result:=FUsedElements.GetList;
end;

end.

