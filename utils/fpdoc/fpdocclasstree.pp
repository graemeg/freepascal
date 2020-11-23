unit fpdocclasstree;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DOM, pastree, contnrs;

Type

  { TPasElementNode }

  TPasElementNode = Class
  Private
    FElement : TPasElement;
    FChildren : TFPObjectList;
    function GetChild(aIndex : Integer): TPasElementNode;
    function GetChildCount: Integer;
  Public
    Constructor Create (aElement : TPaselement);
    Destructor Destroy; override;
    Procedure AddChild(C : TPasElementNode);
    Procedure SortChildren;
    Property Element : TPasElement Read FElement;
    Property Children [aIndex : Integer] : TPasElementNode Read GetChild;
    Property ChildCount : Integer Read GetChildCount;
  end;

  { TClassTreeBuilder }

  TClassTreeBuilder = Class
  Private
    // Full name -> TDomElement;
    FElementList : TFPObjectHashTable;
    FObjectKind : TPasObjKind;
    FPackage: TPasPackage;
    FParentObject : TPasClassType;
    FRootNode : TPasElementNode;
    FRootObjectName : string;
  Protected
    function AddToList(aElement: TPasClassType): TPasElementNode;
  Public
    Constructor Create(APackage : TPasPackage; AObjectKind : TPasObjKind = okClass);
    Destructor Destroy; override;
    Function BuildTree(AObjects : TStringList) : Integer;
    Property RootNode : TPasElementNode Read FRootNode;
  end;

implementation

{ TPasElementNode }

function SortOnElementName(Item1, Item2: Pointer): Integer;
begin
  Result:=CompareText(TPasElementNode(Item1).Element.Name,TPasElementNode(Item2).Element.Name);
end;

function TPasElementNode.GetChild(aIndex : Integer): TPasElementNode;
begin
  if Assigned(FChildren) then
    Result:=TPasElementNode(FChildren[aIndex])
  else
    Raise EListError.Create('Index out of range');
end;

function TPasElementNode.GetChildCount: Integer;
begin
  if Assigned(FChildren) then
    Result:=FChildren.Count
  else
    Result:=0
end;

constructor TPasElementNode.Create(aElement: TPaselement);
begin
  FElement:=aElement;
end;

destructor TPasElementNode.Destroy;
begin
  FreeAndNil(FChildren);
  inherited Destroy;
end;

procedure TPasElementNode.AddChild(C: TPasElementNode);
begin
  if FChildren=Nil then
    FChildren:=TFPObjectList.Create(True);
  FChildren.Add(C);
end;

procedure TPasElementNode.SortChildren;
begin
  if Assigned(FChildren) then
    FChildren.Sort(@SortOnElementName);
end;

constructor TClassTreeBuilder.Create(APackage : TPasPackage;
  AObjectKind: TPasObjKind);

begin
  FPackage:=APAckage;
  FObjectKind:=AObjectKind;
  Case FObjectkind of
    okInterface : FRootObjectName:='#rtl.System.IInterface';
    okObject,
    okClass    : FRootObjectName:='#rtl.System.TObject';
  else
    FRootObjectName:='#rtl.System.TObject';
  end;
  FParentObject:=TPasClassType.Create(FRootObjectName,FPackage);
  FParentObject.ObjKind:=FObjectKind;
  FRootNode:=TPasElementNode.Create(FParentObject);
  FElementList:=TFPObjectHashTable.Create(False);
  FElementList.Add(FRootObjectName,FRootNode);
end;

destructor TClassTreeBuilder.Destroy;
begin
  FreeAndNil(FParentObject);
  FreeAndNil(FRootNode);
  FreeAndNil(FElementList);
  Inherited;
end;

Function TClassTreeBuilder.AddToList(aElement : TPasClassType) : TPasElementNode;

Var
  aParentNode : TPasElementNode;
  aName : String;

begin
  if aElement=Nil then
    aName:=FRootObjectName
  else
    begin
    aName:=aElement.PathName;
    end;
  Result:=TPasElementNode(FElementList.Items[aName]);
  if (Result=Nil) then
    begin
    if aElement.AncestorType is TPasClassType then
      aParentNode:=AddToList(aElement.AncestorType as TPasClassType)
    else
      aParentNode:=FRootNode;
    Result:=TPasElementNode.Create(aElement);
    aParentNode.AddChild(Result);
    FElementList.Add(aName,Result);
    end;
end;


Function TClassTreeBuilder.BuildTree(AObjects : TStringList) : Integer;

(*
Procedure DumpNode(Prefix : String; N : TPasElementNode);

  Var
    I : Integer;

  begin
    Writeln(Prefix,N.FElement.Name);
    if Assigned(N.FChildren) then
       For I:=0 to N.FChildren.Count-1 do
         DumpNode(Prefix+'  ',TPasElementNode(N.FChildren[i]));
  end;
*)

Var
  I : Integer;
  PC : TPasClassType;

begin
  Result:=0;
  For I:=0 to AObjects.Count-1 do
    // Advanced records
    if AObjects.Objects[i] is TPasClassType then
      begin
      PC:=AObjects.Objects[i] as TPasClassType;
      AddToList(PC);
      end;
end;



end.

