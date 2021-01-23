unit fpdocclasstree;

{$mode objfpc}{$H+}


interface

uses
  Classes, SysUtils, dGlobals, pastree, contnrs, DOM ,XMLWrite;

Type

  TPasObjKindSet = set of TPasObjKind;

  { TPasElementNode }

  TPasElementNode = Class
  Private
    FElement : TPasType;
    FParentNode: TPasElementNode;
    FChildren : TFPObjectList;
    function GetChild(aIndex : Integer): TPasElementNode;
    function GetChildCount: Integer;
  Public
    Constructor Create (aElement : TPasType);
    Destructor Destroy; override;
    Procedure AddChild(C : TPasElementNode);
    Procedure SortChildren;
    Property Element : TPasType Read FElement;
    Property ParentNode : TPasElementNode read  FParentNode;
    Property Children [aIndex : Integer] : TPasElementNode Read GetChild;
    Property ChildCount : Integer Read GetChildCount;
  end;

  { TClassTreeBuilder }

  TClassTreeBuilder = Class
  Private
    FEngine:TFPDocEngine;
    FElementList : TFPObjectHashTable;
    FObjectKind : TPasObjKindSet;
    FPackage: TPasPackage;
    FParentObject : TPasClassType;
    FRootNode : TPasElementNode;
    FRootObjectName : string;
    FRootObjectPathName : string;
  Protected
    function AddToList(aElement: TPasType): TPasElementNode;
  Public
    Constructor Create(AEngine:TFPDocEngine; APackage : TPasPackage;
                          AObjectKind : TPasObjKindSet = okWithFields);
    Destructor Destroy; override;
    Function BuildTree(AObjects : TStringList) : Integer;
    Procedure SaveToXml(AFileName: String);
    Property RootNode : TPasElementNode Read FRootNode;
    Property PasElToNodes: TFPObjectHashTable read FElementList;
    function GetPasElNode (APasEl: TPasElement) : TPasElementNode;
  end;

implementation

uses
  fpdocstrs, pasresolver;

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

constructor TPasElementNode.Create(aElement: TPasType);
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

constructor TClassTreeBuilder.Create(AEngine:TFPDocEngine; APackage : TPasPackage;
  AObjectKind: TPasObjKindSet);

begin
  FEngine:= AEngine;
  FPackage:= APAckage;
  FObjectKind:=AObjectKind;
  if (okInterface in FObjectkind) then
      begin
        FRootObjectPathName:='#rtl.System.IInterface';
        FRootObjectName:= 'IInterface';
      end
  else if (FObjectkind * okWithFields) <> [] then
      begin
        FRootObjectPathName:='#rtl.System.TObject';
        FRootObjectName:= 'TObject';
      end
  else  // TODO: I don`t know need it ? Without that the code may be simplified.
    begin
      FRootObjectPathName:='#rtl.System.TObject';
      FRootObjectName:= 'TObject';
    end;
  FParentObject:=TPasClassType.Create(FRootObjectName,FEngine.FindModule('System'));
  if not Assigned(FParentObject) then
    FParentObject:=TPasClassType.Create(FRootObjectName,FPackage);
  if (okInterface in FObjectkind) then
      FParentObject.ObjKind:=okInterface
  else if (FObjectkind * okWithFields) <> [] then
    FParentObject.ObjKind:=okClass
  else
    FParentObject.ObjKind:=okClass;
  FRootNode:=TPasElementNode.Create(FParentObject);
  FRootNode.FParentNode := nil;
  FElementList:=TFPObjectHashTable.Create(False);
  FElementList.Add(FRootObjectPathName,FRootNode);
end;

destructor TClassTreeBuilder.Destroy;
begin
  FreeAndNil(FParentObject);
  FreeAndNil(FRootNode);
  FreeAndNil(FElementList);
  Inherited;
end;

function TClassTreeBuilder.AddToList ( aElement: TPasType
  ) : TPasElementNode;

Var
  aParentNode : TPasElementNode;
  aName : String;
  aElementClass: TPasClassType;

begin
  Result:= nil; aElementClass:=nil;
  if  (aElement is TPasClassType) then
    aElementClass:= TPasClassType(aElement);
  if Assigned(aElementClass) and not (aElementClass.ObjKind in FObjectKind) then exit;
  if not Assigned(aElementClass) and not (aElement is TPasAliasType) then exit;

  aParentNode:= nil;
  if aElement=Nil then
    aName:=FRootObjectName
  else if (aElement is TPasAliasType) then
    aName:=TPasAliasType(aElement).DestType.FullName
  else
    aName:=aElement.PathName;
  Result:=TPasElementNode(FElementList.Items[aName]);
  if (Result=Nil) then
  begin
    if Assigned(aElementClass) and (
        (aElementClass.AncestorType is TPasClassType) or
        (aElementClass.AncestorType is TPasAliasType)
                                    ) then
       aParentNode:=AddToList(aElementClass.AncestorType);
    if not Assigned(aParentNode) then
      aParentNode:=FRootNode;
    if (aElement is TPasAliasType) then
      Result:=TPasElementNode.Create(TPasAliasType(TPasType(aElement)).DestType)
    else
      Result:=TPasElementNode.Create(aElement);
    aParentNode.AddChild(Result);
    Result.FParentNode := aParentNode;
    FElementList.Add(aName,Result);
  end;
end;


function TClassTreeBuilder.BuildTree ( AObjects: TStringList ) : Integer;

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

function TClassTreeBuilder.GetPasElNode ( APasEl: TPasElement
  ) : TPasElementNode;
begin
  Result:= TPasElementNode(FElementList.Items[APasEl.PathName]);
end;

procedure TClassTreeBuilder.SaveToXml ( AFileName: String );

  procedure AddPasElChildsToXml (ParentxmlEl : TDOMElement ; ParentPasEl: TPasElementNode ) ;
  var
    CounterVar: Integer;
    PasElNode: TPasElementNode;
    AXmlDoc: TDOMDocument;
    xmlEl: TDOMElement;
    M: TPasModule;
  begin
    if not Assigned(ParentPasEl) or (ParentPasEl.ChildCount = 0) then exit;
    AXmlDoc:= ParentxmlEl.OwnerDocument;
    for CounterVar := 0 to ParentPasEl.ChildCount-1 do
    begin
      PasElNode:= ParentPasEl.Children[CounterVar];
      xmlEl:= AXmlDoc.CreateElement(UTF8Decode(PasElNode.Element.Name));
      M:= PasElNode.Element.GetModule;
      xmlEl['unit'] := UTF8Decode(M.Name);
      xmlEl['package'] := UTF8Decode(M.PackageName);
      xmlEl['type'] := UTF8Decode(GetElementTypeName(PasElNode.Element));
      ParentxmlEl.AppendChild(xmlEl);
      AddPasElChildsToXml(xmlEl, PasElNode);
    end;
  end;

var
  XmlDoc: TXMLDocument;
  XmlRootEl: TDOMElement;
  M: TPasModule;
begin
  XmlDoc:= TXMLDocument.Create;
  XmlDoc.AppendChild(XmlDoc.CreateComment(UTF8Decode(SDocGeneratedByComment)));
  try
    XmlRootEl:= XmlDoc.CreateElement(UTF8Decode(FRootNode.Element.Name));
    M:= FRootNode.Element.GetModule;
    if Assigned(M) then
    begin
      XmlRootEl['unit'] := UTF8Decode(M.Name);
      XmlRootEl['package'] := UTF8Decode(M.PackageName);
      XmlRootEl['type'] := UTF8Decode(GetElementTypeName(FRootNode.Element));
    end
      else
    begin
      XmlRootEl['unit'] := 'system';
      XmlRootEl['package'] := 'rtl';
      if (okWithFields * FObjectKind) <> [] then
        XmlRootEl['type'] := 'class'
      else if (okInterface in FObjectKind) then
        XmlRootEl['type'] := 'interface'
      else
        XmlRootEl['type'] := 'class';
    end;
    XmlDoc.AppendChild(XmlRootEl);
    AddPasElChildsToXml(XmlRootEl, FRootNode);
    WriteXMLFile(XmlDoc, AFileName);
  finally
    XmlDoc.Free;
  end;
end;

end.

