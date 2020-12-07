unit fpdocclasstree;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, dGlobals, pastree, contnrs, DOM ,XMLWrite;

Type

  { TPasElementNode }

  TPasElementNode = Class
  Private
    FElement : TPasClassType;
    FParentNode: TPasElementNode;
    FChildren : TFPObjectList;
    function GetChild(aIndex : Integer): TPasElementNode;
    function GetChildCount: Integer;
  Public
    Constructor Create (aElement : TPasClassType);
    Destructor Destroy; override;
    Procedure AddChild(C : TPasElementNode);
    Procedure SortChildren;
    Property Element : TPasClassType Read FElement;
    Property ParentNode : TPasElementNode read  FParentNode;
    Property Children [aIndex : Integer] : TPasElementNode Read GetChild;
    Property ChildCount : Integer Read GetChildCount;
  end;

  { TClassTreeBuilder }

  TClassTreeBuilder = Class
  Private
    FEngine:TFPDocEngine;
    FElementList : TFPObjectHashTable;
    FObjectKind : TPasObjKind;
    FPackage: TPasPackage;
    FParentObject : TPasClassType;
    FRootNode : TPasElementNode;
    FRootObjectName : string;
    FRootObjectPathName : string;
  Protected
    function AddToList(aElement: TPasClassType): TPasElementNode;
  Public
    Constructor Create(AEngine:TFPDocEngine; APackage : TPasPackage;
                          AObjectKind : TPasObjKind = okClass);
    Destructor Destroy; override;
    Function BuildTree(AObjects : TStringList) : Integer;
{$IFDEF TREE_TEST}
    Procedure SaveToXml(AFileName: String);
{$ENDIF}
    Property RootNode : TPasElementNode Read FRootNode;
    Property PasElToNodes: TFPObjectHashTable read FElementList;
    function GetPasElNode (APasEl: TPasElement) : TPasElementNode;
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

constructor TPasElementNode.Create(aElement: TPasClassType);
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
  AObjectKind: TPasObjKind);

begin
  FEngine:= AEngine;
  FPackage:= APAckage;
  FObjectKind:=AObjectKind;
  Case FObjectkind of
    okInterface :
      begin
        FRootObjectPathName:='#rtl.System.IInterface';
        FRootObjectName:= 'IInterface';
      end;
    okObject, okClass :
      begin
        FRootObjectPathName:='#rtl.System.TObject';
        FRootObjectName:= 'TObject';
      end
  else
    begin
      FRootObjectPathName:='#rtl.System.TObject';
      FRootObjectName:= 'TObject';
    end;
  end;
  FParentObject:=TPasClassType.Create(FRootObjectName,FEngine.FindModule('System'));
  if not Assigned(FParentObject) then
    FParentObject:=TPasClassType.Create(FRootObjectName,FPackage);
  FParentObject.ObjKind:=FObjectKind;
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

function TClassTreeBuilder.AddToList ( aElement: TPasClassType
  ) : TPasElementNode;

Var
  aParentNode : TPasElementNode;
  aName : String;

begin
  Result:= nil;
  if (aElement.ObjKind <> FObjectKind) then exit;
  aParentNode:= nil;
  if aElement=Nil then
    aName:=FRootObjectName
  else
    aName:=aElement.PathName;
  Result:=TPasElementNode(FElementList.Items[aName]);
  if (Result=Nil) then
  begin
    if aElement.AncestorType is TPasClassType then
      aParentNode:=AddToList(aElement.AncestorType as TPasClassType);
    if not Assigned(aParentNode) then
      aParentNode:=FRootNode;
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

{$IFDEF TREE_TEST}
procedure TClassTreeBuilder.SaveToXml ( AFileName: String ) ;

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
      xmlEl:= AXmlDoc.CreateElement(UnicodeString(PasElNode.Element.Name));
      M:= PasElNode.Element.GetModule;
      xmlEl['unit'] := UnicodeString(M.Name);
      xmlEl['package'] := UnicodeString(M.PackageName);
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
  try
    XmlRootEl:= XmlDoc.CreateElement(UnicodeString(FRootNode.Element.Name));
    M:= FRootNode.Element.GetModule;
    if Assigned(M) then
    begin
      XmlRootEl['unit'] := UnicodeString(M.Name);
      XmlRootEl['package'] := UnicodeString(M.PackageName);
    end
      else
    begin
      XmlRootEl['unit'] := 'system';
      XmlRootEl['package'] := 'rtl';
    end;
    XmlDoc.AppendChild(XmlRootEl);
    AddPasElChildsToXml(XmlRootEl, FRootNode);
    WriteXMLFile(XmlDoc, AFileName);
  finally
    XmlDoc.Free;
  end;
end;
{$ENDIF}

end.

