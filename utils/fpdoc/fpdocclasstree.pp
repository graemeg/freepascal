unit fpdocclasstree;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DOM, pastree;

Type
  TClassTreeBuilder = Class
  Private
    FClassTree : TXMLDocument;
    FTreeStart : TDomElement;
    FObjectKind : TPasObjKind;
    FPackage: TPasPackage;
    FParentObject : TPasClassType;
  Protected
    function LookForElement(PE: TDomElement; AElement: TPasElement; NoPath : Boolean): TDomNode;
    function NodeMatch(N: TDomNode; AElement: TPasElement; NoPath : Boolean): Boolean;
    Function AddToClassTree(AElement : TPasElement; Var ACount : Integer) : TDomElement;
  Public
    Constructor Create(APackage : TPasPackage; AObjectKind : TPasObjKind = okClass);
    Destructor Destroy; override;
    Function BuildTree(AObjects : TStringList) : Integer;
    Property ClassTree : TXMLDocument Read FClassTree;
  end;

implementation

constructor TClassTreeBuilder.Create(APackage : TPasPackage;
  AObjectKind: TPasObjKind);

begin
  FCLassTree:=TXMLDocument.Create;
  FPackage:=APAckage;
  FObjectKind:=AObjectKind;
  Case FObjectkind of
    okObject    : FParentObject:=TPasClassType.Create('System.TObject',FPackage);
    okClass     : FParentObject:=TPasClassType.Create('System.TObject',FPackage);
    okInterface : FParentObject:=TPasClassType.Create('System.IInterface',FPackage);
  end;
  FParentObject.ObjKind:=FObjectKind;
  FTreeStart:=FClassTree.CreateElement('TObject');
  FTreeStart['unit']:='System';
  ClassTree.AppendChild(FTreeStart);
end;

destructor TClassTreeBuilder.Destroy;
begin
  FreeAndNil(FParentObject);
  FreeAndNil(FClassTree);
  Inherited;
end;

Function TClassTreeBuilder.BuildTree(AObjects : TStringList) : Integer;

Var
  I : Integer;
  PC : TPasClassType;

begin
  Result:=0;
  AObjects.Sorted:=True;
  For I:=0 to AObjects.Count-1 do
    // Advanced records
    if AObjects.Objects[i] is TPasClassType then
      begin
      PC:=AObjects.Objects[i] as TPasClassType;
      If (PC.ObjKind=FObjectKind) and Not PC.IsForward then
        AddToClassTree(PC,Result);
      end;
end;

Function TClassTreeBuilder.NodeMatch(N : TDomNode; AElement : TPasElement; NoPath : Boolean) : Boolean;

Var
  PN,S,EN : String;

begin
  EN:=AELement.Name;
  Result:=(N.NodeType=ELEMENT_NODE);
  if Result then
    begin
    S:=UTF8Encode(N.NodeName);
    if NoPath then
      Begin
      Result:=CompareText(S,EN)=0;
      end
    else
      begin
      IF Assigned(Aelement.GetModule) then
        PN:=Aelement.GetModule.PackageName
      else
        PN:=FPackage.Name;
      S:=PN+'.'+UTF8Encode(TDomElement(N)['unit'])+'.'+S;
      Result:=(CompareText(S,AElement.PathName)=0);
      end;
   end;
end;

Function TClassTreeBuilder.LookForElement(PE : TDomElement; AElement : TPasElement; NoPath : boolean) : TDomNode;

Var
  N : TDomNode;

begin
//  Writeln('Enter TClassTreeBuilderLookForElement');
  Result:=PE;
  While (Result<>Nil) and Not NodeMatch(Result,AElement,NoPath) do
    Result:=Result.NextSibling;
  If (Result=Nil) then
    if  Assigned(PE) then
      begin
      N:=PE.FirstChild;
      While (Result=Nil) and (N<>Nil) do
        begin
        if (N.NodeType=ELEMENT_NODE) then
          begin
          Result:=LookForElement(N as TDomElement,AElement,NoPath);
          end;
        N:=N.NextSibling;
        end;
      end;
//  Writeln('Exit TClassTreeBuilderLookForElement');
end;

Function TClassTreeBuilder.AddToClassTree(AElement : TPasElement; Var ACount : Integer) : TDomElement;
// there are several codepaths that use uninitialized variables. (N,PE)
// I initialized them to nil to at least make failures deterministic.
Var
  PC : TPasClassType;
  PE : TDomElement;
  M : TPasModule;
  N : TDomNode;

begin

//  Writeln('Enter TClassTreeBuilder.AddToClassTree');
  //if Assigned(AElement) then
    //Writeln('Addtoclasstree : ',aElement.Name);
  Result:=Nil; M:=Nil; N:=Nil;PE:=NIL;PC:=Nil;
  If (AElement=Nil) then
    begin
    Result:=FTreeStart;
    Exit;
    end
  else If (AElement is TPasUnresolvedTypeRef) then
    begin
    N:=LookForElement(FTreeStart,AElement,True);
    If (N=Nil) then
      PE:=FTreeStart;
    end
  else If (AElement is TPasClassType) then
    begin
    if (AElement=FParentObject) then
      Result:=FTreeStart
    else
      begin
      PC:=AElement as TPasClassType;
      PE:=AddToClassTree(PC.AncestorType,ACount);
      if PE=Nil then
        PE:=FTreeStart;
      N:=LookForElement(PE,PC,False);
      end
    end;
  If (N<>Nil) then
    begin
    Result:=N as TDomElement
    end
  else if AElement.Name<>'' then
    begin // N=NIL, PE might be nil.
    Inc(ACount);
    Result:=FClassTree.CreateElement(UTF8Decode(AElement.Name));
    If Not (AElement is TPasUnresolvedTypeRef) then
      begin
      M:=AElement.GetModule;
      if Assigned(M) then
        Result['unit']:=UTF8Decode(M.Name);
      end;
    if PE=Nil then
      begin
      PE:=FTreeStart
      end;
    // if not assigned, probably needs to be assigned to something else.
    if assigned(PE) then
      PE.AppendChild(Result);
    end;
end;

end.

