{
    This file is part of the Free Component Library

    Object model for DTD.
    Copyright (c) 2010 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit dtdmodel;

{$ifdef fpc}
{$MODE objfpc}{$H+}
{$endif}

interface

uses
  Classes, SysUtils, xmlutils;

type
  TCPType = (ctName, ctChoice, ctSeq);
  TCPQuant = (cqOnce, cqZeroOrOnce, cqZeroOrMore, cqOnceOrMore);

  TContentParticle = class(TObject)
  private
    FParent: TContentParticle;
    FChildren: TFPList;
    FIndex: Integer;
    FDef: TObject;
    FCPType: TCPType;
    FCPQuant: TCPQuant;
    function GetChildCount: Integer;
    function GetChild(Index: Integer): TContentParticle;
  public
    destructor Destroy; override;
    function Add: TContentParticle;
    function IsRequired: Boolean;
    function FindFirst(aDef: TObject): TContentParticle;
    function FindNext(aDef: TObject; ChildIdx: Integer): TContentParticle;
    function MoreRequired(ChildIdx: Integer): Boolean;
    property ChildCount: Integer read GetChildCount;
    property Children[Index: Integer]: TContentParticle read GetChild;
    property Def: TObject read FDef write FDef;
    property CPType: TCPType read FCPType write FCPType;
    property CPQuant: TCPQuant read FCPQuant write FCPQuant;
  end;

  TDTDObject = class(TObject)
  private
    FExternallyDeclared: Boolean;
  public
    property ExternallyDeclared: Boolean read FExternallyDeclared write FExternallyDeclared;
  end;

  TAttrDefault = (
    adImplied,
    adDefault,
    adRequired,
    adFixed
  );

  TAttributeDef = class(TDTDObject)
  private
    FData: PNodeData;
    FDataType: TAttrDataType;
    FDefault: TAttrDefault;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FIndex: Cardinal;
    FIsNamespaceDecl: Boolean;
    FEnumeration: array of XMLString;
=======
    FTag: Cardinal;
    FIsNamespaceDecl: Boolean;
    FEnumeration: array of WideString;
>>>>>>> graemeg/cpstrnew
=======
    FTag: Cardinal;
    FIsNamespaceDecl: Boolean;
    FEnumeration: array of WideString;
>>>>>>> graemeg/cpstrnew
=======
    FTag: Cardinal;
    FIsNamespaceDecl: Boolean;
    FEnumeration: array of WideString;
>>>>>>> graemeg/cpstrnew
=======
    FTag: Cardinal;
    FIsNamespaceDecl: Boolean;
    FEnumeration: array of WideString;
>>>>>>> origin/cpstrnew
  public
    constructor Create(aName: PHashItem; aColonPos: Integer);
    destructor Destroy; override;
    function AddEnumToken(Buf: PWideChar; Len: Integer): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function HasEnumToken(const aValue: XMLString): Boolean;
    function ValidateSyntax(const aValue: XMLString; Namespaces: Boolean): Boolean;
    property Data: PNodeData read FData;
    property Default: TAttrDefault read FDefault write FDefault;
    property DataType: TAttrDataType read FDataType write FDataType;
    property Index: Cardinal read FIndex;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function HasEnumToken(const aValue: WideString): Boolean;
    property Data: PNodeData read FData;
    property Default: TAttrDefault read FDefault write FDefault;
    property DataType: TAttrDataType read FDataType write FDataType;
    property Tag: Cardinal read FTag write FTag;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    property IsNamespaceDecl: Boolean read FIsNamespaceDecl;
  end;

  TElementContentType = (
    ctUndeclared,
    ctAny,
    ctEmpty,
    ctMixed,
    ctChildren
  );

  TElementDecl = class(TDTDObject)
  private
    FAttrDefs: TFPList;
    FNeedsDefaultPass: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FHasRequiredAtts: Boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function GetAttrDefCount: Integer;
    function AttrDefByIndex(index: Integer): TAttributeDef;
  public
    ContentType: TElementContentType;
    IDAttr: TAttributeDef;
    NotationAttr: TAttributeDef;
    RootCP: TContentParticle;
    destructor Destroy; override;
    function GetAttrDef(aName: PHashItem): TAttributeDef;
    procedure AddAttrDef(aDef: TAttributeDef);
    property AttrDefCount: Integer read GetAttrDefCount;
    property AttrDefs[index: Integer]: TAttributeDef read AttrDefByIndex;
    property NeedsDefaultPass: Boolean read FNeedsDefaultPass;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property HasRequiredAtts: Boolean read FHasRequiredAtts;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

  TEntityDecl = class(TDTDObject)
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FName: XMLString;   // TODO: change to PHashItem
    FInputEncoding: XMLString;
    FXMLEncoding: XMLString;
    FPublicID: XMLString;
    FSystemID: XMLString;
    FNotationName: XMLString;
    FURI: XMLString;
    FReplacementText: XMLString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FName: WideString;   // TODO: change to PHashItem
    FInputEncoding: WideString;
    FXMLEncoding: WideString;
    FPublicID: WideString;
    FSystemID: WideString;
    FNotationName: WideString;
    FURI: WideString;
    FReplacementText: WideString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FXMLVersion: TXMLVersion;
    FPrefetched: Boolean;
    FResolved: Boolean;
    FOnStack: Boolean;
    FBetweenDecls: Boolean;
    FIsPE: Boolean;
    FStartLocation: TLocation;
    FCharCount: Cardinal;
  end;

  TNotationDecl = class(TDTDObject)
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FName: XMLString;
    FPublicID: XMLString;
    FSystemID: XMLString;
    FURI: XMLString;
=======
    FName: WideString;
    FPublicID: WideString;
    FSystemID: WideString;
>>>>>>> graemeg/cpstrnew
=======
    FName: WideString;
    FPublicID: WideString;
    FSystemID: WideString;
>>>>>>> graemeg/cpstrnew
=======
    FName: WideString;
    FPublicID: WideString;
    FSystemID: WideString;
>>>>>>> graemeg/cpstrnew
=======
    FName: WideString;
    FPublicID: WideString;
    FSystemID: WideString;
>>>>>>> origin/cpstrnew
  end;

  TDTDModel = class
  private
    FRefCount: Integer;
    FNameTable: THashTable;
    FEntities: THashTable;
    FNotations: THashTable;
    function GetEntities: THashTable;
    function GetNotations: THashTable;
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FName: XMLString;
    FSystemID: XMLString;
    FPublicID: XMLString;
    FInternalSubset: XMLString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FName: WideString;
    FSystemID: WideString;
    FPublicID: WideString;
    FInternalSubset: WideString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    constructor Create(aNameTable: THashTable);
    destructor Destroy; override;
    function Reference: TDTDModel;
    procedure Release;
    property Entities: THashTable read GetEntities;
    property Notations: THashTable read GetNotations;
  end;

implementation

{ TDTDModel }

function TDTDModel.GetEntities: THashTable;
begin
  if FEntities = nil then
    FEntities := THashTable.Create(256, True);
  Result := FEntities;
end;

function TDTDModel.GetNotations: THashTable;
begin
  if FNotations = nil then
    FNotations := THashTable.Create(256, True);
  Result := FNotations;
end;

constructor TDTDModel.Create(aNameTable: THashTable);
begin
  FNameTable := aNameTable;
  FRefCount := 1;
end;

destructor TDTDModel.Destroy;
begin
  FEntities.Free;
  FNotations.Free;
  inherited Destroy;
end;

function TDTDModel.Reference: TDTDModel;
begin
  Inc(FRefCount);
  Result := Self;
end;

procedure TDTDModel.Release;
begin
  if Assigned(Self) then
  begin
    Dec(FRefCount);
    if FRefCount = 0 then
      self.Destroy;
  end;
end;

{ TContentParticle }

function TContentParticle.Add: TContentParticle;
begin
  if FChildren = nil then
    FChildren := TFPList.Create;
  Result := TContentParticle.Create;
  Result.FParent := Self;
  Result.FIndex := FChildren.Add(Result);
end;

destructor TContentParticle.Destroy;
var
  I: Integer;
begin
  if Assigned(FChildren) then
    for I := FChildren.Count-1 downto 0 do
      TObject(FChildren[I]).Free;
  FChildren.Free;
  inherited Destroy;
end;

function TContentParticle.GetChild(Index: Integer): TContentParticle;
begin
  Result := TContentParticle(FChildren[Index]);
end;

function TContentParticle.GetChildCount: Integer;
begin
  if Assigned(FChildren) then
    Result := FChildren.Count
  else
    Result := 0;
end;

function TContentParticle.IsRequired: Boolean;
var
  I: Integer;
begin
  Result := (CPQuant = cqOnce) or (CPQuant = cqOnceOrMore);
  // do not return True if all children are optional
  if (CPType <> ctName) and Result then
  begin
    for I := 0 to ChildCount-1 do
    begin
      Result := Children[I].IsRequired;
      if Result then Exit;
    end;
  end;
end;

function TContentParticle.MoreRequired(ChildIdx: Integer): Boolean;
var
  I: Integer;
begin
  Result := False;
  if CPType = ctSeq then
  begin
    for I := ChildIdx + 1 to ChildCount-1 do
    begin
      Result := Children[I].IsRequired;
      if Result then Exit;
    end;
  end;
  if Assigned(FParent) then
    Result := FParent.MoreRequired(FIndex);
end;

function TContentParticle.FindFirst(aDef: TObject): TContentParticle;
var
  I: Integer;
begin
  Result := nil;
  case CPType of
    ctSeq:
      for I := 0 to ChildCount-1 do with Children[I] do
      begin
        Result := FindFirst(aDef);
        if Assigned(Result) or IsRequired then
          Exit;
      end;
    ctChoice:
      for I := 0 to ChildCount-1 do with Children[I] do
      begin
        Result := FindFirst(aDef);
        if Assigned(Result) then
          Exit;
      end;
  else // ctName
    if aDef = Self.Def then
      Result := Self
  end;
end;

function TContentParticle.FindNext(aDef: TObject;
  ChildIdx: Integer): TContentParticle;
var
  I: Integer;
begin
  Result := nil;
  if CPType = ctSeq then   // search sequence to its end
  begin
    for I := ChildIdx + 1 to ChildCount-1 do with Children[I] do
    begin
      Result := FindFirst(aDef);
      if (Result <> nil) or IsRequired then
        Exit;
    end;
  end;
  if (CPQuant = cqZeroOrMore) or (CPQuant = cqOnceOrMore) then
    Result := FindFirst(aDef);
  if (Result = nil) and Assigned(FParent) then
    Result := FParent.FindNext(aDef, FIndex);
end;

{ TElementDecl }

function TElementDecl.GetAttrDefCount: Integer;
begin
  if Assigned(FAttrDefs) then
    Result := FAttrDefs.Count
  else
    Result := 0;
end;

function TElementDecl.AttrDefByIndex(index: Integer): TAttributeDef;
begin
  if Assigned(FAttrDefs) then
    Result := TAttributeDef(FAttrDefs[index])
  else
    Result := nil;
end;

destructor TElementDecl.Destroy;
var
  i: Integer;
begin
  RootCP.Free;
  if Assigned(FAttrDefs) then
  begin
    for i := FAttrDefs.Count-1 downto 0 do
      TObject(FAttrDefs.List^[i]).Free;
    FAttrDefs.Free;
  end;
  inherited Destroy;
end;

function TElementDecl.GetAttrDef(aName: PHashItem): TAttributeDef;
var
  i: Integer;
begin
  if Assigned(FAttrDefs) then
  begin
    for i := 0 to FAttrDefs.Count-1 do
    begin
      Result := TAttributeDef(FAttrDefs.List^[i]);
      if Result.FData^.FQName = aName then
        Exit;
    end;
  end;
  Result := nil;
end;

procedure TElementDecl.AddAttrDef(aDef: TAttributeDef);
begin
  if FAttrDefs = nil then
    FAttrDefs := TFPList.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  aDef.FIndex := FAttrDefs.Add(aDef);
  if aDef.Default in [adRequired, adDefault, adFixed] then
    FNeedsDefaultPass := True;
  if aDef.Default = adRequired then
    FHasRequiredAtts := True;
=======
  FAttrDefs.Add(aDef);
  if aDef.Default in [adRequired, adDefault, adFixed] then
    FNeedsDefaultPass := True;
>>>>>>> graemeg/cpstrnew
=======
  FAttrDefs.Add(aDef);
  if aDef.Default in [adRequired, adDefault, adFixed] then
    FNeedsDefaultPass := True;
>>>>>>> graemeg/cpstrnew
=======
  FAttrDefs.Add(aDef);
  if aDef.Default in [adRequired, adDefault, adFixed] then
    FNeedsDefaultPass := True;
>>>>>>> graemeg/cpstrnew
=======
  FAttrDefs.Add(aDef);
  if aDef.Default in [adRequired, adDefault, adFixed] then
    FNeedsDefaultPass := True;
>>>>>>> origin/cpstrnew
end;

{ TAttributeDef }

constructor TAttributeDef.Create(aName: PHashItem; aColonPos: Integer);
begin
  New(FData);
  FillChar(FData^, sizeof(TNodeData), 0);
  FData^.FIsDefault := True;
  FData^.FQName := aName;
  FData^.FColonPos := aColonPos;
  FData^.FTypeInfo := Self;
  FIsNamespaceDecl := ((Length(aName^.Key) = 5) or (aColonPos = 6)) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    (Pos(XMLString('xmlns'), aName^.Key) = 1);
=======
    (Pos(WideString('xmlns'), aName^.Key) = 1);
>>>>>>> graemeg/cpstrnew
=======
    (Pos(WideString('xmlns'), aName^.Key) = 1);
>>>>>>> graemeg/cpstrnew
=======
    (Pos(WideString('xmlns'), aName^.Key) = 1);
>>>>>>> graemeg/cpstrnew
=======
    (Pos(WideString('xmlns'), aName^.Key) = 1);
>>>>>>> origin/cpstrnew
end;

destructor TAttributeDef.Destroy;
var
  curr, tmp: PNodeData;
begin
  curr := FData;
  while Assigned(curr) do
  begin
    tmp := curr^.FNext;
    Dispose(curr);
    curr := tmp;
  end;
  inherited Destroy;
end;

function TAttributeDef.AddEnumToken(Buf: PWideChar; Len: Integer): Boolean;
var
  I, L: Integer;
begin
  // TODO: this implementaion is the slowest possible...
  Result := False;
  L := Length(FEnumeration);
  for I := 0 to L-1 do
  begin
    if (Len = Length(FEnumeration[i])) and
      CompareMem(Pointer(FEnumeration[i]), Buf, Len*sizeof(WideChar)) then
        Exit;
  end;
  SetLength(FEnumeration, L+1);
  SetString(FEnumeration[L], Buf, Len);
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TAttributeDef.HasEnumToken(const aValue: XMLString): Boolean;
=======
function TAttributeDef.HasEnumToken(const aValue: WideString): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TAttributeDef.HasEnumToken(const aValue: WideString): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TAttributeDef.HasEnumToken(const aValue: WideString): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TAttributeDef.HasEnumToken(const aValue: WideString): Boolean;
>>>>>>> origin/cpstrnew
var
  I: Integer;
begin
  Result := True;
  if Length(FEnumeration) = 0 then
    Exit;
  for I := 0 to Length(FEnumeration)-1 do
  begin
    if FEnumeration[I] = aValue then
      Exit;
  end;
  Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TAttributeDef.ValidateSyntax(const aValue: XMLString; Namespaces: Boolean): Boolean;
begin
  case FDataType of
    dtId, dtIdRef, dtEntity: Result := IsXmlName(aValue) and
      ((not Namespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtIdRefs, dtEntities: Result := IsXmlNames(aValue) and
      ((not Namespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtNmToken: Result := IsXmlNmToken(aValue) and HasEnumToken(aValue);
    dtNmTokens: Result := IsXmlNmTokens(aValue);
    // IsXmlName() not necessary - enum is never empty and contains valid names
    dtNotation: Result := HasEnumToken(aValue);
  else
    Result := True;
  end;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end.
