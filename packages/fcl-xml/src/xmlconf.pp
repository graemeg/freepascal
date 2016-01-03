{
    This file is part of the Free Component Library

    Implementation of TXMLConfig class
    Copyright (c) 1999 - 2005 by Sebastian Guenther, sg@freepascal.org
    Modified in 2007 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  TXMLConfig enables applications to use XML files for storing their
  configuration data
}

{$IFDEF FPC}
{$MODE objfpc}
{$H+}
{$ENDIF}

unit XMLConf;

interface

uses
  SysUtils, Classes, DOM, XMLRead, XMLWrite;

resourcestring
<<<<<<< HEAD
<<<<<<< HEAD
  SWrongRootName = 'XML file has wrong root element name: expected "%s" but was "%s"';
=======
  SWrongRootName = 'XML file has wrong root element name';
>>>>>>> graemeg/fixes_2_2
=======
  SWrongRootName = 'XML file has wrong root element name';
>>>>>>> origin/fixes_2_2

type
  EXMLConfigError = class(Exception);
  TPathFlags = set of (pfHasValue, pfWriteAccess);

  {"APath" is the path and name of a value: A XML configuration file is
   hierachical. "/" is the path delimiter, the part after the last "/"
   is the name of the value. The path components will be mapped to XML
   elements, the name will be an element attribute.}

<<<<<<< HEAD
<<<<<<< HEAD
  { TXMLConfig }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  TXMLConfig = class(TComponent)
  private
    FFilename: String;
    FStartEmpty: Boolean;
    FRootName: DOMString;
    FDummy: DOMString;
<<<<<<< HEAD
<<<<<<< HEAD
    FPathStack: array of DOMString;
=======
    FPathStack: array of WideString;
>>>>>>> graemeg/fixes_2_2
=======
    FPathStack: array of WideString;
>>>>>>> origin/fixes_2_2
    FPathCount: Integer;
    FPathDirty: Boolean;
    FElement: TDOMElement;
    procedure DoSetFilename(const AFilename: String; ForceReload: Boolean);
    procedure SetFilename(const AFilename: String);
    procedure SetStartEmpty(AValue: Boolean);
    procedure SetRootName(const AValue: DOMString);
<<<<<<< HEAD
<<<<<<< HEAD
    function DoFindNode(const APath: DOMString; var Ident: DOMString;
      Flags: TPathFlags): TDomElement;
    function FindNode(const APath: DOMString; out Ident: DOMString;
=======
    function DoFindNode(const APath: WideString; var Ident: DOMString;
      Flags: TPathFlags): TDomElement;
    function FindNode(const APath: WideString; out Ident: DOMString;
>>>>>>> graemeg/fixes_2_2
=======
    function DoFindNode(const APath: WideString; var Ident: DOMString;
      Flags: TPathFlags): TDomElement;
    function FindNode(const APath: WideString; out Ident: DOMString;
>>>>>>> origin/fixes_2_2
      Flags: TPathFlags): TDOMElement;
  protected
    Doc: TXMLDocument;
    FModified: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FReadOnly: Boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure Loaded; override;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure Clear;
    procedure Flush;    // Writes the XML file
<<<<<<< HEAD
<<<<<<< HEAD
    procedure OpenKey(const aPath: DOMString);
    procedure CloseKey;
    procedure ResetKey;
    procedure SaveToFile(Const AFileName: string);
    procedure SaveToStream(S : TStream);
    procedure LoadFromFile(Const AFileName: string);
    procedure LoadFromStream(S : TStream);

    function  GetValue(const APath: DOMString; const ADefault: DOMString): DOMString; overload;
    function  GetValue(const APath: DOMString; ADefault: Integer): Integer; overload;
    function  GetValue(const APath: DOMString; ADefault: Boolean): Boolean; overload;
    procedure SetValue(const APath: DOMString; const AValue: DOMString); overload;
    procedure SetValue(const APath: DOMString; AValue: Integer); overload;
    procedure SetValue(const APath: DOMString; AValue: Boolean); overload;

    procedure SetDeleteValue(const APath: DOMString; const AValue, DefValue: DOMString); overload;
    procedure SetDeleteValue(const APath: DOMString; AValue, DefValue: Integer); overload;
    procedure SetDeleteValue(const APath: DOMString; AValue, DefValue: Boolean); overload;

    procedure DeletePath(const APath: DOMString);
    procedure DeleteValue(const APath: DOMString);
=======
=======
>>>>>>> origin/fixes_2_2
    procedure OpenKey(const aPath: WideString);
    procedure CloseKey;
    procedure ResetKey;

    function  GetValue(const APath: WideString; const ADefault: WideString): WideString; overload;
    function  GetValue(const APath: WideString; ADefault: Integer): Integer; overload;
    function  GetValue(const APath: WideString; ADefault: Boolean): Boolean; overload;
    procedure SetValue(const APath: WideString; const AValue: WideString); overload;
    procedure SetValue(const APath: WideString; AValue: Integer); overload;
    procedure SetValue(const APath: WideString; AValue: Boolean); overload;

    procedure SetDeleteValue(const APath: WideString; const AValue, DefValue: WideString); overload;
    procedure SetDeleteValue(const APath: WideString; AValue, DefValue: Integer); overload;
    procedure SetDeleteValue(const APath: WideString; AValue, DefValue: Boolean); overload;

    procedure DeletePath(const APath: WideString);
    procedure DeleteValue(const APath: WideString);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Modified: Boolean read FModified;
  published
    property Filename: String read FFilename write SetFilename;
    property StartEmpty: Boolean read FStartEmpty write SetStartEmpty;
    property RootName: DOMString read FRootName write SetRootName;
<<<<<<< HEAD
<<<<<<< HEAD
    property ReadOnly: Boolean read FReadOnly write FReadOnly;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;


// ===================================================================

implementation

constructor TXMLConfig.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FRootName := 'CONFIG';
  Doc := TXMLDocument.Create;
  Doc.AppendChild(Doc.CreateElement(FRootName));
end;

destructor TXMLConfig.Destroy;
begin
  if Assigned(Doc) then
  begin
    Flush;
    Doc.Free;
  end;
  inherited Destroy;
end;

procedure TXMLConfig.Clear;
begin
  Doc.ReplaceChild(Doc.CreateElement(FRootName), Doc.DocumentElement);
end;

procedure TXMLConfig.Flush;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if Modified and not FReadOnly then
    if (FFileName<>'') then
      SaveToFile(FFilename)
end;

procedure TXMLConfig.SaveToFile(const AFileName: string);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(AFileName,fmCreate);
  try
    SaveToStream(F);
    FFileName:=AFileName;
  finally
    F.Free;
  end;
end;

procedure TXMLConfig.SaveToStream(S: TStream);
begin
  WriteXMLFile(Doc,S);
  FModified := False;
end;

procedure TXMLConfig.LoadFromFile(const AFileName: string);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(AFileName,fmOpenread or fmShareDenyWrite);
  try
    ReadXMLFile(Doc, AFilename);
    FFileName:=AFileName;
  finally
    F.Free;
  end;
end;

procedure TXMLConfig.LoadFromStream(S: TStream);
begin
  ReadXMLFile(Doc,S);
  FModified := False;
  if (Doc.DocumentElement.NodeName<>FRootName) then
    raise EXMLConfigError.CreateFmt(SWrongRootName,[FRootName,Doc.DocumentElement.NodeName]);
end;

function TXMLConfig.GetValue(const APath: DOMString; const ADefault: DOMString): DOMString;
=======
=======
>>>>>>> origin/fixes_2_2
  if Modified and (Filename <> '') then
  begin
    WriteXMLFile(Doc, Filename);
    FModified := False;
  end;
end;

function TXMLConfig.GetValue(const APath: WideString; const ADefault: WideString): WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  Node: TDOMElement;
  Attr: TDOMAttr;
  Ident: DOMString;
begin
  Result := ADefault;

  Node := FindNode(APath, Ident, [pfHasValue]);
  if Assigned(Node) then
  begin
    Attr := Node.GetAttributeNode(Ident);
    if Assigned(Attr) then
      Result := Attr.NodeValue;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TXMLConfig.GetValue(const APath: DOMString; ADefault: Integer): Integer;
=======
function TXMLConfig.GetValue(const APath: WideString; ADefault: Integer): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TXMLConfig.GetValue(const APath: WideString; ADefault: Integer): Integer;
>>>>>>> origin/fixes_2_2
begin
  Result := StrToIntDef(GetValue(APath, ''),ADefault);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TXMLConfig.GetValue(const APath: DOMString; ADefault: Boolean): Boolean;
=======
function TXMLConfig.GetValue(const APath: WideString; ADefault: Boolean): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TXMLConfig.GetValue(const APath: WideString; ADefault: Boolean): Boolean;
>>>>>>> origin/fixes_2_2
var
  s: DOMString;
begin
  s := GetValue(APath, '');

  if WideSameText(s, 'TRUE') then
    Result := True
  else if WideSameText(s, 'FALSE') then
    Result := False
  else
    Result := ADefault;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.SetValue(const APath: DOMString; const AValue: DOMString);
=======
procedure TXMLConfig.SetValue(const APath: WideString; const AValue: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.SetValue(const APath: WideString; const AValue: WideString);
>>>>>>> origin/fixes_2_2
var
  Node: TDOMElement;
  Attr: TDOMAttr;
  Ident: DOMString;
begin
  Node := FindNode(APath, Ident, [pfHasValue, pfWriteAccess]);

  Attr := Node.GetAttributeNode(Ident);
  if (Attr = nil) or (Attr.NodeValue <> AValue) then
  begin
    Node[Ident] := AValue;
    FModified := True;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.SetDeleteValue(const APath: DOMString; const AValue, DefValue: DOMString);
=======
procedure TXMLConfig.SetDeleteValue(const APath: WideString; const AValue, DefValue: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.SetDeleteValue(const APath: WideString; const AValue, DefValue: WideString);
>>>>>>> origin/fixes_2_2
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath, AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.SetValue(const APath: DOMString; AValue: Integer);
=======
procedure TXMLConfig.SetValue(const APath: WideString; AValue: Integer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.SetValue(const APath: WideString; AValue: Integer);
>>>>>>> origin/fixes_2_2
begin
  SetValue(APath, IntToStr(AValue));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.SetDeleteValue(const APath: DOMString; AValue,
=======
procedure TXMLConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> origin/fixes_2_2
  DefValue: Integer);
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath, AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.SetValue(const APath: DOMString; AValue: Boolean);
=======
procedure TXMLConfig.SetValue(const APath: WideString; AValue: Boolean);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.SetValue(const APath: WideString; AValue: Boolean);
>>>>>>> origin/fixes_2_2
begin
  if AValue then
    SetValue(APath, 'True')
  else
    SetValue(APath, 'False');
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.SetDeleteValue(const APath: DOMString; AValue,
=======
procedure TXMLConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> origin/fixes_2_2
  DefValue: Boolean);
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath,AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.DeletePath(const APath: DOMString);
=======
procedure TXMLConfig.DeletePath(const APath: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.DeletePath(const APath: WideString);
>>>>>>> origin/fixes_2_2
var
  Node: TDomNode;
  Ident: DOMString;
begin
  Node := FindNode(APath, Ident, []);
  if Assigned(Node) and Assigned(Node.ParentNode) then
  begin
    Node.ParentNode.RemoveChild(Node);
    FPathDirty := True;
    FElement := nil;
    FModified := True;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.DeleteValue(const APath: DOMString);
=======
procedure TXMLConfig.DeleteValue(const APath: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.DeleteValue(const APath: WideString);
>>>>>>> origin/fixes_2_2
var
  Node: TDOMElement;
  Ident: DOMString;
  Parent: TDOMNode;
begin
  Node := FindNode(APath, Ident, [pfHasValue]);
  if Assigned(Node) then
  begin
    if Assigned(Node.GetAttributeNode(Ident)) then
    begin
      Node.RemoveAttribute(Ident);
      FModified := True;
    end;
    while (Node.FirstChild=nil) and Assigned(Node.ParentNode)
      and Assigned(Node.ParentNode.ParentNode) do
    begin
      if Node.HasAttributes then
        Break;
      Parent := Node.ParentNode;
      Parent.RemoveChild(Node);
      Node := TDOMElement(Parent);
      FPathDirty := True;
      FElement := nil;
      FModified := True;
    end;
  end;
end;

procedure TXMLConfig.Loaded;
begin
  inherited Loaded;
  if Length(Filename) > 0 then
    DoSetFilename(Filename,True);              // Load the XML config file
end;

// TODO: copied from dom.pp, make public there and delete here
function CompareDOMStrings(const s1, s2: DOMPChar; l1, l2: integer): integer;
var i: integer;
begin
  Result:=l1-l2;
  i:=0;
  while (i<l1) and (Result=0) do begin
    Result:=ord(s1[i])-ord(s2[i]);
    inc(i);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TXMLConfig.FindNode(const APath: DOMString; out Ident: DOMString;
=======
function TXMLConfig.FindNode(const APath: WideString; out Ident: DOMString;
>>>>>>> graemeg/fixes_2_2
=======
function TXMLConfig.FindNode(const APath: WideString; out Ident: DOMString;
>>>>>>> origin/fixes_2_2
  Flags: TPathFlags): TDOMElement;
var
  I: Integer;
begin
  if FPathDirty then
  begin
    for I := 0 to FPathCount-1 do
      FElement := DoFindNode(FPathStack[I], FDummy, Flags - [pfHasValue]);
<<<<<<< HEAD
<<<<<<< HEAD
    if Assigned(FElement) then FPathDirty := False;
=======
    FPathDirty := False;      
>>>>>>> graemeg/fixes_2_2
=======
    FPathDirty := False;      
>>>>>>> origin/fixes_2_2
  end;
  Result := DoFindNode(APath, Ident, Flags);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TXMLConfig.DoFindNode(const APath: DOMString; var Ident: DOMString;
=======
function TXMLConfig.DoFindNode(const APath: WideString; var Ident: DOMString;
>>>>>>> graemeg/fixes_2_2
=======
function TXMLConfig.DoFindNode(const APath: WideString; var Ident: DOMString;
>>>>>>> origin/fixes_2_2
  Flags: TPathFlags): TDomElement;
var
  StartPos, EndPos: integer;
  PathLen: integer;
  Child: TDOMNode;
begin
  if Assigned(FElement) and (Length(APath) > 0) and (APath[1] <> '/') then
    Result := FElement
  else
    Result := Doc.DocumentElement;

  PathLen := Length(APath);
  StartPos := 1;
  if APath[StartPos] = '/' then Inc(StartPos);
  while Assigned(Result) do
  begin
    EndPos := StartPos;
    while (EndPos <= PathLen) and (APath[EndPos] <> '/') do
      Inc(EndPos);
    if (EndPos > PathLen) and (pfHasValue in Flags) then
    begin
      SetString(Ident, PWideChar(@APath[StartPos]), PathLen-StartPos+1);
      exit;
    end;
    if EndPos = StartPos then
      break;
    Child := Result.FirstChild;
    while Assigned(Child) and not ((Child.NodeType = ELEMENT_NODE)
      and (0 = CompareDOMStrings(DOMPChar(TDOMElement(Child).TagName), @APath[StartPos],
                                 Length(TDOMElement(Child).TagName), EndPos-StartPos))) do
        Child := Child.NextSibling;
    if (Child = nil) and (pfWriteAccess in Flags) then
    begin
      Child := Doc.CreateElementBuf(@APath[StartPos], EndPos-StartPos);
      Result.AppendChild(Child);
    end;
    Result := TDOMElement(Child);
    StartPos := EndPos + 1;
    if StartPos > PathLen then
      exit;
  end;
  Result := nil;
end;

procedure TXMLConfig.DoSetFilename(const AFilename: String; ForceReload: Boolean);
begin
  if (not ForceReload) and (FFilename = AFilename) then
    exit;
    
  Flush;
  FreeAndNil(Doc);
<<<<<<< HEAD
<<<<<<< HEAD
  if csLoading in ComponentState then
    exit;
  if FileExists(AFilename) and not FStartEmpty then
    LoadFromFile(AFilename)
  else if not Assigned(Doc) then
    begin
    FFileName:=AFileName;
    Doc := TXMLDocument.Create;
    Doc.AppendChild(Doc.CreateElement(FRootName))
    end;
=======
=======
>>>>>>> origin/fixes_2_2
    
  FFilename := AFilename;

  if csLoading in ComponentState then
    exit;

  if FileExists(AFilename) and not FStartEmpty then
    ReadXMLFile(Doc, AFilename);

  if not Assigned(Doc) then
    Doc := TXMLDocument.Create;

  if not Assigned(Doc.DocumentElement) then
    Doc.AppendChild(Doc.CreateElement(FRootName))
  else
    if Doc.DocumentElement.NodeName <> FRootName then
      raise EXMLConfigError.Create(SWrongRootName);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TXMLConfig.SetFilename(const AFilename: String);
begin
  DoSetFilename(AFilename, False);
end;

procedure TXMLConfig.SetRootName(const AValue: DOMString);
var
  Cfg, Root: TDOMElement;
begin
  if AValue <> FRootName then
  begin
    FRootName := AValue;
    Root := Doc.DocumentElement;
    Cfg := Doc.CreateElement(AValue);
    while Assigned(Root.FirstChild) do
      Cfg.AppendChild(Root.FirstChild);
    Doc.ReplaceChild(Cfg, Root);
    FModified := True;
  end;
end;

procedure TXMLConfig.SetStartEmpty(AValue: Boolean);
begin
  if AValue <> StartEmpty then
  begin
    FStartEmpty := AValue;
    if (not AValue) and not Modified then
      DoSetFilename(Filename, True);
  end;
end;

procedure TXMLConfig.CloseKey;
begin
  if FPathCount > 0 then
  begin
    FPathStack[FPathCount-1] := '';
    Dec(FPathCount);
    FElement := nil;
    FPathDirty := True;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLConfig.OpenKey(const aPath: DOMString);
=======
procedure TXMLConfig.OpenKey(const aPath: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLConfig.OpenKey(const aPath: WideString);
>>>>>>> origin/fixes_2_2
begin
  if aPath <> '' then
  begin
    if FPathCount >= Length(FPathStack) then
      SetLength(FPathStack, FPathCount + 5);

    FPathStack[FPathCount] := aPath;
    Inc(FPathCount);
    FElement := nil;
    FPathDirty := True;
  end;
end;

procedure TXMLConfig.ResetKey;
var
  I: Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  for I := Length(FPathStack)-1 downto 0 do
=======
  for I := Length(FPathStack) downto 0 do
>>>>>>> graemeg/fixes_2_2
=======
  for I := Length(FPathStack) downto 0 do
>>>>>>> origin/fixes_2_2
    FPathStack[I] := '';
  FElement := nil;    
  FPathDirty := False;
  FPathCount := 0;
end;






end.
