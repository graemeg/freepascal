{
    This file is part of the Free Component Library

    HTML writing routines
    Copyright (c) 2000-2002 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


unit HTMWrite;

{$MODE objfpc}
{$H+}

interface

uses Classes, DOM;

procedure WriteHTMLFile(doc: TXMLDocument; const AFileName: String);
procedure WriteHTMLFile(doc: TXMLDocument; var AFile: Text);
procedure WriteHTMLFile(doc: TXMLDocument; AStream: TStream);

procedure WriteHTML(Element: TDOMNode; const AFileName: String);
procedure WriteHTML(Element: TDOMNode; var AFile: Text);
procedure WriteHTML(Element: TDOMNode; AStream: TStream);


// ===================================================================

implementation

uses SysUtils, HTMLDefs, xmlutils;

type
  TSpecialCharCallback = procedure(c: WideChar) of object;

  THTMLWriter = class(TObject)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    FStream: TStream;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FInsideTextNode: Boolean;
    FBuffer: PChar;
    FBufPos: PChar;
    FCapacity: Integer;
    FLineBreak: string;
    procedure wrtChars(Src: PWideChar; Length: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure wrtStr(const ws: XMLString); {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtChr(c: WideChar); {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtIndent; {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtQuotedLiteral(const ws: XMLString);
    procedure ConvWrite(const s: XMLString; const SpecialChars: TSetOfChar;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure wrtStr(const ws: WideString); {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtChr(c: WideChar); {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtIndent; {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtQuotedLiteral(const ws: WideString);
    procedure ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      const SpecialCharCallback: TSpecialCharCallback);
    procedure AttrSpecialCharCallback(c: WideChar);
    procedure TextNodeSpecialCharCallback(c: WideChar);
  protected
<<<<<<< HEAD
<<<<<<< HEAD
=======
    procedure Write(const Buffer; Count: Longint); virtual; abstract;
>>>>>>> graemeg/fixes_2_2
=======
    procedure Write(const Buffer; Count: Longint); virtual; abstract;
>>>>>>> origin/fixes_2_2
    procedure WriteNode(Node: TDOMNode);
    procedure VisitDocument(Node: TDOMNode);
    procedure VisitElement(Node: TDOMNode);
    procedure VisitText(Node: TDOMNode);
    procedure VisitCDATA(Node: TDOMNode);
    procedure VisitComment(Node: TDOMNode);
    procedure VisitFragment(Node: TDOMNode);
    procedure VisitAttribute(Node: TDOMNode);
    procedure VisitEntityRef(Node: TDOMNode);
    procedure VisitDocumentType(Node: TDOMNode);
    procedure VisitPI(Node: TDOMNode);
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor Create(AStream: TStream; ACapacity : Cardinal = 4096);
=======
    constructor Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
    constructor Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
    constructor Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
    constructor Create(AStream: TStream);
>>>>>>> origin/cpstrnew
    destructor Destroy; override;
  end;

  TTextStream = class(TStream)
  Private
    F : ^Text;
  Public
    constructor Create(var AFile: Text);
    function Write(Const Buffer; Count: Longint): Longint; override;
  end;

{ ---------------------------------------------------------------------
    TTextStream
  ---------------------------------------------------------------------}


constructor TTextStream.Create(var AFile: Text);
begin
  inherited Create;
  f := @AFile;
end;

function TTextStream.Write(const Buffer; Count: Longint): Longint;
var
  s: string;
begin
  if Count>0 then
  begin
    SetString(s, PChar(@Buffer), Count);
    system.Write(f^, s);
  end;
  Result := Count;
end;

{ ---------------------------------------------------------------------
    THTMLWriter
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor THTMLWriter.Create(AStream: TStream; ACapacity : Cardinal = 4096);
=======
constructor THTMLWriter.Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
constructor THTMLWriter.Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
constructor THTMLWriter.Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
constructor THTMLWriter.Create(AStream: TStream);
>>>>>>> origin/cpstrnew
begin
  inherited Create;
  FStream := AStream;
  // some overhead - always be able to write at least one extra UCS4
  FCapacity := ACapacity;
  FBuffer := AllocMem(FCapacity+32);
  FBufPos := FBuffer;
  // Later on, this may be put under user control
  // for now, take OS setting
  FLineBreak := sLineBreak;
end;

destructor THTMLWriter.Destroy;
begin
  if FBufPos > FBuffer then
    FStream.write(FBuffer^, FBufPos-FBuffer);

  FreeMem(FBuffer);
  inherited Destroy;
end;

procedure THTMLWriter.wrtChars(Src: PWideChar; Length: Integer);
var
  pb: PChar;
  wc: Cardinal;
  SrcEnd: PWideChar;
begin
  pb := FBufPos;
  SrcEnd := Src + Length;
  while Src < SrcEnd do
  begin
    if pb >= @FBuffer[FCapacity] then
    begin
      FStream.write(FBuffer^, FCapacity);
      Dec(pb, FCapacity);
      if pb > FBuffer then
        Move(FBuffer[FCapacity], FBuffer^, pb - FBuffer);
    end;

    wc := Cardinal(Src^);  Inc(Src);
    case wc of
      $0A: pb := StrECopy(pb, PChar(FLineBreak));
      $0D: begin
        pb := StrECopy(pb, PChar(FLineBreak));
        if (Src < SrcEnd) and (Src^ = #$0A) then
          Inc(Src);
      end;

      0..$09, $0B, $0C, $0E..$7F:  begin
        pb^ := char(wc); Inc(pb);
      end;

      $80..$7FF: begin
        pb^ := Char($C0 or (wc shr 6));
        pb[1] := Char($80 or (wc and $3F));
        Inc(pb,2);
      end;

      $D800..$DBFF: begin
        if (Src < SrcEnd) and (Src^ >= #$DC00) and (Src^ <= #$DFFF) then
        begin
          wc := ((LongInt(wc) - $D7C0) shl 10) + LongInt(word(Src^) xor $DC00);
          Inc(Src);

          pb^ := Char($F0 or (wc shr 18));
          pb[1] := Char($80 or ((wc shr 12) and $3F));
          pb[2] := Char($80 or ((wc shr 6) and $3F));
          pb[3] := Char($80 or (wc and $3F));
          Inc(pb,4);
        end
        else
          raise EConvertError.Create('High surrogate without low one');
      end;
      $DC00..$DFFF:
        raise EConvertError.Create('Low surrogate without high one');
      else   // $800 >= wc > $FFFF, excluding surrogates
      begin
        pb^ := Char($E0 or (wc shr 12));
        pb[1] := Char($80 or ((wc shr 6) and $3F));
        pb[2] := Char($80 or (wc and $3F));
        Inc(pb,3);
      end;
    end;
  end;
  FBufPos := pb;
end;

procedure THTMLWriter.wrtStr(const ws: XMLString); { inline }
begin
  wrtChars(PWideChar(ws), Length(ws));
end;

{ No checks here - buffer always has 32 extra bytes }
procedure THTMLWriter.wrtChr(c: WideChar); { inline }
begin
  FBufPos^ := char(ord(c));
  Inc(FBufPos);
end;

procedure THTMLWriter.wrtIndent; { inline }
begin
  wrtChars(#10, 1);
end;

procedure THTMLWriter.wrtQuotedLiteral(const ws: XMLString);
var
  Quote: WideChar;
begin
  // TODO: need to check if the string also contains single quote
  // both quotes present is a error
  if Pos('"', ws) > 0 then
    Quote := ''''
  else
    Quote := '"';
  wrtChr(Quote);
  wrtStr(ws);
  wrtChr(Quote);
end;

=======
    constructor Create;
    destructor Destroy; override;
  end;

  TTextHTMLWriter = Class(THTMLWriter)
  Private
    F : ^Text;
  Protected
    Procedure Write(Const Buffer; Count : Longint);override;
  Public
    constructor Create(var AFile: Text);
  end;

  TStreamHTMLWriter = Class(THTMLWriter)
  Private
    F : TStream;
  Protected
    Procedure Write(Const Buffer; Count : Longint);override;
  Public
    constructor Create(AStream: TStream);
  end;

{ ---------------------------------------------------------------------
    TTextHTMLWriter
  ---------------------------------------------------------------------}


constructor TTextHTMLWriter.Create(var AFile: Text);
begin
  inherited Create;
  f := @AFile;
end;

procedure TTextHTMLWriter.Write(const Buffer; Count: Longint);
var
  s: string;
begin
  if Count>0 then
  begin
    SetString(s, PChar(@Buffer), Count);
    system.Write(f^, s);
  end;
end;

{ ---------------------------------------------------------------------
    TStreamHTMLWriter
  ---------------------------------------------------------------------}

constructor TStreamHTMLWriter.Create(AStream: TStream);
begin
  inherited Create;
  F := AStream;
end;


procedure TStreamHTMLWriter.Write(const Buffer; Count: Longint);
begin
  if Count > 0 then
    F.Write(Buffer, Count);
end;


{ ---------------------------------------------------------------------
    THTMLWriter
  ---------------------------------------------------------------------}

constructor THTMLWriter.Create;
var
  I: Integer;
begin
  inherited Create;
  // some overhead - always be able to write at least one extra UCS4
  FBuffer := AllocMem(512+32);
  FBufPos := FBuffer;
  FCapacity := 512;
  // Later on, this may be put under user control
  // for now, take OS setting
  FLineBreak := sLineBreak;
end;

destructor THTMLWriter.Destroy;
begin
  if FBufPos > FBuffer then
    write(FBuffer^, FBufPos-FBuffer);

  FreeMem(FBuffer);
  inherited Destroy;
end;

procedure THTMLWriter.wrtChars(Src: PWideChar; Length: Integer);
var
  pb: PChar;
  wc: Cardinal;
  SrcEnd: PWideChar;
begin
  pb := FBufPos;
  SrcEnd := Src + Length;
  while Src < SrcEnd do
  begin
    if pb >= @FBuffer[FCapacity] then
    begin
      write(FBuffer^, FCapacity);
      Dec(pb, FCapacity);
      if pb > FBuffer then
        Move(FBuffer[FCapacity], FBuffer^, pb - FBuffer);
    end;

    wc := Cardinal(Src^);  Inc(Src);
    case wc of
      $0A: pb := StrECopy(pb, PChar(FLineBreak));

      0..$09, $0B..$7F:  begin
        pb^ := char(wc); Inc(pb);
      end;

      $80..$7FF: begin
        pb^ := Char($C0 or (wc shr 6));
        pb[1] := Char($80 or (wc and $3F));
        Inc(pb,2);
      end;

      $D800..$DBFF: begin
        if (Src < SrcEnd) and (Src^ >= #$DC00) and (Src^ <= #$DFFF) then
        begin
          wc := ((LongInt(wc) - $D7C0) shl 10) + LongInt(word(Src^) xor $DC00);
          Inc(Src);

          pb^ := Char($F0 or (wc shr 18));
          pb[1] := Char($80 or ((wc shr 12) and $3F));
          pb[2] := Char($80 or ((wc shr 6) and $3F));
          pb[3] := Char($80 or (wc and $3F));
          Inc(pb,4);
        end
        else
          raise EConvertError.Create('High surrogate without low one');
      end;
      $DC00..$DFFF:
        raise EConvertError.Create('Low surrogate without high one');
      else   // $800 >= wc > $FFFF, excluding surrogates
      begin
        pb^ := Char($E0 or (wc shr 12));
        pb[1] := Char($80 or ((wc shr 6) and $3F));
        pb[2] := Char($80 or (wc and $3F));
        Inc(pb,3);
      end;
    end;
  end;
  FBufPos := pb;
end;

procedure THTMLWriter.wrtStr(const ws: WideString); { inline }
begin
  wrtChars(PWideChar(ws), Length(ws));
end;

{ No checks here - buffer always has 32 extra bytes }
procedure THTMLWriter.wrtChr(c: WideChar); { inline }
begin
  FBufPos^ := char(ord(c));
  Inc(FBufPos);
end;

procedure THTMLWriter.wrtIndent; { inline }
begin
  wrtChars(#10, 1);
=======
    constructor Create;
    destructor Destroy; override;
  end;

  TTextHTMLWriter = Class(THTMLWriter)
  Private
    F : ^Text;
  Protected
    Procedure Write(Const Buffer; Count : Longint);override;
  Public
    constructor Create(var AFile: Text);
  end;

  TStreamHTMLWriter = Class(THTMLWriter)
  Private
    F : TStream;
  Protected
    Procedure Write(Const Buffer; Count : Longint);override;
  Public
    constructor Create(AStream: TStream);
  end;

{ ---------------------------------------------------------------------
    TTextHTMLWriter
  ---------------------------------------------------------------------}


constructor TTextHTMLWriter.Create(var AFile: Text);
begin
  inherited Create;
  f := @AFile;
end;

procedure TTextHTMLWriter.Write(const Buffer; Count: Longint);
var
  s: string;
begin
  if Count>0 then
  begin
    SetString(s, PChar(@Buffer), Count);
    system.Write(f^, s);
  end;
end;

{ ---------------------------------------------------------------------
    TStreamHTMLWriter
  ---------------------------------------------------------------------}

constructor TStreamHTMLWriter.Create(AStream: TStream);
begin
  inherited Create;
  F := AStream;
end;


procedure TStreamHTMLWriter.Write(const Buffer; Count: Longint);
begin
  if Count > 0 then
    F.Write(Buffer, Count);
end;


{ ---------------------------------------------------------------------
    THTMLWriter
  ---------------------------------------------------------------------}

constructor THTMLWriter.Create;
var
  I: Integer;
begin
  inherited Create;
  // some overhead - always be able to write at least one extra UCS4
  FBuffer := AllocMem(512+32);
  FBufPos := FBuffer;
  FCapacity := 512;
  // Later on, this may be put under user control
  // for now, take OS setting
  FLineBreak := sLineBreak;
end;

destructor THTMLWriter.Destroy;
begin
  if FBufPos > FBuffer then
    write(FBuffer^, FBufPos-FBuffer);

  FreeMem(FBuffer);
  inherited Destroy;
end;

procedure THTMLWriter.wrtChars(Src: PWideChar; Length: Integer);
var
  pb: PChar;
  wc: Cardinal;
  SrcEnd: PWideChar;
begin
  pb := FBufPos;
  SrcEnd := Src + Length;
  while Src < SrcEnd do
  begin
    if pb >= @FBuffer[FCapacity] then
    begin
      write(FBuffer^, FCapacity);
      Dec(pb, FCapacity);
      if pb > FBuffer then
        Move(FBuffer[FCapacity], FBuffer^, pb - FBuffer);
    end;

    wc := Cardinal(Src^);  Inc(Src);
    case wc of
      $0A: pb := StrECopy(pb, PChar(FLineBreak));

      0..$09, $0B..$7F:  begin
        pb^ := char(wc); Inc(pb);
      end;

      $80..$7FF: begin
        pb^ := Char($C0 or (wc shr 6));
        pb[1] := Char($80 or (wc and $3F));
        Inc(pb,2);
      end;

      $D800..$DBFF: begin
        if (Src < SrcEnd) and (Src^ >= #$DC00) and (Src^ <= #$DFFF) then
        begin
          wc := ((LongInt(wc) - $D7C0) shl 10) + LongInt(word(Src^) xor $DC00);
          Inc(Src);

          pb^ := Char($F0 or (wc shr 18));
          pb[1] := Char($80 or ((wc shr 12) and $3F));
          pb[2] := Char($80 or ((wc shr 6) and $3F));
          pb[3] := Char($80 or (wc and $3F));
          Inc(pb,4);
        end
        else
          raise EConvertError.Create('High surrogate without low one');
      end;
      $DC00..$DFFF:
        raise EConvertError.Create('Low surrogate without high one');
      else   // $800 >= wc > $FFFF, excluding surrogates
      begin
        pb^ := Char($E0 or (wc shr 12));
        pb[1] := Char($80 or ((wc shr 6) and $3F));
        pb[2] := Char($80 or (wc and $3F));
        Inc(pb,3);
      end;
    end;
  end;
  FBufPos := pb;
end;

procedure THTMLWriter.wrtStr(const ws: WideString); { inline }
begin
  wrtChars(PWideChar(ws), Length(ws));
end;

{ No checks here - buffer always has 32 extra bytes }
procedure THTMLWriter.wrtChr(c: WideChar); { inline }
begin
  FBufPos^ := char(ord(c));
  Inc(FBufPos);
end;

procedure THTMLWriter.wrtIndent; { inline }
begin
  wrtChars(#10, 1);
end;

procedure THTMLWriter.wrtQuotedLiteral(const ws: WideString);
var
  Quote: WideChar;
begin
  // TODO: need to check if the string also contains single quote
  // both quotes present is a error
  if Pos('"', ws) > 0 then
    Quote := ''''
  else
    Quote := '"';
  wrtChr(Quote);
  wrtStr(ws);
  wrtChr(Quote);
>>>>>>> origin/fixes_2_2
end;

procedure THTMLWriter.wrtQuotedLiteral(const ws: WideString);
var
  Quote: WideChar;
begin
  // TODO: need to check if the string also contains single quote
  // both quotes present is a error
  if Pos('"', ws) > 0 then
    Quote := ''''
  else
    Quote := '"';
  wrtChr(Quote);
  wrtStr(ws);
  wrtChr(Quote);
end;

>>>>>>> graemeg/fixes_2_2
const
  AttrSpecialChars = ['<', '"', '&'];
  TextSpecialChars = ['<', '>', '&'];

<<<<<<< HEAD
<<<<<<< HEAD
procedure THTMLWriter.ConvWrite(const s: XMLString; const SpecialChars: TSetOfChar;
=======
procedure THTMLWriter.ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
>>>>>>> graemeg/fixes_2_2
=======
procedure THTMLWriter.ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
>>>>>>> origin/fixes_2_2
  const SpecialCharCallback: TSpecialCharCallback);
var
  StartPos, EndPos: Integer;
begin
  StartPos := 1;
  EndPos := 1;
  while EndPos <= Length(s) do
  begin
    if (s[EndPos] < #255) and (Char(ord(s[EndPos])) in SpecialChars) then
    begin
      wrtChars(@s[StartPos], EndPos - StartPos);
      SpecialCharCallback(s[EndPos]);
      StartPos := EndPos + 1;
    end;
    Inc(EndPos);
  end;
  if StartPos <= length(s) then
    wrtChars(@s[StartPos], EndPos - StartPos);
end;

const
  QuotStr = '&quot;';
  AmpStr = '&amp;';
  ltStr = '&lt;';
  gtStr = '&gt;';

procedure THTMLWriter.AttrSpecialCharCallback(c: WideChar);
begin
  case c of
    '"': wrtStr(QuotStr);
    '&': wrtStr(AmpStr);
    '<': wrtStr(ltStr);
  else
    wrtChr(c);
  end;
end;

procedure THTMLWriter.TextnodeSpecialCharCallback(c: WideChar);
begin
  case c of
    '<': wrtStr(ltStr);
    '>': wrtStr(gtStr); // Required only in ']]>' literal, otherwise optional
    '&': wrtStr(AmpStr);
  else
    wrtChr(c);
  end;
end;

procedure THTMLWriter.WriteNode(node: TDOMNode);
begin
  case node.NodeType of
    ELEMENT_NODE:                VisitElement(node);
    ATTRIBUTE_NODE:              VisitAttribute(node);
    TEXT_NODE:                   VisitText(node);
    CDATA_SECTION_NODE:          VisitCDATA(node);
    ENTITY_REFERENCE_NODE:       VisitEntityRef(node);
    PROCESSING_INSTRUCTION_NODE: VisitPI(node);
    COMMENT_NODE:                VisitComment(node);
    DOCUMENT_NODE:               VisitDocument(node);
    DOCUMENT_TYPE_NODE:          VisitDocumentType(node);
    ENTITY_NODE,
    DOCUMENT_FRAGMENT_NODE:      VisitFragment(node);
  end;
end;


procedure THTMLWriter.VisitElement(node: TDOMNode);
var
  i: Integer;
  child: TDOMNode;
  SavedInsideTextNode: Boolean;
  s: string;
  ElFlags: THTMLElementFlags;
  j: THTMLElementTag;
<<<<<<< HEAD
<<<<<<< HEAD
  meta: Boolean;
begin
  if not FInsideTextNode then
    wrtIndent;

  meta := False;
=======
begin
  if not FInsideTextNode then
    wrtIndent;
    
>>>>>>> graemeg/fixes_2_2
=======
begin
  if not FInsideTextNode then
    wrtIndent;
    
>>>>>>> origin/fixes_2_2
  s := LowerCase(node.NodeName);
  ElFlags := [efSubelementContent, efPCDATAContent];    // default flags
  for j := Low(THTMLElementTag) to High(THTMLElementTag) do
    if HTMLElementProps[J].Name = s then
    begin
      ElFlags := HTMLElementProps[j].Flags;
      if j = etMeta then
        meta := True;
      break;
    end;

  wrtChr('<');
  wrtStr(TDOMElement(node).TagName);
<<<<<<< HEAD
<<<<<<< HEAD

  { Force charset label to utf-8, because it is the encoding we actually write }
  if meta then
  begin
    s := TDOMElement(node).GetAttribute('http-equiv');
    if SameText(s, 'content-type') then
    begin
      wrtStr(' content="text/html; charset=utf-8" http-equiv="Content-Type">');
      Exit;
    end;
  end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  if node.HasAttributes then
    for i := 0 to node.Attributes.Length - 1 do
    begin
      child := node.Attributes.Item[i];
      VisitAttribute(child);
    end;
  wrtChr('>');
  Child := node.FirstChild;
  if Child <> nil then
  begin
    SavedInsideTextNode := FInsideTextNode;
    FInsideTextNode := efPCDATAContent in ElFlags;
    repeat
      WriteNode(Child);
      Child := Child.NextSibling;
    until Child = nil;
    FInsideTextNode := SavedInsideTextNode;
  end;
  if (not FInsideTextNode) and not (efPCDATAContent in ElFlags) then
    wrtIndent;
  if ElFlags * [efSubelementContent, efPCDATAContent] <> [] then
  begin
    wrtChars('</', 2);
    wrtStr(TDOMElement(Node).TagName);
    wrtChr('>');
  end;
end;

procedure THTMLWriter.VisitText(node: TDOMNode);
begin
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, {$IFDEF FPC}@{$ENDIF}TextnodeSpecialCharCallback);
end;

procedure THTMLWriter.VisitCDATA(node: TDOMNode);
begin
  if not FInsideTextNode then
    wrtIndent;
  wrtChars('<![CDATA[', 9);
  wrtStr(TDOMCharacterData(node).Data);
  wrtChars(']]>', 3);
end;

procedure THTMLWriter.VisitEntityRef(node: TDOMNode);
begin
  wrtChr('&');
  wrtStr(node.NodeName);
  wrtChr(';');
end;

procedure THTMLWriter.VisitPI(node: TDOMNode);
begin
  if not FInsideTextNode then wrtIndent;
  wrtStr('<?');
  wrtStr(TDOMProcessingInstruction(node).Target);
  wrtChr(' ');
  wrtStr(TDOMProcessingInstruction(node).Data);
  wrtStr('?>');
end;

procedure THTMLWriter.VisitComment(node: TDOMNode);
begin
  if not FInsideTextNode then wrtIndent;
  wrtChars('<!--', 4);
  wrtStr(TDOMCharacterData(node).Data);
  wrtChars('-->', 3);
end;

procedure THTMLWriter.VisitDocument(node: TDOMNode);
var
  child: TDOMNode;
begin
  child := node.FirstChild;
  while Assigned(Child) do
  begin
    WriteNode(Child);
    Child := Child.NextSibling;
  end;
  wrtChars(#10, 1);
end;

procedure THTMLWriter.VisitAttribute(Node: TDOMNode);
var
  Child: TDOMNode;
begin
  wrtChr(' ');
  wrtStr(TDOMAttr(Node).Name);
  wrtChars('="', 2);
  Child := Node.FirstChild;
  while Assigned(Child) do
  begin
    case Child.NodeType of
      ENTITY_REFERENCE_NODE:
        VisitEntityRef(Child);
      TEXT_NODE:
        ConvWrite(TDOMCharacterData(Child).Data, AttrSpecialChars, {$IFDEF FPC}@{$ENDIF}AttrSpecialCharCallback);
    end;
    Child := Child.NextSibling;
  end;
  wrtChr('"');
end;

procedure THTMLWriter.VisitDocumentType(Node: TDOMNode);
begin
  wrtStr('<!DOCTYPE ');
  wrtStr(Node.NodeName);
  wrtChr(' ');
  with TDOMDocumentType(Node) do
  begin
    if PublicID <> '' then
    begin
      wrtStr('PUBLIC ');
      wrtQuotedLiteral(PublicID);
      if SystemID <> '' then
      begin
        wrtChr(' ');
        wrtQuotedLiteral(SystemID);
      end;  
    end
    else if SystemID <> '' then
    begin
      wrtStr('SYSTEM ');
      wrtQuotedLiteral(SystemID);
    end;
  end;
  wrtChr('>');
end;

procedure THTMLWriter.VisitFragment(Node: TDOMNode);
var
  Child: TDOMNode;
begin
  // Fragment itself should not be written, only its children should...
  Child := Node.FirstChild;
  while Assigned(Child) do
  begin
    WriteNode(Child);
    Child := Child.NextSibling;
  end;
end;


// -------------------------------------------------------------------
//   Interface implementation
// -------------------------------------------------------------------

procedure WriteHTMLFile(doc: TXMLDocument; const AFileName: String);
var
  fs: TFileStream;
begin
  fs := TFileStream.Create(AFileName, fmCreate);
  try
    WriteHTMLFile(doc, fs);
  finally
    fs.Free;
  end;
end;

procedure WriteHTMLFile(doc: TXMLDocument; var AFile: Text);
var
  s: TStream;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  s := TTextStream.Create(AFile);
  try
    with THTMLWriter.Create(s) do
    try
      WriteNode(doc);
    finally
      Free;
    end;
  finally
    s.Free;
=======
=======
>>>>>>> origin/fixes_2_2
  with TTextHTMLWriter.Create(AFile) do
  try
    WriteNode(doc);
  finally
    Free;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

procedure WriteHTMLFile(doc: TXMLDocument; AStream: TStream);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  with THTMLWriter.Create(AStream) do
=======
  with TStreamHTMLWriter.Create(AStream) do
>>>>>>> graemeg/fixes_2_2
=======
  with TStreamHTMLWriter.Create(AStream) do
>>>>>>> origin/fixes_2_2
  try
    WriteNode(doc);
  finally
    Free;
  end;
end;

procedure WriteHTML(Element: TDOMNode; const AFileName: String);
begin
  WriteHTMLFile(TXMLDocument(Element), AFileName);
end;

procedure WriteHTML(Element: TDOMNode; var AFile: Text);
begin
  WriteHTMLFile(TXMLDocument(Element), AFile);
end;

procedure WriteHTML(Element: TDOMNode; AStream: TStream);
begin
  WriteHTMLFile(TXMLDocument(Element), AStream);
end;


end.
