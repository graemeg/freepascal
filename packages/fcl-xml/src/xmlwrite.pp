{
    This file is part of the Free Component Library

    XML writing routines
    Copyright (c) 1999-2000 by Sebastian Guenther, sg@freepascal.org
    Modified in 2006 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


unit XMLWrite;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef fpc}{$MODE objfpc}{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
{$MODE objfpc}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef fpc}{$MODE objfpc}{$endif}
>>>>>>> origin/fixes_2.4
{$H+}

interface

uses Classes, DOM;

procedure WriteXMLFile(doc: TXMLDocument; const AFileName: String); overload;
procedure WriteXMLFile(doc: TXMLDocument; var AFile: Text); overload;
procedure WriteXMLFile(doc: TXMLDocument; AStream: TStream); overload;

procedure WriteXML(Element: TDOMNode; const AFileName: String); overload;
procedure WriteXML(Element: TDOMNode; var AFile: Text); overload;
procedure WriteXML(Element: TDOMNode; AStream: TStream); overload;


// ===================================================================

implementation

uses SysUtils, xmlutils;

type
<<<<<<< HEAD
<<<<<<< HEAD
  TXMLWriter = class;
  TSpecialCharCallback = procedure(Sender: TXMLWriter; const s: DOMString;
    var idx: Integer);

  PAttrFixup = ^TAttrFixup;
  TAttrFixup = record
    Attr: TDOMNode;
    Prefix: PHashItem;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  TNodeInfo = record
    Name: XMLString;
  end;

  TNodeInfoArray = array of TNodeInfo;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  TSpecialCharCallback = procedure(c: WideChar) of object;
>>>>>>> graemeg/fixes_2_2

  TXMLWriter = class(TObject)
  private
    FStream: TStream;
    FInsideTextNode: Boolean;
    FCanonical: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FIndent: XMLString;
    FNesting: Integer;
    FBuffer: PChar;
    FBufPos: PChar;
    FCapacity: Integer;
    FLineBreak: XMLString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FIndent: WideString;
    FIndentCount: Integer;
    FBuffer: PChar;
    FBufPos: PChar;
    FCapacity: Integer;
    FLineBreak: WideString;
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
    FNSHelper: TNSSupport;
    FAttrFixups: TFPList;
    FScratch: TFPList;
    FNSDefs: TFPList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FNodes: TNodeInfoArray;
    procedure WriteXMLDecl(const aVersion, aEncoding: XMLString;
      aStandalone: Integer);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure wrtChars(Src: PWideChar; Length: Integer);
    procedure IncNesting;
    procedure DecNesting; {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtStr(const ws: XMLString); {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtChr(c: WideChar); {$IFDEF HAS_INLINE} inline; {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
    procedure wrtIndent(EndElement: Boolean = False);
    procedure wrtQuotedLiteral(const ws: XMLString);
    procedure ConvWrite(const s: XMLString; const SpecialChars: TSetOfChar;
=======
    procedure wrtIndent; {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtQuotedLiteral(const ws: WideString);
    procedure ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
>>>>>>> graemeg/fixes_2_2
=======
    procedure wrtIndent; {$IFDEF HAS_INLINE} inline; {$ENDIF}
    procedure wrtQuotedLiteral(const ws: WideString);
    procedure ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
>>>>>>> origin/fixes_2_2
      const SpecialCharCallback: TSpecialCharCallback);
    procedure WriteNSDef(B: TBinding);
    procedure NamespaceFixup(Element: TDOMElement);
  protected
    procedure WriteNode(Node: TDOMNode);
    procedure VisitDocument(Node: TDOMNode);
    procedure VisitDocument_Canonical(Node: TDOMNode);
    procedure VisitElement(Node: TDOMNode);
    procedure WriteString(const Text: XMLString);
    procedure WriteCDATA(const Text: XMLString);
    procedure WriteComment(const Text: XMLString);
    procedure VisitFragment(Node: TDOMNode);
    procedure VisitAttribute(Node: TDOMNode);
    procedure VisitEntityRef(Node: TDOMNode);
    procedure VisitDocumentType(Node: TDOMNode);
    procedure VisitPI(Node: TDOMNode);

    procedure WriteStartElement(const Name: XMLString);
    procedure WriteEndElement(shortForm: Boolean);
    procedure WriteProcessingInstruction(const Target, Data: XMLString);
    procedure WriteEntityRef(const Name: XMLString);
    procedure WriteAttributeString(const Name, Value: XMLString);
    procedure WriteDocType(const Name, PubId, SysId, Subset: XMLString);
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor Create(AStream: TStream; ANameTable: THashTable);
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
    TXMLWriter
  ---------------------------------------------------------------------}

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AttrSpecialChars = ['<', '>', '"', '&', #0..#$1F];
  TextSpecialChars = ['<', '>', '&', #0..#8, #10..#$1F];
  CDSectSpecialChars = [#0..#8, #11, #12, #14..#$1F, ']'];
=======
  AttrSpecialChars = ['<', '"', '&', #9, #10, #13];
  TextSpecialChars = ['<', '>', '&', #10, #13];
  CDSectSpecialChars = [']'];
>>>>>>> graemeg/cpstrnew
=======
  AttrSpecialChars = ['<', '"', '&', #9, #10, #13];
  TextSpecialChars = ['<', '>', '&', #10, #13];
  CDSectSpecialChars = [']'];
>>>>>>> graemeg/cpstrnew
=======
  AttrSpecialChars = ['<', '"', '&', #9, #10, #13];
  TextSpecialChars = ['<', '>', '&', #10, #13];
  CDSectSpecialChars = [']'];
>>>>>>> graemeg/cpstrnew
=======
  AttrSpecialChars = ['<', '"', '&', #9, #10, #13];
  TextSpecialChars = ['<', '>', '&', #10, #13];
  CDSectSpecialChars = [']'];
>>>>>>> origin/cpstrnew
  LineEndingChars = [#13, #10];
  QuotStr = '&quot;';
  AmpStr = '&amp;';
  ltStr = '&lt;';
  gtStr = '&gt;';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TXMLWriter.Create(AStream: TStream; ANameTable: THashTable);
=======
constructor TXMLWriter.Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
constructor TXMLWriter.Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
constructor TXMLWriter.Create(AStream: TStream);
>>>>>>> graemeg/cpstrnew
=======
constructor TXMLWriter.Create(AStream: TStream);
>>>>>>> origin/cpstrnew
var
  I: Integer;
begin
  inherited Create;
  FStream := AStream;
  // some overhead - always be able to write at least one extra UCS4
  FBuffer := AllocMem(512+32);
  FBufPos := FBuffer;
  FCapacity := 512;
  // Later on, this may be put under user control
  // for now, take OS setting
  if FCanonical then
    FLineBreak := #10
  else
    FLineBreak := sLineBreak;
  // Initialize Indent string
  // TODO: this must be done in setter of FLineBreak
  SetLength(FIndent, 100);
<<<<<<< HEAD
<<<<<<< HEAD
  FIndent[1] := FLineBreak[1];
  if Length(FLineBreak) > 1 then
    FIndent[2] := FLineBreak[2]
  else
    FIndent[2] := ' ';
  for I := 3 to 100 do FIndent[I] := ' ';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FNesting := 0;
  SetLength(FNodes, 16);
  FNSHelper := TNSSupport.Create(ANameTable);
=======
=======
  FIndent[1] := #10;
  for I := 2 to 100 do FIndent[I] := ' ';
>>>>>>> graemeg/fixes_2_2
=======
  FIndent[1] := #10;
  for I := 2 to 100 do FIndent[I] := ' ';
>>>>>>> origin/fixes_2_2
  FIndentCount := 0;
  FNSHelper := TNSSupport.Create;
>>>>>>> graemeg/cpstrnew
=======
  FIndentCount := 0;
  FNSHelper := TNSSupport.Create;
>>>>>>> graemeg/cpstrnew
=======
  FIndentCount := 0;
  FNSHelper := TNSSupport.Create;
>>>>>>> graemeg/cpstrnew
=======
  FIndentCount := 0;
  FNSHelper := TNSSupport.Create;
>>>>>>> origin/cpstrnew
  FScratch := TFPList.Create;
  FNSDefs := TFPList.Create;
  FAttrFixups := TFPList.Create;
end;

destructor TXMLWriter.Destroy;
var
  I: Integer;
begin
  for I := FAttrFixups.Count-1 downto 0 do
    Dispose(PAttrFixup(FAttrFixups.List^[I]));
  FAttrFixups.Free;
  FNSDefs.Free;
  FScratch.Free;
  FNSHelper.Free;
  if FBufPos > FBuffer then
    FStream.write(FBuffer^, FBufPos-FBuffer);

  FreeMem(FBuffer);
  inherited Destroy;
end;

procedure TXMLWriter.wrtChars(Src: PWideChar; Length: Integer);
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
<<<<<<< HEAD
<<<<<<< HEAD
      0..$7F:  begin
=======
=======
>>>>>>> origin/fixes_2_2
      $0A: pb := StrECopy(pb, PChar(FLineBreak));

      0..$09, $0B..$7F:  begin
>>>>>>> graemeg/fixes_2_2
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

procedure TXMLWriter.wrtStr(const ws: XMLString); { inline }
begin
  wrtChars(PWideChar(ws), Length(ws));
end;

{ No checks here - buffer always has 32 extra bytes }
procedure TXMLWriter.wrtChr(c: WideChar); { inline }
begin
  FBufPos^ := char(ord(c));
  Inc(FBufPos);
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TXMLWriter.wrtIndent(EndElement: Boolean);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  wrtChars(PWideChar(FIndent), (FNesting-ord(EndElement))*2+Length(FLineBreak));
=======
  wrtChars(PWideChar(FIndent), FIndentCount*2+Length(FLineBreak));
>>>>>>> graemeg/cpstrnew
=======
  wrtChars(PWideChar(FIndent), FIndentCount*2+Length(FLineBreak));
>>>>>>> graemeg/cpstrnew
=======
  wrtChars(PWideChar(FIndent), FIndentCount*2+Length(FLineBreak));
>>>>>>> graemeg/cpstrnew
=======
  wrtChars(PWideChar(FIndent), FIndentCount*2+Length(FLineBreak));
>>>>>>> origin/cpstrnew
end;

procedure TXMLWriter.IncNesting;
=======
=======
>>>>>>> origin/fixes_2_2
end;

procedure TXMLWriter.wrtIndent; { inline }
begin
  wrtChars(PWideChar(FIndent), FIndentCount*2+1);
end;

procedure TXMLWriter.IncIndent;
>>>>>>> graemeg/fixes_2_2
var
  I, NewLen, OldLen: Integer;
begin
  Inc(FNesting);
  if FNesting >= Length(FNodes) then
    SetLength(FNodes, FNesting+8);
  if Length(FIndent) < 2 * FNesting then
  begin
    OldLen := Length(FIndent);
    NewLen := 4 * FNesting;
    SetLength(FIndent, NewLen);
    for I := OldLen to NewLen do
      FIndent[I] := ' ';
  end;
end;

<<<<<<< HEAD
procedure TXMLWriter.DecNesting; { inline }
begin
  if FNesting>0 then dec(FNesting);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLWriter.ConvWrite(const s: XMLString; const SpecialChars: TSetOfChar;
=======
=======
procedure TXMLWriter.DecIndent; { inline }
begin
  if FIndentCount>0 then dec(FIndentCount);
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TXMLWriter.ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
>>>>>>> graemeg/cpstrnew
=======
const
  AttrSpecialChars = ['<', '"', '&', #9, #10, #13];
  TextSpecialChars = ['<', '>', '&'];

procedure TXMLWriter.ConvWrite(const s: WideString; const SpecialChars: TSetOfChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  const SpecialCharCallback: TSpecialCharCallback);
var
  StartPos, EndPos: Integer;
begin
  StartPos := 1;
  EndPos := 1;
  while EndPos <= Length(s) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    if (s[EndPos] < #128) and (Char(ord(s[EndPos])) in SpecialChars) then
=======
    if (s[EndPos] < #255) and (Char(ord(s[EndPos])) in SpecialChars) then
>>>>>>> graemeg/fixes_2_2
=======
    if (s[EndPos] < #255) and (Char(ord(s[EndPos])) in SpecialChars) then
>>>>>>> origin/fixes_2_2
    begin
      wrtChars(@s[StartPos], EndPos - StartPos);
      SpecialCharCallback(Self, s, EndPos);
      StartPos := EndPos + 1;
    end;
    Inc(EndPos);
  end;
  if StartPos <= length(s) then
    wrtChars(@s[StartPos], EndPos - StartPos);
end;

procedure AttrSpecialCharCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
  case s[idx] of
    '"': Sender.wrtStr(QuotStr);
    '&': Sender.wrtStr(AmpStr);
    '<': Sender.wrtStr(ltStr);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    // This is *only* to interoperate with broken parsers out there,
    // Delphi ClientDataset parser being one of them.
    '>': if not Sender.FCanonical then
           Sender.wrtStr(gtStr)
         else
           Sender.wrtChr('>');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    // Escape whitespace using CharRefs to be consistent with W3 spec § 3.3.3
    #9: Sender.wrtStr('&#x9;');
    #10: Sender.wrtStr('&#xA;');
    #13: Sender.wrtStr('&#xD;');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  else
    raise EConvertError.Create('Illegal character');
  end;
end;

procedure TextnodeNormalCallback(Sender: TXMLWriter; const s: DOMString;
=======
  else
    Sender.wrtChr(s[idx]);
  end;
end;

procedure TextnodeNormalCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
  case s[idx] of
    '<': Sender.wrtStr(ltStr);
    '>': Sender.wrtStr(gtStr); // Required only in ']]>' literal, otherwise optional
    '&': Sender.wrtStr(AmpStr);
    #13:
      begin
        // We normalize #13#10 and #13 to FLineBreak, going somewhat
        // beyond the specs here, see issue #13879.
        Sender.wrtStr(Sender.FLineBreak);
        if (idx < Length(s)) and (s[idx+1] = #10) then
          Inc(idx);
      end;
    #10: Sender.wrtStr(Sender.FLineBreak);
  else
    Sender.wrtChr(s[idx]);
  end;
end;

procedure TextnodeCanonicalCallback(Sender: TXMLWriter; const s: DOMString;
>>>>>>> graemeg/cpstrnew
  var idx: Integer);
begin
  case s[idx] of
    '<': Sender.wrtStr(ltStr);
<<<<<<< HEAD
    '>': Sender.wrtStr(gtStr); // Required only in ']]>' literal, otherwise optional
    '&': Sender.wrtStr(AmpStr);
    #13:
      begin
        // We normalize #13#10 and #13 to FLineBreak, going somewhat
        // beyond the specs here, see issue #13879.
        Sender.wrtStr(Sender.FLineBreak);
        if (idx < Length(s)) and (s[idx+1] = #10) then
          Inc(idx);
      end;
    #10: Sender.wrtStr(Sender.FLineBreak);
  else
    raise EConvertError.Create('Illegal character');
  end;
end;

=======
  else
    Sender.wrtChr(s[idx]);
  end;
end;

procedure TextnodeNormalCallback(Sender: TXMLWriter; const s: DOMString;
=======
  else
    Sender.wrtChr(s[idx]);
  end;
end;

procedure TextnodeNormalCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
  case s[idx] of
    '<': Sender.wrtStr(ltStr);
    '>': Sender.wrtStr(gtStr); // Required only in ']]>' literal, otherwise optional
    '&': Sender.wrtStr(AmpStr);
    #13:
      begin
        // We normalize #13#10 and #13 to FLineBreak, going somewhat
        // beyond the specs here, see issue #13879.
        Sender.wrtStr(Sender.FLineBreak);
        if (idx < Length(s)) and (s[idx+1] = #10) then
          Inc(idx);
      end;
    #10: Sender.wrtStr(Sender.FLineBreak);
  else
    Sender.wrtChr(s[idx]);
  end;
end;

procedure TextnodeCanonicalCallback(Sender: TXMLWriter; const s: DOMString;
>>>>>>> origin/cpstrnew
  var idx: Integer);
begin
  case s[idx] of
    '<': Sender.wrtStr(ltStr);
<<<<<<< HEAD
    '>': Sender.wrtStr(gtStr); // Required only in ']]>' literal, otherwise optional
    '&': Sender.wrtStr(AmpStr);
    #13:
      begin
        // We normalize #13#10 and #13 to FLineBreak, going somewhat
        // beyond the specs here, see issue #13879.
        Sender.wrtStr(Sender.FLineBreak);
        if (idx < Length(s)) and (s[idx+1] = #10) then
          Inc(idx);
      end;
    #10: Sender.wrtStr(Sender.FLineBreak);
  else
    Sender.wrtChr(s[idx]);
  end;
end;

>>>>>>> graemeg/cpstrnew
=======
=======
    '>': Sender.wrtStr(gtStr);
    '&': Sender.wrtStr(AmpStr);
    #13: Sender.wrtStr('&#xD;')
>>>>>>> origin/cpstrnew
  else
    Sender.wrtChr(s[idx]);
  end;
end;

<<<<<<< HEAD
procedure TextnodeNormalCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
  case s[idx] of
    '<': Sender.wrtStr(ltStr);
    '>': Sender.wrtStr(gtStr); // Required only in ']]>' literal, otherwise optional
    '&': Sender.wrtStr(AmpStr);
    #13:
      begin
        // We normalize #13#10 and #13 to FLineBreak, going somewhat
        // beyond the specs here, see issue #13879.
        Sender.wrtStr(Sender.FLineBreak);
        if (idx < Length(s)) and (s[idx+1] = #10) then
          Inc(idx);
      end;
    #10: Sender.wrtStr(Sender.FLineBreak);
  else
    Sender.wrtChr(s[idx]);
  end;
end;

>>>>>>> graemeg/cpstrnew
procedure TextnodeCanonicalCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
  case s[idx] of
    '<': Sender.wrtStr(ltStr);
    '>': Sender.wrtStr(gtStr);
    '&': Sender.wrtStr(AmpStr);
<<<<<<< HEAD
<<<<<<< HEAD
    #13: Sender.wrtStr('&#xD;');
    #10: Sender.wrtChr(#10);
  else
    raise EConvertError.Create('Illegal character');
=======
    #13: Sender.wrtStr('&#xD;')
  else
    Sender.wrtChr(s[idx]);
>>>>>>> graemeg/cpstrnew
=======
    #13: Sender.wrtStr('&#xD;')
  else
    Sender.wrtChr(s[idx]);
>>>>>>> graemeg/cpstrnew
  end;
end;

procedure CDSectSpecialCharCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if s[idx]=']' then
  begin
    if (idx <= Length(s)-2) and (s[idx+1] = ']') and (s[idx+2] = '>') then
    begin
      Sender.wrtStr(']]]]><![CDATA[>');
      Inc(idx, 2);
      // TODO: emit warning 'cdata-section-splitted'
    end
    else
      Sender.wrtChr(']');
  end  
  else
    raise EConvertError.Create('Illegal character');
=======
    '>': Sender.wrtStr(gtStr);
    '&': Sender.wrtStr(AmpStr);
    #13: Sender.wrtStr('&#xD;')
  else
    Sender.wrtChr(s[idx]);
  end;
end;

procedure CDSectSpecialCharCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
procedure CDSectSpecialCharCallback(Sender: TXMLWriter; const s: DOMString;
  var idx: Integer);
begin
>>>>>>> origin/cpstrnew
  if (idx <= Length(s)-2) and (s[idx+1] = ']') and (s[idx+2] = '>') then
  begin
    Sender.wrtStr(']]]]><![CDATA[>');
    Inc(idx, 2);
    // TODO: emit warning 'cdata-section-splitted'
  end
  else
    Sender.wrtChr(s[idx]);
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
end;

const
  TextnodeCallbacks: array[boolean] of TSpecialCharCallback = (
    @TextnodeNormalCallback,
    @TextnodeCanonicalCallback
  );

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLWriter.wrtQuotedLiteral(const ws: XMLString);
=======
procedure TXMLWriter.wrtQuotedLiteral(const ws: WideString);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLWriter.wrtQuotedLiteral(const ws: WideString);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLWriter.wrtQuotedLiteral(const ws: WideString);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLWriter.wrtQuotedLiteral(const ws: WideString);
>>>>>>> origin/cpstrnew
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
  ConvWrite(ws, LineEndingChars, @TextnodeNormalCallback);
  wrtChr(Quote);
end;

procedure TXMLWriter.WriteNode(node: TDOMNode);
begin
  case node.NodeType of
    ELEMENT_NODE:                VisitElement(node);
    ATTRIBUTE_NODE:              VisitAttribute(node);
    TEXT_NODE:                   WriteString(TDOMCharacterData(node).Data);
    CDATA_SECTION_NODE:          WriteCDATA(TDOMCharacterData(node).Data);
    ENTITY_REFERENCE_NODE:       VisitEntityRef(node);
    PROCESSING_INSTRUCTION_NODE: VisitPI(node);
<<<<<<< HEAD
    COMMENT_NODE:                WriteComment(TDOMCharacterData(node).Data);
=======
    COMMENT_NODE:                VisitComment(node);
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
    DOCUMENT_NODE:
      if FCanonical then
        VisitDocument_Canonical(node)
      else
        VisitDocument(node);
    DOCUMENT_TYPE_NODE:          VisitDocumentType(node);
    ENTITY_NODE,
    DOCUMENT_FRAGMENT_NODE:      VisitFragment(node);
  end;
end;

procedure TXMLWriter.WriteNSDef(B: TBinding);
begin
  wrtChars(' xmlns', 6);
  if B.Prefix^.Key <> '' then
  begin
    wrtChr(':');
    wrtStr(B.Prefix^.Key);
  end;
  wrtChars('="', 2);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if Assigned(B.uri) then
    ConvWrite(B.uri^.Key, AttrSpecialChars, @AttrSpecialCharCallback);
=======
  ConvWrite(B.uri, AttrSpecialChars, @AttrSpecialCharCallback);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(B.uri, AttrSpecialChars, @AttrSpecialCharCallback);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(B.uri, AttrSpecialChars, @AttrSpecialCharCallback);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(B.uri, AttrSpecialChars, @AttrSpecialCharCallback);
>>>>>>> origin/cpstrnew
  wrtChr('"');
end;

// clone of system.FPC_WIDESTR_COMPARE which cannot be called directly
function Compare(const s1, s2: DOMString): integer;
var
  maxi, temp: integer;
begin
  Result := 0;
  if pointer(S1) = pointer(S2) then
    exit;
  maxi := Length(S1);
  temp := Length(S2);
  if maxi > temp then
    maxi := temp;
  Result := CompareWord(S1[1], S2[1], maxi);
  if Result = 0 then
    Result := Length(S1)-Length(S2);
end;

function SortNSDefs(Item1, Item2: Pointer): Integer;
begin
  Result := Compare(TBinding(Item1).Prefix^.Key, TBinding(Item2).Prefix^.Key);
end;

function SortAtts(Item1, Item2: Pointer): Integer;
var
  p1: PAttrFixup absolute Item1;
  p2: PAttrFixup absolute Item2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result := Compare(p1^.Attr.namespaceURI, p2^.Attr.namespaceURI);
  if Result = 0 then
    Result := Compare(p1^.Attr.localName, p2^.Attr.localName);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  s1, s2: DOMString;
begin
  Result := Compare(p1^.Attr.namespaceURI, p2^.Attr.namespaceURI);
  if Result = 0 then
  begin
    // TODO: Must fix the parser so it doesn't produce Level 1 attributes
    if nfLevel2 in p1^.Attr.Flags then
      s1 := p1^.Attr.localName
    else
      s1 := p1^.Attr.nodeName;
    if nfLevel2 in p2^.Attr.Flags then
      s2 := p2^.Attr.localName
    else
      s2 := p2^.Attr.nodeName;
    Result := Compare(s1, s2);
  end;
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
end;

procedure TXMLWriter.NamespaceFixup(Element: TDOMElement);
var
  B: TBinding;
  i, j: Integer;
  node: TDOMNode;
  s: DOMString;
  action: TAttributeAction;
  p: PAttrFixup;
begin
  FScratch.Count := 0;
  FNSDefs.Count := 0;
  if Element.hasAttributes then
  begin
    j := 0;
    for i := 0 to Element.Attributes.Length-1 do
    begin
      node := Element.Attributes[i];
      if TDOMNode_NS(node).NSI.NSIndex = 2 then
      begin
        if TDOMNode_NS(node).NSI.PrefixLen = 0 then
          s := ''
        else
          s := node.localName;
        FNSHelper.DefineBinding(s, node.nodeValue, B);
        if Assigned(B) then  // drop redundant namespace declarations
          FNSDefs.Add(B);
      end
      else if FCanonical or TDOMAttr(node).Specified then
      begin
        // obtain a TAttrFixup record (allocate if needed)
        if j >= FAttrFixups.Count then
        begin
          New(p);
          FAttrFixups.Add(p);
        end
        else
          p := PAttrFixup(FAttrFixups.List^[j]);
        // add it to the working list
        p^.Attr := node;
        p^.Prefix := nil;
        FScratch.Add(p);
        Inc(j);
      end;
    end;
  end;

  FNSHelper.DefineBinding(Element.Prefix, Element.namespaceURI, B);
  if Assigned(B) then
    FNSDefs.Add(B);

  for i := 0 to FScratch.Count-1 do
  begin
    node := PAttrFixup(FScratch.List^[i])^.Attr;
    action := FNSHelper.CheckAttribute(node.Prefix, node.namespaceURI, B);
    if action = aaBoth then
      FNSDefs.Add(B);

    if action in [aaPrefix, aaBoth] then
      PAttrFixup(FScratch.List^[i])^.Prefix := B.Prefix;
  end;

  if FCanonical then
  begin
    FNSDefs.Sort(@SortNSDefs);
    FScratch.Sort(@SortAtts);
  end;

  // now, at last, dump all this stuff.
  for i := 0 to FNSDefs.Count-1 do
    WriteNSDef(TBinding(FNSDefs.List^[I]));

  for i := 0 to FScratch.Count-1 do
  begin
    wrtChr(' ');
    with PAttrFixup(FScratch.List^[I])^ do
    begin
      if Assigned(Prefix) then
      begin
        wrtStr(Prefix^.Key);
        wrtChr(':');
        wrtStr(Attr.localName);
      end
      else
        wrtStr(Attr.nodeName);

      wrtChars('="', 2);
      // TODO: not correct w.r.t. entities
      ConvWrite(attr.nodeValue, AttrSpecialChars, @AttrSpecialCharCallback);
      wrtChr('"');
    end;
  end;
end;

procedure TXMLWriter.VisitElement(node: TDOMNode);
var
  i: Integer;
  child: TDOMNode;
  SavedInsideTextNode: Boolean;
begin
<<<<<<< HEAD
  WriteStartElement(TDOMElement(node).TagName);
=======
  if not FInsideTextNode then
    wrtIndent;
  FNSHelper.StartElement;
  wrtChr('<');
  wrtStr(TDOMElement(node).TagName);
<<<<<<< HEAD
<<<<<<< HEAD
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

  if nfLevel2 in node.Flags then
    NamespaceFixup(TDOMElement(node))
  else if node.HasAttributes then
    for i := 0 to node.Attributes.Length - 1 do
    begin
      child := node.Attributes.Item[i];
      if FCanonical or TDOMAttr(child).Specified then
=======
=======
>>>>>>> origin/fixes_2_2
  // FIX: Accessing Attributes was causing them to be created for every element :(
  if node.HasAttributes then
    for i := 0 to node.Attributes.Length - 1 do
    begin
      child := node.Attributes.Item[i];
      if TDOMAttr(child).Specified then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        VisitAttribute(child);
    end;
  Child := node.FirstChild;
  if Child = nil then
<<<<<<< HEAD
<<<<<<< HEAD
    WriteEndElement(True)
=======
    wrtChars('/>', 2)
>>>>>>> graemeg/fixes_2_2
=======
    wrtChars('/>', 2)
>>>>>>> origin/fixes_2_2
  else
  begin
    // TODO: presence of zero-length textnodes triggers the indenting logic,
    // while they should be ignored altogeter.
    SavedInsideTextNode := FInsideTextNode;
    wrtChr('>');
<<<<<<< HEAD
<<<<<<< HEAD
    FInsideTextNode := FCanonical or (Child.NodeType in [TEXT_NODE, CDATA_SECTION_NODE]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
    FInsideTextNode := Child.NodeType in [TEXT_NODE, CDATA_SECTION_NODE];
>>>>>>> graemeg/fixes_2_2
=======
    FInsideTextNode := Child.NodeType in [TEXT_NODE, CDATA_SECTION_NODE];
>>>>>>> origin/fixes_2_2
    IncIndent;
>>>>>>> graemeg/cpstrnew
    repeat
      WriteNode(Child);
      Child := Child.NextSibling;
    until Child = nil;
<<<<<<< HEAD
    if not (node.LastChild.NodeType in [TEXT_NODE, CDATA_SECTION_NODE]) then
      wrtIndent(True);
    FInsideTextNode := SavedInsideTextNode;
    writeEndElement(False);
  end;
end;

procedure TXMLWriter.WriteStartElement(const Name: XMLString);
begin
  if not FInsideTextNode then
    wrtIndent;

  FNSHelper.PushScope;
  IncNesting;
  wrtChr('<');
  wrtStr(Name);
  FNodes[FNesting].Name := Name;
=======
    DecIndent;
    if not (node.LastChild.NodeType in [TEXT_NODE, CDATA_SECTION_NODE]) then
      wrtIndent;
    FInsideTextNode := SavedInsideTextNode;
    wrtChars('</', 2);
    wrtStr(TDOMElement(Node).TagName);
    wrtChr('>');
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLWriter.WriteEndElement(shortForm: Boolean);
begin
<<<<<<< HEAD
  if shortForm then
    wrtChars('/>', 2)
  else
  begin
    wrtChars('</', 2);
    wrtStr(FNodes[FNesting].Name);
    wrtChr('>');
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DecNesting;
  FNSHelper.PopScope;
=======
  FNSHelper.EndElement;
>>>>>>> graemeg/cpstrnew
=======
  FNSHelper.EndElement;
>>>>>>> graemeg/cpstrnew
=======
  FNSHelper.EndElement;
>>>>>>> graemeg/cpstrnew
=======
  FNSHelper.EndElement;
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2_2
end;

procedure TXMLWriter.WriteString(const Text: XMLString);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ConvWrite(Text, TextSpecialChars, TextnodeCallbacks[FCanonical]);
=======
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, TextnodeCallbacks[FCanonical]);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, TextnodeCallbacks[FCanonical]);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, TextnodeCallbacks[FCanonical]);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, TextnodeCallbacks[FCanonical]);
>>>>>>> origin/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, {$IFDEF FPC}@{$ENDIF}TextnodeSpecialCharCallback);
>>>>>>> graemeg/fixes_2_2
=======
  ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, {$IFDEF FPC}@{$ENDIF}TextnodeSpecialCharCallback);
>>>>>>> origin/fixes_2_2
end;

procedure TXMLWriter.WriteCDATA(const Text: XMLString);
begin
  if not FInsideTextNode then
    wrtIndent;
<<<<<<< HEAD
<<<<<<< HEAD
  if FCanonical then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ConvWrite(Text, TextSpecialChars, @TextnodeCanonicalCallback)
  else
  begin
    wrtChars('<![CDATA[', 9);
    ConvWrite(Text, CDSectSpecialChars, @CDSectSpecialCharCallback);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    ConvWrite(TDOMCharacterData(node).Data, TextSpecialChars, @TextnodeCanonicalCallback)
  else
  begin
    wrtChars('<![CDATA[', 9);
    ConvWrite(TDOMCharacterData(node).Data, CDSectSpecialChars, @CDSectSpecialCharCallback);
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
    wrtChars(']]>', 3);
  end;
=======
  wrtChars('<![CDATA[', 9);
  wrtStr(TDOMCharacterData(node).Data);
  wrtChars(']]>', 3);
>>>>>>> graemeg/fixes_2_2
=======
  wrtChars('<![CDATA[', 9);
  wrtStr(TDOMCharacterData(node).Data);
  wrtChars(']]>', 3);
>>>>>>> origin/fixes_2_2
end;

procedure TXMLWriter.WriteEntityRef(const Name: XMLString);
begin
  wrtChr('&');
  wrtStr(Name);
  wrtChr(';');
end;

procedure TXMLWriter.VisitEntityRef(node: TDOMNode);
begin
  WriteEntityRef(node.NodeName);
end;

procedure TXMLWriter.WriteProcessingInstruction(const Target, Data: XMLString);
begin
  if not FInsideTextNode then wrtIndent;
  wrtStr('<?');
<<<<<<< HEAD
  wrtStr(Target);
  if Data <> '' then
  begin
    wrtChr(' ');
    // TODO: How does this comply with c14n??
    ConvWrite(Data, LineEndingChars, @TextnodeNormalCallback);
=======
  wrtStr(TDOMProcessingInstruction(node).Target);
  if TDOMProcessingInstruction(node).Data <> '' then
  begin
    wrtChr(' ');
    // TODO: How does this comply with c14n??
    ConvWrite(TDOMProcessingInstruction(node).Data, LineEndingChars, @TextnodeNormalCallback);
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
  end;
  wrtStr('?>');
end;

procedure TXMLWriter.VisitPI(node: TDOMNode);
begin
  WriteProcessingInstruction(TDOMProcessingInstruction(node).Target, TDOMProcessingInstruction(node).Data);
end;

procedure TXMLWriter.WriteComment(const Text: XMLString);
begin
  if not FInsideTextNode then wrtIndent;
  wrtChars('<!--', 4);
<<<<<<< HEAD
<<<<<<< HEAD
  // TODO: How does this comply with c14n??
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ConvWrite(Text, LineEndingChars, @TextnodeNormalCallback);
=======
  ConvWrite(TDOMCharacterData(node).Data, LineEndingChars, @TextnodeNormalCallback);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, LineEndingChars, @TextnodeNormalCallback);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, LineEndingChars, @TextnodeNormalCallback);
>>>>>>> graemeg/cpstrnew
=======
  ConvWrite(TDOMCharacterData(node).Data, LineEndingChars, @TextnodeNormalCallback);
>>>>>>> origin/cpstrnew
=======
  wrtStr(TDOMCharacterData(node).Data);
>>>>>>> graemeg/fixes_2_2
=======
  wrtStr(TDOMCharacterData(node).Data);
>>>>>>> origin/fixes_2_2
  wrtChars('-->', 3);
end;

procedure TXMLWriter.WriteXMLDecl(const aVersion, aEncoding: XMLString; aStandalone: Integer);
begin
  wrtStr('<?xml version="');
  if aVersion <> '' then
    wrtStr(aVersion)
  else
    wrtStr('1.0');
  wrtChr('"');

  wrtStr(' encoding="');
  wrtStr(aEncoding);
  wrtChr('"');

  if aStandalone >= 0 then
  begin
    wrtStr(' standalone="');
    if aStandalone > 0 then
      wrtStr('yes')
    else
      wrtStr('no');
    wrtChr('"');
  end;
<<<<<<< HEAD

  wrtStr('?>');
end;

procedure TXMLWriter.VisitDocument(node: TDOMNode);
var
  child: TDOMNode;
begin
  // Here we ignore doc.xmlEncoding and write a fixed utf-8 label,
  // because it is the only output encoding currently supported.
  WriteXMLDecl(TXMLDocument(node).XMLVersion, 'utf-8', (ord(TXMLDocument(node).XMLStandalone)-1) or 1);
=======
*)
  wrtStr('?>');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  // TODO: now handled as a regular PI, remove this?
  if node is TXMLDocument then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    if Length(TXMLDocument(node).StylesheetType) > 0 then
    begin
      wrtStr(FLineBreak);
      wrtStr('<?xml-stylesheet type="');
      wrtStr(TXMLDocument(node).StylesheetType);
      wrtStr('" href="');
      wrtStr(TXMLDocument(node).StylesheetHRef);
      wrtStr('"?>');
    end;
=======
=======
>>>>>>> origin/fixes_2_2
    wrtStr(#10'<?xml-stylesheet type="');
    wrtStr(TXMLDocument(node).StylesheetType);
    wrtStr('" href="');
    wrtStr(TXMLDocument(node).StylesheetHRef);
    wrtStr('"?>');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  child := node.FirstChild;
  while Assigned(Child) do
  begin
    WriteNode(Child);
    Child := Child.NextSibling;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  wrtStr(FLineBreak);
end;

procedure TXMLWriter.VisitDocument_Canonical(Node: TDOMNode);
var
  child, root: TDOMNode;
begin
  root := TDOMDocument(Node).DocumentElement;
  child := node.FirstChild;
  while Assigned(child) and (child <> root) do
  begin
    if child.nodeType in [COMMENT_NODE, PROCESSING_INSTRUCTION_NODE] then
    begin
      WriteNode(child);
      wrtChr(#10);
    end;
    child := child.nextSibling;
  end;
  if root = nil then
    Exit;
  VisitElement(TDOMElement(root));
  child := root.nextSibling;
  while Assigned(child) do
  begin
    if child.nodeType in [COMMENT_NODE, PROCESSING_INSTRUCTION_NODE] then
    begin
      wrtChr(#10);
      WriteNode(child);
    end;
    child := child.nextSibling;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TXMLWriter.WriteAttributeString(const Name, Value: XMLString);
begin
  wrtChr(' ');
  wrtStr(Name);
  wrtChars('="', 2);
  ConvWrite(Value, AttrSpecialChars, {$IFDEF FPC}@{$ENDIF}AttrSpecialCharCallback);
  wrtChr('"');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  wrtChars(#10, 1);
>>>>>>> graemeg/fixes_2_2
=======
  wrtChars(#10, 1);
>>>>>>> origin/fixes_2_2
end;

procedure TXMLWriter.VisitAttribute(Node: TDOMNode);
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
<<<<<<< HEAD
<<<<<<< HEAD
        ConvWrite(TDOMCharacterData(Child).Data, AttrSpecialChars, @AttrSpecialCharCallback);
=======
        ConvWrite(TDOMCharacterData(Child).Data, AttrSpecialChars, {$IFDEF FPC}@{$ENDIF}AttrSpecialCharCallback);
>>>>>>> graemeg/fixes_2_2
=======
        ConvWrite(TDOMCharacterData(Child).Data, AttrSpecialChars, {$IFDEF FPC}@{$ENDIF}AttrSpecialCharCallback);
>>>>>>> origin/fixes_2_2
    end;
    Child := Child.NextSibling;
  end;
  wrtChr('"');
end;

procedure TXMLWriter.VisitDocumentType(Node: TDOMNode);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  WriteDocType(Node.NodeName, TDOMDocumentType(Node).PublicID, TDOMDocumentType(Node).SystemID,
               TDOMDocumentType(Node).InternalSubset);
end;

procedure TXMLWriter.WriteDocType(const Name, PubId, SysId, Subset: XMLString);
begin
  wrtStr(FLineBreak);
  wrtStr('<!DOCTYPE ');
  wrtStr(Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  wrtStr(FLineBreak);
  wrtStr('<!DOCTYPE ');
  wrtStr(Node.NodeName);
>>>>>>> graemeg/cpstrnew
  wrtChr(' ');
  if PubId <> '' then
  begin
<<<<<<< HEAD
    wrtStr('PUBLIC ');
    wrtQuotedLiteral(PubId);
    wrtChr(' ');
    wrtQuotedLiteral(SysId);
  end
  else if SysId <> '' then
  begin
    wrtStr('SYSTEM ');
    wrtQuotedLiteral(SysId);
  end;
  if Subset <> '' then
=======
  wrtStr(#10'<!DOCTYPE ');
  wrtStr(Node.NodeName);
=======
  wrtStr(#10'<!DOCTYPE ');
  wrtStr(Node.NodeName);
>>>>>>> origin/fixes_2_2
  wrtChr(' ');
  with TDOMDocumentType(Node) do
>>>>>>> graemeg/fixes_2_2
  begin
    wrtChr('[');
    ConvWrite(Subset, LineEndingChars, @TextnodeNormalCallback);
    wrtChr(']');
=======
    if PublicID <> '' then
    begin
      wrtStr('PUBLIC ');
      wrtQuotedLiteral(PublicID);
      wrtChr(' ');
      wrtQuotedLiteral(SystemID);
    end
    else if SystemID <> '' then
    begin
      wrtStr('SYSTEM ');
      wrtQuotedLiteral(SystemID);
    end;
    if InternalSubset <> '' then
    begin
      wrtChr('[');
      ConvWrite(InternalSubset, LineEndingChars, @TextnodeNormalCallback);
      wrtChr(']');
    end;
>>>>>>> graemeg/cpstrnew
  end;
  wrtChr('>');
end;

procedure TXMLWriter.VisitFragment(Node: TDOMNode);
var
  Child: TDOMNode;
begin
  // TODO: TextDecl is probably needed
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

procedure WriteXMLFile(doc: TXMLDocument; const AFileName: String);
begin
  WriteXML(doc, AFileName);
end;

procedure WriteXMLFile(doc: TXMLDocument; var AFile: Text);
var
  s: TStream;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  WriteXML(doc, AFile);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  s := TTextStream.Create(AFile);
  try
    with TXMLWriter.Create(s) do
    try
      WriteNode(doc);
    finally
      Free;
    end;
  finally
    s.Free;
  end;
>>>>>>> graemeg/cpstrnew
end;

procedure WriteXMLFile(doc: TXMLDocument; AStream: TStream);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  WriteXML(doc, AStream);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  with TXMLWriter.Create(AStream) do
  try
    WriteNode(doc);
  finally
    Free;
  end;
>>>>>>> graemeg/cpstrnew
end;

procedure WriteXML(Element: TDOMNode; const AFileName: String);
var
  fs: TFileStream;
begin
  fs := TFileStream.Create(AFileName, fmCreate);
  try
    WriteXML(Element, fs);
  finally
    fs.Free;
  end;
end;

procedure WriteXML(Element: TDOMNode; var AFile: Text);
var
  s: TStream;
  doc: TDOMDocument;
begin
  if Element.NodeType = DOCUMENT_NODE then
    doc := TDOMDocument(Element)
  else
    doc := Element.OwnerDocument;
  s := TTextStream.Create(AFile);
  try
    with TXMLWriter.Create(s, doc.Names) do
    try
      WriteNode(Element);
    finally
      Free;
    end;
  finally
    s.Free;
  end;
end;

procedure WriteXML(Element: TDOMNode; AStream: TStream);
var
  doc: TDOMDocument;
begin
  if Element.NodeType = DOCUMENT_NODE then
    doc := TDOMDocument(Element)
  else
    doc := Element.OwnerDocument;
  with TXMLWriter.Create(AStream, doc.Names) do
  try
    WriteNode(Element);
  finally
    Free;
  end;
end;



end.
