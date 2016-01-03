{
    This file is part of the Free Component Library

    XML utility routines.
    Copyright (c) 2006 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit xmlutils;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef fpc}{$mode objfpc}{$endif}
{$H+}
{$ifopt Q+}{$define overflow_check}{$endif}
=======
{$mode objfpc}
{$H+}
>>>>>>> graemeg/fixes_2_2
=======
{$mode objfpc}
=======
{$ifdef fpc}{$mode objfpc}{$endif}
>>>>>>> origin/fixes_2.4
{$H+}
>>>>>>> origin/fixes_2_2

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  SysUtils, Classes;

type
  TXMLVersion = (xmlVersionUnknown, xmlVersion10, xmlVersion11);
  TSetOfChar = set of Char;
  XMLString = WideString;
  PXMLString = ^XMLString;
  PXMLChar = PWideChar;

function IsXmlName(const Value: XMLString; Xml11: Boolean = False): Boolean; overload;
function IsXmlName(Value: PWideChar; Len: Integer; Xml11: Boolean = False): Boolean; overload;
function IsXmlNames(const Value: XMLString; Xml11: Boolean = False): Boolean;
function IsXmlNmToken(const Value: XMLString; Xml11: Boolean = False): Boolean;
function IsXmlNmTokens(const Value: XMLString; Xml11: Boolean = False): Boolean;
function IsValidXmlEncoding(const Value: XMLString): Boolean;

procedure NormalizeSpaces(var Value: XMLString);
function IsXmlWhiteSpace(c: WideChar): Boolean;
function Hash(InitValue: LongWord; Key: PWideChar; KeyLen: Integer): LongWord;
{ beware, works in ASCII range only }
function WStrLIComp(S1, S2: PWideChar; Len: Integer): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure WStrLower(var S: XMLString);


const
  xmlVersionStr: array[TXMLVersion] of XMLString = ('', '1.0', '1.1');
  // URIs of predefined namespaces
  stduri_xml: XMLString = 'http://www.w3.org/XML/1998/namespace';
  stduri_xmlns: XMLString = 'http://www.w3.org/2000/xmlns/';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure WStrLower(var S: WideString);

type
  TXMLVersion = (xmlVersionUnknown, xmlVersion10, xmlVersion11);

const
  xmlVersionStr: array[TXMLVersion] of WideString = ('', '1.0', '1.1');
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

type
  TXMLNodeType = (ntNone, ntElement, ntAttribute, ntText,
    ntCDATA, ntEntityReference, ntEntity, ntProcessingInstruction,
    ntComment, ntDocument, ntDocumentType, ntDocumentFragment,
    ntNotation,
    ntWhitespace,
    ntSignificantWhitespace,
    ntEndElement,
    ntEndEntity,
    ntXmlDeclaration
  );

  TAttrDataType = (
    dtCdata,
    dtId,
    dtIdRef,
    dtIdRefs,
    dtEntity,
    dtEntities,
    dtNmToken,
    dtNmTokens,
    dtNotation
  );
=======
=======
>>>>>>> origin/fixes_2_2
  SysUtils;

function IsXmlName(const Value: WideString; Xml11: Boolean = False): Boolean; overload;
function IsXmlName(Value: PWideChar; Len: Integer; Xml11: Boolean = False): Boolean; overload;
function IsXmlNames(const Value: WideString; Xml11: Boolean = False): Boolean;
function IsXmlNmToken(const Value: WideString; Xml11: Boolean = False): Boolean;
function IsXmlNmTokens(const Value: WideString; Xml11: Boolean = False): Boolean;
function IsValidXmlEncoding(const Value: WideString): Boolean;
function Xml11NamePages: PByteArray;
procedure NormalizeSpaces(var Value: WideString);
function Hash(InitValue: LongWord; Key: PWideChar; KeyLen: Integer): LongWord;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ a simple hash table with WideString keys }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef fpc}
  PtrInt = LongInt;
  TFPList = TList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
{$endif}  
>>>>>>> graemeg/cpstrnew
=======
{$endif}  
>>>>>>> graemeg/cpstrnew
=======
{$endif}  
>>>>>>> graemeg/cpstrnew
=======
{$endif}  
>>>>>>> origin/cpstrnew

  PPHashItem = ^PHashItem;
  PHashItem = ^THashItem;
  THashItem = record
    Key: XMLString;
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$ifndef fpc}
  PtrInt = LongInt;
{$endif}  

>>>>>>> origin/fixes_2.4
  PPHashItem = ^PHashItem;
  PHashItem = ^THashItem;
  THashItem = record
    Key: WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    HashValue: LongWord;
    Next: PHashItem;
    Data: TObject;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef CPU16}
  THashItemArray = array[0..MaxSmallInt div sizeof(Pointer)-1] of PHashItem;
{$else CPU16}
  THashItemArray = array[0..MaxInt div sizeof(Pointer)-1] of PHashItem;
{$endif CPU16}
=======
  THashItemArray = array[0..MaxInt div sizeof(Pointer)-1] of PHashItem;
>>>>>>> graemeg/cpstrnew
=======
  THashItemArray = array[0..MaxInt div sizeof(Pointer)-1] of PHashItem;
>>>>>>> graemeg/cpstrnew
=======
  THashItemArray = array[0..MaxInt div sizeof(Pointer)-1] of PHashItem;
>>>>>>> graemeg/cpstrnew
=======
  THashItemArray = array[0..MaxInt div sizeof(Pointer)-1] of PHashItem;
>>>>>>> origin/cpstrnew
  PHashItemArray = ^THashItemArray;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  THashItemArray = array[0..0] of PHashItem;
  PHashItemArray = ^THashItemArray;
>>>>>>> origin/fixes_2.4

  THashForEach = function(Entry: PHashItem; arg: Pointer): Boolean;

  THashTable = class(TObject)
  private
    FCount: LongWord;
    FBucketCount: LongWord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FBucket: PHashItemArray;
    FOwnsObjects: Boolean;
    function Lookup(Key: PWideChar; KeyLength: Integer; out Found: Boolean; CanCreate: Boolean): PHashItem;
=======
    FBucket: PPHashItem;
    FOwnsObjects: Boolean;
    function Lookup(Key: PWideChar; KeyLength: Integer; var Found: Boolean; CanCreate: Boolean): PHashItem;
>>>>>>> graemeg/fixes_2_2
=======
    FBucket: PPHashItem;
    FOwnsObjects: Boolean;
    function Lookup(Key: PWideChar; KeyLength: Integer; var Found: Boolean; CanCreate: Boolean): PHashItem;
>>>>>>> origin/fixes_2_2
=======
    FBucket: PHashItemArray;
    FOwnsObjects: Boolean;
    function Lookup(Key: PWideChar; KeyLength: Integer; out Found: Boolean; CanCreate: Boolean): PHashItem;
>>>>>>> origin/fixes_2.4
    procedure Resize(NewCapacity: LongWord);
  public
    constructor Create(InitSize: Integer; OwnObjects: Boolean);
    destructor Destroy; override;
    procedure Clear;
    function Find(Key: PWideChar; KeyLen: Integer): PHashItem;
    function FindOrAdd(Key: PWideChar; KeyLen: Integer; var Found: Boolean): PHashItem; overload;
    function FindOrAdd(Key: PWideChar; KeyLen: Integer): PHashItem; overload;
<<<<<<< HEAD
<<<<<<< HEAD
    function FindOrAdd(const Key: XMLString): PHashItem; overload;
    function Get(Key: PWideChar; KeyLen: Integer): TObject;
    function Remove(Entry: PHashItem): Boolean;
    function RemoveData(aData: TObject): Boolean;
=======
    function Get(Key: PWideChar; KeyLen: Integer): TObject;
    function Remove(Entry: PHashItem): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
    function Get(Key: PWideChar; KeyLen: Integer): TObject;
    function Remove(Entry: PHashItem): Boolean;
>>>>>>> origin/fixes_2_2
    procedure ForEach(proc: THashForEach; arg: Pointer);
    property Count: LongWord read FCount;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
{ another hash, for detecting duplicate namespaced attributes without memory allocations }

  TExpHashEntry = record
    rev: LongWord;
    hash: LongWord;
    uriPtr: Pointer;
    lname: PWideChar;
    lnameLen: Integer;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef CPU16}
  TExpHashEntryArray = array[0..MaxSmallInt div sizeof(TExpHashEntry)-1] of TExpHashEntry;
{$else CPU16}
  TExpHashEntryArray = array[0..MaxInt div sizeof(TExpHashEntry)-1] of TExpHashEntry;
{$endif CPU16}
=======
  TExpHashEntryArray = array[0..MaxInt div sizeof(TExpHashEntry)-1] of TExpHashEntry;
>>>>>>> graemeg/cpstrnew
=======
  TExpHashEntryArray = array[0..MaxInt div sizeof(TExpHashEntry)-1] of TExpHashEntry;
>>>>>>> graemeg/cpstrnew
=======
  TExpHashEntryArray = array[0..MaxInt div sizeof(TExpHashEntry)-1] of TExpHashEntry;
>>>>>>> graemeg/cpstrnew
=======
  TExpHashEntryArray = array[0..MaxInt div sizeof(TExpHashEntry)-1] of TExpHashEntry;
>>>>>>> origin/cpstrnew
=======
  TExpHashEntryArray = array[0..0] of TExpHashEntry;
>>>>>>> origin/fixes_2.4
  PExpHashEntryArray = ^TExpHashEntryArray;

  TDblHashArray = class(TObject)
  private
    FSizeLog: Integer;
    FRevision: LongWord;
    FData: PExpHashEntryArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  public
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  public  
>>>>>>> graemeg/cpstrnew
    procedure Init(NumSlots: Integer);
    function Locate(uri: Pointer; localName: PWideChar; localLength: Integer): Boolean;
    destructor Destroy; override;
  end;

{ Source location. This may be augmented with ByteOffset, UTF8Offset, etc. }
  TLocation = record
    Line: Integer;
    LinePos: Integer;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew

{ generic node info record, shared between DOM and reader }

  PNodeData = ^TNodeData;
  TNodeData = record
    FNext: PNodeData;
    FQName: PHashItem;
    FPrefix: PHashItem;
    FNsUri: PHashItem;
    FColonPos: Integer;
    FTypeInfo: TObject;
    FLoc: TLocation;
    FLoc2: TLocation;              // for attributes: start of value
    FIDEntry: PHashItem;           // ID attributes: entry in ID map
    FNodeType: TXMLNodeType;

    FValueStr: WideString;
    FValueStart: PWideChar;
    FValueLength: Integer;
    FIsDefault: Boolean;
    FDenormalized: Boolean;        // Whether attribute value changes by normalization
  end;

{ TNSSupport provides tracking of prefix-uri pairs and namespace fixup for writer }

  TBinding = class
  public
    uri: WideString;
    next: TBinding;
    prevPrefixBinding: TObject;
    Prefix: PHashItem;
  end;

  TAttributeAction = (
    aaUnchanged,
    aaPrefix,         // only override the prefix
    aaBoth            // override prefix and emit namespace definition
  );

  TNSSupport = class(TObject)
  private
    FNesting: Integer;
    FPrefixSeqNo: Integer;
    FFreeBindings: TBinding;
    FBindings: TFPList;
    FBindingStack: array of TBinding;
    FPrefixes: THashTable;
    FDefaultPrefix: THashItem;
  public
    constructor Create;
    destructor Destroy; override;
    procedure DefineBinding(const Prefix, nsURI: WideString; out Binding: TBinding);
    function CheckAttribute(const Prefix, nsURI: WideString;
      out Binding: TBinding): TAttributeAction;
    function IsPrefixBound(P: PWideChar; Len: Integer; out Prefix: PHashItem): Boolean;
    function GetPrefix(P: PWideChar; Len: Integer): PHashItem;
    function BindPrefix(const nsURI: WideString; aPrefix: PHashItem): TBinding;
    function DefaultNSBinding: TBinding;
    procedure StartElement;
    procedure EndElement;
  end;

<<<<<<< HEAD
=======
{ Source location. This may be augmented with ByteOffset, UTF8Offset, etc. }
  TLocation = record
    Line: Integer;
    LinePos: Integer;
  end;

{ generic node info record, shared between DOM and reader }

  PNodeData = ^TNodeData;
  TNodeData = record
    FNext: PNodeData;
    FQName: PHashItem;
    FPrefix: PHashItem;
    FNsUri: PHashItem;
    FColonPos: Integer;
    FTypeInfo: TObject;
    FLoc: TLocation;
    FLoc2: TLocation;              // for attributes: start of value
    FIDEntry: PHashItem;           // ID attributes: entry in ID map
    FNodeType: TXMLNodeType;

    FValueStr: WideString;
    FValueStart: PWideChar;
    FValueLength: Integer;
    FIsDefault: Boolean;
    FDenormalized: Boolean;        // Whether attribute value changes by normalization
  end;

<<<<<<< HEAD
=======
{ Source location. This may be augmented with ByteOffset, UTF8Offset, etc. }
  TLocation = record
    Line: Integer;
    LinePos: Integer;
  end;

{ generic node info record, shared between DOM and reader }

  PNodeData = ^TNodeData;
  TNodeData = record
    FNext: PNodeData;
    FQName: PHashItem;
    FPrefix: PHashItem;
    FNsUri: PHashItem;
    FColonPos: Integer;
    FTypeInfo: TObject;
    FLoc: TLocation;
    FLoc2: TLocation;              // for attributes: start of value
    FIDEntry: PHashItem;           // ID attributes: entry in ID map
    FNodeType: TXMLNodeType;

    FValueStr: WideString;
    FValueStart: PWideChar;
    FValueLength: Integer;
    FIsDefault: Boolean;
    FDenormalized: Boolean;        // Whether attribute value changes by normalization
  end;

>>>>>>> origin/cpstrnew
{ TNSSupport provides tracking of prefix-uri pairs and namespace fixup for writer }

  TBinding = class
  public
    uri: WideString;
    next: TBinding;
    prevPrefixBinding: TObject;
    Prefix: PHashItem;
  end;

  TAttributeAction = (
    aaUnchanged,
    aaPrefix,         // only override the prefix
    aaBoth            // override prefix and emit namespace definition
  );

  TNSSupport = class(TObject)
  private
    FNesting: Integer;
    FPrefixSeqNo: Integer;
    FFreeBindings: TBinding;
    FBindings: TFPList;
    FBindingStack: array of TBinding;
    FPrefixes: THashTable;
    FDefaultPrefix: THashItem;
  public
    constructor Create;
    destructor Destroy; override;
    procedure DefineBinding(const Prefix, nsURI: WideString; out Binding: TBinding);
    function CheckAttribute(const Prefix, nsURI: WideString;
      out Binding: TBinding): TAttributeAction;
    function IsPrefixBound(P: PWideChar; Len: Integer; out Prefix: PHashItem): Boolean;
    function GetPrefix(P: PWideChar; Len: Integer): PHashItem;
    function BindPrefix(const nsURI: WideString; aPrefix: PHashItem): TBinding;
    function DefaultNSBinding: TBinding;
    procedure StartElement;
    procedure EndElement;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Buffer builder, used to compose long strings without too much memory allocations }

  PWideCharBuf = ^TWideCharBuf;
  TWideCharBuf = record
    Buffer: PWideChar;
    Length: Integer;
    MaxLength: Integer;
  end;

procedure BufAllocate(var ABuffer: TWideCharBuf; ALength: Integer);
procedure BufAppend(var ABuffer: TWideCharBuf; wc: WideChar);
procedure BufAppendChunk(var ABuf: TWideCharBuf; pstart, pend: PWideChar);
function BufEquals(const ABuf: TWideCharBuf; const Arg: WideString): Boolean;
procedure BufNormalize(var Buf: TWideCharBuf; out Modified: Boolean);

{ Built-in decoder functions for UTF-8, UTF-16 and ISO-8859-1 }

function Decode_UCS2(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
function Decode_UCS2_Swapped(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
function Decode_UTF8(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
function Decode_8859_1(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;

{$i names.inc}
>>>>>>> graemeg/cpstrnew

  IXmlLineInfo = interface(IInterface)['{FD0A892B-B26C-4954-9995-103B2A9D178A}']
    function GetHasLineInfo: Boolean;
    function GetLineNumber: Integer;
    function GetLinePosition: Integer;
    property HasLineInfo: Boolean read GetHasLineInfo;
    property LineNumber: Integer read GetLineNumber;
    property LinePosition: Integer read GetLinePosition;
  end;

{ generic node info record, shared between DOM and reader }

  PPNodeData = ^PNodeData;
  PNodeData = ^TNodeData;
  TNodeData = record
    FNext: PNodeData;
    FQName: PHashItem;
    FPrefix: PHashItem;
    FNsUri: PHashItem;
    FColonPos: Integer;
    FTypeInfo: TObject;
    FLoc: TLocation;
    FLoc2: TLocation;              // for attributes: start of value
    FIDEntry: PHashItem;           // ID attributes: entry in ID map
    FNodeType: TXMLNodeType;

    FValueStr: XMLString;
    FValueStart: PWideChar;
    FValueLength: Integer;
    FIsDefault: Boolean;
    FDenormalized: Boolean;        // Whether attribute value changes by normalization
  end;

  IGetNodeDataPtr = interface(IInterface)['{81F6ADA2-8F5E-41D7-872D-226163FF4E45}']
    function CurrentNodePtr: PPNodeData;
  end;

{ TNSSupport provides tracking of prefix-uri pairs and namespace fixup for writer }

  TBinding = class
  public
    uri: PHashItem;
    next: TBinding;
    prevPrefixBinding: TObject;
    Prefix: PHashItem;
  end;

  TAttributeAction = (
    aaUnchanged,
    aaPrefix,         // only override the prefix
    aaBoth            // override prefix and emit namespace definition
  );

  TNSSupport = class(TObject)
  private
    FNameTable: THashTable;
    FNesting: Integer;
    FPrefixSeqNo: Integer;
    FFreeBindings: TBinding;
    FBindings: TFPList;
    FBindingStack: array of TBinding;
    FPrefixes: THashTable;
    FDefaultPrefix: THashItem;
  public
    constructor Create(aNameTable: THashTable);
    destructor Destroy; override;
    procedure DefineBinding(const Prefix, nsURI: XMLString; out Binding: TBinding);
    function CheckAttribute(const Prefix, nsURI: XMLString;
      out Binding: TBinding): TAttributeAction;
    function GetPrefix(P: PWideChar; Len: Integer): PHashItem;
    function BindPrefix(nsURI, aPrefix: PHashItem): TBinding;
    function DefaultNSBinding: TBinding;
    function LookupNamespace(const APrefix: XMLString): XMLString;
    procedure PushScope;
    function PopScope: Boolean;
  end;

{ Buffer builder, used to compose long strings without too much memory allocations }

  PWideCharBuf = ^TWideCharBuf;
  TWideCharBuf = record
    Buffer: PWideChar;
    Length: Integer;
    MaxLength: Integer;
  end;

procedure BufAllocate(var ABuffer: TWideCharBuf; ALength: Integer);
procedure BufAppend(var ABuffer: TWideCharBuf; wc: WideChar);
procedure BufAppendChunk(var ABuf: TWideCharBuf; pstart, pend: PWideChar);
procedure BufAppendString(var ABuf: TWideCharBuf; const AValue: XMLString);
function BufEquals(const ABuf: TWideCharBuf; const Arg: XMLString): Boolean;
procedure BufNormalize(var Buf: TWideCharBuf; out Modified: Boolean);

{ Built-in decoder functions for UTF-8, UTF-16 and ISO-8859-1 }

function Decode_UCS2(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
function Decode_UCS2_Swapped(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
function Decode_UTF8(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
function Decode_8859_1(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;

{$i names.inc}

implementation
=======
=======
>>>>>>> origin/fixes_2_2
{$i names.inc}

implementation

var
  Xml11Pg: PByteArray = nil;

function Xml11NamePages: PByteArray;
var
  I: Integer;
  p: PByteArray;
begin
  if Xml11Pg = nil then
  begin
    GetMem(p, 512);
    for I := 0 to 255 do
      p^[I] := ord(Byte(I) in Xml11HighPages);
    p^[0] := 2;
    p^[3] := $2c;
    p^[$20] := $2a;
    p^[$21] := $2b;
    p^[$2f] := $29;
    p^[$30] := $2d;
    p^[$fd] := $28;
    p^[$ff] := $30;

    Move(p^, p^[256], 256);
    p^[$100] := $19;
    p^[$103] := $2E;
    p^[$120] := $2F;
    Xml11Pg := p;
  end;
  Result := Xml11Pg;
end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function IsXml11Char(Value: PWideChar; var Index: Integer): Boolean; overload;
begin
  if (Value[Index] >= #$D800) and (Value[Index] <= #$DB7F) then
  begin
    Inc(Index);
    Result := (Value[Index] >= #$DC00) and (Value[Index] <= #$DFFF);
  end
  else
    Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsXml11Char(const Value: XMLString; var Index: Integer): Boolean; overload;
=======
function IsXml11Char(const Value: WideString; var Index: Integer): Boolean; overload;
>>>>>>> graemeg/fixes_2_2
=======
function IsXml11Char(const Value: WideString; var Index: Integer): Boolean; overload;
>>>>>>> origin/fixes_2_2
begin
  if (Value[Index] >= #$D800) and (Value[Index] <= #$DB7F) then
  begin
    Inc(Index);
    Result := (Value[Index] >= #$DC00) and (Value[Index] <= #$DFFF);
  end
  else
    Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsXmlName(const Value: XMLString; Xml11: Boolean): Boolean;
=======
function IsXmlName(const Value: WideString; Xml11: Boolean): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function IsXmlName(const Value: WideString; Xml11: Boolean): Boolean;
>>>>>>> origin/fixes_2_2
begin
  Result := IsXmlName(PWideChar(Value), Length(Value), Xml11);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function IsXmlName(Value: PWideChar; Len: Integer; Xml11: Boolean = False): Boolean;
var
  I: Integer;
begin
  Result := False;
  I := 0;
  if (Len = 0) or not ((Byte(Value[I]) in NamingBitmap[NamePages[hi(Word(Value[I]))]]) or
    (Value[I] = ':') or IsXml11Char(Value, I)) then
=======
=======
>>>>>>> origin/fixes_2_2
function IsXmlName(Value: PWideChar; Len: Integer; Xml11: Boolean = False): Boolean; overload;
=======
function IsXmlName(Value: PWideChar; Len: Integer; Xml11: Boolean = False): Boolean;
>>>>>>> origin/fixes_2.4
var
  Pages: PByteArray;
  I: Integer;
begin
  Result := False;
  if Xml11 then
    Pages := Xml11NamePages
  else
    Pages := @NamePages;

  I := 0;
  if (Len = 0) or not ((Byte(Value[I]) in NamingBitmap[Pages^[hi(Word(Value[I]))]]) or
    (Value[I] = ':') or
    (Xml11 and IsXml11Char(Value, I))) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      Exit;
  Inc(I);
  while I < Len do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    if not ((Byte(Value[I]) in NamingBitmap[NamePages[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or IsXml11Char(Value, I)) then
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> graemeg/fixes_2_2
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> origin/fixes_2_2
        Exit;
    Inc(I);
  end;
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsXmlNames(const Value: XMLString; Xml11: Boolean): Boolean;
var
  I: Integer;
  Offset: Integer;
begin
=======
=======
>>>>>>> origin/fixes_2_2
function IsXmlNames(const Value: WideString; Xml11: Boolean): Boolean;
var
  Pages: PByteArray;
  I: Integer;
  Offset: Integer;
begin
  if Xml11 then
    Pages := Xml11NamePages
  else
    Pages := @NamePages;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result := False;
  if Value = '' then
    Exit;
  I := 1;
  Offset := 0;
  while I <= Length(Value) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    if not ((Byte(Value[I]) in NamingBitmap[NamePages[Offset+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or IsXml11Char(Value, I)) then
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[Offset+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> graemeg/fixes_2_2
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[Offset+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> origin/fixes_2_2
    begin
      if (I = Length(Value)) or (Value[I] <> #32) then
        Exit;
      Offset := 0;
      Inc(I);
      Continue;
    end;
    Offset := $100;
    Inc(I);
  end;
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsXmlNmToken(const Value: XMLString; Xml11: Boolean): Boolean;
var
  I: Integer;
begin
=======
=======
>>>>>>> origin/fixes_2_2
function IsXmlNmToken(const Value: WideString; Xml11: Boolean): Boolean;
var
  I: Integer;
  Pages: PByteArray;
begin
  if Xml11 then
    Pages := Xml11NamePages
  else
    Pages := @NamePages;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result := False;
  if Value = '' then
    Exit;
  I := 1;
  while I <= Length(Value) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    if not ((Byte(Value[I]) in NamingBitmap[NamePages[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or IsXml11Char(Value, I)) then
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> graemeg/fixes_2_2
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> origin/fixes_2_2
        Exit;
    Inc(I);
  end;
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsXmlNmTokens(const Value: XMLString; Xml11: Boolean): Boolean;
var
  I: Integer;
begin
=======
=======
>>>>>>> origin/fixes_2_2
function IsXmlNmTokens(const Value: WideString; Xml11: Boolean): Boolean;
var
  I: Integer;
  Pages: PByteArray;
begin
  if Xml11 then
    Pages := Xml11NamePages
  else
    Pages := @NamePages;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  I := 1;
  Result := False;
  if Value = '' then
    Exit;
  while I <= Length(Value) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    if not ((Byte(Value[I]) in NamingBitmap[NamePages[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or IsXml11Char(Value, I)) then
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> graemeg/fixes_2_2
=======
    if not ((Byte(Value[I]) in NamingBitmap[Pages^[$100+hi(Word(Value[I]))]]) or
      (Value[I] = ':') or
      (Xml11 and IsXml11Char(Value, I))) then
>>>>>>> origin/fixes_2_2
    begin
      if (I = Length(Value)) or (Value[I] <> #32) then
        Exit;
    end;
    Inc(I);
  end;
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsValidXmlEncoding(const Value: XMLString): Boolean;
=======
function IsValidXmlEncoding(const Value: WideString): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function IsValidXmlEncoding(const Value: WideString): Boolean;
>>>>>>> origin/fixes_2_2
var
  I: Integer;
begin
  Result := False;
  if (Value = '') or (Value[1] > #255) or not (char(ord(Value[1])) in ['A'..'Z', 'a'..'z']) then
    Exit;
  for I := 2 to Length(Value) do
    if (Value[I] > #255) or not (char(ord(Value[I])) in ['A'..'Z', 'a'..'z', '0'..'9', '.', '_', '-']) then
      Exit;
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure NormalizeSpaces(var Value: XMLString);
=======
procedure NormalizeSpaces(var Value: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure NormalizeSpaces(var Value: WideString);
>>>>>>> origin/fixes_2_2
var
  I, J: Integer;
begin
  I := Length(Value);
  // speed: trim only whed needed
  if (I > 0) and ((Value[1] = #32) or (Value[I] = #32)) then
    Value := Trim(Value);
  I := 1;
  while I < Length(Value) do
  begin
    if Value[I] = #32 then
    begin
      J := I+1;
      while (J <= Length(Value)) and (Value[J] = #32) do Inc(J);
      if J-I > 1 then Delete(Value, I+1, J-I-1);
    end;
    Inc(I);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function IsXmlWhiteSpace(c: WideChar): Boolean;
begin
  Result := (c = #32) or (c = #9) or (c = #10) or (c = #13);
end;

function WStrLIComp(S1, S2: PWideChar; Len: Integer): Integer;
var
  counter: Integer;
  c1, c2: Word;
begin
  counter := 0;
  result := 0;
  if Len = 0 then
    exit;
  repeat
    c1 := ord(S1[counter]);
    c2 := ord(S2[counter]);
    if (c1 = 0) or (c2 = 0) then break;
    if c1 <> c2 then
    begin
      if c1 in [97..122] then
        Dec(c1, 32);
      if c2 in [97..122] then
        Dec(c2, 32);
      if c1 <> c2 then
        Break;
    end;
    Inc(counter);
  until counter >= Len;
  result := c1 - c2;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure WStrLower(var S: XMLString);
=======
procedure WStrLower(var S: WideString);
>>>>>>> graemeg/cpstrnew
=======
procedure WStrLower(var S: WideString);
>>>>>>> graemeg/cpstrnew
=======
procedure WStrLower(var S: WideString);
>>>>>>> graemeg/cpstrnew
=======
procedure WStrLower(var S: WideString);
>>>>>>> origin/cpstrnew
var
  i: Integer;
begin
  for i := 1 to Length(S) do
    if (S[i] >= 'A') and (S[i] <= 'Z') then
      Inc(word(S[i]), 32);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function Hash(InitValue: LongWord; Key: PWideChar; KeyLen: Integer): LongWord;
begin
  Result := InitValue;
  while KeyLen <> 0 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$push}{$r-}{$q-}
    Result := Result * $F4243 xor ord(Key^);
{$pop}
=======
{$ifdef overflow_check}{$q-}{$endif}
    Result := Result * $F4243 xor ord(Key^);
{$ifdef overflow_check}{$q+}{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef overflow_check}{$q-}{$endif}
    Result := Result * $F4243 xor ord(Key^);
{$ifdef overflow_check}{$q+}{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef overflow_check}{$q-}{$endif}
    Result := Result * $F4243 xor ord(Key^);
{$ifdef overflow_check}{$q+}{$endif}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef overflow_check}{$q-}{$endif}
    Result := Result * $F4243 xor ord(Key^);
{$ifdef overflow_check}{$q+}{$endif}
>>>>>>> origin/cpstrnew
=======
    Result := Result * $F4243 xor ord(Key^);
>>>>>>> graemeg/fixes_2_2
=======
    Result := Result * $F4243 xor ord(Key^);
>>>>>>> origin/fixes_2_2
    Inc(Key);
    Dec(KeyLen);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function KeyCompare(const Key1: XMLString; Key2: Pointer; Key2Len: Integer): Boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result := (Length(Key1)=Key2Len) and CompareMem(Pointer(Key1), Key2, Key2Len*2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$IFDEF FPC}
  Result := (Length(Key1)=Key2Len) and (CompareWord(Pointer(Key1)^, Key2^, Key2Len) = 0);
{$ELSE}
  Result := (Length(Key1)=Key2Len) and CompareMem(Pointer(Key1), Key2, Key2Len*2);
{$ENDIF}
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
=======
function KeyCompare(const Key1: WideString; Key2: Pointer; Key2Len: Integer): Boolean;
begin
  Result := (Length(Key1)=Key2Len) and (CompareWord(Pointer(Key1)^, Key2^, Key2Len) = 0);
>>>>>>> graemeg/fixes_2_2
=======
function KeyCompare(const Key1: WideString; Key2: Pointer; Key2Len: Integer): Boolean;
begin
{$IFDEF FPC}
  Result := (Length(Key1)=Key2Len) and (CompareWord(Pointer(Key1)^, Key2^, Key2Len) = 0);
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$ELSE}
  Result := (Length(Key1)=Key2Len) and CompareMem(Pointer(Key1), Key2, Key2Len*2);
{$ENDIF}
>>>>>>> origin/fixes_2.4
end;

{ THashTable }

constructor THashTable.Create(InitSize: Integer; OwnObjects: Boolean);
var
  I: Integer;
begin
  inherited Create;
  FOwnsObjects := OwnObjects;
  I := 256;
  while I < InitSize do I := I shl 1;
  FBucketCount := I;
  FBucket := AllocMem(I * sizeof(PHashItem));
end;

destructor THashTable.Destroy;
begin
  Clear;
  FreeMem(FBucket);
  inherited Destroy;
end;

procedure THashTable.Clear;
var
  I: Integer;
  item, next: PHashItem;
begin
  for I := 0 to FBucketCount-1 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    item := FBucket^[I];
=======
    item := FBucket[I];
>>>>>>> graemeg/fixes_2_2
=======
    item := FBucket[I];
>>>>>>> origin/fixes_2_2
=======
    item := FBucket^[I];
>>>>>>> origin/fixes_2.4
    while Assigned(item) do
    begin
      next := item^.Next;
      if FOwnsObjects then
        item^.Data.Free;
      Dispose(item);
      item := next;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FBucket^[I] := nil;
  end;
=======
  end;
  FillChar(FBucket^, FBucketCount * sizeof(PHashItem), 0);
>>>>>>> graemeg/fixes_2_2
=======
  end;
  FillChar(FBucket^, FBucketCount * sizeof(PHashItem), 0);
>>>>>>> origin/fixes_2_2
=======
    FBucket^[I] := nil;
  end;
>>>>>>> origin/fixes_2.4
end;

function THashTable.Find(Key: PWideChar; KeyLen: Integer): PHashItem;
var
  Dummy: Boolean;
begin
  Result := Lookup(Key, KeyLen, Dummy, False);
end;

function THashTable.FindOrAdd(Key: PWideChar; KeyLen: Integer;
  var Found: Boolean): PHashItem;
begin
  Result := Lookup(Key, KeyLen, Found, True);
end;

function THashTable.FindOrAdd(Key: PWideChar; KeyLen: Integer): PHashItem;
var
  Dummy: Boolean;
begin
  Result := Lookup(Key, KeyLen, Dummy, True);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function THashTable.FindOrAdd(const Key: XMLString): PHashItem;
var
  Dummy: Boolean;
begin
  Result := Lookup(PWideChar(Key), Length(Key), Dummy, True);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function THashTable.Get(Key: PWideChar; KeyLen: Integer): TObject;
var
  e: PHashItem;
  Dummy: Boolean;
begin
  e := Lookup(Key, KeyLen, Dummy, False);
  if Assigned(e) then
    Result := e^.Data
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Result := nil;
end;

function THashTable.Lookup(Key: PWideChar; KeyLength: Integer;
  out Found: Boolean; CanCreate: Boolean): PHashItem;
=======
=======
>>>>>>> origin/fixes_2_2
    Result := nil;  
end;

function THashTable.Lookup(Key: PWideChar; KeyLength: Integer;
<<<<<<< HEAD
  var Found: Boolean; CanCreate: Boolean): PHashItem;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  out Found: Boolean; CanCreate: Boolean): PHashItem;
>>>>>>> origin/fixes_2.4
var
  Entry: PPHashItem;
  h: LongWord;
begin
  h := Hash(0, Key, KeyLength);
<<<<<<< HEAD
<<<<<<< HEAD
  Entry := @FBucket^[h mod FBucketCount];
=======
  Entry := @FBucket[h mod FBucketCount];
>>>>>>> graemeg/fixes_2_2
=======
  Entry := @FBucket[h mod FBucketCount];
>>>>>>> origin/fixes_2_2
  while Assigned(Entry^) and not ((Entry^^.HashValue = h) and KeyCompare(Entry^^.Key, Key, KeyLength) ) do
    Entry := @Entry^^.Next;
  Found := Assigned(Entry^);
  if Found or (not CanCreate) then
  begin
    Result := Entry^;
    Exit;
  end;
  if FCount > FBucketCount then  { arbitrary limit, probably too high }
  begin
    Resize(FBucketCount * 2);
    Result := Lookup(Key, KeyLength, Found, CanCreate);
  end
  else
  begin
    New(Result);
<<<<<<< HEAD
<<<<<<< HEAD
    // SetString for WideStrings trims on zero chars [fixed, #14740]
    SetLength(Result^.Key, KeyLength);
    Move(Key^, Pointer(Result^.Key)^, KeyLength*sizeof(WideChar));
=======
    SetString(Result^.Key, Key, KeyLength);
>>>>>>> graemeg/fixes_2_2
=======
    SetString(Result^.Key, Key, KeyLength);
>>>>>>> origin/fixes_2_2
    Result^.HashValue := h;
    Result^.Data := nil;
    Result^.Next := nil;
    Inc(FCount);
    Entry^ := Result;
  end;
end;

procedure THashTable.Resize(NewCapacity: LongWord);
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  p: PHashItemArray;
  chain: PPHashItem;
=======
  p, chain: PPHashItem;
>>>>>>> graemeg/fixes_2_2
=======
  p, chain: PPHashItem;
>>>>>>> origin/fixes_2_2
=======
  p: PHashItemArray;
  chain: PPHashItem;
>>>>>>> origin/fixes_2.4
  i: Integer;
  e, n: PHashItem;
begin
  p := AllocMem(NewCapacity * sizeof(PHashItem));
  for i := 0 to FBucketCount-1 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    e := FBucket^[i];
    while Assigned(e) do
    begin
      chain := @p^[e^.HashValue mod NewCapacity];
=======
=======
>>>>>>> origin/fixes_2_2
    e := FBucket[i];
=======
    e := FBucket^[i];
>>>>>>> origin/fixes_2.4
    while Assigned(e) do
    begin
      chain := @p[e^.HashValue mod NewCapacity];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      n := e^.Next;
      e^.Next := chain^;
      chain^ := e;
      e := n;
    end;
  end;
  FBucketCount := NewCapacity;
  FreeMem(FBucket);
  FBucket := p;
end;

function THashTable.Remove(Entry: PHashItem): Boolean;
var
  chain: PPHashItem;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  chain := @FBucket^[Entry^.HashValue mod FBucketCount];
=======
  chain := @FBucket[Entry^.HashValue mod FBucketCount];
>>>>>>> graemeg/fixes_2_2
=======
  chain := @FBucket[Entry^.HashValue mod FBucketCount];
>>>>>>> origin/fixes_2_2
  while Assigned(chain^) do
  begin
    if chain^ = Entry then
    begin
      chain^ := Entry^.Next;
      if FOwnsObjects then
        Entry^.Data.Free;
      Dispose(Entry);
      Dec(FCount);
      Result := True;
      Exit;
    end;
    chain := @chain^^.Next;
  end;
  Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
// this does not free the aData object
function THashTable.RemoveData(aData: TObject): Boolean;
var
  i: Integer;
  chain: PPHashItem;
  e: PHashItem;
begin
  for i := 0 to FBucketCount-1 do
  begin
    chain := @FBucket^[i];
    while Assigned(chain^) do
    begin
      if chain^^.Data = aData then
      begin
        e := chain^;
        chain^ := e^.Next;
        Dispose(e);
        Dec(FCount);
        Result := True;
        Exit;
      end;
      chain := @chain^^.Next;
    end;
  end;
  Result := False;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure THashTable.ForEach(proc: THashForEach; arg: Pointer);
var
  i: Integer;
  e: PHashItem;
begin
  for i := 0 to FBucketCount-1 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    e := FBucket^[i];
=======
    e := FBucket[i];
>>>>>>> graemeg/fixes_2_2
=======
    e := FBucket[i];
>>>>>>> origin/fixes_2_2
=======
    e := FBucket^[i];
>>>>>>> origin/fixes_2.4
    while Assigned(e) do
    begin
      if not proc(e, arg) then
        Exit;
      e := e^.Next;
    end;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
{ TDblHashArray }

destructor TDblHashArray.Destroy;
begin
  FreeMem(FData);
  inherited Destroy;
end;

procedure TDblHashArray.Init(NumSlots: Integer);
var
  i: Integer;
begin
  if ((NumSlots * 2) shr FSizeLog) <> 0 then   // need at least twice more entries, and no less than 8
  begin
    FSizeLog := 3;
    while (NumSlots shr FSizeLog) <> 0 do
      Inc(FSizeLog);
    ReallocMem(FData, (1 shl FSizeLog) * sizeof(TExpHashEntry));
    FRevision := 0;
  end;
  if FRevision = 0 then
  begin
    FRevision := $FFFFFFFF;
    for i := (1 shl FSizeLog)-1 downto 0 do
      FData^[i].rev := FRevision;
  end;
  Dec(FRevision);
end;

function TDblHashArray.Locate(uri: Pointer; localName: PWideChar; localLength: Integer): Boolean;
var
  step: Byte;
  mask: LongWord;
  idx: Integer;
  HashValue: LongWord;
begin
  HashValue := Hash(LongWord(PtrUInt(uri)), localName, localLength);

  mask := (1 shl FSizeLog) - 1;
  step := (HashValue and (not mask)) shr (FSizeLog-1) and (mask shr 2) or 1;
  idx := HashValue and mask;
  result := True;
  while FData^[idx].rev = FRevision do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if (HashValue = FData^[idx].hash) and (FData^[idx].uriPtr = uri) and
=======
    if (HashValue = FData^[idx].hash) and (FData^[idx].uriPtr^ = uri^) and
>>>>>>> graemeg/cpstrnew
=======
    if (HashValue = FData^[idx].hash) and (FData^[idx].uriPtr^ = uri^) and
>>>>>>> graemeg/cpstrnew
=======
    if (HashValue = FData^[idx].hash) and (FData^[idx].uriPtr^ = uri^) and
>>>>>>> graemeg/cpstrnew
=======
    if (HashValue = FData^[idx].hash) and (FData^[idx].uriPtr^ = uri^) and
>>>>>>> origin/cpstrnew
=======
    if (HashValue = FData^[idx].hash) and (FData^[idx].uriPtr^ = uri^) and
>>>>>>> origin/fixes_2.4
      (FData^[idx].lnameLen = localLength) and
       CompareMem(FData^[idx].lname, localName, localLength * sizeof(WideChar)) then
      Exit;
    if idx < step then
      Inc(idx, (1 shl FSizeLog) - step)
    else
      Dec(idx, step);
  end;
  with FData^[idx] do
  begin
    rev := FRevision;
    hash := HashValue;
    uriPtr := uri;
    lname := localName;
    lnameLen := localLength;
  end;
  result := False;
end;

{ TNSSupport }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TNSSupport.Create(aNameTable: THashTable);
=======
constructor TNSSupport.Create;
>>>>>>> graemeg/cpstrnew
=======
constructor TNSSupport.Create;
>>>>>>> graemeg/cpstrnew
=======
constructor TNSSupport.Create;
>>>>>>> graemeg/cpstrnew
=======
constructor TNSSupport.Create;
>>>>>>> origin/cpstrnew
var
  b: TBinding;
begin
  inherited Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FNameTable := aNameTable;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FPrefixes := THashTable.Create(16, False);
  FBindings := TFPList.Create;
  SetLength(FBindingStack, 16);

  { provide implicit binding for the 'xml' prefix }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DefineBinding('xml', stduri_xml, b);
  { bind default namespace to empty string }
  DefineBinding('', '', b);
=======
  // TODO: move stduri_xml, etc. to this unit, so they are reused.
  DefineBinding('xml', 'http://www.w3.org/XML/1998/namespace', b);
>>>>>>> graemeg/cpstrnew
=======
  // TODO: move stduri_xml, etc. to this unit, so they are reused.
  DefineBinding('xml', 'http://www.w3.org/XML/1998/namespace', b);
>>>>>>> graemeg/cpstrnew
=======
  // TODO: move stduri_xml, etc. to this unit, so they are reused.
  DefineBinding('xml', 'http://www.w3.org/XML/1998/namespace', b);
>>>>>>> graemeg/cpstrnew
=======
  // TODO: move stduri_xml, etc. to this unit, so they are reused.
  DefineBinding('xml', 'http://www.w3.org/XML/1998/namespace', b);
>>>>>>> origin/cpstrnew
end;

destructor TNSSupport.Destroy;
var
  I: Integer;
begin
  for I := FBindings.Count-1 downto 0 do
    TObject(FBindings.List^[I]).Free;
  FBindings.Free;
  FPrefixes.Free;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TNSSupport.BindPrefix(nsURI, aPrefix: PHashItem): TBinding;
=======
function TNSSupport.BindPrefix(const nsURI: WideString; aPrefix: PHashItem): TBinding;
>>>>>>> graemeg/cpstrnew
=======
function TNSSupport.BindPrefix(const nsURI: WideString; aPrefix: PHashItem): TBinding;
>>>>>>> graemeg/cpstrnew
=======
function TNSSupport.BindPrefix(const nsURI: WideString; aPrefix: PHashItem): TBinding;
>>>>>>> graemeg/cpstrnew
=======
function TNSSupport.BindPrefix(const nsURI: WideString; aPrefix: PHashItem): TBinding;
>>>>>>> origin/cpstrnew
begin
  { try to reuse an existing binding }
  result := FFreeBindings;
  if Assigned(result) then
    FFreeBindings := result.Next
  else { no free bindings, create a new one }
  begin
    result := TBinding.Create;
    FBindings.Add(result);
  end;
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

  { link it into chain of bindings at the current element level }
  result.Next := FBindingStack[FNesting];
  FBindingStack[FNesting] := result;

  { bind }
  result.uri := nsURI;
  result.Prefix := aPrefix;
  result.PrevPrefixBinding := aPrefix^.Data;
  aPrefix^.Data := result;
end;

function TNSSupport.DefaultNSBinding: TBinding;
begin
  result := TBinding(FDefaultPrefix.Data);
end;

procedure TNSSupport.DefineBinding(const Prefix, nsURI: WideString;
  out Binding: TBinding);
var
  Pfx: PHashItem;
begin
  Pfx := @FDefaultPrefix;
  if (nsURI <> '') and (Prefix <> '') then
    Pfx := FPrefixes.FindOrAdd(PWideChar(Prefix), Length(Prefix));
  if (Pfx^.Data = nil) or (TBinding(Pfx^.Data).uri <> nsURI) then
    Binding := BindPrefix(nsURI, Pfx)
  else
    Binding := nil;
end;

function TNSSupport.CheckAttribute(const Prefix, nsURI: WideString;
  out Binding: TBinding): TAttributeAction;
var
  Pfx: PHashItem;
  I: Integer;
  b: TBinding;
  buf: array[0..31] of WideChar;
  p: PWideChar;
begin
  Binding := nil;
  Pfx := nil;
  Result := aaUnchanged;
  if Prefix <> '' then
    Pfx := FPrefixes.FindOrAdd(PWideChar(Prefix), Length(Prefix))
  else if nsURI = '' then
    Exit;
  { if the prefix is already bound to correct URI, we're done }
  if Assigned(Pfx) and Assigned(Pfx^.Data) and (TBinding(Pfx^.Data).uri = nsURI) then
    Exit;

  { see if there's another prefix bound to the target URI }
  // TODO: should use something faster than linear search
  for i := FNesting downto 0 do
  begin
    b := FBindingStack[i];
    while Assigned(b) do
    begin
      if (b.uri = nsURI) and (b.Prefix <> @FDefaultPrefix) then
      begin
        Binding := b;   // found one -> override the attribute's prefix
        Result := aaPrefix;
        Exit;
      end;
      b := b.Next;
    end;
  end;
  { no prefix, or bound (to wrong URI) -> use generated prefix instead }
  if (Pfx = nil) or Assigned(Pfx^.Data) then
  repeat
    Inc(FPrefixSeqNo);
    i := FPrefixSeqNo;    // This is just 'NS'+IntToStr(FPrefixSeqNo);
    p := @Buf[high(Buf)]; // done without using strings
    while i <> 0 do
    begin
      p^ := WideChar(i mod 10+ord('0'));
      dec(p);
      i := i div 10;
    end;
    p^ := 'S'; dec(p);
    p^ := 'N';
    Pfx := FPrefixes.FindOrAdd(p, @Buf[high(Buf)]-p+1);
  until Pfx^.Data = nil;
  Binding := BindPrefix(nsURI, Pfx);
  Result := aaBoth;
end;

function TNSSupport.IsPrefixBound(P: PWideChar; Len: Integer; out
  Prefix: PHashItem): Boolean;
begin
  Prefix := FPrefixes.FindOrAdd(P, Len);
  Result := Assigned(Prefix^.Data) and (TBinding(Prefix^.Data).uri <> '');
end;

function TNSSupport.GetPrefix(P: PWideChar; Len: Integer): PHashItem;
begin
  if Assigned(P) and (Len > 0) then
    Result := FPrefixes.FindOrAdd(P, Len)
  else
    Result := @FDefaultPrefix;
end;

procedure TNSSupport.StartElement;
begin
  Inc(FNesting);
  if FNesting >= Length(FBindingStack) then
    SetLength(FBindingStack, FNesting * 2);
end;

procedure TNSSupport.EndElement;
var
  b, temp: TBinding;
begin
  temp := FBindingStack[FNesting];
  while Assigned(temp) do
  begin
    b := temp;
    temp := b.next;
    b.next := FFreeBindings;
    FFreeBindings := b;
    b.Prefix^.Data := b.prevPrefixBinding;
  end;
  FBindingStack[FNesting] := nil;
  if FNesting > 0 then
    Dec(FNesting);
end;

{ Buffer builder utils }

procedure BufAllocate(var ABuffer: TWideCharBuf; ALength: Integer);
begin
  ABuffer.MaxLength := ALength;
  ABuffer.Length := 0;
  ABuffer.Buffer := AllocMem(ABuffer.MaxLength*SizeOf(WideChar));
end;

procedure BufAppend(var ABuffer: TWideCharBuf; wc: WideChar);
begin
  if ABuffer.Length >= ABuffer.MaxLength then
  begin
    ReallocMem(ABuffer.Buffer, ABuffer.MaxLength * 2 * SizeOf(WideChar));
    FillChar(ABuffer.Buffer[ABuffer.MaxLength], ABuffer.MaxLength * SizeOf(WideChar),0);
    ABuffer.MaxLength := ABuffer.MaxLength * 2;
  end;
  ABuffer.Buffer[ABuffer.Length] := wc;
  Inc(ABuffer.Length);
end;

procedure BufAppendChunk(var ABuf: TWideCharBuf; pstart, pend: PWideChar);
var
  Len: Integer;
begin
  Len := PEnd - PStart;
  if Len <= 0 then
    Exit;
  if Len >= ABuf.MaxLength - ABuf.Length then
  begin
    ABuf.MaxLength := (Len + ABuf.Length)*2;
    // note: memory clean isn't necessary here.
    // To avoid garbage, control Length field.
    ReallocMem(ABuf.Buffer, ABuf.MaxLength * sizeof(WideChar));
  end;
  Move(pstart^, ABuf.Buffer[ABuf.Length], Len * sizeof(WideChar));
  Inc(ABuf.Length, Len);
end;

function BufEquals(const ABuf: TWideCharBuf; const Arg: WideString): Boolean;
begin
  Result := (ABuf.Length = Length(Arg)) and
    CompareMem(ABuf.Buffer, Pointer(Arg), ABuf.Length*sizeof(WideChar));
end;

procedure BufNormalize(var Buf: TWideCharBuf; out Modified: Boolean);
var
  Dst, Src: Integer;
begin
  Dst := 0;
  Src := 0;
  // skip leading space if any
  while (Src < Buf.Length) and (Buf.Buffer[Src] = ' ') do
    Inc(Src);

  while Src < Buf.Length do
  begin
    if Buf.Buffer[Src] = ' ' then
    begin
      // Dst cannot be 0 here, because leading space is already skipped
      if Buf.Buffer[Dst-1] <> ' ' then
      begin
        Buf.Buffer[Dst] := ' ';
        Inc(Dst);
      end;
    end
    else
    begin
      Buf.Buffer[Dst] := Buf.Buffer[Src];
      Inc(Dst);
    end;
    Inc(Src);
  end;
  // trailing space (only one possible due to compression)
  if (Dst > 0) and (Buf.Buffer[Dst-1] = ' ') then
    Dec(Dst);

  Modified := Dst <> Buf.Length;
  Buf.Length := Dst;
end;

{ standard decoders }

function Decode_UCS2(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
var
  cnt: Cardinal;
begin
  cnt := OutCnt;         // num of widechars
  if cnt > InCnt div sizeof(WideChar) then
    cnt := InCnt div sizeof(WideChar);
  Move(InBuf^, OutBuf^, cnt * sizeof(WideChar));
  Dec(InCnt, cnt*sizeof(WideChar));
  Dec(OutCnt, cnt);
  Result := cnt;
end;

function Decode_UCS2_Swapped(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
var
  I: Integer;
  cnt: Cardinal;
  InPtr: PChar;
begin
  cnt := OutCnt;         // num of widechars
  if cnt > InCnt div sizeof(WideChar) then
    cnt := InCnt div sizeof(WideChar);
  InPtr := InBuf;
  for I := 0 to cnt-1 do
  begin
    OutBuf[I] := WideChar((ord(InPtr^) shl 8) or ord(InPtr[1]));
    Inc(InPtr, 2);
  end;
  Dec(InCnt, cnt*sizeof(WideChar));
  Dec(OutCnt, cnt);
  Result := cnt;
end;

function Decode_8859_1(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
var
  I: Integer;
  cnt: Cardinal;
begin
  cnt := OutCnt;         // num of widechars
  if cnt > InCnt then
    cnt := InCnt;
  for I := 0 to cnt-1 do
    OutBuf[I] := WideChar(ord(InBuf[I]));
  Dec(InCnt, cnt);
  Dec(OutCnt, cnt);
  Result := cnt;
end;

function Decode_UTF8(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
const
  MaxCode: array[1..4] of Cardinal = ($7F, $7FF, $FFFF, $1FFFFF);
var
  i, j, bc: Cardinal;
  Value: Cardinal;
begin
  result := 0;
  i := OutCnt;
  while (i > 0) and (InCnt > 0) do
  begin
    bc := 1;
    Value := ord(InBuf^);
    if Value < $80 then
      OutBuf^ := WideChar(Value)
    else
    begin
      if Value < $C2 then
      begin
        Result := -1;
        Break;
      end;
      Inc(bc);
      if Value > $DF then
      begin
        Inc(bc);
        if Value > $EF then
        begin
          Inc(bc);
          if Value > $F7 then  // never encountered in the tests.
          begin
            Result := -1;
            Break;
          end;
        end;
      end;
      if InCnt < bc then
        Break;
      j := 1;
      while j < bc do
      begin
        if InBuf[j] in [#$80..#$BF] then
          Value := (Value shl 6) or (Cardinal(InBuf[j]) and $3F)
        else
        begin
          Result := -1;
          Break;
        end;
        Inc(j);
      end;
      Value := Value and MaxCode[bc];
      // RFC2279 check
      if Value <= MaxCode[bc-1] then
      begin
        Result := -1;
        Break;
      end;
      case Value of
        0..$D7FF, $E000..$FFFF: OutBuf^ := WideChar(Value);
        $10000..$10FFFF:
        begin
          if i < 2 then Break;
          OutBuf^ := WideChar($D7C0 + (Value shr 10));
          OutBuf[1] := WideChar($DC00 xor (Value and $3FF));
          Inc(OutBuf); // once here
          Dec(i);
        end
        else
        begin
          Result := -1;
          Break;
        end;
      end;
    end;
    Inc(OutBuf);
    Inc(InBuf, bc);
    Dec(InCnt, bc);
    Dec(i);
  end;
  if Result >= 0 then
    Result := OutCnt-i;
  OutCnt := i;
end;


initialization
>>>>>>> graemeg/cpstrnew

  { link it into chain of bindings at the current element level }
  result.Next := FBindingStack[FNesting];
  FBindingStack[FNesting] := result;

  { bind }
  result.uri := nsURI;
  result.Prefix := aPrefix;
  result.PrevPrefixBinding := aPrefix^.Data;
  aPrefix^.Data := result;
end;

function TNSSupport.DefaultNSBinding: TBinding;
begin
  result := TBinding(FDefaultPrefix.Data);
end;

procedure TNSSupport.DefineBinding(const Prefix, nsURI: XMLString;
  out Binding: TBinding);
var
  Pfx, uri: PHashItem;
begin
  Pfx := @FDefaultPrefix;
  if (nsURI <> '') and (Prefix <> '') then
    Pfx := FPrefixes.FindOrAdd(PWideChar(Prefix), Length(Prefix));
  uri := FNameTable.FindOrAdd(PWideChar(nsURI),Length(nsURI));
  if (Pfx^.Data = nil) or (TBinding(Pfx^.Data).uri <> uri) then
    Binding := BindPrefix(uri, Pfx)
  else
    Binding := nil;
end;

function TNSSupport.CheckAttribute(const Prefix, nsURI: XMLString;
  out Binding: TBinding): TAttributeAction;
var
  Pfx: PHashItem;
  I: Integer;
  b: TBinding;
  buf: array[0..31] of WideChar;
  p: PWideChar;
  uri: PHashItem;
begin
  Binding := nil;
  Pfx := nil;
  Result := aaUnchanged;
  if Prefix <> '' then
    Pfx := FPrefixes.FindOrAdd(PWideChar(Prefix), Length(Prefix))
  else if nsURI = '' then
    Exit;
  uri := FNameTable.FindOrAdd(PWideChar(nsURI), Length(nsURI));
  { if the prefix is already bound to correct URI, we're done }
  if Assigned(Pfx) and Assigned(Pfx^.Data) and (TBinding(Pfx^.Data).uri = uri) then
    Exit;

  { see if there's another prefix bound to the target URI }
  // TODO: should use something faster than linear search
  for i := FNesting downto 0 do
  begin
    b := FBindingStack[i];
    while Assigned(b) do
    begin
      if (b.uri = uri) and (b.Prefix <> @FDefaultPrefix) then
      begin
        Binding := b;   // found one -> override the attribute's prefix
        Result := aaPrefix;
        Exit;
      end;
      b := b.Next;
    end;
  end;
  { no prefix, or bound (to wrong URI) -> use generated prefix instead }
  if (Pfx = nil) or Assigned(Pfx^.Data) then
  repeat
    Inc(FPrefixSeqNo);
    i := FPrefixSeqNo;    // This is just 'NS'+IntToStr(FPrefixSeqNo);
    p := @Buf[high(Buf)]; // done without using strings
    while i <> 0 do
    begin
      p^ := WideChar(i mod 10+ord('0'));
      dec(p);
      i := i div 10;
    end;
    p^ := 'S'; dec(p);
    p^ := 'N';
    Pfx := FPrefixes.FindOrAdd(p, @Buf[high(Buf)]-p+1);
  until Pfx^.Data = nil;
  Binding := BindPrefix(uri, Pfx);
  Result := aaBoth;
end;

function TNSSupport.GetPrefix(P: PWideChar; Len: Integer): PHashItem;
begin
  if Assigned(P) and (Len > 0) then
    Result := FPrefixes.FindOrAdd(P, Len)
  else
    Result := @FDefaultPrefix;
end;

function TNSSupport.LookupNamespace(const APrefix: XMLString): XMLString;
var
  prefixatom: PHashItem;
  b: TBinding;
begin
  prefixatom := GetPrefix(PWideChar(APrefix),Length(APrefix));
  b := TBinding(prefixatom^.Data);
  if Assigned(b) and Assigned(b.Uri) then
    result := b.Uri^.Key
  else
    result := '';
end;

procedure TNSSupport.PushScope;
begin
  Inc(FNesting);
  if FNesting >= Length(FBindingStack) then
    SetLength(FBindingStack, FNesting * 2);
end;

function TNSSupport.PopScope: Boolean;
var
  b, temp: TBinding;
begin
  { don't unbind prefixes declared before the first call to PushScope }
  Result := FNesting > 0;
  if not Result then
    Exit;
  temp := FBindingStack[FNesting];
  while Assigned(temp) do
  begin
    b := temp;
    temp := b.next;
    b.next := FFreeBindings;
    FFreeBindings := b;
    b.Prefix^.Data := b.prevPrefixBinding;
  end;
  FBindingStack[FNesting] := nil;
  Dec(FNesting);
end;

{ Buffer builder utils }

procedure BufAllocate(var ABuffer: TWideCharBuf; ALength: Integer);
begin
  ABuffer.MaxLength := ALength;
  ABuffer.Length := 0;
  ABuffer.Buffer := AllocMem(ABuffer.MaxLength*SizeOf(WideChar));
end;

procedure BufAppend(var ABuffer: TWideCharBuf; wc: WideChar);
begin
  if ABuffer.Length >= ABuffer.MaxLength then
  begin
    ReallocMem(ABuffer.Buffer, ABuffer.MaxLength * 2 * SizeOf(WideChar));
    FillChar(ABuffer.Buffer[ABuffer.MaxLength], ABuffer.MaxLength * SizeOf(WideChar),0);
    ABuffer.MaxLength := ABuffer.MaxLength * 2;
  end;
  ABuffer.Buffer[ABuffer.Length] := wc;
  Inc(ABuffer.Length);
end;

procedure BufAppendChunk(var ABuf: TWideCharBuf; pstart, pend: PWideChar);
var
  Len: Integer;
begin
  Len := PEnd - PStart;
  if Len <= 0 then
    Exit;
  if Len >= ABuf.MaxLength - ABuf.Length then
  begin
    ABuf.MaxLength := (Len + ABuf.Length)*2;
    // note: memory clean isn't necessary here.
    // To avoid garbage, control Length field.
    ReallocMem(ABuf.Buffer, ABuf.MaxLength * sizeof(WideChar));
  end;
  Move(pstart^, ABuf.Buffer[ABuf.Length], Len * sizeof(WideChar));
  Inc(ABuf.Length, Len);
end;

procedure BufAppendString(var ABuf: TWideCharBuf; const AValue: XMLString);
var
  Len: Integer;
begin
  Len := Length(AValue);
  if Len <= 0 then
    Exit;
  if Len >= ABuf.MaxLength - ABuf.Length then
  begin
    ABuf.MaxLength := (Len + ABuf.Length)*2;
    ReallocMem(ABuf.Buffer, ABuf.MaxLength * sizeof(WideChar));
  end;
  Move(PWideChar(AValue)^, ABuf.Buffer[ABuf.Length], Len * sizeof(WideChar));
  Inc(ABuf.Length, Len);
end;

function BufEquals(const ABuf: TWideCharBuf; const Arg: XMLString): Boolean;
begin
  Result := (ABuf.Length = Length(Arg)) and
    CompareMem(ABuf.Buffer, Pointer(Arg), ABuf.Length*sizeof(WideChar));
end;

procedure BufNormalize(var Buf: TWideCharBuf; out Modified: Boolean);
var
  Dst, Src: Integer;
begin
  Dst := 0;
  Src := 0;
  // skip leading space if any
  while (Src < Buf.Length) and (Buf.Buffer[Src] = ' ') do
    Inc(Src);

  while Src < Buf.Length do
  begin
    if Buf.Buffer[Src] = ' ' then
    begin
      // Dst cannot be 0 here, because leading space is already skipped
      if Buf.Buffer[Dst-1] <> ' ' then
      begin
        Buf.Buffer[Dst] := ' ';
        Inc(Dst);
      end;
    end
    else
    begin
      Buf.Buffer[Dst] := Buf.Buffer[Src];
      Inc(Dst);
    end;
    Inc(Src);
  end;
  // trailing space (only one possible due to compression)
  if (Dst > 0) and (Buf.Buffer[Dst-1] = ' ') then
    Dec(Dst);

  Modified := Dst <> Buf.Length;
  Buf.Length := Dst;
end;

{ standard decoders }

function Decode_UCS2(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
var
  cnt: Cardinal;
begin
  cnt := OutCnt;         // num of widechars
  if cnt > InCnt div sizeof(WideChar) then
    cnt := InCnt div sizeof(WideChar);
  Move(InBuf^, OutBuf^, cnt * sizeof(WideChar));
  Dec(InCnt, cnt*sizeof(WideChar));
  Dec(OutCnt, cnt);
  Result := cnt;
end;

function Decode_UCS2_Swapped(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
var
  I: Integer;
  cnt: Cardinal;
  InPtr: PChar;
begin
  cnt := OutCnt;         // num of widechars
  if cnt > InCnt div sizeof(WideChar) then
    cnt := InCnt div sizeof(WideChar);
  InPtr := InBuf;
  for I := 0 to cnt-1 do
  begin
    OutBuf[I] := WideChar((ord(InPtr^) shl 8) or ord(InPtr[1]));
    Inc(InPtr, 2);
  end;
  Dec(InCnt, cnt*sizeof(WideChar));
  Dec(OutCnt, cnt);
  Result := cnt;
end;

function Decode_8859_1(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
var
  I: Integer;
  cnt: Cardinal;
begin
  cnt := OutCnt;         // num of widechars
  if cnt > InCnt then
    cnt := InCnt;
  for I := 0 to cnt-1 do
    OutBuf[I] := WideChar(ord(InBuf[I]));
  Dec(InCnt, cnt);
  Dec(OutCnt, cnt);
  Result := cnt;
end;

function Decode_UTF8(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
const
  MaxCode: array[1..4] of Cardinal = ($7F, $7FF, $FFFF, $1FFFFF);
var
  i, j, bc: Cardinal;
  Value: Cardinal;
begin
  result := 0;
  i := OutCnt;
  while (i > 0) and (InCnt > 0) do
  begin
    bc := 1;
    Value := ord(InBuf^);
    if Value < $80 then
      OutBuf^ := WideChar(Value)
    else
    begin
      if Value < $C2 then
      begin
        Result := -1;
        Break;
      end;
      Inc(bc);
      if Value > $DF then
      begin
        Inc(bc);
        if Value > $EF then
        begin
          Inc(bc);
          if Value > $F7 then  // never encountered in the tests.
          begin
            Result := -1;
            Break;
          end;
        end;
      end;
      if InCnt < bc then
        Break;
      j := 1;
      while j < bc do
      begin
        if InBuf[j] in [#$80..#$BF] then
          Value := (Value shl 6) or (Cardinal(InBuf[j]) and $3F)
        else
        begin
          Result := -1;
          Break;
        end;
        Inc(j);
      end;
      Value := Value and MaxCode[bc];
      // RFC2279 check
      if Value <= MaxCode[bc-1] then
      begin
        Result := -1;
        Break;
      end;
      case Value of
        0..$D7FF, $E000..$FFFF: OutBuf^ := WideChar(Value);
        $10000..$10FFFF:
        begin
          if i < 2 then Break;
          OutBuf^ := WideChar($D7C0 + (Value shr 10));
          OutBuf[1] := WideChar($DC00 xor (Value and $3FF));
          Inc(OutBuf); // once here
          Dec(i);
        end
        else
        begin
          Result := -1;
          Break;
        end;
      end;
    end;
    Inc(OutBuf);
    Inc(InBuf, bc);
    Dec(InCnt, bc);
    Dec(i);
  end;
  if Result >= 0 then
    Result := OutCnt-i;
  OutCnt := i;
end;
=======
=======
>>>>>>> origin/fixes_2_2
initialization

finalization
  if Assigned(Xml11Pg) then
    FreeMem(Xml11Pg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

end.
