{ Copyright (C) <2008> <Andrew Haines> chmfiftimain.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., i51 Franklin Street, Fifth Floor, Boston, MA 02111-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit chmFiftiMain;
{$mode objfpc}{$H+}
interface

uses Classes, HTMLIndexer;

type
  TFiftiMainHeader = record
    Sig: array [0..3] of byte;  //$00,$00,$28,$00
    HTMLFilesCount: DWord;
    RootNodeOffset: DWord;
    Unknown1: DWord; // = 0
    LeafNodeCount: DWord;
    CopyOfRootNodeOffset: DWord;
    TreeDepth: Word;
    Unknown2: DWord; // = 7
    DocIndexScale: Byte;
    DocIndexRootSize: Byte;
    CodeCountScale: Byte;
    CodeCountRootSize: Byte;
    LocationCodeScale: Byte;
    LocationCodeRootSize: Byte;
    Unknown3: array[0..9] of byte; // = 0
    NodeSize: DWord; // 4096;
    Unknown4: DWord; // 0 or 1;
    LastDupWordIndex: DWord;
    LastDupCharIndex: DWord;
    LongestWordLength: DWord; // maximum 99
    TotalWordsIndexed: DWord; // includes duplicates
    TotalWords: DWord; // word count not including duplicates
    TotalWordsLengthPart1: DWord; // length of all the words with duplicates plus the next dword!
    TotalWordsLengthPart2: DWord;
    TotalWordsLength: DWord; // length of all words not including duplicates
    WordBlockUnusedBytes: DWord; // who knows, this makes no sense when there are more than one blocks
    Unknown5: DWord; // 0
    HTMLFilesCountMinusOne: DWord; // maybe
    Unknown6: array[0..23] of Byte; // 0
    WindowsCodePage: DWord; // usually 1252
    LocalID: DWord;
    //Unknown7: array [0..893] of Byte; // 0
  end;

  { TFIftiNode }

  TFIftiNode = class(TObject)
    FLastWord: String;
    FWriteStream: TStream;
    FBlockStream: TMemoryStream;
    ParentNode: TFIftiNode;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    OwnsParentNode : boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    OwnsParentNode : boolean;
>>>>>>> origin/fixes_2.4
    function  AdjustedWord(AWord: String; out AOffset: Byte; AOldWord: String): String;
    procedure ChildIsFull(AWord: String; ANodeOffset: DWord); virtual; abstract;
    function  GuessIfCanHold(AWord: String): Boolean; virtual; abstract;
    procedure Flush(NewBlockNeeded: Boolean); virtual; abstract;
    procedure FillRemainingSpace;
    function  RemainingSpace: DWord;
    constructor Create(AStream: TStream);
    destructor Destroy; override;
  end;

  { TChmSearchWriter }

  TChmSearchWriter = class(TObject)
  private
    FHeaderRec: TFiftiMainHeader;
    FStream: TStream;
    FWordList: TIndexedWordList;
    FActiveLeafNode: TFIftiNode;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetHasData: Boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure ProcessWords;
    procedure WriteHeader(IsPlaceHolder: Boolean);
    procedure WriteAWord(AWord: TIndexedWord);
  public
    procedure WriteToStream;
<<<<<<< HEAD
<<<<<<< HEAD
    property  HasData: Boolean read GetHasData;
    constructor Create(AStream: TStream; AWordList: TIndexedWordList);
    destructor Destroy; override;
=======
    constructor Create(AStream: TStream; AWordList: TIndexedWordList);
>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(AStream: TStream; AWordList: TIndexedWordList);
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
    destructor Destroy; override;
>>>>>>> origin/fixes_2.4
  end;

  { TChmSearchReader }

  TChmWLCTopic = record
    TopicIndex: DWord;
    LocationCodes: array of DWord;
  end;

  TChmWLCTopicArray = array of TChmWLCTopic;

  TChmSearchReader = class;

  TChmSearchReaderFoundDataEvent = procedure(Sender: TChmSearchReader; AWord: String; ATopic: DWord; AWordIndex: DWord) of object;

  TChmSearchReader = class(TObject)
  private
    FStream: TStream;
    FFileIsValid: Boolean;
    FFreeStreamOnDestroy: Boolean;
    FDocRootSize,
    FCodeCountRootSize,
    FLocCodeRootSize: Integer;
    FTreeDepth: Integer;
    FRootNodeOffset: DWord;
    FActiveNodeStart: DWord;
    FActiveNodeFreeSpace: Word;
    FNextLeafNode: DWord;
    procedure ReadCommonData;
    procedure MoveToFirstLeafNode;
    procedure MoveToRootNode;
    procedure MoveToNode(ANodeOffset: DWord; ANodeDepth: Integer);
    function  ReadWordOrPartialWord(ALastWord: String): String; // returns the whole word using the last word as a base
    function  ReadIndexNodeEntry(ALastWord: String; out AWord: String; out ASubNodeStart: DWord): Boolean;
    function  ReadLeafNodeEntry(ALastWord: String; out AWord: String; out AInTitle: Boolean; out AWLCCount: DWord; out AWLCOffset: DWord; out AWLCSize: DWord): Boolean;
    function  ReadWLCEntries(AWLCCount: DWord; AWLCOffset: DWord; AWLCSize: DWord): TChmWLCTopicArray;
  public
    constructor Create(AStream: TStream; AFreeStreamOnDestroy: Boolean);
    destructor  Destroy; override;
    procedure   DumpData(AFoundDataEvent: TChmSearchReaderFoundDataEvent);
<<<<<<< HEAD
<<<<<<< HEAD
    function    LookupWord(AWord: String; out ATitleHits: TChmWLCTopicArray; AStartsWith: Boolean = True): TChmWLCTopicArray;
=======
    function    LookupWord(AWord: String; out ATitleHits: TChmWLCTopicArray): TChmWLCTopicArray;
>>>>>>> graemeg/fixes_2_2
=======
    function    LookupWord(AWord: String; out ATitleHits: TChmWLCTopicArray): TChmWLCTopicArray;
>>>>>>> origin/fixes_2_2
    property    FileIsValid: Boolean read FFileIsValid;
  end;

const
  FIFTI_NODE_SIZE = 4096;

implementation
uses SysUtils, Math, ChmBase;

type

{ TIndexNode }

  TIndexNode = class(TFIftiNode)
    function  GuessIfCanHold(AWord: String): Boolean; override;
    procedure ChildIsFull ( AWord: String; ANodeOffset: DWord ); override;
    procedure Flush(NewBlockNeeded: Boolean); override;
  end;

  { TLeafNode }

  TLeafNode = class(TFIftiNode)
    FLeafNodeCount: DWord;
    FLastNodeStart: DWord;
    FreeSpace: DWord;
    FDocRootSize,
    FCodeRootSize,
    FLocRootSize: Byte;
    procedure WriteInitialHeader;
    Destructor Destroy; override;
    function  GuessIfCanHold(AWord: String): Boolean; override;
    procedure Flush(NewBlockNeeded: Boolean); override;
    procedure AddWord(AWord: TIndexedWord);
    function WriteWLCEntries(AWord: TIndexedWord; ADocRootSize, ACodeRootSize, ALocRootSize: Byte): DWord;
    property LeafNodeCount: DWord read FLeafNodeCount;
    property DocRootSize: Byte read FDocRootSize write FDocRootSize;
    property CodeRootSize: Byte read FCodeRootSize write FCodeRootSize;
    property LocRootSize: Byte read FLocRootSize write FLocRootSize;
  end;

function GetCompressedIntegerBE(Stream: TStream): DWord;
var
  Buf: Byte;
  Value: Dword = 0;
  Shift: Integer = 0;
begin
  repeat
    Buf := Stream.ReadByte;
    Value := Value or  (Buf and $7F) shl Shift;
    Inc(Shift, 7);
  until (Buf and $80) = 0;
  Result := Value;
end;

procedure WriteCompressedIntegerBE(Stream: TStream; AInt: DWord);
var
  Bits: Integer;
  Tmp: DWord;
  Buf: Byte;
begin
  Tmp := AInt;
  Bits := 0;
  while Tmp <> 0 do
  begin
    Tmp := Tmp shr 1;
    Inc(Bits);
  end;

  repeat
    Buf := (AInt shr (Tmp * 7)) and $7F;
    if Bits > 7 then
      Buf := Buf or $80;
    Dec(Bits, 7);
    Inc(Tmp);
    Stream.WriteByte(Buf);
  until Bits <= 0;
end;

function WriteScaleRootInt(ANumber: DWord; out Bits: DWord; Root: Integer): Byte;
var
  Tmp: DWord;
  Mask: DWord;
//  Scale: Integer;
  NeededBits: Integer;
  PrefixBits: Integer;
  RootBits: Integer;
begin
//  Scale := 2;
  Bits := 0;
  Result := Root;

  Tmp := ANumber;
  NeededBits := 0;
  while Tmp <> 0 do
  begin
    Inc(NeededBits);
    Tmp := Tmp shr 1;
  end;
  PrefixBits := Max(0, NeededBits-Root);

  RootBits := NeededBits -1; //
  if RootBits < Root then
    RootBits := Root;
  if RootBits < 0 then
    RootBits := 0;

  Mask := 0;
  if RootBits-1 >= 0 then
    for Tmp := 0 to RootBits-1 do
       Mask := Mask or (DWord(1) shl Tmp);
  Bits := not Mask;
  Bits := Bits shl 1; // make space for empty bit
  Bits := Bits or (ANumber and Mask);

  Result := PrefixBits + 1 + RootBits;
  Bits := (Bits shl (32-Result)) shr (32 - Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
{ TChmSearchWriter }

procedure TChmSearchWriter.ProcessWords;
begin
  FWordList.ForEach(@WriteAword);
=======
=======
>>>>>>> origin/fixes_2_2


{ TChmSearchWriter }

procedure TChmSearchWriter.ProcessWords;
var
  AWord: TIndexedWord;
begin
  AWord := FWordList.FirstWord;
  while AWord <> nil do
  begin
    WriteAWord(AWord);
    AWord := AWord.NextWord;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  if FActiveLeafNode <> nil then
    FActiveLeafNode.Flush(False); // causes the unwritten parts of the tree to be written
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TChmSearchWriter.GetHasData: Boolean;
begin
  Result := FWordList.IndexedFileCount > 0;
end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
procedure TChmSearchWriter.WriteHeader ( IsPlaceHolder: Boolean ) ;
var
  TmpNode: TFIftiNode;
  i: Integer;
begin
  if IsPlaceHolder then
  begin
    FStream.Size := $400; // the header size. we will fill this after the nodes have been determined
    FStream.Position := $400;
    FillChar(PChar(TMemoryStream(FStream).Memory)^, $400, 0);
    FHeaderRec.DocIndexRootSize := 1;
    FHeaderRec.CodeCountRootSize := 1;
    FHeaderRec.LocationCodeRootSize := 4;
    Exit;
  end;
  // write the glorious header
  FHeaderRec.Sig[2] := $28;
  FHeaderRec.HTMLFilesCount := FWordList.IndexedFileCount;
<<<<<<< HEAD
<<<<<<< HEAD
  FHeaderRec.RootNodeOffset := FStream.Size-FIFTI_NODE_SIZE;
=======
  FHeaderRec.RootNodeOffset := FStream.Size-4096;
>>>>>>> graemeg/fixes_2_2
=======
  FHeaderRec.RootNodeOffset := FStream.Size-4096;
>>>>>>> origin/fixes_2_2
  FHeaderRec.LeafNodeCount := TLeafNode(FActiveLeafNode).LeafNodeCount;
  FHeaderRec.CopyOfRootNodeOffset := FHeaderRec.RootNodeOffset;
  FHeaderRec.TreeDepth := 0;
  TmpNode := FActiveLeafNode;
  while TmpNode <> nil do
  begin
    Inc(FHeaderRec.TreeDepth);
    TmpNode := TmpNode.ParentNode;
  end;
  FHeaderRec.DocIndexScale := 2;
  FHeaderRec.CodeCountScale := 2;
  FHeaderRec.LocationCodeScale := 2;

  //FHeaderRec.DocIndexRootSize := 15;
  //FHeaderRec.CodeCountRootSize := 15;
  //FHeaderRec.LocationCodeRootSize := 15;

<<<<<<< HEAD
<<<<<<< HEAD
  FHeaderRec.NodeSize := FIFTI_NODE_SIZE;
=======
  FHeaderRec.NodeSize := 4096;
>>>>>>> graemeg/fixes_2_2
=======
  FHeaderRec.NodeSize := 4096;
>>>>>>> origin/fixes_2_2
  FHeaderRec.LongestWordLength := FWordList.LongestWord;
  FHeaderRec.TotalWordsIndexed := FWordList.TotalWordCount;
  FHeaderRec.TotalWords := FWordList.TotalDIfferentWords;
  FHeaderRec.TotalWordsLengthPart1 := FWordList.TotalWordLength;
  FHeaderRec.TotalWordsLength := FWordList.TotalDifferentWordLength;
  FHeaderRec.WindowsCodePage := 1252;

  FStream.Position := 0;

  FStream.Write(FHeaderRec.Sig[0], 4);
  FStream.WriteDWord(NtoLE(FHeaderRec.HTMLFilesCount));
  FStream.WriteDWord(NtoLE(FHeaderRec.RootNodeOffset));
  FStream.WriteDWord(NtoLE(0)); // unknown 1
  FStream.WriteDWord(NtoLE(FHeaderRec.LeafNodeCount));
  FStream.WriteDWord(NtoLE(FHeaderRec.RootNodeOffset)); // yes twice
  FStream.WriteWord(NtoLE(FHeaderRec.TreeDepth));
  FStream.WriteDWord(NtoLE(DWord(7)));
  FStream.WriteByte(2);
  FStream.WriteByte(FHeaderRec.DocIndexRootSize);
  FStream.WriteByte(2);
  FStream.WriteByte(FHeaderRec.CodeCountRootSize);
  FStream.WriteByte(2);
  FStream.WriteByte(FHeaderRec.LocationCodeRootSize);
  // eat 10 bytes
  FStream.WriteWord(0);
  FStream.WriteDWord(0);
  FStream.WriteDWord(0);

  FStream.WriteDWord(NtoLE(FHeaderRec.NodeSize));
  FStream.WriteDWord(NtoLE(DWord(0)));
  FStream.WriteDWord(1);
  FStream.WriteDWord(5);
  FStream.WriteDWord(NtoLE(FHeaderRec.LongestWordLength));
  FStream.WriteDWord(NtoLE(FHeaderRec.TotalWordsIndexed));
  FStream.WriteDWord(NtoLE(FHeaderRec.TotalWords));
  FStream.WriteDWord(NtoLE(FHeaderRec.TotalWordsLengthPart1));
  FStream.WriteDWord(NtoLE(FHeaderRec.TotalWordsLengthPart2));
  FStream.WriteDWord(NtoLE(FHeaderRec.TotalWordsLength));
  FStream.WriteDWord(NtoLE(TLeafNode(FActiveLeafNode).FreeSpace));
  FStream.WriteDWord(NtoLE(0));
  FStream.WriteDWord(NtoLE(FHeaderRec.HTMLFilesCount-1));
  for i := 0 to 23 do FStream.WriteByte(0);
  FStream.WriteDWord(NtoLE(FHeaderRec.WindowsCodePage));
  FStream.WriteDWord(NtoLE(DWord(1033))); // LCID
  for i := 0 to 893 do FStream.WriteByte(0);
end;

procedure TChmSearchWriter.WriteAWord ( AWord: TIndexedWord ) ;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
begin
  if FActiveLeafNode = nil then
  begin
    FActiveLeafNode := TLeafNode.Create(FStream);
    with TLeafNode(FActiveLeafNode) do
    begin
      DocRootSize := FHeaderRec.DocIndexRootSize;
      CodeRootSize := FHeaderRec.CodeCountRootSize;
      LocRootSize := FHeaderRec.LocationCodeRootSize;
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
  if FActiveLeafNode.GuessIfCanHold(AWord.TheWord) = False then
  begin
    FActiveLeafNode.Flush(True);
  end;
  TLeafNode(FActiveLeafNode).AddWord(AWord);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
procedure TChmSearchWriter.WriteToStream;
begin
  WriteHeader(True);
  ProcessWords;
  WriteHeader(False);
end;

constructor TChmSearchWriter.Create ( AStream: TStream;
  AWordList: TIndexedWordList ) ;
begin
  FStream := AStream;
  FWordList := AWordList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FActiveLeafNode:=NIL; 
end;

destructor TChmSearchWriter.Destroy;

=======
  FActiveLeafNode:=NIL; 
end;

destructor TChmSearchWriter.Destroy;

>>>>>>> origin/fixes_2.4
begin
 freeandnil(FActiveLeafNode);
end;


<<<<<<< HEAD
=======

end;

>>>>>>> graemeg/fixes_2_2
=======

end;

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ TLeafNode }

function TFIftiNode.RemainingSpace: DWord;
begin
  Result := FIFTI_NODE_SIZE - FBlockStream.Position;
end;

constructor TFIftiNode.Create ( AStream: TStream ) ;
begin
  inherited Create;
  FWriteStream := AStream;
  FBlockStream := TMemoryStream.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  OwnsParentNode :=false;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  OwnsParentNode :=false;
>>>>>>> origin/fixes_2.4
end;

destructor TFIftiNode.Destroy;
begin
  FBlockStream.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if OwnsParentNode then ParentNode.Free;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  if OwnsParentNode then ParentNode.Free;
>>>>>>> origin/fixes_2.4
  inherited Destroy;
end;

procedure TFIftiNode.FillRemainingSpace;
begin
  while RemainingSpace > 0 do
    FBlockStream.WriteByte(0);
end;

function TFIftiNode.AdjustedWord ( AWord: String; out AOffset: Byte; AOldWord: String ) : String;
var
  Count1,
  Count2: Integer;
  Count: Integer;
  i: Integer;
begin
  if AWord = AOldWord then
  begin
    AOffset := Length(AWord);
    Exit('');
  end;
  // else
  Count1 := Length(AOldWord);
  Count2 := Length(AWord);

  if Count1<Count2 then
    Count := Count1
  else
    Count := Count2;

  for i := 1 to Count do
  begin
    AOffset := i-1;
    if AOldWord[i] <> AWord[i]
      then Exit(Copy(AWord, i, Length(AWord)));
  end;
  Result := AWord;
  AOffset := 0;
end;

procedure TLeafNode.WriteInitialHeader;
begin
  FBlockStream.WriteDWord(0);
  FBlockStream.WriteWord(0);
  FBlockStream.WriteWord(0);
end;

destructor TLeafNode.Destroy;
begin
  inherited Destroy;
end;

function TLeafNode.GuessIfCanHold ( AWord: String ) : Boolean;
var
  WordOffset: Byte;
begin
  Result := 17 + Length(AdjustedWord(AWord, WordOffset, FLastWord)) < RemainingSpace;
end;

procedure TLeafNode.Flush(NewBlockNeeded: Boolean);
var
  FTmpPos: DWord;
begin
  Inc(FLeafNodeCount);
  FTmpPos := FWriteStream.Position;
  // update the previous leaf node about our position.
  if FLastNodeStart > 0 then
  begin
    FWriteStream.Position := FLastNodeStart;
    FWriteStream.WriteDWord(NtoLE(FTmpPos));
    FWriteStream.Position := FTmpPos;
  end;
  FLastNodeStart := FTmpPos;

  FreeSpace := RemainingSpace;

  FillRemainingSpace;

  // update the leaf header to show the available space.
  FBlockStream.Position := 6;
  FBlockStream.WriteWord(NtoLE(Word(FreeSpace)));

  // copy the leaf block to the fiftimain file
  FBlockStream.Position := 0;
  FWriteStream.CopyFrom(FBlockStream, FIFTI_NODE_SIZE);
  FBlockStream.Position := 0;

  if NewBlockNeeded or ((NewBlockNeeded = False) and (ParentNode <> nil)) then
  begin
    if ParentNode = nil then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
      begin
        ParentNode := TIndexNode.Create(FWriteStream);
        OwnsParentNode:=True;
      end;
<<<<<<< HEAD
=======
      ParentNode := TIndexNode.Create(FWriteStream);

>>>>>>> graemeg/fixes_2_2
=======
      ParentNode := TIndexNode.Create(FWriteStream);

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    ParentNode.ChildIsFull(FLastWord, FLastNodeStart);
    if (NewBlockNeeded = False) then
      ParentNode.Flush(False);
  end;

  FLastWord := '';
end;

procedure TLeafNode.AddWord ( AWord: TIndexedWord ) ;
var
  Offset: Byte;
  NewWord: String;
  WLCSize: DWord;
begin
  if Length(AWord.TheWord) > 99 then
    Exit; // Maximum word length is 99
  if FBlockStream.Position = 0 then
    WriteInitialHeader;

  NewWord := AdjustedWord(AWord.TheWord, Offset, FLastWord);

  FLastWord := AWord.TheWord;

  FBlockStream.WriteByte(Length(NewWord)+1);
  FBlockStream.WriteByte(Offset);
<<<<<<< HEAD
<<<<<<< HEAD

  // length can be 0 if it is the same word as the last. there is a word entry each for title and content

  if Length(NewWord) > 0 then
    FBlockStream.Write(NewWord[1], Length(NewWord));

=======
  FBlockStream.Write(NewWord[1], Length(Trim(NewWord)));
>>>>>>> graemeg/fixes_2_2
=======
  FBlockStream.Write(NewWord[1], Length(Trim(NewWord)));
>>>>>>> origin/fixes_2_2
  FBlockStream.WriteByte(Ord(AWord.IsTitle));
  WriteCompressedIntegerBE(FBlockStream, AWord.DocumentCount);
  FBlockStream.WriteDWord(NtoLE(DWord(FWriteStream.Position)));
  FBlockStream.WriteWord(0);

  // write WLC to FWriteStream so we can write the size of the wlc entries
  WLCSize := WriteWLCEntries(AWord, FDocRootSize, FCodeRootSize, FLocRootSize);

  WriteCompressedIntegerBE(FBlockStream, WLCSize);
<<<<<<< HEAD
<<<<<<< HEAD
  if FBlockStream.Position > FIFTI_NODE_SIZE then
    raise Exception.Create('FIFTIMAIN Leaf node has written past the block!');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function Min(AValue, BValue: Byte): Byte;
begin
  if AValue < BValue then
    Result := AValue
  else Result := BValue;
end;

function Max(AValue, BValue: Byte): Byte;
begin
  if AValue > BValue then
    Result := AValue
  else Result := BValue;
end;

function Max(AValue, BValue: Integer): Integer;
begin
  if AValue > BValue then
    Result := AValue
  else Result := BValue;
end;

function Max(AValue, BValue: DWord): DWord;
begin
  if AValue > BValue then
    Result := AValue
  else Result := BValue;
end;



function TLeafNode.WriteWLCEntries ( AWord: TIndexedWord ; ADocRootSize, ACodeRootSize, ALocRootSize: Byte) : DWord;
var
  LastDocIndex: DWord;
  LastLocCode: DWord;
  UsedBits: Byte;
  Buf: Byte;
  function NewDocDelta(ADocIndex: DWord): DWord;
  begin
    Result := ADocIndex - LastDocIndex;
    LastDocIndex := ADocIndex;
  end;
  function NewLocCode(ALocCode: DWord): DWord;
  begin
    Result := ALocCode - LastLocCode;
    LastLocCode := ALocCode;
  end;
  procedure AddValue(AValue: DWord; BitCount: Byte);
  var
    NeededBits: Byte;
    Tmp: Byte;
  begin
    AValue := AValue shl (32 - BitCount);
    while BitCount > 0 do
    begin
      NeededBits := 8 - UsedBits;
      Tmp := Hi(Hi(DWord(AValue shr (UsedBits))));
      Buf := Buf or Tmp;
      Inc(UsedBits, Min(BitCount, NeededBits));
      AValue := AValue shl Min(BitCount, NeededBits);
      Dec(BitCount, Min(BitCount, NeededBits));

      if (UsedBits = 8) then
      begin
        FWriteStream.WriteByte(Buf);
        UsedBits := 0;
        NeededBits := 0;
        Buf := 0;
      end;
    end;
  end;
  procedure FlushBuffer;
  begin
    if UsedBits > 0 then
      FWriteStream.WriteByte(Buf);
    UsedBits := 0;
    Buf := 0;
  end;
var
  DocDelta: DWord;
  LocDelta: DWord;
  StartPos: DWord;
  Bits: DWord;
  BitCount: Byte;
  i,
  j: Integer;
  Doc: TIndexDocument;
//  proced
begin
  StartPos := FWriteStream.Position;
  LastDocIndex := 0;
  UsedBits := 0;
  Buf := 0;
  for i := 0 to AWord.DocumentCount-1 do
  begin
    LastLocCode := 0;
    Doc := AWord.GetLogicalDocument(i);
    DocDelta := NewDocDelta(Doc.DocumentIndex);
    BitCount := WriteScaleRootInt(DocDelta, Bits, ADocRootSize);
    AddValue(Bits, BitCount);
<<<<<<< HEAD
<<<<<<< HEAD
    BitCount := WriteScaleRootInt(Doc.NumberOfIndexEntries, Bits, ACodeRootSize);
    AddValue(Bits, BitCount);

    for j := 0 to Doc.NumberOfIndexEntries-1 do
    begin
      LocDelta := NewLocCode(Doc.IndexEntry[j]);
=======
=======
>>>>>>> origin/fixes_2_2
    BitCount := WriteScaleRootInt(Length(Doc.WordIndex), Bits, ACodeRootSize);
    AddValue(Bits, BitCount);

    for j := 0 to High(Doc.WordIndex) do
    begin
      LocDelta := NewLocCode(Doc.WordIndex[j]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      BitCount := WriteScaleRootInt(LocDelta, Bits, ALocRootSize);
      AddValue(Bits, BitCount);
    end;
    FlushBuffer;
  end;


  Result := FWriteStream.Position-StartPos;
end;


{ TIndexNode }

function TIndexNode.GuessIfCanHold ( AWord: String ) : Boolean;
var
  Offset: Byte;
begin
  Result := FBlockStream.Position + 8 + Length(AdjustedWord(AWord, Offset, FLastWord)) < FIFTI_NODE_SIZE;
end;

procedure TIndexNode.ChildIsFull ( AWord: String; ANodeOffset: DWord ) ;
var
  Offset: Byte;
<<<<<<< HEAD
<<<<<<< HEAD
  NewWord: String;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if FBlockStream.Position = 0 then
    FBlockStream.WriteWord(0); // free space at end. updated when the block is flushed
  if GuessIfCanHold(AWord) = False then
    Flush(True);
<<<<<<< HEAD
<<<<<<< HEAD
  NewWord := AdjustedWord(AWord, Offset, FLastWord);
  FLastWord:=AWord;

  // Write the Index node Entry
  FBlockStream.WriteByte(Length(NewWord)+1);
  FBlockStream.WriteByte(Offset);
  FBlockStream.Write(NewWord[1], Length(NewWord));
  FBlockStream.WriteDWord(NtoLE(ANodeOffset));
  FBlockStream.WriteWord(0);
  if FBlockStream.Position > FIFTI_NODE_SIZE then
    raise Exception.Create('FIFTIMAIN Index node has written past the block!');
=======
=======
>>>>>>> origin/fixes_2_2
  AWord := AdjustedWord(AWord, Offset, FLastWord);

  // Write the Index node Entry
  FBlockStream.WriteByte(Length(AWord)+1);
  FBlockStream.WriteByte(Offset);
  FBlockStream.Write(AWord[1], Length(AWord));
  FBlockStream.WriteDWord(NtoLE(ANodeOffset));
  FBlockStream.WriteWord(0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TIndexNode.Flush ( NewBlockNeeded: Boolean ) ;
var
  RemSize: DWord;
begin
  if NewBlockNeeded then
  begin
    if ParentNode = nil then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
      begin
        ParentNode := TIndexNode.Create(FWriteStream);
        OwnsParentNode:=True;
      end;
<<<<<<< HEAD
=======
      ParentNode := TIndexNode.Create(FWriteStream);
>>>>>>> graemeg/fixes_2_2
=======
      ParentNode := TIndexNode.Create(FWriteStream);
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
  end;

  if ParentNode <> nil then
    ParentNode.ChildIsFull(FLastWord, FWriteStream.Position);

  RemSize := RemainingSpace;
  FillRemainingSpace;
  FBlockStream.Position := 0;
  FBlockStream.WriteWord(NtoLE(RemSize));

  FBlockStream.Position := 0;

  FWriteStream.CopyFrom(FBlockStream, FIFTI_NODE_SIZE);
<<<<<<< HEAD
<<<<<<< HEAD
  FBlockStream.Position := 0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FLastWord := '';

  if NewBlockNeeded then
    FBlockStream.WriteDWord(0) // placeholder to write free space in when block is full
  else
    if ParentNode <> nil then
      ParentNode.Flush(NewBlockNeeded);
end;

{ TChmSearchReader }

procedure TChmSearchReader.ReadCommonData;
var
  Sig: DWord;
begin
   FStream.Position := 0;
   Sig := LEtoN(FStream.ReadDWord);
   FFileIsValid :=  Sig = $00280000;

   if not FileIsValid then
     Exit;

   // root node address
   FStream.Position := $8;
   FRootNodeOffset := LEtoN(FStream.ReadDWord);

   // Tree Depth
   FStream.Position := $18;
   FTreeDepth := LEtoN(FStream.ReadWord);

   // Root sizes for scale and root integers
   FStream.Position := $1E;
   if FStream.ReadByte <> 2 then // we only can read the files when scale is 2
     FFileIsValid := False;
   FDocRootSize := FStream.ReadByte;

   if FStream.ReadByte <> 2 then
     FFileIsValid := False;
   FCodeCountRootSize := FStream.ReadByte;

   if FStream.ReadByte <> 2 then
     FFileIsValid := False;
   FLocCodeRootSize := FStream.ReadByte;

end;

procedure TChmSearchReader.MoveToFirstLeafNode;
var
  NodeDepth: Integer;
  NodeOffset: DWord;
  LastWord: String;
  NewWord: String;
begin
  NodeDepth := FTreeDepth;
  MoveToRootNode;
  while NodeDepth > 1 do
  begin
    LastWord := '';
    ReadIndexNodeEntry(LastWord, NewWord, NodeOffset);
    Dec(NodeDepth);
    MoveToNode(NodeOffset, NodeDepth);
  end;
end;

procedure TChmSearchReader.MoveToRootNode;
begin
  MoveToNode(FRootNodeOffset, FTreeDepth);
end;

procedure TChmSearchReader.MoveToNode(ANodeOffset: DWord; ANodeDepth: Integer);
begin
  FStream.Position := ANodeOffset;
  FActiveNodeStart := FStream.Position;
  if ANodeDepth > 1 then
  begin
    FnextLeafNode := 0;
    FActiveNodeFreeSpace := LEtoN(FStream.ReadWord); // empty space at end of node
  end
  else
  begin
    FnextLeafNode := LEtoN(FStream.ReadDWord);
    FStream.ReadWord;
    FActiveNodeFreeSpace := LEtoN(FStream.ReadWord);
  end;
end;

function TChmSearchReader.ReadWordOrPartialWord ( ALastWord: String ) : String;
var
  WordLength: Integer;
  CopyLastWordCharCount: Integer;
begin
  WordLength := FStream.ReadByte;
  CopyLastWordCharCount := FStream.ReadByte;
  if CopyLastWordCharCount > 0 then
    Result := Copy(ALastWord, 1, CopyLastWordCharCount);
  SetLength(Result, (WordLength-1) + CopyLastWordCharCount);
<<<<<<< HEAD
<<<<<<< HEAD
  if WordLength > 1 then
    FStream.Read(Result[1+CopyLastWordCharCount], WordLength-1);
=======
  FStream.Read(Result[1+CopyLastWordCharCount], WordLength-1);
>>>>>>> graemeg/fixes_2_2
=======
  FStream.Read(Result[1+CopyLastWordCharCount], WordLength-1);
>>>>>>> origin/fixes_2_2
end;

function TChmSearchReader.ReadIndexNodeEntry (ALastWord: String;  out AWord: String; out
  ASubNodeStart: DWord ): Boolean;
begin
  Result := FStream.Position - FActiveNodeStart < FIFTI_NODE_SIZE - FActiveNodeFreeSpace;
  if not Result then
    Exit;
  AWord := ReadWordOrPartialWord(ALastWord);
  ASubNodeStart := LEtoN(FStream.ReadDWord);
  FStream.ReadWord;
end;

function TChmSearchReader.ReadLeafNodeEntry ( ALastWord: String; out
  AWord: String; out AInTitle: Boolean; out AWLCCount: DWord; out
  AWLCOffset: DWord; out AWLCSize: DWord ): Boolean;
begin
  Result := FStream.Position - FActiveNodeStart < FIFTI_NODE_SIZE - FActiveNodeFreeSpace;
  if not Result then
    Exit;
  AWord := ReadWordOrPartialWord(ALastWord);
  AInTitle := FStream.ReadByte = 1;
  AWLCCount := GetCompressedIntegerBE(FStream);
  AWLCOffset := LEtoN(FStream.ReadDWord);
  FStream.ReadWord;
  AWLCSize := GetCompressedIntegerBE(FStream);

end;

function TChmSearchReader.ReadWLCEntries (AWLCCount: DWord; AWLCOffset: DWord; AWLCSize: DWord ) : TChmWLCTopicArray;

  function AtEndOfWLCEntries: Boolean;
  begin
    Result := AWLCOffset + AWLCSize <= FStream.Position;
  end;
var
  Buf: Byte;
  BitsInBuffer: Integer;

  procedure FillBuffer;
  begin
    while (BitsInBuffer = 0) and not AtEndOfWLCEntries do
    begin
      Buf := FStream.ReadByte;
      Inc(BitsInBuffer, 8);
    end;
  end;

  function ReadWLC(RootSize: DWord): DWord;
  var
    PrefixBits: Integer = 0;
    BitCount: Integer = 0;
    RemainingBits: Integer; // only the bits for this number not the bits in buffer
  begin
    FillBuffer;
    Result := 0;
    while (Buf and $80) > 0 do // find out how many prefix bits there are
    begin
      Inc(PrefixBits);
      Buf := Buf shl 1;
      Dec(BitsInBuffer);
      FillBuffer;

    end;

    if PrefixBits > 0 then
      Result := 1;
    Inc(BitCount, PrefixBits+1);
    Buf := Buf shl 1;
    Dec(BitsInBuffer);

    FillBuffer;
    Remainingbits := RootSize + Max(Integer(PrefixBits-1), 0);
    while RemainingBits > 0 do
    begin
      Result := Result shl 1;
      Result := Result or (Buf shr 7);
      Dec(RemainingBits);
      Buf := Buf shl 1;
      Dec(BitsInBuffer);
      FillBuffer;
      Inc(BitCount);
    end;
  end;
  procedure ClearBuffer;
  begin
    BitsInBuffer := 0;
    Buf := 0;
  end;

var
  TopicHits: DWord;
  i: Integer;
  j: Integer;
  CachedStreamPos: QWord;
  LastDoc,
  LastLocCode: DWord;
begin
  CachedStreamPos := FStream.Position;
  FStream.Position := AWLCOffset;
  {for i := 0 to AWLCSize-1 do
  begin
    Buf := FStream.ReadByte;
    Write(binStr(Buf, 8), ' ');
  end;}
  FStream.Position := AWLCOffset;
  SetLength(Result, AWLCCount);
  Buf := 0;
  BitsInBuffer := 0;
  LastDoc := 0;

  for i := 0 to AWLCCount-1 do
  begin
    Result[i].TopicIndex := ReadWLC(FDocRootSize) + LastDoc;

    LastDoc := Result[i].TopicIndex;
    TopicHits := ReadWLC(FCodeCountRootSize);
    SetLength(Result[i].LocationCodes, TopicHits);
    LastLocCode := 0;
    for j := 0 to TopicHits-1 do
    begin
      Result[i].LocationCodes[j] := ReadWLC(FLocCodeRootSize) + LastLocCode;
      LastLocCode := Result[i].LocationCodes[j];
    end;
    ClearBuffer;
  end;
  FStream.Position := CachedStreamPos;
end;



constructor TChmSearchReader.Create ( AStream: TStream;
  AFreeStreamOnDestroy: Boolean ) ;
begin
  FStream := AStream;
  FFreeStreamOnDestroy := AFreeStreamOnDestroy;
  ReadCommonData;
end;

destructor TChmSearchReader.Destroy;
begin
  if FFreeStreamOnDestroy then
    FreeAndNil(FStream);
  inherited Destroy;
end;

procedure TChmSearchReader.DumpData (
  AFoundDataEvent: TChmSearchReaderFoundDataEvent ) ;
var
  LastWord: String;
  TheWord: String;
  InTitle: Boolean;
  WLCCount: DWord;
  WLCOffset: DWord;
  WLCSize: DWord;
  FoundHits: TChmWLCTopicArray;
  i: Integer;
  j: Integer;
begin
  MoveToFirstLeafNode;
  LastWord := '';
  repeat
    if  (ReadLeafNodeEntry(LastWord, TheWord, InTitle, WLCCount, WLCOffset, WLCSize) = False) then
    begin
      if FnextLeafNode <> 0 then
      begin
        MoveToNode(FnextLeafNode, 1);
        LastWord := '';
      end
      else
        Break;
    end
    else begin
      LastWord := TheWord;
      //WriteLn('Reading Hits for ', TheWord ,' at ', hexstr(WLCOffset,8) );
      FoundHits := ReadWLCEntries(WLCCount, WLCOffset, WLCSize);
      //WriteLn('DONE Reading Hits for ', TheWord);
     // AFoundDataEvent(Self, TheWord, 0,0);//FoundHits[i].TopicIndex ,-1);//FoundHits[i].LocationCodes[j]);
      for i := 0 to High(FoundHits) do
        for j := 0 to High(FoundHits[i].LocationCodes) do
           AFoundDataEvent(Self, TheWord, FoundHits[i].TopicIndex ,FoundHits[i].LocationCodes[j]);
    end;
  until False; //FStream.Position - FActiveNodeStart >= FIFTI_NODE_SIZE - FActiveNodeFreeSpace
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TChmSearchReader.LookupWord(AWord: String; out ATitleHits: TChmWLCTopicArray; AStartsWith: Boolean = True): TChmWLCTopicArray;
=======
function TChmSearchReader.LookupWord(AWord: String; out ATitleHits: TChmWLCTopicArray): TChmWLCTopicArray;
>>>>>>> graemeg/fixes_2_2
=======
function TChmSearchReader.LookupWord(AWord: String; out ATitleHits: TChmWLCTopicArray): TChmWLCTopicArray;
>>>>>>> origin/fixes_2_2
var
  LastWord: String;
  NewWord: String;
  NodeLevel: Integer;
  NewNodePosition: DWord;
  InTitle: Boolean;
  WLCCount: DWord;
  WLCOffset: DWord;
  WLCSize: DWord;
  CompareResult: Integer;
  ReadNextResult: Boolean;
begin
  AWord := LowerCase(AWord);
  NodeLevel := FTreeDepth;
  MoveToRootNode;
  SetLength(Result, 0);
  LastWord := '';
  // descend the index node tree until we find the leafnode
  while NodeLevel > 1 do begin
     //WriteLn('At Node Level ', NodeLevel);
     if ReadIndexNodeEntry(LastWord, NewWord, NewNodePosition) <> False then
     begin
<<<<<<< HEAD
<<<<<<< HEAD
       LastWord := NewWord;
       //WriteLn('Found Index Entry: ', NewWord, ' Comparing to ', AWord);
       if  ChmCompareText(NewWord, AWord) >= 0 then
=======
       //WriteLn('Found Index Entry: ', NewWord, ' Comparing to ', AWord);
       if ChmCompareText(NewWord, AWord) >= 0 then
>>>>>>> graemeg/fixes_2_2
=======
       //WriteLn('Found Index Entry: ', NewWord, ' Comparing to ', AWord);
       if ChmCompareText(NewWord, AWord) >= 0 then
>>>>>>> origin/fixes_2_2
       begin
         LastWord := '';
         Dec(NodeLevel);
         MoveToNode(NewNodePosition, NodeLevel);
       end;
     end
     else
       Break;
  end;
  if NodeLevel > 1 then
    Exit; // the entry we are looking for is > than the last entry of the last index node

  // now we are in a leafnode
  while ReadLeafNodeEntry(LastWord, NewWord, InTitle, WLCCount, WLCOffset, WLCSize) <> False do
  begin
    //WriteLn('Found Leaf Entry: ', NewWord, ' Comparing to ', AWord);
    LastWord := NewWord;
<<<<<<< HEAD
<<<<<<< HEAD
    if Length(NewWord) < Length(AWord) then
      continue;

    if AStartsWith then //it only has to start with the searched term
      CompareResult := ChmCompareText(AWord, Copy(NewWord, 1, Length(AWord)))
    else // it must match exactly
      CompareResult := ChmCompareText(AWord, NewWord);
=======
    CompareResult := ChmCompareText(AWord, NewWord);
>>>>>>> graemeg/fixes_2_2
=======
    CompareResult := ChmCompareText(AWord, NewWord);
>>>>>>> origin/fixes_2_2
    if CompareResult < 0 then
      Exit;
    if CompareResult = 0 then
    begin
      if InTitle then
        ATitleHits := ReadWLCEntries(WLCCount, WLCOffset, WLCSize)
      else
        Result := ReadWLCEntries(WLCCount, WLCOffset, WLCSize);
      // check if the next entry is the same word since there is an entry for titles and for body

      if  (ReadLeafNodeEntry(LastWord, NewWord, InTitle, WLCCount, WLCOffset, WLCSize)) then
        ReadNextResult := True
      else if (FNextLeafNode <> 0) then
      begin
        MoveToNode(FNextLeafNode, 1);
        LastWord := '';
        ReadNextResult := (ReadLeafNodeEntry(LastWord, NewWord, InTitle, WLCCount, WLCOffset, WLCSize));
      end;
      if ReadNextResult and (NewWord = AWord) then
      begin
        if InTitle then
          ATitleHits := ReadWLCEntries(WLCCount, WLCOffset, WLCSize)
        else
          Result := ReadWLCEntries(WLCCount, WLCOffset, WLCSize);
      end;
      Exit;
    end;
  end;
end;


end.

