{ Copyright (C) <2010> <Andrew Haines> itloitlsreader.pas

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
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/cpstrnew
}
{
  See the file COPYING.modifiedLGPL, included in this distribution,
  for details about the copyright.
}
unit ITOLITLSReader;

{$mode objfpc}{$H+}

{ $DEFINE DEBUG_HELP2}

interface

uses
  Classes, chmreader, itolitlstypes, Sysutils, chmbase, itsftransform;

type

  { TITOLITLSReader }

  TITOLITLSReader = class(TITSFReader)
  private
    FStartStreamPos: QWord; // used when the data we are reading is part of a larger file
    SectionNames: TStringList;
    function GetStreamPos: Qword;
    procedure SetStreamPos(const AValue: Qword);

  private
    Header: TITOLITLSHeader;
    HeaderSectionTable: array of TITOLITLSHeaderSectionEntry;
    PostHeader: TITOLITLSPostHeader;
    CAOLHeader: TCAOLRec;
    function FileSize: QWord;
    function GetChunkType(AStream: TStream): TDirChunkType;
    function GetTransform(const AGuid: TGuid): TITSFTransform;
    procedure ReadHeader; override;
    procedure ReadHeaderEntries; override;
    function  GetTransforms(ASectionPrefix: String): TITSFTransformList;

    property StreamPos: Qword read GetStreamPos write SetStreamPos;
  public
    constructor Create(AStream: TStream; FreeStreamOnDestroy: Boolean); override;
    destructor Destroy; override;
    procedure GetCompleteFileList(ForEach: TFileEntryForEach; AIncludeInternalFiles: Boolean = True); override;
    function ObjectExists(Name: String): QWord; override;
    function GetObject(Name: String): TMemoryStream; override;

  end;

implementation

type

  { TStreamChunk }

  TStreamChunk = class(TStream)
  private
    FStream: TStream;
    FSize: QWord;
    FBasePos: QWord;
    FPos: QWord;
  public
    Function GetSize : Int64; Override;
    function Read(var Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    constructor Create(AHostStream: TStream; ABasePos, ASize: QWord);
  end;

{ TStreamChunk }

function TStreamChunk.GetSize: Int64;
begin
  Result:=FSize;
end;

function TStreamChunk.Read(var Buffer; Count: Longint): Longint;
begin
  FStream.Seek(FBasePos+FPos, soFromBeginning);
  {$IFDEF DEBUG_HELP2}
  //WriteLn('Want Read Count: ', Count,' Pos = ', FPos);
  //if FSize - FPos < Count then
  //  Count := FSize - FPos;
  {$ENDIF}
  Result := FStream.Read(Buffer, Count);
  Inc(FPos, Result);
end;

function TStreamChunk.Seek(Offset: Longint; Origin: Word): Longint;
var
  NewPosition: LongInt;
begin
  Case Origin of
    soFromBeginning : NewPosition:=Offset;
    soFromEnd       : NewPosition:=FSize+Offset;
    soFromCurrent   : NewPosition:=NewPosition+Offset;
  end;
  {$IFDEF DEBUG_HELP2}
  //WriteLn('WantSeek = ', Offset,' Size = ', FSize);
  {$ENDIF}
  FPos:=NewPosition;
  Exit(NewPosition);
  if NewPosition < 0 then NewPosition := 0;
  if NewPosition >= FSize then NewPosition := FSize-1;
  FStream.Position := FBasePos+NewPosition;
  Result := FStream.Position - FBasePos;
  FPos := Result;
  {$IFDEF DEBUG_HELP2}
  //WriteLn('Pos = ', fpos);
  {$ENDIF}
end;

constructor TStreamChunk.Create(AHostStream: TStream; ABasePos, ASize: QWord);
begin
  FStream := AHostStream;
  FBasePos := ABasePos;
  FSize := ASize;
  {$IFDEF DEBUG_HELP2}
  //WriteLn('Created Size = ', FSize, ' Offset = ', ABasePos);
  {$ENDIF}
end;



{ TITOLITLSReader }

function TITOLITLSReader.GetStreamPos: Qword;
begin
  Result := fStream.Position-FStartStreamPos;
end;

procedure TITOLITLSReader.SetStreamPos(const AValue: Qword);
begin
  fStream.Position:=FStartStreamPos+AValue;
end;

function TITOLITLSReader.FileSize: QWord;
begin
  fStream.Position:=LEtoN(HeaderSectionTable[0].OffSet)+8;
  fStream.Read(Result, 8);
  Result := LEtoN(Result);
end;

function TITOLITLSReader.GetChunkType(AStream: TStream): TDirChunkType;
var
  Sig: TSig;
begin
  AStream.Read(Sig, 4);
  if Sig = 'PMGL' then Result := ctPMGL
  else if Sig = 'PMGI' then Result := ctPMGI
  else if Sig = 'AOLL' then Result := ctAOLL
  else if Sig = 'AOLI' then Result := ctAOLI;
  AStream.Seek(-4,soFromCurrent);
end;

function TITOLITLSReader.GetTransform(const AGuid: TGuid): TITSFTransform;
begin
  Result := nil;
end;

procedure TITOLITLSReader.ReadHeader;
var
  CachedPos: QWord;
  SectionName: string;
  i: Integer;
begin
  {$IFDEF DEBUG_HELP2}
  WriteLn(ClassName);
  {$ENDIF}
  fStream.Read(Header, SizeOf(TITOLITLSHeader));
  if not((Header.Sig[0] = 'ITOL') and (Header.Sig[1] = 'ITLS')) then
    Exit;
  ReadHeaderEntries;
  CachedPos := StreamPos;
  fStream.Read(PostHeader, Sizeof(TITOLITLSPostHeader));
  StreamPos := CachedPos + PostHeader.CAOLOffset;
  fStream.Read(CAOLHeader, SizeOf(TCAOLRec));
  {$IFDEF DEBUG_HELP2}
  WriteLn(CAOLHeader.ITSFHeader.ITSFsig);
  {$ENDIF}
  GetSections(SectionNames);
  for i := 1 to SectionNames.Count-1 do
  begin
    FmtStr(SectionName, '::DataSpace/Storage/%s/',[SectionNames[i]]);
    SectionNames.Objects[i] := GetTransforms(SectionName);
  end;
end;

procedure TITOLITLSReader.ReadHeaderEntries;
var
 i: Integer;
begin
  StreamPos := Header.HeaderSectionTableOffset;
  SetLength(HeaderSectionTable, Header.HeaderSectionEntryCount);
  for i := 0 to Header.HeaderSectionEntryCount -1 do
  begin
    fStream.Read(HeaderSectionTable[i], SizeOf(TITOLITLSHeaderSectionEntry));
    HeaderSectionTable[i].OffSet:= LEtoN(HeaderSectionTable[i].OffSet);
    HeaderSectionTable[i].Length:= LEtoN(HeaderSectionTable[i].Length);
    {$IFDEF DEBUG_HELP2}
    //WriteLn('Entry #',i,' Offset = ',Entry.OffSet,' Length = ',Entry.Length);
    {$ENDIF}

  end;
end;

function TITOLITLSReader.GetTransforms(ASectionPrefix: String): TITSFTransformList;
var
  Stream: TMemoryStream;
  Guid: TGuid;
  GCount: Integer;
  Transform: TITSFTransform;
  TransformClass: TITSFTransformClass = nil;
  Idx: Integer;
begin
  Result := nil;
  Stream := GetObject(ASectionPrefix+'Transform/List');
  if Stream = nil then
  begin
    {$IFDEF DEBUG_HELP2}
    //WriteLn(ASectionPrefix+'Transform/List doesnt exist!');
    {$ENDIF}
    Exit;
  end;

  Result := TITSFTransformList.Create;

  FillChar(Guid, SizeOf(Guid), 0);
  TransformClass := RegisteredTransforms.Transform[Guid];
  Idx := Result.AddTransform(TransformClass);
  Transform := TransformClass.Create(@Self.GetObject, nil);
  Result.TransformInstance[Idx] := Transform;
  {$IFDEF DEBUG_HELP2}
  WriteLn('Sec: ', ASectionPrefix, ' Transform Add ', Transform.ClassName);
  {$ENDIF}

  GCount := Stream.Size div SizeOf(TGuid);
  while GCount > 0 do
  begin
    Dec(GCount);
    Stream.Read(Guid, 16);
    TransformClass := RegisteredTransforms.Transform[Guid];
    Idx := Result.AddTransform(TransformClass);
    Transform := TransformClass.Create(@Self.GetObject, Transform);
    Result.TransformInstance[Idx] := Transform;
    {$IFDEF DEBUG_HELP2}
    WriteLn('Sec: ', ASectionPrefix, ' Transform Add ', Transform.ClassName);
    {$ENDIF}
  end;
  Stream.Free;
end;

constructor TITOLITLSReader.Create(AStream: TStream;
  FreeStreamOnDestroy: Boolean);
begin
  inherited Create(AStream, FreeStreamOnDestroy);
end;

destructor TITOLITLSReader.Destroy;
begin
  if SectionNames<> nil then
  begin
    while SectionNames.Count > 0 do
    begin
     if SectionNames.Objects[SectionNames.Count-1] <> nil then
       SectionNames.Objects[SectionNames.Count-1].Free;
     SectionNames.Delete(SectionNames.Count-1);
    end;
    SectionNames.Free;
  end;
  inherited Destroy;
end;

function ReadEntry(AStream: TStream): TPMGListChunkEntry;
var
  NameLength: LongInt;
begin
  NameLength:=GetCompressedInteger(AStream);
  SetLength(Result.Name, NameLength);
  AStream.Read(Result.Name[1], NameLength);
  Result.ContentSection:= GetCompressedInteger(AStream);
  Result.ContentOffset:= GetCompressedInteger(AStream);
  Result.DecompressedLength:= GetCompressedInteger(AStream);
end;

procedure TITOLITLSReader.GetCompleteFileList(ForEach: TFileEntryForEach; AIncludeInternalFiles: Boolean = True);
var
  SecOffset: QWord;
  IFCM: TIFCMRec;
  ChunkType: TDirChunkType;
  Chunk: TMemoryStream;
  i, j: Integer;
  AOLL: TAOLLChunkHeader;
  AOLI: TAOLIChunkHeader;
  Entry: TPMGListChunkEntry;// not really a PMGL entry but the members are the same
  NameLength: LongInt;
  EntryCount: Word;
begin
  StreamPos:=HeaderSectionTable[1].OffSet;
  fStream.Read(IFCM, SizeOf(IFCM));

  for i := 0 to IFCM.ChunkCount-1 do
  begin
    Chunk:= TMemoryStream.Create;
    Chunk.CopyFrom(fStream, IFCM.ChunkSize);
    Chunk.Position:=0;

    ChunkType:= GetChunkType(Chunk);
    case ChunkType of
      ctAOLL :
      begin
        Chunk.Read(AOLL, SizeOf(AOLL));
        Chunk.Seek(-2, soFromEnd);
        EntryCount:= LEtoN(Chunk.ReadWord);
        {$IFDEF DEBUG_HELP2}
        WriteLn(EntryCount);
        {$ENDIF}
        Chunk.Seek(SizeOf(AOLL), soFromBeginning);
        for j := 0 to EntryCount-1 do
        begin
          Entry := ReadEntry(Chunk);
          ForEach(Entry.Name, Entry.ContentOffset, Entry.DecompressedLength, Entry.ContentSection);
        end;
      end;
      ctAOLI :
      begin
        //Chunk.Read(AOLI, SizeOf(AOLI));
      end;
    end;
    Chunk.Free;
  end;

end;

function TITOLITLSReader.ObjectExists(Name: String): QWord;
var
 IFCM: TIFCMRec;
 ChunkIndex: QWord;
 Chunk: TMemoryStream;
 StartOfChunks: QWord;
 EntryCount: Word;
 AOLL: TAOLLChunkHeader;
 AOLI: TAOLIChunkHeader;
 Entry: TPMGListChunkEntry;
 CRes: LongInt;
 i: integer;
begin
  Result := 0;

  if Name = fCachedEntry.Name then
    Exit(fCachedEntry.DecompressedLength); // we've already looked it up

  fCachedEntry.Name:='';
  fCachedEntry.ContentSection:=LongWord(-1);
  fCachedEntry.DecompressedLength:=0;
  fCachedEntry.ContentOffset:=QWord(-1);

  StreamPos:=HeaderSectionTable[1].OffSet;
  fStream.Read(IFCM, SizeOf(IFCM));
  StartOfChunks := fstream.Position;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$R-}
  ChunkIndex:=PostHeader.ListChunkInfo.TopAOLIChunkIndex;
  if ChunkIndex = -1 then
    ChunkIndex := PostHeader.ListChunkInfo.FirstAOLLChunkIndex;

  Chunk := TMemoryStream.Create;
  while ChunkIndex <> -1 do
  begin
    Chunk.Position:=0;
    fStream.Position:= StartOfChunks + (IFCM.ChunkSize*ChunkIndex);
    Chunk.CopyFrom(fStream, IFCM.ChunkSize);
    Chunk.Position:=0;

    case GetChunkType(Chunk) of
      ctAOLL :
      begin
        Chunk.Read(AOLL, SizeOf(AOLL));
        Chunk.Seek(-2, soFromEnd);
        EntryCount:= LEtoN(Chunk.ReadWord);
        {$IFDEF DEBUG_HELP2}
        WriteLn(EntryCount);
        {$ENDIF}
        Chunk.Seek(SizeOf(AOLL), soFromBeginning);
        for i := 0 to EntryCount-1 do
        begin
          Entry := ReadEntry(Chunk);
          CRes := ChmCompareText(Name, Entry.Name);
          if CRes = 0 then
          begin
            ChunkIndex:=-1;
            fCachedEntry := Entry;
            Break;

          end
          else if CRes > 0 then
            Continue
          else
          begin
            ChunkIndex := -1;
            Break;
          end;
        end;
      end;
      ctAOLI :
      begin
        //Chunk.Read(AOLI, SizeOf(AOLI));
      end;
    end;


  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$pop}
=======
  {$R+}
>>>>>>> graemeg/cpstrnew
=======
  {$R+}
>>>>>>> graemeg/cpstrnew
=======
  {$R+}
>>>>>>> graemeg/cpstrnew
=======
  {$R+}
>>>>>>> origin/cpstrnew
  Chunk.Free;
  Result := fCachedEntry.DecompressedLength;

end;

function TITOLITLSReader.GetObject(Name: String): TMemoryStream;
var
  Entry,
  ContentEntry: TPMGListChunkEntry;
  SectionName: String;
  Transforms: TITSFTransformList;
  Transform: TITSFTransform;
  ContentStream: TStream;
  ChunkPos: QWord;
  i: Integer;
begin
  Result := nil;
  {$IFDEF DEBUG_HELP2}
  WriteLn('Want: ', Name);
  {$ENDIF}
  if ObjectExists(Name) = 0 then begin
    //WriteLn('Object ', name,' Doesn''t exist or is zero sized.');
    Exit;
  end;
  if Name = '/' then
    Exit; // wierd bug where written size and offset contain random data
  Entry := fCachedEntry;



  if Entry.ContentSection = 0 then begin
    Result := TMemoryStream.Create;
    {$IFDEF DEBUG_HELP2}
    WriteLn('Offset = ', Entry.ContentOffset);
    {$ENDIF}
    //StreamPos := CAOLHeader.ITSFHeader.Section0Offset + Entry.ContentOffset;
    ChunkPos := CAOLHeader.ITSFHeader.Section0Offset;// + fCachedEntry.ContentOffset;
    ContentStream := TStreamChunk.Create(fStream, ChunkPos, FileSize-ChunkPos);
    ContentStream.Seek(Entry.ContentOffset, soFromBeginning);
    Result.CopyFrom(ContentStream, Entry.DecompressedLength);
    ContentStream.Free;

  end
  else
  begin
    FmtStr(SectionName, '::DataSpace/Storage/%s/',[SectionNames[Entry.ContentSection]]);
    {$IFDEF DEBUG_HELP2}
    WriteLn('Want: ', SectionName);
    {$ENDIF}
    if ObjectExists(SectionName+'Content') = 0 then
      Exit;
    ContentEntry := fCachedEntry;

    Transforms := TITSFTransformList(SectionNames.Objects[Entry.ContentSection]);
    if Transforms = nil then
      Exit;
    ChunkPos := CAOLHeader.ITSFHeader.Section0Offset + ContentEntry.ContentOffset;
    ContentStream := TStreamChunk.Create(fStream, ChunkPos, ContentEntry.DecompressedLength);
    //ContentStream := GetObject(SectionName+'Content');
    Result := TMemoryStream.Create;
    {$IFDEF DEBUG_HELP2}
    {for i := Transforms.Count-1 downto 0 do
    begin
        //WriteLn('Found Transform: ', GUIDToString(Transforms.TransformIndex[i].GUID));

        //WriteLn(Transform.ClassName);
        Transform := Transforms.TransformInstance[i];
        if Transform = nil then
          WriteLn('Trqansform = nil!');

    end;}

    WriteLn('Transform Count = ', Transforms.Count);
    WriteLn('Asking ', Transforms.TransformInstance[Transforms.Count-1].ClassName,' for data');
    {$ENDIF}
    Transforms.TransformInstance[Transforms.Count-1].WantData(SectionName, ContentStream, Entry.ContentOffset, Entry.DecompressedLength, Result);
    ContentStream.Free;
  end;
  Result.Position := 0;
end;

end.
