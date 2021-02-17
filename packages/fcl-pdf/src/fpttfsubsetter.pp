{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2016 by Graeme Geldenhuys

    This unit creates a new TTF subset font file, reducing the file
    size in the process. This is primarily so the new font file can
    be embedded in PDF documents.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpTTFSubsetter;

{$mode objfpc}{$H+}

{ $R+}

// enable this define for more verbose output
{.$define gdebug}

interface

uses
  Classes,
  SysUtils,
  contnrs,
  fpparsettf,
  FPFontTextMapping;

type
  ETTFSubsetter = class(Exception);

  TArrayUInt32 = array of UInt32;

  // forward declaration
  TGIDList = class;
  TGIDListEnumerator = class;


  { TFontSubsetter }

  TFontSubsetter = class(TObject)
  private
    FPrefix: string;
    FHasAddedCompoundReferences: boolean;  // one glyph made up of multiple glyphs
    FKeepTables: TStrings;
    FFontInfo: TTFFileInfo;
    FGlyphIDList: TTextMappingList;
    FStream: TFileStream; // original TTF file
    FGlyphLocations: array of UInt32;
    FGlyphIDs: TGIDList;
    function    Int32HighestOneBit(const AValue: integer): integer;
    function    Int32Log2(const AValue: integer): integer;
    function    ToUInt32(const AHigh, ALow: UInt32): UInt32;
    function    ToUInt32(const ABytes: AnsiString): UInt32;
    function    GetRawTable(const ATableName: AnsiString): TMemoryStream;
    function    WriteFileHeader(AOutStream: TStream; const nTables: integer): uint32;
    function    WriteTableHeader(AOutStream: TStream; const ATag: AnsiString; const AOffset: UInt32; const AData: TStream): UInt32;
    function    GetNewGlyphId(const OldGid: integer): Integer;
    procedure   WriteTableBodies(AOutStream: TStream; const ATables: TStringList);
    procedure   UpdateOrigGlyphIDList;
    // AGlyphID is the original GlyphID in the original TTF file
    function    GetCharIDfromGlyphID(const AGlyphID: uint32): uint32;
    { Copy glyph data as-is for a specific glyphID. }
    function    GetRawGlyphData(const AGlyphID: UInt16): TMemoryStream;
    procedure   LoadLocations;
    // Stream writing functions.
    procedure   WriteInt16(AStream: TStream; const AValue: Int16); inline;
    procedure   WriteUInt16(AStream: TStream; const AValue: UInt16); inline;
    procedure   WriteInt32(AStream: TStream; const AValue: Int32); inline;
    procedure   WriteUInt32(AStream: TStream; const AValue: UInt32); inline;
    function    ReadInt16(AStream: TStream): Int16; inline;
    function    ReadUInt32(AStream: TStream): UInt32; inline;
    function    ReadUInt16(AStream: TStream): UInt16; inline;

    procedure   AddCompoundReferences;
    function    buildHeadTable: TStream;
    function    buildHheaTable: TStream;
    function    buildMaxpTable: TStream;
    function    buildFpgmTable: TStream;
    function    buildPrepTable: TStream;
    function    buildCvtTable: TStream;
    function    buildGlyfTable(var newOffsets: TArrayUInt32): TStream;
    function    buildLocaTable(var newOffsets: TArrayUInt32): TStream;
    function    buildCmapTable: TStream;
    function    buildHmtxTable: TStream;
  public
    constructor Create(const AFont: TTFFileInfo; const AGlyphIDList: TTextMappingList);
    constructor Create(const AFont: TTFFileInfo);
    destructor  Destroy; override;
    procedure   SaveToFile(const AFileName: String);
    procedure   SaveToStream(const AStream: TStream);
    // Add the given Unicode codepoint to the subset.
    procedure   Add(const ACodePoint: uint32);
    // The prefix to add to the font's PostScript name.
    property    Prefix: string read FPrefix write FPrefix;
  end;


  TGIDItem = class(TObject)
  private
    FGID: integer;
    FGlyphData: TMemoryStream;
    FIsCompoundGlyph: boolean;
    FNewGID: integer;
  public
    constructor Create;
    destructor  Destroy; override;
    property    IsCompoundGlyph: boolean read FIsCompoundGlyph write FIsCompoundGlyph;
    property    GID: integer read FGID write FGID;
    property    GlyphData: TMemoryStream read FGlyphData write FGlyphData;
    property    NewGID: integer read FNewGID write FNewGID;
  end;


  TGIDList = class(TObject)
  private
    FList: TFPObjectList;
    function    GetCount: integer;
    function    GetItems(i: integer): TGIDItem;
    procedure   SetItems(i: integer; const AValue: TGIDItem);
  public
    constructor Create;
    destructor  Destroy; override;
    function    Add(const GID: Integer): integer; overload;
    function    Add(const AObject: TGIDItem): integer; overload;
    procedure   Clear;
    function    Contains(const GID: integer): boolean;
    function    GetEnumerator: TGIDListEnumerator;
    function    GetNewGlyphID(const OriginalGID: integer): integer;
    procedure   Sort;
    property    Count: integer read GetCount;
    property    Items[i: integer]: TGIDItem read GetItems write SetItems; default;
  end;


  TGIDListEnumerator = class(TObject)
  private
    FIndex: Integer;
    FList: TGIDList;
  public
    constructor Create(AList: TGIDList);
    function    GetCurrent: TGIDItem;
    function    MoveNext: Boolean;
    property    Current: TGIDItem read GetCurrent;
  end;




implementation

uses
  math;

resourcestring
  rsErrFontInfoNotAssigned = 'FontInfo was not assigned';
  rsErrFailedToReadFromStream = 'Failed to read from file stream';
  rsErrCantFindFontFile = 'Can''t find the actual TTF font file.';
  rsErrGlyphLocationsNotLoaded = 'Glyph Location data has not been loaded yet.';

const
  PAD_BUF: array[ 1..3 ] of Byte = ( $0, $0, $0 );


{ TFontSubsetter }

{ The method simply returns the int value with a single one-bit, in the position
  of the highest-order one-bit in the specified value, or zero if the specified
  value is itself equal to zero. }
function TFontSubsetter.Int32HighestOneBit(const AValue: integer): integer;
var
  i: integer;
begin
  i := AValue;
  i := i or (i shr 1);
  i := i or (i shr 2);
  i := i or (i shr 4);
  i := i or (i shr 8);
  i := i or (i shr 16);
//  i := i or (i shr 32);
  Result := i - (i shr 1);
end;

function TFontSubsetter.Int32Log2(const AValue: integer): integer;
begin
  if AValue <= 0 then
    raise Exception.Create('Illegal argument');
//  Result :=  31 - Integer.numberOfLeadingZeros(n);

  Result := Floor(Log10(AValue) / Log10(2));
end;

function TFontSubsetter.ToUInt32(const AHigh, ALow: UInt32): UInt32;
begin
  result := ((AHigh and $FFFF) shl 16) or (ALow and $FFFF);
end;

function TFontSubsetter.ToUInt32(const ABytes: AnsiString): UInt32;
var
  b: array of Byte absolute ABytes;
begin
  Result := (b[0] and $FF) shl 24
           or (b[1] and $FF) shl 16
           or (b[2] and $FF) shl 8
           or (b[3] and $FF);
end;

function TFontSubsetter.GetRawTable(const ATableName: AnsiString): TMemoryStream;
var
  lEntry: TTableDirectoryEntry;
begin
  Result := nil;
  FillMem(@lEntry, SizeOf(TTableDirectoryEntry), 0);
  if not FFontInfo.GetTableDirEntry(ATableName, lEntry) then
    Exit;

  Result := TMemoryStream.Create;
  FStream.Seek(lEntry.offset, soFromBeginning);
  if Result.CopyFrom(FStream, lEntry.Length) <> lEntry.Length then
    raise ETTF.Create('GetRawTable: ' + rsErrFailedToReadFromStream);
end;

{ AOutStream: the data output stream.
  nTables: the number of font tables.
  result: the file offset of the first TTF table to write. }
function TFontSubsetter.WriteFileHeader(AOutStream: TStream; const nTables: integer): uint32;
var
  mask: integer;
  searchRange: integer;
  entrySelector: integer;
  rangeShift: integer;
begin
  WriteUInt32(AOutStream, $00010000);
  WriteUInt16(AOutStream, nTables);

  mask := Int32HighestOneBit(nTables);
  searchRange := mask * 16;
  WriteUInt16(AOutStream, searchRange);

  entrySelector := Int32Log2(mask);
  WriteUInt16(AOutStream, entrySelector);

  rangeShift := 16 * nTables - searchRange;
  WriteUInt16(AOutStream, rangeShift);

  result := $00010000 + ToUInt32(nTables, searchRange) + ToUInt32(entrySelector, rangeShift);
end;

function TFontSubsetter.WriteTableHeader(AOutStream: TStream; const ATag: AnsiString; const AOffset: UInt32;
  const AData: TStream): UInt32;
var
  checksum, w: UInt32;
  n: integer;
  lByte: Byte;
begin
  AData.Position := 0;
  checksum := 0;
  w := 0;

  for n := 0 to AData.Size-1 do
  begin
    lByte := AData.ReadByte;
    //checksum := checksum + (((lByte and $FF) shl 24) - n mod 4 * 8);
    w := w or (lByte shl ((3 - (n mod 4))*8));
    if n mod 4 = 3 then begin
      Inc(checksum, w);
      w := 0;
    end;
  end;
  Inc(checksum, w);
  //checksum := checksum and $FFFFFFFF;

  AOutStream.WriteBuffer(Pointer(ATag)^, 4); // Tag is always 4 bytes - written as-is, no NtoBE() required
  WriteUInt32(AOutStream, checksum);
  WriteUInt32(AOutStream, AOffset);
  WriteUInt32(AOutStream, AData.Size);

  {$ifdef gdebug}
  writeln(Format('tag: "%s"  CRC: %8.8x  offset: %8.8x (%2:7d bytes)  size: %8.8x (%3:7d bytes)', [ATag, checksum, AOffset, AData.Size]));
  {$endif}

  // account for the checksum twice, once for the header field, once for the content itself
  Result := ToUInt32(ATag) + checksum + checksum + AOffset + AData.Size;
end;

function TFontSubsetter.GetNewGlyphId(const OldGid: integer): Integer;
var
  itm: TGIDItem;
begin
  result := -1;
  for itm in FGlyphIDs do
  begin
    if itm.GID = OldGID then
    begin
      Result := itm.NewGID;
      exit;
    end;
  end;
end;

procedure TFontSubsetter.WriteTableBodies(AOutStream: TStream; const ATables: TStringList);
var
  i: integer;
  n: uint64;
  lData: TStream;
begin
  for i := 0 to ATables.Count-1 do
  begin
    lData := TStream(ATables.Objects[i]);
    if lData <> nil then
    begin
      lData.Position := 0;
      n := lData.Size;
      AOutStream.CopyFrom(lData, lData.Size);
    end;
    if (n mod 4) <> 0 then
    begin
      {$ifdef gdebug}
      writeln('Padding applied at the end of ', ATables[i], ': ', 4 - (n mod 4), ' byte(s)');
      {$endif}
      AOutStream.WriteBuffer(PAD_BUF, 4 - (n mod 4));
    end;
  end;
end;

{ This updates the original GlyphIDList passed in to the constructor - normally
  done by fcl-pdf. This allows fcl-pdf to use the NewGlyphID values in its
  generated PDF output. }
procedure TFontSubsetter.UpdateOrigGlyphIDList;
var
  i: integer;
  itm: TGIDItem;
begin
  for itm in FGlyphIDs do
  begin
    for i := 0 to FGlyphIDList.Count-1 do
    begin
      if FGlyphIDList[i].GlyphID = itm.GID then
      begin
        FGlyphIDList[i].NewGlyphID := itm.NewGID;
        break;
      end;
    end;
  end;
end;

function TFontSubsetter.GetCharIDfromGlyphID(const AGlyphID: uint32): uint32;
var
  i: integer;
begin
  Result := 0;
  for i := 0 to Length(FFontInfo.Chars)-1 do
    if FFontInfo.Chars[i] = AGlyphID then
    begin
      Result := i;
      Exit;
    end;
end;

function TFontSubsetter.GetRawGlyphData(const AGlyphID: UInt16): TMemoryStream;
var
  lGlyf: TTableDirectoryEntry;
  lSize: UInt16;
begin
  Result := nil;
  if Length(FGlyphLocations) < 2 then
    raise ETTF.Create(rsErrGlyphLocationsNotLoaded);
  FillMem(@lGlyf, SizeOf(TTableDirectoryEntry), 0);
  FFontInfo.GetTableDirEntry(TTFTableNames[ttglyf], lGlyf);

  lSize := FGlyphLocations[AGlyphID+1] - FGlyphLocations[AGlyphID];
  Result := TMemoryStream.Create;
  if lSize > 0 then
  begin
    FStream.Seek(lGlyf.offset + FGlyphLocations[AGlyphID], soFromBeginning);
    if Result.CopyFrom(FStream, lSize) <> lSize then
      raise ETTF.Create('GetRawGlyphData: ' + rsErrFailedToReadFromStream)
    else
      Result.Position := 0;
  end;
end;

procedure TFontSubsetter.LoadLocations;
var
  lLocaEntry: TTableDirectoryEntry;
  lGlyf: TTableDirectoryEntry;
  ms: TMemoryStream;
  numLocations: integer;
  n: integer;
begin
  FillMem(@lGlyf, SizeOf(TTableDirectoryEntry), 0);
  FillMem(@lLocaEntry, SizeOf(TTableDirectoryEntry), 0);

  FFontInfo.GetTableDirEntry(TTFTableNames[ttglyf], lGlyf);
  if FFontInfo.GetTableDirEntry(TTFTableNames[ttloca], lLocaEntry) then
  begin
    ms := TMemoryStream.Create;
    try
      FStream.Seek(lLocaEntry.offset, soFromBeginning);
      if ms.CopyFrom(FStream, lLocaEntry.Length) <> lLocaEntry.Length then
        raise ETTF.Create('LoadLocations: ' + rsErrFailedToReadFromStream)
      else
        ms.Position := 0;

      if FFontInfo.Head.IndexToLocFormat = 0 then
      begin
        // Short offsets
        numLocations := lLocaEntry.Length shr 1;
        {$IFDEF gDEBUG}
        Writeln('Number of Glyph locations ( 16 bits offsets ): ', numLocations );
        {$ENDIF}
        SetLength(FGlyphLocations, numLocations);
        for n := 0 to numLocations-1 do
          FGlyphLocations[n] := BEtoN(ms.ReadWord) * 2;
      end
      else
      begin
        // Long offsets
        numLocations := lLocaEntry.Length shr 2;
        {$IFDEF gDEBUG}
        Writeln('Number of Glyph locations ( 32 bits offsets ): ', numLocations );
        {$ENDIF}
        SetLength(FGlyphLocations, numLocations);
        for n := 0 to numLocations-1 do
          FGlyphLocations[n] := BEtoN(ms.ReadDWord);
      end;
    finally
      ms.Free;
    end;
  end
  else
  begin
    {$ifdef gDEBUG}
    Writeln('WARNING: ''loca'' table is not found.');
    {$endif}
  end;
end;

procedure TFontSubsetter.WriteInt16(AStream: TStream; const AValue: Int16);
begin
  AStream.WriteBuffer(NtoBE(AValue), 2);
end;

procedure TFontSubsetter.WriteUInt16(AStream: TStream; const AValue: UInt16);
begin
  AStream.WriteWord(NtoBE(AValue));
end;

procedure TFontSubsetter.WriteInt32(AStream: TStream; const AValue: Int32);
begin
  AStream.WriteBuffer(NtoBE(AValue), 4);
end;

procedure TFontSubsetter.WriteUInt32(AStream: TStream; const AValue: UInt32);
begin
  AStream.WriteDWord(NtoBE(AValue));
end;

function TFontSubsetter.ReadInt16(AStream: TStream): Int16;
begin
  Result:=Int16(ReadUInt16(AStream));
end;

function TFontSubsetter.ReadUInt32(AStream: TStream): UInt32;
begin
  Result:=0;
  AStream.ReadBuffer(Result,SizeOf(Result));
  Result:=BEtoN(Result);
end;

function TFontSubsetter.ReadUInt16(AStream: TStream): UInt16;
begin
  Result:=0;
  AStream.ReadBuffer(Result,SizeOf(Result));
  Result:=BEtoN(Result);
end;

procedure TFontSubsetter.AddCompoundReferences;
var
  GlyphIDsToAdd: TStringList;
  n: integer;
  gs: TMemoryStream;
  buf: TGlyphHeader;
  i: integer;
  flags: uint16;
  glyphIndex: uint16;
  hasNested: boolean;
begin
  if FHasAddedCompoundReferences then
    Exit;
  FHasAddedCompoundReferences := True;

  LoadLocations;

  repeat
    GlyphIDsToAdd := TStringList.Create;
    GlyphIDsToAdd.Duplicates := dupIgnore;
    GlyphIDsToAdd.Sorted := True;

    for n := 0 to FGlyphIDs.Count-1 do
    begin
      if not Assigned(FGlyphIDs[n].GlyphData) then
        FGlyphIDs[n].GlyphData := GetRawGlyphData(FGlyphIDs[n].GID);
      gs := FGlyphIDs[n].GlyphData;
      gs.Position := 0;

      if gs.Size > 0 then
      begin
        FillMem(@buf, SizeOf(TGlyphHeader), 0);
        gs.ReadBuffer(buf, SizeOf(Buf));
        {$IFDEF gDEBUG}
        writeln('   glyph data size: ', gs.Size);
        {$ENDIF}

        if buf.numberOfContours = -1 then
        begin
          FGlyphIDs[n].IsCompoundGlyph := True;
          {$IFDEF gDEBUG}
          writeln('   numberOfContours: ', buf.numberOfContours);
          {$ENDIF}
          repeat
            flags := ReadUInt16(gs);
            glyphIndex := ReadUInt16(gs);
            // find compound glyph IDs and add them to the GlyphIDsToAdd list
            if not FGlyphIDs.Contains(glyphIndex) then
            begin
              {$IFDEF gDEBUG}
              writeln(Format('      glyphIndex: %.4x (%0:d) ', [glyphIndex]));
              {$ENDIF}
              GlyphIDsToAdd.Add(IntToStr(glyphIndex));
            end;
            // ARG_1_AND_2_ARE_WORDS
            if (flags and (1 shl 0)) <> 0 then
              ReadUInt32(gs)
            else
              ReadUInt16(gs);
            // WE_HAVE_A_TWO_BY_TWO
            if (flags and (1 shl 7)) <> 0 then
            begin
              ReadUInt32(gs);
              ReadUInt32(gs);
            end
            // WE_HAVE_AN_X_AND_Y_SCALE
            else if (flags and (1 shl 6)) <> 0 then
            begin
              ReadUInt32(gs);
            end
            // WE_HAVE_A_SCALE
            else if (flags and (1 shl 3)) <> 0 then
            begin
              ReadUInt16(gs);
            end;

          until (flags and (1 shl 5)) = 0;   // MORE_COMPONENTS
        end;  { if buf.numberOfContours = -1 }
      end;  { if gs.Size > 0 }
    end; { for n ... FGlyphIDs.Count-1 }

    if GlyphIDsToAdd.Count > 0 then
    begin
      for i := 0 to GlyphIDsToAdd.Count-1 do
      begin
        glyphIndex := StrToInt(GlyphIDsToAdd[i]);
        FGlyphIDs.Add(glyphIndex);
      end;
    end;
    hasNested := GlyphIDsToAdd.Count > 0;
    {$IFDEF gDEBUG}
    if hasNested then
      writeln('------------------');
    {$ENDIF}
    FreeAndNil(GlyphIDsToAdd);
  until (hasNested = false);
end;

function TFontSubsetter.buildHeadTable: TStream;
var
  t: THead;
  rec: THead;
  i: Integer;
begin
  Result := TMemoryStream.Create;

  t := FFontInfo.Head;
  FillMem(@rec, SizeOf(THead), 0);
  rec.FileVersion.Version := NtoBE(t.FileVersion.Version);
  rec.FontRevision.Version := NtoBE(t.FontRevision.Version);
  rec.CheckSumAdjustment := 0;
  rec.MagicNumber := NtoBE(t.MagicNumber);
  rec.Flags := NtoBE(t.Flags);
  rec.UnitsPerEm := NtoBE(t.UnitsPerEm);
  rec.Created := NtoBE(t.Created);
  rec.Modified := NtoBE(t.Modified);
  For i := 0 to 3 do
    rec.BBox[i] := NtoBE(t.BBox[i]);
  rec.MacStyle := NtoBE(t.MacStyle);
  rec.LowestRecPPEM := NtoBE(t.LowestRecPPEM);
  rec.FontDirectionHint := NtoBE(t.FontDirectionHint);
  // force long format of 'loca' table. ie: 'loca' table offsets are in 4-Bytes each, not Words.
  rec.IndexToLocFormat := NtoBE(Int16(1)); //NtoBE(t.IndexToLocFormat);
  rec.glyphDataFormat := NtoBE(t.glyphDataFormat);

  Result.WriteBuffer(rec, SizeOf(THead));
end;

function TFontSubsetter.buildHheaTable: TStream;
var
  t: THHead;
  rec: THHead;
  hmetrics: UInt16;
begin
  Result := TMemoryStream.Create;

  t := FFontInfo.HHead;
  FillMem(@rec, SizeOf(THHead), 0);
  rec.TableVersion.Version := NtoBE(t.TableVersion.Version);
  rec.Ascender := NtoBE(t.Ascender);
  rec.Descender := NtoBE(t.Descender);
  rec.LineGap := NtoBE(t.LineGap);
  rec.AdvanceWidthMax := NtoBE(t.AdvanceWidthMax);
  rec.MinLeftSideBearing := NtoBE(t.MinLeftSideBearing);
  rec.MinRightSideBearing := NtoBE(t.MinRightSideBearing);
  rec.XMaxExtent := NtoBE(t.XMaxExtent);
  rec.CaretSlopeRise := NtoBE(t.CaretSlopeRise);
  rec.CaretSlopeRun := NtoBE(t.CaretSlopeRun);
  rec.caretOffset := NtoBE(t.caretOffset);
  rec.metricDataFormat := NtoBE(t.metricDataFormat);
//  rec.numberOfHMetrics := NtoBE(t.numberOfHMetrics);

  hmetrics := FGlyphIDs.Count;
  if (FGlyphIDs.Items[FGlyphIDs.Count-1].GID >= t.numberOfHMetrics) and (not FGlyphIDs.Contains(t.numberOfHMetrics-1)) then
    inc(hmetrics);
  rec.numberOfHMetrics := NtoBE(hmetrics);

  Result.WriteBuffer(rec, SizeOf(THHead));
end;

function TFontSubsetter.buildMaxpTable: TStream;
var
  t: TMaxP;
  rec: TMaxP;
  lCount: word;
begin
  Result := TMemoryStream.Create;

  t := FFontInfo.MaxP;
  FillMem(@rec, SizeOf(TMaxP), 0);
  rec.VersionNumber.Version := NtoBE(t.VersionNumber.Version);

  lCount := FGlyphIDs.Count;
  rec.numGlyphs := NtoBE(lCount);

  rec.maxPoints := NtoBE(t.maxPoints);
  rec.maxContours := NtoBE(t.maxContours);
  rec.maxCompositePoints := NtoBE(t.maxCompositePoints);
  rec.maxCompositeContours := NtoBE(t.maxCompositeContours);
  rec.maxZones := NtoBE(t.maxZones);
  rec.maxTwilightPoints := NtoBE(t.maxTwilightPoints);
  rec.maxStorage := NtoBE(t.maxStorage);
  rec.maxFunctionDefs := NtoBE(t.maxFunctionDefs);
  rec.maxInstructionDefs := NtoBE(t.maxInstructionDefs);
  rec.maxStackElements := NtoBE(t.maxStackElements);
  rec.maxSizeOfInstructions := NtoBE(t.maxSizeOfInstructions);
  rec.maxComponentElements := NtoBE(t.maxComponentElements);
  rec.maxComponentDepth := NtoBE(t.maxComponentDepth);

  Result.WriteBuffer(rec, SizeOf(TMaxP));
end;

function TFontSubsetter.buildFpgmTable: TStream;
begin
  Result := GetRawTable('fpgm');
  if Assigned(Result) then
    Result.Position := 0;
end;

function TFontSubsetter.buildPrepTable: TStream;
begin
  Result := GetRawTable('prep');
  if Assigned(Result) then
  Result.Position := 0;
end;

function TFontSubsetter.buildCvtTable: TStream;
begin
  Result := GetRawTable('cvt ');
  if Assigned(Result) then
    Result.Position := 0;
end;

function TFontSubsetter.buildGlyfTable(var newOffsets: TArrayUInt32): TStream;
var
  n: integer;
  lOffset: uint32;
  lLen: uint32;
  gs: TMemoryStream;
  buf: TGlyphHeader;
  flags: uint16;
  glyphIndex: uint16;
begin
  lOffset := 0;
  Result := TMemoryStream.Create;
  LoadLocations;

  {  - Assign new glyph indexes
     - Retrieve glyph data if it doesn't yet exist (retrieved from original TTF file) }
  for n := 0 to FGlyphIDs.Count-1 do
  begin
    FGlyphIDs[n].NewGID := n;
    if not Assigned(FGlyphIDs[n].GlyphData) then
      FGlyphIDs[n].GlyphData := GetRawGlyphData(FGlyphIDs[n].GID);
  end;

  {   - Now fix GlyphID references in Compound Glyphs to point to new GlyphIDs }
  for n := 0 to FGlyphIDs.Count-1 do
  begin
    if not FGlyphIDs[n].IsCompoundGlyph then
      Continue;
    {$IFDEF gDEBUG}
    writeln(Format('found compound glyph:  %.4x   glyphID: %d', [0, FGlyphIDs[n].GID]));
    {$ENDIF}
    gs := TMemoryStream(FGlyphIDs[n].GlyphData);
    gs.Position := 0;

    if gs.Size > 0 then
    begin
      FillMem(@buf, SizeOf(TGlyphHeader), 0);
      gs.ReadBuffer(buf, SizeOf(Buf));

      if buf.numberOfContours = -1 then
      begin
        repeat
          flags := ReadUInt16(gs);
          lOffset := gs.Position;
          glyphIndex := ReadUInt16(gs);
          // now write new GlyphID in it's place.
          gs.Position := lOffset;
          glyphIndex := FGlyphIDs.GetNewGlyphID(glyphIndex);
          WriteUInt16(gs, glyphIndex);

          // ARG_1_AND_2_ARE_WORDS
          if (flags and (1 shl 0)) <> 0 then
            ReadUInt32(gs)
          else
            ReadUInt16(gs);
          // WE_HAVE_A_TWO_BY_TWO
          if (flags and (1 shl 7)) <> 0 then
          begin
            ReadUInt32(gs);
            ReadUInt32(gs);
          end
          // WE_HAVE_AN_X_AND_Y_SCALE
          else if (flags and (1 shl 6)) <> 0 then
          begin
            ReadUInt32(gs);
          end
          // WE_HAVE_A_SCALE
          else if (flags and (1 shl 3)) <> 0 then
          begin
            ReadUInt16(gs);
          end;

        until (flags and (1 shl 5)) = 0;   // MORE_COMPONENTS
      end;  { if buf.numberOfContours = -1 }
    end;  { if gs.Size > 0 }
  end; { for n ... FGlyphIDList.Count-1 }

  // write all glyph data to resulting data stream
  lOffset := 0;
  for n := 0 to FGlyphIDs.Count-1 do
  begin
    newOffsets[n] := lOffset;
    lOffset := lOffset + FGlyphIDs[n].GlyphData.Size;
    FGlyphIDs[n].GlyphData.Position := 0;
    Result.CopyFrom(FGlyphIDs[n].GlyphData, FGlyphIDs[n].GlyphData.Size);
    // 4-byte alignment
    if (lOffset mod 4) <> 0 then
    begin
      lLen := 4 - (lOffset mod 4);
      Result.WriteBuffer(PAD_BUF, lLen);
      lOffset := lOffset + lLen;
    end;
  end;
  newOffsets[n+1] := lOffset;
end;

// write as UInt32 as defined in head.indexToLocFormat field (long format).
function TFontSubsetter.buildLocaTable(var newOffsets: TArrayUInt32): TStream;
var
  i: integer;
begin
  Result := TMemoryStream.Create;
  for i := 0 to Length(newOffsets)-1 do
    WriteUInt32(Result, newOffsets[i]);
end;

function TFontSubsetter.buildCmapTable: TStream;
const
    // platform
    PLATFORM_UNICODE = 0;
    PLATFORM_MACINTOSH = 1;
    // value 2 is reserved; do not use
    PLATFORM_WINDOWS = 3;

    // Mac encodings
    ENCODING_MAC_ROMAN = 0;

    // Windows encodings
    ENCODING_WIN_SYMBOL = 0; // Unicode, non-standard character set
    ENCODING_WIN_UNICODE_BMP = 1; // Unicode BMP (UCS-2)
    ENCODING_WIN_SHIFT_JIS = 2;
    ENCODING_WIN_BIG5 = 3;
    ENCODING_WIN_PRC = 4;
    ENCODING_WIN_WANSUNG = 5;
    ENCODING_WIN_JOHAB = 6;
    ENCODING_WIN_UNICODE_FULL = 10; // Unicode Full (UCS-4)

    // Unicode encodings
    ENCODING_UNICODE_1_0 = 0;
    ENCODING_UNICODE_1_1 = 1;
    ENCODING_UNICODE_2_0_BMP = 3;
    ENCODING_UNICODE_2_0_FULL = 4;
var
  segCount: UInt16;
  searchRange: UInt16;
  i: integer;
  startCode: Array of Integer;
  endCode: Array of Integer;
  idDelta: Array of Integer;
  lastChar: integer;
  prevChar: integer;
  lastGid: integer;
  curGid: integer;
  itm: TTextMapping;
begin
  Result := TMemoryStream.Create;
  SetLength(startCode, FGlyphIDList.Count + 1);
  SetLength(endCode, FGlyphIDList.Count + 1);
  SetLength(idDelta, FGlyphIDList.Count + 1);

  // cmap header
  WriteUInt16(Result, 0);  // version
  WriteUInt16(Result, 1);  // numberSubTables

  // encoding record
  WriteUInt16(Result, PLATFORM_WINDOWS);  // platformID
  WriteUInt16(Result, ENCODING_WIN_UNICODE_BMP);  // platformSpecificID
  WriteUInt32(Result, 4 * 2 + 4); // offset

  // build Format 4 subtable (Unicode BMP)
  lastChar := 0;
  prevChar := lastChar;
  lastGid  := GetNewGlyphId(FGlyphIDList[0].GlyphID);
  segCount := 0;

  for i := 0 to FGlyphIDList.Count-1 do
  begin
    itm := FGlyphIDList[i];
    if itm.CharID > $FFFF then
      raise Exception.Create('non-BMP Unicode character');
    curGid := GetNewGlyphId(itm.GlyphID);

    if (itm.CharID <> FGlyphIDList[prevChar].CharID+1) or ((curGid - lastGid) <> (itm.CharID - FGlyphIDList[lastChar].CharID)) then
    begin
      if (lastGid <> 0) then
      begin
        { don't emit ranges, which map to GID 0, the undef glyph is emitted at the very last segment }
        startCode[segCount] := FGlyphIDList[lastChar].CharID;
        endCode[segCount] := FGlyphIDList[prevChar].CharID;
        idDelta[segCount] := lastGid - FGlyphIDList[lastChar].CharID;
        inc(segCount);
      end
      else if not (FGlyphIDList[lastChar].CharID = FGlyphIDList[prevChar].CharID) then
      begin
        { shorten ranges which start with GID 0 by one }
        startCode[segCount] := FGlyphIDList[lastChar].CharID + 1;
        endCode[segCount] := FGlyphIDList[prevChar].CharID;
        idDelta[segCount] := lastGid - FGlyphIDList[lastChar].CharID;
        inc(segCount);
      end;
      lastGid := curGid;
      lastChar := i;
    end;
    prevChar := i;
  end;

  // trailing segment
  startCode[segCount] := FGlyphIDList[lastChar].CharID;
  endCode[segCount] := FGlyphIDList[prevChar].CharID;
  idDelta[segCount] := lastGid - FGlyphIDList[lastChar].CharID;
  inc(segCount);

  // GID 0
  startCode[segCount] := $FFFF;
  endCode[segCount] := $FFFF;
  idDelta[segCount] := 1;
  inc(segCount);

  // write format 4 subtable
  searchRange := trunc(2 * Power(2, Floor(Log2(segCount))));
  WriteUInt16(Result, 4); // format
  WriteUInt16(Result, 8 * 2 + segCount * 4*2); // length
  WriteUInt16(Result, 0); // language
  WriteUInt16(Result, segCount * 2); // segCountX2
  WriteUInt16(Result, searchRange); // searchRange
  WriteUInt16(Result, trunc(log2(searchRange / 2))); // entrySelector
  WriteUInt16(Result, 2 * segCount - searchRange); // rangeShift

  // write endCode
  for i := 0 to segCount-1 do
    WriteUInt16(Result, endCode[i]);

  // reservedPad
  WriteUInt16(Result, 0);

  // startCode
  for i := 0 to segCount-1 do
    WriteUInt16(Result, startCode[i]);

  // idDelta
  for i := 0 to segCount-1 do
  begin
    {$IFDEF gDEBUG}
    writeln(Format(' idDelta[%d] = %d', [i, idDelta[i]]));
    {$ENDIF}
    WriteInt16(Result, idDelta[i]);
  end;

  // idRangeOffset
  for i := 0 to segCount-1 do
    WriteUInt16(Result, 0);
end;

function TFontSubsetter.buildHmtxTable: TStream;
var
  n: integer;
  GID: longint;
  LastGID: longint;
begin
  Result := TMemoryStream.Create;
  LastGID := Length(FFontInfo.Widths)-1;
  for n := 0 to FGlyphIDs.Count-1 do
  begin
    GID := FGlyphIDs[n].GID;
    if GID > LastGID then
      GID := LastGID;
    WriteUInt16(Result, FFontInfo.Widths[GID].AdvanceWidth);
    WriteInt16(Result, FFontInfo.Widths[GID].LSB);
  end;
end;

constructor TFontSubsetter.Create(const AFont: TTFFileInfo; const AGlyphIDList: TTextMappingList);
var
  i: integer;
begin
  FFontInfo := AFont;
  if not Assigned(FFontInfo) then
    raise ETTFSubsetter.Create(rsErrFontInfoNotAssigned);
  FGlyphIDList := AGlyphIDList;

  FGlyphIDs := TGIDList.Create;
  // always copy GID 0
  FGlyphIDs.Add(0);

  FKeepTables := TStringList.Create;
  FHasAddedCompoundReferences := False;
  FPrefix := '';

  // create a default list
  FKeepTables.Add('head');
  FKeepTables.Add('hhea');
  FKeepTables.Add('maxp');
  FKeepTables.Add('hmtx');
  FKeepTables.Add('cmap');
  FKeepTables.Add('fpgm');
  FKeepTables.Add('prep');
  FKeepTables.Add('cvt ');
  FKeepTables.Add('loca');
  FKeepTables.Add('glyf');

  if Assigned(FGlyphIDList) then
  begin
    FGlyphIDList.Sort;
    for i := 0 to FGlyphIDList.Count-1 do
      FGlyphIDs.Add(FGlyphIDList[i].GlyphID);
  end;

  if FFontInfo.Filename <> '' then
    FStream := TFileStream.Create(FFontInfo.FileName, fmOpenRead or fmShareDenyNone)
  else
    raise ETTF.Create(rsErrCantFindFontFile);
end;

constructor TFontSubsetter.Create(const AFont: TTFFileInfo);
begin
  Create(AFont, nil);
end;

destructor TFontSubsetter.Destroy;
var
  i: integer;
begin
  // the owner of FGlyphIDList doesn't need the GlyphData information
  for i := 0 to FGlyphIDList.Count-1 do
    FGlyphIDList[i].GlyphData.Free;
  FStream.Free;
  FKeepTables.Free;
  FreeAndNil(FGlyphIDs);
  inherited Destroy;
end;

procedure TFontSubsetter.SaveToFile(const AFileName: String);
var
  fs: TFileStream;
begin
  fs := TFileStream.Create(AFileName, fmCreate);
  try
    SaveToStream(fs);
  finally
    FreeAndNil(fs);
  end;
end;

procedure TFontSubsetter.SaveToStream(const AStream: TStream);
var
  checksum: UInt32;
  offset: int64;
  head: TStream;
  hhea: TStream;
  maxp: TStream;
  hmtx: TStream;
  cmap: TStream;
  fpgm: TStream;
  prep: TStream;
  cvt: TStream;
  loca: TStream;
  glyf: TStream;
  newLoca: TArrayUInt32;
  tables: TStringList;
  i: integer;
  o: uint64;
  p: uint64;
  lPadding: byte;
begin
  FGlyphIDs.Sort;

  // resolve compound glyph references
  AddCompoundReferences;

  // always copy GID 0
  FGlyphIDList.Add(0, 0);
  FGlyphIDList.Sort;

  SetLength(newLoca, FGlyphIDs.Count+1);

  head := buildHeadTable();
  hhea := buildHheaTable();
  maxp := buildMaxpTable();
  fpgm := buildFpgmTable();
  prep := buildPrepTable();
  cvt  := buildCvtTable();
  glyf := buildGlyfTable(newLoca);
  loca := buildLocaTable(newLoca);
  cmap := buildCmapTable();
  hmtx := buildHmtxTable();

  tables := TStringList.Create;
  tables.CaseSensitive := True;
  if Assigned(cmap) then
    tables.AddObject('cmap', cmap);
  if Assigned(glyf) then
    tables.AddObject('glyf', glyf);
  tables.AddObject('head', head);
  tables.AddObject('hhea', hhea);
  tables.AddObject('hmtx', hmtx);
  if Assigned(loca) then
    tables.AddObject('loca', loca);
  tables.AddObject('maxp', maxp);
  tables.AddObject('fpgm', fpgm);
  tables.AddObject('prep', prep);
  tables.AddObject('cvt ', cvt);
  tables.Sort;

  // calculate checksum
  checksum := writeFileHeader(AStream, tables.Count);
  offset := 12 + (16 * tables.Count);
  lPadding := 0;
  for i := 0 to tables.Count-1 do
  begin
    if tables.Objects[i] <> nil then
    begin
      checksum := checksum + WriteTableHeader(AStream, tables.Strings[i], offset, TStream(tables.Objects[i]));
      p := TStream(tables.Objects[i]).Size;
      // table bodies must be 4-byte aligned - calculate the padding so the tableHeader.Offset field can reflect that.
      if (p mod 4) = 0 then
        lPadding := 0
      else
        lPadding := 4 - (p mod 4);
      o := p + lPadding;
      offset := offset + o;
    end;
  end;
  checksum := UInt32($B1B0AFBA) - checksum;

  // update head.ChecksumAdjustment field
  head.Seek(8, soBeginning);
  WriteUInt32(head, checksum);

  // write table bodies
  WriteTableBodies(AStream, tables);

  for i := 0 to tables.Count-1 do
    TStream(tables.Objects[i]).Free;
  tables.Free;

  UpdateOrigGlyphIDList;
end;

procedure TFontSubsetter.Add(const ACodePoint: uint32);
var
  gid: uint32;
begin
  gid := FFontInfo.Chars[ACodePoint];
  if gid <> 0 then
  begin
    FGlyphIDList.Add(ACodePoint, FFontInfo.Chars[ACodePoint]);
    FGlyphIDs.Add(gid);
  end;
end;

{ TGIDList }

function TGIDList.GetCount: integer;
begin
  Result := FList.Count;
end;

function TGIDList.GetItems(i: integer): TGIDItem;
begin
  Result := FList[i] as TGIDItem;
end;

procedure TGIDList.SetItems(i: integer; const AValue: TGIDItem);
begin
  FList[i] := AValue;
end;

constructor TGIDList.Create;
begin
  FList := TFPObjectList.Create;
end;

destructor TGIDList.Destroy;
begin
  FList.Free;
  inherited Destroy;
end;

function TGIDList.Add(const GID: Integer): integer;
var
  itm: TGIDItem;
begin
  itm := TGIDItem.Create;
  itm.GID := GID;
  result := Add(itm);
end;

function TGIDList.Add(const AObject: TGIDItem): integer;
begin
  Result := FList.Add(AObject);
end;

procedure TGIDList.Clear;
begin
  FList.Clear;
end;

function TGIDList.Contains(const GID: integer): boolean;
var
  itm: TGIDItem;
begin
  Result := False;
  for itm in self do
  begin
    if itm.GID = GID then
    begin
      Result := True;
      Exit;
    end;
  end;
end;

function TGIDList.GetEnumerator: TGIDListEnumerator;
begin
  Result := TGIDListEnumerator.Create(self);
end;

function TGIDList.GetNewGlyphID(const OriginalGID: integer): integer;
var
  itm: TGIDItem;
begin
  Result := -1;
  for itm in self do
  begin
    if itm.GID = OriginalGID then
    begin
      Result := itm.NewGID;
      Exit;
    end;
  end;
end;

function CompareByGID(A, B: TGIDItem): Integer; inline;
begin
  if A.GID < B.GID then
    Result := -1
  else if A.GID > B.GID then
    Result := 1
  else
    Result := 0;
end;

function CompareByGIDPtr(A, B: Pointer): Integer;
begin
  Result := CompareByGID(TGIDItem(A), TGIDItem(B));
end;

procedure TGIDList.Sort;
begin
  FList.Sort(@CompareByGIDPtr);
end;

{ TGIDListEnumerator }

constructor TGIDListEnumerator.Create(AList: TGIDList);
begin
  FIndex := -1;
  FList := AList;
end;

function TGIDListEnumerator.GetCurrent: TGIDItem;
begin
  Result := FList[FIndex];
end;

function TGIDListEnumerator.MoveNext: Boolean;
begin
  Result := FIndex < (FList.Count-1);
  if Result then
    Inc(FIndex);
end;

{ TGIDItem }

constructor TGIDItem.Create;
begin
  FGID := -1;
  FNewGID := -1;
  FGlyphData := nil;
  FIsCompoundGlyph := False;
end;

destructor TGIDItem.Destroy;
begin
  FreeAndNil(FGlyphData);
  inherited Destroy;
end;


end.

