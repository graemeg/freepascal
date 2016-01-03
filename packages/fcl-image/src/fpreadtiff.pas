{
    This file is part of the Free Pascal run time library.
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 2012-2013 by the Free Pascal development team
=======
    Copyright (c) 2008 by the Free Pascal development team
>>>>>>> graemeg/fixes_2_2
=======
    Copyright (c) 2008 by the Free Pascal development team
>>>>>>> origin/fixes_2_2

    Tiff reader for fpImage.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

<<<<<<< HEAD
<<<<<<< HEAD
 Working:
   Black and white 1 bit,
   Grayscale 8,16bit (optional alpha),
   RGB 8,16bit (optional alpha),
   Orientation,
   skipping Thumbnail to read first image,
   compression: packbits, LZW, deflate
   endian
   multiple images
   strips and tiles

 ToDo:
   Compression: jpeg, ...
   PlanarConfiguration 2
   ColorMap
   separate mask
   fillorder - not needed by baseline tiff reader
   bigtiff 64bit offsets
   XMP tag 700
   ICC profile tag 34675
   orientation with rotation
=======
=======
>>>>>>> origin/fixes_2_2
  Working:
    Grayscale 8,16bit (optional alpha),
    RGB 8,16bit (optional alpha),
    Orientation,
    skipping Thumbnail to read first image,
    compression: packbits, (LZW started)
    endian

  ToDo:
    Compression: LZW, deflate, jpeg, ...
    Planar
    ColorMap
    multiple images
    separate mask
    pages
    fillorder - not needed by baseline tiff reader
    bigtiff 64bit offsets
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
}
unit FPReadTiff;

{$mode objfpc}{$H+}

<<<<<<< HEAD
<<<<<<< HEAD
{$inline on}

interface

uses
  Math, Classes, SysUtils, ctypes, zinflate, zbase, FPimage, FPTiffCmn;
=======
=======
>>>>>>> origin/fixes_2_2
interface

uses
  Math, Classes, SysUtils, FPimage, ctypes, FPTiffCmn;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type
  TFPReaderTiff = class;

  TTiffCreateCompatibleImgEvent = procedure(Sender: TFPReaderTiff;
<<<<<<< HEAD
<<<<<<< HEAD
                                            ImgFileDir: TTiffIFD) of object;

  TTiffCheckIFDOrder = (
    tcioSmart,
    tcioAlways,
    tcioNever
    );
=======
                                        var NewImage: TFPCustomImage) of object;
>>>>>>> graemeg/fixes_2_2
=======
                                        var NewImage: TFPCustomImage) of object;
>>>>>>> origin/fixes_2_2

  { TFPReaderTiff }

  TFPReaderTiff = class(TFPCustomImageReader)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    FCheckIFDOrder: TTiffCheckIFDOrder;
    FFirstIFDStart: DWord;
    FOnCreateImage: TTiffCreateCompatibleImgEvent;
    FReverserEndian: boolean;
    IFD: TTiffIFD;
    {$ifdef FPC_Debug_Image}
    FDebug: boolean;
    {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
    FOnCreateImage: TTiffCreateCompatibleImgEvent;
    FReverserEndian: boolean;
    IDF: TTiffIDF;
    FDebug: boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    fIFDStarts: TFPList;
    FReverseEndian: Boolean;
    fStartPos: int64;
    s: TStream;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetImages(Index: integer): TTiffIFD;
    procedure TiffError(Msg: string);
    procedure SetStreamPos(p: DWord);
    function ReadTiffHeader(QuickTest: boolean; out IFDStart: DWord): boolean; // returns IFD: offset to first IFD
    function ReadIFD(Start: DWord): DWord;// Image File Directory
=======
=======
>>>>>>> origin/fixes_2_2
    procedure TiffError(Msg: string);
    procedure SetStreamPos(p: DWord);
    function ReadTiffHeader(QuickTest: boolean; out IFD: DWord): boolean; // returns IFD: offset to first IFD
    function ReadIFD(Start: dword): DWord;// Image File Directory
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure ReadDirectoryEntry(var EntryTag: Word);
    function ReadEntryUnsigned: DWord;
    function ReadEntrySigned: Cint32;
    function ReadEntryRational: TTiffRational;
    function ReadEntryString: string;
    function ReadByte: Byte;
    function ReadWord: Word;
    function ReadDWord: DWord;
    procedure ReadValues(StreamPos: DWord;
                         out EntryType: word; out EntryCount: DWord;
                         out Buffer: Pointer; out ByteCount: PtrUInt);
    procedure ReadShortOrLongValues(StreamPos: DWord;
                                    out Buffer: PDWord; out Count: DWord);
    procedure ReadShortValues(StreamPos: DWord;
                              out Buffer: PWord; out Count: DWord);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure ReadImageProperties(
      out RedBits, GreenBits, BlueBits, GrayBits, AlphaBits: Word;
      out ExtraSamples: PWord; out ExtraSampleCnt: DWord;
      out SampleBits: PWord; out SampleBitsPerPixel: DWord);
    procedure ReadImgValue(BitCount: Word; var Run: Pointer; x: dword;
      Predictor: word; var LastValue: word; out Value: Word); inline;
    function FixEndian(w: Word): Word; inline;
    function FixEndian(d: DWord): DWord; inline;
    procedure SetFPImgExtras(CurImg: TFPCustomImage);
    procedure DecodePackBits(var Buffer: Pointer; var Count: PtrInt);
    procedure DecodeLZW(var Buffer: Pointer; var Count: PtrInt);
    procedure DecodeDeflate(var Buffer: Pointer; var Count: PtrInt; ExpectedCount: PtrInt);
  protected
    procedure InternalRead(Str: TStream; AnImage: TFPCustomImage); override;
    function InternalCheck(Str: TStream): boolean; override;
    procedure DoCreateImage(ImgFileDir: TTiffIFD); virtual;
  public
    ImageList: TFPList; // list of TTiffIFD
    constructor Create; override;
    destructor Destroy; override;
    procedure Clear;
    procedure LoadFromStream(aStream: TStream; AutoClear: boolean = true);
    procedure LoadHeaderFromStream(aStream: TStream);
    procedure LoadIFDsFromStream; // requires LoadHeaderFromStream, creates Images
    procedure LoadImageFromStream(Index: integer); // requires LoadIFDsFromStream
    {$ifdef FPC_Debug_Image}
    property Debug: boolean read FDebug write FDebug;
    {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
    procedure ReadImage(Index: integer);
    function FixEndian(w: Word): Word; inline;
    function FixEndian(d: DWord): DWord; inline;
    procedure DecompressPackBits(var Buffer: Pointer; var Count: PtrInt);
    procedure DecompressLZW(var Buffer: Pointer; var Count: PtrInt);
  protected
    procedure InternalRead(Str: TStream; AnImage: TFPCustomImage); override;
    function InternalCheck(Str: TStream): boolean; override;
  public
    FirstImg: TTiffIDF;
    constructor Create; override;
    destructor Destroy; override;
    procedure Clear;
    procedure LoadFromStream(aStream: TStream);
    property Debug: boolean read FDebug write FDebug;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property StartPos: int64 read fStartPos;
    property ReverserEndian: boolean read FReverserEndian;
    property TheStream: TStream read s;
    property OnCreateImage: TTiffCreateCompatibleImgEvent read FOnCreateImage
                                                          write FOnCreateImage;
<<<<<<< HEAD
<<<<<<< HEAD
    property CheckIFDOrder: TTiffCheckIFDOrder read FCheckIFDOrder write FCheckIFDOrder;
    function FirstImg: TTiffIFD;
    function GetBiggestImage: TTiffIFD;
    function ImageCount: integer;
    property Images[Index: integer]: TTiffIFD read GetImages; default;
    property FirstIFDStart: DWord read FFirstIFDStart;
  end;

procedure DecompressPackBits(Buffer: Pointer; Count: PtrInt;
  out NewBuffer: Pointer; out NewCount: PtrInt);
procedure DecompressLZW(Buffer: Pointer; Count: PtrInt;
  out NewBuffer: PByte; out NewCount: PtrInt);
function DecompressDeflate(Compressed: PByte; CompressedCount: cardinal;
  out Decompressed: PByte; var DecompressedCount: cardinal;
  ErrorMsg: PAnsiString = nil): boolean;

implementation

function CMYKToFPColor(C,M,Y,K: Word): TFPColor;
var R, G, B : LongWord;
begin
   R := $ffff - ((LongWord(C)*($ffff-LongWord(K))) shr 16) - LongWord(K) ;
   G := $ffff - ((LongWord(M)*($ffff-LongWord(K))) shr 16) - LongWord(K) ;
   B := $ffff - ((LongWord(Y)*($ffff-LongWord(K))) shr 16) - LongWord(K) ;
   Result := FPColor(R and $ffff,G and $ffff,B and $ffff);
end ;

=======
  end;

implementation

>>>>>>> graemeg/fixes_2_2
=======
  end;

implementation

>>>>>>> origin/fixes_2_2
procedure TFPReaderTiff.TiffError(Msg: string);
begin
  Msg:=Msg+' at position '+IntToStr(s.Position);
  if fStartPos>0 then
<<<<<<< HEAD
<<<<<<< HEAD
    Msg:=Msg+' (TiffPosition='+IntToStr(fStartPos)+')';
  raise Exception.Create(Msg);
end;

function TFPReaderTiff.GetImages(Index: integer): TTiffIFD;
begin
  Result:=TTiffIFD(ImageList[Index]);
end;

procedure TFPReaderTiff.ReadImageProperties(out RedBits, GreenBits, BlueBits,
  GrayBits, AlphaBits: Word; out ExtraSamples: PWord; out
  ExtraSampleCnt: DWord; out SampleBits: PWord; out SampleBitsPerPixel: DWord);
var
  BytesPerPixel: Word;
  SampleCnt: DWord;
  i: Integer;
begin
  ReadShortValues(IFD.BitsPerSample, SampleBits, SampleCnt);
  if SampleCnt<>IFD.SamplesPerPixel then
    TiffError('Samples='+IntToStr(SampleCnt)+' <> SamplesPerPixel='+IntToStr(IFD
      .SamplesPerPixel));
  if IFD.ExtraSamples>0 then
    ReadShortValues(IFD.ExtraSamples, ExtraSamples, ExtraSampleCnt);
  if ExtraSampleCnt>=SampleCnt then
    TiffError('Samples='+IntToStr(SampleCnt)+' ExtraSampleCnt='+IntToStr(
      ExtraSampleCnt));

  case IFD.PhotoMetricInterpretation of
  0, 1: if SampleCnt-ExtraSampleCnt<>1 then
    TiffError('gray images expect one sample per pixel, but found '+IntToStr(
      SampleCnt));
  2: if (SampleCnt-ExtraSampleCnt<>3) and (SampleCnt-ExtraSampleCnt<>4) then
    TiffError('rgb(a) images expect three or four samples per pixel, but found '+IntToStr(
      SampleCnt));
  3: if SampleCnt-ExtraSampleCnt<>1 then
    TiffError('palette images expect one sample per pixel, but found '+IntToStr(
      SampleCnt));
  4: if SampleCnt-ExtraSampleCnt<>1 then
    TiffError('mask images expect one sample per pixel, but found '+IntToStr(
      SampleCnt));
  5: if SampleCnt-ExtraSampleCnt<>4 then
    TiffError('cmyk images expect four samples per pixel, but found '+IntToStr(
      SampleCnt));
  end;

  GrayBits:=0;
  RedBits:=0;
  GreenBits:=0;
  BlueBits:=0;
  AlphaBits:=0;
  BytesPerPixel:=0;
  SampleBitsPerPixel:=0;
  for i:=0 to SampleCnt-1 do begin
    if SampleBits[i]>64 then
      TiffError('Samples bigger than 64 bit not supported');
    if not (SampleBits[i] in [1, 8, 12, 16]) then
      TiffError('Only samples of 1, 8, 12 and 16 bit are supported');
    inc(SampleBitsPerPixel, SampleBits[i]);
  end;
  case IFD.PhotoMetricInterpretation of
  0, 1:
    begin
      GrayBits:=SampleBits[0];
      IFD.GrayBits:=GrayBits;
      for i:=0 to ExtraSampleCnt-1 do begin
        if ExtraSamples[i] in [1, 2] then begin
          AlphaBits:=SampleBits[1+i];
          IFD.AlphaBits:=AlphaBits;
        end;
      end;
      if not (GrayBits in [1, 8, 12, 16]) then
        TiffError('gray image only supported with gray BitsPerSample 1, 8, 12 or 16');
      if not (AlphaBits in [0, 8, 16]) then
        TiffError('gray image only supported with alpha BitsPerSample 8 or 16');
    end;
  2:
    begin
      RedBits:=SampleBits[0];
      GreenBits:=SampleBits[1];
      BlueBits:=SampleBits[2];
      if SampleCnt=4 then
        AlphaBits:=SampleBits[3];
      IFD.RedBits:=RedBits;
      IFD.GreenBits:=GreenBits;
      IFD.BlueBits:=BlueBits;
      if SampleCnt=4 then
        IFD.AlphaBits:=AlphaBits
      else
        IFD.AlphaBits:=0;
      for i:=0 to ExtraSampleCnt-1 do begin
        //writeln('  ',i,'/',ExtraSampleCnt,' Type=',ExtraSamples[i],' Count=',SampleBits[3+i]);
        if ExtraSamples[i] in [1, 2] then begin
          AlphaBits:=SampleBits[3+i];
          IFD.AlphaBits:=AlphaBits;
        end;
      end;
      if not (RedBits in [8, 16]) then
        TiffError('RGB image only supported with red BitsPerSample 8 or 16');
      if not (GreenBits in [8, 16]) then
        TiffError('RGB image only supported with green BitsPerSample 8 or 16');
      if not (BlueBits in [8, 16]) then
        TiffError('RGB image only supported with blue BitsPerSample 8 or 16');
      if not (AlphaBits in [0, 8, 16]) then
        TiffError('RGB image only supported with alpha BitsPerSample 8 or 16');
    end;
  5:
    begin
      RedBits:=SampleBits[0];
      GreenBits:=SampleBits[1];
      BlueBits:=SampleBits[2];
      GrayBits:=SampleBits[3];
      IFD.RedBits:=RedBits;
      IFD.GreenBits:=GreenBits;
      IFD.BlueBits:=BlueBits;
      IFD.GrayBits:=GrayBits;
      IFD.AlphaBits:=0;
      for i:=0 to ExtraSampleCnt-1 do begin
        if ExtraSamples[i] in [1, 2] then begin
          AlphaBits:=SampleBits[4+i];
          IFD.AlphaBits:=AlphaBits;
        end;
      end;
      if not (RedBits in [8, 16]) then
        TiffError('CMYK image only supported with cyan BitsPerSample 8 or 16');
      if not (GreenBits in [8, 16]) then
        TiffError('CMYK image only supported with magenta BitsPerSample 8 or 16'
          );
      if not (BlueBits in [8, 16]) then
        TiffError('CMYK image only supported with yellow BitsPerSample 8 or 16'
          );
      if not (GrayBits in [8, 16]) then
        TiffError('CMYK image only supported with black BitsPerSample 8 or 16');
      if not (AlphaBits in [0, 8, 16]) then
        TiffError('CMYK image only supported with alpha BitsPerSample 8 or 16');
    end;
  end;
  BytesPerPixel:=(GrayBits+RedBits+GreenBits+BlueBits+AlphaBits) div 8;
  IFD.BytesPerPixel:=BytesPerPixel;
  {$ifdef FPC_Debug_Image}
  if Debug then
    writeln('BytesPerPixel=', BytesPerPixel);
  {$endif}

  if not (IFD.FillOrder in [0, 1]) then
    TiffError('FillOrder unsupported: '+IntToStr(IFD.FillOrder));
end;

procedure TFPReaderTiff.SetFPImgExtras(CurImg: TFPCustomImage);
begin
  ClearTiffExtras(CurImg);
  // set Tiff extra attributes
  CurImg.Extra[TiffPhotoMetric]:=IntToStr(IFD.PhotoMetricInterpretation);
  //writeln('TFPReaderTiff.SetFPImgExtras PhotoMetric=',CurImg.Extra[TiffPhotoMetric]);
  if IFD.Artist<>'' then
    CurImg.Extra[TiffArtist]:=IFD.Artist;
  if IFD.Copyright<>'' then
    CurImg.Extra[TiffCopyright]:=IFD.Copyright;
  if IFD.DocumentName<>'' then
    CurImg.Extra[TiffDocumentName]:=IFD.DocumentName;
  if IFD.DateAndTime<>'' then
    CurImg.Extra[TiffDateTime]:=IFD.DateAndTime;
  if IFD.HostComputer<>'' then
    CurImg.Extra[TiffHostComputer]:=IFD.HostComputer;
  if IFD.ImageDescription<>'' then
    CurImg.Extra[TiffImageDescription]:=IFD.ImageDescription;
  if IFD.Make_ScannerManufacturer<>'' then
    CurImg.Extra[TiffMake_ScannerManufacturer]:=IFD.Make_ScannerManufacturer;
  if IFD.Model_Scanner<>'' then
    CurImg.Extra[TiffModel_Scanner]:=IFD.Model_Scanner;
  if IFD.Software<>'' then
    CurImg.Extra[TiffSoftware]:=IFD.Software;
  if not (IFD.Orientation in [1..8]) then
    IFD.Orientation:=1;
  CurImg.Extra[TiffOrientation]:=IntToStr(IFD.Orientation);
  if IFD.ResolutionUnit<>0 then
    CurImg.Extra[TiffResolutionUnit]:=IntToStr(IFD.ResolutionUnit);
  if (IFD.XResolution.Numerator<>0) or (IFD.XResolution.Denominator<>0) then
    CurImg.Extra[TiffXResolution]:=TiffRationalToStr(IFD.XResolution);
  if (IFD.YResolution.Numerator<>0) or (IFD.YResolution.Denominator<>0) then
    CurImg.Extra[TiffYResolution]:=TiffRationalToStr(IFD.YResolution);
  CurImg.Extra[TiffRedBits]:=IntToStr(IFD.RedBits);
  CurImg.Extra[TiffGreenBits]:=IntToStr(IFD.GreenBits);
  CurImg.Extra[TiffBlueBits]:=IntToStr(IFD.BlueBits);
  CurImg.Extra[TiffGrayBits]:=IntToStr(IFD.GrayBits);
  CurImg.Extra[TiffAlphaBits]:=IntToStr(IFD.AlphaBits);
  if IFD.PageCount>0 then begin
    CurImg.Extra[TiffPageNumber]:=IntToStr(IFD.PageNumber);
    CurImg.Extra[TiffPageCount]:=IntToStr(IFD.PageCount);
  end;
  if IFD.PageName<>'' then
    CurImg.Extra[TiffPageName]:=IFD.PageName;
  if IFD.ImageIsThumbNail then
    CurImg.Extra[TiffIsThumbnail]:='1';
  if IFD.ImageIsMask then
    CurImg.Extra[TiffIsMask]:='1';
  if IFD.Compression<>TiffCompressionNone then
    CurImg.Extra[TiffCompression]:=IntToStr(IFD.Compression);

  {$ifdef FPC_Debug_Image}
  if Debug then
    WriteTiffExtras('SetFPImgExtras', CurImg);
  {$endif}
end;

procedure TFPReaderTiff.ReadImgValue(BitCount: Word; var Run: Pointer; x: dword;
  Predictor: word; var LastValue: word; out Value: Word); inline;
var
  BitNumber: byte;
  Byte1, Byte2: byte;
begin
  case BitCount of
  1:
    begin
      //Get the value of the right bit depending on x value and scale it to dword.
      BitNumber:=7-(x mod 8); //Leftmost pixel starts with bit 7
      Value:=$ffff*((PCUInt8(Run)^) and (1 shl BitNumber) shr BitNumber);
      if Predictor=2 then begin
        TiffError('predictor 2 not supported for bilevel images');
      end;
      if ((x+1) mod 8)=0 then
        inc(Run); //next byte when all bits read
    end;
  8:
    begin
      Value:=PCUInt8(Run)^;
      if Predictor=2 then begin
        // horizontal difference
        if x>0 then
          Value:=(Value+LastValue) and $ff;
        LastValue:=Value;
      end;
      Value:=Value shl 8+Value;
      inc(Run);
    end;
  12:
    begin
      Byte1 := PCUInt8(Run)^;
      Byte2 := PCUInt8(Run+1)^;
      if (x mod 2) = 0 then begin
        Value := (((Byte1) shl 4) or (Byte2 shr 4)) * 16;
        inc(Run);
      end else begin
        Value := (((Byte1 and $0F) shl 8) or Byte2) * 16;
        inc(Run, 2);
      end;
    end;
  16:
    begin
      Value:=FixEndian(PCUInt16(Run)^);
      if Predictor=2 then begin
        // horizontal difference
        if x>0 then
          Value:=(Value+LastValue) and $ffff;
        LastValue:=Value;
      end;
      inc(Run,2);
    end;
  end;
end;

=======
=======
>>>>>>> origin/fixes_2_2
    Msg:=Msg+'(TiffPosition='+IntToStr(fStartPos)+')';
  raise Exception.Create(Msg);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TFPReaderTiff.SetStreamPos(p: DWord);
var
  NewPosition: int64;
begin
  NewPosition:=Int64(p)+fStartPos;
  if NewPosition>s.Size then
    TiffError('Offset outside of stream');
  s.Position:=NewPosition;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFPReaderTiff.LoadFromStream(aStream: TStream; AutoClear: boolean);
var
  IFDStart: DWord;
  i: Integer;
  aContinue: Boolean;
begin
  if AutoClear then
    Clear;
  aContinue:=true;
  Progress(psStarting, 0, False, Rect(0,0,0,0), '', aContinue);
  if not aContinue then exit;
  LoadHeaderFromStream(aStream);
  try
    IFDStart:=FirstIFDStart;
    i:=0;
    while IFDStart>0 do begin
      if i=ImageCount then
        ImageList.Add(TTiffIFD.Create);
      IFD:=Images[i];
      IFDStart:=ReadIFD(IFDStart);
      LoadImageFromStream(i);
      inc(i);
    end;
  finally
    IFD:=nil;
  end;
  Progress(psEnding, 100, False, Rect(0,0,0,0), '', aContinue);
end;

procedure TFPReaderTiff.LoadHeaderFromStream(aStream: TStream);
begin
  FFirstIFDStart:=0;
  s:=aStream;
  fStartPos:=s.Position;
  ReadTiffHeader(false,FFirstIFDStart);
end;

procedure TFPReaderTiff.LoadIFDsFromStream;
var
  i: Integer;
  IFDStart: DWord;
begin
  try
    IFDStart:=FirstIFDStart;
    i:=0;
    while IFDStart>0 do begin
      if ImageCount=i then
        ImageList.Add(TTiffIFD.Create);
      IFD:=Images[i];
      IFDStart:=ReadIFD(IFDStart);
      inc(i);
    end;
  finally
    IFD:=nil;
  end;
end;

function TFPReaderTiff.FirstImg: TTiffIFD;
begin
  Result:=nil;
  if (ImageList=nil) or (ImageList.Count=0) then exit;
  Result:=TTiffIFD(ImageList[0]);
end;

function TFPReaderTiff.GetBiggestImage: TTiffIFD;
var
  Size: Int64;
  Img: TTiffIFD;
  CurSize: int64;
  i: Integer;
begin
  Result:=nil;
  Size:=0;
  for i:=0 to ImageCount-1 do begin
    Img:=Images[i];
    CurSize:=Int64(Img.ImageWidth)*Img.ImageHeight;
    if CurSize<Size then continue;
    Size:=CurSize;
    Result:=Img;
  end;
end;

function TFPReaderTiff.ImageCount: integer;
begin
  Result:=ImageList.Count;
end;

function TFPReaderTiff.ReadTiffHeader(QuickTest: boolean; out IFDStart: DWord): boolean;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFPReaderTiff.LoadFromStream(aStream: TStream);
var
  IFDStart: LongWord;
  i: Integer;
  aContinue: Boolean;
begin
  Clear;
  aContinue:=true;
  Progress(psStarting, 0, False, Rect(0,0,0,0), '', aContinue);
  if not aContinue then exit;
  s:=aStream;
  fStartPos:=s.Position;
  ReadTiffHeader(false,IFDStart);
  i:=0;
  while IFDStart>0 do begin
    IFDStart:=ReadIFD(IFDStart);
    ReadImage(i);
    inc(i);
  end;
  Progress(psEnding, 100, False, Rect(0,0,0,0), '', aContinue);
end;

function TFPReaderTiff.ReadTiffHeader(QuickTest: boolean; out IFD: DWord): boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  ByteOrder: String;
  BigEndian: Boolean;
  FortyTwo: Word;
begin
  Result:=false;
  // read byte order  II low endian, MM big endian
  ByteOrder:='  ';
  s.Read(ByteOrder[1],2);
  //debugln(['TForm1.ReadTiffHeader ',dbgstr(ByteOrder)]);
  if ByteOrder='II' then
    BigEndian:=false
  else if ByteOrder='MM' then
    BigEndian:=true
  else if QuickTest then
    exit
  else
    TiffError('expected II or MM');
<<<<<<< HEAD
<<<<<<< HEAD
  FReverseEndian:={$ifdef FPC_BIG_ENDIAN}not{$endif} BigEndian;
  {$ifdef FPC_Debug_Image}
  if Debug then
    writeln('TFPReaderTiff.ReadTiffHeader Endian Big=',BigEndian,' ReverseEndian=',FReverseEndian);
  {$endif}
=======
  FReverseEndian:={$IFDEF FPC_BIG_ENDIAN}not{$ENDIF} BigEndian;
  if Debug then
    writeln('TFPReaderTiff.ReadTiffHeader Endian Big=',BigEndian,' ReverseEndian=',FReverseEndian);
>>>>>>> graemeg/fixes_2_2
=======
  FReverseEndian:={$IFDEF FPC_BIG_ENDIAN}not{$ENDIF} BigEndian;
  if Debug then
    writeln('TFPReaderTiff.ReadTiffHeader Endian Big=',BigEndian,' ReverseEndian=',FReverseEndian);
>>>>>>> origin/fixes_2_2
  // read magic number 42
  FortyTwo:=ReadWord;
  if FortyTwo<>42 then begin
    if QuickTest then
      exit
    else
      TiffError('expected 42, because of its deep philosophical impact, but found '+IntToStr(FortyTwo));
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  // read offset to first IFD
  IFDStart:=ReadDWord;
=======
  // read offset to first IDF
  IFD:=ReadDWord;
>>>>>>> graemeg/fixes_2_2
=======
  // read offset to first IDF
  IFD:=ReadDWord;
>>>>>>> origin/fixes_2_2
  //debugln(['TForm1.ReadTiffHeader IFD=',IFD]);
  Result:=true;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TFPReaderTiff.ReadIFD(Start: DWord): DWord;
=======
function TFPReaderTiff.ReadIFD(Start: dword): DWord;
>>>>>>> graemeg/fixes_2_2
=======
function TFPReaderTiff.ReadIFD(Start: dword): DWord;
>>>>>>> origin/fixes_2_2
var
  Count: Word;
  i: Integer;
  EntryTag: Word;
  p: Int64;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef FPC_Debug_Image}
  if Debug then
    writeln('ReadIFD Start=',Start);
  {$endif}
  // set default values if not read from file
  IFD.RowsPerStrip := $FFFFFFFF;
  
  Result:=0;
  SetStreamPos(Start);
  IFD.IFDStart:=Start;
=======
  Result:=0;
  SetStreamPos(Start);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=0;
  SetStreamPos(Start);
>>>>>>> origin/fixes_2_2
  Count:=ReadWord;
  EntryTag:=0;
  p:=s.Position;
  for i:=1 to Count do begin
    ReadDirectoryEntry(EntryTag);
    inc(p,12);
    s.Position:=p;
  end;
  // read start of next IFD
  Result:=ReadDWord;
<<<<<<< HEAD
<<<<<<< HEAD
  IFD.IFDNext:=Result;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  if (Result<>0) and (Result<Start) then begin
    // backward jump: check for loops
    if fIFDStarts=nil then
      fIFDStarts:=TFPList.Create
<<<<<<< HEAD
<<<<<<< HEAD
    else if fIFDStarts.IndexOf({%H-}Pointer(PtrUInt(Result)))>0 then
      TiffError('endless loop in Image File Descriptors');
    fIFDStarts.Add({%H-}Pointer(PtrUInt(Result)));
=======
    else if fIFDStarts.IndexOf(Pointer(PtrUInt(Result)))>0 then
      TiffError('endless loop in Image File Descriptors');
    fIFDStarts.Add(Pointer(PtrUInt(Result)));
>>>>>>> graemeg/fixes_2_2
=======
    else if fIFDStarts.IndexOf(Pointer(PtrUInt(Result)))>0 then
      TiffError('endless loop in Image File Descriptors');
    fIFDStarts.Add(Pointer(PtrUInt(Result)));
>>>>>>> origin/fixes_2_2
  end;
end;

procedure TFPReaderTiff.ReadDirectoryEntry(var EntryTag: Word);
var
  EntryType: Word;
<<<<<<< HEAD
<<<<<<< HEAD
  EntryCount: DWord;
  EntryStart: DWord;
  NewEntryTag: Word;
  UValue: DWord;
=======
=======
>>>>>>> origin/fixes_2_2
  EntryCount: LongWord;
  EntryStart: LongWord;
  NewEntryTag: Word;
  UValue: LongWord;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SValue: integer;
  WordBuffer: PWord;
  Count: DWord;
  i: Integer;
<<<<<<< HEAD
<<<<<<< HEAD

  function GetPos: DWord;
  begin
     Result:=DWord(s.Position-fStartPos-2)
  end;

begin
  NewEntryTag:=ReadWord;
  if (NewEntryTag<EntryTag) then begin
    // the TIFF specification insists on ordered entry tags in each IFD
    // This allows to spot damaged files.
    // But some programs like 'GraphicConverter' do not order the extension tags
    // properly.
    {$ifdef FPC_Debug_Image}
    if Debug then
      writeln('WARNING: Tags must be in ascending order: Last='+IntToStr(EntryTag)+' Next='+IntToStr(NewEntryTag));
    {$endif}
    case CheckIFDOrder of
    tcioAlways: TiffError('Tags must be in ascending order: Last='+IntToStr(EntryTag)+' Next='+IntToStr(NewEntryTag));
    tcioSmart:
      if NewEntryTag<30000 then
        TiffError('Tags must be in ascending order: Last='+IntToStr(EntryTag)+' Next='+IntToStr(NewEntryTag));
    end;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
begin
  NewEntryTag:=ReadWord;
  if NewEntryTag<EntryTag then
    TiffError('Tags must be in ascending order');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  EntryTag:=NewEntryTag;
  case EntryTag of
  254:
    begin
      // NewSubFileType
      UValue:=ReadEntryUnsigned;
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.ImageIsThumbNail:=UValue and 1<>0;
      IFD.ImageIsPage:=UValue and 2<>0;
      IFD.ImageIsMask:=UValue and 4<>0;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 254: NewSubFileType ThumbNail=',IFD.ImageIsThumbNail,' Page=',IFD.ImageIsPage,' Mask=',IFD.ImageIsMask);
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.ImageIsThumbNail:=UValue and 1<>0;
      IDF.ImageIsPage:=UValue and 2<>0;
      IDF.ImageIsMask:=UValue and 4<>0;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry NewSubFileType ThumbNail=',IDF.ImageIsThumbNail,' Page=',IDF.ImageIsPage,' Mask=',IDF.ImageIsMask);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  255:
    begin
      // SubFileType (deprecated)
      UValue:=ReadEntryUnsigned;
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.ImageIsThumbNail:=false;
      IFD.ImageIsPage:=false;
      IFD.ImageIsMask:=false;
      case UValue of
      1: ;
      2: IFD.ImageIsThumbNail:=true;
      3: IFD.ImageIsPage:=true;
      else
        TiffError('SubFileType expected, but found '+IntToStr(UValue));
      end;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 255: SubFileType ThumbNail=',IFD.ImageIsThumbNail,' Page=',IFD.ImageIsPage,' Mask=',IFD.ImageIsMask);
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.ImageIsThumbNail:=false;
      IDF.ImageIsPage:=false;
      IDF.ImageIsMask:=false;
      case UValue of
      1: ;
      2: IDF.ImageIsThumbNail:=true;
      3: IDF.ImageIsPage:=true;
      else
        TiffError('SubFileType expected, but found '+IntToStr(UValue));
      end;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry SubFileType ThumbNail=',IDF.ImageIsThumbNail,' Page=',IDF.ImageIsPage,' Mask=',IDF.ImageIsMask);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  256:
    begin
      // fImageWidth
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.ImageWidth:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 256: ImageWidth=',IFD.ImageWidth);
      {$endif}
    end;
  257:
    begin
      // ImageLength according to TIFF spec, here used as imageheight
      IFD.ImageHeight:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 257: ImageHeight=',IFD.ImageHeight);
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.ImageWidth:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry ImageWidth=',IDF.ImageWidth);
    end;
  257:
    begin
      // ImageLength
      IDF.ImageHeight:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry ImageHeight=',IDF.ImageHeight);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  258:
    begin
      // BitsPerSample
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.BitsPerSample:=GetPos;
      ReadShortValues(IFD.BitsPerSample,WordBuffer,Count);
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Tag 258: BitsPerSample: ');
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.BitsPerSample:=DWord(s.Position-fStartPos-2);
      ReadShortValues(IDF.BitsPerSample,WordBuffer,Count);
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry BitsPerSample: ');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        for i:=0 to Count-1 do
          write(IntToStr(WordBuffer[i]),' ');
        writeln;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      {$endif}
      try
        SetLength(IFD.BitsPerSampleArray,Count);
        for i:=0 to Count-1 do
          IFD.BitsPerSampleArray[i]:=WordBuffer[i];
=======
=======
>>>>>>> origin/fixes_2_2
      try
        SetLength(IDF.BitsPerSampleArray,Count);
        for i:=0 to Count-1 do
          IDF.BitsPerSampleArray[i]:=WordBuffer[i];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      finally
        ReAllocMem(WordBuffer,0);
      end;
    end;
  259:
    begin
<<<<<<< HEAD
<<<<<<< HEAD
      // Compression
      UValue:=ReadEntryUnsigned;
      case UValue of
      TiffCompressionNone,
      TiffCompressionCCITTRLE,
      TiffCompressionCCITTFAX3,
      TiffCompressionCCITTFAX4,
      TiffCompressionLZW,
      TiffCompressionOldJPEG,
      TiffCompressionJPEG,
      TiffCompressionDeflateAdobe,
      TiffCompressionJBIGBW,
      TiffCompressionJBIGCol,
      TiffCompressionNeXT,
      TiffCompressionCCITTRLEW,
      TiffCompressionPackBits,
      TiffCompressionThunderScan,
      TiffCompressionIT8CTPAD,
      TiffCompressionIT8LW,
      TiffCompressionIT8MP,
      TiffCompressionIT8BL,
      TiffCompressionPixarFilm,
      TiffCompressionPixarLog,
      TiffCompressionDeflateZLib,
      TiffCompressionDCS,
      TiffCompressionJBIG,
      TiffCompressionSGILog,
      TiffCompressionSGILog24,
      TiffCompressionJPEG2000: ;
      else
        TiffError('expected Compression, but found '+IntToStr(UValue));
      end;
      IFD.Compression:=UValue;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 259: Compression=',IntToStr(IFD.Compression),'=',TiffCompressionName(IFD.Compression));
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      // fCompression
      UValue:=ReadEntryUnsigned;
      case UValue of
      1: ; { No fCompression, but pack data into bytes as tightly as possible,
           leaving no unused bits (except at the end of a row). The component
           values are stored as an array of type BYTE. Each scan line (row)
           is padded to the next BYTE boundary. }
      2: ; { CCITT Group 3 1-Dimensional Modified Huffman run length encoding. }
      5: ; { LZW }
      7: ; { JPEG }
      32946: ; { Deflate }
      32773: ; { PackBits fCompression, a simple byte-oriented run length scheme.
               See the PackBits section for details. Data fCompression applies
               only to raster image data. All other TIFF fields are unaffected. }
      else
        TiffError('expected Compression, but found '+IntToStr(UValue));
      end;
      IDF.Compression:=UValue;
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Compression=',IntToStr(IDF.Compression),'=');
        case IDF.Compression of
        1: write('no compression');
        2: write('CCITT Group 3 1-Dimensional Modified Huffman run length encoding');
        5: write('LZW');
        7: write('JPEG');
        32946: write('Deflate');
        32773: write('PackBits');
        end;
        writeln;
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  262:
    begin
      // PhotometricInterpretation
      UValue:=ReadEntryUnsigned;
      case UValue of
      0: ; // bilevel grayscale 0 is white
      1: ; // bilevel grayscale 0 is black
      2: ; // RGB 0,0,0 is black
      3: ; // Palette color
      4: ; // Transparency Mask
      5: ; // CMYK
      else
        TiffError('expected PhotometricInterpretation, but found '+IntToStr(UValue));
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.PhotoMetricInterpretation:=UValue;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Tag 262: PhotometricInterpretation=');
        case IFD.PhotoMetricInterpretation of
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.PhotoMetricInterpretation:=UValue;
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry PhotometricInterpretation=');
        case IDF.PhotoMetricInterpretation of
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        0: write('0=bilevel grayscale 0 is white');
        1: write('1=bilevel grayscale 0 is black');
        2: write('2=RGB 0,0,0 is black');
        3: write('3=Palette color');
        4: write('4=Transparency Mask');
        5: write('5=CMYK 8bit');
        end;
        writeln;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      {$endif}
    end;
  263:
    begin
      // Tresholding
=======
=======
>>>>>>> origin/fixes_2_2
    end;
  263:
    begin
      // Treshholding
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      UValue:=ReadEntryUnsigned;
      case UValue of
      1: ; // no dithering or halftoning was applied
      2: ; // an ordered dithering or halftoning was applied
      3: ; // a randomized dithering or halftoning was applied
      else
<<<<<<< HEAD
<<<<<<< HEAD
        TiffError('expected Tresholding, but found '+IntToStr(UValue));
      end;
      IFD.Tresholding:=UValue;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 263: Tresholding=',IFD.Tresholding);
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
        TiffError('expected Treshholding, but found '+IntToStr(UValue));
      end;
      IDF.Treshholding:=UValue;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Treshholding=',IDF.Treshholding);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  264:
    begin
      // CellWidth
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.CellWidth:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 264: CellWidth=',IFD.CellWidth);
      {$endif}
=======
      IDF.CellWidth:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry CellWidth=',IDF.CellWidth);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.CellWidth:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry CellWidth=',IDF.CellWidth);
>>>>>>> origin/fixes_2_2
    end;
  265:
    begin
      // CellLength
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.CellLength:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 265: CellLength=',IFD.CellLength);
      {$endif}
=======
      IDF.CellLength:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry CellLength=',IDF.CellLength);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.CellLength:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry CellLength=',IDF.CellLength);
>>>>>>> origin/fixes_2_2
    end;
  266:
    begin
      // FillOrder
      UValue:=ReadEntryUnsigned;
      case UValue of
<<<<<<< HEAD
<<<<<<< HEAD
      1: IFD.FillOrder:=1; // left to right = high to low
      2: IFD.FillOrder:=2; // left to right = low to high
      else
        TiffError('expected FillOrder, but found '+IntToStr(UValue));
      end;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Tag 266: FillOrder=',IntToStr(IFD.FillOrder),'=');
        case IFD.FillOrder of
=======
=======
>>>>>>> origin/fixes_2_2
      1: IDF.FillOrder:=1; // left to right = high to low
      2: IDF.FillOrder:=2; // left to right = low to high
      else
        TiffError('expected FillOrder, but found '+IntToStr(UValue));
      end;
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry FillOrder=',IntToStr(IDF.FillOrder),'=');
        case IDF.FillOrder of
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        1: write('left to right = high to low');
        2: write('left to right = low to high');
        end;
        writeln;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  269:
    begin
      // DocumentName
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.DocumentName:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 269: DocumentName=',IFD.DocumentName);
      {$endif}
=======
      IDF.DocumentName:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry DocumentName=',IDF.DocumentName);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.DocumentName:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry DocumentName=',IDF.DocumentName);
>>>>>>> origin/fixes_2_2
    end;
  270:
    begin
      // ImageDescription
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.ImageDescription:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 270: ImageDescription=',IFD.ImageDescription);
      {$endif}
=======
      IDF.ImageDescription:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry ImageDescription=',IDF.ImageDescription);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.ImageDescription:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry ImageDescription=',IDF.ImageDescription);
>>>>>>> origin/fixes_2_2
    end;
  271:
    begin
      // Make - scanner manufacturer
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.Make_ScannerManufacturer:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 271: Make_ScannerManufacturer=',IFD.Make_ScannerManufacturer);
      {$endif}
=======
      IDF.Make_ScannerManufacturer:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Make_ScannerManufacturer=',IDF.Make_ScannerManufacturer);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.Make_ScannerManufacturer:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Make_ScannerManufacturer=',IDF.Make_ScannerManufacturer);
>>>>>>> origin/fixes_2_2
    end;
  272:
    begin
      // Model - scanner model
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.Model_Scanner:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 272: Model_Scanner=',IFD.Model_Scanner);
      {$endif}
    end;
  273:
    begin
      // StripOffsets (store offset to entity, not the actual contents of the offsets)
      IFD.StripOffsets:=GetPos; //Store position of entity so we can look up multiple offsets later
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 273: StripOffsets, offset for entry=',IFD.StripOffsets);
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.Model_Scanner:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Model_Scanner=',IDF.Model_Scanner);
    end;
  273:
    begin
      // StripOffsets
      IDF.StripOffsets:=DWord(s.Position-fStartPos-2);
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry StripOffsets=',IDF.StripOffsets);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  274:
    begin
      // Orientation
      UValue:=ReadEntryUnsigned;
      case UValue of
      1: ;// 0,0 is left, top
      2: ;// 0,0 is right, top
      3: ;// 0,0 is right, bottom
      4: ;// 0,0 is left, bottom
      5: ;// 0,0 is top, left (rotated)
      6: ;// 0,0 is top, right (rotated)
      7: ;// 0,0 is bottom, right (rotated)
      8: ;// 0,0 is bottom, left (rotated)
      else
        TiffError('expected Orientation, but found '+IntToStr(UValue));
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.Orientation:=UValue;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Tag 274: Orientation=',IntToStr(IFD.Orientation),'=');
        case IFD.Orientation of
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.Orientation:=UValue;
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Orientation=',IntToStr(IDF.Orientation),'=');
        case IDF.Orientation of
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        1: write('0,0 is left, top');
        2: write('0,0 is right, top');
        3: write('0,0 is right, bottom');
        4: write('0,0 is left, bottom');
        5: write('0,0 is top, left (rotated)');
        6: write('0,0 is top, right (rotated)');
        7: write('0,0 is bottom, right (rotated)');
        8: write('0,0 is bottom, left (rotated)');
        end;
        writeln;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  277:
    begin
      // SamplesPerPixel
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.SamplesPerPixel:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 277: SamplesPerPixel=',IFD.SamplesPerPixel);
      {$endif}
=======
      IDF.SamplesPerPixel:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry SamplesPerPixel=',IDF.SamplesPerPixel);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.SamplesPerPixel:=ReadEntryUnsigned;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry SamplesPerPixel=',IDF.SamplesPerPixel);
>>>>>>> origin/fixes_2_2
    end;
  278:
    begin
      // RowsPerStrip
      UValue:=ReadEntryUnsigned;
      if UValue=0 then
        TiffError('expected RowsPerStrip, but found '+IntToStr(UValue));
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.RowsPerStrip:=UValue;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 278: RowsPerStrip=',IFD.RowsPerStrip);
      {$endif}
    end;
  279:
    begin
      // StripByteCounts (the number of bytes in each strip).
      // We're storing the position of the tag, not the various bytecounts themselves
      IFD.StripByteCounts:=GetPos;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 279: StripByteCounts, offset for entry=',IFD.StripByteCounts);
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      IDF.RowsPerStrip:=UValue;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry RowsPerStrip=',IDF.RowsPerStrip);
    end;
  279:
    begin
      // StripByteCounts
      IDF.StripByteCounts:=DWord(s.Position-fStartPos-2);
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry StripByteCounts=',IDF.StripByteCounts);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  280:
    begin
      // MinSampleValue
<<<<<<< HEAD
<<<<<<< HEAD
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 280: skipping MinSampleValue');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  281:
    begin
      // MaxSampleValue
<<<<<<< HEAD
<<<<<<< HEAD
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 281: skipping MaxSampleValue');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  282:
    begin
      // XResolution
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.XResolution:=ReadEntryRational;
      {$ifdef FPC_Debug_Image}
      try
        if Debug then
          writeln('TFPReaderTiff.ReadDirectoryEntry Tag 282: XResolution=',IFD.XResolution.Numerator,'/',IFD.XResolution.Denominator,'=',IFD.XResolution.Numerator/IFD.XResolution.Denominator);
      except
        //ignore division by 0
      end;
      {$endif}
=======
      IDF.XResolution:=ReadEntryRational;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry XResolution=',IDF.XResolution.Numerator,',',IDF.XResolution.Denominator);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.XResolution:=ReadEntryRational;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry XResolution=',IDF.XResolution.Numerator,',',IDF.XResolution.Denominator);
>>>>>>> origin/fixes_2_2
    end;
  283:
    begin
      // YResolution
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.YResolution:=ReadEntryRational;
      {$ifdef FPC_Debug_Image}
      try
        if Debug then
          writeln('TFPReaderTiff.ReadDirectoryEntry Tag 283: YResolution=',IFD.YResolution.Numerator,'/',IFD.YResolution.Denominator,'=',IFD.YResolution.Numerator/IFD.YResolution.Denominator);
      except
        //ignore division by 0
      end;      {$endif}
=======
      IDF.YResolution:=ReadEntryRational;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry YResolution=',IDF.YResolution.Numerator,',',IDF.YResolution.Denominator);
>>>>>>> graemeg/fixes_2_2
=======
      IDF.YResolution:=ReadEntryRational;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry YResolution=',IDF.YResolution.Numerator,',',IDF.YResolution.Denominator);
>>>>>>> origin/fixes_2_2
    end;
  284:
    begin
      // PlanarConfiguration
      SValue:=ReadEntrySigned;
      case SValue of
<<<<<<< HEAD
<<<<<<< HEAD
      TiffPlanarConfigurationChunky: ; // 1
      TiffPlanarConfigurationPlanar: ; // 2
      else
        TiffError('expected PlanarConfiguration, but found '+IntToStr(SValue));
      end;
      IFD.PlanarConfiguration:=SValue;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Tag 284: PlanarConfiguration=');
        case SValue of
        TiffPlanarConfigurationChunky: write('chunky format');
        TiffPlanarConfigurationPlanar: write('planar format');
        end;
        writeln;
      end;
      {$endif}
    end;
  285:
    begin
      // PageName
      IFD.PageName:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 285: PageName="'+IFD.PageName+'"');
      {$endif}
=======
=======
>>>>>>> origin/fixes_2_2
      1: ; // chunky format
      2: ; // planar format
      else
        TiffError('expected PlanarConfiguration, but found '+IntToStr(SValue));
      end;
      IDF.PlanarConfiguration:=SValue;
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry PlanarConfiguration=');
        case SValue of
        1: write('chunky format');
        2: write('planar format');
        end;
        writeln;
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  288:
    begin
      // FreeOffsets
      // The free bytes in a tiff file are described with FreeByteCount and FreeOffsets
<<<<<<< HEAD
<<<<<<< HEAD
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 288: skipping FreeOffsets');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  289:
    begin
      // FreeByteCount
      // The free bytes in a tiff file are described with FreeByteCount and FreeOffsets
<<<<<<< HEAD
<<<<<<< HEAD
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 289: skipping FreeByteCount');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  290:
    begin
      // GrayResponseUnit
      // precision of GrayResponseCurve
<<<<<<< HEAD
<<<<<<< HEAD
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 290: skipping GrayResponseUnit');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  291:
    begin
      // GrayResponseCurve
      // the optical density for each possible pixel value
<<<<<<< HEAD
<<<<<<< HEAD
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 291: skipping GrayResponseCurve');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  296:
    begin
      // fResolutionUnit
      UValue:=ReadEntryUnsigned;
      case UValue of
<<<<<<< HEAD
<<<<<<< HEAD
      1: IFD.ResolutionUnit:=1; // none
      2: IFD.ResolutionUnit:=2; // inch
      3: IFD.ResolutionUnit:=3; // centimeter
      else
        TiffError('expected ResolutionUnit, but found '+IntToStr(UValue));
      end;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry Tag 296: ResolutionUnit=');
        case IFD.ResolutionUnit of
=======
=======
>>>>>>> origin/fixes_2_2
      1: IDF.ResolutionUnit:=1; // none
      2: IDF.ResolutionUnit:=2; // inch
      3: IDF.ResolutionUnit:=3; // centimeter
      else
        TiffError('expected ResolutionUnit, but found '+IntToStr(UValue));
      end;
      if Debug then begin
        write('TFPReaderTiff.ReadDirectoryEntry ResolutionUnit=');
        case IDF.ResolutionUnit of
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        1: write('none');
        2: write('inch');
        3: write('centimeter');
        end;
        writeln;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      {$endif}
    end;
  297:
    begin
      // page number (starting at 0) and total number of pages
      UValue:=GetPos;
      ReadShortValues(UValue,WordBuffer,Count);
      try
        if Count<>2 then begin
          {$ifdef FPC_Debug_Image}
          if Debug then begin
            write('TFPReaderTiff.ReadDirectoryEntry Tag 297: PageNumber/Count: ');
            for i:=0 to Count-1 do
              write(IntToStr(WordBuffer[i]),' ');
            writeln;
          end;
          {$endif}
          TiffError('PageNumber Count=2 expected, but found '+IntToStr(Count));
        end;
        IFD.PageNumber:=WordBuffer[0];
        IFD.PageCount:=WordBuffer[1];
        if IFD.PageNumber>=IFD.PageCount then begin
          // broken order => repair
          UValue:=IFD.PageNumber;
          IFD.PageNumber:=IFD.PageCount;
          IFD.PageCount:=UValue;
        end;
      finally
        ReAllocMem(WordBuffer,0);
      end;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 297: PageNumber=',IFD.PageNumber,'/',IFD.PageCount);
      end;
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  305:
    begin
      // Software
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.Software:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 305: Software="',IFD.Software,'"');
      {$endif}
=======
      IDF.Software:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Software="',IDF.Software,'"');
>>>>>>> graemeg/fixes_2_2
=======
      IDF.Software:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Software="',IDF.Software,'"');
>>>>>>> origin/fixes_2_2
    end;
  306:
    begin
      // DateAndTime
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.DateAndTime:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 306: DateAndTime="',IFD.DateAndTime,'"');
      {$endif}
=======
      IDF.DateAndTime:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry DateAndTime="',IDF.DateAndTime,'"');
>>>>>>> graemeg/fixes_2_2
    end;
  315:
    begin
      // Artist
<<<<<<< HEAD
      IFD.Artist:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 315: Artist="',IFD.Artist,'"');
      {$endif}
=======
      IDF.Artist:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Artist="',IDF.Artist,'"');
>>>>>>> graemeg/fixes_2_2
    end;
  316:
    begin
      // HostComputer
<<<<<<< HEAD
      IFD.HostComputer:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 316: HostComputer="',IFD.HostComputer,'"');
      {$endif}
    end;
  317:
    begin
      // Predictor
      UValue:=word(ReadEntryUnsigned);
      case UValue of
      1: ;
      2: ;
      else TiffError('expected Predictor, but found '+IntToStr(UValue));
      end;
      IFD.Predictor:=UValue;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 317: Predictor="',IFD.Predictor,'"');
      {$endif}
=======
      IDF.HostComputer:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry HostComputer="',IDF.HostComputer,'"');
>>>>>>> graemeg/fixes_2_2
    end;
  320:
    begin
      // ColorMap: N = 3*2^BitsPerSample
<<<<<<< HEAD
      IFD.ColorMap:=GetPos;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 320: skipping ColorMap');
      {$endif}
    end;
  322:
    begin
      // TileWidth
      IFD.TileWidth:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 322: TileWidth=',IFD.TileWidth);
      {$endif}
      if IFD.TileWidth=0 then
        TiffError('TileWidth=0');
    end;
  323:
    begin
      // TileLength = TileHeight
      IFD.TileLength:=ReadEntryUnsigned;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 323: TileLength=',IFD.TileLength);
      {$endif}
      if IFD.TileLength=0 then
        TiffError('TileLength=0');
    end;
  324:
    begin
      // TileOffsets
      IFD.TileOffsets:=GetPos;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 324: TileOffsets=',IFD.TileOffsets);
      {$endif}
      if IFD.TileOffsets=0 then
        TiffError('TileOffsets=0');
    end;
  325:
    begin
      // TileByteCounts
      IFD.TileByteCounts:=GetPos;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 325: TileByteCounts=',IFD.TileByteCounts);
      {$endif}
      if IFD.TileByteCounts=0 then
        TiffError('TileByteCounts=0');
=======
      IDF.ColorMap:=DWord(s.Position-fStartPos-2);
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry ColorMap');
>>>>>>> graemeg/fixes_2_2
    end;
  338:
    begin
      // ExtraSamples: if SamplesPerPixel is bigger than PhotometricInterpretation
      // then ExtraSamples is an array defining the extra samples
      // 0=unspecified
      // 1=alpha (premultiplied)
      // 2=alpha (unassociated)
<<<<<<< HEAD
      IFD.ExtraSamples:=GetPos;
      {$ifdef FPC_Debug_Image}
      if Debug then begin
        ReadShortValues(IFD.ExtraSamples,WordBuffer,Count);
        write('TFPReaderTiff.ReadDirectoryEntry Tag 338: ExtraSamples: ');
=======
      IDF.ExtraSamples:=DWord(s.Position-fStartPos-2);
      if Debug then begin
        ReadShortValues(IDF.ExtraSamples,WordBuffer,Count);
        write('TFPReaderTiff.ReadDirectoryEntry ExtraSamples: ');
>>>>>>> graemeg/fixes_2_2
        for i:=0 to Count-1 do
          write(IntToStr(WordBuffer[i]),' ');
        writeln;
        ReAllocMem(WordBuffer,0);
      end;
<<<<<<< HEAD
      {$endif}
    end;
  347:
    begin
      // ToDo: JPEGTables
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 347: skipping JPEG Tables');
      {$endif}
    end;
  512:
    begin
      // ToDo: JPEGProc
      // short
      // 1 = baseline sequential
      // 14 = lossless process with Huffman encoding
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 512: skipping JPEGProc');
      {$endif}
    end;
  513:
    begin
      // ToDo: JPEGInterchangeFormat
      // long
      // non zero: start of start of image SOI marker
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 513: skipping JPEGInterchangeFormat');
      {$endif}
    end;
  514:
    begin
      // ToDo: JPEGInterchangeFormatLength
      // long
      // length in bytes of 513
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 514: skipping JPEGInterchangeFormatLength');
      {$endif}
    end;
  515:
    begin
      // ToDo: JPEGRestartInterval
      // short
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 515: skipping JPEGRestartInterval');
      {$endif}
    end;
  517:
    begin
      // ToDo: JPEGLosslessPredictor
      // short
      // Count: SamplesPerPixels
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 517: skipping JPEGLosslessPredictor');
      {$endif}
    end;
  518:
    begin
      // ToDo: JPEGPointTransforms
      // short
      // Count: SamplesPerPixels
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 518: skipping JPEGPointTransforms');
      {$endif}
    end;
  519:
    begin
      // ToDo: JPEGQTables
      // long
      // Count: SamplesPerPixels
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 519: skipping JPEGQTables');
      {$endif}
    end;
  520:
    begin
      // ToDo: JPEGDCTables
      // long
      // Count: SamplesPerPixels
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 520: skipping JPEGDCTables');
      {$endif}
    end;
  521:
    begin
      // ToDo: JPEGACTables
      // long
      // Count: SamplesPerPixels
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 521: skipping JPEGACTables');
      {$endif}
    end;
  530:
    begin
      // ToDo: YCbCrSubSampling alias ChromaSubSampling
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 530: skipping YCbCrSubSampling alias ChromaSubSampling');
      {$endif}
    end;
  700:
    begin
      // ToDo: XMP
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 700: skipping XMP');
      {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
      IDF.DateAndTime:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry DateAndTime="',IDF.DateAndTime,'"');
    end;
  315:
    begin
      // Artist
      IDF.Artist:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Artist="',IDF.Artist,'"');
    end;
  316:
    begin
      // HostComputer
      IDF.HostComputer:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry HostComputer="',IDF.HostComputer,'"');
    end;
  320:
    begin
      // ColorMap: N = 3*2^BitsPerSample
      IDF.ColorMap:=DWord(s.Position-fStartPos-2);
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry ColorMap');
    end;
  338:
    begin
      // ExtraSamples: if SamplesPerPixel is bigger than PhotometricInterpretation
      // then ExtraSamples is an array defining the extra samples
      // 0=unspecified
      // 1=alpha (premultiplied)
      // 2=alpha (unassociated)
      IDF.ExtraSamples:=DWord(s.Position-fStartPos-2);
      if Debug then begin
        ReadShortValues(IDF.ExtraSamples,WordBuffer,Count);
        write('TFPReaderTiff.ReadDirectoryEntry ExtraSamples: ');
        for i:=0 to Count-1 do
          write(IntToStr(WordBuffer[i]),' ');
        writeln;
        ReAllocMem(WordBuffer,0);
      end;
>>>>>>> origin/fixes_2_2
    end;
  33432:
    begin
      // Copyright
<<<<<<< HEAD
<<<<<<< HEAD
      IFD.Copyright:=ReadEntryString;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 33432: Copyright="',IFD.Copyright,'"');
      {$endif}
    end;
  34675:
    begin
      // ToDo: ICC Profile
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag 34675: skipping ICC profile');
      {$endif}
=======
      IDF.Copyright:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Copyright="',IDF.Copyright,'"');
>>>>>>> graemeg/fixes_2_2
=======
      IDF.Copyright:=ReadEntryString;
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Copyright="',IDF.Copyright,'"');
>>>>>>> origin/fixes_2_2
    end;
  else
    begin
      EntryType:=ReadWord;
      EntryCount:=ReadDWord;
      EntryStart:=ReadDWord;
<<<<<<< HEAD
<<<<<<< HEAD
      if (EntryType=0) and (EntryCount=0) and (EntryStart=0) then ;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag=',EntryTag,' Type=',EntryType,' Count=',EntryCount,' ValuesStart=',EntryStart);
      {$endif}
=======
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag=',EntryTag,' Type=',EntryType,' Count=',EntryCount,' ValuesStart=',EntryStart);
>>>>>>> graemeg/fixes_2_2
=======
      if Debug then
        writeln('TFPReaderTiff.ReadDirectoryEntry Tag=',EntryTag,' Type=',EntryType,' Count=',EntryCount,' ValuesStart=',EntryStart);
>>>>>>> origin/fixes_2_2
    end;
  end;
end;

function TFPReaderTiff.ReadEntryUnsigned: DWord;
var
  EntryCount: LongWord;
  EntryType: Word;
begin
  Result:=0;
  EntryType:=ReadWord;
  EntryCount:=ReadDWord;
  if EntryCount<>1 then
    TiffError('EntryCount=1 expected, but found '+IntToStr(EntryCount));
  //writeln('TFPReaderTiff.ReadEntryUnsigned Tag=',EntryTag,' Type=',EntryType,' Count=',EntryCount,' ValuesStart=',EntryStart]);
  case EntryType of
  1: begin
      // byte: 8bit unsigned
      Result:=ReadByte;
    end;
  3: begin
      // short: 16bit unsigned
      Result:=ReadWord;
    end;
  4: begin
      // long: 32bit unsigned long
      Result:=ReadDWord;
    end;
  else
    TiffError('expected single unsigned value, but found type='+IntToStr(EntryType));
  end;
end;

function TFPReaderTiff.ReadEntrySigned: Cint32;
var
  EntryCount: LongWord;
  EntryType: Word;
begin
  Result:=0;
  EntryType:=ReadWord;
  EntryCount:=ReadDWord;
  if EntryCount<>1 then
    TiffError('EntryCount+1 expected, but found '+IntToStr(EntryCount));
  //writeln('TFPReaderTiff.ReadEntrySigned Tag=',EntryTag,' Type=',EntryType,' Count=',EntryCount,' ValuesStart=',EntryStart]);
  case EntryType of
  1: begin
      // byte: 8bit unsigned
      Result:=cint8(ReadByte);
    end;
  3: begin
      // short: 16bit unsigned
      Result:=cint16(ReadWord);
    end;
  4: begin
      // long: 32bit unsigned long
      Result:=cint32(ReadDWord);
    end;
  6: begin
      // sbyte: 8bit signed
      Result:=cint8(ReadByte);
    end;
  8: begin
      // sshort: 16bit signed
      Result:=cint16(ReadWord);
    end;
  9: begin
      // slong: 32bit signed long
      Result:=cint32(ReadDWord);
    end;
  else
    TiffError('expected single signed value, but found type='+IntToStr(EntryType));
  end;
end;

function TFPReaderTiff.ReadEntryRational: TTiffRational;
var
  EntryCount: LongWord;
  EntryStart: LongWord;
  EntryType: Word;
begin
  Result:=TiffRational0;
  EntryType:=ReadWord;
  EntryCount:=ReadDWord;
  if EntryCount<>1 then
    TiffError('EntryCount+1 expected, but found '+IntToStr(EntryCount));
  //writeln('TFPReaderTiff.ReadEntryUnsigned Tag=',EntryTag,' Type=',EntryType,' Count=',EntryCount,' ValuesStart=',EntryStart]);
  case EntryType of
  1: begin
      // byte: 8bit unsigned
      Result.Numerator:=ReadByte;
    end;
  3: begin
      // short: 16bit unsigned
      Result.Numerator:=ReadWord;
    end;
  4: begin
      // long: 32bit unsigned long
      Result.Numerator:=ReadDWord;
    end;
  5: begin
      // rational: Two longs: numerator + denominator
      // this does not fit into 4 bytes
      EntryStart:=ReadDWord;
      SetStreamPos(EntryStart);
      Result.Numerator:=ReadDWord;
      Result.Denominator:=ReadDWord;
    end;
  else
    TiffError('expected rational unsigned value, but found type='+IntToStr(EntryType));
  end;
end;

function TFPReaderTiff.ReadEntryString: string;
var
  EntryType: Word;
  EntryCount: LongWord;
  EntryStart: LongWord;
begin
  Result:='';
  EntryType:=ReadWord;
  if EntryType<>2 then
    TiffError('asciiz expected, but found '+IntToStr(EntryType));
  EntryCount:=ReadDWord;
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(Result,EntryCount-1);
  if EntryCount>4 then begin
    // long string -> next 4 DWord is the offset
    EntryStart:=ReadDWord;
    SetStreamPos(EntryStart);
    s.Read(Result[1],EntryCount-1);
  end else begin
    // short string -> stored directly in the next 4 bytes
    if Result<>'' then
      s.Read(Result[1],length(Result));
    // skip rest of 4 bytes
    if length(Result)<4 then
      s.Read(EntryStart,4-length(Result));
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  EntryStart:=ReadDWord;
  SetStreamPos(EntryStart);
  SetLength(Result,EntryCount-1);
  if EntryCount>1 then
    s.Read(Result[1],EntryCount-1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TFPReaderTiff.ReadByte: Byte;
begin
  Result:=s.ReadByte;
end;

function TFPReaderTiff.ReadWord: Word;
begin
  Result:=FixEndian(s.ReadWord);
end;

function TFPReaderTiff.ReadDWord: DWord;
begin
  Result:=FixEndian(s.ReadDWord);
end;

procedure TFPReaderTiff.ReadValues(StreamPos: DWord;
  out EntryType: word; out EntryCount: DWord;
  out Buffer: Pointer; out ByteCount: PtrUint);
var
  EntryStart: DWord;
begin
  Buffer:=nil;
  ByteCount:=0;
  EntryType:=0;
  EntryCount:=0;
  SetStreamPos(StreamPos);
  ReadWord; // skip tag
  EntryType:=ReadWord;
  EntryCount:=ReadDWord;
  if EntryCount=0 then exit;
  case EntryType of
  1,6,7: ByteCount:=EntryCount; // byte
  2: ByteCount:=EntryCount; // asciiz
  3,8: ByteCount:=2*EntryCount; // short
  4,9: ByteCount:=4*EntryCount; // long
  5,10: ByteCount:=8*EntryCount; // rational
  11: ByteCount:=4*EntryCount; // single
  12: ByteCount:=8*EntryCount; // double
  else
    TiffError('invalid EntryType '+IntToStr(EntryType));
  end;
  if ByteCount>4 then begin
    EntryStart:=ReadDWord;
    SetStreamPos(EntryStart);
  end;
  GetMem(Buffer,ByteCount);
  s.Read(Buffer^,ByteCount);
end;

procedure TFPReaderTiff.ReadShortOrLongValues(StreamPos: DWord; out
  Buffer: PDWord; out Count: DWord);
var
  p: Pointer;
  ByteCount: PtrUInt;
  EntryType: word;
  i: DWord;
begin
  Buffer:=nil;
  Count:=0;
  p:=nil;
  try
    ReadValues(StreamPos,EntryType,Count,p,ByteCount);
    if Count=0 then exit;
    if EntryType=3 then begin
      // short
      GetMem(Buffer,SizeOf(DWord)*Count);
      for i:=0 to Count-1 do
        Buffer[i]:=FixEndian(PWord(p)[i]);
    end else if EntryType=4 then begin
      // long
      Buffer:=p;
      p:=nil;
      if FReverseEndian then
        for i:=0 to Count-1 do
          Buffer[i]:=FixEndian(PDWord(Buffer)[i]);
    end else
      TiffError('only short or long allowed');
  finally
    if p<>nil then FreeMem(p);
  end;
end;

procedure TFPReaderTiff.ReadShortValues(StreamPos: DWord; out Buffer: PWord;
  out Count: DWord);
var
  p: Pointer;
  ByteCount: PtrUInt;
  EntryType: word;
  i: DWord;
begin
  Buffer:=nil;
  Count:=0;
  p:=nil;
  try
    ReadValues(StreamPos,EntryType,Count,p,ByteCount);
    //writeln('ReadShortValues ',FReverseEndian,' ',EntryType,' Count=',Count,' ByteCount=',ByteCount);
    if Count=0 then exit;
    if EntryType=3 then begin
      // short
      Buffer:=p;
      p:=nil;
      if FReverseEndian then
        for i:=0 to Count-1 do
          Buffer[i]:=FixEndian(Buffer[i]);
      //for i:=0 to Count-1 do writeln(i,' ',Buffer[i]);
    end else
      TiffError('only short allowed, but found '+IntToStr(EntryType));
  finally
    if p<>nil then FreeMem(p);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFPReaderTiff.LoadImageFromStream(Index: integer);
var
  ChunkOffsets: PDWord;
  ChunkByteCounts: PDWord;
  Chunk: PByte;
  ChunkCount: DWord;
  ChunkIndex: Dword;
  CurCount: DWord;
  CurOffset: DWord;
  CurByteCnt: PtrInt;
  Run: PByte;
  x, y, cx, cy, dx, dy, sx: integer;
  SampleBits: PWord;
  SampleBitsPerPixel: DWord;
  ExtraSamples: PWord;
  ExtraSampleCnt: DWord;
  GrayBits, GrayValue, LastGrayValue: Word;
  RedBits, RedValue, LastRedValue: Word;
  GreenBits, GreenValue, LastGreenValue: Word;
  BlueBits, BlueValue, LastBlueValue: Word;
  AlphaBits, AlphaValue, LastAlphaValue: Word;
  Col: TFPColor;
  i: Integer;
  CurFPImg: TFPCustomImage;
  aContinue: Boolean;
  ExpectedChunkLength: PtrInt;
  ChunkType: TTiffChunkType;
  TilesAcross, TilesDown: DWord;
  ChunkLeft, ChunkTop, ChunkWidth, ChunkHeight: DWord;
  CurImg: TTiffIFD;
  ChunkBytesPerLine: DWord;
begin
  {$ifdef FPC_Debug_Image}
  if Debug then
    writeln('TFPReaderTiff.LoadImageFromStream Index=',Index);
  {$endif}
  IFD:=Images[Index];

  if IFD.PhotoMetricInterpretation=High(IFD.PhotoMetricInterpretation) then
    TiffError('missing PhotometricInterpretation');
  if IFD.BitsPerSample=0 then
    TiffError('missing BitsPerSample');
  if IFD.TileWidth>0 then begin
    ChunkType:=tctTile;
    if IFD.TileLength=0 then
      TiffError('missing TileLength');
    if IFD.TileOffsets=0 then
      TiffError('missing TileOffsets');
    if IFD.TileByteCounts=0 then
      TiffError('missing TileByteCounts');
  end else begin
    ChunkType:=tctStrip;
    if IFD.RowsPerStrip=0 then
      TiffError('missing RowsPerStrip');
    if IFD.StripOffsets=0 then
      TiffError('missing StripOffsets');
    if IFD.StripByteCounts=0 then
      TiffError('missing StripByteCounts');
  end;

  if (IFD.ImageWidth=0) or (IFD.ImageHeight=0) then
    exit;

  if Index=ImageCount then
    ImageList.Add(TTiffIFD.Create);
  CurImg:=Images[Index];

  {$ifdef FPC_Debug_Image}
  if Debug then
    writeln('TFPReaderTiff.LoadImageFromStream reading ...');
  {$endif}

  ChunkOffsets:=nil;
  ChunkByteCounts:=nil;
  Chunk:=nil;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFPReaderTiff.ReadImage(Index: integer);
var
  StripCount: DWord;
  StripOffsets: PDWord;
  StripByteCounts: PDWord;
  StripIndex: Dword;
  SOCount: DWord;
  SBCCount: DWord;
  CurOffset: DWord;
  CurByteCnt: PtrInt;
  Strip: PByte;
  Run: Dword;
  y: DWord;
  y2: DWord;
  x: DWord;
  GrayValue: DWord;
  dx: LongInt;
  dy: LongInt;
  SampleCnt: DWord;
  SampleBits: PWord;
  ExtraSampleCnt: DWord;
  ExtraSamples: PWord;
  RedValue: Word;
  GreenValue: Word;
  BlueValue: Word;
  AlphaValue: Word;
  Col: TFPColor;
  i: Integer;
  CurImg: TFPCustomImage;
  GrayBits: Word;
  RedBits: Word;
  GreenBits: Word;
  BlueBits: Word;
  AlphaBits: Word;
  BytesPerPixel: Integer;
  aContinue: Boolean;
begin
  CurImg:=nil;
  if Debug then
    writeln('TFPReaderTiff.ReadImage Index=',Index);
  if IDF.PhotoMetricInterpretation=High(IDF.PhotoMetricInterpretation) then
    TiffError('missing PhotometricInterpretation');
  if IDF.RowsPerStrip=0 then
    TiffError('missing RowsPerStrip');
  if IDF.BitsPerSample=0 then
    TiffError('missing BitsPerSample');
  if (IDF.ImageWidth=0) or (IDF.ImageHeight=0) then begin
    exit;
  end;

  if (Index>0) and (not FirstImg.ImageIsThumbNail) then begin
    // Image already read
    exit;
  end;
  if Debug then
    writeln('TFPReaderTiff.ReadImage reading ...');

  StripCount:=((IDF.ImageHeight-1) div IDF.RowsPerStrip)+1;
  StripOffsets:=nil;
  StripByteCounts:=nil;
  Strip:=nil;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ExtraSamples:=nil;
  SampleBits:=nil;
  ExtraSampleCnt:=0;
  try
<<<<<<< HEAD
<<<<<<< HEAD
    // read chunk starts and sizes
    if ChunkType=tctTile then begin
      TilesAcross:=(IFD.ImageWidth+IFD.TileWidth-1) div IFD.TileWidth;
      TilesDown:=(IFD.ImageHeight+IFD.TileLength-1) div IFD.TileLength;
      {$ifdef FPC_Debug_Image}
      if Debug then
        writeln('TFPReaderTiff.LoadImageFromStream TilesAcross=',TilesAcross,' TilesDown=',TilesDown);
      {$endif}
      ChunkCount := TilesAcross * TilesDown;
      ReadShortOrLongValues(IFD.TileOffsets,ChunkOffsets,CurCount);
      if CurCount<>ChunkCount then
        TiffError('number of TileCounts is wrong');
      ReadShortOrLongValues(IFD.TileByteCounts,ChunkByteCounts,CurCount);
      if CurCount<>ChunkCount then
        TiffError('number of TileByteCounts is wrong');
    end else begin //strip
      ChunkCount:=((IFD.ImageHeight-1) div IFD.RowsPerStrip)+1;
      ReadShortOrLongValues(IFD.StripOffsets,ChunkOffsets,CurCount);
      if CurCount<>ChunkCount then
        TiffError('number of StripCounts is wrong');
      ReadShortOrLongValues(IFD.StripByteCounts,ChunkByteCounts,CurCount);
      if CurCount<>ChunkCount then
        TiffError('number of StripByteCounts is wrong');
    end;

    // read image structure
    ReadImageProperties(RedBits, GreenBits, BlueBits, GrayBits, AlphaBits,
      ExtraSamples, ExtraSampleCnt, SampleBits, SampleBitsPerPixel);
    CurImg.Assign(IFD);

    // create FPimage
    DoCreateImage(CurImg);
    CurFPImg:=CurImg.Img;
    if CurFPImg=nil then exit;

    SetFPImgExtras(CurFPImg);

    case IFD.Orientation of
    0,1..4: CurFPImg.SetSize(IFD.ImageWidth,IFD.ImageHeight);
    5..8: CurFPImg.SetSize(IFD.ImageHeight,IFD.ImageWidth);
    end;

    {$ifdef FPC_Debug_Image}
    if Debug then
      writeln('TFPReaderTiff.LoadImageFromStream SampleBitsPerPixel=',SampleBitsPerPixel);
    {$endif}

    // read chunks
    for ChunkIndex:=0 to ChunkCount-1 do begin
      CurOffset:=ChunkOffsets[ChunkIndex];
      CurByteCnt:=ChunkByteCounts[ChunkIndex];
      //writeln('TFPReaderTiff.LoadImageFromStream CurOffset=',CurOffset,' CurByteCnt=',CurByteCnt);
      if CurByteCnt<=0 then continue;
      ReAllocMem(Chunk,CurByteCnt);
      SetStreamPos(CurOffset);
      s.Read(Chunk^,CurByteCnt);

      // decompress
      if ChunkType=tctTile then
        ExpectedChunkLength:=(SampleBitsPerPixel*IFD.TileWidth+7) div 8*IFD.TileLength
      else
        ExpectedChunkLength:=((SampleBitsPerPixel*IFD.ImageWidth+7) div 8)*IFD.RowsPerStrip;
      case IFD.Compression of
      TiffCompressionNone: ;
      TiffCompressionPackBits: DecodePackBits(Chunk,CurByteCnt);
      TiffCompressionLZW: DecodeLZW(Chunk,CurByteCnt);
      TiffCompressionDeflateAdobe,
      TiffCompressionDeflateZLib: DecodeDeflate(Chunk,CurByteCnt,ExpectedChunkLength);
      else
        TiffError('compression '+TiffCompressionName(IFD.Compression)+' not supported yet');
      end;
      if CurByteCnt<=0 then continue;

      // compute current chunk area
      if ChunkType=tctTile then begin
        ChunkLeft:=(ChunkIndex mod TilesAcross)*IFD.TileWidth;
        ChunkTop:=(ChunkIndex div TilesAcross)*IFD.TileLength;
        ChunkWidth:=Min(IFD.TileWidth,IFD.ImageWidth-ChunkLeft);
        ChunkHeight:=Min(IFD.TileLength,IFD.ImageHeight-ChunkTop);
        ChunkBytesPerLine:=(SampleBitsPerPixel*ChunkWidth+7) div 8;
        ExpectedChunkLength:=ChunkBytesPerLine*ChunkHeight;
        if CurByteCnt<ExpectedChunkLength then begin
          //writeln('TFPReaderTiff.LoadImageFromStream SampleBitsPerPixel=',SampleBitsPerPixel,' IFD.ImageWidth=',IFD.ImageWidth,' IFD.ImageHeight=',IFD.ImageHeight,' y=',y,' IFD.TileWidth=',IFD.TileWidth,' IFD.TileLength=',IFD.TileLength,' ExpectedChunkLength=',ExpectedChunkLength,' CurByteCnt=',CurByteCnt);
          TiffError('TFPReaderTiff.LoadImageFromStream Tile too short ByteCnt='+IntToStr(CurByteCnt)+' ChunkWidth='+IntToStr(ChunkWidth)+' ChunkHeight='+IntToStr(ChunkHeight)+' expected='+IntToStr(ExpectedChunkLength));
        end else if CurByteCnt>ExpectedChunkLength then begin
          // boundary tiles have padding
          ChunkBytesPerLine:=(SampleBitsPerPixel*IFD.TileWidth+7) div 8;
        end;
      end else begin //tctStrip
        ChunkLeft:=0;
        ChunkTop:=IFD.RowsPerStrip*ChunkIndex;
        ChunkWidth:=IFD.ImageWidth;
        ChunkHeight:=Min(IFD.RowsPerStrip,IFD.ImageHeight-ChunkTop);
        ChunkBytesPerLine:=(SampleBitsPerPixel*ChunkWidth+7) div 8;
        ExpectedChunkLength:=ChunkBytesPerLine*ChunkHeight;
        //writeln('TFPReaderTiff.LoadImageFromStream SampleBitsPerPixel=',SampleBitsPerPixel,' IFD.ImageWidth=',IFD.ImageWidth,' IFD.ImageHeight=',IFD.ImageHeight,' y=',y,' IFD.RowsPerStrip=',IFD.RowsPerStrip,' ExpectedChunkLength=',ExpectedChunkLength,' CurByteCnt=',CurByteCnt);
        if CurByteCnt<ExpectedChunkLength then
          TiffError('TFPReaderTiff.LoadImageFromStream Strip too short ByteCnt='+IntToStr(CurByteCnt)+' ChunkWidth='+IntToStr(ChunkWidth)+' ChunkHeight='+IntToStr(ChunkHeight)+' expected='+IntToStr(ExpectedChunkLength));
      end;

      // progress
      aContinue:=true;
      Progress(psRunning, 0, false, Rect(0,0,IFD.ImageWidth,ChunkTop), '', aContinue);
      if not aContinue then break;

      // Orientation
      if IFD.Orientation in [1..4] then begin
        x:=ChunkLeft; y:=ChunkTop;
        case IFD.Orientation of
        1: begin dx:=1; dy:=1; end;// 0,0 is left, top
        2: begin x:=IFD.ImageWidth-x-1; dx:=-1; dy:=1; end;// 0,0 is right, top
        3: begin x:=IFD.ImageWidth-x-1; dx:=-1; y:=IFD.ImageHeight-y-1; dy:=-1; end;// 0,0 is right, bottom
        4: begin dx:=1; y:=IFD.ImageHeight-y-1; dy:=-1; end;// 0,0 is left, bottom
        end;
      end else begin
        // rotated
        x:=ChunkTop; y:=ChunkLeft;
        case IFD.Orientation of
        5: begin dx:=1; dy:=1; end;// 0,0 is top, left (rotated)
        6: begin dx:=1; y:=IFD.ImageWidth-y-1; dy:=-1; end;// 0,0 is top, right (rotated)
        7: begin x:=IFD.ImageHeight-x-1; dx:=-1; y:=IFD.ImageWidth-y-1; dy:=-1; end;// 0,0 is bottom, right (rotated)
        8: begin x:=IFD.ImageHeight-x-1; dx:=-1; dy:=1; end;// 0,0 is bottom, left (rotated)
        end;
      end;

      //writeln('TFPReaderTiff.LoadImageFromStream Chunk ',ChunkIndex,' ChunkLeft=',ChunkLeft,' ChunkTop=',ChunkTop,' IFD.ImageWidth=',IFD.ImageWidth,' IFD.ImageHeight=',IFD.ImageHeight,' ChunkWidth=',ChunkWidth,' ChunkHeight=',ChunkHeight,' PaddingRight=',PaddingRight);
      sx:=x;
      for cy:=0 to ChunkHeight-1 do begin
        //writeln('TFPReaderTiff.LoadImageFromStream y=',y);
        Run:=Chunk+ChunkBytesPerLine*cy;
        LastRedValue:=0;
        LastGreenValue:=0;
        LastBlueValue:=0;
        LastGrayValue:=0;
        LastAlphaValue:=0;
        x:=sx;
        for cx:=0 to ChunkWidth-1 do begin
          case IFD.PhotoMetricInterpretation of
          0,1: // 0:bilevel grayscale 0 is white; 1:0 is black
            begin
              ReadImgValue(GrayBits,Run,cx,IFD.Predictor,LastGrayValue,GrayValue);
              if IFD.PhotoMetricInterpretation=0 then
                GrayValue:=$ffff-GrayValue;
              AlphaValue:=alphaOpaque;
              for i:=0 to ExtraSampleCnt-1 do begin
                if ExtraSamples[i] in [1,2] then begin
                  ReadImgValue(AlphaBits,Run,cx,IFD.Predictor,LastAlphaValue,AlphaValue);
=======
=======
>>>>>>> origin/fixes_2_2
    ReadShortOrLongValues(IDF.StripOffsets,StripOffsets,SOCount);
    if SOCount<>StripCount then
      TiffError('number of StripCounts is wrong');
    ReadShortOrLongValues(IDF.StripByteCounts,StripByteCounts,SBCCount);
    if SBCCount<>StripCount then
      TiffError('number of StripByteCounts is wrong');

    ReadShortValues(IDF.BitsPerSample,SampleBits,SampleCnt);
    if SampleCnt<>IDF.SamplesPerPixel then
      TiffError('Samples='+IntToStr(SampleCnt)+' <> SamplesPerPixel='+IntToStr(IDF.SamplesPerPixel));
    if IDF.ExtraSamples>0 then
      ReadShortValues(IDF.ExtraSamples,ExtraSamples,ExtraSampleCnt);
    if ExtraSampleCnt>=SampleCnt then
      TiffError('Samples='+IntToStr(SampleCnt)+' ExtraSampleCnt='+IntToStr(ExtraSampleCnt));

    case IDF.PhotoMetricInterpretation of
    0,1: if SampleCnt-ExtraSampleCnt<>1 then
      TiffError('gray images expect one sample per pixel, but found '+IntToStr(SampleCnt));
    2: if SampleCnt-ExtraSampleCnt<>3 then
      TiffError('rgb images expect three samples per pixel, but found '+IntToStr(SampleCnt));
    3: if SampleCnt-ExtraSampleCnt<>1 then
      TiffError('palette images expect one sample per pixel, but found '+IntToStr(SampleCnt));
    4: if SampleCnt-ExtraSampleCnt<>1 then
      TiffError('mask images expect one sample per pixel, but found '+IntToStr(SampleCnt));
    5: if SampleCnt-ExtraSampleCnt<>4 then
      TiffError('cmyk images expect four samples per pixel, but found '+IntToStr(SampleCnt));
    end;

    GrayBits:=0;
    RedBits:=0;
    GreenBits:=0;
    BlueBits:=0;
    AlphaBits:=0;
    BytesPerPixel:=0;
    case IDF.PhotoMetricInterpretation of
    0,1:
      begin
        GrayBits:=SampleBits[0];
        IDF.GrayBits:=GrayBits;
        for i:=0 to ExtraSampleCnt-1 do
          if ExtraSamples[i]=2 then begin
            AlphaBits:=SampleBits[1+i];
            IDF.AlphaBits:=AlphaBits;
          end;
      end;
    2:
      begin
        RedBits:=SampleBits[0];
        GreenBits:=SampleBits[1];
        BlueBits:=SampleBits[2];
        IDF.RedBits:=RedBits;
        IDF.GreenBits:=GreenBits;
        IDF.BlueBits:=BlueBits;
        for i:=0 to ExtraSampleCnt-1 do
          if ExtraSamples[i]=2 then begin
            AlphaBits:=SampleBits[3+i];
            IDF.AlphaBits:=AlphaBits;
          end;
      end;
    5:
      begin
        RedBits:=SampleBits[0];
        GreenBits:=SampleBits[1];
        BlueBits:=SampleBits[2];
        GrayBits:=SampleBits[3];
        IDF.RedBits:=RedBits;
        IDF.GreenBits:=GreenBits;
        IDF.BlueBits:=BlueBits;
        IDF.GrayBits:=GrayBits;
        for i:=0 to ExtraSampleCnt-1 do
          if ExtraSamples[i]=2 then begin
            AlphaBits:=SampleBits[4+i];
            IDF.AlphaBits:=AlphaBits;
          end;
      end;
    end;
    BytesPerPixel:=(GrayBits+RedBits+GreenBits+BlueBits+AlphaBits) div 8;
    IDF.BytesPerPixel:=BytesPerPixel;

    if not (IDF.FillOrder in [0,1]) then
      TiffError('FillOrder unsupported: '+IntToStr(IDF.FillOrder));

    for StripIndex:=0 to SampleCnt-1 do begin
      if not (SampleBits[StripIndex] in [8,16]) then
        TiffError('SampleBits unsupported: '+IntToStr(SampleBits[StripIndex]));
    end;

    // get image
    FirstImg.Assign(IDF);
    CurImg:=FirstImg.Img;
    if Assigned(OnCreateImage) then begin
      OnCreateImage(Self,CurImg);
      FirstImg.Img:=CurImg;
    end;
    if CurImg=nil then exit;

    ClearTiffExtras(CurImg);
    // set Tiff extra attributes
    CurImg.Extra[TiffPhotoMetric]:=IntToStr(IDF.PhotoMetricInterpretation);
    //writeln('TFPReaderTiff.ReadImage PhotoMetric=',CurImg.Extra[TiffPhotoMetric]);
    if IDF.Artist<>'' then
      CurImg.Extra[TiffArtist]:=IDF.Artist;
    if IDF.Copyright<>'' then
      CurImg.Extra[TiffCopyright]:=IDF.Copyright;
    if IDF.DocumentName<>'' then
      CurImg.Extra[TiffDocumentName]:=IDF.DocumentName;
    if IDF.DateAndTime<>'' then
      CurImg.Extra[TiffDateTime]:=IDF.DateAndTime;
    if IDF.ImageDescription<>'' then
      CurImg.Extra[TiffImageDescription]:=IDF.ImageDescription;
    if not (IDF.Orientation in [1..8]) then
      IDF.Orientation:=1;
    CurImg.Extra[TiffOrientation]:=IntToStr(IDF.Orientation);
    if IDF.ResolutionUnit<>0 then
      CurImg.Extra[TiffResolutionUnit]:=IntToStr(IDF.ResolutionUnit);
    if (IDF.XResolution.Numerator<>0) or (IDF.XResolution.Denominator<>0) then
      CurImg.Extra[TiffXResolution]:=TiffRationalToStr(IDF.XResolution);
    if (IDF.YResolution.Numerator<>0) or (IDF.YResolution.Denominator<>0) then
      CurImg.Extra[TiffYResolution]:=TiffRationalToStr(IDF.YResolution);
    CurImg.Extra[TiffRedBits]:=IntToStr(IDF.RedBits);
    CurImg.Extra[TiffGreenBits]:=IntToStr(IDF.GreenBits);
    CurImg.Extra[TiffBlueBits]:=IntToStr(IDF.BlueBits);
    CurImg.Extra[TiffGrayBits]:=IntToStr(IDF.GrayBits);
    CurImg.Extra[TiffAlphaBits]:=IntToStr(IDF.AlphaBits);
    //WriteTiffExtras('ReadImage',CurImg);

    case IDF.Orientation of
    0,1..4: CurImg.SetSize(IDF.ImageWidth,IDF.ImageHeight);
    5..8: CurImg.SetSize(IDF.ImageHeight,IDF.ImageWidth);
    end;


    y:=0;
    for StripIndex:=0 to StripCount-1 do begin
      // progress
      aContinue:=true;
      Progress(psRunning, 0, false, Rect(0,0,0,0), '', aContinue);
      if not aContinue then break;

      CurOffset:=StripOffsets[StripIndex];
      CurByteCnt:=StripByteCounts[StripIndex];
      //writeln('TFPReaderTiff.ReadImage CurOffset=',CurOffset,' CurByteCnt=',CurByteCnt);
      if CurByteCnt<=0 then continue;
      ReAllocMem(Strip,CurByteCnt);
      SetStreamPos(CurOffset);
      s.Read(Strip^,CurByteCnt);

      // decompress
      case IDF.Compression of
      1: ; // not compressed
      2: DecompressPackBits(Strip,CurByteCnt); // packbits
      5: DecompressLZW(Strip,CurByteCnt); // LZW
      else
        TiffError('compression '+IntToStr(IDF.Compression)+' not supported yet');
      end;
      if CurByteCnt<=0 then continue;

      Run:=0;
      dx:=0;
      dy:=0;
      for y2:=0 to IDF.RowsPerStrip-1 do begin
        if y>=IDF.ImageHeight then break;
        //writeln('TFPReaderTiff.ReadImage y=',y,' IDF.ImageWidth=',IDF.ImageWidth);
        for x:=0 to IDF.ImageWidth-1 do begin
          if PtrInt(Run)+BytesPerPixel>CurByteCnt then begin
            TiffError('TFPReaderTiff.ReadImage Strip too short Run='+IntToStr(Run)+' CurByteCnt='+IntToStr(CurByteCnt)+' x='+IntToStr(x)+' y='+IntToStr(y)+' y2='+IntToStr(y2));
            break;
          end;
          case IDF.PhotoMetricInterpretation of
          0,1:
            begin
              if GrayBits=8 then begin
                GrayValue:=PCUInt8(Strip)[Run];
                GrayValue:=GrayValue shl 8+GrayValue;
                inc(Run);
              end else if GrayBits=16 then begin
                GrayValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end else
                TiffError('gray image only supported with BitsPerSample 8 or 16 not yet supported');
              if IDF.PhotoMetricInterpretation=0 then
                GrayValue:=$ffff-GrayValue;
              AlphaValue:=alphaOpaque;
              for i:=0 to ExtraSampleCnt-1 do begin
                if ExtraSamples[i]=2 then begin
                  if SampleBits[1+i]=8 then begin
                    AlphaValue:=PCUInt8(Strip)[Run];
                    AlphaValue:=AlphaValue shl 8+AlphaValue;
                    inc(Run);
                  end else begin
                    AlphaValue:=FixEndian(PCUInt16(@Strip[Run])^);
                    inc(Run,2);
                  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                end else begin
                  inc(Run,ExtraSamples[i] div 8);
                end;
              end;
              Col:=FPColor(GrayValue,GrayValue,GrayValue,AlphaValue);
            end;

          2: // RGB(A)
            begin
<<<<<<< HEAD
<<<<<<< HEAD
              ReadImgValue(RedBits,Run,cx,IFD.Predictor,LastRedValue,RedValue);
              ReadImgValue(GreenBits,Run,cx,IFD.Predictor,LastGreenValue,GreenValue);
              ReadImgValue(BlueBits,Run,cx,IFD.Predictor,LastBlueValue,BlueValue);
              if SampleBitsPerPixel=32 then
                ReadImgValue(AlphaBits,Run,cx,IFD.Predictor,LastAlphaValue,AlphaValue)
              else
                AlphaValue:=alphaOpaque;
              for i:=0 to ExtraSampleCnt-1 do begin
                if ExtraSamples[i] in [1,2] then begin
                  ReadImgValue(AlphaBits,Run,cx,IFD.Predictor,LastAlphaValue,AlphaValue);
=======
=======
>>>>>>> origin/fixes_2_2
              if RedBits=8 then begin
                RedValue:=PCUInt8(Strip)[Run];
                RedValue:=RedValue shl 8+RedValue;
                inc(Run);
              end else begin
                RedValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              if GreenBits=8 then begin
                GreenValue:=PCUInt8(Strip)[Run];
                GreenValue:=GreenValue shl 8+GreenValue;
                inc(Run);
              end else begin
                GreenValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              if BlueBits=8 then begin
                BlueValue:=PCUInt8(Strip)[Run];
                BlueValue:=BlueValue shl 8+BlueValue;
                inc(Run);
              end else begin
                BlueValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              AlphaValue:=alphaOpaque;
              for i:=0 to ExtraSampleCnt-1 do begin
                if ExtraSamples[i]=2 then begin
                  if SampleBits[3+i]=8 then begin
                    AlphaValue:=PCUInt8(Strip)[Run];
                    AlphaValue:=AlphaValue shl 8+AlphaValue;
                    inc(Run);
                  end else begin
                    AlphaValue:=FixEndian(PCUInt16(@Strip[Run])^);
                    inc(Run,2);
                  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                end else begin
                  inc(Run,ExtraSamples[i] div 8);
                end;
              end;
              Col:=FPColor(RedValue,GreenValue,BlueValue,AlphaValue);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          //3 Palette/color map indexed
          //4 Mask/holdout mask (obsolete by TIFF 6.0 specification)
          5: // CMYK plus optional alpha
            begin
              ReadImgValue(RedBits,Run,cx,IFD.Predictor,LastRedValue,RedValue);
              ReadImgValue(GreenBits,Run,cx,IFD.Predictor,LastGreenValue,GreenValue);
              ReadImgValue(BlueBits,Run,cx,IFD.Predictor,LastBlueValue,BlueValue);
              ReadImgValue(GrayBits,Run,cx,IFD.Predictor,LastGrayValue,GrayValue);
              AlphaValue:=alphaOpaque;
              for i:=0 to ExtraSampleCnt-1 do begin
                if ExtraSamples[i] in [1,2] then begin
                  ReadImgValue(AlphaBits,Run,cx,IFD.Predictor,LastAlphaValue,AlphaValue);
=======
=======
>>>>>>> origin/fixes_2_2

          5: // CMYK plus optional alpha
            begin
              if RedBits=8 then begin
                RedValue:=PCUInt8(Strip)[Run];
                RedValue:=RedValue shl 8+RedValue;
                inc(Run);
              end else begin
                RedValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              if GreenBits=8 then begin
                GreenValue:=PCUInt8(Strip)[Run];
                GreenValue:=GreenValue shl 8+GreenValue;
                inc(Run);
              end else begin
                GreenValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              if BlueBits=8 then begin
                BlueValue:=PCUInt8(Strip)[Run];
                BlueValue:=BlueValue shl 8+BlueValue;
                inc(Run);
              end else begin
                BlueValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              if GrayBits=8 then begin
                GrayValue:=PCUInt8(Strip)[Run];
                GrayValue:=GrayValue shl 8+GrayValue;
                inc(Run);
              end else begin
                GrayValue:=FixEndian(PCUInt16(@Strip[Run])^);
                inc(Run,2);
              end;
              AlphaValue:=alphaOpaque;
              for i:=0 to ExtraSampleCnt-1 do begin
                if ExtraSamples[i]=2 then begin
                  if SampleBits[4+i]=8 then begin
                    AlphaValue:=PCUInt8(Strip)[Run];
                    AlphaValue:=AlphaValue shl 8+AlphaValue;
                    inc(Run);
                  end else begin
                    AlphaValue:=FixEndian(PCUInt16(@Strip[Run])^);
                    inc(Run,2);
                  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                end else begin
                  inc(Run,ExtraSamples[i] div 8);
                end;
              end;
              // CMYK to RGB
<<<<<<< HEAD
<<<<<<< HEAD
              Col:=CMYKToFPColor(RedValue,GreenValue,BlueValue,GrayValue);
            end;
           //6: YCBCR: CCIR 601
           //8: CIELAB: 1976 CIE L*a*b*
           //9: ICCLAB: ICC L*a*b*. Introduced post TIFF rev 6.0 by Adobe TIFF Technote 4
           //10: ITULAB: ITU L*a*b*
           //32844: LOGL: CIE Log2(L)
           //32845: LOGLUV: CIE Log2(L) (u',v')
          else
            TiffError('PhotometricInterpretation='+IntToStr(IFD.PhotoMetricInterpretation)+' not supported');
          end;

          CurFPImg.Colors[x,y]:=Col;
          // next column
          inc(x,dx);
        end;

        // next line
        inc(y,dy);
      end;
      // next chunk
=======
=======
>>>>>>> origin/fixes_2_2
              RedValue:=Max(0,integer($ffff)-RedValue-GrayBits);
              GreenValue:=Max(0,integer($ffff)-GreenValue-GrayBits);
              BlueValue:=Max(0,integer($ffff)-BlueValue-GrayBits);
              // set color
              Col:=FPColor(RedValue,GreenValue,BlueValue,AlphaValue);
            end;

          else
            TiffError('PhotometricInterpretation='+IntToStr(IDF.PhotoMetricInterpretation)+' not supported');
          end;

          // Orientation
          case IDF.Orientation of
          1: begin dx:=x; dy:=y; end;// 0,0 is left, top
          2: begin dx:=IDF.ImageWidth-x-1; dy:=y; end;// 0,0 is right, top
          3: begin dx:=IDF.ImageWidth-x-1; dy:=IDF.ImageHeight-y-1; end;// 0,0 is right, bottom
          4: begin dx:=x; dy:=IDF.ImageHeight-y; end;// 0,0 is left, bottom
          5: begin dx:=y; dy:=x; end;// 0,0 is top, left (rotated)
          6: begin dx:=IDF.ImageHeight-y-1; dy:=x; end;// 0,0 is top, right (rotated)
          7: begin dx:=IDF.ImageHeight-y-1; dy:=IDF.ImageWidth-x-1; end;// 0,0 is bottom, right (rotated)
          8: begin dx:=y; dy:=IDF.ImageWidth-x-1; end;// 0,0 is bottom, left (rotated)
          end;
          CurImg.Colors[dx,dy]:=Col;
        end;
        inc(y);
      end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  finally
    ReAllocMem(ExtraSamples,0);
    ReAllocMem(SampleBits,0);
<<<<<<< HEAD
<<<<<<< HEAD
    ReAllocMem(ChunkOffsets,0);
    ReAllocMem(ChunkByteCounts,0);
    ReAllocMem(Chunk,0);
=======
=======
>>>>>>> origin/fixes_2_2
    ReAllocMem(StripOffsets,0);
    ReAllocMem(StripByteCounts,0);
    ReAllocMem(Strip,0);
    FirstImg.Assign(IDF);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

function TFPReaderTiff.FixEndian(w: Word): Word; inline;
begin
  Result:=w;
  if FReverseEndian then
    Result:=((Result and $ff) shl 8) or (Result shr 8);
end;

function TFPReaderTiff.FixEndian(d: DWord): DWord; inline;
begin
  Result:=d;
  if FReverseEndian then
    Result:=((Result and $ff) shl 24)
          or ((Result and $ff00) shl 8)
          or ((Result and $ff0000) shr 8)
          or (Result shr 24);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TFPReaderTiff.DecodePackBits(var Buffer: Pointer; var Count: PtrInt);
var
  NewBuffer: Pointer;
  NewCount: PtrInt;
begin
  DecompressPackBits(Buffer,Count,NewBuffer,NewCount);
  FreeMem(Buffer);
  Buffer:=NewBuffer;
  Count:=NewCount;
end;

procedure TFPReaderTiff.DecodeLZW(var Buffer: Pointer; var Count: PtrInt);
var
  NewBuffer: Pointer;
  NewCount: PtrInt;
begin
  DecompressLZW(Buffer,Count,NewBuffer,NewCount);
  FreeMem(Buffer);
  Buffer:=NewBuffer;
  Count:=NewCount;
end;

procedure TFPReaderTiff.DecodeDeflate(var Buffer: Pointer; var Count: PtrInt;
  ExpectedCount: PtrInt);
var
  NewBuffer: PByte;
  NewCount: cardinal;
  ErrorMsg: String;
begin
  ErrorMsg:='';
  NewBuffer:=nil;
  try
    NewCount:=ExpectedCount;
    if not DecompressDeflate(Buffer,Count,NewBuffer,NewCount,@ErrorMsg) then
      TiffError(ErrorMsg);
    FreeMem(Buffer);
    Buffer:=NewBuffer;
    Count:=NewCount;
    NewBuffer:=nil;
  finally
    ReAllocMem(NewBuffer,0);
  end;
end;

procedure TFPReaderTiff.InternalRead(Str: TStream; AnImage: TFPCustomImage);
// read the biggest image
var
  Img: TTiffIFD;
  aContinue: Boolean;
  BestSize: PtrInt;
  NewSize: PtrInt;
  Best: integer;
  CurImg: TTiffIFD;
  i: Integer;
begin
  Clear;
  // read header
  aContinue:=true;
  Progress(psStarting, 0, False, Rect(0,0,0,0), '', aContinue);
  if not aContinue then exit;
  LoadHeaderFromStream(Str);
  LoadIFDsFromStream;
  // find the biggest image
  BestSize:=-1;
  Best:=-1;
  for i:=0 to ImageCount-1 do begin
    CurImg:=Images[i];
    NewSize:=Int64(CurImg.ImageWidth)*CurImg.ImageHeight;
    if (NewSize<=BestSize) then continue;
    BestSize:=NewSize;
    Best:=i;
  end;
  Progress(psRunning, 0, False, Rect(0,0,0,0), '', aContinue);
  // read image
  if Best>=0 then begin
    Img:=Images[Best];
    Img.Img:=AnImage;
    LoadImageFromStream(Best);
  end;
  // end
  Progress(psEnding, 100, False, Rect(0,0,0,0), '', aContinue);
end;

function TFPReaderTiff.InternalCheck(Str: TStream): boolean;
var
  IFDStart: DWord;
begin
  try
    s:=Str;
    fStartPos:=s.Position;
    Result:=ReadTiffHeader(true,IFDStart) and (IFDStart<>0);
    s.Position:=fStartPos;
  except
    Result:=false;
  end;
end;

procedure TFPReaderTiff.DoCreateImage(ImgFileDir: TTiffIFD);
begin
  if Assigned(OnCreateImage) then
    OnCreateImage(Self,ImgFileDir);
end;

constructor TFPReaderTiff.Create;
begin
  ImageList:=TFPList.Create;
end;

destructor TFPReaderTiff.Destroy;
begin
  Clear;
  FreeAndNil(ImageList);
  inherited Destroy;
end;

procedure TFPReaderTiff.Clear;
var
  i: Integer;
  Img: TTiffIFD;
begin
  for i:=ImageCount-1 downto 0 do begin
    Img:=Images[i];
    ImageList.Delete(i);
    if IFD=Img then IFD:=nil;
    Img.Free;
  end;
  FReverseEndian:=false;
  FreeAndNil(fIFDStarts);
end;

procedure DecompressPackBits(Buffer: Pointer; Count: PtrInt; out
  NewBuffer: Pointer; out NewCount: PtrInt);
{ Algorithm:
    while not got the expected number of bytes
      read one byte n
      if n in 0..127 copy the next n+1 bytes
      else if n in -127..-1 then copy the next byte 1-n times
      else continue
    end
}
var
  p: Pcint8;
  n: cint8;
  d: pcint8;
  i,j: integer;
  EndP: Pcint8;
begin
  // compute NewCount
  NewCount:=0;
  NewBuffer:=nil;
  if Count=0 then exit;
  p:=Pcint8(Buffer);
  EndP:=p+Count;
  while p<EndP do begin
    n:=p^;
    case n of
    0..127:   begin inc(NewCount,n+1);  inc(p,n+2); end; // copy the next n+1 bytes
    -127..-1: begin inc(NewCount,1-n); inc(p,2);   end; // copy the next byte 1-n times
    else inc(p); // noop
    end;
  end;

  // decompress
  if NewCount=0 then exit;
  GetMem(NewBuffer,NewCount);
  p:=Pcint8(Buffer);
  d:=Pcint8(NewBuffer);
  while p<EndP do begin
    n:=p^;
    case n of
    0..127:
      begin
        // copy the next n+1 bytes
        i:=n+1;
        inc(NewCount,i);
        inc(p);
        System.Move(p^,d^,i);
        inc(p,i);
        inc(d,i);
      end;
    -127..-1:
      begin
        // copy the next byte 1-n times
        i:=1-n;
        inc(NewCount,i);
        inc(p);
        n:=p^;
        for j:=0 to i-1 do
          d[j]:=n;
        inc(d,i);
        inc(p);
      end;
    else inc(p); // noop
    end;
  end;
end;

procedure DecompressLZW(Buffer: Pointer; Count: PtrInt; out NewBuffer: PByte;
  out NewCount: PtrInt);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFPReaderTiff.DecompressPackBits(var Buffer: Pointer; var Count: PtrInt
  );
var
  p: Pcint8;
  n: cint8;
  NewBuffer: Pcint8;
  SrcStep: PtrInt;
  NewCount: Integer;
  i: PtrInt;
  d: pcint8;
  j: ShortInt;
begin
  // compute NewCount
  NewCount:=0;
  p:=Pcint8(Buffer);
  i:=Count;
  while i>0 do begin
    n:=p^;
    case n of
    0..127:   begin inc(NewCount,n+1);  SrcStep:=n+2; end; // copy the next n+1 bytes
    -127..-1: begin inc(NewCount,-n+1); SrcStep:=2;   end; // copy the next byte n+1 times
    else SrcStep:=1; // noop
    end;
    inc(p,SrcStep);
    dec(i,SrcStep);
  end;

  // decompress
  if NewCount=0 then begin
    NewBuffer:=nil;
  end else begin
    GetMem(NewBuffer,NewCount);
    i:=Count;
    p:=Pcint8(Buffer);
    d:=Pcint8(NewBuffer);
    while i>0 do begin
      n:=p^;
      case n of
      0..127:
        begin
          // copy the next n+1 bytes
          inc(NewCount,n+1);  SrcStep:=n+2;
          System.Move(p[1],d^,n+1);
          inc(d,n+1);
        end;
      -127..-1:
        begin
          // copy the next byte n+1 times
          inc(NewCount,-n+1); SrcStep:=2;
          j:=-n;
          n:=p[1];
          while j>=0 do begin
            d[j]:=n;
            dec(j);
          end;
        end;
      else SrcStep:=1; // noop
      end;
      inc(p,SrcStep);
      dec(i,SrcStep);
    end;
  end;
  FreeMem(Buffer);
  Buffer:=NewBuffer;
  Count:=NewCount;
end;

procedure TFPReaderTiff.DecompressLZW(var Buffer: Pointer; var Count: PtrInt);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
  TLZWString = packed record
    Count: integer;
    Data: PByte;
  end;
  PLZWString = ^TLZWString;
const
<<<<<<< HEAD
<<<<<<< HEAD
  ClearCode = 256; // clear table, start with 9bit codes
  EoiCode = 257; // end of input
var
=======
=======
>>>>>>> origin/fixes_2_2
  EoiCode = 257;
  ClearCode = 256;
var
  NewBuffer: PByte;
  NewCount: PtrInt;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  NewCapacity: PtrInt;
  SrcPos: PtrInt;
  SrcPosBit: integer;
  CurBitLength: integer;
  Code: Word;
  Table: PLZWString;
  TableCapacity: integer;
  TableCount: integer;
  OldCode: Word;

<<<<<<< HEAD
<<<<<<< HEAD
  procedure Error(const Msg: string);
  begin
    raise Exception.Create(Msg);
  end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  function GetNextCode: Word;
  var
    v: Integer;
  begin
    Result:=0;
    // CurBitLength can be 9 to 12
<<<<<<< HEAD
<<<<<<< HEAD
    //writeln('GetNextCode CurBitLength=',CurBitLength,' SrcPos=',SrcPos,' SrcPosBit=',SrcPosBit,' ',hexstr(PByte(Buffer)[SrcPos],2),' ',hexstr(PByte(Buffer)[SrcPos+1],2),' ',hexstr(PByte(Buffer)[SrcPos+2],2));
    // read two or three bytes
    if CurBitLength+SrcPosBit>16 then begin
      // read from three bytes
      if SrcPos+3>Count then Error('LZW stream overrun');
=======
=======
>>>>>>> origin/fixes_2_2
    writeln('GetNextCode CurBitLength=',CurBitLength,' SrcPos=',SrcPos,' SrcPosBit=',SrcPosBit,' ',hexstr(PByte(Buffer)[SrcPos],2),' ',hexstr(PByte(Buffer)[SrcPos+1],2),' ',hexstr(PByte(Buffer)[SrcPos+2],2));
    // read two or three bytes
    if CurBitLength+SrcPosBit>16 then begin
      // read from three bytes
      if SrcPos+3>Count then TiffError('LZW stream overrun');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      v:=PByte(Buffer)[SrcPos];
      inc(SrcPos);
      v:=(v shl 8)+PByte(Buffer)[SrcPos];
      inc(SrcPos);
      v:=(v shl 8)+PByte(Buffer)[SrcPos];
      v:=v shr (24-CurBitLength-SrcPosBit);
    end else begin
      // read from two bytes
<<<<<<< HEAD
<<<<<<< HEAD
      if SrcPos+2>Count then Error('LZW stream overrun');
=======
      if SrcPos+2>Count then TiffError('LZW stream overrun');
>>>>>>> graemeg/fixes_2_2
=======
      if SrcPos+2>Count then TiffError('LZW stream overrun');
>>>>>>> origin/fixes_2_2
      v:=PByte(Buffer)[SrcPos];
      inc(SrcPos);
      v:=(v shl 8)+PByte(Buffer)[SrcPos];
      if CurBitLength+SrcPosBit=16 then
        inc(SrcPos);
      v:=v shr (16-CurBitLength-SrcPosBit);
    end;
    Result:=v and ((1 shl CurBitLength)-1);
    SrcPosBit:=(SrcPosBit+CurBitLength) and 7;
<<<<<<< HEAD
<<<<<<< HEAD
    //writeln('GetNextCode END SrcPos=',SrcPos,' SrcPosBit=',SrcPosBit,' Result=',Result,' Result=',hexstr(Result,4));
=======
    writeln('GetNextCode END SrcPos=',SrcPos,' SrcPosBit=',SrcPosBit,' Result=',Result,' Result=',hexstr(Result,4));
>>>>>>> graemeg/fixes_2_2
=======
    writeln('GetNextCode END SrcPos=',SrcPos,' SrcPosBit=',SrcPosBit,' Result=',Result,' Result=',hexstr(Result,4));
>>>>>>> origin/fixes_2_2
  end;

  procedure ClearTable;
  var
    i: Integer;
  begin
    for i:=0 to TableCount-1 do
      ReAllocMem(Table[i].Data,0);
    TableCount:=0;
  end;

  procedure InitializeTable;
  begin
    CurBitLength:=9;
    ClearTable;
  end;

  function IsInTable(Code: word): boolean;
  begin
    Result:=Code<258+TableCount;
  end;

  procedure WriteStringFromCode(Code: integer; AddFirstChar: boolean = false);
  var
    s: TLZWString;
    b: byte;
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    //WriteLn('WriteStringFromCode Code=',Code,' AddFirstChar=',AddFirstChar,' x=',(NewCount div 4) mod IFD.ImageWidth,' y=',(NewCount div 4) div IFD.ImageWidth,' PixelByte=',NewCount mod 4);
=======
    i: Integer;
  begin
    WriteLn('WriteStringFromCode Code=',Code,' AddFirstChar=',AddFirstChar);
>>>>>>> graemeg/fixes_2_2
=======
    i: Integer;
  begin
    WriteLn('WriteStringFromCode Code=',Code,' AddFirstChar=',AddFirstChar);
>>>>>>> origin/fixes_2_2
    if Code<256 then begin
      // write byte
      b:=Code;
      s.Data:=@b;
      s.Count:=1;
<<<<<<< HEAD
<<<<<<< HEAD
    end else if Code>=258 then begin
      // write string
      if Code-258>=TableCount then
        Error('LZW code out of bounds');
      s:=Table[Code-258];
    end else
      Error('LZW code out of bounds');
=======
=======
>>>>>>> origin/fixes_2_2
    end else begin
      // write string
      if Code-258>=TableCount then
        TiffError('LZW code out of bounds');
      s:=Table[Code-258];
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    if NewCount+s.Count+1>NewCapacity then begin
      NewCapacity:=NewCapacity*2+8;
      ReAllocMem(NewBuffer,NewCapacity);
    end;
    System.Move(s.Data^,NewBuffer[NewCount],s.Count);
<<<<<<< HEAD
<<<<<<< HEAD
    //for i:=0 to s.Count-1 do write(HexStr(NewBuffer[NewCount+i],2)); // debug
    inc(NewCount,s.Count);
    if AddFirstChar then begin
      NewBuffer[NewCount]:=s.Data^;
      //write(HexStr(NewBuffer[NewCount],2)); // debug
      inc(NewCount);
    end;
    //writeln(',WriteStringFromCode'); // debug
=======
=======
>>>>>>> origin/fixes_2_2
    for i:=0 to s.Count-1 do
      write(HexStr(NewBuffer[NewCount+i],2));
    inc(NewCount,s.Count);
    if AddFirstChar then begin
      NewBuffer[NewCount]:=s.Data^;
      write(HexStr(NewBuffer[NewCount],2));
      inc(NewCount);
    end;
    writeln(',WriteStringFromCode');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  procedure AddStringToTable(Code, AddFirstCharFromCode: integer);
  // add string from code plus first character of string from code as new string
  var
<<<<<<< HEAD
<<<<<<< HEAD
    b1, b2: byte;
    s1, s2: TLZWString;
    p: PByte;
  begin
    //WriteLn('AddStringToTable Code=',Code,' FCFCode=',AddFirstCharFromCode,' TableCount=',TableCount,' TableCapacity=',TableCapacity);
    if TableCount=4096-259 then
      Error('LZW too many codes');
=======
=======
>>>>>>> origin/fixes_2_2
    b: byte;
    s1, s2: TLZWString;
    p: PByte;
  begin
    WriteLn('AddStringToTable Code=',Code,' FCFCode=',AddFirstCharFromCode,' TableCount=',TableCount,' TableCapacity=',TableCapacity);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // grow table
    if TableCount>=TableCapacity then begin
      TableCapacity:=TableCapacity*2+128;
      ReAllocMem(Table,TableCapacity*SizeOf(TLZWString));
    end;
    // find string 1
    if Code<256 then begin
      // string is byte
<<<<<<< HEAD
<<<<<<< HEAD
      b1:=Code;
      s1.Data:=@b1;
      s1.Count:=1;
    end else if Code>=258 then begin
      // normal string
      if Code-258>=TableCount then
        Error('LZW code out of bounds');
      s1:=Table[Code-258];
    end else
      Error('LZW code out of bounds');
    // find string 2
    if AddFirstCharFromCode<256 then begin
      // string is byte
      b2:=AddFirstCharFromCode;
      s2.Data:=@b2;
=======
=======
>>>>>>> origin/fixes_2_2
      b:=Code;
      s1.Data:=@b;
      s1.Count:=1;
    end else begin
      // normal string
      if Code-258>=TableCount then
        TiffError('LZW code out of bounds');
      s1:=Table[Code-258];
    end;
    // find string 2
    if AddFirstCharFromCode<256 then begin
      // string is byte
      b:=AddFirstCharFromCode;
      s2.Data:=@b;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      s2.Count:=1;
    end else begin
      // normal string
      if AddFirstCharFromCode-258>=TableCount then
<<<<<<< HEAD
<<<<<<< HEAD
        Error('LZW code out of bounds');
=======
        TiffError('LZW code out of bounds');
>>>>>>> graemeg/fixes_2_2
=======
        TiffError('LZW code out of bounds');
>>>>>>> origin/fixes_2_2
      s2:=Table[AddFirstCharFromCode-258];
    end;
    // set new table entry
    Table[TableCount].Count:=s1.Count+1;
    p:=nil;
    GetMem(p,s1.Count+1);
    Table[TableCount].Data:=p;
    System.Move(s1.Data^,p^,s1.Count);
    // add first character from string 2
    p[s1.Count]:=s2.Data^;
    // increase TableCount
    inc(TableCount);
<<<<<<< HEAD
<<<<<<< HEAD
    if ((SrcPos+3=Count) and (CurBitLength+SrcPosBit>16)) or
       ((SrcPos+2=Count) and (CurBitLength+SrcPosBit<=16)) then exit;
    case TableCount+259 of
    512,1024,2048: inc(CurBitLength);
=======
    case TableCount+259 of
    512,1024,2048: inc(CurBitLength);
    4096: TiffError('LZW too many codes');
>>>>>>> graemeg/fixes_2_2
=======
    case TableCount+259 of
    512,1024,2048: inc(CurBitLength);
    4096: TiffError('LZW too many codes');
>>>>>>> origin/fixes_2_2
    end;
  end;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  NewBuffer:=nil;
  NewCount:=0;
  if Count=0 then exit;
  //WriteLn('DecompressLZW START Count=',Count);
  //for SrcPos:=0 to 19 do
  //  write(HexStr(PByte(Buffer)[SrcPos],2));
  //writeln();

=======
=======
>>>>>>> origin/fixes_2_2
  WriteLn('TFPReaderTiff.DecompressLZW START Count=',Count);
  for SrcPos:=0 to 19 do
    write(HexStr(PByte(Buffer)[SrcPos],2));
  writeln();

  NewBuffer:=nil;
  NewCount:=0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  NewCapacity:=Count*2;
  ReAllocMem(NewBuffer,NewCapacity);

  SrcPos:=0;
  SrcPosBit:=0;
  CurBitLength:=9;
  Table:=nil;
  TableCount:=0;
  TableCapacity:=0;
  try
    repeat
      Code:=GetNextCode;
<<<<<<< HEAD
<<<<<<< HEAD
      //WriteLn('DecompressLZW Code=',Code);
=======
      WriteLn('TFPReaderTiff.DecompressLZW Code=',Code);
>>>>>>> graemeg/fixes_2_2
=======
      WriteLn('TFPReaderTiff.DecompressLZW Code=',Code);
>>>>>>> origin/fixes_2_2
      if Code=EoiCode then break;
      if Code=ClearCode then begin
        InitializeTable;
        Code:=GetNextCode;
<<<<<<< HEAD
<<<<<<< HEAD
        //WriteLn('DecompressLZW after clear Code=',Code);
        if Code=EoiCode then break;
        if Code=ClearCode then
          Error('LZW code out of bounds');
=======
        if Code=EoiCode then break;
>>>>>>> graemeg/fixes_2_2
=======
        if Code=EoiCode then break;
>>>>>>> origin/fixes_2_2
        WriteStringFromCode(Code);
        OldCode:=Code;
      end else begin
        if Code<TableCount+258 then begin
          WriteStringFromCode(Code);
          AddStringToTable(OldCode,Code);
          OldCode:=Code;
        end else if Code=TableCount+258 then begin
          WriteStringFromCode(OldCode,true);
          AddStringToTable(OldCode,OldCode);
          OldCode:=Code;
        end else
<<<<<<< HEAD
<<<<<<< HEAD
          Error('LZW code out of bounds');
=======
          TiffError('LZW code out of bounds');
>>>>>>> graemeg/fixes_2_2
=======
          TiffError('LZW code out of bounds');
>>>>>>> origin/fixes_2_2
      end;
    until false;
  finally
    ClearTable;
    ReAllocMem(Table,0);
  end;

  ReAllocMem(NewBuffer,NewCount);
<<<<<<< HEAD
<<<<<<< HEAD
end;

function DecompressDeflate(Compressed: PByte; CompressedCount: cardinal;
  out Decompressed: PByte; var DecompressedCount: cardinal;
  ErrorMsg: PAnsiString = nil): boolean;
var
  stream : z_stream;
  err : integer;
begin
  Result:=false;
  //writeln('DecompressDeflate START');
  Decompressed:=nil;
  if CompressedCount=0 then begin
    DecompressedCount:=0;
    exit;
  end;

  err := inflateInit(stream{%H-});
  if err <> Z_OK then begin
    if ErrorMsg<>nil then
      ErrorMsg^:='inflateInit failed';
    exit;
  end;

  // set input = compressed data
  stream.avail_in := CompressedCount;
  stream.next_in  := Compressed;

  // set output = decompressed data
  if DecompressedCount=0 then
    DecompressedCount:=CompressedCount;
  Getmem(Decompressed,DecompressedCount);
  stream.avail_out := DecompressedCount;
  stream.next_out := Decompressed;

  // Finish the stream
  while TRUE do begin
    //writeln('run: total_in=',stream.total_in,' avail_in=',stream.avail_in,' total_out=',stream.total_out,' avail_out=',stream.avail_out);
    if (stream.avail_out=0) then begin
      // need more space
      if DecompressedCount<128 then
        DecompressedCount:=DecompressedCount+128
      else if DecompressedCount>High(DecompressedCount)-1024 then begin
        if ErrorMsg<>nil then
          ErrorMsg^:='inflate decompression failed, because not enough space';
        exit;
      end else
        DecompressedCount:=DecompressedCount*2;
      ReAllocMem(Decompressed,DecompressedCount);
      stream.next_out:=Decompressed+stream.total_out;
      stream.avail_out:=DecompressedCount-stream.total_out;
    end;
    err := inflate(stream, Z_NO_FLUSH);
    if err = Z_STREAM_END then
      break;
    if err<>Z_OK then begin
      if ErrorMsg<>nil then
        ErrorMsg^:='inflate finish failed';
      exit;
    end;
  end;

  //writeln('decompressed: total_in=',stream.total_in,' total_out=',stream.total_out);
  DecompressedCount:=stream.total_out;
  ReAllocMem(Decompressed,DecompressedCount);

  err := inflateEnd(stream);
  if err<>Z_OK then begin
    if ErrorMsg<>nil then
      ErrorMsg^:='inflateEnd failed';
    exit;
  end;
  Result:=true;
end;

initialization
  if ImageHandlers.ImageReader[TiffHandlerName]=nil then
    ImageHandlers.RegisterImageReader (TiffHandlerName, 'tif;tiff', TFPReaderTiff);
=======
=======
>>>>>>> origin/fixes_2_2
  FreeMem(Buffer);
  Buffer:=NewBuffer;
  Count:=NewCount;
end;

procedure TFPReaderTiff.InternalRead(Str: TStream; AnImage: TFPCustomImage);
begin
  FirstImg.Img:=AnImage;
  try
    LoadFromStream(Str);
  finally
    FirstImg.Img:=nil;
  end;
end;

function TFPReaderTiff.InternalCheck(Str: TStream): boolean;
var
  IFD: DWord;
begin
  try
    s:=Str;
    fStartPos:=s.Position;
    Result:=ReadTiffHeader(true,IFD) and (IFD<>0);
    s.Position:=fStartPos;
  except
    Result:=false;
  end;
end;

constructor TFPReaderTiff.Create;
begin
  IDF:=TTiffIDF.Create;
  FirstImg:=TTiffIDF.Create;
end;

destructor TFPReaderTiff.Destroy;
begin
  Clear;
  FreeAndNil(FirstImg);
  FreeAndNil(IDF);
  inherited Destroy;
end;

procedure TFPReaderTiff.Clear;
begin
  IDF.Clear;
  FirstImg.Clear;
  FReverseEndian:=false;
  FreeAndNil(fIFDStarts);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

