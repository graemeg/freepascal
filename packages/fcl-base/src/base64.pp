{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by Michael Van Canneyt and Florian Klaempfl
    base64 encoder & decoder (c) 1999 Sebastian Guenther

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

// Encoding and decoding streams for base64 data as described in
//   RFC2045 (Mode = bdmMIME) and
//   RFC3548 (Mode = bdmStrict)

// Addition of TBase64DecodingMode supporting both Strict and MIME mode is
//   (C) 2007 Hexis BV, by Bram Kuijvenhoven (bkuijvenhoven@hexis.nl)

{$MODE objfpc}
{$H+}

unit base64;

interface

uses classes, sysutils;

type

<<<<<<< HEAD
<<<<<<< HEAD
  TBase64EncodingStream = class(TOwnerStream)
  protected
=======
  TBase64EncodingStream = class(TStream)
  protected
    OutputStream: TStream;
>>>>>>> graemeg/fixes_2_2
=======
  TBase64EncodingStream = class(TStream)
  protected
    OutputStream: TStream;
>>>>>>> origin/fixes_2_2
    TotalBytesProcessed, BytesWritten: LongWord;
    Buf: array[0..2] of Byte;
    BufSize: Integer;    // # of bytes used in Buf
  public
<<<<<<< HEAD
<<<<<<< HEAD
    destructor Destroy; override;
    Function Flush : Boolean;
=======
    constructor Create(AOutputStream: TStream);
    destructor Destroy; override;

>>>>>>> graemeg/fixes_2_2
=======
    constructor Create(AOutputStream: TStream);
    destructor Destroy; override;

>>>>>>> origin/fixes_2_2
    function Write(const Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
  end;

  (* The TBase64DecodingStream supports two modes:
   * - 'strict mode':
   *    - follows RFC3548
   *    - rejects any characters outside of base64 alphabet,
   *    - only accepts up to two '=' characters at the end and
<<<<<<< HEAD
<<<<<<< HEAD
   *    - requires the input to have a Size being a multiple of 4; otherwise raises an EBase64DecodingException
=======
   *    - requires the input to have a Size being a multiple of 4; otherwise raises an EBase64DecodeException
>>>>>>> graemeg/fixes_2_2
=======
   *    - requires the input to have a Size being a multiple of 4; otherwise raises an EBase64DecodeException
>>>>>>> origin/fixes_2_2
   * - 'MIME mode':
   *    - follows RFC2045
   *    - ignores any characters outside of base64 alphabet
   *    - takes any '=' as end of string
   *    - handles apparently truncated input streams gracefully
   *)
  TBase64DecodingMode = (bdmStrict, bdmMIME);

  { TBase64DecodingStream }

<<<<<<< HEAD
<<<<<<< HEAD
  TBase64DecodingStream = class(TOwnerStream)
=======
  TBase64DecodingStream = class(TStream)
>>>>>>> graemeg/fixes_2_2
=======
  TBase64DecodingStream = class(TStream)
>>>>>>> origin/fixes_2_2
  private
    FMode: TBase64DecodingMode;
    procedure SetMode(const AValue: TBase64DecodingMode);
    function  GetSize: Int64; override;
    function  GetPosition: Int64; override;
  protected
<<<<<<< HEAD
<<<<<<< HEAD
=======
    InputStream: TStream;
>>>>>>> graemeg/fixes_2_2
=======
    InputStream: TStream;
>>>>>>> origin/fixes_2_2
    CurPos,             // 0-based (decoded) position of this stream (nr. of decoded & Read bytes since last reset)
    DecodedSize: Int64; // length of decoded stream ((expected) decoded bytes since last Reset until Mode-dependent end of stream)
    ReadBase64ByteCount: Int64; // number of valid base64 bytes read from input stream since last Reset
    Buf: array[0..2] of Byte; // last 3 decoded bytes
<<<<<<< HEAD
<<<<<<< HEAD
    BufPos: Integer;          // offset in Buf of byte which is to be read next; if >2, next block must be read from Source & decoded
    FEOF: Boolean;            // if true, all decoded bytes have been read
  public
    constructor Create(ASource: TStream);
    constructor Create(ASource: TStream; AMode: TBase64DecodingMode);
=======
=======
>>>>>>> origin/fixes_2_2
    BufPos: Integer;          // offset in Buf of byte which is to be read next; if >2, next block must be read from InputStream & decoded
    FEOF: Boolean;            // if true, all decoded bytes have been read
  public
    constructor Create(AInputStream: TStream);
    constructor Create(AInputStream: TStream; AMode: TBase64DecodingMode);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure Reset;

    function Read(var Buffer; Count: Longint): Longint; override;
    function Seek(Offset: Longint; Origin: Word): Longint; override;
    
    property EOF: Boolean read fEOF;
    property Mode: TBase64DecodingMode read FMode write SetMode;
  end;
  
  EBase64DecodingException = class(Exception)
  end;

<<<<<<< HEAD
<<<<<<< HEAD
function EncodeStringBase64(const s:string):String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function DecodeStringBase64(const s:string;strict:boolean=false):String;
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> graemeg/cpstrnew
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> graemeg/cpstrnew
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> graemeg/cpstrnew
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
implementation

uses
  Math;

const
  SStrictNonBase64Char    = 'Non-valid Base64 Encoding character in input';
  SStrictInputTruncated   = 'Input stream was truncated at non-4 byte boundary';
  SStrictMisplacedPadChar = 'Unexpected padding character ''='' before end of input stream';

  EncodingTable: PChar =
    'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';

const
  NA =  85; // not in base64 alphabet at all; binary: 01010101
  PC = 255; // padding character                      11111111

  DecTable: array[Byte] of Byte =
    (NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,  // 0-15
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,  // 16-31
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 62, NA, NA, NA, 63,  // 32-47
     52, 53, 54, 55, 56, 57, 58, 59, 60, 61, NA, NA, NA, PC, NA, NA,  // 48-63
     NA, 00, 01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13, 14,  // 64-79
     15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, NA, NA, NA, NA, NA,  // 80-95
     NA, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,  // 96-111
     41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, NA, NA, NA, NA, NA,  // 112-127
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
     NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA);

  Alphabet = ['a'..'z','A'..'Z','0'..'9','+','/','=']; // all 65 chars that are in the base64 encoding alphabet

<<<<<<< HEAD
<<<<<<< HEAD
function TBase64EncodingStream.Flush : Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
function TBase64EncodingStream.Flush : Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TBase64EncodingStream.Flush : Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TBase64EncodingStream.Flush : Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TBase64EncodingStream.Flush : Boolean;

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
constructor TBase64EncodingStream.Create(AOutputStream: TStream);
begin
  inherited Create;
  OutputStream := AOutputStream;
end;

destructor TBase64EncodingStream.Destroy;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  WriteBuf: array[0..3] of Char;
begin
  // Fill output to multiple of 4
  case (TotalBytesProcessed mod 3) of
    1: begin
        WriteBuf[0] := EncodingTable[Buf[0] shr 2];
        WriteBuf[1] := EncodingTable[(Buf[0] and 3) shl 4];
        WriteBuf[2] := '=';
        WriteBuf[3] := '=';
<<<<<<< HEAD
<<<<<<< HEAD
        Source.Write(WriteBuf, 4);
        Result:=True;
        Inc(TotalBytesProcessed,2);
=======
        OutputStream.Write(WriteBuf, 4);
>>>>>>> graemeg/fixes_2_2
=======
        OutputStream.Write(WriteBuf, 4);
>>>>>>> origin/fixes_2_2
      end;
    2: begin
        WriteBuf[0] := EncodingTable[Buf[0] shr 2];
        WriteBuf[1] := EncodingTable[(Buf[0] and 3) shl 4 or (Buf[1] shr 4)];
        WriteBuf[2] := EncodingTable[(Buf[1] and 15) shl 2];
        WriteBuf[3] := '=';
<<<<<<< HEAD
<<<<<<< HEAD
        Source.Write(WriteBuf, 4);
        Result:=True;
        Inc(TotalBytesProcessed,1);
      end;
  else
    Result:=False;
  end;
end;

destructor TBase64EncodingStream.Destroy;
begin
  Flush;
=======
        OutputStream.Write(WriteBuf, 4);
      end;
  end;
>>>>>>> graemeg/fixes_2_2
=======
        OutputStream.Write(WriteBuf, 4);
      end;
  end;
>>>>>>> origin/fixes_2_2
  inherited Destroy;
end;

function TBase64EncodingStream.Write(const Buffer; Count: Longint): Longint;
var
  ReadNow: LongInt;
  p: Pointer;
  WriteBuf: array[0..3] of Char;
begin
  Inc(TotalBytesProcessed, Count);
  Result := Count;

  p := @Buffer;
  while count > 0 do begin
    // Fetch data into the Buffer
    ReadNow := 3 - BufSize;
    if ReadNow > Count then break;    // Not enough data available
    Move(p^, Buf[BufSize], ReadNow);
    Inc(p, ReadNow);
    Dec(Count, ReadNow);

    // Encode the 3 bytes in Buf
    WriteBuf[0] := EncodingTable[Buf[0] shr 2];
    WriteBuf[1] := EncodingTable[(Buf[0] and 3) shl 4 or (Buf[1] shr 4)];
    WriteBuf[2] := EncodingTable[(Buf[1] and 15) shl 2 or (Buf[2] shr 6)];
    WriteBuf[3] := EncodingTable[Buf[2] and 63];
<<<<<<< HEAD
<<<<<<< HEAD
    Source.Write(WriteBuf, 4);
=======
    OutputStream.Write(WriteBuf, 4);
>>>>>>> graemeg/fixes_2_2
=======
    OutputStream.Write(WriteBuf, 4);
>>>>>>> origin/fixes_2_2
    Inc(BytesWritten, 4);
    BufSize := 0;
  end;
  Move(p^, Buf[BufSize], count);
  Inc(BufSize, count);
end;

function TBase64EncodingStream.Seek(Offset: Longint; Origin: Word): Longint;
begin
  Result := BytesWritten;
  if BufSize > 0 then
    Inc(Result, 4);

  // This stream only supports the Seek modes needed for determining its size
  if not ((((Origin = soFromCurrent) or (Origin = soFromEnd)) and (Offset = 0))
     or ((Origin = soFromBeginning) and (Offset = Result))) then
    raise EStreamError.Create('Invalid stream operation');
end;

procedure TBase64DecodingStream.SetMode(const AValue: TBase64DecodingMode);
begin
  if FMode = AValue then exit;
  FMode := AValue;
  DecodedSize := -1; // forget any calculations on this
end;

function TBase64DecodingStream.GetSize: Int64;
var
  endBytes: array[0..1] of Char;
  ipos, isize: Int64;
  scanBuf: array[0..1023] of Char;
  count: LongInt;
  i: Integer;
  c: Char;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  // Note: this method only works on Seekable Sources (for bdmStrict we also get the Size property)
  if DecodedSize<>-1 then Exit(DecodedSize);
  ipos := Source.Position; // save position in input stream
=======
  // Note: this method only works on Seekable InputStreams (for bdmStrict we also get the Size property)
  if DecodedSize<>-1 then Exit(DecodedSize);
  ipos := InputStream.Position; // save position in input stream
>>>>>>> graemeg/fixes_2_2
=======
  // Note: this method only works on Seekable InputStreams (for bdmStrict we also get the Size property)
  if DecodedSize<>-1 then Exit(DecodedSize);
  ipos := InputStream.Position; // save position in input stream
>>>>>>> origin/fixes_2_2
  case Mode of
    bdmMIME:  begin
      // read until end of input stream or first occurence of a '='
      Result := ReadBase64ByteCount; // keep number of valid base64 bytes since last Reset in Result
      repeat
<<<<<<< HEAD
<<<<<<< HEAD
        count := Source.Read(scanBuf, SizeOf(scanBuf));
=======
        count := InputStream.Read(scanBuf, SizeOf(scanBuf));
>>>>>>> graemeg/fixes_2_2
=======
        count := InputStream.Read(scanBuf, SizeOf(scanBuf));
>>>>>>> origin/fixes_2_2
        for i := 0 to count-1 do begin
          c := scanBuf[i];
          if c in Alphabet-['='] then // base64 encoding characters except '='
            Inc(Result)
          else if c = '=' then // end marker '='
            Break;
        end;
      until count = 0;
      // writeln(Result);
      // we are now either at the end of the stream, or encountered our first '=', stored in c
      if c = '=' then begin // '=' found
        if Result mod 4 <= 1 then // badly placed '=', disregard last block
          Result := (Result div 4) * 3
        else // 4 byte block ended with '=' or '=='
          Result := (Result div 4) * 3 + Result mod 4 - 1;
      end else // end of stream
        Result := (Result div 4) * 3; // number of valid 4 byte blocks times 3
    end;
    bdmStrict:begin
      // seek to end of input stream, read last two bytes and determine size
<<<<<<< HEAD
<<<<<<< HEAD
      //   from Source size and the number of leading '=' bytes
      // NB we don't raise an exception here if the input does not contains an integer multiple of 4 bytes
      ipos  := Source.Position;
      isize := Source.Size;
      Result := ((ReadBase64ByteCount + (isize - ipos) + 3) div 4) * 3;
      Source.Seek(-2, soFromEnd);
      Source.Read(endBytes, 2);
=======
=======
>>>>>>> origin/fixes_2_2
      //   from InputStream size and the number of leading '=' bytes
      // NB we don't raise an exception here if the input does not contains an integer multiple of 4 bytes
      ipos  := InputStream.Position;
      isize := InputStream.Size;
      Result := ((ReadBase64ByteCount + (isize - ipos) + 3) div 4) * 3;
      InputStream.Seek(-2, soFromEnd);
      InputStream.Read(endBytes, 2);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      if endBytes[1] = '=' then begin // last byte
        Dec(Result);
      if endBytes[0] = '=' then       // second to last byte
        Dec(Result);
      end;
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  Source.Position := ipos; // restore position in input stream
=======
  InputStream.Position := ipos; // restore position in input stream
>>>>>>> graemeg/fixes_2_2
=======
  InputStream.Position := ipos; // restore position in input stream
>>>>>>> origin/fixes_2_2
  // store calculated DecodedSize
  DecodedSize := Result;
end;

function TBase64DecodingStream.GetPosition: Int64;
begin
  Result := CurPos;
end;

<<<<<<< HEAD
<<<<<<< HEAD
constructor TBase64DecodingStream.Create(ASource: TStream);
begin
  Create(ASource, bdmMIME); // MIME mode is default
end;

constructor TBase64DecodingStream.Create(ASource: TStream; AMode: TBase64DecodingMode);
begin
  inherited Create(ASource);
=======
=======
>>>>>>> origin/fixes_2_2
constructor TBase64DecodingStream.Create(AInputStream: TStream);
begin
  Create(AInputStream, bdmMIME); // MIME mode is default
end;

constructor TBase64DecodingStream.Create(AInputStream: TStream; AMode: TBase64DecodingMode);
begin
  inherited Create;
  InputStream := AInputStream;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Mode := AMode;
  Reset;
end;

procedure TBase64DecodingStream.Reset;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  ReadBase64ByteCount := 0; // number of bytes Read form Source since last call to Reset
=======
  ReadBase64ByteCount := 0; // number of bytes Read form InputStream since last call to Reset
>>>>>>> graemeg/fixes_2_2
=======
  ReadBase64ByteCount := 0; // number of bytes Read form InputStream since last call to Reset
>>>>>>> origin/fixes_2_2
  CurPos := 0; // position in decoded byte sequence since last Reset
  DecodedSize := -1; // indicates unknown; will be set after first call to GetSize or when reaching end of stream
  BufPos := 3; // signals we need to read & decode a new block of 4 bytes
  FEOF := False;
end;

function TBase64DecodingStream.Read(var Buffer; Count: Longint): Longint;
var
  p: PByte;
  b: byte;
  ReadBuf: array[0..3] of Byte; // buffer to store last read 4 input bytes
  ToRead, OrgToRead, HaveRead, ReadOK, i: Integer;
  
  procedure DetectedEnd(ASize:Int64);
  begin
    DecodedSize := ASize;
    // Correct Count if at end of base64 input
    if CurPos + Count > DecodedSize then
      Count := DecodedSize - CurPos;
  end;
  
begin
  if Count <= 0 then exit(0); // nothing to read, quit
  if DecodedSize <> -1 then begin // try using calculated size info if possible
    if CurPos + Count > DecodedSize then
      Count := DecodedSize - CurPos;
    if Count <= 0 then exit(0);
  end;

  Result := 0;
  p := @Buffer;
  while true do begin
    // get new 4-byte block if at end of Buf
    if BufPos > 2 then begin
      BufPos := 0;
      // Read the next 4 valid bytes
      ToRead := 4; // number of base64 bytes left to read into ReadBuf
      ReadOK := 0; // number of base64 bytes already read into ReadBuf
      while ToRead > 0 do begin
        OrgToRead := ToRead;
<<<<<<< HEAD
<<<<<<< HEAD
        HaveRead := Source.Read(ReadBuf[ReadOK], ToRead);
=======
        HaveRead := InputStream.Read(ReadBuf[ReadOK], ToRead);
>>>>>>> graemeg/fixes_2_2
=======
        HaveRead := InputStream.Read(ReadBuf[ReadOK], ToRead);
>>>>>>> origin/fixes_2_2
        //WriteLn('ToRead = ', ToRead, ', HaveRead = ', HaveRead, ', ReadOK=', ReadOk);
        if HaveRead > 0 then begin // if any new bytes; in ReadBuf[ReadOK .. ReadOK + HaveRead-1]
          for i := ReadOK to ReadOK + HaveRead - 1 do begin
            b := DecTable[ReadBuf[i]];
            if b <> NA then begin // valid base64 alphabet character ('=' inclusive)
              ReadBuf[ReadOK] := b;
              Inc(ReadOK);
              Dec(ToRead);
            end else if Mode=bdmStrict then begin // non-valid character
              raise EBase64DecodingException.CreateFmt(SStrictNonBase64Char,[]);
            end;
          end;
        end;
        
        if HaveRead <> OrgToRead then begin // less than 4 base64 bytes could be read; end of input stream
          //WriteLn('End: ReadOK=', ReadOK, ', count=', Count);
          for i := ReadOK to 3 do
            ReadBuf[i] := 0; // pad buffer with zeros so decoding of 4-bytes will be correct
          if (Mode = bdmStrict) and (ReadOK > 0) then
            raise EBase64DecodingException.CreateFmt(SStrictInputTruncated,[]);
          Break;
        end;
      end;

      Inc(ReadBase64ByteCount, ReadOK);
      
      // Check for pad characters
      case Mode of
        bdmStrict:begin
          if ReadOK = 0 then // end of input stream was reached at 4-byte boundary
            DetectedEnd(CurPos)
          else if (ReadBuf[0] = PC) or (ReadBuf[1] = PC) then
            raise EBase64DecodingException.CreateFmt(SStrictMisplacedPadChar,[])   // =BBB or B=BB
          else if (ReadBuf[2] = PC) then begin
<<<<<<< HEAD
<<<<<<< HEAD
            if (ReadBuf[3] <> PC) or (Source.Position < Source.Size) then
              raise EBase64DecodingException.CreateFmt(SStrictMisplacedPadChar,[]); // BB=B or BB==, but not at end of input stream
            DetectedEnd(CurPos + 1)  // only one byte left to read;  BB==, at end of input stream
          end else if (ReadBuf[3] = PC) then begin
            if (Source.Position < Source.Size) then
=======
=======
>>>>>>> origin/fixes_2_2
            if (ReadBuf[3] <> PC) or (InputStream.Position < InputStream.Size) then
              raise EBase64DecodingException.CreateFmt(SStrictMisplacedPadChar,[]); // BB=B or BB==, but not at end of input stream
            DetectedEnd(CurPos + 1)  // only one byte left to read;  BB==, at end of input stream
          end else if (ReadBuf[3] = PC) then begin
            if (InputStream.Position < InputStream.Size) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              raise EBase64DecodingException.CreateFmt(SStrictMisplacedPadChar,[]); // BBB=, but not at end of input stream
            DetectedEnd(CurPos + 2); // only two bytes left to read; BBB=, at end of input stream
          end;
        end;
        bdmMIME:begin
          if ReadOK = 0 then // end of input stream was reached at 4-byte boundary
            DetectedEnd(CurPos)
          else if (ReadBuf[0] = PC) or (ReadBuf[1] = PC) then
            DetectedEnd(CurPos)      // =BBB or B=BB: end here
          else if (ReadBuf[2] = PC) then begin
            DetectedEnd(CurPos + 1)  // only one byte left to read;  BB=B or BB==
          end else if (ReadBuf[3] = PC) then begin
            DetectedEnd(CurPos + 2); // only two bytes left to read; BBB=
          end;
        end;
      end;
      
      // Decode the 4 bytes in the buffer to 3 undecoded bytes
      Buf[0] :=  ReadBuf[0]         shl 2 or ReadBuf[1] shr 4;
      Buf[1] := (ReadBuf[1] and 15) shl 4 or ReadBuf[2] shr 2;
      Buf[2] := (ReadBuf[2] and  3) shl 6 or ReadBuf[3];
    end;
    
    if Count <= 0 then begin
      Break;
    end;

    // write one byte to Count
    p^ := Buf[BufPos];
    Inc(p);
    Inc(BufPos);
    Inc(CurPos);
    Dec(Count);
    Inc(Result);
  end;
  
  // check for EOF
  if (DecodedSize <> -1) and (CurPos >= DecodedSize) then begin
    FEOF := true;
  end;
end;

function TBase64DecodingStream.Seek(Offset: Longint; Origin: Word): Longint;
begin
  // TODO: implement Seeking in TBase64DecodingStream
  raise EStreamError.Create('Invalid stream operation');
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function DecodeStringBase64(const s:string;strict:boolean=false):String;
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> graemeg/cpstrnew
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> graemeg/cpstrnew
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> graemeg/cpstrnew
=======
function DecodeStringBase64(const s:string):String;
>>>>>>> origin/cpstrnew

var 
  Instream, 
  Outstream : TStringStream;
  Decoder   : TBase64DecodingStream;
begin
  Instream:=TStringStream.Create(s);
  try
    Outstream:=TStringStream.Create('');
    try 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if strict then
        Decoder:=TBase64DecodingStream.Create(Instream,bdmStrict)
      else
        Decoder:=TBase64DecodingStream.Create(Instream,bdmMIME);
      try
         Outstream.CopyFrom(Decoder,Decoder.Size);
         Result:=Outstream.DataString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      Decoder:=TBase64DecodingStream.Create(Instream,bdmMIME);
      try
         Outstream.CopyFrom(Decoder,Decoder.Size);
         Outstream.Position:=0;
         Result:=Outstream.ReadString(Outstream.Size);
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
      finally
        Decoder.Free;
        end;
    finally 
     Outstream.Free;
     end;
  finally 
    Instream.Free;
    end;
end;

function EncodeStringBase64(const s:string):String;

var
  Outstream : TStringStream;
  Encoder   : TBase64EncodingStream;
begin
  Outstream:=TStringStream.Create('');
  try
    Encoder:=TBase64EncodingStream.create(outstream);
    try 
      Encoder.Write(s[1],Length(s));
    finally 
      Encoder.Free;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=Outstream.DataString;
=======
    Outstream.Position:=0;
    Result:=Outstream.ReadString(Outstream.Size);
>>>>>>> graemeg/cpstrnew
=======
    Outstream.Position:=0;
    Result:=Outstream.ReadString(Outstream.Size);
>>>>>>> graemeg/cpstrnew
=======
    Outstream.Position:=0;
    Result:=Outstream.ReadString(Outstream.Size);
>>>>>>> graemeg/cpstrnew
=======
    Outstream.Position:=0;
    Result:=Outstream.ReadString(Outstream.Size);
>>>>>>> origin/cpstrnew
  finally
    Outstream.free;
    end;
end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

end.
