{*****************************************************************************}
{
    This file is part of the Free Pascal's "Free Components Library".
    Copyright (c) 2014 by Mazen NEIFER of the Free Pascal development team
    and was adapted from wavopenal.pas copyright (c) 2010 Dmitry Boyarintsev.

    RIFF/WAVE sound file writer implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
unit fpwavwriter;

{$mode objfpc}{$H+}

interface

uses
  fpWavFormat,
  Classes;

type
  { TWaveReader }

  { TWavWriter }

  TWavWriter = class(TObject)
  private
    fStream: TStream;
    FFreeStreamOnClose: Boolean;
  public
    fmt: TWaveFormat;
    destructor Destroy; override;
    function CloseAudioFile: Boolean;
    function FlushHeader: Boolean;
    function StoreToFile(const FileName: string): Boolean;
    function StoreToStream(AStream: TStream): Boolean;
    function WriteBuf(var Buffer; BufferSize: Integer): Integer;
  end;

implementation

uses
  SysUtils;

procedure NtoLE(var fmt: TWaveFormat); overload;
begin
  with fmt, ChunkHeader do begin
    Size := NtoLE(Size);
    Format := NtoLE(Format);
    Channels := NtoLE(Channels);
    SampleRate := NtoLE(SampleRate);
    ByteRate := NtoLE(ByteRate);
    BlockAlign := NtoLE(BlockAlign);
    BitsPerSample := NtoLE(BitsPerSample);
  end;
end;

{ TWaveWriter }

destructor TWavWriter.Destroy;
begin
  CloseAudioFile;
  inherited Destroy;
end;

function TWavWriter.CloseAudioFile: Boolean;
begin
  Result := True;
  if not Assigned(fStream) then begin
    Exit(True);
  end;
  FlushHeader;
  if FFreeStreamOnClose then begin
    fStream.Free;
  end;
end;

function TWavWriter.FlushHeader: Boolean;
var
  riff: TRiffHeader;
  fmtLE: TWaveFormat;
  DataChunk: TChunkHeader;
  Pos: Int64;
begin
  Pos := fStream.Position;
  with riff, ChunkHeader do begin
    ID := AUDIO_CHUNK_ID_RIFF;
    Size := NtoLE(Pos - SizeOf(ChunkHeader));
    Format := AUDIO_CHUNK_ID_WAVE;
  end;
  fmtLE := fmt;
  NtoLE(fmtLE);
  with fStream do begin
    Position := 0;
    Result := Write(riff, SizeOf(riff)) = SizeOf(riff);
    Result := Write(fmtLE, sizeof(fmtLE)) = SizeOf(fmtLE);
  end;
  with DataChunk do begin
    Id := AUDIO_CHUNK_ID_data;
    Size := Pos - SizeOf(DataChunk) - fStream.Position;
  end;
  with fStream do begin
    Result := Write(DataChunk, SizeOf(DataChunk)) = SizeOf(DataChunk);
  end;
end;

function TWavWriter.StoreToFile(const FileName: string):Boolean;
begin
  CloseAudioFile;
  fStream := TFileStream.Create(FileName, fmCreate + fmOpenWrite);
  if Assigned(fStream) then begin
    Result := StoreToStream(fStream);
    FFreeStreamOnClose := True;
  end else begin
    Result := False;
  end;
end;

function TWavWriter.StoreToStream(AStream:TStream):Boolean;
begin
  fStream := AStream;
  FFreeStreamOnClose := False;
  with fmt, ChunkHeader do begin
    ID := AUDIO_CHUNK_ID_fmt;
    Size := SizeOf(fmt) - SizeOf(ChunkHeader);
    Format := AUDIO_FORMAT_PCM;
  end;
  Result := FlushHeader;
end;

function TWavWriter.WriteBuf(var Buffer; BufferSize: Integer): Integer;
var
  sz: Integer;
begin
  Result := 0;
  with fStream do begin
    sz := Write(Buffer, BufferSize);
    if sz < 0 then Exit;
    Inc(Result, sz);
  end;
end;

end.

