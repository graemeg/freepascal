{*****************************************************************************}
{
    This file is part of the Free Pascal's "Free Components Library".
    Copyright (c) 2014 by Mazen NEIFER of the Free Pascal development team
    and was adapted from wavopenal.pas copyright (c) 2010 Dmitry Boyarintsev.

    RIFF/WAVE sound file reader implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
unit fpwavreader;

{$mode objfpc}{$H+}

interface

uses
  fpWavFormat,
  Classes;

type
  { TWaveReader }

  TWavReader = class(TObject)
  private
    DataChunk: TChunkHeader;
    ChunkPos: Int64;
    EoF: Boolean;
    fStream: TStream;
    FFileName: string;
  public
    fmt   : TWaveFormat;
    destructor Destroy; override;
    function LoadFromFile(const FileName: string): Boolean;
    function LoadFromStream(AStream: TStream): Boolean;
    function ReadBuf(var Buffer; BufferSize: Integer): Integer;
  end;

implementation

uses
  SysUtils;

procedure LEtoN(var fmt: TWaveFormat); overload;
begin
  with fmt, ChunkHeader do begin
    Size := LEtoN(Size);
    Format := LEtoN(Format);
    Channels := LEtoN(Channels);
    SampleRate := LEtoN(SampleRate);
    ByteRate := LEtoN(ByteRate);
    BlockAlign := LEtoN(BlockAlign);
    BitsPerSample := LEtoN(BitsPerSample);
  end;
end;

{ TWaveReader }

destructor TWavReader.Destroy;
begin
  if (FFileName <> '') and  Assigned(fStream) then begin
    fStream.Free;
  end;
  inherited Destroy;
end;

function TWavReader.LoadFromFile(const FileName: string):Boolean;
begin
  if (FFileName <> '') and Assigned(fStream) then begin
    fStream.Free;
  end;
  fStream := TFileStream.Create(FileName, fmOpenRead + fmShareDenyWrite);
  if Assigned(fStream) then begin
    Result := LoadFromStream(fStream);
    FFileName := FileName;
  end else begin
    Result := False;
  end;
end;

function TWavReader.LoadFromStream(AStream:TStream):Boolean;
var
  riff: TRiffHeader;
begin
  fStream := AStream;
  FFileName := '';
  Result := fStream.Read(riff, sizeof(riff)) = sizeof(riff);
  riff.ChunkHeader.Size := LEtoN(riff.ChunkHeader.Size);
  Result := Result and (riff.ChunkHeader.ID = AUDIO_CHUNK_ID_RIFF) and (riff.Format = AUDIO_CHUNK_ID_WAVE);
  Result := Result and (fStream.Read(fmt, sizeof(fmt)) = sizeof(fmt));
  LEtoN(fmt);
  Result := Result and (fmt.ChunkHeader.ID = AUDIO_CHUNK_ID_fmt);
  if Result and (fmt.Format <> 1) then 
    Exit(False);
end;

function Min(a, b: Integer): Integer;
begin
  if a < b then begin
    Result := a;
  end else begin
    Result := b;
  end;
end;

function TWavReader.ReadBuf(var Buffer; BufferSize: Integer): Integer;
var
  sz: Integer;
  p: TByteArray absolute Buffer;
  i: Integer;
begin
  i := 0;
  while (not EoF) and (i < bufferSize) do begin
    if ChunkPos >= DataChunk.Size then begin
      sz := fstream.Read(DataChunk, sizeof(DataChunk));
      EoF := sz < sizeof(DataChunk);
      if not EoF then begin
        DataChunk.Size := LEtoN(DataChunk.Size);
        if DataChunk.Id <> AUDIO_CHUNK_ID_data then
          ChunkPos := DataChunk.Size
        else
          ChunkPos := 0;
      end;
    end else begin
      sz := Min(BufferSize, DataChunk.Size - ChunkPos);
      sz := fStream.Read(p[i], sz);
      EoF := sz <= 0;
      Inc(ChunkPos, sz);
      Inc(i, sz);
    end;
  end;
  Result := i;
end;

end.

