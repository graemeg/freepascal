{
cdrvectorialreader.pas

Reads a Corel Draw vectorial file

CDR file format specification obtained from:

ADOBE SYSTEMS INCORPORATED. PDF Reference: Adobe®
Portable Document Format. San Jose, 2006. (Sixth edition).

AUTHORS: Felipe Monteiro de Carvalho

License: The same modified LGPL as the Free Pascal RTL
         See the file COPYING.modifiedLGPL for more details
}
unit cdrvectorialreader;

{$ifdef fpc}
  {$mode delphi}
{$endif}

interface

uses
  Classes, SysUtils,
  pdfvrlexico, pdfvrsintatico, pdfvrsemantico, avisozlib,
  fpvectorial;

type

  TCDRChunk = class
    Name: array[0..3] of Char;
    Size: Cardinal;
    ChildChunks: TFPList;
  end;

  TCDRChunkClass = class of TCDRChunk;

  TvCDRInternalData = TCDRChunk;

  TCDRChunkVRSN = class(TCDRChunk)
    VersionStr: string;
    VersionNum: Integer;
  end;

  { TvCDRVectorialReader }

  TvCDRVectorialReader = class(TvCustomVectorialReader)
  private
    procedure ReadVersionChunk(AStream: TStream; var AData: TCDRChunk);
    function AddNewChunk(var AData: TCDRChunk; AClass: TCDRChunkClass): TCDRChunk;
  public
    { General reading methods }
    procedure ReadFromStream(AStream: TStream; AData: TvVectorialDocument); override;
    { File format exploring methods }
    procedure ExploreFromFile(AFilename: string; out AData: TvCDRInternalData);
    procedure ExploreFromStream(AStream: TStream; out AData: TvCDRInternalData);
  end;

implementation

{ TvPDFVectorialReader }

procedure TvCDRVectorialReader.ReadVersionChunk(AStream: TStream;
  var AData: TCDRChunk);
var
  lDWord: DWord;
  lChunk: TCDRChunkVRSN absolute AData;
  lVerBytes: array[0..1] of Byte;
begin
  // Read the Chunk name
  lDWord := AStream.ReadDWord();

  // Read the Chunk size
  lDWord := AStream.ReadDWord();

  // Read the version
  AStream.Read(lVerBytes, 2);

  if (lVerBytes[0] = $BC) and (lVerBytes[1] = $02) then
  begin
    lChunk.VersionNum := 7;
    lChunk.VersionStr := 'CorelDraw 7';
  end
  else if (lVerBytes[0] = $20) and (lVerBytes[1] = $03) then
  begin
    lChunk.VersionNum := 8;
    lChunk.VersionStr := 'CorelDraw 8';
  end
  else if (lVerBytes[0] = $21) and (lVerBytes[1] = $03) then
  begin
    lChunk.VersionNum := 8;
    lChunk.VersionStr := 'CorelDraw 8bidi';
  end
  else if (lVerBytes[0] = $84) and (lVerBytes[1] = $03) then
  begin
    lChunk.VersionNum := 9;
    lChunk.VersionStr := 'CorelDraw 9';
  end
  else if (lVerBytes[0] = $E8) and (lVerBytes[1] = $03) then
  begin
    lChunk.VersionNum := 10;
    lChunk.VersionStr := 'CorelDraw 10';
  end
  else if (lVerBytes[0] = $4C) and (lVerBytes[1] = $04) then
  begin
    lChunk.VersionNum := 11;
    lChunk.VersionStr := 'CorelDraw 11';
  end
  else if (lVerBytes[0] = $B0) and (lVerBytes[1] = $04) then
  begin
    lChunk.VersionNum := 12;
    lChunk.VersionStr := 'CorelDraw 12';
  end
  else if (lVerBytes[0] = $14) and (lVerBytes[1] = $05) then
  begin
    lChunk.VersionNum := 13;
    lChunk.VersionStr := 'CorelDraw X3';
  end;
end;

function TvCDRVectorialReader.AddNewChunk(var AData: TCDRChunk; AClass: TCDRChunkClass): TCDRChunk;
begin
  if AData.ChildChunks = nil then AData.ChildChunks := TFPList.Create;

  Result := AClass.Create;

  AData.ChildChunks.Add(Result);
end;

procedure TvCDRVectorialReader.ReadFromStream(AStream: TStream;
  AData: TvVectorialDocument);
begin
end;

procedure TvCDRVectorialReader.ExploreFromFile(AFilename: string;
  out AData: TvCDRInternalData);
var
  FileStream: TFileStream;
begin
  FileStream := TFileStream.Create(AFileName, fmOpenRead or fmShareDenyNone);
  try
    ExploreFromStream(FileStream, AData);
  finally
    FileStream.Free;
  end;
end;

procedure TvCDRVectorialReader.ExploreFromStream(AStream: TStream;
  out AData: TvCDRInternalData);
var
  lRIFF: array[0..3] of Char;
  lDocSize, lDWord: Cardinal;
  lChild: TCDRChunk;
begin
  // Create the data object
  AData := TCDRChunk.Create;

  // All CorelDraw files starts with "RIFF"
  AStream.Read(lRIFF, 4);
  if lRIFF <> 'RIFF' then
    raise Exception.Create('[TvCDRVectorialReader.ExploreFromStream] The Corel Draw RIFF file marker wasn''t found.');

  // And then 4 bytes for the document size
  lDocSize := AStream.ReadDWord();

  // And mroe 4 bytes of other stuff
  lDWord := AStream.ReadDWord();

  // Now comes the version
  lChild := AddNewChunk(AData, TCDRChunkVRSN);
  ReadVersionChunk(AStream, lChild);
end;

initialization

  RegisterVectorialReader(TvCDRVectorialReader, vfCorelDrawCDR);

end.

