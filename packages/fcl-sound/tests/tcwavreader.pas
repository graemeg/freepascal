unit tcwavreader;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testutils, testregistry,
  fpwavreader;

type

  { TTestWavReader }

  TTestWavReader= class(TTestCase)
  private
    procedure TestValidFile(const FileName: string);
  published
    procedure TestValidFiles;
  end;

implementation

procedure TTestWavReader.TestValidFile(const FileName: string);
const
  CorrectFileDir = 'data/wav/reader/valid/';
var
  WavReader: TWavReader;
  InfoFile: TextFile;
  RawDataFile: File;
  ExpectedSampleRate, ExpectedChannels, ExpectedBitsPerSample,
  ExpectedFormat: Integer;
  ExpectedData: array of Byte;
  ActualData: array of Byte;
  ActualDataLen: Integer;
  SaveFileMode: Byte;
begin
  AssignFile(InfoFile, CorrectFileDir + FileName + '.info.txt');
  Reset(InfoFile);
  try
    Readln(InfoFile, ExpectedSampleRate, ExpectedChannels, ExpectedBitsPerSample, ExpectedFormat);
  finally
    CloseFile(InfoFile);
  end;

  SaveFileMode := FileMode;
  try
    FileMode := 0;
    AssignFile(RawDataFile, CorrectFileDir + FileName + '.raw');
    Reset(RawDataFile, 1);
    try
      SetLength(ExpectedData, FileSize(RawDataFile));
      BlockRead(RawDataFile, ExpectedData[0], Length(ExpectedData));
    finally
      CloseFile(RawDataFile);
    end;
  finally
    FileMode := SaveFileMode;
  end;

  WavReader := TWavReader.Create;
  try
    if not WavReader.LoadFromFile(CorrectFileDir + FileName) then
      Fail('Error loading wave file');
    AssertEquals('Incorrect format', ExpectedFormat, WavReader.fmt.Format);
    AssertEquals('Incorrect sample rate', ExpectedSampleRate, WavReader.fmt.SampleRate);
    AssertEquals('Incorrect number of channels', ExpectedChannels, WavReader.fmt.Channels);
    AssertEquals('Incorrect number of bits per sample', ExpectedBitsPerSample, WavReader.fmt.BitsPerSample);
    SetLength(ActualData, Length(ExpectedData));
    ActualDataLen := WavReader.ReadBuf(ActualData[0], Length(ActualData));
    AssertEquals('Data length', Length(ExpectedData), ActualDataLen);
    AssertTrue('Data differs', CompareMem(@ExpectedData[0], @ActualData[0], ActualDataLen));
  finally
    FreeAndNil(WavReader);
  end;
end;

procedure TTestWavReader.TestValidFiles;
begin
  TestValidFile('44k_mono_8.wav');
  TestValidFile('44k_mono_16.wav');
  TestValidFile('44k_mono_24.wav');
  TestValidFile('44k_mono_32.wav');
  TestValidFile('44k_mono_32float.wav');
  TestValidFile('44k_mono_64float.wav');
  TestValidFile('44k_stereo_8.wav');
  TestValidFile('44k_stereo_16.wav');
  TestValidFile('44k_stereo_24.wav');
  TestValidFile('44k_stereo_32.wav');
  TestValidFile('44k_stereo_32float.wav');
  TestValidFile('44k_stereo_64float.wav');
  TestValidFile('44k_mono_16_tag.wav');
  TestValidFile('euphoric_tape.wav');
  TestValidFile('odd_fmt_size.wav');
end;



initialization

  RegisterTest(TTestWavReader);
end.

