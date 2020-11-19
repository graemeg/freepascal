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
  ExpectedSampleRate, ExpectedChannels, ExpectedBitsPerSample : Integer;
begin
  AssignFile(InfoFile, CorrectFileDir + FileName + '.info.txt');
  Reset(InfoFile);
  try
    Readln(InfoFile, ExpectedSampleRate, ExpectedChannels, ExpectedBitsPerSample);
  finally
    CloseFile(InfoFile);
  end;

  WavReader := TWavReader.Create;
  try
    if not WavReader.LoadFromFile(CorrectFileDir + FileName) then
      Fail('Error loading wave file');
    AssertEquals('Incorrect sample rate', ExpectedSampleRate, WavReader.fmt.SampleRate);
    AssertEquals('Incorrect number of channels', ExpectedChannels, WavReader.fmt.Channels);
    AssertEquals('Incorrect number of bits per sample', ExpectedBitsPerSample, WavReader.fmt.BitsPerSample);
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
  TestValidFile('44k_mono_16_tag.wav');
end;



initialization

  RegisterTest(TTestWavReader);
end.

