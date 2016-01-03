unit fpce_mainform;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs, StdCtrls,
  EditBtn, ExtCtrls, ComCtrls, ShellCtrls;

type

  { TformCorelExplorer }

  TformCorelExplorer = class(TForm)
    Label1: TLabel;
    Label2: TLabel;
    labelSize: TLabel;
    labelVersion: TLabel;
    labelFilename: TLabel;
    shellInput: TShellTreeView;
    procedure buttonQuitClick(Sender: TObject);
    procedure shellInputSelectionChanged(Sender: TObject);
  private
    { private declarations }
    function CheckInput(): Boolean;
  public
    { public declarations }
  end; 

var
  formCorelExplorer: TformCorelExplorer;

implementation

uses
  fpvectorial, cdrvectorialreader, svgvectorialwriter, pdfvectorialreader,
  fpvtocanvas;

{$R *.lfm}

{ TformCorelExplorer }

procedure TformCorelExplorer.buttonQuitClick(Sender: TObject);
begin
  Close;
end;

procedure TformCorelExplorer.shellInputSelectionChanged(Sender: TObject);
var
  Vec: TvVectorialDocument;
  Reader: TvCDRVectorialReader;
  lFormat: TvVectorialFormat;
  lChunk, lCurChunk: TCDRChunk;
  Str: string;
begin
  // First check the in input
  if not CheckInput() then Exit;

  // Now read the data from the input file
  Reader := TvCDRVectorialReader.Create;
  try
    Reader.ExploreFromFile(shellInput.GetSelectedNodePath(), lChunk);

    labelFilename.Caption := 'Filename: ' + shellInput.GetSelectedNodePath();
    if (lChunk.ChildChunks <> nil) and (lChunk.ChildChunks.First <> nil) then
    begin
      // Version Chunk
      lCurChunk := TCDRChunk(lChunk.ChildChunks.First);
      Str := TCDRChunkVRSN(lCurChunk).VersionStr;
      labelVersion.Caption := 'Version: ' + Str;

      // Main data
      lCurChunk := TCDRChunk(lChunk.ChildChunks.Items[1]);
      labelSize.Caption := 'Size: ' + ;
    end;
  finally
    Reader.Free;
  end;
end;

function TformCorelExplorer.CheckInput(): Boolean;
var
  lPath: String;
begin
  lPath := shellInput.GetSelectedNodePath();
  Result := (ExtractFileExt(lPath) = STR_CORELDRAW_EXTENSION);
end;

end.

