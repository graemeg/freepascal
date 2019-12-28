unit tccsvreadwrite;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, csvreadwrite;

type

  { TTestCSVReadWrite }

  TTestCSVReadWrite= class(TTestCase)
  private
    FData: TStrings;
    FParser: TCSVParser;
    procedure AssertLine(ARow: Integer; AValues: array of string);
    procedure HaveNext(ARow, ACol: integer; AValue: String);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    Property Parser : TCSVParser Read FParser;
    Property Data : TStrings Read FData;
  published
    procedure TestEmpty;
    Procedure TestNormalLine;
    Procedure TestQuotedLine;
    Procedure TestInlineQuotedLine;
    Procedure TestQuotedNewLine;
    Procedure Test2Lines;
    Procedure TestEscapedQuotes;
  end;

implementation

procedure TTestCSVReadWrite.TestEmpty;
begin
  AssertNotNull('Have parser',Parser);
end;

procedure TTestCSVReadWrite.HaveNext(ARow,ACol: integer; AValue : String);

Var
  CN : String;

begin
  CN:=Format('Cell(row: %d, col: %d)',[ARow,ACol]);
  AssertTrue('Have '+CN,Parser.ParseNextCell);
  AssertEquals(CN+': Row matches',ARow,Parser.CurrentRow);
  AssertEquals(CN+': Col matched',ACol,Parser.CurrentCol);
  AssertEquals(CN+': Value',AValue,Parser.CurrentCellText);
end;

procedure TTestCSVReadWrite.AssertLine(ARow: Integer; AValues: array of string);

Var
  I : Integer;

begin
  For I:=0 to Length(AValues)-1 do
    HaveNext(ARow,I,AValues[i]);
end;

procedure TTestCSVReadWrite.TestNormalLine;
begin
  FParser.SetSource('this,is,a,normal,line');
  AssertLine(0,['this','is','a','normal','line']);
end;

procedure TTestCSVReadWrite.TestQuotedLine;
begin
   FParser.SetSource('"this","is","a","quoted","line"');
   AssertLine(0,['this','is','a','quoted','line']);
end;

procedure TTestCSVReadWrite.TestInlineQuotedLine;
begin
  FParser.SetSource('"this","line",has,mixed" quoting"');
  AssertLine(0,['this','line','has','mixed quoting']);
end;

procedure TTestCSVReadWrite.TestQuotedNewLine;
begin
  FParser.SetSource('"this","line",has,"an embedded'+lineEnding+'newline"');
  AssertLine(0,['this','line','has','an embedded'+lineending+'newline']);
end;

procedure TTestCSVReadWrite.Test2Lines;
begin
  FParser.SetSource('"this","line",has,an embedded'+lineEnding+'newline');
  AssertLine(0,['this','line','has','an embedded']);
  AssertLine(1,['newline']);
end;

procedure TTestCSVReadWrite.TestEscapedQuotes;
begin
  FParser.SetSource('"this","line",has,"an embedded "" quote"');
  AssertLine(0,['this','line','has','an embedded " quote']);
end;

procedure TTestCSVReadWrite.SetUp;
begin
  FParser:=TCSVParser.Create;
  FData:=TStringList.Create;
end;

procedure TTestCSVReadWrite.TearDown;
begin
  FreeAndNil(FData);
  FreeAndNil(Fparser);
end;

initialization
  RegisterTest(TTestCSVReadWrite);
end.

