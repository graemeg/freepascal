unit tccsvdocument;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, csvdocument;

Type

  { TTestCSVDocument }

  TTestCSVDocument = Class(TTestCase)
  private
    FDoc: TCSVDocument;
    procedure RemoveTestFile;
    function StripQuotes(S: String): string;
    procedure TestTestFile;
  Public
    Procedure SetUp; override;
    Procedure TearDown; override;
    Procedure CreateTestFile;
    Property Doc : TCSVDocument Read FDoc;
  Published
    Procedure TestEmpty;
    Procedure TestRead;
  end;





implementation

Const
  TestFileName = 'test.csv';

{ TTestCSVDocument }

procedure TTestCSVDocument.SetUp;
begin
  FDoc:=TCSVDocument.Create;
  Inherited;
end;

procedure TTestCSVDocument.TearDown;
begin
  RemoveTestFile;
  FreeAndNil(FDoc);
  Inherited;
end;

procedure TTestCSVDocument.RemoveTestFile;

begin
  If FileExists(TestFileName) then
    AssertTrue('Deleting test file',DeleteFile(TestFileName));
end;

Const
  ColCount = 3;
  RowCount = 4;

Type
  TRow = Array[0..ColCount-1] of string;
  TCells = Array[0..RowCount-1] of TRow;

Const
  Cells : TCells = (
    ('a','b','c'),
    ('1','"one"','1.1'),
    ('2','"two"','2.2'),
    ('3','"three"','3.3')
  );

procedure TTestCSVDocument.CreateTestFile;

Var
  L : TStringList;
  R,C : Integer;
  S : String;

begin
  L:=TStringList.Create;
  try
    for R:=0 to RowCount-1 do
      begin
      S:='';
      for C:=0 to ColCount-1 do
        begin
        if S<>'' then
          S:=S+',';
        S:=S+Cells[R,C];
        end;
      L.Add(S);
      end;
    L.SaveToFile(TestFileName);
  finally
    L.Free;
  end;
end;

procedure TTestCSVDocument.TestEmpty;
begin
  AssertNotNull('Have document',Doc);
end;

Function TTestCSVDocument.StripQuotes(S : String) : string;

Var
  L : integer;

begin
  Result:=S;
  L:=Length(Result);
  if (L>1) then
    if (Result[1]='"') and (Result[L]='"') then
      Result:=Copy(Result,2,L-2);
end;

procedure TTestCSVDocument.TestTestFile;

Var
  R,C : Integer;

begin
  AssertEquals('Row count',RowCount,Doc.RowCount);
  For R:=0 to RowCount-1 do
    For C:=0 to ColCount-1 do
      begin
      AssertEquals('Col['+IntToStr(R)+'] count',ColCount,Doc.ColCount[R]);
      AssertEquals(Format('Cell[%d,%d]',[C,R]),StripQuotes(Cells[R,C]),Doc.Cells[C,R]);
      end;
end;

procedure TTestCSVDocument.TestRead;

begin
  CreateTestFile;
  Doc.LoadFromFile(TestFileName);
  TestTestFile;
end;

initialization
  RegisterTest(TTestCSVDocument);
end.

