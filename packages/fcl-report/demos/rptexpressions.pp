unit rptexpressions;

{$mode objfpc}{$H+}
{$I demos.inc}

interface

uses
  Classes,
  SysUtils,
  fpreport,
  udapp;

type

  { TExpressionsDemo }

  TExpressionsDemo = class(TReportDemoApp)
  private
    FReportData: TFPReportUserData;
    sl: TStringList;
    procedure DoBeforePrint(Sender: TFPReportElement);
    procedure   GetReportDataFirst(Sender: TObject);
    procedure   GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
    procedure   GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
    procedure   GetReportFieldNames(Sender: TObject; List: TStrings);
  protected
    procedure   InitialiseData; override;
    procedure   CreateReportDesign; override;
  public
    constructor Create(AOwner : TComponent); override;
    destructor  Destroy; override;
    Class function Description : string; override;
  end;


implementation


{ TExpressionsDemo }

procedure TExpressionsDemo.GetReportDataFirst(Sender: TObject);
begin
  {$IFDEF gdebug}
  writeln('GetReportDataFirst');
  {$ENDIF}
end;

procedure TExpressionsDemo.DoBeforePrint(Sender: TFPReportElement);
begin
  With rpt.Variables.FindVariable('isEven') do
    AsBoolean:=Not AsBoolean;
end;

procedure TExpressionsDemo.GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataValue - %d', [lReportData.RecNo]));
  {$ENDIF}
  if AValueName = 'element' then
  begin
    AValue := sl[FReportData.RecNo-1];
  end;
end;

procedure TExpressionsDemo.GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataEOF - %d', [lReportData.RecNo]));
  {$ENDIF}
  if FReportData.RecNo > sl.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TExpressionsDemo.GetReportFieldNames(Sender: TObject; List: TStrings);
begin
  List.Add('element');
end;

procedure TExpressionsDemo.InitialiseData;
var
  i: integer;
begin
  sl := TStringList.Create;
  for i := 1 to 15 do
    sl.Add(Format(Char(64+i)+'-Item %d', [i]));
end;

procedure TExpressionsDemo.CreateReportDesign;
var
  p: TFPReportPage;
  TitleBand: TFPReportTitleBand;
  DataBand: TFPReportDataBand;
  Memo: TFPReportMemo;
  PageFooter: TFPReportPageFooterBand;
  DataHeader: TFPReportDataHeaderBand;
begin
  Inherited;

  rpt.Author := 'Graeme Geldenhuys';
  rpt.Title := 'FPReport Demo 2 - Expression Evaluation';

  // Line zero : even..
  rpt.Variables.AddVariable('isEven').AsBoolean:=True;

  p := TFPReportPage.Create(rpt);
  p.Orientation := poPortrait;
  p.PageSize.PaperName := 'A4';
  { page margins }
  p.Margins.Left := 20;
  p.Margins.Top := 20;
  p.Margins.Right := 20;
  p.Margins.Bottom := 20;
  p.Data := FReportData;
  p.Font.Name := 'LiberationSans';

  TitleBand := TFPReportTitleBand.Create(p);
  TitleBand.Layout.Height := 40;
  {$ifdef ColorBands}
  TitleBand.Frame.Shape := fsRectangle;
  TitleBand.Frame.BackgroundColor := clReportTitleSummary;
  {$endif}

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 55;
  Memo.Layout.Top := 10;
  Memo.Layout.Width := 50;
  Memo.Layout.Height := 8;
  Memo.Text := 'THE REPORT TITLE';

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 125;
  Memo.Layout.Top := 20;
  Memo.Layout.Width := 50;
  Memo.Layout.Height := 8;
  Memo.Text := 'Report Date: [TODAY]';

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 0;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 30;
  Memo.Layout.Height := 5;
  Memo.Text := '1 + 2 = [1 + 2].';
  Memo.Options := [moDisableExpressions];

  DataHeader := TFPReportDataHeaderBand.Create(p);
  DataHeader.Layout.Height := 10;
  {$ifdef ColorBands}
  DataHeader.Frame.Shape := fsRectangle;
  DataHeader.Frame.BackgroundColor := clDataHeaderFooter;
  {$endif}
  DataHeader.UseParentFont := False;
  DataHeader.Font.Name := 'LiberationSans-Bold';
  DataHeader.Data:=FReportData; 

  Memo := TFPReportMemo.Create(DataHeader);
  Memo.Layout.Left := 5;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 75;
  Memo.Layout.Height := 5;
  Memo.Text := 'Complex Example';
  Memo.Frame.Lines := [flBottom];
  Memo.TextAlignment.Vertical := tlCenter;

  Memo := TFPReportMemo.Create(DataHeader);
  Memo.Layout.Left := 85;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 25;
  Memo.Layout.Height := 5;
  Memo.Text := 'Single Calc';
  Memo.Frame.Lines := [flBottom];
  Memo.TextAlignment.Vertical := tlCenter;

  Memo := TFPReportMemo.Create(DataHeader);
  Memo.Layout.Left := 120;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 35;
  Memo.Layout.Height := 5;
  Memo.Text := 'System Variables';
  Memo.StretchMode := smActualHeight;
  Memo.Frame.Lines := [flBottom];
  Memo.TextAlignment.Vertical := tlCenter;


  DataBand := TFPReportDataBand.Create(p);
  DataBand.Layout.Height := 10;
  {$ifdef ColorBands}
  DataBand.Frame.Shape := fsRectangle;
  DataBand.Frame.BackgroundColor := clDataBand;
  {$endif}
  DataBand.OnBeforePrint:=@DoBeforePrint;
  DataBand.Data:=FReportData; 

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 5;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 75;
  Memo.Layout.Height := 5;
  Memo.Text := 'Even row: [isEven]. Hello world "[element]", and first letter is "[COPY(element, 1, 1)]".';
  Memo.Options := [];

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 85;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 25;
  Memo.Layout.Height := 5;
  Memo.Text := '[2 + RecNo('''') - 6]';
  Memo.Options := [moHideZeros];

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 120;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 25;
  Memo.Layout.Height := 5;
  Memo.Text := 'RecNo = [RecNo('''')].';

  PageFooter := TFPReportPageFooterBand.Create(p);
  PageFooter.Layout.Height := 20;
  {$ifdef ColorBands}
  PageFooter.Frame.Shape := fsRectangle;
  PageFooter.Frame.BackgroundColor := clPageHeaderFooter;
  {$endif}

  Memo := TFPReportMemo.Create(PageFooter);
  Memo.Layout.Left := 135;
  Memo.Layout.Top := 13;
  Memo.Layout.Width := 20;
  Memo.Layout.Height := 5;
  Memo.Text := 'Page [PageNo]';

  Memo := TFPReportMemo.Create(PageFooter);
  Memo.Layout.Left := 0;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 30;
  Memo.Layout.Height := 5;
  Memo.Text := '1 + 2 = [1 + 2].';
end;

constructor TExpressionsDemo.Create(AOwner: TComponent);
begin
  Inherited;
  FReportData := TFPReportUserData.Create(self);
  FReportData.OnGetValue := @GetReportDataValue;
  FReportData.OnGetEOF := @GetReportDataEOF;
  FReportData.OnFirst := @GetReportDataFirst;
  FReportData.OnGetNames := @GetReportFieldNames;
end;

destructor TExpressionsDemo.Destroy;
begin
  FreeAndNil(FReportData);
  FreeAndNil(sl);
  inherited Destroy;
end;

class function TExpressionsDemo.Description: string;
begin
  Result:='Demo with expressions in Memos';
end;



end.

