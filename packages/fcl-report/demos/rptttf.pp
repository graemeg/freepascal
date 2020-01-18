unit rptttf;


{$mode objfpc}{$H+}
{$I demos.inc}

{.$define gdebugframes}

interface

uses
  Classes,
  SysUtils,
  fpreport,
  udapp;

type

  { TTTFDemo }

  TTTFDemo = class(TReportDemoApp)
  private
    lReportData: TFPReportUserData;
    sl: TStringList;
    procedure   GetReportDataFirst(Sender: TObject);
    procedure   GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
    procedure   GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
    procedure   GetReportFieldNames(Sender: TObject; List: TStrings);
  Protected
    procedure   InitialiseData; override;
    procedure   CreateReportDesign; override;
  public
    constructor Create(AOwner : TComponent); override;
    destructor  Destroy; override;
    Class function Description : string; override;
  end;


implementation


{ TTTFDemo }

procedure TTTFDemo.GetReportDataFirst(Sender: TObject);
begin
  {$IFDEF gdebug}
  writeln('GetReportDataFirst');
  {$ENDIF}
end;

procedure TTTFDemo.GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataValue - %d', [lReportData.RecNo]));
  {$ENDIF}
  if AValueName = 'country' then
  begin
    AValue := sl.Names[lReportData.RecNo-1];
  end
  else if AValueName = 'population' then
  begin
    AValue := sl.Values[sl.Names[lReportData.RecNo-1]];
  end;
end;

procedure TTTFDemo.GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataEOF - %d', [lReportData.RecNo]));
  {$ENDIF}
  if lReportData.RecNo > sl.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TTTFDemo.GetReportFieldNames(Sender: TObject; List: TStrings);
begin
  {$IFDEF gdebug}
  writeln('********** GetReportFieldNames');
  {$ENDIF}
  List.Add('country');
  List.Add('population');
end;

procedure TTTFDemo.InitialiseData;
begin
  sl := TStringList.Create;
  {$I countries.inc}
  sl.Sort;
end;

procedure TTTFDemo.CreateReportDesign;
var
  p: TFPReportPage;
  TitleBand: TFPReportTitleBand;
  DataBand: TFPReportDataBand;
  GroupHeader: TFPReportGroupHeaderBand;
  Memo: TFPReportMemo;
  PageFooter: TFPReportPageFooterBand;
begin
  Inherited;
  rpt.Author := 'Graeme Geldenhuys';
  rpt.Title := 'FPReport Demo 5 - TrueType Fonts';
  rpt.TwoPass := True;

  p := TFPReportPage.Create(rpt);
  p.Orientation := poPortrait;
  p.PageSize.PaperName := 'A4';
  { page margins }
  p.Margins.Left := 30;
  p.Margins.Top := 20;
  p.Margins.Right := 30;
  p.Margins.Bottom := 20;
  p.Data := lReportData;
  p.Font.Name := 'LiberationSans';

  TitleBand := TFPReportTitleBand.Create(p);
  TitleBand.Layout.Height := 40;
  {$IFDEF gdebugframes}
  TitleBand.Frame.Shape := fsRectangle;
  TitleBand.Frame.BackgroundColor := clReportTitleSummary;
  {$ENDIF}

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 5;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 140;
  Memo.Layout.Height := 15;
  Memo.Text := 'Country and Population as of 2014';
  Memo.TextAlignment.Vertical := tlCenter;
  Memo.TextAlignment.Horizontal := taCentered;
  Memo.UseParentFont := False;
  Memo.Font.Color := clWhite;
  Memo.Font.Name := 'Ubuntu'; // our custom TTF font
  Memo.Font.Size := 24;
  Memo.Frame.Shape := fsRectangle;
  Memo.Frame.BackgroundColor := TFPReportColor($01579B);

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 5;
  Memo.Layout.Top := 15;
  Memo.Layout.Width := 140;
  Memo.Layout.Height := 8;
  Memo.Text := 'Developed for the <font color="#ffffff" bgcolor="#2E7D32"><a href="http://www.freepascal.org">Free Pascal</a></font> project';
  Memo.TextAlignment.Vertical := tlCenter;
  Memo.TextAlignment.Horizontal := taCentered;
  Memo.UseParentFont := False;
  Memo.Font.Color := clGray;
  Memo.Font.Name := 'Ubuntu'; // our custom TTF font
  Memo.Options := [moAllowHTML, moDisableWordWrap];

  GroupHeader := TFPReportGroupHeaderBand.Create(p);
  GroupHeader.Layout.Height := 15;
  GroupHeader.Data := lReportData;
  GroupHeader.GroupCondition := 'copy(country,1,1)';
  GroupHeader.Frame.BackgroundColor := clYellow;    // this has no affect on rendered PDF because here Shape = fsNone
  GroupHeader.Frame.Color :=   TFPReportColor($01579B);
  GroupHeader.Frame.Lines := [flBottom];
  {$ifdef gdebugframes}
  GroupHeader.Frame.Shape := fsRectangle;
  GroupHeader.Frame.BackgroundColor := clGroupHeaderFooter;
  {$endif}

  Memo := TFPReportMemo.Create(GroupHeader);
  Memo.Layout.Left := 0;
  Memo.Layout.Top := 5;
  Memo.Layout.Width := 7;
  Memo.Layout.Height := 8;
  Memo.Text := '[copy(country,1,1)]';
  Memo.TextAlignment.Vertical := tlCenter;
  Memo.TextAlignment.Horizontal := taCentered;
  Memo.UseParentFont := False;
  Memo.Font.Size := 18;
  Memo.Font.Color := TFPReportColor($01579B);

  DataBand := TFPReportDataBand.Create(p);
  DataBand.Layout.Height := 8;
  {$ifdef gdebugframes}
  DataBand.Frame.Shape := fsRectangle;
  DataBand.Frame.BackgroundColor := clDataBand;
  {$endif}

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 15;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 50;
  Memo.Layout.Height := 5;
  Memo.Text := '<a href="http://en.wikipedia.org/wiki/[country]">[country]</a>';
  Memo.Options := [moAllowHTML, moDisableWordWrap];
  Memo.TextAlignment.Vertical := tlCenter;
  Memo.LinkColor := TFPReportColor($2E7D32);

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 105;
  Memo.Layout.Top := 0;
  Memo.Layout.Width := 30;
  Memo.Layout.Height := 5;
  Memo.Text := '[formatfloat(''#,##0'', StrToFloat(population))]';
  Memo.TextAlignment.Vertical := tlCenter;
  Memo.TextAlignment.Horizontal := taRightJustified;

  PageFooter := TFPReportPageFooterBand.Create(p);
  PageFooter.Layout.Height := 20;
  {$ifdef gdebugframes}
  PageFooter.Frame.Shape := fsRectangle;
  PageFooter.Frame.BackgroundColor := clPageHeaderFooter;
  {$endif}

  Memo := TFPReportMemo.Create(PageFooter);
  Memo.Layout.Left := 125;
  Memo.Layout.Top := 13;
  Memo.Layout.Width := 30;
  Memo.Layout.Height := 5;
  Memo.Text := 'Page [PageNo] of [PAGECOUNT]';
  Memo.UseParentFont := False;
  Memo.Font.Name := 'DejaVuSans'; // our custom TTF font
  Memo.Font.Size := 10;
end;

constructor TTTFDemo.Create(AOwner : TComponent);
begin
  Inherited;
  lReportData := TFPReportUserData.Create(nil);
  lReportData.OnGetValue := @GetReportDataValue;
  lReportData.OnGetEOF := @GetReportDataEOF;
  lReportData.OnFirst := @GetReportDataFirst;
  lReportData.OnGetNames := @GetReportFieldNames;
end;

destructor TTTFDemo.Destroy;
begin
  FreeAndNil(lReportData);
  FreeAndNil(sl);
  inherited Destroy;
end;

class function TTTFDemo.Description: string;
begin
  Result:='Demo showing TrueType Font support';
end;

end.

