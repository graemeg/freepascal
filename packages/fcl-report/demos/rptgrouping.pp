unit rptgrouping;

{$mode objfpc}{$H+}
{$I demos.inc}

interface

uses
  Classes,
  SysUtils,
  fpreport,
  udapp;

type

  { TGroupingDemo }

  TGroupingDemo = class(TReportDemoApp)
  private
    lReportData: TFPReportUserData;
    sl: TStringList;
    procedure   GetReportDataFirst(Sender: TObject);
    procedure   GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
    procedure   GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
    procedure   GetReportFieldNames(Sender: TObject; List: TStrings);
  Protected
    procedure   InitialiseData; override;
    procedure   CreateReportDesign;override;
    procedure   LoadDesignFromFile(const AFilename: string);
    procedure   HookupData(const AComponentName: string; const AData: TFPReportData);
  public
    constructor Create(AOWner :TComponent); override;
    destructor  Destroy; override;
    Class function Description : string; override;
  end;


implementation

uses
  fpReportStreamer,
  fpTTF,
  fpJSON,
{$IFDEF VER3_0}  
  fprepexprpars,
{$ELSE}  
  fpexprpars,
{$ENDIF}
  jsonparser;

{ TGroupingDemo }

procedure TGroupingDemo.GetReportDataFirst(Sender: TObject);
begin
  {$IFDEF gdebug}
  writeln('GetReportDataFirst');
  {$ENDIF}
end;

procedure TGroupingDemo.GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
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

procedure TGroupingDemo.GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataEOF - %d', [lReportData.RecNo]));
  {$ENDIF}
  if lReportData.RecNo > sl.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TGroupingDemo.GetReportFieldNames(Sender: TObject; List: TStrings);
begin
  {$IFDEF gdebug}
  writeln('********** GetReportFieldNames');
  {$ENDIF}
  List.Add('country');
  List.Add('population');
end;

procedure TGroupingDemo.InitialiseData;
begin
  sl := TStringList.Create;
  {$I countries.inc}
  sl.Sort;
end;

procedure TGroupingDemo.CreateReportDesign;
var
  p: TFPReportPage;
  TitleBand: TFPReportTitleBand;
  DataBand: TFPReportDataBand;
  GroupHeader: TFPReportGroupHeaderBand;
  Memo: TFPReportMemo;
  PageFooter: TFPReportPageFooterBand;
  GroupFooter: TFPReportGroupFooterBand;
begin
  Inherited;
  rpt.Author := 'Graeme Geldenhuys';
  rpt.Title := 'FPReport Demo 3 - Grouping';

  {*** page ***}
  p :=  TFPReportPage.Create(rpt);
  p.Orientation := poPortrait;
  p.PageSize.PaperName := 'A4';
  { page margins }
  p.Margins.Left := 30;
  p.Margins.Top := 20;
  p.Margins.Right := 30;
  p.Margins.Bottom := 20;
  p.Data := lReportData;
  p.Font.Name := 'LiberationSans';


  {*** title ***}
  TitleBand := TFPReportTitleBand.Create(p);
  TitleBand.Layout.Height := 40;
  {$IFDEF ColorBands}
  TitleBand.Frame.Shape := fsRectangle;
  TitleBand.Frame.BackgroundColor := clReportTitleSummary;
  {$ENDIF}

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 0;
  Memo.Layout.Top := 20;
  Memo.Layout.Width := p.PageSize.Width - p.Margins.Left - p.Margins.Right;
  Memo.Layout.Height := 10;
  Memo.TextAlignment.Horizontal := taCentered;
  Memo.Text := 'COUNTRY AND POPULATION AS OF 2014';

  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Layout.Left := 0;
  Memo.Layout.Top := 25;
  Memo.Layout.Width := p.PageSize.Width - p.Margins.Left - p.Margins.Right;
  Memo.Layout.Height := 10;
  Memo.TextAlignment.Horizontal := taCentered;
  Memo.Text := '(Total [formatfloat(''#,##0.0'',sum_population_in_M / 1000)] B)';


  {*** group header ***}
  GroupHeader := TFPReportGroupHeaderBand.Create(p);
  GroupHeader.Layout.Height := 15;
  GroupHeader.GroupCondition := 'copy(data.country,1,1)';
  {$ifdef ColorBands}
  GroupHeader.Frame.Shape := fsRectangle;
  GroupHeader.Frame.BackgroundColor := clGroupHeaderFooter;
  {$endif}

  Memo := TFPReportMemo.Create(GroupHeader);
  Memo.Layout.Left := 5;
  Memo.Layout.Top := 3;
  Memo.Layout.Width := 10;
  Memo.Layout.Height := 8;
  Memo.UseParentFont := False;
  Memo.Text := '[copy(data.country,1,1)]';
  Memo.Font.Size := 16;

  Memo := TFPReportMemo.Create(GroupHeader);
  Memo.Layout.Left := 25;
  Memo.Layout.Top := 3;
  Memo.Layout.Width := 100;
  Memo.Layout.Height := 8;
  Memo.UseParentFont := False;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := '[formatfloat(''#,##0.0'', grp_sum_population_in_M)] M - [formatfloat(''#0.0'', grp_sum_population / sum_population * 100)] % ';
  Memo.Font.Size := 16;

  Memo := TFPReportMemo.Create(GroupHeader);
  Memo.Layout.Left := 105;
  Memo.Layout.Top := 11;
  Memo.Layout.Width := 20;
  Memo.Layout.Height := 4;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := 'Group %';

  Memo := TFPReportMemo.Create(GroupHeader);
  Memo.Layout.Left := 130;
  Memo.Layout.Top := 11;
  Memo.Layout.Width := 15;
  Memo.Layout.Height := 4;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := 'Total %';


  {*** variables ***}
  rpt.Variables.AddExprVariable('population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat, rtNone, '');
  rpt.Variables.AddExprVariable('grp_sum_population', 'sum(StrToFloat(data.population))',rtFloat , GroupHeader);
  rpt.Variables.AddExprVariable('grp_sum_population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat, GroupHeader);
  rpt.Variables.AddExprVariable('sum_population', 'sum(StrToFloat(data.population))', rtFloat, rtnone, '');
  rpt.Variables.AddExprVariable('sum_population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat,rtnone,'');


  {*** detail ***}
  DataBand := TFPReportDataBand.Create(p);
  DataBand.Layout.Height := 8;
  {$ifdef ColorBands}
  DataBand.Frame.Shape := fsRectangle;
  DataBand.Frame.BackgroundColor := clDataBand;
  {$endif}
  //DataBand.VisibleExpr := 'StrToFloat(''[population]'') > 50000000';

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 15;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 45;
  Memo.Layout.Height := 5;
  Memo.Text := '[data.country]';

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 55;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 25;
  Memo.Layout.Height := 5;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := '[formatfloat(''#,##0.0'', population_in_M)] M';
  //Memo.VisibleExpr := 'StrToFloat(''[population]'') > 50000000';

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 85;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 20;
  Memo.Layout.Height := 5;
  Memo.Text := '> Germany';
  Memo.UseParentFont := false;
  Memo.Font.Color := clGreen;
  Memo.VisibleExpr := 'StrToFloat(data.population) > 80890000';

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 85;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 20;
  Memo.Layout.Height := 5;
  Memo.Text := '< Germany';
  Memo.UseParentFont := false;
  Memo.Font.Color := clRed;
  Memo.VisibleExpr := 'StrToFloat(data.population) < 80890000';

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 110;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 15;
  Memo.Layout.Height := 5;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := '[formatfloat(''#,##0.0'',StrToFloat(data.population)/grp_sum_population*100)] %';

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Left := 130;
  Memo.Layout.Top := 2;
  Memo.Layout.Width := 15;
  Memo.Layout.Height := 5;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := '[formatfloat(''#,##0.0'',StrToFloat(data.population)/sum_population*100)] %';

  {*** group footer ***}
  GroupFooter := TFPReportGroupFooterBand.Create(p);
  GroupFooter.Layout.Height := 15;
  GroupFooter.GroupHeader := GroupHeader;
  {$ifdef ColorBands}
  GroupFooter.Frame.Shape := fsRectangle;
  GroupFooter.Frame.BackgroundColor := clGroupHeaderFooter;
  {$endif}

  Memo := TFPReportMemo.Create(GroupFooter);
  Memo.Layout.Left := 25;
  Memo.Layout.Top := 5;
  Memo.Layout.Width := 100;
  Memo.Layout.Height := 8;
  Memo.UseParentFont := False;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := '[formatfloat(''#,##0'', grp_sum_population)] - [formatfloat(''#0.0'', grp_sum_population / sum_population * 100)] % ';
  Memo.Font.Size := 16;


  {*** page footer ***}
  PageFooter := TFPReportPageFooterBand.Create(p);
  PageFooter.Layout.Height := 20;
  {$ifdef ColorBands}
  PageFooter.Frame.Shape := fsRectangle;
  PageFooter.Frame.BackgroundColor := clPageHeaderFooter;
  {$endif}

  Memo := TFPReportMemo.Create(PageFooter);
  Memo.Layout.Left := 100;
  Memo.Layout.Top := 13;
  Memo.Layout.Width := 50;
  Memo.Layout.Height := 5;
  Memo.Text := 'Page [PageNo] of [PAGECOUNT]';
  Memo.TextAlignment.Vertical := tlCenter;
  Memo.TextAlignment.Horizontal := taRightJustified;

  Memo := TFPReportMemo.Create(PageFooter);
  Memo.Layout.Left := 25;
  Memo.Layout.Top := 5;
  Memo.Layout.Width := 100;
  Memo.Layout.Height := 8;
  Memo.UseParentFont := False;
  Memo.TextAlignment.Horizontal := taRightJustified;
  Memo.Text := '[formatfloat(''#,##0'', sum_population)]';
  Memo.Font.Size := 16;
end;

procedure TGroupingDemo.LoadDesignFromFile(const AFilename: string);
var
  rs: TFPReportJSONStreamer;
  fs: TFileStream;
  lJSON: TJSONObject;
begin
  if AFilename = '' then
    Exit;
  if not FileExists(AFilename) then
    raise Exception.CreateFmt('The file "%s" can not be found', [AFilename]);

  fs := TFileStream.Create(AFilename, fmOpenRead or fmShareDenyNone);
  try
    lJSON := TJSONObject(GetJSON(fs));
  finally
    fs.Free;
  end;

  rs := TFPReportJSONStreamer.Create(nil);
  rs.JSON := lJSON; // rs takes ownership of lJSON
  try
    rpt.ReadElement(rs);
  finally
    rs.Free;
  end;
end;

procedure TGroupingDemo.HookupData(const AComponentName: string; const AData: TFPReportData);
var
  b: TFPReportCustomBandWithData;
begin
  b := TFPReportCustomBandWithData(rpt.FindRecursive(AComponentName));
  if Assigned(b) then
    b.Data := AData;
end;

constructor TGroupingDemo.Create(AOwner: TComponent);
begin
  inherited;
  lReportData := TFPReportUserData.Create(nil);
  lReportData.Name := 'data';
  lReportData.OnGetValue := @GetReportDataValue;
  lReportData.OnGetEOF := @GetReportDataEOF;
  lReportData.OnFirst := @GetReportDataFirst;
  lReportData.OnGetNames := @GetReportFieldNames;
end;

destructor TGroupingDemo.Destroy;
begin
  FreeAndNil(lReportData);
  FreeAndNil(sl);
  inherited Destroy;
end;

class function TGroupingDemo.Description: string;
begin
  Result:='Demo showing grouping';
end;

end.

