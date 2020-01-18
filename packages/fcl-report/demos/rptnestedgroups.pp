unit rptnestedgroups;

{$mode objfpc}{$H+}
{$I demos.inc}

interface

uses
  Classes,
  SysUtils,
  fpreport,
  udapp;

type

  { TNestedGroupsDemo }

  TNestedGroupsDemo = class(TReportDemoApp)
  private
    FReportData: TFPReportUserData;
    sl: TStringList;
    rec: TStringList;
    procedure   GetReportDataFirst(Sender: TObject);
    procedure   GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
    procedure   GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
    procedure   GetReportFieldNames(Sender: TObject; List: TStrings);
    procedure   ReportDataNext(Sender: TObject);
    procedure   PrepareRecord;
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
  jsonparser,
  fpexprpars;

const
  clGroupHeaderFooter2   = TFPReportColor($EFE1C7);
  clGroupHeaderFooter3   = TFPReportColor($DFD1B7);

{ TNestedGroupsDemo }

procedure TNestedGroupsDemo.GetReportDataFirst(Sender: TObject);
begin
  {$IFDEF gdebug}
  writeln('GetReportDataFirst');
  {$ENDIF}
  PrepareRecord;
end;

procedure TNestedGroupsDemo.GetReportDataValue(Sender: TObject; const AValueName: String; var AValue: Variant);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataValue - %d', [lReportData.RecNo]));
  {$ENDIF}
  case AValueName of
    'region': AValue := rec[0];
    'subregion': AValue := rec[1];
    'country': AValue := rec[2];
    'code': AValue := rec[3];
    'population': AValue := rec[4];
  end;
end;

procedure TNestedGroupsDemo.GetReportDataEOF(Sender: TObject; var IsEOF: Boolean);
begin
  {$IFDEF gdebug}
  writeln(Format('GetReportDataEOF - %d', [lReportData.RecNo]));
  {$ENDIF}
  if FReportData.RecNo > sl.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TNestedGroupsDemo.GetReportFieldNames(Sender: TObject; List: TStrings);
begin
  {$IFDEF gdebug}
  writeln('********** GetReportFieldNames');
  {$ENDIF}
  List.Add('region');
  List.Add('subregion');
  List.Add('country');
  List.Add('code');
  List.Add('population');
end;

procedure TNestedGroupsDemo.ReportDataNext(Sender: TObject);
begin
  PrepareRecord;
end;

procedure TNestedGroupsDemo.PrepareRecord;
begin
  if FReportData.RecNo > sl.Count then
    exit;
  rec.DelimitedText := sl[FReportData.RecNo-1];
end;

procedure TNestedGroupsDemo.InitialiseData;
begin
  sl := TStringList.Create;
  {$I countries2.inc}
  rec := TStringList.Create;
  rec.Delimiter := ';';
  rec.StrictDelimiter := true;
  rec.DelimitedText := sl[0]; // RecNo is 0
end;

procedure TNestedGroupsDemo.CreateReportDesign;
var
  Page: TFPReportPage;
  TitleBand: TFPReportTitleBand;
  DataBand: TFPReportDataBand;
  GroupHeader1Region, GroupHeader2Subregion, GroupHeader3Initial: TFPReportGroupHeaderBand;
  Memo: TFPReportMemo;
  PageFooter: TFPReportPageFooterBand;
  GroupFooter3Initial, GroupFooter2SubRegion, GroupFooter1Region: TFPReportGroupFooterBand;
  ChildBand: TFPReportChildBand;
  Shape: TFPReportShape;
begin
  Inherited;
  rpt.Author := 'Pascal Riekenberg';
  rpt.Title := 'FPReport Demo 13 - Nested Grouping';

  {****************}
  {***   page   ***}
  {****************}

  Page :=  TFPReportPage.Create(rpt);
  Page.Orientation := poPortrait;
  Page.PageSize.PaperName := 'A4';
  { page margins }
  Page.Margins.Left := 25;
  Page.Margins.Top := 20;
  Page.Margins.Right := 10;
  Page.Margins.Bottom := 20;
  Page.Data := FReportData;
  Page.Font.Name := 'LiberationSans';


  {*****************}
  {***   title   ***}
  {*****************}

  TitleBand := TFPReportTitleBand.Create(Page);
  TitleBand.Layout.Height := 40;
  TitleBand.Frame.Shape := fsRectangle;
  TitleBand.Frame.BackgroundColor := clReportTitleSummary;

    Memo := TFPReportMemo.Create(TitleBand);
    Memo.Layout.Left := 0;
    Memo.Layout.Top := 10;
    Memo.Layout.Width := Page.PageSize.Width - Page.Margins.Left - Page.Margins.Right;
    Memo.Layout.Height := 16;
    Memo.TextAlignment.Horizontal := taCentered;
    Memo.UseParentFont := False;
    Memo.Text := 'COUNTRY AND POPULATION AS OF 2016';
    Memo.Font.Size := 16;

    Memo := TFPReportMemo.Create(TitleBand);
    Memo.Layout.Left := 0;
    Memo.Layout.Top := 18;
    Memo.Layout.Width := Page.PageSize.Width - Page.Margins.Left - Page.Margins.Right;
    Memo.Layout.Height := 10;
    Memo.TextAlignment.Horizontal := taCentered;
    Memo.UseParentFont := False;
    Memo.Text := '(Total [formatfloat(''#,##0.0'',total_sum_population_in_M / 1000)] B)';
    Memo.Font.Size := 10;


  {**********************}
  {***  group header  ***}
  {**********************}

  {*** group header 1 region ***}

  {--- group header 1 region - band 1 ---}

  GroupHeader1Region := TFPReportGroupHeaderBand.Create(Page);
  GroupHeader1Region.Layout.Height := 15;
  GroupHeader1Region.GroupCondition := 'data.region';
  GroupHeader1Region.Frame.Shape := fsRectangle;
  GroupHeader1Region.Frame.BackgroundColor := clGroupHeaderFooter;
  //GroupHeader1Region.StartOnNewPage := True;
  GroupHeader1Region.ReprintedHeader := [rsPage];
  GroupHeader1Region.IntermediateFooter := [rsPage];
  //GroupHeader1Region.FooterPosition := fpStackAtBottom;
  GroupHeader1Region.VisibleExpr := 'not InRepeatedGroupHeader';

    Memo := TFPReportMemo.Create(GroupHeader1Region);
    Memo.Layout.Left := 3;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 16;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.Text := 'Region: [data.region] ([formatfloat(''#,##0.0'', grp1region_sum_population_in_M)] M)';

    Memo := TFPReportMemo.Create(GroupHeader1Region);
    Memo.Layout.Left := 25;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 145;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 10;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#0.0'', grp1region_sum_population / total_sum_population * 100)] % in World';


    ChildBand := TFPReportChildBand.Create(Page);
    ChildBand.Layout.Height := 7;
    ChildBand.VisibleExpr := 'InRepeatedGroupHeader';
    ChildBand.Frame.Shape := fsRectangle;
    ChildBand.Frame.BackgroundColor := clGroupHeaderFooter;
    GroupHeader1Region.ChildBand := ChildBand;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 3;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 4;
    Memo.Text := 'Region: [data.region]';


  {*** group header 2 subregion ***}

  {--- group header 2 subregion - band 1 ---}

  GroupHeader2Subregion := TFPReportGroupHeaderBand.Create(Page);
  GroupHeader2Subregion.Layout.Height := 2;
  GroupHeader2Subregion.GroupCondition := 'data.subregion';
  GroupHeader2Subregion.ParentGroupHeader := GroupHeader1Region;
  //GroupHeader2Subregion.StartOnNewPage := True;
  GroupHeader2Subregion.ReprintedHeader := [rsPage];
  GroupHeader2Subregion.IntermediateFooter := [rsPage];
  //GroupHeader2Subregion.FooterPosition := fpStackAtBottom;

    Shape := TFPReportShape.Create(GroupHeader2Subregion);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupHeader2Subregion.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

  {--- group header 2 subregion - band 2 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 15;
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter2;
  ChildBand.VisibleExpr := 'not InRepeatedGroupHeader';
  GroupHeader2Subregion.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 7;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 14;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.Text := 'Subregion: [data.subregion] ([formatfloat(''#,##0.0'', grp2subregion_sum_population_in_M)] M)';

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 25;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 145;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 10;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#0.0'', grp2subregion_sum_population / grp1region_sum_population * 100)] % in [data.region] - [formatfloat(''#0.0'', grp2subregion_sum_population / total_sum_population * 100)] % in World';

  {--- group header 2 subregion - band 3 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 7;
  ChildBand.VisibleExpr := 'InRepeatedGroupHeader';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter2;
  GroupHeader2Subregion.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 7;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 4;
    Memo.Text := 'Subregion: [data.subregion]';


  {*** group header 3 initial ***}

  {--- group header 3 initial - band 1 ---}

  GroupHeader3Initial := TFPReportGroupHeaderBand.Create(Page);
  GroupHeader3Initial.Layout.Height := 2;
  GroupHeader3Initial.GroupCondition := 'copy(data.country,1,1)';
  GroupHeader3Initial.ParentGroupHeader := GroupHeader2Subregion;
  GroupHeader3Initial.ReprintedHeader := [rsPage];
  GroupHeader3Initial.IntermediateFooter := [rsPage];
  //GroupHeader3Initial.OverflowedFooterNeedsReprintedHeader := True;

    Shape := TFPReportShape.Create(GroupHeader3Initial);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupHeader3Initial.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(GroupHeader3Initial);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupHeader3Initial.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

  {--- group header 3 initial - band 2 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 15;
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter3;
  ChildBand.VisibleExpr := 'not InRepeatedGroupHeader';
  GroupHeader3Initial.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 8;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 12;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 12;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.Text := '[copy(data.country,1,1)]  ([formatfloat(''#,##0.0'', grp3initial_sum_population_in_M)] M)';

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 25;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 145;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 10;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#0.0'', grp3initial_sum_population / grp2subregion_sum_population * 100)] % in [data.subregion] - [formatfloat(''#0.0'', grp3initial_sum_population / grp1region_sum_population * 100)] % in [data.region] - [formatfloat(''#0.0'', grp3initial_sum_population / total_sum_population * 100)] % in World';

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 90;
    Memo.Layout.Top := 10.5;
    Memo.Layout.Width := 20;
    Memo.Layout.Height := 4;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := 'Initial %';

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 110;
    Memo.Layout.Top := 10.5;
    Memo.Layout.Width := 20;
    Memo.Layout.Height := 4;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := 'Subreg. %';

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 130;
    Memo.Layout.Top := 10.5;
    Memo.Layout.Width := 20;
    Memo.Layout.Height := 4;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := 'Region %';

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 150;
    Memo.Layout.Top := 10.5;
    Memo.Layout.Width := 20;
    Memo.Layout.Height := 4;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := 'World %';

  {--- group header 3 initial - band 3 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 7;
  ChildBand.VisibleExpr := 'InRepeatedGroupHeader';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter3;
  GroupHeader3Initial.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 8;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 12;
    Memo.Layout.Top := 1;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 4;
    Memo.Text := '[copy(data.country,1,1)]';


  {--- group header 3 initial - band 4 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 2;
  GroupHeader3Initial.ChildBand.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter3;
    Shape.Layout.Left := 10;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter3;

  {*** variables ***}

  rpt.Variables.AddExprVariable('population_in_M', 'StrToFloat(data.population) / 1000000', rtFloat, rtNone, '');
  rpt.Variables.AddExprVariable('grp1region_sum_population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat, GroupHeader1Region);
  rpt.Variables.AddExprVariable('grp1region_sum_population', 'sum(StrToFloat(data.population))', rtFloat, GroupHeader1Region);
  rpt.Variables.AddExprVariable('grp2subregion_sum_population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat, GroupHeader2Subregion);
  rpt.Variables.AddExprVariable('grp2subregion_sum_population', 'sum(StrToFloat(data.population))', rtFloat, GroupHeader2Subregion);
  rpt.Variables.AddExprVariable('grp3initial_sum_population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat, GroupHeader3Initial);
  rpt.Variables.AddExprVariable('grp3initial_sum_population', 'sum(StrToFloat(data.population))', rtFloat, GroupHeader3Initial);
  rpt.Variables.AddExprVariable('total_sum_population_in_M', 'sum(StrToFloat(data.population) / 1000000)', rtFloat);
  rpt.Variables.AddExprVariable('total_sum_population', 'sum(StrToFloat(data.population))', rtFloat);


  {****************}
  {***  detail  ***}
  {****************}

  DataBand := TFPReportDataBand.Create(Page);
  DataBand.Layout.Height := 8;
  DataBand.Frame.Shape := fsRectangle;
  DataBand.Frame.BackgroundColor := clDataBand;
  //DataBand.VisibleExpr := 'StrToFloat(''[population]'') > 50000000';

    Shape := TFPReportShape.Create(DataBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := DataBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(DataBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := DataBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Shape := TFPReportShape.Create(DataBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := DataBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(DataBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 8;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := DataBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Shape := TFPReportShape.Create(DataBand);
    Shape.Color := clGroupHeaderFooter3;
    Shape.Layout.Left := 10;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := DataBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter3;

    Shape := TFPReportShape.Create(DataBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 13;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := DataBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 17;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 45;
    Memo.Layout.Height := 5;
    Memo.Text := '[data.country]';
    Memo.Options := memo.Options + [moDisableWordWrap];

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 55;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 25;
    Memo.Layout.Height := 5;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#,##0'', StrToFloat(data.population))]';

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 80;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 20;
    Memo.Layout.Height := 5;
    Memo.Text := '> DEU';
    Memo.UseParentFont := false;
    Memo.Font.Color := clGreen;
    Memo.VisibleExpr := 'StrToFloat(data.population) > 82667685';

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 80;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 20;
    Memo.Layout.Height := 5;
    Memo.Text := '< DEU';
    Memo.UseParentFont := false;
    Memo.Font.Color := clRed;
    Memo.VisibleExpr := 'StrToFloat(data.population) < 82667685';

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 95;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 15;
    Memo.Layout.Height := 5;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#,##0.0'',StrToFloat(data.population)/grp3initial_sum_population*100)] %';

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 115;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 15;
    Memo.Layout.Height := 5;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#,##0.0'',StrToFloat(data.population)/grp2subregion_sum_population*100)] %';

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 135;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 15;
    Memo.Layout.Height := 5;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#,##0.0'',StrToFloat(data.population)/grp1region_sum_population*100)] %';

    Memo := TFPReportMemo.Create(DataBand);
    Memo.Layout.Left := 155;
    Memo.Layout.Top := 2;
    Memo.Layout.Width := 15;
    Memo.Layout.Height := 5;
    Memo.TextAlignment.Horizontal := taRightJustified;
    Memo.Text := '[formatfloat(''#,##0.0'',StrToFloat(data.population)/total_sum_population*100)] %';


  {**********************}
  {***  group footer  ***}
  {**********************}

  {*** group footer 3 initial ***}

  {--- group footer 3 initial - band 1 ---}

  GroupFooter3Initial := TFPReportGroupFooterBand.Create(Page);
  GroupFooter3Initial.Layout.Height := 2;
  GroupFooter3Initial.GroupHeader := GroupHeader3Initial;
  GroupFooter3Initial.VisibleExpr := 'not IsOverflowed';

    Shape := TFPReportShape.Create(GroupFooter3Initial);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupFooter3Initial.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(GroupFooter3Initial);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupFooter3Initial.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(GroupFooter3Initial);
    Shape.Color := clGroupHeaderFooter3;
    Shape.Layout.Left := 10;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupFooter3Initial.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter3;

  {--- group footer 3 initial - band 2 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 2;
  ChildBand.VisibleExpr := 'IsOverflowed';
  GroupFooter3Initial.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

  {--- group footer 3 initial - band 3 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 15;
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter3;
  ChildBand.VisibleExpr := 'not InIntermediateGroupFooter';
  GroupFooter3Initial.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 8;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 12;
    Memo.Layout.Top := 3;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 12;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.Text := 'Population [copy(data.country,1,1)]: [formatfloat(''#,##0'', grp3initial_sum_population)]';

  {--- group footer 3 initial - band 4 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 3;
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter3;
  ChildBand.VisibleExpr := 'InIntermediateGroupFooter';
  GroupFooter3Initial.ChildBand.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 8;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;


  {*** group footer 2 subregion ***}

  {--- group footer 2 subregion - band 1 ---}

  GroupFooter2SubRegion := TFPReportGroupFooterBand.Create(Page);
  GroupFooter2SubRegion.Layout.Height := 2;
  GroupFooter2SubRegion.GroupHeader := GroupHeader2Subregion;
  GroupFooter2SubRegion.VisibleExpr := '(not IsOverflowed) and (not InIntermediateGroupFooter)';

    Shape := TFPReportShape.Create(GroupFooter2SubRegion);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupFooter2SubRegion.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(GroupFooter2SubRegion);
    Shape.Color := clGroupHeaderFooter2;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupFooter2SubRegion.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

  {--- group footer 2 subregion - band 2 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 2;
  ChildBand.VisibleExpr := 'IsOverflowed';
  GroupFooter2SubRegion.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

  {--- group footer 2 subregion - band 3 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 15;
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter2;
  ChildBand.VisibleExpr := 'not InIntermediateGroupFooter';
  GroupFooter2SubRegion.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 7;
    Memo.Layout.Top := 3;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 14;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.Text := 'Population [data.subregion]: [formatfloat(''#,##0'', grp2subregion_sum_population)]';

  {--- group footer 2 subregion - band 4 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 2;
  ChildBand.VisibleExpr := 'InIntermediateGroupFooter';
  GroupFooter2SubRegion.ChildBand.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 5;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter2;

  {--- group footer 2 subregion - band 5 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 3;
  ChildBand.VisibleExpr := 'InIntermediateGroupFooter';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter2;
  GroupFooter2SubRegion.ChildBand.ChildBand.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clNone;
    Shape.Layout.Left := 3;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 2;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clWhite;



  {*** group footer 1 region ***}

  {--- group footer 1 region - band 1 ---}

  GroupFooter1Region := TFPReportGroupFooterBand.Create(Page);
  GroupFooter1Region.Layout.Height := 2;
  GroupFooter1Region.GroupHeader := GroupHeader1Region;
  GroupFooter1Region.VisibleExpr := '(not IsOverflowed) and (not InIntermediateGroupFooter)';

    Shape := TFPReportShape.Create(GroupFooter1Region);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := GroupFooter1Region.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

  {--- group footer 1 region - band 2 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 15;
  ChildBand.VisibleExpr := 'not InIntermediateGroupFooter';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter;
  GroupFooter1Region.ChildBand := ChildBand;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 3;
    Memo.Layout.Top := 3;
    Memo.Layout.Width := 170;
    Memo.Layout.Height := 6;
    Memo.UseParentFont := False;
    Memo.Font.Size := 16;
    Memo.TextAlignment.Vertical := tlBottom;
    Memo.Text := 'Population [data.region]: [formatfloat(''#,##0'', grp1region_sum_population)]';

  {--- group footer 1 region - band 3 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 2;
  ChildBand.VisibleExpr := 'not InIntermediateGroupFooter';
  GroupFooter1Region.ChildBand.ChildBand := ChildBand;

  {--- group footer 1 region - band 4 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 2;
  ChildBand.VisibleExpr := 'InIntermediateGroupFooter';
  GroupFooter1Region.ChildBand.ChildBand.ChildBand := ChildBand;

    Shape := TFPReportShape.Create(ChildBand);
    Shape.Color := clGroupHeaderFooter;
    Shape.Layout.Left := 0;
    Shape.Layout.Top := 0;
    Shape.Layout.Width := 3;
    Shape.Layout.Height := ChildBand.Layout.Height;
    Shape.Frame.Shape := fsRectangle;
    Shape.Frame.BackgroundColor := clGroupHeaderFooter;

  {--- group footer 1 region - band 5 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 3;
  ChildBand.VisibleExpr := 'InIntermediateGroupFooter';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clGroupHeaderFooter;
  GroupFooter1Region.ChildBand.ChildBand.ChildBand.ChildBand := ChildBand;



  {*******************}
  {*** page footer ***}
  {*******************}

  {--- page footer - band 1 ---}

  PageFooter := TFPReportPageFooterBand.Create(Page);
  PageFooter.Layout.Height := 20;
  PageFooter.Frame.Shape := fsRectangle;
  PageFooter.Frame.BackgroundColor := clPageHeaderFooter;

    Memo := TFPReportMemo.Create(PageFooter);
    Memo.Layout.Left := 123;
    Memo.Layout.Top := 13;
    Memo.Layout.Width := 50;
    Memo.Layout.Height := 5;
    Memo.Text := 'Page [PageNo] of [PageCount]';
    Memo.TextAlignment.Vertical := tlCenter;
    Memo.TextAlignment.Horizontal := taRightJustified;

    Memo := TFPReportMemo.Create(PageFooter);
    Memo.Layout.Left := 0;
    Memo.Layout.Top := 5;
    Memo.Layout.Width := Page.PageSize.Width - Page.Margins.Left - Page.Margins.Right;
    Memo.Layout.Height := 8;
    Memo.UseParentFont := False;
    Memo.TextAlignment.Horizontal := taCentered;
    Memo.Text := 'Population World: [formatfloat(''#,##0'', total_sum_population)]';
    Memo.Font.Size := 16;

  {--- page footer - band 2 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 10;
  ChildBand.VisibleExpr := 'PageNo mod 2 = 0';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clChildBand;
  PageFooter.ChildBand := ChildBand;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 0;
    Memo.Layout.Top := 0;
    Memo.Layout.Width := Page.PageSize.Width - Page.Margins.Left - Page.Margins.Right;
    Memo.Layout.Height := 10;
    Memo.Text := 'even page: [pageno]';
    Memo.TextAlignment.Vertical := tlCenter;
    Memo.TextAlignment.Horizontal := taCentered;

  {--- page footer - band 3 ---}

  ChildBand := TFPReportChildBand.Create(Page);
  ChildBand.Layout.Height := 20;
  ChildBand.VisibleExpr := 'PageNo mod 5 = 0';
  ChildBand.Frame.Shape := fsRectangle;
  ChildBand.Frame.BackgroundColor := clReportTitleSummary;
  PageFooter.ChildBand.ChildBand := ChildBand;

    Memo := TFPReportMemo.Create(ChildBand);
    Memo.Layout.Left := 0;
    Memo.Layout.Top := 0;
    Memo.Layout.Width := Page.PageSize.Width - Page.Margins.Left - Page.Margins.Right;
    Memo.Layout.Height := 20;
    Memo.Text := 'multiple of 5: [pageno]';
    Memo.TextAlignment.Vertical := tlCenter;
    Memo.TextAlignment.Horizontal := taCentered;

end;

procedure TNestedGroupsDemo.LoadDesignFromFile(const AFilename: string);
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

procedure TNestedGroupsDemo.HookupData(const AComponentName: string; const AData: TFPReportData);
var
  b: TFPReportCustomBandWithData;
begin
  b := TFPReportCustomBandWithData(rpt.FindRecursive(AComponentName));
  if Assigned(b) then
    b.Data := AData;
end;

constructor TNestedGroupsDemo.Create(AOWner: TComponent);
begin
  inherited;
  FReportData := TFPReportUserData.Create(nil);
  FReportData.Name := 'Data';
  FReportData.OnGetValue := @GetReportDataValue;
  FReportData.OnGetEOF := @GetReportDataEOF;
  FReportData.OnFirst := @GetReportDataFirst;
  FReportData.OnGetNames := @GetReportFieldNames;
  FReportData.OnNext := @ReportDataNext;
end;

destructor TNestedGroupsDemo.Destroy;
begin
  FreeAndNil(FReportData);
  rec.DelimitedText := '';
  FreeAndNil(rec);
  FreeAndNil(sl);
  inherited Destroy;
end;

class function TNestedGroupsDemo.Description: string;
begin
  Result:='Demo showing grouping';
end;

end.

