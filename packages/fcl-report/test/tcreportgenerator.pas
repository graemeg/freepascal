unit tcreportgenerator;

{$mode objfpc}{$H+}

interface

uses
  Classes,
  SysUtils,
  fpcunit,
  testregistry,
  fpreport,
  udapp,
  fpTTF,
  fpjson,
  {demos}
  rptsimplelist,
  rptexpressions,
  rptgrouping,
  rptgrouping2,
  rptframes,
  rptimages,
  rptttf,
  rptshapes,
  rptdataset,
  rptcolumns,
  rptmasterdetail,
  rptjson,
  rptcontnr,
  rptnestedgroups,
  rptBarcode,
  rptQRcode;

type

  { TTestDemos }

  TTestDemos = class(TTestCase)
  private
    FFilePath: String;
    procedure SaveJSON(pFileName: String; pJSON: TJSONData);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    procedure TestDemo(pName: String; pDemoAppClass: TReportDemoAppClass);
  published
    procedure SimpleList;
    procedure ExpressionDemo;
    procedure GroupingDemo;
    procedure Grouping2Demo;
    procedure FramesDemo;
    procedure ImagesDemo;
    procedure TTFDemo;
    procedure ShapesDemo;
    procedure DatasetDemo;
    procedure ColumnsDemo;
    procedure MasterDetailDemo;
    procedure JSONDemo;
    procedure CollectionDemo;
    procedure ObjectListDemo;
    procedure TestNestedGroupDemo;
    procedure BarcodeDemo;
    procedure QRCodeDemo;
  end;


implementation

uses
  fpjsonreport,
  jsonscanner,
  jsonparser;

{ TTestDemos }

procedure TTestDemos.SaveJSON(pFileName: String; pJSON: TJSONData);
var
  S: TFileStream;
  J: TJSONStringType;
begin
  S:=TFileStream.Create(pFileName,fmCreate);
  try
    J:=pJSON.FormatJSON;
    S.WriteBuffer(J[1],Length(J));
  finally
    S.Free;
  end;
end;

procedure TTestDemos.SetUp;
begin
  inherited SetUp;
  FFilePath:=ExtractFilePath(ParamStr(0));
  if not ForceDirectories(FFilePath+'rendered') then
     Fail('Could not create directory for rendered JSON');
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Clear;
  gTTFontCache.SearchPath.Add(FFilePath+'fonts/');
  gTTFontCache.SearchPath.Add(FFilePath+'../demos/fonts/');
{$IFDEF UNIX}
  gTTFontCache.SearchPath.Add(GetUserDir + '.fonts/');
  gTTFontCache.SearchPath.Add('/usr/share/fonts/truetype/ubuntu-font-family/');
  gTTFontCache.SearchPath.Add('/usr/share/fonts/truetype/ubuntu/');
  gTTFontCache.SearchPath.Add('/usr/share/fonts/truetype/dejavu/');
{$ENDIF}
  // ask to generate the font cache
  gTTFontCache.BuildFontCache;
end;

procedure TTestDemos.TearDown;
begin
  inherited TearDown;
end;

procedure TTestDemos.TestDemo(pName: String; pDemoAppClass: TReportDemoAppClass);
var
  lApp: TReportDemoApp;
  lSetJSON: TJSONData;
  lActualJSON: TJSONObject;
  S: TFileStream;
  P: TJSONParser;
  J: TJSONStringType;
  lEqual: Boolean;
  lSetFile, lActualFile: String;

begin
  lSetFile:=FFilePath+'rendered'+PathDelim+pName+'.set.json';
  lActualFile:=FFilePath+'rendered'+PathDelim+pName+'.actual.json';


  lApp:=pDemoAppClass.Create(Nil);
  lActualJSON := TJSONObject.Create;
  try
    // delete old actual
    DeleteFile(lActualFile);

    // create Report
    lApp.TestInit;
    // run first time
    lApp.rpt.RunReport;
    lApp.rpt.SaveRenderToJSON(lActualJSON);

    // delete DateCreated
    lActualJSON.GetPath('Report.DateCreated').AsString := '';

    //SaveJSON(lSetFile, lActualJSON); // uncomment for regeneration after changes
    if Not FileExists(lSetFile) then
       begin
       SaveJSON(lSetFile, lActualJSON);
       Ignore('No previous test result available, saved result for reference');
       end;
    // load set report
    S:=TFileStream.Create(lSetFile,fmOpenRead);
    try
      P:=TJSONParser.Create(S, []);
      try
        lSetJSON:=TJSONObject(P.Parse);

        // compare reports
        lEqual := lSetJSON.AsJSON = lActualJSON.AsJSON;
        if not lEqual then
          SaveJSON(lActualFile, lActualJSON);
        AssertTrue('equal renders', lEqual);

        // run a second time
        lApp.rpt.RunReport;
        lActualJSON.Clear;
        lApp.rpt.SaveRenderToJSON(lActualJSON);

        // delete DateCreated
        lActualJSON.GetPath('Report.DateCreated').AsString := '';

        // compare reports
        lEqual := lSetJSON.AsJSON = lActualJSON.AsJSON;
        if not lEqual then
          SaveJSON(lActualFile, lActualJSON);
        AssertTrue('equal second renders', lEqual);
      finally
        lSetJSON.Free;
        P.Free;
      end;
    finally
      S.Free;
    end;
  finally
    lActualJSON.Free;
    lApp.Free;
  end;
end;

procedure TTestDemos.SimpleList;
begin
  TestDemo('simplelist', TSimpleListDemo);
end;

procedure TTestDemos.ExpressionDemo;
begin
  TestDemo('expression', TExpressionsDemo);
end;

procedure TTestDemos.GroupingDemo;
begin
  TestDemo('grouping', TGroupingDemo);
end;

procedure TTestDemos.Grouping2Demo;
begin
  TestDemo('grouping2', TGrouping2Demo);
end;

procedure TTestDemos.FramesDemo;
begin
  TestDemo('frames', TFramesDemo);
end;

procedure TTestDemos.ImagesDemo;
var
  cd: String;
begin
  cd := GetCurrentDir;
  SetCurrentDir(cd+PathDelim+'..'+PathDelim+'demos');
  try
    TestDemo('images', TImagesDemo);
  finally
    SetCurrentDir(cd);
  end;
end;

procedure TTestDemos.TTFDemo;
var
  cd: String;
begin
  cd := GetCurrentDir;
  SetCurrentDir(cd+PathDelim+'..'+PathDelim+'demos');
  try
    TestDemo('ttf', TTTFDemo);
  finally
    SetCurrentDir(cd);
  end;
end;

procedure TTestDemos.ShapesDemo;
begin
  TestDemo('shapes', TShapesDemo);
end;

procedure TTestDemos.DatasetDemo;
var
  cd: String;
begin
  cd := GetCurrentDir;
  SetCurrentDir(cd+PathDelim+'..'+PathDelim+'demos');
  try
    TestDemo('dataset', TDatasetDemo);
  finally
    SetCurrentDir(cd);
  end;
end;

procedure TTestDemos.ColumnsDemo;
begin
  TestDemo('columns', TColumnsDemo)
end;

procedure TTestDemos.MasterDetailDemo;
begin
  TestDemo('masterdetail', TMasterDetailDemo);
end;

procedure TTestDemos.JSONDemo;
var
  cd: String;
begin
  cd := GetCurrentDir;
  SetCurrentDir(cd+PathDelim+'..'+PathDelim+'demos');
  try
    TestDemo('json', TJSONDemo);
  finally
    SetCurrentDir(cd);
  end;
end;

procedure TTestDemos.CollectionDemo;
begin
  TestDemo('collection', TCollectionDemo);
end;

procedure TTestDemos.ObjectListDemo;
begin
  TestDemo('objectlist', TObjectListDemo);
end;

procedure TTestDemos.BarcodeDemo;
begin
  TestDemo('barcode', TBarcodeDemo);
end;

procedure TTestDemos.QRCodeDemo;
begin
  TestDemo('qrcode', TQRCodeDemo);
end;

procedure TTestDemos.TestNestedGroupDemo;
begin
  TestDemo('nestedgroups', TNestedGroupsDemo);
end;

initialization
  RegisterTests(
    [TTestDemos
    ]);

end.

