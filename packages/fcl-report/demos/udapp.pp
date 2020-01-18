unit udapp;

{$mode objfpc}
{$h+}
{$I demos.inc}

interface

uses
  Classes, SysUtils, fpttf, fpreport, fpjsonreport,

  {$IFDEF ExportPDF}
  fpreportpdfexport,
  {$ENDIF}
  {$IFDEF ExportFPIMAGE}
  fpreportfpimageexport,
  {$ENDIF}
  {$IFDEF ExportHTML}
  fpreporthtmlexport,
  {$ENDIF}
  {$IFDEF ExportAggPas}
  fpreportaggpasexport,
  {$ENDIF}
  {$IFDEF ExportLCL}
  fpreportformexport,
  fpreportprinterexport,
  fpreportpreview,
  cfgfpreportpdfexport,
  cfgfpreportimageexport,
  forms,
  interfaces,
  {$ENDIF}
  {$IFDEF ExportFPGui}
  fpreport_export_form,
  fpg_base,
  fpg_main,
  fpg_form,
  {$ENDIF}
  custapp,
  fpreportstreamer;

Type
  // Order is important for default. First available class will be used as default.
  TRenderFormat = (rfDefault,rfPDF,rfFPImage,rfAggPas,rfLCL,rfFPGui,rfHTML);
  TFPReportExporterClass = Class of TFPReportExporter;

  { TReportDemoApp }

  TReportDemoApp = class(TComponent)
  private
    Frpt: TFPJSONReport;
  protected
    procedure InitialiseData; virtual;
    procedure CreateReportDesign; virtual;
  public
    procedure TestInit;
    Class Function Description : string; virtual;
//    procedure DoCreateJSON(const AFileName: String; RunTime: Boolean=False);
    Property rpt : TFPJSONReport read Frpt Write FRpt;
  end;
  TReportDemoAppClass = Class of TReportDemoApp;

  { TReportDemoApplication }

  { TReportRunner }
  TExporterEvent = Procedure(Sender :TObject; Exporter : TFPReportExporter) of object;

  TReportRunner = Class (TComponent)
  private
    FBaseOutputFileName: String;
    FDesignFileName: String;
    FLocation: String;
    FCreateJSON: Boolean;
    FOnInitExporter: TExporterEvent;
    FReportApp : TReportDemoApp;
    FExporter : TFPReportExporter;
    FFormat : TRenderFormat;
    FRunFileName: String;
  Protected
    Function  CreateReportExport : TFPReportExporter; virtual;
    procedure DoCreateJSON(const AFileName: String; RunTime: Boolean); virtual;
    procedure ExportReport; virtual;
    procedure RunReport(AFileName: string); virtual;
  Public
    destructor destroy; override;
    Procedure Execute;
    Property CreateJSON : Boolean Read FCreateJSON Write FCreateJSON;
    Property ReportApp : TReportDemoApp Read FReportApp Write FReportApp;
    Property Format : TRenderFormat Read FFormat Write FFormat;
    Property RunFileName : String Read FRunFileName Write FRunFileName;
    Property DesignFileName : String Read FDesignFileName Write FDesignFileName;
    Property Exporter : TFPReportExporter Read FExporter;
    Property Location : String Read FLocation Write FLocation;
    Property BaseOutputFileName : String Read FBaseOutputFileName Write FBaseOutputFileName;
    Property OnInitExporter : TExporterEvent Read FOnInitExporter Write FOnInitExporter;
   end;

  TReportDemoApplication = class(TCustomApplication)
  private
    FRunner: TReportRunner;
    procedure ListReports(AWithIndentation: boolean = False);
    procedure Usage(Msg: String);
  Class
    Var Reports : TStrings;
  Protected
    Property Runner : TReportRunner Read FRunner;
  public
    constructor Create(AOwner :TComponent) ; override;
    destructor  Destroy; override;
    procedure  DoRun; override;
    class function GetReportClass(AName: String): TReportDemoAppClass;
    Class Procedure RegisterReport(aName : String; AClasss : TReportDemoAppClass);
    Class Procedure GetRegisteredReports(aList : TStrings);
    Class function GetRenderClass(F: TRenderFormat): TFPReportExporterClass;
    Class Function FormatName(F : TRenderFormat) : String;
  end;

    { TReportDef }

  TReportDef = Class
    ReportClass: TReportDemoAppClass;
    Constructor create(AClass : TReportDemoAppClass);
  end;

implementation


class function TReportDemoApplication.FormatName(F: TRenderFormat): String;
begin
  Str(F,Result);
  delete(Result,1,2);
end;

{ TReportDemoApp }


procedure TReportDemoApp.InitialiseData;
begin
  // Do nothing
end;

procedure TReportDemoApp.CreateReportDesign;
begin
  if PaperManager.PaperCount=0 then
    PaperManager.RegisterStandardSizes;
end;

procedure TReportDemoApp.TestInit;
begin
  Frpt := TFPJSONReport.Create(Self);
  InitialiseData;
  CreateReportDesign;
end;

class function TReportDemoApp.Description: string;
begin
  Result:='';
end;


class function TReportDemoApplication.GetRenderClass(F: TRenderFormat): TFPReportExporterClass;

begin
  Case F of
  {$IFDEF ExportPDF}
     rfPDF: Result:=TFPReportExportPDF;
  {$ENDIF}
  {$IFDEF ExportFPIMAGE}
     rfFPImage: Result:=TFPReportExportFPImage;
  {$ENDIF}
  {$IFDEF ExportFPIMAGE}
     rfhtml: Result:=TFPReportExportHTML;
  {$ENDIF}
  {$IFDEF ExportAggPas}
     rfAggPas: Result:=TFPReportExportAggPas;
  {$ENDIF}
  {$IFDEF ExportLCL}
     rfLCL: Result:=TFPreportPreviewExport;
  {$ENDIF}
  {$IFDEF ExportFPGui}
     rfFPGui: Result := TFPreportPreviewExport;
  {$ENDIF}
  else
    Result:=Nil;
  end;
end;

function TReportRunner.CreateReportExport: TFPReportExporter;

Var
  C, Def : TFPReportExporterClass;
  F : TRenderFormat;

begin
  Result:=Nil;
  C:=Nil;
  Def:=Nil;
  {$IFDEF ExportLCL}
  def:=TFPreportPreviewExport;
  {$ENDIF}
  {$IFDEF ExportfpGUI}
  def:=fpreport_export_form.TFPreportPreviewExport;
  {$ENDIF}
  F:=Succ(rfDefault);
  While (Result=Nil) and (F<=High(TRenderFormat)) do
    begin
    C:=TReportDemoApplication.GetRenderClass(F);
    if (Def=Nil) and (C<>Nil) then
      Def:=C;
    if (F=FFormat) and (C<>Nil)  then
      Result:=C.Create(Self);
    F:=Succ(F);
    end;
  If (Result=Nil) then
    begin
    if (FFormat=rfDefault) then
      begin
      if Def=Nil then
        Raise Exception.Create('No default render format available. Please check the defines in udapp.pp')
      else
        Result:=Def.Create(Self);
      end
    else
      Raise Exception.Create('Requested format %s not available. Please check the defines in udapp.');
    end;
end;

destructor TReportRunner.destroy;
begin
  FreeAndNil(FReportApp);
  inherited destroy;
end;

procedure TReportRunner.Execute;

begin
  FReportApp.InitialiseData;
  FReportApp.CreateReportDesign;
  If (DesignFileName<>'') then
    DoCreateJSON(DesignFileName,False);
  RunReport(RunFileName);
  ExportReport;
end;

constructor TReportDemoApplication.Create(AOwner : TComponent);
begin
  Inherited;
  StopOnException:=True;
  FRunner:=TReportRunner.Create(Self);
  FRunner.Location:=Location;
end;

destructor TReportDemoApplication.Destroy;
begin
  FreeAndNil(FRunner);
  FreeAndNil(Reports);
  inherited Destroy;
end;

procedure TReportRunner.RunReport(AFileName : string);

begin
  // specify what directories should be used to find TrueType fonts
  gTTFontCache.SearchPath.Add(Location+'/fonts/');
{$IFDEF UNIX}
  gTTFontCache.SearchPath.Add(GetUserDir + '.fonts/');
  gTTFontCache.SearchPath.Add('/usr/share/fonts/truetype/ubuntu-font-family/');
  gTTFontCache.SearchPath.Add('/usr/share/fonts/truetype/dejavu/');
{$ENDIF}
  // ask to generate the font cache
  gTTFontCache.BuildFontCache;
  ReportApp.Rpt.RunReport;
  If (aFileName<>'') then
    DoCreateJSON(aFileName,True);
end;

Type
  THackFPReport = Class(TFPReport)
  Public
    Property RTObjects;
  end;

procedure TReportRunner.DoCreateJSON(const AFileName: String; RunTime: Boolean);

var
  F : Text;
  rs: TFPReportJSONStreamer;
  S :String;
begin
  rs := TFPReportJSONStreamer.Create(Nil);
  try
    if RunTime then
      TFPReportComponent(THackFPReport(FReportApp.rpt).RTObjects[0]).WriteElement(rs)
    else
      THackFPReport(FReportApp.rpt).WriteElement(rs);
    S:=rs.JSON.FormatJSON;
  finally
    rs.Free;
  end;
 // Write to file
  AssignFile(F,AFileName);
  Rewrite(F);
  Writeln(F,S);
  CloseFile(F);
end;

procedure TReportRunner.ExportReport;

begin
  FExporter:=CreateReportExport;
  try
    If Assigned(FOnInitExporter) then
      FOnInitExporter(Self,Exporter);
    {$IFDEF ExportLCL}
    If FExporter is TFPreportPreviewExport then
      Application.Initialize;
    {$ENDIF}
    {$IFDEF ExportFPGui}
    If FExporter is TFPreportPreviewExport then
      fpgApplication.Initialize;
    {$ENDIF}
    if (BaseOutputFileName<>'') and (FExporter.DefaultExtension<>'') then
      begin
      ForceDirectories(ExtractFilePath(BaseOutputFileName));
      FExporter.SetFileName(BaseOutputFileName);
      end;
    FReportApp.rpt.RenderReport(FExporter);
  finally
    FreeAndNil(FExporter);
  end;
end;

procedure TReportDemoApplication.Usage(Msg : String);

var
  F : TRenderFormat;

begin
  if (Msg<>'') then
  begin
    Writeln('Error : ',Msg);
    Writeln('');
  end;
  ExitCode:=Ord((Msg<>''));
  Writeln('Usage : ',ExtractFileName(ParamStr(0)),' [options]');
  Writeln('Where options is one of:');
  Writeln('-h --help          This help');
  Writeln('-l --list          List available reports.');
  Writeln('-j --json=file     Also write report design to JSON file.');
  Writeln('-f --format=FMT    Export format to use (use "default" for first, default format).');
  Writeln('-r --runtime=file  Also write first page of report runtime to JSON file.');
  Writeln('-d --demo=<name>   Run the demo specified by <name>.');
  Writeln('');

  Writeln('Known output formats for this binary: ');
  for F in  TRenderformat do
    if GetRenderClass(F)<>Nil then
    WriteLn('   ', FormatName(F));

  Writeln('');
  Writeln('Known demos for this binary: ');
  ListReports(True);
  ExitCode:=Ord(Msg<>'')
end;

procedure TReportDemoApplication.ListReports(AWithIndentation: boolean);
Var
  S : String;
  lIndent: string;
begin
  if AWithIndentation then
    lIndent := '  ';
  if Assigned(Reports) then
    for S in reports do
    begin
       Writeln(lIndent, s);
    end;
end;



{ TReportDef }

constructor TReportDef.create(AClass: TReportDemoAppClass);
begin
  ReportClass:=AClass;
end;

class function TReportDemoApplication.GetReportClass(AName: String
  ): TReportDemoAppClass;

Var
  I : Integer;

begin
  Result:=Nil;
  if Reports<>Nil then
    begin
    I:=Reports.IndexOf(AName);
    if I<>-1 then
      Result:=TReportDef(Reports.Objects[i]).ReportClass;
    end;
  if Result=Nil then
    Raise Exception.Create('No such demo : '+AName);
end;

class procedure TReportDemoApplication.RegisterReport(aName: String;
  AClasss: TReportDemoAppClass);
begin
  If Reports=Nil then
    begin
    Reports:=TStringList.Create;
    TStringList(Reports).Duplicates:=dupError;
    TStringList(Reports).Sorted:=True;
    TStringList(Reports).OwnsObjects:=True;
    end;
  Reports.AddObject(AName,TReportDef.Create(AClasss));
end;

class procedure TReportDemoApplication.GetRegisteredReports(aList: TStrings);
begin
  aList.Assign(reports);
end;

Var
  Demo : String;

Function GetReportAppName : string;

begin
  Result:='fpreportdemo';
  if (demo<>'') then
    Result:=Result+'-'+demo;
end;

procedure TReportDemoApplication.DoRun;

Var
  D,F,S,J : String;
  Fmt : TRenderFormat;

begin
  OnGetApplicationName:=@GetReportAppName;
  S:=CheckOptions('lj::hf:r:d:',['list','json::','help','format:','runtime:','demo:']);
  if (S<>'') or HasOption('h','help') then
  begin
    Usage(S);
    Terminate;
    exit;
  end;
  if HasOption('l','list') then
    begin
    ListReports;
    Terminate;
    exit;
    end;
  FRunner.RunFileName:=GetoptionValue('r','runtime');
  D:=GetOptionValue('d','demo');
  if (D='') then
    Usage('Need demo name');
  Demo:=D;
  if HasOption('j','json') then
    begin
    J:=GetOptionValue('j','json');
    if J='' then
      J:=ChangeFileExt(Paramstr(0),'.json');
    end;
  F:=GetOptionValue('f','format');
  Fmt:=High(TRenderFormat);
  While (Fmt>rfDefault) and (CompareText(FormatName(Fmt),F)<>0) do
    Fmt:=Pred(Fmt);
  if (F<>'') and (CompareText(F,'default')<>0) and (Fmt=rfDefault) then
    Usage(Format('Unknown output format: %s',[F]));
  FRunner.ReportApp:=GetReportClass(D).Create(Self);
  FRunner.ReportApp.rpt:=TFPJSONReport.Create(FRunner.ReportApp);
  FRunner.Format:=Fmt;
  FRunner.DesignFileName:=J;
  FRunner.Execute;

  Terminate;
end;

end.

