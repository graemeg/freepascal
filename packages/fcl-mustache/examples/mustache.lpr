program mustache;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}
  cthreads,
  {$ENDIF}
  Classes, SysUtils, CustApp, strutils, fpjson, jsonparser, csvdataset, fpMustache, fpexmustache, fpdbmustache, iostream;

type

  { TMustacheApplication }

  TMustacheApplication = class(TCustomApplication)
  private
    FTemplate : TMustacheString;
    FJSON : TJSONStringType;
    FCSV: TCSVDataset;
    FPartials,
    FDefines : TStrings;
    FAllowExpressions : Boolean;
    Foutput,
    FSection,
    FRootPath : String;
    procedure DoGetDefine(const aName: TMustacheString; var aHandled: Boolean;
      var aValue: TMustacheString);
    procedure ProcessOptions;
    Procedure Createoutput;
    procedure Usage(ErrorMsg: String);
  protected
    procedure DoRun; override;

  public
    constructor Create(TheOwner: TComponent); override;
    destructor Destroy; override;
  end;

{ TMustacheApplication }

procedure TMustacheApplication.Usage(ErrorMsg : String);

begin
  If ErrorMsg<>'' then
    Writeln('Error : ',ErrorMsg);
  Writeln('Usage : mustache [options]');
  Writeln('Where options is one or more of:');
  writeln('-c --csv=FILE             Use a CSV file as data source. First line must contain column names.');
  writeln('-d --define=name=value    Define fixed value.');
  writeln('-e --expressions          Allow expressions.');
  writeln('-h --help                 This message.');
  writeln('-j --json=JSON            Use JSON as data source. @FILENAME will read JSON from file (UTF8).');
  writeln('-o --output=FILE          output file to write output to. If empty, stdout is assumed.');
  writeln('-p --partial=name=PARTIAL Register partial. @FILENAME reads partial from file.');
  writeln('-r --root=PATH            Register variables at root path PATH for expression engine');
  writeln('-s --section=SECTIOn    Section name for CSV data');
  writeln('-t --template=TEMPLATE    Use TEMPLATE as data source. @FILENAME will read template from file (UTF8). Required.');
  Halt(Ord(ErrorMsg<>''));
end;

procedure TMustacheApplication.ProcessOptions;

  Function StringOrFile(S : String) : UTF8String;

  begin
    if Copy(S,1,1)<>'@' then
      Result:=S
    else
      With TFileStream.Create(Copy(S,2,Length(S)-1),fmOpenRead or fmShareDenyNone) do
        try
          SetLength(Result,Size);
          ReadBuffer(Result[1],Size);
        finally
          Free;
        end;
  end;

Var
  S : String;

begin
  if Not HasOption('t','template') then
    Raise Exception.Create('Need a template');
  if HasOption('c','csv') and HasOption('j','json')  then
    Raise Exception.Create('Cannot specify both JSON or CSV');
  FTemplate:=StringOrFile(GetOptionValue('t','template'));
  if HasOption('j','json') then
    FJSON:=StringOrFile(GetOptionValue('j','json'))
  else if HasOption('c','csv') then
    begin
    FCSV:=TCSVDataset.Create(Self);
    FCSV.FileName:=GetOptionValue('c','csv');
    FCSV.CSVOptions.FirstLineAsFieldNames:=True;
    FCSV.Open;
    end;
  for S in GetOptionValues('d','define') do
    FDefines.Add(S);
  for S in GetOptionValues('p','partial') do
    FPartials.Add(ExtractWord(1,S,['='])+'='+StringOrFile(ExtractWord(2,S,['='])));
  FAllowExpressions:=HasOption('e','expressions');
  FRootPath:=GetOptionValue('r','root');
  FSection:=GetOptionValue('s','section');
  if FSection='' then
    FSection:='data';
  Foutput:=GetOptionValue('o','output');
end;

procedure TMustacheApplication.DoGetDefine(const aName: TMustacheString;
  var aHandled: Boolean; var aValue: TMustacheString);

Var
  Idx : Integer;

begin
  Writeln('Getting define ',aName);
  Idx:=FDefines.IndexOfName(aName);
  aHandled:=Idx<>-1;
  if aHandled then
    aValue:=FDefines.ValueFromIndex[Idx]
  else
    aValue:='';
end;

procedure TMustacheApplication.DoRun;
var
  ErrorMsg: String;
begin
  Terminate;
  // quick check parameters
  ErrorMsg:=CheckOptions('het:j:c:d:o:r:', ['help','template','json','csv','define','output','expressions','root']);
  if (ErrorMsg<>'') or HasOption('h','help') then
    Usage(ErrorMsg);
  ProcessOptions;
  CreateOutput;
end;

procedure TMustacheApplication.CreateOutput;

Var
  M : TMustache;
  C : TMustacheContext;
  O : TStream;
  S : TMustacheString;

begin
  O:=Nil;
  M:=Nil;
  C:=Nil;
  try
    if FAllowExpressions then
      M:=TMustache.Create(Self)
    else
      begin
      M:=TMustacheExpr.Create(Self);
      if (FRootPath<>'') and (FJSON<>'') then
        TMustacheExpr(M).RegisterVariables(FJSON,FRootPath,True);
      end;
    M.Partials:=FPartials;
    if Assigned(FCSV) then
      begin
      C:=TMustacheDBContext.Create(@DoGetDefine);
      TMustacheDBContext(C).AddDataset(FCSV,FSection);
      end
    else if (FJSON<>'') then
      C:=TMustacheJSONContext.Create(GetJSON(FJSON),@DoGetDefine)
    else
      C:=TMustacheContext.Create(@DoGetDefine);
    if Foutput<>'' then
      O:=TFileStream.Create(Foutput,fmCreate)
    else
      O:=TIOStream.Create(iosOutput);
    M.Template:=FTemplate;
    S:=M.Render(C);
    O.WriteBuffer(S[1],Length(S));
  finally
    O.Free;
    C.Free;
    M.Free;
  end;
end;

constructor TMustacheApplication.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  FPartials:=TStringList.Create;
  FDefines:=TStringList.Create;
  StopOnException:=True;
end;

destructor TMustacheApplication.Destroy;
begin
  FreeAndNil(FPartials);
  FreeAndNil(FDefines);
  FreeAndNil(FCSV);
  inherited Destroy;
end;


var
  Application: TMustacheApplication;
begin
  Application:=TMustacheApplication.Create(nil);
  Application.Title:='Mustache Templater';
  Application.Run;
  Application.Free;
end.

