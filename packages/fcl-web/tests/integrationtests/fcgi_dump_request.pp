program fcgi_dump_request;

{$mode objfpc}{$H+}

uses
  Classes,
  SysUtils,
  httpDefs,
  fpweb,
  custweb,
  custfcgi,
  httpprotocol,
  fpjson,
  sha1;

Type

  { TMyCGIHandler }

  TMyCGIHandler = Class(TFCgiHandler)
  protected
   function GetSHA1FileHash(const AFileName: string): string;
  Public
    procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); override;
    function DumpHeaders(ARequest: TRequest): TJSONObject;
    function DumpContent(ARequest: TRequest): TJSONObject;
    function DumpFiles(ARequest: TRequest): TJSONArray;
  end;

  TMyCGIApp = Class(TCustomFCgiApplication)
  Protected
   function InitializeWebHandler: TWebHandler; override;
  end;

var
  WebApp: TMyCGIApp;

Procedure TMyCGIHandler.HandleRequest(ARequest : Trequest; AResponse : TResponse);
var
  ResponseJson: TJSONObject;
  ShouldShutdown: Boolean;
begin
  if not TryStrToBool(ARequest.QueryFields.Values['shutdown'], ShouldShutdown) then
    ShouldShutdown := False;

  ResponseJson := TJSONObject.Create();
  try
    ResponseJson.Add('headers', DumpHeaders(ARequest));
    ResponseJson.Add('content', DumpContent(ARequest));
    ResponseJson.Add('files', DumpFiles(ARequest));
    if ShouldShutdown then
      ResponseJson.Add('shutdown', True);
    AResponse.Content := ResponseJson.AsJSON;
  finally
    ResponseJson.Free;
  end;

  // Usefull for testing of memory-leaks
  if ShouldShutdown then
    WebApp.Terminate;
end;

function TMyCGIHandler.DumpHeaders(ARequest: TRequest): TJSONObject;
var
  HeaderJson: TJSONObject;
  CustomHeaderJson: TJSONObject;
  Header: THeader;
  i: Integer;
begin
  HeaderJson := TJSONObject.Create;
  try
    for Header := Low(Header) to High(Header) do
      if ARequest.HeaderIsSet(Header) then
        HeaderJson.Add(HTTPHeaderNames[Header], ARequest.GetHeader(Header));

    if ARequest.CustomHeaders.Count > 0 then
      begin
      CustomHeaderJson := TJSONObject.Create;
      HeaderJson.Add('custom', CustomHeaderJson);
      for i := 0 to ARequest.CustomHeaders.Count -1 do
        CustomHeaderJson.Add(ARequest.CustomHeaders.Names[i], ARequest.CustomHeaders.ValueFromIndex[i]);
      end;

    Result := HeaderJson;
    HeaderJson := nil;
  finally
    HeaderJson.Free;
  end;
end;

function TMyCGIHandler.DumpContent(ARequest: TRequest): TJSONObject;
var
  ContentJson: TJSONObject;
  ContentFieldJson: TJSONArray;
  i: Integer;
begin
  ContentJson := TJSONObject.Create;
  try
    if Length(ARequest.Content) > 250 then
      ContentJson.Add('rawTrimmed', TJSONObject.Create([
        'length', Length(ARequest.Content),
        'start', Copy(ARequest.Content, 1, 120),
        'end', Copy(ARequest.Content, Length(ARequest.Content)-79, 120)
      ]))
    else
      ContentJson.Add('raw', ARequest.Content);

    if ARequest.ContentFields.Count > 0 then
      begin
      ContentFieldJson := TJSONArray.Create;
      ContentJson.Add('fields', ContentFieldJson);
      for i := 0 to ARequest.ContentFields.Count -1 do
        begin
        ContentFieldJson.Add(TJSONObject.Create([
          'name', ARequest.ContentFields.Names[i],
          'value' , ARequest.ContentFields.ValueFromIndex[i]]));
        end;
      end;

    Result := ContentJson;
    ContentJson := nil;
  finally
    ContentJson.Free;
  end;
end;

function TMyCGIHandler.DumpFiles(ARequest: TRequest): TJSONArray;
var
  FileJsonArr: TJSONArray;
  FileJson: TJSONObject;
  i: Integer;
  UploadedFile: TUploadedFile;
begin
  FileJsonArr := TJSONArray.Create;
  try
    for i := 0 to ARequest.Files.Count -1 do
      begin
      UploadedFile := ARequest.Files.Files[i];
      FileJson := TJSONObject.Create([
        'filename', UploadedFile.FileName,
        'localFillename', UploadedFile.LocalFileName,
        'description', UploadedFile.Description,
        'contentType', UploadedFile.ContentType,
        'size', UploadedFile.Size,
        'disposition', UploadedFile.Disposition,
        'fieldname', UploadedFile.FieldName,
        'sha1', GetSHA1FileHash(UploadedFile.LocalFileName)
      ]);
      FileJsonArr.Add(FileJson);
      DeleteFile(UploadedFile.LocalFileName);
      end;

    Result := FileJsonArr;
    FileJsonArr := nil;
  finally
    FileJsonArr.Free;
  end;
end;

function TMyCGIHandler.GetSHA1FileHash(const AFileName: string): string;
begin
  if FileExists(AFileName) then
    Result := SHA1Print(SHA1File(AFileName));
end;

Function TMyCGIApp.InitializeWebHandler: TWebHandler; 
begin
  Result:=TMyCgiHandler.Create(self);
end;

begin
  WebApp := TMyCGIApp.Create(Nil);
  With WebApp do
    try
      { Uncomment the port setting here if you want to run the
       FastCGI application stand-alone (e.g. for NGINX) }
      Port := 7005;
      ProtocolOptions := [poReuseAddress];
      Initialize;
      Run;
    finally
      Free;
    end;
end.
