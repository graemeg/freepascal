program simplehttpserver;

{$mode objfpc}{$H+}
{$define UseCThreads}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  sysutils, Classes, fphttpserver, fpmimetypes;

Type

  { TTestHTTPServer }

  TTestHTTPServer = Class(TFPHTTPServer)
  private
    FBaseDir : String;
    FCount : Integer;
    FMimeLoaded : Boolean;
    FMimeTypesFile: String;
    procedure SetBaseDir(const AValue: String);
  Protected
    Procedure DoIdle(Sender : TObject);
    procedure CheckMimeLoaded;

    Property MimeLoaded : Boolean Read FMimeLoaded;
  public
    procedure HandleRequest(Var ARequest: TFPHTTPConnectionRequest;
                            Var AResponse : TFPHTTPConnectionResponse); override;
    Property BaseDir : String Read FBaseDir Write SetBaseDir;
    Property MimeTypesFile : String Read FMimeTypesFile Write FMimeTypesFile;

  end;

Var
  Serv : TTestHTTPServer;
{ TTestHTTPServer }

procedure TTestHTTPServer.SetBaseDir(const AValue: String);
begin
  if FBaseDir=AValue then exit;
  FBaseDir:=AValue;
  If (FBaseDir<>'') then
    FBaseDir:=IncludeTrailingPathDelimiter(FBaseDir);
end;

procedure TTestHTTPServer.DoIdle(Sender: TObject);
begin
  Writeln('Idle, waiting for connections');
end;

procedure TTestHTTPServer.CheckMimeLoaded;
begin
  If (Not MimeLoaded) and (MimeTypesFile<>'') then
    begin
    MimeTypes.LoadFromFile(MimeTypesFile);
    FMimeLoaded:=true;
    end;
end;

procedure TTestHTTPServer.HandleRequest(var ARequest: TFPHTTPConnectionRequest;
  var AResponse: TFPHTTPConnectionResponse);

Var
  F : TFileStream;
  FN : String;

begin
  FN:=ARequest.Url;
  If (length(FN)>0) and (FN[1]='/') then
    Delete(FN,1,1);
  DoDirSeparators(FN);
  FN:=BaseDir+FN;
  if FileExists(FN) then
    begin
    F:=TFileStream.Create(FN,fmOpenRead);
    try
      CheckMimeLoaded;
      AResponse.ContentType:=MimeTypes.GetMimeType(ExtractFileExt(FN));
      Writeln('Serving file: "',Fn,'". Reported Mime type: ',AResponse.ContentType);
      AResponse.ContentLength:=F.Size;
      AResponse.ContentStream:=F;
      AResponse.SendContent;
      AResponse.ContentStream:=Nil;
    finally
      F.Free;
    end;
    end
  else
    begin
    AResponse.Code:=404;
    AResponse.SendContent;
    end;
  Inc(FCount);
  If FCount>=5 then
    Active:=False;
end;

begin
  Serv:=TTestHTTPServer.Create(Nil);
  try
    Serv.BaseDir:=ExtractFilePath(ParamStr(0));
{$ifdef unix}
    Serv.MimeTypesFile:='/etc/mime.types';
{$endif}
    Serv.Threaded:=False;
    Serv.Port:=8080;
    Serv.AcceptIdleTimeout:=1000;
    Serv.OnAcceptIdle:=@Serv.DoIdle;
    Serv.Active:=True;
  finally
    Serv.Free;
  end;
end.

