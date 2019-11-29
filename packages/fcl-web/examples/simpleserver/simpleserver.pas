{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2019 by the Free Pascal development team

    Sample HTTP server application

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}

{ $DEFINE USEGNUTLS}
{$DEFINE USEMICROHTTP}

program simpleserver;

{$IFDEF USEMICROHTTP}
{$UNDEF USEGNUTLS}
{$ENDIF}

uses



{$IFNDEF USEMICROHTTP}
{$ifdef USEGNUTLS}
  gnutlssockets,
{$else}
  opensslsockets,
{$endif}
  custhttpapp,
{$ELSE}
{$ifdef unix}
  cthreads,
{$endif}  
  custmicrohttpapp,
{$ENDIF}
  {$ifdef unix}
  baseunix,
  {$endif}
  sysutils,Classes, inifiles, sslbase, httproute, httpdefs, fpmimetypes, fpwebfile, fpwebproxy, webutil;

Type

  { THTTPApplication }
{$IFDEF USEMICROHTTP}
  TParentApp = TCustomMicroHTTPApplication;
{$ELSE}
  TParentApp = TCustomHTTPApplication;
{$ENDIF}

  THTTPApplication = Class(TParentApp)
  private
    FBaseDir: string;
    FIndexPageName: String;
    FMimeFile: String;
    FNoIndexPage: Boolean;
    FQuiet: Boolean;
    FBackground : Boolean;
    FPassword : string;
    FEcho : Boolean;
    FMaxAge : Integer;
    procedure AddProxy(const aProxyDef: String);
    procedure DoEcho(ARequest: TRequest; AResponse: TResponse);
    procedure DoProxyLog(Sender: TObject; const aMethod, aLocation, aFromURL, aToURL: String);
    procedure Doquit(ARequest: TRequest; AResponse: TResponse);
    procedure LoadMimeTypes;
    procedure ProcessOptions;
    procedure ReadConfigFile(const ConfigFile: string);
    procedure Usage(Msg: String);
    procedure Writeinfo;
  published
    procedure DoLog(EventType: TEventType; const Msg: String); override;
    Procedure DoRun; override;
    property Quiet : Boolean read FQuiet Write FQuiet;
    Property MimeFile : String Read FMimeFile Write FMimeFile;
    Property BaseDir : string Read FBaseDir Write FBaseDir;
    Property NoIndexPage : Boolean Read FNoIndexPage Write FNoIndexPage;
    Property IndexPageName : String Read FIndexPageName Write FIndexPageName;
  end;

Var
  Application : THTTPApplication;

{ THTTPApplication }

procedure THTTPApplication.DoEcho(ARequest: TRequest; AResponse: TResponse);

Var
  L : TStrings;

begin
  L:=TStringList.Create;
  try
    L.AddStrings(['<!doctype html>',
      '<html>',
      '<head>',
      '<title>Echo request</title>',
      '</head>',
      '<body>'
    ]);
    DumpRequest(aRequest,L);
    L.AddStrings(['</body>','</html>']);
    AResponse.Content:=L.Text;
    AResponse.SendResponse;
  finally
    L.Free;
  end;
end;
procedure THTTPApplication.Doquit(ARequest: TRequest; AResponse: TResponse);

Var
  PWD : String;

begin
  PWD:=ARequest.QueryFields.Values['password'];
  if PWD='' then
    ARequest.ContentFields.Values['password'];
  if PWD=FPassword then
    begin
    AResponse.Content:='OK';
    AResponse.SendContent;
    Terminate;
    end
  else
    begin
    AResponse.Code:=403;
    AResponse.CodeText:='Forbidden';
    AResponse.SendContent;
    end;
end;

procedure THTTPApplication.DoLog(EventType: TEventType; const Msg: String);
begin
  if Quiet then
    exit;
  if IsConsole then
    Writeln(FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz',Now),' [',EventType,'] ',Msg)
  else
    inherited DoLog(EventType, Msg);
end;

procedure THTTPApplication.DoProxyLog(Sender: TObject; const aMethod, aLocation, aFromURL, aToURL: String);

Var
  Msg : String;

begin
  if Quiet then
    exit;
  Msg:=Format('(Proxy redirect) location: %s, Method: %s, From: %s, to: %s',[aLocation,aMethod,aFromURl,atoURL]);
  if IsConsole then
    Writeln(FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz',Now),' [',etInfo,'] ',Msg)
  else
    inherited DoLog(etInfo, Msg);
end;

procedure THTTPApplication.Usage(Msg : String);

begin
  if (Msg<>'') then
    Writeln('Error: ',Msg);
  Writeln('Usage ',ExtractFileName(ParamStr(0)),' [options] ');
  Writeln('Where options is one or more of : ');
  Writeln('-c --config=file    Ini configuration file (default: simpleserver.ini)');
{$ifdef unix}
  Writeln('-b --background     fork to background');
{$endif}
  Writeln('-d --directory=dir  Base directory from which to serve files.');
  Writeln('                    Default is current working directory: ',GetCurrentDir);
  Writeln('-h --help           This help text');
  Writeln('-i --indexpage=name Directory index page to use (default: index.html)');
  Writeln('-n --noindexpage    Do not allow index page.');
  Writeln('-p --port=NNNN      TCP/IP port to listen on (default is 3000)');
  Writeln('-m --mimetypes=file path of mime.types. Loaded in addition to OS known types');
  Writeln('-q --quiet          Do not write diagnostic messages');
  Writeln('-Q --quit=PWD       register /quit url. Send request with password variable equal to PWD to stop');
  Writeln('-e --echo       register /quit url. Send request with password variable equal to PWD to stop');
  Writeln('-s --ssl            Use SSL');
  Writeln('-H --hostname=NAME  set hostname for self-signed SSL certificate');
  Writeln('-x --proxy=proxydef Add proxy definition. Definition is of form:');
  Writeln('                    name:BaseURL');
  Writeln('');
  Writeln('Config file is ini file, section [Server]. Key names are long option names');
  Writeln('Proxies are defined in section [Proxy], Key is name, value is URL');
  Halt(Ord(Msg<>''));
end;

procedure THTTPApplication.LoadMimeTypes;

begin
  MimeTypes.LoadKnownTypes;
  if MimeFile<>'' then
    begin
    MimeTypesFile:=MimeFile;
    if (MimeTypesFile<>'') and not FileExists(MimeTypesFile) then
      begin
      Log(etWarning,'mimetypes file not found: '+MimeTypesFile);
      MimeTypesFile:='';
      end;
    end;
  If MimeTypesFile<>'' then
    MimeTypes.LoadFromFile(MimeTypesFile);  
end;

procedure THTTPApplication.AddProxy(Const aProxyDef : String);

Var
  P : Integer;
  N,URL : String;

begin
  P:=Pos(':',aProxyDef);
  If P=0 then Raise
    EHTTP.CreateFmt('Invalid proxy definition: %s',[aProxyDef]);
  N:=Copy(aProxyDef,1,P-1);
  URL:=Copy(aProxyDef,P+1,Length(aProxyDef));
  ProxyManager.RegisterLocation(N,URL).AppendPathInfo:=True;
end;


procedure THTTPApplication.ReadConfigFile(Const ConfigFile : string);

Const
  SConfig  = 'Server';
  SProxy = 'Proxy';
  KeyPort  = 'Port';
  KeyDir   = 'Directory';
  KeyIndexPage = 'IndexPage';
  KeyHostName = 'hostname';
  keyMimetypes = 'mimetypes';
  KeySSL = 'SSL';
  KeyQuiet = 'quiet';
  KeyQuit = 'quit';
  KeyEcho = 'echo';
  KeyNoIndexPage = 'noindexpage';
  KeyBackground = 'background';
  KeyMaxAge = 'MaxAge';

Var
  L : TStringList;
  P,U : String;
  I : Integer;

begin
  if (ConfigFile='') or Not FileExists(ConfigFile) then exit;
  L:=Nil;
  With TMemIniFile.Create(ConfigFile) do
    try
      BaseDir:=ReadString(SConfig,KeyDir,BaseDir);
      Port:=ReadInteger(SConfig,KeyPort,Port);
      Quiet:=ReadBool(SConfig,KeyQuiet,Quiet);
      MimeFile:=ReadString(SConfig,keyMimetypes,MimeFile);
      NoIndexPage:=ReadBool(SConfig,KeyNoIndexPage,NoIndexPage);
      IndexPageName:=ReadString(SConfig,KeyIndexPage,IndexPageName);
      HostName:=ReadString(SConfig,KeyHostName,HostName);
      UseSSL:=ReadBool(SConfig,KeySSL,UseSSL);
      FBackground:=ReadBool(SConfig,Keybackground,FBackGround);
      FPassword:=ReadString(SConfig,KeyQuit,FPassword);
      FEcho:=ReadBool(SConfig,KeyEcho,FEcho);
      FMaxAge:=ReadInteger(SConfig,KeyMaxAge,FMaxAge);
      L:=TstringList.Create;
      ReadSectionValues(SProxy,L,[]);
      For I:=0 to L.Count-1 do
        begin
        L.GetNameValue(I,P,U);
        if (P<>'') and (U<>'') then
          ProxyManager.RegisterLocation(P,U).AppendPathInfo:=true;
        end;
    finally
      L.Free;
      Free;
    end;
end;

procedure THTTPApplication.ProcessOptions;

Var
  S : String;

begin
  for S in GetOptionValues('x','proxy') do
    AddProxy(S);
  FEcho:=HasOption('e','echo');
  Quiet:=HasOption('q','quiet');
  FPassword:=GetOptionValue('Q','quit');
  Port:=StrToIntDef(GetOptionValue('p','port'),Port);
  LoadMimeTypes;
  if HasOption('d','directory') then
    BaseDir:=GetOptionValue('d','directory');
  UseSSL:=HasOption('s','ssl');
  if HasOption('H','hostname') then
    HostName:=GetOptionValue('H','hostname');
  if HasOption('n','noindexpage') then
    NoIndexPage:=True
  else
    IndexPageName:=GetOptionValue('i','indexpage');
  FMaxAge:=StrToIntDef(GetOptionValue('a','max-age'),FMaxAge);
  FBackground:=HasOption('b','background');
end;

procedure THTTPApplication.Writeinfo;

Var
  I : Integer;

begin
  Log(etInfo,'Listening on port %d, serving files from directory: %s (using SSL: %s)',[Port,BaseDir,BoolToStr(UseSSL,'true','false')]);
  For I:=0 to ProxyManager.LocationCount-1 do
    with ProxyManager.Locations[i] do
      Log(etInfo,'Proxy location /proxy/%s redirects to %s',[Path,URL]);
  if not NoIndexPage then
    Log(etInfo,'Using index page %s',[IndexPageName]);
end;

procedure THTTPApplication.DoRun;

Var
  S,ConfigFile : String;

begin
  FMaxAge:=31557600;
  S:=Checkoptions('hqd:ni:p:sH:m:x:c:beQ:a:',['help','quiet','noindexpage','directory:','port:','indexpage:','ssl','hostname:','mimetypes:','proxy:','config:','background','echo','quit:','max-age:']);
  if (S<>'') or HasOption('h','help') then
    usage(S);
  if HasOption('c','config') then
    ConfigFile:=GetOptionValue('c','config')
  else
    ConfigFile:='simpleserver.ini';
  ReadConfigFile(ConfigFile);
  ProcessOptions;
  if FBackground then
    begin
{$ifdef unix}
    if FPFork>0 then Halt(0);
{$else}
    Log(erError,'Background option not supported');
{$endif}
    end;
  if FPassword<>'' then
    HTTPRouter.RegisterRoute('/quit',rmAll,@Doquit,False);
  if FEcho  then
    HTTPRouter.RegisterRoute('/echo',rmAll,@DoEcho,False);
  if ProxyManager.LocationCount>0 then
    begin
    TProxyWebModule.RegisterModule('Proxy',True);
    ProxyManager.OnLog:=@DoProxyLog;
    end;
  DefaultCacheControlMaxAge:=FMaxAge; // one year by default
  if BaseDir='' then
    BaseDir:=GetCurrentDir;
  if (BaseDir<>'') then
    BaseDir:=IncludeTrailingPathDelimiter(BaseDir);
  TSimpleFileModule.RegisterDefaultRoute;
  TSimpleFileModule.BaseDir:=BaseDir;
  TSimpleFileModule.OnLog:=@Log;
  If not NoIndexPage then
    begin
    if (IndexPageName='') then
      IndexPageName:='index.html';
    TSimpleFileModule.IndexPageName:=IndexPageName;
    end;
  if not Quiet then
    WriteInfo;
  inherited;
end;

begin
  Application:=THTTPApplication.Create(Nil);
  Application.Initialize;
  Application.Run;
  Application.Free;
end.

