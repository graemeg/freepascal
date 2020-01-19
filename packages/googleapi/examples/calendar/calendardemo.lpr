program calendardemo;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Classes, SysUtils, CustApp,  fpoauth2ini, fphttpwebclient,  fpoauth2, jsonparser,
  IniFiles, googlebase, googleservice, googleclient, googlecalendar, opensslsockets;

type

  { TGoogleCalendarApplication }

  TGoogleCalendarApplication = class(TCustomApplication)
  private
    FSession,
    FLogFile,
    FConfig : String;
    FClient : TGoogleClient;
    FCalendarAPI: TCalendarAPI;
    procedure DoUserConsent(const AURL: String; out AAuthCode: String);
    procedure EnsureService;
    procedure ListCalendars;
    procedure ListEvents(aCalendarID: String);
  protected
    procedure DoRun; override;
  public
    constructor Create(TheOwner: TComponent); override;
    destructor Destroy; override;
    procedure WriteHelp(const Msg: String); virtual;
  end;

procedure TGoogleCalendarApplication.ListCalendars;

var
  Entry: TCalendarListEntry;
  Resource : TCalendarListResource;
  EN : String;
  i:integer;
  CalendarList: TCalendarList;

begin
  Resource:=Nil;
  try
    Resource:=FCalendarAPI.CreateCalendarListResource;
    CalendarList:=Resource.list('');
    I:=0;
    if assigned(CalendarList) then
      for Entry in CalendarList.items do
        begin
        Inc(i);
        EN:=Entry.ID;
        if Entry.Summary='' then
          EN:=EN+' ('+Entry.description+')'
        else
          EN:=EN+' ('+Entry.Summary+')';
        Writeln(I,': ID: ',EN);
        end;
  finally
    FClient.AuthHandler.SaveSession('me');
    FreeAndNil(Resource);
    FreeAndNil(CalendarList);
  end;
end;

procedure TGoogleCalendarApplication.ListEvents(aCalendarID : String);

var
  Events : TEvents;
  Entry: TEvent;
  EN : String;
  i:integer;

begin
  Events:=FCalendarAPI.EventsResource.list(aCalendarid,'');
  try
    I:=0;
    if assigned(Events) then
      for Entry in Events.items do
        begin
        Inc(i);
        EN:=Entry.Summary;
        if EN='' then
          EN:=Entry.id+' ('+Entry.description+')';
        if Assigned(Entry.Start) then
          if Entry.start.date<>0 then
            EN:=DateToStr(Entry.start.date)+' : '+EN
          else if Entry.start.dateTime<>0 then
            EN:=DateTimeToStr(Entry.start.datetime)+' : '+EN
          else
            EN:='(unspecified time) '+EN;
        Writeln(i,': '+EN);
        end;
  Finally
    FClient.AuthHandler.SaveSession(FSession);
    Events.Free;
  end;
end;


procedure TGoogleCalendarApplication.EnsureService;

{ TGoogleCalendarApplication }

Var
  FIS : TFPOAuth2IniStore;

begin
  // Auth client
  Writeln('Creating client');
  FCLient:=TGoogleClient.Create(Self);
  FClient.WebClient:=TFPHTTPWebClient.Create(Self);
  FClient.WebClient.LogFile:=FLogFile;
  FClient.AuthHandler:=TGoogleOAuth2Handler.Create(Self);
  Writeln('Creating client config store');
  FIS:=TFPOAuth2IniStore.Create(Self);
  FIS.ConfigFileName:=FConfig;
  FIS.SessionFileName:=FConfig;
  FClient.AuthHandler.Store:=FIS;
  Writeln('Loading config');
  FClient.AuthHandler.LoadConfig();
  Writeln('Loading session');
  FClient.AuthHandler.LoadSession(FSession);
  Writeln('Configuring local client');
  FClient.AuthHandler.Config.AuthScope:='https://www.googleapis.com/auth/calendar';
  FClient.AuthHandler.Config.AccessType:=atOffline;
  FClient.AuthHandler.Config.RedirectUri:='urn:ietf:wg:oauth:2.0:oob';
  // We want to enter a code.
  Fclient.OnUserConsent := @DoUserConsent;
  FClient.WebClient.RequestSigner:=FClient.AuthHandler;
  FClient.AuthHandler.WebClient:=FClient.WebClient;
  Writeln('Creating API');
  FCalendarAPI:=TCalendarAPI.Create(Self);
  FCalendarAPI.GoogleClient:=FClient;
end;

procedure TGoogleCalendarApplication.DoUserConsent(Const AURL: String; Out AAuthCode: String);
begin
   Writeln('');
   writeln('User consent required. Please open following URL:' );
   Writeln('');
   writeln(AURL);
   Writeln('');
   writeln('And copy/paste the authorization code here:');
   Writeln('');
   write('Code: ');
   ReadLn(AAuthCode);
   Writeln('');
   writeln('End user consent, returning to Google API...');
   Writeln('');
end;

procedure TGoogleCalendarApplication.DoRun;
var
  ErrorMsg: String;
begin
  // quick check parameters
  ErrorMsg:=CheckOptions('ho::c:s:le:',['help','logfile::','config:','session:','list','entry:']);
  if (ErrorMsg<>'') or HasOption('h', 'help') then
    WriteHelp(ErrorMsg);
  FLogFile:=GetOptionValue('o','logfile');
  if FLogFile='' then
    FLogFile:=ExtractFilePath(ParamStr(0))+'requests.log';
  FConfig:=GetOptionValue('c','config');
  if FConfig='' then
    FConfig:=ExtractFilePath(ParamStr(0))+'google.ini';
  FSession:=GetOptionValue('s','session');
  if FSession='' then
    FSession:='me';
  EnsureService;
  if HasOption('l','list') then
    ListCalendars
  else if HasOption('e','entry') then
    ListEvents(GetOptionValue('e','entry'))
  else // Default
    ListCalendars;
  Terminate;
end;

constructor TGoogleCalendarApplication.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  StopOnException:=True;
  TCalendarAPI.RegisterAPIResources;
end;

destructor TGoogleCalendarApplication.Destroy;
begin
  inherited Destroy;
end;

procedure TGoogleCalendarApplication.WriteHelp(const Msg: String);
begin
  If Msg<>'' then
    Writeln('Error : ',Msg);
  Writeln('Usage: ', ExeName, ' -s session [options]');
  Writeln('Where options : ');
  Writeln('-h --help            this help');
  Writeln('-c --config          config file with session and client data (default msgraph.ini)');
  Writeln('-o --logfile         config file with session and client data (default requests.log)');
  Writeln('-s --session=name    session to load from config file');
  Writeln('-l --list            list calendars (default action).');
  Writeln('-e --events=calID  list events from calendar name "calID" (ID of calendar)');
  Halt(Ord(Msg<>''));
end;

var
  Application: TGoogleCalendarApplication;
begin
  Application:=TGoogleCalendarApplication.Create(nil);
  Application.Title:='Google Calendar demo';
  Application.Run;
  Application.Free;
end.

