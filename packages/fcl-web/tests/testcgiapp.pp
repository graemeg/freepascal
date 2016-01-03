program testcgiapp;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils, CustApp, inifiles, process, httpdefs,custcgi
  { you can add units after this };

type

  { TTestCGIApplication }

  TTestCGIApplication = class(TCustomApplication)
  private
    FCGB: String;
    FCGIE: TStrings;
    FCGV: TStrings;
    FMethod: String;
    Foutput: String;
    FPostData: String;
    FPathInfo : String;
    FScriptName: String;
    FURL: String;
    procedure CheckEnvironment;
    procedure CheckMethod;
    procedure ProcessConfig;
    procedure RunCGI;
  protected
    Property CGIEnvironment : TStrings Read FCGIE Write FCGIE;
    Property URL : String Read FURL Write FURL;
    Property PostData : String Read FPostData Write FPostData;
    Property Method : String Read FMethod Write FMethod;
    Property CGIOutput : String Read Foutput Write FOutput;
    Property CGIBinary : String Read FCGB Write FCGB;
    Property CGIVariables : TStrings Read FCGV Write FCGV;
    Property PathInfo : String Read FPathInfo Write FPathInfo;
    Property ScriptName : String Read FScriptName Write FScriptName;
    procedure DoRun; override;
  public
    constructor Create(TheOwner: TComponent); override;
    Destructor Destroy; override;
    procedure WriteHelp; virtual;
  end;

{ TTestCGIApplication }

Resourcestring
   SErrUnsupportedMethod = 'Unsupported method: "%s"';
   SErrNoCGIBinary       = 'No CGI binary specified';

Const
  SConfig        = 'Config';
  KeyURL         = 'URL';
  KeyEnvironment = 'Environment';
  KeyMethod      = 'Method';
  KeyPost        = 'PostData';

  SEnvironment   = KeyEnvironment;
  SVariables     = 'Variables';


procedure TTestCGIApplication.ProcessConfig;

Var
  Ini : TInifile;
  S : String;

begin
  Ini:=TIniFile.Create(GetOptionValue('c','config'));
  try
    With Ini do
      begin
      URL:=ReadString(SConfig,KeyURL,'');
      S:=ReadString(SConfig,KeyEnvironment,'');
      If (S<>'') and FileExists(S) then
        CGIEnvironment.LoadFromFile(S);
      If SectionExists(SEnvironment) then
        ReadSectionValues(SEnvironment,CGIEnvironment);
      If SectionExists(SVariables) then
        ReadSectionValues(SVariables,CGIVariables);
      If (Method='') then
        Method:=ReadString(SConfig,KeyMethod,'GET');
      PostData:=ReadString(SConfig,KeyPost,'');

      end;
  finally
    Ini.Free;
  end;
end;

procedure TTestCGIApplication.RunCGI;

Var
  Proc : TProcess;

begin
  If (CGIBinary='') then
      Raise Exception.Create(SerrNoCGIBinary);
  Proc:=TProcess.Create(Self);
  try
    Proc.CommandLine:=CGIBinary;
    Proc.Environment:=CGIEnvironment;
    Proc.Execute;

  finally
    Proc.Free;
  end;
end;

procedure TTestCGIApplication.CheckMethod;

begin
  If (Method='') then
    Method:='GET'
  else
    begin
    Method:=Uppercase(Method);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    If (Method<>'POST') and (Method<>'GET') then
      Raise Exception.CreateFmt(SerrUnsupportedMethod,['METHOD']);
>>>>>>> graemeg/cpstrnew
=======
    If (Method<>'POST') and (Method<>'GET') then
      Raise Exception.CreateFmt(SerrUnsupportedMethod,['METHOD']);
>>>>>>> graemeg/cpstrnew
=======
    If (Method<>'POST') and (Method<>'GET') then
      Raise Exception.CreateFmt(SerrUnsupportedMethod,['METHOD']);
>>>>>>> graemeg/cpstrnew
=======
    If (Method<>'POST') and (Method<>'GET') then
      Raise Exception.CreateFmt(SerrUnsupportedMethod,['METHOD']);
>>>>>>> origin/cpstrnew
    end;
end;
(*
   ({ 1: 'AUTH_TYPE'               } fieldWWWAuthenticate, // ?
    { 2: 'CONTENT_LENGTH'          } FieldContentLength,
    { 3: 'CONTENT_TYPE'            } FieldContentType,
    { 4: 'GATEWAY_INTERFACE'       } '',
    { 5: 'PATH_INFO'               } '',
    { 6: 'PATH_TRANSLATED'         } '',
    { 7: 'QUERY_STRING'            } '',
    { 8: 'REMOTE_ADDR'             } '',
    { 9: 'REMOTE_HOST'             } '',
    { 10: 'REMOTE_IDENT'           } '',
    { 11: 'REMOTE_USER'            } '',
    { 12: 'REQUEST_METHOD'         } '',
    { 13: 'SCRIPT_NAME'            } '',
    { 14: 'SERVER_NAME'            } '',
    { 15: 'SERVER_PORT'            } '',
    { 16: 'SERVER_PROTOCOL'        } '',
    { 17: 'SERVER_SOFTWARE'        } '',
    { 18: 'HTTP_ACCEPT'            } FieldAccept,
    { 19: 'HTTP_ACCEPT_CHARSET'    } FieldAcceptCharset,
    { 20: 'HTTP_ACCEPT_ENCODING'   } FieldAcceptEncoding,
    { 21: 'HTTP_IF_MODIFIED_SINCE' } FieldIfModifiedSince,
    { 22: 'HTTP_REFERER'           } FieldReferer,
    { 23: 'HTTP_USER_AGENT'        } FieldUserAgent,
    { 24: 'HTTP_COOKIE'            } FieldCookie,
     // Additional Apache vars
    { 25: 'HTTP_CONNECTION'        } FieldConnection,
    { 26: 'HTTP_ACCEPT_LANGUAGE'   } FieldAcceptLanguage,
    { 27: 'HTTP_HOST'              } '',
    { 28: 'SERVER_SIGNATURE'       } '',
    { 29: 'SERVER_ADDR'            } '',
    { 30: 'DOCUMENT_ROOT'          } '',
    { 31: 'SERVER_ADMIN'           } '',
    { 32: 'SCRIPT_FILENAME'        } '',
    { 33: 'REMOTE_PORT'            } '',
    { 34: 'REQUEST_URI'            } '',
    { 35: 'CONTENT'                } '',
    { 36: 'XHTTPREQUESTEDWITH'     } ''

*)

procedure TTestCGIApplication.CheckEnvironment;

Var
  L : TStrings;
  S,N,V : String;
  I : Integer;

begin
  L:=CGIEnvironment;
  If L.IndexOfName('REQUEST_METHOD')=-1 then
    L.Values['REQUEST_METHOD']:=Method;
  S:=ScriptName;
  If (S='') then
    S:=CGIBinary;
  If L.IndexOfName('SCRIPT_NAME')=-1 then
    L.Values['SCRIPT_NAME']:=S;
  If L.IndexOfName('SCRIPT_FILENAME')=-1 then
    L.Values['SCRIPT_FILENAME']:=S;
  If (PathInfo<>'') then
    L.Values['PATH_INFO']:=PathInfo;
  If (Method='GET') then
    begin
    If L.IndexOfName('QUERY_STRING')=-1 then
      begin
      S:='';
      If (CGIVariables.Count>0) then
        For I:=0 to CGIVariables.Count-1 do
          begin
          CGIVariables.GetNameValue(I,N,V);
          If (S<>'') then
            S:=S+'&';
          S:=S+N+'='+HTTPEncode(V);
          end;
       L.Add('QUERY_STRING='+S)
       end;
    end
end;


procedure TTestCGIApplication.DoRun;
var
  ErrorMsg: String;
begin
  // parse parameters
  if HasOption('h','help') then begin
    WriteHelp;
    Terminate;
    Exit;
  end;
  if HasOption('c','config') then
    ProcessConfig;
  If HasOption('u','url') then
    URL:=GetOptionValue('u','url');
  If HasOption('e','environment') then
    CGIEnvironment.LoadFromFile(GetOptionValue('e','environment'));
  If HasOption('o','output') then
    CGIOutput:=GetOptionValue('o','output');
  If HasOption('m','method') then
    Method:=GetOptionValue('m','method');
  If HasOption('p','pathinfo') then
    PathInfo:=GetOptionValue('p','pathinfo');
  If HasOption('s','scriptname') then
    ScriptName:=GetOptionValue('s','scriptname');
  If HasOption('r','variables') then
    CGIOutput:=GetOptionValue('v','variables');
  If HasOption('i','input') then
    CGIBinary:=GetOptionValue('i','input');
  CheckMethod;
  CheckEnvironment;
  RunCGI;
  { add your program here }
  // stop program loop
  Terminate;
end;

constructor TTestCGIApplication.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  StopOnException:=True;
  FCGIE:=TStringList.Create;
  FCGV:=TStringList.Create;
end;

destructor TTestCGIApplication.Destroy;
begin
  FreeAndNil(FCGIE);
  FreeAndNil(FCGV);
  inherited Destroy;
end;

procedure TTestCGIApplication.WriteHelp;
begin
  Writeln('Usage: ',ExeName,' [options]');
  Writeln('Where options is one of : ');
  Writeln(' -h         this help');
  Writeln(' -c|--config=file         use file for configuration');
  Writeln(' -e|--environment=file    use file for CGI environment (overrides config).');
  Writeln(' -i|--input=file          use file as CGI binary.');
  Writeln(' -m|--method=method       use method to invoke CGI (overrides config, default is GET).');
  Writeln(' -o|--output=file         use file for CGI output (overrides config).');
  Writeln(' -p|--pathinfo=path       use path for PATH_INFO environment variable (overrides config).');
  Writeln(' -r|--variables=file      read query variables from file (overrides config).');
  Writeln(' -u|--url=URL             use URL as the URL (overrides config).');
end;

var
  Application: TTestCGIApplication;

begin
  Application:=TTestCGIApplication.Create(nil);
  Application.Title:='Test CGI application';
  Application.Run;
  Application.Free;
end.

