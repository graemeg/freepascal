{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2019 by the Free Pascal development team

    SQLDB REST dispatcher component.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit sqldbrestbridge;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, SQLDB, httpdefs, httproute, fpjson, sqldbrestschema, sqldbrestio, sqldbrestdata, sqldbrestauth;

Type
  TRestDispatcherOption = (rdoConnectionInURL,     // Route includes connection :Connection/:Resource[/:ID]
                           rdoExposeMetadata,      // expose metadata resource /metadata[/:Resource]
                           rdoCustomView,          // Expose custom view /customview
                           rdoHandleCORS,          // Handle CORS requests
                           rdoAccessCheckNeedsDB,  // Authenticate after connection to database was made.
                           rdoConnectionResource,   // Enable connection managament through /_connection[/:Conn] resource
                           rdoEmptyCORSDomainToOrigin // if CORSAllowedOrigins is empty CORS requests will mirror Origin instead of *
                           // rdoServerInfo        // Enable querying server info through /_serverinfo  resource
                           );

  TRestDispatcherOptions = set of TRestDispatcherOption;
  TRestDispatcherLogOption = (rloUser,           // Include username in log messages, when available
                              rtloHTTP,          // Log HTTP request (remote, URL)
                              rloResource,       // Log resource requests (operation, resource)
                              rloConnection,     // Log database connections (connect to database)
                              rloAuthentication, // Log authentication attempt
                              rloSQL,            // Log SQL statements. (not on user-supplied connection)
                              rloResultStatus    // Log result status.
                             );
  TRestDispatcherLogOptions = Set of TRestDispatcherLogOption;

Const
  DefaultDispatcherOptions = [rdoExposeMetadata];
  AllDispatcherLogOptions = [Low(TRestDispatcherLogOption)..High(TRestDispatcherLogOption)];
  DefaultDispatcherLogOptions = AllDispatcherLogOptions-[rloSQL];
  DefaultLogSQLOptions = LogAllEvents;

Type

  { TSQLDBRestConnection }

  TSQLDBRestConnection = Class(TCollectionItem)
  private
    FCharSet: UTF8String;
    FConnection: TSQLConnection;
    FConnectionType: String;
    FDatabaseName: UTF8String;
    FEnabled: Boolean;
    FHostName: UTF8String;
    FName: UTF8String;
    FParams: TStrings;
    FPassword: UTF8String;
    FPort: Word;
    FRole: UTF8String;
    FSchemaName: UTF8String;
    FUserName: UTF8String;
    FNotifier : TComponent;
    function GetName: UTF8String;
    procedure SetConnection(AValue: TSQLConnection);
    procedure SetParams(AValue: TStrings);
  Protected
    Function GetDisplayName: string; override;
    // For use in the REST Connection resource
    Property SchemaName : UTF8String Read FSchemaName Write FSchemaName;
  Public
    constructor Create(ACollection: TCollection); override;
    Destructor Destroy; override;
    Procedure Assign(Source: TPersistent); override;
    Procedure ConfigConnection(aConn : TSQLConnection); virtual;
  Published
    // Always use this connection instance
    Property SingleConnection : TSQLConnection Read FConnection Write SetConnection;
    // Allow this connection to be used.
    Property Enabled : Boolean Read FEnabled Write FEnabled default true;
    // TSQLConnector type
    property ConnectionType : String Read FConnectionType Write FConnectionType;
    // Name for this connection
    Property Name : UTF8String Read GetName Write FName;
    // Database user password
    property Password : UTF8String read FPassword write FPassword;
    // Database username
    property UserName : UTF8String read FUserName write FUserName;
    // Database character set
    property CharSet : UTF8String read FCharSet write FCharSet;
    // Database hostname
    property HostName : UTF8String Read FHostName Write FHostName;
    // Database role
    Property Role :  UTF8String read FRole write FRole;
    // Database database name
    property DatabaseName : UTF8String Read FDatabaseName Write FDatabaseName;
    // Other parameters
    Property Params : TStrings Read FParams Write SetParams;
    // Port DB is listening on
    Property Port : Word Read FPort Write FPort;
  end;

  { TSQLDBRestConnectionList }

  TSQLDBRestConnectionList = Class(TCollection)
  private
    function GetConn(aIndex : integer): TSQLDBRestConnection;
    procedure SetConn(aIndex : integer; AValue: TSQLDBRestConnection);
  Public
    // Index of connection by name (case insensitive)
    Function IndexOfConnection(const aName : UTF8string) : Integer;
    // Find connection by name (case insensitive), nil if none found
    Function FindConnection(const aName : UTF8string) :  TSQLDBRestConnection;
    // Add new instance, setting basic properties. Return new instance
    Function AddConnection(Const AType,aHostName,aDatabaseName,aUserName,aPassword : UTF8String) : TSQLDBRestConnection;
    // Save connection definitions to JSON file.
    Procedure SaveToFile(Const aFileName : UTF8String);
    // Save connection definitions  to JSON stream
    Procedure SaveToStream(Const aStream : TStream);
    // Return connection definitions as JSON object.
    function AsJSON(const aPropName: UTF8String=''): TJSONData; virtual;
    // Load connection definitions from JSON file.
    Procedure LoadFromFile(Const aFileName : UTF8String);
    // Load connection definitions from JSON stream.
    Procedure LoadFromStream(Const aStream : TStream);
    // Load connection definitions from JSON Object.
    Procedure FromJSON(aData: TJSONData;const aPropName: UTF8String=''); virtual;
    // Indexed access to connection definitions
    Property Connections [aIndex : integer] : TSQLDBRestConnection Read GetConn Write SetConn;  default;
  end;

  { TSQLDBRestSchemaRef }

  TSQLDBRestSchemaRef = Class(TCollectionItem)
  Private
    FEnabled: Boolean;
    Fschema: TSQLDBRestSchema;
    FNotifier : TComponent;
    procedure SetSchema(AValue: TSQLDBRestSchema);
  Protected
    Function GetDisplayName: String; override;
  Public
    Constructor Create(ACollection: TCollection); override;
    Destructor Destroy; override;
    Procedure Assign(Source: TPersistent); override;
  Published
    // Schema reference
    Property Schema : TSQLDBRestSchema Read FSchema Write SetSchema;
    // Allow this schema to be used ?
    Property Enabled: Boolean Read FEnabled Write FEnabled default true;
  end;

  { TSQLDBRestSchemaList }

  TSQLDBRestSchemaList = Class(TCollection)
  private
    function GetSchema(aIndex : Integer): TSQLDBRestSchemaRef;
    procedure SetSchema(aIndex : Integer; AValue: TSQLDBRestSchemaRef);
  Public
    Function AddSchema (aSchema : TSQLDBRestSchema) : TSQLDBRestSchemaRef;
    Function IndexOfSchema(aSchemaName : String) : Integer;
    Property Schemas[aIndex :Integer] : TSQLDBRestSchemaRef Read GetSchema Write SetSchema;default;
  end;



  { TSQLDBRestDispatcher }

  TResourceAuthorizedEvent = Procedure (Sender : TObject; aRequest : TRequest; Const aResource : UTF8String; var AllowResource : Boolean) of object;
  TGetConnectionNameEvent = Procedure(Sender : TObject; aRequest : TRequest; Const AResource : String; var AConnectionName : UTF8String) of object;
  TGetConnectionEvent = Procedure(Sender : TObject; aDef : TSQLDBRestConnection; var aConnection : TSQLConnection) of object;
  TRestExceptionEvent = Procedure(Sender : TObject; aRequest : TRequest; Const AResource : string; E : Exception) of object;
  TRestOperationEvent = Procedure(Sender : TObject; aConn: TSQLConnection; aResource : TSQLDBRestResource) of object;
  TRestGetFormatEvent = Procedure(Sender : TObject; aRest : TRequest; var aFormat : String) of object;
  TRestLogEvent = Procedure(Sender : TObject; aType : TRestDispatcherLogOption; Const aMessage : UTF8String) of object;

  TSQLDBRestDispatcher = Class(TComponent)
  Private
    Class Var FIOClass : TRestIOClass;
    Class Var FDBHandlerClass : TSQLDBRestDBHandlerClass;
  private
    FAdminUserIDs: TStrings;
    FCORSAllowCredentials: Boolean;
    FCORSAllowedOrigins: String;
    FCORSMaxAge: Integer;
    FDBLogOptions: TDBEventTypes;
    FDispatchOptions: TRestDispatcherOptions;
    FInputFormat: String;
    FCustomViewResource : TSQLDBRestResource;
    FLogOptions: TRestDispatcherLogOptions;
    FMetadataResource : TSQLDBRestResource;
    FMetadataDetailResource : TSQLDBRestResource;
    FConnectionResource : TSQLDBRestResource;
    FActive: Boolean;
    FAfterDelete: TRestOperationEvent;
    FAfterGet: TRestOperationEvent;
    FAfterPost: TRestOperationEvent;
    FAfterPut: TRestOperationEvent;
    FAuthenticator: TRestAuthenticator;
    FBaseURL: UTF8String;
    FBeforeDelete: TRestOperationEvent;
    FBeforeGet: TRestOperationEvent;
    FBeforePost: TRestOperationEvent;
    FBeforePut: TRestOperationEvent;
    FConnections: TSQLDBRestConnectionList;
    FDefaultConnection: UTF8String;
    FEnforceLimit: Integer;
    FOnAllowResource: TResourceAuthorizedEvent;
    FOnBasicAuthentication: TBasicAuthenticationEvent;
    FOnException: TRestExceptionEvent;
    FOnGetConnection: TGetConnectionEvent;
    FOnGetConnectionName: TGetConnectionNameEvent;
    FOnGetInputFormat: TRestGetFormatEvent;
    FOnGetOutputFormat: TRestGetFormatEvent;
    FOnLog: TRestLogEvent;
    FOutputFormat: String;
    FOutputOptions: TRestOutputoptions;
    FSchemas: TSQLDBRestSchemaList;
    FListRoute: THTTPRoute;
    FItemRoute: THTTPRoute;
    FConnectionsRoute: THTTPRoute;
    FConnectionItemRoute: THTTPRoute;
    FMetadataRoute: THTTPRoute;
    FMetadataItemRoute: THTTPRoute;
    FStatus: TRestStatusConfig;
    FStrings: TRestStringsConfig;
    function GetRoutesRegistered: Boolean;
    procedure SetActive(AValue: Boolean);
    procedure SetAdminUserIDS(AValue: TStrings);
    procedure SetAuthenticator(AValue: TRestAuthenticator);
    procedure SetConnections(AValue: TSQLDBRestConnectionList);
    procedure SetDispatchOptions(AValue: TRestDispatcherOptions);
    procedure SetSchemas(AValue: TSQLDBRestSchemaList);
    procedure SetStatus(AValue: TRestStatusConfig);
    procedure SetStrings(AValue: TRestStringsConfig);
  Protected
    // Logging
    Function MustLog(aLog : TRestDispatcherLogOption) : Boolean; inline;
    procedure DoSQLLog(Sender: TObject; EventType: TDBEventType;  const Msg: String); virtual;
    procedure DoLog(aLog: TRestDispatcherLogOption; IO : TRestIO; const aMessage: UTF8String);  virtual;
    procedure DoLog(aLog: TRestDispatcherLogOption; IO : TRestIO; const Fmt: UTF8String;
      Args: array of const);
    // Auxiliary methods.
    Procedure Loaded; override;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    function FindConnection(IO: TRestIO): TSQLDBRestConnection;
    // Factory methods. Override these to customize various helper classes.
    function CreateConnection: TSQLConnection; virtual;
    Function CreateConnectionList : TSQLDBRestConnectionList; virtual;
    Function CreateSchemaList : TSQLDBRestSchemaList; virtual;
    function CreateRestStrings: TRestStringsConfig; virtual;
    function CreateRestStatusConfig: TRestStatusConfig; virtual;
    function CreateDBHandler(IO: TRestIO): TSQLDBRestDBHandler; virtual;
    function CreateInputStreamer(IO: TRestIO): TRestInputStreamer; virtual;
    function CreateOutputStreamer(IO: TRestIO): TRestOutputStreamer; virtual;
    function CreateIO(aRequest: TRequest; aResponse: TResponse): TRestIO; virtual;
    function GetInputFormat(IO: TRestIO): String; virtual;
    function GetOutputFormat(IO: TRestIO): String; virtual;
    function GetConnectionName(IO: TRestIO): UTF8String;
    function GetSQLConnection(aConnection: TSQLDBRestConnection; Out aTransaction : TSQLTransaction): TSQLConnection; virtual;
    procedure DoneSQLConnection(aConnection: TSQLDBRestConnection; AConn: TSQLConnection; aTransaction : TSQLTransaction); virtual;
    // Connections dataset API
    procedure ConnectionsToDataset(D: TDataset); virtual;
    procedure DoConnectionDelete(DataSet: TDataSet); virtual;
    procedure DoConnectionPost(DataSet: TDataSet);virtual;
    procedure DatasetToConnection(D: TDataset; C: TSQLDBRestConnection); virtual;
    procedure ConnectionToDataset(C: TSQLDBRestConnection; D: TDataset); virtual;
    procedure DoConnectionResourceAllowed(aSender: TObject; aContext: TBaseRestContext; var allowResource: Boolean);
    // Error handling
    procedure CreateErrorContent(IO: TRestIO; aCode: Integer; AExtraMessage: UTF8String); virtual;
    procedure HandleException(E: Exception; IO: TRestIO); virtual;
    // REST request processing
    // Extract REST operation type from request
    procedure SetDefaultResponsecode(IO: TRestIO); virtual;
    // Must set result code and WWW-Authenticate header when applicable
    Function AuthenticateRequest(IO : TRestIO; Delayed : Boolean) : Boolean; virtual;
    function ExtractRestOperation(aRequest: TRequest;AccessControl : Boolean = false): TRestoperation; virtual;
    function FindRestResource(aResource: UTF8String): TSQLDBRestResource; virtual;
    function AllowRestResource(aIO : TRestIO): Boolean; virtual;
    function AllowRestOperation(aIO: TRestIO): Boolean; virtual;
    // Called twice: once before connection is established, once after.
    // checks rdoAccessCheckNeedsDB and availability of connection
    function CheckResourceAccess(IO: TRestIO): Boolean;
    function ExtractRestResourceName(IO: TRestIO): UTF8String; virtual;
    // Override if you want to create non-sqldb based resources
    function CreateSpecialResourceDataset(IO: TRestIO; AOwner: TComponent): TDataset; virtual;
    function IsSpecialResource(aResource: TSQLDBRestResource): Boolean; virtual;
    function FindSpecialResource(IO: TRestIO; aResource: UTF8String): TSQLDBRestResource; virtual;
    // Special resources for Metadata handling
    function CreateMetadataDataset(IO: TRestIO; AOwner: TComponent): TDataset; virtual;
    function CreateMetadataDetailDataset(IO: TRestIO; Const aResourceName : String; AOwner: TComponent): TDataset; virtual;
    function CreateConnectionDataset(IO: TRestIO; AOwner: TComponent): TDataset; virtual;
    function CreateMetadataDetailResource: TSQLDBRestResource;  virtual;
    function CreateMetadataResource: TSQLDBRestResource; virtual;
    Function CreateConnectionResource : TSQLDBRestResource; virtual;
    // Custom view handling
    function CreateCustomViewResource: TSQLDBRestResource; virtual;
    function CreateCustomViewDataset(IO: TRestIO; const aSQL: String; AOwner: TComponent): TDataset;
    procedure ResourceToDataset(R: TSQLDBRestResource; D: TDataset); virtual;
    procedure SchemasToDataset(D: TDataset);virtual;
    // General HTTP handling
    procedure DoRegisterRoutes; virtual;
    procedure DoHandleEvent(IsBefore : Boolean;IO: TRestIO); virtual;
    function ResolvedCORSAllowedOrigins(aRequest: TRequest): String; virtual;
    procedure HandleCORSRequest(aConnection: TSQLDBRestConnection; IO: TRestIO); virtual;
    procedure HandleResourceRequest(aConnection : TSQLDBRestConnection; IO: TRestIO); virtual;
    procedure DoHandleRequest(IO: TRestIO); virtual;
  Public
    Class Procedure SetIOClass (aClass: TRestIOClass);
    Class Procedure SetDBHandlerClass (aClass: TSQLDBRestDBHandlerClass);
    Constructor Create(AOWner : TComponent); override;
    Destructor Destroy; override;
    procedure RegisterRoutes;
    procedure UnRegisterRoutes;
    procedure HandleMetadataRequest(aRequest : TRequest; aResponse : TResponse);
    procedure HandleConnRequest(aRequest : TRequest; aResponse : TResponse);
    procedure HandleRequest(aRequest : TRequest; aResponse : TResponse);
    Procedure VerifyPathInfo(aRequest : TRequest);
    Function ExposeDatabase(Const aType,aHostName,aDatabaseName,aUserName,aPassword : String; aTables : Array of String; aMinFieldOpts : TRestFieldOptions = []) : TSQLDBRestConnection;
    Function ExposeDatabase(Const aType,aHostName,aDatabaseName,aUserName,aPassword : String; aTables : TStrings = nil; aMinFieldOpts : TRestFieldOptions = []) : TSQLDBRestConnection;
    Function ExposeConnection(aOwner : TComponent; Const aConnection : TSQLDBRestConnection; aTables : TStrings = nil; aMinFieldOpts : TRestFieldOptions = []) : TSQLDBRestSchema;
    Function ExposeConnection(Const aConnection : TSQLDBRestConnection; aTables : TStrings = nil; aMinFieldOpts : TRestFieldOptions = []) : TSQLDBRestSchema;
    Property RoutesRegistered : Boolean Read GetRoutesRegistered;
  Published
    // Register or unregister HTTP routes
    Property Active : Boolean Read FActive Write SetActive;
    // List of database connections to connect to
    Property Connections : TSQLDBRestConnectionList Read FConnections Write SetConnections;
    // List of REST schemas to serve
    Property Schemas : TSQLDBRestSchemaList Read FSchemas Write SetSchemas;
    // Base URL
    property BasePath : UTF8String Read FBaseURL Write FBaseURL;
    // Default connection to use if none is detected from request/schema
    // This connection will also be used to authenticate the user for connection API,
    // so it must be set if you use SQL to authenticate the user.
    Property DefaultConnection : UTF8String Read FDefaultConnection Write FDefaultConnection;
    // Input/Output strings configuration
    Property Strings : TRestStringsConfig Read FStrings Write SetStrings;
    // HTTP Status codes configuration
    Property Statuses : TRestStatusConfig Read FStatus Write SetStatus;
    // default Output options, modifiable by query.
    Property OutputOptions : TRestOutputOptions Read FOutputOptions Write FOutputOptions Default allOutputOptions;
    // Set this to allow only this input format.
    Property InputFormat : String Read FInputFormat Write FInputFormat;
    // Set this to allow only this output format.
    Property OutputFormat : String Read FOutputFormat Write FOutputFormat;
    // Dispatcher options
    Property DispatchOptions : TRestDispatcherOptions Read FDispatchOptions Write SetDispatchOptions default DefaultDispatcherOptions;
    // Authenticator for requests
    Property Authenticator : TRestAuthenticator Read FAuthenticator Write SetAuthenticator;
    // If >0, Enforce a limit on output results.
    Property EnforceLimit : Integer Read FEnforceLimit Write FEnforceLimit;
    // Domains that are allowed to use this REST service
    Property CORSAllowedOrigins: String Read FCORSAllowedOrigins  Write FCORSAllowedOrigins;
    // Access-Control-Max-Age header value. Set to zero not to send the header
    Property CORSMaxAge : Integer Read FCORSMaxAge Write FCORSMaxAge;
    // Access-Control-Allow-Credentials header value. Set to false not to send the header
    Property CORSAllowCredentials : Boolean Read FCORSAllowCredentials Write FCORSAllowCredentials;
    // UserIDs of the user(s) that are allowed to see and modify the connection resource.
    Property AdminUserIDs : TStrings Read FAdminUserIDs Write SetAdminUserIDS;
    // Logging options
    Property LogOptions : TRestDispatcherLogOptions Read FLogOptions write FLogOptions default DefaultDispatcherLogOptions;
    // SQL Log options. Only for connections managed by RestDispatcher
    Property LogSQLOptions : TDBEventTypes Read FDBLogOptions write FDBLogOptions default DefaultLogSQLOptions;
    // Called when Basic authentication is sufficient.
    Property OnBasicAuthentication : TBasicAuthenticationEvent Read FOnBasicAuthentication Write FOnBasicAuthentication;
    // Allow a particular resource or not.
    Property OnAllowResource : TResourceAuthorizedEvent Read FOnAllowResource Write FonAllowResource;
    // Called when determining the connection name for a request.
    Property OnGetConnectionName : TGetConnectionNameEvent Read FOnGetConnectionName Write FOnGetConnectionName;
    // Called when an exception happened during treatment of request.
    Property OnException : TRestExceptionEvent Read FOnException Write FOnException;
    // Called to get an actual connection.
    Property OnGetConnection : TGetConnectionEvent Read FOnGetConnection Write FOnGetConnection;
    // Called to determine input format based on request.
    Property OnGetInputFormat : TRestGetFormatEvent Read FOnGetInputFormat Write FOnGetInputFormat;
    // Called to determine output format based on request.
    Property OnGetOutputFormat : TRestGetFormatEvent Read FOnGetInputFormat Write FOnGetOutputFormat;
    // Called before a GET request.
    Property BeforeGet : TRestOperationEvent Read FBeforeGet Write FBeforeGet;
    // Called After a GET request.
    Property AfterGet : TRestOperationEvent Read FAfterGet Write FAfterGet;
    // Called before a PUT request.
    Property BeforePut : TRestOperationEvent Read FBeforePut Write FBeforePut;
    // Called After a PUT request.
    Property AfterPut : TRestOperationEvent Read FAfterPut Write FAfterPut;
    // Called before a POST request.
    Property BeforePost : TRestOperationEvent Read FBeforePost Write FBeforePost;
    // Called After a POST request.
    Property AfterPost : TRestOperationEvent Read FAfterPost Write FAfterPost;
    // Called before a DELETE request.
    Property BeforeDelete : TRestOperationEvent Read FBeforeDelete Write FBeforeDelete;
    // Called After a DELETE request.
    Property AfterDelete : TRestOperationEvent Read FAfterDelete Write FAfterDelete;
    // Called when logging
    Property OnLog : TRestLogEvent Read FOnLog Write FOnLog;
  end;

Const
  LogNames : Array[TRestDispatcherLogOption] of string = (
    'User','HTTP','Resource','Connection','Authentication','SQL','Result'
  );

implementation

uses uriparser, fpjsonrtti, DateUtils, bufdataset, sqldbrestjson, sqldbrestconst;

Type

  { TRestBufDataset }

  TRestBufDataset = class (TBufDataset)
  protected
    procedure LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField); override;
  end;


  { TSchemaFreeNotifier }

  TSchemaFreeNotifier = Class(TComponent)
    FRef : TSQLDBRestSchemaRef;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
  end;

  { TConnectionFreeNotifier }

  TConnectionFreeNotifier = Class(TComponent)
    FRef : TSQLDBRestConnection;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
  end;

{ TRestBufDataset }

procedure TRestBufDataset.LoadBlobIntoBuffer(FieldDef: TFieldDef; ABlobBuf: PBufBlobField);
begin
  If (FieldDef=Nil) or (aBlobBuf=Nil) then
    exit;
end;





{ TConnectionFreeNotifier }

procedure TConnectionFreeNotifier.Notification(AComponent: TComponent; Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (Operation=opRemove) and Assigned(FRef) and (Fref.SingleConnection=aComponent) then
    Fref.SingleConnection:=Nil;
end;

{ TSQLDBRestSchemaList }

function TSQLDBRestSchemaList.GetSchema(aIndex : Integer): TSQLDBRestSchemaRef;
begin
  Result:=TSQLDBRestSchemaRef(Items[aIndex]);
end;

procedure TSQLDBRestSchemaList.SetSchema(aIndex : Integer; AValue: TSQLDBRestSchemaRef);
begin
  Items[aIndex]:=aValue;
end;

function TSQLDBRestSchemaList.AddSchema(aSchema: TSQLDBRestSchema): TSQLDBRestSchemaRef;
begin
  Result:=(Add as TSQLDBRestSchemaRef);
  Result.Schema:=aSchema;
  Result.Enabled:=True;
end;

function TSQLDBRestSchemaList.IndexOfSchema(aSchemaName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and Not (Assigned(GetSchema(Result).Schema) and SameText(GetSchema(Result).Schema.Name,aSchemaName)) do
    Dec(Result);
end;

{ TSQLDBRestDispatcher }

procedure TSQLDBRestDispatcher.SetConnections(AValue: TSQLDBRestConnectionList);
begin
  if FConnections=AValue then Exit;
  FConnections.Assign(AValue);
end;

procedure TSQLDBRestDispatcher.SetDispatchOptions(AValue: TRestDispatcherOptions);

Var
  DeleteConnection : Boolean;

begin
  DeleteConnection:=(rdoConnectionInURL in FDispatchOptions) and Not (rdoConnectionInURL in aValue);
  if (rdoConnectionResource in aValue) then
    if DeleteConnection then // if user disables rdoConnectionInURL, we disable rdoConnectionResource.
      exclude(aValue,rdoConnectionResource)
    else // else we include rdoConnectionInURL...
      Include(aValue,rdoConnectionInURL);
  if FDispatchOptions=AValue then Exit;
  FDispatchOptions:=AValue;
end;

procedure TSQLDBRestDispatcher.DoConnectionResourceAllowed(aSender: TObject;
  aContext: TBaseRestContext; var allowResource: Boolean);
begin
  AllowResource:=(AdminUserIDs.Count=0) or  (AdminUserIDs.IndexOf(aContext.UserID)<>-1);
end;


procedure TSQLDBRestDispatcher.SetActive(AValue: Boolean);
begin
  if FActive=AValue then
    Exit;
  if Not (csLoading in ComponentState) then
    begin
    if AValue then
      DoRegisterRoutes
    else
      UnRegisterRoutes;
    end;
  FActive:=AValue;
end;

function TSQLDBRestDispatcher.GetRoutesRegistered: Boolean;
begin
  Result:=FItemRoute<>Nil;
end;

procedure TSQLDBRestDispatcher.SetAdminUserIDS(AValue: TStrings);
begin
  if FAdminUserIDs=AValue then Exit;
  FAdminUserIDs.Assign(AValue);
end;

procedure TSQLDBRestDispatcher.SetAuthenticator(AValue: TRestAuthenticator);
begin
  if FAuthenticator=AValue then Exit;
  if Assigned(FAuthenticator) then
    FAuthenticator.RemoveFreeNotification(Self);
  FAuthenticator:=AValue;
  if Assigned(FAuthenticator) then
    FAuthenticator.FreeNotification(Self);
end;

procedure TSQLDBRestDispatcher.SetSchemas(AValue: TSQLDBRestSchemaList);
begin
  if FSchemas=AValue then Exit;
  FSchemas.Assign(AValue);
end;

procedure TSQLDBRestDispatcher.SetStatus(AValue: TRestStatusConfig);
begin
  if FStatus=AValue then Exit;
  FStatus.Assign(AValue);
end;

procedure TSQLDBRestDispatcher.SetStrings(AValue: TRestStringsConfig);
begin
  if FStrings=AValue then Exit;
  FStrings.Assign(AValue);
end;

function TSQLDBRestDispatcher.MustLog(aLog: TRestDispatcherLogOption): Boolean;
begin
  Result:=aLog in FLogOptions;
end;

procedure TSQLDBRestDispatcher.DoSQLLog(Sender: TObject;  EventType: TDBEventType; const Msg: String);

Const
  EventNames : Array [TDBEventType] of string =
    ('Custom','Prepare', 'Execute', 'Fetch', 'Commit', 'RollBack', 'ParamValue', 'ActualSQL');

Var
  aMsg : UTF8String;

begin
  if not MustLog(rloSQl) then // avoid string ops
    exit;
  aMsg:=EventNames[EventType]+': '+Msg;
  if Sender is TRestIO then
    DoLog(rloSQL,TRestIO(Sender),aMsg)
  else
    DoLog(rloSQL,Nil,aMsg)
end;

procedure TSQLDBRestDispatcher.DoLog(aLog: TRestDispatcherLogOption; IO: TRestIO; const aMessage: UTF8String);

Var
  aMsg : UTF8String;

begin
  aMsg:='';
  if MustLog(aLog) and Assigned(FOnLog) then
     begin
     if MustLog(rloUser) and Assigned(IO) then
       begin
       if IO.UserID='' then
         aMsg:='(User: ?) '
       else
         aMsg:=Format('(User: %s) ',[IO.UserID]);
       end;
     aMsg:=aMsg+aMessage;
     FOnLog(Self,aLog,aMsg);
     end;
end;

procedure TSQLDBRestDispatcher.DoLog(aLog: TRestDispatcherLogOption;IO: TRestIO;
  const Fmt: UTF8String; Args: array of const);

Var
  S : UTF8string;

begin
  if not MustLog(aLog) then exit; // avoid expensive format
  try
    S:=Format(fmt,Args); // Encode ?
  except
    on E : exception do
      S:=Format('Error "%s" formatting "%s" with %d arguments: %s',[E.ClassName,Fmt,Length(Args),E.Message])
  end;
  DoLog(aLog,IO,S);
end;

procedure TSQLDBRestDispatcher.Loaded;
begin
  inherited Loaded;
  if FActive then
    RegisterRoutes;
end;

procedure TSQLDBRestDispatcher.HandleConnRequest(aRequest : TRequest; aResponse : TResponse);

begin
  aRequest.RouteParams['resource']:=Strings.ConnectionResourceName;
  HandleRequest(aRequest,aResponse);
end;

procedure TSQLDBRestDispatcher.HandleMetadataRequest(aRequest: TRequest;aResponse: TResponse);

Var
  LogMsg,UN : UTF8String;

begin
  if MustLog(rtloHTTP) then
    begin
    LogMsg:='';
    With aRequest do
      begin
      UN:=RemoteHost;
      if (UN='') then
        UN:=RemoteAddr;
      if (UN<>'') then
        LogMsg:='From: '+UN+'; ';
      LogMsg:=LogMsg+'URL: '+URL;
      end;
    UN:=TRestBasicAuthenticator.ExtractUserName(aRequest);
    if (UN<>'?') then
      LogMsg:='User: '+UN+LogMsg;
    DoLog(rtloHTTP,Nil,LogMsg);
    end;
  aRequest.RouteParams['resource']:=Strings.MetadataResourceName;
  HandleRequest(aRequest,aResponse);
end;

procedure TSQLDBRestDispatcher.DoRegisterRoutes;

Var
  Res,C : UTF8String;

begin
  Res:=IncludeHTTPPathDelimiter(BasePath);
  if (rdoConnectionResource in DispatchOptions) then
    begin
    C:=Strings.GetRestString(rpConnectionResourceName);
    FConnectionsRoute:=HTTPRouter.RegisterRoute(res+C,@HandleConnRequest);
    FConnectionItemRoute:=HTTPRouter.RegisterRoute(res+C+'/:id',@HandleConnRequest);
    end;
  if (rdoConnectionInURL in DispatchOptions) then
    begin
    // Both connection/metadata and /metadata must work.
    // connection/metadata is handled by HandleRequest (FindSpecialResource)
    // /metadata must be handled here.
    if (rdoExposeMetadata in DispatchOptions) then
      begin
      C:=Strings.GetRestString(rpMetadataResourceName);
      FMetadataRoute:=HTTPRouter.RegisterRoute(res+C,@HandleMetaDataRequest);
      FMetadataItemRoute:=HTTPRouter.RegisterRoute(res+C+'/:id',@HandleMetaDataRequest);
      end;
    Res:=Res+':connection/';
    end;
  Res:=Res+':resource';
  FListRoute:=HTTPRouter.RegisterRoute(res,@HandleRequest);
  FItemRoute:=HTTPRouter.RegisterRoute(Res+'/:id',@HandleRequest);
end;

function TSQLDBRestDispatcher.GetInputFormat(IO : TRestIO) : String;

// Order is: InputFormat setting, Content-type, input format, output format if it exists as input

Var
  U : UTF8String;
  D : TStreamerDef;

begin
  Result:=InputFormat;
  if (Result='') then
    begin
    if Result='' then
      if IO.GetVariable(Fstrings.GetRestString(rpInputFormat),U,[vsQuery])<>vsNone then
        Result:=U;
    if (Result='') and (IO.Request.ContentType<>'') then
      begin
      D:=TStreamerFactory.Instance.FindStreamerByContentType(rstInput,IO.Request.ContentType);
      if D<>Nil then
        Result:=D.MyName;
      end;
    if (Result='') then
      if IO.GetVariable(Fstrings.GetRestString(rpOutputFormat),U,[vsQuery])<>vsNone then
        begin
        if TStreamerFactory.Instance.FindStreamerByName(rstInput,U)<>Nil then
          Result:=U;
        end;
    end;
  If Assigned(FOnGetInputFormat) then
    FOnGetInputFormat(Self,IO.Request,Result)
end;

function TSQLDBRestDispatcher.GetOutputFormat(IO : TRestIO) : String;

// Order is: OutputFormat setting, output format, input Content-type, input format if it exists as output

Var
  U : UTF8String;
  D : TStreamerDef;

begin
  Result:=OutputFormat;
  if (Result='') then
    begin
    if IO.GetVariable(Fstrings.GetRestString(rpOutputFormat),U,[vsQuery])<>vsNone then
      Result:=U;
    if (Result='') and (IO.Request.ContentType<>'') then
      begin
      D:=TStreamerFactory.Instance.FindStreamerByContentType(rstOutput,IO.Request.ContentType);
      if D<>Nil then
        Result:=D.MyName;
      end;
    if Result='' then
      if IO.GetVariable(Fstrings.GetRestString(rpInputFormat),U,[vsQuery])<>vsNone then
        begin
        if TStreamerFactory.Instance.FindStreamerByName(rstOutput,U)<>Nil then
          Result:=U;
        end;
    end;
  If Assigned(FOnGetOutputFormat) then
    FOnGetOutputFormat(Self,IO.Request,Result)
end;

function TSQLDBRestDispatcher.CreateInputStreamer(IO : TRestIO): TRestInputStreamer;

Var
  D : TStreamerDef;
  aName : String;

begin
  aName:=GetInputFormat(IO);
  if aName='' then
    aName:='json';
  D:=TStreamerFactory.Instance.FindStreamerByName(rstInput,aName);
  if (D=Nil) then
    Raise ESQLDBRest.CreateFmt(FStatus.GetStatusCode(rsInvalidParam),SErrUnknownOrUnSupportedFormat,[aName]);
  Result:=TRestInputStreamer(D.MyClass.Create(IO.RequestContentStream,Fstrings,FStatus,@IO.DoGetVariable));
end;

function TSQLDBRestDispatcher.CreateOutputStreamer(IO : TRestIO): TRestOutputStreamer;

Var
  D : TStreamerDef;
  aName : String;

begin
  aName:=GetOutputFormat(IO);
  if aName='' then
    aName:='json';
  D:=TStreamerFactory.Instance.FindStreamerByName(rstOutput,aName);
  if (D=Nil) then
    Raise ESQLDBRest.CreateFmt(FStatus.GetStatusCode(rsInvalidParam),SErrUnknownOrUnSupportedFormat,[aName]);
  Result:=TRestOutputStreamer(D.MyClass.Create(IO.Response.ContentStream,Fstrings,FStatus,@IO.DoGetVariable));
end;


function TSQLDBRestDispatcher.CreateIO(aRequest: TRequest; aResponse: TResponse): TRestIO;

Var
  aInput : TRestInputStreamer;
  aOutput : TRestOutputStreamer;

begin
  aInput:=Nil;
  aOutput:=Nil;
  Result:=FIOClass.Create(aRequest,aResponse);
  try
    // Set up output
    Result.Response.ContentStream:=TMemoryStream.Create;
    Result.Response.FreeContentStream:=True;
    Result.SetRestStatuses(FStatus);
    Result.SetRestStrings(FStrings);
    aInput:=CreateInputStreamer(Result);
    aoutPut:=CreateOutPutStreamer(Result);
    Result.SetIO(aInput,aOutput);
    aInput:=Nil;
    aOutput:=Nil;
    aResponse.ContentType:=Result.RestOutput.GetContentType;
    Result.RestOutput.OutputOptions:=Result.GetRequestOutputOptions(OutputOptions);
  except
    On E : Exception do
      begin
      FreeAndNil(aInput);
      FreeAndNil(aOutput);
      FreeAndNil(Result);
      Raise;
      end;
  end;
end;

procedure TSQLDBRestDispatcher.CreateErrorContent(IO : TRestIO; aCode : Integer; AExtraMessage: UTF8String);

begin
  IO.Response.Code:=aCode;
  IO.Response.CodeText:=aExtraMessage;
  IO.RestOutput.CreateErrorContent(aCode,aExtraMessage);
  IO.RESTOutput.FinalizeOutput;
  IO.Response.ContentStream.Position:=0;
  IO.Response.ContentLength:=IO.Response.ContentStream.Size;
  IO.Response.SendResponse;
end;

class procedure TSQLDBRestDispatcher.SetIOClass(aClass: TRestIOClass);

begin
  FIOClass:=aClass;
  if FIOClass=Nil then
    FIOClass:=TRestIO;
end;

class procedure TSQLDBRestDispatcher.SetDBHandlerClass(aClass: TSQLDBRestDBHandlerClass);

begin
  FDBHandlerClass:=aClass;
  if FDBHandlerClass=Nil then
    FDBHandlerClass:=TSQLDBRestDBHandler;
end;

constructor TSQLDBRestDispatcher.Create(AOWner: TComponent);
begin
  inherited Create(AOWner);
  FStrings:=CreateRestStrings;
  FConnections:=CreateConnectionList;
  FSchemas:=CreateSchemaList;
  FOutputOptions:=allOutputOptions;
  FDispatchOptions:=DefaultDispatcherOptions;
  FLogOptions:=DefaultDispatcherLogOptions;
  FDBLogOptions:=DefaultLogSQLOptions;
  FStatus:=CreateRestStatusConfig;
  FCORSMaxAge:=SecsPerDay;
  FCORSAllowCredentials:=True;
  FAdminUserIDs:=TStringList.Create;
end;

destructor TSQLDBRestDispatcher.Destroy;
begin
  if RoutesRegistered then
    UnregisterRoutes;
  Authenticator:=Nil;
  FreeAndNil(FAdminUserIDs);
  FreeAndNil(FCustomViewResource);
  FreeAndNil(FMetadataResource);
  FreeAndNil(FMetadataDetailResource);
  FreeAndNil(FConnectionResource);
  FreeAndNil(FSchemas);
  FreeAndNil(FConnections);
  FreeAndNil(FStrings);
  FreeAndNil(FStatus);
  inherited Destroy;
end;

function TSQLDBRestDispatcher.CreateRestStrings : TRestStringsConfig;

begin
  Result:=TRestStringsConfig.Create
end;

function TSQLDBRestDispatcher.CreateRestStatusConfig: TRestStatusConfig;
begin
  Result:=TRestStatusConfig.Create;
end;

function TSQLDBRestDispatcher.ExtractRestResourceName(IO: TRestIO): UTF8String;

begin
  Result:=IO.Request.RouteParams['resource'];
  if (Result='') then
    Result:=IO.Request.QueryFields.Values[Strings.ResourceParam];
end;

function TSQLDBRestDispatcher.AllowRestResource(aIO: TRestIO): Boolean;

begin
  Result:=aIO.Resource.AllowResource(aIO.RestContext);
  if Assigned(FOnAllowResource) then
    FOnAllowResource(Self,aIO.Request,aIO.ResourceName,Result);
end;


function TSQLDBRestDispatcher.CreateCustomViewResource: TSQLDBRestResource;

begin
  Result:=TCustomViewResource.Create(Nil);
  Result.ResourceName:=FStrings.GetRestString(rpCustomViewResourceName);
  if rdoHandleCORS in DispatchOptions then
    Result.AllowedOperations:=[roGet,roOptions,roHead]
  else
    Result.AllowedOperations:=[roGet,roHead];
end;

function TSQLDBRestDispatcher.CreateMetadataResource: TSQLDBRestResource;

Var
  O : TRestOperation;
  S : String;

begin
  Result:=TSQLDBRestResource.Create(Nil);
  Result.ResourceName:=Strings.GetRestString(rpMetadataResourceName);
  if rdoHandleCORS in DispatchOptions then
    Result.AllowedOperations:=[roGet,roOptions,roHead]
  else
    Result.AllowedOperations:=[roGet,roHead];
  Result.Fields.AddField('name',rftString,[foRequired]).MaxLen:=255;
  Result.Fields.AddField('schemaName',rftString,[foRequired]).MaxLen:=255;
  for O in TRestOperation do
    if O<>roUnknown then
      begin
      Str(O,S);
      delete(S,1,2);
      Result.Fields.AddField(S,rftBoolean,[foRequired]);
      end;
end;

function TSQLDBRestDispatcher.CreateConnectionResource: TSQLDBRestResource;
Var
  Def : TRestFieldOptions;

begin
  Def:=[foInInsert,foInUpdate,foFilter];
  Result:=TSQLDBRestResource.Create(Nil);
  Result.ResourceName:=Strings.GetRestString(rpConnectionResourceName);
  Result.AllowedOperations:=[roGet,roPut,roPost,roDelete];
  if rdoHandleCORS in DispatchOptions then
    Result.AllowedOperations:=Result.AllowedOperations+[roOptions,roHead];
  Result.Fields.AddField('name',rftString,Def+[foInKey,foRequired]);
  Result.Fields.AddField('dbType',rftString,Def+[foRequired]);
  Result.Fields.AddField('dbName',rftString,Def+[foRequired]);
  Result.Fields.AddField('dbHostName',rftString,Def);
  Result.Fields.AddField('dbUserName',rftString,Def);
  Result.Fields.AddField('dbPassword',rftString,Def);
  Result.Fields.AddField('dbCharSet',rftString,Def);
  Result.Fields.AddField('dbRole',rftString,Def);
  Result.Fields.AddField('dbPort',rftInteger,Def);
  Result.Fields.AddField('enabled',rftBoolean,Def);
  Result.Fields.AddField('expose',rftBoolean,Def);
  Result.Fields.AddField('exposeSchemaName',rftString,Def);
  Result.OnResourceAllowed:=@DoConnectionResourceAllowed;
end;

function TSQLDBRestDispatcher.CreateMetadataDetailResource: TSQLDBRestResource;

Var
  O : TRestFieldOption;
  S : String;

begin
  Result:=TSQLDBRestResource.Create(Nil);
  Result.ResourceName:='metaDataField';
  if rdoHandleCORS in DispatchOptions then
    Result.AllowedOperations:=[roGet,roOptions,roHead]
  else
    Result.AllowedOperations:=[roGet,roHead];
  Result.Fields.AddField('name',rftString,[]).MaxLen:=255;
  Result.Fields.AddField('type',rftString,[]).MaxLen:=20;
  Result.Fields.AddField('maxlen',rftInteger,[]);
  Result.Fields.AddField('format',rftString,[]).MaxLen:=50;
  for O in TRestFieldOption do
    begin
    Str(O,S);
    delete(S,1,2);
    Result.Fields.AddField(S,rftBoolean,[]);
    end;
end;

function TSQLDBRestDispatcher.FindSpecialResource(IO : TRestIO; aResource: UTF8String): TSQLDBRestResource;

  Function IsCustomView : Boolean;inline;

  begin
    Result:=(rdoCustomView in DispatchOptions)
            and SameText(aResource,Strings.GetRestString(rpCustomViewResourceName));
  end;

  Function IsMetadata : Boolean;inline;

  begin
    Result:=(rdoExposeMetadata in DispatchOptions)
            and SameText(aResource,Strings.GetRestString(rpMetaDataResourceName));
  end;

  Function IsConnection : Boolean;inline;

  begin
    Result:=(rdoConnectionResource in DispatchOptions)
            and SameText(aResource,Strings.GetRestString(rpConnectionResourceName));
  end;

Var
  N : UTF8String;

begin
  Result:=Nil;
  If isCustomView then
    begin
    if FCustomViewResource=Nil then
      FCustomViewResource:=CreateCustomViewResource;
    Result:=FCustomViewResource;
    end
  else if IsConnection then
    begin
    if FConnectionResource=Nil then
      FConnectionResource:=CreateConnectionResource;
    Result:=FConnectionResource;
    end
  else If isMetadata then
    if (IO.GetVariable('ID',N,[vsRoute,vsQuery])=vsNone) then
      begin
      if FMetadataResource=Nil then
        FMetadataResource:=CreateMetadataResource;
      Result:=FMetadataResource;
      end
    else
      begin
      if FindRestResource(N)<>Nil then
        begin
        if FMetadataDetailResource=Nil then
          FMetadataDetailResource:=CreateMetadataDetailResource;
        Result:=FMetadataDetailResource;
        end;
      end
end;

function TSQLDBRestDispatcher.FindRestResource(aResource: UTF8String): TSQLDBRestResource;

Var
  I : integer;

begin
  Result:=Nil;
  I:=0;
  While (Result=Nil) and (I<Schemas.Count) do
    begin
    if Schemas[i].Enabled then
      Result:=Schemas[i].Schema.Resources.FindResourceByName(aResource);
    Inc(I);
    end;
end;

function TSQLDBRestDispatcher.ExtractRestOperation(aRequest: TRequest;AccessControl : Boolean = false): TRestoperation;

Var
  M : String;

begin
  Result:=roUnknown;
  if not AccessControl then
    M:=aRequest.Method
  else
    M:=aRequest.CustomHeaders.Values['Access-Control-Request-Method'];
  Case lowercase(M) of
    'get' : Result:=roGet;
    'put' : Result:=roPut;
    'post' : Result:=roPost;
    'delete' : Result:=roDelete;
    'options' : Result:=roOptions;
    'head' : Result:=roHead;
  end;
end;

Type

  { TRestSQLConnector }

  { THackSQLConnector }

  THackSQLConnector = Class(TSQLConnection)
  Public
    function DoGetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
  end;
  TRestSQLConnector = Class(TSQLConnector)
  Private
    FUse : Integer;
    FRequestCount : INteger;
  Protected
    function GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string; override;
    Procedure StartUsing;
    Function DoneUsing : Boolean;
  end;

{ THackSQLConnector }

function THackSQLConnector.DoGetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
begin
  Result:=GetNextValueSQL(SequenceName,IncrementBy);
end;

{ TRestSQLConnector }

function TRestSQLConnector.GetNextValueSQL(const SequenceName: string; IncrementBy: Integer): string;
begin
  Result:=THackSQLConnector(Proxy).DoGetNextValueSQL(SequenceName, IncrementBy);
end;

procedure TRestSQLConnector.StartUsing;
begin
  InterLockedIncrement(FUse);
  Inc(FRequestCount);
end;

function TRestSQLConnector.DoneUsing: Boolean;
begin
  InterLockedDecrement(Fuse);
  Result:=(FRequestCount>100) and (FUse=0);
end;

function TSQLDBRestDispatcher.CreateConnection : TSQLConnection;

begin
  Result:=TRestSQLConnector.Create(Self);
end;

function TSQLDBRestDispatcher.GetSQLConnection(
  aConnection: TSQLDBRestConnection; out aTransaction: TSQLTransaction
  ): TSQLConnection;

begin
  Result:=Nil;
  aTransaction:=Nil;
  if aConnection=Nil then
    exit;
  Result:=aConnection.SingleConnection;
  if (Result=Nil) then
    begin
    if Assigned(OnGetConnection) then
      OnGetConnection(Self,aConnection,Result);
    if (Result=Nil) then
      begin
      Result:=CreateConnection;
      aConnection.ConfigConnection(Result);
      aConnection.SingleConnection:=Result;
      end;
    end;
  If (Result is TRestSQLConnector) then
    TRestSQLConnector(Result).StartUsing;
  aTransaction:=TSQLTransaction.Create(Self);
  aTransaction.Database:=Result;
end;

procedure TSQLDBRestDispatcher.DoHandleEvent(IsBefore: Boolean; IO: TRestIO);

Var
  R : TRestOperationEvent;

begin
  R:=Nil;
  if isBefore then
    Case IO.Operation of
      roGet : R:=FBeforeGet;
      roPut : R:=FBeforePut;
      roPost : R:=FBeforePost;
      roDelete : R:=FBeforeDelete;
    end
  else
    Case IO.Operation of
      roGet : R:=FAfterGet;
      roPut : R:=FAfterPut;
      roPost : R:=FAfterPost;
      roDelete : R:=FAfterDelete;
    end;
  If Assigned(R) then
    R(Self,IO.Connection,IO.Resource)
end;



procedure TSQLDBRestDispatcher.DoneSQLConnection(
  aConnection: TSQLDBRestConnection; AConn: TSQLConnection;
  aTransaction: TSQLTransaction);

Var
  NeedNil : Boolean;

begin
  FreeAndNil(aTransaction);
  if (aConn is TRestSQLConnector) then
    begin
    NeedNil:= (aConnection.SingleConnection=aConn) ;
    if TRestSQLConnector(aConn).DoneUsing then
      FreeAndNil(aConn);
    If NeedNil then
      aConnection.SingleConnection:=Nil;
    end;
end;


function TSQLDBRestDispatcher.CreateDBHandler(IO: TRestIO): TSQLDBRestDBHandler;

begin
  Result:=FDBHandlerClass.Create(Self) ;
  Result.Init(IO,FStrings,TSQLQuery);
  Result.EnforceLimit:=Self.EnforceLimit;
end;


procedure TSQLDBRestDispatcher.SetDefaultResponsecode(IO : TRestIO);

Const
  DefaultCodes : Array[TRestOperation] of TRestStatus = (rsError,rsGetOK,rsPOSTOK,rsPUTOK,rsDeleteOK,rsCORSOK,rsGetOK);
  DefaultTexts : Array[TRestOperation] of string = ('Internal Error','OK','Created','OK','No content','OK','OK');

Var
  aCode : TRestStatus;
  aText : String;

begin
  aCode:=DefaultCodes[IO.Operation];
  aText:=DefaultTexts[IO.Operation];
  if IO.Response.Code=0 then
    IO.Response.Code:=FStatus.GetStatusCode(aCode);
  if (IO.Response.CodeText='') then
    IO.Response.CodeText:=aText;
end;

function TSQLDBRestDispatcher.IsSpecialResource(aResource: TSQLDBRestResource
  ): Boolean;

begin
  Result:=(aResource<>Nil);
  if not Result then exit;
  Result:=(aResource=FMetadataResource) or
          (aResource=FMetadataDetailResource) or
          (aResource=FConnectionResource) or
          (aResource=FCustomViewResource);
end;


procedure TSQLDBRestDispatcher.SchemasToDataset(D: TDataset);

Var
  S : TSQLDBRestSchema;
  R : TSQLDBRestResource;
  O : TRestOperation;
  I,J : Integer;
  SO : String;
  FName,FSchema : TField;
  FOperations : Array[TRestOperation] of TField;

begin
  FName:=D.FieldByName('name');
  FSchema:=D.FieldByName('schemaName');
  for O in TRestOperation do
    if O<>roUnknown then
      begin
      Str(O,SO);
      delete(SO,1,2);
      FOperations[O]:=D.FieldByName(SO);
      end;
  For I:=0 to Schemas.Count-1 do
    if Schemas[I].Enabled then
      begin
      S:=Schemas[I].Schema;
      For J:=0 to S.Resources.Count-1 do
        begin
        R:=S.Resources[J];
        if R.Enabled and R.InMetadata then
          begin
          D.Append;
          FName.AsString:=R.ResourceName;
          FSchema.AsString:=S.Name;
          for O in TRestOperation do
            if O<>roUnknown then
              FOperations[O].AsBoolean:=O in R.AllowedOperations;
          end;
        D.Post;
        end;
      end;
end;

function TSQLDBRestDispatcher.CreateMetadataDataset(IO: TRestIO;
  AOwner: TComponent): TDataset;

Var
  BD :  TRestBufDataset;
  O : TRestOperation;
  SO : String;

begin
  if IO=Nil then exit;
  BD:=TRestBufDataset.Create(aOwner);
  try
    Result:=BD;
    Result.FieldDefs.Add('name',ftString,255,False);
    Result.FieldDefs.Add('schemaName',ftString,255,False);
    for O in TRestOperation do
      if O<>roUnknown then
        begin
        Str(O,SO);
        delete(SO,1,2);
        Result.FieldDefs.Add(SO,ftBoolean,0,False);
        end;
    BD.CreateDataset;
    SchemasToDataset(BD);
    BD.First;
  except
    BD.Free;
    Raise;
  end;
end;

procedure TSQLDBRestDispatcher.ResourceToDataset(R: TSQLDBRestResource;
  D: TDataset);

Var
  F : TSQLDBRestField;
  O : TRestFieldOption;
  I : Integer;
  SO : String;
  FName,FType,fMaxLen,fFormat : TField;
  FOptions : Array[TRestFieldOption] of TField;

begin
  FName:=D.FieldByName('name');
  FType:=D.FieldByName('type');
  FMaxLen:=D.FieldByName('maxlen');
  FFormat:=D.FieldByName('format');
  for O in TRestFieldOption do
    begin
    Str(O,SO);
    delete(SO,1,2);
    FOptions[O]:=D.FieldByName(SO);
    end;
  For I:=0 to R.Fields.Count-1 do
    begin
    F:=R.Fields[i];
    D.Append;
    FName.AsString:=F.PublicName;
    Ftype.AsString:=TypeNames[F.FieldType];
    FMaxLen.AsInteger:=F.MaxLen;
    Case F.FieldType of
      rftDate : FFormat.AsString:=FStrings.GetRestString(rpDateFormat);
      rftDateTime : FFormat.AsString:=FStrings.GetRestString(rpDatetimeFormat);
      rftTime : FFormat.AsString:=FStrings.GetRestString(rpTimeFormat);
    end;
    for O in TRestFieldOption do
      FOptions[O].AsBoolean:=O in F.Options;
    D.Post;
    end;
end;

function TSQLDBRestDispatcher.CreateMetadataDetailDataset(IO: TRestIO;
  const aResourceName: String; AOwner: TComponent): TDataset;

Var
  BD :  TRestBufDataset;
  O : TRestFieldOption;
  SO : String;
  R : TSQLDBRestResource;

begin
  if IO=Nil then exit;
  BD:=TRestBufDataset.Create(aOwner);
  try
    Result:=BD;
    Result.FieldDefs.Add('name',ftString,255,False);
    Result.FieldDefs.Add('type',ftString,255,False);
    Result.FieldDefs.Add('maxlen',ftInteger,0,false);
    Result.FieldDefs.Add('format',ftString,50,false);
    for O in TRestFieldOption do
      begin
      Str(O,SO);
      delete(SO,1,2);
      Result.FieldDefs.Add(SO,ftBoolean,0,False);
      end;
    BD.CreateDataset;
    R:=FindRestResource(aResourceName);
    ResourceToDataset(R,BD);
    BD.First;
  except
    BD.Free;
    Raise;
  end;
end;

procedure TSQLDBRestDispatcher.DatasetToConnection(D: TDataset; C : TSQLDBRestConnection);

begin
  C.Name:=UTF8Encode(D.FieldByName('name').AsWideString);
  C.ConnectionType:=D.FieldByName('dbType').AsString;
  C.DatabaseName:=UTF8Encode(D.FieldByName('dbName').AsWideString);
  C.HostName:=D.FieldByName('dbHostName').AsString;
  C.UserName:=UTF8Encode(D.FieldByName('dbUserName').AsWideString);
  C.Password:=UTF8Encode(D.FieldByName('dbPassword').AsWideString);
  C.CharSet:=D.FieldByName('dbCharSet').AsString;
  C.Role:=D.FieldByName('dbRole').AsString;
  C.Port:=D.FieldByName('dbPort').AsInteger;
  C.Enabled:=D.FieldByName('enabled').AsBoolean;
  if D.FieldByName('expose').AsBoolean then
    C.SchemaName:=D.FieldByName('exposeSchemaName').AsString;
end;

procedure TSQLDBRestDispatcher.ConnectionToDataset(C : TSQLDBRestConnection;D: TDataset);

begin
  D.FieldByName('key').AsWideString:=UTF8Decode(C.Name);
  D.FieldByName('name').AsWideString:=UTF8Decode(C.Name);
  D.FieldByName('dbType').AsString:=C.ConnectionType;
  D.FieldByName('dbName').AsWideString:=UTF8Decode(C.DatabaseName);
  D.FieldByName('dbHostName').AsString:=C.HostName;
  D.FieldByName('dbUserName').AsWideString:=UTF8Decode(C.UserName);
  D.FieldByName('dbPassword').AsWideString:=UTF8Decode(C.Password);
  D.FieldByName('dbCharSet').AsString:=C.CharSet;
  D.FieldByName('dbRole').AsString:=C.Role;
  D.FieldByName('dbPort').AsInteger:=C.Port;
  D.FieldByName('enabled').AsBoolean:=C.Enabled;
  D.FieldByName('expose').AsBoolean:=(C.SchemaName<>'');
  D.FieldByName('exposeSchemaName').AsString:=C.SchemaName;
end;

procedure TSQLDBRestDispatcher.ConnectionsToDataset(D: TDataset);

Var
  C : TSQLDBRestConnection;
  I : Integer;

begin
  For I:=0 to Connections.Count-1 do
    begin
    C:=Connections[i];
    D.Append;
    ConnectionToDataset(C,D);
    D.Post;
    end;
end;

procedure TSQLDBRestDispatcher.DoConnectionDelete(DataSet: TDataSet);

Var
  I,J : Integer;
  C : TSQLDBRestConnection;


begin
  I:=Connections.IndexOfConnection(UTF8Encode(Dataset.FieldByName('name').AsWideString));
  if I<>-1 then
    begin
    C:=Connections[i];
    if C.SingleConnection<>Nil then
      DoneSQLConnection(C,C.SingleConnection,Nil);
    if C.SchemaName<>'' then
      begin
      J:=Schemas.IndexOfSchema(C.SchemaName);
      if J<>-1 then
        begin
        Schemas[J].Schema.Free;
        Schemas[J].Schema:=Nil;
        end;
      Schemas.Delete(J);
      end;
    Connections.Delete(I);
    end
  else
    Raise ESQLDBRest.Create(404,'NOT FOUND');
end;

procedure TSQLDBRestDispatcher.DoConnectionPost(DataSet: TDataSet);

Var
  isNew : Boolean;
  C : TSQLDBRestConnection;
  N : UTF8String;
  UN : UnicodeString;
  S : TSQLDBRestSchema;

begin
  IsNew:=Dataset.State=dsInsert;
  if IsNew then
    C:=Connections.Add as TSQLDBRestConnection
  else
    begin
    UN:=UTF8Decode(Dataset.FieldByName('key').AsString);
//    C:=Connections[Dataset.RecNo-1];
    C:=Connections.FindConnection(Utf8Encode(UN));
    if (C=Nil) then
      Raise ESQLDBRest.Create(404,'NOT FOUND');
    end;
  if Assigned(C.SingleConnection) then
    DoneSQLConnection(C,C.SingleConnection,Nil);
  DatasetToConnection(Dataset,C);
  if (Dataset.FieldByName('expose').AsBoolean) and isNew then
    begin
    N:=C.SchemaName;
    if N='' then
      N:=C.Name+'schema';
    if (Schemas.IndexOfSchema(N)<>-1) then
      Raise ESQLDBRest.Create(400,'DUPLICATE SCHEMA');
    try
      S:=ExposeConnection(C,Nil);
    except
      if IsNew then
        C.Free;
      Raise;
    end;
    S.Name:=N;
    end;
end;

function TSQLDBRestDispatcher.CreateConnectionDataset(IO: TRestIO; AOwner: TComponent): TDataset;
Var
  BD :  TRestBufDataset;

begin
  if IO=Nil then exit;
  BD:=TRestBufDataset.Create(aOwner);
  try
    Result:=BD;
    // Key field is not exposed
    Result.FieldDefs.add('key',ftWidestring,255);
    Result.FieldDefs.add('name',ftWidestring,255);
    Result.FieldDefs.add('dbType',ftString,20);
    Result.FieldDefs.add('dbName',ftWideString,255);
    Result.FieldDefs.add('dbHostName',ftString,255);
    Result.FieldDefs.add('dbUserName',ftWideString,255);
    Result.FieldDefs.add('dbPassword',ftWideString,255);
    Result.FieldDefs.add('dbCharSet',ftString,50);
    Result.FieldDefs.add('dbRole',ftString,255);
    Result.FieldDefs.add('dbPort',ftInteger,0);
    Result.FieldDefs.add('enabled',ftBoolean,0);
    Result.FieldDefs.add('expose',ftBoolean,0);
    Result.FieldDefs.add('exposeSchemaName',ftWideString,255);
    BD.CreateDataset;
    ConnectionsToDataset(BD);
    BD.IndexDefs.Add('uName','name',[ixUnique]);
    BD.IndexName:='uName';
    BD.First;
    BD.BeforePost:=@DoConnectionPost;
    BD.BeforeDelete:=@DoConnectionDelete;
  except
    BD.Free;
    Raise;
  end;
end;

function TSQLDBRestDispatcher.CreateCustomViewDataset(IO: TRestIO;
  const aSQL: String; AOwner: TComponent): TDataset;

Var
  Q : TRestSQLQuery;
  ST : TStatementType;

begin
  ST:=IO.Connection.GetStatementInfo(aSQL).StatementType;
  if (st<>stSelect) then
    raise ESQLDBRest.Create(FStatus.GetStatusCode(rsInvalidParam), SErrOnlySELECTSQLAllowedInCustomView); // Should never happen.
  Q:=TRestSQLQuery.Create(aOwner);
  try
    Q.DataBase:=IO.Connection;
    Q.Transaction:=IO.Transaction;
    Q.ParseSQL:=True;
    Q.SQL.Text:=aSQL;
    Result:=Q;
  except
    Q.Free;
    Raise;
  end;
end;


function TSQLDBRestDispatcher.CreateSpecialResourceDataset(IO: TRestIO;
  AOwner: TComponent): TDataset;

Var
  RN : UTF8String;

begin
  Result:=Nil;
  if (IO.Resource=FMetadataResource) then
    Result:=CreateMetadataDataset(IO,AOwner)
  else if (IO.Resource=FConnectionResource) then
    Result:=CreateConnectionDataset(IO,AOwner)
  else if (IO.Resource=FMetadataDetailResource) then
    begin
    if IO.GetVariable('ID',RN,[vsRoute,vsQuery])=vsNone then
      raise ESQLDBRest.Create(FStatus.GetStatusCode(rsError), SErrCouldNotFindResourceName); // Should never happen.
    Result:=CreateMetadataDetailDataset(IO,RN,AOwner)
    end
  else   if (IO.Resource=FCustomViewResource) then
    begin
    if IO.GetVariable(FStrings.GetRestString(rpCustomViewSQLParam),RN,[vsRoute,vsQuery])=vsNone then
      raise ESQLDBRest.Create(FStatus.GetStatusCode(rsInvalidParam), SErrNoSQLStatement); // Should never happen.
    Result:=CreateCustomViewDataset(IO,RN,aOwner);
    end
end;

function TSQLDBRestDispatcher.ResolvedCORSAllowedOrigins(aRequest : TRequest): String;

Var
  URl : String;
  uri : TURI;

begin
  Result:=FCORSAllowedOrigins;
  if Result='' then
    begin
    // Sent with CORS request
    Result:=aRequest.GetCustomHeader('Origin');
    if (Result='') and (rdoEmptyCORSDomainToOrigin in DispatchOptions) then
      begin
      // Fallback
      URL:=aRequest.Referer;
      if (URL<>'') then
        begin
        uri:=ParseURI(URL,'http',0);
        Result:=Format('%s://%s',[URI.Protocol,URI.Host]);
        if (URI.Port<>0) then
          Result:=Result+':'+IntToStr(URI.Port);
        end;
      end;
    end;
  if Result='' then
    Result:='*';
end;

procedure TSQLDBRestDispatcher.HandleCORSRequest(aConnection : TSQLDBRestConnection; IO : TRestIO);

Var
  S : String;
  Allowed : Boolean;


begin
  Allowed:=(rdoHandleCORS in DispatchOptions) and (roOptions in IO.Resource.AllowedOperations);
  if Allowed then
    Allowed:=(ExtractRestOperation(IO.Request,True) in ([roUnknown]+IO.Resource.AllowedOperations));
  if not Allowed then
    begin
    IO.Response.Code:=FStatus.GetStatusCode(rsCORSNotAllowed);
    IO.Response.CodeText:='FORBIDDEN';
    IO.CreateErrorResponse;
    end
  else
    begin
    IO.Response.SetCustomHeader('Access-Control-Allow-Origin',ResolvedCORSAllowedOrigins(IO.Request));
    S:=IO.Resource.GetHTTPAllow;
    IO.Response.SetCustomHeader('Access-Control-Allow-Methods',S);
    IO.Response.SetCustomHeader('Access-Control-Allow-Headers','x-requested-with, content-type, authorization');
    if CorsMaxAge>0 then
      IO.Response.SetCustomHeader('Access-Control-Max-Age',IntToStr(CorsMaxAge));
    IO.Response.SetCustomHeader('Access-Control-Allow-Credentials',BoolToStr(CORSAllowCredentials,'true','false'));
    IO.Response.Code:=FStatus.GetStatusCode(rsCORSOK);
    IO.Response.CodeText:='OK';
    end;
end;

procedure TSQLDBRestDispatcher.HandleResourceRequest(aConnection : TSQLDBRestConnection; IO : TRestIO);

Var
  Conn : TSQLConnection;
  TR : TSQLTransaction;
  H : TSQLDBRestDBHandler;
  l,o : Int64;


begin
  if MustLog(rloResource) then
    DoLog(rloResource,IO,'Resource: %s; Operation: %s',[IO.ResourceName,RestMethods[IO.Operation]]);
  H:=Nil;
  Conn:=GetSQLConnection(aConnection,Tr);
  try
    IO.SetConn(Conn,TR);
    Try
      if MustLog(rloConnection) then
         if Assigned(Conn)  then
           DoLog(rloConnection,IO,'Using connection to Host: %s; Database: %s',[Conn.HostName,Conn.DatabaseName])
         else
           DoLog(rloConnection,IO,'Resource %s does not require connection',[IO.ResourceName]);
      if assigned(Conn) and MustLog(rloSQL) then
        begin
        Conn.LogEvents:=LogSQLOptions;
        Conn.OnLog:=@IO.DoSQLLog;
        end;
      if (rdoHandleCORS in DispatchOptions) then
        begin
        IO.Response.SetCustomHeader('Access-Control-Allow-Origin',ResolvedCORSAllowedOrigins(IO.Request));
        IO.Response.SetCustomHeader('Access-Control-Allow-Credentials',BoolToStr(CORSAllowCredentials,'true','false'));
        end;
      if not AuthenticateRequest(IO,True) then
        exit;
      if Not CheckResourceAccess(IO) then
        exit;
      DoHandleEvent(True,IO);
      H:=CreateDBHandler(IO);
      if IsSpecialResource(IO.Resource) then
        begin
        H.ExternalDataset:=CreateSpecialResourceDataset(IO,H);
        if (IO.Resource=FCustomViewResource) then
          H.DeriveResourceFromDataset:=True;
        H.EmulateOffsetLimit:=IO.GetLimitOffset(EnforceLimit,l,o);
        end;
      H.ExecuteOperation;
      DoHandleEvent(False,IO);
      if Assigned(TR) then
        TR.Commit;
      SetDefaultResponseCode(IO);
    except
      TR.RollBack;
      Raise;
    end;
  finally
    IO.SetConn(Nil,Nil);
    DoneSQLConnection(aConnection,Conn,Tr);
  end;
end;

function TSQLDBRestDispatcher.GetConnectionName(IO: TRestIO): UTF8String;

Var
  N : UTF8String;
  R : TSQLDBRestResource;
begin
  R:=IO.Resource;
  N:='';
  if (N='') then
    N:=R.ConnectionName;
  if (N='') and assigned(R.GetSchema) then
    N:=R.GetSchema.ConnectionName;
  if (N='') then
    IO.GetVariable(Strings.ConnectionParam,N,[vsQuery]);
  if (N='') and (rdoConnectionInURL in DispatchOptions) then
    IO.GetVariable(Strings.ConnectionParam,N,[vsQuery]);
  If Assigned(FOnGetConnectionName) then
    FOnGetConnectionName(Self,IO.Request,R.ResourceName,N);
  if (N='') then
    N:=DefaultConnection;
  Result:=N;
end;

function TSQLDBRestDispatcher.FindConnection(IO: TRestIO): TSQLDBRestConnection;

Var
  N : UTF8String;

begin
  N:=GetConnectionName(IO);
  // If we have a name, look for it
  if (N<>'') then
    begin
    Result:=Connections.FindConnection(N);
    if Assigned(Result) and not (Result.Enabled) then
      Result:=Nil;
    end
  else if Connections.Count=1 then
    Result:=Connections[0]
  else
    Result:=Nil;
end;

function TSQLDBRestDispatcher.CreateConnectionList: TSQLDBRestConnectionList;
begin
  Result:=TSQLDBRestConnectionList.Create(TSQLDBRestConnection);

end;

function TSQLDBRestDispatcher.CreateSchemaList: TSQLDBRestSchemaList;
begin
  Result:=TSQLDBRestSchemaList.Create(TSQLDBRestSchemaRef);
end;

function TSQLDBRestDispatcher.AllowRestOperation(aIO: TRestIO): Boolean;

begin
  Result:=(aIO.Operation in aIO.Resource.GetAllowedOperations(aIO.RestContext));
end;

function TSQLDBRestDispatcher.CheckResourceAccess(IO: TRestIO): Boolean;

Var
  NeedDB : Boolean;

begin
  NeedDB:=(rdoAccessCheckNeedsDB in DispatchOptions);
  Result:=NeedDB<>Assigned(IO.Connection);
  if Result then
    exit;
  Result:=AllowRestResource(IO);
  if not Result then
    CreateErrorContent(IO,FStatus.GetStatusCode(rsResourceNotAllowed),'FORBIDDEN')
  else
    begin
    Result:=AllowRestOperation(IO);
    if not Result then
      CreateErrorContent(IO,FStatus.GetStatusCode(rsRestOperationNotAllowed),'METHOD NOT ALLOWED')
    end;
end;

procedure TSQLDBRestDispatcher.DoHandleRequest(IO : TRestIO);

var
  ResourceName : UTF8String;
  Operation : TRestOperation;
  Resource : TSQLDBRestResource;
  Connection : TSQLDBRestConnection;

begin
  Operation:=ExtractRestOperation(IO.Request);
  if (Operation=roUnknown) then
    CreateErrorContent(IO,FStatus.GetStatusCode(rsInvalidMethod),'INVALID METHOD')
  else
    begin
    IO.SetOperation(Operation);
    ResourceName:=ExtractRestResourceName(IO);
    if (ResourceName='') then
      CreateErrorContent(IO,FStatus.GetStatusCode(rsNoResourceSpecified),'INVALID RESOURCE')
    else
      begin
      Resource:=FindSpecialResource(IO,ResourceName);
      If Resource=Nil then
        Resource:=FindRestResource(ResourceName);
      if Resource=Nil then
        CreateErrorContent(IO,FStatus.GetStatusCode(rsUnknownResource),'NOT FOUND')
      else
        begin
        IO.SetResource(Resource);
        Connection:=FindConnection(IO);
        if (Connection=Nil) and not IsSpecialResource(Resource) then
          begin
          if (rdoConnectionInURL in DispatchOptions) then
            CreateErrorContent(IO,FStatus.GetStatusCode(rsNoConnectionSpecified),Format(SErrNoconnection,[GetConnectionName(IO)]))
          else
            CreateErrorContent(IO,FStatus.GetStatusCode(rsError), Format(SErrNoconnection,[GetConnectionName(IO)]));
          end
        else if CheckResourceAccess(IO) then
          if Operation=roOptions then
            HandleCORSRequest(Connection,IO)
          else
            HandleResourceRequest(Connection,IO);
        end;
      end;
    end;
end;

procedure TSQLDBRestDispatcher.UnRegisterRoutes;

  Procedure Un(Var a : THTTPRoute);

  begin
    if A=Nil then
      exit;
    HTTPRouter.DeleteRoute(A);
    A:=Nil;
  end;

begin
  Un(FListRoute);
  Un(FItemRoute);
  Un(FConnectionItemRoute);
  Un(FConnectionsRoute);
  Un(FMetadataItemRoute);
  Un(FMetadataRoute);
end;


procedure TSQLDBRestDispatcher.RegisterRoutes;
begin
  if (FListRoute<>Nil) then
    UnRegisterRoutes;
  DoRegisterRoutes;
end;

procedure TSQLDBRestDispatcher.HandleException(E : Exception; IO : TRestIO);

  Function StripCR(S : String) : String;
  begin
    Result:=StringReplace(S,#13#10,' ',[rfReplaceAll]);
    Result:=StringReplace(Result,#13,' ',[rfReplaceAll]);
    Result:=StringReplace(Result,#10,' ',[rfReplaceAll]);
  end;

Var
  Code : Integer;
  Msg : String;

begin
  try
    if Assigned(FOnException) then
      FOnException(Self,IO.Request,IO.ResourceName,E);
    if not IO.Response.ContentSent then
      begin
      Code:=0;
      If E is ESQLDBRest then
        begin
        Code:=ESQLDBRest(E).ResponseCode;
        Msg:=E.Message;
        end;
      if (Code=0) then
        begin
        Code:=FStatus.GetStatusCode(rsError);
        Msg:=Format(SErrUnexpectedException,[E.ClassName,E.Message]);
        end;
      IO.Response.Code:=Code;
      IO.Response.CodeText:=StripCR(Msg);
      if (IO.Response.Code=405) and Assigned(IO.Resource) then
        IO.Response.Allow:=IO.Resource.GetHTTPAllow; // ([rmHead,rmOptions]) ?
      IO.RESTOutput.CreateErrorContent(Code,Msg);
      end;
  except
    on Ex : exception do
     begin
     IO.Response.Code:=FStatus.GetStatusCode(rsError);
     IO.Response.CodeText:=Format('Unexpected exception %s while handling original exception %s : "%s" (Original: "%s")',[Ex.ClassName,E.ClassName,Ex.Message,E.Message]);
     end;
  end;
end;

function TSQLDBRestDispatcher.AuthenticateRequest(IO: TRestIO; Delayed : Boolean): Boolean;

Var
  B : TRestBasicAuthenticator;
  A : TRestAuthenticator;


begin
  A:=Nil;
  B:=Nil;
  If Assigned(FAuthenticator) then
    A:=FAuthenticator
  else If Assigned(FOnBAsicAuthentication) then
    begin
    B:=TRestBasicAuthenticator.Create(Self);
    A:=B;
    B.OnBasicAuthentication:=Self.OnBasicAuthentication;
    end;
  try
    Result:=A=Nil;
    if Not Result Then
      begin
      Result:=(A.NeedConnection<>Delayed);
      If Not Result then
        begin
        Result:=A.AuthenticateRequest(IO);
        if MustLog(rloAuthentication) then
          if Result then
            DoLog(rloAuthentication,IO,'Authenticated user: %s',[IO.UserID])
          else
            DoLog(rloAuthentication,IO,'Authentication failed for user: %s',[TRestBasicAuthenticator.ExtractUserName(IO.Request)]);
        end;
      end;
  finally
    if Assigned(B) then
      B.Free;
  end;
end;

procedure TSQLDBRestDispatcher.Notification(AComponent: TComponent;
  Operation: TOperation);

begin
  inherited Notification(AComponent, Operation);
  if Operation=opRemove then
    begin
    if AComponent=FAuthenticator then
      FAuthenticator:=Nil
    end;
end;

procedure TSQLDBRestDispatcher.HandleRequest(aRequest: TRequest; aResponse: TResponse);

Var IO : TRestIO;

begin
  aResponse.Code:=0; // Sentinel
  IO:=CreateIO(aRequest,aResponse);
  try
    try
      // Call initstreaming only here, so IO has set var callback.
      // First output, then input
      IO.RestOutput.InitStreaming;
      IO.RestInput.InitStreaming;
      IO.OnSQLLog:=@Self.DoSQLLog;
      if SameText('OPTIONS',aRequest.Method) or AuthenticateRequest(IO,False) then
        DoHandleRequest(IO)
    except
      On E : Exception do
        HandleException(E,IO);
    end;
  Finally
    // Make sure there is a document in case of error
    // MVC: Disabled for the moment, we need more reliable detection of this. it adds error twice in case of exception.
    // if (aResponse.ContentStream.Size=0) and Not ((aResponse.Code div 100)=2) then
    //  IO.RESTOutput.CreateErrorContent(aResponse.Code,aResponse.CodeText);
    if Not ((IO.Operation in [roOptions,roHEAD]) or aResponse.ContentSent) then
      IO.RestOutput.FinalizeOutput;
    aResponse.ContentStream.Position:=0;
    aResponse.ContentLength:=aResponse.ContentStream.Size;

    if not aResponse.ContentSent then
      aResponse.SendContent;
    if MustLog(rloResultStatus) then
        DoLog(rloResultStatus,IO,'Resource: %s; Operation: %s; Status: %d; Text: %s',[IO.ResourceName,RestMethods[IO.Operation],aResponse.Code,aResponse.CodeText]);

    IO.Free;
  end;
end;

procedure TSQLDBRestDispatcher.VerifyPathInfo(aRequest: TRequest);
Var
  Full,Path : String;
  BasePaths : TStringArray;
  I : Integer;

begin
  // Check & discard basepath parts of the URL
  Path:=aRequest.GetNextPathInfo;
  Full:=BasePath;
  BasePaths:=Full.Split('/',TStringSplitOptions.ExcludeEmpty);
  I:=0;
  While (I<Length(BasePaths)) and SameText(Path,BasePaths[i]) do
    begin
    inc(I);
    Path:=aRequest.GetNextPathInfo;
    end;
  if (I<Length(BasePaths)) then
    Raise ESQLDBRest.Create(400,'NOT FOUND');
  // Path1 is now either connection or resource
  if (rdoConnectionInURL in DispatchOptions) then
    begin
    // Both /metadata and /connection/metadata are possible
    if not ((rdoExposeMetadata in DispatchOptions) and SameText(Path,Strings.getRestString(rpMetadataResourceName))) then
      begin
      aRequest.RouteParams['connection']:=Path;
      Path:=aRequest.GetNextPathInfo;
      end;
    end;
  aRequest.RouteParams['resource']:=Path;
  // Next part is ID
  Path:=aRequest.GetNextPathInfo;
  if (Path<>'') then
    aRequest.RouteParams['ID']:=Path;
end;

function TSQLDBRestDispatcher.ExposeDatabase(const aType, aHostName, aDatabaseName, aUserName, aPassword: String;
  aTables: array of String; aMinFieldOpts: TRestFieldOptions): TSQLDBRestConnection;

Var
  L : TStringList;
  S : String;

begin
  L:=TStringList.Create;
  try
    L.Capacity:=Length(aTables);
    For S in aTables do
      L.Add(S);
    L.Sorted:=True;
    Result:=ExposeDatabase(aType, aHostName, aDatabaseName, aUserName, aPassword,L, aMinFieldOpts);
  finally
    l.Free;
  end;
end;

function TSQLDBRestDispatcher.ExposeDatabase(const aType, aHostName, aDatabaseName, aUserName, aPassword: String; aTables : TStrings = nil; aMinFieldOpts : TRestFieldOptions = []): TSQLDBRestConnection;


begin
  Result:=Connections.AddConnection(aType,aHostName,aDatabaseName,aUserName,aPassword);
  ExposeConnection(Result,aTables,aMinFieldOpts);
end;

function TSQLDBRestDispatcher.ExposeConnection(aOwner: TComponent;
  const aConnection: TSQLDBRestConnection; aTables: TStrings;
  aMinFieldOpts: TRestFieldOptions): TSQLDBRestSchema;

Var
  Conn : TSQLConnection;
  TR : TSQLTransaction;
  S : TSQLDBRestSchema;

begin
  Conn:=GetSQLConnection(aConnection,TR);
  S:=TSQLDBRestSchema.Create(aOwner);
  S.Name:='Schema'+aConnection.Name;
  S.PopulateResources(Conn,aTables,aMinFieldOpts);
  if not (rdoConnectionInURL in DispatchOptions) then
    S.ConnectionName:=aConnection.Name;
  Schemas.AddSchema(S).Enabled:=true;
  Result:=S;
end;

function TSQLDBRestDispatcher.ExposeConnection(
  const aConnection: TSQLDBRestConnection; aTables: TStrings;
  aMinFieldOpts: TRestFieldOptions): TSQLDBRestSchema;
begin
  Result:=ExposeConnection(Self,aConnection,aTables,aMinFieldOpts);
end;

{ TSchemaFreeNotifier }

procedure TSchemaFreeNotifier.Notification(AComponent: TComponent; Operation: TOperation);
begin
  inherited Notification(AComponent, Operation);
  if (Operation=opRemove) and Assigned(FRef) and (Fref.Schema=aComponent) then
    Fref.Schema:=nil;
end;


{ TSQLDBRestSchemaRef }


procedure TSQLDBRestSchemaRef.SetSchema(AValue: TSQLDBRestSchema);
begin
  if (FSchema=AValue) then Exit;
  if Assigned(FSchema) then
    FSchema.RemoveFreeNotification(FNotifier);
  FSchema:=AValue;
  if Assigned(FSchema) then
    FSchema.FreeNotification(FNotifier);
end;

function TSQLDBRestSchemaRef.GetDisplayName: String;
begin
  if Assigned(FSchema) then
    Result:=FSchema.Name
  else
    Result:=inherited GetDisplayName;
end;

constructor TSQLDBRestSchemaRef.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FNotifier:=TSchemaFreeNotifier.Create(Nil);
  TSchemaFreeNotifier(FNotifier).FRef:=Self;
  FEnabled:=True;
end;

destructor TSQLDBRestSchemaRef.Destroy;
begin
  FreeAndNil(FNotifier);
  inherited Destroy;
end;

procedure TSQLDBRestSchemaRef.Assign(Source: TPersistent);

Var
  R : TSQLDBRestSchemaRef;

begin
  if (Source is TSQLDBRestSchemaRef) then
    begin
    R:=Source as TSQLDBRestSchemaRef;
    Schema:=R.Schema;
    Enabled:=R.Enabled;
    end
  else
    inherited Assign(Source);
end;

{ TSQLDBRestConnectionList }

function TSQLDBRestConnectionList.GetConn(aIndex : integer): TSQLDBRestConnection;
begin
  Result:=TSQLDBRestConnection(Items[aIndex]);
end;

procedure TSQLDBRestConnectionList.SetConn(aIndex : integer; AValue: TSQLDBRestConnection);
begin
  Items[aIndex]:=aValue;
end;

function TSQLDBRestConnectionList.IndexOfConnection(const aName: UTF8string
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and not SameText(GetConn(Result).Name,aName) do
    Dec(Result);
end;

function TSQLDBRestConnectionList.FindConnection(const aName: UTF8string): TSQLDBRestConnection;
Var
  Idx : Integer;

begin
  Idx:=IndexOfConnection(aName);
  if Idx=-1 then
    Result:=Nil
  else
    Result:=GetConn(Idx);
end;

function TSQLDBRestConnectionList.AddConnection(const AType, aHostName, aDatabaseName, aUserName, aPassword: UTF8String): TSQLDBRestConnection;

Var
  Idx : Integer;
  N : String;
begin
  Result:=Add as TSQLDBRestConnection;
  IDX:=Result.ID;
  Repeat
    N:='Connection'+IntToStr(IDX);
    Inc(Idx);
  Until IndexOfConnection(N)=-1;
  Result.Name:=N;
  Result.ConnectionType:=aType;
  Result.HostName:=aHostName;
  Result.DatabaseName:=aDatabaseName;
  Result.UserName:=aUserName;
  Result.Password:=aPassword;
end;

procedure TSQLDBRestConnectionList.SaveToFile(const aFileName: UTF8String);

Var
  F : TFileStream;

begin
  F:=TFileStream.Create(aFileName,fmCreate);
  try
    SaveToStream(F);
  finally
    F.Free;
  end;
end;

procedure TSQLDBRestConnectionList.SaveToStream(const aStream: TStream);
Var
  D : TJSONData;
  S : TJSONStringType;

begin
  D:=asJSON(JSONConnectionsRoot);
  try
    S:=D.FormatJSON();
  finally
    D.Free;
  end;
  aStream.WriteBuffer(S[1],Length(S)*SizeOf(TJSONCharType));
end;

function TSQLDBRestConnectionList.AsJSON(const aPropName: UTF8String): TJSONData;
Var
  S : TJSONStreamer;
  A : TJSONArray;

begin
  S:=TJSONStreamer.Create(Nil);
  try
    A:=S.StreamCollection(Self);
  finally
    S.Free;
  end;
  if aPropName='' then
    Result:=A
  else
    Result:=TJSONObject.Create([aPropName,A]);
end;

procedure TSQLDBRestConnectionList.LoadFromFile(const aFileName: UTF8String);
Var
  F : TFileStream;

begin
  F:=TFileStream.Create(aFileName,fmOpenRead or fmShareDenyWrite);
  try
    LoadFromStream(F);
  finally
    F.Free;
  end;
end;

procedure TSQLDBRestConnectionList.LoadFromStream(const aStream: TStream);

Var
  D : TJSONData;

begin
  D:=GetJSON(aStream);
  try
    FromJSON(D,JSONConnectionsRoot);
  finally
    D.Free;
  end;
end;

procedure TSQLDBRestConnectionList.FromJSON(aData: TJSONData; const aPropName: UTF8String);
Var
  A : TJSONArray;
  D : TJSONDestreamer;

begin
  if (aPropName<>'') then
    A:=(aData as TJSONObject).Arrays[aPropName]
  else
    A:=aData as TJSONArray;
  D:=TJSONDestreamer.Create(Nil);
  try
    Clear;
    D.JSONToCollection(A,Self);
  finally
    D.Free;
  end;
end;

{ TSQLDBRestConnection }

procedure TSQLDBRestConnection.SetParams(AValue: TStrings);
begin
  if FParams=AValue then Exit;
  FParams.Assign(AValue);
end;

function TSQLDBRestConnection.GetDisplayName: string;
begin
  Result:=Name;
end;

procedure TSQLDBRestConnection.SetConnection(AValue: TSQLConnection);
begin
  if FConnection=AValue then Exit;
  if Assigned(FConnection) then
    FConnection.RemoveFreeNotification(FNotifier);
  FConnection:=AValue;
  if Assigned(FConnection) then
    FConnection.FreeNotification(FNotifier);
end;

function TSQLDBRestConnection.GetName: UTF8String;
begin
  Result:=FName;
  if (Result='') and Assigned(SingleConnection) then
    Result:=SingleConnection.Name;
  if (Result='') then
    Result:='Connection'+IntToStr(ID);
end;

constructor TSQLDBRestConnection.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FParams:=TStringList.Create;
  FNotifier:=TConnectionFreeNotifier.Create(Nil);
  TConnectionFreeNotifier(FNotifier).FRef:=Self;
  FEnabled:=True;
end;

destructor TSQLDBRestConnection.Destroy;
begin
  TConnectionFreeNotifier(FNotifier).FRef:=Nil;
  FreeAndNil(FNotifier);
  FreeAndNil(FParams);
  inherited Destroy;
end;

procedure TSQLDBRestConnection.Assign(Source: TPersistent);

Var
  C : TSQLDBRestConnection;

begin
  if (Source is TSQLDBRestConnection) then
    begin
    C:=Source as TSQLDBRestConnection;
    Password:=C.Password;
    UserName:=C.UserName;
    CharSet :=C.CharSet;
    HostName:=C.HostName;
    Role:=C.Role;
    DatabaseName:=C.DatabaseName;
    ConnectionType:=C.ConnectionType;
    Port:=C.Port;
    Name:=C.Name;
    SchemaName:=C.SchemaName;
    Params.Assign(C.Params);
    end
  else
    inherited Assign(Source);
end;

procedure TSQLDBRestConnection.ConfigConnection(aConn: TSQLConnection);
begin
  aConn.CharSet:=Self.CharSet;
  aConn.HostName:=Self.HostName;
  aConn.DatabaseName:=Self.DatabaseName;
  aConn.UserName:=Self.UserName;
  aConn.Password:=Self.Password;
  aConn.Params:=Self.Params;
  if aConn is TSQLConnector then
    TSQLConnector(aConn).ConnectorType:=Self.ConnectionType;
end;


Procedure InitSQLDBRest;

begin
  TSQLDBRestDispatcher.SetIOClass(TRestIO);
  TSQLDBRestDispatcher.SetDBHandlerClass(TSQLDBRestDBHandler);
  TSQLDBRestResource.DefaultFieldListClass:=TSQLDBRestFieldList;
  TSQLDBRestResource.DefaultFieldClass:=TSQLDBRestField;
end;

Initialization
  InitSQLDBRest;
end.

