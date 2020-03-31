unit fpwebdata;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, httpdefs, fphttp, db;


type
  { TWebdataInputAdaptor }

  // Translate web request to input for the dataprovider.
  // Descendents must adapt the methods so they fit the particular JS/HTML engine used.

  TWebDataAction = (wdaUnknown,wdaRead,wdaUpdate,wdaInsert,wdaDelete);

  { TCustomWebdataInputAdaptor }
  TTransCodeEvent = Procedure (Sender : TObject; Var S : String);

  TCustomWebdataInputAdaptor = class(TComponent)
  private
    FAction: TWebDataAction;
    FOntransCode: TTransCodeEvent;
    FRequest: TRequest;
    FBatchCount : Integer;
    FRequestPathInfo : String;
    function GetAction: TWebDataAction;
    procedure SetRequest(const AValue: TRequest);
  Protected
    procedure reset; virtual;
    Function GetActionFromRequest : TWebDataAction; virtual;
  Public
    Function GetNextBatch : Boolean; virtual;
    Function TryParamValue(Const AParamName : String; out AValue : String) : Boolean; virtual;
    Function TryFieldValue(Const AFieldName : String; out AValue : String) : Boolean; virtual;
    Function HaveParamValue(Const AParamName : String) : boolean;
    Function HaveFieldValue(Const AFieldName : String) : boolean;
    Function GetParamValue(Const AParamName : String) : String;
    Function GetFieldValue(Const AFieldName : String) : String;
    Property Request : TRequest Read FRequest Write SetRequest;
    Property Action : TWebDataAction Read GetAction Write FAction;
    Property OnTransCode : TTransCodeEvent Read FOntransCode Write FOnTransCode;
  end;
  TCustomWebdataInputAdaptorClass = Class of TCustomWebdataInputAdaptor;

  TWebdataInputAdaptor = Class(TCustomWebdataInputAdaptor)
  Private
    FInputFormat: String;
    FProxy : TCustomWebdataInputAdaptor;
    procedure SetInputFormat(const AValue: String);
  Protected
    Procedure ClearProxy;
    Procedure CheckProxy;
    Function CreateProxy : TCustomWebdataInputAdaptor; virtual;
    Function GetActionFromRequest : TWebDataAction; override;
  Public
    Destructor Destroy; override;
    Function GetNextBatch : Boolean; override;
    Function TryParamValue(Const AParamName : String; out AValue : String) : Boolean; override;
    Function TryFieldValue(Const AFieldName : String; out AValue : String) : Boolean; override;
  Published
    Property InputFormat : String Read FInputFormat Write SetInputFormat;
  end;

  // Manage the data for the content producer
  // return a dataset for data, handles update/delete/insert in a simple TDataset manner.

  { TFPCustomWebDataProvider }
  TWebDataProviderOption = (wdpReadOnly,wdpDisableDelete,wdpDisableEdit,wdpDisableInsert);
  TWebDataProviderOptions = set of TWebDataProviderOption;

  TFPCustomWebDataProvider = Class(TComponent)
  private
    FAdaptor: TCustomWebdataInputAdaptor;
    FIDFieldName: String;
    FOptions: TWebDataProviderOptions;
  Protected
    // Check if adaptor and dataset are available.
    procedure CheckAdaptor;
    // Copy data from input to fields in dataset. Can be overridden
    Procedure CopyFieldData; virtual;
    Procedure DoUpdate; virtual;
    Procedure DoDelete; virtual;
    Procedure DoInsert; virtual;
    // Locate current record. Assumes that
    Procedure LocateCurrent; virtual;
    Procedure DoApplyParams; virtual;
    Function GetDataset : TDataset; virtual; abstract;
  Public
    // Perform an update on the dataset. Correct record is located first.
    Procedure Update;
    // Perform a delete on the dataset. Correct record is located first.
    Procedure Delete;
    // Perform an insert on the dataset.
    Procedure Insert;
    // Apply any parameters passed from request to the dataset. Used only in read operations
    Procedure ApplyParams;
    // get ID Field instance from dataset
    function GetIDField: TField;
    // Get value of ID field as string. After insert, this should contain the newly inserted ID.
    Function IDFieldValue : String; virtual;
    // The dataset
    Property Dataset : TDataset Read GetDataset;
    // Input adaptor
    property Adaptor : TCustomWebdataInputAdaptor Read FAdaptor Write FAdaptor;
    // Fieldname of ID field. If empty, field with pfInKey is searched.
    Property IDFieldName : String Read FIDFieldName Write FIDFieldName;
    // options
    Property Options : TWebDataProviderOptions Read FOptions Write FOptions;
  end;
  TFPCustomWebDataProviderClass = Class of TFPCustomWebDataProvider;

  { TFPWebDataProvider }
  // Simple descendent that has a datasource property, can be dropped on a module.
  TFPWebDataProvider = Class(TFPCustomWebDataProvider)
  private
    FDatasource: TDatasource;
    procedure SetDataSource(const AValue: TDatasource);
  Protected
    Function GetDataset : TDataset; override;
  Public
    procedure Notification(AComponent: TComponent; Operation: TOperation);override;
  Published
    Property DataSource : TDatasource Read FDatasource Write SetDataSource;
  end;

  // Handle request for read/create/update/delete and return a result.

  { TCustomHTTPDataContentProducer }
  // Support for transcoding from/to UTF-8. If outbound is true, the value is going from server to browser.
  TOnTranscodeEvent = Procedure (Sender : TObject; F : TField; Var S : String; Outbound : Boolean) of object;

  TCustomHTTPDataContentProducer = Class(THTTPContentProducer)
  Private
    FAllowPageSize: Boolean;
    FBeforeDelete: TNotifyEvent;
    FBeforeInsert: TNotifyEvent;
    FBeforeUpdate: TNotifyEvent;
    FDataProvider: TFPCustomWebDataProvider;
    FMetadata: Boolean;
    FOnTranscode: TOnTranscodeEvent;
    FPageSize: Integer;
    FPageStart: Integer;
    FSD: Boolean;
    FSortField: String;
    FAdaptor : TCustomWebdataInputAdaptor;
    function GetDataset: TDataset;
    procedure SetAdaptor(const AValue: TCustomWebDataInputAdaptor);
    procedure SetDataProvider(const AValue: TFPCustomWebDataProvider);
  Protected
    Procedure StartBatch(ResponseContent : TStream); virtual;
    Procedure NextBatchItem(ResponseContent : TStream); virtual;
    Procedure EndBatch(ResponseContent : TStream); virtual;
    Function GetDataContentType : String; virtual;
    procedure DatasetToStream(Stream: TStream); virtual;abstract;
    Function CreateAdaptor(ARequest : TRequest) : TCustomWebdataInputAdaptor; virtual;
    Procedure DoGetContent(ARequest : TRequest; Content : TStream; Var Handled : Boolean); override;
    Procedure DoHandleRequest(ARequest : TRequest; AResponse : TResponse; Var Handled : Boolean); override;
    Procedure DoUpdateRecord(ResponseContent : TStream); virtual;
    Procedure DoInsertRecord(ResponseContent : TStream); virtual;
    Procedure DoDeleteRecord(ResponseContent : TStream); virtual;
    Procedure DoReadRecords(ResponseContent : TStream); virtual;
    Procedure DoExceptionToStream(E : Exception; ResponseContent : TStream); virtual; abstract;
    procedure Notification(AComponent: TComponent; Operation: TOperation);override;
    Property Dataset: TDataset Read GetDataSet;
    // Before a record is about to be updated
    Property BeforeUpdate : TNotifyEvent Read FBeforeUpdate Write FBeforeUpdate;
    // Before a record is about to be inserted
    Property BeforeInsert : TNotifyEvent Read FBeforeInsert Write FBeforeInsert;
    // Before a record is about to be deleted
    Property BeforeDelete : TNotifyEvent Read FBeforeDelete Write FBeforeDelete;
  Public
    Constructor Create(AOwner : TComponent); override;
    Property Adaptor : TCustomWebDataInputAdaptor Read FAdaptor Write SetAdaptor;
    Property Provider : TFPCustomWebDataProvider read FDataProvider write SetDataProvider;
    Property DataContentType : String Read GetDataContentType;
  Published
    Property PageStart : Integer Read FPageStart Write FPageStart default 0;
    Property PageSize : Integer Read FPageSize Write FPageSize default 0;
    Property MetaData : Boolean Read FMetadata Write FMetaData Default False;
    Property SortField : String Read FSortField Write FSortField;
    Property SortDescending : Boolean Read FSD Write FSD default False;
    Property AllowPageSize : Boolean Read FAllowPageSize Write FAllowPageSize default True;
    Property OnTransCode : TOnTranscodeEvent Read FOnTranscode Write FOnTranscode;
  end;
  TCustomHTTPDataContentProducerClass = Class of TCustomHTTPDataContentProducer;

  { THTTPDataContentProducer }

  THTTPDataContentProducer = Class(TCustomHTTPDataContentProducer)
  private
    FOnConfigure: TNotifyEvent;
    FProxy : TCustomHTTPDataContentProducer;
    FOutputFormat: String;
    procedure SetOutputFormat(const AValue: String);
  Protected
    Procedure ClearProxy;
    Procedure CheckProxy;
    Function CreateProxy : TCustomHTTPDataContentProducer; virtual;
    procedure ConfigureProxy(AProxy: TCustomHTTPDataContentProducer); virtual;
  Public
    Destructor destroy; override;
  Published
    Property Adaptor;
    Property Provider;
    Property OutputFormat : String Read FOutputFormat Write SetOutputFormat;
    Property OnConfigureFormat : TNotifyEvent Read FOnConfigure Write FOnConfigure;
  end;
  TBeforeCreateWebDataProviderEvent = Procedure (Sender : TObject; Var AClass : TFPCustomWebDataProviderClass) of object;
  TWebDataProviderEvent = Procedure (Sender : TObject; AProvider : TFPCustomWebDataProvider) of object;
  //TWebDataCreateProviderEvent = Procedure (Sender : TObject; Const AProviderName : String; Out AnInstance : TFPCustomWebDataProvider) of object;
  TDataModuleClass = Class of TDataModule;

  { TWebInputAdaptorDef }

  TWebInputAdaptorDef = Class(TCollectionItem)
  private
    FClass: TCustomWebdataInputAdaptorClass;
    FName: String;
    procedure SetName(const AValue: String);
  protected
    Function CreateInstance(AOwner : TComponent) :TCustomWebdataInputAdaptor; virtual;
  Public
    Property AdaptorClass : TCustomWebdataInputAdaptorClass Read FClass Write FClass;
    Property Name : String Read FName Write SetName;
  end;

  { TWebInputAdaptorDefs }

  TWebInputAdaptorDefs = Class(TCollection)
  private
    function GetD(Index : Integer): TWebInputAdaptorDef;
    procedure SetD(Index : Integer; const AValue: TWebInputAdaptorDef);
  Public
    Function IndexOfAdaptor(Const AAdaptorName : String) : Integer;
    Function AddAdaptor(Const AAdaptorName : String; AClass : TCustomWebdataInputAdaptorClass) : TWebInputAdaptorDef;
    Property AdaptorDefs[Index : Integer] : TWebInputAdaptorDef Read GetD Write SetD; default;
  end;

  { THttpDataProducerDef }

  THttpDataProducerDef = Class(TCollectionItem)
  private
    FClass: TCustomHTTPDataContentProducerClass;
    FName: String;
    procedure SetName(const AValue: String);
  protected
    Function CreateInstance(AOwner : TComponent) :TCustomHTTPDataContentProducer; virtual;
  Public
    Property ProducerClass : TCustomHTTPDataContentProducerClass Read FClass Write FClass;
    Property Name : String Read FName Write SetName;
  end;

  { THttpDataProducerDefs }

  THttpDataProducerDefs = Class(TCollection)
  private
    function GetD(Index : Integer): THttpDataProducerDef;
    procedure SetD(Index : Integer; const AValue: THttpDataProducerDef);
  Public
    Function IndexOfProducer(Const AProducerName : String) : Integer;
    Function AddProducer(Const AProducerName : String; AClass : TCustomHTTPDataContentProducerClass) : THttpDataProducerDef;
    Property ProducerDefs[Index : Integer] : THttpDataProducerDef Read GetD Write SetD; default;
  end;


   { TWebDataProviderDef }


  TWebDataProviderDef = Class(TCollectionItem)
  private
    FAfterCreate: TWebDataProviderEvent;
    FBeforeCreate: TBeforeCreateWebDataProviderEvent;
    FPClass: TFPCustomWebDataProviderClass;
    FDataModuleClass : TDataModuleClass;
    FProviderName: String;
    procedure SetFPClass(const AValue: TFPCustomWebDataProviderClass);
    procedure SetProviderName(const AValue: String);
  protected
    Function CreateInstance(AOwner : TComponent; Out AContainer : TComponent) : TFPCUstomWebDataProvider; virtual;
    Property DataModuleClass : TDataModuleClass Read FDataModuleClass;
  Public
    Property ProviderName : String Read FProviderName Write SetProviderName;
    Property ProviderClass : TFPCustomWebDataProviderClass Read FPClass Write SetFPClass;
    Property BeforeCreate : TBeforeCreateWebDataProviderEvent Read FBeforeCreate Write FBeforeCreate;
    Property AfterCreate : TWebDataProviderEvent Read FAfterCreate Write FAfterCreate;
  end;

  { TWebDataProviderDefs }

  TWebDataProviderDefs = Class(TCollection)
  private
    function GetD(Index : Integer): TWebDataProviderDef;
    procedure SetD(Index : Integer; const AValue: TWebDataProviderDef);
  Public
    Function IndexOfProvider(Const AProviderName : String) : Integer;
    Function AddProvider(Const AProviderName : String) : TWebDataProviderDef; overload;
    Function AddProvider(Const AProviderName : String; AClass :TFPCustomWebDataProviderClass) : TWebDataProviderDef; overload;
    Property WebDataProviderDefs[Index : Integer] : TWebDataProviderDef Read GetD Write SetD; default;
  end;

  { TFPCustomWebDataProviderManager }

  TFPCustomWebDataProviderManager = Class(TComponent)
  Private
    FRegistering: Boolean;
  Protected
    procedure Initialize; virtual;
    // Provider support
    Procedure RemoveProviderDef(Const Index : Integer); virtual; abstract;
    function AddProviderDef(Const AProviderName : String) : TWebDataProviderDef; virtual; abstract;
    function IndexOfProviderDef(Const AProviderName : String) : Integer; virtual; abstract;
    function GetProviderDef(Index : Integer): TWebDataProviderDef; virtual; abstract;
    function GetProviderDefCount: Integer; virtual; abstract;
    // Inputadaptor support
    function AddInputAdaptorDef(Const AAdaptorName : String; AClass : TCustomWebdataInputAdaptorClass) : TWebInputAdaptorDef; virtual; abstract;
    function IndexOfInputAdaptorDef(Const AAdaptorName : String) : Integer; virtual; abstract;
    Procedure RemoveInputAdaptorDef(Index : Integer); virtual; abstract;
    function GetInputAdaptorDef(Index : Integer): TWebInputAdaptorDef; virtual; abstract;
    function GetInputAdaptorDefCount: Integer; virtual; abstract;
    // Outputproducer support
    function AddHttpDataProducerDef(Const AProducerName : String; AClass : TCustomHTTPDataContentProducerClass) : THttpDataProducerDef; virtual; abstract;
    function IndexOfHttpDataProducerDef(Const AProducerName : String) : Integer; virtual; abstract;
    Procedure RemoveHttpDataProducerDef(Index : Integer); virtual; abstract;
    function GetHttpDataProducerDef(Index : Integer): THttpDataProducerDef; virtual; abstract;
    function GetHttpDataProducerDefCount: Integer; virtual; abstract;
  Public
    // Input Provider support
    Procedure Unregisterprovider(Const AProviderName : String);
    Procedure RegisterDatamodule(Const AClass : TDatamoduleClass);
    Function RegisterProvider(Const AProviderName : String; AClass : TFPCustomWebDataProviderClass) : TWebDataProviderDef; overload;
    Function FindProviderDefByName(Const AProviderName : String) : TWebDataProviderDef;
    Function GetProviderDefByName(Const AProviderName : String) : TWebDataProviderDef;
    Function GetProvider(Const ADef : TWebDataProviderDef; AOwner : TComponent; Out AContainer : TComponent): TFPCustomWebDataProvider;
    Function GetProvider(Const AProviderName : String; AOwner : TComponent; Out AContainer : TComponent): TFPCustomWebDataProvider;
    // Input Adaptor support
    Function RegisterInputAdaptor(Const AAdaptorName : String; AClass : TCustomWebdataInputAdaptorClass) : TWebInputAdaptorDef;
    Procedure UnRegisterInputAdaptor(Const AAdaptorName : String);
    Function FindInputAdaptorDefByName(Const AAdaptorName : String) : TWebInputAdaptorDef;
    Function GetInputAdaptorDefByName(Const AAdaptorName : String) : TWebInputAdaptorDef;
    Function GetInputAdaptor(Const ADef : TWebInputAdaptorDef; AOwner : TComponent = Nil): TCustomWebdataInputAdaptor; overload;
    Function GetInputAdaptor(Const AAdaptorName : String; AOwner : TComponent = Nil): TCustomWebdataInputAdaptor; overload;
    // Outputproducer support
    function RegisterDataProducer(Const AProducerName : String; AClass : TCustomHTTPDataContentProducerClass) : THttpDataProducerDef;
    Procedure UnRegisterDataProducer(Const AProducerName : String);
    function FindDataProducerDefByName(Const AProducerName : String) : THttpDataProducerDef;
    function GetDataProducerDefByName(Const AProducerName : String) : THttpDataProducerDef;
    function GetDataProducer(ADef : THttpDataProducerDef; AOwner : TComponent) : TCustomHTTPDataContentProducer;
    function GetDataProducer(Const AProducerName: String; AOwner : TComponent) : TCustomHTTPDataContentProducer;
    // properties
    Property Registering : Boolean Read FRegistering;
    Property ProviderCount : Integer Read GetProviderDefCount;
    Property ProviderDefs[Index : Integer] : TWebDataProviderDef Read GetProviderDef;
    Property InputAdaptorDefs[Index : Integer] : TWebInputAdaptorDef Read GetInputAdaptorDef;
    Property InputAdaptorDefCount : Integer Read GetInputAdaptorDefCount;
    Property DataProducerDefs[Index : Integer] : THttpDataProducerDef Read GetHttpDataProducerDef;
    Property DataProducerDefCount : Integer Read GetHttpDataProducerDefCount;
  end;
  TFPCustomWebDataProviderManagerClass = Class of TFPCustomWebDataProviderManager;

  { TFPWebDataProviderManager }

  TFPWebDataProviderManager = Class(TFPCustomWebDataProviderManager)
  Private
    FProviderDefs : TWebDataProviderDefs;
    FAdaptorDefs : TWebInputAdaptorDefs;
    FProducerDefs : THttpDataProducerDefs;
  Protected
    Procedure RemoveProviderDef(Const Index : Integer); override;
    function AddProviderDef(Const AProviderName : String) : TWebDataProviderDef; override;
    function IndexOfProviderDef(Const AProviderName : String) : Integer; override;
    function GetProviderDef(Index : Integer): TWebDataProviderDef; override;
    function GetProviderDefCount: Integer; override;
    // Inputadaptor support
    function AddInputAdaptorDef(Const AAdaptorName : String; AClass : TCustomWebdataInputAdaptorClass) : TWebInputAdaptorDef; Override;
    function IndexOfInputAdaptorDef(Const AAdaptorName : String) : Integer; Override;
    procedure RemoveInputAdaptorDef(Index : Integer); Override;
    function GetInputAdaptorDef(Index : Integer): TWebInputAdaptorDef; Override;
    function GetInputAdaptorDefCount: Integer; Override;
    // Outputproducer support
    function AddHttpDataProducerDef(Const AProducerName : String; AClass : TCustomHTTPDataContentProducerClass) : THttpDataProducerDef; Override;
    function IndexOfHttpDataProducerDef(Const AProducerName : String) : Integer; Override;
    Procedure RemoveHttpDataProducerDef(Index : Integer); Override;
    function GetHttpDataProducerDef(Index : Integer): THttpDataProducerDef; Override;
    function GetHttpDataProducerDefCount: Integer; Override;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
  end;

  THandleWebDataEvent = Procedure (Sender : TObject;AProvider : TFPCustomWebDataProvider; Var Handled : Boolean) of object;
  TWebDataEvent = Procedure (Sender : TObject; AProvider : TFPCustomWebDataProvider) of object;
  TContentProducerEvent = Procedure (Sender : TObject; Var AContentProducer: TCustomHTTPDataContentProducer) of object;
  TInputAdaptorEvent = Procedure (Sender : TObject; Var AInputAdaptor : TCustomWebdataInputAdaptor) of object;
  TContentEvent = Procedure (Sender : TObject; Content : TStream) of Object;
  TGetWebDataProviderEvent = Procedure (Sender : TObject; Const AProviderName : String; Var AnInstance : TFPCustomWebDataProvider) of object;

  { TFPCustomWebDataModule }

  { TFPCustomWebProviderDataModule }

  TFPCustomWebProviderDataModule = Class(TSessionHTTPModule)
  Private
    FAfterDelete: TWebDataEvent;
    FAfterInsert: TWebDataEvent;
    FAfterRead: TWebDataEvent;
    FAfterUpdate: TWebDataEvent;
    FBeforeDelete: THandleWebDataEvent;
    FBeforeInsert: THandleWebDataEvent;
    FBeforeRead: THandleWebDataEvent;
    FBeforeUpdate: THandleWebDataEvent;
    FContentProducer: TCustomHTTPDataContentProducer;
    FInputAdaptor: TCustomWebdataInputAdaptor;
    FOnContent: TContentEvent;
    FOnGetContentProducer: TContentProducerEvent;
    FOnGetInputAdaptor: TInputAdaptorEvent;
    FOnGetProvider: TGetWebDataProviderEvent;
    FRequest: TRequest;
    FResponse: TResponse;
    FUseProviderManager: Boolean;
    function GetAdaptor: TCustomWebDataInputAdaptor;
    function GetContentProducer: TCustomHTTPDataContentProducer;
    Procedure ReadWebData(AProvider : TFPCustomWebDataProvider);
    Procedure InsertWebData(AProvider : TFPCustomWebDataProvider);
    procedure SetContentProducer(const AValue: TCustomHTTPDataContentProducer);
    procedure SetInputAdaptor(const AValue: TCustomWebdataInputAdaptor);
    Procedure UpdateWebData(AProvider : TFPCustomWebDataProvider);
    Procedure DeleteWebData(AProvider : TFPCustomWebDataProvider);
  Protected
    function GetProvider(const AProviderName: String; Out AContainer : TComponent): TFPCustomWebDataProvider; virtual;
    procedure ProduceContent(AProvider : TFPCustomWebDataProvider); virtual;
    Procedure DoReadWebData(AProvider : TFPCustomWebDataProvider); virtual;
    Procedure DoInsertWebData(AProvider : TFPCustomWebDataProvider); virtual;
    Procedure DoUpdateWebData(AProvider : TFPCustomWebDataProvider); virtual;
    Procedure DoDeleteWebData(AProvider : TFPCustomWebDataProvider); virtual;
    // Input adaptor to use when processing request. Can be nil, and provided in OnGetInputAdaptor
    Property InputAdaptor : TCustomWebdataInputAdaptor Read FInputAdaptor Write SetInputAdaptor;
    // Content producer to produce response content
    Property ContentProducer : TCustomHTTPDataContentProducer Read FContentProducer Write SetContentProducer;
    // Triggered before a read request is started
    Property BeforeRead   : THandleWebDataEvent Read FBeforeRead Write FBeforeRead;
    // Triggered after a read request completed
    Property AfterRead    : TWebDataEvent Read FAfterRead Write FAfterRead;
    // Triggered before an insert request is started
    Property BeforeInsert : THandleWebDataEvent Read FBeforeInsert Write FBeforeInsert;
    // Triggered after an insert request completed
    Property AfterInsert  : TWebDataEvent Read FAfterInsert Write FAfterInsert;
    // Triggered before an update request is started
    Property BeforeUpdate : THandleWebDataEvent Read FBeforeUpdate Write FBeforeUpdate;
    // Triggered after an update request completed
    Property AfterUpdate  : TWebDataEvent Read FAfterUpdate Write FAfterUpdate;
    // Triggered before a delete request is started
    Property BeforeDelete : THandleWebDataEvent Read FBeforeDelete Write FBeforeDelete;
    // Triggered after an insert request completed
    Property AfterDelete  : TWebDataEvent Read FAfterDelete Write FAfterDelete;
    // Triggered when the input adaptor needs to be determined.
    Property OnGetInputAdaptor : TInputAdaptorEvent Read FOnGetInputAdaptor Write FOnGetInputAdaptor;
    // Triggered when the WebDataProvider needs to be determined.
    Property OnGetProvider : TGetWebDataProviderEvent Read FOnGetProvider Write FOnGetprovider;
    // Triggered when the contentproducer needs to be determined
    Property OnGetContentProducer : TContentProducerEvent Read FOnGetContentProducer Write  FOnGetContentProducer;
    // Triggered when the content has been created.
    Property OnContent : TContentEvent Read FOnContent Write FOnContent;
    // Set to False if the ProviderManager should not be searched for a provider
    Property UseProviderManager : Boolean Read FUseProviderManager Write FUseProviderManager default True;
  Public
    Constructor CreateNew(AOwner : TComponent; CreateMode : Integer); override;
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); override;
    // Access to request
    Property Request: TRequest Read FRequest;
    // Access to response
    Property Response: TResponse Read FResponse;
  end;

  TFPWebProviderDataModule = Class(TFPCustomWebProviderDataModule)
  Published
    Property CreateSession;
    Property InputAdaptor;
    Property ContentProducer;
    Property UseProviderManager;
    Property OnGetContentProducer;
    Property BeforeRead;
    Property AfterRead;
    Property BeforeInsert;
    Property AfterInsert;
    Property BeforeUpdate;
    Property AfterUpdate;
    Property BeforeDelete;
    Property AfterDelete;
    Property OnGetInputAdaptor;
    Property OnGetProvider;
    Property OnContent;
    Property OnNewSession;
    Property OnSessionExpired;
    property CORS;
  end;

Var
  WebDataProviderManagerClass : TFPCustomWebDataProviderManagerClass = TFPWebDataProviderManager;

Function WebDataProviderManager : TFPCustomWebDataProviderManager;

implementation

{ $define wmdebug}

{$ifdef wmdebug}
uses dbugintf;
{$endif}

Resourcestring
  SErrNoIDField = 'No key field found';
  SErrNoAdaptor = 'No adaptor assigned';
  SErrNoDataset = 'No dataset assigned';
  SErrNoIDValue = 'No key value specified';
  SErrCouldNotLocateRecord = 'Could not locate record with value "%s" for key field "%s"';
  SErrNoDatasource = 'No datasource property available';
  SErrNoAction     = 'Cannot determine action from request';
  SErrDuplicateWebDataProvider = 'Duplicate webdata provider';
  SErrUnknownWebDataProvider = 'Unknown webdata provider: "%s"';
  SErrContentProviderRequest = 'Content provider "%s" does not handle request.';
  SErrUnknownProviderAction = 'Cannot determine action for provider "%s".';
  SErrDuplicateAdaptor = 'Duplicate input adaptor name: "%s"';
  SErrDuplicateHTTPDataProducer = 'Duplicate web data output content producer name: "%s"';
  SErrUnknownInputAdaptor = 'Unknown web data input adaptor name: "%s"';
  SErrUnknownHTTPDataProducer = 'Unknown web data output content producer name: "%s"';
  SErrActionNotAllowed = 'Options of provider %s do not allow %s.';
  SEditing   = 'editing';
  SDeleting  = 'deleting';
  SInserting = 'inserting';


{ TCustomWebdataInputAdaptor }

{ TFPCustomWebDataProvider }


procedure TCustomWebdataInputAdaptor.SetRequest(const AValue: TRequest);
begin
  If FRequest=AValue then Exit;
  FRequest:=AValue;
  Reset;
end;

procedure TCustomWebdataInputAdaptor.reset;
begin
{$ifdef wmdebug}SendDebugFmt('TCustomWebdataInputAdaptor.Reset (%s)',[FRequestPathInfo]);{$endif}
  FBatchCount:=0;
  Faction:=wdaUnknown;
  FRequestPathInfo:='';
end;

function TCustomWebdataInputAdaptor.GetActionFromRequest: TWebDataAction;

Var
  N : String;

begin
  Result:=wdaUnknown;
  If (Request<>Nil) then
    begin
    if (FRequestPathInfo='') then
      FRequestPathInfo:=Request.GetNextPathInfo;
    N:=lowercase(FRequestPathInfo);
{$ifdef wmdebug}SendDebugFmt('TCustomWebdataInputAdaptor.GetActionFromRequest : %s (%s)',[n,Request.Pathinfo]);{$endif}
    If (N='read') then
      Result:=wdaRead
    else If (N='insert') then
      Result:=wdaInsert
    else If (N='delete') then
      Result:=wdaDelete
    else If (N='update') then
      Result:=wdaUpdate;
    end;
end;

function TCustomWebdataInputAdaptor.GetAction: TWebDataAction;

begin
  If (Faction=wdaUnknown) then
    FAction:=GetActionFromRequest;
  Result:=FAction;
  If (Result=wdaUnknown) then
    Raise EFPHTTPError.Create(SErrNoAction)
end;

function TCustomWebdataInputAdaptor.GetNextBatch: Boolean;
begin
  Result:=(FBatchCount=0);
  Inc(FBatchCount);
end;

function TCustomWebdataInputAdaptor.TryParamValue(const AParamName: String;
  out AValue: String): Boolean;

Var
  L : TStrings;
  I : Integer;
  N : String;
begin
  Result:=False;
  If (Request.Method='GET') then
    L:=Request.QueryFields
  else // (Request.Method='POST') then
    L:=FRequest.ContentFields;
  I:=L.IndexOfName(AParamName);
  Result:=(I<>-1);
  If Result then
    L.GetNameValue(I,N,AValue);
  If (AValue<>'') and Assigned(FOnTranscode) then
    FOnTransCode(Self,Avalue);
end;

function TCustomWebdataInputAdaptor.TryFieldValue(const AFieldName: String;
  out AValue: String): Boolean;
begin
  Result:=TryParamValue(AFieldName,AValue);
end;


function TCustomWebdataInputAdaptor.HaveParamValue(const AParamName: String
  ): boolean;

Var
  S: String;

begin
  Result:=TryParamValue(AParamName,S);
end;

function TCustomWebdataInputAdaptor.HaveFieldValue(const AFieldName: String
  ): Boolean;
Var
  S: String;

begin
  Result:=TryFieldValue(AFieldName,S);
end;

function TCustomWebdataInputAdaptor.GetParamValue(const AParamName: String): String;
begin
  If not TryParamValue(AParamName,Result) then
    Result:='';
end;

function TCustomWebdataInputAdaptor.GetFieldValue(const AFieldName: String): String;
begin
  If not TryFieldValue(AFieldName,Result) then
    Result:='';
end;
{ TFPCustomWebDataProvider }

procedure TFPCustomWebDataProvider.CopyFieldData;

Var
  I : Integer;
  F : TField;
  S : String;
  DS : TDataset;

begin
  DS:=Dataset;
  For I:=0 to DS.Fields.Count-1 do
    begin
    F:=DS.Fields[i];
    If (F.DataType<>ftAutoInc) or (DS.State=dsInsert) then
      If ADaptor.TryFieldValue(F.FieldName,S) then
        begin
        If (S<>'') then
          F.AsString:=S
        else if DS.State=dsEdit then
          F.Clear;
        end;
    end;
end;

procedure TFPCustomWebDataProvider.DoUpdate;

Var
  DS : TDataset;

begin
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.DoUpdate: Updating record');{$endif}
  DS:=Dataset;
  LocateCurrent;
  DS.Edit;
  CopyFieldData;
  DS.Post;
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.DoUpdate: Done Updating record');{$endif}
end;

procedure TFPCustomWebDataProvider.DoDelete;

Var
  DS : TDataset;
begin
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.DoDelete: Deleting record');{$endif}
  LocateCurrent;
  DS:=Dataset;
  DS.Delete;
end;

procedure TFPCustomWebDataProvider.DoInsert;
Var
  DS : TDataset;
begin
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.DoInsert: Inserting record');{$endif}
  DS:=Dataset;
  DS.Append;
  CopyFieldData;
  DS.Post;
end;

Function TFPCustomWebDataProvider.GetIDField : TField;

Var
  FN : String;
  I : Integer;

begin
  Result:=Nil;
  FN:=IDFieldName;
  If (FN='') then
    begin
    I:=0;
    While (Result=Nil) and (I<Dataset.Fields.Count) do
      begin
      If pfInKey in Dataset.Fields[i].ProviderFLags then
        Result:=Dataset.Fields[i];
      inc(I);
      end;
    end
  else
    Result:=Dataset.FieldByname(FN);
  if (Result=Nil) then
    Raise EFPHTTPError.Create(SErrNoIDField);
end;

procedure TFPCustomWebDataProvider.LocateCurrent;

Var
  V : String;
  F : TField;

begin
  CheckAdaptor;
  F:=GetIDField;
  V:=Adaptor.GetFieldValue(F.FieldName);
  If (V='') then
    Raise EFPHTTPError.Create(SErrNoIDValue);
  if Not Dataset.Locate(F.FieldName,V,[]) then
    begin
    // Search the hard way
    Dataset.First;
    While (not Dataset.EOF) and (F.AsString<>V)  do
      Dataset.Next;
    If Dataset.EOF and (F.AsString<>V) then
      Raise EFPHTTPError.CreateFmt(SErrCouldNotLocateRecord,[V,F.FieldName]);
    end;
end;

procedure TFPCustomWebDataProvider.DoApplyParams;
begin
  // Do nothing
end;


procedure TFPCustomWebDataProvider.CheckAdaptor;

begin
  if Not Assigned(Adaptor) then
    Raise EFPHTTPError.Create(SErrNoAdaptor);
  if Not Assigned(Dataset) then
    Raise EFPHTTPError.Create(SerrNoDataset);
end;

procedure TFPCustomWebDataProvider.Update;
begin
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.Update enter');{$endif}
  If ((Options * [wdpReadOnly,wdpDisableEdit])<>[]) then
    Raise EFPHTTPError.CreateFmt(SErrActionNotAllowed,[Name,SEditing]);
  CheckAdaptor;
  DoUpdate;
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.Update leave');{$endif}
end;

procedure TFPCustomWebDataProvider.Delete;
begin
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.Delete enter');{$endif}
  If ((Options * [wdpReadOnly,wdpDisableDelete])<>[]) then
    Raise EFPHTTPError.CreateFmt(SErrActionNotAllowed,[Name,SDeleting]);
  CheckAdaptor;
  DoDelete;
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.Delete leave');{$endif}
end;

procedure TFPCustomWebDataProvider.Insert;
begin
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.Insert enter');{$endif}
  If ((Options * [wdpReadOnly,wdpDisableInsert])<>[]) then
    Raise EFPHTTPError.CreateFmt(SErrActionNotAllowed,[Name,SInserting]);
  CheckAdaptor;
  DoInsert;
  {$ifdef wmdebug}SendDebug('TFPCustomWebDataProvider.Insert leave');{$endif}
end;

procedure TFPCustomWebDataProvider.ApplyParams;
begin
  CheckAdaptor;
  DoApplyParams;
end;

function TFPCustomWebDataProvider.IDFieldValue: String;
begin
  Result:=GetIDField.DisplayText;
end;


{ TFPWebDataProvider }

procedure TFPWebDataProvider.SetDataSource(const AValue: TDatasource);
begin
  if FDataSource=AValue then exit;
  If Assigned(FDatasource) then
    FDataSource.RemoveFreeNotification(Self);
  FDataSource:=AValue;
  If Assigned(FDatasource) then
    FDataSource.FreeNotification(Self);
end;

function TFPWebDataProvider.GetDataset: TDataset;
begin
  If Assigned(DataSource) then
    Result:=Datasource.Dataset
  else
    Raise EFPHTTPError.Create(SErrNoDatasource)
end;

procedure TFPWebDataProvider.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  If (Operation=opRemove) and (AComponent=FDatasource) then
    FDatasource:=Nil;
  inherited Notification(AComponent, Operation);
end;

{ TCustomHTTPDataContentProducer }

function TCustomHTTPDataContentProducer.GetDataset: TDataset;
begin
  If Assigned(FDataProvider) then
    Result:=FDataProvider.Dataset;
end;

procedure TCustomHTTPDataContentProducer.SetAdaptor(
  const AValue: TCustomWebDataInputAdaptor);
begin
  If FAdaptor=AValue then
     exit;
  If Assigned(FAdaptor) then
    FAdaptor.RemoveFreeNotification(Self);
  FAdaptor:=AValue;
  If Assigned(FAdaptor) then
    FAdaptor.FreeNotification(Self);
end;

procedure TCustomHTTPDataContentProducer.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  If (Operation=opRemove) then
    if (AComponent=FDataProvider) then
       FDataProvider:=Nil
    else if (AComponent=FAdaptor) then
       FAdaptor:=Nil;
  inherited Notification(AComponent, Operation);
end;

procedure TCustomHTTPDataContentProducer.SetDataProvider(
  const AValue: TFPCustomWebDataProvider);
begin
  if FDataProvider=AValue then exit;
  If Assigned(FDataProvider) then
    FDataProvider.RemoveFreeNotification(Self);
  FDataProvider:=AValue;
  If Assigned(FDataProvider) then
    FDataProvider.FreeNotification(Self);
end;

procedure TCustomHTTPDataContentProducer.StartBatch(ResponseContent: TStream);
begin
  // Do nothing
end;

procedure TCustomHTTPDataContentProducer.NextBatchItem(ResponseContent: TStream
  );
begin
  // do nothing
end;

procedure TCustomHTTPDataContentProducer.EndBatch(ResponseContent: TStream);
begin
  // do nothing
end;

function TCustomHTTPDataContentProducer.GetDataContentType: String;
begin
  Result:='';
end;

function TCustomHTTPDataContentProducer.CreateAdaptor(ARequest : TRequest): TCustomWebdataInputAdaptor;
begin
  Result:=TCustomWebdataInputAdaptor.Create(Self);
  Result.Request:=ARequest
end;


procedure TCustomHTTPDataContentProducer.DoGetContent(ARequest : TRequest; Content : TStream; Var Handled : Boolean);

Var
  B : Boolean;
  A : TCustomWebdataInputAdaptor;

begin
  {$ifdef wmdebug}SendDebugFmt('Request content %s',[ARequest.Content]);{$endif}
  B:=(Adaptor=Nil);
  if B then
    begin
    A:=CreateAdaptor(ARequest);
    Adaptor:=A;
    end;
  try
    try
      Case Adaptor.Action of
        wdaRead : DoReadRecords(Content);
        wdaInsert,
        wdaUpdate,
        wdaDelete :
          begin
          {$ifdef wmdebug}SendDebug('Starting batch Loop');{$endif}
          StartBatch(Content);
          While Adaptor.GetNextBatch do
            begin
            {$ifdef wmdebug}SendDebug('Next batch item');{$endif}
            NextBatchItem(Content);
            Case Adaptor.Action of
              wdaInsert  : DoInsertRecord(Content);
              wdaUpdate  : DoUpdateRecord(Content);
              wdaDelete  : DoDeleteRecord(Content);
            else
              inherited DoGetContent(ARequest, Content,Handled);
            end;
          end;
         EndBatch(Content);
        {$ifdef wmdebug}SendDebug('Ended batch Loop');{$endif}
         end;
      else
        Raise EFPHTTPError.Create(SErrNoAction);
      end;
      Handled:=true;
    except
      On E : Exception do
        begin
        DoExceptionToStream(E,Content);
        Handled:=True;
        end;
    end;
  finally
    If B then
      FreeAndNil(A);
  end;
end;

procedure TCustomHTTPDataContentProducer.DoHandleRequest(ARequest: TRequest;
  AResponse: TResponse; var Handled: Boolean);

Var
  S : String;

begin
  inherited DoHandleRequest(ARequest, AResponse, Handled);
  If Handled then
    begin
    S:=GetDataContentType;
    If (S<>'') then
      AResponse.ContentType:=S;
    end;
end;

procedure TCustomHTTPDataContentProducer.DoUpdateRecord(ResponseContent: TStream);
begin
  {$ifdef wmdebug}SendDebug('DoUpdateRecord: Updating record');{$endif}
  If Assigned(FBeforeUpdate) then
    FBeforeUpdate(Self);
  Provider.Update;
  {$ifdef wmdebug}SendDebug('DoUpdateRecord: Updated record');{$endif}
end;

procedure TCustomHTTPDataContentProducer.DoInsertRecord(ResponseContent: TStream);
begin
  If Assigned(FBeforeInsert) then
    FBeforeInsert(Self);
  Provider.Insert;
end;

procedure TCustomHTTPDataContentProducer.DoDeleteRecord(ResponseContent: TStream);
begin
  If Assigned(FBeforeDelete) then
    FBeforeDelete(Self);
  Provider.Delete;
end;

procedure TCustomHTTPDataContentProducer.DoReadRecords(ResponseContent: TStream);

Var
  DS : TDataset;

begin
  DS:=Provider.Dataset;
  If Not DS.Active then
    begin
    {$ifdef wmdebug}SendDebug('Doreadrecords: Applying parameters');{$endif}
    Provider.ApplyParams;
    {$ifdef wmdebug}SendDebug('Doreadrecords: Applied parameters');{$endif}
    DS.Open;
    {$ifdef wmdebug}SendDebug('Doreadrecords: opened dataset');{$endif}
    end;
  DatasetToStream(ResponseContent);
end;

constructor TCustomHTTPDataContentProducer.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FAllowPagesize:=True;
end;

{ TWebDataProviderDef }

procedure TWebDataProviderDef.SetFPClass(
  const AValue: TFPCustomWebDataProviderClass);
begin
  if FPClass=AValue then exit;
  FPClass:=AValue;
end;

procedure TWebDataProviderDef.SetProviderName(const AValue: String);
begin
  if FProviderName=AValue then exit;
  FProviderName:=AValue;
end;

Function TWebDataProviderDef.CreateInstance(AOwner: TComponent; Out AContainer : TComponent) : TFPCUstomWebDataProvider;

Var
  AClass : TFPCustomWebDataProviderClass;
  DM : TDataModule;
  C : TComponent;

begin
  Result:=Nil;
  {$ifdef wmdebug}SendDebug(Format('Creating instance for %s',[Self.ProviderName]));{$endif}
  If Assigned(FDataModuleClass) then
    begin
    {$ifdef wmdebug}SendDebug(Format('Creating datamodule from class %d ',[Ord(Assigned(FDataModuleClass))]));{$endif}
    DM:=FDataModuleClass.Create(AOwner);
    {$ifdef wmdebug}SendDebug(Format('Created datamodule from class %s ',[DM.ClassName]));{$endif}
    C:=DM.FindComponent(FProviderName);
    If (C<>Nil) and (C is TFPCUstomWebDataProvider) then
      Result:=TFPCUstomWebDataProvider(C)
    else
      begin
      FreeAndNil(DM);
      Raise EFPHTTPError.CreateFmt(SErrUnknownWebDataProvider,[FProviderName]);
      end;
    end
  else
    DM:=TDataModule.CreateNew(AOwner,0);
  AContainer:=DM;
  If (Result=Nil) then
    begin
    {$ifdef wmdebug}SendDebug(Format('Creating from class pointer %d ',[Ord(Assigned(FPClass))]));{$endif}
    AClass:=FPCLass;
    If Assigned(FBeforeCreate) then
      FBeforeCreate(Self,AClass);
    Result:=AClass.Create(AContainer);
    end;
  If Assigned(FAfterCreate) then
    FAfterCreate(Self,Result);
end;

{ TWebDataProviderDefs }

function TWebDataProviderDefs.GetD(Index : Integer): TWebDataProviderDef;
begin
  Result:=TWebDataProviderDef(Items[Index])
end;

procedure TWebDataProviderDefs.SetD(Index : Integer;
  const AValue: TWebDataProviderDef);
begin
  Items[Index]:=AValue;
end;

function TWebDataProviderDefs.IndexOfProvider(const AProviderName: String
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetD(Result).ProviderName,AProviderName)<>0) do
    Dec(Result);
end;

function TWebDataProviderDefs.AddProvider(const AProviderName: String
  ): TWebDataProviderDef;
begin
  If IndexOfProvider(AProviderName)=-1 then
    begin
    Result:=Add as TWebDataProviderDef;
    Result.ProviderName:=AProviderName;
    end
  else
    Raise EFPHTTPError.CreateFmt(SErrDuplicateWebDataProvider,[AProviderName]);
end;

function TWebDataProviderDefs.AddProvider(const AProviderName: String;
  AClass: TFPCustomWebDataProviderClass): TWebDataProviderDef;
begin
  Result:=AddProvider(AProviderName);
  Result.ProviderClass:=AClass;
end;


Var
  AWebDataProviderManager : TFPCustomWebDataProviderManager;

Function WebDataProviderManager : TFPCustomWebDataProviderManager;

begin
  If (AWebDataProviderManager=Nil) then
    begin
    If WebDataProviderManagerClass=Nil then
       WebDataProviderManagerClass:=TFPWebDataProviderManager;
    AWebDataProviderManager:=WebDataProviderManagerClass.Create(Nil);
    AWebDataProviderManager.Initialize;
    end;
  Result:=AWebDataProviderManager;
end;

{ TFPCustomWebDataProviderManager }

procedure TFPCustomWebDataProviderManager.Initialize;
begin
  // Do nothing
end;

procedure TFPCustomWebDataProviderManager.Unregisterprovider(
  const AProviderName: String);

Var
  I : Integer;

begin
  I:=IndexOfProviderDef(AProviderName);
  If (I<>-1) then
    RemoveProviderDef(I)
  else
    Raise EFPHTTPError.CreateFmt(SErrUnknownWebDataProvider,[AProviderName]);
end;

procedure TFPCustomWebDataProviderManager.RegisterDatamodule(
  const AClass: TDatamoduleClass);

Var
  DM : TDatamodule;
  I,J : Integer;
  C : TComponent;
  D : TWebDataProviderDef;

begin
  FRegistering:=True;
  try
    DM:=AClass.Create(Self);
    try
      For I:=0 to DM.ComponentCount-1 do
        begin
        C:=DM.Components[i];
        if C is TFPCustomWebDataProvider then
          begin
          J:=IndexOfProviderDef(C.Name);
          If (J<>-1) then
            Raise EFPHTTPError.CreateFmt(SErrDuplicateWebDataProvider,[C.Name]);
          D:=AddProviderDef(C.Name);
          {$ifdef wmdebug}SendDebug('Registering provider '+C.Name);{$endif}
          D.FDataModuleClass:=TDataModuleClass(DM.ClassType);
          end;
        end;
    finally
      DM.Free;
    end;
  finally
    FRegistering:=False;
  end;
end;

function TFPCustomWebDataProviderManager.RegisterProvider(
  const AProviderName: String; AClass: TFPCustomWebDataProviderClass
  ): TWebDataProviderDef;

Var
  I : Integer;

begin
  FRegistering:=True;
  try
    I:=IndexOfProviderDef(AProviderName);
    If (I<>-1) then
      Raise EFPHTTPError.CreateFmt(SErrDuplicateWebDataProvider,[AProviderName]);
    Result:=AddProviderDef(AProviderName);
    Result.ProviderClass:=AClass;
  finally
    FRegistering:=False;
  end;
end;

function TFPCustomWebDataProviderManager.FindProviderDefByName(
  const AProviderName: String): TWebDataProviderDef;

Var
  I : integer;

begin
  I:=IndexOfProviderDef(AProviderName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetProviderDef(I);
end;

function TFPCustomWebDataProviderManager.GetProviderDefByName(
  const AProviderName: String): TWebDataProviderDef;
begin
  Result:=FindProviderDefByName(AProviderName);
  If (Result=Nil) then
    Raise EFPHTTPError.CreateFmt(SErrUnknownWebDataProvider,[AProviderName]);
end;

function TFPCustomWebDataProviderManager.GetProvider(
  const AProviderName: String; AOwner: TComponent; Out AContainer : TComponent): TFPCustomWebDataProvider;

Var
  D : TWebDataProviderDef;

begin
  D:=GetProviderDefByname(AProviderName);
  Result:=GetProvider(D,AOwner,AContainer);
end;

function TFPCustomWebDataProviderManager.RegisterInputAdaptor(
  const AAdaptorName: String; AClass: TCustomWebdataInputAdaptorClass
  ): TWebInputAdaptorDef;
begin
  If IndexOfInputAdaptorDef(AAdaptorName)<>-1 then
    Raise EFPHTTPError.CreateFmt(SErrDuplicateAdaptor,[AAdaptorName]);
  Result:=AddInputAdaptorDef(AAdaptorName,AClass);
end;

procedure TFPCustomWebDataProviderManager.UnRegisterInputAdaptor(
  const AAdaptorName: String);

Var
  I : Integer;

begin
  I:=IndexOfInputAdaptorDef(AAdaptorName);
  If (I<>-1) then
     RemoveInputAdaptorDef(I);

end;

function TFPCustomWebDataProviderManager.FindInputAdaptorDefByName(
  const AAdaptorName: String): TWebInputAdaptorDef;

Var
  I: integer;

begin
  I:=IndexOfInputAdaptorDef(AAdaptorName);
  If I<>-1 then
    Result:=GetInputAdaptorDef(I)
  else
    Result:=Nil;
end;

function TFPCustomWebDataProviderManager.GetInputAdaptorDefByName(
  const AAdaptorName: String): TWebInputAdaptorDef;
begin
  Result:=FindInputAdaptorDefByName(AAdaptorName);
  If (Result=Nil) then
    Raise EFPHTTPError.CreateFmt(SErrUnknownInputAdaptor,[AAdaptorName]);
end;

function TFPCustomWebDataProviderManager.GetInputAdaptor(
  const ADef: TWebInputAdaptorDef; AOwner: TComponent
  ): TCustomWebdataInputAdaptor;

Var
  O: TComponent;

begin
  O:=AOwner;
  If (O=Nil) then
    O:=Self;
  Result:=ADef.CreateInstance(AOwner);
end;

function TFPCustomWebDataProviderManager.GetInputAdaptor(
  const AAdaptorName: String; AOwner: TComponent): TCustomWebdataInputAdaptor;
begin
  Result:=GetInputAdaptor(GetInputAdaptorDefByName(AAdaptorName),Aowner);
end;

function TFPCustomWebDataProviderManager.RegisterDataProducer(
  const AProducerName: String; AClass: TCustomHTTPDataContentProducerClass
  ): THttpDataProducerDef;
begin
  If IndexOfHttpDataProducerDef(AProducerName)<>-1 then
    Raise EFPHTTPError.CreateFmt(SErrDuplicateHTTPDataProducer,[AProducerName]);
  Result:=AddHttpDataProducerDef(AProducerName,AClass);
end;

procedure TFPCustomWebDataProviderManager.UnRegisterDataProducer(
  const AProducerName: String);

Var
  I : Integer;

begin
  I:=IndexOfHttpDataProducerDef(AProducerName);
  If (I<>-1) then
    RemoveHttpDataProducerDef(I);
end;

function TFPCustomWebDataProviderManager.FindDataProducerDefByName(
  const AProducerName: String): THttpDataProducerDef;
Var
  I : Integer;

begin
  I:=IndexOfHttpDataProducerDef(AProducerName);
  If (I<>-1) then
    Result:=GetHttpDataProducerDef(I)
  else
    Result:=Nil;

end;

function TFPCustomWebDataProviderManager.GetDataProducerDefByName(
  const AProducerName: String): THttpDataProducerDef;
begin
  Result:=FindDataProducerDefByName(AProducerName);
  If (Result=Nil) then
    Raise EFPHTTPError.CreateFmt(SErrUnknownHTTPDataProducer,[AProducerName]);
end;

function TFPCustomWebDataProviderManager.GetDataProducer(
  ADef: THttpDataProducerDef; AOwner: TComponent
  ): TCustomHTTPDataContentProducer;

Var
  O : TComponent;

begin
  O:=AOwner;
  If (O=Nil) then
    O:=Self;
  Result:=ADef.CreateInstance(Aowner);
end;

function TFPCustomWebDataProviderManager.GetDataProducer(
  const AProducerName: String; AOwner : TComponent): TCustomHTTPDataContentProducer;
begin
  Result:=GetDataProducer(GetDataProducerDefByName(AProducerName),Aowner);
end;

function TFPCustomWebDataProviderManager.GetProvider(
  const ADef: TWebDataProviderDef; AOwner: TComponent; Out AContainer : TComponent): TFPCustomWebDataProvider;

Var
  O : TComponent;
begin
  If AOwner<>Nil then
    O:=Self
  else
    O:=AOwner;
  Result:=ADef.CreateInstance(O,AContainer);
end;

{ TFPWebDataProviderManager }

constructor TFPWebDataProviderManager.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FProviderDefs:=TWebDataProviderDefs.Create(TWebDataProviderDef);
  FAdaptorDefs:=TWebInputAdaptorDefs.Create(TWebInputAdaptorDef);
  FProducerDefs:=THttpDataProducerDefs.Create(THttpDataProducerDef);
end;

destructor TFPWebDataProviderManager.Destroy;
begin
  FreeAndNil(FProviderDefs);
  FreeAndNil(FAdaptorDefs);
  FreeAndNil(FProducerDefs);
  inherited Destroy;
end;

procedure TFPWebDataProviderManager.RemoveProviderDef(const Index: Integer);

begin
  FProviderDefs.Delete(Index);
end;

function TFPWebDataProviderManager.AddProviderDef(const AProviderName: String
  ): TWebDataProviderDef;
begin
  Result:=FProviderDefs.AddProvider(AProviderName);
end;

function TFPWebDataProviderManager.IndexOfProviderDef(const AProviderName: String
  ): Integer;
begin
  {$ifdef wmdebug}Senddebug('Entering indexofproviderdef : '+AProviderName);{$endif}
  {$ifdef wmdebug}Senddebug(Format('Providerdefs assigned: %d ',[Ord(Assigned(FProviderDefs))]));{$endif}
  Result:=FProviderDefs.IndexOfProvider(AProviderName);
  {$ifdef wmdebug}Senddebug('Exitining indexofproviderdef: '+IntToStr(result));{$endif}
end;

function TFPWebDataProviderManager.GetProviderDef(Index: Integer
  ): TWebDataProviderDef;
begin
  Result:=FProviderDefs[Index];
end;

function TFPWebDataProviderManager.GetProviderDefCount: Integer;
begin
  Result:=FProviderDefs.Count;
end;

function TFPWebDataProviderManager.AddInputAdaptorDef(
  const AAdaptorName: String; AClass: TCustomWebdataInputAdaptorClass
  ): TWebInputAdaptorDef;
begin
  Result:=FAdaptorDefs.AddAdaptor(AAdaptorName,AClass);
end;

function TFPWebDataProviderManager.IndexOfInputAdaptorDef(
  const AAdaptorName: String): Integer;
begin
  Result:=FAdaptorDefs.IndexOfAdaptor(AAdaptorName);
end;

Procedure TFPWebDataProviderManager.RemoveInputAdaptorDef(Index : integer);

begin
  If (Index<>-1) then
    FAdaptorDefs.Delete(Index);
end;

function TFPWebDataProviderManager.GetInputAdaptorDef(Index: Integer
  ): TWebInputAdaptorDef;
begin
  Result:=FAdaptorDefs[Index];
end;

function TFPWebDataProviderManager.GetInputAdaptorDefCount: Integer;
begin
  Result:=FAdaptorDefs.Count;
end;

function TFPWebDataProviderManager.AddHttpDataProducerDef(
  const AProducerName: String; AClass: TCustomHTTPDataContentProducerClass
  ): THttpDataProducerDef;
begin
  Result:=FProducerDefs.AddProducer(AProducerName,AClass);
end;

function TFPWebDataProviderManager.IndexOfHttpDataProducerDef(
  const AProducerName: String): Integer;
begin
  Result:=FProducerDefs.IndexOfProducer(AProducerName);
end;

procedure TFPWebDataProviderManager.RemoveHttpDataProducerDef(Index: Integer);
begin
  FProducerDefs.Delete(Index);
end;

function TFPWebDataProviderManager.GetHttpDataProducerDef(Index: Integer
  ): THttpDataProducerDef;
begin
  Result:=FProducerDefs[Index];
end;

function TFPWebDataProviderManager.GetHttpDataProducerDefCount: Integer;
begin
  Result:=FProducerDefs.Count;
end;

{ TFPCustomWebProviderDataModule }

procedure TFPCustomWebProviderDataModule.ReadWebData(AProvider: TFPCustomWebDataProvider
  );

Var
  B : Boolean;

begin
  B:=False;
  If Assigned(FBeforeRead) then
    FBeforeRead(Self,AProvider,B);
  if Not B then
      DoReadWebData(AProvider);
  If Assigned(FAfterRead) then
    FAfterRead(Self,AProvider);
end;

procedure TFPCustomWebProviderDataModule.InsertWebData(
  AProvider: TFPCustomWebDataProvider);

Var
  B : Boolean;

begin
  B:=False;
  If Assigned(FBeforeInsert) then
    FBeforeInsert(Self,AProvider,B);
  if Not B then
      DoInsertWebData(AProvider);
  If Assigned(FAfterInsert) then
    FAfterInsert(Self,AProvider);
end;

procedure TFPCustomWebProviderDataModule.SetContentProducer(
  const AValue: TCustomHTTPDataContentProducer);
begin
  if FContentProducer=AValue then exit;
  FContentProducer:=AValue;
end;

procedure TFPCustomWebProviderDataModule.SetInputAdaptor(
  const AValue: TCustomWebdataInputAdaptor);
begin
  if FInputAdaptor=AValue then exit;
  FInputAdaptor:=AValue;
end;

procedure TFPCustomWebProviderDataModule.UpdateWebData(
  AProvider: TFPCustomWebDataProvider);

Var
  B : Boolean;

begin
  B:=False;
  If Assigned(FBeforeUpdate) then
    FBeforeUpdate(Self,AProvider,B);
  if Not B then
      DoUpdateWebData(AProvider);
  If Assigned(FAfterUpdate) then
    FAfterUpdate(Self,AProvider);
end;

procedure TFPCustomWebProviderDataModule.DeleteWebData(
  AProvider: TFPCustomWebDataProvider);
Var
  B : Boolean;

begin
  B:=False;
  If Assigned(FBeforeDelete) then
    FBeforeDelete(Self,AProvider,B);
  if Not B then
      DoDeleteWebData(AProvider);
  If Assigned(FAfterDelete) then
    FAfterDelete(Self,AProvider);
end;

Function TFPCustomWebProviderDataModule.GetAdaptor : TCustomWebdataInputAdaptor;

begin
  Result:=Self.InputAdaptor;
  If Assigned(FOnGetInputAdaptor) then
    FOnGetInputAdaptor(Self,Result);
end;

function TFPCustomWebProviderDataModule.GetContentProducer: TCustomHTTPDataContentProducer;
begin
  Result:=FContentProducer;
  If Assigned(FOnGetContentProducer) then
    FOnGetContentProducer(Self,Result);
end;

procedure TFPCustomWebProviderDataModule.ProduceContent(
  AProvider: TFPCustomWebDataProvider);

Var
  A : TCustomWebdataInputAdaptor;
  C : TCustomHTTPDataContentProducer;
  Handled : boolean;
  M : TmemoryStream;
begin
  A:=GetAdaptor;
  A.Request:=Self.Request;
  AProvider.Adaptor:=A;
  C:=GetContentProducer;
  C.Adaptor:=A;
  C.Provider:=AProvider;
  M:=TMemoryStream.Create;
  try
    Handled:=True;
    C.GetContent(Request,M,Handled);
    If Handled then
      begin
      M.Position:=0;
      If Assigned(FOnContent) then
        FOnContent(Self,M);
      Response.ContentType:=C.DataContentType;
      Response.ContentStream:=M;
      Response.SendResponse;
      Response.ContentStream:=Nil;
      end
    else
      Raise EFPHTTPError.CreateFmt(SErrContentProviderRequest,[C.Name]);
  finally
    M.Free;
  end;
end;

procedure TFPCustomWebProviderDataModule.DoReadWebData(
  AProvider: TFPCustomWebDataProvider);

begin
  ProduceContent(AProvider);
end;

procedure TFPCustomWebProviderDataModule.DoInsertWebData(
  AProvider: TFPCustomWebDataProvider);
begin
  ProduceContent(AProvider);
end;

procedure TFPCustomWebProviderDataModule.DoUpdateWebData(
  AProvider: TFPCustomWebDataProvider);
begin
  ProduceContent(AProvider);
end;

procedure TFPCustomWebProviderDataModule.DoDeleteWebData(
  AProvider: TFPCustomWebDataProvider);
begin
  ProduceContent(AProvider);
end;

Constructor TFPCustomWebProviderDataModule.CreateNew(AOwner : TComponent; CreateMode : Integer);
begin
  inherited;
  FUseProviderManager:=True;
end;

Function TFPCustomWebProviderDataModule.GetProvider(Const AProviderName : String; Out AContainer : TComponent) : TFPCustomWebDataProvider;

Var
  C : TComponent;
  ADef : TWebDataProviderDef;
  P : TFPCustomWebDataProvider;

begin
  Result:=Nil;
  AContainer:=Nil;
  If Assigned(FOnGetProvider) then
    begin
    FOngetProvider(Self,AProviderName,Result);
    If Assigned(Result) then
      begin
      AContainer:=Nil;
      Exit;
      end;
    end;
  P:=Nil;
  C:=FindComponent(AProviderName);
  {$ifdef wmdebug}SendDebug(Format('Searching provider "%s" 1 : %d ',[AProvidername,Ord(Assigned(C))]));{$endif}
  If (C<>Nil) and (C is TFPCustomWebDataProvider) then
    P:=TFPCustomWebDataProvider(C)
  else if UseProviderManager then
    begin
    {$ifdef wmdebug}SendDebug(Format('Searching providerdef "%s" 1 : %d ',[AProvidername,Ord(Assigned(C))]));{$endif}
    ADef:=WebDataProviderManager.FindProviderDefByName(AProviderName);
    If (ADef<>Nil) then
      begin
      {$ifdef wmdebug}SendDebug(Format('Found providerdef "%s" 1 : %d ',[AProvidername,Ord(Assigned(C))]));{$endif}
      P:=WebDataProviderManager.GetProvider(ADef,Self,AContainer);
      end
    else
      P:=Nil;
    end;
  {$ifdef wmdebug}SendDebug(Format('Searching provider "%s" 2 : %d ',[AProvidername,Ord(Assigned(C))]));{$endif}
  Result:=P;
  If (Result=Nil) then
    Raise EFPHTTPError.CreateFmt(SErrUnknownWebDataProvider,[AProviderName]);
end;

procedure TFPCustomWebProviderDataModule.HandleRequest(ARequest: TRequest;
  AResponse: TResponse);

Var
  ProviderName : String;
  AProvider : TFPCustomWebDataProvider;
  A : TCustomWebdataInputAdaptor;
  Wa : TWebDataAction;
  AContainer : TComponent;

begin
  FRequest:=ARequest;
  FResponse:=AResponse;
  try
    {$ifdef wmdebug}SendDebug('Checking session');{$endif}
    CheckSession(ARequest);
    {$ifdef wmdebug}SendDebug('Init session');{$endif}
    InitSession(AResponse);
    {$ifdef wmdebug}SendDebug('Getting providername');{$endif}
    ProviderName:=Request.GetNextPathInfo;
    {$ifdef wmdebug}SendDebug('Handlerequest, providername : '+Providername);{$endif}
    AProvider:=GetProvider(ProviderName,AContainer);
    try
      If not CORS.HandleRequest(aRequest,aResponse,[hcDetect,hcSend]) then
        begin
        A:=GetAdaptor;
        A.Request:=ARequest;
        A.Reset; // Force. for wmKind=pooled, fastcgi, request can be the same.
        Wa:=A.GetAction;
        Case WA of
          wdaUnknown : Raise EFPHTTPError.CreateFmt(SErrUnknownProviderAction,[ProviderName]);
          wdaRead    : ReadWebData(AProvider);
          wdaUpdate  : UpdateWebData(AProvider);
          wdaInsert  : InsertWebdata(AProvider);
          wdaDelete  : DeleteWebData(AProvider);
        end;
        UpdateSession(AResponse);
        end;
    finally
      If (AContainer=Nil) then
        begin
        If (AProvider.Owner<>Self) then
          AProvider.Free;
        end
      else
        AContainer.Free;
    end;
  finally
    FRequest:=Nil;
    FResponse:=Nil;
  end;
end;

{ TWebInputAdaptorDef }

procedure TWebInputAdaptorDef.SetName(const AValue: String);
begin
  if FName=AValue then exit;
  FName:=AValue;
end;

function TWebInputAdaptorDef.CreateInstance(AOwner: TComponent
  ): TCustomWebdataInputAdaptor;
begin
  Result:=FClass.Create(AOwner);
end;

{ TWebInputAdaptorDefs }

function TWebInputAdaptorDefs.GetD(Index : Integer): TWebInputAdaptorDef;
begin
  Result:=TWebInputAdaptorDef(Items[Index]);
end;

procedure TWebInputAdaptorDefs.SetD(Index : Integer;
  const AValue: TWebInputAdaptorDef);
begin
  Items[Index]:=AValue;
end;

function TWebInputAdaptorDefs.IndexOfAdaptor(const AAdaptorName: String
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetD(Result).Name,AAdaptorName)<>0) do
    Dec(Result);
end;

function TWebInputAdaptorDefs.AddAdaptor(const AAdaptorName: String;
  AClass: TCustomWebdataInputAdaptorClass): TWebInputAdaptorDef;

Var
  I : Integer;

begin
  I:=IndexOfAdaptor(AAdaptorName);
  If (I=-1) then
    begin
    Result:=Add as TWebInputAdaptorDef;
    Result.FName:=AAdaptorName;
    Result.FClass:=AClass;
    end
  else
    Raise EFPHTTPError.CreateFmt(SErrDuplicateAdaptor,[AAdaptorName]);
end;

{ THttpDataProducerDef }

procedure THttpDataProducerDef.SetName(const AValue: String);

begin
  If AValue=FName then exit;
  If (AValue<>'') and Assigned(Collection) and (Collection is THttpDataProducerDefs) then
    if THttpDataProducerDefs(Collection).IndexOfProducer(AValue)<>-1 then
      Raise EFPHTTPError.CreateFmt(SErrDuplicateHTTPDataProducer,[AValue]);
  FName:=Avalue;
end;

function THttpDataProducerDef.CreateInstance(AOwner: TComponent
  ): TCustomHTTPDataContentProducer;
begin
  Result:=FClass.Create(AOwner);
end;

{ THttpDataProducerDefs }

function THttpDataProducerDefs.GetD(Index: Integer): THttpDataProducerDef;
begin
  Result:=THttpDataProducerDef(Items[Index]);
end;

procedure THttpDataProducerDefs.SetD(Index: Integer;
  const AValue: THttpDataProducerDef);
begin
  Items[Index]:=AValue;
end;

function THttpDataProducerDefs.IndexOfProducer(const AProducerName: String
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetD(Result).Name,AProducerName)<>0) do
    Dec(Result);
end;

function THttpDataProducerDefs.AddProducer(const AProducerName: String;
  AClass: TCustomHTTPDataContentProducerClass): THttpDataProducerDef;

Var
  I : Integer;

begin
  I:=IndexOfProducer(AProducerName);
  If (I=-1) then
    begin
    Result:=Add as THttpDataProducerDef;
    Result.FName:=AProducerName;
    Result.FClass:=AClass;
    end
  else
    Raise EFPHTTPError.CreateFmt(SErrDuplicateHTTPDataProducer,[AProducerName]);
end;

{ TWebdataInputAdaptor }

procedure TWebdataInputAdaptor.SetInputFormat(const AValue: String);
begin
  if FInputFormat=AValue then exit;
  If Assigned(FProxy) then
    ClearProxy;
  FInputFormat:=AValue;
end;

procedure TWebdataInputAdaptor.ClearProxy;
begin
  FreeAndNil(FProxy);
end;

procedure TWebdataInputAdaptor.CheckProxy;
begin
  If (FProxy=Nil) then
    FProxy:=CreateProxy;
end;

function TWebdataInputAdaptor.CreateProxy: TCustomWebdataInputAdaptor;
begin
  Result:=WebDataProviderManager.GetInputAdaptor(FInputFormat);
end;

function TWebdataInputAdaptor.GetActionFromRequest: TWebDataAction;
begin
  CheckProxy;
  Result:=FProxy.GetActionFromRequest;
end;

destructor TWebdataInputAdaptor.Destroy;
begin
  ClearProxy;
  Inherited;
end;

function TWebdataInputAdaptor.GetNextBatch: Boolean;
begin
  CheckProxy;
  Result:=FProxy.GetNextBatch;
end;

function TWebdataInputAdaptor.TryParamValue(const AParamName: String; out
  AValue: String): Boolean;
begin
  CheckProxy;
  Result:=FProxy.TryParamValue(AParamName, AValue);
end;

function TWebdataInputAdaptor.TryFieldValue(const AFieldName: String; out
  AValue: String): Boolean;
begin
  CheckProxy;
  Result:=FProxy.TryFieldValue(AFieldName, AValue);
end;

{ THTTPDataContentProducer }

procedure THTTPDataContentProducer.SetOutputFormat(const AValue: String);
begin
  if FOutputFormat=AValue then exit;
  If Assigned(FProxy) then
    ClearProxy;
  FOutputFormat:=AValue;
end;

procedure THTTPDataContentProducer.ClearProxy;
begin
  FreeAndNil(FProxy);
end;

procedure THTTPDataContentProducer.CheckProxy;
begin
  If not Assigned(FProxy) then
    begin
    FProxy:=CreateProxy;
    end;
end;

function THTTPDataContentProducer.CreateProxy: TCustomHTTPDataContentProducer;
begin
  Result:=WebDataProviderManager.GetDataProducer(FOutputFormat,Self);
  ConfigureProxy(Result);
end;

Procedure THTTPDataContentProducer.ConfigureProxy(AProxy : TCustomHTTPDataContentProducer);
begin
  AProxy.PageSize:=Self.PageSize;
  AProxy.PageStart:=Self.PageStart;
  AProxy.MetaData:=Self.MetaData;
  AProxy.SortField:=Self.SortField;
  AProxy.SortDescending:=Self.SortDescending;
  AProxy.AllowPageSize:=Self.AllowPageSize;
  If Assigned(FOnConfigure) then
     FOnConfigure(AProxy);
end;

destructor THTTPDataContentProducer.destroy;
begin
  ClearProxy;
  inherited destroy;
end;

initialization

finalization
  FreeAndNil(AWebDataProviderManager);
end.

