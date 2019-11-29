unit fpDBExport;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB;
  
Type
  TCustomDatasetExporter = Class;

  // Quote string fields if value contains a space or delimiter char.
  TQuoteString = (qsAlways,qsSpace,qsDelimiter);
  TQuoteStrings = Set of TQuoteString;

  TAlignField = (afLeft,afRight);

  { TExportFieldItem }

  TExportFieldItem = Class(TCollectionItem)
  private
    FEnabled: Boolean;
    FField: TField;
    FFieldName: String;
    FExportedName: UTF8String;
    function GetExportedName: UTF8String;
    function GetExporter: TCustomDatasetExporter;
    procedure SetExportedName(const AValue: UTF8String);
  Protected
    Procedure BindField (ADataset : TDataset); virtual;
    procedure SetFieldName(const AValue: String); virtual;
  Public
    Constructor Create(ACollection : TCollection); override;
    Procedure Assign(Source : TPersistent); override;
    Property Field : TField Read FField;
    Property Exporter : TCustomDatasetExporter Read GetExporter;
  Published
    Property Enabled : Boolean Read FEnabled Write FEnabled default True;
    Property FieldName : String Read FFieldName Write SetFieldName;
    Property ExportedName : UTF8String Read GetExportedName Write SetExportedName;
  end;
  
  { TExportFields }

  TExportFields = Class(TCollection)
  private
    FExporter : TCustomDatasetExporter;
    function GetFieldItem(Index : Integer): TExportFieldItem;
    procedure SetFieldItem(Index : Integer; const AValue: TExportFieldItem);
  Public
    Function IndexOfField(Const AFieldName : String) : Integer;
    Function IndexOfExportedName(Const AFieldName : String) : Integer;
    Function FindExportField(Const AFieldName : String) : TExportFieldItem;
    Function FindExportName(Const AFieldName : String) : TExportFieldItem;
    Function AddField(Const AFieldName : String) : TExportFieldItem; virtual;
    Property Fields[Index : Integer] : TExportFieldItem Read GetFieldItem Write SetFieldItem; Default;
    Property Exporter : TCustomDatasetExporter Read FExporter;
  end;


  { TCustomExportFormatSettings }

  TCustomExportFormatSettings = Class(TPersistent)
  private
    FBooleanFalse : String;
    FBooleanTrue : String;
    FCurrencyDigits: Integer;
    FCurrencySymbol : String;
    FDateFormat : String;
    FIntegerFormat: String;
    FHandleNullField: Boolean;
    FTimeFormat : String;
    FDateTimeFormat : String;
    FDecimalSeparator: Char;
    FUseDisplayText : Boolean;
  Protected
    Procedure InitSettings; virtual;
    Property HandleNullField : Boolean Read FHandleNullField Write FHandleNullField;
    Property UseDisplayText : Boolean Read FUseDisplayText Write FUseDisplayText;
    Property IntegerFormat : String Read FIntegerFormat Write FIntegerFormat;
    Property DecimalSeparator : Char Read FDecimalSeparator Write FDecimalSeparator;
    Property CurrencySymbol : String Read FCurrencySymbol Write FCurrencySymbol;
    Property CurrencyDigits : Integer Read FCurrencyDigits Write FCurrencyDigits;
    Property BooleanTrue : String Read FBooleanTrue Write FBooleanTrue;
    Property BooleanFalse : String Read FBooleanFalse Write FBooleanFalse;
    Property DateFormat : String Read FDateFormat Write FDateFormat;
    Property TimeFormat : String Read FTimeFormat Write FTimeFormat;
    Property DateTimeFormat : String Read FDateTimeFormat Write FDateTimeFormat;
  Public
    Constructor Create(DoInitSettings : Boolean); virtual;
    Procedure Assign(Source : TPersistent); override;
  end;
  TCustomExportFormatSettingsClass = Class of TCustomExportFormatSettings;
  
  { TExportFormatSettings }
  TExportFormatSettings = Class(TCustomExportFormatSettings)
  Published
    Property IntegerFormat;
    Property DecimalSeparator;
    Property CurrencySymbol;
    Property CurrencyDigits;
    Property BooleanTrue;
    Property BooleanFalse ;
    Property DateFormat;
    Property TimeFormat;
    Property DateTimeFormat;
  end;

  TOnExportRowEvent = Procedure(Sender : TObject; Var AllowExport : Boolean) of object;
  TExportProgressEvent = Procedure(Sender : TObject; Const ItemNo : Integer) of object;
  { TCustomDatasetExporter }

  TCustomDatasetExporter = Class(TComponent)
  private
    FAfterExecute: TNotifyEvent;
    FBeforeExecute: TNotifyEvent;
    FCanceled: Boolean;
    FDataset: TDataset;
    FFormatSettings: TCustomExportFormatSettings;
    FExportFields: TExportFields;
    FFromCurrent: Boolean;
    FOnExportRow: TOnExportRowEvent;
    FonProgress: TExportProgressEvent;
    FRestorePosition: Boolean;
    procedure SetDataset(const AValue: TDataset);
    procedure SetExportFields(const AValue: TExportFields);
    procedure SetFormatSettings(const AValue: TCustomExportFormatSettings);
  Protected
    // Override this if you need a descendent of TExportFormatSettings
    Function CreateFormatSettings : TCustomExportFormatSettings; virtual;
    // Checks if Dataset is assigned and whether it is in browse mode.
    Procedure CheckDataset(InBrowse : Boolean);
    // Allocate TField in TExportFieldItem
    Function BindFields : Boolean; virtual;
    // Nil out fields.
    Procedure UnbindFields;
    // Override if a descendent of TExportFieldItem is needed.
    Function CreateExportFields : TExportFields; Virtual;
    // Executes BeforeExecute event. Override (but call inherited)
    Procedure DoBeforeExecute; virtual;
    // Executes AfterExecute event. Override (but call inherited)
    // Note this is also executed in case of an exception !!
    Procedure DoAfterExecute; virtual;
    // Returns True if current row should be exported
    Function DoDataRow : Boolean; virtual;
    // Override to write data prior to data start.
    Procedure DoDataHeader; virtual;
    // Override to write data after data start.
    Procedure DoDataFooter; virtual;
    // Override to write something at row start.
    Procedure DoDataRowStart; virtual;
    // Override if a simple loop is not enough.
    Procedure ExportDataRow; virtual;
    // Override to write something at row end.
    Procedure DoDataRowEnd; virtual;
    // Called after row was exported
    Procedure DoProgress(ItemNo : Integer); Virtual;
    // Override if each field can be written as-is.
    Procedure ExportField(EF : TExportFieldItem); virtual;
    // Format field as UTF8 string, according to settings
    Function FormatField(F : TField) : UTF8String; virtual;
    // Raise EDataExporter error
    Procedure ExportError(Msg : String); overload;
    Procedure ExportError(Fmt : String; Args: Array of const); overload;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    // Build default fieldmap - adds all fields.
    Procedure BuildDefaultFieldMap(AMap : TExportFields); virtual;
    // Do export. Returns the number of records exported.
    Function Execute : Integer; virtual;
    // Call this to cancel the export
    Procedure Cancel;
    // Show the default configuration dialog, if one was assigned.
    // Returns false if the dialog was cancelled.
    Function ShowConfigDialog : Boolean;
    // Don't use. Needed for nil of dataset.
    Procedure Notification(AComponent: TComponent; Operation : TOperation); override;
    // True if export was canceled (using Cancel);
    Property Canceled : Boolean Read FCanceled;
  Public
    // Properties
    Property Dataset : TDataset Read FDataset Write SetDataset;
    Property ExportFields : TExportFields Read FExportFields Write SetExportFields;
    Property FromCurrent : Boolean Read FFromCurrent Write FFromCurrent Default True;
    Property RestorePosition : Boolean Read FRestorePosition Write FRestorePosition;
    Property FormatSettings : TCustomExportFormatSettings Read FFormatSettings Write SetFormatSettings;
    // Events
    Property AfterExecute : TNotifyEvent Read FAfterExecute Write FAfterExecute;
    Property BeforeExecute : TNotifyEvent Read FBeforeExecute Write FBeforeExecute;
    Property OnExportRow : TOnExportRowEvent Read FOnExportRow Write FOnExportRow;
    Property OnProgress : TExportProgressEvent Read FonProgress Write FOnProgress;
  end;
  
  TCustomDatasetExporterClass = Class of TCustomDatasetExporter;

  { TStreamExporter }

  TStreamExporter = Class(TCustomDatasetExporter)
  Private
    FStream: TStream;
  Protected
    Property Stream : TStream Read FStream;
    // Frees the stream.
    Procedure CloseStream;
  Public
    Procedure ExportToStream(AStream : TStream);
  end;
  
  { TCustomFileExporter }

  TCustomFileExporter = Class(TStreamExporter)
  private
    FFileName: String;
    FTextFile: Text;
    FTextFileOpen: Boolean;
    FopenedStream : Boolean;
  protected
    // Creates a file stream
    procedure OpenStream; virtual;
    // Override if some checking needs to be done on valid names
    procedure SetFileName(const AValue: String); virtual;
    // Override if some checking needs to be done prior to opening.
    Procedure CheckFileName; virtual;
    // Use to open textfile. Creates a file stream.
    Procedure OpenTextFile;
    // Use to close textfile.
    Procedure CloseTextFile;
    // Access to stream/file
    Property TextFile : Text Read FTextFile;
    Property TextFileOpen : Boolean Read FTextFileOpen;
  Public
    Destructor Destroy; override;
    // Publish in descendents.
    Property FileName : String Read FFileName Write SetFileName;
  end;
  




  EDataExporter = Class(Exception);
  
  { TExportFormatItem }

  TExportConfigureEvent = Function (Exporter : TCustomDatasetExporter) : Boolean of object;

  TExportFormatItem = Class(TCollectionItem)
  private
    FClass: TCustomDatasetExporterClass;
    FDescription: String;
    FExtensions: String;
    FName: String;
    FOnConfigure: TExportConfigureEvent;
    procedure SetName(const AValue: String);
  Public
    Property ExportClass : TCustomDatasetExporterClass Read FClass Write FClass;
  Published
    Property Name : String Read FName Write SetName;
    Property Description : String Read FDescription Write FDescription;
    Property Extensions : String Read FExtensions Write FExtensions;
    Property OnConfigureDialog : TExportConfigureEvent Read FOnConfigure Write FOnConfigure;
  end;
  
  TExportFormats = Class(TCollection)
  private
    function GetFormat(Index : Integer): TExportFormatItem;
    procedure SetFormat(Index : Integer; const AValue: TExportFormatItem);
  Public
    // Registration/Unregistration
    Function RegisterExportFormat(Const AName,ADescription,AExtensions : String; AClass : TCustomDatasetExporterClass) : TExportFormatItem;
    Procedure UnRegisterExportFormat(AClass : TCustomDatasetExporterClass);
    Procedure UnRegisterExportFormat(Const AName : String);
    // Searching
    Function IndexOfFormat(Const AName : String): Integer;
    Function IndexOfExportClass(AClass : TCustomDataSetExporterClass): Integer;
    Function FindFormat(Const AName : String) : TExportFormatItem;
    Function FindFormatByClass(AClass : TCustomDataSetExporterClass) : TExportFormatItem;
    // Shows configuration dialog, if one was configured for this class
    Function ConfigureExport(AnExport : TCustomDatasetExporter) : Boolean;
    Function FormatByName(Const AName : String) : TExportFormatItem;
    // Utilityfunctions
    Function ConstructFilter(AnExport : TCustomDatasetExporter) : String;
    Property Formats[Index : Integer] : TExportFormatItem Read GetFormat Write SetFormat; default;
  end;
  
Function ExportFormats : TExportFormats;

// Easy access functions

Function RegisterExportFormat(Const AName,ADescription,AExtensions : String; AClass : TCustomDatasetExporterClass) : TExportFormatItem;
Procedure UnRegisterExportFormat(AClass : TCustomDatasetExporterClass);
Procedure UnRegisterExportFormat(Const AName : String);


Const
  StringFieldTypes = [ftString,ftFixedChar,ftWidestring,ftFixedWideChar];
  IntFieldTypes    = [ftInteger,ftWord,ftSmallint,ftAutoinc];
  FloatFieldTypes = [ftFloat, ftCurrency, ftFMTBcd, ftBCD];
  OrdFieldTypes    = IntFieldTypes +[ftBoolean,ftLargeInt];
  DateFieldTypes   = [ftDate,ftTime,ftDateTime,ftTimeStamp];
  MemoFieldTypes   = [ftMemo,ftFmtMemo,ftWideMemo];
  BlobFieldTypes   =  [ftBlob,ftDBaseOLE,ftGraphic,ftOraBlob,ftOraClob,ftParadoxOLE];
  

implementation

uses streamio;

ResourceString
  SErrNoDataset           = 'Dataset not assigned';
  SErrNoBrowse            = 'Dataset not in browse mode';
  SErrNoFileName          = 'No filename set for export';
  SErrFormatExists        = 'An export format with name "%s" already exists.';
  SUnknownExportFormat    = 'Unknown export format "%s"';
  SExportFilter           = '%s files';
  SAllFilesFilter         = 'All files';
  SErrDuplicateExportName = 'Exported fieldname "%s" already exists';

{ TExportFieldItem }

procedure TExportFieldItem.SetFieldName(const AValue: String);
begin
  if (FFieldName<>AValue) then
    begin
    FField:=Nil;
    FFieldName:=AValue;
    end;
end;

constructor TExportFieldItem.Create(ACollection: TCollection);
begin
  inherited Create(ACollection);
  FEnabled:=True;
end;

function TExportFieldItem.GetExportedName: UTF8String;
begin
  Result:=FExportedName;
  If (Result='') then
    Result:=FFieldName;
end;

function TExportFieldItem.GetExporter: TCustomDatasetExporter;
begin
  If Collection is TExportFields then
    Result:=(Collection as TExportFields).Exporter;
end;

procedure TExportFieldItem.SetExportedName(const AValue: UTF8String);

Var
  I : TExportFieldItem;
  
begin
  If (FExportedName<>AValue) then
    begin
    If (AValue<>'') then
      begin
      I:=(Collection as TExportFields).FindExportName(AValue);
      If (I<>Nil) and (I<>Self) then
        Raise EDataExporter.CreateFmt(SErrDuplicateExportName,[AValue]);
      end;
    FExportedName:=AValue;
    end;
end;

procedure TExportFieldItem.BindField(ADataset: TDataset);
begin
  FField:=ADataSet.FieldByName(FieldName);
end;

procedure TExportFieldItem.Assign(Source: TPersistent);

Var
  EF : TExportFieldItem;

begin
  if (Source is TExportFieldItem) then
    begin
    EF:=Source as TExportFieldItem;
    FieldName:=EF.FieldName;
    Enabled:=EF.Enabled;
    FExportedName:=EF.FExportedName;
    end
  else
    Inherited;
end;

{ TExportFields }

function TExportFields.GetFieldItem(Index : Integer): TExportFieldItem;
begin
  Result:=TExportFieldItem(Items[Index]);
end;

procedure TExportFields.SetFieldItem(Index : Integer; const AValue: TExportFieldItem);

begin
  Items[Index]:=AValue;
end;

function TExportFields.IndexOfField(const AFieldName: String): Integer;

begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetFieldItem(Result).FieldName,AFieldName)<>0) do
    Dec(Result);
end;

function TExportFields.IndexOfExportedName(const AFieldName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetFieldItem(Result).ExportedName,AFieldName)<>0) do
    Dec(Result);
end;

function TExportFields.FindExportField(const AFieldName: String
  ): TExportFieldItem;
  
Var
  I : Integer;
  
begin
  I:=IndexOfField(AFieldName);
  If (I<>-1) then
    Result:=GetFieldItem(I)
  else
    Result:=Nil;
end;

function TExportFields.FindExportName(const AFieldName: String
  ): TExportFieldItem;

Var
  I : Integer;

begin
  I:=IndexOfExportedName(AFieldName);
  If (I<>-1) then
    Result:=GetFieldItem(I)
  else
    Result:=Nil;
end;

function TExportFields.AddField(Const AFieldName: String): TExportFieldItem;
begin
  Result:=(Add as TExportFieldItem);
  Result.FieldName:=AFieldName;
end;

{ TCustomDatasetExporter }

procedure TCustomDatasetExporter.SetDataset(const AValue: TDataset);
begin
  if (FDataset<>AValue) then
    begin
    If (FDataset<>Nil) then
      FDataset.RemoveFreeNotification(Self);
    FDataset:=AValue;
    if (FDataset<>Nil) then
      FDataset.FreeNotification(Self);
    UnbindFields;
    end;
end;

procedure TCustomDatasetExporter.SetExportFields(const AValue: TExportFields);
begin
  FExportFields.Assign(AValue);
end;

procedure TCustomDatasetExporter.SetFormatSettings(
  const AValue: TCustomExportFormatSettings);
begin
  if FFormatSettings<>AValue then
    FFormatSettings.Assign(AValue);
end;

Function TCustomDatasetExporter.CreateFormatSettings : TCustomExportFormatSettings;
begin
  Result:=TExportFormatSettings.Create(False);
end;

procedure TCustomDatasetExporter.CheckDataset(InBrowse : Boolean);
begin
  If Not Assigned(Dataset) then
    Raise EDataExporter.Create(SErrNoDataset);
  If InBrowse and Not (Dataset.State=dsBrowse) then
    Raise EDataExporter.Create(SErrNoBrowse);
end;

function TCustomDatasetExporter.BindFields: Boolean;

Var
  I : integer;

begin
  Result:=(ExportFields.Count=0);
  If Result then
    BuildDefaultFieldMap(ExportFields);
  Try
    For I:=0 to ExportFields.Count-1 do
      ExportFields[i].BindField(Dataset);
  except
    UnbindFields;
    Raise;
  end;
end;

procedure TCustomDatasetExporter.UnbindFields;

Var
  I : Integer;

begin
  For I:=0 TO FExportFields.Count-1 do
    Fexportfields[i].FField:=Nil;
end;

Function TCustomDatasetExporter.CreateExportFields : TExportFields;
begin
  Result:=TExportFields.Create(TExportFieldItem);
end;

procedure TCustomDatasetExporter.DoBeforeExecute;
begin
  If Assigned(FBeforeExecute) then
    FBeforeExecute(Self)
end;

procedure TCustomDatasetExporter.DoAfterExecute;
begin
  If Assigned(FAfterExecute) then
    FAfterExecute(Self)
end;

function TCustomDatasetExporter.DoDataRow: Boolean;
begin
  Result:=True;
  If Assigned(FonExportRow) then
    FOnExportRow(Self,Result);
end;

procedure TCustomDatasetExporter.DoDataHeader;
begin
  // Do nothing
end;

procedure TCustomDatasetExporter.DoDataFooter;
begin
  // No nothing
end;

procedure TCustomDatasetExporter.DoDataRowStart;
begin
  // Do nothing
end;

procedure TCustomDatasetExporter.ExportDataRow;

Var
  I : Integer;

begin
  For I:=0 to FExportFields.Count-1 do
    If FExportFields[I].Enabled then
      ExportField(FExportFields[i]);
end;

procedure TCustomDatasetExporter.DoDataRowEnd;
begin
  // Do nothing
end;

procedure TCustomDatasetExporter.DoProgress(ItemNo: Integer);
begin
  If Assigned(FOnProgress) then
    FOnProgress(Self,ItemNo);
end;

procedure TCustomDatasetExporter.ExportField(EF: TExportFieldItem);
begin
  // Do nothing
end;

Function TCustomDatasetExporter.FormatField(F: TField) : UTF8String;

Var
  FS : TFormatSettings;

begin
  if F.IsNull and FormatSettings.HandleNullField then
    Exit('');
  If (F.DataType in IntFieldTypes) then
    begin
    If ((FormatSettings.IntegerFormat)<>'') and (not F.IsNull) then
      Result:=Format(FormatSettings.IntegerFormat,[F.AsInteger])
    else if FormatSettings.UseDisplayText then
      Result:=F.DisplayText
    else
      Result:=F.AsUTF8String;  
    end
  else if (F.DataType=ftBoolean) then
    begin
    if (Not F.IsNull) then
      If F.AsBoolean then
        Result:=FormatSettings.BooleanTrue
      else
        Result:=FormatSettings.BooleanFalse;
    If (Result='') then
      if FormatSettings.UseDisplayText then
        Result:=F.DisplayText
      else
        Result:=F.AsUTF8String;
    end
  else if (F.DataType=ftDate) then
    begin
    If (FormatSettings.DateFormat<>'') and (not F.IsNull) then
      Result:=FormatDateTime(FormatSettings.DateFormat,F.AsDateTime)
    else if FormatSettings.UseDisplayText then
      Result:=F.DisplayText
    else
      Result:=F.AsUTF8String;
    end
  else if (F.DataType=ftTime) then
    begin
    If (FormatSettings.TimeFormat<>'') and (not F.IsNull) then
      Result:=FormatDateTime(FormatSettings.TimeFormat,F.AsDateTime)
    else if FormatSettings.UseDisplayText then
      Result:=F.DisplayText
    else
      Result:=F.AsUTF8String;
    end
  else if (F.DataType in [ftDateTime,ftTimeStamp]) then
    begin
    If (FormatSettings.DateTimeFormat<>'') and (not F.IsNull) then
      Result:=FormatDateTime(FormatSettings.DateTimeFormat,F.AsDateTime)
    else if FormatSettings.UseDisplayText then
      Result:=F.DisplayText
    else
      Result:=F.AsUTF8String;
    end 
  else if (F.DataType=ftCurrency) then
    begin
    If (FormatSettings.CurrencySymbol<>'') and (not F.IsNull) then
      begin
      FS:=DefaultFormatSettings;
      FS.CurrencyString:=FormatSettings.CurrencySymbol;
      Result:=CurrToStrF(F.AsCurrency,ffCurrency,FormatSettings.CurrencyDigits,FS);
      end
    else if FormatSettings.UseDisplayText then
      Result:=F.DisplayText
    else 
      Result:=F.AsUTF8String;
    end
  else if FormatSettings.UseDisplayText then
    Result:=F.DisplayText
  else
    Result:=F.AsUTF8String;
end;

procedure TCustomDatasetExporter.ExportError(Msg: String);
begin
  Raise EDataExporter.Create(Msg);
end;

procedure TCustomDatasetExporter.ExportError(Fmt: String;
  Args: array of const);
begin
  Raise EDataExporter.CreateFmt(Fmt,Args);
end;


constructor TCustomDatasetExporter.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FromCurrent:=True;
  FExportFields:=CreateExportFields;
  FExportFields.FExporter:=Self;
  FFormatSettings:=CreateFormatSettings;
end;

destructor TCustomDatasetExporter.Destroy;
begin
  FreeAndNil(FExportFields);
  FreeAndNil(FFormatSettings);
  inherited Destroy;
end;

procedure TCustomDatasetExporter.BuildDefaultFieldMap(AMap : TExportFields);

Var
  I : Integer;
  F : TField;
  
begin
  CheckDataset(False);
  AMap.Clear;
  For I:=0 to FDataset.Fields.Count-1 do
    begin
    F:=FDataset.Fields[i];
    AMap.AddField(F.FieldName);
    end;
end;



Function TCustomDatasetExporter.Execute : Integer;

Var
  B : Boolean;
  BM : TBookMark;

begin
  Result:=0;
  FCanceled:=False;
  DoBeforeExecute;
  Try
    CheckDataset(True);
    B:=BindFields;
    try
      DoDataHeader;
      Dataset.DisableControls;
      Try
        BM:=Dataset.BookMark;
        try
          If not FromCurrent then
            Dataset.First;
          While not (Dataset.EOF or FCanceled) do
            begin
            if DoDataRow then
              begin
              Inc(Result);
              DoDataRowStart;
              ExportDataRow;
              DoDataRowEnd;
              DoProgress(Result);
              end;
            Dataset.Next;
            end;
          DoDataFooter;
        finally
          If RestorePosition then
            Dataset.BookMark:=Bm;
        end;
      Finally
        Dataset.EnableControls;
      end;
    Finally
      If B then
        FExportFields.Clear;
    end;
  Finally
    DoAfterExecute;
  end;
end;

procedure TCustomDatasetExporter.Cancel;
begin
  FCanceled:=True;
end;

function TCustomDatasetExporter.ShowConfigDialog: Boolean;
begin
  Result:=ExportFormats.ConfigureExport(Self);
end;

procedure TCustomDatasetExporter.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  If (Operation=opRemove) and (AComponent=FDataset) then
    FDataset:=Nil;
  inherited Notification(AComponent, Operation);
end;

{ TStreamExporter }

procedure TStreamExporter.CloseStream;
begin
  FreeAndNil(FStream);
end;

procedure TStreamExporter.ExportToStream(AStream: TStream);
begin
  FStream:=AStream;
  try
    Execute;
  Finally
    FStream:=Nil;
  end;
end;

{ TCustomFileExporter }

procedure TCustomFileExporter.SetFileName(const AValue: String);
begin
  if FFileName=AValue then exit;
  FFileName:=AValue;
end;

procedure TCustomFileExporter.CheckFileName;
begin
  If (FFileName='') then
    ExportError(SErrNoFileName);
end;

procedure TCustomFileExporter.OpenTextFile;

begin
  CheckFileName;
  FOpenedStream:=(Stream=Nil);
  If FOpenedStream then
    OpenStream;
  AssignStream(FTextFile,Stream);
  Rewrite(FTextFile);
  FTextFileOpen:=True;
end;

procedure TCustomFileExporter.CloseTextFile;
begin
  CloseFile(FTextFile);
  FTextFileOpen:=False;
  If FOpenedStream then
    CloseStream;
end;

procedure TCustomFileExporter.OpenStream;
begin
  CheckFileName;
  FStream:=TFileStream.Create(FFileName,fmCreate);
end;

Destructor TCustomFileExporter.Destroy;
begin
  If TextFileOpen then
    CloseTextFile;
  CloseStream;
  inherited Destroy;
end;


{ TCustomExportFormatSettings }

procedure TCustomExportFormatSettings.InitSettings;
begin
  FIntegerFormat:='%d';
  FHandleNullField:=True;
  FDateFormat:=ShortDateFormat;
  FTimeFormat:=ShortTimeFormat;
  FDateTimeFormat:=ShortDateFormat+' '+ShortTimeFormat;
  FBooleanTrue:='True';
  FBooleanFalse:='False';
  FDecimalSeparator:=sysutils.decimalseparator;
  FCurrencySymbol:=sysutils.CurrencyString;
end;

constructor TCustomExportFormatSettings.Create(DoInitSettings: Boolean);
begin
  If DoInitSettings then
    InitSettings;
end;

procedure TCustomExportFormatSettings.Assign(Source: TPersistent);

Var
  FS : TCustomExportFormatSettings;

begin
  If (Source is TCustomExportFormatSettings) then
    begin
    FS:=Source as TCustomExportFormatSettings;
    FHandleNullField:=FS.FHandleNullField;
    FBooleanFalse:=FS.FBooleanFalse;
    FBooleanTrue:=FS.FBooleanTrue;
    FCurrencyDigits:=FS.FCurrencyDigits;
    FCurrencySymbol:=FS.FCurrencySymbol;
    FDateFormat:=FS.FDateFormat;
    FIntegerFormat:=FS.IntegerFormat;
    FTimeFormat:=FS.FTimeFormat;
    FDateTimeFormat:=FS.FDateTimeFormat;
    FDecimalSeparator:=FS.FDecimalSeparator;
    FUseDisplayText:=FS.FUseDisplayText;
    end
  else
    inherited Assign(Source);
end;



{ TExportFormats }

function TExportFormats.GetFormat(Index : Integer): TExportFormatItem;
begin
  Result:=TExportFormatItem(Items[Index]);
end;

procedure TExportFormats.SetFormat(Index : Integer; const AValue: TExportFormatItem
  );
begin
  Items[Index]:=AValue;
end;

function TExportFormats.RegisterExportFormat(Const AName, ADescription,
  AExtensions: String; AClass: TCustomDatasetExporterClass): TExportFormatItem;
begin
  If (IndexOfFormat(AName)<>-1) then
    Raise EDataExporter.CreateFmt(SErrFormatExists,[AName]);
  Result:=Add as TExportFormatItem;
  Result.Name:=AName;
  Result.Description:=ADescription;
  Result.Extensions:=AExtensions;
  Result.ExportClass:=AClass;
end;

function TExportFormats.IndexOfFormat(const AName: String): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (CompareText(GetFormat(Result).Name,AName)<>0) do
    Dec(Result);
end;

function TExportFormats.IndexOfExportClass(AClass: TCustomDataSetExporterClass
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) and (GetFormat(Result).ExportClass<>AClass) do
    Dec(Result);
end;

function TExportFormats.FindFormat(const AName: String): TExportFormatItem;

Var
  I : Integer;
  
begin
  I:=IndexOfFormat(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetFormat(I);
end;

function TExportFormats.FindFormatByClass(AClass: TCustomDataSetExporterClass): TExportFormatItem;
  
Var
  I : Integer;

begin
  I:=IndexOfExportClass(AClass);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetFormat(I);
end;

function TExportFormats.ConfigureExport(AnExport: TCustomDatasetExporter
  ): Boolean;
  
Var
  F : TExportFormatItem;
  
begin
  Result:=True;
  F:=FindFormatByClass(TCustomDatasetExporterClass(AnExport.ClassType));
  If Assigned(F) and Assigned(F.OnConfigureDialog) then
    Result:=F.OnConfigureDialog(AnExport);
end;

function TExportFormats.FormatByName(const AName: String): TExportFormatItem;
begin
  Result:=FindFormat(AName);
  If (Result=Nil) then
    Raise EDataExporter.CreateFmt(SUnknownExportFormat,[AName]);
end;

function TExportFormats.ConstructFilter(AnExport: TCustomDatasetExporter
  ): String;
  
  Procedure AddToResult(S : String);
  
  begin
    If (Result<>'') and (S<>'') then
      Result:=Result+'|';
    Result:=Result+S;
  end;
  
Var
  F : TExportFormatItem;
  P : Integer;
  S,E : String;
  
begin
  Result:='';
  F:=FindFormatByClass(TCustomDatasetExporterClass(AnExport.ClassType));
  If (F=Nil) then
    Exit;
  S:=F.Extensions;
  While (S<>'') do
    begin
    P:=Pos(';',S);
    If (P=0) then
      P:=Length(S)+1;
    E:=Copy(S,1,P-1);
    If (Length(E)>1) then // Make sure there actually is an extension
      begin
      If (E[1]='.') then
        system.Delete(E,1,1);
      AddToResult(Format(SExportFilter,[E])+Format('|*.%s',[E]));
      end;
    system.Delete(S,1,P);
    end;
  AddToResult(SAllFilesFilter+'|*');
end;


Procedure TExportFormats.UnRegisterExportFormat(AClass : TCustomDatasetExporterClass);

begin
  FindFormatByClass(AClass).Free;
end;

Procedure TExportFormats.UnRegisterExportFormat(Const AName : String);

begin
  FindFormat(AName).Free;
end;


{ TExportFormatItem }

procedure TExportFormatItem.SetName(const AValue: String);

Var
  I : TExportFormatItem;

begin
  if (FName=AValue) then
    exit;
  If (AValue<>'') then
    begin
    I:=TExportFormats(Collection).FindFormat(AValue);
    If (I<>Nil) and (I<>Self) then
      Raise EDataExporter.CreateFmt(SErrFormatExists,[AValue]);
    end;
  FName:=AValue;
end;

Var
  EF : TExportFormats;

Procedure InitExportFormats;

begin
  EF:=TExportFormats.Create(TExportFormatItem);
end;

Procedure DoneExportFormats;

begin
  FreeAndNil(EF);
end;

Function ExportFormats : TExportFormats;

begin
  If (EF=Nil) then
    InitExportFormats;
  Result:=EF;
end;

Function RegisterExportFormat(Const AName,ADescription,AExtensions : String; AClass : TCustomDatasetExporterClass) : TExportFormatItem;

begin
  Result:=ExportFormats.RegisterExportFormat(AName,ADescription,AExtensions,AClass);
end;

Procedure UnRegisterExportFormat(AClass : TCustomDatasetExporterClass);

begin
  ExportFormats.UnregisterExportFormat(AClass);
end;

Procedure UnRegisterExportFormat(Const AName : String);

begin
  ExportFormats.UnregisterExportFormat(AName);
end;

Initialization

Finalization
  DoneExportFormats;
end.


