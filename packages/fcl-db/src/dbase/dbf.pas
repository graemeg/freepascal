unit dbf;

{ design info in dbf_reg.pas }

interface

{$I dbf_common.inc}

uses
  Classes,
  Db,
  dbf_common,
  dbf_dbffile,
  dbf_parser,
  dbf_prsdef,
  dbf_cursor,
  dbf_fields,
  dbf_pgfile,
  dbf_idxfile;
{$ifndef fpc}
// If you got a compilation error here or asking for dsgnintf.pas, then just add
// this file in your project:
// dsgnintf.pas in 'C: \Program Files\Borland\Delphi5\Source\Toolsapi\dsgnintf.pas'
{$endif}

type

//====================================================================
  pBookmarkData = ^TBookmarkData;
  TBookmarkData = record
    PhysicalRecNo: Integer;
  end;

  pDbfRecord = ^TDbfRecordHeader;
  TDbfRecordHeader = record
    BookmarkData: TBookmarkData;
    BookmarkFlag: TBookmarkFlag;
    SequentialRecNo: Integer;
    DeletedFlag: Char;
  end;
//====================================================================
  TDbf = class;
//====================================================================
  TDbfStorage = (stoMemory,stoFile);
  TDbfOpenMode = (omNormal,omAutoCreate,omTemporary);
  TDbfLanguageAction = (laReadOnly, laForceOEM, laForceANSI, laDefault);
  TDbfTranslationMode = (tmNoneAvailable, tmNoneNeeded, tmSimple, tmAdvanced);
  TDbfFileName = (dfDbf, dfMemo, dfIndex);
//====================================================================
  TDbfFileNames = set of TDbfFileName;
//====================================================================
  TCompareRecordEvent = procedure(Dbf: TDbf; var Accept: Boolean) of object;
  TTranslateEvent = function(Dbf: TDbf; Src, Dest: PChar; ToOem: Boolean): Integer of object;
  TLanguageWarningEvent = procedure(Dbf: TDbf; var Action: TDbfLanguageAction) of object;
  TConvertFieldEvent = procedure(Dbf: TDbf; DstField, SrcField: TField) of object;
  TBeforeAutoCreateEvent = procedure(Dbf: TDbf; var DoCreate: Boolean) of object;
//====================================================================
  // TDbfBlobStream keeps a reference count to number of references to
  // this instance. Only if FRefCount reaches zero, then the object will be
  // destructed. AddReference `clones' a reference.
  // This allows the VCL to use Free on the object to `free' that
  // particular reference.

  TDbfBlobStream = class(TMemoryStream)
  private
    FBlobField: TBlobField;
    FMode: TBlobStreamMode;
    FDirty: boolean;            { has possibly modified data, needs to be written }
    FMemoRecNo: Integer;
        { -1 : invalid contents }
        {  0 : clear, no contents }
        { >0 : data from page x }
    FReadSize: Integer;
    FRefCount: Integer;

    function  GetTransliterate: Boolean;
    procedure Translate(ToOem: Boolean);
    procedure SetMode(NewMode: TBlobStreamMode);
  public
    constructor Create(FieldVal: TField);
    destructor Destroy; override;

    function  AddReference: TDbfBlobStream;
    procedure FreeInstance; override;

    procedure Cancel;
    procedure Commit;

    property Dirty: boolean read FDirty;
    property Transliterate: Boolean read GetTransliterate;
    property MemoRecNo: Integer read FMemoRecNo write FMemoRecNo;
    property ReadSize: Integer read FReadSize write FReadSize;
    property Mode: TBlobStreamMode write SetMode;
    property BlobField: TBlobField read FBlobField;
  end;
//====================================================================
  TDbfIndexDefs = class(TCollection)
  public
    FOwner: TDbf;
   private
    function GetItem(N: Integer): TDbfIndexDef;
    procedure SetItem(N: Integer; Value: TDbfIndexDef);
   protected
    function GetOwner: TPersistent; override;
   public
    constructor Create(AOwner: TDbf);

    function  Add: TDbfIndexDef;
    function  GetIndexByName(const Name: string): TDbfIndexDef;
    function  GetIndexByField(const Name: string): TDbfIndexDef;
    procedure Update; {$ifdef SUPPORT_REINTRODUCE} reintroduce; {$endif}

    property Items[N: Integer]: TDbfIndexDef read GetItem write SetItem; default;
  end;
//====================================================================
  TDbfMasterLink = class(TDataLink)
  private
    FDetailDataSet: TDbf;
    FParser: TDbfParser;
    FFieldNames: string;
    FValidExpression: Boolean;
    FOnMasterChange: TNotifyEvent;
    FOnMasterDisable: TNotifyEvent;

    function GetFieldsVal: TRecordBuffer;

    procedure SetFieldNames(const Value: string);
  protected
    procedure ActiveChanged; override;
    procedure CheckBrowseMode; override;
    procedure LayoutChanged; override;
    procedure RecordChanged(Field: TField); override;

  public
    constructor Create(ADataSet: TDbf);
    destructor Destroy; override;

    property FieldNames: string read FFieldNames write SetFieldNames;
    property ValidExpression: Boolean read FValidExpression write FValidExpression;
    property FieldsVal: TRecordBuffer read GetFieldsVal;
    property Parser: TDbfParser read FParser;

    property OnMasterChange: TNotifyEvent read FOnMasterChange write FOnMasterChange;
    property OnMasterDisable: TNotifyEvent read FOnMasterDisable write FOnMasterDisable;
  end;
//====================================================================
  PDbfBlobList = ^TDbfBlobList;
  TDbfBlobList = array[0..MaxListSize-1] of TDbfBlobStream;
//====================================================================
  TDbf = class(TDataSet)
  private
    FDbfFile: TDbfFile;
    FCursor: TVirtualCursor;
    FOpenMode: TDbfOpenMode;
    FStorage: TDbfStorage;
    FMasterLink: TDbfMasterLink;
    FParser: TDbfParser;
    FBlobStreams: PDbfBlobList;
    FUserIndexStream: TStream;
    FUserStream: TStream;  // user stream to open
    FUserMemoStream: TStream; // user-provided/expected stream backing memo file storage
    FTableName: string;    // table path and file name
    FRelativePath: string;
    FAbsolutePath: string;
    FIndexName: string;
    FReadOnly: Boolean;
    FFilterBuffer: TRecordBuffer;
    FTempBuffer: TRecordBuffer;
    FEditingRecNo: Integer;
{$ifdef SUPPORT_VARIANTS}    
    FLocateRecNo: Integer;
{$endif}
    FBackLink: String;
    FLanguageID: Byte;
    FTableLevel: Integer;
    FExclusive: Boolean;
    FShowDeleted: Boolean;
    FPosting: Boolean;
    FDisableResyncOnPost: Boolean;
    FTempExclusive: Boolean;
    FInCopyFrom: Boolean;
    FStoreDefs: Boolean;
    FCopyDateTimeAsString: Boolean;
    FFindRecordFilter: Boolean;
    FIndexFile: TIndexFile;
    FDateTimeHandling: TDateTimeHandling;
    FTranslationMode: TDbfTranslationMode;
    FIndexDefs: TDbfIndexDefs;
    FBeforeAutoCreate: TBeforeAutoCreateEvent;
    FOnTranslate: TTranslateEvent;
    FOnLanguageWarning: TLanguageWarningEvent;
    FOnLocaleError: TDbfLocaleErrorEvent;
    FOnIndexMissing: TDbfIndexMissingEvent;
    FOnCompareRecord: TNotifyEvent;
    FOnCopyDateTimeAsString: TConvertFieldEvent;

    function GetIndexName: string;
    function GetVersion: string;
    function GetPhysicalRecNo: Integer;
    function GetLanguageStr: string;
    function GetCodePage: Cardinal;
    function GetExactRecordCount: Integer;
    function GetPhysicalRecordCount: Integer;
    function GetKeySize: Integer;
    function GetMasterFields: string;
    function FieldDefsStored: Boolean;
    procedure SetBackLink(NewBackLink: String);

    procedure SetIndexName(AIndexName: string);
    procedure SetDbfIndexDefs(const Value: TDbfIndexDefs);
    procedure SetFilePath(const Value: string);
    procedure SetTableName(const S: string);
    procedure SetVersion(const S: string);
    procedure SetLanguageID(NewID: Byte);
    procedure SetDataSource(Value: TDataSource);
    procedure SetMasterFields(const Value: string);
    procedure SetTableLevel(const NewLevel: Integer);
    procedure SetPhysicalRecNo(const NewRecNo: Integer);

    procedure MasterChanged(Sender: TObject);
    procedure MasterDisabled(Sender: TObject);
    procedure DetermineTranslationMode;
    procedure UpdateRange;
    procedure SetShowDeleted(Value: Boolean);
    procedure GetFieldDefsFromDbfFieldDefs;
    procedure InitDbfFile(FileOpenMode: TPagedFileMode);
    function  ParseIndexName(const AIndexName: string): string;
    procedure ParseFilter(const AFilter: string);
    function  GetDbfFieldDefs: TDbfFieldDefs;
<<<<<<< HEAD
<<<<<<< HEAD
    function  ReadCurrentRecord(Buffer: TRecordBuffer; var Acceptable: Boolean): TGetResult;
=======
    function  ReadCurrentRecord(Buffer: PChar; var Acceptable: Boolean): TGetResult;
>>>>>>> graemeg/fixes_2_2
=======
    function  ReadCurrentRecord(Buffer: PChar; var Acceptable: Boolean): TGetResult;
>>>>>>> origin/fixes_2_2
    function  SearchKeyBuffer(Buffer: PChar; SearchType: TSearchKeyType): Boolean;
    procedure SetRangeBuffer(LowRange: PChar; HighRange: PChar);

  protected
    { abstract methods }
    function  AllocRecordBuffer: TRecordBuffer; override; {virtual abstract}
    procedure ClearCalcFields(Buffer: TRecordBuffer); override;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); override; {virtual abstract}
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override; {virtual abstract}
    function  GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; override; {virtual abstract}
    function  GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; override; {virtual abstract}
    function  GetRecordSize: Word; override; {virtual abstract}
    procedure InternalAddRecord(Buffer: Pointer; AAppend: Boolean); override; {virtual abstract}
    procedure InternalClose; override; {virtual abstract}
    procedure InternalDelete; override; {virtual abstract}
    procedure InternalFirst; override; {virtual abstract}
    procedure InternalGotoBookmark(ABookmark: Pointer); override; {virtual abstract}
    procedure InternalHandleException; override; {virtual abstract}
    procedure InternalInitFieldDefs; override; {virtual abstract}
    procedure InternalInitRecord(Buffer: TRecordBuffer); override; {virtual abstract}
    procedure InternalLast; override; {virtual abstract}
    procedure InternalOpen; override; {virtual abstract}
    procedure InternalEdit; override; {virtual}
    procedure InternalCancel; override; {virtual}
{$ifndef FPC}
{$ifndef DELPHI_3}
    procedure InternalInsert; override; {virtual}
{$endif}
{$endif}
    procedure InternalPost; override; {virtual abstract}
    procedure InternalSetToRecord(Buffer: TRecordBuffer); override; {virtual abstract}
    procedure InitFieldDefs; override;
    function  IsCursorOpen: Boolean; override; {virtual abstract}
<<<<<<< HEAD
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); override; {virtual abstract}
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); override; {virtual abstract}
=======
    procedure SetBookmarkFlag(Buffer: PChar; Value: TBookmarkFlag); override; {virtual abstract}
    procedure SetBookmarkData(Buffer: PChar; Data: Pointer); override; {virtual abstract}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SetFieldData(Field: TField; Buffer: Pointer);
      {$ifdef SUPPORT_OVERLOAD} overload; {$endif} override; {virtual abstract}

    { virtual methods (mostly optional) }
    function  GetDataSource: TDataSource; {$ifndef VER1_0}override;{$endif}
    function  GetRecordCount: Integer; override; {virtual}
    function  GetRecNo: Integer; override; {virtual}
    function  GetCanModify: Boolean; override; {virtual}
    procedure SetRecNo(Value: Integer); override; {virual}
    procedure SetFiltered(Value: Boolean); override; {virtual;}
    procedure SetFilterText(const Value: String); override; {virtual;}
{$ifdef SUPPORT_DEFCHANGED}
    procedure DefChanged(Sender: TObject); override;
{$endif}
    function  FindRecord(Restart, GoForward: Boolean): Boolean; override;

    function  GetIndexFieldNames: string; {virtual;}
    procedure SetIndexFieldNames(const Value: string); {virtual;}

{$ifdef SUPPORT_VARIANTS}
    function  LocateRecordLinear(const KeyFields: String; const KeyValues: Variant; Options: TLocateOptions): Boolean;
    function  LocateRecordIndex(const KeyFields: String; const KeyValues: Variant; Options: TLocateOptions): Boolean;
    function  LocateRecord(const KeyFields: String; const KeyValues: Variant; Options: TLocateOptions): Boolean;
{$endif}

    procedure DoFilterRecord(var Acceptable: Boolean);
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;

    { abstract methods }
    function GetFieldData(Field: TField; Buffer: Pointer): Boolean;
      {$ifdef SUPPORT_OVERLOAD} overload; {$endif} override; {virtual abstract}
    { virtual methods (mostly optional) }
    procedure Resync(Mode: TResyncMode); override;
    function CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; override; {virtual}
{$ifdef SUPPORT_NEW_TRANSLATE}
    function Translate(Src, Dest: PChar; ToOem: Boolean): Integer; override; {virtual}
{$else}
    procedure Translate(Src, Dest: PChar; ToOem: Boolean); override; {virtual}
{$endif}

{$ifdef SUPPORT_OVERLOAD}
    function  GetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean): Boolean;
      {$ifdef SUPPORT_BACKWARD_FIELDDATA} overload; override; {$else} reintroduce; overload; {$endif}
    procedure SetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean);
      {$ifdef SUPPORT_BACKWARD_FIELDDATA} overload; override; {$else} reintroduce; overload; {$endif}
{$endif}

    function CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Integer; override;
    procedure CheckDbfFieldDefs(ADbfFieldDefs: TDbfFieldDefs);

    function  FindFirst: Boolean; override;
    function  FindLast: Boolean; override;
    function  FindNext: Boolean; override;
    function  FindPrior: Boolean; override;

{$ifdef VER1_0}
    procedure DataEvent(Event: TDataEvent; Info: Longint); override;
{$endif}

    // my own methods and properties
    // most look like ttable functions but they are not tdataset related
    // I (try to) use the same syntax to facilitate the conversion between bde and TDbf

    // index support (use same syntax as ttable but is not related)
{$ifdef SUPPORT_DEFAULT_PARAMS}
    procedure AddIndex(const AIndexName, AFields: String; Options: TIndexOptions; const DescFields: String='');
{$else}
    procedure AddIndex(const AIndexName, AFields: String; Options: TIndexOptions);
{$endif}
    procedure RegenerateIndexes;

    procedure CancelRange;
    procedure CheckMasterRange;
{$ifdef SUPPORT_VARIANTS}
    function  SearchKey(Key: Variant; SearchType: TSearchKeyType; KeyIsANSI: boolean
      {$ifdef SUPPORT_DEFAULT_PARAMS}= false{$endif}): Boolean;
    procedure SetRange(LowRange: Variant; HighRange: Variant; KeyIsANSI: boolean
      {$ifdef SUPPORT_DEFAULT_PARAMS}= false{$endif});
{$endif}
    function  PrepareKey(Buffer: Pointer; BufferType: TExpressionType): PChar;
    function  SearchKeyPChar(Key: PChar; SearchType: TSearchKeyType; KeyIsANSI: boolean
      {$ifdef SUPPORT_DEFAULT_PARAMS}= false{$endif}): Boolean;
    procedure SetRangePChar(LowRange: PChar; HighRange: PChar; KeyIsANSI: boolean
      {$ifdef SUPPORT_DEFAULT_PARAMS}= false{$endif});
    function  GetCurrentBuffer: TRecordBuffer;
    procedure ExtractKey(KeyBuffer: PChar);
    procedure UpdateIndexDefs; override;
    procedure GetFileNames(Strings: TStrings; Files: TDbfFileNames); {$ifdef SUPPORT_DEFAULT_PARAMS} overload; {$endif}
{$ifdef SUPPORT_DEFAULT_PARAMS}
    function  GetFileNames(Files: TDbfFileNames  = [dfDbf]  ): string; overload;
{$else}
    function  GetFileNamesString(Files: TDbfFileNames (* = [dfDbf] *) ): string;
{$endif}
    procedure GetIndexNames(Strings: TStrings);
    procedure GetAllIndexFiles(Strings: TStrings);

    procedure TryExclusive;
    procedure EndExclusive;
    function  LockTable(const Wait: Boolean): Boolean;
    procedure UnlockTable;
    procedure OpenIndexFile(IndexFile: string);
    procedure DeleteIndex(const AIndexName: string);
    procedure CloseIndexFile(const AIndexName: string);
    procedure RepageIndexFile(const AIndexFile: string);
    procedure CompactIndexFile(const AIndexFile: string);

{$ifdef SUPPORT_VARIANTS}
    function  Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant; override;
    function  Locate(const KeyFields: string; const KeyValues: Variant; Options: TLocateOptions): Boolean; override;
{$endif}

    function  IsDeleted: Boolean;
    procedure Undelete;
    // Call this after setting up fielddefs in order to store the definitions into a table
    procedure CreateTable;
    procedure CreateTableEx(ADbfFieldDefs: TDbfFieldDefs);
    procedure CopyFrom(DataSet: TDataSet; FileName: string; DateTimeAsString: Boolean; Level: Integer);
    procedure RestructureTable(ADbfFieldDefs: TDbfFieldDefs; Pack: Boolean);
    procedure PackTable;
    procedure EmptyTable;
    procedure Zap;

{$ifndef SUPPORT_INITDEFSFROMFIELDS}
    procedure InitFieldDefsFromFields;
{$endif}

    property AbsolutePath: string read FAbsolutePath;
    property DbfFieldDefs: TDbfFieldDefs read GetDbfFieldDefs;
    property PhysicalRecNo: Integer read GetPhysicalRecNo write SetPhysicalRecNo;
    // Visual Foxpro: relative path to .dbc database file containing
    // long field names and other metadata
    // Empty if this is a "free table", not linked to a .dbc file
    // Setting this with a FoxPro tablelevel will auto-upgrade to Visual Foxpro
    // Unsupported for other versions
    property BackLink: String read FBackLink write SetBackLink;
    property LanguageID: Byte read FLanguageID write SetLanguageID;
    property LanguageStr: String read GetLanguageStr;
    property CodePage: Cardinal read GetCodePage;
    property ExactRecordCount: Integer read GetExactRecordCount;
    property PhysicalRecordCount: Integer read GetPhysicalRecordCount;
    property KeySize: Integer read GetKeySize;
    property DbfFile: TDbfFile read FDbfFile;
    // Storage for data file if using memory storage
    property UserStream: TStream read FUserStream write FUserStream;
    // Storage for index file - if any - when using memory storage
    property UserIndexStream: TStream read FUserIndexStream write FUserIndexStream;
    // Storage for memo file - if any - when using memory storage
    property UserMemoStream: TStream read FUserMemoStream write FUserMemoStream;
    property DisableResyncOnPost: Boolean read FDisableResyncOnPost write FDisableResyncOnPost;
  published
    property DateTimeHandling: TDateTimeHandling
             read FDateTimeHandling write FDateTimeHandling default dtBDETimeStamp;
    property Exclusive: Boolean read FExclusive write FExclusive default false;
    property FilePath: string     read FRelativePath write SetFilePath;
    property FilePathFull: string read FAbsolutePath write SetFilePath stored false;
    property Indexes: TDbfIndexDefs read FIndexDefs write SetDbfIndexDefs stored false;
    property IndexDefs: TDbfIndexDefs read FIndexDefs write SetDbfIndexDefs;
    property IndexFieldNames: string read GetIndexFieldNames write SetIndexFieldNames stored false;
    property IndexName: string read GetIndexName write SetIndexName;
    property MasterFields: string read GetMasterFields write SetMasterFields;
    property MasterSource: TDataSource read GetDataSource write SetDataSource;
    property OpenMode: TDbfOpenMode read FOpenMode write FOpenMode default omNormal;
    property ReadOnly: Boolean read FReadOnly write FReadonly default false;
    property ShowDeleted: Boolean read FShowDeleted write SetShowDeleted default false;
    property Storage: TDbfStorage read FStorage write FStorage default stoFile;
    property StoreDefs: Boolean read FStoreDefs write FStoreDefs default False;
    property TableName: string read FTableName write SetTableName;
    property TableLevel: Integer read FTableLevel write SetTableLevel;
    property Version: string read GetVersion write SetVersion stored false;
    property BeforeAutoCreate: TBeforeAutoCreateEvent read FBeforeAutoCreate write FBeforeAutoCreate;
    property OnCompareRecord: TNotifyEvent read FOnCompareRecord write FOnCompareRecord;
    property OnLanguageWarning: TLanguageWarningEvent read FOnLanguageWarning write FOnLanguageWarning;
    property OnLocaleError: TDbfLocaleErrorEvent read FOnLocaleError write FOnLocaleError;
    property OnIndexMissing: TDbfIndexMissingEvent read FOnIndexMissing write FOnIndexMissing;
    property OnCopyDateTimeAsString: TConvertFieldEvent read FOnCopyDateTimeAsString write FOnCopyDateTimeAsString;
    property OnTranslate: TTranslateEvent read FOnTranslate write FOnTranslate;

    // redeclared data set properties
    property Active;
    property FieldDefs stored FieldDefsStored;
    property Filter;
    property Filtered;
    property FilterOptions;
    property BeforeOpen;
    property AfterOpen;
    property BeforeClose;
    property AfterClose;
    property BeforeInsert;
    property AfterInsert;
    property BeforeEdit;
    property AfterEdit;
    property BeforePost;
    property AfterPost;
    property BeforeCancel;
    property AfterCancel;
    property BeforeDelete;
    property AfterDelete;
{$ifdef SUPPORT_REFRESHEVENTS}    
    property BeforeRefresh;
    property AfterRefresh;
{$endif}    
    property BeforeScroll;
    property AfterScroll;
    property OnCalcFields;
    property OnDeleteError;
    property OnEditError;
    property OnFilterRecord;
    property OnNewRecord;
    property OnPostError;
  end;

  TDbf_GetBasePathFunction = function: string;

var
  DbfBasePath: TDbf_GetBasePathFunction;

implementation

uses
  SysUtils,
{$ifndef FPC}
  DBConsts,
{$endif}
{$ifdef WINDOWS}
  Windows,
{$else}
{$ifdef KYLIX}
  Libc,
{$endif}  
  Types,
  dbf_wtil,
{$endif}
{$ifdef SUPPORT_SEPARATE_VARIANTS_UNIT}
  Variants,
{$endif}
  dbf_idxcur,
  dbf_memo,
  dbf_str;

{$ifdef FPC}
const
  // TODO: move these to DBConsts
  SNotEditing = 'Dataset not in edit or insert mode';
  SCircularDataLink = 'Circular datalinks are not allowed';
{$endif}

function TableLevelToDbfVersion(TableLevel: integer): TXBaseVersion;
begin
  case TableLevel of
    3:                            Result := xBaseIII;
    7:                            Result := xBaseVII;
    TDBF_TABLELEVEL_FOXPRO:       Result := xFoxPro;
    TDBF_TABLELEVEL_VISUALFOXPRO: Result := xVisualFoxPro;
  else
    {4:} Result := xBaseIV;
  end;
end;

//==========================================================
//============ TDbfBlobStream
//==========================================================
constructor TDbfBlobStream.Create(FieldVal: TField);
begin
  FBlobField := FieldVal as TBlobField;
  FReadSize := 0;
  FMemoRecNo := 0;
  FRefCount := 1;
  FDirty := false;
end;

destructor TDbfBlobStream.Destroy;
begin
  // only continue destroy if all references released
  if FRefCount = 1 then
  begin
    // this is the last reference
    inherited
  end else begin
    // fire event when dirty, and the last "user" is freeing it's reference
    // tdbf always has the last reference
    if FDirty and (FRefCount = 2) then
    begin
      // a second referer to instance has changed the data, remember modified
//      TDbf(FBlobField.DataSet).SetModified(true);
      // is following better? seems to provide notification for user (from VCL)
      if not (FBlobField.DataSet.State in [dsCalcFields, dsFilter, dsNewValue]) then
        TDbf(FBlobField.DataSet).DataEvent(deFieldChange, PtrInt(FBlobField));
    end;
  end;
  Dec(FRefCount);
end;

procedure TDbfBlobStream.FreeInstance;
begin
  // only continue freeing if all references released
  if FRefCount = 0 then
    inherited;
end;

procedure TDbfBlobStream.SetMode(NewMode: TBlobStreamMode);
begin
  FMode := NewMode;
  FDirty := FDirty or (NewMode = bmWrite) or (NewMode = bmReadWrite);
end;

procedure TDbfBlobStream.Cancel;
begin
  FDirty := false;
  FMemoRecNo := -1;
end;

procedure TDbfBlobStream.Commit;
var
  Dbf: TDbf;
begin
  if FDirty then
  begin
    Size := Position; // Strange but it leaves tailing trash bytes if I do not write that.
    Dbf := TDbf(FBlobField.DataSet);
    Translate(true);
    Dbf.FDbfFile.MemoFile.WriteMemo(FMemoRecNo, FReadSize, Self);
    Dbf.FDbfFile.SetFieldData(FBlobField.FieldNo-1, ftInteger, @FMemoRecNo,
      @pDbfRecord(TDbf(FBlobField.DataSet).ActiveBuffer)^.DeletedFlag, false);
    FDirty := false;
  end;
end;

function TDbfBlobStream.AddReference: TDbfBlobStream;
begin
  Inc(FRefCount);
  Result := Self;
end;

function TDbfBlobStream.GetTransliterate: Boolean;
begin
  Result := FBlobField.Transliterate;
end;

procedure TDbfBlobStream.Translate(ToOem: Boolean);
var
  bytesToDo, numBytes: Integer;
  bufPos: PChar;
  saveChar: Char;
begin
  if (Transliterate) and (Size > 0) then
  begin
    // get number of bytes to be translated
    bytesToDo := Size;
    // make space for final null-terminator
    Size := Size + 1;
    bufPos := Memory;
    repeat
      // process blocks of 512 bytes
      numBytes := bytesToDo;
      if numBytes > 512 then
        numBytes := 512;
      // null-terminate memory
      saveChar := bufPos[numBytes];
      bufPos[numBytes] := #0;
      // translate memory
      TDbf(FBlobField.DataSet).Translate(bufPos, bufPos, ToOem);
      // restore char
      bufPos[numBytes] := saveChar;
      // numBytes bytes translated
      Dec(bytesToDo, numBytes);
      Inc(bufPos, numBytes);
    until bytesToDo = 0;
    // cut ending null-terminator
    Size := Size - 1;
  end;
end;

//====================================================================
// TDbf = TDataset Descendant.
//====================================================================
constructor TDbf.Create(AOwner: TComponent); {override;}
begin
  inherited;

  if DbfGlobals = nil then
    DbfGlobals := TDbfGlobals.Create;

  BookmarkSize := sizeof(TBookmarkData);
  FIndexDefs := TDbfIndexDefs.Create(Self);
  FMasterLink := TDbfMasterLink.Create(Self);
  FMasterLink.OnMasterChange := MasterChanged;
  FMasterLink.OnMasterDisable := MasterDisabled;
  FDateTimeHandling := dtBDETimeStamp;
  FStorage := stoFile;
  FOpenMode := omNormal;
  FParser := nil;
  FPosting := false;
  FReadOnly := false;
  FExclusive := false;
  FDisableResyncOnPost := false;
  FTempExclusive := false;
  FCopyDateTimeAsString := false;
  FInCopyFrom := false;
  FFindRecordFilter := false;
  FEditingRecNo := -1;
  FTableLevel := 4;
  FIndexName := EmptyStr;
  FilePath := EmptyStr;
  FTempBuffer := nil;
  FFilterBuffer := nil;
  FIndexFile := nil;
  FOnTranslate := nil;
  FOnCopyDateTimeAsString := nil;
end;

destructor TDbf.Destroy; {override;}
var
  I: Integer;
begin
  inherited Destroy;

  if FIndexDefs <> nil then
  begin
    for I := FIndexDefs.Count - 1 downto 0 do
      TDbfIndexDef(FIndexDefs.Items[I]).Free;
    FIndexDefs.Free;
  end;
  FMasterLink.Free;
end;

function TDbf.AllocRecordBuffer: TRecordBuffer; {override virtual abstract from TDataset}
begin
  GetMem(Result, SizeOf(TDbfRecordHeader)+FDbfFile.RecordSize+CalcFieldsSize+1);
end;

procedure TDbf.FreeRecordBuffer(var Buffer: TRecordBuffer); {override virtual abstract from TDataset}
begin
  FreeMemAndNil(Pointer(Buffer));
end;

procedure TDbf.GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); {override virtual abstract from TDataset}
begin
  pBookmarkData(Data)^ := pDbfRecord(Buffer)^.BookmarkData;
end;

function TDbf.GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; {override virtual abstract from TDataset}
begin
  Result := pDbfRecord(Buffer)^.BookmarkFlag;
end;

function TDbf.GetCurrentBuffer: TRecordBuffer;
begin
  case State of
    dsFilter:     Result := FFilterBuffer;
    dsCalcFields: Result := CalcBuffer;
//    dsSetKey:     Result := FKeyBuffer;     // TO BE Implemented
  else
    if IsEmpty then
    begin
      Result := nil;
    end else begin
      Result := ActiveBuffer;
    end;
  end;
  if Result <> nil then
    Result := @PDbfRecord(Result)^.DeletedFlag;
end;

// we don't want converted data formats, we want native :-)
// it makes coding easier in TDbfFile.GetFieldData
//  ftCurrency:
//    Delphi 3,4: BCD array
//  ftBCD:
// ftDateTime is more difficult though

function TDbf.GetFieldData(Field: TField; Buffer: Pointer): Boolean; {override virtual abstract from TDataset}
{$ifdef SUPPORT_OVERLOAD}
begin
  { calling through 'old' delphi 3 interface, use compatible/'native' format }
  Result := GetFieldData(Field, Buffer, true);
end;

function TDbf.GetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean): Boolean; {overload; override;}
{$else}
const
  { no overload => delphi 3 => use compatible/'native' format }
  NativeFormat = true;
{$endif}
var
  Src: TRecordBuffer;
begin
  Src := GetCurrentBuffer;
  if Src = nil then
  begin
    Result := false;
    exit;
  end;

  if Field.FieldNo>0 then
  begin
    Result := FDbfFile.GetFieldData(Field.FieldNo-1, Field.DataType, Src, Buffer, NativeFormat);
  end else begin { weird calculated fields voodoo (from dbtables).... }
    Inc(PChar(Src), Field.Offset + GetRecordSize);
    Result := Boolean(Src[0]);
    if Result and (Buffer <> nil) then
      Move(Src[1], Buffer^, Field.DataSize);
  end;
end;

procedure TDbf.SetFieldData(Field: TField; Buffer: Pointer); {override virtual abstract from TDataset}
{$ifdef SUPPORT_OVERLOAD}
begin
  { calling through 'old' delphi 3 interface, use compatible/'native' format }
  SetFieldData(Field, Buffer, true);
end;

function TDbf.FindFirst: Boolean;
begin
  // Use inherited function; if failed use FindRecord
  Result:=inherited FindFirst or FindRecord(True, True);
end;

function TDbf.FindLast: Boolean;
begin
  // Use inherited function; if failed use FindRecord
  Result:=inherited FindLast or FindRecord(True, False);
end;

function TDbf.FindNext: Boolean;
begin
  // Use inherited function; if failed use FindRecord
  Result:=inherited FindNext or FindRecord(False, True);
end;

function TDbf.FindPrior: Boolean;
begin
  // Use inherited function; if failed use FindRecord
  Result:=inherited FindPrior or FindRecord(False, False);
end;

procedure TDbf.SetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean); {overload; override;}
{$else}
const
  { no overload => delphi 3 => use compatible/'native' format }
  NativeFormat = true;
{$endif}
var
  Dst: PChar;
begin
  if (Field.FieldNo >= 0) then
  begin
    if State in [dsEdit, dsInsert, dsNewValue] then
      Field.Validate(Buffer);
    Dst := @PDbfRecord(ActiveBuffer)^.DeletedFlag;
    FDbfFile.SetFieldData(Field.FieldNo - 1, Field.DataType, Buffer, Dst, NativeFormat);
  end else begin    { ***** fkCalculated, fkLookup ***** }
    Dst := @PDbfRecord(CalcBuffer)^.DeletedFlag;
    Inc(PChar(Dst), RecordSize + Field.Offset);
    Boolean(Dst[0]) := Buffer <> nil;
    if Buffer <> nil then
      Move(Buffer^, Dst[1], Field.DataSize)
  end;     { end of ***** fkCalculated, fkLookup ***** }
  if not (State in [dsCalcFields, dsFilter, dsNewValue]) then begin
    DataEvent(deFieldChange, PtrInt(Field));
  end;
end;

procedure TDbf.DoFilterRecord(var Acceptable: Boolean);
begin
  // check filtertext
  if Length(Filter) > 0 then
  begin
{$ifndef VER1_0}
    Acceptable := Boolean((FParser.ExtractFromBuffer(GetCurrentBuffer))^);
{$else}
    // strange problem
    // dbf.pas(716,19) Error: Incompatible types: got "CHAR" expected "BOOLEAN"
    Acceptable := not ((FParser.ExtractFromBuffer(GetCurrentBuffer))^ = #0);
{$endif}
  end;

  // check user filter
  if Acceptable and Assigned(OnFilterRecord) then
    OnFilterRecord(Self, Acceptable);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDbf.ReadCurrentRecord(Buffer: TRecordBuffer; var Acceptable: Boolean): TGetResult;
=======
function TDbf.ReadCurrentRecord(Buffer: PChar; var Acceptable: Boolean): TGetResult;
>>>>>>> graemeg/fixes_2_2
=======
function TDbf.ReadCurrentRecord(Buffer: PChar; var Acceptable: Boolean): TGetResult;
>>>>>>> origin/fixes_2_2
var
  lPhysicalRecNo: Integer;
  pRecord: pDbfRecord;
begin
  lPhysicalRecNo := FCursor.PhysicalRecNo;
  if (lPhysicalRecNo = 0) or not FDbfFile.IsRecordPresent(lPhysicalRecNo) then
  begin
    Result := grError;
    Acceptable := false;
  end else begin
    Result := grOK;
    pRecord := pDbfRecord(Buffer);
    FDbfFile.ReadRecord(lPhysicalRecNo, @pRecord^.DeletedFlag);
    Acceptable := (FShowDeleted or (pRecord^.DeletedFlag <> '*'))
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TDbf.GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; {override virtual abstract from TDataset}
=======
=======
>>>>>>> origin/fixes_2_2
function TDbf.GetRecord(Buffer: PChar; GetMode: TGetMode; DoCheck: Boolean): TGetResult; {override virtual abstract from TDataset}
>>>>>>> graemeg/fixes_2_2
var
  pRecord: pDbfRecord;
  acceptable: Boolean;
  SaveState: TDataSetState;
//  s: string;
begin
  if FCursor = nil then
  begin
    Result := grEOF;
    exit;
  end;

  pRecord := pDbfRecord(Buffer);
  acceptable := false;
  repeat
    Result := grOK;
    case GetMode of
      gmNext :
        begin
          Acceptable := FCursor.Next;
          if Acceptable then begin
            Result := grOK;
          end else begin
            Result := grEOF
          end;
        end;
      gmPrior :
        begin
          Acceptable := FCursor.Prev;
          if Acceptable then begin
            Result := grOK;
          end else begin
            Result := grBOF
          end;
        end;
    end;

    if (Result = grOK) then
      Result := ReadCurrentRecord(Buffer, acceptable);

    if (Result = grOK) and acceptable then
    begin
      pRecord^.BookmarkData.PhysicalRecNo := FCursor.PhysicalRecNo;
      pRecord^.BookmarkFlag := bfCurrent;
      pRecord^.SequentialRecNo := FCursor.SequentialRecNo;
      GetCalcFields(Buffer);

      if Filtered or FFindRecordFilter then
      begin
        FFilterBuffer := Buffer;
        SaveState := SetTempState(dsFilter);
        DoFilterRecord(acceptable);
        RestoreState(SaveState);
      end;
    end;

    if (GetMode = gmCurrent) and not acceptable then
      Result := grError;
  until (Result <> grOK) or acceptable;

  if Result <> grOK then
    pRecord^.BookmarkData.PhysicalRecNo := -1;
end;

function TDbf.GetRecordSize: Word; {override virtual abstract from TDataset}
begin
  Result := FDbfFile.RecordSize;
end;

procedure TDbf.InternalAddRecord(Buffer: Pointer; AAppend: Boolean); {override virtual abstract from TDataset}
  // this function is called from TDataSet.InsertRecord and TDataSet.AppendRecord
  // goal: add record with Edit...Set Fields...Post all in one step
var
  pRecord: pDbfRecord;
  newRecord: integer;
begin
  // if InternalAddRecord is called, we know we are active
  pRecord := Buffer;

  // we can not insert records in DBF files, only append
  // ignore Append parameter
  newRecord := FDbfFile.Insert(@pRecord^.DeletedFlag);
  if newRecord > 0 then
    FCursor.PhysicalRecNo := newRecord;

  // set flag that TDataSet is about to post...so we can disable resync
  FPosting := true;
end;

procedure TDbf.InternalClose; {override virtual abstract from TDataset}
var
  lIndex: TDbfIndexDef;
  I: Integer;
begin
  // clear automatically added MDX index entries
  I := 0;
  while I < FIndexDefs.Count do
  begin
    // is this an MDX index?
    lIndex := FIndexDefs.Items[I];
    if (Length(ExtractFileExt(lIndex.IndexFile)) = 0) and
      TDbfIndexDef(FIndexDefs.Items[I]).Temporary then
    begin
{$ifdef SUPPORT_DEF_DELETE}
      // delete this entry
      FIndexDefs.Delete(I);
{$else}
      // does this work? I hope so :-)
      FIndexDefs.Items[I].Free;
{$endif}
    end else begin
      // NDX entry -> goto next
      Inc(I);
    end;
  end;

  // free blobs
  if FBlobStreams <> nil then
  begin
    for I := 0 to Pred(FieldDefs.Count) do
      FBlobStreams^[I].Free;
    FreeMemAndNil(Pointer(FBlobStreams));
  end;
  FreeRecordBuffer(FTempBuffer);
  // disconnect field objects
  BindFields(false);
  // Destroy field object (if not persistent)
  if DefaultFields then
    DestroyFields;

  if FParser <> nil then
    FreeAndNil(FParser);
  FreeAndNil(FCursor);
  if FDbfFile <> nil then
    FreeAndNil(FDbfFile);
end;

procedure TDbf.InternalCancel;
var
  I: Integer;
begin
  // cancel blobs
  for I := 0 to Pred(FieldDefs.Count) do
    if Assigned(FBlobStreams^[I]) then
      FBlobStreams^[I].Cancel;
  // if we have locked a record, unlock it
  if FEditingRecNo >= 0 then
  begin
    FDbfFile.UnlockPage(FEditingRecNo);
    FEditingRecNo := -1;
  end;
end;

procedure TDbf.InternalDelete; {override virtual abstract from TDataset}
var
  lRecord: pDbfRecord;
begin
  // start editing
  InternalEdit;
  SetState(dsEdit);
  // get record pointer
  lRecord := pDbfRecord(ActiveBuffer);
  // flag we deleted this record
  lRecord^.DeletedFlag := '*';
  // notify indexes this record is deleted
  FDbfFile.RecordDeleted(FEditingRecNo, @lRecord^.DeletedFlag);
  // done!
  InternalPost;
end;

procedure TDbf.InternalFirst; {override virtual abstract from TDataset}
begin
  FCursor.First;
end;

procedure TDbf.InternalGotoBookmark(ABookmark: Pointer); {override virtual abstract from TDataset}
begin
  with PBookmarkData(ABookmark)^ do
  begin
    if (PhysicalRecNo = 0) then begin
      First;
    end else
    if (PhysicalRecNo = MaxInt) then begin
      Last;
    end else begin
      if FCursor.PhysicalRecNo <> PhysicalRecNo then
        FCursor.PhysicalRecNo := PhysicalRecNo;
    end;
  end;
end;

procedure TDbf.InternalHandleException; {override virtual abstract from TDataset}
begin
  SysUtils.ShowException(ExceptObject, ExceptAddr);
end;

procedure TDbf.GetFieldDefsFromDbfFieldDefs;
var
  I: Integer;
  TempFieldDef: TDbfFieldDef;
  TempMdxFile: TIndexFile;
  lIndexName: string;
  lFieldDefCount: integer; //Counter for destination fielddefs

  procedure FixDuplicateNames;
  var
    BaseName: string;
    N: Integer;
  begin
    N := 1;
    BaseName := TempFieldDef.FieldName;
    while FieldDefs.IndexOf(TempFieldDef.FieldName)>=0 do
    begin
      Inc(N);
      TempFieldDef.FieldName:=BaseName+IntToStr(N);
    end;
  end;

begin
  FieldDefs.Clear;

  // get all fields
  lFieldDefCount:=-1; //will be fixed by first addition
  for I := 0 to FDbfFile.FieldDefs.Count - 1 do
  begin
    TempFieldDef := FDbfFile.FieldDefs.Items[I];
    // handle duplicate field names:
    FixDuplicateNames;
    // add field, passing dbase native size if relevant
    // TDbfFieldDef.Size indicates the number of bytes in the physical dbase file
    // TFieldDef.Size is only meant to store size indicator for variable length fields
    case TempFieldDef.FieldType of
      ftString, ftBytes, ftVarBytes: FieldDefs.Add(TempFieldDef.FieldName, TempFieldDef.FieldType, TempFieldDef.Size, false);
      ftBCD:
        begin
          FieldDefs.Add(TempFieldDef.FieldName, TempFieldDef.FieldType, 0, false);;;
        end;
    else
      FieldDefs.Add(TempFieldDef.FieldName, TempFieldDef.FieldType, 0, false);
    end;
    lFieldDefCount:=lFieldDefCount+1;

    FieldDefs[lFieldDefCount].Precision := TempFieldDef.Precision;

{$ifdef SUPPORT_FIELDDEF_ATTRIBUTES}
    // AutoInc fields are readonly
    if TempFieldDef.FieldType = ftAutoInc then
      FieldDefs[lFieldDefCount].Attributes := [Db.faReadOnly];

    // if table has dbase lock field, then hide it
    if TempFieldDef.IsLockField then
      FieldDefs[lFieldDefCount].Attributes := [Db.faHiddenCol];

    // Hide system/hidden fields (e.g. VFP's _NULLFLAGS)
    if TempFieldDef.IsSystemField then
      FieldDefs[lFieldDefCount].Attributes := [Db.faHiddenCol];
{$else}
    // Poor man's way of hiding fields that shouldn't be shown/modified:
    // Note: Visual Foxpro seems to allow adding another _NULLFLAGS field.
    // todo: test this with lockfield, then add this (TempFieldDef.IsLockField)
    if (TempFieldDef.IsSystemField) then
    begin
      FieldDefs.Delete(lFieldDefCount);
      lFieldDefCount:=lFieldDefCount-1;
    end;
{$endif}
  end;

  // get all (new) MDX index defs
  TempMdxFile := FDbfFile.MdxFile;
  for I := 0 to FDbfFile.IndexNames.Count - 1 do
  begin
    // is this an MDX index?
    lIndexName := FDbfFile.IndexNames.Strings[I];
    if FDbfFile.IndexNames.Objects[I] = TempMdxFile then
      if FIndexDefs.GetIndexByName(lIndexName) = nil then
        TempMdxFile.GetIndexInfo(lIndexName, FIndexDefs.Add);
  end;
end;

procedure TDbf.InitFieldDefs;
begin
  InternalInitFieldDefs;
end;

procedure TDbf.InitDbfFile(FileOpenMode: TPagedFileMode);
const
  FileModeToMemMode: array[TPagedFileMode] of TPagedFileMode =
    (pfNone, pfMemoryCreate, pfMemoryOpen, pfMemoryCreate, pfMemoryOpen,
     pfMemoryCreate, pfMemoryOpen, pfMemoryOpen);
begin
  FDbfFile := TDbfFile.Create;
  if FStorage = stoMemory then
  begin
    FDbfFile.Stream := FUserStream;
    FDbfFile.MemoStream := FUserMemoStream;
    FDbfFile.IndexStream := FUserIndexStream;
    FDbfFile.Mode := FileModeToMemMode[FileOpenMode];
  end else begin
    FDbfFile.FileName := FAbsolutePath + FTableName;
    FDbfFile.Mode := FileOpenMode;
  end;
  FDbfFile.AutoCreate := false;
  FDbfFile.DateTimeHandling := FDateTimeHandling;
  FDbfFile.OnLocaleError := FOnLocaleError;
  FDbfFile.OnIndexMissing := FOnIndexMissing;
end;

procedure TDbf.InternalInitFieldDefs; {override virtual abstract from TDataset}
var
  MustReleaseDbfFile: Boolean;
begin
  MustReleaseDbfFile := false;
  with FieldDefs do
  begin
    if FDbfFile = nil then
    begin
      // do not AutoCreate file
      InitDbfFile(pfReadOnly);
      FDbfFile.Open;
      MustReleaseDbfFile := true;
    end;
    GetFieldDefsFromDbfFieldDefs;
    if MustReleaseDbfFile then
      FreeAndNil(FDbfFile);
  end;
end;

procedure TDbf.InternalInitRecord(Buffer: TRecordBuffer); {override virtual abstract from TDataset}
var
  pRecord: pDbfRecord;
begin
  pRecord := pDbfRecord(Buffer);
  pRecord^.BookmarkData.PhysicalRecNo := 0;
  pRecord^.BookmarkFlag := bfCurrent;
  pRecord^.SequentialRecNo := 0;
// Init Record with zero and set autoinc field with next value
  FDbfFile.InitRecord(@pRecord^.DeletedFlag);
end;

procedure TDbf.InternalLast; {override virtual abstract from TDataset}
begin
  FCursor.Last;
end;

procedure TDbf.DetermineTranslationMode;
var
  lCodePage: Cardinal;
begin
  lCodePage := FDbfFile.UseCodePage;
  if lCodePage = GetACP then
    FTranslationMode := tmNoneNeeded
  else
  if lCodePage = GetOEMCP then
    FTranslationMode := tmSimple
  // check if this code page, although non default, is installed
  else
  if DbfGlobals.CodePageInstalled(lCodePage) then
    FTranslationMode := tmAdvanced
  else
    FTranslationMode := tmNoneAvailable;
end;

procedure TDbf.InternalOpen; {override virtual abstract from TDataset}
const
  DbfOpenMode: array[Boolean, Boolean] of TPagedFileMode =
     ((pfReadWriteOpen, pfExclusiveOpen), (pfReadOnly, pfReadOnly));
var
  lIndex: TDbfIndexDef;
  lIndexName: string;
  LanguageAction: TDbfLanguageAction;
  doCreate: Boolean;
  I: Integer;
begin
  // close current file
  FreeAndNil(FDbfFile);

  // does file not exist? -> create
  if ((FStorage = stoFile) and 
        not FileExists(FAbsolutePath + FTableName) and 
        (FOpenMode in [omAutoCreate, omTemporary])) or
     ((FStorage = stoMemory) and (FUserStream = nil)) then
  begin
    doCreate := true;
    if Assigned(FBeforeAutoCreate) then
      FBeforeAutoCreate(Self, doCreate);
    if doCreate then
      CreateTable
    else
      exit;
  end;

  // now we know for sure the file exists
  InitDbfFile(DbfOpenMode[FReadOnly, FExclusive]);
  FDbfFile.Open;

  // fail open?
{$ifndef FPC}  
  if FDbfFile.ForceClose then
    Abort;
{$endif}    

  // determine dbf version
  case FDbfFile.DbfVersion of
    xBaseIII:      FTableLevel := 3;
    xBaseIV:       FTableLevel := 4;
    xBaseVII:      FTableLevel := 7;
    xFoxPro:       FTableLevel := TDBF_TABLELEVEL_FOXPRO;
    xVisualFoxPro: FTableLevel := TDBF_TABLELEVEL_VISUALFOXPRO;
  end;
  FBackLink := FDbfFile.BackLink;
  FLanguageID := FDbfFile.LanguageID;

  // build VCL fielddef list from native DBF FieldDefs
(*
  if (FDbfFile.HeaderSize = 0) or (FDbfFile.FieldDefs.Count = 0) then
  begin
    if FieldDefs.Count > 0 then
    begin
      CreateTableFromFieldDefs;
    end else begin
      CreateTableFromFields;
    end;
  end else begin
*)
//    GetFieldDefsFromDbfFieldDefs;
//  end;

{$ifdef SUPPORT_FIELDDEFS_UPDATED}
  FieldDefs.Updated := False;
  FieldDefs.Update;
{$else}
  InternalInitFieldDefs;
{$endif}

  // create the fields dynamically
  if DefaultFields then
    CreateFields; // Create fields from fielddefs.

  BindFields(true);

  // create array of blobstreams to store memos in. each field is a possible blob
  FBlobStreams := AllocMem(FieldDefs.Count * SizeOf(TDbfBlobStream));

  // check codepage settings
  DetermineTranslationMode;
  if FTranslationMode = tmNoneAvailable then
  begin
    // no codepage available? ask user
    LanguageAction := laReadOnly;
    if Assigned(FOnLanguageWarning) then
      FOnLanguageWarning(Self, LanguageAction);
    case LanguageAction of
      laReadOnly: FTranslationMode := tmNoneAvailable;
      laForceOEM:
        begin
          FDbfFile.UseCodePage := GetOEMCP;
          FTranslationMode := tmSimple;
        end;
      laForceANSI:
        begin
          FDbfFile.UseCodePage := GetACP;
          FTranslationMode := tmNoneNeeded;
        end;
      laDefault:
        begin
          FDbfFile.UseCodePage := DbfGlobals.DefaultOpenCodePage;
          DetermineTranslationMode;
        end;
    end;
  end;

  // allocate a record buffer for temporary data
  FTempBuffer := AllocRecordBuffer;

  // open indexes
  for I := 0 to FIndexDefs.Count - 1 do
  begin
    lIndex := FIndexDefs.Items[I];
    lIndexName := ParseIndexName(lIndex.IndexFile);
    // if index does not exist -> create, if it does exist -> open only
    FDbfFile.OpenIndex(lIndexName, lIndex.SortField, false, lIndex.Options);
  end;

  // parse filter expression
  try
    ParseFilter(Filter);
  except
    // oops, a problem with parsing, clear filter for now
    on E: EDbfError do Filter := EmptyStr;
  end;

  SetIndexName(FIndexName);

// SetIndexName will have made the cursor for us if no index selected :-)
//  if FCursor = nil then FCursor := TDbfCursor.Create(FDbfFile);

  if FMasterLink.Active and Assigned(FIndexFile) then
    CheckMasterRange;
  InternalFirst;

//  FDbfFile.SetIndex(FIndexName);
//  FDbfFile.FIsCursorOpen := true;
end;

function TDbf.GetCodePage: Cardinal;
begin
  if FDbfFile <> nil then
    Result := FDbfFile.UseCodePage
  else
    Result := 0;
end;

function TDbf.GetLanguageStr: string;
begin
  if FDbfFile <> nil then
    Result := FDbfFile.LanguageStr;
end;

function TDbf.LockTable(const Wait: Boolean): Boolean;
begin
  CheckActive;
  Result := FDbfFile.LockAllPages(Wait);
end;

procedure TDbf.UnlockTable;
begin
  CheckActive;
  FDbfFile.UnlockAllPages;
end;

procedure TDbf.InternalEdit;
var
  I: Integer;
begin
  // store recno we are editing
  FEditingRecNo := FCursor.PhysicalRecNo;
  // reread blobs, execute cancel -> clears remembered memo pageno,
  // causing it to reread the x contents
  for I := 0 to Pred(FieldDefs.Count) do
    if Assigned(FBlobStreams^[I]) then
      FBlobStreams^[I].Cancel;
  // try to lock this record
  FDbfFile.LockRecord(FEditingRecNo, @pDbfRecord(ActiveBuffer)^.DeletedFlag);
  // succeeded!
end;

{$ifndef FPC}
{$ifndef DELPHI_3}

procedure TDbf.InternalInsert; {override virtual from TDataset}
begin
  CursorPosChanged;
end;

{$endif}
{$endif}

procedure TDbf.InternalPost; {override virtual abstract from TDataset}
var
  pRecord: pDbfRecord;
  I, newRecord: Integer;
begin
  // if internalpost is called, we know we are active
  pRecord := pDbfRecord(ActiveBuffer);
  // commit blobs
  for I := 0 to Pred(FieldDefs.Count) do
    if Assigned(FBlobStreams^[I]) then
      FBlobStreams^[I].Commit;
  if State = dsEdit then
  begin
    // write changes
    FDbfFile.UnlockRecord(FEditingRecNo, @pRecord^.DeletedFlag);
    // not editing anymore
    FEditingRecNo := -1;
  end else begin
    // insert
    newRecord := FDbfFile.Insert(@pRecord^.DeletedFlag);
    if newRecord > 0 then
      FCursor.PhysicalRecNo := newRecord;
  end;
  // set flag that TDataSet is about to post...so we can disable resync
  FPosting := true;
end;

procedure TDbf.Resync(Mode: TResyncMode);
begin
  // try to increase speed
  if not FDisableResyncOnPost or not FPosting then
    inherited;
  // clear post flag
  FPosting := false;
end;


{$ifndef SUPPORT_INITDEFSFROMFIELDS}

procedure TDbf.InitFieldDefsFromFields;
var
  I: Integer;
  F: TField;
begin
  { create fielddefs from persistent fields if needed }
  for I := 0 to FieldCount - 1 do
  begin
    F := Fields[I];
    with F do
    if FieldKind = fkData then begin
      FieldDefs.Add(FieldName,DataType,Size,Required);
    end;
  end;
end;

{$endif}

procedure TDbf.CreateTable;
begin
  CreateTableEx(nil);
end;

procedure TDbf.CheckDbfFieldDefs(ADbfFieldDefs: TDbfFieldDefs);
var
  I: Integer;
  TempDef: TDbfFieldDef;

    function FieldTypeStr(const FieldType: char): string;
    begin
      if FieldType = #0 then
        Result := 'NULL'
      else if FieldType > #127 then
        Result := 'ASCII '+IntToStr(Byte(FieldType))
      else
        Result := ' "'+fieldType+'" ';
      Result := ' ' + Result + '(#'+IntToHex(Byte(FieldType),SizeOf(FieldType))+') '
    end;

begin
  if ADbfFieldDefs = nil then exit;

  for I := 0 to ADbfFieldDefs.Count - 1 do
  begin
    // check dbffielddefs for errors
    TempDef := ADbfFieldDefs.Items[I];
    if FTableLevel < 7 then
      if not (TempDef.NativeFieldType in ['C', 'F', 'N', 'D', 'L', 'M']) then
        raise EDbfError.CreateFmt(STRING_INVALID_FIELD_TYPE,
          [FieldTypeStr(TempDef.NativeFieldType), TempDef.FieldName]);
  end;
end;

procedure TDbf.CreateTableEx(ADbfFieldDefs: TDbfFieldDefs);
var
  I: Integer;
  lIndex: TDbfIndexDef;
  lIndexName: string;
  tempFieldDefs: Boolean;
begin
  CheckInactive;
  tempFieldDefs := ADbfFieldDefs = nil;
  try
    try
      if tempFieldDefs then
      begin
        ADbfFieldDefs := TDbfFieldDefs.Create(Self);
        ADbfFieldDefs.DbfVersion := TableLevelToDbfVersion(FTableLevel);

        // get fields -> fielddefs if no fielddefs
{$ifndef FPC_VERSION}
        if FieldDefs.Count = 0 then
          InitFieldDefsFromFields;
{$endif}

        // fielddefs -> dbffielddefs
        for I := 0 to FieldDefs.Count - 1 do
        begin
          with ADbfFieldDefs.AddFieldDef do
          begin
            FieldName := FieldDefs.Items[I].Name;
            FieldType := FieldDefs.Items[I].DataType;
            if FieldDefs.Items[I].Size > 0 then
            begin
              Size := FieldDefs.Items[I].Size;
              Precision := FieldDefs.Items[I].Precision;
            end else begin
              SetDefaultSize;
            end;
          end;
        end;
      end;

      InitDbfFile(pfExclusiveCreate);
      FDbfFile.CopyDateTimeAsString := FInCopyFrom and FCopyDateTimeAsString;
      FDbfFile.DbfVersion := TableLevelToDbfVersion(FTableLevel);
      FDbfFile.BackLink := FBackLink;
      FDbfFile.FileLangID := FLanguageID;
      FDbfFile.Open;
      // Default memo blocklength for FoxPro/VisualFoxpro is 64 (not 512 as specs say)
      if FDbfFile.DbfVersion in [xFoxPro,xVisualFoxPro] then
        FDbfFile.FinishCreate(ADbfFieldDefs, 64)
      else
        FDbfFile.FinishCreate(ADbfFieldDefs, 512);

      // if creating memory table, use user-designated stream
      if FStorage = stoMemory then
      begin
        FUserStream := FDbfFile.Stream;
        FUserIndexStream := FDBfFile.IndexStream;
        FUserMemoStream := FDbfFile.MemoStream;
      end;

      // create all indexes
      for I := 0 to FIndexDefs.Count-1 do
      begin
        lIndex := FIndexDefs.Items[I];
        lIndexName := ParseIndexName(lIndex.IndexFile);
        FDbfFile.OpenIndex(lIndexName, lIndex.SortField, true, lIndex.Options);
      end;
    except
      // dbf file created?
      if (FDbfFile <> nil) and (FStorage = stoFile) then
      begin
        FreeAndNil(FDbfFile);
        SysUtils.DeleteFile(FAbsolutePath+FTableName);
      end;
      raise;
    end;
  finally
    // free temporary fielddefs
    if tempFieldDefs and Assigned(ADbfFieldDefs) then
      ADbfFieldDefs.Free;
    FreeAndNil(FDbfFile);
  end;
end;

procedure TDbf.EmptyTable;
begin
  Zap;
end;

procedure TDbf.Zap;
begin
  // are we active?
  CheckActive;
  FDbfFile.Zap;
end;

procedure TDbf.RestructureTable(ADbfFieldDefs: TDbfFieldDefs; Pack: Boolean);
begin
  CheckInactive;

  // check field defs for errors
  CheckDbfFieldDefs(ADbfFieldDefs);

  // open dbf file
  InitDbfFile(pfExclusiveOpen);
  FDbfFile.Open;

  // do restructure
  try
    FDbfFile.RestructureTable(ADbfFieldDefs, Pack);
  finally
    // close file
    FreeAndNil(FDbfFile);
  end;
end;

procedure TDbf.PackTable;
var
  oldIndexName: string;
begin
  CheckBrowseMode;
  // deselect any index while packing
  oldIndexName := IndexName;
  IndexName := EmptyStr;
  // pack
  FDbfFile.RestructureTable(nil, true);
  // reselect index
  IndexName := oldIndexName;
end;

procedure TDbf.CopyFrom(DataSet: TDataSet; FileName: string; DateTimeAsString: Boolean; Level: Integer);
var
  lPhysFieldDefs, lFieldDefs: TDbfFieldDefs;
  lSrcField, lDestField: TField;
  I: integer;
begin
  FInCopyFrom := true;
  lFieldDefs := TDbfFieldDefs.Create(nil);
  lPhysFieldDefs := TDbfFieldDefs.Create(nil);
  try
    if Active then
      Close;
    FilePath := ExtractFilePath(FileName);
    TableName := ExtractFileName(FileName);
    FCopyDateTimeAsString := DateTimeAsString;
    TableLevel := Level;
    if not DataSet.Active then
      DataSet.Open;
    DataSet.FieldDefs.Update;
    // first get a list of physical field defintions
    // we need it for numeric precision in case source is tdbf
    if DataSet is TDbf then
    begin
      lPhysFieldDefs.Assign(TDbf(DataSet).DbfFieldDefs);
      IndexDefs.Assign(TDbf(DataSet).IndexDefs);
    end else begin
{$ifdef SUPPORT_FIELDDEF_TPERSISTENT}
      lPhysFieldDefs.Assign(DataSet.FieldDefs);
{$endif}      
      IndexDefs.Clear;
    end;
    // convert list of tfields into a list of tdbffielddefs
    // so that our tfields will correspond to the source tfields
    for I := 0 to Pred(DataSet.FieldCount) do
    begin
      lSrcField := DataSet.Fields[I];
      with lFieldDefs.AddFieldDef do
      begin
        if Length(lSrcField.Name) > 0 then
          FieldName := lSrcField.Name
        else
          FieldName := lSrcField.FieldName;
        FieldType := lSrcField.DataType;
        Required := lSrcField.Required;

        // Set up size/precision for all physical fields:
        if (1 <= lSrcField.FieldNo) 
            and (lSrcField.FieldNo <= lPhysFieldDefs.Count) then
        begin
          Size := lPhysFieldDefs.Items[lSrcField.FieldNo-1].Size;
          Precision := lPhysFieldDefs.Items[lSrcField.FieldNo-1].Precision;
        end;
      end;
    end;

    CreateTableEx(lFieldDefs);
    Open;
    DataSet.First;
{$ifdef USE_CACHE}
    FDbfFile.BufferAhead := true;
    if DataSet is TDbf then
      TDbf(DataSet).DbfFile.BufferAhead := true;
{$endif}      
    while not DataSet.EOF do
    begin
      Append;
      for I := 0 to Pred(FieldCount) do
      begin
        lSrcField := DataSet.Fields[I];
        lDestField := Fields[I];
        if not lSrcField.IsNull then
        begin
          if lSrcField.DataType = ftDateTime then
          begin
            if FCopyDateTimeAsString then
            begin
              lDestField.AsString := lSrcField.AsString;
              if Assigned(FOnCopyDateTimeAsString) then
                FOnCopyDateTimeAsString(Self, lDestField, lSrcField)
            end else
              lDestField.AsDateTime := lSrcField.AsDateTime;
          end else
            lDestField.Assign(lSrcField);
        end;
      end;
      Post;
      DataSet.Next;
    end;
    Close;
  finally
{$ifdef USE_CACHE}
    if (DataSet is TDbf) and (TDbf(DataSet).DbfFile <> nil) then
      TDbf(DataSet).DbfFile.BufferAhead := false;
{$endif}      
    FInCopyFrom := false;
    lFieldDefs.Free;
    lPhysFieldDefs.Free;
  end;
end;

function TDbf.FindRecord(Restart, GoForward: Boolean): Boolean;
var
  oldRecNo: Integer;
begin
  CheckBrowseMode;
  DoBeforeScroll;
  Result := false;
  UpdateCursorPos;
  oldRecNo := RecNo;
  try
    FFindRecordFilter := true;
    if GoForward then
    begin
      if Restart then FCursor.First;
      Result := GetRecord(FTempBuffer, gmNext, false) = grOK;
    end else begin
      if Restart then FCursor.Last;
      Result := GetRecord(FTempBuffer, gmPrior, false) = grOK;
    end;
  finally
    FFindRecordFilter := false;
    if not Result then
    begin
      RecNo := oldRecNo;
    end else begin
      CursorPosChanged;
      Resync([]);
      DoAfterScroll;
    end;
  end;
end;

{$ifdef SUPPORT_VARIANTS}

function TDbf.Lookup(const KeyFields: string; const KeyValues: Variant;
  const ResultFields: string): Variant;
var
//  OldState:  TDataSetState;
  saveRecNo: integer;
  saveState: TDataSetState;
begin
  Result := Null;
  if (FCursor = nil) or VarIsNull(KeyValues) then exit;

  saveRecNo := FCursor.SequentialRecNo;
  try
    if LocateRecord(KeyFields, KeyValues, []) then
    begin
      // FFilterBuffer contains record buffer
      saveState := SetTempState(dsCalcFields);
      try
        CalculateFields(FFilterBuffer);
        if KeyValues = FieldValues[KeyFields] then
           Result := FieldValues[ResultFields];
      finally
        RestoreState(saveState);
      end;
    end;
  finally
    FCursor.SequentialRecNo := saveRecNo;
  end;
end;

function TDbf.Locate(const KeyFields: string; const KeyValues: Variant; Options: TLocateOptions): Boolean;
var
  saveRecNo: integer;
begin
  if FCursor = nil then
  begin
    CheckActive;
    Result := false;
    exit;
  end;

  DoBeforeScroll;
  saveRecNo := FCursor.SequentialRecNo;
  FLocateRecNo := -1;
  Result := LocateRecord(KeyFields, KeyValues, Options);
  CursorPosChanged;
  if Result then
  begin
    if FLocateRecNo <> -1 then
      FCursor.PhysicalRecNo := FLocateRecNo;
    Resync([]);
    DoAfterScroll;
  end else
    FCursor.SequentialRecNo := saveRecNo;
end;

function TDbf.LocateRecordLinear(const KeyFields: String; const KeyValues: Variant;
    Options: TLocateOptions): Boolean;
var
  lstKeys              : TList;
  iIndex               : Integer;
  Field                : TField;
  bMatchedData         : Boolean;
  bVarIsArray          : Boolean;
  varCompare           : Variant;

  function CompareValues: Boolean;
  var
    sCompare: String;
  begin
    if (Field.DataType in [ftString,ftWideString]) then
    begin
      sCompare := VarToStr(varCompare);
      if loCaseInsensitive in Options then
      begin
        Result := AnsiCompareText(Field.AsString,sCompare) = 0;
        if not Result and (iIndex = lstKeys.Count - 1) and (loPartialKey in Options) and
          (Length(sCompare) < Length(Field.AsString)) then
        begin
          if Length(sCompare) = 0 then
            Result := true
          else
            Result := AnsiCompareText (Copy (Field.AsString,1,Length (sCompare)),sCompare) = 0;
        end;
      end else begin
        Result := Field.AsString = sCompare;
        if not Result and (iIndex = lstKeys.Count - 1) and (loPartialKey in Options) and
          (Length (sCompare) < Length (Field.AsString)) then
        begin
          if Length (sCompare) = 0 then
            Result := true
          else
            Result := Copy(Field.AsString, 1, Length(sCompare)) = sCompare;
        end;
      end;
    end
    else
      // Not a string; could be date, integer etc.
      // Follow e.g. FPC bufdataset by searching for equal  
      Result := Field.Value = varCompare;
  end;

var
  SaveState: TDataSetState;
  lPhysRecNo: integer;
begin
  Result := false;
  bVarIsArray := false;
  lstKeys := TList.Create;
  FFilterBuffer := TempBuffer;
  SaveState := SetTempState(dsFilter);
  try
    GetFieldList(lstKeys, KeyFields);
    if VarArrayDimCount(KeyValues) = 0 then
      bMatchedData := lstKeys.Count = 1
    else if VarArrayDimCount (KeyValues) = 1 then
    begin
      bMatchedData := VarArrayHighBound (KeyValues,1) + 1 = lstKeys.Count;
      bVarIsArray := true;
    end else
      bMatchedData := false;
    if bMatchedData then
    begin
      FCursor.First;
      while not Result and FCursor.Next do
      begin
        lPhysRecNo := FCursor.PhysicalRecNo;
        if (lPhysRecNo = 0) or not FDbfFile.IsRecordPresent(lPhysRecNo) then
          break;
        
        FDbfFile.ReadRecord(lPhysRecNo, @PDbfRecord(FFilterBuffer)^.DeletedFlag);
        Result := FShowDeleted or (PDbfRecord(FFilterBuffer)^.DeletedFlag <> '*');
        if Result and Filtered then
          DoFilterRecord(Result);
        
        iIndex := 0;
        while Result and (iIndex < lstKeys.Count) Do
        begin
          Field := TField (lstKeys [iIndex]);
          if bVarIsArray then
            varCompare := KeyValues [iIndex]
          else
            varCompare := KeyValues;
          Result := CompareValues;
          Inc(iIndex);
        end;
      end;
    end;
  finally
    lstKeys.Free;
    RestoreState(SaveState);
  end;
end;

function TDbf.LocateRecordIndex(const KeyFields: String; const KeyValues: Variant;
    Options: TLocateOptions): Boolean;
var
  searchFlag: TSearchKeyType;
  matchRes: Integer;
  lTempBuffer: array [0..100] of Char;
  acceptable, checkmatch: boolean;
begin
  // Only honor loPartialKey for string types; for others, search for equal
  if (loPartialKey in Options) and
    (TIndexCursor(FCursor).IndexFile.KeyType='C') then
    searchFlag := stGreaterEqual
  else
    searchFlag := stEqual;
  if TIndexCursor(FCursor).VariantToBuffer(KeyValues, @lTempBuffer[0]) = etString then
    Translate(@lTempBuffer[0], @lTempBuffer[0], true);
  Result := FIndexFile.SearchKey(@lTempBuffer[0], searchFlag);
  if not Result then
    exit;

  checkmatch := false;
  repeat
    if ReadCurrentRecord(TempBuffer, acceptable) = grError then
    begin
      Result := false;
      exit;
    end;
    if acceptable then break;
    checkmatch := true;
    FCursor.Next;
  until false;

  if checkmatch then
  begin
    matchRes := TIndexCursor(FCursor).IndexFile.MatchKey(@lTempBuffer[0]);
    if loPartialKey in Options then
      Result := matchRes <= 0
    else
      Result := matchRes =  0;
  end;

  FFilterBuffer := TempBuffer;
end;

function TDbf.LocateRecord(const KeyFields: String; const KeyValues: Variant;
    Options: TLocateOptions): Boolean;
var
  lCursor, lSaveCursor: TVirtualCursor;
  lSaveIndexName, lIndexName: string;
  lIndexDef: TDbfIndexDef;
  lIndexFile, lSaveIndexFile: TIndexFile;
begin
  lCursor := nil;
  lSaveCursor := nil;
  lIndexFile := nil;
  lSaveIndexFile := FIndexFile;
  if (FCursor is TIndexCursor) 
    and (TIndexCursor(FCursor).IndexFile.Expression = KeyFields) then
  begin
    lCursor := FCursor;
  end else begin
    lIndexDef := FIndexDefs.GetIndexByField(KeyFields);
    if lIndexDef <> nil then
    begin
      lIndexName := ParseIndexName(lIndexDef.IndexFile);
      lIndexFile := FDbfFile.GetIndexByName(lIndexName);
      if lIndexFile <> nil then
      begin
        lSaveCursor := FCursor;
        lCursor := TIndexCursor.Create(lIndexFile);
        lSaveIndexName := lIndexFile.IndexName;
        lIndexFile.IndexName := lIndexName;
        FIndexFile := lIndexFile;
      end;
    end;
  end;
  if lCursor <> nil then
  begin
    FCursor := lCursor;
    Result := LocateRecordIndex(KeyFields, KeyValues, Options);
    if lSaveCursor <> nil then
    begin
      FCursor.Free;
      FCursor := lSaveCursor;
    end;
    if lIndexFile <> nil then
    begin
      FLocateRecNo := FIndexFile.PhysicalRecNo;
      lIndexFile.IndexName := lSaveIndexName;
      FIndexFile := lSaveIndexFile;
    end;
  end else
    Result := LocateRecordLinear(KeyFields, KeyValues, Options);
end;

{$endif}

procedure TDbf.TryExclusive;
begin
  // are we active?
  if Active then
  begin
    // already in exclusive mode?
    FDbfFile.TryExclusive;
    // update file mode
    FExclusive := not FDbfFile.IsSharedAccess;
    FReadOnly := FDbfFile.Mode = pfReadOnly;
  end else begin
    // just set exclusive to true
    FExclusive := true;
    FReadOnly := false;
  end;
end;

procedure TDbf.EndExclusive;
begin
  if Active then
  begin
    // call file handler
    FDbfFile.EndExclusive;
    // update file mode
    FExclusive := not FDbfFile.IsSharedAccess;
    FReadOnly := FDbfFile.Mode = pfReadOnly;
  end else begin
    // just set exclusive to false
    FExclusive := false;
  end;
end;

function TDbf.CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; {override virtual}
var
  MemoPageNo: Integer;
  MemoFieldNo: Integer;
  lBlob: TDbfBlobStream;
begin
  // check if in editing mode if user wants to write
  if (Mode = bmWrite) or (Mode = bmReadWrite) then
    if not (State in [dsEdit, dsInsert]) then
{$ifdef DELPHI_3}
      DatabaseError(SNotEditing);
{$else}
      DatabaseError(SNotEditing, Self);
{$endif}
  // already created a `placeholder' blob for this field?
  MemoFieldNo := Field.FieldNo - 1;
  if FBlobStreams^[MemoFieldNo] = nil then
    FBlobStreams^[MemoFieldNo] := TDbfBlobStream.Create(Field);
  lBlob := FBlobStreams^[MemoFieldNo].AddReference;
  // update pageno of blob <-> location where to read/write in memofile
  if FDbfFile.GetFieldData(Field.FieldNo-1, ftInteger, GetCurrentBuffer, @MemoPageNo, false) then
  begin
    // read blob? different blob?
    if (Mode = bmRead) or (Mode = bmReadWrite) then
    begin
      if MemoPageNo <> lBlob.MemoRecNo then
      begin
        FDbfFile.MemoFile.ReadMemo(MemoPageNo, lBlob);
        lBlob.ReadSize := lBlob.Size;
        lBlob.Translate(false);
      end;
    end else begin
      lBlob.Size := 0;
      lBlob.ReadSize := 0;
    end;
    lBlob.MemoRecNo := MemoPageNo;
  end else
  if not lBlob.Dirty or (Mode = bmWrite) then
  begin
    // reading and memo is empty and not written yet, or rewriting
    lBlob.Size := 0;
    lBlob.ReadSize := 0;
    lBlob.MemoRecNo := 0;
  end;
  { this is a hack, we actually need to know per user who's modifying, and who is not }
  { Mode is more like: the mode of the last "creation" }
  { if create/free is nested, then everything will be alright, I think ;-) }
  lBlob.Mode := Mode;
  { this is a hack: we actually need to know per user what its position is }
  lBlob.Position := 0;
  Result := lBlob;
end;

{$ifdef SUPPORT_NEW_TRANSLATE}

function TDbf.Translate(Src, Dest: PChar; ToOem: Boolean): Integer; {override virtual}
var
  FromCP, ToCP: Cardinal;
begin
  if (Src <> nil) and (Dest <> nil) then
  begin
    if Assigned(FOnTranslate) then
    begin
      Result := FOnTranslate(Self, Src, Dest, ToOem);
      if Result = -1 then
        Result := StrLen(Dest);
    end else begin
      if FTranslationMode <> tmNoneNeeded then
      begin
        if ToOem then
        begin
          FromCP := GetACP;
          ToCP := FDbfFile.UseCodePage;
        end else begin
          FromCP := FDbfFile.UseCodePage;
          ToCP := GetACP;
        end;
      end else begin
        FromCP := GetACP;
        ToCP := FromCP;
      end;
      Result := TranslateString(FromCP, ToCP, Src, Dest, -1);
    end;
  end else
    Result := 0;
end;

{$else}

procedure TDbf.Translate(Src, Dest: PChar; ToOem: Boolean); {override virtual}
var
  FromCP, ToCP: Cardinal;
begin
  if (Src <> nil) and (Dest <> nil) then
  begin
    if Assigned(FOnTranslate) then
    begin
      FOnTranslate(Self, Src, Dest, ToOem);
    end else begin
      if FTranslationMode <> tmNoneNeeded then
      begin
        if ToOem then
        begin
          FromCP := GetACP;
          ToCP := FDbfFile.UseCodePage;
        end else begin
          FromCP := FDbfFile.UseCodePage;
          ToCP := GetACP;
        end;
        TranslateString(FromCP, ToCP, Src, Dest, -1);
      end;
    end;
  end;
end;

{$endif}

procedure TDbf.ClearCalcFields(Buffer: TRecordBuffer);
var
  lRealBuffer, lCalcBuffer: PChar;
begin
  lRealBuffer := @pDbfRecord(Buffer)^.DeletedFlag;
  lCalcBuffer := lRealBuffer + FDbfFile.RecordSize;
  FillChar(lCalcBuffer^, CalcFieldsSize, 0);
end;

procedure TDbf.InternalSetToRecord(Buffer: TRecordBuffer); {override virtual abstract from TDataset}
var
  pRecord: pDbfRecord;
begin
  if Buffer <> nil then
  begin
    pRecord := pDbfRecord(Buffer);
    if pRecord^.BookmarkFlag = bfInserted then
    begin
      // do what ???
    end else begin
      FCursor.SequentialRecNo := pRecord^.SequentialRecNo;
    end;
  end;
end;

function TDbf.IsCursorOpen: Boolean; {override virtual abstract from TDataset}
begin
  Result := FCursor <> nil;
end;

function TDbf.FieldDefsStored: Boolean;
begin
  Result := StoreDefs and (FieldDefs.Count > 0);
end;

procedure TDbf.SetBackLink(NewBackLink: String);
begin
  // Only supported in Visual Foxpro but allow auto-upgrade from Foxpro
  // as well as resetting existing backlinks in any tablelevel
  if (NewBackLink<>'') and
    (not(Tablelevel in [TDBF_TABLELEVEL_FOXPRO,TDBF_TABLELEVEL_VISUALFOXPRO])) then
    raise EDbfError.CreateFmt(STRING_FEATURE_NOT_SUPPORTED_THIS_TABLELEVEL,
      [Tablelevel]);
  CheckInactive;

  FBackLink := NewBackLink;
end;

procedure TDbf.SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); {override virtual abstract from TDataset}
begin
  pDbfRecord(Buffer)^.BookmarkFlag := Value;
end;

procedure TDbf.SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); {override virtual abstract from TDataset}
begin
  pDbfRecord(Buffer)^.BookmarkData := pBookmarkData(Data)^;
end;

// this function counts real number of records: skip deleted records, filter, etc.
// warning: is very slow, compared to GetRecordCount
function TDbf.GetExactRecordCount: Integer;
var
  prevRecNo: Integer;
  getRes: TGetResult;
begin
  // init vars
  Result := 0;

  // check if FCursor open
  if FCursor = nil then
    exit; 

  // store current position
  prevRecNo := FCursor.SequentialRecNo;
  FCursor.First;
  repeat
    // repeatedly retrieve next record until eof encountered
    getRes := GetRecord(FTempBuffer, gmNext, true);
    if getRes = grOk then
      inc(Result);
  until getRes <> grOk;
  // restore current position
  FCursor.SequentialRecNo := prevRecNo;
end;

// this functions returns the physical number of records present in file
function TDbf.GetPhysicalRecordCount: Integer;
begin
  if FDbfFile <> nil then
    Result := FDbfFile.RecordCount
  else
    Result := 0
end;

// this function is just for the grid scrollbars
// it doesn't have to be perfectly accurate, but fast.
function TDbf.GetRecordCount: Integer; {override virtual}
begin
  if FCursor <> nil then
    Result := FCursor.SequentialRecordCount
  else
    Result := 0
end;

// this function is just for the grid scrollbars
// it doesn't have to be perfectly accurate, but fast.
function TDbf.GetRecNo: Integer; {override virtual}
var
  pBuffer: pointer;
begin
  if FCursor <> nil then
  begin
    case State of
      dsFilter: pBuffer := FFilterBuffer;
      dsCalcFields: pBuffer := CalcBuffer;
    else
      pBuffer := ActiveBuffer;
    end;
    Result := pDbfRecord(pBuffer)^.SequentialRecNo;
  end else
    Result := 0;
end;

procedure TDbf.SetRecNo(Value: Integer); {override virtual}
begin
  CheckBrowseMode;
  if Value = RecNo then
    exit;

  DoBeforeScroll;
  FCursor.SequentialRecNo := Value;
  CursorPosChanged;
  Resync([]);
  DoAfterScroll;
end;

function TDbf.GetCanModify: Boolean; {override;}
begin
  if FReadOnly or (csDesigning in ComponentState) then
    Result := false
  else
    Result := FTranslationMode > tmNoneAvailable;
end;

{$ifdef SUPPORT_DEFCHANGED}

procedure TDbf.DefChanged(Sender: TObject);
begin
  StoreDefs := true;
end;

{$endif}

procedure TDbf.ParseFilter(const AFilter: string);
begin
  // parser created?
  if Length(AFilter) > 0 then
  begin
    if (FParser = nil) and (FDbfFile <> nil) then
    begin
      FParser := TDbfParser.Create(FDbfFile);
      // we need truncated, translated (to ANSI) strings
      FParser.StringFieldMode := smAnsiTrim;
    end;
    // have a parser now?
    if FParser <> nil then
    begin
      // set options
      FParser.PartialMatch := not (foNoPartialCompare in FilterOptions);
      FParser.CaseInsensitive := foCaseInsensitive in FilterOptions;
      // parse expression
      FParser.ParseExpression(AFilter);
    end;
  end;
end;

procedure TDbf.SetFilterText(const Value: String);
begin
  if Value = Filter then
    exit;

  // parse
  ParseFilter(Value);

  // call dataset method
  inherited;

  // refilter dataset if filtered
  if (FDbfFile <> nil) and Filtered then Refresh;
end;

procedure TDbf.SetFiltered(Value: Boolean); {override;}
begin
  if Value = Filtered then
    exit;

  // pass on to ancestor
  inherited;

  // only refresh if active
  if FCursor <> nil then
    Refresh;
end;

procedure TDbf.SetFilePath(const Value: string);
begin
  CheckInactive;

  FRelativePath := Value;
  if Length(FRelativePath) > 0 then
       FRelativePath := IncludeTrailingPathDelimiter(FRelativePath);

  if IsFullFilePath(Value) then
  begin
    FAbsolutePath := IncludeTrailingPathDelimiter(Value);
  end else begin
    FAbsolutePath := GetCompletePath(DbfBasePath(), FRelativePath);
  end;
end;

procedure TDbf.SetTableName(const S: string);
var
  lPath: string;
begin
  FTableName := ExtractFileName(s);
  lPath := ExtractFilePath(s);
  if (Length(lPath) > 0) then
    FilePath := lPath;
  // force IDE to reread fielddefs when a different file is opened
{$ifdef SUPPORT_FIELDDEFS_UPDATED}
  FieldDefs.Updated := false;
{$else}
  // TODO ... ??
{$endif}
end;

procedure TDbf.SetDbfIndexDefs(const Value: TDbfIndexDefs);
begin
  FIndexDefs.Assign(Value);
end;

procedure TDbf.SetLanguageID(NewID: Byte);
begin
  CheckInactive;
  
  FLanguageID := NewID;
end;

procedure TDbf.SetTableLevel(const NewLevel: Integer);
begin
  if NewLevel <> FTableLevel then
  begin
    // check validity
    if not (NewLevel in [3,4,7,TDBF_TABLELEVEL_FOXPRO,TDBF_TABLELEVEL_VISUALFOXPRO]) then
      exit;

    // can only assign tablelevel if table is closed
    CheckInactive;
    FTableLevel := NewLevel;
  end;
end;

function TDbf.GetIndexName: string;
begin
  Result := FIndexName;
end;

function TDbf.CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Integer;
const
  RetCodes: array[Boolean, Boolean] of ShortInt = ((2,-1),(1,0));
var
  b1,b2: Integer;
begin
  // Check for uninitialized bookmarks
  Result := RetCodes[Bookmark1 = nil, Bookmark2 = nil];
  if (Result = 2) then
  begin
    b1 := PInteger(Bookmark1)^;
    b2 := PInteger(Bookmark2)^;
    if b1 < b2 then Result := -1
    else if b1 > b2 then Result := 1
    else Result := 0;
  end;
end;

function TDbf.GetVersion: string;
begin
  Result := Format('%d.%02d', [TDBF_MAJOR_VERSION, TDBF_MINOR_VERSION]);
end;

procedure TDbf.SetVersion(const S: string);
begin
  // What an idea...
end;

function TDbf.ParseIndexName(const AIndexName: string): string;
begin
  // if no ext, then it is a MDX tag, get complete only if it is a filename
  // MDX: get first 10 characters only
  if Length(ExtractFileExt(AIndexName)) > 0 then
    Result := GetCompleteFileName(FAbsolutePath, AIndexName)
  else
    Result := AIndexName;
end;

procedure TDbf.RegenerateIndexes;
begin
  CheckBrowseMode;
  FDbfFile.RegenerateIndexes;
end;

{$ifdef SUPPORT_DEFAULT_PARAMS}
procedure TDbf.AddIndex(const AIndexName, AFields: String; Options: TIndexOptions; const DescFields: String='');
{$else}
procedure TDbf.AddIndex(const AIndexName, AFields: String; Options: TIndexOptions);
{$endif}
var
  lIndexFileName: string;
begin
  CheckActive;
  lIndexFileName := ParseIndexName(AIndexName);
  FDbfFile.OpenIndex(lIndexFileName, AFields, true, Options);

  // refresh our indexdefs
  InternalInitFieldDefs;
end;

procedure TDbf.SetIndexName(AIndexName: string);
var
  lRecNo: Integer;
begin
  FIndexName := AIndexName;
  if FDbfFile = nil then
    exit;

  // get accompanying index file
  AIndexName := ParseIndexName(Trim(AIndexName));
  FIndexFile := FDbfFile.GetIndexByName(AIndexName);
  // store current lRecNo
  if FCursor = nil then
  begin
    lRecNo := 1;
  end else begin
    UpdateCursorPos;
    lRecNo := FCursor.PhysicalRecNo;
  end;
  // select new cursor
  FreeAndNil(FCursor);
  if FIndexFile <> nil then
  begin
    FCursor := TIndexCursor.Create(FIndexFile);
    // select index
    FIndexFile.IndexName := AIndexName;
    // check if can activate master link
    CheckMasterRange;
  end else begin
    FCursor := TDbfCursor.Create(FDbfFile);
    FIndexName := EmptyStr;
  end;
  // reset previous lRecNo
  FCursor.PhysicalRecNo := lRecNo;
  // refresh records
  if State = dsBrowse then
    Resync([]);
  // warn user if selecting non-existing index
  if (FCursor = nil) and (AIndexName <> EmptyStr) then
    raise EDbfError.CreateFmt(STRING_INDEX_NOT_EXIST, [AIndexName]);
end;

function TDbf.GetIndexFieldNames: string;
var
  lIndexDef: TDbfIndexDef;
begin
  lIndexDef := FIndexDefs.GetIndexByName(IndexName);
  if lIndexDef = nil then
    Result := EmptyStr
  else
    Result := lIndexDef.SortField;
end;

procedure TDbf.SetIndexFieldNames(const Value: string);
var
  lIndexDef: TDbfIndexDef;
begin
  // Exception if index not found?
  lIndexDef := FIndexDefs.GetIndexByField(Value);
  if lIndexDef = nil then
    IndexName := EmptyStr
  else
    IndexName := lIndexDef.IndexFile;
end;

procedure TDbf.DeleteIndex(const AIndexName: string);
var
  lIndexFileName: string;
begin
  // extract absolute path if NDX file
  lIndexFileName := ParseIndexName(AIndexName);
  // try to delete index
  FDbfFile.DeleteIndex(lIndexFileName);

  // refresh index defs
  InternalInitFieldDefs;
end;

procedure TDbf.OpenIndexFile(IndexFile: string);
var
  lIndexFileName: string;
begin
  CheckActive;
  // make absolute path
  lIndexFileName := GetCompleteFileName(FAbsolutePath, IndexFile);
  // open index
  FDbfFile.OpenIndex(lIndexFileName, '', false, []);
end;

procedure TDbf.CloseIndexFile(const AIndexName: string);
var
  lIndexFileName: string;
begin
  CheckActive;
  // make absolute path
  lIndexFileName := GetCompleteFileName(FAbsolutePath, AIndexName);
  // close this index
  FDbfFile.CloseIndex(lIndexFileName);
end;

procedure TDbf.RepageIndexFile(const AIndexFile: string);
begin
  if FDbfFile <> nil then
    FDbfFile.RepageIndex(ParseIndexName(AIndexFile));
end;

procedure TDbf.CompactIndexFile(const AIndexFile: string);
begin
  if FDbfFile <> nil then
    FDbfFile.CompactIndex(ParseIndexName(AIndexFile));
end;

procedure TDbf.GetFileNames(Strings: TStrings; Files: TDbfFileNames);
var
  I: Integer;
begin
  Strings.Clear;
  if FDbfFile <> nil then
  begin
    if dfDbf in Files then
      Strings.Add(FDbfFile.FileName);
    if (dfMemo in Files) and (FDbfFile.MemoFile <> nil) then
      Strings.Add(FDbfFile.MemoFile.FileName);
    if dfIndex in Files then
      for I := 0 to Pred(FDbfFile.IndexFiles.Count) do
        Strings.Add(TPagedFile(FDbfFile.IndexFiles.Items[I]).FileName);
  end else
    Strings.Add(IncludeTrailingPathDelimiter(FilePathFull) + TableName);   
end;

{$ifdef SUPPORT_DEFAULT_PARAMS}
function TDbf.GetFileNames(Files: TDbfFileNames (* = [dfDbf] *) ): string;
{$else}
function TDbf.GetFileNamesString(Files: TDbfFileNames ): string;
{$endif}
var
  sl: TStrings;
begin
  sl := TStringList.Create;
  try
    GetFileNames(sl, Files);
    Result := sl.Text;
  finally
    sl.Free;
  end;
end;



procedure TDbf.GetIndexNames(Strings: TStrings);
begin
  CheckActive;
  Strings.Assign(DbfFile.IndexNames)
end;

procedure TDbf.GetAllIndexFiles(Strings: TStrings);
var
  SR: TSearchRec;
begin
  CheckActive;
  Strings.Clear;
  if SysUtils.FindFirst(IncludeTrailingPathDelimiter(ExtractFilePath(FDbfFile.FileName))
        + '*.NDX', faAnyFile, SR) = 0 then
  begin
    repeat
      Strings.Add(SR.Name);
    until SysUtils.FindNext(SR)<>0;
    SysUtils.FindClose(SR);
  end;
end;

function TDbf.GetPhysicalRecNo: Integer;
var
  pBuffer: pointer;
begin
  // check if active, test state: if inserting, then -1
  if (FCursor <> nil) and (State <> dsInsert) then
  begin
    if State = dsCalcFields then
      pBuffer := CalcBuffer
    else
      pBuffer := ActiveBuffer;
    Result := pDbfRecord(pBuffer)^.BookmarkData.PhysicalRecNo;
  end else
    Result := -1;
end;

procedure TDbf.SetPhysicalRecNo(const NewRecNo: Integer);
begin
  // editing?
  CheckBrowseMode;
  DoBeforeScroll;
  FCursor.PhysicalRecNo := NewRecNo;
  CursorPosChanged;
  Resync([]);
  DoAfterScroll;
end;

function TDbf.GetDbfFieldDefs: TDbfFieldDefs;
begin
  if FDbfFile <> nil then
    Result := FDbfFile.FieldDefs
  else
    Result := nil;
end;

procedure TDbf.SetShowDeleted(Value: Boolean);
begin
  // test if changed
  if Value <> FShowDeleted then
  begin
    // store new value
    FShowDeleted := Value;
    // refresh view only if active
    if FCursor <> nil then
      Refresh;
  end;
end;

function TDbf.IsDeleted: Boolean;
var
  src: TRecordBuffer;
begin
  src := GetCurrentBuffer;
  IsDeleted := (src=nil) or (AnsiChar(src^) = '*')
end;

procedure TDbf.Undelete;
var
  src: TRecordBuffer;
begin
  if State <> dsEdit then
    inherited Edit;
  // get active buffer
  src := GetCurrentBuffer;
  if (src <> nil) and (AnsiChar(src^) = '*') then
  begin
    // notify indexes record is about to be recalled
    FDbfFile.RecordRecalled(FCursor.PhysicalRecNo, src);
    // recall record
    src^ := TRecordBufferBaseType(' ');
    FDbfFile.WriteRecord(FCursor.PhysicalRecNo, src);
  end;
end;

procedure TDbf.CancelRange;
begin
  if FIndexFile = nil then
    exit;

  // disable current range if any
  FIndexFile.CancelRange;
  // reretrieve previous and next records
  Refresh;
end;

procedure TDbf.SetRangeBuffer(LowRange: PChar; HighRange: PChar);
begin
  if FIndexFile = nil then
    exit;

  FIndexFile.SetRange(LowRange, HighRange);
  // go to first in this range
  if Active then
    inherited First;
end;

{$ifdef SUPPORT_VARIANTS}

procedure TDbf.SetRange(LowRange: Variant; HighRange: Variant; KeyIsANSI: boolean);
var
  LowBuf, HighBuf: array[0..100] of Char;
begin
  if (FIndexFile = nil) or VarIsNull(LowRange) or VarIsNull(HighRange) then
    exit;

  // convert variants to index key type
  if (TIndexCursor(FCursor).VariantToBuffer(LowRange,  @LowBuf[0]) = etString) and KeyIsANSI then
    Translate(@LowBuf[0], @LowBuf[0], true);
  if (TIndexCursor(FCursor).VariantToBuffer(HighRange, @HighBuf[0]) = etString) and KeyIsANSI then
    Translate(@HighBuf[0], @HighBuf[0], true);
  SetRangeBuffer(@LowBuf[0], @HighBuf[0]);
end;

{$endif}

procedure TDbf.SetRangePChar(LowRange: PChar; HighRange: PChar; KeyIsANSI: boolean);
var
  LowBuf, HighBuf: array [0..100] of Char;
  LowPtr, HighPtr: PChar;
begin
  if FIndexFile = nil then
    exit;

  // convert to pchars
  if KeyIsANSI then
  begin
    Translate(LowRange, @LowBuf[0], true);
    Translate(HighRange, @HighBuf[0], true);
    LowRange := @LowBuf[0];
    HighRange := @HighBuf[0];
  end;
  LowPtr  := TIndexCursor(FCursor).CheckUserKey(LowRange,  @LowBuf[0]);
  HighPtr := TIndexCursor(FCursor).CheckUserKey(HighRange, @HighBuf[0]);
  SetRangeBuffer(LowPtr, HighPtr);
end;

procedure TDbf.ExtractKey(KeyBuffer: PChar);
begin
  if FIndexFile <> nil then
    StrCopy(FIndexFile.ExtractKeyFromBuffer(GetCurrentBuffer), KeyBuffer)
  else
    KeyBuffer[0] := #0;
end;

function TDbf.GetKeySize: Integer;
begin
  if FCursor is TIndexCursor then
    Result := TIndexCursor(FCursor).IndexFile.KeyLen
  else
    Result := 0;
end;

{$ifdef SUPPORT_VARIANTS}

function TDbf.SearchKey(Key: Variant; SearchType: TSearchKeyType; KeyIsANSI: boolean): Boolean;
var
  TempBuffer: array [0..100] of Char;
begin
  if (FIndexFile = nil) or VarIsNull(Key) then
  begin
    Result := false;
    exit;
  end;

  // FIndexFile <> nil -> FCursor as TIndexCursor <> nil
  if (TIndexCursor(FCursor).VariantToBuffer(Key, @TempBuffer[0]) = etString) and KeyIsANSI then
    Translate(@TempBuffer[0], @TempBuffer[0], true);
  Result := SearchKeyBuffer(@TempBuffer[0], SearchType);
end;

{$endif}

function  TDbf.PrepareKey(Buffer: Pointer; BufferType: TExpressionType): PChar;
begin
  if FIndexFile = nil then
  begin
    Result := nil;
    exit;
  end;
  
  Result := TIndexCursor(FCursor).IndexFile.PrepareKey(Buffer, BufferType);
end;

function TDbf.SearchKeyPChar(Key: PChar; SearchType: TSearchKeyType; KeyIsANSI: boolean): Boolean;
var
  StringBuf: array [0..100] of Char;
begin
  if FCursor = nil then
  begin
    Result := false;
    exit;
  end;

  if KeyIsANSI then
  begin
    Translate(Key, @StringBuf[0], true);
    Key := @StringBuf[0];
  end;
  Result := SearchKeyBuffer(TIndexCursor(FCursor).CheckUserKey(Key, @StringBuf[0]), SearchType);
end;

function TDbf.SearchKeyBuffer(Buffer: PChar; SearchType: TSearchKeyType): Boolean;
var
  matchRes: Integer;
begin
  if FIndexFile = nil then
  begin
    Result := false;
    exit;
  end;

  CheckBrowseMode;
  Result := FIndexFile.SearchKey(Buffer, SearchType);
  { if found, then retrieve new current record }
  if Result then
  begin
    CursorPosChanged;
    Resync([]);
    UpdateCursorPos;
    { recno could have been changed due to deleted record, check if still matches }
    matchRes := TIndexCursor(FCursor).IndexFile.MatchKey(Buffer);
    case SearchType of
      stEqual:        Result := matchRes =  0;
      stGreater:      Result := (not Eof) and (matchRes <  0);
      stGreaterEqual: Result := (not Eof) and (matchRes <= 0);
    end;
  end;
end;

procedure TDbf.UpdateIndexDefs;
begin
  FieldDefs.Update;
end;

// A hack to upgrade method visibility, only necessary for FPC 1.0.x

{$ifdef VER1_0}

procedure TDbf.DataEvent(Event: TDataEvent; Info: Longint);
begin
  inherited;
end;

{$endif}

{ Master / Detail }

procedure TDbf.CheckMasterRange;
begin
  if FMasterLink.Active and FMasterLink.ValidExpression and (FIndexFile <> nil) then
    UpdateRange;
end;

procedure TDbf.UpdateRange;
var
  fieldsVal: TRecordBuffer;
  tempBuffer: array[0..300] of char;
begin
  fieldsVal := FMasterLink.FieldsVal;
  if (TDbf(FMasterLink.DataSet).DbfFile.UseCodePage <> FDbfFile.UseCodePage)
        and (FMasterLink.Parser.ResultType = etString) then
  begin
    FMasterLink.DataSet.Translate(pansichar(fieldsVal), @tempBuffer[0], false);
    fieldsVal := @tempBuffer[0];
    Translate(pansichar(fieldsVal), pansichar(fieldsVal), true);
  end;
  // preparekey, setrangebuffer and updatekeyfrom* are functions which arguments
  // are not entirely classified in pchar<>trecordbuffer terms.
  // so we typecast for now.
  fieldsVal := TRecordBuffer(TIndexCursor(FCursor).IndexFile.PrepareKey((fieldsVal), FMasterLink.Parser.ResultType));
  SetRangeBuffer(pansichar(fieldsVal), pansichar(fieldsVal)); 
end;

procedure TDbf.MasterChanged(Sender: TObject);
begin
  CheckBrowseMode;
  CheckMasterRange;
end;

procedure TDbf.MasterDisabled(Sender: TObject);
begin
  CancelRange;
end;

function TDbf.GetDataSource: TDataSource;
begin
  Result := FMasterLink.DataSource;
end;

procedure TDbf.SetDataSource(Value: TDataSource);
begin
{$ifndef FPC}
  if IsLinkedTo(Value) then
  begin
{$ifdef DELPHI_4}
    DatabaseError(SCircularDataLink, Self);
{$else}
    DatabaseError(SCircularDataLink);
{$endif}
  end;
{$endif}
  FMasterLink.DataSource := Value;
end;

function TDbf.GetMasterFields: string;
begin
  Result := FMasterLink.FieldNames;
end;

procedure TDbf.SetMasterFields(const Value: string);
begin
  FMasterLink.FieldNames := Value;
end;

//==========================================================
//============ TDbfIndexDefs
//==========================================================
constructor TDbfIndexDefs.Create(AOwner: TDbf);
begin
  inherited Create(TDbfIndexDef);
  FOwner := AOwner;
end;

function TDbfIndexDefs.Add: TDbfIndexDef;
begin
  Result := TDbfIndexDef(inherited Add);
end;

procedure TDbfIndexDefs.SetItem(N: Integer; Value: TDbfIndexDef);
begin
  inherited SetItem(N, Value);
end;

function TDbfIndexDefs.GetItem(N: Integer): TDbfIndexDef;
begin
  Result := TDbfIndexDef(inherited GetItem(N));
end;

function TDbfIndexDefs.GetOwner: tpersistent;
begin
  Result := FOwner;
end;

function TDbfIndexDefs.GetIndexByName(const Name: string): TDbfIndexDef;
var
  I: Integer;
  lIndex: TDbfIndexDef;
begin
  for I := 0 to Count-1 do
  begin
    lIndex := Items[I];
    if lIndex.IndexFile = Name then
    begin
      Result := lIndex;
      exit;
    end
  end;
  Result := nil;
end;

function TDbfIndexDefs.GetIndexByField(const Name: string): TDbfIndexDef;
var
  lIndex: TDbfIndexDef;
  searchStr: string;
  i: integer;
begin
  searchStr := AnsiUpperCase(Trim(Name));
  Result := nil;
  if searchStr = EmptyStr then
    exit;

  for I := 0 to Count-1 do
  begin
    lIndex := Items[I];
    if AnsiUpperCase(Trim(lIndex.SortField)) = searchStr then
    begin
      Result := lIndex;
      exit;
    end
  end;
end;

procedure TDbfIndexDefs.Update;
begin
  if Assigned(FOwner) then
    FOwner.UpdateIndexDefs;
end;

//==========================================================
//============ TDbfMasterLink
//==========================================================

constructor TDbfMasterLink.Create(ADataSet: TDbf);
begin
  inherited Create;

  FDetailDataSet := ADataSet;
  FParser := TDbfParser.Create(nil);
  FValidExpression := false;
end;

destructor TDbfMasterLink.Destroy;
begin
  FParser.Free;

  inherited;
end;

procedure TDbfMasterLink.ActiveChanged;
begin
  if Active and (FFieldNames <> EmptyStr) then
  begin
    FValidExpression := false;
    FParser.DbfFile := (DataSet as TDbf).DbfFile;
    FParser.ParseExpression(FFieldNames);
    FValidExpression := true;
  end else begin
    FParser.ClearExpressions;
    FValidExpression := false;
  end;

  if FDetailDataSet.Active and not (csDestroying in FDetailDataSet.ComponentState) then
    if Active then
    begin
      if Assigned(FOnMasterChange) then FOnMasterChange(Self);
    end else
      if Assigned(FOnMasterDisable) then FOnMasterDisable(Self);
end;

procedure TDbfMasterLink.CheckBrowseMode;
begin
  if FDetailDataSet.Active then
    FDetailDataSet.CheckBrowseMode;
end;

procedure TDbfMasterLink.LayoutChanged;
begin
  ActiveChanged;
end;

procedure TDbfMasterLink.RecordChanged(Field: TField);
begin
  if (DataSource.State <> dsSetKey) and FDetailDataSet.Active and Assigned(FOnMasterChange) then
    FOnMasterChange(Self);
end;

procedure TDbfMasterLink.SetFieldNames(const Value: string);
begin
  if FFieldNames <> Value then
  begin
    FFieldNames := Value;
    ActiveChanged;
  end;
end;

function TDbfMasterLink.GetFieldsVal: TRecordBuffer;
begin
  Result := TRecordBuffer(FParser.ExtractFromBuffer(@pDbfRecord(TDbf(DataSet).ActiveBuffer)^.DeletedFlag));
end;

////////////////////////////////////////////////////////////////////////////

function ApplicationPath: string;
begin
  Result := ExtractFilePath(ParamStr(0));
end;


////////////////////////////////////////////////////////////////////////////

initialization

  DbfBasePath := ApplicationPath;

end.

