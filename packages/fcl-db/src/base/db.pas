{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2014 by Michael Van Canneyt, member of the
    Free Pascal development team


    DB header file with interface section.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit DB;

{$mode objfpc}

{$h+}

interface

uses Classes,SysUtils,Variants,FmtBCD,MaskUtils;

const

  dsMaxBufferCount = MAXINT div 8;
  dsMaxStringSize = 8192;

  // Used in AsBoolean for string fields to determine
  // whether it's true or false.
  YesNoChars : Array[Boolean] of char = ('N', 'Y');

  SQLDelimiterCharacters = [';',',',' ','(',')',#13,#10,#9];

type

{LargeInt}
  LargeInt = Int64;
  PLargeInt= ^LargeInt;

{ Auxiliary type }
  TStringFieldBuffer = Array[0..dsMaxStringSize] of AnsiChar;

{ Misc Dataset types }

  TDataSetState = (dsInactive, dsBrowse, dsEdit, dsInsert, dsSetKey,
    dsCalcFields, dsFilter, dsNewValue, dsOldValue, dsCurValue, dsBlockRead,
    dsInternalCalc, dsOpening, dsRefreshFields);

  TDataEvent = (deFieldChange, deRecordChange, deDataSetChange,
    deDataSetScroll, deLayoutChange, deUpdateRecord, deUpdateState,
    deCheckBrowseMode, dePropertyChange, deFieldListChange, deFocusControl,
    deParentScroll,deConnectChange,deReconcileError,deDisabledStateChange);

  TUpdateStatus = (usUnmodified, usModified, usInserted, usDeleted);
  TUpdateStatusSet = SET OF TUpdateStatus;

  TUpdateMode = (upWhereAll, upWhereChanged, upWhereKeyOnly);
  TResolverResponse = (rrSkip, rrAbort, rrMerge, rrApply, rrIgnore);

  TProviderFlag = (pfInUpdate, pfInWhere, pfInKey, pfHidden, pfRefreshOnInsert,pfRefreshOnUpdate);
  TProviderFlags = set of TProviderFlag;

{ Forward declarations }

  TFieldDef = class;
  TFieldDefs = class;
  TField = class;
  TFields = Class;
  TDataSet = class;
  TDataBase = Class;
  TDataSource = Class;
  TDataLink = Class;
  TDBTransaction = Class;

{ Exception classes }

  EDatabaseError = class(Exception);

  EUpdateError   = class(EDatabaseError)
  private
    FContext           : String;
    FErrorCode         : integer;
    FOriginalException : Exception;
    FPreviousError     : Integer;
  public
    constructor Create(NativeError, Context : String;
      ErrCode, PrevError : integer; E: Exception);
    Destructor Destroy; override;
    property Context : String read FContext;
    property ErrorCode : integer read FErrorcode;
    property OriginalException : Exception read FOriginalException;
    property PreviousError : Integer read FPreviousError;
  end;
  

{ TFieldDef }

  TFieldClass = class of TField;

  // Data type for field.
  // The order is determined by Delphi compatibility
  TFieldType = (ftUnknown, ftString, ftSmallint, ftInteger, ftWord,
    ftBoolean, ftFloat, ftCurrency, ftBCD, ftDate,  ftTime, ftDateTime,
    ftBytes, ftVarBytes, ftAutoInc, ftBlob, ftMemo, ftGraphic, ftFmtMemo,
    ftParadoxOle, ftDBaseOle, ftTypedBinary, ftCursor, ftFixedChar,
    ftWideString, ftLargeint, ftADT, ftArray, ftReference,
    ftDataSet, ftOraBlob, ftOraClob, ftVariant, ftInterface,
    ftIDispatch, ftGuid, ftTimeStamp, ftFMTBcd, ftFixedWideChar, ftWideMemo);

{ Part of DBCommon, but temporarily defined here (bug 8206) }

 TFieldMap = array[TFieldType] of Byte;

{ TDateTimeRec }

  TDateTimeAlias = type TDateTime;
  PDateTimeRec = ^TdateTimeRec;
  TDateTimeRec = record
    case TFieldType of
      ftDate: (Date: Longint);
      ftTime: (Time: Longint);
      ftDateTime: (DateTime: TDateTimeAlias);
  end;

  TFieldAttribute = (faHiddenCol, faReadonly, faRequired, faLink, faUnNamed, faFixed);
  TFieldAttributes = set of TFieldAttribute;

{ TNamedItem }

  TNamedItem = class(TCollectionItem)
  private
    FName: string;
  protected
    function GetDisplayName: string; override;
    procedure SetDisplayName(const AValue: string); override;
  Public  
    property DisplayName : string read GetDisplayName write SetDisplayName;
  published
    property Name : string read FName write SetDisplayName;
  end;

{ TDefCollection }

  TDefCollection = class(TOwnedCollection)
  private
    FDataset: TDataset;
    FUpdated: boolean;
  protected
    procedure SetItemName(AItem: TCollectionItem); override;
  public
    constructor create(ADataset: TDataset; AOwner: TPersistent; AClass: TCollectionItemClass);
    function Find(const AName: string): TNamedItem;
    procedure GetItemNames(List: TStrings);
    function IndexOf(const AName: string): Longint;
    property Dataset: TDataset read FDataset;
    property Updated: boolean read FUpdated write FUpdated;
  end;

{ TFieldDef }

  TFieldDef = class(TNamedItem)
  Private
    FAttributes : TFieldAttributes;
    FCodePage : TSystemCodePage;
    FDataType : TFieldType;
    FFieldNo : Longint;
    FInternalCalcField : Boolean;
    FPrecision : Longint;
    FRequired : Boolean;
    FSize : Integer;
    function GetCharSize: Word;
    Function GetFieldClass : TFieldClass;
    procedure SetAttributes(AValue: TFieldAttributes);
    procedure SetDataType(AValue: TFieldType);
    procedure SetPrecision(const AValue: Longint);
    procedure SetSize(const AValue: Integer);
    procedure SetRequired(const AValue: Boolean);
  public
    constructor Create(ACollection : TCollection); override;
    constructor Create(AOwner: TFieldDefs; const AName: string;
      ADataType: TFieldType; ASize: Integer; ARequired: Boolean; AFieldNo: Longint;
      ACodePage: TSystemCodePage = CP_ACP); overload;
    destructor Destroy; override;
    procedure Assign(APersistent: TPersistent); override;
    function CreateField(AOwner: TComponent): TField;
    property FieldClass: TFieldClass read GetFieldClass;
    property FieldNo: Longint read FFieldNo;
    property CharSize: Word read GetCharSize;
    property InternalCalcField: Boolean read FInternalCalcField write FInternalCalcField;
    property Required: Boolean read FRequired write SetRequired;
    Property Codepage : TSystemCodePage Read FCodePage;
  Published
    property Attributes: TFieldAttributes read FAttributes write SetAttributes default [];
    property DataType: TFieldType read FDataType write SetDataType;
    property Precision: Longint read FPrecision write SetPrecision default 0;
    property Size: Integer read FSize write SetSize default 0;
  end;
  TFieldDefClass = Class of TFieldDef;

{ TFieldDefs }

  TFieldDefs = class(TDefCollection)
  private
    FHiddenFields : Boolean;
    function GetItem(Index: Longint): TFieldDef;
    procedure SetItem(Index: Longint; const AValue: TFieldDef);
  Protected
    Class Function FieldDefClass : TFieldDefClass; virtual;
  public
    constructor Create(ADataSet: TDataSet);
//    destructor Destroy; override;
    Function Add(const AName: string; ADataType: TFieldType; ASize, APrecision: Integer; ARequired, AReadOnly: Boolean; AFieldNo : Integer; ACodePage:TSystemCodePage) : TFieldDef; overload;
    Function Add(const AName: string; ADataType: TFieldType; ASize: Word; ARequired: Boolean; AFieldNo : Integer) : TFieldDef; overload;
    procedure Add(const AName: string; ADataType: TFieldType; ASize: Word; ARequired: Boolean); overload;
    procedure Add(const AName: string; ADataType: TFieldType; ASize: Word); overload;
    procedure Add(const AName: string; ADataType: TFieldType); overload;
    Function AddFieldDef : TFieldDef;
    procedure Assign(FieldDefs: TFieldDefs); overload;
    function Find(const AName: string): TFieldDef;
//    procedure Clear;
//    procedure Delete(Index: Longint);
    procedure Update; overload;
    Function MakeNameUnique(const AName : String) : string; virtual;
    Property HiddenFields : Boolean Read FHiddenFields Write FHiddenFields;
    property Items[Index: Longint]: TFieldDef read GetItem write SetItem; default;
  end;
  TFieldDefsClass = Class of TFieldDefs;

{ TField }

  TFieldKind = (fkData, fkCalculated, fkLookup, fkInternalCalc);
  TFieldKinds = Set of TFieldKind;

  TFieldNotifyEvent = procedure(Sender: TField) of object;
  TFieldGetTextEvent = procedure(Sender: TField; var aText: string;
    DisplayText: Boolean) of object;
  TFieldSetTextEvent = procedure(Sender: TField; const aText: string) of object;
  TFieldRef = ^TField;
  TFieldChars = set of Char;

  PLookupListRec = ^TLookupListRec;
  TLookupListRec = record
    Key: Variant;
    Value: Variant;
  end;

{ TLookupList }

  TLookupList = class(TObject)
  private
    FList: TFPList;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Add(const AKey, AValue: Variant);
    procedure Clear;
    function FirstKeyByValue(const AValue: Variant): Variant;
    function ValueOfKey(const AKey: Variant): Variant;
    procedure ValuesToStrings(AStrings: TStrings);
  end;

{ TField }

  TField = class(TComponent)
  private
    FAlignment : TAlignment;
    FAttributeSet : String;
    FCalculated : Boolean;
    FConstraintErrorMessage : String;
    FCustomConstraint : String;
    FDataSet : TDataSet;
//    FDataSize : Word;
    FDataType : TFieldType;
    FDefaultExpression : String;
    FDisplayLabel : String;
    FDisplayWidth : Longint;
    FEditMask: TEditMask;
    FFieldDef: TFieldDef;
    FFieldKind : TFieldKind;
    FFieldName : String;
    FFieldNo : Longint;
    FFields : TFields;
    FHasConstraints : Boolean;
    FImportedConstraint : String;
    FIsIndexField : Boolean;
    FKeyFields : String;
    FLookupCache : Boolean;
    FLookupDataSet : TDataSet;
    FLookupKeyfields : String;
    FLookupresultField : String;
    FLookupList: TLookupList;
    FOffset : Word;
    FOnChange : TFieldNotifyEvent;
    FOnGetText: TFieldGetTextEvent;
    FOnSetText: TFieldSetTextEvent;
    FOnValidate: TFieldNotifyEvent;
    FOrigin : String;
    FReadOnly : Boolean;
    FRequired : Boolean;
    FSize : integer;
    FValidChars : TFieldChars;
    FValueBuffer : Pointer;
    FValidating : Boolean;
    FVisible : Boolean;
    FProviderFlags : TProviderFlags;
    function GetIndex : longint;
    function GetLookup: Boolean;
    procedure SetAlignment(const AValue: TAlignMent);
    procedure SetIndex(const AValue: Longint);
    function GetDisplayText: String;
    function GetEditText: String;
    procedure SetEditText(const AValue: string);
    procedure SetDisplayLabel(const AValue: string);
    procedure SetDisplayWidth(const AValue: Longint);
    function GetDisplayWidth: integer;
    procedure SetLookup(const AValue: Boolean);
    procedure SetReadOnly(const AValue: Boolean);
    procedure SetVisible(const AValue: Boolean);
    function IsDisplayLabelStored : Boolean;
    function IsDisplayWidthStored: Boolean;
    function GetLookupList: TLookupList;
    procedure CalcLookupValue;
  protected
    function AccessError(const TypeName: string): EDatabaseError;
    procedure CheckInactive;
    class procedure CheckTypeSize(AValue: Longint); virtual;
    procedure Change; virtual;
    procedure Bind(Binding: Boolean); virtual;
    procedure DataChanged;
    procedure FreeBuffers; virtual;
    function GetAsBCD: TBCD; virtual;
    function GetAsBoolean: Boolean; virtual;
    function GetAsBytes: TBytes; virtual;
    function GetAsCurrency: Currency; virtual;
    function GetAsLargeInt: Largeint; virtual;
    function GetAsDateTime: TDateTime; virtual;
    function GetAsFloat: Double; virtual;
    function GetAsLongint: Longint; virtual;
    function GetAsInteger: Longint; virtual;
    function GetAsVariant: variant; virtual;
    function GetOldValue: variant; virtual;
    function GetAsString: string; virtual;
    function GetAsAnsiString: AnsiString; virtual;
    function GetAsUnicodeString: UnicodeString; virtual;
    function GetAsUTF8String: UTF8String; virtual;
    function GetAsWideString: WideString; virtual;
    function GetCanModify: Boolean; virtual;
    function GetClassDesc: String; virtual;
    function GetDataSize: Integer; virtual;
    function GetDefaultWidth: Longint; virtual;
    function GetDisplayName : String;
    function GetCurValue: Variant; virtual;
    function GetNewValue: Variant; virtual;
    function GetIsNull: Boolean; virtual;
    function GetParentComponent: TComponent; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); virtual;
    function HasParent: Boolean; override;
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    procedure PropertyChanged(LayoutAffected: Boolean);
    procedure ReadState(Reader: TReader); override;
    procedure SetAsBCD(const AValue: TBCD); virtual;
    procedure SetAsBoolean(AValue: Boolean); virtual;
    procedure SetAsBytes(const AValue: TBytes); virtual;
    procedure SetAsCurrency(AValue: Currency); virtual;
    procedure SetAsDateTime(AValue: TDateTime); virtual;
    procedure SetAsFloat(AValue: Double); virtual;
    procedure SetAsLongint(AValue: Longint); virtual;
    procedure SetAsInteger(AValue: Longint); virtual;
    procedure SetAsLargeInt(AValue: Largeint); virtual;
    procedure SetAsVariant(const AValue: variant); virtual;
    procedure SetAsString(const AValue: string); virtual;
    procedure SetAsAnsiString(const AValue: AnsiString); virtual;
    procedure SetAsUnicodeString(const AValue: UnicodeString); virtual;
    procedure SetAsUTF8String(const AValue: UTF8String); virtual;
    procedure SetAsWideString(const AValue: WideString); virtual;
    procedure SetDataset(AValue : TDataset); virtual;
    procedure SetDataType(AValue: TFieldType);
    procedure SetNewValue(const AValue: Variant);
    procedure SetSize(AValue: Integer); virtual;
    procedure SetParentComponent(AParent: TComponent); override;
    procedure SetText(const AValue: string); virtual;
    procedure SetVarValue(const AValue: Variant); virtual;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure Assign(Source: TPersistent); override;
    procedure AssignValue(const AValue: TVarRec);
    procedure Clear; virtual;
    procedure FocusControl;
    function GetData(Buffer: Pointer): Boolean; overload;
    function GetData(Buffer: Pointer; NativeFormat : Boolean): Boolean; overload;
    class function IsBlob: Boolean; virtual;
    function IsValidChar(InputChar: Char): Boolean; virtual;
    procedure RefreshLookupList;
    procedure SetData(Buffer: Pointer); overload;
    procedure SetData(Buffer: Pointer; NativeFormat : Boolean); overload;
    procedure SetFieldType(AValue: TFieldType); virtual;
    procedure Validate(Buffer: Pointer);
    property AsBCD: TBCD read GetAsBCD write SetAsBCD;
    property AsBoolean: Boolean read GetAsBoolean write SetAsBoolean;
    property AsBytes: TBytes read GetAsBytes write SetAsBytes;
    property AsCurrency: Currency read GetAsCurrency write SetAsCurrency;
    property AsDateTime: TDateTime read GetAsDateTime write SetAsDateTime;
    property AsFloat: Double read GetAsFloat write SetAsFloat;
    property AsLongint: Longint read GetAsLongint write SetAsLongint;
    property AsLargeInt: LargeInt read GetAsLargeInt write SetAsLargeInt;
    property AsInteger: Longint read GetAsInteger write SetAsInteger;
    property AsString: string read GetAsString write SetAsString;
    property AsAnsiString: AnsiString read GetAsAnsiString write SetAsAnsiString;
    property AsUnicodeString: UnicodeString read GetAsUnicodeString write SetAsUnicodeString;
    property AsUTF8String: UTF8String read GetAsUTF8String write SetAsUTF8String;
    property AsWideString: WideString read GetAsWideString write SetAsWideString;
    property AsVariant: variant read GetAsVariant write SetAsVariant;
    property AttributeSet: string read FAttributeSet write FAttributeSet;
    property Calculated: Boolean read FCalculated write FCalculated;
    property CanModify: Boolean read GetCanModify;
    property CurValue: Variant read GetCurValue;
    property DataSet: TDataSet read FDataSet write SetDataSet;
    property DataSize: Integer read GetDataSize;
    property DataType: TFieldType read FDataType;
    property DisplayName: String Read GetDisplayName;
    property DisplayText: String read GetDisplayText;
    property EditMask: TEditMask read FEditMask write FEditMask;
    property EditMaskPtr: TEditMask read FEditMask;
    property FieldNo: Longint read FFieldNo;
    property IsIndexField: Boolean read FIsIndexField;
    property IsNull: Boolean read GetIsNull;
    property NewValue: Variant read GetNewValue write SetNewValue;
    property Offset: word read FOffset;
    property Size: Integer read FSize write SetSize;
    property Text: string read GetEditText write SetEditText;
    property ValidChars : TFieldChars read FValidChars write FValidChars;
    property Value: variant read GetAsVariant write SetAsVariant;
    property OldValue: variant read GetOldValue;
    property LookupList: TLookupList read GetLookupList;
    Property FieldDef : TFieldDef Read FFieldDef;
  published
    property Alignment : TAlignment read FAlignment write SetAlignment default taLeftJustify;
    property CustomConstraint: string read FCustomConstraint write FCustomConstraint;
    property ConstraintErrorMessage: string read FConstraintErrorMessage write FConstraintErrorMessage;
    property DefaultExpression: string read FDefaultExpression write FDefaultExpression;
    property DisplayLabel : string read GetDisplayName write SetDisplayLabel stored IsDisplayLabelStored;
    property DisplayWidth: Longint read GetDisplayWidth write SetDisplayWidth stored IsDisplayWidthStored;
    property FieldKind: TFieldKind read FFieldKind write FFieldKind;
    property FieldName: string read FFieldName write FFieldName;
    property HasConstraints: Boolean read FHasConstraints;
    property Index: Longint read GetIndex write SetIndex;
    property ImportedConstraint: string read FImportedConstraint write FImportedConstraint;
    property KeyFields: string read FKeyFields write FKeyFields;
    property LookupCache: Boolean read FLookupCache write FLookupCache;
    property LookupDataSet: TDataSet read FLookupDataSet write FLookupDataSet;
    property LookupKeyFields: string read FLookupKeyFields write FLookupKeyFields;
    property LookupResultField: string read FLookupResultField write FLookupResultField;
    property Lookup: Boolean read GetLookup write SetLookup stored false; deprecated;
    property Origin: string read FOrigin write FOrigin;
    property ProviderFlags : TProviderFlags read FProviderFlags write FProviderFlags;
    property ReadOnly: Boolean read FReadOnly write SetReadOnly;
    property Required: Boolean read FRequired write FRequired;
    property Visible: Boolean read FVisible write SetVisible default True;
    property OnChange: TFieldNotifyEvent read FOnChange write FOnChange;
    property OnGetText: TFieldGetTextEvent read FOnGetText write FOnGetText;
    property OnSetText: TFieldSetTextEvent read FOnSetText write FOnSetText;
    property OnValidate: TFieldNotifyEvent read FOnValidate write FOnValidate;
  end;

{ TStringField }

  TStringField = class(TField)
  private
    FCodePage      : TSystemCodePage;
    FFixedChar     : boolean;
    FTransliterate : Boolean;
  protected
    class procedure CheckTypeSize(AValue: Longint); override;
    function GetAsBoolean: Boolean; override;
    function GetAsDateTime: TDateTime; override;
    function GetAsFloat: Double; override;
    function GetAsInteger: Longint; override;
    function GetAsLargeInt: Largeint; override;
    function GetAsString: String; override;
    function GetAsAnsiString: AnsiString; override;
    function GetAsUTF8String: UTF8String; override;
    function GetAsVariant: variant; override;
    function GetDataSize: Integer; override;
    function GetDefaultWidth: Longint; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    function GetValue(out AValue: RawByteString): Boolean;
    procedure SetAsBoolean(AValue: Boolean); override;
    procedure SetAsDateTime(AValue: TDateTime); override;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetAsLargeInt(AValue: Largeint); override;
    procedure SetAsString(const AValue: String); override;
    procedure SetAsAnsiString(const AValue: AnsiString); override;
    procedure SetAsUTF8String(const AValue: UTF8String); override;
    procedure SetVarValue(const AValue: Variant); override;
    procedure SetValue(AValue: RawByteString);
  public
    constructor Create(AOwner: TComponent); override;
    procedure SetFieldType(AValue: TFieldType); override;
    property CodePage : TSystemCodePage read FCodePage;
    property FixedChar : Boolean read FFixedChar write FFixedChar;
    property Transliterate: Boolean read FTransliterate write FTransliterate;
    property Value: String read GetAsString write SetAsString;
  published
    property EditMask;
    property Size default 20;
  end;

{ TWideStringField }

  TWideStringField = class(TStringField)
  protected
    class procedure CheckTypeSize(AValue: Integer); override;

    function GetValue(out AValue: UnicodeString): Boolean;

    function GetAsString: string; override;
    procedure SetAsString(const AValue: string); override;

    function GetAsVariant: Variant; override;
    procedure SetVarValue(const AValue: Variant); override;

    function GetAsWideString: WideString; override;
    procedure SetAsWideString(const AValue: WideString); override;

    function GetAsUnicodeString: UnicodeString; override;
    procedure SetAsUnicodeString(const AValue: UnicodeString); override;

    function GetAsUTF8String: UTF8String; override;
    procedure SetAsUTF8String(const AValue: UTF8String); override;

    function GetDataSize: Integer; override;
  public
    constructor Create(AOwner: TComponent); override;
    procedure SetFieldType(AValue: TFieldType); override;
    property Value: WideString read GetAsWideString write SetAsWideString;
  end;

{ TNumericField }

  TNumericField = class(TField)
  Private
    FDisplayFormat : String;
    FEditFormat : String;
  protected
    class procedure CheckTypeSize(AValue: Longint); override;
    procedure RangeError(AValue, Min, Max: Double);
    procedure SetDisplayFormat(const AValue: string);
    procedure SetEditFormat(const AValue: string);
    function  GetAsBoolean: Boolean; override;
    Procedure SetAsBoolean(AValue: Boolean); override;
  public
    constructor Create(AOwner: TComponent); override;
  published
    property Alignment default taRightJustify;
    property DisplayFormat: string read FDisplayFormat write SetDisplayFormat;
    property EditFormat: string read FEditFormat write SetEditFormat;
  end;

{ TLongintField }

  TLongintField = class(TNumericField)
  private
    FMinValue,
    FMaxValue,
    FMinRange,
    FMaxRange  : Longint;
    Procedure SetMinValue (AValue : longint);
    Procedure SetMaxValue (AValue : longint);
  protected
    function GetAsFloat: Double; override;
    function GetAsInteger: Longint; override;
    function GetAsString: string; override;
    function GetAsVariant: variant; override;
    function GetDataSize: Integer; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    function GetValue(var AValue: Longint): Boolean;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetVarValue(const AValue: Variant); override;
    function GetAsLargeInt: Largeint; override;
    procedure SetAsLargeInt(AValue: Largeint); override;
  public
    constructor Create(AOwner: TComponent); override;
    Function CheckRange(AValue : Longint) : Boolean;
    property Value: Longint read GetAsInteger write SetAsInteger;
  published
    property MaxValue: Longint read FMaxValue write SetMaxValue default 0;
    property MinValue: Longint read FMinValue write SetMinValue default 0;
  end;
  TIntegerField = Class(TLongintField);

{ TLargeintField }

  TLargeintField = class(TNumericField)
  private
    FMinValue,
    FMaxValue,
    FMinRange,
    FMaxRange  : Largeint;
    Procedure SetMinValue (AValue : Largeint);
    Procedure SetMaxValue (AValue : Largeint);
  protected
    function GetAsFloat: Double; override;
    function GetAsInteger: Longint; override;
    function GetAsLargeInt: Largeint; override;
    function GetAsString: string; override;
    function GetAsVariant: variant; override;
    function GetDataSize: Integer; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    function GetValue(var AValue: Largeint): Boolean;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetAsLargeInt(AValue: Largeint); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    Function CheckRange(AValue : Largeint) : Boolean;
    property Value: Largeint read GetAsLargeInt write SetAsLargeInt;
  published
    property MaxValue: Largeint read FMaxValue write SetMaxValue default 0;
    property MinValue: Largeint read FMinValue write SetMinValue default 0;
  end;

{ TSmallintField }

  TSmallintField = class(TLongintField)
  protected
    function GetDataSize: Integer; override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TWordField }

  TWordField = class(TLongintField)
  protected
    function GetDataSize: Integer; override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TAutoIncField }

  TAutoIncField = class(TLongintField)
  Protected
    procedure SetAsInteger(AValue: Longint); override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TFloatField }

  TFloatField = class(TNumericField)
  private
    FCurrency: Boolean;
    FMaxValue : Double;
    FMinValue : Double;
    FPrecision : Longint;
    procedure SetCurrency(const AValue: Boolean);
    procedure SetPrecision(const AValue: Longint);
  protected
    function GetAsBCD: TBCD; override;
    function GetAsFloat: Double; override;
    function GetAsLargeInt: LargeInt; override;
    function GetAsInteger: Longint; override;
    function GetAsVariant: variant; override;
    function GetAsString: string; override;
    function GetDataSize: Integer; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    procedure SetAsBCD(const AValue: TBCD); override;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsLargeInt(AValue: LargeInt); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    Function CheckRange(AValue : Double) : Boolean;
    property Value: Double read GetAsFloat write SetAsFloat;

  published
    property Currency: Boolean read FCurrency write SetCurrency default False;
    property MaxValue: Double read FMaxValue write FMaxValue;
    property MinValue: Double read FMinValue write FMinValue;
    property Precision: Longint read FPrecision write SetPrecision default 15; // min 2 instellen, delphi compat
  end;

{ TCurrencyField }

  TCurrencyField = class(TFloatField)
  public
    constructor Create(AOwner: TComponent); override;
  published
    property Currency default True;
  end;

{ TBooleanField }

  TBooleanField = class(TField)
  private
    FDisplayValues : String;
    // First byte indicates uppercase or not.
    FDisplays : Array[Boolean,Boolean] of string;
    Procedure SetDisplayValues(const AValue : String);
  protected
    function GetAsBoolean: Boolean; override;
    function GetAsString: string; override;
    function GetAsVariant: variant; override;
    function GetAsInteger: Longint; override;
    function GetDataSize: Integer; override;
    function GetDefaultWidth: Longint; override;
    procedure SetAsBoolean(AValue: Boolean); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    property Value: Boolean read GetAsBoolean write SetAsBoolean;
  published
    property DisplayValues: string read FDisplayValues write SetDisplayValues;
  end;

{ TDateTimeField }

  TDateTimeField = class(TField)
  private
    FDisplayFormat : String;
    procedure SetDisplayFormat(const AValue: string);
  protected
    function GetAsDateTime: TDateTime; override;
    function GetAsFloat: Double; override;
    function GetAsString: string; override;
    function GetAsVariant: variant; override;
    function GetDataSize: Integer; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    procedure SetAsDateTime(AValue: TDateTime); override;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    property Value: TDateTime read GetAsDateTime write SetAsDateTime;
  published
    property DisplayFormat: string read FDisplayFormat write SetDisplayFormat;
    property EditMask;
  end;

{ TDateField }

  TDateField = class(TDateTimeField)
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TTimeField }

  TTimeField = class(TDateTimeField)
  protected
    procedure SetAsString(const AValue: string); override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TBinaryField }

  TBinaryField = class(TField)
  protected
    class procedure CheckTypeSize(AValue: Longint); override;
    function GetAsBytes: TBytes; override;
    function GetAsString: string; override;
    function GetAsVariant: Variant; override;
    function GetValue(var AValue: TBytes): Boolean;
    procedure SetAsBytes(const AValue: TBytes); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
  published
    property Size default 16;
  end;

{ TBytesField }

  TBytesField = class(TBinaryField)
  protected
    function GetDataSize: Integer; override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TVarBytesField }

  TVarBytesField = class(TBytesField)
  protected
    function GetDataSize: Integer; override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TBCDField }

  TBCDField = class(TNumericField)
  private
    FMinValue,
    FMaxValue   : currency;
    FPrecision  : Longint;
    FCurrency   : boolean;
  protected
    class procedure CheckTypeSize(AValue: Longint); override;
    function GetAsBCD: TBCD; override;
    function GetAsCurrency: Currency; override;
    function GetAsFloat: Double; override;
    function GetAsInteger: Longint; override;
    function GetAsString: string; override;
    function GetValue(var AValue: Currency): Boolean;
    function GetAsVariant: variant; override;
    function GetDataSize: Integer; override;
    function GetDefaultWidth: Longint; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    procedure SetAsBCD(const AValue: TBCD); override;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetAsCurrency(AValue: Currency); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    Function CheckRange(AValue : Currency) : Boolean;
    property Value: Currency read GetAscurrency write SetAscurrency;
  published
    property Precision: Longint read FPrecision write FPrecision;
    property Currency: Boolean read FCurrency write FCurrency;
    property MaxValue: Currency read FMaxValue write FMaxValue;
    property MinValue: Currency read FMinValue write FMinValue;
    property Size default 4;
  end;

{ TFMTBCDField }

  TFMTBCDField = class(TNumericField)
  private
    FMinValue,
    FMaxValue   : TBCD;
    FPrecision  : Longint;
    FCurrency   : boolean;
    function GetMaxValue: string;
    function GetMinValue: string;
    procedure SetMaxValue(const AValue: string);
    procedure SetMinValue(const AValue: string);
  protected
    class procedure CheckTypeSize(AValue: Longint); override;
    function GetAsBCD: TBCD; override;
    function GetAsCurrency: Currency; override;
    function GetAsFloat: Double; override;
    function GetAsLargeInt: LargeInt; override;
    function GetAsInteger: Longint; override;
    function GetAsString: string; override;
    function GetAsVariant: variant; override;
    function GetDataSize: Integer; override;
    function GetDefaultWidth: Longint; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    procedure SetAsBCD(const AValue: TBCD); override;
    procedure SetAsFloat(AValue: Double); override;
    procedure SetAsLargeInt(AValue: LargeInt); override;
    procedure SetAsInteger(AValue: Longint); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetAsCurrency(AValue: Currency); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    function CheckRange(AValue : TBCD) : Boolean;
    property Value: TBCD read GetAsBCD write SetAsBCD;
  published
    property Precision: Longint read FPrecision write FPrecision default 18;
    property Currency: Boolean read FCurrency write FCurrency;
    property MaxValue: string read GetMaxValue write SetMaxValue;
    property MinValue: string read GetMinValue write SetMinValue;
    property Size default 4;
  end;


{ TBlobField }
  TBlobStreamMode = (bmRead, bmWrite, bmReadWrite);
  // This type is needed for compatibility. While it should contain only blob
  // types, it actually does not.
  // Instead of this, please use ftBlobTypes
  TBlobType = ftBlob..ftWideMemo deprecated 'Warning: Does not contain BLOB types. Please use ftBlobTypes.';

  TBlobField = class(TField)
  private
    FModified : Boolean;
    FTransliterate : Boolean;
    Function GetBlobStream (Mode : TBlobStreamMode) : TStream;
    // Wrapper that retrieves FDataType as a TBlobType
    function GetBlobType: TBlobType;
    // Wrapper that calls SetFieldType
    procedure SetBlobType(AValue: TBlobType);
  protected
    procedure FreeBuffers; override;
    function GetAsBytes: TBytes; override;
    function GetAsString: string; override;
    function GetAsAnsiString: AnsiString; override;
    function GetAsUnicodeString: UnicodeString; override;
    function GetAsVariant: Variant; override;
    function GetBlobSize: Longint; virtual;
    function GetIsNull: Boolean; override;
    procedure GetText(var AText: string; ADisplayText: Boolean); override;
    procedure SetAsBytes(const AValue: TBytes); override;
    procedure SetAsString(const AValue: string); override;
    procedure SetAsAnsiString(const AValue: AnsiString); override;
    procedure SetAsUnicodeString(const AValue: UnicodeString); override;
    procedure SetVarValue(const AValue: Variant); override;
  public
    constructor Create(AOwner: TComponent); override;
    procedure Clear; override;
    class function IsBlob: Boolean; override;
    procedure LoadFromFile(const FileName: string);
    procedure LoadFromStream(Stream: TStream);
    procedure SaveToFile(const FileName: string);
    procedure SaveToStream(Stream: TStream);
    procedure SetFieldType(AValue: TFieldType); override;
    property BlobSize: Longint read GetBlobSize;
    property Modified: Boolean read FModified write FModified;
    property Value: string read GetAsString write SetAsString;
    property Transliterate: Boolean read FTransliterate write FTransliterate;
  published
    property BlobType: TBlobType read GetBlobType write SetBlobType default ftBlob;
    property Size default 0;
  end;

{ TMemoField }

  TMemoField = class(TBlobField)
  private
    FCodePage: TSystemCodePage;
  protected
    function GetAsAnsiString: AnsiString; override;
    procedure SetAsAnsiString(const AValue: AnsiString); override;
    function GetAsUnicodeString: UnicodeString; override;
    procedure SetAsUnicodeString(const AValue: UnicodeString); override;
    function GetAsUTF8String: UTF8String; override;
    procedure SetAsUTF8String(const AValue: UTF8String); override;
  public
    constructor Create(AOwner: TComponent); override;
    property CodePage : TSystemCodePage read FCodePage;
  published
    property Transliterate default True;
  end;

{ TWideMemoField }

  TWideMemoField = class(TBlobField)
  protected
    function GetAsVariant: Variant; override;
    procedure SetVarValue(const AValue: Variant); override;
    function GetAsString: string; override;
    procedure SetAsString(const AValue: string); override;
    function GetAsAnsiString: AnsiString; override;
    procedure SetAsAnsiString(const AValue: AnsiString); override;
    function GetAsUTF8String: UTF8String; override;
    procedure SetAsUTF8String(const AValue: UTF8String); override;
  public
    constructor Create(aOwner: TComponent); override;
    property Value: WideString read GetAsWideString write SetAsWideString;
  published
  end;


{ TGraphicField }

  TGraphicField = class(TBlobField)
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TVariantField }

  TVariantField = class(TField)
  protected
    class procedure CheckTypeSize(aValue: Integer); override;

    function GetAsBoolean: Boolean; override;
    procedure SetAsBoolean(aValue: Boolean); override;

    function GetAsDateTime: TDateTime; override;
    procedure SetAsDateTime(aValue: TDateTime); override;

    function GetAsFloat: Double; override;
    procedure SetAsFloat(aValue: Double); override;

    function GetAsInteger: Longint; override;
    procedure SetAsInteger(AValue: Longint); override;

    function GetAsString: string; override;
    procedure SetAsString(const aValue: string); override;

    function GetAsWideString: WideString; override;
    procedure SetAsWideString(const aValue: WideString); override;

    function GetAsVariant: Variant; override;
    procedure SetVarValue(const aValue: Variant); override;

    function GetDefaultWidth: Integer; override;
  public
    constructor Create(AOwner: TComponent); override;
  end;

{ TGuidField }

  TGuidField = class(TStringField)
  protected
    class procedure CheckTypeSize(AValue: Longint); override;
    function GetDefaultWidth: Longint; override;

    function GetAsGuid: TGUID;
    procedure SetAsGuid(const AValue: TGUID);
  public
    constructor Create(AOwner: TComponent); override;
    property AsGuid: TGUID read GetAsGuid write SetAsGuid;
  end;

{ TIndexDef }

  TIndexDefs = class;

  TIndexOption = (ixPrimary, ixUnique, ixDescending, ixCaseInsensitive,
    ixExpression, ixNonMaintained);
  TIndexOptions = set of TIndexOption;

  TIndexDef = class(TNamedItem)
  Private
    FCaseinsFields: string;
    FDescFields: string;
    FExpression : String;
    FFields : String;
    FOptions : TIndexOptions;
    FSource : String;
  protected
    procedure Assign(Source: TPersistent); override;
    function GetExpression: string;
    procedure SetCaseInsFields(const AValue: string); virtual;
    procedure SetDescFields(const AValue: string);
    procedure SetExpression(const AValue: string);
  public
    constructor Create(Owner: TIndexDefs; const AName, TheFields: string;
      TheOptions: TIndexOptions); overload;
  published    
    property Expression: string read GetExpression write SetExpression;
    property Fields: string read FFields write FFields;
    property CaseInsFields: string read FCaseinsFields write SetCaseInsFields;
    property DescFields: string read FDescFields write SetDescFields;
    property Options: TIndexOptions read FOptions write FOptions;
    property Source: string read FSource write FSource;
  end;

{ TIndexDefs }

  TIndexDefs = class(TDefCollection)
  Private
    Function  GetItem(Index: Integer): TIndexDef;
    Procedure SetItem(Index: Integer; Value: TIndexDef);
  public
    constructor Create(ADataSet: TDataSet); virtual; overload;
    procedure Add(const Name, Fields: string; Options: TIndexOptions); overload;
    Function AddIndexDef: TIndexDef;
    function Find(const IndexName: string): TIndexDef;
    function FindIndexForFields(const Fields: string): TIndexDef;
    function GetIndexForFields(const Fields: string;
      CaseInsensitive: Boolean): TIndexDef;
    procedure Update; overload; virtual;
    Property Items[Index: Integer] : TIndexDef read GetItem write SetItem; default;
  end;

{ TCheckConstraint }

  TCheckConstraint = class(TCollectionItem)
  Private
    FCustomConstraint : String;
    FErrorMessage : String;
    FFromDictionary : Boolean;
    FImportedConstraint : String;
  public
    procedure Assign(Source: TPersistent); override;
  //  function GetDisplayName: string; override;
  published
    property CustomConstraint: string read FCustomConstraint write FCustomConstraint;
    property ErrorMessage: string read FErrorMessage write FErrorMessage;
    property FromDictionary: Boolean read FFromDictionary write FFromDictionary;
    property ImportedConstraint: string read FImportedConstraint write FImportedConstraint;
  end;

{ TCheckConstraints }

  TCheckConstraints = class(TCollection)
  Private
   Function GetItem(Index : Longint) : TCheckConstraint;
   Procedure SetItem(index : Longint; Value : TCheckConstraint);
  protected
    function GetOwner: TPersistent; override;
  public
    constructor Create(AOwner: TPersistent);
    function Add: TCheckConstraint;
    property Items[Index: Longint]: TCheckConstraint read GetItem write SetItem; default;
  end;

  { TFieldsEnumerator }

  TFieldsEnumerator = class
  private
    FPosition: Integer;
    FFields: TFields;
    function GetCurrent: TField;
  public
    constructor Create(AFields: TFields);
    function MoveNext: Boolean;
    property Current: TField read GetCurrent;
  end;

{ TFields }

  TFields = Class(TObject)
    Private
      FDataset : TDataset;
      FFieldList : TFpList;
      FOnChange : TNotifyEvent;
      FValidFieldKinds : TFieldKinds;
    Protected
      Procedure ClearFieldDefs;
      Procedure Changed;
      Procedure CheckfieldKind(Fieldkind : TFieldKind; Field : TField);
      Function GetCount : Longint;
      Function GetField (Index : Integer) : TField;
      Procedure SetField(Index: Integer; Value: TField);
      Procedure SetFieldIndex (Field : TField;Value : Integer);
      Property OnChange : TNotifyEvent Read FOnChange Write FOnChange;
      Property ValidFieldKinds : TFieldKinds Read FValidFieldKinds;
    Public
      Constructor Create(ADataset : TDataset);
      Destructor Destroy;override;
      Procedure Add(Field : TField);
      Procedure CheckFieldName (Const Value : String);
      Procedure CheckFieldNames (Const Value : String);
      Procedure Clear;
      Function FindField (Const Value : String) : TField;
      Function FieldByName (Const Value : String) : TField;
      Function FieldByNumber(FieldNo : Integer) : TField;
      Function GetEnumerator: TFieldsEnumerator;
      Procedure GetFieldNames (Values : TStrings);
      Function IndexOf(Field : TField) : Longint;
      procedure Remove(Value : TField);
      Property Count : Integer Read GetCount;
      Property Dataset : TDataset Read FDataset;
      Property Fields [Index : Integer] : TField Read GetField Write SetField; default;
    end;
  TFieldsClass = Class of TFields;

{ TParam }

  TBlobData = TBytes;

  TParamBinding = array of integer;

  TParamType = (ptUnknown, ptInput, ptOutput, ptInputOutput, ptResult);
  TParamTypes = set of TParamType;

  TParamStyle = (psInterbase,psPostgreSQL,psSimulated);

  TParams = class;

  TParam = class(TCollectionItem)
  private
    FNativeStr: string;
    FValue: Variant;
    FPrecision: Integer;
    FNumericScale: Integer;
    FName: string;
    FDataType: TFieldType;
    FBound: Boolean;
    FParamType: TParamType;
    FSize: Integer;
    Function GetDataSet: TDataSet;
    Function IsParamStored: Boolean;
  protected
    Procedure AssignParam(Param: TParam);
    Procedure AssignTo(Dest: TPersistent); override;
    Function GetAsBoolean: Boolean;
    Function GetAsBytes: TBytes;
    Function GetAsCurrency: Currency;
    Function GetAsDateTime: TDateTime;
    Function GetAsFloat: Double;
    Function GetAsInteger: Longint;
    Function GetAsLargeInt: LargeInt;
    Function GetAsMemo: string;
    Function GetAsString: string;
    Function GetAsAnsiString: AnsiString;
    Function GetAsUnicodeString: UnicodeString;
    Function GetAsUTF8String: UTF8String;
    Function GetAsWideString: WideString;
    Function GetAsVariant: Variant;
    Function GetAsFMTBCD: TBCD;
    Function GetDisplayName: string; override;
    Function GetIsNull: Boolean;
    Function IsEqual(AValue: TParam): Boolean;
    Procedure SetAsBCD(const AValue: Currency);
    Procedure SetAsBlob(const AValue: TBlobData);
    Procedure SetAsBoolean(AValue: Boolean);
    Procedure SetAsBytes(const AValue: TBytes);
    Procedure SetAsCurrency(const AValue: Currency);
    Procedure SetAsDate(const AValue: TDateTime);
    Procedure SetAsDateTime(const AValue: TDateTime);
    Procedure SetAsFloat(const AValue: Double);
    Procedure SetAsInteger(AValue: Longint);
    Procedure SetAsLargeInt(AValue: LargeInt);
    Procedure SetAsMemo(const AValue: string);
    Procedure SetAsSmallInt(AValue: LongInt);
    Procedure SetAsString(const AValue: string);
    Procedure SetAsAnsiString(const AValue: AnsiString);
    Procedure SetAsUTF8String(const AValue: UTF8String);
    Procedure SetAsUnicodeString(const AValue: UnicodeString);
    Procedure SetAsWideString(const AValue: WideString);
    Procedure SetAsTime(const AValue: TDateTime);
    Procedure SetAsVariant(const AValue: Variant);
    Procedure SetAsWord(AValue: LongInt);
    Procedure SetAsFMTBCD(const AValue: TBCD);
    Procedure SetDataType(AValue: TFieldType);
    Procedure SetText(const AValue: string);
  public
    constructor Create(ACollection: TCollection); overload; override;
    constructor Create(AParams: TParams; AParamType: TParamType); reintroduce; overload;
    Procedure Assign(Source: TPersistent); override;
    Procedure AssignField(Field: TField);
    Procedure AssignToField(Field: TField);
    Procedure AssignFieldValue(Field: TField; const AValue: Variant);
    Procedure AssignFromField(Field : TField);
    Procedure Clear;
    Procedure GetData(Buffer: Pointer);
    Function  GetDataSize: Integer;
    Procedure LoadFromFile(const FileName: string; BlobType: TBlobType);
    Procedure LoadFromStream(Stream: TStream; BlobType: TBlobType);
    Procedure SetBlobData(Buffer: Pointer; ASize: Integer);
    Procedure SetData(Buffer: Pointer);
    Property AsBCD : Currency read GetAsCurrency write SetAsBCD;
    Property AsBlob : TBlobData read GetAsBytes write SetAsBlob;
    Property AsBoolean : Boolean read GetAsBoolean write SetAsBoolean;
    Property AsBytes : TBytes read GetAsBytes write SetAsBytes;
    Property AsCurrency : Currency read GetAsCurrency write SetAsCurrency;
    Property AsDate : TDateTime read GetAsDateTime write SetAsDate;
    Property AsDateTime : TDateTime read GetAsDateTime write SetAsDateTime;
    Property AsFloat : Double read GetAsFloat write SetAsFloat;
    Property AsInteger : LongInt read GetAsInteger write SetAsInteger;
    Property AsLargeInt : LargeInt read GetAsLargeInt write SetAsLargeInt;
    Property AsMemo : string read GetAsMemo write SetAsMemo;
    Property AsSmallInt : LongInt read GetAsInteger write SetAsSmallInt;
    Property AsString : string read GetAsString write SetAsString;
    Property AsAnsiString : AnsiString read GetAsAnsiString write SetAsAnsiString;
    Property AsUTF8String: UTF8String read GetAsUTF8String write SetAsUTF8String;
    Property AsUnicodeString: UnicodeString read GetAsUnicodeString write SetAsUnicodeString;
    Property AsTime : TDateTime read GetAsDateTime write SetAsTime;
    Property AsWord : LongInt read GetAsInteger write SetAsWord;
    Property AsFMTBCD: TBCD read GetAsFMTBCD write SetAsFMTBCD;
    Property Bound : Boolean read FBound write FBound;
    Property Dataset : TDataset Read GetDataset;
    Property IsNull : Boolean read GetIsNull;
    Property NativeStr : string read FNativeStr write FNativeStr;
    Property Text : string read GetAsString write SetText;
    property AsWideString: WideString read GetAsWideString write SetAsWideString;
  published
    Property DataType : TFieldType read FDataType write SetDataType;
    Property Name : string read FName write FName;
    Property NumericScale : Integer read FNumericScale write FNumericScale default 0;
    Property ParamType : TParamType read FParamType write FParamType;
    Property Precision : Integer read FPrecision write FPrecision default 0;
    Property Size : Integer read FSize write FSize default 0;
    Property Value : Variant read GetAsVariant write SetAsVariant stored IsParamStored;
  end;
  TParamClass = Class of TParam;

  { TParamsEnumerator }

  TParamsEnumerator = class
  private
    FPosition: Integer;
    FParams: TParams;
    function GetCurrent: TParam;
  public
    constructor Create(AParams: TParams);
    function MoveNext: Boolean;
    property Current: TParam read GetCurrent;
  end;

{ TParams }
  TSQLParseOption = (spoCreate,spoEscapeSlash,spoEscapeRepeat,spoUseMacro);
  TSQLParseOptions = Set of TSQLParseOption;

  TParams = class(TCollection)
  private
    FOwner: TPersistent;
    Function  GetItem(Index: Integer): TParam;
    Function  GetParamValue(const ParamName: string): Variant;
    Procedure SetItem(Index: Integer; Value: TParam);
    Procedure SetParamValue(const ParamName: string; const Value: Variant);
  protected
    Function CreateParseOpts(DoCreate, EscapeSlash, EscapeRepeat : Boolean) : TSQLParseOptions;
    function DoParseSQL(SQL: String; Options : TSQLParseOptions; ParameterStyle: TParamStyle; out  ParamBinding: TParambinding; MacroChar: Char; out ReplaceString: string): String; virtual;
    Procedure AssignTo(Dest: TPersistent); override;
    Function  GetDataSet: TDataSet;
    Function  GetOwner: TPersistent; override;
    Class Function ParamClass : TParamClass; virtual;
  public
    Constructor Create(AOwner: TPersistent; AItemClass : TCollectionItemClass); overload;
    Constructor Create(AOwner: TPersistent); overload;
    Constructor Create; overload;
    Procedure AddParam(Value: TParam);
    Procedure AssignValues(Value: TParams);
    Function  CreateParam(FldType: TFieldType; const ParamName: string; ParamType: TParamType): TParam;
    Function  FindParam(const Value: string): TParam;
    Procedure GetParamList(List: TList; const ParamNames: string);
    Function  IsEqual(Value: TParams): Boolean;
    Function GetEnumerator: TParamsEnumerator;
    Function  ParamByName(const Value: string): TParam;
    Function  ParseSQL(SQL: String; DoCreate: Boolean): String; overload;
    Function  ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle): String; overload;
    Function  ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle; out ParamBinding: TParambinding): String; overload;
    Function  ParseSQL(SQL: String; DoCreate, EscapeSlash, EscapeRepeat : Boolean; ParameterStyle : TParamStyle; out ParamBinding: TParambinding; out ReplaceString : string): String; overload;
    function  ParseSQL(SQL: String; Options : TSQLParseOptions; ParameterStyle: TParamStyle; out ParamBinding: TParambinding; MacroChar: Char; out ReplaceString: string): String;
    Procedure RemoveParam(Value: TParam);
    Procedure CopyParamValuesFromDataset(ADataset : TDataset; CopyBound : Boolean);
    Property Dataset : TDataset Read GetDataset;
    Property Items[Index: Integer] : TParam read GetItem write SetItem; default;
    Property ParamValues[const ParamName: string] : Variant read GetParamValue write SetParamValue;
  end;

{ TDataSet }
  
  {$ifdef noautomatedbookmark}
  TBookmark = Pointer;
  {$else}
  TBookMark = TBytes;
  {$endif}
  TBookmarkStr = ansistring;

  PBookmarkFlag = ^TBookmarkFlag;
  TBookmarkFlag = (bfCurrent, bfBOF, bfEOF, bfInserted);

{ These types are used by Delphi/Unicode to replace the ambiguous "pchar" buffer types.
  For now, they are just aliases to PAnsiChar, but in Delphi/Unicode it is pbyte. This will
  be changed later (2.8?), to allow a grace period for descendents to catch up.
  
  Testing with TRecordBuffer=PByte will turn up typing problems. TRecordBuffer=pansichar is backwards
  compatible, even if overriden with "pchar" variants.
}
  TRecordBufferBaseType = AnsiChar; // must match TRecordBuffer. 
  TRecordBuffer = PAnsiChar;
  PBufferList = ^TBufferList;
  TBufferList = array[0..dsMaxBufferCount - 1] of TRecordBuffer;  // Dynamic array in Delphi.
  TBufferArray = ^TRecordBuffer;
  
  TGetMode = (gmCurrent, gmNext, gmPrior);

  TGetResult = (grOK, grBOF, grEOF, grError);

  TResyncMode = set of (rmExact, rmCenter);

  TDataAction = (daFail, daAbort, daRetry);

  TUpdateAction = (uaFail, uaAbort, uaSkip, uaRetry, uaApplied);

  TUpdateKind = (ukModify, ukInsert, ukDelete);


  TLocateOption = (loCaseInsensitive, loPartialKey);
  TLocateOptions = set of TLocateOption;

  TDataOperation = procedure of object;

  TDataSetNotifyEvent = procedure(DataSet: TDataSet) of object;
  TDataSetErrorEvent = procedure(DataSet: TDataSet; E: EDatabaseError;
    var DataAction: TDataAction) of object;

  TFilterOption = (foCaseInsensitive, foNoPartialCompare);
  TFilterOptions = set of TFilterOption;

  TFilterRecordEvent = procedure(DataSet: TDataSet;
    var Accept: Boolean) of object;

  TDatasetClass = Class of TDataset;


{------------------------------------------------------------------------------}
{IProviderSupport interface}

  TPSCommandType = (
    ctUnknown,
    ctQuery,
    ctTable,
    ctStoredProc,
    ctSelect,
    ctInsert,
    ctUpdate,
    ctDelete,
    ctDDL
  );

  IProviderSupport = interface
    procedure PSEndTransaction(ACommit: Boolean);
    procedure PSExecute;
    function PSExecuteStatement(const ASQL: string; AParams: TParams;
                                ResultSet: Pointer = nil): Integer;
    procedure PSGetAttributes(List: TList);
    function PSGetCommandText: string;
    function PSGetCommandType: TPSCommandType;
    function PSGetDefaultOrder: TIndexDef;
    function PSGetIndexDefs(IndexTypes: TIndexOptions = [ixPrimary..ixNonMaintained])
                                : TIndexDefs;
    function PSGetKeyFields: string;
    function PSGetParams: TParams;
    function PSGetQuoteChar: string;
    function PSGetTableName: string;
    function PSGetUpdateException(E: Exception; Prev: EUpdateError): EUpdateError;
    function PSInTransaction: Boolean;
    function PSIsSQLBased: Boolean;
    function PSIsSQLSupported: Boolean;
    procedure PSReset;
    procedure PSSetCommandText(const CommandText: string);
    procedure PSSetParams(AParams: TParams);
    procedure PSStartTransaction;
    function PSUpdateRecord(UpdateKind: TUpdateKind; Delta: TDataSet): Boolean;
  end;
{------------------------------------------------------------------------------}

  TDataSet = class(TComponent)
  Private
    FOpenAfterRead : boolean;
    FActiveRecord: Longint;
    FAfterCancel: TDataSetNotifyEvent;
    FAfterClose: TDataSetNotifyEvent;
    FAfterDelete: TDataSetNotifyEvent;
    FAfterEdit: TDataSetNotifyEvent;
    FAfterInsert: TDataSetNotifyEvent;
    FAfterOpen: TDataSetNotifyEvent;
    FAfterPost: TDataSetNotifyEvent;
    FAfterRefresh: TDataSetNotifyEvent;
    FAfterScroll: TDataSetNotifyEvent;
    FAutoCalcFields: Boolean;
    FBOF: Boolean;
    FBeforeCancel: TDataSetNotifyEvent;
    FBeforeClose: TDataSetNotifyEvent;
    FBeforeDelete: TDataSetNotifyEvent;
    FBeforeEdit: TDataSetNotifyEvent;
    FBeforeInsert: TDataSetNotifyEvent;
    FBeforeOpen: TDataSetNotifyEvent;
    FBeforePost: TDataSetNotifyEvent;
    FBeforeRefresh: TDataSetNotifyEvent;
    FBeforeScroll: TDataSetNotifyEvent;
    FBlobFieldCount: Longint;
    FBlockReadSize: Integer;
    FBookmarkSize: Longint;
    FBuffers : TBufferArray;
    FBufferCount: Longint;
    FCalcBuffer: TRecordBuffer;
    FCalcFieldsSize: Longint;
    FConstraints: TCheckConstraints;
    FDisableControlsCount : Integer;
    FDisableControlsState : TDatasetState;
    FCurrentRecord: Longint;
    FDataSources : TFPList;
    FDefaultFields: Boolean;
    FEOF: Boolean;
    FEnableControlsEvent : TDataEvent;
    FFieldList : TFields;
    FFieldDefs: TFieldDefs;
    FFilterOptions: TFilterOptions;
    FFilterText: string;
    FFiltered: Boolean;
    FFound: Boolean;
    FInternalCalcFields: Boolean;
    FModified: Boolean;
    FOnCalcFields: TDataSetNotifyEvent;
    FOnDeleteError: TDataSetErrorEvent;
    FOnEditError: TDataSetErrorEvent;
    FOnFilterRecord: TFilterRecordEvent;
    FOnNewRecord: TDataSetNotifyEvent;
    FOnPostError: TDataSetErrorEvent;
    FRecordCount: Longint;
    FIsUniDirectional: Boolean;
    FState : TDataSetState;
    FInternalOpenComplete: Boolean;
    Procedure DoInsertAppend(DoAppend : Boolean);
    Procedure DoInternalOpen;
    Function  GetBuffer (Index : longint) : TRecordBuffer;
    function GetDatasourceCount: Integer;
    function GetDatasources(aIndex : integer): TDatasource;
    Function  GetField (Index : Longint) : TField;
    Procedure RegisterDataSource(ADataSource : TDataSource);
    Procedure RemoveField (Field : TField);
    procedure SetConstraints(Value: TCheckConstraints);
    Procedure SetField (Index : Longint;Value : TField);
    Procedure ShiftBuffersForward;
    Procedure ShiftBuffersBackward;
    Function  TryDoing (P : TDataOperation; Ev : TDatasetErrorEvent) : Boolean;
    Function GetActive : boolean;
    Procedure UnRegisterDataSource(ADataSource : TDataSource);
    Procedure UpdateFieldDefs;
    procedure SetBlockReadSize(AValue: Integer); virtual;
    Procedure SetFieldDefs(AFieldDefs: TFieldDefs);
    procedure DoInsertAppendRecord(const Values: array of const; DoAppend : boolean);
  protected
    procedure RecalcBufListSize;
    procedure ActivateBuffers; virtual;
    procedure BindFields(Binding: Boolean);
    procedure BlockReadNext; virtual;
    function  BookmarkAvailable: Boolean;
    procedure CalculateFields(Buffer: TRecordBuffer); virtual;
    procedure CheckActive; virtual;
    procedure CheckInactive; virtual;
    procedure CheckBiDirectional;
    procedure Loaded; override;
    procedure ClearBuffers; virtual;
    procedure ClearCalcFields(Buffer: TRecordBuffer); virtual;
    procedure CloseBlob(Field: TField); virtual;
    procedure CloseCursor; virtual;
    procedure CreateFields; virtual;
    procedure DataEvent(Event: TDataEvent; Info: Ptrint); virtual;
    procedure DestroyFields; virtual;
    procedure DoAfterCancel; virtual;
    procedure DoAfterClose; virtual;
    procedure DoAfterDelete; virtual;
    procedure DoAfterEdit; virtual;
    procedure DoAfterInsert; virtual;
    procedure DoAfterOpen; virtual;
    procedure DoAfterPost; virtual;
    procedure DoAfterScroll; virtual;
    procedure DoAfterRefresh; virtual;
    procedure DoBeforeCancel; virtual;
    procedure DoBeforeClose; virtual;
    procedure DoBeforeDelete; virtual;
    procedure DoBeforeEdit; virtual;
    procedure DoBeforeInsert; virtual;
    procedure DoBeforeOpen; virtual;
    procedure DoBeforePost; virtual;
    procedure DoBeforeScroll; virtual;
    procedure DoBeforeRefresh; virtual;
    procedure DoOnCalcFields; virtual;
    procedure DoOnNewRecord; virtual;
    function  FieldByNumber(FieldNo: Longint): TField;
    function  FindRecord(Restart, GoForward: Boolean): Boolean; virtual;
    procedure FreeFieldBuffers; virtual;
    function  GetBookmarkStr: TBookmarkStr; virtual;
    procedure GetCalcFields(Buffer: TRecordBuffer); virtual;
    function  GetCanModify: Boolean; virtual;
    procedure GetChildren(Proc: TGetChildProc; Root: TComponent); override;
    function  GetFieldClass(FieldType: TFieldType): TFieldClass; virtual;
    Function  GetfieldCount : Integer;
    function  GetFieldValues(const FieldName : string) : Variant; virtual;
    function  GetIsIndexField(Field: TField): Boolean; virtual;
    function  GetIndexDefs(IndexDefs : TIndexDefs; IndexTypes : TIndexOptions) : TIndexDefs;
    function  GetNextRecords: Longint; virtual;
    function  GetNextRecord: Boolean; virtual;
    function  GetPriorRecords: Longint; virtual;
    function  GetPriorRecord: Boolean; virtual;
    function  GetRecordCount: Longint; virtual;
    function  GetRecNo: Longint; virtual;
    procedure InitFieldDefs; virtual;
    procedure InitFieldDefsFromFields;
    procedure InitRecord(Buffer: TRecordBuffer); virtual;
    procedure InternalCancel; virtual;
    procedure InternalEdit; virtual;
    procedure InternalInsert; virtual;
    procedure InternalRefresh; virtual;
    procedure OpenCursor(InfoQuery: Boolean); virtual;
    procedure OpenCursorcomplete; virtual;
    procedure RefreshInternalCalcFields(Buffer: TRecordBuffer); virtual;
    procedure RestoreState(const Value: TDataSetState);
    Procedure SetActive (Value : Boolean); virtual;
    procedure SetBookmarkStr(const Value: TBookmarkStr); virtual;
    procedure SetBufListSize(Value: Longint); virtual;
    procedure SetChildOrder(Component: TComponent; Order: Longint); override;
    procedure SetCurrentRecord(Index: Longint); virtual;
    procedure SetDefaultFields(const Value: Boolean);
    procedure SetFiltered(Value: Boolean); virtual;
    procedure SetFilterOptions(Value: TFilterOptions); virtual;
    procedure SetFilterText(const Value: string); virtual;
    procedure SetFieldValues(const FieldName: string; Value: Variant); virtual;
    procedure SetFound(const Value: Boolean); virtual;
    procedure SetModified(Value: Boolean);
    procedure SetName(const Value: TComponentName); override;
    procedure SetOnFilterRecord(const Value: TFilterRecordEvent); virtual;
    procedure SetRecNo(Value: Longint); virtual;
    procedure SetState(Value: TDataSetState);
    function SetTempState(const Value: TDataSetState): TDataSetState;
    Function TempBuffer: TRecordBuffer;
    procedure UpdateIndexDefs; virtual;
    property ActiveRecord: Longint read FActiveRecord;
    property CurrentRecord: Longint read FCurrentRecord;
    property BlobFieldCount: Longint read FBlobFieldCount;
    property BookmarkSize: Longint read FBookmarkSize write FBookmarkSize;
    property Buffers[Index: Longint]: TRecordBuffer read GetBuffer;
    property BufferCount: Longint read FBufferCount;
    property CalcBuffer: TRecordBuffer read FCalcBuffer;
    property CalcFieldsSize: Longint read FCalcFieldsSize;
    property InternalCalcFields: Boolean read FInternalCalcFields;
    property Constraints: TCheckConstraints read FConstraints write SetConstraints;
    function AllocRecordBuffer: TRecordBuffer; virtual;
    procedure FreeRecordBuffer(var Buffer: TRecordBuffer); virtual;
    procedure GetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); virtual;
    function GetBookmarkFlag(Buffer: TRecordBuffer): TBookmarkFlag; virtual;
    function GetDataSource: TDataSource; virtual;
    function GetRecordSize: Word; virtual;
    procedure InternalAddRecord(Buffer: Pointer; AAppend: Boolean); virtual;
    procedure InternalDelete; virtual;
    procedure InternalFirst; virtual;
    procedure InternalGotoBookmark(ABookmark: Pointer); virtual;
    procedure InternalHandleException; virtual;
    procedure InternalInitRecord(Buffer: TRecordBuffer); virtual;
    procedure InternalLast; virtual;
    procedure InternalPost; virtual;
    procedure InternalSetToRecord(Buffer: TRecordBuffer); virtual;
    procedure SetBookmarkFlag(Buffer: TRecordBuffer; Value: TBookmarkFlag); virtual;
    procedure SetBookmarkData(Buffer: TRecordBuffer; Data: Pointer); virtual;
    procedure SetUniDirectional(const Value: Boolean);
    class function FieldDefsClass : TFieldDefsClass; virtual;
    class function FieldsClass : TFieldsClass; virtual;
    Property MyDataSources[aIndex : integer] : TDatasource Read GetDatasources;
    Property MyDataSourceCount : Integer Read GetDatasourceCount;
  protected { abstract methods }
    function GetRecord(Buffer: TRecordBuffer; GetMode: TGetMode; DoCheck: Boolean): TGetResult; virtual; abstract;
    procedure InternalClose; virtual; abstract;
    procedure InternalOpen; virtual; abstract;
    procedure InternalInitFieldDefs; virtual; abstract;
    function IsCursorOpen: Boolean; virtual; abstract;
  protected { IProviderSupport methods }
    procedure PSEndTransaction(Commit: Boolean); virtual;
    procedure PSExecute; virtual;
    function PSExecuteStatement(const ASQL: string; AParams: TParams;
                                ResultSet: Pointer = nil): Integer; virtual;
    procedure PSGetAttributes(List: TList); virtual;
    function PSGetCommandText: string; virtual;
    function PSGetCommandType: TPSCommandType; virtual;
    function PSGetDefaultOrder: TIndexDef; virtual;
    function PSGetIndexDefs(IndexTypes: TIndexOptions = [ixPrimary..ixNonMaintained])
                                : TIndexDefs; virtual;
    function PSGetKeyFields: string; virtual;
    function PSGetParams: TParams; virtual;
    function PSGetQuoteChar: string; virtual;
    function PSGetTableName: string; virtual;
    function PSGetUpdateException(E: Exception; Prev: EUpdateError)
                                : EUpdateError; virtual;
    function PSInTransaction: Boolean; virtual;
    function PSIsSQLBased: Boolean; virtual;
    function PSIsSQLSupported: Boolean; virtual;
    procedure PSReset; virtual;
    procedure PSSetCommandText(const CommandText: string); virtual;
    procedure PSSetParams(AParams: TParams); virtual;
    procedure PSStartTransaction; virtual;
    function PSUpdateRecord(UpdateKind: TUpdateKind; Delta: TDataSet)
                                : Boolean; virtual;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    function ActiveBuffer: TRecordBuffer;
    function GetFieldData(Field: TField; Buffer: Pointer): Boolean; overload; virtual;
    function GetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean): Boolean; overload; virtual;
    procedure SetFieldData(Field: TField; Buffer: Pointer); overload; virtual;
    procedure SetFieldData(Field: TField; Buffer: Pointer; NativeFormat: Boolean); overload; virtual;
    procedure Append;
    procedure AppendRecord(const Values: array of const);
    function BookmarkValid(ABookmark: TBookmark): Boolean; virtual;
    procedure Cancel; virtual;
    procedure CheckBrowseMode;
    procedure ClearFields;
    procedure Close;
    function  ControlsDisabled: Boolean;
    function CompareBookmarks(Bookmark1, Bookmark2: TBookmark): Longint; virtual;
    function CreateBlobStream(Field: TField; Mode: TBlobStreamMode): TStream; virtual;
    procedure CursorPosChanged;
    procedure DataConvert(aField: TField; aSource, aDest: Pointer; aToNative: Boolean); virtual;
    procedure Delete; virtual;
    procedure DisableControls;
    procedure Edit;
    procedure EnableControls;
    function FieldByName(const FieldName: string): TField;
    function FindField(const FieldName: string): TField;
    function FindFirst: Boolean; virtual;
    function FindLast: Boolean; virtual;
    function FindNext: Boolean; virtual;
    function FindPrior: Boolean; virtual;
    procedure First;
    procedure FreeBookmark(ABookmark: TBookmark); virtual;
    function GetBookmark: TBookmark; virtual;
    function GetCurrentRecord(Buffer: TRecordBuffer): Boolean; virtual;
    procedure GetFieldList(List: TList; const FieldNames: string);
    procedure GetFieldNames(List: TStrings);
    procedure GotoBookmark(const ABookmark: TBookmark);
    procedure Insert;
    procedure InsertRecord(const Values: array of const);
    function IsEmpty: Boolean;
    function IsLinkedTo(ADataSource: TDataSource): Boolean;
    function IsSequenced: Boolean; virtual;
    procedure Last;
    function Locate(const KeyFields: string; const KeyValues: Variant; Options: TLocateOptions) : boolean; virtual;
    function Lookup(const KeyFields: string; const KeyValues: Variant; const ResultFields: string): Variant; virtual;
    function MoveBy(Distance: Longint): Longint;
    procedure Next;
    procedure Open;
    procedure Post; virtual;
    procedure Prior;
    procedure Refresh;
    procedure Resync(Mode: TResyncMode); virtual;
    procedure SetFields(const Values: array of const);
    function  Translate(Src, Dest: PChar; ToOem: Boolean): Integer; virtual;
    procedure UpdateCursorPos;
    procedure UpdateRecord;
    function UpdateStatus: TUpdateStatus; virtual;
    property BlockReadSize: Integer read FBlockReadSize write SetBlockReadSize;
    property BOF: Boolean read FBOF;
    property Bookmark: TBookmark read GetBookmark write GotoBookmark;
    property CanModify: Boolean read GetCanModify;
    property DataSource: TDataSource read GetDataSource;
    property DefaultFields: Boolean read FDefaultFields;
    property EOF: Boolean read FEOF;
    property FieldCount: Longint read GetFieldCount;
    property FieldDefs: TFieldDefs read FFieldDefs write SetFieldDefs;
//    property Fields[Index: Longint]: TField read GetField write SetField;
    property Found: Boolean read FFound;
    property Modified: Boolean read FModified;
    property IsUniDirectional: Boolean read FIsUniDirectional default False;
    property RecordCount: Longint read GetRecordCount;
    property RecNo: Longint read GetRecNo write SetRecNo;
    property RecordSize: Word read GetRecordSize;
    property State: TDataSetState read FState;
    property Fields : TFields read FFieldList;
    property FieldValues[FieldName : string] : Variant read GetFieldValues write SetFieldValues; default;
    property Filter: string read FFilterText write SetFilterText;
    property Filtered: Boolean read FFiltered write SetFiltered default False;
    property FilterOptions: TFilterOptions read FFilterOptions write SetFilterOptions;
    property Active: Boolean read GetActive write SetActive default False;
    property AutoCalcFields: Boolean read FAutoCalcFields write FAutoCalcFields default true;
    property BeforeOpen: TDataSetNotifyEvent read FBeforeOpen write FBeforeOpen;
    property AfterOpen: TDataSetNotifyEvent read FAfterOpen write FAfterOpen;
    property BeforeClose: TDataSetNotifyEvent read FBeforeClose write FBeforeClose;
    property AfterClose: TDataSetNotifyEvent read FAfterClose write FAfterClose;
    property BeforeInsert: TDataSetNotifyEvent read FBeforeInsert write FBeforeInsert;
    property AfterInsert: TDataSetNotifyEvent read FAfterInsert write FAfterInsert;
    property BeforeEdit: TDataSetNotifyEvent read FBeforeEdit write FBeforeEdit;
    property AfterEdit: TDataSetNotifyEvent read FAfterEdit write FAfterEdit;
    property BeforePost: TDataSetNotifyEvent read FBeforePost write FBeforePost;
    property AfterPost: TDataSetNotifyEvent read FAfterPost write FAfterPost;
    property BeforeCancel: TDataSetNotifyEvent read FBeforeCancel write FBeforeCancel;
    property AfterCancel: TDataSetNotifyEvent read FAfterCancel write FAfterCancel;
    property BeforeDelete: TDataSetNotifyEvent read FBeforeDelete write FBeforeDelete;
    property AfterDelete: TDataSetNotifyEvent read FAfterDelete write FAfterDelete;
    property BeforeScroll: TDataSetNotifyEvent read FBeforeScroll write FBeforeScroll;
    property AfterScroll: TDataSetNotifyEvent read FAfterScroll write FAfterScroll;
    property BeforeRefresh: TDataSetNotifyEvent read FBeforeRefresh write FBeforeRefresh;
    property AfterRefresh: TDataSetNotifyEvent read FAfterRefresh write FAfterRefresh;
    property OnCalcFields: TDataSetNotifyEvent read FOnCalcFields write FOnCalcFields;
    property OnDeleteError: TDataSetErrorEvent read FOnDeleteError write FOnDeleteError;
    property OnEditError: TDataSetErrorEvent read FOnEditError write FOnEditError;
    property OnFilterRecord: TFilterRecordEvent read FOnFilterRecord write SetOnFilterRecord;
    property OnNewRecord: TDataSetNotifyEvent read FOnNewRecord write FOnNewRecord;
    property OnPostError: TDataSetErrorEvent read FOnPostError write FOnPostError;
  end;

  TDataSetEnumerator = class
  private
    FDataSet: TDataSet;
    FBOF: Boolean;
    function GetCurrent: TFields;
  public  
    constructor Create(ADataSet: TDataSet);
    function MoveNext: Boolean;
    property Current: TFields read GetCurrent;
  end;

{ TDataLink }

  TDataLink = class(TPersistent)
  private
    FFirstRecord,
    FBufferCount : Integer;
    FActive,
    FDataSourceFixed,
    FEditing,
    FReadOnly,
    FUpdatingRecord,
    FVisualControl : Boolean;
    FDataSource : TDataSource;
    Function  CalcFirstRecord(Index : Integer) : Integer;
    Procedure CalcRange;
    Procedure CheckActiveAndEditing;
    Function  GetDataset : TDataset;
    procedure SetActive(AActive: Boolean);
    procedure SetDataSource(Value: TDataSource);
    Procedure SetReadOnly(Value : Boolean);
  protected
    procedure ActiveChanged; virtual;
    procedure CheckBrowseMode; virtual;
    procedure DataEvent(Event: TDataEvent; Info: Ptrint); virtual;
    procedure DataSetChanged; virtual;
    procedure DataSetScrolled(Distance: Integer); virtual;
    procedure EditingChanged; virtual;
    procedure FocusControl(Field: TFieldRef); virtual;
    function  GetActiveRecord: Integer; virtual;
    function  GetBOF: Boolean; virtual;
    function  GetBufferCount: Integer; virtual;
    function  GetEOF: Boolean; virtual;
    function  GetRecordCount: Integer; virtual;
    procedure LayoutChanged; virtual;
    function  MoveBy(Distance: Integer): Integer; virtual;
    procedure RecordChanged(Field: TField); virtual;
    procedure SetActiveRecord(Value: Integer); virtual;
    procedure SetBufferCount(Value: Integer); virtual;
    procedure UpdateData; virtual;
    property VisualControl: Boolean read FVisualControl write FVisualControl;
    property FirstRecord: Integer read FFirstRecord write FFirstRecord;
  public
    constructor Create;
    destructor Destroy; override;
    function  Edit: Boolean;
    procedure UpdateRecord;
    function ExecuteAction(Action: TBasicAction): Boolean; virtual;
    function UpdateAction(Action: TBasicAction): Boolean; virtual;
    property Active: Boolean read FActive;
    property ActiveRecord: Integer read GetActiveRecord write SetActiveRecord;
    property BOF: Boolean read GetBOF;
    property BufferCount: Integer read GetBufferCount write SetBufferCount;
    property DataSet: TDataSet read GetDataSet;
    property DataSource: TDataSource read FDataSource write SetDataSource;
    property DataSourceFixed: Boolean read FDataSourceFixed write FDataSourceFixed;
    property Editing: Boolean read FEditing;
    property Eof: Boolean read GetEOF;
    property ReadOnly: Boolean read FReadOnly write SetReadOnly;
    property RecordCount: Integer read GetRecordCount;
  end;

{ TDetailDataLink }

  TDetailDataLink = class(TDataLink)
  protected
    function GetDetailDataSet: TDataSet; virtual;
  public
    property DetailDataSet: TDataSet read GetDetailDataSet;
  end;

{ TMasterDataLink }

  TMasterDataLink = class(TDetailDataLink)
  private
    FDetailDataSet: TDataSet;
    FFieldNames: string;
    FFields: TList;
    FOnMasterChange: TNotifyEvent;
    FOnMasterDisable: TNotifyEvent;
    procedure SetFieldNames(const Value: string);
  protected
    procedure ActiveChanged; override;
    procedure CheckBrowseMode; override;
    function GetDetailDataSet: TDataSet; override;
    procedure LayoutChanged; override;
    procedure RecordChanged(Field: TField); override;
    Procedure DoMasterDisable; virtual;
    Procedure DoMasterChange; virtual;
  public
    constructor Create(ADataSet: TDataSet);virtual;
    destructor Destroy; override;
    property FieldNames: string read FFieldNames write SetFieldNames;
    property Fields: TList read FFields;
    property OnMasterChange: TNotifyEvent read FOnMasterChange write FOnMasterChange;
    property OnMasterDisable: TNotifyEvent read FOnMasterDisable write FOnMasterDisable;
  end;

{ TMasterParamsDataLink }

  TMasterParamsDataLink = Class(TMasterDataLink)
  Private
    FParams : TParams;
    Procedure SetParams(AValue : TParams);
  Protected
    Procedure DoMasterDisable; override;
    Procedure DoMasterChange; override;
  Public
    constructor Create(ADataSet: TDataSet); override;
    Procedure RefreshParamNames; virtual;
    Procedure CopyParamsFromMaster(CopyBound : Boolean); virtual;
    Property Params : TParams Read FParams Write SetParams;
  end;

{ TDataSource }

  TDataChangeEvent = procedure(Sender: TObject; Field: TField) of object;

  TDataSource = class(TComponent)
  private
    FDataSet: TDataSet;
    FDataLinks: TList;
    FEnabled: Boolean;
    FAutoEdit: Boolean;
    FState: TDataSetState;
    FOnStateChange: TNotifyEvent;
    FOnDataChange: TDataChangeEvent;
    FOnUpdateData: TNotifyEvent;
    procedure DistributeEvent(Event: TDataEvent; Info: Ptrint);
    function GetLink(AIndex : Integer): TDataLink;
    function GetLinkCount: Integer;
    procedure RegisterDataLink(DataLink: TDataLink);
    Procedure ProcessEvent(Event : TDataEvent; Info : Ptrint);
    procedure SetDataSet(ADataSet: TDataSet);
    procedure SetEnabled(Value: Boolean);
    procedure UnregisterDataLink(DataLink: TDataLink);
  protected
    Procedure DoDataChange (Info : Pointer);virtual;
    Procedure DoStateChange; virtual;
    Procedure DoUpdateData;
    property DataLinks: TList read FDataLinks;
    Property DataLink[AIndex : Integer] : TDataLink Read GetLink;
    Property DataLinkCount : Integer Read GetLinkCount;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure Edit;
    function IsLinkedTo(ADataset: TDataSet): Boolean;
    property State: TDataSetState read FState;
  published
    property AutoEdit: Boolean read FAutoEdit write FAutoEdit default True;
    property DataSet: TDataSet read FDataSet write SetDataSet;
    property Enabled: Boolean read FEnabled write SetEnabled default True;
    property OnStateChange: TNotifyEvent read FOnStateChange write FOnStateChange;
    property OnDataChange: TDataChangeEvent read FOnDataChange write FOnDataChange;
    property OnUpdateData: TNotifyEvent read FOnUpdateData write FOnUpdateData;
  end;

  { TDBDataset }

  TDBDatasetClass = Class of TDBDataset;
  TDBDataset = Class(TDataset)
    Private
      FDatabase : TDatabase;
      FTransaction : TDBTransaction;
    Protected
      Procedure SetDatabase (Value : TDatabase); virtual;
      Procedure SetTransaction(Value : TDBTransaction); virtual;
      Procedure CheckDatabase;
    Public
      Destructor destroy; override;
      Property DataBase : TDatabase Read FDatabase Write SetDatabase;
      Property Transaction : TDBTransaction Read FTransaction Write SetTransaction;
    end;

  { TDBTransaction }

  TDBTransactionClass = Class of TDBTransaction;
  TDBTransaction = Class(TComponent)
  Private
    FActive        : boolean;
    FDatabase      : TDatabase;
    FDataSets      : TThreadList;
    FOpenAfterRead : boolean;
    Function GetDataSetCount : Longint;
    Function GetDataset(Index : longint) : TDBDataset;
    procedure RegisterDataset (DS : TDBDataset);
    procedure UnRegisterDataset (DS : TDBDataset);
    procedure RemoveDataSets;
    procedure SetActive(Value : boolean);
  Protected
    Function AllowClose(DS: TDBDataset): Boolean; virtual;
    Procedure SetDatabase (Value : TDatabase); virtual;
    procedure CloseTrans;
    procedure OpenTrans;
    Procedure CheckDatabase;
    Procedure CheckActive;
    Procedure CheckInactive;
    procedure Commit; virtual;
    procedure CommitRetaining; virtual;
    procedure Rollback; virtual;
    procedure RollbackRetaining; virtual;
    procedure EndTransaction; virtual; abstract;
    procedure StartTransaction; virtual; abstract;
    procedure InternalHandleException; virtual;
    procedure Loaded; override;
  Public
    constructor Create(AOwner: TComponent); override;
    Destructor Destroy; override;
    procedure CloseDataSets;
    Property DataBase : TDatabase Read FDatabase Write SetDatabase;
  published
    property Active : boolean read FActive write setactive;
  end;

  { TCustomConnection }

  TLoginEvent = procedure(Sender: TObject; Username, Password: string) of object;

  TCustomConnection = class(TComponent)
  private
    FAfterConnect: TNotifyEvent;
    FAfterDisconnect: TNotifyEvent;
    FBeforeConnect: TNotifyEvent;
    FBeforeDisconnect: TNotifyEvent;
    FForcedClose: Boolean;
    FLoginPrompt: Boolean;
    FOnLogin: TLoginEvent;
    FStreamedConnected: Boolean;
    procedure SetAfterConnect(const AValue: TNotifyEvent);
    procedure SetAfterDisconnect(const AValue: TNotifyEvent);
    procedure SetBeforeConnect(const AValue: TNotifyEvent);
    procedure SetBeforeDisconnect(const AValue: TNotifyEvent);
  protected
    procedure DoLoginPrompt; virtual;
    procedure DoConnect; virtual;
    procedure DoDisconnect; virtual;
    function GetConnected : boolean; virtual;
    Function GetDataset(Index : longint) : TDataset; virtual;
    Function GetDataSetCount : Longint; virtual;
    procedure GetLoginParams(out ADatabaseName, AUserName, APassword: string); virtual;
    procedure InternalHandleException; virtual;
    procedure Loaded; override;
    procedure SetConnected (Value : boolean); virtual;
    procedure SetLoginParams(const ADatabaseName, AUserName, APassword: string); virtual;
    property ForcedClose : Boolean read FForcedClose write FForcedClose;
    property StreamedConnected: Boolean read FStreamedConnected write FStreamedConnected;
  public
    procedure Close(ForceClose: Boolean=False);
    destructor Destroy; override;
    procedure Open;
    property DataSetCount: Longint read GetDataSetCount;
    property DataSets[Index: Longint]: TDataSet read GetDataSet;
  published
    property Connected: Boolean read GetConnected write SetConnected;
    property LoginPrompt: Boolean read FLoginPrompt write FLoginPrompt;

    property AfterConnect : TNotifyEvent read FAfterConnect write SetAfterConnect;
    property AfterDisconnect : TNotifyEvent read FAfterDisconnect write SetAfterDisconnect;
    property BeforeConnect : TNotifyEvent read FBeforeConnect write SetBeforeConnect;
    property BeforeDisconnect : TNotifyEvent read FBeforeDisconnect write SetBeforeDisconnect;
    property OnLogin: TLoginEvent read FOnLogin write FOnLogin;
  end;


  { TDatabase }

  TDatabaseClass = Class Of TDatabase;

  TDatabase = class(TCustomConnection)
  private
    FConnected : Boolean;
    FDataBaseName : String;
    FDataSets : TThreadList;
    FTransactions : TThreadList;
    FDirectory : String;
    FKeepConnection : Boolean;
    FParams : TStrings;
    FSQLBased : Boolean;
    FOpenAfterRead : boolean;
    Function GetTransactionCount : Longint;
    Function GetTransaction(Index : longint) : TDBTransaction;
    procedure RegisterDataset (DS : TDBDataset);
    procedure RegisterTransaction (TA : TDBTransaction);
    procedure UnRegisterDataset (DS : TDBDataset);
    procedure UnRegisterTransaction(TA : TDBTransaction);
    procedure RemoveDataSets;
    procedure RemoveTransactions;
    procedure SetParams(AValue: TStrings);
  protected
    Procedure CheckConnected;
    Procedure CheckDisConnected;
    procedure DoConnect; override;
    procedure DoDisconnect; override;
    function GetConnected : boolean; override;
    Function GetDataset(Index : longint) : TDataset; override;
    Function GetDataSetCount : Longint; override;
    Procedure DoInternalConnect; Virtual;Abstract;
    Procedure DoInternalDisConnect; Virtual;Abstract;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure CloseDataSets;
    procedure CloseTransactions;
//    procedure ApplyUpdates;
    procedure StartTransaction; virtual; abstract;
    procedure EndTransaction; virtual; abstract;
    property TransactionCount: Longint read GetTransactionCount;
    property Transactions[Index: Longint]: TDBTransaction read GetTransaction;
    property Directory: string read FDirectory write FDirectory;
    property IsSQLBased: Boolean read FSQLBased;
  published
    property Connected: Boolean read FConnected write SetConnected;
    property DatabaseName: string read FDatabaseName write FDatabaseName;
    property KeepConnection: Boolean read FKeepConnection write FKeepConnection;
    property Params : TStrings read FParams Write SetParams;
  end;


const
  FieldTypetoVariantMap : array[TFieldType] of Integer =
    (
      {ftUnknown} varError,
      {ftString} varOleStr,
      {ftSmallint} varSmallint,
      {ftInteger} varInteger,
      {ftWord} varSmallint,
      {ftBoolean} varBoolean,
      {ftFloat} varDouble,
      {ftCurrency} varCurrency,
      {ftBCD} varCurrency,
      {ftDate} varDate,
      {ftTime} varDate,
      {ftDateTime} varDate,
      {ftBytes} varOleStr,
      {ftVarBytes} varOleStr,
      {ftAutoInc} varInteger,
      {ftBlob} varOleStr,
      {ftMemo} varOleStr,
      {ftGraphic} varOleStr,
      {ftFmtMemo} varOleStr,
      {ftParadoxOle} varOleStr,
      {ftDBaseOle} varOleStr,
      {ftTypedBinary} varOleStr,
      {ftCursor} varError,
      {ftFixedChar} varOleStr,
      {ftWideString} varOleStr,
      {ftLargeint} varint64,
      {ftADT} varError,
      {ftArray} varError,
      {ftReference} varError,
      {ftDataSet} varError,
      {ftOraBlob} varOleStr,
      {ftOraClob} varOleStr,
      {ftVariant} varVariant,
      {ftInterface} varUnknown,
      {ftIDispatch} varDispatch,
      {ftGuid} varOleStr,
      {ftTimeStamp} varOleStr,
      {ftFMTBcd} varDouble,
      {ftFixedWideChar} varOleStr,
      {ftWideMemo} varOleStr
    );


Const
  Fieldtypenames : Array [TFieldType] of String[15] =
    (
      {ftUnknown} 'Unknown',
      {ftString} 'String',
      {ftSmallint} 'Smallint',
      {ftInteger} 'Integer',
      {ftWord} 'Word',
      {ftBoolean} 'Boolean',
      {ftFloat} 'Float',
      {ftCurrency} 'Currency',
      {ftBCD} 'BCD',
      {ftDate} 'Date',
      {ftTime} 'Time',
      {ftDateTime} 'DateTime',
      {ftBytes} 'Bytes',
      {ftVarBytes} 'VarBytes',
      {ftAutoInc} 'AutoInc',
      {ftBlob} 'Blob',
      {ftMemo} 'Memo',
      {ftGraphic} 'Graphic',
      {ftFmtMemo} 'FmtMemo',
      {ftParadoxOle} 'ParadoxOle',
      {ftDBaseOle} 'DBaseOle',
      {ftTypedBinary} 'TypedBinary',
      {ftCursor} 'Cursor',
      {ftFixedChar} 'FixedChar',
      {ftWideString} 'WideString',
      {ftLargeint} 'Largeint',
      {ftADT} 'ADT',
      {ftArray} 'Array',
      {ftReference} 'Reference',
      {ftDataSet} 'DataSet',
      {ftOraBlob} 'OraBlob',
      {ftOraClob} 'OraClob',
      {ftVariant} 'Variant',
      {ftInterface} 'Interface',
      {ftIDispatch} 'IDispatch',
      {ftGuid} 'Guid',
      {ftTimeStamp} 'TimeStamp',
      {ftFMTBcd} 'FMTBcd',
      {ftFixedWideChar} 'FixedWideChar',
      {ftWideMemo} 'WideMemo'
    );


const
  DefaultFieldClasses : Array [TFieldType] of TFieldClass =
    (
      { ftUnknown} Tfield,
      { ftString} TStringField,
      { ftSmallint} TSmallIntField,
      { ftInteger} TLongintField,
      { ftWord} TWordField,
      { ftBoolean} TBooleanField,
      { ftFloat} TFloatField,
      { ftCurrency} TCurrencyField,
      { ftBCD} TBCDField,
      { ftDate} TDateField,
      { ftTime} TTimeField,
      { ftDateTime} TDateTimeField,
      { ftBytes} TBytesField,
      { ftVarBytes} TVarBytesField,
      { ftAutoInc} TAutoIncField,
      { ftBlob} TBlobField,
      { ftMemo} TMemoField,
      { ftGraphic} TGraphicField,
      { ftFmtMemo} TBlobField,
      { ftParadoxOle} TBlobField,
      { ftDBaseOle} TBlobField,
      { ftTypedBinary} TBlobField,
      { ftCursor} Nil,
      { ftFixedChar} TStringField,
      { ftWideString} TWideStringField,
      { ftLargeint} TLargeIntField,
      { ftADT} Nil,
      { ftArray} Nil,
      { ftReference} Nil,
      { ftDataSet} Nil,
      { ftOraBlob} TBlobField,
      { ftOraClob} TMemoField,
      { ftVariant} TVariantField,
      { ftInterface} Nil,
      { ftIDispatch} Nil,
      { ftGuid} TGuidField,
      { ftTimeStamp} Nil,
      { ftFMTBcd} TFMTBCDField,
      { ftFixedWideString} TWideStringField,
      { ftWideMemo} TWideMemoField
    );

  dsEditModes = [dsEdit, dsInsert, dsSetKey];
  dsWriteModes = [dsEdit, dsInsert, dsSetKey, dsCalcFields, dsFilter,
                  dsNewValue, dsInternalCalc, dsRefreshFields];
  // Correct list of all field types that are BLOB types.
  // Please use this instead of checking TBlobType which will give
  // incorrect results
  ftBlobTypes = [ftBlob, ftMemo, ftGraphic, ftFmtMemo, ftParadoxOle,
    ftDBaseOle, ftTypedBinary, ftOraBlob, ftOraClob, ftWideMemo];

var
  LoginDialogExProc: function(const ADatabaseName: string; var AUserName, APassword: string; UserNameReadOnly: Boolean): Boolean = nil;


{ Auxiliary functions }

Procedure DatabaseError (Const Msg : String); overload;
Procedure DatabaseError (Const Msg : String; Comp : TComponent); overload;
Procedure DatabaseErrorFmt (Const Fmt : String; Const Args : Array Of Const); overload;
Procedure DatabaseErrorFmt (Const Fmt : String; Const Args : Array Of Const; Comp : TComponent); overload;
Function ExtractFieldName(Const Fields: String; var Pos: Integer): String;
Function DateTimeRecToDateTime(DT: TFieldType; Data: TDateTimeRec): TDateTime;
Function DateTimeToDateTimeRec(DT: TFieldType; Data: TDateTime): TDateTimeRec;

procedure DisposeMem(var Buffer; Size: Integer);
function BuffersEqual(Buf1, Buf2: Pointer; Size: Integer): Boolean;

function SkipComments(var p: PChar; EscapeSlash, EscapeRepeat : Boolean) : boolean;

operator Enumerator(ADataSet: TDataSet): TDataSetEnumerator;
 
implementation

uses dbconst,typinfo;

{ ---------------------------------------------------------------------
    Auxiliary functions
  ---------------------------------------------------------------------}

Procedure DatabaseError (Const Msg : String);

begin
  Raise EDataBaseError.Create(Msg);
end;

Procedure DatabaseError (Const Msg : String; Comp : TComponent);

begin
  if assigned(Comp) and (Comp.Name <> '') then
    Raise EDatabaseError.CreateFmt('%s : %s',[Comp.Name,Msg])
  else
    DatabaseError(Msg);
end;

Procedure DatabaseErrorFmt (Const Fmt : String; Const Args : Array Of Const);

begin
  Raise EDatabaseError.CreateFmt(Fmt,Args);
end;

Procedure DatabaseErrorFmt (Const Fmt : String; Const Args : Array Of Const;
                            Comp : TComponent);
begin
  if assigned(comp) then
    Raise EDatabaseError.CreateFmt(Format('%s : %s',[Comp.Name,Fmt]),Args)
  else
    DatabaseErrorFmt(Fmt, Args);
end;

function ExtractFieldName(const Fields: string; var Pos: Integer): string;
var
  i: Integer;
  FieldsLength: Integer;
begin
  i:=Pos;
  FieldsLength:=Length(Fields);
  while (i<=FieldsLength) and (Fields[i]<>';') do Inc(i);
  Result:=Trim(Copy(Fields,Pos,i-Pos));
  if (i<=FieldsLength) and (Fields[i]=';') then Inc(i);
  Pos:=i;
end;


{ EUpdateError }
constructor EUpdateError.Create(NativeError, Context : String;
                                ErrCode, PrevError : integer; E: Exception);
                                
begin
  Inherited CreateFmt(NativeError,[Context]);
  FContext := Context;
  FErrorCode := ErrCode;
  FPreviousError := PrevError;
  FOriginalException := E;
end;

Destructor EUpdateError.Destroy;

begin
  FOriginalException.Free;
  Inherited;
end;

{ TNamedItem }

function TNamedItem.GetDisplayName: string;
begin
  Result := FName;
end;

procedure TNamedItem.SetDisplayName(const AValue: string);
Var TmpInd : Integer;
begin
  if FName=AValue then exit;
  if (AValue <> '') and (Collection is TFieldDefs ) then
    begin
    TmpInd :=  (TDefCollection(Collection).IndexOf(AValue));
    if (TmpInd >= 0) and (TmpInd <> Index) then
      DatabaseErrorFmt(SDuplicateName, [AValue, Collection.ClassName]);
    end;
  FName:=AValue;
  inherited SetDisplayName(AValue);
end;

{ TDefCollection }

procedure TDefCollection.SetItemName(AItem: TCollectionItem);
begin
  with AItem as TNamedItem do
    if Name = '' then
      begin
      if assigned(Dataset) then
        Name := Dataset.Name + Copy(ClassName, 2, 5) + IntToStr(ID+1)
      else
        Name := Copy(ClassName, 2, 5) + IntToStr(ID+1);
      end
  else inherited SetItemName(AItem);
end;

constructor TDefCollection.create(ADataset: TDataset; AOwner: TPersistent;
  AClass: TCollectionItemClass);
begin
  inherited Create(AOwner,AClass);
  FDataset := ADataset;
end;

function TDefCollection.Find(const AName: string): TNamedItem;
var i: integer;
begin
  Result := Nil;
  for i := 0 to Count - 1 do if AnsiSameText(TNamedItem(Items[i]).Name, AName) then
    begin
    Result := TNamedItem(Items[i]);
    Break;
    end;
end;

procedure TDefCollection.GetItemNames(List: TStrings);
var i: LongInt;
begin
  for i := 0 to Count - 1 do
    List.Add(TNamedItem(Items[i]).Name);
end;

function TDefCollection.IndexOf(const AName: string): Longint;
var i: LongInt;
begin
  Result := -1;
  for i := 0 to Count - 1 do
    if AnsiSameText(TNamedItem(Items[i]).Name, AName) then
    begin
    Result := i;
    Break;
    end;
end;

{ TIndexDef }

procedure TIndexDef.SetDescFields(const AValue: string);
begin
  if FDescFields=AValue then exit;
  if AValue <> '' then FOptions:=FOptions + [ixDescending];
  FDescFields:=AValue;
end;

procedure TIndexDef.Assign(Source: TPersistent);
var idef : TIndexDef;
begin
  idef := nil;
  if Source is TIndexDef then idef := Source as TIndexDef;
  if Assigned(idef) then
     begin
     FName := idef.Name;
     FFields := idef.Fields;
     FOptions := idef.Options;
     FCaseinsFields := idef.CaseInsFields;
     FDescFields := idef.DescFields;
     FSource := idef.Source;
     FExpression := idef.Expression;
     end
  else
    inherited Assign(Source);
end;

function TIndexDef.GetExpression: string;
begin
  Result := FExpression;
end;

procedure TIndexDef.SetExpression(const AValue: string);
begin
  FExpression := AValue;
end;

procedure TIndexDef.SetCaseInsFields(const AValue: string);
begin
  if FCaseinsFields=AValue then exit;
  if AValue <> '' then FOptions:=FOptions + [ixCaseInsensitive];
  FCaseinsFields:=AValue;
end;

constructor TIndexDef.Create(Owner: TIndexDefs; const AName, TheFields: string;
      TheOptions: TIndexOptions);

begin
  FName := aname;
  inherited create(Owner);
  FFields := TheFields;
  FOptions := TheOptions;
end;


{ TIndexDefs }

Function TIndexDefs.GetItem (Index : integer) : TIndexDef;

begin
  Result:=(Inherited GetItem(Index)) as TIndexDef;
end;

Procedure TIndexDefs.SetItem(Index: Integer; Value: TIndexDef);
begin
  Inherited SetItem(Index,Value);
end;

constructor TIndexDefs.Create(ADataSet: TDataSet);

begin
  inherited create(ADataset, Owner, TIndexDef);
end;


Function TIndexDefs.AddIndexDef: TIndexDef;

begin
  Result := inherited add as TIndexDef;
end;

procedure TIndexDefs.Add(const Name, Fields: string; Options: TIndexOptions);

Var
  D : TIndexDef;

begin
  D:=AddIndexDef;
  D.Name:=Name;
  D.Fields:=Fields;
  D.Options:=Options;
end;

function TIndexDefs.Find(const IndexName: string): TIndexDef;
begin
  Result := (inherited Find(IndexName)) as TIndexDef;
  if (Result=Nil) Then
    DatabaseErrorFmt(SIndexNotFound, [IndexName], FDataSet);
end;

function TIndexDefs.FindIndexForFields(const Fields: string): TIndexDef;

begin
  //!! To be implemented
end;


function TIndexDefs.GetIndexForFields(const Fields: string;
  CaseInsensitive: Boolean): TIndexDef;

var
  i, FieldsLen: integer;
  Last: TIndexDef;
begin
  Last := nil;
  FieldsLen := Length(Fields);
  for i := 0 to Count - 1 do
  begin
    Result := Items[I];
    if (Result.Options * [ixDescending, ixExpression] = []) and
       (not CaseInsensitive or (ixCaseInsensitive in Result.Options)) and
       AnsiSameText(Fields, Result.Fields) then
    begin
      Exit;
    end else
    if AnsiSameText(Fields, Copy(Result.Fields, 1, FieldsLen)) and
       ((Length(Result.Fields) = FieldsLen) or
       (Result.Fields[FieldsLen + 1] = ';')) then
    begin
      if (Last = nil) or
         ((Last <> nil) And (Length(Last.Fields) > Length(Result.Fields))) then
           Last := Result;
    end;
  end;
  Result := Last;
end;

procedure TIndexDefs.Update;

begin
  if (not updated) and assigned(Dataset) then
    begin
    Dataset.UpdateIndexDefs;
    updated := True;
    end;
end;

{ TCheckConstraint }

procedure TCheckConstraint.Assign(Source: TPersistent);

begin
  //!! To be implemented
end;



{ TCheckConstraints }

Function TCheckConstraints.GetItem(Index : Longint) : TCheckConstraint;

begin
  //!! To be implemented
  Result := nil;
end;


Procedure TCheckConstraints.SetItem(index : Longint; Value : TCheckConstraint);

begin
  //!! To be implemented
end;


function TCheckConstraints.GetOwner: TPersistent;

begin
  //!! To be implemented
  Result := nil;
end;


constructor TCheckConstraints.Create(AOwner: TPersistent);

begin
  //!! To be implemented
  inherited Create(TCheckConstraint);
end;


function TCheckConstraints.Add: TCheckConstraint;

begin
  //!! To be implemented
  Result := nil;
end;

{ TLookupList }

constructor TLookupList.Create;

begin
  FList := TFPList.Create;
end;

destructor TLookupList.Destroy;

begin
  Clear;
  FList.Destroy;
  inherited Destroy;
end;

procedure TLookupList.Add(const AKey, AValue: Variant);

var LookupRec: PLookupListRec;
begin
  New(LookupRec);
  LookupRec^.Key := AKey;
  LookupRec^.Value := AValue;
  FList.Add(LookupRec);
end;

procedure TLookupList.Clear;
var i: integer;
begin
  for i := 0 to FList.Count - 1 do Dispose(PLookupListRec(FList[i]));
  FList.Clear;
end;

function TLookupList.FirstKeyByValue(const AValue: Variant): Variant;
var
  i: Integer;
begin
  for i := 0 to FList.Count - 1 do
    with PLookupListRec(FList[i])^ do
      if Value = AValue then
        begin
        Result := Key;
        exit;
        end;
  Result := Null;
end;

function TLookupList.ValueOfKey(const AKey: Variant): Variant;

  Function VarArraySameValues(VarArray1,VarArray2 : Variant) : Boolean;
  // This only works for one-dimensional vararrays with a lower bound of 0
  // and equal higher bounds wich only contains variants.
  // The vararrays returned by GetFieldValues do apply.
  var i : integer;
  begin
    Result := True;
    if (VarArrayHighBound(VarArray1,1))<> (VarArrayHighBound(VarArray2,1)) then exit;
    for i := 0 to VarArrayHighBound(VarArray1,1) do
    begin
      if VarArray1[i]<>VarArray2[i] then
        begin
        Result := false;
        Exit;
        end;
    end;
  end;

var I: Integer;
begin
  Result := Null;
  if VarIsNull(AKey) then Exit;
  i := FList.Count - 1;
  if VarIsArray(AKey) then
    while (i >= 0) And not VarArraySameValues(PLookupListRec(FList.Items[I])^.Key,AKey) do Dec(i)
  else
    while (i >= 0) And (PLookupListRec(FList.Items[I])^.Key <> AKey) do Dec(i);
  if i >= 0 then Result := PLookupListRec(FList.Items[I])^.Value;
end;

procedure TLookupList.ValuesToStrings(AStrings: TStrings);
var
  i: Integer;
  p: PLookupListRec;
begin
  AStrings.Clear;
  for i := 0 to FList.Count - 1 do
    begin
    p := PLookupListRec(FList[i]);
    AStrings.AddObject(p^.Value, TObject(p));
    end;
end;

procedure DisposeMem(var Buffer; Size: Integer);
begin
  if Pointer(Buffer) <> nil then
    begin
    FreeMem(Pointer(Buffer), Size);
    Pointer(Buffer) := nil;
    end;
end;

function BuffersEqual(Buf1, Buf2: Pointer; Size: Integer): Boolean; 

begin
  Result:=CompareByte(Buf1,Buf2,Size)=0
end;

{$i dataset.inc}
{$i fields.inc}
{$i datasource.inc}
{$i database.inc}
{$i dsparams.inc}

end.
