{
    This file is part of the Free Pascal Run Time Library (rtl)
    Copyright (c) 1999-2008 by Michael Van Canneyt, Florian Klaempfl,
    and Micha Nelissen

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$H+}


{$ifdef CLASSESINLINE}{$inline on}{$endif}


type
   { extra types to compile with FPC }
   HRSRC =  TFPResourceHandle deprecated;
   TComponentName = string;
   THandle = System.THandle;

   TPoint=Types.TPoint;
   TRect=Types.TRect;
   TSmallPoint=Types.TSmallPoint;

{$ifndef FPC_HAS_FEATURE_DYNLIBS}
   HMODULE = ptrint; 
{$else}
   HModule = System.HModule;
{$endif}

const

{ Maximum TList size }

{$ifdef cpu16}
  MaxListSize = {Maxint div 16}1024;
{$else cpu16}
  MaxListSize = Maxint div 16;
{$endif cpu16}

{ values for TShortCut }

  scShift = $2000;
  scCtrl = $4000;
  scAlt = $8000;
  scNone = 0;

{ TStream seek origins }
const
  soFromBeginning = 0;
  soFromCurrent = 1;
  soFromEnd = 2;

type
  TSeekOrigin = (soBeginning, soCurrent, soEnd);
  TDuplicates = Types.TDuplicates;

// For Delphi and backwards compatibility.
const
  dupIgnore = Types.dupIgnore;
  dupAccept = Types.dupAccept;
  dupError  = Types.dupError;

{ TFileStream create mode }
const
  fmCreate        = $FF00;
  fmOpenRead      = 0;
  fmOpenWrite     = 1;
  fmOpenReadWrite = 2;

{ TParser special tokens }

  toEOF     = Char(0);
  toSymbol  = Char(1);
  toString  = Char(2);
  toInteger = Char(3);
  toFloat   = Char(4);
  toWString = Char(5);

Const
  FilerSignature : Array[1..4] of char = 'TPF0';

type
{ Text alignment types }
  TAlignment = (taLeftJustify, taRightJustify, taCenter);

  TLeftRight = taLeftJustify..taRightJustify;
  TVerticalAlignment = (taAlignTop, taAlignBottom, taVerticalCenter);
  TTopBottom = taAlignTop..taAlignBottom;

  TBiDiMode = (bdLeftToRight,bdRightToLeft,bdRightToLeftNoAlign,bdRightToLeftReadingOnly);


{ Types used by standard events }
  TShiftStateEnum = (ssShift, ssAlt, ssCtrl,
    ssLeft, ssRight, ssMiddle, ssDouble,
    // Extra additions
    ssMeta, ssSuper, ssHyper, ssAltGr, ssCaps, ssNum,
    ssScroll,ssTriple,ssQuad,ssExtra1,ssExtra2);

{$packset 1}
  TShiftState = set of TShiftStateEnum;
{$packset default}

  THelpContext = -MaxLongint..MaxLongint;
  THelpType = (htKeyword, htContext);

  TShortCut = Low(Word)..High(Word);

{ Standard events }

  TNotifyEvent = procedure(Sender: TObject) of object;
  THelpEvent = function (Command: Word; Data: Longint;
    var CallHelp: Boolean): Boolean of object;
  TGetStrProc = procedure(const S: string) of object;

{ Exception classes }

  EStreamError = class(Exception);
  EFCreateError = class(EStreamError);
  EFOpenError = class(EStreamError);
  EFilerError = class(EStreamError);
  EReadError = class(EFilerError);
  EWriteError = class(EFilerError);
  EClassNotFound = class(EFilerError);
  EMethodNotFound = class(EFilerError);
  EInvalidImage = class(EFilerError);
  EResNotFound = class(Exception);
{$ifdef FPC_TESTGENERICS}
  EListError = fgl.EListError;
{$else}
  EListError = class(Exception);
{$endif}
  EBitsError = class(Exception);
  EStringListError = class(Exception);
  EComponentError = class(Exception);
  EParserError = class(Exception);
  EOutOfResources = class(EOutOfMemory);
  EInvalidOperation = class(Exception);
  TExceptionClass = Class of Exception;

{ ---------------------------------------------------------------------
  Free Pascal Observer support
  ---------------------------------------------------------------------}


Const
  SGUIDObserved = '{663C603C-3F3C-4CC5-823C-AC8079F979E5}';
  SGUIDObserver = '{BC7376EA-199C-4C2A-8684-F4805F0691CA}';

Type
  // Notification operations :
  // Observer has changed, is freed, item added to/deleted from list, custom event.
  TFPObservedOperation = (ooChange,ooFree,ooAddItem,ooDeleteItem,ooCustom);
{$INTERFACES CORBA}

  { IFPObserved }

  IFPObserved = Interface [SGUIDObserved]
    // attach a new observer
    Procedure FPOAttachObserver(AObserver : TObject);
    // Detach an observer
    Procedure FPODetachObserver(AObserver : TObject);
    // Notify all observers of a change.
    Procedure FPONotifyObservers(ASender : TObject; AOperation : TFPObservedOperation; Data : Pointer);
  end;

  { IFPObserver }

  IFPObserver = Interface  [SGUIDObserver]
    // Called by observed when observers are notified.
    Procedure FPOObservedChanged(ASender : TObject; Operation : TFPObservedOperation; Data : Pointer);
  end;
{$INTERFACES COM}

  EObserver = Class(Exception);

{ Forward class declarations }

  TStream = class;
  TFiler = class;
  TReader = class;
  TWriter = class;
  TComponent = class;

{ TFPList class }

  PPointerList = ^TPointerList;
  TPointerList = array[0..MaxListSize - 1] of Pointer;
  TListSortCompare = function (Item1, Item2: Pointer): Integer;
  TListCallback = Types.TListCallback;
  TListStaticCallback = Types.TListStaticCallback;


{$IFNDEF FPC_TESTGENERICS}

  TListAssignOp = (laCopy, laAnd, laOr, laXor, laSrcUnique, laDestUnique);
  TFPList = class;

  TFPListEnumerator = class
  private
    FList: TFPList;
    FPosition: Integer;
  public
    constructor Create(AList: TFPList);
    function GetCurrent: Pointer;
    function MoveNext: Boolean;
    property Current: Pointer read GetCurrent;
  end;

  TFPList = class(TObject)
  private
    FList: PPointerList;
    FCount: Integer;
    FCapacity: Integer;
    procedure CopyMove (aList : TFPList);
    procedure MergeMove (aList : TFPList);
    procedure DoCopy(ListA, ListB : TFPList);
    procedure DoSrcUnique(ListA, ListB : TFPList);
    procedure DoAnd(ListA, ListB : TFPList);
    procedure DoDestUnique(ListA, ListB : TFPList);
    procedure DoOr(ListA, ListB : TFPList);
    procedure DoXOr(ListA, ListB : TFPList);
  protected
    function Get(Index: Integer): Pointer; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    procedure Put(Index: Integer; Item: Pointer); {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    procedure SetCapacity(NewCapacity: Integer);
    procedure SetCount(NewCount: Integer);
    Procedure RaiseIndexError(Index: Integer); deprecated;
    Procedure CheckIndex(AIndex : Integer); {$ifdef CLASSESINLINE} inline;{$ENDIF}
  public
    Type
      TDirection = (FromBeginning, FromEnd);
    destructor Destroy; override;
    Procedure AddList(AList : TFPList);
    function Add(Item: Pointer): Integer; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    procedure Clear;
    procedure Delete(Index: Integer); {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    class procedure Error(const Msg: string; Data: PtrInt); static;
    procedure Exchange(Index1, Index2: Integer);
    function Expand: TFPList; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function Extract(Item: Pointer): Pointer;
    function First: Pointer;
    function GetEnumerator: TFPListEnumerator;
    function IndexOf(Item: Pointer): Integer;
    function IndexOfItem(Item: Pointer; Direction: TDirection): Integer;
    procedure Insert(Index: Integer; Item: Pointer); {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function Last: Pointer;
    procedure Move(CurIndex, NewIndex: Integer);
    procedure Assign (ListA: TFPList; AOperator: TListAssignOp=laCopy; ListB: TFPList=nil);
    function Remove(Item: Pointer): Integer;
    procedure Pack;
    procedure Sort(Compare: TListSortCompare);
    procedure Sort(Compare: TListSortCompare; SortingAlgorithm: PSortingAlgorithm);
    procedure Sort(Compare: TListSortComparer_Context; Context: Pointer);
    procedure Sort(Compare: TListSortComparer_Context; Context: Pointer; SortingAlgorithm: PSortingAlgorithm);
    procedure ForEachCall(proc2call:TListCallback;arg:pointer);
    procedure ForEachCall(proc2call:TListStaticCallback;arg:pointer);
    property Capacity: Integer read FCapacity write SetCapacity;
    property Count: Integer read FCount write SetCount;
    property Items[Index: Integer]: Pointer read Get write Put; default;
    property List: PPointerList read FList;
  end;

{$else}

  TFPPtrList = specialize TFPGList<Pointer>;

  TFPList = class(TFPPtrList)
  public
    procedure Assign(Source: TFPList);
    procedure Sort(Compare: TListSortCompare);
    procedure Sort(Compare: TListSortCompare; SortingAlgorithm: PSortingAlgorithm);
    procedure ForEachCall(Proc2call: TListCallback; Arg: Pointer);
    procedure ForEachCall(Proc2call: TListStaticCallback; Arg: Pointer);
  end;

{$endif}

{ TList class}

  TListNotification = (lnAdded, lnExtracted, lnDeleted);
  TList = class;

  TListEnumerator = class(TFPListEnumerator)
  public
    constructor Create(AList: TList);
  end;

  TList = class(TObject,IFPObserved)
  private
    FList: TFPList;
    FObservers : TFPList;
    procedure CopyMove (aList : TList);
    procedure MergeMove (aList : TList);
    procedure DoCopy(ListA, ListB : TList);
    procedure DoSrcUnique(ListA, ListB : TList);
    procedure DoAnd(ListA, ListB : TList);
    procedure DoDestUnique(ListA, ListB : TList);
    procedure DoOr(ListA, ListB : TList);
    procedure DoXOr(ListA, ListB : TList);
  protected
    function Get(Index: Integer): Pointer; inline;
    procedure Grow; virtual;
    procedure Put(Index: Integer; Item: Pointer);
    procedure Notify(Ptr: Pointer; Action: TListNotification); virtual;
    procedure SetCapacity(NewCapacity: Integer); inline;
    function GetCapacity: Integer; inline;
    procedure SetCount(NewCount: Integer);
    function GetCount: Integer; inline;
    function GetList: PPointerList; inline;
  public
    constructor Create;
    destructor Destroy; override;
    Procedure FPOAttachObserver(AObserver : TObject);
    Procedure FPODetachObserver(AObserver : TObject);
    Procedure FPONotifyObservers(ASender : TObject; AOperation : TFPObservedOperation; Data : Pointer);
    Procedure AddList(AList : TList);
    function Add(Item: Pointer): Integer;
    procedure Clear; virtual;
    procedure Delete(Index: Integer);
    class procedure Error(const Msg: string; Data: PtrInt); virtual;
    procedure Exchange(Index1, Index2: Integer);
    function Expand: TList;
    function Extract(item: Pointer): Pointer;
    function First: Pointer; inline;
    function GetEnumerator: TListEnumerator;
    function IndexOf(Item: Pointer): Integer; inline;
    procedure Insert(Index: Integer; Item: Pointer);
    function Last: Pointer; inline;
    procedure Move(CurIndex, NewIndex: Integer); inline;
    procedure Assign (ListA: TList; AOperator: TListAssignOp=laCopy; ListB: TList=nil);
    function Remove(Item: Pointer): Integer;
    procedure Pack; inline;
    procedure Sort(Compare: TListSortCompare); inline;
    procedure Sort(Compare: TListSortCompare; SortingAlgorithm: PSortingAlgorithm); inline;
    procedure Sort(Compare: TListSortComparer_Context; Context: Pointer); inline;
    procedure Sort(Compare: TListSortComparer_Context; Context: Pointer; SortingAlgorithm: PSortingAlgorithm); inline;
    property Capacity: Integer read GetCapacity write SetCapacity;
    property Count: Integer read GetCount write SetCount;
    property Items[Index: Integer]: Pointer read Get write Put; default;
    property List: PPointerList read GetList;
  end;

{ TThreadList class }

  TThreadList = class
  private
    FList: TList;
    FDuplicates: TDuplicates;
    FLock: TRTLCriticalSection;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Add(Item: Pointer);
    procedure Clear;
    function  LockList: TList;
    procedure Remove(Item: Pointer);
    procedure UnlockList;
    property Duplicates: TDuplicates read FDuplicates write FDuplicates;
  end;

{TBits Class}

const
   BITSHIFT = 5;
   MASK = 31; {for longs that are 32-bit in size}
   // to further increase, signed integer limits have to be researched.
{$ifdef cpu16}
   MaxBitFlags = $7FE0;
{$else cpu16}
   MaxBitFlags = $7FFFFFE0;
{$endif cpu16}
   MaxBitRec = MaxBitFlags Div (SizeOf(cardinal)*8);
type
   TBitArray = array[0..MaxBitRec - 1] of cardinal;

   TBits = class(TObject)
   private
      { Private declarations }
      FBits : ^TBitArray;
      FSize : longint;  { total longints currently allocated }
      FBSize: longint;  {total bits currently allocated}
      findIndex : longint;
      findState : boolean;

      { functions and properties to match TBits class }
      procedure SetBit(bit : longint; value : Boolean);
      procedure SetSize(value : longint);
   Protected
      procedure CheckBitIndex (Bit : longint;CurrentSize : Boolean);
   public
      { Public declarations }
      constructor Create(TheSize : longint = 0); virtual;
      destructor Destroy; override;
      function  GetFSize : longint;
      procedure SetOn(Bit : longint);
      procedure Clear(Bit : longint);
      procedure Clearall;
      procedure CopyBits(BitSet : TBits);
      procedure AndBits(BitSet : TBits);
      procedure OrBits(BitSet : TBits);
      procedure XorBits(BitSet : TBits);
      procedure NotBits(BitSet : TBits);
      function  Get(Bit : longint) : boolean;
      procedure Grow(NBit : longint);
      function  Equals(Obj : TObject): Boolean; override; overload;
      function  Equals(BitSet : TBits) : Boolean; overload;
      procedure SetIndex(Index : longint);
      function  FindFirstBit(State : boolean) : longint;
      function  FindNextBit : longint;
      function  FindPrevBit : longint;

      { functions and properties to match TBits class }
      function OpenBit: longint;
      property Bits[Bit: longint]: Boolean read get write SetBit; default;
      property Size: longint read FBSize write setSize;
   end;

{ TPersistent abstract class }

{$M+}

  TPersistent = class(TObject,IFPObserved)
  private
    FObservers : TFPList;
    procedure AssignError(Source: TPersistent);
  protected
    procedure AssignTo(Dest: TPersistent); virtual;
    procedure DefineProperties(Filer: TFiler); virtual;
    function  GetOwner: TPersistent; dynamic;
  public
    Destructor Destroy; override;
    procedure Assign(Source: TPersistent); virtual;
    Procedure FPOAttachObserver(AObserver : TObject);
    Procedure FPODetachObserver(AObserver : TObject);
    Procedure FPONotifyObservers(ASender : TObject; AOperation : TFPObservedOperation; Data : Pointer);
    function  GetNamePath: string; virtual; {dynamic;}
  end;

{$M-}

{ TPersistent class reference type }

  TPersistentClass = class of TPersistent;

{ TInterfaced Persistent }

  TInterfacedPersistent = class(TPersistent, IInterface)
  private
    FOwnerInterface: IInterface;
  protected
    { IInterface }
    function _AddRef: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    function _Release: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
  public
    function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} IID: TGUID; out Obj): HResult; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    procedure AfterConstruction; override;
  end;

{ TRecall class }

  TRecall = class(TObject)
  private
    FStorage, FReference: TPersistent;
  public
    constructor Create(AStorage, AReference: TPersistent);
    destructor Destroy; override;
    procedure Store;
    procedure Forget;
    property Reference: TPersistent read FReference;
  end;

{ TCollection class }

  TCollection = class;

  TCollectionItem = class(TPersistent)
  private
    FCollection: TCollection;
    FID: Integer;
    FUpdateCount: Integer;
    function GetIndex: Integer;
  protected
    procedure SetCollection(Value: TCollection);virtual;
    procedure Changed(AllItems: Boolean);
    function GetOwner: TPersistent; override;
    function GetDisplayName: string; virtual;
    procedure SetIndex(Value: Integer); virtual;
    procedure SetDisplayName(const Value: string); virtual;
    property UpdateCount: Integer read FUpdateCount;
  public
    constructor Create(ACollection: TCollection); virtual;
    destructor Destroy; override;
    function GetNamePath: string; override;
    property Collection: TCollection read FCollection write SetCollection;
    property ID: Integer read FID;
    property Index: Integer read GetIndex write SetIndex;
    property DisplayName: string read GetDisplayName write SetDisplayName;
  end;

  TCollectionEnumerator = class
  private
    FCollection: TCollection;
    FPosition: Integer;
  public
    constructor Create(ACollection: TCollection);
    function GetCurrent: TCollectionItem;
    function MoveNext: Boolean;
    property Current: TCollectionItem read GetCurrent;
  end;

  TCollectionItemClass = class of TCollectionItem;
  TCollectionNotification = (cnAdded, cnExtracting, cnDeleting);
  TCollectionSortCompare = function (Item1, Item2: TCollectionItem): Integer;

  TCollection = class(TPersistent)
  private
    FItemClass: TCollectionItemClass;
    FItems: TFpList;
    FUpdateCount: Integer;
    FNextID: Integer;
    FPropName: string;
    function GetCount: Integer;
    function GetPropName: string;
    procedure InsertItem(Item: TCollectionItem);
    procedure RemoveItem(Item: TCollectionItem);
    procedure DoClear;
  protected
    { Design-time editor support }
    function GetAttrCount: Integer; dynamic;
    function GetAttr(Index: Integer): string; dynamic;
    function GetItemAttr(Index, ItemIndex: Integer): string; dynamic;
    procedure Changed;
    function GetItem(Index: Integer): TCollectionItem;
    procedure SetItem(Index: Integer; Value: TCollectionItem);
    procedure SetItemName(Item: TCollectionItem); virtual;
    procedure SetPropName; virtual;
    procedure Update(Item: TCollectionItem); virtual;
    procedure Notify(Item: TCollectionItem;Action: TCollectionNotification); virtual;
    property PropName: string read GetPropName write FPropName;
    property UpdateCount: Integer read FUpdateCount;
  public
    constructor Create(AItemClass: TCollectionItemClass);
    destructor Destroy; override;
    function Owner: TPersistent;
    function Add: TCollectionItem;
    procedure Assign(Source: TPersistent); override;
    procedure BeginUpdate; virtual;
    procedure Clear;
    procedure EndUpdate; virtual;
    procedure Delete(Index: Integer);
    function GetEnumerator: TCollectionEnumerator;
    function GetNamePath: string; override;
    function Insert(Index: Integer): TCollectionItem;
    function FindItemID(ID: Integer): TCollectionItem;
    procedure Exchange(Const Index1, index2: integer);
    procedure Move(Const Index1, index2: integer);
    procedure Sort(Const Compare : TCollectionSortCompare);
    property Count: Integer read GetCount;
    property ItemClass: TCollectionItemClass read FItemClass;
    property Items[Index: Integer]: TCollectionItem read GetItem write SetItem;
  end;

  TOwnedCollection = class(TCollection)
  private
    FOwner: TPersistent;
  protected
    Function GetOwner: TPersistent; override;
  public
    Constructor Create(AOwner: TPersistent;AItemClass: TCollectionItemClass);
  end;


  TStrings = class;

{ IStringsAdapter interface }

  { Maintains link between TStrings and IStrings implementations }
  IStringsAdapter = interface ['{739C2F34-52EC-11D0-9EA6-0020AF3D82DA}']
    procedure ReferenceStrings(S: TStrings);
    procedure ReleaseStrings;
  end;

{ TStringsEnumerator class }

  TStringsEnumerator = class
  private
    FStrings: TStrings;
    FPosition: Integer;
  public
    constructor Create(AStrings: TStrings);
    function GetCurrent: String;
    function MoveNext: Boolean;
    property Current: String read GetCurrent;
  end;

{ TStrings class }
  TStringsFilterMethod = function(const s: string): boolean of object;
  TStringsReduceMethod = function(const s1, s2: string): string of object;
  TStringsMapMethod = function(const s: string): string of object;
  TStringsForEachMethodExObj = procedure(const CurrentValue: string; const index: integer; Obj : TObject) of object;
  TStringsForEachMethodEx = procedure(const CurrentValue: string; const index: integer) of object;
  TStringsForEachMethod = procedure(const CurrentValue: string) of object;
  TMissingNameValueSeparatorAction = (mnvaValue,mnvaName,mnvaEmpty,mnvaError);
  TMissingNameValueSeparatorActions = set of TMissingNameValueSeparatorAction;
  TStringsOption = (soStrictDelimiter,soWriteBOM,soTrailingLineBreak,soUseLocale,soPreserveBOM);
  TStringsOptions = set of TStringsOption;

  TStrings = class(TPersistent)
  private
    FDefaultEncoding: TEncoding;
    FEncoding: TEncoding;
    FMissingNameValueSeparatorAction: TMissingNameValueSeparatorAction;
    FSpecialCharsInited : boolean;
    FAlwaysQuote: Boolean;
    FQuoteChar : Char;
    FDelimiter : Char;
    FNameValueSeparator : Char;
    FUpdateCount: Integer;
    FAdapter: IStringsAdapter;
    FLBS : TTextLineBreakStyle;
    FOptions : TStringsOptions;
    FLineBreak : String;
    function GetCommaText: string;
    function GetLineBreakCharLBS: string;
    function GetMissingNameValueSeparatorAction: TMissingNameValueSeparatorAction;
    function GetName(Index: Integer): string;
    function GetStrictDelimiter: Boolean;
    function GetTrailingLineBreak: Boolean;
    function GetUseLocale: Boolean;
    function GetValue(const Name: string): string;
    function GetWriteBOM: Boolean;
    Function GetLBS : TTextLineBreakStyle;
    procedure SetDefaultEncoding(const ADefaultEncoding: TEncoding);
    procedure SetEncoding(const AEncoding: TEncoding);
    Procedure SetLBS (AValue : TTextLineBreakStyle);
    procedure ReadData(Reader: TReader);
    procedure SetCommaText(const Value: string);
    procedure SetMissingNameValueSeparatorAction(AValue: TMissingNameValueSeparatorAction);
    procedure SetStringsAdapter(const Value: IStringsAdapter);
    procedure SetStrictDelimiter(AValue: Boolean);
    procedure SetTrailingLineBreak(AValue: Boolean);
    procedure SetUseLocale(AValue: Boolean);
    procedure SetWriteBOM(AValue: Boolean);
    procedure SetValue(const Name, Value: string);
    procedure SetDelimiter(c:Char);
    procedure SetQuoteChar(c:Char);
    procedure SetNameValueSeparator(c:Char);
    procedure WriteData(Writer: TWriter);
    procedure DoSetTextStr(const Value: string; DoClear : Boolean);
    Function GetDelimiter : Char;
    Function GetNameValueSeparator : Char;
    Function GetQuoteChar: Char;
    Function GetLineBreak : String;
    procedure SetLineBreak(const S : String);
    Function GetSkipLastLineBreak : Boolean;
    procedure SetSkipLastLineBreak(const AValue : Boolean);
    Procedure DoSetDelimitedText(const AValue: string; DoClear,aStrictDelimiter : Boolean; aQuoteChar,aDelimiter : Char);
  protected
    function CompareStrings(const s1,s2 : string) : Integer; virtual;
    procedure DefineProperties(Filer: TFiler); override;
    procedure Error(const Msg: string; Data: Integer);
    procedure Error(const Msg: pstring; Data: Integer);
    function Get(Index: Integer): string; virtual; abstract;
    function GetCapacity: Integer; virtual;
    function GetCount: Integer; virtual; abstract;
    function GetObject(Index: Integer): TObject; virtual;
    function GetTextStr: string; virtual;
    procedure Put(Index: Integer; const S: string); virtual;
    procedure PutObject(Index: Integer; AObject: TObject); virtual;
    procedure SetCapacity(NewCapacity: Integer); virtual;
    procedure SetTextStr(const Value: string); virtual;
    procedure SetUpdateState(Updating: Boolean); virtual;
    property UpdateCount: Integer read FUpdateCount;
    Function DoCompareText(const s1,s2 : string) : PtrInt; virtual;
    Function GetDelimitedText: string;
    Procedure SetDelimitedText(Const AValue: string);
    Function GetValueFromIndex(Index: Integer): string;
    Procedure SetValueFromIndex(Index: Integer; const Value: string);
    Procedure CheckSpecialChars;
    Class Function GetNextLine (Const Value : String; Var S : String; Var P : SizeInt) : Boolean;
    Function GetNextLinebreak (Const Value : String; Var S : String; Var P : SizeInt) : Boolean;
    {$IF (SizeOf(Integer) < SizeOf(SizeInt)) }
    class function GetNextLine(const Value: string; var S: string; var P: Integer) : Boolean; deprecated;
    function GetNextLineBreak(const Value: string; var S: string; var P: Integer) : Boolean; deprecated;
    {$IFEND}
  public
    constructor Create;
    destructor Destroy; override;
    function Add(const S: string): Integer; virtual; overload;
    function AddObject(const S: string; AObject: TObject): Integer; virtual; overload;
    function Add(const Fmt : string; const Args : Array of const): Integer; overload;
    function AddObject(const Fmt: string; Args : Array of const; AObject: TObject): Integer; overload;
    function AddPair(const AName, AValue: string): TStrings; overload; {$IFDEF CLASSESINLINE}inline;{$ENDIF}
    function AddPair(const AName, AValue: string; AObject: TObject): TStrings; overload;
    procedure AddStrings(TheStrings: TStrings); overload; virtual;
    procedure AddStrings(TheStrings: TStrings; ClearFirst : Boolean); overload;
    procedure AddStrings(const TheStrings: array of string); overload; virtual;
    procedure AddStrings(const TheStrings: array of string; ClearFirst : Boolean); overload;
    Procedure AddText(Const S : String); virtual;
    procedure AddCommaText(const S: String);
    procedure AddDelimitedText(const S: String; ADelimiter: char; AStrictDelimiter: Boolean); overload;
    procedure AddDelimitedtext(const S: String); overload;
    procedure Append(const S: string);
    procedure Assign(Source: TPersistent); override;
    procedure BeginUpdate;
    procedure Clear; virtual; abstract;
    procedure Delete(Index: Integer); virtual; abstract;
    procedure EndUpdate;
    function Equals(Obj: TObject): Boolean; override; overload;
    function Equals(TheStrings: TStrings): Boolean; overload;
    procedure Exchange(Index1, Index2: Integer); virtual;
    function  ExtractName(Const S:String):String;
    Procedure Filter(aFilter: TStringsFilterMethod; aList : TStrings);
    Function Filter(aFilter: TStringsFilterMethod) :  TStrings;
    Procedure Fill(const aValue : String; aStart,aEnd : Integer);
    procedure ForEach(aCallback: TStringsForeachMethod);
    procedure ForEach(aCallback: TStringsForeachMethodEx);
    procedure ForEach(aCallback: TStringsForeachMethodExObj);
    function GetEnumerator: TStringsEnumerator;
    procedure GetNameValue(Index : Integer; Out AName,AValue : String);
    function GetText: PChar; virtual;
    function IndexOf(const S: string): Integer; virtual;
    function IndexOf(const S: string; aStart : Integer): Integer; virtual;
    function IndexOfName(const Name: string): Integer; virtual;
    function IndexOfObject(AObject: TObject): Integer; virtual;
    procedure Insert(Index: Integer; const S: string); virtual; abstract;
    procedure InsertObject(Index: Integer; const S: string; AObject: TObject);
    function LastIndexOf(const S: string; aStart : Integer): Integer; virtual;
    function LastIndexOf(const S: string): Integer;
    procedure LoadFromFile(const FileName: string); overload; virtual;
    procedure LoadFromFile(const FileName: string; IgnoreEncoding : Boolean);
    procedure LoadFromFile(const FileName: string; AEncoding: TEncoding); overload; virtual;
    procedure LoadFromStream(Stream: TStream); overload; virtual;
    procedure LoadFromStream(Stream: TStream; IgnoreEncoding : Boolean); overload;
    procedure LoadFromStream(Stream: TStream; AEncoding: TEncoding); overload; virtual;
    Procedure Map(aMap: TStringsMapMethod; aList : TStrings);
    Function Map(aMap: TStringsMapMethod) : TStrings;
    procedure Move(CurIndex, NewIndex: Integer); virtual;
    Function Pop : String;
    function Reduce(aReduceMethod: TStringsReduceMethod; const startingValue: string): string;
    Function Reverse : TStrings;
    Procedure Reverse(aList : TStrings);
    procedure SaveToFile(const FileName: string); overload; virtual;
    procedure SaveToFile(const FileName: string; IgnoreEncoding : Boolean); overload;
    procedure SaveToFile(const FileName: string; AEncoding: TEncoding); overload; virtual;
    procedure SaveToStream(Stream: TStream); overload; virtual;
    procedure SaveToStream(Stream: TStream; IgnoreEncoding : Boolean); overload;
    procedure SaveToStream(Stream: TStream; AEncoding: TEncoding); overload; virtual;
    function Shift : String;
    Procedure Slice(fromIndex: integer; aList : TStrings);
    Function Slice(fromIndex: integer) : TStrings;
    procedure SetText(TheText: PChar); virtual;
    property AlwaysQuote: Boolean read FAlwaysQuote write FAlwaysQuote;
    property Capacity: Integer read GetCapacity write SetCapacity;
    property CommaText: string read GetCommaText write SetCommaText;
    property Count: Integer read GetCount;
    property DefaultEncoding: TEncoding read FDefaultEncoding write SetDefaultEncoding;
    property DelimitedText: string read GetDelimitedText write SetDelimitedText;
    property Delimiter: Char read GetDelimiter write SetDelimiter;
    property Encoding: TEncoding read FEncoding;
    property LineBreak : string Read GetLineBreak write SetLineBreak;
    Property MissingNameValueSeparatorAction : TMissingNameValueSeparatorAction Read GetMissingNameValueSeparatorAction Write SetMissingNameValueSeparatorAction;
    property Names[Index: Integer]: string read GetName;
    Property NameValueSeparator : Char Read GetNameValueSeparator Write SetNameValueSeparator;
    property Objects[Index: Integer]: TObject read GetObject write PutObject;
    property Options: TStringsOptions read FOptions write FOptions;
    property QuoteChar: Char read GetQuoteChar write SetQuoteChar;
    Property SkipLastLineBreak : Boolean Read GetSkipLastLineBreak Write SetSkipLastLineBreak;
    // Same as SkipLastLineBreak but for Delphi compatibility. Note it has opposite meaning.
    Property TrailingLineBreak : Boolean Read GetTrailingLineBreak Write SetTrailingLineBreak;
    Property StrictDelimiter : Boolean Read GetStrictDelimiter Write SetStrictDelimiter;
    property Strings[Index: Integer]: string read Get write Put; default;
    property StringsAdapter: IStringsAdapter read FAdapter write SetStringsAdapter;
    property Text: string read GetTextStr write SetTextStr;
    Property TextLineBreakStyle : TTextLineBreakStyle Read GetLBS Write SetLBS;
    Property UseLocale : Boolean Read GetUseLocale Write SetUseLocale;
    property ValueFromIndex[Index: Integer]: string read GetValueFromIndex write SetValueFromIndex;
    property Values[const Name: string]: string read GetValue write SetValue;
    property WriteBOM: Boolean read GetWriteBOM write SetWriteBOM;
  end;
  TStringsClass = Class of TStrings;

{ TStringList class }

  TStringList = class;

  TStringListSortCompare = function(List: TStringList; Index1, Index2: Integer): Integer;

{$IFNDEF FPC_TESTGENERICS}

  PStringItem = ^TStringItem;
  TStringItem = record
    FString: string;
    FObject: TObject;
  end;

  PStringItemList = ^TStringItemList;
  TStringItemList = array[0..MaxListSize] of TStringItem;

  TStringsSortStyle = (sslNone,sslUser,sslAuto);
  TStringsSortStyles = Set of TStringsSortStyle;

  TStringList = class(TStrings)
  private
    FList: PStringItemList;
    FCount: Integer;
    FCapacity: Integer;
    FOnChange: TNotifyEvent;
    FOnChanging: TNotifyEvent;
    FDuplicates: TDuplicates;
    FCaseSensitive : Boolean;
    FForceSort : Boolean;
    FOwnsObjects : Boolean;
    FSortStyle: TStringsSortStyle;
    procedure ExchangeItemsInt(Index1, Index2: Integer); inline;
    function GetSorted: Boolean;
    procedure Grow;
    procedure InternalClear(FromIndex : Integer = 0; ClearOnly : Boolean = False);
    procedure SetSorted(Value: Boolean);
    procedure SetCaseSensitive(b : boolean);
    procedure SetSortStyle(AValue: TStringsSortStyle);
  protected
    Procedure CheckIndex(AIndex : Integer); inline;
    procedure ExchangeItems(Index1, Index2: Integer); virtual;
    procedure Changed; virtual;
    procedure Changing; virtual;
    function Get(Index: Integer): string; override;
    function GetCapacity: Integer; override;
    function GetCount: Integer; override;
    function GetObject(Index: Integer): TObject; override;
    procedure Put(Index: Integer; const S: string); override;
    procedure PutObject(Index: Integer; AObject: TObject); override;
    procedure SetCapacity(NewCapacity: Integer); override;
    procedure SetUpdateState(Updating: Boolean); override;
    procedure InsertItem(Index: Integer; const S: string); virtual;
    procedure InsertItem(Index: Integer; const S: string; O: TObject); virtual;
    Function DoCompareText(const s1,s2 : string) : PtrInt; override;

  public
    destructor Destroy; override;
    function Add(const S: string): Integer; override;
    procedure Clear; override;
    procedure Delete(Index: Integer); override;
    procedure Exchange(Index1, Index2: Integer); override;
    function Find(const S: string; Out Index: Integer): Boolean; virtual;
    function IndexOf(const S: string): Integer; override;
    procedure Insert(Index: Integer; const S: string); override;
    procedure Sort; virtual;
    procedure Sort(SortingAlgorithm: PSortingAlgorithm); virtual;
    procedure CustomSort(CompareFn: TStringListSortCompare); virtual;
    procedure CustomSort(CompareFn: TStringListSortCompare; SortingAlgorithm: PSortingAlgorithm); virtual;
    property Duplicates: TDuplicates read FDuplicates write FDuplicates;
    property Sorted: Boolean read GetSorted write SetSorted;
    property CaseSensitive: Boolean read FCaseSensitive write SetCaseSensitive;
    property OnChange: TNotifyEvent read FOnChange write FOnChange;
    property OnChanging: TNotifyEvent read FOnChanging write FOnChanging;
    property OwnsObjects : boolean read FOwnsObjects write FOwnsObjects;
    Property SortStyle : TStringsSortStyle Read FSortStyle Write SetSortStyle;
  end;

{$else}

  TFPStrObjMap = specialize TFPGMap<string, TObject>;

  TStringListTextCompare = function(const s1, s2: string): PtrInt of object;

  TStringList = class(TStrings)
  private
    FMap: TFPStrObjMap;
    FCaseSensitive: Boolean;
    FOnChange: TNotifyEvent;
    FOnChanging: TNotifyEvent;
    FOnCompareText: TStringListTextCompare;
    FOwnsObjects : Boolean;
    procedure SetCaseSensitive(NewSensitive: Boolean);
  protected
    procedure Changed; virtual;
    procedure Changing; virtual;
    function DefaultCompareText(const s1, s2: string): PtrInt;
    function DoCompareText(const s1, s2: string): PtrInt; override;
    function Get(Index: Integer): string; override;
    function GetCapacity: Integer; override;
    function GetDuplicates: TDuplicates;
    function GetCount: Integer; override;
    function GetObject(Index: Integer): TObject; override;
    function GetSorted: Boolean; {$ifdef CLASSESINLINE} inline; {$endif}
    function MapPtrCompare(Key1, Key2: Pointer): Integer;
    procedure Put(Index: Integer; const S: string); override;
    procedure PutObject(Index: Integer; AObject: TObject); override;
    procedure QuickSort(L, R: Integer; CompareFn: TStringListSortCompare);
    procedure SetCapacity(NewCapacity: Integer); override;
    procedure SetDuplicates(NewDuplicates: TDuplicates);
    procedure SetSorted(NewSorted: Boolean); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetUpdateState(Updating: Boolean); override;
  public
    constructor Create;
    destructor Destroy; override;
    function Add(const S: string): Integer; override;
    procedure Clear; override;
    procedure Delete(Index: Integer); override;
    procedure Exchange(Index1, Index2: Integer); override;
    function Find(const S: string; var Index: Integer): Boolean; virtual;
    function IndexOf(const S: string): Integer; override;
    procedure Insert(Index: Integer; const S: string); override;
    procedure Sort; virtual;
    procedure Sort(SortingAlgorithm: PSortingAlgorithm); virtual;
    procedure CustomSort(CompareFn: TStringListSortCompare);
    procedure CustomSort(CompareFn: TStringListSortCompare; SortingAlgorithm: PSortingAlgorithm);
    property Duplicates: TDuplicates read GetDuplicates write SetDuplicates;
    property Sorted: Boolean read GetSorted write SetSorted;
    property CaseSensitive: Boolean read FCaseSensitive write SetCaseSensitive;
    property OnChange: TNotifyEvent read FOnChange write FOnChange;
    property OnChanging: TNotifyEvent read FOnChanging write FOnChanging;
    property OnCompareText: TStringListTextCompare read FOnCompareText write FOnCompareText;
    property OwnsObjects : boolean read FOwnsObjects write FOwnsObjects;
  end;

{$endif}


{ TStream abstract class }

  TStream = class(TObject)
  private
  protected
    procedure InvalidSeek; virtual;
    procedure Discard(const Count: Int64);
    procedure DiscardLarge(Count: int64; const MaxBufferSize: Longint);
    procedure FakeSeekForward(Offset: Int64; const Origin: TSeekOrigin; const Pos: Int64);
    function  GetPosition: Int64; virtual;
    procedure SetPosition(const Pos: Int64); virtual;
    function  GetSize: Int64; virtual;
    procedure SetSize64(const NewSize: Int64); virtual;
    procedure SetSize(NewSize: Longint); virtual;overload;
    procedure SetSize(const NewSize: Int64); virtual;overload;
    procedure ReadNotImplemented;
    procedure WriteNotImplemented;
    function ReadMaxSizeData(Var Buffer; aSize,aCount : NativeInt) : NativeInt;
    Procedure ReadExactSizeData(Var Buffer; aSize,aCount : NativeInt);
    function WriteMaxSizeData(Const Buffer; aSize,aCount : NativeInt) : NativeInt;
    Procedure WriteExactSizeData(Const Buffer; aSize,aCount : NativeInt);
  public
    function Read(var Buffer; Count: Longint): Longint; virtual; overload;
    function Read(var Buffer: TBytes; Count: Longint): Longint; overload;
    function Read( Buffer : TBytes; aOffset, Count: Longint): Longint; overload;

    function Write(const Buffer: TBytes; Offset, Count: Longint): Longint; overload;
    function Write(const Buffer: TBytes; Count: Longint): Longint; overload;
    function Write(const Buffer; Count: Longint): Longint; virtual; overload;

    function Seek(Offset: Longint; Origin: Word): Longint; virtual; overload;
    function Seek(const Offset: Int64; Origin: TSeekOrigin): Int64; virtual; overload;

    function ReadData(Buffer: Pointer; Count: NativeInt): NativeInt; overload;
    function ReadData(Buffer: TBytes; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Boolean): NativeInt; overload;
    function ReadData(var Buffer: Boolean; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: AnsiChar): NativeInt; overload;
    function ReadData(var Buffer: AnsiChar; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: WideChar): NativeInt; overload;
    function ReadData(var Buffer: WideChar; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Int8): NativeInt; overload;
    function ReadData(var Buffer: Int8; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: UInt8): NativeInt; overload;
    function ReadData(var Buffer: UInt8; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Int16): NativeInt; overload;
    function ReadData(var Buffer: Int16; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: UInt16): NativeInt; overload;
    function ReadData(var Buffer: UInt16; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Int32): NativeInt; overload;
    function ReadData(var Buffer: Int32; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: UInt32): NativeInt; overload;
    function ReadData(var Buffer: UInt32; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Int64): NativeInt; overload;
    function ReadData(var Buffer: Int64; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: UInt64): NativeInt; overload;
    function ReadData(var Buffer: UInt64; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Single): NativeInt; overload;
    function ReadData(var Buffer: Single; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: Double): NativeInt; overload;
    function ReadData(var Buffer: Double; Count: NativeInt): NativeInt; overload;
{$IFDEF FPC_HAS_TYPE_EXTENDED}
    function ReadData(var Buffer: Extended): NativeInt; overload;
    function ReadData(var Buffer: Extended; Count: NativeInt): NativeInt; overload;
    function ReadData(var Buffer: TExtended80Rec): NativeInt; overload;
    function ReadData(var Buffer: TExtended80Rec; Count: NativeInt): NativeInt; overload;
{$ENDIF}
    procedure ReadBuffer(var Buffer; Count: Longint);
    procedure ReadBuffer(var Buffer: TBytes; Count: NativeInt); overload;
    procedure ReadBuffer(var Buffer: TBytes; Offset, Count: NativeInt); overload;

    procedure ReadBufferData(var Buffer: Boolean); overload;
    procedure ReadBufferData(var Buffer: Boolean; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: AnsiChar); overload;
    procedure ReadBufferData(var Buffer: AnsiChar; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: WideChar); overload;
    procedure ReadBufferData(var Buffer: WideChar; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: Int8); overload;
    procedure ReadBufferData(var Buffer: Int8; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: UInt8); overload;
    procedure ReadBufferData(var Buffer: UInt8; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: Int16); overload;
    procedure ReadBufferData(var Buffer: Int16; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: UInt16); overload;
    procedure ReadBufferData(var Buffer: UInt16; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: Int32); overload;
    procedure ReadBufferData(var Buffer: Int32; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: UInt32); overload;
    procedure ReadBufferData(var Buffer: UInt32; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: Int64); overload;
    procedure ReadBufferData(var Buffer: Int64; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: UInt64); overload;
    procedure ReadBufferData(var Buffer: UInt64; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: Single); overload;
    procedure ReadBufferData(var Buffer: Single; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: Double); overload;
    procedure ReadBufferData(var Buffer: Double; Count: NativeInt); overload;
{$IFDEF FPC_HAS_TYPE_EXTENDED}
    procedure ReadBufferData(var Buffer: Extended); overload;
    procedure ReadBufferData(var Buffer: Extended; Count: NativeInt); overload;
    procedure ReadBufferData(var Buffer: TExtended80Rec); overload;
    procedure ReadBufferData(var Buffer: TExtended80Rec; Count: NativeInt); overload;
{$ENDIF}
    procedure WriteBuffer(const Buffer; Count: Longint);
    procedure WriteBuffer(const Buffer: TBytes; Count: NativeInt); overload;
    procedure WriteBuffer(const Buffer: TBytes; Offset, Count: NativeInt); overload;

    function WriteData(const Buffer: TBytes; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Pointer; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Boolean): NativeInt; overload;
    function WriteData(const Buffer: Boolean; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: AnsiChar): NativeInt; overload;
    function WriteData(const Buffer: AnsiChar; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: WideChar): NativeInt; overload;
    function WriteData(const Buffer: WideChar; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Int8): NativeInt; overload;
    function WriteData(const Buffer: Int8; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: UInt8): NativeInt; overload;
    function WriteData(const Buffer: UInt8; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Int16): NativeInt; overload;
    function WriteData(const Buffer: Int16; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: UInt16): NativeInt; overload;
    function WriteData(const Buffer: UInt16; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Int32): NativeInt; overload;
    function WriteData(const Buffer: Int32; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: UInt32): NativeInt; overload;
    function WriteData(const Buffer: UInt32; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Int64): NativeInt; overload;
    function WriteData(const Buffer: Int64; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: UInt64): NativeInt; overload;
    function WriteData(const Buffer: UInt64; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Single): NativeInt; overload;
    function WriteData(const Buffer: Single; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: Double): NativeInt; overload;
    function WriteData(const Buffer: Double; Count: NativeInt): NativeInt; overload;
{$IFDEF FPC_HAS_TYPE_EXTENDED}
    function WriteData(const Buffer: Extended): NativeInt; overload;
    function WriteData(const Buffer: Extended; Count: NativeInt): NativeInt; overload;
    function WriteData(const Buffer: TExtended80Rec): NativeInt; overload;
    function WriteData(const Buffer: TExtended80Rec; Count: NativeInt): NativeInt; overload;
{$ENDIF}
    procedure WriteBufferData(Buffer: Int32); overload;
    procedure WriteBufferData(Buffer: Int32; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: Boolean); overload;
    procedure WriteBufferData(Buffer: Boolean; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: AnsiChar); overload;
    procedure WriteBufferData(Buffer: AnsiChar; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: WideChar); overload;
    procedure WriteBufferData(Buffer: WideChar; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: Int8); overload;
    procedure WriteBufferData(Buffer: Int8; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: UInt8); overload;
    procedure WriteBufferData(Buffer: UInt8; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: Int16); overload;
    procedure WriteBufferData(Buffer: Int16; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: UInt16); overload;
    procedure WriteBufferData(Buffer: UInt16; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: UInt32); overload;
    procedure WriteBufferData(Buffer: UInt32; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: Int64); overload;
    procedure WriteBufferData(Buffer: Int64; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: UInt64); overload;
    procedure WriteBufferData(Buffer: UInt64; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: Single); overload;
    procedure WriteBufferData(Buffer: Single; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: Double); overload;
    procedure WriteBufferData(Buffer: Double; Count: NativeInt); overload;
{$IFDEF FPC_HAS_TYPE_EXTENDED}
    procedure WriteBufferData(Buffer: Extended); overload;
    procedure WriteBufferData(Buffer: Extended; Count: NativeInt); overload;
    procedure WriteBufferData(Buffer: TExtended80Rec); overload;
    procedure WriteBufferData(Buffer: TExtended80Rec; Count: NativeInt); overload;
{$ENDIF}
    function CopyFrom(Source: TStream; Count: Int64): Int64;
    function ReadComponent(Instance: TComponent): TComponent;
    function ReadComponentRes(Instance: TComponent): TComponent;
    procedure WriteComponent(Instance: TComponent);
    procedure WriteComponentRes(const ResName: string; Instance: TComponent);
    procedure WriteDescendent(Instance, Ancestor: TComponent);
    procedure WriteDescendentRes(const ResName: string; Instance, Ancestor: TComponent);
    procedure WriteResourceHeader(const ResName: string; {!!!:out} var FixupInfo: Longint);
    procedure FixupResourceHeader(FixupInfo: Longint);
    procedure ReadResHeader;
    function ReadByte : Byte;
    function ReadWord : Word;
    function ReadDWord : Cardinal;
    function ReadQWord : QWord;
    function ReadAnsiString : String;
    procedure WriteByte(b : Byte);
    procedure WriteWord(w : Word);
    procedure WriteDWord(d : Cardinal);
    procedure WriteQWord(q : QWord);
    Procedure WriteAnsiString (const S : String);
    property Position: Int64 read GetPosition write SetPosition;
    property Size: Int64 read GetSize write SetSize64;
  end;

  TProxyStream = class(TStream)
  private
    FStream: IStream;
  protected
    function GetIStream: IStream;
  public
    constructor Create(const Stream: IStream);
    function Read(var Buffer; Count: Longint): Longint; override;
    function Write(const Buffer; Count: Longint): Longint; override;
    function Seek(const Offset: int64; Origin: TSeekOrigin): int64; override;
    procedure Check(err:integer); virtual;
  end;

  { TOwnerStream }
  TOwnerStream = Class(TStream)
  Protected
    FOwner : Boolean;
    FSource : TStream;
  Public
    Constructor Create(ASource : TStream);
    Destructor Destroy; override;
    Property Source : TStream Read FSource;
    Property SourceOwner : Boolean Read Fowner Write FOwner;
  end;


  IStreamPersist = interface ['{B8CD12A3-267A-11D4-83DA-00C04F60B2DD}']
    procedure LoadFromStream(Stream: TStream);
    procedure SaveToStream(Stream: TStream);
  end;

{ THandleStream class }

  THandleStream = class(TStream)
  private
    FHandle: THandle;
  protected
    procedure SetSize(NewSize: Longint); override;
    procedure SetSize(const NewSize: Int64); override;
  public
    constructor Create(AHandle: THandle);
    function Read(var Buffer; Count: Longint): Longint; override;
    function Write(const Buffer; Count: Longint): Longint; override;
    function Seek(const Offset: Int64; Origin: TSeekOrigin): Int64; override;
    property Handle: THandle read FHandle;
  end;

{ TFileStream class }

  TFileStream = class(THandleStream)
  Private
    FFileName : String;
  public
    constructor Create(const AFileName: string; Mode: Word);
    constructor Create(const AFileName: string; Mode: Word; Rights: Cardinal);
    destructor Destroy; override;
    property FileName : String Read FFilename;
  end;

{ TCustomMemoryStream abstract class }

  TCustomMemoryStream = class(TStream)
  private
    FMemory: Pointer;
    FSize, FPosition: PtrInt;
    FSizeBoundsSeek : Boolean;
  protected
    Function GetSize : Int64; Override;
    function GetPosition: Int64; Override;
    procedure SetPointer(Ptr: Pointer; ASize: PtrInt);
  public
    function Read(var Buffer; Count: LongInt): LongInt; override;
    function Seek(const Offset: Int64; Origin: TSeekOrigin): Int64; override;
    procedure SaveToStream(Stream: TStream);
    procedure SaveToFile(const FileName: string);
    property Memory: Pointer read FMemory;
    Property SizeBoundsSeek : Boolean Read FSizeBoundsSeek Write FSizeBoundsSeek;
  end;

{ TMemoryStream }

  TMemoryStream = class(TCustomMemoryStream)
  private
    FCapacity: PtrInt;
    procedure SetCapacity(NewCapacity: PtrInt);
  protected
    function Realloc(var NewCapacity: PtrInt): Pointer; virtual;
    property Capacity: PtrInt read FCapacity write SetCapacity;
  public
    destructor Destroy; override;
    procedure Clear;
    procedure LoadFromStream(Stream: TStream);
    procedure LoadFromFile(const FileName: string);
    procedure SetSize({$ifdef CPU64}const NewSize: Int64{$else}NewSize: LongInt{$endif}); override;
    function Write(const Buffer; Count: LongInt): LongInt; override;
  end;

{ TBytesStream }

  TBytesStream = class(TMemoryStream)
  private
    FBytes: TBytes;
  protected
    function Realloc(var NewCapacity: PtrInt): Pointer; override;
  public
    constructor Create(const ABytes: TBytes); virtual; overload;
    property Bytes: TBytes read FBytes;
  end;

{ TStringStream }

  TStringStream = class(TBytesStream)
  private
    FEncoding: TEncoding;
    FOwnsEncoding : Boolean;
    function GetDataString: string;
    function GetUnicodeDataString: UnicodeString;
  protected
  public
    constructor Create(const ABytes: TBytes); override; overload;
    constructor Create(const AString: string = ''); overload;
    constructor CreateRaw(const AString: RawByteString); overload;
    constructor Create(const AString: string; AEncoding: TEncoding; AOwnsEncoding: Boolean = True); overload;
    constructor Create(const AString: string; ACodePage: Integer); overload;
    // UnicodeString versions
    constructor Create(const AString: UnicodeString); overload;
    constructor Create(const AString: UnicodeString; AEncoding: TEncoding; AOwnsEncoding: Boolean = True); overload;
    constructor Create(const AString: UnicodeString; ACodePage: Integer); overload;
    Destructor Destroy; override;
    function ReadUnicodeString(Count: Longint): UnicodeString;
    procedure WriteUnicodeString(const AString: UnicodeString);
    function ReadAnsiString(Count: Longint): AnsiString;
    procedure WriteAnsiString(const AString: AnsiString);
    function ReadString(Count: Longint): string;
    procedure WriteString(const AString: string);
    property DataString: string read GetDataString;
    Property UnicodeDataString : UnicodeString Read GetUnicodeDataString;
    Property OwnsEncoding : Boolean Read FOwnsEncoding;
    Property Encoding : TEncoding Read FEncoding;
  end;

{ TRawByteStringStream }

  TRawByteStringStream = Class(TBytesStream)
  public
    Constructor Create (const aData : RawByteString); overload;
    function DataString: RawByteString;

    function ReadString(Count: Longint): RawByteString;
    procedure WriteString(const AString: RawByteString);
  end;

{ TResourceStream }

{$ifdef FPC_OS_UNICODE}
  TResourceStream = class(TCustomMemoryStream)
  private
    Res: TFPResourceHandle;
    Handle: TFPResourceHGLOBAL;
    procedure Initialize(Instance: TFPResourceHMODULE; Name, ResType: PWideChar; NameIsID: Boolean);
  public
    constructor Create(Instance: TFPResourceHMODULE; const ResName: WideString; ResType: PWideChar);
    constructor CreateFromID(Instance: TFPResourceHMODULE; ResID: Integer; ResType: PWideChar);
    destructor Destroy; override;
  end;
{$else}
  TResourceStream = class(TCustomMemoryStream)
  private
    Res: TFPResourceHandle;
    Handle: TFPResourceHGLOBAL;
    procedure Initialize(Instance: TFPResourceHMODULE; Name, ResType: PChar; NameIsID: Boolean);
  public
    constructor Create(Instance: TFPResourceHMODULE; const ResName: string; ResType: PChar);
    constructor CreateFromID(Instance: TFPResourceHMODULE; ResID: Integer; ResType: PChar);
    destructor Destroy; override;
  end;
{$endif FPC_OS_UNICODE}

{ TStreamAdapter }

  TStreamOwnership = (soReference, soOwned);

{ Implements OLE IStream on TStream }
  TStreamAdapter = class(TInterfacedObject, IStream)
  private
    FStream    : TStream;
    FOwnership : TStreamOwnership;
    m_bReverted: Boolean;
  public
    constructor Create(Stream: TStream; Ownership: TStreamOwnership = soReference);
    destructor Destroy; override;
    function Read(pv: Pointer; cb: DWORD; pcbRead: PDWORD): HResult; virtual; stdcall;
    function Write(pv: Pointer; cb: DWORD; pcbWritten: PDWORD): HResult; virtual; stdcall;
    function Seek(dlibMove: LargeInt; dwOrigin: DWORD; out libNewPosition: LargeUint): HResult; virtual; stdcall;
    function SetSize(libNewSize: LargeUint): HResult; virtual; stdcall;
    function CopyTo(stm: IStream; cb: LargeUint; out cbRead: LargeUint; out cbWritten: LargeUint): HResult; virtual; stdcall;
    function Commit(grfCommitFlags: DWORD): HResult; virtual; stdcall;
    function Revert: HResult; virtual; stdcall;
    function LockRegion(libOffset: LargeUint; cb: LargeUint; dwLockType: DWORD): HResult; virtual; stdcall;
    function UnlockRegion(libOffset: LargeUint; cb: LargeUint; dwLockType: DWORD): HResult; virtual; stdcall;
    function Stat(out statstg: TStatStg; grfStatFlag: DWORD): HResult; virtual; stdcall;
    function Clone(out stm: IStream): HResult; virtual; stdcall;
    property Stream: TStream read FStream;
    property StreamOwnership: TStreamOwnership read FOwnership write FOwnership;
  end;

{ TFiler }

  TValueType = (vaNull, vaList, vaInt8, vaInt16, vaInt32, vaExtended,
    vaString, vaIdent, vaFalse, vaTrue, vaBinary, vaSet, vaLString,
    vaNil, vaCollection, vaSingle, vaCurrency, vaDate, vaWString, vaInt64,
    vaUTF8String, vaUString, vaQWord);

  TFilerFlag = (ffInherited, ffChildPos, ffInline);
  TFilerFlags = set of TFilerFlag;

  TReaderProc = procedure(Reader: TReader) of object;
  TWriterProc = procedure(Writer: TWriter) of object;
  TStreamProc = procedure(Stream: TStream) of object;

  TFiler = class(TObject)
  private
    FRoot: TComponent;
    FLookupRoot: TComponent;
    FAncestor: TPersistent;
    FIgnoreChildren: Boolean;
  protected
    procedure SetRoot(ARoot: TComponent); virtual;
  public
    procedure DefineProperty(const Name: string;
      ReadData: TReaderProc; WriteData: TWriterProc;
      HasData: Boolean); virtual; abstract;
    procedure DefineBinaryProperty(const Name: string;
      ReadData, WriteData: TStreamProc;
      HasData: Boolean); virtual; abstract;
    Procedure FlushBuffer; virtual; abstract;
    property Root: TComponent read FRoot write SetRoot;
    property LookupRoot: TComponent read FLookupRoot;
    property Ancestor: TPersistent read FAncestor write FAncestor;
    property IgnoreChildren: Boolean read FIgnoreChildren write FIgnoreChildren;
  end;


{ TComponent class reference type }

  TComponentClass = class of TComponent;


{ TReader }

  { TAbstractObjectReader }

  TAbstractObjectReader = class
  public
    Procedure FlushBuffer; virtual;
    function NextValue: TValueType; virtual; abstract;
    function ReadValue: TValueType; virtual; abstract;
    procedure BeginRootComponent; virtual; abstract;
    procedure BeginComponent(var Flags: TFilerFlags; var AChildPos: Integer;
      var CompClassName, CompName: String); virtual; abstract;
    function BeginProperty: String; virtual; abstract;

    //Please don't use read, better use ReadBinary whenever possible
    procedure Read(var Buf; Count: LongInt); virtual; abstract;
    { All ReadXXX methods are called _after_ the value type has been read! }
    procedure ReadBinary(const DestData: TMemoryStream); virtual; abstract;
{$ifndef FPUNONE}
    function ReadFloat: Extended; virtual; abstract;
    function ReadSingle: Single; virtual; abstract;
    function ReadDate: TDateTime; virtual; abstract;
{$endif}
    function ReadCurrency: Currency; virtual; abstract;
    function ReadIdent(ValueType: TValueType): String; virtual; abstract;
    function ReadInt8: ShortInt; virtual; abstract;
    function ReadInt16: SmallInt; virtual; abstract;
    function ReadInt32: LongInt; virtual; abstract;
    function ReadInt64: Int64; virtual; abstract;
    function ReadSet(EnumType: Pointer): Integer; virtual; abstract;
    procedure ReadSignature; virtual; abstract;
    function ReadStr: String; virtual; abstract;
    function ReadString(StringType: TValueType): String; virtual; abstract;
    function ReadWideString: WideString;virtual;abstract;
    function ReadUnicodeString: UnicodeString;virtual;abstract;
    procedure SkipComponent(SkipComponentInfos: Boolean); virtual; abstract;
    procedure SkipValue; virtual; abstract;
  end;

  { TBinaryObjectReader }

  TBinaryObjectReader = class(TAbstractObjectReader)
  protected
    FStream: TStream;
    FBuffer: Pointer;
    FBufSize: Integer;
    FBufPos: Integer;
    FBufEnd: Integer;

    function ReadWord : word; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
    function ReadDWord : longword; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
    function ReadQWord : qword; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
{$ifndef FPUNONE}
    function ReadExtended : extended; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
{$endif}
    procedure SkipProperty;
    procedure SkipSetBody;
  public
    constructor Create(Stream: TStream; BufSize: Integer);
    destructor Destroy; override;
    function NextValue: TValueType; override;
    function ReadValue: TValueType; override;
    procedure BeginRootComponent; override;
    procedure BeginComponent(var Flags: TFilerFlags; var AChildPos: Integer;
      var CompClassName, CompName: String); override;
    function BeginProperty: String; override;

    //Please don't use read, better use ReadBinary whenever possible
    procedure Read(var Buf; Count: LongInt); override;
    procedure ReadBinary(const DestData: TMemoryStream); override;
{$ifndef FPUNONE}
    function ReadFloat: Extended; override;
    function ReadSingle: Single; override;
    function ReadDate: TDateTime; override;
{$endif}
    function ReadCurrency: Currency; override;
    function ReadIdent(ValueType: TValueType): String; override;
    function ReadInt8: ShortInt; override;
    function ReadInt16: SmallInt; override;
    function ReadInt32: LongInt; override;
    function ReadInt64: Int64; override;
    function ReadSet(EnumType: Pointer): Integer; override;
    procedure ReadSignature; override;
    function ReadStr: String; override;
    function ReadString(StringType: TValueType): String; override;
    function ReadWideString: WideString;override;
    function ReadUnicodeString: UnicodeString;override;
    procedure SkipComponent(SkipComponentInfos: Boolean); override;
    procedure SkipValue; override;
  end;


  TFindMethodEvent = procedure(Reader: TReader; const MethodName: string;
    var Address: CodePointer; var Error: Boolean) of object;
  TSetMethodPropertyEvent = procedure(Reader: TReader; Instance: TPersistent;
    PropInfo: PPropInfo; const TheMethodName: string;
    var Handled: boolean) of object;
  TSetNameEvent = procedure(Reader: TReader; Component: TComponent;
    var Name: string) of object;
  TReferenceNameEvent = procedure(Reader: TReader; var Name: string) of object;
  TAncestorNotFoundEvent = procedure(Reader: TReader; const ComponentName: string;
    ComponentClass: TPersistentClass; var Component: TComponent) of object;
  TReadComponentsProc = procedure(Component: TComponent) of object;
  TReaderError = procedure(Reader: TReader; const Message: string;
    var Handled: Boolean) of object;
  TPropertyNotFoundEvent = procedure(Reader: TReader; Instance: TPersistent;
    var PropName: string; IsPath: boolean; var Handled, Skip: Boolean) of object;
  TFindComponentClassEvent = procedure(Reader: TReader; const ClassName: string;
    var ComponentClass: TComponentClass) of object;
  TCreateComponentEvent = procedure(Reader: TReader;
    ComponentClass: TComponentClass; var Component: TComponent) of object;

  TReadWriteStringPropertyEvent = procedure(Sender:TObject;
    const Instance: TPersistent; PropInfo: PPropInfo;
    var Content:string) of object;


  { TReader }

  TReader = class(TFiler)
  private
    FDriver: TAbstractObjectReader;
    FOwner: TComponent;
    FParent: TComponent;
    FFixups: TObject;
    FLoaded: TFpList;
    FLock: TRTLCriticalSection;
    FOnFindMethod: TFindMethodEvent;
    FOnSetMethodProperty: TSetMethodPropertyEvent;
    FOnSetName: TSetNameEvent;
    FOnReferenceName: TReferenceNameEvent;
    FOnAncestorNotFound: TAncestorNotFoundEvent;
    FOnError: TReaderError;
    FOnPropertyNotFound: TPropertyNotFoundEvent;
    FOnFindComponentClass: TFindComponentClassEvent;
    FOnCreateComponent: TCreateComponentEvent;
    FPropName: string;
    FCanHandleExcepts: Boolean;
    FOnReadStringProperty:TReadWriteStringPropertyEvent;
    procedure DoFixupReferences;
    function FindComponentClass(const AClassName: string): TComponentClass;
    procedure Lock;
    procedure Unlock;
  protected
    function Error(const Message: string): Boolean; virtual;
    function FindMethod(ARoot: TComponent; const AMethodName: string): CodePointer; virtual;
    procedure ReadProperty(AInstance: TPersistent);
    procedure ReadPropValue(Instance: TPersistent; PropInfo: Pointer);
    procedure PropertyError;
    procedure ReadData(Instance: TComponent);
    property PropName: string read FPropName;
    property CanHandleExceptions: Boolean read FCanHandleExcepts;
    function CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectReader; virtual;
  public
    constructor Create(Stream: TStream; BufSize: Integer);
    destructor Destroy; override;
    Procedure FlushBuffer; override;
    procedure BeginReferences;
    procedure CheckValue(Value: TValueType);
    procedure DefineProperty(const Name: string;
      AReadData: TReaderProc; WriteData: TWriterProc;
      HasData: Boolean); override;
    procedure DefineBinaryProperty(const Name: string;
      AReadData, WriteData: TStreamProc;
      HasData: Boolean); override;
    function EndOfList: Boolean;
    procedure EndReferences;
    procedure FixupReferences;
    function NextValue: TValueType;
    //Please don't use read, better use ReadBinary whenever possible
    //uuups, ReadBinary is protected ..
    procedure Read(var Buf; Count: LongInt); virtual;

    function ReadBoolean: Boolean;
    function ReadChar: Char;
    function ReadWideChar: WideChar;
    function ReadUnicodeChar: UnicodeChar;
    procedure ReadCollection(Collection: TCollection);
    function ReadComponent(Component: TComponent): TComponent;
    procedure ReadComponents(AOwner, AParent: TComponent;
      Proc: TReadComponentsProc);
{$ifndef FPUNONE}
    function ReadFloat: Extended;
    function ReadSingle: Single;
    function ReadDate: TDateTime;
{$endif}
    function ReadCurrency: Currency;
    function ReadIdent: string;
    function ReadInteger: Longint;
    function ReadInt64: Int64;
    function ReadSet(EnumType: Pointer): Integer;
    procedure ReadListBegin;
    procedure ReadListEnd;
    function ReadRootComponent(ARoot: TComponent): TComponent;
    function ReadVariant: Variant;
    procedure ReadSignature;
    function ReadString: string;
    function ReadWideString: WideString;
    function ReadUnicodeString: UnicodeString;
    function ReadValue: TValueType;
    procedure CopyValue(Writer: TWriter);
    property Driver: TAbstractObjectReader read FDriver;
    property Owner: TComponent read FOwner write FOwner;
    property Parent: TComponent read FParent write FParent;
    property OnError: TReaderError read FOnError write FOnError;
    property OnPropertyNotFound: TPropertyNotFoundEvent read FOnPropertyNotFound write FOnPropertyNotFound;
    property OnFindMethod: TFindMethodEvent read FOnFindMethod write FOnFindMethod;
    property OnSetMethodProperty: TSetMethodPropertyEvent read FOnSetMethodProperty write FOnSetMethodProperty;
    property OnSetName: TSetNameEvent read FOnSetName write FOnSetName;
    property OnReferenceName: TReferenceNameEvent read FOnReferenceName write FOnReferenceName;
    property OnAncestorNotFound: TAncestorNotFoundEvent read FOnAncestorNotFound write FOnAncestorNotFound;
    property OnCreateComponent: TCreateComponentEvent read FOnCreateComponent write FOnCreateComponent;
    property OnFindComponentClass: TFindComponentClassEvent read FOnFindComponentClass write FOnFindComponentClass;
    property OnReadStringProperty: TReadWriteStringPropertyEvent read FOnReadStringProperty write FOnReadStringProperty;
  end;


{ TWriter }

  { TAbstractObjectWriter }

  TAbstractObjectWriter = class
  public
    { Begin/End markers. Those ones who don't have an end indicator, use
      "EndList", after the occurrence named in the comment. Note that this
      only counts for "EndList" calls on the same level; each BeginXXX call
      increases the current level. }
    procedure BeginCollection; virtual; abstract;  { Ends with the next "EndList" }
    procedure BeginComponent(Component: TComponent; Flags: TFilerFlags;
      ChildPos: Integer); virtual; abstract;  { Ends after the second "EndList" }
    procedure WriteSignature; virtual; abstract;
    procedure BeginList; virtual; abstract;
    procedure EndList; virtual; abstract;
    procedure BeginProperty(const PropName: String); virtual; abstract;
    procedure EndProperty; virtual; abstract;
    Procedure FlushBuffer; virtual;
    //Please don't use write, better use WriteBinary whenever possible
    procedure Write(const Buffer; Count: Longint); virtual;abstract;

    procedure WriteBinary(const Buffer; Count: Longint); virtual; abstract;
    procedure WriteBoolean(Value: Boolean); virtual; abstract;
    // procedure WriteChar(Value: Char);
{$ifndef FPUNONE}
    procedure WriteFloat(const Value: Extended); virtual; abstract;
    procedure WriteSingle(const Value: Single); virtual; abstract;
    procedure WriteDate(const Value: TDateTime); virtual; abstract;
{$endif}
    procedure WriteCurrency(const Value: Currency); virtual; abstract;
    procedure WriteIdent(const Ident: string); virtual; abstract;
    procedure WriteInteger(Value: Int64); virtual; abstract;
    procedure WriteUInt64(Value: QWord); virtual; abstract;
    procedure WriteVariant(const Value: Variant); virtual; abstract;
    procedure WriteMethodName(const Name: String); virtual; abstract;
    procedure WriteSet(Value: LongInt; SetType: Pointer); virtual; abstract;
    procedure WriteString(const Value: String); virtual; abstract;
    procedure WriteWideString(const Value: WideString);virtual;abstract;
    procedure WriteUnicodeString(const Value: UnicodeString);virtual;abstract;
  end;

  { TBinaryObjectWriter }

  TBinaryObjectWriter = class(TAbstractObjectWriter)
  protected
    FStream: TStream;
    FBuffer: Pointer;
    FBufSize: Integer;
    FBufPos: Integer;
    FBufEnd: Integer;
    procedure WriteWord(w : word); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
    procedure WriteDWord(lw : longword); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
    procedure WriteQWord(qw : qword); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
{$ifndef FPUNONE}
    procedure WriteExtended(e : extended); {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
{$endif}
    procedure WriteValue(Value: TValueType);
  public
    constructor Create(Stream: TStream; BufSize: Integer);
    destructor Destroy; override;
    procedure WriteSignature; override;
    procedure FlushBuffer; override;
    procedure BeginCollection; override;
    procedure BeginComponent(Component: TComponent; Flags: TFilerFlags;
      ChildPos: Integer); override;
    procedure BeginList; override;
    procedure EndList; override;
    procedure BeginProperty(const PropName: String); override;
    procedure EndProperty; override;

    //Please don't use write, better use WriteBinary whenever possible
    procedure Write(const Buffer; Count: Longint); override;
    procedure WriteBinary(const Buffer; Count: LongInt); override;
    procedure WriteBoolean(Value: Boolean); override;
{$ifndef FPUNONE}
    procedure WriteFloat(const Value: Extended); override;
    procedure WriteSingle(const Value: Single); override;
    procedure WriteDate(const Value: TDateTime); override;
{$endif}
    procedure WriteCurrency(const Value: Currency); override;
    procedure WriteIdent(const Ident: string); override;
    procedure WriteInteger(Value: Int64); override;
    procedure WriteUInt64(Value: QWord); override;
    procedure WriteMethodName(const Name: String); override;
    procedure WriteSet(Value: LongInt; SetType: Pointer); override;
    procedure WriteStr(const Value: String);
    procedure WriteString(const Value: String); override;
    procedure WriteWideString(const Value: WideString); override;
    procedure WriteUnicodeString(const Value: UnicodeString); override;
    procedure WriteVariant(const VarValue: Variant);override;
  end;

  TTextObjectWriter = class(TAbstractObjectWriter)
  end;


  TFindAncestorEvent = procedure (Writer: TWriter; Component: TComponent;
    const Name: string; var Ancestor, RootAncestor: TComponent) of object;
  TWriteMethodPropertyEvent = procedure (Writer: TWriter; Instance: TPersistent;
    PropInfo: PPropInfo;
    const MethodValue, DefMethodValue: TMethod;
    var Handled: boolean) of object;

  TWriter = class(TFiler)
  private
    FDriver: TAbstractObjectWriter;
    FDestroyDriver: Boolean;
    FRootAncestor: TComponent;
    FPropPath: String;
    FAncestors: TStringList;
    FAncestorPos: Integer;
    FCurrentPos: Integer;
    FOnFindAncestor: TFindAncestorEvent;
    FOnWriteMethodProperty: TWriteMethodPropertyEvent;
    FOnWriteStringProperty:TReadWriteStringPropertyEvent;
    procedure AddToAncestorList(Component: TComponent);
    procedure WriteComponentData(Instance: TComponent);
    Procedure DetermineAncestor(Component: TComponent);
    procedure DoFindAncestor(Component : TComponent);
  protected
    procedure SetRoot(ARoot: TComponent); override;
    procedure WriteBinary(AWriteData: TStreamProc);
    procedure WriteProperty(Instance: TPersistent; PropInfo: Pointer);
    procedure WriteProperties(Instance: TPersistent);
    procedure WriteChildren(Component: TComponent);
    function CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectWriter; virtual;
  public
    constructor Create(ADriver: TAbstractObjectWriter);
    constructor Create(Stream: TStream; BufSize: Integer);
    destructor Destroy; override;
    Procedure FlushBuffer; override;
    procedure DefineProperty(const Name: string;
      ReadData: TReaderProc; AWriteData: TWriterProc;
      HasData: Boolean); override;
    procedure DefineBinaryProperty(const Name: string;
      ReadData, AWriteData: TStreamProc;
      HasData: Boolean); override;
    //Please don't use write, better use WriteBinary whenever possible
    //uuups, WriteBinary is protected ..
    procedure Write(const Buffer; Count: Longint); virtual;
    procedure WriteBoolean(Value: Boolean);
    procedure WriteCollection(Value: TCollection);
    procedure WriteComponent(Component: TComponent);
    procedure WriteChar(Value: Char);
    procedure WriteWideChar(Value: WideChar);
    procedure WriteDescendent(ARoot: TComponent; AAncestor: TComponent);
{$ifndef FPUNONE}
    procedure WriteFloat(const Value: Extended);
    procedure WriteSingle(const Value: Single);
    procedure WriteDate(const Value: TDateTime);
{$endif}
    procedure WriteCurrency(const Value: Currency);
    procedure WriteIdent(const Ident: string);
    procedure WriteInteger(Value: Longint); overload;
    procedure WriteInteger(Value: Int64); overload;
    procedure WriteSet(Value: LongInt; SetType: Pointer);
    procedure WriteListBegin;
    procedure WriteListEnd;
    Procedure WriteSignature;
    procedure WriteRootComponent(ARoot: TComponent);
    procedure WriteString(const Value: string);
    procedure WriteWideString(const Value: WideString);
    procedure WriteUnicodeString(const Value: UnicodeString);
    procedure WriteVariant(const VarValue: Variant);
    property RootAncestor: TComponent read FRootAncestor write FRootAncestor;
    property OnFindAncestor: TFindAncestorEvent read FOnFindAncestor write FOnFindAncestor;
    property OnWriteMethodProperty: TWriteMethodPropertyEvent read FOnWriteMethodProperty write FOnWriteMethodProperty;
    property OnWriteStringProperty: TReadWriteStringPropertyEvent read FOnWriteStringProperty write FOnWriteStringProperty;

    property Driver: TAbstractObjectWriter read FDriver;
    property PropertyPath: string read FPropPath;
  end;


{ TParser }

  TParser = class(TObject)
  private
    fStream : TStream;
    fBuf : pchar;
    fBufLen : integer;
    fPos : integer;
    fDeltaPos : integer;
    fFloatType : char;
    fSourceLine : integer;
    fToken : char;
    fEofReached : boolean;
    fLastTokenStr : string;
    fLastTokenWStr : widestring;
    function GetTokenName(aTok : char) : string;
    procedure LoadBuffer;
    procedure CheckLoadBuffer; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    procedure ProcessChar; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function IsNumber : boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function IsHexNum : boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function IsAlpha : boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function IsAlphaNum : boolean; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function GetHexValue(c : char) : byte; {$ifdef CLASSESINLINE} inline; {$endif CLASSESINLINE}
    function GetAlphaNum : string;
    procedure HandleNewLine;
    procedure SkipBOM;
    procedure SkipSpaces;
    procedure SkipWhitespace;
    procedure HandleEof;
    procedure HandleAlphaNum;
    procedure HandleNumber;
    procedure HandleHexNumber;
    function HandleQuotedString : string;
    procedure HandleDecimalCharacter(var ascii : boolean;
                                     out WideChr: widechar; out StringChr: char);
    procedure HandleString;
    procedure HandleMinus;
    procedure HandleUnknown;
  public
    constructor Create(Stream: TStream);
    destructor Destroy; override;
    procedure CheckToken(T: Char);
    procedure CheckTokenSymbol(const S: string);
    procedure Error(const Ident: string);
    procedure ErrorFmt(const Ident: string; const Args: array of const);
    procedure ErrorStr(const Message: string);
    procedure HexToBinary(Stream: TStream);
    function NextToken: Char;
    function SourcePos: Longint;
    function TokenComponentIdent: string;
{$ifndef FPUNONE}
    function TokenFloat: Extended;
{$endif}
    function TokenInt: Int64;
    function TokenString: string;
    function TokenWideString: WideString;
    function TokenSymbolIs(const S: string): Boolean;
    property FloatType: Char read fFloatType;
    property SourceLine: Integer read fSourceLine;
    property Token: Char read fToken;
  end;

{ TThread }
  TThread = Class;

  EThread = class(Exception);
  EThreadExternalException = class(EThread);
  EThreadDestroyCalled = class(EThread);
  TSynchronizeProcVar = procedure;
  TThreadMethod = procedure of object;

  TThreadReportStatus = Procedure(Const status : String) of Object;

  TThreadStatusNotifyEvent = Procedure(Sender : TThread; Const status : String) of Object;
  TThreadExecuteHandler = TThreadMethod;
  TThreadExecuteStatusHandler = Procedure(ReportStatus : TThreadReportStatus) of object;

  TNotifyCallBack = Procedure(Sender : TObject; AData : Pointer);
  TThreadStatusNotifyCallBack = Procedure(Sender : TThread; AData : Pointer; Const status : String);
  TThreadExecuteCallBack = Procedure(AData : Pointer);
  TThreadExecuteStatusCallBack = Procedure(AData : Pointer; ReportStatus : TThreadReportStatus);

  TThreadPriority = (tpIdle, tpLowest, tpLower, tpNormal, tpHigher, tpHighest,
    tpTimeCritical);



  TThread = class
  private type
    PThreadQueueEntry = ^TThreadQueueEntry;
    TThreadQueueEntry = record
      Method: TThreadMethod;
      // uncomment once closures are supported
      //ThreadProc: TThreadProcedure;
      Thread: TThread;
      ThreadID: TThreadID;
      Exception: TObject;
      SyncEvent: PRtlEvent;
      Next: PThreadQueueEntry;
    end;
  public type
    TSystemTimes = record
      IdleTime: QWord;
      UserTime: QWord;
      KernelTime: QWord;
      NiceTime: QWord;
    end;
  private
    class var FProcessorCount: LongWord;
  private
    FHandle: TThreadID;
    FTerminated: Boolean;
    FFreeOnTerminate: Boolean;
    FFinished: Boolean;
    FSuspended: LongBool;
    FReturnValue: Integer;
    FOnTerminate: TNotifyEvent;
    FFatalException: TObject;
    FExternalThread: Boolean;
    FSynchronizeEntry: PThreadQueueEntry;
    class function GetCurrentThread: TThread; static;
    class function GetIsSingleProcessor: Boolean; static; inline;
    class procedure InternalQueue(aThread: TThread; aMethod: TThreadMethod; aQueueIfMain: Boolean); static;
    procedure CallOnTerminate;
    function GetPriority: TThreadPriority;
    procedure SetPriority(Value: TThreadPriority);
    procedure SetSuspended(Value: Boolean);
    function GetSuspended: Boolean;
    procedure InitSynchronizeEvent;
    procedure DoneSynchronizeEvent;
    { these two need to be implemented per platform }
    procedure SysCreate(CreateSuspended: Boolean;
                             const StackSize: SizeUInt);
    procedure SysDestroy;
  protected
    FThreadID: TThreadID; // someone might need it for pthread_* calls
    procedure DoTerminate; virtual;
    procedure Execute; virtual; abstract;
    procedure Synchronize(AMethod: TThreadMethod);
    procedure Queue(aMethod: TThreadMethod);
    procedure ForceQueue(aMethod: TThreadMethod); inline;
    property ReturnValue: Integer read FReturnValue write FReturnValue;
    property Terminated: Boolean read FTerminated;
{$if defined(windows) or defined(OS2)}
  private
    FInitialSuspended: boolean;
{$endif}
{$ifdef Unix}
  private
    // see tthread.inc, ThreadFunc and TThread.Resume
    FSuspendEvent: PRTLEvent;
    FInitialSuspended: boolean;
    FSuspendedInternal: longbool;
    FThreadReaped: boolean;
{$endif}
{$ifdef netwlibc}
  private
    // see tthread.inc, ThreadFunc and TThread.Resume
    FSem: Pointer;
    FInitialSuspended: boolean;
    FSuspendedExternal: boolean;
    FPid: LongInt;
{$endif}
{$if defined(hasamiga)}
  private
    FInitialSuspended: boolean;
{$endif}
{$ifdef beos}
    FSem : pointer;
    FSuspendedExternal: boolean;
{$endif}
  public
    constructor Create(CreateSuspended: Boolean;
                       const StackSize: SizeUInt = DefaultStackSize);
    destructor Destroy; override;
    { Note: Once closures are supported aProc will be changed to TProc }
    class function CreateAnonymousThread(aProc: TProcedure): TThread; static;
    { Use HAS_TTHREAD_NAMETHREADFORDEBUGGING to implement a platform specific
      variant of the UnicodeString method. The AnsiString method calls the
      UnicodeString method. If your platform's API only supports AnsiString you
      can additionally define THREADNAME_IS_ANSISTRING to swap the logic. Then
      the UnicodeString variant will call the AnsiString variant which can be
      implemented for a specific platform }
    class procedure NameThreadForDebugging(aThreadName: UnicodeString; aThreadID: TThreadID = TThreadID(-1)); static;
    class procedure NameThreadForDebugging(aThreadName: AnsiString; aThreadID: TThreadID = TThreadID(-1)); static; inline;
    class procedure SetReturnValue(aValue: Integer); static;
    class function CheckTerminated: Boolean; static;
    class procedure Synchronize(AThread: TThread; AMethod: TThreadMethod);
    class procedure Queue(aThread: TThread; aMethod: TThreadMethod); static;
    class procedure ForceQueue(aThread: TThread; aMethod: TThreadMethod); inline; static;
    class procedure RemoveQueuedEvents(aThread: TThread; aMethod: TThreadMethod); static;
    class procedure RemoveQueuedEvents(aMethod: TThreadMethod); static;
    class procedure RemoveQueuedEvents(aThread: TThread); static;
    class procedure SpinWait(aIterations: LongWord); static;
    class procedure Sleep(aMilliseconds: Cardinal); static;
    class procedure Yield; static;
    { use HAS_TTHREAD_GETSYSTEMTIMES to implement a platform specific variant
      which does not return a zeroed record }
    class procedure GetSystemTimes(out aSystemTimes: TSystemTimes); static;
    class function GetTickCount: LongWord; static; deprecated 'Use TThread.GetTickCount64 instead';
    class function GetTickCount64: QWord; static;
    // Object based
    Class Function ExecuteInThread(AMethod : TThreadExecuteHandler; AOnTerminate : TNotifyEvent = Nil) : TThread; overload; static;
    Class Function ExecuteInThread(AMethod : TThreadExecuteStatusHandler; AOnStatus : TThreadStatusNotifyEvent; AOnTerminate : TNotifyEvent = Nil) : TThread; overload;static;
    // Plain methods.
    Class Function ExecuteInThread(AMethod : TThreadExecuteCallback; AData : Pointer = Nil; AOnTerminate: TNotifyCallBack = Nil) : TThread; overload;static;
    Class Function ExecuteInThread(AMethod : TThreadExecuteStatusCallback; AOnStatus : TThreadStatusNotifyCallback; AData : Pointer = Nil; AOnTerminate : TNotifyCallBack = Nil) : TThread; overload;static;
    procedure AfterConstruction; override;
    procedure Start;
    procedure Resume; deprecated;
    procedure Suspend; deprecated;
    procedure Terminate;
    function WaitFor: Integer;
    class property CurrentThread: TThread read GetCurrentThread;
    class property ProcessorCount: LongWord read FProcessorCount;
    class property IsSingleProcessor: Boolean read GetIsSingleProcessor;
    property FreeOnTerminate: Boolean read FFreeOnTerminate write FFreeOnTerminate;
    property Handle: TThreadID read FHandle;
    property ExternalThread: Boolean read FExternalThread;
    property Priority: TThreadPriority read GetPriority write SetPriority;
    property Suspended: Boolean read GetSuspended write SetSuspended;
    property Finished: Boolean read FFinished;
    property ThreadID: TThreadID read FThreadID;
    property OnTerminate: TNotifyEvent read FOnTerminate write FOnTerminate;
    property FatalException: TObject read FFatalException;
  end;


{ TComponent class }

  TOperation = (opInsert, opRemove);
  TComponentState = set of (csLoading, csReading, csWriting, csDestroying,
    csDesigning, csAncestor, csUpdating, csFixups, csFreeNotification,
    csInline, csDesignInstance);
  TComponentStyle = set of (csInheritable, csCheckPropAvail, csSubComponent,
    csTransient);
  TGetChildProc = procedure (Child: TComponent) of object;

  IVCLComObject = interface
    ['{E07892A0-F52F-11CF-BD2F-0020AF0E5B81}']
    function GetTypeInfoCount(out Count: Integer): HResult; stdcall;
    function GetTypeInfo(Index, LocaleID: Integer; out TypeInfo): HResult; stdcall;
    function GetIDsOfNames(const IID: TGUID; Names: Pointer;
      NameCount, LocaleID: Integer; DispIDs: Pointer): HResult; stdcall;
    function Invoke(DispID: Integer; const IID: TGUID; LocaleID: Integer;
      Flags: Word; var Params; VarResult, ExcepInfo, ArgErr: Pointer): HResult; stdcall;
    function SafeCallException(ExceptObject: TObject; ExceptAddr: CodePointer): HResult;
    procedure FreeOnRelease;
  end;

  IInterfaceComponentReference = interface
    ['{3FEEC8E1-E400-4A24-BCAC-1F01476439B1}']
    function GetComponent:TComponent;
   end;

  IDesignerNotify = interface
    ['{B971E807-E3A6-11D1-AAB1-00C04FB16FBC}']
    procedure Modified;
    procedure Notification(AnObject: TPersistent; Operation: TOperation);
  end;

  TComponentEnumerator = class
  private
    FComponent: TComponent;
    FPosition: Integer;
  public
    constructor Create(AComponent: TComponent);
    function GetCurrent: TComponent;
    function MoveNext: Boolean;
    property Current: TComponent read GetCurrent;
  end;

  TBasicAction = class;

  { TComponent }

  TComponent = class(TPersistent,IUnknown,IInterfaceComponentReference)
  private
    FOwner: TComponent;
    FName: TComponentName;
    FTag: Ptrint;
    FComponents: TFpList;
    FFreeNotifies: TFpList;
    FDesignInfo: Longint;
    FVCLComObject: Pointer;
    FComponentState: TComponentState;
    function GetComObject: IUnknown;
    function GetComponent(AIndex: Integer): TComponent;
    function GetComponentCount: Integer;
    function GetComponentIndex: Integer;
    procedure Insert(AComponent: TComponent);
    procedure ReadLeft(Reader: TReader);
    procedure ReadTop(Reader: TReader);
    procedure Remove(AComponent: TComponent);
    procedure RemoveNotification(AComponent: TComponent);
    procedure SetComponentIndex(Value: Integer);
    procedure SetReference(Enable: Boolean);
    procedure WriteLeft(Writer: TWriter);
    procedure WriteTop(Writer: TWriter);
  protected
    FComponentStyle: TComponentStyle;
    procedure ChangeName(const NewName: TComponentName);
    procedure DefineProperties(Filer: TFiler); override;
    procedure GetChildren(Proc: TGetChildProc; Root: TComponent); dynamic;
    function GetChildOwner: TComponent; dynamic;
    function GetChildParent: TComponent; dynamic;
    function GetOwner: TPersistent; override;
    procedure Loaded; virtual;
    procedure Loading; virtual;
    procedure Notification(AComponent: TComponent;
      Operation: TOperation); virtual;
    procedure PaletteCreated; dynamic;
    procedure ReadState(Reader: TReader); virtual;
    procedure SetAncestor(Value: Boolean);
    procedure SetDesigning(Value: Boolean; SetChildren : Boolean = True);
    procedure SetDesignInstance(Value: Boolean);
    procedure SetInline(Value: Boolean);
    procedure SetName(const NewName: TComponentName); virtual;
    procedure SetChildOrder(Child: TComponent; Order: Integer); dynamic;
    procedure SetParentComponent(Value: TComponent); dynamic;
    procedure Updating; dynamic;
    procedure Updated; dynamic;
    class procedure UpdateRegistry(Register: Boolean; const ClassID, ProgID: string); dynamic;
    procedure ValidateRename(AComponent: TComponent;
      const CurName, NewName: string); virtual;
    procedure ValidateContainer(AComponent: TComponent); dynamic;
    procedure ValidateInsert(AComponent: TComponent); dynamic;
    { IUnknown }
    function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} IID: TGUID; out Obj): Hresult; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    function _AddRef: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    function _Release: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    function iicrGetComponent: TComponent;
    { IDispatch }
    function GetTypeInfoCount(out Count: Integer): HResult; stdcall;
    function GetTypeInfo(Index, LocaleID: Integer; out TypeInfo): HResult; stdcall;
    function GetIDsOfNames(const IID: TGUID; Names: Pointer;
      NameCount, LocaleID: Integer; DispIDs: Pointer): HResult; stdcall;
    function Invoke(DispID: Integer; const IID: TGUID; LocaleID: Integer;
      Flags: Word; var Params; VarResult, ExcepInfo, ArgErr: Pointer): HResult; stdcall;
  public
    //!! Moved temporary
    // fpdoc doesn't handle this yet :(
{$ifndef fpdocsystem}
    function IInterfaceComponentReference.GetComponent=iicrgetcomponent;
{$endif}
    procedure WriteState(Writer: TWriter); virtual;
    constructor Create(AOwner: TComponent); virtual;
    destructor Destroy; override;
    procedure BeforeDestruction; override;
    procedure DestroyComponents;
    procedure Destroying;
    function ExecuteAction(Action: TBasicAction): Boolean; dynamic;
    function FindComponent(const AName: string): TComponent;
    procedure FreeNotification(AComponent: TComponent);
    procedure RemoveFreeNotification(AComponent: TComponent);
    procedure FreeOnRelease;
    function GetEnumerator: TComponentEnumerator;
    function GetNamePath: string; override;
    function GetParentComponent: TComponent; dynamic;
    function HasParent: Boolean; dynamic;
    procedure InsertComponent(AComponent: TComponent);
    procedure RemoveComponent(AComponent: TComponent);
    function SafeCallException(ExceptObject: TObject;
      ExceptAddr: CodePointer): HResult; override;
    procedure SetSubComponent(ASubComponent: Boolean);
    function UpdateAction(Action: TBasicAction): Boolean; dynamic;
    property ComObject: IUnknown read GetComObject;
    function IsImplementorOf (const Intf:IInterface):boolean;
    procedure ReferenceInterface(const intf:IInterface;op:TOperation);
    property Components[Index: Integer]: TComponent read GetComponent;
    property ComponentCount: Integer read GetComponentCount;
    property ComponentIndex: Integer read GetComponentIndex write SetComponentIndex;
    property ComponentState: TComponentState read FComponentState;
    property ComponentStyle: TComponentStyle read FComponentStyle;
    property DesignInfo: Longint read FDesignInfo write FDesignInfo;
    property Owner: TComponent read FOwner;
    property VCLComObject: Pointer read FVCLComObject write FVCLComObject;
  published
    property Name: TComponentName read FName write SetName stored False;
    property Tag: PtrInt read FTag write FTag default 0;
  end;

{ TBasicActionLink }

  TBasicActionLink = class(TObject)
  private
    FOnChange: TNotifyEvent;
  protected
    FAction: TBasicAction;
    procedure AssignClient(AClient: TObject); virtual;
    procedure Change; virtual;
    function IsOnExecuteLinked: Boolean; virtual;
    procedure SetAction(Value: TBasicAction); virtual;
    procedure SetOnExecute(Value: TNotifyEvent); virtual;
  public
    constructor Create(AClient: TObject); virtual;
    destructor Destroy; override;
    function Execute(AComponent: TComponent = nil): Boolean; virtual;
    function Update: Boolean; virtual;
    property Action: TBasicAction read FAction write SetAction;
    property OnChange: TNotifyEvent read FOnChange write FOnChange;
  end;

  TBasicActionLinkClass = class of TBasicActionLink;

{ TBasicAction }

  TBasicAction = class(TComponent)
  private
    FActionComponent: TComponent;
    FOnChange: TNotifyEvent;
    FOnExecute: TNotifyEvent;
    FOnUpdate: TNotifyEvent;
  protected
    FClients: TFpList;
    procedure Change; virtual;
    procedure SetOnExecute(Value: TNotifyEvent); virtual;
    property OnChange: TNotifyEvent read FOnChange write FOnChange;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    function HandlesTarget(Target: TObject): Boolean; virtual;
    procedure UpdateTarget(Target: TObject); virtual;
    procedure ExecuteTarget(Target: TObject); virtual;
    function Execute: Boolean; dynamic;
    procedure RegisterChanges(Value: TBasicActionLink);
    procedure UnRegisterChanges(Value: TBasicActionLink);
    function Update: Boolean; virtual;
    property ActionComponent: TComponent read FActionComponent write FActionComponent;
    property OnExecute: TNotifyEvent read FOnExecute write SetOnExecute;
    property OnUpdate: TNotifyEvent read FOnUpdate write FOnUpdate;
  end;

{ TBasicAction class reference type }

  TBasicActionClass = class of TBasicAction;

{ Component registration handlers }

  TActiveXRegType = (axrComponentOnly, axrIncludeDescendants);

  IInterfaceList = interface ['{285DEA8A-B865-11D1-AAA7-00C04FB17A72}']
    function Get(i : Integer) : IUnknown;
    function GetCapacity : Integer;
    function GetCount : Integer;
    procedure Put(i : Integer;item : IUnknown);
    procedure SetCapacity(NewCapacity : Integer);
    procedure SetCount(NewCount : Integer);
    procedure Clear;
    procedure Delete(index : Integer);
    procedure Exchange(index1,index2 : Integer);
    function First : IUnknown;
    function IndexOf(const item : IUnknown) : Integer;
    function Add(item : IUnknown) : Integer;
    procedure Insert(i : Integer;item : IUnknown);
    function Last : IUnknown;
    function Remove(item : IUnknown): Integer;
    procedure Lock;
    procedure Unlock;
    property Capacity : Integer read GetCapacity write SetCapacity;
    property Count : Integer read GetCount write SetCount;
    property Items[index : Integer] : IUnknown read Get write Put;default;
  end;

  TInterfaceList = class;

  TInterfaceListEnumerator = class
  private
    FList: TInterfaceList;
    FPosition: Integer;
  public
    constructor Create(AList: TInterfaceList);
    function GetCurrent: IUnknown;
    function MoveNext: Boolean;
    property Current: IUnknown read GetCurrent;
  end;

  TInterfaceList = class(TInterfacedObject,IInterfaceList)
  private
    FList : TThreadList;
  protected
    function Get(i : Integer) : IUnknown;
    function GetCapacity : Integer;
    function GetCount : Integer;
    procedure Put(i : Integer;item : IUnknown);
    procedure SetCapacity(NewCapacity : Integer);
    procedure SetCount(NewCount : Integer);
  public
    constructor Create;
    destructor Destroy; override;

    procedure Clear;
    procedure Delete(index : Integer);
    procedure Exchange(index1,index2 : Integer);
    function First : IUnknown;
    function GetEnumerator: TInterfaceListEnumerator;
    function IndexOf(const item : IUnknown) : Integer;
    function Add(item : IUnknown) : Integer;
    procedure Insert(i : Integer;item : IUnknown);
    function Last : IUnknown;
    function Remove(item : IUnknown): Integer;
    procedure Lock;
    procedure Unlock;

    function Expand : TInterfaceList;

    property Capacity : Integer read GetCapacity write SetCapacity;
    property Count : Integer read GetCount write SetCount;
    property Items[Index : Integer] : IUnknown read Get write Put;default;
  end;

{ ---------------------------------------------------------------------
    TDatamodule support
  ---------------------------------------------------------------------}
  TDataModule = class(TComponent)
  private
    FDPos: TPoint;
    FDSize: TPoint;
    FDPPI: Integer;
    FOnCreate: TNotifyEvent;
    FOnDestroy: TNotifyEvent;
    FOldOrder : Boolean;
    Procedure ReadP(Reader: TReader);
    Procedure WriteP(Writer: TWriter);
    Procedure ReadT(Reader: TReader);
    Procedure WriteT(Writer: TWriter);
    Procedure ReadL(Reader: TReader);
    Procedure WriteL(Writer: TWriter);
    Procedure ReadW(Reader: TReader);
    Procedure WriteW(Writer: TWriter);
    Procedure ReadH(Reader: TReader);
    Procedure WriteH(Writer: TWriter);
  protected
    Procedure DoCreate; virtual;
    Procedure DoDestroy; virtual;
    Procedure DefineProperties(Filer: TFiler); override;
    Procedure GetChildren(Proc: TGetChildProc; Root: TComponent); override;
    Function HandleCreateException: Boolean; virtual;
    Procedure ReadState(Reader: TReader); override;
  public
    constructor Create(AOwner: TComponent); override;
    Constructor CreateNew(AOwner: TComponent);
    Constructor CreateNew(AOwner: TComponent; CreateMode: Integer); virtual;
    destructor Destroy; override;
    Procedure AfterConstruction; override;
    Procedure BeforeDestruction; override;
    property DesignOffset: TPoint read FDPos write FDPos;
    property DesignSize: TPoint read FDSize write FDSize;
    property DesignPPI: Integer read FDPPI write FDPPI;
  published
    property OnCreate: TNotifyEvent read FOnCreate write FOnCreate;
    property OnDestroy: TNotifyEvent read FOnDestroy write FOnDestroy;
    property OldCreateOrder: Boolean read FOldOrder write FOldOrder;
  end;
  TDataModuleClass = Class of TDataModule;

var
  // IDE hooks for TDatamodule support.
  AddDataModule              : procedure (DataModule: TDataModule) of object;
  RemoveDataModule           : procedure (DataModule: TDataModule) of object;
  ApplicationHandleException : procedure (Sender: TObject) of object;
  ApplicationShowException   : procedure (E: Exception) of object;

{ ---------------------------------------------------------------------
    tthread helpers
  ---------------------------------------------------------------------}

{ function to be called when gui thread is ready to execute method
  result is true if a method has been executed
}
function CheckSynchronize(timeout : longint=0) : boolean;

var
  { method proc that is called to trigger gui thread to execute a
method }
  WakeMainThread : TNotifyEvent = nil;

{ ---------------------------------------------------------------------
    General streaming and registration routines
  ---------------------------------------------------------------------}


var
  RegisterComponentsProc: procedure(const Page: string;
    ComponentClasses: array of TComponentClass);
  RegisterNoIconProc: procedure(ComponentClasses: array of TComponentClass);
{!!!!  RegisterNonActiveXProc: procedure(ComponentClasses: array of TComponentClass;
    AxRegType: TActiveXRegType) = nil;
  CurrentGroup: Integer = -1;}
  CreateVCLComObjectProc: procedure(Component: TComponent) = nil;

{ Point and rectangle constructors }

function Point(AX, AY: Integer): TPoint;
function SmallPoint(AX, AY: SmallInt): TSmallPoint;
function Rect(ALeft, ATop, ARight, ABottom: Integer): TRect;
function Bounds(ALeft, ATop, AWidth, AHeight: Integer): TRect;

function PointsEqual(const P1, P2: TPoint): Boolean; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
function PointsEqual(const P1, P2: TSmallPoint): Boolean; {$ifdef CLASSESINLINE}inline;{$endif CLASSESINLINE}
function InvalidPoint(X, Y: Integer): Boolean;
function InvalidPoint(const At: TPoint): Boolean;
function InvalidPoint(const At: TSmallPoint): Boolean;

{ Class registration routines }

procedure RegisterClass(AClass: TPersistentClass);
procedure RegisterClasses(AClasses: array of TPersistentClass);
procedure RegisterClassAlias(AClass: TPersistentClass; const Alias: string);
procedure UnRegisterClass(AClass: TPersistentClass);
procedure UnRegisterClasses(AClasses: array of TPersistentClass);
procedure UnRegisterModuleClasses(Module: HMODULE);
function FindClass(const AClassName: string): TPersistentClass;
function GetClass(const AClassName: string): TPersistentClass;
procedure StartClassGroup(AClass: TPersistentClass);
procedure GroupDescendentsWith(AClass, AClassGroup: TPersistentClass);
function ActivateClassGroup(AClass: TPersistentClass): TPersistentClass;
function ClassGroupOf(AClass: TPersistentClass): TPersistentClass;
function ClassGroupOf(Instance: TPersistent): TPersistentClass;

{ Component registration routines }

procedure RegisterComponents(const Page: string;
  ComponentClasses: array of TComponentClass);
procedure RegisterNoIcon(ComponentClasses: array of TComponentClass);
procedure RegisterNonActiveX(ComponentClasses: array of TComponentClass;
  AxRegType: TActiveXRegType);

var
  GlobalNameSpace: IReadWriteSync;

{ Object filing routines }

type
  TIdentMapEntry = record
    Value: Integer;
    Name: String;
  end;

  TIdentToInt = function(const Ident: string; var Int: Longint): Boolean;
  TIntToIdent = function(Int: Longint; var Ident: string): Boolean;
  TFindGlobalComponent = function(const Name: string): TComponent;
  TInitComponentHandler = function(Instance: TComponent; RootAncestor : TClass): boolean;

var
  MainThreadID: TThreadID;

procedure RegisterIntegerConsts(IntegerType: Pointer; IdentToIntFn: TIdentToInt;
  IntToIdentFn: TIntToIdent);
function IdentToInt(const Ident: string; out Int: Longint; const Map: array of TIdentMapEntry): Boolean;
function IntToIdent(Int: Longint; var Ident: string; const Map: array of TIdentMapEntry): Boolean;
function FindIntToIdent(AIntegerType: Pointer): TIntToIdent;
function FindIdentToInt(AIntegerType: Pointer): TIdentToInt;

procedure RegisterFindGlobalComponentProc(AFindGlobalComponent: TFindGlobalComponent);
procedure UnregisterFindGlobalComponentProc(AFindGlobalComponent: TFindGlobalComponent);
function FindGlobalComponent(const Name: string): TComponent;

function InitInheritedComponent(Instance: TComponent; RootAncestor: TClass): Boolean;
function InitComponentRes(const ResName: string; Instance: TComponent): Boolean;
function ReadComponentRes(const ResName: string; Instance: TComponent): TComponent;
function ReadComponentResEx(HInstance: THandle; const ResName: string): TComponent;
function ReadComponentResFile(const FileName: string; Instance: TComponent): TComponent;
procedure WriteComponentResFile(const FileName: string; Instance: TComponent);
procedure RegisterInitComponentHandler(ComponentClass: TComponentClass;   Handler: TInitComponentHandler);

procedure GlobalFixupReferences;
procedure GetFixupReferenceNames(Root: TComponent; Names: TStrings);
procedure GetFixupInstanceNames(Root: TComponent;
  const ReferenceRootName: string; Names: TStrings);
procedure RedirectFixupReferences(Root: TComponent; const OldRootName,
  NewRootName: string);
procedure RemoveFixupReferences(Root: TComponent; const RootName: string);
procedure RemoveFixups(Instance: TPersistent);
Function FindNestedComponent(Root : TComponent; APath : String; CStyle : Boolean = True) : TComponent;

procedure BeginGlobalLoading;
procedure NotifyGlobalLoading;
procedure EndGlobalLoading;

function CollectionsEqual(C1, C2: TCollection): Boolean;
function CollectionsEqual(C1, C2: TCollection; Owner1, Owner2: TComponent): Boolean;

{ Object conversion routines }

type
  TObjectTextEncoding = (
    oteDFM,
    oteLFM
    );

procedure ObjectBinaryToText(Input, Output: TStream; Encoding: TObjectTextEncoding);
procedure ObjectBinaryToText(Input, Output: TStream);
procedure ObjectTextToBinary(Input, Output: TStream);

procedure ObjectResourceToText(Input, Output: TStream);
procedure ObjectTextToResource(Input, Output: TStream);

{ Utility routines }

function LineStart(Buffer, BufPos: PChar): PChar;
procedure BinToHex(BinValue, HexValue: PChar; BinBufSize: Integer);
function HexToBin(HexValue, BinValue: PChar; BinBufSize: Integer): Integer;
function ExtractStrings(Separators, WhiteSpace: TSysCharSet; Content: PChar; Strings: TStrings; AddEmptyStrings : Boolean = False): Integer;

