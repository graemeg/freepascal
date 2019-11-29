{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Micha Nelissen
    member of the Free Pascal development team

    It contains the Free Pascal generics library

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}

{$define FGLINLINE}

{$ifdef FGLINLINE}
{$inline on}
{$endif FGLINLINE}

unit fgl;

interface

uses
  types, sysutils, sortbase;

{$IF defined(VER2_4)}
  {$DEFINE OldSyntax}
{$IFEND}

const
  MaxListSize = Maxint div 16;

type
  EListError = class(Exception);

  TFPSList = class;
  TFPSListCompareFunc = function(Key1, Key2: Pointer): Integer of object;

  { TFPSList }

  TFPSList = class(TObject)
  protected
    FList: PByte;
    FCount: Integer;
    FCapacity: Integer; { list has room for capacity+1 items, contains room for a temporary item }
    FItemSize: Integer;
    procedure CopyItem(Src, Dest: Pointer); virtual;
    procedure CopyItems(Src, Dest: Pointer; aCount : Integer); virtual;
    procedure Deref(Item: Pointer); virtual; overload;
    procedure Deref(FromIndex, ToIndex: Integer); overload;
    function Get(Index: Integer): Pointer;
    procedure InternalExchange(Index1, Index2: Integer);
    function  InternalGet(Index: Integer): Pointer; {$ifdef FGLINLINE} inline; {$endif}
    procedure InternalPut(Index: Integer; NewItem: Pointer);
    procedure Put(Index: Integer; Item: Pointer);
    procedure QuickSort(L, R: Integer; Compare: TFPSListCompareFunc);
    procedure SetCapacity(NewCapacity: Integer);
    procedure SetCount(NewCount: Integer);
    procedure RaiseIndexError(Index : Integer);
    property InternalItems[Index: Integer]: Pointer read InternalGet write InternalPut;
    function GetLast: Pointer;
    procedure SetLast(const Value: Pointer);
    function GetFirst: Pointer;
    procedure SetFirst(const Value: Pointer);
    Procedure CheckIndex(AIndex : Integer); inline;
  public
    constructor Create(AItemSize: Integer = sizeof(Pointer));
    destructor Destroy; override;
    class Function ItemIsManaged : Boolean; virtual;
    function Add(Item: Pointer): Integer;
    procedure Clear;
    procedure Delete(Index: Integer);
    procedure DeleteRange(IndexFrom, IndexTo : Integer);
    class procedure Error(const Msg: string; Data: PtrInt);
    procedure Exchange(Index1, Index2: Integer);
    function Expand: TFPSList;
    procedure Extract(Item: Pointer; ResultPtr: Pointer);
    function IndexOf(Item: Pointer): Integer;
    procedure Insert(Index: Integer; Item: Pointer);
    function Insert(Index: Integer): Pointer;
    procedure Move(CurIndex, NewIndex: Integer);
    procedure Assign(Obj: TFPSList);
    procedure AddList(Obj: TFPSList);
    function Remove(Item: Pointer): Integer;
    procedure Pack;
    procedure Sort(Compare: TFPSListCompareFunc);
    procedure Sort(Compare: TFPSListCompareFunc; SortingAlgorithm: PSortingAlgorithm);
    property Capacity: Integer read FCapacity write SetCapacity;
    property Count: Integer read FCount write SetCount;
    property Items[Index: Integer]: Pointer read Get write Put; default;
    property ItemSize: Integer read FItemSize;
    property List: PByte read FList;
    property First: Pointer read GetFirst write SetFirst;
    property Last: Pointer read GetLast write SetLast;
  end;

const
{$ifdef cpu16}
  MaxGListSize = {MaxInt div} 1024 deprecated;
{$else cpu16}
  MaxGListSize = MaxInt div 1024 deprecated;
{$endif cpu16}

type
  generic TFPGListEnumerator<T> = class(TObject)
  protected
    FList: TFPSList;
    FPosition: Integer;
    function GetCurrent: T;
  public
    constructor Create(AList: TFPSList);
    function MoveNext: Boolean;
    property Current: T read GetCurrent;
  end;

  { TFPGList }

  generic TFPGList<T> = class(TFPSList)
  private
    type
      TCompareFunc = function(const Item1, Item2: T): Integer;
      PT = ^T;
      TTypeList = PT;
      PTypeList = ^TTypeList;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnCompare: TCompareFunc;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    function  Get(Index: Integer): T; {$ifdef FGLINLINE} inline; {$endif}
    function  GetList: PTypeList; {$ifdef FGLINLINE} inline; {$endif}
    function  ItemPtrCompare(Item1, Item2: Pointer): Integer;
    procedure Put(Index: Integer; const Item: T); {$ifdef FGLINLINE} inline; {$endif}
    function GetLast: T; {$ifdef FGLINLINE} inline; {$endif}
    procedure SetLast(const Value: T); {$ifdef FGLINLINE} inline; {$endif}
    function GetFirst: T; {$ifdef FGLINLINE} inline; {$endif}
    procedure SetFirst(const Value: T); {$ifdef FGLINLINE} inline; {$endif}
  public
    Type
      TFPGListEnumeratorSpec = specialize TFPGListEnumerator<T>;
    constructor Create;

    class Function ItemIsManaged : Boolean; override;
    function Add(const Item: T): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Extract(const Item: T): T; {$ifdef FGLINLINE} inline; {$endif}
    property First: T read GetFirst write SetFirst;
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef FGLINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef FGLINLINE} inline; {$endif}
    property Last: T read GetLast write SetLast;
{$ifndef VER2_4}
    procedure Assign(Source: TFPGList);
    procedure AddList(Source: TFPGList);
{$endif VER2_4}
    function Remove(const Item: T): Integer; {$ifdef FGLINLINE} inline; {$endif}
    procedure Sort(Compare: TCompareFunc);
    procedure Sort(Compare: TCompareFunc; SortingAlgorithm: PSortingAlgorithm);
    property Items[Index: Integer]: T read Get write Put; default;
    property List: PTypeList read GetList;
  end;

  generic TFPGObjectList<T: TObject> = class(TFPSList)
  private
    type
      TCompareFunc = function(const Item1, Item2: T): Integer;
      PT = ^T;
      TTypeList = PT;
      PTypeList = ^TTypeList;
      TFPGListEnumeratorSpec = specialize TFPGListEnumerator<T>;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnCompare: TCompareFunc;
      FFreeObjects: Boolean;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    function  Get(Index: Integer): T; {$ifdef FGLINLINE} inline; {$endif}
    function  GetList: PTypeList; {$ifdef FGLINLINE} inline; {$endif}
    function  ItemPtrCompare(Item1, Item2: Pointer): Integer;
    procedure Put(Index: Integer; const Item: T); {$ifdef FGLINLINE} inline; {$endif}
    function GetLast: T; {$ifdef FGLINLINE} inline; {$endif}
    procedure SetLast(const Value: T); {$ifdef FGLINLINE} inline; {$endif}
    function GetFirst: T; {$ifdef FGLINLINE} inline; {$endif}
    procedure SetFirst(const Value: T); {$ifdef FGLINLINE} inline; {$endif}
  public
    constructor Create(FreeObjects: Boolean = True);
    function Add(const Item: T): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Extract(const Item: T): T; {$ifdef FGLINLINE} inline; {$endif}
    property First: T read GetFirst write SetFirst;
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef FGLINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef FGLINLINE} inline; {$endif}
    property Last: T read GetLast write SetLast;
{$ifndef VER2_4}
    procedure AddList(Source: TFPGObjectList);
    procedure Assign(Source: TFPGObjectList);
{$endif VER2_4}
    function Remove(const Item: T): Integer; {$ifdef FGLINLINE} inline; {$endif}
    procedure Sort(Compare: TCompareFunc);
    procedure Sort(Compare: TCompareFunc; SortingAlgorithm: PSortingAlgorithm);
    property Items[Index: Integer]: T read Get write Put; default;
    property List: PTypeList read GetList;
    property FreeObjects: Boolean read FFreeObjects write FFreeObjects;
  end;

  generic TFPGInterfacedObjectList<T> = class(TFPSList)
  private
    type
      TCompareFunc = function(const Item1, Item2: T): Integer;
      PT = ^T;
      TTypeList = PT;
      PTypeList = ^TTypeList;
      TFPGListEnumeratorSpec = specialize TFPGListEnumerator<T>;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnCompare: TCompareFunc;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    function  Get(Index: Integer): T; {$ifdef FGLINLINE} inline; {$endif}
    function  GetList: PTypeList; {$ifdef FGLINLINE} inline; {$endif}
    function  ItemPtrCompare(Item1, Item2: Pointer): Integer;
    procedure Put(Index: Integer; const Item: T); {$ifdef FGLINLINE} inline; {$endif}
    function GetLast: T; {$ifdef FGLINLINE} inline; {$endif}
    procedure SetLast(const Value: T); {$ifdef FGLINLINE} inline; {$endif}
    function GetFirst: T; {$ifdef FGLINLINE} inline; {$endif}
    procedure SetFirst(const Value: T); {$ifdef FGLINLINE} inline; {$endif}
  public
    constructor Create;
    function Add(const Item: T): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Extract(const Item: T): T; {$ifdef FGLINLINE} inline; {$endif}
    property First: T read GetFirst write SetFirst;
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef FGLINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef FGLINLINE} inline; {$endif}
    property Last: T read GetLast write SetLast;
{$ifndef VER2_4}
    procedure Assign(Source: TFPGInterfacedObjectList);
    procedure AddList(Source: TFPGInterfacedObjectList);
{$endif VER2_4}
    function Remove(const Item: T): Integer; {$ifdef FGLINLINE} inline; {$endif}
    procedure Sort(Compare: TCompareFunc);
    procedure Sort(Compare: TCompareFunc; SortingAlgorithm: PSortingAlgorithm);
    property Items[Index: Integer]: T read Get write Put; default;
    property List: PTypeList read GetList;
  end;

  TFPSMap = class(TFPSList)
  private
    FKeySize: Integer;
    FDataSize: Integer;
    FDuplicates: TDuplicates;
    FSorted: Boolean;
    FOnKeyPtrCompare: TFPSListCompareFunc;
    FOnDataPtrCompare: TFPSListCompareFunc;
    procedure SetSorted(Value: Boolean);
  protected
    function BinaryCompareKey(Key1, Key2: Pointer): Integer;
    function BinaryCompareData(Data1, Data2: Pointer): Integer;
    procedure SetOnKeyPtrCompare(Proc: TFPSListCompareFunc);
    procedure SetOnDataPtrCompare(Proc: TFPSListCompareFunc);
    procedure InitOnPtrCompare; virtual;
    procedure CopyKey(Src, Dest: Pointer); virtual;
    procedure CopyData(Src, Dest: Pointer); virtual;
    function GetKey(Index: Integer): Pointer;
    function GetKeyData(AKey: Pointer): Pointer;
    function GetData(Index: Integer): Pointer;
    function LinearIndexOf(AKey: Pointer): Integer;
    procedure PutKey(Index: Integer; AKey: Pointer);
    procedure PutKeyData(AKey: Pointer; NewData: Pointer);
    procedure PutData(Index: Integer; AData: Pointer);
  public
    constructor Create(AKeySize: Integer = sizeof(Pointer);
      ADataSize: integer = sizeof(Pointer));
    function Add(AKey, AData: Pointer): Integer;
    function Add(AKey: Pointer): Integer;
    function Find(AKey: Pointer; out Index: Integer): Boolean;
    function IndexOf(AKey: Pointer): Integer;
    function IndexOfData(AData: Pointer): Integer;
    function Insert(Index: Integer): Pointer;
    procedure Insert(Index: Integer; out AKey, AData: Pointer);
    procedure InsertKey(Index: Integer; AKey: Pointer);
    procedure InsertKeyData(Index: Integer; AKey, AData: Pointer);
    function Remove(AKey: Pointer): Integer;
    procedure Sort;
    procedure Sort(SortingAlgorithm: PSortingAlgorithm);
    property Duplicates: TDuplicates read FDuplicates write FDuplicates;
    property KeySize: Integer read FKeySize;
    property DataSize: Integer read FDataSize;
    property Keys[Index: Integer]: Pointer read GetKey write PutKey;
    property Data[Index: Integer]: Pointer read GetData write PutData;
    property KeyData[Key: Pointer]: Pointer read GetKeyData write PutKeyData; default;
    property Sorted: Boolean read FSorted write SetSorted;
    property OnPtrCompare: TFPSListCompareFunc read FOnKeyPtrCompare write SetOnKeyPtrCompare; //deprecated;
    property OnKeyPtrCompare: TFPSListCompareFunc read FOnKeyPtrCompare write SetOnKeyPtrCompare;
    property OnDataPtrCompare: TFPSListCompareFunc read FOnDataPtrCompare write SetOnDataPtrCompare;
  end;

  generic TFPGMap<TKey, TData> = class(TFPSMap)
  private
    type
      TKeyCompareFunc = function(const Key1, Key2: TKey): Integer;
      TDataCompareFunc = function(const Data1, Data2: TData): Integer;
      PKey = ^TKey;
// unsed      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure CopyKey(Src, Dest: Pointer); override;
    procedure CopyData(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    procedure InitOnPtrCompare; override;
    function GetKey(Index: Integer): TKey; {$ifdef FGLINLINE} inline; {$endif}
    function GetKeyData(const AKey: TKey): TData; {$ifdef FGLINLINE} inline; {$endif}
    function GetData(Index: Integer): TData; {$ifdef FGLINLINE} inline; {$endif}
    function KeyCompare(Key1, Key2: Pointer): Integer;
    function KeyCustomCompare(Key1, Key2: Pointer): Integer;
    //function DataCompare(Data1, Data2: Pointer): Integer;
    function DataCustomCompare(Data1, Data2: Pointer): Integer;
    procedure PutKey(Index: Integer; const NewKey: TKey); {$ifdef FGLINLINE} inline; {$endif}
    procedure PutKeyData(const AKey: TKey; const NewData: TData); {$ifdef FGLINLINE} inline; {$endif}
    procedure PutData(Index: Integer; const NewData: TData); {$ifdef FGLINLINE} inline; {$endif}
    procedure SetOnKeyCompare(NewCompare: TKeyCompareFunc);
    procedure SetOnDataCompare(NewCompare: TDataCompareFunc);
  public
    constructor Create;
    function Add(const AKey: TKey; const AData: TData): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Add(const AKey: TKey): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Find(const AKey: TKey; out Index: Integer): Boolean; {$ifdef FGLINLINE} inline; {$endif}
    function TryGetData(const AKey: TKey; out AData: TData): Boolean; {$ifdef FGLINLINE} inline; {$endif}
    procedure AddOrSetData(const AKey: TKey; const AData: TData); {$ifdef FGLINLINE} inline; {$endif}
    function IndexOf(const AKey: TKey): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function IndexOfData(const AData: TData): Integer;
    procedure InsertKey(Index: Integer; const AKey: TKey);
    procedure InsertKeyData(Index: Integer; const AKey: TKey; const AData: TData);
    function Remove(const AKey: TKey): Integer;
    property Keys[Index: Integer]: TKey read GetKey write PutKey;
    property Data[Index: Integer]: TData read GetData write PutData;
    property KeyData[const AKey: TKey]: TData read GetKeyData write PutKeyData; default;
    property OnCompare: TKeyCompareFunc read FOnKeyCompare write SetOnKeyCompare; //deprecated;
    property OnKeyCompare: TKeyCompareFunc read FOnKeyCompare write SetOnKeyCompare;
    property OnDataCompare: TDataCompareFunc read FOnDataCompare write SetOnDataCompare;
  end;

  generic TFPGMapObject<TKey; TData: TObject> = class(TFPSMap)
  private
    type
      TKeyCompareFunc = function(const Key1, Key2: TKey): Integer;
      TDataCompareFunc = function(const Data1, Data2: TData): Integer;
      PKey = ^TKey;
// unsed      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
      FFreeObjects: Boolean;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure CopyKey(Src, Dest: Pointer); override;
    procedure CopyData(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    procedure InitOnPtrCompare; override;
    function GetKey(Index: Integer): TKey; {$ifdef FGLINLINE} inline; {$endif}
    function GetKeyData(const AKey: TKey): TData; {$ifdef FGLINLINE} inline; {$endif}
    function GetData(Index: Integer): TData; {$ifdef FGLINLINE} inline; {$endif}
    function KeyCompare(Key1, Key2: Pointer): Integer;
    function KeyCustomCompare(Key1, Key2: Pointer): Integer;
    //function DataCompare(Data1, Data2: Pointer): Integer;
    function DataCustomCompare(Data1, Data2: Pointer): Integer;
    procedure PutKey(Index: Integer; const NewKey: TKey); {$ifdef FGLINLINE} inline; {$endif}
    procedure PutKeyData(const AKey: TKey; const NewData: TData); {$ifdef FGLINLINE} inline; {$endif}
    procedure PutData(Index: Integer; const NewData: TData); {$ifdef FGLINLINE} inline; {$endif}
    procedure SetOnKeyCompare(NewCompare: TKeyCompareFunc);
    procedure SetOnDataCompare(NewCompare: TDataCompareFunc);
  public
    constructor Create(AFreeObjects: Boolean);
    constructor Create;
    function Add(const AKey: TKey; const AData: TData): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Add(const AKey: TKey): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Find(const AKey: TKey; out Index: Integer): Boolean; {$ifdef FGLINLINE} inline; {$endif}
    function TryGetData(const AKey: TKey; out AData: TData): Boolean; {$ifdef FGLINLINE} inline; {$endif}
    procedure AddOrSetData(const AKey: TKey; const AData: TData); {$ifdef FGLINLINE} inline; {$endif}
    function IndexOf(const AKey: TKey): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function IndexOfData(const AData: TData): Integer;
    procedure InsertKey(Index: Integer; const AKey: TKey);
    procedure InsertKeyData(Index: Integer; const AKey: TKey; const AData: TData);
    function Remove(const AKey: TKey): Integer;
    property Keys[Index: Integer]: TKey read GetKey write PutKey;
    property Data[Index: Integer]: TData read GetData write PutData;
    property KeyData[const AKey: TKey]: TData read GetKeyData write PutKeyData; default;
    property OnCompare: TKeyCompareFunc read FOnKeyCompare write SetOnKeyCompare; //deprecated;
    property OnKeyCompare: TKeyCompareFunc read FOnKeyCompare write SetOnKeyCompare;
    property OnDataCompare: TDataCompareFunc read FOnDataCompare write SetOnDataCompare;
  end;

  generic TFPGMapInterfacedObjectData<TKey, TData> = class(TFPSMap)
  private
    type
      TKeyCompareFunc = function(const Key1, Key2: TKey): Integer;
      TDataCompareFunc = function(const Data1, Data2: TData): Integer;
      PKey = ^TKey;
// unsed      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure CopyKey(Src, Dest: Pointer); override;
    procedure CopyData(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    procedure InitOnPtrCompare; override;
    function GetKey(Index: Integer): TKey; {$ifdef FGLINLINE} inline; {$endif}
    function GetKeyData(const AKey: TKey): TData; {$ifdef FGLINLINE} inline; {$endif}
    function GetData(Index: Integer): TData; {$ifdef FGLINLINE} inline; {$endif}
    function KeyCompare(Key1, Key2: Pointer): Integer;
    function KeyCustomCompare(Key1, Key2: Pointer): Integer;
    //function DataCompare(Data1, Data2: Pointer): Integer;
    function DataCustomCompare(Data1, Data2: Pointer): Integer;
    procedure PutKey(Index: Integer; const NewKey: TKey); {$ifdef FGLINLINE} inline; {$endif}
    procedure PutKeyData(const AKey: TKey; const NewData: TData); {$ifdef FGLINLINE} inline; {$endif}
    procedure PutData(Index: Integer; const NewData: TData); {$ifdef FGLINLINE} inline; {$endif}
    procedure SetOnKeyCompare(NewCompare: TKeyCompareFunc);
    procedure SetOnDataCompare(NewCompare: TDataCompareFunc);
  public
    constructor Create;
    function Add(const AKey: TKey; const AData: TData): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Add(const AKey: TKey): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function Find(const AKey: TKey; out Index: Integer): Boolean; {$ifdef FGLINLINE} inline; {$endif}
    function TryGetData(const AKey: TKey; out AData: TData): Boolean; {$ifdef FGLINLINE} inline; {$endif}
    procedure AddOrSetData(const AKey: TKey; const AData: TData); {$ifdef FGLINLINE} inline; {$endif}
    function IndexOf(const AKey: TKey): Integer; {$ifdef FGLINLINE} inline; {$endif}
    function IndexOfData(const AData: TData): Integer;
    procedure InsertKey(Index: Integer; const AKey: TKey);
    procedure InsertKeyData(Index: Integer; const AKey: TKey; const AData: TData);
    function Remove(const AKey: TKey): Integer;
    property Keys[Index: Integer]: TKey read GetKey write PutKey;
    property Data[Index: Integer]: TData read GetData write PutData;
    property KeyData[const AKey: TKey]: TData read GetKeyData write PutKeyData; default;
    property OnCompare: TKeyCompareFunc read FOnKeyCompare write SetOnKeyCompare; //deprecated;
    property OnKeyCompare: TKeyCompareFunc read FOnKeyCompare write SetOnKeyCompare;
    property OnDataCompare: TDataCompareFunc read FOnDataCompare write SetOnDataCompare;
  end;

implementation

uses
  rtlconsts;

{****************************************************************************
                             TFPSList
 ****************************************************************************}

constructor TFPSList.Create(AItemSize: integer);
begin
  inherited Create;
  FItemSize := AItemSize;
end;

destructor TFPSList.Destroy;
begin
  Clear;
  // Clear() does not clear the whole list; there is always a single temp entry
  // at the end which is never freed. Take care of that one here.
  FreeMem(FList);
  inherited Destroy;
end;

procedure TFPSList.CopyItem(Src, Dest: Pointer);
begin
  System.Move(Src^, Dest^, FItemSize);
end;

procedure TFPSList.CopyItems(Src, Dest: Pointer; aCount: Integer);
begin
  System.Move(Src^, Dest^, FItemSize*aCount);
end;

procedure TFPSList.RaiseIndexError(Index : Integer);
begin
  Error(SListIndexError, Index);
end;

function TFPSList.InternalGet(Index: Integer): Pointer;
begin
  Result:=FList+Index*ItemSize;
end;

procedure TFPSList.InternalPut(Index: Integer; NewItem: Pointer);
var
  ListItem: Pointer;
begin
  ListItem := InternalItems[Index];
  CopyItem(NewItem, ListItem);
end;

function TFPSList.Get(Index: Integer): Pointer;
begin
  CheckIndex(Index);
  Result := InternalItems[Index];
end;

procedure TFPSList.Put(Index: Integer; Item: Pointer);
var p : Pointer;
begin
  CheckIndex(Index);
  p:=InternalItems[Index];
  if assigned(p) then
    DeRef(p);	
  InternalItems[Index] := Item;
end;

procedure TFPSList.SetCapacity(NewCapacity: Integer);
begin
  if (NewCapacity < FCount) or (NewCapacity > MaxListSize) then
    Error(SListCapacityError, NewCapacity);
  if NewCapacity = FCapacity then
    exit;
  ReallocMem(FList, (NewCapacity+1) * FItemSize);
  FillChar(InternalItems[FCapacity]^, (NewCapacity+1-FCapacity) * FItemSize, #0);
  FCapacity := NewCapacity;
end;

procedure TFPSList.Deref(Item: Pointer);
begin
end;

procedure TFPSList.Deref(FromIndex, ToIndex: Integer);
var
  ListItem, ListItemLast: Pointer;
begin
  ListItem := InternalItems[FromIndex];
  ListItemLast := InternalItems[ToIndex];
  repeat
    Deref(ListItem);
    if ListItem = ListItemLast then
      break;
    ListItem := PByte(ListItem) + ItemSize;
  until false;
end;

procedure TFPSList.SetCount(NewCount: Integer);
begin
  if (NewCount < 0) or (NewCount > MaxListSize) then
    Error(SListCountError, NewCount);
  if NewCount > FCapacity then
    SetCapacity(NewCount);
  if NewCount > FCount then
    FillByte(InternalItems[FCount]^, (NewCount-FCount) * FItemSize, 0)
  else if NewCount < FCount then
    Deref(NewCount, FCount-1);
  FCount := NewCount;
end;

function TFPSList.Add(Item: Pointer): Integer;
begin
  if FCount = FCapacity then
    Self.Expand;
  CopyItem(Item, InternalItems[FCount]);
  Result := FCount;
  Inc(FCount);
end;

procedure TFPSList.CheckIndex(AIndex : Integer);

begin
  if (AIndex < 0) or (AIndex >= FCount) then
    Error(SListIndexError, AIndex);
end;

class function TFPSList.ItemIsManaged: Boolean;
begin
  Result:=False;
end;


procedure TFPSList.Clear;
begin
  if Assigned(FList) then
  begin
    SetCount(0);
    SetCapacity(0);
  end;
end;

procedure TFPSList.Delete(Index: Integer);
var
  ListItem: Pointer;
begin
  CheckIndex(Index);
  Dec(FCount);
  ListItem := InternalItems[Index];
  Deref(ListItem);
  System.Move(InternalItems[Index+1]^, ListItem^, (FCount - Index) * FItemSize);
  // Shrink the list if appropriate
  if (FCapacity > 256) and (FCount < FCapacity shr 2) then
  begin
    FCapacity := FCapacity shr 1;
    ReallocMem(FList, (FCapacity+1) * FItemSize);
  end;
  { Keep the ending of the list filled with zeros, don't leave garbage data
    there. Otherwise, we could accidentally have there a copy of some item
    on the list, and accidentally Deref it too soon.
    See http://bugs.freepascal.org/view.php?id=20005. }
  FillChar(InternalItems[FCount]^, (FCapacity+1-FCount) * FItemSize, #0);
end;

procedure TFPSList.DeleteRange(IndexFrom, IndexTo : Integer);
var
  ListItem: Pointer;
  I: Integer;
  OldCnt : Integer;
begin
  CheckIndex(IndexTo);
  CheckIndex(IndexFrom);
  OldCnt:=FCount;
  Dec(FCount,IndexTo-IndexFrom+1);
  For I :=IndexFrom To Indexto Do
    begin
      ListItem := InternalItems[I];
      Deref(ListItem);
    end;
  System.Move(InternalItems[IndexTo+1]^, InternalItems[IndexFrom]^, (OldCnt - IndexTo-1) * FItemSize);
  // Shrink the list if appropriate
  if (FCapacity > 256) and (FCount < FCapacity shr 2) then
  begin
    FCapacity := FCapacity shr 1;
    ReallocMem(FList, (FCapacity+1) * FItemSize);
  end;
  { Keep the ending of the list filled with zeros, don't leave garbage data
    there. Otherwise, we could accidentally have there a copy of some item
    on the list, and accidentally Deref it too soon.
    See http://bugs.freepascal.org/view.php?id=20005. }
  FillChar(InternalItems[FCount]^, (FCapacity+1-FCount) * FItemSize, #0);
end;

procedure TFPSList.Extract(Item: Pointer; ResultPtr: Pointer);
var
  i : Integer;
  ListItemPtr : Pointer;
begin
  i := IndexOf(Item);
  if i >= 0 then
  begin
    ListItemPtr := InternalItems[i];
    System.Move(ListItemPtr^, ResultPtr^, FItemSize);
    { fill with zeros, to avoid freeing/decreasing reference on following Delete }
    System.FillByte(ListItemPtr^, FItemSize, 0);
    Delete(i);
  end else
    System.FillByte(ResultPtr^, FItemSize, 0);
end;

class procedure TFPSList.Error(const Msg: string; Data: PtrInt);
begin
  raise EListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;

procedure TFPSList.Exchange(Index1, Index2: Integer);
begin
  CheckIndex(Index1);
  CheckIndex(Index2);
  InternalExchange(Index1, Index2);
end;

procedure TFPSList.InternalExchange(Index1, Index2: Integer);
begin
  System.Move(InternalItems[Index1]^, InternalItems[FCapacity]^, FItemSize);
  System.Move(InternalItems[Index2]^, InternalItems[Index1]^, FItemSize);
  System.Move(InternalItems[FCapacity]^, InternalItems[Index2]^, FItemSize);
end;

function TFPSList.Expand: TFPSList;
var
  IncSize : Longint;
begin
  if FCount < FCapacity then exit;
  IncSize := 4;
  if FCapacity > 3 then IncSize := IncSize + 4;
  if FCapacity > 8 then IncSize := IncSize + 8;
  if FCapacity > 127 then Inc(IncSize, FCapacity shr 2);
  SetCapacity(FCapacity + IncSize);
  Result := Self;
end;

function TFPSList.GetFirst: Pointer;
begin
  If FCount = 0 then
    Result := Nil
  else
    Result := InternalItems[0];
end;

procedure TFPSList.SetFirst(const Value: Pointer);
begin
  Put(0, Value);
end;

function TFPSList.IndexOf(Item: Pointer): Integer;
var
  ListItem: Pointer;
begin
  Result := 0;
  ListItem := First;
  while (Result < FCount) and (CompareByte(ListItem^, Item^, FItemSize) <> 0) do
  begin
    Inc(Result);
    ListItem := PByte(ListItem)+FItemSize;
  end;
  if Result = FCount then Result := -1;
end;

function TFPSList.Insert(Index: Integer): Pointer;
begin
  if (Index < 0) or (Index > FCount) then
    Error(SListIndexError, Index);
  if FCount = FCapacity then Self.Expand;
  Result := InternalItems[Index];
  if Index<FCount then
  begin
    System.Move(Result^, (Result+FItemSize)^, (FCount - Index) * FItemSize);
    { clear for compiler assisted types }
    System.FillByte(Result^, FItemSize, 0);
  end;
  Inc(FCount);
end;

procedure TFPSList.Insert(Index: Integer; Item: Pointer);
begin
  CopyItem(Item, Insert(Index));
end;

function TFPSList.GetLast: Pointer;
begin
  if FCount = 0 then
    Result := nil
  else
    Result := InternalItems[FCount - 1];
end;

procedure TFPSList.SetLast(const Value: Pointer);
begin
  Put(FCount - 1, Value);
end;

procedure TFPSList.Move(CurIndex, NewIndex: Integer);
var
  CurItem, NewItem, TmpItem, Src, Dest: Pointer;
  MoveCount: Integer;
begin
  CheckIndex(CurIndex);
  CheckIndex(NewIndex);
  if CurIndex = NewIndex then
    exit;
  CurItem := InternalItems[CurIndex];
  NewItem := InternalItems[NewIndex];
  TmpItem := InternalItems[FCapacity];
  System.Move(CurItem^, TmpItem^, FItemSize);
  if NewIndex > CurIndex then
  begin
    Src := InternalItems[CurIndex+1];
    Dest := CurItem;
    MoveCount := NewIndex - CurIndex;
  end else begin
    Src := NewItem;
    Dest := InternalItems[NewIndex+1];
    MoveCount := CurIndex - NewIndex;
  end;
  System.Move(Src^, Dest^, MoveCount * FItemSize);
  System.Move(TmpItem^, NewItem^, FItemSize);
end;

function TFPSList.Remove(Item: Pointer): Integer;
begin
  Result := IndexOf(Item);
  if Result <> -1 then
    Delete(Result);
end;

const LocalThreshold = 64;

procedure TFPSList.Pack;
var
  LItemSize : integer;
  NewCount,
  i : integer;
  pdest,
  psrc : Pointer;
  localnul : array[0..LocalThreshold-1] of byte;
  pnul : pointer;
begin
  LItemSize:=FItemSize;
  pnul:=@localnul;
  if LItemSize>Localthreshold then
    getmem(pnul,LItemSize);
  fillchar(pnul^,LItemSize,#0);
  NewCount:=0;
  psrc:=First;
  pdest:=psrc;

  For I:=0 To FCount-1 Do
    begin
        if not CompareMem(psrc,pnul,LItemSize) then
        begin
          System.Move(psrc^, pdest^, LItemSize);
          inc(pdest,LItemSIze);
          inc(NewCount);
        end
      else
        deref(psrc);
      inc(psrc,LitemSize);
    end;
  if LItemSize>Localthreshold then
    FreeMem(pnul,LItemSize);

  FCount:=NewCount;
end;

procedure TFPSList.Sort(Compare: TFPSListCompareFunc);
begin
  Sort(Compare, SortBase.DefaultSortingAlgorithm);
end;

type
  PFPSList_Sort_Comparer_Context = ^TFPSList_Sort_Comparer_Context;
  TFPSList_Sort_Comparer_Context = record
    Compare: TFPSListCompareFunc;
  end;

function TFPSList_Sort_Comparer(Item1, Item2, Context: Pointer): Integer;
begin
  Result := PFPSList_Sort_Comparer_Context(Context)^.Compare(Item1, Item2);
end;

procedure TFPSList.Sort(Compare: TFPSListCompareFunc; SortingAlgorithm: PSortingAlgorithm);
var
  Context: TFPSList_Sort_Comparer_Context;
begin
  Context.Compare := Compare;
  SortingAlgorithm^.ItemListSorter_ContextComparer(FList, FCount, FItemSize, @TFPSList_Sort_Comparer, @Context);
end;

procedure TFPSList.QuickSort(L, R: Integer; Compare: TFPSListCompareFunc);
var
  Context: TFPSList_Sort_Comparer_Context;
  SortingAlgorithm: PSortingAlgorithm;
begin
  if (R > L) and (L >= 0) then
  begin
    Context.Compare := Compare;
    SortingAlgorithm := SortBase.DefaultSortingAlgorithm;
    SortingAlgorithm^.ItemListSorter_ContextComparer(FList + FItemSize*L, R-L+1, FItemSize, @TFPSList_Sort_Comparer, @Context);
  end;
end;

procedure TFPSList.AddList(Obj: TFPSList);
var
  i: Integer;
begin
  if Obj.ItemSize <> FItemSize then
    Error(SListItemSizeError, 0);
  // Do this now.
  Capacity:=Capacity+Obj.Count;
  if ItemIsManaged then
    begin
    // nothing for it, need to do it manually to give deref a chance.
    For I:=0 to Obj.Count-1 do
      Add(Obj[i])
    end
  else
    begin
    if Obj.Count=0 then
      exit;
    CopyItems(Obj.InternalItems[0],InternalItems[FCount],Obj.Count);
    FCount:=FCount+Obj.Count;
    end
end;

procedure TFPSList.Assign(Obj: TFPSList);

begin
  // We must do this check here, to avoid clearing the list.
  if Obj.ItemSize <> FItemSize then
    Error(SListItemSizeError, 0);
  Clear;
  AddList(Obj);
end;

{****************************************************************************}
{*             TFPGListEnumerator                                           *}
{****************************************************************************}

function TFPGListEnumerator.GetCurrent: T;
begin
  Result := T(FList.Items[FPosition]^);
end;

constructor TFPGListEnumerator.Create(AList: TFPSList);
begin
  inherited Create;
  FList := AList;
  FPosition := -1;
end;

function TFPGListEnumerator.MoveNext: Boolean;
begin
  inc(FPosition);
  Result := FPosition < FList.Count;
end;

{****************************************************************************}
{*                TFPGList                                                  *}
{****************************************************************************}

constructor TFPGList.Create;
begin
  inherited Create(sizeof(T));
end;

procedure TFPGList.CopyItem(Src, Dest: Pointer);
begin
  T(Dest^) := T(Src^);
end;

procedure TFPGList.Deref(Item: Pointer);
begin
 Finalize(T(Item^));
end;

function TFPGList.Get(Index: Integer): T;
begin
  Result := T(inherited Get(Index)^);
end;

function TFPGList.GetList: PTypeList;
begin
  Result := PTypeList(@FList);
end;

function TFPGList.ItemPtrCompare(Item1, Item2: Pointer): Integer;
begin
  Result := FOnCompare(T(Item1^), T(Item2^));
end;

procedure TFPGList.Put(Index: Integer; const Item: T);
begin
  inherited Put(Index, @Item);
end;

function TFPGList.Add(const Item: T): Integer;
begin
  Result := inherited Add(@Item);
end;

function TFPGList.Extract(const Item: T): T;
begin
  inherited Extract(@Item, @Result);
end;

function TFPGList.GetFirst: T;
begin
  if FCount<>0 then
    Result := T(inherited GetFirst^)
  else
    Result:=Default(T);
end;

procedure TFPGList.SetFirst(const Value: T);
begin
  inherited SetFirst(@Value);
end;

class function TFPGList.ItemIsManaged: Boolean;
begin
{$IFNDEF VER3_0}
{$IFNDEF VER3_2}
  Result:=IsManagedType(T);
{$ELSE}  
  Result:=True; // Fallback to old behaviour  
{$ENDIF}
{$ELSE}
  Result:=True; // Fallback to old behaviour  
{$ENDIF}
end;

function TFPGList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGList.IndexOf(const Item: T): Integer;
begin
  Result := 0;
  {$info TODO: fix inlining to work! InternalItems[Result]^}
  while (Result < FCount) and (PT(FList)[Result] <> Item) do
    Inc(Result);
  if Result = FCount then
    Result := -1;
end;

procedure TFPGList.Insert(Index: Integer; const Item: T);
begin
  T(inherited Insert(Index)^) := Item;
end;

function TFPGList.GetLast: T;
begin
  if FCount<>0 then
    Result := T(inherited GetLast^)
  else
    result:=Default(T);
end;

procedure TFPGList.SetLast(const Value: T);
begin
  inherited SetLast(@Value);
end;

{$ifndef VER2_4}
procedure TFPGList.AddList(Source: TFPGList);

var
  i: Integer;
  
begin
  if ItemIsManaged then
    begin
    Capacity:=Capacity+Source.Count;
    for I := 0 to Source.Count - 1 do
      Add(Source[i]);
    end
  else
    Inherited AddList(TFPSList(source))
end;

procedure TFPGList.Assign(Source: TFPGList);
begin
  if ItemIsManaged then
    begin
    Clear;
    AddList(Source);
    end
  else
    Inherited Assign(TFPSList(source))
end;
{$endif VER2_4}

function TFPGList.Remove(const Item: T): Integer;
begin
  Result := IndexOf(Item);
  if Result >= 0 then
    Delete(Result);
end;

procedure TFPGList.Sort(Compare: TCompareFunc);
begin
  FOnCompare := Compare;
  inherited Sort(@ItemPtrCompare);
end;

procedure TFPGList.Sort(Compare: TCompareFunc; SortingAlgorithm: PSortingAlgorithm);
begin
  FOnCompare := Compare;
  inherited Sort(@ItemPtrCompare, SortingAlgorithm);
end;


{****************************************************************************}
{*                TFPGObjectList                                            *}
{****************************************************************************}

constructor TFPGObjectList.Create(FreeObjects: Boolean);
begin
  inherited Create;
  FFreeObjects := FreeObjects;
end;

procedure TFPGObjectList.CopyItem(Src, Dest: Pointer);
begin
  T(Dest^) := T(Src^);
end;

procedure TFPGObjectList.Deref(Item: Pointer);
begin
  if FFreeObjects then
    T(Item^).Free;
end;

function TFPGObjectList.Get(Index: Integer): T;
begin
  Result := T(inherited Get(Index)^);
end;

function TFPGObjectList.GetList: PTypeList;
begin
  Result := PTypeList(@FList);
end;

function TFPGObjectList.ItemPtrCompare(Item1, Item2: Pointer): Integer;
begin
  Result := FOnCompare(T(Item1^), T(Item2^));
end;

procedure TFPGObjectList.Put(Index: Integer; const Item: T);
begin
  inherited Put(Index, @Item);
end;

function TFPGObjectList.Add(const Item: T): Integer;
begin
  Result := inherited Add(@Item);
end;

function TFPGObjectList.Extract(const Item: T): T;
begin
  inherited Extract(@Item, @Result);
end;

function TFPGObjectList.GetFirst: T;
begin
  Result := T(inherited GetFirst^);
end;

procedure TFPGObjectList.SetFirst(const Value: T);
begin
  inherited SetFirst(@Value);
end;

function TFPGObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGObjectList.IndexOf(const Item: T): Integer;
begin
  Result := 0;
  {$info TODO: fix inlining to work! InternalItems[Result]^}
  while (Result < FCount) and (PT(FList)[Result] <> Item) do
    Inc(Result);
  if Result = FCount then
    Result := -1;
end;

procedure TFPGObjectList.Insert(Index: Integer; const Item: T);
begin
  T(inherited Insert(Index)^) := Item;
end;

function TFPGObjectList.GetLast: T;
begin
  Result := T(inherited GetLast^);
end;

procedure TFPGObjectList.SetLast(const Value: T);
begin
  inherited SetLast(@Value);
end;

{$ifndef VER2_4}
procedure TFPGObjectList.AddList(Source: TFPGObjectList);
var
  i: Integer;
begin
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;

procedure TFPGObjectList.Assign(Source: TFPGObjectList);
begin
  Clear;
  AddList(Source);
end;
{$endif VER2_4}

function TFPGObjectList.Remove(const Item: T): Integer;
begin
  Result := IndexOf(Item);
  if Result >= 0 then
    Delete(Result);
end;

procedure TFPGObjectList.Sort(Compare: TCompareFunc);
begin
  FOnCompare := Compare;
  inherited Sort(@ItemPtrCompare);
end;

procedure TFPGObjectList.Sort(Compare: TCompareFunc; SortingAlgorithm: PSortingAlgorithm);
begin
  FOnCompare := Compare;
  inherited Sort(@ItemPtrCompare, SortingAlgorithm);
end;


{****************************************************************************}
{*                TFPGInterfacedObjectList                                  *}
{****************************************************************************}

constructor TFPGInterfacedObjectList.Create;
begin
  inherited Create;
end;

procedure TFPGInterfacedObjectList.CopyItem(Src, Dest: Pointer);
begin
  if Assigned(Pointer(Dest^)) then
    T(Dest^)._Release;
  Pointer(Dest^) := Pointer(Src^);
  if Assigned(Pointer(Dest^)) then
    T(Dest^)._AddRef;
end;

procedure TFPGInterfacedObjectList.Deref(Item: Pointer);
begin
  if Assigned(Pointer(Item^)) then
    T(Item^)._Release;
end;

function TFPGInterfacedObjectList.Get(Index: Integer): T;
begin
  Result := T(inherited Get(Index)^);
end;

function TFPGInterfacedObjectList.GetList: PTypeList;
begin
  Result := PTypeList(@FList);
end;

function TFPGInterfacedObjectList.ItemPtrCompare(Item1, Item2: Pointer): Integer;
begin
  Result := FOnCompare(T(Item1^), T(Item2^));
end;

procedure TFPGInterfacedObjectList.Put(Index: Integer; const Item: T);
begin
  inherited Put(Index, @Item);
end;

function TFPGInterfacedObjectList.Add(const Item: T): Integer;
begin
  Result := inherited Add(@Item);
end;

function TFPGInterfacedObjectList.Extract(const Item: T): T;
begin
  inherited Extract(@Item, @Result);
end;

function TFPGInterfacedObjectList.GetFirst: T;
begin
  Result := T(inherited GetFirst^);
end;

procedure TFPGInterfacedObjectList.SetFirst(const Value: T);
begin
  inherited SetFirst(@Value);
end;

function TFPGInterfacedObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGInterfacedObjectList.IndexOf(const Item: T): Integer;
begin
  Result := 0;
  {$info TODO: fix inlining to work! InternalItems[Result]^}
  while (Result < FCount) and (PT(FList)[Result] <> Item) do
    Inc(Result);
  if Result = FCount then
    Result := -1;
end;

procedure TFPGInterfacedObjectList.Insert(Index: Integer; const Item: T);
begin
  T(inherited Insert(Index)^) := Item;
end;

function TFPGInterfacedObjectList.GetLast: T;
begin
  Result := T(inherited GetLast^);
end;

procedure TFPGInterfacedObjectList.SetLast(const Value: T);
begin
  inherited SetLast(@Value);
end;

{$ifndef VER2_4}
procedure TFPGInterfacedObjectList.Assign(Source: TFPGInterfacedObjectList);

begin
  Clear;
  AddList(Source);
end;

procedure TFPGInterfacedObjectList.AddList(Source: TFPGInterfacedObjectList);
var
  i: Integer;
begin
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

function TFPGInterfacedObjectList.Remove(const Item: T): Integer;
begin
  Result := IndexOf(Item);
  if Result >= 0 then
    Delete(Result);
end;

procedure TFPGInterfacedObjectList.Sort(Compare: TCompareFunc);
begin
  FOnCompare := Compare;
  inherited Sort(@ItemPtrCompare);
end;

procedure TFPGInterfacedObjectList.Sort(Compare: TCompareFunc; SortingAlgorithm: PSortingAlgorithm);
begin
  FOnCompare := Compare;
  inherited Sort(@ItemPtrCompare, SortingAlgorithm);
end;

{****************************************************************************
                             TFPSMap
 ****************************************************************************}

constructor TFPSMap.Create(AKeySize: Integer; ADataSize: integer);
begin
  inherited Create(AKeySize+ADataSize);
  FKeySize := AKeySize;
  FDataSize := ADataSize;
  InitOnPtrCompare;
end;

procedure TFPSMap.CopyKey(Src, Dest: Pointer);
begin
  System.Move(Src^, Dest^, FKeySize);
end;

procedure TFPSMap.CopyData(Src, Dest: Pointer);
begin
  System.Move(Src^, Dest^, FDataSize);
end;

function TFPSMap.GetKey(Index: Integer): Pointer;
begin
  Result := Items[Index];
end;

function TFPSMap.GetData(Index: Integer): Pointer;
begin
  Result := PByte(Items[Index])+FKeySize;
end;

function TFPSMap.GetKeyData(AKey: Pointer): Pointer;
var
  I: Integer;
begin
  I := IndexOf(AKey);
  if I >= 0 then
    Result := InternalItems[I]+FKeySize
  else
    Error(SMapKeyError, PtrUInt(AKey));
end;

function TFPSMap.BinaryCompareKey(Key1, Key2: Pointer): Integer;
begin
  Result := CompareByte(Key1^, Key2^, FKeySize);
end;

function TFPSMap.BinaryCompareData(Data1, Data2: Pointer): Integer;
begin
  Result := CompareByte(Data1^, Data2^, FDataSize);
end;

procedure TFPSMap.SetOnKeyPtrCompare(Proc: TFPSListCompareFunc);
begin
  if Proc <> nil then
    FOnKeyPtrCompare := Proc
  else
    FOnKeyPtrCompare := @BinaryCompareKey;
end;

procedure TFPSMap.SetOnDataPtrCompare(Proc: TFPSListCompareFunc);
begin
  if Proc <> nil then
    FOnDataPtrCompare := Proc
  else
    FOnDataPtrCompare := @BinaryCompareData;
end;

procedure TFPSMap.InitOnPtrCompare;
begin
  SetOnKeyPtrCompare(nil);
  SetOnDataPtrCompare(nil);
end;

procedure TFPSMap.PutKey(Index: Integer; AKey: Pointer);
begin
  if FSorted then
    Error(SSortedListError, 0);
  CopyKey(AKey, Items[Index]);
end;

procedure TFPSMap.PutData(Index: Integer; AData: Pointer);
begin
  CopyData(AData, PByte(Items[Index])+FKeySize);
end;

procedure TFPSMap.PutKeyData(AKey: Pointer; NewData: Pointer);
var
  I: Integer;
begin
  I := IndexOf(AKey);
  if I >= 0 then
    Data[I] := NewData
  else
    Add(AKey, NewData);
end;

procedure TFPSMap.SetSorted(Value: Boolean);
begin
  if Value = FSorted then exit;
  FSorted := Value;
  if Value then Sort;
end;

function TFPSMap.Add(AKey: Pointer): Integer;
begin
  if Sorted then
  begin
    if Find(AKey, Result) then
      case Duplicates of
        dupIgnore: exit;
        dupError: Error(SDuplicateItem, 0)
      end;
  end else
    Result := Count;
  CopyKey(AKey, inherited Insert(Result));
end;

function TFPSMap.Add(AKey, AData: Pointer): Integer;
begin
  Result := Add(AKey);
  Data[Result] := AData;
end;

function TFPSMap.Find(AKey: Pointer; out Index: Integer): Boolean;
{ Searches for the first item <= Key, returns True if exact match,
  sets index to the index of the found string. }
var
  I,L,R,Dir: Integer;
begin
  Result := false;
  Index := -1;
  if not Sorted then
    raise EListError.Create(SErrFindNeedsSortedList);
  // Use binary search.
  L := 0;
  R := FCount-1;
  while L<=R do
  begin
    I := L + (R - L) div 2;
    Dir := FOnKeyPtrCompare(Items[I], AKey);
    if Dir < 0 then
      L := I+1
    else begin
      R := I-1;
      if Dir = 0 then
      begin
        Result := true;
        if Duplicates <> dupAccept then
          L := I;
      end;
    end;
  end;
  Index := L;
end;

function TFPSMap.LinearIndexOf(AKey: Pointer): Integer;
var
  ListItem: Pointer;
begin
  Result := 0;
  ListItem := First;
  while (Result < FCount) and (FOnKeyPtrCompare(ListItem, AKey) <> 0) do
  begin
    Inc(Result);
    ListItem := PByte(ListItem)+FItemSize;
  end;
  if Result = FCount then Result := -1;
end;

function TFPSMap.IndexOf(AKey: Pointer): Integer;
begin
  if Sorted then
  begin
    if not Find(AKey, Result) then
      Result := -1;
  end else
    Result := LinearIndexOf(AKey);
end;

function TFPSMap.IndexOfData(AData: Pointer): Integer;
var
  ListItem: Pointer;
begin
  Result := 0;
  ListItem := First+FKeySize;
  while (Result < FCount) and (FOnDataPtrCompare(ListItem, AData) <> 0) do
  begin
    Inc(Result);
    ListItem := PByte(ListItem)+FItemSize;
  end;
  if Result = FCount then Result := -1;
end;

function TFPSMap.Insert(Index: Integer): Pointer;
begin
  if FSorted then
    Error(SSortedListError, 0);
  Result := inherited Insert(Index);
end;

procedure TFPSMap.Insert(Index: Integer; out AKey, AData: Pointer);
begin
  AKey := Insert(Index);
  AData := PByte(AKey) + FKeySize;
end;

procedure TFPSMap.InsertKey(Index: Integer; AKey: Pointer);
begin
  CopyKey(AKey, Insert(Index));
end;

procedure TFPSMap.InsertKeyData(Index: Integer; AKey, AData: Pointer);
var
  ListItem: Pointer;
begin
  ListItem := Insert(Index);
  CopyKey(AKey, ListItem);
  CopyData(AData, PByte(ListItem)+FKeySize);
end;

function TFPSMap.Remove(AKey: Pointer): Integer;
begin
  Result := IndexOf(AKey);
  if Result >= 0 then
    Delete(Result);
end;

procedure TFPSMap.Sort;
begin
  inherited Sort(FOnKeyPtrCompare);
end;

procedure TFPSMap.Sort(SortingAlgorithm: PSortingAlgorithm);
begin
  inherited Sort(FOnKeyPtrCompare, SortingAlgorithm);
end;

{****************************************************************************
                             TFPGMap
 ****************************************************************************}

constructor TFPGMap.Create;
begin
  inherited Create(SizeOf(TKey), SizeOf(TData));
end;

procedure TFPGMap.CopyItem(Src, Dest: Pointer);
begin
  CopyKey(Src, Dest);
  CopyData(PByte(Src)+KeySize, PByte(Dest)+KeySize);
end;

procedure TFPGMap.CopyKey(Src, Dest: Pointer);
begin
  TKey(Dest^) := TKey(Src^);
end;

procedure TFPGMap.CopyData(Src, Dest: Pointer);
begin
  TData(Dest^) := TData(Src^);
end;

procedure TFPGMap.Deref(Item: Pointer);
begin
  Finalize(TKey(Item^));
  Finalize(TData(Pointer(PByte(Item)+KeySize)^));
end;

function TFPGMap.GetKey(Index: Integer): TKey;
begin
  Result := TKey(inherited GetKey(Index)^);
end;

function TFPGMap.GetData(Index: Integer): TData;
begin
  Result := TData(inherited GetData(Index)^);
end;

function TFPGMap.GetKeyData(const AKey: TKey): TData;
begin
  Result := TData(inherited GetKeyData(@AKey)^);
end;

function TFPGMap.KeyCompare(Key1, Key2: Pointer): Integer;
begin
  if PKey(Key1)^ < PKey(Key2)^ then
    Result := -1
  else if PKey(Key1)^ > PKey(Key2)^ then
    Result := 1
  else
    Result := 0;
end;

{function TFPGMap.DataCompare(Data1, Data2: Pointer): Integer;
begin
  if PData(Data1)^ < PData(Data2)^ then
    Result := -1
  else if PData(Data1)^ > PData(Data2)^ then
    Result := 1
  else
    Result := 0;
end;}

function TFPGMap.KeyCustomCompare(Key1, Key2: Pointer): Integer;
begin
  Result := FOnKeyCompare(TKey(Key1^), TKey(Key2^));
end;

function TFPGMap.DataCustomCompare(Data1, Data2: Pointer): Integer;
begin
  Result := FOnDataCompare(TData(Data1^), TData(Data2^));
end;

procedure TFPGMap.SetOnKeyCompare(NewCompare: TKeyCompareFunc);
begin
  FOnKeyCompare := NewCompare;
  if NewCompare <> nil then
    OnKeyPtrCompare := @KeyCustomCompare
  else
    OnKeyPtrCompare := @KeyCompare;
end;

procedure TFPGMap.SetOnDataCompare(NewCompare: TDataCompareFunc);
begin
  FOnDataCompare := NewCompare;
  if NewCompare <> nil then
    OnDataPtrCompare := @DataCustomCompare
  else
    OnDataPtrCompare := nil;
end;

procedure TFPGMap.InitOnPtrCompare;
begin
  SetOnKeyCompare(nil);
  SetOnDataCompare(nil);
end;

procedure TFPGMap.PutKey(Index: Integer; const NewKey: TKey);
begin
  inherited PutKey(Index, @NewKey);
end;

procedure TFPGMap.PutData(Index: Integer; const NewData: TData);
begin
  inherited PutData(Index, @NewData);
end;

procedure TFPGMap.PutKeyData(const AKey: TKey; const NewData: TData);
begin
  inherited PutKeyData(@AKey, @NewData);
end;

function TFPGMap.Add(const AKey: TKey): Integer;
begin
  Result := inherited Add(@AKey);
end;

function TFPGMap.Add(const AKey: TKey; const AData: TData): Integer;
begin
  Result := inherited Add(@AKey, @AData);
end;

function TFPGMap.Find(const AKey: TKey; out Index: Integer): Boolean;
begin
  Result := inherited Find(@AKey, Index);
end;

function TFPGMap.TryGetData(const AKey: TKey; out AData: TData): Boolean;
var
  I: Integer;
begin
  I := IndexOf(AKey);
  Result := (I >= 0);
  if Result then
    AData := TData(inherited GetData(I)^)
  else
    AData := Default(TData);
end;

procedure TFPGMap.AddOrSetData(const AKey: TKey; const AData: TData);
begin
  inherited PutKeyData(@AKey, @AData);
end;

function TFPGMap.IndexOf(const AKey: TKey): Integer;
begin
  Result := inherited IndexOf(@AKey);
end;

function TFPGMap.IndexOfData(const AData: TData): Integer;
begin
  { TODO: loop ? }
  Result := inherited IndexOfData(@AData);
end;

procedure TFPGMap.InsertKey(Index: Integer; const AKey: TKey);
begin
  inherited InsertKey(Index, @AKey);
end;

procedure TFPGMap.InsertKeyData(Index: Integer; const AKey: TKey; const AData: TData);
begin
  inherited InsertKeyData(Index, @AKey, @AData);
end;

function TFPGMap.Remove(const AKey: TKey): Integer;
begin
  Result := inherited Remove(@AKey);
end;

{****************************************************************************
                             TFPGMapObject
 ****************************************************************************}

constructor TFPGMapObject.Create(AFreeObjects: Boolean);
begin
  inherited Create(SizeOf(TKey), SizeOf(TData));
  FFreeObjects := AFreeObjects;
end;

constructor TFPGMapObject.Create;
begin
  Create(True);
end;

procedure TFPGMapObject.CopyItem(Src, Dest: Pointer);
begin
  CopyKey(Src, Dest);
  CopyData(PByte(Src)+KeySize, PByte(Dest)+KeySize);
end;

procedure TFPGMapObject.CopyKey(Src, Dest: Pointer);
begin
  TKey(Dest^) := TKey(Src^);
end;

procedure TFPGMapObject.CopyData(Src, Dest: Pointer);
begin
  if Assigned(Pointer(Dest^)) then
    TData(Dest^).Free;
  TData(Dest^) := TData(Src^);
end;

procedure TFPGMapObject.Deref(Item: Pointer);
begin
  Finalize(TKey(Item^));
  if Assigned(PPointer(PByte(Item)+KeySize)^) and FFreeObjects then
    TData(Pointer(PByte(Item)+KeySize)^).Free;
end;

function TFPGMapObject.GetKey(Index: Integer): TKey;
begin
  Result := TKey(inherited GetKey(Index)^);
end;

function TFPGMapObject.GetData(Index: Integer): TData;
begin
  Result := TData(inherited GetData(Index)^);
end;

function TFPGMapObject.GetKeyData(const AKey: TKey): TData;
begin
  Result := TData(inherited GetKeyData(@AKey)^);
end;

function TFPGMapObject.KeyCompare(Key1, Key2: Pointer): Integer;
begin
  if PKey(Key1)^ < PKey(Key2)^ then
    Result := -1
  else if PKey(Key1)^ > PKey(Key2)^ then
    Result := 1
  else
    Result := 0;
end;

{function TFPGMapObject.DataCompare(Data1, Data2: Pointer): Integer;
begin
  if PData(Data1)^ < PData(Data2)^ then
    Result := -1
  else if PData(Data1)^ > PData(Data2)^ then
    Result := 1
  else
    Result := 0;
end;}

function TFPGMapObject.KeyCustomCompare(Key1, Key2: Pointer): Integer;
begin
  Result := FOnKeyCompare(TKey(Key1^), TKey(Key2^));
end;

function TFPGMapObject.DataCustomCompare(Data1, Data2: Pointer): Integer;
begin
  Result := FOnDataCompare(TData(Data1^), TData(Data2^));
end;

procedure TFPGMapObject.SetOnKeyCompare(NewCompare: TKeyCompareFunc);
begin
  FOnKeyCompare := NewCompare;
  if NewCompare <> nil then
    OnKeyPtrCompare := @KeyCustomCompare
  else
    OnKeyPtrCompare := @KeyCompare;
end;

procedure TFPGMapObject.SetOnDataCompare(NewCompare: TDataCompareFunc);
begin
  FOnDataCompare := NewCompare;
  if NewCompare <> nil then
    OnDataPtrCompare := @DataCustomCompare
  else
    OnDataPtrCompare := nil;
end;

procedure TFPGMapObject.InitOnPtrCompare;
begin
  SetOnKeyCompare(nil);
  SetOnDataCompare(nil);
end;

procedure TFPGMapObject.PutKey(Index: Integer; const NewKey: TKey);
begin
  inherited PutKey(Index, @NewKey);
end;

procedure TFPGMapObject.PutData(Index: Integer; const NewData: TData);
begin
  inherited PutData(Index, @NewData);
end;

procedure TFPGMapObject.PutKeyData(const AKey: TKey; const NewData: TData);
begin
  inherited PutKeyData(@AKey, @NewData);
end;

function TFPGMapObject.Add(const AKey: TKey): Integer;
begin
  Result := inherited Add(@AKey);
end;

function TFPGMapObject.Add(const AKey: TKey; const AData: TData): Integer;
begin
  Result := inherited Add(@AKey, @AData);
end;

function TFPGMapObject.Find(const AKey: TKey; out Index: Integer): Boolean;
begin
  Result := inherited Find(@AKey, Index);
end;

function TFPGMapObject.TryGetData(const AKey: TKey; out AData: TData): Boolean;
var
  I: Integer;
begin
  I := IndexOf(AKey);
  Result := (I >= 0);
  if Result then
    AData := TData(inherited GetData(I)^)
  else
    AData := Default(TData);
end;

procedure TFPGMapObject.AddOrSetData(const AKey: TKey; const AData: TData);
begin
  inherited PutKeyData(@AKey, @AData);
end;

function TFPGMapObject.IndexOf(const AKey: TKey): Integer;
begin
  Result := inherited IndexOf(@AKey);
end;

function TFPGMapObject.IndexOfData(const AData: TData): Integer;
begin
  { TODO: loop ? }
  Result := inherited IndexOfData(@AData);
end;

procedure TFPGMapObject.InsertKey(Index: Integer; const AKey: TKey);
begin
  inherited InsertKey(Index, @AKey);
end;

procedure TFPGMapObject.InsertKeyData(Index: Integer; const AKey: TKey; const AData: TData);
begin
  inherited InsertKeyData(Index, @AKey, @AData);
end;

function TFPGMapObject.Remove(const AKey: TKey): Integer;
begin
  Result := inherited Remove(@AKey);
end;

{****************************************************************************
                             TFPGMapInterfacedObjectData
 ****************************************************************************}

constructor TFPGMapInterfacedObjectData.Create;
begin
  inherited Create(SizeOf(TKey), SizeOf(TData));
end;

procedure TFPGMapInterfacedObjectData.CopyItem(Src, Dest: Pointer);
begin
  CopyKey(Src, Dest);
  CopyData(PByte(Src)+KeySize, PByte(Dest)+KeySize);
end;

procedure TFPGMapInterfacedObjectData.CopyKey(Src, Dest: Pointer);
begin
  TKey(Dest^) := TKey(Src^);
end;

procedure TFPGMapInterfacedObjectData.CopyData(Src, Dest: Pointer);
begin
  if Assigned(Pointer(Dest^)) then
    TData(Dest^)._Release;
  TData(Dest^) := TData(Src^);
  if Assigned(Pointer(Dest^)) then
    TData(Dest^)._AddRef;
end;

procedure TFPGMapInterfacedObjectData.Deref(Item: Pointer);
begin
  Finalize(TKey(Item^));
  if Assigned(PPointer(PByte(Item)+KeySize)^) then
    TData(Pointer(PByte(Item)+KeySize)^)._Release;
end;

function TFPGMapInterfacedObjectData.GetKey(Index: Integer): TKey;
begin
  Result := TKey(inherited GetKey(Index)^);
end;

function TFPGMapInterfacedObjectData.GetData(Index: Integer): TData;
begin
  Result := TData(inherited GetData(Index)^);
end;

function TFPGMapInterfacedObjectData.GetKeyData(const AKey: TKey): TData;
begin
  Result := TData(inherited GetKeyData(@AKey)^);
end;

function TFPGMapInterfacedObjectData.KeyCompare(Key1, Key2: Pointer): Integer;
begin
  if PKey(Key1)^ < PKey(Key2)^ then
    Result := -1
  else if PKey(Key1)^ > PKey(Key2)^ then
    Result := 1
  else
    Result := 0;
end;

{function TFPGMapInterfacedObjectData.DataCompare(Data1, Data2: Pointer): Integer;
begin
  if PData(Data1)^ < PData(Data2)^ then
    Result := -1
  else if PData(Data1)^ > PData(Data2)^ then
    Result := 1
  else
    Result := 0;
end;}

function TFPGMapInterfacedObjectData.KeyCustomCompare(Key1, Key2: Pointer): Integer;
begin
  Result := FOnKeyCompare(TKey(Key1^), TKey(Key2^));
end;

function TFPGMapInterfacedObjectData.DataCustomCompare(Data1, Data2: Pointer): Integer;
begin
  Result := FOnDataCompare(TData(Data1^), TData(Data2^));
end;

procedure TFPGMapInterfacedObjectData.SetOnKeyCompare(NewCompare: TKeyCompareFunc);
begin
  FOnKeyCompare := NewCompare;
  if NewCompare <> nil then
    OnKeyPtrCompare := @KeyCustomCompare
  else
    OnKeyPtrCompare := @KeyCompare;
end;

procedure TFPGMapInterfacedObjectData.SetOnDataCompare(NewCompare: TDataCompareFunc);
begin
  FOnDataCompare := NewCompare;
  if NewCompare <> nil then
    OnDataPtrCompare := @DataCustomCompare
  else
    OnDataPtrCompare := nil;
end;

procedure TFPGMapInterfacedObjectData.InitOnPtrCompare;
begin
  SetOnKeyCompare(nil);
  SetOnDataCompare(nil);
end;

procedure TFPGMapInterfacedObjectData.PutKey(Index: Integer; const NewKey: TKey);
begin
  inherited PutKey(Index, @NewKey);
end;

procedure TFPGMapInterfacedObjectData.PutData(Index: Integer; const NewData: TData);
begin
  inherited PutData(Index, @NewData);
end;

procedure TFPGMapInterfacedObjectData.PutKeyData(const AKey: TKey; const NewData: TData);
begin
  inherited PutKeyData(@AKey, @NewData);
end;

function TFPGMapInterfacedObjectData.Add(const AKey: TKey): Integer;
begin
  Result := inherited Add(@AKey);
end;

function TFPGMapInterfacedObjectData.Add(const AKey: TKey; const AData: TData): Integer;
begin
  Result := inherited Add(@AKey, @AData);
end;

function TFPGMapInterfacedObjectData.Find(const AKey: TKey; out Index: Integer): Boolean;
begin
  Result := inherited Find(@AKey, Index);
end;

function TFPGMapInterfacedObjectData.TryGetData(const AKey: TKey; out AData: TData): Boolean;
var
  I: Integer;
begin
  I := IndexOf(AKey);
  Result := (I >= 0);
  if Result then
    AData := TData(inherited GetData(I)^)
  else
    AData := Default(TData);
end;

procedure TFPGMapInterfacedObjectData.AddOrSetData(const AKey: TKey;
  const AData: TData);
begin
  inherited PutKeyData(@AKey, @AData);
end;

function TFPGMapInterfacedObjectData.IndexOf(const AKey: TKey): Integer;
begin
  Result := inherited IndexOf(@AKey);
end;

function TFPGMapInterfacedObjectData.IndexOfData(const AData: TData): Integer;
begin
  { TODO: loop ? }
  Result := inherited IndexOfData(@AData);
end;

procedure TFPGMapInterfacedObjectData.InsertKey(Index: Integer; const AKey: TKey);
begin
  inherited InsertKey(Index, @AKey);
end;

procedure TFPGMapInterfacedObjectData.InsertKeyData(Index: Integer; const AKey: TKey; const AData: TData);
begin
  inherited InsertKeyData(Index, @AKey, @AData);
end;

function TFPGMapInterfacedObjectData.Remove(const AKey: TKey): Integer;
begin
  Result := inherited Remove(@AKey);
end;

end.
