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

{.$define CLASSESINLINE}

{ be aware, this unit is a prototype and subject to be changed heavily }
unit fgl;

interface

uses
  types, sysutils;

{$IF defined(VER2_4)}
  {$DEFINE OldSyntax}
{$IFEND}

const
  MaxListSize = Maxint div 16;

type
  EListError = class(Exception);

  TFPSList = class;
  TFPSListCompareFunc = function(Key1, Key2: Pointer): Integer of object;

  TFPSList = class(TObject)
  protected
    FList: PByte;
    FCount: Integer;
    FCapacity: Integer; { list is one longer sgthan capacity, for temp }
    FItemSize: Integer;
    procedure CopyItem(Src, Dest: Pointer); virtual;
    procedure Deref(Item: Pointer); virtual; overload;
    procedure Deref(FromIndex, ToIndex: Integer); overload;
    function Get(Index: Integer): Pointer;
    procedure InternalExchange(Index1, Index2: Integer);
    function  InternalGet(Index: Integer): Pointer; {$ifdef CLASSESINLINE} inline; {$endif}
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
  public
    constructor Create(AItemSize: Integer = sizeof(Pointer));
    destructor Destroy; override;
    function Add(Item: Pointer): Integer;
    procedure Clear;
    procedure Delete(Index: Integer);
    class procedure Error(const Msg: string; Data: PtrInt);
    procedure Exchange(Index1, Index2: Integer);
    function Expand: TFPSList;
    procedure Extract(Item: Pointer; ResultPtr: Pointer);
    function IndexOf(Item: Pointer): Integer;
    procedure Insert(Index: Integer; Item: Pointer);
    function Insert(Index: Integer): Pointer;
    procedure Move(CurIndex, NewIndex: Integer);
    procedure Assign(Obj: TFPSList);
    function Remove(Item: Pointer): Integer;
    procedure Pack;
    procedure Sort(Compare: TFPSListCompareFunc);
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
  MaxGListSize = {MaxInt div} 1024;
{$else cpu16}
  MaxGListSize = MaxInt div 1024;
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

const
  MaxGListSize = MaxInt div 1024;

<<<<<<< HEAD
=======
const
  MaxGListSize = MaxInt div 1024;

>>>>>>> origin/fixes_2_2
type
  generic TFPGList<T> = class(TFPSList)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  private
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> origin/cpstrnew
    type
      TCompareFunc = function(const Item1, Item2: T): Integer;
      TTypeList = array[0..MaxGListSize] of T;
      PTypeList = ^TTypeList;
      PT = ^T;
      TFPGListEnumeratorSpec = specialize TFPGListEnumerator<T>;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnCompare: TCompareFunc;
=======
  type public
    TCompareFunc = function(const Item1, Item2: T): Integer;
    TTypeList = array[0..MaxGListSize] of T;
    PTypeList = ^TTypeList;
    PT = ^T;
  var protected
    FOnCompare: TCompareFunc;
>>>>>>> graemeg/fixes_2_2
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    function  Get(Index: Integer): T; {$ifdef CLASSESINLINE} inline; {$endif}
    function  GetList: PTypeList; {$ifdef CLASSESINLINE} inline; {$endif}
    function  ItemPtrCompare(Item1, Item2: Pointer): Integer;
    procedure Put(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function GetLast: T; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetLast(const Value: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function GetFirst: T; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetFirst(const Value: T); {$ifdef CLASSESINLINE} inline; {$endif}
  public
    constructor Create;
    function Add(const Item: T): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Extract(const Item: T): T; {$ifdef CLASSESINLINE} inline; {$endif}
<<<<<<< HEAD
    property First: T read GetFirst write SetFirst;
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    property Last: T read GetLast write SetLast;
=======
    function First: T; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function Last: T; {$ifdef CLASSESINLINE} inline; {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef VER2_4}
    procedure Assign(Source: TFPGList);
{$endif VER2_4}
    function Remove(const Item: T): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure Sort(Compare: TCompareFunc);
    property Items[Index: Integer]: T read Get write Put; default;
    property List: PTypeList read GetList;
  end;

  generic TFPGObjectList<T> = class(TFPSList)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  private
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> origin/cpstrnew
    type
      TCompareFunc = function(const Item1, Item2: T): Integer;
      TTypeList = array[0..MaxGListSize] of T;
      PTypeList = ^TTypeList;
      PT = ^T;
      TFPGListEnumeratorSpec = specialize TFPGListEnumerator<T>;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnCompare: TCompareFunc;
      FFreeObjects: Boolean;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    function  Get(Index: Integer): T; {$ifdef CLASSESINLINE} inline; {$endif}
    function  GetList: PTypeList; {$ifdef CLASSESINLINE} inline; {$endif}
    function  ItemPtrCompare(Item1, Item2: Pointer): Integer;
    procedure Put(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function GetLast: T; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetLast(const Value: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function GetFirst: T; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetFirst(const Value: T); {$ifdef CLASSESINLINE} inline; {$endif}
  public
    constructor Create(FreeObjects: Boolean = True);
    function Add(const Item: T): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Extract(const Item: T): T; {$ifdef CLASSESINLINE} inline; {$endif}
<<<<<<< HEAD
    property First: T read GetFirst write SetFirst;
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    property Last: T read GetLast write SetLast;
=======
    function First: T; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function Last: T; {$ifdef CLASSESINLINE} inline; {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef VER2_4}
    procedure Assign(Source: TFPGObjectList);
{$endif VER2_4}
=======
=======
>>>>>>> origin/fixes_2_2
    {$info FIXME: bug #10479: implement TFPGList<T>.Assign(TFPGList) to work somehow}
    {procedure Assign(Source: TFPGList);}
>>>>>>> graemeg/fixes_2_2
    function Remove(const Item: T): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure Sort(Compare: TCompareFunc);
    property Items[Index: Integer]: T read Get write Put; default;
    property List: PTypeList read GetList;
<<<<<<< HEAD
<<<<<<< HEAD
    property FreeObjects: Boolean read FFreeObjects write FFreeObjects;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  generic TFPGInterfacedObjectList<T> = class(TFPSList)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  private
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> origin/cpstrnew
    type
      TCompareFunc = function(const Item1, Item2: T): Integer;
      TTypeList = array[0..MaxGListSize] of T;
      PTypeList = ^TTypeList;
      PT = ^T;
      TFPGListEnumeratorSpec = specialize TFPGListEnumerator<T>;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnCompare: TCompareFunc;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    function  Get(Index: Integer): T; {$ifdef CLASSESINLINE} inline; {$endif}
    function  GetList: PTypeList; {$ifdef CLASSESINLINE} inline; {$endif}
    function  ItemPtrCompare(Item1, Item2: Pointer): Integer;
    procedure Put(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function GetLast: T; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetLast(const Value: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function GetFirst: T; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetFirst(const Value: T); {$ifdef CLASSESINLINE} inline; {$endif}
  public
    constructor Create;
    function Add(const Item: T): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Extract(const Item: T): T; {$ifdef CLASSESINLINE} inline; {$endif}
<<<<<<< HEAD
    property First: T read GetFirst write SetFirst;
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    property Last: T read GetLast write SetLast;
=======
    function First: T; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetEnumerator: TFPGListEnumeratorSpec; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const Item: T): Integer;
    procedure Insert(Index: Integer; const Item: T); {$ifdef CLASSESINLINE} inline; {$endif}
    function Last: T; {$ifdef CLASSESINLINE} inline; {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef VER2_4}
    procedure Assign(Source: TFPGInterfacedObjectList);
{$endif VER2_4}
    function Remove(const Item: T): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    procedure Sort(Compare: TCompareFunc);
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
    procedure InitOnPtrCompare; virtual;
>>>>>>> graemeg/fixes_2_2
=======
    procedure InitOnPtrCompare; virtual;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  private
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> origin/cpstrnew
    type
      TKeyCompareFunc = function(const Key1, Key2: TKey): Integer;
      TDataCompareFunc = function(const Data1, Data2: TData): Integer;
      PKey = ^TKey;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// unsed      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
=======
  type public
    TCompareFunc = function(const Key1, Key2: TKey): Integer;
    PKey = ^TKey;
    PData = ^TData;
  var protected
    FOnCompare: TCompareFunc;
>>>>>>> graemeg/fixes_2_2
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure CopyKey(Src, Dest: Pointer); override;
    procedure CopyData(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    procedure InitOnPtrCompare; override;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
      procedure CopyItem(Src, Dest: Pointer); override;
      procedure CopyKey(Src, Dest: Pointer); override;
      procedure CopyData(Src, Dest: Pointer); override;
      procedure Deref(Item: Pointer); override;
      procedure InitOnPtrCompare; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function GetKey(Index: Integer): TKey; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetKeyData(const AKey: TKey): TData; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetData(Index: Integer): TData; {$ifdef CLASSESINLINE} inline; {$endif}
    function KeyCompare(Key1, Key2: Pointer): Integer;
    function KeyCustomCompare(Key1, Key2: Pointer): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    //function DataCompare(Data1, Data2: Pointer): Integer;
    function DataCustomCompare(Data1, Data2: Pointer): Integer;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure PutKey(Index: Integer; const NewKey: TKey); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure PutKeyData(const AKey: TKey; const NewData: TData); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure PutData(Index: Integer; const NewData: TData); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetOnKeyCompare(NewCompare: TKeyCompareFunc);
    procedure SetOnDataCompare(NewCompare: TDataCompareFunc);
  public
    constructor Create;
    function Add(const AKey: TKey; const AData: TData): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Add(const AKey: TKey): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Find(const AKey: TKey; out Index: Integer): Boolean; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const AKey: TKey): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  private
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> origin/cpstrnew
    type
      TKeyCompareFunc = function(const Key1, Key2: TKey): Integer;
      TDataCompareFunc = function(const Data1, Data2: TData): Integer;
      PKey = ^TKey;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// unsed      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
    procedure CopyItem(Src, Dest: Pointer); override;
    procedure CopyKey(Src, Dest: Pointer); override;
    procedure CopyData(Src, Dest: Pointer); override;
    procedure Deref(Item: Pointer); override;
    procedure InitOnPtrCompare; override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      PData = ^TData;
  {$ifndef OldSyntax}protected var{$else}var protected{$endif}
      FOnKeyCompare: TKeyCompareFunc;
      FOnDataCompare: TDataCompareFunc;
      procedure CopyItem(Src, Dest: Pointer); override;
      procedure CopyKey(Src, Dest: Pointer); override;
      procedure CopyData(Src, Dest: Pointer); override;
      procedure Deref(Item: Pointer); override;
      procedure InitOnPtrCompare; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function GetKey(Index: Integer): TKey; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetKeyData(const AKey: TKey): TData; {$ifdef CLASSESINLINE} inline; {$endif}
    function GetData(Index: Integer): TData; {$ifdef CLASSESINLINE} inline; {$endif}
    function KeyCompare(Key1, Key2: Pointer): Integer;
    function KeyCustomCompare(Key1, Key2: Pointer): Integer;
    //function DataCompare(Data1, Data2: Pointer): Integer;
    function DataCustomCompare(Data1, Data2: Pointer): Integer;
    procedure PutKey(Index: Integer; const NewKey: TKey); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure PutKeyData(const AKey: TKey; const NewData: TData); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure PutData(Index: Integer; const NewData: TData); {$ifdef CLASSESINLINE} inline; {$endif}
    procedure SetOnKeyCompare(NewCompare: TKeyCompareFunc);
    procedure SetOnDataCompare(NewCompare: TDataCompareFunc);
  public
    constructor Create;
    function Add(const AKey: TKey; const AData: TData): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Add(const AKey: TKey): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
    function Find(const AKey: TKey; out Index: Integer): Boolean; {$ifdef CLASSESINLINE} inline; {$endif}
    function IndexOf(const AKey: TKey): Integer; {$ifdef CLASSESINLINE} inline; {$endif}
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
  if (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  Result := InternalItems[Index];
end;

procedure TFPSList.Put(Index: Integer; Item: Pointer);
var p : Pointer;
begin
  if (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
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
<<<<<<< HEAD
<<<<<<< HEAD
    FillByte(InternalItems[FCount]^, (NewCount-FCount) * FItemSize, 0)
  else if NewCount < FCount then
    Deref(NewCount, FCount-1);
=======
  begin
    if NewCount > FCapacity then
      SetCapacity(NewCount);
    if NewCount > FCount then
      FillByte(InternalItems[FCount]^, (NewCount-FCount) * FItemSize, 0)
    else if NewCount < FCount then
      Deref(NewCount, FCount-1);
  end;
>>>>>>> graemeg/fixes_2_2
=======
    FillByte(InternalItems[FCount]^, (NewCount-FCount) * FItemSize, 0)
  else if NewCount < FCount then
    Deref(NewCount, FCount-1);
>>>>>>> origin/fixes_2.4
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
  if (Index < 0) or (Index >= FCount) then
    Error(SListIndexError, Index);
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
  if ((Index1 >= FCount) or (Index1 < 0)) then
    Error(SListIndexError, Index1);
  if ((Index2 >= FCount) or (Index2 < 0)) then
    Error(SListIndexError, Index2);
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
  if (CurIndex < 0) or (CurIndex >= Count) then
    Error(SListIndexError, CurIndex);
  if (NewIndex < 0) or (NewIndex >= Count) then
    Error(SListIndexError, NewIndex);
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

// Needed by Sort method.

procedure TFPSList.QuickSort(L, R: Integer; Compare: TFPSListCompareFunc);
var
  I, J, P: Integer;
  PivotItem: Pointer;
begin
  repeat
    I := L;
    J := R;
    { cast to dword to avoid overflow to a negative number during addition which
      would result again in a negative number when being divided }
    P := (dword(L) + dword(R)) div 2;
    repeat
      PivotItem := InternalItems[P];
      while Compare(PivotItem, InternalItems[I]) > 0 do
        Inc(I);
      while Compare(PivotItem, InternalItems[J]) < 0 do
        Dec(J);
      if I <= J then
      begin
        InternalExchange(I, J);
        if P = I then
          P := J
        else if P = J then
          P := I;
        Inc(I);
        Dec(J);
      end;
    until I > J;
    if L < J then
      QuickSort(L, J, Compare);
    L := I;
  until I >= R;
end;

procedure TFPSList.Sort(Compare: TFPSListCompareFunc);
begin
  if not Assigned(FList) or (FCount < 2) then exit;
  QuickSort(0, FCount-1, Compare);
end;

procedure TFPSList.Assign(Obj: TFPSList);
var
  i: Integer;
begin
  if Obj.ItemSize <> FItemSize then
    Error(SListItemSizeError, 0);
  Clear;
  for I := 0 to Obj.Count - 1 do
    Add(Obj[i]);
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
  Result := PTypeList(FList);
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
<<<<<<< HEAD
  inherited Extract(@Item, @Result);
=======
  ResPtr := inherited Extract(@Item);
  if ResPtr <> nil then
    Result := T(ResPtr^)
  else
    FillByte(Result, sizeof(T), 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

function TFPGList.GetFirst: T;
begin
  Result := T(inherited GetFirst^);
end;

procedure TFPGList.SetFirst(const Value: T);
begin
  inherited SetFirst(@Value);
end;

function TFPGList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

function TFPGList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
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
  Result := T(inherited GetLast^);
end;

<<<<<<< HEAD
procedure TFPGList.SetLast(const Value: T);
begin
  inherited SetLast(@Value);
end;

=======
>>>>>>> graemeg/cpstrnew
{$ifndef VER2_4}
procedure TFPGList.Assign(Source: TFPGList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

{$ifndef VER2_4}
procedure TFPGList.Assign(Source: TFPGList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

{$ifndef VER2_4}
procedure TFPGList.Assign(Source: TFPGList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

{$ifndef VER2_4}
procedure TFPGList.Assign(Source: TFPGList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
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
  Result := PTypeList(FList);
end;

function TFPGObjectList.ItemPtrCompare(Item1, Item2: Pointer): Integer;
begin
  Result := FOnCompare(T(Item1^), T(Item2^));
end;

procedure TFPGObjectList.Put(Index: Integer; const Item: T);
begin
<<<<<<< HEAD
  inherited Put(Index, @Item);
end;

function TFPGObjectList.Add(const Item: T): Integer;
begin
  Result := inherited Add(@Item);
end;

function TFPGObjectList.Extract(const Item: T): T;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Extract(@Item, @Result);
=======
  ResPtr := inherited Extract(@Item);
  if ResPtr <> nil then
    Result := T(ResPtr^)
  else
    FillByte(Result, sizeof(T), 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  I := IndexOf(AKey);
  if I >= 0 then
    Result := InternalItems[I]+FKeySize
  else
    Error(SMapKeyError, PtrUInt(AKey));
>>>>>>> graemeg/fixes_2_2
=======
  I := IndexOf(AKey);
  if I >= 0 then
    Result := InternalItems[I]+FKeySize
  else
    Error(SMapKeyError, PtrUInt(AKey));
>>>>>>> origin/fixes_2_2
end;

function TFPGObjectList.GetFirst: T;
begin
  Result := T(inherited GetFirst^);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TFPGObjectList.SetFirst(const Value: T);
begin
  inherited SetFirst(@Value);
end;

function TFPGObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
=======
>>>>>>> graemeg/cpstrnew
end;

function TFPGObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := TFPGListEnumeratorSpec.Create(Self);
=======
=======
>>>>>>> origin/fixes_2_2
  I := IndexOf(AKey);
  if I >= 0 then
    Data[I] := NewData
  else
    Add(AKey, NewData);
>>>>>>> graemeg/fixes_2_2
end;

function TFPGObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
<<<<<<< HEAD
  Result := TFPGListEnumeratorSpec.Create(Self);
=======
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
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TFPGObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

<<<<<<< HEAD
function TFPGObjectList.IndexOf(const Item: T): Integer;
=======
function TFPSMap.Find(AKey: Pointer; var Index: Integer): Boolean;
{ Searches for the first item <= Key, returns True if exact match,
  sets index to the index f the found string. }
var
  I,L,R,Dir: Integer;
>>>>>>> graemeg/fixes_2_2
begin
<<<<<<< HEAD
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

<<<<<<< HEAD
procedure TFPGObjectList.SetLast(const Value: T);
begin
  inherited SetLast(@Value);
end;

{$ifndef VER2_4}
procedure TFPGObjectList.Assign(Source: TFPGObjectList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

{$ifndef VER2_4}
procedure TFPGObjectList.Assign(Source: TFPGObjectList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

=======
>>>>>>> graemeg/cpstrnew
{$ifndef VER2_4}
procedure TFPGObjectList.Assign(Source: TFPGObjectList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

{$ifndef VER2_4}
procedure TFPGObjectList.Assign(Source: TFPGObjectList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Pointer(Dest^) := Pointer(Src^);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  T(Dest^) := T(Src^);
>>>>>>> graemeg/cpstrnew
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
  Result := PTypeList(FList);
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
<<<<<<< HEAD
  inherited Extract(@Item, @Result);
=======
  ResPtr := inherited Extract(@Item);
  if ResPtr <> nil then
    Result := T(ResPtr^)
  else
    FillByte(Result, sizeof(T), 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

function TFPGInterfacedObjectList.GetFirst: T;
begin
  Result := T(inherited GetFirst^);
end;

procedure TFPGInterfacedObjectList.SetFirst(const Value: T);
begin
  inherited SetFirst(@Value);
=======
>>>>>>> origin/cpstrnew
end;

function TFPGInterfacedObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
=======
>>>>>>> graemeg/cpstrnew
end;

function TFPGInterfacedObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGInterfacedObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
end;

function TFPGInterfacedObjectList.GetEnumerator: TFPGListEnumeratorSpec;
begin
  Result := TFPGListEnumeratorSpec.Create(Self);
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
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef VER2_4}
procedure TFPGInterfacedObjectList.Assign(Source: TFPGInterfacedObjectList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
{$ifndef VER2_4}
procedure TFPGInterfacedObjectList.Assign(Source: TFPGInterfacedObjectList);
var
  i: Integer;
begin
  Clear;
  for I := 0 to Source.Count - 1 do
    Add(Source[i]);
end;
{$endif VER2_4}

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result := CompareByte(Data1^, Data2^, FDataSize);
=======
  Result := CompareByte(Data1^, Data1^, FDataSize);
>>>>>>> graemeg/cpstrnew
=======
  Result := CompareByte(Data1^, Data1^, FDataSize);
>>>>>>> graemeg/cpstrnew
=======
  Result := CompareByte(Data1^, Data1^, FDataSize);
>>>>>>> graemeg/cpstrnew
=======
  Result := CompareByte(Data1^, Data1^, FDataSize);
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> origin/fixes_2_2
  Result := false;
  // Use binary search.
  L := 0;
  R := FCount-1;
  while L<=R do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    I := L + (R - L) div 2;
=======
    I := (L+R) div 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Dir := FOnKeyPtrCompare(Items[I], AKey);
=======
    I := (L+R) div 2;
    Dir := FOnPtrCompare(Items[I], AKey);
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
=======
procedure TFPGMap.InitOnPtrCompare;
=======
procedure TFPGMap.InitOnPtrCompare;
begin
  OnPtrCompare := @KeyCompare;
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

function TFPGMap.KeyCustomCompare(Key1, Key2: Pointer): Integer;
>>>>>>> origin/fixes_2_2
begin
  OnPtrCompare := @KeyCompare;
end;

>>>>>>> graemeg/fixes_2_2
function TFPGMap.KeyCompare(Key1, Key2: Pointer): Integer;
begin
  if PKey(Key1)^ < PKey(Key2)^ then
    Result := -1
  else if PKey(Key1)^ > PKey(Key2)^ then
    Result := 1
  else
    Result := 0;
end;

<<<<<<< HEAD
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
=======
function TFPGMap.KeyCustomCompare(Key1, Key2: Pointer): Integer;
>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
=======
procedure TFPGMap.SetOnCompare(NewCompare: TCompareFunc);
begin
  FOnCompare := NewCompare;
  if NewCompare <> nil then
    OnPtrCompare := @KeyCustomCompare
  else
    InitOnPtrCompare;
end;

>>>>>>> graemeg/fixes_2_2
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
