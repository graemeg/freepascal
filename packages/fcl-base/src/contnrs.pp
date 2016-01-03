{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2002 by Florian Klaempfl

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$ifdef fpc}
{$mode objfpc}
{$endif}
{$H+}
{$ifdef CLASSESINLINE}{$inline on}{$endif}

unit contnrs;

interface

uses
  SysUtils,Classes;


Type
<<<<<<< HEAD
<<<<<<< HEAD
  TObjectListCallback = Procedure(data:TObject;arg:pointer) of object;
  TObjectListStaticCallback = Procedure(data:TObject;arg:pointer);
=======
  TObjectListCallback = procedure(data:TObject;arg:pointer) of object;
  TObjectListStaticCallback = procedure(data:TObject;arg:pointer);
>>>>>>> graemeg/fixes_2_2
=======
  TObjectListCallback = procedure(data:TObject;arg:pointer) of object;
  TObjectListStaticCallback = procedure(data:TObject;arg:pointer);
>>>>>>> origin/fixes_2_2

  TFPObjectList = class(TObject)
  private
    FFreeObjects : Boolean;
    FList: TFPList;
<<<<<<< HEAD
<<<<<<< HEAD
    Function GetCount: integer;
    Procedure SetCount(const AValue: integer);
  protected
    Function GetItem(Index: Integer): TObject; {$ifdef CLASSESINLINE}inline;{$endif}
    Procedure SetItem(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
    Procedure SetCapacity(NewCapacity: Integer);
    Function GetCapacity: integer;
=======
=======
>>>>>>> origin/fixes_2_2
    function GetCount: integer;
    procedure SetCount(const AValue: integer);
  protected
    function GetItem(Index: Integer): TObject; {$ifdef CLASSESINLINE}inline;{$endif}
    procedure SetItem(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
    procedure SetCapacity(NewCapacity: Integer);
    function GetCapacity: integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  public
    constructor Create;
    constructor Create(FreeObjects : Boolean);
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Clear;
    Function Add(AObject: TObject): Integer; {$ifdef CLASSESINLINE}inline;{$endif}
    Procedure Delete(Index: Integer); {$ifdef CLASSESINLINE}inline;{$endif}
    Procedure Exchange(Index1, Index2: Integer);
    Function Expand: TFPObjectList;
    Function Extract(Item: TObject): TObject;
    Function Remove(AObject: TObject): Integer;
    Function IndexOf(AObject: TObject): Integer;
    Function FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt: Integer): Integer;
    Procedure Insert(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
    Function First: TObject;
    Function Last: TObject;
    Procedure Move(CurIndex, NewIndex: Integer);
    Procedure Assign(Obj:TFPObjectList);
    Procedure Pack;
    Procedure Sort(Compare: TListSortCompare);
    Procedure ForEachCall(proc2call:TObjectListCallback;arg:pointer);
    Procedure ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
=======
=======
>>>>>>> origin/fixes_2_2
    procedure Clear;
    function Add(AObject: TObject): Integer; {$ifdef CLASSESINLINE}inline;{$endif}
    procedure Delete(Index: Integer); {$ifdef CLASSESINLINE}inline;{$endif}
    procedure Exchange(Index1, Index2: Integer);
    function Expand: TFPObjectList;
    function Extract(Item: TObject): TObject;
    function Remove(AObject: TObject): Integer;
    function IndexOf(AObject: TObject): Integer;
    function FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt: Integer): Integer;
    procedure Insert(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
    function First: TObject;
    function Last: TObject;
    procedure Move(CurIndex, NewIndex: Integer);
    procedure Assign(Obj:TFPObjectList);
    procedure Pack;
    procedure Sort(Compare: TListSortCompare);
    procedure ForEachCall(proc2call:TObjectListCallback;arg:pointer);
    procedure ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Capacity: Integer read GetCapacity write SetCapacity;
    property Count: Integer read GetCount write SetCount;
    property OwnsObjects: Boolean read FFreeObjects write FFreeObjects;
    property Items[Index: Integer]: TObject read GetItem write SetItem; default;
    property List: TFPList read FList;
  end;


<<<<<<< HEAD
<<<<<<< HEAD
  { TObjectList }

  TObjectList = class(TList)
  private
    FFreeObjects : Boolean;
  Protected
    Procedure Notify(Ptr: Pointer; Action: TListNotification); override;
    Function GetItem(Index: Integer): TObject;
    Procedure SetItem(Index: Integer; AObject: TObject);
  public
    constructor Create;
    constructor Create(FreeObjects : boolean);
    Function Add(AObject: TObject): Integer;
    Function Extract(Item: TObject): TObject;
    Function Remove(AObject: TObject): Integer;
    Function IndexOf(AObject: TObject): Integer;
    Function FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt: Integer): Integer;
    Procedure Insert(Index: Integer; AObject: TObject);
    Function First: TObject;
=======
=======
>>>>>>> origin/fixes_2_2
  TObjectList = class(TList)
  private
    ffreeobjects : boolean;
  Protected
    Procedure Notify(Ptr: Pointer; Action: TListNotification); override;
    function GetItem(Index: Integer): TObject;
    Procedure SetItem(Index: Integer; AObject: TObject);
  public
    constructor create;
    constructor create(freeobjects : boolean);
    function Add(AObject: TObject): Integer;
    function Extract(Item: TObject): TObject;
    function Remove(AObject: TObject): Integer;
    function IndexOf(AObject: TObject): Integer;
    function FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt: Integer): Integer;
    Procedure Insert(Index: Integer; AObject: TObject);
    function First: TObject;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function Last: TObject;
    property OwnsObjects: Boolean read FFreeObjects write FFreeObjects;
    property Items[Index: Integer]: TObject read GetItem write SetItem; default;
  end;

  TComponentList = class(TObjectList)
  Private
    FNotifier : TComponent;
  Protected
    Procedure Notify(Ptr: Pointer; Action: TListNotification); override;
    Function GetItems(Index: Integer): TComponent;
    Procedure SetItems(Index: Integer; AComponent: TComponent);
    Procedure HandleFreeNotify(Sender: TObject; AComponent: TComponent);
  public
    destructor Destroy; override;
    Function Add(AComponent: TComponent): Integer;
    Function Extract(Item: TComponent): TComponent;
    Function Remove(AComponent: TComponent): Integer;
    Function IndexOf(AComponent: TComponent): Integer;
    Function First: TComponent;
    Function Last: TComponent;
    Procedure Insert(Index: Integer; AComponent: TComponent);
    property Items[Index: Integer]: TComponent read GetItems write SetItems; default;
  end;

  TClassList = class(TList)
  protected
    Function GetItems(Index: Integer): TClass;
    Procedure SetItems(Index: Integer; AClass: TClass);
  public
    Function Add(AClass: TClass): Integer;
    Function Extract(Item: TClass): TClass;
    Function Remove(AClass: TClass): Integer;
    Function IndexOf(AClass: TClass): Integer;
    Function First: TClass;
    Function Last: TClass;
    Procedure Insert(Index: Integer; AClass: TClass);
    property Items[Index: Integer]: TClass read GetItems write SetItems; default;
  end;

  TOrderedList = class(TObject)
  private
    FList: TList;
  protected
    Procedure PushItem(AItem: Pointer); virtual; abstract;
    Function PopItem: Pointer; virtual;
    Function PeekItem: Pointer; virtual;
    property List: TList read FList;
  public
    constructor Create;
    destructor Destroy; override;
    Function Count: Integer;
    Function AtLeast(ACount: Integer): Boolean;
    Function Push(AItem: Pointer): Pointer;
    Function Pop: Pointer;
    Function Peek: Pointer;
  end;

{ TStack class }

  TStack = class(TOrderedList)
  protected
    Procedure PushItem(AItem: Pointer); override;
  end;

{ TObjectStack class }

  TObjectStack = class(TStack)
  public
    Function Push(AObject: TObject): TObject;
    Function Pop: TObject;
    Function Peek: TObject;
  end;

{ TQueue class }

  TQueue = class(TOrderedList)
  protected
    Procedure PushItem(AItem: Pointer); override;
  end;

{ TObjectQueue class }

  TObjectQueue = class(TQueue)
  public
    Function Push(AObject: TObject): TObject;
    Function Pop: TObject;
    Function Peek: TObject;
  end;

{ ---------------------------------------------------------------------
    TFPList with Hash support
  ---------------------------------------------------------------------}

type
  THashItem=record
    HashValue : LongWord;
    StrIndex  : Integer;
    NextIndex : Integer;
    Data      : Pointer;
  end;
  PHashItem=^THashItem;

const
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef CPU16}
  MaxHashListSize = maxsmallint div 16;
  MaxHashStrSize  = maxsmallint;
  MaxHashTableSize = maxsmallint div 4;
{$else CPU16}
  MaxHashListSize = Maxint div 16;
  MaxHashStrSize  = Maxint;
  MaxHashTableSize = Maxint div 4;
{$endif CPU16}
=======
  MaxHashListSize = Maxint div 16;
  MaxHashStrSize  = Maxint;
  MaxHashTableSize = Maxint div 4;
>>>>>>> graemeg/fixes_2_2
=======
  MaxHashListSize = Maxint div 16;
  MaxHashStrSize  = Maxint;
  MaxHashTableSize = Maxint div 4;
>>>>>>> origin/fixes_2_2
  MaxItemsPerHash = 3;

type
  PHashItemList = ^THashItemList;
  THashItemList = array[0..MaxHashListSize - 1] of THashItem;
  PHashTable = ^THashTable;
  THashTable = array[0..MaxHashTableSize - 1] of Integer;

  TFPHashList = class(TObject)
  private
    { ItemList }
    FHashList     : PHashItemList;
    FCount,
    FCapacity : Integer;
    { Hash }
    FHashTable    : PHashTable;
    FHashCapacity : Integer;
    { Strings }
    FStrs     : PChar;
    FStrCount,
    FStrCapacity : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    Function InternalFind(AHash:LongWord;const AName:shortstring;out PrevIndex:Integer):Integer;
  protected
    Function Get(Index: Integer): Pointer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure Put(Index: Integer; Item: Pointer); {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure SetCapacity(NewCapacity: Integer);
    Procedure SetCount(NewCount: Integer);
    Procedure RaiseIndexError(Index : Integer);
    Function  AddStr(const s:shortstring): Integer;
    Procedure AddToHashTable(Index: Integer);
    Procedure StrExpand(MinIncSize:Integer);
    Procedure SetStrCapacity(NewCapacity: Integer);
    Procedure SetHashCapacity(NewCapacity: Integer);
    Procedure ReHash;
  public
    constructor Create;
    destructor Destroy; override;
    Function Add(const AName:shortstring;Item: Pointer): Integer;
    Procedure Clear;
    Function NameOfIndex(Index: Integer): ShortString; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function HashOfIndex(Index: Integer): LongWord; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function GetNextCollision(Index: Integer): Integer;
    Procedure Delete(Index: Integer);
    class Procedure Error(const Msg: string; Data: PtrInt);
    Function Expand: TFPHashList;
    Function Extract(item: Pointer): Pointer;
    Function IndexOf(Item: Pointer): Integer;
    Function Find(const AName:shortstring): Pointer;
    Function FindIndexOf(const AName:shortstring): Integer;
    Function FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
    Function Rename(const AOldName,ANewName:shortstring): Integer;
    Function Remove(Item: Pointer): Integer;
    Procedure Pack;
    Procedure ShowStatistics;
    Procedure ForEachCall(proc2call:TListCallback;arg:pointer);
    Procedure ForEachCall(proc2call:TListStaticCallback;arg:pointer);
=======
=======
>>>>>>> origin/fixes_2_2
    function InternalFind(AHash:LongWord;const AName:shortstring;out PrevIndex:Integer):Integer;
  protected
    function Get(Index: Integer): Pointer; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure Put(Index: Integer; Item: Pointer); {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure SetCapacity(NewCapacity: Integer);
    procedure SetCount(NewCount: Integer);
    Procedure RaiseIndexError(Index : Integer);
    function  AddStr(const s:shortstring): Integer;
    procedure AddToHashTable(Index: Integer);
    procedure StrExpand(MinIncSize:Integer);
    procedure SetStrCapacity(NewCapacity: Integer);
    procedure SetHashCapacity(NewCapacity: Integer);
    procedure ReHash;
  public
    constructor Create;
    destructor Destroy; override;
    function Add(const AName:shortstring;Item: Pointer): Integer;
    procedure Clear;
    function NameOfIndex(Index: Integer): ShortString; {$ifdef CCLASSESINLINE}inline;{$endif}
    function HashOfIndex(Index: Integer): LongWord; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure Delete(Index: Integer);
    class procedure Error(const Msg: string; Data: PtrInt);
    function Expand: TFPHashList;
    function Extract(item: Pointer): Pointer;
    function IndexOf(Item: Pointer): Integer;
    function Find(const AName:shortstring): Pointer;
    function FindIndexOf(const AName:shortstring): Integer;
    function FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
    function Rename(const AOldName,ANewName:shortstring): Integer;
    function Remove(Item: Pointer): Integer;
    procedure Pack;
    procedure ShowStatistics;
    procedure ForEachCall(proc2call:TListCallback;arg:pointer);
    procedure ForEachCall(proc2call:TListStaticCallback;arg:pointer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Capacity: Integer read FCapacity write SetCapacity;
    property Count: Integer read FCount write SetCount;
    property Items[Index: Integer]: Pointer read Get write Put; default;
    property List: PHashItemList read FHashList;
    property Strs: PChar read FStrs;
  end;


{*******************************************************
        TFPHashObjectList (From fcl/inc/contnrs.pp)
********************************************************}

  TFPHashObjectList = class;

  { TFPHashObject }

  TFPHashObject = class
  private
    FOwner     : TFPHashObjectList;
    FCachedStr : pshortstring;
    FStrIndex  : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure InternalChangeOwner(HashObjectList:TFPHashObjectList;const s:shortstring);
  protected
    Function GetName:shortstring;virtual;
    Function GetHash:Longword;virtual;
  public
    constructor CreateNotOwned;
    constructor Create(HashObjectList:TFPHashObjectList;const s:shortstring);
    Procedure ChangeOwner(HashObjectList:TFPHashObjectList); {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure ChangeOwnerAndName(HashObjectList:TFPHashObjectList;const s:shortstring); {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure Rename(const ANewName:shortstring);
=======
=======
>>>>>>> origin/fixes_2_2
    procedure InternalChangeOwner(HashObjectList:TFPHashObjectList;const s:shortstring);
  protected
    function GetName:shortstring;virtual;
    function GetHash:Longword;virtual;
  public
    constructor CreateNotOwned;
    constructor Create(HashObjectList:TFPHashObjectList;const s:shortstring);
    procedure ChangeOwner(HashObjectList:TFPHashObjectList); {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure ChangeOwnerAndName(HashObjectList:TFPHashObjectList;const s:shortstring); {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure Rename(const ANewName:shortstring);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Name:shortstring read GetName;
    property Hash:Longword read GetHash;
  end;

  TFPHashObjectList = class(TObject)
  private
    FFreeObjects : Boolean;
    FHashList: TFPHashList;
<<<<<<< HEAD
<<<<<<< HEAD
    Function GetCount: integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure SetCount(const AValue: integer); {$ifdef CCLASSESINLINE}inline;{$endif}
  protected
    Function GetItem(Index: Integer): TObject; {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure SetItem(Index: Integer; AObject: TObject); {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure SetCapacity(NewCapacity: Integer); {$ifdef CCLASSESINLINE}inline;{$endif}
    Function GetCapacity: integer; {$ifdef CCLASSESINLINE}inline;{$endif}
  public
    constructor Create(FreeObjects : boolean = True);
    destructor Destroy; override;
    Procedure Clear;
    Function Add(const AName:shortstring;AObject: TObject): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function NameOfIndex(Index: Integer): ShortString; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function HashOfIndex(Index: Integer): LongWord; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function GetNextCollision(Index: Integer): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure Delete(Index: Integer);
    Function Expand: TFPHashObjectList; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function Extract(Item: TObject): TObject; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function Remove(AObject: TObject): Integer;
    Function IndexOf(AObject: TObject): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function Find(const s:shortstring): TObject; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function FindIndexOf(const s:shortstring): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
    Function Rename(const AOldName,ANewName:shortstring): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    Function FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt: Integer): Integer;
    Procedure Pack; {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure ShowStatistics; {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure ForEachCall(proc2call:TObjectListCallback;arg:pointer); {$ifdef CCLASSESINLINE}inline;{$endif}
    Procedure ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer); {$ifdef CCLASSESINLINE}inline;{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
    function GetCount: integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure SetCount(const AValue: integer); {$ifdef CCLASSESINLINE}inline;{$endif}
  protected
    function GetItem(Index: Integer): TObject; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure SetItem(Index: Integer; AObject: TObject); {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure SetCapacity(NewCapacity: Integer); {$ifdef CCLASSESINLINE}inline;{$endif}
    function GetCapacity: integer; {$ifdef CCLASSESINLINE}inline;{$endif}
  public
    constructor Create(FreeObjects : boolean = True);
    destructor Destroy; override;
    procedure Clear;
    function Add(const AName:shortstring;AObject: TObject): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    function NameOfIndex(Index: Integer): ShortString; {$ifdef CCLASSESINLINE}inline;{$endif}
    function HashOfIndex(Index: Integer): LongWord; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure Delete(Index: Integer);
    function Expand: TFPHashObjectList; {$ifdef CCLASSESINLINE}inline;{$endif}
    function Extract(Item: TObject): TObject; {$ifdef CCLASSESINLINE}inline;{$endif}
    function Remove(AObject: TObject): Integer;
    function IndexOf(AObject: TObject): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    function Find(const s:shortstring): TObject; {$ifdef CCLASSESINLINE}inline;{$endif}
    function FindIndexOf(const s:shortstring): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    function FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
    function Rename(const AOldName,ANewName:shortstring): Integer; {$ifdef CCLASSESINLINE}inline;{$endif}
    function FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt: Integer): Integer;
    procedure Pack; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure ShowStatistics; {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure ForEachCall(proc2call:TObjectListCallback;arg:pointer); {$ifdef CCLASSESINLINE}inline;{$endif}
    procedure ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer); {$ifdef CCLASSESINLINE}inline;{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Capacity: Integer read GetCapacity write SetCapacity;
    property Count: Integer read GetCount write SetCount;
    property OwnsObjects: Boolean read FFreeObjects write FFreeObjects;
    property Items[Index: Integer]: TObject read GetItem write SetItem; default;
    property List: TFPHashList read FHashList;
  end;

{ ---------------------------------------------------------------------
    Hash support, implemented by Dean Zobec
  ---------------------------------------------------------------------}


  { Must return a Longword value in the range 0..TableSize,
   usually via a mod operator;  }
<<<<<<< HEAD
<<<<<<< HEAD
  THashFunction = Function(const S: string; const TableSize: Longword): Longword;
=======
  THashFunction = function(const S: string; const TableSize: Longword): Longword;
>>>>>>> graemeg/fixes_2_2
=======
  THashFunction = function(const S: string; const TableSize: Longword): Longword;
>>>>>>> origin/fixes_2_2


  { THTNode }

  THTCustomNode = class(TObject)
  private
    FKey: string;
  public
    constructor CreateWith(const AString: String);
<<<<<<< HEAD
<<<<<<< HEAD
    Function HasKey(const AKey: string): boolean;
=======
    function HasKey(const AKey: string): boolean;
>>>>>>> graemeg/fixes_2_2
=======
    function HasKey(const AKey: string): boolean;
>>>>>>> origin/fixes_2_2
    property Key: string read FKey;
  end;
  THTCustomNodeClass = Class of THTCustomNode;


  { TFPCustomHashTable }

  TFPCustomHashTable = class(TObject)
  private
    FHashTable: TFPObjectList;
<<<<<<< HEAD
<<<<<<< HEAD
    FHashFunction: THashFunction;
    FCount: Longword;
<<<<<<< HEAD
    Function GetDensity: Longword;
    Function GetNumberOfCollisions: Longword;
    Procedure SetHashTableSize(const Value: Longword);
    Procedure InitializeHashTable;
    Function GetVoidSlots: Longword;
    Function GetLoadFactor: double;
    Function GetAVGChainLen: double;
    Function GetMaxChainLength: Longword;
  protected
    FHashTableSize: Longword;
    Function Chain(const index: Longword):TFPObjectList;
=======
=======
    FHashTableSize: Longword;
    FHashFunction: THashFunction;
    FCount: Longword;
>>>>>>> graemeg/fixes_2_2
=======
    FHashTableSize: Longword;
    FHashFunction: THashFunction;
    FCount: Longword;
>>>>>>> origin/fixes_2_2
    function GetDensity: Longword;
    function GetNumberOfCollisions: Longword;
    procedure SetHashTableSize(const Value: Longword);
    procedure InitializeHashTable;
    function GetVoidSlots: Longword;
    function GetLoadFactor: double;
    function GetAVGChainLen: double;
    function GetMaxChainLength: Longword;
<<<<<<< HEAD
<<<<<<< HEAD
  protected
    function Chain(const index: Longword):TFPObjectList;
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
    Function CreateNewNode(const aKey : string) : THTCustomNode; virtual; abstract;
    Procedure AddNode(ANode : THTCustomNode); virtual; abstract;
    Function ChainLength(const ChainIndex: Longword): Longword; virtual;
    Function FindOrCreateNew(const aKey: string): THTCustomNode; virtual;
    Procedure SetHashFunction(AHashFunction: THashFunction); virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    function Chain(const index: Longword):TFPObjectList;
  protected
    Function CreateNewNode(const aKey : string) : THTCustomNode; virtual; abstract;
    Procedure AddNode(ANode : THTCustomNode); virtual; abstract;
    function ChainLength(const ChainIndex: Longword): Longword; virtual;
    function FindOrCreateNew(const aKey: string): THTCustomNode; virtual;
    procedure SetHashFunction(AHashFunction: THashFunction); virtual;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function FindChainForAdd(Const aKey : String) : TFPObjectList;
  public
    constructor Create;
    constructor CreateWith(AHashTableSize: Longword; aHashFunc: THashFunction);
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure ChangeTableSize(const ANewSize: Longword); virtual;
    Procedure Clear; virtual;
    Procedure Delete(const aKey: string); virtual;
    Function Find(const aKey: string): THTCustomNode;
    Function IsEmpty: boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure ChangeTableSize(const ANewSize: Longword); virtual;
    procedure Clear; virtual;
    procedure Delete(const aKey: string); virtual;
    function Find(const aKey: string): THTCustomNode;
    function IsEmpty: boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property HashFunction: THashFunction read FHashFunction write SetHashFunction;
    property Count: Longword read FCount;
    property HashTableSize: Longword read FHashTableSize write SetHashTableSize;
    property HashTable: TFPObjectList read FHashTable;
    property VoidSlots: Longword read GetVoidSlots;
    property LoadFactor: double read GetLoadFactor;
    property AVGChainLen: double read GetAVGChainLen;
    property MaxChainLength: Longword read GetMaxChainLength;
    property NumberOfCollisions: Longword read GetNumberOfCollisions;
    property Density: Longword read GetDensity;
  end;

  { TFPDataHashTable : Hash table with simple data pointers }

  THTDataNode = Class(THTCustomNode)
  Private
    FData: pointer;
  public
    property Data: pointer read FData write FData;
  end;
  // For compatibility
  THTNode = THTDataNode;

<<<<<<< HEAD
<<<<<<< HEAD
  TDataIteratorMethod = Procedure(Item: Pointer; const Key: string; var Continue: Boolean) of object;
=======
  TDataIteratorMethod = procedure(Item: Pointer; const Key: string; var Continue: Boolean) of object;
>>>>>>> graemeg/fixes_2_2
=======
  TDataIteratorMethod = procedure(Item: Pointer; const Key: string; var Continue: Boolean) of object;
>>>>>>> origin/fixes_2_2
  // For compatibility
  TIteratorMethod = TDataIteratorMethod;

  TFPDataHashTable = Class(TFPCustomHashTable)
  Protected
    Function CreateNewNode(const aKey : String) : THTCustomNode; override;
    Procedure AddNode(ANode : THTCustomNode); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure SetData(const index: string; const AValue: Pointer); virtual;
    Function GetData(const index: string):Pointer; virtual;
    Function ForEachCall(aMethod: TDataIteratorMethod): THTDataNode; virtual;
  Public
    Function Iterate(aMethod: TDataIteratorMethod): Pointer; virtual;
    Procedure Add(const aKey: string; AItem: pointer); virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure SetData(const index: string; const AValue: Pointer); virtual;
    function GetData(const index: string):Pointer; virtual;
    function ForEachCall(aMethod: TDataIteratorMethod): THTDataNode; virtual;
  Public
    procedure Add(const aKey: string; AItem: pointer); virtual;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Items[const index: string]: Pointer read GetData write SetData; default;
  end;

  { TFPStringHashTable : Hash table with simple strings as data }
  THTStringNode = Class(THTCustomNode)
  Private
    FData : String;
  public
    property Data: String read FData write FData;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  TStringIteratorMethod = Procedure(Item: String; const Key: string; var Continue: Boolean) of object;
=======
  TStringIteratorMethod = procedure(Item: String; const Key: string; var Continue: Boolean) of object;
>>>>>>> graemeg/fixes_2_2
=======
  TStringIteratorMethod = procedure(Item: String; const Key: string; var Continue: Boolean) of object;
>>>>>>> origin/fixes_2_2

  TFPStringHashTable = Class(TFPCustomHashTable)
  Protected
    Function CreateNewNode(const aKey : String) : THTCustomNode; override;
    Procedure AddNode(ANode : THTCustomNode); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure SetData(const Index, AValue: string); virtual;
    Function GetData(const index: string): String; virtual;
    Function ForEachCall(aMethod: TStringIteratorMethod): THTStringNode; virtual;
  Public
    Function Iterate(aMethod: TStringIteratorMethod): String; virtual;
    Procedure Add(const aKey,aItem: string); virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure SetData(const Index, AValue: string); virtual;
    function GetData(const index: string): String; virtual;
    function ForEachCall(aMethod: TStringIteratorMethod): THTStringNode; virtual;
  Public
    procedure Add(const aKey,aItem: string); virtual;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Items[const index: string]: String read GetData write SetData; default;
  end;

  { TFPStringHashTable : Hash table with simple strings as data }


  THTObjectNode = Class(THTCustomNode)
  Private
    FData : TObject;
  public
    property Data: TObject read FData write FData;
  end;

  THTOwnedObjectNode = Class(THTObjectNode)
  public
<<<<<<< HEAD
<<<<<<< HEAD
    destructor Destroy; override;
  end;
  TObjectIteratorMethod = Procedure(Item: TObject; const Key: string; var Continue: Boolean) of object;
=======
    Destructor Destroy; override;
  end;
  TObjectIteratorMethod = procedure(Item: TObject; const Key: string; var Continue: Boolean) of object;
>>>>>>> graemeg/fixes_2_2
=======
    Destructor Destroy; override;
  end;
  TObjectIteratorMethod = procedure(Item: TObject; const Key: string; var Continue: Boolean) of object;
>>>>>>> origin/fixes_2_2

  TFPObjectHashTable = Class(TFPCustomHashTable)
  Private
    FOwnsObjects : Boolean;
  Protected
    Function CreateNewNode(const aKey : String) : THTCustomNode; override;
    Procedure AddNode(ANode : THTCustomNode); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure SetData(const Index: string; AObject : TObject); virtual;
    Function GetData(const index: string): TObject; virtual;
    Function ForEachCall(aMethod: TObjectIteratorMethod): THTObjectNode; virtual;
  Public
    constructor Create(AOwnsObjects : Boolean = True);
    constructor CreateWith(AHashTableSize: Longword; aHashFunc: THashFunction; AOwnsObjects : Boolean = True);
    Function Iterate(aMethod: TObjectIteratorMethod): TObject; virtual;
    Procedure Add(const aKey: string; AItem : TObject); virtual;
    property Items[const index: string]: TObject read GetData write SetData; default;
    Property OwnsObjects : Boolean Read FOwnsObjects;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure SetData(const Index: string; AObject : TObject); virtual;
    function GetData(const index: string): TObject; virtual;
    function ForEachCall(aMethod: TObjectIteratorMethod): THTObjectNode; virtual;
  Public
    constructor Create(AOwnsObjects : Boolean = True);
    constructor CreateWith(AHashTableSize: Longword; aHashFunc: THashFunction; AOwnsObjects : Boolean = True);
    procedure Add(const aKey: string; AItem : TObject); virtual;
    property Items[const index: string]: TObject read GetData write SetData; default;
    Property OwnsObjects : Boolean Read FOwnsObjects Write FOwnsObjects;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  EDuplicate = class(Exception);
  EKeyNotFound = class(Exception);

<<<<<<< HEAD
<<<<<<< HEAD
  Function RSHash(const S: string; const TableSize: Longword): Longword;
=======
  function RSHash(const S: string; const TableSize: Longword): Longword;
>>>>>>> graemeg/fixes_2_2
=======
  function RSHash(const S: string; const TableSize: Longword): Longword;
>>>>>>> origin/fixes_2_2

{ ---------------------------------------------------------------------
    Bucket lists as in Delphi
  ---------------------------------------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2

Type
  TBucketItem = record
    Item, Data: Pointer;
  end;
  TBucketItemArray = array of TBucketItem;

  TBucket = record
    Count : Integer;
    Items : TBucketItemArray;
  end;
  PBucket = ^TBucket;
  TBucketArray = array of TBucket;

<<<<<<< HEAD
<<<<<<< HEAD
  TBucketProc = Procedure(AInfo, AItem, AData: Pointer; out AContinue: Boolean);
  TBucketProcObject = Procedure(AItem, AData: Pointer; out AContinue: Boolean) of Object;
=======
  TBucketProc = procedure(AInfo, AItem, AData: Pointer; out AContinue: Boolean);
  TBucketProcObject = procedure(AItem, AData: Pointer; out AContinue: Boolean) of Object;
>>>>>>> graemeg/fixes_2_2
=======
  TBucketProc = procedure(AInfo, AItem, AData: Pointer; out AContinue: Boolean);
  TBucketProcObject = procedure(AItem, AData: Pointer; out AContinue: Boolean) of Object;
>>>>>>> origin/fixes_2_2

{ ---------------------------------------------------------------------
  TCustomBucketList
  ---------------------------------------------------------------------}

  { TCustomBucketList }

  TCustomBucketList = class(TObject)
  private
    FBuckets: TBucketArray;
<<<<<<< HEAD
<<<<<<< HEAD
    Function GetBucketCount: Integer;
    Function GetData(AItem: Pointer): Pointer;
    Procedure SetData(AItem: Pointer; const AData: Pointer);
    Procedure SetBucketCount(const Value: Integer);
  protected
    Procedure GetBucketItem(AItem: Pointer; out ABucket, AIndex: Integer);
    Function AddItem(ABucket: Integer; AItem, AData: Pointer): Pointer; virtual;
    Function BucketFor(AItem: Pointer): Integer; virtual; abstract;
    Function DeleteItem(ABucket: Integer; AIndex: Integer): Pointer; virtual;
    Procedure Error(Msg : String; Args : Array of Const);
    Function FindItem(AItem: Pointer; out ABucket, AIndex: Integer): Boolean; virtual;
=======
=======
>>>>>>> origin/fixes_2_2
    function GetBucketCount: Integer;
    function GetData(AItem: Pointer): Pointer;
    procedure SetData(AItem: Pointer; const AData: Pointer);
    procedure SetBucketCount(const Value: Integer);
  protected
    Procedure GetBucketItem(AItem: Pointer; out ABucket, AIndex: Integer);
    function AddItem(ABucket: Integer; AItem, AData: Pointer): Pointer; virtual;
    function BucketFor(AItem: Pointer): Integer; virtual; abstract;
    function DeleteItem(ABucket: Integer; AIndex: Integer): Pointer; virtual;
    Procedure Error(Msg : String; Args : Array of Const);
    function FindItem(AItem: Pointer; out ABucket, AIndex: Integer): Boolean; virtual;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Buckets: TBucketArray read FBuckets;
    property BucketCount: Integer read GetBucketCount write SetBucketCount;
  public
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure Clear;
    Function Add(AItem, AData: Pointer): Pointer;
    Procedure Assign(AList: TCustomBucketList);
    Function Exists(AItem: Pointer): Boolean;
    Function Find(AItem: Pointer; out AData: Pointer): Boolean;
    Function ForEach(AProc: TBucketProc; AInfo: Pointer = nil): Boolean;
    Function ForEach(AProc: TBucketProcObject): Boolean;
    Function Remove(AItem: Pointer): Pointer;
=======
=======
>>>>>>> origin/fixes_2_2
    procedure Clear;
    function Add(AItem, AData: Pointer): Pointer;
    procedure Assign(AList: TCustomBucketList);
    function Exists(AItem: Pointer): Boolean;
    function Find(AItem: Pointer; out AData: Pointer): Boolean;
    function ForEach(AProc: TBucketProc; AInfo: Pointer = nil): Boolean;
    function ForEach(AProc: TBucketProcObject): Boolean;
    function Remove(AItem: Pointer): Pointer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Data[AItem: Pointer]: Pointer read GetData write SetData; default;
  end;

{ ---------------------------------------------------------------------
  TBucketList
  ---------------------------------------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2

  TBucketListSizes = (bl2, bl4, bl8, bl16, bl32, bl64, bl128, bl256);

  { TBucketList }

  TBucketList = class(TCustomBucketList)
  private
    FBucketMask: Byte;
  protected
<<<<<<< HEAD
<<<<<<< HEAD
    Function BucketFor(AItem: Pointer): Integer; override;
=======
    function BucketFor(AItem: Pointer): Integer; override;
>>>>>>> graemeg/fixes_2_2
=======
    function BucketFor(AItem: Pointer): Integer; override;
>>>>>>> origin/fixes_2_2
  public
    constructor Create(ABuckets: TBucketListSizes = bl16);
  end;

{ ---------------------------------------------------------------------
  TObjectBucketList
  ---------------------------------------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  { TObjectBucketList }

  TObjectBucketList = class(TBucketList)
  protected
<<<<<<< HEAD
<<<<<<< HEAD
    Function GetData(AItem: TObject): TObject;
    Procedure SetData(AItem: TObject; const AData: TObject);
  public
    Function Add(AItem, AData: TObject): TObject;
    Function Remove(AItem: TObject): TObject;
=======
=======
>>>>>>> origin/fixes_2_2
    function GetData(AItem: TObject): TObject;
    procedure SetData(AItem: TObject; const AData: TObject);
  public
    function Add(AItem, AData: TObject): TObject;
    function Remove(AItem: TObject): TObject;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Data[AItem: TObject]: TObject read GetData write SetData; default;
  end;


implementation

uses
  RtlConsts;

ResourceString
  DuplicateMsg   = 'An item with key %0:s already exists';
  KeyNotFoundMsg = 'Method: %0:s key [''%1:s''] not found in container';
  NotEmptyMsg    = 'Hash table not empty.';
  SErrNoSuchItem = 'No item in list for %p';
  SDuplicateItem = 'Item already exists in list: %p';

const
  NPRIMES = 28;

  PRIMELIST: array[0 .. NPRIMES-1] of Longword =
  ( 53,         97,         193,       389,       769,
    1543,       3079,       6151,      12289,     24593,
    49157,      98317,      196613,    393241,    786433,
    1572869,    3145739,    6291469,   12582917,  25165843,
    50331653,   100663319,  201326611, 402653189, 805306457,
    1610612741, 3221225473, 4294967291 );

constructor TFPObjectList.Create(FreeObjects : boolean);
begin
  Create;
<<<<<<< HEAD
<<<<<<< HEAD
  FFreeObjects:=Freeobjects;
=======
  FFreeObjects := Freeobjects;
>>>>>>> graemeg/fixes_2_2
=======
  FFreeObjects := Freeobjects;
>>>>>>> origin/fixes_2_2
end;

destructor TFPObjectList.Destroy;
begin
  if (FList <> nil) then
  begin
    Clear;
    FList.Destroy;
  end;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.Clear;
=======
procedure TFPObjectList.Clear;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.Clear;
>>>>>>> origin/fixes_2_2
var
  i: integer;
begin
  if FFreeObjects then
<<<<<<< HEAD
<<<<<<< HEAD
    for i:=FList.Count-1 downto 0  do
=======
    for i := 0 to FList.Count - 1 do
>>>>>>> graemeg/fixes_2_2
=======
    for i := 0 to FList.Count - 1 do
>>>>>>> origin/fixes_2_2
      TObject(FList[i]).Free;
  FList.Clear;
end;

constructor TFPObjectList.Create;
begin
  inherited Create;
<<<<<<< HEAD
<<<<<<< HEAD
  FList:=TFPList.Create;
  FFreeObjects:=True;
end;

Function TFPObjectList.GetCount: integer;
begin
  Result:=FList.Count;
end;

Procedure TFPObjectList.SetCount(const AValue: integer);
begin
  if FList.Count <> AValue then
    FList.Count:=AValue;
end;

Function TFPObjectList.GetItem(Index: Integer): TObject; {$ifdef CLASSESINLINE}inline;{$endif}
begin
  Result:=TObject(FList[Index]);
end;

Procedure TFPObjectList.SetItem(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
begin
  if OwnsObjects then
    TObject(FList[Index]).Free;
  FList[index]:=AObject;
end;

Procedure TFPObjectList.SetCapacity(NewCapacity: Integer);
begin
  FList.Capacity:=NewCapacity;
end;

Function TFPObjectList.GetCapacity: integer;
begin
  Result:=FList.Capacity;
end;

Function TFPObjectList.Add(AObject: TObject): Integer; {$ifdef CLASSESINLINE}inline;{$endif}
begin
  Result:=FList.Add(AObject);
end;

Procedure TFPObjectList.Delete(Index: Integer); {$ifdef CLASSESINLINE}inline;{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
  FList := TFPList.Create;
  FFreeObjects := True;
end;

function TFPObjectList.GetCount: integer;
begin
  Result := FList.Count;
end;

procedure TFPObjectList.SetCount(const AValue: integer);
begin
  if FList.Count <> AValue then
    FList.Count := AValue;
end;

function TFPObjectList.GetItem(Index: Integer): TObject; {$ifdef CLASSESINLINE}inline;{$endif}
begin
  Result := TObject(FList[Index]);
end;

procedure TFPObjectList.SetItem(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
begin
  if OwnsObjects then
    TObject(FList[Index]).Free;
  FList[index] := AObject;
end;

procedure TFPObjectList.SetCapacity(NewCapacity: Integer);
begin
  FList.Capacity := NewCapacity;
end;

function TFPObjectList.GetCapacity: integer;
begin
  Result := FList.Capacity;
end;

function TFPObjectList.Add(AObject: TObject): Integer; {$ifdef CLASSESINLINE}inline;{$endif}
begin
  Result := FList.Add(AObject);
end;

procedure TFPObjectList.Delete(Index: Integer); {$ifdef CLASSESINLINE}inline;{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if OwnsObjects then
    TObject(FList[Index]).Free;
  FList.Delete(Index);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.Exchange(Index1, Index2: Integer);
=======
procedure TFPObjectList.Exchange(Index1, Index2: Integer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.Exchange(Index1, Index2: Integer);
>>>>>>> origin/fixes_2_2
begin
  FList.Exchange(Index1, Index2);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPObjectList.Expand: TFPObjectList;
begin
  FList.Expand;
  Result:=Self;
end;

Function TFPObjectList.Extract(Item: TObject): TObject;
begin
  Result:=TObject(FList.Extract(Item));
end;

Function TFPObjectList.Remove(AObject: TObject): Integer;
begin
  Result:=IndexOf(AObject);
  if (Result <> -1) then
    begin
    if OwnsObjects then
      TObject(FList[Result]).Free;
    FList.Delete(Result);
    end;
end;

Function TFPObjectList.IndexOf(AObject: TObject): Integer;
begin
  Result:=FList.IndexOf(Pointer(AObject));
end;

Function TFPObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt : Integer): Integer;
=======
=======
>>>>>>> origin/fixes_2_2
function TFPObjectList.Expand: TFPObjectList;
begin
  FList.Expand;
  Result := Self;
end;

function TFPObjectList.Extract(Item: TObject): TObject;
begin
  Result := TObject(FList.Extract(Item));
end;

function TFPObjectList.Remove(AObject: TObject): Integer;
begin
  Result := IndexOf(AObject);
  if (Result <> -1) then
  begin
    if OwnsObjects then
      TObject(FList[Result]).Free;
    FList.Delete(Result);
  end;
end;

function TFPObjectList.IndexOf(AObject: TObject): Integer;
begin
  Result := FList.IndexOf(Pointer(AObject));
end;

function TFPObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt : Integer): Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  I : Integer;
begin
  I:=AStartAt;
  Result:=-1;
<<<<<<< HEAD
<<<<<<< HEAD
  if AExact then
    while (I<Count) and (Result=-1) do
      if Items[i].ClassType=AClass then
=======
  If AExact then
    while (I<Count) and (Result=-1) do
      If Items[i].ClassType=AClass then
>>>>>>> graemeg/fixes_2_2
=======
  If AExact then
    while (I<Count) and (Result=-1) do
      If Items[i].ClassType=AClass then
>>>>>>> origin/fixes_2_2
        Result:=I
      else
        Inc(I)
  else
    while (I<Count) and (Result=-1) do
<<<<<<< HEAD
<<<<<<< HEAD
      if Items[i].InheritsFrom(AClass) then
=======
      If Items[i].InheritsFrom(AClass) then
>>>>>>> graemeg/fixes_2_2
=======
      If Items[i].InheritsFrom(AClass) then
>>>>>>> origin/fixes_2_2
        Result:=I
      else
        Inc(I);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.Insert(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
=======
procedure TFPObjectList.Insert(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.Insert(Index: Integer; AObject: TObject); {$ifdef CLASSESINLINE}inline;{$endif}
>>>>>>> origin/fixes_2_2
begin
  FList.Insert(Index, Pointer(AObject));
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.Move(CurIndex, NewIndex: Integer);
=======
procedure TFPObjectList.Move(CurIndex, NewIndex: Integer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.Move(CurIndex, NewIndex: Integer);
>>>>>>> origin/fixes_2_2
begin
  FList.Move(CurIndex, NewIndex);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.Assign(Obj: TFPObjectList);
=======
procedure TFPObjectList.Assign(Obj: TFPObjectList);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.Assign(Obj: TFPObjectList);
>>>>>>> origin/fixes_2_2
var
  i: Integer;
begin
  Clear;
<<<<<<< HEAD
<<<<<<< HEAD
  for i:=0 to Obj.Count - 1 do
    Add(Obj[i]);
end;

Procedure TFPObjectList.Pack;
=======
=======
>>>>>>> origin/fixes_2_2
  for I := 0 to Obj.Count - 1 do
    Add(Obj[i]);
end;

procedure TFPObjectList.Pack;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  FList.Pack;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.Sort(Compare: TListSortCompare);
=======
procedure TFPObjectList.Sort(Compare: TListSortCompare);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.Sort(Compare: TListSortCompare);
>>>>>>> origin/fixes_2_2
begin
  FList.Sort(Compare);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPObjectList.First: TObject;
begin
  Result:=TObject(FList.First);
end;

Function TFPObjectList.Last: TObject;
begin
  Result:=TObject(FList.Last);
end;

Procedure TFPObjectList.ForEachCall(proc2call:TObjectListCallback;arg:pointer);
=======
=======
>>>>>>> origin/fixes_2_2
function TFPObjectList.First: TObject;
begin
  Result := TObject(FList.First);
end;

function TFPObjectList.Last: TObject;
begin
  Result := TObject(FList.Last);
end;

procedure TFPObjectList.ForEachCall(proc2call:TObjectListCallback;arg:pointer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  FList.ForEachCall(TListCallBack(proc2call),arg);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPObjectList.ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
=======
procedure TFPObjectList.ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPObjectList.ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
>>>>>>> origin/fixes_2_2
begin
  FList.ForEachCall(TListStaticCallBack(proc2call),arg);
end;


{ TObjectList }

<<<<<<< HEAD
<<<<<<< HEAD
constructor TObjectList.Create(FreeObjects: boolean);
begin
  inherited Create;
  FFreeObjects:=FreeObjects;
end;

constructor TObjectList.Create;
begin
  inherited Create;
  FFreeObjects:=True;
end;

Procedure TObjectList.Notify(Ptr: Pointer; Action: TListNotification);
=======
=======
>>>>>>> origin/fixes_2_2
constructor tobjectlist.create(freeobjects : boolean);

begin
  inherited create;
  ffreeobjects:=freeobjects;
end;

Constructor tobjectlist.create;

begin
  inherited create;
  ffreeobjects:=True;
end;

Procedure TObjectList.Notify(Ptr: Pointer; Action: TListNotification);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if FFreeObjects then
    if (Action=lnDeleted) then
      TObject(Ptr).Free;
  inherited Notify(Ptr,Action);
end;


Function TObjectList.GetItem(Index: Integer): TObject;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=TObject(inherited Get(Index));
=======

begin
  Result:=TObject(Inherited Get(Index));
>>>>>>> graemeg/fixes_2_2
=======

begin
  Result:=TObject(Inherited Get(Index));
>>>>>>> origin/fixes_2_2
end;


Procedure TObjectList.SetItem(Index: Integer; AObject: TObject);
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
begin
  // Put will take care of deleting old one in Notify.
  Put(Index,Pointer(AObject));
end;


Function TObjectList.Add(AObject: TObject): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=inherited Add(Pointer(AObject));
=======

begin
  Result:=Inherited Add(Pointer(AObject));
>>>>>>> graemeg/fixes_2_2
=======

begin
  Result:=Inherited Add(Pointer(AObject));
>>>>>>> origin/fixes_2_2
end;


Function TObjectList.Extract(Item: TObject): TObject;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=TObject(inherited Extract(Pointer(Item)));
=======

begin
  Result:=Tobject(Inherited Extract(Pointer(Item)));
>>>>>>> graemeg/fixes_2_2
=======

begin
  Result:=Tobject(Inherited Extract(Pointer(Item)));
>>>>>>> origin/fixes_2_2
end;


Function TObjectList.Remove(AObject: TObject): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=inherited Remove(Pointer(AObject));
=======

begin
  Result:=Inherited Remove(Pointer(AObject));
>>>>>>> graemeg/fixes_2_2
=======

begin
  Result:=Inherited Remove(Pointer(AObject));
>>>>>>> origin/fixes_2_2
end;


Function TObjectList.IndexOf(AObject: TObject): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=inherited IndexOf(Pointer(AObject));
end;


Function TObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean;
  AStartAt: Integer): Integer;
var
  I : Integer;
begin
  I:=AStartAt;
  Result:=-1;
  if AExact then
    while (I<Count) and (Result=-1) do
      if Items[i].ClassType=AClass then
=======
=======
>>>>>>> origin/fixes_2_2

begin
  Result:=Inherited indexOF(Pointer(AObject));
end;


Function TObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt : Integer): Integer;

Var
  I : Integer;

begin
  I:=AStartAt;
  Result:=-1;
  If AExact then
    While (I<Count) and (Result=-1) do
      If Items[i].ClassType=AClass then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        Result:=I
      else
        Inc(I)
  else
<<<<<<< HEAD
<<<<<<< HEAD
    while (I<Count) and (Result=-1) do
      if Items[i].InheritsFrom(AClass) then
=======
    While (I<Count) and (Result=-1) do
      If Items[i].InheritsFrom(AClass) then
>>>>>>> graemeg/fixes_2_2
=======
    While (I<Count) and (Result=-1) do
      If Items[i].InheritsFrom(AClass) then
>>>>>>> origin/fixes_2_2
        Result:=I
      else
        Inc(I);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TObjectList.Insert(Index: Integer; AObject: TObject);
=======
procedure TObjectList.Insert(Index: Integer; AObject: TObject);
>>>>>>> graemeg/fixes_2_2
=======
procedure TObjectList.Insert(Index: Integer; AObject: TObject);
>>>>>>> origin/fixes_2_2
begin
  Inherited Insert(Index,Pointer(AObject));
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TObjectList.First: TObject;
begin
  Result:=TObject(inherited First);
end;


Function TObjectList.Last: TObject;
begin
  Result:=TObject(inherited Last);
=======
=======
>>>>>>> origin/fixes_2_2
function TObjectList.First: TObject;

begin
  Result := TObject(Inherited First);
end;


function TObjectList.Last: TObject;

begin
  Result := TObject(Inherited Last);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ TListComponent }

<<<<<<< HEAD
<<<<<<< HEAD
type
  TlistComponent = class(TComponent)
  private
    Flist : TComponentList;
  public
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
  end;

Procedure TlistComponent.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  if (Operation=opRemove) then
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TlistComponent = Class(TComponent)
  Private
    Flist : TComponentList;
  Public
    procedure Notification(AComponent: TComponent; Operation: TOperation); override;
  end;

procedure TlistComponent.Notification(AComponent: TComponent;
  Operation: TOperation);
begin
  If (Operation=opremove) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Flist.HandleFreeNotify(Self,AComponent);
  inherited;
end;

{ TComponentList }

Function TComponentList.Add(AComponent: TComponent): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=inherited Add(AComponent);
=======
  Result:=Inherited Add(AComponent);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Inherited Add(AComponent);
>>>>>>> origin/fixes_2_2
end;

destructor TComponentList.Destroy;
begin
  inherited;
  FreeAndNil(FNotifier);
end;

Function TComponentList.Extract(Item: TComponent): TComponent;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TComponent(inherited Extract(Item));
=======
  Result:=TComponent(Inherited Extract(Item));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TComponent(Inherited Extract(Item));
>>>>>>> origin/fixes_2_2
end;

Function TComponentList.First: TComponent;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TComponent(inherited First);
=======
  Result:=TComponent(Inherited First);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TComponent(Inherited First);
>>>>>>> origin/fixes_2_2
end;

Function TComponentList.GetItems(Index: Integer): TComponent;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TComponent(inherited Items[Index]);
=======
  Result:=TComponent(Inherited Items[Index]);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TComponent(Inherited Items[Index]);
>>>>>>> origin/fixes_2_2
end;

Procedure TComponentList.HandleFreeNotify(Sender: TObject;
  AComponent: TComponent);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Extract(AComponent);
=======
  Extract(Acomponent);
>>>>>>> graemeg/fixes_2_2
=======
  Extract(Acomponent);
>>>>>>> origin/fixes_2_2
end;

Function TComponentList.IndexOf(AComponent: TComponent): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=inherited IndexOf(AComponent);
=======
  Result:=Inherited IndexOf(AComponent);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Inherited IndexOf(AComponent);
>>>>>>> origin/fixes_2_2
end;

Procedure TComponentList.Insert(Index: Integer; AComponent: TComponent);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Insert(Index,AComponent)
=======
  Inherited Insert(Index,Acomponent)
>>>>>>> graemeg/fixes_2_2
=======
  Inherited Insert(Index,Acomponent)
>>>>>>> origin/fixes_2_2
end;

Function TComponentList.Last: TComponent;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TComponent(inherited Last);
=======
  Result:=TComponent(Inherited Last);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TComponent(Inherited Last);
>>>>>>> origin/fixes_2_2
end;

Procedure TComponentList.Notify(Ptr: Pointer; Action: TListNotification);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if FNotifier=nil then
=======
  If FNotifier=NIl then
>>>>>>> graemeg/fixes_2_2
=======
  If FNotifier=NIl then
>>>>>>> origin/fixes_2_2
    begin
    FNotifier:=TlistComponent.Create(nil);
    TlistComponent(FNotifier).FList:=Self;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  if Assigned(Ptr) then
    with TComponent(Ptr) do
=======
  If Assigned(Ptr) then
    With TComponent(Ptr) do
>>>>>>> graemeg/fixes_2_2
=======
  If Assigned(Ptr) then
    With TComponent(Ptr) do
>>>>>>> origin/fixes_2_2
      case Action of
        lnAdded : FreeNotification(FNotifier);
        lnExtracted, lnDeleted: RemoveFreeNotification(FNotifier);
      end;
  inherited Notify(Ptr, Action);
end;

Function TComponentList.Remove(AComponent: TComponent): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=inherited Remove(AComponent);
=======
  Result:=Inherited Remove(AComponent);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Inherited Remove(AComponent);
>>>>>>> origin/fixes_2_2
end;

Procedure TComponentList.SetItems(Index: Integer; AComponent: TComponent);
begin
  Put(Index,AComponent);
end;

{ TClassList }

Function TClassList.Add(AClass: TClass): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=inherited Add(Pointer(AClass));
=======
  Result:=Inherited Add(Pointer(AClass));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Inherited Add(Pointer(AClass));
>>>>>>> origin/fixes_2_2
end;

Function TClassList.Extract(Item: TClass): TClass;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TClass(inherited Extract(Pointer(Item)));
=======
  Result:=TClass(Inherited Extract(Pointer(Item)));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TClass(Inherited Extract(Pointer(Item)));
>>>>>>> origin/fixes_2_2
end;

Function TClassList.First: TClass;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TClass(inherited First);
=======
  Result:=TClass(Inherited First);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TClass(Inherited First);
>>>>>>> origin/fixes_2_2
end;

Function TClassList.GetItems(Index: Integer): TClass;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TClass(inherited Items[Index]);
=======
  Result:=TClass(Inherited Items[Index]);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TClass(Inherited Items[Index]);
>>>>>>> origin/fixes_2_2
end;

Function TClassList.IndexOf(AClass: TClass): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=inherited IndexOf(Pointer(AClass));
=======
  Result:=Inherited IndexOf(Pointer(AClass));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Inherited IndexOf(Pointer(AClass));
>>>>>>> origin/fixes_2_2
end;

Procedure TClassList.Insert(Index: Integer; AClass: TClass);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Insert(Index,Pointer(AClass));
=======
  Inherited Insert(index,Pointer(AClass));
>>>>>>> graemeg/fixes_2_2
=======
  Inherited Insert(index,Pointer(AClass));
>>>>>>> origin/fixes_2_2
end;

Function TClassList.Last: TClass;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TClass(inherited Last);
=======
  Result:=TClass(Inherited Last);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TClass(Inherited Last);
>>>>>>> origin/fixes_2_2
end;

Function TClassList.Remove(AClass: TClass): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=inherited Remove(Pointer(AClass));
=======
  Result:=Inherited Remove(Pointer(AClass));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Inherited Remove(Pointer(AClass));
>>>>>>> origin/fixes_2_2
end;

Procedure TClassList.SetItems(Index: Integer; AClass: TClass);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Put(Index,Pointer(AClass));
=======
  Put(Index,Pointer(Aclass));
>>>>>>> graemeg/fixes_2_2
=======
  Put(Index,Pointer(Aclass));
>>>>>>> origin/fixes_2_2
end;

{ TOrderedList }

Function TOrderedList.AtLeast(ACount: Integer): Boolean;
begin
  Result:=(FList.Count>=Acount)
end;

Function TOrderedList.Count: Integer;
begin
  Result:=FList.Count;
end;

constructor TOrderedList.Create;
begin
  FList:=Tlist.Create;
end;

destructor TOrderedList.Destroy;
begin
  FList.Free;
end;

Function TOrderedList.Peek: Pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if AtLeast(1) then
    Result:=PeekItem
  else
    Result:=nil;
=======
=======
>>>>>>> origin/fixes_2_2
  If AtLeast(1) then
    Result:=PeekItem
  else
    Result:=Nil;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function TOrderedList.PeekItem: Pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  with Flist do
=======
  With Flist do
>>>>>>> graemeg/fixes_2_2
=======
  With Flist do
>>>>>>> origin/fixes_2_2
    Result:=Items[Count-1]
end;

Function TOrderedList.Pop: Pointer;
begin
  If Atleast(1) then
    Result:=PopItem
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=nil;
=======
    Result:=Nil;
>>>>>>> graemeg/fixes_2_2
=======
    Result:=Nil;
>>>>>>> origin/fixes_2_2
end;

Function TOrderedList.PopItem: Pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  with FList do
    if Count>0 then
=======
  With FList do
    If Count>0 then
>>>>>>> graemeg/fixes_2_2
=======
  With FList do
    If Count>0 then
>>>>>>> origin/fixes_2_2
      begin
      Result:=Items[Count-1];
      Delete(Count-1);
      end
    else
<<<<<<< HEAD
<<<<<<< HEAD
      Result:=nil;
=======
      Result:=Nil;
>>>>>>> graemeg/fixes_2_2
=======
      Result:=Nil;
>>>>>>> origin/fixes_2_2
end;

Function TOrderedList.Push(AItem: Pointer): Pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  PushItem(AItem);
=======
  PushItem(Aitem);
>>>>>>> graemeg/fixes_2_2
=======
  PushItem(Aitem);
>>>>>>> origin/fixes_2_2
  Result:=AItem;
end;

{ TStack }

Procedure TStack.PushItem(AItem: Pointer);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FList.Add(AItem);
=======
  FList.Add(Aitem);
>>>>>>> graemeg/fixes_2_2
=======
  FList.Add(Aitem);
>>>>>>> origin/fixes_2_2
end;

{ TObjectStack }

Function TObjectStack.Peek: TObject;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TObject(inherited Peek);
=======
  Result:=TObject(Inherited Peek);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TObject(Inherited Peek);
>>>>>>> origin/fixes_2_2
end;

Function TObjectStack.Pop: TObject;
begin
  Result:=TObject(Inherited Pop);
end;

Function TObjectStack.Push(AObject: TObject): TObject;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TObject(inherited Push(Pointer(AObject)));
=======
  Result:=TObject(Inherited Push(Pointer(AObject)));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TObject(Inherited Push(Pointer(AObject)));
>>>>>>> origin/fixes_2_2
end;

{ TQueue }

Procedure TQueue.PushItem(AItem: Pointer);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  with FList Do
=======
  With Flist Do
>>>>>>> graemeg/fixes_2_2
=======
  With Flist Do
>>>>>>> origin/fixes_2_2
    Insert(0,AItem);
end;

{ TObjectQueue }

Function TObjectQueue.Peek: TObject;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TObject(inherited Peek);
=======
  Result:=TObject(Inherited Peek);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TObject(Inherited Peek);
>>>>>>> origin/fixes_2_2
end;

Function TObjectQueue.Pop: TObject;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TObject(inherited Pop);
=======
  Result:=TObject(Inherited Pop);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TObject(Inherited Pop);
>>>>>>> origin/fixes_2_2
end;

Function TObjectQueue.Push(AObject: TObject): TObject;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TObject(inherited Push(Pointer(AObject)));
=======
  Result:=TObject(Inherited Push(Pointer(Aobject)));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=TObject(Inherited Push(Pointer(Aobject)));
>>>>>>> origin/fixes_2_2
end;


{*****************************************************************************
                            TFPHashList
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
    Function FPHash(const s:shortstring):LongWord;
    var
      p,pmax : PChar;
    begin
{$push}
{$Q-}
      Result:=0;
      p:=@s[1];
      pmax:=@s[length(s)+1];
      while (p<pmax) do
        begin
          Result:=LongWord(LongInt(Result shl 5) - LongInt(Result)) xor LongWord(P^);
          Inc(p);
        end;
{$pop}
    end;

    Function FPHash(P: PChar; Len: Integer): LongWord;
    var
      pmax : PChar;
    begin
{$push}
{$Q-}
      Result:=0;
      pmax:=p+len;
      while (p<pmax) do
        begin
          Result:=LongWord(LongInt(Result shl 5) - LongInt(Result)) xor LongWord(P^);
          Inc(p);
        end;
{$pop}
    end;


Procedure TFPHashList.RaiseIndexError(Index : Integer);
=======
=======
>>>>>>> origin/fixes_2_2
    function FPHash1(const s:shortstring):LongWord;
      Var
        g : LongWord;
        p,pmax : pchar;
      begin
        result:=0;
        p:=@s[1];
        pmax:=@s[length(s)+1];
        while (p<pmax) do
          begin
            result:=result shl 4 + LongWord(p^);
            g:=result and LongWord($F0000000);
            if g<>0 then
              result:=result xor (g shr 24) xor g;
            inc(p);
          end;
        If result=0 then
          result:=$ffffffff;
      end;

    function FPHash(const s:shortstring):LongWord;
      Var
        p,pmax : pchar;
      begin
{$ifopt Q+}
{$define overflowon}
{$Q-}
{$endif}
        result:=0;
        p:=@s[1];
        pmax:=@s[length(s)+1];
        while (p<pmax) do
          begin
            result:=LongWord((result shl 5) - result) xor LongWord(P^);
            inc(p);
          end;
{$ifdef overflowon}
{$Q+}
{$undef overflowon}
{$endif}
      end;


procedure TFPHashList.RaiseIndexError(Index : Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Error(SListIndexError, Index);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.Get(Index: Integer): Pointer;
=======
function TFPHashList.Get(Index: Integer): Pointer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.Get(Index: Integer): Pointer;
>>>>>>> origin/fixes_2_2
begin
  If (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  Result:=FHashList^[Index].Data;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashList.Put(Index: Integer; Item: Pointer);
begin
  if (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  FHashList^[Index].Data:=Item;
end;


Function TFPHashList.NameOfIndex(Index: Integer): shortstring;
begin
  if (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  with FHashList^[Index] do
    begin
    if StrIndex>=0 then
      Result:=PShortString(@FStrs[StrIndex])^
    else
      Result:='';
=======
=======
>>>>>>> origin/fixes_2_2
procedure TFPHashList.Put(Index: Integer; Item: Pointer);
begin
  if (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  FHashList^[Index].Data:=Item;;
end;


function TFPHashList.NameOfIndex(Index: Integer): shortstring;
begin
  If (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  with FHashList^[Index] do
    begin
      if StrIndex>=0 then
        Result:=PShortString(@FStrs[StrIndex])^
      else
        Result:='';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.HashOfIndex(Index: Integer): LongWord;
=======
function TFPHashList.HashOfIndex(Index: Integer): LongWord;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.HashOfIndex(Index: Integer): LongWord;
>>>>>>> origin/fixes_2_2
begin
  If (Index < 0) or (Index >= FCount) then
    RaiseIndexError(Index);
  Result:=FHashList^[Index].HashValue;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.GetNextCollision(Index: Integer): Integer;
begin
  Result:=-1;
  if ((Index > -1) and (Index < FCount)) then
    Result:=FHashList^[Index].NextIndex;
end;


Function TFPHashList.Extract(item: Pointer): Pointer;
var
  i : Integer;
begin
  Result:=nil;
  i:=IndexOf(item);
  if i >= 0 then
    begin
    Result:=item;
    Delete(i);
    end;
end;


Procedure TFPHashList.SetCapacity(NewCapacity: Integer);
begin
  if (NewCapacity < FCount) or (NewCapacity > MaxHashListSize) then
     Error (SListCapacityError, NewCapacity);
  if NewCapacity = FCapacity then
    Exit;
  ReallocMem(FHashList, NewCapacity*SizeOf(THashItem));
  FCapacity:=NewCapacity;
  { Maybe expand hash also }
  if FCapacity>FHashCapacity*MaxItemsPerHash then
    SetHashCapacity(FCapacity div MaxItemsPerHash);
end;


Procedure TFPHashList.SetCount(NewCount: Integer);
begin
  if (NewCount < 0) or (NewCount > MaxHashListSize)then
    Error(SListCountError, NewCount);
  if NewCount > FCount then
    begin
    if NewCount > FCapacity then
      SetCapacity(NewCount);
    if FCount < NewCount then
      FillChar(FHashList^[FCount], (NewCount-FCount) div SizeOf(THashItem), 0);
    end;
  FCount:=NewCount;
end;


Procedure TFPHashList.SetStrCapacity(NewCapacity: Integer);
begin
  if (NewCapacity < FStrCount) or (NewCapacity > MaxHashStrSize) then
    Error(SListCapacityError, NewCapacity);
  if NewCapacity = FStrCapacity then
    Exit;
  ReallocMem(FStrs, NewCapacity);
  FStrCapacity:=NewCapacity;
end;


Procedure TFPHashList.SetHashCapacity(NewCapacity: Integer);
begin
  if (NewCapacity < 1) then
    Error(SListCapacityError, NewCapacity);
  if FHashCapacity=NewCapacity then
    Exit;
  FHashCapacity:=NewCapacity;
  ReallocMem(FHashTable, FHashCapacity*SizeOf(Integer));
=======
=======
>>>>>>> origin/fixes_2_2
function TFPHashList.Extract(item: Pointer): Pointer;
var
  i : Integer;
begin
  result := nil;
  i := IndexOf(item);
  if i >= 0 then
   begin
     Result := item;
     Delete(i);
   end;
end;


procedure TFPHashList.SetCapacity(NewCapacity: Integer);
begin
  If (NewCapacity < FCount) or (NewCapacity > MaxHashListSize) then
     Error (SListCapacityError, NewCapacity);
  if NewCapacity = FCapacity then
    exit;
  ReallocMem(FHashList, NewCapacity*SizeOf(THashItem));
  FCapacity := NewCapacity;
end;


procedure TFPHashList.SetCount(NewCount: Integer);
begin
  if (NewCount < 0) or (NewCount > MaxHashListSize)then
    Error(SListCountError, NewCount);
  If NewCount > FCount then
    begin
      If NewCount > FCapacity then
        SetCapacity(NewCount);
      If FCount < NewCount then
        FillChar(FHashList^[FCount], (NewCount-FCount) div Sizeof(THashItem), 0);
    end;
  FCount := Newcount;
end;


procedure TFPHashList.SetStrCapacity(NewCapacity: Integer);
begin
  If (NewCapacity < FStrCount) or (NewCapacity > MaxHashStrSize) then
     Error (SListCapacityError, NewCapacity);
  if NewCapacity = FStrCapacity then
    exit;
  ReallocMem(FStrs, NewCapacity);
  FStrCapacity := NewCapacity;
end;


procedure TFPHashList.SetHashCapacity(NewCapacity: Integer);
begin
  If (NewCapacity < 1) then
    Error (SListCapacityError, NewCapacity);
  if FHashCapacity=NewCapacity then
    exit;
  FHashCapacity:=NewCapacity;
  ReallocMem(FHashTable, FHashCapacity*sizeof(Integer));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ReHash;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashList.ReHash;
=======
procedure TFPHashList.ReHash;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashList.ReHash;
>>>>>>> origin/fixes_2_2
var
  i : Integer;
begin
  FillDword(FHashTable^,FHashCapacity,LongWord(-1));
<<<<<<< HEAD
<<<<<<< HEAD
  for i:=0 to FCount-1 do
=======
  For i:=0 To FCount-1 Do
>>>>>>> graemeg/fixes_2_2
=======
  For i:=0 To FCount-1 Do
>>>>>>> origin/fixes_2_2
    AddToHashTable(i);
end;


constructor TFPHashList.Create;
begin
  SetHashCapacity(1);
end;


destructor TFPHashList.Destroy;
begin
  Clear;
<<<<<<< HEAD
<<<<<<< HEAD
  if Assigned(FHashTable) then
=======
  if assigned(FHashTable) then
>>>>>>> graemeg/fixes_2_2
=======
  if assigned(FHashTable) then
>>>>>>> origin/fixes_2_2
    FreeMem(FHashTable);
  inherited Destroy;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.AddStr(const s:shortstring): Integer;
var
  Len : Integer;
begin
  len:=Length(s)+1;
  if FStrCount+Len >= FStrCapacity then
    StrExpand(Len);
  System.Move(s[0],FStrs[FStrCount],Len);
  Result:=FStrCount;
  Inc(FStrCount,Len);
end;


Procedure TFPHashList.AddToHashTable(Index: Integer);
=======
=======
>>>>>>> origin/fixes_2_2
function TFPHashList.AddStr(const s:shortstring): Integer;
var
  Len : Integer;
begin
  len:=length(s)+1;
  if FStrCount+Len >= FStrCapacity then
    StrExpand(Len);
  System.Move(s[0],FStrs[FStrCount],Len);
  result:=FStrCount;
  inc(FStrCount,Len);
end;


procedure TFPHashList.AddToHashTable(Index: Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  HashIndex : Integer;
begin
  with FHashList^[Index] do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    if not Assigned(Data) then
      Exit;
    HashIndex:=HashValue mod LongWord(FHashCapacity);
    NextIndex:=FHashTable^[HashIndex];
    FHashTable^[HashIndex]:=Index;
=======
=======
>>>>>>> origin/fixes_2_2
      if not assigned(Data) then
        exit;
      HashIndex:=HashValue mod LongWord(FHashCapacity);
      NextIndex:=FHashTable^[HashIndex];
      FHashTable^[HashIndex]:=Index;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.Add(const AName:shortstring;Item: Pointer): Integer;
=======
function TFPHashList.Add(const AName:shortstring;Item: Pointer): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.Add(const AName:shortstring;Item: Pointer): Integer;
>>>>>>> origin/fixes_2_2
begin
  if FCount = FCapacity then
    Expand;
  with FHashList^[FCount] do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    HashValue:=FPHash(AName);
    Data:=Item;
    StrIndex:=AddStr(AName);
    end;
  AddToHashTable(FCount);
  Result:=FCount;
  Inc(FCount);
end;

Procedure TFPHashList.Clear;
begin
  if Assigned(FHashList) then
    begin
    FCount:=0;
    SetCapacity(0);
    FHashList:=nil;
    end;
  SetHashCapacity(1);
  FHashTable^[0]:=(-1); // sethashcapacity does not always call rehash
  if Assigned(FStrs) then
    begin
    FStrCount:=0;
    SetStrCapacity(0);
    FStrs:=nil;
    end;
end;

Procedure TFPHashList.Delete(Index: Integer);
begin
  if (Index<0) or (Index>=FCount) then
    Error(SListIndexError, Index);
  { Remove from HashList }
  Dec(FCount);
  System.Move(FHashList^[Index+1], FHashList^[Index], (FCount - Index) * SizeOf(THashItem));
  { All indexes are updated, we need to build the hashtable again }
  ReHash;
  { Shrink the list if appropriate }
  if (FCapacity > 256) and (FCount < FCapacity shr 2) then
    begin
    FCapacity:=FCapacity shr 1;
    ReAllocMem(FHashList, SizeOf(THashItem) * FCapacity);
    end;
end;

Function TFPHashList.Remove(Item: Pointer): Integer;
begin
  Result:=IndexOf(Item);
=======
=======
>>>>>>> origin/fixes_2_2
      HashValue:=FPHash(AName);
      Data:=Item;
      StrIndex:=AddStr(AName);
    end;
  AddToHashTable(FCount);
  Result := FCount;
  inc(FCount);
end;

procedure TFPHashList.Clear;
begin
  if Assigned(FHashList) then
    begin
      FCount:=0;
      SetCapacity(0);
      FHashList := nil;
    end;
  SetHashCapacity(1);
  FHashTable^[0]:=longword(-1); // sethashcapacity does not always call rehash
  if Assigned(FStrs) then
    begin
      FStrCount:=0;
      SetStrCapacity(0);
      FStrs := nil;
    end;
end;

procedure TFPHashList.Delete(Index: Integer);
begin
  If (Index<0) or (Index>=FCount) then
    Error (SListIndexError, Index);
  { Remove from HashList }
  dec(FCount);
  System.Move (FHashList^[Index+1], FHashList^[Index], (FCount - Index) * Sizeof(THashItem));
  { All indexes are updated, we need to build the hashtable again }
  Rehash;
  { Shrink the list if appropriate }
  if (FCapacity > 256) and (FCount < FCapacity shr 2) then
    begin
      FCapacity := FCapacity shr 1;
      ReallocMem(FHashList, Sizeof(THashItem) * FCapacity);
    end;
end;

function TFPHashList.Remove(Item: Pointer): Integer;
begin
  Result := IndexOf(Item);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  If Result <> -1 then
    Self.Delete(Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
class Procedure TFPHashList.Error(const Msg: string; Data: PtrInt);
begin
  raise EListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;

Function TFPHashList.Expand: TFPHashList;
var
  IncSize : Longint;
begin
  Result:=Self;
  if FCount < FCapacity then
    Exit;
  IncSize:=SizeOf(PtrInt)*2;
  if FCapacity > 127 then
    Inc(IncSize, FCapacity shr 2)
  else if FCapacity > SizeOf(PtrInt)*3 then
    Inc(IncSize, FCapacity shr 1)
  else if FCapacity >= SizeOf(PtrInt) then
    Inc(IncSize,sizeof(PtrInt));
  SetCapacity(FCapacity + IncSize);
end;

Procedure TFPHashList.StrExpand(MinIncSize:Integer);
=======
class procedure TFPHashList.Error(const Msg: string; Data: PtrInt);
begin
  Raise EListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame);
end;

function TFPHashList.Expand: TFPHashList;
var
  IncSize : Longint;
begin
  Result := Self;
  if FCount < FCapacity then
    exit;
  IncSize := sizeof(ptrint)*2;
  if FCapacity > 127 then
    Inc(IncSize, FCapacity shr 2)
  else if FCapacity > sizeof(ptrint)*3 then
    Inc(IncSize, FCapacity shr 1)
  else if FCapacity >= sizeof(ptrint) then
    inc(IncSize,sizeof(ptrint));
=======
class procedure TFPHashList.Error(const Msg: string; Data: PtrInt);
begin
  Raise EListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame);
end;

function TFPHashList.Expand: TFPHashList;
var
  IncSize : Longint;
begin
  Result := Self;
  if FCount < FCapacity then
    exit;
  IncSize := sizeof(ptrint)*2;
  if FCapacity > 127 then
    Inc(IncSize, FCapacity shr 2)
  else if FCapacity > sizeof(ptrint)*3 then
    Inc(IncSize, FCapacity shr 1)
  else if FCapacity >= sizeof(ptrint) then
    inc(IncSize,sizeof(ptrint));
>>>>>>> origin/fixes_2_2
  SetCapacity(FCapacity + IncSize);
  { Maybe expand hash also }
  if FCount>FHashCapacity*MaxItemsPerHash then
    SetHashCapacity(FCount div MaxItemsPerHash);
end;

procedure TFPHashList.StrExpand(MinIncSize:Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  IncSize : Longint;
begin
  if FStrCount+MinIncSize < FStrCapacity then
<<<<<<< HEAD
<<<<<<< HEAD
    Exit;
  IncSize:=64;
=======
    exit;
  IncSize := 64;
>>>>>>> graemeg/fixes_2_2
=======
    exit;
  IncSize := 64;
>>>>>>> origin/fixes_2_2
  if FStrCapacity > 255 then
    Inc(IncSize, FStrCapacity shr 2);
  SetStrCapacity(FStrCapacity + IncSize + MinIncSize);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.IndexOf(Item: Pointer): Integer;
=======
function TFPHashList.IndexOf(Item: Pointer): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.IndexOf(Item: Pointer): Integer;
>>>>>>> origin/fixes_2_2
var
  psrc  : PHashItem;
  Index : integer;
begin
  Result:=-1;
  psrc:=@FHashList^[0];
<<<<<<< HEAD
<<<<<<< HEAD
  for Index:=0 to FCount-1 do
    begin
    if psrc^.Data=Item then
      begin
      Result:=Index;
      Exit;
      end;
    Inc(psrc);
    end;
end;

Function TFPHashList.InternalFind(AHash:LongWord;const AName:shortstring;out PrevIndex:Integer):Integer;
=======
=======
>>>>>>> origin/fixes_2_2
  For Index:=0 To FCount-1 Do
    begin
      if psrc^.Data=Item then
        begin
          Result:=Index;
          exit;
        end;
      inc(psrc);
    end;
end;

function TFPHashList.InternalFind(AHash:LongWord;const AName:shortstring;out PrevIndex:Integer):Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  HashIndex : Integer;
  Len,
  LastChar  : Char;
begin
  HashIndex:=AHash mod LongWord(FHashCapacity);
  Result:=FHashTable^[HashIndex];
  Len:=Char(Length(AName));
  LastChar:=AName[Byte(Len)];
  PrevIndex:=-1;
  while Result<>-1 do
<<<<<<< HEAD
<<<<<<< HEAD
    with FHashList^[Result] do
      begin
      if Assigned(Data) and
         (HashValue=AHash) and
         (Len=FStrs[StrIndex]) and
         (LastChar=FStrs[StrIndex+Byte(Len)]) and
         (AName=PShortString(@FStrs[StrIndex])^) then
        Exit;
      PrevIndex:=Result;
      Result:=NextIndex;
      end;
end;


Function TFPHashList.Find(const AName:shortstring): Pointer;
=======
=======
>>>>>>> origin/fixes_2_2
    begin
      with FHashList^[Result] do
        begin
          if assigned(Data) and
             (HashValue=AHash) and
             (Len=FStrs[StrIndex]) and
             (LastChar=FStrs[StrIndex+Byte(Len)]) and
             (AName=PShortString(@FStrs[StrIndex])^) then
            exit;
          PrevIndex:=Result;
          Result:=NextIndex;
        end;
    end;
end;


function TFPHashList.Find(const AName:shortstring): Pointer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  Index,
  PrevIndex : Integer;
begin
  Result:=nil;
  Index:=InternalFind(FPHash(AName),AName,PrevIndex);
  if Index=-1 then
<<<<<<< HEAD
<<<<<<< HEAD
    Exit;
=======
    exit;
>>>>>>> graemeg/fixes_2_2
=======
    exit;
>>>>>>> origin/fixes_2_2
  Result:=FHashList^[Index].Data;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.FindIndexOf(const AName:shortstring): Integer;
=======
function TFPHashList.FindIndexOf(const AName:shortstring): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.FindIndexOf(const AName:shortstring): Integer;
>>>>>>> origin/fixes_2_2
var
  PrevIndex : Integer;
begin
  Result:=InternalFind(FPHash(AName),AName,PrevIndex);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
=======
function TFPHashList.FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
>>>>>>> origin/fixes_2_2
var
  Index,
  PrevIndex : Integer;
begin
  Result:=nil;
  Index:=InternalFind(AHash,AName,PrevIndex);
  if Index=-1 then
<<<<<<< HEAD
<<<<<<< HEAD
    Exit;
=======
    exit;
>>>>>>> graemeg/fixes_2_2
=======
    exit;
>>>>>>> origin/fixes_2_2
  Result:=FHashList^[Index].Data;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashList.Rename(const AOldName,ANewName:shortstring): Integer;
=======
function TFPHashList.Rename(const AOldName,ANewName:shortstring): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashList.Rename(const AOldName,ANewName:shortstring): Integer;
>>>>>>> origin/fixes_2_2
var
  PrevIndex,
  Index : Integer;
  OldHash : LongWord;
begin
  Result:=-1;
  OldHash:=FPHash(AOldName);
  Index:=InternalFind(OldHash,AOldName,PrevIndex);
  if Index=-1 then
<<<<<<< HEAD
<<<<<<< HEAD
    Exit;
=======
    exit;
>>>>>>> graemeg/fixes_2_2
=======
    exit;
>>>>>>> origin/fixes_2_2
  { Remove from current Hash }
  if PrevIndex<>-1 then
    FHashList^[PrevIndex].NextIndex:=FHashList^[Index].NextIndex
  else
    FHashTable^[OldHash mod LongWord(FHashCapacity)]:=FHashList^[Index].NextIndex;
  { Set new name and hash }
  with FHashList^[Index] do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    HashValue:=FPHash(ANewName);
    StrIndex:=AddStr(ANewName);
=======
      HashValue:=FPHash(ANewName);
      StrIndex:=AddStr(ANewName);
>>>>>>> graemeg/fixes_2_2
=======
      HashValue:=FPHash(ANewName);
      StrIndex:=AddStr(ANewName);
>>>>>>> origin/fixes_2_2
    end;
  { Insert back in Hash }
  AddToHashTable(Index);
  { Return Index }
  Result:=Index;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashList.Pack;
=======
procedure TFPHashList.Pack;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashList.Pack;
>>>>>>> origin/fixes_2_2
var
  NewCount,
  i : integer;
  pdest,
  psrc : PHashItem;
<<<<<<< HEAD
<<<<<<< HEAD
  FOldStr : Pchar;
begin
  NewCount:=0;
  psrc:=@FHashList^[0];
  FOldStr:=FStrs;
  try
    FStrs:=nil;
    FStrCount:=0;
    FStrCapacity:=0;
    pdest:=psrc;
    for I:=0 to FCount-1 do
      begin
      if Assigned(psrc^.Data) then
        begin
        pdest^:=psrc^;
        pdest^.StrIndex:=AddStr(PShortString(@FOldStr[PDest^.StrIndex])^);
        Inc(pdest);
        Inc(NewCount);
        end;
      Inc(psrc);
      end;
  finally
    FreeMem(FoldStr);
  end;
=======
=======
>>>>>>> origin/fixes_2_2
begin
  NewCount:=0;
  psrc:=@FHashList^[0];
  pdest:=psrc;
  For I:=0 To FCount-1 Do
    begin
      if assigned(psrc^.Data) then
        begin
          pdest^:=psrc^;
          inc(pdest);
          inc(NewCount);
        end;
      inc(psrc);
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FCount:=NewCount;
  { We need to ReHash to update the IndexNext }
  ReHash;
  { Release over-capacity }
  SetCapacity(FCount);
  SetStrCapacity(FStrCount);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashList.ShowStatistics;
=======
procedure TFPHashList.ShowStatistics;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashList.ShowStatistics;
>>>>>>> origin/fixes_2_2
var
  HashMean,
  HashStdDev : Double;
  Index,
  i,j : Integer;
begin
  { Calculate Mean and StdDev }
  HashMean:=0;
  HashStdDev:=0;
  for i:=0 to FHashCapacity-1 do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    j:=0;
    Index:=FHashTable^[i];
    while (Index<>-1) do
      begin
      Inc(j);
      Index:=FHashList^[Index].NextIndex;
      end;
    HashMean:=HashMean+j;
    HashStdDev:=HashStdDev+Sqr(j);
    end;
  HashMean:=HashMean/FHashCapacity;
  HashStdDev:=(HashStdDev-FHashCapacity*Sqr(HashMean));
  if FHashCapacity>1 then
=======
=======
>>>>>>> origin/fixes_2_2
      j:=0;
      Index:=FHashTable^[i];
      while (Index<>-1) do
        begin
          inc(j);
          Index:=FHashList^[Index].NextIndex;
        end;
      HashMean:=HashMean+j;
      HashStdDev:=HashStdDev+Sqr(j);
    end;
  HashMean:=HashMean/FHashCapacity;
  HashStdDev:=(HashStdDev-FHashCapacity*Sqr(HashMean));
  If FHashCapacity>1 then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    HashStdDev:=Sqrt(HashStdDev/(FHashCapacity-1))
  else
    HashStdDev:=0;
  { Print info to stdout }
  Writeln('HashSize   : ',FHashCapacity);
  Writeln('HashMean   : ',HashMean:1:4);
  Writeln('HashStdDev : ',HashStdDev:1:4);
  Writeln('ListSize   : ',FCount,'/',FCapacity);
  Writeln('StringSize : ',FStrCount,'/',FStrCapacity);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashList.ForEachCall(proc2call:TListCallback;arg:pointer);
=======
procedure TFPHashList.ForEachCall(proc2call:TListCallback;arg:pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashList.ForEachCall(proc2call:TListCallback;arg:pointer);
>>>>>>> origin/fixes_2_2
var
  i : integer;
  p : pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  for i:=0 to Count-1 Do
    begin
    p:=FHashList^[i].Data;
    if Assigned(p) then
      proc2call(p,arg);
=======
=======
>>>>>>> origin/fixes_2_2
  For I:=0 To Count-1 Do
    begin
      p:=FHashList^[i].Data;
      if assigned(p) then
        proc2call(p,arg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashList.ForEachCall(proc2call:TListStaticCallback;arg:pointer);
=======
procedure TFPHashList.ForEachCall(proc2call:TListStaticCallback;arg:pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashList.ForEachCall(proc2call:TListStaticCallback;arg:pointer);
>>>>>>> origin/fixes_2_2
var
  i : integer;
  p : pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  for i:=0 to Count-1 Do
    begin
    p:=FHashList^[i].Data;
    if Assigned(p) then
      proc2call(p,arg);
=======
=======
>>>>>>> origin/fixes_2_2
  For I:=0 To Count-1 Do
    begin
      p:=FHashList^[i].Data;
      if assigned(p) then
        proc2call(p,arg);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;


{*****************************************************************************
                               TFPHashObject
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObject.InternalChangeOwner(HashObjectList:TFPHashObjectList;const s:shortstring);
=======
procedure TFPHashObject.InternalChangeOwner(HashObjectList:TFPHashObjectList;const s:shortstring);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObject.InternalChangeOwner(HashObjectList:TFPHashObjectList;const s:shortstring);
>>>>>>> origin/fixes_2_2
var
  Index : integer;
begin
  FOwner:=HashObjectList;
  Index:=HashObjectList.Add(s,Self);
  FStrIndex:=HashObjectList.List.List^[Index].StrIndex;
  FCachedStr:=PShortString(@FOwner.List.Strs[FStrIndex]);
end;


constructor TFPHashObject.CreateNotOwned;
begin
  FStrIndex:=-1;
end;


constructor TFPHashObject.Create(HashObjectList:TFPHashObjectList;const s:shortstring);
begin
  InternalChangeOwner(HashObjectList,s);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObject.ChangeOwner(HashObjectList:TFPHashObjectList);
=======
procedure TFPHashObject.ChangeOwner(HashObjectList:TFPHashObjectList);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObject.ChangeOwner(HashObjectList:TFPHashObjectList);
>>>>>>> origin/fixes_2_2
begin
  InternalChangeOwner(HashObjectList,PShortString(@FOwner.List.Strs[FStrIndex])^);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObject.ChangeOwnerAndName(HashObjectList:TFPHashObjectList;const s:shortstring);
=======
procedure TFPHashObject.ChangeOwnerAndName(HashObjectList:TFPHashObjectList;const s:shortstring);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObject.ChangeOwnerAndName(HashObjectList:TFPHashObjectList;const s:shortstring);
>>>>>>> origin/fixes_2_2
begin
  InternalChangeOwner(HashObjectList,s);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObject.Rename(const ANewName:shortstring);
=======
procedure TFPHashObject.Rename(const ANewName:shortstring);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObject.Rename(const ANewName:shortstring);
>>>>>>> origin/fixes_2_2
var
  Index : integer;
begin
  Index:=FOwner.Rename(PShortString(@FOwner.List.Strs[FStrIndex])^,ANewName);
  if Index<>-1 then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    FStrIndex:=FOwner.List.List^[Index].StrIndex;
    FCachedStr:=PShortString(@FOwner.List.Strs[FStrIndex]);
=======
      FStrIndex:=FOwner.List.List^[Index].StrIndex;
      FCachedStr:=PShortString(@FOwner.List.Strs[FStrIndex]);
>>>>>>> graemeg/fixes_2_2
=======
      FStrIndex:=FOwner.List.List^[Index].StrIndex;
      FCachedStr:=PShortString(@FOwner.List.Strs[FStrIndex]);
>>>>>>> origin/fixes_2_2
    end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashObject.GetName:shortstring;
begin
  if FOwner<>nil then
    begin
    FCachedStr:=PShortString(@FOwner.List.Strs[FStrIndex]);
    Result:=FCachedStr^;
=======
=======
>>>>>>> origin/fixes_2_2
function TFPHashObject.GetName:shortstring;
begin
  if FOwner<>nil then
    begin
      FCachedStr:=PShortString(@FOwner.List.Strs[FStrIndex]);
      Result:=FCachedStr^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end
  else
    Result:='';
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashObject.GetHash:Longword;
=======
function TFPHashObject.GetHash:Longword;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashObject.GetHash:Longword;
>>>>>>> origin/fixes_2_2
begin
  if FOwner<>nil then
    Result:=FPHash(PShortString(@FOwner.List.Strs[FStrIndex])^)
  else
    Result:=$ffffffff;
end;


{*****************************************************************************
            TFPHashObjectList (Copied from rtl/objpas/classes/lists.inc)
*****************************************************************************}

constructor TFPHashObjectList.Create(FreeObjects : boolean = True);
begin
  inherited Create;
<<<<<<< HEAD
<<<<<<< HEAD
  FHashList:=TFPHashList.Create;
  FFreeObjects:=Freeobjects;
=======
  FHashList := TFPHashList.Create;
  FFreeObjects := Freeobjects;
>>>>>>> graemeg/fixes_2_2
=======
  FHashList := TFPHashList.Create;
  FFreeObjects := Freeobjects;
>>>>>>> origin/fixes_2_2
end;

destructor TFPHashObjectList.Destroy;
begin
  if (FHashList <> nil) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    Clear;
    FHashList.Destroy;
=======
      Clear;
      FHashList.Destroy;
>>>>>>> graemeg/fixes_2_2
=======
      Clear;
      FHashList.Destroy;
>>>>>>> origin/fixes_2_2
    end;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObjectList.Clear;
=======
procedure TFPHashObjectList.Clear;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObjectList.Clear;
>>>>>>> origin/fixes_2_2
var
  i: integer;
begin
  if FFreeObjects then
<<<<<<< HEAD
<<<<<<< HEAD
    for i:=0 to FHashList.Count - 1 do
=======
    for i := 0 to FHashList.Count - 1 do
>>>>>>> graemeg/fixes_2_2
=======
    for i := 0 to FHashList.Count - 1 do
>>>>>>> origin/fixes_2_2
      TObject(FHashList[i]).Free;
  FHashList.Clear;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashObjectList.GetCount: integer;
begin
  Result:=FHashList.Count;
end;

Procedure TFPHashObjectList.SetCount(const AValue: integer);
begin
  if FHashList.Count <> AValue then
    FHashList.Count:=AValue;
end;

Function TFPHashObjectList.GetItem(Index: Integer): TObject;
begin
  Result:=TObject(FHashList[Index]);
end;

Procedure TFPHashObjectList.SetItem(Index: Integer; AObject: TObject);
begin
  if OwnsObjects then
    TObject(FHashList[Index]).Free;
  FHashList[Index]:=AObject;
end;

Procedure TFPHashObjectList.SetCapacity(NewCapacity: Integer);
begin
  FHashList.Capacity:=NewCapacity;
end;

Function TFPHashObjectList.GetCapacity: integer;
begin
  Result:=FHashList.Capacity;
end;

Function TFPHashObjectList.Add(const AName:shortstring;AObject: TObject): Integer;
begin
  Result:=FHashList.Add(AName,AObject);
end;

Function TFPHashObjectList.NameOfIndex(Index: Integer): shortstring;
begin
  Result:=FHashList.NameOfIndex(Index);
end;

Function TFPHashObjectList.HashOfIndex(Index: Integer): LongWord;
begin
  Result:=FHashList.HashOfIndex(Index);
end;

Function TFPHashObjectList.GetNextCollision(Index: Integer): Integer;
begin
  Result:=FHashList.GetNextCollision(Index);
end;

Procedure TFPHashObjectList.Delete(Index: Integer);
=======
=======
>>>>>>> origin/fixes_2_2
function TFPHashObjectList.GetCount: integer;
begin
  Result := FHashList.Count;
end;

procedure TFPHashObjectList.SetCount(const AValue: integer);
begin
  if FHashList.Count <> AValue then
    FHashList.Count := AValue;
end;

function TFPHashObjectList.GetItem(Index: Integer): TObject;
begin
  Result := TObject(FHashList[Index]);
end;

procedure TFPHashObjectList.SetItem(Index: Integer; AObject: TObject);
begin
  if OwnsObjects then
    TObject(FHashList[Index]).Free;
  FHashList[index] := AObject;
end;

procedure TFPHashObjectList.SetCapacity(NewCapacity: Integer);
begin
  FHashList.Capacity := NewCapacity;
end;

function TFPHashObjectList.GetCapacity: integer;
begin
  Result := FHashList.Capacity;
end;

function TFPHashObjectList.Add(const AName:shortstring;AObject: TObject): Integer;
begin
  Result := FHashList.Add(AName,AObject);
end;

function TFPHashObjectList.NameOfIndex(Index: Integer): shortstring;
begin
  Result := FHashList.NameOfIndex(Index);
end;

function TFPHashObjectList.HashOfIndex(Index: Integer): LongWord;
begin
  Result := FHashList.HashOfIndex(Index);
end;

procedure TFPHashObjectList.Delete(Index: Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if OwnsObjects then
    TObject(FHashList[Index]).Free;
  FHashList.Delete(Index);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashObjectList.Expand: TFPHashObjectList;
begin
  FHashList.Expand;
  Result:=Self;
end;

Function TFPHashObjectList.Extract(Item: TObject): TObject;
begin
  Result:=TObject(FHashList.Extract(Item));
end;

Function TFPHashObjectList.Remove(AObject: TObject): Integer;
begin
  Result:=IndexOf(AObject);
  if (Result <> -1) then
    begin
    if OwnsObjects then
      TObject(FHashList[Result]).Free;
    FHashList.Delete(Result);
    end;
end;

Function TFPHashObjectList.IndexOf(AObject: TObject): Integer;
begin
  Result:=FHashList.IndexOf(Pointer(AObject));
end;


Function TFPHashObjectList.Find(const s:shortstring): TObject;
begin
  Result:=TObject(FHashList.Find(s));
end;


Function TFPHashObjectList.FindIndexOf(const s:shortstring): Integer;
begin
  Result:=FHashList.FindIndexOf(s);
end;


Function TFPHashObjectList.FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
=======
=======
>>>>>>> origin/fixes_2_2
function TFPHashObjectList.Expand: TFPHashObjectList;
begin
  FHashList.Expand;
  Result := Self;
end;

function TFPHashObjectList.Extract(Item: TObject): TObject;
begin
  Result := TObject(FHashList.Extract(Item));
end;

function TFPHashObjectList.Remove(AObject: TObject): Integer;
begin
  Result := IndexOf(AObject);
  if (Result <> -1) then
    begin
      if OwnsObjects then
        TObject(FHashList[Result]).Free;
      FHashList.Delete(Result);
    end;
end;

function TFPHashObjectList.IndexOf(AObject: TObject): Integer;
begin
  Result := FHashList.IndexOf(Pointer(AObject));
end;


function TFPHashObjectList.Find(const s:shortstring): TObject;
begin
  result:=TObject(FHashList.Find(s));
end;


function TFPHashObjectList.FindIndexOf(const s:shortstring): Integer;
begin
  result:=FHashList.FindIndexOf(s);
end;


function TFPHashObjectList.FindWithHash(const AName:shortstring;AHash:LongWord): Pointer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Result:=TObject(FHashList.FindWithHash(AName,AHash));
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashObjectList.Rename(const AOldName,ANewName:shortstring): Integer;
=======
function TFPHashObjectList.Rename(const AOldName,ANewName:shortstring): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashObjectList.Rename(const AOldName,ANewName:shortstring): Integer;
>>>>>>> origin/fixes_2_2
begin
  Result:=FHashList.Rename(AOldName,ANewName);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPHashObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt : Integer): Integer;
=======
function TFPHashObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt : Integer): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TFPHashObjectList.FindInstanceOf(AClass: TClass; AExact: Boolean; AStartAt : Integer): Integer;
>>>>>>> origin/fixes_2_2
var
  I : Integer;
begin
  I:=AStartAt;
  Result:=-1;
<<<<<<< HEAD
<<<<<<< HEAD
  if AExact then
    while (I<Count) and (Result=-1) do
      if Items[i].ClassType=AClass then
=======
  If AExact then
    while (I<Count) and (Result=-1) do
      If Items[i].ClassType=AClass then
>>>>>>> graemeg/fixes_2_2
=======
  If AExact then
    while (I<Count) and (Result=-1) do
      If Items[i].ClassType=AClass then
>>>>>>> origin/fixes_2_2
        Result:=I
      else
        Inc(I)
  else
    while (I<Count) and (Result=-1) do
<<<<<<< HEAD
<<<<<<< HEAD
      if Items[i].InheritsFrom(AClass) then
=======
      If Items[i].InheritsFrom(AClass) then
>>>>>>> graemeg/fixes_2_2
=======
      If Items[i].InheritsFrom(AClass) then
>>>>>>> origin/fixes_2_2
        Result:=I
      else
        Inc(I);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObjectList.Pack;
=======
procedure TFPHashObjectList.Pack;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObjectList.Pack;
>>>>>>> origin/fixes_2_2
begin
  FHashList.Pack;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObjectList.ShowStatistics;
=======
procedure TFPHashObjectList.ShowStatistics;
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObjectList.ShowStatistics;
>>>>>>> origin/fixes_2_2
begin
  FHashList.ShowStatistics;
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObjectList.ForEachCall(proc2call:TObjectListCallback;arg:pointer);
=======
procedure TFPHashObjectList.ForEachCall(proc2call:TObjectListCallback;arg:pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObjectList.ForEachCall(proc2call:TObjectListCallback;arg:pointer);
>>>>>>> origin/fixes_2_2
begin
  FHashList.ForEachCall(TListCallBack(proc2call),arg);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPHashObjectList.ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
=======
procedure TFPHashObjectList.ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPHashObjectList.ForEachCall(proc2call:TObjectListStaticCallback;arg:pointer);
>>>>>>> origin/fixes_2_2
begin
  FHashList.ForEachCall(TListStaticCallBack(proc2call),arg);
end;


{ ---------------------------------------------------------------------
    Hash support, by Dean Zobec
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
{ Default hash Function }

Function RSHash(const S: string; const TableSize: Longword): Longword;
=======
{ Default hash function }

function RSHash(const S: string; const TableSize: Longword): Longword;
>>>>>>> graemeg/fixes_2_2
=======
{ Default hash function }

function RSHash(const S: string; const TableSize: Longword): Longword;
>>>>>>> origin/fixes_2_2
const
  b = 378551;
var
  a: Longword;
  i: Longword;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  a:=63689;
  Result:=0;
  if length(s)>0 then
    for i:=1 to Length(S) do
      begin
      Result:=Result * a + Ord(S[i]);
      a:=a * b;
      end;
  Result:=(Result and $7FFFFFFF) mod TableSize;
=======
=======
>>>>>>> origin/fixes_2_2
 a := 63689;
 Result := 0;
 if length(s)>0 then
   for i := 1 to Length(S) do
   begin
     Result := Result * a + Ord(S[i]);
     a := a * b;
   end;
 Result := (Result and $7FFFFFFF) mod TableSize;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ THTNode }

constructor THTCustomNode.CreateWith(const AString: string);
begin
  inherited Create;
<<<<<<< HEAD
<<<<<<< HEAD
  FKey:=AString;
end;

Function THTCustomNode.HasKey(const AKey: string): boolean;
begin
  if Length(AKey) <> Length(FKey) then
    begin
    Result:=false;
    Exit;
    end
  else
    Result:=CompareMem(PChar(FKey), PChar(AKey), Length(AKey));
=======
=======
>>>>>>> origin/fixes_2_2
  FKey := AString;
end;

function THTCustomNode.HasKey(const AKey: string): boolean;
begin
  if Length(AKey) <> Length(FKey) then
  begin
    Result := false;
    exit;
  end
  else
    Result := CompareMem(PChar(FKey), PChar(AKey), length(AKey));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ TFPCustomHashTable }

constructor TFPCustomHashTable.Create;
begin
  CreateWith(196613,@RSHash);
end;

constructor TFPCustomHashTable.CreateWith(AHashTableSize: Longword;
  aHashFunc: THashFunction);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Create;
  FHashTable:=TFPObjectList.Create(True);
  HashTableSize:=AHashTableSize;
  FHashFunction:=aHashFunc;
=======
=======
>>>>>>> origin/fixes_2_2
  Inherited Create;
  FHashTable := TFPObjectList.Create(True);
  HashTableSize := AHashTableSize;
  FHashFunction := aHashFunc;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TFPCustomHashTable.Destroy;
begin
  FHashTable.Free;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPCustomHashTable.GetDensity: Longword;
begin
  Result:=FHashTableSize - VoidSlots
end;

Function TFPCustomHashTable.GetNumberOfCollisions: Longword;
begin
  Result:=FCount -(FHashTableSize - VoidSlots)
end;

Procedure TFPCustomHashTable.SetHashTableSize(const Value: Longword);
=======
=======
>>>>>>> origin/fixes_2_2
function TFPCustomHashTable.GetDensity: Longword;
begin
  Result := FHashTableSize - VoidSlots
end;

function TFPCustomHashTable.GetNumberOfCollisions: Longword;
begin
  Result := FCount -(FHashTableSize - VoidSlots)
end;

procedure TFPCustomHashTable.SetHashTableSize(const Value: Longword);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  i: Longword;
  newSize: Longword;
begin
  if Value <> FHashTableSize then
<<<<<<< HEAD
<<<<<<< HEAD
    begin
    i:=0;
    while (PRIMELIST[i] < Value) and (i < 27) do
     Inc(i);
    newSize:=PRIMELIST[i];
    if Count = 0 then
      begin
      FHashTableSize:=newSize;
      InitializeHashTable;
      end
    else
      ChangeTableSize(newSize);
    end;
end;

Procedure TFPCustomHashTable.InitializeHashTable;
=======
=======
>>>>>>> origin/fixes_2_2
  begin
    i := 0;
    while (PRIMELIST[i] < Value) and (i < 27) do
     inc(i);
    newSize := PRIMELIST[i];
    if Count = 0 then
    begin
      FHashTableSize := newSize;
      InitializeHashTable;
    end
    else
      ChangeTableSize(newSize);
  end;
end;

procedure TFPCustomHashTable.InitializeHashTable;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  i: LongWord;
begin
  if FHashTableSize>0 Then
<<<<<<< HEAD
<<<<<<< HEAD
    for i:=0 to FHashTableSize-1 do
      FHashTable.Add(nil);
  FCount:=0;
end;

Procedure TFPCustomHashTable.ChangeTableSize(const ANewSize: Longword);
=======
=======
>>>>>>> origin/fixes_2_2
    for i := 0 to FHashTableSize-1 do
     FHashTable.Add(nil);
  FCount := 0;
end;

procedure TFPCustomHashTable.ChangeTableSize(const ANewSize: Longword);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  SavedTable: TFPObjectList;
  SavedTableSize: Longword;
  i, j: Longword;
  temp: THTCustomNode;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  SavedTable:=FHashTable;
  SavedTableSize:=FHashTableSize;
  FHashTableSize:=ANewSize;
  FHashTable:=TFPObjectList.Create(True);
  InitializeHashTable;
  if SavedTableSize>0 Then
    for i:=0 to SavedTableSize-1 do
      if Assigned(SavedTable[i]) then
        for j:=0 to TFPObjectList(SavedTable[i]).Count -1 do
          begin
          temp:=THTCustomNode(TFPObjectList(SavedTable[i])[j]);
          AddNode(temp);
          end;
  SavedTable.Free;
end;

Procedure TFPCustomHashTable.SetHashFunction(AHashFunction: THashFunction);
begin
  if IsEmpty then
    FHashFunction:=AHashFunction
=======
=======
>>>>>>> origin/fixes_2_2
  SavedTable := FHashTable;
  SavedTableSize := FHashTableSize;
  FHashTableSize := ANewSize;
  FHashTable := TFPObjectList.Create(True);
  InitializeHashTable;
  If SavedTableSize>0 Then
    for i := 0 to SavedTableSize-1 do
    begin
      if Assigned(SavedTable[i]) then
      for j := 0 to TFPObjectList(SavedTable[i]).Count -1 do
      begin
        temp := THTCustomNode(TFPObjectList(SavedTable[i])[j]);
        AddNode(temp);
      end;
    end;
  SavedTable.Free;
end;

procedure TFPCustomHashTable.SetHashFunction(AHashFunction: THashFunction);
begin
  if IsEmpty then
    FHashFunction := AHashFunction
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    raise Exception.Create(NotEmptyMsg);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPCustomHashTable.Find(const aKey: string): THTCustomNode;
=======
function TFPCustomHashTable.Find(const aKey: string): THTCustomNode;
>>>>>>> graemeg/fixes_2_2
=======
function TFPCustomHashTable.Find(const aKey: string): THTCustomNode;
>>>>>>> origin/fixes_2_2
var
  hashCode: Longword;
  chn: TFPObjectList;
  i: Longword;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  hashCode:=FHashFunction(aKey, FHashTableSize);
  chn:=Chain(hashCode);
  if Assigned(chn) then
    if chn.count>0 then
      for i:=0 to chn.Count - 1 do
        if THTCustomNode(chn[i]).HasKey(aKey) then
          begin
          Result:=THTCustomNode(chn[i]);
          Exit;
          end;
  Result:=nil;
end;

Function TFPCustomHashTable.FindChainForAdd(Const aKey : String) : TFPObjectList;
var
  hashCode: Longword;
  i: Longword;
begin
  hashCode:=FHashFunction(aKey, FHashTableSize);
  Result:=Chain(hashCode);
  if Assigned(Result)  then
    begin
    if Result.count>0 then
      for i:=0 to Result.Count - 1 do
        if THTCustomNode(Result[i]).HasKey(aKey) then
          raise EDuplicate.CreateFmt(DuplicateMsg, [aKey]);
    end
  else
    begin
    FHashTable[hashcode]:=TFPObjectList.Create(True);
    Result:=Chain(hashCode);
    end;
  Inc(FCount);
end;


Procedure TFPCustomHashTable.Delete(const aKey: string);
=======
=======
>>>>>>> origin/fixes_2_2
  hashCode := FHashFunction(aKey, FHashTableSize);
  chn := Chain(hashCode);
  if Assigned(chn) then
  begin
    if chn.count>0 then
     for i := 0 to chn.Count - 1 do
      if THTCustomNode(chn[i]).HasKey(aKey) then
      begin
        result := THTCustomNode(chn[i]);
        exit;
      end;
  end;
  Result := nil;
end;

Function TFPCustomHashTable.FindChainForAdd(Const aKey : String) : TFPObjectList;

var
  hashCode: Longword;
  i: Longword;

begin
  hashCode := FHashFunction(aKey, FHashTableSize);
  Result := Chain(hashCode);
  if Assigned(Result)  then
    begin
    if Result.count>0 then
      for i := 0 to Result.Count - 1 do
        if THTCustomNode(Result[i]).HasKey(aKey) then
          Raise EDuplicate.CreateFmt(DuplicateMsg, [aKey]);
    end
  else
    begin
    FHashTable[hashcode] := TFPObjectList.Create(true);
    Result := Chain(hashcode);
    end;
  inc(FCount);
end;


procedure TFPCustomHashTable.Delete(const aKey: string);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  hashCode: Longword;
  chn: TFPObjectList;
  i: Longword;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  hashCode:=FHashFunction(aKey, FHashTableSize);
  chn:=Chain(hashCode);
  if Assigned(chn) then
    if chn.count>0 then
      for i:=0 to chn.Count - 1 do
        if THTCustomNode(chn[i]).HasKey(aKey) then
          begin
          chn.Delete(i);
          dec(FCount);
          Exit;
          end;
end;

Function TFPCustomHashTable.IsEmpty: boolean;
begin
  Result:=(FCount = 0);
end;

Function TFPCustomHashTable.Chain(const index: Longword): TFPObjectList;
begin
  Result:=TFPObjectList(FHashTable[index]);
end;

Function TFPCustomHashTable.GetVoidSlots: Longword;
=======
=======
>>>>>>> origin/fixes_2_2
  hashCode := FHashFunction(aKey, FHashTableSize);
  chn := Chain(hashCode);
  if Assigned(chn) then
  begin
    if chn.count>0 then
    for i := 0 to chn.Count - 1 do
      if THTCustomNode(chn[i]).HasKey(aKey) then
      begin
        chn.Delete(i);
        dec(FCount);
        exit;
      end;
  end;
  raise EKeyNotFound.CreateFmt(KeyNotFoundMsg, ['Delete', aKey]);
end;

function TFPCustomHashTable.IsEmpty: boolean;
begin
  Result := (FCount = 0);
end;

function TFPCustomHashTable.Chain(const index: Longword): TFPObjectList;
begin
  Result := TFPObjectList(FHashTable[index]);
end;

function TFPCustomHashTable.GetVoidSlots: Longword;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  i: Longword;
  num: Longword;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  num:=0;
  if FHashTableSize>0 then
    for i:= 0 to FHashTableSize-1 do
      if not Assigned(Chain(i)) then
        Inc(num);
  Result:=num;
end;

Function TFPCustomHashTable.GetLoadFactor: double;
begin
  Result:=Count / FHashTableSize;
end;

Function TFPCustomHashTable.GetAVGChainLen: double;
begin
  Result:=Count / (FHashTableSize - VoidSlots);
end;

Function TFPCustomHashTable.GetMaxChainLength: Longword;
var
  i: Longword;
begin
  Result:=0;
  if FHashTableSize>0 Then
   for i:=0 to FHashTableSize-1 do
      if ChainLength(i) > Result then
        Result:=ChainLength(i);
end;

Function TFPCustomHashTable.FindOrCreateNew(const aKey: string): THTCustomNode;
=======
=======
>>>>>>> origin/fixes_2_2
  num := 0;
  if FHashTableSize>0 Then
    for i:= 0 to FHashTableSize-1 do
      if Not Assigned(Chain(i)) then
        inc(num);
  result := num;
end;

function TFPCustomHashTable.GetLoadFactor: double;
begin
  Result := Count / FHashTableSize;
end;

function TFPCustomHashTable.GetAVGChainLen: double;
begin
  result := Count / (FHashTableSize - VoidSlots);
end;

function TFPCustomHashTable.GetMaxChainLength: Longword;
var
  i: Longword;
begin
  Result := 0;
  if FHashTableSize>0 Then
   for i := 0 to FHashTableSize-1 do
      if ChainLength(i) > Result then
        Result := ChainLength(i);
end;

function TFPCustomHashTable.FindOrCreateNew(const aKey: string): THTCustomNode;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  hashCode: Longword;
  chn: TFPObjectList;
  i: Longword;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  hashCode:=FHashFunction(aKey, FHashTableSize);
  chn:=Chain(hashCode);
  if Assigned(chn)  then
    begin
    if chn.count>0 then
      for i:=0 to chn.Count - 1 do
        if THTCustomNode(chn[i]).HasKey(aKey) then
          begin
          Result:=THTNode(chn[i]);
          Exit;
          end
    end
  else
    begin
    FHashTable[hashcode]:=TFPObjectList.Create(true);
    chn:=Chain(hashcode);
    end;
  Inc(FCount);
  Result:=CreateNewNode(aKey);
  chn.Add(Result);
end;

Function TFPCustomHashTable.ChainLength(const ChainIndex: Longword): Longword;
begin
  if Assigned(Chain(ChainIndex)) then
    Result:=Chain(ChainIndex).Count
  else
    Result:=0;
end;

Procedure TFPCustomHashTable.Clear;
var
  i: Longword;
begin
  if FHashTableSize>0 then
    for i:=0 to FHashTableSize - 1 do
      if Assigned(Chain(i)) then
        Chain(i).Clear;
  FCount:=0;
=======
=======
>>>>>>> origin/fixes_2_2
  hashCode := FHashFunction(aKey, FHashTableSize);
  chn := Chain(hashCode);
  if Assigned(chn)  then
  begin
    if chn.count>0 then
     for i := 0 to chn.Count - 1 do
      if THTCustomNode(chn[i]).HasKey(aKey) then
        begin
          Result := THTNode(chn[i]);
          exit;
        end
  end
  else
    begin
      FHashTable[hashcode] := TFPObjectList.Create(true);
      chn := Chain(hashcode);
    end;
  inc(FCount);
  Result := CreateNewNode(aKey);
  chn.Add(Result);
end;

function TFPCustomHashTable.ChainLength(const ChainIndex: Longword): Longword;
begin
  if Assigned(Chain(ChainIndex)) then
    Result := Chain(ChainIndex).Count
  else
    Result := 0;
end;

procedure TFPCustomHashTable.Clear;
var
  i: Longword;
begin
  if FHashTableSize>0 Then
    for i := 0 to FHashTableSize - 1 do
      begin
        if Assigned(Chain(i)) then
          Chain(i).Clear;
      end;
  FCount := 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;



{ TFPDataHashTable }

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPDataHashTable.Add(const aKey: string; aItem: pointer);
=======
procedure TFPDataHashTable.Add(const aKey: string; aItem: pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TFPDataHashTable.Add(const aKey: string; aItem: pointer);
>>>>>>> origin/fixes_2_2
var
  chn: TFPObjectList;
  NewNode: THtDataNode;
begin
  chn:=FindChainForAdd(akey);
<<<<<<< HEAD
<<<<<<< HEAD
  NewNode:=THtDataNode(CreateNewNode(aKey));
  NewNode.Data:=aItem;
  chn.Add(NewNode);
end;

Function TFPDataHashTable.GetData(const Index: string): Pointer;
var
  node: THTDataNode;
begin
  node:=THTDataNode(Find(Index));
  if Assigned(node) then
    Result:=node.Data
  else
    Result:=nil;
end;

Procedure TFPDataHashTable.SetData(const index: string; const AValue: Pointer);
begin
  THTDataNode(FindOrCreateNew(index)).Data:=AValue;
=======
=======
>>>>>>> origin/fixes_2_2
  NewNode := THtDataNode(CreateNewNode(aKey));
  NewNode.Data := aItem;
  chn.Add(NewNode);
end;

function TFPDataHashTable.GetData(const Index: string): Pointer;
var
  node: THTDataNode;
begin
  node := THTDataNode(Find(Index));
  if Assigned(node) then
    Result := node.Data
  else
    Result := nil;
end;

procedure TFPDataHashTable.SetData(const index: string; const AValue: Pointer);
begin
  THTDataNode(FindOrCreateNew(index)).Data := AValue;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function TFPDataHashTable.CreateNewNode(const aKey : string) : THTCustomNode;

begin
  Result:=THTDataNode.CreateWith(aKey);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPDataHashTable.Iterate(aMethod: TDataIteratorMethod): Pointer;
var
  N : THTDataNode;
begin
  N:=ForEachCall(AMethod);
  if Assigned(N) then
    Result:=N.Data
  else
    Result:=nil;
end;

Function TFPDataHashTable.ForEachCall(aMethod: TDataIteratorMethod): THTDataNode;
var
  i, j: Longword;
  continue: Boolean;
begin
  Result:=nil;
  continue:=true;
  if FHashTableSize>0 then
    for i:=0 to FHashTableSize-1 do
      if Assigned(Chain(i)) then
        if chain(i).count>0 then
          for j:=0 to Chain(i).Count-1 do
            begin
            aMethod(THTDataNode(Chain(i)[j]).Data, THTDataNode(Chain(i)[j]).Key, continue);
            if not continue then
              begin
              Result:=THTDataNode(Chain(i)[j]);
              Exit;
              end;
           end;
end;

Procedure TFPDataHashTable.AddNode(ANode : THTCustomNode);
begin
  with THTDataNode(ANode) do
=======
=======
>>>>>>> origin/fixes_2_2
function TFPDataHashTable.ForEachCall(aMethod: TDataIteratorMethod): THTDataNode;
var
  i, j: Longword;
  continue: boolean;
begin
  Result := nil;
  continue := true;
  if FHashTableSize>0 then
   for i := 0 to FHashTableSize-1 do
    begin
      if assigned(Chain(i)) then
      begin
       if chain(i).count>0 then
        for j := 0 to Chain(i).Count-1 do
        begin
          aMethod(THTDataNode(Chain(i)[j]).Data, THTDataNode(Chain(i)[j]).Key, continue);
          if not continue then
          begin
            Result := THTDataNode(Chain(i)[j]);
            Exit;
          end;
        end;
      end;
    end;
end;

Procedure TFPDataHashTable.AddNode(ANode : THTCustomNode);

begin
  With THTDataNode(ANode) do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Add(Key,Data);
end;

{ TFPStringHashTable }

Procedure TFPStringHashTable.AddNode(ANode : THTCustomNode);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  with THTStringNode(ANode) do
    Add(Key,Data);
end;

Function TFPStringHashTable.GetData(const Index: string): String;
var
  node: THTStringNode;
begin
  node:=THTStringNode(Find(Index));
  if Assigned(node) then
    Result:=node.Data
  else
    Result:='';
end;

Procedure TFPStringHashTable.SetData(const index, AValue: string);
begin
  THTStringNode(FindOrCreateNew(index)).Data:=AValue;
end;

Procedure TFPStringHashTable.Add(const aKey, aItem: string);
var
  chn: TFPObjectList;
  NewNode: THtStringNode;
begin
  chn:=FindChainForAdd(akey);
  NewNode:=THtStringNode(CreateNewNode(aKey));
  NewNode.Data:=aItem;
=======
=======
>>>>>>> origin/fixes_2_2

begin
  With THTStringNode(ANode) do
    Add(Key,Data);
end;

function TFPStringHashTable.GetData(const Index: string): String;
var
  node: THTStringNode;
begin
  node := THTStringNode(Find(Index));
  if Assigned(node) then
    Result := node.Data
  else
    Result := '';
end;

procedure TFPStringHashTable.SetData(const index, AValue: string);
begin
  THTStringNode(FindOrCreateNew(index)).Data := AValue;
end;

procedure TFPStringHashTable.Add(const aKey, aItem: string);
var
  chn: TFPObjectList;
  NewNode: THtStringNode;

begin
  chn:=FindChainForAdd(akey);
  NewNode := THtStringNode(CreateNewNode(aKey));
  NewNode.Data := aItem;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  chn.Add(NewNode);
end;

Function TFPStringHashTable.CreateNewNode(const aKey : string) : THTCustomNode;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
begin
  Result:=THTStringNode.CreateWith(aKey);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPStringHashTable.Iterate(aMethod: TStringIteratorMethod): String;
var
  N : THTStringNode;
begin
  N:=ForEachCall(AMethod);
  if Assigned(N) then
    Result:=N.Data
  else
    Result:='';
end;

Function TFPStringHashTable.ForEachCall(aMethod: TStringIteratorMethod): THTStringNode;
=======

function TFPStringHashTable.ForEachCall(aMethod: TStringIteratorMethod): THTStringNode;
>>>>>>> graemeg/fixes_2_2
=======

function TFPStringHashTable.ForEachCall(aMethod: TStringIteratorMethod): THTStringNode;
>>>>>>> origin/fixes_2_2
var
  i, j: Longword;
  continue: boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=nil;
  continue:=True;
  if FHashTableSize>0 then
    for i:=0 to FHashTableSize-1 do
      if Assigned(Chain(i)) then
        if chain(i).Count>0 then
          for j:=0 to Chain(i).Count-1 do
            begin
            aMethod(THTStringNode(Chain(i)[j]).Data, THTStringNode(Chain(i)[j]).Key, continue);
            if not continue then
              begin
              Result:=THTStringNode(Chain(i)[j]);
              Exit;
              end;
            end;
=======
=======
>>>>>>> origin/fixes_2_2
  Result := nil;
  continue := true;
  if FHashTableSize>0 then
   for i := 0 to FHashTableSize-1 do
    begin
      if assigned(Chain(i)) then
      begin
       if chain(i).count>0 then
        for j := 0 to Chain(i).Count-1 do
        begin
          aMethod(THTStringNode(Chain(i)[j]).Data, THTStringNode(Chain(i)[j]).Key, continue);
          if not continue then
          begin
            Result := THTStringNode(Chain(i)[j]);
            Exit;
          end;
        end;
      end;
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ TFPObjectHashTable }

Procedure TFPObjectHashTable.AddNode(ANode : THTCustomNode);
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
begin
  With THTObjectNode(ANode) do
    Add(Key,Data);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TFPObjectHashTable.GetData(const Index: string): TObject;
var
  node: THTObjectNode;
begin
  node:=THTObjectNode(Find(Index));
  if Assigned(node) then
    Result:=node.Data
  else
    Result:=nil;
end;

Procedure TFPObjectHashTable.SetData(const index : string; AObject : TObject);
begin
  THTObjectNode(FindOrCreateNew(index)).Data:=AObject;
end;

Procedure TFPObjectHashTable.Add(const aKey: string; AItem : TObject);
var
  chn: TFPObjectList;
  NewNode: THTObjectNode;
begin
  chn:=FindChainForAdd(akey);
  NewNode:=THTObjectNode(CreateNewNode(aKey));
  NewNode.Data:=aItem;
=======
=======
>>>>>>> origin/fixes_2_2
function TFPObjectHashTable.GetData(const Index: string): TObject;
var
  node: THTObjectNode;
begin
  node := THTObjectNode(Find(Index));
  if Assigned(node) then
    Result := node.Data
  else
    Result := Nil;
end;

procedure TFPObjectHashTable.SetData(const index : string; AObject : TObject);
begin
  THTObjectNode(FindOrCreateNew(index)).Data := AObject;
end;

procedure TFPObjectHashTable.Add(const aKey: string; AItem : TObject);
var
  chn: TFPObjectList;
  NewNode: THTObjectNode;

begin
  chn:=FindChainForAdd(akey);
  NewNode := THTObjectNode(CreateNewNode(aKey));
  NewNode.Data := aItem;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  chn.Add(NewNode);
end;

Function TFPObjectHashTable.CreateNewNode(const aKey : string) : THTCustomNode;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  if OwnsObjects then
=======

begin
  If OwnsObjects then
>>>>>>> graemeg/fixes_2_2
=======

begin
  If OwnsObjects then
>>>>>>> origin/fixes_2_2
    Result:=THTOwnedObjectNode.CreateWith(aKey)
  else
    Result:=THTObjectNode.CreateWith(aKey);
end;


<<<<<<< HEAD
<<<<<<< HEAD
Function TFPObjectHashTable.Iterate(aMethod: TObjectIteratorMethod): TObject;
var
  N : THTObjectNode;
begin
  N:=ForEachCall(AMethod);
  if Assigned(N) then
    Result:=N.Data
  else
    Result:=nil;
end;

Function TFPObjectHashTable.ForEachCall(aMethod: TObjectIteratorMethod): THTObjectNode;
=======
function TFPObjectHashTable.ForEachCall(aMethod: TObjectIteratorMethod): THTObjectNode;
>>>>>>> graemeg/fixes_2_2
=======
function TFPObjectHashTable.ForEachCall(aMethod: TObjectIteratorMethod): THTObjectNode;
>>>>>>> origin/fixes_2_2
var
  i, j: Longword;
  continue: boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=nil;
  continue:=true;
  if FHashTableSize>0 then
    for i:=0 to FHashTableSize-1 do
      if Assigned(Chain(i)) then
        if Chain(i).Count>0 then
          for j:=0 to Chain(i).Count-1 do
           begin
           aMethod(THTObjectNode(Chain(i)[j]).Data, THTObjectNode(Chain(i)[j]).Key, continue);
           if not continue then
             begin
             Result:=THTObjectNode(Chain(i)[j]);
             Exit;
             end;
           end;
end;

constructor TFPObjectHashTable.Create(AOwnsObjects : Boolean = True);
begin
  inherited Create;
=======
=======
>>>>>>> origin/fixes_2_2
  Result := nil;
  continue := true;
  if FHashTableSize>0 then
   for i := 0 to FHashTableSize-1 do
    begin
      if assigned(Chain(i)) then
      begin
       if chain(i).count>0 then
        for j := 0 to Chain(i).Count-1 do
        begin
          aMethod(THTObjectNode(Chain(i)[j]).Data, THTObjectNode(Chain(i)[j]).Key, continue);
          if not continue then
          begin
            Result := THTObjectNode(Chain(i)[j]);
            Exit;
          end;
        end;
      end;
    end;
end;

constructor TFPObjectHashTable.Create(AOwnsObjects : Boolean = True);

begin
  Inherited Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FOwnsObjects:=AOwnsObjects;
end;

constructor TFPObjectHashTable.CreateWith(AHashTableSize: Longword; aHashFunc: THashFunction; AOwnsObjects : Boolean = True);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  inherited CreateWith(AHashTableSize,AHashFunc);
  FOwnsObjects:=AOwnsObjects;
end;

destructor THTOwnedObjectNode.Destroy;
begin
  FreeAndNil(FData);
  inherited;
=======
=======
>>>>>>> origin/fixes_2_2

begin
  Inherited CreateWith(AHashTableSize,AHashFunc);
  FOwnsObjects:=AOwnsObjects;
end;

Destructor THTOwnedObjectNode.Destroy;

begin
  FreeAndNil(FData);
  Inherited;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{ TCustomBucketList }

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.GetData(AItem: Pointer): Pointer;
var
  B,I : Integer;
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomBucketList.GetData(AItem: Pointer): Pointer;

Var
  B,I : Integer;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  GetBucketItem(AItem,B,I);
  Result:=FBuckets[B].Items[I].Data;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.GetBucketCount: Integer;
=======
function TCustomBucketList.GetBucketCount: Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TCustomBucketList.GetBucketCount: Integer;
>>>>>>> origin/fixes_2_2
begin
  Result:=Length(FBuckets);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TCustomBucketList.SetData(AItem: Pointer; const AData: Pointer);
var
  B,I : Integer;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomBucketList.SetData(AItem: Pointer; const AData: Pointer);

Var
  B,I : Integer;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  GetBucketItem(AItem,B,I);
  FBuckets[B].Items[I].Data:=AData;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TCustomBucketList.SetBucketCount(const Value: Integer);
begin
  if (Value<>GetBucketCount) then
    SetLength(FBuckets,Value);
end;

Procedure TCustomBucketList.GetBucketItem(AItem: Pointer; out ABucket,
  AIndex: Integer);
begin
  if not FindItem(AItem,ABucket,AIndex) then
    Error(SErrNoSuchItem,[AItem]);
end;

Function TCustomBucketList.AddItem(ABucket: Integer; AItem, AData: Pointer
  ): Pointer;
var
  B : PBucket;
  L : Integer;
begin
  B:=@FBuckets[ABucket];
  L:=Length(B^.Items);
  if (B^.Count=L) then
    begin
    if L<8 then
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomBucketList.SetBucketCount(const Value: Integer);

begin
  If (Value<>GetBucketCount) then
    SetLength(FBuckets,Value);
end;

procedure TCustomBucketList.GetBucketItem(AItem: Pointer; out ABucket,
  AIndex: Integer);
begin
  If Not FindItem(AItem,ABucket,AIndex) then
    Error(SErrNoSuchItem,[AItem]);
end;

function TCustomBucketList.AddItem(ABucket: Integer; AItem, AData: Pointer
  ): Pointer;
  
Var
  B : PBucket;
  L : Integer;
  
begin
  B:=@FBuckets[ABucket];
  L:=Length(B^.Items);
  If (B^.Count=L) then
    begin
    If L<8 then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      L:=8
    else
      L:=L+L div 2;
    SetLength(B^.Items,L);
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  with B^ do
=======
  With B^ do
>>>>>>> graemeg/fixes_2_2
=======
  With B^ do
>>>>>>> origin/fixes_2_2
    begin
    Items[Count].Item:=AItem;
    Items[Count].Data:=AData;
    Result:=AData;
    Inc(Count);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.DeleteItem(ABucket: Integer; AIndex: Integer): Pointer;
var
  B : PBucket;
  L : Integer;
begin
  B:=@FBuckets[ABucket];
  Result:=B^.Items[AIndex].Data;
  if B^.Count=1 then
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomBucketList.DeleteItem(ABucket: Integer; AIndex: Integer): Pointer;
  
Var
  B : PBucket;
  L : Integer;
  
begin
  B:=@FBuckets[ABucket];
  Result:=B^.Items[Aindex].Data;
  If B^.Count=1 then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    SetLength(B^.Items,0)
  else
    begin
    L:=(B^.Count-AIndex-1);// No point in moving if last one...
<<<<<<< HEAD
<<<<<<< HEAD
    if L>0 then
=======
    If L>0 then
>>>>>>> graemeg/fixes_2_2
=======
    If L>0 then
>>>>>>> origin/fixes_2_2
      Move(B^.Items[AIndex+1],B^.Items[AIndex],L*SizeOf(TBucketItem));
    end;
  Dec(B^.Count);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TCustomBucketList.Error(Msg: String; Args: array of const);
begin
  raise ElistError.CreateFmt(Msg,Args);
end;

Function TCustomBucketList.FindItem(AItem: Pointer; out ABucket, AIndex: Integer
  ): Boolean;
var
  I : Integer;
  B : TBucket;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomBucketList.Error(Msg: String; Args: array of const);
begin
  Raise ElistError.CreateFmt(Msg,Args);
end;

function TCustomBucketList.FindItem(AItem: Pointer; out ABucket, AIndex: Integer
  ): Boolean;
  
Var
  I : Integer;
  B : TBucket;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  ABucket:=BucketFor(AItem);
  B:=FBuckets[ABucket];
  I:=B.Count-1;
<<<<<<< HEAD
<<<<<<< HEAD
  while (I>=0) and (B.Items[I].Item<>AItem) do
    Dec(I);
  Result:=I>=0;
  if Result then
=======
=======
>>>>>>> origin/fixes_2_2
  While (I>=0) And (B.Items[I].Item<>AItem) do
    Dec(I);
  Result:=I>=0;
  If Result then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    AIndex:=I;
end;

destructor TCustomBucketList.Destroy;
begin
  Clear;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TCustomBucketList.Clear;
var
  B : TBucket;
  I,J : Integer;
begin
  for I:=0 to Length(FBuckets)-1 do
    begin
    B:=FBuckets[I];
    for J:=B.Count-1 downto 0 do
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomBucketList.Clear;

Var
  B : TBucket;
  I,J : Integer;

begin
  For I:=0 to Length(FBuckets)-1 do
    begin
    B:=FBuckets[I];
    For J:=B.Count-1 downto 0 do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      DeleteItem(I,J);
    end;
  SetLength(FBuckets,0);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.Add(AItem, AData: Pointer): Pointer;
var
  B,I : Integer;
begin
  if FindItem(AItem,B,I) then
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomBucketList.Add(AItem, AData: Pointer): Pointer;

Var
  B,I : Integer;

begin
  If FindItem(AItem,B,I) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Error(SDuplicateItem,[AItem]);
  Result:=AddItem(B,AItem,AData);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TCustomBucketList.Assign(AList: TCustomBucketList);
var
  I,J : Integer;
begin
  Clear;
  SetLength(FBuckets,Length(Alist.FBuckets));
  for I:=0 to BucketCount-1 do
    begin
    SetLength(FBuckets[i].Items,Length(AList.Fbuckets[I].Items));
    for J:=0 to AList.Fbuckets[I].Count-1 do
      with AList.Fbuckets[I].Items[J] do
=======
=======
>>>>>>> origin/fixes_2_2
procedure TCustomBucketList.Assign(AList: TCustomBucketList);

Var
  I,J : Integer;

begin
  Clear;
  SetLength(FBuckets,Length(Alist.FBuckets));
  For I:=0 to BucketCount-1 do
    begin
    SetLength(FBuckets[i].Items,Length(AList.Fbuckets[I].Items));
    For J:=0 to AList.Fbuckets[I].Count-1 do
      With AList.Fbuckets[I].Items[J] do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        AddItem(I,Item,Data);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.Exists(AItem: Pointer): Boolean;
var
  B,I : Integer;
begin
  Result:=FindItem(AItem,B,I);
end;

Function TCustomBucketList.Find(AItem: Pointer; out AData: Pointer): Boolean;
var
  B,I : integer;
begin
  Result:=FindItem(AItem,B,I);
  if Result then
    AData:=FBuckets[B].Items[I].Data;
end;

Function TCustomBucketList.ForEach(AProc: TBucketProc; AInfo: Pointer
  ): Boolean;
var
  I,J,S : Integer;
  Bu : TBucket;
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomBucketList.Exists(AItem: Pointer): Boolean;

Var
  B,I : Integer;

begin
  Result:=FindItem(Aitem,B,I);
end;

function TCustomBucketList.Find(AItem: Pointer; out AData: Pointer): Boolean;

Var
  B,I : integer;

begin
  Result:=FindItem(AItem,B,I);
  If Result then
    AData:=FBuckets[B].Items[I].Data;
end;

function TCustomBucketList.ForEach(AProc: TBucketProc; AInfo: Pointer
  ): Boolean;
  
Var
  I,J,S : Integer;
  Bu : TBucket;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  I:=0;
  Result:=True;
  S:=GetBucketCount;
<<<<<<< HEAD
<<<<<<< HEAD
  while Result and (I<S) do
    begin
    J:=0;
    Bu:=FBuckets[I];
    while Result and (J<Bu.Count) do
      begin
      with Bu.Items[J] do
=======
=======
>>>>>>> origin/fixes_2_2
  While Result and (I<S) do
    begin
    J:=0;
    Bu:=FBuckets[I];
    While Result and (J<Bu.Count) do
      begin
      With Bu.Items[J] do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        AProc(AInfo,Item,Data,Result);
      Inc(J);
      end;
    Inc(I);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.ForEach(AProc: TBucketProcObject): Boolean;
var
  I,J,S : Integer;
  Bu : TBucket;
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomBucketList.ForEach(AProc: TBucketProcObject): Boolean;

Var
  I,J,S : Integer;
  Bu : TBucket;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  I:=0;
  Result:=True;
  S:=GetBucketCount;
<<<<<<< HEAD
<<<<<<< HEAD
  while Result and (I<S) do
    begin
    J:=0;
    Bu:=FBuckets[I];
    while Result and (J<Bu.Count) do
      begin
      with Bu.Items[J] do
=======
=======
>>>>>>> origin/fixes_2_2
  While Result and (I<S) do
    begin
    J:=0;
    Bu:=FBuckets[I];
    While Result and (J<Bu.Count) do
      begin
      With Bu.Items[J] do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        AProc(Item,Data,Result);
      Inc(J);
      end;
    Inc(I);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function TCustomBucketList.Remove(AItem: Pointer): Pointer;
var
  B,I : integer;
begin
  if FindItem(AItem,B,I) then
=======
=======
>>>>>>> origin/fixes_2_2
function TCustomBucketList.Remove(AItem: Pointer): Pointer;

Var
  B,I : integer;

begin
  If FindItem(AItem,B,I) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    begin
    Result:=FBuckets[B].Items[I].Data;
    DeleteItem(B,I);
    end
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=nil;
=======
    Result:=Nil;
>>>>>>> graemeg/fixes_2_2
=======
    Result:=Nil;
>>>>>>> origin/fixes_2_2
end;

{ TBucketList }

<<<<<<< HEAD
<<<<<<< HEAD
Function TBucketList.BucketFor(AItem: Pointer): Integer;
=======
function TBucketList.BucketFor(AItem: Pointer): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TBucketList.BucketFor(AItem: Pointer): Integer;
>>>>>>> origin/fixes_2_2
begin
  // Pointers on average have a granularity of 4
  Result:=(PtrInt(AItem) shr 2) and FBucketMask;
end;

constructor TBucketList.Create(ABuckets: TBucketListSizes);
<<<<<<< HEAD
<<<<<<< HEAD
var
  L : Integer;
begin
  inherited Create;
=======
=======
>>>>>>> origin/fixes_2_2

Var
  L : Integer;
  
begin
  Inherited Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  L:=1 shl (Ord(Abuckets)+1);
  SetBucketCount(L);
  FBucketMask:=L-1;
end;

{ TObjectBucketList }

<<<<<<< HEAD
<<<<<<< HEAD
Function TObjectBucketList.GetData(AItem: TObject): TObject;
begin
  Result:=TObject(inherited GetData(AItem));
end;

Procedure TObjectBucketList.SetData(AItem: TObject; const AData: TObject);
begin
  inherited SetData(Pointer(AItem),Pointer(AData));
end;

Function TObjectBucketList.Add(AItem, AData: TObject): TObject;
begin
  Result:=TObject(inherited Add(Pointer(AItem),Pointer(AData)));
end;

Function TObjectBucketList.Remove(AItem: TObject): TObject;
begin
  Result:=TObject(inherited Remove(Pointer(AItem)));
=======
=======
>>>>>>> origin/fixes_2_2
function TObjectBucketList.GetData(AItem: TObject): TObject;
begin
  Result:=TObject(Inherited GetData(AItem));
end;

procedure TObjectBucketList.SetData(AItem: TObject; const AData: TObject);
begin
  Inherited SetData(Pointer(AItem),Pointer(AData));
end;

function TObjectBucketList.Add(AItem, AData: TObject): TObject;
begin
  Result:=TObject(Inherited Add(Pointer(AItem),Pointer(AData)));
end;

function TObjectBucketList.Remove(AItem: TObject): TObject;
begin
  Result:=TObject(Inherited Remove(Pointer(AItem)));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

end.
