unit tb0664;

{$mode delphi}

interface

uses Generics.Defaults, Generics.Collections, SysUtils;


type

  TuList<T> = class
  {$IFDEF FPC}
  type
    TArrayT = Array of T;
  {$ENDIF}
  private
    function GetCapacity: integer;
    procedure SetCapacity(const Value: integer);

  protected
  {$IFDEF FPC}
    FData: TArrayT;
  {$ELSE}
    FData: TArray<T>;
  {$ENDIF}
    FCount : integer;
    function GetItem(Index: Integer): T;
    procedure SetItem(Index: Integer; const Value: T);
  public
    constructor Create; overload;
    constructor Create(const aCapacity: integer); overload;
    function Add(const item: T): integer;
    property Count: integer read FCount;
    property Items[Index: Integer]: T read GetItem write SetItem; default;
    procedure Clear; virtual;
    {$IFDEF FPC}
    function ToArray: TArrayT;
    {$ELSE}
    function ToArray: TArray<T>;
    {$ENDIF}
    procedure TrimExcess;
    property Capacity: integer read GetCapacity write SetCapacity;
  end;

  ListHelper = record
    public
    class procedure Reverse<T>(const List: TuList<T>); static;
    class function ToArray<T>(const List: TuList<T>): TArray<T>; static;
    class procedure Sort<T>(const List: TuList<T>); overload; static;
    class procedure Sort<T>(const List: TuList<T>; AComparer: IComparer<T>); overload; static;
    class procedure StableSort<T>(const List: TuList<T>; AComparer: IComparer<T>); overload; static;
    class function BinarySearch<T>(const List: TuList<T>; const Item: T): Integer; overload; static;
    class function BinarySearch<T>(const List: TuList<T>; const Item: T; AComparer: IComparer<T>): SizeInt; overload; static;
    class procedure Insert<T>(const List: TuList<T>; const index: integer; const Item: T); static;
    class procedure Delete<T>(const List: TuList<T>; const index: integer); static;
    class procedure InsertRange<T>(const List: TuList<T>; const index: integer; const Items: TArray<T>); static;
    class procedure DeleteRange<T>(const List: TuList<T>; const index: integer; const dCount: integer); static;

    class procedure Remove<T>(const List: TuList<T>; const obj: T; AComparer: IComparer<T>); static;
    class procedure AddManyInts(const List: TuList<UInt32>; const obj: UInt32; const aCount: Int32); static;
    class procedure AddArrayByte(const List: TuList<UInt32>; const Source: TArray<Byte>; const aCount: Int32); static;
  end;

  TuObjectList = class(TuList<TObject>)
  private
    FDontFree: Boolean;
  public
    constructor Create(const aCapacity: integer; const OwnsObjects: boolean);
    destructor Destroy; override;
    procedure Clear; override;
    procedure Insert(Index: Integer; const Value: TObject);
    procedure Delete(Index: Integer);
    procedure DeleteAndNotFree(Index: Integer);
    procedure RemoveRange(Index, Count: Integer);
    procedure SetCount(const a: integer);

    function IndexOfPointer(const Value: TObject): Integer;
    procedure RemovePointer(const obj: TObject);
    procedure SetButDontDestroy(const i: integer; const r: TObject);

    property DontFree: Boolean read FDontFree write FDontFree;
  end;

  {TuVListEnumerator<T> = class(TEnumerator<T>)
  private
    FList: TObject; //silly generics don't allow forward decl
    Position: integer;
  protected
    function DoGetCurrent: T; override;
    function DoMoveNext: Boolean; override;
  public
    constructor Create(const aList: TObject);
  end;}

  TuVList<T: class> = class(TuObjectList)
  protected
    function GetItemTyped(Index: Integer): T;
    procedure SetItemTyped(Index: Integer; const Value: T);
  protected
    FComparer: IComparer<T>;
  public
    constructor Create(const aCapacity: integer; const OwnsObjects: boolean); overload;
    constructor Create(const aComparer: IComparer<T>); overload;
    property ItemTyped[Index: Integer]: T read GetItemTyped write SetItemTyped; default;
    //function GetEnumerator: TEnumerator<T>;
    function ToArray: TArray<T>;
    procedure Sort; overload;
    procedure Sort(aComparer: IComparer<T>); overload;
    procedure StableSort(aComparer: IComparer<T>);
    function BinarySearch(const Item: T): Integer; overload;
    function BinarySearch(const Item: T; aComparer: IComparer<T>): Integer; overload;

  end;

    {$ifdef blubb}
  TuStack<T> = class
  private
    FData: TArray<T>;
    FCount: integer;
  public
    procedure Push(const v: T);
    function Pop: T;
    function Peek: T;
    function Count: integer;
    procedure Clear;
  end;

  TuOStack<T: class> = class
  private
    FData: TArray<T>;
    FCount: integer;
  public
    procedure Push(const v: T);
    procedure Pop;
    function Peek: T;
    function Count: integer;
    procedure Clear;
  end;


{  //can't create an instance in XE6 android
   TKeyComparer<K, V> = class(TInterfacedObject, IComparer<TPair<K, V>>)
    function Compare(const Left, Right: TPair<K, V>): Integer;
  end; }

  TuSortedList<K, V> = class
  type
    TPairKV = TPair<K, V>;
  private
    List: TuList<TPairKV>;
    function GetItems(const key: K): V;
    procedure SetItems(const key: K; const value: V);
  protected
    FComparer: IComparer<K>;

  public
    constructor Create; overload;
    constructor Create(const aComparer: IComparer<K>); overload;
    destructor Destroy; override;

    procedure Add(const key: K; const value: V);
    procedure Remove(const key: K);
    function ContainsKey(const key: K): boolean;
    function IndexOfKey(const key: K): integer;

    function TryGetValue(const key: K; out value: V): boolean;

    property Items[const key: K]: V read GetItems write SetItems; default;

    function Count: integer;

    function Values(const index: integer): V;
    function Keys(const index: integer): K;
  end;
    {$endif}

  procedure TrueFree(const obj: TObject); inline;
  procedure FreeObj(var obj); inline;

procedure RaiseArgEx;
function GetNextCapacity(const c: integer): integer;

implementation

//uses b;

procedure TrueFree(const obj: TObject);
begin
{$ifdef AUTOREFCOUNT}
begin
  obj.DisposeOf;
end;
{$else}
begin
  obj.Free;
end;
{$endif}
end;

procedure FreeObj(var obj);
{$ifdef AUTOREFCOUNT}
begin
  if (Pointer(obj) <> nil) then TObject(obj).DisposeOf;

  TObject(obj) := nil;
end;
{$else}
var
  Temp: TObject;
begin
  Temp := TObject(Obj);
  Pointer(Obj) := nil;
  Temp.Free;
end;
{$endif}

procedure RaiseArgEx;
begin
  raise EArgumentOutOfRangeException.Create('Index out of bounds');
end;

function GetNextCapacity(const c: integer): integer;
begin
  if (c = 0) then exit(4);

  if c < 1000 then exit (c * 2);
  exit(c + 1000);
end;

{$RANGECHECKS OFF}
procedure FillDWord(var Dest; Count, Value: UInt32);
{$IFDEF CPUX86}
asm
  XCHG  EDX, ECX
  PUSH  EDI
  MOV   EDI, EAX
  MOV   EAX, EDX
  REP   STOSD
  POP   EDI
end;
{$ELSE}
type
  IntArray = array[0..0] of integer;
  PIntArray = ^IntArray;
var
  i: integer;
  Arr: PIntArray;
begin
  Arr := PIntArray(@Dest);
  for i := 0 to Count - 1 do Arr[i] := Value;

end;
{$ENDIF CPUX86}

{ TuList<T> }
constructor TuList<T>.Create;
begin
  inherited Create;
end;


constructor TuList<T>.Create(const aCapacity: integer);
begin
  inherited Create;
  Capacity := aCapacity;
end;


{$RANGECHECKS OFF}
function TuList<T>.Add(const item: T): integer;
begin
  if FCount >= Length(FData) then
  begin
    SetLength(FData, GetNextCapacity(FCount));
  end;
  FData[FCount] := item;
  Result := FCount;
  inc(FCount);
end;

procedure TuList<T>.Clear;
begin
  FData := nil;
  FCount := 0;
end;

function TuList<T>.GetItem(Index: Integer): T;
begin
  {$R-}
  if (Index >= Count) or (Index < 0)then RaiseArgEx;
  Result := FData[Index];
  {$R+}
end;

procedure TuList<T>.SetItem(Index: Integer; const Value: T);
begin
  {$R-}
  if (Index >= Count) or (Index < 0)then RaiseArgEx;
  FData[Index] := Value;
  {$R+}
end;

{$IFDEF FPC}
function TuList<T>.ToArray: TArrayT;
{$ELSE}
function TuList<T>.ToArray: TArray<T>;
{$ENDIF}
begin
  SetLength(FData, FCount);
  Result := FData; //we won't return a deep copy.
end;

procedure TuList<T>.TrimExcess;
begin
  SetLength(FData, FCount);
end;

function TuList<T>.GetCapacity: integer;
begin
  Result := Length(FData);
end;

procedure TuList<T>.SetCapacity(const Value: integer);
begin
  if Value > FCount then SetLength(FData, value) else SetLength(FData, FCount);
end;

{ TuVList<T> }

function TuVList<T>.BinarySearch(const Item: T): Integer;
begin
  Result := ListHelper.BinarySearch<T>(TuList<T>(self), Item, FComparer);
end;

function TuVList<T>.BinarySearch(const Item: T;
  aComparer: IComparer<T>): Integer;
begin
  Result := ListHelper.BinarySearch<T>(TuList<T>(self), Item, aComparer);
end;

constructor TuVList<T>.Create(const aCapacity: integer;
  const OwnsObjects: boolean);
var
  Tmp: T;
begin
  inherited Create(aCapacity, OwnsObjects);
//  Tmp := Default(T);
//  if Supports(TObject(Tmp), IComparable<T>) then FComparer := TComparableComparer<T>.Create();

end;

constructor TuVList<T>.Create(const aComparer: IComparer<T>);
begin
  inherited Create(4, true);
  FComparer := aComparer;
end;

{function TuVList<T>.GetEnumerator: TEnumerator<T>;
begin
  Result := TuVListEnumerator<T>.Create(self);
end;}

function TuVList<T>.GetItemTyped(Index: Integer): T;
begin
  {$R-}
  if (Index >= Count) or (Index < 0)then RaiseArgEx;
  Result := T(FData[Index]);
  {$R+}
end;

procedure TuVList<T>.SetItemTyped(Index: Integer; const Value: T);
begin
  {$R-}
  if (Index >= Count) or (Index < 0)then RaiseArgEx;
  if not DontFree then TrueFree(FData[Index]);
  FData[Index] := Value;
  {$R+}
end;

procedure TuVList<T>.Sort(aComparer: IComparer<T>);
begin
  ListHelper.Sort<T>(TuList<T>(self), aComparer);
end;

procedure TuVList<T>.StableSort(aComparer: IComparer<T>);
var
  CastData: TArray<T>;
begin
  TrimExcess;
  CastData := TArray<T>(FData);
//  MergeSort.Sort<T>(CastData, aComparer);
end;

procedure TuVList<T>.Sort;
begin
  ListHelper.Sort<T>(TuList<T>(self), FComparer);
end;

function TuVList<T>.ToArray: TArray<T>;
var
  i: integer;
begin
  Result := nil;
  SetLength(Result, FCount);
  for i := 0 to FCount - 1 do Result[i] := T(FData[i]);
    
end;

{ ListHelper }

class function ListHelper.BinarySearch<T>(const List: TuList<T>; const Item: T): Integer;
begin
  Result := BinarySearch<T>(List, Item, TComparer<T>.Default);
end;

class function ListHelper.BinarySearch<T>(const List: TuList<T>; const Item: T;
  AComparer: IComparer<T>): SizeInt;
var
  b: boolean;
begin
  if AComparer = nil then AComparer := TComparer<T>.Default;

  b := {$IFDEF FPC}TArrayHelper<T>.BinarySearch{$ELSE}TArray.BinarySearch<T>{$ENDIF}(List.FData, Item, Result, AComparer, 0, List.Count);
  if not b then Result := not Result;
end;

class procedure ListHelper.Delete<T>(const List: TuList<T>;
  const index: integer);
begin
  if (index > List.Count) or (index < 0) then RaiseArgEx;
  {$IFDEF FPC}
  List[index] := Default(T); //clear strings or interfaces.
  {$ELSE}
  {$if (CompilerVersion = 33.0) and (Defined(MACOS32) or Defined(IOS))} //see https://quality.embarcadero.com/browse/RSB-2792  We don't do it for the future since it should be fixed after RIO (and likely in a RIO ServicePack.)
  Finalize(List.FData[Index]);
  {$else}
  List[index] := Default(T); //clear strings or interfaces.
  {$ifend}
  {$ENDIF}
  if index + 1 < List.Count then
  begin
    Move(List.FData[index + 1], List.FData[index], SizeOf(T) * (List.Count - (index + 1)));
    FillChar(List.FData[List.FCount - 1], SizeOf(T), 0); //avoid having those records finalized
  end;

  dec(List.FCount);
end;

class procedure ListHelper.DeleteRange<T>(const List: TuList<T>; const index,
  dCount: integer);
var
  i: integer;
  remaining, ToClean: integer;
begin
  if dCount = 0 then exit;

  if (index > List.Count) or (index < 0) or (dCount < 0) or (dCount + index > List.Count) then RaiseArgEx;
  for i := 0 to dCount - 1 do List[index + i] := Default(T); //clear strings or interfaces.
  if index + dCount < List.Count then
  begin
    remaining := List.Count - (index + dcount);
    Move(List.FData[index + dCount], List.FData[index], SizeOf(T) * remaining);
    ToClean := remaining; if ToClean > dCount then ToClean := dCount;

    FillChar(List.FData[List.FCount - ToClean], ToClean * SizeOf(T), 0); //avoid having those records finalized
  end;

  dec(List.FCount, dCount);
end;

class procedure ListHelper.Insert<T>(const List: TuList<T>;
  const index: integer; const Item: T);
begin
  if (index > List.Count) or (index < 0) then RaiseArgEx;
  if List.Count >= Length(List.FData) then
  begin
    SetLength(List.FData, GetNextCapacity(List.FCount));
  end;

  if index < List.Count then
  begin
    Move(List.FData[index], List.FData[index + 1], SizeOf(T) * (List.Count - index));
    FillChar(List.FData[index], SizeOf(T), 0); //avoid having those records finalized
  end;

  List.FData[index] := item;
  inc(List.FCount);
end;

class procedure ListHelper.InsertRange<T>(const List: TuList<T>;
  const index: integer; const Items: TArray<T>);
var
  i: integer;
begin
  if Length(Items) = 0 then exit;

  if (index > List.Count) or (index < 0) then RaiseArgEx;
  if List.Count + Length(Items) > Length(List.FData) then
  begin
    SetLength(List.FData, GetNextCapacity(List.FCount + Length(Items)));
  end;

  if index < List.Count then
  begin
    Move(List.FData[index], List.FData[index + Length(Items)], SizeOf(T) * (List.Count - index));
    FillChar(List.FData[index], Length(Items) * SizeOf(T), 0); //avoid having those records finalized
  end;

  for i := 0 to Length(Items) - 1 do List.FData[index + i] := Items[i];
  inc(List.FCount, Length(Items));
end;

class procedure ListHelper.AddManyInts(const List: TuList<UInt32>; const obj: UInt32; const aCount: Int32);
begin
  if (aCount <= 0) then exit;

  if List.Count + aCount  > Length(List.FData) then
  begin
    SetLength(List.FData, GetNextCapacity(List.FCount + aCount));
  end;

  FillDWord(List.FData[List.Count], aCount, Obj);
  inc(List.FCount, aCount);
end;

class procedure ListHelper.AddArrayByte(const List: TuList<UInt32>; const Source: TArray<Byte>; const aCount: Int32);
begin
  if (aCount <= 0) then exit;

  if List.Count + aCount  > Length(List.FData) then
  begin
    SetLength(List.FData, GetNextCapacity(List.FCount + aCount));
  end;

  System.Move(Source[0], List.FData[List.Count], aCount * SizeOf(UInt32));
  inc(List.FCount, aCount);
end;


class procedure ListHelper.Remove<T>(const List: TuList<T>; const obj: T; AComparer: IComparer<T>);
var
  i: Integer;
begin
  if (AComparer = nil) then AComparer := TComparer<T>.Default;

  for i := 0 to List.Count - 1 do
  begin
    if AComparer.Compare(List[i], obj) = 0 then
    begin
      Delete<T>(List, i);
      exit;
    end;

  end;
end;

class procedure ListHelper.Reverse<T>(const List: TuList<T>);
var
  i, k: integer;
  Tmp: T;
begin
  k := List.Count - 1;
  for i := 0 to (List.Count div 2) - 1 do
  begin
    Tmp := List[i];
    List[i] := List[k];
    List[k] := Tmp;
    dec(k);
  end;
end;

class procedure ListHelper.Sort<T>(const List: TuList<T>;
  AComparer: IComparer<T>);
begin
  if AComparer = nil then AComparer := TComparer<T>.Default;
  {$IFDEF FPC}TArrayHelper<T>.Sort{$ELSE}TArray.Sort<T>{$ENDIF}(List.FData, AComparer, 0, List.Count);
end;

class procedure ListHelper.Sort<T>(const List: TuList<T>);
begin
  {$IFDEF FPC}TArrayHelper<T>.Sort{$ELSE}TArray.Sort<T>{$ENDIF}(List.FData, TComparer<T>.Default, 0, List.Count);
end;

class procedure ListHelper.StableSort<T>(const List: TuList<T>;
  AComparer: IComparer<T>);
begin
  if AComparer = nil then AComparer := TComparer<T>.Default;
  List.TrimExcess;
//  MergeSort.Sort<T>(List.FData, AComparer);
end;

class function ListHelper.ToArray<T>(const List: TuList<T>): TArray<T>;
var
  i: Integer;
begin
  SetLength(Result, List.Count);
  for i := 0 to Length(Result) - 1 do
  begin
    Result[i] := List[i];
  end;

end;

{ TuObjectList }

procedure TuObjectList.Clear;
var
  i: integer;
begin
  if not DontFree then for i := 0 to FCount - 1 do TrueFree(FData[i]);
  inherited;
end;

constructor TuObjectList.Create(const aCapacity: integer;
  const OwnsObjects: boolean);
begin
  inherited Create;
  SetLength(FData, aCapacity);
  DontFree := not OwnsObjects;
end;

procedure TuObjectList.Delete(Index: Integer);
begin
  if not DontFree then TrueFree(Self[Index]);
  ListHelper.Delete<TObject>(self, Index);

end;

procedure TuObjectList.DeleteAndNotFree(Index: Integer);
begin
  ListHelper.Delete<TObject>(self, Index);

end;

procedure TuObjectList.RemoveRange(Index, Count: Integer);
var
  i: integer;
begin
  if not DontFree then for i := Index to Index + Count - 1 do TrueFree(Self[i]);
  ListHelper.DeleteRange<TObject>(self, Index, Count);

end;

destructor TuObjectList.Destroy;
begin
  if not DontFree then Clear;

  inherited;
end;

function TuObjectList.IndexOfPointer(const Value: TObject): Integer;
var
  i: Integer;
begin
  for i := 0 to Count - 1 do
    if Items[i] = Value then
      Exit(i);
  Result := -1;
end;
procedure TuObjectList.Insert(Index: Integer; const Value: TObject);
begin
  ListHelper.Insert<TObject>(self, Index, Value);
end;

procedure TuObjectList.RemovePointer(const obj: TObject);
var
  i: integer;
begin
  i := IndexOfPointer(obj);
  if i >= 0 then Delete(i);
end;

procedure TuObjectList.SetButDontDestroy(const i: integer; const r: TObject);
begin
  SetItem(i, r);
end;

procedure TuObjectList.SetCount(const a: integer);
begin
  FCount := a;
  Capacity := a;
end;

{ TuVListEnumerator<T> }
{$ifdef blubb}

constructor TuVListEnumerator<T>.Create(const aList: TObject);
begin
  inherited Create;
  FList := aList;
  Position := -1;
end;

function TuVListEnumerator<T>.DoGetCurrent: T;
begin
  Result := TuList<T>(FList)[Position];
end;

function TuVListEnumerator<T>.DoMoveNext: Boolean;
begin
  if Position >= (TuList<T>(FList)).Count then exit(false);

  inc(Position);
  Result := Position < (TuList<T>(FList)).Count;
end;

{ TuStack<T> }

procedure TuStack<T>.Clear;
begin
  FData := nil;
  FCount := 0;
end;

function TuStack<T>.Count: integer;
begin
  Result := FCount;
end;

function TuStack<T>.Peek: T;
begin
  if (FCount <= 0) then RaiseArgEx;
  Result := FData[FCount - 1];
end;

function TuStack<T>.Pop: T;
begin
  if (FCount <= 0) then RaiseArgEx;
  dec(FCount);
  Result := FData[FCount];
  FData[FCount] := Default(T);
end;

procedure TuStack<T>.Push(const v: T);
begin
  if FCount >= Length(FData) then
  begin
    SetLength(FData, GetNextCapacity(FCount));
  end;
  FData[FCount] := v;
  inc(FCount);
end;

{ TuStack<T> }

procedure TuOStack<T>.Clear;
begin
  FData := nil;
  FCount := 0;
end;

function TuOStack<T>.Count: integer;
begin
  Result := FCount;
end;

function TuOStack<T>.Peek: T;
begin
  if (FCount <= 0) then RaiseArgEx;
  Result := FData[FCount - 1];
end;

procedure TuOStack<T>.Pop;
begin
  if (FCount <= 0) then RaiseArgEx;
  dec(FCount);
  TrueFree(FData[Count]);
  FData[FCount] := Default(T);
end;

procedure TuOStack<T>.Push(const v: T);
begin
  if FCount >= Length(FData) then
  begin
    SetLength(FData, GetNextCapacity(FCount));
  end;
  FData[FCount] := v;
  inc(FCount);
end;

{ TuSortedList<K, V> }

constructor TuSortedList<K, V>.Create;
begin
  Create(TComparer<K>.Default);
end;

constructor TuSortedList<K, V>.Create(const aComparer: IComparer<K>);
begin
  List := TuList<TPairKV>.Create;
  FComparer := aComparer;
end;

destructor TuSortedList<K, V>.Destroy;
begin
  FreeObj(List);

  inherited;
end;

procedure TuSortedList<K, V>.Add(const key: K; const value: V);
var
  index: integer;
begin
  index := IndexOfKey(key);
  if index >= 0 then RaiseArgEx;

  index := not index;
  ListHelper.Insert<TPairKV>(List, index, TPairKV.Create(key, value));
end;

procedure TuSortedList<K, V>.Remove(const key: K);
var
  index: integer;
begin
  index := IndexOfKey(key);
  if index < 0 then RaiseArgEx;

  ListHelper.Delete<TPairKV>(List, index);
end;

function TuSortedList<K, V>.IndexOfKey(const key: K): integer;
//begin
// fails in android /iosdevice XE6  Result := ListHelper.BinarySearch<TPairKV>(List, TPairKV.Create(key, Default(V)), TKeyComparer<K, V>.Create);
var
  L, H: Integer;
  mid, cmp: Integer;
  found: boolean;
begin
  found := false;
  if Count = 0 then exit(not 0);

  L := 0;
  H := Count - 1;
  while L <= H do
  begin
    mid := L + (H - L) shr 1;
    cmp := FComparer.Compare(List[mid].Key, key);
    if cmp < 0 then
      L := mid + 1
    else
    begin
      H := mid - 1;
      if cmp = 0 then
      begin
        found := true;
      end;
    end;
  end;

  Result := L;
  if (not found) then Result := not Result;

end;

function TuSortedList<K, V>.GetItems(const key: K): V;
begin
  Result := List[IndexOfKey(key)].Value;
end;

function TuSortedList<K, V>.Keys(const index: integer): K;
begin
  Result := List[index].Key;
end;

procedure TuSortedList<K, V>.SetItems(const key: K; const value: V);
var
  index: integer;
begin
  index := IndexOfKey(key);
  if (index >= 0) then List[index] := TPairKV.Create(key, value)
  else Add(key, value);
end;

function TuSortedList<K, V>.TryGetValue(const key: K; out value: V): boolean;
var
  i: integer;
begin
  i := IndexOfKey(key);
  if (i < 0) then
  begin
    value := Default(V);
    exit(false);
  end;

  value := List[i].Value;
  Result := true;
end;

function TuSortedList<K, V>.Values(const index: integer): V;
begin
  Result := List[index].Value;
end;

function TuSortedList<K, V>.ContainsKey(const key: K): boolean;
begin
  Result := IndexOfKey(key) >= 0;
end;

function TuSortedList<K, V>.Count: integer;
begin
  Result := List.Count;
end;
{$endif}

(*
{ TKeyComparer<K, V> }

function TKeyComparer<K, V>.Compare(const Left, Right: TPairKV): Integer;
begin
  Result := TComparer<K>.Default.Compare(Left.Key, Right.Key);
end;*)

end.
