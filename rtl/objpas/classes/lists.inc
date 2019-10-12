{
    This file is part of the Free Pascal Run Time Library (rtl)
    Copyright (c) 1999-2005 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$if defined(VER2_0) or not defined(FPC_TESTGENERICS)}

{****************************************************************************}
{*                           TFPListEnumerator                              *}
{****************************************************************************}

constructor TFPListEnumerator.Create(AList: TFPList);
begin
  inherited Create;
  FList := AList;
  FPosition := -1;
end;

function TFPListEnumerator.GetCurrent: Pointer;
begin
  Result := FList[FPosition];
end;

function TFPListEnumerator.MoveNext: Boolean;
begin
  Inc(FPosition);
  Result := FPosition < FList.Count;
end;

{****************************************************************************}
{*                           TFPList                                        *}
{****************************************************************************}

Const
  // Ratio of Pointer and Word Size.
  WordRatio = SizeOf(Pointer) Div SizeOf(Word);

procedure TFPList.RaiseIndexError(Index : Integer);
begin
  // We should be able to remove this, but unfortunately it is marked protected.
  Error(SListIndexError, Index);
end;

Procedure TFPList.CheckIndex(AIndex : Integer);

begin
  If (AIndex < 0) or (AIndex >= FCount) then
    Error(SListIndexError, AIndex); // Don't use RaiseIndexError, exception address will be better if we use error.
end;

function TFPList.Get(Index: Integer): Pointer;
begin
  CheckIndex(Index);
  Result:=FList^[Index];
end;

procedure TFPList.Put(Index: Integer; Item: Pointer);
begin
  CheckIndex(Index);
  Flist^[Index] := Item;
end;

function TFPList.Extract(Item: Pointer): Pointer;
var
  i : Integer;
begin
  i := IndexOf(item);
  if i >= 0 then
   begin
     Result := item;
     Delete(i);
   end
  else
    result := nil;
end;

procedure TFPList.SetCapacity(NewCapacity: Integer);
begin
  If (NewCapacity < FCount) or (NewCapacity > MaxListSize) then
     Error (SListCapacityError, NewCapacity);
  if NewCapacity = FCapacity then
    exit;
  ReallocMem(FList, SizeOf(Pointer)*NewCapacity);
  FCapacity := NewCapacity;
end;

procedure TFPList.SetCount(NewCount: Integer);
begin
  if (NewCount < 0) or (NewCount > MaxListSize)then
    Error(SListCountError, NewCount);
  If NewCount > FCount then
    begin
    If NewCount > FCapacity then
      SetCapacity(NewCount);
    If FCount < NewCount then
      FillWord(Flist^[FCount], (NewCount-FCount) *  WordRatio, 0);
    end;
  FCount := Newcount;
end;

destructor TFPList.Destroy;
begin
  Self.Clear;
  inherited Destroy;
end;

Procedure TFPList.AddList(AList : TFPList);

Var
  I : Integer;

begin
  If (Capacity<Count+AList.Count) then
    Capacity:=Count+AList.Count;
  For I:=0 to AList.Count-1 do
    Add(AList[i]);
end;


function TFPList.Add(Item: Pointer): Integer;
begin
  if FCount = FCapacity then
    Self.Expand;
  FList^[FCount] := Item;
  Result := FCount;
  FCount := FCount + 1;
end;

procedure TFPList.Clear;
begin
  if Assigned(FList) then
  begin
    SetCount(0);
    SetCapacity(0);
    // FList := nil; // Already set by SetCapacity
  end;
end;

procedure TFPList.Delete(Index: Integer);
begin
  CheckIndex(Index);
  FCount := FCount-1;
  System.Move (FList^[Index+1], FList^[Index], (FCount - Index) * SizeOf(Pointer));
  // Shrink the list if appropriate:
  // If capacity>256 and the list is less than a quarter filled,  shrink to 1/2 the size.
  // Shr is used because it is faster than div.
  if (FCapacity > 256) and (FCount < FCapacity shr 2) then
  begin
    FCapacity := FCapacity shr 1;
    ReallocMem(FList, SizeOf(Pointer) * FCapacity);
  end;
end;

class procedure TFPList.Error(const Msg: string; Data: PtrInt);
begin
  Raise EListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;

procedure TFPList.Exchange(Index1, Index2: Integer);
var
  Temp : Pointer;
begin
  CheckIndex(Index1);
  CheckIndex(Index2);
  Temp := FList^[Index1];
  FList^[Index1] := FList^[Index2];
  FList^[Index2] := Temp;
end;

function TFPList.Expand: TFPList;
var
  IncSize : Longint;
begin
  if FCount < FCapacity then exit(self);
  {
    For really big lists, (128Mb elements), increase with fixed amount: 16Mb elements (=1/8th of 128Mb).
    For big lists (8mb elements), increase with 1/8th of the size
    For moderate lists (128 or more, increase with 1/4th the size
    For smaller sizes, increase with 16 or 4
  }
  if FCapacity > 128*1024*1024 then IncSize := 16*1024*1024
  else if FCapacity > 8*1024*1024 then IncSize := FCapacity shr 3
  else if FCapacity > 128 then IncSize := FCapacity shr 2
  else if FCapacity > 8 then IncSize := 16
  else IncSize := 4;
  SetCapacity(FCapacity + IncSize);
  Result := Self;
end;

function TFPList.First: Pointer;
begin
  If FCount = 0 then
    Result := Nil
  else
    Result := Items[0];
end;

function TFPList.GetEnumerator: TFPListEnumerator;
begin
  Result := TFPListEnumerator.Create(Self);
end;

function TFPList.IndexOf(Item: Pointer): Integer;

Var
  C : Integer;

begin
  Result:=0;
  C:=Count;
  while (Result<C) and (Flist^[Result]<>Item) do
    Inc(Result);
  If Result>=C then
    Result:=-1;
end;

function TFPList.IndexOfItem(Item: Pointer; Direction: TDirection): Integer;

begin
  if Direction=fromBeginning then
    Result:=IndexOf(Item)
  else
    begin
    Result:=Count-1;
    while (Result >=0) and (Flist^[Result]<>Item) do
      Result:=Result - 1;
    end;
end;

procedure TFPList.Insert(Index: Integer; Item: Pointer);
begin
  if (Index < 0) or (Index > FCount )then
    Error(SlistIndexError, Index);
  iF FCount = FCapacity then Self.Expand;
  if Index<FCount then
    System.Move(Flist^[Index], Flist^[Index+1], (FCount - Index) * SizeOf(Pointer));
  FList^[Index] := Item;
  FCount := FCount + 1;
end;

function TFPList.Last: Pointer;
begin
{ Wouldn't it be better to return nil if the count is zero ?}
  If FCount = 0 then
    Result := nil
  else
    Result := Items[FCount - 1];
end;

procedure TFPList.Move(CurIndex, NewIndex: Integer);
var
  Temp : Pointer;
begin
  CheckIndex(CurIndex);
  CheckIndex(NewIndex);
  Temp := FList^[CurIndex];
  if NewIndex > CurIndex then
    System.Move(FList^[CurIndex+1], FList^[CurIndex], (NewIndex - CurIndex) * SizeOf(Pointer))
  else
    System.Move(FList^[NewIndex], FList^[NewIndex+1], (CurIndex - NewIndex) * SizeOf(Pointer));
  FList^[NewIndex] := Temp;
end;

function TFPList.Remove(Item: Pointer): Integer;
begin
  Result := IndexOf(Item);
  If Result <> -1 then
    Self.Delete(Result);
end;

procedure TFPList.Pack;
var
  NewCount,
  i : integer;
  pdest,
  psrc : PPointer;
begin
  NewCount:=0;
  psrc:=@FList^[0];
  pdest:=psrc;
  For I:=0 To FCount-1 Do
    begin
      if assigned(psrc^) then
        begin
          pdest^:=psrc^;
          inc(pdest);
          inc(NewCount);
        end;
      inc(psrc);
    end;
  FCount:=NewCount;
end;


procedure TFPList.Sort(Compare: TListSortCompare);
begin
  Sort(Compare, SortBase.DefaultSortingAlgorithm);
end;


procedure TFPList.Sort(Compare: TListSortCompare; SortingAlgorithm: PSortingAlgorithm);
begin
  SortingAlgorithm^.PtrListSorter_NoContextComparer(PPointer(FList), FCount, Compare);
end;


procedure TFPList.Sort(Compare: TListSortComparer_Context; Context: Pointer);
begin
  Sort(Compare, Context, SortBase.DefaultSortingAlgorithm);
end;


procedure TFPList.Sort(Compare: TListSortComparer_Context; Context: Pointer; SortingAlgorithm: PSortingAlgorithm);
begin
  SortingAlgorithm^.PtrListSorter_ContextComparer(PPointer(FList), FCount, Compare, Context);
end;


procedure TFPList.ForEachCall(proc2call:TListCallback;arg:pointer);
var
  i : integer;
  p : pointer;
begin
  For I:=0 To Count-1 Do
    begin
      p:=FList^[i];
      if assigned(p) then
        proc2call(p,arg);
    end;
end;


procedure TFPList.ForEachCall(proc2call:TListStaticCallback;arg:pointer);
var
  i : integer;
  p : pointer;
begin
  For I:=0 To Count-1 Do
    begin
      p:=FList^[i];
      if assigned(p) then
        proc2call(p,arg);
    end;
end;

procedure TFPList.CopyMove (aList : TFPList);
var r : integer;
begin
  Clear;
  for r := 0 to aList.count-1 do
    Add (aList[r]);
end;

procedure TFPList.MergeMove (aList : TFPList);
var r : integer;
begin
  For r := 0 to aList.count-1 do
    if self.indexof(aList[r]) < 0 then
      self.Add (aList[r]);
end;

procedure TFPList.DoCopy(ListA, ListB : TFPList);
begin
  if assigned (ListB) then
    CopyMove (ListB)
  else
    CopyMove (ListA);
end;

procedure TFPList.DoDestUnique(ListA, ListB : TFPList);
  procedure MoveElements (src, dest : TFPList);
  var r : integer;
  begin
    self.clear;
    for r := 0 to src.count-1 do
      if dest.indexof(src[r]) < 0 then
        self.Add (src[r]);
  end;

var dest : TFPList;
begin
  if assigned (ListB) then
    MoveElements (ListB, ListA)
  else
    try
      dest := TFPList.Create;
      dest.CopyMove (self);
      MoveElements (ListA, dest)
    finally
      dest.Free;
    end;
end;

procedure TFPList.DoAnd(ListA, ListB : TFPList);
var r : integer;
begin
  if assigned (ListB) then
    begin
    self.clear;
    for r := 0 to ListA.count-1 do
      if ListB.indexOf (ListA[r]) >= 0 then
        self.Add (ListA[r]);
    end
  else
    begin
    for r := self.Count-1 downto 0 do
      if ListA.indexof (Self[r]) < 0 then
        self.delete (r);
    end;
end;

procedure TFPList.DoSrcUnique(ListA, ListB : TFPList);
var r : integer;
begin
  if assigned (ListB) then
    begin
    self.Clear;
    for r := 0 to ListA.Count-1 do
      if ListB.indexof (ListA[r]) < 0 then
        self.Add (ListA[r]);
    end
  else
    begin
    for r := self.count-1 downto 0 do
      if ListA.indexof (self[r]) >= 0 then
        self.delete (r);
    end;
end;

procedure TFPList.DoOr(ListA, ListB : TFPList);
begin
  if assigned (ListB) then
    begin
    CopyMove (ListA);
    MergeMove (ListB);
    end
  else
    MergeMove (ListA);
end;

procedure TFPList.DoXOr(ListA, ListB : TFPList);
var r : integer;
    l : TFPList;
begin
  if assigned (ListB) then
    begin
    self.Clear;
    for r := 0 to ListA.count-1 do
      if ListB.indexof (ListA[r]) < 0 then
        self.Add (ListA[r]);
    for r := 0 to ListB.count-1 do
      if ListA.indexof (ListB[r]) < 0 then
        self.Add (ListB[r]);
    end
  else
    try
      l := TFPList.Create;
      l.CopyMove (Self);
      for r := self.count-1 downto 0 do
        if listA.indexof (self[r]) >= 0 then
          self.delete (r);
      for r := 0 to ListA.count-1 do
        if l.indexof (ListA[r]) < 0 then
          self.add (ListA[r]);
    finally
      l.Free;
    end;
end;


procedure TFPList.Assign (ListA: TFPList; AOperator: TListAssignOp=laCopy; ListB: TFPList=nil);
begin
  case AOperator of
    laCopy : DoCopy (ListA, ListB);             // replace dest with src
    laSrcUnique : DoSrcUnique (ListA, ListB);   // replace dest with src that are not in dest
    laAnd : DoAnd (ListA, ListB);               // remove from dest that are not in src
    laDestUnique : DoDestUnique (ListA, ListB); // remove from dest that are in src
    laOr : DoOr (ListA, ListB);                 // add to dest from src and not in dest
    laXOr : DoXOr (ListA, ListB);               // add to dest from src and not in dest, remove from dest that are in src
  end;
end;

{$else}

{ generics based implementation of TFPList follows }

procedure TFPList.Assign(Source: TFPList);
begin
  inherited Assign(Source);
end;

type
  TFPPtrListSortCompare = function(const Item1, Item2: Pointer): Integer;

procedure TFPList.Sort(Compare: TListSortCompare);
begin
  inherited Sort(TFPPtrListSortCompare(Compare));
end;

procedure TFPList.Sort(Compare: TListSortCompare; SortingAlgorithm: PSortingAlgorithm);
begin
  inherited Sort(TFPPtrListSortCompare(Compare), SortingAlgorithm);
end;

procedure TFPList.ForEachCall(Proc2call: TListCallback; Arg: Pointer);
var
  I: integer;
begin
  for I:=0 to Count-1 do
    proc2call(InternalItems[I],arg);
end;


procedure TFPList.ForEachCall(Proc2call: TListStaticCallback; Arg: Pointer);
var
  I: integer;
begin
  for I:=0 to Count-1 do
    Proc2call(InternalItems[I], Arg);
end;

{$endif}

{****************************************************************************}
{*                TListEnumerator                                           *}
{****************************************************************************}

constructor TListEnumerator.Create(AList: TList);
begin
  inherited Create(AList.FList);
end;


{****************************************************************************}
{*                TList                                                     *}
{****************************************************************************}

{  TList = class(TObject)
  private
    FList: TFPList;
}

function TList.Get(Index: Integer): Pointer;
begin
  Result := FList.Get(Index);
end;

procedure TList.Grow;
begin
  // Only for compatibility with Delphi. Not needed.
end;

procedure TList.Put(Index: Integer; Item: Pointer);
var p : pointer;
begin
  p := get(Index);
  FList.Put(Index, Item);
  if assigned (p) then
    Notify (p, lnDeleted);
  if assigned (Item) then
    Notify (Item, lnAdded);
end;

function TList.Extract(item: Pointer): Pointer;
var c : integer;
begin
  c := FList.Count;
  Result := FList.Extract(item);
  if c <> FList.Count then
    Notify (Result, lnExtracted);
end;

procedure TList.Notify(Ptr: Pointer; Action: TListNotification);
begin
   if Assigned(FObservers) then
     Case ACtion of
       lnAdded     : FPONotifyObservers(Self,ooAddItem,Ptr);
       lnExtracted : FPONotifyObservers(Self,ooDeleteItem,Ptr);
       lnDeleted   : FPONotifyObservers(Self,ooDeleteItem,Ptr);
     end;
end;

function TList.GetCapacity: integer;
begin
  Result := FList.Capacity;
end;

procedure TList.SetCapacity(NewCapacity: Integer);
begin
  FList.SetCapacity(NewCapacity);
end;

function TList.GetCount: Integer;
begin
  Result := FList.Count;
end;

procedure TList.SetCount(NewCount: Integer);
begin
  if NewCount < FList.Count then
    while FList.Count > NewCount do
      Delete(FList.Count - 1)
  else
    FList.SetCount(NewCount);
end;

constructor TList.Create;
begin
  inherited Create;
  FList := TFPList.Create;
end;

destructor TList.Destroy;
begin
  if Assigned(Flist) then
    Clear;
  If Assigned(FObservers) then
    begin
    FPONotifyObservers(Self,ooFree,Nil);
    FreeAndNil(FObservers);
    end;
  FreeAndNil(FList);
  inherited Destroy;
end;

procedure TList.FPOAttachObserver(AObserver: TObject);

Var
   I : IFPObserver;

begin
  If Not AObserver.GetInterface(SGUIDObserver,I) then
    Raise EObserver.CreateFmt(SErrNotObserver,[AObserver.ClassName]);
  If not Assigned(FObservers) then
    FObservers:=TFPList.Create;
  FObservers.Add(I);
end;

procedure TList.FPODetachObserver(AObserver: TObject);
Var
  I : IFPObserver;

begin
  If Not AObserver.GetInterface(SGUIDObserver,I) then
    Raise EObserver.CreateFmt(SErrNotObserver,[AObserver.ClassName]);
  If Assigned(FObservers) then
    begin
    FObservers.Remove(I);
    If (FObservers.Count=0) then
      FreeAndNil(FObservers);
    end;
end;

procedure TList.FPONotifyObservers(ASender: TObject;
  AOperation: TFPObservedOperation; Data : Pointer);

Var
  I : Integer;
  Obs : IFPObserver;

begin
  If Assigned(FObservers) then
    For I:=FObservers.Count-1 downto 0 do
      begin
      Obs:=IFPObserver(FObservers[i]);
      Obs.FPOObservedChanged(ASender,AOperation,Data);
      end;
end;

function TList.Add(Item: Pointer): Integer;
begin
  Result := FList.Add(Item);
  if Item <> nil then
    Notify(Item, lnAdded);
end;

Procedure TList.AddList(AList : TList);
var
  I: Integer;
begin
  { this only does FList.AddList(AList.FList), avoiding notifications }
  FList.AddList(AList.FList);

  { make lnAdded notifications }
  for I := 0 to AList.Count - 1 do
    if AList[I] <> nil then
      Notify(AList[I], lnAdded);
end;

procedure TList.Clear;

begin
  While (FList.Count>0) do
    Delete(Count-1);
end;

procedure TList.Delete(Index: Integer);

var P : pointer;

begin
  P:=FList.Get(Index);
  FList.Delete(Index);
  if assigned(p) then
    Notify(p, lnDeleted);
end;

class procedure TList.Error(const Msg: string; Data: PtrInt);
begin
  Raise EListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;

procedure TList.Exchange(Index1, Index2: Integer);
begin
  FList.Exchange(Index1, Index2);
  FPONotifyObservers(Self,ooChange,Nil);
end;

function TList.Expand: TList;
begin
  FList.Expand;
  Result:=Self;
end;

function TList.First: Pointer;
begin
  Result := FList.First;
end;

function TList.GetEnumerator: TListEnumerator;
begin
  Result := TListEnumerator.Create(Self);
end;

function TList.IndexOf(Item: Pointer): Integer;
begin
  Result := FList.IndexOf(Item);
end;

procedure TList.Insert(Index: Integer; Item: Pointer);
begin
  FList.Insert(Index, Item);
  if Item <> nil then
    Notify(Item,lnAdded);
end;

function TList.Last: Pointer;
begin
  Result := FList.Last;
end;

procedure TList.Move(CurIndex, NewIndex: Integer);
begin
  FList.Move(CurIndex, NewIndex);
end;

function TList.Remove(Item: Pointer): Integer;
begin
  Result := IndexOf(Item);
  if Result <> -1 then
    Self.Delete(Result);
end;

procedure TList.Pack;
begin
  FList.Pack;
end;

procedure TList.Sort(Compare: TListSortCompare);
begin
  FList.Sort(Compare);
end;

procedure TList.Sort(Compare: TListSortCompare; SortingAlgorithm: PSortingAlgorithm);
begin
  FList.Sort(Compare, SortingAlgorithm);
end;

procedure TList.Sort(Compare: TListSortComparer_Context; Context: Pointer);
begin
  FList.Sort(Compare, Context);
end;

procedure TList.Sort(Compare: TListSortComparer_Context; Context: Pointer; SortingAlgorithm: PSortingAlgorithm);
begin
  FList.Sort(Compare, Context, SortingAlgorithm);
end;

procedure TList.CopyMove (aList : TList);
var r : integer;
begin
  Clear;
  for r := 0 to aList.count-1 do
    Add (aList[r]);
end;

procedure TList.MergeMove (aList : TList);
var r : integer;
begin
  For r := 0 to aList.count-1 do
    if self.indexof(aList[r]) < 0 then
      self.Add (aList[r]);
end;

procedure TList.DoCopy(ListA, ListB : TList);
begin
  if assigned (ListB) then
    CopyMove (ListB)
  else
    CopyMove (ListA);
end;

procedure TList.DoDestUnique(ListA, ListB : TList);
  procedure MoveElements (src, dest : TList);
  var r : integer;
  begin
    self.clear;
    for r := 0 to src.count-1 do
      if dest.indexof(src[r]) < 0 then
        self.Add (src[r]);
  end;

var dest : TList;
begin
  if assigned (ListB) then
    MoveElements (ListB, ListA)
  else
    try
      dest := TList.Create;
      dest.CopyMove (self);
      MoveElements (ListA, dest)
    finally
      dest.Free;
    end;
end;

procedure TList.DoAnd(ListA, ListB : TList);
var r : integer;
begin
  if assigned (ListB) then
    begin
    self.clear;
    for r := 0 to ListA.count-1 do
      if ListB.indexOf (ListA[r]) >= 0 then
        self.Add (ListA[r]);
    end
  else
    begin
    for r := self.Count-1 downto 0 do
      if ListA.indexof (Self[r]) < 0 then
        self.delete (r);
    end;
end;

procedure TList.DoSrcUnique(ListA, ListB : TList);
var r : integer;
begin
  if assigned (ListB) then
    begin
    self.Clear;
    for r := 0 to ListA.Count-1 do
      if ListB.indexof (ListA[r]) < 0 then
        self.Add (ListA[r]);
    end
  else
    begin
    for r := self.count-1 downto 0 do
      if ListA.indexof (self[r]) >= 0 then
        self.delete (r);
    end;
end;

procedure TList.DoOr(ListA, ListB : TList);
begin
  if assigned (ListB) then
    begin
    CopyMove (ListA);
    MergeMove (ListB);
    end
  else
    MergeMove (ListA);
end;

procedure TList.DoXOr(ListA, ListB : TList);
var r : integer;
    l : TList;
begin
  if assigned (ListB) then
    begin
    self.Clear;
    for r := 0 to ListA.count-1 do
      if ListB.indexof (ListA[r]) < 0 then
        self.Add (ListA[r]);
    for r := 0 to ListB.count-1 do
      if ListA.indexof (ListB[r]) < 0 then
        self.Add (ListB[r]);
    end
  else
    try
      l := TList.Create;
      l.CopyMove (Self);
      for r := self.count-1 downto 0 do
        if listA.indexof (self[r]) >= 0 then
          self.delete (r);
      for r := 0 to ListA.count-1 do
        if l.indexof (ListA[r]) < 0 then
          self.add (ListA[r]);
    finally
      l.Free;
    end;
end;


procedure TList.Assign (ListA: TList; AOperator: TListAssignOp=laCopy; ListB: TList=nil);
begin
  case AOperator of
    laCopy : DoCopy (ListA, ListB);             // replace dest with src
    laSrcUnique : DoSrcUnique (ListA, ListB);   // replace dest with src that are not in dest
    laAnd : DoAnd (ListA, ListB);               // remove from dest that are not in src
    laDestUnique : DoDestUnique (ListA, ListB); // remove from dest that are in src
    laOr : DoOr (ListA, ListB);                 // add to dest from src and not in dest
    laXOr : DoXOr (ListA, ListB);               // add to dest from src and not in dest, remove from dest that are in src
  end;
end;


function TList.GetList: PPointerList;
begin
  Result := PPointerList(FList.List);
end;


{****************************************************************************}
{*                             TThreadList                                  *}
{****************************************************************************}


constructor TThreadList.Create;
  begin
    inherited Create;
    FDuplicates:=dupIgnore;
{$ifdef FPC_HAS_FEATURE_THREADING}
    InitCriticalSection(FLock);
{$endif}
    FList:=TList.Create;
  end;


destructor TThreadList.Destroy;
  begin
    LockList;
    try
      FList.Free;
      inherited Destroy;
    finally
      UnlockList;
{$ifdef FPC_HAS_FEATURE_THREADING}
      DoneCriticalSection(FLock);
{$endif}
    end;
  end;


procedure TThreadList.Add(Item: Pointer);
  begin
    LockList;
    try
      if (Duplicates=dupAccept) or
        // make sure it's not already in the list
        (FList.IndexOf(Item)=-1) then
         FList.Add(Item)
       else if (Duplicates=dupError) then
         FList.Error(SDuplicateItem,PtrUInt(Item));
    finally
      UnlockList;
    end;
  end;


procedure TThreadList.Clear;
  begin
    Locklist;
    try
      FList.Clear;
    finally
      UnLockList;
    end;
  end;


function TThreadList.LockList: TList;
  begin
    Result:=FList;
{$ifdef FPC_HAS_FEATURE_THREADING}
    System.EnterCriticalSection(FLock);
{$endif}
  end;


procedure TThreadList.Remove(Item: Pointer);
  begin
    LockList;
    try
      FList.Remove(Item);
    finally
      UnlockList;
    end;
  end;


procedure TThreadList.UnlockList;
  begin
{$ifdef FPC_HAS_FEATURE_THREADING}
    System.LeaveCriticalSection(FLock);
{$endif}
  end;
