{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{****************************************************************************}
{*                             TCollectionItem                              *}
{****************************************************************************}


function TCollectionItem.GetIndex: Integer;

begin
  if FCollection<>nil then
    Result:=FCollection.FItems.IndexOf(Pointer(Self))
  else
    Result:=-1;
end;



procedure TCollectionItem.SetCollection(Value: TCollection);

begin
  IF Value<>FCollection then
    begin
    If FCollection<>Nil then FCollection.RemoveItem(Self);
    if Value<>Nil then Value.InsertItem(Self);
    end;
end;



procedure TCollectionItem.Changed(AllItems: Boolean);

begin
 If (FCollection<>Nil) and (FCollection.UpdateCount=0) then
  begin
  If AllItems then
    FCollection.Update(Nil)
  else
    FCollection.Update(Self);
  end;
end;



function TCollectionItem.GetNamePath: string;

begin
  If FCollection<>Nil then
    Result:=FCollection.GetNamePath+'['+IntToStr(Index)+']'
  else
    Result:=ClassName;
end;


function TCollectionItem.GetOwner: TPersistent;

begin
  Result:=FCollection;
end;



function TCollectionItem.GetDisplayName: string;

begin
  Result:=ClassName;
end;



procedure TCollectionItem.SetIndex(Value: Integer);

Var Temp : Longint;

begin
  Temp:=GetIndex;
  If (Temp>-1) and (Temp<>Value) then
    begin
    FCollection.FItems.Move(Temp,Value);
    Changed(True);
    end;
end;


procedure TCollectionItem.SetDisplayName(const Value: string);

begin
  Changed(False);
end;



constructor TCollectionItem.Create(ACollection: TCollection);

begin
  Inherited Create;
  SetCollection(ACollection);
end;



destructor TCollectionItem.Destroy;

begin
  SetCollection(Nil);
  Inherited Destroy;
end;

{****************************************************************************}
{*                          TCollectionEnumerator                           *}
{****************************************************************************}

constructor TCollectionEnumerator.Create(ACollection: TCollection);
begin
  inherited Create;
  FCollection := ACollection;
  FPosition := -1;
end;

function TCollectionEnumerator.GetCurrent: TCollectionItem;
begin
  Result := FCollection.Items[FPosition];
end;

function TCollectionEnumerator.MoveNext: Boolean;
begin
  Inc(FPosition);
  Result := FPosition < FCollection.Count;
end;


{****************************************************************************}
{*                             TCollection                                  *}
{****************************************************************************}

function TCollection.Owner: TPersistent;
begin
  result:=getowner;
end;


function TCollection.GetCount: Integer;

begin
  Result:=FItems.Count;
end;


Procedure TCollection.SetPropName;

Var
  TheOwner : TPersistent;
  PropList : PPropList;
  I, PropCount : Integer;

begin
  FPropName:='';
  TheOwner:=GetOwner;
  if (TheOwner=Nil) Or (TheOwner.Classinfo=Nil) Then Exit;
  // get information from the owner RTTI
  PropCount:=GetPropList(TheOwner, PropList);
  Try
    For I:=0 To PropCount-1 Do
      If (PropList^[i]^.PropType^.Kind=tkClass) And
         (GetObjectProp(TheOwner, PropList^[i], ClassType)=Self) Then
        Begin
          FPropName:=PropList^[i]^.Name;
          Exit;
        End;
  Finally
    FreeMem(PropList);
  End;
end;


function TCollection.GetPropName: string;

Var
  TheOwner : TPersistent;

begin
  Result:=FPropNAme;
  TheOwner:=GetOwner;
  If (Result<>'') or (TheOwner=Nil) Or (TheOwner.Classinfo=Nil) then exit;
  SetPropName;
  Result:=FPropName;
end;


procedure TCollection.InsertItem(Item: TCollectionItem);
begin
  If Not(Item Is FitemClass) then
    exit;
  FItems.add(Pointer(Item));
  Item.FCollection:=Self;
  Item.FID:=FNextID;
  inc(FNextID);
  SetItemName(Item);
  Notify(Item,cnAdded);
  Changed;
end;


procedure TCollection.RemoveItem(Item: TCollectionItem);

Var
  I : Integer;

begin
  Notify(Item,cnExtracting);
  I:=FItems.IndexOfItem(Item,fromEnd);
  If (I<>-1) then
    FItems.Delete(I);
  Item.FCollection:=Nil;
  Changed;
end;


function TCollection.GetAttrCount: Integer;
begin
  Result:=0;
end;


function TCollection.GetAttr(Index: Integer): string;
begin
  Result:='';
end;


function TCollection.GetItemAttr(Index, ItemIndex: Integer): string;
begin
  Result:=TCollectionItem(FItems.Items[ItemIndex]).DisplayName;
end;


function TCollection.GetEnumerator: TCollectionEnumerator;
begin
  Result := TCollectionEnumerator.Create(Self);
end;


function TCollection.GetNamePath: string;
<<<<<<< HEAD
<<<<<<< HEAD
var o : TPersistent;
begin
  o:=getowner;
  if assigned(o) and (propname<>'') then 
     result:=o.getnamepath+'.'+propname
=======
var o : TObject;
begin
  o:=getowner;
  if assigned(o) and (propname<>'') and (o IS TPersistent) then 
     result:=TPersistent(o).getnamepath+'.'+propname
>>>>>>> graemeg/fixes_2_2
=======
var o : TObject;
begin
  o:=getowner;
  if assigned(o) and (propname<>'') and (o IS TPersistent) then 
     result:=TPersistent(o).getnamepath+'.'+propname
>>>>>>> origin/fixes_2_2
   else
     result:=classname;
end;


procedure TCollection.Changed;
begin
  if FUpdateCount=0 then
    Update(Nil);
end;


function TCollection.GetItem(Index: Integer): TCollectionItem;
begin
  Result:=TCollectionItem(FItems.Items[Index]);
end;


procedure TCollection.SetItem(Index: Integer; Value: TCollectionItem);
begin
  TCollectionItem(FItems.items[Index]).Assign(Value);
end;


procedure TCollection.SetItemName(Item: TCollectionItem);
begin
end;



procedure TCollection.Update(Item: TCollectionItem);
begin
  FPONotifyObservers(Self,ooChange,Pointer(Item));
end;


constructor TCollection.Create(AItemClass: TCollectionItemClass);
begin
  inherited create;
  FItemClass:=AItemClass;
  FItems:=TFpList.Create;
end;


destructor TCollection.Destroy;
begin
  FUpdateCount:=1; // Prevent OnChange
  try
    DoClear;
  Finally
    FUpdateCount:=0;
  end;
  FItems.Free;
  Inherited Destroy;
end;


function TCollection.Add: TCollectionItem;
begin
  Result:=FItemClass.Create(Self);
end;


procedure TCollection.Assign(Source: TPersistent);
Var I : Longint;
begin
  If Source is TCollection then
    begin
    Clear;
    For I:=0 To TCollection(Source).Count-1 do
     Add.Assign(TCollection(Source).Items[I]);
    exit;
    end
  else
    Inherited Assign(Source);
end;


procedure TCollection.BeginUpdate;
begin
  inc(FUpdateCount);
end;


procedure TCollection.Clear;
begin
  if FItems.Count=0 then
    exit; // Prevent Changed
  BeginUpdate;
  try
    DoClear;
  finally
    EndUpdate;
  end;    
end;


procedure TCollection.DoClear;
begin
  While FItems.Count>0 do TCollectionItem(FItems.Last).Free;
end;


procedure TCollection.EndUpdate;
begin
  if FUpdateCount>0 then
    dec(FUpdateCount);
  if FUpdateCount=0 then
    Changed;
end;


function TCollection.FindItemID(ID: Integer): TCollectionItem;
Var
          I : Longint;
begin
  For I:=0 to Fitems.Count-1 do
   begin
     Result:=TCollectionItem(FItems.items[I]);
     If Result.Id=Id then
       exit;
   end;
  Result:=Nil;
end;


procedure TCollection.Delete(Index: Integer);
Var
  Item : TCollectionItem;
begin
  Item:=TCollectionItem(FItems[Index]);
  Notify(Item,cnDeleting);
  Item.Free;
end;


function TCollection.Insert(Index: Integer): TCollectionItem;
begin
  Result:=Add;
  Result.Index:=Index;
end;


procedure TCollection.Notify(Item: TCollectionItem;Action: TCollectionNotification);
begin
  if Assigned(FObservers) and (FUpdateCount = 0) then
    Case Action of
      cnAdded      : FPONotifyObservers(Self,ooAddItem,Pointer(Item));
      cnExtracting : FPONotifyObservers(Self,ooDeleteItem,Pointer(Item));
      cnDeleting   : FPONotifyObservers(Self,ooDeleteItem,Pointer(Item));
    end;
end;

procedure TCollection.Sort(Const Compare : TCollectionSortCompare);

begin
  BeginUpdate;
  try
    FItems.Sort(TListSortCompare(Compare));
  Finally
    EndUpdate;
  end;
end;

procedure TCollection.Exchange(Const Index1, index2: integer);

begin
  FItems.Exchange(Index1,Index2);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if FUpdateCount = 0 then
    FPONotifyObservers(Self,ooChange,Nil);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


{****************************************************************************}
{*                             TOwnedCollection                             *}
{****************************************************************************}



Constructor TOwnedCollection.Create(AOwner: TPersistent; AItemClass: TCollectionItemClass);

Begin
  FOwner := AOwner;
  inherited Create(AItemClass);
end;



Function TOwnedCollection.GetOwner: TPersistent;

begin
  Result:=FOwner;
end;



