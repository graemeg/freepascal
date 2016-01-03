<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the Free Pascal Run Time Library (rtl)
    Copyright (c) 2007 by Michael Van Canneyt,
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Type
  TLinkedListItem = Class
  Public
    Next : TLinkedListItem;
  end;
  TLinkedListItemClass = Class of TLinkedListItem;
  
  { TLinkedListVisitor }

  TLinkedListVisitor = Class
    Function Visit(Item : TLinkedListItem) : Boolean; virtual; abstract;
  end;
  { TLinkedList }

  TLinkedList = Class
  private
    FItemClass: TLinkedListItemClass;
    FRoot: TLinkedListItem;
    function GetCount: Integer;
  Public
    Constructor Create(AnItemClass : TLinkedListItemClass); virtual;
    Destructor Destroy; override;
    Procedure Clear;
    Function Add : TLinkedListItem;
    Procedure ForEach(Visitor: TLinkedListVisitor);
    Procedure RemoveItem(Item : TLinkedListItem; FreeItem : Boolean = False);
    Property Root : TLinkedListItem Read FRoot;
    Property ItemClass : TLinkedListItemClass Read FItemClass;
    Property Count : Integer Read GetCount;
  end;

{ TLinkedList }

function TLinkedList.GetCount: Integer;

Var
  I : TLinkedListItem;

begin
  I:=FRoot;
  Result:=0;
  While I<>Nil do
    begin
    I:=I.Next;
    Inc(Result);
    end;
end;

constructor TLinkedList.Create(AnItemClass: TLinkedListItemClass);
begin
  FItemClass:=AnItemClass;
end;

destructor TLinkedList.Destroy;
begin
  Clear;
  inherited Destroy;
end;

procedure TLinkedList.Clear;

Var
   I : TLinkedListItem;

begin
  // Can't use visitor, because it'd kill the next pointer...
  I:=FRoot;
  While I<>Nil do
    begin
    FRoot:=I;
    I:=I.Next;
    FRoot.Next:=Nil;
    FreeAndNil(FRoot);
    end;
end;

function TLinkedList.Add: TLinkedListItem;
begin
  Result:=FItemClass.Create;
  Result.Next:=FRoot;
  FRoot:=Result;
end;

procedure TLinkedList.ForEach(Visitor : TLinkedListVisitor);

Var
  I : TLinkedListItem;

begin
  I:=FRoot;
  While (I<>Nil) and Visitor.Visit(I) do
    I:=I.Next;
end;

procedure TLinkedList.RemoveItem(Item: TLinkedListItem; FreeItem : Boolean = False);

Var
  I : TLinkedListItem;

begin
  If (Item<>Nil) and (FRoot<>Nil) then
    begin
    If (Item=FRoot) then
      FRoot:=Item.Next
    else
      begin
      I:=FRoot;
      While (I.Next<>Nil) and (I.Next<>Item) do
        I:=I.Next;
      If (I.Next=Item) then
        I.Next:=Item.Next;
      end;
    If FreeItem Then
      Item.Free;
    end;
end;

