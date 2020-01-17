{
    This file is part of the Free Pascal Run Time Library (rtl)
    Copyright (c) 1999-2019 by the Free Pascal development team

    This file provides the base for the pluggable sorting algorithm
    support. It also provides a default QuickSort implementation.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit sortbase;

{$MODE objfpc}

interface

type
  TListSortComparer_NoContext = function(Item1, Item2: Pointer): Integer;
  TPtrListSorter_NoContext = procedure(ItemPtrs: PPointer; ItemCount: SizeUInt; Comparer: TListSortComparer_NoContext);
  TItemListSorter_NoContext = procedure(Items: Pointer; ItemCount, ItemSize: SizeUInt; Comparer: TListSortComparer_NoContext);

  TListSortComparer_Context = function(Item1, Item2, Context: Pointer): Integer;
  TListSortCustomItemExchanger_Context = procedure(Item1, Item2, Context: Pointer);
  TPtrListSorter_Context = procedure(ItemPtrs: PPointer; ItemCount: SizeUInt; Comparer: TListSortComparer_Context; Context: Pointer);
  TItemListSorter_Context = procedure(Items: Pointer; ItemCount, ItemSize: SizeUInt; Comparer: TListSortComparer_Context; Context: Pointer);
  TItemListSorter_CustomItemExchanger_Context = procedure(Items: Pointer;
    ItemCount, ItemSize: SizeUInt; Comparer: TListSortComparer_Context;
    Exchanger: TListSortCustomItemExchanger_Context; Context: Pointer);

  PSortingAlgorithm = ^TSortingAlgorithm;
  TSortingAlgorithm = record
    PtrListSorter_NoContextComparer: TPtrListSorter_NoContext;
    PtrListSorter_ContextComparer: TPtrListSorter_Context;
    ItemListSorter_ContextComparer: TItemListSorter_Context;
    ItemListSorter_CustomItemExchanger_ContextComparer: TItemListSorter_CustomItemExchanger_Context;
  end;

{
                       QuickSort

  Average performance: O(n log n)
    Worst performance: O(n*n)
     Extra memory use: O(log n) on the stack
               Stable: no
     Additional notes: Uses the middle element as the pivot. This makes it work
                       well also on already sorted sequences, which can occur
                       often in practice. As expected from QuickSort, it works
                       best on random sequences and is usually the fastest
                       algorithm to sort them. It is, however, possible for a
                       malicious user to craft special sequences, which trigger
                       its worst O(n*n) case. They can also occur in practice,
                       although they are very unlikely. If this is not an
                       acceptable risk (e.g. for high risk applications,
                       security-conscious applications or applications with hard
                       real-time requirements), another sorting algorithm must
                       be used.
}

procedure QuickSort_PtrList_NoContext(
                ItemPtrs: PPointer;
                ItemCount: SizeUInt;
                Comparer: TListSortComparer_NoContext);
procedure QuickSort_PtrList_Context(
                ItemPtrs: PPointer;
                ItemCount: SizeUInt;
                Comparer: TListSortComparer_Context;
                Context: Pointer);
procedure QuickSort_ItemList_Context(
                Items: Pointer;
                ItemCount, ItemSize: SizeUInt;
                Comparer: TListSortComparer_Context;
                Context: Pointer);
procedure QuickSort_ItemList_CustomItemExchanger_Context(
                Items: Pointer;
                ItemCount, ItemSize: SizeUInt;
                Comparer: TListSortComparer_Context;
                Exchanger: TListSortCustomItemExchanger_Context;
                Context: Pointer);

const
  QuickSort: TSortingAlgorithm = (
    PtrListSorter_NoContextComparer: @QuickSort_PtrList_NoContext;
    PtrListSorter_ContextComparer: @QuickSort_PtrList_Context;
    ItemListSorter_ContextComparer: @QuickSort_ItemList_Context;
    ItemListSorter_CustomItemExchanger_ContextComparer: @QuickSort_ItemList_CustomItemExchanger_Context;
  );

var
  DefaultSortingAlgorithm: PSortingAlgorithm = @QuickSort;

implementation

Procedure QuickSort_PtrList_NoContext(ItemPtrs: PPointer; L, R : SizeUInt;
                                      Comparer: TListSortComparer_NoContext);
var
  I, J, PivotIdx : SizeUInt;
  P, Q : Pointer;
begin
 repeat
   I := L;
   J := R;
   PivotIdx := L + ((R - L) shr 1); { same as ((L + R) div 2), but without the possibility of overflow }
   P := ItemPtrs[PivotIdx];
   repeat
     while (I < PivotIdx) and (Comparer(P, ItemPtrs[i]) > 0) do
       Inc(I);
     while (J > PivotIdx) and (Comparer(P, ItemPtrs[J]) < 0) do
       Dec(J);
     if I < J then
     begin
       Q := ItemPtrs[I];
       ItemPtrs[I] := ItemPtrs[J];
       ItemPtrs[J] := Q;
       if PivotIdx = I then
       begin
         PivotIdx := J;
         Inc(I);
       end
       else if PivotIdx = J then
       begin
         PivotIdx := I;
         Dec(J);
       end
       else
       begin
         Inc(I);
         Dec(J);
       end;
     end;
   until I >= J;
   // sort the smaller range recursively
   // sort the bigger range via the loop
   // Reasons: memory usage is O(log(n)) instead of O(n) and loop is faster than recursion
   if (PivotIdx - L) < (R - PivotIdx) then
   begin
     if (L + 1) < PivotIdx then
       QuickSort_PtrList_NoContext(ItemPtrs, L, PivotIdx - 1, Comparer);
     L := PivotIdx + 1;
   end
   else
   begin
     if (PivotIdx + 1) < R then
       QuickSort_PtrList_NoContext(ItemPtrs, PivotIdx + 1, R, Comparer);
     if (L + 1) < PivotIdx then
       R := PivotIdx - 1
     else
       exit;
   end;
 until L >= R;
end;

procedure QuickSort_PtrList_NoContext(ItemPtrs: PPointer; ItemCount: SizeUInt; Comparer: TListSortComparer_NoContext);
begin
  if not Assigned(ItemPtrs) or (ItemCount < 2) then
    exit;
  QuickSort_PtrList_NoContext(ItemPtrs, 0, ItemCount - 1, Comparer);
end;

procedure QuickSort_PtrList_Context(ItemPtrs: PPointer; ItemCount: SizeUInt; Comparer: TListSortComparer_Context; Context: Pointer);

  procedure QuickSort(L, R : SizeUInt);
  var
    I, J, PivotIdx : SizeUInt;
    P, Q : Pointer;
  begin
    repeat
      I := L;
      J := R;
      PivotIdx := L + ((R - L) shr 1); { same as ((L + R) div 2), but without the possibility of overflow }
      P := ItemPtrs[PivotIdx];
      repeat
        while (I < PivotIdx) and (Comparer(P, ItemPtrs[I], Context) > 0) do
          Inc(I);
        while (J > PivotIdx) and (Comparer(P, ItemPtrs[J], Context) < 0) do
          Dec(J);
        if I < J then
        begin
          Q := ItemPtrs[I];
          ItemPtrs[I] := ItemPtrs[J];
          ItemPtrs[J] := Q;
          if PivotIdx = I then
          begin
            PivotIdx := J;
            Inc(I);
          end
          else if PivotIdx = J then
          begin
            PivotIdx := I;
            Dec(J);
          end
          else
          begin
            Inc(I);
            Dec(J);
          end;
        end;
      until I >= J;
      // sort the smaller range recursively
      // sort the bigger range via the loop
      // Reasons: memory usage is O(log(n)) instead of O(n) and loop is faster than recursion
      if (PivotIdx - L) < (R - PivotIdx) then
      begin
        if (L + 1) < PivotIdx then
          QuickSort(L, PivotIdx - 1);
        L := PivotIdx + 1;
      end
      else
      begin
        if (PivotIdx + 1) < R then
          QuickSort(PivotIdx + 1, R);
        if (L + 1) < PivotIdx then
          R := PivotIdx - 1
        else
          exit;
      end;
    until L >= R;
  end;

begin
  if not Assigned(ItemPtrs) or (ItemCount < 2) then
    exit;
  QuickSort(0, ItemCount - 1);
end;

procedure QuickSort_ItemList_Context(Items: Pointer; ItemCount, ItemSize: SizeUInt; Comparer: TListSortComparer_Context; Context: Pointer);

var
  TempBuf: Pointer;

  procedure QuickSort(L, R : SizeUInt);
  var
    I, J, PivotIdx : SizeUInt;
    P : Pointer;
  begin
    repeat
      I := L;
      J := R;
      PivotIdx := L + ((R - L) shr 1); { same as ((L + R) div 2), but without the possibility of overflow }
      P := Items + ItemSize*PivotIdx;
      repeat
        while (I < PivotIdx) and (Comparer(P, Items + ItemSize*I, Context) > 0) do
          Inc(I);
        while (J > PivotIdx) and (Comparer(P, Items + ItemSize*J, Context) < 0) do
          Dec(J);
        if I < J then
        begin
          Move((Items + ItemSize*I)^, TempBuf^, ItemSize);
          Move((Items + ItemSize*J)^, (Items + ItemSize*I)^, ItemSize);
          Move(TempBuf^, (Items + ItemSize*J)^, ItemSize);
          if PivotIdx = I then
          begin
            PivotIdx := J;
            P := Items + ItemSize*PivotIdx;
            Inc(I);
          end
          else if PivotIdx = J then
          begin
            PivotIdx := I;
            P := Items + ItemSize*PivotIdx;
            Dec(J);
          end
          else
          begin
            Inc(I);
            Dec(J);
          end;
        end;
      until I >= J;
      // sort the smaller range recursively
      // sort the bigger range via the loop
      // Reasons: memory usage is O(log(n)) instead of O(n) and loop is faster than recursion
      if (PivotIdx - L) < (R - PivotIdx) then
      begin
        if (L + 1) < PivotIdx then
          QuickSort(L, PivotIdx - 1);
        L := PivotIdx + 1;
      end
      else
      begin
        if (PivotIdx + 1) < R then
          QuickSort(PivotIdx + 1, R);
        if (L + 1) < PivotIdx then
          R := PivotIdx - 1
        else
          exit;
      end;
    until L >= R;
  end;

begin
  if not Assigned(Items) or (ItemCount < 2) or (ItemSize < 1) then
    exit;
  GetMem(TempBuf, ItemSize);
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  try
    QuickSort(0, ItemCount - 1);
  finally
    FreeMem(TempBuf, ItemSize);
  end;
{$else FPC_HAS_FEATURE_EXCEPTIONS}
  QuickSort(0, ItemCount - 1);
  FreeMem(TempBuf, ItemSize);
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
end;

procedure QuickSort_ItemList_CustomItemExchanger_Context(
                Items: Pointer;
                ItemCount, ItemSize: SizeUInt;
                Comparer: TListSortComparer_Context;
                Exchanger: TListSortCustomItemExchanger_Context;
                Context: Pointer);

  procedure QuickSort(L, R : SizeUInt);
  var
    I, J, PivotIdx : SizeUInt;
    P : Pointer;
  begin
    repeat
      I := L;
      J := R;
      PivotIdx := L + ((R - L) shr 1); { same as ((L + R) div 2), but without the possibility of overflow }
      P := Items + ItemSize*PivotIdx;
      repeat
        while (I < PivotIdx) and (Comparer(P, Items + ItemSize*I, Context) > 0) do
          Inc(I);
        while (J > PivotIdx) and (Comparer(P, Items + ItemSize*J, Context) < 0) do
          Dec(J);
        if I < J then
        begin
          Exchanger(Items + ItemSize*I, Items + ItemSize*J, Context);
          if PivotIdx = I then
          begin
            PivotIdx := J;
            P := Items + ItemSize*PivotIdx;
            Inc(I);
          end
          else if PivotIdx = J then
          begin
            PivotIdx := I;
            P := Items + ItemSize*PivotIdx;
            Dec(J);
          end
          else
          begin
            Inc(I);
            Dec(J);
          end;
        end;
      until I >= J;
      // sort the smaller range recursively
      // sort the bigger range via the loop
      // Reasons: memory usage is O(log(n)) instead of O(n) and loop is faster than recursion
      if (PivotIdx - L) < (R - PivotIdx) then
      begin
        if (L + 1) < PivotIdx then
          QuickSort(L, PivotIdx - 1);
        L := PivotIdx + 1;
      end
      else
      begin
        if (PivotIdx + 1) < R then
          QuickSort(PivotIdx + 1, R);
        if (L + 1) < PivotIdx then
          R := PivotIdx - 1
        else
          exit;
      end;
    until L >= R;
  end;

begin
  if not Assigned(Items) or (ItemCount < 2) or (ItemSize < 1) then
    exit;
  QuickSort(0, ItemCount - 1);
end;

end.
