{

   Tabbed group for TV/FV dialogs

   Copyright 2000-4 by Free Pascal core team

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the Free
   Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************}
unit tabs;

{$I platform.inc}    (* Multi-platform support defines *)
{$CODEPAGE cp437}

interface

uses
  objects,
  drivers,
  views,
  fvconsts;


type
    PTabItem = ^TTabItem;
    TTabItem = record
      Next : PTabItem;
      View : PView;
      Dis  : boolean;
    end;

    PTabDef = ^TTabDef;
    TTabDef = record
      Next     : PTabDef;
      Name     : PString;
      Items    : PTabItem;
      DefItem  : PView;
      ShortCut : char;
    end;

    PTab = ^TTab;
    TTab = object(TGroup)
      TabDefs   : PTabDef;
      ActiveDef : integer;
      DefCount  : word;
      constructor Init(var Bounds: TRect; ATabDef: PTabDef);
      constructor Load (var S: TStream);
      function    AtTab(Index: integer): PTabDef; virtual;
      procedure   SelectTab(Index: integer); virtual;
      procedure   Store (var S: TStream);
      function    TabCount: integer;
      function    Valid(Command: Word): Boolean; virtual;
      procedure   ChangeBounds(var Bounds: TRect); virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      function    GetPalette: PPalette; virtual;
      procedure   Draw; virtual;
      function    DataSize: sw_word;virtual;
      procedure   SetData(var Rec);virtual;
      procedure   GetData(var Rec);virtual;
      procedure   SetState(AState: Word; Enable: Boolean); virtual;
      destructor  Done; virtual;
    private
      InDraw: boolean;
      function FirstSelectable: PView;
      function LastSelectable: PView;
    end;

function  NewTabItem(AView: PView; ANext: PTabItem): PTabItem;
procedure DisposeTabItem(P: PTabItem);
function  NewTabDef(AName: string; ADefItem: PView; AItems: PTabItem; ANext: PTabDef): PTabDef;
procedure DisposeTabDef(P: PTabDef);

procedure RegisterTab;

const
  RTab: TStreamRec = (
    ObjType: idTab;
{$IFDEF BP_VMTLink}                              { BP style VMT link }
    VmtLink: Ofs (TypeOf (TTab)^);
{$ELSE BP_VMTLink}                               { Alt style VMT link }
    VmtLink: TypeOf (TTab);
{$ENDIF BP_VMTLink}
    Load: @TTab.Load;
    Store: @TTab.Store
  );


implementation

uses
  FvCommon,
  dialogs;

constructor TTab.Init(var Bounds: TRect; ATabDef: PTabDef);
begin
  inherited Init(Bounds);
  Options:=Options or ofSelectable or ofFirstClick or ofPreProcess or ofPostProcess;
  GrowMode:=gfGrowHiX+gfGrowHiY+gfGrowRel;
  TabDefs:=ATabDef;
  ActiveDef:=-1;
  SelectTab(0);
  ReDraw;
end;

constructor TTab.Load (var S: TStream);

    function DoLoadTabItems (var XDefItem: PView; ActItem: longint): PTabItem;
    var
      Count: longint;
      Cur, First: PTabItem;
      Last: ^PTabItem;
    begin
      Cur := nil;                                      { Preset nil }
      Last := @First;                                  { Start on first item }
      S.Read (Count, SizeOf(Count));                   { Read item count }
      while (Count > 0) do
       begin
        New (Cur);                                     { New status item }
        Last^ := Cur;                                  { First chain part }
        if (Cur <> nil) then                           { Check pointer valid }
         begin
          Last := @Cur^.Next;                          { Chain complete }
          S.Read (Cur^.Dis, SizeOf (Cur^.Dis));
          Cur^.View := PView (S.Get);
          if ActItem = 0 then
           XDefItem := Cur^.View;                      { Find default view }
         end;
        Dec (Count);                                   { One item loaded }
        Dec (ActItem);
      end;
      Last^ := nil;                                    { Now chain end }
      DoLoadTabItems := First;                         { Return the list }
    end;

   function DoLoadTabDefs: PTabDef;
   var
     Count: longint;
     Cur, First: PTabDef;
     Last: ^PTabDef;
     ActItem: longint;
   begin
     Last := @First;                                  { Start on first }
     Count := DefCount;
     while (Count > 0) do
      begin
       New (Cur);                                     { New status def }
       Last^ := Cur;                                  { First part of chain }
       if (Cur <> nil) then                           { Check pointer valid }
        begin
         Last := @Cur^.Next;                          { Chain complete }
         Cur^.Name := S.ReadStr;                      { Read name }
         S.Read (Cur^.ShortCut, SizeOf (Cur^.ShortCut));
         S.Read (ActItem, SizeOf (ActItem));
         Cur^.Items := DoLoadTabItems (Cur^.DefItem, ActItem); { Set pointer }
        end;
       Dec (Count);                                   { One item loaded }
      end;
     Last^ := nil;                                    { Now chain ends }
     DoLoadTabDefs := First;                          { Return item list }
   end;

begin
  inherited Load (S);
  S.Read (DefCount, SizeOf (DefCount));
  S.Read (ActiveDef, SizeOf (ActiveDef));
  TabDefs := DoLoadTabDefs;
end;

procedure TTab.Store (var S: TStream);

  procedure DoStoreTabItems (Cur: PTabItem; XDefItem: PView);
  var
    Count: longint;
    T: PTabItem;
    ActItem: longint;
  begin
    Count := 0;                                       { Clear count }
    T := Cur;                                         { Start on current }
    while (T <> nil) do
     begin
      if T^.View = XDefItem then                      { Current = active? }
       ActItem := Count;                              { => set order }
      Inc (Count);                                    { Count items }
      T := T^.Next;                                   { Next item }
     end;
    S.Write (ActItem, SizeOf (ActItem));
    S.Write (Count, SizeOf (Count));                  { Write item count }
    while (Cur <> nil) do
     begin
      S.Write (Cur^.Dis, SizeOf (Cur^.Dis));
      S.Put (Cur^.View);
     end;
  end;

  procedure DoStoreTabDefs (Cur: PTabDef);
  begin
    while (Cur <> nil) do
     begin
      with Cur^ do
       begin
        S.WriteStr (Cur^.Name);                       { Write name }
        S.Write (Cur^.ShortCut, SizeOf (Cur^.ShortCut));
        DoStoreTabItems (Items, DefItem);             { Store the items }
       end;
      Cur := Cur^.Next;                               { Next status item }
     end;
  end;

begin
  inherited Store (S);
  S.Write (DefCount, SizeOf (DefCount));
  S.Write (ActiveDef, SizeOf (ActiveDef));
  DoStoreTabDefs (TabDefs);
end;

function TTab.TabCount: integer;
var i: integer;
    P: PTabDef;
begin
  I:=0; P:=TabDefs;
  while (P<>nil) do
    begin
      Inc(I);
      P:=P^.Next;
    end;
  TabCount:=I;
end;


function TTab.AtTab(Index: integer): PTabDef;
var i: integer;
    P: PTabDef;
begin
  i:=0; P:=TabDefs;
  while (I<Index) do
    begin
      if P=nil then RunError($AA);
      P:=P^.Next;
      Inc(i);
    end;
  AtTab:=P;
end;

procedure TTab.SelectTab(Index: integer);
var P: PTabItem;
    V: PView;
begin
  if ActiveDef<>Index then
  begin
    if Owner<>nil then Owner^.Lock;
    Lock;
    { --- Update --- }
    if TabDefs<>nil then
       begin
         DefCount:=1;
         while AtTab(DefCount-1)^.Next<>nil do Inc(DefCount);
       end
       else DefCount:=0;
    if ActiveDef<>-1 then
    begin
      P:=AtTab(ActiveDef)^.Items;
      while P<>nil do
        begin
          if P^.View<>nil then Delete(P^.View);
          P:=P^.Next;
        end;
    end;
    ActiveDef:=Index;
    P:=AtTab(ActiveDef)^.Items;
    while P<>nil do
      begin
        if P^.View<>nil then Insert(P^.View);
        P:=P^.Next;
      end;
    V:=AtTab(ActiveDef)^.DefItem;
    if V<>nil then V^.Select;
    ReDraw;
    { --- Update --- }
    UnLock;
    if Owner<>nil then Owner^.UnLock;
    DrawView;
  end;
end;

procedure TTab.ChangeBounds(var Bounds: TRect);
var D: TPoint;
procedure DoCalcChange(P: PView); {$ifndef FPC}far;{$endif}
var
  R: TRect;
begin
  if P^.Owner=nil then Exit; { it think this is a bug in TV }
  P^.CalcBounds(R, D);
  P^.ChangeBounds(R);
end;
var
    P: PTabItem;
    I: integer;
begin
  D.X := Bounds.B.X - Bounds.A.X - Size.X;
  D.Y := Bounds.B.Y - Bounds.A.Y - Size.Y;
  inherited ChangeBounds(Bounds);
  for I:=0 to TabCount-1 do
  if I<>ActiveDef then
    begin
      P:=AtTab(I)^.Items;
      while P<>nil do
        begin
          if P^.View<>nil then DoCalcChange(P^.View);
          P:=P^.Next;
        end;
    end;
end;


function TTab.FirstSelectable: PView;
var
    FV : PView;
begin
  FV := First;
  while (FV<>nil) and ((FV^.Options and ofSelectable)=0) and (FV<>Last) do
        FV:=FV^.Next;
  if FV<>nil then
    if (FV^.Options and ofSelectable)=0 then FV:=nil;
  FirstSelectable:=FV;
end;


function TTab.LastSelectable: PView;
var
    LV : PView;
begin
  LV := Last;
  while (LV<>nil) and ((LV^.Options and ofSelectable)=0) and (LV<>First) do
        LV:=LV^.Prev;
  if LV<>nil then
    if (LV^.Options and ofSelectable)=0 then LV:=nil;
  LastSelectable:=LV;
end;

procedure TTab.HandleEvent(var Event: TEvent);
var Index : integer;
    I     : integer;
    X     : integer;
    Len   : byte;
    P     : TPoint;
    V     : PView;
    CallOrig: boolean;
    LastV : PView;
    FirstV: PView;
begin
  if (Event.What and evMouseDown)<>0 then
     begin
       MakeLocal(Event.Where,P);
       if P.Y<3 then
          begin
            Index:=-1; X:=1;
            for i:=0 to DefCount-1 do
                begin
                  Len:=CStrLen(AtTab(i)^.Name^);
                  if (P.X>=X) and (P.X<=X+Len+1) then Index:=i;
                  X:=X+Len+3;
                end;
            if Index<>-1 then
               SelectTab(Index);
          end;
     end;
  if Event.What=evKeyDown then
     begin
       Index:=-1;
       case Event.KeyCode of
            kbTab,kbShiftTab  :
              if GetState(sfSelected) then
                 begin
                   if Current<>nil then
                   begin
                   LastV:=LastSelectable; FirstV:=FirstSelectable;
                   if ((Current=LastV) or (Current=PLabel(LastV)^.Link)) and (Event.KeyCode=kbShiftTab) then
                      begin
                        if Owner<>nil then Owner^.SelectNext(true);
                      end else
                   if ((Current=FirstV) or (Current=PLabel(FirstV)^.Link)) and (Event.KeyCode=kbTab) then
                      begin
                        Lock;
                        if Owner<>nil then Owner^.SelectNext(false);
                        UnLock;
                      end else
                   SelectNext(Event.KeyCode=kbShiftTab);
                   ClearEvent(Event);
                   end;
                 end;
       else
       for I:=0 to DefCount-1 do
           begin
             if Upcase(GetAltChar(Event.KeyCode))=AtTab(I)^.ShortCut
                then begin
                       Index:=I;
                       ClearEvent(Event);
                       Break;
                     end;
           end;
       end;
       if Index<>-1 then
          begin
            Select;
            SelectTab(Index);
            V:=AtTab(ActiveDef)^.DefItem;
            if V<>nil then V^.Focus;
          end;
     end;
  CallOrig:=true;
  if Event.What=evKeyDown then
     begin
     if ((Owner<>nil) and (Owner^.Phase=phPostProcess)
       and (GetAltChar(Event.KeyCode)<>#0)) or GetState(sfFocused)
        then
        else CallOrig:=false;
     end;
  if CallOrig then inherited HandleEvent(Event);
end;

function TTab.GetPalette: PPalette;
begin
  GetPalette:=nil;
end;

{$define AVOIDTHREELINES}

procedure TTab.Draw;
const
{$ifdef AVOIDTHREELINES}
  UDL='¿';
  LUR='Ä';
  URD='Ú';
{$else not AVOIDTHREELINES}
  UDL='´';
  LUR='Á';
  URD='Ã';
{$endif not AVOIDTHREELINES}


var B     : TDrawBuffer;
    i     : integer;
    C1,C2,C3,C : word;
    HeaderLen  : integer;
    X,X2       : integer;
    Name       : PString;
    ActiveKPos : integer;
    ActiveVPos : integer;
    FC   : char;
procedure SWriteBuf(X,Y,W,H: integer; var Buf);
var i: integer;
begin
  if Y+H>Size.Y then H:=Size.Y-Y;
  if X+W>Size.X then W:=Size.X-X;
  if Buffer=nil then WriteBuf(X,Y,W,H,Buf)
                else for i:=1 to H do
                         Move(Buf,Buffer^[X+(Y+i-1)*Size.X],W*2);
end;
procedure ClearBuf;
begin
  MoveChar(B,' ',C1,Size.X);
end;
begin
  if InDraw then Exit;
  InDraw:=true;
  { - Start of TGroup.Draw - }
{  if Buffer = nil then
  begin
    GetBuffer;
  end; }
  { - Start of TGroup.Draw - }

  C1:=GetColor(1);
  C2:=(GetColor(7) and $f0 or $08)+GetColor(9)*256;
  C3:=GetColor(8)+GetColor({9}8)*256;

  { Calculate the size of the headers }
  HeaderLen:=0;
  for i:=0 to DefCount-1 do
    HeaderLen:=HeaderLen+CStrLen(AtTab(i)^.Name^)+3;
  Dec(HeaderLen);
  if HeaderLen>Size.X-2 then HeaderLen:=Size.X-2;

  { --- 1. sor --- }
  ClearBuf;
  MoveChar(B[0],'³',C1,1);
  MoveChar(B[HeaderLen+1],'³',C1,1);
  X:=1;
  for i:=0 to DefCount-1 do
      begin
        Name:=AtTab(i)^.Name; X2:=CStrLen(Name^);
        if i=ActiveDef
           then begin
                  ActiveKPos:=X-1;
                  ActiveVPos:=X+X2+2;
                  if GetState(sfFocused) then C:=C3 else C:=C2;
                end
           else C:=C2;
        MoveCStr(B[X],' '+Name^+' ',C);
        X:=X+X2+3;
        MoveChar(B[X-1],'³',C1,1);
      end;
  SWriteBuf(0,1,Size.X,1,B);

  { --- 0. sor --- }
  ClearBuf; MoveChar(B[0],'Ú',C1,1);
  X:=1;
  for i:=0 to DefCount-1 do
      begin
{$ifdef AVOIDTHREELINES}
        if I<ActiveDef then
          FC:='Ú'
        else
          FC:='¿';
{$else not AVOIDTHREELINES}
        FC:='Â';
{$endif not AVOIDTHREELINES}
        X2:=CStrLen(AtTab(i)^.Name^)+2;
        MoveChar(B[X+X2],FC,C1,1);
        if i=DefCount-1 then X2:=X2+1;
        if X2>0 then
        MoveChar(B[X],'Ä',C1,X2);
        X:=X+X2+1;
      end;
  MoveChar(B[HeaderLen+1],'¿',C1,1);
  MoveChar(B[ActiveKPos],'Ú',C1,1);
  MoveChar(B[ActiveVPos],'¿',C1,1);
  SWriteBuf(0,0,Size.X,1,B);

  { --- 2. sor --- }
  MoveChar(B[1],'Ä',C1,Max(HeaderLen,0));
  MoveChar(B[HeaderLen+2],'Ä',C1,Max(Size.X-HeaderLen-3,0));
  MoveChar(B[HeaderLen+1],LUR,C1,1);
  MoveChar(B[ActiveKPos],'Ù',C1,1);
  if ActiveDef=0 then
    MoveChar(B[0],'³',C1,1)
  else
    MoveChar(B[0],URD,C1,1);
  MoveChar(B[ActiveKPos+1],' ',C1,Max(ActiveVPos-ActiveKPos-1,0));
  MoveChar(B[ActiveVPos],'À',C1,1);
  if HeaderLen+1<Size.X-1 then
    MoveChar(B[Size.X-1],'¿',C1,1)
  else if (ActiveDef=DefCount-1) then
    MoveChar(B[Size.X-1],'³',C1,1)
  else
    MoveChar(B[Size.X-1],UDL,C1,1);
  SWriteBuf(0,2,Size.X,1,B);

  { --- marad‚k sor --- }
  ClearBuf; MoveChar(B[0],'³',C1,1);
  MoveChar(B[Size.X-1],'³',C1,1);
  {SWriteBuf(0,3,Size.X,Size.Y-4,B);}
  for i:=3 to Size.Y-1 do
    SWriteBuf(0,i,Size.X,1,B);

  { --- Size.X . sor --- }
  MoveChar(B[0],'À',C1,1);
  MoveChar(B[1],'Ä',C1,Max(Size.X-2,0));
  MoveChar(B[Size.X-1],'Ù',C1,1);
  SWriteBuf(0,Size.Y-1,Size.X,1,B);

  { - End of TGroup.Draw - }
  if Buffer <> nil then
  begin
    Lock;
    Redraw;
    UnLock;
  end;
  if Buffer <> nil then
    WriteBuf(0, 0, Size.X, Size.Y, Buffer^)
  else
    Redraw;
  { - End of TGroup.Draw - }
  InDraw:=false;
end;

function TTab.Valid(Command: Word): Boolean;
var PT : PTabDef;
    PI : PTabItem;
    OK : boolean;
begin
  OK:=true;
  PT:=TabDefs;
  while (PT<>nil) and (OK=true) do
        begin
          PI:=PT^.Items;
          while (PI<>nil) and (OK=true) do
                begin
                  if PI^.View<>nil then OK:=OK and PI^.View^.Valid(Command);
                  PI:=PI^.Next;
                end;
          PT:=PT^.Next;
        end;
  Valid:=OK;
end;


procedure TTab.SetData(var Rec);
type
  Bytes = array[0..65534] of Byte;
var
  I: Sw_Word;
  PT : PTabDef;
  PI : PTabItem;
begin
  I := 0;
  PT:=TabDefs;
  while (PT<>nil) do
   begin
     PI:=PT^.Items;
     while (PI<>nil) do
      begin
        if PI^.View<>nil then
         begin
           PI^.View^.SetData(Bytes(Rec)[I]);
           Inc(I, PI^.View^.DataSize);
         end;
        PI:=PI^.Next;
      end;
     PT:=PT^.Next;
   end;
end;


function TTab.DataSize: sw_word;
var
  I: Sw_Word;
  PT : PTabDef;
  PI : PTabItem;
begin
  I := 0;
  PT:=TabDefs;
  while (PT<>nil) do
   begin
     PI:=PT^.Items;
     while (PI<>nil) do
      begin
        if PI^.View<>nil then
         begin
           Inc(I, PI^.View^.DataSize);
         end;
        PI:=PI^.Next;
      end;
     PT:=PT^.Next;
   end;
  DataSize:=i;
end;


procedure TTab.GetData(var Rec);
type
  Bytes = array[0..65534] of Byte;
var
  I: Sw_Word;
  PT : PTabDef;
  PI : PTabItem;
begin
  I := 0;
  PT:=TabDefs;
  while (PT<>nil) do
   begin
     PI:=PT^.Items;
     while (PI<>nil) do
      begin
        if PI^.View<>nil then
         begin
           PI^.View^.GetData(Bytes(Rec)[I]);
           Inc(I, PI^.View^.DataSize);
         end;
        PI:=PI^.Next;
      end;
     PT:=PT^.Next;
   end;
end;


procedure TTab.SetState(AState: Word; Enable: Boolean);
var
  LastV : PView;
begin
  inherited SetState(AState,Enable);
  { Select first item }
  if (AState and sfSelected)<>0 then
    begin
      LastV:=LastSelectable;
      if LastV<>nil then
        LastV^.Select;
    end;
end;

destructor TTab.Done;
var P,X: PTabDef;
procedure DeleteViews(P: PView); {$ifndef FPC}far;{$endif}
begin
  if P<>nil then Delete(P);
end;
begin
  ForEach(@DeleteViews);
  inherited Done;
  P:=TabDefs;
  while P<>nil do
        begin
          X:=P^.Next;
          DisposeTabDef(P);
          P:=X;
        end;
end;


function NewTabItem(AView: PView; ANext: PTabItem): PTabItem;
var P: PTabItem;
begin
  New(P); FillChar(P^,SizeOf(P^),0);
  P^.Next:=ANext; P^.View:=AView;
  NewTabItem:=P;
end;

procedure DisposeTabItem(P: PTabItem);
begin
  if P<>nil then
  begin
    if P^.View<>nil then Dispose(P^.View, Done);
    Dispose(P);
  end;
end;

function NewTabDef(AName: string; ADefItem: PView; AItems: PTabItem; ANext: PTabDef): PTabDef;
var P: PTabDef;
    x: byte;
begin
  New(P);
  P^.Next:=ANext; P^.Name:=NewStr(AName); P^.Items:=AItems;
  x:=pos('~',AName);
  if (x<>0) and (x<length(AName)) then P^.ShortCut:=Upcase(AName[x+1])
                                  else P^.ShortCut:=#0;
  P^.DefItem:=ADefItem;
  NewTabDef:=P;
end;

procedure DisposeTabDef(P: PTabDef);
var PI,X: PTabItem;
begin
  DisposeStr(P^.Name);
  PI:=P^.Items;
  while PI<>nil do
    begin
      X:=PI^.Next;
      DisposeTabItem(PI);
      PI:=X;
    end;
  Dispose(P);
end;

procedure RegisterTab;
begin
  RegisterType (RTab);
end;


begin
  RegisterTab;
end.
