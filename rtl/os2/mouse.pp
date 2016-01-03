{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Mouse unit for OS/2

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Mouse;
interface

{$i mouseh.inc}

const
  MouseIsVisible: boolean = false;

implementation

uses
 Video,
 MouCalls, DosCalls;

{$i mouse.inc}

var
 PendingMouseEventOrder: array [0..MouseEventBufSize-1] of cardinal;
 MouseEventOrderHead, MouseEventOrderTail: cardinal;

const
 NoMouse = $FFFF;
 DefaultMouse = 0;
 Handle: word = DefaultMouse;
 HideCounter: cardinal = 0;
 OldEventMask: longint = -1;

procedure SysInitMouse;
var
 Loc: TPtrLoc;
 SetPrev: boolean;
 SysEvent: TMouEventInfo;
 QI: TMouQueInfo;
 W: word;
begin
 SetPrev := MouGetPtrPos (Loc, DefaultMouse) = 0;
 if MouGetEventMask (W, DefaultMouse) = 0 then OldEventMask := W;
 MouseEventOrderTail := 0;
 MouseEventOrderHead := 0;
 HideCounter := 0;
 if MouOpen (nil, Handle) = Error_Mouse_No_Device then Handle := NoMouse else
 begin
  W := Mou_NoWait;
  repeat
   MouGetNumQueEl (QI, Handle);
   if QI.cEvents <> 0 then MouReadEventQue (SysEvent, W, Handle);
  until QI.cEvents = 0;
  W := $FFFF;
  MouSetEventMask (W, Handle);
  if SetPrev then MouSetPtrPos (Loc, Handle);

(*
 It would be possible to issue a MouRegister call here to hook our own mouse
 handler, but such handler would have to be in a DLL and it is questionable,
 whether there would be so many advantages in doing so.
*)

  MouDrawPtr (Handle);
  MouseIsVisible := true;
 end;
end;

procedure SysDoneMouse;
var
 W: word;
begin
 if (Handle <> NoMouse) and (Handle <> DefaultMouse) then
 begin

(*
 If our own mouse handler would be installed in InitMouse, MouDeregister would
 have appeared here.
*)

  HideCounter := 0;
  HideMouse;
  MouClose (Handle);
 end;
 if OldEventMask <> -1 then
 begin
  W := OldEventMask;
  MouSetEventMask (W, 0);
 end;
end;

function SysDetectMouse:byte;
var
 Buttons: word;
 TempHandle: word;
begin
 MouOpen (nil, TempHandle);
 if MouGetNumButtons (Buttons, TempHandle) = 0 then
   SysDetectMouse := Buttons
 else
   SysDetectMouse := 0;
 MouClose (TempHandle);
end;

procedure SysShowMouse;
begin
 if Handle <> NoMouse then
 begin
  if HideCounter <> 0 then
  begin
   Dec (HideCounter);
   if HideCounter = 0 then MouDrawPtr (Handle);
   MouseIsVisible := true;
  end;
 end;
end;

procedure SysHideMouse;
var
 PtrRect: TNoPtrRect;
begin
 if Handle <> NoMouse then
 begin
  Inc (HideCounter);
  case HideCounter of
   0: Dec (HideCounter); (* HideCounter overflowed - stay at the maximum *)
   1: begin
       PtrRect.Row := 0;
       PtrRect.Col := 0;
       PtrRect.cRow := Pred (ScreenHeight);
       PtrRect.cCol := Pred (ScreenWidth);
       MouRemovePtr (PtrRect, Handle);
       MouseIsVisible := false;
      end;
  end;
 end;
end;

function SysGetMouseX: word;
var
 Event: TMouseEvent;
begin
 if Handle = NoMouse then
   SysGetMouseX := 0
 else
   begin
   PollMouseEvent (Event);
   SysGetMouseX := Event.X;
   end;
end;

function SysGetMouseY: word;
var
 Event: TMouseEvent;
begin
 if Handle = NoMouse then
   SysGetMouseY := 0
 else
   begin
   PollMouseEvent (Event);
   SysGetMouseY := Event.Y;
   end;
end;

procedure SysGetMouseXY (var X: word; var Y: word);
var
 Loc: TPtrLoc;
begin
 if Handle = NoMouse then
 begin
  X := 0;
  Y := 0;
 end else if MouGetPtrPos (Loc, Handle) <> 0 then
 begin
  X := $FFFF;
  Y := $FFFF;
 end else
 begin
  X := Loc.Col;
  Y := Loc.Row;
 end;
end;

procedure SysSetMouseXY (X, Y: word);
var
 Loc: TPtrLoc;
begin
 if Handle <> NoMouse then
 begin
  Loc.Row := Y;
  Loc.Col := X;
  MouSetPtrPos (Loc, Handle);
 end;
end;

procedure TranslateEvents (const SysEvent: TMouEventInfo;
                                                       var Event: TMouseEvent);
begin
 Event.Buttons := 0;
 Event.Action := 0;
 if SysEvent.fs and (Mouse_Motion_With_BN1_Down or Mouse_BN1_Down) <> 0 then
                             Event.Buttons := Event.Buttons or MouseLeftButton;
 if SysEvent.fs and (Mouse_Motion_With_BN2_Down or Mouse_BN2_Down) <> 0 then
                            Event.Buttons := Event.Buttons or MouseRightButton;
 if SysEvent.fs and (Mouse_Motion_With_BN3_Down or Mouse_BN3_Down) <> 0 then
                           Event.Buttons := Event.Buttons or MouseMiddleButton;
 Event.X := SysEvent.Col;
 Event.Y := SysEvent.Row;
 if Event.Buttons <> LastMouseEvent.Buttons then
  if (Event.Buttons and MouseLeftButton = 0) and
      (LastMouseEvent.Buttons and MouseLeftButton = MouseLeftButton)
                                   then Event.Action := MouseActionUp else
  if (Event.Buttons and MouseRightButton = 0) and
      (LastMouseEvent.Buttons and MouseRightButton = MouseRightButton)
                                   then Event.Action := MouseActionUp else
  if (Event.Buttons and MouseMiddleButton = 0) and
   (LastMouseEvent.Buttons and MouseMiddleButton = MouseMiddleButton)
    then Event.Action := MouseActionUp
     else Event.Action := MouseActionDown
      else if (Event.X <> LastMouseEvent.X) or (Event.Y <> LastMouseEvent.Y)
                                          then Event.Action := MouseActionMove;
 LastMouseEvent := Event;
end;

procedure NullOrder;
var
 I: cardinal;
begin
 if PendingMouseEvents > 0 then
 begin
  I := MouseEventOrderHead;
  repeat
   PendingMouseEventOrder [I] := 0;
   if I = Pred (MouseEventBufSize) then I := 0 else Inc (I);
  until (I <> MouseEventOrderTail);
 end;
end;

procedure LowerOrder;
var
 I: cardinal;
begin
 if PendingMouseEvents > 0 then
 begin
  I := MouseEventOrderHead;
  repeat
   if PendingMouseEventOrder [I] <> 0 then
   begin
    Dec (PendingMouseEventOrder [I]);
    if I = Pred (MouseEventBufSize) then I := 0 else Inc (I);
   end;
  until (I <> MouseEventOrderTail) or (PendingMouseEventOrder [I] = 0);
 end;
end;

function SysPollMouseEvent (var MouseEvent: TMouseEvent) :boolean;
var
 SysEvent: TMouEventInfo;
 P, Q: PMouseEvent;
 Event: TMouseEvent;
 WF: word;
 QI: TMouQueInfo;
begin
 if (PendingMouseEvents = 0) or
         (PendingMouseEventOrder [MouseEventOrderHead] <> 0) and
                                  (PendingMouseEvents < MouseEventBufSize) then
 begin
  MouGetNumQueEl (QI, Handle);
  if QI.cEvents = 0 then NullOrder else
  begin
   LowerOrder;
   WF := Mou_NoWait;
   if (MouReadEventQue (SysEvent, WF, Handle) = 0) then
   begin
    if PendingMouseHead = @PendingMouseEvent[0] then
                           P := @PendingMouseEvent [MouseEventBufSize - 1] else
    begin
     P := PendingMouseHead;
     Dec (P);
    end;
    TranslateEvents (SysEvent, P^);
    if P^.Action <> 0 then
    begin
     if PendingMouseEvents < MouseEventBufSize then
     begin
      Q := P;
      WF := Mou_NoWait;
      while (P^.Action = MouseActionMove) and
       (PendingMouseEventOrder [MouseEventOrderHead] <> 0) and
         (MouReadEventQue (SysEvent, WF, Handle) = 0) and
                       ((SysEvent.fs <> 0) or (LastMouseEvent.Buttons <> 0)) do
      begin
       LowerOrder;
       TranslateEvents (SysEvent, Event);
       if Event.Action <> MouseActionMove then
       begin
        if Q = @PendingMouseEvent[0] then
                  Q := @PendingMouseEvent [MouseEventBufSize - 1] else Dec (Q);
        if MouseEventOrderHead = 0 then
                  MouseEventOrderHead := MouseEventBufSize - 1 else
                                                     Dec (MouseEventOrderHead);
        PendingMouseEventOrder [MouseEventOrderHead] := 0;
        Q^ := P^;
        Inc (PendingMouseEvents);
        if MouseEventOrderHead = 0 then
               MouseEventOrderHead := MouseEventBufSize - 1 else
                                                     Dec (MouseEventOrderHead);
        PendingMouseEventOrder [MouseEventOrderHead] := 0;
       end else WF := Mou_NoWait;
       P^ := Event;
      end;
      P := Q;
     end;
     Inc (PendingMouseEvents);
     if MouseEventOrderHead = 0 then
               MouseEventOrderHead := MouseEventBufSize - 1 else
                                                     Dec (MouseEventOrderHead);
     PendingMouseEventOrder [MouseEventOrderHead] := 0;
     PendingMouseHead := P;
    end;
   end else NullOrder;
  end;
 end;
 if PendingMouseEvents <> 0 then
 begin
  MouseEvent := PendingMouseHead^;
  LastMouseEvent := MouseEvent;
  SysPollMouseEvent := true;
 end else
 begin
  SysPollMouseEvent := false;
  MouseEvent := LastMouseEvent;
  MouseEvent.Action := 0;
 end;
end;

function SysGetMouseButtons: word;
var
 Event: TMouseEvent;
begin
 PollMouseEvent (Event);
 SysGetMouseButtons := Event.Buttons;
end;

procedure SysGetMouseEvent (var MouseEvent: TMouseEvent);
begin
 if (PendingMouseEvents = 0) or
                       (PendingMouseEventOrder [MouseEventOrderHead] <> 0) then
 repeat
  DosSleep (1);
  PollMouseEvent (MouseEvent);
 until (PendingMouseEvents <> 0) and
                        (PendingMouseEventOrder [MouseEventOrderHead] = 0) else
 begin
  MouseEvent := PendingMouseHead^;
  LastMouseEvent := MouseEvent;
 end;
 Inc (PendingMouseHead);
 if PendingMouseHead = @PendingMouseEvent[0]+MouseEventBufsize then
   PendingMouseHead := @PendingMouseEvent[0];
 Inc (MouseEventOrderHead);
 if MouseEventOrderHead = MouseEventBufSize then MouseEventOrderHead := 0;
 Dec (PendingMouseEvents);
end;

procedure SysPutMouseEvent (const MouseEvent: TMouseEvent);
var
 QI: TMouQueInfo;
begin
 if PendingMouseEvents < MouseEventBufSize then
 begin
  PendingMouseTail^ := MouseEvent;
  Inc (PendingMouseTail);
  if PendingMouseTail=@PendingMouseEvent[0]+MouseEventBufSize then
    PendingMouseTail := @PendingMouseEvent[0];
  MouGetNumQueEl (QI, Handle);
  PendingMouseEventOrder [MouseEventOrderTail] := QI.cEvents;
  Inc (MouseEventOrderTail);
  if MouseEventOrderTail = MouseEventBufSize then MouseEventOrderTail := 0;
  Inc (PendingMouseEvents);
 end;
end;

Const
  SysMouseDriver : TMouseDriver = (
    UseDefaultQueue : False;
    InitDriver      : @SysInitMouse;
    DoneDriver      : @SysDoneMouse;
    DetectMouse     : @SysDetectMouse;
    ShowMouse       : @SysShowMouse;
    HideMouse       : @SysHideMouse;
    GetMouseX       : @SysGetMouseX;
    GetMouseY       : @SysGetMouseY;
    GetMouseButtons : @SysGetMouseButtons;
    SetMouseXY      : @SysSetMouseXY;
    GetMouseEvent   : @SysGetMouseEvent;
    PollMouseEvent  : @SysPollMouseEvent;
    PutMouseEvent   : @SysPutMouseEvent;
  );

Begin
  SetMouseDriver(SysMouseDriver);
end.
