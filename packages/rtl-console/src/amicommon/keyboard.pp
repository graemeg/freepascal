{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Karoly Balogh

    Keyboard unit for MorphOS and Amiga and AROS

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Keyboard;
interface

{$i keybrdh.inc}

{
  Amiga specific function, waits for a system event to occur on the
  message port of the window. This is mainly used in Free Vision to
  give up the Task''s timeslice instead of dos.library/Delay() which
  blocks the event handling and ruins proper window refreshing among
  others 
  input: specify a timeout to wait for an event to arrive. this is the
         maximum timeout. the function might return earlier or even
         immediately if there's an event. it's specified in milliseconds
  result: boolean if there is an incoming system event. false otherwise
}

function WaitForSystemEvent(millisec: Integer): boolean;

implementation

uses
   video, exec, intuition, inputevent, mouse, sysutils, keymap, timer;

{$i keyboard.inc}
{$i keyscan.inc}
var
   LastShiftState : Byte;               {set by handler for PollShiftStateEvent}
   OldMouseX : LongInt;
   OldmouseY : LongInt;
   OldButtons: Word;

procedure SysInitKeyboard;
begin
//  writeln('sysinitkeyboard');
{$IFDEF MORPHOS}
  InitKeyMapLibrary;
{$ENDIF}
  LastShiftState := 0;
  OldMouseX := -1;
  OldmouseY := -1;
  OldButtons := 0;
end;

procedure SysDoneKeyboard;
begin

end;

function IsMsgPortEmpty(Port: PMsgPort): Boolean; inline;
begin
  IsMsgPortEmpty := (Port^.mp_MsgList.lh_TailPred = @(Port^.mp_MsgList));
end;

var
  KeyQueue: TKeyEvent;

type 
  RawCodeEntry = record
    rc,n,s,c,a : Word; { raw code, normal, shift, ctrl, alt }
  end;

const
  RCTABLE_MAXIDX = 25;
  RawCodeTable : array[0..RCTABLE_MAXIDX] of RawCodeEntry =
    (
     (rc: 66; n: $0F09; s: $0F00; c: $9400; a: $A500; ), // TAB
     (rc: 68; n: $1C0D; s: $1C0D; c: $1C0A; a: $1C0D; ), // Enter  // shift, alt?
     (rc: 69; n: $011B; s: $011B; c: $011B; a: $0100; ), // ESC    // shift?
     (rc: 70; n: $5300; s: $0700; c: $A300; a: $A200; ), // Delete
     (rc: 71; n: $5200; s: $0500; c: $0400; a: $A200; ), // Insert
     (rc: 72; n: $4900; s: $4900; c: $8400; a: $9900; ), // PgUP   // shift?
     (rc: 73; n: $5100; s: $5100; c: $7600; a: $A100; ), // PgDOWN // shift?

     (rc: 76; n: $4800; s: $4800; c: $8D00; a: $9800; ), // UP     // shift?
     (rc: 77; n: $5000; s: $5000; c: $9100; a: $A000; ), // DOWN   // shift?
     (rc: 78; n: $4D00; s: $4D00; c: $7400; a: $9D00; ), // RIGHT  // shift?
     (rc: 79; n: $4B00; s: $4B00; c: $7300; a: $9B00; ), // LEFT   // shift?
 
     (rc: 80; n: $3B00; s: $5400; c: $5E00; a: $6800; ), // F1
     (rc: 81; n: $3C00; s: $5500; c: $5F00; a: $6900; ), // F2
     (rc: 82; n: $3D00; s: $5600; c: $6000; a: $6A00; ), // F3
     (rc: 83; n: $3E00; s: $5700; c: $6100; a: $6B00; ), // F4
     (rc: 84; n: $3F00; s: $5800; c: $6200; a: $6C00; ), // F5
     (rc: 85; n: $4000; s: $5900; c: $6300; a: $6D00; ), // F6
     (rc: 86; n: $4100; s: $5A00; c: $6400; a: $6E00; ), // F7
     (rc: 87; n: $4200; s: $5B00; c: $6500; a: $6F00; ), // F8
     (rc: 88; n: $4300; s: $5C00; c: $6600; a: $7000; ), // F9
     (rc: 89; n: $4400; s: $5D00; c: $6700; a: $7100; ), // F10
     (rc: 75; n: $8500; s: $8700; c: $8900; a: $8B00; ), // F11
     (rc: 76; n: $8600; s: $8800; c: $8A00; a: $8C00; ), // F12

     (rc: 95;  n: $FF14; s: $FF14; c: $FF14; a: $FF14; ), // Help -> F20
     (rc: 112; n: $4700; s: $4700; c: $7700; a: $9700; ), // Home    // shift?
     (rc: 113; n: $4F00; s: $4F00; c: $7500; a: $9F00; )  // End     // shift?
    );

function rcTableIdx(rc: LongInt): LongInt;
var
  Counter: LongInt;
begin
  rcTableIdx := -1;
  Counter := 0;
  while (RawCodeTable[Counter].rc <> rc) and (Counter <= RCTABLE_MAXIDX) do
    Inc(Counter);
  if (Counter <= RCTABLE_MAXIDX) then
    rcTableIdx := Counter;
end;

function HasShift(IQual: Word): Boolean; inline;
begin
  HasShift := ((IQual and IEQUALIFIER_LSHIFT) <> 0) or
     ((IQual and IEQUALIFIER_RSHIFT) <> 0);
end;

function HasCtrl(IQual: Word): Boolean; inline;
begin
  HasCtrl := ((IQual and IEQUALIFIER_CONTROL) <> 0);
end;

function HasAlt(IQual: Word): Boolean; inline;
begin
  HasAlt := ((IQual and IEQUALIFIER_LALT) <> 0) or
     ((IQual and IEQUALIFIER_RALT) <> 0);
end;

function rcTableCode(IQual: Word; Idx: LongInt): LongInt;
begin
  if (Idx < 0) or (Idx > RCTABLE_MAXIDX) then
  begin
    rcTableCode := -1;
    Exit;
  end;

  if HasShift(IQual) then
    rcTableCode:=RawCodeTable[Idx].s
  else
    if HasCtrl(IQual) then
      rcTableCode:=RawCodeTable[Idx].c
    else
      if HasAlt(IQual) then
        rcTableCode:=RawCodeTable[Idx].a
      else
        rcTableCode:=RawCodeTable[Idx].n;
end;

procedure setShiftState(IQual: Word);
begin
  LastShiftState := 0;
  if ((IQual and IEQUALIFIER_LSHIFT) <> 0) then
    LastShiftState := LastShiftState or $01;
  if ((IQual and IEQUALIFIER_RSHIFT) <> 0) then
    LastShiftState := LastShiftState or $02;
  if HasCtrl(IQual) then
    LastShiftState := LastShiftState or $04;
  if HasAlt(IQual) then
    LastShiftState := LastShiftState or $08;
  if ((IQual and IEQUALIFIER_NUMERICPAD) <> 0) then
    LastShiftState := LastShiftState or $20;
  if ((IQual and IEQUALIFIER_CAPSLOCK) <> 0) then
    LastShiftState := LastShiftState or $40;
end;

function SysPollKeyEvent: TKeyEvent;
var
  MouseEvent: Boolean;   // got a mouseevent -> do not leave cycle
  SendMouse: Boolean;    // we got a (or many) mouse move  send the last one
  mes: TMouseEvent;      // save mouse message send after cycle -> prevent mouse move stacking
  me: TMouseEvent;
  KeyCode: LongInt;
  OldKeyCode: LongInt;
  KeySet: ^TKeyRecord;   // points to result to set fields directly
  Ret: LongInt;
  //
  iMsg: PIntuiMessage;
  ICode: Word;           // save items from Message
  IQual: Word;
  IClass: Longword;
  MouseX: LongInt;
  MouseY: LongInt;
  KeyUp: Boolean;        // Event is a key up event
  Buff: array[0..19] of Char;
  ie: TInputEvent;       // for mapchar
  IAddr: Pointer;
begin
  KeyCode := 0;
  SysPollKeyEvent := 0;
  KeySet := @SysPollKeyEvent;
  FillChar(me, SizeOf(TMouseEvent), 0);

  if KeyQueue <> 0 then
  begin
    SysPollKeyEvent := KeyQueue;
    Exit;
  end;
  SendMouse := False;
  repeat
    MouseEvent := False;
    if VideoWindow <> nil then
    begin
      if IsMsgPortEmpty(videoWindow^.UserPort) then
        Break;
    end else
      Exit;
    PMessage(iMsg) := GetMsg(VideoWindow^.UserPort);
    if (iMsg <> nil) then
    begin
      ICode := iMsg^.Code;
      IQual := iMsg^.Qualifier;
      IClass := iMsg^.iClass;
      MouseX := iMsg^.MouseX;
      MouseY := iMsg^.MouseY;
      IAddr := iMsg^.IAddress;
      ReplyMsg(PMessage(iMsg)); // fast reply to system
      SetShiftState(IQual); // set Shift state qualifiers. do this for all messages we get.
      // main event case
      case (IClass) of
        IDCMP_ACTIVEWINDOW: begin
            GotActiveWindow;
          end;
        IDCMP_INACTIVEWINDOW: begin
            // force cursor off. we stop getting IntuiTicks when 
            // the window is inactive, so the blinking stops.
            ToggleCursor(true);
            GotInactiveWindow;
          end;
        IDCMP_INTUITICKS: begin
            ToggleCursor(false);
            TranslateToCharXY(MouseX - VideoWindow^.BorderLeft, MouseY - VideoWindow^.BorderTop, MouseX, MouseY);
            if (MouseX >= 0) and (MouseY >= 0) and
               (MouseX < Video.ScreenWidth) and (MouseY < Video.ScreenHeight) and
               ((MouseX <> OldMouseX) or (MouseY <> OldmouseY))
              then begin
//              //writeln('mousemove:',Mousex,'/',Mousey,' oldbutt:',OldButtons);
              // Drawing is very slow so when moving window it will drag behind
              // because the mouse events stack in the messageport
              // -> so we override move until messageport is empty or keyevent is fired
              SendMouse := True;
              MouseEvent := True;
              mes.Action := MouseActionMove;
              mes.Buttons := OldButtons;
              mes.X := MouseX;
              mes.Y := MouseY;
              //PutMouseEvent(me);
            end;
          end;
        IDCMP_CLOSEWINDOW: begin
            //writeln('got close');
            GotCloseWindow;
          end;
        IDCMP_CHANGEWINDOW: begin
            GotResizeWindow;
          end;
        IDCMP_REFRESHWINDOW: begin
            GotRefreshWindow;
          end;
        IDCMP_MOUSEBUTTONS: begin
            MouseEvent := True;
            TranslateToCharXY(MouseX - videoWindow^.BorderLeft, MouseY - videoWindow^.BorderTop, MouseX, MouseY);
            me.x := MouseX;
            me.y := MouseY;
            case ICode of
              SELECTDOWN: begin
                  //writeln('left down!');
                  me.Action := MouseActionDown;
                  OldButtons := OldButtons or MouseLeftButton;
                  me.Buttons := OldButtons;
                  PutMouseEvent(me);
                end;
              SELECTUP: begin
                  //writeln('left up!');
                  me.Action := MouseActionUp;
                  OldButtons := OldButtons and (not MouseLeftButton);
                  me.Buttons := OldButtons;
                  PutMouseEvent(me);
                end;
              MENUDOWN: begin
                  //writeln('right down!');
                  me.Action := MouseActionDown;
                  OldButtons := OldButtons or MouseRightButton;
                  me.Buttons := OldButtons;
                  PutMouseEvent(me);
                end;
              MENUUP: begin
                  //writeln('right up!');
                  me.Action := MouseActionUp;
                  OldButtons := OldButtons and (not MouseRightButton);
                  me.Buttons := OldButtons;
                  PutMouseEvent(me);
                end;
            end;
            //writeln('Buttons: ' , me.Buttons);
          end;
        IDCMP_MOUSEMOVE: begin
            { IDCMP_MOUSEMOVE is disabled now in the video unit,
              according to autodocs INTUITICKS should be enough
              to handle most moves, esp. in a "textmode" app }
            TranslateToCharXY(MouseX - VideoWindow^.BorderLeft, MouseY - VideoWindow^.BorderTop, MouseX, MouseY);
            if (MouseX >= 0) and (MouseY >= 0) and
               (MouseX < Video.ScreenWidth) and (MouseY < Video.ScreenHeight) and
               ((MouseX <> OldMouseX) or (MouseY <> OldmouseY))
              then begin
//              //writeln('mousemove:',Mousex,'/',Mousey,' oldbutt:',OldButtons);
              // Drawing is very slow so when moving window it will drag behind
              // because the mouse events stack in the messageport
              // -> so we override move until messageport is empty or keyevent is fired
              SendMouse := True;
              MouseEvent := True;
              mes.Action := MouseActionMove;
              mes.Buttons := OldButtons;
              mes.X := MouseX;
              mes.Y := MouseY;
              //PutMouseEvent(me);
            end;
          end;
        IDCMP_RAWKEY: begin
          // mouse wheel up or down -> pgup and pgdown
          if ICode = 122 then
            ICode := 72;
          if ICode = 123 then
            ICode := 73;
          // get char from rawkey
          KeyUp := (ICode and IECODE_UP_PREFIX) <> 0;   // is key up
          ICode := ICode and not IECODE_UP_PREFIX;      // remove key up from ICode
          ie.ie_Class := IECLASS_RAWKEY;
          ie.ie_SubClass := 0;
          ie.ie_Code := ICode;
          ie.ie_Qualifier := IQual;
          ie.ie_NextEvent := nil;
          ie.ie_position.ie_addr := IAddr;
          Buff[0] := #0;
          Ret := MapRawKey(@ie, @Buff[0], 1, nil);
          KeyCode := Ord(Buff[0]);
          KeySet^.KeyCode := Ord(Buff[0]);         // if maprawkey does not work it still is 0
          KeySet^.ShiftState := LastShiftState;    // shift state set before the case
          KeySet^.Flags := 0;
          if keyup then                            // we do not need key up events up to now
          begin
            KeySet^.Flags := KeySet^.Flags or kbReleased; // kbReleased does work but make strange effects
            SysPollKeyEvent := 0;
            Exit;
          end;
          // check our hard coed list if there is an entry -> leave it must be right ;)
          // F-keys, cursor, esc, del, ins, del, pgup, pgdown, pos, end, enter, tab
          if rcTableCode(IQual,rcTableIdx(ICode)) >= 0 then
          begin
            KeyCode := rcTableCode(IQual,rcTableIdx(ICode));
            KeySet^.KeyCode := KeyCode;
            KeySet^.Flags := kbPhys;
          end else
          begin
            // left alt or ctrl is pressed -> check for alternative Scancode -> commando
            if ((IQual and IEQUALIFIER_LALT) <> 0) or  HasCtrl(IQual) then
            begin
              OldKeyCode := KeyCode;  // save keycode if nothing found
              KeyCode := 0;
              ie.ie_Class := IECLASS_RAWKEY;    // get keycode without qualifier easier case
              ie.ie_SubClass := 0;
              ie.ie_Code := ICode;
              ie.ie_Qualifier := 0;
              ie.ie_NextEvent := nil;
              ie.ie_position.ie_addr := IAddr;
              Buff[0] := #0;
              Ret := MapRawKey(@ie, @Buff[0], 1, nil);
              if Ret > 0 then
              begin
                if ((IQual and IEQUALIFIER_LALT) <> 0) then   // check left alt keycodes
                begin
                  case Buff[0] of        // Alt - keys already defined
                    'a': KeyCode := kbAltA shl 8;
                    'b': KeyCode := kbAltB shl 8;
                    'c': KeyCode := kbAltC shl 8;
                    'd': KeyCode := kbAltD shl 8;
                    'e': KeyCode := kbAltE shl 8;
                    'f': KeyCode := kbAltF shl 8;
                    'g': KeyCode := kbAltG shl 8;
                    'h': KeyCode := kbAltH shl 8;
                    'i': KeyCode := kbAltI shl 8;
                    'j': KeyCode := kbAltJ shl 8;
                    'k': KeyCode := kbAltK shl 8;
                    'l': KeyCode := kbAltL shl 8;
                    'm': KeyCode := kbAltM shl 8;
                    'n': KeyCode := kbAltN shl 8;
                    'o': KeyCode := kbAltO shl 8;
                    'p': KeyCode := kbAltP shl 8;
                    'q': KeyCode := kbAltQ shl 8;
                    'r': KeyCode := kbAltR shl 8;
                    's': KeyCode := kbAltS shl 8;
                    't': KeyCode := kbAltT shl 8;
                    'u': KeyCode := kbAltU shl 8;
                    'v': KeyCode := kbAltV shl 8;
                    'w': KeyCode := kbAltW shl 8;
                    'x': KeyCode := kbAltX shl 8;
                    'y': KeyCode := kbAltY shl 8;
                    'z': KeyCode := kbAltZ shl 8;
                  end;
                end else
                begin
                  case Buff[0] of      // ctrl - keys defined in FreeVision/drivers.pas -> so here direct numbers
                    'a': KeyCode := $1E01;
                    'b': KeyCode := $3002;
                    'c': KeyCode := $2E03;
                    'd': KeyCode := $2004;
                    'e': KeyCode := $1205;
                    'f': KeyCode := $2106;
                    'g': KeyCode := $2207;
                    'h': KeyCode := $2308;
                    'i': KeyCode := $1709;
                    'j': KeyCode := $240a;
                    'k': KeyCode := $250b;
                    'l': KeyCode := $260c;
                    'm': KeyCode := $320d;
                    'n': KeyCode := $310e;
                    'o': KeyCode := $180f;
                    'p': KeyCode := $1910;
                    'q': KeyCode := $1011;
                    'r': KeyCode := $1312;
                    's': KeyCode := $1F13;
                    't': KeyCode := $1414;
                    'u': KeyCode := $1615;
                    'v': KeyCode := $2F16;
                    'w': KeyCode := $1117;
                    'x': KeyCode := $2D18;
                    'y': KeyCode := $1519;
                    'z': KeyCode := $2C1A;
                  end;
                end;
              end;
              if KeyCode <= 0 then      // nothing found restore keycode
                KeyCode := OldKeyCode;
              KeySet^.KeyCode := KeyCode;
              KeySet^.Flags := kbPhys;
            end;
          end;
          if keycode <= 0 then
          begin
            KeySet^.KeyCode := 0;
            KeyCode := 0;
          end;
          //writeln('raw keycode: ',iMsg^.code, ' -> $', IntToHex(keycode,4), ' ret: ', ret);
        end;
        else begin
          KeyCode := 0;
        end;
      end;
    end else
      Break;
  until (not MouseEvent);
  //
  if SendMouse then
  begin
    PutMouseEvent(mes);
    OldMouseX:=Mousex;
    OldmouseY:=Mousey;
  end;
  if KeyCode <= 0 then     // no keycode found then also delete flags and shiftstate
    SysPollKeyEvent := 0
  else
    KeyQueue:=SysPollKeyEvent;
end;

function SysGetKeyEvent: TKeyEvent;
var
  Res: TKeyEvent;
  me: TMouseEvent;
begin
  Res := 0;
  if VideoWindow <> nil then
  begin
    if KeyQueue <> 0 then
    begin
      SysGetKeyEvent := KeyQueue;
      KeyQueue := 0;
      Exit;
    end;
    repeat
      WaitPort(VideoWindow^.UserPort);
      Res := SysPollKeyEvent;
      // remove event from KeyQueue, because we return it here,
      // else we get double keys if GetKeyevent is called without a PollKeyEvent called first
      KeyQueue := 0; 
    until Res <> 0;
  end else
  begin
    me.Action := MouseActionDown;
    me.Buttons := MouseRightButton;
    PutMouseEvent(me);
  end;
  SysGetKeyEvent := Res;
end;

{function SysTranslateKeyEvent(KeyEvent: TKeyEvent): TKeyEvent;
begin
end;}

function SysGetShiftState: Byte;
begin
  //writeln('SysgetShiftState:',hexstr(LastShiftState,2));
  SysGetShiftState := LastShiftState;
end;

var
  waitTPort:  PMsgPort;
  waitTimer: PTimeRequest;
  waitTimerFired: boolean;

function WaitForSystemEvent(millisec: Integer): boolean;
var
  windowbit: PtrUInt;
  timerbit: PtrUInt;
  recvbits: PtrUInt;
begin
  WaitForSystemEvent:=false;
  if waitTPort = nil then
  begin
    { this really shouldn't happen, but it's enough to avoid a
      crash if the timer init failed during startup }
    if VideoWindow <> nil then
      WaitPort(VideoWindow^.UserPort);
    exit;
  end;

  windowbit:=0;
  if VideoWindow <> nil then
  begin
    if not IsMsgPortEmpty(VideoWindow^.UserPort) then
    begin
      WaitForSystemEvent:=true;
      exit;
    end;
    windowbit:=1 shl (VideoWindow^.UserPort^.mp_SigBit);
  end;
  timerbit:=0;
  if waitTPort <> nil then
    timerbit:=1 shl (waitTPort^.mp_SigBit);
  if (windowbit or timerbit) = 0 then exit;

  if not waitTimerFired then
  begin
    waitTimer^.tr_node.io_Command:=TR_ADDREQUEST;
    waitTimer^.tr_time.tv_secs:=millisec div 1000;
    waitTimer^.tr_time.tv_micro:=(millisec mod 1000) * 1000;
    SendIO(PIORequest(waitTimer));
    waitTimerFired:=true;
  end;

  recvbits:=Wait(windowbit or timerbit);
  if (recvbits and windowbit) > 0 then
    WaitForSystemEvent:=true;

  if waitTimerFired then 
  begin
    AbortIO(PIORequest(waitTimer));
    WaitIO(PIORequest(waitTimer));
    SetSignal(0,timerbit);
    waitTimerFired:=false;
  end;
end;

procedure DoneSystemEventWait;
begin
  if assigned(waitTimer) then
  begin
    if waitTimerFired then 
    begin
      AbortIO(PIORequest(waitTimer));
      WaitIO(PIORequest(waitTimer));
      waitTimerFired:=false;
    end;
    CloseDevice(PIORequest(waitTimer));
    DeleteIORequest(PIORequest(waitTimer));
    waitTimer:=nil;
  end;
  if assigned(waitTPort) then
  begin
    DeleteMsgPort(waitTPort);
    waitTPort:=nil;
  end;
end;

procedure InitSystemEventWait;
var
  initOK: boolean;
begin
  waitTimerFired:=false;
  waitTPort:=CreateMsgPort();
  if assigned(waitTPort) then
  begin
    waitTimer:=PTimeRequest(CreateIORequest(waitTPort,sizeof(TTimeRequest)));
    if assigned(waitTimer) then
    begin
      if OpenDevice(TIMERNAME,UNIT_VBLANK,PIORequest(waitTimer),0) = 0 then
      begin
        initOK:=true;
        waitTimerFired:=false;
      end;
    end;
  end;
  if not initOK then begin
    {* this really shouldn't happen if everything is OK with the system *}
    SysDebugLn('FPC RTL-Console: SystemEventWait Initialization failed!');
    DoneSystemEventWait;
  end;
end;


const
  SysKeyboardDriver : TKeyboardDriver = (
    InitDriver : @SysInitKeyBoard;
    DoneDriver : @SysDoneKeyBoard;
    GetKeyevent : @SysGetKeyEvent;
    PollKeyEvent : @SysPollKeyEvent;
    GetShiftState : @SysGetShiftState;
//    TranslateKeyEvent : @SysTranslateKeyEvent;
    TranslateKeyEvent : Nil;
    TranslateKeyEventUnicode : Nil;
  );


initialization
  SetKeyBoardDriver(SysKeyBoardDriver);
  InitSystemEventWait;
finalization
  DoneSystemEventWait;
end.
