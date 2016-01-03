{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

var
  PendingKeyEvent : TKeyEvent;

procedure PutKeyEvent(KeyEvent: TKeyEvent);
begin
  PendingKeyEvent := KeyEvent;
end;

function GetKeyEventFlags(KeyEvent: TKeyEvent): Byte;
begin
  GetKeyEventFlags := (KeyEvent and $FF000000) shr 24;
end;

function GetKeyEventChar(KeyEvent: TKeyEvent): Char;
begin
  if KeyEvent and $03000000 = $00000000 then
    GetKeyEventChar := Chr(KeyEvent and $000000FF)
   else
    GetKeyEventChar := #0;
end;

function GetKeyEventUniCode(KeyEvent: TKeyEvent): Word;
begin
  if KeyEvent and $03000000 = $01000000 then
    GetKeyEventUniCode := KeyEvent and $0000FFFF
   else
    GetKeyEventUniCode := 0;
end;

function GetKeyEventCode(KeyEvent: TKeyEvent): Word;
begin
  GetKeyEventCode := KeyEvent and $0000FFFF
end;

function GetKeyEventShiftState(KeyEvent: TKeyEvent): Byte;
begin
  GetKeyEventShiftState := (KeyEvent and $00FF0000) shr 16;
end;

function IsFunctionKey(KeyEvent: TKeyEvent): Boolean;
begin
  IsFunctionKey := KeyEvent and $03000000 = $02000000;
end;

Var
  KeyBoardInitialized : Boolean;
  CurrentKeyboardDriver : TKeyboardDriver;

procedure InitKeyboard;

begin
  If Not KeyboardInitialized then
    begin
    If Assigned(CurrentKeyBoardDriver.InitDriver) Then
      CurrentKeyBoardDriver.InitDriver();
    KeyboardInitialized:=True;
    end;
end;

procedure DoneKeyboard;

begin
  If KeyboardInitialized then
    begin
    If Assigned(CurrentKeyBoardDriver.DoneDriver) Then
      CurrentKeyBoardDriver.DoneDriver();
    KeyboardInitialized:=False;
    end;
end;


function GetKeyEvent: TKeyEvent;

begin
  if PendingKeyEvent<>0 then
    begin
    GetKeyEvent:=PendingKeyEvent;
    PendingKeyEvent:=0;
    exit;
    end;
  If Assigned(CurrentKeyBoardDriver.GetKeyEvent) Then
    GetKeyEvent:=CurrentKeyBoardDriver.GetKeyEvent()
  else
    GetKeyEvent:=0;
end;

function PollKeyEvent: TKeyEvent;

begin
  if PendingKeyEvent<>0 then
   exit(PendingKeyEvent);
  If Assigned(CurrentKeyBoardDriver.PollKeyEvent) Then
    begin
    PollKeyEvent:=CurrentKeyBoardDriver.PollKeyEvent();
    // PollKeyEvent:=PendingKeyEvent;
    // Must be done inside every keyboard specific
    // PollKeyEvent procedure
    // to avoid problems if that procedure is called directly PM
    end
  else
    PollKeyEvent:=0;
end;

Function SetKeyboardDriver (Const Driver : TKeyboardDriver) : Boolean;

begin
  If Not KeyBoardInitialized then
    CurrentKeyBoardDriver:=Driver;
  SetKeyboardDriver:=Not KeyBoardInitialized;
end;

Procedure GetKeyboardDriver (Var Driver : TKeyboardDriver);

begin
  Driver:=CurrentKeyBoardDriver;
end;

function PollShiftStateEvent: TKeyEvent;

begin
  If Assigned(CurrentKeyBoardDriver.GetShiftState) then
    PollShiftStateEvent:=CurrentKeyBoardDriver.GetShiftState() shl 16
  else
    PollShiftStateEvent:=0;
end;

function DefaultTranslateKeyEventUniCode(KeyEvent: TKeyEvent): TKeyEvent;

begin
  DefaultTranslateKeyEventUniCode:=KeyEvent;
  ErrorCode:=errKbdNotImplemented;
end;


function TranslateKeyEventUniCode(KeyEvent: TKeyEvent): TKeyEvent;

begin
  if Assigned(CurrentKeyBoardDriver.TranslateKeyEventUnicode) then
    TranslateKeyEventUnicode:=CurrentKeyBoardDriver.TranslateKeyEventUnicode(KeyEvent)
  else
    TranslateKeyEventUnicode:=DefaultTranslateKeyEventUnicode(KeyEvent);
end;

type
  TTranslationEntry = packed record
    Min, Max: Byte;
    Offset: Word;
  end;
const
  TranslationTableEntries = 12;
  TranslationTable: array [1..TranslationTableEntries] of TTranslationEntry =
    ((Min: $3B; Max: $44; Offset: kbdF1),   { function keys F1-F10 }
     (Min: $54; Max: $5D; Offset: kbdF1),   { Shift fn keys F1-F10 }
     (Min: $5E; Max: $67; Offset: kbdF1),   { Ctrl fn keys F1-F10 }
     (Min: $68; Max: $71; Offset: kbdF1),   { Alt fn keys F1-F10 }
     (Min: $85; Max: $86; Offset: kbdF11),  { function keys F11-F12 }
     (Min: $87; Max: $88; Offset: kbdF11),  { Shift+function keys F11-F12 }
     (Min: $89; Max: $8A; Offset: kbdF11),  { Ctrl+function keys F11-F12 }
     (Min: $8B; Max: $8C; Offset: kbdF11),  { Alt+function keys F11-F12 }
     (Min: $47; Max: $49; Offset: kbdHome), { Keypad keys kbdHome-kbdPgUp }
     (Min: $4B; Max: $4D; Offset: kbdLeft), { Keypad keys kbdLeft-kbdRight }
     (Min: $4F; Max: $51; Offset: kbdEnd),  { Keypad keys kbdEnd-kbdPgDn }
     (Min: $52; Max: $53; Offset: kbdInsert));


function DefaultTranslateKeyEvent(KeyEvent: TKeyEvent): TKeyEvent;

var
  I: Integer;
  ScanCode: Byte;
begin
  if KeyEvent and $03000000 = $03000000 then
   begin
     if KeyEvent and $000000FF <> 0 then
      begin
        DefaultTranslateKeyEvent := KeyEvent and $00FFFFFF;
        exit;
      end
     else
      begin
        { This is a function key }
        ScanCode := (KeyEvent and $0000FF00) shr 8;
        for I := 1 to TranslationTableEntries do
         begin
           if (TranslationTable[I].Min <= ScanCode) and (ScanCode <= TranslationTable[I].Max) then
            begin
              DefaultTranslateKeyEvent := $02000000 + (KeyEvent and $00FF0000) +
                Byte(ScanCode - TranslationTable[I].Min) + TranslationTable[I].Offset;
              exit;
            end;
         end;
      end;
   end;
  DefaultTranslateKeyEvent := KeyEvent;
end;

function TranslateKeyEvent(KeyEvent: TKeyEvent): TKeyEvent;

begin
  if Assigned(CurrentKeyBoardDriver.TranslateKeyEvent) then
    TranslateKeyEvent:=CurrentKeyBoardDriver.TranslateKeyEvent(KeyEvent)
  else
    TranslateKeyEvent:=DefaultTranslateKeyEvent(KeyEvent);
end;

{ ---------------------------------------------------------------------
    KeyEvent to String representation section.
  ---------------------------------------------------------------------}

Procedure AddToString (Var S : String; Const A : String);

begin
  If Length(S)=0 then
    S:=A
  else
    S:=S+' '+A;
end;

Function IntToStr(Int : Longint) : String;

begin
  Str(Int,IntToStr);
end;

Function ShiftStateToString(KeyEvent : TKeyEvent; UseLeftRight : Boolean) : String;

Var
  S : Integer;
  T : String;

begin
  S:=GetKeyEventShiftState(KeyEvent);
  T:='';
  If (S and kbShift)<>0 then
    begin
    if UseLeftRight then
      case (S and kbShift) of
        kbShift      : AddToString(T,SLeftRight[1]+' '+SAnd+' '+SLeftRight[2]);
        kbLeftShift  : AddToString(T,SLeftRight[1]);
        kbRightShift : AddToString(T,SLeftRight[2]);
      end;
    AddToString(T,SShift[1]);
    end;
  If (S and kbCtrl)<>0 Then
    AddToString(T,SShift[2]);
  If (S and kbAlt)<>0 Then
    AddToString(T,SShift[3]);
  ShiftStateToString:=T;
end;

Function FunctionKeyName (KeyCode : Word) : String;

begin
  If ((KeyCode-KbdF1)<$1F) Then
    FunctionKeyName:='F'+IntToStr((KeyCode-KbdF1+1))
  else
    begin
    If (KeyCode-kbdHome)<($2F-$1F) then
      FunctionKeyName:=SKeyPad[(KeyCode-kbdHome)]
    else
      FunctionKeyName:=SUnknownFunctionKey + IntToStr(KeyCode);
    end;
end;

Function KeyEventToString(KeyEvent : TKeyEvent) : String;

Var
  T : String;

begin
  T:=ShiftStateToString(KeyEvent,False);
  Case GetKeyEventFlags(KeyEvent) of
    kbASCII   : AddToString(T,GetKeyEventChar(KeyEvent));
    kbUniCode : AddToString(T,SUniCodeChar+IntToStr(GetKeyEventUniCode(Keyevent)));
    kbFnKey   : AddToString(T,FunctionKeyName(GetKeyEventCode(KeyEvent)));
                // Not good, we need a GetKeyEventScanCode function !!
    kbPhys    : AddToString(T,SScanCode+IntToStr(KeyEvent and $ffff));
  end;
  KeyEventToString:=T;
end;


const
  PrevCtrlBreakHandler: TCtrlBreakHandler = nil;

function KbdCtrlBreakHandler (CtrlBreak: boolean): boolean;
begin
(* Earlier registered handlers (user specific) have priority. *)
  if Assigned (PrevCtrlBreakHandler) then
    if PrevCtrlBreakHandler (CtrlBreak) then
      begin
        KbdCtrlBreakHandler := true;
        Exit;
      end;
(* If Ctrl-Break was pressed, either ignore it or allow default processing. *)
  if CtrlBreak then
    KbdCtrlBreakHandler := false
  else (* Ctrl-C pressed or not possible to distinguish *)
    begin
      PutKeyEvent ((kbCtrl shl 16) or 3);
      KbdCtrlBreakHandler := true;
    end;
end;


procedure SetKbdCtrlBreakHandler;
begin
  PrevCtrlBreakHandler := SysSetCtrlBreakHandler (@KbdCtrlBreakHandler);
  if PrevCtrlBreakHandler = TCtrlBreakHandler (pointer (-1)) then
    PrevCtrlBreakHandler := nil;
end;


