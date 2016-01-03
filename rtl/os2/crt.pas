{****************************************************************************


                            Standard CRT unit.
                    Free Pascal runtime library for OS/2.
                    Copyright (c) 1997 Daniel Mantione.

      This file may be reproduced and modified under the same conditions
                      as all other Free Pascal source code.

****************************************************************************}

unit crt;

interface

{$INLINE ON}

{$i crth.inc}

procedure Window32 (X1, Y1, X2, Y2: dword);
procedure GotoXY32 (X, Y: dword);
function WhereX32: dword;
function WhereY32: dword;


var
 ScreenHeight, ScreenWidth: dword;
(* API *)


implementation

{uses keyboard, video;}


{$i textrec.inc}

const
 VioHandle: word = 0;


type
 TKbdKeyInfo = record
  CharCode, ScanCode: char;
  fbStatus, bNlsShift: byte;
  fsState: word;
  Time: longint;
 end;

 VioModeInfo = record
  cb: word;                         { length of the entire data
                                               structure }
  fbType,                          { bit mask of mode being set}
  Color: byte;                     { number of colors (power of 2) }
  Col,                             { number of text columns }
  Row,                             { number of text rows }
  HRes,                            { horizontal resolution }
  VRes: word;                      { vertical resolution }
  fmt_ID,                          { attribute format }
  Attrib: byte;                    { number of attributes }
  Buf_Addr,                        { physical address of
                                               videobuffer, e.g. $0b800}
  Buf_Length,                      { length of a videopage (bytes)}
  Full_Length,                     { total video-memory on video-
                                               card (bytes)}
  Partial_Length: longint;          { ????? info wanted !}
  Ext_Data_Addr: pointer;           { ????? info wanted !}
 end;

 TVioCursorInfo=record
  case boolean of
   false: (
        yStart: word;    {Cursor start (top) scan line (0-based)}
        cEnd: word;      {Cursor end (bottom) scan line}
        cx: word;        {Cursor width (0=default width)}
        Attr: word);     {Cursor colour attribute (-1=hidden)}
   true:(
        yStartInt: integer; {integer variants can be used to specify negative}
        cEndInt: integer; {negative values (interpreted as percentage by OS/2)}
        cxInt: integer;
        AttrInt: integer);
 end;
 PVioCursorInfo = ^TVioCursorInfo;


function KbdCharIn (var AKeyRec: TKbdKeyInfo; Wait, KbdHandle: longint):
                                                                   word; cdecl;
                   external 'EMXWRAP' index 204;
function KbdPeek (var AKeyRec: TKbdKeyInfo; KbdHandle: longint): word; cdecl;
                 external 'EMXWRAP' index 222;

function DosSleep (Time: cardinal): word; cdecl;
                  external 'DOSCALLS' index 229;
function VioScrollUp (Top, Left, Bottom, Right, Lines: longint;
                      var ScrEl: word; VioHandle: word): word; cdecl;
                      external 'EMXWRAP' index 107;
{$WARNING ScrEl as word not DBCS safe!}
function VioScrollDn (Top, Left, Bottom, Right, Lines: longint;
                      var ScrEl: word; VioHandle: word): word; cdecl;
                      external 'EMXWRAP' index 147;
function VioScrollRight (Top, Left, Bottom, Right, Columns: word;
                                var ScrEl: word; VioHandle: word): word; cdecl;
external 'EMXWRAP' index 112;
{external 'VIOCALLS' index 12;}
function VioGetCurPos (var Row, Column: word; VioHandle: word): word; cdecl;
                       external 'EMXWRAP' index 109;
function VioSetCurPos (Row, Column, VioHandle: word): word; cdecl;
                       external 'EMXWRAP' index 115;
function VioWrtCharStrAtt (S: PChar; Len, Row, Col: longint; var Attr: byte;
                           VioHandle: word): word; cdecl;
                           external 'EMXWRAP' index 148;
function VioGetMode (var AModeInfo: VioModeInfo; VioHandle: word): word; cdecl;
                     external 'EMXWRAP' index 121;
function VioSetMode (var AModeInfo: VioModeInfo; VioHandle: word): word; cdecl;
                     external 'EMXWRAP' index 122;
function VioSetCurType (var CurData: TVioCursorInfo; VioHandle: word): word;
                                                                         cdecl;
external 'EMXWRAP' index 132;
{external 'VIOCALLS' index 32;}
function VioGetCurType (var CurData: TVioCursorInfo; VioHandle: word): word;
                                                                         cdecl;
external 'EMXWRAP' index 127;
{external 'VIOCALLS' index 27;}
function VioCreatePS (var VPS: word; Depth, Width, Format, Attrs: integer;
                                                  Reserved: word): word; cdecl;
external 'EMXWRAP' index 156;
{external 'VIOCALLS' index 56;}
function DosBeep (Freq, MS: cardinal): cardinal; cdecl;
external 'DOSCALLS' index 286;



procedure GetScreenCursor (var X, Y: dword);inline;
(* Return current cursor postion - 0-based. *)
var
 X0, Y0: word;
begin
 X := 0;
 Y := 0;
 if VioGetCurPos (Y0, X0, VioHandle) = 0 then
  begin
   X := X0;
   Y := Y0;
  end;
end;


procedure SetScreenCursor (X, Y: dword); inline;
(* Set current cursor postion - 0-based. *)
begin
 VioSetCurPos (Y, X, VioHandle);
end;


procedure RemoveLines (Row: dword; Cnt: dword); inline;
(* Remove Cnt lines from screen starting with (0-based) Row. *)
var
 ScrEl: word;
begin
 ScrEl := $20 or (TextAttr shl 8);
 VioScrollUp (Row + WindMinY, WindMinX, WindMaxY, WindMaxX, Cnt, ScrEl,
                                                                    VioHandle);
end;


procedure ClearCells (X, Y, Cnt: dword); inline;
(* Clear Cnt cells in line Y (0-based) starting with position X (0-based). *)
var
 ScrEl: word;
begin
 ScrEl := $20 or (TextAttr shl 8);
 VioScrollRight (Y, X, Y, X + Pred (Cnt), Cnt, ScrEl, VioHandle);
end;


procedure InsLine;
(* Inserts a line at cursor position. *)
var
 ScrEl: word;
begin
 ScrEl := $20 or (TextAttr shl 8);
 VioScrollDn (Pred (WhereY32) + WindMinY, WindMinX, WindMaxY, WindMaxX, 1,
                                                             ScrEl, VioHandle);
end;


procedure SetScreenMode (Mode: word);
var
 NewMode: VioModeInfo;
begin
 NewMode.cb := 8;
 VioGetMode (NewMode, VioHandle);
 NewMode.fbType := 1;  {Non graphics colour mode.}
 NewMode.Color := 4;   {We want 16 colours, 2^4=16 - requests for BW ignored.}
 case Mode and $FF of
  BW40, CO40: NewMode.Col := 40;
  BW80, CO80: NewMode.Col := 80;
 else
  begin
(* Keep current amount of columns! *)
  end;
 end;
 case Mode and $100 of
  0: NewMode.Row := 25;
  $100: NewMode.Row := 50
 else
  begin
(* Keep current amount of rows! *)
  end;
 end;
 VioSetMode (NewMode, VioHandle);
 ScreenWidth := NewMode.Col;
 ScreenHeight := NewMode.Row;
end;


procedure Delay (Ms: word);
{Waits ms milliseconds.}
begin
 DosSleep (Ms)
end;


procedure WriteNormal (C: char; X, Y: dword); inline;
(* Write C to console at X, Y (0-based). *)
begin
 VioWrtCharStrAtt (@C, 1, Y, X, TextAttr, VioHandle);
end;


procedure WriteBell; inline;
(* Write character #7 - beep. *)
begin
 DosBeep (800, 250);
end;



{****************************************************************************
                             Extra Crt Functions
****************************************************************************}


procedure CursorOn;
var
 I: TVioCursorInfo;
begin
 VioGetCurType (I, VioHandle);
 with I do
  begin
   yStartInt := -90;
   cEndInt := -100;
   Attr := 15;
  end;
 VioSetCurType (I, VioHandle);
end;


procedure CursorOff;
var
 I: TVioCursorInfo;
begin
 VioGetCurType (I, VioHandle);
 I.AttrInt := -1;
 VioSetCurType (I, VioHandle);
end;


procedure CursorBig;
var
 I: TVioCursorInfo;
begin
 VioGetCurType (I, VioHandle);
 with I do
  begin
   yStart := 0;
   cEndInt := -100;
   Attr := 15;
  end;
 VioSetCurType (I, VioHandle);
end;


(* Include common, platform independent part. *)
{$I crt.inc}


function KeyPressed: boolean;
{Checks if a key is pressed.}
var
 AKeyRec: TKbdKeyinfo;
begin
 if SpecialKey or (ScanCode <> 0) then
  KeyPressed := true
 else
  KeyPressed := (KbdPeek (AKeyRec, 0) = 0)
                                         and ((AKeyRec.fbStatus and $40) <> 0);
end;


function ReadKey: char;
{Reads the next character from the keyboard.}
var
 AKeyRec: TKbdKeyInfo;
 C, S: char;
begin
 if SpecialKey then
  begin
   SpecialKey := false;
   ReadKey := char (ScanCode);
   ScanCode := 0;
  end
 else
  if ScanCode <> 0 then
   begin
    ReadKey := char (ScanCode);
    ScanCode := 0;
   end
  else
   begin
    while ((KbdCharIn (AKeyRec, 1, 0) <> 0)
                    or (AKeyRec.fbStatus and $41 <> $40)) and (ScanCode = 0) do
     DosSleep (5);
    if ScanCode = 0 then
     begin
      C := AKeyRec.CharCode;
      S := AKeyRec.ScanCode;
      if (C = #224) and (S <> #0) then
       C := #0;
      if C = #0 then
       begin
        SpecialKey := true;
        ScanCode := byte (S);
       end;
      ReadKey := C;
     end
    else
     begin
      ReadKey := char (ScanCode);
      ScanCode := 0;
     end;
   end;
end;


{Initialization.}

var
 CurMode: VioModeInfo;
begin
 if not (IsConsole) then
  VioCreatePS (VioHandle, 25, 80, 1, 1, 0);
{  InitVideo;}
 CurMode.cb := SizeOf (CurMode);
 VioGetMode (CurMode, VioHandle);
 ScreenWidth := CurMode.Col;
 ScreenHeight := CurMode.Row;
 LastMode := 0;
 case ScreenWidth of
  40: LastMode := CO40;
  80: LastMode := CO80
 else
  LastMode := 255
 end;
 case ScreenHeight of
  50: LastMode := LastMode + $100
 else
  LastMode := LastMode + $FF00;
 end;
 CrtInit;
end.
