{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Nils Sjoholm and Carl Eric Codere
    Copyright (c) 2019 by Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit crt;

interface

{$i crth.inc}

implementation

uses
  exec, amigados, Utility, conunit, intuition, agraphics;

var
  MaxCols, MaxRows: LongInt;

type
  TANSIColor = record
    r,g,b: Byte;
    m: Byte;    // pen on MorphOS
    o: Byte;    // Pen on AmigaOS4
  end;

const
  AnsiColors: array[0..15] of TANSIColor = (
    (r:000; g:000; b:000; m:016; o:000), // 0 = Black
    (r:000; g:000; b:170; m:019; o:004), // 1 = Blue
    (r:000; g:170; b:000; m:034; o:002), // 2 = Green
    (r:000; g:170; b:170; m:037; o:006), // 3 = Cyan
    (r:170; g:000; b:000; m:124; o:001), // 4 = Red
    (r:170; g:000; b:170; m:127; o:005), // 5 = Magenta
    (r:170; g:085; b:000; m:130; o:103), // 6 = Brown
    (r:170; g:170; b:170; m:249; o:107), // 7 = Light Gray
    (r:085; g:085; b:085; m:240; o:107), // 8 = Dark Gray
    (r:000; g:000; b:255; m:021; o:104), // 9 = LightBlue
    (r:000; g:255; b:000; m:046; o:102), // 10 = LightGreen
    (r:000; g:255; b:255; m:087; o:106), // 11 = LightCyan
    (r:255; g:000; b:000; m:196; o:101), // 12 = LightRed
    (r:255; g:000; b:255; m:201; o:105), // 13 = LightMagenta
    (r:255; g:255; b:000; m:226; o:003), // 14 = Yellow
    (r:255; g:255; b:255; m:231; o:007)  // 15 = White
  );



const
  CD_CURRX = 1;
  CD_CURRY = 2;
  CD_MAXX  = 3;
  CD_MAXY  = 4;
  // Special Character for commands to console
  CSI = Chr($9b);

var
  // multiple keys
  LastKeys: string = '';
  Pens: array[0..15] of LongInt;
  FGPen: Byte = Black;
  BGPen: Byte = LightGray;


function IntToStr(i: LongInt): AnsiString;
var
  s: AnsiString;
begin
  Str(i, s);
  IntToStr := s;
end;

function SendActionPacket(Port: PMsgPort; Arg: BPTR): LongInt;
var
  ReplyPort: PMsgPort;
  Packet: PStandardPacket;
  Ret: NativeInt;
begin
  SendActionPacket := 0;
  ReplyPort := CreateMsgPort;
  if not Assigned(ReplyPort) then
    Exit;

  Packet := AllocMem(SizeOf(TStandardPacket));

  if not Assigned(Packet) then
  begin
    DeleteMsgPort(ReplyPort);
    Exit;
  end;

  Packet^.sp_Msg.mn_Node.ln_Name := @(Packet^.sp_Pkt);
  Packet^.sp_Pkt.dp_Link := @(Packet^.sp_Msg);
  Packet^.sp_Pkt.dp_Port := ReplyPort;
  Packet^.sp_Pkt.dp_Type := ACTION_DISK_INFO;
  Packet^.sp_Pkt.dp_Arg1 := NativeInt(Arg);

  PutMsg(Port, PMessage(Packet));
  WaitPort(ReplyPort);
  GetMsg(ReplyPort);

  Ret := Packet^.sp_Pkt.dp_Res1;

  FreeMem(Packet);
  DeleteMsgPort(ReplyPort);

  SendActionPacket := Ret;
end;

function GetConUnit: PConUnit;
var
  Port: PMsgPort;
  Info:  PInfoData;
  Bptr1: BPTR;
begin
  Info := PInfoData(AllocMem(SizeOf(TInfoData)));
  GetConUnit := nil;
  //
  if Assigned(Info) then
  begin
    {$ifdef AmigaOS4}
    Port := PFileHandle(BADDR(DosInput()))^.fh_MsgPort;
    {$else}
    Port := PFileHandle(BADDR(DosInput()))^.fh_Type;
    {$endif}
    //GetConsoleTask;
    Bptr1  := MKBADDR(Info);

    if Assigned(Port) then
    begin
      if SendActionPacket(Port, Bptr1) = 0 then
        Port := nil;
    end;

    if Port = nil then
    begin
      FreeMem(Info);
      Info := nil;
      Exit;
    end;
    GetConUnit := PConUnit((PIoStdReq(Info^.id_InUse))^.io_Unit);
  end;
  FreeMem(Info);
end;

{$if defined(MorphOS)}
//Extract two Integer Values from string ";" separated and space at end
function GetIntValues(Text: AnsiString; var Val1: LongInt; var Val2: LongInt): Boolean;
var
  Start, Ende: LongInt;
  n: Integer;
begin
  GetIntValues := False;
  // First Value
  Start := 1;
  Ende := Pos(';', Text);
  Val(Copy(Text, Start, Ende - Start), Val1, n);
  if n <> 0 then
    Exit;
  // Second Value
  Start := Ende + 1;
  Ende := Pos(' ', Text);
  if Ende <= 0 then
    Ende := Length(Text) + 1;
  Val(Copy(Text, Start, Ende - Start), Val2, n);
  if n <> 0 then
    Exit;
  GetIntValues := True;
end;
{$endif}

// Get the size of Display, this time, MorphOS is broken :(
// does not support ConUnit, is always nil, so we use the slow, error prune way directly via console commands
function GetDisplaySize: TPoint;
{$ifdef MorphOS}
var
  Pt: TPoint;
  fh: BPTR;
  Actual: Integer;
  Width, Height: LongInt;
  report: array[0..25] of Char;
  ToSend: AnsiString;
  Start, Ende: LongInt;
begin
  Pt.X := 2;
  Pt.Y := 2;
  fh := DosOutput();
  if fh <> 0 then
  begin
    //SetMode(fh, 1); // RAW mode
    ToSend := Chr($9b)+'0 q';

    if DosWrite(fh, @ToSend[1], Length(ToSend)) > 0  then
    begin
      actual := DosRead(fh, @report[0], 25);
      if actual >= 0 then
      begin
        report[actual] := #0;
        // Search for position of display message
        Start := 0;
        Ende := 0;
        while Ende < actual do
        begin
          if Report[Ende] = Chr($9b) then
            Start := Ende;
          if Report[Ende] = 'r' then
          begin
            Report[Ende] := #0;
            Break;
          end;
          Inc(Ende);
        end;
        // skip over #$9b'1;1;'
        if GetIntValues(PChar(@report[Start + 5]), Height, Width) then
        begin
          Pt.X := Width + 1;
          Pt.Y := Height + 1;
        end
        else
          sysdebugln('scan failed. ' + PChar(@report[Start + 5]));
      end;
      //SetMode(fh, 0); // Normal mode
    end;
  end;
  GetDisplaySize := Pt;
  MaxCols := Pt.X;
  MaxRows := Pt.Y;
end;
{$else}
var
  Pt: TPoint;
  TheUnit: PConUnit;
begin
  Pt.X := 2;
  Pt.Y := 2;
  TheUnit := GetConUnit;
  if Assigned(TheUnit) then
  begin
    Pt.X := TheUnit^.cu_XMax + 1;
    Pt.Y := TheUnit^.cu_YMax + 1;
  end;
  GetDisplaySize := Pt;
  MaxCols := Pt.X;
  MaxRows := Pt.Y;
end;
{$endif}

// Get the current position of caret, this time, MorphOS is broken :(
// does not support ConUnit, is always nil, so we use the slow, error prune way directly via console commands
function GetCurrentPosition: TPoint;
{$ifdef MorphOS}
var
  Pt: TPoint;
  fh: BPTR;
  Actual: Integer;
  PosX, PosY: LongInt;
  report: array[0..25] of Char;
  ToSend: AnsiString;
  Start, Ende: LongInt;
begin
  Pt.X := 2;
  Pt.Y := 2;
  fh := DosOutput();
  if fh <> 0 then
  begin
    //SetMode(fh, 1); // RAW mode
    ToSend := Chr($9b)+'6n';

    if DosWrite(fh, @ToSend[1], Length(ToSend)) > 0  then
    begin
      actual := DosRead(fh, @report[0], 25);
      if actual >= 0 then
      begin
        report[actual] := #0;
        // search for the position message
        Start := 0;
        Ende := 0;
        while Ende < actual do
        begin
          if Report[Ende] = Chr($9b) then
            Start := Ende;
          if Report[Ende] = 'R' then
          begin
            Report[Ende] := ' ';
            Break;
          end;
          Inc(Ende);
        end;
        // skip over #$9b
        if GetIntValues(PChar(@report[Start + 1]), PosY, PosX) then
        begin
          Pt.X := PosX;
          Pt.Y := PosY;
        end
        else
          sysdebugln('scan failed. ' +  PChar(@report[Start + 1]));
      end;
      //SetMode(fh, 0); // Normal mode
    end;
  end;
  GetCurrentPosition := Pt;
end;
{$else}
var
  Pt: TPoint;
  TheUnit: PConUnit;
begin
  Pt.X := 1;
  Pt.Y := 1;
  TheUnit := GetConUnit;
  if Assigned(TheUnit) then
  begin
    Pt.X := TheUnit^.cu_Xcp + 1;
    Pt.Y := TheUnit^.cu_Ycp + 1;
  end;
  GetCurrentPosition := Pt;
end;
{$endif}

procedure InternalWrite(s: AnsiString);
begin
  DosWrite(DosOutput(), @s[1], Length(s));
end;

function RealX: Byte;
begin
  RealX := Byte(GetCurrentPosition.X);
end;

function WhereX: TCrtCoord;
begin
  WhereX := Byte(RealX) - WindMinX;
end;

function RealY: Byte;
begin
  RealY := Byte(GetCurrentPosition.Y);
end;

function WhereY: TCrtCoord;
begin
  WhereY := Byte(RealY) - WindMinY;
end;

function ScreenCols: Integer;
begin
  Screencols := MaxCols;
end;

function ScreenRows: Integer;
begin
  ScreenRows := MaxRows;
end;

procedure RealGotoXY(x, y: Integer);
begin
  InternalWrite(CSI + IntToStr(y) + ';' + IntToStr(x) + 'H');
end;

procedure GotoXY(x, y: TCrtCoord);
begin
  if y + WindMinY - 2 >= WindMaxY then
    y := WindMaxY - WindMinY + 1;
  if x + WindMinX - 2 >= WindMaxX then
    x := WindMaxX - WindMinX + 1;
  InternalWrite(CSI + IntToStr(y + WindMinY) +  ';' + IntToStr(x + WindMinX) + 'H');
end;

procedure CursorOff;
begin
  InternalWrite(CSI + '0 p');
end;

procedure CursorOn;
begin
  InternalWrite(CSI + ' p');
end;

procedure ClrScr;
var
  i: Integer;
begin
  for i :=  1 to (WindMaxY - WindMinY) + 1 do
  begin
    GotoXY(1, i);
    InternalWrite(StringOfChar(' ', WindMaxX - WindMinX + 1));
  end;
  GotoXY(1, 1);
end;

function WaitForKey: string;
var
  OutP: BPTR; // Output file handle
  Res: Char; // Char to get from console
  Key: string; // result
begin
  Key := '';
  OutP := DosOutput();
  //SetMode(OutP, 1); // change to Raw Mode
  // Special for AROS
  // AROS always sends a #184, #185 or #0, ignore them
  repeat
    Res := #0;
    DosRead(OutP, @Res, 1);
    if not (Ord(Res) in [184, 185, 0]) then
      Break;
    Delay(1);
  until False;
  // get the key
  Key := Res;
  // Check if Special OP
  if Res = CSI then
  begin
    repeat
      Res := #0;
      DosRead(OutP, @Res, 1);
      if Ord(Res) in [184, 185, 0] then // just to make sure on AROS that it ends when nothing left
        Break;
      if Ord(Res) = 126 then // end marker
        Break;
      Key := Key + Res; // add to final string
      // stop on cursor, they have no end marker...
      case Ord(Res) of
        64..69,83,84: Break;
      end;
    until False;
  end;
  // set result
  WaitForKey := Key;
  // set back mode to CON:
  //SetMode(OutP, 0);
end;

type
  TKeyMap = record
    con: string;
    c1: Char;
    c2: Char;
  end;
const
  KeyMapping: array[0..37] of TKeyMap =
    ((con: #127;    c1: #0; c2:#83;), // Del

     (con: #155'0'; c1: #0; c2:#59;), // F1
     (con: #155'1'; c1: #0; c2:#60;), // F2
     (con: #155'2'; c1: #0; c2:#61;), // F3
     (con: #155'3'; c1: #0; c2:#62;), // F4
     (con: #155'4'; c1: #0; c2:#63;), // F5
     (con: #155'5'; c1: #0; c2:#64;), // F6
     (con: #155'6'; c1: #0; c2:#65;), // F7
     (con: #155'7'; c1: #0; c2:#66;), // F8
     (con: #155'8'; c1: #0; c2:#67;), // F9
     (con: #155'9'; c1: #0; c2:#68;), // F10
     (con: #155'20'; c1: #0; c2:#133;), // F11
     (con: #155'21'; c1: #0; c2:#134;), // F12

     (con: #155'10'; c1: #0; c2:#84;), // Shift F1
     (con: #155'11'; c1: #0; c2:#85;), // Shift F2
     (con: #155'12'; c1: #0; c2:#86;), // Shift F3
     (con: #155'13'; c1: #0; c2:#87;), // Shift F4
     (con: #155'14'; c1: #0; c2:#88;), // Shift F5
     (con: #155'15'; c1: #0; c2:#89;), // Shift F6
     (con: #155'16'; c1: #0; c2:#90;), // Shift F7
     (con: #155'17'; c1: #0; c2:#91;), // Shift F8
     (con: #155'18'; c1: #0; c2:#92;), // Shift F9
     (con: #155'19'; c1: #0; c2:#93;), // Shift F10
     (con: #155'30'; c1: #0; c2:#135;), // Shift F11
     (con: #155'31'; c1: #0; c2:#136;), // Shift F12

     (con: #155'40'; c1: #0; c2:#82;), // Ins
     (con: #155'44'; c1: #0; c2:#71;), // Home
     (con: #155'45'; c1: #0; c2:#70;), // End
     (con: #155'41'; c1: #0; c2:#73;), // Page Up
     (con: #155'42'; c1: #0; c2:#81;), // Page Down

     (con: #155'A'; c1: #0; c2:#72;), // Cursor Up
     (con: #155'B'; c1: #0; c2:#80;), // Cursor Down
     (con: #155'C'; c1: #0; c2:#77;), // Cursor Right
     (con: #155'D'; c1: #0; c2:#75;), // Cursor Left
     (con: #155'T'; c1: #0; c2:#65;), // Shift Cursor Up
     (con: #155'S'; c1: #0; c2:#66;), // Shift Cursor Down
     (con: #155' A'; c1: #0; c2:#67;), // Shift Cursor Right
     (con: #155' @'; c1: #0; c2:#68;)  // Shift Cursor Left
     );

function ReadKey: Char;
var
  Res: string;
  i: Integer;
begin
  // we got a key to sent
  if Length(LastKeys) > 0 then
  begin
    ReadKey := LastKeys[1];
    Delete(LastKeys, 1, 1);
    Exit;
  end;
  Res := WaitForKey;
  // Search for Map Key
  for i := 0 to High(KeyMapping) do
  begin
    if KeyMapping[i].Con = Res then
    begin
      ReadKey := KeyMapping[i].c1;
      if KeyMapping[i].c2 <> #0 then
        LastKeys := KeyMapping[i].c2;
      Exit;
    end;
  end;
  ReadKey := Res[1];
end;


// Wait for Key, does not work for AROS currently
// because WaitForChar ALWAYS returns even no key is pressed, but this
// is clearly an AROS bug
function KeyPressed : Boolean;
var
  OutP: BPTR;
begin
  if Length(LastKeys) > 0 then
  begin
    KeyPressed := True;
    Exit;
  end;
  OutP := DosOutput();
  //SetMode(OutP, 1);
  // Wait one millisecond for the key (-1 = timeout)
  {$if defined(AROS)}
  KeyPressed := WaitForChar(OutP, 1) <> 0;
  {$else}
  KeyPressed := WaitForChar(OutP, 1);
  {$endif}
  //SetMode(OutP, 0);
end;

procedure TextColor(color : byte);
{$ifndef MorphOS}
var
  TheUnit: PConUnit;
{$endif}
begin
  Color := Color and $F;
  FGPen := Color;
  {$ifdef MorphOS}
  InternalWrite(CSI + '38;5;'+ IntToStr(AnsiColors[Color].m) + 'm');
  {$else}
  {$ifdef AmigaOS4}
  if AnsiColors[Color].o > 100 then
    InternalWrite(CSI + '1;3'+ IntToStr(AnsiColors[Color].o - 100) + 'm')
  else
    InternalWrite(CSI + '22;3'+ IntToStr(AnsiColors[Color].o) + 'm')
  {$else}
  if Pens[Color] < 0 then
    Pens[Color] := ObtainBestPen(IntuitionBase^.ActiveScreen^.ViewPort.ColorMap, AnsiColors[color].r shl 24, AnsiColors[color].g shl 24, AnsiColors[color].b shl 24, [TAG_END]);
  TheUnit := GetConUnit;
  if Assigned(TheUnit) then
  begin
    if Pens[Color] >= 0 then
    begin
      TheUnit^.cu_Mask := -1; // set the mask to show all colors!
      TheUnit^.cu_FgPen := Pens[Color]
    end
    else
    begin
      TheUnit^.cu_FgPen := 2;
      SysDebugLn('Cannot obtain Text Pen ' + IntToStr(color) + ' use default');
    end;
  end
  else
    SysDebugLn('ConUnit not found');
  {$endif} // AmigaOS4
  {$endif} // MorphOS
end;

procedure TextBackground(color : byte);
{$ifndef MorphOS}
var
  TheUnit: PConUnit;
{$endif}
begin
  Color := Color and $F;
  BGPen := Color;
  {$ifdef MorphOS}
  InternalWrite(CSI + '48;5;'+ IntToStr(AnsiColors[Color].m) + 'm');
  {$else}
  {$ifdef AmigaOS4}
  if AnsiColors[Color].o > 100 then
    InternalWrite(CSI + '1;4'+ IntToStr(AnsiColors[Color].o - 100) + 'm')
  else
    InternalWrite(CSI + '22;4'+ IntToStr(AnsiColors[Color].o) + 'm')
  {$else}
  if Pens[Color] < 0 then
    Pens[Color] := ObtainBestPen(IntuitionBase^.ActiveScreen^.ViewPort.ColorMap, AnsiColors[color].r shl 24, AnsiColors[color].g shl 24, AnsiColors[color].b shl 24, [TAG_END]);
  TheUnit := GetConUnit;
  if Assigned(TheUnit) then
  begin
    if Pens[Color] >= 0 then
    begin
      TheUnit^.cu_Mask := -1; // set the mask to show all colors!
      TheUnit^.cu_BgPen := Pens[Color]
    end
    else
    begin
      TheUnit^.cu_FgPen := 0;
      SysDebugLn('Cannot obtain Background Pen ' + IntToStr(color) + ' use default');
    end;
  end
  else
    SysDebugLn('ConUnit not found');
  {$endif} // AmigaOS4
  {$endif} // MorphOS
end;

function GetTextBackground: Byte;
begin
  GetTextBackground := BGPen;
end;

function GetTextColor: Byte;
begin
  GetTextColor := FGPen;
end;

procedure Window(X1,Y1,X2,Y2: Byte);
begin
  if x2 > ScreenCols then
    x2 := ScreenCols;
  if y2 > ScreenRows then
    y2 := ScreenRows;
  WindMinX := x1 - 1;
  WindMinY := y1 - 1;
  WindMaxX := x2 - 1;
  WindMaxY := y2 - 1;
  GotoXY(1, 1);
end;


procedure DelLine;
begin
  InternalWrite(CSI + 'X');
end;

procedure ClrEol;
begin
  InternalWrite(CSI + 'K');
end;

procedure InsLine;
begin
  InternalWrite(CSI + '1 L');
end;

procedure CursorBig;
begin
end;

procedure LowVideo;
begin
end;

procedure HighVideo;
begin
end;

procedure NoSound;
begin
end;

procedure Sound(hz: Word);
begin
end;

procedure NormVideo;
begin
end;

procedure Delay(ms: Word);
var
  Dummy: Longint;
begin
  dummy := Trunc((ms / 1000.0) * 50.0);
  DOSDelay(dummy);
end;

procedure TextMode(Mode: word);
begin
  LastMode := Mode;
  Mode := Mode and $ff;
  MaxCols := ScreenCols;
  MaxRows := ScreenRows;
  WindMinX := 0;
  WindMinY := 0;
  WindMaxX := MaxCols - 1;
  WindMaxY := MaxRows - 1;
end;

procedure WriteChar(c: Char; var Curr: TPoint; var s: AnsiString);
//var
//  i: Integer;
var
  isEmpty: boolean;
begin
  IsEmpty := Length(s) = 0;
  // ignore #13, we only use #10
  case c of
    #13: Exit;
    #7: begin
       DisplayBeep(nil);
       Exit;
    end;
    #8: begin
       if Length(s) > 0 then
       begin
         Delete(s, Length(s), 1);
         Dec(Curr.X);
         Exit;
       end;
    end;
    else
    begin
      // all other Chars
      s := s + c;
      //sysdebugln(' Char: ' + c + ' ' + IntToStr(Curr.X) + ' ' + IntToStr(Curr.Y) + ' - ' + IntToStr(WindMinY) + ' ' + IntToStr(WindMaxY));
      case c of
        #10: begin
          if WindMinX > 0 then
            s := s + CSI + IntToStr(WindMinX) + 'C';
          Curr.X := WindMinX + 1;
          if Curr.Y <= WindMaxY then
            Inc(Curr.Y)
          else
          begin
            // only start at top again for smaller windows
            if WindMaxY < MaxRows - 1 then
              Curr.Y := WindMinY + 1;
            s := s + CSI + IntToStr(Curr.Y) + ';' + IntToStr(WindMinX + 1) + 'H';
            if not isEmpty then
              s := s + StringOfChar(' ', WindMaxX - WindMinX + 1);
          end;
          if isEmpty then
            s := s + StringOfChar(' ', WindMaxX - WindMinX);
          s := s + CSI + IntToStr(Curr.Y) + ';' + IntToStr(Curr.X) + 'H';
        end;
        #8: begin
          Curr.X := RealX;
        end;
        else
        begin
          Inc(Curr.X);
        end;
      end;
    end;
  end;
  // wrap line
  if Curr.X > (WindMaxX + 1) then
  begin
    if Curr.Y <= WindMaxY - 1 then
      Inc(Curr.Y);
    s := s + CSI + IntToStr(Curr.Y) + ';' + IntToStr(WindMinX + 1) + 'H' + CSI + 'K';
    //sysdebugln('clear 2');
    Curr.X := WindMinX + 1;
  end;
end;

procedure CrtWrite(Var F: TextRec);
var
  i: Smallint;
  Curr: TPoint;
  s: AnsiString;
begin
  Curr := GetCurrentPosition;
  s := '';
  for i := 0 to f.BufPos - 1 do
    WriteChar(F.Buffer[i], Curr, s);
  InternalWrite(s);
  F.BufPos := 0;
end;

Procedure CrtRead(Var F: TextRec);
var
  ch : Char;

  procedure BackSpace;
  begin
    if (f.bufpos>0) and (f.bufpos=f.bufend) then
     begin
       InternalWrite(#8);
       InternalWrite(' ');
       InternalWrite(#8);
       dec(f.bufpos);
       dec(f.bufend);
     end;
  end;


Begin
  //Curr := GetCurrentPosition;
  f.bufpos:=0;
  f.bufend:=0;
  repeat
    if f.bufpos > f.bufend then
     f.bufend := f.bufpos;
    //SetScreenCursor(CurrX,CurrY);
    ch := readkey;
    case ch of
      #0: begin
        readkey;
        Exit;
      end;
      ^S,
      #8: BackSpace;
      ^Y,
      #27: begin
        while f.bufpos < f.bufend do
        begin
          InternalWrite(f.bufptr^[f.bufpos]);
          Inc(f.bufpos);
        end;
        while f.bufend>0 do
          BackSpace;
      end;
      #13: begin
        InternalWrite(#13);
        InternalWrite(#10);
        f.bufptr^[f.bufend] := #13;
        f.bufptr^[f.bufend + 1] := #10;
        Inc(f.bufend, 2);
          break;
      end;
      #26:
        if CheckEOF then
        begin
          f.bufptr^[f.bufend] := #26;
          Inc(f.bufend);
          break;
        end;
      else
      begin
        if f.bufpos < f.bufsize - 2 then
        begin
          f.buffer[f.bufpos] := ch;
          Inc(f.bufpos);
          InternalWrite(ch);
        end;
      end;
    end;
  until False;
  f.bufpos := 0;
  //SetScreenCursor(CurrX,CurrY);
End;

procedure CrtReturn(var F: TextRec);
begin
end;

procedure CrtClose(var F: TextRec);
begin
  F.Mode:=fmClosed;
end;


procedure CrtOpen(var F: TextRec);
begin
  if F.Mode = fmOutput then
  begin
    TextRec(F).InOutFunc := @CrtWrite;
    TextRec(F).FlushFunc := @CrtWrite;
  end
  else
  begin
    F.Mode:=fmInput;
    TextRec(F).InOutFunc:=@CrtRead;
    TextRec(F).FlushFunc:=@CrtReturn;
  end;
  TextRec(F).CloseFunc := @CrtClose;
end;

procedure AssignCrt(var F: Text);
begin
  Assign(F,'');
  TextRec(F).OpenFunc:=@CrtOpen;
end;

procedure InitCRT;
var
  i: Integer;
begin
  SetMode(DosOutput(), 1);
  //
  AssignCrt(Output);
  Rewrite(Output);
  TextRec(Output).Handle := StdOutputHandle;
  //
  AssignCrt(Input);
  Reset(Input);
  TextRec(Input).Handle := StdInputHandle;
  for i := 0 to High(Pens) do
    Pens[i] := -1;
  // get screensize (sets MaxCols/MaxRows)
  GetDisplaySize;
  // set output window
  WindMaxX := MaxCols - 1;
  WindMaxY := MaxRows - 1;
end;

procedure FreeCRT;
var
  i: Integer;
begin
  SetMode(DosOutput(), 0);
  for i := 0 to High(Pens) do
  begin
    if Pens[i] >= 0 then
      ReleasePen(IntuitionBase^.ActiveScreen^.ViewPort.ColorMap, Pens[i]);
    Pens[i] := -1;
  end;
  // reset colors and delete to end of screen (get rid of old drawings behind the last caret position)
  InternalWrite(CSI + '0m' + CSI + 'J');
  CursorOn;
end;


initialization
  InitCRT;
finalization
  FreeCRT;
end.
