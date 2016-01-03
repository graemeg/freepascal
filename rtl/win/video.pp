{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Video unit for Win32

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Video;
interface

{$i videoh.inc}
const
  useunicodefunctions : boolean = false;

implementation

uses
  windows,dos;

{$i video.inc}

  type
    tunicodecharmappingflag = (umf_noinfo,umf_leadbyte,umf_undefined,
      umf_unused);

    punicodecharmapping = ^tunicodecharmapping;
    tunicodecharmapping = record
       unicode : word;
       flag : tunicodecharmappingflag;
       reserved : byte;
    end;

  const
     mapcp850 : array[0..255] of tunicodecharmapping = (
       (unicode : 0; flag : umf_noinfo; reserved : 0),
       (unicode : 1; flag : umf_noinfo; reserved : 0),
       (unicode : 2; flag : umf_noinfo; reserved : 0),
       (unicode : 3; flag : umf_noinfo; reserved : 0),
       (unicode : 4; flag : umf_noinfo; reserved : 0),
       (unicode : 5; flag : umf_noinfo; reserved : 0),
       (unicode : 6; flag : umf_noinfo; reserved : 0),
       (unicode : 7; flag : umf_noinfo; reserved : 0),
       (unicode : 8; flag : umf_noinfo; reserved : 0),
       (unicode : 9; flag : umf_noinfo; reserved : 0),
       (unicode : 10; flag : umf_noinfo; reserved : 0),
       (unicode : 11; flag : umf_noinfo; reserved : 0),
       (unicode : 12; flag : umf_noinfo; reserved : 0),
       (unicode : 13; flag : umf_noinfo; reserved : 0),
       (unicode : 14; flag : umf_noinfo; reserved : 0),
       (unicode : 15; flag : umf_noinfo; reserved : 0),
       (unicode : 16; flag : umf_noinfo; reserved : 0),
       (unicode : 17; flag : umf_noinfo; reserved : 0),
       (unicode : 18; flag : umf_noinfo; reserved : 0),
       (unicode : 19; flag : umf_noinfo; reserved : 0),
       (unicode : 20; flag : umf_noinfo; reserved : 0),
       (unicode : 21; flag : umf_noinfo; reserved : 0),
       (unicode : 22; flag : umf_noinfo; reserved : 0),
       (unicode : 23; flag : umf_noinfo; reserved : 0),
       (unicode : 24; flag : umf_noinfo; reserved : 0),
       (unicode : 25; flag : umf_noinfo; reserved : 0),
       (unicode : 26; flag : umf_noinfo; reserved : 0),
       (unicode : 27; flag : umf_noinfo; reserved : 0),
       (unicode : 28; flag : umf_noinfo; reserved : 0),
       (unicode : 29; flag : umf_noinfo; reserved : 0),
       (unicode : 30; flag : umf_noinfo; reserved : 0),
       (unicode : 31; flag : umf_noinfo; reserved : 0),
       (unicode : 32; flag : umf_noinfo; reserved : 0),
       (unicode : 33; flag : umf_noinfo; reserved : 0),
       (unicode : 34; flag : umf_noinfo; reserved : 0),
       (unicode : 35; flag : umf_noinfo; reserved : 0),
       (unicode : 36; flag : umf_noinfo; reserved : 0),
       (unicode : 37; flag : umf_noinfo; reserved : 0),
       (unicode : 38; flag : umf_noinfo; reserved : 0),
       (unicode : 39; flag : umf_noinfo; reserved : 0),
       (unicode : 40; flag : umf_noinfo; reserved : 0),
       (unicode : 41; flag : umf_noinfo; reserved : 0),
       (unicode : 42; flag : umf_noinfo; reserved : 0),
       (unicode : 43; flag : umf_noinfo; reserved : 0),
       (unicode : 44; flag : umf_noinfo; reserved : 0),
       (unicode : 45; flag : umf_noinfo; reserved : 0),
       (unicode : 46; flag : umf_noinfo; reserved : 0),
       (unicode : 47; flag : umf_noinfo; reserved : 0),
       (unicode : 48; flag : umf_noinfo; reserved : 0),
       (unicode : 49; flag : umf_noinfo; reserved : 0),
       (unicode : 50; flag : umf_noinfo; reserved : 0),
       (unicode : 51; flag : umf_noinfo; reserved : 0),
       (unicode : 52; flag : umf_noinfo; reserved : 0),
       (unicode : 53; flag : umf_noinfo; reserved : 0),
       (unicode : 54; flag : umf_noinfo; reserved : 0),
       (unicode : 55; flag : umf_noinfo; reserved : 0),
       (unicode : 56; flag : umf_noinfo; reserved : 0),
       (unicode : 57; flag : umf_noinfo; reserved : 0),
       (unicode : 58; flag : umf_noinfo; reserved : 0),
       (unicode : 59; flag : umf_noinfo; reserved : 0),
       (unicode : 60; flag : umf_noinfo; reserved : 0),
       (unicode : 61; flag : umf_noinfo; reserved : 0),
       (unicode : 62; flag : umf_noinfo; reserved : 0),
       (unicode : 63; flag : umf_noinfo; reserved : 0),
       (unicode : 64; flag : umf_noinfo; reserved : 0),
       (unicode : 65; flag : umf_noinfo; reserved : 0),
       (unicode : 66; flag : umf_noinfo; reserved : 0),
       (unicode : 67; flag : umf_noinfo; reserved : 0),
       (unicode : 68; flag : umf_noinfo; reserved : 0),
       (unicode : 69; flag : umf_noinfo; reserved : 0),
       (unicode : 70; flag : umf_noinfo; reserved : 0),
       (unicode : 71; flag : umf_noinfo; reserved : 0),
       (unicode : 72; flag : umf_noinfo; reserved : 0),
       (unicode : 73; flag : umf_noinfo; reserved : 0),
       (unicode : 74; flag : umf_noinfo; reserved : 0),
       (unicode : 75; flag : umf_noinfo; reserved : 0),
       (unicode : 76; flag : umf_noinfo; reserved : 0),
       (unicode : 77; flag : umf_noinfo; reserved : 0),
       (unicode : 78; flag : umf_noinfo; reserved : 0),
       (unicode : 79; flag : umf_noinfo; reserved : 0),
       (unicode : 80; flag : umf_noinfo; reserved : 0),
       (unicode : 81; flag : umf_noinfo; reserved : 0),
       (unicode : 82; flag : umf_noinfo; reserved : 0),
       (unicode : 83; flag : umf_noinfo; reserved : 0),
       (unicode : 84; flag : umf_noinfo; reserved : 0),
       (unicode : 85; flag : umf_noinfo; reserved : 0),
       (unicode : 86; flag : umf_noinfo; reserved : 0),
       (unicode : 87; flag : umf_noinfo; reserved : 0),
       (unicode : 88; flag : umf_noinfo; reserved : 0),
       (unicode : 89; flag : umf_noinfo; reserved : 0),
       (unicode : 90; flag : umf_noinfo; reserved : 0),
       (unicode : 91; flag : umf_noinfo; reserved : 0),
       (unicode : 92; flag : umf_noinfo; reserved : 0),
       (unicode : 93; flag : umf_noinfo; reserved : 0),
       (unicode : 94; flag : umf_noinfo; reserved : 0),
       (unicode : 95; flag : umf_noinfo; reserved : 0),
       (unicode : 96; flag : umf_noinfo; reserved : 0),
       (unicode : 97; flag : umf_noinfo; reserved : 0),
       (unicode : 98; flag : umf_noinfo; reserved : 0),
       (unicode : 99; flag : umf_noinfo; reserved : 0),
       (unicode : 100; flag : umf_noinfo; reserved : 0),
       (unicode : 101; flag : umf_noinfo; reserved : 0),
       (unicode : 102; flag : umf_noinfo; reserved : 0),
       (unicode : 103; flag : umf_noinfo; reserved : 0),
       (unicode : 104; flag : umf_noinfo; reserved : 0),
       (unicode : 105; flag : umf_noinfo; reserved : 0),
       (unicode : 106; flag : umf_noinfo; reserved : 0),
       (unicode : 107; flag : umf_noinfo; reserved : 0),
       (unicode : 108; flag : umf_noinfo; reserved : 0),
       (unicode : 109; flag : umf_noinfo; reserved : 0),
       (unicode : 110; flag : umf_noinfo; reserved : 0),
       (unicode : 111; flag : umf_noinfo; reserved : 0),
       (unicode : 112; flag : umf_noinfo; reserved : 0),
       (unicode : 113; flag : umf_noinfo; reserved : 0),
       (unicode : 114; flag : umf_noinfo; reserved : 0),
       (unicode : 115; flag : umf_noinfo; reserved : 0),
       (unicode : 116; flag : umf_noinfo; reserved : 0),
       (unicode : 117; flag : umf_noinfo; reserved : 0),
       (unicode : 118; flag : umf_noinfo; reserved : 0),
       (unicode : 119; flag : umf_noinfo; reserved : 0),
       (unicode : 120; flag : umf_noinfo; reserved : 0),
       (unicode : 121; flag : umf_noinfo; reserved : 0),
       (unicode : 122; flag : umf_noinfo; reserved : 0),
       (unicode : 123; flag : umf_noinfo; reserved : 0),
       (unicode : 124; flag : umf_noinfo; reserved : 0),
       (unicode : 125; flag : umf_noinfo; reserved : 0),
       (unicode : 126; flag : umf_noinfo; reserved : 0),
       (unicode : 127; flag : umf_noinfo; reserved : 0),
       (unicode : 199; flag : umf_noinfo; reserved : 0),
       (unicode : 252; flag : umf_noinfo; reserved : 0),
       (unicode : 233; flag : umf_noinfo; reserved : 0),
       (unicode : 226; flag : umf_noinfo; reserved : 0),
       (unicode : 228; flag : umf_noinfo; reserved : 0),
       (unicode : 224; flag : umf_noinfo; reserved : 0),
       (unicode : 229; flag : umf_noinfo; reserved : 0),
       (unicode : 231; flag : umf_noinfo; reserved : 0),
       (unicode : 234; flag : umf_noinfo; reserved : 0),
       (unicode : 235; flag : umf_noinfo; reserved : 0),
       (unicode : 232; flag : umf_noinfo; reserved : 0),
       (unicode : 239; flag : umf_noinfo; reserved : 0),
       (unicode : 238; flag : umf_noinfo; reserved : 0),
       (unicode : 236; flag : umf_noinfo; reserved : 0),
       (unicode : 196; flag : umf_noinfo; reserved : 0),
       (unicode : 197; flag : umf_noinfo; reserved : 0),
       (unicode : 201; flag : umf_noinfo; reserved : 0),
       (unicode : 230; flag : umf_noinfo; reserved : 0),
       (unicode : 198; flag : umf_noinfo; reserved : 0),
       (unicode : 244; flag : umf_noinfo; reserved : 0),
       (unicode : 246; flag : umf_noinfo; reserved : 0),
       (unicode : 242; flag : umf_noinfo; reserved : 0),
       (unicode : 251; flag : umf_noinfo; reserved : 0),
       (unicode : 249; flag : umf_noinfo; reserved : 0),
       (unicode : 255; flag : umf_noinfo; reserved : 0),
       (unicode : 214; flag : umf_noinfo; reserved : 0),
       (unicode : 220; flag : umf_noinfo; reserved : 0),
       (unicode : 248; flag : umf_noinfo; reserved : 0),
       (unicode : 163; flag : umf_noinfo; reserved : 0),
       (unicode : 216; flag : umf_noinfo; reserved : 0),
       (unicode : 215; flag : umf_noinfo; reserved : 0),
       (unicode : 402; flag : umf_noinfo; reserved : 0),
       (unicode : 225; flag : umf_noinfo; reserved : 0),
       (unicode : 237; flag : umf_noinfo; reserved : 0),
       (unicode : 243; flag : umf_noinfo; reserved : 0),
       (unicode : 250; flag : umf_noinfo; reserved : 0),
       (unicode : 241; flag : umf_noinfo; reserved : 0),
       (unicode : 209; flag : umf_noinfo; reserved : 0),
       (unicode : 170; flag : umf_noinfo; reserved : 0),
       (unicode : 186; flag : umf_noinfo; reserved : 0),
       (unicode : 191; flag : umf_noinfo; reserved : 0),
       (unicode : 174; flag : umf_noinfo; reserved : 0),
       (unicode : 172; flag : umf_noinfo; reserved : 0),
       (unicode : 189; flag : umf_noinfo; reserved : 0),
       (unicode : 188; flag : umf_noinfo; reserved : 0),
       (unicode : 161; flag : umf_noinfo; reserved : 0),
       (unicode : 171; flag : umf_noinfo; reserved : 0),
       (unicode : 187; flag : umf_noinfo; reserved : 0),
       (unicode : 9617; flag : umf_noinfo; reserved : 0),
       (unicode : 9618; flag : umf_noinfo; reserved : 0),
       (unicode : 9619; flag : umf_noinfo; reserved : 0),
       (unicode : 9474; flag : umf_noinfo; reserved : 0),
       (unicode : 9508; flag : umf_noinfo; reserved : 0),
       (unicode : 193; flag : umf_noinfo; reserved : 0),
       (unicode : 194; flag : umf_noinfo; reserved : 0),
       (unicode : 192; flag : umf_noinfo; reserved : 0),
       (unicode : 169; flag : umf_noinfo; reserved : 0),
       (unicode : 9571; flag : umf_noinfo; reserved : 0),
       (unicode : 9553; flag : umf_noinfo; reserved : 0),
       (unicode : 9559; flag : umf_noinfo; reserved : 0),
       (unicode : 9565; flag : umf_noinfo; reserved : 0),
       (unicode : 162; flag : umf_noinfo; reserved : 0),
       (unicode : 165; flag : umf_noinfo; reserved : 0),
       (unicode : 9488; flag : umf_noinfo; reserved : 0),
       (unicode : 9492; flag : umf_noinfo; reserved : 0),
       (unicode : 9524; flag : umf_noinfo; reserved : 0),
       (unicode : 9516; flag : umf_noinfo; reserved : 0),
       (unicode : 9500; flag : umf_noinfo; reserved : 0),
       (unicode : 9472; flag : umf_noinfo; reserved : 0),
       (unicode : 9532; flag : umf_noinfo; reserved : 0),
       (unicode : 227; flag : umf_noinfo; reserved : 0),
       (unicode : 195; flag : umf_noinfo; reserved : 0),
       (unicode : 9562; flag : umf_noinfo; reserved : 0),
       (unicode : 9556; flag : umf_noinfo; reserved : 0),
       (unicode : 9577; flag : umf_noinfo; reserved : 0),
       (unicode : 9574; flag : umf_noinfo; reserved : 0),
       (unicode : 9568; flag : umf_noinfo; reserved : 0),
       (unicode : 9552; flag : umf_noinfo; reserved : 0),
       (unicode : 9580; flag : umf_noinfo; reserved : 0),
       (unicode : 164; flag : umf_noinfo; reserved : 0),
       (unicode : 240; flag : umf_noinfo; reserved : 0),
       (unicode : 208; flag : umf_noinfo; reserved : 0),
       (unicode : 202; flag : umf_noinfo; reserved : 0),
       (unicode : 203; flag : umf_noinfo; reserved : 0),
       (unicode : 200; flag : umf_noinfo; reserved : 0),
       (unicode : 305; flag : umf_noinfo; reserved : 0),
       (unicode : 205; flag : umf_noinfo; reserved : 0),
       (unicode : 206; flag : umf_noinfo; reserved : 0),
       (unicode : 207; flag : umf_noinfo; reserved : 0),
       (unicode : 9496; flag : umf_noinfo; reserved : 0),
       (unicode : 9484; flag : umf_noinfo; reserved : 0),
       (unicode : 9608; flag : umf_noinfo; reserved : 0),
       (unicode : 9604; flag : umf_noinfo; reserved : 0),
       (unicode : 166; flag : umf_noinfo; reserved : 0),
       (unicode : 204; flag : umf_noinfo; reserved : 0),
       (unicode : 9600; flag : umf_noinfo; reserved : 0),
       (unicode : 211; flag : umf_noinfo; reserved : 0),
       (unicode : 223; flag : umf_noinfo; reserved : 0),
       (unicode : 212; flag : umf_noinfo; reserved : 0),
       (unicode : 210; flag : umf_noinfo; reserved : 0),
       (unicode : 245; flag : umf_noinfo; reserved : 0),
       (unicode : 213; flag : umf_noinfo; reserved : 0),
       (unicode : 181; flag : umf_noinfo; reserved : 0),
       (unicode : 254; flag : umf_noinfo; reserved : 0),
       (unicode : 222; flag : umf_noinfo; reserved : 0),
       (unicode : 218; flag : umf_noinfo; reserved : 0),
       (unicode : 219; flag : umf_noinfo; reserved : 0),
       (unicode : 217; flag : umf_noinfo; reserved : 0),
       (unicode : 253; flag : umf_noinfo; reserved : 0),
       (unicode : 221; flag : umf_noinfo; reserved : 0),
       (unicode : 175; flag : umf_noinfo; reserved : 0),
       (unicode : 180; flag : umf_noinfo; reserved : 0),
       (unicode : 173; flag : umf_noinfo; reserved : 0),
       (unicode : 177; flag : umf_noinfo; reserved : 0),
       (unicode : 8215; flag : umf_noinfo; reserved : 0),
       (unicode : 190; flag : umf_noinfo; reserved : 0),
       (unicode : 182; flag : umf_noinfo; reserved : 0),
       (unicode : 167; flag : umf_noinfo; reserved : 0),
       (unicode : 247; flag : umf_noinfo; reserved : 0),
       (unicode : 184; flag : umf_noinfo; reserved : 0),
       (unicode : 176; flag : umf_noinfo; reserved : 0),
       (unicode : 168; flag : umf_noinfo; reserved : 0),
       (unicode : 183; flag : umf_noinfo; reserved : 0),
       (unicode : 185; flag : umf_noinfo; reserved : 0),
       (unicode : 179; flag : umf_noinfo; reserved : 0),
       (unicode : 178; flag : umf_noinfo; reserved : 0),
       (unicode : 9632; flag : umf_noinfo; reserved : 0),
       (unicode : 160; flag : umf_noinfo; reserved : 0)
     );


const
    LastCursorType: word = crUnderline;
    OrigScreen: PVideoBuf = nil;
    OrigScreenSize: cardinal = 0;


var ConsoleInfo : TConsoleScreenBufferInfo;
    ConsoleCursorInfo : TConsoleCursorInfo;

    OrigCP: cardinal;
    OrigConsoleCursorInfo : TConsoleCursorInfo;
    OrigConsoleInfo : TConsoleScreenBufferInfo;

procedure SysInitVideo;

begin
  ScreenColor:=true;
  GetConsoleScreenBufferInfo(TextRec(Output).Handle, OrigConsoleInfo);
  GetConsoleCursorInfo(TextRec(Output).Handle, OrigConsoleCursorInfo);
  OrigCP := GetConsoleCP;
  ConsoleInfo:=OrigConsoleInfo;
  ConsoleCursorInfo:=OrigConsoleCursorInfo;
  {
    About the ConsoleCursorInfo record: There are 3 possible
    structures in it that can be regarded as the 'screen':
    - dwsize   : contains the cols & row in current screen buffer.
    - srwindow : Coordinates (relative to buffer) of upper left
                 & lower right corners of visible console.
    - dmMaximumWindowSize : Maximal size of Screen buffer.
    The first implementation of video used srWindow. After some
    bug-reports, this was switched to dwMaximumWindowSize.
  }
  with ConsoleInfo.dwMaximumWindowSize do
    begin
    ScreenWidth:=X;
    ScreenHeight:=Y;
    end;
  { TDrawBuffer only has FVMaxWidth elements
    larger values lead to crashes }
  if ScreenWidth> FVMaxWidth then
    ScreenWidth:=FVMaxWidth;
  CursorX:=ConsoleInfo.dwCursorPosition.x;
  CursorY:=ConsoleInfo.dwCursorPosition.y;
  if not ConsoleCursorInfo.bvisible then
    CursorLines:=0
  else
    CursorLines:=ConsoleCursorInfo.dwSize;
end;


procedure SysDoneVideo;
begin
  SetConsoleScreenBufferSize (TextRec (Output).Handle, OrigConsoleInfo.dwSize);
  SetConsoleWindowInfo (cardinal (TextRec (Output).Handle), true, OrigConsoleInfo.srWindow);
  SetConsoleCursorInfo(TextRec(Output).Handle, OrigConsoleCursorInfo);
  SetConsoleCP(OrigCP);
end;


function SysGetCapabilities: Word;
begin
  SysGetCapabilities:=cpColor or cpChangeCursor;
end;


procedure SysSetCursorPos(NewCursorX, NewCursorY: Word);
var
  pos : COORD;
begin
   pos.x:=NewCursorX;
   pos.y:=NewCursorY;
   SetConsoleCursorPosition(TextRec(Output).Handle,pos);
   CursorX:=pos.x;
   CursorY:=pos.y;
end;


function SysGetCursorType: Word;
begin
   GetConsoleCursorInfo(TextRec(Output).Handle,ConsoleCursorInfo);
   if not ConsoleCursorInfo.bvisible then
     SysGetCursorType:=crHidden
   else
     case ConsoleCursorInfo.dwSize of
        1..30:
          SysGetCursorType:=crUnderline;
        31..70:
          SysGetCursorType:=crHalfBlock;
        71..100:
          SysGetCursorType:=crBlock;
     end;
end;


procedure SysSetCursorType(NewType: Word);
begin
   GetConsoleCursorInfo(TextRec(Output).Handle,ConsoleCursorInfo);
   if newType=crHidden then
     ConsoleCursorInfo.bvisible:=false
   else
     begin
        ConsoleCursorInfo.bvisible:=true;
        case NewType of
           crUnderline:
             ConsoleCursorInfo.dwSize:=10;

           crHalfBlock:
             ConsoleCursorInfo.dwSize:=50;

           crBlock:
             ConsoleCursorInfo.dwSize:=99;
        end
     end;
   SetConsoleCursorInfo(TextRec(Output).Handle,ConsoleCursorInfo);
end;

function SysVideoModeSelector (const VideoMode: TVideoMode): boolean;

var MI: Console_Screen_Buffer_Info;
    C: Coord;
    SR: Small_Rect;

begin
  if not (GetConsoleScreenBufferInfo (TextRec (Output).Handle, MI)) then
    SysVideoModeSelector := false
  else
    begin
      with MI do
        begin
          C.X := VideoMode.Col;
          C.Y := VideoMode.Row;
        end;
      with SR do
        begin
          Top := 0;
          Left := 0;
          { First, we need to make sure we reach the minimum window size
            to always fit in the new buffer after changing buffer size. }
          Right := MI.srWindow.Right - MI.srWindow.Left;
          if VideoMode.Col <= Right then
            Right := Pred (VideoMode.Col);
          Bottom := MI.srWindow.Bottom - MI.srWindow.Top;
          if VideoMode.Row <= Bottom then
            Bottom := Pred (VideoMode.Row);
        end;
      if SetConsoleWindowInfo (cardinal (TextRec (Output).Handle), true, SR) then
        if SetConsoleScreenBufferSize (TextRec (Output).Handle, C) then
          begin
            with SR do
              begin
                { Now, we can resize the window to the final size. }
                Right := Pred (VideoMode.Col);
                Bottom := Pred (VideoMode.Row);
              end;
            if SetConsoleWindowInfo (cardinal (TextRec (Output).Handle), true, SR) then
              begin
                SysVideoModeSelector := true;
                SetCursorType (LastCursorType);
                ClearScreen;
              end
            else
              begin
                SysVideoModeSelector := false;
                SetConsoleScreenBufferSize (TextRec (Output).Handle, MI.dwSize);
                SetConsoleWindowInfo (cardinal (TextRec (Output).Handle), true, MI.srWindow);
                SetCursorType (LastCursorType);
              end
          end
        else
          begin
            SysVideoModeSelector := false;
            SetConsoleWindowInfo (cardinal (TextRec (Output).Handle), true, MI.srWindow);
            SetCursorType (LastCursorType);
          end
      else
        SysVideoModeSelector := false;
    end;
end;

Const
  SysVideoModeCount = 6;
  SysVMD : Array[0..SysVideoModeCount-1] of TVideoMode = (
   (Col: 40; Row: 25; Color: True),
   (Col: 80; Row: 25; Color: True),
   (Col: 80; Row: 30; Color: True),
   (Col: 80; Row: 43; Color: True),
   (Col: 80; Row: 50; Color: True),
   (Col: 80; Row: 25; Color: True) // Reserved for TargetEntry
  );


Function SysSetVideoMode (Const Mode : TVideoMode) : Boolean;

Var
  I : Integer;

begin
  I:=SysVideoModeCount-1;
  SysSetVideoMode:=False;
  While (I>=0) and Not SysSetVideoMode do
    If (Mode.col=SysVMD[i].col) and
       (Mode.Row=SysVMD[i].Row) and
       (Mode.Color=SysVMD[i].Color) then
      SysSetVideoMode:=True
    else
      Dec(I);
  If SysSetVideoMode then
    begin
    if SysVideoModeSelector(Mode) then
      begin
      ScreenWidth:=SysVMD[I].Col;
      ScreenHeight:=SysVMD[I].Row;
      ScreenColor:=SysVMD[I].Color;
      end else SysSetVideoMode := false;
    end;
end;

Function SysGetVideoModeData (Index : Word; Var Data : TVideoMode) : boolean;

begin
  SysGetVideoModeData:=(Index<=high(SysVMD));
  If SysGetVideoModeData then
    Data:=SysVMD[Index];
end;

Function SysGetVideoModeCount : Word;

begin
  SysGetVideoModeCount:=SysVideoModeCount;
end;

procedure SysClearScreen;
begin
  UpdateScreen(true);
end;

procedure SysUpdateScreen(Force: Boolean);

type WordRec = record
                  One, Two: Byte;
               end; { wordrec }

var
   BufSize,
   BufCoord    : COORD;
   WriteRegion : SMALL_RECT;
   LineBuf     : Array[0..(1024*32) - 1] of TCharInfo;
   BufCounter  : Longint;
   LineCounter,
   ColCounter  : Longint;
   smallforce  : boolean;
   x1,y1,x2,y2 : longint;
   p1,p2,p3    : PCardinal;
   j           : integer;
begin
  if force then
   smallforce:=true
  else
   begin
    {$ifdef cpui386}
     asm
        pushl   %esi
        pushl   %edi
        movl    VideoBuf,%esi
        movl    OldVideoBuf,%edi
        movl    VideoBufSize,%ecx
        shrl    $2,%ecx
        repe
        cmpsl
        setne   smallforce
        popl    %edi
        popl    %esi
     end;
   {$else}
    {$ifdef cpux86_64}
     asm
        pushq   %rsi
        pushq   %rdi
        xorq    %rcx,%rcx  
        movq    VideoBuf,%rsi
        movq    OldVideoBuf,%rdi
        movl    VideoBufSize,%ecx
        shrq    $2,%rcx
        repe
        cmpsl
        setne   smallforce
        popq    %rdi
        popq    %rsi
     end;
    {$else}
      {$INFO No optimized version for this CPU, reverting to a pascal version}
       j:=Videobufsize shr 2;
       smallforce:=false;
       p1:=pcardinal(VideoBuf);
       p2:=pcardinal(OldVideoBuf);
       p3:=@pcardinal(videobuf)[j];
       while (p1<p3) and (p1^=p2^) do
         begin
           inc(p1); inc(p2);
         end; 
       smallforce:=p1<>p3;  
    {$ENDIF}
   {$endif}
   end;
  if SmallForce then
   begin
      BufSize.X := ScreenWidth;
      BufSize.Y := ScreenHeight;

      BufCoord.X := 0;
      BufCoord.Y := 0;
      with WriteRegion do
        begin
           Top :=0;
           Left :=0;
           Bottom := ScreenHeight-1;
           Right := ScreenWidth-1;
        end;
      BufCounter := 0;
      x1:=ScreenWidth+1;
      x2:=-1;
      y1:=ScreenHeight+1;
      y2:=-1;
      for LineCounter := 1 to ScreenHeight do
        begin
           for ColCounter := 1 to ScreenWidth do
             begin
               if (WordRec(VideoBuf^[BufCounter]).One<>WordRec(OldVideoBuf^[BufCounter]).One) or
                 (WordRec(VideoBuf^[BufCounter]).Two<>WordRec(OldVideoBuf^[BufCounter]).Two) then
                 begin
                    if ColCounter<x1 then
                      x1:=ColCounter;
                    if ColCounter>x2 then
                      x2:=ColCounter;
                    if LineCounter<y1 then
                      y1:=LineCounter;
                    if LineCounter>y2 then
                      y2:=LineCounter;
                 end;
               if useunicodefunctions then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 LineBuf[BufCounter].UniCodeChar := Widechar(mapcp850[WordRec(VideoBuf^[BufCounter]).One].unicode)
               else
                 LineBuf[BufCounter].UniCodeChar := Widechar(WordRec(VideoBuf^[BufCounter]).One);
               { If (WordRec(VideoBuf^[BufCounter]).Two and $80)<>0 then
                 LineBuf^[BufCounter].Attributes := $100+WordRec(VideoBuf^[BufCounter]).Two
               else }
               LineBuf[BufCounter].Attributes := WordRec(VideoBuf^[BufCounter]).Two;
=======
                 LineBuf^[BufCounter].UniCodeChar := Widechar(mapcp850[WordRec(VideoBuf^[BufCounter]).One].unicode)
=======
                 LineBuf[BufCounter].UniCodeChar := Widechar(mapcp850[WordRec(VideoBuf^[BufCounter]).One].unicode)
>>>>>>> origin/fixes_2.4
               else
                 LineBuf[BufCounter].UniCodeChar := Widechar(WordRec(VideoBuf^[BufCounter]).One);
               { If (WordRec(VideoBuf^[BufCounter]).Two and $80)<>0 then
                 LineBuf^[BufCounter].Attributes := $100+WordRec(VideoBuf^[BufCounter]).Two
               else }
<<<<<<< HEAD
               LineBuf^[BufCounter].Attributes := WordRec(VideoBuf^[BufCounter]).Two;
>>>>>>> graemeg/fixes_2_2
=======
                 LineBuf^[BufCounter].UniCodeChar := Widechar(mapcp850[WordRec(VideoBuf^[BufCounter]).One].unicode)
               else
                 LineBuf^[BufCounter].UniCodeChar := Widechar(WordRec(VideoBuf^[BufCounter]).One);
               { If (WordRec(VideoBuf^[BufCounter]).Two and $80)<>0 then
                 LineBuf^[BufCounter].Attributes := $100+WordRec(VideoBuf^[BufCounter]).Two
               else }
               LineBuf^[BufCounter].Attributes := WordRec(VideoBuf^[BufCounter]).Two;
>>>>>>> origin/fixes_2_2
=======
               LineBuf[BufCounter].Attributes := WordRec(VideoBuf^[BufCounter]).Two;
>>>>>>> origin/fixes_2.4

               Inc(BufCounter);
             end; { for }
        end; { for }
      BufSize.X := ScreenWidth;
      BufSize.Y := ScreenHeight;

      with WriteRegion do
        begin
           if force then
             begin
               Top := 0;
               Left :=0;
               Bottom := ScreenHeight-1;
               Right := ScreenWidth-1;
               BufCoord.X := 0;
               BufCoord.Y := 0;
             end
           else
             begin
               Top := y1-1;
               Left :=x1-1;
               Bottom := y2-1;
               Right := x2-1;
               BufCoord.X := x1-1;
               BufCoord.Y := y1-1;
             end;
        end;
      {
      writeln('X1: ',x1);
      writeln('Y1: ',y1);
      writeln('X2: ',x2);
      writeln('Y2: ',y2);
      }
      if useunicodefunctions then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        WriteConsoleOutputW(TextRec(Output).Handle, @LineBuf, BufSize, BufCoord, WriteRegion)
      else
        WriteConsoleOutput(TextRec(Output).Handle, @LineBuf, BufSize, BufCoord, WriteRegion);
=======
=======
>>>>>>> origin/fixes_2_2
        WriteConsoleOutputW(TextRec(Output).Handle, LineBuf, BufSize, BufCoord, WriteRegion)
      else
        WriteConsoleOutput(TextRec(Output).Handle, LineBuf, BufSize, BufCoord, WriteRegion);
      Dispose(LineBuf);
>>>>>>> graemeg/fixes_2_2
=======
        WriteConsoleOutputW(TextRec(Output).Handle, @LineBuf, BufSize, BufCoord, WriteRegion)
      else
        WriteConsoleOutput(TextRec(Output).Handle, @LineBuf, BufSize, BufCoord, WriteRegion);
>>>>>>> origin/fixes_2.4

      move(VideoBuf^,OldVideoBuf^,VideoBufSize);
   end;
end;

Const
  SysVideoDriver : TVideoDriver = (
    InitDriver : @SysInitVideo;
    DoneDriver : @SysDoneVideo;
    UpdateScreen : @SysUpdateScreen;
    ClearScreen : @SysClearScreen;
    SetVideoMode : @SysSetVideoMode;
    GetVideoModeCount : @SysGetVideoModeCount;
    GetVideoModeData : @SysGetVideoModeData;
    SetCursorPos : @SysSetCursorPos;
    GetCursorType : @SysGetCursorType;
    SetCursorType : @SysSetCursorType;
    GetCapabilities : @SysGetCapabilities

  );

procedure TargetEntry;
var
  C: Coord;
  SR: Small_Rect;
  VioMode: TConsoleScreenBufferInfo;
begin
  GetConsoleScreenBufferInfo (TextRec (Output).Handle, VioMode);
  { Register the curent video mode in reserved slot in System Modes}
  with VioMode do
    begin
      {Assume we have at least 16 colours available in "colour" modes}
      SysVMD[SysVideoModeCount-1].Col:=dwMaximumWindowSize.X;
      SysVMD[SysVideoModeCount-1].Row:=dwMaximumWindowSize.Y;
      SysVMD[SysVideoModeCount-1].Color:=true;
      OrigScreenSize := max(dwMaximumWindowSize.X,dwSize.X) * max(dwMaximumWindowSize.Y,dwSize.Y) * SizeOf (Char_Info);
    end;
  GetMem (OrigScreen, OrigScreenSize);
  with C do
    begin
      X := 0;
      Y := 0;
    end;
  with SR do
    begin
      Top := 0;
      Left := 0;
      Right := Pred (VioMode.dwSize.X);
      Bottom := Pred (VioMode.dwSize.Y);
    end;
  if not (ReadConsoleOutput (TextRec (Output).Handle, OrigScreen, VioMode.dwSize, C, SR)) then
    begin
      FreeMem (OrigScreen, OrigScreenSize);
      OrigScreen := nil;
      OrigScreenSize := 0;
    end;
end;


initialization
  SetVideoDriver(SysVideoDriver);
  TargetEntry;
end.
