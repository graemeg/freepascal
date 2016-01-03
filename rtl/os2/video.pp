{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Video unit for OS/2

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Video;

interface

{$i videoh.inc}

implementation

uses
  DosCalls, VioCalls, Mouse;

{$i video.inc}


const
    LastCursorType: word = crUnderline;
    EmptyCell: cardinal = $0720;
    OrigScreen: PVideoBuf = nil;
    OrigScreenSize: cardinal = 0;

var OrigCurType: TVioCursorInfo;
    OrigVioMode: TVioModeInfo;
    OrigHighBit: TVioIntensity;
    OrigCurRow: word;
    OrigCurCol: word;
    CellHeight: byte;

procedure CheckCellHeight;

var OldCD, CD: TVioCursorInfo;

begin
    VioGetCurType (OldCD, 0);
    Move (OldCD, CD, SizeOf (CD));
    with CD do
        begin
            Attr := 0;
            yStart := word (-90);
            cEnd := word (-100);
        end;
    VioSetCurType (CD, 0);
    VioGetCurType (CD, 0);
    CellHeight := CD.cEnd;
    VioSetCurType (OldCD, 0);
end;




procedure SetHighBitBlink (Blink: boolean);

var VI: TVioIntensity;

begin
    with VI do
        begin
            cb := 6;
            rType := 2;
            fs := byte (not (Blink));
        end;
    VioSetState (VI, 0);
end;


Var
  SysVideoBuf : PVideoBuf;

procedure SysInitVideo;

var
  MI: TVioModeInfo;
  NewBuf: PVideoBuf;

begin
  MI.cb := SizeOf (MI);
  VioGetMode (MI, 0);
  with MI do
    begin
    ScreenWidth := Col;
    ScreenHeight := Row;
    ScreenColor := Color >= Colors_16;
    end;
  VioGetCurPos (CursorY, CursorX, 0);
  SetCursorType (LastCursorType);
{ Get the address of the videobuffer.}
  if VioGetBuf (NewBuf, PWord (@VideoBufSize)^, 0) = 0 then
    begin
    SysVideoBuf := SelToFlat (PtrUInt (NewBuf));
    SetHighBitBlink (true);
    end
  else
    ErrorHandler (errVioInit, nil);
end;


procedure SysSetCursorPos (NewCursorX, NewCursorY: word);

begin
  if VioSetCurPos (NewCursorY, NewCursorX, 0) = 0 then
    begin
    CursorX := NewCursorX;
    CursorY := NewCursorY;
    end
{  else
   Do not set an error code; people should fix invalid NewCursorX
    or NewCursorY values when designing, there is no need for detecting
    these errors at runtime.
    RunError (225);}
end;


function SysGetCursorType: word;

var CD: TVioCursorInfo;

begin
    VioGetCurType (CD, 0);    {Never fails, because handle is default handle.}
    with CD do
      begin
      CursorLines := Succ (cEnd) - yStart;
      if Attr = word (-1) then
         SysGetCursorType := crHidden
      else
       {Because the cursor's start and end lines are returned, we'll have
        to guess heuristically what cursor type we have.}
        if CursorLines = 0 then
           {Probably this does not occur, but you'll never know.}
           SysGetCursorType := crHidden
         else if CursorLines <= Succ (CellHeight div 4) then
           SysGetCursorType := crUnderline
         else if CursorLines <= Succ (CellHeight div 2) then
           SysGetCursorType := crHalfBlock
         else
           SysGetCursorType := crBlock;
   end;
end;


procedure SysSetCursorType (NewType: word);

var CD: TVioCursorInfo;

begin
  VioGetCurType (CD, 0);
  with CD do
    begin
    case NewType of
      crHidden: Attr := word (-1);
      crUnderline:
        begin
        Attr := 0;
        yStart := word (-90);
        cEnd := word (-100);
        end;
      crHalfBlock:
        begin
        Attr := 0;
        yStart := word (-50);
        cEnd := word (-100);
        end;
      crBlock:
        begin
        Attr := 0;
        yStart := 0;
        cEnd := word (-100);
        end;
    end;
    VioSetCurType (CD, 0);
    VioGetCurType (CD, 0);
    CursorLines := Succ (cEnd) - yStart;
    end;
end;


procedure SysClearScreen;

begin
  VioScrollDn (0, 0, word (-1), word (-1), word (-1), PWord (@EmptyCell)^, 0);
  FillWord (SysVideoBuf^, VideoBufSize shr 1, PWord (@EmptyCell)^);
end;


procedure SysDoneVideo;

var PScr: pointer;
    ScrSize: cardinal;

begin
  LastCursorType := GetCursorType;
  SysClearScreen;
  {Restore original settings}
  VioSetMode (OrigVioMode, 0);
  CheckCellHeight;
{Set CursorX and CursorY}
  SetCursorPos (0, 0);
  VioSetState (OrigHighBit, 0);
  VioSetCurType (OrigCurType, 0);
  VioSetCurPos (OrigCurRow, OrigCurCol, 0);
  if (OrigScreenSize <> 0) and (OrigScreen <> nil) then
    begin
    ScrSize := 0;
    if (VioGetBuf (PScr, PWord (@ScrSize)^, 0) = 0) and
       (ScrSize = OrigScreenSize) then
      begin
      PScr := SelToFlat (PtrUInt (PScr));
      Move (OrigScreen^, PScr^, OrigScreenSize);
      VioShowBuf (0, ScrSize, 0);
      end;
    end;
end;


function SysGetCapabilities: word;

begin
  SysGetCapabilities := $3F;
end;


function SysVideoModeSelector (const VideoMode: TVideoMode): boolean;

var
  OldMI, MI: TVioModeInfo;
  NewBuf: PVideoBuf;

begin
  OldMI.cb := SizeOf (OldMI);
  if VioGetMode (OldMI, 0) <> 0 then
    SysVideoModeSelector := false
  else
    begin
    with MI do
      begin
      cb := 8;
      fbType := 1;
      if VideoMode.Color then
        Color := Colors_16
      else
        Color := Colors_2;
      Col := VideoMode.Col;
      Row := VideoMode.Row;
      end;
    if VioSetMode (MI, 0) = 0 then
      if VioGetBuf (NewBuf, PWord (@VideoBufSize)^, 0) = 0 then
        begin
        SysVideoBuf := SelToFlat (PtrUInt (NewBuf));
        SysVideoModeSelector := true;
        SetHighBitBlink (true);
        CheckCellHeight;
        SetCursorType (LastCursorType);
        SysClearScreen;
        end
      else
        begin
        SysVideoModeSelector := false;
        VioSetMode (OldMI, 0);
        if (VioGetBuf (NewBuf, PWord (@VideoBufSize)^, 0) = 0) then
          SysVideoBuf := SelToFlat (PtrUInt (NewBuf));
        SetHighBitBlink (true);
        CheckCellHeight;
        SetCursorType (LastCursorType);
        SysClearScreen;
        end
    else
      begin
      SysVideoModeSelector := false;
      if VioGetBuf (NewBuf, PWord (@VideoBufSize)^, 0) = 0 then
        SysVideoBuf := SelToFlat (PtrUInt (NewBuf));
      SetHighBitBlink (true);
      SetCursorType (LastCursorType);
      end;
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

{ .MVC. were commented:
   BW modes are rejected on my (colour) configuration. I can't imagine
   OS/2 running on MCGA anyway... ;-)
   (Col: 40; Row: 25;Color: False),
   (Col: 80; Row: 25;Color: False),
   The following modes wouldn't work on plain VGA; is it useful to check
   for their availability on the program startup?
   (Col: 132;Row: 25;Color: True),
   (Col: 132;Row: 30;Color: True),
   (Col: 132;Row: 43;Color: True),
   (Col: 132;Row: 50;Color: True),
}

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
      begin;
      ScreenWidth:=SysVMD[I].Col;
      ScreenHeight:=SysVMD[I].Row;
      ScreenColor:=SysVMD[I].Color;
      end else SysSetVideoMode := false;
    end;
end;

Function SysGetVideoModeData (Index : Word; Var Data : TVideoMode) : boolean;

begin
  SysGetVideoModeData:=(Index<=SysVideoModeCount);
  If SysGetVideoModeData then
    Data:=SysVMD[Index];
end;

Function SysGetVideoModeCount : Word;

begin
  SysGetVideoModeCount:=SysVideoModeCount;
end;

{$ASMMODE INTEL}

procedure SysUpdateScreen (Force: boolean);

var SOfs, CLen: cardinal;
    Mouse_Visible: boolean;

begin
  if not (Force) then
    asm
      push ebx
      push esi
      push edi
      cld
      mov esi, VideoBuf
      mov edi, OldVideoBuf
      mov eax, VideoBufSize
      mov ecx, eax
      shr ecx, 1
      shr ecx, 1
      repe
      cmpsd
      je @no_update
      inc ecx
      mov edx, eax
      mov ebx, ecx
      shl ebx, 1
      shl ebx, 1
      sub edx, ebx
      mov SOfs, edx
      mov Force, 1
      std
      mov edi, eax
      mov esi, VideoBuf
      add eax, esi
      sub eax, 4
      mov esi, eax
      mov eax, OldVideoBuf
      add eax, edi
      sub eax, 4
      mov edi, eax
      repe
      cmpsd
      inc ecx
      shl ecx, 1
      shl ecx, 1
      mov CLen, ecx
      cld
@no_update:
      pop edi
      pop esi
      pop ebx
    end ['eax', 'ecx', 'edx']
  else
    begin
    SOfs := 0;
    CLen := VideoBufSize;
    end;
  // .MVC. Move video buffer to system video buffer.
{$HINT Change so that only relevant parts calculated above are moved}
  Move(VideoBuf^,SysVideoBuf^,VideoBufSize);
  if Force then
    begin
      Mouse_Visible := MouseIsVisible; {MouseIsVisible is from Mouse unit}
      if Mouse_Visible then
        HideMouse;
      VioShowBuf (SOfs, CLen, 0);
      Move (VideoBuf^ [SOfs div SizeOf (TVideoCell)],
          OldVideoBuf^ [SOfs div SizeOf (TVideoCell)], CLen);
      if Mouse_Visible then
        ShowMouse;
    end;
end;

Const
  SysVideoDriver : TVideoDriver = (
    InitDriver        : @SysInitVideo;
    DoneDriver        : @SysDoneVideo;
    UpdateScreen      : @SysUpdateScreen;
    ClearScreen       : @SysClearScreen;
    SetVideoMode      : @SysSetVideoMode;
    GetVideoModeCount : @SysGetVideoModeCount;
    GetVideoModeData  : @SysGetVideoModedata;
    SetCursorPos      : @SysSetCursorPos;
    GetCursorType     : @SysGetCursorType;
    SetCursorType     : @SysSetCursorType;
    GetCapabilities   : @SysGetCapabilities
  );

procedure TargetEntry;

var
  PScr: pointer;

begin
{Remember original video mode, cursor type and high bit behaviour setting}
  OrigVioMode.cb := SizeOf (OrigVioMode);
  VioGetMode (OrigVioMode, 0);
  with OrigVioMode do
    begin
    ScreenWidth := Col;
    ScreenHeight := Row;
    ScreenColor := Color >= Colors_16;
    end;
  VioGetCurType (OrigCurType, 0);
  VioGetCurPos (OrigCurRow, OrigCurCol, 0);
  with OrigHighBit do
    begin
    cb := 6;
    rType := 2;
    end;
  VioGetState (OrigHighBit, 0);
  { Register the curent video mode in reserved slot in System Modes}
  with OrigVioMode do
    begin
    {Assume we have at least 16 colours available in "colour" modes}
    SysVMD[SysVideoModeCount-1].Col:=Col;
    SysVMD[SysVideoModeCount-1].Row:=Row;
    SysVMD[SysVideoModeCount-1].Color:=(Color >= Colors_16);
    end;
  {Get the address of the original videobuffer and size.}
  if VioGetBuf (PScr, PWord (@OrigScreenSize)^, 0) = 0 then
    begin
    PScr := SelToFlat (PtrUInt (PScr));
    GetMem (OrigScreen, OrigScreenSize);
    Move (PScr^, OrigScreen^, OrigScreenSize);
    end;
end;


initialization
begin
  SetVideoDriver(SysVideoDriver);
  TargetEntry;
end;

finalization
  if (OrigScreenSize <> 0) and (OrigScreen <> nil) then
    begin
      FreeMem (OrigScreen, OrigScreenSize);
      OrigScreen := nil;
      OrigScreenSize := 0;
    end;
end.
