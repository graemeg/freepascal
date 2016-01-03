{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Video unit for DOS

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Video;

interface

{$i videoh.inc}

var
  VideoSeg : word;


implementation

uses
  mouse,
  go32;

{$i video.inc}

{$ASMMODE ATT}

  { used to know if LastCursorType is valid }
const
  LastCursorType : word = crUnderline;

{ allways set blink state again }

procedure SetHighBitBlink;
var
  regs : trealregs;
begin
  regs.ax:=$1003;
  regs.bx:=$0001;
  realintr($10,regs);
end;

function BIOSGetScreenMode(var Cols,Rows: word; var Color: boolean): boolean;
var r: trealregs;
    L: longint;
    LSel,LSeg: word;
    B: array[0..63] of byte;
type
  TWord = word;
  PWord = ^TWord;
var
  OK: boolean;
begin
  L:=global_dos_alloc(64);
  LSeg:=(L shr 16);
  LSel:=(L and $ffff);

  r.ah:=$1b; r.bx:=0;
  r.es:=LSeg; r.di:=0;
  realintr($10,r);
  OK:=(r.al=$1b);
  if OK then
  begin
    dpmi_dosmemget(LSeg,0,B,64);
    Cols:=PWord(@B[5])^; Rows:=B[$22];
    Color:=PWord(@B[$27])^<>0;
  end;
  global_dos_free(LSel);
  BIOSGetScreenMode:=OK;
end;

procedure SysInitVideo;
var
  regs : trealregs;
begin
  VideoSeg:=$b800;
  if (ScreenWidth=$ffff) or (ScreenHeight=$ffff) or
    (ScreenWidth=0) or (ScreenHeight=0) then
    begin
       ScreenColor:=true;
       regs.ah:=$0f;
       realintr($10,regs);
       if (regs.al and 1)=0 then
         ScreenColor:=false;
       if regs.al=7 then
         begin
            ScreenColor:=false;
            VideoSeg:=$b000;
         end
       else
         VideoSeg:=$b800;
       ScreenWidth:=regs.ah;
       regs.ax:=$1130;
       regs.bx:=0;
       realintr($10,regs);
       ScreenHeight:=regs.dl+1;
       BIOSGetScreenMode(ScreenWidth,ScreenHeight,ScreenColor);
    end;
  regs.ah:=$03;
  regs.bh:=0;
  realintr($10,regs);
  CursorLines:=regs.cl;
  CursorX:=regs.dl;
  CursorY:=regs.dh;
  SetHighBitBlink;
  SetCursorType(LastCursorType);
end;


procedure SysDoneVideo;
begin
  LastCursorType:=GetCursorType;
  ClearScreen;
  SetCursorType(crUnderLine);
  SetCursorPos(0,0);
end;


function SysGetCapabilities: Word;
begin
  SysGetCapabilities := $3F;
end;


procedure SysSetCursorPos(NewCursorX, NewCursorY: Word);
var
  regs : trealregs;
begin
  regs.ah:=$02;
  regs.bh:=0;
  regs.dh:=NewCursorY;
  regs.dl:=NewCursorX;
  realintr($10,regs);
  CursorY:=regs.dh;
  CursorX:=regs.dl;
end;

{ I don't know the maximum value for the scan line
  probably 7 or 15 depending on resolution !!
  }
function SysGetCursorType: Word;
var
  regs : trealregs;
begin
  regs.ah:=$03;
  regs.bh:=0;
  realintr($10,regs);
  SysGetCursorType:=crHidden;
  if (regs.ch and $60)=0 then
   begin
     SysGetCursorType:=crBlock;
     if (regs.ch and $1f)<>0 then
      begin
        SysGetCursorType:=crHalfBlock;
        if regs.cl-1=(regs.ch and $1F) then
         SysGetCursorType:=crUnderline;
      end;
   end;
end;


procedure SysSetCursorType(NewType: Word);
var
  regs : trealregs;
const
  MaxCursorLines = 7;
begin
  regs.ah:=$01;
  regs.bx:=NewType;
  case NewType of
   crHidden    : regs.cx:=$2000;
   crHalfBlock : begin
                   regs.ch:=MaxCursorLines shr 1;
                   regs.cl:=MaxCursorLines;
                 end;
   crBlock     : begin
                   regs.ch:=0;
                   regs.cl:=MaxCursorLines;
                 end;
   else          begin
                   regs.ch:=MaxCursorLines-1;
                   regs.cl:=MaxCursorLines;
                 end;
  end;
  realintr($10,regs);
end;

procedure SysUpdateScreen(Force: Boolean);
var
  Is_Mouse_Vis: boolean;
begin
  Is_Mouse_Vis := MouseIsVisible;     {MouseIsVisible is from Mouse unit}
  if Is_Mouse_Vis then
   HideMouse;
  if not force then
   begin
     asm
        pushl   %esi
        pushl   %edi
        movl    VideoBuf,%esi
        movl    OldVideoBuf,%edi
        movl    VideoBufSize,%ecx
        shrl    $2,%ecx
        repe
        cmpsl
        setne   force
        popl    %edi
        popl    %esi
     end;
   end;
  if Force then
   begin
     dosmemput(videoseg,0,videobuf^,VideoBufSize);
     move(videobuf^,oldvideobuf^,VideoBufSize);
   end;
  if Is_Mouse_Vis then
   ShowMouse;
end;

Procedure DoSetVideoMode(Params: Longint);

type
  wordrec=packed record
    lo,hi : word;
  end;
var
  regs : trealregs;
begin
  regs.ax:=wordrec(Params).lo;
  regs.bx:=wordrec(Params).hi;
  realintr($10,regs);
end;

Procedure SetVideo8x8;

type
  wordrec=packed record
    lo,hi : word;
  end;
var
  regs : trealregs;
begin
  regs.ax:=3;
  regs.bx:=0;
  realintr($10,regs);
  regs.ax:=$1112;
  regs.bx:=$0;
  realintr($10,regs);
end;

Const
  SysVideoModeCount = 5;
  SysVMD : Array[0..SysVideoModeCount-1] of TVideoMode = (
   (Col: 40; Row : 25;  Color : False),
   (Col: 40; Row : 25;  Color : True),
   (Col: 80; Row : 25;  Color : False),
   (Col: 80; Row : 25;  Color : True),
   (Col: 80; Row : 50;  Color : True)
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
    If (I<SysVideoModeCount-1) then
      DoSetVideoMode(I)
    else
      SetVideo8x8;
    ScreenWidth:=SysVMD[I].Col;
    ScreenHeight:=SysVMD[I].Row;
    ScreenColor:=SysVMD[I].Color;
    DoCustomMouse(false);
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

Const
  SysVideoDriver : TVideoDriver = (
    InitDriver      : @SysInitVideo;
    DoneDriver      : @SysDoneVideo;
    UpdateScreen    : @SysUpdateScreen;
    ClearScreen     : Nil;
    SetVideoMode    : @SysSetVideoMode;
    GetVideoModeCount : @SysGetVideoModeCount;
    GetVideoModeData : @SysGetVideoModedata;
    SetCursorPos    : @SysSetCursorPos;
    GetCursorType   : @SysGetCursorType;
    SetCursorType   : @SysSetCursorType;
    GetCapabilities : @SysGetCapabilities
  );

initialization
  SetVideoDriver(SysVideoDriver);
end.
