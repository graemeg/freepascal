{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl

    This file implements the go32v2 support for the graph unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Graph;
interface

{$i graphh.inc}
{$i vesah.inc}

CONST
  m640x200x16       = VGALo;
  m640x400x16       = VGAMed;
  m640x480x16       = VGAHi;

  { VESA Specific video modes. }
  m320x200x32k      = $10D;
  m320x200x64k      = $10E;

  m640x400x256      = $100;

  m640x480x256      = $101;
  m640x480x32k      = $110;
  m640x480x64k      = $111;

  m800x600x16       = $102;
  m800x600x256      = $103;
  m800x600x32k      = $113;
  m800x600x64k      = $114;

  m1024x768x16      = $104;
  m1024x768x256     = $105;
  m1024x768x32k     = $116;
  m1024x768x64k     = $117;

  m1280x1024x16     = $106;
  m1280x1024x256    = $107;
  m1280x1024x32k    = $119;
  m1280x1024x64k    = $11A;

const
  UseLFB : boolean = false;
  UseNoSelector : boolean = false;
  LFBPointer : pointer = nil;
{ Helpful variable to get save/restore support in IDE PM }
const
  DontClearGraphMemory : boolean = false;



implementation

uses
  go32,ports;

const
   InternalDriverName = 'DOSGX';

{$i graph.inc}


Type
  TDPMIRegisters = go32.registers;

{$asmmode intel}

{ How to access real mode memory }
{ using 32-bit DPMI memory       }
{  1. Allocate a descriptor      }
{  2. Set segment limit          }
{  3. Set base linear address    }
const
{$ifdef fpc}
   VideoOfs : longint = 0;   { Segment to draw to }
{$else fpc}
   VideoOfs : word = 0;   { Segment to draw to }
{$endif fpc}
   FirstPlane = $0102;   (* 02 = Index to Color plane Select, *)
                         (* 01 = Enable color plane 1         *)

{    ; ===== VGA Register Values ===== }

    SCREEN_WIDTH    =     80     ; { MODE-X 320 SCREEN WIDTH         }
                                   { CHANGE THE VALUE IF OTHER MODES }
                                   { OTHER THEN 320 ARE USED.        }
    ATTRIB_Ctrl     =   $03C0    ; { VGA Attribute Controller        }
    GC_Index        =   $03CE    ; { VGA Graphics Controller         }
    SC_Index        =   $03C4    ; { VGA Sequencer Controller        }
    SC_Data         =   $03C5    ; { VGA Sequencer Data Port         }
    CRTC_Index      =   $03D4    ; { VGA CRT Controller              }
    CRTC_Data       =   $03D5    ; { VGA CRT Controller Data         }
    MISC_OUTPUT     =   $03C2    ; { VGA Misc Register               }
    INPUT_1         =   $03DA    ; { Input Status #1 Register        }

    DAC_WRITE_ADDR  =   $03C8    ; { VGA DAC Write Addr Register     }
    DAC_READ_ADDR   =   $03C7    ; { VGA DAC Read Addr Register      }
    PEL_DATA_REG    =   $03C9    ; { VGA DAC/PEL data Register R/W   }

    PIXEL_PAN_REG   =   $033     ; { Attrib Index: Pixel Pan Reg     }
    MAP_MASK        =   $002     ; { S=   $Index: Write Map Mask reg }
    READ_MAP        =   $004     ; { GC Index: Read Map Register     }
    START_DISP_HI   =   $00C     ; { CRTC Index: Display Start Hi    }
    START_DISP_LO   =   $00D     ; { CRTC Index: Display Start Lo    }

    MAP_MASK_PLANE1 =   $00102   ; { Map Register + Plane 1          }
    MAP_MASK_PLANE2 =   $01102   ; { Map Register + Plane 1          }
    ALL_PLANES_ON   =   $00F02   ; { Map Register + All Bit Planes   }

    CHAIN4_OFF      =   $00604   ; { Chain 4 mode Off                }
    ASYNC_RESET     =   $00100   ; { (A)synchronous Reset            }
    SEQU_RESTART    =   $00300   ; { Sequencer Restart               }

    LATCHES_ON      =   $00008   ; { Bit Mask + Data from Latches    }
    LATCHES_OFF     =   $0FF08   ; { Bit Mask + Data from CPU        }

    VERT_RETRACE    =   $08      ; { INPUT_1: Vertical Retrace Bit   }
    PLANE_BITS      =   $03      ; { Bits 0-1 of Xpos = Plane #      }
    ALL_PLANES      =   $0F      ; { All Bit Planes Selected         }
    CHAR_BITS       =   $0F      ; { Bits 0-3 of Character Data      }

    GET_CHAR_PTR    =   $01130   ; { VGA BIOS Func: Get Char Set     }
    ROM_8x8_Lo      =   $03      ; { ROM 8x8 Char Set Lo Pointer     }
    ROM_8x8_Hi      =   $04      ; { ROM 8x8 Char Set Hi Pointer     }

    { Constants Specific for these routines                          }

    NUM_MODES       =   $8       ; { # of Mode X Variations           }

    { in 16 color modes, the actual colors used are not 0..15, but: }
    ToRealCols16: Array[0..15] of word =
      (0,1,2,3,4,5,20,7,56,57,58,59,60,61,62,63);

  var
     ScrWidth : word absolute $40:$4a;
     inWindows: boolean;

{$ifndef tp}
  Procedure seg_bytemove(sseg : word;source : longint;dseg : word;dest : longint;count : longint); assembler;
    asm
      push edi
      push esi
      push es
      push ds
      cld
      mov es, dseg
      mov esi, source
      mov edi, dest
      mov ecx, count
      mov ds,sseg
      rep movsb
      pop ds
      pop es
      pop esi
      pop edi
    end ['ECX'];
{$endif tp}

 Procedure CallInt10(val_ax : word); assembler;
   asm
{$IFNDEF REGCALL}
     mov ax,val_ax
{$ENDIF REGCALL}
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
   end ['EAX'];

<<<<<<< HEAD
<<<<<<< HEAD
{************************************************************************}
{*                   720x348x2 Hercules mode routines                   *}
{************************************************************************}
<<<<<<< HEAD
=======

var
  DummyHGCBkColor: Word;

procedure InitHGC720;
const
  RegValues: array [0..11] of byte =
    ($35, $2D, $2E, $07, $5B, $02, $57, $57, $02, $03, $00, $00);
var
  I: Integer;
begin
  Port[$3BF] := 3; { graphic and page 2 possible }
  Port[$3B8] := 2; { display page 0, graphic mode, display off }
  for I := 0 to 11 do
    PortW[$3B4] := I or (RegValues[I] shl 8);
  Port[$3B8] := 10; { display page 0, graphic mode, display on }
  DosMemFillChar($B000, 0, 65536, #0);
  VideoOfs := 0;
  DummyHGCBkColor := 0;
end;

{ compatible with TP7's HERC.BGI }
procedure SetBkColorHGC720(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  DummyHGCBkColor := ColorNum;
end;

{ compatible with TP7's HERC.BGI }
function GetBkColorHGC720: Word;
begin
  GetBkColorHGC720 := DummyHGCBkColor;
end;

procedure SetHGCRGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
begin
end;

procedure GetHGCRGBPalette(ColorNum: smallint; Var
      RedValue, GreenValue, BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
begin
end;

procedure PutPixelHGC720(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := (Y shr 2) * 90 + (X shr 3) + VideoOfs;
  case Y and 3 of
    1: Inc(Offset, $2000);
    2: Inc(Offset, $4000);
    3: Inc(Offset, $6000);
  end;
  Shift := 7 - (X and 7);
  Mask := 1 shl Shift;
  B := Mem[SegB000:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegB000:Offset] := B;
end;

function GetPixelHGC720(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := (Y shr 2) * 90 + (X shr 3) + VideoOfs;
  case Y and 3 of
    1: Inc(Offset, $2000);
    2: Inc(Offset, $4000);
    3: Inc(Offset, $6000);
  end;
  Shift := 7 - (X and 7);
  B := Mem[SegB000:Offset];
  GetPixelHGC720 := (B shr Shift) and 1;
end;

procedure DirectPutPixelHGC720(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := (Y shr 2) * 90 + (X shr 3) + VideoOfs;
  case Y and 3 of
    1: Inc(Offset, $2000);
    2: Inc(Offset, $4000);
    3: Inc(Offset, $6000);
  end;
  Shift := 7 - (X and 7);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB000:Offset] := Mem[SegB000:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB000:Offset] := Mem[SegB000:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 1 then
          exit;
        { therefore, CurrentColor must be 0 }
        Mem[SegB000:Offset] := Mem[SegB000:Offset] and (not (1 shl Shift));
      end;
    NotPut:
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB000:Offset];
        B := B and (not Mask) or ((CurrentColor xor $01) shl Shift);
        Mem[SegB000:Offset] := B;
      end
    else
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB000:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegB000:Offset] := B;
      end;
  end;
end;

procedure HLineHGC720(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := (Y shr 2) * 90 + VideoOfs;
  case Y and 3 of
    1: Inc(YOffset, $2000);
    2: Inc(YOffset, $4000);
    3: Inc(YOffset, $6000);
  end;
  LOffset := YOffset + (X shr 3);
  ROffset := YOffset + (X2 shr 3);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $01
  else
    Color := CurrentColor;
  if Color = 1 then
    ForeMask := $FF
  else
    ForeMask := $00;

  LBackMask := Byte($FF00 shr (X and $07));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (7 - (X2 and $07))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] and LBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB000:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegB000:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] xor $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := 0;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  { draw the final right byte, if less than 100% full }
  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] and RBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB000:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegB000:CurrentOffset] := B;
        end;
    end;
  end;
end;

procedure SetVisualHGC720(page: word); {$ifndef fpc}far;{$endif fpc}
{ two page supPort... }
begin
  if page > HardwarePages then exit;

  case page of
   0 : Port[$3B8] := 10; { display page 0, graphic mode, display on }
   1 : Port[$3B8] := 10+128; { display page 1, graphic mode, display on }
  end;
end;

procedure SetActiveHGC720(page: word); {$ifndef fpc}far;{$endif fpc}
{ two page supPort... }
begin
  case page of
   0 : VideoOfs := 0;
   1 : VideoOfs := 32768;
  else
    VideoOfs := 0;
  end;
end;

{************************************************************************}
{*                     320x200x4 CGA mode routines                      *}
{************************************************************************}
var
  CurrentCGABorder: Word;

procedure SetCGAPalette(CGAPaletteID: Byte); assembler;
asm
{$IFNDEF REGCALL}
  mov ax,val_ax
{$ENDIF REGCALL}
{$ifdef fpc}
  push ebp
  push esi
  push edi
  push ebx
{$endif fpc}
  mov bl, al
  mov bh, 1
  mov ah, 0Bh
  int 10h
{$ifdef fpc}
  pop ebx
  pop edi
  pop esi
  pop ebp
{$endif fpc}
end;

procedure SetCGABorder(CGABorder: Byte); assembler;
asm
{$IFNDEF REGCALL}
  mov ax,val_ax
{$ENDIF REGCALL}
{$ifdef fpc}
  push ebp
  push esi
  push edi
  push ebx
{$endif fpc}
  mov bl, al
  mov bh, 0
  mov ah, 0Bh
  int 10h
{$ifdef fpc}
  pop ebx
  pop edi
  pop esi
  pop ebp
{$endif fpc}
end;

procedure SetBkColorCGA320(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABorder := (CurrentCGABorder and 16) or ColorNum;
  SetCGABorder(CurrentCGABorder);
end;

function GetBkColorCGA320: Word;
begin
  GetBkColorCGA320 := CurrentCGABorder and 15;
end;

procedure InitCGA320C0;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(0);
  SetCGABorder(16);
  CurrentCGABorder := 16;
end;

procedure InitCGA320C1;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(1);
  SetCGABorder(16);
  CurrentCGABorder := 16;
end;

procedure InitCGA320C2;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(2);
  SetCGABorder(0);
  CurrentCGABorder := 0;
end;

procedure InitCGA320C3;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(3);
  SetCGABorder(0);
  CurrentCGABorder := 0;
end;

procedure PutPixelCGA320(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := (Y shr 1) * 80 + (X shr 2);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 6 - ((X and 3) shl 1);
  Mask := $03 shl Shift;
  B := Mem[SegB800:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegB800:Offset] := B;
end;

function GetPixelCGA320(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := (Y shr 1) * 80 + (X shr 2);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 6 - ((X and 3) shl 1);
  B := Mem[SegB800:Offset];
  GetPixelCGA320 := (B shr Shift) and $03;
end;

procedure DirectPutPixelCGA320(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := (Y shr 1) * 80 + (X shr 2);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 6 - ((X and 3) shl 1);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 3 then
          exit;
        Mask := $03 shl Shift;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] and ((CurrentColor shl Shift) or (not Mask));
      end;
    NotPut:
      begin
        Mask := $03 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or ((CurrentColor xor $03) shl Shift);
        Mem[SegB800:Offset] := B;
      end
    else
      begin
        Mask := $03 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegB800:Offset] := B;
      end;
  end;
end;

procedure HLineCGA320(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := (Y shr 1) * 80;
  if (Y and 1) <> 0 then
    Inc(YOffset, 8192);
  LOffset := YOffset + (X shr 2);
  ROffset := YOffset + (X2 shr 2);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $03
  else
    Color := CurrentColor;
  case Color of
    0: ForeMask := $00;
    1: ForeMask := $55;
    2: ForeMask := $AA;
    3: ForeMask := $FF;
  end;

  LBackMask := Byte($FF00 shr ((X and $03) shl 1));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (6 - ((X2 and $03) shl 1))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and (LBackMask or LForeMask);
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  { draw the final right byte, if less than 100% full }
  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and (RBackMask or RForeMask);
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
  end;
end;

{************************************************************************}
{*                     640x200x2 CGA mode routines                      *}
{************************************************************************}

procedure InitCGA640;
begin
  if DontClearGraphMemory then
    CallInt10($86)
  else
    CallInt10($06);
  VideoOfs := 0;
  CurrentCGABorder := 0; {yes, TP7 CGA.BGI behaves *exactly* like that}
end;

{yes, TP7 CGA.BGI behaves *exactly* like that}
procedure SetBkColorCGA640(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABorder := ColorNum;
  if ColorNum = 0 then
    exit;
  SetCGABorder(CurrentCGABorder);
end;

function GetBkColorCGA640: Word;
begin
  GetBkColorCGA640 := CurrentCGABorder and 15;
end;

procedure PutPixelCGA640(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := (Y shr 1) * 80 + (X shr 3);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 7 - (X and 7);
  Mask := 1 shl Shift;
  B := Mem[SegB800:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegB800:Offset] := B;
end;

function GetPixelCGA640(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := (Y shr 1) * 80 + (X shr 3);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 7 - (X and 7);
  B := Mem[SegB800:Offset];
  GetPixelCGA640 := (B shr Shift) and 1;
end;

procedure DirectPutPixelCGA640(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := (Y shr 1) * 80 + (X shr 3);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 7 - (X and 7);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 1 then
          exit;
        { therefore, CurrentColor must be 0 }
        Mem[SegB800:Offset] := Mem[SegB800:Offset] and (not (1 shl Shift));
      end;
    NotPut:
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or ((CurrentColor xor $01) shl Shift);
        Mem[SegB800:Offset] := B;
      end
    else
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegB800:Offset] := B;
      end;
  end;
end;

procedure HLineCGA640(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := (Y shr 1) * 80;
  if (Y and 1) <> 0 then
    Inc(YOffset, 8192);
  LOffset := YOffset + (X shr 3);
  ROffset := YOffset + (X2 shr 3);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $01
  else
    Color := CurrentColor;
  if Color = 1 then
    ForeMask := $FF
  else
    ForeMask := $00;

  LBackMask := Byte($FF00 shr (X and $07));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (7 - (X2 and $07))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and LBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := 0;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  { draw the final right byte, if less than 100% full }
  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and RBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
  end;
end;

{************************************************************************}
{*                    640x480x2 MCGA mode routines                      *}
{************************************************************************}

procedure InitMCGA640;
begin
  if DontClearGraphMemory then
    CallInt10($91)
  else
    CallInt10($11);
  VideoOfs := 0;
  CurrentCGABorder := 0; {yes, TP7 CGA.BGI behaves *exactly* like that}
end;

procedure SetBkColorMCGA640(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABorder := (CurrentCGABorder and 16) or ColorNum;
  SetCGABorder(CurrentCGABorder);
end;

function GetBkColorMCGA640: Word;
begin
  GetBkColorMCGA640 := CurrentCGABorder and 15;
end;

procedure PutPixelMCGA640(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := Y * 80 + (X shr 3);
  Shift := 7 - (X and 7);
  Mask := 1 shl Shift;
  B := Mem[SegA000:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegA000:Offset] := B;
end;

function GetPixelMCGA640(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := Y * 80 + (X shr 3);
  Shift := 7 - (X and 7);
  B := Mem[SegA000:Offset];
  GetPixelMCGA640 := (B shr Shift) and 1;
end;

procedure DirectPutPixelMCGA640(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := Y * 80 + (X shr 3);
  Shift := 7 - (X and 7);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegA000:Offset] := Mem[SegA000:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegA000:Offset] := Mem[SegA000:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 1 then
          exit;
        { therefore, CurrentColor must be 0 }
        Mem[SegA000:Offset] := Mem[SegA000:Offset] and (not (1 shl Shift));
      end;
    NotPut:
      begin
        Mask := 1 shl Shift;
        B := Mem[SegA000:Offset];
        B := B and (not Mask) or ((CurrentColor xor $01) shl Shift);
        Mem[SegA000:Offset] := B;
      end
    else
      begin
        Mask := 1 shl Shift;
        B := Mem[SegA000:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegA000:Offset] := B;
      end;
  end;
end;

procedure HLineMCGA640(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := Y * 80;
  LOffset := YOffset + (X shr 3);
  ROffset := YOffset + (X2 shr 3);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $01
  else
    Color := CurrentColor;
  if Color = 1 then
    ForeMask := $FF
  else
    ForeMask := $00;

  LBackMask := Byte($FF00 shr (X and $07));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (7 - (X2 and $07))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] and LBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegA000:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegA000:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] xor $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := 0;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] and RBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegA000:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegA000:CurrentOffset] := B;
        end;
    end;
  end;
end;

 {************************************************************************}
 {*                     4-bit planar VGA mode routines                   *}
 {************************************************************************}

  Procedure Init640x200x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($8e)
      else
        CallInt10($e);
      VideoOfs := 0;
    end;
>>>>>>> origin/cpstrnew

var
  DummyHGCBkColor: Word;

<<<<<<< HEAD
procedure InitHGC720;
const
  RegValues: array [0..11] of byte =
    ($35, $2D, $2E, $07, $5B, $02, $57, $57, $02, $03, $00, $00);
var
  I: Integer;
begin
  Port[$3BF] := 3; { graphic and page 2 possible }
  Port[$3B8] := 2; { display page 0, graphic mode, display off }
  for I := 0 to 11 do
    PortW[$3B4] := I or (RegValues[I] shl 8);
  Port[$3B8] := 10; { display page 0, graphic mode, display on }
  DosMemFillChar($B000, 0, 65536, #0);
  VideoOfs := 0;
  DummyHGCBkColor := 0;
end;
=======
   Procedure Init640x350x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($90)
      else
        CallInt10($10);
      VideoOfs := 0;
    end;
>>>>>>> origin/cpstrnew

{ compatible with TP7's HERC.BGI }
procedure SetBkColorHGC720(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  DummyHGCBkColor := ColorNum;
end;

{ compatible with TP7's HERC.BGI }
function GetBkColorHGC720: Word;
begin
  GetBkColorHGC720 := DummyHGCBkColor;
end;

<<<<<<< HEAD
procedure SetHGCRGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
begin
end;
=======
  Procedure Init640x480x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($92)
      else
        CallInt10($12);
      VideoOfs := 0;
    end;
>>>>>>> origin/cpstrnew

procedure GetHGCRGBPalette(ColorNum: smallint; Var
      RedValue, GreenValue, BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
begin
end;

procedure PutPixelHGC720(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := (Y shr 2) * 90 + (X shr 3) + VideoOfs;
  case Y and 3 of
    1: Inc(Offset, $2000);
    2: Inc(Offset, $4000);
    3: Inc(Offset, $6000);
  end;
  Shift := 7 - (X and 7);
  Mask := 1 shl Shift;
  B := Mem[SegB000:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegB000:Offset] := B;
end;

function GetPixelHGC720(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := (Y shr 2) * 90 + (X shr 3) + VideoOfs;
  case Y and 3 of
    1: Inc(Offset, $2000);
    2: Inc(Offset, $4000);
    3: Inc(Offset, $6000);
  end;
  Shift := 7 - (X and 7);
  B := Mem[SegB000:Offset];
  GetPixelHGC720 := (B shr Shift) and 1;
end;

procedure DirectPutPixelHGC720(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := (Y shr 2) * 90 + (X shr 3) + VideoOfs;
  case Y and 3 of
    1: Inc(Offset, $2000);
    2: Inc(Offset, $4000);
    3: Inc(Offset, $6000);
  end;
  Shift := 7 - (X and 7);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB000:Offset] := Mem[SegB000:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB000:Offset] := Mem[SegB000:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 1 then
          exit;
        { therefore, CurrentColor must be 0 }
        Mem[SegB000:Offset] := Mem[SegB000:Offset] and (not (1 shl Shift));
      end;
    NotPut:
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB000:Offset];
        B := B and (not Mask) or ((CurrentColor xor $01) shl Shift);
        Mem[SegB000:Offset] := B;
      end
    else
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB000:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegB000:Offset] := B;
      end;
  end;
end;

<<<<<<< HEAD
procedure HLineHGC720(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := (Y shr 2) * 90 + VideoOfs;
  case Y and 3 of
    1: Inc(YOffset, $2000);
    2: Inc(YOffset, $4000);
    3: Inc(YOffset, $6000);
  end;
  LOffset := YOffset + (X shr 3);
  ROffset := YOffset + (X2 shr 3);
=======
     PortW[$3ce] := ($8000 shr (x and $7)) or 8; { Select correct bits to modify }
     dummy := Mem[SegA000: offset];  { Latch the data into host space.  }
     Mem[Sega000: offset] := dummy;  { Write the data into video memory }
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
{$else asmgraph}
      asm
 {$ifndef fpc}
        mov  es, [SegA000]
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        mov  ax,[X]
        shr  ax,3
        push ax
        { determine the address }
        mov  ax,80
        mov  bx,[Y]
        mul  bx
        pop  cx
        add  ax,cx
        mov  di,ax
        add  di, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,es:[di]
        mov  es:[di],bl
>>>>>>> origin/cpstrnew

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $01
  else
    Color := CurrentColor;
  if Color = 1 then
    ForeMask := $FF
  else
    ForeMask := $00;

  LBackMask := Byte($FF00 shr (X and $07));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (7 - (X2 and $07))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

<<<<<<< HEAD
  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] and LBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB000:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegB000:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;
=======
 Function GetPixel16(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 Var dummy, offset: Word;
     shift: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
{$ifndef asmgraph}
    offset := Y * 80 + (x shr 3) + VideoOfs;
    PortW[$3ce] := $0004;
    shift := 7 - (X and 7);
    dummy := (Mem[Sega000:offset] shr shift) and 1;
    Port[$3cf] := 1;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 1);
    Port[$3cf] := 2;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 2);
    Port[$3cf] := 3;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 3);
    GetPixel16 := dummy;
{$else asmgraph}
    asm
  {$ifndef fpc}
      mov   ax, [X]          { Get X address                    }
      push  ax
      shr   ax, 3
      push  ax
>>>>>>> origin/cpstrnew

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] xor $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := 0;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB000:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  { draw the final right byte, if less than 100% full }
  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB000:CurrentOffset] := Mem[SegB000:CurrentOffset] and RBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB000:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegB000:CurrentOffset] := B;
        end;
    end;
  end;
end;

procedure SetVisualHGC720(page: word); {$ifndef fpc}far;{$endif fpc}
{ two page supPort... }
begin
  if page > HardwarePages then exit;

  case page of
   0 : Port[$3B8] := 10; { display page 0, graphic mode, display on }
   1 : Port[$3B8] := 10+128; { display page 1, graphic mode, display on }
  end;
end;

procedure SetActiveHGC720(page: word); {$ifndef fpc}far;{$endif fpc}
{ two page supPort... }
begin
  case page of
   0 : VideoOfs := 0;
   1 : VideoOfs := 32768;
  else
    VideoOfs := 0;
  end;
end;

{************************************************************************}
{*                     320x200x4 CGA mode routines                      *}
{************************************************************************}
var
  CurrentCGABorder: Word;

procedure SetCGAPalette(CGAPaletteID: Byte); assembler;
asm
{$IFNDEF REGCALL}
  mov ax,val_ax
{$ENDIF REGCALL}
{$ifdef fpc}
  push ebp
  push esi
  push edi
  push ebx
{$endif fpc}
  mov bl, al
  mov bh, 1
  mov ah, 0Bh
  int 10h
{$ifdef fpc}
  pop ebx
  pop edi
  pop esi
  pop ebp
{$endif fpc}
end;

procedure SetCGABorder(CGABorder: Byte); assembler;
asm
{$IFNDEF REGCALL}
  mov ax,val_ax
{$ENDIF REGCALL}
{$ifdef fpc}
  push ebp
  push esi
  push edi
  push ebx
{$endif fpc}
  mov bl, al
  mov bh, 0
  mov ah, 0Bh
  int 10h
{$ifdef fpc}
  pop ebx
  pop edi
  pop esi
  pop ebp
{$endif fpc}
end;

procedure SetBkColorCGA320(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABorder := (CurrentCGABorder and 16) or ColorNum;
  SetCGABorder(CurrentCGABorder);
end;

function GetBkColorCGA320: Word;
begin
  GetBkColorCGA320 := CurrentCGABorder and 15;
end;

procedure InitCGA320C0;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(0);
  SetCGABorder(16);
  CurrentCGABorder := 16;
end;

procedure InitCGA320C1;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(1);
  SetCGABorder(16);
  CurrentCGABorder := 16;
end;

procedure InitCGA320C2;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(2);
  SetCGABorder(0);
  CurrentCGABorder := 0;
end;

procedure InitCGA320C3;
begin
  if DontClearGraphMemory then
    CallInt10($84)
  else
    CallInt10($04);
  VideoOfs := 0;
  SetCGAPalette(3);
  SetCGABorder(0);
  CurrentCGABorder := 0;
end;

procedure PutPixelCGA320(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := (Y shr 1) * 80 + (X shr 2);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 6 - ((X and 3) shl 1);
  Mask := $03 shl Shift;
  B := Mem[SegB800:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegB800:Offset] := B;
end;

function GetPixelCGA320(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := (Y shr 1) * 80 + (X shr 2);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 6 - ((X and 3) shl 1);
  B := Mem[SegB800:Offset];
  GetPixelCGA320 := (B shr Shift) and $03;
end;

procedure DirectPutPixelCGA320(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := (Y shr 1) * 80 + (X shr 2);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 6 - ((X and 3) shl 1);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 3 then
          exit;
        Mask := $03 shl Shift;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] and ((CurrentColor shl Shift) or (not Mask));
      end;
    NotPut:
      begin
        Mask := $03 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or ((CurrentColor xor $03) shl Shift);
        Mem[SegB800:Offset] := B;
      end
    else
      begin
        Mask := $03 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegB800:Offset] := B;
      end;
  end;
end;

procedure HLineCGA320(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := (Y shr 1) * 80;
  if (Y and 1) <> 0 then
    Inc(YOffset, 8192);
  LOffset := YOffset + (X shr 2);
  ROffset := YOffset + (X2 shr 2);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $03
  else
    Color := CurrentColor;
  case Color of
    0: ForeMask := $00;
    1: ForeMask := $55;
    2: ForeMask := $AA;
    3: ForeMask := $FF;
  end;

  LBackMask := Byte($FF00 shr ((X and $03) shl 1));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (6 - ((X2 and $03) shl 1))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          if CurrentColor = 3 then
=======
          if CurrentColor = 1 then
>>>>>>> graemeg/cpstrnew
=======
          if CurrentColor = 1 then
>>>>>>> graemeg/cpstrnew
=======
          if CurrentColor = 1 then
>>>>>>> graemeg/cpstrnew
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and (LBackMask or LForeMask);
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

<<<<<<< HEAD
  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          if CurrentColor = 3 then
            exit;
=======
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
>>>>>>> graemeg/cpstrnew
=======
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
>>>>>>> graemeg/cpstrnew
=======
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
>>>>>>> graemeg/cpstrnew
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  { draw the final right byte, if less than 100% full }
  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          if CurrentColor = 3 then
=======
          if CurrentColor = 1 then
>>>>>>> graemeg/cpstrnew
=======
          if CurrentColor = 1 then
>>>>>>> graemeg/cpstrnew
=======
          if CurrentColor = 1 then
>>>>>>> graemeg/cpstrnew
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and (RBackMask or RForeMask);
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
  end;
end;
=======
var dummylong: longint;
    Offset, count, count2, amount, index: word;
    plane: byte;
Begin
  inc(x1,StartXViewPort);
  inc(x2,StartXViewPort);
{$ifdef logging}
  LogLn('GetScanLine16 start, length to get: '+strf(x2-x1+1)+' at y = '+strf(y));
{$Endif logging}
  offset := (Y + StartYViewPort) * 80 + (x1 shr 3) + VideoOfs;
{$ifdef logging}
  LogLn('Offset: '+HexStr(offset,4)+' - ' + strf(offset));
{$Endif logging}
  { first get enough pixels so offset is 32bit aligned }
  amount := 0;
  index := 0;
  If ((x1 and 31) <> 0) Or
     ((x2-x1+1) < 32) Then
    Begin
      If ((x2-x1+1) >= 32+32-(x1 and 31)) Then
        amount := 32-(x1 and 31)
      Else amount := x2-x1+1;
{$ifdef logging}
      LogLn('amount to align to 32bits or to get all: ' + strf(amount));
{$Endif logging}
      For count := 0 to amount-1 do
        WordArray(Data)[Count] := getpixel16(x1-StartXViewPort+Count,y);
      index := amount;
      Inc(Offset,(amount+7) shr 3);
{$ifdef logging}
      LogLn('offset now: '+HexStr(offset,4)+' - ' + strf(offset));
      LogLn('index now: '+strf(index));
{$Endif logging}
    End;
  amount := x2-x1+1 - amount;
{$ifdef logging}
  LogLn('amount left: ' + strf(amount));
{$Endif logging}
  If amount = 0 Then Exit;
  { first get everything from plane 3 (4th plane) }
  PortW[$3ce] := $0304;
  Count := 0;
  For Count := 1 to (amount shr 5) Do
    Begin
      dummylong := MemL[SegA000:offset+(Count-1)*4];
      dummylong :=
        ((dummylong and $ff) shl 24) or
        ((dummylong and $ff00) shl 8) or
        ((dummylong and $ff0000) shr 8) or
        ((dummylong and $ff000000) shr 24);
      For Count2 := 31 downto 0 Do
        Begin
          WordArray(Data)[index+Count2] := DummyLong and 1;
          DummyLong := DummyLong shr 1;
        End;
      Inc(Index, 32);
    End;
{ Now get the data from the 3 other planes }
  plane := 3;
  Repeat
    Dec(Index,Count*32);
    Dec(plane);
    Port[$3cf] := plane;
    Count := 0;
    For Count := 1 to (amount shr 5) Do
      Begin
        dummylong := MemL[SegA000:offset+(Count-1)*4];
        dummylong :=
          ((dummylong and $ff) shl 24) or
          ((dummylong and $ff00) shl 8) or
          ((dummylong and $ff0000) shr 8) or
          ((dummylong and $ff000000) shr 24);
        For Count2 := 31 downto 0 Do
          Begin
            WordArray(Data)[index+Count2] :=
              (WordArray(Data)[index+Count2] shl 1) or (DummyLong and 1);
            DummyLong := DummyLong shr 1;
          End;
        Inc(Index, 32);
      End;
  Until plane = 0;
  amount := amount and 31;
  Dec(index);
{$ifdef Logging}
  LogLn('Last array index written to: '+strf(index));
  LogLn('amount left: '+strf(amount)+' starting at x = '+strf(index+1));
{$Endif logging}
  dec(x1,startXViewPort);
  For Count := 1 to amount Do
    WordArray(Data)[index+Count] := getpixel16(x1+index+Count,y);
{$ifdef logging}
  inc(x1,startXViewPort);
  LogLn('First 32 bytes gotten with getscanline16: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanline16: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  GetScanLineDefault(x1-StartXViewPort,x2-StartXViewPort,y,Data);
  LogLn('First 32 bytes gotten with getscanlinedef: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanlinedef: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  LogLn('GetScanLine16 end');
{$Endif logging}
End;

 Procedure DirectPutPixel16(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
  var
   color: word;
{$ifndef asmgraph}
  offset: word;
  dummy: byte;
{$endif asmgraph}
 begin
    If CurrentWriteMode <> NotPut Then
      Color := CurrentColor
    else Color := not CurrentColor;

    case CurrentWriteMode of
       XORPut:
         PortW[$3ce]:=((3 shl 3) shl 8) or 3;
       ANDPut:
         PortW[$3ce]:=((1 shl 3) shl 8) or 3;
       ORPut:
         PortW[$3ce]:=((2 shl 3) shl 8) or 3;
       {not needed, this is the default state (e.g. PutPixel16 requires it)}
       {NormalPut, NotPut:
         PortW[$3ce]:=$0003
       else
         PortW[$3ce]:=$0003}
    end;
{$ifndef asmgraph}
    offset := Y * 80 + (X shr 3) + VideoOfs;
    PortW[$3ce] := $f01;
    PortW[$3ce] := Color shl 8;
    PortW[$3ce] := ($8000 shr (X and 7)) or 8;
    dummy := Mem[SegA000: offset];
    Mem[Sega000: offset] := dummy;
    PortW[$3ce] := $ff08;
    PortW[$3ce] := $0001;
    if (CurrentWriteMode = XORPut) or
       (CurrentWriteMode = ANDPut) or
       (CurrentWriteMode = ORPut) then
      PortW[$3ce] := $0003;
{$else asmgraph}
{ note: still needs xor/or/and/notput support !!!!! (JM) }
    asm
  {$ifndef fpc}
      mov  es, [SegA000]
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      mov  ax,[X]
      shr  ax,3
      push ax
      { determine the address }
      mov  ax,80
      mov  bx,[Y]
      mul  bx
      pop  cx
      add  ax,cx
      mov  di,ax
      { send the data through the display memory through set/reset }
      add  di,[VideoOfs]   { add correct page }
      mov  bl,es:[di]
      mov  es:[di],bl
>>>>>>> origin/cpstrnew

{************************************************************************}
{*                     640x200x2 CGA mode routines                      *}
{************************************************************************}

procedure InitCGA640;
begin
  if DontClearGraphMemory then
    CallInt10($86)
  else
    CallInt10($06);
  VideoOfs := 0;
  CurrentCGABorder := 0; {yes, TP7 CGA.BGI behaves *exactly* like that}
end;

{yes, TP7 CGA.BGI behaves *exactly* like that}
procedure SetBkColorCGA640(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABorder := ColorNum;
  if ColorNum = 0 then
    exit;
  SetCGABorder(CurrentCGABorder);
end;

function GetBkColorCGA640: Word;
begin
  GetBkColorCGA640 := CurrentCGABorder and 15;
end;

procedure PutPixelCGA640(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := (Y shr 1) * 80 + (X shr 3);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 7 - (X and 7);
  Mask := 1 shl Shift;
  B := Mem[SegB800:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegB800:Offset] := B;
end;

function GetPixelCGA640(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := (Y shr 1) * 80 + (X shr 3);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 7 - (X and 7);
  B := Mem[SegB800:Offset];
  GetPixelCGA640 := (B shr Shift) and 1;
end;

procedure DirectPutPixelCGA640(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := (Y shr 1) * 80 + (X shr 3);
  if (Y and 1) <> 0 then
    Inc(Offset, 8192);
  Shift := 7 - (X and 7);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] xor (CurrentColor shl Shift);
      end;
<<<<<<< HEAD
    OrPut:
=======
    ScrOfs:=y*ScrWidth+x div 8 + VideoOfs;
    HLength:=x2 div 8-x div 8;
    LMask:=$ff shr (x and 7);
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    RMask:=$ff shl (7-(x2 and 7));
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    if HLength=0 then
      LMask:=LMask and RMask;
    If CurrentWriteMode <> NotPut Then
      PortW[$3ce]:= CurrentColor shl 8
    else PortW[$3ce]:= (not CurrentColor) shl 8;
    PortW[$3ce]:=$0f01;
    case CurrentWriteMode of
       XORPut:
         PortW[$3ce]:=((3 shl 3) shl 8) or 3;
       ANDPut:
         PortW[$3ce]:=((1 shl 3) shl 8) or 3;
       ORPut:
         PortW[$3ce]:=((2 shl 3) shl 8) or 3;
       NormalPut, NotPut:
         PortW[$3ce]:=$0003
       else
         PortW[$3ce]:=$0003
    end;

    PortW[$3ce]:=(LMask shl 8) or 8;
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    Mem[SegA000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    {Port[$3ce]:=8;}{not needed, the register is already selected}
    if HLength>0 then
>>>>>>> origin/cpstrnew
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegB800:Offset] := Mem[SegB800:Offset] or (CurrentColor shl Shift);
      end;
<<<<<<< HEAD
    AndPut:
      begin
        { optimization }
        if CurrentColor = 1 then
          exit;
        { therefore, CurrentColor must be 0 }
        Mem[SegB800:Offset] := Mem[SegB800:Offset] and (not (1 shl Shift));
      end;
    NotPut:
=======
    { clean up }
    {Port[$3cf]:=0;}{not needed, the register is reset by the next operation:}
    PortW[$3ce]:=$ff08;
    PortW[$3ce]:=$0001;
    PortW[$3ce]:=$0003;
   end;

  procedure VLine16(x,y,y2: smallint); {$ifndef fpc}far;{$endif fpc}

   var
     ytmp: smallint;
     ScrOfs,i : longint;
     BitMask : byte;

  Begin
    { must we swap the values? }
    if y > y2 then
     Begin
       ytmp := y2;
       y2 := y;
       y:= ytmp;
     end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    Y2  := Y2 + StartYViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8 + VideoOfs;
    BitMask:=$80 shr (x and 7);
    If CurrentWriteMode <> NotPut Then
      PortW[$3ce]:= (CurrentColor shl 8)
    else PortW[$3ce]:= (not CurrentColor) shl 8;
    PortW[$3ce]:=$0f01;
    PortW[$3ce]:=(BitMask shl 8) or 8;
    case CurrentWriteMode of
       XORPut:
         PortW[$3ce]:=((3 shl 3) shl 8) or 3;
       ANDPut:
         PortW[$3ce]:=((1 shl 3) shl 8) or 3;
       ORPut:
         PortW[$3ce]:=((2 shl 3) shl 8) or 3;
       NormalPut, NotPut:
         PortW[$3ce]:=$0003
       else
         PortW[$3ce]:=$0003
    end;
    for i:=y to y2 do
>>>>>>> origin/cpstrnew
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or ((CurrentColor xor $01) shl Shift);
        Mem[SegB800:Offset] := B;
      end
    else
      begin
        Mask := 1 shl Shift;
        B := Mem[SegB800:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegB800:Offset] := B;
      end;
<<<<<<< HEAD
  end;
end;

procedure HLineCGA640(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := (Y shr 1) * 80;
  if (Y and 1) <> 0 then
    Inc(YOffset, 8192);
  LOffset := YOffset + (X shr 3);
  ROffset := YOffset + (X2 shr 3);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $01
  else
    Color := CurrentColor;
  if Color = 1 then
    ForeMask := $FF
  else
    ForeMask := $00;

  LBackMask := Byte($FF00 shr (X and $07));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (7 - (X2 and $07))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
=======
    { clean up }
    {Port[$3cf]:=0;}{not needed, the register is reset by the next operation}
    PortW[$3ce]:=$ff08;
    PortW[$3ce]:=$0001;
    PortW[$3ce]:=$0003;
  End;


 procedure SetVisual200(page: word); {$ifndef fpc}far;{$endif fpc}
  { two page supPort... }
>>>>>>> origin/cpstrnew
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and LBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := 0;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegB800:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  { draw the final right byte, if less than 100% full }
  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegB800:CurrentOffset] := Mem[SegB800:CurrentOffset] and RBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegB800:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegB800:CurrentOffset] := B;
        end;
    end;
  end;
end;

{************************************************************************}
{*                    640x480x2 MCGA mode routines                      *}
{************************************************************************}

procedure InitMCGA640;
begin
  if DontClearGraphMemory then
    CallInt10($91)
  else
    CallInt10($11);
  VideoOfs := 0;
  CurrentCGABorder := 0; {yes, TP7 CGA.BGI behaves *exactly* like that}
end;

procedure SetBkColorMCGA640(ColorNum: Word);
begin
  if ColorNum > 15 then
    exit;
  CurrentCGABorder := (CurrentCGABorder and 16) or ColorNum;
  SetCGABorder(CurrentCGABorder);
end;

function GetBkColorMCGA640: Word;
begin
  GetBkColorMCGA640 := CurrentCGABorder and 15;
end;

procedure PutPixelMCGA640(X, Y: SmallInt; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  { convert to absolute coordinates and then verify clipping...}
  if ClipPixels then
  begin
    if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
      exit;
    if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
      exit;
  end;
  Offset := Y * 80 + (X shr 3);
  Shift := 7 - (X and 7);
  Mask := 1 shl Shift;
  B := Mem[SegA000:Offset];
  B := B and (not Mask) or (Pixel shl Shift);
  Mem[SegA000:Offset] := B;
end;

function GetPixelMCGA640(X, Y: SmallInt): Word; {$ifndef fpc}far;{$endif fpc}
var
  Offset: Word;
  B, Shift: Byte;
begin
  X:= X + StartXViewPort;
  Y:= Y + StartYViewPort;
  Offset := Y * 80 + (X shr 3);
  Shift := 7 - (X and 7);
  B := Mem[SegA000:Offset];
  GetPixelMCGA640 := (B shr Shift) and 1;
end;

procedure DirectPutPixelMCGA640(X, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
var
  Offset: Word;
  B, Mask, Shift: Byte;
begin
  Offset := Y * 80 + (X shr 3);
  Shift := 7 - (X and 7);
  case CurrentWriteMode of
    XORPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegA000:Offset] := Mem[SegA000:Offset] xor (CurrentColor shl Shift);
      end;
    OrPut:
      begin
        { optimization }
        if CurrentColor = 0 then
          exit;
        Mem[SegA000:Offset] := Mem[SegA000:Offset] or (CurrentColor shl Shift);
      end;
    AndPut:
      begin
        { optimization }
        if CurrentColor = 1 then
          exit;
        { therefore, CurrentColor must be 0 }
        Mem[SegA000:Offset] := Mem[SegA000:Offset] and (not (1 shl Shift));
      end;
    NotPut:
      begin
        Mask := 1 shl Shift;
        B := Mem[SegA000:Offset];
        B := B and (not Mask) or ((CurrentColor xor $01) shl Shift);
        Mem[SegA000:Offset] := B;
      end
    else
      begin
        Mask := 1 shl Shift;
        B := Mem[SegA000:Offset];
        B := B and (not Mask) or (CurrentColor shl Shift);
        Mem[SegA000:Offset] := B;
      end;
  end;
end;

procedure HLineMCGA640(X, X2, Y: SmallInt); {$ifndef fpc}far;{$endif fpc}
var
  Color: Word;
  YOffset, LOffset, ROffset, CurrentOffset, MiddleAreaLength: Word;
  B, ForeMask, LForeMask, LBackMask, RForeMask, RBackMask: Byte;
  xtmp: SmallInt;
begin
  { must we swap the values? }
  if x > x2 then
  begin
    xtmp := x2;
    x2 := x;
    x:= xtmp;
  end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  X2  := X2 + StartXViewPort;
  Y   := Y + StartYViewPort;
  if ClipPixels then
  begin
    if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
           StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
      exit;
  end;
  YOffset := Y * 80;
  LOffset := YOffset + (X shr 3);
  ROffset := YOffset + (X2 shr 3);

  if CurrentWriteMode = NotPut then
    Color := CurrentColor xor $01
  else
    Color := CurrentColor;
  if Color = 1 then
    ForeMask := $FF
  else
    ForeMask := $00;

  LBackMask := Byte($FF00 shr (X and $07));
  LForeMask := (not LBackMask) and ForeMask;

  RBackMask := Byte(not ($FF shl (7 - (X2 and $07))));
  RForeMask := (not RBackMask) and ForeMask;

  if LOffset = ROffset then
  begin
    LBackMask := LBackMask or RBackMask;
    LForeMask := LForeMask and RForeMask;
  end;

  CurrentOffset := LOffset;

  { check if the first byte is only partially full
    (otherwise, it's completely full and is handled as a part of the middle area) }
  if LBackMask <> 0 then
  begin
    { draw the first byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] xor LForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] or LForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] and LBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegA000:CurrentOffset];
          B := B and LBackMask or LForeMask;
          Mem[SegA000:CurrentOffset] := B;
        end;
    end;
    Inc(CurrentOffset);
  end;

  if CurrentOffset > ROffset then
    exit;

  MiddleAreaLength := ROffset + 1 - CurrentOffset;
  if RBackMask <> 0 then
    Dec(MiddleAreaLength);

  { draw the middle area }
  if MiddleAreaLength > 0 then
  begin
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] xor $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := $FF;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := 0;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
      else
        begin
          { note: NotPut is also handled here }
          while MiddleAreaLength > 0 do
          begin
            Mem[SegA000:CurrentOffset] := ForeMask;
            Inc(CurrentOffset);
            Dec(MiddleAreaLength);
          end;
        end;
    end;
  end;

  if RBackMask <> 0 then
  begin
    { draw the last byte }
    case CurrentWriteMode of
      XORPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] xor RForeMask;
        end;
      OrPut:
        begin
          { optimization }
          if CurrentColor = 0 then
            exit;
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] or RForeMask;
        end;
      AndPut:
        begin
          { optimization }
          if CurrentColor = 1 then
            exit;
          { therefore, CurrentColor must be 0 }
          Mem[SegA000:CurrentOffset] := Mem[SegA000:CurrentOffset] and RBackMask;
        end;
      else
        begin
          { note: NotPut is also handled here }
          B := Mem[SegA000:CurrentOffset];
          B := B and RBackMask or RForeMask;
          Mem[SegA000:CurrentOffset] := B;
        end;
    end;
  end;
end;

 {************************************************************************}
 {*                     4-bit planar VGA mode routines                   *}
 {************************************************************************}

  Procedure Init640x200x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($8e)
      else
<<<<<<< HEAD
        CallInt10($e);
=======
        CallInt10($13);
>>>>>>> origin/cpstrnew
      VideoOfs := 0;
    end;


   Procedure Init640x350x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($90)
      else
        CallInt10($10);
      VideoOfs := 0;
    end;



  Procedure Init640x480x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($92)
      else
        CallInt10($12);
      VideoOfs := 0;
    end;




 Procedure PutPixel16(X,Y : smallint; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
     dummy: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
{$ifndef asmgraph}
     offset := y * 80 + (x shr 3) + VideoOfs;
     PortW[$3ce] := $0f01;       { Index 01 : Enable ops on all 4 planes }
     PortW[$3ce] := (Pixel and $ff) shl 8; { Index 00 : Enable correct plane and write color }

     PortW[$3ce] := ($8000 shr (x and $7)) or 8; { Select correct bits to modify }
     dummy := Mem[SegA000: offset];  { Latch the data into host space.  }
     Mem[Sega000: offset] := dummy;  { Write the data into video memory }
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
{$else asmgraph}
      asm
 {$ifndef fpc}
        mov  es, [SegA000]
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        mov  ax,[X]
        shr  ax,3
        push ax
        { determine the address }
        mov  ax,80
        mov  bx,[Y]
        mul  bx
        pop  cx
        add  ax,cx
        mov  di,ax
        add  di, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,es:[di]
        mov  es:[di],bl

<<<<<<< HEAD
        { reset for formal vga operation }
        mov  dx,3ceh
        mov  ax,0ff08h
        out  dx,ax

        { restore enable set/reset register }
        mov  ax,0001h
        out  dx,ax
 {$else fpc}
        push eax
        push ebx
        push ecx
        push edx
        push edi
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        movzx eax,[X]
        shr  eax,3
        push eax
        { determine the address }
        mov  eax,80
        mov  bx,[Y]
        mul  bx
        pop  ecx
        add  eax,ecx
        mov  edi,eax
        add  edi, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,fs:[edi+$a0000]
        mov  fs:[edi+$a0000],bl

        { reset for formal vga operation }
        mov  dx,3ceh
        mov  ax,0ff08h
        out  dx,ax

        { restore enable set/reset register }
        mov  ax,0001h
        out  dx,ax
        pop edi
        pop edx
        pop ecx
        pop ebx
        pop eax
=======
 Function GetPixel320(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
  {$ifndef fpc}
  Begin
   X:= X + StartXViewPort;
   Y:= Y + StartYViewPort;
   asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      xor    ax, ax
      add    di, [VideoOfs]   { point to correct gfx page ... }
      mov    al,es:[di]
      mov    @Result,ax
    end;
  {$else fpc}
  assembler;
  asm
    push ebx
    push ecx
    push edx
    push edi
{$IFDEF REGCALL}
    movsx  edi, ax
    movsx  ebx, dx
{$ELSE REGCALL}
    movsx  edi, x
    movsx  ebx, y
{$ENDIF REGCALL}
    movsx  ecx, StartYViewPort
    movsx  edx, StartXViewPort
    add    ebx, ecx
    add    edi, edx
 {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
    shl    ebx, 6
    add    edi, ebx
    movzx  eax, byte ptr fs:[edi+ebx*4+$a0000]
    pop edi
    pop edx
    pop ecx
    pop ebx
>>>>>>> origin/cpstrnew
 {$endif fpc}
      end;
{$endif asmgraph}
   end;


 Function GetPixel16(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 Var dummy, offset: Word;
     shift: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
{$ifndef asmgraph}
    offset := Y * 80 + (x shr 3) + VideoOfs;
    PortW[$3ce] := $0004;
    shift := 7 - (X and 7);
    dummy := (Mem[Sega000:offset] shr shift) and 1;
    Port[$3cf] := 1;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 1);
    Port[$3cf] := 2;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 2);
    Port[$3cf] := 3;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 3);
    GetPixel16 := dummy;
{$else asmgraph}
    asm
  {$ifndef fpc}
      mov   ax, [X]          { Get X address                    }
      push  ax
      shr   ax, 3
      push  ax

      mov   ax,80
      mov   bx,[Y]
      mul   bx
      pop   cx
      add   ax,cx
      mov   si,ax            { SI = correct offset into video segment }

      mov   es,[SegA000]
      add   si,[VideoOfs]    { Point to correct page offset... }

      mov   dx,03ceh
      mov   ax,4
      out   dx,al
      inc   dx

      pop   ax
      and   ax,0007h
      mov   cl,07
      sub   cl,al
      mov   bl,cl

      { read plane 0 }
      mov   al,0             { Select plane to read }
      out   dx,al
      mov   al,es:[si]       { read display memory }
      shr   al,cl
      and   al,01h
      mov   ah,al            { save bit in AH       }

      { read plane 1 }
      mov   al,1             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,1
      or    ah,al            { save bit in AH      }

      { read plane 2 }
      mov   al,2             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,2
      or    ah,al            { save bit in AH       }

      { read plane 3 }
      mov   al,3             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,3
      or    ah,al            { save bit in AH       }

      mov   al,ah            { 16-bit pixel in AX   }
      xor   ah,ah
      mov   @Result, ax
  {$else fpc}
      push eax
      push ebx
      push ecx
      push edx
      push esi
      movzx eax, [X]          { Get X address                    }
      push  eax
      shr   eax, 3
      push  eax

      mov   eax,80
      mov   bx,[Y]
      mul   bx
      pop   ecx
      add   eax,ecx
      mov   esi,eax            { SI = correct offset into video segment }

      add   esi,[VideoOfs]    { Point to correct page offset... }

      mov   dx,03ceh
      mov   ax,4
      out   dx,al
      inc   dx

      pop   eax
      and   eax,0007h
      mov   cl,07
      sub   cl,al
      mov   bl,cl

      { read plane 0 }
      mov   al,0             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]       { read display memory }
      shr   al,cl
      and   al,01h
      mov   ah,al            { save bit in AH       }

      { read plane 1 }
      mov   al,1             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,1
      or    ah,al            { save bit in AH      }

      { read plane 2 }
      mov   al,2             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,2
      or    ah,al            { save bit in AH       }

      { read plane 3 }
      mov   al,3             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,3
      or    ah,al            { save bit in AH       }

      mov   al,ah            { 16-bit pixel in AX   }
      xor   ah,ah
      mov   @Result, ax
      pop esi
      pop edx
      pop ecx
      pop ebx
      pop eax
  {$endif fpc}
    end;
{$endif asmgraph}
  end;

Procedure GetScanLine16(x1, x2, y: smallint; var data);

var dummylong: longint;
    Offset, count, count2, amount, index: word;
    plane: byte;
Begin
  inc(x1,StartXViewPort);
  inc(x2,StartXViewPort);
{$ifdef logging}
  LogLn('GetScanLine16 start, length to get: '+strf(x2-x1+1)+' at y = '+strf(y));
{$Endif logging}
  offset := (Y + StartYViewPort) * 80 + (x1 shr 3) + VideoOfs;
{$ifdef logging}
  LogLn('Offset: '+HexStr(offset,4)+' - ' + strf(offset));
{$Endif logging}
  { first get enough pixels so offset is 32bit aligned }
  amount := 0;
  index := 0;
  If ((x1 and 31) <> 0) Or
     ((x2-x1+1) < 32) Then
    Begin
      If ((x2-x1+1) >= 32+32-(x1 and 31)) Then
        amount := 32-(x1 and 31)
      Else amount := x2-x1+1;
{$ifdef logging}
      LogLn('amount to align to 32bits or to get all: ' + strf(amount));
{$Endif logging}
      For count := 0 to amount-1 do
        WordArray(Data)[Count] := getpixel16(x1-StartXViewPort+Count,y);
      index := amount;
      Inc(Offset,(amount+7) shr 3);
{$ifdef logging}
      LogLn('offset now: '+HexStr(offset,4)+' - ' + strf(offset));
      LogLn('index now: '+strf(index));
{$Endif logging}
    End;
  amount := x2-x1+1 - amount;
{$ifdef logging}
  LogLn('amount left: ' + strf(amount));
{$Endif logging}
  If amount = 0 Then Exit;
  { first get everything from plane 3 (4th plane) }
  PortW[$3ce] := $0304;
  Count := 0;
  For Count := 1 to (amount shr 5) Do
    Begin
      dummylong := MemL[SegA000:offset+(Count-1)*4];
      dummylong :=
        ((dummylong and $ff) shl 24) or
        ((dummylong and $ff00) shl 8) or
        ((dummylong and $ff0000) shr 8) or
        ((dummylong and $ff000000) shr 24);
      For Count2 := 31 downto 0 Do
        Begin
          WordArray(Data)[index+Count2] := DummyLong and 1;
          DummyLong := DummyLong shr 1;
        End;
      Inc(Index, 32);
    End;
{ Now get the data from the 3 other planes }
  plane := 3;
  Repeat
    Dec(Index,Count*32);
    Dec(plane);
    Port[$3cf] := plane;
    Count := 0;
    For Count := 1 to (amount shr 5) Do
      Begin
        dummylong := MemL[SegA000:offset+(Count-1)*4];
        dummylong :=
          ((dummylong and $ff) shl 24) or
          ((dummylong and $ff00) shl 8) or
          ((dummylong and $ff0000) shr 8) or
          ((dummylong and $ff000000) shr 24);
        For Count2 := 31 downto 0 Do
          Begin
            WordArray(Data)[index+Count2] :=
              (WordArray(Data)[index+Count2] shl 1) or (DummyLong and 1);
            DummyLong := DummyLong shr 1;
          End;
        Inc(Index, 32);
      End;
  Until plane = 0;
  amount := amount and 31;
  Dec(index);
{$ifdef Logging}
  LogLn('Last array index written to: '+strf(index));
  LogLn('amount left: '+strf(amount)+' starting at x = '+strf(index+1));
{$Endif logging}
  dec(x1,startXViewPort);
  For Count := 1 to amount Do
    WordArray(Data)[index+Count] := getpixel16(x1+index+Count,y);
{$ifdef logging}
  inc(x1,startXViewPort);
  LogLn('First 32 bytes gotten with getscanline16: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanline16: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  GetScanLineDefault(x1-StartXViewPort,x2-StartXViewPort,y,Data);
  LogLn('First 32 bytes gotten with getscanlinedef: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanlinedef: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  LogLn('GetScanLine16 end');
{$Endif logging}
End;

 Procedure DirectPutPixel16(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
  var
   color: word;
{$ifndef asmgraph}
  offset: word;
  dummy: byte;
{$endif asmgraph}
 begin
    If CurrentWriteMode <> NotPut Then
      Color := CurrentColor
    else Color := not CurrentColor;

    case CurrentWriteMode of
       XORPut:
         PortW[$3ce]:=((3 shl 3) shl 8) or 3;
       ANDPut:
         PortW[$3ce]:=((1 shl 3) shl 8) or 3;
       ORPut:
         PortW[$3ce]:=((2 shl 3) shl 8) or 3;
       {not needed, this is the default state (e.g. PutPixel16 requires it)}
       {NormalPut, NotPut:
         PortW[$3ce]:=$0003
       else
         PortW[$3ce]:=$0003}
    end;
{$ifndef asmgraph}
    offset := Y * 80 + (X shr 3) + VideoOfs;
    PortW[$3ce] := $f01;
    PortW[$3ce] := Color shl 8;
    PortW[$3ce] := ($8000 shr (X and 7)) or 8;
    dummy := Mem[SegA000: offset];
    Mem[Sega000: offset] := dummy;
    PortW[$3ce] := $ff08;
    PortW[$3ce] := $0001;
    if (CurrentWriteMode = XORPut) or
       (CurrentWriteMode = ANDPut) or
       (CurrentWriteMode = ORPut) then
      PortW[$3ce] := $0003;
{$else asmgraph}
{ note: still needs xor/or/and/notput support !!!!! (JM) }
    asm
  {$ifndef fpc}
      mov  es, [SegA000]
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      mov  ax,[X]
      shr  ax,3
      push ax
      { determine the address }
      mov  ax,80
      mov  bx,[Y]
      mul  bx
      pop  cx
      add  ax,cx
      mov  di,ax
      { send the data through the display memory through set/reset }
      add  di,[VideoOfs]   { add correct page }
      mov  bl,es:[di]
      mov  es:[di],bl

      { reset for formal vga operation }
      mov  dx,3ceh
      mov  ax,0ff08h
      out  dx,ax

      { restore enable set/reset register }
      mov  ax,0001h
      out  dx,ax
  {$else fpc}
      push eax
      push ebx
      push ecx
      push edx
      push edi
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      movzx eax,[X]
      shr  eax,3
      push eax
      { determine the address }
      mov  eax,80
      mov  bx,[Y]
      mul  bx
      pop  ecx
      add  eax,ecx
      mov  edi,eax
      { send the data through the display memory through set/reset }
      add  edi,[VideoOfs]   { add correct page }
      mov  bl,fs:[edi+$a0000]
      mov  fs:[edi+$a0000],bl

      { reset for formal vga operation }
      mov  dx,3ceh
      mov  ax,0ff08h
      out  dx,ax

      { restore enable set/reset register }
      mov  ax,0001h
      out  dx,ax
      pop edi
      pop edx
      pop ecx
      pop ebx
      pop eax
  {$endif fpc}
    end;
{$endif asmgraph}
 end;


  procedure HLine16(x,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}

   var
      xtmp: smallint;
      ScrOfs,HLength : word;
      LMask,RMask : byte;

   Begin

    { must we swap the values? }
    if x > x2 then
      Begin
        xtmp := x2;
        x2 := x;
        x:= xtmp;
      end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    X2  := X2 + StartXViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8 + VideoOfs;
    HLength:=x2 div 8-x div 8;
    LMask:=$ff shr (x and 7);
{$push}
{$r-}
{$q-}
    RMask:=$ff shl (7-(x2 and 7));
{$pop}
    if HLength=0 then
      LMask:=LMask and RMask;
    If CurrentWriteMode <> NotPut Then
      PortW[$3ce]:= CurrentColor shl 8
    else PortW[$3ce]:= (not CurrentColor) shl 8;
    PortW[$3ce]:=$0f01;
    case CurrentWriteMode of
       XORPut:
         PortW[$3ce]:=((3 shl 3) shl 8) or 3;
       ANDPut:
         PortW[$3ce]:=((1 shl 3) shl 8) or 3;
       ORPut:
         PortW[$3ce]:=((2 shl 3) shl 8) or 3;
       NormalPut, NotPut:
         PortW[$3ce]:=$0003
       else
         PortW[$3ce]:=$0003
    end;

    PortW[$3ce]:=(LMask shl 8) or 8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$ifopt r+}
{$define rangeOn}
>>>>>>> graemeg/cpstrnew
{$r-}
{$q-}
    Mem[SegA000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
<<<<<<< HEAD
{$pop}
=======
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    {Port[$3ce]:=8;}{not needed, the register is already selected}
    if HLength>0 then
      begin
         dec(HLength);
         inc(ScrOfs);
         if HLength>0 then
           begin
              Port[$3cf]:=$ff;
{$ifndef tp}
              seg_bytemove(dosmemselector,$a0000+ScrOfs,dosmemselector,$a0000+ScrOfs,HLength);
{$else}
              move(Ptr(SegA000,ScrOfs)^, Ptr(SegA000,ScrOfs)^, HLength);
{$endif}
              ScrOfs:=ScrOfs+HLength;
           end;
         Port[$3cf]:=RMask;
{$push}
{$r-}
{$q-}
         Mem[Sega000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
{$pop}
      end;
    { clean up }
    {Port[$3cf]:=0;}{not needed, the register is reset by the next operation:}
    PortW[$3ce]:=$ff08;
    PortW[$3ce]:=$0001;
    PortW[$3ce]:=$0003;
   end;

  procedure VLine16(x,y,y2: smallint); {$ifndef fpc}far;{$endif fpc}

   var
     ytmp: smallint;
     ScrOfs,i : longint;
     BitMask : byte;

  Begin
    { must we swap the values? }
    if y > y2 then
     Begin
       ytmp := y2;
       y2 := y;
       y:= ytmp;
     end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    Y2  := Y2 + StartYViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8 + VideoOfs;
    BitMask:=$80 shr (x and 7);
    If CurrentWriteMode <> NotPut Then
      PortW[$3ce]:= (CurrentColor shl 8)
    else PortW[$3ce]:= (not CurrentColor) shl 8;
    PortW[$3ce]:=$0f01;
    PortW[$3ce]:=(BitMask shl 8) or 8;
    case CurrentWriteMode of
       XORPut:
         PortW[$3ce]:=((3 shl 3) shl 8) or 3;
       ANDPut:
         PortW[$3ce]:=((1 shl 3) shl 8) or 3;
       ORPut:
         PortW[$3ce]:=((2 shl 3) shl 8) or 3;
       NormalPut, NotPut:
         PortW[$3ce]:=$0003
       else
         PortW[$3ce]:=$0003
    end;
    for i:=y to y2 do
      begin
{$push}
{$r-}
{$q-}
         Mem[SegA000:ScrOfs]:=Mem[Sega000:ScrOfs]+1;
{$pop}
         ScrOfs:=ScrOfs+ScrWidth;
      end;
    { clean up }
    {Port[$3cf]:=0;}{not needed, the register is reset by the next operation}
    PortW[$3ce]:=$ff08;
    PortW[$3ce]:=$0001;
    PortW[$3ce]:=$0003;
  End;


 procedure SetVisual200(page: word); {$ifndef fpc}far;{$endif fpc}
  { four page support... }
  begin
    if page > HardwarePages then exit;
    asm
      mov ax,[page]    { only lower byte is supPorted. }
      mov ah,05h
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}

      { read start address }
      mov dx,3d4h
      mov al,0ch
      out dx,al
      inc dx
      in  al,dx
      mov ah,al
      dec dx
      mov al,0dh
      out dx,al
      in  al,dx
    end ['EDX','EAX'];
  end;

 procedure SetActive200(page: word); {$ifndef fpc}far;{$endif fpc}
  { four page support... }
  begin
    case page of
     0 : VideoOfs := 0;
     1 : VideoOfs := 16384;
     2 : VideoOfs := 32768;
     3 : VideoOfs := 49152;
    else
      VideoOfs := 0;
    end;
  end;

 procedure SetVisual350(page: word); {$ifndef fpc}far;{$endif fpc}
  { one page supPort... }
  begin
    if page > HardwarePages then exit;
    asm
      mov ax,[page]    { only lower byte is supPorted. }
      mov ah,05h
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
    end ['EAX'];
  end;

 procedure SetActive350(page: word); {$ifndef fpc}far;{$endif fpc}
  { one page supPort... }
  begin
    case page of
     0 : VideoOfs := 0;
     1 : VideoOfs := 32768;
    else
      VideoOfs := 0;
    end;
  end;





 {************************************************************************}
 {*                     320x200x256c Routines                            *}
 {************************************************************************}

 Procedure Init320; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($93)
      else
        CallInt10($13);
      VideoOfs := 0;
    end;



 Procedure PutPixel320(X,Y : smallint; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in local coordinates. Clipping if required. }
  {$ifndef fpc}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
    asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      add    di, [VideoOfs]    { point to correct page.. }
      mov    ax, [Pixel]
      mov    es:[di], al
    end;
  {$else fpc}
  assembler;
  asm
      push eax
      push ebx
      push ecx
      push edi
{$IFDEF REGCALL}
      movsx  edi, ax
      movsx  ebx, dx
      mov    al, cl
{$ELSE REGCALL}
      movsx  edi, x
      movsx  ebx, y
{$ENDIF REGCALL}
      cmp    clippixels, 0
      je     @putpix320noclip
      test   edi, edi
      jl     @putpix320done
      test   ebx, ebx
      jl     @putpix320done
      cmp    di, ViewWidth
      jg     @putpix320done
      cmp    bx, ViewHeight
      jg     @putpix320done
@putpix320noclip:
      movsx  ecx, StartYViewPort
      movsx  edx, StartXViewPort
      add    ebx, ecx
      add    edi, edx
{    add    edi, [VideoOfs]      no multiple pages in 320*200*256 }
{$IFNDEF REGCALL}
      mov    ax, [pixel]
{$ENDIF REGCALL}
      shl    ebx, 6
      add    edi, ebx
      mov    fs:[edi+ebx*4+$a0000], al
@putpix320done:
      pop edi
      pop ecx
      pop ebx
      pop eax
{$endif fpc}
 end;


 Function GetPixel320(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
  {$ifndef fpc}
  Begin
   X:= X + StartXViewPort;
   Y:= Y + StartYViewPort;
   asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      xor    ax, ax
      add    di, [VideoOfs]   { point to correct gfx page ... }
      mov    al,es:[di]
      mov    @Result,ax
    end;
  {$else fpc}
  assembler;
  asm
    push ebx
    push ecx
    push edx
    push edi
{$IFDEF REGCALL}
    movsx  edi, ax
    movsx  ebx, dx
{$ELSE REGCALL}
    movsx  edi, x
    movsx  ebx, y
{$ENDIF REGCALL}
    movsx  ecx, StartYViewPort
    movsx  edx, StartXViewPort
    add    ebx, ecx
    add    edi, edx
 {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
    shl    ebx, 6
    add    edi, ebx
    movzx  eax, byte ptr fs:[edi+ebx*4+$a0000]
    pop edi
    pop edx
    pop ecx
    pop ebx
 {$endif fpc}
  end;


 Procedure DirectPutPixel320(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
{$ifndef asmgraph}
 var offset: word;
     dummy: Byte;
 begin
   dummy := CurrentColor;
   offset := y * 320 + x + VideoOfs;
   case CurrentWriteMode of
     XorPut: dummy := dummy xor Mem[Sega000:offset];
     OrPut: dummy := dummy or Mem[Sega000:offset];
     AndPut: dummy := dummy and Mem[SegA000:offset];
     NotPut: dummy := Not dummy;
   end;
   Mem[SegA000:offset] := dummy;
 end;
{$else asmgraph}
{ note: still needs or/and/notput support !!!!! (JM) }
  assembler;
    asm
  {$ifndef fpc}
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
{      add    di, [VideoOfs] no multiple pages support in 320*200*256 }
      mov    ax, [CurrentColor]
      cmp    [CurrentWriteMode],XORPut   { check write mode   }
      jne    @MOVMode
      mov    ah,es:[di]        { read the byte...             }
      xor    al,ah             { xor it and return value into AL }
    @MovMode:
      mov    es:[di], al
  {$else fpc}
      push eax
      push ebx
      push edi
{$IFDEF REGCALL}
      movzx  edi, ax
      movzx  ebx, dx
{$ELSE REGCALL}
      movzx  edi, x
      movzx  ebx, y
{$ENDIF REGCALL}
   {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
      shl    ebx, 6
      add    edi, ebx
      mov    ax, [CurrentColor]
      cmp    [CurrentWriteMode],XORPut   { check write mode   }
      jne    @MOVMode
      xor    al, fs:[edi+ebx*4+$a0000]
     @MovMode:
      mov    fs:[edi+ebx*4+$a0000], al
      pop edi
      pop ebx
      pop eax
{$endif fpc}
  end;
{$endif asmgraph}


 procedure SetVisual320(page: word); {$ifndef fpc}far;{$endif fpc}
  { no page supPort... }
  begin
    VideoOfs := 0;
  end;

 procedure SetActive320(page: word); {$ifndef fpc}far;{$endif fpc}
  { no page supPort... }
  begin
    VideoOfs := 0;
  end;

 {************************************************************************}
 {*                       Mode-X related routines                        *}
 {************************************************************************}
const CrtAddress: word = 0;

 procedure InitModeX; {$ifndef fpc}far;{$endif fpc}
  begin
   asm
     {see if we are using color-/monochorme display}
     MOV DX,3CCh  {use output register:     }
     IN AL,DX
     TEST AL,1    {is it a color display?    }
     MOV DX,3D4h
     JNZ @L1      {yes  }
     MOV DX,3B4h  {no  }
  @L1:          {DX = 3B4h / 3D4h = CRTAddress-register for monochrome/color}
     MOV CRTAddress,DX

     MOV  AX, 0013h
     MOV  BL, DontClearGraphMemory
     OR   BL,BL
     JZ   @L2
     OR   AX, 080h
  @L2:
{$ifdef fpc}
     push ebp
     push esi
     push edi
     push ebx
{$EndIf fpc}
     INT  10h
{$ifdef fpc}
     pop ebx
     pop edi
     pop esi
     pop ebp
{$EndIf fpc}
     MOV DX,03C4h   {select memory-mode-register at sequencer Port    }
     MOV AL,04
     OUT DX,AL
     INC DX         {read in data via the according data register     }
     IN  AL,DX
     AND AL,0F7h    {bit 3 := 0: don't chain the 4 planes}
     OR  AL,04      {bit 2 := 1: no odd/even mechanism }
     OUT DX,AL      {activate new settings    }
     MOV DX,03C4h   {s.a.: address sequencer reg. 2 (=map-mask),...   }
     MOV AL,02
     OUT DX,AL
     INC DX
     MOV AL,0Fh     {...and allow access to all 4 bit maps            }
     OUT DX,AL
{$ifndef fpc}
     MOV AX,[SegA000]  {starting with segment A000h, set 8000h logical     }
     MOV ES,AX      {words = 4*8000h physical words (because of 4     }
     XOR DI,DI      {bitplanes) to 0                                  }
     XOR AX,AX
     MOV CX,8000h
     CLD
     REP STOSW
{$else fpc}
     push eax
     push ecx
     push es
     push edi
     push fs
     mov edi, $a0000
     pop es
     xor eax, eax
     mov ecx, 4000h
     cld
     rep stosd
     pop edi
     pop es
     pop ecx
     pop eax
{$EndIf fpc}
     MOV DX,CRTAddress  {address the underline-location-register at }
     MOV AL,14h         {the CRT-controller Port, read out the according      }
     OUT DX,AL          {data register:                            }
     INC DX
     IN  AL,DX
     AND AL,0BFh    {bit 6:=0: no double word addressing scheme in}
     OUT DX,AL      {video RAM                              }
     DEC DX
     MOV AL,17h     {select mode control register     }
     OUT DX,AL
     INC DX
     IN  AL,DX
     OR  AL,40h     {bit 6 := 1: memory access scheme=linear bit array      }
     OUT DX,AL
  end ['EDX','EBX','EAX'];
 end;


 Function GetPixelX(X,Y: smallint): word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
{$endif asmgraph}
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort;
{$ifndef asmgraph}
     offset := y * 80 + x shr 2 + VideoOfs;
     PortW[$3ce] := ((x and 3) shl 8) + 4;
     GetPixelX := Mem[SegA000:offset];
{$else asmgraph}
    asm
  {$ifndef fpc}
     mov di,[Y]                   ; (* DI = Y coordinate                 *)
     (* Multiply by 80 start *)
     mov bx, di
     shl di, 6                    ; (* Faster on 286/386/486 machines    *)
     shl bx, 4
     add di, bx                   ;  (* Multiply Value by 80             *)
     (* End multiply by 80  *)
     mov cx, [X]
     mov ax, cx
    {DI = Y * LINESIZE, BX = X, coordinates admissible}
     shr ax, 1                    ; (* Faster on 286/86 machines         *)
     shr ax, 1
     add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
     add di, [VideoOfs]  ; (* Pointing at start of Active page *)
    (* Select plane to use *)
    mov dx, 03c4h
    mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
    and cl, 03h               ; (* Get Plane Bits                   *)
    shl ah, cl                ; (* Get Plane Select Value           *)
    out dx, ax
   (* End selection of plane *)
    mov es,[SegA000]
    mov al, ES:[DI]
    xor ah, ah
    mov @Result, ax
  {$else fpc}
     push eax
     push ebx
     push ecx
     push edx
     push edi
     movzx edi,[Y]                   ; (* DI = Y coordinate                 *)
     (* Multiply by 80 start *)
     mov ebx, edi
     shl edi, 6                    ; (* Faster on 286/386/486 machines    *)
     shl ebx, 4
     add edi, ebx                   ;  (* Multiply Value by 80             *)
     (* End multiply by 80  *)
     movzx ecx, [X]
     movzx eax, [Y]
    {DI = Y * LINESIZE, BX = X, coordinates admissible}
     shr eax, 2
     add edi, eax                ; {DI = Y * LINESIZE + (X SHR 2) }
     add edi, [VideoOfs]  ; (* Pointing at start of Active page *)
    (* Select plane to use *)
    mov dx, 03c4h
    mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
    and cl, 03h               ; (* Get Plane Bits                   *)
    shl ah, cl                ; (* Get Plane Select Value           *)
    out dx, ax
   (* End selection of plane *)
    mov ax, fs:[edi+$a0000]
    mov @Result, ax
    pop edi
    pop edx
    pop ecx
    pop ebx
    pop eax
  {$endif fpc}
   end;
{$endif asmgraph}
 end;

 procedure SetVisualX(page: word); {$ifndef fpc}far;{$endif fpc}
  { 4 page supPort... }

   Procedure SetVisibleStart(AOffset: word); Assembler;
   (* Select where the left corner of the screen will be *)
   { By Matt Pritchard }
    asm
     push ax
     push cx
     push dx
{$IFDEF REGCALL}
     mov cx, dx
{$ENDIF REGCALL}
      { Wait if we are currently in a Vertical Retrace        }
     MOV     DX, INPUT_1         { Input Status #1 Register       }
   @DP_WAIT0:
     IN      AL, DX              { Get VGA status                 }
     AND     AL, VERT_RETRACE    { In Display mode yet?           }
     JNZ     @DP_WAIT0           { If Not, wait for it            }

    { Set the Start Display Address to the new page         }

     MOV     DX, CRTC_Index      { We Change the VGA Sequencer    }
     MOV     AL, START_DISP_LO   { Display Start Low Register     }
{$ifndef fpc}
     MOV     AH, BYTE PTR [AOffset] { Low 8 Bits of Start Addr    }
     OUT     DX, AX              { Set Display Addr Low           }
     MOV     AL, START_DISP_HI   { Display Start High Register    }
     MOV     AH, BYTE PTR [AOffset+1] { High 8 Bits of Start Addr }
{$else fpc}
{$IFDEF REGCALL}
    mov ah, cl
{$ELSE REGCALL}
    mov ah, byte [AOffset]
{$ENDIF REGCALL}
    out dx, ax
    mov AL, START_DISP_HI
{$IFDEF REGCALL}
    mov ah, ch
{$ELSE REGCALL}
    mov ah, byte [AOffset+1]
{$ENDIF REGCALL}
{$endif fpc}
     OUT     DX, AX              { Set Display Addr High          }
     { Wait for a Vertical Retrace to smooth out things      }

     MOV     DX, INPUT_1         { Input Status #1 Register       }

  @DP_WAIT1:
     IN      AL, DX              { Get VGA status                 }
     AND     AL, VERT_RETRACE    { Vertical Retrace Start?        }
     JZ      @DP_WAIT1           { If Not, wait for it            }
    { Now Set Display Starting Address                     }
     pop dx
     pop cx
     pop ax
  end;

{$ifdef fpc}
  {$undef asmgraph}
{$endif fpc}

  begin
    Case page of
      0: SetVisibleStart(0);
      1: SetVisibleStart(16000);
      2: SetVisibleStart(32000);
      3: SetVisibleStart(48000);
    else
      SetVisibleStart(0);
    end;
  end;

 procedure SetActiveX(page: word); {$ifndef fpc}far;{$endif fpc}
  { 4 page supPort... }
  begin
   case page of
     0: VideoOfs := 0;
     1: VideoOfs := 16000;
     2: VideoOfs := 32000;
     3: VideoOfs := 48000;
   else
     VideoOfs:=0;
   end;
  end;

 Procedure PutPixelX(X,Y: smallint; color:word); {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
{$endif asmgraph}
  begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
{$ifndef asmgraph}
    offset := y * 80 + x shr 2 + VideoOfs;
    PortW[$3c4] := (hi(word(FirstPlane)) shl 8) shl (x and 3)+ lo(word(FirstPlane));
    Mem[SegA000:offset] := color;
{$else asmgraph}
     asm
      push ax
      push bx
      push cx
      push dx
      push es
      push di
      mov di,[Y]                   ; (* DI = Y coordinate                 *)
      (* Multiply by 80 start *)
      mov bx, di
      shl di, 6                    ; (* Faster on 286/386/486 machines    *)
      shl bx, 4
      add di, bx                   ;  (* Multiply Value by 80             *)
      (* End multiply by 80  *)
      mov cx, [X]
      mov ax, cx
      {DI = Y * LINESIZE, BX = X, coordinates admissible}
      shr ax, 2
      add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
      add di, [VideoOfs]        ; (* Pointing at start of Active page *)
      (* Select plane to use *)
      mov dx, 03c4h
      mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
      and cl, 03h               ; (* Get Plane Bits                   *)
      shl ah, cl                ; (* Get Plane Select Value           *)
      out dx, ax
      (* End selection of plane *)
      mov es,[SegA000]
      mov ax,[Color]            ; { only lower byte is used. }
      cmp [CurrentWriteMode],XORPut   { check write mode   }
      jne @MOVMode
      mov ah,es:[di]        { read the byte...             }
      xor al,ah             { xor it and return value into AL }
    @MovMode:
      mov es:[di], al
      pop di
      pop es
      pop dx
      pop cx
      pop bx
      pop ax
    end;
{$endif asmgraph}
  end;


 Procedure DirectPutPixelX(X,Y: smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
{$ifndef asmgraph}
 Var offset: Word;
     dummy: Byte;
 begin
   offset := y * 80 + x shr 2 + VideoOfs;
   case CurrentWriteMode of
     XorPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor xor Mem[Sega000: offset];
       end;
     OrPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor or Mem[Sega000: offset];
       end;
     AndPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor and Mem[Sega000: offset];
       end;
     NotPut: dummy := Not CurrentColor;
     else dummy := CurrentColor;
   end;
   PortW[$3c4] := (hi(word(FirstPlane)) shl 8) shl (x and 3)+ lo(word(FirstPlane));
   Mem[Sega000: offset] := Dummy;
 end;
{$else asmgraph}
{ note: still needs or/and/notput support !!!!! (JM) }
 Assembler;
 asm
   push ax
   push bx
   push cx
   push dx
   push es
   push di
{$IFDEF REGCALL}
   mov cl, al
   mov di, dx
{$ELSE REGCALL}
   mov cx, [X]
   mov ax, cx
   mov di, [Y]                   ; (* DI = Y coordinate                 *)
{$ENDIF REGCALL}
 (* Multiply by 80 start *)
   mov bx, di
   shl di, 6                    ; (* Faster on 286/386/486 machines    *)
   shl bx, 4
   add di, bx                   ;  (* Multiply Value by 80             *)
 (* End multiply by 80  *)
  {DI = Y * LINESIZE, BX = X, coordinates admissible}
   shr ax, 2
   add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
   add di, [VideoOfs]        ; (* Pointing at start of Active page *)
 (* Select plane to use *)
   mov dx, 03c4h
   mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
   and cl, 03h               ; (* Get Plane Bits                   *)
   shl ah, cl                ; (* Get Plane Select Value           *)
   out dx, ax
 (* End selection of plane *)
   mov es,[SegA000]
   mov ax,[CurrentColor]     ; { only lower byte is used. }
   cmp [CurrentWriteMode],XORPut   { check write mode   }
   jne @MOVMode
   mov ah,es:[di]        { read the byte...             }
   xor al,ah             { xor it and return value into AL }
 @MovMode:
   mov es:[di], al
   pop di
   pop es
   pop dx
   pop cx
   pop bx
   pop ax
 end;
{$endif asmgraph}



 {************************************************************************}
 {*                       General routines                               *}
 {************************************************************************}
 var
  SavePtr : pointer;    { pointer to video state                 }
{  CrtSavePtr: pointer;}  { pointer to video state when CrtMode gets called }
  StateSize: word;      { size in 64 byte blocks for video state }
  VideoMode: byte;      { old video mode before graph mode       }
  SaveSupPorted : Boolean;    { Save/Restore video state supPorted? }


      {**************************************************************}
      {*                     DPMI Routines                          *}
      {**************************************************************}

{$IFDEF DPMI}
  RealStateSeg: word;    { Real segment of saved video state }

 Procedure SaveStateVGA; {$ifndef fpc}far;{$endif fpc}
 var
  PtrLong: longint;
  regs: TDPMIRegisters;
  begin
    SaveSupPorted := FALSE;
    SavePtr := nil;
    { Get the video mode }
    asm
      mov  ah,0fh
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
      mov  [VideoMode], al
    end ['EAX'];
    { saving/restoring video state screws up Windows (JM) }
    if inWindows then
      exit;
    { Prepare to save video state...}
    asm
      mov  ax, 1C00h       { get buffer size to save state }
      mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
{$ifdef fpc}
      push ebx
      push ebp
      push esi
      push edi
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop edi
      pop esi
      pop ebp
{$endif fpc}
      mov  [StateSize], bx
{$ifdef fpc}
      pop ebx
{$endif fpc}
      cmp  al,01ch
      jnz  @notok
      mov  [SaveSupPorted],TRUE
     @notok:
    end ['ECX','EAX'];
    if SaveSupPorted then
      begin

{$ifndef fpc}
        PtrLong:=GlobalDosAlloc(64*StateSize);  { values returned in 64-byte blocks }
{$else fpc}
        PtrLong:=Global_Dos_Alloc(64*StateSize);  { values returned in 64-byte blocks }
{$endif fpc}
        if PtrLong = 0 then
           RunError(203);
        SavePtr := pointer(longint(PtrLong and $0000ffff) shl 16);
{$ifndef fpc}
        { In FPC mode, we can't do anything with this (no far pointers)  }
        { However, we still need to keep it to be able to free the       }
        { memory afterwards. Since this data is not accessed in PM code, }
        { there's no need to save it in a seperate buffer (JM)           }
        if not assigned(SavePtr) then
           RunError(203);
{$endif fpc}
        RealStateSeg := word(PtrLong shr 16);
        FillChar(regs, sizeof(regs), #0);
        { call the real mode interrupt ... }
        regs.eax := $1C01;      { save the state buffer                   }
        regs.ecx := $07;        { Save DAC / Data areas / Hardware states }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        regs.eax := $1C02;      { restore the state buffer                }
        regs.ecx := $07;        { rest DAC / Data areas / Hardware states }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
      end;
  end;

 procedure RestoreStateVGA; {$ifndef fpc}far;{$endif fpc}
  var
   regs:TDPMIRegisters;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
     end ['EAX'];
     { then restore all state information }
{$ifndef fpc}
     if assigned(SavePtr) and (SaveSupPorted=TRUE) then
{$else fpc}
     { No far pointer supPort, so it's possible that that assigned(SavePtr) }
     { would return false under FPC. Just check if it's different from nil. }
     if (SavePtr <> nil) and (SaveSupPorted=TRUE) then
{$endif fpc}
      begin
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
         regs.eax := $1C02;      { restore the state buffer                }
         regs.ecx := $07;        { rest DAC / Data areas / Hardware states }
         regs.es := RealStateSeg;
         regs.ebx := 0;
         RealIntr($10,regs);
(*
{$ifndef fpc}
         if GlobalDosFree(longint(SavePtr) shr 16)<>0 then
{$else fpc}
         if Not Global_Dos_Free(longint(SavePtr) shr 16) then
{$endif fpc}
          RunError(216);

         SavePtr := nil;
*)
       end;
  end;

{$ELSE}

      {**************************************************************}
      {*                     Real mode routines                     *}
      {**************************************************************}


 Procedure SaveStateVGA; far;
  begin
    SavePtr := nil;
    SaveSupPorted := FALSE;
    { Get the video mode }
    asm
      mov  ah,0fh
      int  10h
      mov  [VideoMode], al
    end;
    { Prepare to save video state...}
    asm
      mov  ax, 1C00h       { get buffer size to save state }
      mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
      int  10h
      mov  [StateSize], bx
      cmp  al,01ch
      jnz  @notok
      mov  [SaveSupPorted],TRUE
     @notok:
    end;
    if SaveSupPorted then
      Begin
        GetMem(SavePtr, 64*StateSize); { values returned in 64-byte blocks }
        if not assigned(SavePtr) then
           RunError(203);
        asm
         mov  ax, 1C01h       { save the state buffer                   }
         mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        asm
         mov  ax, 1C02h       { save the state buffer                   }
         mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
      end;
  end;

 procedure RestoreStateVGA; far;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
      int  10h
     end;

     { then restore all state information }
     if assigned(SavePtr) and (SaveSupPorted=TRUE) then
       begin
         { restore state, according to Ralph Brown Interrupt list }
         asm
           mov  ax, 1C02h       { save the state buffer                   }
           mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
           mov  es, WORD PTR [SavePtr+2]
           mov  bx, WORD PTR [SavePtr]
           int  10h
         end;
{        done in exitproc (JM)
         FreeMem(SavePtr, 64*StateSize);}
         SavePtr := nil;
       end;
  end;
{$ENDIF DPMI}

   Procedure SetVGARGBAllPalette(const Palette:PaletteType); {$ifndef fpc}far;{$endif fpc}
    var
      c: byte;
    begin
      { wait for vertical retrace start/end}
      while (port[$3da] and $8) <> 0 do;
      while (port[$3da] and $8) = 0 do;
      If MaxColor = 16 Then
        begin
          for c := 0 to 15 do
            begin
              { translate the color number for 16 color mode }
              portb[$3c8] := toRealCols16[c];
              portb[$3c9] := palette.colors[c].red shr 2;
              portb[$3c9] := palette.colors[c].green shr 2;
              portb[$3c9] := palette.colors[c].blue shr 2;
            end
        end
      else
        begin
          portb[$3c8] := 0;
          for c := 0 to 255 do
            begin
              { no need to set port[$3c8] every time if you set the entries }
              { for successive colornumbers (JM)                            }
              portb[$3c9] := palette.colors[c].red shr 2;
              portb[$3c9] := palette.colors[c].green shr 2;
              portb[$3c9] := palette.colors[c].blue shr 2;
          end
        end;
    End;


   { VGA is never a direct color mode, so no need to check ... }
   Procedure SetVGARGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
    begin
      { translate the color number for 16 color mode }
      If MaxColor = 16 Then
        ColorNum := ToRealCols16[ColorNum];
      asm
        { on some hardware - there is a snow like effect       }
        { when changing the palette register directly          }
        { so we wait for a vertical retrace start period.      }
        push ax
        push dx
        mov dx, $03da
      @1:
        in    al, dx          { Get input status register    }
        test  al, $08         { check if in vertical retrace }
        jnz   @1              { yes, complete it             }
                              { we have to wait for the next }
                              { retrace to assure ourselves  }
                              { that we have time to complete }
                              { the DAC operation within      }
                              { the vertical retrace period   }
       @2:
        in    al, dx
        test  al, $08
        jz    @2              { repeat until vertical retrace start }

        mov dx, $03c8       { Set color register address to use }
        mov ax, [ColorNum]
        out dx, al
        inc dx              { Point to DAC registers            }
        mov ax, [RedValue]  { Get RedValue                      }
        shr ax, 2
        out dx, al
        mov ax, [GreenValue]{ Get RedValue                      }
        shr ax, 2
        out dx, al
        mov ax, [BlueValue] { Get RedValue                      }
        shr ax, 2
        out dx, al
        pop dx
        pop ax
      end
    End;


   { VGA is never a direct color mode, so no need to check ... }
  Procedure GetVGARGBPalette(ColorNum: smallint; Var
      RedValue, GreenValue, BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
   begin
     If MaxColor = 16 Then
       ColorNum := ToRealCols16[ColorNum];
     Port[$03C7] := ColorNum;
     { we must convert to lsb values... because the vga uses the 6 msb bits }
     { which is not compatible with anything.                               }
     RedValue := smallint(Port[$3C9]) shl 2;
     GreenValue := smallint(Port[$3C9]) shl 2;
     BlueValue := smallint(Port[$3C9]) shl 2;
   end;


 {************************************************************************}
 {*                       VESA related routines                          *}
 {************************************************************************}
{$I vesa.inc}

 {************************************************************************}
 {*                       General routines                               *}
 {************************************************************************}
 procedure CloseGraph;
 Begin
    If not isgraphmode then
      begin
        _graphresult := grnoinitgraph;
        exit
      end;
    if not assigned(RestoreVideoState) then
      RunError(216);
    RestoreVideoState;
    isgraphmode := false;
 end;
(*
 procedure LoadFont8x8;

   var
      r : registers;
      x,y,c : longint;
      data : array[0..127,0..7] of byte;

   begin
      r.ah:=$11;
      r.al:=$30;
      r.bh:=1;
      RealIntr($10,r);
      dosmemget(r.es,r.bp,data,sizeof(data));
      for c:=0 to 127 do
        for y:=0 to 7 do
          for x:=0 to 7 do
            if (data[c,y] and ($80 shr x))<>0 then
              DefaultFontData[chr(c),y,x]:=1
            else
              DefaultFontData[chr(c),y,x]:=0;
      { second part }
      r.ah:=$11;
      r.al:=$30;
      r.bh:=0;
      RealIntr($10,r);
      dosmemget(r.es,r.bp,data,sizeof(data));
      for c:=0 to 127 do
        for y:=0 to 7 do
          for x:=0 to 7 do
            if (data[c,y] and ($80 shr x))<>0 then
              DefaultFontData[chr(c+128),y,x]:=1
            else
              DefaultFontData[chr(c+128),y,x]:=0;
   end;
*)
  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supPorted graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }

    function Test6845(CRTCPort: Word): Boolean;
    const
      TestRegister = $0F;
    var
      OldValue, TestValue, ReadValue: Byte;
    begin
      { save the old value }
      Port[CRTCPort] := TestRegister;
      OldValue := Port[CRTCPort + 1];
      TestValue := OldValue xor $56;

      { try writing a new value to the CRTC register }
      Port[CRTCPort] := TestRegister;
      Port[CRTCPort + 1] := TestValue;

      { check if the value has been written }
      Port[CRTCPort] := TestRegister;
      ReadValue := Port[CRTCPort + 1];
      if ReadValue = TestValue then
      begin
        Test6845 := True;
        { restore old value }
        Port[CRTCPort] := TestRegister;
        Port[CRTCPort + 1] := OldValue;
      end
      else
        Test6845 := False;
    end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure FillCommonCGA320(var mode: TModeInfo);
    begin
      mode.HardwarePages := 0;
      mode.MaxColor := 4;
      mode.PaletteSize := 16;
      mode.DirectColor := FALSE;
      mode.MaxX := 319;
      mode.MaxY := 199;
      mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
      mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
      mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
      mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
      mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
      mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
      mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
      mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
      mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
      mode.XAspect := 8333;
      mode.YAspect := 10000;
    end;

    procedure FillCommonCGA640(var mode: TModeInfo);
    begin
      mode.HardwarePages := 0;
      mode.MaxColor := 2;
      mode.PaletteSize := 16;
      mode.DirectColor := FALSE;
      mode.MaxX := 639;
      mode.MaxY := 199;
      mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA640;
      mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA640;
      mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA640;
      mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
      mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
      mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
      mode.HLine := {$ifdef fpc}@{$endif}HLineCGA640;
      mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA640;
      mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA640;
      mode.XAspect := 4167;
      mode.YAspect := 10000;
    end;

    procedure FillCommonEGAVGA16(var mode: TModeInfo);
    begin
      mode.MaxColor := 16;
      mode.DirectColor := FALSE;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
      mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
      mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
      mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
      mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
      mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
      mode.HLine := {$ifdef fpc}@{$endif}HLine16;
      mode.VLine := {$ifdef fpc}@{$endif}VLine16;
      mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
    end;

    procedure FillCommonVESA16(var mode: TModeInfo);
    begin
      mode.MaxColor := 16;
      { the ModeInfo is automatically set if the mode is supPorted }
      { by the call to SearchVESAMode.                             }
      mode.HardwarePages := VESAModeInfo.NumberOfPages;
      mode.DirectColor := FALSE;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA16;
      mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
      mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
      mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
      mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA16;
      mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA16;
      mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
      mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
      mode.HLine := {$ifdef fpc}@{$endif}HLineVESA16;
    end;

    procedure FillCommonVESA256(var mode: TModeInfo);
    begin
      mode.MaxColor := 256;
      { the ModeInfo is automatically set if the mode is supPorted }
      { by the call to SearchVESAMode.                             }
      mode.HardwarePages := VESAModeInfo.NumberOfPages;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectColor := FALSE;
      mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA256;
      mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA256;
      mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA256;
      mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
      mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
      mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
      mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
      mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
      mode.hline := {$ifdef fpc}@{$endif}HLineVESA256;
      mode.vline := {$ifdef fpc}@{$endif}VLineVESA256;
      mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLineVESA256;
      mode.PatternLine := {$ifdef fpc}@{$endif}PatternLineVESA256;
    end;

    procedure FillCommonVESA32kOr64k(var mode: TModeInfo);
    begin
      { the ModeInfo is automatically set if the mode is supPorted }
      { by the call to SearchVESAMode.                             }
      mode.HardwarePages := VESAModeInfo.NumberOfPages;
      mode.DirectColor := TRUE;
      mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
      mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
      mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
      mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
      mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
      mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
      mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
      mode.HLine := {$ifdef fpc}@{$endif}HLineVESA32kOr64k;
    end;

    procedure FillCommonVESA32k(var mode: TModeInfo);
    begin
      FillCommonVESA32kOr64k(mode);
      mode.MaxColor := 32768;
      mode.PaletteSize := mode.MaxColor;
    end;
    procedure FillCommonVESA64k(var mode: TModeInfo);
    begin
      FillCommonVESA32kOr64k(mode);
      mode.MaxColor := 65536;
      mode.PaletteSize := mode.MaxColor;
    end;

   var
    HGCDetected : Boolean = FALSE;
    CGADetected : Boolean = FALSE; { TRUE means real CGA, *not* EGA or VGA }
    EGAColorDetected : Boolean = FALSE; { TRUE means true EGA with a color monitor }
    EGAMonoDetected : Boolean = FALSE; { TRUE means true EGA with a monochrome (MDA) monitor }
    MCGADetected : Boolean = FALSE;
    VGADetected : Boolean = FALSE;
=======
   var
=======
   var
>>>>>>> graemeg/cpstrnew
=======
   var
>>>>>>> graemeg/cpstrnew
    HGCDetected : Boolean;
    CGADetected : Boolean; { TRUE means real CGA, *not* EGA or VGA }
    EGADetected : Boolean; { TRUE means EGA or higher (VGA) }
    VGADetected : Boolean;
>>>>>>> graemeg/cpstrnew
    mode: TModeInfo;
    regs: TDPMIRegisters;
   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;

<<<<<<< HEAD
     { check if VGA/MCGA adapter supported...       }
     regs.ax:=$1a00;
     RealIntr($10,regs);    { get display combination code...}
     if regs.al=$1a then
       begin
         while regs.bx <> 0 do
           begin
             case regs.bl of
               1: { monochrome adapter (MDA or HGC) }
                 begin
                   { check if Hercules adapter supported ... }
                   HGCDetected:=Test6845($3B4);
                 end;
               2: CGADetected:=TRUE;
               4: EGAColorDetected:=TRUE;
               5: EGAMonoDetected:=TRUE;
               {6: PGA, this is rare stuff, how do we handle it? }
               7, 8: VGADetected:=TRUE;
               10, 11, 12: MCGADetected:=TRUE;
             end;
             { check both primary and secondary display adapter }
             regs.bx:=regs.bx shr 8;
           end;
       end;
     if VGADetected then
=======

     HGCDetected := FALSE;
     CGADetected := FALSE;
     EGADetected := FALSE;
     VGADetected := FALSE;
     { check if EGA adapter supPorted...       }
     asm
       mov ah,12h
       mov bx,0FF10h
{$ifdef fpc}
       push ebx
       push ebp
       push esi
       push edi
{$endif fpc}
       int 10h              { get EGA information }
{$ifdef fpc}
       pop edi
       pop esi
       pop ebp
{$endif fpc}
       cmp bh,0ffh
{$ifdef fpc}
       pop ebx
{$endif fpc}
       jz  @noega
       mov [EGADetected],TRUE
     @noega:
     end ['EBX','EAX'];
{$ifdef logging}
     LogLn('EGA detected: '+strf(Longint(EGADetected)));
{$endif logging}
     { check if VGA adapter supPorted...       }
     if EGADetected then
>>>>>>> graemeg/cpstrnew
       begin
         { now check if this is the ATI EGA }
         regs.ax:=$1c00; { get state size for save...     }
                         { ... all important data         }
         regs.cx:=$07;
         RealIntr($10,regs);
         VGADetected:=regs.al=$1c;
       end;
     if not VGADetected and not MCGADetected and
        not EGAColorDetected and not EGAMonoDetected and
        not CGADetected and not HGCDetected then
       begin
         { check if EGA adapter supported...       }
         regs.ah:=$12;
         regs.bx:=$FF10;
         RealIntr($10,regs);     { get EGA information }
         if regs.bh<>$FF then
           case regs.cl of
             0..3, { primary: MDA/HGC,   secondary: EGA color }
             6..9: { primary: EGA color, secondary: MDA/HGC (optional) }
               begin
                 EGAColorDetected:=TRUE;
                 { check if Hercules adapter supported ... }
                 HGCDetected:=Test6845($3B4);
               end;
             4..5, { primary: CGA,        secondary: EGA mono }
             10..11: { primary: EGA mono, secondary: CGA (optional) }
               begin
                 EGAMonoDetected:=TRUE;
                 { check if CGA adapter supported ... }
                 CGADetected := Test6845($3D4);
               end;
           end;
       end;
     { older than EGA? }
     if not VGADetected and not MCGADetected and
        not EGAColorDetected and not EGAMonoDetected and
        not CGADetected and not HGCDetected then
       begin
         { check if Hercules adapter supported ... }
         HGCDetected := Test6845($3B4);
         { check if CGA adapter supported ... }
         CGADetected := Test6845($3D4);
       end;
{$ifdef logging}
     LogLn('HGC detected: '+strf(Longint(HGCDetected)));
     LogLn('CGA detected: '+strf(Longint(CGADetected)));
     LogLn('EGA color detected: '+strf(Longint(EGAColorDetected)));
     LogLn('EGA mono detected: '+strf(Longint(EGAMonoDetected)));
     LogLn('MCGA detected: '+strf(Longint(MCGADetected)));
     LogLn('VGA detected: '+strf(Longint(VGADetected)));
{$endif logging}
<<<<<<< HEAD
=======
     { older than EGA? }
     if not EGADetected then
       begin
         { check if Hercules adapter supPorted ... }
         HGCDetected := Test6845($3B4);
         { check if CGA adapter supPorted ... }
         CGADetected := Test6845($3D4);
       end;
>>>>>>> graemeg/cpstrnew
     if HGCDetected then
       begin
         { HACK:
           until we create Save/RestoreStateHGC, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.DriverNumber := HercMono;
         mode.HardwarePages := 1;
         mode.ModeNumber := HercMonoHi;
         mode.ModeName:='720 x 348 HERCULES';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 719;
         mode.MaxY := 347;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelHGC720;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelHGC720;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelHGC720;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetHGCRGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetHGCRGBPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualHGC720;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveHGC720;
         mode.InitMode := {$ifdef fpc}@{$endif}InitHGC720;
         mode.HLine := {$ifdef fpc}@{$endif}HLineHGC720;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorHGC720;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorHGC720;
         mode.XAspect := 7500;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
<<<<<<< HEAD
     if CGADetected or EGAColorDetected or MCGADetected or VGADetected then
=======
     if CGADetected or EGADetected then
>>>>>>> graemeg/cpstrnew
       begin
         { HACK:
           until we create Save/RestoreStateCGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         { now add all standard CGA modes...       }
         InitMode(mode);
<<<<<<< HEAD
         FillCommonCGA320(mode);
         mode.DriverNumber := CGA;
         mode.ModeNumber := CGAC0;
         mode.ModeName:='320 x 200 CGA C0';
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA320(mode);
         mode.DriverNumber := CGA;
         mode.ModeNumber := CGAC1;
         mode.ModeName:='320 x 200 CGA C1';
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA320(mode);
         mode.DriverNumber := CGA;
         mode.ModeNumber := CGAC2;
         mode.ModeName:='320 x 200 CGA C2';
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA320(mode);
         mode.DriverNumber := CGA;
         mode.ModeNumber := CGAC3;
         mode.ModeName:='320 x 200 CGA C3';
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA640(mode);
         mode.DriverNumber := CGA;
         mode.ModeNumber := CGAHi;
         mode.ModeName:='640 x 200 CGA';
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
         AddMode(mode);
       end;

     if EGAColorDetected or VGADetected then
=======
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC0;
         mode.ModeName:='320 x 200 CGA C0';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC1;
         mode.ModeName:='320 x 200 CGA C1';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC2;
         mode.ModeName:='320 x 200 CGA C2';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC3;
         mode.ModeName:='320 x 200 CGA C3';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAHi;
         mode.ModeName:='640 x 200 CGA';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA640;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA640;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA640;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA640;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA640;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA640;
         mode.XAspect := 4167;
         mode.YAspect := 10000;
         AddMode(mode);
       end;

     if EGADetected then
>>>>>>> graemeg/cpstrnew
       begin
         { HACK:
           until we create Save/RestoreStateEGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
<<<<<<< HEAD
         FillCommonEGAVGA16(mode);
         mode.ModeNumber:=EGALo;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 200 EGA';
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.HardwarePages := 3;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
=======
         mode.ModeNumber:=EGALo;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 200 EGA';
         mode.MaxColor := 16;
         mode.HardwarePages := 2;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
>>>>>>> graemeg/cpstrnew
         mode.XAspect := 4500;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
<<<<<<< HEAD
         FillCommonEGAVGA16(mode);
         mode.ModeNumber:=EGAHi;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 350 EGA';
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.HardwarePages := 1;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
=======
         mode.ModeNumber:=EGAHi;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 350 EGA';
         mode.HardwarePages := 1;
         mode.MaxColor := 16;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
>>>>>>> graemeg/cpstrnew
         mode.XAspect := 7750;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
<<<<<<< HEAD

<<<<<<< HEAD
     if MCGADetected or VGADetected then
=======
=======
{$ifdef logging}
       LogLn('VGA detected: '+strf(Longint(VGADetected)));
{$endif logging}
     { older than EGA? }
     if not EGADetected then
       begin
         { check if Hercules adapter supPorted ... }
         HGCDetected := Test6845($3B4);
         { check if CGA adapter supPorted ... }
         CGADetected := Test6845($3D4);
       end;
     if HGCDetected then
       begin
         { HACK:
           until we create Save/RestoreStateHGC, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.DriverNumber := HercMono;
         mode.HardwarePages := 1;
         mode.ModeNumber := HercMonoHi;
         mode.ModeName:='720 x 348 HERCULES';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 719;
         mode.MaxY := 347;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelHGC720;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelHGC720;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelHGC720;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetHGCRGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetHGCRGBPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualHGC720;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveHGC720;
         mode.InitMode := {$ifdef fpc}@{$endif}InitHGC720;
         mode.HLine := {$ifdef fpc}@{$endif}HLineHGC720;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorHGC720;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorHGC720;
         mode.XAspect := 7500;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
     if CGADetected or EGADetected then
       begin
         { HACK:
           until we create Save/RestoreStateCGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         { now add all standard CGA modes...       }
         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC0;
         mode.ModeName:='320 x 200 CGA C0';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC1;
         mode.ModeName:='320 x 200 CGA C1';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC2;
         mode.ModeName:='320 x 200 CGA C2';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC3;
         mode.ModeName:='320 x 200 CGA C3';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAHi;
         mode.ModeName:='640 x 200 CGA';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA640;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA640;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA640;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA640;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA640;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA640;
         mode.XAspect := 4167;
         mode.YAspect := 10000;
         AddMode(mode);
       end;

     if EGADetected then
       begin
         { HACK:
           until we create Save/RestoreStateEGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.ModeNumber:=EGALo;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 200 EGA';
         mode.MaxColor := 16;
         mode.HardwarePages := 2;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
         mode.XAspect := 4500;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.ModeNumber:=EGAHi;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 350 EGA';
         mode.HardwarePages := 1;
         mode.MaxColor := 16;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
         mode.XAspect := 7750;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
<<<<<<< HEAD

>>>>>>> graemeg/cpstrnew
=======
{$ifdef logging}
       LogLn('VGA detected: '+strf(Longint(VGADetected)));
{$endif logging}
     { older than EGA? }
     if not EGADetected then
       begin
         { check if Hercules adapter supPorted ... }
         HGCDetected := Test6845($3B4);
         { check if CGA adapter supPorted ... }
         CGADetected := Test6845($3D4);
       end;
     if HGCDetected then
       begin
         { HACK:
           until we create Save/RestoreStateHGC, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.DriverNumber := HercMono;
         mode.HardwarePages := 1;
         mode.ModeNumber := HercMonoHi;
         mode.ModeName:='720 x 348 HERCULES';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 719;
         mode.MaxY := 347;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelHGC720;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelHGC720;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelHGC720;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetHGCRGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetHGCRGBPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualHGC720;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveHGC720;
         mode.InitMode := {$ifdef fpc}@{$endif}InitHGC720;
         mode.HLine := {$ifdef fpc}@{$endif}HLineHGC720;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorHGC720;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorHGC720;
         mode.XAspect := 7500;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
     if CGADetected or EGADetected then
       begin
         { HACK:
           until we create Save/RestoreStateCGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

<<<<<<< HEAD
         { now add all standard CGA modes...       }
         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC0;
         mode.ModeName:='320 x 200 CGA C0';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);
=======
=======
 {************************************************************************}
 {*                     4-bit planar VGA mode routines                   *}
 {************************************************************************}

  Procedure Init640x200x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($8e)
      else
        CallInt10($e);
    end;


   Procedure Init640x350x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($90)
      else
        CallInt10($10);
    end;



  Procedure Init640x480x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($92)
      else
        CallInt10($12);
    end;




 Procedure PutPixel16(X,Y : smallint; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
     dummy: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
{$ifndef asmgraph}
     offset := y * 80 + (x shr 3) + VideoOfs;
     PortW[$3ce] := $0f01;       { Index 01 : Enable ops on all 4 planes }
     PortW[$3ce] := (Pixel and $ff) shl 8; { Index 00 : Enable correct plane and write color }

     Port[$3ce] := 8;
     Port[$3cf] := $80 shr (x and $7); { Select correct bits to modify }
     dummy := Mem[SegA000: offset];  { Latch the data into host space.  }
     Mem[Sega000: offset] := dummy;  { Write the data into video memory }
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
{$else asmgraph}
      asm
 {$ifndef fpc}
        mov  es, [SegA000]
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        mov  ax,[X]
        shr  ax,3
        push ax
        { determine the address }
        mov  ax,80
        mov  bx,[Y]
        mul  bx
        pop  cx
        add  ax,cx
        mov  di,ax
        add  di, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,es:[di]
        mov  es:[di],bl

        { reset for formal vga operation }
        mov  dx,3ceh
        mov  ax,0ff08h
        out  dx,ax

        { restore enable set/reset register }
        mov  ax,0001h
        out  dx,ax
 {$else fpc}
        push eax
        push ebx
        push ecx
        push edx
        push edi
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        movzx eax,[X]
        shr  eax,3
        push eax
        { determine the address }
        mov  eax,80
        mov  bx,[Y]
        mul  bx
        pop  ecx
        add  eax,ecx
        mov  edi,eax
        add  edi, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,fs:[edi+$a0000]
        mov  fs:[edi+$a0000],bl

        { reset for formal vga operation }
        mov  dx,3ceh
        mov  ax,0ff08h
        out  dx,ax

        { restore enable set/reset register }
        mov  ax,0001h
        out  dx,ax
        pop edi
        pop edx
        pop ecx
        pop ebx
        pop eax
 {$endif fpc}
      end;
{$endif asmgraph}
   end;


 Function GetPixel16(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 Var dummy, offset: Word;
     shift: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
{$ifndef asmgraph}
    offset := Y * 80 + (x shr 3) + VideoOfs;
    Port[$3ce] := 4;
    shift := 7 - (X and 7);
    Port[$3cf] := 0;
    dummy := (Mem[Sega000:offset] shr shift) and 1;
    Port[$3cf] := 1;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 1);
    Port[$3cf] := 2;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 2);
    Port[$3cf] := 3;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 3);
    GetPixel16 := dummy;
{$else asmgraph}
    asm
  {$ifndef fpc}
      mov   ax, [X]          { Get X address                    }
      push  ax
      shr   ax, 3
      push  ax

      mov   ax,80
      mov   bx,[Y]
      mul   bx
      pop   cx
      add   ax,cx
      mov   si,ax            { SI = correct offset into video segment }

      mov   es,[SegA000]
      add   si,[VideoOfs]    { Point to correct page offset... }

      mov   dx,03ceh
      mov   ax,4
      out   dx,al
      inc   dx

      pop   ax
      and   ax,0007h
      mov   cl,07
      sub   cl,al
      mov   bl,cl

      { read plane 0 }
      mov   al,0             { Select plane to read }
      out   dx,al
      mov   al,es:[si]       { read display memory }
      shr   al,cl
      and   al,01h
      mov   ah,al            { save bit in AH       }

      { read plane 1 }
      mov   al,1             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,1
      or    ah,al            { save bit in AH      }

      { read plane 2 }
      mov   al,2             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,2
      or    ah,al            { save bit in AH       }

      { read plane 3 }
      mov   al,3             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,3
      or    ah,al            { save bit in AH       }

      mov   al,ah            { 16-bit pixel in AX   }
      xor   ah,ah
      mov   @Result, ax
  {$else fpc}
      push eax
      push ebx
      push ecx
      push edx
      push esi
      movzx eax, [X]          { Get X address                    }
      push  eax
      shr   eax, 3
      push  eax

      mov   eax,80
      mov   bx,[Y]
      mul   bx
      pop   ecx
      add   eax,ecx
      mov   esi,eax            { SI = correct offset into video segment }

      add   esi,[VideoOfs]    { Point to correct page offset... }

      mov   dx,03ceh
      mov   ax,4
      out   dx,al
      inc   dx

      pop   eax
      and   eax,0007h
      mov   cl,07
      sub   cl,al
      mov   bl,cl

      { read plane 0 }
      mov   al,0             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]       { read display memory }
      shr   al,cl
      and   al,01h
      mov   ah,al            { save bit in AH       }

      { read plane 1 }
      mov   al,1             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,1
      or    ah,al            { save bit in AH      }

      { read plane 2 }
      mov   al,2             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,2
      or    ah,al            { save bit in AH       }

      { read plane 3 }
      mov   al,3             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,3
      or    ah,al            { save bit in AH       }

      mov   al,ah            { 16-bit pixel in AX   }
      xor   ah,ah
      mov   @Result, ax
      pop esi
      pop edx
      pop ecx
      pop ebx
      pop eax
  {$endif fpc}
    end;
{$endif asmgraph}
  end;

Procedure GetScanLine16(x1, x2, y: smallint; var data);

var dummylong: longint;
    Offset, count, count2, amount, index: word;
    plane: byte;
Begin
  inc(x1,StartXViewPort);
  inc(x2,StartXViewPort);
{$ifdef logging}
  LogLn('GetScanLine16 start, length to get: '+strf(x2-x1+1)+' at y = '+strf(y));
{$Endif logging}
  offset := (Y + StartYViewPort) * 80 + (x1 shr 3) + VideoOfs;
{$ifdef logging}
  LogLn('Offset: '+HexStr(offset,4)+' - ' + strf(offset));
{$Endif logging}
  { first get enough pixels so offset is 32bit aligned }
  amount := 0;
  index := 0;
  If ((x1 and 31) <> 0) Or
     ((x2-x1+1) < 32) Then
    Begin
      If ((x2-x1+1) >= 32+32-(x1 and 31)) Then
        amount := 32-(x1 and 31)
      Else amount := x2-x1+1;
{$ifdef logging}
      LogLn('amount to align to 32bits or to get all: ' + strf(amount));
{$Endif logging}
      For count := 0 to amount-1 do
        WordArray(Data)[Count] := getpixel16(x1-StartXViewPort+Count,y);
      index := amount;
      Inc(Offset,(amount+7) shr 3);
{$ifdef logging}
      LogLn('offset now: '+HexStr(offset,4)+' - ' + strf(offset));
      LogLn('index now: '+strf(index));
{$Endif logging}
    End;
  amount := x2-x1+1 - amount;
{$ifdef logging}
  LogLn('amount left: ' + strf(amount));
{$Endif logging}
  If amount = 0 Then Exit;
  Port[$3ce] := 4;
  { first get everything from plane 3 (4th plane) }
  Port[$3cf] := 3;
  Count := 0;
  For Count := 1 to (amount shr 5) Do
    Begin
      dummylong := MemL[SegA000:offset+(Count-1)*4];
      dummylong :=
        ((dummylong and $ff) shl 24) or
        ((dummylong and $ff00) shl 8) or
        ((dummylong and $ff0000) shr 8) or
        ((dummylong and $ff000000) shr 24);
      For Count2 := 31 downto 0 Do
        Begin
          WordArray(Data)[index+Count2] := DummyLong and 1;
          DummyLong := DummyLong shr 1;
        End;
      Inc(Index, 32);
    End;
{ Now get the data from the 3 other planes }
  plane := 3;
  Repeat
    Dec(Index,Count*32);
    Dec(plane);
    Port[$3cf] := plane;
    Count := 0;
    For Count := 1 to (amount shr 5) Do
      Begin
        dummylong := MemL[SegA000:offset+(Count-1)*4];
        dummylong :=
          ((dummylong and $ff) shl 24) or
          ((dummylong and $ff00) shl 8) or
          ((dummylong and $ff0000) shr 8) or
          ((dummylong and $ff000000) shr 24);
        For Count2 := 31 downto 0 Do
          Begin
            WordArray(Data)[index+Count2] :=
              (WordArray(Data)[index+Count2] shl 1) or (DummyLong and 1);
            DummyLong := DummyLong shr 1;
          End;
        Inc(Index, 32);
      End;
  Until plane = 0;
  amount := amount and 31;
  Dec(index);
{$ifdef Logging}
  LogLn('Last array index written to: '+strf(index));
  LogLn('amount left: '+strf(amount)+' starting at x = '+strf(index+1));
{$Endif logging}
  dec(x1,startXViewPort);
  For Count := 1 to amount Do
    WordArray(Data)[index+Count] := getpixel16(x1+index+Count,y);
{$ifdef logging}
  inc(x1,startXViewPort);
  LogLn('First 32 bytes gotten with getscanline16: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanline16: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  GetScanLineDefault(x1-StartXViewPort,x2-StartXViewPort,y,Data);
  LogLn('First 32 bytes gotten with getscanlinedef: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanlinedef: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  LogLn('GetScanLine16 end');
{$Endif logging}
End;

 Procedure DirectPutPixel16(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
  var
   color: word;
{$ifndef asmgraph}
  offset: word;
  dummy: byte;
{$endif asmgraph}
 begin
    case CurrentWriteMode of
      XORPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Xor Color;
        end;
      OrPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Or Color;
        end;
      AndPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor And Color;
        end;
      NotPut:
        begin
          Color := (Not CurrentColor) and 15;
        end
      else
        Color := CurrentColor;
    end;
{$ifndef asmgraph}
    offset := Y * 80 + (X shr 3) + VideoOfs;
    PortW[$3ce] := $f01;
    PortW[$3ce] := Color shl 8;
    Port[$3ce] := 8;
    Port[$3cf] := $80 shr (X and 7);
    dummy := Mem[SegA000: offset];
    Mem[Sega000: offset] := dummy;
    PortW[$3ce] := $ff08;
    PortW[$3ce] := $0001;
{$else asmgraph}
{ note: still needs xor/or/and/notput support !!!!! (JM) }
    asm
  {$ifndef fpc}
      mov  es, [SegA000]
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      mov  ax,[X]
      shr  ax,3
      push ax
      { determine the address }
      mov  ax,80
      mov  bx,[Y]
      mul  bx
      pop  cx
      add  ax,cx
      mov  di,ax
      { send the data through the display memory through set/reset }
      add  di,[VideoOfs]   { add correct page }
      mov  bl,es:[di]
      mov  es:[di],bl

      { reset for formal vga operation }
      mov  dx,3ceh
      mov  ax,0ff08h
      out  dx,ax

      { restore enable set/reset register }
      mov  ax,0001h
      out  dx,ax
  {$else fpc}
      push eax
      push ebx
      push ecx
      push edx
      push edi
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      movzx eax,[X]
      shr  eax,3
      push eax
      { determine the address }
      mov  eax,80
      mov  bx,[Y]
      mul  bx
      pop  ecx
      add  eax,ecx
      mov  edi,eax
      { send the data through the display memory through set/reset }
      add  edi,[VideoOfs]   { add correct page }
      mov  bl,fs:[edi+$a0000]
      mov  fs:[edi+$a0000],bl

      { reset for formal vga operation }
      mov  dx,3ceh
      mov  ax,0ff08h
      out  dx,ax

      { restore enable set/reset register }
      mov  ax,0001h
      out  dx,ax
      pop edi
      pop edx
      pop ecx
      pop ebx
      pop eax
  {$endif fpc}
    end;
{$endif asmgraph}
 end;


  procedure HLine16(x,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}

   var
      xtmp: smallint;
      ScrOfs,HLength : word;
      LMask,RMask : byte;

   Begin

    { must we swap the values? }
    if x > x2 then
      Begin
        xtmp := x2;
        x2 := x;
        x:= xtmp;
      end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    X2  := X2 + StartXViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8;
    HLength:=x2 div 8-x div 8;
    LMask:=$ff shr (x and 7);
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    RMask:=$ff shl (7-(x2 and 7));
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    if HLength=0 then
      LMask:=LMask and RMask;
    Port[$3ce]:=0;
    If CurrentWriteMode <> NotPut Then
      Port[$3cf]:= CurrentColor
    else Port[$3cf]:= not CurrentColor;
    Port[$3ce]:=1;
    Port[$3cf]:=$f;
    Port[$3ce]:=3;
    case CurrentWriteMode of
       XORPut:
         Port[$3cf]:=3 shl 3;
       ANDPut:
         Port[$3cf]:=1 shl 3;
       ORPut:
         Port[$3cf]:=2 shl 3;
       NormalPut, NotPut:
         Port[$3cf]:=0
       else
         Port[$3cf]:=0
    end;

    Port[$3ce]:=8;
    Port[$3cf]:=LMask;
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    Mem[SegA000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    Port[$3ce]:=8;
    if HLength>0 then
      begin
         dec(HLength);
         inc(ScrOfs);
         if HLength>0 then
           begin
              Port[$3cf]:=$ff;
{$ifndef tp}
              seg_bytemove(dosmemselector,$a0000+ScrOfs,dosmemselector,$a0000+ScrOfs,HLength);
{$else}
              move(Ptr(SegA000,ScrOfs)^, Ptr(SegA000,ScrOfs)^, HLength);
{$endif}
              ScrOfs:=ScrOfs+HLength;
           end;
         Port[$3cf]:=RMask;
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
         Mem[Sega000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
      end;
    { clean up }
    Port[$3cf]:=0;
    Port[$3ce]:=8;
    Port[$3cf]:=$ff;
    Port[$3ce]:=1;
    Port[$3cf]:=0;
    Port[$3ce]:=3;
    Port[$3cf]:=0;
   end;

  procedure VLine16(x,y,y2: smallint); {$ifndef fpc}far;{$endif fpc}

   var
     ytmp: smallint;
     ScrOfs,i : longint;
     BitMask : byte;

  Begin
    { must we swap the values? }
    if y > y2 then
     Begin
       ytmp := y2;
       y2 := y;
       y:= ytmp;
     end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    Y2  := Y2 + StartYViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8;
    BitMask:=$80 shr (x and 7);
    Port[$3ce]:=0;
    If CurrentWriteMode <> NotPut Then
      Port[$3cf]:= CurrentColor
    else Port[$3cf]:= not CurrentColor;
    Port[$3ce]:=1;
    Port[$3cf]:=$f;
    Port[$3ce]:=8;
    Port[$3cf]:=BitMask;
    Port[$3ce]:=3;
    case CurrentWriteMode of
       XORPut:
         Port[$3cf]:=3 shl 3;
       ANDPut:
         Port[$3cf]:=1 shl 3;
       ORPut:
         Port[$3cf]:=2 shl 3;
       NormalPut, NotPut:
         Port[$3cf]:=0
       else
         Port[$3cf]:=0
    end;
    for i:=y to y2 do
      begin
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
         Mem[SegA000:ScrOfs]:=Mem[Sega000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
         ScrOfs:=ScrOfs+ScrWidth;
      end;
    { clean up }
    Port[$3cf]:=0;
    Port[$3ce]:=8;
    Port[$3cf]:=$ff;
    Port[$3ce]:=1;
    Port[$3cf]:=0;
    Port[$3ce]:=3;
    Port[$3cf]:=0;
  End;


 procedure SetVisual480(page: word); {$ifndef fpc}far;{$endif fpc}
 { no page flipping supPort in 640x480 mode }
  begin
    VideoOfs := 0;
  end;

 procedure SetActive480(page: word); {$ifndef fpc}far;{$endif fpc}
 { no page flipping supPort in 640x480 mode }
  begin
    VideoOfs := 0;
  end;


 procedure SetVisual200(page: word); {$ifndef fpc}far;{$endif fpc}
  { two page supPort... }
  begin
    if page > HardwarePages then exit;
    asm
      mov ax,[page]    { only lower byte is supPorted. }
      mov ah,05h
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}

      { read start address }
      mov dx,3d4h
      mov al,0ch
      out dx,al
      inc dx
      in  al,dx
      mov ah,al
      dec dx
      mov al,0dh
      out dx,al
      in  al,dx
    end ['EDX','EAX'];
  end;

 procedure SetActive200(page: word); {$ifndef fpc}far;{$endif fpc}
  { two page supPort... }
  begin
    case page of
     0 : VideoOfs := 0;
     1 : VideoOfs := 16384;
     2 : VideoOfs := 32768;
    else
      VideoOfs := 0;
    end;
  end;

 procedure SetVisual350(page: word); {$ifndef fpc}far;{$endif fpc}
  { one page supPort... }
  begin
    if page > HardwarePages then exit;
    asm
      mov ax,[page]    { only lower byte is supPorted. }
      mov ah,05h
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
    end ['EAX'];
  end;

 procedure SetActive350(page: word); {$ifndef fpc}far;{$endif fpc}
  { one page supPort... }
  begin
    case page of
     0 : VideoOfs := 0;
     1 : VideoOfs := 32768;
    else
      VideoOfs := 0;
    end;
  end;





 {************************************************************************}
 {*                     320x200x256c Routines                            *}
 {************************************************************************}

 Procedure Init320; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($93)
      else
        CallInt10($13);
    end;



 Procedure PutPixel320(X,Y : smallint; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in local coordinates. Clipping if required. }
  {$ifndef fpc}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
    asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      add    di, [VideoOfs]    { point to correct page.. }
      mov    ax, [Pixel]
      mov    es:[di], al
    end;
  {$else fpc}
  assembler;
  asm
      push eax
      push ebx
      push ecx
      push edi
{$IFDEF REGCALL}
      movsx  edi, ax
      movsx  ebx, dx
      mov    al, cl
{$ELSE REGCALL}
      movsx  edi, x
      movsx  ebx, y
{$ENDIF REGCALL}
      cmp    clippixels, 0
      je     @putpix320noclip
      test   edi, edi
      jl     @putpix320done
      test   ebx, ebx
      jl     @putpix320done
      cmp    di, ViewWidth
      jg     @putpix320done
      cmp    bx, ViewHeight
      jg     @putpix320done
@putpix320noclip:
      movsx  ecx, StartYViewPort
      movsx  edx, StartXViewPort
      add    ebx, ecx
      add    edi, edx
{    add    edi, [VideoOfs]      no multiple pages in 320*200*256 }
{$IFNDEF REGCALL}
      mov    ax, [pixel]
{$ENDIF REGCALL}
      shl    ebx, 6
      add    edi, ebx
      mov    fs:[edi+ebx*4+$a0000], al
@putpix320done:
      pop edi
      pop ecx
      pop ebx
      pop eax
{$endif fpc}
 end;


 Function GetPixel320(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
  {$ifndef fpc}
  Begin
   X:= X + StartXViewPort;
   Y:= Y + StartYViewPort;
   asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      xor    ax, ax
      add    di, [VideoOfs]   { point to correct gfx page ... }
      mov    al,es:[di]
      mov    @Result,ax
    end;
  {$else fpc}
  assembler;
  asm
    push eax
    push ebx
    push ecx
    push edx
    push edi
{$IFDEF REGCALL}
    movsx  edi, ax
    movsx  ebx, dx
{$ELSE REGCALL}
    movsx  edi, x
    movsx  ebx, y
{$ENDIF REGCALL}
    movsx  ecx, StartYViewPort
    movsx  edx, StartXViewPort
    add    ebx, ecx
    add    edi, edx
 {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
    shl    ebx, 6
    add    edi, ebx
    movzx  ax, byte ptr fs:[edi+ebx*4+$a0000]
    pop edi
    pop edx
    pop ecx
    pop ebx
    pop eax
 {$endif fpc}
  end;


 Procedure DirectPutPixel320(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
{$ifndef asmgraph}
 var offset: word;
     dummy: Byte;
 begin
   dummy := CurrentColor;
   offset := y * 320 + x + VideoOfs;
   case CurrentWriteMode of
     XorPut: dummy := dummy xor Mem[Sega000:offset];
     OrPut: dummy := dummy or Mem[Sega000:offset];
     AndPut: dummy := dummy and Mem[SegA000:offset];
     NotPut: dummy := Not dummy;
   end;
   Mem[SegA000:offset] := dummy;
 end;
{$else asmgraph}
{ note: still needs or/and/notput support !!!!! (JM) }
  assembler;
    asm
  {$ifndef fpc}
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
{      add    di, [VideoOfs] no multiple pages support in 320*200*256 }
      mov    ax, [CurrentColor]
      cmp    [CurrentWriteMode],XORPut   { check write mode   }
      jne    @MOVMode
      mov    ah,es:[di]        { read the byte...             }
      xor    al,ah             { xor it and return value into AL }
    @MovMode:
      mov    es:[di], al
  {$else fpc}
      push eax
      push ebx
      push edi
{$IFDEF REGCALL}
      movzx  edi, ax
      movzx  ebx, dx
{$ELSE REGCALL}
      movzx  edi, x
      movzx  ebx, y
{$ENDIF REGCALL}
   {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
      shl    ebx, 6
      add    edi, ebx
      mov    ax, [CurrentColor]
      cmp    [CurrentWriteMode],XORPut   { check write mode   }
      jne    @MOVMode
      xor    al, fs:[edi+ebx*4+$a0000]
     @MovMode:
      mov    fs:[edi+ebx*4+$a0000], al
      pop edi
      pop ebx
      pop eax
{$endif fpc}
  end;
{$endif asmgraph}


 procedure SetVisual320(page: word); {$ifndef fpc}far;{$endif fpc}
  { no page supPort... }
  begin
    VideoOfs := 0;
  end;

 procedure SetActive320(page: word); {$ifndef fpc}far;{$endif fpc}
  { no page supPort... }
  begin
    VideoOfs := 0;
  end;

 {************************************************************************}
 {*                       Mode-X related routines                        *}
 {************************************************************************}
const CrtAddress: word = 0;

 procedure InitModeX; {$ifndef fpc}far;{$endif fpc}
=======
 {************************************************************************}
 {*                     4-bit planar VGA mode routines                   *}
 {************************************************************************}

  Procedure Init640x200x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($8e)
      else
        CallInt10($e);
    end;


   Procedure Init640x350x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($90)
      else
        CallInt10($10);
    end;



  Procedure Init640x480x16; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($92)
      else
        CallInt10($12);
    end;




 Procedure PutPixel16(X,Y : smallint; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
     dummy: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
{$ifndef asmgraph}
     offset := y * 80 + (x shr 3) + VideoOfs;
     PortW[$3ce] := $0f01;       { Index 01 : Enable ops on all 4 planes }
     PortW[$3ce] := (Pixel and $ff) shl 8; { Index 00 : Enable correct plane and write color }

     Port[$3ce] := 8;
     Port[$3cf] := $80 shr (x and $7); { Select correct bits to modify }
     dummy := Mem[SegA000: offset];  { Latch the data into host space.  }
     Mem[Sega000: offset] := dummy;  { Write the data into video memory }
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
{$else asmgraph}
      asm
 {$ifndef fpc}
        mov  es, [SegA000]
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        mov  ax,[X]
        shr  ax,3
        push ax
        { determine the address }
        mov  ax,80
        mov  bx,[Y]
        mul  bx
        pop  cx
        add  ax,cx
        mov  di,ax
        add  di, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,es:[di]
        mov  es:[di],bl

        { reset for formal vga operation }
        mov  dx,3ceh
        mov  ax,0ff08h
        out  dx,ax

        { restore enable set/reset register }
        mov  ax,0001h
        out  dx,ax
 {$else fpc}
        push eax
        push ebx
        push ecx
        push edx
        push edi
        { enable the set / reset function and load the color }
        mov  dx, 3ceh
        mov  ax, 0f01h
        out  dx, ax
        { setup set/reset register }
        mov  ax, [Pixel]
        shl  ax, 8
        out  dx, ax
        { setup the bit mask register }
        mov  al, 8
        out  dx, al
        inc  dx
        { load the bitmask register }
        mov  cx, [X]
        and  cx, 0007h
        mov  al, 80h
        shr  al, cl
        out  dx, ax
        { get the x index and divide by 8 for 16-color }
        movzx eax,[X]
        shr  eax,3
        push eax
        { determine the address }
        mov  eax,80
        mov  bx,[Y]
        mul  bx
        pop  ecx
        add  eax,ecx
        mov  edi,eax
        add  edi, [VideoOfs]
        { send the data through the display memory through set/reset }
        mov  bl,fs:[edi+$a0000]
        mov  fs:[edi+$a0000],bl

        { reset for formal vga operation }
        mov  dx,3ceh
        mov  ax,0ff08h
        out  dx,ax

        { restore enable set/reset register }
        mov  ax,0001h
        out  dx,ax
        pop edi
        pop edx
        pop ecx
        pop ebx
        pop eax
 {$endif fpc}
      end;
{$endif asmgraph}
   end;


 Function GetPixel16(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 Var dummy, offset: Word;
     shift: byte;
{$endif asmgraph}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
{$ifndef asmgraph}
    offset := Y * 80 + (x shr 3) + VideoOfs;
    Port[$3ce] := 4;
    shift := 7 - (X and 7);
    Port[$3cf] := 0;
    dummy := (Mem[Sega000:offset] shr shift) and 1;
    Port[$3cf] := 1;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 1);
    Port[$3cf] := 2;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 2);
    Port[$3cf] := 3;
    dummy := dummy or (((Mem[Sega000:offset] shr shift) and 1) shl 3);
    GetPixel16 := dummy;
{$else asmgraph}
    asm
  {$ifndef fpc}
      mov   ax, [X]          { Get X address                    }
      push  ax
      shr   ax, 3
      push  ax

      mov   ax,80
      mov   bx,[Y]
      mul   bx
      pop   cx
      add   ax,cx
      mov   si,ax            { SI = correct offset into video segment }

      mov   es,[SegA000]
      add   si,[VideoOfs]    { Point to correct page offset... }

      mov   dx,03ceh
      mov   ax,4
      out   dx,al
      inc   dx

      pop   ax
      and   ax,0007h
      mov   cl,07
      sub   cl,al
      mov   bl,cl

      { read plane 0 }
      mov   al,0             { Select plane to read }
      out   dx,al
      mov   al,es:[si]       { read display memory }
      shr   al,cl
      and   al,01h
      mov   ah,al            { save bit in AH       }

      { read plane 1 }
      mov   al,1             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,1
      or    ah,al            { save bit in AH      }

      { read plane 2 }
      mov   al,2             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,2
      or    ah,al            { save bit in AH       }

      { read plane 3 }
      mov   al,3             { Select plane to read }
      out   dx,al
      mov   al,es:[si]
      shr   al,cl
      and   al,01h
      shl   al,3
      or    ah,al            { save bit in AH       }

      mov   al,ah            { 16-bit pixel in AX   }
      xor   ah,ah
      mov   @Result, ax
  {$else fpc}
      push eax
      push ebx
      push ecx
      push edx
      push esi
      movzx eax, [X]          { Get X address                    }
      push  eax
      shr   eax, 3
      push  eax

      mov   eax,80
      mov   bx,[Y]
      mul   bx
      pop   ecx
      add   eax,ecx
      mov   esi,eax            { SI = correct offset into video segment }

      add   esi,[VideoOfs]    { Point to correct page offset... }

      mov   dx,03ceh
      mov   ax,4
      out   dx,al
      inc   dx

      pop   eax
      and   eax,0007h
      mov   cl,07
      sub   cl,al
      mov   bl,cl

      { read plane 0 }
      mov   al,0             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]       { read display memory }
      shr   al,cl
      and   al,01h
      mov   ah,al            { save bit in AH       }

      { read plane 1 }
      mov   al,1             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,1
      or    ah,al            { save bit in AH      }

      { read plane 2 }
      mov   al,2             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,2
      or    ah,al            { save bit in AH       }

      { read plane 3 }
      mov   al,3             { Select plane to read }
      out   dx,al
      mov   al,fs:[esi+$a0000]
      shr   al,cl
      and   al,01h
      shl   al,3
      or    ah,al            { save bit in AH       }

      mov   al,ah            { 16-bit pixel in AX   }
      xor   ah,ah
      mov   @Result, ax
      pop esi
      pop edx
      pop ecx
      pop ebx
      pop eax
  {$endif fpc}
    end;
{$endif asmgraph}
  end;

Procedure GetScanLine16(x1, x2, y: smallint; var data);

var dummylong: longint;
    Offset, count, count2, amount, index: word;
    plane: byte;
Begin
  inc(x1,StartXViewPort);
  inc(x2,StartXViewPort);
{$ifdef logging}
  LogLn('GetScanLine16 start, length to get: '+strf(x2-x1+1)+' at y = '+strf(y));
{$Endif logging}
  offset := (Y + StartYViewPort) * 80 + (x1 shr 3) + VideoOfs;
{$ifdef logging}
  LogLn('Offset: '+HexStr(offset,4)+' - ' + strf(offset));
{$Endif logging}
  { first get enough pixels so offset is 32bit aligned }
  amount := 0;
  index := 0;
  If ((x1 and 31) <> 0) Or
     ((x2-x1+1) < 32) Then
    Begin
      If ((x2-x1+1) >= 32+32-(x1 and 31)) Then
        amount := 32-(x1 and 31)
      Else amount := x2-x1+1;
{$ifdef logging}
      LogLn('amount to align to 32bits or to get all: ' + strf(amount));
{$Endif logging}
      For count := 0 to amount-1 do
        WordArray(Data)[Count] := getpixel16(x1-StartXViewPort+Count,y);
      index := amount;
      Inc(Offset,(amount+7) shr 3);
{$ifdef logging}
      LogLn('offset now: '+HexStr(offset,4)+' - ' + strf(offset));
      LogLn('index now: '+strf(index));
{$Endif logging}
    End;
  amount := x2-x1+1 - amount;
{$ifdef logging}
  LogLn('amount left: ' + strf(amount));
{$Endif logging}
  If amount = 0 Then Exit;
  Port[$3ce] := 4;
  { first get everything from plane 3 (4th plane) }
  Port[$3cf] := 3;
  Count := 0;
  For Count := 1 to (amount shr 5) Do
    Begin
      dummylong := MemL[SegA000:offset+(Count-1)*4];
      dummylong :=
        ((dummylong and $ff) shl 24) or
        ((dummylong and $ff00) shl 8) or
        ((dummylong and $ff0000) shr 8) or
        ((dummylong and $ff000000) shr 24);
      For Count2 := 31 downto 0 Do
        Begin
          WordArray(Data)[index+Count2] := DummyLong and 1;
          DummyLong := DummyLong shr 1;
        End;
      Inc(Index, 32);
    End;
{ Now get the data from the 3 other planes }
  plane := 3;
  Repeat
    Dec(Index,Count*32);
    Dec(plane);
    Port[$3cf] := plane;
    Count := 0;
    For Count := 1 to (amount shr 5) Do
      Begin
        dummylong := MemL[SegA000:offset+(Count-1)*4];
        dummylong :=
          ((dummylong and $ff) shl 24) or
          ((dummylong and $ff00) shl 8) or
          ((dummylong and $ff0000) shr 8) or
          ((dummylong and $ff000000) shr 24);
        For Count2 := 31 downto 0 Do
          Begin
            WordArray(Data)[index+Count2] :=
              (WordArray(Data)[index+Count2] shl 1) or (DummyLong and 1);
            DummyLong := DummyLong shr 1;
          End;
        Inc(Index, 32);
      End;
  Until plane = 0;
  amount := amount and 31;
  Dec(index);
{$ifdef Logging}
  LogLn('Last array index written to: '+strf(index));
  LogLn('amount left: '+strf(amount)+' starting at x = '+strf(index+1));
{$Endif logging}
  dec(x1,startXViewPort);
  For Count := 1 to amount Do
    WordArray(Data)[index+Count] := getpixel16(x1+index+Count,y);
{$ifdef logging}
  inc(x1,startXViewPort);
  LogLn('First 32 bytes gotten with getscanline16: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanline16: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  GetScanLineDefault(x1-StartXViewPort,x2-StartXViewPort,y,Data);
  LogLn('First 32 bytes gotten with getscanlinedef: ');
  If x2-x1+1 >= 32 Then
    Count2 := 32
  Else Count2 := x2-x1+1;
  For Count := 0 to Count2-1 Do
    Log(strf(WordArray(Data)[Count])+' ');
  LogLn('');
  If x2-x1+1 >= 32 Then
    Begin
      LogLn('Last 32 bytes gotten with getscanlinedef: ');
      For Count := 31 downto 0 Do
      Log(strf(WordArray(Data)[x2-x1-Count])+' ');
    End;
  LogLn('');
  LogLn('GetScanLine16 end');
{$Endif logging}
End;

 Procedure DirectPutPixel16(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
  var
   color: word;
{$ifndef asmgraph}
  offset: word;
  dummy: byte;
{$endif asmgraph}
 begin
    case CurrentWriteMode of
      XORPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Xor Color;
        end;
      OrPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Or Color;
        end;
      AndPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixel(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor And Color;
        end;
      NotPut:
        begin
          Color := (Not CurrentColor) and 15;
        end
      else
        Color := CurrentColor;
    end;
{$ifndef asmgraph}
    offset := Y * 80 + (X shr 3) + VideoOfs;
    PortW[$3ce] := $f01;
    PortW[$3ce] := Color shl 8;
    Port[$3ce] := 8;
    Port[$3cf] := $80 shr (X and 7);
    dummy := Mem[SegA000: offset];
    Mem[Sega000: offset] := dummy;
    PortW[$3ce] := $ff08;
    PortW[$3ce] := $0001;
{$else asmgraph}
{ note: still needs xor/or/and/notput support !!!!! (JM) }
    asm
  {$ifndef fpc}
      mov  es, [SegA000]
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      mov  ax,[X]
      shr  ax,3
      push ax
      { determine the address }
      mov  ax,80
      mov  bx,[Y]
      mul  bx
      pop  cx
      add  ax,cx
      mov  di,ax
      { send the data through the display memory through set/reset }
      add  di,[VideoOfs]   { add correct page }
      mov  bl,es:[di]
      mov  es:[di],bl

      { reset for formal vga operation }
      mov  dx,3ceh
      mov  ax,0ff08h
      out  dx,ax

      { restore enable set/reset register }
      mov  ax,0001h
      out  dx,ax
  {$else fpc}
      push eax
      push ebx
      push ecx
      push edx
      push edi
      { enable the set / reset function and load the color }
      mov  dx, 3ceh
      mov  ax, 0f01h
      out  dx, ax
      { setup set/reset register }
      mov  ax, [Color]
      shl  ax, 8
      out  dx, ax
      { setup the bit mask register }
      mov  al, 8
      out  dx, al
      inc  dx
      { load the bitmask register }
      mov  cx, [X]
      and  cx, 0007h
      mov  al, 80h
      shr  al, cl
      out  dx, ax
      { get the x index and divide by 8 for 16-color }
      movzx eax,[X]
      shr  eax,3
      push eax
      { determine the address }
      mov  eax,80
      mov  bx,[Y]
      mul  bx
      pop  ecx
      add  eax,ecx
      mov  edi,eax
      { send the data through the display memory through set/reset }
      add  edi,[VideoOfs]   { add correct page }
      mov  bl,fs:[edi+$a0000]
      mov  fs:[edi+$a0000],bl

      { reset for formal vga operation }
      mov  dx,3ceh
      mov  ax,0ff08h
      out  dx,ax

      { restore enable set/reset register }
      mov  ax,0001h
      out  dx,ax
      pop edi
      pop edx
      pop ecx
      pop ebx
      pop eax
  {$endif fpc}
    end;
{$endif asmgraph}
 end;


  procedure HLine16(x,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}

   var
      xtmp: smallint;
      ScrOfs,HLength : word;
      LMask,RMask : byte;

   Begin

    { must we swap the values? }
    if x > x2 then
      Begin
        xtmp := x2;
        x2 := x;
        x:= xtmp;
      end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    X2  := X2 + StartXViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x2,y,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8;
    HLength:=x2 div 8-x div 8;
    LMask:=$ff shr (x and 7);
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    RMask:=$ff shl (7-(x2 and 7));
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    if HLength=0 then
      LMask:=LMask and RMask;
    Port[$3ce]:=0;
    If CurrentWriteMode <> NotPut Then
      Port[$3cf]:= CurrentColor
    else Port[$3cf]:= not CurrentColor;
    Port[$3ce]:=1;
    Port[$3cf]:=$f;
    Port[$3ce]:=3;
    case CurrentWriteMode of
       XORPut:
         Port[$3cf]:=3 shl 3;
       ANDPut:
         Port[$3cf]:=1 shl 3;
       ORPut:
         Port[$3cf]:=2 shl 3;
       NormalPut, NotPut:
         Port[$3cf]:=0
       else
         Port[$3cf]:=0
    end;

    Port[$3ce]:=8;
    Port[$3cf]:=LMask;
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    Mem[SegA000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
    Port[$3ce]:=8;
    if HLength>0 then
      begin
         dec(HLength);
         inc(ScrOfs);
         if HLength>0 then
           begin
              Port[$3cf]:=$ff;
{$ifndef tp}
              seg_bytemove(dosmemselector,$a0000+ScrOfs,dosmemselector,$a0000+ScrOfs,HLength);
{$else}
              move(Ptr(SegA000,ScrOfs)^, Ptr(SegA000,ScrOfs)^, HLength);
{$endif}
              ScrOfs:=ScrOfs+HLength;
           end;
         Port[$3cf]:=RMask;
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
         Mem[Sega000:ScrOfs]:=Mem[SegA000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
      end;
    { clean up }
    Port[$3cf]:=0;
    Port[$3ce]:=8;
    Port[$3cf]:=$ff;
    Port[$3ce]:=1;
    Port[$3cf]:=0;
    Port[$3ce]:=3;
    Port[$3cf]:=0;
   end;

  procedure VLine16(x,y,y2: smallint); {$ifndef fpc}far;{$endif fpc}

   var
     ytmp: smallint;
     ScrOfs,i : longint;
     BitMask : byte;

  Begin
    { must we swap the values? }
    if y > y2 then
     Begin
       ytmp := y2;
       y2 := y;
       y:= ytmp;
     end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    Y2  := Y2 + StartYViewPort;
    Y   := Y + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    ScrOfs:=y*ScrWidth+x div 8;
    BitMask:=$80 shr (x and 7);
    Port[$3ce]:=0;
    If CurrentWriteMode <> NotPut Then
      Port[$3cf]:= CurrentColor
    else Port[$3cf]:= not CurrentColor;
    Port[$3ce]:=1;
    Port[$3cf]:=$f;
    Port[$3ce]:=8;
    Port[$3cf]:=BitMask;
    Port[$3ce]:=3;
    case CurrentWriteMode of
       XORPut:
         Port[$3cf]:=3 shl 3;
       ANDPut:
         Port[$3cf]:=1 shl 3;
       ORPut:
         Port[$3cf]:=2 shl 3;
       NormalPut, NotPut:
         Port[$3cf]:=0
       else
         Port[$3cf]:=0
    end;
    for i:=y to y2 do
      begin
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
         Mem[SegA000:ScrOfs]:=Mem[Sega000:ScrOfs]+1;
{$ifdef rangeOn}
{$undef rangeOn}
{$r+}
{$endif}
{$ifdef overflowOn}
{$undef overflowOn}
{$q+}
{$endif}
         ScrOfs:=ScrOfs+ScrWidth;
      end;
    { clean up }
    Port[$3cf]:=0;
    Port[$3ce]:=8;
    Port[$3cf]:=$ff;
    Port[$3ce]:=1;
    Port[$3cf]:=0;
    Port[$3ce]:=3;
    Port[$3cf]:=0;
  End;


 procedure SetVisual480(page: word); {$ifndef fpc}far;{$endif fpc}
 { no page flipping supPort in 640x480 mode }
  begin
    VideoOfs := 0;
  end;

 procedure SetActive480(page: word); {$ifndef fpc}far;{$endif fpc}
 { no page flipping supPort in 640x480 mode }
  begin
    VideoOfs := 0;
  end;


 procedure SetVisual200(page: word); {$ifndef fpc}far;{$endif fpc}
  { two page supPort... }
  begin
    if page > HardwarePages then exit;
    asm
      mov ax,[page]    { only lower byte is supPorted. }
      mov ah,05h
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}

      { read start address }
      mov dx,3d4h
      mov al,0ch
      out dx,al
      inc dx
      in  al,dx
      mov ah,al
      dec dx
      mov al,0dh
      out dx,al
      in  al,dx
    end ['EDX','EAX'];
  end;

 procedure SetActive200(page: word); {$ifndef fpc}far;{$endif fpc}
  { two page supPort... }
  begin
    case page of
     0 : VideoOfs := 0;
     1 : VideoOfs := 16384;
     2 : VideoOfs := 32768;
    else
      VideoOfs := 0;
    end;
  end;

 procedure SetVisual350(page: word); {$ifndef fpc}far;{$endif fpc}
  { one page supPort... }
  begin
    if page > HardwarePages then exit;
    asm
      mov ax,[page]    { only lower byte is supPorted. }
      mov ah,05h
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int 10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
    end ['EAX'];
  end;

 procedure SetActive350(page: word); {$ifndef fpc}far;{$endif fpc}
  { one page supPort... }
  begin
    case page of
     0 : VideoOfs := 0;
     1 : VideoOfs := 32768;
    else
      VideoOfs := 0;
    end;
  end;





 {************************************************************************}
 {*                     320x200x256c Routines                            *}
 {************************************************************************}

 Procedure Init320; {$ifndef fpc}far;{$endif fpc}
    begin
      if DontClearGraphMemory then
        CallInt10($93)
      else
        CallInt10($13);
    end;



 Procedure PutPixel320(X,Y : smallint; Pixel: Word); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in local coordinates. Clipping if required. }
  {$ifndef fpc}
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
    asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      add    di, [VideoOfs]    { point to correct page.. }
      mov    ax, [Pixel]
      mov    es:[di], al
    end;
  {$else fpc}
  assembler;
  asm
      push eax
      push ebx
      push ecx
      push edi
{$IFDEF REGCALL}
      movsx  edi, ax
      movsx  ebx, dx
      mov    al, cl
{$ELSE REGCALL}
      movsx  edi, x
      movsx  ebx, y
{$ENDIF REGCALL}
      cmp    clippixels, 0
      je     @putpix320noclip
      test   edi, edi
      jl     @putpix320done
      test   ebx, ebx
      jl     @putpix320done
      cmp    di, ViewWidth
      jg     @putpix320done
      cmp    bx, ViewHeight
      jg     @putpix320done
@putpix320noclip:
      movsx  ecx, StartYViewPort
      movsx  edx, StartXViewPort
      add    ebx, ecx
      add    edi, edx
{    add    edi, [VideoOfs]      no multiple pages in 320*200*256 }
{$IFNDEF REGCALL}
      mov    ax, [pixel]
{$ENDIF REGCALL}
      shl    ebx, 6
      add    edi, ebx
      mov    fs:[edi+ebx*4+$a0000], al
@putpix320done:
      pop edi
      pop ecx
      pop ebx
      pop eax
{$endif fpc}
 end;


 Function GetPixel320(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
  {$ifndef fpc}
  Begin
   X:= X + StartXViewPort;
   Y:= Y + StartYViewPort;
   asm
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
      xor    ax, ax
      add    di, [VideoOfs]   { point to correct gfx page ... }
      mov    al,es:[di]
      mov    @Result,ax
    end;
  {$else fpc}
  assembler;
  asm
    push eax
    push ebx
    push ecx
    push edx
    push edi
{$IFDEF REGCALL}
    movsx  edi, ax
    movsx  ebx, dx
{$ELSE REGCALL}
    movsx  edi, x
    movsx  ebx, y
{$ENDIF REGCALL}
    movsx  ecx, StartYViewPort
    movsx  edx, StartXViewPort
    add    ebx, ecx
    add    edi, edx
 {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
    shl    ebx, 6
    add    edi, ebx
    movzx  ax, byte ptr fs:[edi+ebx*4+$a0000]
    pop edi
    pop edx
    pop ecx
    pop ebx
    pop eax
 {$endif fpc}
  end;


 Procedure DirectPutPixel320(X,Y : smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
{$ifndef asmgraph}
 var offset: word;
     dummy: Byte;
 begin
   dummy := CurrentColor;
   offset := y * 320 + x + VideoOfs;
   case CurrentWriteMode of
     XorPut: dummy := dummy xor Mem[Sega000:offset];
     OrPut: dummy := dummy or Mem[Sega000:offset];
     AndPut: dummy := dummy and Mem[SegA000:offset];
     NotPut: dummy := Not dummy;
   end;
   Mem[SegA000:offset] := dummy;
 end;
{$else asmgraph}
{ note: still needs or/and/notput support !!!!! (JM) }
  assembler;
    asm
  {$ifndef fpc}
      mov    es, [SegA000]
      mov    ax, [Y]
      mov    di, [X]
      xchg   ah, al            { The value of Y must be in AH }
      add    di, ax
      shr    ax, 2
      add    di, ax
{      add    di, [VideoOfs] no multiple pages support in 320*200*256 }
      mov    ax, [CurrentColor]
      cmp    [CurrentWriteMode],XORPut   { check write mode   }
      jne    @MOVMode
      mov    ah,es:[di]        { read the byte...             }
      xor    al,ah             { xor it and return value into AL }
    @MovMode:
      mov    es:[di], al
  {$else fpc}
      push eax
      push ebx
      push edi
{$IFDEF REGCALL}
      movzx  edi, ax
      movzx  ebx, dx
{$ELSE REGCALL}
      movzx  edi, x
      movzx  ebx, y
{$ENDIF REGCALL}
   {   add    edi, [VideoOfs]       no multiple pages in 320*200*256 }
      shl    ebx, 6
      add    edi, ebx
      mov    ax, [CurrentColor]
      cmp    [CurrentWriteMode],XORPut   { check write mode   }
      jne    @MOVMode
      xor    al, fs:[edi+ebx*4+$a0000]
     @MovMode:
      mov    fs:[edi+ebx*4+$a0000], al
      pop edi
      pop ebx
      pop eax
{$endif fpc}
  end;
{$endif asmgraph}


 procedure SetVisual320(page: word); {$ifndef fpc}far;{$endif fpc}
  { no page supPort... }
  begin
    VideoOfs := 0;
  end;

 procedure SetActive320(page: word); {$ifndef fpc}far;{$endif fpc}
  { no page supPort... }
  begin
    VideoOfs := 0;
  end;

 {************************************************************************}
 {*                       Mode-X related routines                        *}
 {************************************************************************}
const CrtAddress: word = 0;

 procedure InitModeX; {$ifndef fpc}far;{$endif fpc}
>>>>>>> origin/fixes_2_2
  begin
   asm
     {see if we are using color-/monochorme display}
     MOV DX,3CCh  {use output register:     }
     IN AL,DX
     TEST AL,1    {is it a color display?    }
     MOV DX,3D4h
     JNZ @L1      {yes  }
     MOV DX,3B4h  {no  }
  @L1:          {DX = 3B4h / 3D4h = CRTAddress-register for monochrome/color}
     MOV CRTAddress,DX

<<<<<<< HEAD
     MOV  AX, 0013h
     MOV  BL, DontClearGraphMemory
     OR   BL,BL
     JZ   @L2
     OR   AX, 080h
  @L2:
{$ifdef fpc}
     push ebp
     push esi
     push edi
     push ebx
{$EndIf fpc}
     INT  10h
{$ifdef fpc}
     pop ebx
     pop edi
     pop esi
     pop ebp
{$EndIf fpc}
     MOV DX,03C4h   {select memory-mode-register at sequencer Port    }
     MOV AL,04
     OUT DX,AL
     INC DX         {read in data via the according data register     }
     IN  AL,DX
     AND AL,0F7h    {bit 3 := 0: don't chain the 4 planes}
     OR  AL,04      {bit 2 := 1: no odd/even mechanism }
     OUT DX,AL      {activate new settings    }
     MOV DX,03C4h   {s.a.: address sequencer reg. 2 (=map-mask),...   }
     MOV AL,02
     OUT DX,AL
     INC DX
     MOV AL,0Fh     {...and allow access to all 4 bit maps            }
     OUT DX,AL
{$ifndef fpc}
     MOV AX,[SegA000]  {starting with segment A000h, set 8000h logical     }
     MOV ES,AX      {words = 4*8000h physical words (because of 4     }
     XOR DI,DI      {bitplanes) to 0                                  }
     XOR AX,AX
     MOV CX,8000h
     CLD
     REP STOSW
{$else fpc}
     push eax
     push ecx
     push es
     push edi
     push fs
     mov edi, $a0000
     pop es
     xor eax, eax
     mov ecx, 4000h
     cld
     rep stosd
     pop edi
     pop es
     pop ecx
     pop eax
{$EndIf fpc}
     MOV DX,CRTAddress  {address the underline-location-register at }
     MOV AL,14h         {the CRT-controller Port, read out the according      }
     OUT DX,AL          {data register:                            }
     INC DX
     IN  AL,DX
     AND AL,0BFh    {bit 6:=0: no double word addressing scheme in}
     OUT DX,AL      {video RAM                              }
     DEC DX
     MOV AL,17h     {select mode control register     }
     OUT DX,AL
     INC DX
     IN  AL,DX
     OR  AL,40h     {bit 6 := 1: memory access scheme=linear bit array      }
     OUT DX,AL
  end ['EDX','EBX','EAX'];
 end;


 Function GetPixelX(X,Y: smallint): word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
{$endif asmgraph}
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort;
{$ifndef asmgraph}
     offset := y * 80 + x shr 2 + VideoOfs;
     PortW[$3ce] := ((x and 3) shl 8) + 4;
     GetPixelX := Mem[SegA000:offset];
{$else asmgraph}
    asm
  {$ifndef fpc}
     mov di,[Y]                   ; (* DI = Y coordinate                 *)
     (* Multiply by 80 start *)
     mov bx, di
     shl di, 6                    ; (* Faster on 286/386/486 machines    *)
     shl bx, 4
     add di, bx                   ;  (* Multiply Value by 80             *)
     (* End multiply by 80  *)
     mov cx, [X]
     mov ax, cx
    {DI = Y * LINESIZE, BX = X, coordinates admissible}
     shr ax, 1                    ; (* Faster on 286/86 machines         *)
     shr ax, 1
     add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
     add di, [VideoOfs]  ; (* Pointing at start of Active page *)
    (* Select plane to use *)
    mov dx, 03c4h
    mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
    and cl, 03h               ; (* Get Plane Bits                   *)
    shl ah, cl                ; (* Get Plane Select Value           *)
    out dx, ax
   (* End selection of plane *)
    mov es,[SegA000]
    mov al, ES:[DI]
    xor ah, ah
    mov @Result, ax
  {$else fpc}
     push eax  
     push ebx
     push ecx
     push edx
     push edi
     movzx edi,[Y]                   ; (* DI = Y coordinate                 *)
     (* Multiply by 80 start *)
     mov ebx, edi
     shl edi, 6                    ; (* Faster on 286/386/486 machines    *)
     shl ebx, 4
     add edi, ebx                   ;  (* Multiply Value by 80             *)
     (* End multiply by 80  *)
     movzx ecx, [X]
     movzx eax, [Y]
    {DI = Y * LINESIZE, BX = X, coordinates admissible}
     shr eax, 2
     add edi, eax                ; {DI = Y * LINESIZE + (X SHR 2) }
     add edi, [VideoOfs]  ; (* Pointing at start of Active page *)
    (* Select plane to use *)
    mov dx, 03c4h
    mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
    and cl, 03h               ; (* Get Plane Bits                   *)
    shl ah, cl                ; (* Get Plane Select Value           *)
    out dx, ax
   (* End selection of plane *)
    mov ax, fs:[edi+$a0000]
    mov @Result, ax
    pop edi
    pop edx
    pop ecx
    pop ebx
    pop eax
  {$endif fpc}
   end;
{$endif asmgraph}
 end;

 procedure SetVisualX(page: word); {$ifndef fpc}far;{$endif fpc}
  { 4 page supPort... }

   Procedure SetVisibleStart(AOffset: word); Assembler;
   (* Select where the left corner of the screen will be *)
   { By Matt Pritchard }
    asm
     push ax
     push cx
     push dx
{$IFDEF REGCALL}
     mov cx, ax
{$ENDIF REGCALL}
      { Wait if we are currently in a Vertical Retrace        }
     MOV     DX, INPUT_1         { Input Status #1 Register       }
   @DP_WAIT0:
     IN      AL, DX              { Get VGA status                 }
     AND     AL, VERT_RETRACE    { In Display mode yet?           }
     JNZ     @DP_WAIT0           { If Not, wait for it            }

    { Set the Start Display Address to the new page         }

     MOV     DX, CRTC_Index      { We Change the VGA Sequencer    }
     MOV     AL, START_DISP_LO   { Display Start Low Register     }
{$ifndef fpc}
     MOV     AH, BYTE PTR [AOffset] { Low 8 Bits of Start Addr    }
     OUT     DX, AX              { Set Display Addr Low           }
     MOV     AL, START_DISP_HI   { Display Start High Register    }
     MOV     AH, BYTE PTR [AOffset+1] { High 8 Bits of Start Addr }
{$else fpc}
{$IFDEF REGCALL}
    mov ah, cl
{$ELSE REGCALL}
    mov ah, byte [AOffset]
{$ENDIF REGCALL}
    out dx, ax
    mov AL, START_DISP_HI
{$IFDEF REGCALL}
    mov ah, ch
{$ELSE REGCALL}
    mov ah, byte [AOffset+1]
{$ENDIF REGCALL}
{$endif fpc}
     OUT     DX, AX              { Set Display Addr High          }
     { Wait for a Vertical Retrace to smooth out things      }

     MOV     DX, INPUT_1         { Input Status #1 Register       }

  @DP_WAIT1:
     IN      AL, DX              { Get VGA status                 }
     AND     AL, VERT_RETRACE    { Vertical Retrace Start?        }
     JZ      @DP_WAIT1           { If Not, wait for it            }
    { Now Set Display Starting Address                     }
     pop dx
     pop cx
     pop ax
  end;

{$ifdef fpc}
  {$undef asmgraph}
{$endif fpc}

  begin
    Case page of
      0: SetVisibleStart(0);
      1: SetVisibleStart(16000);
      2: SetVisibleStart(32000);
      3: SetVisibleStart(48000);
    else
      SetVisibleStart(0);
    end;
  end;

 procedure SetActiveX(page: word); {$ifndef fpc}far;{$endif fpc}
  { 4 page supPort... }
  begin
   case page of
     0: VideoOfs := 0;
     1: VideoOfs := 16000;
     2: VideoOfs := 32000;
     3: VideoOfs := 48000;
   else
     VideoOfs:=0;
   end;
  end;

 Procedure PutPixelX(X,Y: smallint; color:word); {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
{$endif asmgraph}
  begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
{$ifndef asmgraph}
    offset := y * 80 + x shr 2 + VideoOfs;
    PortW[$3c4] := (hi(word(FirstPlane)) shl 8) shl (x and 3)+ lo(word(FirstPlane));
    Mem[SegA000:offset] := color;
{$else asmgraph}
     asm
      push ax
      push bx
      push cx
      push dx
      push es
      push di
      mov di,[Y]                   ; (* DI = Y coordinate                 *)
      (* Multiply by 80 start *)
      mov bx, di
      shl di, 6                    ; (* Faster on 286/386/486 machines    *)
      shl bx, 4
      add di, bx                   ;  (* Multiply Value by 80             *)
      (* End multiply by 80  *)
      mov cx, [X]
      mov ax, cx
      {DI = Y * LINESIZE, BX = X, coordinates admissible}
      shr ax, 2
      add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
      add di, [VideoOfs]        ; (* Pointing at start of Active page *)
      (* Select plane to use *)
      mov dx, 03c4h
      mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
      and cl, 03h               ; (* Get Plane Bits                   *)
      shl ah, cl                ; (* Get Plane Select Value           *)
      out dx, ax
      (* End selection of plane *)
      mov es,[SegA000]
      mov ax,[Color]            ; { only lower byte is used. }
      cmp [CurrentWriteMode],XORPut   { check write mode   }
      jne @MOVMode
      mov ah,es:[di]        { read the byte...             }
      xor al,ah             { xor it and return value into AL }
    @MovMode:
      mov es:[di], al
      pop di
      pop es
      pop dx
      pop cx
      pop bx
      pop ax
    end;
{$endif asmgraph}
  end;


 Procedure DirectPutPixelX(X,Y: smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
{$ifndef asmgraph}
 Var offset: Word;
     dummy: Byte;
 begin
   offset := y * 80 + x shr 2 + VideoOfs;
   case CurrentWriteMode of
     XorPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor xor Mem[Sega000: offset];
       end;
     OrPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor or Mem[Sega000: offset];
       end;
     AndPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor and Mem[Sega000: offset];
       end;
     NotPut: dummy := Not CurrentColor;
     else dummy := CurrentColor;
   end;
   PortW[$3c4] := (hi(word(FirstPlane)) shl 8) shl (x and 3)+ lo(word(FirstPlane));
   Mem[Sega000: offset] := Dummy;
 end;
{$else asmgraph}
{ note: still needs or/and/notput support !!!!! (JM) }
 Assembler;
 asm
   push ax
   push bx
   push cx
   push dx
   push es
   push di
{$IFDEF REGCALL}
   mov cl, al
   mov di, dx
{$ELSE REGCALL}
   mov cx, [X]
   mov ax, cx
   mov di, [Y]                   ; (* DI = Y coordinate                 *)
{$ENDIF REGCALL}
 (* Multiply by 80 start *)
   mov bx, di
   shl di, 6                    ; (* Faster on 286/386/486 machines    *)
   shl bx, 4
   add di, bx                   ;  (* Multiply Value by 80             *)
 (* End multiply by 80  *)
  {DI = Y * LINESIZE, BX = X, coordinates admissible}
   shr ax, 2
   add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
   add di, [VideoOfs]        ; (* Pointing at start of Active page *)
 (* Select plane to use *)
   mov dx, 03c4h
   mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
   and cl, 03h               ; (* Get Plane Bits                   *)
   shl ah, cl                ; (* Get Plane Select Value           *)
   out dx, ax
 (* End selection of plane *)
   mov es,[SegA000]
   mov ax,[CurrentColor]     ; { only lower byte is used. }
   cmp [CurrentWriteMode],XORPut   { check write mode   }
   jne @MOVMode
   mov ah,es:[di]        { read the byte...             }
   xor al,ah             { xor it and return value into AL }
 @MovMode:
   mov es:[di], al
   pop di
   pop es
   pop dx
   pop cx
   pop bx
   pop ax
 end;
{$endif asmgraph}



 {************************************************************************}
 {*                       General routines                               *}
 {************************************************************************}
 var
  SavePtr : pointer;    { pointer to video state                 }
{  CrtSavePtr: pointer;}  { pointer to video state when CrtMode gets called }
  StateSize: word;      { size in 64 byte blocks for video state }
  VideoMode: byte;      { old video mode before graph mode       }
  SaveSupPorted : Boolean;    { Save/Restore video state supPorted? }


      {**************************************************************}
      {*                     DPMI Routines                          *}
      {**************************************************************}

{$IFDEF DPMI}
  RealStateSeg: word;    { Real segment of saved video state }

 Procedure SaveStateVGA; {$ifndef fpc}far;{$endif fpc}
 var
  PtrLong: longint;
  regs: TDPMIRegisters;
  begin
    SaveSupPorted := FALSE;
    SavePtr := nil;
    { Get the video mode }
    asm
      mov  ah,0fh
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
      mov  [VideoMode], al
    end ['EAX'];
    { saving/restoring video state screws up Windows (JM) }
    if inWindows then
      exit;
    { Prepare to save video state...}
    asm
      mov  ax, 1C00h       { get buffer size to save state }
      mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
{$ifdef fpc}
      push ebx
      push ebp
      push esi
      push edi
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop edi
      pop esi
      pop ebp
{$endif fpc}
      mov  [StateSize], bx
{$ifdef fpc}
      pop ebx
{$endif fpc}
      cmp  al,01ch
      jnz  @notok
      mov  [SaveSupPorted],TRUE
     @notok:
    end ['ECX','EAX'];
    if SaveSupPorted then
      begin

{$ifndef fpc}
        PtrLong:=GlobalDosAlloc(64*StateSize);  { values returned in 64-byte blocks }
{$else fpc}
        PtrLong:=Global_Dos_Alloc(64*StateSize);  { values returned in 64-byte blocks }
{$endif fpc}
        if PtrLong = 0 then
           RunError(203);
        SavePtr := pointer(longint(PtrLong and $0000ffff) shl 16);
{$ifndef fpc}
        { In FPC mode, we can't do anything with this (no far pointers)  }
        { However, we still need to keep it to be able to free the       }
        { memory afterwards. Since this data is not accessed in PM code, }
        { there's no need to save it in a seperate buffer (JM)           }
        if not assigned(SavePtr) then
           RunError(203);
{$endif fpc}
        RealStateSeg := word(PtrLong shr 16);
        FillChar(regs, sizeof(regs), #0);
        { call the real mode interrupt ... }
        regs.eax := $1C01;      { save the state buffer                   }
        regs.ecx := $07;        { Save DAC / Data areas / Hardware states }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        regs.eax := $1C02;      { restore the state buffer                }
        regs.ecx := $07;        { rest DAC / Data areas / Hardware states }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
      end;
  end;

 procedure RestoreStateVGA; {$ifndef fpc}far;{$endif fpc}
  var
   regs:TDPMIRegisters;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
     end ['EAX'];
     { then restore all state information }
{$ifndef fpc}
     if assigned(SavePtr) and (SaveSupPorted=TRUE) then
{$else fpc}
     { No far pointer supPort, so it's possible that that assigned(SavePtr) }
     { would return false under FPC. Just check if it's different from nil. }
     if (SavePtr <> nil) and (SaveSupPorted=TRUE) then
{$endif fpc}
      begin
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
         regs.eax := $1C02;      { restore the state buffer                }
         regs.ecx := $07;        { rest DAC / Data areas / Hardware states }
         regs.es := RealStateSeg;
         regs.ebx := 0;
         RealIntr($10,regs);
(*
{$ifndef fpc}
         if GlobalDosFree(longint(SavePtr) shr 16)<>0 then
{$else fpc}
         if Not Global_Dos_Free(longint(SavePtr) shr 16) then
{$endif fpc}
          RunError(216);

         SavePtr := nil;
*)
       end;
  end;

{$ELSE}

      {**************************************************************}
      {*                     Real mode routines                     *}
      {**************************************************************}


 Procedure SaveStateVGA; far;
  begin
    SavePtr := nil;
    SaveSupPorted := FALSE;
    { Get the video mode }
    asm
      mov  ah,0fh
      int  10h
      mov  [VideoMode], al
    end;
    { Prepare to save video state...}
    asm
      mov  ax, 1C00h       { get buffer size to save state }
      mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
      int  10h
      mov  [StateSize], bx
      cmp  al,01ch
      jnz  @notok
      mov  [SaveSupPorted],TRUE
     @notok:
    end;
    if SaveSupPorted then
      Begin
        GetMem(SavePtr, 64*StateSize); { values returned in 64-byte blocks }
        if not assigned(SavePtr) then
           RunError(203);
        asm
         mov  ax, 1C01h       { save the state buffer                   }
         mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        asm
         mov  ax, 1C02h       { save the state buffer                   }
         mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
      end;
  end;

 procedure RestoreStateVGA; far;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
      int  10h
     end;

     { then restore all state information }
     if assigned(SavePtr) and (SaveSupPorted=TRUE) then
       begin
         { restore state, according to Ralph Brown Interrupt list }
         asm
           mov  ax, 1C02h       { save the state buffer                   }
           mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
           mov  es, WORD PTR [SavePtr+2]
           mov  bx, WORD PTR [SavePtr]
           int  10h
         end;
{        done in exitproc (JM)
         FreeMem(SavePtr, 64*StateSize);}
         SavePtr := nil;
       end;
  end;
{$ENDIF DPMI}

   Procedure SetVGARGBAllPalette(const Palette:PaletteType); {$ifndef fpc}far;{$endif fpc}
    var
      c: byte;
    begin
      { wait for vertical retrace start/end}
      while (port[$3da] and $8) <> 0 do;
      while (port[$3da] and $8) = 0 do;
      If MaxColor = 16 Then
        begin
          for c := 0 to 15 do
            begin
              { translate the color number for 16 color mode }
              portb[$3c8] := toRealCols16[c];
              portb[$3c9] := palette.colors[c].red shr 2;
              portb[$3c9] := palette.colors[c].green shr 2;
              portb[$3c9] := palette.colors[c].blue shr 2;
            end
        end
      else
        begin
          portb[$3c8] := 0;
          for c := 0 to 255 do
            begin
              { no need to set port[$3c8] every time if you set the entries }
              { for successive colornumbers (JM)                            }
              portb[$3c9] := palette.colors[c].red shr 2;
              portb[$3c9] := palette.colors[c].green shr 2;
              portb[$3c9] := palette.colors[c].blue shr 2;
          end
        end;
    End;


   { VGA is never a direct color mode, so no need to check ... }
   Procedure SetVGARGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
    begin
      { translate the color number for 16 color mode }
      If MaxColor = 16 Then
        ColorNum := ToRealCols16[ColorNum];
      asm
        { on some hardware - there is a snow like effect       }
        { when changing the palette register directly          }
        { so we wait for a vertical retrace start period.      }
        push ax
        push dx
        mov dx, $03da
      @1:
        in    al, dx          { Get input status register    }
        test  al, $08         { check if in vertical retrace }
        jnz   @1              { yes, complete it             }
                              { we have to wait for the next }
                              { retrace to assure ourselves  }
                              { that we have time to complete }
                              { the DAC operation within      }
                              { the vertical retrace period   }
       @2:
        in    al, dx
        test  al, $08
        jz    @2              { repeat until vertical retrace start }

        mov dx, $03c8       { Set color register address to use }
        mov ax, [ColorNum]
        out dx, al
        inc dx              { Point to DAC registers            }
        mov ax, [RedValue]  { Get RedValue                      }
        shr ax, 2
        out dx, al
        mov ax, [GreenValue]{ Get RedValue                      }
        shr ax, 2
        out dx, al
        mov ax, [BlueValue] { Get RedValue                      }
        shr ax, 2
        out dx, al
        pop dx
        pop ax
      end
    End;


   { VGA is never a direct color mode, so no need to check ... }
  Procedure GetVGARGBPalette(ColorNum: smallint; Var
      RedValue, GreenValue, BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
   begin
     If MaxColor = 16 Then
       ColorNum := ToRealCols16[ColorNum];
     Port[$03C7] := ColorNum;
     { we must convert to lsb values... because the vga uses the 6 msb bits }
     { which is not compatible with anything.                               }
     RedValue := smallint(Port[$3C9]) shl 2;
     GreenValue := smallint(Port[$3C9]) shl 2;
     BlueValue := smallint(Port[$3C9]) shl 2;
   end;


 {************************************************************************}
 {*                       VESA related routines                          *}
 {************************************************************************}
{$I vesa.inc}
=======
     MOV  AX, 0013h
     MOV  BL, DontClearGraphMemory
     OR   BL,BL
     JZ   @L2
     OR   AX, 080h
  @L2:
{$ifdef fpc}
     push ebp
     push esi
     push edi
     push ebx
{$EndIf fpc}
     INT  10h
{$ifdef fpc}
     pop ebx
     pop edi
     pop esi
     pop ebp
{$EndIf fpc}
     MOV DX,03C4h   {select memory-mode-register at sequencer Port    }
     MOV AL,04
     OUT DX,AL
     INC DX         {read in data via the according data register     }
     IN  AL,DX
     AND AL,0F7h    {bit 3 := 0: don't chain the 4 planes}
     OR  AL,04      {bit 2 := 1: no odd/even mechanism }
     OUT DX,AL      {activate new settings    }
     MOV DX,03C4h   {s.a.: address sequencer reg. 2 (=map-mask),...   }
     MOV AL,02
     OUT DX,AL
     INC DX
     MOV AL,0Fh     {...and allow access to all 4 bit maps            }
     OUT DX,AL
{$ifndef fpc}
     MOV AX,[SegA000]  {starting with segment A000h, set 8000h logical     }
     MOV ES,AX      {words = 4*8000h physical words (because of 4     }
     XOR DI,DI      {bitplanes) to 0                                  }
     XOR AX,AX
     MOV CX,8000h
     CLD
     REP STOSW
{$else fpc}
     push eax
     push ecx
     push es
     push edi
     push fs
     mov edi, $a0000
     pop es
     xor eax, eax
     mov ecx, 4000h
     cld
     rep stosd
     pop edi
     pop es
     pop ecx
     pop eax
{$EndIf fpc}
     MOV DX,CRTAddress  {address the underline-location-register at }
     MOV AL,14h         {the CRT-controller Port, read out the according      }
     OUT DX,AL          {data register:                            }
     INC DX
     IN  AL,DX
     AND AL,0BFh    {bit 6:=0: no double word addressing scheme in}
     OUT DX,AL      {video RAM                              }
     DEC DX
     MOV AL,17h     {select mode control register     }
     OUT DX,AL
     INC DX
     IN  AL,DX
     OR  AL,40h     {bit 6 := 1: memory access scheme=linear bit array      }
     OUT DX,AL
  end ['EDX','EBX','EAX'];
 end;


 Function GetPixelX(X,Y: smallint): word; {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
{$endif asmgraph}
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort;
{$ifndef asmgraph}
     offset := y * 80 + x shr 2 + VideoOfs;
     PortW[$3ce] := ((x and 3) shl 8) + 4;
     GetPixelX := Mem[SegA000:offset];
{$else asmgraph}
    asm
  {$ifndef fpc}
     mov di,[Y]                   ; (* DI = Y coordinate                 *)
     (* Multiply by 80 start *)
     mov bx, di
     shl di, 6                    ; (* Faster on 286/386/486 machines    *)
     shl bx, 4
     add di, bx                   ;  (* Multiply Value by 80             *)
     (* End multiply by 80  *)
     mov cx, [X]
     mov ax, cx
    {DI = Y * LINESIZE, BX = X, coordinates admissible}
     shr ax, 1                    ; (* Faster on 286/86 machines         *)
     shr ax, 1
     add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
     add di, [VideoOfs]  ; (* Pointing at start of Active page *)
    (* Select plane to use *)
    mov dx, 03c4h
    mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
    and cl, 03h               ; (* Get Plane Bits                   *)
    shl ah, cl                ; (* Get Plane Select Value           *)
    out dx, ax
   (* End selection of plane *)
    mov es,[SegA000]
    mov al, ES:[DI]
    xor ah, ah
    mov @Result, ax
  {$else fpc}
     push eax  
     push ebx
     push ecx
     push edx
     push edi
     movzx edi,[Y]                   ; (* DI = Y coordinate                 *)
     (* Multiply by 80 start *)
     mov ebx, edi
     shl edi, 6                    ; (* Faster on 286/386/486 machines    *)
     shl ebx, 4
     add edi, ebx                   ;  (* Multiply Value by 80             *)
     (* End multiply by 80  *)
     movzx ecx, [X]
     movzx eax, [Y]
    {DI = Y * LINESIZE, BX = X, coordinates admissible}
     shr eax, 2
     add edi, eax                ; {DI = Y * LINESIZE + (X SHR 2) }
     add edi, [VideoOfs]  ; (* Pointing at start of Active page *)
    (* Select plane to use *)
    mov dx, 03c4h
    mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
    and cl, 03h               ; (* Get Plane Bits                   *)
    shl ah, cl                ; (* Get Plane Select Value           *)
    out dx, ax
   (* End selection of plane *)
    mov ax, fs:[edi+$a0000]
    mov @Result, ax
    pop edi
    pop edx
    pop ecx
    pop ebx
    pop eax
  {$endif fpc}
   end;
{$endif asmgraph}
 end;

 procedure SetVisualX(page: word); {$ifndef fpc}far;{$endif fpc}
  { 4 page supPort... }

   Procedure SetVisibleStart(AOffset: word); Assembler;
   (* Select where the left corner of the screen will be *)
   { By Matt Pritchard }
    asm
     push ax
     push cx
     push dx
{$IFDEF REGCALL}
     mov cx, ax
{$ENDIF REGCALL}
      { Wait if we are currently in a Vertical Retrace        }
     MOV     DX, INPUT_1         { Input Status #1 Register       }
   @DP_WAIT0:
     IN      AL, DX              { Get VGA status                 }
     AND     AL, VERT_RETRACE    { In Display mode yet?           }
     JNZ     @DP_WAIT0           { If Not, wait for it            }

    { Set the Start Display Address to the new page         }

     MOV     DX, CRTC_Index      { We Change the VGA Sequencer    }
     MOV     AL, START_DISP_LO   { Display Start Low Register     }
{$ifndef fpc}
     MOV     AH, BYTE PTR [AOffset] { Low 8 Bits of Start Addr    }
     OUT     DX, AX              { Set Display Addr Low           }
     MOV     AL, START_DISP_HI   { Display Start High Register    }
     MOV     AH, BYTE PTR [AOffset+1] { High 8 Bits of Start Addr }
{$else fpc}
{$IFDEF REGCALL}
    mov ah, cl
{$ELSE REGCALL}
    mov ah, byte [AOffset]
{$ENDIF REGCALL}
    out dx, ax
    mov AL, START_DISP_HI
{$IFDEF REGCALL}
    mov ah, ch
{$ELSE REGCALL}
    mov ah, byte [AOffset+1]
{$ENDIF REGCALL}
{$endif fpc}
     OUT     DX, AX              { Set Display Addr High          }
     { Wait for a Vertical Retrace to smooth out things      }

     MOV     DX, INPUT_1         { Input Status #1 Register       }

  @DP_WAIT1:
     IN      AL, DX              { Get VGA status                 }
     AND     AL, VERT_RETRACE    { Vertical Retrace Start?        }
     JZ      @DP_WAIT1           { If Not, wait for it            }
    { Now Set Display Starting Address                     }
     pop dx
     pop cx
     pop ax
  end;

{$ifdef fpc}
  {$undef asmgraph}
{$endif fpc}

  begin
    Case page of
      0: SetVisibleStart(0);
      1: SetVisibleStart(16000);
      2: SetVisibleStart(32000);
      3: SetVisibleStart(48000);
    else
      SetVisibleStart(0);
    end;
  end;

 procedure SetActiveX(page: word); {$ifndef fpc}far;{$endif fpc}
  { 4 page supPort... }
  begin
   case page of
     0: VideoOfs := 0;
     1: VideoOfs := 16000;
     2: VideoOfs := 32000;
     3: VideoOfs := 48000;
   else
     VideoOfs:=0;
   end;
  end;

 Procedure PutPixelX(X,Y: smallint; color:word); {$ifndef fpc}far;{$endif fpc}
{$ifndef asmgraph}
 var offset: word;
{$endif asmgraph}
  begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort;
    { convert to absolute coordinates and then verify clipping...}
    if ClipPixels then
     Begin
       if (X < StartXViewPort) or (X > (StartXViewPort + ViewWidth)) then
         exit;
       if (Y < StartYViewPort) or (Y > (StartYViewPort + ViewHeight)) then
         exit;
     end;
{$ifndef asmgraph}
    offset := y * 80 + x shr 2 + VideoOfs;
    PortW[$3c4] := (hi(word(FirstPlane)) shl 8) shl (x and 3)+ lo(word(FirstPlane));
    Mem[SegA000:offset] := color;
{$else asmgraph}
     asm
      push ax
      push bx
      push cx
      push dx
      push es
      push di
      mov di,[Y]                   ; (* DI = Y coordinate                 *)
      (* Multiply by 80 start *)
      mov bx, di
      shl di, 6                    ; (* Faster on 286/386/486 machines    *)
      shl bx, 4
      add di, bx                   ;  (* Multiply Value by 80             *)
      (* End multiply by 80  *)
      mov cx, [X]
      mov ax, cx
      {DI = Y * LINESIZE, BX = X, coordinates admissible}
      shr ax, 2
      add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
      add di, [VideoOfs]        ; (* Pointing at start of Active page *)
      (* Select plane to use *)
      mov dx, 03c4h
      mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
      and cl, 03h               ; (* Get Plane Bits                   *)
      shl ah, cl                ; (* Get Plane Select Value           *)
      out dx, ax
      (* End selection of plane *)
      mov es,[SegA000]
      mov ax,[Color]            ; { only lower byte is used. }
      cmp [CurrentWriteMode],XORPut   { check write mode   }
      jne @MOVMode
      mov ah,es:[di]        { read the byte...             }
      xor al,ah             { xor it and return value into AL }
    @MovMode:
      mov es:[di], al
      pop di
      pop es
      pop dx
      pop cx
      pop bx
      pop ax
    end;
{$endif asmgraph}
  end;


 Procedure DirectPutPixelX(X,Y: smallint); {$ifndef fpc}far;{$endif fpc}
 { x,y -> must be in global coordinates. No clipping. }
{$ifndef asmgraph}
 Var offset: Word;
     dummy: Byte;
 begin
   offset := y * 80 + x shr 2 + VideoOfs;
   case CurrentWriteMode of
     XorPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor xor Mem[Sega000: offset];
       end;
     OrPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor or Mem[Sega000: offset];
       end;
     AndPut:
       begin
         PortW[$3ce] := ((x and 3) shl 8) + 4;
         dummy := CurrentColor and Mem[Sega000: offset];
       end;
     NotPut: dummy := Not CurrentColor;
     else dummy := CurrentColor;
   end;
   PortW[$3c4] := (hi(word(FirstPlane)) shl 8) shl (x and 3)+ lo(word(FirstPlane));
   Mem[Sega000: offset] := Dummy;
 end;
{$else asmgraph}
{ note: still needs or/and/notput support !!!!! (JM) }
 Assembler;
 asm
   push ax
   push bx
   push cx
   push dx
   push es
   push di
{$IFDEF REGCALL}
   mov cl, al
   mov di, dx
{$ELSE REGCALL}
   mov cx, [X]
   mov ax, cx
   mov di, [Y]                   ; (* DI = Y coordinate                 *)
{$ENDIF REGCALL}
 (* Multiply by 80 start *)
   mov bx, di
   shl di, 6                    ; (* Faster on 286/386/486 machines    *)
   shl bx, 4
   add di, bx                   ;  (* Multiply Value by 80             *)
 (* End multiply by 80  *)
  {DI = Y * LINESIZE, BX = X, coordinates admissible}
   shr ax, 2
   add di, ax                ; {DI = Y * LINESIZE + (X SHR 2) }
   add di, [VideoOfs]        ; (* Pointing at start of Active page *)
 (* Select plane to use *)
   mov dx, 03c4h
   mov ax, FirstPlane        ; (* Map Mask & Plane Select Register *)
   and cl, 03h               ; (* Get Plane Bits                   *)
   shl ah, cl                ; (* Get Plane Select Value           *)
   out dx, ax
 (* End selection of plane *)
   mov es,[SegA000]
   mov ax,[CurrentColor]     ; { only lower byte is used. }
   cmp [CurrentWriteMode],XORPut   { check write mode   }
   jne @MOVMode
   mov ah,es:[di]        { read the byte...             }
   xor al,ah             { xor it and return value into AL }
 @MovMode:
   mov es:[di], al
   pop di
   pop es
   pop dx
   pop cx
   pop bx
   pop ax
 end;
{$endif asmgraph}


>>>>>>> origin/fixes_2_2

 {************************************************************************}
 {*                       General routines                               *}
 {************************************************************************}
<<<<<<< HEAD
 procedure CloseGraph;
 Begin
    If not isgraphmode then
      begin
        _graphresult := grnoinitgraph;
        exit
      end;
    if not assigned(RestoreVideoState) then
      RunError(216);
    RestoreVideoState;
    isgraphmode := false;
 end;
(*
 procedure LoadFont8x8;

   var
      r : registers;
      x,y,c : longint;
      data : array[0..127,0..7] of byte;

>>>>>>> graemeg/fixes_2_2
   begin
      r.ah:=$11;
      r.al:=$30;
      r.bh:=1;
      RealIntr($10,r);
      dosmemget(r.es,r.bp,data,sizeof(data));
      for c:=0 to 127 do
        for y:=0 to 7 do
          for x:=0 to 7 do
            if (data[c,y] and ($80 shr x))<>0 then
              DefaultFontData[chr(c),y,x]:=1
            else
              DefaultFontData[chr(c),y,x]:=0;
      { second part }
      r.ah:=$11;
      r.al:=$30;
      r.bh:=0;
      RealIntr($10,r);
      dosmemget(r.es,r.bp,data,sizeof(data));
      for c:=0 to 127 do
        for y:=0 to 7 do
          for x:=0 to 7 do
            if (data[c,y] and ($80 shr x))<>0 then
              DefaultFontData[chr(c+128),y,x]:=1
            else
              DefaultFontData[chr(c+128),y,x]:=0;
   end;
*)
  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supPorted graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }
<<<<<<< HEAD

    function Test6845(CRTCPort: Word): Boolean;
    const
      TestRegister = $0F;
    var
      OldValue, TestValue, ReadValue: Byte;
    begin
      { save the old value }
      Port[CRTCPort] := TestRegister;
      OldValue := Port[CRTCPort + 1];
      TestValue := OldValue xor $56;

      { try writing a new value to the CRTC register }
      Port[CRTCPort] := TestRegister;
      Port[CRTCPort + 1] := TestValue;

      { check if the value has been written }
      Port[CRTCPort] := TestRegister;
      ReadValue := Port[CRTCPort + 1];
      if ReadValue = TestValue then
      begin
        Test6845 := True;
        { restore old value }
        Port[CRTCPort] := TestRegister;
        Port[CRTCPort + 1] := OldValue;
      end
      else
        Test6845 := False;
    end;

   var
    HGCDetected : Boolean;
    CGADetected : Boolean; { TRUE means real CGA, *not* EGA or VGA }
    EGADetected : Boolean; { TRUE means EGA or higher (VGA) }
=======
   var
    EGADetected : Boolean;
>>>>>>> graemeg/fixes_2_2
    VGADetected : Boolean;
    mode: TModeInfo;
   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;
<<<<<<< HEAD
>>>>>>> origin/cpstrnew

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC1;
         mode.ModeName:='320 x 200 CGA C1';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC2;
         mode.ModeName:='320 x 200 CGA C2';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC3;
         mode.ModeName:='320 x 200 CGA C3';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAHi;
         mode.ModeName:='640 x 200 CGA';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA640;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA640;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA640;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA640;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA640;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA640;
         mode.XAspect := 4167;
         mode.YAspect := 10000;
         AddMode(mode);
       end;

<<<<<<< HEAD
=======
     HGCDetected := FALSE;
     CGADetected := FALSE;
     EGADetected := FALSE;
     VGADetected := FALSE;
=======


     EGADetected := FALSE;
     VGADetected := FALSE;
     { check if Hercules adapter supPorted ... }
>>>>>>> graemeg/fixes_2_2
     { check if EGA adapter supPorted...       }
     asm
       mov ah,12h
       mov bx,0FF10h
{$ifdef fpc}
       push ebx
       push ebp
<<<<<<< HEAD
       push esi
       push edi
{$endif fpc}
       int 10h              { get EGA information }
{$ifdef fpc}
       pop edi
       pop esi
       pop ebp
{$endif fpc}
       cmp bh,0ffh
{$ifdef fpc}
       pop ebx
{$endif fpc}
       jz  @noega
       mov [EGADetected],TRUE
     @noega:
     end ['EBX','EAX'];
{$ifdef logging}
     LogLn('EGA detected: '+strf(Longint(EGADetected)));
{$endif logging}
     { check if VGA adapter supPorted...       }
>>>>>>> origin/cpstrnew
     if EGADetected then
       begin
         { HACK:
           until we create Save/RestoreStateEGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.ModeNumber:=EGALo;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 200 EGA';
         mode.MaxColor := 16;
         mode.HardwarePages := 2;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
         mode.XAspect := 4500;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.ModeNumber:=EGAHi;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 350 EGA';
         mode.HardwarePages := 1;
         mode.MaxColor := 16;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
         mode.XAspect := 7750;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
<<<<<<< HEAD

>>>>>>> graemeg/cpstrnew
=======
{$ifdef logging}
       LogLn('VGA detected: '+strf(Longint(VGADetected)));
{$endif logging}
     { older than EGA? }
     if not EGADetected then
       begin
         { check if Hercules adapter supPorted ... }
         HGCDetected := Test6845($3B4);
         { check if CGA adapter supPorted ... }
         CGADetected := Test6845($3D4);
       end;
     if HGCDetected then
       begin
         { HACK:
           until we create Save/RestoreStateHGC, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.DriverNumber := HercMono;
         mode.HardwarePages := 1;
         mode.ModeNumber := HercMonoHi;
         mode.ModeName:='720 x 348 HERCULES';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 719;
         mode.MaxY := 347;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelHGC720;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelHGC720;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelHGC720;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetHGCRGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetHGCRGBPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualHGC720;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveHGC720;
         mode.InitMode := {$ifdef fpc}@{$endif}InitHGC720;
         mode.HLine := {$ifdef fpc}@{$endif}HLineHGC720;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorHGC720;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorHGC720;
         mode.XAspect := 7500;
         mode.YAspect := 10000;
         AddMode(mode);
       end;
     if CGADetected or EGADetected then
       begin
         { HACK:
           until we create Save/RestoreStateCGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         { now add all standard CGA modes...       }
         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC0;
         mode.ModeName:='320 x 200 CGA C0';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC1;
         mode.ModeName:='320 x 200 CGA C1';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC2;
         mode.ModeName:='320 x 200 CGA C2';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAC3;
         mode.ModeName:='320 x 200 CGA C3';
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := CGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := CGAHi;
         mode.ModeName:='640 x 200 CGA';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA640;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA640;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA640;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA640;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA640;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA640;
         mode.XAspect := 4167;
         mode.YAspect := 10000;
         AddMode(mode);
       end;

     if EGADetected then
       begin
         { HACK:
           until we create Save/RestoreStateEGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
         SaveVideoState := @SaveStateVGA;
         RestoreVideoState := @RestoreStateVGA;

         InitMode(mode);
         mode.ModeNumber:=EGALo;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 200 EGA';
         mode.MaxColor := 16;
         mode.HardwarePages := 2;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
         mode.XAspect := 4500;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.ModeNumber:=EGAHi;
         mode.DriverNumber := EGA;
         mode.ModeName:='640 x 350 EGA';
         mode.HardwarePages := 1;
         mode.MaxColor := 16;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
         mode.XAspect := 7750;
         mode.YAspect := 10000;
         AddMode(mode);
       end;

>>>>>>> origin/cpstrnew
     if VGADetected then
>>>>>>> graemeg/cpstrnew
       begin
         { HACK:
           until we create Save/RestoreStateEGA, we use Save/RestoreStateVGA
           with the inWindows flag enabled (so we only save the mode number
           and nothing else) }
         if not VGADetected then
           inWindows := true;
=======
=======
 var
  SavePtr : pointer;    { pointer to video state                 }
{  CrtSavePtr: pointer;}  { pointer to video state when CrtMode gets called }
  StateSize: word;      { size in 64 byte blocks for video state }
  VideoMode: byte;      { old video mode before graph mode       }
  SaveSupPorted : Boolean;    { Save/Restore video state supPorted? }


      {**************************************************************}
      {*                     DPMI Routines                          *}
      {**************************************************************}

{$IFDEF DPMI}
  RealStateSeg: word;    { Real segment of saved video state }

 Procedure SaveStateVGA; {$ifndef fpc}far;{$endif fpc}
 var
  PtrLong: longint;
  regs: TDPMIRegisters;
  begin
    SaveSupPorted := FALSE;
    SavePtr := nil;
    { Get the video mode }
    asm
      mov  ah,0fh
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
      mov  [VideoMode], al
    end ['EAX'];
    { saving/restoring video state screws up Windows (JM) }
    if inWindows then
      exit;
    { Prepare to save video state...}
    asm
      mov  ax, 1C00h       { get buffer size to save state }
      mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
{$ifdef fpc}
      push ebx
      push ebp
      push esi
      push edi
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop edi
      pop esi
      pop ebp
{$endif fpc}
      mov  [StateSize], bx
{$ifdef fpc}
      pop ebx
{$endif fpc}
      cmp  al,01ch
      jnz  @notok
      mov  [SaveSupPorted],TRUE
     @notok:
    end ['ECX','EAX'];
    if SaveSupPorted then
      begin

{$ifndef fpc}
        PtrLong:=GlobalDosAlloc(64*StateSize);  { values returned in 64-byte blocks }
{$else fpc}
        PtrLong:=Global_Dos_Alloc(64*StateSize);  { values returned in 64-byte blocks }
{$endif fpc}
        if PtrLong = 0 then
           RunError(203);
        SavePtr := pointer(longint(PtrLong and $0000ffff) shl 16);
{$ifndef fpc}
        { In FPC mode, we can't do anything with this (no far pointers)  }
        { However, we still need to keep it to be able to free the       }
        { memory afterwards. Since this data is not accessed in PM code, }
        { there's no need to save it in a seperate buffer (JM)           }
        if not assigned(SavePtr) then
           RunError(203);
{$endif fpc}
        RealStateSeg := word(PtrLong shr 16);
        FillChar(regs, sizeof(regs), #0);
        { call the real mode interrupt ... }
        regs.eax := $1C01;      { save the state buffer                   }
        regs.ecx := $07;        { Save DAC / Data areas / Hardware states }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        regs.eax := $1C02;      { restore the state buffer                }
        regs.ecx := $07;        { rest DAC / Data areas / Hardware states }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
      end;
  end;

 procedure RestoreStateVGA; {$ifndef fpc}far;{$endif fpc}
  var
   regs:TDPMIRegisters;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
{$ifdef fpc}
      push ebp
      push esi
      push edi
      push ebx
{$endif fpc}
      int  10h
{$ifdef fpc}
      pop ebx
      pop edi
      pop esi
      pop ebp
{$endif fpc}
     end ['EAX'];
     { then restore all state information }
{$ifndef fpc}
     if assigned(SavePtr) and (SaveSupPorted=TRUE) then
{$else fpc}
     { No far pointer supPort, so it's possible that that assigned(SavePtr) }
     { would return false under FPC. Just check if it's different from nil. }
     if (SavePtr <> nil) and (SaveSupPorted=TRUE) then
{$endif fpc}
      begin
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
         regs.eax := $1C02;      { restore the state buffer                }
         regs.ecx := $07;        { rest DAC / Data areas / Hardware states }
         regs.es := RealStateSeg;
         regs.ebx := 0;
         RealIntr($10,regs);
(*
{$ifndef fpc}
         if GlobalDosFree(longint(SavePtr) shr 16)<>0 then
{$else fpc}
         if Not Global_Dos_Free(longint(SavePtr) shr 16) then
{$endif fpc}
          RunError(216);

         SavePtr := nil;
*)
       end;
  end;

{$ELSE}

      {**************************************************************}
      {*                     Real mode routines                     *}
      {**************************************************************}


 Procedure SaveStateVGA; far;
  begin
    SavePtr := nil;
    SaveSupPorted := FALSE;
    { Get the video mode }
    asm
      mov  ah,0fh
      int  10h
      mov  [VideoMode], al
    end;
    { Prepare to save video state...}
    asm
      mov  ax, 1C00h       { get buffer size to save state }
      mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
      int  10h
      mov  [StateSize], bx
      cmp  al,01ch
      jnz  @notok
      mov  [SaveSupPorted],TRUE
     @notok:
    end;
    if SaveSupPorted then
      Begin
        GetMem(SavePtr, 64*StateSize); { values returned in 64-byte blocks }
        if not assigned(SavePtr) then
           RunError(203);
        asm
         mov  ax, 1C01h       { save the state buffer                   }
         mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        asm
         mov  ax, 1C02h       { save the state buffer                   }
         mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
      end;
  end;

 procedure RestoreStateVGA; far;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
      int  10h
     end;

     { then restore all state information }
     if assigned(SavePtr) and (SaveSupPorted=TRUE) then
       begin
         { restore state, according to Ralph Brown Interrupt list }
         asm
           mov  ax, 1C02h       { save the state buffer                   }
           mov  cx, 00000111b   { Save DAC / Data areas / Hardware states }
           mov  es, WORD PTR [SavePtr+2]
           mov  bx, WORD PTR [SavePtr]
           int  10h
         end;
{        done in exitproc (JM)
         FreeMem(SavePtr, 64*StateSize);}
         SavePtr := nil;
       end;
  end;
{$ENDIF DPMI}

   Procedure SetVGARGBAllPalette(const Palette:PaletteType); {$ifndef fpc}far;{$endif fpc}
    var
      c: byte;
    begin
      { wait for vertical retrace start/end}
      while (port[$3da] and $8) <> 0 do;
      while (port[$3da] and $8) = 0 do;
      If MaxColor = 16 Then
        begin
          for c := 0 to 15 do
            begin
              { translate the color number for 16 color mode }
              portb[$3c8] := toRealCols16[c];
              portb[$3c9] := palette.colors[c].red shr 2;
              portb[$3c9] := palette.colors[c].green shr 2;
              portb[$3c9] := palette.colors[c].blue shr 2;
            end
        end
      else
        begin
          portb[$3c8] := 0;
          for c := 0 to 255 do
            begin
              { no need to set port[$3c8] every time if you set the entries }
              { for successive colornumbers (JM)                            }
              portb[$3c9] := palette.colors[c].red shr 2;
              portb[$3c9] := palette.colors[c].green shr 2;
              portb[$3c9] := palette.colors[c].blue shr 2;
          end
        end;
    End;


   { VGA is never a direct color mode, so no need to check ... }
   Procedure SetVGARGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
    begin
      { translate the color number for 16 color mode }
      If MaxColor = 16 Then
        ColorNum := ToRealCols16[ColorNum];
      asm
        { on some hardware - there is a snow like effect       }
        { when changing the palette register directly          }
        { so we wait for a vertical retrace start period.      }
        push ax
        push dx
        mov dx, $03da
      @1:
        in    al, dx          { Get input status register    }
        test  al, $08         { check if in vertical retrace }
        jnz   @1              { yes, complete it             }
                              { we have to wait for the next }
                              { retrace to assure ourselves  }
                              { that we have time to complete }
                              { the DAC operation within      }
                              { the vertical retrace period   }
       @2:
        in    al, dx
        test  al, $08
        jz    @2              { repeat until vertical retrace start }

        mov dx, $03c8       { Set color register address to use }
        mov ax, [ColorNum]
        out dx, al
        inc dx              { Point to DAC registers            }
        mov ax, [RedValue]  { Get RedValue                      }
        shr ax, 2
        out dx, al
        mov ax, [GreenValue]{ Get RedValue                      }
        shr ax, 2
        out dx, al
        mov ax, [BlueValue] { Get RedValue                      }
        shr ax, 2
        out dx, al
        pop dx
        pop ax
      end
    End;


   { VGA is never a direct color mode, so no need to check ... }
  Procedure GetVGARGBPalette(ColorNum: smallint; Var
      RedValue, GreenValue, BlueValue : smallint); {$ifndef fpc}far;{$endif fpc}
   begin
     If MaxColor = 16 Then
       ColorNum := ToRealCols16[ColorNum];
     Port[$03C7] := ColorNum;
     { we must convert to lsb values... because the vga uses the 6 msb bits }
     { which is not compatible with anything.                               }
     RedValue := smallint(Port[$3C9]) shl 2;
     GreenValue := smallint(Port[$3C9]) shl 2;
     BlueValue := smallint(Port[$3C9]) shl 2;
   end;


 {************************************************************************}
 {*                       VESA related routines                          *}
 {************************************************************************}
{$I vesa.inc}

 {************************************************************************}
 {*                       General routines                               *}
 {************************************************************************}
 procedure CloseGraph;
 Begin
    If not isgraphmode then
      begin
        _graphresult := grnoinitgraph;
        exit
      end;
    if not assigned(RestoreVideoState) then
      RunError(216);
    RestoreVideoState;
    isgraphmode := false;
 end;
(*
 procedure LoadFont8x8;

   var
      r : registers;
      x,y,c : longint;
      data : array[0..127,0..7] of byte;

   begin
      r.ah:=$11;
      r.al:=$30;
      r.bh:=1;
      RealIntr($10,r);
      dosmemget(r.es,r.bp,data,sizeof(data));
      for c:=0 to 127 do
        for y:=0 to 7 do
          for x:=0 to 7 do
            if (data[c,y] and ($80 shr x))<>0 then
              DefaultFontData[chr(c),y,x]:=1
            else
              DefaultFontData[chr(c),y,x]:=0;
      { second part }
      r.ah:=$11;
      r.al:=$30;
      r.bh:=0;
      RealIntr($10,r);
      dosmemget(r.es,r.bp,data,sizeof(data));
      for c:=0 to 127 do
        for y:=0 to 7 do
          for x:=0 to 7 do
            if (data[c,y] and ($80 shr x))<>0 then
              DefaultFontData[chr(c+128),y,x]:=1
            else
              DefaultFontData[chr(c+128),y,x]:=0;
   end;
*)
  function QueryAdapterInfo:PModeInfo;
  { This routine returns the head pointer to the list }
  { of supPorted graphics modes.                      }
  { Returns nil if no graphics mode supported.        }
  { This list is READ ONLY!                           }
   var
    EGADetected : Boolean;
    VGADetected : Boolean;
    mode: TModeInfo;
   begin
     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;


     EGADetected := FALSE;
     VGADetected := FALSE;
     { check if Hercules adapter supPorted ... }
     { check if EGA adapter supPorted...       }
     asm
       mov ah,12h
       mov bx,0FF10h
{$ifdef fpc}
       push ebx
       push ebp
>>>>>>> origin/fixes_2_2
       push esi
       push edi
{$endif fpc}
       int 10h              { get EGA information }
{$ifdef fpc}
       pop edi
       pop esi
       pop ebp
{$endif fpc}
       cmp bh,0ffh
{$ifdef fpc}
       pop ebx
{$endif fpc}
       jz  @noega
       mov [EGADetected],TRUE
     @noega:
     end ['EBX','EAX'];
{$ifdef logging}
     LogLn('EGA detected: '+strf(Longint(EGADetected)));
{$endif logging}
     { check if VGA adapter supPorted...       }
     if EGADetected then
       begin
        asm
         mov ax,1a00h
{$ifdef fpc}
         push ebp
         push esi
         push edi
         push ebx
{$endif fpc}
         int 10h            { get display combination code...}
{$ifdef fpc}
         pop ebx
         pop edi
         pop esi
         pop ebp
{$endif fpc}
         cmp al,1ah         { check if supPorted...          }
         jne @novga
         { now check if this is the ATI EGA }
         mov ax,1c00h       { get state size for save...     }
                            { ... all imPortant data         }
         mov cx,07h
{$ifdef fpc}
         push ebp
         push esi
         push edi
         push ebx
{$endif fpc}
         int 10h
{$ifdef fpc}
         pop ebx
         pop edi
         pop esi
         pop ebp
{$endif fpc}
         cmp al,1ch         { success?                       }
         jne @novga
         mov [VGADetected],TRUE
        @novga:
        end ['ECX','EAX'];
       end;
{$ifdef logging}
       LogLn('VGA detected: '+strf(Longint(VGADetected)));
{$endif logging}
     if VGADetected then
       begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         SaveVideoState := @SaveStateVGA;
{$ifdef logging}
         LogLn('Setting VGA SaveVideoState to '+strf(longint(SaveVideoState)));
{$endif logging}
         RestoreVideoState := @RestoreStateVGA;
{$ifdef logging}
         LogLn('Setting VGA RestoreVideoState to '+strf(longint(RestoreVideoState)));
{$endif logging}

<<<<<<< HEAD
<<<<<<< HEAD
         { now add all standard MCGA modes...       }
         { yes, most of these are the same as the CGA modes; this is TP7
           compatible }
         InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         FillCommonCGA320(mode);
         mode.DriverNumber := MCGA;
         mode.ModeNumber := MCGAC0;
         mode.ModeName:='320 x 200 CGA C0'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA320(mode);
         mode.DriverNumber := MCGA;
         mode.ModeNumber := MCGAC1;
         mode.ModeName:='320 x 200 CGA C1'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA320(mode);
         mode.DriverNumber := MCGA;
         mode.ModeNumber := MCGAC2;
         mode.ModeName:='320 x 200 CGA C2'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA320(mode);
         mode.DriverNumber := MCGA;
         mode.ModeNumber := MCGAC3;
         mode.ModeName:='320 x 200 CGA C3'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         AddMode(mode);

         InitMode(mode);
         FillCommonCGA640(mode);
         mode.DriverNumber := MCGA;
         mode.ModeNumber := MCGAMed;
         mode.ModeName:='640 x 200 CGA'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         mode.DriverNumber := MCGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := MCGAC0;
         mode.ModeName:='320 x 200 CGA C0'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C0;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := MCGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := MCGAC1;
         mode.ModeName:='320 x 200 CGA C1'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C1;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := MCGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := MCGAC2;
         mode.ModeName:='320 x 200 CGA C2'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C2;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := MCGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := MCGAC3;
         mode.ModeName:='320 x 200 CGA C3'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.MaxColor := 4;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA320C3;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA320;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA320;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := MCGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := MCGAMed;
         mode.ModeName:='640 x 200 CGA'; { yes, it says 'CGA' even for the MCGA driver; this is TP7 compatible }
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelCGA640;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelCGA640;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelCGA640;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitCGA640;
         mode.HLine := {$ifdef fpc}@{$endif}HLineCGA640;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorCGA640;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorCGA640;
         mode.XAspect := 4167;
         mode.YAspect := 10000;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         AddMode(mode);

         InitMode(mode);
         mode.DriverNumber := MCGA;
         mode.HardwarePages := 0;
         mode.ModeNumber := MCGAHi;
         mode.ModeName:='640 x 480 MCGA';
         mode.MaxColor := 2;
         mode.PaletteSize := 16;
         mode.DirectColor := FALSE;
         mode.MaxX := 639;
         mode.MaxY := 479;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelMCGA640;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelMCGA640;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelMCGA640;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}InitMCGA640;
         mode.HLine := {$ifdef fpc}@{$endif}HLineMCGA640;
         mode.SetBkColor := {$ifdef fpc}@{$endif}SetBkColorMCGA640;
         mode.GetBkColor := {$ifdef fpc}@{$endif}GetBkColorMCGA640;
         mode.XAspect := 10000;
         mode.YAspect := 10000;
         AddMode(mode);


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         InitMode(mode);
         { now add all standard VGA modes...       }
         mode.DriverNumber:= LowRes;
         mode.HardwarePages:= 0;
         mode.ModeNumber:=0;
         mode.ModeName:='320 x 200 VGA';
         mode.MaxColor := 256;
         mode.PaletteSize := mode.MaxColor;
         mode.DirectColor := FALSE;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel320;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel320;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel320;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
<<<<<<< HEAD
<<<<<<< HEAD
=======
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual320;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive320;
>>>>>>> graemeg/fixes_2_2
=======
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual320;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive320;
>>>>>>> origin/fixes_2_2
         mode.InitMode := {$ifdef fpc}@{$endif}Init320;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
       end;

     if VGADetected then
       begin
         SaveVideoState := @SaveStateVGA;
{$ifdef logging}
         LogLn('Setting VGA SaveVideoState to '+strf(longint(SaveVideoState)));
{$endif logging}
         RestoreVideoState := @RestoreStateVGA;
{$ifdef logging}
         LogLn('Setting VGA RestoreVideoState to '+strf(longint(RestoreVideoState)));
{$endif logging}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
         { now add all standard VGA modes...       }
         InitMode(mode);
         mode.DriverNumber:= LowRes;
         mode.ModeNumber:=1;
         mode.HardwarePages := 3; { 0..3 }
         mode.ModeName:='320 x 200 ModeX';
         mode.MaxColor := 256;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 319;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixelX;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixelX;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixelX;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualX;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveX;
         mode.InitMode := {$ifdef fpc}@{$endif}InitModeX;
         mode.XAspect := 8333;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
         FillCommonEGAVGA16(mode);
         mode.ModeNumber:=VGALo;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 200 EGA'; { yes, it says 'EGA' even for the VGA driver; this is TP7 compatible }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
         mode.ModeNumber:=VGALo;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 200 VGA';
>>>>>>> graemeg/fixes_2_2
=======
         mode.ModeNumber:=VGALo;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 200 VGA';
>>>>>>> origin/fixes_2_2
         mode.MaxColor := 16;
         mode.HardwarePages := 2;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.HardwarePages := 3;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
=======
=======
>>>>>>> origin/fixes_2_2
         mode.MaxX := 639;
         mode.MaxY := 199;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual200;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive200;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x200x16;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         mode.XAspect := 4500;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
         FillCommonEGAVGA16(mode);
         mode.ModeNumber:=VGAMed;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 350 EGA'; { yes, it says 'EGA' even for the VGA driver; this is TP7 compatible }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
         mode.ModeNumber:=VGAMed;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 350 VGA';
>>>>>>> graemeg/fixes_2_2
=======
         mode.ModeNumber:=VGAMed;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 350 VGA';
>>>>>>> origin/fixes_2_2
         mode.HardwarePages := 1;
         mode.MaxColor := 16;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.HardwarePages := 1;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
=======
=======
>>>>>>> origin/fixes_2_2
         mode.MaxX := 639;
         mode.MaxY := 349;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x350x16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual350;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive350;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         mode.XAspect := 7750;
         mode.YAspect := 10000;
         AddMode(mode);

         InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
         FillCommonEGAVGA16(mode);
         mode.ModeNumber:=VGAHi;
         mode.DriverNumber := VGA;
         mode.ModeName:='640 x 480 VGA';
         mode.MaxX := 639;
         mode.MaxY := 479;
         mode.HardwarePages := 0;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
         mode.ModeNumber:=VGAHi;
         mode.DriverNumber := VGA;
         mode.HardwarePages := 0;
         mode.ModeName:='640 x 480 VGA';
         mode.MaxColor := 16;
         mode.DirectColor := FALSE;
         mode.PaletteSize := mode.MaxColor;
         mode.MaxX := 639;
         mode.MaxY := 479;
         mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16;
         mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16;
         mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16;
         mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVGARGBPalette;
         mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVGARGBPalette;
         mode.SetAllPalette := {$ifdef fpc}@{$endif}SetVGARGBAllPalette;
         mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x16;
         mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisual480;
         mode.SetActivePage := {$ifdef fpc}@{$endif}SetActive480;
         mode.HLine := {$ifdef fpc}@{$endif}HLine16;
         mode.VLine := {$ifdef fpc}@{$endif}VLine16;
         mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLine16;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         mode.XAspect := 10000;
         mode.YAspect := 10000;
         AddMode(mode);
       end;

     { check if VESA adapter supPorted...      }
{$ifndef noSupPortVESA}
     hasVesa := getVesaInfo(VESAInfo);
     { VBE Version v1.00 is unstable, therefore }
     { only VBE v1.1 and later are supported.   }
     if (hasVESA=TRUE) and (VESAInfo.Version <= $0100) then
       hasVESA := False;
{$else noSupPortVESA}
     hasVESA := false;
{$endif noSupPortVESA}
     if hasVesa then
       begin
         { We have to set and restore the entire VESA state }
         { otherwise, if we use the VGA BIOS only function  }
         { there might be a crash under DPMI, such as in the}
         { ATI Mach64                                       }
         SaveVideoState := @SaveStateVESA;
{$ifdef logging}
         LogLn('Setting SaveVideoState to '+strf(longint(SaveVideoState)));
{$endif logging}
         RestoreVideoState := @RestoreStateVESA;
{$ifdef logging}
         LogLn('Setting RestoreVideoState to '+strf(longint(RestoreVideoState)));
{$endif logging}
         { now check all supported modes...}
         if SearchVESAModes(m320x200x32k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA32k(mode);
             mode.ModeNumber:=m320x200x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='320 x 200 VESA';
             mode.MaxX := 319;
             mode.MaxY := 199;
             mode.InitMode := {$ifdef fpc}@{$endif}Init320x200x32k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m320x200x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='320 x 200 VESA';
             mode.MaxColor := 32768;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 319;
             mode.MaxY := 199;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init320x200x32k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 8333;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m320x200x64k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA64k(mode);
             mode.ModeNumber:=m320x200x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='320 x 200 VESA';
             mode.MaxX := 319;
             mode.MaxY := 199;
             mode.InitMode := {$ifdef fpc}@{$endif}Init320x200x64k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m320x200x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='320 x 200 VESA';
             mode.MaxColor := 65536;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 319;
             mode.MaxY := 199;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init320x200x64k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 8333;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m640x400x256) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA256(mode);
             mode.ModeNumber:=m640x400x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 400 VESA';
             mode.MaxX := 639;
             mode.MaxY := 399;
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x400x256;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m640x400x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 400 VESA';
             mode.MaxColor := 256;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := FALSE;
             mode.MaxX := 639;
             mode.MaxY := 399;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA256;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA256;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA256;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x400x256;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.hline := {$ifdef fpc}@{$endif}HLineVESA256;
             mode.vline := {$ifdef fpc}@{$endif}VLineVESA256;
             mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLineVESA256;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 8333;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m640x480x256) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA256(mode);
             mode.ModeNumber:=m640x480x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 480 VESA';
             mode.MaxX := 639;
             mode.MaxY := 479;
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x256;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m640x480x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 480 VESA';
             mode.MaxColor := 256;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 639;
             mode.MaxY := 479;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA256;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA256;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA256;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x256;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.hline := {$ifdef fpc}@{$endif}HLineVESA256;
             mode.vline := {$ifdef fpc}@{$endif}VLineVESA256;
             mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLineVESA256;
             mode.PatternLine := {$ifdef fpc}@{$endif}PatternLineVESA256;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m640x480x32k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA32k(mode);
             mode.ModeNumber:=m640x480x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 480 VESA';
             mode.MaxX := 639;
             mode.MaxY := 479;
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x32k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m640x480x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 480 VESA';
             mode.MaxColor := 32768;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 639;
             mode.MaxY := 479;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x32k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m640x480x64k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA64k(mode);
             mode.ModeNumber:=m640x480x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 480 VESA';
             mode.MaxX := 639;
             mode.MaxY := 479;
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x64k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m640x480x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='640 x 480 VESA';
             mode.MaxColor := 65536;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 639;
             mode.MaxY := 479;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init640x480x64k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m800x600x16) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA16(mode);
             mode.ModeNumber:=m800x600x16;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x16;
<<<<<<< HEAD
=======
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.HLine := {$ifdef fpc}@{$endif}HLineVESA16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m800x600x16;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxColor := 16;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.DirectColor := FALSE;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA16;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA16;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA16;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x16;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m800x600x256) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA256(mode);
             mode.ModeNumber:=m800x600x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x256;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m800x600x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxColor := 256;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := FALSE;
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA256;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA256;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA256;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x256;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.hline := {$ifdef fpc}@{$endif}HLineVESA256;
             mode.vline := {$ifdef fpc}@{$endif}VLineVESA256;
             mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLineVESA256;
             mode.PatternLine := {$ifdef fpc}@{$endif}PatternLineVESA256;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m800x600x32k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA32k(mode);
             mode.ModeNumber:=m800x600x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x32k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m800x600x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxColor := 32768;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x32k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m800x600x64k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA64k(mode);
             mode.ModeNumber:=m800x600x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x64k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m800x600x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='800 x 600 VESA';
             mode.MaxColor := 65536;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 799;
             mode.MaxY := 599;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init800x600x64k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1024x768x16) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA16(mode);
             mode.ModeNumber:=m1024x768x16;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x16;
<<<<<<< HEAD
=======
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.HLine := {$ifdef fpc}@{$endif}HLineVESA16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1024x768x16;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxColor := 16;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := FALSE;
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA16;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA16;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA16;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x16;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1024x768x256) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA256(mode);
             mode.ModeNumber:=m1024x768x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x256;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1024x768x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxColor := 256;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := FALSE;
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA256;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA256;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA256;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x256;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.vline := {$ifdef fpc}@{$endif}VLineVESA256;
             mode.hline := {$ifdef fpc}@{$endif}HLineVESA256;
             mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLineVESA256;
             mode.PatternLine := {$ifdef fpc}@{$endif}PatternLineVESA256;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1024x768x32k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA32k(mode);
             mode.ModeNumber:=m1024x768x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x32k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1024x768x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxColor := 32768;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.DirectColor := TRUE;
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x32k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1024x768x64k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA64k(mode);
             mode.ModeNumber:=m1024x768x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x64k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1024x768x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1024 x 768 VESA';
             mode.MaxColor := 65536;
             mode.DirectColor := TRUE;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 1023;
             mode.MaxY := 767;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1024x768x64k;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1280x1024x16) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA16(mode);
             mode.ModeNumber:=m1280x1024x16;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x16;
<<<<<<< HEAD
=======
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.HLine := {$ifdef fpc}@{$endif}HLineVESA16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1280x1024x16;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxColor := 16;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.DirectColor := FALSE;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA16;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA16;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA16;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x16;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1280x1024x256) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA256(mode);
             mode.ModeNumber:=m1280x1024x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x256;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1280x1024x256;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxColor := 256;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.DirectColor := FALSE;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA256;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA256;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA256;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x256;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
{$ifdef fpc}
             mode.SetAllPalette := @SetVESARGBAllPalette;
{$endif fpc}
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
             mode.vline := {$ifdef fpc}@{$endif}VLineVESA256;
             mode.hline := {$ifdef fpc}@{$endif}HLineVESA256;
             mode.GetScanLine := {$ifdef fpc}@{$endif}GetScanLineVESA256;
             mode.PatternLine := {$ifdef fpc}@{$endif}PatternLineVESA256;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1280x1024x32k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA32k(mode);
             mode.ModeNumber:=m1280x1024x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x32k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1280x1024x32k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxColor := 32768;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.DirectColor := TRUE;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x32k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
         if SearchVESAModes(m1280x1024x64k) then
           begin
             InitMode(mode);
<<<<<<< HEAD
<<<<<<< HEAD
             FillCommonVESA64k(mode);
             mode.ModeNumber:=m1280x1024x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x64k;
=======
=======
>>>>>>> origin/fixes_2_2
             mode.ModeNumber:=m1280x1024x64k;
             mode.DriverNumber := VESA;
             mode.ModeName:='1280 x 1024 VESA';
             mode.MaxColor := 65536;
             { the ModeInfo is automatically set if the mode is supPorted }
             { by the call to SearchVESAMode.                             }
             mode.HardwarePages := VESAModeInfo.NumberOfPages;
             mode.DirectColor := TRUE;
             mode.PaletteSize := mode.MaxColor;
             mode.MaxX := 1279;
             mode.MaxY := 1023;
             mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixVESA32kOr64k;
             mode.PutPixel:={$ifdef fpc}@{$endif}PutPixVESA32kOr64k;
             mode.GetPixel:={$ifdef fpc}@{$endif}GetPixVESA32kOr64k;
             mode.InitMode := {$ifdef fpc}@{$endif}Init1280x1024x64k;
             mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetVESARGBPalette;
             mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetVESARGBPalette;
             mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualVESA;
             mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveVESA;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             mode.XAspect := 10000;
             mode.YAspect := 10000;
             AddMode(mode);
           end;
       end;
   end;

var
  go32exitsave: pointer;

procedure freeSaveStateBuffer; {$ifndef fpc}far; {$endif}
begin
  if savePtr <> nil then
    begin
{$ifdef dpmi}
{$ifndef fpc}
      if GlobalDosFree(longint(SavePtr) shr 16)<>0 then;
{$else fpc}
      if Not Global_Dos_Free(longint(SavePtr) shr 16) then;
{$endif fpc}
{$else dpmi}
      FreeMem(SavePtr, 64*StateSize);
{$endif dpmi}
      SavePtr := nil;
  end;
  exitproc := go32exitsave;
end;

begin
  { must be done *before* initialize graph is called, because the save }
  { buffer can be used in the normal exit_proc (which is hooked in     }
  { initializegraph and as such executed first) (JM)                   }
  go32exitsave := exitproc;
  exitproc := @freeSaveStateBuffer;
  { windows screws up the display if the savestate/restore state  }
  { stuff is used (or uses an abnormal amount of cpu time after   }
  { such a problem has exited), so detect its presense and do not }
  { use those functions if it's running. I'm really tired of      }
  { working around Windows bugs :( (JM)                           }
  asm
    mov  ax,$160a
{$ifdef fpc}
    push ebp
    push esi
    push edi
    push ebx
{$endif fpc}
    int  $2f
{$ifdef fpc}
    pop ebx
    pop edi
    pop esi
    pop ebp
{$endif fpc}
    test ax,ax
    sete al
    mov  inWindows,al
  end ['EAX'];
  InitializeGraph;
end.
