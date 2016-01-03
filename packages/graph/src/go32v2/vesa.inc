{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Carl Eric Codere

    This include implements VESA basic access.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
type

  palrec = packed record              { record used for set/get DAC palette }
       blue, green, red, align: byte;
  end;

const
  { VESA attributes     }
  attrSwitchDAC        = $01;    { DAC is switchable           (1.2)   }
  attrNotVGACompatible = $02;    { Video is NOT VGA compatible (2.0)   }
  attrSnowCheck        = $04;    { Video must use snow checking(2.0)   }

  { mode attribute bits }
  modeAvail          = $01;      { Hardware supports this mode (1.0)   }
  modeExtendInfo     = $02;      { Extended information        (1.0)   }
  modeBIOSSupport    = $04;      { TTY BIOS Support            (1.0)   }
  modeColor          = $08;      { This is a color mode        (1.0)   }
  modeGraphics       = $10;      { This is a graphics mode     (1.0)   }
  modeNotVGACompatible = $20;    { this mode is NOT I/O VGA compatible (2.0)}
  modeNoWindowed     = $40;      { This mode does not support Windows (2.0) }
  modeLinearBuffer   = $80;      { This mode supports linear buffers  (2.0) }

  { window attributes }
  winSupported       = $01;
  winReadable        = $02;
  winWritable        = $04;

  { memory model }
  modelText          = $00;
  modelCGA           = $01;
  modelHerc          = $02;
  model4plane        = $03;
  modelPacked        = $04;
  modelModeX         = $05;
  modelRGB           = $06;
  modelYUV           = $07;

{$ifndef dpmi}
{$i vesah.inc}
{ otherwise it's already included in graph.pp }
{$endif dpmi}

var

  BytesPerLine: word;              { Number of bytes per scanline }
  YOffset : word;                  { Pixel offset for VESA page flipping }

  { window management }
  ReadWindow : byte;      { Window number for reading. }
  WriteWindow: byte;      { Window number for writing. }
  winReadSeg : word;      { Address of segment for read  }
  winWriteSeg: word;      { Address of segment for writes}
  CurrentReadBank : smallint; { active read bank          }
  CurrentWriteBank: smallint; { active write bank         }

  BankShift : word;       { address to shift by when switching banks. }

  { linear mode specific stuff }
  InLinear  : boolean;    { true if in linear mode }
  LinearPageOfs : longint; { offset used to set active page }
  FrameBufferLinearAddress : longint;

  ScanLines: word;        { maximum number of scan lines for mode }

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
function hexstr(val : longint;cnt : byte) : string;
const
  HexTbl : array[0..15] of char='0123456789ABCDEF';
var
  i : longint;
begin
  hexstr[0]:=char(cnt);
  for i:=cnt downto 1 do
   begin
     hexstr[i]:=hextbl[val and $f];
     val:=val shr 4;
   end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$IFDEF DPMI}

  function getVESAInfo(var VESAInfo: TVESAInfo) : boolean;
   var
    ptrlong : longint;
    VESAPtr : ^TVESAInfo;
    st : string[4];
    regs : TDPMIRegisters;
{$ifndef fpc}
    ModeSel: word;
    offs: longint;
{$endif fpc}
    { added... }
    modelist: PmodeList;
    i: longint;
    RealSeg : word;
   begin
    { Allocate real mode buffer }
{$ifndef fpc}
    Ptrlong:=GlobalDosAlloc(sizeof(TVESAInfo));
    { Get selector value }
    VESAPtr := pointer(Ptrlong shl 16);
{$else fpc}
    Ptrlong:=Global_Dos_Alloc(sizeof(TVESAInfo));
    New(VESAPtr);
{$endif fpc}
    { Get segment value }
    RealSeg := word(Ptrlong shr 16);
    if not assigned(VESAPtr) then
      RunError(203);
    FillChar(regs, sizeof(regs), #0);

    { Get VESA Mode information ... }
    regs.eax := $4f00;
    regs.es := RealSeg;
    regs.edi := $00;
    RealIntr($10, regs);
{$ifdef fpc}
   { no far pointer support in FPC yet, so move the vesa info into a memory }
   { block in the DS slector space (JM)                                     }
    dosmemget(RealSeg,0,VesaPtr^,SizeOf(TVESAInfo));
{$endif fpc}
    St:=Vesaptr^.signature;
    if st<>'VESA' then
     begin
{$ifdef logging}
         LogLn('No VESA detected.');
{$endif logging}
         getVesaInfo := FALSE;
{$ifndef fpc}
         GlobalDosFree(word(PtrLong and $ffff));
{$else fpc}
         If not Global_Dos_Free(word(PtrLong and $ffff)) then
           RunError(216);
         { also free the extra allocated buffer }
         Dispose(VESAPtr);
{$endif fpc}
         exit;
     end
    else
      getVesaInfo := TRUE;

{$ifndef fpc}
    { The mode pointer buffer points to a real mode memory }
    { Therefore steps to get the modes:                    }
    {  1. Allocate Selector and SetLimit to max number of  }
    {     of possible modes.                               }
    ModeSel := AllocSelector(0);
    SetSelectorLimit(ModeSel, 256*sizeof(word));

    {  2. Set Selector linear address to the real mode pointer }
    {     returned.                                            }
    offs := longint(longint(VESAPtr^.ModeList) shr 16) shl 4;
   {shouldn't the OR in the next line be a + ?? (JM)}
    offs :=  offs OR (Longint(VESAPtr^.ModeList) and $ffff);
    SetSelectorBase(ModeSel, offs);

     { copy VESA mode information to a protected mode buffer and }
     { then free the real mode buffer...                         }
     Move(VESAPtr^, VESAInfo, sizeof(VESAInfo));
     GlobalDosFree(word(PtrLong and $ffff));

    { ModeList points to the mode list     }
    { We must copy it somewhere...         }
    ModeList := Ptr(ModeSel, 0);

{$else fpc}
    { No far pointer support, so the Ptr(ModeSel, 0) doesn't work.     }
    { Immediately copy everything to a buffer in the DS selector space }
     New(ModeList);
    { The following may copy data from outside the VESA buffer, but it   }
    { shouldn't get past the 1MB limit, since that would mean the buffer }
    { has been allocated in the BIOS or high memory region, which seems  }
    { impossible to me (JM)}
     DosMemGet(word(longint(VESAPtr^.ModeList) shr 16),
        word(longint(VESAPtr^.ModeList) and $ffff), ModeList^,256*sizeof(word));

     { copy VESA mode information to a protected mode buffer and }
     { then free the real mode buffer...                         }
     Move(VESAPtr^, VESAInfo, sizeof(VESAInfo));
     If not Global_Dos_Free(word(PtrLong and $ffff)) then
       RunError(216);
     Dispose(VESAPtr);
{$endif fpc}

    i:=0;
    new(VESAInfo.ModeList);
    while ModeList^[i]<> $ffff do
     begin
{$ifdef logging}
      LogLn('Found mode $'+hexstr(ModeList^[i],4));
{$endif loggin}
      VESAInfo.ModeList^[i] := ModeList^[i];
      Inc(i);
     end;
    VESAInfo.ModeList^[i]:=$ffff;
    { Free the temporary selector used to get mode information }
{$ifdef logging}
    LogLn(strf(i) + ' modes found.');
{$endif logging}
{$ifndef fpc}
    FreeSelector(ModeSel);
{$else fpc}
    Dispose(ModeList);
{$endif fpc}
   end;

  function getVESAModeInfo(var ModeInfo: TVESAModeInfo;mode:word):boolean;
   var
    Ptr: longint;
{$ifndef fpc}
    VESAPtr : ^TVESAModeInfo;
{$endif fpc}
    regs : TDPMIRegisters;
    RealSeg: word;
   begin
    { Alllocate real mode buffer }
{$ifndef fpc}
    Ptr:=GlobalDosAlloc(sizeof(TVESAModeInfo));
    { get the selector value }
    VESAPtr := pointer(longint(Ptr shl 16));
    if not assigned(VESAPtr) then
      RunError(203);
{$else fpc}
    Ptr:=Global_Dos_Alloc(sizeof(TVESAModeInfo));
{$endif fpc}
    { get the segment value }
    RealSeg := word(Ptr shr 16);
<<<<<<< HEAD
<<<<<<< HEAD
    { we have to init everything to zero, since VBE < 1.1  }
    { may not setup fields correctly.                      }
{$ifndef fpc}
    FillChar(VESAPtr^, sizeof(ModeInfo), #0);
{$else fpc}
    DosMemFillChar(RealSeg, 0, sizeof(ModeInfo), #0);
{$endif fpc}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { setup interrupt registers }
    FillChar(regs, sizeof(regs), #0);
    { call VESA mode information...}
    regs.eax := $4f01;
    regs.es := RealSeg;
    regs.edi := $00;
    regs.ecx := mode;
    RealIntr($10, regs);
    if word(regs.eax) <> $4f then
      getVESAModeInfo := FALSE
    else
      getVESAModeInfo := TRUE;
    { copy to protected mode buffer ... }
{$ifndef fpc}
    Move(VESAPtr^, ModeInfo, sizeof(ModeInfo));
{$else fpc}
    DosMemGet(RealSeg,0,ModeInfo,sizeof(ModeInfo));
{$endif fpc}
    { free real mode memory  }
{$ifndef fpc}
    GlobalDosFree(Word(Ptr and $ffff));
{$else fpc}
    If not Global_Dos_Free(Word(Ptr and $ffff)) then
      RunError(216);
{$endif fpc}
   end;

{$ELSE}
  function getVESAInfo(var VESAInfo: TVESAInfo) : boolean; assembler;
  asm
       mov ax,4F00h
       les di,VESAInfo
       int 10h
       sub ax,004Fh  {make sure we got 004Fh back}
       cmp ax,1
       sbb al,al
       cmp word ptr es:[di],'V'or('E'shl 8)  {signature should be 'VESA'}
       jne @@ERR
       cmp word ptr es:[di+2],'S'or('A'shl 8)
       je @@X
     @@ERR:
       mov al,0
     @@X:
  end;


  function getVESAModeInfo(var ModeInfo: TVESAModeInfo;mode:word):boolean;assembler;
   asm
     mov ax,4F01h
     mov cx,mode
     les di,ModeInfo
     int 10h
     sub ax,004Fh   {make sure it's 004Fh}
     cmp ax,1
     sbb al,al
   end;

{$ENDIF}

  function SearchVESAModes(mode: Word): boolean;
  {********************************************************}
  { Searches for a specific DEFINED vesa mode. If the mode }
  { is not available for some reason, then returns FALSE   }
  { otherwise returns TRUE.                                }
  {********************************************************}
   var
     i: word;
     ModeSupported : Boolean;
    begin
      i:=0;
      { let's assume it's not available ... }
      ModeSupported := FALSE;
      { This is a STUB VESA implementation  }
      if VESAInfo.ModeList^[0] = $FFFF then exit;
      repeat
        if VESAInfo.ModeList^[i] = mode then
         begin
            { we found it, the card supports this mode... }
            ModeSupported := TRUE;
            break;
         end;
        Inc(i);
      until VESAInfo.ModeList^[i] = $ffff;
      { now check if the hardware supports it... }
      If ModeSupported then
        begin
          { we have to init everything to zero, since VBE < 1.1  }
          { may not setup fields correctly.                      }
<<<<<<< HEAD
<<<<<<< HEAD
          { bugfix: for DPMI this is now done in GetVESAModeInfo }
{$IFNDEF DPMI}
          FillChar(VESAModeInfo, sizeof(VESAModeInfo), #0);
{$ENDIF}
=======
          FillChar(VESAModeInfo, sizeof(VESAModeInfo), #0);
>>>>>>> graemeg/fixes_2_2
=======
          FillChar(VESAModeInfo, sizeof(VESAModeInfo), #0);
>>>>>>> origin/fixes_2_2
          If GetVESAModeInfo(VESAModeInfo, Mode) And
             ((VESAModeInfo.attr and modeAvail) <> 0) then
            ModeSupported := TRUE
          else
            ModeSupported := FALSE;
        end;
       SearchVESAModes := ModeSupported;
    end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure SetBankIndex(win: byte; BankNr: smallint);
{I don't know why but the previous assembler version changed by some mechanism
 unknown to me some places in memory what lead to changing some variables not
 belonging to this procedure (Laaca)}
var r:TDPMIregisters;
begin
  r.ax:=$4f05;
  r.bx:=win;
  r.dx:=BankNr;
  RealIntr($10,r);
end;
=======
=======
>>>>>>> origin/fixes_2_2


  procedure SetBankIndex(win: byte; BankNr: smallint); assembler;
   asm
{$IFDEF REGCALL}
     mov  bl, al
{$ELSE REGCALL}
     mov  bl,[Win]
{$ENDIF REGCALL}
     mov  ax,4f05h
     mov  bh,00h
{$IFNDEF REGCALL}
     mov  dx,[BankNr]
{$ENDIF REGCALL}
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
   end ['EDX','EBX','EAX'];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  {********************************************************}
  { There are two routines for setting banks. This may in  }
  { in some cases optimize a bit some operations, if the   }
  { hardware supports it, because one window is used for   }
  { reading and one window is used for writing.            }
  {********************************************************}
  procedure SetReadBank(BankNr: smallint);
   begin
     { check if this is the current bank... if so do nothing. }
     if BankNr = CurrentReadBank then exit;
{$ifdef logging}
{     LogLn('Setting read bank to '+strf(BankNr));}
{$endif logging}
     CurrentReadBank := BankNr;          { save current bank number     }
     BankNr := BankNr shl BankShift;     { adjust to window granularity }
     { we set both banks, since one may read only }
     SetBankIndex(ReadWindow, BankNr);
     { if the hardware supports only one window }
     { then there is only one single bank, so   }
     { update both bank numbers.                }
     if ReadWindow = WriteWindow then
       CurrentWriteBank := CurrentReadBank;
   end;

  procedure SetWriteBank(BankNr: smallint);
   begin
     { check if this is the current bank... if so do nothing. }
     if BankNr = CurrentWriteBank then exit;
{$ifdef logging}
{     LogLn('Setting write bank to '+strf(BankNr));}
{$endif logging}
     CurrentWriteBank := BankNr;          { save current bank number     }
     BankNr := BankNr shl BankShift;     { adjust to window granularity }
     { we set both banks, since one may read only }
     SetBankIndex(WriteWindow, BankNr);
     { if the hardware supports only one window }
     { then there is only one single bank, so   }
     { update both bank numbers.                }
     if ReadWindow = WriteWindow then
       CurrentReadBank := CurrentWriteBank;
   end;

 {************************************************************************}
 {*                     8-bit pixels VESA mode routines                  *}
 {************************************************************************}

  procedure PutPixVESA256(x, y : smallint; color : word); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
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
     Y := Y + YOffset; { adjust pixel for correct virtual page }
     offs := longint(y) * BytesPerLine + x;
       begin
         SetWriteBank(smallint(offs shr 16));
         mem[WinWriteSeg : word(offs)] := byte(color);
       end;
  end;

  procedure DirectPutPixVESA256(x, y : smallint); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
     col : byte;
  begin
     offs := (longint(y) + YOffset) * BytesPerLine + x;
     Case CurrentWriteMode of
       XorPut:
         Begin
           SetReadBank(smallint(offs shr 16));
           col := mem[WinReadSeg : word(offs)] xor byte(CurrentColor);
         End;
       AndPut:
         Begin
           SetReadBank(smallint(offs shr 16));
           col := mem[WinReadSeg : word(offs)] And byte(CurrentColor);
         End;
       OrPut:
         Begin
           SetReadBank(smallint(offs shr 16));
           col := mem[WinReadSeg : word(offs)] or byte(currentcolor);
         End
       else
         Begin
           If CurrentWriteMode <> NotPut then
             col := Byte(CurrentColor)
           else col := Not(Byte(CurrentColor));
         End
     End;
     SetWriteBank(smallint(offs shr 16));
     mem[WinWriteSeg : word(offs)] := Col;
  end;

  function GetPixVESA256(x, y : smallint): word; {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort + YOffset;
     offs := longint(y) * BytesPerLine + x;
     SetReadBank(smallint(offs shr 16));
     GetPixVESA256:=mem[WinReadSeg : word(offs)];
  end;

  Procedure GetScanLineVESA256(x1, x2, y: smallint; var data); {$ifndef fpc}far;{$endif}
  var offs: Longint;
      l, amount, bankrest, index, pixels: longint;
      curbank: smallint;
  begin
    inc(x1,StartXViewPort);
    inc(x2,StartXViewPort);
    {$ifdef logging}
    LogLn('getscanline256 '+strf(x1)+' - '+strf(x2)+' at '+strf(y+StartYViewPort));
    {$endif logging}
    index := 0;
    amount := x2-x1+1;
    Offs:=(Longint(y)+StartYViewPort+YOffset)*bytesperline+x1;
    Repeat
      curbank := smallint(offs shr 16);
      SetReadBank(curbank);
      {$ifdef logging}
      LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
      {$endif logging}
      If ((amount >= 4) and
          ((offs and 3) = 0)) or
         (amount >= 4+4-(offs and 3)) Then
<<<<<<< HEAD
<<<<<<< HEAD
      { align target }
=======
      { allign target }
>>>>>>> graemeg/fixes_2_2
=======
      { allign target }
>>>>>>> origin/fixes_2_2
        Begin
          If (offs and 3) <> 0 then
          { this cannot go past a window boundary bacause the }
          { size of a window is always a multiple of 4        }
            Begin
              {$ifdef logging}
<<<<<<< HEAD
<<<<<<< HEAD
              LogLn('Aligning by reading '+strf(4-(offs and 3))+' pixels');
=======
              LogLn('Alligning by reading '+strf(4-(offs and 3))+' pixels');
>>>>>>> graemeg/fixes_2_2
=======
              LogLn('Alligning by reading '+strf(4-(offs and 3))+' pixels');
>>>>>>> origin/fixes_2_2
              {$endif logging}
              for l := 1 to 4-(offs and 3) do
                WordArray(Data)[index+l-1] :=
                  Mem[WinReadSeg:word(offs)+l-1];
              inc(index, l);
              inc(offs, l);
              dec(amount, l);
            End;
          {$ifdef logging}
          LogLn('Offset is now '+hexstr(offs,8)+', amount left: '+strf(amount));
          {$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
          { offs is now 4-bytes aligned }
=======
          { offs is now 4-bytes alligned }
>>>>>>> graemeg/fixes_2_2
=======
          { offs is now 4-bytes alligned }
>>>>>>> origin/fixes_2_2
          If amount <= ($10000-(Offs and $ffff)) Then
             bankrest := amount
          else {the rest won't fit anymore in the current window }
            bankrest := $10000 - (Offs and $ffff);
          { it is possible that by aligning, we ended up in a new }
          { bank, so set the correct bank again to make sure      }
          setreadbank(offs shr 16);
          {$ifdef logging}
          LogLn('Rest to be read from this window: '+strf(bankrest));
          {$endif logging}
          For l := 0 to (Bankrest div 4)-1 Do
            begin
<<<<<<< HEAD
<<<<<<< HEAD
              pixels := MemL[WinReadSeg:word(offs)+l*4];
=======
              pixels := MemL[WinWriteSeg:word(offs)+l*4];
>>>>>>> graemeg/fixes_2_2
=======
              pixels := MemL[WinWriteSeg:word(offs)+l*4];
>>>>>>> origin/fixes_2_2
              WordArray(Data)[index+l*4] := pixels and $ff;
              pixels := pixels shr 8;
              WordArray(Data)[index+l*4+1] := pixels and $ff;
              pixels := pixels shr 8;
              WordArray(Data)[index+l*4+2] := pixels and $ff;
              pixels := pixels shr 8;
              WordArray(Data)[index+l*4+3] := pixels{ and $ff};
            end;
          inc(index,l*4+4);
          inc(offs,l*4+4);
          dec(amount,l*4+4);
          {$ifdef logging}
          LogLn('Offset is now '+hexstr(offs,8)+', amount left: '+strf(amount));
          {$endif logging}
        End
      Else
        Begin
          {$ifdef logging}
          LogLn('Leftover: '+strf(amount)+' at offset '+hexstr(offs,8));
          {$endif logging}
          For l := 0 to amount - 1 do
            begin
              { this may cross a bank at any time, so adjust          }
              { because this loop alwys runs for very little pixels,  }
              { there's little gained by splitting it up              }
              setreadbank(offs shr 16);
              WordArray(Data)[index+l] := mem[WinReadSeg:word(offs)];
              inc(offs);
            end;
          amount := 0
        End
    Until amount = 0;
  end;

  procedure HLineVESA256(x,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}

   var Offs: Longint;
       mask, l, bankrest: longint;
       curbank, hlength: smallint;
   Begin
    { must we swap the values? }
    if x > x2 then
      Begin
        x := x xor x2;
        x2 := x xor x2;
        x:= x xor x2;
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
    {$ifdef logging2}
    LogLn('hline '+strf(x)+' - '+strf(x2)+' on '+strf(y)+' in mode '+strf(currentwritemode));
    {$endif logging2}
    HLength := x2 - x + 1;
    {$ifdef logging2}
    LogLn('length: '+strf(hlength));
    {$endif logging2}
    if HLength>0 then
      begin
         Offs:=(Longint(y)+YOffset)*bytesperline+x;
         {$ifdef logging2}
         LogLn('Offs: '+strf(offs)+' -- '+hexstr(offs,8));
         {$endif logging2}
         Mask := byte(CurrentColor)+byte(CurrentColor) shl 8;
         Mask := Mask + Mask shl 16;
         Case CurrentWriteMode of
           AndPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging2}
                 If ((HLength >= 4) and
                     ((offs and 3) = 0)) or
                    (HLength >= 4+4-(offs and 3)) Then
                 { align target }
                   Begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
                     l := 0;
>>>>>>> graemeg/fixes_2_2
=======
                     l := 0;
>>>>>>> origin/fixes_2_2
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary bacause the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
<<<<<<< HEAD
<<<<<<< HEAD
                         LogLn('Aligning by drawing '+strf(4-(offs and 3))+' pixels');
=======
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
>>>>>>> graemeg/fixes_2_2
=======
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
>>>>>>> origin/fixes_2_2
                         {$endif logging2}
                         for l := 1 to 4-(offs and 3) do
                           Mem[WinWriteSeg:word(offs)+l-1] :=
                             Mem[WinReadSeg:word(offs)+l-1] And Byte(CurrentColor);
<<<<<<< HEAD
<<<<<<< HEAD
                         Dec(HLength, l);
                         inc(offs, l);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
=======
=======
>>>>>>> origin/fixes_2_2
                       End;
                     Dec(HLength, l);
                     inc(offs, l);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes alligned }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                     If HLength <= ($10000-(Offs and $ffff)) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := $10000 - (Offs and $ffff);
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 4)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] :=
                         MemL[WinReadSeg:word(offs)+l*4] And Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*4+4);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     For l := 0 to HLength - 1 do
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { becauese this loop alwys runs for very little pixels, }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         Mem[WinWriteSeg:word(offs)] :=
                           Mem[WinReadSeg:word(offs)] And byte(currentColor);
                         inc(offs);
                       end;
                     HLength := 0
                   End
               Until HLength = 0;
             End;
           XorPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging}
                 If ((HLength >= 4) and
                     ((offs and 3) = 0)) or
                    (HLength >= 4+4-(offs and 3)) Then
<<<<<<< HEAD
<<<<<<< HEAD
                 { align target }
                   Begin
=======
                 { allign target }
                   Begin
                     l := 0;
>>>>>>> graemeg/fixes_2_2
=======
                 { allign target }
                   Begin
                     l := 0;
>>>>>>> origin/fixes_2_2
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary bacause the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
<<<<<<< HEAD
<<<<<<< HEAD
                         LogLn('Aligning by drawing '+strf(4-(offs and 3))+' pixels');
=======
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
>>>>>>> graemeg/fixes_2_2
=======
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
>>>>>>> origin/fixes_2_2
                         {$endif logging}
                         for l := 1 to 4-(offs and 3) do
                           Mem[WinWriteSeg:word(offs)+l-1] :=
                             Mem[WinReadSeg:word(offs)+l-1] Xor Byte(CurrentColor);
<<<<<<< HEAD
<<<<<<< HEAD
                         Dec(HLength, l);
                         inc(offs, l);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
=======
=======
>>>>>>> origin/fixes_2_2
                       End;
                     Dec(HLength, l);
                     inc(offs, l);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes alligned }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                     If HLength <= ($10000-(Offs and $ffff)) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := $10000 - (Offs and $ffff);
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 4)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] :=
                         MemL[WinReadSeg:word(offs)+l*4] Xor Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*4+4);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     For l := 0 to HLength - 1 do
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop alwys runs for very little pixels,  }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         Mem[WinWriteSeg:word(offs)] :=
                           Mem[WinReadSeg:word(offs)] xor byte(currentColor);
                         inc(offs);
                       end;
                     HLength := 0
                   End
               Until HLength = 0;
             End;
           OrPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging}
                 If ((HLength >= 4) and
                     ((offs and 3) = 0)) or
                    (HLength >= 4+4-(offs and 3)) Then
<<<<<<< HEAD
<<<<<<< HEAD
                 { align target }
                   Begin
=======
                 { allign target }
                   Begin
                     l := 0;
>>>>>>> graemeg/fixes_2_2
=======
                 { allign target }
                   Begin
                     l := 0;
>>>>>>> origin/fixes_2_2
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary bacause the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
<<<<<<< HEAD
<<<<<<< HEAD
                         LogLn('Aligning by drawing '+strf(4-(offs and 3))+' pixels');
=======
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
>>>>>>> graemeg/fixes_2_2
=======
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
>>>>>>> origin/fixes_2_2
                         {$endif logging}
                         for l := 1 to 4-(offs and 3) do
                           Mem[WinWriteSeg:word(offs)+l-1] :=
                             Mem[WinReadSeg:word(offs)+l-1] Or Byte(CurrentColor);
<<<<<<< HEAD
<<<<<<< HEAD
                         Dec(HLength, l);
                         inc(offs, l);
                       End;
=======
                       End;
                     Dec(HLength, l);
                     inc(offs, l);
>>>>>>> graemeg/fixes_2_2
=======
                       End;
                     Dec(HLength, l);
                     inc(offs, l);
>>>>>>> origin/fixes_2_2
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
                     { offs is now 4-bytes aligned }
=======
                     { offs is now 4-bytes alligned }
>>>>>>> graemeg/fixes_2_2
=======
                     { offs is now 4-bytes alligned }
>>>>>>> origin/fixes_2_2
                     If HLength <= ($10000-(Offs and $ffff)) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := $10000 - (Offs and $ffff);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 4)-1 Do
                       MemL[WinWriteSeg:offs+l*4] :=
                         MemL[WinReadSeg:word(offs)+l*4] Or Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*4+4);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     For l := 0 to HLength - 1 do
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop alwys runs for very little pixels,  }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         Mem[WinWriteSeg:word(offs)] :=
<<<<<<< HEAD
<<<<<<< HEAD
                           Mem[WinReadSeg:word(offs)] Or byte(currentColor);
=======
                           Mem[WinReadSeg:word(offs)] And byte(currentColor);
>>>>>>> graemeg/fixes_2_2
=======
                           Mem[WinReadSeg:word(offs)] And byte(currentColor);
>>>>>>> origin/fixes_2_2
                         inc(offs);
                       end;
                     HLength := 0
                   End
               Until HLength = 0;
             End
           Else
             Begin
               If CurrentWriteMode = NotPut Then
                 Mask := Not(Mask);
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8)+' -- '+strf(offs));
                 {$endif logging}
                 If ((HLength >= 4) and
                     ((offs and 3) = 0)) or
                    (HLength >= 4+4-(offs and 3)) Then
<<<<<<< HEAD
<<<<<<< HEAD
                 { align target }
                   Begin
=======
                 { allign target }
                   Begin
                     l := 0;
>>>>>>> graemeg/fixes_2_2
=======
                 { allign target }
                   Begin
                     l := 0;
>>>>>>> origin/fixes_2_2
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary bacause the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
<<<<<<< HEAD
<<<<<<< HEAD
                         LogLn('Aligning by drawing '+strf(4-(offs and 3))+' pixels');
                         {$endif logging}
                         for l := 1 to 4-(offs and 3) do
                           Mem[WinWriteSeg:word(offs)+l-1] := Byte(Mask);
                         Dec(HLength, l);
                         inc(offs, l);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
=======
=======
>>>>>>> origin/fixes_2_2
                         LogLn('Alligning by drawing '+strf(4-(offs and 3))+' pixels');
                         {$endif logging}
                         for l := 1 to 4-(offs and 3) do
                           Mem[WinWriteSeg:word(offs)+l-1] := Byte(Mask);
                       End;
                     Dec(HLength, l);
                     inc(offs, l);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes alligned }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                     If HLength <= ($10000-(Offs and $ffff)) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := $10000 - (Offs and $ffff);
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest)+' -- '+hexstr(bankrest,8));
                     {$endif logging}
                     For l := 0 to (Bankrest div 4)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] := Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*4+4);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     For l := 0 to HLength - 1 do
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop alwys runs for very little pixels,  }
                         { there's little gained by splitting it up              }
                         setwritebank(offs shr 16);
                         Mem[WinWriteSeg:word(offs)] := byte(mask);
                         inc(offs);
                       end;
                     HLength := 0
                   End
               Until HLength = 0;
             End;
         End;
       end;
   end;

  procedure VLineVESA256(x,y,y2: smallint); {$ifndef fpc}far;{$endif fpc}

   var Offs: Longint;
       l, bankrest: longint;
       curbank, vlength: smallint;
       col: byte;
   Begin
    { must we swap the values? }
    if y > y2 then
      Begin
        y := y xor y2;
        y2 := y xor y2;
        y:= y xor y2;
      end;
    { First convert to global coordinates }
    X   := X + StartXViewPort;
    Y   := Y + StartYViewPort;
    Y2  := Y2 + StartYViewPort;
    if ClipPixels then
      Begin
         if LineClipped(x,y,x,y2,StartXViewPort,StartYViewPort,
                StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
            exit;
      end;
    Col := Byte(CurrentColor);
    {$ifdef logging2}
    LogLn('vline '+strf(y)+' - '+strf(y2)+' on '+strf(x)+' in mode '+strf(currentwritemode));
    {$endif logging}
    VLength := y2 - y + 1;
    {$ifdef logging2}
    LogLn('length: '+strf(vlength));
    {$endif logging}
    if VLength>0 then
      begin
         Offs:=(Longint(y)+YOffset)*bytesperline+x;
         {$ifdef logging2}
         LogLn('Offs: '+strf(offs)+' -- '+hexstr(offs,8));
         {$endif logging}
         Case CurrentWriteMode of
           AndPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
                 If (VLength-1)*bytesperline <= ($ffff-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($ffff - (Offs and $ffff)) div bytesperline)+1;
=======
=======
>>>>>>> origin/fixes_2_2
                 If (VLength-1)*bytesperline <= ($10000-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($10000 - (Offs and $ffff)) div bytesperline)+1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                 {$ifdef logging2}
                 LogLn('Rest to be drawn in this window: '+strf(bankrest));
                 {$endif logging}
                 For l := 0 to Bankrest-1 Do
                   begin
                     Mem[WinWriteSeg:word(offs)] :=
                       Mem[WinReadSeg:word(offs)] And Col;
                     inc(offs,bytesperline);
                   end;
                 dec(VLength,l+1);
                 {$ifdef logging2}
                 LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(vlength));
                 {$endif logging}
               Until VLength = 0;
             End;
           XorPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
                 If (VLength-1)*bytesperline <= ($ffff-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($ffff - (Offs and $ffff)) div bytesperline)+1;
=======
=======
>>>>>>> origin/fixes_2_2
                 If (VLength-1)*bytesperline <= ($10000-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($10000 - (Offs and $ffff)) div bytesperline)+1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                 {$ifdef logging2}
                 LogLn('Rest to be drawn in this window: '+strf(bankrest));
                 {$endif logging}
                 For l := 0 to Bankrest-1 Do
                   begin
                     Mem[WinWriteSeg:word(offs)] :=
                       Mem[WinReadSeg:word(offs)] Xor Col;
                     inc(offs,bytesperline);
                   end;
                 dec(VLength,l+1);
                 {$ifdef logging2}
                 LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(vlength));
                 {$endif logging}
               Until VLength = 0;
             End;
           OrPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
                 If (VLength-1)*bytesperline <= ($ffff-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($ffff - (Offs and $ffff)) div bytesperline)+1;
=======
=======
>>>>>>> origin/fixes_2_2
                 If (VLength-1)*bytesperline <= ($10000-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($10000 - (Offs and $ffff)) div bytesperline)+1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                 {$ifdef logging2}
                 LogLn('Rest to be drawn in this window: '+strf(bankrest));
                 {$endif logging}
                 For l := 0 to Bankrest-1 Do
                   begin
                     Mem[WinWriteSeg:word(offs)] :=
                       Mem[WinReadSeg:word(offs)] Or Col;
                     inc(offs,bytesperline);
                   end;
                 dec(VLength,l+1);
                 {$ifdef logging2}
                 LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(vlength));
                 {$endif logging}
               Until VLength = 0;
             End;
           Else
             Begin
               If CurrentWriteMode = NotPut Then
                 Col := Not(Col);
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
                 If (VLength-1)*bytesperline <= ($ffff-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($ffff - (Offs and $ffff)) div bytesperline)+1;
=======
=======
>>>>>>> origin/fixes_2_2
                 If (VLength-1)*bytesperline <= ($10000-(Offs and $ffff)) Then
                   bankrest := VLength
                 else {the rest won't fit anymore in the current window }
                   bankrest := (($10000 - (Offs and $ffff)) div bytesperline)+1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                 {$ifdef logging2}
                 LogLn('Rest to be drawn in this window: '+strf(bankrest));
                 {$endif logging}
                 For l := 0 to Bankrest-1 Do
                   begin
                     Mem[WinWriteSeg:word(offs)] := Col;
                     inc(offs,bytesperline);
                   end;
                 dec(VLength,l+1);
                 {$ifdef logging2}
                 LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(vlength));
                 {$endif logging}
               Until VLength = 0;
             End;
         End;
       end;
   end;

  procedure PatternLineVESA256(x1,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}
  {********************************************************}
  { Draws a horizontal patterned line according to the     }
  { current Fill Settings.                                 }
  {********************************************************}
  { Important notes:                                       }
  {  - CurrentColor must be set correctly before entering  }
  {    this routine.                                       }
  {********************************************************}
   type
     TVESA256Fill = Record
       case byte of
         0: (data1, data2: longint);
         1: (pat: array[0..7] of byte);
     end;

   var
    fill: TVESA256Fill;
    bankrest, l : longint;
    offs, amount: longint;
    i           : smallint;
    j           : smallint;
    OldWriteMode : word;
    TmpFillPattern, patternPos : byte;
   begin
     { convert to global coordinates ... }
     x1 := x1 + StartXViewPort;
     x2 := x2 + StartXViewPort;
     y  := y + StartYViewPort;
     { if line was fully clipped then exit...}
     if LineClipped(x1,y,x2,y,StartXViewPort,StartYViewPort,
        StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
         exit;
     OldWriteMode := CurrentWriteMode;
     CurrentWriteMode := NormalPut;
     { Get the current pattern }
     TmpFillPattern := FillPatternTable
       [FillSettings.Pattern][((y + startYViewPort) and $7)+1];
     {$ifdef logging2}
     LogLn('patternline '+strf(x1)+' - '+strf(x2)+' on '+strf(y));
     {$endif logging2}
     { how long is the line }
     amount := x2 - x1 + 1;
     { offset to start at }
     offs := (longint(y)+yoffset)*bytesperline+x1;
     { convert the pattern data into the actual color sequence }
     j := 1;
     FillChar(fill,sizeOf(fill),byte(currentBkColor));
     for i := 0 to 7 do
       begin
         if TmpFillPattern and j <> 0 then
           fill.pat[7-i] := currentColor;
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$q-}
         j := j shl 1;
{$pop}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifopt q+}
{$q-}
{$define overflowOn}
{$endif}
         j := j shl 1;
{$ifdef overflowOn}
{$q+}
{$undef overflowOn}
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       end;
     Repeat
       SetWriteBank(smallint(offs shr 16));
       If (amount > 7) and
          (((offs and 7) = 0) or
           (amount > 7+8-(offs and 7))) Then
         Begin
           { align target }
<<<<<<< HEAD
<<<<<<< HEAD
=======
           l := 0;
>>>>>>> graemeg/fixes_2_2
=======
           l := 0;
>>>>>>> origin/fixes_2_2
           If (offs and 7) <> 0 then
           { this cannot go past a window boundary bacause the }
           { size of a window is always a multiple of 8        }
             Begin
               { position in the pattern where to start }
               patternPos := offs and 7;
               {$ifdef logging2}
               LogLn('Aligning by drawing '+strf(8-(offs and 7))+' pixels');
               {$endif logging2}
               for l := 1 to 8-(offs and 7) do
                 begin
                   Mem[WinWriteSeg:word(offs)+l-1] := fill.pat[patternPos and 7];
                   inc(patternPos)
                 end;
<<<<<<< HEAD
<<<<<<< HEAD
               Dec(amount, l);
               inc(offs, l);
             End;
           {$ifdef logging2}
           LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(amount));
           {$endif logging2}
           { offs is now 8-bytes aligned }
=======
=======
>>>>>>> origin/fixes_2_2
             End;
           Dec(amount, l);
           inc(offs, l);
           {$ifdef logging2}
           LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(amount));
           {$endif logging2}
           { offs is now 8-bytes alligned }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           If amount <= ($10000-(Offs and $ffff)) Then
              bankrest := amount
           else {the rest won't fit anymore in the current window }
             bankrest := $10000 - (Offs and $ffff);
           { it is possible that by aligningm we ended up in a new }
           { bank, so set the correct bank again to make sure      }
           setwritebank(offs shr 16);
           {$ifdef logging2}
           LogLn('Rest to be drawn in this window: '+strf(bankrest));
           {$endif logging2}
           for l := 0 to (bankrest div 8)-1 Do
             begin
               MemL[WinWriteSeg:word(offs)+l*8] := fill.data1;
               MemL[WinWriteSeg:word(offs)+l*8+4] := fill.data2;
             end;
           inc(offs,l*8+8);
           dec(amount,l*8+8);
           {$ifdef logging2}
           LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(amount));
           {$endif logging2}
         End
       Else
         Begin
           {$ifdef logging2}
           LogLn('Drawing leftover: '+strf(amount)+' at offset '+hexstr(offs,8));
           {$endif logging2}
           patternPos := offs and 7;
           For l := 0 to amount - 1 do
             begin
               { this may cross a bank at any time, so adjust          }
               { because this loop alwys runs for very little pixels,  }
               { there's little gained by splitting it up              }
               setwritebank(offs shr 16);
               Mem[WinWriteSeg:word(offs)] := fill.pat[patternPos and 7];
               inc(offs);
               inc(patternPos);
             end;
           amount := 0;
         End
     Until amount = 0;
     currentWriteMode := oldWriteMode;
   end;


 {************************************************************************}
 {*                    256 colors VESA mode routines  Linear mode        *}
 {************************************************************************}
{$ifdef FPC}
type
  pbyte = ^byte;
  pword = ^word;

  procedure DirectPutPixVESA256Linear(x, y : smallint); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
     col : byte;
  begin
     offs := longint(y) * BytesPerLine + x;
     Case CurrentWriteMode of
       XorPut:
         Begin
           if UseNoSelector then
             col:=pbyte(LFBPointer+offs+LinearPageOfs)^
           else
             seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),1);
           col := col xor byte(CurrentColor);
         End;
       AndPut:
         Begin
           if UseNoSelector then
             col:=pbyte(LFBPointer+offs+LinearPageOfs)^
           else
             seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),1);
           col := col and byte(CurrentColor);
         End;
       OrPut:
         Begin
           if UseNoSelector then
             col:=pbyte(LFBPointer+offs+LinearPageOfs)^
           else
             seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),1);
           col := col or byte(CurrentColor);
         End
       else
         Begin
           If CurrentWriteMode <> NotPut then
             col := Byte(CurrentColor)
           else col := Not(Byte(CurrentColor));
         End
     End;
     if UseNoSelector then
       pbyte(LFBPointer+offs+LinearPageOfs)^:=col
     else
       seg_move(get_ds,longint(@col),WinWriteSeg,offs+LinearPageOfs,1);
  end;

  procedure PutPixVESA256Linear(x, y : smallint; color : word); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
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
     offs := longint(y) * BytesPerLine + x;
     {$ifdef logging}
     logln('putpix offset: '+hexstr(offs,8)+', color: '+strf(color)+', lpo: $'+
       hexstr(LinearPageOfs,8));
     {$endif logging}
     if UseNoSelector then
       pbyte(LFBPointer+offs+LinearPageOfs)^:=byte(color)
     else
       seg_move(get_ds,longint(@color),WinWriteSeg,offs+LinearPageOfs,1);
  end;

  function GetPixVESA256Linear(x, y : smallint): word; {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
     col : byte;
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort;
     offs := longint(y) * BytesPerLine + x;
     {$ifdef logging}
     logln('getpix offset: '+hexstr(offs,8)+', lpo: $'+
       hexstr(LinearPageOfs,8));
     {$endif logging}
     if UseNoSelector then
       col:=pbyte(LFBPointer+offs+LinearPageOfs)^
     else
       seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),1);
     GetPixVESA256Linear:=col;
  end;
(*
function SetVESADisplayStart(PageNum : word;x,y : smallint):Boolean;
var
  dregs : registers;
begin
  if PageNum>VesaModeInfo.NumberOfPages then
    PageNum:=0;
{$ifdef DEBUG}
  if PageNum>0 then
    writeln(stderr,'Setting Display Page ',PageNum);
{$endif DEBUG}
  dregs.RealEBX:=0{ $80 for Wait for retrace };
  dregs.RealECX:=x;
  dregs.RealEDX:=y+PageNum*maxy;
  dregs.RealSP:=0;
  dregs.RealSS:=0;
  dregs.RealEAX:=$4F07; RealIntr($10,dregs);
  { idem as above !!! }
  if (dregs.RealEAX and $1FF) <> $4F then
    begin
{$ifdef DEBUG}
       writeln(stderr,'Set Display start error');
{$endif DEBUG}
       SetVESADisplayStart:=false;
    end
  else
    SetVESADisplayStart:=true;
end;
*)
{$endif FPC}


 {************************************************************************}
 {*                    15/16bit pixels VESA mode routines                *}
 {************************************************************************}

  procedure PutPixVESA32kOr64k(x, y : smallint; color : word); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
<<<<<<< HEAD
<<<<<<< HEAD
     place: word;
     bank : shortint;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  begin
{$ifdef logging}
     logln('putpixvesa32kor64k('+strf(x)+','+strf(y)+')');
{$endif logging}
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
     Y := Y + YOffset; { adjust pixel for correct virtual page }
     offs := longint(y) * BytesPerLine + 2*x;
<<<<<<< HEAD
<<<<<<< HEAD
     bank := offs div 65536;
     place:= offs mod 65536;
     SetWriteBank(bank);

{$ifdef logging}
     logln('putpixvesa32kor64k offset: '+strf(word(offs)));
{$endif logging}
     memW[WinWriteSeg : place] := color;
=======
=======
>>>>>>> origin/fixes_2_2
     SetWriteBank(smallint(offs shr 16));
{$ifdef logging}
     logln('putpixvesa32kor64k offset: '+strf(word(offs)));
{$endif logging}
     memW[WinWriteSeg : word(offs)] := color;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  function GetPixVESA32kOr64k(x, y : smallint): word; {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort + YOffset;
     offs := longint(y) * BytesPerLine + 2*x;
     SetReadBank(smallint(offs shr 16));
     GetPixVESA32kOr64k:=memW[WinReadSeg : word(offs)];
  end;

  procedure DirectPutPixVESA32kOr64k(x, y : smallint); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
<<<<<<< HEAD
<<<<<<< HEAD
     bank : smallint;
     place,col : word;
=======
     col : word;
>>>>>>> graemeg/fixes_2_2
=======
     col : word;
>>>>>>> origin/fixes_2_2
  begin
{$ifdef logging}
     logln('directputpixvesa32kor64k('+strf(x)+','+strf(y)+')');
{$endif logging}
     y:= Y + YOffset;
<<<<<<< HEAD
<<<<<<< HEAD

     offs := longint(y) * BytesPerLine + 2*x;
     bank:=offs div 65536;
     place:=offs mod 65536;

     SetWriteBank(bank and $FF); // unknown why this and $FF is here.
     Case CurrentWriteMode of
       XorPut:
         Begin
           SetReadBank(bank);
           memW[WinWriteSeg : place] := memW[WinReadSeg : place] xor currentcolor;
         End;
       AndPut:
         Begin
           SetReadBank(bank);
           memW[WinWriteSeg : place] := memW[WinReadSeg : place] And currentcolor;
         End;
       OrPut:
         Begin
           SetReadBank(bank);
           memW[WinWriteSeg : place] := memW[WinReadSeg : place] or currentcolor;
=======
=======
>>>>>>> origin/fixes_2_2
     offs := longint(y) * BytesPerLine + 2*x;
     SetWriteBank(smallint((offs shr 16) and $ff));
     Case CurrentWriteMode of
       XorPut:
         Begin
           SetReadBank(smallint(offs shr 16));
           memW[WinWriteSeg : word(offs)] := memW[WinReadSeg : word(offs)] xor currentcolor;
         End;
       AndPut:
         Begin
           SetReadBank(smallint(offs shr 16));
           memW[WinWriteSeg : word(offs)] := memW[WinReadSeg : word(offs)] And currentcolor;
         End;
       OrPut:
         Begin
           SetReadBank(smallint(offs shr 16));
           memW[WinWriteSeg : word(offs)] := memW[WinReadSeg : word(offs)] or currentcolor;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         End
       else
         Begin
           If CurrentWriteMode <> NotPut Then
             col := CurrentColor
           Else col := Not(CurrentColor);
{$ifdef logging}
           logln('directputpixvesa32kor64k offset: '+strf(word(offs)));
{$endif logging}
<<<<<<< HEAD
<<<<<<< HEAD
           memW[WinWriteSeg : place] := Col;
=======
           memW[WinWriteSeg : word(offs)] := Col;
>>>>>>> graemeg/fixes_2_2
=======
           memW[WinWriteSeg : word(offs)] := Col;
>>>>>>> origin/fixes_2_2
         End
     End;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  procedure HLineVESA32kOr64k(x,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}

   var Offs: Longint;
       mask, l, bankrest: longint;
       curbank, hlength: smallint;
   Begin
    { must we swap the values? }
    if x > x2 then
      Begin
        x := x xor x2;
        x2 := x xor x2;
        x:= x xor x2;
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
    {$ifdef logging2}
    LogLn('hline '+strf(x)+' - '+strf(x2)+' on '+strf(y)+' in mode '+strf(currentwritemode));
    {$endif logging2}
    HLength := x2 - x + 1;
    {$ifdef logging2}
    LogLn('length: '+strf(hlength));
    {$endif logging2}
    if HLength>0 then
      begin
         Offs:=(Longint(y)+YOffset)*bytesperline+2*x;
         {$ifdef logging2}
         LogLn('Offs: '+strf(offs)+' -- '+hexstr(offs,8));
         {$endif logging2}
         Mask := longint(word(CurrentColor)+word(CurrentColor) shl 16);
         Case CurrentWriteMode of
           AndPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging2}
                 If ((HLength >= 2) and
                     ((offs and 3) = 0)) or
                    (HLength >= 3) Then
                 { align target }
                   Begin
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary because the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
                         LogLn('Aligning by drawing 1 pixel');
                         {$endif logging2}
                         MemW[WinWriteSeg:word(offs)] :=
                           MemW[WinReadSeg:word(offs)] And Word(CurrentColor);
                         Dec(HLength);
                         inc(offs, 2);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
                     If HLength <= (($10000-(Offs and $ffff)) shr 1) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := ($10000 - (Offs and $ffff)) shr 1;
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 2)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] :=
                         MemL[WinReadSeg:word(offs)+l*4] And Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*2+2);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     if HLength > 0 then
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop always runs for very little pixels, }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         MemW[WinWriteSeg:word(offs)] :=
                           MemW[WinReadSeg:word(offs)] And Word(currentColor);
                         HLength := 0
                       end;
                   End
               Until HLength = 0;
             End;
           XorPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging2}
                 If ((HLength >= 2) and
                     ((offs and 3) = 0)) or
                    (HLength >= 3) Then
                 { align target }
                   Begin
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary because the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
                         LogLn('Aligning by drawing 1 pixel');
                         {$endif logging2}
                         MemW[WinWriteSeg:word(offs)] :=
                           MemW[WinReadSeg:word(offs)] Xor Word(CurrentColor);
                         Dec(HLength);
                         inc(offs, 2);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
                     If HLength <= (($10000-(Offs and $ffff)) shr 1) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := ($10000 - (Offs and $ffff)) shr 1;
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 2)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] :=
                         MemL[WinReadSeg:word(offs)+l*4] Xor Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*2+2);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     if HLength > 0 then
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop always runs for very little pixels, }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         MemW[WinWriteSeg:word(offs)] :=
                           MemW[WinReadSeg:word(offs)] Xor Word(currentColor);
                         HLength := 0
                       end;
                   End
               Until HLength = 0;
             End;
           OrPut:
             Begin
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging2}
                 If ((HLength >= 2) and
                     ((offs and 3) = 0)) or
                    (HLength >= 3) Then
                 { align target }
                   Begin
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary because the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
                         LogLn('Aligning by drawing 1 pixel');
                         {$endif logging2}
                         MemW[WinWriteSeg:word(offs)] :=
                           MemW[WinReadSeg:word(offs)] Or Word(CurrentColor);
                         Dec(HLength);
                         inc(offs, 2);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
                     If HLength <= (($10000-(Offs and $ffff)) shr 1) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := ($10000 - (Offs and $ffff)) shr 1;
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 2)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] :=
                         MemL[WinReadSeg:word(offs)+l*4] Or Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*2+2);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     if HLength > 0 then
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop always runs for very little pixels, }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         MemW[WinWriteSeg:word(offs)] :=
                           MemW[WinReadSeg:word(offs)] Or Word(currentColor);
                         HLength := 0
                       end;
                   End
               Until HLength = 0;
             End
           Else
             Begin
               If CurrentWriteMode = NotPut Then
                 Mask := Not(Mask);
               Repeat
                 curbank := smallint(offs shr 16);
                 SetWriteBank(curbank);
                 SetReadBank(curbank);
                 {$ifdef logging2}
                 LogLn('set bank '+strf(curbank)+' for offset '+hexstr(offs,8));
                 {$endif logging2}
                 If ((HLength >= 2) and
                     ((offs and 3) = 0)) or
                    (HLength >= 3) Then
                 { align target }
                   Begin
                     If (offs and 3) <> 0 then
                     { this cannot go past a window boundary because the }
                     { size of a window is always a multiple of 4        }
                       Begin
                         {$ifdef logging2}
                         LogLn('Aligning by drawing 1 pixel');
                         {$endif logging2}
                         MemW[WinWriteSeg:word(offs)] := Word(Mask);
                         Dec(HLength);
                         inc(offs, 2);
                       End;
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                     { offs is now 4-bytes aligned }
                     If HLength <= (($10000-(Offs and $ffff)) shr 1) Then
                        bankrest := HLength
                     else {the rest won't fit anymore in the current window }
                       bankrest := ($10000 - (Offs and $ffff)) shr 1;
                     { it is possible that by aligningm we ended up in a new }
                     { bank, so set the correct bank again to make sure      }
                     setwritebank(offs shr 16);
                     setreadbank(offs shr 16);
                     {$ifdef logging2}
                     LogLn('Rest to be drawn in this window: '+strf(bankrest));
                     {$endif logging}
                     For l := 0 to (Bankrest div 2)-1 Do
                       MemL[WinWriteSeg:word(offs)+l*4] := Mask;
                     inc(offs,l*4+4);
                     dec(hlength,l*2+2);
                     {$ifdef logging2}
                     LogLn('Offset is now '+hexstr(offs,8)+', length left: '+strf(hlength));
                     {$endif logging}
                   End
                 Else
                   Begin
                     {$ifdef logging2}
                     LogLn('Drawing leftover: '+strf(HLength)+' at offset '+hexstr(offs,8));
                     {$endif logging}
                     if HLength > 0 then
                       begin
                         { this may cross a bank at any time, so adjust          }
                         { because this loop always runs for very little pixels, }
                         { there's little gained by splitting it up              }
                         setreadbank(offs shr 16);
                         setwritebank(offs shr 16);
                         MemW[WinWriteSeg:word(offs)] := Word(Mask);
                         HLength := 0
                       end;
                   End
               Until HLength = 0;
             End;
         End;
       end;
   end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC}
 {************************************************************************}
 {*                    15/16bit pixels VESA mode routines  Linear mode   *}
 {************************************************************************}

  procedure PutPixVESA32kor64kLinear(x, y : smallint; color : word); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
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
     offs := longint(y) * BytesPerLine + 2*x;
     if UseNoSelector then
       pword(LFBPointer+offs+LinearPageOfs)^:=color
     else
       seg_move(get_ds,longint(@color),WinWriteSeg,offs+LinearPageOfs,2);
  end;

  function GetPixVESA32kor64kLinear(x, y : smallint): word; {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
     color : word;
  begin
     X:= X + StartXViewPort;
     Y:= Y + StartYViewPort;
     offs := longint(y) * BytesPerLine + 2*x;
     if UseNoSelector then
       color:=pword(LFBPointer+offs+LinearPageOfs)^
     else
       seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@color),2);
     GetPixVESA32kor64kLinear:=color;
  end;

  procedure DirectPutPixVESA32kor64kLinear(x, y : smallint); {$ifndef fpc}far;{$endif fpc}
  var
     offs : longint;
     col : word;
  begin
     offs := longint(y) * BytesPerLine + 2*x;
     Case CurrentWriteMode of
       XorPut:
         Begin
           if UseNoSelector then
             col:=pword(LFBPointer+offs+LinearPageOfs)^
           else
             seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),2);
           col := col xor currentcolor;
         End;
       AndPut:
         Begin
           if UseNoSelector then
             col:=pword(LFBPointer+offs+LinearPageOfs)^
           else
             seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),2);
           col := col and currentcolor;
         End;
       OrPut:
         Begin
           if UseNoSelector then
             col:=pword(LFBPointer+offs+LinearPageOfs)^
           else
             seg_move(WinWriteSeg,offs+LinearPageOfs,get_ds,longint(@col),2);
           col := col or currentcolor;
         End
       else
         Begin
           If CurrentWriteMode <> NotPut Then
             col := CurrentColor
           Else col := Not(CurrentColor);
         End
     End;
     if UseNoSelector then
       pword(LFBPointer+offs+LinearPageOfs)^:=col
     else
       seg_move(get_ds,longint(@col),WinWriteSeg,offs+LinearPageOfs,2);
  end;

{$endif FPC}

 {************************************************************************}
 {*                     4-bit pixels VESA mode routines                  *}
 {************************************************************************}

  procedure PutPixVESA16(x, y : smallint; color : word); {$ifndef fpc}far;{$endif fpc}
    var
     offs : longint;
     dummy : byte;
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
     Y := Y + YOffset; { adjust pixel for correct virtual page }
     { }
     offs := longint(y) * BytesPerLine + (x div 8);
<<<<<<< HEAD
<<<<<<< HEAD
     SetReadBank(smallint(offs shr 16));
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     SetWriteBank(smallint(offs shr 16));

     PortW[$3ce] := $0f01;       { Index 01 : Enable ops on all 4 planes }
     PortW[$3ce] := color shl 8; { Index 00 : Enable correct plane and write color }

<<<<<<< HEAD
<<<<<<< HEAD
     { Index 08 : Bitmask register.          }
     PortW[$3ce] := ($8000 shr (x and $7)) or 8; { Select correct bits to modify }

     dummy := Mem[WinReadSeg: word(offs)];  { Latch the data into host space.  }
     Mem[WinWriteSeg: word(offs)] := dummy;  { Write the data into video memory }
=======
=======
>>>>>>> origin/fixes_2_2
     Port[$3ce] := 8;           { Index 08 : Bitmask register.          }
     Port[$3cf] := $80 shr (x and $7); { Select correct bits to modify }

     dummy := Mem[WinWriteSeg: offs];  { Latch the data into host space.  }
     Mem[WinWriteSeg: offs] := dummy;  { Write the data into video memory }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
     { }
  end;


 Function GetPixVESA16(X,Y: smallint):word; {$ifndef fpc}far;{$endif fpc}
<<<<<<< HEAD
<<<<<<< HEAD
 Var dummy: Word;
     offset: longint;
=======
 Var dummy, offset: Word;
>>>>>>> graemeg/fixes_2_2
=======
 Var dummy, offset: Word;
>>>>>>> origin/fixes_2_2
     shift: byte;
  Begin
    X:= X + StartXViewPort;
    Y:= Y + StartYViewPort + YOffset;
    offset := longint(Y) * BytesPerLine + (x div 8);
    SetReadBank(smallint(offset shr 16));
<<<<<<< HEAD
<<<<<<< HEAD
    PortW[$3ce] := $0004;
    shift := 7 - (X and 7);
    dummy := (Mem[WinReadSeg:word(offset)] shr shift) and 1;
    Port[$3cf] := 1;
    dummy := dummy or (((Mem[WinReadSeg:word(offset)] shr shift) and 1) shl 1);
    Port[$3cf] := 2;
    dummy := dummy or (((Mem[WinReadSeg:word(offset)] shr shift) and 1) shl 2);
    Port[$3cf] := 3;
    dummy := dummy or (((Mem[WinReadSeg:word(offset)] shr shift) and 1) shl 3);
=======
=======
>>>>>>> origin/fixes_2_2
    Port[$3ce] := 4;
    shift := 7 - (X and 7);
    Port[$3cf] := 0;
    dummy := (Mem[WinReadSeg:offset] shr shift) and 1;
    Port[$3cf] := 1;
    dummy := dummy or (((Mem[WinReadSeg:offset] shr shift) and 1) shl 1);
    Port[$3cf] := 2;
    dummy := dummy or (((Mem[WinReadSeg:offset] shr shift) and 1) shl 2);
    Port[$3cf] := 3;
    dummy := dummy or (((Mem[WinReadSeg:offset] shr shift) and 1) shl 3);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    GetPixVESA16 := dummy;
  end;


  procedure DirectPutPixVESA16(x, y : smallint); {$ifndef fpc}far;{$endif fpc}
    var
     offs : longint;
     dummy : byte;
     Color : word;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
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

     Y := Y + YOffset;
     offs := longint(y) * BytesPerLine + (x div 8);
     SetReadBank(smallint(offs shr 16));
=======
=======
>>>>>>> origin/fixes_2_2
    y:= Y + YOffset;
    case CurrentWriteMode of
      XORPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixVESA16(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Xor Color;
        end;
      OrPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixVESA16(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor Or Color;
        end;
      AndPut:
        begin
      { getpixel wants local/relative coordinates }
          Color := GetPixVESA16(x-StartXViewPort,y-StartYViewPort);
          Color := CurrentColor And Color;
        end;
      NotPut:
        begin
          Color := Not Color;
        end
      else
        Color := CurrentColor;
    end;
     offs := longint(y) * BytesPerLine + (x div 8);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     SetWriteBank(smallint(offs shr 16));
     PortW[$3ce] := $0f01;       { Index 01 : Enable ops on all 4 planes }
     PortW[$3ce] := color shl 8; { Index 00 : Enable correct plane and write color }

<<<<<<< HEAD
<<<<<<< HEAD
     { Index 08 : Bitmask register.          }
     PortW[$3ce] := ($8000 shr (x and $7)) or 8; { Select correct bits to modify }

     dummy := Mem[WinReadSeg: word(offs)];  { Latch the data into host space.  }
     Mem[WinWriteSeg: word(offs)] := dummy;  { Write the data into video memory }
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
     if (CurrentWriteMode = XORPut) or
        (CurrentWriteMode = ANDPut) or
        (CurrentWriteMode = ORPut) then
       PortW[$3ce] := $0003;
  end;


  procedure HLineVESA16(x,x2,y: smallint); {$ifndef fpc}far;{$endif fpc}
  var
      xtmp: smallint;
      ScrOfs, BankRest: longint;
      HLength : word;
      LMask,RMask : byte;
  begin

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
    Y := Y + YOffset;
    ScrOfs := longint(y) * BytesPerLine + (x div 8);
    SetReadBank(smallint(ScrOfs shr 16));
    SetWriteBank(smallint(ScrOfs shr 16));
    HLength:=x2 div 8-x div 8;
    LMask:=$ff shr (x and 7);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$r-}
{$q-}
    RMask:=$ff shl (7-(x2 and 7));
{$pop}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
{$push}
{$r-}
{$q-}
    Mem[WinWriteSeg:word(ScrOfs)]:=Mem[WinReadSeg:word(ScrOfs)]+1;
{$pop}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
    Mem[WinWriteSeg:word(ScrOfs)]:=Mem[WinReadSeg:word(ScrOfs)]+1;
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    {Port[$3ce]:=8;}{not needed, the register is already selected}
    if HLength>0 then
      begin
         dec(HLength);
         inc(ScrOfs);
         while (HLength>0) do
           begin
              SetReadBank(smallint(ScrOfs shr 16));
              SetWriteBank(smallint(ScrOfs shr 16));
              Port[$3cf]:=$ff;
              if HLength <= ($10000-(ScrOfs and $ffff)) Then
                 BankRest := HLength
              else {the rest won't fit anymore in the current window }
                BankRest := $10000 - (ScrOfs and $ffff);
{$ifndef tp}
              seg_bytemove(dosmemselector,(WinReadSeg shl 4)+word(ScrOfs),dosmemselector,(WinWriteSeg shl 4)+word(ScrOfs),BankRest);
{$else}
              move(Ptr(WinReadSeg,word(ScrOfs))^, Ptr(WinWriteSeg,word(ScrOfs))^, BankRest);
{$endif}
              ScrOfs := ScrOfs + BankRest;
              HLength := HLength - BankRest;
           end;
         SetReadBank(smallint(ScrOfs shr 16));
         SetWriteBank(smallint(ScrOfs shr 16));
         Port[$3cf]:=RMask;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$r-}
{$q-}
         Mem[WinWriteSeg:word(ScrOfs)]:=Mem[WinReadSeg:word(ScrOfs)]+1;
{$pop}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifopt r+}
{$define rangeOn}
{$r-}
{$endif}
{$ifopt q+}
{$define overflowOn}
{$q-}
{$endif}
         Mem[WinWriteSeg:word(ScrOfs)]:=Mem[WinReadSeg:word(ScrOfs)]+1;
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;
    { clean up }
    {Port[$3cf]:=0;}{not needed, the register is reset by the next operation:}
    PortW[$3ce]:=$ff08;
    PortW[$3ce]:=$0001;
    PortW[$3ce]:=$0003;
   end;


=======
=======
>>>>>>> origin/fixes_2_2
     Port[$3ce] := 8;           { Index 08 : Bitmask register.          }
     Port[$3cf] := $80 shr (x and $7); { Select correct bits to modify }

     dummy := Mem[WinWriteSeg: offs];  { Latch the data into host space.  }
     Mem[WinWriteSeg: offs] := dummy;  { Write the data into video memory }
     PortW[$3ce] := $ff08;         { Enable all bit planes.           }
     PortW[$3ce] := $0001;         { Index 01 : Disable ops on all four planes.         }
  end;


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


 {************************************************************************}
 {*                     VESA Palette entries                             *}
 {************************************************************************}


{$IFDEF DPMI}
{$ifdef fpc}
   Procedure SetVESARGBAllPalette(const Palette:PaletteType);
    var
     pal: array[0..255] of palrec;
     regs: TDPMIRegisters;
     c, Ptr: longint;
     RealSeg: word;
     FunctionNr : byte;   { use blankbit or normal RAMDAC programming? }
    begin
      if DirectColor then
        Begin
          _GraphResult := grError;
          exit;
        end;
      { use the set/get palette function }
      if VESAInfo.Version >= $0200 then
        Begin
          { check if blanking bit must be set when programming }
          { the RAMDAC.                                        }
          if (VESAInfo.caps and attrSnowCheck) <> 0 then
            FunctionNr := $80
          else
            FunctionNr := $00;

          fillChar(pal,sizeof(pal),0);
          { Convert to vesa format }
          for c := 0 to 255 do
            begin
              pal[c].red := byte(palette.colors[c].red);
              pal[c].green := byte(palette.colors[c].green);
              pal[c].blue := byte(palette.colors[c].blue);
            end;

        { Alllocate real mode buffer }
          Ptr:=Global_Dos_Alloc(sizeof(pal));
          {get the segment value}
          RealSeg := word(Ptr shr 16);
          { setup interrupt registers }
          FillChar(regs, sizeof(regs), #0);
          { copy palette values to real mode buffer }
          DosMemPut(RealSeg,0,pal,sizeof(pal));
          regs.eax := $4F09;
          regs.ebx := FunctionNr;
          regs.ecx := 256;
          regs.edx := 0;
          regs.es  := RealSeg;
          regs.edi := 0;         { offset is always zero }
          RealIntr($10, regs);

          { free real mode memory  }
          If not Global_Dos_Free(word(Ptr and $ffff)) then
            RunError(216);

          if word(regs.eax) <> $004F then
            begin
              _GraphResult := grError;
              exit;
            end;
        end
      else
        { assume it's fully VGA compatible palette-wise. }
        Begin
          SetVGARGBAllPalette(palette);
        end;
      setallpalettedefault(palette);
    end;
{$endif fpc}

   Procedure SetVESARGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint);
    var
     pal: palrec;
     regs: TDPMIRegisters;
     Ptr: longint;
{$ifndef fpc}
     PalPtr : ^PalRec;
{$endif fpc}
     RealSeg: word;
     FunctionNr : byte;   { use blankbit or normal RAMDAC programming? }
    begin
      if DirectColor then
        Begin
{$ifdef logging}
          logln('setvesargbpalette called with directcolor = true');
{$endif logging}
          _GraphResult := grError;
          exit;
        end;
        pal.align := 0;
        pal.red := byte(RedValue) shr 2;
        pal.green := byte(GreenValue) shr 2;
        pal.blue := byte(BlueValue) shr 2;
        { use the set/get palette function }
        if VESAInfo.Version >= $0200 then
          Begin
            { check if blanking bit must be set when programming }
            { the RAMDAC.                                        }
            if (VESAInfo.caps and attrSnowCheck) <> 0 then
              FunctionNr := $80
            else
              FunctionNr := $00;

            { Alllocate real mode buffer }
{$ifndef fpc}
            Ptr:=GlobalDosAlloc(sizeof(palrec));
            { get the selector values }
            PalPtr := pointer(Ptr shl 16);
            if not assigned(PalPtr) then
               RunError(203);
{$else fpc}
            Ptr:=Global_Dos_Alloc(sizeof(palrec));
{$endif fpc}
            {get the segment value}
            RealSeg := word(Ptr shr 16);
            { setup interrupt registers }
            FillChar(regs, sizeof(regs), #0);
            { copy palette values to real mode buffer }
{$ifndef fpc}
            move(pal, palptr^, sizeof(pal));
{$else fpc}
            DosMemPut(RealSeg,0,pal,sizeof(pal));
{$endif fpc}
            regs.eax := $4F09;
            regs.ebx := FunctionNr;
            regs.ecx := $01;
            regs.edx := ColorNum;
            regs.es  := RealSeg;
            regs.edi := 0;         { offset is always zero }
            RealIntr($10, regs);

            { free real mode memory  }
{$ifndef fpc}
            GlobalDosFree(word(Ptr and $ffff));
{$else fpc}
            If not Global_Dos_Free(word(Ptr and $ffff)) then
              RunError(216);
{$endif fpc}

            if word(regs.eax) <> $004F then
              begin
{$ifdef logging}
                logln('setvesargbpalette failed while directcolor = false!');
{$endif logging}
                _GraphResult := grError;
                exit;
              end;
          end
        else
          { assume it's fully VGA compatible palette-wise. }
          Begin
            SetVGARGBPalette(ColorNum, RedValue, GreenValue, BlueValue);
          end;
    end;


  Procedure GetVESARGBPalette(ColorNum: smallint; Var
      RedValue, GreenValue, BlueValue : smallint);
   var
    pal: PalRec;
{$ifndef fpc}
    palptr : ^PalRec;
{$endif fpc}
    regs : TDPMIRegisters;
    RealSeg: word;
    ptr: longint;
   begin
      if DirectColor then
        Begin
{$ifdef logging}
         logln('getvesargbpalette called with directcolor = true');
{$endif logging}
          _GraphResult := grError;
          exit;
        end;
        { use the set/get palette function }
        if VESAInfo.Version >= $0200 then
          Begin
            { Alllocate real mode buffer }
{$ifndef fpc}
            Ptr:=GlobalDosAlloc(sizeof(palrec));
            { get the selector value }
            PalPtr := pointer(longint(Ptr and $0000ffff) shl 16);
            if not assigned(PalPtr) then
               RunError(203);
{$else fpc}
            Ptr:=Global_Dos_Alloc(sizeof(palrec));
{$endif fpc}
            { get the segment value }
            RealSeg := word(Ptr shr 16);
            { setup interrupt registers }
            FillChar(regs, sizeof(regs), #0);

            regs.eax := $4F09;
            regs.ebx := $01;       { get palette data      }
            regs.ecx := $01;
            regs.edx := ColorNum;
            regs.es  := RealSeg;
            regs.edi := 0;         { offset is always zero }
            RealIntr($10, regs);

           { copy to protected mode buffer ... }
{$ifndef fpc}
           Move(PalPtr^, Pal, sizeof(pal));
{$else fpc}
           DosMemGet(RealSeg,0,Pal,sizeof(pal));
{$endif fpc}
           { free real mode memory  }
{$ifndef fpc}
           GlobalDosFree(word(Ptr and $ffff));
{$else fpc}
           If not Global_Dos_Free(word(Ptr and $ffff)) then
             RunError(216);
{$endif fpc}

            if word(regs.eax) <> $004F then
              begin
{$ifdef logging}
                logln('getvesargbpalette failed while directcolor = false!');
{$endif logging}
                _GraphResult := grError;
                exit;
              end
            else
              begin
                RedValue := smallint(pal.Red);
                GreenValue := smallint(pal.Green);
                BlueValue := smallint(pal.Blue);
              end;
          end
        else
            GetVGARGBPalette(ColorNum, RedValue, GreenValue, BlueValue);
   end;
{$ELSE}

   Procedure SetVESARGBPalette(ColorNum, RedValue, GreenValue,
      BlueValue : smallint); far;
    var
     FunctionNr : byte;   { use blankbit or normal RAMDAC programming? }
     pal: ^palrec;
     Error : boolean;     { VBE call error                             }
    begin
      if DirectColor then
        Begin
          _GraphResult := grError;
          exit;
        end;
        Error := FALSE;
        new(pal);
        if not assigned(pal) then RunError(203);
        pal^.align := 0;
        pal^.red := byte(RedValue);
        pal^.green := byte(GreenValue);
        pal^.blue := byte(BlueValue);
        { use the set/get palette function }
        if VESAInfo.Version >= $0200 then
          Begin
            { check if blanking bit must be set when programming }
            { the RAMDAC.                                        }
            if (VESAInfo.caps and attrSnowCheck) <> 0 then
              FunctionNr := $80
            else
              FunctionNr := $00;
            asm
              mov  ax, 4F09h         { Set/Get Palette data    }
              mov  bl, [FunctionNr]  { Set palette data        }
              mov  cx, 01h           { update one palette reg. }
              mov  dx, [ColorNum]    { register number to update }
              les  di, [pal]         { get palette address     }
              int  10h
              cmp  ax, 004Fh         { check if success        }
              jz   @noerror
              mov  [Error], TRUE
             @noerror:
            end;
            if not Error then
                Dispose(pal)
            else
              begin
                _GraphResult := grError;
                exit;
              end;
          end
        else
          { assume it's fully VGA compatible palette-wise. }
          Begin
            SetVGARGBPalette(ColorNum, RedValue, GreenValue, BlueValue);
          end;
    end;




  Procedure GetVESARGBPalette(ColorNum: smallint; Var RedValue, GreenValue,
              BlueValue : smallint); far;
   var
    Error: boolean;
    pal: ^palrec;
   begin
      if DirectColor then
        Begin
          _GraphResult := grError;
          exit;
        end;
      Error := FALSE;
      new(pal);
      if not assigned(pal) then RunError(203);
      FillChar(pal^, sizeof(palrec), #0);
      { use the set/get palette function }
      if VESAInfo.Version >= $0200 then
        Begin
          asm
            mov  ax, 4F09h         { Set/Get Palette data    }
            mov  bl, 01h           { Set palette data        }
            mov  cx, 01h           { update one palette reg. }
            mov  dx, [ColorNum]    { register number to update }
            les  di, [pal]         { get palette address     }
            int  10h
            cmp  ax, 004Fh         { check if success        }
            jz   @noerror
            mov  [Error], TRUE
          @noerror:
          end;
          if not Error then
            begin
              RedValue := smallint(pal^.Red);
              GreenValue := smallint(pal^.Green);
              BlueValue := smallint(pal^.Blue);
              Dispose(pal);
            end
          else
            begin
              _GraphResult := grError;
              exit;
            end;
        end
        else
            GetVGARGBPalette(ColorNum, RedValue, GreenValue, BlueValue);

   end;
{$ENDIF}


(*
type
  heaperrorproc=function(size:longint):smallint;

Const
  HeapErrorIsHooked : boolean = false;
  OldHeapError : HeapErrorProc = nil;
  DsLimit : dword = 0;

  function NewHeapError(size : longint) : smallint;
    begin
      set_segment_limit(get_ds,DsLimit);
      NewHeapError:=OldHeapError(size);
      DsLimit:=get_segment_limit(get_ds);
      { The base of ds can be changed
        we need to compute the address again PM }
      LFBPointer:=pointer(FrameBufferLinearAddress-get_segment_base_address(get_ds));
      if dword(LFBPointer)+dword(VESAInfo.TotalMem shl 16)-1 > DsLimit then
        set_segment_limit(get_ds,dword(LFBPointer)+dword(VESAInfo.TotalMem shl 16)-1);
    end;

  procedure HookHeapError;
    begin
      if HeapErrorIsHooked then
        exit;
      DsLimit:=get_segment_limit(get_ds);
      OldHeapError:=HeapErrorProc(HeapError);
      HeapError:=@NewHeapError;
      HeapErrorIsHooked:=true;
    end;

  procedure UnHookHeapError;
    begin
      if not HeapErrorIsHooked then
        exit;
      LFBPointer:=nil;
      set_segment_limit(get_ds,DsLimit);
      HeapError:=OldHeapError;
      HeapErrorIsHooked:=false;
    end;
*)

  function SetupLinear(var ModeInfo: TVESAModeInfo;mode : word) : boolean;
   begin
     SetUpLinear:=false;
<<<<<<< HEAD
<<<<<<< HEAD

     if VESAInfo.Version >= $0300 then
       BytesPerLine := VESAModeInfo.LinBytesPerScanLine
     else
       BytesPerLine := VESAModeInfo.BytesPerScanLine;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC}
     case mode of
       m320x200x32k,
       m320x200x64k,
       m640x480x32k,
       m640x480x64k,
       m800x600x32k,
       m800x600x64k,
       m1024x768x32k,
       m1024x768x64k,
       m1280x1024x32k,
       m1280x1024x64k :
         begin
           DirectPutPixel:=@DirectPutPixVESA32kor64kLinear;
           PutPixel:=@PutPixVESA32kor64kLinear;
           GetPixel:=@GetPixVESA32kor64kLinear;
           { linear mode for lines not yet implemented PM }
           HLine:=@HLineDefault;
           VLine:=@VLineDefault;
           GetScanLine := @GetScanLineDefault;
           PatternLine := @PatternLineDefault;
         end;
       m640x400x256,
       m640x480x256,
       m800x600x256,
       m1024x768x256,
       m1280x1024x256:
         begin
           DirectPutPixel:=@DirectPutPixVESA256Linear;
           PutPixel:=@PutPixVESA256Linear;
           GetPixel:=@GetPixVESA256Linear;
           { linear mode for lines not yet implemented PM }
           HLine:=@HLineDefault;
           VLine:=@VLineDefault;
           GetScanLine := @GetScanLineDefault;
           PatternLine := @PatternLineDefault;
         end;
     else
       exit;
     end;
     FrameBufferLinearAddress:=Get_linear_addr(VESAModeInfo.PhysAddress and $FFFF0000,
       VESAInfo.TotalMem shl 16);
{$ifdef logging}
     logln('framebuffer linear address: '+hexstr(FrameBufferLinearAddress div (1024*1024),8));
     logln('total mem shl 16: '+strf(vesainfo.totalmem shl 16));
{$endif logging}
     if int31error<>0 then
       begin
{$ifdef logging}
         logln('Unable to get linear address for '+hexstr(VESAModeInfo.PhysAddress,8));
{$endif logging}
         writeln(stderr,'Unable to get linear address for ',hexstr(VESAModeInfo.PhysAddress,8));
         exit;
       end;
     if UseNoSelector then
       begin
{         HookHeapError; }
         LFBPointer:=pointer(FrameBufferLinearAddress-get_segment_base_address(get_ds));
         if dword(LFBPointer)+dword(VESAInfo.TotalMem shl 16)-1 > dword(get_segment_limit(get_ds)) then
           set_segment_limit(get_ds,dword(LFBPointer)+dword(VESAInfo.TotalMem shl 16)-1);
       end
     else
       begin
         WinWriteSeg:=allocate_ldt_descriptors(1);
{$ifdef logging}
         logln('writeseg1: '+hexstr(winwriteseg,8));
{$endif logging}
         set_segment_base_address(WinWriteSeg,FrameBufferLinearAddress);
         set_segment_limit(WinWriteSeg,(VESAInfo.TotalMem shl 16)-1);
         lock_linear_region(FrameBufferLinearAddress,(VESAInfo.TotalMem shl 16));
         if int31error<>0 then
           begin
{$ifdef logging}
             logln('Error in linear memory selectors creation');
{$endif logging}
             writeln(stderr,'Error in linear memory selectors creation');
             exit;
           end;
       end;
     LinearPageOfs := 0;
     InLinear:=true;
     SetUpLinear:=true;
     { WinSize:=(VGAInfo.TotalMem shl 16);
     WinLoMask:=(VGAInfo.TotalMem shl 16)-1;
     WinShift:=15;
     Temp:=VGAInfo.TotalMem;
     while Temp>0 do
       begin
         inc(WinShift);
         Temp:=Temp shr 1;
       end; }
{$endif FPC}
   end;

  procedure SetupWindows(var ModeInfo: TVESAModeInfo);
   begin
     InLinear:=false;
<<<<<<< HEAD
<<<<<<< HEAD

     BytesPerLine := VESAModeInfo.BytesPerScanLine;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     { now we check the windowing scheme ...}
     if (ModeInfo.WinAAttr and WinSupported) <> 0 then
       { is this window supported ... }
       begin
         { now check if the window is R/W }
         if (ModeInfo.WinAAttr and WinReadable) <> 0 then
         begin
           ReadWindow := 0;
           WinReadSeg := ModeInfo.WinASeg;
         end;
         if (ModeInfo.WinAAttr and WinWritable) <> 0 then
         begin
           WriteWindow := 0;
           WinWriteSeg := ModeInfo.WinASeg;
         end;
       end;
     if (ModeInfo.WinBAttr and WinSupported) <> 0 then
       { is this window supported ... }
       begin

         { OPTIMIZATION ... }
         { if window A supports both read/write, then we try to optimize }
         { everything, by using a different window for Read and/or write.}
         if (WinReadSeg <> 0) and (WinWriteSeg <> 0) then
           begin
              { check if winB supports read }
              if (ModeInfo.WinBAttr and winReadable) <> 0 then
                begin
                  WinReadSeg := ModeInfo.WinBSeg;
                  ReadWindow := 1;
                end
              else
              { check if WinB supports write }
              if (ModeInfo.WinBAttr and WinWritable) <> 0 then
                begin
                  WinWriteSeg := ModeInfo.WinBSeg;
                  WriteWindow := 1;
                end;
           end
         else
         { Window A only supported Read OR Write, no we have to make }
         { sure that window B supports the other mode.               }
         if (WinReadSeg = 0) and (WinWriteSeg<>0) then
           begin
              if (ModeInfo.WinBAttr and WinReadable <> 0) then
                begin
                  ReadWindow := 1;
                  WinReadSeg := ModeInfo.WinBSeg;
                end
              else
                { impossible, this VESA mode is WRITE only! }
                begin
                  WriteLn('Invalid VESA Window attribute.');
                  Halt(255);
                end;
           end
         else
         if (winWriteSeg = 0) and (WinReadSeg<>0) then
           begin
             if (ModeInfo.WinBAttr and WinWritable) <> 0 then
               begin
                 WriteWindow := 1;
                 WinWriteSeg := ModeInfo.WinBSeg;
               end
             else
               { impossible, this VESA mode is READ only! }
               begin
                  WriteLn('Invalid VESA Window attribute.');
                  Halt(255);
               end;
           end
         else
         if (winReadSeg = 0) and (winWriteSeg = 0) then
         { no read/write in this mode! }
           begin
                  WriteLn('Invalid VESA Window attribute.');
                  Halt(255);
           end;
         YOffset := 0;
       end;

     { if both windows are not supported, then we can assume }
     { that there is ONE single NON relocatable window.      }
     if (WinWriteSeg = 0) and (WinReadSeg = 0) then
       begin
         WinWriteSeg := ModeInfo.WinASeg;
         WinReadSeg := ModeInfo.WinASeg;
       end;

    { 16-bit Protected mode checking code...  }
    { change segment values to protected mode }
    { selectors.                              }
    if WinReadSeg = $A000 then
      WinReadSeg := SegA000
    else
    if WinReadSeg = $B000 then
      WinReadSeg := SegB000
    else
    if WinReadSeg = $B800 then
      WinReadSeg := SegB800
    else
      begin
        WriteLn('Invalid segment address.');
        Halt(255);
      end;
    if WinWriteSeg = $A000 then
      WinWriteSeg := SegA000
    else
    if WinWriteSeg = $B000 then
      WinWriteSeg := SegB000
    else
    if WinWriteSeg = $B800 then
      WinWriteSeg := SegB800
    else
      begin
        WriteLn('Invalid segment address.');
        Halt(255);
      end;

   end;



  function setVESAMode(mode:word):boolean;
    var i:word;
        res: boolean;
  begin
   { Init mode information, for compatibility with VBE < 1.1 }
   FillChar(VESAModeInfo, sizeof(TVESAModeInfo), #0);
   { get the video mode information }
   if getVESAModeInfo(VESAmodeinfo, mode) then
   begin
     { checks if the hardware supports the video mode. }
     if (VESAModeInfo.attr and modeAvail) = 0 then
       begin
         SetVESAmode := FALSE;
{$ifdef logging}
         logln('  vesa mode '+strf(mode)+' not supported!!!');
{$endif logging}
         _GraphResult := grError;
         exit;
       end;

     SetVESAMode := TRUE;
     BankShift := 0;
     while (64 shr BankShift) <> VESAModeInfo.WinGranularity do
        Inc(BankShift);
     CurrentWriteBank := -1;
     CurrentReadBank := -1;
<<<<<<< HEAD
<<<<<<< HEAD
{    nickysn: setting BytesPerLine moved to SetupLinear and SetupWindowed
     BytesPerLine := VESAModeInfo.BytesPerScanLine;}
=======
     BytesPerLine := VESAModeInfo.BytesPerScanLine;
>>>>>>> graemeg/fixes_2_2
=======
     BytesPerLine := VESAModeInfo.BytesPerScanLine;
>>>>>>> origin/fixes_2_2

     { These are the window adresses ... }
     WinWriteSeg := 0;  { This is the segment to use for writes }
     WinReadSeg := 0;   { This is the segment to use for reads  }
     ReadWindow := 0;
     WriteWindow := 0;

     { VBE 2.0 and higher supports >= non VGA linear buffer types...}
     { this is backward compatible.                                 }
     if (((VESAModeInfo.Attr and ModeNoWindowed) <> 0) or UseLFB) and
          ((VESAModeInfo.Attr and ModeLinearBuffer) <> 0) then
        begin
          if not SetupLinear(VESAModeInfo,mode) then
            SetUpWindows(VESAModeInfo);
        end
     else
     { if linear and windowed is supported, then use windowed }
     { method.                                                }
        SetUpWindows(VESAModeInfo);

{$ifdef logging}
  LogLn('Entering vesa mode '+strf(mode));
  LogLn('Read segment: $'+hexstr(winreadseg,4));
  LogLn('Write segment: $'+hexstr(winwriteseg,4));
  LogLn('Window granularity: '+strf(VESAModeInfo.WinGranularity)+'kb');
  LogLn('Window size: '+strf(VESAModeInfo.winSize)+'kb');
  LogLn('Bytes per line: '+strf(bytesperline));
{$endif logging}
   { Select the correct mode number if we're going to use linear access! }
   if InLinear then
     inc(mode,$4000);

   asm
    mov ax,4F02h
    mov bx,mode
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
    sub ax,004Fh
    cmp ax,1
    sbb al,al
    mov res,al
   end ['EBX','EAX'];
   if not res then
     _GraphResult := GrNotDetected
   else _GraphResult := grOk;
  end;
 end;

(*
 function getVESAMode:word;assembler;
   asm  {return -1 if error}
    mov ax,4F03h
{$ifdef fpc}
    push ebx
    push ebp
    push esi
    push edi
{$endif fpc}
    int 10h
{$ifdef fpc}
    pop edi
    pop esi
    pop ebp
{$endif fpc}
    cmp ax,004Fh
    je @@OK
    mov ax,-1
    jmp @@X
  @@OK:
    mov ax,bx
  @@X:
{$ifdef fpc}
    pop ebx
{$endif fpc}
   end ['EAX'];
*)



 {************************************************************************}
 {*                     VESA Modes inits                                 *}
 {************************************************************************}

{$IFDEF DPMI}

  {******************************************************** }
  { Function GetMaxScanLines()                              }
  {-------------------------------------------------------- }
  { This routine returns the maximum number of scan lines   }
  { possible for this mode. This is done using the Get      }
  { Scan Line length VBE function.                          }
  {******************************************************** }
  function GetMaxScanLines: word;
   var
    regs : TDPMIRegisters;
   begin
     FillChar(regs, sizeof(regs), #0);
     { play it safe, call the real mode int, the 32-bit entry point }
     { may not be defined as stated in VBE v3.0                     }
     regs.eax := $4f06; {_ setup function      }
     regs.ebx := $0001; { get scan line length }
     RealIntr($10, regs);
     GetMaxScanLines := (regs.edx and $0000ffff);
   end;

{$ELSE}

  function GetMaxScanLines: word; assembler;
     asm
      mov ax, 4f06h
      mov bx, 0001h
      int 10h
      mov ax, dx
   end;

{$ENDIF}

 procedure Init1280x1024x64k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVesaMode(m1280x1024x64k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init1280x1024x32k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1280x1024x32k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init1280x1024x256; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1280x1024x256);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;


 procedure Init1280x1024x16; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1280x1024x16);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init1024x768x64k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1024x768x64k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init1024x768x32k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1024x768x32k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init1024x768x256; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1024x768x256);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init1024x768x16; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m1024x768x16);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init800x600x64k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m800x600x64k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init800x600x32k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m800x600x32k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init800x600x256; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m800x600x256);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init800x600x16; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVesaMode(m800x600x16);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init640x480x64k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m640x480x64k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init640x480x32k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m640x480x32k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init640x480x256; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m640x480x256);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init640x400x256; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m640x400x256);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init320x200x64k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m320x200x64k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;

 procedure Init320x200x32k; {$ifndef fpc}far;{$endif fpc}
  begin
    SetVESAMode(m320x200x32k);
    { Get maximum number of scanlines for page flipping }
    ScanLines := GetMaxScanLines;
  end;


{$IFDEF DPMI}

 Procedure SaveStateVESA; {$ifndef fpc}far;{$endif fpc}
 var
  PtrLong: longint;
  regs: TDPMIRegisters;
  begin
    SaveSupported := FALSE;
    SavePtr := nil;
{$ifdef logging}
        LogLn('Get the video mode...');
{$endif logging}
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
{$ifdef logging}
        LogLn('Prepare to save VESA video state');
{$endif logging}
    { Prepare to save video state...}
<<<<<<< HEAD
<<<<<<< HEAD
    regs.eax := $4F04;     { get buffer size to save state }
    regs.edx := $0000;
    regs.ecx := $000F;     { Save DAC / Data areas / Hardware states }
=======
=======
>>>>>>> origin/fixes_2_2
    asm
      mov  ax, 4F04h       { get buffer size to save state }
      mov  dx, 00h
      mov  cx, 00001111b   { Save DAC / Data areas / Hardware states }
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
      cmp  al,04fh
      jnz  @notok
      mov  [SaveSupported],TRUE
     @notok:
    end ['EDX','ECX','EAX'];
    regs.eax := $4f04;
    regs.edx := $0000;
    regs.ecx := $000F;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    RealIntr($10, regs);
    StateSize := word(regs.ebx);
    if byte(regs.eax) = $4f then
      SaveSupported := TRUE;
    if SaveSupported then
      begin
{$ifdef logging}
        LogLn('allocating VESA save buffer of '+strf(64*StateSize));
{$endif logging}
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
        regs.eax := $4F04;      { save the state buffer                   }
        regs.ecx := $0F;        { Save DAC / Data areas / Hardware states }
        regs.edx := $01;        { save state                              }
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        regs.eax := $4F04;      { restore the state buffer                }
        regs.ecx := $0F;        { rest DAC / Data areas / Hardware states }
        regs.edx := $02;
        regs.es := RealStateSeg;
        regs.ebx := 0;
        RealIntr($10,regs);
      end;
  end;

 procedure RestoreStateVESA; {$ifndef fpc}far;{$endif fpc}
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
     if assigned(SavePtr) and (SaveSupported=TRUE) then
{$else fpc}
     { No far pointer support, so it's possible that that assigned(SavePtr) }
     { would return false under FPC. Just check if it's different from nil. }
     if (SavePtr <> nil) and (SaveSupported=TRUE) then
{$endif fpc}
       begin
        FillChar(regs, sizeof(regs), #0);
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
         regs.eax := $4F04;      { restore the state buffer                }
         regs.ecx := $0F;        { rest DAC / Data areas / Hardware states }
         regs.edx := $02;        { restore state                           }
         regs.es := RealStateSeg;
         regs.ebx := 0;
         RealIntr($10,regs);
{$ifndef fpc}
         if GlobalDosFree(longint(SavePtr) shr 16)<>0 then
{$else fpc}
         if Not(Global_Dos_Free(longint(SavePtr) shr 16)) then
{$endif fpc}
          RunError(216);
         SavePtr := nil;
       end;
  end;

{$ELSE}

      {**************************************************************}
      {*                     Real mode routines                     *}
      {**************************************************************}

 Procedure SaveStateVESA; far;
  begin
    SavePtr := nil;
    SaveSupported := FALSE;
    { Get the video mode }
    asm
      mov  ah,0fh
      int  10h
      mov  [VideoMode], al
    end;
    { Prepare to save video state...}
    asm
      mov  ax, 4f04h       { get buffer size to save state }
      mov  cx, 00001111b   { Save DAC / Data areas / Hardware states }
      mov  dx, 00h
      int  10h
      mov  [StateSize], bx
      cmp  al,04fh
      jnz  @notok
      mov  [SaveSupported],TRUE
     @notok:
    end;
    if SaveSupported then
      Begin
        GetMem(SavePtr, 64*StateSize); { values returned in 64-byte blocks }
        if not assigned(SavePtr) then
           RunError(203);
        asm
         mov  ax, 4F04h       { save the state buffer                   }
         mov  cx, 00001111b   { Save DAC / Data areas / Hardware states }
         mov  dx, 01h
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
        { restore state, according to Ralph Brown Interrupt list }
        { some BIOS corrupt the hardware after a save...         }
        asm
         mov  ax, 4F04h       { save the state buffer                   }
         mov  cx, 00001111b   { Save DAC / Data areas / Hardware states }
         mov  dx, 02h
         mov  es, WORD PTR [SavePtr+2]
         mov  bx, WORD PTR [SavePtr]
         int  10h
        end;
      end;
  end;

 procedure RestoreStateVESA; far;
  begin
     { go back to the old video mode...}
     asm
      mov  ah,00
      mov  al,[VideoMode]
      int  10h
     end;

     { then restore all state information }
     if assigned(SavePtr) and (SaveSupported=TRUE) then
       begin
         { restore state, according to Ralph Brown Interrupt list }
         asm
           mov  ax, 4F04h       { save the state buffer                   }
           mov  cx, 00001111b   { Save DAC / Data areas / Hardware states }
           mov  dx, 02h         { restore state                           }
           mov  es, WORD PTR [SavePtr+2]
           mov  bx, WORD PTR [SavePtr]
           int  10h
         end;
         FreeMem(SavePtr, 64*StateSize);
         SavePtr := nil;
       end;
  end;
{$ENDIF DPMI}

 {************************************************************************}
 {*                     VESA Page flipping routines                      *}
 {************************************************************************}
 { Note: These routines, according  to the VBE3 specification, will NOT   }
 { work with the 24 bpp modes, because of the alignment.                  }
 {************************************************************************}

  {******************************************************** }
  { Procedure SetVisualVESA()                               }
  {-------------------------------------------------------- }
  { This routine changes the page which will be displayed   }
  { on the screen, since the method has changed somewhat    }
  { between VBE versions , we will use the old method where }
  { the new pixel offset is used to display different pages }
  {******************************************************** }
 procedure SetVisualVESA(page: word); {$ifndef fpc}far;{$endif fpc}
  var
   newStartVisible : word;
  begin
    if page > HardwarePages then
      begin
        _graphresult := grError;
        exit;
      end;
    newStartVisible := (MaxY+1)*page;
    if newStartVisible > ScanLines then
      begin
        _graphresult := grError;
        exit;
      end;
    asm
      mov ax, 4f07h
      mov bx, 0000h   { set display start }
      mov cx, 0000h   { pixel zero !      }
      mov dx, [NewStartVisible]  { new scanline }
{$ifdef fpc}
      push    ebp
      push    esi
      push    edi
      push    ebx
{$endif}
      int     10h
{$ifdef fpc}
      pop     ebx
      pop     edi
      pop     esi
      pop     ebp
{$endif}
    end ['EDX','ECX','EBX','EAX'];
  end;

 procedure SetActiveVESA(page: word); {$ifndef fpc}far;{$endif fpc}
  begin
    { video offset is in pixels under VESA VBE! }
    { This value is reset after a mode set to page ZERO = YOffset = 0 ) }
    if page > HardwarePages then
      begin
        _graphresult := grError;
        exit;
      end;
    YOffset := (MaxY+1)*page;
    LinearPageOfs := YOffset*(MaxX+1);
  end;

