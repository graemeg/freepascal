{$MODE objfpc}
{$ASMMODE intel}

{ $DEFINE DEBUGOUTPUT}

Unit vesa;

Interface

Type
  TVesaModeInfoBlock = Packed Record
    {Mandatory information for all VBE revisions}
    ModeAttributes : Word;        {mode attributes}
    WinAAttributes : Byte;        {window A attributes}
    WinBAttributes : Byte;        {window B attributes}
    WinGranularity : Word;        {window granularity}
    WinSize : Word;               {window size}
    WinASegment : Word;           {window A start segment}
    WinBSegment : Word;           {window B start segment}
    WinFuncPtr : DWord;           {real mode pointer to window function}
    BytesPerScanLine : Word;      {bytes per scan line}

    {Mandatory information for VBE 1.2 and above}
    XResolution : Word;           {horizontal resolution in pixels or characters}
    YResolution : Word;           {vertical resolution in pixels or characters}
    XCharSize : Byte;             {character cell width in pixels}
    YCharSize : Byte;             {character cell height in pixels}
    NumberOfPlanes : Byte;        {number of memory planes}
    BitsPerPixel : Byte;          {bits per pixel}
    NumberOfBanks : Byte;         {number of banks}
    MemoryModel : Byte;           {memory model type}
    BankSize : Byte;              {bank size in KB}
    NumberOfImagePages : Byte;    {number of images}
    Reserved : Byte;{=1}          {reserved for page function}

    {Direct color fields (required for direct/6 and YUV/7 memory models)}
    RedMaskSize : Byte;           {size of direct color red mask in bits}
    RedFieldPosition : Byte;      {bit position of lsb of red mask}
    GreenMaskSize : Byte;         {size of direct color green mask in bits}
    GreenFieldPosition : Byte;    {bit position of lsb of green mask}
    BlueMaskSize : Byte;          {size of direct color blue mask in bits}
    BlueFieldPosition : Byte;     {bit position of lsb of blue mask}
    RsvdMaskSize : Byte;          {size of direct color reserved mask in bits}
    RsvdFieldPosition : Byte;     {bit position of lsb of reserved mask}
    DirectColorModeInfo : Byte;   {direct color mode attributes}

    {Mandatory information for VBE 2.0 and above}
    PhysBasePtr : DWord;          {physical address for flat memory frame buffer}
    Reserved2 : DWord;{=0}        {Reserved - always set to 0}
    Reserved3 : Word;{=0}         {Reserved - always set to 0}

    {Mandatory information for VBE 3.0 and above}
    LinBytesPerScanLine : Word;   {bytes per scan line for linear modes}
    BnkNumberOfImagePages : Byte; {number of images for banked modes}
    LinNumberOfImagePages : Byte; {number of images for linear modes}
    LinRedMaskSize : Byte;        {size of direct color red mask (linear modes)}
    LinRedFieldPosition : Byte;   {bit position of lsb of red mask (linear modes)}
    LinGreenMaskSize : Byte;      {size of direct color green mask (linear modes)}
    LinGreenFieldPosition : Byte; {bit position of lsb of green mask (linear modes)}
    LinBlueMaskSize : Byte;       {size of direct color blue mask (linear modes)}
    LinBlueFieldPosition : Byte;  {bit position of lsb of blue mask (linear modes)}
    LinRsvdMaskSize : Byte;       {size of direct color reserved mask (linear modes)}
    LinRsvdFieldPosition : Byte;  {bit position of lsb of reserved mask (linear modes)}
    MaxPixelClock : DWord;        {maximum pixel clock (in Hz) for graphics mode}

    Reserved4 : Array[1..189] Of Byte; {remainder of ModeInfoBlock}
  End;
  PModeInfo = ^TModeInfo;
  TModeInfo = Record
    ModeNumber : DWord;
    VesaModeInfo : TVesaModeInfoBlock;
  End;

Var
  ModeInfo : PModeInfo;
  NrOfModes : Integer;
  VBEPresent : Boolean;

Procedure InitVESA;
Function SetVESAMode(M : Integer) : Boolean;
Procedure RestoreTextMode;
Procedure WriteToVideoMemory(Src : Pointer; Dest : DWord; Size : DWord);
Procedure SetPalette(Palette : Pointer; First, Num : Integer);
Procedure GetPalette(Palette : Pointer; First, Num : Integer);
Function MakeMask(MaskSize, FieldPosition : Integer) : DWord;

Implementation

Uses
  go32;

Type
  TVBEInfoBlock = Packed Record
    {VBE 1.0+}
    VBESignature : DWord; {'VESA'}
    VBEVersion : Word;
    OemStringPtr : DWord; {VbeFarPtr to OEM String}
    Capabilities : DWord; {Capabilities of graphics controller}
    VideoModePtr : DWord; {VbeFarPtr to VideoModeList}
    {added for VBE 1.1+}
    TotalMemory : Word; {Number of 64kb memory blocks}
    {added for VBE 2.0+}
    OemSoftwareRev : Word; {VBE implementation Software revision}
    OemVendorNamePtr : DWord; {VbeFarPtr to Vendor Name String}
    OemProductNamePtr : DWord; {VbeFarPtr to Product Name String}
    OemProductRevPtr : DWord; {VbeFarPtr to Product Revision String}
    Reserved : Array[1..222] Of Byte; {Reserved for VBE implementation scratch area}
    OemData : Array[1..256] Of Char; {Data Area for OEM Strings}
  End;

Var
  VBEInfoBlock : TVBEInfoBlock;
  VideoMemory : DWord;
  EightBitDACSupported : Boolean;
  nonVGA : Boolean;
  SnowyRAMDAC : Boolean;
  StereoSignalingSupport : Boolean;
  StereoSignalingVesaEVC : Boolean;
  OEMString : String;
  OEMVendorName : String;
  OEMProductName : String;
  OEMProductRev : String;
  OEMSoftwareRev : Integer;
  CurrentMode : Integer;
  LFBUsed : Boolean;
  UseLFB : Boolean;

  RealModePaletteSel : Word;
  RealModePaletteSeg : Word;
  SetPaletteHW : Boolean;
  PaletteDACbits : Integer;

  ReadWindow, WriteWindow : Integer;
  ReadWindowStart, WriteWindowStart : Integer;
  ReadWindowAddress, WriteWindowAddress : Integer;
  WindowGranularity : DWord;
  WindowSize, WindowSizeG : DWord;

  VESAInit : Boolean;

  RealRegs : TRealRegs;

  temp : Pointer;

Procedure StandardMode(ModeNumber : DWord; Var ModeInfo : TVesaModeInfoBlock);

Begin
{
100 640x400x256
101 640x480x256
102 800x600x16
103 800x600x256
104 1024x768x16
105 1024x768x256
106 1280x1024x16
107 1280x1024x256
108 80x60t
109 132x25t
10A 132x43t
10B 132x50t
10C 132x60t
10D 320x200x32k
10E 320x200x64k
10F 320x200x16.8m
110 640x480x32k
111 640x480x64k
112 640x480x16.8m
113 800x600x32k
114 800x600x64k
115 800x600x16.8m
116 1024x768x32k
117 1024x768x64k
118 1024x768x16.8m
119 1280x1024x32k
11A 1280x1024x64k
11B 1280x1024x16.8m
}
  With ModeInfo Do
  Begin
    ModeAttributes := ModeAttributes Or 2;
    Case ModeNumber Of
      $100 : Begin
        XResolution := 640;
        YResolution := 400;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 1;
        BitsPerPixel := 8;
        MemoryModel := 4;
      End;
      $101 : Begin
        XResolution := 640;
        YResolution := 480;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 1;
        BitsPerPixel := 8;
        MemoryModel := 4;
      End;
      $102 : Begin
        XResolution := 800;
        YResolution := 600;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 3;
      End;
      $103 : Begin
        XResolution := 800;
        YResolution := 600;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 1;
        BitsPerPixel := 8;
        MemoryModel := 4;
      End;
      $104 : Begin
        XResolution := 1024;
        YResolution := 768;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 3;
      End;
      $105 : Begin
        XResolution := 1024;
        YResolution := 768;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 1;
        BitsPerPixel := 8;
        MemoryModel := 4;
      End;
      $106 : Begin
        XResolution := 1280;
        YResolution := 1024;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 3;
      End;
      $107 : Begin
        XResolution := 1280;
        YResolution := 1024;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 1;
        BitsPerPixel := 8;
        MemoryModel := 4;
      End;
      $108 : Begin
        XResolution := 80;
        YResolution := 60;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 0;
      End;
      $109 : Begin
        XResolution := 132;
        YResolution := 25;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 0;
      End;
      $10A : Begin
        XResolution := 132;
        YResolution := 43;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 0;
      End;
      $10B : Begin
        XResolution := 132;
        YResolution := 50;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 0;
      End;
      $10C : Begin
        XResolution := 132;
        YResolution := 60;
        XCharSize := 8;
        YCharSize := 16;
        NumberOfPlanes := 4;
        BitsPerPixel := 4;
        MemoryModel := 0;
      End;
      {todo:10D..11B}
      Else
        ModeAttributes := ModeAttributes And $FFFD;
    End;
//        NumberOfImagePages := 0;{...}
  End;
End;

Function bcd(q : Integer) : Integer;

Begin
  q := q And $FF;
  If ((q And $F) < 10) And ((q Shr 4) < 10) Then
    bcd := (q And $F) + (q Shr 4) * 10
  Else
    bcd := q;
End;

Procedure DisposeRealModePalette;

Begin
  If RealModePaletteSel = 0 Then
    Exit;
  global_dos_free(RealModePaletteSel);
  RealModePaletteSel := 0;
  RealModePaletteSeg := 0;
End;

Procedure AllocateRealModePalette;

Var
  Addr : DWord;

Begin
  DisposeRealModePalette;
  Addr := global_dos_alloc(256*4);
  RealModePaletteSeg := Addr Shr 16;
  RealModePaletteSel := Addr And $FFFF;
End;

Procedure SetPalette2(Palette : Pointer; Num : Integer); Assembler;

Asm
  push es

  cld
  mov ax, fs
  mov es, ax
  mov esi, [Palette]
  movzx edi, word [RealModePaletteSeg]
  shl edi, 4
  mov ecx, Num
{  mov edx, 03F3F3F3Fh}
  mov edx, 0003F3F3Fh

@@1:
  lodsd

  shr eax, 2 {convert 8->6bit}
  and eax, edx

  stosd
  dec ecx
  jnz @@1

  pop es
End;

Procedure SetPalette3(Palette : Pointer; Num : Integer); Assembler;

Asm
  push es

  cld
  mov ax, fs
  mov es, ax
  mov esi, [Palette]
  movzx edi, word [RealModePaletteSeg]
  shl edi, 4
  mov ecx, Num
{  mov edx, 07F7F7F7Fh}
  mov edx, 0007F7F7Fh

@@1:
  lodsd

  shr eax, 1 {convert 8->7bit}
  and eax, edx

  stosd
  dec ecx
  jnz @@1

  pop es
End;

Procedure SetPaletteHW6(Palette : Pointer; First, Num : Integer);

Var
  I : Integer;
  p : PDWord;
  c : DWord;

Begin
  p := PDWord(Palette);
  outportb($3C8, First);
  While Num > 0 Do
  Begin
    c := (p^ Shr 2) And $3F3F3F;
    outportb($3C9, c Shr 16);
    outportb($3C9, c Shr 8);
    outportb($3C9, c);

    Inc(p);
    Dec(Num);
  End;
End;

Procedure SetPaletteHW7(Palette : Pointer; First, Num : Integer);

Var
  I : Integer;
  p : PDWord;
  c : DWord;

Begin
  p := PDWord(Palette);
  outportb($3C8, First);
  While Num > 0 Do
  Begin
    c := (p^ Shr 1) And $7F7F7F;
    outportb($3C9, c Shr 16);
    outportb($3C9, c Shr 8);
    outportb($3C9, c);

    Inc(p);
    Dec(Num);
  End;
End;

Procedure SetPaletteHW8(Palette : Pointer; First, Num : Integer);

Var
  I : Integer;
  p : PDWord;

Begin
  p := PDWord(Palette);
  outportb($3C8, First);
  While Num > 0 Do
  Begin
    outportb($3C9, p^ Shr 16);
    outportb($3C9, p^ Shr 8);
    outportb($3C9, p^);

    Inc(p);
    Dec(Num);
  End;
End;

Procedure SetPalette(Palette : Pointer; First, Num : Integer);

Begin
  If SetPaletteHW Then
  Begin
    Case PaletteDACbits Of
      8 : SetPaletteHW8(Palette, First, Num);
      7 : SetPaletteHW7(Palette, First, Num);
      6 : SetPaletteHW6(Palette, First, Num);
    End;
  End
  Else
  Begin
    If PaletteDACbits = 8 Then
      dosmemput(RealModePaletteSeg, 0, Palette^, Num * 4) {8bits}
    Else
      If PaletteDACbits = 7 Then
        SetPalette3(Palette, Num) {7bits}
      Else
        SetPalette2(Palette, Num); {6bits}
    RealRegs.ax := $4F09;
    RealRegs.bl := 0;
    RealRegs.cx := Num;
    RealRegs.dx := First;
    RealRegs.es := RealModePaletteSeg;
    RealRegs.di := 0;
    realintr($10, RealRegs);
  End;
End;

Procedure GetPalette(Palette : Pointer; First, Num : Integer);

Begin
  RealRegs.ax := $4F09;
  RealRegs.bl := 1;
  RealRegs.cx := Num;
  RealRegs.dx := First;
  RealRegs.es := RealModePaletteSeg;
  RealRegs.di := 0;
  realintr($10, RealRegs);
  {...}
End;

Procedure SwitchTo8bitDAC;

Begin
  RealRegs.ax := $4F08;
  RealRegs.bl := 0;
  RealRegs.bh := 8;
  realintr($10, RealRegs);
  PaletteDACbits := RealRegs.bh;
  If PaletteDACbits < 6 Then
    PaletteDACbits := 6;
End;

Function MakeMask(MaskSize, FieldPosition : Integer) : DWord;

Var
  Mask : DWord;
  I : Integer;

Begin
  Mask := 1 Shl FieldPosition;
  For I := 2 To MaskSize Do
    Mask := Mask Or (Mask Shl 1);
  MakeMask := Mask;
End;

Function GetRMString(SegOfs : DWord) : String;

Var
  S : String;
  C : Char;
  Seg, Ofs : Word;

Begin
  If SegOfs = 0 Then
  Begin
    GetRMString := '';
    Exit;
  End;
  S := '';
  Ofs := SegOfs And $FFFF;
  Seg := SegOfs Shr 16;
  Repeat
    dosmemget(Seg, Ofs, C, 1);
    If C <> #0 Then
    Begin
      S := S + C;
      If Ofs = $FFFF Then
      Begin
        Ofs := 0;
        Inc(Seg, $1000);
      End
      Else
        Inc(Ofs);
    End;
  Until C = #0;
  GetRMString := S;
End;

Procedure SetWriteWindowStart(WinPos : DWord);

Begin
  RealRegs.ax := $4F05;
  RealRegs.bx := WriteWindow;
  RealRegs.dx := WinPos;
  realintr($10, RealRegs);
End;

Procedure WriteToVideoMemory(Src : Pointer; Dest : DWord; Size : DWord);

Var
  WW : Integer;
  ToDo : Integer;

Begin
  WW := Dest Div WindowGranularity;
  Dest := Dest Mod WindowGranularity;
{  Writeln(WindowSize);}
  While Size > 0 Do
  Begin
{    Write(WW, ' ');}
    SetWriteWindowStart(WW);
    ToDo := WindowSize - Dest;
    If Size < ToDo Then
      ToDo := Size;
    Asm
      push es
      mov esi, Src
      mov edi, Dest
      add edi, WriteWindowAddress
      mov ax, fs
      mov es, ax
      mov ecx, ToDo
      shr ecx, 2
      cld
      rep movsd
      mov ecx, ToDo
      and ecx, 3
      jz @@1
      rep movsb
@@1:
      pop es
    End ['EAX', 'ECX', 'ESI', 'EDI'];
    Dest := 0;
    Inc(WW, WindowSizeG);
{    Inc(WW);}
    Inc(Src, ToDo);
    Dec(Size, ToDo);
  End;
End;

{$IFDEF DEBUGOUTPUT}
Procedure WinAttrib(q : Integer);

Begin
  If (q And 1) <> 0 Then
    Write(' supported')
  Else
    Write(' not_supported');
  If (q And 2) <> 0 Then
    Write(' readable');
  If (q And 4) <> 0 Then
    Write(' writeable');
  Writeln;
End;
{$ENDIF DEBUGOUTPUT}

Procedure GetModes;

Type
  PModesList = ^TModesList;
  TModesList = Record
    ModeInfo : TModeInfo;
    Next : PModesList;
  End;

Var
  First, Last, Run, Tmp : PModesList;

  Procedure AddToList;

  Begin
    If Last = Nil Then
    Begin
      New(Last);
      First := Last;
    End
    Else
    Begin
      New(Last^.Next);
      Last := Last^.Next;
      Last^.Next := Nil;
    End;
  End;

Var
  I : DWord;
  Addr : DWord;
  AddrSeg, AddrSel : Word;
  VesaModeInfo : TVesaModeInfoBlock;
  ScanStart, ScanEnd : Integer;
  ModeAttr : Integer;
  IsModeOk : Boolean;
  hasReadWindow, hasWriteWindow : Boolean;

Begin
  NrOfModes := -1;
  First := Nil;
  Last := Nil;
  Addr := global_dos_alloc(512);
  AddrSeg := Addr Shr 16;
  AddrSel := Addr And $FFFF;
  ScanStart := 0;
{  ScanEnd := $7FFF;} {VBE 1.0+ ??}
{  ScanEnd := $3FFF;} {VBE 1.2+ ??}
  ScanEnd := $7FF; {VBE 3.0+}
  {$IFDEF DEBUGOUTPUT}
  Writeln('scanning modes $', HexStr(ScanStart, 4), '..$', HexStr(ScanEnd, 4));
  {$ENDIF DEBUGOUTPUT}
  For I := ScanStart To ScanEnd Do
  Begin
    FillChar(VesaModeInfo, SizeOf(VesaModeInfo), 0);
    dosmemput(AddrSeg, 0, VesaModeInfo, SizeOf(VesaModeInfo));
    RealRegs.ax := $4F01; {return VBE mode information}
    RealRegs.cx := I;
    RealRegs.es := AddrSeg;
    RealRegs.di := 0;
    realintr($10, RealRegs);
    dosmemget(AddrSeg, 0, VesaModeInfo, SizeOf(VesaModeInfo));

    {display mode info}
    {$IFDEF DEBUGOUTPUT}
    If ((VesaModeInfo.ModeAttributes And 1) <> 0) Or
       (VesaModeInfo.BytesPerScanLine <> 0) Then
    Begin
      Writeln('ModeNumber: $', HexStr(I, 4));
      Write('ModeAttributes:');
      If (VesaModeInfo.ModeAttributes And 1) <> 0 Then
        Write(' supported')
      Else
        Write(' not_supported');
      If (VesaModeInfo.ModeAttributes And 2) <> 0 Then
        Write('')
      Else
        Write(' reserved_is_zero(noresolutioninfo_for_vbe1.1-)');
      If (VesaModeInfo.ModeAttributes And 4) <> 0 Then
        Write(' TTY')
      Else
        Write(' noTTY');
      If (VesaModeInfo.ModeAttributes And 8) <> 0 Then
        Write(' color')
      Else
        Write(' monochrome');
      If (VesaModeInfo.ModeAttributes And 16) <> 0 Then
        Write(' graph')
      Else
        Write(' text');
      If (VesaModeInfo.ModeAttributes And 32) <> 0 Then
        Write(' nonVGA')
      Else
        Write(' VGA');
      If (VesaModeInfo.ModeAttributes And 64) <> 0 Then
        Write(' noWINDOWED')
      Else
        Write(' WINDOWED');
      If (VesaModeInfo.ModeAttributes And 128) <> 0 Then
        Write(' LFB')
      Else
        Write(' noLFB');
      If (VesaModeInfo.ModeAttributes And 256) <> 0 Then
        Write(' DoubleScanMode_is_available')
      Else
        Write('');
      If (VesaModeInfo.ModeAttributes And 512) <> 0 Then
        Write(' InterlacedMode_is_available')
      Else
        Write('');
      If (VesaModeInfo.ModeAttributes And 1024) <> 0 Then
        Write(' TripleBuffering')
      Else
        Write('');
      If (VesaModeInfo.ModeAttributes And 2048) <> 0 Then
        Write(' StereoscopicDisplaySupport')
      Else
        Write('');
      If (VesaModeInfo.ModeAttributes And 4096) <> 0 Then
        Write(' DualDisplayStartAddressSupport')
      Else
        Write('');
      Writeln;

      Write('WinAAtributes:');
      WinAttrib(VesaModeInfo.WinAAttributes);
      Write('WinBAttributes:');
      WinAttrib(VesaModeInfo.WinBAttributes);
      Writeln('WinGranularity: ', VesaModeInfo.WinGranularity, ' KB');
      Writeln('WinSize: ', VesaModeInfo.WinSize, ' KB');
      Writeln('WinASegment: $', HexStr(VesaModeInfo.WinASegment, 4));
      Writeln('WinBSegment: $', HexStr(VesaModeInfo.WinBSegment, 4));
      Writeln('WinFuncPtr: ', HexStr(VesaModeInfo.WinFuncPtr Shr 16, 4), ':', HexStr(VesaModeInfo.WinFuncPtr And $FFFF, 4));
      Writeln('BytesPerScanLine: ', VesaModeInfo.BytesPerScanLine);
      Writeln('vbe1.2+');
      Writeln('XResolution: ', VesaModeInfo.XResolution);
      Writeln('YResolution: ', VesaModeInfo.YResolution);
      Writeln('XCharSize: ', VesaModeInfo.XCharSize);
      Writeln('YCharSize: ', VesaModeInfo.YCharSize);
      Writeln('NumberOfPlanes: ', VesaModeInfo.NumberOfPlanes);
      Writeln('BitsPerPixel: ', VesaModeInfo.BitsPerPixel);
      Writeln('NumberOfBanks: ', VesaModeInfo.NumberOfBanks);
      Write('MemoryModel: ');
      Case VesaModeInfo.MemoryModel Of
        0 : Write('Text mode');
        1 : Write('CGA graphics');
        2 : Write('Hercules graphics');
        3 : Write('Planar');
        4 : Write('Packed pixel');
        5 : Write('Non-chain 4, 256 color');
        6 : Write('Direct Color');
        7 : Write('YUV');
        8..15 : Write('Reserved, to be defined by VESA');
        Else
          Write('To be defined by OEM');
      End;
      Writeln('/', VesaModeInfo.MemoryModel);
      Writeln('BankSize: ', VesaModeInfo.BankSize, ' KB');
      Writeln('NumberOfImagePages: ', VesaModeInfo.NumberOfImagePages);
      Writeln('Reserved(=1): ', VesaModeInfo.Reserved);
      Writeln('RedMaskSize: ', VesaModeInfo.RedMaskSize);
      Writeln('RedFieldPosition: ', VesaModeInfo.RedFieldPosition);
      Writeln('GreenMaskSize: ', VesaModeInfo.GreenMaskSize);
      Writeln('GreenFieldPosition: ', VesaModeInfo.GreenFieldPosition);
      Writeln('BlueMaskSize: ', VesaModeInfo.BlueMaskSize);
      Writeln('BlueFieldPosition: ', VesaModeInfo.BlueFieldPosition);
      Writeln('RsvdMaskSize: ', VesaModeInfo.RsvdMaskSize);
      Writeln('RsvdFieldPosition: ', VesaModeInfo.RsvdFieldPosition);
      Write('DirectColorModeInfo:');
      If (VesaModeInfo.DirectColorModeInfo And 1) <> 0 Then
        Write(' Color_ramp_is_programmable')
      Else
        Write(' Color_ramp_is_fixed');
      If (VesaModeInfo.DirectColorModeInfo And 2) <> 0 Then
        Write(' Rsvd_bits_usable_by_app')
      Else
        Write(' Rsvd_bits_reserved');
      Writeln;
      Writeln('vbe2.0+');
      Writeln('PhysBasePtr: $', HexStr(VesaModeInfo.PhysBasePtr, 8));
      Writeln('Reserved2(=0): ', VesaModeInfo.Reserved2);
      Writeln('Reserved3(=0): ', VesaModeInfo.Reserved3);

      Writeln;
{      Write(VesaModeInfo.XResolution, 'x', VesaModeInfo.YResolution, 'x',
            VesaModeInfo.BitsPerPixel, '-', VesaModeInfo.MemoryModel,
            'R', VesaModeInfo.RedMaskSize, ':', VesaModeInfo.RedFieldPosition,
            'G', VesaModeInfo.GreenMaskSize, ':', VesaModeInfo.GreenFieldPosition,
            'B', VesaModeInfo.BlueMaskSize, ':', VesaModeInfo.BlueFieldPosition,
            'A', VesaModeInfo.RsvdMaskSize, ':', VesaModeInfo.RsvdFieldPosition, ' ');}
    End;
    {$ENDIF DEBUGOUTPUT}
    {/display mode info}

    If (VesaModeInfo.ModeAttributes And 1) <> 0 Then
    Begin
      If (VesaModeInfo.ModeAttributes And 2) = 0 Then
      Begin
        If VBEInfoBlock.VBEVersion >= $0102 Then
          IsModeOk := False
        Else
          StandardMode(I, VesaModeInfo);
      End;
      ModeAttr := (VesaModeInfo.ModeAttributes And $C0) Shr 6;
      IsModeOk := True;
      If ModeAttr = 1 Then
        IsModeOk := False;
      If IsModeOk And ((ModeAttr = 0) Or (ModeAttr = 2)) Then
      Begin {check windowed}
        hasReadWindow := False;
        hasWriteWindow := False;
        If (VesaModeInfo.WinAAttributes And $01) <> 0 Then
        Begin
          If (VesaModeInfo.WinAAttributes And $02) <> 0 Then
            hasReadWindow := True;
          If (VesaModeInfo.WinAAttributes And $04) <> 0 Then
            hasWriteWindow := True;
        End;
        If (VesaModeInfo.WinBAttributes And $01) <> 0 Then
        Begin
          If (VesaModeInfo.WinBAttributes And $02) <> 0 Then
            hasReadWindow := True;
          If (VesaModeInfo.WinBAttributes And $04) <> 0 Then
            hasWriteWindow := True;
        End;
        If (Not hasReadWindow) Or (Not hasWriteWindow) Then
          IsModeOk := False;
      End;
      If IsModeOk And ((ModeAttr = 2) Or (ModeAttr = 3)) Then
      Begin {check lfb...}
        {...}
      End;

      If IsModeOk Then
      Begin
//        Write(HexStr(I, 4), ' ');
        AddToList;
        Inc(NrOfModes);
        Last^.ModeInfo.ModeNumber := I;
        Last^.ModeInfo.VesaModeInfo := VesaModeInfo;
      End;
    End;
  End;
  global_dos_free(AddrSel);
  If ModeInfo <> Nil Then
    FreeMem(ModeInfo);
  If NrOfModes <> -1 Then
    ModeInfo := GetMem((NrOfModes + 1) * SizeOf(TModeInfo))
  Else
    ModeInfo := Nil;
  Run := First;
  For I := 0 To NrOfModes Do
  Begin
    ModeInfo[I] := Run^.ModeInfo;
    Tmp := Run;
    Run := Run^.Next;
    Dispose(Tmp);
  End;
  {$IFDEF DEBUGOUTPUT}
  Writeln;
  {$ENDIF DEBUGOUTPUT}
End;

Procedure GetVBEInfo;

Var
  Addr : DWord;
  AddrSeg : Word;
  AddrSel : Word;
  tmp : DWord;

Begin
  Addr := global_dos_alloc(512);
  AddrSeg := Addr Shr 16;
  AddrSel := Addr And $FFFF;
  VBEInfoBlock.VBESignature := $32454256; {'VBE2'}
  dosmemput(AddrSeg, 0, VBEInfoBlock, 4);
  RealRegs.ax := $4F00;
  RealRegs.es := AddrSeg;
  RealRegs.di := 0;
  realintr($10, RealRegs);
  VBEPresent := RealRegs.al = $4F;
  If VBEPresent Then
  Begin
    dosmemget(AddrSeg, 0, VBEInfoBlock, SizeOf(VBEInfoBlock));
    {todo: check for 'VESA' id string}
    VideoMemory := VBEInfoBlock.TotalMemory * 64;
    EightBitDACSupported := (VBEInfoBlock.Capabilities And 1) <> 0;
    nonVGA := (VBEInfoBlock.Capabilities And 2) <> 0;
    SnowyRAMDAC := (VBEInfoBlock.Capabilities And 4) <> 0;
    StereoSignalingSupport := (VBEInfoBlock.Capabilities And 8) <> 0;
    StereoSignalingVesaEVC := (VBEInfoBlock.Capabilities And 16) <> 0;
    OEMString := GetRMString(VBEInfoBlock.OemStringPtr);
    If VBEInfoBlock.VBEVersion >= $0200 Then
    Begin
      OEMVendorName := GetRMString(VBEInfoBlock.OemVendorNamePtr);
      OEMProductName := GetRMString(VBEInfoBlock.OemProductNamePtr);
      OEMProductRev := GetRMString(VBEInfoBlock.OemProductRevPtr);
      OEMSoftwareRev := VBEInfoBlock.OemSoftwareRev;
    End
    Else
    Begin
      OEMVendorName := '';
      OEMProductName := '';
      OEMProductRev := '';
      OEMSoftwareRev := -1;
    End;
  End;
  global_dos_free(AddrSel);

  {$IFDEF DEBUGOUTPUT}
  If VBEPresent Then
  Begin
    Writeln('VBEVersion: ', bcd(VBEInfoBlock.VBEVersion Shr 8), '.', bcd(VBEInfoBlock.VBEVersion And $FF));
    Writeln('VideoMemory: ', VideoMemory, ' KB');
    Writeln('EightBitDACSupported: ', EightBitDACSupported);
    Writeln('nonVGA: ', nonVGA);
    Writeln('SnowyRAMDAC: ', SnowyRAMDAC);
    Writeln('StereoSignalingSupport: ', StereoSignalingSupport);
    If StereoSignalingSupport Then
     If StereoSignalingVesaEVC Then
       Writeln('Stereo signaling supported via VESA EVC connector')
     Else
       Writeln('Stereo signaling supported via external VESA stereo connector');
    If OEMString <> '' Then
      Writeln('OEMString: ', OEMString);
    If OEMVendorName <> '' Then
      Writeln('OEMVendorName: ', OEMVendorName);
    If OEMProductName <> '' Then
      Writeln('OEMProductName: ', OEMProductName);
    If OEMProductRev <> '' Then
      Writeln('OEMProductRev: ', OEMProductRev);
    If OEMSoftwareRev <> -1 Then
      Writeln('OEMSoftwareRev: ', bcd(OEMSoftwareRev Shr 8), '.', bcd(OEMSoftwareRev And $FF));
    Write('VideoModeList:');
    tmp := (VBEInfoBlock.VideoModePtr Shr 16) * 16 + (VBEInfoBlock.VideoModePtr And $FFFF);
    While MemW[tmp] <> $FFFF Do
    Begin
      Write(' $', HexStr(MemW[tmp], 4));
      Inc(tmp, 2);
    End;
    Writeln;
    Writeln;
  End;
  {$ENDIF DEBUGOUTPUT}
End;

Function SetVESAMode(M : Integer) : Boolean;

Var
  ModeAttr : DWord;
  lLFBUsed : Boolean;
  lReadWindow, lWriteWindow : Integer;
  lReadWindowStart, lWriteWindowStart : Integer;
  lReadWindowAddress, lWriteWindowAddress : Integer;
  lWindowGranularity : DWord;
  lWindowSize, lWindowSizeG : DWord;

Begin
  SetVESAMode := False;
  DisposeRealModePalette;
  ModeAttr := (ModeInfo[M].VesaModeInfo.ModeAttributes And $C0) Shr 6;
  Case ModeAttr Of
    0 : lLFBUsed := False; {windowed frame buffer only}
    2 : lLFBUsed := UseLFB; {both windowed and linear}
    3 : lLFBUsed := True; {linear frame buffer only}
  End;
  If Not lLFBUsed Then
  Begin
    With ModeInfo[M].VesaModeInfo Do
    Begin
      lReadWindow := -1;
      lWriteWindow := -1;
      If (WinAAttributes And $01) <> 0 Then
      Begin
        If (WinAAttributes And $02) <> 0 Then
          lReadWindow := 0;
        If (WinAAttributes And $04) <> 0 Then
          lWriteWindow := 0;
      End;
      If (lReadWindow = -1) Or (lWriteWindow = -1) Then
        If (WinBAttributes And $01) <> 0 Then
        Begin
          If (lReadWindow = -1) And ((WinBAttributes And $02) <> 0) Then
            lReadWindow := 1;
          If (lWriteWindow = -1) And ((WinBAttributes And $04) <> 0) Then
            lWriteWindow := 1;
        End;
      Case lReadWindow Of
        -1 : Exit{err};
        0 : lReadWindowAddress := WinASegment Shl 4;
        1 : lReadWindowAddress := WinBSegment Shl 4;
      End;
      Case lWriteWindow Of
        -1 : Exit{err};
        0 : lWriteWindowAddress := WinASegment Shl 4;
        1 : lWriteWindowAddress := WinBSegment Shl 4;
      End;
      lWindowGranularity := WinGranularity * 1024;
      lWindowSize := WinSize * 1024;
      lWindowSizeG := lWindowSize Div lWindowGranularity;
      lWindowSize := lWindowSizeG * lWindowGranularity;
    End;
  End
  Else
  Begin
    {TODO: lfb}
  End;
  RealRegs.ax := $4F02;
  If lLFBUsed Then
    RealRegs.bx := ModeInfo[M].ModeNumber Or $4000
  Else
    RealRegs.bx := ModeInfo[M].ModeNumber;
  realintr($10, RealRegs);
  PaletteDACbits := 6;
  With ModeInfo[M].VesaModeInfo Do
  Begin
    If (BitsPerPixel = 8) And (MemoryModel = 4{packed pixel}) Then
    Begin
      SetPaletteHW := True;
      If (VBEInfoBlock.VBEVersion >= $200) And
         ((ModeAttributes And 32) <> 0) Then {if nonVGA, use func9 to set palette}
        SetPaletteHW := False;

      If EightBitDACSupported Then
        SwitchTo8bitDAC;

      If Not SetPaletteHW Then
        AllocateRealModePalette;
    End;
  End;

  LFBUsed := lLFBUsed;
  ReadWindow := lReadWindow;
  WriteWindow := lWriteWindow;
  ReadWindowStart := lReadWindowStart;
  WriteWindowStart := lWriteWindowStart;
  ReadWindowAddress := lReadWindowAddress;
  WriteWindowAddress := lWriteWindowAddress;
  WindowGranularity := lWindowGranularity;
  WindowSize := lWindowSize;
  WindowSizeG := lWindowSizeG;

  SetVESAMode := True;
End;

Procedure RestoreTextMode;

Begin
  DisposeRealModePalette;
  RealRegs.ax := $0003;
  realintr($10, RealRegs);
End;

Procedure InitVESA;

Begin
  If Not VESAInit Then
    VESAInit := True
  Else
    Exit;
  GetVBEInfo;
  If VBEPresent Then
    GetModes;
End;

Initialization
  VESAInit := False;
  CurrentMode := -1;
  UseLFB := {True}False;
  ModeInfo := Nil;
  RealModePaletteSel := 0;
  RealModePaletteSeg := 0;

Finalization
  temp := ModeInfo;
  ModeInfo := Nil;
  If temp <> Nil Then
    FreeMem(temp);
  DisposeRealModePalette;

End.
