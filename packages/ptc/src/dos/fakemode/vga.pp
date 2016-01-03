{$MODE objfpc}
{$ASMMODE intel}

Unit vga;

Interface

Const
{mode types}
  FAKEMODE = 0;
  RGB332 = 1;
  INDEX8 = 2;
{fakemode types}
  FAKEMODE1A = 0;
  FAKEMODE1B = 1;
  FAKEMODE1C = 2;
  FAKEMODE2A = 3;
  FAKEMODE2B = 4;
  FAKEMODE2C = 5;
  FAKEMODE3A = 6;
  FAKEMODE3B = 7;
  FAKEMODE3C = 8;

Var
  m_mode_type : Integer;
  m_fake_type : Integer;
  m_dispoffset : Integer;

Procedure VGASetMode(xres, yres, modetype, faketype : Integer);
Procedure fakemode_load(src : PByte; wvr : Boolean);

Implementation

Uses
  go32;

Var
  RealRegs : TRealRegs;

Procedure vgamode;

Begin
  RealRegs.ax := $13;
  realintr($10, RealRegs);
End;

Procedure biostextmode;

Begin
  RealRegs.ax := 3;
  realintr($10, RealRegs);
End;

Procedure wait_retrace;

Begin
  While (inportb($3DA) And 8) <> 0 Do;
  While (inportb($3DA) And 8) = 0 Do;
End;

Procedure clearmem(d : DWord); Assembler;

Asm
  cld
  push es
  mov ax, fs
  mov es, ax
  mov edi, [d]
  mov ecx, 2048/4
  mov eax, 0
  rep stosd
  pop es
End;

Procedure clear_memory;

Var
  dest : DWord;
  strip : Integer;

Begin
  wait_retrace;
  dest := $A0000;
  For strip := 0 To 31 Do
  Begin
    outportw($3C4, $102);
    clearmem(dest);
    outportw($3C4, $202);
    clearmem(dest);
    outportw($3C4, $402);
    clearmem(dest);
    outportw($3C4, $802);
    clearmem(dest);
    Inc(dest, 2048);
  End;
End;

Procedure palette(data : PDWord);

Var
  I : Integer;
  C : DWord;

Begin
  outportb($3C8, 0);
  For I := 0 To 255 Do
  Begin
    C := (data[I] Shr 2) And $3F3F3F;
    outportb($3C9, C Shr 16);
    outportb($3C9, C Shr 8);
    outportb($3C9, C);
  End;
End;

Procedure VGASetMode(xres, yres, modetype, faketype : Integer);

Var
  pal : Array[0..255] Of DWord;
  I : Integer;
  r, g, b : Integer;
  z : Integer;

Begin
  m_mode_type := modetype;
  { set up display offset to centre image on display }
  m_dispoffset := ((100 - (yres Shr 1)) * 320) + (160 - (xres Shr 1));
  If (faketype < FAKEMODE1A) Or (faketype > FAKEMODE2C) Then
    faketype := FAKEMODE2A;
  m_fake_type := faketype;

  vgamode;
  If modetype = FAKEMODE Then
  Begin
    FillChar(pal, SizeOf(pal), 0);
    palette(@pal);
    m_dispoffset := 0;
    wait_retrace;
    If (faketype >= FAKEMODE1A) And (faketype <= FAKEMODE1C) Then
    Begin
      {FAKEMODE1x - 320x600}
      outportb($3D4, $11);
      outportb($3D5, inportb($3D5) And $7F);
      outportb($3C2, $E7);
      outportb($3D4, $00); outportb($3D5, $5F);
      outportb($3D4, $01); outportb($3D5, $4F);
      outportb($3D4, $02); outportb($3D5, $50);
      outportb($3D4, $03); outportb($3D5, $82);
      outportb($3D4, $04); outportb($3D5, $54);
      outportb($3D4, $05); outportb($3D5, $80);
      outportb($3D4, $06); outportb($3D5, $70);
      outportb($3D4, $07); outportb($3D5, $F0);
      outportb($3D4, $08); outportb($3D5, $00);
      outportb($3D4, $09); outportb($3D5, $60);
      outportb($3D4, $10); outportb($3D5, $5B);
      outportb($3D4, $11); outportb($3D5, $8C);
      outportb($3D4, $12); outportb($3D5, $57);
      outportb($3D4, $13); outportb($3D5, $28);
      outportb($3D4, $14); outportb($3D5, $00);
      outportb($3D4, $15); outportb($3D5, $58);
      outportb($3D4, $16); outportb($3D5, $70);
      outportb($3D4, $17); outportb($3D5, $E3);
      outportb($3C4, $01); outportb($3C5, $01);
      outportb($3C4, $04); outportb($3C5, $06);
      outportb($3CE, $05); outportb($3CF, $40);
      outportb($3CE, $06); outportb($3CF, $05);
      outportb($3CE, $06); outportb($3CF, $05);
    End
    Else
    Begin
      outportb($3D4, $11); outportb($3D5, inportb($3D5) And $7F);
      outportb($3C2, $63);
      outportb($3D4, $00); outportb($3D5, $5F);
      outportb($3D4, $01); outportb($3D5, $4F);
      outportb($3D4, $02); outportb($3D5, $50);
      outportb($3D4, $03); outportb($3D5, $82);
      outportb($3D4, $04); outportb($3D5, $54);
      outportb($3D4, $05); outportb($3D5, $80);
      outportb($3D4, $06); outportb($3D5, $BF);
      outportb($3D4, $07); outportb($3D5, $1F);
      outportb($3D4, $08); outportb($3D5, $00);
      outportb($3D4, $09); outportb($3D5, $40);
      outportb($3D4, $10); outportb($3D5, $9C);
      outportb($3D4, $11); outportb($3D5, $8E);
      outportb($3D4, $12); outportb($3D5, $8F);
      outportb($3D4, $13); outportb($3D5, $28);
      outportb($3D4, $14); outportb($3D5, $00);
      outportb($3D4, $15); outportb($3D5, $96);
      outportb($3D4, $16); outportb($3D5, $B9);
      outportb($3D4, $17); outportb($3D5, $E3);
      outportb($3C4, $01); outportb($3C5, $01);
      outportb($3C4, $04); outportb($3C5, $06);
      outportb($3CE, $05); outportb($3CF, $40);
      outportb($3CE, $06); outportb($3CF, $05);
      outportb($3CE, $06); outportb($3CF, $05);
    End;
    clear_memory;
    If (faketype >= FAKEMODE2A) And (faketype <= FAKEMODE2C) Then
    Begin
      {FAKEMODE2 palette}
      {taken from PTC 0.73}
      For I := 0 To $7F Do
      Begin
        {bit 7 = 0 (top section)}
	{red (4 bits)}
        r := Round(((I Shr 3) * 255) / 15);
	{blue (3 bits)}
	b := Round(((I And 7) * 255) / 7);
	pal[I] := (r Shl 16) Or b;
      End;
      For I := $80 To $FF Do
      Begin
        {bit 7 = 1 (bottom section)}
	{green}
	g := Round(((I And $1F) * 255) / 31);
	pal[I] := g Shl 8;
      End;
    End
    Else
    Begin
      For I := 0 To 63 Do
      Begin
        {FAKEMODE(1,3) palette}
	z := Round((I * 255) / 63);
	pal[I] := z Shl 16;
	pal[I + 64] := z Shl 8;
	pal[I + 128] := z;
	pal[I + 192] := (z Shl 16) Or (z Shl 8) Or z;
      End;
    End;
    palette(@pal);
  End
  Else
    If modetype = RGB332 Then
    Begin
      For I := 0 To 255 Do
      Begin
        r := Round(((I Shr 5) * 255) / 7);
        g := Round((((I And $1C) Shr 2) * 255) / 7);
        b := Round(((I And $03) * 255) / 3);
        pal[I] := (r Shl 16) Or (g Shl 8) Or b;
      End;
      palette(@pal);
    End;
End;

Function PlaneBlt1_RGB(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := r;
      MemL[dest + 20*4] := gl;
      MemL[dest + 40*4] := b;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 40 * 4);
  End;
  PlaneBlt1_RGB := dest;
End;

Function PlaneBlt1_RBG(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := r;
      MemL[dest + 20*4] := b;
      MemL[dest + 40*4] := gl;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 40 * 4);
  End;
  PlaneBlt1_RBG := dest;
End;

Function PlaneBlt1_GRB(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := gl;
      MemL[dest + 20*4] := r;
      MemL[dest + 40*4] := b;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 40 * 4);
  End;
  PlaneBlt1_GRB := dest;
End;

Function PlaneBlt2_RBG(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      b := gl;
      gl := gl And $C0C0C0C0;
      gl := gl Shr 6;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 2;

      b := b And $1C1C1C1C;
      b := b Shr 2;

      r := r And $F0F0F0F0;
      r := r Shr 1;

      Inc(r, b);
      Inc(gl, gh);
      gl := gl Or $80808080;

      MemL[dest] := r;
      MemL[dest + 20*4] := gl;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 20 * 4);
  End;
  PlaneBlt2_RBG := dest;
End;

Function PlaneBlt2_GBR(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      b := gl;
      gl := gl And $C0C0C0C0;
      gl := gl Shr 6;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 2;

      b := b And $1C1C1C1C;
      b := b Shr 2;

      r := r And $F0F0F0F0;
      r := r Shr 1;

      Inc(r, b);
      Inc(gl, gh);
      gl := gl Or $80808080;

      MemL[dest] := gl;
      MemL[dest + 20*4] := r;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 20 * 4);
  End;
  PlaneBlt2_GBR := dest;
End;

Function PlaneBlt3_RGBRGB(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := ((src[ 0+(320*2)])       ) Or ((src[ 8+(320*2)]) Shl 8) Or
           ((src[16+(320*2)]) Shl 16) Or ((src[24+(320*2)]) Shl 24);
      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := r;
      MemL[dest + 20*4] := gl;
      MemL[dest + 40*4] := b;

      r := ((src[ 1+(320*2)])       ) Or ((src[ 9+(320*2)]) Shl 8) Or
           ((src[17+(320*2)]) Shl 16) Or ((src[25+(320*2)]) Shl 24);
      r := r And $F8F8F8F8;
      r := r Shr 2;

      gl := ((src[ 0+(640*2)])       ) Or ((src[ 8+(640*2)]) Shl 8) Or
            ((src[16+(640*2)]) Shl 16) Or ((src[24+(640*2)]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1+(640*2)])       ) Or ((src[ 9+(640*2)]) Shl 8) Or
            ((src[17+(640*2)]) Shl 16) Or ((src[25+(640*2)]) Shl 24);
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest + 60*4] := r;
      MemL[dest + 80*4] := gl;
      MemL[dest + 100*4] := b;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 100 * 4);
    Inc(src, 320 * 2 * 2);
  End;
  PlaneBlt3_RGBRGB := dest;
End;

Function PlaneBlt3_GRBGRB(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := ((src[ 0+(320*2)])       ) Or ((src[ 8+(320*2)]) Shl 8) Or
           ((src[16+(320*2)]) Shl 16) Or ((src[24+(320*2)]) Shl 24);
      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := gl;
      MemL[dest + 20*4] := r;
      MemL[dest + 40*4] := b;

      r := ((src[ 1+(320*2)])       ) Or ((src[ 9+(320*2)]) Shl 8) Or
           ((src[17+(320*2)]) Shl 16) Or ((src[25+(320*2)]) Shl 24);
      r := r And $F8F8F8F8;
      r := r Shr 2;

      gl := ((src[ 0+(640*2)])       ) Or ((src[ 8+(640*2)]) Shl 8) Or
            ((src[16+(640*2)]) Shl 16) Or ((src[24+(640*2)]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1+(640*2)])       ) Or ((src[ 9+(640*2)]) Shl 8) Or
            ((src[17+(640*2)]) Shl 16) Or ((src[25+(640*2)]) Shl 24);
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest + 60*4] := gl;
      MemL[dest + 80*4] := r;
      MemL[dest + 100*4] := b;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 100 * 4);
    Inc(src, 320 * 2 * 2);
  End;
  PlaneBlt3_GRBGRB := dest;
End;

Function PlaneBlt3_RBGRBG(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := ((src[ 0+(320*2)])       ) Or ((src[ 8+(320*2)]) Shl 8) Or
           ((src[16+(320*2)]) Shl 16) Or ((src[24+(320*2)]) Shl 24);
      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := r;
      MemL[dest + 20*4] := b;
      MemL[dest + 40*4] := gl;

      r := ((src[ 1+(320*2)])       ) Or ((src[ 9+(320*2)]) Shl 8) Or
           ((src[17+(320*2)]) Shl 16) Or ((src[25+(320*2)]) Shl 24);
      r := r And $F8F8F8F8;
      r := r Shr 2;

      gl := ((src[ 0+(640*2)])       ) Or ((src[ 8+(640*2)]) Shl 8) Or
            ((src[16+(640*2)]) Shl 16) Or ((src[24+(640*2)]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1+(640*2)])       ) Or ((src[ 9+(640*2)]) Shl 8) Or
            ((src[17+(640*2)]) Shl 16) Or ((src[25+(640*2)]) Shl 24);
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest + 60*4] := r;
      MemL[dest + 80*4] := b;
      MemL[dest + 100*4] := gl;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 100 * 4);
    Inc(src, 320 * 2 * 2);
  End;
  PlaneBlt3_RBGRBG := dest;
End;

Function PlaneBlt3_GRBRBG(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := ((src[ 0+(320*2)])       ) Or ((src[ 8+(320*2)]) Shl 8) Or
           ((src[16+(320*2)]) Shl 16) Or ((src[24+(320*2)]) Shl 24);
      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := gl;
      MemL[dest + 20*4] := r;
      MemL[dest + 40*4] := b;

      r := ((src[ 1+(320*2)])       ) Or ((src[ 9+(320*2)]) Shl 8) Or
           ((src[17+(320*2)]) Shl 16) Or ((src[25+(320*2)]) Shl 24);
      r := r And $F8F8F8F8;
      r := r Shr 2;

      gl := ((src[ 0+(640*2)])       ) Or ((src[ 8+(640*2)]) Shl 8) Or
            ((src[16+(640*2)]) Shl 16) Or ((src[24+(640*2)]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1+(640*2)])       ) Or ((src[ 9+(640*2)]) Shl 8) Or
            ((src[17+(640*2)]) Shl 16) Or ((src[25+(640*2)]) Shl 24);
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest + 60*4] := r;
      MemL[dest + 80*4] := b;
      MemL[dest + 100*4] := gl;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 100 * 4);
    Inc(src, 320 * 2 * 2);
  End;
  PlaneBlt3_GRBRBG := dest;
End;

Function PlaneBlt3_RBGGRB(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  row, col : Integer;
  r, gl, gh, b : DWord;

Begin
  For row := 1 To rows Do
  Begin
    For col := 0 To 19 Do
    Begin
      gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
            ((src[16]) Shl 16) Or ((src[24]) Shl 24);
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
            ((src[17]) Shl 16) Or ((src[25]) Shl 24);
      r := gh;
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := ((src[ 0+(320*2)])       ) Or ((src[ 8+(320*2)]) Shl 8) Or
           ((src[16+(320*2)]) Shl 16) Or ((src[24+(320*2)]) Shl 24);
      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      r := r And $F8F8F8F8;
      r := r Shr 2;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest] := r;
      MemL[dest + 20*4] := b;
      MemL[dest + 40*4] := gl;

      r := ((src[ 1+(320*2)])       ) Or ((src[ 9+(320*2)]) Shl 8) Or
           ((src[17+(320*2)]) Shl 16) Or ((src[25+(320*2)]) Shl 24);
      r := r And $F8F8F8F8;
      r := r Shr 2;

      gl := ((src[ 0+(640*2)])       ) Or ((src[ 8+(640*2)]) Shl 8) Or
            ((src[16+(640*2)]) Shl 16) Or ((src[24+(640*2)]) Shl 24);
      b := gl;
      gl := gl And $E0E0E0E0;
      gl := gl Shr 5;

      gh := ((src[ 1+(640*2)])       ) Or ((src[ 9+(640*2)]) Shl 8) Or
            ((src[17+(640*2)]) Shl 16) Or ((src[25+(640*2)]) Shl 24);
      gh := gh And $07070707;
      gh := gh Shl 3;

      b := b And $1F1F1F1F;
      b := b Shl 1;
      b := b Or $80808080;

      Inc(gl, gh);
      gl := gl Or $40404040;

      MemL[dest + 60*4] := gl;
      MemL[dest + 80*4] := r;
      MemL[dest + 100*4] := b;

      Inc(dest, 4);
      Inc(src, 4 * 4 * 2);
    End;
    Inc(dest, 100 * 4);
    Inc(src, 320 * 2 * 2);
  End;
  PlaneBlt3_RBGGRB := dest;
End;

Function PlaneBlt3_RGBR(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  {row,} col : Integer;
  r, gl, gh, b : DWord;

Begin
  For col := 0 To 19 Do
  Begin
    gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
          ((src[16]) Shl 16) Or ((src[24]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
          ((src[17]) Shl 16) Or ((src[25]) Shl 24);
    r := gh;
    gh := gh And $07070707;
    gh := gh Shl 3;

    b := ((src[ 0+320*2])       ) Or ((src[ 8+320*2]) Shl  8) Or
         ((src[16+320*2]) Shl 16) Or ((src[24+320*2]) Shl 24);
    b := b And $1F1F1F1F;
    b := b Shl 1;
    b := b Or $80808080;

    r := r And $F8F8F8F8;
    r := r Shr 2;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest] := r;
    MemL[dest + 20*4] := gl;
    MemL[dest + 40*4] := b;

    r := ((src[ 1+320*2])       ) Or ((src[ 9+320*2]) Shl  8) Or
         ((src[17+320*2]) Shl 16) Or ((src[25+320*2]) Shl 24);
    r := r Or $F8F8F8F8;
    r := r Shr 2;

    MemL[dest + 60*4] := r;

    Inc(dest, 4);
    Inc(src, 4 * 4 * 2);
  End;
  PlaneBlt3_RGBR := dest;
End;

Function PlaneBlt3_GRBG(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  {row,} col : Integer;
  r, gl, gh, b : DWord;

Begin
  For col := 0 To 19 Do
  Begin
    gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
          ((src[16]) Shl 16) Or ((src[24]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
          ((src[17]) Shl 16) Or ((src[25]) Shl 24);
    r := gh;
    gh := gh And $07070707;
    gh := gh Shl 3;

    b := ((src[ 0+320*2])       ) Or ((src[ 8+320*2]) Shl  8) Or
         ((src[16+320*2]) Shl 16) Or ((src[24+320*2]) Shl 24);
    b := b And $1F1F1F1F;
    b := b Shl 1;
    b := b Or $80808080;

    r := r And $F8F8F8F8;
    r := r Shr 2;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest] := gl;
    MemL[dest + 20*4] := r;
    MemL[dest + 40*4] := b;

    gl := ((src[ 0+640*2])       ) Or ((src[ 8+640*2]) Shl  8) Or
          ((src[16+640*2]) Shl 16) Or ((src[24+640*2]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1+640*2])       ) Or ((src[ 9+640*2]) Shl  8) Or
          ((src[17+640*2]) Shl 16) Or ((src[25+640*2]) Shl 24);
    gh := gh And $07070707;
    gh := gh Shl 3;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest + 60*4] := gl;

    Inc(dest, 4);
    Inc(src, 4 * 4 * 2);
  End;
  PlaneBlt3_GRBG := dest;
End;

Function PlaneBlt3_RBGR(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  {row,} col : Integer;
  r, gl, gh, b : DWord;

Begin
  For col := 0 To 19 Do
  Begin
    gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
          ((src[16]) Shl 16) Or ((src[24]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
          ((src[17]) Shl 16) Or ((src[25]) Shl 24);
    r := gh;
    gh := gh And $07070707;
    gh := gh Shl 3;

    b := ((src[ 0+320*2])       ) Or ((src[ 8+320*2]) Shl  8) Or
         ((src[16+320*2]) Shl 16) Or ((src[24+320*2]) Shl 24);
    b := b And $1F1F1F1F;
    b := b Shl 1;
    b := b Or $80808080;

    r := r And $F8F8F8F8;
    r := r Shr 2;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest] := r;
    MemL[dest + 20*4] := b;
    MemL[dest + 40*4] := gl;

    r := ((src[ 1+320*2])       ) Or ((src[ 9+320*2]) Shl  8) Or
         ((src[17+320*2]) Shl 16) Or ((src[25+320*2]) Shl 24);
    r := r Or $F8F8F8F8;
    r := r Shr 2;

    MemL[dest + 60*4] := r;

    Inc(dest, 4);
    Inc(src, 4 * 4 * 2);
  End;
  PlaneBlt3_RBGR := dest;
End;

Function PlaneBlt3_GRBR(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  {row,} col : Integer;
  r, gl, gh, b : DWord;

Begin
  For col := 0 To 19 Do
  Begin
    gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
          ((src[16]) Shl 16) Or ((src[24]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
          ((src[17]) Shl 16) Or ((src[25]) Shl 24);
    r := gh;
    gh := gh And $07070707;
    gh := gh Shl 3;

    b := ((src[ 0+320*2])       ) Or ((src[ 8+320*2]) Shl  8) Or
         ((src[16+320*2]) Shl 16) Or ((src[24+320*2]) Shl 24);
    b := b And $1F1F1F1F;
    b := b Shl 1;
    b := b Or $80808080;

    r := r And $F8F8F8F8;
    r := r Shr 2;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest] := gl;
    MemL[dest + 20*4] := r;
    MemL[dest + 40*4] := b;

    r := ((src[ 1+320*2])       ) Or ((src[ 9+320*2]) Shl  8) Or
         ((src[17+320*2]) Shl 16) Or ((src[25+320*2]) Shl 24);
    r := r Or $F8F8F8F8;
    r := r Shr 2;

    MemL[dest + 60*4] := r;

    Inc(dest, 4);
    Inc(src, 4 * 4 * 2);
  End;
  PlaneBlt3_GRBR := dest;
End;

Function PlaneBlt3_RBGG(src : PByte; dest : DWord; rows : Integer) : DWord;

Var
  {row,} col : Integer;
  r, gl, gh, b : DWord;

Begin
  For col := 0 To 19 Do
  Begin
    gl := ((src[ 0])       ) Or ((src[ 8]) Shl  8) Or
          ((src[16]) Shl 16) Or ((src[24]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1])       ) Or ((src[ 9]) Shl 8) Or
          ((src[17]) Shl 16) Or ((src[25]) Shl 24);
    r := gh;
    gh := gh And $07070707;
    gh := gh Shl 3;

    b := ((src[ 0+320*2])       ) Or ((src[ 8+320*2]) Shl  8) Or
         ((src[16+320*2]) Shl 16) Or ((src[24+320*2]) Shl 24);
    b := b And $1F1F1F1F;
    b := b Shl 1;
    b := b Or $80808080;

    r := r And $F8F8F8F8;
    r := r Shr 2;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest] := r;
    MemL[dest + 20*4] := b;
    MemL[dest + 40*4] := gl;

    gl := ((src[ 0+640*2])       ) Or ((src[ 8+640*2]) Shl  8) Or
          ((src[16+640*2]) Shl 16) Or ((src[24+640*2]) Shl 24);
    gl := gl And $E0E0E0E0;
    gl := gl Shr 5;

    gh := ((src[ 1+640*2])       ) Or ((src[ 9+640*2]) Shl  8) Or
          ((src[17+640*2]) Shl 16) Or ((src[25+640*2]) Shl 24);
    gh := gh And $07070707;
    gh := gh Shl 3;

    Inc(gl, gh);
    gl := gl Or $40404040;

    MemL[dest + 60*4] := gl;

    Inc(dest, 4);
    Inc(src, 4 * 4 * 2);
  End;
  PlaneBlt3_RBGG := dest;
End;

Procedure fakemode_load(src : PByte; wvr : Boolean);

Var
  dest, d : DWord;
  w, s : Integer;

Begin
  dest := $A0000;
  Case m_fake_type Of
    FAKEMODE1A :
      For w := 0 To 24 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt1_RGB(src + 0, dest, 8);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt1_RGB(src + 2, dest, 8);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt1_RGB(src + 4, dest, 8);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt1_RGB(src + 6, dest, 8);
	Inc(src, 320 * 4 * 4);
      End;
    FAKEMODE1B :
      For w := 0 To 24 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	d := PlaneBlt1_RBG(src + (4*4*2*20*0), dest, 1);
	d := PlaneBlt1_GRB(src + (4*4*2*20*1), d, 1);
	d := PlaneBlt1_RBG(src + (4*4*2*20*2), d, 1);
	d := PlaneBlt1_GRB(src + (4*4*2*20*3), d, 1);
	d := PlaneBlt1_RBG(src + (4*4*2*20*4), d, 1);
	d := PlaneBlt1_GRB(src + (4*4*2*20*5), d, 1);
	d := PlaneBlt1_RBG(src + (4*4*2*20*6), d, 1);
	d := PlaneBlt1_GRB(src + (4*4*2*20*7), d, 1);
	
        {plane 1}
	outportw($3C4, $202);
	d := PlaneBlt1_GRB(src + 2 + (4*4*2*20*0), dest, 1);
	d := PlaneBlt1_RBG(src + 2 + (4*4*2*20*1), d, 1);
	d := PlaneBlt1_GRB(src + 2 + (4*4*2*20*2), d, 1);
	d := PlaneBlt1_RBG(src + 2 + (4*4*2*20*3), d, 1);
	d := PlaneBlt1_GRB(src + 2 + (4*4*2*20*4), d, 1);
	d := PlaneBlt1_RBG(src + 2 + (4*4*2*20*5), d, 1);
	d := PlaneBlt1_GRB(src + 2 + (4*4*2*20*6), d, 1);
	d := PlaneBlt1_RBG(src + 2 + (4*4*2*20*7), d, 1);
	
        {plane 2}
	outportw($3C4, $402);
	d := PlaneBlt1_RBG(src + 4 + (4*4*2*20*0), dest, 1);
	d := PlaneBlt1_GRB(src + 4 + (4*4*2*20*1), d, 1);
	d := PlaneBlt1_RBG(src + 4 + (4*4*2*20*2), d, 1);
	d := PlaneBlt1_GRB(src + 4 + (4*4*2*20*3), d, 1);
	d := PlaneBlt1_RBG(src + 4 + (4*4*2*20*4), d, 1);
	d := PlaneBlt1_GRB(src + 4 + (4*4*2*20*5), d, 1);
	d := PlaneBlt1_RBG(src + 4 + (4*4*2*20*6), d, 1);
	d := PlaneBlt1_GRB(src + 4 + (4*4*2*20*7), d, 1);
	
        {plane 3}
	outportw($3C4, $802);
	d := PlaneBlt1_GRB(src + 6 + (4*4*2*20*0), dest, 1);
	d := PlaneBlt1_RBG(src + 6 + (4*4*2*20*1), d, 1);
	d := PlaneBlt1_GRB(src + 6 + (4*4*2*20*2), d, 1);
	d := PlaneBlt1_RBG(src + 6 + (4*4*2*20*3), d, 1);
	d := PlaneBlt1_GRB(src + 6 + (4*4*2*20*4), d, 1);
	d := PlaneBlt1_RBG(src + 6 + (4*4*2*20*5), d, 1);
	d := PlaneBlt1_GRB(src + 6 + (4*4*2*20*6), d, 1);
	dest := PlaneBlt1_RBG(src + 6 + (4*4*2*20*7), d, 1);
	Inc(src, 320*4*4);
      End;
    FAKEMODE1C :
      For w := 0 To 24 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt1_RBG(src + 0, dest, 8);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt1_GRB(src + 2, dest, 8);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt1_RBG(src + 4, dest, 8);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt1_GRB(src + 6, dest, 8);
	Inc(src, 320 * 4 * 4);
      End;
    FAKEMODE2A :
      For w := 0 To 24 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt2_RBG(src + 0, dest, 8);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt2_RBG(src + 2, dest, 8);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt2_RBG(src + 4, dest, 8);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt2_RBG(src + 6, dest, 8);
	Inc(src, 320 * 4 * 4);
      End;
    FAKEMODE2B :
      For w := 0 To 24 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	d := PlaneBlt2_RBG(src + (4*4*2*20*0), dest, 1);
	d := PlaneBlt2_GBR(src + (4*4*2*20*1), d, 1);
	d := PlaneBlt2_RBG(src + (4*4*2*20*2), d, 1);
	d := PlaneBlt2_GBR(src + (4*4*2*20*3), d, 1);
	d := PlaneBlt2_RBG(src + (4*4*2*20*4), d, 1);
	d := PlaneBlt2_GBR(src + (4*4*2*20*5), d, 1);
	d := PlaneBlt2_RBG(src + (4*4*2*20*6), d, 1);
	d := PlaneBlt2_GBR(src + (4*4*2*20*7), d, 1);
	
        {plane 1}
	outportw($3C4, $202);
	d := PlaneBlt2_GBR(src + 2 + (4*4*2*20*0), dest, 1);
	d := PlaneBlt2_RBG(src + 2 + (4*4*2*20*1), d, 1);
	d := PlaneBlt2_GBR(src + 2 + (4*4*2*20*2), d, 1);
	d := PlaneBlt2_RBG(src + 2 + (4*4*2*20*3), d, 1);
	d := PlaneBlt2_GBR(src + 2 + (4*4*2*20*4), d, 1);
	d := PlaneBlt2_RBG(src + 2 + (4*4*2*20*5), d, 1);
	d := PlaneBlt2_GBR(src + 2 + (4*4*2*20*6), d, 1);
	d := PlaneBlt2_RBG(src + 2 + (4*4*2*20*7), d, 1);
	
        {plane 2}
	outportw($3C4, $402);
	d := PlaneBlt2_RBG(src + 4 + (4*4*2*20*0), dest, 1);
	d := PlaneBlt2_GBR(src + 4 + (4*4*2*20*1), d, 1);
	d := PlaneBlt2_RBG(src + 4 + (4*4*2*20*2), d, 1);
	d := PlaneBlt2_GBR(src + 4 + (4*4*2*20*3), d, 1);
	d := PlaneBlt2_RBG(src + 4 + (4*4*2*20*4), d, 1);
	d := PlaneBlt2_GBR(src + 4 + (4*4*2*20*5), d, 1);
	d := PlaneBlt2_RBG(src + 4 + (4*4*2*20*6), d, 1);
	d := PlaneBlt2_GBR(src + 4 + (4*4*2*20*7), d, 1);
	
        {plane 3}
	outportw($3C4, $802);
	d := PlaneBlt2_GBR(src + 6 + (4*4*2*20*0), dest, 1);
	d := PlaneBlt2_RBG(src + 6 + (4*4*2*20*1), d, 1);
	d := PlaneBlt2_GBR(src + 6 + (4*4*2*20*2), d, 1);
	d := PlaneBlt2_RBG(src + 6 + (4*4*2*20*3), d, 1);
	d := PlaneBlt2_GBR(src + 6 + (4*4*2*20*4), d, 1);
	d := PlaneBlt2_RBG(src + 6 + (4*4*2*20*5), d, 1);
	d := PlaneBlt2_GBR(src + 6 + (4*4*2*20*6), d, 1);
	dest := PlaneBlt2_RBG(src + 6 + (4*4*2*20*7), d, 1);
	Inc(src, 320*4*4);
      End;
    FAKEMODE2C :
      For w := 0 To 24 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt2_RBG(src + 0, dest, 8);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt2_GBR(src + 2, dest, 8);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt2_RBG(src + 4, dest, 8);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt2_GBR(src + 6, dest, 8);
	Inc(src, 320 * 4 * 4);
      End;
    FAKEMODE3A : Begin
      For w := 0 To 15 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt3_RGBRGB(src + 0, dest, 4);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt3_RGBRGB(src + 2, dest, 4);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt3_RGBRGB(src + 4, dest, 4);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt3_RGBRGB(src + 6, dest, 4);
	Inc(src, 320 * 4 * 2 * 3);
      End;
      s := (4*4*2*20) + (320*2*2*2);
      outportw($3C4, $102);
      d := PlaneBlt3_RGBRGB(src, dest, 2);
      PlaneBlt3_RGBR(src + s, d, 1);

      outportw($3C4, $202);
      d := PlaneBlt3_RGBRGB(src + 2, dest, 2);
      PlaneBlt3_RGBR(src + s + 2, d, 1);

      outportw($3C4, $402);
      d := PlaneBlt3_RGBRGB(src + 4, dest, 2);
      PlaneBlt3_RGBR(src + s + 4, d, 1);

      outportw($3C4, $802);
      d := PlaneBlt3_RGBRGB(src + 6, dest, 2);
      PlaneBlt3_RGBR(src + s + 6, d, 1);
    End;
    FAKEMODE3B : Begin
      For w := 0 To 15 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt3_GRBRBG(src + 0, dest, 4);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt3_RBGGRB(src + 2, dest, 4);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt3_GRBRBG(src + 4, dest, 4);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt3_RBGGRB(src + 6, dest, 4);
	Inc(src, 320 * 4 * 2 * 3);
      End;
      s := (4*4*2*20) + (320*2*2*2);
      outportw($3C4, $102);
      d := PlaneBlt3_GRBRBG(src, dest, 2);
      PlaneBlt3_GRBR(src + s, d, 1);

      outportw($3C4, $202);
      d := PlaneBlt3_RBGGRB(src + 2, dest, 2);
      PlaneBlt3_RBGG(src + s + 2, d, 1);

      outportw($3C4, $402);
      d := PlaneBlt3_GRBRBG(src + 4, dest, 2);
      PlaneBlt3_GRBR(src + s + 4, d, 1);

      outportw($3C4, $802);
      d := PlaneBlt3_RBGGRB(src + 6, dest, 2);
      PlaneBlt3_RBGG(src + s + 6, d, 1);
    End;
    FAKEMODE3C : Begin
      For w := 0 To 15 Do
      Begin
        {plane 0}
	outportw($3C4, $102);
	PlaneBlt3_GRBGRB(src + 0, dest, 4);
	
        {plane 1}
	outportw($3C4, $202);
	PlaneBlt3_RBGRBG(src + 2, dest, 4);
	
        {plane 2}
	outportw($3C4, $402);
	PlaneBlt3_GRBGRB(src + 4, dest, 4);
	
        {plane 3}
	outportw($3C4, $802);
	dest := PlaneBlt3_RBGRBG(src + 6, dest, 4);
	Inc(src, 320 * 4 * 2 * 3);
      End;
      s := (4*4*2*20) + (320*2*2*2);
      outportw($3C4, $102);
      d := PlaneBlt3_GRBGRB(src, dest, 2);
      PlaneBlt3_GRBG(src + s, d, 1);

      outportw($3C4, $202);
      d := PlaneBlt3_RBGRBG(src + 2, dest, 2);
      PlaneBlt3_RBGR(src + s + 2, d, 1);

      outportw($3C4, $402);
      d := PlaneBlt3_GRBGRB(src + 4, dest, 2);
      PlaneBlt3_GRBG(src + s + 4, d, 1);

      outportw($3C4, $802);
      d := PlaneBlt3_RBGRBG(src + 6, dest, 2);
      PlaneBlt3_RBGR(src + s + 6, d, 1);
    End;
  End;
  If wvr Then
    wait_retrace;
End;

End.
