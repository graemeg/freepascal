{$MODE objfpc}
{$ASMMODE intel}

Unit CGA;

Interface

Procedure CGAText;
Procedure CGA320;
Procedure CGA640;
Procedure CGADump(q : PByte);
Procedure CGASetPalette(palette, border : Integer);
Procedure CGAPrecalc;

Implementation

Uses
  go32, crt;

Const
  palette : Array[0..15, 0..2] Of Byte = (
    ( 0, 0, 0), ( 0, 0,42), ( 0,42, 0), ( 0,42,42), (42, 0, 0), (42, 0,42), (42,21, 0), (42,42,42),
    (21,21,21), (21,21,63), (21,63,21), (21,63,63), (63,21,21), (63,21,63), (63,63,21), (63,63,63));
  cgaback : Array[0..3, 0..12] Of Integer = (
    (  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 11, 13, 15),
    (  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 12, 14),
    (  0,  1,  3,  5,  7,  8,  9, 10, 11, 12, 13, 14, 15),
    (  0,  1,  2,  4,  6,  8,  9, 10, 11, 12, 13, 14, 15));

Type
  Float = Extended;
  TCGAVideoBuffer = Array[0..16383] Of Byte;
  PCGAPrecalc = ^TCGAPrecalc;
  TCGAPrecalc = Array[0..15{r}, 0..15{g}, 0..15{b}, 0..3{y}, 0..3{x}] Of Byte;
  PCGAPrecalcError = ^TCGAPrecalcError;
  TCGAPrecalcError = Array[0..15{r}, 0..15{g}, 0..15{b}] Of Integer;

Var
  cgapal : Array[0..3] Of Integer;
  videobuf : TCGAVideoBuffer;
  precalcbuf : Array[0..12, 0..3] Of PCGAPrecalc; {3.25mb}
  precalcerror : Array[0..12, 0..3] Of PCGAPrecalcError; {0.8125mb}
  error : Integer;
  lastpalette, lastback : Integer;

Procedure CGA320;

Var
  regs : TRealRegs;

Begin
  regs.ax := $0004;
  RealIntr($10, regs);
  lastpalette := -1;
  lastback := -1;
End;

Procedure CGA640;

Var
  regs : TRealRegs;

Begin
  regs.ax := $0004;
  RealIntr($10, regs);
End;

Procedure CGAText;

Var
  regs : TRealRegs;

Begin
  regs.ax := $0003;
  RealIntr($10, regs);
End;

Procedure CGASetPalette(palette, border : Integer);

Var
  regs : TRealRegs;

Begin
  If (palette = lastpalette) And (border = lastback) Then
    Exit;
  lastpalette := palette;
  lastback := border;
  regs.ah := $0B;
  regs.bh := 1;
  regs.bl := palette And 1;
  RealIntr($10, regs);
  If (palette And 2) = 0 Then
    Inc(border, 16);
  regs.ah := $0B;
  regs.bh := 0;
  regs.bl := border;
  RealIntr($10, regs);
End;

Procedure CGABlitToScreen(p : Pointer); Assembler;

Asm
  mov edi, $B8000
  push es
  mov ax, fs
  mov es, ax
  mov esi, [p]
  mov ecx, 16192/4
  rep movsd
  pop es
End;

Function CGACalc2(r, g, b : Integer; dx, dy : Integer; back, pal : Integer) : Integer;{ Inline;}

Begin
  CGACalc2 := precalcbuf[back, pal]^[r Shr 4, g Shr 4, b Shr 4, dy, dx];
End;

Procedure CGACalc(r, g, b : Integer; {dx, dy : Integer;}
                  Var dither, best1, best2 : Integer);

Var
  I, J : Integer;
  mindist : Float;
  dist : Float;
  r1, g1, b1 : Integer;
  tmp : Integer;
{  dither : Integer;} {0-none; 1-50%; 2-25%; 3-12.5%; 4-37.5%}

Begin
  r := Round(r*63 / 15);
  g := Round(g*63 / 15);
  b := Round(b*63 / 15);
  mindist := $7FFFFFFF;
  For I := 0 To 3 Do
  Begin
    dist := Sqr(r - palette[cgapal[I], 0]) +
            Sqr(g - palette[cgapal[I], 1]) +
            Sqr(b - palette[cgapal[I], 2]);
    If dist < mindist Then
    Begin
      mindist := dist;
      best1 := I;
      dither := 0;
    End;
  End;

  For J := 0 To 3 Do
  Begin
    r1 := palette[cgapal[J], 0];
    g1 := palette[cgapal[J], 1];
    b1 := palette[cgapal[J], 2];
    For I := 0 To 3 Do
    Begin
      If I = J Then
        Continue;
      dist := Sqr(r - (palette[cgapal[I], 0] + r1)*0.5) +
              Sqr(g - (palette[cgapal[I], 1] + g1)*0.5) +
              Sqr(b - (palette[cgapal[I], 2] + b1)*0.5);
      If dist < mindist Then
      Begin
        mindist := dist;
        best1 := J;
        best2 := I;
        dither := 1;
      End;
      dist := Sqr(r - (0.25*palette[cgapal[I], 0] + 0.75*r1)) +
              Sqr(g - (0.25*palette[cgapal[I], 1] + 0.75*g1)) +
              Sqr(b - (0.25*palette[cgapal[I], 2] + 0.75*b1));
      If dist < mindist Then
      Begin
        mindist := dist;
        best1 := J;
        best2 := I;
        dither := 2;
      End;
      dist := Sqr(r - (0.125*palette[cgapal[I], 0] + 0.875*r1)) +
              Sqr(g - (0.125*palette[cgapal[I], 1] + 0.875*g1)) +
              Sqr(b - (0.125*palette[cgapal[I], 2] + 0.875*b1));
      If dist < mindist Then
      Begin
        mindist := dist;
        best1 := J;
        best2 := I;
        dither := 3;
      End;
      dist := Sqr(r - (0.375*palette[cgapal[I], 0] + 0.625*r1)) +
              Sqr(g - (0.375*palette[cgapal[I], 1] + 0.625*g1)) +
              Sqr(b - (0.375*palette[cgapal[I], 2] + 0.625*b1));
      If dist < mindist Then
      Begin
        mindist := dist;
        best1 := J;
        best2 := I;
        dither := 4;
      End;
    End;
  End;

  error:=error+round(Sqrt(mindist) * 290);
  Case dither Of
    0 : best2 := best1;
    1 : Begin
      If best1 > best2 Then
      Begin
        tmp := best1;
        best1 := best2;
        best2 := tmp;
      End;
    End;
  End;
End;

Function CGACalcError(s : PByte; back, pal : Integer) : Integer;

Var
  X, Y : Integer;
  r, g, b : Integer;

Begin
  CGACalcError := 0;
  For Y := 0 To 199 {Div 4} Do
  Begin
    For X := 0 To 319 {Div 4} Do
    Begin
      b := s[0];
      g := s[1];
      r := s[2];
      inc(CGACalcError,precalcerror[back, pal]^[b Shr 4, g Shr 4, r Shr 4]);
      Inc(s, 4{ + 4 + 4 + 4});
    End;
//    Inc(s, 320*4*3);
  End;
End;

Procedure CGADump2(s, d : PByte; back, pal : Integer);

Var
  I : Integer;
  src, dest : PByte;
  X, Y : Integer;
  r1, g1, b1 : Integer;
  r2, g2, b2 : Integer;
  r3, g3, b3 : Integer;
  r4, g4, b4 : Integer;

Begin
  error := 0;
  src := s;
  dest := d;
  For Y := 0 To 99 Do
  Begin
    For X := 0 To 79 Do
    Begin
      b1 := src[0];
      g1 := src[1];
      r1 := src[2];
      b2 := src[4];
      g2 := src[5];
      r2 := src[6];
      b3 := src[8];
      g3 := src[9];
      r3 := src[10];
      b4 := src[12];
      g4 := src[13];
      r4 := src[14];
      dest^ := (CGACalc2(r1, g1, b1, 0, (Y And 1) Shl 1, back, pal) Shl 6) Or
               (CGACalc2(r2, g2, b2, 1, (Y And 1) Shl 1, back, pal) Shl 4) Or
               (CGACalc2(r3, g3, b3, 2, (Y And 1) Shl 1, back, pal) Shl 2) Or
               (CGACalc2(r4, g4, b4, 3, (Y And 1) Shl 1, back, pal));

      Inc(src, 4*4);
      Inc(dest);
    End;
    Inc(src, 320*4);
  End;
  src := s + 320*4;
  dest := d + 8192;
  For Y := 0 To 99 Do
  Begin
    For X := 0 To 79 Do
    Begin
      b1 := src[0];
      g1 := src[1];
      r1 := src[2];
      b2 := src[4];
      g2 := src[5];
      r2 := src[6];
      b3 := src[8];
      g3 := src[9];
      r3 := src[10];
      b4 := src[12];
      g4 := src[13];
      r4 := src[14];
      dest^ := (CGACalc2(r1, g1, b1, 0, ((Y And 1) Shl 1) + 1, back, pal) Shl 6) Or
               (CGACalc2(r2, g2, b2, 1, ((Y And 1) Shl 1) + 1, back, pal) Shl 4) Or
               (CGACalc2(r3, g3, b3, 2, ((Y And 1) Shl 1) + 1, back, pal) Shl 2) Or
               (CGACalc2(r4, g4, b4, 3, ((Y And 1) Shl 1) + 1, back, pal));

      Inc(src, 4*4);
      Inc(dest);
    End;
    Inc(src, 320*4);
  End;
End;

Procedure CGADump(q : PByte);

Var
  pal, back : Integer;
  bestpal, bestback : Integer;
  besterror : Integer;

Begin
  besterror := $7FFFFFFF;
  For pal := 0 To 3 Do
  Begin
    For back := 0 To 12 Do
    Begin
      error := CGACalcError(q, back, pal);
      If error < besterror Then
      Begin
        besterror := error;
        bestpal := pal;
        bestback := back;
      End;
    End;
  End;

  CGADump2(q, videobuf, bestback, bestpal);

  CGASetPalette(bestpal, cgaback[bestpal, bestback]);
  CGABlitToScreen(@videobuf);
End;

Procedure CGAPrecalc;

Var
  pal, back : Integer;
  r, g, b : Integer;
  x, y : Integer;
  dither : Integer;
  best1, best2 : Integer;
  res : Integer;

Begin
  For pal := 0 To 3 Do
  Begin
    Case pal Of
      0 : Begin
        cgapal[1] := 10;
        cgapal[2] := 12;
        cgapal[3] := 14;
      End;
      1 : Begin
        cgapal[1] := 11;
        cgapal[2] := 13;
        cgapal[3] := 15;
      End;
      2 : Begin
        cgapal[1] := 2;
        cgapal[2] := 4;
        cgapal[3] := 6;
      End;
      3 : Begin
        cgapal[1] := 3;
        cgapal[2] := 5;
        cgapal[3] := 7;
      End;
    End;
    For back := 0 To 12 Do
    Begin
      If (precalcbuf[back, pal] = Nil) And (precalcerror[back, pal] = Nil) Then
      Begin
        New(precalcbuf[back, pal]);
        New(precalcerror[back, pal]);
      End
      Else
        Continue;

      cgapal[0] := cgaback[pal, back];
      error := 0;
      Write(pal, back:3, ' ');
      TextAttr := cgapal[0];
      Write('*');
      TextAttr := cgapal[1];
      Write('*');
      TextAttr := cgapal[2];
      Write('*');
      TextAttr := cgapal[3];
      Writeln('*');
      TextAttr := 7;
      For r := 0 To 15 Do
        For g := 0 To 15 Do
          For b := 0 To 15 Do
          Begin
            error := 0;
            CGACalc(r, g, b, dither, best1, best2);
            precalcerror[back, pal]^[r, g, b] := error;
            For y := 0 To 3 Do
              For x := 0 To 3 Do
              Begin
                Case dither Of
                  0 : res := best1;
                  1 : Begin
                    If ((x + y) And 1) <> 0 Then
                      res := best1
                    Else
                      res := best2;
                  End;
                  2 : Begin
                    If ((x And 1) = 0) And ((y And 1) = 0) Then
                      res := best2
                    Else
                      res := best1;
                  End;
                  3 : Begin
                    If (x = y) And ((x And 1) = 0) Then
                      res := best2
                    Else
                      res := best1;
                  End;
                  4 : Begin
                    If (((x And 1) = 0) And ((y And 1) = 0)) Or (x = y) Then
                      res := best2
                    Else
                      res := best1;
                  End;
                End;
                precalcbuf[back, pal]^[r, g, b, y, x] := res;
              End;
          End;
      //Function CGACalc(r, g, b : Integer; dx, dy : Integer) : Integer;
    End;
  End;
End;

Begin
  FillChar(precalcbuf, SizeOf(precalcbuf), 0);
  FillChar(precalcerror, SizeOf(precalcerror), 0);
End.
