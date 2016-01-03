{*
 * TextFX2 Copyright (c) 1998 Jari Komppa aka Sol/Trauma
 * <mailto:solar@compart.fi>
 *
 * Textmode low-level functions
 *
 * This sourcefile is kinda long-ish, and should be split into several
 * sources, but I have wanted to keep it in one file since everything
 * here is kinda small and.. well, I wanted to keep it as a single .obj
 * file.
 *
 * If you make improvements, send me a copy!
 * If you use this for something, let me know!
 *}

{$MODE objfpc}

Unit textfx2;

Interface

Const
{*
 * Charsets in 'lightness' order. First byte = num of chars
 *
 * Please note that these don't work with the current calcpal
 * strategy :)
 *
 *}
  charset_b8ibm : Array[0..254] Of Byte = { all imbscii characters }
( 254, 32, 96, 39, 250, 95, 126, 46, 94, 34, 249, 248, 44, 58, 45,
196, 59, 253, 167, 61, 166, 252, 47, 28, 217, 192, 169, 205, 246, 7,
170, 27, 190, 43, 212, 62, 60, 124, 26, 226, 193, 40, 243, 242, 240,
63, 41, 37, 139, 191, 55, 91, 207, 218, 200, 176, 9, 105, 241, 92,
141, 33, 125, 238, 102, 161, 231, 123, 211, 202, 247, 108, 99, 168,
188, 73, 93, 67, 29, 175, 174, 106, 114, 189, 140, 24, 76, 116, 194,
208, 49, 115, 50, 70, 228, 13, 84, 80, 156, 51, 120, 122, 179, 173,
184, 53, 89, 155, 244, 213, 90, 25, 135, 223, 57, 42, 83, 118, 128,
101, 245, 127, 171, 74, 19, 159, 4, 1, 180, 110, 137, 230, 195, 209,
18, 97, 111, 117, 86, 229, 31, 138, 69, 144, 22, 148, 130, 16, 132,
181, 129, 36, 157, 198, 136, 12, 214, 71, 239, 133, 160, 162, 149,
163, 151, 52, 54, 98, 107, 251, 104, 224, 197, 183, 154, 235, 164,
112, 131, 85, 147, 121, 236, 150, 232, 134, 153, 11, 210, 225, 79,
172, 145, 227, 152, 100, 23, 21, 88, 17, 48, 119, 75, 68, 113, 30, 72,
15, 233, 56, 103, 65, 142, 234, 5, 82, 109, 216, 201, 254, 66, 38,
158, 143, 237, 203, 187, 77, 221, 146, 14, 78, 35, 81, 64, 20, 177,
87, 6, 165, 3, 204, 186, 222, 199, 206, 185, 182, 215, 220, 2, 178,
10, 8, 219);

  charset_b7asc : Array[0..94] Of Byte = { 7b ascii (chars 32 - 126) }
( 94, 32, 96, 39, 95, 126, 46, 94, 34, 44, 58, 45, 59, 61, 47, 43, 62,
60, 40, 63, 41, 37, 55, 91, 105, 92, 33, 125, 102, 123, 108, 99, 73,
93, 67, 106, 114, 76, 116, 49, 115, 50, 70, 84, 80, 51, 120, 122, 53,
89, 90, 57, 42, 83, 118, 101, 74, 110, 97, 111, 117, 86, 69, 36, 71,
52, 54, 98, 107, 104, 112, 85, 121, 79, 100, 88, 48, 119, 75, 68, 113,
72, 56, 103, 65, 82, 109, 66, 38, 77, 78, 35, 81, 64, 87);

  charset_b7sml : Array[0..14] Of Byte = { " crsxzvenaouwm" dark->light. }
( 14, 32, 99, 114, 115, 120, 122, 118, 101, 110, 97, 111, 117, 119,
109 );

  charset_b8gry : Array[0..5] Of Byte = { 8b ibm grayscale characters }
( 5, 32, 176, 177, 178, 219 );

  charset_b7nws : Array[0..6] Of Byte = { 7b grayscale 'newschool' askee chars}
( 6, 32{' '}, 46{'.'}, 111{'o'}, 109{'m'}, 87{'W'}, 77{'M'} );

  use_charset : Pbyte = @charset_b7asc;
  { Character set to use. Can be changed run-time. }

  colmap : PSmallInt = Nil;

Procedure set80x43; { Sets up 80x43, no blink, no cursor. }
Procedure set80x50; { Sets up 80x50, no blink, no cursor. }
Procedure set80x25; { Resets 80x25, blink, cursor. }
Procedure border(color : Byte); { _ONLY_ for debugging! }
Procedure vrc; { Although all should be timer-synced instead.. }

{*
 * calc_ functions are pretty *S*L*O*W* so use them to precalculate
 * color tables and then use those tables instead.
 *}

Function calcpal_colorbase(red, green, blue : Real) : Word;
Function calcpal_lightbase(red, green, blue : Real) : Word;
Function calcpal_lightbase_g(red, green, blue : Real) : Word;
{Function (*calcpal)(float red, float green, float blue) : Word;}
Const calcpal : Function(red, green, blue : Real) : Word = @calcpal_colorbase;
    {* Finds the closest color/char combo for any 0:63,0:63,0:63 value.
     *
     * calcpal_colorbase is the 'old' calcpal, only "a bit" optimized.
     * calcpal is now function pointer so calcpal function can be changed
     * run-time. Use the functions directly if you need speed (and
     * compile with -oe256 or something to force inlining)
     *}

Function calc_gscale(light : Real) : Word;
Function calc_gscale2(light : Real) : Word;
    {* Finds the closes gscale color/char combo for 0..1 range
     * gscale2 uses colors 8,7,15, normal just uses 7.
     *}

Procedure build_colormap(dots : Integer);
    {* Used to calculate colormap for dump_nnx() -functions.
     * if dots=0, will output nothing.
     *         1, will cprintf .:s as process.
     *         2, will cprintf rolling wheel as process.
     *}

Procedure dispose_colormap;

Procedure dump_80x(y0, y1 : Integer; buffer : PInteger);
    {* Dumps 80-pixel wide 0bgr-truecolor buffer from y0 to y1.
     * (For fullscreen dump in 80x43 use dump_80x(0,43,buf);
     *}

Procedure dump_160x(y0, y1 : Integer; buffer : PInteger);
    {* Dumps 160-pixel wide 0bgr-truecolor buffer from y0 to y1
     * with 4-to-1 pixel averaging.
     *}

Procedure dump_320x(y0, y1 : Integer; buffer : PInteger);
    {* Dumps 160-pixel wide 0bgr-truecolor buffer from y0 to y1
     * with 16-to-1 pixel averaging. (this is tad bit slow :)
     *}

Implementation

Uses
  go32;

{ $define __USE_178NOT176}
 { uncomment to use 75% char instead of 25% char }

{$DEFINE __USE_REALIBMPAL}
 { comment out to use 'clean' truecolor palette for calculations }

Const
  COLORMAP_DEPTH = 4;
 {* Normally, build 1<<4, ie. 16x16x16 colormap.
  * If you require bigger map, increase the value.
  * (5 will mean 32x32x32 etc).
  * 8 is max for dump_80x and _320x, 6 is max for _160x.
  * If you make your own routines, well, nothing is too much :)
  *}
{ Don't touch the rest of the defines. }
  COLMAPDIM = 1 Shl COLORMAP_DEPTH;
  TRUCOLBITS = 8 - COLORMAP_DEPTH;

{$IFDEF __USE_REALIBMPAL}
  palette : Array[0..16*3-1] Of Byte = ( {IBM basic palette, 16c}
     0, 0, 0,  0, 0,42,  0,42, 0,  0,42,42, 42, 0, 0, 42, 0,42, 42,21, 0, 42,42,42,
    21,21,21, 21,21,63, 21,63,21, 21,63,63, 63,21,21, 63,21,63, 63,63,21, 63,63,63);
{$ELSE}
  palette : Array[0..16*3-1] Of Byte = ( { 'clean' RGB palette }
     0, 0, 0,  0, 0,32,  0,32, 0,  0,32,32, 32, 0, 0, 32, 0,32, 32,32, 0, 32,32,32,
    32,32,32,  0, 0,63,  0,63, 0,  0,63,63, 63, 0, 0, 63, 0,63, 63,63, 0, 63,63,63);
{$ENDIF}

Procedure set80x43; { Sets up 80x43, no blink, no cursor. }

Var
  regs : TRealRegs;

Begin
  regs.ax := $1201; { Set 350 scanlines }
  regs.bl := $30;
  realintr($10, regs);
  regs.ax := $3; { Set text mode }
  realintr($10, regs);
  regs.ax := $1112; { Set font }
  regs.bx := 0;
  realintr($10, regs);
  regs.bh := 0; { Kill cursor - doesn't seem to work.. }
  regs.ah := 3;
  realintr($10, regs);
  regs.cx := $2000;
  regs.ah := 1;
  realintr($10, regs);
  regs.ax := $1003; { Kill blink }
  regs.bl := 0;
  realintr($10, regs);
  regs.ax := $0200; { Position cursor to 51,80 - better way to kill. }
  regs.bx := $0033;
  regs.dx := $004f;
  realintr($10, regs);
End;

Procedure set80x50; { Sets up 80x50, no blink, no cursor. }

Var
  regs : TRealRegs;

Begin
  regs.ax := $1202; { Set 400 scanlines }
  regs.bl := $30;
  realintr($10, regs);
  regs.ax := $3; { Set text mode }
  realintr($10, regs);
  regs.ax := $1112; { Set font }
  regs.bx := 0;
  realintr($10, regs);
  regs.bh := 0; { Kill cursor - doesn't seem to work.. }
  regs.ah := 3;
  realintr($10, regs);
  regs.cx := $2000;
  regs.ah := 1;
  realintr($10, regs);
  regs.ax := $1003; { Kill blink }
  regs.bl := 0;
  realintr($10, regs);
  regs.ax := $0200; { Position cursor to 51,80 - better way to kill. }
  regs.bx := $0033;
  regs.dx := $004f;
  realintr($10, regs);
End;

Procedure set80x25; { Resets 80x25, blink, cursor. }

Var
  regs : TRealRegs;

Begin
  regs.ax := $1202; { Set 400 scanlines }
  regs.bl := $30;
  realintr($10, regs);
  regs.ax := $3;    { Set text mode }
  realintr($10, regs);
  regs.ax := $1114; { Set font }
  regs.bx := 0;
  realintr($10, regs);
  regs.bh := 0;      { Ressurrect cursor }
  regs.ah := 3;
  realintr($10, regs);
  regs.cx := regs.cx And $dfff;
  regs.ah := 1;
  realintr($10, regs);
  regs.ax := $1003; { Enable blink }
  regs.bl := 1;
  realintr($10, regs);
End;

Procedure border(color : Byte); { _ONLY_ for debugging! }

Begin
  inportb($3da);
  outportb($3c0, 17+32);
  outportb($3c0, color);
End;

Procedure vrc; { Although all should be timer-synced instead.. }

Begin
  While (inportb($3da) And 8) = 0 Do ;
  While (inportb($3da) And 8) <> 0 Do ;
End;

{#define COLMAP(r,g,b) *(colmap+((r)<<(COLORMAP_DEPTH*2))+((g)<<COLORMAP_DEPTH)+(b))}
Function COLMAP_(r, g, b : Integer) : Integer;{ Inline;}

Begin
  COLMAP_ := (colmap + ((r Shl (COLORMAP_DEPTH*2)) + (g Shl COLORMAP_DEPTH) + b))^;
End;

Procedure COLMAPSet(r, g, b, v : Integer);{ Inline;}

Begin
  (colmap + ((r Shl (COLORMAP_DEPTH*2)) + (g Shl COLORMAP_DEPTH) + b))^ := v;
End;

Function calcpal_colorbase(red, green, blue : Real) : Word;

Var
  a, b, c, d, ch, co : Integer;
  lastdist, dist : Double;

Begin
  red := red * 1.2;
  green := green * 1.2;
  blue := blue * 1.2;
  lastdist := 1e242;
  d := 0;
  For c := 0 To 15 Do
  Begin
    dist := sqr(palette[d + 0] - red) +
            sqr(palette[d + 1] - green) +
            sqr(palette[d + 2] - blue);
    If dist < lastdist Then
    Begin
      lastdist := dist;
      co := c;
      ch := 219; { 100% block in IBMSCII }
    End;
    Inc(d, 3);
  End;
  c := co;
  d := c*3;
  a := 0;
  For b := 0 To 15 Do
  Begin
    dist := sqr(((palette[a+0]+palette[d+0]) / 2.0) - red) +
            sqr(((palette[a+1]+palette[d+1]) / 2.0) - green) +
            sqr(((palette[a+2]+palette[d+2]) / 2.0) - blue);
    If dist < lastdist Then
    Begin
      lastdist := dist;
      co := b + (c Shl 4);
      ch := 177; { 50% block in IBMSCII }
    End;
    {$IFDEF __USE_178NOT176}
      dist := sqr((palette[a+0]*0.75+palette[d+0]*0.25) - red) +
              sqr((palette[a+1]*0.75+palette[d+1]*0.25) - green) +
              sqr((palette[a+2]*0.75+palette[d+2]*0.25) - blue);
      If dist < lastdist Then
      Begin
        lastdist := dist;
        co := b + (c Shl 4);
        ch := 178; { 75% block in IBMSCII }
      End;
      dist := sqr((palette[a+0]*0.25+palette[d+0]*0.75) - red) +
              sqr((palette[a+1]*0.25+palette[d+1]*0.75) - green) +
              sqr((palette[a+2]*0.25+palette[d+2]*0.75) - blue);
      If dist < lastdist Then
      Begin
        lastdist := dist;
        co := c + (b Shl 4);
        ch := 178; { 75% block in IBMSCII }
      End;
    {$ELSE}
      dist := sqr((palette[a+0]*0.25+palette[d+0]*0.75) - red) +
              sqr((palette[a+1]*0.25+palette[d+1]*0.75) - green) +
              sqr((palette[a+2]*0.25+palette[d+2]*0.75) - blue);
      If dist < lastdist Then
      Begin
        lastdist := dist;
        co := b + (c Shl 4);
        ch := 176; { 25% block in IBMSCII }
      End;
      dist := sqr((palette[a+0]*0.75+palette[d+0]*0.25) - red) +
              sqr((palette[a+1]*0.75+palette[d+1]*0.25) - green) +
              sqr((palette[a+2]*0.75+palette[d+2]*0.25) - blue);
      If dist < lastdist Then
      Begin
        lastdist := dist;
        co := c + (b Shl 4);
        ch := 176; { 25% block in IBMSCII }
      End;
    {$ENDIF}
    Inc(a, 3);
  End;
  calcpal_colorbase := (co Shl 8) + ch;
End;

{*
 * Unlike _colorbase, _lightbase and _gscale calculations are
 * based on some trivial assumptions, such as that the character
 * tables have linear grayscale ramps and stuff like that.
 *
 * ie: they are *not* accurate!
 *
 * The tables were generated by calculating the dot distance from
 * center of character, ((xdistmax-xdist)^2)+((ydistmax-ydist)^2),
 * and sorting by this value. (HOW are you supposed to calculate
 * random pattern lightness value anyway?! =)
 *
 * Bright and dark color values are just thrown in without any
 * math background. (How could there be some? At this point you
 * should realize we have thrown all accurancy out the window).
 *
 * So. They work - kinda. They don't work correctly, but there
 * you go.
 *
 * color ramp= (dark color) [0 .. 1] + (light color) [0.3 .. 1]
 *
 * (didn't bother to rip AAlib :)
 *}

Function calcpal_lightbase(red, green, blue : Real) : Word;

Var
  light, col, a, a3 : Integer;
  lastdist, dist : Real;

Begin
  lastdist := 1e24;
  a3 := 3;
  For a := 1 To 15 Do
  Begin
    dist := Sqr(palette[a * 3 + 0] - red) +
            Sqr(palette[a * 3 + 1] - green) +
            Sqr(palette[a * 3 + 2] - blue);
    If dist < lastdist Then
    Begin
      lastdist := dist;
      col := a;
    End;
    Inc(a3, 3);
  End;
  light := Trunc(((0.2990 * red + 0.5870 * green + 0.1140 * blue) / 63) * 64);
  If light < 32 Then
    light := Trunc(((0.2990 * red + 0.5870 * green + 0.1140 * blue) / 63) * 1.5 * use_charset^)
  Else
    light := Trunc(((0.2990 * red + 0.5870 * green + 0.1140 * blue) / 63) * use_charset^);
  calcpal_lightbase := (col Shl 8) + (use_charset + light + 1)^;
End;

Function calcpal_lightbase_g(red, green, blue : Real) : Word;

Var
  light : Integer;

Begin
  light := Trunc(((0.2990 * red + 0.5870 * green + 0.1140 * blue) / 63) * use_charset^);
  calcpal_lightbase_g := (7 Shl 8) + (use_charset + light + 1)^;
End;

Function calc_gscale(light : Real) : Word;

Begin
  calc_gscale := (7 Shl 8) + (use_charset + Trunc(light * (use_charset^ + 1)))^;
End;

Function calc_gscale2(light : Real) : Word;

Begin
  If light < 0.3 Then
    calc_gscale2 := (8 Shl 8) + (use_charset + Trunc(light * 3 * (use_charset^ + 1)))^
  Else
    If light < 0.6 Then
      calc_gscale2 := (7 Shl 8) + (use_charset + Trunc((light + 0.3) * (use_charset^ + 1)))^
    Else
      calc_gscale2 := (15 Shl 8) + (use_charset + Trunc(light * (use_charset^ + 1)))^;
End;

Procedure build_colormap(dots : Integer);

Const
  wheel : Array[0..3] Of Char = ('-', '\', '|', '/');

Var
  r, g, b : Integer;
  f : Double;

Begin
  If dots = 2 Then
    Write(' ');
  If colmap <> Nil Then
    FreeMem(colmap);
  f := 64.0 / COLMAPDIM;
  colmap := GetMem(SizeOf(SmallInt) * COLMAPDIM * COLMAPDIM * COLMAPDIM);
  For r := 0 To COLMAPDIM - 1 Do
  Begin
    For g := 0 To COLMAPDIM - 1 Do
      For b := 0 To COLMAPDIM - 1 Do
        COLMAPSet(r, g, b, calcpal(r * f, g * f, b * f));
    If dots = 1 Then
      Write('.');
    If dots = 2 Then
      Write({#127}#8, wheel[r And 3]);
  End;
End;

Procedure dispose_colormap;

Begin
  If colmap <> Nil Then
    FreeMem(colmap);
  colmap := Nil;
End;

Procedure dump_80x(y0, y1 : Integer; buffer : PInteger);

Var
  x, y, yd : Integer;
  scr : DWord;
  buf : PByte;

Begin
  buf := PByte(buffer);
  scr := $b8000 + (y0 * 160);
  yd := y1 - y0;
  For y := 0 To yd - 1 Do
    For x := 0 To 79 Do
    Begin
      MemW[scr] := COLMAP_((buf + 0)^ Shr TRUCOLBITS,
                           (buf + 1)^ Shr TRUCOLBITS,
                           (buf + 2)^ Shr TRUCOLBITS);
      Inc(scr, 2);
      Inc(buf, 4);
    End;
End;

Procedure dump_160x(y0, y1 : Integer; buffer : PInteger);

Var
  x, y, yd : Integer;
  i : DWord;
  scr : DWord;
  buf : PByte;

Begin
  buf := @i;
  scr := $b8000 + (y0 * 160);
  yd := y1 - y0;
  For y := 0 To yd - 1 Do
  Begin
    For x := 0 To 79 Do
    Begin
      i := ((buffer+0)^ And $fcfcfcfc)+
           ((buffer+1)^ And $fcfcfcfc)+
           ((buffer+160)^ And $fcfcfcfc)+
           ((buffer+161)^ And $fcfcfcfc);
      i := i Shr 2;
      i := i And $fcfcfcfc;
      MemW[scr] := COLMAP_((buf + 0)^ Shr TRUCOLBITS,
                           (buf + 1)^ Shr TRUCOLBITS,
                           (buf + 2)^ Shr TRUCOLBITS);
      Inc(scr, 2);
      Inc(buffer, 2);
    End;
    Inc(buffer, 160);
  End;
End;

Procedure dump_320x(y0, y1 : Integer; buffer : PInteger);

Var
  x, y, yd, r, g, b, xx, yy : Integer;
  buf : PByte;
  scr : DWord;

Begin
  buf := PByte(buffer);
  scr := $b8000 + (y0 * 160);
  yd := y1 - y0;
  For y := 0 To yd - 1 Do
  Begin
    For x := 0 To 79 Do
    Begin
      r := 0; g := 0; b:= 0;
      xx := 0;
      While xx < 4 * 4 Do
      Begin
        yy := 0;
        While yy < 4 * 4 * 320 Do
        Begin
          Inc(r, (buf + xx + yy + 0)^);
          Inc(g, (buf + xx + yy + 1)^);
          Inc(b, (buf + xx + yy + 2)^);
          Inc(yy, 320 * 4);
        End;
        Inc(xx, 4);
      End;
      MemW[scr] := COLMAP_(r Shr (TRUCOLBITS + 4),
                           g Shr (TRUCOLBITS + 4),
                           b Shr (TRUCOLBITS + 4));
      Inc(scr, 2);
      Inc(buf, 4 * 4);
    End;
    Inc(buf, 80 * 4 * 4 * 3);
  End;
End;

End.
