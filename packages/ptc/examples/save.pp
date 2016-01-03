{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Save example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program SaveExample;

{$MODE objfpc}

Uses
  ptc, Math;

Procedure save(surface : TPTCSurface; filename : String);

Const
  { generate the header for a true color targa image }
  header : Array[0..17] Of char8 =
    (0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

Var
  F : File;
  width, height : Integer;
  size : Integer;
  y : Integer;
  pixels : Pchar8;
  format : TPTCFormat;
  palette : TPTCPalette;

Begin
  { open image file for writing }
  ASSign(F, filename);
  Rewrite(F, 1);

  { get surface dimensions }
  width := surface.width;
  height := surface.height;

  { set targa image width }
  header[12] := width And $FF;
  header[13] := width Shr 8;

  { set targa image height }
  header[14] := height And $FF;
  header[15] := height Shr 8;

  { set bits per pixel }
  header[16] := 24;

  { write tga header }
  BlockWrite(F, header, 18);

  { calculate size of image pixels }
  size := width * height * 3;

  { allocate image pixels }
  pixels := GetMem(size);

  format := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
  palette := TPTCPalette.Create;

  { save surface to image pixels }
  surface.save(pixels, width, height, width * 3, format, palette);

  palette.Free;
  format.Free;

  { write image pixels one line at a time }
  For y := height - 1 DownTo 0 Do
    BlockWrite(F, pixels[width * y * 3], width * 3);

  { free image pixels }
  FreeMem(pixels);

  Close(F);
End;

Function calculate(real, imaginary : Single; maximum : Integer) : Integer;

Var
  c_r, c_i : Single;
  z_r, z_i : Single;
  z_r_squared, z_i_squared : Single;
  z_squared_magnitude : Single;
  count : Integer;

Begin
  { complex number 'c' }
  c_r := real;
  c_i := imaginary;

  { complex 'z' }
  z_r := 0;
  z_i := 0;

  { complex 'z' squares }
  z_r_squared := 0;
  z_i_squared := 0;

  { mandelbrot function iteration loop }
  For count := 0 To maximum - 1 Do
  Begin
    { square 'z' and add 'c' }
    z_i := 2 * z_r * z_i + c_i;
    z_r := z_r_squared - z_i_squared + c_r;

    { update 'z' squares }
    z_r_squared := z_r * z_r;
    z_i_squared := z_i * z_i;

    { calculate squared magnitude of complex 'z' }
    z_squared_magnitude := z_r_squared + z_i_squared;

    { stop iterating if the magnitude of 'z' is greater than two }
    If z_squared_magnitude > 4 Then
    Begin
      calculate := Count;
      Exit;
    End;
  End;

  { maximum }
  calculate := 0;
End;

Procedure mandelbrot(console : TPTCConsole; surface : TPTCSurface;
		     x1, y1, x2, y2 : Single);

Const
  { constant values }
  entries = 1024;
  maximum = 1024;

Var
  { fractal color table }
  table : Array[0..entries - 1] Of int32;
  i : Integer;
  f_index : Single;
  time : Single;
  intensity : Single;
  pixels, pixel : Pint32;
  width, height : Integer;
  dx, dy : Single;
  real, imaginary : Single;
  x, y : Integer;
  count : Integer;
  index : Integer;
  color : int32;
  area : TPTCArea;

Begin
  { generate fractal color table }
  For i := 0 To entries - 1 Do
  Begin
    { calculate normalized index }
    f_index := i / entries;

    { calculate sine curve time value }
    time := f_index * pi - pi / 2;

    { lookup sine curve intensity at time and scale to [0,1] }
    intensity := (sin(time) + 1) / 2;

    { raise the intensity to a power }
    intensity := power(intensity, 0.1);

    { store intensity as a shade of blue }
    table[i] := Trunc(255 * intensity);
  End;

  { lock surface pixels }
  pixels := surface.lock;
  Try
    { get surface dimensions }
    width := surface.width;
    height := surface.height;

    { current pixel pointer }
    pixel := pixels;

    { calculate real x,y deltas }
    dx := (x2 - x1) / width;
    dy := (y2 - y1) / height;

    { imaginary axis }
    imaginary := y1;

    { iterate down surface y }
    For y := 0 To height - 1 Do
    Begin
      { real axis }
      real := x1;

      { iterate across surface x }    
      For x := 0 To width - 1 Do
      Begin
        { calculate the mandelbrot interation count }
        count := calculate(real, imaginary, maximum);

        { calculate color table index }
        index := count Mod entries;

        { lookup color from iteration }
        color := table[index];

        { store color }
        pixel^ := color;

        { next pixel }
        Inc(pixel);

        { update real }
        real := real + dx;
      End;

      { update imaginary }
      imaginary := imaginary + dy;

      { setup line area }
      area := TPTCArea.Create(0, y, width, y + 1);
      Try
        { copy surface area to console }
        surface.copy(console, area, area);
      Finally
        area.Free;
      End;

      { update console area }
      console.update;
    End;
  Finally
    { unlock surface }
    surface.unlock;
  End;
End;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;
  x1, y1, x2, y2 : Single;

Begin
  format := Nil;
  surface := Nil;
  console := Nil;
  Try
    Try
      { create console }
      console := TPTCConsole.Create;

      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { open the console with a single page }
      console.open('Save example', format, 1);

      { create surface matching console dimensions }
      surface := TPTCSurface.Create(console.width, console.height, format);

      { setup viewing area }
      x1 := -2.00;
      y1 := -1.25;
      x2 := +1.00;
      y2 := +1.25;

      { render the mandelbrot fractal }
      mandelbrot(console, surface, x1, y1, x2, y2);

      { save mandelbrot image }
      save(surface, 'save.tga');

      { read key }
      console.ReadKey;
    Finally
      console.close;
      console.Free;
      surface.Free;
      format.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
