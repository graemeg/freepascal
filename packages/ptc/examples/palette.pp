{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Palette example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program PaletteExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;
  palette : TPTCPalette;
  data : Array[0..255] Of int32;
  pixels : Pchar8;
  width, height : Integer;
  i : Integer;
  x, y, index : Integer;

Begin
  Try
    { create console }
    console := TPTCConsole.Create;

    { create format }
    format := TPTCFormat.Create(8);

    { open console }
    console.open('Palette example', format);

    { create surface }
    surface := TPTCSurface.Create(console.width, console.height, format);
    format.Free;

    { create palette }
    palette := TPTCPalette.Create;

    { generate palette }
    For i := 0 To 255 Do
      data[i] := i;

    { load palette data }
    palette.load(data);

    { set console palette }
    console.palette(palette);

    { set surface palette }
    surface.palette(palette);
    palette.Free;

    { loop until a key is pressed }
    While Not console.KeyPressed Do
    Begin
      { lock surface }
      pixels := surface.lock;

      { get surface dimensions }
      width := surface.width;
      height := surface.height;

      { draw random pixels }
      For i := 1 To 100 Do
      Begin
        { get random position }
	x := Random(width);
	y := Random(height);

        { get random color index }
	index := Random(256);

        { draw color [index] at position [x,y] }
	pixels[x + y * width] := index;
      End;

      { unlock surface }
      surface.unlock;

      { copy to console }
      surface.copy(console);

      { update console }
      console.update;
    End;
    console.close;
    console.Free;
    surface.Free;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
