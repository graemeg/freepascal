{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Console example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program ConsoleExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  palette : TPTCPalette;
  data : Array[0..255] Of DWord;
  i : Integer;
  pixels : PByte;
  width, height, pitch : Integer;
  format : TPTCFormat;
  bits, bytes : Integer;
  x, y : Integer;
  color : DWord;
  pixel : PByte;
  _data : PByte;

Begin
  Try
    { create console }
    console := TPTCConsole.Create;

    { open the console with one page }
    console.open('Console example', 1);

    { create palette }
    palette := TPTCPalette.Create;

    { generate palette }
    For i := 0 To 255 Do
      data[i] := i;

    { load palette data }
    palette.load(data);

    { set console palette }
    console.palette(palette);

    { loop until a key is pressed }
    While Not console.KeyPressed Do
    Begin
      { lock console }
      pixels := console.lock;

      { get console dimensions }
      width := console.width;
      height := console.height;
      pitch := console.pitch;

      { get console format }
      format := console.format;

      { get format information }
      bits := format.bits;
      bytes := format.bytes;

      { draw random pixels }
      For i := 1 To 100 Do
      Begin
        { get random position }
	x := Random(width);
	y := Random(height);

        { generate random color integer }
	color := (Random(256) Shl 0) Or
		 (Random(256) Shl 8) Or
		 (Random(256) Shl 16) Or
		 (Random(256) Shl 24);

        { calculate pointer to pixel [x,y] }
	pixel := pixels + y * pitch + x * bytes;

        { check bits }
	Case bits Of
               { 32 bits per pixel }
	  32 : PDWord(pixel)^ := color;
	  24 : Begin
            { 24 bits per pixel }
	    _data := pixel;
	    _data[0] := (color And $000000FF) Shr 0;
	    _data[1] := (color And $0000FF00) Shr 8;
	    _data[2] := (color And $00FF0000) Shr 16;
	  End;
               { 16 bits per pixel }
	  16 : PWord(pixel)^ := color;
              { 8 bits per pixel }
	  8 : PByte(pixel)^ := color;
	End;
      End;

      { unlock console }
      console.unlock;

      { update console }
      console.update;
    End;
    palette.Free;
    console.close;
    console.Free;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
