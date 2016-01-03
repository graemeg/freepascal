{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Buffer example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program BufferExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  palette : TPTCPalette;
  width, height : Integer;
  pixels : Pint32;
  x, y, r, g, b : Integer;
  i : Integer;

Begin
  pixels := Nil;
  format := Nil;
  palette := Nil;
  console := Nil;
  Try
    Try
      { create console }
      console := TPTCConsole.Create;

      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { open the console }
      console.open('Buffer example', format);

      { get console dimensions }
      width := console.width;
      height := console.height;

      { allocate a buffer of pixels }
      pixels := GetMem(width * height * SizeOf(int32));
      palette := TPTCPalette.Create;

      { loop until a key is pressed }
      While Not console.KeyPressed Do
      Begin
        { draw random pixels }
        For i := 1 To 100 Do
        Begin
          { get random position }
	  x := Random(width);
	  y := Random(height);

          { get random color }	
	  r := Random(256);
	  g := Random(256);
	  b := Random(256);

          { draw color [r,g,b] at position [x,y] }
	  pixels[x + y * width] := (r Shl 16) Or (g Shl 8) Or b;
        End;

        { load pixels to console }
        console.load(pixels, width, height, width * 4, format, palette);

        { update console }
        console.update;
      End;
    Finally
      { free pixels buffer }
      If Assigned(pixels) Then
        FreeMem(pixels);
      console.close;
      palette.Free;
      format.Free;
      console.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
