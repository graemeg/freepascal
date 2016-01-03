{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Pixel example for OpenPTC 1.0 C++ API
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is licensed under the GNU GPL
}

Program PixelExample;

{$MODE objfpc}

Uses
  ptc;

Procedure putpixel(surface : TPTCSurface; x, y : Integer; r, g, b : char8);

Var
  pixels : Pint32;
  color : int32;

Begin
  { lock surface }
  pixels := surface.lock;
  Try
    { pack the color integer from r,g,b components }
    color := (r Shl 16) Or (g Shl 8) Or b;

    { plot the pixel on the surface }
    pixels[x + y * surface.width] := color;
  Finally
    { unlock surface }
    surface.unlock;
  End;
End;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;

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

      { open the console }
      console.open('Pixel example', format);

      { create surface matching console dimensions }
      surface := TPTCSurface.Create(console.width, console.height, format);

      { plot a white pixel in the middle of the surface }
      putpixel(surface, surface.width Div 2, surface.height Div 2, 255, 255, 255);

      { copy to console }
      surface.copy(console);

      { update console }
      console.update;

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
