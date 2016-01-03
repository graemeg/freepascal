{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Clear example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program ClearExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  surface : TPTCSurface;
  width, height : Integer;
  x, y : Integer;
  size : Integer;
  area : TPTCArea;
  color : TPTCColor;

Begin
  Try
    { create console }
    console := TPTCConsole.Create;

    { create format }
    format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

    { open the console }
    console.open('Clear example', format);

    { create surface matching console dimensions }
    surface := TPTCSurface.Create(console.width, console.height, format);

    { loop until a key is pressed }
    While Not console.KeyPressed Do
    Begin
      { get surface dimensions }
      width := surface.width;
      height := surface.height;

      { get random position }
      x := Random(width);
      y := Random(height);

      { get random area size }
      size := Random(width Div 8);

      { setup clear area }
      area := TPTCArea.Create(x-size, y-size, x+size, y+size);

      { create random color }
      color := TPTCColor.Create(Random, Random, Random);

      { clear surface area with color }
      surface.clear(color, area);

      { copy to console }
      surface.copy(console);

      { update console }
      console.update;
      area.Free;
      color.Free;
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
