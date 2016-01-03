{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Area example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program AreaExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  surface : TPTCSurface;
  pixels : PDWord;
  width, height : Integer;
  i : Integer;
  x, y, r, g, b : Integer;
  area : TPTCArea;

Begin
  area := Nil;
  format := Nil;
  surface := Nil;
  console := Nil;
  Try
    Try
      { create console }
      console := TPTCConsole.Create;

      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { create console }
      console.open('Area example', format);

      { create surface half the size of the console }
      surface := TPTCSurface.Create(console.width Div 2, console.height Div 2, format);
      
      { setup destination area }
      x := console.width Div 4;
      y := console.height Div 4;
      area := TPTCArea.Create(x, y, x + surface.width, y + surface.height);

      { loop until a key is pressed }
      While Not console.KeyPressed Do
      Begin
        { lock surface }
        pixels := surface.lock;
        Try
          { get surface dimensions }
          width := surface.width;
          height := surface.height;

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
	    pixels[x + y * width] := (r Shl 16) + (g Shl 8) + b;
          End;
	Finally
          { unlock surface }
          surface.unlock;
	End;

        { copy surface to console destination area }
        surface.copy(console, surface.area, area);

        { update console area }
        console.update;
      End;
    Finally
      console.close;
      console.Free;
      surface.Free;
      format.Free;
      area.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
