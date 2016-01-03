{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Keyboard example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program KeyboardExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;
  color : TPTCColor;
  key : TPTCKey;
  area : TPTCArea;
  x, y : Integer;
  size : Integer;
  delta : Integer;

Begin
  key := Nil;
  color := Nil;
  format := Nil;
  surface := Nil;
  console := Nil;
  Try
    Try
      { create key }
      key := TPTCKey.Create;
      
      { create console }
      console := TPTCConsole.Create;

      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { open the console }
      console.open('Keyboard example', format);

      { create surface matching console dimensions }
      surface := TPTCSurface.Create(console.width, console.height, format);

      { setup cursor data }
      x := surface.width Div 2;
      y := surface.height Div 2;
      size := surface.width Div 10;
      color := TPTCColor.Create(1, 1, 1);

      { main loop }
      Repeat
        { check for key press }
        If console.KeyPressed Then
        Begin
          { read console key press }
          console.ReadKey(key);
	
          { shift modifier }
	  If key.shift Then
            { move fast }
	    delta := 10
	  Else
            { move slow }
	    delta := 1;

          { handle cursor keys }
          Case key.code Of
            PTCKEY_LEFT : Dec(x, delta);
            PTCKEY_RIGHT : Inc(x, delta);
            PTCKEY_UP : Dec(y, delta);
            PTCKEY_DOWN : Inc(y, delta);
            { exit when escape is pressed }
            PTCKEY_ESCAPE : Break;
          End;
        End;

        { clear surface }
        surface.clear;

        { setup cursor area }
        area := TPTCArea.Create(x - size, y - size, x + size, y + size);
        Try
          { draw cursor as a quad }
          surface.clear(color, area);
        Finally
          area.Free;
        End;

        { copy to console }
        surface.copy(console);

        { update console }
        console.update;
      Until False;
    Finally
      color.Free;
      console.close;
      console.Free;
      surface.Free;
      key.Free;
      format.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
