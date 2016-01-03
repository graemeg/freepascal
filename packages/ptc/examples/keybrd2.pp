Program KeyboardExample2;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;
  color : TPTCColor;
  timer : TPTCTimer;
  key : TPTCKey;
  area : TPTCArea;
  x, y, delta : Real;
  left, right, up, down : Boolean;
  size : Integer;
  Done : Boolean;

Begin
  left := False;
  right := False;
  up := False;
  down := False;
  Try
    Try
      { create key }
      key := TPTCKey.Create;
      
      { create console }
      console := TPTCConsole.Create;
      
      { enable key release events }
      console.KeyReleaseEnabled := True;

      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { open the console }
      console.open('Keyboard example 2', format);
      
      { create timer }
      timer := TPTCTimer.Create;
      
      { create surface matching console dimensions }
      surface := TPTCSurface.Create(console.width, console.height, format);

      { setup cursor data }
      x := surface.width Div 2;
      y := surface.height Div 2;
      size := surface.width Div 10;
      color := TPTCColor.Create(1, 1, 1);
      
      { start timer }
      timer.start;

      { main loop }
      Done := False;
      Repeat
        { check for key press/release }
        While console.KeyPressed Do
        Begin
          console.ReadKey(key);
          Case key.code Of
	    PTCKEY_LEFT : left := key.press;
	    PTCKEY_RIGHT : right := key.press;
	    PTCKEY_UP : up := key.press;
	    PTCKEY_DOWN : down := key.press;
	    PTCKEY_ESCAPE : Begin
	      Done := True;
	      Break;
	    End;
	  End;
        End;

        { move square }
        delta := timer.delta*100;
        If left Then
          x -= delta;
        If right Then
          x += delta;
        If up Then
          y -= delta;
        If down Then
          y += delta;

        { clear surface }
        surface.clear;

        { setup cursor area }
        area := TPTCArea.Create(Trunc(x) - size, Trunc(y) - size, Trunc(x) + size, Trunc(y) + size);
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
      Until Done;
    Finally
      color.Free;
      console.close;
      console.Free;
      surface.Free;
      key.Free;
      timer.Free;
      format.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
