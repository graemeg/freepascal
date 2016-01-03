{
 Keyboard example for the PTCPas library
 This source code is in the public domain
}

program KeyboardExample2;

{$MODE objfpc}

uses
  ptc;

var
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
  color: TPTCColor = nil;
  timer: TPTCTimer = nil;
  key: TPTCKeyEvent = nil;
  area: TPTCArea;
  x, y, delta: Real;
  left, right, up, down: Boolean;
  size: Integer;
  Done: Boolean;
begin
  left := False;
  right := False;
  up := False;
  down := False;
  try
    try
      { create key }
      key := TPTCKeyEvent.Create;

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
      x := surface.width div 2;
      y := surface.height div 2;
      size := surface.width div 10;
      color := TPTCColor.Create(1, 1, 1);

      { start timer }
      timer.start;

      { main loop }
      Done := False;
      repeat
        { check for key press/release }
        while console.KeyPressed do
        begin
          console.ReadKey(key);
          case key.code of
            PTCKEY_LEFT: left := key.press;
            PTCKEY_RIGHT: right := key.press;
            PTCKEY_UP: up := key.press;
            PTCKEY_DOWN: down := key.press;
            PTCKEY_ESCAPE: begin
              Done := True;
              Break;
            end;
          end;
        end;

        { move square }
        delta := timer.delta*100;
        if left then
          x := x - delta;
        if right then
          x := x + delta;
        if up then
          y := y - delta;
        if down then
          y := y + delta;

        { clear surface }
        surface.clear;

        { setup cursor area }
        area := TPTCArea.Create(Trunc(x) - size, Trunc(y) - size, Trunc(x) + size, Trunc(y) + size);
        try
          { draw cursor as a quad }
          surface.clear(color, area);
        finally
          area.Free;
        end;

        { copy to console }
        surface.copy(console);

        { update console }
        console.update;
      until Done;
    finally
      color.Free;
      console.close;
      console.Free;
      surface.Free;
      key.Free;
      timer.Free;
      format.Free;
    end;
  except
    on error: TPTCError do
      { report error }
      error.report;
  end;
end.
