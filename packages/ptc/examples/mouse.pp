{
 Mouse example for the PTCPas library
 This source code is in the public domain
}

program MouseExample;

{$MODE objfpc}

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ptc, SysUtils;

var
  console: IPTCConsole;
  surface: IPTCSurface;
  format: IPTCFormat;
  event: IPTCEvent;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ptc;

var
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
  event: TPTCEvent = nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  pixels: PUint32;
  color: Uint32;
  width, height: Integer;
  I: Integer;
  X, Y: Integer;
  button: Boolean;
  Done: Boolean = False;
begin
  try
    try
      { create console }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      console := TPTCConsoleFactory.CreateNew;

      { create format }
      format := TPTCFormatFactory.CreateNew(32, $FF0000, $FF00, $FF);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      console := TPTCConsole.Create;

      { create format }
      format := TPTCFormat.Create(32, $FF0000, $FF00, $FF);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

      { open the console }
      console.open('Mouse example', format);

      { we're going to draw our own cursor, so disable the default cursor }
      console.option('hide cursor');

      { create surface matching console dimensions }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      surface := TPTCSurfaceFactory.CreateNew(console.width, console.height, format);
=======
      surface := TPTCSurface.Create(console.width, console.height, format);
>>>>>>> graemeg/cpstrnew
=======
      surface := TPTCSurface.Create(console.width, console.height, format);
>>>>>>> graemeg/cpstrnew
=======
      surface := TPTCSurface.Create(console.width, console.height, format);
>>>>>>> graemeg/cpstrnew
=======
      surface := TPTCSurface.Create(console.width, console.height, format);
>>>>>>> origin/cpstrnew

      { initialization }
      X := 0;
      Y := 0;

      repeat
        { wait for events }
        console.NextEvent(event, True, PTCAnyEvent);

        { handle mouse events }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if Supports(event, IPTCMouseEvent) then
        begin
          { if there's more than one mouse event, process them all... }
          repeat
            X := (event as IPTCMouseEvent).X;
            Y := (event as IPTCMouseEvent).Y;
            button := PTCMouseButton1 in (event as IPTCMouseEvent).ButtonState;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if event is TPTCMouseEvent then
        begin
          { if there's more than one mouse event, process them all... }
          repeat
            X := (event as TPTCMouseEvent).X;
            Y := (event as TPTCMouseEvent).Y;
            button := PTCMouseButton1 in (event as TPTCMouseEvent).ButtonState;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          until not console.NextEvent(event, False, [PTCMouseEvent]);
        end;

        { handle keyboard events }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if Supports(event, IPTCKeyEvent) and (event as IPTCKeyEvent).Press then
        begin
          case (event as IPTCKeyEvent).Code of
=======
        if (event is TPTCKeyEvent) and (event as TPTCKeyEvent).Press then
        begin
          case (event as TPTCKeyEvent).Code of
>>>>>>> graemeg/cpstrnew
=======
        if (event is TPTCKeyEvent) and (event as TPTCKeyEvent).Press then
        begin
          case (event as TPTCKeyEvent).Code of
>>>>>>> graemeg/cpstrnew
=======
        if (event is TPTCKeyEvent) and (event as TPTCKeyEvent).Press then
        begin
          case (event as TPTCKeyEvent).Code of
>>>>>>> graemeg/cpstrnew
=======
        if (event is TPTCKeyEvent) and (event as TPTCKeyEvent).Press then
        begin
          case (event as TPTCKeyEvent).Code of
>>>>>>> origin/cpstrnew
            PTCKEY_G: console.Option('grab mouse');
            PTCKEY_U: console.Option('ungrab mouse');
            PTCKEY_ESCAPE: Done := True;
          end;
        end;

        { clear surface }
        surface.clear;

        { lock surface }
        pixels := surface.lock;

        try
          { get surface dimensions }
          width := surface.width;
          height := surface.height;

          if button then
            color := $00FF00 { green cursor, if button 1 is pressed }
          else
            color := $FFFFFF; { white cursor if button 1 is not pressed }

          { draw a small cross for a cursor }
          for I := 2 to 10 do
          begin
            if (X - I) >= 0 then
              pixels[X - I + Y * width] := color;

            if (X + I) < width then
              pixels[X + I + Y * width] := color;

            if (Y - I) >= 0 then
              pixels[X + (Y - I) * width] := color;

            if (Y + I) < height then
              pixels[X + (Y + I) * width] := color;
          end;

        finally
          { unlock surface }
          surface.unlock;
        end;

        { copy to console }
        surface.copy(console);

        { update console }
        console.update;

      until Done;
    finally
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if Assigned(console) then
        console.close;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      console.close;
      console.Free;
      surface.Free;
      format.Free;
      event.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;
  except
    on error: TPTCError do
      { report error }
      error.report;
  end;
end.
