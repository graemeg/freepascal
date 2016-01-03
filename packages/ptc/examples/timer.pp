{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Timer example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program TimerExample;

{$MODE objfpc}

Uses
  ptc;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  surface : TPTCSurface;
  timer : TPTCTimer;
  time, t : Double;
  pixels : PDWord;
  width, height : Integer;
  repeats, center, magnitude, intensity, sx : Single;
  x, y : Integer;

Begin
  timer := Nil;
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
      console.open('Timer example', format);

      { create surface matching console dimensions }
      surface := TPTCSurface.Create(console.width, console.height, format);

      { create timer }
      timer := TPTCTimer.Create;

      { start timer }
      timer.start;

      { loop until a key is pressed }
      While Not console.KeyPressed Do
      Begin
        { get current time from timer }
        time := timer.time;

        { clear surface }
        surface.clear;

        { lock surface }
        pixels := surface.lock;
        Try
          { get surface dimensions }
          width := surface.width;
          height := surface.height;

          { sine curve parameters }
          repeats := 2;
          center := height / 2;
          magnitude := height / 3;

          { render a sine curve }
          For x := 0 To width - 1 Do
          Begin
            { rescale 'x' in the range [0,2*pi] }
	    sx := x / width * 2 * pi;

            { calculate time at current position }
	    t := time + sx * repeats;

            { lookup sine intensity at time 't' }
	    intensity := sin(t);

            { convert intensity to a y position on the surface }
	    y := Trunc(center + intensity * magnitude);

            { plot pixel on sine curve }
	    pixels[x + y * width] := $000000FF;
          End;
        Finally
          { unlock surface }
          surface.unlock;
	End;

        { copy to console }
        surface.copy(console);

        { update console }
        console.update;
      End;
    Finally
      timer.Free;
      surface.Free;
      console.close;
      console.Free;
      format.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
