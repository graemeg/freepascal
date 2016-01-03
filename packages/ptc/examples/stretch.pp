{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Stretch example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program StretchExample;

{$MODE objfpc}

Uses
  ptc;

Procedure load(surface : TPTCSurface; filename : String);

Var
  F : File;
  width, height : Integer;
  pixels : PByte;
  y : Integer;
  tmp : TPTCFormat;
  tmp2 : TPTCPalette;

Begin
  { open image file }
  ASSign(F, filename);
  Reset(F, 1);

  { skip header }
  Seek(F, 18);

  { get surface dimensions }
  width := surface.width;
  height := surface.height;

  { allocate image pixels }
  pixels := GetMem(width * height * 3);
  Try
    { read image pixels one line at a time }
    For y := height - 1 DownTo 0 Do
      BlockRead(F, pixels[width * y * 3], width * 3);

    { load pixels to surface }
    tmp := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
    Try
      tmp2 := TPTCPalette.Create;
      Try
        surface.load(pixels, width, height, width * 3, tmp, tmp2);
      Finally
        tmp2.Free;
      End;
    Finally
      tmp.Free;
    End;
  Finally
    { free image pixels }
    FreeMem(pixels);
  End;
End;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  image : TPTCSurface;
  format : TPTCFormat;
  timer : TPTCTimer;
  area : TPTCArea;
  color : TPTCColor;
  time : Double;
  zoom : Single;
  x, y, x1, y1, x2, y2, dx, dy : Integer;

Begin
  format := Nil;
  color := Nil;
  timer := Nil;
  image := Nil;
  surface := Nil;
  console := Nil;
  Try
    Try
      { create console }
      console := TPTCConsole.Create;

      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { open the console }
      console.open('Stretch example', format);

      { create surface matching console dimensions }
      surface := TPTCSurface.Create(console.width, console.height, format);

      { create image surface }
      image := TPTCSurface.Create(320, 140, format);

      { load image to surface }
      load(image, 'stretch.tga');

      { setup stretching parameters }
      x := surface.width Div 2;
      y := surface.height Div 2;
      dx := surface.width Div 2;
      dy := surface.height Div 3;

      { create timer }
      timer := TPTCTimer.Create;

      { start timer }
      timer.start;
      color := TPTCColor.Create(1, 1, 1);

      { loop until a key is pressed }
      While Not console.KeyPressed Do
      Begin
        { get current time from timer }
        time := timer.time;

        { clear surface to white background }
        surface.clear(color);

        { calculate zoom factor at current time }
        zoom := 2.5 * (1 - cos(time));

        { calculate zoomed image coordinates }
        x1 := Trunc(x - zoom * dx);
        y1 := Trunc(y - zoom * dy);
        x2 := Trunc(x + zoom * dx);
        y2 := Trunc(y + zoom * dy);

        { setup image copy area }
        area := TPTCArea.Create(x1, y1, x2, y2);
	Try
          { copy and stretch image to surface }
          image.copy(surface, image.area, area);

          { copy surface to console }
          surface.copy(console);

          { update console }
          console.update;
	Finally
          area.Free;
	End;
      End;
    Finally
      console.close;
      console.Free;
      surface.Free;
      format.Free;
      image.Free;
      color.Free;
      timer.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
