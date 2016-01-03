{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Image example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program ImageExample;

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

  { read image pixels one line at a time }
  For y := height - 1 DownTo 0 Do
    BlockRead(F, pixels[width * y * 3], width * 3);

  { load pixels to surface }
  tmp := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
  tmp2 := TPTCPalette.Create;
  surface.load(pixels, width, height, width * 3, tmp, tmp2);
  tmp2.Free;
  tmp.Free;

  { free image pixels }
  FreeMem(pixels);
End;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  surface : TPTCSurface;

Begin
  Try
    { create console }
    console := TPTCConsole.Create;

    { create format }
    format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

    Try
      { try to open the console matching the image resolution }
      console.open('Image example', 320, 200, format);
    Except
      On TPTCError Do
        { fallback to the default resolution }
        console.open('Image example', format);
    End;

    { create surface }
    surface := TPTCSurface.Create(320, 200, format);
    format.Free;

    { load image to surface }
    load(surface, 'image.tga');

    { copy surface to console }
    surface.copy(console);

    { update console }
    console.update;

    { read key }
    console.ReadKey;

    { close console }
    console.close;

    console.Free;
    surface.Free;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
