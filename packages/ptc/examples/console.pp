{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Console example for OpenPTC 1.0 C++ implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

program ConsoleExample;

{$MODE objfpc}

uses
  ptc;

var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  console: IPTCConsole;
  palette: IPTCPalette;
=======
  console: TPTCConsole = nil;
  palette: TPTCPalette = nil;
>>>>>>> graemeg/cpstrnew
=======
  console: TPTCConsole = nil;
  palette: TPTCPalette = nil;
>>>>>>> graemeg/cpstrnew
=======
  console: TPTCConsole = nil;
  palette: TPTCPalette = nil;
>>>>>>> graemeg/cpstrnew
=======
  console: TPTCConsole = nil;
  palette: TPTCPalette = nil;
>>>>>>> origin/cpstrnew
  data: array [0..255] of DWord;
  i: Integer;
  pixels: PByte;
  width, height, pitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  format: IPTCFormat;
=======
  format: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
  format: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
  format: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
  format: TPTCFormat;
>>>>>>> origin/cpstrnew
  bits, bytes: Integer;
  x, y: Integer;
  color: DWord;
  pixel: PByte;
  _data: PByte;
begin
  try
    try
      { create console }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      console := TPTCConsoleFactory.CreateNew;
=======
      console := TPTCConsole.Create;
>>>>>>> graemeg/cpstrnew
=======
      console := TPTCConsole.Create;
>>>>>>> graemeg/cpstrnew
=======
      console := TPTCConsole.Create;
>>>>>>> graemeg/cpstrnew
=======
      console := TPTCConsole.Create;
>>>>>>> origin/cpstrnew

      { open the console with one page }
      console.open('Console example', 1);

      { create palette }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      palette := TPTCPaletteFactory.CreateNew;
=======
      palette := TPTCPalette.Create;
>>>>>>> graemeg/cpstrnew
=======
      palette := TPTCPalette.Create;
>>>>>>> graemeg/cpstrnew
=======
      palette := TPTCPalette.Create;
>>>>>>> graemeg/cpstrnew
=======
      palette := TPTCPalette.Create;
>>>>>>> origin/cpstrnew

      { generate palette }
      for i := 0 to 255 do
        data[i] := i;

      { load palette data }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      palette.Load(data);

      { set console palette }
      console.Palette(palette);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      palette.load(data);

      { set console palette }
      console.palette(palette);
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

      { loop until a key is pressed }
      while not console.KeyPressed do
      begin
        { lock console }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pixels := console.Lock;
=======
        pixels := console.lock;
>>>>>>> graemeg/cpstrnew
=======
        pixels := console.lock;
>>>>>>> graemeg/cpstrnew
=======
        pixels := console.lock;
>>>>>>> graemeg/cpstrnew
=======
        pixels := console.lock;
>>>>>>> origin/cpstrnew

        try
          { get console dimensions }
          width := console.width;
          height := console.height;
          pitch := console.pitch;

          { get console format }
          format := console.format;

          { get format information }
          bits := format.bits;
          bytes := format.bytes;

          { draw random pixels }
          for i := 1 to 100 do
          begin
            { get random position }
            x := Random(width);
            y := Random(height);

            { generate random color integer }
            color := (DWord(Random(256)) shl 0) or
                     (DWord(Random(256)) shl 8) or
                     (DWord(Random(256)) shl 16) or
                     (DWord(Random(256)) shl 24);

            { calculate pointer to pixel [x,y] }
            pixel := pixels + y * pitch + x * bytes;

            { check bits }
            case bits of
                   { 32 bits per pixel }
              32: PDWord(pixel)^ := color;
              24: begin
                { 24 bits per pixel }
                _data := pixel;
                _data[0] := (color and $000000FF) shr 0;
                _data[1] := (color and $0000FF00) shr 8;
                _data[2] := (color and $00FF0000) shr 16;
              end;
                   { 16 bits per pixel }
              16: PWord(pixel)^ := color;
                  { 8 bits per pixel }
              8: PByte(pixel)^ := color;
            end;
          end;
        finally
          { unlock console }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          console.Unlock;
        end;

        { update console }
        console.Update;
      end;
    finally
      if Assigned(console) then
        console.Close;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          console.unlock;
        end;

        { update console }
        console.update;
      end;
    finally
      palette.Free;
      console.close;
      console.Free;
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
