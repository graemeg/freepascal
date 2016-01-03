{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Flower demo for OpenPTC 1.0 C++ API
 Copyright (c) Scott Buchanan (aka Goblin)
 This source code is licensed under the GNU GPL
}

<<<<<<< HEAD
<<<<<<< HEAD
program Flower;

{$MODE objfpc}

uses
  ptc, Math;

function pack(r, g, b: Uint32): Uint32;
begin
  { pack color integer }
  pack := (r shl 16) or (g shl 8) or b;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure generate_flower(flower: IPTCSurface);
=======
procedure generate_flower(flower: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure generate_flower(flower: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure generate_flower(flower: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure generate_flower(flower: TPTCSurface);
>>>>>>> origin/cpstrnew
var
  data: PUint8;
  x, y, fx, fy, fx2, fy2: Integer;
  TWO_PI: Single;
begin
  { lock surface }
  data := flower.lock;

  try
    { surface width and height constants for cleaner code }
    fx := flower.width;
    fy := flower.height;
    fx2 := fx div 2;
    fy2 := fy div 2;
=======
=======
>>>>>>> origin/fixes_2_2
Program Flower;

{$MODE objfpc}

Uses
  ptc, Math;

Function pack(r, g, b : Uint32) : Uint32;

Begin
  { pack color integer }
  pack := (r Shl 16) Or (g Shl 8) Or b;
End;

Procedure generate_flower(flower : TPTCSurface);

Var
  data : PUint8;
  x, y, fx, fy, fx2, fy2 : Integer;
  TWO_PI : Single;

Begin
  { lock surface }
  data := flower.lock;
  
  Try
    { surface width and height constants for cleaner code }
    fx := flower.width;
    fy := flower.height;
    fx2 := fx Div 2;
    fy2 := fy Div 2;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    { useful 2*pi constant }
    TWO_PI := 2 * PI;

    { generate flower image }
<<<<<<< HEAD
<<<<<<< HEAD
    for y := 0 to fy - 1 do
      for x := 0 to fx - 1 do
        data[x + y * fx] := Trunc(1.0 * Cos(18*ArcTan2((y - fy2),(x - fx2))) * 255 / TWO_PI +
                                  0.3 * Sin(15*ArcTan2((y - fy2),(x - fx2))) * 255 / TWO_PI +
                                  Sqrt((y - fy2) * (y - fy2) + (x - fx2) * (x - fx2))) and $FF;
=======
=======
>>>>>>> origin/fixes_2_2
    For y := 0 To fy - 1 Do
      For x := 0 To fx - 1 Do
        data[x + y * fx] := Trunc(1.0 * Cos(18*ArcTan2((y - fy2),(x - fx2))) * 255 / TWO_PI +
		                  0.3 * Sin(15*ArcTan2((y - fy2),(x - fx2))) * 255 / TWO_PI +
                                  Sqrt((y - fy2) * (y - fy2) + (x - fx2) * (x - fx2))) And $FF;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    { You might want to move the 1.0 and 0.3 and the 18 and the 15
      to parameters passed to the generate function...
      the 1.0 and the 0.3 define the 'height' of the flower, while the
      18 and 15 control the number of 'petals' }
<<<<<<< HEAD
<<<<<<< HEAD
  finally
    flower.unlock;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure generate(palette: IPTCPalette);
=======
procedure generate(palette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure generate(palette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure generate(palette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure generate(palette: TPTCPalette);
>>>>>>> origin/cpstrnew
var
  data: PUint32;
  i, c: Integer;
begin
  { lock palette data }
<<<<<<< HEAD
  data := palette.Lock;
=======
  data := palette.lock;
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

  try
    { black to yellow }
    i := 0;
    c := 0;
    while i < 64 do
    begin
      data[i] := pack(c, c, 0);
      Inc(c, 4);
      Inc(i);
    end;

    { yellow to red }
    c := 0;
    while i < 128 do
    begin
      data[i] := pack(255, 255 - c, 0);
      Inc(c, 4);
      Inc(i);
    end;

    { red to white }
    c := 0;
    while i < 192 do
    begin
      data[i] := pack(255, c, c);
      Inc(c, 4);
      Inc(i);
    end;

    { white to black }
    c := 0;
    while i < 256 do
    begin
      data[i] := pack(255 - c, 255 - c, 255 - c);
      Inc(c, 4);
      Inc(i);
    end;
  finally
    { unlock palette }
<<<<<<< HEAD
    palette.Unlock;
=======
    palette.unlock;
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
end;

var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  console: IPTCConsole;
  format: IPTCFormat;
  flower_surface: IPTCSurface;
  surface: IPTCSurface;
  palette: IPTCPalette;
  area: IPTCArea;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  console: TPTCConsole = nil;
  format: TPTCFormat = nil;
  flower_surface: TPTCSurface = nil;
  surface: TPTCSurface = nil;
  palette: TPTCPalette = nil;
  area: TPTCArea = nil;
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
  time, delta: Single;
  scr, map: PUint8;
  width, height, mapWidth: Integer;
  xo, yo, xo2, yo2, xo3, yo3: Single;
  offset1, offset2, offset3: Integer;
  x, y: Integer;
begin
  try
    try
      { create format }
      format := TPTCFormatFactory.CreateNew(8);

      { create console }
      console := TPTCConsoleFactory.CreateNew;

      { create flower surface }
      flower_surface := TPTCSurfaceFactory.CreateNew(640, 400, format);
=======
=======
>>>>>>> origin/fixes_2_2
  Finally
    flower.unlock;
  End;
End;

Procedure generate(palette : TPTCPalette);

Var
  data : PUint32;
  i, c : Integer;

Begin
  { lock palette data }
  data := palette.lock;
  
  Try
    { black to yellow }
    i := 0;
    c := 0;
    While i < 64 Do
    Begin
      data[i] := pack(c, c, 0);
      Inc(c, 4);
      Inc(i);
    End;

    { yellow to red }
    c := 0;
    While i < 128 Do
    Begin
      data[i] := pack(255, 255 - c, 0);
      Inc(c, 4);
      Inc(i);
    End;

    { red to white }
    c := 0;
    While i < 192 Do
    Begin
      data[i] := pack(255, c, c);
      Inc(c, 4);
      Inc(i);
    End;

    { white to black }
    c := 0;
    While i < 256 Do
    Begin
      data[i] := pack(255 - c, 255 - c, 255 - c);
      Inc(c, 4);
      Inc(i);
    End;
  Finally
    { unlock palette }
    palette.unlock;
  End;
End;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  flower_surface : TPTCSurface;
  surface : TPTCSurface;
  palette : TPTCPalette;
  area : TPTCArea;
  time, delta : Single;
  scr, map : PUint8;
  width, height, mapWidth : Integer;
  xo, yo, xo2, yo2, xo3, yo3 : Single;
  offset1, offset2, offset3 : Integer;
  x, y : Integer;

Begin
  area := Nil;
  format := Nil;
  palette := Nil;
  surface := Nil;
  flower_surface := Nil;
  console := Nil;
  Try
    Try
      { create format }
      format := TPTCFormat.Create(8);

      { create console }
      console := TPTCConsole.Create;

      { create flower surface }
      flower_surface := TPTCSurface.Create(640, 400, format);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      { generate flower }
      generate_flower(flower_surface);

      { open console }
      console.open('Flower demo', 320, 200, format);

      { create surface }
<<<<<<< HEAD
<<<<<<< HEAD
      surface := TPTCSurfaceFactory.CreateNew(320, 200, format);

      { create palette }
      palette := TPTCPaletteFactory.CreateNew;
=======
=======
>>>>>>> origin/fixes_2_2
      surface := TPTCSurface.Create(320, 200, format);

      { create palette }
      palette := TPTCPalette.Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      { generate palette }
      generate(palette);

      { set console palette }
      console.palette(palette);

      { set surface palette }
      surface.palette(palette);

      { setup copy area }
<<<<<<< HEAD
<<<<<<< HEAD
      area := TPTCAreaFactory.CreateNew(0, 0, 320, 200);
=======
      area := TPTCArea.Create(0, 0, 320, 200);
>>>>>>> graemeg/fixes_2_2
=======
      area := TPTCArea.Create(0, 0, 320, 200);
>>>>>>> origin/fixes_2_2

      { time data }
      time := 0;
      delta := 0.04;

      { main loop }
<<<<<<< HEAD
<<<<<<< HEAD
      while not console.KeyPressed do
      begin
        { lock surface pixels }
        scr := surface.lock;
        try
          map := flower_surface.lock;
          try
=======
=======
>>>>>>> origin/fixes_2_2
      While Not console.KeyPressed Do
      Begin
        { lock surface pixels }
        scr := surface.lock;
	Try
          map := flower_surface.lock;
	  Try
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            { get surface dimensions }
            width := surface.width;
            height := surface.height;
            mapWidth := flower_surface.width;

            xo := (width / 2) + 120 * sin(time * 1.1 + 1.5);
            yo := (height / 2) + 90 * cos(time * 0.8 + 1.1);
            offset1 := Trunc(xo) + Trunc(yo) * mapWidth;

            xo2 := (width / 2) + 120 * sin(time * 0.9 + 4.2);
            yo2 := (height / 2) + 90 * cos(time * 0.7 + 6.9);
            offset2 := Trunc(xo2) + Trunc(yo2) * mapWidth;

            xo3 := (width / 2) + 120 * sin(time * 0.9 + 3.1);
            yo3 := (height / 2) + 90 * cos(time * 1.1 + 1.2);
            offset3 := Trunc(xo3) + Trunc(yo3) * mapWidth;

            { vertical loop }
<<<<<<< HEAD
<<<<<<< HEAD
            for y := 0 to height - 1 do
              { horizontal loop }
              for x := 0 to width - 1 do
                scr[x + y * width] := (map[x + y * mapWidth + offset1] +
                                       map[x + y * mapWidth + offset2] +
                                       map[x + y * mapWidth + offset3]) and $FF;
          finally
            { unlock surface }
            flower_surface.unlock;
          end;
        finally
          { unlock surface }
          surface.unlock;
        end;
=======
=======
>>>>>>> origin/fixes_2_2
            For y := 0 To height - 1 Do
              { horizontal loop }
	      For x := 0 To width - 1 Do
	        scr[x + y * width] := (map[x + y * mapWidth + offset1] +
				       map[x + y * mapWidth + offset2] +
				       map[x + y * mapWidth + offset3]) And $FF;
	  Finally
            { unlock surface }
            flower_surface.unlock;
	  End;
	Finally
          { unlock surface }
          surface.unlock;
	End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

        { copy surface to console }
        surface.copy(console, area, area);

        { update console }
        console.update;

        { update time }
        time := time + delta;
<<<<<<< HEAD
<<<<<<< HEAD
      end;
    finally
      if Assigned(console) then
        console.close;
<<<<<<< HEAD
=======
=======
=======
>>>>>>> origin/fixes_2_2
      End;
    Finally
      If Assigned(console) Then
        console.close;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      area.Free;
      format.Free;
      palette.Free;
      surface.Free;
      flower_surface.Free;
      console.Free;
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
