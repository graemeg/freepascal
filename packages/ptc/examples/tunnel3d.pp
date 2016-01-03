{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Tunnel3D demo for OpenPTC 1.0 C++ API

 Realtime raytraced tunnel
 Copyright (c) 1998 Christian Nentwich (brn@eleet.mcb.at)
 This source code is licensed under the GNU LGPL

<<<<<<< HEAD
<<<<<<< HEAD
 and do not just blatantly cut&paste this into your demo :)
}

program Tunnel3D;

{$MODE objfpc}

uses
  ptc, Math;

type
  PVector = ^TVector;
  TVector = array [0..2] of Single;      { X,Y,Z }
  TMatrix = array [0..3, 0..3] of Single;{ FIRST  = COLUMN
=======
=======
>>>>>>> origin/fixes_2_2
 And do not just blatantly cut&paste this into your demo :)
}

Program Tunnel3D;

{$MODE objfpc}

Uses
  ptc, Math;

Type
  PVector = ^TVector;
  TVector = Array[0..2] Of Single;      { X,Y,Z }
  TMatrix = Array[0..3, 0..3] Of Single;{ FIRST  = COLUMN
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                                          SECOND = ROW

                                          [0, 0]  [1, 0]  [2, 0]
                                          [0, 1]  [1, 1]  [2, 1]
                                          [0, 2]  [1, 2]  [2, 2]
  (I know the matrices are the wrong way round, so what, the code is quite
  old :) }

<<<<<<< HEAD
<<<<<<< HEAD
  TRayTunnel = class
  private
    tunneltex: PUint8;                      { Texture }
    tunneltex_orig: PUint8;                 { Original start of texture memory block }
    pal: PUint8;                            { Original palette }
    lookup: PUint32;                         { Lookup table for lighting }

    sintab, costab: PSingle;                { Take a guess }

    u_array, v_array, l_array: PInteger;    { Raytraced coordinates and light }
    norms: PVector;

    radius, radius_sqr: Single;
    rot: TMatrix;

    pos, light: TVector;                    { Position in the tunnel, pos of }
    xa, ya, za: Integer;                    { lightsource, angles }

    lightstatus: Boolean;                   { Following the viewer ? }

  public
    constructor Create(rad: Single);        { constructor takes the radius }
    destructor Destroy; override;

    procedure load_texture;

    procedure tilt(x, y, z: Integer);              { Rotate relative }
    procedure tilt(x, y, z: Integer; abs: Uint8); { Absolute }

    procedure move(dx, dy, dz: Single);            { Relative move }
    procedure move(x, y, z: Single; abs: Uint8);  { Absolute }

    procedure movelight(dx, dy, dz: Single);
    procedure movelight(x, y, z: Single; abs: Uint8);

    procedure locklight(lock: Boolean);    { Make the light follow the viewer }

    procedure interpolate;                  { Raytracing }

    procedure draw(dest: PUint32);          { Draw the finished tunnel }
  end;

{ VECTOR ROUTINES }
procedure vector_normalize(var v: TVector);
var
  length: Single;
begin
  length := v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  length := sqrt(length);
  if length <> 0 then
  begin
    v[0] := v[0] / length;
    v[1] := v[1] / length;
    v[2] := v[2] / length;
  end
  else
  begin
    v[0] := 0;
    v[1] := 0;
    v[2] := 0;
  end;
end;

procedure vector_times_matrix(const v: TVector; const m: TMatrix;
                              var res: TVector);
var
  i, j: Integer;
begin
  for j := 0 to 2 do
  begin
    res[j] := 0;
    for i := 0 to 2 do
      res[j] := res[j] + (m[j, i] * v[i]);
  end;
end;

procedure matrix_idle(var m: TMatrix);
begin
=======
=======
>>>>>>> origin/fixes_2_2
  TRayTunnel = Class(TObject)
  Private
    tunneltex : PUint8;                      { Texture }
    pal : PUint8;                            { Original palette }
    lookup : PUint32;                         { Lookup table for lighting }

    sintab, costab : PSingle;                { Take a guess }

    u_array, v_array, l_array : PInteger;    { Raytraced coordinates and light }
    norms : PVector;

    radius, radius_sqr : Single;
    rot : TMatrix;

    pos, light : TVector;                    { Position in the tunnel, pos of }
    xa, ya, za : Integer;                    { lightsource, angles }

    lightstatus : Boolean;                   { Following the viewer ? }

  Public
    Constructor Create(rad : Single);        { Constructor takes the radius }
    Destructor Destroy; Override;

    Procedure load_texture;

    Procedure tilt(x, y, z : Integer);              { Rotate relative }
    Procedure tilt(x, y, z : Integer; abs : Uint8); { Absolute }

    Procedure move(dx, dy, dz : Single);            { Relative move }
    Procedure move(x, y, z : Single; abs : Uint8);  { Absolute }

    Procedure movelight(dx, dy, dz : Single);
    Procedure movelight(x, y, z : Single; abs : Uint8);

    Procedure locklight(lock : Boolean);    { Make the light follow the viewer }

    Procedure interpolate;                  { Raytracing }

    Procedure draw(dest : PUint32);          { Draw the finished tunnel }
  End;

{ VECTOR ROUTINES }
Procedure vector_normalize(Var v : TVector);

Var
  length : Single;

Begin
  length := v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  length := sqrt(length);
  If length <> 0 Then
  Begin
    v[0] := v[0] / length;
    v[1] := v[1] / length;
    v[2] := v[2] / length;
  End
  Else
  Begin
    v[0] := 0;
    v[1] := 0;
    v[2] := 0;
  End;
End;

Procedure vector_times_matrix(Const v : TVector; Const m : TMatrix;
                              Var res : TVector);

Var
  i, j : Integer;

Begin
  For j := 0 To 2 Do
  Begin
    res[j] := 0;
    For i := 0 To 2 Do
      res[j] := res[j] + (m[j, i] * v[i]);
  End;
End;

Procedure matrix_idle(Var m : TMatrix);

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FillChar(m, SizeOf(TMatrix), 0);
  m[0, 0] := 1;
  m[1, 1] := 1;
  m[2, 2] := 1;
  m[3, 3] := 1;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure matrix_times_matrix(const m1, m2: TMatrix; var res: TMatrix);
var
  i, j, k: Integer;
begin
  for j := 0 to 3 do
    for i := 0 to 3 do
    begin
      res[i, j] := 0;
      for k := 0 to 3 do
        res[i, j] := res[i, j] + (m1[k, j] * m2[i, k]);
    end;
end;

procedure matrix_rotate_x(var m: TMatrix; angle: Integer; sintab, costab: PSingle);
var
  tmp, tmp2: TMatrix;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure matrix_times_matrix(Const m1, m2 : TMatrix; Var res : TMatrix);

Var
  i, j, k : Integer;

Begin
  For j := 0 To 3 Do
    For i := 0 To 3 Do
    Begin
      res[i, j] := 0;
      For k := 0 To 3 Do
        res[i, j] := res[i, j] + (m1[k, j] * m2[i, k]);
    End;
End;

Procedure matrix_rotate_x(Var m : TMatrix; angle : Integer; sintab, costab : PSingle);

Var
  tmp, tmp2 : TMatrix;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  matrix_idle(tmp);
  tmp[1, 1] := costab[angle];
  tmp[2, 1] := sintab[angle];
  tmp[1, 2] := -sintab[angle];
  tmp[2, 2] := costab[angle];
  matrix_times_matrix(tmp, m, tmp2);
  Move(tmp2, m, SizeOf(TMatrix));
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure matrix_rotate_y(var m: TMatrix; angle: Integer; sintab, costab: PSingle);
var
  tmp, tmp2: TMatrix;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure matrix_rotate_y(Var m : TMatrix; angle : Integer; sintab, costab : PSingle);

Var
  tmp, tmp2 : TMatrix;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  matrix_idle(tmp);
  tmp[0, 0] := costab[angle];
  tmp[2, 0] := -sintab[angle];
  tmp[0, 2] := sintab[angle];
  tmp[2, 2] := costab[angle];
  matrix_times_matrix(tmp, m, tmp2);
  Move(tmp2, m, SizeOf(TMatrix));
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure matrix_rotate_z(var m: TMatrix; angle: Integer; sintab, costab: PSingle);
var
  tmp, tmp2: TMatrix;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure matrix_rotate_z(Var m : TMatrix; angle : Integer; sintab, costab : PSingle);

Var
  tmp, tmp2 : TMatrix;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  matrix_idle(tmp);
  tmp[0, 0] := costab[angle];
  tmp[1, 0] := sintab[angle];
  tmp[0, 1] := -sintab[angle];
  tmp[1, 1] := costab[angle];
  matrix_times_matrix(tmp, m, tmp2);
  Move(tmp2, m, SizeOf(TMatrix));
<<<<<<< HEAD
<<<<<<< HEAD
end;

constructor TRayTunnel.Create(rad: Single);
var
  x, y: Single;
  i, j: Integer;
  tmp: TVector;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Constructor TRayTunnel.Create(rad : Single);

Var
  x, y : Single;
  i, j : Integer;
  tmp : TVector;

Begin
  tunneltex := Nil;
  sintab := Nil;
  costab := Nil;
  u_array := Nil;
  v_array := Nil;
  norms := Nil;
  lookup := Nil;
  pal := Nil;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  radius := rad;
  radius_sqr := rad * rad;

  sintab := GetMem(1024 * SizeOf(Single)); { Set trigonometry and lookups }
  costab := GetMem(1024 * SizeOf(Single));
  u_array := GetMem(64 * 26 * SizeOf(Integer));
  v_array := GetMem(64 * 26 * SizeOf(Integer));
  l_array := GetMem(64 * 26 * SizeOf(Integer));
  norms := GetMem(64 * 26 * 3 * SizeOf(Single));

  lookup := GetMem(65 * 256 * SizeOf(Uint32));
  pal := GetMem(768 * SizeOf(Uint8));

<<<<<<< HEAD
<<<<<<< HEAD
  for i := 0 to 1023 do
  begin
    sintab[i] := sin(i * pi / 512);
    costab[i] := cos(i * pi / 512);
  end;

  { Generate normal vectors }
  y := -100;
  for j := 0 to 25 do
  begin
    x := -160;
    for i := 0 to 40 do
    begin
=======
=======
>>>>>>> origin/fixes_2_2
  For i := 0 To 1023 Do
  Begin
    sintab[i] := sin(i * pi / 512);
    costab[i] := cos(i * pi / 512);
  End;

  { Generate normal vectors }
  y := -100;
  For j := 0 To 25 Do
  Begin
    x := -160;
    For i := 0 To 40 Do
    Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      tmp[0] := x;
      tmp[1] := y;
      tmp[2] := 128;
      vector_normalize(tmp);
      norms[j * 64 + i] := tmp;
      x := x + 8;
<<<<<<< HEAD
<<<<<<< HEAD
    end;
    y := y + 8;
  end;
=======
    End;
    y := y + 8;
  End;
>>>>>>> graemeg/fixes_2_2
=======
    End;
    y := y + 8;
  End;
>>>>>>> origin/fixes_2_2

  { Reset tunnel and light position and all angles }
  pos[0] := 0; pos[1] := 0; pos[2] := 0;
  light[0] := 1; light[1] := 1; light[2] := 0;

  xa := 0; ya := 0; za := 0;

  lightstatus := False;

  { Normalize light vector to length 1.0 }
  vector_normalize(light);
<<<<<<< HEAD
<<<<<<< HEAD
end;

destructor TRayTunnel.Destroy;
begin
  FreeMem(tunneltex_orig);
  FreeMem(pal);
  FreeMem(lookup);
  FreeMem(norms);
  FreeMem(l_array);
  FreeMem(v_array);
  FreeMem(u_array);
  FreeMem(costab);
  FreeMem(sintab);
end;

procedure TRayTunnel.load_texture;
var
  texfile: File;
  tmp: PUint8 = nil;
  i, j: Uint32;
  r, g, b: Uint32;
  newoffs: Integer;
begin
  try
    { Allocate tunnel texture 65536+33 bytes too big }

    if tunneltex_orig <> nil then
    begin
      FreeMem(tunneltex_orig);
      tunneltex_orig := nil;
    end;
    tunneltex_orig := GetMem(2*65536 + 33);
    tmp := GetMem(65536);

    { Align the texture on a 64k boundary }
    tunneltex := tunneltex_orig;
    while (PtrUInt(tunneltex) and $FFFF) <> 0 do
      Inc(tunneltex);

    AssignFile(texfile, 'tunnel3d.raw');
    Reset(texfile, 1);
    try
      BlockRead(texfile, pal^, 768);
      BlockRead(texfile, tmp^, 65536);
    finally
      CloseFile(texfile);
    end;

    { Generate lookup table for lighting (65 because of possible inaccuracies) }

    for j := 0 to 64 do
      for i := 0 to 255 do
      begin
        r := pal[i * 3] shl 2;
        g := pal[i * 3 + 1] shl 2;
        b := pal[i * 3 + 2] shl 2;
        r := (r * j) shr 6;
        g := (g * j) shr 6;
        b := (b * j) shr 6;
        if r > 255 then
          r := 255;
        if g > 255 then
          g := 255;
        if b > 255 then
          b := 255;
        lookup[j * 256 + i] := (r shl 16) or (g shl 8) or b;
      end;

    { Arrange texture for cache optimised mapping }

    for j := 0 to 255 do
      for i := 0 to 255 do
      begin
        newoffs := ((i shl 8) and $F800) + (i and $0007) + ((j shl 3) and $7F8);
        (tunneltex + newoffs)^ := (tmp + j * 256 + i)^;
      end;
  finally
    FreeMem(tmp);
  end;
end;

procedure TRayTunnel.interpolate;
var
  ray, intsc, norm, lvec: TVector;
  x, y, a, b, c, discr, t, res: Single;
  i, j: Integer;
begin
  if lightstatus then { Lightsource locked to viewpoint }
    light := pos;

  matrix_idle(rot);
  matrix_rotate_x(rot, xa and $3FF, sintab, costab);
  matrix_rotate_y(rot, ya and $3FF, sintab, costab);
  matrix_rotate_z(rot, za and $3FF, sintab, costab);
=======
=======
>>>>>>> origin/fixes_2_2
End;

Destructor TRayTunnel.Destroy;

Begin
  If Assigned(tunneltex) Then
    FreeMem(tunneltex);
  If Assigned(pal) Then
    FreeMem(pal);
  If Assigned(lookup) Then
    FreeMem(lookup);
  If Assigned(norms) Then
    FreeMem(norms);
  If Assigned(l_array) Then
    FreeMem(l_array);
  If Assigned(v_array) Then
    FreeMem(v_array);
  If Assigned(u_array) Then
    FreeMem(u_array);
  If Assigned(costab) Then
    FreeMem(costab);
  If Assigned(sintab) Then
    FreeMem(sintab);
End;

Procedure TRayTunnel.load_texture;

Var
  texfile : File;
  tmp : PUint8;
  i, j : Uint32;
  r, g, b : Uint32;
  newoffs : Integer;

Begin
  { Allocate tunnel texture 65536+33 bytes too big }

  If tunneltex <> Nil Then
  Begin
    FreeMem(tunneltex);
    tunneltex := Nil;
  End;
  tunneltex := GetMem(2*65536 + 33);
  tmp := GetMem(65536);

  { Align the texture on a 64k boundary }
  While (PtrUInt(tunneltex) And $FFFF) <> 0 Do
    Inc(tunneltex);

  ASSign(texfile, 'tunnel3d.raw');
  Reset(texfile, 1);
  BlockRead(texfile, pal^, 768);
  BlockRead(texfile, tmp^, 65536);
  Close(texfile);

  { Generate lookup table for lighting (65 because of possible inaccuracies) }

  For j := 0 To 64 Do
    For i := 0 To 255 Do
    Begin
      r := pal[i * 3] Shl 2;
      g := pal[i * 3 + 1] Shl 2;
      b := pal[i * 3 + 2] Shl 2;
      r := (r * j) Shr 6;
      g := (g * j) Shr 6;
      b := (b * j) Shr 6;
      If r > 255 Then
        r := 255;
      If g > 255 Then
        g := 255;
      If b > 255 Then
        b := 255;
      lookup[j * 256 + i] := (r Shl 16) Or (g Shl 8) Or b;
    End;

  { Arrange texture for cache optimised mapping }

  For j := 0 To 255 Do
    For i := 0 To 255 Do
    Begin
      newoffs := ((i Shl 8) And $F800) + (i And $0007) + ((j Shl 3) And $7F8);
      (tunneltex + newoffs)^ := (tmp + j * 256 + i)^;
    End;

  FreeMem(tmp);
End;

Procedure TRayTunnel.interpolate;

Var
  ray, intsc, norm, lvec : TVector;
  x, y, a, b, c, discr, t, res : Single;
  i, j : Integer;

Begin
  If lightstatus Then { Lightsource locked to viewpoint }
    light := pos;

  matrix_idle(rot);
  matrix_rotate_x(rot, xa And $3FF, sintab, costab);
  matrix_rotate_y(rot, ya And $3FF, sintab, costab);
  matrix_rotate_z(rot, za And $3FF, sintab, costab);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { Constant factor }
  c := 2 * (pos[0] * pos[0] + pos[1] * pos[1] - radius_sqr);

  { Start raytracing }
  y := -100;
<<<<<<< HEAD
<<<<<<< HEAD
  for j := 0 to 25 do
  begin
    x := -160;
    for i := 0 to 40 do
    begin
      vector_times_matrix(norms[(j shl 6) + i], rot, ray);
=======
=======
>>>>>>> origin/fixes_2_2
  For j := 0 To 25 Do
  Begin
    x := -160;
    For i := 0 To 40 Do
    Begin
      vector_times_matrix(norms[(j Shl 6) + i], rot, ray);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      a := 2 * (ray[0] * ray[0] + ray[1] * ray[1]);
      b := 2 * (pos[0] * ray[0] + pos[1] * ray[1]);

      discr := b * b - a * c;
<<<<<<< HEAD
<<<<<<< HEAD
      if discr > 0 then
      begin
=======
      If discr > 0 Then
      Begin
>>>>>>> graemeg/fixes_2_2
=======
      If discr > 0 Then
      Begin
>>>>>>> origin/fixes_2_2
        discr := sqrt(discr);
        t := (- b + discr) / a;

        { Calculate intersection point }
        intsc[0] := pos[0] + t * ray[0];
        intsc[1] := pos[1] + t * ray[1];
        intsc[2] := pos[2] + t * ray[2];

        { Calculate texture index at intersection point (cylindrical mapping) }
<<<<<<< HEAD
<<<<<<< HEAD
        { try and adjust the 0.2 to stretch/shrink the texture }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        u_array[(j shl 6) + i] := Integer(Trunc(intsc[2] * 0.2) shl 16);
=======
        u_array[(j shl 6) + i] := Trunc(intsc[2] * 0.2) shl 16;
>>>>>>> graemeg/cpstrnew
=======
        u_array[(j shl 6) + i] := Trunc(intsc[2] * 0.2) shl 16;
>>>>>>> graemeg/cpstrnew
=======
        u_array[(j shl 6) + i] := Trunc(intsc[2] * 0.2) shl 16;
>>>>>>> graemeg/cpstrnew
=======
        u_array[(j shl 6) + i] := Trunc(intsc[2] * 0.2) shl 16;
>>>>>>> origin/cpstrnew
        v_array[(j shl 6) + i] := Trunc(abs(arctan2(intsc[1], intsc[0]) * 256 / pi)) shl 16;
=======
        { Try and adjust the 0.2 to stretch/shrink the texture }
        u_array[(j Shl 6) + i] := Trunc(intsc[2] * 0.2) Shl 16;
        v_array[(j Shl 6) + i] := Trunc(abs(arctan2(intsc[1], intsc[0]) * 256 / pi)) Shl 16;
>>>>>>> graemeg/fixes_2_2
=======
        { Try and adjust the 0.2 to stretch/shrink the texture }
        u_array[(j Shl 6) + i] := Trunc(intsc[2] * 0.2) Shl 16;
        v_array[(j Shl 6) + i] := Trunc(abs(arctan2(intsc[1], intsc[0]) * 256 / pi)) Shl 16;
>>>>>>> origin/fixes_2_2

        { Calculate the dotproduct between the normal vector and the vector }
        { from the intersection point to the lightsource }
        norm[0] := intsc[0] / radius;
        norm[1] := intsc[1] / radius;
        norm[2] := 0;

        lvec[0] := intsc[0] - light[0];
        lvec[1] := intsc[1] - light[1];
        lvec[2] := intsc[2] - light[2];
        vector_normalize(lvec);

        res := lvec[0] * norm[0] + lvec[1] * norm[1] + lvec[2] * norm[2];

        { Scale the light a bit }
<<<<<<< HEAD
<<<<<<< HEAD
        res := Sqr(res);
        if res < 0 then
          res := 0;
        if res > 1 then
          res := 1;
        res := res * 63;

        { Put it into the light array }
        l_array[(j shl 6) + i] := Trunc(res) shl 16;
      end
      else
      begin
        u_array[(j shl 6) + i] := 0;
        v_array[(j shl 6) + i] := 0;
        l_array[(j shl 6) + i] := 0;
      end;
      x := x + 8;
    end;
    y := y + 8;
  end;
end;

procedure TRayTunnel.draw(dest: PUint32);
var
  x, y, lu, lv, ru, rv, liu, liv, riu, riv: Integer;
  iu, iv, i, j, ll, rl, lil, ril, l, il: Integer;
  iadr, adr, til_u, til_v, til_iu, til_iv: DWord;
  bla: Uint8;
begin
  for j := 0 to 24 do
    for i := 0 to 39 do
    begin
      iadr := (j shl 6) + i;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

      { Set up gradients }
      lu := u_array[iadr]; ru := u_array[iadr + 1];
<<<<<<< HEAD
      liu := (u_array[iadr + 64] - lu) div 8;
      riu := (u_array[iadr + 65] - ru) div 8;

      lv := v_array[iadr]; rv := v_array[iadr + 1];
      liv := (v_array[iadr + 64] - lv) div 8;
      riv := (v_array[iadr + 65] - rv) div 8;

      ll := l_array[iadr]; rl := l_array[iadr + 1];
      lil := (l_array[iadr + 64] - ll) div 8;
      ril := (l_array[iadr + 65] - rl) div 8;

      for y := 0 to 7 do
      begin
        iu := (ru - lu) div 8;
        iv := (rv - lv) div 8;
        l := ll;
        il := (rl - ll) div 8;
=======
=======

      { Set up gradients }
      lu := u_array[iadr]; ru := u_array[iadr + 1];
>>>>>>> graemeg/cpstrnew
=======

      { Set up gradients }
      lu := u_array[iadr]; ru := u_array[iadr + 1];
>>>>>>> graemeg/cpstrnew
=======

      { Set up gradients }
      lu := u_array[iadr]; ru := u_array[iadr + 1];
>>>>>>> origin/cpstrnew
      liu := (u_array[iadr + 64] - lu) shr 3;
      riu := (u_array[iadr + 65] - ru) shr 3;

      lv := v_array[iadr]; rv := v_array[iadr + 1];
      liv := (v_array[iadr + 64] - lv) shr 3;
      riv := (v_array[iadr + 65] - rv) shr 3;

      ll := l_array[iadr]; rl := l_array[iadr + 1];
      lil := (l_array[iadr + 64] - ll) shr 3;
      ril := (l_array[iadr + 65] - rl) shr 3;

      for y := 0 to 7 do
      begin
        iu := (ru - lu) shr 3;
        iv := (rv - lv) shr 3;
        l := ll;
        il := (rl - ll) shr 3;
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

        { Mess up everything for the sake of cache optimised mapping :) }
        til_u := DWord(((lu shl 8) and $F8000000) or ((lu shr 1) and $00007FFF) or (lu and $00070000));
        til_v := DWord(((lv shl 3) and $07F80000) or ((lv shr 1) and $00007FFF));
        til_iu := DWord((((iu shl 8) and $F8000000) or ((iu shr 1) and $00007FFF) or
                          (iu and $00070000)) or $07F88000);
        til_iv := DWord((((iv shl 3) and $07F80000) or ((iv shr 1) and $00007FFF)) or $F8078000);

        adr := til_u + til_v;

        for x := 0 to 7 do
        begin
          { Interpolate texture u,v and light }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          til_u := DWord(til_u + til_iu);
          til_v := DWord(til_v + til_iv);
=======
          Inc(til_u, til_iu);
          Inc(til_v, til_iv);
>>>>>>> graemeg/cpstrnew
=======
          Inc(til_u, til_iu);
          Inc(til_v, til_iv);
>>>>>>> graemeg/cpstrnew
=======
          Inc(til_u, til_iu);
          Inc(til_v, til_iv);
>>>>>>> graemeg/cpstrnew
=======
          Inc(til_u, til_iu);
          Inc(til_v, til_iv);
>>>>>>> origin/cpstrnew
          Inc(l, il);

          adr := adr shr 16;

          til_u := til_u and DWord($F8077FFF);
          til_v := til_v and $07F87FFF;
=======
=======
>>>>>>> origin/fixes_2_2
        res *= res;
        If res < 0 Then
          res := 0;
        If res > 1 Then
          res := 1;
        res *= 63;

        { Put it into the light array }
        l_array[(j Shl 6) + i] := Trunc(res) Shl 16;
      End
      Else
      Begin
        u_array[(j Shl 6) + i] := 0;
        v_array[(j Shl 6) + i] := 0;
        l_array[(j Shl 6) + i] := 0;
      End;
      x := x + 8;
    End;
    y := y + 8;
  End;
End;

Procedure TRayTunnel.draw(dest : PUint32);

Var
  x, y, lu, lv, ru, rv, liu, liv, riu, riv : Integer;
  iu, iv, i, j, ll, rl, lil, ril, l, il : Integer;
  iadr, adr, til_u, til_v, til_iu, til_iv : DWord;
  bla : Uint8;

Begin
  For j := 0 To 24 Do
    For i := 0 To 39 Do
    Begin
      iadr := (j Shl 6) + i;

      { Set up gradients }
      lu := u_array[iadr]; ru := u_array[iadr + 1];
      liu := (u_array[iadr + 64] - lu) Shr 3;
      riu := (u_array[iadr + 65] - ru) Shr 3;

      lv := v_array[iadr]; rv := v_array[iadr + 1];
      liv := (v_array[iadr + 64] - lv) Shr 3;
      riv := (v_array[iadr + 65] - rv) Shr 3;

      ll := l_array[iadr]; rl := l_array[iadr + 1];
      lil := (l_array[iadr + 64] - ll) Shr 3;
      ril := (l_array[iadr + 65] - rl) Shr 3;

      For y := 0 To 7 Do
      Begin
        iu := (ru - lu) Shr 3;
        iv := (rv - lv) Shr 3;
        l := ll;
        il := (rl - ll) Shr 3;

        { Mess up everything for the sake of cache optimised mapping :) }
        til_u := DWord(((lu Shl 8) And $F8000000) Or ((lu Shr 1) And $00007FFF) Or (lu And $00070000));
        til_v := DWord(((lv Shl 3) And $07F80000) Or ((lv Shr 1) And $00007FFF));
        til_iu := DWord((((iu Shl 8) And $F8000000) Or ((iu Shr 1) And $00007FFF) Or
                          (iu And $00070000)) Or $07F88000);
        til_iv := DWord((((iv Shl 3) And $07F80000) Or ((iv Shr 1) And $00007FFF)) Or $F8078000);

        adr := til_u + til_v;

        For x := 0 To 7 Do
        Begin
          { Interpolate texture u,v and light }
	  Inc(til_u, til_iu);
	  Inc(til_v, til_iv);
          Inc(l, il);

          adr := adr Shr 16;

          til_u := til_u And DWord($F8077FFF);
          til_v := til_v And $07F87FFF;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

          bla := (tunneltex + adr)^;

          adr := til_u + til_v;

          { Look up the light and write to buffer }
<<<<<<< HEAD
<<<<<<< HEAD
          (dest + ((j shl 3) + y) * 320 + (I shl 3) + x)^ := lookup[((l and $3F0000) shr 8) + bla];
        end;
=======
          (dest + ((j Shl 3) + y) * 320 + (I Shl 3) + x)^ := lookup[((l And $3F0000) Shr 8) + bla];
        End;
>>>>>>> graemeg/fixes_2_2
=======
          (dest + ((j Shl 3) + y) * 320 + (I Shl 3) + x)^ := lookup[((l And $3F0000) Shr 8) + bla];
        End;
>>>>>>> origin/fixes_2_2

        Inc(lu, liu); Inc(ru, riu);
        Inc(lv, liv); Inc(rv, riv);
        Inc(ll, lil); Inc(rl, ril);
<<<<<<< HEAD
<<<<<<< HEAD
      end;
    end;
end;

{ tilt rotates the viewer in the tunnel in a relative / absolute way }
procedure TRayTunnel.tilt(x, y, z: Integer);
begin
  xa := (xa + x) and $3FF;
  ya := (ya + y) and $3FF;
  za := (za + z) and $3FF;
end;

procedure TRayTunnel.tilt(x, y, z: Integer; abs: Uint8);
begin
  xa := x and $3FF;
  ya := y and $3FF;
  za := z and $3FF;
end;

{ Relative / absolute move }
procedure TRayTunnel.move(dx, dy, dz: Single);
begin
  pos[0] := pos[0] + dx;
  pos[1] := pos[1] + dy;
  pos[2] := pos[2] + dz;
end;

procedure TRayTunnel.move(x, y, z: Single; abs: Uint8);
begin
  pos[0] := x;
  pos[1] := y;
  pos[2] := z;
end;

{ Relative / absolute move for the lightsource }
procedure TRayTunnel.movelight(dx, dy, dz: Single);
begin
  light[0] := light[0] + dx;
  light[1] := light[1] + dy;
  light[2] := light[2] + dz;
end;

procedure TRayTunnel.movelight(x, y, z: Single; abs: Uint8);
begin
  light[0] := x;
  light[1] := y;
  light[2] := z;
end;

{ Lock lightsource to the viewer }
procedure TRayTunnel.locklight(lock: Boolean);
begin
  lightstatus := lock;
end;

var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  console: IPTCConsole;
  surface: IPTCSurface;
  format: IPTCFormat;
=======
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> graemeg/cpstrnew
=======
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> graemeg/cpstrnew
=======
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> graemeg/cpstrnew
=======
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
  format: TPTCFormat = nil;
>>>>>>> origin/cpstrnew
  tunnel: TRayTunnel = nil;
  posz, phase_x, phase_y: Single;
  angle_x, angle_y: Integer;
  buffer: PUint32;
begin
  try
    try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      format := TPTCFormatFactory.CreateNew(32, $00FF0000, $0000FF00, $000000FF);

      console := TPTCConsoleFactory.CreateNew;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      console := TPTCConsole.create;
>>>>>>> graemeg/cpstrnew
      console.open('Tunnel3D demo', 320, 200, format);

      surface := TPTCSurfaceFactory.CreateNew(320, 200, format);
=======
=======
>>>>>>> origin/fixes_2_2
      End;
    End;
End;

{ tilt rotates the viewer in the tunnel in a relative / absolute way }
Procedure TRayTunnel.tilt(x, y, z : Integer);

Begin
  xa := (xa + x) And $3FF;
  ya := (ya + y) And $3FF;
  za := (za + z) And $3FF;
End;

Procedure TRayTunnel.tilt(x, y, z : Integer; abs : Uint8);

Begin
  xa := x And $3FF;
  ya := y And $3FF;
  za := z And $3FF;
End;

{ Relative / absolute move }
Procedure TRayTunnel.move(dx, dy, dz : Single);

Begin
  pos[0] := pos[0] + dx;
  pos[1] := pos[1] + dy;
  pos[2] := pos[2] + dz;
End;

Procedure TRayTunnel.move(x, y, z : Single; abs : Uint8);

Begin
  pos[0] := x;
  pos[1] := y;
  pos[2] := z;
End;

{ Relative / absolute move for the lightsource }
Procedure TRayTunnel.movelight(dx, dy, dz : Single);

Begin
  light[0] := light[0] + dx;
  light[1] := light[1] + dy;
  light[2] := light[2] + dz;
End;

Procedure TRayTunnel.movelight(x, y, z : Single; abs : Uint8);

Begin
  light[0] := x;
  light[1] := y;
  light[2] := z;
End;

{ Lock lightsource to the viewer }
Procedure TRayTunnel.locklight(lock : Boolean);

Begin
  lightstatus := lock;
End;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;
  tunnel : TRayTunnel;
  posz, phase_x, phase_y : Single;
  angle_x, angle_y : Integer;
  buffer : PUint32;

Begin
  format := Nil;
  surface := Nil;
  console := Nil;
  tunnel := Nil;
  Try
    Try
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      console := TPTCConsole.create;
      console.open('Tunnel3D demo', 320, 200, format);

      surface := TPTCSurface.create(320, 200, format);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      { Create a tunnel, radius=700 }
      tunnel := TRayTunnel.Create(700);

      tunnel.load_texture;

      { Light follows the viewer }
      tunnel.locklight(True);

      posz := 80; phase_x := 0; phase_y := 0;
      angle_x := 6; angle_y := 2;

<<<<<<< HEAD
<<<<<<< HEAD
      while not console.KeyPressed do
      begin
        buffer := surface.lock;
        try
=======
=======
>>>>>>> origin/fixes_2_2
      While Not console.KeyPressed Do
      Begin
        buffer := surface.lock;
	Try
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          tunnel.interpolate;

          { Draw to offscreen buffer }
          tunnel.draw(buffer);
<<<<<<< HEAD
<<<<<<< HEAD
        finally
          surface.unlock;
        end;

        { and copy to screen }
=======
=======
>>>>>>> origin/fixes_2_2
	Finally
          surface.unlock;
	End;

        { And copy to screen }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        surface.copy(console);

        console.update;

        tunnel.tilt(angle_x, angle_y, 0);
        tunnel.move(sin(phase_x), cos(phase_y), posz);

        phase_x := phase_x + 0.2;
        phase_y := phase_y + 0.1;
<<<<<<< HEAD
<<<<<<< HEAD
      end;
    finally
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if Assigned(console) then
        console.close;
      tunnel.Free;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
      End;
    Finally
>>>>>>> graemeg/fixes_2_2
=======
      End;
    Finally
>>>>>>> origin/fixes_2_2
      console.close;
      console.Free;
      surface.Free;
      tunnel.Free;
      format.Free;
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
      error.report;
  end;
end.
=======
=======
>>>>>>> origin/fixes_2_2
    End;
  Except
    On error : TPTCError Do
      error.report;
  End;
End.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
