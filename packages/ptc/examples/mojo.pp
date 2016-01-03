{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
Mojo demo for OpenPTC 1.0 C++ API
Coded by Alex Evans and adapted to OpenPTC 1.0 by Glenn Fiedler

nasty code by alex "statix" evans for ptc. (c) copyright alex evans 1998
time... 02.00 am on 13/1/98.
have fun
it's my take on some classic light mask effect
it's raytracing through properly modelled fog with occlusion, multiple
shadow rays cast back to the light for each pixel ray, and erm, its
s l o w... but it looks nice don't it?

oh and fresnel fall off... or something

UNTESTED! ok?

define inv for interesting fx (not)
}

<<<<<<< HEAD
<<<<<<< HEAD
program Mojo;

{$MODE objfpc}
{$INLINE on}

uses
=======
=======
>>>>>>> origin/fixes_2_2
Program Mojo;

{$MODE objfpc}

Uses
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ptc, SysUtils;

{ $DEFINE INV}

<<<<<<< HEAD
<<<<<<< HEAD
const
  SC = 12;
  MINSEGSIZE = 2.5;
  NSEG = 5;
  frandtab_seed: Uint16 = 54;

var
  MaskMap: PUint8;
  frandtab: array [0..65535] of Uint16;

type
  FVector = object
{    case Boolean of
      False: (X, Y, Z: Single);
      True: (R, G, B: Single);}
    X, Y, Z: Single;

    constructor Init;
    constructor Init(_x, _y, _z: Single);

    function Magnitude: Single; inline;
    function MagnitudeSq: Single; inline;
    procedure Normalise; inline;
  end;
  FMatrix = object
    Row: array [0..2] of FVector;
    constructor Init;
    constructor Init(a, b, c: FVector);
    function Column0: FVector; inline;
    function Column1: FVector; inline;
    function Column2: FVector; inline;
    procedure MakeXRot(theta: Single); inline;
    procedure MakeYRot(theta: Single); inline;
    procedure MakeZRot(theta: Single); inline;
    procedure MakeID; inline;
    function Transpose: FMatrix; inline;
    procedure TransposeInPlace; inline;
    procedure Normalise; inline;
  end;
  PRay = ^TRay;
  TRay = Object
    mPosn: FVector;
    mDir: FVector;
    constructor Init(const p, d: FVector);
  end;
  VLight = class
    mAng: Single;
    mPosn: FVector;
    mTarget: FVector;
    mAxis: FMatrix;
    mCol: FVector;

    p, p2, _d: FVector; { temp space }

    constructor Create(const col: FVector);
    procedure Move(const q: FVector);
    procedure MoveT(const q: FVector);
    procedure Update;
    function Light(const ray: TRay): FVector;
    function CalcLight(t: Single): Single;
  end;

constructor FVector.Init;
begin
end;

constructor FVector.Init(_x, _y, _z: Single);
begin
  X := _x;
  Y := _y;
  Z := _z;
end;

function FVector.Magnitude: Single; inline;
begin
  Result := Sqrt(Sqr(X) + Sqr(Y) + Sqr(Z));
end;

function FVector.MagnitudeSq: Single; inline;
begin
  Result := Sqr(X) + Sqr(Y) + Sqr(Z);
end;

procedure FVector.Normalise; inline;
var
  l: Single;
begin
  l := 1 / Magnitude;
  X := X * l;
  Y := Y * l;
  Z := Z * l;
end;

operator * (a, b: FVector): Single; inline;
begin
  Result := a.X * b.X + a.Y * b.Y + a.Z * b.Z;
end;

operator * (a: FVector; b: Single): FVector; inline;
begin
  Result.X := a.X * b;
  Result.Y := a.Y * b;
  Result.Z := a.Z * b;
end;

operator + (a, b: FVector): FVector; inline;
begin
  Result.X := a.X + b.X;
  Result.Y := a.Y + b.Y;
  Result.Z := a.Z + b.Z;
end;

operator - (a, b: FVector): FVector; inline;
begin
  Result.X := a.X - b.X;
  Result.Y := a.Y - b.Y;
  Result.Z := a.Z - b.Z;
end;

operator ** (a, b: FVector) res: FVector; inline;
begin
  Result.X := a.Y * b.Z - a.Z * b.Y;
  Result.Y := a.Z * b.X - a.X * b.Z;
  Result.Z := a.X * b.Y - a.Y * b.X;
end;

constructor FMatrix.Init;
begin
end;

constructor FMatrix.Init(a, b, c: FVector);
begin
  Row[0] := a;
  Row[1] := b;
  Row[2] := c;
end;

function FMatrix.Column0: FVector; inline;
begin
  Result.Init(Row[0].X, Row[1].X, Row[2].X);
end;

function FMatrix.Column1: FVector; inline;
begin
  Result.Init(Row[0].Y, Row[1].Y, Row[2].Y);
end;

function FMatrix.Column2: FVector; inline;
begin
  Result.Init(Row[0].Z, Row[1].Z, Row[2].Z);
end;

procedure FMatrix.MakeXRot(theta: Single); inline;
var
  c, s: Single;
begin
=======
=======
>>>>>>> origin/fixes_2_2
Const
  SC = 12;
  MINSEGSIZE = 2.5;
  NSEG = 5;
  frandtab_seed : Uint16 = 54;

Var
  MaskMap : PUint8;
  frandtab : Array[0..65535] Of Uint16;

Type
  FVector = Object
{    Case Boolean Of
      False : (X, Y, Z : Single);
      True : (R, G, B : Single);}
    X, Y, Z : Single;

    Constructor Init;
    Constructor Init(_x, _y, _z : Single);

    Function Magnitude : Single;
    Function MagnitudeSq : Single;
    Procedure Normalise;
  End;
  FMatrix = Object
    Row : Array[0..2] Of FVector;
    Constructor Init;
    Constructor Init(a, b, c : FVector);
    Function Column0 : FVector;
    Function Column1 : FVector;
    Function Column2 : FVector;
    Procedure MakeXRot(theta : Single);
    Procedure MakeYRot(theta : Single);
    Procedure MakeZRot(theta : Single);
    Procedure MakeID;
    Function Transpose : FMatrix;
    Procedure TransposeInPlace;
    Procedure Normalise;
  End;
  PRay = ^TRay;
  TRay = Object
    mPosn : FVector;
    mDir : FVector;
    Constructor Init(Const p, d : FVector);
  End;
  VLight = Class(TObject)
    mAng : Single;
    mPosn : FVector;
    mTarget : FVector;
    mAxis : FMatrix;
    mCol : FVector;

    p, p2, _d : FVector; { temp space }

    Constructor Create(Const col : FVector);
    Procedure Move(Const q : FVector);
    Procedure MoveT(Const q : FVector);
    Procedure Update;
    Function Light(Const ray : TRay) : FVector;
    Function CalcLight(t : Single) : Single;
  End;

Constructor FVector.Init;

Begin
End;

Constructor FVector.Init(_x, _y, _z : Single);

Begin
  X := _x;
  Y := _y;
  Z := _z;
End;

Function FVector.Magnitude : Single;

Begin
  Magnitude := Sqrt(Sqr(X) + Sqr(Y) + Sqr(Z));
End;

Function FVector.MagnitudeSq : Single;

Begin
  MagnitudeSq := Sqr(X) + Sqr(Y) + Sqr(Z);
End;

Procedure FVector.Normalise;

Var
  l : Single;

Begin
  l := 1 / Magnitude;
  X *= l;
  Y *= l;
  Z *= l;
End;

Operator * (a, b : FVector) res : Single;

Begin
  res := a.X * b.X + a.Y * b.Y + a.Z * b.Z;
End;

Operator * (a : FVector; b : Single) res : FVector;

Begin
  res.X := a.X * b;
  res.Y := a.Y * b;
  res.Z := a.Z * b;
End;

Operator + (a, b : FVector) res : FVector;

Begin
  res.X := a.X + b.X;
  res.Y := a.Y + b.Y;
  res.Z := a.Z + b.Z;
End;

Operator - (a, b : FVector) res : FVector;

Begin
  res.X := a.X - b.X;
  res.Y := a.Y - b.Y;
  res.Z := a.Z - b.Z;
End;

Operator ** (a, b : FVector) res : FVector;

Begin
  res.X := a.Y * b.Z - a.Z * b.Y;
  res.Y := a.Z * b.X - a.X * b.Z;
  res.Z := a.X * b.Y - a.Y * b.X;
End;

Constructor FMatrix.Init;

Begin
End;

Constructor FMatrix.Init(a, b, c : FVector);

Begin
  Row[0] := a;
  Row[1] := b;
  Row[2] := c;
End;

Function FMatrix.Column0 : FVector;

Var
  res : FVector;

Begin
  res.Init(Row[0].X, Row[1].X, Row[2].X);
  Column0 := res;
End;

Function FMatrix.Column1 : FVector;

Var
  res : FVector;

Begin
  res.Init(Row[0].Y, Row[1].Y, Row[2].Y);
  Column1 := res;
End;

Function FMatrix.Column2 : FVector;

Var
  res : FVector;

Begin
  res.Init(Row[0].Z, Row[1].Z, Row[2].Z);
  Column2 := res;
End;

Procedure FMatrix.MakeXRot(theta : Single);

Var
  c, s : Single;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  c := cos(theta);
  s := sin(theta);
  Row[1].Y := c; Row[1].Z := s; Row[1].X := 0;
  Row[2].Y := -s; Row[2].Z := c; Row[2].X := 0;
  Row[0].Y := 0; Row[0].Z := 0; Row[0].X := 1;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure FMatrix.MakeYRot(theta: Single); inline;
var
  c, s: Single;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure FMatrix.MakeYRot(theta : Single);

Var
  c, s : Single;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  c := cos(theta);
  s := sin(theta);
  Row[2].Z := c; Row[2].X := s; Row[2].Y := 0;
  Row[0].Z := -s; Row[0].X := c; Row[0].Y := 0;
  Row[1].Z := 0; Row[1].X := 0; Row[1].Y := 1;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure FMatrix.MakeZRot(theta: Single); inline;
var
  c, s: Single;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure FMatrix.MakeZRot(theta : Single);

Var
  c, s : Single;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  c := cos(theta);
  s := sin(theta);
  Row[0].X := c; Row[0].Y := s; Row[0].Z := 0;
  Row[1].X := -s; Row[1].Y := c; Row[1].Z := 0;
  Row[2].X := 0; Row[2].Y := 0; Row[2].Z := 1;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure FMatrix.MakeID; inline;
begin
  Row[0].Init(1, 0, 0);
  Row[1].Init(0, 1, 0);
  Row[2].Init(0, 0, 1);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

function FMatrix.Transpose: FMatrix; inline;
begin
  Result.Init(Column0, Column1, Column2);
end;

<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew

function FMatrix.Transpose: FMatrix; inline;
begin
  Result.Init(Column0, Column1, Column2);
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure FMatrix.TransposeInPlace; inline;
begin
  Init(Column0, Column1, Column2);
end;

procedure FMatrix.Normalise; inline;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure FMatrix.MakeID;

Begin
  Row[0].Init(1, 0, 0);
  Row[1].Init(0, 1, 0);
  Row[2].Init(0, 0, 1);
End;

Function FMatrix.Transpose : FMatrix;

Var
  res : FMatrix;

Begin
  res.Init(Column0, Column1, Column2);
  Transpose := res;
End;

Procedure FMatrix.TransposeInPlace;

Begin
  Init(Column0, Column1, Column2);
End;

Procedure FMatrix.Normalise;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Row[2].Normalise;
  Row[0] := Row[1]**Row[2];
  Row[0].Normalise;
  Row[1] := Row[2]**Row[0];
  Row[1].Normalise;
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD

operator * (const m: FMatrix; const a: Single): FMatrix; inline;
begin
  Result.Init(m.Row[0]*a, m.Row[1]*a, m.Row[2]*a);
end;

=======

operator * (const m: FMatrix; const a: Single): FMatrix; inline;
begin
  Result.Init(m.Row[0]*a, m.Row[1]*a, m.Row[2]*a);
end;

>>>>>>> origin/cpstrnew
operator * (const m, a: FMatrix): FMatrix; inline;
var
  v1, v2, v3: FVector;
begin
  v1.Init(m.Row[0].X*a.Row[0].X+m.Row[0].Y*a.Row[1].X+m.Row[0].Z*a.Row[2].X,
          m.Row[0].X*a.Row[0].Y+m.Row[0].Y*a.Row[1].Y+m.Row[0].Z*a.Row[2].Y,
          m.Row[0].X*a.Row[0].Z+m.Row[0].Y*a.Row[1].Z+m.Row[0].Z*a.Row[2].Z);
  v2.Init(m.Row[1].X*a.Row[0].X+m.Row[1].Y*a.Row[1].X+m.Row[1].Z*a.Row[2].X,
          m.Row[1].X*a.Row[0].Y+m.Row[1].Y*a.Row[1].Y+m.Row[1].Z*a.Row[2].Y,
          m.Row[1].X*a.Row[0].Z+m.Row[1].Y*a.Row[1].Z+m.Row[1].Z*a.Row[2].Z);
  v3.Init(m.Row[2].X*a.Row[0].X+m.Row[2].Y*a.Row[1].X+m.Row[2].Z*a.Row[2].X,
          m.Row[2].X*a.Row[0].Y+m.Row[2].Y*a.Row[1].Y+m.Row[2].Z*a.Row[2].Y,
          m.Row[2].X*a.Row[0].Z+m.Row[2].Y*a.Row[1].Z+m.Row[2].Z*a.Row[2].Z);
  Result.Init(v1, v2, v3);
end;

operator * (const m: FMatrix; const a: FVector): FVector; inline;
begin
  Result.Init(a*m.Row[0], a*m.Row[1], a*m.Row[2]);
end;

operator + (const m, a: FMatrix): FMatrix; inline;
begin
  Result.Init(m.Row[0]+a.Row[0], m.Row[1]+a.Row[1], m.Row[2]+a.Row[2]);
end;

operator - (const m, a: FMatrix): FMatrix; inline;
begin
  Result.Init(m.Row[0]+a.Row[0], m.Row[1]+a.Row[1], m.Row[2]+a.Row[2]);
end;

constructor TRay.Init(const p, d: FVector);
begin
  mPosn := p;
  mDir := d;
  mDir.Normalise;
end;

constructor VLight.Create(const col: FVector);
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Operator * (Const m : FMatrix; Const a : Single) res : FMatrix;

Begin
  res.Init(m.Row[0]*a, m.Row[1]*a, m.Row[2]*a);
End;

Operator * (Const m, a : FMatrix) res : FMatrix;

Var
  v1, v2, v3 : FVector;

Begin
  v1.Init(m.Row[0].X*a.Row[0].X+m.Row[0].Y*a.Row[1].X+m.Row[0].Z*a.Row[2].X,
	  m.Row[0].X*a.Row[0].Y+m.Row[0].Y*a.Row[1].Y+m.Row[0].Z*a.Row[2].Y,
	  m.Row[0].X*a.Row[0].Z+m.Row[0].Y*a.Row[1].Z+m.Row[0].Z*a.Row[2].Z);
  v2.Init(m.Row[1].X*a.Row[0].X+m.Row[1].Y*a.Row[1].X+m.Row[1].Z*a.Row[2].X,
	  m.Row[1].X*a.Row[0].Y+m.Row[1].Y*a.Row[1].Y+m.Row[1].Z*a.Row[2].Y,
	  m.Row[1].X*a.Row[0].Z+m.Row[1].Y*a.Row[1].Z+m.Row[1].Z*a.Row[2].Z);
  v3.Init(m.Row[2].X*a.Row[0].X+m.Row[2].Y*a.Row[1].X+m.Row[2].Z*a.Row[2].X,
	  m.Row[2].X*a.Row[0].Y+m.Row[2].Y*a.Row[1].Y+m.Row[2].Z*a.Row[2].Y,
	  m.Row[2].X*a.Row[0].Z+m.Row[2].Y*a.Row[1].Z+m.Row[2].Z*a.Row[2].Z);
  res.Init(v1, v2, v3);
End;

Operator * (Const m : FMatrix; Const a : FVector) res : FVector;

Begin
  res.Init(a*m.Row[0], a*m.Row[1], a*m.Row[2]);
End;

Operator + (Const m, a : FMatrix) res : FMatrix;

Begin
  res.Init(m.Row[0]+a.Row[0], m.Row[1]+a.Row[1], m.Row[2]+a.Row[2]);
End;

Operator - (Const m, a : FMatrix) res : FMatrix;

Begin
  res.Init(m.Row[0]+a.Row[0], m.Row[1]+a.Row[1], m.Row[2]+a.Row[2]);
End;

Constructor TRay.Init(Const p, d : FVector);

Begin
  mPosn := p;
  mDir := d;
  mDir.Normalise;
End;

Constructor VLight.Create(Const col : FVector);

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  mCol := col * 0.9;
  mAng := 2.8;
  mPosn.Init(0, 0, 20);
  mTarget.Init(0, 0, 0.1);
  mAxis.MakeID;
  Update;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure VLight.Move(const q: FVector);
begin
  mPosn := q;
  Update;
end;

procedure VLight.MoveT(const q: FVector);
begin
  mTarget := q;
  Update;
end;

procedure VLight.Update;
begin
  mAxis.Row[2] := (mTarget - mPosn);
  mAxis.Normalise;
end;

function VLight.Light(const ray: TRay): FVector;
var
  f, A, B, C, D, t1, t2, t3, fr, l1, l2, t, h: Single;
  frc, x, y, q: Integer;
  pp: FVector;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure VLight.Move(Const q : FVector);

Begin
  mPosn := q;
  Update;
End;

Procedure VLight.MoveT(Const q : FVector);

Begin
  mTarget := q;
  Update;
End;

Procedure VLight.Update;

Begin
  mAxis.Row[2] := (mTarget - mPosn);
  mAxis.Normalise;
End;

Function VLight.Light(Const ray : TRay) : FVector;

Var
  f, A, B, C, D, t1, t2, t3, fr, l1, l2, t, h : Single;
  frc, x, y, q : Integer;
  pp : FVector;
  res : FVector;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  f := 0;

  p2 := ray.mPosn;
  p := mAxis * (ray.mPosn - mPosn);
  _d := mAxis * ray.mDir;
  A := (_d.X*_d.X+_d.Y*_d.Y);
  B := 2*(_d.X*p.X+_d.Y*p.Y)-mAng*(_d.Z);
  C := (p.X*p.X+p.Y*p.Y)-mAng*(p.Z);
  D := B*B-4*A*C;
<<<<<<< HEAD
<<<<<<< HEAD
  if D <= 0 then
  begin
    Result.Init(0, 0, 0);
    exit;
  end;
  D := Sqrt(D);
  A := A * 2;
=======
=======
>>>>>>> origin/fixes_2_2
  If D <= 0 Then
  Begin
    res.Init(0, 0, 0);
    Light := res;
    Exit;
  End;
  D := Sqrt(D);
  A *= 2;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  t1 := (-B-D)/A;
  t2 := (-B+D)/A;
  frc := 255;
  t3 := -ray.mPosn.Z/ray.mDir.Z;
<<<<<<< HEAD
<<<<<<< HEAD
  if t2<=0 then
  begin
    Result.Init(0, 0, 0);
    exit;
  end;
  if t1<0 then
    t1 := 0;
  if t3>0 then
  begin
=======
=======
>>>>>>> origin/fixes_2_2
  If t2<=0 Then
  Begin
    res.Init(0, 0, 0);
    Light := res;
    Exit;
  End;
  If t1<0 Then
    t1 := 0;
  If t3>0 Then
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { clip to bitmap plane }
    pp := ray.mPosn + ray.mDir*t3;
    x := 160+Trunc(SC*pp.X);
{$IFNDEF INV}
<<<<<<< HEAD
<<<<<<< HEAD
    if (x>=0) and (x<=319) then
    begin
      y := 100 + Trunc(SC*pp.Y);
      if (y>=0) and (y<=199) then
      begin
        {Result.Init(0, 0, 1);
        exit;}
        frc := MaskMap[y*320+x];
        if frc<1 then
        begin
          if t1>t3 then
            t1 := t3;
          if t2>t3 then
            t2 := t3;
        end;
      end
      else
        t3 := t2
    end
    else
      t3 := t2;
{$ELSE}
    if (x >= 0) and (x <= 319) then
    begin
      y := 100 + Trunc(SC*pp.Y);
      if (y >= 0) and (y <= 199) and (MaskMap[y*320 + x] < 128) then
        t3 := t2;
    end;
    if t1 > t3 then
      t1 := t3;
    if t2 > t3 then
      t2 := t3;
{$ENDIF}
  end;
  if t1>=t2 then
  begin
    Result.Init(0, 0, 0);
    exit;
  end;
  fr := frc/255;
  l1 := CalcLight(t1);
  if t1>t3 then
    l1 := l1 * fr;
  q := NSEG;
  t := t1;
  h := (t2-t1)/NSEG;
  if h<MINSEGSIZE then
    h := MINSEGSIZE;
  while (t<t3) and (q>0) and (t<t2) do
  begin
    t := t + h;
    if (t>t2) then
    begin
      h := h - (t2-t);
      t := t2;
      q := 0;
    end
    else
      Dec(q);
    h := (t-t1);
    p := p + _d*h;
    p2 := p2 + ray.mDir*h;
    l2 := CalcLight(t);
    f := f + (l1+l2)*h;
    l1 := l2;
    t1 := t;
  end;
  while (q>0) and (t<t2) do
  begin
    t := t + h;
    if t>t2 then
    begin
      h := h - (t2-t);
      t := t2;
      q := 0;
    end
    else
      Dec(q);
    p := p + _d*h;
    p2 := p2 + ray.mDir*h;
    l2 := CalcLight(t);
    if t>t3 then
      l2 := l2 * fr;
    f := f + (l1+l2)*h;
    l1 := l2;
    t1 := t;
  end;
  Result := mCol*f;
end;

function VLight.CalcLight(t: Single): Single;
var
  f: Single;
  x, y, c: Integer;
begin
  { trace line to bitmap from mPosn to p2 }
  if not ((mPosn.Z > 0) xor (p2.Z > 0)) then
  begin
    { fresnel fall off... }
    Result := p.Z / p.MagnitudeSq;
    exit;
  end;
  f := -(mPosn.Z)/(p2.Z - mPosn.Z);
  x := 160 + Trunc(SC*((p2.X-mPosn.X)*f+mPosn.X));
{$IFNDEF INV}
  if (x < 0) or (x > 319) then
  begin
    Result := p.Z / p.MagnitudeSq;
    exit;
  end;
  y := 100 + Trunc(SC*((p2.Y-mPosn.Y)*f+mPosn.Y));
  if (y < 0) or (y > 199) then
  begin
    Result := p.Z / p.MagnitudeSq;
    exit;
  end;
  c := MaskMap[y * 320 + x];
{$ELSE}
  if (x < 0) or (x > 319) then
  begin
    Result := 0;
    exit;
  end;
  y := 100 + Trunc(SC*((p2.Y-mPosn.Y)*f+mPosn.Y));
  if (y < 0) or (y > 199) then
  begin
    Result := 0;
    exit;
  end;
  c := 255 - MaskMap[y * 320 + x];
{$ENDIF}
  if c = 0 then
  begin
    Result := 0;
    exit;
  end;
  Result := (c*(1/255))*p.Z / p.MagnitudeSq;
end;

function CLIPC(f: Single): Integer; inline;
begin
  Result := Trunc(f * 255);
  if Result < 0 then
    Result := 0
  else
    if Result > 255 then
      Result := 255;
end;

procedure initfrand;
var
  s, c1: Integer;
begin
  FillChar(frandtab, SizeOf(frandtab), 0);
  s := 1;
  for c1 := 1 to 65535 do
  begin
    frandtab[c1] := s and $FFFF;
    s := (((s shr 4) xor (s shr 13) xor (s shr 15)) and 1) + (s shl 1);
  end;
end;

function frand: Integer; inline;
begin
  Result := frandtab[frandtab_seed];
  frandtab_seed := (frandtab_seed + 1) and $FFFF;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure VLightPart(console: IPTCConsole; surface: IPTCSurface);
=======
procedure VLightPart(console: TPTCConsole; surface: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure VLightPart(console: TPTCConsole; surface: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure VLightPart(console: TPTCConsole; surface: TPTCSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure VLightPart(console: TPTCConsole; surface: TPTCSurface);
>>>>>>> origin/cpstrnew
var
  vl: VLight = nil;
  vl2: VLight = nil;
  camposn: FVector;
  camaxis: FMatrix;
  c1, c2, c3, ti, xx, yy, zz, i, a, x, y: Integer;
  idx: array [0..(200 div 16) - 1, 0..(320 div 16) - 1] of Uint8;
  order: array [0..10*19 - 1, 0..1] of Integer;
  vlightt, t, cz, camf: Single;
  col: FVector;
  ray: TRay;
  oc, c, c2_: Uint32;
  time, delta: Single;
  pitch: Integer;
  screenbuf, pd: PUint8;
  tmp: FVector;
  F: File;
begin
  MaskMap := nil;

  try
    oc := 0;
    initfrand;
    tmp.Init(0.1, 0.4, 1);
    vl := VLight.Create(tmp);
    tmp.Init(1, 0.5, 0.2);
    vl2 := VLight.Create(tmp);
    tmp.Init(0, 0, 20);
    vl.Move(tmp);
    tmp.Init(0, 6, 30);
    vl2.Move(tmp);

    camposn.Init(7, 0.5, -10);
    camaxis.Init;
    camaxis.MakeID;
    tmp.Init(0, 0, 0);
    camaxis.Row[2] := tmp - camposn;
    camaxis.Normalise;
    camf := 100;

    MaskMap := GetMem(320 * 200);
    FillChar(MaskMap^, 320 * 200, 0);

    { load mojo.raw }
    AssignFile(F, 'mojo.raw');
    Reset(F, 1);
    try
      BlockRead(F, MaskMap^, 320*200);
    finally
      CloseFile(F);
    end;

    { build the order of the squares }
    for c1 := 0 to 10*19 - 1 do
    begin
      order[c1, 0] := c1 mod 19;
      order[c1, 1] := (c1 div 19) + 1;
    end;

    { swap them around }
    for c1 := 0 to 9999 do
    begin
=======
=======
>>>>>>> origin/fixes_2_2
    If (x>=0) And (x<=319) Then
    Begin
      y := 100 + Trunc(SC*pp.Y);
      If (y>=0) And (y<=199) Then
      Begin
	{res.Init(0, 0, 1);
	Light := res;
	Exit;}
	frc := MaskMap[y*320+x];
	If frc<1 Then
	Begin
	  If t1>t3 Then
	    t1 := t3;
	  If t2>t3 Then
	    t2 := t3;
	End;
      End
      Else
	t3 := t2
    End
    Else
      t3 := t2;
{$ELSE}
    If (x >= 0) And (x <= 319) Then
    Begin
      y := 100 + Trunc(SC*pp.Y);
      If (y >= 0) And (y <= 199) And (MaskMap[y*320 + x] < 128) Then
        t3 := t2;
    End;
    If t1 > t3 Then
      t1 := t3;
    If t2 > t3 Then
      t2 := t3;
{$ENDIF}
  End;
  If t1>=t2 Then
  Begin
    res.Init(0, 0, 0);
    Light := res;
    Exit;
  End;
  fr := frc/255;
  l1 := CalcLight(t1);
  If t1>t3 Then
    l1 *= fr;
  q := NSEG;
  t := t1;
  h := (t2-t1)/NSEG;
  If h<MINSEGSIZE Then
    h := MINSEGSIZE;
  While (t<t3) And (q>0) And (t<t2) Do
  Begin
    t += h;
    If (t>t2) Then
    Begin
      h -= t2-t;
      t := t2;
      q := 0;
    End
    Else
      Dec(q);
    h := (t-t1);
    p += _d*h;
    p2 += ray.mDir*h;
    l2 := CalcLight(t);
    f += (l1+l2)*h;
    l1 := l2;
    t1 := t;
  End;
  While (q>0) And (t<t2) Do
  Begin
    t += h;
    If t>t2 Then
    Begin
      h -= t2-t;
      t := t2;
      q := 0;
    End
    Else
      Dec(q);
    p += _d*h;
    p2 += ray.mDir*h;
    l2 := CalcLight(t);
    If t>t3 Then
      l2 *= fr;
    f += (l1+l2)*h;
    l1 := l2;
    t1 := t;
  End;
  Light := mCol*f;
End;

Function VLight.CalcLight(t : Single) : Single;

Var
  f : Single;
  x, y, c : Integer;

Begin
  { trace line to bitmap from mPosn to p2 }
  If Not ((mPosn.Z > 0) Xor (p2.Z > 0)) Then
  Begin
    { fresnel fall off... }
    CalcLight := p.Z / p.MagnitudeSq;
    Exit;
  End;
  f := -(mPosn.Z)/(p2.Z - mPosn.Z);
  x := 160 + Trunc(SC*((p2.X-mPosn.X)*f+mPosn.X));
{$IFNDEF INV}
  If (x < 0) Or (x > 319) Then
  Begin
    CalcLight := p.Z / p.MagnitudeSq;
    Exit;
  End;
  y := 100 + Trunc(SC*((p2.Y-mPosn.Y)*f+mPosn.Y));
  If (y < 0) Or (y > 199) Then
  Begin
    CalcLight := p.Z / p.MagnitudeSq;
    Exit;
  End;
  c := MaskMap[y * 320 + x];
{$ELSE}
  If (x < 0) Or (x > 319) Then
  Begin
    CalcLight := 0;
    Exit;
  End;
  y := 100 + Trunc(SC*((p2.Y-mPosn.Y)*f+mPosn.Y));
  If (y < 0) Or (y > 199) Then
  Begin
    CalcLight := 0;
    Exit;
  End;
  c := 255 - MaskMap[y * 320 + x];
{$ENDIF}
  If c = 0 Then
  Begin
    CalcLight := 0;
    Exit;
  End;
  CalcLight := (c*(1/255))*p.Z / p.MagnitudeSq;
End;

Function CLIPC(f : Single) : Integer; {Inline;}

Var
  a : Integer;

Begin
  a := Trunc(f * 255);
  If a < 0 Then
    a := 0
  Else
    If a > 255 Then
      a := 255;
  CLIPC := a;
End;

Procedure initfrand;

Var
  s, c1 : Integer;

Begin
  FillChar(frandtab, SizeOf(frandtab), 0);
  s := 1;
  For c1 := 1 To 65535 Do
  Begin
    frandtab[c1] := s And $FFFF;
    s := (((s Shr 4) Xor (s Shr 13) Xor (s Shr 15)) And 1) + (s Shl 1);
  End;
End;

Function frand : Integer; {Inline;}

Begin
  frand := frandtab[frandtab_seed];
  frandtab_seed := (frandtab_seed + 1) And $FFFF;
End;

Procedure VLightPart(console : TPTCConsole; surface : TPTCSurface);

Var
  vl, vl2 : VLight;
  camposn : FVector;
  camaxis : FMatrix;
  c1, c2, c3, ti, xx, yy, zz, i, a, x, y : Integer;
  idx : Array[0..(200 Div 16) - 1, 0..(320 Div 16) - 1] Of Uint8;
  order : Array[0..10*19 - 1, 0..1] Of Integer;
  vlightt, t, cz, camf : Single;
  col : FVector;
  ray : TRay;
  oc, c, c2_ : Uint32;
  time, delta : Single;
  pitch : Integer;
  screenbuf, pd : PUint8;
  tmp : FVector;
  F : File;

Begin
  oc := 0;
  initfrand;
  tmp.Init(0.1, 0.4, 1);
  vl := VLight.Create(tmp);
  tmp.Init(1, 0.5, 0.2);
  vl2 := VLight.Create(tmp);
  tmp.Init(0, 0, 20);
  vl.Move(tmp);
  tmp.Init(0, 6, 30);
  vl2.Move(tmp);

  camposn.Init(7, 0.5, -10);
  camaxis.Init;
  camaxis.MakeID;
  tmp.Init(0, 0, 0);
  camaxis.Row[2] := tmp - camposn;
  camaxis.Normalise;
  camf := 100;

  MaskMap := GetMem(320 * 200);
  FillChar(MaskMap^, 320 * 200, 0);

  { load mojo.raw }
  ASSign(F, 'mojo.raw');
  Reset(F, 1);
  BlockRead(F, MaskMap^, 320*200);
  Close(F);

  { build the order of the squares }
  For c1 := 0 To 10*19 - 1 Do
  Begin
    order[c1, 0] := c1 Mod 19;
    order[c1, 1] := (c1 Div 19) + 1;
  End;

  { swap them around }
  For c1 := 0 To 9999 Do
  Begin
    c2 := Random(190);
    c3 := Random(190);
    ti := order[c2, 0]; order[c2, 0] := order[c3, 0]; order[c3, 0] := ti;
    ti := order[c2, 1]; order[c2, 1] := order[c3, 1]; order[c3, 1] := ti;
  End;

  { time settings }
  time := 0;
  delta := 0.01; { this controls the speed of the effect }

  { main loop }
  While Not console.KeyPressed Do
  Begin
    { get surface data }
    pitch := surface.pitch;

    { light time (makes the effect loop) }
    vlightt := 320 * Abs(Sin(time/5));
    
    t := 13 - 0.1822 * vlightt;
    cz := 1 - 0.01 * vlightt;
    {tmp.Init(Sin(t)*5, Cos(t*-0.675+4543)*5, 15);
    vl.Move(tmp);
    tmp.Init(0, 0, -15);
    vl.Move(tmp);}
    tmp.Init(t, 0, 22);
    vl.Move(tmp);
    tmp.Init(-t, -7, 28);
    vl2.Move(tmp);

    camposn.Init(cz*4+9, cz, -t/7-13);
    tmp.Init(0, 0, 0);
    camaxis.Row[2] := tmp - camposn;
    camaxis.Normalise;

    FillChar(idx, SizeOf(idx), 25);

    { swap them around }
    For c1 := 0 To 99 Do
    Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      c2 := Random(190);
      c3 := Random(190);
      ti := order[c2, 0]; order[c2, 0] := order[c3, 0]; order[c3, 0] := ti;
      ti := order[c2, 1]; order[c2, 1] := order[c3, 1]; order[c3, 1] := ti;
<<<<<<< HEAD
<<<<<<< HEAD
    end;

    { time settings }
    time := 0;
    delta := 0.01; { this controls the speed of the effect }

    { main loop }
    while not console.KeyPressed do
    begin
      { get surface data }
      pitch := surface.pitch;

      { light time (makes the effect loop) }
      vlightt := 320 * Abs(Sin(time/5));

      t := 13 - 0.1822 * vlightt;
      cz := 1 - 0.01 * vlightt;
      {tmp.Init(Sin(t)*5, Cos(t*-0.675+4543)*5, 15);
      vl.Move(tmp);
      tmp.Init(0, 0, -15);
      vl.Move(tmp);}
      tmp.Init(t, 0, 22);
      vl.Move(tmp);
      tmp.Init(-t, -7, 28);
      vl2.Move(tmp);

      camposn.Init(cz*4+9, cz, -t/7-13);
      tmp.Init(0, 0, 0);
      camaxis.Row[2] := tmp - camposn;
      camaxis.Normalise;

      FillChar(idx, SizeOf(idx), 25);

      { swap them around }
      for c1 := 0 to 99 do
      begin
        c2 := Random(190);
        c3 := Random(190);
        ti := order[c2, 0]; order[c2, 0] := order[c3, 0]; order[c3, 0] := ti;
        ti := order[c2, 1]; order[c2, 1] := order[c3, 1]; order[c3, 1] := ti;
      end;
      for zz := 0 to 189 do
      begin
        xx := order[zz, 0];
        yy := order[zz, 1];
        i := 0;

        { lock surface }
        screenbuf := surface.lock;
        try
          c2 := idx[yy, xx] shr 1;
          for c1 := 0 to c2 - 1 do
          begin
            a := frand and 255;
            x := xx * 16 + (a and 15) + 6 + 4;
            y := yy * 16 + (a shr 4) + 6;

            col.Init(0, 0, 0);
            ray.Init(camposn, camaxis.Row[2]*camf+camaxis.Row[0]*(x-160)+camaxis.Row[1]*(y-100));
            col := col + vl.Light(ray);
            col := col + vl2.Light(ray);

            c := (CLIPC(col.X) shl 16) + (CLIPC(col.Y) shl 8) + (CLIPC(col.Z));
            pd := screenbuf + x*4 + y*pitch;
            Inc(i, Abs(Integer(c and 255)-Integer(pd[321] and 255)) + Abs(Integer(c shr 16)-Integer(pd[321] shr 16)));
            if c1 <> 0 then
              Inc(i, Abs(Integer(c and 255)-Integer(oc and 255)) + Abs(Integer(c shr 16)-Integer(oc shr 16)));
            oc := c;

            c2_ := (c shr 1) and $7F7F7F;
            PUint32(pd)[1] := ((PUint32(pd)[1]) shr 1) and $7F7F7F+ c2_;
            PUint32(pd)[2] := ((PUint32(pd)[2]) shr 1) and $7F7F7F+ c2_;
            Inc(pd, pitch);
            PUint32(pd)[0] := ((PUint32(pd)[0]) shr 1) and $7F7F7F+ c2_;
            PUint32(pd)[1] := c;
            PUint32(pd)[2] := c;
            PUint32(pd)[3] := ((PUint32(pd)[3]) shr 1) and $7F7F7F+ c2_;
            Inc(pd, pitch);
            PUint32(pd)[0] := ((PUint32(pd)[0]) shr 1) and $7F7F7F+ c2_;
            PUint32(pd)[1] := c;
            PUint32(pd)[2] := c;
            PUint32(pd)[3] := ((PUint32(pd)[3]) shr 1) and $7F7F7F+ c2_;
            Inc(pd, pitch);
            PUint32(pd)[1] := ((PUint32(pd)[1]) shr 1) and $7F7F7F+ c2_;
            PUint32(pd)[2] := ((PUint32(pd)[2]) shr 1) and $7F7F7F+ c2_;
          end;
          i := i * 5;
          i := i div (3*idx[yy, xx]);
          if i < 2 then
            i := 2;
          if i > {256}255 then
            i := {256}255;
          idx[yy, xx] := i;
        finally
          { unlock surface }
          surface.unlock;
        end;

        if (zz mod 95) = 0 then
        begin
          { copy surface to console }
          surface.copy(console);

          { update console }
          console.update;
        end;
      end;
      { update time }
      time := time + delta;
    end;
  finally
    FreeMem(MaskMap);
    vl.Free;
    vl2.Free;
  end;
end;

var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  format: IPTCFormat;
  console: IPTCConsole;
  surface: IPTCSurface;
=======
  format: TPTCFormat = nil;
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
>>>>>>> graemeg/cpstrnew
=======
  format: TPTCFormat = nil;
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
>>>>>>> graemeg/cpstrnew
=======
  format: TPTCFormat = nil;
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
>>>>>>> graemeg/cpstrnew
=======
  format: TPTCFormat = nil;
  console: TPTCConsole = nil;
  surface: TPTCSurface = nil;
>>>>>>> origin/cpstrnew
begin
  try
    try
      { create format }
<<<<<<< HEAD
      format := TPTCFormatFactory.CreateNew(32, $00FF0000, $0000FF00, $000000FF);

      { create console }
      console := TPTCConsoleFactory.CreateNew;
=======
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

      { create console }
      console := TPTCConsole.Create;
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

      { open console }
      console.open('mojo by statix', 320, 200, format);

      { create main drawing surface }
<<<<<<< HEAD
      surface := TPTCSurfaceFactory.CreateNew(320, 200, format);
=======
      surface := TPTCSurface.Create(320, 200, format);
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

      { do the light effect }
      VLightPart(console, surface);

    finally
      { close console }
<<<<<<< HEAD
      if Assigned(console) then
        console.close;
=======
=======
=======
>>>>>>> origin/fixes_2_2
    End;
    For zz := 0 To 189 Do
    Begin
      xx := order[zz, 0];
      yy := order[zz, 1];
      i := 0;
      
      { lock surface }
      screenbuf := surface.lock;
      Try
        c2 := idx[yy, xx] Shr 1;
        For c1 := 0 To c2 - 1 Do
        Begin
	  a := frand And 255;
	  x := xx * 16 + (a And 15) + 6 + 4;
	  y := yy * 16 + (a Shr 4) + 6;

	  col.Init(0, 0, 0);
	  ray.Init(camposn, camaxis.Row[2]*camf+camaxis.Row[0]*(x-160)+camaxis.Row[1]*(y-100));
	  col += vl.Light(ray);
	  col += vl2.Light(ray);

	  c := (CLIPC(col.X) Shl 16) + (CLIPC(col.Y) Shl 8) + (CLIPC(col.Z));
	  pd := screenbuf + x*4 + y*pitch;
	  Inc(i, Abs(Integer(c And 255)-Integer(pd[321] And 255)) + Abs(Integer(c Shr 16)-Integer(pd[321] Shr 16)));
	  If c1 <> 0 Then
	    Inc(i, Abs(Integer(c And 255)-Integer(oc And 255)) + Abs(Integer(c Shr 16)-Integer(oc Shr 16)));
	  oc := c;

	  c2_ := (c Shr 1) And $7F7F7F;
	  PUint32(pd)[1] := ((PUint32(pd)[1]) Shr 1) And $7F7F7F+ c2_;
	  PUint32(pd)[2] := ((PUint32(pd)[2]) Shr 1) And $7F7F7F+ c2_;
	  Inc(pd, pitch);
	  PUint32(pd)[0] := ((PUint32(pd)[0]) Shr 1) And $7F7F7F+ c2_;
	  PUint32(pd)[1] := c;
	  PUint32(pd)[2] := c;
	  PUint32(pd)[3] := ((PUint32(pd)[3]) Shr 1) And $7F7F7F+ c2_;
	  Inc(pd, pitch);
	  PUint32(pd)[0] := ((PUint32(pd)[0]) Shr 1) And $7F7F7F+ c2_;
	  PUint32(pd)[1] := c;
	  PUint32(pd)[2] := c;
	  PUint32(pd)[3] := ((PUint32(pd)[3]) Shr 1) And $7F7F7F+ c2_;
	  Inc(pd, pitch);
	  PUint32(pd)[1] := ((PUint32(pd)[1]) Shr 1) And $7F7F7F+ c2_;
	  PUint32(pd)[2] := ((PUint32(pd)[2]) Shr 1) And $7F7F7F+ c2_;
        End;
        i *= 5;
        i := i Div (3*idx[yy, xx]);
        If i < 2 Then
	  i := 2;
        If i > {256}255 Then
	  i := {256}255;
        idx[yy, xx] := i;
      Finally
        { unlock surface }
        surface.unlock;
      End;
      
      If (zz Mod 95) = 0 Then
      Begin
        { copy surface to console }
	surface.copy(console);
	
        { update console }
	console.update;
      End;
    End;
    { update time }
    time += delta;
  End;
  FreeMem(MaskMap);
  vl.Free;
  vl2.Free;
End;

Var
  format : TPTCFormat;
  console : TPTCConsole;
  surface : TPTCSurface;

Begin
  format := Nil;
  surface := Nil;
  console := Nil;
  Try
    Try
      { create format }
      format := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);
      
      { create console }
      console := TPTCConsole.Create;
      
      { open console }
      console.open('mojo by statix', 320, 200, format);
      
      { create main drawing surface }
      surface := TPTCSurface.Create(320, 200, format);
      
      { do the light effect }
      VLightPart(console, surface);
      
    Finally
      { close console }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      console.close;
      console.Free;
      surface.Free;
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

=======
    End;
    
>>>>>>> graemeg/fixes_2_2
=======
    End;
    
>>>>>>> origin/fixes_2_2
    { print message to stdout }
    Writeln('mojo by alex "statix" evans');
    Writeln('to be used as an example of bad coding and good ptc');
    Writeln('no responsibility taken for this!');
    Writeln('enjoy ptc! it''s great');
    Writeln;
    Writeln('-statix 13/1/98');
<<<<<<< HEAD
<<<<<<< HEAD
  except
    on error: TPTCError do
      { report error }
      error.report;
  end;
end.
=======
=======
>>>>>>> origin/fixes_2_2
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
