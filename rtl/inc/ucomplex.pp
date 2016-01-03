{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Pierre Muller,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
Unit UComplex;
{$ifndef VER2_0}
{$INLINE ON}
{$define TEST_INLINE}
{$endif VER2_0}

{ created for FPC by Pierre Muller }
{ inpired from the complex unit from  JD GAYRARD mai 95 }
{ FPC supports operator overloading }


  interface

{$ifndef FPUNONE}
    uses math;

    type complex = record
                     re : real;
                     im : real;
                   end;

    pcomplex = ^complex;

    const i : complex = (re : 0.0; im : 1.0);
          _0 : complex = (re : 0.0; im : 0.0);


    { assignment overloading is also used in type conversions
      (beware also in implicit type conversions)
      after this operator any real can be passed to a function
      as a complex arg !! }

    operator := (r : real) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    { operator := (i : longint) z : complex;
      not needed because longint can be converted to real }


    { four operator : +, -, * , /  and comparison = }
    operator + (z1, z2 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    { these ones are created because the code
      is simpler and thus faster }
    operator + (z1 : complex; r : real) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator + (r : real; z1 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}


    operator - (z1, z2 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator - (z1 : complex;r : real) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator - (r : real; z1 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}


    operator * (z1, z2 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator * (z1 : complex; r : real) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator * (r : real; z1 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}


    operator / (znum, zden : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator / (znum : complex; r : real) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator / (r : real; zden : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    { ** is the exponentiation operator }
    operator ** (z1, z2 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator ** (z1 : complex; r : real) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator ** (r : real; z1 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}


    operator = (z1, z2 : complex) b : boolean;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator = (z1 : complex;r : real) b : boolean;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator = (r : real; z1 : complex) b : boolean;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}

    operator - (z1 : complex) z : complex;
    {$ifdef TEST_INLINE}
    inline;
    {$endif TEST_INLINE}


    { complex functions }
    function cong (z : complex) : complex;      { conjuge }

    { inverse function 1/z }
    function cinv (z : complex) : complex;

    { complex functions with real return values }
    function cmod (z : complex) : real;           { module }
    function carg (z : complex) : real;           { argument : a / z = p.e^ia }

    { fonctions elementaires }
    function cexp (z : complex) : complex;       { exponential }
    function cln (z : complex) : complex;        { natural logarithm }
    function csqrt (z : complex) : complex;      { square root }

    { complex trigonometric functions  }
    function ccos (z : complex) : complex;       { cosinus }
    function csin (z : complex) : complex;       { sinus }
    function ctg  (z : complex) : complex;       { tangent }

    { inverse complex trigonometric functions }
    function carc_cos (z : complex) : complex;   { arc cosinus }
    function carc_sin (z : complex) : complex;   { arc sinus }
    function carc_tg  (z : complex) : complex;   { arc tangent }

    { hyperbolic complex functions }
    function cch (z : complex) : complex;        { hyperbolic cosinus }
    function csh (z : complex) : complex;        { hyperbolic sinus }
    function cth (z : complex) : complex;        { hyperbolic tangent }

    { inverse hyperbolic complex functions }
    function carg_ch (z : complex) : complex;    { hyperbolic arc cosinus }
    function carg_sh (z : complex) : complex;    { hyperbolic arc sinus }
    function carg_th (z : complex) : complex;    { hyperbolic arc tangente }

    { functions to write out a complex value }
    function cstr(z : complex) : string;
    function cstr(z:complex;len : integer) : string;
    function cstr(z:complex;len,dec : integer) : string;

  implementation

  operator := (r : real) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}

    begin
       z.re:=r;
       z.im:=0.0;
    end;

  { four base operations  +, -, * , / }

  operator + (z1, z2 : complex) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { addition : z := z1 + z2 }
    begin
       z.re := z1.re + z2.re;
       z.im := z1.im + z2.im;
    end;

  operator + (z1 : complex; r : real) z : complex;
  { addition : z := z1 + r }
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    begin
       z.re := z1.re + r;
       z.im := z1.im;
    end;

  operator + (r : real; z1 : complex) z : complex;
  { addition : z := r + z1 }
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}

    begin
       z.re := z1.re + r;
       z.im := z1.im;
    end;

  operator - (z1, z2 : complex) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { substraction : z := z1 - z2 }
    begin
       z.re := z1.re - z2.re;
       z.im := z1.im - z2.im;
    end;

  operator - (z1 : complex; r : real) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { substraction : z := z1 - r }
    begin
       z.re := z1.re - r;
       z.im := z1.im;
    end;

  operator - (z1 : complex) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { substraction : z := - z1 }
    begin
       z.re := -z1.re;
       z.im := -z1.im;
    end;

  operator - (r : real; z1 : complex) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { substraction : z := r - z1 }
    begin
       z.re := r - z1.re;
       z.im := - z1.im;
    end;

  operator * (z1, z2 : complex) z : complex;
  { multiplication : z := z1 * z2 }
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    begin
       z.re := (z1.re * z2.re) - (z1.im * z2.im);
       z.im := (z1.re * z2.im) + (z1.im * z2.re);
    end;

  operator * (z1 : complex; r : real) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { multiplication : z := z1 * r }
    begin
       z.re := z1.re * r;
       z.im := z1.im * r;
    end;

  operator * (r : real; z1 : complex) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
  { multiplication : z := r * z1 }
    begin
       z.re := z1.re * r;
       z.im := z1.im * r;
    end;

  operator / (znum, zden : complex) z : complex;
  {$ifdef TEST_INLINE}
  inline;
  {$endif TEST_INLINE}
    { division : z := znum / zden }
    { The following algorithm is used to properly handle
      denominator overflow:

                 |  a + b(d/c)   c - a(d/c)
                 |  ---------- + ---------- I     if |d| < |c|
      a + b I    |  c + d(d/c)   a + d(d/c)
      -------  = |
      c + d I    |  b + a(c/d)   -a+ b(c/d)
                 |  ---------- + ---------- I     if |d| >= |c|
                 |  d + c(c/d)   d + c(c/d)
    }
     var
       tmp, denom : real;
     begin
       if ( abs(zden.re) > abs(zden.im) ) then
       begin
          tmp := zden.im / zden.re;
          denom := zden.re + zden.im * tmp;
          z.re := (znum.re + znum.im * tmp) / denom;
          z.im := (znum.im - znum.re * tmp) / denom;
       end
       else
       begin
          tmp := zden.re / zden.im;
          denom := zden.im + zden.re * tmp;
          z.re := (znum.im + znum.re * tmp) / denom;
          z.im := (-znum.re + znum.im * tmp) / denom;
       end;
     end;

    operator / (znum : complex; r : real) z : complex;
      { division : z := znum / r }
      begin
         z.re := znum.re / r;
         z.im := znum.im / r;
      end;

  operator / (r : real; zden : complex) z : complex;
    { division : z := r / zden }
    var denom : real;
    begin
       with zden do denom := (re * re) + (im * im);
       { generates a fpu exception if denom=0 as for reals }
       z.re := (r * zden.re) / denom;
       z.im := - (r * zden.im) / denom;
    end;

  function cmod (z : complex): real;
    { module : r = |z| }
    begin
       with z do
         cmod := sqrt((re * re) + (im * im));
    end;

  function carg (z : complex): real;
    { argument : 0 / z = p ei0 }
    begin
       carg := arctan2(z.im, z.re);
    end;

  function cong (z : complex) : complex;
    { complex conjugee :
       if z := x + i.y
       then cong is x - i.y }
    begin
       cong.re := z.re;
       cong.im := - z.im;
    end;

  function cinv (z : complex) : complex;
    { inverse : r := 1 / z }
    var
       denom : real;
    begin
       with z do denom := (re * re) + (im * im);
       { generates a fpu exception if denom=0 as for reals }
       cinv.re:=z.re/denom;
       cinv.im:=-z.im/denom;
    end;

  operator = (z1, z2 : complex) b : boolean;
    { returns TRUE if z1 = z2 }
    begin
       b := (z1.re = z2.re) and (z1.im = z2.im);
    end;

  operator = (z1 : complex; r :real) b : boolean;
    { returns TRUE if z1 = r }
    begin
       b := (z1.re = r) and (z1.im = 0.0)
    end;

  operator = (r : real; z1 : complex) b : boolean;
    { returns TRUE if z1 = r }
    begin
       b := (z1.re = r) and (z1.im = 0.0)
    end;


  { fonctions elementaires }

  function cexp (z : complex) : complex;
    { exponantial : r := exp(z) }
    { exp(x + iy) = exp(x).exp(iy) = exp(x).[cos(y) + i sin(y)] }
    var expz : real;
    begin
       expz := exp(z.re);
       cexp.re := expz * cos(z.im);
       cexp.im := expz * sin(z.im);
    end;

  function cln (z : complex) : complex;
    { natural logarithm : r := ln(z) }
    { ln( p exp(i0)) = ln(p) + i0 + 2kpi }
    begin
       cln.re := ln(cmod(z));
       cln.im := arctan2(z.im, z.re);
    end;

  function csqrt (z : complex) : complex;
    { square root : r := sqrt(z) }
    var
       root, q : real;
    begin
      if (z.re<>0.0) or (z.im<>0.0) then
        begin
           root := sqrt(0.5 * (abs(z.re) + cmod(z)));
           q := z.im / (2.0 * root);
           if z.re >= 0.0 then
             begin
                csqrt.re := root;
                csqrt.im := q;
             end
           else if z.im < 0.0 then
             begin
                csqrt.re := - q;
                csqrt.im := - root
             end
           else
             begin
                csqrt.re :=  q;
                csqrt.im :=  root
             end
        end
       else csqrt := z;
    end;


  operator ** (z1, z2 : complex) z : complex;
    { exp : z := z1 ** z2 }
    begin
       z := cexp(z2*cln(z1));
    end;

  operator ** (z1 : complex; r : real) z : complex;
    { multiplication : z := z1 * r }
    begin
       z := cexp( r *cln(z1));
    end;

  operator ** (r : real; z1 : complex) z : complex;
    { multiplication : z := r + z1 }
    begin
       z := cexp(z1*ln(r));
    end;

  { direct trigonometric functions }

  function ccos (z : complex) : complex;
    { complex cosinus }
    { cos(x+iy) = cos(x).cos(iy) - sin(x).sin(iy) }
    { cos(ix) = cosh(x) et sin(ix) = i.sinh(x) }
    begin
       ccos.re := cos(z.re) * cosh(z.im);
       ccos.im := - sin(z.re) * sinh(z.im);
    end;

  function csin (z : complex) : complex;
    { sinus complex }
    { sin(x+iy) = sin(x).cos(iy) + cos(x).sin(iy) }
    { cos(ix) = cosh(x) et sin(ix) = i.sinh(x) }
    begin
       csin.re := sin(z.re) * cosh(z.im);
       csin.im := cos(z.re) * sinh(z.im);
    end;

  function ctg (z : complex) : complex;
    { tangente }
    var ccosz, temp : complex;
    begin
       ccosz := ccos(z);
       temp := csin(z);
       ctg := temp / ccosz;
    end;

  { fonctions trigonometriques inverses }

  function carc_cos (z : complex) : complex;
    { arc cosinus complex }
    { arccos(z) = -i.argch(z) }
    begin
       carc_cos := -i*carg_ch(z);
    end;

  function carc_sin (z : complex) : complex;
    { arc sinus complex }
    { arcsin(z) = -i.argsh(i.z) }
    begin
       carc_sin := -i*carg_sh(i*z);
    end;

  function carc_tg (z : complex) : complex;
    { arc tangente complex }
    { arctg(z) = -i.argth(i.z) }
    begin
       carc_tg := -i*carg_th(i*z);
    end;

  { hyberbolic complex functions }

  function cch (z : complex) : complex;
    { hyberbolic cosinus }
    { cosh(x+iy) = cosh(x).cosh(iy) + sinh(x).sinh(iy) }
    { cosh(iy) = cos(y) et sinh(iy) = i.sin(y) }
    begin
       cch.re := cosh(z.re) * cos(z.im);
       cch.im := sinh(z.re) * sin(z.im);
    end;

  function csh (z : complex) : complex;
    { hyberbolic sinus }
    { sinh(x+iy) = sinh(x).cosh(iy) + cosh(x).sinh(iy) }
    { cosh(iy) = cos(y) et sinh(iy) = i.sin(y) }
    begin
       csh.re := sinh(z.re) * cos(z.im);
       csh.im := cosh(z.re) * sin(z.im);
    end;

  function cth (z : complex) : complex;
    { hyberbolic complex tangent }
    { th(x) = sinh(x) / cosh(x) }
    { cosh(x) > 1 qq x }
    var temp : complex;
    begin
       temp := cch(z);
       z := csh(z);
       cth := z / temp;
    end;

  { inverse complex hyperbolic functions }

  function carg_ch (z : complex) : complex;
    {   hyberbolic arg cosinus }
    {                          _________  }
    { argch(z) = -/+ ln(z + i.V 1 - z.z)  }
    begin
       carg_ch:=-cln(z+i*csqrt(1.0-z*z));
    end;

  function carg_sh (z : complex) : complex;
    {   hyperbolic arc sinus       }
    {                    ________  }
    { argsh(z) = ln(z + V 1 + z.z) }
    begin
       carg_sh:=cln(z+csqrt(z*z+1.0));
    end;

  function carg_th (z : complex) : complex;
    { hyperbolic arc tangent }
    { argth(z) = 1/2 ln((z + 1) / (1 - z)) }
    begin
       carg_th:=cln((z+1.0)/(1.0-z))/2.0;
    end;

  { functions to write out a complex value }
  function cstr(z : complex) : string;
    var
       istr : string;
    begin
       str(z.im,istr);
       str(z.re,cstr);
       while istr[1]=' ' do
         delete(istr,1,1);
       if z.im<0 then
         cstr:=cstr+istr+'i'
       else if z.im>0 then
         cstr:=cstr+'+'+istr+'i';
    end;

    function cstr(z:complex;len : integer) : string;
    var
       istr : string;
    begin
       str(z.im:len,istr);
       while istr[1]=' ' do
         delete(istr,1,1);
       str(z.re:len,cstr);
       if z.im<0 then
         cstr:=cstr+istr+'i'
       else if z.im>0 then
         cstr:=cstr+'+'+istr+'i';
    end;

    function cstr(z:complex;len,dec : integer) : string;
    var
       istr : string;
    begin
       str(z.im:len:dec,istr);
       while istr[1]=' ' do
         delete(istr,1,1);
       str(z.re:len:dec,cstr);
       if z.im<0 then
         cstr:=cstr+istr+'i'
       else if z.im>0 then
         cstr:=cstr+'+'+istr+'i';
    end;


{$else}
implementation
{$endif FPUNONE}
end.
