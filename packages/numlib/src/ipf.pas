{
    This file is part of the Numlib package.
    Copyright (c) 1986-2000 by
     Kees van Ginneken, Wil Kortsmit and Loek van Reij of the
     Computational centre of the Eindhoven University of Technology

    FPC port Code          by Marco van de Voort (marco@freepascal.org)
             documentation by Michael van Canneyt (Michael@freepascal.org)

    Interpolate and (curve) fitting.
    Slegpb in this unit patched parameters slightly. Units IPF and sle
    were not in the same revision in this numlib copy (which was a
    copy of the work directory of the author) .

    Contains two undocumented functions. If you recognize the algoritm,
    mail us.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
 }
unit ipf;
{$I direct.inc}
interface

uses typ, mdt, dsl, sle, spe;

{ Determine natural cubic spline "s" for data set (x,y), output to (a,d2a)
 term=1 success,
     =2 failure calculating "s"
     =3 wrong input (e.g. x,y is not sorted increasing on x)}
procedure ipffsn(n: ArbInt; var x, y, a, d2a: ArbFloat; var term: ArbInt);

{calculate d2s from x,y, which can be used to calculate s}
procedure ipfisn(n: ArbInt; var x, y, d2s: ArbFloat; var term: ArbInt);

{Calculate function value for dataset (x,y), with n.c. spline d2s for
x value t. Return (corrected) y value.
s calculated from x,y, with e.g. ipfisn}
function  ipfspn(n: ArbInt; var x, y, d2s: ArbFloat; t: ArbFloat;
                 var term: ArbInt): ArbFloat;

{Calculate n-degree polynomal b for dataset (x,y) with n elements
 using the least squares method.}
procedure ipfpol(m, n: ArbInt; var x, y, b: ArbFloat; var term: ArbInt);


                {**** undocumented ****}

function spline(    n     : ArbInt;
                    x     : complex;
                var ac    : complex;
                var gammar: ArbFloat;
                    u1    : ArbFloat;
                    pf    : complex): ArbFloat;

                {**** undocumented ****}

procedure splineparameters
          (    n                 : ArbInt;
           var ac, alfadc        : complex;
           var lambda,
               gammar, u1,
               kwsom, energie    : ArbFloat;
           var pf                : complex);

implementation


procedure ipffsn(n: ArbInt; var x, y, a, d2a: ArbFloat; var term: ArbInt);

var                    i, j, sr, n1s, ns1, ns2: ArbInt;
   s, lam, lam0, lam1, lambda, ey, ca, p, q, r: ArbFloat;
     px, py, pd, pa, pd2a,
  h, z, diagb, dinv, qty, qtdinvq, c, t, tl: ^arfloat1;
                                         ub: boolean;

  procedure solve; {n, py, qty, h, qtdinvq, dinv, lam, t, pa, pd2a, term}
  var i: ArbInt;
      p, q, r, ca: ArbFloat;
             f, c: ^arfloat1;
  begin
    getmem(f, 3*ns1); getmem(c, ns1);
    for i:=1 to n-1 do
      begin
        f^[3*i]:=qtdinvq^[3*i]+lam*t^[2*i];
        if i > 1
        then
          f^[3*i-1]:=qtdinvq^[3*i-1]+lam*t^[2*i-1];
        if i > 2
        then
          f^[3*i-2]:=qtdinvq^[3*i-2];
        if lam=0
        then
          c^[i]:=qty^[i]
        else
          c^[i]:=lam*qty^[i]
      end;
    slegpb(n-1, 2,{ 3,} f^[1], c^[1], pd2a^[1], ca, term);
    if term=2
    then
      begin
        freemem(f, 3*ns1); freemem(c, ns1);
        exit
      end;
    p:=1/h^[1];
    if lam=0
    then
      r:=1
    else
      r:=1/lam;
    q:=1/h^[2]; pa^[1]:=py^[1]-r*dinv^[1]*p*pd2a^[1];
    pa^[2]:=py^[2]-r*dinv^[2]*(pd2a^[2]*q-(p+q)*pd2a^[1]); p:=q;
    for i:=3 to n-1 do
      begin
        q:=1/h^[i];
        pa^[i]:=py^[i]-r*dinv^[i]*
                (p*pd2a^[i-2]-(p+q)*pd2a^[i-1]+q*pd2a^[i]);
        p:=q
      end;
    q:=1/h^[n];
    pa^[n]:=py^[n]-r*dinv^[n]*(p*pd2a^[n-2]-(p+q)*pd2a^[n-1]);
    pa^[n+1]:=py^[n+1]-r*dinv^[n+1]*q*pd2a^[n-1];
    if lam=0
    then
      for i:=1 to n-1 do
        pd2a^[i]:=0;
    freemem(f, 3*ns1); freemem(c, ns1);
  end; {solve}

    function e(var c, h: ArbFloat; n:ArbInt): ArbFloat;
    var i:ArbInt;
        s:ArbFloat;
        pc, ph: ^arfloat1;
    begin
      ph:=@h; pc:=@c;
      s:=ph^[1]*pc^[1]*pc^[1];
      for i:=1 to n-2 do
        s:=s+(pc^[i]*(pc^[i]+pc^[i+1])+pc^[i+1]*pc^[i+1])*ph^[i+1];
      e:=(s+pc^[n-1]*pc^[n-1]*ph^[n])/3
    end; {e}

    function cr(lambda: ArbFloat): ArbFloat;
    var s, crs: ArbFloat;
             i: ArbInt;
    begin
      cr:=0; lam:=lambda;
      solve; { n, py, qty, h, qtdinvq, dinv, lam, t, pa, pd2a, term }
      if term=2
      then
        exit;
      crs:=ey;
      if lam <> 0
      then
        begin
          crs:=crs+e(pd2a^[1], h^[1], n);
          s:=0;
          for i:=1 to n-1 do
            s:=s+pd2a^[i]*qty^[i];
          crs:=crs-2*s
        end;
      s:=0;
      for i:=1 to n+1 do
        s:=s+sqr(pa^[i]-py^[i])*diagb^[i];
      cr:=crs-s
    end; {cr}

    procedure roof1r(a, b, ae, re: ArbFloat; var x: ArbFloat);

    var fa, fb, c, fc, m, tol, w1, w2 : ArbFloat;
                                    k : ArbInt;
                                 stop : boolean;

    begin
      fa:=cr(a);
      if term=2
      then
        exit;
      fb:=cr(b);
      if term=2
      then
        exit;
      if abs(fb)>abs(fa)
      then
        begin
          c:=b; fc:=fb; x:=a; b:=a; fb:=fa; a:=c; fa:=fc
        end
      else
        begin
          c:=a; fc:=fa; x:=b
        end;
      k:=0;
      tol:=ae+re*spemax(abs(a), abs(b));
      w1:=abs(b-a); stop:=false;
      while (abs(b-a)>tol) and (fb<>0) and (not stop) do
        begin
          m:=(a+b)/2;
          if (k>=2) or (fb=fc)
          then
            x:=m
          else
            begin
              x:=(b*fc-c*fb)/(fc-fb);
              if abs(b-x)<tol
              then
                x:=b-tol*spesgn(b-a);
              if spesgn(x-m)=spesgn(x-b)
              then
                x:=m
            end;
          c:=b; fc:=fb; b:=x; fb:=cr(x);
          if term=2
          then
            exit;
          if spesgn(fa)*spesgn(fb)>0
          then
            begin
              a:=c; fa:=fc; k:=0
            end
          else
            k:=k+1;
          if abs(fb)>=abs(fa)
          then
            begin
              c:=b; fc:=fb; x:=a; b:=a; fb:=fa; a:=c; fa:=fc; k:=0
            end;
          tol:=ae+re*spemax(abs(a), abs(b));
          w2:=abs(b-a);
          if w2>=w1
          then
            stop:=true;
          w1:=w2
        end
    end; {roof1r}

procedure NoodGreep;
var I, j: ArbInt;
begin
  i:=1;
  while i <= n do
    begin
      if (pd^[i] <= 0) or (px^[i+1] <= px^[i])
      then
        begin
          term:=3;
          exit
        end;
      i:=i+1
    end;
  if pd^[n+1] <= 0
  then
    begin
      term:=3;
      exit
    end;
  for i:=1 to n+1 do
    dinv^[i]:=1/pd^[i];
  for i:=1 to n do
    h^[i]:=px^[i+1]-px^[i];
  t^[2]:=(h^[1]+h^[2])/3;
  for i:=2 to n-1 do
    begin
      t^[2*i]:=(h^[i]+h^[i+1])/3; t^[2*i-1]:=h^[i]/6
    end;
  move(t^[1], tl^[1], ns2);
  mdtgpb(n-1, 1, 2, tl^[1], ca, term);
  if term=2
  then
    exit;
  z^[1]:=1/(h^[1]*tl^[2]);
  for j:=2 to n-1 do
    z^[j]:=-(tl^[2*j-1]*z^[j-1])/tl^[2*j];
  s:=0;
  for j:=1 to n-1 do
    s:=s+sqr(z^[j]);
  diagb^[1]:=s;
  z^[1]:=(-1/h^[1]-1/h^[2])/tl^[2];
  if n>2
  then
    z^[2]:=(1/h^[2]-tl^[3]*z^[1])/tl^[4];
  for j:=3 to n-1 do
    z^[j]:=-tl^[2*j-1]*z^[j-1]/tl^[2*j];
  s:=0;
  for j:=1 to n-1 do
    s:=s+sqr(z^[j]);
  diagb^[2]:=s;
  for i:=2 to n-2 do
    begin
      z^[i-1]:=1/(h^[i]*tl^[2*(i-1)]);
      z^[i]:=(-1/h^[i]-1/h^[i+1]-tl^[2*i-1]*z^[i-1])/tl^[2*i];
      z^[i+1]:=(1/h^[i+1]-tl^[2*i+1]*z^[i])/tl^[2*(i+1)];
      for j:=i+2 to n-1 do
        z^[j]:=-tl^[2*j-1]*z^[j-1]/tl^[2*j];
      s:=0;
      for j:=i-1 to n-1 do
        s:=s+sqr(z^[j]);
      diagb^[i+1]:=s
    end;
  z^[n-2]:=1/(h^[n-1]*tl^[2*(n-2)]);
  z^[n-1]:=(-1/h^[n-1]-1/h^[n]-tl^[2*n-3]*z^[n-2])/tl^[2*(n-1)];
  s:=0;
  for j:=n-2 to n-1 do
    s:=s+sqr(z^[j]);
  diagb^[n]:=s;
  diagb^[n+1]:=1/sqr(h^[n]*tl^[2*(n-1)]);
  p:=1/h^[1];
  for i:=2 to n do
    begin
      q:=1/h^[i]; qty^[i-1]:=py^[i+1]*q-py^[i]*(p+q)+py^[i-1]*p;
      p:=q
    end;
  p:=1/h^[1]; q:=1/h^[2]; r:=1/h^[3];
  qtdinvq^[3]:=dinv^[1]*p*p+dinv^[2]*(p+q)*(p+q)+dinv^[3]*q*q;
  if n>2
  then
    begin
      qtdinvq^[6]:=dinv^[2]*q*q+dinv^[3]*(q+r)*(q+r)+dinv^[4]*r*r;
      qtdinvq^[5]:=-(dinv^[2]*(p+q)+dinv^[3]*(q+r))*q;
      p:=q; q:=r;
      for i:=3 to n-1 do
        begin
          r:=1/h^[i+1];
          qtdinvq^[3*i]:=dinv^[i]*q*q+dinv^[i+1]*(q+r)*(q+r)+dinv^[i+2]*r*r;
          qtdinvq^[3*i-1]:=-(dinv^[i]*(p+q)+dinv^[i+1]*(q+r))*q;
          qtdinvq^[3*i-2]:=dinv^[i]*p*q;
          p:=q; q:=r
        end
    end;
  dslgpb(n-1, 1, 2, tl^[1], qty^[1], c^[1], term);
  if term=2
  then
    exit;
  ey:=e(c^[1], h^[1], n);
  lam0:=0;
  s:=cr(lam0);
  if term=2
  then
    exit;
  if s >= 0
  then
    begin
      lambda:=0; term:=4
    end
  else
    begin
      lam1:=1e-8; ub:=false;
      while (not ub) and (lam1<=1.1e8) do
        begin
          s:=cr(lam1);
          if term=2
          then
            exit;
          if s  >= 0
          then
            ub:=true
          else
            begin
              lam0:=lam1; lam1:=10*lam1
            end
        end;
      if not ub
      then
        begin
          term:=4; lambda:=lam0
        end
      else
        roof1r(lam0, lam1, 0, 1e-6, lambda);
      if term=2
      then
        exit
    end;

end;

begin
  term:=1;
  if n < 2
  then
    begin
      term:=3; exit
    end;
  sr:=sizeof(ArbFloat);
  n1s:=(n+1)*sr;
  ns2:=2*(n-1)*sr;
  ns1:=(n-1)*sr;
  getmem(dinv, n1s);
  getmem(h, n*sr);
  getmem(t, ns2);
  getmem(tl, ns2);
  getmem(z, ns1);
  getmem(diagb, n1s);
  getmem(qtdinvq, 3*ns1);
  getmem(c, ns1);
  getmem(qty, ns1);

   getmem(pd, n1s);
 { pd:=@d; }
  px:=@x;
  py:=@y;
  pa:=@a;
  pd2a:=@d2a;
  { de gewichten van de punten worden op 1 gezet}
  for i:=1 to n+1 do
    pd^[i]:=1;

  NoodGreep;

  freemem(dinv, n1s);
  freemem(h, n*sr);
  freemem(t, ns2);
  freemem(tl, ns2);
  freemem(z, ns1);
  freemem(diagb, n1s);
  freemem(qtdinvq, 3*ns1);
  freemem(c, ns1);
  freemem(qty, ns1);

  freemem(pd, n1s);
end; {ipffsn}

procedure ortpol(m, n: ArbInt; var x, alfa, beta: ArbFloat);
// this function used to use mark/release.
var
                             i, j, ms : ArbInt;
    xppn1, ppn1, ppn, p, alfaj, betaj : ArbFloat;
               px, pal, pbe, pn, pn1 : ^arfloat1;
begin
  px:=@x; pal:=@alfa; pbe:=@beta; ms:=m*sizeof(ArbFloat);
  getmem(pn, ms); getmem(pn1, ms);
  xppn1:=0; ppn1:=m;
  for i:=1 to m do
    begin
      pn^[i]:=0; pn1^[i]:=1; xppn1:=xppn1+px^[i]
    end;
  pal^[1]:=xppn1/ppn1; pbe^[1]:=0;
  for j:=2 to n do
    begin
      alfaj:=pal^[j-1]; betaj:=pbe^[j-1];
      ppn:=ppn1; ppn1:=0; xppn1:=0;
      for i:=1 to m do
        begin
          p:=(px^[i]-alfaj)*pn1^[i]-betaj*pn^[i];
          pn^[i]:=pn1^[i]; pn1^[i]:=p; p:=p*p;
          ppn1:=ppn1+p; xppn1:=xppn1+px^[i]*p
        end; {i}
      pal^[j]:=xppn1/ppn1; pbe^[j]:=ppn1/ppn
    end; {j}
    freemem(pn); freemem(pn1);
end; {ortpol}

procedure ortcoe(m, n: ArbInt; var x, y, alfa, beta, a: ArbFloat);
// this function used to use mark/release.
var                        i, j, mr : ArbInt;
         fpn, ppn, p, alphaj, betaj : ArbFloat;
    px, py, pal, pbe, pa, pn, pn1 : ^arfloat1;

begin
  mr:=m*sizeof(ArbFloat);
  px:=@x; py:=@y; pal:=@alfa; pbe:=@beta; pa:=@a;
  getmem(pn, mr); getmem(pn1, mr);
  fpn:=0;
  for i:=1 to m do
    begin
      pn^[i]:=0; pn1^[i]:=1; fpn:=fpn+py^[i]
    end; {i}
  pa^[1]:=fpn/m;
  for j:=1 to n do
    begin
      fpn:=0; ppn:=0; alphaj:=pal^[j]; betaj:=pbe^[j];
      for i:=1 to m do
        begin
          p:=(px^[i]-alphaj)*pn1^[i]-betaj*pn^[i];
          pn^[i]:=pn1^[i]; pn1^[i]:=p;
          fpn:=fpn+py^[i]*p; ppn:=ppn+p*p
        end; {i}
      pa^[j+1]:=fpn/ppn
    end; {j}
    freemem(pn); freemem(pn1);  
end; {ortcoe}

procedure polcoe(n:ArbInt; var alfa, beta, a, b: ArbFloat);

var            k, j : ArbInt;
           pal, pbe : ^arfloat1;
            pa, pb  : ^arfloat0;

begin
  pal:=@alfa; pbe:=@beta; pa:=@a; pb:=@b;
  move(pa^[0], pb^[0], (n+1)*sizeof(ArbFloat));
  for j:=0 to n-1 do
    for k:=n-j-1 downto 0 do
      begin
        pb^[k+j]:=pb^[k+j]-pal^[k+1]*pb^[k+j+1];
        if k+j<>n-1
        then
          pb^[k+j]:=pb^[k+j]-pbe^[k+2]*pb^[k+j+2]
      end
end; {polcoe}

procedure ipfpol(m, n: ArbInt; var x, y, b: ArbFloat; var term: ArbInt);

var                      i, ns: ArbInt;
                          fsum: ArbFloat;
            px, py, alfa, beta: ^arfloat1;
                         pb, a: ^arfloat0;
begin
  if (n<0) or (m<1)
  then
    begin
      term:=3; exit
    end;
  term:=1;
  if n = 0
  then
    begin
      py:=@y; pb:=@b;
      fsum:=0;
      for i:=1 to m do
        fsum:=fsum+py^[i];
      pb^[0]:=fsum/m
    end
  else
    begin
      if n>m-1
      then
        begin
          pb:=@b;
          fillchar(pb^[m], (n-m+1)*sizeof(ArbFloat), 0);
          n:=m-1
        end;
      ns:=n*sizeof(ArbFloat);
      getmem(alfa, ns); getmem(beta, ns);
      getmem(a, (n+1)*sizeof(ArbFloat));
      ortpol(m, n, x, alfa^[1], beta^[1]);
      ortcoe(m, n, x, y, alfa^[1], beta^[1], a^[0]);
      polcoe(n, alfa^[1], beta^[1], a^[0], b);
      freemem(alfa, ns); freemem(beta, ns);
      freemem(a, (n+1)*sizeof(ArbFloat));
    end
end; {ipfpol}

procedure ipfisn(n: ArbInt; var x, y, d2s: ArbFloat; var term: ArbInt);

var
                   s, i : ArbInt;
               p, q, ca : ArbFloat;
        px, py, h, b, t : ^arfloat0;
                   pd2s : ^arfloat1;
begin
  px:=@x; py:=@y; pd2s:=@d2s;
  term:=1;
  if n < 2
  then
    begin
      term:=3; exit
    end; {n<2}
  s:=sizeof(ArbFloat);
  getmem(h, n*s);
  getmem(b, (n-1)*s);
  getmem(t, 2*(n-1)*s);
  for i:=0 to n-1 do
    h^[i]:=px^[i+1]-px^[i];
  q:=1/6; p:=2*q;
  t^[1]:=p*(h^[0]+h^[1]);
  for i:=2 to n-1 do
    begin
      t^[2*i-1]:=p*(h^[i-1]+h^[i]); t^[2*i-2]:=q*h^[i-1]
    end; {i}
  p:=1/h^[0];
  for i:=2 to n do
    begin
      q:=1/h^[i-1]; b^[i-2]:=py^[i]*q-py^[i-1]*(p+q)+py^[i-2]*p; p:=q
    end;
  slegpb(n-1, 1, {2,} t^[0], b^[0], pd2s^[1], ca, term);
  freemem(h, n*s);
  freemem(b, (n-1)*s);
  freemem(t, 2*(n-1)*s);
end; {ipfisn}

function ipfspn(n: ArbInt; var x, y, d2s: ArbFloat; t:ArbFloat;
                var term: ArbInt): ArbFloat;

var
   px, py       : ^arfloat0;
   pd2s         : ^arfloat1;
   i, j, m      : ArbInt;
   d, s3, h, dy : ArbFloat;
begin
  i:=1; term:=1;
  if n<2
  then
    begin
      term:=3; exit
    end; {n<2}
  px:=@x; py:=@y; pd2s:=@d2s;
  if t <= px^[0]
  then
    begin
      h:=px^[1]-px^[0];
      dy:=(py^[1]-py^[0])/h-h*pd2s^[1]/6;
      ipfspn:=py^[0]+(t-px^[0])*dy
    end { t <= x[0] }
  else
  if t >= px^[n]
  then
    begin
      h:=px^[n]-px^[n-1];
      dy:=(py^[n]-py^[n-1])/h+h*pd2s^[n-1]/6;
      ipfspn:=py^[n]+(t-px^[n])*dy
    end { t >= x[n] }
  else
    begin
      i:=0; j:=n;
      while j <> i+1 do
        begin
          m:=(i+j) div 2;
          if t>=px^[m]
          then
            i:=m
          else
            j:=m
        end; {j}
      h:=px^[i+1]-px^[i];
      d:=t-px^[i];
      if i=0
      then
        begin
          s3:=pd2s^[1]/h;
          dy:=(py^[1]-py^[0])/h-h*pd2s^[1]/6;
          ipfspn:=py^[0]+d*(dy+d*d*s3/6)
        end
      else
      if i=n-1
      then
        begin
          s3:=-pd2s^[n-1]/h;
          dy:=(py^[n]-py^[n-1])/h-h*pd2s^[n-1]/3;
          ipfspn:=py^[n-1]+d*(dy+d*(pd2s^[n-1]/2+d*s3/6))
        end
      else
        begin
          s3:=(pd2s^[i+1]-pd2s^[i])/h;
          dy:=(py^[i+1]-py^[i])/h-h*(2*pd2s^[i]+pd2s^[i+1])/6;
          ipfspn:=py^[i]+d*(dy+d*(pd2s^[i]/2+d*s3/6))
        end
   end  { x[0] < t < x[n] }
end; {ipfspn}

function p(x, a, z:complex): ArbFloat;
begin
      x.sub(a);
      p:=x.Inp(z)
end;

function e(x, y: complex): ArbFloat;
const c1: ArbFloat = 0.01989436788646;
var s: ArbFloat;
begin x.sub(y);
      s := x.norm;
      if s=0 then e:=0 else e:=c1*s*ln(s)
end;

function spline(    n     : ArbInt;
                    x     : complex;
                var ac    : complex;
                var gammar: ArbFloat;
                    u1    : ArbFloat;
                    pf    : complex): ArbFloat;
var i     : ArbInt;
    s     : ArbFloat;
    a     : arcomp0 absolute ac;
    gamma : arfloat0 absolute gammar;
begin
    s := u1 + p(x, a[n-2], pf);
    for i:=0 to n do s := s + gamma[i]*e(x,a[i]);
    spline := s
end;

procedure splineparameters
          (    n                 : ArbInt;
           var ac, alfadc        : complex;
           var lambda,
               gammar, u1,
               kwsom, energie    : ArbFloat;
           var pf                : complex);

   procedure SwapC(var v, w: complex);
   var x: complex;
   begin
       x := v; v := w; w := x
   end;

   procedure pxpy(a, b, c: complex; var p:complex);
   var det: ArbFloat;
   begin
        b.sub(a); c.sub(a); det := b.xreal*c.imag-b.imag*c.xreal;
        b.sub(c); p.Init(b.imag/det, -b.xreal/det)
   end;

   procedure pfxpfy(a, b, c: complex; f: vector; var pf: complex);
   begin
      b.sub(a); c.sub(a);
      f.j := f.j-f.i; f.k := f.k-f.i;
      pf.init(f.j*c.imag - f.k*b.imag, -f.j*c.xreal + f.k*b.xreal);
      pf.scale(1/(b.xreal*c.imag - b.imag*c.xreal))
   end;

   function InpV(n: ArbInt; var v1, v2: ArbFloat): ArbFloat;
   var i: ArbInt;
       a1: arfloat0 absolute v1;
       a2: arfloat0 absolute v2;
       s : ArbFloat;
   begin
       s := 0;
       for i:=0 to n-1 do s := s + a1[i]*a2[i];
       InpV := s
   end;

   PROCEDURE SPDSOL(    N  : INTEGER;
                    VAR AP : pointer;
                    VAR B  : ArbFloat);
   VAR I, J, K : INTEGER;
       H       : ArbFloat;
       a       : ^ar2dr absolute ap;
       bx      : arfloat0 absolute b;
   BEGIN
      for k:=0 to n do
      BEGIN
          h := sqrt(a^[k]^[k]-InpV(k, a^[k]^[0], a^[k]^[0]));
          a^[k]^[k] := h;
          FOR I:=K+1 TO N do a^[i]^[k] := (a^[i]^[k] - InpV(k, a^[k]^[0], a^[i]^[0]))/h;
          BX[K] := (bx[k] - InpV(k, a^[k]^[0], bx[0]))/h
      END;
      FOR I:=N DOWNTO 0 do
      BEGIN
          H := BX[I];
          FOR J:=I+1 TO N DO H := H - A^[J]^[I]*BX[J];
          BX[I] := H/A^[I]^[I]
      END
   END;

var i, j, i1 : ArbInt;
    x, h,
    absdet,
    absdetmax,
    s, s1, ca: ArbFloat;
    alfa, dv, hulp,
    u, v, w  : vector;
    e22      : array[0..2] of vector;
    e21, b   : ^arvect0;
    k, c     : ^ar2dr;
    gamma    : arfloat0 absolute gammar;
    an2, an1, an, z,
    vz, wz   : complex;
    a        : arcomp0 absolute ac;
    alfad    : arcomp0 absolute alfadc;

begin

  i1:=0;
  x:=a[0].xreal;
  for i:=1 to n do
  begin
       h:=a[i].xreal;
       if h<x then begin i1:=i; x:=h end
  end;
  SwapC(a[n-2], a[i1]);
  SwapC(alfad[n-2], alfad[i1]);

  x:=a[0].xreal;
  i1 := 0;
  for i:=1 to n do
  begin
       h:=a[i].xreal;
       if h>x then begin i1:=i; x:=h end
  end;
  SwapC(a[n-1], a[i1]);
  SwapC(alfad[n-1], alfad[i1]);

  vz := a[n-2]; vz.sub(a[n-1]);

  absdetmax := -1;
  for i:=0 to n do
  begin
    wz := a[i]; wz.sub(a[n-2]);
    absdet := abs(wz.imag*vz.xreal-wz.xreal*vz.imag);
    if absdet>absdetmax then begin i1:=i; absdetmax:=absdet end
  end;
  SwapC(a[n], a[i1]);
  SwapC(alfad[n], alfad[i1]);

  an2 := a[n-2]; an1 := a[n-1]; an := a[n];
  alfa.i := alfad[n-2].xreal; dv.i := alfad[n-2].imag;
  alfa.j := alfad[n-1].xreal; dv.j := alfad[n-1].imag;
  alfa.k := alfad[n  ].xreal; dv.k := alfad[n  ].imag;

  n := n - 3;

  GetMem(k, (n+1)*SizeOf(pointer));
  for j:=0 to n do GetMem(k^[j], (j+1)*SizeOf(ArbFloat));

  GetMem(e21, (n+1)*SizeOf(vector));
  GetMem(b, (n+1)*SizeOf(vector));

  pxpy(an2,an1,an,z); for i:=0 to n do b^[i].i:=1+p(a[i],an2,z);
  pxpy(an1,an,an2,z); for i:=0 to n do b^[i].j:=1+p(a[i],an1,z);
  pxpy(an,an2,an1,z); for i:=0 to n do b^[i].k:=1+p(a[i],an,z);

  e22[0].init(0,e(an1,an2),e(an,an2));
  e22[1].init(e(an1,an2),0,e(an,an1));
  e22[2].init(e(an,an2),e(an,an1),0);

  for j:=0 to n do e21^[j].init(e(an2,a[j]),e(an1,a[j]),e(an,a[j]));

  GetMem(c, (n+1)*SizeOf(pointer));
  for j:=0 to n do GetMem(c^[j], (j+1)*SizeOf(ArbFloat));

  for i:=0 to n do
  for j:=0 to i do
  begin
    if j=i then s:=0 else s:=e(a[i],a[j]);
    hulp.init(b^[j].Inprod(e22[0]), b^[j].Inprod(e22[1]), b^[j].Inprod(e22[2]));
    hulp.sub(e21^[j]);
    k^[i]^[j] := s+b^[i].InProd(hulp)-b^[j].Inprod(e21^[i]);
    if j=i then s:=1/alfad[i].imag else s:=0;
    hulp.init(b^[j].i/dv.i, b^[j].j/dv.j, b^[j].k/dv.k);
    c^[i]^[j] := k^[i]^[j] + (s + b^[i].Inprod(hulp))/lambda
  end;

  for i:=0 to n do gamma[i]:=alfad[i].xreal - b^[i].Inprod(alfa);

  SpdSol(n, pointer(c), gamma[0]);

  for j:=n downto 0 do FreeMem(c^[j], (j+1)*SizeOf(ArbFloat));
  FreeMem(c, (n+1)*SizeOf(pointer));

  s:=0; for j:=0 to n do s:=s+b^[j].i*gamma[j]; w.i:=s; gamma[n+1] := -s;
  s:=0; for j:=0 to n do s:=s+b^[j].j*gamma[j]; w.j:=s; gamma[n+2] := -s;
  s:=0; for j:=0 to n do s:=s+b^[j].k*gamma[j]; w.k:=s; gamma[n+3] := -s;
  FreeMem(b, (n+1)*SizeOf(vector));

  u.init(w.i/dv.i, w.j/dv.j, w.k/dv.k);
  u.scale(1/lambda);
  u.add(alfa);

  s:=0; for j:=0 to n do s:=s+e21^[j].i*gamma[j]; v.i := e22[0].inprod(w)-s;
  s:=0; for j:=0 to n do s:=s+e21^[j].j*gamma[j]; v.j := e22[1].inprod(w)-s;
  s:=0; for j:=0 to n do s:=s+e21^[j].k*gamma[j]; v.k := e22[2].inprod(w)-s;
  FreeMem(e21, (n+1)*SizeOf(vector));

  u.add(v);

  pfxpfy(an2, an1, an, u, pf); u1:=u.i;

  kwsom := 0; for j:=0 to n do kwsom:=kwsom+sqr(gamma[j])/alfad[j].imag;
  kwsom := kwsom+sqr(w.i)/dv.i+sqr(w.j)/dv.j+sqr(w.k)/dv.k;
  kwsom := kwsom/sqr(lambda);

  s:=0;
  for i:=0 to n do
  begin s1:=0;
        for j:=0 to i do s1:=s1+k^[i]^[j]*gamma[j];
        for j:=i+1 to n do s1:=s1+k^[j]^[i]*gamma[j];
        s := gamma[i]*s1+s
  end;
  for j:=n downto 0 do FreeMem(k^[j], (j+1)*SizeOf(ArbFloat));
  FreeMem(k, (n+1)*SizeOf(pointer));
  energie := s

end {splineparameters};

end.
