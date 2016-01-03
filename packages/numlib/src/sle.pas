{
    This file is part of the Numlib package.
    Copyright (c) 1986-2000 by
     Kees van Ginneken, Wil Kortsmit and Loek van Reij of the
     Computational centre of the Eindhoven University of Technology

    FPC port Code          by Marco van de Voort (marco@freepascal.org)
             documentation by Michael van Canneyt (Michael@freepascal.org)

    !! modifies randseed, might not exactly work as TP version!!!

    Solve set of linear equations of the type Ax=b, for generic, and a
    variety of special matrices.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{Solve set of linear equations of the type Ax=b, for generic, and a variety of
special matrices.
One (generic) function for overdetermined sets of this kind : slegls

overdetermined are sets that look like this: (I don't know if I
translated "overdetermined" right)

    6   1  2  3     9
    3   9  3  4     2
   17  27 42 15    62
   17  27 42 15    61

The two bottom rows look much alike, which introduces a big uncertainty in the
result, therefore these matrices need special treatment.

All procedures have similar procedure with a "L" appended to the name. We
didn't receive docs for those procedures. If you know what the difference is,
please mail us }

Unit sle;
interface
{$I DIRECT.INC}

uses typ, omv;

{solve for special tridiagonal matrices}
Procedure sledtr(n: ArbInt; Var l, d, u, b, x: ArbFloat; Var term: ArbInt);

{solve for generic bandmatrices}
Procedure slegba(n, l, r: ArbInt;
                 Var a, b, x, ca: ArbFloat; Var term:ArbInt);

Procedure slegbal(n, l, r: ArbInt;
                  Var a1; Var b1, x1, ca: ArbFloat; Var term: ArbInt);

{generic solve for all matrices}
Procedure slegen(n, rwidth: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Procedure slegenl(    n: ArbInt;
                  Var a1;
                  Var b1, x1, ca: ArbFloat;
                  Var term: ArbInt);

{solve for overdetermined matrices, see unit comments}
Procedure slegls(Var a: ArbFloat; m, n, rwidtha: ArbInt; Var b, x: ArbFloat;
                 Var term: ArbInt);


Procedure sleglsl(Var a1; m, n: ArbInt; Var b1, x1: ArbFloat;
                  Var term: ArbInt);

{Symmetrical positive definitive bandmatrices}
Procedure slegpb(n, l: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Procedure slegpbl(n, l: ArbInt;
                  Var a1; Var b1, x1, ca: ArbFloat; Var term: ArbInt);

{Symmetrical positive definitive matrices}
Procedure slegpd(n, rwidth: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Procedure slegpdl(n: ArbInt; Var a1; Var b1, x1, ca: ArbFloat;
                  Var term: ArbInt);

{Symmetrical matrices}
Procedure slegsy(n, rwidth: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Procedure slegsyl(n: ArbInt; Var a1; Var b1, x1, ca: ArbFloat;
                  Var term: ArbInt);

{tridiagonal matrices}
Procedure slegtr(n:ArbInt; Var l, d, u, b, x, ca: ArbFloat;
                 Var term: ArbInt);

implementation

Uses DSL,MDT;

{Here originally stood an exact copy of mdtgtr from unit mdt}
{Here originally stood an exact copy of dslgtr from unit DSL}

Procedure decomp(Var qr: ArbFloat; m, n, rwidthq: ArbInt; Var alpha: ArbFloat;
                 Var pivot, term: ArbInt);

Var  i, j, jbar, k, ns, ii        : ArbInt;
     beta, sigma, alphak, qrkk, s : ArbFloat;
     pqr, pal, y, sum             : ^arfloat1;
     piv                          : ^arint1;

Begin
  term := 1;
  pqr := @qr;
  pal := @alpha;
  piv := @pivot;
  ns := n*sizeof(ArbFloat);
  getmem(y, ns);
  getmem(sum, ns);
  For j:=1 To n Do
    Begin
      s := 0;
      For i:=1 To m Do
        s := s+sqr(pqr^[(i-1)*rwidthq+j]);
      sum^[j] := s;
      piv^[j] := j
    End; {j}
  For k:=1 To n Do
    Begin
      sigma := sum^[k];
      jbar := k;
      For j:=k+1 To n Do
        If sigma < sum^[j] Then
          Begin
            sigma := sum^[j];
           jbar := j
          End;
      If jbar <> k
       Then
        Begin
          i := piv^[k];
          piv^[k] := piv^[jbar];
          piv^[jbar] := i;
          sum^[jbar] := sum^[k];
          sum^[k] := sigma;
          For i:=1 To m Do
            Begin
              ii := (i-1)*rwidthq;
              sigma := pqr^[ii+k];
              pqr^[ii+k] := pqr^[ii+jbar];
              pqr^[ii+jbar] := sigma
            End; {i}
        End; {column interchange}
      sigma := 0;
      For i:=k To m Do
        sigma := sigma+sqr(pqr^[(i-1)*rwidthq+k]);
      If sigma=0 Then
        Begin
          term := 2;
          freemem(y, ns);
          freemem(sum, ns);
          exit
        End;
      qrkk := pqr^[(k-1)*rwidthq+k];
      If qrkk < 0 Then
        alphak := sqrt(sigma)
      Else
        alphak := -sqrt(sigma);
      pal^[k] := alphak;
      beta := 1/(sigma-qrkk*alphak);
      pqr^[(k-1)*rwidthq+k] := qrkk-alphak;
      For j:=k+1 To n Do
        Begin
          s := 0;
          For i:=k To m Do
            Begin
              ii := (i-1)*rwidthq;
              s := s+pqr^[ii+k]*pqr^[ii+j]
            End; {i}
          y^[j] := beta*s
        End; {j}
      For j:=k+1 To n Do
        Begin
          For i:=k To m Do
            Begin
              ii := (i-1)*rwidthq;
              pqr^[ii+j] := pqr^[ii+j]-pqr^[ii+k]*y^[j]
            End; {i}
          sum^[j] := sum^[j]-sqr(pqr^[(k-1)*rwidthq+j])
        End {j}
    End; {k}
  freemem(y, ns);
 freemem(sum, ns);
End; {decomp}

Procedure decomp1(Var qr1; m, n: ArbInt; Var alpha1: ArbFloat;
                  Var pivot1, term: ArbInt);

Var             i, j, jbar, k, ns : ArbInt;
     beta, sigma, alphak, qrkk, s : ArbFloat;
     qr                           : ar2dr1 absolute qr1;
     alpha                        : arfloat1 absolute alpha1;
     pivot                        : arint1 absolute pivot1;
     y, sum                       : ^arfloat1;
Begin
  term := 1;
  ns := n*sizeof(ArbFloat);
  getmem(y, ns);
 getmem(sum, ns);
  For j:=1 To n Do
    Begin
      s := 0;
      For i:=1 To m Do
       s := s+sqr(qr[i]^[j]);
      sum^[j] := s;
     pivot[j] := j
    End; {j}
  For k:=1 To n Do
    Begin
      sigma := sum^[k];
     jbar := k;
      For j:=k+1 To n Do
        If sigma < sum^[j]
         Then
          Begin
            sigma := sum^[j];
           jbar := j
          End;
      If jbar <> k
       Then
        Begin
          i := pivot[k];
         pivot[k] := pivot[jbar];
         pivot[jbar] := i;
          sum^[jbar] := sum^[k];
         sum^[k] := sigma;
          For i:=1 To m Do
            Begin
              sigma := qr[i]^[k];
             qr[i]^[k] := qr[i]^[jbar];
              qr[i]^[jbar] := sigma
            End; {i}
        End; {column interchange}
      sigma := 0;
      For i:=k To m Do
       sigma := sigma+sqr(qr[i]^[k]);
      If sigma=0
       Then
        Begin
          term := 2;
         freemem(y, ns);
         freemem(sum, ns);
         exit
        End;
      qrkk := qr[k]^[k];
      If qrkk < 0 Then alphak := sqrt(sigma)
     Else alphak := -sqrt(sigma);
      alpha[k] := alphak;
      beta := 1/(sigma-qrkk*alphak);
      qr[k]^[k] := qrkk-alphak;
      For j:=k+1 To n Do
        Begin
          s := 0;
         For i:=k To m Do
          s := s+qr[i]^[k]*qr[i]^[j];
         y^[j] := beta*s
        End; {j}
      For j:=k+1 To n Do
        Begin
          For i:=k To m Do
           qr[i]^[j] := qr[i]^[j]-qr[i]^[k]*y^[j];
          sum^[j] := sum^[j]-sqr(qr[k]^[j])
        End {j}
    End; {k}
  freemem(y, ns);
 freemem(sum, ns);
End; {decomp1}

Procedure solve(Var qr: ArbFloat; m, n, rwidthq: ArbInt; Var alpha: ArbFloat;
                Var pivot: ArbInt; Var r, y: ArbFloat);

Var    i, j, ii            : ArbInt;
       gamma, s            : ArbFloat;
       pqr, pal, pr, py, z : ^arfloat1;
       piv                 : ^arint1;
Begin
  pqr := @qr;
  pal := @alpha;
  piv := @pivot;
  pr := @r;
  py := @y;
  getmem(z, n*sizeof(ArbFloat));
  For j:=1 To n Do
    Begin
      gamma := 0;
      For i:=j To m Do
        gamma := gamma+pqr^[(i-1)*rwidthq+j]*pr^[i];
      gamma := gamma/(pal^[j]*pqr^[(j-1)*rwidthq+j]);
      For i:=j To m Do
        pr^[i] := pr^[i]+gamma*pqr^[(i-1)*rwidthq+j]
    End; {j}
  z^[n] := pr^[n]/pal^[n];
  For i:=n-1 Downto 1 Do
    Begin
      s := pr^[i];
      ii := (i-1)*rwidthq;
      For j:=i+1 To n Do
        s := s-pqr^[ii+j]*z^[j];
      z^[i] := s/pal^[i]
    End; {i}
  For i:=1 To n Do
    py^[piv^[i]] := z^[i];
  freemem(z, n*sizeof(ArbFloat));
End; {solve}

Procedure solve1(Var qr1; m, n: ArbInt; Var alpha1: ArbFloat;
                 Var pivot1: ArbInt; Var r1, y1: ArbFloat);

Var    i, j                : ArbInt;
       gamma, s            : ArbFloat;
       qr                  : ar2dr1 absolute qr1;
       alpha               : arfloat1 absolute alpha1;
       r                   : arfloat1 absolute r1;
       y                   : arfloat1 absolute y1;
       pivot               : arint1 absolute pivot1;
       z                   : ^arfloat1;
Begin
  getmem(z, n*sizeof(ArbFloat));
  For j:=1 To n Do
    Begin
      gamma := 0;
      For i:=j To m Do
       gamma := gamma+qr[i]^[j]*r[i];
      gamma := gamma/(alpha[j]*qr[j]^[j]);
      For i:=j To m Do
       r[i] := r[i]+gamma*qr[i]^[j]
    End; {j}
  z^[n] := r[n]/alpha[n];
  For i:=n-1 Downto 1 Do
    Begin
      s := r[i];
      For j:=i+1 To n Do
       s := s-qr[i]^[j]*z^[j];
      z^[i] := s/alpha[i]
    End; {i}
  For i:=1 To n Do
   y[pivot[i]] := z^[i];
  freemem(z, n*sizeof(ArbFloat));
End; {solve1}

Procedure sledtr(n: ArbInt; Var l, d, u, b, x: ArbFloat; Var term: ArbInt);

Var               i, j, sr : ArbInt;
                    lj, di : ArbFloat;
        pd, pu, pb, px, dd : ^arfloat1;
                        pl : ^arfloat2;
Begin
  If n<1
   Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  pl := @l;
 pd := @d;
 pu := @u;
 pb := @b;
 px := @x;
  sr := sizeof(ArbFloat);
  getmem(dd, n*sr);
  move(pb^, px^, n*sr);
  j := 1;
 di := pd^[j];
 dd^[j] := di;
  If di=0
   Then
    term := 2
  Else
    term := 1;
  while (term=1) and (j <> n) Do
    Begin
      i := j;
     j := j+1;
     lj := pl^[j]/di;
      di := pd^[j]-lj*pu^[i];
     dd^[j] := di;
      If di=0
       Then
        term := 2
      Else
        px^[j] := px^[j]-lj*px^[i]
    End; {j}
  If term=1
   Then
    Begin
      px^[n] := px^[n]/dd^[n];
      For i:=n-1 Downto 1 Do
        px^[i] := (px^[i]-pu^[i]*px^[i+1])/dd^[i]
    End; {term=1}
  freemem(dd, n*sr);
End; {sledtr}

Procedure slegba(n, l, r: ArbInt;
                 Var a, b, x, ca: ArbFloat; Var term:ArbInt);

Var
  sr, i, j, k, ipivot, lbj, lbi, ubi, ls,
         ii, jj, ll, ubj, rwidth       : ArbInt;
  normr, sumrowi, pivot, normt, maxim, h  : ArbFloat;
  pa, pb, px, au, sumrow, t, row              : ^arfloat1;
Begin
  If (n<1) Or (l<0) Or (r<0) Or (l>n-1) Or (r>n-1)
   Then
    Begin
      term := 3;
     exit
    End; {term=3}
  sr := sizeof(ArbFloat);
  pa := @a;
 pb := @b;
 px := @x;
  ll := l+r+1;
  ls := ll*sr;
  getmem(au, ls*n);
  getmem(sumrow, n*sr);
  getmem(t, n*sr);
  getmem(row, ls);
  move(pb^, px^, n*sr);
  jj := 1;
 ii := 1;
  For i:=1 To n Do
    Begin
      If i <= l+1 Then
        Begin
          If i <= n-r Then rwidth := r+i
         Else rwidth := n
        End
     Else
          If i <= n-r Then rwidth := ll
     Else rwidth := n-i+l+1;
      move(pa^[jj], au^[ii], rwidth*sr);
      fillchar(au^[ii+rwidth], (ll-rwidth)*sr, 0);
      jj := jj+rwidth;
     ii := ii+ll;
    End; {i}
  lbi := n-r+1;
 lbj := 0;
  normr := 0;
 term := 1;
  ii := 1;
  For i:=1 To n Do
    Begin
      sumrowi := omvn1v(au^[ii], ll);
      ii := ii+ll;
      sumrow^[i] := sumrowi;
      h := 2*random-1;
     t^[i] := sumrowi*h;
      h := abs(h);
     If normr<h Then normr := h;
      If sumrowi=0 Then term := 2
    End; {i}
  ubi := l;
 k := 0;
 jj := 1;
  while (k<n) and (term=1) Do
    Begin
      maxim := 0;
     k := k+1;
     ipivot := k;
     ii := jj;
      If ubi<n
       Then ubi := ubi+1;
      For i:=k To ubi Do
        Begin
          sumrowi := sumrow^[i];
          If sumrowi <> 0
           Then
            Begin
              h := abs(au^[ii])/sumrowi;
              ii := ii+ll;
              If maxim<h
               Then
                Begin
                  maxim := h;
                 ipivot := i
                End {maxim<h}
            End {sumrowi <> 0}
        End; {i}
      If maxim=0
       Then
        term := 2
      Else
        Begin
          If ipivot <> k
           Then
            Begin
              ii := (ipivot-1)*ll+1;
              move(au^[ii], row^, ls);
              move(au^[jj], au^[ii], ls);
              move(row^, au^[jj], ls);
              h := t^[ipivot];
             t^[ipivot] := t^[k];
             t^[k] := h;
              h := px^[ipivot];
             px^[ipivot] := px^[k];
             px^[k] := h;
              sumrow^[ipivot] := sumrow^[k]
            End; {ipivot <> k}
          pivot := au^[jj];
         ii := jj;
          For i:=k+1 To ubi Do
            Begin
              ii := ii+ll;
              h := au^[ii]/pivot;
              For j:=0 To ll-2 Do
                au^[ii+j] := au^[ii+j+1]-h*au^[jj+j+1];
              au^[ii+ll-1] := 0;
              t^[i] := t^[i]-h*t^[k];
              px^[i] := px^[i]-h*px^[k];
            End {i}
        End; {maxim <> 0}
        jj := jj+ll
    End; {k}
  If term=1
   Then
    Begin
      normt := 0;
     ubj := -l-1;
      jj := n*ll+1;
      For i:=n Downto 1 Do
        Begin
          jj := jj-ll;
          If ubj<r
           Then
            ubj := ubj+1;
          h := t^[i];
          For j:=1 To ubj+l Do
            h := h-au^[jj+j]*t^[i+j];
          t^[i] := h/au^[jj];
          h := px^[i];
          For j:=1 To ubj+l Do
            h := h-au^[jj+j]*px^[i+j];
          px^[i] := h/au^[jj];
          h := abs(t^[i]);
          If normt<h
           Then
            normt := h
        End; {i}
        ca := normt/normr
    End; {term=1}
  freemem(au, ls*n);
  freemem(sumrow, n*sr);
  freemem(t, n*sr);
  freemem(row, ls)
End; {slegba}

Procedure slegbal(n, l, r: ArbInt;
                  Var a1; Var b1, x1, ca: ArbFloat; Var term:ArbInt);

Var
  sr, i, j, k, ipivot, ubi, ls,
                 ll, ubj, rwidth       : ArbInt;
  normr, sumrowi, pivot, normt, maxim, h  : ArbFloat;
  a                                           : ar2dr1 absolute a1;
  b                                           : arfloat1 absolute b1;
  x                                           : arfloat1 absolute x1;
  au                                          : par2dr1;
  sumrow, t, row                              : ^arfloat1;
Begin
  If (n<1) Or (l<0) Or (r<0) Or (l>n-1) Or (r>n-1)
   Then
    Begin
      term := 3;
     exit
    End; {term=3}
  sr := sizeof(ArbFloat);
 ll := l+r+1;
 ls := ll*sr;
  AllocateAr2dr(n, ll, au);
  getmem(sumrow, n*sr);
 getmem(t, n*sr);
 getmem(row, ls);
  move(b[1], x[1], n*sr);
  For i:=1 To n Do
    Begin
      If i <= l+1 Then
        Begin
          If i <= n-r Then rwidth := r+i
         Else rwidth := n
        End
     Else
          If i <= n-r Then rwidth := ll
     Else rwidth := n-i+l+1;
      move(a[i]^, au^[i]^, rwidth*sr);
      fillchar(au^[i]^[rwidth+1], (ll-rwidth)*sr, 0);
    End; {i}
  normr := 0;
 term := 1;
  For i:=1 To n Do
    Begin
      sumrowi := omvn1v(au^[i]^[1], ll);
     sumrow^[i] := sumrowi;
      h := 2*random-1;
     t^[i] := sumrowi*h;
      h := abs(h);
     If normr<h Then normr := h;
      If sumrowi=0 Then term := 2
    End; {i}
  ubi := l;
 k := 0;
  while (k<n) and (term=1) Do
    Begin
      maxim := 0;
     k := k+1;
     ipivot := k;
      If ubi<n Then ubi := ubi+1;
      For i:=k To ubi Do
        Begin
          sumrowi := sumrow^[i];
          If sumrowi <> 0 Then
            Begin
              h := abs(au^[i]^[1])/sumrowi;
              If maxim<h Then
                Begin
                  maxim := h;
                 ipivot := i
                End {maxim<h}
            End {sumrowi <> 0}
        End; {i}
      If maxim=0 Then term := 2
     Else
        Begin
          If ipivot <> k Then
            Begin
              move(au^[ipivot]^, row^, ls);
              move(au^[k]^, au^[ipivot]^, ls);
              move(row^, au^[k]^, ls);
              h := t^[ipivot];
             t^[ipivot] := t^[k];
             t^[k] := h;
              h := x[ipivot];
             x[ipivot] := x[k];
             x[k] := h;
              sumrow^[ipivot] := sumrow^[k]
            End; {ipivot <> k}
          pivot := au^[k]^[1];
          For i:=k+1 To ubi Do
            Begin
              h := au^[i]^[1]/pivot;
              For j:=0 To ll-2 Do
                au^[i]^[j+1] := au^[i]^[j+2]-h*au^[k]^[j+2];
              au^[i]^[ll] := 0;
              t^[i] := t^[i]-h*t^[k];
              x[i] := x[i]-h*x[k];
            End {i}
        End; {maxim <> 0}
    End; {k}
  If term=1 Then
    Begin
      normt := 0;
     ubj := -l-1;
      For i:=n Downto 1 Do
        Begin
          If ubj<r Then ubj := ubj+1;
          h := t^[i];
          For j:=1 To ubj+l Do
           h := h-au^[i]^[j+1]*t^[i+j];
          t^[i] := h/au^[i]^[1];
          h := x[i];
          For j:=1 To ubj+l Do
           h := h-au^[i]^[j+1]*x[i+j];
          x[i] := h/au^[i]^[1];
          h := abs(t^[i]);
         If normt<h Then normt := h
        End; {i}
        ca := normt/normr
    End; {term=1}
  freemem(sumrow, n*sr);
 freemem(t, n*sr);
 freemem(row, ls);
  DeAllocateAr2dr(n, ll, au);
End; {slegbal}

Procedure slegen(n, rwidth: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Var
          nsr, i, j, k, ipiv, ip, ik, i1n, k1n : ArbInt;
                                      singular : boolean;
           normr, pivot, l, normt, maxim, h, s : ArbFloat;
                pa, px, pb, au, sumrow, t, row : ^arfloat1;

Begin
  If (n<1) Or (rwidth<1)
   Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  getmem(au, sqr(n)*sizeof(ArbFloat));
  nsr := n*sizeof(ArbFloat);
  getmem(t, nsr);
  getmem(row, nsr);
  getmem(sumrow, nsr);
  pa := @a;
 pb := @b;
 px := @x;
  For i:= 1 To n Do
    move(pa^[1+(i-1)*rwidth], au^[1+(i-1)*n], nsr);
  move(pb^[1], px^[1], nsr);
  normr := 0;
 singular := false ;
 i := 0;
 j := 0;
  while (i<n) and  (Not singular) Do
    Begin
      i := i+1;
     sumrow^[i] := omvn1v(au^[1+(i-1)*n], n);
      If sumrow^[i]=0
       Then
        singular := true
      Else
        Begin
          h := 2*random-1;
         t^[i] := sumrow^[i]*h;
         h := abs(h);
          If normr<h
           Then
            normr := h
        End
    End;
  k := 0;
  while (k<n) and  not singular Do
    Begin
      k := k+1;
     maxim := 0;
     ipiv := k;
      For i:=k To n Do
        Begin
          h := abs(au^[k+(i-1)*n])/sumrow^[i];
          If maxim<h
           Then
            Begin
              maxim := h;
             ipiv := i
            End
        End;
      If maxim=0
       Then
        singular := true
      Else
        Begin
          k1n := (k-1)*n;
          If ipiv <> k
           Then
            Begin
              ip := 1+(ipiv-1)*n;
             ik := 1+k1n;
              move(au^[ip], row^[1], nsr);
             move(au^[ik], au^[ip], nsr);
              move(row^[1], au^[ik], nsr);
              h := t^[ipiv];
             t^[ipiv] := t^[k];
             t^[k] := h;
              h := px^[ipiv];
             px^[ipiv] := px^[k];
             px^[k] := h;
              sumrow^[ipiv] := sumrow^[k]
            End;
          pivot := au^[k+k1n];
          For i:=k+1 To n Do
            Begin
              i1n := (i-1)*n;
             l := au^[k+i1n]/pivot;
              If l <> 0
               Then
                Begin
                  For j:=k+1 To n Do
                    au^[j+i1n] := au^[j+i1n]-l*au^[j+k1n];
                  t^[i] := t^[i]-l*t^[k];
                  px^[i] := px^[i]-l*px^[k]
                End
            End
        End
    End;
  If  Not singular
   Then
    Begin
      normt := 0;
      For i:=n Downto 1 Do
        Begin
          s := 0;
         i1n := (i-1)*n;
          For j:=i+1 To n Do
            s := s+t^[j]*au^[j+i1n];
          t^[i] := (t^[i]-s)/au^[i+i1n];
          s := 0;
          For j:=i+1 To n Do
            s := s+px^[j]*au^[j+i1n];
          px^[i] := (px^[i]-s)/au^[i+i1n];
          h := abs(t^[i]);
          If normt<h
           Then
            normt := h
        End;
      ca := normt/normr
    End;
   If singular
    Then
     term := 2
   Else
     term := 1;
  freemem(au, sqr(n)*sizeof(ArbFloat));
  freemem(t, nsr);
  freemem(row, nsr);
  freemem(sumrow, nsr);
End; {slegen}

Procedure slegenl(     n: ArbInt;
                  Var a1;
                  Var b1, x1, ca: ArbFloat;
                  Var term: ArbInt);

Var
     nsr, i, j, k, ipiv : ArbInt;
               singular : boolean;
     normr, pivot, l, normt, maxim, h, s : ArbFloat;
     a : ar2dr1 absolute a1;
     x : arfloat1 absolute x1;
     b : arfloat1 absolute b1;
     au: par2dr1;
     sumrow, t, row : ^arfloat1;
Begin
  If n<1 Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  AllocateAr2dr(n, n, au);
  nsr := n*sizeof(ArbFloat);
  getmem(t, nsr);
  getmem(row, nsr);
  getmem(sumrow, nsr);
  For i:= 1 To n Do
   move(a[i]^, au^[i]^, nsr);
  move(b[1], x[1], nsr);
  normr := 0;
 singular := false ;
 i := 0;
 j := 0;
  while (i<n) and  (Not singular) Do
    Begin
      i := i+1;
     sumrow^[i] := omvn1v(au^[i]^[1], n);
      If sumrow^[i]=0
       Then
        singular := true
      Else
        Begin
          h := 2*random-1;
         t^[i] := sumrow^[i]*h;
         h := abs(h);
          If normr<h
           Then
            normr := h
        End
    End;
  k := 0;
  while (k<n) and  not singular Do
    Begin
      k := k+1;
     maxim := 0;
     ipiv := k;
      For i:=k To n Do
        Begin
          h := abs(au^[i]^[k])/sumrow^[i];
          If maxim<h
           Then
            Begin
              maxim := h;
             ipiv := i
            End
        End;
      If maxim=0
       Then
        singular := true
      Else
        Begin
          If ipiv <> k
           Then
            Begin
              move(au^[ipiv]^, row^, nsr);
              move(au^[k]^, au^[ipiv]^, nsr);
              move(row^, au^[k]^, nsr);
              h := t^[ipiv];
             t^[ipiv] := t^[k];
             t^[k] := h;
              h := x[ipiv];
             x[ipiv] := x[k];
             x[k] := h;
              sumrow^[ipiv] := sumrow^[k]
            End;
          pivot := au^[k]^[k];
          For i:=k+1 To n Do
            Begin
              l := au^[i]^[k]/pivot;
              If l <> 0
               Then
                Begin
                  For j:=k+1 To n Do
                    au^[i]^[j] := au^[i]^[j]-l*au^[k]^[j];
                  t^[i] := t^[i]-l*t^[k];
                  x[i] := x[i]-l*x[k]
                End
            End
        End
    End;
  If  Not singular
   Then
    Begin
      normt := 0;
      For i:=n Downto 1 Do
        Begin
          s := 0;
          For j:=i+1 To n Do
            s := s+t^[j]*au^[i]^[j];
          t^[i] := (t^[i]-s)/au^[i]^[i];
          s := 0;
          For j:=i+1 To n Do
            s := s+x[j]*au^[i]^[j];
          x[i] := (x[i]-s)/au^[i]^[i];
          h := abs(t^[i]);
          If normt<h
           Then
            normt := h
        End;
      ca := normt/normr
    End;
   If singular
    Then
     term := 2
   Else
     term := 1;
  freemem(t, nsr);
  freemem(row, nsr);
  freemem(sumrow, nsr);
  DeAllocateAr2dr(n, n, au);
End; {slegenl}

Procedure slegls(Var a: ArbFloat; m, n, rwidtha: ArbInt; Var b, x: ArbFloat;
                 Var term: ArbInt);

Var     i, j, ns, ms, ii                : ArbInt;
        normy0, norme1, s       : ArbFloat;
        pa, pb, px, qr, alpha, e, y, r  : ^arfloat1;
        pivot                           : ^arint1;
Begin
  If (n<1) Or (m<n)
   Then
    Begin
      term := 3;
     exit
    End;
  pa := @a;
 pb := @b;
 px := @x;
  ns := n*sizeof(ArbFloat);
 ms := m*sizeof(ArbFloat);
  getmem(qr, m*ns);
 getmem(alpha, ns);
 getmem(e, ns);
 getmem(y, ns);
  getmem(r, m*sizeof(ArbFloat));
 getmem(pivot, n*sizeof(ArbInt));
  For i:=1 To m Do
    move(pa^[(i-1)*rwidtha+1], qr^[(i-1)*n+1], ns);
  decomp(qr^[1], m, n, n, alpha^[1], pivot^[1], term);
  If term=2
   Then
    Begin
      freemem(qr, m*ns);
     freemem(alpha, ns);
     freemem(e, ns);
     freemem(y, ns);
      freemem(r, m*sizeof(ArbFloat));
     freemem(pivot, n*sizeof(ArbInt));
      exit
    End;
  move(pb^[1], r^[1], ms);
  solve(qr^[1], m, n, n, alpha^[1], pivot^[1], r^[1], y^[1]);
  For i:=1 To m Do
    Begin
      s := pb^[i];
     ii := (i-1)*rwidtha;
      For j:=1 To n Do
        s := s-pa^[ii+j]*y^[j];
      r^[i] := s
    End; {i}
  solve(qr^[1], m, n, n, alpha^[1], pivot^[1], r^[1], e^[1]);
  normy0 := 0;
 norme1 := 0;
  For i:=1 To n Do
    Begin
      normy0 := normy0+sqr(y^[i]);
     norme1 := norme1+sqr(e^[i])
    End; {i}
  If norme1 > 0.0625*normy0
   Then
    Begin
      term := 2;
      freemem(qr, m*ns);
     freemem(alpha, ns);
     freemem(e, ns);
     freemem(y, ns);
      freemem(r, m*sizeof(ArbFloat));
     freemem(pivot, n*sizeof(ArbInt));
      exit
    End;
  For i:=1 To n Do
    px^[i] := y^[i];
  freemem(qr, m*ns);
 freemem(alpha, ns);
 freemem(e, ns);
 freemem(y, ns);
  freemem(r, m*sizeof(ArbFloat));
 freemem(pivot, n*sizeof(ArbInt));
End; {slegls}

Procedure sleglsl(Var a1; m, n: ArbInt; Var b1, x1: ArbFloat;
                  Var term: ArbInt);

Var     i, j, ns, ms                    : ArbInt;
        normy0, norme1, s       : ArbFloat;
        a                               : ar2dr1 absolute a1;
        b                               : arfloat1 absolute b1;
        x                               : arfloat1 absolute x1;
        alpha, e, y, r                  : ^arfloat1;
        qr                              : par2dr1;
        pivot                           : ^arint1;
Begin
  If (n<1) Or (m<n)
   Then
    Begin
      term := 3;
     exit
    End;
  AllocateAr2dr(m, n, qr);
  ns := n*sizeof(ArbFloat);
 ms := m*sizeof(ArbFloat);
  getmem(alpha, ns);
 getmem(e, ns);
 getmem(y, ns);
  getmem(r, ms);
 getmem(pivot, n*sizeof(ArbInt));
  For i:=1 To m Do
    move(a[i]^, qr^[i]^, ns);
  decomp1(qr^[1], m, n, alpha^[1], pivot^[1], term);
  If term=2
   Then
    Begin
      freemem(qr, m*ns);
     freemem(alpha, ns);
     freemem(e, ns);
     freemem(y, ns);
      freemem(r, ms);
     freemem(pivot, n*sizeof(ArbInt));
      exit
    End;
  move(b[1], r^, ms);
  solve1(qr^[1], m, n, alpha^[1], pivot^[1], r^[1], y^[1]);
  For i:=1 To m Do
    Begin
      s := b[i];
      For j:=1 To n Do
       s := s-a[i]^[j]*y^[j];
      r^[i] := s
    End; {i}
  solve1(qr^[1], m, n, alpha^[1], pivot^[1], r^[1], e^[1]);
  normy0 := 0;
 norme1 := 0;
  For i:=1 To n Do
    Begin
      normy0 := normy0+sqr(y^[i]);
     norme1 := norme1+sqr(e^[i])
    End; {i}
  If norme1 > 0.0625*normy0
   Then
    Begin
      term := 2;
      freemem(qr, m*ns);
     freemem(alpha, ns);
     freemem(e, ns);
     freemem(y, ns);
      freemem(r, m*sizeof(ArbFloat));
     freemem(pivot, n*sizeof(ArbInt));
      exit
    End;
  For i:=1 To n Do
   x[i] := y^[i];
  freemem(alpha, ns);
 freemem(e, ns);
 freemem(y, ns);
  freemem(r, ms);
 freemem(pivot, n*sizeof(ArbInt));
  DeAllocateAr2dr(m, n, qr);
End; {sleglsl}

Procedure slegpb(n, l: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Var
    posdef                                                 : boolean;
    i, j, k, r, p, q, jmin1, ii, jj, ri, ind,
                                      ll, llm1, sr, rwidth : ArbInt;
    h, normr, normt, sumrowi, hh, alim, norma              : ArbFloat;
    pa, pb, px, al, t, v                                   : ^arfloat1;

    Procedure decomp(i, r: ArbInt);

    Var k: ArbInt;
    Begin
      ri := (r-1)*ll;
      h := al^[ii+j];
     q := ll-j+p;
      For k:=p To jmin1 Do
        Begin
          h := h-al^[ii+k]*al^[ri+q];
         q := q+1
        End ;
      If j<ll
       Then
        al^[ii+j] := h/al^[ri+ll];
    End; {decomp}

Begin
  If (n<1) Or (l<0) Or (l>n-1)
   Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  sr := sizeof(ArbFloat);
  pa := @a;
  pb := @b;
  px := @x;
  ll := l+1;
  getmem(al, ll*n*sr);
  getmem(t, n*sr);
  getmem(v, ll*sr);
  move(pb^, px^, n*sr);
  jj := 1;
  ii := 1;
  For i:=1 To n Do
    Begin
      If i>l Then rwidth := ll
     Else rwidth := i;
      move(pa^[jj], al^[ii+ll-rwidth], rwidth*sr);
      jj := jj+rwidth;
     ii := ii+ll
    End; {i}
  normr := 0;
 p := ll+1;
 norma := 0;
  For i:=1 To n Do
    Begin
      If p>1
       Then
        p := p-1;
      For j:=p To ll Do
        v^[j] := al^[j+(i-1)*ll];
      sumrowi := omvn1v(v^[p], ll-p+1);
      r := i;
     j := ll;
      while (r<n) and (j>1) Do
        Begin
          r := r+1;
         j := j-1;
          sumrowi := sumrowi+abs(al^[j+(r-1)*ll])
        End; {r,j}
      If norma<sumrowi
       Then
        norma := sumrowi;
      h := 2*random-1;
     t^[i] := h;
      h := abs(h);
      If normr<h
       Then
        normr := h
    End; {i}
  llm1 := ll-1;
 p := ll+1;
 i := 0;
 posdef := true ;
  while (i<n) and posdef Do
    Begin
      i := i+1;
     If p>1 Then p := p-1;
     r := i-ll+p;
     j := p-1;
      ii := (i-1)*ll;
      while j<llm1 Do
        Begin
          jmin1 := j;
         j := j+1;
          decomp(i, r);
          r := r+1
        End ; {j}
      jmin1 := llm1;
     j := ll;
      decomp(i, i);
      If h <= 0
       Then
        posdef := false
      Else
        Begin
          alim := sqrt(h);
         al^[ii+ll] := alim;
          h := t^[i];
         q := i;
          For k:=llm1 Downto p Do
            Begin
              q := q-1;
             h := h-al^[ii+k]*t^[q]
            End ;
          t^[i] := h/alim;
          h := px^[i];
         q := i;
          For k:=llm1 Downto p Do
            Begin
              q := q-1;
             h := h-al^[ii+k]*px^[q]
            End; {k}
          px^[i] := h/alim
        End {posdef}
    End; {i}
    If posdef
     Then
      Begin
        normt := 0;
       p := ll+1;
        For i:=n Downto 1 Do
          Begin
            If p>1
             Then
              p := p-1;
            q := i;
           h := t^[i];
           hh := px^[i];
            For k:=llm1 Downto p Do
              Begin
                q := q+1;
                ind := (q-1)*ll+k;
                h := h-al^[ind]*t^[q];
               hh := hh-al^[ind]*px^[q]
              End; {k}
            ind := i*ll;
            t^[i] := h/al^[ind];
           px^[i] := hh/al^[ind];
            h := abs(t^[i]);
            If normt<h
             Then
              normt := h
         End; {i}
       ca := norma*normt/normr
     End ; {posdef}
  If posdef
   Then
    term := 1
  Else
    term := 2;
  freemem(al, ll*n*sr);
  freemem(t, n*sr);
  freemem(v, ll*sr);
End;  {slegpb}

Procedure slegpbl(n, l: ArbInt;
                  Var a1; Var b1, x1, ca: ArbFloat; Var term: ArbInt);

Var
    posdef                                    : boolean;
    i, j, k, r, p, q, ll, sr, rwidth          : ArbInt;
    h, normr, normt, sumrowi, hh, alim, norma : ArbFloat;
    a                                         : ar2dr1 absolute a1;
    b                                         : arfloat1 absolute b1;
    x                                         : arfloat1 absolute x1;
    al                                        : par2dr1;
    t, v                                      : ^arfloat1;

    Procedure decomp(r: ArbInt);

    Var k: ArbInt;
    Begin
      h := al^[i]^[j];
     q := ll-j+p;
      For k:=p To j-1 Do
        Begin
          h := h-al^[i]^[k]*al^[r]^[q];
         Inc(q)
        End ;
      If j<ll Then al^[i]^[j] := h/al^[r]^[ll];
    End; {decomp}

Begin
  If (n<1) Or (l<0) Or (l>n-1)
   Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  sr := sizeof(ArbFloat);
  ll := l+1;
  AllocateAr2dr(n, ll, al);
  getmem(t, n*sr);
 getmem(v, ll*sr);
  move(b[1], x[1], n*sr);
  For i:=1 To n Do
    Begin
      If i>l Then rwidth := ll
     Else rwidth := i;
      move(a[i]^, al^[i]^[ll-rwidth+1], rwidth*sr);
    End; {i}
  normr := 0;
 p := ll+1;
 norma := 0;
  For i:=1 To n Do
    Begin
      If p>1 Then Dec(p);
      For j:=p To ll Do
       v^[j] := al^[i]^[j];
      sumrowi := omvn1v(v^[p], ll-p+1);
      r := i;
     j := ll;
      while (r<n) and (j>1) Do
        Begin
          Inc(r);
         Dec(j);
          sumrowi := sumrowi+abs(al^[r]^[j])
        End; {r,j}
      If norma<sumrowi Then norma := sumrowi;
      h := 2*random-1;
     t^[i] := h;
      h := abs(h);
     If normr<h Then normr := h
    End; {i}
  p := ll+1;
 i := 0;
 posdef := true ;
  while (i<n) and posdef Do
    Begin
      Inc(i);
     If p>1 Then Dec(p);
     r := i-ll+p;
     j := p-1;
      while j<ll-1 Do
        Begin
          Inc(j);
         decomp(r);
         Inc(r)
        End ; {j}
      j := ll;
     decomp(i);
      If h <= 0 Then posdef := false
     Else
        Begin
          alim := sqrt(h);
         al^[i]^[ll] := alim;
          h := t^[i];
         q := i;
          For k:=ll-1 Downto p Do
            Begin
              q := q-1;
             h := h-al^[i]^[k]*t^[q]
            End ;
          t^[i] := h/alim;
          h := x[i];
         q := i;
          For k:=ll-1 Downto p Do
            Begin
              q := q-1;
             h := h-al^[i]^[k]*x[q]
            End; {k}
          x[i] := h/alim
        End {posdef}
    End; {i}
    If posdef
     Then
      Begin
        normt := 0;
       p := ll+1;
        For i:=n Downto 1 Do
          Begin
            If p>1 Then Dec(p);
            q := i;
           h := t^[i];
           hh := x[i];
            For k:=ll-1 Downto p Do
              Begin
                Inc(q);
                h := h-al^[q]^[k]*t^[q];
               hh := hh-al^[q]^[k]*x[q]
              End; {k}
            t^[i] := h/al^[i]^[ll];
           x[i] := hh/al^[i]^[ll];
            h := abs(t^[i]);
           If normt<h Then normt := h
         End; {i}
       ca := norma*normt/normr
     End ; {posdef}
  If posdef Then term := 1
 Else term := 2;
  freemem(t, n*sr);
 freemem(v, ll*sr);
  DeAllocateAr2dr(n, ll, al);
End;  {slegpbl}

Procedure slegpd(n, rwidth: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Var
    sr, i, j, k, kmin1, kk, k1n, i1n, ik, ii : ArbInt;
                                          pd : boolean;
        h, lkk, normr, normt, sumrowi, norma : ArbFloat;
                           pa, pb, px, al, t : ^arfloat1;

Begin
  If (n<1) Or (rwidth<1)
   Then
    Begin
      term := 3;
     exit
    End;
  sr := sizeof(ArbFloat);
  getmem(al, sqr(n)*sr);
 getmem(t, n*sr);
  pa := @a;
 pb := @b;
 px := @x;
  For i:=1 To n Do
    move(pa^[1+(i-1)*rwidth], al^[1+(i-1)*n], i*sr);
  move(pb^[1], px^[1], n*sr);
  normr := 0;
 pd := true ;
 norma := 0;
  For i:=1 To n Do
    Begin
      sumrowi := 0;
      For j:=1 To i Do
        sumrowi := sumrowi+abs(al^[j+(i-1)*n]);
      For j:=i+1 To n Do
        sumrowi := sumrowi+abs(al^[i+(j-1)*n]);
      If norma<sumrowi
       Then
        norma := sumrowi;
      t^[i] := 2*random-1;
     h := abs(t^[i]);
      If normr<h
       Then
        normr := h
    End; {i}
  k := 0;
  while (k<n) and pd Do
    Begin
      kmin1 := k;
     k := k+1;
     k1n := (k-1)*n;
     kk := k+k1n;
     lkk := al^[kk];
      For j:=1 To kmin1 Do
        lkk := lkk-sqr(al^[j+k1n]);
      If lkk<=0
       Then
        pd := false
      Else
        Begin
          al^[kk] := sqrt(lkk);
         lkk := al^[kk];
          For i:=k+1 To n Do
            Begin
              i1n := (i-1)*n;
             ik := k+i1n;
             h := al^[ik];
              For j:=1 To kmin1 Do
                h := h-al^[j+k1n]*al^[j+i1n];
              al^[ik] := h/lkk
            End; {i}
          h := t^[k];
          For j:=1 To kmin1 Do
            h := h-al^[j+k1n]*t^[j];
          t^[k] := h/lkk;
          h := px^[k];
          For j:=1 To kmin1 Do
            h := h-al^[j+k1n]*px^[j];
          px^[k] := h/lkk
        End {lkk > 0}
    End; {k}
    If pd
     Then
      Begin
        normt := 0;
        For i:=n Downto 1 Do
          Begin
            ii := i+(i-1)*n;
           h := t^[i];
            For j:=i+1 To n Do
              h := h-al^[i+(j-1)*n]*t^[j];
            t^[i] := h/al^[ii];
            h := px^[i];
            For j:=i+1 To n Do
              h := h-al^[i+(j-1)*n]*px^[j];
            px^[i] := h/al^[ii];
            h := abs(t^[i]);
            If normt<h
              Then
                normt := h
          End; {i}
        ca := norma*normt/normr
      End; {pd}
  If pd
   Then
    term := 1
  Else
    term := 2;
  freemem(al, sqr(n)*sr);
 freemem(t, n*sr);
End; {slegpd}

Procedure slegpdl(n: ArbInt; Var a1; Var b1, x1, ca: ArbFloat;
                  Var term: ArbInt);

Var                   sr, i, j, k, kmin1 : ArbInt;
                                      pd : boolean;
    h, lkk, normr, normt, sumrowi, norma : ArbFloat;
                                       a : ar2dr1 absolute a1;
                                       b : arfloat1 absolute b1;
                                       x : arfloat1 absolute x1;
                                      al : par2dr1;
                                       t : ^arfloat1;

Begin
  If n<1 Then
    Begin
      term := 3;
     exit
    End;
  sr := sizeof(ArbFloat);
  AllocateL2dr(n, al);
 getmem(t, n*sr);
  For i:=1 To n Do
   move(a[i]^, al^[i]^, i*sr);
  move(b[1], x[1], n*sr);
  normr := 0;
 pd := true ;
 norma := 0;
  For i:=1 To n Do
    Begin
      sumrowi := 0;
      For j:=1 To i Do
       sumrowi := sumrowi+abs(al^[i]^[j]);
      For j:=i+1 To n Do
       sumrowi := sumrowi+abs(al^[j]^[i]);
      If norma<sumrowi Then norma := sumrowi;
      t^[i] := 2*random-1;
     h := abs(t^[i]);
      If normr<h Then normr := h
    End; {i}
  k := 0;
  while (k<n) and pd Do
    Begin
      kmin1 := k;
     k := k+1;
     lkk := al^[k]^[k];
      For j:=1 To kmin1 Do
       lkk := lkk-sqr(al^[k]^[j]);
      If lkk<=0 Then pd := false
     Else
        Begin
          al^[k]^[k] := sqrt(lkk);
         lkk := al^[k]^[k];
          For i:=k+1 To n Do
            Begin
              h := al^[i]^[k];
              For j:=1 To kmin1 Do
               h := h-al^[k]^[j]*al^[i]^[j];
              al^[i]^[k] := h/lkk
            End; {i}
          h := t^[k];
          For j:=1 To kmin1 Do
           h := h-al^[k]^[j]*t^[j];
          t^[k] := h/lkk;
         h := x[k];
          For j:=1 To kmin1 Do
           h := h-al^[k]^[j]*x[j];
          x[k] := h/lkk
        End {lkk > 0}
    End; {k}
    If pd Then
      Begin
        normt := 0;
        For i:=n Downto 1 Do
          Begin
            h := t^[i];
            For j:=i+1 To n Do
             h := h-al^[j]^[i]*t^[j];
            t^[i] := h/al^[i]^[i];
            h := x[i];
            For j:=i+1 To n Do
             h := h-al^[j]^[i]*x[j];
            x[i] := h/al^[i]^[i];
           h := abs(t^[i]);
            If normt<h Then normt := h
          End; {i}
        ca := norma*normt/normr
      End; {pd}
  If pd Then term := 1
 Else term := 2;
  DeAllocateL2dr(n, al);
 freemem(t, n*sr);
End; {slegpdl}

Procedure slegsy(n, rwidth: ArbInt; Var a, b, x, ca: ArbFloat;
                 Var term:ArbInt);

Var
   i, j, kmin1, k, kplus1, kmin2, nsr, nsi, nsb,
   imin1, jmin1, indexpivot, iplus1, indi, indj, indk, indp       : ArbInt;
   h, absh, maxim, pivot, ct, norma, sumrowi, normt, normr, s : ArbFloat;
              pa, pb, pb1, px, alt, l, d, t, u, v, l1, d1, u1, t1 : ^arfloat1;
                                                                p : ^arint1;
                                                                q : ^arbool1;
Begin
  If (n<1) Or (rwidth<1)
   Then
    Begin
      term := 3;
     exit
    End; {if}
  pa := @a;
 pb := @b;
 px := @x;
  nsr := n*sizeof(ArbFloat);
  nsi := n*sizeof(ArbInt);
  nsb := n*sizeof(boolean);
  getmem(alt, n*nsr);
  getmem(l, nsr);
  getmem(d, nsr);
  getmem(t, nsr);
  getmem(u, nsr);
  getmem(v, nsr);
  getmem(p, nsi);
  getmem(q, nsb);
  getmem(l1, nsr);
  getmem(d1, nsr);
  getmem(u1, nsr);
  getmem(t1, nsr);
  getmem(pb1, nsr);
  move(pb^, pb1^, nsr);
  For i:=1 To n Do
    Begin
      indi := (i-1)*n;
      For j:=1 To i Do
        alt^[indi+j] := pa^[(i-1)*rwidth+j];
    End; {i}
  norma := 0;
  For i:=1 To n Do
    Begin
      indi := (i-1)*n;
      p^[i] := i;
     sumrowi := 0;
      For j:=1 To i Do
        sumrowi := sumrowi+abs(alt^[indi+j]);
      For j:=i+1 To n Do
        sumrowi := sumrowi+abs(alt^[(j-1)*n+i]);
      If norma<sumrowi
       Then
        norma := sumrowi
    End; {i}
  kmin1 := -1;
 k := 0;
 kplus1 := 1;
  while k<n Do
    Begin
      kmin2 := kmin1;
     kmin1 := k;
     k := kplus1;
     kplus1 := kplus1+1;
      indk := kmin1*n;
      If k>3
       Then
        Begin
          t^[2] := alt^[n+2]*alt^[indk+1]+alt^[2*n+2]*alt^[indk+2];
          For i:=3 To kmin2 Do
            Begin
              indi := (i-1)*n;
              t^[i] := alt^[indi+i-1]*alt^[indk+i-2]+alt^[indi+i]
                       *alt^[indk+i-1]+alt^[indi+n+i]*alt^[indk+i]
            End; {i}
          t^[kmin1] := alt^[indk-n+kmin2]*alt^[indk+k-3]
                       +alt^[indk-n+kmin1]*alt^[indk+kmin2]
                       +alt^[indk+kmin1];
          h := alt^[indk+k];
          For j:=2 To kmin1 Do
            h := h-t^[j]*alt^[indk+j-1];
          t^[k] := h;
          alt^[indk+k] := h-alt^[indk+kmin1]*alt^[indk+kmin2]
        End {k>3}
      Else
       If k=3
        Then
        Begin
          t^[2] := alt^[n+2]*alt^[2*n+1]+alt^[2*n+2];
          h := alt^[2*n+3]-t^[2]*alt^[2*n+1];
          t^[3] := h;
          alt^[2*n+3] := h-alt^[2*n+2]*alt^[2*n+1]
        End  {k=3}
      Else
       If k=2
        Then
        t^[2] := alt^[n+2];
      maxim := 0;
      For i:=kplus1 To n Do
        Begin
          indi := (i-1)*n;
          h := alt^[indi+k];
          For j:=2 To k Do
            h := h-t^[j]*alt^[indi+j-1];
          absh := abs(h);
          If maxim<absh
           Then
            Begin
              maxim := absh;
             indexpivot := i
            End; {if}
          alt^[indi+k] := h
        End; {i}
      If maxim <> 0
       Then
        Begin
          If indexpivot>kplus1
           Then
            Begin
              indp := (indexpivot-1)*n;
              indk := k*n;
              p^[kplus1] := indexpivot;
              For j:=1 To k Do
                Begin
                  h := alt^[indk+j];
                  alt^[indk+j] := alt^[indp+j];
                  alt^[indp+j] := h
                End; {j}
              For j:=indexpivot Downto kplus1 Do
                Begin
                  indj := (j-1)*n;
                  h := alt^[indj+kplus1];
                  alt^[indj+kplus1] := alt^[indp+j];
                  alt^[indp+j] := h
                End; {j}
              For i:=indexpivot To n Do
                Begin
                  indi := (i-1)*n;
                  h := alt^[indi+kplus1];
                  alt^[indi+kplus1] := alt^[indi+indexpivot];
                  alt^[indi+indexpivot] := h
                End  {i}
            End; {if}
          pivot := alt^[k*n+k];
          For i:=k+2 To n Do
            alt^[(i-1)*n+k] := alt^[(i-1)*n+k]/pivot
        End {maxim <> 0}
    End; {k}
  d^[1] := alt^[1];
 i := 1;
  while i<n Do
    Begin
      imin1 := i;
     i := i+1;
      u^[imin1] := alt^[(i-1)*n+imin1];
      l^[imin1] := u^[imin1];
     d^[i] := alt^[(i-1)*n+i]
    End; {i}
  mdtgtr(n, l^[1], d^[1], u^[1], l1^[1], d1^[1], u1^[1], v^[1],
         q^[1], ct, term);
  If term=1
   Then
    Begin
      normr := 0;
      For i:=1 To n Do
        Begin
          t^[i] := 2*random-1;
         h := t^[i];
          h := abs(h);
          If normr<h
           Then
            normr := h
        End; {i}
      For i:=1 To n Do
        Begin
          indexpivot := p^[i];
          If indexpivot <> i
           Then
            Begin
              h := pb1^[i];
             pb1^[i] := pb1^[indexpivot];
              pb1^[indexpivot] := h
            End {if}
        End; {i}
      i := 0;
      while i<n Do
        Begin
          indi := i*n;
          imin1 := i;
         i := i+1;
         j := 1;
         h := t^[i];
         s := pb1^[i];
          while j<imin1 Do
            Begin
              jmin1 := j;
             j := j+1;
              s := s-alt^[indi+jmin1]*pb1^[j];
              h := h-alt^[indi+jmin1]*t^[j]
            End; {j}
          t^[i] := h;
         pb1^[i] := s
        End; {i}
      dslgtr(n, l1^[1], d1^[1], u1^[1], v^[1], q^[1], pb1^[1], px^[1], term);
      dslgtr(n, l1^[1], d1^[1], u1^[1], v^[1], q^[1], t^[1], t1^[1], term);
      i := n+1;
     imin1 := n;
     normt := 0;
      while i>2 Do
        Begin
          iplus1 := i;
         i := imin1;
         imin1 := imin1-1;
          h := t1^[i];
         s := px^[i];
          For j:=iplus1 To n Do
            Begin
              indj := (j-1)*n+imin1;
              h := h-alt^[indj]*t1^[j];
              s := s-alt^[indj]*px^[j]
            End; {j}
          px^[i] := s;
          t1^[i] := h;
         h := abs(h);
          If normt<h
           Then
            normt := h
        End; {i}
      For i:=n Downto 1 Do
        Begin
          indexpivot := p^[i];
          If indexpivot <> i
           Then
            Begin
              h := px^[i];
             px^[i] := px^[indexpivot];
              px^[indexpivot] := h
            End {if}
        End; {i}
      ca := norma*normt/normr
    End {term=1}
  Else
    term := 2;
  freemem(alt, n*nsr);
  freemem(l, nsr);
  freemem(d, nsr);
  freemem(t, nsr);
  freemem(u, nsr);
  freemem(v, nsr);
  freemem(p, nsi);
  freemem(q, nsb);
  freemem(l1, nsr);
  freemem(d1, nsr);
  freemem(u1, nsr);
  freemem(t1, nsr);
  freemem(pb1, nsr);
End; {slegsy}

Procedure slegsyl(n: ArbInt; Var a1; Var b1, x1, ca: ArbFloat;
                  Var term: ArbInt);

Var
   i, j, k, nsr, nsi, nsb, indexpivot: ArbInt;
   h, absh, maxim, pivot, ct, norma, sumrowi, normt, normr, s : ArbFloat;
                                           a : ar2dr1 absolute a1;
                                           b : arfloat1 absolute b1;
                                           x : arfloat1 absolute x1;
           b0, l, d, t, u, v, l1, d1, u1, t1 : ^arfloat1;
                                         alt : par2dr1;
                                           p : ^arint1;
                                           q : ^arbool1;
Begin
  If n<1 Then
    Begin
      term := 3;
     exit
    End; {if}
  nsr := n*sizeof(ArbFloat);
 nsi := n*sizeof(ArbInt);
 nsb := n*sizeof(boolean);
  AllocateL2dr(n, alt);
  getmem(l, nsr);
 getmem(d, nsr);
 getmem(t, nsr);
  getmem(u, nsr);
 getmem(v, nsr);
 getmem(p, nsi);
  getmem(q, nsb);
 getmem(l1, nsr);
 getmem(d1, nsr);
  getmem(u1, nsr);
 getmem(t1, nsr);
 getmem(b0, nsr);
  move(b[1], b0^, nsr);
  For i:=1 To n Do
   move(a[i]^, alt^[i]^, i*sizeof(ArbFloat));
  norma := 0;
  For i:=1 To n Do
    Begin
      p^[i] := i;
     sumrowi := 0;
      For j:=1 To i Do
       sumrowi := sumrowi+abs(alt^[i]^[j]);
      For j:=i+1 To n Do
       sumrowi := sumrowi+abs(alt^[j]^[i]);
      If norma<sumrowi Then norma := sumrowi
    End; {i}
  k := 0;
  while k<n Do
    Begin
      Inc(k);
      If k>3 Then
        Begin
          t^[2] := alt^[2]^[2]*alt^[k]^[1]+alt^[3]^[2]*alt^[k]^[2];
          For i:=3 To k-2 Do
            t^[i] := alt^[i]^[i-1]*alt^[k]^[i-2]+alt^[i]^[i]
                     *alt^[k]^[i-1]+alt^[i+1]^[i]*alt^[k]^[i];
          t^[k-1] := alt^[k-1]^[k-2]*alt^[k]^[k-3]
                     +alt^[k-1]^[k-1]*alt^[k]^[k-2]+alt^[k]^[k-1];
          h := alt^[k]^[k];
          For j:=2 To k-1 Do
           h := h-t^[j]*alt^[k]^[j-1];
          t^[k] := h;
          alt^[k]^[k] := h-alt^[k]^[k-1]*alt^[k]^[k-2]
        End {k>3}
      Else
       If k=3
        Then
        Begin
          t^[2] := alt^[2]^[2]*alt^[3]^[1]+alt^[3]^[2];
          h := alt^[3]^[3]-t^[2]*alt^[3]^[1];
          t^[3] := h;
          alt^[3]^[3] := h-alt^[3]^[2]*alt^[3]^[1]
        End  {k=3}
      Else
       If k=2 Then t^[2] := alt^[2]^[2];
      maxim := 0;
      For i:=k+1 To n Do
        Begin
          h := alt^[i]^[k];
          For j:=2 To k Do
           h := h-t^[j]*alt^[i]^[j-1];
          absh := abs(h);
          If maxim<absh Then
            Begin
              maxim := absh;
             indexpivot := i
            End; {if}
          alt^[i]^[k] := h
        End; {i}
      If maxim <> 0
       Then
        Begin
          If indexpivot>k+1 Then
            Begin
              p^[k+1] := indexpivot;
              For j:=1 To k Do
                Begin
                  h := alt^[k+1]^[j];
                  alt^[k+1]^[j] := alt^[indexpivot]^[j];
                  alt^[indexpivot]^[j] := h
                End; {j}
              For j:=indexpivot Downto k+1 Do
                Begin
                  h := alt^[j]^[k+1];
                  alt^[j]^[k+1] := alt^[indexpivot]^[j];
                  alt^[indexpivot]^[j] := h
                End; {j}
              For i:=indexpivot To n Do
                Begin
                  h := alt^[i]^[k+1];
                  alt^[i]^[k+1] := alt^[i]^[indexpivot];
                  alt^[i]^[indexpivot] := h
                End  {i}
            End; {if}
          pivot := alt^[k+1]^[k];
          For i:=k+2 To n Do
           alt^[i]^[k] := alt^[i]^[k]/pivot
        End {maxim <> 0}
    End; {k}
  d^[1] := alt^[1]^[1];
 i := 1;
  while i<n Do
    Begin
      Inc(i);
      u^[i-1] := alt^[i]^[i-1];
      l^[i-1] := u^[i-1];
     d^[i] := alt^[i]^[i]
    End; {i}
  mdtgtr(n, l^[1], d^[1], u^[1], l1^[1], d1^[1], u1^[1], v^[1],
         q^[1], ct, term);
  If term=1 Then
    Begin
      normr := 0;
      For i:=1 To n Do
        Begin
          t^[i] := 2*random-1;
         h := t^[i];
          h := abs(h);
          If normr<h Then normr := h
        End; {i}
      For i:=1 To n Do
        Begin
          indexpivot := p^[i];
          If indexpivot <> i
           Then
            Begin
              h := b0^[i];
             b0^[i] := b0^[indexpivot];
              b0^[indexpivot] := h
            End {if}
        End; {i}
      i := 0;
      while i<n Do
        Begin
          Inc(i);
         j := 1;
         h := t^[i];
         s := b0^[i];
          while j<i-1 Do
            Begin
              Inc(j);
              s := s-alt^[i]^[j-1]*b0^[j];
              h := h-alt^[i]^[j-1]*t^[j]
            End; {j}
          t^[i] := h;
         b0^[i] := s
        End; {i}
      dslgtr(n, l1^[1], d1^[1], u1^[1], v^[1], q^[1], b0^[1], x[1], term);
      dslgtr(n, l1^[1], d1^[1], u1^[1], v^[1], q^[1], t^[1], t1^[1], term);
      i := n+1;
     normt := 0;
      while i>2 Do
        Begin
          Dec(i);
          h := t1^[i];
         s := x[i];
          For j:=i+1 To n Do
            Begin
              h := h-alt^[j]^[i-1]*t1^[j];
              s := s-alt^[j]^[i-1]*x[j]
            End; {j}
          x[i] := s;
         t1^[i] := h;
         h := abs(h);
          If normt<h Then normt := h
        End; {i}
      For i:=n Downto 1 Do
        Begin
          indexpivot := p^[i];
          If indexpivot <> i Then
            Begin
              h := x[i];
             x[i] := x[indexpivot];
             x[indexpivot] := h
            End {if}
        End; {i}
      ca := norma*normt/normr
    End {term=1}
  Else
    term := 2;
  freemem(l, nsr);
 freemem(d, nsr);
 freemem(t, nsr);
  freemem(u, nsr);
 freemem(v, nsr);
 freemem(p, nsi);
  freemem(q, nsb);
 freemem(l1, nsr);
 freemem(d1, nsr);
  freemem(u1, nsr);
 freemem(t1, nsr);
 freemem(b0, nsr);
  DeAllocateL2dr(n, alt);
End; {slegsyl}

Procedure slegtr(n:ArbInt; Var l, d, u, b, x, ca: ArbFloat;
                 Var term: ArbInt);

Var                           singular, ch : boolean;
               i, j, nm1, sr, n1s, ns, n2s : ArbInt;
            normr, normt, h, lj, di, ui, m : ArbFloat;
                                    pl, ll : ^arfloat2;
    pd, pu, pb, px, dd, uu1, u2, t, sumrow : ^arfloat1;
Begin
  If n<1
   Then
    Begin
      term := 3;
     exit
    End; {n<1}
  sr := sizeof(ArbFloat);
 n1s := (n-1)*sr;
 ns := n1s+sr;
 n2s := n1s;
  getmem(ll, n1s);
  getmem(uu1, n1s);
  getmem(u2, n2s);
  getmem(dd, ns);
  getmem(t, ns);
  getmem(sumrow, ns);

  pl := @l;
 pd := @d;
 pu := @u;
 pb := @b;
 px := @x;
  move(pl^[2], ll^[2], n1s);
  move(pd^[1], dd^[1], ns);
  If n>1
   Then
    move(pu^[1], uu1^[1], n1s);
  move(pb^[1], px^[1], ns);
  normr := 0;
 singular := false;
  nm1 := n-1;
 i := 0;
  while (i<n) and not singular Do
    Begin
      i := i+1;
      If i=1
       Then
        Begin
          sumrow^[i] := abs(dd^[1]);
          If n>1
           Then
            sumrow^[i] := sumrow^[i]+abs(uu1^[1])
        End {i=1}
      Else
        If i=n
         Then
          sumrow^[i] := abs(ll^[n])+abs(dd^[n])
        Else
          sumrow^[i] := abs(ll^[i])+abs(dd^[i])+abs(uu1^[i]);
      If sumrow^[i]=0
       Then
        singular := true
      Else
        Begin
          h := 2*random-1;
         t^[i] := sumrow^[i]*h;
          h := abs(h);
          If normr<h
           Then
            normr := h
        End {sumrow^[i] <> 0}
    End; {i}
  j := 1;
  while (j <> n) and  not singular Do
    Begin
      i := j;
     j := j+1;
     lj := ll^[j];
      If lj <> 0
       Then
        Begin
          di := dd^[i];
          ch := abs(di/sumrow^[i])<abs(lj/sumrow^[j]);
          If ch
           Then
            Begin
              ui := uu1^[i];
             dd^[i] := lj;
             uu1^[i] := dd^[j];
             m := di/lj;
              dd^[j] := ui-m*dd^[j];
              If i<nm1
               Then
                Begin
                  u2^[i] := uu1^[j];
                 uu1^[j] := -m*u2^[i]
                End; {i<nm1}
              sumrow^[j] := sumrow^[i];
              h := t^[i];
             t^[i] := t^[j];
             t^[j] := h-m*t^[i];
             h := px^[i];
              px^[i] := px^[j];
             px^[j] := h-m*px^[i]
            End {ch}
          Else
            Begin
              m := lj/di;
             dd^[j] := dd^[j]-m*uu1^[i];
              If i<nm1
               Then
                u2^[i] := 0;
              t^[j] := t^[j]-m*t^[i];
             px^[j] := px^[j]-m*px^[i]
            End {not ch}
        End {lj <> 0}
      Else
        Begin
          If i < nm1
            Then
              u2^[i] := 0;
          If dd^[i]=0
           Then
            singular := true
        End {lj=0}
    End; {j}
  If dd^[n]=0
   Then
    singular := true;
  If  Not singular
    Then
      Begin
        normt := 0;
       t^[n] := t^[n]/dd^[n];
       px^[n] := px^[n]/dd^[n];
       h := abs(t^[n]);
        If normt<h
         Then
          normt := h;
        If n>1
         Then
          Begin
            t^[nm1] := (t^[nm1]-uu1^[nm1]*t^[n])/dd^[nm1];
            px^[nm1] := (px^[nm1]-uu1^[nm1]*px^[n])/dd^[nm1];
           h := abs(t^[nm1])
          End; {n>1}
        If normt<h
         Then
          normt := h;
        For i:=n-2 Downto 1 Do
          Begin
            t^[i] := (t^[i]-uu1^[i]*t^[i+1]-u2^[i]*t^[i+2])/dd^[i];
            px^[i] := (px^[i]-uu1^[i]*px^[i+1]-u2^[i]*px^[i+2])/dd^[i];
            h := abs(t^[i]);
            If normt<h
             Then
              normt := h
          End; {i}
        ca := normt/normr
      End; {not singular}
  If singular
   Then
    term := 2
  Else
    term := 1;
  freemem(ll, n1s);
  freemem(uu1, n1s);
  freemem(u2, n2s);
  freemem(dd, ns);
  freemem(t, ns);
  freemem(sumrow, ns);
End; {slegtr}

Begin
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef fixate_random}
  randseed := 12345
  {$endif}
=======
  randseed := 12345
>>>>>>> graemeg/fixes_2_2
=======
  randseed := 12345
>>>>>>> origin/fixes_2_2
End.
