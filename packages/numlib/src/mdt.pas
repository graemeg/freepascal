{
    This file is part of the Numlib package.
    Copyright (c) 1986-2000 by
     Kees van Ginneken, Wil Kortsmit and Loek van Reij of the
     Computational centre of the Eindhoven University of Technology

    FPC port Code          by Marco van de Voort (marco@freepascal.org)
             documentation by Michael van Canneyt (Michael@freepascal.org)

    Unit was originally undocumented, but is probably an variant of DET.
    Det accepts vectors as arguments, while MDT calculates determinants for
    matrices.

    Contrary to the other undocumented units, this unit is exported in the
    DLL.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
Unit mdt;

interface
{$I DIRECT.INC}

uses typ, dsl, omv;

Procedure mdtgen(n, rwidth: ArbInt; Var alu: ArbFloat; Var p: ArbInt;
                 Var ca:ArbFloat; Var term: ArbInt);

Procedure mdtgtr(n: ArbInt; Var l, d, u, l1, d1, u1, u2: ArbFloat;
                 Var p: boolean; Var ca: ArbFloat; Var term: ArbInt);

Procedure mdtgsy(n, rwidth: ArbInt; Var a: ArbFloat; Var pp:ArbInt;
                 Var qq:boolean; Var ca:ArbFloat; Var term:ArbInt);

Procedure mdtgpd(n, rwidth: ArbInt; Var al, ca: ArbFloat; Var term: ArbInt);

Procedure mdtgba(n, lb, rb, rwa: ArbInt; Var a: ArbFloat; rwl: ArbInt;
                 Var l:ArbFloat; Var p: ArbInt; Var ca: ArbFloat; Var term:ArbInt);

Procedure mdtgpb(n, lb, rwidth: ArbInt; Var al, ca: ArbFloat;
                 Var term: ArbInt);

Procedure mdtdtr(n: ArbInt; Var l, d, u, l1, d1, u1: ArbFloat;
                 Var term:ArbInt);

implementation

Procedure mdtgen(n, rwidth: ArbInt; Var alu: ArbFloat; Var p: ArbInt;
                 Var ca:ArbFloat; Var term: ArbInt);

Var
         indi, indk, nsr, ind, i, j, k, indexpivot : ArbInt;
      normr, sumrowi, pivot, l, normt, maxim, h, s : ArbFloat;
                                   palu, sumrow, t : ^arfloat1;
                                                pp : ^arint1;
                                          singular : boolean;
Begin
  If (n<1) Or (rwidth<1) Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  palu := @alu;
  pp := @p;
  nsr := n*sizeof(ArbFloat);
  getmem(sumrow, nsr);
  getmem(t, nsr);
  normr := 0;
  singular := false ;
  For i:=1 To n Do
    Begin
      ind := (i-1)*rwidth;
      pp^[i] := i;
      sumrowi := 0;
      For j:=1 To n Do
        sumrowi := sumrowi+abs(palu^[ind+j]);
      sumrow^[i] := sumrowi;
     h := 2*random-1;
     t^[i] := sumrowi*h;
      h := abs(h);
     If normr<h Then normr := h;
      If sumrowi=0 Then
        singular := true
    End; {i}
  For k:=1 To n Do
    Begin
      maxim := 0;
     indexpivot := k;
      For i:=k To n Do
        Begin
          ind := (i-1)*rwidth;
          sumrowi := sumrow^[i];
          If sumrowi <> 0 Then
            Begin
              h := abs(palu^[ind+k])/sumrowi;
              If maxim<h Then
                Begin
                  maxim := h;
                 indexpivot := i
                End {maxim<h}
            End {sumrow <> 0}
        End; {i}
      If maxim=0 Then
        singular := true
      Else
        Begin
          If indexpivot <> k Then
            Begin
              ind := (indexpivot-1)*rwidth;
              indk := (k-1)*rwidth;
              For j:=1 To n Do
                Begin
                  h := palu^[ind+j];
                  palu^[ind+j] := palu^[indk+j];
                  palu^[indk+j] := h
                End; {j}
              h := t^[indexpivot];
             t^[indexpivot] := t^[k];
              t^[k] := h;
             pp^[k] := indexpivot;
              sumrow^[indexpivot] := sumrow^[k]
            End; {indexpivot <> k}
          pivot := palu^[(k-1)*rwidth+k];
          For i:=k+1 To n Do
            Begin
              ind := (i-1)*rwidth;
              l := palu^[ind+k]/pivot;
              palu^[ind+k] := l;
              If l <> 0 Then
                Begin
                  For j:=k+1 To n Do
                    palu^[ind+j] := palu^[ind+j]-l*palu^[(k-1)*rwidth+j];
                  If Not singular Then
                    t^[i] := t^[i]-l*t^[k]
                End {l <> 0}
            End {i}
        End {maxim <> 0}
    End; {k}
    If Not singular Then
      Begin
        normt := 0;
        For i:=n Downto 1 Do
          Begin
            indi := (i-1)*rwidth;
            s := 0;
            For j:=i+1 To n Do
              s := s+t^[j]*palu^[indi+j];
            t^[i] := (t^[i]-s)/palu^[indi+i];
            h := abs(t^[i]);
            If normt<h Then
              normt := h
          End; {i}
        ca := normt/normr
      End; {not singular}
    If singular Then
      Begin
        term := 4;
       ca := giant
      End
    Else
      term := 1;
  freemem(sumrow, nsr);
  freemem(t, nsr)
End; {mdtgen}

Procedure mdtgtr(n: ArbInt; Var l, d, u, l1, d1, u1, u2: ArbFloat;
                 Var p: boolean; Var ca: ArbFloat; Var term: ArbInt);

Var
                            i, j, nmin1, sr : ArbInt;
   normr, normt, sumrowi, h, lj, di, ui, ll : ArbFloat;
                                       sing : boolean;
           pd, pu, pd1, pu1, pu2, t, sumrow : ^arfloat1;
                                    pl, pl1 : ^arfloat2;
                                         pp : ^arbool1;
Begin
  If n<1 Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  pl := @l;
 pd := @d;
 pu := @u;
  pl1 := @l1;
 pd1 := @d1;
 pu1 := @u1;
 pu2 := @u2;
 pp := @p;
  sr := sizeof(ArbFloat);
  move(pl^, pl1^, (n-1)*sr);
  move(pd^, pd1^, n*sr);
  move(pu^, pu1^, (n-1)*sr);
  getmem(t, n*sr);
  getmem(sumrow, n*sr);
  normr := 0;
 sing := false;
  nmin1 := n-1;
  For i:=1 To n Do
    Begin
      pp^[i] := false;
      If i=1 Then
        sumrowi := abs(pd^[1])+abs(pu^[1])
      Else
        If i=n Then
          sumrowi := abs(pl^[n])+abs(pd^[n])
        Else
          sumrowi := abs(pl^[i])+abs(pd^[i])+abs(pu^[i]);
      sumrow^[i] := sumrowi;
     h := 2*random-1;
     t^[i] := sumrowi*h;
      h := abs(h);
      If normr<h Then
        normr := h;
      If sumrowi=0 Then
        sing := true
    End; {i}
  j := 1;
  while (j <> n) Do
    Begin
      i := j;
     j := j+1;
     lj := pl1^[j];
      If lj <> 0 Then
        Begin
          di := pd1^[i];
          If di=0 Then
            pp^[i] := true
          Else
            pp^[i] := abs(di/sumrow^[i])<abs(lj/sumrow^[j]);
          If pp^[i] Then
            Begin
              ui := pu1^[i];
             pd1^[i] := lj;
              pu1^[i] := pd1^[j];
             pl1^[j] := di/lj;
             ll := pl1^[j];
              pd1^[j] := ui-ll*pd1^[j];
              If i<nmin1 Then
                Begin
                  pu2^[i] := pu1^[j];
                  pu1^[j] := -ll*pu2^[i]
                End; {i<nmin1}
              sumrow^[j] := sumrow^[i];
              If (Not sing) Then
                Begin
                  h := t^[i];
                 t^[i] := t^[j];
                  t^[j] := h-ll*t^[i]
                End {not sing}
            End {pp^[i]}
          Else
            Begin
              pl1^[j] := lj/di;
             ll := pl1^[j];
              pd1^[j] := pd1^[j]-ll*pu1^[i];
              If i<nmin1 Then
                pu2^[i] := 0;
              If (Not sing) Then
                t^[j] := t^[j]-ll*t^[i]
            End {not pp^[i]}
        End {lj<>0}
      Else
        Begin
          If i<nmin1 Then
            pu2^[i] := 0;
          If pd1^[i]=0 Then
            sing := true
        End {lj=0}
    End; {j}
  If pd1^[n]=0 Then
    sing := true;
  If (Not sing) Then
    Begin
      normt := 0;
      t^[n] := t^[n]/pd1^[n];
      h := abs(t^[n]);
      If normt<h Then
        normt := h;
      If n > 1 Then
        Begin
          t^[nmin1] := (t^[nmin1]-pu1^[nmin1]*t^[n])/pd1^[nmin1];
          h := abs(t^[nmin1])
        End; {n > 1}
      If normt<h Then
        normt := h;
      For i:=n-2 Downto 1 Do
        Begin
          t^[i] := (t^[i]-pu1^[i]*t^[i+1]-pu2^[i]*t^[i+2])/pd1^[i];
          h := abs(t^[i]);
          If normt<h Then
            normt := h
        End; {i}
      ca := normt/normr
    End; {not sing}
  If (sing) Then
    Begin
      term := 4;
     ca := giant
    End {sing}
  Else
    term := 1;
  freemem(t, n*sr);
  freemem(sumrow, n*sr)
End; {mdtgtr}

Procedure mdtgsy(n, rwidth: ArbInt; Var a: ArbFloat; Var pp:ArbInt;
                 Var qq:boolean; Var ca:ArbFloat; Var term:ArbInt);

Var
   i, j, kmin1, k, kplus1, kmin2, imin2, nsr, nsi, nsb,
   imin1, jmin1, indexpivot, iplus1, indi, indj, indk, indp : ArbInt;
    h, absh, maxim, pivot, ct, norma, sumrowi, normt, normr : ArbFloat;
                         alt, l, d, t, u, v, l1, d1, u1, t1 : ^arfloat1;
                                                          p : ^arint1;
                                                          q : ^arbool1;
Begin
  If (n<1) Or (rwidth<1) Then
    Begin
      term := 3;
     exit
    End; {if}
  alt := @a;
 p := @pp;
 q := @qq;
  nsr := n*sizeof(ArbFloat);
  nsi := n*sizeof(ArbInt);
  nsb := n*sizeof(boolean);
  getmem(l, nsr);
  getmem(d, nsr);
  getmem(t, nsr);
  getmem(u, nsr);
  getmem(v, nsr);
  getmem(l1, nsr);
  getmem(d1, nsr);
  getmem(u1, nsr);
  getmem(t1, nsr);
  norma := 0;
  For i:=1 To n Do
    Begin
      indi := (i-1)*rwidth;
      p^[i] := i;
     sumrowi := 0;
      For j:=1 To i Do
        sumrowi := sumrowi+abs(alt^[indi+j]);
      For j:=i+1 To n Do
        sumrowi := sumrowi+abs(alt^[(j-1)*rwidth+i]);
      If norma<sumrowi Then
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
      indk := kmin1*rwidth;
      If k>3 Then
        Begin
          t^[2] := alt^[rwidth+2]*alt^[indk+1]+alt^[2*rwidth+2]*alt^[indk+2];
          For i:=3 To kmin2 Do
            Begin
              indi := (i-1)*rwidth;
              t^[i] := alt^[indi+i-1]*alt^[indk+i-2]+alt^[indi+i]
                       *alt^[indk+i-1]+alt^[indi+rwidth+i]*alt^[indk+i]
            End; {i}
          t^[kmin1] := alt^[indk-rwidth+kmin2]*alt^[indk+k-3]
                       +alt^[indk-rwidth+kmin1]*alt^[indk+kmin2]
                       +alt^[indk+kmin1];
          h := alt^[indk+k];
          For j:=2 To kmin1 Do
            h := h-t^[j]*alt^[indk+j-1];
          t^[k] := h;
          alt^[indk+k] := h-alt^[indk+kmin1]*alt^[indk+kmin2]
        End {k>3}
      Else
       If k=3 Then
        Begin
          t^[2] := alt^[rwidth+2]*alt^[2*rwidth+1]+alt^[2*rwidth+2];
          h := alt^[2*rwidth+3]-t^[2]*alt^[2*rwidth+1];
          t^[3] := h;
          alt^[2*rwidth+3] := h-alt^[2*rwidth+2]*alt^[2*rwidth+1]
        End  {k=3}
      Else
       If k=2 Then
        t^[2] := alt^[rwidth+2];
      maxim := 0;
      For i:=kplus1 To n Do
        Begin
          indi := (i-1)*rwidth;
          h := alt^[indi+k];
          For j:=2 To k Do
            h := h-t^[j]*alt^[indi+j-1];
          absh := abs(h);
          If maxim<absh Then
            Begin
              maxim := absh;
             indexpivot := i
            End; {if}
          alt^[indi+k] := h
        End; {i}
      If maxim <> 0 Then
        Begin
          If indexpivot>kplus1 Then
            Begin
              indp := (indexpivot-1)*rwidth;
              indk := k*rwidth;
              p^[kplus1] := indexpivot;
              For j:=1 To k Do
                Begin
                  h := alt^[indk+j];
                  alt^[indk+j] := alt^[indp+j];
                  alt^[indp+j] := h
                End; {j}
              For j:=indexpivot Downto kplus1 Do
                Begin
                  indj := (j-1)*rwidth;
                  h := alt^[indj+kplus1];
                  alt^[indj+kplus1] := alt^[indp+j];
                  alt^[indp+j] := h
                End; {j}
              For i:=indexpivot To n Do
                Begin
                  indi := (i-1)*rwidth;
                  h := alt^[indi+kplus1];
                  alt^[indi+kplus1] := alt^[indi+indexpivot];
                  alt^[indi+indexpivot] := h
                End  {i}
            End; {if}
          pivot := alt^[k*rwidth+k];
          For i:=k+2 To n Do
            alt^[(i-1)*rwidth+k] := alt^[(i-1)*rwidth+k]/pivot
        End {maxim <> 0}
    End; {k}
  d^[1] := alt^[1];
 i := 1;
  while i<n Do
    Begin
      imin1 := i;
     i := i+1;
      u^[imin1] := alt^[(i-1)*rwidth+imin1];
      l^[imin1] := u^[imin1];
     d^[i] := alt^[(i-1)*rwidth+i]
    End; {i}
  mdtgtr(n, l^[1], d^[1], u^[1], l1^[1], d1^[1], u1^[1], v^[1],
         q^[1], ct, term);
  alt^[1] := d1^[1];
 alt^[rwidth+1] := l1^[1];
  alt^[rwidth+2] := d1^[2];
 alt^[2] := u1^[1];
  imin1 := 1;
 i := 2;
  while i<n Do
    Begin
      imin2 := imin1;
     imin1 := i;
     i := i+1;
      indi := imin1*rwidth;
      alt^[indi+imin1] := l1^[imin1];
     alt^[indi+i] := d1^[i];
      alt^[(imin1-1)*rwidth+i] := u1^[imin1];
      alt^[(imin2-1)*rwidth+i] := v^[imin2]
    End; {i}
  If term=1 Then
    Begin
      normr := 0;
      For i:=1 To n Do
        Begin
          t^[i] := 2*random-1;
         h := t^[i];
          h := abs(h);
          If normr<h Then
            normr := h
        End; {i}
      i := 0;
      while i<n Do
        Begin
          imin1 := i;
         i := i+1;
         j := 1;
         h := t^[i];
          while j<imin1 Do
            Begin
              jmin1 := j;
             j := j+1;
              h := h-alt^[(i-1)*rwidth+jmin1]*t^[j]
            End; {j}
          t^[i] := h
        End; {i}
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
          For j:=iplus1 To n Do
            h := h-alt^[(j-1)*rwidth+imin1]*t1^[j];
          t1^[i] := h;
         h := abs(h);
          If normt<h Then
            normt := h
        End; {i}
      ca := norma*normt/normr
    End {term=1}
  Else ca := giant;
  freemem(l, nsr);
  freemem(d, nsr);
  freemem(t, nsr);
  freemem(u, nsr);
  freemem(v, nsr);
  freemem(l1, nsr);
  freemem(d1, nsr);
  freemem(u1, nsr);
  freemem(t1, nsr)
End; {mdtgsy}

Procedure mdtgpd(n, rwidth: ArbInt; Var al, ca: ArbFloat; Var term: ArbInt);

Var
    posdef                               : boolean;
    i, j, k, kmin1, indk, indi           : ArbInt;
    h, lkk, normr, normt, sumrowi, norma : ArbFloat;
    pal, t                               : ^arfloat1;
Begin
  If (n<1) Or (rwidth<1) Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  getmem(t, sizeof(ArbFloat)*n);
  pal := @al;
  normr := 0;
  posdef := true;
  norma := 0;
  For i:=1 To n Do
    Begin
      sumrowi := 0;
      For j:=1 To i Do
        sumrowi := sumrowi+abs(pal^[(i-1)*rwidth+j]);
      For j:=i+1 To n Do
        sumrowi := sumrowi+abs(pal^[(j-1)*rwidth+i]);
      If norma<sumrowi Then
        norma := sumrowi;
      t^[i] := 2*random-1;
     h := t^[i];
      h := abs(h);
      If normr<h Then
        normr := h
    End; {i}
  k := 0;
  while (k<n) and posdef Do
    Begin
      kmin1 := k;
     k := k+1;
      indk := (k-1)*rwidth;
      lkk := pal^[indk+k];
      For j:=1 To kmin1 Do
        lkk := lkk-sqr(pal^[indk+j]);
      If lkk <= 0 Then
        posdef := false
      Else
        Begin
          pal^[indk+k] := sqrt(lkk);
         lkk := pal^[indk+k];
          For i:=k+1 To n Do
            Begin
              indi := (i-1)*rwidth;
              h := pal^[indi+k];
              For j:=1 To kmin1 Do
                h := h-pal^[indk+j]*pal^[indi+j];
              pal^[indi+k] := h/lkk
            End; {i}
          h := t^[k];
          For j:=1 To kmin1 Do
            h := h-pal^[indk+j]*t^[j];
          t^[k] := h/lkk
        End {posdef}
    End; {k}
  If posdef Then
    Begin
      normt := 0;
      For i:=n Downto 1 Do
        Begin
          h := t^[i];
          For j:=i+1 To n Do
            h := h-pal^[(j-1)*rwidth+i]*t^[j];
          t^[i] := h/pal^[(i-1)*rwidth+i];
          h := abs(t^[i]);
          If normt<h Then
            normt := h
        End; {i}
      ca := norma*normt/normr
    End; {posdef}
  If posdef Then
    term := 1
  Else
    term := 2;
  freemem(t, sizeof(ArbFloat)*n);
End; {mdtgpd}

Procedure mdtgba(n, lb, rb, rwa: ArbInt; Var a: ArbFloat; rwl: ArbInt;
                 Var l:ArbFloat; Var p: ArbInt; Var ca: ArbFloat; Var term:ArbInt);

Var
  sr, i, j, k, ipivot, lbj, lbi, ubi, ls,
                ii, jj, ll, jl, ubj       : ArbInt;
  normr, sumrowi, pivot, normt, maxim, h  : ArbFloat;
      pl, au, sumrow, t, row              : ^arfloat1;
                                       pp : ^arint1;

Begin
  If (n<1) Or (lb<0) Or (rb<0) Or (lb>n-1) Or (rb>n-1) Or (rwl<0) Or (rwa<1) Then
    Begin
      term := 3;
     exit
    End; {term=3}
  sr := sizeof(ArbFloat);
  au := @a;
  pl := @l;
  pp := @p;
  ll := lb+rb+1;
  ls := ll*sr;
  getmem(sumrow, n*sr);
  getmem(t, n*sr);
  getmem(row, ls);
  lbi := n-rb+1;
 lbj := 0;
  jj := 1;
  For i:=lb Downto 1 Do
    Begin
      move(au^[i+jj], au^[jj], (ll-i)*sr);
      fillchar(au^[jj+ll-i], i*sr, 0);
      jj := jj+rwa
    End; {i}
  jj := (n-rb)*rwa+ll;
  For i:=1 To rb Do
    Begin
      fillchar(au^[jj], i*sr, 0);
      jj := jj+rwa-1
    End; {i}
  normr := 0;
 term := 1;
  ii := 1;
  For i:=1 To n Do
    Begin
      pp^[i] := i;
      sumrowi := omvn1v(au^[ii], ll);
      ii := ii+rwa;
      sumrow^[i] := sumrowi;
      h := 2*random-1;
     t^[i] := sumrowi*h;
      h := abs(h);
      If normr<h Then
        normr := h;
      If sumrowi=0 Then
        term := 4
    End; {i}
  ubi := lb;
  jj := 1;
  For k:=1 To n Do
    Begin
     maxim := 0;
     ipivot := k;
     ii := jj;
      If ubi<n Then
        ubi := ubi+1;
      For i:=k To ubi Do
        Begin
          sumrowi := sumrow^[i];
          If sumrowi <> 0 Then
            Begin
              h := abs(au^[ii])/sumrowi;
              ii := ii+rwa;
              If maxim<h Then
                Begin
                  maxim := h;
                 ipivot := i
                End {maxim<h}
            End {sumrowi <> 0}
        End; {i}
      If maxim=0 Then
        Begin
          lbj := 1;
         ubj := ubi-k;
          For j:=lbj To ubj Do
            pl^[(k-1)*rwl+j] := 0;
          For i:=k+1 To ubi Do
            Begin
              ii := (i-1)*rwa;
              For j:=2 To ll Do
                au^[ii+j-1] := au^[ii+j];
              au^[ii+ll] := 0
            End; {i}
          term := 4
        End {maxim=0}
      Else
        Begin
          If ipivot <> k Then
            Begin
              ii := (ipivot-1)*rwa+1;
              move(au^[ii], row^, ls);
              move(au^[jj], au^[ii], ls);
              move(row^, au^[jj], ls);
              h := t^[ipivot];
              t^[ipivot] := t^[k];
              t^[k] := h;
              pp^[k] := ipivot;
              sumrow^[ipivot] := sumrow^[k]
            End; {ipivot <> k}
          pivot := au^[jj];
          jl := 0;
          ii := jj;
          For i:=k+1 To ubi Do
            Begin
              jl := jl+1;
              ii := ii+rwa;
              h := au^[ii]/pivot;
              pl^[(k-1)*rwl+jl] := h;
              For j:=0 To ll-2 Do
                au^[ii+j] := au^[ii+j+1]-h*au^[jj+j+1];
              au^[ii+ll-1] := 0;
              If term=1 Then
                t^[i] := t^[i]-h*t^[k]
            End {i}
        End; {maxim <> 0}
      jj := jj+rwa
    End; {k}
  If term=1 Then
    Begin
      normt := 0;
      ubj := -lb-1;
      jj := n*rwa+1;
      For i:=n Downto 1 Do
        Begin
          jj := jj-rwa;
          If ubj<rb Then
            ubj := ubj+1;
          h := t^[i];
          For j:=1 To ubj+lb Do
            h := h-au^[jj+j]*t^[i+j];
          t^[i] := h/au^[jj];
          h := abs(t^[i]);
          If normt<h Then
            normt := h
        End; {i}
      ca := normt/normr
    End {term=1}
  Else
   ca := giant;
  freemem(sumrow, n*sr);
  freemem(t, n*sr);
  freemem(row, ls)
End; {mdtgba}

Procedure mdtgpb(n, lb, rwidth: ArbInt; Var al, ca: ArbFloat;
                 Var term: ArbInt);

Var
    posdef                                           : boolean;
    i, j, k, r, p, q, ll, llmin1, jmin1, indi        : ArbInt;
    h, normr, normt, sumrowi, alim, norma            : ArbFloat;
    pal, t                                           : ^arfloat1;

    Procedure decomp(i, r: ArbInt);

    Var
        k, ii, ir : ArbInt;
    Begin
      ii := (i-1)*rwidth;
      ir := (r-1)*rwidth;
      h := pal^[ii+j];
     q := ll-j+p;
      For k:=p To jmin1 Do
        Begin
          h := h-pal^[ii+k]*pal^[ir+q];
         q := q+1
        End; {k}
      If j<ll Then
        pal^[ii+j] := h/pal^[ir+ll]
    End; {decomp}

    Procedure lmin1t(i: ArbInt);

    Var
        k:ArbInt;
    Begin
      h := t^[i];
     q := i;
      For k:=llmin1 Downto p Do
        Begin
          q := q-1;
         h := h-pal^[indi+k]*t^[q]
        End; {k}
      t^[i] := h/alim
    End; {lmin1t}

Begin
  If (lb<0) Or (lb>n-1) Or (n<1) Or (rwidth<1) Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  pal := @al;
  getmem(t, n*sizeof(ArbFloat));
  ll := lb+1;
 normr := 0;
 p := ll+1;
 norma := 0;
  For i:=1 To n Do
    Begin
      If p>1 Then
        p := p-1;
      indi := (i-1)*rwidth+p;
      sumrowi := omvn1v(pal^[indi], ll-p+1);
      r := i;
     j := ll;
      while (r<n) and (j>1) Do
        Begin
          r := r+1;
         j := j-1;
          sumrowi := sumrowi+abs(pal^[(r-1)*rwidth+j])
        End; {r,j}
      If norma<sumrowi Then
        norma := sumrowi;
      h := 2*random-1;
     t^[i] := h;
      h := abs(h);
      If normr<h Then
        normr := h
    End; {i}
    llmin1 := ll-1;
    p := ll+1;
    i := 0;
    posdef := true ;
    while (i<n) and posdef Do
      Begin
        i := i+1;
        indi := (i-1)*rwidth;
        If p>1 Then
          p := p-1;
        r := i-ll+p;
       j := p-1;
        while j<llmin1 Do
          Begin
            jmin1 := j;
           j := j+1;
            decomp(i, r);
           r := r+1
          End; {j}
        jmin1 := llmin1;
       j := ll;
       decomp(i, i);
        If h <= 0 Then
          posdef := false
        Else
          Begin
            alim := sqrt(h);
           pal^[indi+ll] := alim;
            lmin1t(i)
          End
      End; {i}
    If posdef Then
      Begin
        normt := 0;
       p := ll+1;
        For i:=n Downto 1 Do
          Begin
            If p>1 Then
              p := p-1;
            q := i;
           h := t^[i];
            For k:=llmin1 Downto p Do
              Begin
                q := q+1;
               h := h-pal^[(q-1)*rwidth+k]*t^[q]
              End; {k}
            t^[i] := h/pal^[(i-1)*rwidth+ll];
            h := abs(t^[i]);
            If normt<h Then
              normt := h
          End; {i}
        ca := norma*normt/normr
      End; {posdef}
    If posdef Then
      term := 1
    Else
      term := 2;
  freemem(t, n*sizeof(ArbFloat));
End; {mdtgpb}

Procedure mdtdtr(n: ArbInt; Var l, d, u, l1, d1, u1: ArbFloat;
                 Var term:ArbInt);

Var
                      i, j, s : ArbInt;
                       lj, di : ArbFloat;
             pd, pu, pd1, pu1 : ^arfloat1;
                      pl, pl1 : ^arfloat2;

Begin
  If n<1 Then
    Begin
      term := 3;
     exit
    End; {wrong input}
  pl := @l;
  pd := @d;
  pu := @u;
  pl1 := @l1;
  pd1 := @d1;
  pu1 := @u1;
  s := sizeof(ArbFloat);
  move(pl^, pl1^, (n-1)*s);
  move(pd^, pd1^, n*s);
  move(pu^, pu1^, (n-1)*s);
  j := 1;
  di := pd1^[j];
  If di=0 Then
    term := 2
  Else
    term := 1;
  while (term=1) and (j <> n) Do
    Begin
     i := j;
     j := j+1;
     lj := pl1^[j]/di;
     pl1^[j] := lj;
     di := pd1^[j]-lj*pu1^[i];
     pd1^[j] := di;
     If di=0 Then
      term := 2
    End {j}
End; {mdtdtr}

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
