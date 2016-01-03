program intge1te;

uses
  typ,
  spe,
  int;

const
  e      = 2.71828182845905;
  fnames = 'KI A0 A1 A2 A3 A4 SS SL SE V1 V2 ';
  ogs: array[1..11] of ArbFloat = (0, 0, 1, e, 0, 1, 0, 1, 1, 1, 1);
  integraaltekst: array[1..11, 1..5] of string[60] =
    (('  ì                                                       ',
    ' ô  -àcosh(x)                                             ',
    {k0}  ' ³ e         dx  = k0(à), mits à > 0.                     ',
    ' õ                                                        ',
    '0                                                         '),
    ('  ì                                                       ',
    ' ô  sin x     àcos x                                      ',
    {a0}  ' ³ ------- + ---------- dx  =  1, mits  à>0               ',
    ' õ (x+1)à   (x+1)(à+1)                                  ',
    '0                                                         '),
    ('  ì                                                       ',
    ' ô   1                                                    ',
    {a1}  ' ³ ---- dx   =  1/(à-1), mits  à>1                        ',
    ' õ  xà                                                   ',
    '1                                                         '),
    ('  ì                                                       ',
    ' ô     dx                                                 ',
    {a2}  ' ³  ---------    =  1/(à-1), mits  à>1                    ',
    ' õ  x.ln(x)à                                             ',
    'e                                                         '),
    ('  ì                                                       ',
    ' ô Ú àùxàùsin(xà)    cos(xà)¿                          ',
    {a3}  ' ³ ³ -------------- + ---------³ dx = 1, mits  à>0        ',
    ' õ À    x(x+1)          (x+1)ý Ù                          ',
    '0                                                         '),
    ('  ì                                                       ',
    ' ô Ú 2sin(«ãùxà)       xàùcos(«ãùxà) ¿                 ',
    {a4}  ' ³ ³-------------- + ãà-----------------³ dx = 1, mits àò0',
    ' õ À   (x+1)ý                x(x+1)     Ù                 ',
    '1                                                         '),
    ('  ss(n)=2*(n+1)(à-1)/n  (n=1,2,3...), àò0                ',
    {ss}  '  ss(x)=0 als min(|n-x|) ò 0.5/(n+1)à                    ',
    '  ss lineair interpoleren als min(|n-x|) ó 0.5/(n+1)à    ',
    '  int. 0:ì = ä [1:ì] 1/(n(n+1)) = 1                       ',
    '                                                          '),
    ('  ì                                                       ',
    ' ô  sin(ln(x))          1                                 ',
    {sl}  ' ³  --------- dx =  ---------,   mits  à>1                ',
    ' õ     xà          (à-1)ý+1                              ',
    '1                                                         '),
    (' ì                                                        ',
    ' ô  sin(xà)-à.x(à-1).cos(xà)       sin(1)              ',
    {se}  ' ³  --------------------------- dx =  ------              ',
    ' õ            ex                       e                 ',
    '1                                                         '),
    ('  ì                                                       ',
    '  ô    à.|x|(à-1)                                        ',
    {v1}  '  ³  ---------------- dx =  1, mits à > 0                 ',
    '  õ  ã.(|x|(2à) + 1)                                     ',
    '-ì                                                        '),
    ('  ì            0           ì                              ',
    '  ô            ô  àx      ô  -x/à                         ',
    {v2}  '  ³ v2(x)dx =  ³ e   dx + ³ e    dx =  à + 1/à, mits à > 0',
    '  õ            õ          õ                               ',
    '-ì           -ì          0                                '));

var
  alfa, ond, inte, int1: ArbFloat;
  u, i:  ArbInt;
  s:     string;
  q:     char;
  f:     rfunc1r;
  scale: boolean;

  function Ki(x: ArbFloat): ArbFloat;
  var
    kk: ArbFloat;
  begin
    if abs(x) < ln(100 / alfa) then
      kk := Exp(-alfa * Specoh(x))
    else
      kk := 0;
    if scale then
      ki := kk / int1
    else
      ki := kk;
  end;

  function uki(u: ArbFloat): ArbFloat;   {u=1/(x+1), of x=1/u-1}
  begin
    if u > 0 then
      uki := ki((1 - u) / u) / sqr(u)
    else
      uki := 0;
  end;

  function a0(x: ArbFloat): ArbFloat;
  begin
    a0 := ((x + 1) * sin(x) + alfa * cos(x)) * spepow(x + 1, -alfa - 1);
  end;

  function ua0(u: ArbFloat): ArbFloat;   {u=1/(x+1), of x=1/u-1}
  begin
    if u > 0 then
      ua0 := a0((1 - u) / u) / sqr(u)
    else
      ua0 := 0;
  end;

  function a1(x: ArbFloat): ArbFloat;
  var
    a: ArbFloat;
  begin
    a := spepow(x, -alfa);
    if scale then
      a1 := (alfa - 1) * a
    else
      a1 := a;
  end;

  function ua1(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      ua1 := a1(ond / u) * ond / sqr(u)
    else
      ua1 := 0;
  end;

  function a2(x: ArbFloat): ArbFloat;
  var
    a: ArbFloat;
  begin
    a := spepow(ln(x), -alfa) / x;
    if scale then
      a2 := (alfa - 1) * a
    else
      a2 := a;
  end;

  function ua2(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      ua2 := a2(ond / u) * ond / sqr(u)
    else
      ua2 := 0;
  end;

  function a3(x: ArbFloat): ArbFloat;
  var
    y: ArbFloat;
  begin
    if x = 0 then
      a3 := 0
    else
    begin
      y  := spepow(x, alfa);
      a3 := alfa * y * sin(y) / (x * (x + 1)) + cos(y) / sqr(x + 1);
    end;
  end;

  function ua3(u: ArbFloat): ArbFloat;   {u=1/(x+1), of x=1/u-1}
  begin
    if u > 0 then
      ua3 := a3((1 - u) / u) / sqr(u)
    else
      ua3 := 0;
  end;

  function a4(x: ArbFloat): ArbFloat;
  var
    y, z: ArbFloat;
  begin
    y  := spepow(x, alfa);
    z  := y * pi / 2;
    a4 := 2 * sin(z) / sqr(x + 1) - pi * alfa * y * cos(z) / (x * (x + 1));
  end;

  function ua4(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      ua4 := a4(ond / u) * ond / sqr(u)
    else
      ua4 := 0;
  end;

  function ss(x: ArbFloat): ArbFloat;
  var
    d, eps, r: ArbFloat;
  begin
    if x > 0.5 then
    begin
      d := frac(x);
      r := x - d;
      if d > 0.5 then
      begin
        d := 1 - d;
        r := r + 1;
      end;
      eps := 0.5 / spepow(r + 1, alfa);
      if d > eps then
        ss := 0
      else
        ss := (1 - d / eps) / (r * (r + 1) * eps);
    end
    else
      ss := 0;
  end;

  function uss(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      uss := ss(ond / u) * ond / sqr(u)
    else
      uss := 0;
  end;

  function sl(x: ArbFloat): ArbFloat;
  var
    sl1: ArbFloat;
  begin
    sl1 := sin(ln(x)) * spepow(x, -alfa);
    if scale then
      sl := sl1 / int1
    else
      sl := sl1;
  end;

  function usl(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      usl := sl(ond / u) * ond / sqr(u)
    else
      usl := 0;
  end;

  function se(x: ArbFloat): ArbFloat;
  var
    y, se1: ArbFloat;
  begin
    y   := spepow(x, alfa);
    se1 := (sin(y) - alfa * (y / x) * cos(y)) * exp(-x);
    if scale then
      se := se1 / int1
    else
      se := se1;
  end;

  function use(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      use := se(ond / u) * ond / sqr(u)
    else
      use := 0;
  end;

  function v1(x: ArbFloat): ArbFloat;
  var
    a, y: ArbFloat;
  begin
    x    := abs(x);
    alfa := abs(alfa);
    if x = 0 then
    begin
      if alfa = 1 then
        v1 := alfa / pi
      else
        v1 := 0;
    end
    else
    begin
      if x > 1 then
        a := -alfa - 1
      else
        a := alfa - 1;
      y := spepow(x, a);
      v1 := alfa * y / (pi * (sqr(x * y) + 1));
    end;
  end;

  function uv1(u: ArbFloat): ArbFloat;  { u=«((2/ã)arctan(x)+1) of x=tan(«ã(2u-1)) }
  var
    y: ArbFloat;                         { 0 ó u ó 1 }
  begin
    if (u = 0) or (u = 1) then
      uv1 := 0
    else
    begin
      y   := 1 / sqr(cos(pi * (u - 0.5)));
      uv1 := pi * v1(sqrt(y - 1)) * y;
    end;
  end;

  function v2(x: ArbFloat): ArbFloat;
  var
    v: ArbFloat;
  begin
    alfa := abs(alfa);
    if x > 0 then
      v := exp(-x / alfa)
    else
    if x < 0 then
      v := exp(x * alfa)
    else
      v := 1;
    if scale then
      v2 := v / (alfa + 1 / alfa)
    else
      v2 := v;
  end;

  function uv2(u: ArbFloat): ArbFloat;  { u=«((2/ã)arctan(x)+1) of x=tan(«ã(2u-1)) }
  var
    y: ArbFloat;                         { 0 ó u ó 1 }
  begin
    if (u = 0) or (u = 1) then
      uv2 := 0
    else
    begin
      y := 1 / sqr(cos(pi * (u - 0.5)));
      if u > 0.5 then
        uv2 := pi * v2(sqrt(y - 1)) * y
      else
        uv2 := pi * v2(-sqrt(y - 1)) * y;
    end;
  end;

var
  integral, ae, err: ArbFloat;
  term, num2:   ArbInt;
  intex, First: boolean;

  procedure Header;
  var
    i: ArbInt;
  begin
    for i := 1 to 5 do
      if i = 3 then
        writeln(s: 3, ': ', Integraaltekst[u, i])
      else
        writeln('': 5, Integraaltekst[u, i]);
  end;

  procedure ShowResults;
  var
    f: ArbFloat;
  begin
    if First then
      writeln('alfa': num2, '': numdig - num2, 'ae': 7, ' ': 4, 'int': num2,
        '': numdig - num2, ' ', 'err': 7, ' ': 4, 'f': 6);
    First := False;
    if intex then
      f := inte - integral;
    case term of
      1:
      begin
        Write(alfa: numdig, ae: 10, integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
      end;
      2:
      begin
        Write(alfa: numdig, ae: 10, integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
<<<<<<< HEAD
<<<<<<< HEAD
        Writeln('    process afgebroken, te hoge nauwkeurigheid?');
=======
        Writeln('    proces afgebroken, te hoge nauwkeurigheid?');
>>>>>>> graemeg/fixes_2_2
=======
        Writeln('    proces afgebroken, te hoge nauwkeurigheid?');
>>>>>>> origin/fixes_2_2
      end;
      3: Writeln('Verkeerde waarde ae (<=0) bij aanroep: ', ae: 8);
      4:
      begin
        Write(alfa: numdig, ae: 10, integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
<<<<<<< HEAD
<<<<<<< HEAD
        writeln('    process afgebroken, moeilijk, mogelijk divergent?');
=======
        writeln('    proces afgebroken, moeilijk, mogelijk divergent?');
>>>>>>> graemeg/fixes_2_2
=======
        writeln('    proces afgebroken, moeilijk, mogelijk divergent?');
>>>>>>> origin/fixes_2_2
      end;
    end;
  end;

const
  fint: array[boolean, 1..11] of rfunc1r =
    ((@ki, @a0, @a1, @a2, @a3, @a4, @ss, @sl, @se, @v1, @v2),
    (@uki, @ua0, @ua1, @ua2, @ua3, @ua4, @uss, @usl, @use, @uv1, @uv2));
begin
  s := ParamStr(1);
  if s = '' then
  begin
    writeln(' Vergeten  functienaam mee te geven!');
    writeln(' Kies uit: ', fnames);
    halt;
  end;
  for i := 1 to length(s) do
    s[i] := Upcase(s[i]);
  u := (Pos(s, fnames) + 2) div 3;
  if u = 0 then
  begin
    writeln(' Commandlineparameter ', s, ' bestaat niet');
    writeln(' Kies uit: ', fnames);
    halt;
  end;

  Write('program results int1fr function ' + s);
  case SizeOf(ArbFloat) of
    4: writeln('(single)');
    8: writeln('(double)');
    6: writeln('(real)');
  end;
  num2 := numdig div 2;
  if Pos(s, 'a0 a4 a3 ss v1') > 0 then
    scale := True
  else
  begin
    Write(' scale ? (y or n)');
    readln(q);
    scale := Upcase(q) = 'Y';
  end;
  Write('Transformatie naar 0 => 1 ? (y or n)');
  readln(q);
  ond := ogs[u];
  f   := fint[Upcase(q) = 'Y'][u];
  Header;
  Writeln('à en ae: ');
  First := True;
  while not eoln do
  begin
    Read(alfa, ae);
    intex := True;
    case u of
      1: int1 := spebk0(alfa);
      2:
      begin
        int1  := 1;
        intex := alfa > 0;
      end;
      3:
      begin
        if alfa > 1 then
          int1 := 1 / (alfa - 1);
        intex := alfa > 1;
      end;
      4:
      begin
        if alfa > 1 then
          int1 := 1 / (alfa - 1);
        intex := alfa > 1;
      end;
      5:
      begin
        if alfa > 0 then
          int1 := 1
        else
          int1 := cos(1);
        intex := alfa > 0;
      end;
      6:
      begin
        int1  := 1;
        intex := alfa > 0;
      end;
      7: int1 := 1;
      8:
      begin
        if alfa > 1 then
          int1 := 1 / (sqr(alfa - 1) + 1);
        intex := alfa > 1;
      end;
      9: int1 := sin(1) / e;
      10:
      begin
        int1  := 1;
        intex := alfa <> 0;
      end;
      11:
      begin
        if alfa <> 0 then
          int1 := abs(alfa) + 1 / abs(alfa);
        intex := alfa <> 0;
      end;
    end;
    if scale then
      inte := 1
    else
      inte := int1;
    if Upcase(q) = 'Y' then
      int1fr(f, 0, 1, ae, integral, err, term)
    else if u < 10 then
      int1fr(f, ond, infinity, ae, integral, err, term)
    else
      int1fr(f, -infinity, infinity, ae, integral, err, term);
    Showresults;
  end;
end.
program intge1te;

uses
  typ,
  spe,
  int;

const
  e      = 2.71828182845905;
  fnames = 'KI A0 A1 A2 A3 A4 SS SL SE V1 V2 ';
  ogs: array[1..11] of ArbFloat = (0, 0, 1, e, 0, 1, 0, 1, 1, 1, 1);
  integraaltekst: array[1..11, 1..5] of string[60] =
    (('  ì                                                       ',
    ' ô  -àcosh(x)                                             ',
    {k0}  ' ³ e         dx  = k0(à), mits à > 0.                     ',
    ' õ                                                        ',
    '0                                                         '),
    ('  ì                                                       ',
    ' ô  sin x     àcos x                                      ',
    {a0}  ' ³ ------- + ---------- dx  =  1, mits  à>0               ',
    ' õ (x+1)à   (x+1)(à+1)                                  ',
    '0                                                         '),
    ('  ì                                                       ',
    ' ô   1                                                    ',
    {a1}  ' ³ ---- dx   =  1/(à-1), mits  à>1                        ',
    ' õ  xà                                                   ',
    '1                                                         '),
    ('  ì                                                       ',
    ' ô     dx                                                 ',
    {a2}  ' ³  ---------    =  1/(à-1), mits  à>1                    ',
    ' õ  x.ln(x)à                                             ',
    'e                                                         '),
    ('  ì                                                       ',
    ' ô Ú àùxàùsin(xà)    cos(xà)¿                          ',
    {a3}  ' ³ ³ -------------- + ---------³ dx = 1, mits  à>0        ',
    ' õ À    x(x+1)          (x+1)ý Ù                          ',
    '0                                                         '),
    ('  ì                                                       ',
    ' ô Ú 2sin(«ãùxà)       xàùcos(«ãùxà) ¿                 ',
    {a4}  ' ³ ³-------------- + ãà-----------------³ dx = 1, mits àò0',
    ' õ À   (x+1)ý                x(x+1)     Ù                 ',
    '1                                                         '),
    ('  ss(n)=2*(n+1)(à-1)/n  (n=1,2,3...), àò0                ',
    {ss}  '  ss(x)=0 als min(|n-x|) ò 0.5/(n+1)à                    ',
    '  ss lineair interpoleren als min(|n-x|) ó 0.5/(n+1)à    ',
    '  int. 0:ì = ä [1:ì] 1/(n(n+1)) = 1                       ',
    '                                                          '),
    ('  ì                                                       ',
    ' ô  sin(ln(x))          1                                 ',
    {sl}  ' ³  --------- dx =  ---------,   mits  à>1                ',
    ' õ     xà          (à-1)ý+1                              ',
    '1                                                         '),
    (' ì                                                        ',
    ' ô  sin(xà)-à.x(à-1).cos(xà)       sin(1)              ',
    {se}  ' ³  --------------------------- dx =  ------              ',
    ' õ            ex                       e                 ',
    '1                                                         '),
    ('  ì                                                       ',
    '  ô    à.|x|(à-1)                                        ',
    {v1}  '  ³  ---------------- dx =  1, mits à > 0                 ',
    '  õ  ã.(|x|(2à) + 1)                                     ',
    '-ì                                                        '),
    ('  ì            0           ì                              ',
    '  ô            ô  àx      ô  -x/à                         ',
    {v2}  '  ³ v2(x)dx =  ³ e   dx + ³ e    dx =  à + 1/à, mits à > 0',
    '  õ            õ          õ                               ',
    '-ì           -ì          0                                '));

var
  alfa, ond, inte, int1: ArbFloat;
  u, i:  ArbInt;
  s:     string;
  q:     char;
  f:     rfunc1r;
  scale: boolean;

  function Ki(x: ArbFloat): ArbFloat;
  var
    kk: ArbFloat;
  begin
    if abs(x) < ln(100 / alfa) then
      kk := Exp(-alfa * Specoh(x))
    else
      kk := 0;
    if scale then
      ki := kk / int1
    else
      ki := kk;
  end;

  function uki(u: ArbFloat): ArbFloat;   {u=1/(x+1), of x=1/u-1}
  begin
    if u > 0 then
      uki := ki((1 - u) / u) / sqr(u)
    else
      uki := 0;
  end;

  function a0(x: ArbFloat): ArbFloat;
  begin
    a0 := ((x + 1) * sin(x) + alfa * cos(x)) * spepow(x + 1, -alfa - 1);
  end;

  function ua0(u: ArbFloat): ArbFloat;   {u=1/(x+1), of x=1/u-1}
  begin
    if u > 0 then
      ua0 := a0((1 - u) / u) / sqr(u)
    else
      ua0 := 0;
  end;

  function a1(x: ArbFloat): ArbFloat;
  var
    a: ArbFloat;
  begin
    a := spepow(x, -alfa);
    if scale then
      a1 := (alfa - 1) * a
    else
      a1 := a;
  end;

  function ua1(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      ua1 := a1(ond / u) * ond / sqr(u)
    else
      ua1 := 0;
  end;

  function a2(x: ArbFloat): ArbFloat;
  var
    a: ArbFloat;
  begin
    a := spepow(ln(x), -alfa) / x;
    if scale then
      a2 := (alfa - 1) * a
    else
      a2 := a;
  end;

  function ua2(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      ua2 := a2(ond / u) * ond / sqr(u)
    else
      ua2 := 0;
  end;

  function a3(x: ArbFloat): ArbFloat;
  var
    y: ArbFloat;
  begin
    if x = 0 then
      a3 := 0
    else
    begin
      y  := spepow(x, alfa);
      a3 := alfa * y * sin(y) / (x * (x + 1)) + cos(y) / sqr(x + 1);
    end;
  end;

  function ua3(u: ArbFloat): ArbFloat;   {u=1/(x+1), of x=1/u-1}
  begin
    if u > 0 then
      ua3 := a3((1 - u) / u) / sqr(u)
    else
      ua3 := 0;
  end;

  function a4(x: ArbFloat): ArbFloat;
  var
    y, z: ArbFloat;
  begin
    y  := spepow(x, alfa);
    z  := y * pi / 2;
    a4 := 2 * sin(z) / sqr(x + 1) - pi * alfa * y * cos(z) / (x * (x + 1));
  end;

  function ua4(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      ua4 := a4(ond / u) * ond / sqr(u)
    else
      ua4 := 0;
  end;

  function ss(x: ArbFloat): ArbFloat;
  var
    d, eps, r: ArbFloat;
  begin
    if x > 0.5 then
    begin
      d := frac(x);
      r := x - d;
      if d > 0.5 then
      begin
        d := 1 - d;
        r := r + 1;
      end;
      eps := 0.5 / spepow(r + 1, alfa);
      if d > eps then
        ss := 0
      else
        ss := (1 - d / eps) / (r * (r + 1) * eps);
    end
    else
      ss := 0;
  end;

  function uss(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      uss := ss(ond / u) * ond / sqr(u)
    else
      uss := 0;
  end;

  function sl(x: ArbFloat): ArbFloat;
  var
    sl1: ArbFloat;
  begin
    sl1 := sin(ln(x)) * spepow(x, -alfa);
    if scale then
      sl := sl1 / int1
    else
      sl := sl1;
  end;

  function usl(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      usl := sl(ond / u) * ond / sqr(u)
    else
      usl := 0;
  end;

  function se(x: ArbFloat): ArbFloat;
  var
    y, se1: ArbFloat;
  begin
    y   := spepow(x, alfa);
    se1 := (sin(y) - alfa * (y / x) * cos(y)) * exp(-x);
    if scale then
      se := se1 / int1
    else
      se := se1;
  end;

  function use(u: ArbFloat): ArbFloat;  {u=ond/x of x=ond/u}
  begin
    if u > 0 then
      use := se(ond / u) * ond / sqr(u)
    else
      use := 0;
  end;

  function v1(x: ArbFloat): ArbFloat;
  var
    a, y: ArbFloat;
  begin
    x    := abs(x);
    alfa := abs(alfa);
    if x = 0 then
    begin
      if alfa = 1 then
        v1 := alfa / pi
      else
        v1 := 0;
    end
    else
    begin
      if x > 1 then
        a := -alfa - 1
      else
        a := alfa - 1;
      y := spepow(x, a);
      v1 := alfa * y / (pi * (sqr(x * y) + 1));
    end;
  end;

  function uv1(u: ArbFloat): ArbFloat;  { u=«((2/ã)arctan(x)+1) of x=tan(«ã(2u-1)) }
  var
    y: ArbFloat;                         { 0 ó u ó 1 }
  begin
    if (u = 0) or (u = 1) then
      uv1 := 0
    else
    begin
      y   := 1 / sqr(cos(pi * (u - 0.5)));
      uv1 := pi * v1(sqrt(y - 1)) * y;
    end;
  end;

  function v2(x: ArbFloat): ArbFloat;
  var
    v: ArbFloat;
  begin
    alfa := abs(alfa);
    if x > 0 then
      v := exp(-x / alfa)
    else
    if x < 0 then
      v := exp(x * alfa)
    else
      v := 1;
    if scale then
      v2 := v / (alfa + 1 / alfa)
    else
      v2 := v;
  end;

  function uv2(u: ArbFloat): ArbFloat;  { u=«((2/ã)arctan(x)+1) of x=tan(«ã(2u-1)) }
  var
    y: ArbFloat;                         { 0 ó u ó 1 }
  begin
    if (u = 0) or (u = 1) then
      uv2 := 0
    else
    begin
      y := 1 / sqr(cos(pi * (u - 0.5)));
      if u > 0.5 then
        uv2 := pi * v2(sqrt(y - 1)) * y
      else
        uv2 := pi * v2(-sqrt(y - 1)) * y;
    end;
  end;

var
  integral, ae, err: ArbFloat;
  term, num2:   ArbInt;
  intex, First: boolean;

  procedure Header;
  var
    i: ArbInt;
  begin
    for i := 1 to 5 do
      if i = 3 then
        writeln(s: 3, ': ', Integraaltekst[u, i])
      else
        writeln('': 5, Integraaltekst[u, i]);
  end;

  procedure ShowResults;
  var
    f: ArbFloat;
  begin
    if First then
      writeln('alfa': num2, '': numdig - num2, 'ae': 7, ' ': 4, 'int': num2,
        '': numdig - num2, ' ', 'err': 7, ' ': 4, 'f': 6);
    First := False;
    if intex then
      f := inte - integral;
    case term of
      1:
      begin
        Write(alfa: numdig, ae: 10, integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
      end;
      2:
      begin
        Write(alfa: numdig, ae: 10, integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
<<<<<<< HEAD
<<<<<<< HEAD
        Writeln('    process afgebroken, te hoge nauwkeurigheid?');
=======
        Writeln('    proces afgebroken, te hoge nauwkeurigheid?');
>>>>>>> graemeg/fixes_2_2
=======
        Writeln('    proces afgebroken, te hoge nauwkeurigheid?');
>>>>>>> origin/fixes_2_2
      end;
      3: Writeln('Verkeerde waarde ae (<=0) bij aanroep: ', ae: 8);
      4:
      begin
        Write(alfa: numdig, ae: 10, integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
<<<<<<< HEAD
<<<<<<< HEAD
        writeln('    process afgebroken, moeilijk, mogelijk divergent?');
=======
        writeln('    proces afgebroken, moeilijk, mogelijk divergent?');
>>>>>>> graemeg/fixes_2_2
=======
        writeln('    proces afgebroken, moeilijk, mogelijk divergent?');
>>>>>>> origin/fixes_2_2
      end;
    end;
  end;

const
  fint: array[boolean, 1..11] of rfunc1r =
    ((@ki, @a0, @a1, @a2, @a3, @a4, @ss, @sl, @se, @v1, @v2),
    (@uki, @ua0, @ua1, @ua2, @ua3, @ua4, @uss, @usl, @use, @uv1, @uv2));
begin
  s := ParamStr(1);
  if s = '' then
  begin
    writeln(' Vergeten  functienaam mee te geven!');
    writeln(' Kies uit: ', fnames);
    halt;
  end;
  for i := 1 to length(s) do
    s[i] := Upcase(s[i]);
  u := (Pos(s, fnames) + 2) div 3;
  if u = 0 then
  begin
    writeln(' Commandlineparameter ', s, ' bestaat niet');
    writeln(' Kies uit: ', fnames);
    halt;
  end;

  Write('program results int1fr function ' + s);
  case SizeOf(ArbFloat) of
    4: writeln('(single)');
    8: writeln('(double)');
    6: writeln('(real)');
  end;
  num2 := numdig div 2;
  if Pos(s, 'a0 a4 a3 ss v1') > 0 then
    scale := True
  else
  begin
    Write(' scale ? (y or n)');
    readln(q);
    scale := Upcase(q) = 'Y';
  end;
  Write('Transformatie naar 0 => 1 ? (y or n)');
  readln(q);
  ond := ogs[u];
  f   := fint[Upcase(q) = 'Y'][u];
  Header;
  Writeln('à en ae: ');
  First := True;
  while not eoln do
  begin
    Read(alfa, ae);
    intex := True;
    case u of
      1: int1 := spebk0(alfa);
      2:
      begin
        int1  := 1;
        intex := alfa > 0;
      end;
      3:
      begin
        if alfa > 1 then
          int1 := 1 / (alfa - 1);
        intex := alfa > 1;
      end;
      4:
      begin
        if alfa > 1 then
          int1 := 1 / (alfa - 1);
        intex := alfa > 1;
      end;
      5:
      begin
        if alfa > 0 then
          int1 := 1
        else
          int1 := cos(1);
        intex := alfa > 0;
      end;
      6:
      begin
        int1  := 1;
        intex := alfa > 0;
      end;
      7: int1 := 1;
      8:
      begin
        if alfa > 1 then
          int1 := 1 / (sqr(alfa - 1) + 1);
        intex := alfa > 1;
      end;
      9: int1 := sin(1) / e;
      10:
      begin
        int1  := 1;
        intex := alfa <> 0;
      end;
      11:
      begin
        if alfa <> 0 then
          int1 := abs(alfa) + 1 / abs(alfa);
        intex := alfa <> 0;
      end;
    end;
    if scale then
      inte := 1
    else
      inte := int1;
    if Upcase(q) = 'Y' then
      int1fr(f, 0, 1, ae, integral, err, term)
    else if u < 10 then
      int1fr(f, ond, infinity, ae, integral, err, term)
    else
      int1fr(f, -infinity, infinity, ae, integral, err, term);
    Showresults;
  end;
end.
