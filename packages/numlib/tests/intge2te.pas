program InitInfEx;

uses
  Typ,
  Spe,
  Int;

var
  num2: ArbInt;
  inte: ArbFloat;
const
  e = 2.71828182845905;

  function cx(x: ArbFloat): ArbFloat;
  begin
    cx := cos(x) / (sqr(x) + 1);
  end;

  function pcx(x: ArbFloat): ArbFloat;
  begin
    pcx := 2 * x * sin(x) / sqr(sqr(x) + 1);
  end;

  function ppcx(x: ArbFloat): ArbFloat;
  var
    s: ArbFloat;
  begin
    s    := sqr(x);
    ppcx := cos(x) * (2 - 6 * s) / ((s + 1) * sqr(s + 1));
  end;

  function cc2(x: ArbFloat): ArbFloat;
  var
    x2: ArbFloat;
  begin
    x2  := sqr(x);
    cc2 := cos(x) / (sqr(x2) + x2 + 1);
  end;

  function ucx(x: ArbFloat): ArbFloat;
  begin
    if x = 0 then
      ucx := 0
    else
      ucx := cx((1 - x) / x) / sqr(x);
  end;

  function ucc2(x: ArbFloat): ArbFloat;
  begin
    if x = 0 then
      ucc2 := 0
    else
      ucc2 := cc2((1 - x) / x) / sqr(x);
  end;

  function uz(x: ArbFloat): ArbFloat;
  begin
    uz := sin(x) * exp(-x);
  end;

  function ss1(x: ArbFloat): ArbFloat;    { f(«n(n-1))=0 (n=1,2,3,...) }
  var
    n, s, c: ArbFloat;                    { f(«ný)=2/(ný(n+1))  }
  begin                                   { overigens: f linear interpoleren}
    s := sqrt(2 * x);
    n := trunc(s);
    if n * (n + 1) / 2 <= x then
      n := n + 1;  { n z.d.d. «n(n-1) ó x ó «n(n-1) }
    c := 4 / (n * sqr(n) * (n + 1));
    if s < n then
      ss1 := c * (x - n * (n - 1) / 2)
    else
      ss1 := c * (n * (n + 1) / 2 - x);
  end;

  function ss2(x: ArbFloat): ArbFloat;   { als ss1 met f(«ný)=2/(n.2ü)  }
  var
    n, s, c: ArbFloat;
  begin
    s := sqrt(2 * x);
    n := trunc(s);
    if n * (n + 1) / 2 <= x then
      n := n + 1;
    c := spepow(2, 2 - n) / sqr(n);
    if s < n then
      ss2 := c * (x - n * (n - 1) / 2)
    else
      ss2 := c * (n * (n + 1) / 2 - x);
  end;

  function ss3(x: ArbFloat): ArbFloat;       { x  z.d.d. «.2ü ó x+1 ó 2ü  s=2ü}
  var
    s, c, f, x1: ArbFloat;
    n: ArbInt;        {n even: f(n)=-4/(n.2ü)}
  begin                                  {n oneven: f(n) = 4/(n.2ü)}
    n  := 0;
    s  := 1;
    x1 := x + 1;               { overigens: f lineair interpol.}
    repeat
      n := n + 1;
      s := s * 2
    until s > x1;
    c := 16 / (n * sqr(s));
    if x1 < 0.75 * s then
      f := c * (x1 - s / 2)
    else
      f := c * (s - x1);
    if odd(n) then
      ss3 := f
    else
      ss3 := -f;
  end;

  function ss4(x: ArbFloat): ArbFloat;        { 0 ó x ó 1}
  var
    y, h:  ArbFloat;                        { zij x = ä [1:ì] c(n)/3ü c(n)=0,1,2}
    ready: boolean;                         { zoek kleinste k met c(k)=1}
  begin                                     { dan geldt f(x)=u(y)/3k }
    y     := 3 * x;
    h     := 1 / 3;
    ready := False;         { met u(y)=|y-1«|, 1 ó y ó 2}
    repeat                                { en y =  ä [k:ì] (c(n)/3ü)*3k }
      if (y < 1) or (y > 2) then
      begin
        if y < 1 then
          y := 3 * y
        else
          y := 3 * (y - 2);
        h := h / 3;
        if h < macheps then
        begin
          ready := True;
          ss4   := 0;
        end;
      end
      else
      begin
        ready := True;
        if y < 1.5 then
          ss4 := h * (y - 1)
        else
          ss4 := h * (2 - y);
      end
    until ready;
  end;

  function ss5(x: ArbFloat): ArbFloat;   { uitbreiding ss4}
  var
    y, h:  ArbFloat;                   {functiewaarden op 'volgend' interval}
    ready: boolean;                    { [n, n+1] telkens halveren}
  begin
    y     := 3 * frac(x);
    h     := spepow(0.5, trunc(x)) / 3;
    ready := False;
    repeat
      if (y < 1) or (y > 2) then
      begin
        if y < 1 then
          y := 3 * y
        else
          y := 3 * (y - 2);
        h := h / 3;
        if h < macheps then
        begin
          ready := True;
          ss5   := 0;
        end;
      end
      else
      begin
        ready := True;
        if y < 1.5 then
          ss5 := h * (y - 1)
        else
          ss5 := h * (2 - y);
      end
    until ready;
  end;

  function ss6(x: ArbFloat): ArbFloat;   { 0 ó x ó 1, 'gladdere' variant van ss4}
  var
    y, h:  ArbFloat;
    ready: boolean;

    function f(y: ArbFloat): ArbFloat; { 1 ó y ó 2, 1 x cont. diff, symm. max in 1.5}
    begin
      if y > 1.5 then
        y := 3 - y;
      if y < 1.25 then
        f := sqr(y - 1)
      else
        f := 0.125 - sqr(1.5 - y);
    end;

  begin
    y     := 3 * x;
    h     := 1 / 3;
    ready := False;
    repeat
      if (y < 1) or (y > 2) then
      begin
        if y < 1 then
          y := 3 * y
        else
          y := 3 * (y - 2);
        h := h / 3;
        if h < macheps then
        begin
          ready := True;
          ss6   := 0;
        end;
      end
      else
      begin
        ready := True;
        ss6   := h * f(y);
      end
    until ready;
  end;

  function bb(x: ArbFloat): ArbFloat;
  begin
    bb := spepow(x, -x) * (ln(x) + 1);
  end;

var
  integral, ae, err: ArbFloat;
  term:  ArbInt;
  intex: boolean;

  procedure Header;
  begin
    Write('int': num2, '': numdig - num2, ' ', 'err': 7, ' ': 4);
    if intex then
      Write('f': 6);
    writeln;
  end;

  procedure ShowResults;
  var
    f: ArbFloat;
  begin
    if intex then
      f := inte - integral;
    case term of
      1:
      begin
        Write(integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
      end;
      2:
      begin
        Write(integral: numdig, ' ', err: 10, ' ');
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
      3: Writeln('Verkeerde parameterwaarde (<=0) bij aanroep: ', ae: 8);
      4:
      begin
        Write(integral: numdig, ' ', err: 10, ' ');
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

begin
  num2 := numdig div 2;

  Writeln('     ì   ');
  Writeln('    ô  cos x           ã ');
  Writeln('    ³ ------- dx   =   -- ');
  Writeln('  0 õ  xý+ 1           2e ');
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  inte  := 0.5 * pi / e;
  intex := True;
  writeln(inte: numdig, ' is ''exacte'' oplossing');
  Int1fr(@cx, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
  writeln;
  Writeln('berekend met Int1fr via transformatie x=(1-t)/t');
  writeln;
  Int1fr(@ucx, 0, 1, ae, integral, err, term);
  Header;
  ShowResults;
  Writeln('     ì   ');
  Writeln('    ô  2x sin x           ã ');
  Writeln('    ³ --------- dx   =    -- ');
  Writeln('  0 õ  (xý+ 1)ý           2e ');
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@pcx, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
  Writeln('     ì   ');
  Writeln('    ô (2-6xý)cos x            ã ');
  Writeln('    ³ ------------ dx   =    -- ');
  Writeln('  0 õ  (xý+ 1)3             2e ');
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@ppcx, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;

  Writeln('     ì   ');
  Writeln('    ô     cos x ');
  Writeln('    ³ ------------ dx   =  (ã/û3) exp(-«û3) sin(ã/6+«) ');
  Writeln('  0 õ  (xý)ý+xý+ 1 ');
  writeln;
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  inte := (pi / sqrt(3)) * exp(-sqrt(0.75)) * sin(pi / 6 + 0.5);
  writeln(inte: numdig, ' is ''exacte'' oplossing');
  Int1fr(@cc2, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
  writeln;
  Writeln('berekend met Int1fr via transformatie x=(1-t)/t');
  writeln;
  writeln;
  writeln(inte: numdig, ' is ''exacte'' oplossing');
  Int1fr(@ucc2, 0, 1, ae, integral, err, term);
  Header;
  ShowResults;

  Writeln('     ì   ');
  Writeln('    ô sin u                ');
  Writeln('    ³ ------ du   =  « ');
  Writeln('    õ exp(u)            ');
  writeln('   0 ');
  ae    := 1e-8;
  intex := True;
  inte  := 0.5;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@uz, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;

  writeln(' functie ss1;  int = ä {1:ì}1/n(n+1)  =  1');
  ae    := 1e-8;
  intex := True;
  inte  := 1;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss1, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss2;  int = ä {1:ì} («)ü  =  1');
  ae    := 1e-8;
  intex := True;
  inte  := 1;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss2, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss3;  int = ä {1:ì} (-1)ü/n  =  ln(2)');
  ae    := 1e-8;
  intex := True;
  inte  := ln(2);
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss3, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss4 (op [0,1]); int = 1/28 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1 / 28;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss4, 0, 1, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss5 (op [0,ì)); int = 1/14 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1 / 14;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss5, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss6 (op [0,1]); int = 1/112 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1 / 112;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss6, 0, 1, ae, integral, err, term);
  Header;
  Showresults;

  Writeln('     ì   ');
  Writeln('    ô  ln(x)+1         ');
  Writeln('    ³ ---------- dx   = 1  ');
  Writeln('    õ     xx            ');
  writeln('   1 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@bb, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
end.
program InitInfEx;

uses
  Typ,
  Spe,
  Int;

var
  num2: ArbInt;
  inte: ArbFloat;
const
  e = 2.71828182845905;

  function cx(x: ArbFloat): ArbFloat;
  begin
    cx := cos(x) / (sqr(x) + 1);
  end;

  function pcx(x: ArbFloat): ArbFloat;
  begin
    pcx := 2 * x * sin(x) / sqr(sqr(x) + 1);
  end;

  function ppcx(x: ArbFloat): ArbFloat;
  var
    s: ArbFloat;
  begin
    s    := sqr(x);
    ppcx := cos(x) * (2 - 6 * s) / ((s + 1) * sqr(s + 1));
  end;

  function cc2(x: ArbFloat): ArbFloat;
  var
    x2: ArbFloat;
  begin
    x2  := sqr(x);
    cc2 := cos(x) / (sqr(x2) + x2 + 1);
  end;

  function ucx(x: ArbFloat): ArbFloat;
  begin
    if x = 0 then
      ucx := 0
    else
      ucx := cx((1 - x) / x) / sqr(x);
  end;

  function ucc2(x: ArbFloat): ArbFloat;
  begin
    if x = 0 then
      ucc2 := 0
    else
      ucc2 := cc2((1 - x) / x) / sqr(x);
  end;

  function uz(x: ArbFloat): ArbFloat;
  begin
    uz := sin(x) * exp(-x);
  end;

  function ss1(x: ArbFloat): ArbFloat;    { f(«n(n-1))=0 (n=1,2,3,...) }
  var
    n, s, c: ArbFloat;                    { f(«ný)=2/(ný(n+1))  }
  begin                                   { overigens: f linear interpoleren}
    s := sqrt(2 * x);
    n := trunc(s);
    if n * (n + 1) / 2 <= x then
      n := n + 1;  { n z.d.d. «n(n-1) ó x ó «n(n-1) }
    c := 4 / (n * sqr(n) * (n + 1));
    if s < n then
      ss1 := c * (x - n * (n - 1) / 2)
    else
      ss1 := c * (n * (n + 1) / 2 - x);
  end;

  function ss2(x: ArbFloat): ArbFloat;   { als ss1 met f(«ný)=2/(n.2ü)  }
  var
    n, s, c: ArbFloat;
  begin
    s := sqrt(2 * x);
    n := trunc(s);
    if n * (n + 1) / 2 <= x then
      n := n + 1;
    c := spepow(2, 2 - n) / sqr(n);
    if s < n then
      ss2 := c * (x - n * (n - 1) / 2)
    else
      ss2 := c * (n * (n + 1) / 2 - x);
  end;

  function ss3(x: ArbFloat): ArbFloat;       { x  z.d.d. «.2ü ó x+1 ó 2ü  s=2ü}
  var
    s, c, f, x1: ArbFloat;
    n: ArbInt;        {n even: f(n)=-4/(n.2ü)}
  begin                                  {n oneven: f(n) = 4/(n.2ü)}
    n  := 0;
    s  := 1;
    x1 := x + 1;               { overigens: f lineair interpol.}
    repeat
      n := n + 1;
      s := s * 2
    until s > x1;
    c := 16 / (n * sqr(s));
    if x1 < 0.75 * s then
      f := c * (x1 - s / 2)
    else
      f := c * (s - x1);
    if odd(n) then
      ss3 := f
    else
      ss3 := -f;
  end;

  function ss4(x: ArbFloat): ArbFloat;        { 0 ó x ó 1}
  var
    y, h:  ArbFloat;                        { zij x = ä [1:ì] c(n)/3ü c(n)=0,1,2}
    ready: boolean;                         { zoek kleinste k met c(k)=1}
  begin                                     { dan geldt f(x)=u(y)/3k }
    y     := 3 * x;
    h     := 1 / 3;
    ready := False;         { met u(y)=|y-1«|, 1 ó y ó 2}
    repeat                                { en y =  ä [k:ì] (c(n)/3ü)*3k }
      if (y < 1) or (y > 2) then
      begin
        if y < 1 then
          y := 3 * y
        else
          y := 3 * (y - 2);
        h := h / 3;
        if h < macheps then
        begin
          ready := True;
          ss4   := 0;
        end;
      end
      else
      begin
        ready := True;
        if y < 1.5 then
          ss4 := h * (y - 1)
        else
          ss4 := h * (2 - y);
      end
    until ready;
  end;

  function ss5(x: ArbFloat): ArbFloat;   { uitbreiding ss4}
  var
    y, h:  ArbFloat;                   {functiewaarden op 'volgend' interval}
    ready: boolean;                    { [n, n+1] telkens halveren}
  begin
    y     := 3 * frac(x);
    h     := spepow(0.5, trunc(x)) / 3;
    ready := False;
    repeat
      if (y < 1) or (y > 2) then
      begin
        if y < 1 then
          y := 3 * y
        else
          y := 3 * (y - 2);
        h := h / 3;
        if h < macheps then
        begin
          ready := True;
          ss5   := 0;
        end;
      end
      else
      begin
        ready := True;
        if y < 1.5 then
          ss5 := h * (y - 1)
        else
          ss5 := h * (2 - y);
      end
    until ready;
  end;

  function ss6(x: ArbFloat): ArbFloat;   { 0 ó x ó 1, 'gladdere' variant van ss4}
  var
    y, h:  ArbFloat;
    ready: boolean;

    function f(y: ArbFloat): ArbFloat; { 1 ó y ó 2, 1 x cont. diff, symm. max in 1.5}
    begin
      if y > 1.5 then
        y := 3 - y;
      if y < 1.25 then
        f := sqr(y - 1)
      else
        f := 0.125 - sqr(1.5 - y);
    end;

  begin
    y     := 3 * x;
    h     := 1 / 3;
    ready := False;
    repeat
      if (y < 1) or (y > 2) then
      begin
        if y < 1 then
          y := 3 * y
        else
          y := 3 * (y - 2);
        h := h / 3;
        if h < macheps then
        begin
          ready := True;
          ss6   := 0;
        end;
      end
      else
      begin
        ready := True;
        ss6   := h * f(y);
      end
    until ready;
  end;

  function bb(x: ArbFloat): ArbFloat;
  begin
    bb := spepow(x, -x) * (ln(x) + 1);
  end;

var
  integral, ae, err: ArbFloat;
  term:  ArbInt;
  intex: boolean;

  procedure Header;
  begin
    Write('int': num2, '': numdig - num2, ' ', 'err': 7, ' ': 4);
    if intex then
      Write('f': 6);
    writeln;
  end;

  procedure ShowResults;
  var
    f: ArbFloat;
  begin
    if intex then
      f := inte - integral;
    case term of
      1:
      begin
        Write(integral: numdig, ' ', err: 10, ' ');
        if intex then
          writeln(f: 10)
        else
          writeln;
      end;
      2:
      begin
        Write(integral: numdig, ' ', err: 10, ' ');
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
      3: Writeln('Verkeerde parameterwaarde (<=0) bij aanroep: ', ae: 8);
      4:
      begin
        Write(integral: numdig, ' ', err: 10, ' ');
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

begin
  num2 := numdig div 2;

  Writeln('     ì   ');
  Writeln('    ô  cos x           ã ');
  Writeln('    ³ ------- dx   =   -- ');
  Writeln('  0 õ  xý+ 1           2e ');
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  inte  := 0.5 * pi / e;
  intex := True;
  writeln(inte: numdig, ' is ''exacte'' oplossing');
  Int1fr(@cx, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
  writeln;
  Writeln('berekend met Int1fr via transformatie x=(1-t)/t');
  writeln;
  Int1fr(@ucx, 0, 1, ae, integral, err, term);
  Header;
  ShowResults;
  Writeln('     ì   ');
  Writeln('    ô  2x sin x           ã ');
  Writeln('    ³ --------- dx   =    -- ');
  Writeln('  0 õ  (xý+ 1)ý           2e ');
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@pcx, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
  Writeln('     ì   ');
  Writeln('    ô (2-6xý)cos x            ã ');
  Writeln('    ³ ------------ dx   =    -- ');
  Writeln('  0 õ  (xý+ 1)3             2e ');
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@ppcx, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;

  Writeln('     ì   ');
  Writeln('    ô     cos x ');
  Writeln('    ³ ------------ dx   =  (ã/û3) exp(-«û3) sin(ã/6+«) ');
  Writeln('  0 õ  (xý)ý+xý+ 1 ');
  writeln;
  writeln;
  ae := 1e-8;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  inte := (pi / sqrt(3)) * exp(-sqrt(0.75)) * sin(pi / 6 + 0.5);
  writeln(inte: numdig, ' is ''exacte'' oplossing');
  Int1fr(@cc2, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
  writeln;
  Writeln('berekend met Int1fr via transformatie x=(1-t)/t');
  writeln;
  writeln;
  writeln(inte: numdig, ' is ''exacte'' oplossing');
  Int1fr(@ucc2, 0, 1, ae, integral, err, term);
  Header;
  ShowResults;

  Writeln('     ì   ');
  Writeln('    ô sin u                ');
  Writeln('    ³ ------ du   =  « ');
  Writeln('    õ exp(u)            ');
  writeln('   0 ');
  ae    := 1e-8;
  intex := True;
  inte  := 0.5;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@uz, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;

  writeln(' functie ss1;  int = ä {1:ì}1/n(n+1)  =  1');
  ae    := 1e-8;
  intex := True;
  inte  := 1;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss1, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss2;  int = ä {1:ì} («)ü  =  1');
  ae    := 1e-8;
  intex := True;
  inte  := 1;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss2, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss3;  int = ä {1:ì} (-1)ü/n  =  ln(2)');
  ae    := 1e-8;
  intex := True;
  inte  := ln(2);
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss3, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss4 (op [0,1]); int = 1/28 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1 / 28;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss4, 0, 1, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss5 (op [0,ì)); int = 1/14 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1 / 14;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss5, 0, infinity, ae, integral, err, term);
  Header;
  Showresults;

  writeln(' functie ss6 (op [0,1]); int = 1/112 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1 / 112;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  int1fr(@ss6, 0, 1, ae, integral, err, term);
  Header;
  Showresults;

  Writeln('     ì   ');
  Writeln('    ô  ln(x)+1         ');
  Writeln('    ³ ---------- dx   = 1  ');
  Writeln('    õ     xx            ');
  writeln('   1 ');
  ae    := 1e-8;
  intex := True;
  inte  := 1;
  Writeln('   Gevraagde nauwkeurigheid', ae: 12);
  writeln;
  Int1fr(@bb, 0, infinity, ae, integral, err, term);
  Header;
  ShowResults;
end.
