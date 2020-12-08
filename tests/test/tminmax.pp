{ %opt=-O- -Oonofastmath }  { with fast math, the operands of min/max might be swapped and this breaks the tests using NaN }

{$mode objfpc}
uses
  Math;

procedure TestSingle;

  function Min1(a, b: Single): Single; inline;
    begin
      if a < b then
        Result := a
      else
        Result := b;
    end;


  function Max1(a, b: Single): Single; inline;
    begin
      if a > b then
        Result := a
      else
        Result := b;
    end;

  function Min2(a, b: Single): Single; inline;
    begin
      if a <= b then
        Result := a
      else
        Result := b;
    end;


  function Max2(a, b: Single): Single; inline;
    begin
      if a >= b then
        Result := a
      else
        Result := b;
    end;

  var
    v1,v3,vNaN : Single;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(1);
    if Max1(1,3)<>3 then
      halt(2);
    if Min2(1,3)<>1 then
      halt(3);
    if Max2(1,3)<>3 then
      halt(4);
    if Min1(1,v3)<>1 then
      halt(11);
    if Max1(1,v3)<>3 then
      halt(12);
    if Min2(1,v3)<>1 then
      halt(13);
    if Max2(1,v3)<>3 then
      halt(14);
    if Min1(1,v3)<>1 then
      halt(21);
    if Max1(1,v3)<>v3 then
      halt(22);
    if Min2(1,v3)<>1 then
      halt(23);
    if Max2(1,v3)<>v3 then
      halt(24);
    if Min1(v1,v3)<>v1 then
      halt(31);
    if Max1(v1,v3)<>v3 then
      halt(32);
    if Min2(v1,v3)<>v1 then
      halt(33);
    if Max2(v1,v3)<>v3 then
      halt(34);
    SetExceptionMask([exInvalidOp]);
    vNaN:=NaN;
    if not(IsNaN(Min1(v1,vNaN))) then
      halt(41);
    if Min1(NaN,v1)<>v1 then
      halt(42);
    if not(IsNaN(Max1(v1,vNaN))) then
      halt(43);
    if Max1(vNaN,v3)<>v3 then
      halt(44);
    if not(IsNaN(Min2(v1,vNaN))) then
      halt(45);
    if Min2(vNaN,v3)<>v3 then
      halt(46);
    if not(IsNaN(Max2(v1,vNaN))) then
      halt(47);
    if Max2(vNaN,v3)<>v3 then
      halt(48);
    SetExceptionMask([]);
  end;

procedure TestDouble;

  function Min1(a, b: Double): Double; inline;
    begin
      if a < b then
        Result := a
      else
        Result := b;
    end;


  function Max1(a, b: Double): Double; inline;
    begin
      if a > b then
        Result := a
      else
        Result := b;
    end;

  function Min2(a, b: Double): Double; inline;
    begin
      if a <= b then
        Result := a
      else
        Result := b;
    end;


  function Max2(a, b: Double): Double; inline;
    begin
      if a >= b then
        Result := a
      else
        Result := b;
    end;

  var
    v1,v3,vNaN : Double;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(1);
    if Max1(1,3)<>3 then
      halt(2);
    if Min2(1,3)<>1 then
      halt(3);
    if Max2(1,3)<>3 then
      halt(4);
    if Min1(1,v3)<>1 then
      halt(111);
    if Max1(1,v3)<>3 then
      halt(112);
    if Min2(1,v3)<>1 then
      halt(113);
    if Max2(1,v3)<>3 then
      halt(114);
    if Min1(1,v3)<>1 then
      halt(121);
    if Max1(1,v3)<>v3 then
      halt(122);
    if Min2(1,v3)<>1 then
      halt(123);
    if Max2(1,v3)<>v3 then
      halt(124);
    if Min1(v1,v3)<>v1 then
      halt(131);
    if Max1(v1,v3)<>v3 then
      halt(132);
    if Min2(v1,v3)<>v1 then
      halt(133);
    if Max2(v1,v3)<>v3 then
      halt(134);
    SetExceptionMask([exInvalidOp]);
    vNaN:=NaN;
    if not(IsNaN(Min1(v1,vNaN))) then
      halt(141);
    if Min1(NaN,v1)<>v1 then
      halt(142);
    if not(IsNaN(Max1(v1,vNaN))) then
      halt(143);
    if Max1(vNaN,v3)<>v3 then
      halt(144);
    if not(IsNaN(Min2(v1,vNaN))) then
      halt(145);
    if Min2(vNaN,v3)<>v3 then
      halt(146);
    if not(IsNaN(Max2(v1,vNaN))) then
      halt(147);
    if Max2(vNaN,v3)<>v3 then
      halt(148);
    SetExceptionMask([]);
  end;


procedure TestDWord;

  function Min1(a, b: DWord): DWord; inline;
    begin
      if a < b then
        Result := a
      else
        Result := b;
    end;


  function Max1(a, b: DWord): DWord; inline;
    begin
      if a > b then
        Result := a
      else
        Result := b;
    end;

  function Min2(a, b: DWord): DWord; inline;
    begin
      if a <= b then
        Result := a
      else
        Result := b;
    end;


  function Max2(a, b: DWord): DWord; inline;
    begin
      if a >= b then
        Result := a
      else
        Result := b;
    end;

  var
    v1,v3 : DWord;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(1);
    if Max1(1,3)<>3 then
      halt(2);
    if Min2(1,3)<>1 then
      halt(3);
    if Max2(1,3)<>3 then
      halt(4);
    if Min1(1,v3)<>1 then
      halt(211);
    if Max1(1,v3)<>3 then
      halt(212);
    if Min2(1,v3)<>1 then
      halt(213);
    if Max2(1,v3)<>3 then
      halt(214);
    if Min1(1,v3)<>1 then
      halt(221);
    if Max1(1,v3)<>v3 then
      halt(222);
    if Min2(1,v3)<>1 then
      halt(223);
    if Max2(1,v3)<>v3 then
      halt(224);
    if Min1(v1,v3)<>v1 then
      halt(231);
    if Max1(v1,v3)<>v3 then
      halt(232);
    if Min2(v1,v3)<>v1 then
      halt(233);
    if Max2(v1,v3)<>v3 then
      halt(234);
  end;

procedure TestLongint;

  function Min1(a, b: Longint): Longint; inline;
    begin
      if a < b then
        Result := a
      else
        Result := b;
    end;


  function Max1(a, b: Longint): Longint; inline;
    begin
      if a > b then
        Result := a
      else
        Result := b;
    end;

  function Min2(a, b: Longint): Longint; inline;
    begin
      if a <= b then
        Result := a
      else
        Result := b;
    end;


  function Max2(a, b: Longint): Longint; inline;
    begin
      if a >= b then
        Result := a
      else
        Result := b;
    end;

  var
    v1,v3 : Longint;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(1);
    if Max1(1,3)<>3 then
      halt(2);
    if Min2(1,3)<>1 then
      halt(3);
    if Max2(1,3)<>3 then
      halt(4);
    if Min1(1,v3)<>1 then
      halt(311);
    if Max1(1,v3)<>3 then
      halt(312);
    if Min2(1,v3)<>1 then
      halt(313);
    if Max2(1,v3)<>3 then
      halt(314);
    if Min1(1,v3)<>1 then
      halt(321);
    if Max1(1,v3)<>v3 then
      halt(322);
    if Min2(1,v3)<>1 then
      halt(323);
    if Max2(1,v3)<>v3 then
      halt(324);
    v1:=1;
    if Min1(v1,v3)<>v1 then
      halt(331);
    if Max1(v1,v3)<>v3 then
      halt(332);
    if Min2(v1,v3)<>v1 then
      halt(333);
    if Max2(v1,v3)<>v3 then
      halt(334);
  end;

begin
  TestSingle;
  TestDWord;
  TestLongint;
end.
