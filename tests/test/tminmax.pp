{ %opt=-Oonofastmath }  { with fast math, the operands of min/max might be swapped and this breaks the tests using NaN }

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


  function Min3(a, b: Single): Single; inline;
    begin
      Result := b;
      if a < b then
        Result := a;
    end;


  function Max3(a, b: Single): Single; inline;
    begin
      Result := b;
      if a > b then
        Result := a;
    end;


  function Min4(a, b: Single): Single; inline;
    begin
      Result := b;
      if a <= b then
        Result := a;
    end;


  function Max4(a, b: Single): Single; inline;
    begin
      Result := b;
      if a >= b then
        Result := a;
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
    if Min3(1,3)<>1 then
      halt(5);
    if Max3(1,3)<>3 then
      halt(6);
    if Min3(1,3)<>1 then
      halt(7);
    if Max3(1,3)<>3 then
      halt(8);
    if Min1(1,v3)<>1 then
      halt(11);
    if Max1(1,v3)<>3 then
      halt(12);
    if Min2(1,v3)<>1 then
      halt(13);
    if Max2(1,v3)<>3 then
      halt(14);
    if Min3(1,v3)<>1 then
      halt(15);
    if Max3(1,v3)<>3 then
      halt(16);
    if Min4(1,v3)<>1 then
      halt(17);
    if Max4(1,v3)<>3 then
      halt(18);
    if Min1(1,v3)<>1 then
      halt(21);
    if Max1(1,v3)<>v3 then
      halt(22);
    if Min2(1,v3)<>1 then
      halt(23);
    if Max2(1,v3)<>v3 then
      halt(24);
    if Min3(1,v3)<>1 then
      halt(25);
    if Max3(1,v3)<>v3 then
      halt(26);
    if Min4(1,v3)<>1 then
      halt(27);
    if Max4(1,v3)<>v3 then
      halt(28);
    if Min1(v1,v3)<>v1 then
      halt(31);
    if Max1(v1,v3)<>v3 then
      halt(32);
    if Min2(v1,v3)<>v1 then
      halt(33);
    if Max2(v1,v3)<>v3 then
      halt(34);
    if Min3(v1,v3)<>v1 then
      halt(35);
    if Max3(v1,v3)<>v3 then
      halt(36);
    if Min4(v1,v3)<>v1 then
      halt(37);
    if Max4(v1,v3)<>v3 then
      halt(38);
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
    if not(IsNaN(Min3(v1,vNaN))) then
      halt(49);
    if Min3(NaN,v1)<>v1 then
      halt(50);
    if not(IsNaN(Max3(v1,vNaN))) then
      halt(51);
    if Max3(vNaN,v3)<>v3 then
      halt(52);
    if not(IsNaN(Min4(v1,vNaN))) then
      halt(53);
    if Min4(vNaN,v3)<>v3 then
      halt(54);
    if not(IsNaN(Max4(v1,vNaN))) then
      halt(55);
    if Max4(vNaN,v3)<>v3 then
      halt(56);
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


  function Min3(a, b: Double): Double; inline;
    begin
      Result := b;
      if a < b then
        Result := a;
    end;


  function Max3(a, b: Double): Double; inline;
    begin
      Result := b;
      if a > b then
        Result := a;
    end;


  function Min4(a, b: Double): Double; inline;
    begin
      Result := b;
      if a <= b then
        Result := a;
    end;


  function Max4(a, b: Double): Double; inline;
    begin
      Result := b;
      if a >= b then
        Result := a;
    end;

  var
    v1,v3,vNaN : Double;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(101);
    if Max1(1,3)<>3 then
      halt(102);
    if Min2(1,3)<>1 then
      halt(103);
    if Max2(1,3)<>3 then
      halt(104);
    if Min3(1,3)<>1 then
      halt(105);
    if Max3(1,3)<>3 then
      halt(106);
    if Min3(1,3)<>1 then
      halt(107);
    if Max3(1,3)<>3 then
      halt(108);
    if Min1(1,v3)<>1 then
      halt(111);
    if Max1(1,v3)<>3 then
      halt(112);
    if Min2(1,v3)<>1 then
      halt(113);
    if Max2(1,v3)<>3 then
      halt(114);
    if Min3(1,v3)<>1 then
      halt(115);
    if Max3(1,v3)<>3 then
      halt(116);
    if Min4(1,v3)<>1 then
      halt(117);
    if Max4(1,v3)<>3 then
      halt(118);
    if Min1(1,v3)<>1 then
      halt(121);
    if Max1(1,v3)<>v3 then
      halt(122);
    if Min2(1,v3)<>1 then
      halt(123);
    if Max2(1,v3)<>v3 then
      halt(124);
    if Min3(1,v3)<>1 then
      halt(125);
    if Max3(1,v3)<>v3 then
      halt(126);
    if Min4(1,v3)<>1 then
      halt(127);
    if Max4(1,v3)<>v3 then
      halt(128);
    if Min1(v1,v3)<>v1 then
      halt(131);
    if Max1(v1,v3)<>v3 then
      halt(132);
    if Min2(v1,v3)<>v1 then
      halt(133);
    if Max2(v1,v3)<>v3 then
      halt(134);
    if Min3(v1,v3)<>v1 then
      halt(135);
    if Max3(v1,v3)<>v3 then
      halt(136);
    if Min4(v1,v3)<>v1 then
      halt(137);
    if Max4(v1,v3)<>v3 then
      halt(138);
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
    if not(IsNaN(Min3(v1,vNaN))) then
      halt(149);
    if Min3(NaN,v1)<>v1 then
      halt(150);
    if not(IsNaN(Max3(v1,vNaN))) then
      halt(151);
    if Max3(vNaN,v3)<>v3 then
      halt(152);
    if not(IsNaN(Min4(v1,vNaN))) then
      halt(153);
    if Min4(vNaN,v3)<>v3 then
      halt(154);
    if not(IsNaN(Max4(v1,vNaN))) then
      halt(155);
    if Max4(vNaN,v3)<>v3 then
      halt(156);
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


  function Min3(a, b: DWord): DWord; inline;
    begin
      Result := b;
      if a < b then
        Result := a;
    end;


  function Max3(a, b: DWord): DWord; inline;
    begin
      Result := b;
      if a > b then
        Result := a;
    end;


  function Min4(a, b: DWord): DWord; inline;
    begin
      Result := b;
      if a <= b then
        Result := a;
    end;


  function Max4(a, b: Double): Double; inline;
    begin
      Result := b;
      if a >= b then
        Result := a;
    end;

  var
    v1,v3 : DWord;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(201);
    if Max1(1,3)<>3 then
      halt(202);
    if Min2(1,3)<>1 then
      halt(203);
    if Max2(1,3)<>3 then
      halt(204);
    if Min3(1,3)<>1 then
      halt(205);
    if Max3(1,3)<>3 then
      halt(206);
    if Min3(1,3)<>1 then
      halt(207);
    if Max3(1,3)<>3 then
      halt(208);
    if Min1(1,v3)<>1 then
      halt(211);
    if Max1(1,v3)<>3 then
      halt(212);
    if Min2(1,v3)<>1 then
      halt(213);
    if Max2(1,v3)<>3 then
      halt(214);
    if Min3(1,v3)<>1 then
      halt(215);
    if Max3(1,v3)<>3 then
      halt(216);
    if Min4(1,v3)<>1 then
      halt(217);
    if Max4(1,v3)<>3 then
      halt(218);
    if Min1(1,v3)<>1 then
      halt(221);
    if Max1(1,v3)<>v3 then
      halt(222);
    if Min2(1,v3)<>1 then
      halt(223);
    if Max2(1,v3)<>v3 then
      halt(224);
    if Min3(1,v3)<>1 then
      halt(225);
    if Max3(1,v3)<>v3 then
      halt(226);
    if Min4(1,v3)<>1 then
      halt(227);
    if Max4(1,v3)<>v3 then
      halt(228);
    if Min1(v1,v3)<>v1 then
      halt(231);
    if Max1(v1,v3)<>v3 then
      halt(232);
    if Min2(v1,v3)<>v1 then
      halt(233);
    if Max2(v1,v3)<>v3 then
      halt(234);
    if Min3(v1,v3)<>v1 then
      halt(235);
    if Max3(v1,v3)<>v3 then
      halt(236);
    if Min4(v1,v3)<>v1 then
      halt(237);
    if Max4(v1,v3)<>v3 then
      halt(238);
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


  function Min3(a, b: Longint): Longint; inline;
    begin
      Result := b;
      if a < b then
        Result := a;
    end;


  function Max3(a, b: Longint): Longint; inline;
    begin
      Result := b;
      if a > b then
        Result := a;
    end;


  function Min4(a, b: Longint): Longint; inline;
    begin
      Result := b;
      if a <= b then
        Result := a;
    end;


  function Max4(a, b: Longint): Longint; inline;
    begin
      Result := b;
      if a >= b then
        Result := a;
    end;

  var
    v1,v3 : Longint;

  begin
    v1:=1;
    v3:=3;
    if Min1(1,3)<>1 then
      halt(301);
    if Max1(1,3)<>3 then
      halt(302);
    if Min2(1,3)<>1 then
      halt(303);
    if Max2(1,3)<>3 then
      halt(304);
    if Min3(1,3)<>1 then
      halt(305);
    if Max3(1,3)<>3 then
      halt(306);
    if Min3(1,3)<>1 then
      halt(307);
    if Max3(1,3)<>3 then
      halt(308);
    if Min1(1,v3)<>1 then
      halt(311);
    if Max1(1,v3)<>3 then
      halt(312);
    if Min2(1,v3)<>1 then
      halt(313);
    if Max2(1,v3)<>3 then
      halt(314);
    if Min3(1,v3)<>1 then
      halt(315);
    if Max3(1,v3)<>3 then
      halt(316);
    if Min4(1,v3)<>1 then
      halt(317);
    if Max4(1,v3)<>3 then
      halt(318);
    if Min1(1,v3)<>1 then
      halt(321);
    if Max1(1,v3)<>v3 then
      halt(322);
    if Min2(1,v3)<>1 then
      halt(323);
    if Max2(1,v3)<>v3 then
      halt(324);
    if Min3(1,v3)<>1 then
      halt(325);
    if Max3(1,v3)<>v3 then
      halt(326);
    if Min4(1,v3)<>1 then
      halt(327);
    if Max4(1,v3)<>v3 then
      halt(328);
    if Min1(v1,v3)<>v1 then
      halt(331);
    if Max1(v1,v3)<>v3 then
      halt(332);
    if Min2(v1,v3)<>v1 then
      halt(333);
    if Max2(v1,v3)<>v3 then
      halt(334);
    if Min3(v1,v3)<>v1 then
      halt(335);
    if Max3(v1,v3)<>v3 then
      halt(336);
    if Min4(v1,v3)<>v1 then
      halt(337);
    if Max4(v1,v3)<>v3 then
      halt(338);
  end;

begin
  TestSingle;
  TestDWord;
  TestLongint;
end.
