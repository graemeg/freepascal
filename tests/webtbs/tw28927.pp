type
  TRecord1 = record
  end align 16;

  TRecord2 = record
  end align 8;

  TRecord3 = record
  end align 4;

  TRecord1Outer = record  
    b : Byte;
    Record1 : TRecord1;
  end;

  TRecord2Outer = record  
    b : Byte;
    Record2 : TRecord2;
  end;

  TRecord3Outer = record  
    b : Byte;
    Record3 : TRecord3;
  end;

var
  Record1Outer : TRecord1Outer;
  Record2Outer : TRecord2Outer;
  Record3Outer : TRecord3Outer;

begin
  if PtrUInt(@Record1Outer.Record1) mod 16<>0 then
    halt(1);
  if PtrUInt(@Record2Outer.Record2) mod 8<>0 then
    halt(2);
  if PtrUInt(@Record3Outer.Record3) mod 4<>0 then
    halt(3);
  writeln('ok');
end.