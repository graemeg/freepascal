var
  d0,d1,d2,d3 : double;
  s0,s1,s2,s3 : single;

procedure testsingle;
  var
    l0,l1,l2,l3 : single;
  begin
    l1:=2;
    l2:=3;
    l3:=4;
    s0:=0;

    l0:=FMASingle(l1,l2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(l1+1.0,l2,l3);
    writeln(l0);
    if l0<>13.0 then
      halt(1);

    l0:=FMASingle(l1,l1+1.0,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(s1,l2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(l1,s2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(l1,l2,s3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(s1,s2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(s1,l2,s3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMASingle(l1,s2,s3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    { first operand negative }
    l0:=FMASingle(-l1,l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-(l1+1.0),l2,l3);
    writeln(l0);
    if l0<>-5.0 then
      halt(1);

    l0:=FMASingle(-l1,l1+1.0,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-s1,l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-l1,s2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-l1,l2,s3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-s1,s2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-s1,l2,s3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(-l1,s2,s3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    { second operand negative }
    l0:=FMASingle(l1,-l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(l1+1.0,-l2,l3);
    writeln(l0);
    if l0<>-5.0 then
      halt(1);

    l0:=FMASingle(l1,-(l1+1.0),l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(s1,-l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(l1,-s2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(l1,-l2,s3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(s1,-s2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(s1,-l2,s3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMASingle(l1,-s2,s3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    { third operand negative }
    l0:=FMASingle(l1,l2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(l1+1.0,l2,-l3);
    writeln(l0);
    if l0<>5.0 then
      halt(1);

    l0:=FMASingle(l1,l1+1.0,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(s1,l2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(l1,s2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(l1,l2,-s3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(s1,s2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(s1,l2,-s3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMASingle(l1,s2,-s3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    { first and third operand negative }
    l0:=FMASingle(-l1,l2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-(l1+1.0),l2,-l3);
    writeln(l0);
    if l0<>-13.0 then
      halt(1);

    l0:=FMASingle(-l1,l1+1.0,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-s1,l2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-l1,s2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-l1,l2,-s3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-s1,s2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-s1,l2,-s3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMASingle(-l1,s2,-s3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);
  end;

{$ifndef NODOUBLE}

procedure testdouble;
  var
    l0,l1,l2,l3 : double;
  begin
    l1:=2;
    l2:=3;
    l3:=4;
    d0:=0;

    l0:=FMADouble(l1,l2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(l1+1.0,l2,l3);
    writeln(l0);
    if l0<>13.0 then
      halt(1);

    l0:=FMADouble(l1,l1+1.0,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(d1,l2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(l1,d2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(l1,l2,d3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(d1,d2,l3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(d1,l2,d3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    l0:=FMADouble(l1,d2,d3);
    writeln(l0);
    if l0<>10.0 then
      halt(1);

    { first operand negative }
    l0:=FMADouble(-l1,l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-(l1+1.0),l2,l3);
    writeln(l0);
    if l0<>-5.0 then
      halt(1);

    l0:=FMADouble(-l1,l1+1.0,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-d1,l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-l1,d2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-l1,l2,d3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-d1,d2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-d1,l2,d3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(-l1,d2,d3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    { second operand negative }
    l0:=FMADouble(l1,-l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(l1+1.0,-l2,l3);
    writeln(l0);
    if l0<>-5.0 then
      halt(1);

    l0:=FMADouble(l1,-(l1+1.0),l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(d1,-l2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(l1,-d2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(l1,-l2,d3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(d1,-d2,l3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(d1,-l2,d3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    l0:=FMADouble(l1,-d2,d3);
    writeln(l0);
    if l0<>-2.0 then
      halt(1);

    { third operand negative }
    l0:=FMADouble(l1,l2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(l1+1.0,l2,-l3);
    writeln(l0);
    if l0<>5.0 then
      halt(1);

    l0:=FMADouble(l1,l1+1.0,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(d1,l2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(l1,d2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(l1,l2,-d3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(d1,d2,-l3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(d1,l2,-d3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    l0:=FMADouble(l1,d2,-d3);
    writeln(l0);
    if l0<>2.0 then
      halt(1);

    { first and third operand negative }
    l0:=FMADouble(-l1,l2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-(l1+1.0),l2,-l3);
    writeln(l0);
    if l0<>-13.0 then
      halt(1);

    l0:=FMADouble(-l1,l1+1.0,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-d1,l2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-l1,d2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-l1,l2,-d3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-d1,d2,-l3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-d1,l2,-d3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);

    l0:=FMADouble(-l1,d2,-d3);
    writeln(l0);
    if l0<>-10.0 then
      halt(1);
  end;

{$endif NODOUBLE}
