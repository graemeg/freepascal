{$mode objfpc}
{$PIC+}

{
  Test for proper initialization of GOT register.
}

const
  BufSize = 128*1024;

var
  gvar: longint;

procedure check(c, e: longint);
begin
  if c <> e then begin
    writeln('ERROR. Result: ', c, '  Expected: ', e);
    Halt(1);
  end;
end;

function test101(p1, p2: longint): longint;
begin
  result:=gvar+p1+p2;
end;

function test102(p1, p2: longint): longint;
var
  Buffer: array[0..BufSize] of byte;
begin
  Buffer[0]:=0;
  result:=gvar+p1+p2+Buffer[0];
end;

function test103(p1, p2: longint): longint;
var
  a, j: longint;
begin
  a:=0;
  for j:=1 to 1 do begin
    a:=a + j;
    a:=a - j;
  end;
  result:=gvar+p1+p2+a;
end;

function test111(p1, p2: longint): longint;
var
  i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15: longint;
  a, j: longint;
begin
  i1:=1;i2:=2;i3:=3;i4:=4;i5:=5;i6:=6;i7:=7;i8:=8;i9:=9;i10:=10;i11:=11;i12:=12;i13:=13;i14:=14;i15:=15;
  a:=0;
  for j:=1 to 1 do begin
    a:=a + (i1+i2+i3+i4+i5+i6+i7+i8+i9+i10+i11+i12+i13+i14+i15);
    a:=a - (i1+i2+i3+i4+i5+i6+i7+i8+i9+i10+i11+i12+i13+i14+i15);
  end;
  result:=gvar+p1+p2+a;
end;

function test112(p1, p2: longint): longint;
var
  i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15: longint;
  a, j: longint;
  Buffer: array[0..BufSize] of byte;
begin
  i1:=1;i2:=2;i3:=3;i4:=4;i5:=5;i6:=6;i7:=7;i8:=8;i9:=9;i10:=10;i11:=11;i12:=12;i13:=13;i14:=14;i15:=15;
  a:=0;
  Buffer[0]:=0;
  for j:=1 to 1 do begin
    a:=a + (i1+i2+i3+i4+i5+i6+i7+i8+i9+i10+i11+i12+i13+i14+i15);
    a:=a - (i1+i2+i3+i4+i5+i6+i7+i8+i9+i10+i11+i12+i13+i14+i15);
    a:=a + Buffer[0];
  end;
  result:=gvar+p1+p2+a;
end;

begin
  gvar:=100;
  check(test101(10, 20), 130);
  check(test102(20, 30), 150);
  check(test103(30, 40), 170);
  check(test111(110, 20), 230);
  check(test112(120, 30), 250);
end.
