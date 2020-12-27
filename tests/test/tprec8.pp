{$bitpacking on}

type
  tenum = (ea,eb,ec,ed,ee,ef,eg,eh);
  tr = packed record
         a: 0..3;    // 2 bits
         i: int64;
         c: boolean; // 1 bit
         d: 0..31;   // 5 bits
         e: tenum;   // 3 bits
         case byte of
           0: (g: 0..7);
           1: (h: 0..65536; k: boolean);
           2: (j: boolean);
       end;

procedure t(var r2: tr);
var
  r: tr;
begin
  r.a := 2;
  r.i := 1234567890123456789;
  r.c := true;
  r.d := 5;
  r.e := ed;
  r2 := r;
end;

var
  r: tr;
  b: byte;
begin
  b := 0;
  t(r);
  if (r.a <> 2) or
     (r.i <> 1234567890123456789) or
     (not r.c) or
     (r.d <> 5) or
     (r.e <> ed) then
    halt(1);
  r.g := 5;
  if (r.g <> 5) then
    halt(2);
  r.h := 65535;
  if (r.h <> 65535) then
    halt(3);
  r.k := true;
  if not (r.k) then
    halt(4);
  r.j := false;
  if r.j then
    halt(5);
  if b <> 0 then
    halt(6);
  if sizeof(tr) <> 13 then
    halt(7);
end.

