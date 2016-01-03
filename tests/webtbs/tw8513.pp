type
  TMyType = cardinal;
  tr = record
    a,b,c,d: byte;
  end;

procedure t(var l: cardinal);
begin
  if (l <> $cafebabe) then
    halt(4);
  l := $c001d00d;
end;

var
  Item: TMyType;
  ItemAsByte: byte absolute Item;
<<<<<<< HEAD
<<<<<<< HEAD
  ItemAsWord: word absolute Item;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  r: tr;
  b: byte absolute r.b;

  l: cardinal;
  labs: cardinal absolute l;
begin
  { Of course I understand fully that this code is bad
    (unless you really want to read the 1st byte of 4-byte LongInt
    type, messing with endianess problems).

    In real code, I accessed ItemAsByte only when
    SizeOf(TMyType) = 1 (the code is
    used like a simple template, so it must work with any
    TMyType, and the case when SizeOf(TMyType) = 1 uses some
    specially optimized versions (e.g. FillChar(..., ItemAsByte)
    can be used in this case to fill the array of TMyType). }

{$ifdef FPC_BIG_ENDIAN}
  item:=$deadbeef;
{$else}
  item:=$efbeadde;
{$endif}
  if (itemasbyte <> $de) then
    halt(1);

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_BIG_ENDIAN}
  if (itemasword <> $dead) then
{$else}
  if (itemasword <> $adde) then
{$endif}
    halt(3);

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  r.a := $de;
  r.b := $ad;
  r.c := $be;
  r.d := $ef;
  if (b <> $ad) then
    halt(2);

  l := $cafebabe;
  t(labs);
  if (l <> $c001d00d) then
    halt(6);
end.
