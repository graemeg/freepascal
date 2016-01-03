
<<<<<<< HEAD
<<<<<<< HEAD
{ This test requires overflow and range check to be off }
{$Q-}
{$R-}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure error(n: longint);
begin
  writeln('Test failed: ', n);
  Halt(n);
end;

var
  b,b2: byte;
  w,w2: word;
  c,c2: cardinal;
  shi,shi2: shortint;
  si,si2: smallint;
  i64: int64;
begin
  b:=$ff;
  Inc(b,$ff);
  if shortint(b)<>-2 then
    error(1);
  if smallint(b)<>$fe then
    error(2);
  if word(b)<>$fe then
    error(3);
  if longint(b)<>$fe then
    error(4);
  if cardinal(b)<>$fe then
    error(5);
  if int64(b)<>$fe then
    error(6);
{$ifdef FPC}
  if qword(b)<>$fe then
    error(7);
{$endif FPC}
  b2:=$fe;
  if b<>b2 then
    error(8);

  w:=$8000;
  if shortint(w)<>0 then
    error(8);

  w:=$ffff;
  Inc(w,$ffff);
  if shortint(w)<>-2 then
    error(11);
  if byte(w)<>$fe then
    error(12);
  if smallint(w)<>-2 then
    error(13);
  if longint(w)<>$fffe then
    error(14);
  if cardinal(w)<>$fffe then
    error(15);
  if int64(w)<>$fffe then
    error(16);
{$ifdef FPC}
  if qword(w)<>$fffe then
    error(17);
{$endif FPC}
  w2:=$fffe;
  if w<>w2 then
    error(18);

  c:=$ffffffff;
  Inc(c,$ffffffff);
  if int64(c)<>$fffffffe then
    error(21);
{$ifdef FPC}
  if qword(c)<>$fffffffe then
    error(22);
{$endif FPC}
  c2:=$fffffffe;
  if c<>c2 then
    error(23);

  shi:=$7f;
  Inc(shi,$7f);
  if word(shi)<>$fffe then
    error(31);
  if cardinal(shi)<>$fffffffe then
    error(32);
  i64:=cardinal(shi);
  if i64<>$fffffffe then
    error(33);
{$ifdef FPC}
  if qword(shi)<>qword($fffffffffffffffe) then
    error(34);
{$endif FPC}
  shi2:=-2;
  if shi<>shi2 then
    error(35);

  si:=$7fff;
  Inc(si,$7fff);
  if word(si)<>$fffe then
    error(41);
  if cardinal(si)<>$fffffffe then
    error(42);
  i64:=cardinal(si);
  if i64<>$fffffffe then
    halt(43);
{$ifdef FPC}
  if qword(si)<>qword($fffffffffffffffe) then
    error(44);
{$endif FPC}
  si2:=-2;
  if si<>si2 then
    error(45);

  writeln('Test OK.');
end.
