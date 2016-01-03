program test;
{$CODEPAGE UTF8}
// file encoding is UTF8
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  CP866String = type AnsiString(866);
  CP1251String = type AnsiString(1251);
=======
  CP866String = string<866>;
  CP1251String = string<1251>;
>>>>>>> graemeg/cpstrnew
=======
  CP866String = string<866>;
  CP1251String = string<1251>;
>>>>>>> graemeg/cpstrnew
=======
  CP866String = string<866>;
  CP1251String = string<1251>;
>>>>>>> graemeg/cpstrnew
=======
  CP866String = string<866>;
  CP1251String = string<1251>;
>>>>>>> origin/cpstrnew

procedure WriteString(const s: RawByteString);
begin
  Write(StringCodePage(s), ' : ');
  WriteLn(s);
end;

var
  u1: UTF8String;
  s1: CP1251String;
  s2: CP866String;
begin
  u1 := 'мама';
  s1 := u1;
  u1 := ' мыла';
  s2 := u1;
  u1 := ' раму';
  s2 := s1 + u1 + s2;
  WriteString(s2);
end.


