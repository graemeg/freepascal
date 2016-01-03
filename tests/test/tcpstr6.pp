{$CODEPAGE cp1251}
// file encoding is cp1251
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Cp866String = type AnsiString(866);
  Cp1251String = type AnsiString(1251);
=======
  Cp866String = string<866>;
  Cp1251String = string<1251>;
>>>>>>> graemeg/cpstrnew
=======
  Cp866String = string<866>;
  Cp1251String = string<1251>;
>>>>>>> graemeg/cpstrnew
=======
  Cp866String = string<866>;
  Cp1251String = string<1251>;
>>>>>>> graemeg/cpstrnew
=======
  Cp866String = string<866>;
  Cp1251String = string<1251>;
>>>>>>> origin/cpstrnew

procedure WriteString(const s: RawByteString);
begin
  Write(StringCodePage(s), ' : ');
  WriteLn(s);
end;

var
  u : UnicodeString;
  c1251: Cp1251String;
  c866: Cp866String;
begin
  c1251 := 'Привет';
  WriteString(c1251);
  u := c1251;
  WriteString(u);
  c866 := c1251;
  c866 := c866 + c1251;
  WriteString(c866);
end.

