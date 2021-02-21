{$mode objfpc}

uses
  StrUtils;
const
  result1 : array of SizeInt = (1, 4, 7, 10, 13, 16);
var 
  a : array of SizeInt;
  i : LongInt;
begin
  if FindMatchesBoyerMooreCaseSensitive('abcabcabcabcabcabcab','abcab',a,false) then
    begin
      if Length(a)<>1 then
        halt(2);
      if a[0]<>result1[0] then
        halt(3);
    end
  else
    halt(1);

  if FindMatchesBoyerMooreCaseSensitive('abcabcabcabcabcabcab','abcab',a,true) then
    begin
      if Length(a)<>Length(result1) then
        halt(12);
      for i:=Low(a) to High(a) do
        if a[i]<>result1[i] then
          halt(13);
    end
  else
    halt(11);

  if FindMatchesBoyerMooreCaseInSensitive('abcabcabcabcabcabcab','abcab',a,false) then
    begin
      if Length(a)<>1 then
        halt(22);
      if a[0]<>result1[0] then
        halt(23);
    end
  else
    halt(21);

{
  apparently not working yet:
  
  if FindMatchesBoyerMooreCaseInSensitive('abcabcabcabcabcabcab','abcab',a,true) then
    begin
      if Length(a)<>Length(result1) then
        halt(32);
      for i:=Low(a) to High(a) do
        if a[i]<>result1[i] then
          halt(33);
    end
  else
    halt(31);

  if FindMatchesBoyerMooreCaseInSensitive('abcabcabcAbcabcAbcab','abcaB',a,false) then
    begin
      if Length(a)<>1 then
        halt(42);
      if a[0]<>result1[0] then
        halt(43);
    end
  else
    halt(41);

  if FindMatchesBoyerMooreCaseInSensitive('abcabCabcAbcabcABcab','abcaB',a,true) then
    begin
      if Length(a)<>Length(result1) then
        halt(52);
      for i:=Low(a) to High(a) do
        if a[i]<>result1[i] then
          halt(53);
    end
  else
    halt(51);
}

  writeln('ok');
end.
