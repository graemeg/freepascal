uses
{$ifdef unix}
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cwstring,
>>>>>>> graemeg/cpstrnew
{$endif unix}
  SysUtils;

type
<<<<<<< HEAD
  ts866 = type AnsiString(866);
=======
  ts866 = type string<866>;
>>>>>>> graemeg/cpstrnew
var
  a866 : ts866;
begin
  a866 := '';
  SetLength(a866,10);
  if (StringCodePage(a866) <> 866) then
    halt(1);
  WriteLn('ok');
end.
