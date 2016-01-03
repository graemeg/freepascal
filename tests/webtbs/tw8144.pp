
function DoCheck(Key:WideChar):boolean;
begin
 DoCheck:=(Key in [WideChar(#0), WideChar(#8), WideChar(#10),
<<<<<<< HEAD
<<<<<<< HEAD
    WideChar(#13), WideChar(#27)
{$IFNDEF OS2}
{ OS/2 Unicode support uses #127 as replacement character for translation
  of characters which do not exist in the current system codepage. }
                                , WideChar(#127)
{$ENDIF OS2}
                                                ]);
=======
    WideChar(#13), WideChar(#27), WideChar(#127)]);
>>>>>>> graemeg/fixes_2_2
=======
    WideChar(#13), WideChar(#27), WideChar(#127)]);
>>>>>>> origin/fixes_2_2
end;

var
  Key:WideChar;
  err : boolean;
begin
  for Key:=WideChar(128) to WideChar(2048) do
   if DoCheck(Key) then
    begin
     writeln(word(Key),' (',Key,') in set');
     err:=true;
    end;
  if err then halt(1);
end.
