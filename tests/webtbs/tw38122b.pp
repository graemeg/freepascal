program tw38122b;
{$mode delphi}
uses sysutils;
type trec=record
  i:integer;
 end;

 var rec:trec;
     prec:^trec;
     s: string;
begin
  rec.i:=20;
  prec:=@rec;
  s:=prec.i.tostring;
  //writeln(s);
  if s<>'20' then
    halt(1);
end.

