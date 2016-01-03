{ %VERSION=1.1}
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;
var
   p : pwidechar;
   c1,c2 : widechar;
   i : longint;
   a : ansistring;
   w : widestring;
   err : boolean;

const somestr : pwidechar = 'blaat';

begin
   p:=@c1;
   i:=0;
   c2:=p[i];

   w:='hello';
   a:=w;

   writeln(a);
   if a<>'hello' then
    err:=true;
   writeln(w);
   if w<>'hello' then
    err:=true;

   p:='';
   p:='hello';
   writeln(widestring(p));
   if widestring(p)<>'hello' then
    err:=true;

   if err then
    halt(1);
end.
