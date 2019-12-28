var
   c: currency;
   s: string;
begin
   c:=922337203685.47;
   writeln(c:18:4,' = ', ' Trunc(c*10000)=', Trunc(c*10000)); // expected 9223372036854700, but get -75
   str(trunc(c*10000),s);
   if s<>'9223372036854700' then
     halt(1);
   c:=-92233720368547;
   writeln(c:18:4,' = ', ' Trunc(c*10000)=', Trunc(c*10000)); // expected -922337203685470000, but get 7580
   str(trunc(c*10000),s);
   if s<>'-922337203685470000' then
     halt(1);
   writeln('ok');
end.
