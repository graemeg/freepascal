{ %target=linux,freebsd,openbsd,aix,darwin,netbsd }
uses
  dateutils;
var
  y,m,d,h,mn,s,s1000 : word;
begin
  DecodeDateTime(UnixToDateTime(15796372693),y,m,d,h,mn,s,s1000);
  if (y<>2470) or (m<>7) or (d<>26) or (h<>9) or (mn<>18) or (s<>13) or (s1000<>0) then
    halt(1);
  DecodeDateTime(UnixToDateTime(DateTimeToUnix(EncodeDateTime(2345,12,12,4,45,49,0))),y,m,d,h,mn,s,s1000);
  
  if (y<>2345) or (m<>12) or (d<>12) or (h<>4) or (mn<>45) or (s<>49) or (s1000<>0) then
    halt(1);
  writeln('ok');
end.
