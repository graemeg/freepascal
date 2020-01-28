{ %target=linux,freebsd,openbsd,aix,darwin,netbsd }
uses
  unixutil;
var
  y,m,d,h,mn,s : word;
begin
  EpochToLocal(15796372693,y,m,d,h,mn,s);
  if (y<>2470) or (m<>7) or (d<>26) or (h<>9) or (mn<>18) or (s<>13) then
    halt(1);
  EpochToLocal(LocalToEpoch(2345,12,12,4,45,49),y,m,d,h,mn,s);
  if (y<>2345) or (m<>12) or (d<>12) or (h<>4) or (mn<>45) or (s<>49) then
    halt(1);
  writeln('ok');
end.
