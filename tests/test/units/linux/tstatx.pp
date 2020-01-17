{ %target=linux }
uses
  ctypes,baseunix,linux;
  
var
  mystatx : statx;
  res : cint;
  f : text;
  
begin
  assign(f,'test.txt');
  rewrite(f);
  write(f,'ccccc');
  close(f);
  res:=fpstatx(AT_FDCWD,'test.txt',AT_SYMLINK_NOFOLLOW,STATX_ALL,mystatx);
  erase(f);
  if res<>0 then
    begin
      halt(1);
    end;
  writeln('statx.stx_mask = %',binstr(mystatx.stx_mask,32));
  writeln('statx.size = ',mystatx.stx_size);
  if mystatx.stx_size<>5 then
    halt(1);
  writeln('statx.mode = %',binstr(mystatx.stx_mode,16));
  writeln('ok');
end.
