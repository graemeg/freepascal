{ %target=linux }
uses
  ctypes,baseunix,linux;

var
  un : utsname;
  mystatx : tstatx;
  res : cint;
  f : text;
  st,major,minor : string;
  i,p,e : longint;
  err : word;
  major_release, minor_release : longint;
begin
  fpuname(un);
  st:=un.release;
  for i:=1 to UTSNAME_LENGTH do
    if st[i]='.' then
      begin
        p:=i;
        major:=system.copy(st,1,p-1);
        system.val(major,major_release,err);
        if err<>0 then
          begin
            writeln('Unable to parse first part of linux version ',st,'(',major,') correctly');
            halt(2);
          end;
        break;
      end;

  for i:=p+1 to UTSNAME_LENGTH do
    if st[i]='.' then
      begin
        e:=i;
        minor:=system.copy(st,p+1,e-p-1);
        system.val(minor,minor_release,err);
        if err<>0 then
          begin
            writeln('Unable to second part of parse linux version ',st,'i(',minor,') correctly');
            halt(2);
          end;
        break;
      end;
  if (major_release<4) or ((major_release=4) and (minor_release<11)) then
    begin
      writeln('This version of Linux: ',st,' does not have fstatx syscall');
      halt(0);
    end
  else
    writeln('This linux version ',st,' should support statx syscall');

  assign(f,'test.txt');
  rewrite(f);
  write(f,'ccccc');
  close(f);
  res:=statx(AT_FDCWD,'test.txt',AT_SYMLINK_NOFOLLOW,STATX_ALL,mystatx);
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
