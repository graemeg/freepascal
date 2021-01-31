{ %target=linux }
uses
  ctypes,baseunix,linux;

var
  un : utsname;
  res : cint;
  f1,f2 : text;
  err : word;
  mystatx1,mystatx2 : tstatx;
  times : tkernel_timespecs;
  st,major,minor : string;
  i,p,e : longint;
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

  assign(f1,'tutimensat1.txt');
  rewrite(f1);
  write(f1,'ccccc');
  close(f1);
  assign(f2,'tutimensat2.txt');
  rewrite(f2);
  write(f2,'ccccc');
  close(f2);

  res:=statx(AT_FDCWD,'tutimensat1.txt',AT_SYMLINK_NOFOLLOW,STATX_ALL,mystatx1);
  if res<>0 then
    halt(1);
  times[0].tv_sec:=mystatx1.stx_atime.tv_sec;
  times[0].tv_nsec:=mystatx1.stx_atime.tv_nsec;
  times[1].tv_sec:=mystatx1.stx_mtime.tv_sec;
  times[1].tv_nsec:=mystatx1.stx_mtime.tv_nsec;
  res:=utimensat(AT_FDCWD,'tutimensat2.txt',times,0);
  if res<>0 then
    halt(1);
  res:=statx(AT_FDCWD,'tutimensat2.txt',AT_SYMLINK_NOFOLLOW,STATX_ALL,mystatx2);
  if res<>0 then
    halt(1);

  erase(f1);
  erase(f2);

  if (mystatx1.stx_atime.tv_sec<>mystatx2.stx_atime.tv_sec) or (mystatx1.stx_atime.tv_nsec<>mystatx2.stx_atime.tv_nsec) or
    (mystatx1.stx_mtime.tv_sec<>mystatx2.stx_mtime.tv_sec) or (mystatx1.stx_mtime.tv_nsec<>mystatx2.stx_mtime.tv_nsec) then
    halt(1);
  writeln('ok');
end.
