{ %target=linux }
uses
  baseunix,linux,ctypes;
  
var
  e : tepoll_event;
  es : array[0..10] of tepoll_event;
  fd : cint;
  i : Longint;
begin
  fillchar(es,sizeof(es),$de);
  fd:=epoll_create(1);
  
  e.Events:=EPOLLIN;
  e.Data.u32:=$1234568;
  if (epoll_ctl(fd,EPOLL_CTL_ADD,0,@e)<>0) then
    begin
      writeln('Error in epoll_ctl');
      fpclose(fd);
      halt(1);
    end;
  i:=epoll_wait(fd,@es,length(es),100);
  fpclose(fd);
end.
