program ip6test;

uses
  sockets,
  unix,
  errors,
  netdb,
  baseunix;

var
  dest: TInetSockAddr6;
  sock: LongInt;
  s: shortstring;
  i: integer;
  t1,t2:text;
  x: array of thostaddr6;

begin
  setlength(x, 100);
  setlength(x,resolvename6('www.6bone.net', x));
  if length(x) = 0 then halt(2);
  with dest do begin
    sin6_family := PF_INET6;
    sin6_port   := shorthosttonet(80);
    sin6_addr.u6_addr16 := x[0];
  end;
  sock := socket(PF_INET6, SOCK_STREAM, 6 {TCP});

  if Connect(sock, dest, sizeof(dest)) then begin
    sock2text(sock,t1,t2);
    writeln(t2, 'GET / HTTP/1.0');
    writeln(t2);
    while not eof(t1) do begin
      readln(t1, s);
      writeln(s);
    end;
  end else begin
    writeln('not connected: ',getlasterror, ': ', StrError(getlasterror));
  end;
  closesocket(sock);
end.
