{ %target=linux,darwin,freebsd,netbsd,openbsd,sunos,beos,haiku }
{ %cpu=x86_64,powerpc64,mips64,sparc64,ia64,alpha }
{ %opt=-Xa }

{ windows does not support statics > 2GB }
var
  i : longint;
  a : array[0..1500000000] of longint;
begin
  writeln(a[i]);
end.
