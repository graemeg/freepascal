{ %norun }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %needlibrary }
{ %target=darwin,linux,freebsd,solaris,beos,haiku,aix,android }
=======
{ %target=darwin,linux,freebsd,solaris,beos,haiku }
>>>>>>> graemeg/cpstrnew
=======
{ %target=darwin,linux,freebsd,solaris,beos,haiku }
>>>>>>> graemeg/cpstrnew
=======
{ %target=darwin,linux,freebsd,solaris,beos,haiku }
>>>>>>> graemeg/cpstrnew
=======
{ %target=darwin,linux,freebsd,solaris,beos,haiku }
>>>>>>> origin/cpstrnew

{$mode delphi}

{$ifdef darwin}
{$PIC+}
{$endif darwin}

{$ifdef CPUX86_64}
{$ifndef WINDOWS}
{$PIC+}
{$endif WINDOWS}
{$endif CPUX86_64}

library tw16263a;

function divide(d1,d2: double): double; cdecl;
begin
  divide:=d1/d2;
end;

begin
  // check that the library does not re-enable fpu exceptions
  divide(1.0,0.0);
end.
