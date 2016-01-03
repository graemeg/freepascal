{ %target=linux,solaris,freebsd,win32 }
{ %fail }
<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-vw -Sew }
=======
{ %opt=-Sew }
>>>>>>> graemeg/fixes_2_2
=======
{ %opt=-Sew }
>>>>>>> origin/fixes_2_2

{ just some random non-darwin targets }

{$linkframework Carbon}

type
  CFStringRef = pointer;

function CFSTR( c: PChar ): CFStringRef; external name '___CFStringMakeConstantString'; mwpascal;

begin
  CFSTR('hello');  
end.
