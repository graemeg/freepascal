{%CPU=x86_64,powerpc64}
{%skiptarget=darwin,aix,win64}
{ %opt=-Xa }  { use the large option }


{ darwin limits statically declared data structures to 32 bit for efficiency reasons }
{ the aix assembler cannot deal with the way we declare these arrays in assembler code }
{ win64 limits executable image size, as well as sizes of its individual sections, to 32 bit }

program tb0528;

{This program tests if huge arrays work on 64-bit systems. I got the idea
 testing this because today I had a Fortran program that didn't work. After
 inspection it appeared the mad scientist was using arrays > 2GB.

 So, I did a test how well FPC handled such code. Disappointment, FPC
 did generate wrong code.

 Note that this test does not use huge amounts of memory, as the operating
 system should allocate a page on write.
 does not get allocated.}

type huge_array=array[0..$ffffffff] of word;

var a,b,c:huge_array;

begin
  a[$ffffffff]:=1;
  b[$ffffffff]:=2;
  c[$ffffffff]:=3;
  if (a[$ffffffff]+b[$ffffffff]+c[$ffffffff])<>6 then
    halt(1);
  writeln(a[$ffffffff]);
  writeln(b[$ffffffff]);
  writeln(c[$ffffffff]);
end.
