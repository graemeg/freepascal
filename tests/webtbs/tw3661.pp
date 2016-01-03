{ %cpu=i386,powerpc}
{ %target=linux,win32,go32v2,os2,beos,haiku,morphos }
{ %OPT=-glhc }
{ Source provided for Free Pascal Bug Report 3661 }
{ Submitted by "Martin Schreiber" on  2005-02-16 }
{ e-mail:  }
program project1;

{$mode objfpc}{$H+}
uses
  Classes;


var
  f: text;
begin
 HaltOnNotReleased := true;
<<<<<<< HEAD
<<<<<<< HEAD
 { The subject matter of the test is heaptrc checking pointers belonging to .data
   or .bss sections. However, using a standard io file for this purpose is not a good
   idea, because it is a threadvar and resides in .bss only in certain circumstances
   (FPC-specific threadvar model and multithreading not initialized).
   Using a 'normal' file is fine. }
 //writeln('abc');
 
 assign(f,'tw3661.txt');
 rewrite(f);
 writeln(f,'abc');
 close(f);
 erase(f);
=======
=======
>>>>>>> origin/fixes_2_2
 writeln('abc');
>>>>>>> graemeg/fixes_2_2
end.
