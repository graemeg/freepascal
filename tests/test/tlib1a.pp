<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %skiptarget=go32v2,os2 }
{ %opt=-gl -O- win32,win64%-WN }
{ %delopt=-XX }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ %skiptarget=go32v2 }
{ %opt=-gl -O- }
>>>>>>> graemeg/cpstrnew
{ %norun }
library tlib1a;
{$goto on}
  procedure p(var a : pointer);
    label
      we;
    begin
    we:
      a:=@we;
    end;

  exports p;

begin
end.
