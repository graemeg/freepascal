{ %opt=-gh}

uses sysutils;

var ps : pstring;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  HaltOnNotReleased := true;
=======
  HaltOnNotReleased := true;  
>>>>>>> graemeg/fixes_2_2
=======
  HaltOnNotReleased := true;  
>>>>>>> origin/fixes_2_2
  ps:=newstr('TEST');
  writeln(ps^);
  disposestr(ps);
end.
