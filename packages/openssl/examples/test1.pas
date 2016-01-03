program test1;

{$mode objfpc}{$H+}

uses
  OpenSSL;

begin
  if InitSSLInterface then
    Writeln('Success')
  else
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Load failed, missing functions: ',OpenSSL_unavailable_functions);
=======
    Writeln('Holy shit!');
>>>>>>> graemeg/fixes_2_2
=======
    Writeln('Holy shit!');
>>>>>>> origin/fixes_2_2
end.

