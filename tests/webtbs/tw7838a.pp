{ %norun }
<<<<<<< HEAD
<<<<<<< HEAD
{ %needlibrary }
{ %target=win32,win64,wince,linux,android}
=======
{ %target=win32,win64,wince,linux}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,linux}
>>>>>>> origin/fixes_2_2

library tw7838a;

{$mode objfpc} {$h+}

uses uw7838a;

function dllf: longint;
begin
  result:=exetest;
  if (result<>aa) then
    halt(1);
end;

exports dllf;

begin

end.
