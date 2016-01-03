unit uw6822a;
{$mode objfpc}{$H+}

interface

implementation

var
  t: text;

initialization
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef wince}
  writeln('Unit 1');
  writeln('initialization');
{$endif wince}
finalization
{$ifndef wince}
  writeln('Unit 1'); // problem
  writeln('finalization'); 
{$endif wince}
=======
  writeln('Unit 1');
  writeln('initialization');
finalization
  writeln('Unit 1'); // problem
  writeln('finalization'); 
>>>>>>> graemeg/fixes_2_2
=======
  writeln('Unit 1');
  writeln('initialization');
finalization
  writeln('Unit 1'); // problem
  writeln('finalization'); 
>>>>>>> origin/fixes_2_2
  assign(t,'uw6822a.txt');
  rewrite(t);
  close(t);
end.
