{ Source provided for Free Pascal Bug Report 4529 }
{ Submitted by "Vincent Snijders" on  2005-11-20 }
{ e-mail: vsnijders@quicknet.nl }
program Project1;

<<<<<<< HEAD
<<<<<<< HEAD
{$mode objfpc}{$H+}
=======
{$mode objfpc}{$H+}{$static on}
>>>>>>> graemeg/fixes_2_2
=======
{$mode objfpc}{$H+}{$static on}
>>>>>>> origin/fixes_2_2

uses
  Classes
  { add your units here };

type
  MyClass = class(TComponent)
  //private
  public
    FClassVar: integer; static;
  end;

begin
end.
