{
    This file is part of the Free Component Library

    JSON fpcunit tester program
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
program testjson;

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, testjsondata, testjsonparser, consoletestrunner; //, testjsonrtti, fpjsonrtti;
=======
  Classes, consoletestrunner, testjsondata, testjsonparser,
  fpcunitconsolerunner, testjsonrtti, fpjsonrtti;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  Classes, consoletestrunner, testjsondata, testjsonparser,
  fpcunitconsolerunner;
>>>>>>> graemeg/fixes_2_2
=======
  Classes, consoletestrunner, testjsondata, testjsonparser,
  fpcunitconsolerunner;
>>>>>>> origin/fixes_2_2
type
  { TLazTestRunner }
   TMyTestRunner = class(TTestRunner)
   protected
     // override the protected methods of TTestRunner to customize its behavior
   end;
      
var
  Application: TMyTestRunner;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  DefaultFormat := fPlain;
  DefaultRunAllTests := True;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Application := TMyTestRunner.Create(nil); 
  Application.Initialize;
  Application.Run;  
  Application.Free;
end.
