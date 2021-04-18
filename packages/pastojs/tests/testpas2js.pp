{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2014 by Michael Van Canneyt

    Unit tests runner for Pascal-to-Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program testpas2js;

{$mode objfpc}{$H+}

uses
  {$IFDEF EnableMemCheck}
  MemCheck,
  {$ENDIF}
  Classes, consoletestrunner, tcconverter, TCModules, TCSrcMap,
  TCFiler, TCUnitSearch, TCOptimizations, TCGenerics, TCPrecompile;

type

  TMyTestRunner = class(TTestRunner)
  protected
  end;

var
  Application: TMyTestRunner;

begin
  DefaultFormat:=fPlain;
  DefaultRunAllTests:=True;
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Run;
  Application.Free;
end.
