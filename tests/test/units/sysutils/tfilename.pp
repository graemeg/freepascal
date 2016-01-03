program filefunctest;

{$IFDEF FPC}
  {$mode objfpc}{$H+}
{$ENDIF}

uses
  SysUtils;

var
  err : boolean;
procedure TestFuncs(testidx:integer;const res,expected: string);
begin
  if res<>expected then
    begin
      Writeln('FAILURE at ',testidx:5,' got "',res,'", expected "',expected,'"  ');
      err:=true;
    end;
end;

begin
  // Default Unix
  AllowDirectorySeparators:=['/','\'];
  AllowDriveSeparators:=[];
  TestFuncs(1,ExtractFilePath('./:'),'./');
  TestFuncs(2,ExtractFileName('./:'),':');
  TestFuncs(3,ExtractFileDrive('./:'),'');

  TestFuncs(4,ExtractFilePath('C:/blah:blah'),'C:/');
  TestFuncs(5,ExtractFileName('C:/blah:blah'),'blah:blah');
  TestFuncs(6,ExtractFileDrive('C:/blah:blah'),'');

  TestFuncs(7,ExtractFilePath('./\'),'./\');
  TestFuncs(8,ExtractFileName('./\'),'');
  TestFuncs(9,ExtractFileDrive('./\'),'');

  TestFuncs(10,ExtractFilePath('./c:'),'./');
  TestFuncs(11,ExtractFileName('./c:'),'c:');
  TestFuncs(12,ExtractFileDrive('./c:'),'');

  TestFuncs(13,ExtractFilePath('\\server\share\file'),'\\server\share\');
  TestFuncs(14,ExtractFileName('\\server\share\file'),'file');
<<<<<<< HEAD
<<<<<<< HEAD
  TestFuncs(15,ExtractFileDrive('\\server\share\file'),'\\server\share');
=======
  TestFuncs(15,ExtractFileDrive('\\server\share\file'),'\\server');
>>>>>>> graemeg/fixes_2_2
=======
  TestFuncs(15,ExtractFileDrive('\\server\share\file'),'\\server');
>>>>>>> origin/fixes_2_2

  // Kylix compatibility mode
  AllowDirectorySeparators:=['/'];
  AllowDriveSeparators:=[];
  TestFuncs(101,ExtractFilePath('./:'),'./');
  TestFuncs(102,ExtractFileName('./:'),':');
  TestFuncs(103,ExtractFileDrive('./:'),'');

  TestFuncs(104,ExtractFilePath('C:/blah:blah'),'C:/');
  TestFuncs(105,ExtractFileName('C:/blah:blah'),'blah:blah');
  TestFuncs(106,ExtractFileDrive('C:/blah:blah'),'');

  TestFuncs(107,ExtractFilePath('./\'),'./');
  TestFuncs(108,ExtractFileName('./\'),'\');
  TestFuncs(109,ExtractFileDrive('./\'),'');

  TestFuncs(110,ExtractFilePath('./c:'),'./');
  TestFuncs(111,ExtractFileName('./c:'),'c:');
  TestFuncs(112,ExtractFileDrive('./c:'),'');

  TestFuncs(113,ExtractFilePath('\\server\share\file'),'');
  TestFuncs(114,ExtractFileName('\\server\share\file'),'\\server\share\file');
  TestFuncs(115,ExtractFileDrive('\\server\share\file'),'');

  // Default Windows/DOS/SO2
  AllowDirectorySeparators:=['/','\'];
  AllowDriveSeparators:=[':'];
  TestFuncs(201,ExtractFilePath('./:'),'./:');
  TestFuncs(202,ExtractFileName('./:'),'');
  TestFuncs(203,ExtractFileDrive('./:'),'');

  TestFuncs(204,ExtractFilePath('C:/blah:blah'),'C:/blah:');
  TestFuncs(205,ExtractFileName('C:/blah:blah'),'blah');
  TestFuncs(206,ExtractFileDrive('C:/blah:blah'),'C:');

  TestFuncs(207,ExtractFilePath('./\'),'./\');
  TestFuncs(208,ExtractFileName('./\'),'');
  TestFuncs(209,ExtractFileDrive('./\'),'');

  TestFuncs(210,ExtractFilePath('./c:'),'./c:');
  TestFuncs(211,ExtractFileName('./c:'),'');
  TestFuncs(212,ExtractFileDrive('./c:'),'');

  TestFuncs(213,ExtractFilePath('\\server\share\file'),'\\server\share\');
  TestFuncs(214,ExtractFileName('\\server\share\file'),'file');
<<<<<<< HEAD
<<<<<<< HEAD
  TestFuncs(215,ExtractFileDrive('\\server\share\file'),'\\server\share');
=======
  TestFuncs(215,ExtractFileDrive('\\server\share\file'),'\\server');
>>>>>>> graemeg/fixes_2_2
=======
  TestFuncs(215,ExtractFileDrive('\\server\share\file'),'\\server');
>>>>>>> origin/fixes_2_2

  // Windows/DOS/SO2 Delphi Compatibility
  AllowDirectorySeparators:=['\'];
  AllowDriveSeparators:=[':'];
  TestFuncs(301,ExtractFilePath('./:'),'./:');
  TestFuncs(302,ExtractFileName('./:'),'');
  TestFuncs(303,ExtractFileDrive('./:'),'');

  TestFuncs(304,ExtractFilePath('C:/blah:blah'),'C:/blah:');
  TestFuncs(305,ExtractFileName('C:/blah:blah'),'blah');
  TestFuncs(306,ExtractFileDrive('C:/blah:blah'),'C:');

  TestFuncs(307,ExtractFilePath('./\'),'./\');
  TestFuncs(308,ExtractFileName('./\'),'');
  TestFuncs(309,ExtractFileDrive('./\'),'');

  TestFuncs(310,ExtractFilePath('./c:'),'./c:');
  TestFuncs(311,ExtractFileName('./c:'),'');
  TestFuncs(312,ExtractFileDrive('./c:'),'');

  TestFuncs(313,ExtractFilePath('\\server\share\file'),'\\server\share\');
  TestFuncs(314,ExtractFileName('\\server\share\file'),'file');
<<<<<<< HEAD
<<<<<<< HEAD
  TestFuncs(315,ExtractFileDrive('\\server\share\file'),'\\server\share');
=======
  TestFuncs(315,ExtractFileDrive('\\server\share\file'),'\\server');
>>>>>>> graemeg/fixes_2_2
=======
  TestFuncs(315,ExtractFileDrive('\\server\share\file'),'\\server');
>>>>>>> origin/fixes_2_2

  if err then
    halt(1);
end.

