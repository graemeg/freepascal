{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2021 by Free Pascal development team

    This file implements the startup code for WebAssembly programs that
    don't link to the C library.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


unit si_prc;

interface

procedure _start;

implementation

procedure PASCALMAIN; external 'PASCALMAIN';
//procedure main; external 'main';

procedure _start;
begin
  PASCALMAIN;
//  main;
end;

exports
  _start;

end.
