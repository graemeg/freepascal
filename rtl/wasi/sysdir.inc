{
    This file is part of the Free Pascal run time library.

    Main OS dependant body of the system unit, loosely modelled
    after POSIX.  *BSD version (Linux version is near identical)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                           Directory Handling
*****************************************************************************}

procedure Do_MkDir(s: rawbytestring);
begin
  DebugWriteLn('Do_MkDir');
end;

procedure Do_RmDir(s: rawbytestring);
begin
  DebugWriteLn('Do_RmDir');
end;

procedure do_ChDir(s: rawbytestring);
begin
  DebugWriteLn('do_ChDir');
end;

procedure do_getdir(drivenr : byte;var dir : rawbytestring);
begin
  DebugWriteLn('do_getdir');
end;
