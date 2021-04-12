{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Free Pascal development team

    Low level directory functions for the Sinclair QL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                           Directory Handling
*****************************************************************************}
procedure do_mkdir(const s : rawbytestring);
var
  chanId: tchanid;
  errCode: longint;
begin
  { The QL needs a open file handle on which to create a
    directory. The file should be opened as a new file as
    errors occur if the file exists when creating a new
    directory when it's already there. }
  chanId := io_open(PChar(s), Q_OPEN_OVER);
  if chanId < 0 then
    begin
      Error2InOutRes(chanId); 
      exit;
    end;

  { Convert the opened file to a directory. }
  errCode := iof_mkdr(chanId);

  { Close the file/directory. No errors occur. }
  io_close(chanId);

  { Check if the mkdir actually worked. }
  if errCode < 0 then
    Error2InOutRes(errCode);
end;


procedure do_rmdir(const s : rawbytestring);
begin
  { Deleting a directory is as simple as deleting
    a file. There must be no files in the directory
    though. However, SMSQ seems to return zero for a file
    or directory name that is not present. It should return
    ERR_NF. (At least on RAM_/FLP_ or WIN_)
  }
  Error2InOutRes(io_delet(pchar(s)));
end;


procedure do_ChDir(const s: rawbytestring);
begin
end;


procedure do_GetDir (DriveNr: byte; var Dir: RawByteString);
begin
end;
