{$MODE OBJFPC}
{$APPTYPE CONSOLE}

uses Classes, BufStream, Sysutils;

procedure TestBufferedFileStream;
var
  F: TStream;
  pf: File;
begin
  Assign(pf,'tw38351.tmp');
  Rewrite(pf,1);
  Seek(pf,100);
  Close(pf);
  F := TBufferedFileStream.Create('tw38351.tmp', fmOpenRead);
  try
    Writeln(F.Position);
    if F.Position<>0 then
      halt(1);
    Writeln(F.Seek(0, soBeginning)); // TFileStream = 0, TBufferedFileStream = -1
    Writeln(F.Position);
    if F.Position<>0 then
      halt(1);
  finally
    F.Free;
    DeleteFile('tw38351.tmp');
  end;
end;

begin
  TestBufferedFileStream;
  writeln('ok');
end.
