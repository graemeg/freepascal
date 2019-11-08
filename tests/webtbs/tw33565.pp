{ %OPT=-O- -OoREGVAR -Oolevel1 -Cr -Mobjfpc -Oopeephole }
program app_test_core;

uses

  SysUtils, Classes;

  procedure SetMemory(Stream: TStream; var P: Pointer; var PSize: Integer);
  begin
    PSize := Stream.Size;
    GetMem(P, PSize);
    Stream.Position := 0;
    Stream.Read(P^, PSize);
  end;

var
  M: TMemoryStream;
  L, V: Integer;
  P: Pointer;
begin
  M := TMemoryStream.Create;
  V := -1;
  M.Write(V, SizeOf(V));
  M.Position := 0;

  P := nil;
  L := 0;
  SetMemory(M, P, L);
  FreeMem(P);

  M.Free;
end.
