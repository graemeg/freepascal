{
    This file is part of the Free Pascal Sinclair QL support package.
    Copyright (c) 2021 by Karoly Balogh

    Interface QDOS OS functions for applications

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$MODE FPC}
unit qlutil;

interface

procedure toqlstring(const s: ansistring; qlstring: pointer; qlstrbuflen: longint);
function toqlstring(const s: ansistring): pointer;

implementation


procedure toqlstring(const s: ansistring; qlstring: pointer; qlstrbuflen: longint);
var
  len: longint;
begin
  len:=length(s);
  if len > qlstrbuflen-sizeof(word) then
    len:=qlstrbuflen-sizeof(word);

  if assigned(qlstring) then
    begin
      pword(qlstring)[0]:=len;
      move(s[1],pword(qlstring)[1],len);
    end;
end;

function toqlstring(const s: ansistring): pointer;
var
  qlstring: pointer;
begin
  qlstring:=GetMem(length(s)+sizeof(word));
  if assigned(qlstring) then
    begin
      pword(qlstring)[0]:=length(s);
      move(s[1],pword(qlstring)[1],length(s));
    end;
  toqlstring:=qlstring;
end;

end.
