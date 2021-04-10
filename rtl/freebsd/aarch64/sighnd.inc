{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    Signal handler is arch dependant due to processor to language
    exception conversion.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ procedure SignalToRunerror(Sig: longint; SigInfo: PSigInfo; UContext: PUContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl; }
procedure SignalToRunerror(Sig: cint; info : psiginfo;  SigContext:PSigContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl;

var
  res : word;
begin
  res:=0;
  case sig of
    SIGFPE:
        res:=207;
    SIGILL:
        res:=216;
    SIGSEGV :
        res:=216;
    SIGBUS:
        res:=214;
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
  end;
  reenable_signal(sig);
  { give runtime error at the position where the signal was raised }
  if res<>0 then
    HandleError(res);
end;
