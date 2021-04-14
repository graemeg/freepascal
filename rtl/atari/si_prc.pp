{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by the Free Pascal development team

    System Entry point for Atari/TOS

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit si_prc;

interface

implementation

{$i gemdos.inc}

var
  procdesc: PPD; public name '__base';
  tpasize: longint;
  stacktop: pointer; public name '__stktop';
  stklen: longint; external name '__stklen';


procedure PascalMain; external name 'PASCALMAIN';


{ this function must be the first in this unit which contains code }
{$OPTIMIZATION OFF}
procedure _FPC_proc_start(pd: PPD); cdecl; public name '_start';
begin
  procdesc:=pd;
  tpasize:=align(sizeof(pd^) + pd^.p_tlen + pd^.p_dlen + pd^.p_blen + stklen, sizeof(pointer));

  if gemdos_mshrink(0, pd, tpasize) < 0 then
    begin
      gemdos_cconws('Not enough memory.'#13#10);
      gemdos_pterm(-39);
    end
  else
    begin
      stacktop:=pd^.p_lowtpa + tpasize;
      asm
        move.l stacktop, sp
      end;
      PascalMain;

      { this should be unreachable... }
      gemdos_pterm(-1);
    end;
end;

procedure _FPC_proc_halt(_ExitCode: longint); cdecl; public name '_haltproc';
begin
  gemdos_pterm(_ExitCode);
end;


end.
