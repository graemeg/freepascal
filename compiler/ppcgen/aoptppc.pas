{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit implements the generic PowerPC optimizer object

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
Unit aoptppc;

Interface

{$i fpcdefs.inc}

uses cpubase, cgbase, aopt, aasmtai;

Type
  TPPCAsmOptimizer = class(TAsmOptimizer)
    function RegLoadedWithNewValue(reg : tregister; hp : tai) : boolean; override;
 End;

Implementation

  uses
    cutils, verbose, cgcpu, cgobj, aasmcpu;

  function TPPCAsmOptimizer.RegLoadedWithNewValue(reg: tregister; hp: tai): boolean;
    var
      p: taicpu;
    begin
      Result := false;
      if not(assigned(hp) and (hp.typ = ait_instruction)) then
        exit;

      p := taicpu(hp);
      if not(p.ops > 0) then
        exit;

      case p.opcode of
        A_CMP,
        A_CMPI,
        A_CMPL,
        A_CMPLI:
          begin
            result:=reg=NR_CR;
            exit;
          end;
        A_STB,
        { the register forming the address is modified so no new value is loaded }
        A_STBU,
        A_STBUX,
        A_STBX,
        A_STH,
        A_STHBRX,
        A_STHU,
        A_STHUX,
        A_STHX,
        A_STMW:
          exit;
        else
          ;
      end;
      case p.oper[0]^.typ of
        top_reg:
          Result := (p.oper[0]^.reg = reg) ;
        else
          ;
      end;
    end;

End.
