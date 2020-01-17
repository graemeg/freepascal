{
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit implements the Risc-V32 optimizer object

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


Unit aoptcpu;

Interface

{$i fpcdefs.inc}

uses
  cpubase,
  aoptobj, aoptcpub, aopt, aoptcpurv,
  aasmtai,aasmdata, aasmcpu;

Type
  TCpuAsmOptimizer = class(TRVCpuAsmOptimizer)
    { uses the same constructor as TAopObj }
    function PeepHoleOptPass1Cpu(var p: tai): boolean; override;

    function PostPeepHoleOptsCpu(var p: tai): boolean; override;
  End;

Implementation

  uses
    cutils, verbose, cgbase, cgcpu, cgobj;


  function TCpuAsmOptimizer.PeepHoleOptPass1Cpu(var p: tai): boolean;
    var
      next1, next2: tai;
      l1, l2, shlcount: longint;
    begin
      result := inherited PeepHoleOptPass1Cpu(p);
      if result then
        exit;

      case p.typ of
        ait_instruction:
          begin
            {case taicpu(p).opcode of
            end;}
          end;
        else
          ;
      end;
    end;


  function TCpuAsmOptimizer.PostPeepHoleOptsCpu(var p: tai): boolean;
    var
      next1: tai;
    begin
      result := inherited PostPeepHoleOptsCpu(p);
      if result then
        exit;

      case p.typ of
        ait_instruction:
          begin
          end;
        else
          ;
      end;
    end;

begin
  casmoptimizer:=TCpuAsmOptimizer;
End.
