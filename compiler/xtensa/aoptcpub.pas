 {
    Copyright (c) 1998-2002 by Jonas Maebe, member of the Free Pascal
    Development Team

    This unit contains several types and constants necessary for the
    optimizer to work on the ARM64 architecture

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
Unit aoptcpub; { Assembler OPTimizer CPU specific Base }

{$i fpcdefs.inc}

{ enable the following define if memory references can have both a base and }
{ index register in 1 operand                                               }

{ enable the following define if memory references can have a scaled index }
{ define RefsHaveScale}

{ enable the following define if memory references can have a segment }
{ override                                                            }

{ define RefsHaveSegment}

Interface

Uses
  cgbase,aasmtai,
  cpubase,aasmcpu,AOptBase;

Type

{ type of a normal instruction }
  TInstr = Taicpu;
  PInstr = ^TInstr;

{ ************************************************************************* }
{ **************************** TCondRegs ********************************** }
{ ************************************************************************* }
{ Info about the conditional registers                                      }
  TCondRegs = Object
    Constructor Init;
    Destructor Done;
  End;

{ ************************************************************************* }
{ **************************** TAoptBaseCpu ******************************* }
{ ************************************************************************* }

  TAoptBaseCpu = class(TAoptBase)
    function RegModifiedByInstruction(Reg: TRegister; p1: tai): boolean; override;
  End;


{ ************************************************************************* }
{ ******************************* Constants ******************************* }
{ ************************************************************************* }
Const

{ the maximum number of things (registers, memory, ...) a single instruction }
{ changes                                                                    }

  MaxCh = 3;

{Oper index of operand that contains the source (reference) with a load }
{instruction                                                            }

  LoadSrc = 0;

{Oper index of operand that contains the destination (register) with a load }
{instruction                                                                }

  LoadDst = 1;

{Oper index of operand that contains the source (register) with a store }
{instruction                                                            }

  StoreSrc = 0;

{Oper index of operand that contains the destination (reference) with a load }
{instruction                                                                 }

  StoreDst = 1;

  aopt_uncondjmp = A_J;
  aopt_condjmp = A_Bcc;

Implementation

{ ************************************************************************* }
{ **************************** TCondRegs ********************************** }
{ ************************************************************************* }
  Constructor TCondRegs.init;
    Begin
    End;


  Destructor TCondRegs.Done; {$ifdef inl} inline; {$endif inl}
    Begin
    End;


  function TAoptBaseCpu.RegModifiedByInstruction(reg: tregister; p1: tai): boolean;
    var
      i: longint;
      preg: tregister;
    begin
      result:=false;
      for i:=0 to taicpu(p1).ops-1 do
        case taicpu(p1).oper[i]^.typ of
          top_reg:
            begin
              preg:=taicpu(p1).oper[i]^.reg;
              if (getregtype(preg)=getregtype(reg)) and
                 (getsupreg(preg)=getsupreg(reg)) and
                 (taicpu(p1).spilling_get_operation_type(i) in [operand_write,operand_readwrite]) then
                begin
                  result:=true;
                  exit;
                end;
            end;
          //top_ref:
          //  begin
          //    if (taicpu(p1).oper[i]^.ref^.addressmode<>am_offset) and
          //       (reg=taicpu(p1).oper[i]^.ref^.base) then
          //      begin
          //        result:=true;
          //        exit
          //      end;
          //  end;
          else
            ;
        end;
    end;

End.
