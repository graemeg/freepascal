{
    Copyright (c) 1998-2009 by Mazen NEIFER and David Zhang

    This unit contains the MIPS GAS instruction tables

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
unit itcpugas;

{$i fpcdefs.inc}

interface

uses
  cpubase, cgbase;

const
  gas_op2str: array[tasmop] of string[15] = ({$INCLUDE strinst.inc});

function gas_regnum_search(const s: string): Tregister;
function gas_regname(r: Tregister): string;


implementation

uses
  cutils, verbose;

const
  gas_regname_table: array[tregisterindex] of string[7] = (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        {$i rmipsgas.inc}
=======
        {$i rmipsstd.inc}
>>>>>>> graemeg/cpstrnew
=======
        {$i rmipsstd.inc}
>>>>>>> graemeg/cpstrnew
=======
        {$i rmipsstd.inc}
>>>>>>> graemeg/cpstrnew
=======
        {$i rmipsstd.inc}
>>>>>>> origin/cpstrnew
    );

  gas_regname_index: array[tregisterindex] of tregisterindex = (
        {$i rmipssri.inc}
    );


function findreg_by_gasname(const s: string): tregisterindex;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  p: tregisterindex;
=======
  i, p: tregisterindex;
>>>>>>> graemeg/cpstrnew
=======
  i, p: tregisterindex;
>>>>>>> graemeg/cpstrnew
=======
  i, p: tregisterindex;
>>>>>>> graemeg/cpstrnew
=======
  i, p: tregisterindex;
>>>>>>> origin/cpstrnew
begin
  for p := low(tregisterindex) to high(tregisterindex) do
  begin
    if gas_regname_table[gas_regname_index[p]] = s then
    begin
      findreg_by_gasname := gas_regname_index[p];
      exit
    end;
  end;
  findreg_by_gasname := 0;
end;


function gas_regnum_search(const s: string): Tregister;
begin
  Result := regnumber_table[findreg_by_gasname(s)];
end;


function gas_regname(r: Tregister): string;
var
  hr: tregister;
  p:  longint;
begin
  { Double uses the same table as single }
  hr := r;
  case getsubreg(hr) of
    R_SUBFD:
      setsubreg(hr, R_SUBFS);
    R_SUBL, R_SUBW, R_SUBD, R_SUBQ:
      setsubreg(hr, R_SUBD);
  end;
  p := findreg_by_number(hr);
  if p <> 0 then
    Result := gas_regname_table[p]
  else
    Result := generic_regname(r);
end;

end.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{
  $Log: itcpugas.pas,v $
  Revision 1.7  2005/02/14 17:13:10  peter
    * truncate log

}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
