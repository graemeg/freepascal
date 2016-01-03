{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Includes the PowerPC64 code generator

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
unit cpunode;

{$I fpcdefs.inc}

interface

implementation

uses
  { generic nodes }
  ncgbas, ncgld, ncgflw, ncgcnv, ncgmem, ncgcon, ncgcal, ncgset, ncginl, ncgopt,
  ncgobjc,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { symtable }
  symcpu,
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { to be able to only parts of the generic code,
    the processor specific nodes must be included
    after the generic one (FK)
  }
{$ifndef llvm}
  nppcadd,
  nppccal,
  //       nppccon,
  //       nppcflw,
  //       nppcmem,
  ngppcset,
  ngppcinl,
  //       nppcopt,
  nppcmat,
  nppccnv,
  nppcld
{$else not llvm}
  llvmnode
{$endif not llvm}
  ;

end.

