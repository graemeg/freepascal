{
    Copyright (c) 2000 by Florian Klaempfl

    Includes the x86-64 code generator

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
{ This is a helper unit to include the necessary code generator units
  for the x86-64 processor.
}
unit cpunode;

{$i fpcdefs.inc}

  interface

  implementation

    uses
       { generic nodes }
       ncgbas,
       ncgld,
       ncgflw,
       ncgcnv,
       ncgmem,
       ncgmat,
       ncgcon,
       ncgcal,
       ncgset,
       ncgopt,
       ncgobjc,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       { symtable }
       symcpu,
{$ifndef llvm}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       // n386con,n386flw,n386mat,n386mem,
       // n386set,n386inl,n386opt,
>>>>>>> graemeg/cpstrnew
       { the cpu specific node units must be used after the generic ones to
         get the correct class pointer }
       nx86set,
       nx86con,
       nx86mem,
       nx64add,
       nx64cal,
       nx64cnv,
       nx64mat,
{$ifndef DISABLE_WIN64_SEH}
       nx64flw,
{$endif DISABLE_WIN64_SEH}
       nx64inl,
       nx64set
{$else}
       llvmnode
{$endif ndef llvm}
       ;

end.
