{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Includes the 680x0/Coldfire code generator

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

{$i fpcdefs.inc}

  interface

  implementation

    uses
       { generic nodes }
       ncgbas,ncgld,ncgflw,ncgcnv,ncgmem,ncgcon,ncgcal,ncgset,ncginl,ncgmat,ncgadd,
       { symtable }
       symcpu,
       aasmdef,
       { to be able to only parts of the generic code,
         the processor specific nodes must be included
         after the generic one (FK)
       }
       n68kadd,
       n68kcal,
       n68kmem,
       n68kset,
       n68kinl,
       n68kmat,
       n68kcnv,
       n68kutil
       ;

end.
