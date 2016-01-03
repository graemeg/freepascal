{
    Copyright (c) 1998-2006 by Florian Klaempfl

    Basic Processor information

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
Unit cpuinfo;

{$i fpcdefs.inc}

Interface

uses
  globtype;

Type
   bestreal = extended;
   ts32real = single;
   ts64real = double;
   ts80real = extended;
   ts128real = type extended;
   ts64comp = type extended;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_itanium
      );

   tfputype =
     (fpu_none,
      fpu_itanium
     );
     
const
   { calling conventions supported by the code generator }
   supported_calling_conventions : tproccalloptions = [
     pocall_internproc,
     pocall_stdcall,
     pocall_cdecl,
     pocall_cppdecl
   ];


   cputypestr : array[tcputype] of string[10] = ('',
     'ITANIUM'
   );

   fputypestr : array[tfputype] of string[6] = ('',
     'ITANIUM'
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = [cs_opt_peephole,cs_opt_regvar,cs_opt_stackframe,
								  cs_opt_asmcse,cs_opt_loopunroll,cs_opt_uncertain,
								  cs_opt_nodecse];

   level1optimizerswitches = [cs_opt_level1,cs_opt_peephole];
   level2optimizerswitches = level1optimizerswitches + 
     [cs_opt_level2,cs_opt_regvar,cs_opt_stackframe,cs_opt_asmcse,cs_opt_nodecse];
   level3optimizerswitches = level2optimizerswitches + [cs_opt_level3{,cs_opt_loopunroll}];

Implementation

end.

