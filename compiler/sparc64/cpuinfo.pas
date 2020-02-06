{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Basic Processor information for the SPARC64

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
unit cpuinfo;

{$i fpcdefs.inc}

interface

uses
  globtype;

type
  bestreal = double;
{$if FPC_FULLVERSION>20700}
  bestrealrec = TDoubleRec;
{$endif FPC_FULLVERSION>20700}
  ts32real = single;
  ts64real = double;
  ts80real = extended;
  ts128real = type extended;
  ts64comp = type extended;
  pbestreal=^bestreal;

  { possible supported processors for this target }
  tcputype=(cpu_none,
    cpu_SPARC_V9
  );

  tfputype =(fpu_none,
    fpu_soft,
    fpu_hard
  );

  tcontrollertype =(ct_none
  );

   tcontrollerdatatype = record
      controllertypestr, controllerunitstr: string[20];
      cputype: tcputype; fputype: tfputype;
      flashbase, flashsize, srambase, sramsize, eeprombase, eepromsize, bootbase, bootsize: dword;
   end;


Const
  { Is there support for dealing with multiple microcontrollers available }
  { for this platform? }
  ControllerSupport = false;

  { We know that there are fields after sramsize
    but we don't care about this warning }
  {$PUSH}
   {$WARN 3177 OFF}
  embedded_controllers : array [tcontrollertype] of tcontrollerdatatype =
  (
      (controllertypestr:''; controllerunitstr:''; cputype:cpu_none; fputype:fpu_none; flashbase:0; flashsize:0; srambase:0; sramsize:0));
  {$POP}

  { calling conventions supported by the code generator }
  supported_calling_conventions : tproccalloptions = [
    pocall_internproc,
    pocall_stdcall,
    pocall_safecall,
    pocall_cdecl,
    pocall_cppdecl
  ];

   cputypestr : array[tcputype] of string[10] = ('',
     'SPARCV9'
   );

   fputypestr : array[tfputype] of string[6] = (
     'NONE',
     'SOFT',
     'HARD'
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,
                                  cs_opt_tailrecursion,cs_opt_nodecse,
                                  cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches + 
     [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_tailrecursion,cs_opt_nodecse];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

implementation

end.
