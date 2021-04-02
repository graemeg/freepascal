{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for AArch64

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Unit CPUInfo;

{$i fpcdefs.inc}

Interface

  uses
    globtype;

Type
   bestreal = double;
{$if FPC_FULLVERSION>20700}
   bestrealrec = TDoubleRec;
{$endif FPC_FULLVERSION>20700}
   ts32real = single;
   ts64real = double;
   ts80real = type extended;
   ts128real = type extended;
   ts64comp = comp;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_armv8,
       cpu_armv8a,
       cpu_armv81a,
       cpu_armv82a,
       cpu_armv83a,
       cpu_armv84a,
       cpu_armv85a,
       cpu_armv86a
      );

Type
   tfputype =
     (fpu_none,
      fpu_vfp
     );

   tcontrollertype =
     (ct_none
     );

   tcontrollerdatatype = record
      controllertypestr, controllerunitstr: string[20];
      cputype: tcputype; fputype: tfputype;
      flashbase, flashsize, srambase, sramsize, eeprombase, eepromsize, bootbase, bootsize: dword;
   end;


Const
   fputypestrllvm : array[tfputype] of string[6] = ('',
     ''
   );

   { Is there support for dealing with multiple microcontrollers available }
   { for this platform? }
   ControllerSupport = false; (* Not yet at least ;-) *)
   {# Size of native extended floating point type }
   extended_size = 8;
   { target cpu string (used by compiler options) }
   target_cpu_string = 'aarch64';

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
     pocall_safecall,
     pocall_stdcall,
     { same as stdcall only different name mangling }
     pocall_cdecl,
     { same as stdcall only different name mangling }
     pocall_cppdecl,
     { same as stdcall but floating point numbers are handled like equal sized integers }
     pocall_softfloat,
     { same as stdcall (requires that all const records are passed by
       reference, but that's already done for stdcall) }
     pocall_mwpascal,
     { used for interrupt handling }
     pocall_interrupt
   ];

   cputypestr : array[tcputype] of string[9] = ('',
     'ARMV8',
     'ARMV8-A',
     'ARMV8.1-A',
     'ARMV8.2-A',
     'ARMV8.3-A',
     'ARMV8.4-A',
     'ARMV8.5-A',
     'ARMV8.6-A'
   );

   fputypestr : array[tfputype] of string[9] = ('',
     'VFP'
   );


   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,cs_opt_tailrecursion,
				  cs_opt_nodecse,cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches +
     [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_tailrecursion,cs_opt_nodecse];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

type
   tcpuflags =
     (CPUAARCH64_HAS_LSE       { CPU supports Large System Extensions }
     );

   tfpuflags =
     (CPUAARCH64_HAS_VFP       { CPU supports VFP }
     );

const
   cpu_capabilities : array[tcputype] of set of tcpuflags =
     ( { cpu_none      } [],
       { cpu_armv8     } [],
       { cpu_armv8a    } [],
       { cpu_armv81a   } [CPUAARCH64_HAS_LSE],
       { cpu_armv82a   } [CPUAARCH64_HAS_LSE],
       { cpu_armv83a   } [CPUAARCH64_HAS_LSE],
       { cpu_armv84a   } [CPUAARCH64_HAS_LSE],
       { cpu_armv85a   } [CPUAARCH64_HAS_LSE],
       { cpu_armv86a   } [CPUAARCH64_HAS_LSE]
     );

   fpu_capabilities : array[tfputype] of set of tfpuflags =
     ( { fpu_none         } [],
       { fpu_vfp          } [CPUAARCH64_HAS_VFP]
     );

Implementation

end.

