{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for the Xtensa

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
    globtype,
    systems;

Type
   bestreal = double;
   bestrealrec = TDoubleRec;
   ts32real = single;
   ts64real = double;
   ts80real = type extended;
   ts128real = type extended;
   ts64comp = comp;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_lx106,
       cpu_lx6
      );


Type
   tfputype =
     (fpu_none,
      fpu_soft,
      fpu_libgcc,
      fpu_hard
     );

Type
   tcontrollertype =
     (ct_none,
      ct_esp8266,
      ct_esp32,
      ct_esp32_d0wd,
      ct_esp32_d2wd,
      ct_esp32_sOwd
     );

   tcontrollerdatatype = record
      controllertypestr, controllerunitstr: string[20];
      cputype: tcputype; fputype: tfputype; abi: tabi;
      flashbase, flashsize, srambase, sramsize, eeprombase, eepromsize, bootbase, bootsize: dword;
   end;

Const
   { Is there support for dealing with multiple microcontrollers available }
   { for this platform? }
   ControllerSupport = true;
   {# Size of native extended floating point type }
   extended_size = 12;
   { target cpu string (used by compiler options) }
   target_cpu_string = 'xtensa';

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
     { used for interrupt handling }
     pocall_interrupt
   ];

   cputypestr : array[tcputype] of string[8] = (
     '',
     'LX106',
     'LX6'
   );

   fputypestr : array[tfputype] of string[10] = (
     'NONE',
     'SOFT',
     'LIBGCC',
     'HARD'
   );


    { We know that there are fields after sramsize
      but we don't care about this warning }
    {$WARN 3177 OFF}

   embedded_controllers : array [tcontrollertype] of tcontrollerdatatype =
   (
      (controllertypestr:'';		controllerunitstr:'';	cputype:cpu_none; fputype:fpu_none; abi: abi_default; flashbase:0),
      (controllertypestr:'ESP8266';	controllerunitstr:'ESP8266';	cputype:cpu_lx106; fputype:fpu_none; abi: abi_xtensa_call0; { flashbase:$40000000; flashsize:448*1024; srambase:$40070000; sramsize: 520*1024 }),
      (controllertypestr:'ESP32';	controllerunitstr:'ESP32';	cputype:cpu_lx6; fputype:fpu_hard; abi: abi_xtensa_windowed; flashbase:$40000000; flashsize:2*1024*1024),
      (controllertypestr:'ESP32_D0WD';	controllerunitstr:'ESP32_D0WD';	cputype:cpu_lx6; fputype:fpu_hard; abi: abi_xtensa_windowed; flashbase:$40000000; flashsize:448*1024; srambase:$40070000; sramsize: 520*1024),
      (controllertypestr:'ESP32_D2WD';	controllerunitstr:'ESP32_D2WD';	cputype:cpu_lx6; fputype:fpu_hard; abi: abi_xtensa_windowed; flashbase:$40000000; flashsize:448*1024; srambase:$40070000; sramsize: 520*1024),
      (controllertypestr:'ESP32_S0WD';	controllerunitstr:'ESP32_S0WD';	cputype:cpu_lx6; fputype:fpu_hard; abi: abi_xtensa_windowed; flashbase:$40000000; flashsize:448*1024; srambase:$40070000; sramsize: 520*1024)
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,cs_opt_tailrecursion,
                                  cs_opt_stackframe,cs_opt_nodecse,cs_opt_reorder_fields,cs_opt_fastmath,cs_opt_forcenostackframe];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches +
     [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_tailrecursion,cs_opt_nodecse];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{cs_opt_scheduler,}cs_opt_loopunroll];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

 type
   tcpuflags =
      (
        CPUXTENSA_REGWINDOW,
        CPUXTENSA_HAS_SEXT
      );

   tfpuflags =
      (
        FPUXTENSA_SINGLE, { FPU has single support }
        FPUXTENSA_DOUBLE  { FPU has double support, this is a dummy so far for easier checking what code to generate }
      );

 const
   cpu_capabilities : array[tcputype] of set of tcpuflags =
     (
       { cpu_none     } [],
       { cpu_lx106    } [],
       { cpu_lx6      } [CPUXTENSA_REGWINDOW, CPUXTENSA_HAS_SEXT]
     );

   fpu_capabilities : array[tfputype] of set of tfpuflags =
     (
       { fpu_none       } [],
       { fpu_soft       } [],
       { fpu_libgcc     } [],
       { fpu_hard       } [FPUXTENSA_SINGLE]
     );

Implementation

end.

