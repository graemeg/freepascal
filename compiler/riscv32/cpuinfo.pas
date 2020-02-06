{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for the Risc-V32

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
   ts80real = extended;
   ts128real = extended;
   ts64comp = comp;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_rv32imac,
       cpu_rv32ima,
       cpu_rv32im,
       cpu_rv32i
      );

   tfputype =
     (fpu_none,  
      fpu_libgcc,
      fpu_soft,
      fpu_fd
     );

   tcontrollertype =
     (ct_none,
      ct_fe310g000,
      ct_fe310g002,
      ct_hifive1,
      ct_hifive1revb,
      ct_redfive,
      ct_redfivething,
      ct_gd32vf103c4,
      ct_gd32vf103c6,
      ct_gd32vf103c8,
      ct_gd32vf103cb,
      ct_gd32vf103r4,
      ct_gd32vf103r6,
      ct_gd32vf103r8,
      ct_gd32vf103rb,
      ct_gd32vf103t4,
      ct_gd32vf103t6,
      ct_gd32vf103t8,
      ct_gd32vf103tb,
      ct_gd32vf103v8,
      ct_gd32vf103vb
     );

   tcontrollerdatatype = record
      controllertypestr, controllerunitstr: string[20];
      cputype: tcputype; fputype: tfputype;
      flashbase, flashsize, srambase, sramsize, eeprombase, eepromsize, bootbase, bootsize: dword;
   end;


Const
   { Is there support for dealing with multiple microcontrollers available }
   { for this platform? }
   ControllerSupport = true;

   { We know that there are fields after sramsize
     but we don't care about this warning }
   {$PUSH}
    {$WARN 3177 OFF}
   embedded_controllers : array [tcontrollertype] of tcontrollerdatatype =
   (
      (controllertypestr:''            ; controllerunitstr:''; cputype:cpu_none; fputype:fpu_none; flashbase:0; flashsize:0; srambase:0; sramsize:0),
      (controllertypestr:'FE310G000'   ; controllerunitstr:'FE310G000';   cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$20400000; flashsize:$01000000; srambase:$80000000; sramsize:$00004000),
      (controllertypestr:'FE310G002'   ; controllerunitstr:'FE310G002';   cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$20010000; flashsize:$00400000; srambase:$80000000; sramsize:$00004000),
      (controllertypestr:'HIFIVE1'     ; controllerunitstr:'FE310G000';   cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$20400000; flashsize:$01000000; srambase:$80000000; sramsize:$00004000),
      (controllertypestr:'HIFIVE1REVB' ; controllerunitstr:'FE310G002';   cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$20010000; flashsize:$00400000; srambase:$80000000; sramsize:$00004000),
      (controllertypestr:'REDFIVE'     ; controllerunitstr:'FE310G002';   cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$20010000; flashsize:$00400000; srambase:$80000000; sramsize:$00004000),
      (controllertypestr:'REDFIVETHING'; controllerunitstr:'FE310G002';   cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$20010000; flashsize:$02400000; srambase:$80000000; sramsize:$00004000),

      (controllertypestr:'GD32VF103C4' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001800),
      (controllertypestr:'GD32VF103C6' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00002800),
      (controllertypestr:'GD32VF103C8' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00005000),
      (controllertypestr:'GD32VF103CB' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'GD32VF103R4' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001800),
      (controllertypestr:'GD32VF103R6' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00002800),
      (controllertypestr:'GD32VF103R8' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00005000),
      (controllertypestr:'GD32VF103RB' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'GD32VF103T4' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001800),
      (controllertypestr:'GD32VF103T6' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00002800),
      (controllertypestr:'GD32VF103T8' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00005000),
      (controllertypestr:'GD32VF103TB' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'GD32VF103V8' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00005000),
      (controllertypestr:'GD32VF103VB' ; controllerunitstr:'GD32VF103XX'; cputype:cpu_rv32imac; fputype:fpu_none; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000)
   );
   {$POP}

   { calling conventions supported by the code generator }
   supported_calling_conventions : tproccalloptions = [
     pocall_internproc,
     pocall_stdcall,
     { the difference to stdcall is only the name mangling }
     pocall_cdecl,
     { the difference to stdcall is only the name mangling }
     pocall_cppdecl,
     { pass all const records by reference }
     pocall_mwpascal
   ];

   cputypestr : array[tcputype] of string[10] = ('',
     'RV32IMAC',
     'RV32IMA',
     'RV32IM',
     'RV32I'
   );

   fputypestr : array[tfputype] of string[8] = (         
     'LIBGCC',
     'NONE',
     'SOFT',
     'FD'
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,cs_opt_nodecse,
                                  cs_opt_tailrecursion,cs_opt_reorder_fields,cs_opt_fastmath,
                                  cs_opt_stackframe];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches + [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_nodecse,cs_opt_tailrecursion];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [cs_opt_stackframe]; 

 type
   tcpuflags =
      (CPURV_HAS_MUL,
       CPURV_HAS_ATOMIC,
       CPURV_HAS_COMPACT
      );

 const
   cpu_capabilities : array[tcputype] of set of tcpuflags =
     ( { cpu_none      } [],
       { cpu_rv32imac  } [CPURV_HAS_MUL,CPURV_HAS_ATOMIC,CPURV_HAS_COMPACT],
       { cpu_rv32ima   } [CPURV_HAS_MUL,CPURV_HAS_ATOMIC],
       { cpu_rv32im    } [CPURV_HAS_MUL],
       { cpu_rv32i     } []     
     );

Implementation

end.
