{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for the m68k

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Unit CPUInfo;

Interface

  uses
    globtype;

Type
   bestreal = double;
<<<<<<< HEAD
<<<<<<< HEAD
{$if FPC_FULLVERSION>20700}
   bestrealrec = TDoubleRec;
{$endif FPC_FULLVERSION>20700}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   ts32real = single;
   ts64real = double;
   ts80real = extended;
   ts128real = type extended;
   ts64comp = extended;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_MC68000,
       cpu_MC68020,
       cpu_MC68040,
       cpu_isa_a,
       cpu_isa_a_p,
       cpu_isa_b,
       cpu_isa_c
      );

   tfputype =
     (fpu_none,
      fpu_soft,
      fpu_libgcc,
      fpu_68881
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
     { the difference to stdcall is only the name mangling }
     pocall_cdecl,
     { the difference to stdcall is only the name mangling }
     pocall_cppdecl,
     { this used by the PalmOS port only }
     pocall_syscall
   ];

   cputypestr : array[tcputype] of string[8] = ('',
     '68000',
     '68020',
     '68040',
     'ISAA',
     'ISAA+',
     'ISAB',
     'ISAC'
   );

   gascputypestr : array[tcputype] of string[8] = ('',
     '68000',
     '68020',
     '68040',
     'isaa',
     'isaaplus',
     'isab',
     'isac'
   );

   fputypestr : array[tfputype] of string[6] = ('',
     'SOFT',
     'LIBGCC',
     '68881'
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                 [cs_opt_regvar,cs_opt_loopunroll,cs_opt_nodecse,
                                  cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches +
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                                 [cs_opt_regvar,cs_opt_loopunroll,cs_opt_nodecse];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches + 
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
     [cs_opt_regvar,cs_opt_stackframe,cs_opt_nodecse];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

type
  tcpuflags =
     (CPUM68K_HAS_DBRA,      { CPU supports the DBRA instruction                         }
      CPUM68K_HAS_CAS,       { CPU supports the CAS instruction                          }
      CPUM68K_HAS_TAS,       { CPU supports the TAS instruction                          }
      CPUM68K_HAS_BRAL,      { CPU supports the BRA.L/Bcc.L instructions                 }
      CPUM68K_HAS_ROLROR,    { CPU supports the ROL/ROR and ROXL/ROXR instructions       }
      CPUM68K_HAS_BYTEREV    { CPU supports the BYTEREV instruction                      }
     );

const
  cpu_capabilities : array[tcputype] of set of tcpuflags =
    ( { cpu_none     } [],
      { cpu_68000    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_TAS,CPUM68K_HAS_ROLROR],
      { cpu_68020    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_CAS,CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_ROLROR],
      { cpu_68040    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_CAS,CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_ROLROR],
      { cpu_isaa     } [],
      { cpu_isaap    } [CPUM68K_HAS_BRAL,CPUM68K_HAS_BYTEREV],
      { cpu_isab     } [CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL],
      { cpu_isac     } [CPUM68K_HAS_TAS,CPUM68K_HAS_BYTEREV]
    );

  { all CPUs commonly called "coldfire" }
  cpu_coldfire = [cpu_isa_a,cpu_isa_a_p,cpu_isa_b,cpu_isa_c];

Implementation

end.
