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
   ts128real = type extended;
   ts64comp = extended;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_MC68000,
       cpu_MC68020,
       cpu_MC68040,
       cpu_MC68060,
       cpu_isa_a,
       cpu_isa_a_p,
       cpu_isa_b,
       cpu_isa_c,
       cpu_cfv4e
      );

   tfputype =
     (fpu_none,
      fpu_soft,
      fpu_libgcc,
      fpu_68881,
      fpu_68040,
      fpu_68060,
      fpu_coldfire
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
     pocall_register,
     pocall_stdcall,
     pocall_safecall,
     { the difference to stdcall is only the name mangling }
     pocall_cdecl,
     { the difference to stdcall is only the name mangling }
     pocall_cppdecl,
     { this is used by PalmOS, Atari and Amiga-likes }
     pocall_syscall
   ];

   cputypestr : array[tcputype] of string[8] = ('',
     '68000',
     '68020',
     '68040',
     '68060',
     'ISAA',
     'ISAA+',
     'ISAB',
     'ISAC',
     'CFV4E'
   );

   gascputypestr : array[tcputype] of string[8] = ('',
     '68000',
     '68020',
     '68040',
     '68060',
     'isaa',
     'isaaplus',
     'isab',
     'isac',
     'cfv4e'
   );

   fputypestr : array[tfputype] of string[8] = (
     'NONE',
     'SOFT',
     'LIBGCC',
     '68881',
     '68040',
     '68060',
     'COLDFIRE'
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_loopunroll,
                                  cs_opt_tailrecursion,cs_opt_nodecse,
                                  cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches +
     [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_tailrecursion,cs_opt_nodecse];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

type
  tcpuflags =
     (CPUM68K_HAS_DBRA,        { CPU supports the DBRA instruction                         }
      CPUM68K_HAS_RTD,         { CPU supports the RTD instruction                          }
      CPUM68K_HAS_CAS,         { CPU supports the CAS instruction                          }
      CPUM68K_HAS_TAS,         { CPU supports the TAS instruction                          }
      CPUM68K_HAS_BRAL,        { CPU supports the BRA.L/Bcc.L instructions                 }
      CPUM68K_HAS_ROLROR,      { CPU supports the ROL/ROR and ROXL/ROXR instructions       }
      CPUM68K_HAS_BYTEREV,     { CPU supports the BYTEREV instruction                      }
      CPUM68K_HAS_MVSMVZ,      { CPU supports the MVZ and MVS instructions                 }
      CPUM68K_HAS_MOVE16,      { CPU supports the MOVE16 instruction                       }
      CPUM68K_HAS_MULIMM,      { CPU supports MULS/MULU with immediate value               }
      CPUM68K_HAS_32BITMUL,    { CPU supports MULS/MULU 32x32 -> 32bit                     }
      CPUM68K_HAS_64BITMUL,    { CPU supports MULS/MULU 32x32 -> 64bit                     }
      CPUM68K_HAS_16BITDIV,    { CPU supports DIVS/DIVU 32/16 -> 16bit                     }
      CPUM68K_HAS_32BITDIV,    { CPU supports DIVS/DIVU 32/32 -> 32bit                     }
      CPUM68K_HAS_64BITDIV,    { CPU supports DIVS/DIVU 64/32 -> 32bit                     }
      CPUM68K_HAS_REMSREMU,    { CPU supports the REMS/REMU instructions                   }
      CPUM68K_HAS_UNALIGNED,   { CPU supports unaligned access                             }
      CPUM68K_HAS_BASEDISP,    { CPU supports addressing with 32bit base displacements     }
      CPUM68K_HAS_INDEXSCALE,  { CPU supports scaling the index register with 2 or 4       }
      CPUM68K_HAS_INDEXSCALE8, { CPU supports scaling the index register with 2, 4 or 8    }
      CPUM68K_HAS_INDEXWORD,   { CPU supports indexing with 16bit index                    }
      CPUM68K_HAS_BYTEWORDMATH { CPU supports supports 8 and 16bit aritmetic operations    }
     );

  tfpuflags =
     (FPUM68K_HAS_HARDWARE,        { FPU is actually a hardware implementation, not a software library }
      FPUM68K_HAS_EXTENDED,        { FPU has 80 bit extended support }
      FPUM68K_HAS_TRIGONOMETRY,    { FPU supports trigonometric instructions (FSIN/FCOS, etc) }
      FPUM68K_HAS_RESULTPRECISION, { FPU supports encoding the result precision into instructions }
      FPUM68K_HAS_FLOATIMMEDIATE,  { FPU supports floating point immediate values }
      FPUM68K_HAS_FINTRZ           { FPU supports the FINT/FINTRZ instruction }
     );

const
  cpu_capabilities : array[tcputype] of set of tcpuflags =
    ( { cpu_none     } [],
      { cpu_68000    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_TAS,CPUM68K_HAS_ROLROR,CPUM68K_HAS_MULIMM,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_BYTEWORDMATH],
      { cpu_68020    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_RTD,CPUM68K_HAS_CAS,CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_ROLROR,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_BASEDISP,CPUM68K_HAS_MULIMM,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_64BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_64BITDIV,CPUM68K_HAS_INDEXSCALE,CPUM68K_HAS_INDEXSCALE8,CPUM68K_HAS_INDEXWORD,CPUM68K_HAS_BYTEWORDMATH],
      { cpu_68040    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_RTD,CPUM68K_HAS_CAS,CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_ROLROR,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_BASEDISP,CPUM68K_HAS_MULIMM,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_64BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_64BITDIV,CPUM68K_HAS_MOVE16,CPUM68K_HAS_INDEXSCALE,CPUM68K_HAS_INDEXSCALE8,CPUM68K_HAS_INDEXWORD,CPUM68K_HAS_BYTEWORDMATH],
      { cpu_68060    } [CPUM68K_HAS_DBRA,CPUM68K_HAS_RTD,CPUM68K_HAS_CAS,CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_ROLROR,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_BASEDISP,CPUM68K_HAS_MULIMM,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_MOVE16,CPUM68K_HAS_INDEXSCALE,CPUM68K_HAS_INDEXSCALE8,CPUM68K_HAS_INDEXWORD,CPUM68K_HAS_BYTEWORDMATH],
      { cpu_isaa     } [CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_REMSREMU,CPUM68K_HAS_INDEXSCALE],
      { cpu_isaap    } [CPUM68K_HAS_BRAL,CPUM68K_HAS_BYTEREV,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_REMSREMU,CPUM68K_HAS_INDEXSCALE],
      { cpu_isab     } [CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_MVSMVZ,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_REMSREMU,CPUM68K_HAS_INDEXSCALE],
      { cpu_isac     } [CPUM68K_HAS_TAS,CPUM68K_HAS_BYTEREV,CPUM68K_HAS_MVSMVZ,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_REMSREMU,CPUM68K_HAS_INDEXSCALE],
      { cpu_cfv4e    } [CPUM68K_HAS_TAS,CPUM68K_HAS_BRAL,CPUM68K_HAS_MVSMVZ,CPUM68K_HAS_UNALIGNED,CPUM68K_HAS_32BITMUL,CPUM68K_HAS_16BITDIV,CPUM68K_HAS_32BITDIV,CPUM68K_HAS_REMSREMU,CPUM68K_HAS_INDEXSCALE]
    );

  { on m68k, Motorola provided a software-library, which provides full '881/2 instruction set
    compatibility on 040/060 FPU types via emulation for user code compatibility. this is slow
    though, so this capabilities list contains the capabilities supported in the hardware itself }
  fpu_capabilities : array[tfputype] of set of tfpuflags =
    ( { fpu_none     } [],
      { fpu_soft     } [],
      { fpu_libgcc   } [],
      { fpu_68881    } [FPUM68K_HAS_HARDWARE,FPUM68K_HAS_EXTENDED,FPUM68K_HAS_FLOATIMMEDIATE,FPUM68K_HAS_TRIGONOMETRY,FPUM68K_HAS_FINTRZ],
      { fpu_68040    } [FPUM68K_HAS_HARDWARE,FPUM68K_HAS_EXTENDED,FPUM68K_HAS_RESULTPRECISION,FPUM68K_HAS_FLOATIMMEDIATE],
      { fpu_68060    } [FPUM68K_HAS_HARDWARE,FPUM68K_HAS_EXTENDED,FPUM68K_HAS_RESULTPRECISION,FPUM68K_HAS_FLOATIMMEDIATE,FPUM68K_HAS_FINTRZ],
      { fpu_coldfire } [FPUM68K_HAS_HARDWARE,FPUM68K_HAS_RESULTPRECISION,FPUM68K_HAS_FINTRZ]
    );

  { all CPUs commonly called "coldfire" }
  cpu_coldfire = [cpu_isa_a,cpu_isa_a_p,cpu_isa_b,cpu_isa_c,cpu_cfv4e];

  { all CPUs commonly called "68020+" }
  cpu_mc68020p = [cpu_mc68020,cpu_mc68040,cpu_mc68060];

  { all FPUs commonly called "68881/2" }
  fpu_mc68881 = [fpu_68881,fpu_68040,fpu_68060];

Implementation

end.
