{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for the MIPS

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
   ts80real = type double;
   ts128real = type double;
   ts64comp = comp;

   pbestreal=^bestreal;

   { possible supported processors for this target }
   tcputype =
      (cpu_none,
       cpu_mips1,
       cpu_mips2,
       cpu_mips3,
       cpu_mips4,
       cpu_mips5,
       cpu_mips32,
       cpu_mips32r2,
       cpu_pic32mx
      );

   tfputype =(fpu_none,fpu_soft,fpu_mips2,fpu_mips3);

   tabitype = 
     (
     abi_none,
     abi_default,
     abi_o32,
     abi_n32,
     abi_o64,
     abi_n64,
     abi_eabi
     );

Const
   {# Size of native extended floating point type }
   extended_size = 8;
   { calling conventions supported by the code generator }
   supported_calling_conventions : tproccalloptions = [
     pocall_internproc,
     pocall_stdcall,
     pocall_safecall,
     { same as stdcall only different name mangling }
     pocall_cdecl,
     { same as stdcall only different name mangling }
     pocall_cppdecl
   ];

   { cpu strings as accepted by 
     GNU assembler in -arch=XXX option 
     this ilist needs to be uppercased }
   cputypestr : array[tcputype] of string[8] = ('',
     { cpu_mips1        } 'MIPS1',
     { cpu_mips2        } 'MIPS2',
     { cpu_mips3        } 'MIPS3',
     { cpu_mips4        } 'MIPS4',
     { cpu_mips5        } 'MIPS5',
     { cpu_mips32       } 'MIPS32',
     { cpu_mips32r2     } 'MIPS32R2',
     { cpu_pic32mx      } 'PIC32MX'
   );

   fputypestr : array[tfputype] of string[9] = (
     'NONE',
     'SOFT',
     'MIPS2','MIPS3'
   );

   { abi strings as accepted by 
     GNU assembler in -abi=XXX option }
   abitypestr : array[tabitype] of string[4] =
     ({ abi_none    } '',
      { abi_default } '32',
      { abi_o32     } '32',
      { abi_n32     } 'n32',
      { abi_o64     } 'o64',
      { abi_n64     } '64',
      { abi_eabi    } 'eabi'
     );

   mips_abi : tabitype = abi_default;

type
   tcpuflags=(
     CPUMIPS_HAS_CMOV,             { conditional move instructions (mips4+) }
     CPUMIPS_HAS_ISA32R2           { mips32r2 instructions (also on PIC32)  }
   );

   tcontrollerdatatype = record
      controllertypestr, controllerunitstr: string[20];
      cputype: tcputype; fputype: tfputype;
      flashbase, flashsize, srambase, sramsize, eeprombase, eepromsize, bootbase, bootsize: dword;
   end;

const
  cpu_capabilities : array[tcputype] of set of tcpuflags =
    ( { cpu_none }     [],
      { cpu_mips1 }    [],
      { cpu_mips2 }    [],
      { cpu_mips3 }    [],
      { cpu_mips4 }    [CPUMIPS_HAS_CMOV],
      { cpu_mips5 }    [CPUMIPS_HAS_CMOV],
      { cpu_mips32 }   [CPUMIPS_HAS_CMOV],
      { cpu_mips32r2 } [CPUMIPS_HAS_CMOV,CPUMIPS_HAS_ISA32R2],
      { cpu_pic32mx }  [CPUMIPS_HAS_CMOV,CPUMIPS_HAS_ISA32R2]
    );

{$ifndef MIPSEL}
type
   tcontrollertype =
     (ct_none
     );


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
{$ELSE MIPSEL}
   { Is there support for dealing with multiple microcontrollers available }
   { for this platform? }
   ControllerSupport = true;

type
   tcontrollertype =
     (ct_none,
      { pic32mx }
      ct_pic32mx110f016b,
      ct_pic32mx110f016c,
      ct_pic32mx110f016d,
      ct_pic32mx120f032b,
      ct_pic32mx120f032c,
      ct_pic32mx120f032d,
      ct_pic32mx130f064b,
      ct_pic32mx130f064c,
      ct_pic32mx130f064d,
      ct_pic32mx150f128b,
      ct_pic32mx150f128c,
      ct_pic32mx150f128d,
      ct_pic32mx210f016b,
      ct_pic32mx210f016c,
      ct_pic32mx210f016d,
      ct_pic32mx220f032b,
      ct_pic32mx220f032c,
      ct_pic32mx220f032d,
      ct_pic32mx230f064b,
      ct_pic32mx230f064c,
      ct_pic32mx230f064d,
      ct_pic32mx250f128b,
      ct_pic32mx250f128c,
      ct_pic32mx250f128d,
      ct_pic32mx775f256h,
      ct_pic32mx775f256l,
      ct_pic32mx775f512h,
      ct_pic32mx775f512l,
      ct_pic32mx795f512h,
      ct_pic32mx795f512l
     );

    { We know that there are fields after sramsize
      but we don't care about this warning }
   {$WARN 3177 OFF}
const
   embedded_controllers : array [tcontrollertype] of tcontrollerdatatype =
   (
      (controllertypestr:'';		controllerunitstr:'';	cputype: cpu_none; fputype: fpu_none; flashbase:0;	flashsize:0;	srambase:0;	sramsize:0),

      { PIC32MX1xx Series}
      (controllertypestr:'PIC32MX110F016B';	controllerunitstr:'PIC32MX1xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00004000;	srambase:$A0000000;	sramsize:$00001000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX110F016C';	controllerunitstr:'PIC32MX1xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00004000;	srambase:$A0000000;	sramsize:$00001000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX110F016D';	controllerunitstr:'PIC32MX1xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00004000;	srambase:$A0000000;	sramsize:$00001000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX120F032B';	controllerunitstr:'PIC32MX1xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00008000;	srambase:$A0000000;	sramsize:$00002000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX120F032C';	controllerunitstr:'PIC32MX1xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00008000;	srambase:$A0000000;	sramsize:$00002000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX120F032D';	controllerunitstr:'PIC32MX1xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00008000;	srambase:$A0000000;	sramsize:$00002000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX130F064B';	controllerunitstr:'PIC32MX1xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00010000;	srambase:$A0000000;	sramsize:$00004000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX130F064C';	controllerunitstr:'PIC32MX1xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00010000;	srambase:$A0000000;	sramsize:$00004000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX130F064D';	controllerunitstr:'PIC32MX1xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00010000;	srambase:$A0000000;	sramsize:$00004000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX150F128B';	controllerunitstr:'PIC32MX1xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00020000;	srambase:$A0000000;	sramsize:$00008000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX150F128C';	controllerunitstr:'PIC32MX1xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00020000;	srambase:$A0000000;	sramsize:$00008000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX150F128D';	controllerunitstr:'PIC32MX1xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00020000;	srambase:$A0000000;	sramsize:$00008000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),

      { PIC32MX2xx Series}
      (controllertypestr:'PIC32MX210F016B';	controllerunitstr:'PIC32MX2xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00004000;	srambase:$A0000000;	sramsize:$00001000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX210F016C';	controllerunitstr:'PIC32MX2xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00004000;	srambase:$A0000000;	sramsize:$00001000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX210F016D';	controllerunitstr:'PIC32MX2xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00004000;	srambase:$A0000000;	sramsize:$00001000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX220F032B';	controllerunitstr:'PIC32MX2xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00008000;	srambase:$A0000000;	sramsize:$00002000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX220F032C';	controllerunitstr:'PIC32MX2xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00008000;	srambase:$A0000000;	sramsize:$00002000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX220F032D';	controllerunitstr:'PIC32MX2xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00008000;	srambase:$A0000000;	sramsize:$00002000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX230F064B';	controllerunitstr:'PIC32MX2xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00010000;	srambase:$A0000000;	sramsize:$00004000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX230F064C';	controllerunitstr:'PIC32MX2xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00010000;	srambase:$A0000000;	sramsize:$00004000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX230F064D';	controllerunitstr:'PIC32MX2xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00010000;	srambase:$A0000000;	sramsize:$00004000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX250F128B';	controllerunitstr:'PIC32MX2xxFxxxB';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00020000;	srambase:$A0000000;	sramsize:$00008000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX250F128C';	controllerunitstr:'PIC32MX2xxFxxxC';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00020000;	srambase:$80000000;	sramsize:$00008000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),
      (controllertypestr:'PIC32MX250F128D';	controllerunitstr:'PIC32MX2xxFxxxD';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00020000;	srambase:$A0000000;	sramsize:$00008000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00000BEF),

      { PIC32MX7x5 Series}
      (controllertypestr:'PIC32MX775F256H';	controllerunitstr:'PIC32MX7x5FxxxH';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00040000;	srambase:$A0000000;	sramsize:$00010000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00002FEF),
      (controllertypestr:'PIC32MX775F256L';	controllerunitstr:'PIC32MX7x5FxxxL';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00040000;	srambase:$A0000000;	sramsize:$00010000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00002FEF),
      (controllertypestr:'PIC32MX775F512H';	controllerunitstr:'PIC32MX7x5FxxxH';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00080000;	srambase:$A0000000;	sramsize:$00010000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00002FEF),
      (controllertypestr:'PIC32MX775F512L';	controllerunitstr:'PIC32MX7x5FxxxL';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00080000;	srambase:$A0000000;	sramsize:$00010000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00002FEF),
      (controllertypestr:'PIC32MX795F512H';	controllerunitstr:'PIC32MX7x5FxxxH';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00080000;	srambase:$A0000000;	sramsize:$00020000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00002FEF),
      (controllertypestr:'PIC32MX795F512L';	controllerunitstr:'PIC32MX7x5FxxxL';	cputype: cpu_pic32mx; fputype: fpu_soft; flashbase:$9d000000;	flashsize:$00080000;	srambase:$A0000000;	sramsize:$00020000;  eeprombase:0;   eepromsize:0;    bootbase:$BFC00000;     bootsize:$00002FEF)
  );

{$endif MIPSEL}

   { Supported optimizations, only used for information }
   supported_optimizerswitches = [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,cs_opt_nodecse,
                                  cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = level1optimizerswitches + [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_nodecse];
   level3optimizerswitches = level2optimizerswitches + [cs_opt_loopunroll];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

function SetMipsABIType(const s : string) : boolean;

Implementation

uses
  cutils;

function SetMipsABIType(const s : string) : boolean;

  var
    abi : tabitype;
  begin
    SetMipsABIType:=false;
    for abi := low(tabitype) to high(tabitype) do
      if (lower(s)=abitypestr[abi]) then
        begin
          mips_abi:=abi;
          SetMipsABIType:=true;
          break;
        end;
  end;
           
end.
