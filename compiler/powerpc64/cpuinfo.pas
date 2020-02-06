{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for the PowerPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit CPUInfo;

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
  ts128real = extended;
  ts64comp = comp;

  pbestreal = ^bestreal;

  { possible supported processors for this target }
  tcputype = (cpu_none,
    cpu_ppc970
    );

  tfputype =
    (fpu_none,
    fpu_soft,
    fpu_standard
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
  supported_calling_conventions: tproccalloptions = [
    pocall_internproc,
    pocall_stdcall,
    pocall_safecall,
    { the difference to stdcall is only the name mangling }
    pocall_cdecl,
    { the difference to stdcall is only the name mangling }
    pocall_cppdecl,
    { the difference with stdcall is that all const record
      parameters are passed by reference }
    pocall_mwpascal
    ];

  cputypestr: array[tcputype] of string[10] = ('',
    '970'
    );

  fputypestr: array[tfputype] of string[8] = (
    'NONE',
    'SOFT',
    'STANDARD'
    );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,cs_opt_nodecse,
                                  cs_opt_tailrecursion,cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches + 
     [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_nodecse,cs_opt_tailrecursion];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

implementation

end.

