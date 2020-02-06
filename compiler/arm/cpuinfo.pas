{
    Copyright (c) 1998-2002 by the Free Pascal development team

    Basic Processor information for the ARM

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
       cpu_armv3,
       cpu_armv4,
       cpu_armv4t,
       cpu_armv5,
       cpu_armv5t,
       cpu_armv5te,
       cpu_armv5tej,
       cpu_armv6,
       cpu_armv6k,
       cpu_armv6t2,
       cpu_armv6z,
       cpu_armv6m,
       cpu_armv7,
       cpu_armv7a,
       cpu_armv7r,
       cpu_armv7m,
       cpu_armv7em
       { when new elements added afterwards,
         update class procedure tarmnodeutils.InsertObjectInfo; in narmutil.pas
       }
      );

   tinstructionset = (is_thumb,is_arm);

Type
   tfputype =
     (fpu_none,
      fpu_soft,
      fpu_libgcc,
      fpu_fpa,
      fpu_fpa10,
      fpu_fpa11,
      fpu_vfpv2,
      fpu_vfpv3,
      fpu_neon_vfpv3,
      fpu_vfpv3_d16,
      fpu_fpv4_s16,
      fpu_vfpv4,
      fpu_neon_vfpv4
      { when new elements added afterwards, update also fpu_vfp_last below and
        update class procedure tarmnodeutils.InsertObjectInfo; in narmutil.pas }
     );

Const
   fpu_vfp_first = fpu_vfpv2;
   fpu_vfp_last  = fpu_neon_vfpv4;

  fputypestrllvm : array[tfputype] of string[14] = ('',
    '',
    '',
    '',
    '',
    '',
    'fpu=vfpv2',
    'fpu=vfpv3',
    'fpu=neon-vfpv3',
    'fpu=vfpv3-d16',
    'fpu=vfpv4-s16',
    'fpu=vfpv4',
    'fpu=neon-vfpv4'
  );

Type
   tcontrollertype =
     (ct_none,

      { Phillips }
      ct_lpc810m021fn8,
      ct_lpc811m001fdh16,
      ct_lpc812m101fdh16,
      ct_lpc812m101fd20,
      ct_lpc812m101fdh20,
      ct_lpc1110fd20,
      ct_lpc1111fdh20_002,
      ct_lpc1111fhn33_101,
      ct_lpc1111fhn33_102,
      ct_lpc1111fhn33_103,
      ct_lpc1111fhn33_201,
      ct_lpc1111fhn33_202,
      ct_lpc1111fhn33_203,
      ct_lpc1112fd20_102,
      ct_lpc1112fdh20_102,
      ct_lpc1112fdh28_102,
      ct_lpc1112fhn33_101,
      ct_lpc1112fhn33_102,
      ct_lpc1112fhn33_103,
      ct_lpc1112fhn33_201,
      ct_lpc1112fhn24_202,
      ct_lpc1112fhn33_202,
      ct_lpc1112fhn33_203,
      ct_lpc1112fhi33_202,
      ct_lpc1112fhi33_203,
      ct_lpc1113fhn33_201,
      ct_lpc1113fhn33_202,
      ct_lpc1113fhn33_203,
      ct_lpc1113fhn33_301,
      ct_lpc1113fhn33_302,
      ct_lpc1113fhn33_303,
      ct_lpc1113bfd48_301,
      ct_lpc1113bfd48_302,
      ct_lpc1113bfd48_303,
      ct_lpc1114fdh28_102,
      ct_lpc1114fn28_102,
      ct_lpc1114fhn33_201,
      ct_lpc1114fhn33_202,
      ct_lpc1114fhn33_203,
      ct_lpc1114fhn33_301,
      ct_lpc1114fhn33_302,
      ct_lpc1114fhn33_303,
      ct_lpc1114fhn33_333,
      ct_lpc1114fhi33_302,
      ct_lpc1114fhi33_303,
      ct_lpc1114bfd48_301,
      ct_lpc1114bfd48_302,
      ct_lpc1114bfd48_303,
      ct_lpc1114bfd48_323,
      ct_lpc1114bfd48_333,
      ct_lpc1115bfd48_303,
      ct_lpc11c12fd48_301,
      ct_lpc11c14fd48_301,
      ct_lpc11c22fd48_301,
      ct_lpc11c24fd48_301,
      ct_lpc11d24fd48_301,
      ct_lpc1224fbd48_101,
      ct_lpc1224fbd48_121,
      ct_lpc1224fbd64_101,
      ct_lpc1224fbd64_121,
      ct_lpc1225fbd48_301,
      ct_lpc1225fbd48_321,
      ct_lpc1225fbd64_301,
      ct_lpc1225fbd64_321,
      ct_lpc1226fbd48_301,
      ct_lpc1226fbd64_301,
      ct_lpc1227fbd48_301,
      ct_lpc1227fbd64_301,
      ct_lpc12d27fbd100_301,
      ct_lpc1311fhn33,
      ct_lpc1311fhn33_01,
      ct_lpc1313fhn33,
      ct_lpc1313fhn33_01,
      ct_lpc1313fbd48,
      ct_lpc1313fbd48_01,
      ct_lpc1315fhn33,
      ct_lpc1315fbd48,
      ct_lpc1316fhn33,
      ct_lpc1316fbd48,
      ct_lpc1317fhn33,
      ct_lpc1317fbd48,
      ct_lpc1317fbd64,
      ct_lpc1342fhn33,
      ct_lpc1342fbd48,
      ct_lpc1343fhn33,
      ct_lpc1343fbd48,
      ct_lpc1345fhn33,
      ct_lpc1345fbd48,
      ct_lpc1346fhn33,
      ct_lpc1346fbd48,
      ct_lpc1347fhn33,
      ct_lpc1347fbd48,
      ct_lpc1347fbd64,

      ct_lpc2114,
      ct_lpc2124,
      ct_lpc2194,
      ct_lpc1754,
      ct_lpc1756,
      ct_lpc1758,
      ct_lpc1764,
      ct_lpc1766,
      ct_lpc1768,

      { ATMEL }
      ct_at91sam7s256,
      ct_at91sam7se256,
      ct_at91sam7x256,
      ct_at91sam7xc256,

      { STMicroelectronics }
      ct_stm32f030c6,
      ct_stm32f030c8,
      ct_stm32f030f4,
      ct_stm32f030k6,
      ct_stm32f030r8,
      ct_stm32f050c4,
      ct_stm32f050c6,
      ct_stm32f050f4,
      ct_stm32f050f6,
      ct_stm32f050g4,
      ct_stm32f050g6,
      ct_stm32f050k4,
      ct_stm32f050k6,
      ct_stm32f051c4,
      ct_stm32f051c6,
      ct_stm32f051c8,
      ct_stm32f051k4,
      ct_stm32f051k6,
      ct_stm32f051k8,
      ct_stm32f051r4,
      ct_stm32f051r6,
      ct_stm32f051r8,
      ct_stm32f091cc,
      ct_stm32f091cb,
      ct_stm32f091rc,
      ct_stm32f091rb,
      ct_stm32f091vc,
      ct_stm32f091vb,
      ct_stm32f100x4, // LD&MD value line, 4=16,6=32,8=64,b=128
      ct_stm32f100x6,
      ct_stm32f100x8,
      ct_stm32f100xB,
      ct_stm32f100xC, // HD value line, r=512,d=384,c=256
      ct_stm32f100xD,
      ct_stm32f100xE,
      ct_stm32f101x4, // LD Access line, 4=16,6=32
      ct_stm32f101x6,
      ct_stm32f101x8, // MD Access line, 8=64,B=128
      ct_stm32f101xB,
      ct_stm32f101xC, // HD Access line, C=256,D=384,E=512
      ct_stm32f101xD,
      ct_stm32f101xE,
      ct_stm32f101xF, // XL Access line, F=768,G=1M
      ct_stm32f101xG,
      ct_stm32f102x4, // LD usb access line, 4=16,6=32
      ct_stm32f102x6,
      ct_stm32f102x8, // MD usb access line, 8=64,B=128
      ct_stm32f102xB,
      ct_stm32f103x4, // LD performance line, 4=16,6=32
      ct_stm32f103x6,
      ct_stm32f103x8, // MD performance line, 8=64,B=128
      ct_stm32f103xB,
      ct_stm32f103xC, // HD performance line, C=256,D=384,E=512
      ct_stm32f103xD,
      ct_stm32f103xE,
      ct_stm32f103xF, // XL performance line, F=768,G=1M
      ct_stm32f103xG,
      ct_stm32f107x8, // MD and HD connectivity line, 8=64,B=128,C=256
      ct_stm32f107xB,
      ct_stm32f107xC,
      ct_stm32f105r8,
      ct_stm32f105rb,
      ct_stm32f105rc,
      ct_stm32f105v8,
      ct_stm32f105vb,
      ct_stm32f105vc,
      ct_stm32f107rb,
      ct_stm32f107rc,
      ct_stm32f107vb,
      ct_stm32f107vc,
      
      ct_stm32f401cb,
      ct_stm32f401rb,
      ct_stm32f401vb,
      ct_stm32f401cc,
      ct_stm32f401rc,
      ct_stm32f401vc,
      ct_discoveryf401vc,
      ct_stm32f401cd,
      ct_stm32f401rd,
      ct_stm32f401vd,
      ct_stm32f401ce,
      ct_stm32f401re,
      ct_nucleof401re,
      ct_stm32f401ve,
      ct_stm32f407vg,
      ct_discoveryf407vg,
      ct_stm32f407ig,
      ct_stm32f407zg,
      ct_stm32f407ve,
      ct_stm32f407ze,
      ct_stm32f407ie,
      ct_stm32f411cc,
      ct_stm32f411rc,
      ct_stm32f411vc,
      ct_stm32f411ce,
      ct_stm32f411re,
      ct_nucleof411re,
      ct_stm32f411ve,
      ct_discoveryf411ve,
      ct_stm32f429vg,
      ct_stm32f429zg,
      ct_stm32f429ig,
      ct_stm32f429vi,
      ct_stm32f429zi,
      ct_discoveryf429zi,
      ct_stm32f429ii,
      ct_stm32f429ve,
      ct_stm32f429ze,
      ct_stm32f429ie,
      ct_stm32f429bg,
      ct_stm32f429bi,
      ct_stm32f429be,
      ct_stm32f429ng,
      ct_stm32f429ni,
      ct_stm32f429ne,
      ct_stm32f446mc,
      ct_stm32f446rc,
      ct_stm32f446vc,
      ct_stm32f446zc,
      ct_stm32f446me,
      ct_stm32f446re,
      ct_nucleof446re,
      ct_stm32f446ve,
      ct_stm32f446ze,

      ct_stm32f745xe,
      ct_stm32f745xg,
      ct_stm32f746xe,
      ct_stm32f746xg,
      ct_stm32f756xe,
      ct_stm32f756xg,

      { TI - Fury Class - 64 K Flash, 16 K SRAM Devices }
      ct_lm3s1110,
      ct_lm3s1133,
      ct_lm3s1138,
      ct_lm3s1150,
      ct_lm3s1162,
      ct_lm3s1165,
      ct_lm3s1166,
      ct_lm3s2110,
      ct_lm3s2139,
      ct_lm3s6100,
      ct_lm3s6110,

      { TI - Fury Class - 128K Flash, 32K SRAM devices }
      ct_lm3s1601,
      ct_lm3s1608,
      ct_lm3s1620,
      ct_lm3s1635,
      ct_lm3s1636,
      ct_lm3s1637,
      ct_lm3s1651,
      ct_lm3s2601,
      ct_lm3s2608,
      ct_lm3s2620,
      ct_lm3s2637,
      ct_lm3s2651,
      ct_lm3s6610,
      ct_lm3s6611,
      ct_lm3s6618,
      ct_lm3s6633,
      ct_lm3s6637,
      ct_lm3s8630,

      { TI - Fury Class - 256K Flash, 64K SRAM devices }
      ct_lm3s1911,
      ct_lm3s1918,
      ct_lm3s1937,
      ct_lm3s1958,
      ct_lm3s1960,
      ct_lm3s1968,
      ct_lm3s1969,
      ct_lm3s2911,
      ct_lm3s2918,
      ct_lm3s2919,
      ct_lm3s2939,
      ct_lm3s2948,
      ct_lm3s2950,
      ct_lm3s2965,
      ct_lm3s6911,
      ct_lm3s6918,
      ct_lm3s6938,
      ct_lm3s6950,
      ct_lm3s6952,
      ct_lm3s6965,
      ct_lm3s8930,
      ct_lm3s8933,
      ct_lm3s8938,
      ct_lm3s8962,
      ct_lm3s8970,
      ct_lm3s8971,

      { TI - Tempest Tempest - 256 K Flash, 64 K SRAM }
      ct_lm3s5951,
      ct_lm3s5956,
      ct_lm3s1b21,
      ct_lm3s2b93,
      ct_lm3s5b91,
      ct_lm3s9b81,
      ct_lm3s9b90,
      ct_lm3s9b92,
      ct_lm3s9b95,
      ct_lm3s9b96,

      ct_lm3s5d51,

      { TI Stellaris }
      ct_lm4f120h5,
      
      { SAMSUNG }
      ct_sc32442b,
      
      { Infineon }
      ct_xmc4500x1024,
      ct_xmc4500x768,
      ct_xmc4502x768,
      ct_xmc4504x512,

      { Allwinner }
      ct_allwinner_a20,

      { Freescale }
      ct_mk20dx128vfm5,
      ct_mk20dx128vft5,
      ct_mk20dx128vlf5,
      ct_mk20dx128vlh5,
      ct_teensy30,
      ct_mk20dx128vmp5,

      ct_mk20dx32vfm5,
      ct_mk20dx32vft5,
      ct_mk20dx32vlf5,
      ct_mk20dx32vlh5,
      ct_mk20dx32vmp5,

      ct_mk20dx64vfm5,
      ct_mk20dx64vft5,
      ct_mk20dx64vlf5,
      ct_mk20dx64vlh5,
      ct_mk20dx64vmp5,

      ct_mk20dx128vlh7,
      ct_mk20dx128vlk7,
      ct_mk20dx128vll7,
      ct_mk20dx128vmc7,

      ct_mk20dx256vlh7,
      ct_mk20dx256vlk7,
      ct_mk20dx256vll7,
      ct_mk20dx256vmc7,
      ct_teensy31,
      ct_teensy32,

      ct_mk20dx64vlh7,
      ct_mk20dx64vlk7,
      ct_mk20dx64vmc7,

      ct_mk22fn512cap12,
      ct_mk22fn512cbp12,
      ct_mk22fn512vdc12,
      ct_mk22fn512vlh12,
      ct_mk22fn512vll12,
      ct_mk22fn512vmp12,
      ct_freedom_k22f,

      ct_mk64fn1m0vdc12,
      ct_mk64fn1m0vll12,
      ct_freedom_k64f,
      ct_mk64fn1m0vlq12,
      ct_mk64fn1m0vmd12,

      ct_mk64fx512vdc12,
      ct_mk64fx512vll12,
      ct_mk64fx512vlq12,
      ct_mk64fx512vmd12,

      { Atmel }
      ct_sam3x8e,
      ct_arduino_due,
      ct_flip_n_click,
      
      { Nordic Semiconductor }
      ct_nrf51422_xxaa,
      ct_nrf51422_xxab,
      ct_nrf51422_xxac,
      ct_nrf51822_xxaa,
      ct_nrf51822_xxab,
      ct_nrf51822_xxac,
      ct_nrf52832_xxaa,
      ct_nrf52840_xxaa,

      { Raspberry Pi 2 }
      ct_raspi2,

      // generic Thumb2 target
      ct_thumb2bare
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
   {# Size of native extended floating point type }
   extended_size = 12;
   { target cpu string (used by compiler options) }
   target_cpu_string = 'arm';

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
     pocall_interrupt,
     { needed sometimes on android }
     pocall_hardfloat
   ];

   cputypestr : array[tcputype] of string[8] = ('',
     'ARMV3',
     'ARMV4',
     'ARMV4T',
     'ARMV5',
     'ARMV5T',
     'ARMV5TE',
     'ARMV5TEJ',
     'ARMV6',
     'ARMV6K',
     'ARMV6T2',
     'ARMV6Z',
     'ARMV6M',
     'ARMV7',
     'ARMV7A',
     'ARMV7R',
     'ARMV7M',
     'ARMV7EM'
   );

   fputypestr : array[tfputype] of string[10] = (
     'NONE',
     'SOFT',
     'LIBGCC',
     'FPA',
     'FPA10',
     'FPA11',
     'VFPV2',
     'VFPV3',
     'NEON_VFPV3',
     'VFPV3_D16',
     'FPV4_S16',
     'VFPV4',
     'NEON_VFPV4'
   );


    { We know that there are fields after sramsize
      but we don't care about this warning }
    {$WARN 3177 OFF}

   embedded_controllers : array [tcontrollertype] of tcontrollerdatatype =
   (
      (controllertypestr:'';		controllerunitstr:'';	cputype:cpu_none; fputype:fpu_none; flashbase:0;	flashsize:0;	srambase:0;	sramsize:0),

      { LPC 8xx Series}
      (controllertypestr:'LPC810M021FN8';	  controllerunitstr:'LPC8xx';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00001000;	srambase:$10000000;	sramsize:$00000400),
      (controllertypestr:'LPC811M001FDH16';	controllerunitstr:'LPC8xx';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC812M101FDH16';	controllerunitstr:'LPC8xx';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC812M101FD20';	controllerunitstr:'LPC8xx';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC812M101FDH20';	controllerunitstr:'LPC8xx';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),

      { LPC 11xx Series}
      (controllertypestr:'LPC1110FD20';		    controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00001000;	srambase:$10000000;	sramsize:$00000400),
      (controllertypestr:'LPC1111FDH20_002';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1111FHN33_101';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1111FHN33_102';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1111FHN33_103';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1111FHN33_201';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1111FHN33_202';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1111FHN33_203';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00001000),

      (controllertypestr:'LPC1112FD20_102';	controllerunitstr:'LPC11XX';	  cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FDH20_102';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FDH28_102';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FHN33_101';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1112FHN33_102';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1112FHN33_103';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00000800),
      (controllertypestr:'LPC1112FHN33_201';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FHN24_202';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FHN33_202';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FHN33_203';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FHI33_202';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1112FHI33_203';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),

      (controllertypestr:'LPC1113FHN33_201';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1113FHN33_202';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1113FHN33_203';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1113FHN33_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1113FHN33_302';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1113FHN33_303';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1113FBD48_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1113FBD48_302';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1113FBD48_303';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00006000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1114FDH28_102';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1114FN28_102';	  controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1114FHN33_201';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1114FHN33_202';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1114FHN33_203';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1114FHN33_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FHN33_302';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FHN33_303';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FHN33_333';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000E000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FHI33_302';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FHI33_303';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FBD48_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FBD48_302';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FBD48_303';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FBD48_323';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1114FBD48_333';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000E000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1115FBD48_303';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC11C12FBD48_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC11C14FBD48_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC11C22FBD48_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC11C24FBD48_301';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC11D14FBD100_302';	controllerunitstr:'LPC11XX';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      {LPC 122x Series}
      (controllertypestr:'LPC1224FBD48_101';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1224FBD48_121';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1224FBD64_101';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1224FBD64_121';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00001000),

      (controllertypestr:'LPC1225FBD48_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1225FBD48_321';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00014000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1225FBD64_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1225FBD64_321';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00014000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1226FBD48_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00018000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1226FBD64_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00018000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1227FBD48_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1227FBD64_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC12D27FBD100_301';	controllerunitstr:'LPC122X';	cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$10000000;	sramsize:$00002000),


      (controllertypestr:'LPC1311FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1311FHN33_01';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$10000000;	sramsize:$00001000),

      (controllertypestr:'LPC1313FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1313FHN33_01';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1313FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1313FBD48_01';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1315FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1315FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1316FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1316FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1317FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1317FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1317FBD64';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1342FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),
      (controllertypestr:'LPC1342FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00004000;	srambase:$10000000;	sramsize:$00001000),

      (controllertypestr:'LPC1343FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1343FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1345FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1345FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00008000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1346FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1346FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$0000C000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC1347FHN33';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1347FBD48';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),
      (controllertypestr:'LPC1347FBD64';	controllerunitstr:'LPC13XX';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$10000000;	sramsize:$00002000),

      (controllertypestr:'LPC2114';	controllerunitstr:'LPC21x4';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$40000000;	sramsize:$00004000),
      (controllertypestr:'LPC2124';	controllerunitstr:'LPC21x4';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$40000000;	sramsize:$00004000),
      (controllertypestr:'LPC2194';	controllerunitstr:'LPC21x4';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$40000000;	sramsize:$00004000),
      
      (controllertypestr:'LPC1754';	controllerunitstr:'LPC1754';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$10000000;	sramsize:$00004000),
      (controllertypestr:'LPC1756';	controllerunitstr:'LPC1756';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$10000000;	sramsize:$00004000),
      (controllertypestr:'LPC1758';	controllerunitstr:'LPC1758';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00080000;	srambase:$10000000;	sramsize:$00008000),
      (controllertypestr:'LPC1764';	controllerunitstr:'LPC1764';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$10000000;	sramsize:$00004000),
      (controllertypestr:'LPC1766';	controllerunitstr:'LPC1766';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$10000000;	sramsize:$00008000),
      (controllertypestr:'LPC1768';	controllerunitstr:'LPC1768';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00080000;	srambase:$10000000;	sramsize:$00008000),

      { AT91 }
      (controllertypestr:'AT91SAM7S256';	controllerunitstr:'AT91SAM7x256';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$00200000;	sramsize:$00010000),
      (controllertypestr:'AT91SAM7SE256';	controllerunitstr:'AT91SAM7x256';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$00200000;	sramsize:$00010000),
      (controllertypestr:'AT91SAM7X256';	controllerunitstr:'AT91SAM7x256';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$00200000;	sramsize:$00010000),
      (controllertypestr:'AT91SAM7XC256';	controllerunitstr:'AT91SAM7x256';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$00200000;	sramsize:$00010000),

      { STM32F0 series }
      (controllertypestr:'STM32F030C6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F030C8';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F030F4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F030K6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F030R8';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F050C4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050C6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050F4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050F6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050G4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050G6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050K4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F050K6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051C4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051C6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051C8';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F051K4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051K6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051K8';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F051R4';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051R6';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F051R8';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F091CC';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F091CB';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F091RC';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F091RB';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F091VC';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F091VB';     controllerunitstr:'STM32F0XX';        cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00008000),

      { STM32F1 series }
      (controllertypestr:'STM32F100X4';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F100X6';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F100X8';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F100XB';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'STM32F100XC';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00006000),
      (controllertypestr:'STM32F100XD';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00060000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F100XE';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F101X4';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F101X6';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001800),
      (controllertypestr:'STM32F101X8';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002800),
      (controllertypestr:'STM32F101XB';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'STM32F101XC';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'STM32F101XD';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00060000; srambase:$20000000; sramsize:$0000C000),
      (controllertypestr:'STM32F101XE';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$0000C000),
      (controllertypestr:'STM32F101XF';     controllerunitstr:'STM32F10X_XL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$000C0000; srambase:$20000000; sramsize:$00014000),
      (controllertypestr:'STM32F101XG';     controllerunitstr:'STM32F10X_XL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00014000),
      (controllertypestr:'STM32F102X4';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F102X6';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00001800),
      (controllertypestr:'STM32F102X8';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00002800),
      (controllertypestr:'STM32F102XB';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'STM32F103X4';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00004000; srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'STM32F103X6';     controllerunitstr:'STM32F10X_LD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00008000; srambase:$20000000; sramsize:$00002800),
      (controllertypestr:'STM32F103X8';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00005000),
      (controllertypestr:'STM32F103XB';     controllerunitstr:'STM32F10X_MD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00005000),
      (controllertypestr:'STM32F103XC';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$0000C000),
      (controllertypestr:'STM32F103XD';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00060000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F103XE';     controllerunitstr:'STM32F10X_HD';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F103XF';     controllerunitstr:'STM32F10X_XL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$000C0000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F103XG';     controllerunitstr:'STM32F10X_XL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F107X8';     controllerunitstr:'STM32F10X_CONN';   cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F107XB';     controllerunitstr:'STM32F10X_CONN';   cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F107XC';     controllerunitstr:'STM32F10X_CONN';   cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),

      (controllertypestr:'STM32F105R8';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F105RB';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F105RC';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F105V8';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00010000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F105VB';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F105VC';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F107RB';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F107RC';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F107VB';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F107VC';     controllerunitstr:'STM32F10X_CL';     cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
    
      (controllertypestr:'STM32F401CB';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F401RB';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F401VB';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00020000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F401CC';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F401RC';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F401VC';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'DISCOVERYF401VC'; controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'STM32F401CD';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00060000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F401RD';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00060000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F401VD';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00060000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F401CE';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F401RE';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'NUCLEOF401RE';    controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F401VE';     controllerunitstr:'STM32F401XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00018000),
      (controllertypestr:'STM32F407VG';     controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'DISCOVERYF407VG'; controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F407IG';     controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F407ZG';     controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F407VE';     controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F407ZE';     controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F407IE';     controllerunitstr:'STM32F407XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F411CC';     controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F411RC';     controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F411VC';     controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F411CE';     controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F411RE';     controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'NUCLEOF411RE';    controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F411VE';     controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'DISCOVERYF411VE'; controllerunitstr:'STM32F411XE';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F429VG';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429ZG';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429IG';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429VI';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00200000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429ZI';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00200000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'DISCOVERYF429ZI'; controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00200000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429II';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00200000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429VE';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429ZE';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429IE';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429BG';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429BI';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00200000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429BE';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429NG';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429NI';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00200000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F429NE';     controllerunitstr:'STM32F429XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'STM32F446MC';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446RC';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446VC';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446ZC';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00040000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446ME';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446RE';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'NUCLEOF446RE';    controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446VE';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'STM32F446ZE';     controllerunitstr:'STM32F446XX';      cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20000000; sramsize:$00020000),
 
      (controllertypestr:'STM32F745XE';     controllerunitstr:'STM32F745';        cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20010000; sramsize:$00040000),
      (controllertypestr:'STM32F745XG';     controllerunitstr:'STM32F745';        cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20010000; sramsize:$00040000),
      (controllertypestr:'STM32F746XE';     controllerunitstr:'STM32F746';        cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20010000; sramsize:$00040000),
      (controllertypestr:'STM32F746XG';     controllerunitstr:'STM32F746';        cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20010000; sramsize:$00040000),
      (controllertypestr:'STM32F756XE';     controllerunitstr:'STM32F756';        cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00080000; srambase:$20010000; sramsize:$00040000),
      (controllertypestr:'STM32F756XG';     controllerunitstr:'STM32F756';        cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000; flashsize:$00100000; srambase:$20010000; sramsize:$00040000),

      (controllertypestr:'LM3S1110';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S1133';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S1138';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S1150';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S1162';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S1165';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S1166';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S2110';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S2139';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S6100';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),
      (controllertypestr:'LM3S6110';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00010000;	srambase:$20000000;	sramsize:$00004000),

      { TI - 128K Flash, 32K SRAM devices }
      (controllertypestr:'LM3S1601';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S1608';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S1620';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S1635';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S1636';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S1637';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S1651';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S2601';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S2608';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S2620';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S2637';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S2651';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S6610';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S6611';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S6618';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S6633';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S6637';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),
      (controllertypestr:'LM3S8630';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00020000;	srambase:$20000000;	sramsize:$00008000),

      { TI - 256K Flash, 64K SRAM devices }
      (controllertypestr:'LM3S1911';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1918';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1937';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1958';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1960';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1968';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1969';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2911';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2918';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2919';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2939';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2948';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2950';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S2965';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S6911';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S6918';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S6938';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S6950';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S6952';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S6965';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S8930';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S8933';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S8938';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S8962';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S8970';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S8971';	controllerunitstr:'LM3FURY';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),

      { TI - Tempest parts - up to 512 K Flash, 96 K SRAM }
      (controllertypestr:'LM3S5951';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S5956';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'LM3S1B21';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S2B93';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S5B91';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S9B81';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S9B90';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S9B92';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S9B95';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S9B96';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00018000),
      (controllertypestr:'LM3S5D51';	controllerunitstr:'LM3TEMPEST';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00080000;	srambase:$20000000;	sramsize:$00018000),

      { TI }
      (controllertypestr:'LM4F120H5';	controllerunitstr:'LM4F120';	cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00040000;	srambase:$20000000;	sramsize:$00008000),

      { Samsung }
      (controllertypestr:'SC32442B';	controllerunitstr:'SC32442b';	cputype:cpu_armv4t; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00000000;	srambase:$00000000;	sramsize:$08000000),
      
      { Infinion }
      (controllertypestr:'XMC4500X1024';  controllerunitstr:'XMC4500'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000;	flashsize:$00100000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'XMC4500X768';   controllerunitstr:'XMC4500'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000;	flashsize:$000C0000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'XMC4502X768';   controllerunitstr:'XMC4502'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000;	flashsize:$000C0000;	srambase:$20000000;	sramsize:$00010000),
      (controllertypestr:'XMC4504X512';   controllerunitstr:'XMC4504'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$08000000;	flashsize:$00080000;	srambase:$20000000;	sramsize:$00010000),

      { Allwinner }
      (controllertypestr:'ALLWINNER_A20'; controllerunitstr:'ALLWINNER_A20'; cputype:cpu_armv7a; fputype:fpu_vfpv4; flashbase:$00000000; flashsize:$00000000;  srambase:$40000000; sramsize:$80000000),

      { Freescale }
      (controllertypestr:'MK20DX128VFM5'; controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX128VFT5'; controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX128VLF5'; controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX128VLH5'; controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'TEENSY30'     ; controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX128VMP5'; controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00002000),

      (controllertypestr:'MK20DX32VFM5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00008000;   srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'MK20DX32VFT5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00008000;   srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'MK20DX32VLF5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00008000;   srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'MK20DX32VLH5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00008000;   srambase:$20000000; sramsize:$00001000),
      (controllertypestr:'MK20DX32VMP5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00008000;   srambase:$20000000; sramsize:$00001000),

      (controllertypestr:'MK20DX64VFM5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX64VFT5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX64VLF5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX64VLH5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX64VMP5';  controllerunitstr:'MK20D5';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),

      (controllertypestr:'MK20DX128VLH7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'MK20DX128VLK7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'MK20DX128VLL7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'MK20DX128VMC7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00020000;   srambase:$20000000; sramsize:$00004000),

      (controllertypestr:'MK20DX256VLH7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00040000;   srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'MK20DX256VLK7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00040000;   srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'MK20DX256VLL7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00040000;   srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'MK20DX256VMC7'; controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00040000;   srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'TEENSY31';      controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00040000;   srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'TEENSY32';      controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00040000;   srambase:$20000000; sramsize:$00008000),

      (controllertypestr:'MK20DX64VLH7';  controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX64VLK7';  controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),
      (controllertypestr:'MK20DX64VMC7';  controllerunitstr:'MK20D7';  cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00010000;   srambase:$20000000; sramsize:$00002000),

      (controllertypestr:'MK22FN512CAP12';controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'MK22FN512CBP12';controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'MK22FN512VDC12';controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'MK22FN512VLH12';controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'MK22FN512VLL12';controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'MK22FN512VMP12';controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'FREEDOM_K22F';  controllerunitstr:'MK22F51212'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;   flashsize:$00080000;   srambase:$20000000; sramsize:$00010000),

      (controllertypestr:'MK64FN1M0VDC12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00100000;   srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'MK64FN1M0VLL12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00100000;   srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'FREEDOM_K64F';  controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00100000;   srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'MK64FN1M0VLQ12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00100000;   srambase:$20000000; sramsize:$00030000),
      (controllertypestr:'MK64FN1M0VMD12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00100000;   srambase:$20000000; sramsize:$00030000),

      (controllertypestr:'MK64FX512VDC12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00080000;   srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'MK64FX512VLL12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00080000;   srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'MK64FX512VLQ12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00080000;   srambase:$20000000; sramsize:$00020000),
      (controllertypestr:'MK64FX512VMD12';controllerunitstr:'MK64F12'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000;      flashsize:$00080000;   srambase:$20000000; sramsize:$00020000),

      { Atmel }
      (controllertypestr:'ATSAM3X8E';     controllerunitstr:'SAM3X8E'; cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00080000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'ARDUINO_DUE';   controllerunitstr:'SAM3X8E'; cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00080000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'FLIP_N_CLICK';  controllerunitstr:'SAM3X8E'; cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00080000; flashsize:$00040000; srambase:$20000000; sramsize:$00010000),
      
      { Nordic Semiconductor }
      (controllertypestr:'NRF51422_XXAA'; controllerunitstr:'NRF51'; cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000; flashsize:$00040000;      srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'NRF51422_XXAB'; controllerunitstr:'NRF51'; cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000; flashsize:$00020000;      srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'NRF51422_XXAC'; controllerunitstr:'NRF51'; cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000; flashsize:$00040000;      srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'NRF51822_XXAA'; controllerunitstr:'NRF51'; cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000; flashsize:$00040000;      srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'NRF51822_XXAB'; controllerunitstr:'NRF51'; cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000; flashsize:$00020000;      srambase:$20000000; sramsize:$00004000),
      (controllertypestr:'NRF51822_XXAC'; controllerunitstr:'NRF51'; cputype:cpu_armv6m; fputype:fpu_soft; flashbase:$00000000; flashsize:$00040000;      srambase:$20000000; sramsize:$00008000),
      (controllertypestr:'NRF52832_XXAA'; controllerunitstr:'NRF52'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000; flashsize:$00080000; srambase:$20000000; sramsize:$00010000),
      (controllertypestr:'NRF52840_XXAA'; controllerunitstr:'NRF52'; cputype:cpu_armv7em; fputype:fpu_soft; flashbase:$00000000; flashsize:$00080000; srambase:$20000000; sramsize:$00010000),
      
      { Raspberry Pi 2 }
      (controllertypestr:'RASPI2'; controllerunitstr:'RASPI2'; cputype:cpu_armv7a; fputype:fpu_vfpv4; flashbase:$00000000; flashsize:$00000000; srambase:$00008000; sramsize:$10000000),

      { Bare bones }
      (controllertypestr:'THUMB2_BARE';	controllerunitstr:'THUMB2_BARE';	cputype:cpu_armv7m; fputype:fpu_soft; flashbase:$00000000;	flashsize:$00002000;	srambase:$20000000;	sramsize:$00000400)
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
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [cs_opt_scheduler{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

 type
   tcpuflags =
      (CPUARM_HAS_ALL_MEM,    { CPU supports LDRSB/LDRSH/LDRH/STRH instructions           }
       CPUARM_HAS_BX,         { CPU supports the BX instruction                           }
       CPUARM_HAS_BLX,        { CPU supports the BLX rX instruction                       }
       CPUARM_HAS_BLX_LABEL,  { CPU supports the BLX <label> instruction                  }
       CPUARM_HAS_CLZ,        { CPU supports the CLZ instruction                          }
       CPUARM_HAS_EDSP,       { CPU supports the PLD,STRD,LDRD,MCRR and MRRC instructions }
       CPUARM_HAS_REV,        { CPU supports the REV instruction                          }
       CPUARM_HAS_RBIT,       { CPU supports the RBIT instruction                         }
       CPUARM_HAS_DMB,        { CPU has memory barrier instructions (DMB, DSB, ISB)       }
       CPUARM_HAS_LDREX,
       CPUARM_HAS_IDIV,
       CPUARM_HAS_THUMB_IDIV,
       CPUARM_HAS_THUMB2,
       CPUARM_HAS_UMULL
      );

   tfpuflags =
      (
        FPUARM_HAS_FPA,                { fpu is an fpa based FPU                                                               }
        FPUARM_HAS_VFP_EXTENSION,      { fpu is a vfp extension                                                                }
        FPUARM_HAS_VFP_DOUBLE,         { vfp has double support                                                                }
        FPUARM_HAS_VFP_SINGLE_ONLY,    { vfp has only single support, disjunct to FPUARM_HAS_VFP_DOUBLE, for error checking    }
        FPUARM_HAS_32REGS,             { vfp has 32 regs, without this flag, 16 are assumed                                    }
        FPUARM_HAS_VMOV_CONST,         { vmov supports (some) real constants                                                   }
        FPUARM_HAS_EXCEPTION_TRAPPING, { vfp does exceptions trapping                                                          }
        FPUARM_HAS_NEON,               { fpu has neon extensions                                                               }
        FPUARM_HAS_FMA                 { fpu has fused multiply/add instructions                                               }
      );

 const
   cpu_capabilities : array[tcputype] of set of tcpuflags =
     ( { cpu_none     } [],
       { cpu_armv3    } [],
       { cpu_armv4    } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_UMULL],
       { cpu_armv4t   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_UMULL],
       { cpu_armv5    } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_CLZ,CPUARM_HAS_UMULL],
       { cpu_armv5t   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_UMULL],
       { cpu_armv5te  } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_UMULL],
       { cpu_armv5tej } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_UMULL],
       { cpu_armv6    } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_LDREX,CPUARM_HAS_UMULL],
       { cpu_armv6k   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_LDREX,CPUARM_HAS_UMULL],
       { cpu_armv6t2  } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_RBIT,CPUARM_HAS_LDREX,CPUARM_HAS_THUMB2,CPUARM_HAS_UMULL],
       { cpu_armv6z   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_LDREX,CPUARM_HAS_UMULL],
       { cpu_armv6m   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_REV],
       { the identifier armv7 is should not be used, it is considered being equal to armv7a }
       { cpu_armv7    } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_RBIT,CPUARM_HAS_LDREX,CPUARM_HAS_DMB,CPUARM_HAS_THUMB2,CPUARM_HAS_UMULL],
       { cpu_armv7a   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_RBIT,CPUARM_HAS_LDREX,CPUARM_HAS_DMB,CPUARM_HAS_THUMB2,CPUARM_HAS_UMULL],
       { cpu_armv7r   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_BLX_LABEL,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_RBIT,CPUARM_HAS_LDREX,CPUARM_HAS_THUMB_IDIV,CPUARM_HAS_DMB,CPUARM_HAS_THUMB2,CPUARM_HAS_UMULL],
       { cpu_armv7m   } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_RBIT,CPUARM_HAS_LDREX,CPUARM_HAS_THUMB_IDIV,CPUARM_HAS_DMB,CPUARM_HAS_THUMB2,CPUARM_HAS_UMULL],
       { cpu_armv7em  } [CPUARM_HAS_ALL_MEM,CPUARM_HAS_BX,CPUARM_HAS_BLX,CPUARM_HAS_CLZ,CPUARM_HAS_EDSP,CPUARM_HAS_REV,CPUARM_HAS_RBIT,CPUARM_HAS_LDREX,CPUARM_HAS_THUMB_IDIV,CPUARM_HAS_DMB,CPUARM_HAS_THUMB2,CPUARM_HAS_UMULL]
     );

     fpu_capabilities : array[tfputype] of set of tfpuflags =
       ( { fpu_none       } [],
         { fpu_soft       } [],
         { fpu_libgcc     } [],
         { fpu_fpa        } [FPUARM_HAS_FPA],
         { fpu_fpa10      } [FPUARM_HAS_FPA],
         { fpu_fpa11      } [FPUARM_HAS_FPA],
         { fpu_vfpv2      } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_DOUBLE],
         { fpu_vfpv3      } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_DOUBLE,FPUARM_HAS_32REGS,FPUARM_HAS_VMOV_CONST],
         { fpu_neon_vfpv3 } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_DOUBLE,FPUARM_HAS_32REGS,FPUARM_HAS_VMOV_CONST,FPUARM_HAS_NEON],
         { fpu_vfpv3_d16  } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_DOUBLE,FPUARM_HAS_VMOV_CONST],
         { fpu_fpv4_s16   } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_SINGLE_ONLY,FPUARM_HAS_VMOV_CONST],
         { fpu_vfpv4      } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_DOUBLE,FPUARM_HAS_32REGS,FPUARM_HAS_VMOV_CONST,FPUARM_HAS_FMA],
         { fpu_neon_vfpv4 } [FPUARM_HAS_VFP_EXTENSION,FPUARM_HAS_VFP_DOUBLE,FPUARM_HAS_32REGS,FPUARM_HAS_VMOV_CONST,FPUARM_HAS_NEON,FPUARM_HAS_FMA]
       );

   { contains all CPU supporting any kind of thumb instruction set }
   cpu_has_thumb = [cpu_armv4t,cpu_armv5t,cpu_armv5te,cpu_armv5tej,cpu_armv6t2,cpu_armv6z,cpu_armv6m,cpu_armv7a,cpu_armv7r,cpu_armv7m,cpu_armv7em];

Implementation

end.

