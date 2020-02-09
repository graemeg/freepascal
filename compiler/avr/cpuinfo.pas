{
    Copyright (c) 2008 by the Free Pascal development team

    Basic Processor information for the AVR

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
       cpu_avrtiny,
       cpu_avr1,
       cpu_avr2,
       cpu_avr25,
       cpu_avr3,
       cpu_avr31,
       cpu_avr35,
       cpu_avr4,
       cpu_avr5,
       cpu_avr51,
       cpu_avr6,
       cpu_avrxmega3
      );

   tfputype =
     (fpu_none,
      fpu_soft,
      fpu_libgcc
     );

   tcontrollertype =
     (ct_none,

      ct_avrsim,

      ct_at90can32,
      ct_at90can64,
      ct_at90can128,
      ct_at90pwm1,
      ct_at90pwm2b,
      ct_at90pwm3b,
      ct_at90pwm81,
      ct_at90pwm161,
      ct_at90pwm216,
      ct_at90pwm316,
      ct_at90usb82,
      ct_at90usb162,
      ct_at90usb646,
      ct_at90usb647,
      ct_at90usb1286,
      ct_at90usb1287,
      ct_ata6285,
      ct_ata6286,
      ct_atmega8,
      ct_atmega8a,
      ct_atmega8hva,
      ct_atmega8u2,
      ct_atmega16,
      ct_atmega16a,
      ct_atmega16hva,
      ct_atmega16hvb,
      ct_atmega16hvbrevb,
      ct_atmega16m1,
      ct_atmega16u2,
      ct_atmega16u4,
      ct_atmega32,
      ct_atmega32a,
      ct_atmega32c1,
      ct_atmega32hvb,
      ct_atmega32hvbrevb,
      ct_atmega32m1,
      ct_atmega32u2,
      ct_atmega32u4,
      ct_atmega48,
      ct_atmega48a,
      ct_atmega48p,
      ct_atmega48pa,
      ct_atmega48pb,
      ct_atmega64,
      ct_atmega64a,
      ct_atmega64c1,
      ct_atmega64hve2,
      ct_atmega64m1,
      ct_atmega64rfr2,
      ct_atmega88,
      ct_atmega88a,
      ct_atmega88p,
      ct_atmega88pa,
      ct_atmega88pb,
      ct_atmega128,
      ct_atmega128a,
      ct_atmega128rfa1,
      ct_atmega128rfr2,
      ct_atmega162,
      ct_atmega164a,
      ct_atmega164p,
      ct_atmega164pa,
      ct_atmega165a,
      ct_atmega165p,
      ct_atmega165pa,
      ct_atmega168,
      ct_atmega168a,
      ct_atmega168p,
      ct_atmega168pa,
      ct_atmega168pb,
      ct_atmega169a,
      ct_atmega169p,
      ct_atmega169pa,
      ct_atmega256rfr2,
      ct_atmega324a,
      ct_atmega324p,
      ct_atmega324pa,
      ct_atmega324pb,
      ct_atmega325,
      ct_atmega325a,
      ct_atmega325p,
      ct_atmega325pa,
      ct_atmega328,
      ct_atmega328p,
      ct_atmega328pb,
      ct_atmega329,
      ct_atmega329a,
      ct_atmega329p,
      ct_atmega329pa,
      ct_atmega406,
      ct_atmega640,
      ct_atmega644,
      ct_atmega644a,
      ct_atmega644p,
      ct_atmega644pa,
      ct_atmega644rfr2,
      ct_atmega645,
      ct_atmega645a,
      ct_atmega645p,
      ct_atmega649,
      ct_atmega649a,
      ct_atmega649p,
      ct_atmega808,
      ct_atmega809,
      ct_atmega1280,
      ct_atmega1281,
      ct_atmega1284,
      ct_atmega1284p,
      ct_atmega1284rfr2,
      ct_atmega1608,
      ct_atmega1609,
      ct_atmega2560,
      ct_atmega2561,
      ct_atmega2564rfr2,
      ct_atmega3208,
      ct_atmega3209,
      ct_atmega3250,
      ct_atmega3250a,
      ct_atmega3250p,
      ct_atmega3250pa,
      ct_atmega3290,
      ct_atmega3290a,
      ct_atmega3290p,
      ct_atmega3290pa,
      ct_atmega4808,
      ct_atmega4809,
      ct_atmega6450,
      ct_atmega6450a,
      ct_atmega6450p,
      ct_atmega6490,
      ct_atmega6490a,
      ct_atmega6490p,
      ct_atmega8515,
      ct_atmega8535,
      ct_attiny4,
      ct_attiny5,
      ct_attiny9,
      ct_attiny10,
      ct_attiny11,
      ct_attiny12,
      ct_attiny13,
      ct_attiny13a,
      ct_attiny15,
      ct_attiny20,
      ct_attiny24,
      ct_attiny24a,
      ct_attiny25,
      ct_attiny26,
      ct_attiny28,
      ct_attiny40,
      ct_attiny43u,
      ct_attiny44,
      ct_attiny44a,
      ct_attiny45,
      ct_attiny48,
      ct_attiny84,
      ct_attiny84a,
      ct_attiny85,
      ct_attiny87,
      ct_attiny88,
      ct_attiny102,
      ct_attiny104,
      ct_attiny167,
      ct_attiny202,
      ct_attiny204,
      ct_attiny212,
      ct_attiny214,
      ct_attiny261,
      ct_attiny261a,
      ct_attiny402,
      ct_attiny404,
      ct_attiny406,
      ct_attiny412,
      ct_attiny414,
      ct_attiny416,
      ct_attiny416auto,
      ct_attiny417,
      ct_attiny441,
      ct_attiny461,
      ct_attiny461a,
      ct_attiny804,
      ct_attiny806,
      ct_attiny807,
      ct_attiny814,
      ct_attiny816,
      ct_attiny817,
      ct_attiny828,
      ct_attiny841,
      ct_attiny861,
      ct_attiny861a,
      ct_attiny1604,
      ct_attiny1606,
      ct_attiny1607,
      ct_attiny1614,
      ct_attiny1616,
      ct_attiny1617,
      ct_attiny1624,
      ct_attiny1626,
      ct_attiny1627,
      ct_attiny1634,
      ct_attiny2313,
      ct_attiny2313a,
      ct_attiny3214,
      ct_attiny3216,
      ct_attiny3217,
      ct_attiny4313,
      // Controller board aliases
      ct_arduinoleonardo,
      ct_arduinomega,
      ct_arduinomicro,
      ct_arduinonano,
      ct_arduinonanoevery,
      ct_arduinouno,
      ct_atmega256rfr2xpro,
      ct_atmega324pbxpro,
      ct_atmega1284pxplained,
      ct_atmega4809xpro,
      ct_attiny817xpro,
      ct_attiny3217xpro
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
   target_cpu_string = 'avr';

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
     pocall_softfloat
   ];

   cputypestr : array[tcputype] of string[9] = ('',
     'AVRTINY',
     'AVR1',
     'AVR2',
     'AVR25',
     'AVR3',
     'AVR31',
     'AVR35',
     'AVR4',
     'AVR5',
     'AVR51',
     'AVR6',
     'AVRXMEGA3'
   );

   fputypestr : array[tfputype] of string[6] = (
     'NONE',
     'SOFT',
     'LIBGCC'
   );

    { We know that there are fields after sramsize
      but we don't care about this warning }
   {$WARN 3177 OFF}
   embedded_controllers : array [tcontrollertype] of tcontrollerdatatype =
   ((
        controllertypestr:'';
        controllerunitstr:'';
        cputype: cpu_none;
        fputype: fpu_soft;
        flashbase:0;
        flashsize:0;
        srambase:0;
        sramsize:0;
        eeprombase:0;
        eepromsize:0
        ),
        (
        controllertypestr:'AVRSIM';
        controllerunitstr:'AVRSIM';
        
        cputype: cpu_avr5;
        fputype: fpu_soft;
        flashbase:0;
        flashsize:$20000;
        srambase:256;
        sramsize:32*1024;
        eeprombase:0;
        eepromsize:4096;
        )
        ,(controllertypestr:'AT90CAN32';controllerunitstr:'AT90CAN32';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'AT90CAN64';controllerunitstr:'AT90CAN64';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'AT90CAN128';controllerunitstr:'AT90CAN128';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:4096;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'AT90PWM1';controllerunitstr:'AT90PWM1';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90PWM2B';controllerunitstr:'AT90PWM2B';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90PWM3B';controllerunitstr:'AT90PWM3B';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90PWM81';controllerunitstr:'AT90PWM81';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:256;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90PWM161';controllerunitstr:'AT90PWM161';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90PWM216';controllerunitstr:'AT90PWM216';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90PWM316';controllerunitstr:'AT90PWM316';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90USB82';controllerunitstr:'AT90USB82';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90USB162';controllerunitstr:'AT90USB162';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'AT90USB646';controllerunitstr:'AT90USB646';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'AT90USB647';controllerunitstr:'AT90USB647';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'AT90USB1286';controllerunitstr:'AT90USB1286';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'AT90USB1287';controllerunitstr:'AT90USB1287';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATA6285';controllerunitstr:'ATA6285';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:320)
        ,(controllertypestr:'ATA6286';controllerunitstr:'ATA6286';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:320)
        ,(controllertypestr:'ATMEGA8';controllerunitstr:'ATMEGA8';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA8A';controllerunitstr:'ATMEGA8A';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA8HVA';controllerunitstr:'ATMEGA8HVA';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA8U2';controllerunitstr:'ATMEGA8U2';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16';controllerunitstr:'ATMEGA16';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:96;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16A';controllerunitstr:'ATMEGA16A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:96;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16HVA';controllerunitstr:'ATMEGA16HVA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA16HVB';controllerunitstr:'ATMEGA16HVB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16HVBREVB';controllerunitstr:'ATMEGA16HVBREVB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16M1';controllerunitstr:'ATMEGA16M1';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16U2';controllerunitstr:'ATMEGA16U2';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA16U4';controllerunitstr:'ATMEGA16U4';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1280;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA32';controllerunitstr:'ATMEGA32';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:96;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32A';controllerunitstr:'ATMEGA32A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:96;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32C1';controllerunitstr:'ATMEGA32C1';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32HVB';controllerunitstr:'ATMEGA32HVB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32HVBREVB';controllerunitstr:'ATMEGA32HVBREVB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32M1';controllerunitstr:'ATMEGA32M1';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32U2';controllerunitstr:'ATMEGA32U2';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:1024;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA32U4';controllerunitstr:'ATMEGA32U4';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2560;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA48';controllerunitstr:'ATMEGA48';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA48A';controllerunitstr:'ATMEGA48A';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA48P';controllerunitstr:'ATMEGA48P';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA48PA';controllerunitstr:'ATMEGA48PA';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA48PB';controllerunitstr:'ATMEGA48PB';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATMEGA64';controllerunitstr:'ATMEGA64';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA64A';controllerunitstr:'ATMEGA64A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA64C1';controllerunitstr:'ATMEGA64C1';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA64HVE2';controllerunitstr:'ATMEGA64HVE2';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA64M1';controllerunitstr:'ATMEGA64M1';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA64RFR2';controllerunitstr:'ATMEGA64RFR2';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:512;sramsize:8192;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA88';controllerunitstr:'ATMEGA88';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA88A';controllerunitstr:'ATMEGA88A';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA88P';controllerunitstr:'ATMEGA88P';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA88PA';controllerunitstr:'ATMEGA88PA';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA88PB';controllerunitstr:'ATMEGA88PB';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA128';controllerunitstr:'ATMEGA128';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:4096;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA128A';controllerunitstr:'ATMEGA128A';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:4096;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA128RFA1';controllerunitstr:'ATMEGA128RFA1';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:512;sramsize:16384;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA128RFR2';controllerunitstr:'ATMEGA128RFR2';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:512;sramsize:16384;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA162';controllerunitstr:'ATMEGA162';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA164A';controllerunitstr:'ATMEGA164A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA164P';controllerunitstr:'ATMEGA164P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA164PA';controllerunitstr:'ATMEGA164PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA165A';controllerunitstr:'ATMEGA165A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA165P';controllerunitstr:'ATMEGA165P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA165PA';controllerunitstr:'ATMEGA165PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA168';controllerunitstr:'ATMEGA168';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA168A';controllerunitstr:'ATMEGA168A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA168P';controllerunitstr:'ATMEGA168P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA168PA';controllerunitstr:'ATMEGA168PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA168PB';controllerunitstr:'ATMEGA168PB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA169A';controllerunitstr:'ATMEGA169A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA169P';controllerunitstr:'ATMEGA169P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA169PA';controllerunitstr:'ATMEGA169PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA256RFR2';controllerunitstr:'ATMEGA256RFR2';cputype:cpu_avr6;fputype:fpu_soft;flashbase:0;flashsize:262144;srambase:512;sramsize:32768;eeprombase:0;eepromsize:8192)
        ,(controllertypestr:'ATMEGA324A';controllerunitstr:'ATMEGA324A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA324P';controllerunitstr:'ATMEGA324P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA324PA';controllerunitstr:'ATMEGA324PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA324PB';controllerunitstr:'ATMEGA324PB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA325';controllerunitstr:'ATMEGA325';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA325A';controllerunitstr:'ATMEGA325A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA325P';controllerunitstr:'ATMEGA325P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA325PA';controllerunitstr:'ATMEGA325PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA328';controllerunitstr:'ATMEGA328';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA328P';controllerunitstr:'ATMEGA328P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA328PB';controllerunitstr:'ATMEGA328PB';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA329';controllerunitstr:'ATMEGA329';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA329A';controllerunitstr:'ATMEGA329A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA329P';controllerunitstr:'ATMEGA329P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA329PA';controllerunitstr:'ATMEGA329PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA406';controllerunitstr:'ATMEGA406';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:40960;srambase:256;sramsize:2048;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA640';controllerunitstr:'ATMEGA640';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:512;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA644';controllerunitstr:'ATMEGA644';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA644A';controllerunitstr:'ATMEGA644A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA644P';controllerunitstr:'ATMEGA644P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA644PA';controllerunitstr:'ATMEGA644PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA644RFR2';controllerunitstr:'ATMEGA644RFR2';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:512;sramsize:8192;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA645';controllerunitstr:'ATMEGA645';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA645A';controllerunitstr:'ATMEGA645A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA645P';controllerunitstr:'ATMEGA645P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA649';controllerunitstr:'ATMEGA649';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA649A';controllerunitstr:'ATMEGA649A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA649P';controllerunitstr:'ATMEGA649P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA808';controllerunitstr:'ATMEGA808';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15360;sramsize:1024;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA809';controllerunitstr:'ATMEGA809';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15360;sramsize:1024;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA1280';controllerunitstr:'ATMEGA1280';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:512;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA1281';controllerunitstr:'ATMEGA1281';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:512;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA1284';controllerunitstr:'ATMEGA1284';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:16384;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA1284P';controllerunitstr:'ATMEGA1284P';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:256;sramsize:16384;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA1284RFR2';controllerunitstr:'ATMEGA1284RFR2';cputype:cpu_avr51;fputype:fpu_soft;flashbase:0;flashsize:131072;srambase:512;sramsize:16384;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA1608';controllerunitstr:'ATMEGA1608';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA1609';controllerunitstr:'ATMEGA1609';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA2560';controllerunitstr:'ATMEGA2560';cputype:cpu_avr6;fputype:fpu_soft;flashbase:0;flashsize:262144;srambase:512;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA2561';controllerunitstr:'ATMEGA2561';cputype:cpu_avr6;fputype:fpu_soft;flashbase:0;flashsize:262144;srambase:512;sramsize:8192;eeprombase:0;eepromsize:4096)
        ,(controllertypestr:'ATMEGA2564RFR2';controllerunitstr:'ATMEGA2564RFR2';cputype:cpu_avr6;fputype:fpu_soft;flashbase:0;flashsize:262144;srambase:512;sramsize:32768;eeprombase:0;eepromsize:8192)
        ,(controllertypestr:'ATMEGA3208';controllerunitstr:'ATMEGA3208';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:12288;sramsize:4096;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA3209';controllerunitstr:'ATMEGA3209';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:12288;sramsize:4096;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA3250';controllerunitstr:'ATMEGA3250';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3250A';controllerunitstr:'ATMEGA3250A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3250P';controllerunitstr:'ATMEGA3250P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3250PA';controllerunitstr:'ATMEGA3250PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3290';controllerunitstr:'ATMEGA3290';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3290A';controllerunitstr:'ATMEGA3290A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3290P';controllerunitstr:'ATMEGA3290P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA3290PA';controllerunitstr:'ATMEGA3290PA';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:256;sramsize:2048;eeprombase:0;eepromsize:1024)
        ,(controllertypestr:'ATMEGA4808';controllerunitstr:'ATMEGA4808';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:49152;srambase:10240;sramsize:6144;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA4809';controllerunitstr:'ATMEGA4809';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:49152;srambase:10240;sramsize:6144;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATMEGA6450';controllerunitstr:'ATMEGA6450';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA6450A';controllerunitstr:'ATMEGA6450A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA6450P';controllerunitstr:'ATMEGA6450P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA6490';controllerunitstr:'ATMEGA6490';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA6490A';controllerunitstr:'ATMEGA6490A';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA6490P';controllerunitstr:'ATMEGA6490P';cputype:cpu_avr5;fputype:fpu_soft;flashbase:0;flashsize:65536;srambase:256;sramsize:4096;eeprombase:0;eepromsize:2048)
        ,(controllertypestr:'ATMEGA8515';controllerunitstr:'ATMEGA8515';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATMEGA8535';controllerunitstr:'ATMEGA8535';cputype:cpu_avr4;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY4';controllerunitstr:'ATTINY4';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:512;srambase:64;sramsize:32;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY5';controllerunitstr:'ATTINY5';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:512;srambase:64;sramsize:32;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY9';controllerunitstr:'ATTINY9';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:64;sramsize:32;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY10';controllerunitstr:'ATTINY10';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:64;sramsize:32;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY11';controllerunitstr:'ATTINY11';cputype:cpu_avr1;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:0;sramsize:0;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY12';controllerunitstr:'ATTINY12';cputype:cpu_avr1;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:0;sramsize:0;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY13';controllerunitstr:'ATTINY13';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:96;sramsize:64;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY13A';controllerunitstr:'ATTINY13A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:96;sramsize:64;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY15';controllerunitstr:'ATTINY15';cputype:cpu_avr1;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:0;sramsize:0;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY20';controllerunitstr:'ATTINY20';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:64;sramsize:128;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY24';controllerunitstr:'ATTINY24';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY24A';controllerunitstr:'ATTINY24A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY25';controllerunitstr:'ATTINY25';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY26';controllerunitstr:'ATTINY26';cputype:cpu_avr2;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY28';controllerunitstr:'ATTINY28';cputype:cpu_avr1;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:0;sramsize:0;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY40';controllerunitstr:'ATTINY40';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:64;sramsize:256;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY43U';controllerunitstr:'ATTINY43U';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY44';controllerunitstr:'ATTINY44';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY44A';controllerunitstr:'ATTINY44A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY45';controllerunitstr:'ATTINY45';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY48';controllerunitstr:'ATTINY48';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:256;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY84';controllerunitstr:'ATTINY84';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY84A';controllerunitstr:'ATTINY84A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY85';controllerunitstr:'ATTINY85';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY87';controllerunitstr:'ATTINY87';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY88';controllerunitstr:'ATTINY88';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:64)
        ,(controllertypestr:'ATTINY102';controllerunitstr:'ATTINY102';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:64;sramsize:32;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY104';controllerunitstr:'ATTINY104';cputype:cpu_avrtiny;fputype:fpu_soft;flashbase:0;flashsize:1024;srambase:64;sramsize:32;eeprombase:0;eepromsize:0)
        ,(controllertypestr:'ATTINY167';controllerunitstr:'ATTINY167';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY202';controllerunitstr:'ATTINY202';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:16256;sramsize:128;eeprombase:5120;eepromsize:64)
        ,(controllertypestr:'ATTINY204';controllerunitstr:'ATTINY204';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:16256;sramsize:128;eeprombase:5120;eepromsize:64)
        ,(controllertypestr:'ATTINY212';controllerunitstr:'ATTINY212';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:16256;sramsize:128;eeprombase:5120;eepromsize:64)
        ,(controllertypestr:'ATTINY214';controllerunitstr:'ATTINY214';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:16256;sramsize:128;eeprombase:5120;eepromsize:64)
        ,(controllertypestr:'ATTINY261';controllerunitstr:'ATTINY261';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY261A';controllerunitstr:'ATTINY261A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY402';controllerunitstr:'ATTINY402';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY404';controllerunitstr:'ATTINY404';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY406';controllerunitstr:'ATTINY406';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY412';controllerunitstr:'ATTINY412';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY414';controllerunitstr:'ATTINY414';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY416';controllerunitstr:'ATTINY416';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY416AUTO';controllerunitstr:'ATTINY416AUTO';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY417';controllerunitstr:'ATTINY417';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:16128;sramsize:256;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY441';controllerunitstr:'ATTINY441';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:256;sramsize:256;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY461';controllerunitstr:'ATTINY461';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY461A';controllerunitstr:'ATTINY461A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY804';controllerunitstr:'ATTINY804';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15872;sramsize:512;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY806';controllerunitstr:'ATTINY806';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15872;sramsize:512;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY807';controllerunitstr:'ATTINY807';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15872;sramsize:512;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY814';controllerunitstr:'ATTINY814';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15872;sramsize:512;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY816';controllerunitstr:'ATTINY816';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15872;sramsize:512;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY817';controllerunitstr:'ATTINY817';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:15872;sramsize:512;eeprombase:5120;eepromsize:128)
        ,(controllertypestr:'ATTINY828';controllerunitstr:'ATTINY828';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY841';controllerunitstr:'ATTINY841';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:256;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY861';controllerunitstr:'ATTINY861';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY861A';controllerunitstr:'ATTINY861A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:8192;srambase:96;sramsize:512;eeprombase:0;eepromsize:512)
        ,(controllertypestr:'ATTINY1604';controllerunitstr:'ATTINY1604';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:15360;sramsize:1024;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1606';controllerunitstr:'ATTINY1606';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:15360;sramsize:1024;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1607';controllerunitstr:'ATTINY1607';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:15360;sramsize:1024;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1614';controllerunitstr:'ATTINY1614';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1616';controllerunitstr:'ATTINY1616';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1617';controllerunitstr:'ATTINY1617';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1624';controllerunitstr:'ATTINY1624';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1626';controllerunitstr:'ATTINY1626';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1627';controllerunitstr:'ATTINY1627';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY1634';controllerunitstr:'ATTINY1634';cputype:cpu_avr35;fputype:fpu_soft;flashbase:0;flashsize:16384;srambase:256;sramsize:1024;eeprombase:0;eepromsize:256)
        ,(controllertypestr:'ATTINY2313';controllerunitstr:'ATTINY2313';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY2313A';controllerunitstr:'ATTINY2313A';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:2048;srambase:96;sramsize:128;eeprombase:0;eepromsize:128)
        ,(controllertypestr:'ATTINY3214';controllerunitstr:'ATTINY3214';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY3216';controllerunitstr:'ATTINY3216';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY3217';controllerunitstr:'ATTINY3217';cputype:cpu_avrxmega3;fputype:fpu_soft;flashbase:0;flashsize:32768;srambase:14336;sramsize:2048;eeprombase:5120;eepromsize:256)
        ,(controllertypestr:'ATTINY4313';controllerunitstr:'ATTINY4313';cputype:cpu_avr25;fputype:fpu_soft;flashbase:0;flashsize:4096;srambase:96;sramsize:256;eeprombase:0;eepromsize:256)
        // Controller board aliases
        ,(controllertypestr:'ARDUINOLEONARDO'; controllerunitstr:'ATMEGA32U4'; cputype: cpu_avr5; fputype:fpu_soft; flashbase:0; flashsize:32768; srambase:256; sramsize:2560; eeprombase:0; eepromsize:1024)
        ,(controllertypestr:'ARDUINOMEGA'; controllerunitstr:'ATMEGA2560'; cputype: cpu_avr6; fputype:fpu_soft; flashbase:0; flashsize:262144; srambase:512; sramsize:8192; eeprombase:0; eepromsize:4096)
        ,(controllertypestr:'ARDUINOMICRO'; controllerunitstr:'ATMEGA32U4'; cputype: cpu_avr5; fputype:fpu_soft; flashbase:0; flashsize:32768; srambase:256; sramsize:2560; eeprombase:0; eepromsize:1024)
        ,(controllertypestr:'ARDUINONANO'; controllerunitstr:'ATMEGA328P'; cputype: cpu_avr5; fputype:fpu_soft; flashbase:0; flashsize:32768; srambase:256; sramsize:2048; eeprombase:0; eepromsize:1024)
        ,(controllertypestr:'ARDUINONANOEVERY'; controllerunitstr:'ATMEGA4809'; cputype: cpu_avrxmega3; fputype:fpu_soft; flashbase:0; flashsize:49152; srambase:10240; sramsize:6144; eeprombase:5120; eepromsize:256)
        ,(controllertypestr:'ARDUINOUNO'; controllerunitstr:'ATMEGA328P'; cputype: cpu_avr5; fputype:fpu_soft; flashbase:0; flashsize:32768; srambase:256; sramsize:2048; eeprombase:0; eepromsize:1024)
        ,(controllertypestr:'ATMEGA256RFR2XPRO';controllerunitstr:'ATMEGA256RFR2';cputype:cpu_avr6;fputype:fpu_soft;flashbase:0;flashsize:262144;srambase:512;sramsize:32768;eeprombase:0;eepromsize:8192)
        ,(controllertypestr:'ATMEGA324PBXPRO'; controllerunitstr:'ATMEGA324PB'; cputype: cpu_avr5; fputype:fpu_soft; flashbase:0; flashsize:32768; srambase:256; sramsize:2048; eeprombase:0; eepromsize:1024)
        ,(controllertypestr:'ATMEGA1284PXPLAINED'; controllerunitstr:'ATMEGA1284P'; cputype: cpu_avr51; fputype:fpu_soft; flashbase:0; flashsize:131072; srambase:256; sramsize:16384; eeprombase:0; eepromsize:4096)
        ,(controllertypestr:'ATMEGA4809XPRO'; controllerunitstr:'ATMEGA4809'; cputype: cpu_avrxmega3; fputype:fpu_soft; flashbase:0; flashsize:49152; srambase:10240; sramsize:6144; eeprombase:5120; eepromsize:256)
        ,(controllertypestr:'ATTINY817XPRO'; controllerunitstr:'ATTINY817'; cputype: cpu_avrxmega3; fputype:fpu_soft; flashbase:0; flashsize:8192; srambase:15872; sramsize:512; eeprombase:5120; eepromsize:128)
        ,(controllertypestr:'ATTINY3217XPRO'; controllerunitstr:'ATTINY3217'; cputype: cpu_avrxmega3; fputype:fpu_soft; flashbase:0; flashsize:32768; srambase:14336; sramsize:2048; eeprombase:5120; eepromsize:256)
   );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_loopunroll,cs_opt_tailrecursion,
                                  cs_opt_stackframe,cs_opt_nodecse,cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches +
     [{$ifndef llvm}cs_opt_regvar,{$endif}cs_opt_stackframe,cs_opt_tailrecursion];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [{,cs_opt_loopunroll}];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

 type
   tcpuflags =
      (CPUAVR_HAS_JMP_CALL,
       CPUAVR_HAS_MOVW,
       CPUAVR_HAS_LPMX,
       CPUAVR_HAS_MUL,
       CPUAVR_HAS_RAMPZ,
       CPUAVR_HAS_ELPM,
       CPUAVR_HAS_ELPMX,
       CPUAVR_2_BYTE_PC,
       CPUAVR_3_BYTE_PC,
       CPUAVR_16_REGS,
       CPUAVR_NOMEMMAPPED_REGS
      );

 const
   cpu_capabilities : array[tcputype] of set of tcpuflags =
     ( { cpu_none      } [],
       { cpu_avrtiny   } [CPUAVR_16_REGS,CPUAVR_2_BYTE_PC,CPUAVR_NOMEMMAPPED_REGS],
       { cpu_avr1      } [CPUAVR_2_BYTE_PC],
       { cpu_avr2      } [CPUAVR_HAS_LPMX,CPUAVR_2_BYTE_PC],
       { cpu_avr25     } [CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_2_BYTE_PC],
       { cpu_avr3      } [CPUAVR_HAS_JMP_CALL,CPUAVR_2_BYTE_PC],
       { cpu_avr31     } [CPUAVR_HAS_JMP_CALL,CPUAVR_HAS_RAMPZ,CPUAVR_HAS_ELPM,CPUAVR_2_BYTE_PC],
       { cpu_avr35     } [CPUAVR_HAS_JMP_CALL,CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_2_BYTE_PC],
       { cpu_avr4      } [CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_HAS_MUL,CPUAVR_2_BYTE_PC],
       { cpu_avr5      } [CPUAVR_HAS_JMP_CALL,CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_HAS_MUL,CPUAVR_2_BYTE_PC],
       { cpu_avr51     } [CPUAVR_HAS_JMP_CALL,CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_HAS_MUL,CPUAVR_HAS_RAMPZ,CPUAVR_HAS_ELPM,CPUAVR_HAS_ELPMX,CPUAVR_2_BYTE_PC],
       { cpu_avr6      } [CPUAVR_HAS_JMP_CALL,CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_HAS_MUL,CPUAVR_HAS_RAMPZ,CPUAVR_HAS_ELPM,CPUAVR_HAS_ELPMX,CPUAVR_3_BYTE_PC],
       { cpu_avrxmega3 } [CPUAVR_HAS_JMP_CALL,CPUAVR_HAS_MOVW,CPUAVR_HAS_LPMX,CPUAVR_HAS_MUL,CPUAVR_2_BYTE_PC,CPUAVR_NOMEMMAPPED_REGS]
     );

Implementation

end.
