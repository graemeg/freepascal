{$mode objfpc}
{$asmmode default}
{$H-}
{$goto on}
{$inline on}
<<<<<<< HEAD
<<<<<<< HEAD
{$interfaces corba}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ This reduces the memory requirements a lot }
{$PACKENUM 1}
{$ifndef FPC_BIG_ENDIAN}
{ $define USE_PACKSET1}
{$endif}

{$ifdef USE_PACKSET1}
{$PACKSET 1}
{$endif USE_PACKSET1}

{ We don't use exceptions, so turn off the implicit
  exceptions in the constructors }
{$IMPLICITEXCEPTIONS OFF}

{ Inline small functions, but not when EXTDEBUG is used }
{$ifndef EXTDEBUG}
  {$define USEINLINE}
{$endif EXTDEBUG}

{$define USEEXCEPT}

{ This fake CPU is used to allow incorporation of globtype unit
  into utils/ppudump without any CPU specific code PM }
{$ifdef generic_cpu}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cpuextended}
{$endif generic_cpu}

{$ifdef cpuarm}
  {$packrecords c}
{$endif cpuarm}

{$ifdef i8086}
  {$define cpu16bit}
  {$define cpu16bitaddr}
  {$define cpu16bitalu}
  {$define x86}
  {$define cpuflags}
  {$define cpuextended}
  {//$define SUPPORT_MMX}
  {$define cpumm}
  {$define fewintregisters}
  {//$define cpurox}
  {$define SUPPORT_SAFECALL}
  {$define cpuneedsmulhelper}
  { TODO: add another define in order to disable the div helper for 16-bit divs? }
  {$define cpuneedsdivhelper}
  {$define VOLATILE_ES}
  {$define SUPPORT_GET_FRAME}
{$endif i8086}

{$ifdef i386}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define x86}
  {$define cpuflags}
  {$define cpuextended}
  {$define SUPPORT_MMX}
  {$define cpumm}
  {$define fewintregisters}
  {$define cpurox}
  {$define SUPPORT_SAFECALL}
  {$define SUPPORT_GET_FRAME}
  {$define cpucapabilities}
{$endif i386}

{$ifdef x86_64}
  {$define x86}
  {$define cpuflags}
  {$define cpu64bitalu}
  {$define cpu64bitaddr}
  {$define cpuextended}
  {$define cpufloat128}
  {$define cputargethasfixedstack}
  {$define cpumm}
  {$define cpurox}
  {$define cpurefshaveindexreg}
  {$define SUPPORT_SAFECALL}
  {$define SUPPORT_GET_FRAME}
  {$define cpucapabilities}
{$endif x86_64}

{$ifdef sparc}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cputargethasfixedstack}
<<<<<<< HEAD
<<<<<<< HEAD
  {$define cpurefshaveindexreg}
  {$define SUPPORT_SAFECALL}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif sparc}

{$ifdef powerpc}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cputargethasfixedstack}
  {$define cpumm}
  {$define cpurox}
  {$define cpurefshaveindexreg}
{$endif powerpc}

{$ifdef powerpc64}
  {$define cpu64bitalu}
  {$define cpu64bitaddr}
  {$define cpuflags}
  {$define cputargethasfixedstack}
  {$define cpumm}
<<<<<<< HEAD
<<<<<<< HEAD
  {$define cpurox}
  {$define cpurefshaveindexreg}
  {$define cpuno32bitops}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif powerpc64}

{$ifdef arm}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cpufpemu}
  {$define cpuneedsdivhelper}
  {$define cpurox}
  {$define cputargethasfixedstack}
<<<<<<< HEAD
<<<<<<< HEAD
  {$define cpurefshaveindexreg}
  {$define cpucapabilities}
  {$define SUPPORT_SAFECALL}
  {$define SUPPORT_GET_FRAME}
  { default to armel }
  {$if not(defined(CPUARM)) and not(defined(CPUARMEB)) and not(defined(FPC_OARM)) and not(defined(FPC_ARMEB)) and not(defined(FPC_ARMHF))}
    {$define FPC_ARMEL}
  {$endif}
  { inherit FPC_ARMEL? }
  {$if defined(CPUARMEL) and not(defined(FPC_OARM)) and not(defined(FPC_ARMEB)) and not(defined(FPC_ARMHF))}
    {$define FPC_ARMEL}
  {$endif}
  { inherit FPC_ARMEB? }
  {$if defined(CPUARMEB) and not(defined(FPC_OARM)) and not(defined(FPC_ARMEL)) and not(defined(FPC_ARMHF))}
    {$define FPC_ARMEB}
  {$endif}
  { inherit FPC_ARMHF? }
  {$if defined(CPUARMHF) and not(defined(FPC_OARM)) and not(defined(FPC_ARMEL)) and not(defined(FPC_ARMEB))}
    {$define FPC_ARMHF}
  {$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif arm}

{$ifdef m68k}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cpufpemu}
  {$define cpurefshaveindexreg}
  {$define cpucapabilities}
  {$define cpuneedsdivhelper}
{$endif m68k}

{$ifdef avr}
  {$define cpu8bit}
  {$define cpu16bitaddr}
  {$define cpu8bitalu}
  {$define cpuflags}
  {$define cpunofpu}
  {$define cpunodefaultint}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$define cpuneedsdivhelper}
  {$define cpuneedsmulhelper}
  {$define cpurefshaveindexreg}
  {$define cpucapabilities}
=======
  {$define cpuneedsdiv32helper}
>>>>>>> graemeg/cpstrnew
=======
  {$define cpuneedsdiv32helper}
>>>>>>> graemeg/cpstrnew
=======
  {$define cpuneedsdiv32helper}
>>>>>>> graemeg/cpstrnew
=======
  {$define cpuneedsdiv32helper}
>>>>>>> origin/cpstrnew
{$endif avr}

{$ifdef mipsel}
  {$define mips}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$else not mipsel}
  { Define both mips and mipseb if mipsel is not defined
    but mips cpu is wanted. }
  {$ifdef mipseb}
    {$define mips}
  {$endif mipseb}
  {$ifdef mips}
    {$define mipseb}
  {$endif mips}
{$endif mipsel}


{$ifdef mips}
  {$ifndef mips64}
    {$define cpu32bit}
    {$define cpu32bitalu}
    {$define cpu32bitaddr}
  {$else}
    {$error mips64 not yet supported}
  {$endif}
  {$define cpuflags} { Flags are emulated }
  {$define cputargethasfixedstack}
  {$define cpurequiresproperalignment}
  { define cpumm}
  {$define cpurefshaveindexreg}
  {$define SUPPORT_GET_FRAME}
  {$define SUPPORT_SAFECALL}
{$endif mips}

{$ifdef jvm}
  {$define cpu32bit}
  {$define cpu64bitalu}
  {$define cpu32bitaddr}
  {$define cpuhighleveltarget}
  {$define symansistr}
  {$define SUPPORT_GET_FRAME}
{$endif}

{$ifdef aarch64}
  {$define cpu64bit}
  {$define cpu64bitaddr}
  {$define cpu64bitalu}
  {$define cpuflags}
  {$define cpurox}
  {$define cputargethasfixedstack}
  {$define cpurefshaveindexreg}
  {$define SUPPORT_GET_FRAME}
{$endif aarch64}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endif mipsel}

{$ifdef mips}
  {$define cpu32bitalu}
  {$define cpu32bitaddr}
  { $define cpuflags}
  {$define cputargethasfixedstack}
  {$define cpurequiresproperalignment}
  {$define cpumm}
{$endif mips}

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
{$IFDEF MACOS}
{$DEFINE USE_FAKE_SYSUTILS}
{$ENDIF MACOS}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Stabs is not officially supported on 64 bit targets by gdb, except on Mac OS X
  (but there we don't support it)
}
{$ifdef cpu64bitaddr}
{$ifndef USE_STABS_64}
{$define NoDbgStabs}
{$endif}
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$if not defined(FPC_HAS_TYPE_EXTENDED) and defined(i386)}
{$error Cross-compiling from systems without support for an 80 bit extended floating point type to i386 is not yet supported at this time }
{$endif}

{ llvm backends partially use other backends for parameter info calculation,
  alignment info, data sizes etc. They always support 64 bit alu though.
}
{$ifdef llvm}
  {$undef SUPPORT_MMX}
  {$undef cpu16bitalu}
  {$undef cpu32bitalu}
  {$define cpu64bitalu}
  {$define cpuhighleveltarget}
  {$define symansistr}
=======
{$define SUPPORT_UNALIGNED}

{$if not defined(cpui386) and defined(i386)}
{$error Cross-compiling from non-i386 to i386 is not yet supported at this time }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif}
