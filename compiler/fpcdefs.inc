{$mode objfpc}
{$asmmode default}
{$H-}
{$goto on}
{$inline on}
{$interfaces corba}

{ This reduces the memory requirements a lot }
{$PACKENUM 1}
{$ifdef FPC_HAS_VARSETS}
{$ifndef FPC_BIG_ENDIAN}
{ $define USE_PACKSET1}
{$endif}
{$endif FPC_HAS_VARSETS}

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

{ If anyone wants to use interrupt for
  a specific target, add a
  $define FPC_HAS_SYSTEMS_INTERRUPT_TABLE
  to fpcdefs.inc to reactivate
  the corresponding code }
{$undef FPC_HAS_SYSTEMS_INTERRUPT_TABLE}

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
{$endif x86_64}

{$ifdef ia64}
  {$define cpuflags}
  {$define cpu64bitalu}
  {$define cpu64bitaddr}
  {$define cpuextended}
  {$define cpufloat128}
{$endif ia64}

{$ifdef alpha}
  {$define cpu64bitalu}
  {$define cpu64bitaddr}
  {$define cpurefshaveindexreg}
{$endif alpha}

{$ifdef sparc}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cputargethasfixedstack}
  {$define cpurefshaveindexreg}
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
  {$define cpurox}
  {$define cpurefshaveindexreg}
{$endif powerpc64}

{$ifdef arm}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cpufpemu}
  {$define cpuneedsdiv32helper}
  {$define cpurox}
  {$define cputargethasfixedstack}
  {$define cpurefshaveindexreg}
  {$define FPC_HAS_SYSTEMS_INTERRUPT_TABLE}
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
{$endif arm}

{$ifdef m68k}
  {$define cpu32bit}
  {$define cpu32bitaddr}
  {$define cpu32bitalu}
  {$define cpuflags}
  {$define cpufpemu}
  {$define cpurefshaveindexreg}
{$endif m68k}

{$ifdef avr}
  {$define cpu8bit}
  {$define cpu16bitaddr}
  {$define cpu8bitalu}
  {$define cpuflags}
  {$define cpunofpu}
  {$define cpunodefaultint}
  {$define cpuneedsdiv32helper}
  {$define cpuneedsmulhelper}
  {$define cpurefshaveindexreg}
{$endif avr}

{$ifdef mipsel}
  {$define mips}
{$endif mipsel}

{$ifdef mips}
  {$define cpu32bit}
  {$define cpu32bitalu}
  {$define cpu32bitaddr}
  { define cpuflags}
  {$define cputargethasfixedstack}
  {$define cpurequiresproperalignment}
  { define cpumm}
  {$define cpurefshaveindexreg}
{$endif mips}

{$ifdef jvm}
  {$define cpu32bit}
  {$define cpu64bitalu}
  {$define cpu32bitaddr}
  {$define cpuhighleveltarget}
  {$define symansistr}
{$endif}

{$IFDEF MACOS}
{$DEFINE USE_FAKE_SYSUTILS}
{$ENDIF MACOS}

{$if not defined(FPC_HAS_TYPE_EXTENDED) and defined(i386)}
{$error Cross-compiling from systems without support for an 80 bit extended floating point type to i386 is not yet supported at this time }
{$endif}
