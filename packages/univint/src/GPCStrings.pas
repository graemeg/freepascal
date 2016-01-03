{
  FPC unit compatible with GPC support for handling UCSD-Pascal strings.
}

{
    Modified for use with FPC Pascal
    Adapted from Version 200 of the GPC unit by <gpc@microbizz.nl>
    Please report any bugs to <jonas@freepascal.org>
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
    Modified for use with Free Pascal
    Version 308
=======
=======
>>>>>>> origin/fixes_2_2
{$inline on}

{
    Modified for use with Free Pascal
    Version 210
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
    Modified for use with Free Pascal
    Version 308
>>>>>>> origin/fixes_2.4
    Please report any bugs to <gpc@microbizz.nl>
}

{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit GPCStrings;
interface
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$definec UNIVERSAL_INTERFACES_VERSION $0400}
{$definec GAP_INTERFACES_VERSION $0308}
=======
{$definec UNIVERSAL_INTERFACES_VERSION $0342}
{$definec GAP_INTERFACES_VERSION $0210}
>>>>>>> graemeg/fixes_2_2
=======
{$definec UNIVERSAL_INTERFACES_VERSION $0342}
{$definec GAP_INTERFACES_VERSION $0210}
>>>>>>> origin/fixes_2_2
=======
{$definec UNIVERSAL_INTERFACES_VERSION $0400}
{$definec GAP_INTERFACES_VERSION $0308}
>>>>>>> origin/fixes_2.4

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$definec USE_CFSTR_CONSTANT_MACROS TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
>>>>>>> origin/fixes_2.4
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __arm64__ and defined CPUAARCH64}
  {$setc __arm64__ := 1}
{$elsec}
  {$setc __arm64__ := 0}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_PPC64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
>>>>>>> origin/fixes_2.4
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
<<<<<<< HEAD
{$setc TARGET_CPU_PPC_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$definec ACCESSOR_CALLS_ARE_FUNCTIONS   TRUE}
{$definec CALL_NOT_IN_CARBON             FALSE}
{$definec OLDROUTINENAMES                FALSE}
{$definec OPAQUE_TOOLBOX_STRUCTS         TRUE}
{$definec OPAQUE_UPP_TYPES               TRUE}
{$definec OTCARBONAPPLICATION            TRUE}
{$definec OTKERNEL                       FALSE}
{$definec PM_USE_SESSION_APIS            TRUE}
{$definec TARGET_API_MAC_CARBON          TRUE}
{$definec TARGET_API_MAC_OS8             FALSE}
{$definec TARGET_API_MAC_OSX             TRUE}
{$definec TARGET_CARBON                  TRUE}
{$definec TARGET_CPU_68K                 FALSE}
{$definec TARGET_CPU_MIPS                FALSE}
{$definec TARGET_CPU_SPARC               FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$definec TARGET_OS_MAC                  TRUE}
>>>>>>> graemeg/fixes_2_2
=======
{$definec TARGET_OS_MAC                  TRUE}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$definec TARGET_OS_UNIX                 FALSE}
{$definec TARGET_OS_WIN32                FALSE}
{$definec TARGET_RT_MAC_68881            FALSE}
{$definec TARGET_RT_MAC_CFM              FALSE}
{$definec TARGET_RT_MAC_MACHO            TRUE}
{$definec TYPED_FUNCTION_POINTERS        TRUE}
{$definec TYPE_BOOL                      FALSE}
{$definec TYPE_EXTENDED                  FALSE}
{$definec TYPE_LONGLONG                  TRUE}

<<<<<<< HEAD
<<<<<<< HEAD
{$modeswitch result}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
uses MacTypes;

const
  kEmptyStr15 : Str15 = '';
  kEmptyStr27 : Str27 = '';
  kEmptyStr31 : Str31 = '';
  kEmptyStr32 : Str32 = '';
  kEmptyStr63 : Str63 = '';
  kEmptyStr255 : Str255 = '';

type
  String15      = String[15];
  String27      = String[27];
  String31      = String[31];
  String32      = String[32];
  String36      = String[36];
  String63      = String[63];
  String255     = String[255];

  function Str15Length( const s: Str15 ) : Integer; inline;
  function Str27Length( const s: Str27 ) : Integer; inline;
  function Str31Length( const s: Str31 ) : Integer; inline;
  function Str32Length( const s: Str32 ) : Integer; inline;
  function Str63Length( const s: Str63 ) : Integer; inline;
  function Str255Length( const s: Str255 ) : Integer; inline;

  procedure SetStr15Length( var s: Str15; len: Integer ); inline;
  procedure SetStr27Length( var s: Str27; len: Integer ); inline;
  procedure SetStr31Length( var s: Str31; len: Integer ); inline;
  procedure SetStr32Length( var s: Str32; len: Integer ); inline;
  procedure SetStr63Length( var s: Str63; len: Integer ); inline;
  procedure SetStr255Length( var s: Str255; len: Integer ); inline;

  function CharToStr15( ch: char) : Str15; inline;
  function CharToStr27( ch: char) : Str27; inline;
  function CharToStr31( ch: char) : Str31; inline;
  function CharToStr32( ch: char) : Str32; inline;
  function CharToStr63( ch: char) : Str63; inline;
  function CharToStr255( ch: char) : Str255; inline;
  function Str15ToStr15 ( const s: Str15 ) : Str15; inline;
  function Str15ToStr27 ( const s: Str15 ) : Str27; inline;
  function Str15ToStr31 ( const s: Str15 ) : Str31; inline;
  function Str15ToStr32 ( const s: Str15 ) : Str32; inline;
  function Str15ToStr63 ( const s: Str15 ) : Str63; inline;
  function Str15ToStr255 ( const s: Str15 ) : Str255; inline;
  function Str27ToStr15 ( const s: Str27 ) : Str15; inline;
  function Str27ToStr27 ( const s: Str27 ) : Str27; inline;
  function Str27ToStr31 ( const s: Str27 ) : Str31; inline;
  function Str27ToStr32 ( const s: Str27 ) : Str32; inline;
  function Str27ToStr63 ( const s: Str27 ) : Str63; inline;
  function Str27ToStr255 ( const s: Str27 ) : Str255; inline;
  function Str31ToStr15 ( const s: Str31 ) : Str15; inline;
  function Str31ToStr27 ( const s: Str31 ) : Str27; inline;
  function Str31ToStr31 ( const s: Str31 ) : Str31; inline;
  function Str31ToStr32 ( const s: Str31 ) : Str32; inline;
  function Str31ToStr63 ( const s: Str31 ) : Str63; inline;
  function Str31ToStr255 ( const s: Str31 ) : Str255; inline;
  function Str32ToStr15 ( const s: Str32 ) : Str15; inline;
  function Str32ToStr27 ( const s: Str32 ) : Str27; inline;
  function Str32ToStr31 ( const s: Str32 ) : Str31; inline;
  function Str32ToStr32 ( const s: Str32 ) : Str32; inline;
  function Str32ToStr63 ( const s: Str32 ) : Str63; inline;
  function Str32ToStr255 ( const s: Str32 ) : Str255; inline;
  function Str63ToStr15 ( const s: Str63 ) : Str15; inline;
  function Str63ToStr27 ( const s: Str63 ) : Str27; inline;
  function Str63ToStr31 ( const s: Str63 ) : Str31; inline;
  function Str63ToStr32 ( const s: Str63 ) : Str32; inline;
  function Str63ToStr63 ( const s: Str63 ) : Str63; inline;
  function Str63ToStr255 ( const s: Str63 ) : Str255; inline;
  function Str255ToStr15 ( const s: Str255 ) : Str15; inline;
  function Str255ToStr27 ( const s: Str255 ) : Str27; inline;
  function Str255ToStr31 ( const s: Str255 ) : Str31; inline;
  function Str255ToStr32 ( const s: Str255 ) : Str32; inline;
  function Str255ToStr63 ( const s: Str255 ) : Str63; inline;
  function Str255ToStr255 ( const s: Str255 ) : Str255; inline;

  procedure Str15IntoStr15 ( const s: Str15; var theResult: Str15 ); inline;
  procedure Str15IntoStr27 ( const s: Str15; var theResult: Str27 ); inline;
  procedure Str15IntoStr31 ( const s: Str15; var theResult: Str31 ); inline;
  procedure Str15IntoStr32 ( const s: Str15; var theResult: Str32 ); inline;
  procedure Str15IntoStr63 ( const s: Str15; var theResult: Str63 ); inline;
  procedure Str15IntoStr255 ( const s: Str15; var theResult: Str255 ); inline;
  procedure Str27IntoStr15 ( const s: Str27; var theResult: Str15 ); inline;
  procedure Str27IntoStr27 ( const s: Str27; var theResult: Str27 ); inline;
  procedure Str27IntoStr31 ( const s: Str27; var theResult: Str31 ); inline;
  procedure Str27IntoStr32 ( const s: Str27; var theResult: Str32 ); inline;
  procedure Str27IntoStr63 ( const s: Str27; var theResult: Str63 ); inline;
  procedure Str27IntoStr255 ( const s: Str27; var theResult: Str255 ); inline;
  procedure Str31IntoStr15 ( const s: Str31; var theResult: Str15 ); inline;
  procedure Str31IntoStr27 ( const s: Str31; var theResult: Str27 ); inline;
  procedure Str31IntoStr31 ( const s: Str31; var theResult: Str31 ); inline;
  procedure Str31IntoStr32 ( const s: Str31; var theResult: Str32 ); inline;
  procedure Str31IntoStr63 ( const s: Str31; var theResult: Str63 ); inline;
  procedure Str31IntoStr255 ( const s: Str31; var theResult: Str255 ); inline;
  procedure Str32IntoStr15 ( const s: Str32; var theResult: Str15 ); inline;
  procedure Str32IntoStr27 ( const s: Str32; var theResult: Str27 ); inline;
  procedure Str32IntoStr31 ( const s: Str32; var theResult: Str31 ); inline;
  procedure Str32IntoStr32 ( const s: Str32; var theResult: Str32 ); inline;
  procedure Str32IntoStr63 ( const s: Str32; var theResult: Str63 ); inline;
  procedure Str32IntoStr255 ( const s: Str32; var theResult: Str255 ); inline;
  procedure Str63IntoStr15 ( const s: Str63; var theResult: Str15 ); inline;
  procedure Str63IntoStr27 ( const s: Str63; var theResult: Str27 ); inline;
  procedure Str63IntoStr31 ( const s: Str63; var theResult: Str31 ); inline;
  procedure Str63IntoStr32 ( const s: Str63; var theResult: Str32 ); inline;
  procedure Str63IntoStr63 ( const s: Str63; var theResult: Str63 ); inline;
  procedure Str63IntoStr255 ( const s: Str63; var theResult: Str255 ); inline;
  procedure Str255IntoStr15 ( const s: Str255; var theResult: Str15 ); inline;
  procedure Str255IntoStr27 ( const s: Str255; var theResult: Str27 ); inline;
  procedure Str255IntoStr31 ( const s: Str255; var theResult: Str31 ); inline;
  procedure Str255IntoStr32 ( const s: Str255; var theResult: Str32 ); inline;
  procedure Str255IntoStr63 ( const s: Str255; var theResult: Str63 ); inline;
  procedure Str255IntoStr255 ( const s: Str255; var theResult: Str255 ); inline;

  function StringToStr15( const s: String ) : Str15; inline;
  procedure StringIntoStr15( const s: String; var theResult: Str15 ); inline;
  function StringToStr27( const s: String ) : Str27; inline;
  procedure StringIntoStr27( const s: String; var theResult: Str27 ); inline;
  function StringToStr31( const s: String ) : Str31; inline;
  procedure StringIntoStr31( const s: String; var theResult: Str31 ); inline;
  function StringToStr32( const s: String ) : Str32; inline;
  procedure StringIntoStr32( const s: String; var theResult: Str32 ); inline;
  function StringToStr63( const s: String ) : Str63; inline;
  procedure StringIntoStr63( const s: String; var theResult: Str63 ); inline;
  function StringToStr255( const s: String ) : Str255; inline;
  procedure StringIntoStr255( const s: String; var theResult: Str255 ); inline;

  function Str15ToString( const s: Str15 ) : String15; inline;
  function Str27ToString( const s: Str27 ) : String27; inline;
  function Str31ToString( const s: Str31 ) : String31; inline;
  function Str32ToString( const s: Str32 ) : String32; inline;
  function Str63ToString( const s: Str63 ) : String63; inline;
  function Str255ToString( const s: Str255 ) : String255; inline;

  procedure InitStr15( var s: Str15); inline;
  procedure InitStr27( var s: Str27); inline;
  procedure InitStr31( var s: Str31); inline;
  procedure InitStr32( var s: Str32); inline;
  procedure InitStr63( var s: Str63); inline;
  procedure InitStr255( var s: Str255); inline;

  function CopyStr15( const theSource: Str15; theStart, theCount: Integer) : Str255; inline;
  function CopyStr27( const theSource: Str27; theStart, theCount: Integer) : Str255; inline;
  function CopyStr31( const theSource: Str31; theStart, theCount: Integer) : Str255; inline;
  function CopyStr32( const theSource: Str32; theStart, theCount: Integer) : Str255; inline;
  function CopyStr63( const theSource: Str63; theStart, theCount: Integer) : Str255; inline;
  function CopyStr255( const theSource: Str255; theStart, theCount: Integer) : Str255; inline;

  procedure DeleteStr15( var theDest: Str15; theStart, theCount: Integer); inline;
  procedure DeleteStr27( var theDest: Str27; theStart, theCount: Integer); inline;
  procedure DeleteStr31( var theDest: Str31; theStart, theCount: Integer); inline;
  procedure DeleteStr32( var theDest: Str32; theStart, theCount: Integer); inline;
  procedure DeleteStr63( var theDest: Str63; theStart, theCount: Integer); inline;
  procedure DeleteStr255( var theDest: Str255; theStart, theCount: Integer); inline;

  procedure InsertStr255( const theSource: Str255; var theDest: Str255; theStart: Integer); inline;

  function PosCharInStr15( theCh: Char; const theStr: Str15) : Integer; inline;
  function PosCharInStr27( theCh: Char; const theStr: Str27)  : Integer; inline;
  function PosCharInStr31( theCh: Char; const theStr: Str31)  : Integer; inline;
  function PosCharInStr32( theCh: Char; const theStr: Str32)  : Integer; inline;
  function PosCharInStr63( theCh: Char; const theStr: Str63)  : Integer; inline;
  function PosCharInStr255( theCh: Char; const theStr: Str255)  : Integer; inline;

  function PosInStr255( const theSubStr, theSourceStr: Str255)  : Integer; inline;


implementation

{$R-}
  function Str15Length( const s: Str15 ) : Integer; inline;
  begin
    result := length(s);
  end;
  
  function Str27Length( const s: Str27 )  : Integer;
  begin
    result := length(s);
  end;
  
  function Str31Length( const s: Str31 )  : Integer;
  begin
    result := length(s);
  end;
  
  function Str32Length( const s: Str32 )  : Integer;
  begin
    result := length(s);
  end;
  
  function Str63Length( const s: Str63 )  : Integer;
  begin
    result := length(s);
  end;
  
  function Str255Length( const s: Str255 )  : Integer;
  begin
    result := length(s);
  end;
  

  procedure SetStr15Length( var s: Str15; len: Integer );
  begin
    setlength(s,len);
  end;
  
  procedure SetStr27Length( var s: Str27; len: Integer );
  begin
    setlength(s,len);
  end;
  
  procedure SetStr31Length( var s: Str31; len: Integer );
  begin
    setlength(s,len);
  end;
  
  procedure SetStr32Length( var s: Str32; len: Integer );
  begin
    setlength(s,len);
  end;
  
  procedure SetStr63Length( var s: Str63; len: Integer );
  begin
    setlength(s,len);
  end;
  
  procedure SetStr255Length( var s: Str255; len: Integer );
  begin
    setlength(s,len);
  end;



  function CharToStr15( ch: char): Str15;
  begin
    result := ch;
  end;

  function CharToStr27( ch: char) : Str27;
  begin
    result := ch;
  end;

  function CharToStr31( ch: char) : Str31;
  begin
    result := ch;
  end;

  function CharToStr32( ch: char) : Str32;
  begin
    result := ch;
  end;

  function CharToStr63( ch: char) : Str63;
  begin
    result := ch;
  end;

  function CharToStr255( ch: char) : Str255;
  begin
    result := ch;
  end;


  function Str15ToStr15 ( const s: Str15 ) : Str15;
  begin
    result := s;
  end;

  function Str15ToStr27 ( const s: Str15 ) : Str27;
  begin
    result := s;
  end;

  function Str15ToStr31 ( const s: Str15 ) : Str31;
  begin
    result := s;
  end;

  function Str15ToStr32 ( const s: Str15 ) : Str32;
  begin
    result := s;
  end;

  function Str15ToStr63 ( const s: Str15 ) : Str63;
  begin
    result := s;
  end;

  function Str15ToStr255 ( const s: Str15 ) : Str255;
  begin
    result := s;
  end;

  function Str27ToStr15 ( const s: Str27 ) : Str15;
  begin
    result := s;
  end;

  function Str27ToStr27 ( const s: Str27 ) : Str27;
  begin
    result := s;
  end;

  function Str27ToStr31 ( const s: Str27 ) : Str31;
  begin
    result := s;
  end;

  function Str27ToStr32 ( const s: Str27 ) : Str32;
  begin
    result := s;
  end;

  function Str27ToStr63 ( const s: Str27 ) : Str63;
  begin
    result := s;
  end;

  function Str27ToStr255 ( const s: Str27 ) : Str255;
  begin
    result := s;
  end;

  function Str31ToStr15 ( const s: Str31 ) : Str15;
  begin
    result := s;
  end;

  function Str31ToStr27 ( const s: Str31 ) : Str27;
  begin
    result := s;
  end;

  function Str31ToStr31 ( const s: Str31 ) : Str31;
  begin
    result := s;
  end;

  function Str31ToStr32 ( const s: Str31 ) : Str32;
  begin
    result := s;
  end;

  function Str31ToStr63 ( const s: Str31 ) : Str63;
  begin
    result := s;
  end;

  function Str31ToStr255 ( const s: Str31 ) : Str255;
  begin
    result := s;
  end;

  function Str32ToStr15 ( const s: Str32 ) : Str15;
  begin
    result := s;
  end;

  function Str32ToStr27 ( const s: Str32 ) : Str27;
  begin
    result := s;
  end;

  function Str32ToStr31 ( const s: Str32 ) : Str31;
  begin
    result := s;
  end;

  function Str32ToStr32 ( const s: Str32 ) : Str32;
  begin
    result := s;
  end;

  function Str32ToStr63 ( const s: Str32 ) : Str63;
  begin
    result := s;
  end;

  function Str32ToStr255 ( const s: Str32 ) : Str255;
  begin
    result := s;
  end;

  function Str63ToStr15 ( const s: Str63 ) : Str15;
  begin
    result := s;
  end;

  function Str63ToStr27 ( const s: Str63 ) : Str27;
  begin
    result := s;
  end;

  function Str63ToStr31 ( const s: Str63 ) : Str31;
  begin
    result := s;
  end;

  function Str63ToStr32 ( const s: Str63 ) : Str32;
  begin
    result := s;
  end;

  function Str63ToStr63 ( const s: Str63 ) : Str63;
  begin
    result := s;
  end;

  function Str63ToStr255 ( const s: Str63 ) : Str255;
  begin
    result := s;
  end;

  function Str255ToStr15 ( const s: Str255 ) : Str15;
  begin
    result := s;
  end;

  function Str255ToStr27 ( const s: Str255 ) : Str27;
  begin
    result := s;
  end;

  function Str255ToStr31 ( const s: Str255 ) : Str31;
  begin
    result := s;
  end;

  function Str255ToStr32 ( const s: Str255 ) : Str32;
  begin
    result := s;
  end;

  function Str255ToStr63 ( const s: Str255 ) : Str63;
  begin
    result := s;
  end;

  function Str255ToStr255 ( const s: Str255 ) : Str255;
  begin
    result := s;
  end;


  procedure Str15IntoStr15 ( const s: Str15; var theResult: Str15 );
  begin
    theResult := s;
  end;

  procedure Str15IntoStr27 ( const s: Str15; var theResult: Str27 );
  begin
    theResult := s;
  end;

  procedure Str15IntoStr31 ( const s: Str15; var theResult: Str31 );
  begin
    theResult := s;
  end;

  procedure Str15IntoStr32 ( const s: Str15; var theResult: Str32 );
  begin
    theResult := s;
  end;

  procedure Str15IntoStr63 ( const s: Str15; var theResult: Str63 );
  begin
    theResult := s;
  end;

  procedure Str15IntoStr255 ( const s: Str15; var theResult: Str255 );
  begin
    theResult := s;
  end;

  procedure Str27IntoStr15 ( const s: Str27; var theResult: Str15 );
  begin
    theResult := s;
  end;

  procedure Str27IntoStr27 ( const s: Str27; var theResult: Str27 );
  begin
    theResult := s;
  end;

  procedure Str27IntoStr31 ( const s: Str27; var theResult: Str31 );
  begin
    theResult := s;
  end;

  procedure Str27IntoStr32 ( const s: Str27; var theResult: Str32 );
  begin
    theResult := s;
  end;

  procedure Str27IntoStr63 ( const s: Str27; var theResult: Str63 );
  begin
    theResult := s;
  end;

  procedure Str27IntoStr255 ( const s: Str27; var theResult: Str255 );
  begin
    theResult := s;
  end;

  procedure Str31IntoStr15 ( const s: Str31; var theResult: Str15 );
  begin
    theResult := s;
  end;

  procedure Str31IntoStr27 ( const s: Str31; var theResult: Str27 );
  begin
    theResult := s;
  end;

  procedure Str31IntoStr31 ( const s: Str31; var theResult: Str31 );
  begin
    theResult := s;
  end;

  procedure Str31IntoStr32 ( const s: Str31; var theResult: Str32 );
  begin
    theResult := s;
  end;

  procedure Str31IntoStr63 ( const s: Str31; var theResult: Str63 );
  begin
    theResult := s;
  end;

  procedure Str31IntoStr255 ( const s: Str31; var theResult: Str255 );
  begin
    theResult := s;
  end;

  procedure Str32IntoStr15 ( const s: Str32; var theResult: Str15 );
  begin
    theResult := s;
  end;

  procedure Str32IntoStr27 ( const s: Str32; var theResult: Str27 );
  begin
    theResult := s;
  end;

  procedure Str32IntoStr31 ( const s: Str32; var theResult: Str31 );
  begin
    theResult := s;
  end;

  procedure Str32IntoStr32 ( const s: Str32; var theResult: Str32 );
  begin
    theResult := s;
  end;

  procedure Str32IntoStr63 ( const s: Str32; var theResult: Str63 );
  begin
    theResult := s;
  end;

  procedure Str32IntoStr255 ( const s: Str32; var theResult: Str255 );
  begin
    theResult := s;
  end;

  procedure Str63IntoStr15 ( const s: Str63; var theResult: Str15 );
  begin
    theResult := s;
  end;

  procedure Str63IntoStr27 ( const s: Str63; var theResult: Str27 );
  begin
    theResult := s;
  end;

  procedure Str63IntoStr31 ( const s: Str63; var theResult: Str31 );
  begin
    theResult := s;
  end;

  procedure Str63IntoStr32 ( const s: Str63; var theResult: Str32 );
  begin
    theResult := s;
  end;

  procedure Str63IntoStr63 ( const s: Str63; var theResult: Str63 );
  begin
    theResult := s;
  end;

  procedure Str63IntoStr255 ( const s: Str63; var theResult: Str255 );
  begin
    theResult := s;
  end;

  procedure Str255IntoStr15 ( const s: Str255; var theResult: Str15 );
  begin
    theResult := s;
  end;

  procedure Str255IntoStr27 ( const s: Str255; var theResult: Str27 );
  begin
    theResult := s;
  end;

  procedure Str255IntoStr31 ( const s: Str255; var theResult: Str31 );
  begin
    theResult := s;
  end;

  procedure Str255IntoStr32 ( const s: Str255; var theResult: Str32 );
  begin
    theResult := s;
  end;

  procedure Str255IntoStr63 ( const s: Str255; var theResult: Str63 );
  begin
    theResult := s;
  end;

  procedure Str255IntoStr255 ( const s: Str255; var theResult: Str255 );
  begin
    theResult := s;
  end;


  function StringToStr15( const s: String ) : Str15;
  begin
    result := s;
  end;

  procedure StringIntoStr15( const s: String; var theResult: Str15 );
  begin
    theResult := s;
  end;

  function StringToStr27( const s: String ) : Str27;
  begin
    result := s;
  end;

  procedure StringIntoStr27( const s: String; var theResult: Str27 );
  begin
    theResult := s;
  end;

  function StringToStr31( const s: String ) : Str31;
  begin
    result := s;
  end;

  procedure StringIntoStr31( const s: String; var theResult: Str31 );
  begin
    theResult := s;
  end;

  function StringToStr32( const s: String ) : Str32;
  begin
    result := s;
  end;

  procedure StringIntoStr32( const s: String; var theResult: Str32 );
  begin
    theResult := s;
  end;

  function StringToStr63( const s: String ) : Str63;
  begin
    result := s;
  end;

  procedure StringIntoStr63( const s: String; var theResult: Str63 );
  begin
    theResult := s;
  end;

  function StringToStr255( const s: String ) : Str255;
  begin
    result := s;
  end;

  procedure StringIntoStr255( const s: String; var theResult: Str255 );
  begin
    theResult := s;
  end;


  function Str15ToString( const s: Str15 ) : String15;
  begin
    result := s;
  end;

  function Str27ToString( const s: Str27 ) : String27;
  begin
    result := s;
  end;

  function Str31ToString( const s: Str31 ) : String31;
  begin
    result := s;
  end;

  function Str32ToString( const s: Str32 ) : String32;
  begin
    result := s;
  end;

  function Str63ToString( const s: Str63 ) : String63;
  begin
    result := s;
  end;

  function Str255ToString( const s: Str255 ) : String255;
  begin
    result := s;
  end;


  procedure InitStr15( var s: Str15);
  begin
    s := ''
  end;

  procedure InitStr27( var s: Str27);
  begin
    s := ''
  end;

  procedure InitStr31( var s: Str31);
  begin
    s := ''
  end;

  procedure InitStr32( var s: Str32);
  begin
    s := ''
  end;

  procedure InitStr63( var s: Str63);
  begin
    s := ''
  end;

  procedure InitStr255( var s: Str255);
  begin
    s := ''
  end;


  function CopyStr15( const theSource: Str15; theStart, theCount: Integer) : Str255;
  begin
    result := copy(theSource,theStart,theCount)
  end;

  function CopyStr27( const theSource: Str27; theStart, theCount: Integer) : Str255;
  begin
    result := copy(theSource,theStart,theCount)
  end;

  function CopyStr31( const theSource: Str31; theStart, theCount: Integer) : Str255;
  begin
    result := copy(theSource,theStart,theCount)
  end;

  function CopyStr32( const theSource: Str32; theStart, theCount: Integer) : Str255;
  begin
    result := copy(theSource,theStart,theCount)
  end;

  function CopyStr63( const theSource: Str63; theStart, theCount: Integer) : Str255;
  begin
    result := copy(theSource,theStart,theCount)
  end;

  function CopyStr255( const theSource: Str255; theStart, theCount: Integer) : Str255;
  begin
    result := copy(theSource,theStart,theCount)
  end;


  procedure DeleteStr15( var theDest: Str15; theStart, theCount: Integer);
  begin
    delete(theDest,theStart,theCount)
  end;

  procedure DeleteStr27( var theDest: Str27; theStart, theCount: Integer);
  begin
    delete(theDest,theStart,theCount)
  end;

  procedure DeleteStr31( var theDest: Str31; theStart, theCount: Integer);
  begin
    delete(theDest,theStart,theCount)
  end;

  procedure DeleteStr32( var theDest: Str32; theStart, theCount: Integer);
  begin
    delete(theDest,theStart,theCount)
  end;

  procedure DeleteStr63( var theDest: Str63; theStart, theCount: Integer);
  begin
    delete(theDest,theStart,theCount)
  end;

  procedure DeleteStr255( var theDest: Str255; theStart, theCount: Integer);
  begin
    delete(theDest,theStart,theCount)
  end;


  procedure InsertStr255( const theSource: Str255; var theDest: Str255; theStart: Integer);
  begin
    insert(theSource,theDest,theStart);
  end;

  function PosCharInStr15( theCh: Char; const theStr: Str15)  : Integer;
  begin
    result := pos(theCh,theStr)
  end;

  function PosCharInStr27( theCh: Char; const theStr: Str27) : Integer;
  begin
    result := pos(theCh,theStr)
  end;

  function PosCharInStr31( theCh: Char; const theStr: Str31)  : Integer;
  begin
    result := pos(theCh,theStr)
  end;

  function PosCharInStr32( theCh: Char; const theStr: Str32)  : Integer;
  begin
    result := pos(theCh,theStr)
  end;

  function PosCharInStr63( theCh: Char; const theStr: Str63)  : Integer;
  begin
    result := pos(theCh,theStr)
  end;

  function PosCharInStr255( theCh: Char; const theStr: Str255)  : Integer;
  begin
    result := pos(theCh,theStr)
  end;


  function PosInStr255( const theSubStr, theSourceStr: Str255)  : Integer;
  begin
    result := pos(theSubStr,theSourceStr);
  end;

end.

