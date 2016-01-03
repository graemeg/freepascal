{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Script.h
 
     Contains:   Script Manager interfaces
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1986-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1986-2008 by Apple Inc., all rights reserved
=======
     File:       Script.p
 
     Contains:   Script Manager interfaces
 
     Version:    Technology: Mac OS 9 / Carbon
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 1986-2002 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/fixes_2_2
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======


{
    Modified for use with Free Pascal
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

>>>>>>> graemeg/fixes_2_2
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit Script;
interface
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

<<<<<<< HEAD
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
=======
>>>>>>> graemeg/fixes_2_2
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
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

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

=======
>>>>>>> graemeg/fixes_2_2

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
<<<<<<< HEAD
	{$setc TARGET_CPU_PPC64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}
>>>>>>> graemeg/fixes_2_2

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$setc ACCESSOR_CALLS_ARE_FUNCTIONS := TRUE}
{$setc CALL_NOT_IN_CARBON := FALSE}
{$setc OLDROUTINENAMES := FALSE}
{$setc OPAQUE_TOOLBOX_STRUCTS := TRUE}
{$setc OPAQUE_UPP_TYPES := TRUE}
{$setc OTCARBONAPPLICATION := TRUE}
{$setc OTKERNEL := FALSE}
{$setc PM_USE_SESSION_APIS := TRUE}
{$setc TARGET_API_MAC_CARBON := TRUE}
{$setc TARGET_API_MAC_OS8 := FALSE}
{$setc TARGET_API_MAC_OSX := TRUE}
{$setc TARGET_CARBON := TRUE}
{$setc TARGET_CPU_68K := FALSE}
{$setc TARGET_CPU_MIPS := FALSE}
{$setc TARGET_CPU_SPARC := FALSE}
<<<<<<< HEAD
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,IntlResources;
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ Meta script codes:}
const
	smSystemScript = -1;   { designates system script.}
	smCurrentScript = -2;   { designates current font script.}
	smAllScripts = -3;    { designates any script}

{
   Script codes:
   These specify a Mac OS encoding that is related to a FOND ID range.
   Some of the encodings have several variants (e.g. for different localized systems)
    which all share the same script code.
   Not all of these script codes are currently supported by Apple software.
   Notes:
   - Script code 0 (smRoman) is also used (instead of smGreek) for the Greek encoding
     in the Greek localized system.
   - Script code 28 (smEthiopic) is also used for the Inuit encoding in the Inuktitut
     system.
}
const
	smRoman = 0;
	smJapanese = 1;
	smTradChinese = 2;    { Traditional Chinese}
	smKorean = 3;
	smArabic = 4;
	smHebrew = 5;
	smGreek = 6;
	smCyrillic = 7;
	smRSymbol = 8;    { Right-left symbol}
	smDevanagari = 9;
	smGurmukhi = 10;
	smGujarati = 11;
	smOriya = 12;
	smBengali = 13;
	smTamil = 14;
	smTelugu = 15;
	smKannada = 16;   { Kannada/Kanarese}
	smMalayalam = 17;
	smSinhalese = 18;
	smBurmese = 19;
	smKhmer = 20;   { Khmer/Cambodian}
	smThai = 21;
	smLao = 22;
	smGeorgian = 23;
	smArmenian = 24;
	smSimpChinese = 25;   { Simplified Chinese}
	smTibetan = 26;
	smMongolian = 27;
	smEthiopic = 28;
	smGeez = 28;   { Synonym for smEthiopic}
	smCentralEuroRoman = 29;   { For Czech, Slovak, Polish, Hungarian, Baltic langs}
	smVietnamese = 30;
	smExtArabic = 31;   { extended Arabic}
	smUninterp = 32;    { uninterpreted symbols, e.g. palette symbols}

{ Extended script code for full Unicode input}
const
	smUnicodeScript = $7E;

{ Obsolete script code names (kept for backward compatibility):}
const
	smChinese = 2;    { (Use smTradChinese or smSimpChinese)}
	smRussian = 7;    { Use smCyrillic}
                                        { smMaldivian = 25: deleted, no code for Maldivian}
	smLaotian = 22;   { Use smLao                                     }
	smAmharic = 28;   { Use smEthiopic or smGeez}
	smSlavic = 29;   { Use smCentralEuroRoman}
	smEastEurRoman = 29;   { Use smCentralEuroRoman}
	smSindhi = 31;   { Use smExtArabic}
	smKlingon = 32;

{
   Language codes:
   These specify a language implemented using a particular Mac OS encoding.
   Not all of these language codes are currently supported by Apple software.
}
const
	langEnglish = 0;    { smRoman script}
	langFrench = 1;    { smRoman script}
	langGerman = 2;    { smRoman script}
	langItalian = 3;    { smRoman script}
	langDutch = 4;    { smRoman script}
	langSwedish = 5;    { smRoman script}
	langSpanish = 6;    { smRoman script}
	langDanish = 7;    { smRoman script}
	langPortuguese = 8;    { smRoman script}
	langNorwegian = 9;    { (Bokmal) smRoman script}
	langHebrew = 10;   { smHebrew script}
	langJapanese = 11;   { smJapanese script}
	langArabic = 12;   { smArabic script}
	langFinnish = 13;   { smRoman script}
	langGreek = 14;   { Greek script (monotonic) using smRoman script code}
	langIcelandic = 15;   { modified smRoman/Icelandic script}
	langMaltese = 16;   { Roman script}
	langTurkish = 17;   { modified smRoman/Turkish script}
	langCroatian = 18;   { modified smRoman/Croatian script}
	langTradChinese = 19;   { Chinese (Mandarin) in traditional characters}
	langUrdu = 20;   { smArabic script}
	langHindi = 21;   { smDevanagari script}
	langThai = 22;   { smThai script}
	langKorean = 23;    { smKorean script}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

const
	langLithuanian = 24;   { smCentralEuroRoman script}
	langPolish = 25;   { smCentralEuroRoman script}
	langHungarian = 26;   { smCentralEuroRoman script}
	langEstonian = 27;   { smCentralEuroRoman script}
	langLatvian = 28;   { smCentralEuroRoman script}
	langSami = 29;   { language of the Sami people of N. Scandinavia             }
	langFaroese = 30;   { modified smRoman/Icelandic script                      }
	langFarsi = 31;   { modified smArabic/Farsi script}
	langPersian = 31;   { Synonym for langFarsi}
	langRussian = 32;   { smCyrillic script}
	langSimpChinese = 33;   { Chinese (Mandarin) in simplified characters}
	langFlemish = 34;   { smRoman script}
	langIrishGaelic = 35;   { smRoman or modified smRoman/Celtic script (without dot above)   }
	langAlbanian = 36;   { smRoman script}
	langRomanian = 37;   { modified smRoman/Romanian script}
	langCzech = 38;   { smCentralEuroRoman script}
	langSlovak = 39;   { smCentralEuroRoman script}
	langSlovenian = 40;   { modified smRoman/Croatian script}
	langYiddish = 41;   { smHebrew script}
	langSerbian = 42;   { smCyrillic script}
	langMacedonian = 43;   { smCyrillic script}
	langBulgarian = 44;   { smCyrillic script}
	langUkrainian = 45;   { modified smCyrillic/Ukrainian script}
	langByelorussian = 46;   { smCyrillic script}
	langBelorussian = 46;    { Synonym for langByelorussian                          }

const
	langUzbek = 47;   { Cyrillic script}
	langKazakh = 48;   { Cyrillic script}
	langAzerbaijani = 49;   { Azerbaijani in Cyrillic script}
	langAzerbaijanAr = 50;   { Azerbaijani in Arabic script}
	langArmenian = 51;   { smArmenian script}
	langGeorgian = 52;   { smGeorgian script}
	langMoldavian = 53;   { smCyrillic script}
	langKirghiz = 54;   { Cyrillic script}
	langTajiki = 55;   { Cyrillic script}
	langTurkmen = 56;   { Cyrillic script}
	langMongolian = 57;   { Mongolian in smMongolian script}
	langMongolianCyr = 58;   { Mongolian in Cyrillic script}
	langPashto = 59;   { Arabic script}
	langKurdish = 60;   { smArabic script}
	langKashmiri = 61;   { Arabic script}
	langSindhi = 62;   { Arabic script}
	langTibetan = 63;   { smTibetan script}
	langNepali = 64;   { smDevanagari script}
	langSanskrit = 65;   { smDevanagari script}
	langMarathi = 66;   { smDevanagari script}
	langBengali = 67;   { smBengali script}
	langAssamese = 68;   { smBengali script}
	langGujarati = 69;   { smGujarati script}
	langPunjabi = 70;    { smGurmukhi script}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	langLithuanian = 24;   { smCentralEuroRoman script}
	langPolish = 25;   { smCentralEuroRoman script}
	langHungarian = 26;   { smCentralEuroRoman script}
	langEstonian = 27;   { smCentralEuroRoman script}
	langLatvian = 28;   { smCentralEuroRoman script}
	langSami = 29;   { language of the Sami people of N. Scandinavia             }
	langFaroese = 30;   { modified smRoman/Icelandic script                      }
	langFarsi = 31;   { modified smArabic/Farsi script}
	langPersian = 31;   { Synonym for langFarsi}
	langRussian = 32;   { smCyrillic script}
	langSimpChinese = 33;   { Chinese (Mandarin) in simplified characters}
	langFlemish = 34;   { smRoman script}
	langIrishGaelic = 35;   { smRoman or modified smRoman/Celtic script (without dot above)   }
	langAlbanian = 36;   { smRoman script}
	langRomanian = 37;   { modified smRoman/Romanian script}
	langCzech = 38;   { smCentralEuroRoman script}
	langSlovak = 39;   { smCentralEuroRoman script}
	langSlovenian = 40;   { modified smRoman/Croatian script}
	langYiddish = 41;   { smHebrew script}
	langSerbian = 42;   { smCyrillic script}
	langMacedonian = 43;   { smCyrillic script}
	langBulgarian = 44;   { smCyrillic script}
	langUkrainian = 45;   { modified smCyrillic/Ukrainian script}
	langByelorussian = 46;   { smCyrillic script}
	langBelorussian = 46;    { Synonym for langByelorussian                          }

const
	langUzbek = 47;   { Cyrillic script}
	langKazakh = 48;   { Cyrillic script}
	langAzerbaijani = 49;   { Azerbaijani in Cyrillic script}
	langAzerbaijanAr = 50;   { Azerbaijani in Arabic script}
	langArmenian = 51;   { smArmenian script}
	langGeorgian = 52;   { smGeorgian script}
	langMoldavian = 53;   { smCyrillic script}
	langKirghiz = 54;   { Cyrillic script}
	langTajiki = 55;   { Cyrillic script}
	langTurkmen = 56;   { Cyrillic script}
	langMongolian = 57;   { Mongolian in smMongolian script}
	langMongolianCyr = 58;   { Mongolian in Cyrillic script}
	langPashto = 59;   { Arabic script}
	langKurdish = 60;   { smArabic script}
	langKashmiri = 61;   { Arabic script}
	langSindhi = 62;   { Arabic script}
	langTibetan = 63;   { smTibetan script}
	langNepali = 64;   { smDevanagari script}
	langSanskrit = 65;   { smDevanagari script}
	langMarathi = 66;   { smDevanagari script}
	langBengali = 67;   { smBengali script}
	langAssamese = 68;   { smBengali script}
	langGujarati = 69;   { smGujarati script}
	langPunjabi = 70;    { smGurmukhi script}

const
	langOriya = 71;   { smOriya script}
	langMalayalam = 72;   { smMalayalam script}
	langKannada = 73;   { smKannada script}
	langTamil = 74;   { smTamil script}
	langTelugu = 75;   { smTelugu script}
	langSinhalese = 76;   { smSinhalese script}
	langBurmese = 77;   { smBurmese script}
	langKhmer = 78;   { smKhmer script}
	langLao = 79;   { smLao script}
	langVietnamese = 80;   { smVietnamese script}
	langIndonesian = 81;   { smRoman script}
	langTagalog = 82;   { Roman script}
	langMalayRoman = 83;   { Malay in smRoman script}
	langMalayArabic = 84;   { Malay in Arabic script}
	langAmharic = 85;   { smEthiopic script}
	langTigrinya = 86;   { smEthiopic script}
	langOromo = 87;   { smEthiopic script}
	langSomali = 88;   { smRoman script}
	langSwahili = 89;   { smRoman script}
	langKinyarwanda = 90;   { smRoman script}
	langRuanda = 90;   { synonym for langKinyarwanda}
	langRundi = 91;   { smRoman script}
	langNyanja = 92;   { smRoman script}
	langChewa = 92;   { synonym for langNyanja}
	langMalagasy = 93;   { smRoman script}
	langEsperanto = 94;    { Roman script}

const
	langWelsh = 128;  { modified smRoman/Celtic script}
	langBasque = 129;  { smRoman script}
	langCatalan = 130;  { smRoman script}
	langLatin = 131;  { smRoman script}
	langQuechua = 132;  { smRoman script}
	langGuarani = 133;  { smRoman script}
	langAymara = 134;  { smRoman script}
	langTatar = 135;  { Cyrillic script}
	langUighur = 136;  { Arabic script}
	langDzongkha = 137;  { (lang of Bhutan) smTibetan script}
	langJavaneseRom = 138;  { Javanese in smRoman script}
	langSundaneseRom = 139;  { Sundanese in smRoman script}
	langGalician = 140;  { smRoman script}
	langAfrikaans = 141;   { smRoman script                                   }

const
	langBreton = 142;  { smRoman or modified smRoman/Celtic script                 }
	langInuktitut = 143;  { Inuit script using smEthiopic script code                 }
	langScottishGaelic = 144;  { smRoman or modified smRoman/Celtic script                 }
	langManxGaelic = 145;  { smRoman or modified smRoman/Celtic script                 }
	langIrishGaelicScript = 146;  { modified smRoman/Gaelic script (using dot above)               }
	langTongan = 147;  { smRoman script                                   }
	langGreekAncient = 148;  { Classical Greek, polytonic orthography                    }
	langGreenlandic = 149;  { smRoman script                                   }
	langAzerbaijanRoman = 150;  { Azerbaijani in Roman script                             }
	langNynorsk = 151;   { Norwegian Nyorsk in smRoman}

const
	langUnspecified = 32767; { Special code for use in resources (such as 'itlm')           }

{
   Obsolete language code names (kept for backward compatibility):
   Misspelled, ambiguous, misleading, considered pejorative, archaic, etc.
}
const
	langPortugese = 8;    { Use langPortuguese}
	langMalta = 16;   { Use langMaltese}
	langYugoslavian = 18;   { (use langCroatian, langSerbian, etc.)}
	langChinese = 19;   { (use langTradChinese or langSimpChinese)}
	langLettish = 28;   { Use langLatvian                                     }
	langLapponian = 29;   { Use langSami}
	langLappish = 29;   { Use langSami}
	langSaamisk = 29;   { Use langSami                                    }
	langFaeroese = 30;   { Use langFaroese                                     }
	langIrish = 35;   { Use langIrishGaelic                                  }
	langGalla = 87;   { Use langOromo                                 }
	langAfricaans = 141;  { Use langAfrikaans                                }
	langGreekPoly = 148;   { Use langGreekAncient}

{
   Region codes:
   These typically specify a combination of a language code and a particular region.
   Some of these numeric values are reserved just for extra resource IDs associated
   with certain regions; these are not actual region codes, and are noted in the comments.
   Not all of the region codes are currently supported by Apple software.
   When relevant, the following list also provides:
   - The Apple part number (P/N) code for software localized for the specified region.
   - The two-letter ISO language and country codes (from ISO 639 and ISO 3166). The
     language code (lowercase) is first, then '_', then the country code (uppercase).
}

const
{ P/N    ISO    codes  comments}
	verUS = 0;    {       en_US}
	verFrance = 1;    { F  fr_FR}
	verBritain = 2;    { B  en_GB}
	verGermany = 3;    { D  de_DE}
	verItaly = 4;    { T  it_IT}
	verNetherlands = 5;    { N  nl_NL}
	verFlemish = 6;    { FN nl_BE     Flemish (Dutch) for Belgium                 }
	verSweden = 7;    { S  sv_SE}
	verSpain = 8;    { E  es_ES       Spanish for Spain}
	verDenmark = 9;    { DK da_DK}
	verPortugal = 10;   { PO pt_PT     Portuguese for Portugal}
	verFrCanada = 11;   { C  fr_CA       French for Canada}
	verNorway = 12;   { H  nb_NO       Bokmål}
	verIsrael = 13;   { HB he_IL     Hebrew}
	verJapan = 14;   { J  ja_JP}
	verAustralia = 15;   { X  en_AU       English for Australia}
	verArabic = 16;   { AB ar       Arabic for N Africa, Arabian peninsula, Levant}
	verFinland = 17;   { K  fi_FI}
	verFrSwiss = 18;   { SF fr_CH     French Swiss}
	verGrSwiss = 19;   { SD de_CH     German Swiss}
	verGreece = 20;   { GR el_GR     Monotonic Greek (modern)}
	verIceland = 21;   { IS is_IS}
	verMalta = 22;   { MA mt_MT}
	verCyprus = 23;   { CY el_CY}
	verTurkey = 24;   { TU tr_TR}
	verYugoCroatian = 25;    { YU hr_HR     Croatian for Yugoslavia; now use verCroatia (68)}

const
	verNetherlandsComma = 26;   {              ID for KCHR resource - Dutch}
	verFlemishPoint = 27;   {              ID for KCHR resource - Belgium}
	verCanadaComma = 28;   {              ID for KCHR resource - Canadian ISO}
	verCanadaPoint = 29;   {              ID for KCHR resource - Canadian; now unused}
	vervariantPortugal = 30;   {              ID for resource; now unused}
	vervariantNorway = 31;   {              ID for resource; now unused}
	vervariantDenmark = 32;    {              ID for KCHR resource - Danish Mac Plus}

const
	verIndiaHindi = 33;   {       hi_IN     Hindi for India}
	verPakistanUrdu = 34;   { UR ur_PK     Urdu for Pakistan                        }
	verTurkishModified = 35;   {       tr_TR}
	verItalianSwiss = 36;   { ST it_CH     Italian Swiss}
	verInternational = 37;   { Z  en-ascii English for international use; ASCII chars only     }
                                        {              38 is unassigned}
	verRomania = 39;   { RO ro_RO}
	verGreekAncient = 40;   {       grc      Ancient Greek, polytonic orthography          }
	verLithuania = 41;   { LT lt_LT}
	verPoland = 42;   { PL pl_PL}
	verHungary = 43;   { MG hu_HU}
	verEstonia = 44;   { EE et_EE}
	verLatvia = 45;   { LV lv_LV}
	verSami = 46;   {       se                                        }
	verFaroeIsl = 47;   { FA fo_FO                                    }
	verIran = 48;   { PS fa_IR     Persian/Farsi}
	verRussia = 49;   { RS ru_RU     Russian}
	verIreland = 50;   { GA ga_IE     Irish Gaelic for Ireland (without dot above)     }
	verKorea = 51;   { KH ko_KR}
	verChina = 52;   { CH zh_CN     Simplified Chinese}
	verTaiwan = 53;   { TA zh_TW     Traditional Chinese}
	verThailand = 54;   { TH th_TH}
	verScriptGeneric = 55;   { SS          Generic script system (no language or script)       }
	verCzech = 56;   { CZ cs_CZ}
	verSlovak = 57;   { SL sk_SK}
	verEastAsiaGeneric = 58;   { FE          Generic East Asia system (no language or script) }
	verMagyar = 59;   {       hu_HU     Unused; see verHungary}
	verBengali = 60;   {       bn         Bangladesh or India}
	verBelarus = 61;    { BY be_BY}

const
	verUkraine = 62;   { UA uk_UA}
                                        {              63 is unassigned}
	verGreeceAlt = 64;   {       el_GR     unused                              }
	verSerbian = 65;   { SR sr_CS                                    }
	verSlovenian = 66;   { SV sl_SI                                    }
	verMacedonian = 67;   { MD mk_MK                                    }
	verCroatia = 68;   { CR hr_HR}
                                        {              69 is unassigned}
	verGermanReformed = 70;   {       de-1996     Reformed orthography (used formerly unassigned 70)   }
	verBrazil = 71;   { BR pt_BR     Portuguese for Brazil}
	verBulgaria = 72;   { BG bg_BG}
	verCatalonia = 73;   { CA ca_ES     Catalan for Spain}
	verMultilingual = 74;   { ZM mul        (no language or script)}
	verScottishGaelic = 75;   { GD gd}
	verManxGaelic = 76;   { GV gv       Isle of Man}
	verBreton = 77;   { BZ br}
	verNunavut = 78;   { IU iu_CA     Inuktitut for Canada}
	verWelsh = 79;   { CU cy}
                                        {              80 is ID for KCHR resource - Canadian CSA}
	verIrishGaelicScript = 81;   { GS ga-Latg_IE  Irish Gaelic for Ireland (using dot above)}
	verEngCanada = 82;   { V  en_CA       English for Canada}
	verBhutan = 83;   { BH dz_BT     Dzongkha for Bhutan}
	verArmenian = 84;   { HY hy_AM}
	verGeorgian = 85;   { KR ka_GE}
	verSpLatinAmerica = 86;   { LA es_XL     Spanish for Latin America (private ISO 3166 code)}
                                        {              87 is ID for KCHR resource - Spanish ISO}
	verTonga = 88;   { TS to_TO}
                                        {              89 is ID for KCHR resource - Polish Modified}
                                        {              90 is ID for KCHR resource - Catalan ISO}
	verFrenchUniversal = 91;   {       fr         French generic}
	verAustria = 92;   { AU de_AT     German for Austria}
                                        { Y          93 is unused alternate for verSpLatinAmerica}
	verGujarati = 94;   {       gu_IN}
	verPunjabi = 95;   {       pa         Pakistan or India}
	verIndiaUrdu = 96;   {       ur_IN     Urdu for India}
	verVietnam = 97;    {       vi_VN}

const
	verFrBelgium = 98;   { BF fr_BE     French for Belgium                       }
	verUzbek = 99;   { BD uz_UZ                                    }
	verSingapore = 100;  { SG en_SG     (Assume English, not Chinese or Malay)                                 }
	verNynorsk = 101;  { NY nn_NO     Norwegian Nynorsk                        }
	verAfrikaans = 102;  { AK af_ZA                                    }
	verEsperanto = 103;  {       eo                                        }
	verMarathi = 104;  {       mr_IN                                      }
	verTibetan = 105;  {       bo                                        }
	verNepal = 106;  {       ne_NP                                      }
	verGreenland = 107;  {       kl                                        }
	verIrelandEnglish = 108;   {       en_IE     English for Ireland, with Euro for currency}

{
   Other extra resource IDs assigned in the same number space:
    179 is ID for KCHR & itl_ resources - Cornish
    581 is ID for KCHR resource - Irish Gaelic script alternate
    582 is ID for KCHR resource - Ogham
    779 is ID for KCHR resource - Welsh alternate
   1111 is ID for KCHR resource - French numeric
}

{
   Obsolete region code names (kept for backward compatibility):
   Misspelled or alternate form, ambiguous, misleading, considered pejorative, archaic, etc.
}
const
	verFrBelgiumLux = 6;    { Incorrect; 6 is Flemish, not French, for Belgium; use verFlemish     }
	verBelgiumLux = 6;    { Use verFlemish}
	verArabia = 16;   { Use verArabic}
	verYugoslavia = 25;   { Use verYugoCroatian (same number, now unused), or newer verCroatia}
	verBelgiumLuxPoint = 27;   { Use verFlemishPoint}
	verIndia = 33;   { Use verIndiaHindi}
	verPakistan = 34;   { Use verPakistanUrdu                                  }
	verRumania = 39;   { Alternate for verRomania                              }
	verGreecePoly = 40;   { Use verGreekAncient                                  }
	verLapland = 46;   { Use verSami                                       }
	verFaeroeIsl = 47;   { Use verFaroeIsl                                     }
	verGenericFE = 58;   { Use verEastAsiaGeneric                              }
	verFarEastGeneric = 58;   { Use verEastAsiaGeneric}
	verByeloRussian = 61;   { Alternate for verBelarus                              }
	verUkrania = 62;   { Use verUkraine}
	verAlternateGr = 64;   { Use verGreeceAlt                                   }
	verSerbia = 65;   { Alternate for verSerbian                              }
	verSlovenia = 66;   { Alternate for verSlovenian                            }
	verMacedonia = 67;   { Alternate for verMacedonian                             }
	verBrittany = 77;   { Alternate for verBreton                              }
	verWales = 79;   { Alternate for verWelsh                              }
	verArmenia = 84;   { Alternate for verArmenian                           }
	verGeorgia = 85;   { Alternate for verGeorgian                           }
	verAustriaGerman = 92;   { Use verAustria                                   }
	verTibet = 105;   { Use verTibetan                                   }

const
	minCountry = verUS;
	maxCountry = verIrelandEnglish;

const
{ Calendar Codes }
	calGregorian = 0;
	calArabicCivil = 1;
	calArabicLunar = 2;
	calJapanese = 3;
	calJewish = 4;
	calCoptic = 5;
	calPersian = 6;

const
{ Integer Format Codes }
	intWestern = 0;
	intArabic = 1;
	intRoman = 2;
	intJapanese = 3;
	intEuropean = 4;
	intOutputMask = $8000;

const
{ CharByte byte types }
	smSingleByte = 0;
	smFirstByte = -1;
	smLastByte = 1;
	smMiddleByte = 2;

const
{ CharType field masks }
	smcTypeMask = $000F;
	smcReserved = $00F0;
	smcClassMask = $0F00;
	smcOrientationMask = $1000; {two-byte script glyph orientation}
	smcRightMask = $2000;
	smcUpperMask = $4000;
	smcDoubleMask = $8000;

const
{ Basic CharType character types }
	smCharPunct = $0000;
	smCharAscii = $0001;
	smCharEuro = $0007;
	smCharExtAscii = $0007; { More correct synonym for smCharEuro }
                                        { Additional CharType character types for script systems }
	smCharKatakana = $0002; {Japanese Katakana}
	smCharHiragana = $0003; {Japanese Hiragana}
	smCharIdeographic = $0004; {Hanzi, Kanji, Hanja}
	smCharTwoByteGreek = $0005; {2-byte Greek in Far East systems}
	smCharTwoByteRussian = $0006; {2-byte Cyrillic in Far East systems}
	smCharBidirect = $0008; {Arabic/Hebrew}
	smCharContextualLR = $0009; {Contextual left-right: Thai, Indic scripts}
	smCharNonContextualLR = $000A; {Non-contextual left-right: Cyrillic, Greek}
	smCharHangul = $000C; {Korean Hangul}
	smCharJamo = $000D; {Korean Jamo}
	smCharBopomofo = $000E; {Chinese Bopomofo}
	smCharGanaKana = $000F; {Shared for Japanese Hiragana & Katakana}
                                        { old names for some of above, for backward compatibility }
	smCharFISKana = $0002; {Katakana}
	smCharFISGana = $0003; {Hiragana}
	smCharFISIdeo = $0004; {Hanzi, Kanji, Hanja}

const
	smCharFISGreek = $0005; {2-byte Greek in Far East systems}
	smCharFISRussian = $0006; {2-byte Cyrillic in Far East systems}
                                        { CharType classes for punctuation (smCharPunct) }
	smPunctNormal = $0000;
	smPunctNumber = $0100;
	smPunctSymbol = $0200;
	smPunctBlank = $0300; { Additional CharType classes for punctuation in two-byte systems }
	smPunctRepeat = $0400; { repeat marker }
	smPunctGraphic = $0500; { line graphics }
                                        { CharType Katakana and Hiragana classes for two-byte systems }
	smKanaSmall = $0100; {small kana character}
	smKanaHardOK = $0200; {can have dakuten}
	smKanaSoftOK = $0300; {can have dakuten or han-dakuten}
                                        { CharType Ideographic classes for two-byte systems }
	smIdeographicLevel1 = $0000; {level 1 char}
	smIdeographicLevel2 = $0100; {level 2 char}
	smIdeographicUser = $0200; {user char}
                                        { old names for above, for backward compatibility }
	smFISClassLvl1 = $0000; {level 1 char}
	smFISClassLvl2 = $0100; {level 2 char}
	smFISClassUser = $0200; {user char}
                                        { CharType Jamo classes for Korean systems }
	smJamoJaeum = $0000; {simple consonant char}
	smJamoBogJaeum = $0100; {complex consonant char}
	smJamoMoeum = $0200; {simple vowel char}
	smJamoBogMoeum = $0300; {complex vowel char}

const
{ CharType glyph orientation for two-byte systems }
	smCharHorizontal = $0000; { horizontal character form, or for both }
	smCharVertical = $1000; { vertical character form }
                                        { CharType directions }
	smCharLeft = $0000;
	smCharRight = $2000; { CharType case modifers }
	smCharLower = $0000;
	smCharUpper = $4000; { CharType character size modifiers (1 or multiple bytes). }
	smChar1byte = $0000;
	smChar2byte = $8000;

const
{ TransliterateText target types for Roman }
	smTransAscii = 0;    {convert to ASCII}
	smTransNative = 1;    {convert to font script}
	smTransCase = $FE; {convert case for all text}
	smTransSystem = $FF; {convert to system script}
                                        { TransliterateText target types for two-byte scripts }
	smTransAscii1 = 2;    {1-byte Roman}
	smTransAscii2 = 3;    {2-byte Roman}
	smTransKana1 = 4;    {1-byte Japanese Katakana}
	smTransKana2 = 5;     {2-byte Japanese Katakana}

const
	smTransGana2 = 7;    {2-byte Japanese Hiragana (no 1-byte Hiragana)}
	smTransHangul2 = 8;    {2-byte Korean Hangul}
	smTransJamo2 = 9;    {2-byte Korean Jamo}
	smTransBopomofo2 = 10;   {2-byte Chinese Bopomofo}
                                        { TransliterateText target modifiers }
	smTransLower = $4000; {target becomes lowercase}
	smTransUpper = $8000; {target becomes uppercase}
                                        { TransliterateText resource format numbers }
	smTransRuleBaseFormat = 1;    {Rule based trsl resource format }
	smTransHangulFormat = 2;    {Table based Hangul trsl resource format}
                                        { TransliterateText property flags }
	smTransPreDoubleByting = 1;    {Convert all text to double byte before transliteration}
	smTransPreLowerCasing = 2;     {Convert all text to lower case before transliteration}

const
{ TransliterateText source mask - general }
	smMaskAll = -1; {Convert all text}
                                        { TransliterateText source masks }
	smMaskAscii = $00000001; {2^smTransAscii}
	smMaskNative = $00000002; {2^smTransNative}
                                        { TransliterateText source masks for two-byte scripts }
	smMaskAscii1 = $00000004; {2^smTransAscii1}
	smMaskAscii2 = $00000008; {2^smTransAscii2}
	smMaskKana1 = $00000010; {2^smTransKana1}
	smMaskKana2 = $00000020; {2^smTransKana2}
	smMaskGana2 = $00000080; {2^smTransGana2}
	smMaskHangul2 = $00000100; {2^smTransHangul2}
	smMaskJamo2 = $00000200; {2^smTransJamo2}
	smMaskBopomofo2 = $00000400; {2^smTransBopomofo2}

const
{ Special script code values for International Utilities }
	iuSystemScript = -1;   { <obsolete>  system script }
	iuCurrentScript = -2;    { <obsolete>  current script (for font of grafPort) }

const
{ Negative verbs for KeyScript }
	smKeyNextScript = -1;   { Switch to next available script }
	smKeySysScript = -2;   { Switch to the system script }
	smKeySwapScript = -3;   { Switch to previously-used script }
                                        { New for System 7.0: }
	smKeyNextKybd = -4;   { Switch to next keyboard in current keyscript }
	smKeySwapKybd = -5;   { Switch to previously-used keyboard in current keyscript }
	smKeyDisableKybds = -6;   { Disable keyboards not in system or Roman script }
	smKeyEnableKybds = -7;   { Re-enable keyboards for all enabled scripts }
	smKeyToggleInline = -8;   { Toggle inline input for current keyscript }
	smKeyToggleDirection = -9;   { Toggle default line direction (TESysJust) }
	smKeyNextInputMethod = -10;  { Switch to next input method in current keyscript }
	smKeySwapInputMethod = -11;  { Switch to last-used input method in current keyscript }
	smKeyDisableKybdSwitch = -12;  { Disable switching from the current keyboard }
	smKeySetDirLeftRight = -15;  { Set default line dir to left-right, align left }
	smKeySetDirRightLeft = -16;  { Set default line dir to right-left, align right }
	smKeyRoman = -17;   { Set keyscript to Roman. Does nothing if Roman-only system, unlike KeyScript(smRoman) which forces an update to current default Roman keyboard }

const
{ Force keyboard script switching flag and mask for zero and positive KeyScript verbs }
	smKeyForceKeyScriptBit = 7;    { Force keyboard script switching flag }
	smKeyForceKeyScriptMask = 1 shl smKeyForceKeyScriptBit; { its mask }

const
{ Roman script constants }
                                        { The following are here for backward compatibility, but should not be used. }
                                        { This information should be obtained using GetScript. }
	romanSysFond = $3FFF; {system font id number}
	romanAppFond = 3;    {application font id number}
	romanFlags = $0007; {roman settings}
                                        { Script Manager font equates. }
	smFondStart = $4000; {start from 16K}
	smFondEnd = $C000; {past end of range at 48K}
                                        { Miscellaneous font equates. }
	smUprHalfCharSet = $80;  {first char code in top half of std char set}

const
{ Character Set Extensions }
	diaeresisUprY = $D9;
	fraction = $DA;
	intlCurrency = $DB;
	leftSingGuillemet = $DC;
	rightSingGuillemet = $DD;
	fiLigature = $DE;
	flLigature = $DF;
	dblDagger = $E0;
	centeredDot = $E1;
	baseSingQuote = $E2;
	baseDblQuote = $E3;
	perThousand = $E4;
	circumflexUprA = $E5;
	circumflexUprE = $E6;
	acuteUprA = $E7;
	diaeresisUprE = $E8;
	graveUprE = $E9;
	acuteUprI = $EA;
	circumflexUprI = $EB;
	diaeresisUprI = $EC;
	graveUprI = $ED;
	acuteUprO = $EE;
	circumflexUprO = $EF;
	appleLogo = $F0;
	graveUprO = $F1;
	acuteUprU = $F2;
	circumflexUprU = $F3;
	graveUprU = $F4;
	dotlessLwrI = $F5;
	circumflex = $F6;
	tilde = $F7;
	macron = $F8;
	breveMark = $F9;
	overDot = $FA;
	ringMark = $FB;
	cedilla = $FC;
	doubleAcute = $FD;
	ogonek = $FE;
	hachek = $FF;

const
{ ScriptTokenType values }
	tokenIntl = 4;    {the itl resource number of the tokenizer}
	tokenEmpty = -1;    {used internally as an empty flag}

const
	tokenUnknown = 0;    {chars that do not match a defined token type}
	tokenWhite = 1;    {white space}
	tokenLeftLit = 2;    {literal begin}
	tokenRightLit = 3;    {literal end}
	tokenAlpha = 4;    {alphabetic}
	tokenNumeric = 5;    {numeric}
	tokenNewLine = 6;    {new line}
	tokenLeftComment = 7;    {open comment}
	tokenRightComment = 8;    {close comment}
	tokenLiteral = 9;    {literal}
	tokenEscape = 10;   {character escape (e.g. '\' in "\n", "\t")}
	tokenAltNum = 11;   {alternate number (e.g. $B0-B9 in Arabic,Hebrew)}
	tokenRealNum = 12;   {real number}
	tokenAltReal = 13;   {alternate real number}
	tokenReserve1 = 14;   {reserved}
	tokenReserve2 = 15;   {reserved}
	tokenLeftParen = 16;   {open parenthesis}
	tokenRightParen = 17;   {close parenthesis}
	tokenLeftBracket = 18;   {open square bracket}
	tokenRightBracket = 19;    {close square bracket}

const
	tokenLeftCurly = 20;   {open curly bracket}
	tokenRightCurly = 21;   {close curly bracket}
	tokenLeftEnclose = 22;   {open guillemet}
	tokenRightEnclose = 23;   {close guillemet}
	tokenPlus = 24;
	tokenMinus = 25;
	tokenAsterisk = 26;   {times/multiply}
	tokenDivide = 27;
	tokenPlusMinus = 28;   {plus or minus symbol}
	tokenSlash = 29;
	tokenBackSlash = 30;
	tokenLess = 31;   {less than symbol}
	tokenGreat = 32;   {greater than symbol}
	tokenEqual = 33;
	tokenLessEqual2 = 34;   {less than or equal, 2 characters (e.g. <=)}
	tokenLessEqual1 = 35;   {less than or equal, 1 character}
	tokenGreatEqual2 = 36;   {greater than or equal, 2 characters (e.g. >=)}
	tokenGreatEqual1 = 37;   {greater than or equal, 1 character}
	token2Equal = 38;   {double equal (e.g. ==)}
	tokenColonEqual = 39;    {colon equal}

const
	tokenNotEqual = 40;   {not equal, 1 character}
	tokenLessGreat = 41;   {less/greater, Pascal not equal (e.g. <>)}
	tokenExclamEqual = 42;   {exclamation equal, C not equal (e.g. !=)}
	tokenExclam = 43;   {exclamation point}
	tokenTilde = 44;   {centered tilde}
	tokenComma = 45;
	tokenPeriod = 46;
	tokenLeft2Quote = 47;   {open double quote}
	tokenRight2Quote = 48;   {close double quote}
	tokenLeft1Quote = 49;   {open single quote}
	tokenRight1Quote = 50;   {close single quote}
	token2Quote = 51;   {double quote}
	token1Quote = 52;   {single quote}
	tokenSemicolon = 53;
	tokenPercent = 54;
	tokenCaret = 55;
	tokenUnderline = 56;
	tokenAmpersand = 57;
	tokenAtSign = 58;
	tokenBar = 59;    {vertical bar}

const
	tokenQuestion = 60;
	tokenPi = 61;   {lower-case pi}
	tokenRoot = 62;   {square root symbol}
	tokenSigma = 63;   {capital sigma}
	tokenIntegral = 64;   {integral sign}
	tokenMicro = 65;
	tokenCapPi = 66;   {capital pi}
	tokenInfinity = 67;
	tokenColon = 68;
	tokenHash = 69;   {e.g. #}
	tokenDollar = 70;
	tokenNoBreakSpace = 71;   {non-breaking space}
	tokenFraction = 72;
	tokenIntlCurrency = 73;
	tokenLeftSingGuillemet = 74;
	tokenRightSingGuillemet = 75;
	tokenPerThousand = 76;
	tokenEllipsis = 77;
	tokenCenterDot = 78;
	tokenNil = 127;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	delimPad = -2;   { obsolete, misspelled token names kept for backward compatibility }
	tokenTilda = 44;
	tokenCarat = 55;
<<<<<<< HEAD
<<<<<<< HEAD

const
=======

const
	delimPad = -2;   { obsolete, misspelled token names kept for backward compatibility }
	tokenTilda = 44;
	tokenCarat = 55;

const
>>>>>>> graemeg/cpstrnew
{ Table selectors for GetItlTable }
	smWordSelectTable = 0;    { get word select break table from 'itl2' }
	smWordWrapTable = 1;    { get word wrap break table from 'itl2' }
	smNumberPartsTable = 2;    { get default number parts table from 'itl4' }
	smUnTokenTable = 3;    { get unToken table from 'itl4' }
	smWhiteSpaceList = 4;    { get white space list from 'itl4' }
	iuWordSelectTable = 0;    { <obsolete>  get word select break table from 'itl2' }
	iuWordWrapTable = 1;    { <obsolete>  get word wrap break table from 'itl2' }
	iuNumberPartsTable = 2;    { <obsolete>  get default number parts table from 'itl4' }
	iuUnTokenTable = 3;    { <obsolete>  get unToken table from 'itl4' }
	iuWhiteSpaceList = 4;     { <obsolete>  get white space list from 'itl4' }

{ end of stuff moved from Packages.h }
const
	tokenOK = 0;    { TokenResults }
	tokenOverflow = 1;    { TokenResults }
	stringOverflow = 2;    { TokenResults }
	badDelim = 3;    { TokenResults }
	badEnding = 4;    { TokenResults }
	crash = 5;     { TokenResults }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

const
	langOriya = 71;   { smOriya script}
	langMalayalam = 72;   { smMalayalam script}
	langKannada = 73;   { smKannada script}
	langTamil = 74;   { smTamil script}
	langTelugu = 75;   { smTelugu script}
	langSinhalese = 76;   { smSinhalese script}
	langBurmese = 77;   { smBurmese script}
	langKhmer = 78;   { smKhmer script}
	langLao = 79;   { smLao script}
	langVietnamese = 80;   { smVietnamese script}
	langIndonesian = 81;   { smRoman script}
	langTagalog = 82;   { Roman script}
	langMalayRoman = 83;   { Malay in smRoman script}
	langMalayArabic = 84;   { Malay in Arabic script}
	langAmharic = 85;   { smEthiopic script}
	langTigrinya = 86;   { smEthiopic script}
	langOromo = 87;   { smEthiopic script}
	langSomali = 88;   { smRoman script}
	langSwahili = 89;   { smRoman script}
	langKinyarwanda = 90;   { smRoman script}
	langRuanda = 90;   { synonym for langKinyarwanda}
	langRundi = 91;   { smRoman script}
	langNyanja = 92;   { smRoman script}
	langChewa = 92;   { synonym for langNyanja}
	langMalagasy = 93;   { smRoman script}
	langEsperanto = 94;    { Roman script}

const
	langWelsh = 128;  { modified smRoman/Celtic script}
	langBasque = 129;  { smRoman script}
	langCatalan = 130;  { smRoman script}
	langLatin = 131;  { smRoman script}
	langQuechua = 132;  { smRoman script}
	langGuarani = 133;  { smRoman script}
	langAymara = 134;  { smRoman script}
	langTatar = 135;  { Cyrillic script}
	langUighur = 136;  { Arabic script}
	langDzongkha = 137;  { (lang of Bhutan) smTibetan script}
	langJavaneseRom = 138;  { Javanese in smRoman script}
	langSundaneseRom = 139;  { Sundanese in smRoman script}
	langGalician = 140;  { smRoman script}
	langAfrikaans = 141;   { smRoman script                                   }

const
	langBreton = 142;  { smRoman or modified smRoman/Celtic script                 }
	langInuktitut = 143;  { Inuit script using smEthiopic script code                 }
	langScottishGaelic = 144;  { smRoman or modified smRoman/Celtic script                 }
	langManxGaelic = 145;  { smRoman or modified smRoman/Celtic script                 }
	langIrishGaelicScript = 146;  { modified smRoman/Gaelic script (using dot above)               }
	langTongan = 147;  { smRoman script                                   }
	langGreekAncient = 148;  { Classical Greek, polytonic orthography                    }
	langGreenlandic = 149;  { smRoman script                                   }
	langAzerbaijanRoman = 150;  { Azerbaijani in Roman script                             }
	langNynorsk = 151;   { Norwegian Nyorsk in smRoman}

const
	langUnspecified = 32767; { Special code for use in resources (such as 'itlm')           }

{
   Obsolete language code names (kept for backward compatibility):
   Misspelled, ambiguous, misleading, considered pejorative, archaic, etc.
}
const
	langPortugese = 8;    { Use langPortuguese}
	langMalta = 16;   { Use langMaltese}
	langYugoslavian = 18;   { (use langCroatian, langSerbian, etc.)}
	langChinese = 19;   { (use langTradChinese or langSimpChinese)}
	langLettish = 28;   { Use langLatvian                                     }
	langLapponian = 29;   { Use langSami}
	langLappish = 29;   { Use langSami}
	langSaamisk = 29;   { Use langSami                                    }
	langFaeroese = 30;   { Use langFaroese                                     }
	langIrish = 35;   { Use langIrishGaelic                                  }
	langGalla = 87;   { Use langOromo                                 }
	langAfricaans = 141;  { Use langAfrikaans                                }
	langGreekPoly = 148;   { Use langGreekAncient}

{
   Region codes:
   These typically specify a combination of a language code and a particular region.
   Some of these numeric values are reserved just for extra resource IDs associated
   with certain regions; these are not actual region codes, and are noted in the comments.
   Not all of the region codes are currently supported by Apple software.
   When relevant, the following list also provides:
   - The Apple part number (P/N) code for software localized for the specified region.
   - The two-letter ISO language and country codes (from ISO 639 and ISO 3166). The
     language code (lowercase) is first, then '_', then the country code (uppercase).
}

const
{ P/N    ISO    codes  comments}
	verUS = 0;    {       en_US}
	verFrance = 1;    { F  fr_FR}
	verBritain = 2;    { B  en_GB}
	verGermany = 3;    { D  de_DE}
	verItaly = 4;    { T  it_IT}
	verNetherlands = 5;    { N  nl_NL}
	verFlemish = 6;    { FN nl_BE     Flemish (Dutch) for Belgium                 }
	verSweden = 7;    { S  sv_SE}
	verSpain = 8;    { E  es_ES       Spanish for Spain}
	verDenmark = 9;    { DK da_DK}
	verPortugal = 10;   { PO pt_PT     Portuguese for Portugal}
	verFrCanada = 11;   { C  fr_CA       French for Canada}
	verNorway = 12;   { H  nb_NO       Bokmål}
	verIsrael = 13;   { HB he_IL     Hebrew}
	verJapan = 14;   { J  ja_JP}
	verAustralia = 15;   { X  en_AU       English for Australia}
	verArabic = 16;   { AB ar       Arabic for N Africa, Arabian peninsula, Levant}
	verFinland = 17;   { K  fi_FI}
	verFrSwiss = 18;   { SF fr_CH     French Swiss}
	verGrSwiss = 19;   { SD de_CH     German Swiss}
	verGreece = 20;   { GR el_GR     Monotonic Greek (modern)}
	verIceland = 21;   { IS is_IS}
	verMalta = 22;   { MA mt_MT}
	verCyprus = 23;   { CY el_CY}
	verTurkey = 24;   { TU tr_TR}
	verYugoCroatian = 25;    { YU hr_HR     Croatian for Yugoslavia; now use verCroatia (68)}

const
	verNetherlandsComma = 26;   {              ID for KCHR resource - Dutch}
	verFlemishPoint = 27;   {              ID for KCHR resource - Belgium}
	verCanadaComma = 28;   {              ID for KCHR resource - Canadian ISO}
	verCanadaPoint = 29;   {              ID for KCHR resource - Canadian; now unused}
	vervariantPortugal = 30;   {              ID for resource; now unused}
	vervariantNorway = 31;   {              ID for resource; now unused}
	vervariantDenmark = 32;    {              ID for KCHR resource - Danish Mac Plus}

const
	verIndiaHindi = 33;   {       hi_IN     Hindi for India}
	verPakistanUrdu = 34;   { UR ur_PK     Urdu for Pakistan                        }
	verTurkishModified = 35;   {       tr_TR}
	verItalianSwiss = 36;   { ST it_CH     Italian Swiss}
	verInternational = 37;   { Z  en-ascii English for international use; ASCII chars only     }
                                        {              38 is unassigned}
	verRomania = 39;   { RO ro_RO}
	verGreekAncient = 40;   {       grc      Ancient Greek, polytonic orthography          }
	verLithuania = 41;   { LT lt_LT}
	verPoland = 42;   { PL pl_PL}
	verHungary = 43;   { MG hu_HU}
	verEstonia = 44;   { EE et_EE}
	verLatvia = 45;   { LV lv_LV}
	verSami = 46;   {       se                                        }
	verFaroeIsl = 47;   { FA fo_FO                                    }
	verIran = 48;   { PS fa_IR     Persian/Farsi}
	verRussia = 49;   { RS ru_RU     Russian}
	verIreland = 50;   { GA ga_IE     Irish Gaelic for Ireland (without dot above)     }
	verKorea = 51;   { KH ko_KR}
	verChina = 52;   { CH zh_CN     Simplified Chinese}
	verTaiwan = 53;   { TA zh_TW     Traditional Chinese}
	verThailand = 54;   { TH th_TH}
	verScriptGeneric = 55;   { SS          Generic script system (no language or script)       }
	verCzech = 56;   { CZ cs_CZ}
	verSlovak = 57;   { SL sk_SK}
	verEastAsiaGeneric = 58;   { FE          Generic East Asia system (no language or script) }
	verMagyar = 59;   {       hu_HU     Unused; see verHungary}
	verBengali = 60;   {       bn         Bangladesh or India}
	verBelarus = 61;    { BY be_BY}

const
	verUkraine = 62;   { UA uk_UA}
                                        {              63 is unassigned}
	verGreeceAlt = 64;   {       el_GR     unused                              }
	verSerbian = 65;   { SR sr_CS                                    }
	verSlovenian = 66;   { SV sl_SI                                    }
	verMacedonian = 67;   { MD mk_MK                                    }
	verCroatia = 68;   { CR hr_HR}
                                        {              69 is unassigned}
	verGermanReformed = 70;   {       de-1996     Reformed orthography (used formerly unassigned 70)   }
	verBrazil = 71;   { BR pt_BR     Portuguese for Brazil}
	verBulgaria = 72;   { BG bg_BG}
	verCatalonia = 73;   { CA ca_ES     Catalan for Spain}
	verMultilingual = 74;   { ZM mul        (no language or script)}
	verScottishGaelic = 75;   { GD gd}
	verManxGaelic = 76;   { GV gv       Isle of Man}
	verBreton = 77;   { BZ br}
	verNunavut = 78;   { IU iu_CA     Inuktitut for Canada}
	verWelsh = 79;   { CU cy}
                                        {              80 is ID for KCHR resource - Canadian CSA}
	verIrishGaelicScript = 81;   { GS ga-Latg_IE  Irish Gaelic for Ireland (using dot above)}
	verEngCanada = 82;   { V  en_CA       English for Canada}
	verBhutan = 83;   { BH dz_BT     Dzongkha for Bhutan}
	verArmenian = 84;   { HY hy_AM}
	verGeorgian = 85;   { KR ka_GE}
	verSpLatinAmerica = 86;   { LA es_XL     Spanish for Latin America (private ISO 3166 code)}
                                        {              87 is ID for KCHR resource - Spanish ISO}
	verTonga = 88;   { TS to_TO}
                                        {              89 is ID for KCHR resource - Polish Modified}
                                        {              90 is ID for KCHR resource - Catalan ISO}
	verFrenchUniversal = 91;   {       fr         French generic}
	verAustria = 92;   { AU de_AT     German for Austria}
                                        { Y          93 is unused alternate for verSpLatinAmerica}
	verGujarati = 94;   {       gu_IN}
	verPunjabi = 95;   {       pa         Pakistan or India}
	verIndiaUrdu = 96;   {       ur_IN     Urdu for India}
	verVietnam = 97;    {       vi_VN}

const
	verFrBelgium = 98;   { BF fr_BE     French for Belgium                       }
	verUzbek = 99;   { BD uz_UZ                                    }
	verSingapore = 100;  { SG en_SG     (Assume English, not Chinese or Malay)                                 }
	verNynorsk = 101;  { NY nn_NO     Norwegian Nynorsk                        }
	verAfrikaans = 102;  { AK af_ZA                                    }
	verEsperanto = 103;  {       eo                                        }
	verMarathi = 104;  {       mr_IN                                      }
	verTibetan = 105;  {       bo                                        }
	verNepal = 106;  {       ne_NP                                      }
	verGreenland = 107;  {       kl                                        }
	verIrelandEnglish = 108;   {       en_IE     English for Ireland, with Euro for currency}
=======
=======

const
{ Table selectors for GetItlTable }
	smWordSelectTable = 0;    { get word select break table from 'itl2' }
	smWordWrapTable = 1;    { get word wrap break table from 'itl2' }
	smNumberPartsTable = 2;    { get default number parts table from 'itl4' }
	smUnTokenTable = 3;    { get unToken table from 'itl4' }
	smWhiteSpaceList = 4;    { get white space list from 'itl4' }
	iuWordSelectTable = 0;    { <obsolete>  get word select break table from 'itl2' }
	iuWordWrapTable = 1;    { <obsolete>  get word wrap break table from 'itl2' }
	iuNumberPartsTable = 2;    { <obsolete>  get default number parts table from 'itl4' }
	iuUnTokenTable = 3;    { <obsolete>  get unToken table from 'itl4' }
	iuWhiteSpaceList = 4;     { <obsolete>  get white space list from 'itl4' }

{ end of stuff moved from Packages.h }
const
	tokenOK = 0;    { TokenResults }
	tokenOverflow = 1;    { TokenResults }
	stringOverflow = 2;    { TokenResults }
	badDelim = 3;    { TokenResults }
	badEnding = 4;    { TokenResults }
	crash = 5;     { TokenResults }
>>>>>>> origin/cpstrnew

const
{ Table selectors for GetItlTable }
	smWordSelectTable = 0;    { get word select break table from 'itl2' }
	smWordWrapTable = 1;    { get word wrap break table from 'itl2' }
	smNumberPartsTable = 2;    { get default number parts table from 'itl4' }
	smUnTokenTable = 3;    { get unToken table from 'itl4' }
	smWhiteSpaceList = 4;    { get white space list from 'itl4' }
	iuWordSelectTable = 0;    { <obsolete>  get word select break table from 'itl2' }
	iuWordWrapTable = 1;    { <obsolete>  get word wrap break table from 'itl2' }
	iuNumberPartsTable = 2;    { <obsolete>  get default number parts table from 'itl4' }
	iuUnTokenTable = 3;    { <obsolete>  get unToken table from 'itl4' }
	iuWhiteSpaceList = 4;     { <obsolete>  get white space list from 'itl4' }

{ end of stuff moved from Packages.h }
const
	tokenOK = 0;    { TokenResults }
	tokenOverflow = 1;    { TokenResults }
	stringOverflow = 2;    { TokenResults }
	badDelim = 3;    { TokenResults }
	badEnding = 4;    { TokenResults }
	crash = 5;     { TokenResults }
>>>>>>> graemeg/cpstrnew

{
   Other extra resource IDs assigned in the same number space:
    179 is ID for KCHR & itl_ resources - Cornish
    581 is ID for KCHR resource - Irish Gaelic script alternate
    582 is ID for KCHR resource - Ogham
    779 is ID for KCHR resource - Welsh alternate
   1111 is ID for KCHR resource - French numeric
}

{
   Obsolete region code names (kept for backward compatibility):
   Misspelled or alternate form, ambiguous, misleading, considered pejorative, archaic, etc.
}
const
	verFrBelgiumLux = 6;    { Incorrect; 6 is Flemish, not French, for Belgium; use verFlemish     }
	verBelgiumLux = 6;    { Use verFlemish}
	verArabia = 16;   { Use verArabic}
	verYugoslavia = 25;   { Use verYugoCroatian (same number, now unused), or newer verCroatia}
	verBelgiumLuxPoint = 27;   { Use verFlemishPoint}
	verIndia = 33;   { Use verIndiaHindi}
	verPakistan = 34;   { Use verPakistanUrdu                                  }
	verRumania = 39;   { Alternate for verRomania                              }
	verGreecePoly = 40;   { Use verGreekAncient                                  }
	verLapland = 46;   { Use verSami                                       }
	verFaeroeIsl = 47;   { Use verFaroeIsl                                     }
	verGenericFE = 58;   { Use verEastAsiaGeneric                              }
	verFarEastGeneric = 58;   { Use verEastAsiaGeneric}
	verByeloRussian = 61;   { Alternate for verBelarus                              }
	verUkrania = 62;   { Use verUkraine}
	verAlternateGr = 64;   { Use verGreeceAlt                                   }
	verSerbia = 65;   { Alternate for verSerbian                              }
	verSlovenia = 66;   { Alternate for verSlovenian                            }
	verMacedonia = 67;   { Alternate for verMacedonian                             }
	verBrittany = 77;   { Alternate for verBreton                              }
	verWales = 79;   { Alternate for verWelsh                              }
	verArmenia = 84;   { Alternate for verArmenian                           }
	verGeorgia = 85;   { Alternate for verGeorgian                           }
	verAustriaGerman = 92;   { Use verAustria                                   }
	verTibet = 105;   { Use verTibetan                                   }

const
	minCountry = verUS;
	maxCountry = verIrelandEnglish;

const
{ Calendar Codes }
	calGregorian = 0;
	calArabicCivil = 1;
	calArabicLunar = 2;
	calJapanese = 3;
	calJewish = 4;
	calCoptic = 5;
	calPersian = 6;

const
{ Integer Format Codes }
	intWestern = 0;
	intArabic = 1;
	intRoman = 2;
	intJapanese = 3;
	intEuropean = 4;
	intOutputMask = $8000;

const
{ CharByte byte types }
	smSingleByte = 0;
	smFirstByte = -1;
	smLastByte = 1;
	smMiddleByte = 2;
=======
>>>>>>> graemeg/cpstrnew

const
{ CharType field masks }
	smcTypeMask = $000F;
	smcReserved = $00F0;
	smcClassMask = $0F00;
	smcOrientationMask = $1000; {two-byte script glyph orientation}
	smcRightMask = $2000;
	smcUpperMask = $4000;
	smcDoubleMask = $8000;

const
{ Basic CharType character types }
	smCharPunct = $0000;
	smCharAscii = $0001;
	smCharEuro = $0007;
	smCharExtAscii = $0007; { More correct synonym for smCharEuro }
                                        { Additional CharType character types for script systems }
	smCharKatakana = $0002; {Japanese Katakana}
	smCharHiragana = $0003; {Japanese Hiragana}
	smCharIdeographic = $0004; {Hanzi, Kanji, Hanja}
	smCharTwoByteGreek = $0005; {2-byte Greek in Far East systems}
	smCharTwoByteRussian = $0006; {2-byte Cyrillic in Far East systems}
	smCharBidirect = $0008; {Arabic/Hebrew}
	smCharContextualLR = $0009; {Contextual left-right: Thai, Indic scripts}
	smCharNonContextualLR = $000A; {Non-contextual left-right: Cyrillic, Greek}
	smCharHangul = $000C; {Korean Hangul}
	smCharJamo = $000D; {Korean Jamo}
	smCharBopomofo = $000E; {Chinese Bopomofo}
	smCharGanaKana = $000F; {Shared for Japanese Hiragana & Katakana}
                                        { old names for some of above, for backward compatibility }
	smCharFISKana = $0002; {Katakana}
	smCharFISGana = $0003; {Hiragana}
	smCharFISIdeo = $0004; {Hanzi, Kanji, Hanja}

const
	smCharFISGreek = $0005; {2-byte Greek in Far East systems}
	smCharFISRussian = $0006; {2-byte Cyrillic in Far East systems}
                                        { CharType classes for punctuation (smCharPunct) }
	smPunctNormal = $0000;
	smPunctNumber = $0100;
	smPunctSymbol = $0200;
	smPunctBlank = $0300; { Additional CharType classes for punctuation in two-byte systems }
	smPunctRepeat = $0400; { repeat marker }
	smPunctGraphic = $0500; { line graphics }
                                        { CharType Katakana and Hiragana classes for two-byte systems }
	smKanaSmall = $0100; {small kana character}
	smKanaHardOK = $0200; {can have dakuten}
	smKanaSoftOK = $0300; {can have dakuten or han-dakuten}
                                        { CharType Ideographic classes for two-byte systems }
	smIdeographicLevel1 = $0000; {level 1 char}
	smIdeographicLevel2 = $0100; {level 2 char}
	smIdeographicUser = $0200; {user char}
                                        { old names for above, for backward compatibility }
	smFISClassLvl1 = $0000; {level 1 char}
	smFISClassLvl2 = $0100; {level 2 char}
	smFISClassUser = $0200; {user char}
                                        { CharType Jamo classes for Korean systems }
	smJamoJaeum = $0000; {simple consonant char}
	smJamoBogJaeum = $0100; {complex consonant char}
	smJamoMoeum = $0200; {simple vowel char}
	smJamoBogMoeum = $0300; {complex vowel char}

const
{ CharType glyph orientation for two-byte systems }
	smCharHorizontal = $0000; { horizontal character form, or for both }
	smCharVertical = $1000; { vertical character form }
                                        { CharType directions }
	smCharLeft = $0000;
	smCharRight = $2000; { CharType case modifers }
	smCharLower = $0000;
	smCharUpper = $4000; { CharType character size modifiers (1 or multiple bytes). }
	smChar1byte = $0000;
	smChar2byte = $8000;

const
{ TransliterateText target types for Roman }
	smTransAscii = 0;    {convert to ASCII}
	smTransNative = 1;    {convert to font script}
	smTransCase = $FE; {convert case for all text}
	smTransSystem = $FF; {convert to system script}
                                        { TransliterateText target types for two-byte scripts }
	smTransAscii1 = 2;    {1-byte Roman}
	smTransAscii2 = 3;    {2-byte Roman}
	smTransKana1 = 4;    {1-byte Japanese Katakana}
	smTransKana2 = 5;     {2-byte Japanese Katakana}

const
	smTransGana2 = 7;    {2-byte Japanese Hiragana (no 1-byte Hiragana)}
	smTransHangul2 = 8;    {2-byte Korean Hangul}
	smTransJamo2 = 9;    {2-byte Korean Jamo}
	smTransBopomofo2 = 10;   {2-byte Chinese Bopomofo}
                                        { TransliterateText target modifiers }
	smTransLower = $4000; {target becomes lowercase}
	smTransUpper = $8000; {target becomes uppercase}
                                        { TransliterateText resource format numbers }
	smTransRuleBaseFormat = 1;    {Rule based trsl resource format }
	smTransHangulFormat = 2;    {Table based Hangul trsl resource format}
                                        { TransliterateText property flags }
	smTransPreDoubleByting = 1;    {Convert all text to double byte before transliteration}
	smTransPreLowerCasing = 2;     {Convert all text to lower case before transliteration}

const
{ TransliterateText source mask - general }
	smMaskAll = -1; {Convert all text}
                                        { TransliterateText source masks }
	smMaskAscii = $00000001; {2^smTransAscii}
	smMaskNative = $00000002; {2^smTransNative}
                                        { TransliterateText source masks for two-byte scripts }
	smMaskAscii1 = $00000004; {2^smTransAscii1}
	smMaskAscii2 = $00000008; {2^smTransAscii2}
	smMaskKana1 = $00000010; {2^smTransKana1}
	smMaskKana2 = $00000020; {2^smTransKana2}
	smMaskGana2 = $00000080; {2^smTransGana2}
	smMaskHangul2 = $00000100; {2^smTransHangul2}
	smMaskJamo2 = $00000200; {2^smTransJamo2}
	smMaskBopomofo2 = $00000400; {2^smTransBopomofo2}

const
{ Special script code values for International Utilities }
	iuSystemScript = -1;   { <obsolete>  system script }
	iuCurrentScript = -2;    { <obsolete>  current script (for font of grafPort) }

const
{ Negative verbs for KeyScript }
	smKeyNextScript = -1;   { Switch to next available script }
	smKeySysScript = -2;   { Switch to the system script }
	smKeySwapScript = -3;   { Switch to previously-used script }
                                        { New for System 7.0: }
	smKeyNextKybd = -4;   { Switch to next keyboard in current keyscript }
	smKeySwapKybd = -5;   { Switch to previously-used keyboard in current keyscript }
	smKeyDisableKybds = -6;   { Disable keyboards not in system or Roman script }
	smKeyEnableKybds = -7;   { Re-enable keyboards for all enabled scripts }
	smKeyToggleInline = -8;   { Toggle inline input for current keyscript }
	smKeyToggleDirection = -9;   { Toggle default line direction (TESysJust) }
	smKeyNextInputMethod = -10;  { Switch to next input method in current keyscript }
	smKeySwapInputMethod = -11;  { Switch to last-used input method in current keyscript }
	smKeyDisableKybdSwitch = -12;  { Disable switching from the current keyboard }
	smKeySetDirLeftRight = -15;  { Set default line dir to left-right, align left }
	smKeySetDirRightLeft = -16;  { Set default line dir to right-left, align right }
	smKeyRoman = -17;   { Set keyscript to Roman. Does nothing if Roman-only system, unlike KeyScript(smRoman) which forces an update to current default Roman keyboard }

const
{ Force keyboard script switching flag and mask for zero and positive KeyScript verbs }
	smKeyForceKeyScriptBit = 7;    { Force keyboard script switching flag }
	smKeyForceKeyScriptMask = 1 shl smKeyForceKeyScriptBit; { its mask }

const
{ Roman script constants }
                                        { The following are here for backward compatibility, but should not be used. }
                                        { This information should be obtained using GetScript. }
	romanSysFond = $3FFF; {system font id number}
	romanAppFond = 3;    {application font id number}
	romanFlags = $0007; {roman settings}
                                        { Script Manager font equates. }
	smFondStart = $4000; {start from 16K}
	smFondEnd = $C000; {past end of range at 48K}
                                        { Miscellaneous font equates. }
	smUprHalfCharSet = $80;  {first char code in top half of std char set}

const
{ Character Set Extensions }
	diaeresisUprY = $D9;
	fraction = $DA;
	intlCurrency = $DB;
	leftSingGuillemet = $DC;
	rightSingGuillemet = $DD;
	fiLigature = $DE;
	flLigature = $DF;
	dblDagger = $E0;
	centeredDot = $E1;
	baseSingQuote = $E2;
	baseDblQuote = $E3;
	perThousand = $E4;
	circumflexUprA = $E5;
	circumflexUprE = $E6;
	acuteUprA = $E7;
	diaeresisUprE = $E8;
	graveUprE = $E9;
	acuteUprI = $EA;
	circumflexUprI = $EB;
	diaeresisUprI = $EC;
	graveUprI = $ED;
	acuteUprO = $EE;
	circumflexUprO = $EF;
	appleLogo = $F0;
	graveUprO = $F1;
	acuteUprU = $F2;
	circumflexUprU = $F3;
	graveUprU = $F4;
	dotlessLwrI = $F5;
	circumflex = $F6;
	tilde = $F7;
	macron = $F8;
	breveMark = $F9;
	overDot = $FA;
	ringMark = $FB;
	cedilla = $FC;
	doubleAcute = $FD;
	ogonek = $FE;
	hachek = $FF;

const
{ ScriptTokenType values }
	tokenIntl = 4;    {the itl resource number of the tokenizer}
	tokenEmpty = -1;    {used internally as an empty flag}

const
	tokenUnknown = 0;    {chars that do not match a defined token type}
	tokenWhite = 1;    {white space}
	tokenLeftLit = 2;    {literal begin}
	tokenRightLit = 3;    {literal end}
	tokenAlpha = 4;    {alphabetic}
	tokenNumeric = 5;    {numeric}
	tokenNewLine = 6;    {new line}
	tokenLeftComment = 7;    {open comment}
	tokenRightComment = 8;    {close comment}
	tokenLiteral = 9;    {literal}
	tokenEscape = 10;   {character escape (e.g. '\' in "\n", "\t")}
	tokenAltNum = 11;   {alternate number (e.g. $B0-B9 in Arabic,Hebrew)}
	tokenRealNum = 12;   {real number}
	tokenAltReal = 13;   {alternate real number}
	tokenReserve1 = 14;   {reserved}
	tokenReserve2 = 15;   {reserved}
	tokenLeftParen = 16;   {open parenthesis}
	tokenRightParen = 17;   {close parenthesis}
	tokenLeftBracket = 18;   {open square bracket}
	tokenRightBracket = 19;    {close square bracket}

const
	tokenLeftCurly = 20;   {open curly bracket}
	tokenRightCurly = 21;   {close curly bracket}
	tokenLeftEnclose = 22;   {open guillemet}
	tokenRightEnclose = 23;   {close guillemet}
	tokenPlus = 24;
	tokenMinus = 25;
	tokenAsterisk = 26;   {times/multiply}
	tokenDivide = 27;
	tokenPlusMinus = 28;   {plus or minus symbol}
	tokenSlash = 29;
	tokenBackSlash = 30;
	tokenLess = 31;   {less than symbol}
	tokenGreat = 32;   {greater than symbol}
	tokenEqual = 33;
	tokenLessEqual2 = 34;   {less than or equal, 2 characters (e.g. <=)}
	tokenLessEqual1 = 35;   {less than or equal, 1 character}
	tokenGreatEqual2 = 36;   {greater than or equal, 2 characters (e.g. >=)}
	tokenGreatEqual1 = 37;   {greater than or equal, 1 character}
	token2Equal = 38;   {double equal (e.g. ==)}
	tokenColonEqual = 39;    {colon equal}

const
	tokenNotEqual = 40;   {not equal, 1 character}
	tokenLessGreat = 41;   {less/greater, Pascal not equal (e.g. <>)}
	tokenExclamEqual = 42;   {exclamation equal, C not equal (e.g. !=)}
	tokenExclam = 43;   {exclamation point}
	tokenTilde = 44;   {centered tilde}
	tokenComma = 45;
	tokenPeriod = 46;
	tokenLeft2Quote = 47;   {open double quote}
	tokenRight2Quote = 48;   {close double quote}
	tokenLeft1Quote = 49;   {open single quote}
	tokenRight1Quote = 50;   {close single quote}
	token2Quote = 51;   {double quote}
	token1Quote = 52;   {single quote}
	tokenSemicolon = 53;
	tokenPercent = 54;
	tokenCaret = 55;
	tokenUnderline = 56;
	tokenAmpersand = 57;
	tokenAtSign = 58;
	tokenBar = 59;    {vertical bar}

const
	tokenQuestion = 60;
	tokenPi = 61;   {lower-case pi}
	tokenRoot = 62;   {square root symbol}
	tokenSigma = 63;   {capital sigma}
	tokenIntegral = 64;   {integral sign}
	tokenMicro = 65;
	tokenCapPi = 66;   {capital pi}
	tokenInfinity = 67;
	tokenColon = 68;
	tokenHash = 69;   {e.g. #}
	tokenDollar = 70;
	tokenNoBreakSpace = 71;   {non-breaking space}
	tokenFraction = 72;
	tokenIntlCurrency = 73;
	tokenLeftSingGuillemet = 74;
	tokenRightSingGuillemet = 75;
	tokenPerThousand = 76;
	tokenEllipsis = 77;
	tokenCenterDot = 78;
	tokenNil = 127;

const
	delimPad = -2;   { obsolete, misspelled token names kept for backward compatibility }
	tokenTilda = 44;
	tokenCarat = 55;

const
{ Table selectors for GetItlTable }
	smWordSelectTable = 0;    { get word select break table from 'itl2' }
	smWordWrapTable = 1;    { get word wrap break table from 'itl2' }
	smNumberPartsTable = 2;    { get default number parts table from 'itl4' }
	smUnTokenTable = 3;    { get unToken table from 'itl4' }
	smWhiteSpaceList = 4;    { get white space list from 'itl4' }
	iuWordSelectTable = 0;    { <obsolete>  get word select break table from 'itl2' }
	iuWordWrapTable = 1;    { <obsolete>  get word wrap break table from 'itl2' }
	iuNumberPartsTable = 2;    { <obsolete>  get default number parts table from 'itl4' }
	iuUnTokenTable = 3;    { <obsolete>  get unToken table from 'itl4' }
	iuWhiteSpaceList = 4;     { <obsolete>  get white space list from 'itl4' }

{ end of stuff moved from Packages.h }
const
	tokenOK = 0;    { TokenResults }
	tokenOverflow = 1;    { TokenResults }
	stringOverflow = 2;    { TokenResults }
	badDelim = 3;    { TokenResults }
	badEnding = 4;    { TokenResults }
	crash = 5;     { TokenResults }

type
	TokenResults = SInt8;
	CharByteTable = packed array [0..255] of char;
{ "TokenType" was renamed to "ScriptTokenType" because of a name collisions}
type
<<<<<<< HEAD
=======
	TokenResults = SInt8;
	CharByteTable = packed array [0..255] of char;
{ "TokenType" was renamed to "ScriptTokenType" because of a name collisions}
type
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	TokenResults = SInt8;
	CharByteTable = packed array [0..255] of char;
{ "TokenType" was renamed to "ScriptTokenType" because of a name collisions}
type
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
	TokenResults = SInt8;
	CharByteTable = packed array [0..255] of char;
{ "TokenType" was renamed to "ScriptTokenType" because of a name collisions}
type
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	ScriptTokenType = SInt16;
{$ifc TARGET_OS_MAC}
	TokenType							= ScriptTokenType;
{$endc}  {TARGET_OS_MAC}
type
	DelimType = array [0..1] of ScriptTokenType;
	CommentType = array [0..3] of ScriptTokenType;
type
	TokenRec = record
		theToken: ScriptTokenType;
		position: Ptr;               {pointer into original source}
		length: SIGNEDLONG;                 {length of text in original source}
		stringPosition: StringPtr;         {Pascal/C string copy of identifier}
	end;
	TokenRecPtr = ^TokenRec;
type
	TokenBlock = record
		source: Ptr;                 {pointer to stream of characters}
		sourceLength: SIGNEDLONG;           {length of source stream}
		tokenList: Ptr;              {pointer to array of tokens}
		tokenLength: SIGNEDLONG;            {maximum length of TokenList}
		tokenCount: SIGNEDLONG;             {number tokens generated by tokenizer}
		stringList: Ptr;             {pointer to stream of identifiers}
		stringLength: SIGNEDLONG;           {length of string list}
		stringCount: SIGNEDLONG;            {number of bytes currently used}
		doString: Boolean;               {make strings & put into StringList}
		doAppend: Boolean;               {append to TokenList rather than replace}
		doAlphanumeric: Boolean;         {identifiers may include numeric}
		doNest: Boolean;                 {do comments nest?}
		leftDelims: array [0..1] of ScriptTokenType;
		rightDelims: array [0..1] of ScriptTokenType;
		leftComment: array [0..3] of ScriptTokenType;
		rightComment: array [0..3] of ScriptTokenType;
		escapeCode: ScriptTokenType;             {escape symbol code}
		decimalCode: ScriptTokenType;
		itlResource: Handle;            {handle to itl4 resource of current script}
		reserved: array [0..7] of SIGNEDLONG;            { must be zero! }
	end;
	TokenBlockPtr = ^TokenBlock;

{ Result values from GetScriptManagerVariable and SetScriptManagerVariable calls. }
const
	smNotInstalled = 0;    {routine not available in script}
	smBadVerb = -1;   {Bad verb passed to a routine}
	smBadScript = -2;    {Bad script code passed to a routine}

{ Bits in the smGenFlags long. }
{ First (high-order) byte is set from itlc flags byte. }
const
	smfShowIcon = 31;   {Show icon even if only one script}
	smfDualCaret = 30;   {Use dual caret for mixed direction text}
	smfNameTagEnab = 29;   {Reserved for internal use}
	smfUseAssocFontInfo = 28;   {Use the associated font info for FontMetrics calls}
	smfDisableKeyScriptSync = 27;    {Disable font and keyboard script synchronization}

{ We should define masks, too. }
const
	smfDisableKeyScriptSyncMask = 1 shl smfDisableKeyScriptSync; {Disable font and keyboard script synchronization mask}

{ GetScriptManagerVariable and SetScriptManagerVariable verbs (deprecated) }
const
	smSysScript = 18;   {System script}
	smKeyScript = 22;   {Keyboard script}
	smKCHRCache = 38;   {Returns pointer to KCHR cache}
	smRegionCode = 40;    {Returns current region code (verXxx)}

{$ifc not TARGET_CPU_64}
const
	smVersion = 0;    {Script Manager version number}
	smMunged = 2;    {Globals change count}
	smEnabled = 4;    {Count of enabled scripts, incl Roman}
	smBidirect = 6;    {At least one bidirectional script}
	smFontForce = 8;    {Force font flag}
	smIntlForce = 10;   {Force intl flag}
	smForced = 12;   {Script was forced to system script}
	smDefault = 14;   {Script was defaulted to Roman script}
	smPrint = 16;   {Printer action routine}
	smLastScript = 20;   {Last keyboard script}
	smSysRef = 24;   {System folder refNum}
	smKeyCache = 26;   {obsolete}
	smKeySwap = 28;   {Swapping table handle}
	smGenFlags = 30;   {General flags long}
	smOverride = 32;   {Script override flags}
	smCharPortion = 34;   {Ch vs SpExtra proportion}
	smDoubleByte = 36;   {Flag for double-byte script installed}
	smKeyDisableState = 42;    {Returns current keyboard disable state}

{$endc} {not TARGET_CPU_64}

{
 *  GetScriptManagerVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The replacement for GetScriptManagerVariable depends on the
 *    selector used with it. Many of the selectors refer to information
 *    that is not meaningful on a Unicode system, or refer to details
 *    of the Script Manager itself; in general there is no replacement
 *    functionality for these. 
 *    Selectors that have meaningful replacements are as follows (these
 *    are not direct replacements; they provide analogous but more
 *    modern functionality): 
 *    1. smSysScript - To obtain a TextEncoding for the legacy Mac OS
 *    encoding associated with the user's preferred UI language or with
 *    the application's default text encoding, use
 *    CFStringGetSystemEncoding or GetApplicationTextEncoding.
 *    Sometimes smSysScript is just used to get a script code to pass
 *    to GetScriptVariable; in this case the replacements for
 *    GetScriptVariable selectors may provide more information. 
 *    2. smKeyScript - To obtain the intended language associated with
 *    the user's current keyboard input source (plus other languages
 *    that can be input using it): Use
 *    TISCopyCurrentKeyboardInputSource to get that input source, then
 *    pass it to TISGetInputSourceProperty with the
 *    kTISPropertyInputSourceLanguages key. 
 *    3. smKCHRCache - To obtain the key layout data for the keyboard
 *    layout currently in use, use
 *    TISCopyCurrentKeyboardLayoutInputSource to get that input source,
 *    then pass it to TISGetInputSourceProperty with the
 *    kTISPropertyUnicodeKeyLayoutData key (this returns 'uchr' Unicode
 *    layout data only; it will not return any data for keyboard
 *    layouts that only have 'KCHR' data). 
 *    4. smRegionCode - To obtain the locale associated with the user's
 *    preferred formats (for dates, times, numbers, etc.) do: 
 *     CFStringRef curLocaleStringRef = NULL;  // 
 *    localeRef = CFLocaleCopyCurrent();  // 
 *    if (localeRef) (    // 
 *    curLocaleStringRef = CFLocaleGetIdentifier(localeRef);  // 
 *     CFRelease(localeRef);   // 
 *    )   // 
 *    To obtain the user's preferred UI language, do: 
 *    CFArrayRef langArray = (CFArrayRef)CFPreferencesCopyAppValue(
 *    CFSTR("AppleLanguages"), kCFPreferencesCurrentApplication );  //
 *    
 *    the first entry in langArray indicates the preferred language.
 *    For Mac OS X 10.5 there is also a proposed API to return this.
 *    
 *    Selectors that have no meaningful replacement on a Unicode system
 *    include smEnabled, smBidirect, smDoubleByte, etc. 
 *    Selectors that pertain to internal operation of the Script
 *    Manager itself and thus have no meaningful replacement include
 *    smVersion, smMunged, smPrint, smSysRef, etc.
 *  
 *  Summary:
 *    Check the value of Script Manager general environment variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetScriptManagerVariable( selector: SInt16 ): SIGNEDLONG; external name '_GetScriptManagerVariable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SetScriptManagerVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This is mainly used to set the value of variables that control
 *    the internal operation of the Script Manager (selectors
 *    smIntlForce, smGenFlags) and therefore there is no modern
 *    replacement.
 *  
 *  Summary:
 *    Set the value of (some) Script Manager general environment
 *    variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetScriptManagerVariable( selector: SInt16; param: SIGNEDLONG ): OSErr; external name '_SetScriptManagerVariable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{$ifc not TARGET_CPU_64}
{ Values for script redraw flag. }
const
	smRedrawChar = 0;    {Redraw character only}
	smRedrawWord = 1;    {Redraw entire word (2-byte systems)}
	smRedrawLine = -1;    {Redraw entire line (bidirectional systems)}

{$endc} {not TARGET_CPU_64}

{ Bits in the smScriptFlags word }
{  (bits above 8 are non-static) }
const
	smsfIntellCP = 0;    {Script has intelligent cut & paste}
	smsfSingByte = 1;    {Script has only single bytes}
	smsfNatCase = 2;    {Native chars have upper & lower case}
	smsfContext = 3;    {Script is contextual}
	smsfNoForceFont = 4;    {Script will not force characters}
	smsfB0Digits = 5;    {Script has alternate digits at B0-B9}
	smsfAutoInit = 6;    {Auto initialize the script}
	smsfUnivExt = 7;    {Script is handled by universal extension}
	smsfSynchUnstyledTE = 8;    {Script synchronizes for unstyled TE}
	smsfForms = 13;   {Uses contextual forms for letters}
	smsfLigatures = 14;   {Uses contextual ligatures}
	smsfReverse = 15;    {Reverses native text, right-left}

{ GetScriptVariable and SetScriptVariable verbs (deprecated). }
{ Note: Verbs private to script systems are negative, while }
{ those general across script systems are non-negative. }
{$ifc not TARGET_CPU_64}
const
	smScriptVersion = 0;    {Script software version}
	smScriptMunged = 2;    {Script entry changed count}
	smScriptEnabled = 4;    {Script enabled flag}
	smScriptRight = 6;    {Right to left flag}
	smScriptJust = 8;    {Justification flag}
	smScriptRedraw = 10;   {Word redraw flag}
	smScriptSysFond = 12;   {Preferred system font}
	smScriptAppFond = 14;   {Preferred Application font}
	smScriptBundle = 16;   {Beginning of itlb verbs}
	smScriptNumber = 16;   {Script itl0 id}
	smScriptDate = 18;   {Script itl1 id}
	smScriptSort = 20;   {Script itl2 id}
	smScriptFlags = 22;   {flags word}
	smScriptToken = 24;   {Script itl4 id}
	smScriptEncoding = 26;   {id of optional itl5, if present}
	smScriptLang = 28;   {Current language for script}
	smScriptNumDate = 30;   {Script Number/Date formats.}
	smScriptKeys = 32;   {Script KCHR id}
	smScriptIcon = 34;   {ID # of SICN or kcs#/kcs4/kcs8 suite}
	smScriptPrint = 36;   {Script printer action routine}
	smScriptTrap = 38;   {Trap entry pointer}
	smScriptCreator = 40;   {Script file creator}
	smScriptFile = 42;   {Script file name}
	smScriptName = 44;   {Script name}
                                        { There is a hole here for old Kanji private verbs 46-76 }
	smScriptMonoFondSize = 78;   {default monospace FOND (hi) & size (lo)}
	smScriptPrefFondSize = 80;   {preferred FOND (hi) & size (lo)}
	smScriptSmallFondSize = 82;   {default small FOND (hi) & size (lo)}
	smScriptSysFondSize = 84;   {default system FOND (hi) & size (lo)}
	smScriptAppFondSize = 86;   {default app FOND (hi) & size (lo)}
	smScriptHelpFondSize = 88;   {default Help Mgr FOND (hi) & size (lo)}
	smScriptValidStyles = 90;   {mask of valid styles for script}
	smScriptAliasStyle = 92;    {style (set) to use for aliases}

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{ WorldScript private verbs }
{ special negative verbs for Get/SetScriptVariable that were associated with WorldScriptI }
{ move them here to be public }
const
	smLayoutCache = -309; { HiWrd(param) is # entries, LoWrd is max input length}
	smOldVerbSupport = -311; { param is added to old verbs to map to WSI verb}
	smSetKashidas = -291; { param is ON or OFF, old verb = -36}
	smSetKashProp = -287; { param is kashida proportion, old verb = -32}
	smScriptSysBase = -281; { param is associated font to use w/ system font (old verb = -26)}
	smScriptAppBase = -283; { param is associated font to use w/ app font (old verb = -28)}
	smScriptFntBase = -285; { param is associated font to use w/ all other fonts (old verb = -30)}
	smScriptLigatures = -263; { old verb = -8}
	smScriptNumbers = -267;  { old verb = -12}

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  GetScriptVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The replacement for GetScriptVariable depends on the selector
 *    used with it. Many of the selectors refer to information that is
 *    not meaningful on a Unicode system, or refer to details of the
 *    Script Manager itself; in general there is no replacement
 *    functionality for these. 
 *    Selectors that have meaningful replacements are as follows (these
 *    are not direct replacements; they provide analogous but more
 *    modern functionality): 
 *    1. smScriptLang - This was typically used with the system script
 *    to determine the system language. Instead, to obtain the user's
 *    preferred UI language, do: 
 *    CFArrayRef langArray = (CFArrayRef)CFPreferencesCopyAppValue(
 *    CFSTR("AppleLanguages"), kCFPreferencesCurrentApplication );  //
 *    
 *    the first entry in langArray indicates the preferred language.
 *    For Mac OS X 10.5 there is also a proposed API to return this.
 *    
 *    2. Font selectors smScriptSysFond / smScriptSysFondSize,
 *    smScriptAppFond / smScriptAppFondSize,  smScriptMonoFondSize,
 *    smScriptPrefFondSize, smScriptSmallFondSize, smScriptHelpFondSize
 *    - On Mac OS X you generally do not need to worry about setting an
 *    appropriate font based on character script to ensure that
 *    characters are displayed correctly; Unicode encoding and font
 *    fallbacks (to automatically find a font that can display a
 *    character) take care of this. 
 *    However for cases where you do need to do this (such as Carbon
 *    applications that handle non-Unicode text), the CoreText function
 *    CTFontCreateUIFontForLanguage (available in 10.5 and later)
 *    provides a way to get a CTFontRef for a specified language and UI
 *    usage. 
 *    3. Script resource ID selectors smScriptNumber, smScriptDate,
 *    smScriptSort, smScriptToken - These were used in several ways.
 *    Sometimes they were used to get a resource ID so specific fields
 *    in the resource could be examined (e.g. to determine the
 *    appropriate decimal separator or time format). For this usage
 *    CFLocaleGetValue can now be used with an appropriate key (e.g.
 *    kCFLocaleDecimalSeparator) to get similar information (Much of
 *    the information associated with the resource specified by
 *    smScriptToken is not relevant for a Unicode system). Another
 *    usage was to get a resource ID (or a Handle) to pass to some
 *    other system function. For text sorting, this is replaced by the
 *    collation functionality in CFString. For formatting of times,
 *    dates, and numbers, this is replaced by functionality in
 *    CFLocale, CFDateFormatter, CFNumberFormatter. 
 *    4. smScriptKeys - To determine an appropriate keyboard input
 *    source for a particular language, use
 *    TISCopyInputSourceForLanguage. 
 *    5. smScriptIcon - To obtain an icon for a particular keyboard
 *    input source, use TISGetInputSourceProperty with the
 *    kTISPropertyIconRef or kTISPropertyIconImageURL key.
 *  
 *  Summary:
 *    Check the value of a script system's variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetScriptVariable( script: SInt16; selector: SInt16 ): SIGNEDLONG; external name '_GetScriptVariable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  SetScriptVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The primary (perhaps only) recent usage of SetScriptVariable is
 *    to set a specific keyboard layout, as follows: 
 *    err = SetScriptVariable(aScript, smScriptKeys, aKeyLayoutID);  
 *    // 
 *    KeyScript(aScript); // 
 *    The replacement depends on whether the goal is to set the
 *    keyboard layout globally or for a specific TSMDocument. 
 *    To set it globally, use TISSelectInputSource. 
 *    To set it for a specific document, use the TSMDocument property
 *    kTSMDocumentInputSourceOverridePropertyTag.
 *  
 *  Summary:
 *    Set the value of (some of) a script system's variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetScriptVariable( script: SInt16; selector: SInt16; param: SIGNEDLONG ): OSErr; external name '_SetScriptVariable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  GetSysDirection()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The preferred replacement is +[NSParagraphStyle
 *    defaultWritingDirectionForLanguage:]. If you cannot use that, you
 *    may instead call CFLocaleGetLanguageCharacterDirection for the UI
 *    language in which the application is running. See discussion.
 *  
 *  Discussion:
 *    Although GetSysDirection ostensibly provided an overall system
 *    direction, it was patched by the Language Manager in Classic Mac
 *    OS to provide a direction appropriate for the application that
 *    was running. This is similar to the desired behavior in Mac OS X,
 *    in which an application is normally interested in the text
 *    direction associated with the user-interface language in which it
 *    is running (e.g. the first language in the array returned by
 *    CFBundleCopyPreferredLocalizationsFromArray when given the
 *    application's bundle localizations). 
 *    
 *    Calling +[NSParagraphStyle defaultWritingDirectionForLanguage:]
 *    is the preferred way to determine this. If that is not possible
 *    for your application, then an alternative is to call
 *    CFLocaleGetLanguageCharacterDirection for the application's
 *    user-interface language (determined as described above).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSysDirection: SInt16; external name '_GetSysDirection';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  SetSysDirection()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetSysDirection( value: SInt16 ); external name '_SetSysDirection';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  FontScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontFamilyGetEncoding instead.
 *  
 *  Discussion:
 *    This function returns the ScriptCode of the font of the Quickdraw
 *    grafPort or - if that font is Roman and the smFontForce flag is
 *    true - it returns the system ScriptCode. Both the Quickdraw
 *    grafPort and the ScriptCode type are deprecated. Instead, you can
 *    obtain the TextEncoding for an ATSFontFamilyRef using the
 *    function ATSFontFamilyGetEncoding. To get an ATSFontFamilyRef for
 *    a (deprecated) Quickdraw-style FMFontFamily ID, use the
 *    deprecated Quickdraw function FMGetATSFontFamilyRefFromFontFamily.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FontScript: SInt16; external name '_FontScript';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  IntlScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontFamilyGetEncoding instead.
 *  
 *  Discussion:
 *    This function returns the ScriptCode of the font of the Quickdraw
 *    grafPort or - if the smIntlForce flag is true - it returns the
 *    system ScriptCode. Both the Quickdraw grafPort and the ScriptCode
 *    type are deprecated. Instead, you can obtain the TextEncoding for
 *    an ATSFontFamilyRef using the function ATSFontFamilyGetEncoding.
 *    To get an ATSFontFamilyRef for a (deprecated) Quickdraw-style
 *    FMFontFamily ID, use the deprecated Quickdraw function
 *    FMGetATSFontFamilyRefFromFontFamily.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IntlScript: SInt16; external name '_IntlScript';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  FontToScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FMGetATSFontFamilyRefFromFontFamily +
 *    ATSFontFamilyGetEncoding instead.
 *  
 *  Discussion:
 *    This function returns the ScriptCode associated with a Quickdraw
 *    FMFontFamily ID. Both the FMFontFamily type and the ScriptCode
 *    type are deprecated. Instead, you can obtain the TextEncoding for
 *    an ATSFontFamilyRef using the function ATSFontFamilyGetEncoding.
 *    To get an ATSFontFamilyRef for a FMFontFamily ID, use the
 *    deprecated Quickdraw function FMGetATSFontFamilyRefFromFontFamily.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FontToScript( fontNumber: SInt16 ): SInt16; external name '_FontToScript';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  CharacterByteType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use the combination of CFString, UCGetCharProperty and
 *    CFCharacterSet  instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use the
 *    combination of CFString, UCGetCharProperty and CFCharacterSet 
 *    instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function CharacterByteType( textBuf: Ptr; textOffset: SInt16; script: ScriptCode ): SInt16; external name '_CharacterByteType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  CharacterType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use the combination of CFString, UCGetCharProperty and
 *    CFCharacterSet  instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use the
 *    combination of CFString, UCGetCharProperty and CFCharacterSet 
 *    instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function CharacterType( textBuf: Ptr; textOffset: SInt16; script: ScriptCode ): SInt16; external name '_CharacterType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  TransliterateText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFStringTransform instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFStringTransform instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TransliterateText( srcHandle: Handle; dstHandle: Handle; target: SInt16; srcMask: SIGNEDLONG; script: ScriptCode ): OSErr; external name '_TransliterateText';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  FillParseTable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use the combination of CFString, UCGetCharProperty and
 *    CFCharacterSet  instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use the
 *    combination of CFString, UCGetCharProperty and CFCharacterSet 
 *    instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FillParseTable( var table: CharByteTable; script: ScriptCode ): Boolean; external name '_FillParseTable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  GetIntlResource()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Depending on the information that GetIntlResource was being
 *    called to obtain, it can be replaced by the use of
 *    CFLocaleCopyCurrent to get an appropriate CFLocaleRef followed by
 *    one of the following: 
 *    
 *    - CFLocaleGetValue with keys such as kCFLocaleUsesMetricSystem,
 *    kCFLocaleDecimalSeparator, kCFLocaleCurrencySymbol. 
 *    
 *    - CFDateFormatterCreate to get an appropriate CFDateFormatterRef,
 *    followed by CFDateFormatterCopyProperty with keys such as
 *    kCFDateFormatterMonthSymbols, kCFDateFormatterWeekdaySymbols,
 *    kCFDateFormatterAMSymbol. 
 *    
 *    - CFNumberFormatterCreate to get an appropriate
 *    CFNumberFormatterRef, followed by CFNumberFormatterCopyProperty
 *    with keys such as kCFNumberFormatterCurrencyDecimalSeparator,
 *    kCFNumberFormatterMinusSign, kCFNumberFormatterPercentSymbol,
 *    kCFNumberFormatterNegativePrefix.
 *  
 *  Discussion:
 *    GetIntlResource was used to get a handle to the specified
 *    international resource type (e.g. 'itl0', 'itl1', 'itl2, 'itl4').
 *    Typically the resource handle was then used to obtain
 *    locale-specific information, or was passed to a function that
 *    performed - for example - string comparison. GetIntlResource is
 *    no longer recommended; please use the replacements described in
 *    the "deprecated" section.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIntlResource( theID: SInt16 ): Handle; external name '_GetIntlResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  ClearIntlResourceCache()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure ClearIntlResourceCache; external name '_ClearIntlResourceCache';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  GetIntlResourceTable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetIntlResourceTable( script: ScriptCode; tableCode: SInt16; var itlHandle: Handle; var offset: SIGNEDLONG; var length: SIGNEDLONG ); external name '_GetIntlResourceTable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

=======
=======
     File:       Script.p
=======
     File:       CarbonCore/Script.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Script Manager interfaces
 
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1986-2008 by Apple Inc., all rights reserved
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit Script;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
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

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$setc ACCESSOR_CALLS_ARE_FUNCTIONS := TRUE}
{$setc CALL_NOT_IN_CARBON := FALSE}
{$setc OLDROUTINENAMES := FALSE}
{$setc OPAQUE_TOOLBOX_STRUCTS := TRUE}
{$setc OPAQUE_UPP_TYPES := TRUE}
{$setc OTCARBONAPPLICATION := TRUE}
{$setc OTKERNEL := FALSE}
{$setc PM_USE_SESSION_APIS := TRUE}
{$setc TARGET_API_MAC_CARBON := TRUE}
{$setc TARGET_API_MAC_OS8 := FALSE}
{$setc TARGET_API_MAC_OSX := TRUE}
{$setc TARGET_CARBON := TRUE}
{$setc TARGET_CPU_68K := FALSE}
{$setc TARGET_CPU_MIPS := FALSE}
{$setc TARGET_CPU_SPARC := FALSE}
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,IntlResources;
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ Meta script codes:}
const
	smSystemScript = -1;   { designates system script.}
	smCurrentScript = -2;   { designates current font script.}
	smAllScripts = -3;    { designates any script}

{
   Script codes:
   These specify a Mac OS encoding that is related to a FOND ID range.
   Some of the encodings have several variants (e.g. for different localized systems)
    which all share the same script code.
   Not all of these script codes are currently supported by Apple software.
   Notes:
   - Script code 0 (smRoman) is also used (instead of smGreek) for the Greek encoding
     in the Greek localized system.
   - Script code 28 (smEthiopic) is also used for the Inuit encoding in the Inuktitut
     system.
}
const
	smRoman = 0;
	smJapanese = 1;
	smTradChinese = 2;    { Traditional Chinese}
	smKorean = 3;
	smArabic = 4;
	smHebrew = 5;
	smGreek = 6;
	smCyrillic = 7;
	smRSymbol = 8;    { Right-left symbol}
	smDevanagari = 9;
	smGurmukhi = 10;
	smGujarati = 11;
	smOriya = 12;
	smBengali = 13;
	smTamil = 14;
	smTelugu = 15;
	smKannada = 16;   { Kannada/Kanarese}
	smMalayalam = 17;
	smSinhalese = 18;
	smBurmese = 19;
	smKhmer = 20;   { Khmer/Cambodian}
	smThai = 21;
	smLao = 22;
	smGeorgian = 23;
	smArmenian = 24;
	smSimpChinese = 25;   { Simplified Chinese}
	smTibetan = 26;
	smMongolian = 27;
	smEthiopic = 28;
	smGeez = 28;   { Synonym for smEthiopic}
	smCentralEuroRoman = 29;   { For Czech, Slovak, Polish, Hungarian, Baltic langs}
	smVietnamese = 30;
	smExtArabic = 31;   { extended Arabic}
	smUninterp = 32;    { uninterpreted symbols, e.g. palette symbols}

{ Extended script code for full Unicode input}
const
	smUnicodeScript = $7E;

{ Obsolete script code names (kept for backward compatibility):}
const
	smChinese = 2;    { (Use smTradChinese or smSimpChinese)}
	smRussian = 7;    { Use smCyrillic}
                                        { smMaldivian = 25: deleted, no code for Maldivian}
	smLaotian = 22;   { Use smLao                                     }
	smAmharic = 28;   { Use smEthiopic or smGeez}
	smSlavic = 29;   { Use smCentralEuroRoman}
	smEastEurRoman = 29;   { Use smCentralEuroRoman}
	smSindhi = 31;   { Use smExtArabic}
	smKlingon = 32;

{
   Language codes:
   These specify a language implemented using a particular Mac OS encoding.
   Not all of these language codes are currently supported by Apple software.
}
const
	langEnglish = 0;    { smRoman script}
	langFrench = 1;    { smRoman script}
	langGerman = 2;    { smRoman script}
	langItalian = 3;    { smRoman script}
	langDutch = 4;    { smRoman script}
	langSwedish = 5;    { smRoman script}
	langSpanish = 6;    { smRoman script}
	langDanish = 7;    { smRoman script}
	langPortuguese = 8;    { smRoman script}
	langNorwegian = 9;    { (Bokmal) smRoman script}
	langHebrew = 10;   { smHebrew script}
	langJapanese = 11;   { smJapanese script}
	langArabic = 12;   { smArabic script}
	langFinnish = 13;   { smRoman script}
	langGreek = 14;   { Greek script (monotonic) using smRoman script code}
	langIcelandic = 15;   { modified smRoman/Icelandic script}
	langMaltese = 16;   { Roman script}
	langTurkish = 17;   { modified smRoman/Turkish script}
	langCroatian = 18;   { modified smRoman/Croatian script}
	langTradChinese = 19;   { Chinese (Mandarin) in traditional characters}
	langUrdu = 20;   { smArabic script}
	langHindi = 21;   { smDevanagari script}
	langThai = 22;   { smThai script}
	langKorean = 23;    { smKorean script}

const
	langLithuanian = 24;   { smCentralEuroRoman script}
	langPolish = 25;   { smCentralEuroRoman script}
	langHungarian = 26;   { smCentralEuroRoman script}
	langEstonian = 27;   { smCentralEuroRoman script}
	langLatvian = 28;   { smCentralEuroRoman script}
	langSami = 29;   { language of the Sami people of N. Scandinavia             }
	langFaroese = 30;   { modified smRoman/Icelandic script                      }
	langFarsi = 31;   { modified smArabic/Farsi script}
	langPersian = 31;   { Synonym for langFarsi}
	langRussian = 32;   { smCyrillic script}
	langSimpChinese = 33;   { Chinese (Mandarin) in simplified characters}
	langFlemish = 34;   { smRoman script}
	langIrishGaelic = 35;   { smRoman or modified smRoman/Celtic script (without dot above)   }
	langAlbanian = 36;   { smRoman script}
	langRomanian = 37;   { modified smRoman/Romanian script}
	langCzech = 38;   { smCentralEuroRoman script}
	langSlovak = 39;   { smCentralEuroRoman script}
	langSlovenian = 40;   { modified smRoman/Croatian script}
	langYiddish = 41;   { smHebrew script}
	langSerbian = 42;   { smCyrillic script}
	langMacedonian = 43;   { smCyrillic script}
	langBulgarian = 44;   { smCyrillic script}
	langUkrainian = 45;   { modified smCyrillic/Ukrainian script}
	langByelorussian = 46;   { smCyrillic script}
	langBelorussian = 46;    { Synonym for langByelorussian                          }

const
	langUzbek = 47;   { Cyrillic script}
	langKazakh = 48;   { Cyrillic script}
	langAzerbaijani = 49;   { Azerbaijani in Cyrillic script}
	langAzerbaijanAr = 50;   { Azerbaijani in Arabic script}
	langArmenian = 51;   { smArmenian script}
	langGeorgian = 52;   { smGeorgian script}
	langMoldavian = 53;   { smCyrillic script}
	langKirghiz = 54;   { Cyrillic script}
	langTajiki = 55;   { Cyrillic script}
	langTurkmen = 56;   { Cyrillic script}
	langMongolian = 57;   { Mongolian in smMongolian script}
	langMongolianCyr = 58;   { Mongolian in Cyrillic script}
	langPashto = 59;   { Arabic script}
	langKurdish = 60;   { smArabic script}
	langKashmiri = 61;   { Arabic script}
	langSindhi = 62;   { Arabic script}
	langTibetan = 63;   { smTibetan script}
	langNepali = 64;   { smDevanagari script}
	langSanskrit = 65;   { smDevanagari script}
	langMarathi = 66;   { smDevanagari script}
	langBengali = 67;   { smBengali script}
	langAssamese = 68;   { smBengali script}
	langGujarati = 69;   { smGujarati script}
	langPunjabi = 70;    { smGurmukhi script}

const
	langOriya = 71;   { smOriya script}
	langMalayalam = 72;   { smMalayalam script}
	langKannada = 73;   { smKannada script}
	langTamil = 74;   { smTamil script}
	langTelugu = 75;   { smTelugu script}
	langSinhalese = 76;   { smSinhalese script}
	langBurmese = 77;   { smBurmese script}
	langKhmer = 78;   { smKhmer script}
	langLao = 79;   { smLao script}
	langVietnamese = 80;   { smVietnamese script}
	langIndonesian = 81;   { smRoman script}
	langTagalog = 82;   { Roman script}
	langMalayRoman = 83;   { Malay in smRoman script}
	langMalayArabic = 84;   { Malay in Arabic script}
	langAmharic = 85;   { smEthiopic script}
	langTigrinya = 86;   { smEthiopic script}
	langOromo = 87;   { smEthiopic script}
	langSomali = 88;   { smRoman script}
	langSwahili = 89;   { smRoman script}
	langKinyarwanda = 90;   { smRoman script}
	langRuanda = 90;   { synonym for langKinyarwanda}
	langRundi = 91;   { smRoman script}
	langNyanja = 92;   { smRoman script}
	langChewa = 92;   { synonym for langNyanja}
	langMalagasy = 93;   { smRoman script}
	langEsperanto = 94;    { Roman script}

const
	langWelsh = 128;  { modified smRoman/Celtic script}
	langBasque = 129;  { smRoman script}
	langCatalan = 130;  { smRoman script}
	langLatin = 131;  { smRoman script}
	langQuechua = 132;  { smRoman script}
	langGuarani = 133;  { smRoman script}
	langAymara = 134;  { smRoman script}
	langTatar = 135;  { Cyrillic script}
	langUighur = 136;  { Arabic script}
	langDzongkha = 137;  { (lang of Bhutan) smTibetan script}
	langJavaneseRom = 138;  { Javanese in smRoman script}
	langSundaneseRom = 139;  { Sundanese in smRoman script}
	langGalician = 140;  { smRoman script}
	langAfrikaans = 141;   { smRoman script                                   }

const
	langBreton = 142;  { smRoman or modified smRoman/Celtic script                 }
	langInuktitut = 143;  { Inuit script using smEthiopic script code                 }
	langScottishGaelic = 144;  { smRoman or modified smRoman/Celtic script                 }
	langManxGaelic = 145;  { smRoman or modified smRoman/Celtic script                 }
	langIrishGaelicScript = 146;  { modified smRoman/Gaelic script (using dot above)               }
	langTongan = 147;  { smRoman script                                   }
	langGreekAncient = 148;  { Classical Greek, polytonic orthography                    }
	langGreenlandic = 149;  { smRoman script                                   }
	langAzerbaijanRoman = 150;  { Azerbaijani in Roman script                             }
	langNynorsk = 151;   { Norwegian Nyorsk in smRoman}

const
	langUnspecified = 32767; { Special code for use in resources (such as 'itlm')           }

{
   Obsolete language code names (kept for backward compatibility):
   Misspelled, ambiguous, misleading, considered pejorative, archaic, etc.
}
const
	langPortugese = 8;    { Use langPortuguese}
	langMalta = 16;   { Use langMaltese}
	langYugoslavian = 18;   { (use langCroatian, langSerbian, etc.)}
	langChinese = 19;   { (use langTradChinese or langSimpChinese)}
	langLettish = 28;   { Use langLatvian                                     }
	langLapponian = 29;   { Use langSami}
	langLappish = 29;   { Use langSami}
	langSaamisk = 29;   { Use langSami                                    }
	langFaeroese = 30;   { Use langFaroese                                     }
	langIrish = 35;   { Use langIrishGaelic                                  }
	langGalla = 87;   { Use langOromo                                 }
	langAfricaans = 141;  { Use langAfrikaans                                }
	langGreekPoly = 148;   { Use langGreekAncient}

{
   Region codes:
   These typically specify a combination of a language code and a particular region.
   Some of these numeric values are reserved just for extra resource IDs associated
   with certain regions; these are not actual region codes, and are noted in the comments.
   Not all of the region codes are currently supported by Apple software.
   When relevant, the following list also provides:
   - The Apple part number (P/N) code for software localized for the specified region.
   - The two-letter ISO language and country codes (from ISO 639 and ISO 3166). The
     language code (lowercase) is first, then '_', then the country code (uppercase).
}

const
{ P/N    ISO    codes  comments}
	verUS = 0;    {       en_US}
	verFrance = 1;    { F  fr_FR}
	verBritain = 2;    { B  en_GB}
	verGermany = 3;    { D  de_DE}
	verItaly = 4;    { T  it_IT}
	verNetherlands = 5;    { N  nl_NL}
	verFlemish = 6;    { FN nl_BE     Flemish (Dutch) for Belgium                 }
	verSweden = 7;    { S  sv_SE}
	verSpain = 8;    { E  es_ES       Spanish for Spain}
	verDenmark = 9;    { DK da_DK}
	verPortugal = 10;   { PO pt_PT     Portuguese for Portugal}
	verFrCanada = 11;   { C  fr_CA       French for Canada}
	verNorway = 12;   { H  nb_NO       Bokmål}
	verIsrael = 13;   { HB he_IL     Hebrew}
	verJapan = 14;   { J  ja_JP}
	verAustralia = 15;   { X  en_AU       English for Australia}
	verArabic = 16;   { AB ar       Arabic for N Africa, Arabian peninsula, Levant}
	verFinland = 17;   { K  fi_FI}
	verFrSwiss = 18;   { SF fr_CH     French Swiss}
	verGrSwiss = 19;   { SD de_CH     German Swiss}
	verGreece = 20;   { GR el_GR     Monotonic Greek (modern)}
	verIceland = 21;   { IS is_IS}
	verMalta = 22;   { MA mt_MT}
	verCyprus = 23;   { CY el_CY}
	verTurkey = 24;   { TU tr_TR}
	verYugoCroatian = 25;    { YU hr_HR     Croatian for Yugoslavia; now use verCroatia (68)}

const
	verNetherlandsComma = 26;   {              ID for KCHR resource - Dutch}
	verFlemishPoint = 27;   {              ID for KCHR resource - Belgium}
	verCanadaComma = 28;   {              ID for KCHR resource - Canadian ISO}
	verCanadaPoint = 29;   {              ID for KCHR resource - Canadian; now unused}
	vervariantPortugal = 30;   {              ID for resource; now unused}
	vervariantNorway = 31;   {              ID for resource; now unused}
	vervariantDenmark = 32;    {              ID for KCHR resource - Danish Mac Plus}

const
	verIndiaHindi = 33;   {       hi_IN     Hindi for India}
	verPakistanUrdu = 34;   { UR ur_PK     Urdu for Pakistan                        }
	verTurkishModified = 35;   {       tr_TR}
	verItalianSwiss = 36;   { ST it_CH     Italian Swiss}
	verInternational = 37;   { Z  en-ascii English for international use; ASCII chars only     }
                                        {              38 is unassigned}
	verRomania = 39;   { RO ro_RO}
	verGreekAncient = 40;   {       grc      Ancient Greek, polytonic orthography          }
	verLithuania = 41;   { LT lt_LT}
	verPoland = 42;   { PL pl_PL}
	verHungary = 43;   { MG hu_HU}
	verEstonia = 44;   { EE et_EE}
	verLatvia = 45;   { LV lv_LV}
	verSami = 46;   {       se                                        }
	verFaroeIsl = 47;   { FA fo_FO                                    }
	verIran = 48;   { PS fa_IR     Persian/Farsi}
	verRussia = 49;   { RS ru_RU     Russian}
	verIreland = 50;   { GA ga_IE     Irish Gaelic for Ireland (without dot above)     }
	verKorea = 51;   { KH ko_KR}
	verChina = 52;   { CH zh_CN     Simplified Chinese}
	verTaiwan = 53;   { TA zh_TW     Traditional Chinese}
	verThailand = 54;   { TH th_TH}
	verScriptGeneric = 55;   { SS          Generic script system (no language or script)       }
	verCzech = 56;   { CZ cs_CZ}
	verSlovak = 57;   { SL sk_SK}
	verEastAsiaGeneric = 58;   { FE          Generic East Asia system (no language or script) }
	verMagyar = 59;   {       hu_HU     Unused; see verHungary}
	verBengali = 60;   {       bn         Bangladesh or India}
	verBelarus = 61;    { BY be_BY}

const
	verUkraine = 62;   { UA uk_UA}
                                        {              63 is unassigned}
	verGreeceAlt = 64;   {       el_GR     unused                              }
	verSerbian = 65;   { SR sr_CS                                    }
	verSlovenian = 66;   { SV sl_SI                                    }
	verMacedonian = 67;   { MD mk_MK                                    }
	verCroatia = 68;   { CR hr_HR}
                                        {              69 is unassigned}
	verGermanReformed = 70;   {       de-1996     Reformed orthography (used formerly unassigned 70)   }
	verBrazil = 71;   { BR pt_BR     Portuguese for Brazil}
	verBulgaria = 72;   { BG bg_BG}
	verCatalonia = 73;   { CA ca_ES     Catalan for Spain}
	verMultilingual = 74;   { ZM mul        (no language or script)}
	verScottishGaelic = 75;   { GD gd}
	verManxGaelic = 76;   { GV gv       Isle of Man}
	verBreton = 77;   { BZ br}
	verNunavut = 78;   { IU iu_CA     Inuktitut for Canada}
	verWelsh = 79;   { CU cy}
                                        {              80 is ID for KCHR resource - Canadian CSA}
	verIrishGaelicScript = 81;   { GS ga-Latg_IE  Irish Gaelic for Ireland (using dot above)}
	verEngCanada = 82;   { V  en_CA       English for Canada}
	verBhutan = 83;   { BH dz_BT     Dzongkha for Bhutan}
	verArmenian = 84;   { HY hy_AM}
	verGeorgian = 85;   { KR ka_GE}
	verSpLatinAmerica = 86;   { LA es_XL     Spanish for Latin America (private ISO 3166 code)}
                                        {              87 is ID for KCHR resource - Spanish ISO}
	verTonga = 88;   { TS to_TO}
                                        {              89 is ID for KCHR resource - Polish Modified}
                                        {              90 is ID for KCHR resource - Catalan ISO}
	verFrenchUniversal = 91;   {       fr         French generic}
	verAustria = 92;   { AU de_AT     German for Austria}
                                        { Y          93 is unused alternate for verSpLatinAmerica}
	verGujarati = 94;   {       gu_IN}
	verPunjabi = 95;   {       pa         Pakistan or India}
	verIndiaUrdu = 96;   {       ur_IN     Urdu for India}
	verVietnam = 97;    {       vi_VN}

const
	verFrBelgium = 98;   { BF fr_BE     French for Belgium                       }
	verUzbek = 99;   { BD uz_UZ                                    }
	verSingapore = 100;  { SG en_SG     (Assume English, not Chinese or Malay)                                 }
	verNynorsk = 101;  { NY nn_NO     Norwegian Nynorsk                        }
	verAfrikaans = 102;  { AK af_ZA                                    }
	verEsperanto = 103;  {       eo                                        }
	verMarathi = 104;  {       mr_IN                                      }
	verTibetan = 105;  {       bo                                        }
	verNepal = 106;  {       ne_NP                                      }
	verGreenland = 107;  {       kl                                        }
	verIrelandEnglish = 108;   {       en_IE     English for Ireland, with Euro for currency}

{
   Other extra resource IDs assigned in the same number space:
    179 is ID for KCHR & itl_ resources - Cornish
    581 is ID for KCHR resource - Irish Gaelic script alternate
    582 is ID for KCHR resource - Ogham
    779 is ID for KCHR resource - Welsh alternate
   1111 is ID for KCHR resource - French numeric
}

{
   Obsolete region code names (kept for backward compatibility):
   Misspelled or alternate form, ambiguous, misleading, considered pejorative, archaic, etc.
}
const
	verFrBelgiumLux = 6;    { Incorrect; 6 is Flemish, not French, for Belgium; use verFlemish     }
	verBelgiumLux = 6;    { Use verFlemish}
	verArabia = 16;   { Use verArabic}
	verYugoslavia = 25;   { Use verYugoCroatian (same number, now unused), or newer verCroatia}
	verBelgiumLuxPoint = 27;   { Use verFlemishPoint}
	verIndia = 33;   { Use verIndiaHindi}
	verPakistan = 34;   { Use verPakistanUrdu                                  }
	verRumania = 39;   { Alternate for verRomania                              }
	verGreecePoly = 40;   { Use verGreekAncient                                  }
	verLapland = 46;   { Use verSami                                       }
	verFaeroeIsl = 47;   { Use verFaroeIsl                                     }
	verGenericFE = 58;   { Use verEastAsiaGeneric                              }
	verFarEastGeneric = 58;   { Use verEastAsiaGeneric}
	verByeloRussian = 61;   { Alternate for verBelarus                              }
	verUkrania = 62;   { Use verUkraine}
	verAlternateGr = 64;   { Use verGreeceAlt                                   }
	verSerbia = 65;   { Alternate for verSerbian                              }
	verSlovenia = 66;   { Alternate for verSlovenian                            }
	verMacedonia = 67;   { Alternate for verMacedonian                             }
	verBrittany = 77;   { Alternate for verBreton                              }
	verWales = 79;   { Alternate for verWelsh                              }
	verArmenia = 84;   { Alternate for verArmenian                           }
	verGeorgia = 85;   { Alternate for verGeorgian                           }
	verAustriaGerman = 92;   { Use verAustria                                   }
	verTibet = 105;   { Use verTibetan                                   }

const
	minCountry = verUS;
	maxCountry = verIrelandEnglish;

const
{ Calendar Codes }
	calGregorian = 0;
	calArabicCivil = 1;
	calArabicLunar = 2;
	calJapanese = 3;
	calJewish = 4;
	calCoptic = 5;
	calPersian = 6;

const
{ Integer Format Codes }
	intWestern = 0;
	intArabic = 1;
	intRoman = 2;
	intJapanese = 3;
	intEuropean = 4;
	intOutputMask = $8000;

const
{ CharByte byte types }
	smSingleByte = 0;
	smFirstByte = -1;
	smLastByte = 1;
	smMiddleByte = 2;

const
{ CharType field masks }
	smcTypeMask = $000F;
	smcReserved = $00F0;
	smcClassMask = $0F00;
	smcOrientationMask = $1000; {two-byte script glyph orientation}
	smcRightMask = $2000;
	smcUpperMask = $4000;
	smcDoubleMask = $8000;

const
{ Basic CharType character types }
	smCharPunct = $0000;
	smCharAscii = $0001;
	smCharEuro = $0007;
	smCharExtAscii = $0007; { More correct synonym for smCharEuro }
                                        { Additional CharType character types for script systems }
	smCharKatakana = $0002; {Japanese Katakana}
	smCharHiragana = $0003; {Japanese Hiragana}
	smCharIdeographic = $0004; {Hanzi, Kanji, Hanja}
	smCharTwoByteGreek = $0005; {2-byte Greek in Far East systems}
	smCharTwoByteRussian = $0006; {2-byte Cyrillic in Far East systems}
	smCharBidirect = $0008; {Arabic/Hebrew}
	smCharContextualLR = $0009; {Contextual left-right: Thai, Indic scripts}
	smCharNonContextualLR = $000A; {Non-contextual left-right: Cyrillic, Greek}
	smCharHangul = $000C; {Korean Hangul}
	smCharJamo = $000D; {Korean Jamo}
	smCharBopomofo = $000E; {Chinese Bopomofo}
	smCharGanaKana = $000F; {Shared for Japanese Hiragana & Katakana}
                                        { old names for some of above, for backward compatibility }
	smCharFISKana = $0002; {Katakana}
	smCharFISGana = $0003; {Hiragana}
	smCharFISIdeo = $0004; {Hanzi, Kanji, Hanja}

const
	smCharFISGreek = $0005; {2-byte Greek in Far East systems}
	smCharFISRussian = $0006; {2-byte Cyrillic in Far East systems}
                                        { CharType classes for punctuation (smCharPunct) }
	smPunctNormal = $0000;
	smPunctNumber = $0100;
	smPunctSymbol = $0200;
	smPunctBlank = $0300; { Additional CharType classes for punctuation in two-byte systems }
	smPunctRepeat = $0400; { repeat marker }
	smPunctGraphic = $0500; { line graphics }
                                        { CharType Katakana and Hiragana classes for two-byte systems }
	smKanaSmall = $0100; {small kana character}
	smKanaHardOK = $0200; {can have dakuten}
	smKanaSoftOK = $0300; {can have dakuten or han-dakuten}
                                        { CharType Ideographic classes for two-byte systems }
	smIdeographicLevel1 = $0000; {level 1 char}
	smIdeographicLevel2 = $0100; {level 2 char}
	smIdeographicUser = $0200; {user char}
                                        { old names for above, for backward compatibility }
	smFISClassLvl1 = $0000; {level 1 char}
	smFISClassLvl2 = $0100; {level 2 char}
	smFISClassUser = $0200; {user char}
                                        { CharType Jamo classes for Korean systems }
	smJamoJaeum = $0000; {simple consonant char}
	smJamoBogJaeum = $0100; {complex consonant char}
	smJamoMoeum = $0200; {simple vowel char}
	smJamoBogMoeum = $0300; {complex vowel char}

const
{ CharType glyph orientation for two-byte systems }
	smCharHorizontal = $0000; { horizontal character form, or for both }
	smCharVertical = $1000; { vertical character form }
                                        { CharType directions }
	smCharLeft = $0000;
	smCharRight = $2000; { CharType case modifers }
	smCharLower = $0000;
	smCharUpper = $4000; { CharType character size modifiers (1 or multiple bytes). }
	smChar1byte = $0000;
	smChar2byte = $8000;

const
{ TransliterateText target types for Roman }
	smTransAscii = 0;    {convert to ASCII}
	smTransNative = 1;    {convert to font script}
	smTransCase = $FE; {convert case for all text}
	smTransSystem = $FF; {convert to system script}
                                        { TransliterateText target types for two-byte scripts }
	smTransAscii1 = 2;    {1-byte Roman}
	smTransAscii2 = 3;    {2-byte Roman}
	smTransKana1 = 4;    {1-byte Japanese Katakana}
	smTransKana2 = 5;     {2-byte Japanese Katakana}

const
	smTransGana2 = 7;    {2-byte Japanese Hiragana (no 1-byte Hiragana)}
	smTransHangul2 = 8;    {2-byte Korean Hangul}
	smTransJamo2 = 9;    {2-byte Korean Jamo}
	smTransBopomofo2 = 10;   {2-byte Chinese Bopomofo}
                                        { TransliterateText target modifiers }
	smTransLower = $4000; {target becomes lowercase}
	smTransUpper = $8000; {target becomes uppercase}
                                        { TransliterateText resource format numbers }
	smTransRuleBaseFormat = 1;    {Rule based trsl resource format }
	smTransHangulFormat = 2;    {Table based Hangul trsl resource format}
                                        { TransliterateText property flags }
	smTransPreDoubleByting = 1;    {Convert all text to double byte before transliteration}
	smTransPreLowerCasing = 2;     {Convert all text to lower case before transliteration}

const
{ TransliterateText source mask - general }
	smMaskAll = -1; {Convert all text}
                                        { TransliterateText source masks }
	smMaskAscii = $00000001; {2^smTransAscii}
	smMaskNative = $00000002; {2^smTransNative}
                                        { TransliterateText source masks for two-byte scripts }
	smMaskAscii1 = $00000004; {2^smTransAscii1}
	smMaskAscii2 = $00000008; {2^smTransAscii2}
	smMaskKana1 = $00000010; {2^smTransKana1}
	smMaskKana2 = $00000020; {2^smTransKana2}
	smMaskGana2 = $00000080; {2^smTransGana2}
	smMaskHangul2 = $00000100; {2^smTransHangul2}
	smMaskJamo2 = $00000200; {2^smTransJamo2}
	smMaskBopomofo2 = $00000400; {2^smTransBopomofo2}

const
{ Special script code values for International Utilities }
	iuSystemScript = -1;   { <obsolete>  system script }
	iuCurrentScript = -2;    { <obsolete>  current script (for font of grafPort) }

const
{ Negative verbs for KeyScript }
	smKeyNextScript = -1;   { Switch to next available script }
	smKeySysScript = -2;   { Switch to the system script }
	smKeySwapScript = -3;   { Switch to previously-used script }
                                        { New for System 7.0: }
	smKeyNextKybd = -4;   { Switch to next keyboard in current keyscript }
	smKeySwapKybd = -5;   { Switch to previously-used keyboard in current keyscript }
	smKeyDisableKybds = -6;   { Disable keyboards not in system or Roman script }
	smKeyEnableKybds = -7;   { Re-enable keyboards for all enabled scripts }
	smKeyToggleInline = -8;   { Toggle inline input for current keyscript }
	smKeyToggleDirection = -9;   { Toggle default line direction (TESysJust) }
	smKeyNextInputMethod = -10;  { Switch to next input method in current keyscript }
	smKeySwapInputMethod = -11;  { Switch to last-used input method in current keyscript }
	smKeyDisableKybdSwitch = -12;  { Disable switching from the current keyboard }
	smKeySetDirLeftRight = -15;  { Set default line dir to left-right, align left }
	smKeySetDirRightLeft = -16;  { Set default line dir to right-left, align right }
	smKeyRoman = -17;   { Set keyscript to Roman. Does nothing if Roman-only system, unlike KeyScript(smRoman) which forces an update to current default Roman keyboard }

const
{ Force keyboard script switching flag and mask for zero and positive KeyScript verbs }
	smKeyForceKeyScriptBit = 7;    { Force keyboard script switching flag }
	smKeyForceKeyScriptMask = 1 shl smKeyForceKeyScriptBit; { its mask }

const
{ Roman script constants }
                                        { The following are here for backward compatibility, but should not be used. }
                                        { This information should be obtained using GetScript. }
	romanSysFond = $3FFF; {system font id number}
	romanAppFond = 3;    {application font id number}
	romanFlags = $0007; {roman settings}
                                        { Script Manager font equates. }
	smFondStart = $4000; {start from 16K}
	smFondEnd = $C000; {past end of range at 48K}
                                        { Miscellaneous font equates. }
	smUprHalfCharSet = $80;  {first char code in top half of std char set}

const
{ Character Set Extensions }
	diaeresisUprY = $D9;
	fraction = $DA;
	intlCurrency = $DB;
	leftSingGuillemet = $DC;
	rightSingGuillemet = $DD;
	fiLigature = $DE;
	flLigature = $DF;
	dblDagger = $E0;
	centeredDot = $E1;
	baseSingQuote = $E2;
	baseDblQuote = $E3;
	perThousand = $E4;
	circumflexUprA = $E5;
	circumflexUprE = $E6;
	acuteUprA = $E7;
	diaeresisUprE = $E8;
	graveUprE = $E9;
	acuteUprI = $EA;
	circumflexUprI = $EB;
	diaeresisUprI = $EC;
	graveUprI = $ED;
	acuteUprO = $EE;
	circumflexUprO = $EF;
	appleLogo = $F0;
	graveUprO = $F1;
	acuteUprU = $F2;
	circumflexUprU = $F3;
	graveUprU = $F4;
	dotlessLwrI = $F5;
	circumflex = $F6;
	tilde = $F7;
	macron = $F8;
	breveMark = $F9;
	overDot = $FA;
	ringMark = $FB;
	cedilla = $FC;
	doubleAcute = $FD;
	ogonek = $FE;
	hachek = $FF;

const
{ ScriptTokenType values }
	tokenIntl = 4;    {the itl resource number of the tokenizer}
	tokenEmpty = -1;    {used internally as an empty flag}

const
	tokenUnknown = 0;    {chars that do not match a defined token type}
	tokenWhite = 1;    {white space}
	tokenLeftLit = 2;    {literal begin}
	tokenRightLit = 3;    {literal end}
	tokenAlpha = 4;    {alphabetic}
	tokenNumeric = 5;    {numeric}
	tokenNewLine = 6;    {new line}
	tokenLeftComment = 7;    {open comment}
	tokenRightComment = 8;    {close comment}
	tokenLiteral = 9;    {literal}
	tokenEscape = 10;   {character escape (e.g. '\' in "\n", "\t")}
	tokenAltNum = 11;   {alternate number (e.g. $B0-B9 in Arabic,Hebrew)}
	tokenRealNum = 12;   {real number}
	tokenAltReal = 13;   {alternate real number}
	tokenReserve1 = 14;   {reserved}
	tokenReserve2 = 15;   {reserved}
	tokenLeftParen = 16;   {open parenthesis}
	tokenRightParen = 17;   {close parenthesis}
	tokenLeftBracket = 18;   {open square bracket}
	tokenRightBracket = 19;    {close square bracket}

const
	tokenLeftCurly = 20;   {open curly bracket}
	tokenRightCurly = 21;   {close curly bracket}
	tokenLeftEnclose = 22;   {open guillemet}
	tokenRightEnclose = 23;   {close guillemet}
	tokenPlus = 24;
	tokenMinus = 25;
	tokenAsterisk = 26;   {times/multiply}
	tokenDivide = 27;
	tokenPlusMinus = 28;   {plus or minus symbol}
	tokenSlash = 29;
	tokenBackSlash = 30;
	tokenLess = 31;   {less than symbol}
	tokenGreat = 32;   {greater than symbol}
	tokenEqual = 33;
	tokenLessEqual2 = 34;   {less than or equal, 2 characters (e.g. <=)}
	tokenLessEqual1 = 35;   {less than or equal, 1 character}
	tokenGreatEqual2 = 36;   {greater than or equal, 2 characters (e.g. >=)}
	tokenGreatEqual1 = 37;   {greater than or equal, 1 character}
	token2Equal = 38;   {double equal (e.g. ==)}
	tokenColonEqual = 39;    {colon equal}

const
	tokenNotEqual = 40;   {not equal, 1 character}
	tokenLessGreat = 41;   {less/greater, Pascal not equal (e.g. <>)}
	tokenExclamEqual = 42;   {exclamation equal, C not equal (e.g. !=)}
	tokenExclam = 43;   {exclamation point}
	tokenTilde = 44;   {centered tilde}
	tokenComma = 45;
	tokenPeriod = 46;
	tokenLeft2Quote = 47;   {open double quote}
	tokenRight2Quote = 48;   {close double quote}
	tokenLeft1Quote = 49;   {open single quote}
	tokenRight1Quote = 50;   {close single quote}
	token2Quote = 51;   {double quote}
	token1Quote = 52;   {single quote}
	tokenSemicolon = 53;
	tokenPercent = 54;
	tokenCaret = 55;
	tokenUnderline = 56;
	tokenAmpersand = 57;
	tokenAtSign = 58;
	tokenBar = 59;    {vertical bar}

const
	tokenQuestion = 60;
	tokenPi = 61;   {lower-case pi}
	tokenRoot = 62;   {square root symbol}
	tokenSigma = 63;   {capital sigma}
	tokenIntegral = 64;   {integral sign}
	tokenMicro = 65;
	tokenCapPi = 66;   {capital pi}
	tokenInfinity = 67;
	tokenColon = 68;
	tokenHash = 69;   {e.g. #}
	tokenDollar = 70;
	tokenNoBreakSpace = 71;   {non-breaking space}
	tokenFraction = 72;
	tokenIntlCurrency = 73;
	tokenLeftSingGuillemet = 74;
	tokenRightSingGuillemet = 75;
	tokenPerThousand = 76;
	tokenEllipsis = 77;
	tokenCenterDot = 78;
	tokenNil = 127;

const
	delimPad = -2;   { obsolete, misspelled token names kept for backward compatibility }
	tokenTilda = 44;
	tokenCarat = 55;

const
{ Table selectors for GetItlTable }
	smWordSelectTable = 0;    { get word select break table from 'itl2' }
	smWordWrapTable = 1;    { get word wrap break table from 'itl2' }
	smNumberPartsTable = 2;    { get default number parts table from 'itl4' }
	smUnTokenTable = 3;    { get unToken table from 'itl4' }
	smWhiteSpaceList = 4;    { get white space list from 'itl4' }
	iuWordSelectTable = 0;    { <obsolete>  get word select break table from 'itl2' }
	iuWordWrapTable = 1;    { <obsolete>  get word wrap break table from 'itl2' }
	iuNumberPartsTable = 2;    { <obsolete>  get default number parts table from 'itl4' }
	iuUnTokenTable = 3;    { <obsolete>  get unToken table from 'itl4' }
	iuWhiteSpaceList = 4;     { <obsolete>  get white space list from 'itl4' }

{ end of stuff moved from Packages.h }
const
	tokenOK = 0;    { TokenResults }
	tokenOverflow = 1;    { TokenResults }
	stringOverflow = 2;    { TokenResults }
	badDelim = 3;    { TokenResults }
	badEnding = 4;    { TokenResults }
	crash = 5;     { TokenResults }

type
	TokenResults = SInt8;
	CharByteTable = packed array [0..255] of char;
{ "TokenType" was renamed to "ScriptTokenType" because of a name collisions}
type
	ScriptTokenType = SInt16;
{$ifc TARGET_OS_MAC}
	TokenType							= ScriptTokenType;
{$endc}  {TARGET_OS_MAC}
type
	DelimType = array [0..1] of ScriptTokenType;
	CommentType = array [0..3] of ScriptTokenType;
type
	TokenRec = record
		theToken: ScriptTokenType;
		position: Ptr;               {pointer into original source}
		length: SIGNEDLONG;                 {length of text in original source}
		stringPosition: StringPtr;         {Pascal/C string copy of identifier}
	end;
	TokenRecPtr = ^TokenRec;
type
	TokenBlock = record
		source: Ptr;                 {pointer to stream of characters}
		sourceLength: SIGNEDLONG;           {length of source stream}
		tokenList: Ptr;              {pointer to array of tokens}
		tokenLength: SIGNEDLONG;            {maximum length of TokenList}
		tokenCount: SIGNEDLONG;             {number tokens generated by tokenizer}
		stringList: Ptr;             {pointer to stream of identifiers}
		stringLength: SIGNEDLONG;           {length of string list}
		stringCount: SIGNEDLONG;            {number of bytes currently used}
		doString: Boolean;               {make strings & put into StringList}
		doAppend: Boolean;               {append to TokenList rather than replace}
		doAlphanumeric: Boolean;         {identifiers may include numeric}
		doNest: Boolean;                 {do comments nest?}
		leftDelims: array [0..1] of ScriptTokenType;
		rightDelims: array [0..1] of ScriptTokenType;
		leftComment: array [0..3] of ScriptTokenType;
		rightComment: array [0..3] of ScriptTokenType;
		escapeCode: ScriptTokenType;             {escape symbol code}
		decimalCode: ScriptTokenType;
		itlResource: Handle;            {handle to itl4 resource of current script}
		reserved: array [0..7] of SIGNEDLONG;            { must be zero! }
	end;
	TokenBlockPtr = ^TokenBlock;

{ Result values from GetScriptManagerVariable and SetScriptManagerVariable calls. }
const
	smNotInstalled = 0;    {routine not available in script}
	smBadVerb = -1;   {Bad verb passed to a routine}
	smBadScript = -2;    {Bad script code passed to a routine}

{ Bits in the smGenFlags long. }
{ First (high-order) byte is set from itlc flags byte. }
const
	smfShowIcon = 31;   {Show icon even if only one script}
	smfDualCaret = 30;   {Use dual caret for mixed direction text}
	smfNameTagEnab = 29;   {Reserved for internal use}
	smfUseAssocFontInfo = 28;   {Use the associated font info for FontMetrics calls}
	smfDisableKeyScriptSync = 27;    {Disable font and keyboard script synchronization}

{ We should define masks, too. }
const
	smfDisableKeyScriptSyncMask = 1 shl smfDisableKeyScriptSync; {Disable font and keyboard script synchronization mask}

{ GetScriptManagerVariable and SetScriptManagerVariable verbs (deprecated) }
const
	smSysScript = 18;   {System script}
	smKeyScript = 22;   {Keyboard script}
	smKCHRCache = 38;   {Returns pointer to KCHR cache}
	smRegionCode = 40;    {Returns current region code (verXxx)}

{$ifc not TARGET_CPU_64}
const
	smVersion = 0;    {Script Manager version number}
	smMunged = 2;    {Globals change count}
	smEnabled = 4;    {Count of enabled scripts, incl Roman}
	smBidirect = 6;    {At least one bidirectional script}
	smFontForce = 8;    {Force font flag}
	smIntlForce = 10;   {Force intl flag}
	smForced = 12;   {Script was forced to system script}
	smDefault = 14;   {Script was defaulted to Roman script}
	smPrint = 16;   {Printer action routine}
	smLastScript = 20;   {Last keyboard script}
	smSysRef = 24;   {System folder refNum}
	smKeyCache = 26;   {obsolete}
	smKeySwap = 28;   {Swapping table handle}
	smGenFlags = 30;   {General flags long}
	smOverride = 32;   {Script override flags}
	smCharPortion = 34;   {Ch vs SpExtra proportion}
	smDoubleByte = 36;   {Flag for double-byte script installed}
	smKeyDisableState = 42;    {Returns current keyboard disable state}

{$endc} {not TARGET_CPU_64}

{
 *  GetScriptManagerVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The replacement for GetScriptManagerVariable depends on the
 *    selector used with it. Many of the selectors refer to information
 *    that is not meaningful on a Unicode system, or refer to details
 *    of the Script Manager itself; in general there is no replacement
 *    functionality for these. 
 *    Selectors that have meaningful replacements are as follows (these
 *    are not direct replacements; they provide analogous but more
 *    modern functionality): 
 *    1. smSysScript - To obtain a TextEncoding for the legacy Mac OS
 *    encoding associated with the user's preferred UI language or with
 *    the application's default text encoding, use
 *    CFStringGetSystemEncoding or GetApplicationTextEncoding.
 *    Sometimes smSysScript is just used to get a script code to pass
 *    to GetScriptVariable; in this case the replacements for
 *    GetScriptVariable selectors may provide more information. 
 *    2. smKeyScript - To obtain the intended language associated with
 *    the user's current keyboard input source (plus other languages
 *    that can be input using it): Use
 *    TISCopyCurrentKeyboardInputSource to get that input source, then
 *    pass it to TISGetInputSourceProperty with the
 *    kTISPropertyInputSourceLanguages key. 
 *    3. smKCHRCache - To obtain the key layout data for the keyboard
 *    layout currently in use, use
 *    TISCopyCurrentKeyboardLayoutInputSource to get that input source,
 *    then pass it to TISGetInputSourceProperty with the
 *    kTISPropertyUnicodeKeyLayoutData key (this returns 'uchr' Unicode
 *    layout data only; it will not return any data for keyboard
 *    layouts that only have 'KCHR' data). 
 *    4. smRegionCode - To obtain the locale associated with the user's
 *    preferred formats (for dates, times, numbers, etc.) do: 
 *     CFStringRef curLocaleStringRef = NULL;  // 
 *    localeRef = CFLocaleCopyCurrent();  // 
 *    if (localeRef) (    // 
 *    curLocaleStringRef = CFLocaleGetIdentifier(localeRef);  // 
 *     CFRelease(localeRef);   // 
 *    )   // 
 *    To obtain the user's preferred UI language, do: 
 *    CFArrayRef langArray = (CFArrayRef)CFPreferencesCopyAppValue(
 *    CFSTR("AppleLanguages"), kCFPreferencesCurrentApplication );  //
 *    
 *    the first entry in langArray indicates the preferred language.
 *    For Mac OS X 10.5 there is also a proposed API to return this.
 *    
 *    Selectors that have no meaningful replacement on a Unicode system
 *    include smEnabled, smBidirect, smDoubleByte, etc. 
 *    Selectors that pertain to internal operation of the Script
 *    Manager itself and thus have no meaningful replacement include
 *    smVersion, smMunged, smPrint, smSysRef, etc.
 *  
 *  Summary:
 *    Check the value of Script Manager general environment variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetScriptManagerVariable( selector: SInt16 ): SIGNEDLONG; external name '_GetScriptManagerVariable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetScriptManagerVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This is mainly used to set the value of variables that control
 *    the internal operation of the Script Manager (selectors
 *    smIntlForce, smGenFlags) and therefore there is no modern
 *    replacement.
 *  
 *  Summary:
 *    Set the value of (some) Script Manager general environment
 *    variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetScriptManagerVariable( selector: SInt16; param: SIGNEDLONG ): OSErr; external name '_SetScriptManagerVariable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$ifc not TARGET_CPU_64}
{ Values for script redraw flag. }
const
	smRedrawChar = 0;    {Redraw character only}
	smRedrawWord = 1;    {Redraw entire word (2-byte systems)}
	smRedrawLine = -1;    {Redraw entire line (bidirectional systems)}

{$endc} {not TARGET_CPU_64}

{ Bits in the smScriptFlags word }
{  (bits above 8 are non-static) }
const
	smsfIntellCP = 0;    {Script has intelligent cut & paste}
	smsfSingByte = 1;    {Script has only single bytes}
	smsfNatCase = 2;    {Native chars have upper & lower case}
	smsfContext = 3;    {Script is contextual}
	smsfNoForceFont = 4;    {Script will not force characters}
	smsfB0Digits = 5;    {Script has alternate digits at B0-B9}
	smsfAutoInit = 6;    {Auto initialize the script}
	smsfUnivExt = 7;    {Script is handled by universal extension}
	smsfSynchUnstyledTE = 8;    {Script synchronizes for unstyled TE}
	smsfForms = 13;   {Uses contextual forms for letters}
	smsfLigatures = 14;   {Uses contextual ligatures}
	smsfReverse = 15;    {Reverses native text, right-left}

{ GetScriptVariable and SetScriptVariable verbs (deprecated). }
{ Note: Verbs private to script systems are negative, while }
{ those general across script systems are non-negative. }
{$ifc not TARGET_CPU_64}
const
	smScriptVersion = 0;    {Script software version}
	smScriptMunged = 2;    {Script entry changed count}
	smScriptEnabled = 4;    {Script enabled flag}
	smScriptRight = 6;    {Right to left flag}
	smScriptJust = 8;    {Justification flag}
	smScriptRedraw = 10;   {Word redraw flag}
	smScriptSysFond = 12;   {Preferred system font}
	smScriptAppFond = 14;   {Preferred Application font}
	smScriptBundle = 16;   {Beginning of itlb verbs}
	smScriptNumber = 16;   {Script itl0 id}
	smScriptDate = 18;   {Script itl1 id}
	smScriptSort = 20;   {Script itl2 id}
	smScriptFlags = 22;   {flags word}
	smScriptToken = 24;   {Script itl4 id}
	smScriptEncoding = 26;   {id of optional itl5, if present}
	smScriptLang = 28;   {Current language for script}
	smScriptNumDate = 30;   {Script Number/Date formats.}
	smScriptKeys = 32;   {Script KCHR id}
	smScriptIcon = 34;   {ID # of SICN or kcs#/kcs4/kcs8 suite}
	smScriptPrint = 36;   {Script printer action routine}
	smScriptTrap = 38;   {Trap entry pointer}
	smScriptCreator = 40;   {Script file creator}
	smScriptFile = 42;   {Script file name}
	smScriptName = 44;   {Script name}
                                        { There is a hole here for old Kanji private verbs 46-76 }
	smScriptMonoFondSize = 78;   {default monospace FOND (hi) & size (lo)}
	smScriptPrefFondSize = 80;   {preferred FOND (hi) & size (lo)}
	smScriptSmallFondSize = 82;   {default small FOND (hi) & size (lo)}
	smScriptSysFondSize = 84;   {default system FOND (hi) & size (lo)}
	smScriptAppFondSize = 86;   {default app FOND (hi) & size (lo)}
	smScriptHelpFondSize = 88;   {default Help Mgr FOND (hi) & size (lo)}
	smScriptValidStyles = 90;   {mask of valid styles for script}
	smScriptAliasStyle = 92;    {style (set) to use for aliases}

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{ WorldScript private verbs }
{ special negative verbs for Get/SetScriptVariable that were associated with WorldScriptI }
{ move them here to be public }
const
	smLayoutCache = -309; { HiWrd(param) is # entries, LoWrd is max input length}
	smOldVerbSupport = -311; { param is added to old verbs to map to WSI verb}
	smSetKashidas = -291; { param is ON or OFF, old verb = -36}
	smSetKashProp = -287; { param is kashida proportion, old verb = -32}
	smScriptSysBase = -281; { param is associated font to use w/ system font (old verb = -26)}
	smScriptAppBase = -283; { param is associated font to use w/ app font (old verb = -28)}
	smScriptFntBase = -285; { param is associated font to use w/ all other fonts (old verb = -30)}
	smScriptLigatures = -263; { old verb = -8}
	smScriptNumbers = -267;  { old verb = -12}

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  GetScriptVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The replacement for GetScriptVariable depends on the selector
 *    used with it. Many of the selectors refer to information that is
 *    not meaningful on a Unicode system, or refer to details of the
 *    Script Manager itself; in general there is no replacement
 *    functionality for these. 
 *    Selectors that have meaningful replacements are as follows (these
 *    are not direct replacements; they provide analogous but more
 *    modern functionality): 
 *    1. smScriptLang - This was typically used with the system script
 *    to determine the system language. Instead, to obtain the user's
 *    preferred UI language, do: 
 *    CFArrayRef langArray = (CFArrayRef)CFPreferencesCopyAppValue(
 *    CFSTR("AppleLanguages"), kCFPreferencesCurrentApplication );  //
 *    
 *    the first entry in langArray indicates the preferred language.
 *    For Mac OS X 10.5 there is also a proposed API to return this.
 *    
 *    2. Font selectors smScriptSysFond / smScriptSysFondSize,
 *    smScriptAppFond / smScriptAppFondSize,  smScriptMonoFondSize,
 *    smScriptPrefFondSize, smScriptSmallFondSize, smScriptHelpFondSize
 *    - On Mac OS X you generally do not need to worry about setting an
 *    appropriate font based on character script to ensure that
 *    characters are displayed correctly; Unicode encoding and font
 *    fallbacks (to automatically find a font that can display a
 *    character) take care of this. 
 *    However for cases where you do need to do this (such as Carbon
 *    applications that handle non-Unicode text), the CoreText function
 *    CTFontCreateUIFontForLanguage (available in 10.5 and later)
 *    provides a way to get a CTFontRef for a specified language and UI
 *    usage. 
 *    3. Script resource ID selectors smScriptNumber, smScriptDate,
 *    smScriptSort, smScriptToken - These were used in several ways.
 *    Sometimes they were used to get a resource ID so specific fields
 *    in the resource could be examined (e.g. to determine the
 *    appropriate decimal separator or time format). For this usage
 *    CFLocaleGetValue can now be used with an appropriate key (e.g.
 *    kCFLocaleDecimalSeparator) to get similar information (Much of
 *    the information associated with the resource specified by
 *    smScriptToken is not relevant for a Unicode system). Another
 *    usage was to get a resource ID (or a Handle) to pass to some
 *    other system function. For text sorting, this is replaced by the
 *    collation functionality in CFString. For formatting of times,
 *    dates, and numbers, this is replaced by functionality in
 *    CFLocale, CFDateFormatter, CFNumberFormatter. 
 *    4. smScriptKeys - To determine an appropriate keyboard input
 *    source for a particular language, use
 *    TISCopyInputSourceForLanguage. 
 *    5. smScriptIcon - To obtain an icon for a particular keyboard
 *    input source, use TISGetInputSourceProperty with the
 *    kTISPropertyIconRef or kTISPropertyIconImageURL key.
 *  
 *  Summary:
 *    Check the value of a script system's variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetScriptVariable( script: SInt16; selector: SInt16 ): SIGNEDLONG; external name '_GetScriptVariable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  SetScriptVariable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The primary (perhaps only) recent usage of SetScriptVariable is
 *    to set a specific keyboard layout, as follows: 
 *    err = SetScriptVariable(aScript, smScriptKeys, aKeyLayoutID);  
 *    // 
 *    KeyScript(aScript); // 
 *    The replacement depends on whether the goal is to set the
 *    keyboard layout globally or for a specific TSMDocument. 
 *    To set it globally, use TISSelectInputSource. 
 *    To set it for a specific document, use the TSMDocument property
 *    kTSMDocumentInputSourceOverridePropertyTag.
 *  
 *  Summary:
 *    Set the value of (some of) a script system's variables.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetScriptVariable( script: SInt16; selector: SInt16; param: SIGNEDLONG ): OSErr; external name '_SetScriptVariable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  GetSysDirection()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The preferred replacement is +[NSParagraphStyle
 *    defaultWritingDirectionForLanguage:]. If you cannot use that, you
 *    may instead call CFLocaleGetLanguageCharacterDirection for the UI
 *    language in which the application is running. See discussion.
 *  
 *  Discussion:
 *    Although GetSysDirection ostensibly provided an overall system
 *    direction, it was patched by the Language Manager in Classic Mac
 *    OS to provide a direction appropriate for the application that
 *    was running. This is similar to the desired behavior in Mac OS X,
 *    in which an application is normally interested in the text
 *    direction associated with the user-interface language in which it
 *    is running (e.g. the first language in the array returned by
 *    CFBundleCopyPreferredLocalizationsFromArray when given the
 *    application's bundle localizations). 
 *    
 *    Calling +[NSParagraphStyle defaultWritingDirectionForLanguage:]
 *    is the preferred way to determine this. If that is not possible
 *    for your application, then an alternative is to call
 *    CFLocaleGetLanguageCharacterDirection for the application's
 *    user-interface language (determined as described above).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSysDirection: SInt16; external name '_GetSysDirection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetSysDirection()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetSysDirection( value: SInt16 ); external name '_SetSysDirection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  FontScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontFamilyGetEncoding instead.
 *  
 *  Discussion:
 *    This function returns the ScriptCode of the font of the Quickdraw
 *    grafPort or - if that font is Roman and the smFontForce flag is
 *    true - it returns the system ScriptCode. Both the Quickdraw
 *    grafPort and the ScriptCode type are deprecated. Instead, you can
 *    obtain the TextEncoding for an ATSFontFamilyRef using the
 *    function ATSFontFamilyGetEncoding. To get an ATSFontFamilyRef for
 *    a (deprecated) Quickdraw-style FMFontFamily ID, use the
 *    deprecated Quickdraw function FMGetATSFontFamilyRefFromFontFamily.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FontScript: SInt16; external name '_FontScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IntlScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontFamilyGetEncoding instead.
 *  
 *  Discussion:
 *    This function returns the ScriptCode of the font of the Quickdraw
 *    grafPort or - if the smIntlForce flag is true - it returns the
 *    system ScriptCode. Both the Quickdraw grafPort and the ScriptCode
 *    type are deprecated. Instead, you can obtain the TextEncoding for
 *    an ATSFontFamilyRef using the function ATSFontFamilyGetEncoding.
 *    To get an ATSFontFamilyRef for a (deprecated) Quickdraw-style
 *    FMFontFamily ID, use the deprecated Quickdraw function
 *    FMGetATSFontFamilyRefFromFontFamily.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IntlScript: SInt16; external name '_IntlScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  FontToScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FMGetATSFontFamilyRefFromFontFamily +
 *    ATSFontFamilyGetEncoding instead.
 *  
 *  Discussion:
 *    This function returns the ScriptCode associated with a Quickdraw
 *    FMFontFamily ID. Both the FMFontFamily type and the ScriptCode
 *    type are deprecated. Instead, you can obtain the TextEncoding for
 *    an ATSFontFamilyRef using the function ATSFontFamilyGetEncoding.
 *    To get an ATSFontFamilyRef for a FMFontFamily ID, use the
 *    deprecated Quickdraw function FMGetATSFontFamilyRefFromFontFamily.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FontToScript( fontNumber: SInt16 ): SInt16; external name '_FontToScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  CharacterByteType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use the combination of CFString, UCGetCharProperty and
 *    CFCharacterSet  instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use the
 *    combination of CFString, UCGetCharProperty and CFCharacterSet 
 *    instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function CharacterByteType( textBuf: Ptr; textOffset: SInt16; script: ScriptCode ): SInt16; external name '_CharacterByteType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  CharacterType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use the combination of CFString, UCGetCharProperty and
 *    CFCharacterSet  instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use the
 *    combination of CFString, UCGetCharProperty and CFCharacterSet 
 *    instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function CharacterType( textBuf: Ptr; textOffset: SInt16; script: ScriptCode ): SInt16; external name '_CharacterType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TransliterateText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFStringTransform instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFStringTransform instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TransliterateText( srcHandle: Handle; dstHandle: Handle; target: SInt16; srcMask: SIGNEDLONG; script: ScriptCode ): OSErr; external name '_TransliterateText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  FillParseTable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use the combination of CFString, UCGetCharProperty and
 *    CFCharacterSet  instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use the
 *    combination of CFString, UCGetCharProperty and CFCharacterSet 
 *    instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FillParseTable( var table: CharByteTable; script: ScriptCode ): Boolean; external name '_FillParseTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetIntlResource()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Depending on the information that GetIntlResource was being
 *    called to obtain, it can be replaced by the use of
 *    CFLocaleCopyCurrent to get an appropriate CFLocaleRef followed by
 *    one of the following: 
 *    
 *    - CFLocaleGetValue with keys such as kCFLocaleUsesMetricSystem,
 *    kCFLocaleDecimalSeparator, kCFLocaleCurrencySymbol. 
 *    
 *    - CFDateFormatterCreate to get an appropriate CFDateFormatterRef,
 *    followed by CFDateFormatterCopyProperty with keys such as
 *    kCFDateFormatterMonthSymbols, kCFDateFormatterWeekdaySymbols,
 *    kCFDateFormatterAMSymbol. 
 *    
 *    - CFNumberFormatterCreate to get an appropriate
 *    CFNumberFormatterRef, followed by CFNumberFormatterCopyProperty
 *    with keys such as kCFNumberFormatterCurrencyDecimalSeparator,
 *    kCFNumberFormatterMinusSign, kCFNumberFormatterPercentSymbol,
 *    kCFNumberFormatterNegativePrefix.
 *  
 *  Discussion:
 *    GetIntlResource was used to get a handle to the specified
 *    international resource type (e.g. 'itl0', 'itl1', 'itl2, 'itl4').
 *    Typically the resource handle was then used to obtain
 *    locale-specific information, or was passed to a function that
 *    performed - for example - string comparison. GetIntlResource is
 *    no longer recommended; please use the replacements described in
 *    the "deprecated" section.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIntlResource( theID: SInt16 ): Handle; external name '_GetIntlResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ClearIntlResourceCache()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure ClearIntlResourceCache; external name '_ClearIntlResourceCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetIntlResourceTable()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure GetIntlResourceTable(script: ScriptCode; tableCode: SInt16; var itlHandle: Handle; var offset: SInt32; var length: SInt32); external name '_GetIntlResourceTable';
{$ifc CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetIntlResourceTable( script: ScriptCode; tableCode: SInt16; var itlHandle: Handle; var offset: SIGNEDLONG; var length: SIGNEDLONG ); external name '_GetIntlResourceTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  GetScriptUtilityAddress()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function GetScriptUtilityAddress(selector: SInt16; Before: boolean; script: ScriptCode): UniversalProcPtr; external name '_GetScriptUtilityAddress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  SetScriptUtilityAddress()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure SetScriptUtilityAddress(selector: SInt16; Before: boolean; routineAddr: UniversalProcPtr; script: ScriptCode); external name '_SetScriptUtilityAddress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  GetScriptQDPatchAddress()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function GetScriptQDPatchAddress(trapNum: SInt16; Before: boolean; forPrinting: boolean; script: ScriptCode): UniversalProcPtr; external name '_GetScriptQDPatchAddress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  SetScriptQDPatchAddress()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure SetScriptQDPatchAddress(trapNum: SInt16; Before: boolean; forPrinting: boolean; routineAddr: UniversalProcPtr; script: ScriptCode); external name '_SetScriptQDPatchAddress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{
 *  SetIntlResource()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure SetIntlResource(refNum: SInt16; theID: SInt16; intlHandle: Handle); external name '_SetIntlResource';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{
 *  CharByte()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         not available
 *    CarbonLib:        not available
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function CharByte(textBuf: Ptr; textOffset: SInt16): SInt16; external name '_CharByte';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  SetIntlResource()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  Transliterate()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  ParseTable()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{$ifc not TARGET_CPU_64}
{
 *  IntlTokenize()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IntlTokenize( tokenParam: TokenBlockPtr ): TokenResults; external name '_IntlTokenize';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  CharByte()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  CharType()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  Transliterate()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  ParseTable()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{$ifc not TARGET_CPU_64}
{
 *  IntlTokenize()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Discussion:
 *    This function is obsolete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IntlTokenize( tokenParam: TokenBlockPtr ): TokenResults; external name '_IntlTokenize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
