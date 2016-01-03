{
     File:       PrintCore/PMDefinitionsDeprecated.h
 
     Contains:   Deprecated Carbon Printing Manager Interfaces.
 
     Copyright (c) 1998-2006,2008 Apple Inc. All Rights Reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
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

unit PMDefinitionsDeprecated;
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ifc not TARGET_CPU_64}

{$ALIGN MAC68K}

{ Printing objects }
type
	PMPrintContext = ^SInt32; { an opaque type }
	PMPrintContextPtr = ^PMPrintContext;
	PMDialog = ^SInt32; { an opaque type }
	PMDialogPtr = ^PMDialog;
const
	kSizeOfTPrint = 120;   { size of old TPrint record }

// Ticket items cannot be locked in 10.5 and later.
const
	kPMLocked = true;

type
	PMColorMode = UInt16;
	PMColorModePtr = ^PMColorMode;
const
	kPMBlackAndWhite = 1;
	kPMGray = 2;
	kPMColor = 3;
	kPMColorModeDuotone = 4;    { 2 channels }
	kPMColorModeSpecialColor = 5;     { to allow for special colors such as metalic, light cyan, etc. }

{ Constants to define the ColorSync Intents. These intents may be used }
{ to set an intent part way through a page or for an entire document. }
type
	PMColorSyncIntent = UInt32;
	PMColorSyncIntentPtr = ^PMColorSyncIntent;
const
	kPMColorIntentUndefined = $0000; { User or application have not declared an intent, use the printer's default. }
	kPMColorIntentAutomatic = $0001; { Automatically match for photos and graphics anywhere on the page. }
	kPMColorIntentPhoto = $0002; { Use Photographic (cmPerceptual) intent for all contents. }
	kPMColorIntentBusiness = $0004; { Use Business Graphics (cmSaturation) intent for all contents. }
	kPMColorIntentRelColor = $0008; { Use Relative Colormetrics (Logo Colors) for the page. }
	kPMColorIntentAbsColor = $0010; { Use absolute colormetric for the page. }
	kPMColorIntentUnused = $FFE0; { Remaining bits unused at this time. }

{ Unused duplex binding directions: }
type
	PMDuplexBinding = UInt16;
	PMDuplexBindingPtr = ^PMDuplexBinding;
const
	kPMDuplexBindingLeftRight = 1;
	kPMDuplexBindingTopDown = 2;


type
	PMTag = UInt32;
	PMTagPtr = ^PMTag;
const
{ common tags }
	kPMCurrentValue = FourCharCode('curr'); { current setting or value }
	kPMDefaultValue = FourCharCode('dflt'); { default setting or value }
	kPMMinimumValue = FourCharCode('minv'); { the minimum setting or value }
	kPMMaximumValue = FourCharCode('maxv'); { the maximum setting or value }
                                        { profile tags }
	kPMSourceProfile = FourCharCode('srcp'); { source profile }
                                        { resolution tags }
	kPMMinRange = FourCharCode('mnrg'); { Min range supported by a printer }
	kPMMaxRange = FourCharCode('mxrg'); { Max range supported by a printer }
	kPMMinSquareResolution = FourCharCode('mins'); { Min with X and Y resolution equal }
	kPMMaxSquareResolution = FourCharCode('maxs'); { Max with X and Y resolution equal }
	kPMDefaultResolution = FourCharCode('dftr'); { printer default resolution }

{ Useful Constants for PostScript Injection }
const
	kPSPageInjectAllPages = -1;   { specifies to inject on all pages }
	kPSInjectionMaxDictSize = 5;     { maximum size of a dictionary used for PSInjection }

{ PostScript Injection values for kPSInjectionPlacementKey }
type
	PSInjectionPlacement = UInt16;
	PSInjectionPlacementPtr = ^PSInjectionPlacement;
const
	kPSInjectionBeforeSubsection = 1;
	kPSInjectionAfterSubsection = 2;
	kPSInjectionReplaceSubsection = 3;

{ PostScript Injection values for kPSInjectionSectionKey }
type
	PSInjectionSection = SInt32;
	PSInjectionSectionPtr = ^PSInjectionSection;
const
{ Job }
	kInjectionSectJob = 1;    { CoverPage }
	kInjectionSectCoverPage = 2;

{ PostScript Injection values for kPSInjectionSubSectionKey }
type
	PSInjectionSubsection = SInt32;
	PSInjectionSubsectionPtr = ^PSInjectionSubsection;
const
	kInjectionSubPSAdobe = 1;    { %!PS-Adobe           }
	kInjectionSubPSAdobeEPS = 2;    { %!PS-Adobe-3.0 EPSF-3.0    }
	kInjectionSubBoundingBox = 3;    { BoundingBox          }
	kInjectionSubEndComments = 4;    { EndComments          }
	kInjectionSubOrientation = 5;    { Orientation          }
	kInjectionSubPages = 6;    { Pages            }
	kInjectionSubPageOrder = 7;    { PageOrder          }
	kInjectionSubBeginProlog = 8;    { BeginProlog          }
	kInjectionSubEndProlog = 9;    { EndProlog          }
	kInjectionSubBeginSetup = 10;   { BeginSetup          }
	kInjectionSubEndSetup = 11;   { EndSetup             }
	kInjectionSubBeginDefaults = 12;   { BeginDefaults       }
	kInjectionSubEndDefaults = 13;   { EndDefaults          }
	kInjectionSubDocFonts = 14;   { DocumentFonts       }
	kInjectionSubDocNeededFonts = 15;   { DocumentNeededFonts       }
	kInjectionSubDocSuppliedFonts = 16;   { DocumentSuppliedFonts  }
	kInjectionSubDocNeededRes = 17;   { DocumentNeededResources    }
	kInjectionSubDocSuppliedRes = 18;   { DocumentSuppliedResources}
	kInjectionSubDocCustomColors = 19;   { DocumentCustomColors     }
	kInjectionSubDocProcessColors = 20;   { DocumentProcessColors  }
	kInjectionSubPlateColor = 21;   { PlateColor          }
	kInjectionSubPageTrailer = 22;   { PageTrailer           }
	kInjectionSubTrailer = 23;   { Trailer              }
	kInjectionSubEOF = 24;   { EOF                 }
	kInjectionSubBeginFont = 25;   { BeginFont          }
	kInjectionSubEndFont = 26;   { EndFont              }
	kInjectionSubBeginResource = 27;   { BeginResource       }
	kInjectionSubEndResource = 28;   { EndResource          }
	kInjectionSubPage = 29;   { Page               }
	kInjectionSubBeginPageSetup = 30;   { BeginPageSetup        }
	kInjectionSubEndPageSetup = 31;    { EndPageSetup          }

{ Document format strings }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatPICT CFSTRP('application/vnd.apple.printing-pict')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatPICTPS CFSTRP('application/vnd.apple.printing-pict-ps')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatPICTPSwPSNormalizer CFSTRP('application/vnd.apple.printing-pict-ps-viapsnormalizer')}
{$endc}
{ Data format strings }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDataFormatPDF CFSTRP('application/pdf')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDataFormatPS CFSTRP('application/postscript')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDataFormatPICT CFSTRP('application/vnd.apple.printing-pict')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDataFormatPICTwPS CFSTRP('application/vnd.apple.printing-pict-ps')}
{$endc}

{ Graphic context strings }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMGraphicsContextQuickdraw CFSTRP('com.apple.graphicscontext.quickdraw')}
{$endc}

{ PostScript Injection Dictionary Keys }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPSInjectionSectionKey CFSTRP('section')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPSInjectionSubSectionKey CFSTRP('subsection')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPSInjectionPageKey CFSTRP('page')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPSInjectionPlacementKey CFSTRP('place')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPSInjectionPostScriptKey CFSTRP('psdata')}
{$endc}
{ PDF Workflow Keys }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowForlderURLKey CFSTRP('folderURL')}
{$endc}

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
