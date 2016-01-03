{
    This file is assembled from all the Interface files.
}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$setc MACOSALLINCLUDE := TRUE}
unit MacOSAll;
interface

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc (defined CPUPOWERPC32 or defined CPUI386) and not defined(iphonesim)}
=======
{$ifc defined CPUPOWERPC32 or defined CPUI386}
>>>>>>> graemeg/cpstrnew
=======
{$ifc defined CPUPOWERPC32 or defined CPUI386}
>>>>>>> graemeg/cpstrnew
=======
{$ifc defined CPUPOWERPC32 or defined CPUI386}
>>>>>>> graemeg/cpstrnew
=======
{$ifc defined CPUPOWERPC32 or defined CPUI386}
>>>>>>> origin/cpstrnew
{$linkframework Carbon}
{$elsec}
{$linkframework CoreFoundation}
{$endc}

{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
unit MacOSAll;
interface

{$linkframework Carbon}

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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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

<<<<<<< HEAD
{unit AXConstants}
{$i AXConstants.pas}
{unit Accessibility}
{$i Accessibility.pas}
{unit ConditionalMacros}
{$i ConditionalMacros.pas}
{unit MacTypes}
{$i MacTypes.pas}
{unit MachineExceptions}
{$i MachineExceptions.pas}
{unit Math64}
{$i Math64.pas}
{unit MixedMode}
{$i MixedMode.pas}
{unit Multiprocessing}
{$i Multiprocessing.pas}
(* conflicts with FPC ObjC support
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit ObjC}
{$i ObjC.pas}
=======
{unit ObjCRuntime}
{$i ObjCRuntime.pas}
>>>>>>> graemeg/cpstrnew
=======
{unit ObjCRuntime}
{$i ObjCRuntime.pas}
>>>>>>> graemeg/cpstrnew
=======
{unit ObjCRuntime}
{$i ObjCRuntime.pas}
>>>>>>> graemeg/cpstrnew
=======
{unit ObjCRuntime}
{$i ObjCRuntime.pas}
>>>>>>> origin/cpstrnew
*)
{unit PEFBinaryFormat}
{$i PEFBinaryFormat.pas}
{unit PLStringFuncs}
{$i PLStringFuncs.pas}
{unit PMDefinitionsDeprecated}
{$i PMDefinitionsDeprecated.pas}
{unit PMPrintSettingsKeys}
{$i PMPrintSettingsKeys.pas}
{unit PMPrintingDialogExtensions}
{$i PMPrintingDialogExtensions.pas}
{unit QDCMCommon}
{$i QDCMCommon.pas}
{unit QLBase}
{$i QLBase.pas}
{unit QTML}
{$i QTML.pas}
{unit QuickTimeErrors}
{$i QuickTimeErrors.pas}
{unit SFNTLayoutTypes}
{$i SFNTLayoutTypes.pas}
{unit SFNTTypes}
{$i SFNTTypes.pas}
{unit ScalerStreamTypes}
{$i ScalerStreamTypes.pas}
{unit TextCommon}
{$i TextCommon.pas}
{unit Threads}
{$i Threads.pas}
{unit UTCUtils}
{$i UTCUtils.pas}
{unit UnicodeConverter}
{$i UnicodeConverter.pas}
{unit cblas}
{$i cblas.pas}
{unit cssmconfig}
{$i cssmconfig.pas}
{unit cssmerr}
{$i cssmerr.pas}
{unit cssmtype}
{$i cssmtype.pas}
{unit fenv}
{$i fenv.pas}
{unit fp}
{$i fp.pas}
{unit gliContexts}
{$i gliContexts.pas}
{unit kern_return}
{$i kern_return.pas}
{unit macgl}
{$i macgl.pas}
{unit macglext}
{$i macglext.pas}
{unit macglu}
{$i macglu.pas}
{unit mach_error}
{$i mach_error.pas}
{unit vBLAS}
{$i vBLAS.pas}
{unit vDSP}
{$i vDSP.pas}
{unit x509defs}
{$i x509defs.pas}
{unit xattr}
{$i xattr.pas}
{unit ABTypedefs}
{$i ABTypedefs.pas}
{unit AEDataModel}
{$i AEDataModel.pas}
{unit AEUserTermTypes}
{$i AEUserTermTypes.pas}
{unit AIFF}
{$i AIFF.pas}
{unit AVLTree}
{$i AVLTree.pas}
{unit AXActionConstants}
{$i AXActionConstants.pas}
{unit AXAttributeConstants}
{$i AXAttributeConstants.pas}
{unit AXErrors}
{$i AXErrors.pas}
{unit AXNotificationConstants}
{$i AXNotificationConstants.pas}
{unit AXRoleConstants}
{$i AXRoleConstants.pas}
{unit AXValueConstants}
{$i AXValueConstants.pas}
{unit AppleDiskPartitions}
{$i AppleDiskPartitions.pas}
{unit AudioUnitParameters}
{$i AudioUnitParameters.pas}
{unit Authorization}
{$i Authorization.pas}
{unit AuthorizationPlugin}
{$i AuthorizationPlugin.pas}
{unit AuthorizationTags}
{$i AuthorizationTags.pas}
{unit CFBase}
{$i CFBase.pas}
{unit CFBinaryHeap}
{$i CFBinaryHeap.pas}
{unit CFBitVector}
{$i CFBitVector.pas}
{unit CFByteOrders}
{$i CFByteOrders.pas}
{unit CFData}
{$i CFData.pas}
{unit CFDate}
{$i CFDate.pas}
{unit CFDictionary}
{$i CFDictionary.pas}
{unit CFError}
{$i CFError.pas}
{unit CFNetworkErrorss}
{$i CFNetworkErrorss.pas}
{unit CFNotificationCenter}
{$i CFNotificationCenter.pas}
{unit CFNumber}
{$i CFNumber.pas}
{unit CFSet}
{$i CFSet.pas}
{unit CFStringEncodingExt}
{$i CFStringEncodingExt.pas}
{unit CFTree}
{$i CFTree.pas}
{unit CGBase}
{$i CGBase.pas}
{unit CGErrors}
{$i CGErrors.pas}
{unit CGFunction}
{$i CGFunction.pas}
{unit CGGeometry}
{$i CGGeometry.pas}
{unit CGImageProperties}
{$i CGImageProperties.pas}
{unit CGLProfiler}
{$i CGLProfiler.pas}
{unit CGLProfilerFunctionEnums}
{$i CGLProfilerFunctionEnums.pas}
{unit CGLRenderers}
{$i CGLRenderers.pas}
{unit CGLTypes}
{$i CGLTypes.pas}
{unit CGPDFObject}
{$i CGPDFObject.pas}
{unit CGPDFOperatorTable}
{$i CGPDFOperatorTable.pas}
{unit CGPDFScanner}
{$i CGPDFScanner.pas}
{unit CGPDFStream}
{$i CGPDFStream.pas}
{unit CGPDFString}
{$i CGPDFString.pas}
{unit CGSession}
{$i CGSession.pas}
{unit CGWindowLevels}
{$i CGWindowLevels.pas}
{unit CSIdentityAuthority}
{$i CSIdentityAuthority.pas}
{unit CTFontManagerErrors}
{$i CTFontManagerErrors.pas}
{unit CTFontTraits}
{$i CTFontTraits.pas}
{unit CVBase}
{$i CVBase.pas}
{unit CVHostTime}
{$i CVHostTime.pas}
{unit CVReturns}
{$i CVReturns.pas}
{unit Collections}
{$i Collections.pas}
{unit CoreAudioTypes}
{$i CoreAudioTypes.pas}
{unit DHCPClientPreferences}
{$i DHCPClientPreferences.pas}
{unit DateTimeUtils}
{$i DateTimeUtils.pas}
{unit Debugging}
{$i Debugging.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit DictionaryServices}
{$i DictionaryServices.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit DigitalHubRegistry}
{$i DigitalHubRegistry.pas}
{unit DriverServices}
{$i DriverServices.pas}
{unit DriverSynchronization}
{$i DriverSynchronization.pas}
{unit Endian}
{$i Endian.pas}
{unit FileTypesAndCreators}
{$i FileTypesAndCreators.pas}
{unit Finder}
{$i Finder.pas}
{unit FixMath}
{$i FixMath.pas}
{unit GestaltEqu}
{$i GestaltEqu.pas}
{unit HFSVolumes}
{$i HFSVolumes.pas}
{unit HIGeometry}
{$i HIGeometry.pas}
{unit HITextUtils}
{$i HITextUtils.pas}
{unit HostTime}
{$i HostTime.pas}
{unit ICACamera}
{$i ICACamera.pas}
{unit IOKitReturn}
{$i IOKitReturn.pas}
{unit IconStorage}
{$i IconStorage.pas}
{unit IntlResources}
{$i IntlResources.pas}
{unit Keyboards}
{$i Keyboards.pas}
{unit LSQuarantine}
{$i LSQuarantine.pas}
{unit MIDIServices}
{$i MIDIServices.pas}
{unit MIDIThruConnection}
{$i MIDIThruConnection.pas}
{unit MacErrors}
{$i MacErrors.pas}
{unit MacLocales}
{$i MacLocales.pas}
{unit MacMemory}
{$i MacMemory.pas}
{unit MacOSXPosix}
{$i MacOSXPosix.pas}
{unit MultiProcessingInfo}
{$i MultiProcessingInfo.pas}
{unit NumberFormatting}
{$i NumberFormatting.pas}
{unit OSAComp}
{$i OSAComp.pas}
{unit OSUtils}
{$i OSUtils.pas}
{unit OpenTransport}
{$i OpenTransport.pas}
{unit OpenTransportProviders}
{$i OpenTransportProviders.pas}
{unit Power}
{$i Power.pas}
{unit QuickdrawText}
{$i QuickdrawText.pas}
{unit QuickdrawTypes}
{$i QuickdrawTypes.pas}
{unit SCDynamicStoreKey}
{$i SCDynamicStoreKey.pas}
{unit SCNetwork}
{$i SCNetwork.pas}
{unit SCSI}
{$i SCSI.pas}
{unit SCSchemaDefinitions}
{$i SCSchemaDefinitions.pas}
{unit Script}
{$i Script.pas}
{unit SecBase}
{$i SecBase.pas}
{unit StringCompare}
{$i StringCompare.pas}
{unit SystemConfiguration}
{$i SystemConfiguration.pas}
{unit TextEdit}
{$i TextEdit.pas}
{unit TextEncodingConverter}
{$i TextEncodingConverter.pas}
{unit TextEncodingPlugin}
{$i TextEncodingPlugin.pas}
{unit TextUtils}
{$i TextUtils.pas}
{unit Timer}
{$i Timer.pas}
{unit UTCoreTypes}
{$i UTCoreTypes.pas}
{unit UnicodeUtilities}
{$i UnicodeUtilities.pas}
{unit UniversalAccess}
{$i UniversalAccess.pas}
{unit Video}
{$i Video.pas}
{unit WSTypes}
{$i WSTypes.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit acl}
{$i acl.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit certextensions}
{$i certextensions.pas}
{unit cssmapple}
{$i cssmapple.pas}
{unit cssmkrapi}
{$i cssmkrapi.pas}
{unit gliDispatch}
{$i gliDispatch.pas}
{unit gluContext}
{$i gluContext.pas}
{unit ABGlobals}
{$i ABGlobals.pas}
{unit AEMach}
{$i AEMach.pas}
{unit AXTextAttributedString}
{$i AXTextAttributedString.pas}
{unit AXValue}
{$i AXValue.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit AudioHardwareBase}
{$i AudioHardwareBase.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit AuthSession}
{$i AuthSession.pas}
{unit BackupCore}
{$i BackupCore.pas}
{unit CFArray}
{$i CFArray.pas}
{unit CFBag}
{$i CFBag.pas}
{unit CFCharacterSet}
{$i CFCharacterSet.pas}
{unit CFLocale}
{$i CFLocale.pas}
{unit CFNumberFormatter}
{$i CFNumberFormatter.pas}
{unit CFString}
{$i CFString.pas}
{unit CFStringTokenizer}
{$i CFStringTokenizer.pas}
{unit CFTimeZone}
{$i CFTimeZone.pas}
{unit CFUUID}
{$i CFUUID.pas}
{unit CGAffineTransforms}
{$i CGAffineTransforms.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CGImageMetadata}
{$i CGImageMetadata.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CGLCurrent}
{$i CGLCurrent.pas}
{unit CGLDevice}
{$i CGLDevice.pas}
{unit CGPDFArray}
{$i CGPDFArray.pas}
{unit CGPDFContentStream}
{$i CGPDFContentStream.pas}
{unit CGPDFDictionary}
{$i CGPDFDictionary.pas}
{unit CGPath}
{$i CGPath.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CSIdentityBase}
{$i CSIdentityBase.pas}
{unit CTFontDescriptor}
{$i CTFontDescriptor.pas}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CTFontDescriptor}
{$i CTFontDescriptor.pas}
{unit CTFontManager}
{$i CTFontManager.pas}
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
{unit CTParagraphStyle}
{$i CTParagraphStyle.pas}
{unit CTTextTab}
{$i CTTextTab.pas}
{unit CVBuffer}
{$i CVBuffer.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CaptiveNetwork}
{$i CaptiveNetwork.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit ColorSyncProfile}
{$i ColorSyncProfile.pas}
{unit ColorSyncTransform}
{$i ColorSyncTransform.pas}
{unit Events}
{$i Events.pas}
{unit HITextLengthFilter}
{$i HITextLengthFilter.pas}
{unit IOSurfaceAPI}
{$i IOSurfaceAPI.pas}
{unit MDItem}
{$i MDItem.pas}
{unit MDQuery}
{$i MDQuery.pas}
{unit MDSchema}
{$i MDSchema.pas}
{unit MIDISetup}
{$i MIDISetup.pas}
{unit MacOpenGL}
{$i MacOpenGL.pas}
{unit Notification}
{$i Notification.pas}
{unit PMDefinitions}
{$i PMDefinitions.pas}
{unit PMErrors}
{$i PMErrors.pas}
{unit Palettes}
{$i Palettes.pas}
{unit PictUtils}
{$i PictUtils.pas}
{unit QDOffscreen}
{$i QDOffscreen.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit SKAnalysis}
{$i SKAnalysis.pas}
{unit SKSummary}
{$i SKSummary.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit Scrap}
{$i Scrap.pas}
{unit SecTrust}
{$i SecTrust.pas}
{unit TypeSelect}
{$i TypeSelect.pas}
{unit ABAddressBook}
{$i ABAddressBook.pas}
{unit CFAttributedString}
{$i CFAttributedString.pas}
{unit CFCalendar}
{$i CFCalendar.pas}
{unit CFDateFormatter}
{$i CFDateFormatter.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CFFileSecurity}
{$i CFFileSecurity.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CFRunLoop}
{$i CFRunLoop.pas}
{unit CFSocket}
{$i CFSocket.pas}
{unit CSIdentity}
{$i CSIdentity.pas}
{unit CSIdentityQuery}
{$i CSIdentityQuery.pas}
{unit CTFontCollection}
{$i CTFontCollection.pas}
{unit ColorSyncDevice}
{$i ColorSyncDevice.pas}
{unit DASession}
{$i DASession.pas}
{unit FSEvents}
{$i FSEvents.pas}
{unit SCNetworkConnection}
{$i SCNetworkConnection.pas}
{unit SCNetworkReachability}
{$i SCNetworkReachability.pas}
{unit ABActions}
{$i ABActions.pas}
{unit AudioHardware}
{$i AudioHardware.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit AudioHardwareDeprecated}
{$i AudioHardwareDeprecated.pas}
{unit CFFileDescriptor}
{$i CFFileDescriptor.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CFMachPort}
{$i CFMachPort.pas}
{unit CFMessagePort}
{$i CFMessagePort.pas}
{unit CGRemoteOperation}
{$i CGRemoteOperation.pas}
{unit DADisk}
{$i DADisk.pas}
{unit Files}
{$i Files.pas}
{unit Folders}
{$i Folders.pas}
{unit ICAApplication}
{$i ICAApplication.pas}
{unit ICADevice}
{$i ICADevice.pas}
{unit LowMem}
{$i LowMem.pas}
{unit Processes}
{$i Processes.pas}
{unit Resources}
{$i Resources.pas}
{unit SpeechRecognition}
{$i SpeechRecognition.pas}
{unit SystemSound}
{$i SystemSound.pas}
{unit ATSTypes}
{$i ATSTypes.pas}
{unit AXUIElement}
{$i AXUIElement.pas}
{unit Aliases}
{$i Aliases.pas}
{unit AppleHelp}
{$i AppleHelp.pas}
{unit CFURL}
{$i CFURL.pas}
{unit CFURLAccess}
{$i CFURLAccess.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CFURLEnumerator}
{$i CFURLEnumerator.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CFUserNotification}
{$i CFUserNotification.pas}
{unit CFXMLNode}
{$i CFXMLNode.pas}
{unit CFXMLParser}
{$i CFXMLParser.pas}
{unit CGDataConsumer}
{$i CGDataConsumer.pas}
{unit CGDataProvider}
{$i CGDataProvider.pas}
{unit CGEventTypes}
{$i CGEventTypes.pas}
{unit CGFont}
{$i CGFont.pas}
{unit CGPDFDocument}
{$i CGPDFDocument.pas}
{unit CGPDFPage}
{$i CGPDFPage.pas}
{unit CGPSConverter}
{$i CGPSConverter.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CTFontManager}
{$i CTFontManager.pas}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CTFont}
{$i CTFont.pas}
{unit CTGlyphInfo}
{$i CTGlyphInfo.pas}
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
{unit CarbonEventsCore}
{$i CarbonEventsCore.pas}
{unit ColorSyncDeprecated}
{$i ColorSyncDeprecated.pas}
{unit Components}
{$i Components.pas}
{unit Fonts}
{$i Fonts.pas}
{unit IconsCore}
{$i IconsCore.pas}
{unit InternetConfig}
{$i InternetConfig.pas}
{unit LSInfo}
{$i LSInfo.pas}
{unit LSOpen}
{$i LSOpen.pas}
{unit LSSharedFileList}
{$i LSSharedFileList.pas}
{unit MDLineage}
{$i MDLineage.pas}
{unit PMCoreDeprecated}
{$i PMCoreDeprecated.pas}
{unit Pasteboard}
{$i Pasteboard.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit SKDocument}
{$i SKDocument.pas}
{unit SKIndex}
{$i SKIndex.pas}
{unit SKSearch}
{$i SKSearch.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit SpeechSynthesis}
{$i SpeechSynthesis.pas}
{unit TextInputSources}
{$i TextInputSources.pas}
{unit ToolUtils}
{$i ToolUtils.pas}
{unit TranslationExtensions}
{$i TranslationExtensions.pas}
{unit TranslationServices}
{$i TranslationServices.pas}
{unit UTType}
{$i UTType.pas}
{unit WSMethodInvocation}
{$i WSMethodInvocation.pas}
{unit WSProtocolHandler}
{$i WSProtocolHandler.pas}
{unit ABPeoplePicker}
{$i ABPeoplePicker.pas}
{unit AEInteraction}
{$i AEInteraction.pas}
{unit ATSLayoutTypes}
{$i ATSLayoutTypes.pas}
{unit ATSUnicodeTypes}
{$i ATSUnicodeTypes.pas}
{unit AudioComponents}
{$i AudioComponents.pas}
{unit CFBundle}
{$i CFBundle.pas}
{unit CFHTTPMessage}
{$i CFHTTPMessage.pas}
{unit CFPlugIn}
{$i CFPlugIn.pas}
{unit CFPlugInCOM}
{$i CFPlugInCOM.pas}
{unit CFStream}
{$i CFStream.pas}
{unit CGColorSpace}
{$i CGColorSpace.pas}
{unit CGEventSource}
{$i CGEventSource.pas}
{unit CGGradient}
{$i CGGradient.pas}
{unit CGImage}
{$i CGImage.pas}
{unit CGImageSource}
{$i CGImageSource.pas}
{unit CGShading}
{$i CGShading.pas}
{unit CGWindow}
{$i CGWindow.pas}
{unit CMCalibrator}
{$i CMCalibrator.pas}
{unit CVImageBuffer}
{$i CVImageBuffer.pas}
{unit CVOpenGLBuffer}
{$i CVOpenGLBuffer.pas}
{unit CVOpenGLBufferPool}
{$i CVOpenGLBufferPool.pas}
{unit CVOpenGLTexture}
{$i CVOpenGLTexture.pas}
{unit CVOpenGLTextureCache}
{$i CVOpenGLTextureCache.pas}
{unit CVPixelBuffer}
{$i CVPixelBuffer.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CVPixelBufferIOSurface}
{$i CVPixelBufferIOSurface.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit CVPixelBufferPool}
{$i CVPixelBufferPool.pas}
{unit CVPixelFormatDescription}
{$i CVPixelFormatDescription.pas}
{unit CodeFragments}
{$i CodeFragments.pas}
{unit ColorPicker}
{$i ColorPicker.pas}
{unit ColorSyncCMM}
{$i ColorSyncCMM.pas}
{unit FontSync}
{$i FontSync.pas}
{unit HIObject}
{$i HIObject.pas}
{unit KeychainCore}
{$i KeychainCore.pas}
{unit KeychainHI}
{$i KeychainHI.pas}
{unit MDExternalDatastore}
{$i MDExternalDatastore.pas}
{unit MDImporter}
{$i MDImporter.pas}
{unit MIDIDriver}
{$i MIDIDriver.pas}
{unit NSLCore}
{$i NSLCore.pas}
{unit OpenTransportProtocol}
{$i OpenTransportProtocol.pas}
{unit QLThumbnailImage}
{$i QLThumbnailImage.pas}
{unit Translation}
{$i Translation.pas}
{unit URLAccess}
{$i URLAccess.pas}
{unit ATSUnicodeDirectAccess}
{$i ATSUnicodeDirectAccess.pas}
{unit ATSUnicodeDrawing}
{$i ATSUnicodeDrawing.pas}
{unit ATSUnicodeFlattening}
{$i ATSUnicodeFlattening.pas}
{unit ATSUnicodeFonts}
{$i ATSUnicodeFonts.pas}
{unit ATSUnicodeGlyphs}
{$i ATSUnicodeGlyphs.pas}
{unit ATSUnicodeObjects}
{$i ATSUnicodeObjects.pas}
{unit AUComponent}
{$i AUComponent.pas}
{unit AppleEvents}
{$i AppleEvents.pas}
{unit AudioCodecs}
{$i AudioCodecs.pas}
{unit AudioOutputUnit}
{$i AudioOutputUnit.pas}
{unit AudioUnitProperties}
{$i AudioUnitProperties.pas}
{unit AuthorizationDB}
{$i AuthorizationDB.pas}
{unit CFFTPStream}
{$i CFFTPStream.pas}
{unit CFHTTPAuthentication}
{$i CFHTTPAuthentication.pas}
{unit CFHTTPStream}
{$i CFHTTPStream.pas}
{unit CFHost}
{$i CFHost.pas}
{unit CFNetDiagnostics}
{$i CFNetDiagnostics.pas}
{unit CFNetServices}
{$i CFNetServices.pas}
{unit CFPropertyList}
{$i CFPropertyList.pas}
{unit CFProxySupport}
{$i CFProxySupport.pas}
{unit CFSocketStream}
{$i CFSocketStream.pas}
{unit CGColor}
{$i CGColor.pas}
{unit CGContext}
{$i CGContext.pas}
{unit CGDirectDisplay}
{$i CGDirectDisplay.pas}
{unit CGDirectPalette}
{$i CGDirectPalette.pas}
{unit CGDisplayConfiguration}
{$i CGDisplayConfiguration.pas}
{unit CGDisplayFades}
{$i CGDisplayFades.pas}
{unit CGEvent}
{$i CGEvent.pas}
{unit CGGLContext}
{$i CGGLContext.pas}
{unit CGImageDestination}
{$i CGImageDestination.pas}
{unit CGLayer}
{$i CGLayer.pas}
{unit CGPDFContext}
{$i CGPDFContext.pas}
{unit CGPattern}
{$i CGPattern.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit CTFont}
{$i CTFont.pas}
{unit CTFrame}
{$i CTFrame.pas}
{unit CTGlyphInfo}
{$i CTGlyphInfo.pas}
=======
{unit CTFrame}
{$i CTFrame.pas}
>>>>>>> graemeg/cpstrnew
=======
{unit CTFrame}
{$i CTFrame.pas}
>>>>>>> graemeg/cpstrnew
=======
{unit CTFrame}
{$i CTFrame.pas}
>>>>>>> graemeg/cpstrnew
=======
{unit CTFrame}
{$i CTFrame.pas}
>>>>>>> origin/cpstrnew
{unit CTLine}
{$i CTLine.pas}
{unit CTRun}
{$i CTRun.pas}
{unit CTStringAttributes}
{$i CTStringAttributes.pas}
{unit CTTypesetter}
{$i CTTypesetter.pas}
{unit CVDisplayLink}
{$i CVDisplayLink.pas}
{unit Dialogs}
{$i Dialogs.pas}
{unit Displays}
{$i Displays.pas}
{unit Drag}
{$i Drag.pas}
{unit DrawSprocket}
{$i DrawSprocket.pas}
{unit HIArchive}
{$i HIArchive.pas}
{unit ImageCompression}
{$i ImageCompression.pas}
{unit MusicDevice}
{$i MusicDevice.pas}
{unit NSL}
{$i NSL.pas}
{unit PMApplication}
{$i PMApplication.pas}
{unit PMApplicationDeprecated}
{$i PMApplicationDeprecated.pas}
{unit PMCore}
{$i PMCore.pas}
{unit QDPictToCGContext}
{$i QDPictToCGContext.pas}
{unit QLGenerator}
{$i QLGenerator.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{unit QLThumbnail}
{$i QLThumbnail.pas}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{unit Quickdraw}
{$i Quickdraw.pas}
{unit SCDynamicStore}
{$i SCDynamicStore.pas}
{unit SCDynamicStoreCopyDHCPInfos}
{$i SCDynamicStoreCopyDHCPInfos.pas}
{unit SCDynamicStoreCopySpecific}
{$i SCDynamicStoreCopySpecific.pas}
{unit SCPreferences}
{$i SCPreferences.pas}
{unit SCPreferencesPath}
{$i SCPreferencesPath.pas}
{unit SCPreferencesSetSpecific}
{$i SCPreferencesSetSpecific.pas}
{unit Sound}
{$i Sound.pas}
{unit AEHelpers}
{$i AEHelpers.pas}
{unit AEObjects}
{$i AEObjects.pas}
{unit AEPackObject}
{$i AEPackObject.pas}
{unit AERegistry}
{$i AERegistry.pas}
{unit ASRegistry}
{$i ASRegistry.pas}
{unit ATSFont}
{$i ATSFont.pas}
{unit Appearance}
{$i Appearance.pas}
{unit CFPreferences}
{$i CFPreferences.pas}
{unit CGBitmapContext}
{$i CGBitmapContext.pas}
{unit CTFramesetter}
{$i CTFramesetter.pas}
{unit CoreFoundation}
{$i CoreFoundation.pas}
{unit CoreGraphics}
{$i CoreGraphics.pas}
{unit CoreText}
{$i CoreText.pas}
{unit Dictionary}
{$i Dictionary.pas}
{unit FontPanel}
{$i FontPanel.pas}
{unit LanguageAnalysis}
{$i LanguageAnalysis.pas}
{unit Menus}
{$i Menus.pas}
{unit Movies}
{$i Movies.pas}
{unit MoviesFormat}
{$i MoviesFormat.pas}
{unit OSA}
{$i OSA.pas}
{unit OSAGeneric}
{$i OSAGeneric.pas}
{unit PMPrintAETypes}
{$i PMPrintAETypes.pas}
{unit QuickTimeMusic}
{$i QuickTimeMusic.pas}
{unit SCNetworkConfiguration}
{$i SCNetworkConfiguration.pas}
{unit AppleScript}
{$i AppleScript.pas}
{unit CarbonEvents}
{$i CarbonEvents.pas}
{unit FinderRegistry}
{$i FinderRegistry.pas}
{unit HIShape}
{$i HIShape.pas}
{unit HIToolbar}
{$i HIToolbar.pas}
{unit Icons}
{$i Icons.pas}
{unit ImageCodec}
{$i ImageCodec.pas}
{unit MacApplication}
{$i MacApplication.pas}
{unit MacWindows}
{$i MacWindows.pas}
{unit MediaHandlers}
{$i MediaHandlers.pas}
{unit Navigation}
{$i Navigation.pas}
{unit QuickTimeComponents}
{$i QuickTimeComponents.pas}
{unit QuickTimeStreaming}
{$i QuickTimeStreaming.pas}
{unit QuickTimeVR}
{$i QuickTimeVR.pas}
{unit QuickTimeVRFormat}
{$i QuickTimeVRFormat.pas}
{unit TextServices}
{$i TextServices.pas}
{unit ASDebugging}
{$i ASDebugging.pas}
{unit AudioUnitCarbonViews}
{$i AudioUnitCarbonViews.pas}
{unit Controls}
{$i Controls.pas}
{unit HIAccessibility}
{$i HIAccessibility.pas}
{unit HIButtonViews}
{$i HIButtonViews.pas}
{unit HIClockView}
{$i HIClockView.pas}
{unit HIContainerViews}
{$i HIContainerViews.pas}
{unit HILittleArrows}
{$i HILittleArrows.pas}
{unit HIPopupButton}
{$i HIPopupButton.pas}
{unit HIProgressViews}
{$i HIProgressViews.pas}
{unit HIRelevanceBar}
{$i HIRelevanceBar.pas}
{unit HISeparator}
{$i HISeparator.pas}
{unit HITabbedView}
{$i HITabbedView.pas}
{unit HITheme}
{$i HITheme.pas}
{unit HIToolboxDebugging}
{$i HIToolboxDebugging.pas}
{unit HIView}
{$i HIView.pas}
{unit HIWindowViews}
{$i HIWindowViews.pas}
{unit HTMLRendering}
{$i HTMLRendering.pas}
{unit Lists}
{$i Lists.pas}
{unit MacHelp}
{$i MacHelp.pas}
{unit MacTextEditor}
{$i MacTextEditor.pas}
{unit QTSMovie}
{$i QTSMovie.pas}
{unit QTStreamingComponents}
{$i QTStreamingComponents.pas}
{unit TSMTE}
{$i TSMTE.pas}
{unit ControlDefinitions}
{$i ControlDefinitions.pas}
{unit HIComboBox}
{$i HIComboBox.pas}
{unit HIDataBrowser}
{$i HIDataBrowser.pas}
{unit HIDisclosureViews}
{$i HIDisclosureViews.pas}
{unit HIImageViews}
{$i HIImageViews.pas}
{unit HIMenuView}
{$i HIMenuView.pas}
{unit HIMovieView}
{$i HIMovieView.pas}
{unit HIScrollView}
{$i HIScrollView.pas}
{unit HISearchField}
{$i HISearchField.pas}
{unit HISegmentedView}
{$i HISegmentedView.pas}
{unit HISlider}
{$i HISlider.pas}
{unit HITextViews}
{$i HITextViews.pas}
{unit IBCarbonRuntime}
{$i IBCarbonRuntime.pas}
{unit HIToolbox}
{$i HIToolbox.pas}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

implementation

{implementation of unit MixedMode}


{$R-}

function NewRoutineDescriptor( theProc: ProcPtr; theProcInfo: ProcInfoType; theISA: ISAType ): UniversalProcPtr; inline;
begin
	NewRoutineDescriptor := UniversalProcPtr(theProc);
end;

procedure DisposeRoutineDescriptor( theUPP: UniversalProcPtr ); inline;
begin
end;

{implementation of unit cssmerr}

{$ifc TARGET_OS_MAC}


function CSSM_ERRCODE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRCODE:=(arg - CSSM_BASE_ERROR) and (CSSM_ERRORCODE_MODULE_EXTENT - 1)
end;

function CSSM_ERRBASE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRBASE:=((arg - CSSM_BASE_ERROR) and not(CSSM_ERRORCODE_MODULE_EXTENT - 1)) + CSSM_BASE_ERROR
end;

function CSSM_ERR_IS_CONVERTIBLE(arg: UInt32): Boolean; inline;
begin
  CSSM_ERR_IS_CONVERTIBLE:=CSSM_ERRCODE(arg) < CSSM_ERRORCODE_COMMON_EXTENT
end;

function CSSM_ERR_TAG(code, base: UInt32): UInt32; inline;
begin
  CSSM_ERR_TAG:=CSSM_ERRCODE(code) + base
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit mach_error}
=======

implementation

{implementation of unit MixedMode}


{$R-}

function NewRoutineDescriptor( theProc: ProcPtr; theProcInfo: ProcInfoType; theISA: ISAType ): UniversalProcPtr; inline;
begin
	NewRoutineDescriptor := UniversalProcPtr(theProc);
end;

procedure DisposeRoutineDescriptor( theUPP: UniversalProcPtr ); inline;
begin
end;

{implementation of unit cssmerr}

{$ifc TARGET_OS_MAC}


function CSSM_ERRCODE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRCODE:=(arg - CSSM_BASE_ERROR) and (CSSM_ERRORCODE_MODULE_EXTENT - 1)
end;

function CSSM_ERRBASE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRBASE:=((arg - CSSM_BASE_ERROR) and not(CSSM_ERRORCODE_MODULE_EXTENT - 1)) + CSSM_BASE_ERROR
end;

function CSSM_ERR_IS_CONVERTIBLE(arg: UInt32): Boolean; inline;
begin
  CSSM_ERR_IS_CONVERTIBLE:=CSSM_ERRCODE(arg) < CSSM_ERRORCODE_COMMON_EXTENT
end;

function CSSM_ERR_TAG(code, base: UInt32): UInt32; inline;
begin
  CSSM_ERR_TAG:=CSSM_ERRCODE(code) + base
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit mach_error}


{$push}
{$R-,Q-}

function err_system(x: mach_error_t): mach_error_t; inline;
begin
  err_system:=(((x) and $3f) shl 26)
end;

function err_sub(x: mach_error_t): mach_error_t; inline;
begin
  err_sub:=(((x) shr 14) and $fff)
end;

>>>>>>> graemeg/cpstrnew

function err_get_system(err: mach_error_t): mach_error_t; inline;
begin
  err_get_system:=(((err) shr 26) and $3f)
end;

<<<<<<< HEAD
{$push}
{$R-,Q-}

function err_system(x: mach_error_t): mach_error_t; inline;
begin
  err_system:=(((x) and $3f) shl 26)
end;

function err_sub(x: mach_error_t): mach_error_t; inline;
begin
  err_sub:=(((x) shr 14) and $fff)
end;


function err_get_system(err: mach_error_t): mach_error_t; inline;
begin
  err_get_system:=(((err) shr 26) and $3f)
end;

function err_get_sub(err: mach_error_t): mach_error_t; inline;
begin
  err_get_sub:=(((err) shr 14) and $fff)
end;

function err_get_code(err: mach_error_t): mach_error_t; inline;
begin
  err_get_code:=((err) and $3fff)
end;


function unix_err(errno: SInt32): mach_error_t; inline;
begin
  unix_err:=err_kern or (((3) and $fff) shl 14) or errno;
end;

=======
function err_get_sub(err: mach_error_t): mach_error_t; inline;
begin
  err_get_sub:=(((err) shr 14) and $fff)
end;

function err_get_code(err: mach_error_t): mach_error_t; inline;
begin
  err_get_code:=((err) and $3fff)
end;


function unix_err(errno: SInt32): mach_error_t; inline;
begin
  unix_err:=err_kern or (((3) and $fff) shl 14) or errno;
end;

>>>>>>> graemeg/cpstrnew
{$pop}

{implementation of unit CFByteOrders}


{$R-}

function CFByteOrderGetCurrent: CFByteOrder; inline;
	var
		x: UInt32 = (CFByteOrderBigEndian shl 24) or CFByteOrderLittleEndian;
begin
	CFByteOrderGetCurrent := CFByteOrder(UInt8Ptr(@x)^);
end;

function CFSwapInt16( arg: UInt16 ): UInt16; inline;
begin
	CFSwapInt16 := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function CFSwapInt32( arg: UInt32 ): UInt32; inline;
begin
    CFSwapInt32 := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function CFSwapInt64( arg: UInt64 ): UInt64; inline;
begin
	CFSwapInt64 := (UInt64(CFSwapInt32( arg and $FFFFFFFF )) shl 32) or CFSwapInt32( (arg shr 32) and $FFFFFFFF );
end;

{$ifc TARGET_RT_BIG_ENDIAN}
function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16BigToHost := arg;
end;

function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32BigToHost := arg;
end;

function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64BigToHost := arg;
end;

function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToBig := arg;
end;

function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToBig := arg;
end;

function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToBig := arg;
end;

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16LittleToHost := CFSwapInt16(arg);
end;

function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32LittleToHost := CFSwapInt32(arg);
end;

function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64LittleToHost := CFSwapInt64(arg);
end;

function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToLittle := CFSwapInt16(arg);
end;

function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToLittle := CFSwapInt32(arg);
end;

function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToLittle := CFSwapInt64(arg);
end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloat32HostToSwapped := CFSwappedFloat32(arg);
end;

function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloat32SwappedToHost := Float32(arg);
end;

function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertFloat64HostToSwapped := CFSwappedFloat64(arg);
end;

function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertFloat64SwappedToHost := Float64(arg);
end;

function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped := CFSwappedFloat32(arg);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(arg);
end;

function CFConvertDoubleHostToSwapped( arg: Float64): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped := CFSwappedFloat64(arg);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(arg);
end;

{$elsec}

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16LittleToHost := arg;
end;

function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32LittleToHost := arg;
end;

function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64LittleToHost := arg;
end;

function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToLittle := arg;
end;

function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToLittle := arg;
end;

function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToLittle := arg;
end;

function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16BigToHost := CFSwapInt16(arg);
end;

function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32BigToHost := CFSwapInt32(arg);
end;

function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64BigToHost := CFSwapInt64(arg);
end;

function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToBig := CFSwapInt16(arg);
end;

function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToBig := CFSwapInt32(arg);
end;

function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToBig := CFSwapInt64(arg);
end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloat32HostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloat32SwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertFloat64HostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertFloat64SwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;

function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;
{$endc}


{implementation of unit CGGeometry}

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
end;


function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
end;


{implementation of unit CGGeometry}

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
<<<<<<< HEAD
  CFConvertFloatSwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;
{$endc}


{implementation of unit CGGeometry}

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGVectorMake(dx: CGFloat; dy: CGFloat): CGVector; inline;
begin
  CGVectorMake.dx := dx;
  CGVectorMake.dy := dy;
=======

implementation

{implementation of unit MixedMode}


{$R-}

function NewRoutineDescriptor( theProc: ProcPtr; theProcInfo: ProcInfoType; theISA: ISAType ): UniversalProcPtr; inline;
begin
	NewRoutineDescriptor := UniversalProcPtr(theProc);
>>>>>>> graemeg/cpstrnew
end;

procedure DisposeRoutineDescriptor( theUPP: UniversalProcPtr ); inline;
begin
end;

<<<<<<< HEAD
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
=======
>>>>>>> graemeg/cpstrnew
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
end;


function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
end;


{implementation of unit Endian}

{$R-}

function Endian16_Swap( arg: UInt16 ): UInt16; inline;
begin
	Endian16_Swap := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function Endian32_Swap( arg: UInt32 ): UInt32; inline;
begin
    Endian32_Swap := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function Endian64_Swap_Pascal( arg: UInt64 ): UInt64; inline;
begin
	Endian64_Swap_Pascal := (Endian32_Swap( arg and $FFFFFFFF ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
end;

function Endian64_Swap( arg: UInt64 ): UInt64; inline;
begin
	Endian64_Swap := Endian64_Swap_Pascal(arg);
end;

function EndianS16_Swap( arg: SInt16 ): SInt16; inline;
begin
	EndianS16_Swap := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function EndianS32_Swap( arg: SInt32 ): SInt32; inline;
begin
    EndianS32_Swap := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function EndianS64_Swap( arg: SInt64 ): SInt64; inline;
begin
	EndianS64_Swap := (SInt64( Endian32_Swap( arg and $FFFFFFFF ) ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
end;

{$ifc TARGET_RT_BIG_ENDIAN}
function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoN := arg;
end;

function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoB := arg;
end;

function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoN := arg;
end;

function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoB := arg;
end;
=======
{implementation of unit cssmerr}

{$ifc TARGET_OS_MAC}


function CSSM_ERRCODE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRCODE:=(arg - CSSM_BASE_ERROR) and (CSSM_ERRORCODE_MODULE_EXTENT - 1)
end;

function CSSM_ERRBASE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRBASE:=((arg - CSSM_BASE_ERROR) and not(CSSM_ERRORCODE_MODULE_EXTENT - 1)) + CSSM_BASE_ERROR
end;

function CSSM_ERR_IS_CONVERTIBLE(arg: UInt32): Boolean; inline;
begin
  CSSM_ERR_IS_CONVERTIBLE:=CSSM_ERRCODE(arg) < CSSM_ERRORCODE_COMMON_EXTENT
end;

function CSSM_ERR_TAG(code, base: UInt32): UInt32; inline;
begin
  CSSM_ERR_TAG:=CSSM_ERRCODE(code) + base
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit mach_error}


{$push}
{$R-,Q-}

function err_system(x: mach_error_t): mach_error_t; inline;
begin
  err_system:=(((x) and $3f) shl 26)
end;

function err_sub(x: mach_error_t): mach_error_t; inline;
begin
  err_sub:=(((x) shr 14) and $fff)
end;


function err_get_system(err: mach_error_t): mach_error_t; inline;
begin
  err_get_system:=(((err) shr 26) and $3f)
end;

function err_get_sub(err: mach_error_t): mach_error_t; inline;
begin
  err_get_sub:=(((err) shr 14) and $fff)
end;

function err_get_code(err: mach_error_t): mach_error_t; inline;
begin
  err_get_code:=((err) and $3fff)
end;


function unix_err(errno: SInt32): mach_error_t; inline;
begin
  unix_err:=err_kern or (((3) and $fff) shl 14) or errno;
end;

{$pop}

{implementation of unit CFByteOrders}
>>>>>>> graemeg/cpstrnew

function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoN := arg;
end;

function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoB := arg;
end;

function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoN := arg;
end;

function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoB := arg;
end;

function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoN := arg;
end;

function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoB := arg;
end;

function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoN := arg;
end;

function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoB := arg;
end;

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoN := EndianS16_Swap(arg);
end;

function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoL := EndianS16_Swap(arg);
end;

function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoN := Endian16_Swap(arg);
end;

function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoL := Endian16_Swap(arg);
end;

function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoN := EndianS32_Swap(arg);
end;

function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoL := EndianS32_Swap(arg);
end;

function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoN := Endian32_Swap(arg);
end;

function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoL := Endian32_Swap(arg);
end;


function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoN := EndianS64_Swap(arg);
end;

function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoL := EndianS64_Swap(arg);
end;

function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoN := Endian64_Swap(arg);
end;

function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoL := Endian64_Swap(arg);
end;

{$elsec}
function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoN := EndianS16_Swap(arg);
end;

function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoB := EndianS16_Swap(arg);
end;

function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoN := Endian16_Swap(arg);
end;

function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoB := Endian16_Swap(arg);
end;

function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoN := EndianS32_Swap(arg);
end;

function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoB := EndianS32_Swap(arg);
end;

function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoN := Endian32_Swap(arg);
end;

function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoB := Endian32_Swap(arg);
end;


function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoN := EndianS64_Swap(arg);
end;

function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoB := EndianS64_Swap(arg);
end;

function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoN := Endian64_Swap(arg);
end;

function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoB := Endian64_Swap(arg);
end;

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoN := arg;
end;

function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoL := arg;
end;

function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoN := arg;
end;

function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoL := arg;
end;

function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoN := arg;
end;

function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoL := arg;
end;

function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoN := arg;
end;

function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoL := arg;
end;

function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoN := arg;
end;

function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoL := arg;
end;

<<<<<<< HEAD
function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoN := arg;
end;

function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoL := arg;
end;

{$endc}

function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoB:=EndianS16_Swap(arg);
=======
function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;
{$endc}


{implementation of unit CGGeometry}

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
>>>>>>> graemeg/cpstrnew
end;
<<<<<<< HEAD

<<<<<<< HEAD
function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;
=======

function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
end;


{implementation of unit Endian}

{$R-}
>>>>>>> graemeg/cpstrnew

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
<<<<<<< HEAD
  EndianS64_LtoB:=EndianS64_Swap(arg);
=======
	EndianS64_Swap := (SInt64( Endian32_Swap( arg and $FFFFFFFF ) ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
>>>>>>> graemeg/cpstrnew
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
end;



{implementation of unit IOKitReturn}


{$push}
{$R-,Q-}

function iokit_common_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_common_err:=(sys_iokit or sub_iokit_common or (ret))
end;

function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
begin
  iokit_family_err:=(sys_iokit or (sub) or (ret))
end;

function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_vendor_specific_err:=(sys_iokit or sub_iokit_vendor_specific or (ret))
end;

{$pop}

{implementation of unit MIDIServices}
{$ifc TARGET_OS_MAC}
=======

function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoB:=EndianS64_Swap(arg);
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;
=======

implementation

{implementation of unit MixedMode}


{$R-}

function NewRoutineDescriptor( theProc: ProcPtr; theProcInfo: ProcInfoType; theISA: ISAType ): UniversalProcPtr; inline;
begin
	NewRoutineDescriptor := UniversalProcPtr(theProc);
end;

procedure DisposeRoutineDescriptor( theUPP: UniversalProcPtr ); inline;
begin
end;

{implementation of unit cssmerr}

{$ifc TARGET_OS_MAC}


function CSSM_ERRCODE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRCODE:=(arg - CSSM_BASE_ERROR) and (CSSM_ERRORCODE_MODULE_EXTENT - 1)
end;

function CSSM_ERRBASE(arg: UInt32): UInt32; inline;
begin
  CSSM_ERRBASE:=((arg - CSSM_BASE_ERROR) and not(CSSM_ERRORCODE_MODULE_EXTENT - 1)) + CSSM_BASE_ERROR
end;

function CSSM_ERR_IS_CONVERTIBLE(arg: UInt32): Boolean; inline;
begin
  CSSM_ERR_IS_CONVERTIBLE:=CSSM_ERRCODE(arg) < CSSM_ERRORCODE_COMMON_EXTENT
end;

function CSSM_ERR_TAG(code, base: UInt32): UInt32; inline;
begin
  CSSM_ERR_TAG:=CSSM_ERRCODE(code) + base
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit mach_error}


{$push}
{$R-,Q-}

function err_system(x: mach_error_t): mach_error_t; inline;
begin
  err_system:=(((x) and $3f) shl 26)
end;

function err_sub(x: mach_error_t): mach_error_t; inline;
begin
  err_sub:=(((x) shr 14) and $fff)
end;


function err_get_system(err: mach_error_t): mach_error_t; inline;
begin
  err_get_system:=(((err) shr 26) and $3f)
end;

function err_get_sub(err: mach_error_t): mach_error_t; inline;
begin
  err_get_sub:=(((err) shr 14) and $fff)
end;

function err_get_code(err: mach_error_t): mach_error_t; inline;
begin
  err_get_code:=((err) and $3fff)
end;


function unix_err(errno: SInt32): mach_error_t; inline;
begin
  unix_err:=err_kern or (((3) and $fff) shl 14) or errno;
end;

{$pop}

{implementation of unit CFByteOrders}
>>>>>>> origin/cpstrnew

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
end;



{implementation of unit IOKitReturn}

>>>>>>> graemeg/cpstrnew

{$push}
{$R-,Q-}

<<<<<<< HEAD
=======
function iokit_common_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_common_err:=(sys_iokit or sub_iokit_common or (ret))
end;

function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
begin
  iokit_family_err:=(sys_iokit or (sub) or (ret))
end;

function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_vendor_specific_err:=(sys_iokit or sub_iokit_vendor_specific or (ret))
end;

{$pop}

{implementation of unit MIDIServices}
{$ifc TARGET_OS_MAC}


>>>>>>> graemeg/cpstrnew
{$R-}
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;
begin
	MIDIPacketNext := MIDIPacketPtr(@pkt^.data[pkt^.length])
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit MIDIThruConnection}
{$ifc TARGET_OS_MAC}


function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;
begin
	MIDIThruConnectionParamsSize := sizeof(MIDIThruConnectionParams) +
									(ptr^.numControlTransforms * sizeof(MIDIControlTransform)) +
									(ptr^.numMaps * sizeof(MIDIValueMap))
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit OSUtils}

{$ifc TARGET_OS_MAC}

{$R-}

function GetMMUMode: SInt8; inline;
begin
	GetMMUMode:= true32b
end;

procedure SwapMMUMode( var mode: SInt8 ); inline;
begin
	mode := true32b;
end;


{$endc} {TARGET_OS_MAC}

{implementation of unit cssmapple}

{$ifc TARGET_OS_MAC}



function CSSM_ACL_AUTHORIZATION_PREAUTH(slot: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH:=CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + slot
end;

function CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT:=auth - CSSM_ACL_AUTHORIZATION_PREAUTH_BASE
end;

function CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth: UInt32): Boolean; inline;
begin
  CSSM_ACL_AUTHORIZATION_IS_PREAUTH:=
    (auth >= CSSM_ACL_AUTHORIZATION_PREAUTH_BASE) and
    (auth < CSSM_ACL_AUTHORIZATION_PREAUTH_END)
end;

<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
=======
{$endc}

function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoB:=EndianS16_Swap(arg);
end;

function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoB:=EndianS64_Swap(arg);
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
end;



{implementation of unit IOKitReturn}
>>>>>>> graemeg/cpstrnew


{$push}
{$R-,Q-}

<<<<<<< HEAD
{implementation of unit Events}

{$ifc TARGET_OS_MAC and not TARGET_CPU_64}
=======
function iokit_common_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_common_err:=(sys_iokit or sub_iokit_common or (ret))
end;

function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
begin
  iokit_family_err:=(sys_iokit or (sub) or (ret))
end;

function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_vendor_specific_err:=(sys_iokit or sub_iokit_vendor_specific or (ret))
end;

{$pop}

{implementation of unit MIDIServices}
{$ifc TARGET_OS_MAC}


{$R-}
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;
begin
	MIDIPacketNext := MIDIPacketPtr(@pkt^.data[pkt^.length])
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit MIDIThruConnection}
{$ifc TARGET_OS_MAC}


function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;
begin
	MIDIThruConnectionParamsSize := sizeof(MIDIThruConnectionParams) +
									(ptr^.numControlTransforms * sizeof(MIDIControlTransform)) +
									(ptr^.numMaps * sizeof(MIDIValueMap))
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit OSUtils}

{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew


{$ifc TARGET_RT_BIG_ENDIAN}

procedure GetKeys( var theKeys: KeyMap );
var
	theReverseKeys: KeyMap;
	theKey: 0..127;
begin
	__GetKeys( theReverseKeys);
	for theKey:= 0 to 127 do
		theKeys[ theKey]:= theReverseKeys[ ((theKey div 8) * 8) + (7 - (theKey mod 8))]
end;

{$elsec}

procedure GetKeys( var theKeys: KeyMap );
begin
	__GetKeys( theKeys)
end;

<<<<<<< HEAD
=======

{$endc} {TARGET_OS_MAC}

{implementation of unit cssmapple}

{$ifc TARGET_OS_MAC}



function CSSM_ACL_AUTHORIZATION_PREAUTH(slot: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH:=CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + slot
end;

function CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT:=auth - CSSM_ACL_AUTHORIZATION_PREAUTH_BASE
end;

function CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth: UInt32): Boolean; inline;
begin
  CSSM_ACL_AUTHORIZATION_IS_PREAUTH:=
    (auth >= CSSM_ACL_AUTHORIZATION_PREAUTH_BASE) and
    (auth < CSSM_ACL_AUTHORIZATION_PREAUTH_END)
end;

<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
=======
function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoB:=EndianS16_Swap(arg);
end;

function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

<<<<<<< HEAD
function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoB:=EndianS64_Swap(arg);
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
=======
function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;
{$endc}


{implementation of unit CGGeometry}

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
end;


function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
>>>>>>> origin/cpstrnew
end;



{implementation of unit IOKitReturn}
>>>>>>> graemeg/cpstrnew


{$push}
{$R-,Q-}

<<<<<<< HEAD
{implementation of unit Events}

{$ifc TARGET_OS_MAC and not TARGET_CPU_64}
=======
function iokit_common_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_common_err:=(sys_iokit or sub_iokit_common or (ret))
end;

function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
begin
  iokit_family_err:=(sys_iokit or (sub) or (ret))
end;

function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_vendor_specific_err:=(sys_iokit or sub_iokit_vendor_specific or (ret))
end;

{$pop}

{implementation of unit MIDIServices}
{$ifc TARGET_OS_MAC}


{$R-}
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;
begin
	MIDIPacketNext := MIDIPacketPtr(@pkt^.data[pkt^.length])
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit MIDIThruConnection}
{$ifc TARGET_OS_MAC}


function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;
begin
<<<<<<< HEAD
	MIDIThruConnectionParamsSize := sizeof(MIDIThruConnectionParams) +
									(ptr^.numControlTransforms * sizeof(MIDIControlTransform)) +
									(ptr^.numMaps * sizeof(MIDIValueMap))
=======
	EndianS64_Swap := (SInt64( Endian32_Swap( arg and $FFFFFFFF ) ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
>>>>>>> origin/cpstrnew
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit OSUtils}

{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew


{$ifc TARGET_RT_BIG_ENDIAN}

procedure GetKeys( var theKeys: KeyMap );
var
	theReverseKeys: KeyMap;
	theKey: 0..127;
begin
	__GetKeys( theReverseKeys);
	for theKey:= 0 to 127 do
		theKeys[ theKey]:= theReverseKeys[ ((theKey div 8) * 8) + (7 - (theKey mod 8))]
end;

{$elsec}

procedure GetKeys( var theKeys: KeyMap );
begin
	__GetKeys( theKeys)
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

{$endc} {TARGET_OS_MAC}

{implementation of unit cssmapple}

{$ifc TARGET_OS_MAC}



function CSSM_ACL_AUTHORIZATION_PREAUTH(slot: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH:=CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + slot
end;

function CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT:=auth - CSSM_ACL_AUTHORIZATION_PREAUTH_BASE
end;

function CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth: UInt32): Boolean; inline;
begin
  CSSM_ACL_AUTHORIZATION_IS_PREAUTH:=
    (auth >= CSSM_ACL_AUTHORIZATION_PREAUTH_BASE) and
    (auth < CSSM_ACL_AUTHORIZATION_PREAUTH_END)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit CFString}


{implementation of unit Events}

{$ifc TARGET_OS_MAC and not TARGET_CPU_64}


{$ifc TARGET_RT_BIG_ENDIAN}

procedure GetKeys( var theKeys: KeyMap );
var
	theReverseKeys: KeyMap;
	theKey: 0..127;
begin
	__GetKeys( theReverseKeys);
	for theKey:= 0 to 127 do
		theKeys[ theKey]:= theReverseKeys[ ((theKey div 8) * 8) + (7 - (theKey mod 8))]
end;

{$elsec}

procedure GetKeys( var theKeys: KeyMap );
begin
	__GetKeys( theKeys)
end;

>>>>>>> graemeg/cpstrnew
{$endc}


{$endc} {TARGET_OS_MAC and not TARGET_CPU_64}

{implementation of unit CFUserNotification}


{$R-}

function CFUserNotificationCheckBoxChecked( i: CFIndex ): CFOptionFlags; inline;
begin
	CFUserNotificationCheckBoxChecked := CFOptionFlags(1 shl (8+i));
end;

function CFUserNotificationSecureTextField( i: CFIndex ): CFOptionFlags; inline;
begin
	CFUserNotificationSecureTextField := CFOptionFlags(1 shl (16+i));
end;

function CFUserNotificationPopUpSelection( n: CFIndex ): CFOptionFlags; inline;
begin
	CFUserNotificationPopUpSelection := CFOptionFlags(n shl 24);
end;

{implementation of unit ToolUtils}

{$ifc TARGET_OS_MAC}



function HiWord(arg: SInt32): SInt16; inline;
begin
  HiWord := arg shr 16;
end;


function HiWord(arg: UInt32): UInt16; inline;
begin
  HiWord := arg shr 16;
end;


function LoWord(arg: SInt32): SInt16; inline;
begin
  LoWord := SInt16(arg);
end;


function LoWord(arg: UInt32): UInt16; inline;
begin
  LoWord := UInt16(arg);
end;
  

{$endc} {TARGET_OS_MAC}

{implementation of unit CFBundle}


{$R-}

function CFCopyLocalizedString( key: CFStringRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedString := CFBundleCopyLocalizedString( CFBundleGetMainBundle, key, key, nil );
end;

function CFCopyLocalizedStringFromTable( key: CFStringRef; tableName: CFStringRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedStringFromTable := CFBundleCopyLocalizedString( CFBundleGetMainBundle, key, key, tableName );
end;

function CFCopyLocalizedStringFromTableInBundle( key: CFStringRef; tableName: CFStringRef; bundle: CFBundleRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedStringFromTableInBundle := CFBundleCopyLocalizedString( bundle, key, key, tableName );
end;

function CFCopyLocalizedStringWithDefaultValue( key: CFStringRef; tableName: CFStringRef; bundle: CFBundleRef; value: CFStringRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedStringWithDefaultValue := CFBundleCopyLocalizedString( bundle, key, value, tableName );
end;


{implementation of unit CFPlugInCOM}


{$R-}

function SUCCEEDED( Status: HRESULT ): Boolean; inline;
begin
	SUCCEEDED := Status >= 0;
end;

function FAILED( Status: HRESULT ): Boolean; inline;
begin
	FAILED := Status < 0;
end;

function IS_ERROR( Status: HRESULT ): Boolean; inline;
begin
	IS_ERROR := Status shr 31 = SEVERITY_ERROR;
end;

function HRESULT_CODE( hr: HRESULT ): HRESULT; inline;
begin
	HRESULT_CODE := hr and $FFFF;
end;

function HRESULT_FACILITY( hr: HRESULT ): HRESULT; inline;
begin
	HRESULT_FACILITY := (hr shr 16) and $1FFF;
end;

function HRESULT_SEVERITY( hr: HRESULT ): HRESULT; inline;
begin
	HRESULT_SEVERITY := (hr shr 31) and $01;
end;

function MAKE_HRESULT( sev, fac, code: ULONG ): HRESULT; inline;
begin
	MAKE_HRESULT := HRESULT((sev shl 31) or (fac shl 16) or code);
end;

function IUnknownUUID: CFUUIDRef; inline;
begin
	IUnknownUUID:= CFUUIDGetConstantUUIDWithBytes( nil, $00, $00, $00, $00, $00, $00, $00, $00, $C0, $00, $00, $00, $00, $00, $00, $46 )
end;


{implementation of unit MDExternalDatastore}
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew
=======

{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

function kMDExternalDatastoreTypeID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$49,$68,$94,$B1,$00,$30,$47,$E0,$96,$11,$F2,$48,$FB,$E0,$B8,$CA)
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
function kMDExternalDatastoreStoreInterfaceID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreStoreInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$DA,$62,$12,$99,$ED,$BE,$4A,$63,$92,$39,$CB,$24,$13,$73,$E2,$07)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit MDImporter}

{$ifc TARGET_OS_MAC}


function kMDImporterTypeID: CFUUIDRef; inline;
begin
	kMDImporterTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$8B,$08,$C4,$BF,$41,$5B,$11,$D8,$B3,$F9,$00,$03,$93,$67,$26,$FC)
end;

function kMDImporterInterfaceID: CFUUIDRef; inline;
begin
	kMDImporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$6E,$BC,$27,$C4,$89,$9C,$11,$D8,$84,$AE,$00,$03,$93,$67,$26,$FC)
end;

function kMDExporterInterfaceID: CFUUIDRef; inline;
begin
	kMDExporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$B4,$1C,$60,$74,$7D,$FB,$40,$57,$96,$9D,$31,$C8,$E8,$61,$A8,$D4)
end;

<<<<<<< HEAD
<<<<<<< HEAD
function kMDImporterURLInterfaceID: CFUUIDRef; inline;
begin
	kMDImporterURLInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$13,$F6,$0F,$02,$36,$22,$4F,$35,$98,$91,$EC,$10,$E6,$CD,$08,$F8)
end;
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew

{implementation of unit MIDIDriver}

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}

{implementation of unit MIDIDriver}

{$ifc TARGET_OS_MAC}

=======
>>>>>>> graemeg/cpstrnew
=======
function kMDExternalDatastoreTypeID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$49,$68,$94,$B1,$00,$30,$47,$E0,$96,$11,$F2,$48,$FB,$E0,$B8,$CA)
end;

function kMDExternalDatastoreStoreInterfaceID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreStoreInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$DA,$62,$12,$99,$ED,$BE,$4A,$63,$92,$39,$CB,$24,$13,$73,$E2,$07)
end;

{$endc} {TARGET_OS_MAC}
=======

function kMIDIDriverTypeID : CFUUIDRef; inline;
begin
	kMIDIDriverTypeID := CFUUIDGetConstantUUIDWithBytes(nil, $EC, $DE, $95, $74, $0F, $E4, $11, $D4, $BB, $1A, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterfaceID : CFUUIDRef; inline;
begin
	kMIDIDriverInterfaceID := CFUUIDGetConstantUUIDWithBytes(nil, $49, $DF, $CA, $9E, $0F, $E5, $11, $D4, $95, $0D, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterface2ID : CFUUIDRef; inline;
begin
	kMIDIDriverInterface2ID := CFUUIDGetConstantUUIDWithBytes(nil, $43, $C9, $8C, $3C, $30, $6C, $11, $D5, $AF, $73, $00, $30, $65, $A8, $30, $1E)
end;
{$endc}
{implementation of unit AudioUnitProperties}
>>>>>>> graemeg/cpstrnew

{implementation of unit MDImporter}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
=======
function GetAudioUnitParameterDisplayType(flags : UInt32) : UInt32; inline;
begin
	GetAudioUnitParameterDisplayType := flags and kAudioUnitParameterFlag_DisplayMask
end;
>>>>>>> graemeg/cpstrnew

function AudioUnitDisplayTypeIsLogarithmic(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsLogarithmic := GetAudioUnitParameterDisplayType(flags) = 	kAudioUnitParameterFlag_DisplayLogarithmic
end;

<<<<<<< HEAD
function kMDImporterTypeID: CFUUIDRef; inline;
begin
	kMDImporterTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$8B,$08,$C4,$BF,$41,$5B,$11,$D8,$B3,$F9,$00,$03,$93,$67,$26,$FC)
end;

function kMDImporterInterfaceID: CFUUIDRef; inline;
begin
	kMDImporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$6E,$BC,$27,$C4,$89,$9C,$11,$D8,$84,$AE,$00,$03,$93,$67,$26,$FC)
end;

function kMDExporterInterfaceID: CFUUIDRef; inline;
begin
	kMDExporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$B4,$1C,$60,$74,$7D,$FB,$40,$57,$96,$9D,$31,$C8,$E8,$61,$A8,$D4)
end;

<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}

{implementation of unit MIDIDriver}
=======
{$endc}

function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoB:=EndianS16_Swap(arg);
end;

function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoB:=EndianS64_Swap(arg);
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
end;



{implementation of unit IOKitReturn}


{$push}
{$R-,Q-}

function iokit_common_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_common_err:=(sys_iokit or sub_iokit_common or (ret))
end;

function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
begin
  iokit_family_err:=(sys_iokit or (sub) or (ret))
end;

function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_vendor_specific_err:=(sys_iokit or sub_iokit_vendor_specific or (ret))
end;

{$pop}

{implementation of unit MIDIServices}
{$ifc TARGET_OS_MAC}


{$R-}
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;
begin
	MIDIPacketNext := MIDIPacketPtr(@pkt^.data[pkt^.length])
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit MIDIThruConnection}
{$ifc TARGET_OS_MAC}


function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;
begin
	MIDIThruConnectionParamsSize := sizeof(MIDIThruConnectionParams) +
									(ptr^.numControlTransforms * sizeof(MIDIControlTransform)) +
									(ptr^.numMaps * sizeof(MIDIValueMap))
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit OSUtils}
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew

function kMIDIDriverTypeID : CFUUIDRef; inline;
begin
	kMIDIDriverTypeID := CFUUIDGetConstantUUIDWithBytes(nil, $EC, $DE, $95, $74, $0F, $E4, $11, $D4, $BB, $1A, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterfaceID : CFUUIDRef; inline;
begin
	kMIDIDriverInterfaceID := CFUUIDGetConstantUUIDWithBytes(nil, $49, $DF, $CA, $9E, $0F, $E5, $11, $D4, $95, $0D, $00, $50, $E4, $CE, $A5, $26)
end;

<<<<<<< HEAD
function kMIDIDriverInterface2ID : CFUUIDRef; inline;
=======

{$endc} {TARGET_OS_MAC}

{implementation of unit cssmapple}

{$ifc TARGET_OS_MAC}



function CSSM_ACL_AUTHORIZATION_PREAUTH(slot: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH:=CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + slot
end;

function CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT:=auth - CSSM_ACL_AUTHORIZATION_PREAUTH_BASE
end;

function CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth: UInt32): Boolean; inline;
begin
  CSSM_ACL_AUTHORIZATION_IS_PREAUTH:=
    (auth >= CSSM_ACL_AUTHORIZATION_PREAUTH_BASE) and
    (auth < CSSM_ACL_AUTHORIZATION_PREAUTH_END)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit CFString}


{implementation of unit Events}

{$ifc TARGET_OS_MAC and not TARGET_CPU_64}


{$ifc TARGET_RT_BIG_ENDIAN}

procedure GetKeys( var theKeys: KeyMap );
var
	theReverseKeys: KeyMap;
	theKey: 0..127;
begin
	__GetKeys( theReverseKeys);
	for theKey:= 0 to 127 do
		theKeys[ theKey]:= theReverseKeys[ ((theKey div 8) * 8) + (7 - (theKey mod 8))]
end;

{$elsec}

procedure GetKeys( var theKeys: KeyMap );
begin
	__GetKeys( theKeys)
end;

{$endc}


{$endc} {TARGET_OS_MAC and not TARGET_CPU_64}

{implementation of unit CFUserNotification}


{$R-}

function CFUserNotificationCheckBoxChecked( i: CFIndex ): CFOptionFlags; inline;
>>>>>>> origin/cpstrnew
begin
	kMIDIDriverInterface2ID := CFUUIDGetConstantUUIDWithBytes(nil, $43, $C9, $8C, $3C, $30, $6C, $11, $D5, $AF, $73, $00, $30, $65, $A8, $30, $1E)
=======
function AudioUnitDisplayTypeIsSquareRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquareRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquareRoot
end;

function AudioUnitDisplayTypeIsSquared(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquared := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquared
end;

function AudioUnitDisplayTypeIsCubed(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubed := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubed
end;

<<<<<<< HEAD
function AudioUnitDisplayTypeIsCubeRoot(flags : UInt32) : Boolean; inline;
=======
{implementation of unit ToolUtils}

{$ifc TARGET_OS_MAC}



function HiWord(arg: SInt32): SInt16; inline;
begin
  HiWord := arg shr 16;
end;


function HiWord(arg: UInt32): UInt16; inline;
begin
  HiWord := arg shr 16;
end;


function LoWord(arg: SInt32): SInt16; inline;
begin
  LoWord := SInt16(arg);
end;


function LoWord(arg: UInt32): UInt16; inline;
begin
  LoWord := UInt16(arg);
end;
  

{$endc} {TARGET_OS_MAC}

{implementation of unit CFBundle}


{$R-}

function CFCopyLocalizedString( key: CFStringRef; comment: PChar ): CFStringRef; inline;
>>>>>>> origin/cpstrnew
begin
	AudioUnitDisplayTypeIsCubeRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubeRoot
>>>>>>> graemeg/cpstrnew
end;
{$endc}
{implementation of unit AudioUnitProperties}

function AudioUnitDisplayTypeIsExponential(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsExponential := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayExponential
end;

<<<<<<< HEAD
function GetAudioUnitParameterDisplayType(flags : UInt32) : UInt32; inline;
begin
	GetAudioUnitParameterDisplayType := flags and kAudioUnitParameterFlag_DisplayMask
end;

function AudioUnitDisplayTypeIsLogarithmic(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsLogarithmic := GetAudioUnitParameterDisplayType(flags) = 	kAudioUnitParameterFlag_DisplayLogarithmic
end;

function AudioUnitDisplayTypeIsSquareRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquareRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquareRoot
end;

function AudioUnitDisplayTypeIsSquared(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquared := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquared
end;

function AudioUnitDisplayTypeIsCubed(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubed := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubed
end;

function AudioUnitDisplayTypeIsCubeRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubeRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubeRoot
end;

function AudioUnitDisplayTypeIsExponential(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsExponential := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayExponential
end;

procedure SetAudioUnitParameterDisplayType(var flags : UInt32; displayType : UInt32); inline;
begin
	flags := (flags and (not kAudioUnitParameterFlag_DisplayMask)) or displayType
end;

{implementation of unit QLGenerator}

{$ifc TARGET_OS_MAC}


function kQLGeneratorTypeID : CFUUIDRef; inline;
begin
	kQLGeneratorTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $5E, $2D, $96, $80, $50, $22, $40, $FA, $B8, $06, $43, $34, $96, $22, $E5, $B9)
end;

=======
procedure SetAudioUnitParameterDisplayType(var flags : UInt32; displayType : UInt32); inline;
begin
<<<<<<< HEAD
	flags := (flags and (not kAudioUnitParameterFlag_DisplayMask)) or displayType
=======
	IUnknownUUID:= CFUUIDGetConstantUUIDWithBytes( nil, $00, $00, $00, $00, $00, $00, $00, $00, $C0, $00, $00, $00, $00, $00, $00, $46 )
>>>>>>> origin/cpstrnew
end;

{implementation of unit QLGenerator}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}


function kQLGeneratorTypeID : CFUUIDRef; inline;
begin
	kQLGeneratorTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $5E, $2D, $96, $80, $50, $22, $40, $FA, $B8, $06, $43, $34, $96, $22, $E5, $B9)
end;

>>>>>>> graemeg/cpstrnew
function kQLGeneratorCallbacksInterfaceID: CFUUIDRef; inline; 
begin
	kQLGeneratorCallbacksInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $86, $5A, $F5, $E0, $6D, $30, $43, $45, $95, $1B, $D3, $71, $05, $75, $4F, $2D)
end;

{$endc} {TARGET_OS_MAC}
=======
{implementation of unit MDExternalDatastore}

{$ifc TARGET_OS_MAC}


function kMDExternalDatastoreTypeID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$49,$68,$94,$B1,$00,$30,$47,$E0,$96,$11,$F2,$48,$FB,$E0,$B8,$CA)
end;

function kMDExternalDatastoreStoreInterfaceID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreStoreInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$DA,$62,$12,$99,$ED,$BE,$4A,$63,$92,$39,$CB,$24,$13,$73,$E2,$07)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit MDImporter}

{$ifc TARGET_OS_MAC}


function kMDImporterTypeID: CFUUIDRef; inline;
begin
	kMDImporterTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$8B,$08,$C4,$BF,$41,$5B,$11,$D8,$B3,$F9,$00,$03,$93,$67,$26,$FC)
end;

function kMDImporterInterfaceID: CFUUIDRef; inline;
begin
	kMDImporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$6E,$BC,$27,$C4,$89,$9C,$11,$D8,$84,$AE,$00,$03,$93,$67,$26,$FC)
end;

function kMDExporterInterfaceID: CFUUIDRef; inline;
begin
	kMDExporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$B4,$1C,$60,$74,$7D,$FB,$40,$57,$96,$9D,$31,$C8,$E8,$61,$A8,$D4)
end;
>>>>>>> origin/cpstrnew

{$endc} {TARGET_OS_MAC}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
{$ifc not TARGET_CPU_64}

=======
{implementation of unit MIDIDriver}
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}


function kMIDIDriverTypeID : CFUUIDRef; inline;
begin
	kMIDIDriverTypeID := CFUUIDGetConstantUUIDWithBytes(nil, $EC, $DE, $95, $74, $0F, $E4, $11, $D4, $BB, $1A, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterfaceID : CFUUIDRef; inline;
begin
	kMIDIDriverInterfaceID := CFUUIDGetConstantUUIDWithBytes(nil, $49, $DF, $CA, $9E, $0F, $E5, $11, $D4, $95, $0D, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterface2ID : CFUUIDRef; inline;
begin
	kMIDIDriverInterface2ID := CFUUIDGetConstantUUIDWithBytes(nil, $43, $C9, $8C, $3C, $30, $6C, $11, $D5, $AF, $73, $00, $30, $65, $A8, $30, $1E)
end;
{$endc}
{implementation of unit AudioUnitProperties}

{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
=======
function GetAudioUnitParameterDisplayType(flags : UInt32) : UInt32; inline;
begin
	GetAudioUnitParameterDisplayType := flags and kAudioUnitParameterFlag_DisplayMask
end;

function AudioUnitDisplayTypeIsLogarithmic(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsLogarithmic := GetAudioUnitParameterDisplayType(flags) = 	kAudioUnitParameterFlag_DisplayLogarithmic
end;

function AudioUnitDisplayTypeIsSquareRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquareRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquareRoot
end;

function AudioUnitDisplayTypeIsSquared(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquared := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquared
end;

function AudioUnitDisplayTypeIsCubed(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubed := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubed
end;

function AudioUnitDisplayTypeIsCubeRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubeRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubeRoot
end;

function AudioUnitDisplayTypeIsExponential(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsExponential := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayExponential
end;

procedure SetAudioUnitParameterDisplayType(var flags : UInt32; displayType : UInt32); inline;
begin
	flags := (flags and (not kAudioUnitParameterFlag_DisplayMask)) or displayType
end;

{implementation of unit QLGenerator}

{$ifc TARGET_OS_MAC}


function kQLGeneratorTypeID : CFUUIDRef; inline;
begin
	kQLGeneratorTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $5E, $2D, $96, $80, $50, $22, $40, $FA, $B8, $06, $43, $34, $96, $22, $E5, $B9)
end;

function kQLGeneratorCallbacksInterfaceID: CFUUIDRef; inline; 
begin
	kQLGeneratorCallbacksInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $86, $5A, $F5, $E0, $6D, $30, $43, $45, $95, $1B, $D3, $71, $05, $75, $4F, $2D)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit Menus}

{$ifc TARGET_OS_MAC}
{$ifc not TARGET_CPU_64}
=======
=======
=======
{$setc MACOSALLINCLUDE := TRUE}
>>>>>>> origin/fixes_2.4
unit MacOSAll;
interface

{$ifc defined CPUPOWERPC32 or defined CPUI386}
{$linkframework Carbon}
{$elsec}
{$linkframework CoreFoundation}
{$endc}

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

<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{unit AXConstants}
{$i AXConstants.pas}
{unit Accessibility}
{$i Accessibility.pas}
>>>>>>> origin/fixes_2.4
{unit ConditionalMacros}
{$i ConditionalMacros.pas}
{unit MacTypes}
{$i MacTypes.pas}
{unit MachineExceptions}
{$i MachineExceptions.pas}
{unit Math64}
{$i Math64.pas}
{unit MixedMode}
{$i MixedMode.pas}
{unit Multiprocessing}
{$i Multiprocessing.pas}
(* conflicts with FPC ObjC support
{unit ObjCRuntime}
{$i ObjCRuntime.pas}
*)
{unit PEFBinaryFormat}
{$i PEFBinaryFormat.pas}
{unit PLStringFuncs}
{$i PLStringFuncs.pas}
{unit PMDefinitionsDeprecated}
{$i PMDefinitionsDeprecated.pas}
{unit PMPrintSettingsKeys}
{$i PMPrintSettingsKeys.pas}
{unit PMPrintingDialogExtensions}
{$i PMPrintingDialogExtensions.pas}
{unit QDCMCommon}
{$i QDCMCommon.pas}
{unit QLBase}
{$i QLBase.pas}
{unit QTML}
{$i QTML.pas}
{unit QuickTimeErrors}
{$i QuickTimeErrors.pas}
{unit SFNTLayoutTypes}
{$i SFNTLayoutTypes.pas}
{unit SFNTTypes}
{$i SFNTTypes.pas}
{unit ScalerStreamTypes}
{$i ScalerStreamTypes.pas}
{unit TextCommon}
{$i TextCommon.pas}
{unit Threads}
{$i Threads.pas}
{unit UTCUtils}
{$i UTCUtils.pas}
{unit UnicodeConverter}
{$i UnicodeConverter.pas}
{unit cblas}
{$i cblas.pas}
{unit fenv}
{$i fenv.pas}
{unit fp}
{$i fp.pas}
{unit gliContexts}
{$i gliContexts.pas}
{unit macgl}
{$i macgl.pas}
{unit macglext}
{$i macglext.pas}
{unit macglu}
{$i macglu.pas}
{unit vBLAS}
{$i vBLAS.pas}
{unit vDSP}
{$i vDSP.pas}
{unit xattr}
{$i xattr.pas}
{unit ABTypedefs}
{$i ABTypedefs.pas}
{unit AEDataModel}
{$i AEDataModel.pas}
{unit AEUserTermTypes}
{$i AEUserTermTypes.pas}
{unit AIFF}
{$i AIFF.pas}
{unit AVLTree}
{$i AVLTree.pas}
{unit AXActionConstants}
{$i AXActionConstants.pas}
{unit AXAttributeConstants}
{$i AXAttributeConstants.pas}
{unit AXErrors}
{$i AXErrors.pas}
{unit AXNotificationConstants}
{$i AXNotificationConstants.pas}
{unit AXRoleConstants}
{$i AXRoleConstants.pas}
{unit AXValueConstants}
{$i AXValueConstants.pas}
{unit AppleDiskPartitions}
{$i AppleDiskPartitions.pas}
{unit AudioUnitParameters}
{$i AudioUnitParameters.pas}
{unit Authorization}
{$i Authorization.pas}
{unit AuthorizationPlugin}
{$i AuthorizationPlugin.pas}
{unit AuthorizationTags}
{$i AuthorizationTags.pas}
{unit CFBase}
{$i CFBase.pas}
{unit CFBinaryHeap}
{$i CFBinaryHeap.pas}
{unit CFBitVector}
{$i CFBitVector.pas}
{unit CFByteOrders}
{$i CFByteOrders.pas}
{unit CFData}
{$i CFData.pas}
{unit CFDate}
{$i CFDate.pas}
{unit CFDictionary}
{$i CFDictionary.pas}
{unit CFError}
{$i CFError.pas}
{unit CFNetworkErrorss}
{$i CFNetworkErrorss.pas}
{unit CFNotificationCenter}
{$i CFNotificationCenter.pas}
{unit CFNumber}
{$i CFNumber.pas}
{unit CFSet}
{$i CFSet.pas}
{unit CFStringEncodingExt}
{$i CFStringEncodingExt.pas}
{unit CFTree}
{$i CFTree.pas}
{unit CGBase}
{$i CGBase.pas}
{unit CGErrors}
{$i CGErrors.pas}
{unit CGFunction}
{$i CGFunction.pas}
{unit CGGeometry}
{$i CGGeometry.pas}
{unit CGImageProperties}
{$i CGImageProperties.pas}
{unit CGLProfiler}
{$i CGLProfiler.pas}
{unit CGLProfilerFunctionEnums}
{$i CGLProfilerFunctionEnums.pas}
{unit CGLRenderers}
{$i CGLRenderers.pas}
{unit CGLTypes}
{$i CGLTypes.pas}
{unit CGPDFObject}
{$i CGPDFObject.pas}
{unit CGPDFOperatorTable}
{$i CGPDFOperatorTable.pas}
{unit CGPDFScanner}
{$i CGPDFScanner.pas}
{unit CGPDFStream}
{$i CGPDFStream.pas}
{unit CGPDFString}
{$i CGPDFString.pas}
{unit CGSession}
{$i CGSession.pas}
{unit CGWindowLevels}
{$i CGWindowLevels.pas}
{unit CSIdentityAuthority}
{$i CSIdentityAuthority.pas}
{unit CTFontManagerErrors}
{$i CTFontManagerErrors.pas}
{unit CTFontTraits}
{$i CTFontTraits.pas}
{unit CVBase}
{$i CVBase.pas}
{unit CVHostTime}
{$i CVHostTime.pas}
{unit CVReturns}
{$i CVReturns.pas}
{unit Collections}
{$i Collections.pas}
{unit CoreAudioTypes}
{$i CoreAudioTypes.pas}
{unit DHCPClientPreferences}
{$i DHCPClientPreferences.pas}
{unit DateTimeUtils}
{$i DateTimeUtils.pas}
{unit Debugging}
{$i Debugging.pas}
{unit DigitalHubRegistry}
{$i DigitalHubRegistry.pas}
{unit DriverServices}
{$i DriverServices.pas}
{unit DriverSynchronization}
{$i DriverSynchronization.pas}
{unit Endian}
{$i Endian.pas}
{unit FileTypesAndCreators}
{$i FileTypesAndCreators.pas}
{unit Finder}
{$i Finder.pas}
{unit FixMath}
{$i FixMath.pas}
{unit GestaltEqu}
{$i GestaltEqu.pas}
{unit HFSVolumes}
{$i HFSVolumes.pas}
{unit HIGeometry}
{$i HIGeometry.pas}
{unit HITextUtils}
{$i HITextUtils.pas}
{unit HostTime}
{$i HostTime.pas}
{unit ICACamera}
{$i ICACamera.pas}
{unit IconStorage}
{$i IconStorage.pas}
{unit IntlResources}
{$i IntlResources.pas}
{unit Keyboards}
{$i Keyboards.pas}
{unit LSQuarantine}
{$i LSQuarantine.pas}
{unit MIDIServices}
{$i MIDIServices.pas}
{unit MIDIThruConnection}
{$i MIDIThruConnection.pas}
{unit MacErrors}
{$i MacErrors.pas}
{unit MacLocales}
{$i MacLocales.pas}
{unit MacMemory}
{$i MacMemory.pas}
{unit MacOSXPosix}
{$i MacOSXPosix.pas}
{unit MultiProcessingInfo}
{$i MultiProcessingInfo.pas}
{unit NumberFormatting}
{$i NumberFormatting.pas}
{unit OSAComp}
{$i OSAComp.pas}
{unit OSUtils}
{$i OSUtils.pas}
{unit OpenTransport}
{$i OpenTransport.pas}
{unit OpenTransportProviders}
{$i OpenTransportProviders.pas}
{unit Power}
{$i Power.pas}
{unit QuickdrawText}
{$i QuickdrawText.pas}
{unit QuickdrawTypes}
{$i QuickdrawTypes.pas}
{unit SCDynamicStoreKey}
{$i SCDynamicStoreKey.pas}
{unit SCNetwork}
{$i SCNetwork.pas}
{unit SCSI}
{$i SCSI.pas}
{unit SCSchemaDefinitions}
{$i SCSchemaDefinitions.pas}
{unit Script}
{$i Script.pas}
{unit SecBase}
{$i SecBase.pas}
{unit StringCompare}
{$i StringCompare.pas}
{unit SystemConfiguration}
{$i SystemConfiguration.pas}
{unit TextEdit}
{$i TextEdit.pas}
{unit TextEncodingConverter}
{$i TextEncodingConverter.pas}
{unit TextEncodingPlugin}
{$i TextEncodingPlugin.pas}
{unit TextUtils}
{$i TextUtils.pas}
{unit Timer}
{$i Timer.pas}
{unit UTCoreTypes}
{$i UTCoreTypes.pas}
{unit UnicodeUtilities}
{$i UnicodeUtilities.pas}
{unit UniversalAccess}
{$i UniversalAccess.pas}
{unit Video}
{$i Video.pas}
{unit WSTypes}
{$i WSTypes.pas}
{unit gliDispatch}
{$i gliDispatch.pas}
{unit gluContext}
{$i gluContext.pas}
{unit ABGlobals}
{$i ABGlobals.pas}
{unit AEMach}
{$i AEMach.pas}
{unit AXTextAttributedString}
{$i AXTextAttributedString.pas}
{unit AXValue}
{$i AXValue.pas}
{unit AuthSession}
{$i AuthSession.pas}
{unit BackupCore}
{$i BackupCore.pas}
{unit CFArray}
{$i CFArray.pas}
{unit CFBag}
{$i CFBag.pas}
{unit CFCharacterSet}
{$i CFCharacterSet.pas}
{unit CFLocale}
{$i CFLocale.pas}
{unit CFNumberFormatter}
{$i CFNumberFormatter.pas}
{unit CFString}
{$i CFString.pas}
{unit CFTimeZone}
{$i CFTimeZone.pas}
{unit CFUUID}
{$i CFUUID.pas}
{unit CGAffineTransforms}
{$i CGAffineTransforms.pas}
{unit CGLCurrent}
{$i CGLCurrent.pas}
{unit CGLDevice}
{$i CGLDevice.pas}
{unit CGPDFArray}
{$i CGPDFArray.pas}
{unit CGPDFContentStream}
{$i CGPDFContentStream.pas}
{unit CGPDFDictionary}
{$i CGPDFDictionary.pas}
{unit CGPath}
{$i CGPath.pas}
{unit CTFontDescriptor}
{$i CTFontDescriptor.pas}
{unit CTFontManager}
{$i CTFontManager.pas}
{unit CTParagraphStyle}
{$i CTParagraphStyle.pas}
{unit CTTextTab}
{$i CTTextTab.pas}
{unit CVBuffer}
{$i CVBuffer.pas}
{unit ColorSyncProfile}
{$i ColorSyncProfile.pas}
{unit ColorSyncTransform}
{$i ColorSyncTransform.pas}
{unit Events}
{$i Events.pas}
{unit HITextLengthFilter}
{$i HITextLengthFilter.pas}
{unit MDItem}
{$i MDItem.pas}
{unit MDQuery}
{$i MDQuery.pas}
{unit MDSchema}
{$i MDSchema.pas}
{unit MIDISetup}
{$i MIDISetup.pas}
{unit MacOpenGL}
{$i MacOpenGL.pas}
{unit Notification}
{$i Notification.pas}
{unit PMDefinitions}
{$i PMDefinitions.pas}
{unit PMErrors}
{$i PMErrors.pas}
{unit Palettes}
{$i Palettes.pas}
{unit PictUtils}
{$i PictUtils.pas}
{unit QDOffscreen}
{$i QDOffscreen.pas}
{unit Scrap}
{$i Scrap.pas}
{unit TypeSelect}
{$i TypeSelect.pas}
{unit ABAddressBook}
{$i ABAddressBook.pas}
{unit CFAttributedString}
{$i CFAttributedString.pas}
{unit CFCalendar}
{$i CFCalendar.pas}
{unit CFDateFormatter}
{$i CFDateFormatter.pas}
{unit CFRunLoop}
{$i CFRunLoop.pas}
{unit CFSocket}
{$i CFSocket.pas}
{unit CSIdentity}
{$i CSIdentity.pas}
{unit CSIdentityQuery}
{$i CSIdentityQuery.pas}
{unit CTFontCollection}
{$i CTFontCollection.pas}
{unit ColorSyncDevice}
{$i ColorSyncDevice.pas}
{unit DASession}
{$i DASession.pas}
{unit FSEvents}
{$i FSEvents.pas}
{unit SCNetworkConnection}
{$i SCNetworkConnection.pas}
{unit SCNetworkReachability}
{$i SCNetworkReachability.pas}
{unit ABActions}
{$i ABActions.pas}
{unit AudioHardware}
{$i AudioHardware.pas}
{unit CFMachPort}
{$i CFMachPort.pas}
{unit CFMessagePort}
{$i CFMessagePort.pas}
{unit CGRemoteOperation}
{$i CGRemoteOperation.pas}
{unit DADisk}
{$i DADisk.pas}
{unit Files}
{$i Files.pas}
{unit Folders}
{$i Folders.pas}
{unit ICAApplication}
{$i ICAApplication.pas}
{unit ICADevice}
{$i ICADevice.pas}
{unit LowMem}
{$i LowMem.pas}
{unit Processes}
{$i Processes.pas}
{unit Resources}
{$i Resources.pas}
{unit SpeechRecognition}
{$i SpeechRecognition.pas}
{unit SystemSound}
{$i SystemSound.pas}
{unit ATSTypes}
{$i ATSTypes.pas}
{unit AXUIElement}
{$i AXUIElement.pas}
{unit Aliases}
{$i Aliases.pas}
{unit AppleHelp}
{$i AppleHelp.pas}
{unit CFURL}
{$i CFURL.pas}
{unit CFURLAccess}
{$i CFURLAccess.pas}
{unit CFUserNotification}
{$i CFUserNotification.pas}
{unit CFXMLNode}
{$i CFXMLNode.pas}
{unit CFXMLParser}
{$i CFXMLParser.pas}
{unit CGDataConsumer}
{$i CGDataConsumer.pas}
{unit CGDataProvider}
{$i CGDataProvider.pas}
{unit CGEventTypes}
{$i CGEventTypes.pas}
{unit CGFont}
{$i CGFont.pas}
{unit CGPDFDocument}
{$i CGPDFDocument.pas}
{unit CGPDFPage}
{$i CGPDFPage.pas}
{unit CGPSConverter}
{$i CGPSConverter.pas}
{unit CTFont}
{$i CTFont.pas}
{unit CTGlyphInfo}
{$i CTGlyphInfo.pas}
{unit CarbonEventsCore}
{$i CarbonEventsCore.pas}
{unit ColorSyncDeprecated}
{$i ColorSyncDeprecated.pas}
{unit Components}
{$i Components.pas}
{unit Fonts}
{$i Fonts.pas}
{unit IconsCore}
{$i IconsCore.pas}
{unit InternetConfig}
{$i InternetConfig.pas}
{unit LSInfo}
{$i LSInfo.pas}
{unit LSOpen}
{$i LSOpen.pas}
{unit LSSharedFileList}
{$i LSSharedFileList.pas}
{unit MDLineage}
{$i MDLineage.pas}
{unit PMCoreDeprecated}
{$i PMCoreDeprecated.pas}
{unit Pasteboard}
{$i Pasteboard.pas}
{unit SpeechSynthesis}
{$i SpeechSynthesis.pas}
{unit TextInputSources}
{$i TextInputSources.pas}
{unit ToolUtils}
{$i ToolUtils.pas}
{unit TranslationExtensions}
{$i TranslationExtensions.pas}
{unit TranslationServices}
{$i TranslationServices.pas}
{unit UTType}
{$i UTType.pas}
{unit WSMethodInvocation}
{$i WSMethodInvocation.pas}
{unit WSProtocolHandler}
{$i WSProtocolHandler.pas}
{unit ABPeoplePicker}
{$i ABPeoplePicker.pas}
{unit AEInteraction}
{$i AEInteraction.pas}
{unit ATSLayoutTypes}
{$i ATSLayoutTypes.pas}
{unit ATSUnicodeTypes}
{$i ATSUnicodeTypes.pas}
{unit AudioComponents}
{$i AudioComponents.pas}
{unit CFBundle}
{$i CFBundle.pas}
{unit CFHTTPMessage}
{$i CFHTTPMessage.pas}
{unit CFPlugIn}
{$i CFPlugIn.pas}
{unit CFPlugInCOM}
{$i CFPlugInCOM.pas}
{unit CFStream}
{$i CFStream.pas}
{unit CGColorSpace}
{$i CGColorSpace.pas}
{unit CGEventSource}
{$i CGEventSource.pas}
{unit CGGradient}
{$i CGGradient.pas}
{unit CGImage}
{$i CGImage.pas}
{unit CGImageSource}
{$i CGImageSource.pas}
{unit CGShading}
{$i CGShading.pas}
{unit CGWindow}
{$i CGWindow.pas}
{unit CMCalibrator}
{$i CMCalibrator.pas}
{unit CVImageBuffer}
{$i CVImageBuffer.pas}
{unit CVOpenGLBuffer}
{$i CVOpenGLBuffer.pas}
{unit CVOpenGLBufferPool}
{$i CVOpenGLBufferPool.pas}
{unit CVOpenGLTexture}
{$i CVOpenGLTexture.pas}
{unit CVOpenGLTextureCache}
{$i CVOpenGLTextureCache.pas}
{unit CVPixelBuffer}
{$i CVPixelBuffer.pas}
{unit CVPixelBufferPool}
{$i CVPixelBufferPool.pas}
{unit CVPixelFormatDescription}
{$i CVPixelFormatDescription.pas}
{unit CodeFragments}
{$i CodeFragments.pas}
{unit ColorPicker}
{$i ColorPicker.pas}
{unit ColorSyncCMM}
{$i ColorSyncCMM.pas}
{unit FontSync}
{$i FontSync.pas}
{unit HIObject}
{$i HIObject.pas}
{unit KeychainCore}
{$i KeychainCore.pas}
{unit KeychainHI}
{$i KeychainHI.pas}
{unit MDExternalDatastore}
{$i MDExternalDatastore.pas}
{unit MDImporter}
{$i MDImporter.pas}
{unit MIDIDriver}
{$i MIDIDriver.pas}
{unit NSLCore}
{$i NSLCore.pas}
{unit OpenTransportProtocol}
{$i OpenTransportProtocol.pas}
{unit QLThumbnailImage}
{$i QLThumbnailImage.pas}
{unit Translation}
{$i Translation.pas}
{unit URLAccess}
{$i URLAccess.pas}
{unit ATSUnicodeDirectAccess}
{$i ATSUnicodeDirectAccess.pas}
{unit ATSUnicodeDrawing}
{$i ATSUnicodeDrawing.pas}
{unit ATSUnicodeFlattening}
{$i ATSUnicodeFlattening.pas}
{unit ATSUnicodeFonts}
{$i ATSUnicodeFonts.pas}
{unit ATSUnicodeGlyphs}
{$i ATSUnicodeGlyphs.pas}
{unit ATSUnicodeObjects}
{$i ATSUnicodeObjects.pas}
{unit AUComponent}
{$i AUComponent.pas}
{unit AppleEvents}
{$i AppleEvents.pas}
{unit AudioCodecs}
{$i AudioCodecs.pas}
{unit AudioOutputUnit}
{$i AudioOutputUnit.pas}
{unit AudioUnitProperties}
{$i AudioUnitProperties.pas}
{unit AuthorizationDB}
{$i AuthorizationDB.pas}
{unit CFFTPStream}
{$i CFFTPStream.pas}
{unit CFHTTPAuthentication}
{$i CFHTTPAuthentication.pas}
{unit CFHTTPStream}
{$i CFHTTPStream.pas}
{unit CFHost}
{$i CFHost.pas}
{unit CFNetDiagnostics}
{$i CFNetDiagnostics.pas}
{unit CFNetServices}
{$i CFNetServices.pas}
{unit CFPropertyList}
{$i CFPropertyList.pas}
{unit CFProxySupport}
{$i CFProxySupport.pas}
{unit CFSocketStream}
{$i CFSocketStream.pas}
{unit CGColor}
{$i CGColor.pas}
{unit CGContext}
{$i CGContext.pas}
{unit CGDirectDisplay}
{$i CGDirectDisplay.pas}
{unit CGDirectPalette}
{$i CGDirectPalette.pas}
{unit CGDisplayConfiguration}
{$i CGDisplayConfiguration.pas}
{unit CGDisplayFades}
{$i CGDisplayFades.pas}
{unit CGEvent}
{$i CGEvent.pas}
{unit CGGLContext}
{$i CGGLContext.pas}
{unit CGImageDestination}
{$i CGImageDestination.pas}
{unit CGLayer}
{$i CGLayer.pas}
{unit CGPDFContext}
{$i CGPDFContext.pas}
{unit CGPattern}
{$i CGPattern.pas}
{unit CTFrame}
{$i CTFrame.pas}
{unit CTLine}
{$i CTLine.pas}
{unit CTRun}
{$i CTRun.pas}
{unit CTStringAttributes}
{$i CTStringAttributes.pas}
{unit CTTypesetter}
{$i CTTypesetter.pas}
{unit CVDisplayLink}
{$i CVDisplayLink.pas}
{unit Dialogs}
{$i Dialogs.pas}
{unit Displays}
{$i Displays.pas}
{unit Drag}
{$i Drag.pas}
{unit DrawSprocket}
{$i DrawSprocket.pas}
{unit HIArchive}
{$i HIArchive.pas}
{unit ImageCompression}
{$i ImageCompression.pas}
{unit MusicDevice}
{$i MusicDevice.pas}
{unit NSL}
{$i NSL.pas}
{unit PMApplication}
{$i PMApplication.pas}
{unit PMApplicationDeprecated}
{$i PMApplicationDeprecated.pas}
{unit PMCore}
{$i PMCore.pas}
{unit QDPictToCGContext}
{$i QDPictToCGContext.pas}
{unit QLGenerator}
{$i QLGenerator.pas}
{unit Quickdraw}
{$i Quickdraw.pas}
{unit SCDynamicStore}
{$i SCDynamicStore.pas}
{unit SCDynamicStoreCopyDHCPInfos}
{$i SCDynamicStoreCopyDHCPInfos.pas}
{unit SCDynamicStoreCopySpecific}
{$i SCDynamicStoreCopySpecific.pas}
{unit SCPreferences}
{$i SCPreferences.pas}
{unit SCPreferencesPath}
{$i SCPreferencesPath.pas}
{unit SCPreferencesSetSpecific}
{$i SCPreferencesSetSpecific.pas}
{unit Sound}
{$i Sound.pas}
{unit AEHelpers}
{$i AEHelpers.pas}
{unit AEObjects}
{$i AEObjects.pas}
{unit AEPackObject}
{$i AEPackObject.pas}
{unit AERegistry}
{$i AERegistry.pas}
{unit ASRegistry}
{$i ASRegistry.pas}
{unit ATSFont}
{$i ATSFont.pas}
{unit Appearance}
{$i Appearance.pas}
{unit CFPreferences}
{$i CFPreferences.pas}
{unit CGBitmapContext}
{$i CGBitmapContext.pas}
{unit CTFramesetter}
{$i CTFramesetter.pas}
{unit CoreFoundation}
{$i CoreFoundation.pas}
{unit CoreGraphics}
{$i CoreGraphics.pas}
{unit CoreText}
{$i CoreText.pas}
{unit Dictionary}
{$i Dictionary.pas}
{unit FontPanel}
{$i FontPanel.pas}
{unit LanguageAnalysis}
{$i LanguageAnalysis.pas}
{unit Menus}
{$i Menus.pas}
{unit Movies}
{$i Movies.pas}
{unit MoviesFormat}
{$i MoviesFormat.pas}
{unit OSA}
{$i OSA.pas}
{unit OSAGeneric}
{$i OSAGeneric.pas}
{unit PMPrintAETypes}
{$i PMPrintAETypes.pas}
{unit QuickTimeMusic}
{$i QuickTimeMusic.pas}
{unit SCNetworkConfiguration}
{$i SCNetworkConfiguration.pas}
{unit AppleScript}
{$i AppleScript.pas}
{unit CarbonEvents}
{$i CarbonEvents.pas}
{unit FinderRegistry}
{$i FinderRegistry.pas}
{unit HIShape}
{$i HIShape.pas}
{unit HIToolbar}
{$i HIToolbar.pas}
{unit Icons}
{$i Icons.pas}
{unit ImageCodec}
{$i ImageCodec.pas}
{unit MacApplication}
{$i MacApplication.pas}
{unit MacWindows}
{$i MacWindows.pas}
{unit MediaHandlers}
{$i MediaHandlers.pas}
{unit Navigation}
{$i Navigation.pas}
{unit QuickTimeComponents}
{$i QuickTimeComponents.pas}
{unit QuickTimeStreaming}
{$i QuickTimeStreaming.pas}
{unit QuickTimeVR}
{$i QuickTimeVR.pas}
{unit QuickTimeVRFormat}
{$i QuickTimeVRFormat.pas}
{unit TextServices}
{$i TextServices.pas}
{unit ASDebugging}
{$i ASDebugging.pas}
{unit AudioUnitCarbonViews}
{$i AudioUnitCarbonViews.pas}
{unit Controls}
{$i Controls.pas}
{unit HIAccessibility}
{$i HIAccessibility.pas}
{unit HIButtonViews}
{$i HIButtonViews.pas}
{unit HIClockView}
{$i HIClockView.pas}
{unit HIContainerViews}
{$i HIContainerViews.pas}
{unit HILittleArrows}
{$i HILittleArrows.pas}
{unit HIPopupButton}
{$i HIPopupButton.pas}
{unit HIProgressViews}
{$i HIProgressViews.pas}
{unit HIRelevanceBar}
{$i HIRelevanceBar.pas}
{unit HISeparator}
{$i HISeparator.pas}
{unit HITabbedView}
{$i HITabbedView.pas}
{unit HITheme}
{$i HITheme.pas}
{unit HIToolboxDebugging}
{$i HIToolboxDebugging.pas}
{unit HIView}
{$i HIView.pas}
{unit HIWindowViews}
{$i HIWindowViews.pas}
{unit HTMLRendering}
{$i HTMLRendering.pas}
{unit Lists}
{$i Lists.pas}
{unit MacHelp}
{$i MacHelp.pas}
{unit MacTextEditor}
{$i MacTextEditor.pas}
{unit QTSMovie}
{$i QTSMovie.pas}
{unit QTStreamingComponents}
{$i QTStreamingComponents.pas}
{unit TSMTE}
{$i TSMTE.pas}
{unit ControlDefinitions}
{$i ControlDefinitions.pas}
{unit HIComboBox}
{$i HIComboBox.pas}
{unit HIDataBrowser}
{$i HIDataBrowser.pas}
{unit HIDisclosureViews}
{$i HIDisclosureViews.pas}
{unit HIImageViews}
{$i HIImageViews.pas}
{unit HIMenuView}
{$i HIMenuView.pas}
{unit HIMovieView}
{$i HIMovieView.pas}
{unit HIScrollView}
{$i HIScrollView.pas}
{unit HISearchField}
{$i HISearchField.pas}
{unit HISegmentedView}
{$i HISegmentedView.pas}
{unit HISlider}
{$i HISlider.pas}
{unit HITextViews}
{$i HITextViews.pas}
{unit IBCarbonRuntime}
{$i IBCarbonRuntime.pas}
{unit HIToolbox}
{$i HIToolbox.pas}

implementation

{implementation of unit MixedMode}


{$R-}

function NewRoutineDescriptor( theProc: ProcPtr; theProcInfo: ProcInfoType; theISA: ISAType ): UniversalProcPtr; inline;
begin
	NewRoutineDescriptor := UniversalProcPtr(theProc);
end;

procedure DisposeRoutineDescriptor( theUPP: UniversalProcPtr ); inline;
begin
end;

{implementation of unit CFByteOrders}


{$R-}

function CFByteOrderGetCurrent: CFByteOrder; inline;
	var
		x: UInt32 = (CFByteOrderBigEndian shl 24) or CFByteOrderLittleEndian;
begin
	CFByteOrderGetCurrent := CFByteOrder(UInt8Ptr(@x)^);
end;

function CFSwapInt16( arg: UInt16 ): UInt16; inline;
begin
	CFSwapInt16 := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function CFSwapInt32( arg: UInt32 ): UInt32; inline;
begin
    CFSwapInt32 := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function CFSwapInt64( arg: UInt64 ): UInt64; inline;
begin
	CFSwapInt64 := (UInt64(CFSwapInt32( arg and $FFFFFFFF )) shl 32) or CFSwapInt32( (arg shr 32) and $FFFFFFFF );
end;

{$ifc TARGET_RT_BIG_ENDIAN}
function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16BigToHost := arg;
end;

function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32BigToHost := arg;
end;

function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64BigToHost := arg;
end;

function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToBig := arg;
end;

function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToBig := arg;
end;

function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToBig := arg;
end;

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16LittleToHost := CFSwapInt16(arg);
end;

function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32LittleToHost := CFSwapInt32(arg);
end;

function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64LittleToHost := CFSwapInt64(arg);
end;

function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToLittle := CFSwapInt16(arg);
end;

function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToLittle := CFSwapInt32(arg);
end;

function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToLittle := CFSwapInt64(arg);
end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloat32HostToSwapped := CFSwappedFloat32(arg);
end;

function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloat32SwappedToHost := Float32(arg);
end;

function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertFloat64HostToSwapped := CFSwappedFloat64(arg);
end;

function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertFloat64SwappedToHost := Float64(arg);
end;

function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped := CFSwappedFloat32(arg);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(arg);
end;

function CFConvertDoubleHostToSwapped( arg: Float64): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped := CFSwappedFloat64(arg);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(arg);
end;

{$elsec}

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16LittleToHost := arg;
end;

function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32LittleToHost := arg;
end;

function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64LittleToHost := arg;
end;

function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToLittle := arg;
end;

function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToLittle := arg;
end;

function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToLittle := arg;
end;

function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16BigToHost := CFSwapInt16(arg);
end;

function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32BigToHost := CFSwapInt32(arg);
end;

function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64BigToHost := CFSwapInt64(arg);
end;

function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToBig := CFSwapInt16(arg);
end;

function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToBig := CFSwapInt32(arg);
end;

function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToBig := CFSwapInt64(arg);
end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloat32HostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloat32SwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertFloat64HostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertFloat64SwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;

function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;
{$endc}


{implementation of unit CGGeometry}

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
end;


function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
end;


{implementation of unit Endian}

{$R-}

function Endian16_Swap( arg: UInt16 ): UInt16; inline;
begin
	Endian16_Swap := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function Endian32_Swap( arg: UInt32 ): UInt32; inline;
begin
    Endian32_Swap := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function Endian64_Swap_Pascal( arg: UInt64 ): UInt64; inline;
begin
	Endian64_Swap_Pascal := (Endian32_Swap( arg and $FFFFFFFF ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
end;

function Endian64_Swap( arg: UInt64 ): UInt64; inline;
begin
	Endian64_Swap := Endian64_Swap_Pascal(arg);
end;

function EndianS16_Swap( arg: SInt16 ): SInt16; inline;
begin
	EndianS16_Swap := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function EndianS32_Swap( arg: SInt32 ): SInt32; inline;
begin
    EndianS32_Swap := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function EndianS64_Swap( arg: SInt64 ): SInt64; inline;
begin
	EndianS64_Swap := (SInt64( Endian32_Swap( arg and $FFFFFFFF ) ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
end;

{$ifc TARGET_RT_BIG_ENDIAN}
function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoN := arg;
end;

function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoB := arg;
end;

function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoN := arg;
end;

function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoB := arg;
end;

function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoN := arg;
end;

function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoB := arg;
end;

function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoN := arg;
end;

function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoB := arg;
end;

function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoN := arg;
end;

function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoB := arg;
end;

function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoN := arg;
end;

function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoB := arg;
end;

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoN := EndianS16_Swap(arg);
end;

function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoL := EndianS16_Swap(arg);
end;

function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoN := Endian16_Swap(arg);
end;

function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoL := Endian16_Swap(arg);
end;

function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoN := EndianS32_Swap(arg);
end;

function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoL := EndianS32_Swap(arg);
end;

function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoN := Endian32_Swap(arg);
end;

function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoL := Endian32_Swap(arg);
end;


function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoN := EndianS64_Swap(arg);
end;

function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoL := EndianS64_Swap(arg);
end;

function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoN := Endian64_Swap(arg);
end;

function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoL := Endian64_Swap(arg);
end;

{$elsec}
function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoN := EndianS16_Swap(arg);
end;

function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoB := EndianS16_Swap(arg);
end;

function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoN := Endian16_Swap(arg);
end;

function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoB := Endian16_Swap(arg);
end;

function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoN := EndianS32_Swap(arg);
end;

function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoB := EndianS32_Swap(arg);
end;

function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoN := Endian32_Swap(arg);
end;

function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoB := Endian32_Swap(arg);
end;


function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoN := EndianS64_Swap(arg);
end;

function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoB := EndianS64_Swap(arg);
end;

function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoN := Endian64_Swap(arg);
end;

function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoB := Endian64_Swap(arg);
end;

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoN := arg;
end;

function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoL := arg;
end;

function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoN := arg;
end;

function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoL := arg;
end;

function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoN := arg;
end;

function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoL := arg;
end;

function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoN := arg;
end;

function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoL := arg;
end;

function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoN := arg;
end;

function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoL := arg;
end;

function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoN := arg;
end;

function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoL := arg;
end;

{$endc}

function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoB:=EndianS16_Swap(arg);
end;

function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoB:=EndianS64_Swap(arg);
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
end;



{implementation of unit MIDIServices}
{$ifc TARGET_OS_MAC}


{$R-}
function MIDIPacketNext(pkt : MIDIPacketPtr) : MIDIPacketPtr; inline;
begin
	MIDIPacketNext := MIDIPacketPtr(@pkt^.data[pkt^.length])
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit MIDIThruConnection}
{$ifc TARGET_OS_MAC}


function MIDIThruConnectionParamsSize(ptr : MIDIThruConnectionParamsPtr) : size_t; inline;
begin
	MIDIThruConnectionParamsSize := sizeof(MIDIThruConnectionParams) +
									(ptr^.numControlTransforms * sizeof(MIDIControlTransform)) +
									(ptr^.numMaps * sizeof(MIDIValueMap))
end;
{$endc} { TARGET_OS_MAC }
{implementation of unit OSUtils}

{$ifc TARGET_OS_MAC}

{$R-}

function GetMMUMode: SInt8; inline;
begin
	GetMMUMode:= true32b
end;

procedure SwapMMUMode( var mode: SInt8 ); inline;
begin
	mode := true32b;
end;


{$endc} {TARGET_OS_MAC}

{implementation of unit CFString}


{implementation of unit Events}

{$ifc TARGET_OS_MAC and not TARGET_CPU_64}


{$ifc TARGET_RT_BIG_ENDIAN}

procedure GetKeys( var theKeys: KeyMap );
var
	theReverseKeys: KeyMap;
	theKey: 0..127;
begin
	__GetKeys( theReverseKeys);
	for theKey:= 0 to 127 do
		theKeys[ theKey]:= theReverseKeys[ ((theKey div 8) * 8) + (7 - (theKey mod 8))]
end;

{$elsec}

procedure GetKeys( var theKeys: KeyMap );
begin
	__GetKeys( theKeys)
end;

{$endc}


{$endc} {TARGET_OS_MAC and not TARGET_CPU_64}

{implementation of unit CFUserNotification}


{$R-}

function CFUserNotificationCheckBoxChecked( i: CFIndex ): CFOptionFlags; inline;
begin
	CFUserNotificationCheckBoxChecked := CFOptionFlags(1 shl (8+i));
end;

function CFUserNotificationSecureTextField( i: CFIndex ): CFOptionFlags; inline;
begin
	CFUserNotificationSecureTextField := CFOptionFlags(1 shl (16+i));
end;

function CFUserNotificationPopUpSelection( n: CFIndex ): CFOptionFlags; inline;
begin
	CFUserNotificationPopUpSelection := CFOptionFlags(n shl 24);
end;

{implementation of unit ToolUtils}

{$ifc TARGET_OS_MAC}



function HiWord(arg: SInt32): SInt16; inline;
begin
  HiWord := arg shr 16;
end;


function HiWord(arg: UInt32): UInt16; inline;
begin
  HiWord := arg shr 16;
end;


function LoWord(arg: SInt32): SInt16; inline;
begin
  LoWord := SInt16(arg);
end;


function LoWord(arg: UInt32): UInt16; inline;
begin
  LoWord := UInt16(arg);
end;
  

{$endc} {TARGET_OS_MAC}

{implementation of unit CFBundle}


{$R-}

function CFCopyLocalizedString( key: CFStringRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedString := CFBundleCopyLocalizedString( CFBundleGetMainBundle, key, key, nil );
end;

function CFCopyLocalizedStringFromTable( key: CFStringRef; tableName: CFStringRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedStringFromTable := CFBundleCopyLocalizedString( CFBundleGetMainBundle, key, key, tableName );
end;

function CFCopyLocalizedStringFromTableInBundle( key: CFStringRef; tableName: CFStringRef; bundle: CFBundleRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedStringFromTableInBundle := CFBundleCopyLocalizedString( bundle, key, key, tableName );
end;

function CFCopyLocalizedStringWithDefaultValue( key: CFStringRef; tableName: CFStringRef; bundle: CFBundleRef; value: CFStringRef; comment: PChar ): CFStringRef; inline;
begin
	CFCopyLocalizedStringWithDefaultValue := CFBundleCopyLocalizedString( bundle, key, value, tableName );
end;


{implementation of unit CFPlugInCOM}


{$R-}

function SUCCEEDED( Status: HRESULT ): Boolean; inline;
begin
	SUCCEEDED := Status >= 0;
end;

function FAILED( Status: HRESULT ): Boolean; inline;
begin
	FAILED := Status < 0;
end;

function IS_ERROR( Status: HRESULT ): Boolean; inline;
begin
	IS_ERROR := Status shr 31 = SEVERITY_ERROR;
end;

function HRESULT_CODE( hr: HRESULT ): HRESULT; inline;
begin
	HRESULT_CODE := hr and $FFFF;
end;

function HRESULT_FACILITY( hr: HRESULT ): HRESULT; inline;
begin
	HRESULT_FACILITY := (hr shr 16) and $1FFF;
end;

function HRESULT_SEVERITY( hr: HRESULT ): HRESULT; inline;
begin
	HRESULT_SEVERITY := (hr shr 31) and $01;
end;

function MAKE_HRESULT( sev, fac, code: ULONG ): HRESULT; inline;
begin
	MAKE_HRESULT := HRESULT((sev shl 31) or (fac shl 16) or code);
end;

function IUnknownUUID: CFUUIDRef; inline;
begin
	IUnknownUUID:= CFUUIDGetConstantUUIDWithBytes( nil, $00, $00, $00, $00, $00, $00, $00, $00, $C0, $00, $00, $00, $00, $00, $00, $46 )
end;


{implementation of unit MDExternalDatastore}

{$ifc TARGET_OS_MAC}


function kMDExternalDatastoreTypeID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$49,$68,$94,$B1,$00,$30,$47,$E0,$96,$11,$F2,$48,$FB,$E0,$B8,$CA)
end;

function kMDExternalDatastoreStoreInterfaceID : CFUUIDRef; inline;
begin
	kMDExternalDatastoreStoreInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$DA,$62,$12,$99,$ED,$BE,$4A,$63,$92,$39,$CB,$24,$13,$73,$E2,$07)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit MDImporter}

{$ifc TARGET_OS_MAC}


function kMDImporterTypeID: CFUUIDRef; inline;
begin
	kMDImporterTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$8B,$08,$C4,$BF,$41,$5B,$11,$D8,$B3,$F9,$00,$03,$93,$67,$26,$FC)
end;

function kMDImporterInterfaceID: CFUUIDRef; inline;
begin
	kMDImporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$6E,$BC,$27,$C4,$89,$9C,$11,$D8,$84,$AE,$00,$03,$93,$67,$26,$FC)
end;

function kMDExporterInterfaceID: CFUUIDRef; inline;
begin
	kMDExporterInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault,$B4,$1C,$60,$74,$7D,$FB,$40,$57,$96,$9D,$31,$C8,$E8,$61,$A8,$D4)
end;

<<<<<<< HEAD
{implementation of unit Menus}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{implementation of unit MIDIDriver}

{$ifc TARGET_OS_MAC}


function kMIDIDriverTypeID : CFUUIDRef; inline;
begin
	kMIDIDriverTypeID := CFUUIDGetConstantUUIDWithBytes(nil, $EC, $DE, $95, $74, $0F, $E4, $11, $D4, $BB, $1A, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterfaceID : CFUUIDRef; inline;
begin
	kMIDIDriverInterfaceID := CFUUIDGetConstantUUIDWithBytes(nil, $49, $DF, $CA, $9E, $0F, $E5, $11, $D4, $95, $0D, $00, $50, $E4, $CE, $A5, $26)
end;

function kMIDIDriverInterface2ID : CFUUIDRef; inline;
begin
	kMIDIDriverInterface2ID := CFUUIDGetConstantUUIDWithBytes(nil, $43, $C9, $8C, $3C, $30, $6C, $11, $D5, $AF, $73, $00, $30, $65, $A8, $30, $1E)
end;
{$endc}
{implementation of unit AudioUnitProperties}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
{$endc} {TARGET_OS_MAC}
=======
=======
>>>>>>> origin/fixes_2_2

function GetAudioUnitParameterDisplayType(flags : UInt32) : UInt32; inline;
begin
	GetAudioUnitParameterDisplayType := flags and kAudioUnitParameterFlag_DisplayMask
end;

function AudioUnitDisplayTypeIsLogarithmic(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsLogarithmic := GetAudioUnitParameterDisplayType(flags) = 	kAudioUnitParameterFlag_DisplayLogarithmic
end;

function AudioUnitDisplayTypeIsSquareRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquareRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquareRoot
end;

function AudioUnitDisplayTypeIsSquared(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsSquared := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplaySquared
end;

function AudioUnitDisplayTypeIsCubed(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubed := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubed
end;

function AudioUnitDisplayTypeIsCubeRoot(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsCubeRoot := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayCubeRoot
end;

function AudioUnitDisplayTypeIsExponential(flags : UInt32) : Boolean; inline;
begin
	AudioUnitDisplayTypeIsExponential := GetAudioUnitParameterDisplayType(flags) = kAudioUnitParameterFlag_DisplayExponential
end;

procedure SetAudioUnitParameterDisplayType(var flags : UInt32; displayType : UInt32); inline;
begin
	flags := (flags and (not kAudioUnitParameterFlag_DisplayMask)) or displayType
end;

{implementation of unit QLGenerator}

{$ifc TARGET_OS_MAC}


function kQLGeneratorTypeID : CFUUIDRef; inline;
begin
	kQLGeneratorTypeID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $5E, $2D, $96, $80, $50, $22, $40, $FA, $B8, $06, $43, $34, $96, $22, $E5, $B9)
end;
<<<<<<< HEAD
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

function kQLGeneratorCallbacksInterfaceID: CFUUIDRef; inline; 
begin
	kQLGeneratorCallbacksInterfaceID := CFUUIDGetConstantUUIDWithBytes(kCFAllocatorDefault, $86, $5A, $F5, $E0, $6D, $30, $43, $45, $95, $1B, $D3, $71, $05, $75, $4F, $2D)
end;

{$endc} {TARGET_OS_MAC}

{implementation of unit Menus}

{$ifc TARGET_OS_MAC}
{$ifc not TARGET_CPU_64}



procedure GetItemMark( theMenu: MenuRef; item: MenuItemIndex; var markChar: CharParameter ); inline;
var
	markCharInt: UInt16;
begin
	__GetItemMark( theMenu, item, markCharInt);
	markChar:= CharParameter( markCharInt)
end;

procedure GetItemCmd( theMenu: MenuRef; item: MenuItemIndex; var cmdChar: CharParameter ); inline;
var
	cmdCharInt: UInt16;
begin
	__GetItemCmd( theMenu, item, cmdCharInt);
	cmdChar:= CharParameter( cmdCharInt)
end;


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4


end.
