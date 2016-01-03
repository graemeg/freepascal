{
     File:       PMErrors.h
 
     Contains:   Mac OS X Printing Manager Error Codes.
 
     Copyright (c) 2001-2006,2008 Apple Inc. All Rights Reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{    Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit PMErrors;
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
uses MacErrors,PMDefinitions;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


const
// general purpose printing error codes used by various printing modules 
	kPMAllocationFailure = memFullErr;	{ out of memory error }
	kPMInternalError = kPMGeneralError;	{ internal printing error }

	kPMInvalidIndex = -30882;		{ invalid index in array }
	kPMStringConversionFailure = -30883;		{ error converting a string }
	kPMXMLParseError = -30884;		{ error parsing XML data }

	kPMInvalidJobTemplate = -30885;		{ invalid job template }
	kPMInvalidPrinterInfo = -30886;		{ invalid printer info ticket }
	kPMInvalidConnection = -30887;		{ invalid connection type }
	kPMInvalidKey = -30888;		{ invalid key in ticket or template or dictionary }
	kPMInvalidValue = -30889;		{ invalid value in ticket or template or dictionary }
	kPMInvalidAllocator = -30890;		{ invalid memory allocator }
	kPMInvalidTicket = -30891;		{ invalid job ticket }
	kPMInvalidItem = -30892;		{ invalid item in ticket or template or dictionary }
	kPMInvalidType = -30893;		{ invalid type in ticket or template or dictionary }
	kPMInvalidReply = -30894;		{ invalid reply from a remote server/client }
	kPMInvalidFileType = -30895;		{ invalid file type in queue }
	kPMInvalidObject = -30896;		{ invalid object or internal error }
	kPMInvalidPaper = -30897;		{ Invalid PMPaper. }
	kPMInvalidCalibrationTarget = -30898;		{ invalid dictionary specifying printer calibration target }
    
	{ Print Job Creator and Printing Dialog Extension error codes (-9500 to -9540) }
	kPMNoDefaultItem = -9500;
	kPMNoDefaultSettings = -9501;		{ unused; to be removed }
	kPMInvalidPDEContext = -9530;		{ invalid printing dialog extension context }
	kPMDontSwitchPDEError = -9531;		{ tells the pjc not to switch panels }
	kPMUnableToFindProcess = -9532;		{ unable to find the Finder.app process }
	kPMFeatureNotInstalled = -9533;		{ printer is feature capable, but not installed }
    
    { PrintCenter and Printer Browser error codes (-9540 to -9579) }
	kPMInvalidPBMRef = -9540;		{ invalid printer browser module reference.}
	kPMNoSelectedPrinters = -9541;		{ no selected printers or error getting selection.}
	kPMInvalidLookupSpec = -9542;		{ error retrieving lookup specification. }
	kPMSyncRequestFailed = -9543;		{ error handling sync request. }
	kPMEditRequestFailed = -9544;		{ error handling request to update Edit menu }
	kPMPrBrowserNoUI = -9545;		{ got UI function call with no UI present. }

	{ Job Ticket error codes (-9580 to -9619) }
	kPMTicketTypeNotFound = -9580;        { we can't find the ticket type in our ticket. }
	kPMUpdateTicketFailed = -9581;        { attempt to update ticket to current API failed. }
	kPMValidateTicketFailed = -9582;		{ ticket has at least one key that's invalid. }
	kPMSubTicketNotFound = -9583;        { sub ticket requested is not stored in this ticket. }
	kPMInvalidSubTicket = -9584;        { unable to add the requested sub-ticket. }
	kPMDeleteSubTicketFailed = -9585;        { sub ticket could not be deleted. }
	kPMItemIsLocked = -9586;        { item's locked flag was true when attempt made to update. }
	kPMTicketIsLocked = -9587;        { caller may not change a locked ticket. }
	kPMTemplateIsLocked = -9588;        { caller can't change the template. }
	kPMKeyNotFound = -9589;        { the requested update is for a key that doesn't exist. }
	kPMKeyNotUnique = -9590;      	{ the key passed in already exists in the ticket, can't make a new one. }
	kPMUnknownDataType = -9591;        { couldn't determine proper CF type for the value passed in. }
   
    { ClientPrintingLib (-9620 to -9629) }
	kPMCreateMessageFailed = -9620;		{ could not create message }
	kPMServerCommunicationFailed = -9621;		{ communication with print server failed }
	kPMKeyOrValueNotFound = -9623;		{ missing required key or value }
	kPMMessagingError = -9624;		{ could not connect to message port or send a message to remote client }

    { Queue Manager (-9630 to -9659) }
	kPMServerNotFound = -9630;		{ print server not found }
	kPMServerAlreadyRunning = -9631;		{ print server is already running }
	kPMServerSuspended = -9632;		{ server suspended }
	kPMServerAttributeRestricted = -9633;		{ access to attribute restricted }
	kPMFileOrDirOperationFailed = -9634;		{ file/directory operation failed }
	kPMUserOrGroupNotFound = -9635;		{ specified user/group not found }
	kPMPermissionError = -9636;		{ permission related error }
	kPMUnknownMessage = -9637;		{ unknown message }
	kPMQueueNotFound = -9638;		{ queue not found }
	kPMQueueAlreadyExists = -9639;		{ queue already exists }
	kPMQueueJobFailed = -9640;		{ could not queue a new job }
	kPMJobNotFound = -9641;		{ job not found }
	kPMJobBusy = -9642;		{ job is busy }
	kPMJobCanceled = -9643;		{ job has aborted }
	kPMDocumentNotFound = -9644;		{ document not found }
    
    { Job Manager (-9660 to -9699) }
	kPMPMSymbolNotFound = -9660;		{ a required printer module symbol is missing }
	kPMIOMSymbolNotFound = -9661; 		{ a required IO module symbol is missing }
	kPMCVMSymbolNotFound = -9662;		{ a required converter module symbol is missing }
	kPMInvalidPMContext = -9663;		{ PrinterModule context is invalid }
	kPMInvalidIOMContext = -9664;		{ IO Module context is invalid }
	kPMInvalidCVMContext = -9665;		{ Converter Module context is invalid }
	kPMInvalidJobID = -9666;		{ JobID passed from Printer Module is not valid }
	kPMNoPrinterJobID = -9667;		{ no JobID from target printer/connection }
	kPMJobStreamOpenFailed = -9668;		{ failed to open job stream }
	kPMJobStreamReadFailed = -9669;		{ failed to read from job stream }
	kPMJobStreamEndError = -9670;		{ reached end of job stream }
	kPMJobManagerAborted = -9671;		{ Job Manager is aborting }
	kPMJobGetTicketBadFormatError = -9672;		{ The XML for the printer module tickets could not be parsed. }
	kPMJobGetTicketReadError = -9673;		{ There was an unknown error reading stdout from the PrintJobMgr }
    
	{ Converters (-9700 to -9739) }
	kPMPluginNotFound = -9701;		{ Converter plugin not found }
	kPMPluginRegisterationFailed = -9702;		{ Converter Plugin error }
	kPMFontNotFound = -9703;		{ Font not found }
	kPMFontNameTooLong = -9704;		{ font name too long }
	kPMGeneralCGError = -9705;		{ CoreGraphics returned error }
	kPMInvalidState = -9706;		{ Invalid converter state }
	kPMUnexpectedImagingError = -9707;		{ An unexpected imaging error occurred }
	
	{ Printer Modules (-9740 to -9779) }

	{ IO Modules (-9780 to -9799) }					
	kPMInvalidPrinterAddress = -9780;		{ a file or connection could not be open }
	kPMOpenFailed = -9781;		{ a file or connection could not be open }
	kPMReadFailed = -9782;		{ file/connection read failed }
	kPMWriteFailed = -9783;		{ file/connection write failed }
	kPMStatusFailed = -9784;		{ connection status failed }
	kPMCloseFailed = -9785;		{ close file/connection failed }
	kPMUnsupportedConnection = -9786;		{ connection type not supported }
	kPMIOAttrNotAvailable = -9787;		{ IO attribute not available on current connection type }
	kPMReadGotZeroData = -9788;		{ Read got zero bytes, but no error. }
	{ End of list }
	kPMLastErrorCodeToMakeMaintenanceOfThisListEasier = -9799;

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
