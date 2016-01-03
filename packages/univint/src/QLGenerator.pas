{
 *  QLGenerator.h
 *  Quick Look
 *
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Copyright 2007-2010 Apple Inc.
=======
 *  Copyright 2007 Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
 *  Copyright 2007 Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
 *  Copyright 2007 Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
 *  Copyright 2007 Apple Inc.
>>>>>>> origin/cpstrnew
=======
 *  Copyright 2007 Apple Inc.
>>>>>>> origin/fixes_2.4
 *  All rights reserved.
 *
 }
{ Pascal Translation: Gorazd Krosl <gorazd_1957@yahoo.ca>, November 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Pascal Translation updated: Jonas Maebe <jonas@freepascal.org>, October 2012 }
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

unit QLGenerator;
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
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
uses MacTypes,CFBase,CFURL,CFDictionary,CFArray,CFData,CFBundle,CFUUID,CGGeometry,CGImage,CGContext,QLBase;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}



{ #pragma mark Quick Look Plug-in Info.plist Keys }

{
 * QLThumbnailMinimumSize: (ex: <real>17</real>)
 *      Minimum useful size (in points) of generated thumbnails.
 *      Generator Thumbnail callback won't be called for sizes less than
 *      the value for this key. 17 is a good minimum value.
 *
 * QLPreviewWidth, QLPreviewHeight: (ex: <real>800</real>)
 *      Preview size hint. These values are used if the generator takes
 *      too long to produce the preview.
 *
 * QLSupportsConcurrentRequests: (ex: <true/>)
 *      This tells Quick Look never to call generator callbacks twice at the same time.
 *
 * QLNeedsToBeRunInMainThread: (ex: <false/>)
 *      This tells Quick Look to call generator callbacks in a main thread.
 *
 }

{ #pragma mark Common return code for generator's callback }

const
	kQLReturnMask = $af00;
	kQLReturnNoError = noErr;
	kQLReturnHasMore = (kQLReturnMask or 10);

{ #pragma mark Thumbnail generator callback }

{!
 *      @typedef QLThumbnailRequestRef
 *      @abstract This is the type of a reference to Thumbnail requests.
 }
type
	QLThumbnailRequestRef = ^__QLThumbnailRequest; { an opaque type }
	__QLThumbnailRequest = record end;

{!
 *      @function QLThumbnailRequestGetTypeID
 *      @abstract Returns the CoreFoundation type ID for QLThumbnailRequests.
 }
function QLThumbnailRequestGetTypeID: CFTypeID; external name '_QLThumbnailRequestGetTypeID';

{!
 *      @function QLThumbnailRequestCopyURL
 *      @abstract Returns the url of the file for the thumbnail request.
 *      @param thumbnail The thumbnail request.
 *      @result The url of the file for the thumbnail request.
 }
function QLThumbnailRequestCopyURL( thumbnail: QLThumbnailRequestRef ): CFURLRef; external name '_QLThumbnailRequestCopyURL';

{!
 *      @function QLThumbnailRequestCopyOptions
 *      @abstract Returns the desired options for the thumbnail request.
 *      @param thumbnail The thumbnail request.
 *      @result The desired options for the thumbnail request.
 }
function QLThumbnailRequestCopyOptions( thumbnail: QLThumbnailRequestRef ): CFDictionaryRef; external name '_QLThumbnailRequestCopyOptions';

{!
 *      @function QLThumbnailRequestCopyContentUTI
 *      @abstract Returns the UTI for the thumbnail request.
 *      @param thumbnail The thumbnail request.
 *      @result The UTI of the content being thumbnailed, NULL if not available.
 }
function QLThumbnailRequestCopyContentUTI( thumbnail: QLThumbnailRequestRef ): CFStringRef; external name '_QLThumbnailRequestCopyContentUTI';

{!
 *      @function QLThumbnailRequestGetMaximumSize
 *      @abstract Returns the maximum desired size (in points) for the thumbnail request.
 *      @param thumbnail The thumbnail request.
 *      @result The maximum desired size (in points) for the thumbnail request.
 }
function QLThumbnailRequestGetMaximumSize( thumbnail: QLThumbnailRequestRef ): CGSize; external name '_QLThumbnailRequestGetMaximumSize';

{!
 *      @function QLThumbnailRequestGetGeneratorBundle
 *      @abstract Get the thumbnail request generator bundle.
 *      @param thumbnail The thumbnail request.
 }
function QLThumbnailRequestGetGeneratorBundle( thumbnail: QLThumbnailRequestRef ): CFBundleRef; external name '_QLThumbnailRequestGetGeneratorBundle';

{!
 *      @function QLThumbnailRequestSetDocumentObject
 *      @abstract Store some object in thumbnail request.
 *      @param thumbnail The thumbnail request.
 *      @param object The object representing the document
 *      @param callbacks Callbacks to retain/release/etc. the object.
 *      @discussion You can only call this function once per request.
 }
procedure QLThumbnailRequestSetDocumentObject( thumbnail: QLThumbnailRequestRef; objct: {const} UnivPtr; const (*var*) callbacks: CFArrayCallBacks ); external name '_QLThumbnailRequestSetDocumentObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLThumbnailRequestGetDocumentObject
 *      @abstract Get the object previously stored with QLThumbnailRequestSetDocumentObject.
 *      @param thumbnail The thumbnail request.
 *      @result The object representing the document
 }
function QLThumbnailRequestGetDocumentObject( thumbnail: QLThumbnailRequestRef ): UnivPtr; external name '_QLThumbnailRequestGetDocumentObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLThumbnailRequestSetImage
 *      @abstract Sets the thumbnail request response to image.
 *      @param thumbnail The thumbnail request.
 *      @param image The thumbnail image response.
 *      @param properties See possible properties below.
 }
procedure QLThumbnailRequestSetImage( thumbnail: QLThumbnailRequestRef; image: CGImageRef; properties: CFDictionaryRef ); external name '_QLThumbnailRequestSetImage';

{!
 *      @function QLThumbnailRequestSetImageWithData
 *      @abstract Sets the thumbnail request response to image data.
 *      @param thumbnail The thumbnail request.
 *      @param data The thumbnail image response as data. The image format should be supported by ImageIO
 *      @param properties See possible properties below. Additional useful properties: kCGImageSourceTypeIdentifierHint (see ImageIO documentation).
 }
procedure QLThumbnailRequestSetImageWithData( thumbnail: QLThumbnailRequestRef; data: CFDataRef; properties: CFDictionaryRef ); external name '_QLThumbnailRequestSetImageWithData';

{!
 *      @function QLThumbnailRequestCreateContext
 *      @abstract Creates a graphic context to draw the thumbnail response in.
 *      @param thumbnail The thumbnail request.
 *      @param size Size in points of the context for the thumbnail response.
 *      @param isBitmap True if thumbnail contents is based on bitmap. size will then be interpreted as pixels, not points.
 *      @param properties See possible properties below.
 *      @result A graphic context to draw to.
 *      @discussion Once the thumbnail is fully drawn, you should call QLThumbnailRequestFlushContext().
 }
function QLThumbnailRequestCreateContext( thumbnail: QLThumbnailRequestRef; size: CGSize; isBitmap: Boolean; properties: CFDictionaryRef ): CGContextRef; external name '_QLThumbnailRequestCreateContext';

{!
 *      @function QLThumbnailRequestFlushContext
 *      @abstract Flushes the graphic context and creates the thumbnail image response.
 *      @param thumbnail The thumbnail request.
 *      @param context The graphic context created by QLThumbnailRequestCreateContext().
 }
procedure QLThumbnailRequestFlushContext( thumbnail: QLThumbnailRequestRef; context: CGContextRef ); external name '_QLThumbnailRequestFlushContext';

{!
 *      @function QLThumbnailRequestSetImageAtURL
 *      @abstract Sets the thumbnail request response to the image contained at url.
 *      @param thumbnail The thumbnail request.
 *      @param url The url to the thumbnail image response.
 *      @param properties Currently unused.
 }
procedure QLThumbnailRequestSetImageAtURL( thumbnail: QLThumbnailRequestRef; url: CFURLRef; properties: CFDictionaryRef ); external name '_QLThumbnailRequestSetImageAtURL';

{!
 *      @function QLThumbnailRequestSetThumbnailWithDataRepresentation
 *      @abstract Sets the thumbnail request response to the image produced by the equivalent preview representation.
 *      @param thumbnail The thumbnail request.
 *      @param data The content data.
 *      @param contentTypeUTI The contentTypeUTI for the preview representation.
 *      @param previewProperties Additional properties for the preview response.
 *      @param properties Currently unused.
 *      @discussion Currently supported UTIs are: none. This call only works if your gemeratpr is set to be run in the main thread
 }
procedure QLThumbnailRequestSetThumbnailWithDataRepresentation( thumbnail: QLThumbnailRequestRef; data: CFDataRef; contentTypeUTI: CFStringRef; previewProperties: CFDictionaryRef; properties: CFDictionaryRef ); external name '_QLThumbnailRequestSetThumbnailWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLThumbnailRequestSetThumbnailWithURLRepresentation
 *      @abstract Sets the thumbnail request response to the image produced by the equivalent preview representation.
 *      @param thumbnail The thumbnail request.
 *      @param url The url to the preview response.
 *      @param contentTypeUTI The contentTypeUTI for the preview representation.
 *      @param properties Additional properties for the preview response.
 *      @discussion Currently supported UTIs are: none. This call only works if your gemeratpr is set to be run in the main thread
 }
procedure QLThumbnailRequestSetThumbnailWithURLRepresentation( thumbnail: QLThumbnailRequestRef; url: CFURLRef; contentTypeUTI: CFStringRef; previewProperties: CFDictionaryRef; properties: CFDictionaryRef ); external name '_QLThumbnailRequestSetThumbnailWithURLRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLThumbnailRequestIsCancelled
 *      @abstract Returns wether the thumbnail request was cancelled or not.
 *      @param thumbnail The thumbnail request.
 *      @result true if the request was cancelled.
 }
function QLThumbnailRequestIsCancelled( thumbnail: QLThumbnailRequestRef ): Boolean; external name '_QLThumbnailRequestIsCancelled';

{!
 *      @constant kQLThumbnailPropertyExtensionKey
 *      @abstract Value should be a CFString. The extension is used as a badge when producing an icon.
 }
var kQLThumbnailPropertyExtensionKey: CFStringRef; external name '_kQLThumbnailPropertyExtensionKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @constant kQLThumbnailPropertyExtensionKey
 *      @abstract Value should be a CGImage. The badge is used when producing an icon.
 }
var kQLThumbnailPropertyBadgeImageKey: CFStringRef; external name '_kQLThumbnailPropertyBadgeImageKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @constant kQLThumbnailPropertyExtensionKey
 *      @abstract Extends the security scope where Quick Look will accept to look at a file. Value is a path as CFString.
 *      @discussion Only useful when using QLThumbnailRequestSetImageAtURL() or QLThumbnailRequestSetThumbnailWithURLRepresentation().
 *                  By default, Quick Look will only accept files within the current document bundle.
 }
var kQLThumbnailPropertyBaseBundlePathKey: CFStringRef; external name '_kQLThumbnailPropertyBaseBundlePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ #pragma mark Preview generator callback }

{!
 *      @typedef QLPreviewRequestRef
 *      This is the type of a reference to Preview requests.
 }
type
	QLPreviewRequestRef = ^__QLPreviewRequest; { an opaque type }
	__QLPreviewRequest = record end;

{!
 *      @function QLPreviewRequestGetTypeID
 *      @abstract Returns the CoreFoundation type ID for QLPreviewRequests.
 }
function QLPreviewRequestGetTypeID: CFTypeID; external name '_QLPreviewRequestGetTypeID';

{!
 *      @constant kQLPreviewPropertyDisplayNameKey
 *      @abstract Customizes Displayed name in the preview panel. This replaces the document's display name. Value is a CFString.
 }
var kQLPreviewPropertyDisplayNameKey: CFStringRef; external name '_kQLPreviewPropertyDisplayNameKey'; (* attribute const *) // useful to customize the title of the QuickLook panel

{!
 *      @constant kQLPreviewPropertyWidthKey
 *      @abstract Gives the width (in points) of the preview. Value is a CFNumber.
 }
var kQLPreviewPropertyWidthKey: CFStringRef; external name '_kQLPreviewPropertyWidthKey'; (* attribute const *)

{!
 *      @constant kQLPreviewPropertyHeightKey
 *      @abstract Gives the height (in points) of the preview. Value is a CFNumber.
 }
var kQLPreviewPropertyHeightKey: CFStringRef; external name '_kQLPreviewPropertyHeightKey'; (* attribute const *)

{!
 *      @constant kQLPreviewPropertyBaseBundlePathKey
 *      @abstract Extends the security scope where Quick Look will accept to look at a file. Value is a path as CFString.
 *      @discussion Only useful when using QLPreviewRequestSetURLRepresentation().
 *                  By default, Quick Look will only accept files within the current document bundle.
 }
var kQLPreviewPropertyBaseBundlePathKey: CFStringRef; external name '_kQLPreviewPropertyBaseBundlePathKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @constant kQLPreviewPropertyStringEncodingKey
 *      @abstract Gives the CFStringEncoding of the preview data if the preview type is plain text. Value is a CFNumber.
 }
var kQLPreviewPropertyStringEncodingKey: CFStringRef; external name '_kQLPreviewPropertyStringEncodingKey'; (* attribute const *)

const
    kQLPreviewPDFStandardStyle						= 0;
    kQLPreviewPDFPagesWithThumbnailsOnRightStyle	= 3;
    kQLPreviewPDFPagesWithThumbnailsOnLeftStyle		= 4;
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	QLPreviewPDFStyle = UInt32;
=======
	QLPreviewPDFStyle = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	QLPreviewPDFStyle = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	QLPreviewPDFStyle = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	QLPreviewPDFStyle = UNSIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	QLPreviewPDFStyle = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4

{!
 *      @constant kQLPreviewPropertyPDFStyleKey
 *      @abstract Specify the preferred way to display PDF content. Value is a CFNumber using QLPreviewPDFStyle values.
 }
var kQLPreviewPropertyPDFStyleKey: CFStringRef; external name '_kQLPreviewPropertyPDFStyleKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @constant kQLPreviewOptionCursorKey
 *      @abstract Value is the same CFNumber passed by potential previous calls to generator's preview callback for the same document with kQLPreviewPropertyCursorKey.
 *      @discussion Use this value to provide more of the preview content.
 }
var kQLPreviewOptionCursorKey: CFStringRef; external name '_kQLPreviewOptionCursorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @constant kQLPreviewPropertyCursorKey
 *      @abstract Value should be a CFNumber. This value will be used to get more of the document's preview if necessary
 *                (and if the preview genererator returns kQLReturnHasMore)
 }
var kQLPreviewPropertyCursorKey: CFStringRef; external name '_kQLPreviewPropertyCursorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLPreviewRequestCopyURL
 *      @abstract Returns the url of the file for the preview request.
 *      @param preview The preview request.
 *      @result The url of the file for the preview request.
 }
function QLPreviewRequestCopyURL( preview: QLPreviewRequestRef ): CFURLRef; external name '_QLPreviewRequestCopyURL';

{!
 *      @function QLPreviewRequestCopyOptions
 *      @abstract Returns the desired options for the preview request.
 *      @param preview The preview request.
 *      @result The desired options for the preview request.
 }
function QLPreviewRequestCopyOptions( preview: QLPreviewRequestRef ): CFDictionaryRef; external name '_QLPreviewRequestCopyOptions';

{!
 *      @function QLPreviewRequestCopyContentUTI
 *      @abstract Returns the UTI for the preview request.
 *      @param preview The preview request.
 *      @result The UTI of the content being previewed, NULL if not available.
 }
function QLPreviewRequestCopyContentUTI( preview: QLPreviewRequestRef ): CFStringRef; external name '_QLPreviewRequestCopyContentUTI';

{!
 *      @function QLPreviewRequestGetGeneratorBundle
 *      @abstract Gets the preview request generator bundle.
 *      @param preview The preview request.
 }
function QLPreviewRequestGetGeneratorBundle( preview: QLPreviewRequestRef ): CFBundleRef; external name '_QLPreviewRequestGetGeneratorBundle';

{!
 *      @function QLPreviewRequestSetDocumentObject
 *      @abstract Store some object in preview request.
 *      @param thumbnail The preview request.
 *      @param object The object representing the document
 *      @param callbacks Callbacks to retain/release/etc. the object.
 *      @discussion You can only call this function once per request.
 }
procedure QLPreviewRequestSetDocumentObject( preview: QLPreviewRequestRef; objct: {const} UnivPtr; const (*var*) callbacks: CFArrayCallBacks ); external name '_QLPreviewRequestSetDocumentObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLPreviewRequestGetDocumentObject
 *      @abstract Get the object previously stored with QLPreviewRequestSetDocumentObject.
 *      @param preview The preview request.
 *      @result The object representing the document
 }
function QLPreviewRequestGetDocumentObject( preview: QLPreviewRequestRef ): UnivPtr; external name '_QLPreviewRequestGetDocumentObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 *      @function QLPreviewRequestIsCancelled
 *      @abstract Returns wether the preview request was cancelled or not.
 *      @param preview The preview request.
 *      @result true if the request was cancelled.
 }
function QLPreviewRequestIsCancelled( preview: QLPreviewRequestRef ): Boolean; external name '_QLPreviewRequestIsCancelled';

{ #pragma mark Set preview to some native type preview data }

{!
 * @function QLPreviewRequestSetDataRepresentation
 * @abstract Sets the preview response with the provided data.
 * @param preview The preview request.
 * @param data The content data.
 * @param contentTypeUTI The contentTypeUTI for the preview representation.
 * @param properties Additional properties for the preview response.
 * @discussion Currently supported UTIs are: kUTTypeImage, kUTTypePDF, kUTTypeHTML,
 *             kUTTypeXML, kUTTypePlainText, kUTTypeRTF, kUTTypeMovie, kUTTypeAudio
 }
procedure QLPreviewRequestSetDataRepresentation( preview: QLPreviewRequestRef; data: CFDataRef; contentTypeUTI: CFStringRef; properties: CFDictionaryRef ); external name '_QLPreviewRequestSetDataRepresentation';

{!
 *      @function QLPreviewRequestSetURLRepresentation
 *      @abstract Sets the preview request response with contents at url.
 *      @param preview The preview request.
 *      @param url The url to the preview response.
 *      @param contentTypeUTI The contentTypeUTI for the preview representation.
 *      @param properties Additional properties for the preview response.
 *      @discussion Currently supported UTIs are: kUTTypeImage, kUTTypePDF, kUTTypeHTML, kUTTypeXML, kUTTypePlainText, kUTTypeRTF, kUTTypeRTFD, kUTTypeMovie, kUTTypeAudio
 }
procedure QLPreviewRequestSetURLRepresentation( preview: QLPreviewRequestRef; url: CFURLRef; contentTypeUTI: CFStringRef; properties: CFDictionaryRef ); external name '_QLPreviewRequestSetURLRepresentation';

{ #pragma mark Draw preview in a context }

{!
 *      @function QLPreviewRequestCreateContext
 *      @abstract Creates a context to draw the preview in. Context should be flushed with QLPreviewRequestFlushContext()
 *      @param preview The preview request.
 *      @param size The size of the context.
 *      @param isBitmap true if preview is bitmap-based.
 *      @param properties Additional properties for the preview response.
 }
function QLPreviewRequestCreateContext( preview: QLPreviewRequestRef; size: CGSize; isBitmap: Boolean; properties: CFDictionaryRef ): CGContextRef; external name '_QLPreviewRequestCreateContext';

{!
 *      @function QLPreviewRequestCreatePDFContext
 *      @abstract Creates a PDF context to draw the preview in, likely to be multi-pages. Context should be flushed with QLPreviewRequestFlushContext()
 *      @param preview The preview request.
 *      @param mediaBox The media box of the context. see CGPDFContextCreate().
 *      @param auxiliaryInfo The PDF auxiliary info. see CGPDFContextCreate().
 *      @param properties Additional properties for the preview response.
 }
function QLPreviewRequestCreatePDFContext( preview: QLPreviewRequestRef; const (*var*) mediaBox: CGRect; auxiliaryInfo: CFDictionaryRef; properties: CFDictionaryRef ): CGContextRef; external name '_QLPreviewRequestCreatePDFContext';


{!
 *      @function QLPreviewRequestFlushContext
 *      @abstract Flush the context and sets the preview response.
 *      @param preview The preview request.
 *      @param context context previously created by QLPreviewRequestCreateContext() or QLPreviewRequestCreatePDFContext().
 }

procedure QLPreviewRequestFlushContext( preview: QLPreviewRequestRef; context: CGContextRef ); external name '_QLPreviewRequestFlushContext';

{ #pragma mark Provide preview as Web content using QLPreviewRequestSetDataRepresentation }

{!
 *      @constant kQLPreviewPropertyMIMETypeKey
 *      @abstract Gives the web content or attachment mime type. For the main data, default is text/html. Value is a CFString.
 }
var kQLPreviewPropertyMIMETypeKey: CFStringRef; external name '_kQLPreviewPropertyMIMETypeKey'; (* attribute const *)

{!
 *      @constant kQLPreviewPropertyTextEncodingNameKey
 *      @abstract Gives the web content or attachment text encoding. Use IANA encodings like UTF-8. Value is a CFString.
 }
var kQLPreviewPropertyTextEncodingNameKey: CFStringRef; external name '_kQLPreviewPropertyTextEncodingNameKey'; (* attribute const *)

{!
 *      @constant kQLPreviewPropertyAttachmentDataKey
 *      @abstract Gives the attachment data. Value is a CFData.
 }
var kQLPreviewPropertyAttachmentDataKey: CFStringRef; external name '_kQLPreviewPropertyAttachmentDataKey'; (* attribute const *)

{!
 *      @constant kQLPreviewPropertyAttachmentsKey
 *      @abstract Gives the list of attachments (or sub-resources). Value is a CFDictionary.
 *      @discussion Keys are the attachment ids (CFStringRef) that can be referenced with "cid:id" URL and
 *                  Values are dictionaries using kQLPreviewPropertyAttachmentDataKey,
 *                  kQLPreviewPropertyMIMETypeKey and kQLPreviewPropertyTextEncodingNameKey keys.
 }
var kQLPreviewPropertyAttachmentsKey: CFStringRef; external name '_kQLPreviewPropertyAttachmentsKey'; (* attribute const *)

{!
 *      @constant kQLPreviewContentIDScheme
 *      @abstract Is the "cid" URL scheme.
 }
var kQLPreviewContentIDScheme: CFStringRef; external name '_kQLPreviewContentIDScheme'; (* attribute const *)


{
 * Definition of the Quick Look Generator interface.
 *
 }

{ 5E2D9680-5022-40FA-B806-43349622E5B9 }
function kQLGeneratorTypeID : CFUUIDRef; inline;
{ 865AF5E0-6D30-4345-951B-D37105754F2D }
function kQLGeneratorCallbacksInterfaceID: CFUUIDRef; inline; 


{$endif} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

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

end.

{$endc} {not MACOSALLINCLUDE}
