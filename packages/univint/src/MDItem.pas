{      MDItem.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        Copyright (c) 2003-2010, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
        Copyright (c) 2003-2004, Apple Computer, Inc. All rights reserved.
}

 { Pascal Translation: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit MDItem;
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
uses MacTypes,CFBase,CFString,CFDictionary,CFArray;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{!
        @header MDItem
        MDItem is a CF-compliant object, and follows the CF conventions,
        and can be used with the CF polymorphic functions, like CFRetain().

        An MDItemRef represents a file in the Metadata database, and the
        metadata associated with the file.

        Undefined Behavior
                For functions which take an MDItemRef parameter, if this
                parameter is not a valid MDItemRef, the behavior is
                undefined. NULL is not a valid MDItemRef.

                For functions which take CF*Ref parameters, such as
                CFStringRef and CFArrayRef, if this parameter is not a
                valid CF object of the correct type, the behavior is
                undefined. NULL is not a valid CF*Ref.
                A NULL CF*Ref parameter value is allowed only where
                explicitly noted below.

                Additional constraints or allowed values on parameters
                are noted with the specific functions.

}


{!
        @typedef MDItemRef
        This is the type of a reference to MDItems.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	MDItemRef = ^__MDItem; { an opaque type }
	__MDItem = record end;
=======
	MDItemRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MDItemRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MDItemRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MDItemRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	MDItemRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{!
        @function MDItemGetTypeID
        Returns the type identifier of all MDItem instances.
}
function MDItemGetTypeID: CFTypeID; external name '_MDItemGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
        @function MDItemCreate
        Returns an metadata item for the given path.
        @param allocator The CFAllocator which should be used to allocate
                memory for the query and its sub-storage. This
                parameter may be NULL in which case the current default
                CFAllocator is used.
        @param path A path to the file for which to create the MDItem.
                [[Currently, the path must exist. MDItemRefs may or
                may not be uniqued. Use CFEqual() to compare them.]]
        @result An MDItemRef, or NULL on failure.
}
function MDItemCreate( allocator: CFAllocatorRef; path: CFStringRef ): MDItemRef; external name '_MDItemCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 @function MDItemCreateWithURL
 Returns an metadata item for the given path.
 @param allocator The CFAllocator which should be used to allocate
 memory for the query and its sub-storage. This
 parameter may be NULL in which case the current default
 CFAllocator is used.
 @param url A url to the file for which to create the MDItem.
 [[Currently, the file must exist. MDItemRefs may or
 may not be uniqued. Use CFEqual() to compare them.]]
 @result An MDItemRef, or NULL on failure.
 }
function MDItemCreateWithURL( allocator: CFAllocatorRef; url: CFURLRef ): MDItemRef; external name '_MDItemCreateWithURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
 @function MDItemsCreateWithURLs
 Returns metadata items for the given urls.
 @param allocator The CFAllocator which should be used to allocate
 memory for the array. This parameter may be NULL in which case the current default
 CFAllocator is used.
 @param urls A CFArray of urls to the file for which to create the MDItem.
 @result A CFArrayRef of MDItemRefs, or NULL on failure. Missing items will have kCFNull entries in the result array.
 }
function MDItemsCreateWithURLs( allocator: CFAllocatorRef; urls: CFArrayRef ): CFArrayRef; external name '_MDItemsCreateWithURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)

{!
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
        @function MDItemCopyAttribute
        Returns the value of the given attribute for the item.
        @param item The item to be interrogated.
        @param name The name of the desired attribute.
        @result A CFTypeRef, or NULL on failure, or if the attribute
                does not exist, of if the attribute is not readable.
}
function MDItemCopyAttribute( item: MDItemRef; name: CFStringRef ): CFTypeRef; external name '_MDItemCopyAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
        @function MDItemCopyAttributes
        Returns the values of the given attributes for the item.
        @param item The item to be interrogated.
        @param names A CFArray of the names of the desired attributes.
        @result A CFDictionary where the keys are the attribute names,
                and the values are the attribute values, or NULL on
                failure. If an attribute does not exist, or is
                unreadable, there will be no key-value pair for it
                in the dictionary.
}
function MDItemCopyAttributes( item: MDItemRef; names: CFArrayRef ): CFDictionaryRef; external name '_MDItemCopyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
        @function MDItemCopyAttributeList
        Returns the values of the given attributes for the item.
        @param item The item to be interrogated.
        @param ... A comma-separated varargs list of the string
                attribute names.
        @result A CFDictionary where the keys are the attribute names,
                and the values are the attribute values, or NULL on
                failure. If an attribute does not exist, or is
                unreadable, there will be no key-value pair for
                it in the dictionary.
}
{ GK NOTE: list must be NULL terminated }
function MDItemCopyAttributeList( item: MDItemRef; ... { CFStringRef names } ): CFDictionaryRef; external name '___MDItemCopyAttributesEllipsis1';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
        @function MDItemCopyAttributeNames
        Returns an array of the attribute names existing in the item.
        @param item The item to be interrogated.
        @result A CFArray of CFString attribute names, or NULL on
                failure.
}
function MDItemCopyAttributeNames( item: MDItemRef ): CFArrayRef; external name '_MDItemCopyAttributeNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{!
 @function MDItemsCopyAttributes
 Returns metadata for the given items.
 @param items A CFArray of MDItemRefs to items for which to fetch data
 @param names A CFArray of attribute names for which to fetch data. 
				The attribute names are CFStrings
 @result A CFArrayRef, or NULL on failure. Each entry in the array is either kCFNull, 
  if the item is not accessible, or a CFArray of attribute values. 
  If an attribute is not available, there will be a kCFNull in its slot in the nested array.
 }
function MDItemsCopyAttributes( items: CFArrayRef; names: CFArrayRef ): CFArrayRef; external name '_MDItemsCopyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)
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

{ List of well-known attributes }

{!
   @constant kMDItemAttributeChangeDate
   This is the date that the last metadata attribute was changed.

   @constant kMDItemContentType
   UTI Type pedigree for a file for example a jpeg file will have the
   following ItemContentType public.jpeg/public.image/public.data
   the kMDItemContentType is set by the sniffer, any changes to this
   value will get over written by the sniffer when the contents of
   this MDItemRef changes. Type is a CFStringRef

   @constant kMDItemKeywords
   Represents keywords associated with this particular
   MDItemRef. Example Keywords might be Birthday,Important etc. Type
   is a CFArray of CFStrings

   @constant kMDItemTitle
   The title of this particular MDItemRef. Title of the document, or
   it could be the title of this mp3 or a subject of a mail
   message. Type is a CFString

   @constant kMDItemAuthors
   The list of author/authors that has worked on this file. There
   could be 0 or more authors of a particular file. The order of the
   authors in the array is preserved, but is not intended to represent
   the main author or relative importance of the authors. Type is a
   CFArray of CFStrings.
 
   @constant kMDItemEditors
   The list of editor/editors that has worked on this file. There
   could be 0 or more editors of a particular file. The order of the
   editors in the array is preserved, but is not intended to represent
   the main editor or relative importance of the editors. Type is a
   CFArray of CFStrings.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   @constant kMDItemParticipants
   The list of people who are visible in an image or movie or
   written about in a document. Type is CFArray of CFStrings.

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
   @constant kMDItemProjects
   The list of projects etc that this file is part of. For example if
   you were working on a movie, all of the movie files could be marked
   as belonging to the project "My movie" then a query could be done
   kMDItemProjects = "My movie" and all of the related files would
   show up. Type is a CFArray of CFStrings

   @constant kMDItemComment
   This is a comment related to a file, and can be any random
   string. Type is a CFString

   @constant kMDItemCopyright
   This is the copyright of the content. Type is a CFString
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   
   @constant kMDItemDownloadedDate
   This is the date that the file was last downloaded / received.
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

   @constant kMDItemWhereFroms
   This attribute indicates where the item was obtained from.
   Examples:
   - downloaded file may refer to the site they were downloaded from,
    the refering URL, etc
  - files reveived by email may indicate who sent the file, the
    message subject, etc
   Type is a CFArray of CFStrings

   @constant kMDItemLastUsedDate
   This is the date that the file was last used, this field is updated
   by LaunchServices everytime a file is opend by double clicking or
   by asking LaunchServices to open a file. Type is a CFDate

   @constant kMDItemContentCreationDate
   This is the date that the contents of the file were created,
   has an application specific semantic.
   Type is a CFDate.

   @constant kMDItemContentModificationDate
   This is the date that the contents of the file were last
   modified, has an application specific semantic. For example an
   application can use this field to mark when the file was last
   modified, this date is not related to the file system modification
   date, but can be independent of that. This allows tracking of the
   last time the content was modified irrespective of the last time the
   file was modified. Type is a CFDate.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   
   @constant kMDItemDateAdded
   This is the date that the file was moved into the current location.
   Not all files will have this attribute.  Not all file systems support
   this attribute.
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

   @constant kMDItemDurationSeconds
   This is the duration, in seconds, of the content of the file (if
   appropriate).  A value of 10.5 represents media whose content is
   10 and 1/2 seconds long.  Type is a CFNumber.

   @constant kMDItemContactKeywords
   A list of contacts that are somehow associated with this document,
   beyond what is captured as Author.

   @constant kMDItemVersion
   A version number for this item. Type is a CFString

   @constant kMDItemPixelHeight
   The height of the document in pixels (ie Image height or Video frame height)

   @constant kMDItemPixelWidth
   The width of the document in pixels (ie Image width or Video frame width)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   @constant kMDItemPixelCount
   The total number of pixels in the document.  Type is a CFNumber.

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
   @constant kMDItemColorSpace
   What color space model is this document following
   (For example, are examples "RGB", "CMYK", "YUV", "YCbCr")

   @constant kMDItemBitsPerSample
   Number of bits per sample
   For example bit depth of an image (8-bit, 16-bit etc..) or bit
   depth per audio sample of uncompressed audio data (8, 16, 24, 32,
   64, etc..)

   @constant kMDItemFlashOnOff
   Indicates if the flash was used to take the picture. 0 means flash did not fire

   @constant kMDItemFocalLength
   The actual focal length of the lens in mm.

   @constant kMDItemAcquisitionMake
   Device make that was used to acquire this document

   @constant kMDItemAcquisitionModel
   Device model that was used to acquire this document

   @const kMDItemISOSpeed
   The ISO Speed the camera was set to when the image was
   taken. Examples are 100, 200, 400, etc.

   @const kMDItemOrientation
   The orientation of the data. Values are 0 is "Landscape" or 1 is "Portrait"

   @const kMDItemLayerNames
   The names of the various layers in the file

   @const kMDItemWhiteBalance
   The white balance setting of the camera when the image was
   acquired. 0 is auto white balance and 1 is manual

   @const kMDItemAperture
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   The size of the lens aperture as a log-scale APEX value
   when the image was acquired.
=======
   The aperture setting of the camera when the image was
   acquired. This unit is the APEX value.
>>>>>>> graemeg/cpstrnew
=======
   The aperture setting of the camera when the image was
   acquired. This unit is the APEX value.
>>>>>>> graemeg/cpstrnew
=======
   The aperture setting of the camera when the image was
   acquired. This unit is the APEX value.
>>>>>>> graemeg/cpstrnew
=======
   The aperture setting of the camera when the image was
   acquired. This unit is the APEX value.
>>>>>>> origin/cpstrnew
=======
   The aperture setting of the camera when the image was
   acquired. This unit is the APEX value.
>>>>>>> origin/fixes_2.4

   @const kMDItemProfileName
   Name of the color profile used for the image

   @const kMDItemResolutionWidthDPI
   Resolution width of this image in DPI

   @const kMDItemResolutionHeightDPI
   Resolution height of this image in DPI

   @const kMDItemExposureMode
   Mode that was used for the exposure. 0 is auto exposure, 1 is
   manual, and 2 is auto bracket.

   @const kMDItemExposureTimeSeconds
   Time that the lens was open during exposure

   @const kMDItemEXIFVersion
   The verion of the EXIF header that was used to generate the metadata

   @const kMDItemEXIFGPSVersion
   The version of GPSInfoIFD header that was used to generate the metadata
 
   @const kMDItemCodecs
   The codecs used to encode/decode the media

   @const kMDItemMediaTypes
   Media types present in the content

   @const kMDItemStreamable
   Whether the content is prepared for streaming

   @const kMDItemTotalBitRate
   The total byte rate (audio & video combined) of the media

   @const kMDItemVideoBitRate
   The video byte rate

   @const kMDItemAudioBitRate
   The audio byte rate

   @const kMDItemDeliveryType
   Delivery type Fast start or RTSP

   @constant kMDItemAlbum
   The title for a collection of media. This is analagous to a record album,
   or photo album whichs are collections of audio or images. Type is a CFString.

   @constant kMDItemHasAlphaChannel
   Boolean indicating if this image file has an alpha channel. Type is
   a CFBoolean.

   @constant kMDItemRedEyeOnOff
   Indicates if the flash was used to take the picture. 0 means no
   red-eye reduction mode or unknown. 1 means red-eye reduction
   supported.

   @const kMDItemMeteringMode
   The metering mode (Unknown, Average, CenterWeightedAverage, Spot,
   MultiSpot, Pattern, Partial)

   @const kMDItemMaxAperture
   The smallest F number of the lens. The unit is the APEX
   value. Ordinarily it is given in the range of 00.00 to 99.99.

   @const kMDItemFNumber
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   The focal length of the lens divided by the diameter of the aperture
   when the image was acquired.
=======
   FNumber expresses the diameter of the diaphragm aperture in terms
   of the effective focal length of the lens.
>>>>>>> graemeg/cpstrnew
=======
   FNumber expresses the diameter of the diaphragm aperture in terms
   of the effective focal length of the lens.
>>>>>>> graemeg/cpstrnew
=======
   FNumber expresses the diameter of the diaphragm aperture in terms
   of the effective focal length of the lens.
>>>>>>> graemeg/cpstrnew
=======
   FNumber expresses the diameter of the diaphragm aperture in terms
   of the effective focal length of the lens.
>>>>>>> origin/cpstrnew
=======
   FNumber expresses the diameter of the diaphragm aperture in terms
   of the effective focal length of the lens.
>>>>>>> origin/fixes_2.4

   @const kMDItemExposureProgram
   The class of the program used by the camera to set exposure when
   the picture is taken (Manual, Normal, Aperture priority, ...)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   @const kMDItemExposureTimeString
=======
   const kMDItemExposureTimeString
>>>>>>> graemeg/cpstrnew
=======
   const kMDItemExposureTimeString
>>>>>>> graemeg/cpstrnew
=======
   const kMDItemExposureTimeString
>>>>>>> graemeg/cpstrnew
=======
   const kMDItemExposureTimeString
>>>>>>> origin/cpstrnew
=======
   const kMDItemExposureTimeString
>>>>>>> origin/fixes_2.4
   The time  of the exposure.

   @const kMDItemHeadline
   A publishable entry providing a synopsis of the contents of the
   objectdata.

   @const kMDItemInstructions
   Other editorial instructions concerning the use of the objectdata,
   such as embargoes and warnings.

   @const kMDItemCity
   Identifies city of objectdata origin according to guidelines
   established by the provider.

   @const kMDItemStateOrProvince
   Identifies Province/State of origin according to guidelines
   established by the provider.

   @const kMDItemCountry
   Provides full, publishable, name of the country/primary location
   where the intellectual property of the objectdata was created,
   according to guidelines of the provider.
 
 @const kMDItemEXIFGPSVersion
 The version of GPSInfoIFD in EXIF used to generate the metadata.
 
 @const kMDItemAltitude
 The altitude of the item in meters above sea level, expressed 
 using the WGS84 datum.  Negative values lie below sea level.
 
 @const kMDItemLatitude
 The latitude of the item in degrees north of the equator, expressed
 using the WGS84 datum.  Negative values lie south of the equator.
 
 @const kMDItemLongitude
 The longitude of the item in degrees east of the prime meridian,
 expressed using the WGS84 datum.  Negative values lie west of the prime meridian.
 
 @const kMDItemTimestamp
 The timestamp on the item.  This generally is used to indicate the time at
 which the event captured by the item took place.
 
 @const kMDItemSpeed
 The speed of the item, in kilometers per hour.
 
 @const kMDItemGPSTrack
 The direction of travel of the item, in degrees from true north.
 
 @const kMDItemImageDirection
 The direction of the item's image, in degrees from true north.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 @const kMDItemNamedLocation
 The name of the location or point of interest associated with the item.
 The name may be user provided.
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
}

var kMDItemAttributeChangeDate: CFStringRef; external name '_kMDItemAttributeChangeDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)       // CFDate
var kMDItemContentType: CFStringRef; external name '_kMDItemContentType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFString
var kMDItemContentTypeTree: CFStringRef; external name '_kMDItemContentTypeTree'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // CFArray of CFStringRef
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Array of CFStringRef
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Array of CFStringRef
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Array of CFStringRef
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Array of CFStringRef
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Array of CFStringRef
>>>>>>> origin/fixes_2.4
var kMDItemKeywords: CFStringRef; external name '_kMDItemKeywords'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
var kMDItemTitle: CFStringRef; external name '_kMDItemTitle'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                     // CFString
var kMDItemAuthors: CFStringRef; external name '_kMDItemAuthors'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFArray of CFString
var kMDItemEditors: CFStringRef; external name '_kMDItemEditors'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)           // CFArray of CFString
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kMDItemParticipants: CFStringRef; external name '_kMDItemParticipants'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *) // CFArray of CFString
var kMDItemProjects: CFStringRef; external name '_kMDItemProjects'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
var kMDItemDownloadedDate: CFStringRef; external name '_kMDItemDownloadedDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *) // CFDate
=======
var kMDItemProjects: CFStringRef; external name '_kMDItemProjects'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
>>>>>>> graemeg/cpstrnew
=======
var kMDItemProjects: CFStringRef; external name '_kMDItemProjects'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
>>>>>>> graemeg/cpstrnew
=======
var kMDItemProjects: CFStringRef; external name '_kMDItemProjects'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
>>>>>>> graemeg/cpstrnew
=======
var kMDItemProjects: CFStringRef; external name '_kMDItemProjects'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
>>>>>>> origin/cpstrnew
=======
var kMDItemProjects: CFStringRef; external name '_kMDItemProjects'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFArray of CFString
>>>>>>> origin/fixes_2.4
var kMDItemWhereFroms: CFStringRef; external name '_kMDItemWhereFroms'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFArray of CFString
var kMDItemComment: CFStringRef; external name '_kMDItemComment'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFString
var kMDItemCopyright: CFStringRef; external name '_kMDItemCopyright'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                 // CFString
var kMDItemLastUsedDate: CFStringRef; external name '_kMDItemLastUsedDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFDate
var kMDItemContentCreationDate: CFStringRef; external name '_kMDItemContentCreationDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)       // CFDate
var kMDItemContentModificationDate: CFStringRef; external name '_kMDItemContentModificationDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)   // CFDate
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kMDItemDateAdded: CFStringRef; external name '_kMDItemDateAdded'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                 // CFDate
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
var kMDItemDurationSeconds: CFStringRef; external name '_kMDItemDurationSeconds'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFNumber
var kMDItemContactKeywords: CFStringRef; external name '_kMDItemContactKeywords'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFArray of CFString
var kMDItemVersion: CFStringRef; external name '_kMDItemVersion'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFString

var kMDItemPixelHeight: CFStringRef; external name '_kMDItemPixelHeight'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFNumber
var kMDItemPixelWidth: CFStringRef; external name '_kMDItemPixelWidth'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFNumber
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kMDItemPixelCount: CFStringRef; external name '_kMDItemPixelCount'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *) // CFNumber
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
var kMDItemColorSpace: CFStringRef; external name '_kMDItemColorSpace'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFString
var kMDItemBitsPerSample: CFStringRef; external name '_kMDItemBitsPerSample'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // CFNumber
var kMDItemFlashOnOff: CFStringRef; external name '_kMDItemFlashOnOff'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFBoolean
var kMDItemFocalLength: CFStringRef; external name '_kMDItemFocalLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFNumber
var kMDItemAcquisitionMake: CFStringRef; external name '_kMDItemAcquisitionMake'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFString
var kMDItemAcquisitionModel: CFStringRef; external name '_kMDItemAcquisitionModel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)          // CFString
var kMDItemISOSpeed: CFStringRef; external name '_kMDItemISOSpeed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFNumber
var kMDItemOrientation: CFStringRef; external name '_kMDItemOrientation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFNumber
var kMDItemLayerNames: CFStringRef; external name '_kMDItemLayerNames'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFArray of CFString
var kMDItemWhiteBalance: CFStringRef; external name '_kMDItemWhiteBalance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFNumber
var kMDItemAperture: CFStringRef; external name '_kMDItemAperture'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFNumber
var kMDItemProfileName: CFStringRef; external name '_kMDItemProfileName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFString
var kMDItemResolutionWidthDPI: CFStringRef; external name '_kMDItemResolutionWidthDPI'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)        // CFNumber
var kMDItemResolutionHeightDPI: CFStringRef; external name '_kMDItemResolutionHeightDPI'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)       // CFNumber
var kMDItemExposureMode: CFStringRef; external name '_kMDItemExposureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFNumber
var kMDItemExposureTimeSeconds: CFStringRef; external name '_kMDItemExposureTimeSeconds'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)       // CFNumber
var kMDItemEXIFVersion: CFStringRef; external name '_kMDItemEXIFVersion'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFString

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kMDItemCameraOwner: CFStringRef; external name '_kMDItemCameraOwner'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)       // CFString
var kMDItemFocalLength35mm: CFStringRef; external name '_kMDItemFocalLength35mm'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)   // CFNumber
var kMDItemLensModel: CFStringRef; external name '_kMDItemLensModel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)         // CFString

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
var kMDItemEXIFGPSVersion: CFStringRef; external name '_kMDItemEXIFGPSVersion'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)    // CFString
var kMDItemAltitude: CFStringRef; external name '_kMDItemAltitude'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFNumber
var kMDItemLatitude: CFStringRef; external name '_kMDItemLatitude'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFNumber
var kMDItemLongitude: CFStringRef; external name '_kMDItemLongitude'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)         // CFNumber
var kMDItemSpeed: CFStringRef; external name '_kMDItemSpeed'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)             // CFNumber
var kMDItemTimestamp: CFStringRef; external name '_kMDItemTimestamp'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)         // CFDate
var kMDItemGPSTrack: CFStringRef; external name '_kMDItemGPSTrack'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFNumber
var kMDItemImageDirection: CFStringRef; external name '_kMDItemImageDirection'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)    // CFNumber
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kMDItemNamedLocation: CFStringRef; external name '_kMDItemNamedLocation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *) // CFString

var kMDItemGPSStatus: CFStringRef; external name '_kMDItemGPSStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)             // CFString
var kMDItemGPSMeasureMode: CFStringRef; external name '_kMDItemGPSMeasureMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)        // CFString
var kMDItemGPSDOP: CFStringRef; external name '_kMDItemGPSDOP'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)                // CFNumber
var kMDItemGPSMapDatum: CFStringRef; external name '_kMDItemGPSMapDatum'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)           // CFString
var kMDItemGPSDestLatitude: CFStringRef; external name '_kMDItemGPSDestLatitude'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)       // CFNumber
var kMDItemGPSDestLongitude: CFStringRef; external name '_kMDItemGPSDestLongitude'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)      // CFNumber
var kMDItemGPSDestBearing: CFStringRef; external name '_kMDItemGPSDestBearing'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)        // CFNumber
var kMDItemGPSDestDistance: CFStringRef; external name '_kMDItemGPSDestDistance'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)       // CFNumber
var kMDItemGPSProcessingMethod: CFStringRef; external name '_kMDItemGPSProcessingMethod'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)   // CFString
var kMDItemGPSAreaInformation: CFStringRef; external name '_kMDItemGPSAreaInformation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)    // CFString
var kMDItemGPSDateStamp: CFStringRef; external name '_kMDItemGPSDateStamp'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)          // CFDate
var kMDItemGPSDifferental: CFStringRef; external name '_kMDItemGPSDifferental'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)        // CFNumber
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

var kMDItemCodecs: CFStringRef; external name '_kMDItemCodecs'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFArray of CFString
var kMDItemMediaTypes: CFStringRef; external name '_kMDItemMediaTypes'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFArray of CFString
var kMDItemStreamable: CFStringRef; external name '_kMDItemStreamable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFBoolean
var kMDItemTotalBitRate: CFStringRef; external name '_kMDItemTotalBitRate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFNumber
var kMDItemVideoBitRate: CFStringRef; external name '_kMDItemVideoBitRate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFNumber
var kMDItemAudioBitRate: CFStringRef; external name '_kMDItemAudioBitRate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFNumber
var kMDItemDeliveryType: CFStringRef; external name '_kMDItemDeliveryType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFString

var kMDItemAlbum: CFStringRef; external name '_kMDItemAlbum'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                     // CFString
var kMDItemHasAlphaChannel: CFStringRef; external name '_kMDItemHasAlphaChannel'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		   // CFBoolean
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		  // CFBoolean
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		  // CFBoolean
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		  // CFBoolean
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		  // CFBoolean
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		  // CFBoolean
>>>>>>> origin/fixes_2.4
var kMDItemRedEyeOnOff: CFStringRef; external name '_kMDItemRedEyeOnOff'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFBoolean
var kMDItemMeteringMode: CFStringRef; external name '_kMDItemMeteringMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFString
var kMDItemMaxAperture: CFStringRef; external name '_kMDItemMaxAperture'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFNumber
var kMDItemFNumber: CFStringRef; external name '_kMDItemFNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFNumber
var kMDItemExposureProgram: CFStringRef; external name '_kMDItemExposureProgram'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFString
var kMDItemExposureTimeString: CFStringRef; external name '_kMDItemExposureTimeString'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)        // CFString

{ From IPTC }
var kMDItemHeadline: CFStringRef; external name '_kMDItemHeadline'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFString
var kMDItemInstructions: CFStringRef; external name '_kMDItemInstructions'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)              // CFString
var kMDItemCity: CFStringRef; external name '_kMDItemCity'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                      // CFString
var kMDItemStateOrProvince: CFStringRef; external name '_kMDItemStateOrProvince'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFString
var kMDItemCountry: CFStringRef; external name '_kMDItemCountry'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFString

{!
   @constant kMDItemTextContent
   Contains the text content of the document. Type is a CFString.

   @constant kMDItemDisplayName
   This is the localized version of the LaunchServices call
   LSCopyDisplayNameForURL()/LSCopyDisplayNameForRef().
 
   @constant kMDItemFSName
   This is the file name of the MDItemRef. Type is a CFString

   @constant kMDItemPath
   This is the complete path to the MDItemRef. Type is a CFString.

   @constant kMDItemFSSize
   The total logical size of the file (data and resources) on disk in bytes. Type is a CFNumber.

   @constant kMDItemFSCreationDate
   This is the date that the file was created. Type is a CFDate.

   @constant  kMDItemFSContentChangeDate
   This is the date the the file content last changed. This is a CFDate.

   @constant kMDItemFSOwnerUserID
   User-id of owner of the file. Type is a CFNumber.

   @constant kMDItemFSOwnerGroupID
   Group-id of owner of the file. Type is a CFNumber.

   @constant kMDItemFSExists *** DEPRECATED ***
   Boolean indicating if this MDItem references a file that still
   exists. The file that the MDItem references might have been
   deleted. Type is a CFBoolean.

   @constant kMDItemFSIsReadable *** DEPRECATED ***
   Boolean indicating if this file is readable. Type is a CFBoolean.

   @constant kMDItemFSIsWriteable *** DEPRECATED ***
   Boolean indicating if this file is writable. Type is a CFBoolean.

   @constant kMDItemFSNodeCount
   Number of files in directory. Type is a CFNumber.

   @constant kMDItemFSHasCustomIcon
   Boolean indicating if this file has a custom icon. Type is a CFBoolean.

   @constant kMDItemFSIsExtensionHidden
   Boolean indicating if this file has its extension hidden. Type is a CFBoolean.

   @constant kMDItemFSIsStationery
   Boolean indicating if this file is stationery. Type is a CFBoolean.

   @constant kMDItemFSInvisible
   Boolean indicating if this file is visible. Type is a CFBoolean.

   @constant kMDItemFSLabel
   Number indicating which finder label is in use (0-7). Type is a CFNumber.

}

var kMDItemFSName: CFStringRef; external name '_kMDItemFSName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFString
var kMDItemDisplayName: CFStringRef; external name '_kMDItemDisplayName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // CFString
var kMDItemPath: CFStringRef; external name '_kMDItemPath'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFString
var kMDItemFSSize: CFStringRef; external name '_kMDItemFSSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                  // CFNumber
var kMDItemFSCreationDate: CFStringRef; external name '_kMDItemFSCreationDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)          // CFDate
var kMDItemFSContentChangeDate: CFStringRef; external name '_kMDItemFSContentChangeDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)     // CFDate
var kMDItemFSOwnerUserID: CFStringRef; external name '_kMDItemFSOwnerUserID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFNumber
var kMDItemFSOwnerGroupID: CFStringRef; external name '_kMDItemFSOwnerGroupID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)          // CFNumber
var kMDItemFSExists: CFStringRef; external name '_kMDItemFSExists'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED *)
var kMDItemFSIsReadable: CFStringRef; external name '_kMDItemFSIsReadable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED *)
var kMDItemFSIsWriteable: CFStringRef; external name '_kMDItemFSIsWriteable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED *)
var kMDItemFSHasCustomIcon: CFStringRef; external name '_kMDItemFSHasCustomIcon'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)         // CFBoolean
var kMDItemFSIsExtensionHidden: CFStringRef; external name '_kMDItemFSIsExtensionHidden'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)     // CFBoolean
var kMDItemFSIsStationery: CFStringRef; external name '_kMDItemFSIsStationery'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)          // CFBoolean
var kMDItemFSInvisible: CFStringRef; external name '_kMDItemFSInvisible'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // CFBoolean
var kMDItemFSLabel: CFStringRef; external name '_kMDItemFSLabel'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                 // CFNumber
var kMDItemFSNodeCount: CFStringRef; external name '_kMDItemFSNodeCount'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // CFNumber

var kMDItemTextContent: CFStringRef; external name '_kMDItemTextContent'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // a Write-Only CFString

{!
   @constant kMDItemAudioSampleRate
   The sample rate of the audio data contained in the file. The sample rate is a
   float value representing hz (audio_frames/second). For example: 44100.0, 22254.54.
   Type is a CFNumber (float).

   @constant kMDItemAudioChannelCount
   The number of channels in the audio data contained in the file. This item only represents
   the number of discreet channels of audio data found in the file. It does not indicate
   any configuration of the data in regards to a user's speaker setup.
   Type is a CFNumber (integer).

   @constant kMDItemTempo
   The tempo of the music contained in the audio file in Beats Per Minute.
   Type is a CFNumber (float).

   @constant kMDItemKeySignature
   The musical key of the song/composition contained in an audio file.
   For example: C, Dm, F#m, Bb. Type is a CFString.

   @constant kMDItemTimeSignature
   The time signature of the musical composition contained in the audio/MIDI file.
   For example: "4/4", "7/8". Type is a CFString.

   @constant kMDItemAudioEncodingApplication
   The name of the application that encoded the data contained in the audio file.
   Type is a CFString.

   @constant kMDItemComposer
   The composer of the song/composition contained in the audio file.
   Type is a CFString.

   @constant kMDItemLyricist
   The lyricist/text writer for song/composition contained in the audio file.
   Type is a CFString.

   @constant kMDItemAudioTrackNumber
   The track number of a song/composition when it is part of an album (kMDItemAlbum).
   Type is a CFNumber (integer).

   @constant kMDItemRecordingDate
   The recording date of the song/composition. This information differs from
   the kMDItemContentCreationDate attribute as it indicates the date that the
   'art' was created, in contrast to ContentCreationDate which for example, could indicate
   the creation date of an edited or 'mastered' version of the original art.
   Type is a CFDate.

   @constant kMDItemMusicalGenre
   The musical genre of the song/composition contained in the audio file.
   For example: Jazz, Pop, Rock, Classical. Type is a CFString.

   @constant kMDItemIsGeneralMIDISequence
   This attribute indicates whether the MIDI sequence contained in the file
   is setup for use with a General MIDI device. Type is a CFBoolean.

   @const kMDItemRecordingYear
   This attribute indicates what year the item was recorded on.
   Type is a CFNumber
}

var kMDItemAudioSampleRate: CFStringRef; external name '_kMDItemAudioSampleRate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                 // CFNumber
var kMDItemAudioChannelCount: CFStringRef; external name '_kMDItemAudioChannelCount'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFNumber
var kMDItemTempo: CFStringRef; external name '_kMDItemTempo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                           // CFNumber
var kMDItemKeySignature: CFStringRef; external name '_kMDItemKeySignature'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFString
var kMDItemTimeSignature: CFStringRef; external name '_kMDItemTimeSignature'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFString
var kMDItemAudioEncodingApplication: CFStringRef; external name '_kMDItemAudioEncodingApplication'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)        // CFString
var kMDItemComposer: CFStringRef; external name '_kMDItemComposer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                        // CFString
var kMDItemLyricist: CFStringRef; external name '_kMDItemLyricist'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                        // CFString
var kMDItemAudioTrackNumber: CFStringRef; external name '_kMDItemAudioTrackNumber'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFNumber
var kMDItemRecordingDate: CFStringRef; external name '_kMDItemRecordingDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFDate
var kMDItemMusicalGenre: CFStringRef; external name '_kMDItemMusicalGenre'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFString
var kMDItemIsGeneralMIDISequence: CFStringRef; external name '_kMDItemIsGeneralMIDISequence'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)           // CFBoolean
var kMDItemRecordingYear: CFStringRef; external name '_kMDItemRecordingYear'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFNumber

{!
        @const kMDItemOrganizations
        Used to indicate company/Organization that created the document.
        Type is a CFArray of CFStrings.

        @const kMDItemLanguages
        Used to designate the languages of the intellectual content of the
        resource. Recommended best practice for the values of the Language
        element is defined by RFC 3066.
        Type is a CFArray of CFStrings.

        @const kMDItemRights
        Used to provide a link to information about rights held in and
        over the resource. Typically a Rights element will contain a
        rights management statement for the resource, or reference a
        service providing such information. Rights information often
        encompasses Intellectual Property Rights (IPR), Copyright, and
        various Property Rights. If the rights element is absent, no
        assumptions can be made about the status of these and other rights
        with respect to the resource. Type is a CFString type.

        @const kMDItemPublishers
        Used to designate the entity responsible for making the resource
        available. Examples of a Publisher include a person, an
        organization, or a service. Typically, the name of a Publisher
        should be used to indicate the entity. Type is a CFArray of CFStrings.

        @const kMDItemContributors
        Used to designate the entity responsible for making contributions
        to the content of the resource. Examples of a Contributor include
        a person, an organization or a service. Typically, the name of a
        Contributor should be used to indicate the entity. Type is a
        CFArray of CFStrings.

        @const kMDItemCoverage
        Used to designate the extent or scope of the content of the
        resource. Coverage will typically include spatial location (a
        place name or geographic co-ordinates), temporal period (a period
        label, date, or date range) or jurisdiction (such as a named
        administrative entity). Recommended best practice is to select a
        value from a controlled vocabulary, and that, where appropriate,
        named places or time periods be used in preference to numeric
        identifiers such as sets of co-ordinates or date ranges. Type is a
        CFString.

        @const kMDItemSubject
        Subject of the this item. Type is a CFString.

        @const kMDItemTheme
        Theme of the this item. Type is a CFString.
 
        @const kMDItemDescription
        An account of the content of the resource. Description may include
        but is not limited to: an abstract, table of contents, reference
        to a graphical representation of content or a free-text account of
        the content. Type is a CFString.

        @const kMDItemIdentifier
        Used  to reference to the resource within a given
        context. Recommended best practice is to identify the resource by
        means of a string or number conforming to a formal identification
        system. Type is a CFString.

        @const kMDItemAudiences
        A class of entity for whom the resource is intended or useful. A
        class of entity may be determined by the creator or the publisher
        or by a third party. Type is a  CFArray of CFString.
}

var kMDItemOrganizations: CFStringRef; external name '_kMDItemOrganizations'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFArray of CFStrings
var kMDItemLanguages: CFStringRef; external name '_kMDItemLanguages'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                        // CFArray of CFStrings
var kMDItemRights: CFStringRef; external name '_kMDItemRights'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                           // CFString
var kMDItemPublishers: CFStringRef; external name '_kMDItemPublishers'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                       // CFArray of CFStrings
var kMDItemContributors: CFStringRef; external name '_kMDItemContributors'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                     // CFArray of CFStrings
var kMDItemCoverage: CFStringRef; external name '_kMDItemCoverage'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                         // CFArray of CFStrings
var kMDItemSubject: CFStringRef; external name '_kMDItemSubject'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                          // CFString
var kMDItemTheme: CFStringRef; external name '_kMDItemTheme'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                            // CFString
var kMDItemDescription: CFStringRef; external name '_kMDItemDescription'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                      // CFString
var kMDItemIdentifier: CFStringRef; external name '_kMDItemIdentifier'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                       // CFString
var kMDItemAudiences: CFStringRef; external name '_kMDItemAudiences'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                        // CFArray of CFStrings


{!
        @const kMDItemNumberOfPages
        Number of pages in the item. Type is a CFNumberRef

        @const kMDItemPageWidth
        Width in points (72 points per inch) of the document page
        (first page only for PDF's - other pages within the PDF may
        not be the same width). Type is a CFNumber.

        @const kMDItemPageHeight
        Height in points (72 points per inch) of the document page
        (first page only for PDF's - other pages within the PDF may
        not be the same height). Type is a CFNumber.

        @const kMDItemSecurityMethod
        Security (encryption) method used in the file, for a PDF will be one of:
        "Password Encrypted" or "None". Type is a CFStrings.

        @const kMDItemCreator
        Application used to create the document content (e.g. "Word",
        "Framemaker", etc.). Type is a CFStrings.

        @const kMDItemEncodingApplications
        Software used to convert the original content into a PDF stream
        (e.g. "Distiller", etc.). Type is a Array of CFStrings.

        @const kMDItemDueDate
        Date this item is due. Type is a CFDate.

        @const kMDItemStarRating
        User rate of this item like iTunes. Type is a CFNumber

        @const kMDItemPhoneNumbers
        Phone numbers for this item. Type is an Array of CFStrings.

        @const kMDItemEmailAddresses
        Email addresses for this item. Type is an Array of CFStrings.

        @const  kMDItemInstantMessageAddresses
        Instant message addresses for this item. Type is an Array of CFStrings.

        @const kMDItemKind
        Kind that this item represents. Type is a CFString.

        @const kMDItemRecipients
        This attribute indicates the recipients of this item. Type is a Array of CFStrings

        @const  kMDItemFinderComment
        These are the finder comments for this item. Type is a CFString.

        @const kMDItemFonts
        Array of font names used in the item. Attribute would store the Fonts
        full name, the postscript name or the font family name based on whats available.
        Type is an Array of CFStrings.
}

var kMDItemNumberOfPages: CFStringRef; external name '_kMDItemNumberOfPages'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFNumber
var kMDItemPageWidth: CFStringRef; external name '_kMDItemPageWidth'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                        // CFNumber
var kMDItemPageHeight: CFStringRef; external name '_kMDItemPageHeight'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                       // CFNumber
var kMDItemSecurityMethod: CFStringRef; external name '_kMDItemSecurityMethod'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFNumber
var kMDItemCreator: CFStringRef; external name '_kMDItemCreator'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                          // CFString
var kMDItemEncodingApplications: CFStringRef; external name '_kMDItemEncodingApplications'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // CFArray of CFStrings
var kMDItemDueDate: CFStringRef; external name '_kMDItemDueDate'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                          // CFDate
var kMDItemStarRating: CFStringRef; external name '_kMDItemStarRating'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                       // CFNumber
var kMDItemPhoneNumbers: CFStringRef; external name '_kMDItemPhoneNumbers'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                     // CFArray of CFStrings
var kMDItemEmailAddresses: CFStringRef; external name '_kMDItemEmailAddresses'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                   // CFArray of CFStrings
var kMDItemInstantMessageAddresses: CFStringRef; external name '_kMDItemInstantMessageAddresses'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)          // CFArray of CFStrings
var kMDItemKind: CFStringRef; external name '_kMDItemKind'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                             // CFStrings
var kMDItemRecipients: CFStringRef; external name '_kMDItemRecipients'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                       // CFArray of CFStrings
var kMDItemFinderComment: CFStringRef; external name '_kMDItemFinderComment'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                    // CFString
var kMDItemFonts: CFStringRef; external name '_kMDItemFonts'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                            // CFArray of CFString


{!
        @const kMDItemAppleLoopsRootKey
        Meta data attribute that stores the root note or tonic for the
        loop, and does not include the scale type. The root key is
        represented as follows: "C" "C#/Db" "D" "D#/Eb" "E" "F"
        "F#/Gb" "G" "G#/Ab" "A" "A#/Bb" "B" "NoKey"

        @const kMDItemAppleLoopsKeyFilterType
        Meta data attribute that stores key filtering information
        about a loop. Loops are matched against projects that often in
        a major or minor key. To assist users in identifying loops
        that will "fit" with their compositions, loops can be tagged
        with one of the following key filters: "AnyKey" "Minor"
        "Major" "NeitherKey" "BothKeys". AnyKey means that it fits
        with anything (whether in a major key, minor key or
        neither). Minor fits with compositions in a minor
        key. NeitherKey doesn't work well with compositions that are
        in major or minor key. BothKeys means it fits with major or
        minor key.

        @const kMDItemAppleLoopsLoopMode
        Meta data attribute that stores how a file should be
        played. Tagged files can either be loops or non-loops (e.g., a
        cymbal crash). "Looping" indicates if the file should be
        treated as a loop. "Non-looping" indicates the file should not
        be treated as a loop.

        @const kMDItemAppleLoopDescriptors
        Meta data attribute that stores multiple pieces of descriptive
        information about a loop. Besides genre and instrument, files
        can contain descriptive information that help users in
        refining searches. A file can have multiple descriptors
        associated with them, though they come in pairs of antonyms
        (e.g., "Acoustic" and "Electric"). A file can have zero or
        more descriptors.


        @const kMDItemMusicalInstrumentCategory
        Meta data attribute that stores the category of
        instrument. Files should have an instrument associated with
        them ("Other Instrument" is provided as a catch-all). For some
        categories, like "Keyboards" there are instrument names which
        provide a more detailed instrument definition (e.g., Piano,
        Organ, etc.)

        @const kMDItemMusicalInstrumentName
        Meta data attribute that stores the name of instrument
        (relative to the instrument category) Files can have an
        instrument name associated with them if they have certain
        instrument categories (e.g., the category Percussion has
        multiple instruments, including Conga and Bongo).

        @const kMDItemCFBundleIdentifier
        If this item is a bundle, then this is the CFBundleIdentifier
}

var kMDItemAppleLoopsRootKey: CFStringRef; external name '_kMDItemAppleLoopsRootKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)                // CFString
var kMDItemAppleLoopsKeyFilterType: CFStringRef; external name '_kMDItemAppleLoopsKeyFilterType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)          // CFString
var kMDItemAppleLoopsLoopMode: CFStringRef; external name '_kMDItemAppleLoopsLoopMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)               // CFString
var kMDItemAppleLoopDescriptors: CFStringRef; external name '_kMDItemAppleLoopDescriptors'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)             // CFArray of CFStrings
var kMDItemMusicalInstrumentCategory: CFStringRef; external name '_kMDItemMusicalInstrumentCategory'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)        // CFString
var kMDItemMusicalInstrumentName: CFStringRef; external name '_kMDItemMusicalInstrumentName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)            // CFString

var kMDItemCFBundleIdentifier: CFStringRef; external name '_kMDItemCFBundleIdentifier'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)       // CFString
var kMDItemSupportFileType: CFStringRef; external name '_kMDItemSupportFileType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED *)          // CFArray of CFStrings
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFArray of CFStrings
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFArray of CFStrings
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFArray of CFStrings
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFArray of CFStrings
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)          // CFArray of CFStrings
>>>>>>> origin/fixes_2.4

{!
        @const kMDItemInformation
        Information about the item

        @const kMDItemDirector
        Director of the movie

        @const kMDItemProducer
        Producer of the content

        @const kMDItemGenre
        Genre of the movie

        @const kMDItemPerformers
        Performers in the movie

        @const kMDItemOriginalFormat
        Original format of the movie

        @const kMDItemOriginalSource
        Original source of the movie

        @const kMDItemAuthorEmailAddresses
        This attribute indicates the author of the emails message addresses. (This is always
        the email address, and not the human readable version)

        @const kMDItemRecipientEmailAddresses
        This attribute indicates the reciepients email addresses. (This is always the email
        address,  and not the human readable version).

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        @const kMDItemAuthorAddresses
        This attribute indicates the author addresses of the document.
 
        @const kMDItemRecipientAddresses
        This attribute indicates the recipient addresses of the document. 
 
        @const kMDItemURL
        Url of the item
        
        @const kMDItemIsLikelyJunk
        This attribute indicates if the document is likely to be considered junk.
        
        @const kMDItemExecutableArchitectures
        Array of executables architectures the item contains.
 
        @const kMDItemExecutablePlatform
        Indicates platform required to execute this application.

        @const kMDItemApplicationCategories
        Array of categories the item application is a member of.
=======
        @const kMDItemURL
        Url of the item
>>>>>>> graemeg/cpstrnew
=======
        @const kMDItemURL
        Url of the item
>>>>>>> graemeg/cpstrnew
=======
        @const kMDItemURL
        Url of the item
>>>>>>> graemeg/cpstrnew
=======
        @const kMDItemURL
        Url of the item
>>>>>>> origin/cpstrnew
=======
        @const kMDItemURL
        Url of the item
>>>>>>> origin/fixes_2.4

}
var kMDItemInformation: CFStringRef; external name '_kMDItemInformation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)              // CFString
var kMDItemDirector: CFStringRef; external name '_kMDItemDirector'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                 // CFString
var kMDItemProducer: CFStringRef; external name '_kMDItemProducer'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                 // CFString
var kMDItemGenre: CFStringRef; external name '_kMDItemGenre'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                    // CFString
var kMDItemPerformers: CFStringRef; external name '_kMDItemPerformers'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)               // CFArray of CFString
var kMDItemOriginalFormat: CFStringRef; external name '_kMDItemOriginalFormat'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)           // CFString
var kMDItemOriginalSource: CFStringRef; external name '_kMDItemOriginalSource'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)           // CFString
var kMDItemAuthorEmailAddresses: CFStringRef; external name '_kMDItemAuthorEmailAddresses'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)     // CFArray of CFString
var kMDItemRecipientEmailAddresses: CFStringRef; external name '_kMDItemRecipientEmailAddresses'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)  // CFArray of CFString
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kMDItemAuthorAddresses: CFStringRef; external name '_kMDItemAuthorAddresses'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)     // CFArray of CFString
var kMDItemRecipientAddresses: CFStringRef; external name '_kMDItemRecipientAddresses'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)  // CFArray of CFString
var kMDItemURL: CFStringRef; external name '_kMDItemURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                      // CFString

var kMDItemLabelIcon: CFStringRef; external name '_kMDItemLabelIcon'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER_BUT_DEPRECATED *)
var kMDItemLabelID: CFStringRef; external name '_kMDItemLabelID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER_BUT_DEPRECATED *)
var kMDItemLabelKind: CFStringRef; external name '_kMDItemLabelKind'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER_BUT_DEPRECATED *)
var kMDItemLabelUUID: CFStringRef; external name '_kMDItemLabelUUID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER_BUT_DEPRECATED *)

var kMDItemIsLikelyJunk: CFStringRef; external name '_kMDItemIsLikelyJunk'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *) // CFBoolean
var kMDItemExecutableArchitectures: CFStringRef; external name '_kMDItemExecutableArchitectures'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *) // CFArray of CFString
var kMDItemExecutablePlatform: CFStringRef; external name '_kMDItemExecutablePlatform'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *) // CFString
var kMDItemApplicationCategories: CFStringRef; external name '_kMDItemApplicationCategories'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *) // CFArray of CFString

var kMDItemIsApplicationManaged: CFStringRef; external name '_kMDItemIsApplicationManaged'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *) // CFBoolean

=======
var kMDItemURL: CFStringRef; external name '_kMDItemURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                      // CFString

>>>>>>> graemeg/cpstrnew
=======
var kMDItemURL: CFStringRef; external name '_kMDItemURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                      // CFString

>>>>>>> graemeg/cpstrnew
=======
var kMDItemURL: CFStringRef; external name '_kMDItemURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                      // CFString

>>>>>>> graemeg/cpstrnew
=======
var kMDItemURL: CFStringRef; external name '_kMDItemURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                      // CFString

>>>>>>> origin/cpstrnew
=======
var kMDItemURL: CFStringRef; external name '_kMDItemURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)                      // CFString

>>>>>>> origin/fixes_2.4
{ ================================================================ }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
