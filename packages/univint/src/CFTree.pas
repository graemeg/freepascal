{	CFTree.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1998-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
}
{	  Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
=======
>>>>>>> origin/fixes_2_2
	Copyright (c) 1998-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{	  Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit CFTree;
interface
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> origin/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
>>>>>>> origin/fixes_2.4

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
<<<<<<< HEAD
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,CFBase;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

{!
        @header CFTree
        CFTree implements a container which stores references to other CFTrees.
        Each tree may have a parent, and a variable number of children.
}


{!
        @typedef CFTreeRetainCallBack
        Type of the callback function used to add a retain to the user-specified
        info parameter.  This callback may returns the value to use whenever the
        info parameter is retained, which is usually the value parameter passed
        to this callback, but may be a different value if a different value
        should be used.
        @param info A user-supplied info parameter provided in a CFTreeContext.
        @result The retained info parameter.
}
type
	CFTreeRetainCallBack = function( info: {const} UnivPtr ): UnivPtr;

{!
        @typedef CFTreeReleaseCallBack
        Type of the callback function used to remove a retain previously
        added to the user-specified info parameter.
        @param info A user-supplied info parameter provided in a CFTreeContext.
}
type
	CFTreeReleaseCallBack = procedure( info: {const} UnivPtr );

{!
        @typedef CFTreeCopyDescriptionCallBack
        Type of the callback function used to provide a description of the
        user-specified info parameter.
        @param info A user-supplied info parameter provided in a CFTreeContext.
        @result A description of the info parameter.
}
type
	CFTreeCopyDescriptionCallBack = function( info: {const} UnivPtr ): CFStringRef;

{!
        @typedef CFTreeContext
        Structure containing user-specified data and callbacks for a CFTree.
        @field version The version number of the structure type being passed
                in as a parameter to the CFTree creation function.
                This structure is version 0.
        @field info A C pointer to a user-specified block of data.
        @field retain The callback used to add a retain for the info field.
                If this parameter is not a pointer to a function of the correct
                prototype, the behavior is undefined.  The value may be NULL.
        @field release The calllback used to remove a retain previously added
                for the info field.  If this parameter is not a pointer to a 
                function of the correct prototype, the behavior is undefined.
                The value may be NULL.
        @field copyDescription The callback used to provide a description of
                the info field.
}
type
	CFTreeContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: CFTreeRetainCallBack;
		release: CFTreeReleaseCallBack;	
		copyDescription: CFTreeCopyDescriptionCallBack;
	end;
	CFTreeContextPtr = ^CFTreeContext;

{!
        @typedef CFTreeApplierFunction
        Type of the callback function used by the apply functions of
                CFTree.
        @param value The current value from the CFTree
        @param context The user-defined context parameter give to the apply
                function.
}
type
	CFTreeApplierFunction = procedure( value: {const} UnivPtr; context: UnivPtr );

{!
        @typedef CFTreeRef
        This is the type of a reference to CFTrees.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFTreeRef = ^__CFTree; { an opaque type }
	__CFTree = record end;
=======
	CFTreeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTreeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTreeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTreeRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFTreeRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFTreeRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFTreeRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFTreeRefPtr = ^CFTreeRef;

{!
        @function CFTreeGetTypeID
        Returns the type identifier of all CFTree instances.
}
function CFTreeGetTypeID: CFTypeID; external name '_CFTreeGetTypeID';

{!
        @function CFTreeCreate
        Creates a new mutable tree.
        @param allocator The CFAllocator which should be used to allocate
                memory for the tree and storage for its children.  This
                parameter may be NULL in which case the current default
                CFAllocator is used.  If this reference is not a valid
                CFAllocator, the behavior is undefined.
        @param context A C pointer to a CFTreeContext structure to be copied 
                and used as the context of the new tree.  The info parameter
                will be retained by the tree if a retain function is provided.
                If this value is not a valid C pointer to a CFTreeContext 
                structure-sized block of storage, the result is undefined. 
                If the version number of the storage is not a valid CFTreeContext
                version number, the result is undefined.
        @result A reference to the new CFTree.
}
function CFTreeCreate( allocator: CFAllocatorRef; const (*var*) context: CFTreeContext ): CFTreeRef; external name '_CFTreeCreate';

{!
        @function CFTreeGetParent
        Returns the parent of the specified tree.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @result The parent of the tree.
}
function CFTreeGetParent( tree: CFTreeRef ): CFTreeRef; external name '_CFTreeGetParent';

{!
        @function CFTreeGetNextSibling
        Returns the sibling after the specified tree in the parent tree's list.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @result The next sibling of the tree.
}
function CFTreeGetNextSibling( tree: CFTreeRef ): CFTreeRef; external name '_CFTreeGetNextSibling';

{!
        @function CFTreeGetFirstChild
        Returns the first child of the tree.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @result The first child of the tree.
}
function CFTreeGetFirstChild( tree: CFTreeRef ): CFTreeRef; external name '_CFTreeGetFirstChild';

{!
        @function CFTreeGetContext
        Returns the context of the specified tree.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @param context A C pointer to a CFTreeContext structure to be filled in with
                the context of the specified tree.  If this value is not a valid C
                pointer to a CFTreeContext structure-sized block of storage, the
                result is undefined.  If the version number of the storage is not
                a valid CFTreeContext version number, the result is undefined.
}
procedure CFTreeGetContext( tree: CFTreeRef; var context: CFTreeContext ); external name '_CFTreeGetContext';

{!
        @function CFTreeGetChildCount
        Returns the number of children of the specified tree.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @result The number of children.
}
function CFTreeGetChildCount( tree: CFTreeRef ): CFIndex; external name '_CFTreeGetChildCount';

{!
        @function CFTreeGetChildAtIndex
        Returns the nth child of the specified tree.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @param idx The index of the child tree to be returned.  If this parameter
                is less than zero or greater than the number of children of the
                tree, the result is undefined.
        @result A reference to the specified child tree.
}
function CFTreeGetChildAtIndex( tree: CFTreeRef; idx: CFIndex ): CFTreeRef; external name '_CFTreeGetChildAtIndex';

{!
        @function CFTreeGetChildren
        Fills the buffer with children from the tree.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
	@param children A C array of pointer-sized values to be filled with
		children from the tree.  If this parameter is not a valid pointer to a 
                C array of at least CFTreeGetChildCount() pointers, the behavior is undefined.
        @result A reference to the specified child tree.
}
procedure CFTreeGetChildren( tree: CFTreeRef; var children: CFTreeRef ); external name '_CFTreeGetChildren';

{!
	@function CFTreeApplyFunctionToChildren
	Calls a function once for each child of the tree.  Note that the applier
        only operates one level deep, and does not operate on descendents further
        removed than the immediate children of the tree.
        @param heap The tree to be operated upon.  If this parameter is not a
		valid CFTree, the behavior is undefined.
	@param applier The callback function to call once for each child of
		the given tree.  If this parameter is not a pointer to a 
                function of the correct prototype, the behavior is undefined.
                If there are values in the tree which the applier function does
                not expect or cannot properly apply to, the behavior is undefined.
	@param context A pointer-sized user-defined value, which is passed
		as the second parameter to the applier function, but is
		otherwise unused by this function.  If the context is not
		what is expected by the applier function, the behavior is
		undefined.
}
procedure CFTreeApplyFunctionToChildren( tree: CFTreeRef; applier: CFTreeApplierFunction; context: UnivPtr ); external name '_CFTreeApplyFunctionToChildren';

{!
        @function CFTreeFindRoot
        Returns the root tree of which the specified tree is a descendent.
        @param tree The tree to be queried.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @result A reference to the root of the tree.
}
function CFTreeFindRoot( tree: CFTreeRef ): CFTreeRef; external name '_CFTreeFindRoot';

{!
        @function CFTreeSetContext
        Replaces the context of a tree.  The tree releases its retain on the
        info of the previous context, and retains the info of the new context.
        @param tree The tree to be operated on.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @param context A C pointer to a CFTreeContext structure to be copied 
                and used as the context of the new tree.  The info parameter
                will be retained by the tree if a retain function is provided.
                If this value is not a valid C pointer to a CFTreeContext 
                structure-sized block of storage, the result is undefined. 
                If the version number of the storage is not a valid CFTreeContext
                version number, the result is undefined.
}
procedure CFTreeSetContext( tree: CFTreeRef; const (*var*) context: CFTreeContext ); external name '_CFTreeSetContext';

{!
        @function CFTreePrependChild
        Adds the newChild to the specified tree as the first in its list of children.
        @param tree The tree to be operated on.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @param newChild The child to be added.
                If this parameter is not a valid CFTree, the behavior is undefined.
                If this parameter is a tree which is already a child of any tree,
                the behavior is undefined.
}
procedure CFTreePrependChild( tree: CFTreeRef; newChild: CFTreeRef ); external name '_CFTreePrependChild';

{!
        @function CFTreeAppendChild
        Adds the newChild to the specified tree as the last in its list of children.
        @param tree The tree to be operated on.  If this parameter is not a valid
                CFTree, the behavior is undefined.
        @param newChild The child to be added.
                If this parameter is not a valid CFTree, the behavior is undefined.
                If this parameter is a tree which is already a child of any tree,
                the behavior is undefined.
}
procedure CFTreeAppendChild( tree: CFTreeRef; newChild: CFTreeRef ); external name '_CFTreeAppendChild';

{!
        @function CFTreeInsertSibling
        Inserts newSibling into the the parent tree's linked list of children after
        tree.  The newSibling will have the same parent as tree.
        @param tree The tree to insert newSibling after.  If this parameter is not a valid
                CFTree, the behavior is undefined.  If the tree does not have a
                parent, the behavior is undefined.
        @param newSibling The sibling to be added.
                If this parameter is not a valid CFTree, the behavior is undefined.
                If this parameter is a tree which is already a child of any tree,
                the behavior is undefined.  
}
procedure CFTreeInsertSibling( tree: CFTreeRef; newSibling: CFTreeRef ); external name '_CFTreeInsertSibling';

{!
        @function CFTreeRemove
        Removes the tree from its parent.
        @param tree The tree to be removed.  If this parameter is not a valid
                CFTree, the behavior is undefined.
}
procedure CFTreeRemove( tree: CFTreeRef ); external name '_CFTreeRemove';

{!
        @function CFTreeRemoveAllChildren
        Removes all the children of the tree.
        @param tree The tree to remove all children from.  If this parameter is not a valid
                CFTree, the behavior is undefined.
}
procedure CFTreeRemoveAllChildren( tree: CFTreeRef ); external name '_CFTreeRemoveAllChildren';

{!
        @function CFTreeSortChildren
        Sorts the children of the specified tree using the specified comparator function.
        @param tree The tree to be operated on.  If this parameter is not a valid
                CFTree, the behavior is undefined.
	@param comparator The function with the comparator function type
		signature which is used in the sort operation to compare
		children of the tree with the given value. If this parameter
		is not a pointer to a function of the correct prototype, the
		the behavior is undefined. The children of the tree are sorted 
                from least to greatest according to this function.
	@param context A pointer-sized user-defined value, which is passed
		as the third parameter to the comparator function, but is
		otherwise unused by this function. If the context is not
		what is expected by the comparator function, the behavior is
		undefined.
}
procedure CFTreeSortChildren( tree: CFTreeRef; comparator: CFComparatorFunction; context: UnivPtr ); external name '_CFTreeSortChildren';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
