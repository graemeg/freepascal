{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/AVLTree.h
 
     Contains:   Interfaces for AVL balanced trees.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                 The contents of this header file are deprecated.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       AVLTree.p
=======
     File:       CarbonCore/AVLTree.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Interfaces for AVL balanced trees.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1999-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
 
     Copyright:  © 1999-2011 by Apple Inc. All rights reserved.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
 
}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit AVLTree;
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
=======
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
=======
{$elsec}
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,MixedMode;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,MixedMode;
=======
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  AVLVisitStage
 *  
 *  Discussion:
 *    The visit stage for AVLWalk() walkProcs
 }
<<<<<<< HEAD
type
	AVLVisitStage = UInt16;
const
{
   * Passed the first time AVLWalk iterates thru a given node.
   }
	kAVLPreOrder = 0;

  {
   * Passed the AVLWalk iterates thru a given node when it is 'in
   * order'.
   }
	kAVLInOrder = 1;

  {
   * Passed the last time AVLWalk iterates thru a given node.
   }
	kAVLPostOrder = 2;
<<<<<<< HEAD


{
 *  AVLOrder
 *  
 *  Discussion:
 *    The order the tree is walked or disposed of.
 }
type
	AVLOrder = UInt16;
const
{
   * Walk the tree in left-to-right order ( smaller to bigger, usually )
   }
	kLeftToRight = 0;
<<<<<<< HEAD

  {
   * Walk the tree in right-to-left order ( bigger to smaller, usually )
   }
	kRightToLeft = 1;


=======

  {
   * Walk the tree in right-to-left order ( bigger to smaller, usually )
   }
	kRightToLeft = 1;


>>>>>>> graemeg/cpstrnew
{
 *  AVLNodeType
 *  
 *  Discussion:
 *    The type of the node being passed to a callback proc.
 }
type
	AVLNodeType = UInt16;
<<<<<<< HEAD
const
	kAVLIsTree = 0;
	kAVLIsLeftBranch = 1;
	kAVLIsRightBranch = 2;
	kAVLIsLeaf = 3;
	kAVLNullNode = 4;

const
=======
const
	kAVLIsTree = 0;
	kAVLIsLeftBranch = 1;
	kAVLIsRightBranch = 2;
	kAVLIsLeaf = 3;
	kAVLNullNode = 4;

const
>>>>>>> graemeg/cpstrnew
=======


{
 *  AVLOrder
 *  
 *  Discussion:
 *    The order the tree is walked or disposed of.
 }
type
	AVLOrder = UInt16;
const
{
   * Walk the tree in left-to-right order ( smaller to bigger, usually )
   }
	kLeftToRight = 0;

  {
   * Walk the tree in right-to-left order ( bigger to smaller, usually )
   }
	kRightToLeft = 1;


{
 *  AVLNodeType
 *  
 *  Discussion:
 *    The type of the node being passed to a callback proc.
 }
type
	AVLNodeType = UInt16;
const
	kAVLIsTree = 0;
	kAVLIsLeftBranch = 1;
	kAVLIsRightBranch = 2;
	kAVLIsLeaf = 3;
	kAVLNullNode = 4;

const
>>>>>>> origin/cpstrnew
	errItemAlreadyInTree = -960;
	errNotValidTree = -961;
	errItemNotFoundInTree = -962;
	errCanNotInsertWhileWalkProcInProgress = -963;
	errTreeIsLocked = -964;


{
 *  AVLTreeStruct
 *  
 *  Summary:
 *    An opaque structure for a balanced binary tree.
 *  
 *  Discussion:
 *    The structure of a tree.  It's opaque; don't assume it's 36 bytes
 *    in size.
 }
type
	AVLTreeStructPtr = ^AVLTreeStruct;
	AVLTreeStruct = record
		signature: OSType;
		privateStuff:			array [0..7] of UNSIGNEDLONG;
	end;
type
	AVLTreePtr = AVLTreeStructPtr;

{
 *  AVLCompareItemsProcPtr
 *  
 *  Summary:
 *    A callback function which compares two data items and returns
 *    their ordering.
 *  
 *  Discussion:
 *    Every tree must have a function which compares the data for two
 *    items and returns < 0, 0, or >0 for the items - < 0 if the first
 *    item is 'before' the second item according to some criteria, == 0
 *    if the two items are identical according to the criteria, or > 0
 *    if the first item is 'after' the second item according to the
 *    criteria.  The comparison function is also passed the node type,
 *    but most of the time this can be ignored.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree which contains the items being compared
 *    
 *    i1:
 *      A pointer to the first item
 *    
 *    i2:
 *      A pointer to the second item
 *    
 *    nd_typ:
 *      The type of the nodes being compared.  This is not terribly
 *      useful most of the time.
 *  
 *  Result:
 *    A value < 0 if i1 is 'before' i2, > 0 if i1 is 'after' i2, or ==
 *    0 if i1 is equal to i2.
 }
type
	AVLCompareItemsProcPtr = function( tree: AVLTreePtr; i1: {const} UnivPtr; i2: {const} UnivPtr; nd_typ: AVLNodeType ): SInt32;

{
 *  AVLItemSizeProcPtr
 *  
 *  Summary:
 *    A callback function which returns the size of an item.
 *  
 *  Discussion:
 *    Every tree must have a itemSizeProc; this routine gets passed a
 *    pointer to the item's data and returns the size of the data.  If
 *    a tree contains records of a fixed size, this function can just
 *    return sizeof( that-struct ); otherwise it should calculate the
 *    size of the item based on the data for the item.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree which contains the item whose size is being requested.
 *    
 *    itemPtr:
 *      A pointer to the item whose size is being returned.
 *  
 *  Result:
 *    The size of the item.
 }
type
	AVLItemSizeProcPtr = function( tree: AVLTreePtr; itemPtr: {const} UnivPtr ): ByteCount;

{
 *  AVLDisposeItemProcPtr
 *  
 *  Summary:
 *    Dispose of any additional memory associated with an item in the
 *    tree.
 *  
 *  Discussion:
 *    A tree may have an optional disposeItemProc, which gets called
 *    whenever an item is removed from the tree ( via AVLRemove() or
 *    when AVLDispose() deletes all of the items in the tree ). This
 *    might be useful if the nodes in the tree own 'resources'  ( like,
 *    open files ) which should be released before the item is removed.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree containing the item being disposed.
 *    
 *    dataP:
 *      A pointer to the data for the item being disposed.
 }
type
	AVLDisposeItemProcPtr = procedure( tree: AVLTreePtr; dataP: {const} UnivPtr );

{
 *  AVLWalkProcPtr
 *  
 *  Summary:
 *    A callback function which gets passed each item in the tree, in a
 *    specified order.
 *  
 *  Discussion:
 *    The common way to iterate across all of the items in a tree is
 *    via AVLWalk(), which takes a walkProcPtr.  This function will get
 *    called for every item in the tree three times, as the tree is
 *    being walked across.  First, the walkProc will get called with
 *    visitStage == kAVLPreOrder, at which point internally the node of
 *    the tree for the given data has just been reached.  Later, this
 *    function will get called with visitStage == kAVLInOrder, and
 *    lastly this function will get called with visitStage ==
 *    kAVLPostOrder. The 'minimum' item in the tree will get called
 *    with visitStage == kInOrder first, followed by the 'next' item in
 *    the tree, up until the last item in the tree structure is called.
 *    In general, you'll only care about calls to this function when
 *    visitStage == kAVLInOrder.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree being walked.
 *    
 *    dataPtr:
 *      A pointer to the data for an item in the tree.
 *    
 *    visitStage:
 *      The stage of the walk for the given node.
 *    
 *    node:
 *      The type of the given node. This is not terribly useful most of
 *      the time.
 *    
 *    level:
 *      How 'deep' in the tree the given node is.  This is not terribly
 *      useful most of the time.
 *    
 *    balance:
 *      How balanced the given node in the tree is.  This is not
 *      terribly useful most of the time.
 *    
 *    refCon:
 *      The refCon passed into AVLWalk() for this call.
 *  
 *  Result:
 *    Return 0 to continue walking the tree, or 1 to terminate.
 }
type
	AVLWalkProcPtr = function( tree: AVLTreePtr; dataPtr: {const} UnivPtr; visitStage: AVLVisitStage; node: AVLNodeType; level: UInt32; balance: SInt32; refCon: UnivPtr ): OSErr;
	AVLCompareItemsUPP = AVLCompareItemsProcPtr;
	AVLItemSizeUPP = AVLItemSizeProcPtr;
	AVLDisposeItemUPP = AVLDisposeItemProcPtr;
	AVLWalkUPP = AVLWalkProcPtr;
{
 *  NewAVLCompareItemsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAVLCompareItemsUPP( userRoutine: AVLCompareItemsProcPtr ): AVLCompareItemsUPP; external name '_NewAVLCompareItemsUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
type
	AVLVisitStage = UInt16;
const
{
   * Passed the first time AVLWalk iterates thru a given node.
   }
	kAVLPreOrder = 0;

  {
   * Passed the AVLWalk iterates thru a given node when it is 'in
   * order'.
   }
	kAVLInOrder = 1;

  {
   * Passed the last time AVLWalk iterates thru a given node.
   }
	kAVLPostOrder = 2;


{
 *  AVLOrder
 *  
 *  Discussion:
 *    The order the tree is walked or disposed of.
 }
type
	AVLOrder = UInt16;
const
{
   * Walk the tree in left-to-right order ( smaller to bigger, usually )
   }
	kLeftToRight = 0;

  {
   * Walk the tree in right-to-left order ( bigger to smaller, usually )
   }
	kRightToLeft = 1;


{
 *  AVLNodeType
 *  
 *  Discussion:
 *    The type of the node being passed to a callback proc.
 }
type
	AVLNodeType = UInt16;
const
	kAVLIsTree = 0;
	kAVLIsLeftBranch = 1;
	kAVLIsRightBranch = 2;
	kAVLIsLeaf = 3;
	kAVLNullNode = 4;

const
	errItemAlreadyInTree = -960;
	errNotValidTree = -961;
	errItemNotFoundInTree = -962;
	errCanNotInsertWhileWalkProcInProgress = -963;
	errTreeIsLocked = -964;


{
 *  AVLTreeStruct
 *  
 *  Summary:
 *    An opaque structure for a balanced binary tree.
 *  
 *  Discussion:
 *    The structure of a tree.  It's opaque; don't assume it's 36 bytes
 *    in size.
 }
type
	AVLTreeStructPtr = ^AVLTreeStruct;
	AVLTreeStruct = record
		signature: OSType;
		privateStuff:			array [0..7] of UNSIGNEDLONG;
	end;
type
	AVLTreePtr = AVLTreeStructPtr;

{
 *  AVLCompareItemsProcPtr
 *  
 *  Summary:
 *    A callback function which compares two data items and returns
 *    their ordering.
 *  
 *  Discussion:
 *    Every tree must have a function which compares the data for two
 *    items and returns < 0, 0, or >0 for the items - < 0 if the first
 *    item is 'before' the second item according to some criteria, == 0
 *    if the two items are identical according to the criteria, or > 0
 *    if the first item is 'after' the second item according to the
 *    criteria.  The comparison function is also passed the node type,
 *    but most of the time this can be ignored.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree which contains the items being compared
 *    
 *    i1:
 *      A pointer to the first item
 *    
 *    i2:
 *      A pointer to the second item
 *    
 *    nd_typ:
 *      The type of the nodes being compared.  This is not terribly
 *      useful most of the time.
 *  
 *  Result:
 *    A value < 0 if i1 is 'before' i2, > 0 if i1 is 'after' i2, or ==
 *    0 if i1 is equal to i2.
 }
type
	AVLCompareItemsProcPtr = function( tree: AVLTreePtr; i1: {const} UnivPtr; i2: {const} UnivPtr; nd_typ: AVLNodeType ): SInt32;

{
 *  AVLItemSizeProcPtr
 *  
 *  Summary:
 *    A callback function which returns the size of an item.
 *  
 *  Discussion:
 *    Every tree must have a itemSizeProc; this routine gets passed a
 *    pointer to the item's data and returns the size of the data.  If
 *    a tree contains records of a fixed size, this function can just
 *    return sizeof( that-struct ); otherwise it should calculate the
 *    size of the item based on the data for the item.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree which contains the item whose size is being requested.
 *    
 *    itemPtr:
 *      A pointer to the item whose size is being returned.
 *  
 *  Result:
 *    The size of the item.
 }
type
	AVLItemSizeProcPtr = function( tree: AVLTreePtr; itemPtr: {const} UnivPtr ): ByteCount;

{
 *  AVLDisposeItemProcPtr
 *  
 *  Summary:
 *    Dispose of any additional memory associated with an item in the
 *    tree.
 *  
 *  Discussion:
 *    A tree may have an optional disposeItemProc, which gets called
 *    whenever an item is removed from the tree ( via AVLRemove() or
 *    when AVLDispose() deletes all of the items in the tree ). This
 *    might be useful if the nodes in the tree own 'resources'  ( like,
 *    open files ) which should be released before the item is removed.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree containing the item being disposed.
 *    
 *    dataP:
 *      A pointer to the data for the item being disposed.
 }
type
	AVLDisposeItemProcPtr = procedure( tree: AVLTreePtr; dataP: {const} UnivPtr );

{
 *  AVLWalkProcPtr
 *  
 *  Summary:
 *    A callback function which gets passed each item in the tree, in a
 *    specified order.
 *  
 *  Discussion:
 *    The common way to iterate across all of the items in a tree is
 *    via AVLWalk(), which takes a walkProcPtr.  This function will get
 *    called for every item in the tree three times, as the tree is
 *    being walked across.  First, the walkProc will get called with
 *    visitStage == kAVLPreOrder, at which point internally the node of
 *    the tree for the given data has just been reached.  Later, this
 *    function will get called with visitStage == kAVLInOrder, and
 *    lastly this function will get called with visitStage ==
 *    kAVLPostOrder. The 'minimum' item in the tree will get called
 *    with visitStage == kInOrder first, followed by the 'next' item in
 *    the tree, up until the last item in the tree structure is called.
 *    In general, you'll only care about calls to this function when
 *    visitStage == kAVLInOrder.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree being walked.
 *    
 *    dataPtr:
 *      A pointer to the data for an item in the tree.
 *    
 *    visitStage:
 *      The stage of the walk for the given node.
 *    
 *    node:
 *      The type of the given node. This is not terribly useful most of
 *      the time.
 *    
 *    level:
 *      How 'deep' in the tree the given node is.  This is not terribly
 *      useful most of the time.
 *    
 *    balance:
 *      How balanced the given node in the tree is.  This is not
 *      terribly useful most of the time.
 *    
 *    refCon:
 *      The refCon passed into AVLWalk() for this call.
 *  
 *  Result:
 *    Return 0 to continue walking the tree, or 1 to terminate.
 }
type
	AVLWalkProcPtr = function( tree: AVLTreePtr; dataPtr: {const} UnivPtr; visitStage: AVLVisitStage; node: AVLNodeType; level: UInt32; balance: SInt32; refCon: UnivPtr ): OSErr;
	AVLCompareItemsUPP = AVLCompareItemsProcPtr;
	AVLItemSizeUPP = AVLItemSizeProcPtr;
	AVLDisposeItemUPP = AVLDisposeItemProcPtr;
	AVLWalkUPP = AVLWalkProcPtr;
{
 *  NewAVLCompareItemsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAVLCompareItemsUPP( userRoutine: AVLCompareItemsProcPtr ): AVLCompareItemsUPP; external name '_NewAVLCompareItemsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
const
	uppAVLCompareItemsProcInfo = $00002FF0;
	uppAVLItemSizeProcInfo = $000003F0;
	uppAVLDisposeItemProcInfo = $000003C0;
	uppAVLWalkProcInfo = $000FEBE0;
	{
	 *  NewAVLCompareItemsUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewAVLCompareItemsUPP(userRoutine: AVLCompareItemsProcPtr): AVLCompareItemsUPP; external name '_NewAVLCompareItemsUPP'; { old name was NewAVLCompareItemsProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewAVLItemSizeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAVLItemSizeUPP( userRoutine: AVLItemSizeProcPtr ): AVLItemSizeUPP; external name '_NewAVLItemSizeUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewAVLItemSizeUPP(userRoutine: AVLItemSizeProcPtr): AVLItemSizeUPP; external name '_NewAVLItemSizeUPP'; { old name was NewAVLItemSizeProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewAVLItemSizeUPP( userRoutine: AVLItemSizeProcPtr ): AVLItemSizeUPP; external name '_NewAVLItemSizeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  NewAVLDisposeItemUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAVLDisposeItemUPP( userRoutine: AVLDisposeItemProcPtr ): AVLDisposeItemUPP; external name '_NewAVLDisposeItemUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewAVLDisposeItemUPP(userRoutine: AVLDisposeItemProcPtr): AVLDisposeItemUPP; external name '_NewAVLDisposeItemUPP'; { old name was NewAVLDisposeItemProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewAVLDisposeItemUPP( userRoutine: AVLDisposeItemProcPtr ): AVLDisposeItemUPP; external name '_NewAVLDisposeItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  NewAVLWalkUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAVLWalkUPP( userRoutine: AVLWalkProcPtr ): AVLWalkUPP; external name '_NewAVLWalkUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewAVLWalkUPP(userRoutine: AVLWalkProcPtr): AVLWalkUPP; external name '_NewAVLWalkUPP'; { old name was NewAVLWalkProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewAVLWalkUPP( userRoutine: AVLWalkProcPtr ): AVLWalkUPP; external name '_NewAVLWalkUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeAVLCompareItemsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAVLCompareItemsUPP( userUPP: AVLCompareItemsUPP ); external name '_DisposeAVLCompareItemsUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAVLCompareItemsUPP(userUPP: AVLCompareItemsUPP); external name '_DisposeAVLCompareItemsUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAVLCompareItemsUPP( userUPP: AVLCompareItemsUPP ); external name '_DisposeAVLCompareItemsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeAVLItemSizeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAVLItemSizeUPP( userUPP: AVLItemSizeUPP ); external name '_DisposeAVLItemSizeUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAVLItemSizeUPP(userUPP: AVLItemSizeUPP); external name '_DisposeAVLItemSizeUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAVLItemSizeUPP( userUPP: AVLItemSizeUPP ); external name '_DisposeAVLItemSizeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeAVLDisposeItemUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAVLDisposeItemUPP( userUPP: AVLDisposeItemUPP ); external name '_DisposeAVLDisposeItemUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAVLDisposeItemUPP(userUPP: AVLDisposeItemUPP); external name '_DisposeAVLDisposeItemUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAVLDisposeItemUPP( userUPP: AVLDisposeItemUPP ); external name '_DisposeAVLDisposeItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeAVLWalkUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAVLWalkUPP( userUPP: AVLWalkUPP ); external name '_DisposeAVLWalkUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAVLWalkUPP(userUPP: AVLWalkUPP); external name '_DisposeAVLWalkUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAVLWalkUPP( userUPP: AVLWalkUPP ); external name '_DisposeAVLWalkUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeAVLCompareItemsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAVLCompareItemsUPP( tree: AVLTreePtr; i1: {const} UnivPtr; i2: {const} UnivPtr; nd_typ: AVLNodeType; userUPP: AVLCompareItemsUPP ): SInt32; external name '_InvokeAVLCompareItemsUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeAVLCompareItemsUPP(tree: AVLTreePtr; i1: UnivPtr; i2: UnivPtr; nd_typ: AVLNodeType; userRoutine: AVLCompareItemsUPP): SInt32; external name '_InvokeAVLCompareItemsUPP'; { old name was CallAVLCompareItemsProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeAVLCompareItemsUPP( tree: AVLTreePtr; i1: {const} UnivPtr; i2: {const} UnivPtr; nd_typ: AVLNodeType; userUPP: AVLCompareItemsUPP ): SInt32; external name '_InvokeAVLCompareItemsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeAVLItemSizeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAVLItemSizeUPP( tree: AVLTreePtr; itemPtr: {const} UnivPtr; userUPP: AVLItemSizeUPP ): ByteCount; external name '_InvokeAVLItemSizeUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeAVLItemSizeUPP(tree: AVLTreePtr; itemPtr: UnivPtr; userRoutine: AVLItemSizeUPP): UInt32; external name '_InvokeAVLItemSizeUPP'; { old name was CallAVLItemSizeProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeAVLItemSizeUPP( tree: AVLTreePtr; itemPtr: {const} UnivPtr; userUPP: AVLItemSizeUPP ): ByteCount; external name '_InvokeAVLItemSizeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeAVLDisposeItemUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeAVLDisposeItemUPP( tree: AVLTreePtr; dataP: {const} UnivPtr; userUPP: AVLDisposeItemUPP ); external name '_InvokeAVLDisposeItemUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure InvokeAVLDisposeItemUPP(tree: AVLTreePtr; dataP: UnivPtr; userRoutine: AVLDisposeItemUPP); external name '_InvokeAVLDisposeItemUPP'; { old name was CallAVLDisposeItemProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvokeAVLDisposeItemUPP( tree: AVLTreePtr; dataP: {const} UnivPtr; userUPP: AVLDisposeItemUPP ); external name '_InvokeAVLDisposeItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeAVLWalkUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAVLWalkUPP( tree: AVLTreePtr; dataPtr: {const} UnivPtr; visitStage: AVLVisitStage; node: AVLNodeType; level: UInt32; balance: SInt32; refCon: UnivPtr; userUPP: AVLWalkUPP ): OSErr; external name '_InvokeAVLWalkUPP';
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
{
 *  AVLInit()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Create an AVL ( balanced binary ) tree
 *  
 *  Discussion:
 *    Create an AVL tree.  The compareItemsProc and the sizeItemProc
 *    are required; disposeItemProc is optional and can be nil.  The
 *    refCon is stored with the list, and is passed back to the
 *    compareItemsProc, sizeItemProc, and disposeItemsProc calls.  The
 *    allocation of the tree ( and all nodes later added to the list
 *    with AVLInsert ) will be created in what is the current zone at
 *    the time AVLInit() is called.  Always call AVLDispose() to
 *    dispose of a list created with AVLInit().
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    flags:
 *      Creation flags.  Currently, no flags are defined, so pass 0.
 *    
 *    compareItemsProc:
 *      A UPP for a function which compares two data items, and returns
 *      a value which is < 0, == 0, or > 0 depending on whether the
 *      first item is 'before', 'equal', or 'after' the first item.
 *    
 *    sizeItemProc:
 *      A UPP for a function which takes a pointer to a data item, and
 *      returns the size in bytes which it requires to store.
 *    
 *    disposeItemProc:
 *      A UPP for a function which takes a pointer to a data item, and
 *      disposes of any memory which may have been allocated for the
 *      item.  This does not need to dispose of the item itself ( the
 *      AVLTree Manager will do that ), only any memory which the item
 *      passed in may be pointing to.  This function may be NULL if
 *      data items do not use any memory besides that taken up by the
 *      item itself.
 *    
 *    refCon:
 *      A 32 bit quantity which is stored with this tree, and can be
 *      retrieved at an time ( and in a callback, if desired ) with
 *      AVLGetRefcon().
 *    
 *    tree:
 *      The created AVLTree, or NULL if there is an error.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLInit( flags: UInt32; compareItemsProc: AVLCompareItemsUPP; sizeItemProc: AVLItemSizeUPP; disposeItemProc: AVLDisposeItemUPP; refCon: UnivPtr; var tree: AVLTreePtr ): OSErr; external name '_AVLInit';
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
 *  AVLDispose()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Dispose of an AVL tree created with AVLInit()
 *  
 *  Discussion:
 *    Dispose of an AVL tree.  This will dispose of each item in the
 *    tree in the order specified, call the tree's disposeProc proc for
 *    each item, and then dispose of the space allocated for the tree
 *    itself.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLDispose is thread safe provided no other thread is still using
 *    the tree being dispose.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to dispose, which was created with AVLInit().
 *    
 *    order:
 *      The order to dispose of the items in the tree.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLDispose( var tree: AVLTreePtr; order: AVLOrder ): OSErr; external name '_AVLDispose';
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
 *  AVLWalk()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Iterate across all of the items in an AVL tree, in a specified
 *    order.
 *  
 *  Discussion:
 *    Iterate across all of the items in the tree, in the order
 *    specified.  kLeftToRight is basically lowest-to-highest order,
 *    kRightToLeft is highest-to-lowest order.  For each node in the
 *    tree, it will call the walkProc with three messages ( at the
 *    appropriate time ).  First, with kAVLPreOrder when the walking
 *    gets to this node in the tree, before handling either the left or
 *    right subtree, secondly, with kAVLInOrder after handling one
 *    subtree but before handling the other, and lastly with
 *    kAVLPostOrder after handling both subtrees.  If you want to
 *    handle items in order, then only do something if the visit stage
 *    is kAVLInOrder.  You can only call AVLRemove() from inside a
 *    walkProc if visit stage is kAVLPostOrder ( because if you remove
 *    a node during the pre or in order stages you will corrupt the
 *    list ) OR if you return a non-zero result from the walkProc call
 *    which called AVLRemove() to immediately terminate the walkProc. 
 *    Do not call AVLInsert() to insert a node into the tree from
 *    inside a walkProc. The walkProc function gets called with the
 *    AVLTreePtr, a pointer to the data for the current node ( which
 *    you can change in place as long as you do not affect the order
 *    within the tree ), the visit stage, the type of the current node
 *    ( leaf node, right or left branch, or full tree ), the level
 *    within the tree ( the root is level 1 ), the balance for the
 *    current node, and the refCon passed to AVLWalk().  This refCon is
 *    different from the one passed into AVLInit(); use AVLGetRefCon()
 *    to get that refCon if you want it inside a walkProc. ( Most
 *    walkProcs will not care about the values for node type, level, or
 *    balance. )
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLWalk is thread safe as long as no other thread tries to modify
 *    the tree by inserting or removing an item, or disposing of the
 *    tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to dispose, which was created with AVLInit().
 *    
 *    walkProc:
 *      A UPP for a function which is called during the walk thru the
 *      tree for each item.
 *    
 *    order:
 *      The order to iterate thru the tree.
 *    
 *    walkRefCon:
 *      A 32 bit value passed to the walkProc each time it is called.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLWalk( tree: AVLTreePtr; walkProc: AVLWalkUPP; order: AVLOrder; walkRefCon: UnivPtr ): OSErr; external name '_AVLWalk';
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
 *  AVLCount()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Return the number of items in the given tree.
 *  
 *  Discussion:
 *    Return the number of items in the given tree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLCount is thread safe as long as no other thread tries to
 *    modify the tree by inserting or removing an item, or disposing of
 *    the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the count of items for.
 *    
 *    count:
 *      On return, the count of items ( 1-based ) in the tree.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLCount( tree: AVLTreePtr; var count: UInt32 ): OSErr; external name '_AVLCount';
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
 *  AVLGetIndItem()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Return the data of the index-th item from the tree.
 *  
 *  Discussion:
 *    Return the one-based index-th item from the tree by putting it's
 *    data at dataPtr if dataPtr is non-nil, and it's size into
 *    *itemSize if itemSize is non-nil. If index is out of range,
 *    return errItemNotFoundInTree.  ( Internally, this does an
 *    AVLWalk(), so the tree can not be modified while this call is in
 *    progress ).
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLGetIndItem is thread safe as long as no other thread tries to
 *    modify the tree by inserting or removing an item, or disposing of
 *    the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    index:
 *      A index of the item to return.  The 'first' item in the tree is
 *      index = 1, the last item is index = the count of items in the
 *      tree.
 *    
 *    dataPtr:
 *      An address in memory to return the data for the item, or NULL
 *      if you don not want data returned.  The memory point to must be
 *      large enough to hold all of the data for the item.
 *    
 *    itemSize:
 *      On return, the size of the data for the index-th item.  If you
 *      do not care about the size of the data, pass NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLGetIndItem( tree: AVLTreePtr; index: UInt32; dataPtr: UnivPtr; var itemSize: ByteCount ): OSErr; external name '_AVLGetIndItem';
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
 *  AVLInsert()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Insert an item into the tree.
 *  
 *  Discussion:
 *    Insert the given item into the tree.  This will call the tree's
 *    sizeItemProc to determine how big the item at data is, and then
 *    will make a copy of the item and insert it into the tree in the
 *    appropriate place.  If an item already exists in the tree with
 *    the same key ( so that the compareItemsUPP returns 0 when asked
 *    to compare this item to an existing one ), then it will return
 *    errItemNotFoundInTree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLInsert is thread safe as long as no other thread tries to walk
 *    or modify the tree by inserting or removing an item, or disposing
 *    of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    data:
 *      A pointer to the item to be inserted.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLInsert( tree: AVLTreePtr; data: {const} UnivPtr ): OSErr; external name '_AVLInsert';
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
 *  AVLRemove()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Remove an item from the tree.
 *  
 *  Discussion:
 *    Remove any item from the tree with the given key.  If dataPtr !=
 *    nil, then copy the item's data to dataPtr before removing it from
 *    the tree.  Before removing the item, call the tree's
 *    disposeItemProc to let it release anything used by the data in
 *    the tree.  It is not necessary to fill in a complete record for
 *    key, only that the compareItemsProc return 0 when asked to
 *    compare the data at key with the node in the tree to be deleted. 
 *    If the item cannot be found in the tree, this will return
 *    errItemNotFoundInTree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLRemove is thread safe as long as no other thread tries to walk
 *    or modify the tree by inserting or removing an item, or disposing
 *    of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    key:
 *      A pointer to the item to be removed ( or, enough of the item
 *      that it can be found in the tree ).
 *    
 *    dataPtr:
 *      An address in memory to return the data for the item, or NULL
 *      if you don not want data returned.  The memory point to must be
 *      large enough to hold all of the data for the item.
 *    
 *    itemSize:
 *      On return, the size of the data for the index-th item.  If you
 *      do not care about the size of the data, pass NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLRemove( tree: AVLTreePtr; key: {const} UnivPtr; dataPtr: UnivPtr; var itemSize: ByteCount ): OSErr; external name '_AVLRemove';
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
 *  AVLFind()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Remove an item from the tree.
 *  
 *  Discussion:
 *    Find the item in the tree with the given key, and return it's
 *    data in dataPtr ( if dataPtr != nil ), and it's size in *itemSize
 *    ( if itemSize != nil ).  It is not necessary to fill in a
 *    complete record for key, only that the compareItemsProc return 0
 *    when asked to compare the data at key with the node in the tree
 *    to be deleted.  If the item cannot be found in the tree, this
 *    will return errItemNotFoundInTree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLRemove is thread safe as long as no other thread tries to walk
 *    or modify the tree by inserting or removing an item, or disposing
 *    of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    key:
 *      A pointer to the item to be found ( or, enough of the item that
 *      it can be found in the tree ).
 *    
 *    dataPtr:
 *      An address in memory to return the data for the item, or NULL
 *      if you don not want data returned.  The memory point to must be
 *      large enough to hold all of the data for the item.
 *    
 *    itemSize:
 *      On return, the size of the data for the index-th item.  If you
 *      do not care about the size of the data, pass NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLFind( tree: AVLTreePtr; key: {const} UnivPtr; dataPtr: UnivPtr; var itemSize: ByteCount ): OSErr; external name '_AVLFind';
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
 *  AVLGetRefcon()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Return the refCon set when the tree was created.
 *  
 *  Discussion:
 *    Get the refCon for the given tree ( set in AVLInit ) and return
 *    it.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLGetRefcon is thread safe as long as no other thread tries to
 *    dispose of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the refcon for.
 *    
 *    refCon:
 *      On return, the refCon for the tree, or NULL if tree is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLGetRefcon( tree: AVLTreePtr; var refCon: UnivPtr ): OSErr; external name '_AVLGetRefcon';
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

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAVLWalkUPP( tree: AVLTreePtr; dataPtr: {const} UnivPtr; visitStage: AVLVisitStage; node: AVLNodeType; level: UInt32; balance: SInt32; refCon: UnivPtr; userUPP: AVLWalkUPP ): OSErr; external name '_InvokeAVLWalkUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$ifc not TARGET_CPU_64}
{
 *  AVLInit()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Create an AVL ( balanced binary ) tree
 *  
 *  Discussion:
 *    Create an AVL tree.  The compareItemsProc and the sizeItemProc
 *    are required; disposeItemProc is optional and can be nil.  The
 *    refCon is stored with the list, and is passed back to the
 *    compareItemsProc, sizeItemProc, and disposeItemsProc calls.  The
 *    allocation of the tree ( and all nodes later added to the list
 *    with AVLInsert ) will be created in what is the current zone at
 *    the time AVLInit() is called.  Always call AVLDispose() to
 *    dispose of a list created with AVLInit().
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    flags:
 *      Creation flags.  Currently, no flags are defined, so pass 0.
 *    
 *    compareItemsProc:
 *      A UPP for a function which compares two data items, and returns
 *      a value which is < 0, == 0, or > 0 depending on whether the
 *      first item is 'before', 'equal', or 'after' the first item.
 *    
 *    sizeItemProc:
 *      A UPP for a function which takes a pointer to a data item, and
 *      returns the size in bytes which it requires to store.
 *    
 *    disposeItemProc:
 *      A UPP for a function which takes a pointer to a data item, and
 *      disposes of any memory which may have been allocated for the
 *      item.  This does not need to dispose of the item itself ( the
 *      AVLTree Manager will do that ), only any memory which the item
 *      passed in may be pointing to.  This function may be NULL if
 *      data items do not use any memory besides that taken up by the
 *      item itself.
 *    
 *    refCon:
 *      A 32 bit quantity which is stored with this tree, and can be
 *      retrieved at an time ( and in a callback, if desired ) with
 *      AVLGetRefcon().
 *    
 *    tree:
 *      The created AVLTree, or NULL if there is an error.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLInit( flags: UInt32; compareItemsProc: AVLCompareItemsUPP; sizeItemProc: AVLItemSizeUPP; disposeItemProc: AVLDisposeItemUPP; refCon: UnivPtr; var tree: AVLTreePtr ): OSErr; external name '_AVLInit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLDispose()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Dispose of an AVL tree created with AVLInit()
 *  
 *  Discussion:
 *    Dispose of an AVL tree.  This will dispose of each item in the
 *    tree in the order specified, call the tree's disposeProc proc for
 *    each item, and then dispose of the space allocated for the tree
 *    itself.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLDispose is thread safe provided no other thread is still using
 *    the tree being dispose.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to dispose, which was created with AVLInit().
 *    
 *    order:
 *      The order to dispose of the items in the tree.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLDispose( var tree: AVLTreePtr; order: AVLOrder ): OSErr; external name '_AVLDispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLWalk()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Iterate across all of the items in an AVL tree, in a specified
 *    order.
 *  
 *  Discussion:
 *    Iterate across all of the items in the tree, in the order
 *    specified.  kLeftToRight is basically lowest-to-highest order,
 *    kRightToLeft is highest-to-lowest order.  For each node in the
 *    tree, it will call the walkProc with three messages ( at the
 *    appropriate time ).  First, with kAVLPreOrder when the walking
 *    gets to this node in the tree, before handling either the left or
 *    right subtree, secondly, with kAVLInOrder after handling one
 *    subtree but before handling the other, and lastly with
 *    kAVLPostOrder after handling both subtrees.  If you want to
 *    handle items in order, then only do something if the visit stage
 *    is kAVLInOrder.  You can only call AVLRemove() from inside a
 *    walkProc if visit stage is kAVLPostOrder ( because if you remove
 *    a node during the pre or in order stages you will corrupt the
 *    list ) OR if you return a non-zero result from the walkProc call
 *    which called AVLRemove() to immediately terminate the walkProc. 
 *    Do not call AVLInsert() to insert a node into the tree from
 *    inside a walkProc. The walkProc function gets called with the
 *    AVLTreePtr, a pointer to the data for the current node ( which
 *    you can change in place as long as you do not affect the order
 *    within the tree ), the visit stage, the type of the current node
 *    ( leaf node, right or left branch, or full tree ), the level
 *    within the tree ( the root is level 1 ), the balance for the
 *    current node, and the refCon passed to AVLWalk().  This refCon is
 *    different from the one passed into AVLInit(); use AVLGetRefCon()
 *    to get that refCon if you want it inside a walkProc. ( Most
 *    walkProcs will not care about the values for node type, level, or
 *    balance. )
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLWalk is thread safe as long as no other thread tries to modify
 *    the tree by inserting or removing an item, or disposing of the
 *    tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to dispose, which was created with AVLInit().
 *    
 *    walkProc:
 *      A UPP for a function which is called during the walk thru the
 *      tree for each item.
 *    
 *    order:
 *      The order to iterate thru the tree.
 *    
 *    walkRefCon:
 *      A 32 bit value passed to the walkProc each time it is called.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLWalk( tree: AVLTreePtr; walkProc: AVLWalkUPP; order: AVLOrder; walkRefCon: UnivPtr ): OSErr; external name '_AVLWalk';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLCount()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Return the number of items in the given tree.
 *  
 *  Discussion:
 *    Return the number of items in the given tree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLCount is thread safe as long as no other thread tries to
 *    modify the tree by inserting or removing an item, or disposing of
 *    the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the count of items for.
 *    
 *    count:
 *      On return, the count of items ( 1-based ) in the tree.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLCount( tree: AVLTreePtr; var count: UInt32 ): OSErr; external name '_AVLCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLGetIndItem()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Return the data of the index-th item from the tree.
 *  
 *  Discussion:
 *    Return the one-based index-th item from the tree by putting it's
 *    data at dataPtr if dataPtr is non-nil, and it's size into
 *    *itemSize if itemSize is non-nil. If index is out of range,
 *    return errItemNotFoundInTree.  ( Internally, this does an
 *    AVLWalk(), so the tree can not be modified while this call is in
 *    progress ).
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLGetIndItem is thread safe as long as no other thread tries to
 *    modify the tree by inserting or removing an item, or disposing of
 *    the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    index:
 *      A index of the item to return.  The 'first' item in the tree is
 *      index = 1, the last item is index = the count of items in the
 *      tree.
 *    
 *    dataPtr:
 *      An address in memory to return the data for the item, or NULL
 *      if you don not want data returned.  The memory point to must be
 *      large enough to hold all of the data for the item.
 *    
 *    itemSize:
 *      On return, the size of the data for the index-th item.  If you
 *      do not care about the size of the data, pass NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLGetIndItem( tree: AVLTreePtr; index: UInt32; dataPtr: UnivPtr; var itemSize: ByteCount ): OSErr; external name '_AVLGetIndItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLInsert()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Insert an item into the tree.
 *  
 *  Discussion:
 *    Insert the given item into the tree.  This will call the tree's
 *    sizeItemProc to determine how big the item at data is, and then
 *    will make a copy of the item and insert it into the tree in the
 *    appropriate place.  If an item already exists in the tree with
 *    the same key ( so that the compareItemsUPP returns 0 when asked
 *    to compare this item to an existing one ), then it will return
 *    errItemNotFoundInTree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLInsert is thread safe as long as no other thread tries to walk
 *    or modify the tree by inserting or removing an item, or disposing
 *    of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    data:
 *      A pointer to the item to be inserted.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLInsert( tree: AVLTreePtr; data: {const} UnivPtr ): OSErr; external name '_AVLInsert';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLRemove()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Remove an item from the tree.
 *  
 *  Discussion:
 *    Remove any item from the tree with the given key.  If dataPtr !=
 *    nil, then copy the item's data to dataPtr before removing it from
 *    the tree.  Before removing the item, call the tree's
 *    disposeItemProc to let it release anything used by the data in
 *    the tree.  It is not necessary to fill in a complete record for
 *    key, only that the compareItemsProc return 0 when asked to
 *    compare the data at key with the node in the tree to be deleted. 
 *    If the item cannot be found in the tree, this will return
 *    errItemNotFoundInTree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLRemove is thread safe as long as no other thread tries to walk
 *    or modify the tree by inserting or removing an item, or disposing
 *    of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    key:
 *      A pointer to the item to be removed ( or, enough of the item
 *      that it can be found in the tree ).
 *    
 *    dataPtr:
 *      An address in memory to return the data for the item, or NULL
 *      if you don not want data returned.  The memory point to must be
 *      large enough to hold all of the data for the item.
 *    
 *    itemSize:
 *      On return, the size of the data for the index-th item.  If you
 *      do not care about the size of the data, pass NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLRemove( tree: AVLTreePtr; key: {const} UnivPtr; dataPtr: UnivPtr; var itemSize: ByteCount ): OSErr; external name '_AVLRemove';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLFind()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Remove an item from the tree.
 *  
 *  Discussion:
 *    Find the item in the tree with the given key, and return it's
 *    data in dataPtr ( if dataPtr != nil ), and it's size in *itemSize
 *    ( if itemSize != nil ).  It is not necessary to fill in a
 *    complete record for key, only that the compareItemsProc return 0
 *    when asked to compare the data at key with the node in the tree
 *    to be deleted.  If the item cannot be found in the tree, this
 *    will return errItemNotFoundInTree.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLRemove is thread safe as long as no other thread tries to walk
 *    or modify the tree by inserting or removing an item, or disposing
 *    of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the index-th items for.
 *    
 *    key:
 *      A pointer to the item to be found ( or, enough of the item that
 *      it can be found in the tree ).
 *    
 *    dataPtr:
 *      An address in memory to return the data for the item, or NULL
 *      if you don not want data returned.  The memory point to must be
 *      large enough to hold all of the data for the item.
 *    
 *    itemSize:
 *      On return, the size of the data for the index-th item.  If you
 *      do not care about the size of the data, pass NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLFind( tree: AVLTreePtr; key: {const} UnivPtr; dataPtr: UnivPtr; var itemSize: ByteCount ): OSErr; external name '_AVLFind';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AVLGetRefcon()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Return the refCon set when the tree was created.
 *  
 *  Discussion:
 *    Get the refCon for the given tree ( set in AVLInit ) and return
 *    it.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    AVLGetRefcon is thread safe as long as no other thread tries to
 *    dispose of the tree itself.
 *  
 *  Parameters:
 *    
 *    tree:
 *      The tree to return the refcon for.
 *    
 *    refCon:
 *      On return, the refCon for the tree, or NULL if tree is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function AVLGetRefcon( tree: AVLTreePtr; var refCon: UnivPtr ): OSErr; external name '_AVLGetRefcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


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
