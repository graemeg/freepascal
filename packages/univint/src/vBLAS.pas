{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       vecLib/vBLAS.h
 
     Contains:   Header for the Basic Linear Algebra Subprograms, with Apple extensions.
 
     Version:    vecLib-$(vDSP_Version0).$(vDSP_Version1)
 
     Copyright:  © 2000-$(Year) by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       vBLAS.p
=======
     File:       vecLib/vBLAS.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Header for the Basic Linear Algebra Subprograms, with Apple extensions.
 
     Version:    vecLib-$(vDSP_Version0).$(vDSP_Version1)
 
<<<<<<< HEAD
     Copyright:  © 2000-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 2000-$(Year) by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
                     http://www.freepascal.org/bugs.html
 
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

unit vBLAS;
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
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
uses MacTypes,ConditionalMacros;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> origin/cpstrnew

<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
{$ALIGN POWER}

=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

>>>>>>> origin/cpstrnew
{ ==========================================================================================================================}


{
   =================================================================================================
   Definitions of the Basic Linear Algebra Subprograms (BLAS) as provided Apple Computer.
   A few additional functions, unique to Mac OS, have also been provided.
   These are clearly documented as Apple extensions.
   Documentation on the BLAS standard, including reference implementations, can be found on the web
   starting from the BLAS FAQ page at these URLs (verified live as of April 2002):
        http://www.netlib.org/blas/faq.html
        http://www.netlib.org/blas/blast-forum/blast-forum.html
   =================================================================================================
}


{
   =================================================================================================
   Matrix shape and storage
   ========================
   Keeping the various matrix shape and storage parameters straight can be difficult.  The BLAS
   documentation generally makes a distinction between the concpetual "matrix" and the physical
   "array".  However there are a number of places where this becomes fuzzy because of the overall
   bias towards FORTRAN's column major storage.  The confusion is made worse by style differences
   between the level 2 and level 3 functions.  It is amplified further by the explicit choice of row
   or column major storage in the C interface.
   The storage order does not affect the actual computation that is performed.  That is, it does not
   affect the results other than where they appear in memory.  It does affect the values passed
   for so-called "leading dimension" parameters, such as lda in sgemv.  These are always the major
   stride in storage, allowing operations on rectangular subsets of larger matrices.  For row major
   storage this is the number of columns in the parent matrix, and for column major storage this is
   the number of rows in the parent matrix.
   For the level 2 functions, which deal with only a single matrix, the matrix shape parameters are
   always M and N.  These are the logical shape of the matrix, M rows by N columns.  The transpose
   parameter, such as transA in sgemv, defines whether the regular matrix or its transpose is used
   in the operation.  This affects the implicit length of the input and output vectors.  For example,
   if the regular matrix A is used in sgemv, the input vector X has length N, the number of columns
   of A, and the output vector Y has length M, the number of rows of A.  The length of the input and
   output vectors is not affected by the storage order of the matrix.
   The level 3 functions deal with 2 input matrices and one output matrix, the matrix shape parameters
   are M, N, and K.  The logical shape of the output matrix is always M by N, while K is the common
   dimension of the input matrices.  Like level 2, the transpose parameters, such as transA and transB
   in sgemm, define whether the regular input or its transpose is used in the operation.  However
   unlike level 2, in level 3 the transpose parameters affect the implicit shape of the input matrix.
   Consider sgemm, which computes "C = (alpha * A * B) + (beta * C)", where A and B might be regular
   or transposed.  The logical shape of C is always M rows by N columns.  The physical shape depends
   on the storage order parameter.  Using column major storage the declaration of C (the array) in C
   (the language) would be something like "float C[N][M]".  The logical shape of A without transposition
   is M by K, and B is K by N.  The one storage order parameter affects all three matrices.
   For those readers still wondering about the style differences between level 2 and level 3, they
   involve whether the input or output shapes are explicit.  For level 2, the input matrix shape is
   always M by N.  The input and output vector lengths are implicit and vary according to the
   transpose parameter.  For level 3, the output matrix shape is always M by N.  The input matrix
   shapes are implicit and vary according to the transpose parameters.
   =================================================================================================
}


{ ==========================================================================================================================}


{
   ------------------------------------------------------------------------------------------------------------------
   IsAlignedCount   - True if an integer is positive and a multiple of 4.  Negative strides are considered unaligned.
   IsAlignedAddr    - True if an address is a multiple of 16.
}

// #define IsAlignedCount(n)   ( (n > 0) && ((n & 3) == 0) )
// #define IsAlignedAddr(a)    ( ((long)a & 15L) == 0 )

{
   =================================================================================================
   Prototypes for FORTRAN BLAS
   ===========================
   These are prototypes for the FORTRAN callable BLAS functions.  They are implemented in C for
   Mac OS, as thin shims that simply call the C BLAS counterpart.  These routines should never be
   called from C, but need to be included here so they will get output for the stub library.  It
   won't hurt to call them from C, but who would want to since you can't pass literals for sizes?
   FORTRAN compilers are typically MPW tools and use PPCLink, so they will link with the official
   vecLib stub from Apple.
   =================================================================================================
}
{
 *  SDOT()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SDOT( (*const*) var N: SInt32; {const} X: Float32Ptr; (*const*) var incX: SInt32; {const} Y: Float32Ptr; (*const*) var incY: SInt32 ): Float32; external name '_SDOT';
=======
function SDOT( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) Y: Float32; (*const*) var incY: SInt32 ): Float32; external name '_SDOT';
>>>>>>> graemeg/cpstrnew
=======
function SDOT( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) Y: Float32; (*const*) var incY: SInt32 ): Float32; external name '_SDOT';
>>>>>>> graemeg/cpstrnew
=======
function SDOT( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) Y: Float32; (*const*) var incY: SInt32 ): Float32; external name '_SDOT';
>>>>>>> graemeg/cpstrnew
=======
function SDOT( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) Y: Float32; (*const*) var incY: SInt32 ): Float32; external name '_SDOT';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SNRM2()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SNRM2( (*const*) var N: SInt32; {const} X: Float32Ptr; (*const*) var incX: SInt32 ): Float32; external name '_SNRM2';
=======
function SNRM2( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SNRM2';
>>>>>>> graemeg/cpstrnew
=======
function SNRM2( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SNRM2';
>>>>>>> graemeg/cpstrnew
=======
function SNRM2( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SNRM2';
>>>>>>> graemeg/cpstrnew
=======
function SNRM2( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SNRM2';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SASUM()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SASUM( (*const*) var N: SInt32; {const} X: Float32Ptr; (*const*) var incX: SInt32 ): Float32; external name '_SASUM';
=======
function SASUM( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SASUM';
>>>>>>> graemeg/cpstrnew
=======
function SASUM( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SASUM';
>>>>>>> graemeg/cpstrnew
=======
function SASUM( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SASUM';
>>>>>>> graemeg/cpstrnew
=======
function SASUM( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SASUM';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ISAMAX()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ISAMAX( (*const*) var N: SInt32; {const} X: Float32Ptr; (*const*) var incX: SInt32 ): SInt32; external name '_ISAMAX';
=======
function ISAMAX( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): SInt32; external name '_ISAMAX';
>>>>>>> graemeg/cpstrnew
=======
function ISAMAX( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): SInt32; external name '_ISAMAX';
>>>>>>> graemeg/cpstrnew
=======
function ISAMAX( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): SInt32; external name '_ISAMAX';
>>>>>>> graemeg/cpstrnew
=======
function ISAMAX( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): SInt32; external name '_ISAMAX';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SSWAP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SSWAP( (*const*) var N: SInt32; X: Float32Ptr; (*const*) var incX: SInt32; Y: Float32Ptr; (*const*) var incY: SInt32 ); external name '_SSWAP';
=======
procedure SSWAP( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SSWAP';
>>>>>>> graemeg/cpstrnew
=======
procedure SSWAP( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SSWAP';
>>>>>>> graemeg/cpstrnew
=======
procedure SSWAP( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SSWAP';
>>>>>>> graemeg/cpstrnew
=======
procedure SSWAP( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SSWAP';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SCOPY()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SCOPY( (*const*) var N: SInt32; {const} X: Float32Ptr; (*const*) var incX: SInt32; Y: Float32Ptr; (*const*) var incY: SInt32 ); external name '_SCOPY';
=======
procedure SCOPY( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SCOPY';
>>>>>>> graemeg/cpstrnew
=======
procedure SCOPY( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SCOPY';
>>>>>>> graemeg/cpstrnew
=======
procedure SCOPY( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SCOPY';
>>>>>>> graemeg/cpstrnew
=======
procedure SCOPY( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SCOPY';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SAXPY()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SAXPY( (*const*) var N: SInt32; const (*var*) alpha: Float32; {const} X: Float32Ptr; (*const*) var incX: SInt32; Y: Float32Ptr; (*const*) var incY: SInt32 ); external name '_SAXPY';
=======
procedure SAXPY( (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SAXPY';
>>>>>>> graemeg/cpstrnew
=======
procedure SAXPY( (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SAXPY';
>>>>>>> graemeg/cpstrnew
=======
procedure SAXPY( (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SAXPY';
>>>>>>> graemeg/cpstrnew
=======
procedure SAXPY( (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SAXPY';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SROT()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SROT( (*const*) var N: SInt32; X: Float32Ptr; (*const*) var incX: SInt32; Y: Float32Ptr; (*const*) var incY: SInt32; const (*var*) c: Float32; const (*var*) s: Float32 ); external name '_SROT';
=======
procedure SROT( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32; const (*var*) c: Float32; const (*var*) s: Float32 ); external name '_SROT';
>>>>>>> graemeg/cpstrnew
=======
procedure SROT( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32; const (*var*) c: Float32; const (*var*) s: Float32 ); external name '_SROT';
>>>>>>> graemeg/cpstrnew
=======
procedure SROT( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32; const (*var*) c: Float32; const (*var*) s: Float32 ); external name '_SROT';
>>>>>>> graemeg/cpstrnew
=======
procedure SROT( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32; const (*var*) c: Float32; const (*var*) s: Float32 ); external name '_SROT';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SSCAL()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SSCAL( (*const*) var N: SInt32; const (*var*) alpha: Float32; X: Float32Ptr; (*const*) var incX: SInt32 ); external name '_SSCAL';
=======
procedure SSCAL( (*const*) var N: SInt32; const (*var*) alpha: Float32; var X: Float32; (*const*) var incX: SInt32 ); external name '_SSCAL';
>>>>>>> graemeg/cpstrnew
=======
procedure SSCAL( (*const*) var N: SInt32; const (*var*) alpha: Float32; var X: Float32; (*const*) var incX: SInt32 ); external name '_SSCAL';
>>>>>>> graemeg/cpstrnew
=======
procedure SSCAL( (*const*) var N: SInt32; const (*var*) alpha: Float32; var X: Float32; (*const*) var incX: SInt32 ); external name '_SSCAL';
>>>>>>> graemeg/cpstrnew
=======
procedure SSCAL( (*const*) var N: SInt32; const (*var*) alpha: Float32; var X: Float32; (*const*) var incX: SInt32 ); external name '_SSCAL';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SGEMV()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SGEMV( transA: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; const (*var*) alpha: Float32; {const} A: Float32Ptr; (*const*) var lda: SInt32; {const} X: Float32Ptr; (*const*) var incX: SInt32; const (*var*) beta: Float32; Y: Float32Ptr; (*const*) var incY: SInt32 ); external name '_SGEMV';
=======
procedure SGEMV( transA: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) beta: Float32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SGEMV';
>>>>>>> graemeg/cpstrnew
=======
procedure SGEMV( transA: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) beta: Float32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SGEMV';
>>>>>>> graemeg/cpstrnew
=======
procedure SGEMV( transA: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) beta: Float32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SGEMV';
>>>>>>> graemeg/cpstrnew
=======
procedure SGEMV( transA: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) beta: Float32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SGEMV';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SGEMM()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure SGEMM( transA: ConstCStringPtr; transB: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; (*const*) var K: SInt32; const (*var*) alpha: Float32; {const} A: Float32Ptr; (*const*) var lda: SInt32; {const} B: Float32Ptr; (*const*) var ldb: SInt32; const (*var*) beta: Float32; C: Float32Ptr; (*const*) var ldc: SInt32 ); external name '_SGEMM';
=======
procedure SGEMM( transA: ConstCStringPtr; transB: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; (*const*) var K: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) B: Float32; (*const*) var ldb: SInt32; const (*var*) beta: Float32; var C: Float32; (*const*) var ldc: SInt32 ); external name '_SGEMM';
>>>>>>> graemeg/cpstrnew
=======
procedure SGEMM( transA: ConstCStringPtr; transB: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; (*const*) var K: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) B: Float32; (*const*) var ldb: SInt32; const (*var*) beta: Float32; var C: Float32; (*const*) var ldc: SInt32 ); external name '_SGEMM';
>>>>>>> graemeg/cpstrnew
=======
procedure SGEMM( transA: ConstCStringPtr; transB: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; (*const*) var K: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) B: Float32; (*const*) var ldb: SInt32; const (*var*) beta: Float32; var C: Float32; (*const*) var ldc: SInt32 ); external name '_SGEMM';
>>>>>>> graemeg/cpstrnew
=======
procedure SGEMM( transA: ConstCStringPtr; transB: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; (*const*) var K: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) B: Float32; (*const*) var ldb: SInt32; const (*var*) beta: Float32; var C: Float32; (*const*) var ldc: SInt32 ); external name '_SGEMM';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ ==========================================================================================================================}
{ ==========================================================================================================================}

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2

{$ALIGN POWER}


{
   ==========================================================================================================================
   Types and constants
   ===================
}


type
	CBLAS_ORDER 				= SInt32;
const
	CblasRowMajor				= 101;
	CblasColMajor				= 102;
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{ ==========================================================================================================================}


{
   =================================================================================================
   Definitions of the Basic Linear Algebra Subprograms (BLAS) as provided Apple Computer.
   A few additional functions, unique to Mac OS, have also been provided.
   These are clearly documented as Apple extensions.
   Documentation on the BLAS standard, including reference implementations, can be found on the web
   starting from the BLAS FAQ page at these URLs (verified live as of April 2002):
        http://www.netlib.org/blas/faq.html
        http://www.netlib.org/blas/blast-forum/blast-forum.html
   =================================================================================================
}


{
   =================================================================================================
   Matrix shape and storage
   ========================
   Keeping the various matrix shape and storage parameters straight can be difficult.  The BLAS
   documentation generally makes a distinction between the concpetual "matrix" and the physical
   "array".  However there are a number of places where this becomes fuzzy because of the overall
   bias towards FORTRAN's column major storage.  The confusion is made worse by style differences
   between the level 2 and level 3 functions.  It is amplified further by the explicit choice of row
   or column major storage in the C interface.
   The storage order does not affect the actual computation that is performed.  That is, it does not
   affect the results other than where they appear in memory.  It does affect the values passed
   for so-called "leading dimension" parameters, such as lda in sgemv.  These are always the major
   stride in storage, allowing operations on rectangular subsets of larger matrices.  For row major
   storage this is the number of columns in the parent matrix, and for column major storage this is
   the number of rows in the parent matrix.
   For the level 2 functions, which deal with only a single matrix, the matrix shape parameters are
   always M and N.  These are the logical shape of the matrix, M rows by N columns.  The transpose
   parameter, such as transA in sgemv, defines whether the regular matrix or its transpose is used
   in the operation.  This affects the implicit length of the input and output vectors.  For example,
   if the regular matrix A is used in sgemv, the input vector X has length N, the number of columns
   of A, and the output vector Y has length M, the number of rows of A.  The length of the input and
   output vectors is not affected by the storage order of the matrix.
   The level 3 functions deal with 2 input matrices and one output matrix, the matrix shape parameters
   are M, N, and K.  The logical shape of the output matrix is always M by N, while K is the common
   dimension of the input matrices.  Like level 2, the transpose parameters, such as transA and transB
   in sgemm, define whether the regular input or its transpose is used in the operation.  However
   unlike level 2, in level 3 the transpose parameters affect the implicit shape of the input matrix.
   Consider sgemm, which computes "C = (alpha * A * B) + (beta * C)", where A and B might be regular
   or transposed.  The logical shape of C is always M rows by N columns.  The physical shape depends
   on the storage order parameter.  Using column major storage the declaration of C (the array) in C
   (the language) would be something like "float C[N][M]".  The logical shape of A without transposition
   is M by K, and B is K by N.  The one storage order parameter affects all three matrices.
   For those readers still wondering about the style differences between level 2 and level 3, they
   involve whether the input or output shapes are explicit.  For level 2, the input matrix shape is
   always M by N.  The input and output vector lengths are implicit and vary according to the
   transpose parameter.  For level 3, the output matrix shape is always M by N.  The input matrix
   shapes are implicit and vary according to the transpose parameters.
   =================================================================================================
}


{ ==========================================================================================================================}


{
   ------------------------------------------------------------------------------------------------------------------
   IsAlignedCount   - True if an integer is positive and a multiple of 4.  Negative strides are considered unaligned.
   IsAlignedAddr    - True if an address is a multiple of 16.
}

// #define IsAlignedCount(n)   ( (n > 0) && ((n & 3) == 0) )
// #define IsAlignedAddr(a)    ( ((long)a & 15L) == 0 )

{
   =================================================================================================
   Prototypes for FORTRAN BLAS
   ===========================
   These are prototypes for the FORTRAN callable BLAS functions.  They are implemented in C for
   Mac OS, as thin shims that simply call the C BLAS counterpart.  These routines should never be
   called from C, but need to be included here so they will get output for the stub library.  It
   won't hurt to call them from C, but who would want to since you can't pass literals for sizes?
   FORTRAN compilers are typically MPW tools and use PPCLink, so they will link with the official
   vecLib stub from Apple.
   =================================================================================================
}
{
 *  SDOT()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
function SDOT( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) Y: Float32; (*const*) var incY: SInt32 ): Float32; external name '_SDOT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SNRM2()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
function SNRM2( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SNRM2';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SASUM()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
function SASUM( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): Float32; external name '_SASUM';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ISAMAX()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
function ISAMAX( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32 ): SInt32; external name '_ISAMAX';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SSWAP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SSWAP( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SSWAP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SCOPY()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SCOPY( (*const*) var N: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SCOPY';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SAXPY()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SAXPY( (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SAXPY';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SROT()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SROT( (*const*) var N: SInt32; var X: Float32; (*const*) var incX: SInt32; var Y: Float32; (*const*) var incY: SInt32; const (*var*) c: Float32; const (*var*) s: Float32 ); external name '_SROT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SSCAL()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SSCAL( (*const*) var N: SInt32; const (*var*) alpha: Float32; var X: Float32; (*const*) var incX: SInt32 ); external name '_SSCAL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SGEMV()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SGEMV( transA: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) X: Float32; (*const*) var incX: SInt32; const (*var*) beta: Float32; var Y: Float32; (*const*) var incY: SInt32 ); external name '_SGEMV';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SGEMM()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in vecLib.framework
 *    CarbonLib:        not in Carbon, but vecLib is compatible with CarbonLib
 *    Non-Carbon CFM:   in vecLib 1.0.2 and later
 }
procedure SGEMM( transA: ConstCStringPtr; transB: ConstCStringPtr; (*const*) var M: SInt32; (*const*) var N: SInt32; (*const*) var K: SInt32; const (*var*) alpha: Float32; const (*var*) A: Float32; (*const*) var lda: SInt32; const (*var*) B: Float32; (*const*) var ldb: SInt32; const (*var*) beta: Float32; var C: Float32; (*const*) var ldc: SInt32 ); external name '_SGEMM';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ ==========================================================================================================================}
{ ==========================================================================================================================}

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
