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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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

unit cblas;
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
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


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

type
	CBLAS_ORDER = SInt32;
const
	CblasRowMajor = 101;
	CblasColMajor = 102;
type
	CBLAS_TRANSPOSE = SInt32;
const
	CblasNoTrans = 111;
	CblasTrans = 112;
	CblasConjTrans = 113;
	AtlasConj = 114;
type
	CBLAS_UPLO = SInt32;
const
	CblasUpper = 121;
	CblasLower = 122;
type
	CBLAS_DIAG = SInt32;
const
	CblasNonUnit = 131;
	CblasUnit = 132;
type
	CBLAS_SIDE = SInt32;
const
	CblasLeft = 141;
	CblasRight = 142;

type
  CBLAS_INDEX = SInt32;

function cblas_errprn( ierr: SInt32; info: SInt32; var form: char; ... ): SInt32; external name '_cblas_errprn';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_xerbla( p: SInt32; var rout: char; var form: char; ... ); external name '_cblas_xerbla';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
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
 * ===========================================================================
 * Prototypes for level 1 BLAS functions (complex are recast as routines)
 * ===========================================================================
 }
function cblas_sdsdot( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdsdot';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_dsdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_dsdot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_sdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_ddot( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_ddot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
function cblas_dsdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_dsdot';
function cblas_sdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdot';
function cblas_ddot( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_ddot';
>>>>>>> graemeg/cpstrnew
=======
function cblas_dsdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_dsdot';
function cblas_sdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdot';
function cblas_ddot( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_ddot';
>>>>>>> graemeg/cpstrnew
=======
function cblas_dsdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_dsdot';
function cblas_sdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdot';
function cblas_ddot( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_ddot';
>>>>>>> graemeg/cpstrnew
=======
function cblas_dsdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_dsdot';
function cblas_sdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdot';
function cblas_ddot( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_ddot';
>>>>>>> origin/cpstrnew
=======
function cblas_dsdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_dsdot';
function cblas_sdot( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ): Float32; external name '_cblas_sdot';
function cblas_ddot( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ): Float64; external name '_cblas_ddot';
>>>>>>> origin/fixes_2.4
{
 * Functions having prefixes Z and C only
 }
procedure cblas_cdotu_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotu: UnivPtr ); external name '_cblas_cdotu_sub';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_cdotc_sub';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_zdotu_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotu: UnivPtr ); external name '_cblas_zdotu_sub';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_zdotc_sub';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
procedure cblas_cdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_cdotc_sub';

procedure cblas_zdotu_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotu: UnivPtr ); external name '_cblas_zdotu_sub';
procedure cblas_zdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_zdotc_sub';
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
procedure cblas_cdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_cdotc_sub';

procedure cblas_zdotu_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotu: UnivPtr ); external name '_cblas_zdotu_sub';
procedure cblas_zdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_zdotc_sub';
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2.4
procedure cblas_cdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_cdotc_sub';

procedure cblas_zdotu_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotu: UnivPtr ); external name '_cblas_zdotu_sub';
procedure cblas_zdotc_sub( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; dotc: UnivPtr ); external name '_cblas_zdotc_sub';
<<<<<<< HEAD
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 * Functions having prefixes S D SC DZ
 }
function cblas_snrm2( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32 ): Float32; external name '_cblas_snrm2';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_sasum( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32 ): Float32; external name '_cblas_sasum';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

function cblas_dnrm2( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): Float64; external name '_cblas_dnrm2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_dasum( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): Float64; external name '_cblas_dasum';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

function cblas_scnrm2( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float32; external name '_cblas_scnrm2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_scasum( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float32; external name '_cblas_scasum';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

function cblas_dznrm2( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float64; external name '_cblas_dznrm2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_dzasum( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float64; external name '_cblas_dzasum';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/fixes_2.4
function cblas_sasum( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32 ): Float32; external name '_cblas_sasum';

function cblas_dnrm2( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): Float64; external name '_cblas_dnrm2';
function cblas_dasum( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): Float64; external name '_cblas_dasum';

function cblas_scnrm2( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float32; external name '_cblas_scnrm2';
function cblas_scasum( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float32; external name '_cblas_scasum';

function cblas_dznrm2( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float64; external name '_cblas_dznrm2';
function cblas_dzasum( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float64; external name '_cblas_dzasum';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
function cblas_sasum( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32 ): Float32; external name '_cblas_sasum';

function cblas_dnrm2( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): Float64; external name '_cblas_dnrm2';
function cblas_dasum( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): Float64; external name '_cblas_dasum';

function cblas_scnrm2( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float32; external name '_cblas_scnrm2';
function cblas_scasum( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float32; external name '_cblas_scasum';

function cblas_dznrm2( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float64; external name '_cblas_dznrm2';
function cblas_dzasum( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): Float64; external name '_cblas_dzasum';
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 * Functions having standard 4 prefixes (S D C Z)
 }
function cblas_isamax( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_isamax';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_idamax( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_idamax';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_icamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_icamax';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
function cblas_izamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_izamax';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
function cblas_idamax( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_idamax';
function cblas_icamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_icamax';
function cblas_izamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_izamax';
>>>>>>> graemeg/cpstrnew
=======
function cblas_idamax( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_idamax';
function cblas_icamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_icamax';
function cblas_izamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_izamax';
>>>>>>> graemeg/cpstrnew
=======
function cblas_idamax( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_idamax';
function cblas_icamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_icamax';
function cblas_izamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_izamax';
>>>>>>> graemeg/cpstrnew
=======
function cblas_idamax( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_idamax';
function cblas_icamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_icamax';
function cblas_izamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_izamax';
>>>>>>> origin/cpstrnew
=======
function cblas_idamax( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_idamax';
function cblas_icamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_icamax';
function cblas_izamax( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32 ): CBLAS_INDEX; external name '_cblas_izamax';
>>>>>>> origin/fixes_2.4

{
 * ===========================================================================
 * Prototypes for level 1 BLAS routines
 * ===========================================================================
 }

{
 * Routines with standard 4 prefixes (s, d, c, z)
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure cblas_sswap( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_sswap';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_scopy( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_scopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_saxpy( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_saxpy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_saxpby( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_catlas_saxpby';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_sset( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32 ); external name '_catlas_sset';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_dswap( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dswap';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dcopy( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dcopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_daxpy( {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_daxpy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_daxpby( {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; Y: Float64Ptr; {const} incY: SInt32 ); external name '_catlas_daxpby';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_dset( {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32 ); external name '_catlas_dset';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_cswap( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_cswap';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ccopy( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_ccopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_caxpy( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_caxpy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_caxpby( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_catlas_caxpby';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_cset( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_catlas_cset';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_zswap( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zswap';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zcopy( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zcopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zaxpy( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zaxpy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_zaxpby( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_catlas_zaxpby';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure catlas_zset( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_catlas_zset';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure cblas_sswap( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_sswap';
procedure cblas_scopy( {const} N: SInt32; X: Float32Ptr; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_scopy';
procedure cblas_saxpy( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_saxpy';
procedure catlas_saxpby( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; var Y: Float32; {const} incY: SInt32 ); external name '_catlas_saxpby';
procedure catlas_sset( {const} N: SInt32; {const} alpha: Float32; var X: Float32; {const} incX: SInt32 ); external name '_catlas_sset';

procedure cblas_dswap( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dswap';
procedure cblas_dcopy( {const} N: SInt32; X: Float64Ptr; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dcopy';
procedure cblas_daxpy( {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_daxpy';
procedure catlas_daxpby( {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; var Y: Float64; {const} incY: SInt32 ); external name '_catlas_daxpby';
procedure catlas_dset( {const} N: SInt32; {const} alpha: Float64; var X: Float64; {const} incX: SInt32 ); external name '_catlas_dset';
                       
procedure cblas_cswap( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_cswap';
procedure cblas_ccopy( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_ccopy';
procedure cblas_caxpy( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_caxpy';
procedure catlas_caxpby( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_catlas_caxpby';
procedure catlas_cset( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_catlas_cset';

procedure cblas_zswap( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zswap';
procedure cblas_zcopy( {const} N: SInt32; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zcopy';
procedure cblas_zaxpy( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zaxpy';
procedure catlas_zaxpby( {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_catlas_zaxpby';
procedure catlas_zset( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_catlas_zset';
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
 * Routines with S and D prefix only
 }
procedure cblas_srotg( var a: Float32; var b: Float32; var c: Float32; var s: Float32 ); external name '_cblas_srotg';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_srotmg( var d1: Float32; var d2: Float32; var b1: Float32; {const} b2: Float32; var P: Float32 ); external name '_cblas_srotmg';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_srot( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_srot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_srotm( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; P: Float32Ptr ); external name '_cblas_srotm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_drotg( var a: Float64; var b: Float64; var c: Float64; var s: Float64 ); external name '_cblas_drotg';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_drotmg( var d1: Float64; var d2: Float64; var b1: Float64; {const} b2: Float64; var P: Float64 ); external name '_cblas_drotmg';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_drot( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_drot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_drotm( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; P: Float64Ptr ); external name '_cblas_drotm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure cblas_srotmg( var d1: Float32; var d2: Float32; var b1: Float32; {const} b2: Float32; var P: Float32 ); external name '_cblas_srotmg';
procedure cblas_srot( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_srot';
procedure cblas_srotm( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; P: Float32Ptr ); external name '_cblas_srotm';

procedure cblas_drotg( var a: Float64; var b: Float64; var c: Float64; var s: Float64 ); external name '_cblas_drotg';
procedure cblas_drotmg( var d1: Float64; var d2: Float64; var b1: Float64; {const} b2: Float64; var P: Float64 ); external name '_cblas_drotmg';
procedure cblas_drot( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_drot';
procedure cblas_drotm( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; P: Float64Ptr ); external name '_cblas_drotm';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
procedure cblas_srotmg( var d1: Float32; var d2: Float32; var b1: Float32; {const} b2: Float32; var P: Float32 ); external name '_cblas_srotmg';
procedure cblas_srot( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_srot';
procedure cblas_srotm( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; P: Float32Ptr ); external name '_cblas_srotm';

procedure cblas_drotg( var a: Float64; var b: Float64; var c: Float64; var s: Float64 ); external name '_cblas_drotg';
procedure cblas_drotmg( var d1: Float64; var d2: Float64; var b1: Float64; {const} b2: Float64; var P: Float64 ); external name '_cblas_drotmg';
procedure cblas_drot( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_drot';
procedure cblas_drotm( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; P: Float64Ptr ); external name '_cblas_drotm';
>>>>>>> graemeg/cpstrnew
=======
procedure cblas_srotmg( var d1: Float32; var d2: Float32; var b1: Float32; {const} b2: Float32; var P: Float32 ); external name '_cblas_srotmg';
procedure cblas_srot( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_srot';
procedure cblas_srotm( {const} N: SInt32; var X: Float32; {const} incX: SInt32; var Y: Float32; {const} incY: SInt32; P: Float32Ptr ); external name '_cblas_srotm';

procedure cblas_drotg( var a: Float64; var b: Float64; var c: Float64; var s: Float64 ); external name '_cblas_drotg';
procedure cblas_drotmg( var d1: Float64; var d2: Float64; var b1: Float64; {const} b2: Float64; var P: Float64 ); external name '_cblas_drotmg';
procedure cblas_drot( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_drot';
procedure cblas_drotm( {const} N: SInt32; var X: Float64; {const} incX: SInt32; var Y: Float64; {const} incY: SInt32; P: Float64Ptr ); external name '_cblas_drotm';
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 * Routines with S D C Z CS and ZD prefixes
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure cblas_sscal( {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_sscal';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dscal( {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dscal';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cscal( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_cscal';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zscal( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_zscal';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_csscal( {const} N: SInt32; {const} alpha: Float32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_csscal';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zdscal( {const} N: SInt32; {const} alpha: Float64; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_zdscal';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure cblas_sscal( {const} N: SInt32; {const} alpha: Float32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_sscal';
procedure cblas_dscal( {const} N: SInt32; {const} alpha: Float64; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dscal';
procedure cblas_cscal( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_cscal';
procedure cblas_zscal( {const} N: SInt32; alpha: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_zscal';
procedure cblas_csscal( {const} N: SInt32; {const} alpha: Float32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_csscal';
procedure cblas_zdscal( {const} N: SInt32; {const} alpha: Float64; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_zdscal';
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
 * Extra reference routines provided by ATLAS, but not mandated by the standard
 }
procedure cblas_crotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_crotg';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zrotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_zrotg';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_csrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_csrot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zdrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_zdrot';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
procedure cblas_zrotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_zrotg';
procedure cblas_csrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_csrot';
procedure cblas_zdrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_zdrot';
>>>>>>> graemeg/cpstrnew
=======
procedure cblas_zrotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_zrotg';
procedure cblas_csrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_csrot';
procedure cblas_zdrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_zdrot';
>>>>>>> graemeg/cpstrnew
=======
procedure cblas_zrotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_zrotg';
procedure cblas_csrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_csrot';
procedure cblas_zdrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_zdrot';
>>>>>>> graemeg/cpstrnew
=======
procedure cblas_zrotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_zrotg';
procedure cblas_csrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_csrot';
procedure cblas_zdrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_zdrot';
>>>>>>> origin/cpstrnew
=======
procedure cblas_zrotg( a: UnivPtr; b: UnivPtr; c: UnivPtr; s: UnivPtr ); external name '_cblas_zrotg';
procedure cblas_csrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float32; {const} s: Float32 ); external name '_cblas_csrot';
procedure cblas_zdrot( {const} N: SInt32; X: UnivPtr; {const} incX: SInt32; Y: UnivPtr; {const} incY: SInt32; {const} c: Float64; {const} s: Float64 ); external name '_cblas_zdrot';
>>>>>>> origin/fixes_2.4

{
 * ===========================================================================
 * Prototypes for level 2 BLAS
 * ===========================================================================
 }

{
 * Routines with standard 4 prefixes (S, D, C, Z)
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure cblas_sgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_sgemv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_sgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_sgbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_strmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_strmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_stbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_stbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_stpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} Ap: Float32Ptr; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_stpmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_strsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_strsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_stbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_stbsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_stpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} Ap: Float32Ptr; X: Float32Ptr; {const} incX: SInt32 ); external name '_cblas_stpsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_dgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dgemv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dgbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtrmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dtrmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dtbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} Ap: Float64Ptr; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dtpmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtrsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dtrsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dtbsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} Ap: Float64Ptr; X: Float64Ptr; {const} incX: SInt32 ); external name '_cblas_dtpsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_cgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_cgemv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_cgbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctrmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctrmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctpmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctrsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctrsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctbsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctpsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_zgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zgemv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zgbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztrmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztrmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztpmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztrsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztrsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztbsv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure cblas_sgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_sgemv';
procedure cblas_sgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_sgbmv';
procedure cblas_strmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float32Ptr; {const} lda: SInt32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_strmv';
procedure cblas_stbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float32Ptr; {const} lda: SInt32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_stbmv';
procedure cblas_stpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; const (*var*) Ap: Float32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_stpmv';
procedure cblas_strsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float32Ptr; {const} lda: SInt32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_strsv';
procedure cblas_stbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float32Ptr; {const} lda: SInt32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_stbsv';
procedure cblas_stpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; const (*var*) Ap: Float32; var X: Float32; {const} incX: SInt32 ); external name '_cblas_stpsv';

procedure cblas_dgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dgemv';
procedure cblas_dgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dgbmv';
procedure cblas_dtrmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float64Ptr; {const} lda: SInt32; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dtrmv';
procedure cblas_dtbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float64Ptr; {const} lda: SInt32; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dtbmv';
procedure cblas_dtpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; const (*var*) Ap: Float64; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dtpmv';
procedure cblas_dtrsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: Float64Ptr; {const} lda: SInt32; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dtrsv';
procedure cblas_dtbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: Float64Ptr; {const} lda: SInt32; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dtbsv';
procedure cblas_dtpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; const (*var*) Ap: Float64; var X: Float64; {const} incX: SInt32 ); external name '_cblas_dtpsv';

procedure cblas_cgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_cgemv';
procedure cblas_cgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_cgbmv';
procedure cblas_ctrmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctrmv';
procedure cblas_ctbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctbmv';
procedure cblas_ctpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctpmv';
procedure cblas_ctrsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctrsv';
procedure cblas_ctbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctbsv';
procedure cblas_ctpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ctpsv';

procedure cblas_zgemv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zgemv';
procedure cblas_zgbmv( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} KL: SInt32; {const} KU: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zgbmv';
procedure cblas_ztrmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztrmv';
procedure cblas_ztbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztbmv';
procedure cblas_ztpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztpmv';
procedure cblas_ztrsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztrsv';
procedure cblas_ztbsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; {const} K: SInt32; A: {const} UnivPtr; {const} lda: SInt32; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztbsv';
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
procedure cblas_ztpsv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} N: SInt32; Ap: {const} UnivPtr; X: UnivPtr; {const} incX: SInt32 ); external name '_cblas_ztpsv';


{
 * Routines with S and D prefixes only
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure cblas_ssymv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_ssymv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ssbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_ssbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_sspmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; {const} Ap: Float32Ptr; {const} X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; Y: Float32Ptr; {const} incY: SInt32 ); external name '_cblas_sspmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_sger( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32; A: Float32Ptr; {const} lda: SInt32 ); external name '_cblas_sger';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ssyr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; A: Float32Ptr; {const} lda: SInt32 ); external name '_cblas_ssyr';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_sspr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Ap: Float32Ptr ); external name '_cblas_sspr';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ssyr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32; A: Float32Ptr; {const} lda: SInt32 ); external name '_cblas_ssyr2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_sspr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32; A: Float32Ptr ); external name '_cblas_sspr2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_dsymv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dsymv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dsbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dsbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dspmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; {const} Ap: Float64Ptr; {const} X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; Y: Float64Ptr; {const} incY: SInt32 ); external name '_cblas_dspmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dger( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32; A: Float64Ptr; {const} lda: SInt32 ); external name '_cblas_dger';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dsyr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; A: Float64Ptr; {const} lda: SInt32 ); external name '_cblas_dsyr';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dspr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Ap: Float64Ptr ); external name '_cblas_dspr';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dsyr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32; A: Float64Ptr; {const} lda: SInt32 ); external name '_cblas_dsyr2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dspr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32; A: Float64Ptr ); external name '_cblas_dspr2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure cblas_ssymv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_ssymv';
procedure cblas_ssbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_ssbmv';
procedure cblas_sspmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; const (*var*) Ap: Float32; X: Float32Ptr; {const} incX: SInt32; {const} beta: Float32; var Y: Float32; {const} incY: SInt32 ); external name '_cblas_sspmv';
procedure cblas_sger( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32; var A: Float32; {const} lda: SInt32 ); external name '_cblas_sger';
procedure cblas_ssyr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; var A: Float32; {const} lda: SInt32 ); external name '_cblas_ssyr';
procedure cblas_sspr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; var Ap: Float32 ); external name '_cblas_sspr';
procedure cblas_ssyr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32; var A: Float32; {const} lda: SInt32 ); external name '_cblas_ssyr2';
procedure cblas_sspr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: Float32Ptr; {const} incX: SInt32; Y: Float32Ptr; {const} incY: SInt32; var A: Float32 ); external name '_cblas_sspr2';

procedure cblas_dsymv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dsymv';
procedure cblas_dsbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dsbmv';
procedure cblas_dspmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; const (*var*) Ap: Float64; X: Float64Ptr; {const} incX: SInt32; {const} beta: Float64; var Y: Float64; {const} incY: SInt32 ); external name '_cblas_dspmv';
procedure cblas_dger( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32; var A: Float64; {const} lda: SInt32 ); external name '_cblas_dger';
procedure cblas_dsyr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; var A: Float64; {const} lda: SInt32 ); external name '_cblas_dsyr';
procedure cblas_dspr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; var Ap: Float64 ); external name '_cblas_dspr';
procedure cblas_dsyr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32; var A: Float64; {const} lda: SInt32 ); external name '_cblas_dsyr2';
procedure cblas_dspr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: Float64Ptr; {const} incX: SInt32; Y: Float64Ptr; {const} incY: SInt32; var A: Float64 ); external name '_cblas_dspr2';
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
 * Routines with C and Z prefixes only
 }
procedure cblas_chemv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chemv';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_chbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_chpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chpmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgeru';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgerc';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_chpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_chpr';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_chpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_chpr2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_zhemv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhemv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zhbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhbmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zhpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhpmv';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgeru';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgerc';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zhpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_zhpr';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zhpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_zhpr2';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
procedure cblas_chbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chbmv';
procedure cblas_chpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chpmv';
procedure cblas_cgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgeru';
procedure cblas_cgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgerc';
procedure cblas_cher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher';
procedure cblas_chpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_chpr';
procedure cblas_cher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher2';
procedure cblas_chpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_chpr2';

procedure cblas_zhemv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhemv';
procedure cblas_zhbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhbmv';
procedure cblas_zhpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhpmv';
procedure cblas_zgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgeru';
procedure cblas_zgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgerc';
procedure cblas_zher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher';
procedure cblas_zhpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_zhpr';
procedure cblas_zher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher2';
procedure cblas_zhpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_zhpr2';
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
procedure cblas_chbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chbmv';
procedure cblas_chpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chpmv';
procedure cblas_cgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgeru';
procedure cblas_cgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgerc';
procedure cblas_cher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher';
procedure cblas_chpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_chpr';
procedure cblas_cher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher2';
procedure cblas_chpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_chpr2';

procedure cblas_zhemv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhemv';
procedure cblas_zhbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhbmv';
procedure cblas_zhpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhpmv';
procedure cblas_zgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgeru';
procedure cblas_zgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgerc';
procedure cblas_zher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher';
procedure cblas_zhpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_zhpr';
procedure cblas_zher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher2';
procedure cblas_zhpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_zhpr2';
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2.4
procedure cblas_chbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chbmv';
procedure cblas_chpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_chpmv';
procedure cblas_cgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgeru';
procedure cblas_cgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cgerc';
procedure cblas_cher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher';
procedure cblas_chpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float32; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_chpr';
procedure cblas_cher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_cher2';
procedure cblas_chpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_chpr2';

procedure cblas_zhemv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhemv';
procedure cblas_zhbmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhbmv';
procedure cblas_zhpmv( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; Ap: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; beta: {const} UnivPtr; Y: UnivPtr; {const} incY: SInt32 ); external name '_cblas_zhpmv';
procedure cblas_zgeru( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgeru';
procedure cblas_zgerc( {const} Order: CBLAS_ORDER; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zgerc';
procedure cblas_zher( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher';
procedure cblas_zhpr( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; {const} alpha: Float64; X: {const} UnivPtr; {const} incX: SInt32; A: UnivPtr ); external name '_cblas_zhpr';
procedure cblas_zher2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; A: UnivPtr; {const} lda: SInt32 ); external name '_cblas_zher2';
procedure cblas_zhpr2( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} N: SInt32; alpha: {const} UnivPtr; X: {const} UnivPtr; {const} incX: SInt32; Y: {const} UnivPtr; {const} incY: SInt32; Ap: UnivPtr ); external name '_cblas_zhpr2';
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{
 * ===========================================================================
 * Prototypes for level 3 BLAS
 * ===========================================================================
 }

{
 * Routines with standard 4 prefixes (S, D, C, Z)
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure cblas_sgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32; {const} beta: Float32; C: Float32Ptr; {const} ldc: SInt32 ); external name '_cblas_sgemm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ssymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32; {const} beta: Float32; C: Float32Ptr; {const} ldc: SInt32 ); external name '_cblas_ssymm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ssyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; {const} beta: Float32; C: Float32Ptr; {const} ldc: SInt32 ); external name '_cblas_ssyrk';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ssyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32; {const} beta: Float32; C: Float32Ptr; {const} ldc: SInt32 ); external name '_cblas_ssyr2k';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_strmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32 ); external name '_cblas_strmm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_strsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32 ); external name '_cblas_strsm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_dgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32; {const} beta: Float64; C: Float64Ptr; {const} ldc: SInt32 ); external name '_cblas_dgemm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dsymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32; {const} beta: Float64; C: Float64Ptr; {const} ldc: SInt32 ); external name '_cblas_dsymm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dsyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; {const} beta: Float64; C: Float64Ptr; {const} ldc: SInt32 ); external name '_cblas_dsyrk';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dsyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32; {const} beta: Float64; C: Float64Ptr; {const} ldc: SInt32 ); external name '_cblas_dsyr2k';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtrmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32 ); external name '_cblas_dtrmm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_dtrsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32 ); external name '_cblas_dtrsm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_cgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_cgemm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_csymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_csymm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_csyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_csyrk';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_csyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_csyr2k';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctrmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ctrmm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ctrsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ctrsm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

procedure cblas_zgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zgemm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zsymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zsymm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zsyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zsyrk';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zsyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zsyr2k';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztrmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ztrmm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_ztrsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ztrsm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure cblas_sgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32; {const} beta: Float32; var C: Float32; {const} ldc: SInt32 ); external name '_cblas_sgemm';
procedure cblas_ssymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32; {const} beta: Float32; var C: Float32; {const} ldc: SInt32 ); external name '_cblas_ssymm';
procedure cblas_ssyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; {const} beta: Float32; var C: Float32; {const} ldc: SInt32 ); external name '_cblas_ssyrk';
procedure cblas_ssyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; B: Float32Ptr; {const} ldb: SInt32; {const} beta: Float32; var C: Float32; {const} ldc: SInt32 ); external name '_cblas_ssyr2k';
procedure cblas_strmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; var B: Float32; {const} ldb: SInt32 ); external name '_cblas_strmm';
procedure cblas_strsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float32; A: Float32Ptr; {const} lda: SInt32; var B: Float32; {const} ldb: SInt32 ); external name '_cblas_strsm';

procedure cblas_dgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32; {const} beta: Float64; var C: Float64; {const} ldc: SInt32 ); external name '_cblas_dgemm';
procedure cblas_dsymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32; {const} beta: Float64; var C: Float64; {const} ldc: SInt32 ); external name '_cblas_dsymm';
procedure cblas_dsyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; {const} beta: Float64; var C: Float64; {const} ldc: SInt32 ); external name '_cblas_dsyrk';
procedure cblas_dsyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; B: Float64Ptr; {const} ldb: SInt32; {const} beta: Float64; var C: Float64; {const} ldc: SInt32 ); external name '_cblas_dsyr2k';
procedure cblas_dtrmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; var B: Float64; {const} ldb: SInt32 ); external name '_cblas_dtrmm';
procedure cblas_dtrsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; {const} alpha: Float64; A: Float64Ptr; {const} lda: SInt32; var B: Float64; {const} ldb: SInt32 ); external name '_cblas_dtrsm';

procedure cblas_cgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_cgemm';
procedure cblas_csymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_csymm';
procedure cblas_csyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_csyrk';
procedure cblas_csyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_csyr2k';
procedure cblas_ctrmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ctrmm';
procedure cblas_ctrsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ctrsm';

procedure cblas_zgemm( {const} Order: CBLAS_ORDER; {const} TransA: CBLAS_TRANSPOSE; {const} TransB: CBLAS_TRANSPOSE; {const} M: SInt32; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zgemm';
procedure cblas_zsymm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zsymm';
procedure cblas_zsyrk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zsyrk';
procedure cblas_zsyr2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zsyr2k';
procedure cblas_ztrmm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ztrmm';
procedure cblas_ztrsm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} TransA: CBLAS_TRANSPOSE; {const} Diag: CBLAS_DIAG; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: UnivPtr; {const} ldb: SInt32 ); external name '_cblas_ztrsm';
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
 * Routines with prefixes C and Z only
 }
procedure cblas_chemm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_chemm';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cherk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: {const} UnivPtr; {const} lda: SInt32; {const} beta: Float32; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_cherk';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_cher2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; {const} beta: Float32; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_cher2k';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zhemm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zhemm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zherk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: {const} UnivPtr; {const} lda: SInt32; {const} beta: Float64; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zherk';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
procedure cblas_zher2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; {const} beta: Float64; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zher2k';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

{
 Apple extensions follow.
 }

{
 The level 3 BLAS may allocate a large (approximately 4Mb) buffer to hold intermediate operands
 and results. These intermediate quantities are organized in memory for efficient access and
 so contribute to optimal performance. By default, this buffer is retained across calls to the
 BLAS. This strategy has substantial advantages when the BLAS are executed repeatedly. Clients
 who wish to free this buffer and return the memory allocation to the malloc heap can call the
 following routine at any time. Note that subsequent calls to the level 3 BLAS may allocate this
 buffer again.
 }

procedure ATLU_DestroyThreadMemory; external name '_ATLU_DestroyThreadMemory';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)

{
 -------------------------------------------------------------------------------------------------
 The BLAS standard requires that parameter errors be reported and cause the program to terminate.
 The default behavior for the Mac OS implementation of the BLAS is to print a message in English
 to stdout using printf and call exit with EXIT_FAILURE as the status.  If this is adequate, then
 you need do nothing more or worry about error handling.
 The BLAS standard also mentions a function, cblas_xerbla, suggesting that a program provide its
 own implementation to override the default error handling.  This will not work in the shared
 library environment of Mac OS.  Instead the Mac OS implementation provides a means to install
 an error handler.  There can only be one active error handler, installing a new one causes any
 previous handler to be forgotten.  Passing a null function pointer installs the default handler.
 The default handler is automatically installed at startup and implements the default behavior
 defined above.
 An error handler may return, it need not abort the program.  If the error handler returns, the
 BLAS routine also returns immediately without performing any processing.  Level 1 functions that
 return a numeric value return zero if the error handler returns.
 -------------------------------------------------------------------------------------------------
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
procedure cblas_cherk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float32; A: {const} UnivPtr; {const} lda: SInt32; {const} beta: Float32; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_cherk';
procedure cblas_cher2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; {const} beta: Float32; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_cher2k';
procedure cblas_zhemm( {const} Order: CBLAS_ORDER; {const} Side: CBLAS_SIDE; {const} Uplo: CBLAS_UPLO; {const} M: SInt32; {const} N: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; beta: {const} UnivPtr; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zhemm';
procedure cblas_zherk( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; {const} alpha: Float64; A: {const} UnivPtr; {const} lda: SInt32; {const} beta: Float64; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zherk';
procedure cblas_zher2k( {const} Order: CBLAS_ORDER; {const} Uplo: CBLAS_UPLO; {const} Trans: CBLAS_TRANSPOSE; {const} N: SInt32; {const} K: SInt32; alpha: {const} UnivPtr; A: {const} UnivPtr; {const} lda: SInt32; B: {const} UnivPtr; {const} ldb: SInt32; {const} beta: Float64; C: UnivPtr; {const} ldc: SInt32 ); external name '_cblas_zher2k';

{
   Apple extensions follow.
}

{
   The level 3 BLAS may allocate a large (approximately 4Mb) buffer to hold intermediate operands
   and results. These intermediate quantities are organized in memory for efficient access and
   so contribute to optimal performance. By default, this buffer is retained across calls to the
   BLAS. This strategy has substantial advantages when the BLAS are executed repeatedly. Clients
   who wish to free this buffer and return the memory allocation to the malloc heap can call the
   following routine at any time. Note that subsequent calls to the level 3 BLAS may allocate this
   buffer again.
}

procedure ATLU_DestroyThreadMemory; external name '_ATLU_DestroyThreadMemory';

{
   -------------------------------------------------------------------------------------------------
   The BLAS standard requires that parameter errors be reported and cause the program to terminate.
   The default behavior for the Mac OS implementation of the BLAS is to print a message in English
   to stdout using printf and call exit with EXIT_FAILURE as the status.  If this is adequate, then
   you need do nothing more or worry about error handling.
   The BLAS standard also mentions a function, cblas_xerbla, suggesting that a program provide its
   own implementation to override the default error handling.  This will not work in the shared
   library environment of Mac OS.  Instead the Mac OS implementation provides a means to install
   an error handler.  There can only be one active error handler, installing a new one causes any
   previous handler to be forgotten.  Passing a null function pointer installs the default handler.
   The default handler is automatically installed at startup and implements the default behavior
   defined above.
   An error handler may return, it need not abort the program.  If the error handler returns, the
   BLAS routine also returns immediately without performing any processing.  Level 1 functions that
   return a numeric value return zero if the error handler returns.
   -------------------------------------------------------------------------------------------------
}
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

type
	BLASParamErrorProc = procedure( funcName: ConstCStringPtr; paramName: ConstCStringPtr; (*const*) var paramPos: SInt32; (*const*) var paramValue: SInt32 );
procedure SetBLASParamErrorProc( ErrorProc: BLASParamErrorProc ); external name '_SetBLASParamErrorProc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_4_0) *)
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

{$ifc not undefined TARGET_CPU_PPC or defined TARGET_CPU_PPC64}

{$elsec}
  {$ifc not undefined TARGET_CPU_X86 or defined TARGET_CPU_X86_64}

  {$elsec}
	   {$error Unknown architecture}
  {$endc}
{$endc}

{$ifc not undefined __VEC__ or defined __SSE__}
(*
{
   -------------------------------------------------------------------------------------------------
   These routines provide optimized, SIMD-only support for common small matrix multiplications.
   They do not check for the availability of SIMD instructions or parameter errors.  They just do
   the multiplication as fast as possible.  Matrices are presumed to use row major storage.  Because
   these are all square, column major matrices can be multiplied by simply reversing the parameters.
   -------------------------------------------------------------------------------------------------
}


procedure vMultVecMat_4x4( ConstVectorFloat X[1]; ConstVectorFloat A[4][1]; VectorFloat Y[1] ); external name '_vMultVecMat_4x4';
procedure vMultMatVec_4x4( ConstVectorFloat A[4][1]; ConstVectorFloat X[1]; VectorFloat Y[1] ); external name '_vMultMatVec_4x4';
procedure vMultMatMat_4x4( ConstVectorFloat A[4][1]; ConstVectorFloat B[4][1]; VectorFloat C[4][1] ); external name '_vMultMatMat_4x4';
procedure vMultVecMat_8x8( ConstVectorFloat X[2]; ConstVectorFloat A[8][2]; VectorFloat Y[2] ); external name '_vMultVecMat_8x8';
procedure vMultMatVec_8x8( ConstVectorFloat A[8][2]; ConstVectorFloat X[2]; VectorFloat Y[2] ); external name '_vMultMatVec_8x8';
procedure vMultMatMat_8x8( ConstVectorFloat A[8][2]; ConstVectorFloat B[8][2]; VectorFloat C[8][2] ); external name '_vMultMatMat_8x8';
procedure vMultVecMat_16x16( ConstVectorFloat X[4]; ConstVectorFloat A[16][4]; VectorFloat Y[4] ); external name '_vMultVecMat_16x16';
procedure vMultMatVec_16x16( ConstVectorFloat A[16][4]; ConstVectorFloat X[4]; VectorFloat Y[4] ); external name '_vMultMatVec_16x16';
procedure vMultMatMat_16x16( ConstVectorFloat A[16][4]; ConstVectorFloat B[16][4]; VectorFloat C[16][4] ); external name '_vMultMatMat_16x16';
procedure vMultVecMat_32x32( ConstVectorFloat X[8]; ConstVectorFloat A[32][8]; VectorFloat Y[8] ); external name '_vMultVecMat_32x32';
procedure vMultMatVec_32x32( ConstVectorFloat A[32][8]; ConstVectorFloat X[8]; VectorFloat Y[8] ); external name '_vMultMatVec_32x32';
procedure vMultMatMat_32x32( ConstVectorFloat A[32][8]; ConstVectorFloat B[32][8]; VectorFloat C[32][8] ); external name '_vMultMatMat_32x32';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    
    {
     -------------------------------------------------------------------------------------------------
     These routines provide optimized support for common small matrix multiplications. They use
     the scalar floating point unit and have no dependancy on SIMD instructions. They are intended
     as complements to the AltiVec-only routines above. They do not check for parameter errors.  They just do
     the multiplication as fast as possible.  Matrices are presumed to use row major storage.  Because
     these are all square, column major matrices can be multiplied by simply reversing the parameters.
     -------------------------------------------------------------------------------------------------
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

{
   -------------------------------------------------------------------------------------------------
   These routines provide optimized support for common small matrix multiplications. They use
   the scalar floating point unit and have no dependancy on SIMD instructions. They are intended
   as complements to the AltiVec-only routines above. They do not check for parameter errors.  They just do
   the multiplication as fast as possible.  Matrices are presumed to use row major storage.  Because
   these are all square, column major matrices can be multiplied by simply reversing the parameters.
   -------------------------------------------------------------------------------------------------
}
  
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
procedure sMultVecMat_4x4( ConstVectorFloat X[1]; ConstVectorFloat A[4][1]; VectorFloat Y[1] ); external name '_sMultVecMat_4x4';
procedure sMultMatVec_4x4( ConstVectorFloat A[4][1]; ConstVectorFloat X[1]; VectorFloat Y[1] ); external name '_sMultMatVec_4x4';
procedure sMultMatMat_4x4( ConstVectorFloat A[4][1]; ConstVectorFloat B[4][1]; VectorFloat C[4][1] ); external name '_sMultMatMat_4x4';
procedure sMultVecMat_8x8( ConstVectorFloat X[2]; ConstVectorFloat A[8][2]; VectorFloat Y[2] ); external name '_sMultVecMat_8x8';
procedure sMultMatVec_8x8( ConstVectorFloat A[8][2]; ConstVectorFloat X[2]; VectorFloat Y[2] ); external name '_sMultMatVec_8x8';
procedure sMultMatMat_8x8( ConstVectorFloat A[8][2]; ConstVectorFloat B[8][2]; VectorFloat C[8][2] ); external name '_sMultMatMat_8x8';
procedure sMultVecMat_16x16( ConstVectorFloat X[4]; ConstVectorFloat A[16][4]; VectorFloat Y[4] ); external name '_sMultVecMat_16x16';
procedure sMultMatVec_16x16( ConstVectorFloat A[16][4]; ConstVectorFloat X[4]; VectorFloat Y[4] ); external name '_sMultMatVec_16x16';
procedure sMultMatMat_16x16( ConstVectorFloat A[16][4]; ConstVectorFloat B[16][4]; VectorFloat C[16][4] ); external name '_sMultMatMat_16x16';
procedure sMultVecMat_32x32( ConstVectorFloat X[8]; ConstVectorFloat A[32][8]; VectorFloat Y[8] ); external name '_sMultVecMat_32x32';
procedure sMultMatVec_32x32( ConstVectorFloat A[32][8]; ConstVectorFloat X[8]; VectorFloat Y[8] ); external name '_sMultMatVec_32x32';
procedure sMultMatMat_32x32( ConstVectorFloat A[32][8]; ConstVectorFloat B[32][8]; VectorFloat C[32][8] ); external name '_sMultMatMat_32x32';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    
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
procedure dMultVecMat_4x4( const double X[4]; const double A[4][4]; double Y[4] ); external name '_dMultVecMat_4x4';
procedure dMultMatVec_4x4( const double A[4][4]; const double X[4]; double Y[4] ); external name '_dMultMatVec_4x4';
procedure dMultMatMat_4x4( const double A[4][4]; const double B[4][4]; double C[4][4] ); external name '_dMultMatMat_4x4';
procedure dMultVecMat_8x8( const double X[8]; const double A[8][8]; double Y[8] ); external name '_dMultVecMat_8x8';
procedure dMultMatVec_8x8( const double A[8][8]; const double X[8]; double Y[8] ); external name '_dMultMatVec_8x8';
procedure dMultMatMat_8x8( const double A[8][8]; const double B[8][8]; double C[8][8] ); external name '_dMultMatMat_8x8';
procedure dMultVecMat_16x16( const double X[16]; const double A[16][16]; double Y[16] ); external name '_dMultVecMat_16x16';
procedure dMultMatVec_16x16( const double A[16][16]; const double X[16]; double Y[16] ); external name '_dMultMatVec_16x16';
procedure dMultMatMat_16x16( const double A[16][16]; const double B[16][16]; double C[16][16] ); external name '_dMultMatMat_16x16';
procedure dMultVecMat_32x32( const double X[32]; const double A[32][32]; double Y[32] ); external name '_dMultVecMat_32x32';
procedure dMultMatVec_32x32( const double A[32][32]; const double X[32]; double Y[32] ); external name '_dMultMatVec_32x32';
procedure dMultMatMat_32x32( const double A[32][32]; const double B[32][32]; double C[32][32] ); external name '_dMultMatMat_32x32';
*)

{$endc} { defined(__VEC__) || defined(__SSE__)}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
