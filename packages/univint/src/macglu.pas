{
    Copyright:  (c) 1999-2008 Apple Inc. All rights reserved.
}
{       Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }

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

unit macglu;
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
>>>>>>> graemeg/cpstrnew
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
uses MacTypes, macgl;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{***********************************************************}

{ Extensions }
const
	GLU_EXT_object_space_tess = 1;
const
	GLU_EXT_nurbs_tessellator = 1;

{ Boolean }
const
	GLU_FALSE = 0;
const
	GLU_TRUE = 1;

{ Version }
{$definec	GLU_VERSION_1_1 TRUE}
{$definec	GLU_VERSION_1_2 TRUE}
{$definec	GLU_VERSION_1_3 TRUE}

{ StringName }
const
	GLU_VERSION = 100800;
const
	GLU_EXTENSIONS = 100801;

{ ErrorCode }
const
	GLU_INVALID_ENUM = 100900;
const
	GLU_INVALID_VALUE = 100901;
const
	GLU_OUT_OF_MEMORY = 100902;
const
	GLU_INCOMPATIBLE_GL_VERSION = 100903;
const
	GLU_INVALID_OPERATION = 100904;


{ NurbsDisplay }
{      GLU_FILL }
const
	GLU_OUTLINE_POLYGON = 100240;
const
	GLU_OUTLINE_PATCH = 100241;

{ NurbsCallback }
const
	GLU_NURBS_ERROR = 100103;
const
	GLU_ERROR = 100103;
const
	GLU_NURBS_BEGIN = 100164;
const
	GLU_NURBS_BEGIN_EXT = 100164;
const
	GLU_NURBS_VERTEX = 100165;
const
	GLU_NURBS_VERTEX_EXT = 100165;
const
	GLU_NURBS_NORMAL = 100166;
const
	GLU_NURBS_NORMAL_EXT = 100166;
const
	GLU_NURBS_COLOR = 100167;
const
	GLU_NURBS_COLOR_EXT = 100167;
const
	GLU_NURBS_TEXTURE_COORD = 100168;
const
	GLU_NURBS_TEX_COORD_EXT = 100168;
const
	GLU_NURBS_END = 100169;
const
	GLU_NURBS_END_EXT = 100169;
const
	GLU_NURBS_BEGIN_DATA = 100170;
const
	GLU_NURBS_BEGIN_DATA_EXT = 100170;
const
	GLU_NURBS_VERTEX_DATA = 100171;
const
	GLU_NURBS_VERTEX_DATA_EXT = 100171;
const
	GLU_NURBS_NORMAL_DATA = 100172;
const
	GLU_NURBS_NORMAL_DATA_EXT = 100172;
const
	GLU_NURBS_COLOR_DATA = 100173;
const
	GLU_NURBS_COLOR_DATA_EXT = 100173;
const
	GLU_NURBS_TEXTURE_COORD_DATA = 100174;
const
	GLU_NURBS_TEX_COORD_DATA_EXT = 100174;
const
	GLU_NURBS_END_DATA = 100175;
const
	GLU_NURBS_END_DATA_EXT = 100175;

{ NurbsError }
const GLU_NURBS_ERROR1                     = 100251;   { spline order un-supported }
const GLU_NURBS_ERROR2                     = 100252;   { too few knots }
const GLU_NURBS_ERROR3                     = 100253;   { valid knot range is empty }
const GLU_NURBS_ERROR4                     = 100254;   { decreasing knot sequence }
const GLU_NURBS_ERROR5                     = 100255;   { knot multiplicity > spline order }
const GLU_NURBS_ERROR6                     = 100256;   { endcurve() must follow bgncurve() }
const GLU_NURBS_ERROR7                     = 100257;   { bgncurve() must precede endcurve() }
const GLU_NURBS_ERROR8                     = 100258;   { ctrlarray or knot vector is NULL }
const GLU_NURBS_ERROR9                     = 100259;   { can't draw pwlcurves }
const GLU_NURBS_ERROR10                    = 100260;   { missing gluNurbsCurve() }
const GLU_NURBS_ERROR11                    = 100261;   { missing gluNurbsSurface() }
const GLU_NURBS_ERROR12                    = 100262;   { endtrim() must precede endsurface() }
const GLU_NURBS_ERROR13                    = 100263;   { bgnsurface() must precede endsurface() }
const GLU_NURBS_ERROR14                    = 100264;   { curve of improper type passed as trim curve }
const GLU_NURBS_ERROR15                    = 100265;   { bgnsurface() must precede bgntrim() }
const GLU_NURBS_ERROR16                    = 100266;   { endtrim() must follow bgntrim() }
const GLU_NURBS_ERROR17                    = 100267;   { bgntrim() must precede endtrim()}
const GLU_NURBS_ERROR18                    = 100268;   { invalid or missing trim curve}
const GLU_NURBS_ERROR19                    = 100269;   { bgntrim() must precede pwlcurve() }
const GLU_NURBS_ERROR20                    = 100270;   { pwlcurve referenced twice}
const GLU_NURBS_ERROR21                    = 100271;   { pwlcurve and nurbscurve mixed }
const GLU_NURBS_ERROR22                    = 100272;   { improper usage of trim data type }
const GLU_NURBS_ERROR23                    = 100273;   { nurbscurve referenced twice }
const GLU_NURBS_ERROR24                    = 100274;   { nurbscurve and pwlcurve mixed }
const GLU_NURBS_ERROR25                    = 100275;   { nurbssurface referenced twice }
const GLU_NURBS_ERROR26                    = 100276;   { invalid property }
const GLU_NURBS_ERROR27                    = 100277;   { endsurface() must follow bgnsurface() }
const GLU_NURBS_ERROR28                    = 100278;   { intersecting or misoriented trim curves }
const GLU_NURBS_ERROR29                    = 100279;   { intersecting trim curves }
const GLU_NURBS_ERROR30                    = 100280;   { UNUSED }
const GLU_NURBS_ERROR31                    = 100281;   { unconnected trim curves }
const GLU_NURBS_ERROR32                    = 100282;   { unknown knot error }
const GLU_NURBS_ERROR33                    = 100283;   { negative vertex count encountered }
const GLU_NURBS_ERROR34                    = 100284;   { negative byte-stride }
const GLU_NURBS_ERROR35                    = 100285;   { unknown type descriptor }
const GLU_NURBS_ERROR36                    = 100286;   { null control point reference }
const GLU_NURBS_ERROR37                    = 100287;   { duplicate point on pwlcurve }

{ NurbsProperty }
const
	GLU_AUTO_LOAD_MATRIX = 100200;
const
	GLU_CULLING = 100201;
const
	GLU_SAMPLING_TOLERANCE = 100203;
const
	GLU_DISPLAY_MODE = 100204;
const
	GLU_PARAMETRIC_TOLERANCE = 100202;
const
	GLU_SAMPLING_METHOD = 100205;
const
	GLU_U_STEP = 100206;
const
	GLU_V_STEP = 100207;
const
	GLU_NURBS_MODE = 100160;
const
	GLU_NURBS_MODE_EXT = 100160;
const
	GLU_NURBS_TESSELLATOR = 100161;
const
	GLU_NURBS_TESSELLATOR_EXT = 100161;
const
	GLU_NURBS_RENDERER = 100162;
const
	GLU_NURBS_RENDERER_EXT = 100162;

{ NurbsSampling }
const
	GLU_OBJECT_PARAMETRIC_ERROR = 100208;
const
	GLU_OBJECT_PARAMETRIC_ERROR_EXT = 100208;
const
	GLU_OBJECT_PATH_LENGTH = 100209;
const
	GLU_OBJECT_PATH_LENGTH_EXT = 100209;
const
	GLU_PATH_LENGTH = 100215;
const
	GLU_PARAMETRIC_ERROR = 100216;
const
	GLU_DOMAIN_DISTANCE = 100217;

{ NurbsTrim }
const
	GLU_MAP1_TRIM_2 = 100210;
const
	GLU_MAP1_TRIM_3 = 100211;

{ QuadricDrawStyle } 
const
	GLU_POINT = 100010;
const
	GLU_LINE = 100011;
const
	GLU_FILL = 100012;
const
	GLU_SILHOUETTE = 100013;
  
{ QuadricCallback }
{      GLU_ERROR }

{ QuadricNormal }
const
	GLU_SMOOTH = 100000;
const
	GLU_FLAT = 100001;
const
	GLU_NONE = 100002;
 
{ QuadricOrientation }
const
	GLU_OUTSIDE = 100020;
const
	GLU_INSIDE = 100021;

{ TessCallback }
const
	GLU_TESS_BEGIN = 100100;
const
	GLU_BEGIN = 100100;
const
	GLU_TESS_VERTEX = 100101;
const
	GLU_VERTEX = 100101;
const
	GLU_TESS_END = 100102;
const
	GLU_END = 100102;
const
	GLU_TESS_ERROR = 100103;
const
	GLU_TESS_EDGE_FLAG = 100104;
const
	GLU_EDGE_FLAG = 100104;
const
	GLU_TESS_COMBINE = 100105;
const
	GLU_TESS_BEGIN_DATA = 100106;
const
	GLU_TESS_VERTEX_DATA = 100107;
const
	GLU_TESS_END_DATA = 100108;
const
	GLU_TESS_ERROR_DATA = 100109;
const
	GLU_TESS_EDGE_FLAG_DATA = 100110;
const
	GLU_TESS_COMBINE_DATA = 100111;

{ TessContour }
const
	GLU_CW = 100120;
const
	GLU_CCW = 100121;
const
	GLU_INTERIOR = 100122;
const
	GLU_EXTERIOR = 100123;
const
	GLU_UNKNOWN = 100124;

{ TessProperty }
const
	GLU_TESS_WINDING_RULE = 100140;
const
	GLU_TESS_BOUNDARY_ONLY = 100141;
const
	GLU_TESS_TOLERANCE = 100142;

{ TessError }
const
	GLU_TESS_ERROR1 = 100151;
const
	GLU_TESS_ERROR2 = 100152;
const
	GLU_TESS_ERROR3 = 100153;
const
	GLU_TESS_ERROR4 = 100154;
const
	GLU_TESS_ERROR5 = 100155;
const
	GLU_TESS_ERROR6 = 100156;
const
	GLU_TESS_ERROR7 = 100157;
const
	GLU_TESS_ERROR8 = 100158;
const
	GLU_TESS_MISSING_BEGIN_POLYGON = 100151;
const
	GLU_TESS_MISSING_BEGIN_CONTOUR = 100152;
const
	GLU_TESS_MISSING_END_POLYGON = 100153;
const
	GLU_TESS_MISSING_END_CONTOUR = 100154;
const
	GLU_TESS_COORD_TOO_LARGE = 100155;
const
	GLU_TESS_NEED_COMBINE_CALLBACK = 100156;

{ TessWinding }
const
	GLU_TESS_WINDING_ODD = 100130;
const
	GLU_TESS_WINDING_NONZERO = 100131;
const
	GLU_TESS_WINDING_POSITIVE = 100132;
const
	GLU_TESS_WINDING_NEGATIVE = 100133;
const
	GLU_TESS_WINDING_ABS_GEQ_TWO = 100134;

{***********************************************************}


{ Same as in opengl FPC package }
type
  GLUnurbs = record end;
  PGLUnurbs = ^GLUnurbs;
  
  GLUquadric = record end;
  PGLUquadric = ^GLUquadric;
  
  GLUtesselator = record end;
  PGLUtesselator = ^GLUtesselator;

  GLUnurbsObj = GLUnurbs;
  PGLUnurbsObj = PGLUnurbs;
  
  GLUquadricObj = GLUquadric;
  PGLUquadricObj = PGLUquadric;
  
  GLUtesselatorObj = GLUtesselator;
  PGLUtesselatorObj = PGLUtesselator;
  
  GLUtriangulatorObj = GLUtesselator;
  PGLUtriangulatorObj = PGLUtesselator;

const GLU_TESS_MAX_COORD = 1.0e150;

procedure gluBeginCurve( nurb: PGLUnurbs ); external name '_gluBeginCurve';
procedure gluBeginPolygon( tess: PGLUtesselator ); external name '_gluBeginPolygon';
procedure gluBeginSurface( nurb: PGLUnurbs ); external name '_gluBeginSurface';
procedure gluBeginTrim( nurb: PGLUnurbs ); external name '_gluBeginTrim';
function gluBuild1DMipmapLevels( target: GLenum; internalFormat: GLint; width: GLsizei; format: GLenum; typ: GLenum; level: GLint; base: GLint; max: GLint; data: {const} UnivPtr ): GLint; external name '_gluBuild1DMipmapLevels';
function gluBuild1DMipmaps( target: GLenum; internalFormat: GLint; width: GLsizei; format: GLenum; typ: GLenum; data: {const} UnivPtr ): GLint; external name '_gluBuild1DMipmaps';
function gluBuild2DMipmapLevels( target: GLenum; internalFormat: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; level: GLint; base: GLint; max: GLint; data: {const} UnivPtr ): GLint; external name '_gluBuild2DMipmapLevels';
function gluBuild2DMipmaps( target: GLenum; internalFormat: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; data: {const} UnivPtr ): GLint; external name '_gluBuild2DMipmaps';
function gluBuild3DMipmapLevels( target: GLenum; internalFormat: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; typ: GLenum; level: GLint; base: GLint; max: GLint; data: {const} UnivPtr ): GLint; external name '_gluBuild3DMipmapLevels';
function gluBuild3DMipmaps( target: GLenum; internalFormat: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; typ: GLenum; data: {const} UnivPtr ): GLint; external name '_gluBuild3DMipmaps';
function gluCheckExtension( const extName: PChar; const extString: PChar ): GLboolean; external name '_gluCheckExtension';
procedure gluCylinder( quad: PGLUquadric; base: GLdouble; top: GLdouble; height: GLdouble; slices: GLint; stacks: GLint ); external name '_gluCylinder';
procedure gluDeleteNurbsRenderer( nurb: PGLUnurbs ); external name '_gluDeleteNurbsRenderer';
procedure gluDeleteQuadric( quad: PGLUquadric ); external name '_gluDeleteQuadric';
procedure gluDeleteTess( tess: PGLUtesselator ); external name '_gluDeleteTess';
procedure gluDisk( quad: PGLUquadric; inner: GLdouble; outer: GLdouble; slices: GLint; loops: GLint ); external name '_gluDisk';
procedure gluEndCurve( nurb: PGLUnurbs ); external name '_gluEndCurve';
procedure gluEndPolygon( tess: PGLUtesselator ); external name '_gluEndPolygon';
procedure gluEndSurface( nurb: PGLUnurbs ); external name '_gluEndSurface';
procedure gluEndTrim( nurb: PGLUnurbs ); external name '_gluEndTrim';
function gluErrorString( error: GLenum ): PChar; external name '_gluErrorString';
procedure gluGetNurbsProperty( nurb: pGLUnurbs; property: GLenum; data: PGLfloat ); external name '_gluGetNurbsProperty';
function gluGetString( name: GLenum ): PChar; external name '_gluGetString';
procedure gluGetTessProperty( tess: PGLUtesselator; which: GLenum; data: PGLdouble ); external name '_gluGetTessProperty';
procedure gluLoadSamplingMatrices( nurb: PGLUnurbs; const model: PGLfloat; const perspective: PGLfloat; const view: PGLint ); external name '_gluLoadSamplingMatrices';
procedure gluLookAt( eyeX: GLdouble; eyeY: GLdouble; eyeZ: GLdouble; centerX: GLdouble; centerY: GLdouble; centerZ: GLdouble; upX: GLdouble; upY: GLdouble; upZ: GLdouble ); external name '_gluLookAt';
function gluNewNurbsRenderer: PGLUnurbs; external name '_gluNewNurbsRenderer';
function gluNewQuadric: PGLUquadric; external name '_gluNewQuadric';
function gluNewTess: PGLUtesselator; external name '_gluNewTess';
procedure gluNextContour( tess: PGLUtesselator; typ: GLenum ); external name '_gluNextContour';

type
	GLUCallBackFunc = procedure;
	
procedure gluNurbsCallback(nurb : PGLUnurbs; which : GLenum; CallBackFunc : GLUCallBackFunc); external name '_gluNurbsCallback';
procedure gluNurbsCallbackData( nurb: PGLUnurbs; userData: UnivPtr ); external name '_gluNurbsCallbackData';
procedure gluNurbsCallbackDataEXT( nurb: PGLUnurbs; userData: UnivPtr ); external name '_gluNurbsCallbackDataEXT';
procedure gluNurbsCurve( nurb: PGLUnurbs; knotCount: GLint; knots: PGLfloat; stride: GLint; control: PGLfloat; order: GLint; typ: GLenum ); external name '_gluNurbsCurve';
procedure gluNurbsProperty( nurb: PGLUnurbs; property: GLenum; value: GLfloat ); external name '_gluNurbsProperty';
procedure gluNurbsSurface( nurb: PGLUnurbs; sKnotCount: GLint; sKnots: PGLfloat; tKnotCount: GLint; tKnots: PGLfloat; sStride: GLint; tStride: GLint; control: PGLfloat; sOrder: GLint; tOrder: GLint; typ: GLenum ); external name '_gluNurbsSurface';
procedure gluOrtho2D( left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble ); external name '_gluOrtho2D';
procedure gluPartialDisk( quad: PGLUquadric; inner: GLdouble; outer: GLdouble; slices: GLint; loops: GLint; start: GLdouble; sweep: GLdouble ); external name '_gluPartialDisk';
procedure gluPerspective( fovy: GLdouble; aspect: GLdouble; zNear: GLdouble; zFar: GLdouble ); external name '_gluPerspective';
procedure gluPickMatrix( x: GLdouble; y: GLdouble; delX: GLdouble; delY: GLdouble; viewport: PGLint ); external name '_gluPickMatrix';
function gluProject( objX: GLdouble; objY: GLdouble; objZ: GLdouble; const model: PGLdouble; const proj: PGLdouble; const view: PGLint; winX: PGLdouble; winY: PGLdouble; winZ: PGLdouble ): GLint; external name '_gluProject';
procedure gluPwlCurve( nurb: PGLUnurbs; count: GLint; data: PGLfloat; stride: GLint; typ: GLenum ); external name '_gluPwlCurve';
procedure gluQuadricCallback(quad : PGLUquadric; which : GLenum; CallBackFunc : GLUCallBackFunc); external name '_gluQuadricCallback';
procedure gluQuadricDrawStyle( quad: PGLUquadric; draw: GLenum ); external name '_gluQuadricDrawStyle';
procedure gluQuadricNormals( quad: PGLUquadric; normal: GLenum ); external name '_gluQuadricNormals';
procedure gluQuadricOrientation( quad: PGLUquadric; orientation: GLenum ); external name '_gluQuadricOrientation';
procedure gluQuadricTexture( quad: PGLUquadric; texture: GLboolean ); external name '_gluQuadricTexture';
function gluScaleImage( format: GLenum; wIn: GLsizei; hIn: GLsizei; typeIn: GLenum; dataIn: {const} UnivPtr; wOut: GLsizei; hOut: GLsizei; typeOut: GLenum; dataOut: UnivPtr ): GLint; external name '_gluScaleImage';
procedure gluSphere( quad: PGLUquadric; radius: GLdouble; slices: GLint; stacks: GLint ); external name '_gluSphere';
procedure gluTessBeginContour( tess: PGLUtesselator ); external name '_gluTessBeginContour';
procedure gluTessBeginPolygon( tess: PGLUtesselator; data: UnivPtr ); external name '_gluTessBeginPolygon';
procedure gluTessCallback(tess : PGLUtesselator; which : GLenum; CallBackFunc : GLUCallBackFunc); external name '_gluTessCallback';
procedure gluTessEndContour( tess: PGLUtesselator ); external name '_gluTessEndContour';
procedure gluTessEndPolygon( tess: PGLUtesselator ); external name '_gluTessEndPolygon';
procedure gluTessNormal( tess: PGLUtesselator; valueX: GLdouble; valueY: GLdouble; valueZ: GLdouble ); external name '_gluTessNormal';
procedure gluTessProperty( tess: PGLUtesselator; which: GLenum; data: GLdouble ); external name '_gluTessProperty';
procedure gluTessVertex( tess: PGLUtesselator; location: PGLdouble; data: UnivPtr ); external name '_gluTessVertex';
function gluUnProject( winX: GLdouble; winY: GLdouble; winZ: GLdouble; const model: PGLdouble; const proj: PGLdouble; const view: PGLint; objX: PGLdouble; objY: PGLdouble; objZ: PGLdouble ): GLint; external name '_gluUnProject';
function gluUnProject4( winX: GLdouble; winY: GLdouble; winZ: GLdouble; clipW: GLdouble; const model: PGLdouble; const proj: PGLdouble; const view: PGLint; nearPlane: GLdouble; farPlane: GLdouble; objX: PGLdouble; objY: PGLdouble; objZ: PGLdouble; objW: PGLdouble ): GLint; external name '_gluUnProject4';

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
