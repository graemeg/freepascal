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

unit gliDispatch;
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
uses MacTypes, macgl, macglext, gliContexts;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
** GL function dispatch table type
}
type
	PGLIFunctionDispatch = ^GLIFunctionDispatch;
	GLIFunctionDispatch = record
		accum : procedure( ctx: GLIContext; op: GLenum; value: GLfloat );
		alpha_func : procedure( ctx: GLIContext; func: GLenum; ref: GLclampf );
		are_textures_resident : function( ctx: GLIContext; n: GLsizei; const textures: PGLuint; residences: PGLboolean ): GLboolean;
		array_element : procedure( ctx: GLIContext; i: GLint );
		begin_ : procedure( ctx: GLIContext; mode: GLenum );
		bind_texture : procedure( ctx: GLIContext; target: GLenum; texture: GLuint );
		bitmap : procedure( ctx: GLIContext; width: GLsizei; height: GLsizei; xorig: GLfloat; yorig: GLfloat; xmove: GLfloat; ymove: GLfloat; const bitmap: PGLubyte );
		blend_func : procedure( ctx: GLIContext; sfactor: GLenum; dfactor: GLenum );
		call_list : procedure( ctx: GLIContext; list: GLuint );
		call_lists : procedure( ctx: GLIContext; n: GLsizei; typ: GLenum; const lists: UnivPtr );
		clear : procedure( ctx: GLIContext; mask: GLbitfield );
		clear_accum : procedure( ctx: GLIContext; red: GLfloat; green: GLfloat; blue: GLfloat; alpha: GLfloat );
		clear_color : procedure( ctx: GLIContext; red: GLclampf; green: GLclampf; blue: GLclampf; alpha: GLclampf );
		clear_depth : procedure( ctx: GLIContext; depth: GLclampd );
		clear_index : procedure( ctx: GLIContext; c: GLfloat );
		clear_stencil : procedure( ctx: GLIContext; s: GLint );
		clip_plane : procedure( ctx: GLIContext; plane: GLenum; const equation: PGLdouble );
		color3b : procedure( ctx: GLIContext; red: GLbyte; green: GLbyte; blue: GLbyte );
		color3bv : procedure( ctx: GLIContext; const v: PGLbyte );
		color3d : procedure( ctx: GLIContext; red: GLdouble; green: GLdouble; blue: GLdouble );
		color3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		color3f : procedure( ctx: GLIContext; red: GLfloat; green: GLfloat; blue: GLfloat );
		color3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		color3i : procedure( ctx: GLIContext; red: GLint; green: GLint; blue: GLint );
		color3iv : procedure( ctx: GLIContext; const v: PGLint );
		color3s : procedure( ctx: GLIContext; red: GLshort; green: GLshort; blue: GLshort );
		color3sv : procedure( ctx: GLIContext; const v: PGLshort );
		color3ub : procedure( ctx: GLIContext; red: GLubyte; green: GLubyte; blue: GLubyte );
		color3ubv : procedure( ctx: GLIContext; const v: PGLubyte );
		color3ui : procedure( ctx: GLIContext; red: GLuint; green: GLuint; blue: GLuint );
		color3uiv : procedure( ctx: GLIContext; const v: PGLuint );
		color3us : procedure( ctx: GLIContext; red: GLushort; green: GLushort; blue: GLushort );
		color3usv : procedure( ctx: GLIContext; const v: PGLushort );
		color4b : procedure( ctx: GLIContext; red: GLbyte; green: GLbyte; blue: GLbyte; alpha: GLbyte );
		color4bv : procedure( ctx: GLIContext; const v: PGLbyte );
		color4d : procedure( ctx: GLIContext; red: GLdouble; green: GLdouble; blue: GLdouble; alpha: GLdouble );
		color4dv : procedure( ctx: GLIContext; const v: PGLdouble );
		color4f : procedure( ctx: GLIContext; red: GLfloat; green: GLfloat; blue: GLfloat; alpha: GLfloat );
		color4fv : procedure( ctx: GLIContext; const v: PGLfloat );
		color4i : procedure( ctx: GLIContext; red: GLint; green: GLint; blue: GLint; alpha: GLint );
		color4iv : procedure( ctx: GLIContext; const v: PGLint );
		color4s : procedure( ctx: GLIContext; red: GLshort; green: GLshort; blue: GLshort; alpha: GLshort );
		color4sv : procedure( ctx: GLIContext; const v: PGLshort );
		color4ub : procedure( ctx: GLIContext; red: GLubyte; green: GLubyte; blue: GLubyte; alpha: GLubyte );
		color4ubv : procedure( ctx: GLIContext; const v: PGLubyte );
		color4ui : procedure( ctx: GLIContext; red: GLuint; green: GLuint; blue: GLuint; alpha: GLuint );
		color4uiv : procedure( ctx: GLIContext; const v: PGLuint );
		color4us : procedure( ctx: GLIContext; red: GLushort; green: GLushort; blue: GLushort; alpha: GLushort );
		color4usv : procedure( ctx: GLIContext; const v: PGLushort );
		color_mask : procedure( ctx: GLIContext; red: GLboolean; green: GLboolean; blue: GLboolean; alpha: GLboolean );
		color_material : procedure( ctx: GLIContext; face: GLenum; mode: GLenum );
		color_pointer : procedure( ctx: GLIContext; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		copy_pixels : procedure( ctx: GLIContext; x: GLint; y: GLint; width: GLsizei; height: GLsizei; typ: GLenum );
		copy_tex_image1D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalFormat: GLenum; x: GLint; y: GLint; width: GLsizei; border: GLint );
		copy_tex_image2D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalFormat: GLenum; x: GLint; y: GLint; width: GLsizei; height: GLsizei; border: GLint );
		copy_tex_sub_image1D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; x: GLint; y: GLint; width: GLsizei );
		copy_tex_sub_image2D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
		cull_face : procedure( ctx: GLIContext; mode: GLenum );
		delete_lists : procedure( ctx: GLIContext; list: GLuint; range: GLsizei );
		delete_textures : procedure( ctx: GLIContext; n: GLsizei; const textures: PGLuint );
		depth_func : procedure( ctx: GLIContext; func: GLenum );
		depth_mask : procedure( ctx: GLIContext; flag: GLboolean );
		depth_range : procedure( ctx: GLIContext; zNear: GLclampd; zFar: GLclampd );
		disable : procedure( ctx: GLIContext; cap: GLenum );
		disable_client_state : procedure( ctx: GLIContext; arry: GLenum );
		draw_arrays : procedure( ctx: GLIContext; mode: GLenum; first: GLint; count: GLsizei );
		draw_buffer : procedure( ctx: GLIContext; mode: GLenum );
		draw_elements : procedure( ctx: GLIContext; mode: GLenum; count: GLsizei; typ: GLenum; const indices: UnivPtr );
		draw_pixels : procedure( ctx: GLIContext; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		edge_flag : procedure( ctx: GLIContext; flag: GLboolean );
		edge_flag_pointer : procedure( ctx: GLIContext; stride: GLsizei; const pointr: UnivPtr );
		edge_flagv : procedure( ctx: GLIContext; const flag: PGLboolean );
		enable : procedure( ctx: GLIContext; cap: GLenum );
		enable_client_state : procedure( ctx: GLIContext; arry: GLenum );
		end_ : procedure( ctx: GLIContext );
		end_list : procedure( ctx: GLIContext );
		eval_coord1d : procedure( ctx: GLIContext; u: GLdouble );
		eval_coord1dv : procedure( ctx: GLIContext; const u: PGLdouble );
		eval_coord1f : procedure( ctx: GLIContext; u: GLfloat );
		eval_coord1fv : procedure( ctx: GLIContext; const u: PGLfloat );
		eval_coord2d : procedure( ctx: GLIContext; u: GLdouble; v: GLdouble );
		eval_coord2dv : procedure( ctx: GLIContext; const u: PGLdouble );
		eval_coord2f : procedure( ctx: GLIContext; u: GLfloat; v: GLfloat );
		eval_coord2fv : procedure( ctx: GLIContext; const u: PGLfloat );
		eval_mesh1 : procedure( ctx: GLIContext; mode: GLenum; i1: GLint; i2: GLint );
		eval_mesh2 : procedure( ctx: GLIContext; mode: GLenum; i1: GLint; i2: GLint; j1: GLint; j2: GLint );
		eval_point1 : procedure( ctx: GLIContext; i: GLint );
		eval_point2 : procedure( ctx: GLIContext; i: GLint; j: GLint );
		feedback_buffer : procedure( ctx: GLIContext; size: GLsizei; typ: GLenum; buffer: PGLfloat );
		finish : procedure( ctx: GLIContext );
		flush : procedure( ctx: GLIContext );
		fogf : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		fogfv : procedure( ctx: GLIContext; pname: GLenum; const params: PGLfloat );
		fogi : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		fogiv : procedure( ctx: GLIContext; pname: GLenum; const params: PGLint );
		front_face : procedure( ctx: GLIContext; mode: GLenum );
		frustum : procedure( ctx: GLIContext; left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble; zNear: GLdouble; zFar: GLdouble );
		gen_lists : function( ctx: GLIContext; range: GLsizei ): GLuint;
		gen_textures : procedure( ctx: GLIContext; n: GLsizei; textures: PGLuint );
		get_booleanv : procedure( ctx: GLIContext; pname: GLenum; params: PGLboolean );
		get_clip_plane : procedure( ctx: GLIContext; plane: GLenum; equation: PGLdouble );
		get_doublev : procedure( ctx: GLIContext; pname: GLenum; params: PGLdouble );
		get_error : function( ctx: GLIContext ): GLenum;
		get_floatv : procedure( ctx: GLIContext; pname: GLenum; params: PGLfloat );
		get_integerv : procedure( ctx: GLIContext; pname: GLenum; params: PGLint );
		get_lightfv : procedure( ctx: GLIContext; light: GLenum; pname: GLenum; params: PGLfloat );
		get_lightiv : procedure( ctx: GLIContext; light: GLenum; pname: GLenum; params: PGLint );
		get_mapdv : procedure( ctx: GLIContext; target: GLenum; query: GLenum; v: PGLdouble );
		get_mapfv : procedure( ctx: GLIContext; target: GLenum; query: GLenum; v: PGLfloat );
		get_mapiv : procedure( ctx: GLIContext; target: GLenum; query: GLenum; v: PGLint );
		get_materialfv : procedure( ctx: GLIContext; face: GLenum; pname: GLenum; params: PGLfloat );
		get_materialiv : procedure( ctx: GLIContext; face: GLenum; pname: GLenum; params: PGLint );
		get_pixel_mapfv : procedure( ctx: GLIContext; map: GLenum; values: PGLfloat );
		get_pixel_mapuiv : procedure( ctx: GLIContext; map: GLenum; values: PGLuint );
		get_pixel_mapusv : procedure( ctx: GLIContext; map: GLenum; values: PGLushort );
		get_pointerv : procedure( ctx: GLIContext; pname: GLenum; params: UnivPtrPtr );
		get_polygon_stipple : procedure( ctx: GLIContext; mask: PGLubyte );
		get_string : function( ctx: GLIContext; name: GLenum ): PChar;
		get_tex_envfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLfloat );
		get_tex_enviv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		get_tex_gendv : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; params: PGLdouble );
		get_tex_genfv : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; params: PGLfloat );
		get_tex_geniv : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; params: PGLint );
		get_tex_image : procedure( ctx: GLIContext; target: GLenum; level: GLint; format: GLenum; typ: GLenum; pixels: UnivPtr );
		get_tex_level_parameterfv : procedure( ctx: GLIContext; target: GLenum; level: GLint; pname: GLenum; params: PGLfloat );
		get_tex_level_parameteriv : procedure( ctx: GLIContext; target: GLenum; level: GLint; pname: GLenum; params: PGLint );
		get_tex_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLfloat );
		get_tex_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		hint : procedure( ctx: GLIContext; target: GLenum; mode: GLenum );
		index_mask : procedure( ctx: GLIContext; mask: GLuint );
		index_pointer : procedure( ctx: GLIContext; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		indexd : procedure( ctx: GLIContext; c: GLdouble );
		indexdv : procedure( ctx: GLIContext; const c: PGLdouble );
		indexf : procedure( ctx: GLIContext; c: GLfloat );
		indexfv : procedure( ctx: GLIContext; const c: PGLfloat );
		indexi : procedure( ctx: GLIContext; c: GLint );
		indexiv : procedure( ctx: GLIContext; const c: PGLint );
		indexs : procedure( ctx: GLIContext; c: GLshort );
		indexsv : procedure( ctx: GLIContext; const c: PGLshort );
		indexub : procedure( ctx: GLIContext; c: GLubyte );
		indexubv : procedure( ctx: GLIContext; const c: PGLubyte );
		init_names : procedure( ctx: GLIContext );
		interleaved_arrays : procedure( ctx: GLIContext; format: GLenum; stride: GLsizei; const pointr: UnivPtr );
		is_enabled : function( ctx: GLIContext; cap: GLenum ): GLboolean;
		is_list : function( ctx: GLIContext; list: GLuint ): GLboolean;
		is_texture : function( ctx: GLIContext; texture: GLuint ): GLboolean;
		light_modelf : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		light_modelfv : procedure( ctx: GLIContext; pname: GLenum; const params: PGLfloat );
		light_modeli : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		light_modeliv : procedure( ctx: GLIContext; pname: GLenum; const params: PGLint );
		lightf : procedure( ctx: GLIContext; light: GLenum; pname: GLenum; param: GLfloat );
		lightfv : procedure( ctx: GLIContext; light: GLenum; pname: GLenum; const params: PGLfloat );
		lighti : procedure( ctx: GLIContext; light: GLenum; pname: GLenum; param: GLint );
		lightiv : procedure( ctx: GLIContext; light: GLenum; pname: GLenum; const params: PGLint );
		line_stipple : procedure( ctx: GLIContext; factor: GLint; pattern: GLushort );
		line_width : procedure( ctx: GLIContext; width: GLfloat );
		list_base : procedure( ctx: GLIContext; base: GLuint );
		load_identity : procedure( ctx: GLIContext );
		load_matrixd : procedure( ctx: GLIContext; const m: PGLdouble );
		load_matrixf : procedure( ctx: GLIContext; const m: PGLfloat );
		load_name : procedure( ctx: GLIContext; name: GLuint );
		logic_op : procedure( ctx: GLIContext; opcode: GLenum );
		map1d : procedure( ctx: GLIContext; target: GLenum; u1: GLdouble; u2: GLdouble; stride: GLint; order: GLint; const points: PGLdouble );
		map1f : procedure( ctx: GLIContext; target: GLenum; u1: GLfloat; u2: GLfloat; stride: GLint; order: GLint; const points: PGLfloat );
		map2d : procedure( ctx: GLIContext; target: GLenum; u1: GLdouble; u2: GLdouble; ustride: GLint; uorder: GLint; v1: GLdouble; v2: GLdouble; vstride: GLint; vorder: GLint; const points: PGLdouble );
		map2f : procedure( ctx: GLIContext; target: GLenum; u1: GLfloat; u2: GLfloat; ustride: GLint; uorder: GLint; v1: GLfloat; v2: GLfloat; vstride: GLint; vorder: GLint; const points: PGLfloat );
		map_grid1d : procedure( ctx: GLIContext; un: GLint; u1: GLdouble; u2: GLdouble );
		map_grid1f : procedure( ctx: GLIContext; un: GLint; u1: GLfloat; u2: GLfloat );
		map_grid2d : procedure( ctx: GLIContext; un: GLint; u1: GLdouble; u2: GLdouble; vn: GLint; v1: GLdouble; v2: GLdouble );
		map_grid2f : procedure( ctx: GLIContext; un: GLint; u1: GLfloat; u2: GLfloat; vn: GLint; v1: GLfloat; v2: GLfloat );
		materialf : procedure( ctx: GLIContext; face: GLenum; pname: GLenum; param: GLfloat );
		materialfv : procedure( ctx: GLIContext; face: GLenum; pname: GLenum; const params: PGLfloat );
		materiali : procedure( ctx: GLIContext; face: GLenum; pname: GLenum; param: GLint );
		materialiv : procedure( ctx: GLIContext; face: GLenum; pname: GLenum; const params: PGLint );
		matrix_mode : procedure( ctx: GLIContext; mode: GLenum );
		mult_matrixd : procedure( ctx: GLIContext; const m: PGLdouble );
		mult_matrixf : procedure( ctx: GLIContext; const m: PGLfloat );
		new_list : procedure( ctx: GLIContext; list: GLuint; mode: GLenum );
		normal3b : procedure( ctx: GLIContext; nx: GLbyte; ny: GLbyte; nz: GLbyte );
		normal3bv : procedure( ctx: GLIContext; const v: PGLbyte );
		normal3d : procedure( ctx: GLIContext; nx: GLdouble; ny: GLdouble; nz: GLdouble );
		normal3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		normal3f : procedure( ctx: GLIContext; nx: GLfloat; ny: GLfloat; nz: GLfloat );
		normal3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		normal3i : procedure( ctx: GLIContext; nx: GLint; ny: GLint; nz: GLint );
		normal3iv : procedure( ctx: GLIContext; const v: PGLint );
		normal3s : procedure( ctx: GLIContext; nx: GLshort; ny: GLshort; nz: GLshort );
		normal3sv : procedure( ctx: GLIContext; const v: PGLshort );
		normal_pointer : procedure( ctx: GLIContext; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		ortho : procedure( ctx: GLIContext; left: GLdouble; right: GLdouble; bottom: GLdouble; top: GLdouble; zNear: GLdouble; zFar: GLdouble );
		pass_through : procedure( ctx: GLIContext; token: GLfloat );
		pixel_mapfv : procedure( ctx: GLIContext; map: GLenum; mapsize: GLsizei; const values: PGLfloat );
		pixel_mapuiv : procedure( ctx: GLIContext; map: GLenum; mapsize: GLsizei; const values: PGLuint );
		pixel_mapusv : procedure( ctx: GLIContext; map: GLenum; mapsize: GLsizei; const values: PGLushort );
		pixel_storef : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		pixel_storei : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		pixel_transferf : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		pixel_transferi : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		pixel_zoom : procedure( ctx: GLIContext; xfactor: GLfloat; yfactor: GLfloat );
		point_size : procedure( ctx: GLIContext; size: GLfloat );
		polygon_mode : procedure( ctx: GLIContext; face: GLenum; mode: GLenum );
		polygon_offset : procedure( ctx: GLIContext; factor: GLfloat; units: GLfloat );
		polygon_stipple : procedure( ctx: GLIContext; const mask: PGLubyte );
		pop_attrib : procedure( ctx: GLIContext );
		pop_client_attrib : procedure( ctx: GLIContext );
		pop_matrix : procedure( ctx: GLIContext );
		pop_name : procedure( ctx: GLIContext );
		prioritize_textures : procedure( ctx: GLIContext; n: GLsizei; const textures: PGLuint; const priorities: PGLclampf );
		push_attrib : procedure( ctx: GLIContext; mask: GLbitfield );
		push_client_attrib : procedure( ctx: GLIContext; mask: GLbitfield );
		push_matrix : procedure( ctx: GLIContext );
		push_name : procedure( ctx: GLIContext; name: GLuint );
		raster_pos2d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble );
		raster_pos2dv : procedure( ctx: GLIContext; const v: PGLdouble );
		raster_pos2f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat );
		raster_pos2fv : procedure( ctx: GLIContext; const v: PGLfloat );
		raster_pos2i : procedure( ctx: GLIContext; x: GLint; y: GLint );
		raster_pos2iv : procedure( ctx: GLIContext; const v: PGLint );
		raster_pos2s : procedure( ctx: GLIContext; x: GLshort; y: GLshort );
		raster_pos2sv : procedure( ctx: GLIContext; const v: PGLshort );
		raster_pos3d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble );
		raster_pos3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		raster_pos3f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat );
		raster_pos3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		raster_pos3i : procedure( ctx: GLIContext; x: GLint; y: GLint; z: GLint );
		raster_pos3iv : procedure( ctx: GLIContext; const v: PGLint );
		raster_pos3s : procedure( ctx: GLIContext; x: GLshort; y: GLshort; z: GLshort );
		raster_pos3sv : procedure( ctx: GLIContext; const v: PGLshort );
		raster_pos4d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
		raster_pos4dv : procedure( ctx: GLIContext; const v: PGLdouble );
		raster_pos4f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
		raster_pos4fv : procedure( ctx: GLIContext; const v: PGLfloat );
		raster_pos4i : procedure( ctx: GLIContext; x: GLint; y: GLint; z: GLint; w: GLint );
		raster_pos4iv : procedure( ctx: GLIContext; const v: PGLint );
		raster_pos4s : procedure( ctx: GLIContext; x: GLshort; y: GLshort; z: GLshort; w: GLshort );
		raster_pos4sv : procedure( ctx: GLIContext; const v: PGLshort );
		read_buffer : procedure( ctx: GLIContext; mode: GLenum );
		read_pixels : procedure( ctx: GLIContext; x: GLint; y: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; pixels: UnivPtr );
		rectd : procedure( ctx: GLIContext; x1: GLdouble; y1: GLdouble; x2: GLdouble; y2: GLdouble );
		rectdv : procedure( ctx: GLIContext; const v1: PGLdouble; const v2: PGLdouble );
		rectf : procedure( ctx: GLIContext; x1: GLfloat; y1: GLfloat; x2: GLfloat; y2: GLfloat );
		rectfv : procedure( ctx: GLIContext; const v1: PGLfloat; const v2: PGLfloat );
		recti : procedure( ctx: GLIContext; x1: GLint; y1: GLint; x2: GLint; y2: GLint );
		rectiv : procedure( ctx: GLIContext; const v1: PGLint; const v2: PGLint );
		rects : procedure( ctx: GLIContext; x1: GLshort; y1: GLshort; x2: GLshort; y2: GLshort );
		rectsv : procedure( ctx: GLIContext; const v1: PGLshort; const v2: PGLshort );
		render_mode : function( ctx: GLIContext; mode: GLenum ): GLint;
		rotated : procedure( ctx: GLIContext; angle: GLdouble; x: GLdouble; y: GLdouble; z: GLdouble );
		rotatef : procedure( ctx: GLIContext; angle: GLfloat; x: GLfloat; y: GLfloat; z: GLfloat );
		scaled : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble );
		scalef : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat );
		scissor : procedure( ctx: GLIContext; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
		select_buffer : procedure( ctx: GLIContext; size: GLsizei; buffer: PGLuint );
		shade_model : procedure( ctx: GLIContext; mode: GLenum );
		stencil_func : procedure( ctx: GLIContext; func: GLenum; ref: GLint; mask: GLuint );
		stencil_mask : procedure( ctx: GLIContext; mask: GLuint );
		stencil_op : procedure( ctx: GLIContext; fail: GLenum; zfail: GLenum; zpass: GLenum );
		tex_coord1d : procedure( ctx: GLIContext; s: GLdouble );
		tex_coord1dv : procedure( ctx: GLIContext; const v: PGLdouble );
		tex_coord1f : procedure( ctx: GLIContext; s: GLfloat );
		tex_coord1fv : procedure( ctx: GLIContext; const v: PGLfloat );
		tex_coord1i : procedure( ctx: GLIContext; s: GLint );
		tex_coord1iv : procedure( ctx: GLIContext; const v: PGLint );
		tex_coord1s : procedure( ctx: GLIContext; s: GLshort );
		tex_coord1sv : procedure( ctx: GLIContext; const v: PGLshort );
		tex_coord2d : procedure( ctx: GLIContext; s: GLdouble; t: GLdouble );
		tex_coord2dv : procedure( ctx: GLIContext; const v: PGLdouble );
		tex_coord2f : procedure( ctx: GLIContext; s: GLfloat; t: GLfloat );
		tex_coord2fv : procedure( ctx: GLIContext; const v: PGLfloat );
		tex_coord2i : procedure( ctx: GLIContext; s: GLint; t: GLint );
		tex_coord2iv : procedure( ctx: GLIContext; const v: PGLint );
		tex_coord2s : procedure( ctx: GLIContext; s: GLshort; t: GLshort );
		tex_coord2sv : procedure( ctx: GLIContext; const v: PGLshort );
		tex_coord3d : procedure( ctx: GLIContext; s: GLdouble; t: GLdouble; r: GLdouble );
		tex_coord3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		tex_coord3f : procedure( ctx: GLIContext; s: GLfloat; t: GLfloat; r: GLfloat );
		tex_coord3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		tex_coord3i : procedure( ctx: GLIContext; s: GLint; t: GLint; r: GLint );
		tex_coord3iv : procedure( ctx: GLIContext; const v: PGLint );
		tex_coord3s : procedure( ctx: GLIContext; s: GLshort; t: GLshort; r: GLshort );
		tex_coord3sv : procedure( ctx: GLIContext; const v: PGLshort );
		tex_coord4d : procedure( ctx: GLIContext; s: GLdouble; t: GLdouble; r: GLdouble; q: GLdouble );
		tex_coord4dv : procedure( ctx: GLIContext; const v: PGLdouble );
		tex_coord4f : procedure( ctx: GLIContext; s: GLfloat; t: GLfloat; r: GLfloat; q: GLfloat );
		tex_coord4fv : procedure( ctx: GLIContext; const v: PGLfloat );
		tex_coord4i : procedure( ctx: GLIContext; s: GLint; t: GLint; r: GLint; q: GLint );
		tex_coord4iv : procedure( ctx: GLIContext; const v: PGLint );
		tex_coord4s : procedure( ctx: GLIContext; s: GLshort; t: GLshort; r: GLshort; q: GLshort );
		tex_coord4sv : procedure( ctx: GLIContext; const v: PGLshort );
		tex_coord_pointer : procedure( ctx: GLIContext; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		tex_envf : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; param: GLfloat );
		tex_envfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLfloat );
		tex_envi : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; param: GLint );
		tex_enviv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLint );
		tex_gend : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; param: GLdouble );
		tex_gendv : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; const params: PGLdouble );
		tex_genf : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; param: GLfloat );
		tex_genfv : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; const params: PGLfloat );
		tex_geni : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; param: GLint );
		tex_geniv : procedure( ctx: GLIContext; coord: GLenum; pname: GLenum; const params: PGLint );
		tex_image1D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		tex_image2D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		tex_parameterf : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; param: GLfloat );
		tex_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLfloat );
		tex_parameteri : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; param: GLint );
		tex_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLint );
		tex_sub_image1D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; width: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		tex_sub_image2D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		translated : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble );
		translatef : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat );
		vertex2d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble );
		vertex2dv : procedure( ctx: GLIContext; const v: PGLdouble );
		vertex2f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat );
		vertex2fv : procedure( ctx: GLIContext; const v: PGLfloat );
		vertex2i : procedure( ctx: GLIContext; x: GLint; y: GLint );
		vertex2iv : procedure( ctx: GLIContext; const v: PGLint );
		vertex2s : procedure( ctx: GLIContext; x: GLshort; y: GLshort );
		vertex2sv : procedure( ctx: GLIContext; const v: PGLshort );
		vertex3d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble );
		vertex3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		vertex3f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat );
		vertex3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		vertex3i : procedure( ctx: GLIContext; x: GLint; y: GLint; z: GLint );
		vertex3iv : procedure( ctx: GLIContext; const v: PGLint );
		vertex3s : procedure( ctx: GLIContext; x: GLshort; y: GLshort; z: GLshort );
		vertex3sv : procedure( ctx: GLIContext; const v: PGLshort );
		vertex4d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
		vertex4dv : procedure( ctx: GLIContext; const v: PGLdouble );
		vertex4f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
		vertex4fv : procedure( ctx: GLIContext; const v: PGLfloat );
		vertex4i : procedure( ctx: GLIContext; x: GLint; y: GLint; z: GLint; w: GLint );
		vertex4iv : procedure( ctx: GLIContext; const v: PGLint );
		vertex4s : procedure( ctx: GLIContext; x: GLshort; y: GLshort; z: GLshort; w: GLshort );
		vertex4sv : procedure( ctx: GLIContext; const v: PGLshort );
		vertex_pointer : procedure( ctx: GLIContext; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		viewport : procedure( ctx: GLIContext; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
		blend_func_separate : procedure( ctx: GLIContext; sfactorRGB: GLenum; dfactorRGB: GLenum; sfactorAlpha: GLenum; dfactorAlpha: GLenum );
		blend_color : procedure( ctx: GLIContext; red: GLclampf; green: GLclampf; blue: GLclampf; alpha: GLclampf );
		blend_equation : procedure( ctx: GLIContext; mode: GLenum );
		lock_arrays_EXT : procedure( ctx: GLIContext; first: GLint; count: GLsizei );
		unlock_arrays_EXT : procedure( ctx: GLIContext );
		client_active_texture : procedure( ctx: GLIContext; target: GLenum );
		active_texture : procedure( ctx: GLIContext; target: GLenum );
		multi_tex_coord1d : procedure( ctx: GLIContext; target: GLenum; s: GLdouble );
		multi_tex_coord1dv : procedure( ctx: GLIContext; target: GLenum; const v: PGLdouble );
		multi_tex_coord1f : procedure( ctx: GLIContext; target: GLenum; s: GLfloat );
		multi_tex_coord1fv : procedure( ctx: GLIContext; target: GLenum; const v: PGLfloat );
		multi_tex_coord1i : procedure( ctx: GLIContext; target: GLenum; s: GLint );
		multi_tex_coord1iv : procedure( ctx: GLIContext; target: GLenum; const v: PGLint );
		multi_tex_coord1s : procedure( ctx: GLIContext; target: GLenum; s: GLshort );
		multi_tex_coord1sv : procedure( ctx: GLIContext; target: GLenum; const v: PGLshort );
		multi_tex_coord2d : procedure( ctx: GLIContext; target: GLenum; s: GLdouble; t: GLdouble );
		multi_tex_coord2dv : procedure( ctx: GLIContext; target: GLenum; const v: PGLdouble );
		multi_tex_coord2f : procedure( ctx: GLIContext; target: GLenum; s: GLfloat; t: GLfloat );
		multi_tex_coord2fv : procedure( ctx: GLIContext; target: GLenum; const v: PGLfloat );
		multi_tex_coord2i : procedure( ctx: GLIContext; target: GLenum; s: GLint; t: GLint );
		multi_tex_coord2iv : procedure( ctx: GLIContext; target: GLenum; const v: PGLint );
		multi_tex_coord2s : procedure( ctx: GLIContext; target: GLenum; s: GLshort; t: GLshort );
		multi_tex_coord2sv : procedure( ctx: GLIContext; target: GLenum; const v: PGLshort );
		multi_tex_coord3d : procedure( ctx: GLIContext; target: GLenum; s: GLdouble; t: GLdouble; r: GLdouble );
		multi_tex_coord3dv : procedure( ctx: GLIContext; target: GLenum; const v: PGLdouble );
		multi_tex_coord3f : procedure( ctx: GLIContext; target: GLenum; s: GLfloat; t: GLfloat; r: GLfloat );
		multi_tex_coord3fv : procedure( ctx: GLIContext; target: GLenum; const v: PGLfloat );
		multi_tex_coord3i : procedure( ctx: GLIContext; target: GLenum; s: GLint; t: GLint; r: GLint );
		multi_tex_coord3iv : procedure( ctx: GLIContext; target: GLenum; const v: PGLint );
		multi_tex_coord3s : procedure( ctx: GLIContext; target: GLenum; s: GLshort; t: GLshort; r: GLshort );
		multi_tex_coord3sv : procedure( ctx: GLIContext; target: GLenum; const v: PGLshort );
		multi_tex_coord4d : procedure( ctx: GLIContext; target: GLenum; s: GLdouble; t: GLdouble; r: GLdouble; q: GLdouble );
		multi_tex_coord4dv : procedure( ctx: GLIContext; target: GLenum; const v: PGLdouble );
		multi_tex_coord4f : procedure( ctx: GLIContext; target: GLenum; s: GLfloat; t: GLfloat; r: GLfloat; q: GLfloat );
		multi_tex_coord4fv : procedure( ctx: GLIContext; target: GLenum; const v: PGLfloat );
		multi_tex_coord4i : procedure( ctx: GLIContext; target: GLenum; s: GLint; t: GLint; r: GLint; q: GLint );
		multi_tex_coord4iv : procedure( ctx: GLIContext; target: GLenum; const v: PGLint );
		multi_tex_coord4s : procedure( ctx: GLIContext; target: GLenum; s: GLshort; t: GLshort; r: GLshort; q: GLshort );
		multi_tex_coord4sv : procedure( ctx: GLIContext; target: GLenum; const v: PGLshort );
		load_transpose_matrixd : procedure( ctx: GLIContext; const m: PGLdouble );
		load_transpose_matrixf : procedure( ctx: GLIContext; const m: PGLfloat );
		mult_transpose_matrixd : procedure( ctx: GLIContext; const m: PGLdouble );
		mult_transpose_matrixf : procedure( ctx: GLIContext; const m: PGLfloat );
		compressed_tex_image3D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; depth: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr );
		compressed_tex_image2D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; height: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr );
		compressed_tex_image1D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalformat: GLenum; width: GLsizei; border: GLint; imageSize: GLsizei; const data: UnivPtr );
		compressed_tex_sub_image3D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr );
		compressed_tex_sub_image2D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; width: GLsizei; height: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr );
		compressed_tex_sub_image1D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; width: GLsizei; format: GLenum; imageSize: GLsizei; const data: UnivPtr );
		get_compressed_tex_image : procedure( ctx: GLIContext; target: GLenum; level: GLint; img: UnivPtr );
		secondary_color3b : procedure( ctx: GLIContext; red: GLbyte; green: GLbyte; blue: GLbyte );
		secondary_color3bv : procedure( ctx: GLIContext; const v: PGLbyte );
		secondary_color3d : procedure( ctx: GLIContext; red: GLdouble; green: GLdouble; blue: GLdouble );
		secondary_color3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		secondary_color3f : procedure( ctx: GLIContext; red: GLfloat; green: GLfloat; blue: GLfloat );
		secondary_color3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		secondary_color3i : procedure( ctx: GLIContext; red: GLint; green: GLint; blue: GLint );
		secondary_color3iv : procedure( ctx: GLIContext; const v: PGLint );
		secondary_color3s : procedure( ctx: GLIContext; red: GLshort; green: GLshort; blue: GLshort );
		secondary_color3sv : procedure( ctx: GLIContext; const v: PGLshort );
		secondary_color3ub : procedure( ctx: GLIContext; red: GLubyte; green: GLubyte; blue: GLubyte );
		secondary_color3ubv : procedure( ctx: GLIContext; const v: PGLubyte );
		secondary_color3ui : procedure( ctx: GLIContext; red: GLuint; green: GLuint; blue: GLuint );
		secondary_color3uiv : procedure( ctx: GLIContext; const v: PGLuint );
		secondary_color3us : procedure( ctx: GLIContext; red: GLushort; green: GLushort; blue: GLushort );
		secondary_color3usv : procedure( ctx: GLIContext; const v: PGLushort );
		secondary_color_pointer : procedure( ctx: GLIContext; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		vertex_array_range_EXT : procedure( ctx: GLIContext; count: GLsizei; const pointr: UnivPtr );
		flush_vertex_array_range_EXT : procedure( ctx: GLIContext; count: GLsizei; const pointr: UnivPtr );
		draw_range_elements : procedure( ctx: GLIContext; mode: GLenum; start: GLuint; finish: GLuint; count: GLsizei; typ: GLenum; const indices: UnivPtr );
		color_table : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; width: GLsizei; format: GLenum; typ: GLenum; const table: UnivPtr );
		color_table_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLfloat );
		color_table_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLint );
		copy_color_table : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei );
		get_color_table : procedure( ctx: GLIContext; target: GLenum; format: GLenum; typ: GLenum; table: UnivPtr );
		get_color_table_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLfloat );
		get_color_table_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		color_sub_table : procedure( ctx: GLIContext; target: GLenum; start: GLsizei; count: GLsizei; format: GLenum; typ: GLenum; const data: UnivPtr );
		copy_color_sub_table : procedure( ctx: GLIContext; target: GLenum; start: GLsizei; x: GLint; y: GLint; width: GLsizei );
		convolution_filter1D : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; width: GLsizei; format: GLenum; typ: GLenum; const image: UnivPtr );
		convolution_filter2D : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const image: UnivPtr );
		convolution_parameterf : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: GLfloat );
		convolution_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLfloat );
		convolution_parameteri : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: GLint );
		convolution_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; const params: PGLint );
		copy_convolution_filter1D : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei );
		copy_convolution_filter2D : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
		get_convolution_filter : procedure( ctx: GLIContext; target: GLenum; format: GLenum; typ: GLenum; image: UnivPtr );
		get_convolution_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLfloat );
		get_convolution_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		get_separable_filter : procedure( ctx: GLIContext; target: GLenum; format: GLenum; typ: GLenum; row: UnivPtr; column: UnivPtr; span: UnivPtr );
		separable_filter2D : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei; format: GLenum; typ: GLenum; const row: UnivPtr; const column: UnivPtr );
		get_histogram : procedure( ctx: GLIContext; target: GLenum; reset: GLboolean; format: GLenum; typ: GLenum; values: UnivPtr );
		get_histogram_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLfloat );
		get_histogram_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		get_minmax : procedure( ctx: GLIContext; target: GLenum; reset: GLboolean; format: GLenum; typ: GLenum; values: UnivPtr );
		get_minmax_parameterfv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLfloat );
		get_minmax_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		histogram : procedure( ctx: GLIContext; target: GLenum; width: GLsizei; internalformat: GLenum; sink: GLboolean );
		minmax : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; sink: GLboolean );
		reset_histogram : procedure( ctx: GLIContext; target: GLenum );
		reset_minmax : procedure( ctx: GLIContext; target: GLenum );
		tex_image3D : procedure( ctx: GLIContext; target: GLenum; level: GLint; internalFormat: GLenum; width: GLsizei; height: GLsizei; depth: GLsizei; border: GLint; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		tex_sub_image3D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; width: GLsizei; height: GLsizei; depth: GLsizei; format: GLenum; typ: GLenum; const pixels: UnivPtr );
		copy_tex_sub_image3D : procedure( ctx: GLIContext; target: GLenum; level: GLint; xoffset: GLint; yoffset: GLint; zoffset: GLint; x: GLint; y: GLint; width: GLsizei; height: GLsizei );
		combiner_parameterfv_NV : procedure( ctx: GLIContext; pname: GLenum; const params: PGLfloat );
		combiner_parameterf_NV : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		combiner_parameteriv_NV : procedure( ctx: GLIContext; pname: GLenum; const params: PGLint );
		combiner_parameteri_NV : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		combiner_input_NV : procedure( ctx: GLIContext; stage: GLenum; portion: GLenum; variable: GLenum; input: GLenum; mapping: GLenum; componentUsage: GLenum );
		combiner_output_NV : procedure( ctx: GLIContext; stage: GLenum; portion: GLenum; abOutput: GLenum; cdOutput: GLenum; sumOutput: GLenum; scale: GLenum; bias: GLenum; abDotProduct: GLboolean; cdDotProduct: GLboolean; muxSum: GLboolean );
		final_combiner_input_NV : procedure( ctx: GLIContext; variable: GLenum; input: GLenum; mapping: GLenum; componentUsage: GLenum );
		get_combiner_input_parameterfv_NV : procedure( ctx: GLIContext; stage: GLenum; portion: GLenum; variable: GLenum; pname: GLenum; params: PGLfloat );
		get_combiner_input_parameteriv_NV : procedure( ctx: GLIContext; stage: GLenum; portion: GLenum; variable: GLenum; pname: GLenum; params: PGLint );
		get_combiner_output_parameterfv_NV : procedure( ctx: GLIContext; stage: GLenum; portion: GLenum; pname: GLenum; params: PGLfloat );
		get_combiner_output_parameteriv_NV : procedure( ctx: GLIContext; stage: GLenum; portion: GLenum; pname: GLenum; params: PGLint );
		get_final_combiner_input_parameterfv_NV : procedure( ctx: GLIContext; variable: GLenum; pname: GLenum; params: PGLfloat );
		get_final_combiner_input_parameteriv_NV : procedure( ctx: GLIContext; variable: GLenum; pname: GLenum; params: PGLint );
		combiner_stage_parameterfv_NV : procedure( ctx: GLIContext; stage: GLenum; pname: GLenum; const params: PGLfloat );
		get_combiner_stage_parameterfv_NV : procedure( ctx: GLIContext; stage: GLenum; pname: GLenum; params: PGLfloat );
		texture_range_APPLE : procedure( ctx: GLIContext; target: GLenum; length: GLsizei; const pointr: UnivPtr );
		get_tex_parameter_pointerv_APPLE : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: UnivPtrPtr );
		blend_equation_separate_EXT : procedure( ctx: GLIContext; equationRGB: GLenum; equationAlpha: GLenum );
		sample_coverage : procedure( ctx: GLIContext; value: GLclampf; invert: GLboolean );
		sample_pass : procedure( ctx: GLIContext; mode: GLenum );
		pn_trianglesi_ATI : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		pn_trianglesf_ATI : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		gen_fences_APPLE : procedure( ctx: GLIContext; n: GLsizei; fences: PGLuint );
		delete_fences_APPLE : procedure( ctx: GLIContext; n: GLsizei; const fences: PGLuint );
		set_fence_APPLE : procedure( ctx: GLIContext; fence: GLuint );
		is_fence_APPLE : function( ctx: GLIContext; fence: GLuint ): GLboolean;
		test_fence_APPLE : function( ctx: GLIContext; fence: GLuint ): GLboolean;
		finish_fence_APPLE : procedure( ctx: GLIContext; fence: GLuint );
		test_object_APPLE : function( ctx: GLIContext; objct: GLenum; name: GLuint ): GLboolean;
		finish_object_APPLE : procedure( ctx: GLIContext; objct: GLenum; name: GLuint );
		bind_program_ARB : procedure( ctx: GLIContext; target: GLenum; program_: GLuint );
		delete_programs_ARB : procedure( ctx: GLIContext; n: GLsizei; const programs: PGLuint );
		gen_programs_ARB : procedure( ctx: GLIContext; n: GLsizei; programs: PGLuint );
		is_program_ARB : function( ctx: GLIContext; program_: GLuint ): GLboolean;
		vertex_attrib1s_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLshort );
		vertex_attrib1f_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLfloat );
		vertex_attrib1d_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLdouble );
		vertex_attrib2s_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLshort; y: GLshort );
		vertex_attrib2f_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLfloat; y: GLfloat );
		vertex_attrib2d_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLdouble; y: GLdouble );
		vertex_attrib3s_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLshort; y: GLshort; z: GLshort );
		vertex_attrib3f_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat );
		vertex_attrib3d_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble );
		vertex_attrib4s_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort );
		vertex_attrib4f_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
		vertex_attrib4d_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
		vertex_attrib4Nub_ARB : procedure( ctx: GLIContext; index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte );
		vertex_attrib1sv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLshort );
		vertex_attrib1fv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLfloat );
		vertex_attrib1dv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLdouble );
		vertex_attrib2sv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLshort );
		vertex_attrib2fv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLfloat );
		vertex_attrib2dv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLdouble );
		vertex_attrib3sv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLshort );
		vertex_attrib3fv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLfloat );
		vertex_attrib3dv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLdouble );
		vertex_attrib4bv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLbyte );
		vertex_attrib4sv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLshort );
		vertex_attrib4iv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLint );
		vertex_attrib4ubv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLubyte );
		vertex_attrib4usv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLushort );
		vertex_attrib4uiv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLuint );
		vertex_attrib4fv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLfloat );
		vertex_attrib4dv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLdouble );
		vertex_attrib4Nbv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLbyte );
		vertex_attrib4Nsv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLshort );
		vertex_attrib4Niv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLint );
		vertex_attrib4Nubv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLubyte );
		vertex_attrib4Nusv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLushort );
		vertex_attrib4Nuiv_ARB : procedure( ctx: GLIContext; index: GLuint; const v: PGLuint );
		vertex_attrib_pointer_ARB : procedure( ctx: GLIContext; index: GLuint; size: GLint; typ: GLenum; normalized: GLboolean; stride: GLsizei; const pointr: UnivPtr );
		enable_vertex_attrib_array_ARB : procedure( ctx: GLIContext; index: GLuint );
		disable_vertex_attrib_array_ARB : procedure( ctx: GLIContext; index: GLuint );
		get_vertex_attribdv_ARB : procedure( ctx: GLIContext; index: GLuint; pname: GLenum; params: PGLdouble );
		get_vertex_attribfv_ARB : procedure( ctx: GLIContext; index: GLuint; pname: GLenum; params: PGLfloat );
		get_vertex_attribiv_ARB : procedure( ctx: GLIContext; index: GLuint; pname: GLenum; params: PGLint );
		get_vertex_attrib_pointerv_ARB : procedure( ctx: GLIContext; index: GLuint; pname: GLenum; pointr: UnivPtrPtr );
		program_env_parameter4d_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
		program_env_parameter4dv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; const params: PGLdouble );
		program_env_parameter4f_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
		program_env_parameter4fv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; const params: PGLfloat );
		program_local_parameter4d_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
		program_local_parameter4dv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; const params: PGLdouble );
		program_local_parameter4f_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
		program_local_parameter4fv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; const params: PGLfloat );
		get_program_env_parameterdv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; params: PGLdouble );
		get_program_env_parameterfv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; params: PGLfloat );
		get_program_local_parameterdv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; params: PGLdouble );
		get_program_local_parameterfv_ARB : procedure( ctx: GLIContext; target: GLenum; index: GLuint; params: PGLfloat );
		program_string_ARB : procedure( ctx: GLIContext; target: GLenum; format: GLenum; len: GLsizei; const strng: UnivPtr );
		get_program_string_ARB : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; strng: UnivPtr );
		get_programiv_ARB : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		enable_vertex_attrib_ARB : procedure( ctx: GLIContext; index: GLuint; pname: GLenum );
		disable_vertex_attrib_ARB : procedure( ctx: GLIContext; index: GLuint; pname: GLenum );
		is_vertex_attrib_enabled_ARB : function( ctx: GLIContext; index: GLuint; pname: GLenum ): GLboolean;
		map_vertex_attrib1d_ARB : procedure( ctx: GLIContext; index: GLuint; size: GLuint; u1: GLdouble; u2: GLdouble; stride: GLint; order: GLint; const points: PGLdouble );
		map_vertex_attrib1f_ARB : procedure( ctx: GLIContext; index: GLuint; size: GLuint; u1: GLfloat; u2: GLfloat; stride: GLint; order: GLint; const points: PGLfloat );
		map_vertex_attrib2d_ARB : procedure( ctx: GLIContext; index: GLuint; size: GLuint; u1: GLdouble; u2: GLdouble; ustride: GLint; uorder: GLint; v1: GLdouble; v2: GLdouble; vstride: GLint; vorder: GLint; const points: PGLdouble );
		map_vertex_attrib2f_ARB : procedure( ctx: GLIContext; index: GLuint; size: GLuint; u1: GLfloat; u2: GLfloat; ustride: GLint; uorder: GLint; v1: GLfloat; v2: GLfloat; vstride: GLint; vorder: GLint; const points: PGLfloat );
		point_parameterf : procedure( ctx: GLIContext; pname: GLenum; param: GLfloat );
		point_parameterfv : procedure( ctx: GLIContext; pname: GLenum; const params: PGLfloat );
		point_parameteri : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		point_parameteriv : procedure( ctx: GLIContext; pname: GLenum; const params: PGLint );
		fog_coordf : procedure( ctx: GLIContext; coord: GLfloat );
		fog_coordfv : procedure( ctx: GLIContext; const coord: PGLfloat );
		fog_coordd : procedure( ctx: GLIContext; coord: GLdouble );
		fog_coorddv : procedure( ctx: GLIContext; const coord: PGLdouble );
		fog_coord_pointer : procedure( ctx: GLIContext; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		vertex_array_parameteri_EXT : procedure( ctx: GLIContext; pname: GLenum; param: GLint );
		bind_vertex_array_EXT : procedure( ctx: GLIContext; id: GLuint );
		delete_vertex_arrays_EXT : procedure( ctx: GLIContext; n: GLsizei; const ids: PGLuint );
		gen_vertex_arrays_EXT : procedure( ctx: GLIContext; n: GLsizei; ids: PGLuint );
		is_vertex_array_EXT : function( ctx: GLIContext; id: GLuint ): GLboolean;
		element_pointer_APPLE : procedure( ctx: GLIContext; typ: GLenum; const pointr: UnivPtr );
		draw_element_array_APPLE : procedure( ctx: GLIContext; mode: GLenum; first: GLint; count: GLsizei );
		draw_range_element_array_APPLE : procedure( ctx: GLIContext; mode: GLenum; start: GLuint; finish: GLuint; first: GLint; count: GLsizei );
		weightbv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLbyte );
		weightsv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLshort );
		weightiv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLint );
		weightfv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLfloat );
		weightdv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLdouble );
		weightubv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLubyte );
		weightusv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLushort );
		weightuiv_ARB : procedure( ctx: GLIContext; size: GLint; const weights: PGLuint );
		weight_pointer_ARB : procedure( ctx: GLIContext; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		vertex_blend_ARB : procedure( ctx: GLIContext; count: GLint );
		multi_draw_arrays : procedure( ctx: GLIContext; mode: GLenum; const first: PGLint; const count: PGLsizei; primcount: GLsizei );
		multi_draw_elements : procedure( ctx: GLIContext; mode: GLenum; const count: PGLsizei; typ: GLenum; {const} indices: UnivPtrPtr; primcount: GLsizei );
		window_pos2d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble );
		window_pos2dv : procedure( ctx: GLIContext; const v: PGLdouble );
		window_pos2f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat );
		window_pos2fv : procedure( ctx: GLIContext; const v: PGLfloat );
		window_pos2i : procedure( ctx: GLIContext; x: GLint; y: GLint );
		window_pos2iv : procedure( ctx: GLIContext; const v: PGLint );
		window_pos2s : procedure( ctx: GLIContext; x: GLshort; y: GLshort );
		window_pos2sv : procedure( ctx: GLIContext; const v: PGLshort );
		window_pos3d : procedure( ctx: GLIContext; x: GLdouble; y: GLdouble; z: GLdouble );
		window_pos3dv : procedure( ctx: GLIContext; const v: PGLdouble );
		window_pos3f : procedure( ctx: GLIContext; x: GLfloat; y: GLfloat; z: GLfloat );
		window_pos3fv : procedure( ctx: GLIContext; const v: PGLfloat );
		window_pos3i : procedure( ctx: GLIContext; x: GLint; y: GLint; z: GLint );
		window_pos3iv : procedure( ctx: GLIContext; const v: PGLint );
		window_pos3s : procedure( ctx: GLIContext; x: GLshort; y: GLshort; z: GLshort );
		window_pos3sv : procedure( ctx: GLIContext; const v: PGLshort );
		active_stencil_face_EXT : procedure( ctx: GLIContext; face: GLenum );
		stencil_op_separate_ATI : procedure( ctx: GLIContext; face: GLenum; sfail: GLenum; dpfail: GLenum; dppass: GLenum );
		stencil_func_separate_ATI : procedure( ctx: GLIContext; frontfunc: GLenum; backfunc: GLenum; ref: GLint; mask: GLuint );
		flush_render_APPLE : procedure( ctx: GLIContext );
		finish_render_APPLE : procedure( ctx: GLIContext );
		swap_APPLE : procedure( ctx: GLIContext );
		delete_object_ARB : procedure( ctx: GLIContext; obj: GLhandleARB );
		get_handle_ARB : function( ctx: GLIContext; pname: GLenum ): GLhandleARB;
		detach_object_ARB : procedure( ctx: GLIContext; containerObj: GLhandleARB; attachedObj: GLhandleARB );
		create_shader_object_ARB : function( ctx: GLIContext; shaderType: GLenum ): GLhandleARB;
		shader_source_ARB : procedure( ctx: GLIContext; shaderObj: GLhandleARB; count: GLsizei; {const} strng: PPChar; const length: PGLint );
		compile_shader_ARB : procedure( ctx: GLIContext; shaderObj: GLhandleARB );
		create_program_object_ARB : function( ctx: GLIContext ): GLhandleARB;
		attach_object_ARB : procedure( ctx: GLIContext; containerObj: GLhandleARB; obj: GLhandleARB );
		link_program_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB );
		use_program_object_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB );
		validate_program_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB );
		uniform1f_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLfloat );
		uniform2f_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLfloat; v1: GLfloat );
		uniform3f_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat );
		uniform4f_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat; v3: GLfloat );
		uniform1i_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLint );
		uniform2i_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLint; v1: GLint );
		uniform3i_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLint; v1: GLint; v2: GLint );
		uniform4i_ARB : procedure( ctx: GLIContext; location: GLint; v0: GLint; v1: GLint; v2: GLint; v3: GLint );
		uniform1fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLfloat );
		uniform2fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLfloat );
		uniform3fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLfloat );
		uniform4fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLfloat );
		uniform1iv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLint );
		uniform2iv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLint );
		uniform3iv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLint );
		uniform4iv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLint );
		uniform_matrix2fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix3fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix4fv_ARB : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		get_object_parameterfv_ARB : procedure( ctx: GLIContext; obj: GLhandleARB; pname: GLenum; params: PGLfloat );
		get_object_parameteriv_ARB : procedure( ctx: GLIContext; obj: GLhandleARB; pname: GLenum; params: PGLint );
		get_info_log_ARB : procedure( ctx: GLIContext; obj: GLhandleARB; maxLength: GLsizei; length: PGLsizei; infoLog: PGLcharARB );
		get_attached_objects_ARB : procedure( ctx: GLIContext; containerObj: GLhandleARB; maxCount: GLsizei; count: PGLsizei; obj: PGLhandleARB );
		get_uniform_location_ARB : function( ctx: GLIContext; programObj: GLhandleARB; const name: PGLcharARB ): GLint;
		get_active_uniform_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB; index: GLuint; maxLength: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PGLcharARB );
		get_uniformfv_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB; location: GLint; params: PGLfloat );
		get_uniformiv_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB; location: GLint; params: PGLint );
		get_shader_source_ARB : procedure( ctx: GLIContext; obj: GLhandleARB; maxLength: GLsizei; length: PGLsizei; source: PGLcharARB );
		bind_attrib_location_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB; index: GLuint; const name: PGLcharARB );
		get_active_attrib_ARB : procedure( ctx: GLIContext; programObj: GLhandleARB; index: GLuint; maxLength: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PGLcharARB );
		get_attrib_location_ARB : function( ctx: GLIContext; programObj: GLhandleARB; const name: PGLcharARB ): GLint;
		clamp_color_ARB : procedure( ctx: GLIContext; target: GLenum; clamp: GLenum );
		gen_queries : procedure( ctx: GLIContext; n: GLsizei; ids: PGLuint );
		delete_queries : procedure( ctx: GLIContext; n: GLsizei; const ids: PGLuint );
		is_query : function( ctx: GLIContext; id: GLuint ): GLboolean;
		begin_query : procedure( ctx: GLIContext; target: GLenum; id: GLuint );
		end_query : procedure( ctx: GLIContext; target: GLenum );
		get_queryiv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		get_query_objectiv : procedure( ctx: GLIContext; id: GLuint; pname: GLenum; params: PGLint );
		get_query_objectuiv : procedure( ctx: GLIContext; id: GLuint; pname: GLenum; params: PGLuint );
		bind_buffer : procedure( ctx: GLIContext; target: GLenum; buffer: GLuint );
		delete_buffers : procedure( ctx: GLIContext; n: GLsizei; const buffers: PGLuint );
		gen_buffers : procedure( ctx: GLIContext; n: GLsizei; buffers: PGLuint );
		is_buffer : function( ctx: GLIContext; buffer: GLuint ): GLboolean;
		buffer_data : procedure( ctx: GLIContext; target: GLenum; size: GLsizeiptrARB; const data: UnivPtr; usage: GLenum );
		buffer_sub_data : procedure( ctx: GLIContext; target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB; const data: UnivPtr );
		get_buffer_sub_data : procedure( ctx: GLIContext; target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB; data: UnivPtr );
		map_buffer : function( ctx: GLIContext; target: GLenum; access: GLenum ): UnivPtr;
		unmap_buffer : function( ctx: GLIContext; target: GLenum ): GLboolean;
		get_buffer_parameteriv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		get_buffer_pointerv : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: UnivPtrPtr );
		depth_bounds_EXT : procedure( ctx: GLIContext; zmin: GLclampd; zmax: GLclampd );
		draw_buffers_ARB : procedure( ctx: GLIContext; n: GLsizei; const bufs: PGLenum );
	
		is_shader : function( ctx: GLIContext; shader: GLuint ): GLboolean;
		is_program : function( ctx: GLIContext; program_: GLuint ): GLboolean;
		get_shaderiv : procedure( ctx: GLIContext; shader: GLuint; pname: GLenum; params: PGLint );
		get_programiv : procedure( ctx: GLIContext; program_: GLuint; pname: GLenum; params: PGLint );
		get_shader_info_log : procedure( ctx: GLIContext; shader: GLuint; bufSize: GLsizei; length: PGLsizei; infoLog: PChar );
		get_program_info_log : procedure( ctx: GLIContext; program_: GLuint; bufSize: GLsizei; length: PGLsizei; infoLog: PChar );
	
		stencil_func_separate : procedure( ctx: GLIContext; face: GLenum; func: GLenum; ref: GLint; mask: GLuint );
		stencil_mask_separate : procedure( ctx: GLIContext; face: GLenum; mask: GLuint );
		
		multi_draw_element_array_APPLE : procedure( ctx: GLIContext; mode: GLenum; const first: PGLint; const count: PGLsizei; primcount: GLsizei );
		multi_draw_range_element_array_APPLE : procedure( ctx: GLIContext; mode: GLenum; start: GLuint; finish: GLuint; const first: PGLint; const count: PGLsizei; primcount: GLsizei );
	
		{ frame buffer object }
		is_renderbuffer_EXT : function( ctx: GLIContext; renderbuffer: GLuint ): GLboolean;
		bind_renderbuffer_EXT : procedure( ctx: GLIContext; target: GLenum; renderbuffer: GLuint );
		delete_renderbuffers_EXT : procedure( ctx: GLIContext; n: GLsizei; const renderbuffers: PGLuint );
		gen_renderbuffers_EXT : procedure( ctx: GLIContext; n: GLsizei; renderbuffers: PGLuint );
		renderbuffer_storage_EXT : procedure( ctx: GLIContext; target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei );
		get_renderbuffer_parameteriv_EXT : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		is_framebuffer_EXT : function( ctx: GLIContext; framebuffer: GLuint ): GLboolean;
		bind_framebuffer_EXT : procedure( ctx: GLIContext; target: GLenum; framebuffer: GLuint );
		delete_framebuffers_EXT : procedure( ctx: GLIContext; n: GLsizei; const framebuffers: PGLuint );
		gen_framebuffers_EXT : procedure( ctx: GLIContext; n: GLsizei; framebuffers: PGLuint );
		check_framebuffer_status_EXT : function( ctx: GLIContext; target: GLenum ): GLenum;
		framebuffer_texture1D_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint );
		framebuffer_texture2D_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint );
		framebuffer_texture3D_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint; zoffset: GLint );
		framebuffer_renderbuffer_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; renderbuffertarget: GLenum; renderbuffer: GLuint );
		get_framebuffer_attachment_parameteriv_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; pname: GLenum; params: PGLint );
		generate_mipmap_EXT : procedure( ctx: GLIContext; target: GLenum );
	
		buffer_parameteri_APPLE : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; param: GLint );
		flush_mapped_buffer_range_APPLE : procedure( ctx: GLIContext; target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB );
		
		program_env_parameters4fv_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint; count: GLsizei; const params: PGLfloat );
		program_local_parameters4fv_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint; count: GLsizei; const params: PGLfloat );
	
		object_purgeable_APPLE : function( ctx: GLIContext; objectType: GLenum; name: GLuint; option: GLenum ): GLenum;
		object_unpurgeable_APPLE : function( ctx: GLIContext; objectType: GLenum; name: GLuint; option: GLenum ): GLenum;
		get_object_parameteriv_APPLE : procedure( ctx: GLIContext; objectType: GLenum; name: GLuint; pname: GLenum; params: PGLint );
	
		{ geometry shader4 }
		program_parameteri_EXT : procedure( ctx: GLIContext; program_name: GLuint; pname: GLenum; value: GLint );
		framebuffer_texture_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; texture: GLuint; level: GLint );
		framebuffer_texture_layer_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; texture: GLuint; level: GLint; layer: GLint );
		framebuffer_texture_face_EXT : procedure( ctx: GLIContext; target: GLenum; attachment: GLenum; texture: GLuint; level: GLint; face: GLenum );
		
		{ transform feedback }
		bind_buffer_range_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint; buffer: GLuint; offset: GLintptr; size: GLsizeiptr );
		bind_buffer_offset_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint; buffer: GLuint; offset: GLintptr );
		bind_buffer_base_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint; buffer: GLuint );
		begin_transform_feedback_EXT : procedure( ctx: GLIContext; primitiveMode: GLenum );
		end_transform_feedback_EXT : procedure( ctx: GLIContext );
		transform_feedback_varyings_EXT : procedure( ctx: GLIContext; program_: GLuint; count: GLsizei; {const} varyings: PPChar; bufferMode: GLenum );
		get_transform_feedback_varying_EXT : procedure( ctx: GLIContext; program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLsizei; typ: PGLenum; name: PChar );
		get_integer_indexedv_EXT : procedure( ctx: GLIContext; param: GLenum; index: GLuint; values: PGLint ); 
		get_boolean_indexedv_EXT : procedure( ctx: GLIContext; param: GLenum; index: GLuint; values: PGLboolean );
	
		{ bindable uniform }
		uniform_buffer_EXT : procedure( ctx: GLIContext; program_: GLuint; location: GLint; buffer: GLuint );
		get_uniform_buffer_size_EXT : function( ctx: GLIContext; program_: GLuint; location: GLint ): GLint;
		get_uniform_buffer_offset_EXT : function( ctx: GLIContext; program_: GLuint; location: GLint ): GLintptr;
	
		{ texture integer }
		clear_colorIi_EXT : procedure( ctx: GLIContext; r: GLint; g: GLint; b: GLint; a: GLint );
		clear_colorIui_EXT : procedure( ctx: GLIContext; r: GLuint; g: GLuint; b: GLuint; a: GLuint );
		tex_parameterIiv_EXT : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		tex_parameterIuiv_EXT : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLuint );
		get_tex_parameterIiv_EXT : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLint );
		get_tex_parameterIuiv_EXT : procedure( ctx: GLIContext; target: GLenum; pname: GLenum; params: PGLuint );
	
		{ gpu_shader4 }
		vertex_attribI1i_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLint );
		vertex_attribI2i_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLint; y: GLint );
		vertex_attribI3i_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLint; y: GLint; z: GLint );
		vertex_attribI4i_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLint; y: GLint; z: GLint; w: GLint );
		vertex_attribI1ui_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLuint );
		vertex_attribI2ui_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLuint; y: GLuint );
		vertex_attribI3ui_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLuint; y: GLuint; z: GLuint );
		vertex_attribI4ui_EXT : procedure( ctx: GLIContext; index: GLuint; x: GLuint; y: GLuint; z: GLuint; w: GLuint );
		vertex_attribI1iv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLint );
		vertex_attribI2iv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLint );
		vertex_attribI3iv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLint );
		vertex_attribI4iv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLint );
		vertex_attribI1uiv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLuint );
		vertex_attribI2uiv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLuint );
		vertex_attribI3uiv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLuint );
		vertex_attribI4uiv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLuint );
		vertex_attribI4bv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLbyte );
		vertex_attribI4sv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLshort );
		vertex_attribI4ubv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLubyte );
		vertex_attribI4usv_EXT : procedure( ctx: GLIContext; index: GLuint; const v: PGLushort );
		vertex_attribI_pointer_EXT : procedure( ctx: GLIContext; index: GLuint; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
		get_vertex_attribIiv_EXT : procedure( ctx: GLIContext; index: GLuint; pname: GLenum; params: PGLint );
		get_vertex_attribIuiv_EXT : procedure( ctx: GLIContext; index: GLuint; pname: GLenum; params: PGLuint );
		uniform1ui_EXT : procedure( ctx: GLIContext; location: GLint; v0: GLuint );
		uniform2ui_EXT : procedure( ctx: GLIContext; location: GLint; v0: GLuint; v1: GLuint );
		uniform3ui_EXT : procedure( ctx: GLIContext; location: GLint; v0: GLuint; v1: GLuint; v2: GLuint );
		uniform4ui_EXT : procedure( ctx: GLIContext; location: GLint; v0: GLuint; v1: GLuint; v2: GLuint; v3: GLuint );
		uniform1uiv_EXT : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLuint );
		uniform2uiv_EXT : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLuint );
		uniform3uiv_EXT : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLuint );
		uniform4uiv_EXT : procedure( ctx: GLIContext; location: GLint; count: GLsizei; const value: PGLuint );
		get_uniformuiv_EXT : procedure( ctx: GLIContext; program_: GLuint; location: GLint; params: PGLuint );
		bind_frag_data_location_EXT : procedure( ctx: GLIContext; program_: GLuint; colorNumber: GLuint; const name: PChar );
		get_frag_data_location_EXT : function( ctx: GLIContext; program_: GLuint; const name: PChar ): GLint;
	
		{ EXT_draw_buffers2 }
		color_mask_indexed_EXT : procedure( ctx: GLIContext; index: GLuint; r: GLboolean; g: GLboolean; b: GLboolean; a: GLboolean );
		enable_indexed_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint );
		disable_indexed_EXT : procedure( ctx: GLIContext; target: GLenum; index: GLuint );
		is_enabled_indexed_EXT : function( ctx: GLIContext; target: GLenum; index: GLuint ): GLboolean;
	
		{ OpenGL 2.1 }
		uniform_matrix2x3fv : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix3x2fv : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix2x4fv : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix4x2fv : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix3x4fv : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
		uniform_matrix4x3fv : procedure( ctx: GLIContext; location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	
		{ EXT_framebuffer_blit and EXT_framebuffer_multisample }
		blit_framebuffer_EXT : procedure( ctx: GLIContext; srcX0: GLint; srcY0: GLint; srcX1: GLint; srcY1: GLint; dstX0: GLint; dstY0: GLint; dstX1: GLint; dstY1: GLint; mask: GLbitfield; filter: GLenum );
		renderbuffer_storage_multisample_EXT : procedure( ctx: GLIContext; target: GLenum; samples: GLsizei; internalformat: GLenum; width: GLsizei; height: GLsizei );
	
		{ NV_conditional_render }
		begin_conditional_render_NV : procedure( ctx: GLIContext; id: GLuint; mode: GLenum );
		end_conditional_render_NV : procedure( ctx: GLIContext );
	
		get_attached_shaders : procedure( ctx: GLIContext; program_: GLuint; maxCount: GLsizei; count: PGLsizei; shaders: PGLuint );
	end;

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
