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

unit macglext;
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


{
** License Applicability. Except to the extent portions of this file are
** made subject to an alternative license as permitted in the SGI Free
** Software License B, Version 1.1 (the "License"), the contents of this
** file are subject only to the provisions of the License. You may not use
** this file except in compliance with the License. You may obtain a copy
** of the License at Silicon Graphics, Inc., attn: Legal Services, 1600
** Amphitheatre Parkway, Mountain View, CA 94043-1351, or at:
**
** http://oss.sgi.com/projects/FreeB
**
** Note that, as provided in the License, the Software is distributed on an
** "AS IS" basis, with ALL EXPRESS AND IMPLIED WARRANTIES AND CONDITIONS
** DISCLAIMED, INCLUDING, WITHOUT LIMITATION, ANY IMPLIED WARRANTIES AND
** CONDITIONS OF MERCHANTABILITY, SATISFACTORY QUALITY, FITNESS FOR A
** PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
**
** Original Code. The Original Code is: OpenGL Sample Implementation,
** Version 1.2.1, released January 26, 2000, developed by Silicon Graphics,
** Inc. The Original Code is Copyright (c) 1991-2000 Silicon Graphics, Inc.
** Copyright in any portions created by third parties is as indicated
** elsewhere herein. All Rights Reserved.
**
** Additional Notice Provisions: This software was created using the
** OpenGL(R) version 1.2.1 Sample Implementation published by SGI, but has
** not been independently verified as being compliant with the OpenGL(R)
** version 1.2.1 Specification.
}

{ switches to providing function pointers }
{ #define GL_GLEXT_FUNCTION_POINTERS 1 }

{$setc GL_ARB_imaging := TRUE}
{$setc GL_ARB_transpose_matrix := TRUE}
{$setc GL_ARB_multitexture := TRUE}
{$setc GL_ARB_texture_env_add := TRUE}
{$setc GL_ARB_texture_env_combine := TRUE}
{$setc GL_ARB_texture_env_dot3 := TRUE}
{$setc GL_ARB_texture_env_crossbar := TRUE}
{$setc GL_ARB_texture_cube_map := TRUE}
{$setc GL_ARB_texture_compression := TRUE}
{$setc GL_ARB_multisample := TRUE}
{$setc GL_ARB_texture_border_clamp := TRUE}
{$setc GL_ARB_point_parameters := TRUE}
{$setc GL_ARB_vertex_program := TRUE}
{$setc GL_ARB_fragment_program := TRUE}
{$setc GL_ARB_fragment_program_shadow := TRUE}
{$setc GL_ARB_texture_mirrored_repeat := TRUE}
{$setc GL_ARB_depth_texture := TRUE}
{$setc GL_ARB_shadow := TRUE}
{$setc GL_ARB_shadow_ambient := TRUE}
{$setc GL_ARB_vertex_blend := TRUE}
{$setc GL_ARB_window_pos := TRUE}
{$setc GL_ARB_occlusion_query := TRUE}
{$setc GL_ARB_shader_objects := TRUE}
{$setc GL_ARB_vertex_shader := TRUE}
{$setc GL_ARB_fragment_shader := TRUE}
{$setc GL_ARB_shading_language_100 := TRUE}
{$setc GL_ARB_vertex_buffer_object := TRUE}
{$setc GL_ARB_point_sprite := TRUE}
{$setc GL_ARB_texture_non_power_of_two := TRUE}
{$setc GL_ARB_texture_rectangle := TRUE}
{$setc GL_ARB_draw_buffers := TRUE}
{$setc GL_ARB_pixel_buffer_object := TRUE}
{$setc GL_ARB_shader_texture_lod := TRUE}
{$setc GL_ARB_texture_float := TRUE}
{$setc GL_ARB_half_float_pixel := TRUE}
{$setc GL_ARB_color_buffer_float := TRUE}
{$setc GL_ARB_half_float_vertex := TRUE}
{$setc GL_ARB_texture_compression_rgtc := TRUE}
{$setc GL_ARB_texture_rg := TRUE}
{$setc GL_EXT_clip_volume_hint := TRUE}
{$setc GL_EXT_rescale_normal := TRUE}
{$setc GL_EXT_blend_color := TRUE}
{$setc GL_EXT_blend_minmax := TRUE}
{$setc GL_EXT_blend_subtract := TRUE}
{$setc GL_EXT_compiled_vertex_array := TRUE}
{$setc GL_EXT_texture_lod_bias := TRUE}
{$setc GL_EXT_texture_env_add := TRUE}
{$setc GL_EXT_abgr := TRUE}
{$setc GL_EXT_bgra := TRUE}
{$setc GL_EXT_texture_filter_anisotropic := TRUE}
{$setc GL_EXT_secondary_color := TRUE}
{$setc GL_EXT_separate_specular_color := TRUE}
{$setc GL_EXT_texture_compression_s3tc := TRUE}
{$setc GL_EXT_texture_rectangle := TRUE}
{$setc GL_EXT_fog_coord := TRUE}
{$setc GL_EXT_draw_range_elements := TRUE}
{$setc GL_EXT_stencil_wrap := TRUE}
{$setc GL_EXT_blend_func_separate := TRUE}
{$setc GL_EXT_multi_draw_arrays := TRUE}
{$setc GL_EXT_shadow_funcs := TRUE}
{$setc GL_EXT_stencil_two_side := TRUE}
{$setc GL_EXT_depth_bounds_test := TRUE}
{$setc GL_EXT_blend_equation_separate := TRUE}
{$setc GL_EXT_texture_mirror_clamp := TRUE}
{$setc GL_EXT_texture_compression_dxt1 := TRUE}
{$setc GL_EXT_texture_sRGB := TRUE}
{$setc GL_EXT_framebuffer_object := TRUE}
{$setc GL_EXT_framebuffer_blit := TRUE}
{$setc GL_EXT_framebuffer_multisample := TRUE}
{$setc GL_EXT_packed_depth_stencil := TRUE}
{$setc GL_EXT_gpu_program_parameters := TRUE}
{$setc GL_EXT_geometry_shader4 := TRUE}
{$setc GL_EXT_transform_feedback := TRUE}
{$setc GL_EXT_bindable_uniform := TRUE}
{$setc GL_EXT_texture_integer := TRUE}
{$setc GL_EXT_gpu_shader4 := TRUE}
{$setc GL_EXT_draw_buffers2 := TRUE}
{$setc GL_EXT_framebuffer_sRGB := TRUE}
{$setc GL_APPLE_flush_buffer_range := TRUE}
{$setc GL_APPLE_specular_vector := TRUE}
{$setc GL_APPLE_transform_hint := TRUE}
{$setc GL_APPLE_packed_pixels := TRUE}
{$setc GL_APPLE_client_storage := TRUE}
{$setc GL_APPLE_ycbcr_422 := TRUE}
{$setc GL_APPLE_texture_range := TRUE}
{$setc GL_APPLE_fence := TRUE}
{$setc GL_APPLE_vertex_array_range := TRUE}
{$setc GL_APPLE_vertex_array_object := TRUE}
{$setc GL_APPLE_element_array := TRUE}
{$setc GL_APPLE_vertex_program_evaluators := TRUE}
{$setc GL_APPLE_float_pixels := TRUE}
{$setc GL_APPLE_flush_render := TRUE}
{$setc GL_APPLE_pixel_buffer := TRUE}
{$setc GL_APPLE_aux_depth_stencil := TRUE}
{$setc GL_APPLE_row_bytes := TRUE}
{$setc GL_APPLE_object_purgeable := TRUE}
{$setc GL_APPLE_rgb_422 := TRUE}
{$setc GL_ATI_point_cull_mode := TRUE}
{$setc GL_ATI_texture_mirror_once := TRUE}
{$setc GL_ATI_pn_triangles := TRUE}
{$setc GL_ATI_blend_equation_separate := TRUE}
{$setc GL_ATI_blend_weighted_minmax := TRUE}
{$setc GL_ATI_texture_env_combine3 := TRUE}
{$setc GL_ATI_separate_stencil := TRUE}
{$setc GL_ATI_texture_compression_3dc := TRUE}
{$setc GL_ATI_texture_float := TRUE}
{$setc GL_ATIX_pn_triangles := TRUE}
{$setc GL_IBM_rasterpos_clip := TRUE}
{$setc GL_NV_point_sprite := TRUE}
{$setc GL_NV_blend_square := TRUE}
{$setc GL_NV_fog_distance := TRUE}
{$setc GL_NV_multisample_filter_hint := TRUE}
{$setc GL_NV_texgen_reflection := TRUE}
{$setc GL_NV_depth_clamp := TRUE}
{$setc GL_NV_light_max_exponent := TRUE}
{$setc GL_NV_fragment_program_option := TRUE}
{$setc GL_NV_fragment_program2 := TRUE}
{$setc GL_NV_vertex_program2_option := TRUE}
{$setc GL_NV_vertex_program3 := TRUE}
{$setc GL_SGI_color_matrix := TRUE}
{$setc GL_SGIS_texture_edge_clamp := TRUE}
{$setc GL_SGIS_generate_mipmap := TRUE}
{$setc GL_SGIS_texture_lod := TRUE}
{$setc GL_NV_conditional_render := TRUE}

{$ifc not undefined GL_GLEXT_WUNDEF_SUPPORT and GL_GLEXT_WUNDEF_SUPPORT}
{$setc GL_SGIX_pixel_texture := FALSE}
{$setc GL_SGIX_pixel_tiles := FALSE}
{$setc GL_SGIX_polynomial_ffd := FALSE}
{$setc GL_SGIX_reference_plane := FALSE}
{$setc GL_SGIX_resample := FALSE}
{$setc GL_SGIX_shadow := FALSE}
{$setc GL_SGIX_sprite := FALSE}
{$setc GL_SGIX_subsample := FALSE}
{$setc GL_SGIX_tag_sample_buffer := FALSE}
{$setc GL_SGIX_texture_add_env := FALSE}
{$setc GL_SGIX_texture_lod_bias := FALSE}
{$setc GL_SGIX_texture_multi_buffer := FALSE}
{$setc GL_SGIX_texture_scale_bias := FALSE}
{$setc GL_SGIX_vertex_preclip := FALSE}
{$setc GL_SGIX_ycrcb := FALSE}
{$setc GL_SGIX_ycrcba := FALSE}
{$setc GL_SUN_convolution_border_modes := FALSE}
{$setc GL_SUN_global_alpha := FALSE}
{$setc GL_SUN_triangle_list := FALSE}
{$setc GL_SUN_vertex := FALSE}
{$setc GL_SUNX_constant_data := FALSE}
{$setc GL_WIN_phong_shading := FALSE}
{$setc GL_WIN_specular_fog := FALSE}
{$setc GL_3DFX_multisample := FALSE}
{$setc GL_3DFX_tbuffer := FALSE}
{$setc GL_3DFX_texture_compression_FXT1 := FALSE}
{$endc}

{***********************************************************}
{$ifc not undefined GL_ARB_shader_objects and GL_ARB_shader_objects}
type
	GLcharARB = char;
	PGLcharARB = ^GLcharARB;
	
	GLhandleARB = UnivPtr;
	PGLhandleARB = ^GLhandleARB;
{$endc}

{$ifc not undefined GL_ARB_vertex_buffer_object and GL_ARB_vertex_buffer_object}
type
	GLintptrARB = SIGNEDLONG;
	PGLintptrARB = ^GLintptrARB;
	
	GLsizeiptrARB = SIGNEDLONG;
	PGLsizeiptrARB = ^GLsizeiptrARB;
	
{$endc}

{$ifc not undefined GL_ARB_half_float_pixel and GL_ARB_half_float_pixel}
type
	GLhalfARB = UInt16;
	PGLhalfARB = ^GLhalfARB;
	
{$endc}

{$ifc not undefined GL_ARB_half_float_vertex and GL_ARB_half_float_vertex}
type
	GLhalf = UInt16;
	PGLhalf = ^GLhalf;
{$endc}

{***********************************************************}
const
	GL_GLEXT_VERSION = 7;

{$ifc not undefined GL_ARB_multitexture and GL_ARB_multitexture}
const GL_TEXTURE0_ARB                   = $84C0;
const GL_TEXTURE1_ARB                   = $84C1;
const GL_TEXTURE2_ARB                   = $84C2;
const GL_TEXTURE3_ARB                   = $84C3;
const GL_TEXTURE4_ARB                   = $84C4;
const GL_TEXTURE5_ARB                   = $84C5;
const GL_TEXTURE6_ARB                   = $84C6;
const GL_TEXTURE7_ARB                   = $84C7;
const GL_TEXTURE8_ARB                   = $84C8;
const GL_TEXTURE9_ARB                   = $84C9;
const GL_TEXTURE10_ARB                  = $84CA;
const GL_TEXTURE11_ARB                  = $84CB;
const GL_TEXTURE12_ARB                  = $84CC;
const GL_TEXTURE13_ARB                  = $84CD;
const GL_TEXTURE14_ARB                  = $84CE;
const GL_TEXTURE15_ARB                  = $84CF;
const GL_TEXTURE16_ARB                  = $84D0;
const GL_TEXTURE17_ARB                  = $84D1;
const GL_TEXTURE18_ARB                  = $84D2;
const GL_TEXTURE19_ARB                  = $84D3;
const GL_TEXTURE20_ARB                  = $84D4;
const GL_TEXTURE21_ARB                  = $84D5;
const GL_TEXTURE22_ARB                  = $84D6;
const GL_TEXTURE23_ARB                  = $84D7;
const GL_TEXTURE24_ARB                  = $84D8;
const GL_TEXTURE25_ARB                  = $84D9;
const GL_TEXTURE26_ARB                  = $84DA;
const GL_TEXTURE27_ARB                  = $84DB;
const GL_TEXTURE28_ARB                  = $84DC;
const GL_TEXTURE29_ARB                  = $84DD;
const GL_TEXTURE30_ARB                  = $84DE;
const GL_TEXTURE31_ARB                  = $84DF;
const GL_ACTIVE_TEXTURE_ARB             = $84E0;
const GL_CLIENT_ACTIVE_TEXTURE_ARB      = $84E1;
const GL_MAX_TEXTURE_UNITS_ARB          = $84E2;
{$endc}

{$ifc not undefined GL_ARB_transpose_matrix and GL_ARB_transpose_matrix}
const GL_TRANSPOSE_MODELVIEW_MATRIX_ARB = $84E3;
const GL_TRANSPOSE_PROJECTION_MATRIX_ARB = $84E4;
const GL_TRANSPOSE_TEXTURE_MATRIX_ARB   = $84E5;
const GL_TRANSPOSE_COLOR_MATRIX_ARB     = $84E6;
{$endc}

{$ifc not undefined GL_ARB_multisample and GL_ARB_multisample}
const GL_MULTISAMPLE_ARB                = $809D;
const GL_SAMPLE_ALPHA_TO_COVERAGE_ARB   = $809E;
const GL_SAMPLE_ALPHA_TO_ONE_ARB        = $809F;
const GL_SAMPLE_COVERAGE_ARB            = $80A0;
const GL_SAMPLE_BUFFERS_ARB             = $80A8;
const GL_SAMPLES_ARB                    = $80A9;
const GL_SAMPLE_COVERAGE_VALUE_ARB      = $80AA;
const GL_SAMPLE_COVERAGE_INVERT_ARB     = $80AB;
const GL_MULTISAMPLE_BIT_ARB            = $20000000;
{$endc}

{$ifc not undefined GL_ARB_texture_cube_map and GL_ARB_texture_cube_map}
const GL_NORMAL_MAP_ARB                 = $8511;
const GL_REFLECTION_MAP_ARB             = $8512;
const GL_TEXTURE_CUBE_MAP_ARB           = $8513;
const GL_TEXTURE_BINDING_CUBE_MAP_ARB   = $8514;
const GL_TEXTURE_CUBE_MAP_POSITIVE_X_ARB = $8515;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_X_ARB = $8516;
const GL_TEXTURE_CUBE_MAP_POSITIVE_Y_ARB = $8517;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_ARB = $8518;
const GL_TEXTURE_CUBE_MAP_POSITIVE_Z_ARB = $8519;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_ARB = $851A;
const GL_PROXY_TEXTURE_CUBE_MAP_ARB     = $851B;
const GL_MAX_CUBE_MAP_TEXTURE_SIZE_ARB  = $851C;
{$endc}

{$ifc not undefined GL_ARB_texture_compression and GL_ARB_texture_compression}
const GL_COMPRESSED_ALPHA_ARB           = $84E9;
const GL_COMPRESSED_LUMINANCE_ARB       = $84EA;
const GL_COMPRESSED_LUMINANCE_ALPHA_ARB = $84EB;
const GL_COMPRESSED_INTENSITY_ARB       = $84EC;
const GL_COMPRESSED_RGB_ARB             = $84ED;
const GL_COMPRESSED_RGBA_ARB            = $84EE;
const GL_TEXTURE_COMPRESSION_HINT_ARB   = $84EF;
const GL_TEXTURE_COMPRESSED_IMAGE_SIZE_ARB = $86A0;
const GL_TEXTURE_COMPRESSED_ARB         = $86A1;
const GL_NUM_COMPRESSED_TEXTURE_FORMATS_ARB = $86A2;
const GL_COMPRESSED_TEXTURE_FORMATS_ARB = $86A3;
{$endc}

{$ifc not undefined GL_ARB_vertex_blend and GL_ARB_vertex_blend}
const GL_MAX_VERTEX_UNITS_ARB           = $86A4;
const GL_ACTIVE_VERTEX_UNITS_ARB        = $86A5;
const GL_WEIGHT_SUM_UNITY_ARB           = $86A6;
const GL_VERTEX_BLEND_ARB               = $86A7;
const GL_CURRENT_WEIGHT_ARB             = $86A8;
const GL_WEIGHT_ARRAY_TYPE_ARB          = $86A9;
const GL_WEIGHT_ARRAY_STRIDE_ARB        = $86AA;
const GL_WEIGHT_ARRAY_SIZE_ARB          = $86AB;
const GL_WEIGHT_ARRAY_POINTER_ARB       = $86AC;
const GL_WEIGHT_ARRAY_ARB               = $86AD;
const GL_MODELVIEW0_ARB                 = $1700;
const GL_MODELVIEW1_ARB                 = $850A;
const GL_MODELVIEW2_ARB                 = $8722;
const GL_MODELVIEW3_ARB                 = $8723;
const GL_MODELVIEW4_ARB                 = $8724;
const GL_MODELVIEW5_ARB                 = $8725;
const GL_MODELVIEW6_ARB                 = $8726;
const GL_MODELVIEW7_ARB                 = $8727;
const GL_MODELVIEW8_ARB                 = $8728;
const GL_MODELVIEW9_ARB                 = $8729;
const GL_MODELVIEW10_ARB                = $872A;
const GL_MODELVIEW11_ARB                = $872B;
const GL_MODELVIEW12_ARB                = $872C;
const GL_MODELVIEW13_ARB                = $872D;
const GL_MODELVIEW14_ARB                = $872E;
const GL_MODELVIEW15_ARB                = $872F;
const GL_MODELVIEW16_ARB                = $8730;
const GL_MODELVIEW17_ARB                = $8731;
const GL_MODELVIEW18_ARB                = $8732;
const GL_MODELVIEW19_ARB                = $8733;
const GL_MODELVIEW20_ARB                = $8734;
const GL_MODELVIEW21_ARB                = $8735;
const GL_MODELVIEW22_ARB                = $8736;
const GL_MODELVIEW23_ARB                = $8737;
const GL_MODELVIEW24_ARB                = $8738;
const GL_MODELVIEW25_ARB                = $8739;
const GL_MODELVIEW26_ARB                = $873A;
const GL_MODELVIEW27_ARB                = $873B;
const GL_MODELVIEW28_ARB                = $873C;
const GL_MODELVIEW29_ARB                = $873D;
const GL_MODELVIEW30_ARB                = $873E;
const GL_MODELVIEW31_ARB                = $873F;
{$endc}

{$ifc not undefined GL_ARB_occlusion_query and GL_ARB_occlusion_query}
const GL_SAMPLES_PASSED_ARB             = $8914;
const GL_QUERY_COUNTER_BITS_ARB         = $8864;
const GL_CURRENT_QUERY_ARB              = $8865;
const GL_QUERY_RESULT_ARB               = $8866;
const GL_QUERY_RESULT_AVAILABLE_ARB     = $8867;
{$endc}

{$ifc not undefined GL_ARB_texture_border_clamp and GL_ARB_texture_border_clamp}
const GL_CLAMP_TO_BORDER_ARB           = $812D;
{$endc}

{$ifc not undefined GL_ARB_depth_texture and GL_ARB_depth_texture}
const GL_DEPTH_COMPONENT16_ARB          = $81A5;
const GL_DEPTH_COMPONENT24_ARB          = $81A6;
const GL_DEPTH_COMPONENT32_ARB          = $81A7;
const GL_TEXTURE_DEPTH_SIZE_ARB         = $884A;
const GL_DEPTH_TEXTURE_MODE_ARB         = $884B;
{$endc}

{$ifc not undefined GL_ARB_shadow and GL_ARB_shadow}
const GL_TEXTURE_COMPARE_MODE_ARB       = $884C;
const GL_TEXTURE_COMPARE_FUNC_ARB       = $884D;
const GL_COMPARE_R_TO_TEXTURE_ARB       = $884E;
{$endc}

{$ifc not undefined GL_ARB_shadow_ambient and GL_ARB_shadow_ambient}
const GL_TEXTURE_COMPARE_FAIL_VALUE_ARB = $80BF;
{$endc}

{$ifc not undefined GL_ARB_texture_env_combine and GL_ARB_texture_env_combine}
const GL_COMBINE_ARB                    = $8570;
const GL_COMBINE_RGB_ARB                = $8571;
const GL_COMBINE_ALPHA_ARB              = $8572;
const GL_RGB_SCALE_ARB                  = $8573;
const GL_ADD_SIGNED_ARB                 = $8574;
const GL_INTERPOLATE_ARB                = $8575;
const GL_CONSTANT_ARB                   = $8576;
const GL_PRIMARY_COLOR_ARB              = $8577;
const GL_PREVIOUS_ARB                   = $8578;
const GL_SUBTRACT_ARB                   = $84E7;
const GL_SOURCE0_RGB_ARB                = $8580;
const GL_SOURCE1_RGB_ARB                = $8581;
const GL_SOURCE2_RGB_ARB                = $8582;
const GL_SOURCE3_RGB_ARB                = $8583;
const GL_SOURCE4_RGB_ARB                = $8584;
const GL_SOURCE5_RGB_ARB                = $8585;
const GL_SOURCE6_RGB_ARB                = $8586;
const GL_SOURCE7_RGB_ARB                = $8587;
const GL_SOURCE0_ALPHA_ARB              = $8588;
const GL_SOURCE1_ALPHA_ARB              = $8589;
const GL_SOURCE2_ALPHA_ARB              = $858A;
const GL_SOURCE3_ALPHA_ARB              = $858B;
const GL_SOURCE4_ALPHA_ARB              = $858C;
const GL_SOURCE5_ALPHA_ARB              = $858D;
const GL_SOURCE6_ALPHA_ARB              = $858E;
const GL_SOURCE7_ALPHA_ARB              = $858F;
const GL_OPERAND0_RGB_ARB               = $8590;
const GL_OPERAND1_RGB_ARB               = $8591;
const GL_OPERAND2_RGB_ARB               = $8592;
const GL_OPERAND3_RGB_ARB               = $8593;
const GL_OPERAND4_RGB_ARB               = $8594;
const GL_OPERAND5_RGB_ARB               = $8595;
const GL_OPERAND6_RGB_ARB               = $8596;
const GL_OPERAND7_RGB_ARB               = $8597;
const GL_OPERAND0_ALPHA_ARB             = $8598;
const GL_OPERAND1_ALPHA_ARB             = $8599;
const GL_OPERAND2_ALPHA_ARB             = $859A;
const GL_OPERAND3_ALPHA_ARB             = $859B;
const GL_OPERAND4_ALPHA_ARB             = $859C;
const GL_OPERAND5_ALPHA_ARB             = $859D;
const GL_OPERAND6_ALPHA_ARB             = $859E;
const GL_OPERAND7_ALPHA_ARB             = $859F;
{$endc}

{$ifc not undefined GL_ARB_texture_mirrored_repeat and GL_ARB_texture_mirrored_repeat}
const GL_MIRRORED_REPEAT_ARB            = $8370;
{$endc}

{$ifc not undefined GL_ARB_texture_env_dot3 and GL_ARB_texture_env_dot3}
const GL_DOT3_RGB_ARB                   = $86AE;
const GL_DOT3_RGBA_ARB                  = $86AF;
{$endc}

{$ifc not undefined GL_ARB_point_parameters and GL_ARB_point_parameters}
const GL_POINT_SIZE_MIN_ARB                            = $8126;
const GL_POINT_SIZE_MAX_ARB                            = $8127;
const GL_POINT_FADE_THRESHOLD_SIZE_ARB                 = $8128;
const GL_POINT_DISTANCE_ATTENUATION_ARB                = $8129;
{$endc}

{$ifc not undefined GL_ARB_fragment_program and GL_ARB_fragment_program}
const GL_FRAGMENT_PROGRAM_ARB                         = $8804;
const GL_PROGRAM_ALU_INSTRUCTIONS_ARB                 = $8805;
const GL_PROGRAM_TEX_INSTRUCTIONS_ARB                 = $8806;
const GL_PROGRAM_TEX_INDIRECTIONS_ARB                 = $8807;
const GL_PROGRAM_NATIVE_ALU_INSTRUCTIONS_ARB          = $8808;
const GL_PROGRAM_NATIVE_TEX_INSTRUCTIONS_ARB          = $8809;
const GL_PROGRAM_NATIVE_TEX_INDIRECTIONS_ARB          = $880A;
const GL_MAX_PROGRAM_ALU_INSTRUCTIONS_ARB             = $880B;
const GL_MAX_PROGRAM_TEX_INSTRUCTIONS_ARB             = $880C;
const GL_MAX_PROGRAM_TEX_INDIRECTIONS_ARB             = $880D;
const GL_MAX_PROGRAM_NATIVE_ALU_INSTRUCTIONS_ARB      = $880E;
const GL_MAX_PROGRAM_NATIVE_TEX_INSTRUCTIONS_ARB      = $880F;
const GL_MAX_PROGRAM_NATIVE_TEX_INDIRECTIONS_ARB      = $8810;
const GL_MAX_TEXTURE_COORDS_ARB                       = $8871;
const GL_MAX_TEXTURE_IMAGE_UNITS_ARB                  = $8872;
{$endc}

{$ifc not undefined GL_ARB_vertex_program and GL_ARB_vertex_program}
const GL_VERTEX_PROGRAM_ARB                            = $8620;
const GL_VERTEX_PROGRAM_POINT_SIZE_ARB                 = $8642;
const GL_VERTEX_PROGRAM_TWO_SIDE_ARB                   = $8643;
const GL_PROGRAM_FORMAT_ASCII_ARB                      = $8875;
const GL_VERTEX_ATTRIB_ARRAY_ENABLED_ARB               = $8622;
const GL_VERTEX_ATTRIB_ARRAY_SIZE_ARB                  = $8623;
const GL_VERTEX_ATTRIB_ARRAY_STRIDE_ARB                = $8624;
const GL_VERTEX_ATTRIB_ARRAY_TYPE_ARB                  = $8625;
const GL_VERTEX_ATTRIB_ARRAY_NORMALIZED_ARB            = $886A;
const GL_CURRENT_VERTEX_ATTRIB_ARB                     = $8626;
const GL_VERTEX_ATTRIB_ARRAY_POINTER_ARB               = $8645;
const GL_PROGRAM_LENGTH_ARB                            = $8627;
const GL_PROGRAM_FORMAT_ARB                            = $8876;
const GL_PROGRAM_NAME_ARB                              = $8677;
const GL_PROGRAM_BINDING_ARB                           = $8677;
const GL_PROGRAM_INSTRUCTIONS_ARB                      = $88A0;
const GL_MAX_PROGRAM_INSTRUCTIONS_ARB                  = $88A1;
const GL_PROGRAM_NATIVE_INSTRUCTIONS_ARB               = $88A2;
const GL_MAX_PROGRAM_NATIVE_INSTRUCTIONS_ARB           = $88A3;
const GL_PROGRAM_TEMPORARIES_ARB                       = $88A4;
const GL_MAX_PROGRAM_TEMPORARIES_ARB                   = $88A5;
const GL_PROGRAM_NATIVE_TEMPORARIES_ARB                = $88A6;
const GL_MAX_PROGRAM_NATIVE_TEMPORARIES_ARB            = $88A7;
const GL_PROGRAM_PARAMETERS_ARB                        = $88A8;
const GL_MAX_PROGRAM_PARAMETERS_ARB                    = $88A9;
const GL_PROGRAM_NATIVE_PARAMETERS_ARB                 = $88AA;
const GL_MAX_PROGRAM_NATIVE_PARAMETERS_ARB             = $88AB;
const GL_PROGRAM_ATTRIBS_ARB                           = $88AC;
const GL_MAX_PROGRAM_ATTRIBS_ARB                       = $88AD;
const GL_PROGRAM_NATIVE_ATTRIBS_ARB                    = $88AE;
const GL_MAX_PROGRAM_NATIVE_ATTRIBS_ARB                = $88AF;
const GL_PROGRAM_ADDRESS_REGISTERS_ARB                 = $88B0;
const GL_MAX_PROGRAM_ADDRESS_REGISTERS_ARB             = $88B1;
const GL_PROGRAM_NATIVE_ADDRESS_REGISTERS_ARB          = $88B2;
const GL_MAX_PROGRAM_NATIVE_ADDRESS_REGISTERS_ARB      = $88B3;
const GL_MAX_PROGRAM_LOCAL_PARAMETERS_ARB              = $88B4;
const GL_MAX_PROGRAM_ENV_PARAMETERS_ARB                = $88B5;
const GL_PROGRAM_UNDER_NATIVE_LIMITS_ARB               = $88B6;
const GL_PROGRAM_STRING_ARB                            = $8628;
const GL_PROGRAM_ERROR_POSITION_ARB                    = $864B;
const GL_CURRENT_MATRIX_ARB                            = $8641;
const GL_TRANSPOSE_CURRENT_MATRIX_ARB                  = $88B7;
const GL_CURRENT_MATRIX_STACK_DEPTH_ARB                = $8640;
const GL_MAX_VERTEX_ATTRIBS_ARB                        = $8869;
const GL_MAX_PROGRAM_MATRICES_ARB                      = $862F;
const GL_MAX_PROGRAM_MATRIX_STACK_DEPTH_ARB            = $862E;
const GL_PROGRAM_ERROR_STRING_ARB                      = $8874;

const GL_MATRIX0_ARB                                   = $88C0;
const GL_MATRIX1_ARB                                   = $88C1;
const GL_MATRIX2_ARB                                   = $88C2;
const GL_MATRIX3_ARB                                   = $88C3;
const GL_MATRIX4_ARB                                   = $88C4;
const GL_MATRIX5_ARB                                   = $88C5;
const GL_MATRIX6_ARB                                   = $88C6;
const GL_MATRIX7_ARB                                   = $88C7;
const GL_MATRIX8_ARB                                   = $88C8;
const GL_MATRIX9_ARB                                   = $88C9;
const GL_MATRIX10_ARB                                  = $88CA;
const GL_MATRIX11_ARB                                  = $88CB;
const GL_MATRIX12_ARB                                  = $88CC;
const GL_MATRIX13_ARB                                  = $88CD;
const GL_MATRIX14_ARB                                  = $88CE;
const GL_MATRIX15_ARB                                  = $88CF;
const GL_MATRIX16_ARB                                  = $88D0;
const GL_MATRIX17_ARB                                  = $88D1;
const GL_MATRIX18_ARB                                  = $88D2;
const GL_MATRIX19_ARB                                  = $88D3;
const GL_MATRIX20_ARB                                  = $88D4;
const GL_MATRIX21_ARB                                  = $88D5;
const GL_MATRIX22_ARB                                  = $88D6;
const GL_MATRIX23_ARB                                  = $88D7;
const GL_MATRIX24_ARB                                  = $88D8;
const GL_MATRIX25_ARB                                  = $88D9;
const GL_MATRIX26_ARB                                  = $88DA;
const GL_MATRIX27_ARB                                  = $88DB;
const GL_MATRIX28_ARB                                  = $88DC;
const GL_MATRIX29_ARB                                  = $88DD;
const GL_MATRIX30_ARB                                  = $88DE;
const GL_MATRIX31_ARB                                  = $88DF;

const GL_COLOR_SUM_ARB                                 = $8458;
{$endc}

{$ifc not undefined GL_ARB_shader_objects and GL_ARB_shader_objects}
const GL_PROGRAM_OBJECT_ARB                              = $8B40;
const GL_OBJECT_TYPE_ARB                                 = $8B4E;
const GL_OBJECT_SUBTYPE_ARB                              = $8B4F;
const GL_OBJECT_DELETE_STATUS_ARB                        = $8B80;
const GL_OBJECT_COMPILE_STATUS_ARB                       = $8B81;
const GL_OBJECT_LINK_STATUS_ARB                          = $8B82;
const GL_OBJECT_VALIDATE_STATUS_ARB                      = $8B83;
const GL_OBJECT_INFO_LOG_LENGTH_ARB                      = $8B84;
const GL_OBJECT_ATTACHED_OBJECTS_ARB                     = $8B85;
const GL_OBJECT_ACTIVE_UNIFORMS_ARB                      = $8B86;
const GL_OBJECT_ACTIVE_UNIFORM_MAX_LENGTH_ARB            = $8B87;
const GL_OBJECT_SHADER_SOURCE_LENGTH_ARB                 = $8B88;
const GL_SHADER_OBJECT_ARB                               = $8B48;
const GL_FLOAT_VEC2_ARB                                  = $8B50;
const GL_FLOAT_VEC3_ARB                                  = $8B51;
const GL_FLOAT_VEC4_ARB                                  = $8B52;
const GL_INT_VEC2_ARB                                    = $8B53;
const GL_INT_VEC3_ARB                                    = $8B54;
const GL_INT_VEC4_ARB                                    = $8B55;
const GL_BOOL_ARB                                        = $8B56;
const GL_BOOL_VEC2_ARB                                   = $8B57;
const GL_BOOL_VEC3_ARB                                   = $8B58;
const GL_BOOL_VEC4_ARB                                   = $8B59;
const GL_FLOAT_MAT2_ARB                                  = $8B5A;
const GL_FLOAT_MAT3_ARB                                  = $8B5B;
const GL_FLOAT_MAT4_ARB                                  = $8B5C;
const GL_SAMPLER_1D_ARB                                  = $8B5D;
const GL_SAMPLER_2D_ARB                                  = $8B5E;
const GL_SAMPLER_3D_ARB                                  = $8B5F;
const GL_SAMPLER_CUBE_ARB                                = $8B60;
const GL_SAMPLER_1D_SHADOW_ARB                           = $8B61;
const GL_SAMPLER_2D_SHADOW_ARB                           = $8B62;
const GL_SAMPLER_2D_RECT_ARB                             = $8B63;
const GL_SAMPLER_2D_RECT_SHADOW_ARB                      = $8B64;
{$endc}

{$ifc not undefined GL_ARB_vertex_shader and GL_ARB_vertex_shader or defined GL_NV_vertex_program3 and GL_NV_vertex_program3}
const GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS_ARB              = $8B4C;
{$endc}

{$ifc not undefined GL_ARB_vertex_shader and GL_ARB_vertex_shader}
const GL_VERTEX_SHADER_ARB                               = $8B31;
const GL_MAX_VERTEX_UNIFORM_COMPONENTS_ARB               = $8B4A;
{const GL_MAX_TEXTURE_COORDS_ARB                          = $8871;}
{const GL_MAX_TEXTURE_IMAGE_UNITS_ARB                     = $8872;}
const GL_MAX_VARYING_FLOATS_ARB                          = $8B4B;
{const GL_MAX_VERTEX_ATTRIBS_ARB                          = $8869;}
const GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS_ARB            = $8B4D;
{const GL_VERTEX_PROGRAM_POINT_SIZE_ARB                   = $8642;}
{const GL_VERTEX_PROGRAM_TWO_SIDE_ARB                     = $8643;}
const GL_OBJECT_ACTIVE_ATTRIBUTES_ARB                    = $8B89;
const GL_OBJECT_ACTIVE_ATTRIBUTE_MAX_LENGTH_ARB          = $8B8A;
{const GL_VERTEX_ATTRIB_ARRAY_ENABLED_ARB                 = $8622;}
{const GL_VERTEX_ATTRIB_ARRAY_SIZE_ARB                    = $8623;}
{const GL_VERTEX_ATTRIB_ARRAY_STRIDE_ARB                  = $8624;}
{const GL_VERTEX_ATTRIB_ARRAY_TYPE_ARB                    = $8625;}
{const GL_VERTEX_ATTRIB_ARRAY_NORMALIZED_ARB              = $886A;}
{const GL_CURRENT_VERTEX_ATTRIB_ARB                       = $8626;}
{const GL_VERTEX_ATTRIB_ARRAY_POINTER_ARB                 = $8645;}
{$endc}

{$ifc undefined GL_ARB_fragment_shader and GL_ARB_fragment_shader}
const GL_FRAGMENT_SHADER_ARB                             = $8B30;
const GL_MAX_FRAGMENT_UNIFORM_COMPONENTS_ARB             = $8B49;
const GL_FRAGMENT_SHADER_DERIVATIVE_HINT_ARB             = $8B8B;
{const GL_MAX_TEXTURE_COORDS_ARB                          = $8871;}
{const GL_MAX_TEXTURE_IMAGE_UNITS_ARB                     = $8872;}
{$endc}

{$ifc not undefined GL_ARB_shading_language_100 and GL_ARB_shading_language_100}
const GL_SHADING_LANGUAGE_VERSION_ARB                    = $8B8C;
{$endc}

{$ifc not undefined GL_ARB_vertex_buffer_object and GL_ARB_vertex_buffer_object}
const GL_ARRAY_BUFFER_ARB                                = $8892;
const GL_ELEMENT_ARRAY_BUFFER_ARB                        = $8893;
const GL_ARRAY_BUFFER_BINDING_ARB                        = $8894;
const GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB                = $8895;
const GL_VERTEX_ARRAY_BUFFER_BINDING_ARB                 = $8896;
const GL_NORMAL_ARRAY_BUFFER_BINDING_ARB                 = $8897;
const GL_COLOR_ARRAY_BUFFER_BINDING_ARB                  = $8898;
const GL_INDEX_ARRAY_BUFFER_BINDING_ARB                  = $8899;
const GL_TEXTURE_COORD_ARRAY_BUFFER_BINDING_ARB          = $889A;
const GL_EDGE_FLAG_ARRAY_BUFFER_BINDING_ARB              = $889B;
const GL_SECONDARY_COLOR_ARRAY_BUFFER_BINDING_ARB        = $889C;
const GL_FOG_COORD_ARRAY_BUFFER_BINDING_ARB              = $889D;
const GL_WEIGHT_ARRAY_BUFFER_BINDING_ARB                 = $889E;
const GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING_ARB          = $889F;
const GL_STREAM_DRAW_ARB                                 = $88E0;
const GL_STREAM_READ_ARB                                 = $88E1;
const GL_STREAM_COPY_ARB                                 = $88E2;
const GL_STATIC_DRAW_ARB                                 = $88E4;
const GL_STATIC_READ_ARB                                 = $88E5;
const GL_STATIC_COPY_ARB                                 = $88E6;
const GL_DYNAMIC_DRAW_ARB                                = $88E8;
const GL_DYNAMIC_READ_ARB                                = $88E9;
const GL_DYNAMIC_COPY_ARB                                = $88EA;
const GL_READ_ONLY_ARB                                   = $88B8;
const GL_WRITE_ONLY_ARB                                  = $88B9;
const GL_READ_WRITE_ARB                                  = $88BA;
const GL_BUFFER_SIZE_ARB                                 = $8764;
const GL_BUFFER_USAGE_ARB                                = $8765;
const GL_BUFFER_ACCESS_ARB                               = $88BB;
const GL_BUFFER_MAPPED_ARB                               = $88BC;
const GL_BUFFER_MAP_POINTER_ARB                          = $88BD;
{ Obsolete }
const GL_FOG_COORDINATE_ARRAY_BUFFER_BINDING_ARB         = $889D;
{$endc}

{$ifc not undefined GL_ARB_point_sprite and GL_ARB_point_sprite}
const GL_POINT_SPRITE_ARB                                = $8861;
const GL_COORD_REPLACE_ARB                               = $8862;
{$endc}

{$ifc not undefined GL_ARB_texture_rectangle and GL_ARB_texture_rectangle}
const GL_TEXTURE_RECTANGLE_ARB          = $84F5;
const GL_TEXTURE_BINDING_RECTANGLE_ARB  = $84F6;
const GL_PROXY_TEXTURE_RECTANGLE_ARB    = $84F7;
const GL_MAX_RECTANGLE_TEXTURE_SIZE_ARB = $84F8;
{$endc}

{$ifc not undefined GL_ARB_draw_buffers and GL_ARB_draw_buffers}
const GL_MAX_DRAW_BUFFERS_ARB           = $8824;
const GL_DRAW_BUFFER0_ARB               = $8825;
const GL_DRAW_BUFFER1_ARB               = $8826;
const GL_DRAW_BUFFER2_ARB               = $8827;
const GL_DRAW_BUFFER3_ARB               = $8828;
const GL_DRAW_BUFFER4_ARB               = $8829;
const GL_DRAW_BUFFER5_ARB               = $882A;
const GL_DRAW_BUFFER6_ARB               = $882B;
const GL_DRAW_BUFFER7_ARB               = $882C;
const GL_DRAW_BUFFER8_ARB               = $882D;
const GL_DRAW_BUFFER9_ARB               = $882E;
const GL_DRAW_BUFFER10_ARB              = $882F;
const GL_DRAW_BUFFER11_ARB              = $8830;
const GL_DRAW_BUFFER12_ARB              = $8831;
const GL_DRAW_BUFFER13_ARB              = $8832;
const GL_DRAW_BUFFER14_ARB              = $8833;
const GL_DRAW_BUFFER15_ARB              = $8834;
{$endc}

{$ifc not undefined GL_ARB_pixel_buffer_object and GL_ARB_pixel_buffer_object}
const GL_PIXEL_PACK_BUFFER_ARB                        = $88EB;
const GL_PIXEL_UNPACK_BUFFER_ARB                      = $88EC;
const GL_PIXEL_PACK_BUFFER_BINDING_ARB                = $88ED;
const GL_PIXEL_UNPACK_BUFFER_BINDING_ARB              = $88EF;
{$endc}

{$ifc not undefined GL_ARB_texture_float and GL_ARB_texture_float}
const GL_TEXTURE_RED_TYPE_ARB             = $8C10;
const GL_TEXTURE_GREEN_TYPE_ARB           = $8C11;
const GL_TEXTURE_BLUE_TYPE_ARB            = $8C12;
const GL_TEXTURE_ALPHA_TYPE_ARB           = $8C13;
const GL_TEXTURE_LUMINANCE_TYPE_ARB       = $8C14;
const GL_TEXTURE_INTENSITY_TYPE_ARB       = $8C15;
const GL_TEXTURE_DEPTH_TYPE_ARB           = $8C16;
const GL_UNSIGNED_NORMALIZED_ARB          = $8C17;
const GL_RGBA32F_ARB                      = $8814;
const GL_RGB32F_ARB                       = $8815;
const GL_ALPHA32F_ARB                     = $8816;
const GL_INTENSITY32F_ARB                 = $8817;
const GL_LUMINANCE32F_ARB                 = $8818;
const GL_LUMINANCE_ALPHA32F_ARB           = $8819;
const GL_RGBA16F_ARB                      = $881A;
const GL_RGB16F_ARB                       = $881B;
const GL_ALPHA16F_ARB                     = $881C;
const GL_INTENSITY16F_ARB                 = $881D;
const GL_LUMINANCE16F_ARB                 = $881E;
const GL_LUMINANCE_ALPHA16F_ARB           = $881F;
{$endc}

{$ifc not undefined GL_ARB_half_float_pixel and GL_ARB_half_float_pixel}
const GL_HALF_FLOAT_ARB                 = $140B;
{$endc}

{$ifc not undefined GL_ARB_color_buffer_float and GL_ARB_color_buffer_float}
const GL_RGBA_FLOAT_MODE_ARB            = $8820;
const GL_CLAMP_VERTEX_COLOR_ARB         = $891A;
const GL_CLAMP_FRAGMENT_COLOR_ARB       = $891B;
const GL_CLAMP_READ_COLOR_ARB           = $891C;
const GL_FIXED_ONLY_ARB                 = $891D;
{$endc}

{$ifc not undefined GL_ARB_half_float_vertex and GL_ARB_half_float_vertex}
const GL_HALF_FLOAT                     = $140B;
{$endc}

{$ifc not undefined GL_ARB_texture_rg and GL_ARB_texture_rg}
const GL_COMPRESSED_RED                 = $8225;
const GL_COMPRESSED_RG                  = $8226;
const GL_RG                             = $8227;
const GL_RG_INTEGER                     = $8228;
const GL_R8                             = $8229;
const GL_R16                            = $822A;
const GL_RG8                            = $822B;
const GL_RG16                           = $822C;
const GL_R16F                           = $822D;
const GL_R32F                           = $822E;
const GL_RG16F                          = $822F;
const GL_RG32F                          = $8230;
const GL_R8I                            = $8231;
const GL_R8UI                           = $8232;
const GL_R16I                           = $8233;
const GL_R16UI                          = $8234;
const GL_R32I                           = $8235;
const GL_R32UI                          = $8236;
const GL_RG8I                           = $8237;
const GL_RG8UI                          = $8238;
const GL_RG16I                          = $8239;
const GL_RG16UI                         = $823A;
const GL_RG32I                          = $823B;
const GL_RG32UI                         = $823C;
{$endc}

{$ifc not undefined GL_EXT_abgr and GL_EXT_abgr}
const GL_ABGR_EXT                       = $8000;
{$endc}

{$ifc not undefined GL_EXT_blend_color and GL_EXT_blend_color}
const GL_CONSTANT_COLOR_EXT             = $8001;
const GL_ONE_MINUS_CONSTANT_COLOR_EXT   = $8002;
const GL_CONSTANT_ALPHA_EXT             = $8003;
const GL_ONE_MINUS_CONSTANT_ALPHA_EXT   = $8004;
const GL_BLEND_COLOR_EXT                = $8005;
{$endc}

{$ifc not undefined GL_EXT_polygon_offset and GL_EXT_polygon_offset}
const GL_POLYGON_OFFSET_EXT             = $8037;
const GL_POLYGON_OFFSET_FACTOR_EXT      = $8038;
const GL_POLYGON_OFFSET_BIAS_EXT        = $8039;
{$endc}

{$ifc not undefined GL_EXT_texture and GL_EXT_texture}
const GL_ALPHA4_EXT                     = $803B;
const GL_ALPHA8_EXT                     = $803C;
const GL_ALPHA12_EXT                    = $803D;
const GL_ALPHA16_EXT                    = $803E;
const GL_LUMINANCE4_EXT                 = $803F;
const GL_LUMINANCE8_EXT                 = $8040;
const GL_LUMINANCE12_EXT                = $8041;
const GL_LUMINANCE16_EXT                = $8042;
const GL_LUMINANCE4_ALPHA4_EXT          = $8043;
const GL_LUMINANCE6_ALPHA2_EXT          = $8044;
const GL_LUMINANCE8_ALPHA8_EXT          = $8045;
const GL_LUMINANCE12_ALPHA4_EXT         = $8046;
const GL_LUMINANCE12_ALPHA12_EXT        = $8047;
const GL_LUMINANCE16_ALPHA16_EXT        = $8048;
const GL_INTENSITY_EXT                  = $8049;
const GL_INTENSITY4_EXT                 = $804A;
const GL_INTENSITY8_EXT                 = $804B;
const GL_INTENSITY12_EXT                = $804C;
const GL_INTENSITY16_EXT                = $804D;
const GL_RGB2_EXT                       = $804E;
const GL_RGB4_EXT                       = $804F;
const GL_RGB5_EXT                       = $8050;
const GL_RGB8_EXT                       = $8051;
const GL_RGB10_EXT                      = $8052;
const GL_RGB12_EXT                      = $8053;
const GL_RGB16_EXT                      = $8054;
const GL_RGBA2_EXT                      = $8055;
const GL_RGBA4_EXT                      = $8056;
const GL_RGB5_A1_EXT                    = $8057;
const GL_RGBA8_EXT                      = $8058;
const GL_RGB10_A2_EXT                   = $8059;
const GL_RGBA12_EXT                     = $805A;
const GL_RGBA16_EXT                     = $805B;
const GL_TEXTURE_RED_SIZE_EXT           = $805C;
const GL_TEXTURE_GREEN_SIZE_EXT         = $805D;
const GL_TEXTURE_BLUE_SIZE_EXT          = $805E;
const GL_TEXTURE_ALPHA_SIZE_EXT         = $805F;
const GL_TEXTURE_LUMINANCE_SIZE_EXT     = $8060;
const GL_TEXTURE_INTENSITY_SIZE_EXT     = $8061;
const GL_REPLACE_EXT                    = $8062;
const GL_PROXY_TEXTURE_1D_EXT           = $8063;
const GL_PROXY_TEXTURE_2D_EXT           = $8064;
const GL_TEXTURE_TOO_LARGE_EXT          = $8065;
{$endc}

{$ifc not undefined GL_EXT_texture3D and GL_EXT_texture3D}
const GL_PACK_SKIP_IMAGES_EXT           = $806B;
const GL_PACK_IMAGE_HEIGHT_EXT          = $806C;
const GL_UNPACK_SKIP_IMAGES_EXT         = $806D;
const GL_UNPACK_IMAGE_HEIGHT_EXT        = $806E;
const GL_TEXTURE_3D_EXT                 = $806F;
const GL_PROXY_TEXTURE_3D_EXT           = $8070;
const GL_TEXTURE_DEPTH_EXT              = $8071;
const GL_TEXTURE_WRAP_R_EXT             = $8072;
const GL_MAX_3D_TEXTURE_SIZE_EXT        = $8073;
{$endc}

{$ifc not undefined GL_EXT_histogram and GL_EXT_histogram}
const GL_HISTOGRAM_EXT                  = $8024;
const GL_PROXY_HISTOGRAM_EXT            = $8025;
const GL_HISTOGRAM_WIDTH_EXT            = $8026;
const GL_HISTOGRAM_FORMAT_EXT           = $8027;
const GL_HISTOGRAM_RED_SIZE_EXT         = $8028;
const GL_HISTOGRAM_GREEN_SIZE_EXT       = $8029;
const GL_HISTOGRAM_BLUE_SIZE_EXT        = $802A;
const GL_HISTOGRAM_ALPHA_SIZE_EXT       = $802B;
const GL_HISTOGRAM_LUMINANCE_SIZE_EXT   = $802C;
const GL_HISTOGRAM_SINK_EXT             = $802D;
const GL_MINMAX_EXT                     = $802E;
const GL_MINMAX_FORMAT_EXT              = $802F;
const GL_MINMAX_SINK_EXT                = $8030;
const GL_TABLE_TOO_LARGE_EXT            = $8031;
{$endc}

{$ifc not undefined GL_EXT_convolution and GL_EXT_convolution}
const GL_CONVOLUTION_1D_EXT             = $8010;
const GL_CONVOLUTION_2D_EXT             = $8011;
const GL_SEPARABLE_2D_EXT               = $8012;
const GL_CONVOLUTION_BORDER_MODE_EXT    = $8013;
const GL_CONVOLUTION_FILTER_SCALE_EXT   = $8014;
const GL_CONVOLUTION_FILTER_BIAS_EXT    = $8015;
const GL_REDUCE_EXT                     = $8016;
const GL_CONVOLUTION_FORMAT_EXT         = $8017;
const GL_CONVOLUTION_WIDTH_EXT          = $8018;
const GL_CONVOLUTION_HEIGHT_EXT         = $8019;
const GL_MAX_CONVOLUTION_WIDTH_EXT      = $801A;
const GL_MAX_CONVOLUTION_HEIGHT_EXT     = $801B;
const GL_POST_CONVOLUTION_RED_SCALE_EXT = $801C;
const GL_POST_CONVOLUTION_GREEN_SCALE_EXT = $801D;
const GL_POST_CONVOLUTION_BLUE_SCALE_EXT = $801E;
const GL_POST_CONVOLUTION_ALPHA_SCALE_EXT = $801F;
const GL_POST_CONVOLUTION_RED_BIAS_EXT  = $8020;
const GL_POST_CONVOLUTION_GREEN_BIAS_EXT = $8021;
const GL_POST_CONVOLUTION_BLUE_BIAS_EXT = $8022;
const GL_POST_CONVOLUTION_ALPHA_BIAS_EXT = $8023;
{$endc}

{$ifc not undefined GL_EXT_cmyka and GL_EXT_cmyka}
const GL_CMYK_EXT                       = $800C;
const GL_CMYKA_EXT                      = $800D;
const GL_PACK_CMYK_HINT_EXT             = $800E;
const GL_UNPACK_CMYK_HINT_EXT           = $800F;
{$endc}

{$ifc not undefined GL_EXT_texture_object and GL_EXT_texture_object}
const GL_TEXTURE_PRIORITY_EXT           = $8066;
const GL_TEXTURE_RESIDENT_EXT           = $8067;
const GL_TEXTURE_1D_BINDING_EXT         = $8068;
const GL_TEXTURE_2D_BINDING_EXT         = $8069;
const GL_TEXTURE_3D_BINDING_EXT         = $806A;
{$endc}

{$ifc not undefined GL_EXT_packed_pixels and GL_EXT_packed_pixels}
const GL_UNSIGNED_BYTE_3_3_2_EXT        = $8032;
const GL_UNSIGNED_SHORT_4_4_4_4_EXT     = $8033;
const GL_UNSIGNED_SHORT_5_5_5_1_EXT     = $8034;
const GL_UNSIGNED_INT_8_8_8_8_EXT       = $8035;
const GL_UNSIGNED_INT_10_10_10_2_EXT    = $8036;
{$endc}

{$ifc not undefined GL_EXT_rescale_normal and GL_EXT_rescale_normal}
const GL_RESCALE_NORMAL_EXT             = $803A;
{$endc}

{$ifc not undefined GL_EXT_vertex_array and GL_EXT_vertex_array}
const GL_VERTEX_ARRAY_EXT               = $8074;
const GL_NORMAL_ARRAY_EXT               = $8075;
const GL_COLOR_ARRAY_EXT                = $8076;
const GL_INDEX_ARRAY_EXT                = $8077;
const GL_TEXTURE_COORD_ARRAY_EXT        = $8078;
const GL_EDGE_FLAG_ARRAY_EXT            = $8079;
const GL_VERTEX_ARRAY_SIZE_EXT          = $807A;
const GL_VERTEX_ARRAY_TYPE_EXT          = $807B;
const GL_VERTEX_ARRAY_STRIDE_EXT        = $807C;
const GL_VERTEX_ARRAY_COUNT_EXT         = $807D;
const GL_NORMAL_ARRAY_TYPE_EXT          = $807E;
const GL_NORMAL_ARRAY_STRIDE_EXT        = $807F;
const GL_NORMAL_ARRAY_COUNT_EXT         = $8080;
const GL_COLOR_ARRAY_SIZE_EXT           = $8081;
const GL_COLOR_ARRAY_TYPE_EXT           = $8082;
const GL_COLOR_ARRAY_STRIDE_EXT         = $8083;
const GL_COLOR_ARRAY_COUNT_EXT          = $8084;
const GL_INDEX_ARRAY_TYPE_EXT           = $8085;
const GL_INDEX_ARRAY_STRIDE_EXT         = $8086;
const GL_INDEX_ARRAY_COUNT_EXT          = $8087;
const GL_TEXTURE_COORD_ARRAY_SIZE_EXT   = $8088;
const GL_TEXTURE_COORD_ARRAY_TYPE_EXT   = $8089;
const GL_TEXTURE_COORD_ARRAY_STRIDE_EXT = $808A;
const GL_TEXTURE_COORD_ARRAY_COUNT_EXT  = $808B;
const GL_EDGE_FLAG_ARRAY_STRIDE_EXT     = $808C;
const GL_EDGE_FLAG_ARRAY_COUNT_EXT      = $808D;
const GL_VERTEX_ARRAY_POINTER_EXT       = $808E;
const GL_NORMAL_ARRAY_POINTER_EXT       = $808F;
const GL_COLOR_ARRAY_POINTER_EXT        = $8090;
const GL_INDEX_ARRAY_POINTER_EXT        = $8091;
const GL_TEXTURE_COORD_ARRAY_POINTER_EXT = $8092;
const GL_EDGE_FLAG_ARRAY_POINTER_EXT    = $8093;
{$endc}

{$ifc not undefined GL_EXT_blend_minmax and GL_EXT_blend_minmax}
const GL_FUNC_ADD_EXT                   = $8006;
const GL_MIN_EXT                        = $8007;
const GL_MAX_EXT                        = $8008;
const GL_BLEND_EQUATION_EXT             = $8009;
{$endc}

{$ifc not undefined GL_EXT_blend_subtract and GL_EXT_blend_subtract}
const GL_FUNC_SUBTRACT_EXT              = $800A;
const GL_FUNC_REVERSE_SUBTRACT_EXT      = $800B;
{$endc}

{$ifc not undefined GL_EXT_paletted_texture and GL_EXT_paletted_texture}
const GL_COLOR_INDEX1_EXT               = $80E2;
const GL_COLOR_INDEX2_EXT               = $80E3;
const GL_COLOR_INDEX4_EXT               = $80E4;
const GL_COLOR_INDEX8_EXT               = $80E5;
const GL_COLOR_INDEX12_EXT              = $80E6;
const GL_COLOR_INDEX16_EXT              = $80E7;
const GL_TEXTURE_INDEX_SIZE_EXT         = $80ED;
{$endc}

{$ifc not undefined GL_EXT_clip_volume_hint and GL_EXT_clip_volume_hint}
const GL_CLIP_VOLUME_CLIPPING_HINT_EXT  = $80F0;
{$endc}

{$ifc not undefined GL_EXT_index_material and GL_EXT_index_material}
const GL_INDEX_MATERIAL_EXT             = $81B8;
const GL_INDEX_MATERIAL_PARAMETER_EXT   = $81B9;
const GL_INDEX_MATERIAL_FACE_EXT        = $81BA;
{$endc}

{$ifc not undefined GL_EXT_index_func and GL_EXT_index_func}
const GL_INDEX_TEST_EXT                 = $81B5;
const GL_INDEX_TEST_FUNC_EXT            = $81B6;
const GL_INDEX_TEST_REF_EXT             = $81B7;
{$endc}

{$ifc not undefined GL_EXT_index_array_formats and GL_EXT_index_array_formats}
const GL_IUI_V2F_EXT                    = $81AD;
const GL_IUI_V3F_EXT                    = $81AE;
const GL_IUI_N3F_V2F_EXT                = $81AF;
const GL_IUI_N3F_V3F_EXT                = $81B0;
const GL_T2F_IUI_V2F_EXT                = $81B1;
const GL_T2F_IUI_V3F_EXT                = $81B2;
const GL_T2F_IUI_N3F_V2F_EXT            = $81B3;
const GL_T2F_IUI_N3F_V3F_EXT            = $81B4;
{$endc}

{$ifc not undefined GL_EXT_compiled_vertex_array and GL_EXT_compiled_vertex_array}
const GL_ARRAY_ELEMENT_LOCK_FIRST_EXT   = $81A8;
const GL_ARRAY_ELEMENT_LOCK_COUNT_EXT   = $81A9;
{$endc}

{$ifc not undefined GL_EXT_cull_vertex and GL_EXT_cull_vertex}
const GL_CULL_VERTEX_EXT                = $81AA;
const GL_CULL_VERTEX_EYE_POSITION_EXT   = $81AB;
const GL_CULL_VERTEX_OBJECT_POSITION_EXT = $81AC;
{$endc}

{$ifc not undefined GL_EXT_draw_range_elements and GL_EXT_draw_range_elements}
const GL_MAX_ELEMENTS_VERTICES_EXT      = $80E8;
const GL_MAX_ELEMENTS_INDICES_EXT       = $80E9;
{$endc}

{$ifc not undefined GL_EXT_light_texture and GL_EXT_light_texture}
const GL_FRAGMENT_MATERIAL_EXT          = $8349;
const GL_FRAGMENT_NORMAL_EXT            = $834A;
const GL_FRAGMENT_COLOR_EXT             = $834C;
const GL_ATTENUATION_EXT                = $834D;
const GL_SHADOW_ATTENUATION_EXT         = $834E;
const GL_TEXTURE_APPLICATION_MODE_EXT   = $834F;
const GL_TEXTURE_LIGHT_EXT              = $8350;
const GL_TEXTURE_MATERIAL_FACE_EXT      = $8351;
const GL_TEXTURE_MATERIAL_PARAMETER_EXT = $8352;
{ reuse GL_FRAGMENT_DEPTH_EXT }
{$endc}

{$ifc not undefined GL_EXT_bgra and GL_EXT_bgra}
const GL_BGR_EXT                        = $80E0;
const GL_BGRA_EXT                       = $80E1;
{$endc}

{$ifc not undefined GL_EXT_pixel_transform and GL_EXT_pixel_transform}
const GL_PIXEL_TRANSFORM_2D_EXT         = $8330;
const GL_PIXEL_MAG_FILTER_EXT           = $8331;
const GL_PIXEL_MIN_FILTER_EXT           = $8332;
const GL_PIXEL_CUBIC_WEIGHT_EXT         = $8333;
const GL_CUBIC_EXT                      = $8334;
const GL_AVERAGE_EXT                    = $8335;
const GL_PIXEL_TRANSFORM_2D_STACK_DEPTH_EXT = $8336;
const GL_MAX_PIXEL_TRANSFORM_2D_STACK_DEPTH_EXT = $8337;
const GL_PIXEL_TRANSFORM_2D_MATRIX_EXT  = $8338;
{$endc}

{$ifc not undefined GL_EXT_shared_texture_palette and GL_EXT_shared_texture_palette}
const GL_SHARED_TEXTURE_PALETTE_EXT     = $81FB;
{$endc}

{$ifc not undefined GL_EXT_separate_specular_color and GL_EXT_separate_specular_color}
const GL_LIGHT_MODEL_COLOR_CONTROL_EXT  = $81F8;
const GL_SINGLE_COLOR_EXT               = $81F9;
const GL_SEPARATE_SPECULAR_COLOR_EXT    = $81FA;
{$endc}

{$ifc not undefined GL_EXT_secondary_color and GL_EXT_secondary_color}
const GL_COLOR_SUM_EXT                  = $8458;
const GL_CURRENT_SECONDARY_COLOR_EXT    = $8459;
const GL_SECONDARY_COLOR_ARRAY_SIZE_EXT = $845A;
const GL_SECONDARY_COLOR_ARRAY_TYPE_EXT = $845B;
const GL_SECONDARY_COLOR_ARRAY_STRIDE_EXT = $845C;
const GL_SECONDARY_COLOR_ARRAY_POINTER_EXT = $845D;
const GL_SECONDARY_COLOR_ARRAY_EXT      = $845E;
{$endc}

{$ifc not undefined GL_EXT_texture_perturb_normal and GL_EXT_texture_perturb_normal}
const GL_PERTURB_EXT                    = $85AE;
const GL_TEXTURE_NORMAL_EXT             = $85AF;
{$endc}

{$ifc not undefined GL_EXT_fog_coord and GL_EXT_fog_coord}
const GL_FOG_COORDINATE_SOURCE_EXT      = $8450;
const GL_FOG_COORDINATE_EXT             = $8451;
const GL_FRAGMENT_DEPTH_EXT             = $8452;
const GL_CURRENT_FOG_COORDINATE_EXT     = $8453;
const GL_FOG_COORDINATE_ARRAY_TYPE_EXT  = $8454;
const GL_FOG_COORDINATE_ARRAY_STRIDE_EXT = $8455;
const GL_FOG_COORDINATE_ARRAY_POINTER_EXT = $8456;
const GL_FOG_COORDINATE_ARRAY_EXT       = $8457;
{$endc}

{$ifc not undefined GL_EXT_coordinate_frame and GL_EXT_coordinate_frame}
const GL_TANGENT_ARRAY_EXT              = $8439;
const GL_BINORMAL_ARRAY_EXT             = $843A;
const GL_CURRENT_TANGENT_EXT            = $843B;
const GL_CURRENT_BINORMAL_EXT           = $843C;
const GL_TANGENT_ARRAY_TYPE_EXT         = $843E;
const GL_TANGENT_ARRAY_STRIDE_EXT       = $843F;
const GL_BINORMAL_ARRAY_TYPE_EXT        = $8440;
const GL_BINORMAL_ARRAY_STRIDE_EXT      = $8441;
const GL_TANGENT_ARRAY_POINTER_EXT      = $8442;
const GL_BINORMAL_ARRAY_POINTER_EXT     = $8443;
const GL_MAP1_TANGENT_EXT               = $8444;
const GL_MAP2_TANGENT_EXT               = $8445;
const GL_MAP1_BINORMAL_EXT              = $8446;
const GL_MAP2_BINORMAL_EXT              = $8447;
{$endc}

{$ifc not undefined GL_EXT_texture_env_combine and GL_EXT_texture_env_combine}
const GL_COMBINE_EXT                    = $8570;
const GL_COMBINE_RGB_EXT                = $8571;
const GL_COMBINE_ALPHA_EXT              = $8572;
const GL_RGB_SCALE_EXT                  = $8573;
const GL_ADD_SIGNED_EXT                 = $8574;
const GL_INTERPOLATE_EXT                = $8575;
const GL_CONSTANT_EXT                   = $8576;
const GL_PRIMARY_COLOR_EXT              = $8577;
const GL_PREVIOUS_EXT                   = $8578;
const GL_SOURCE0_RGB_EXT                = $8580;
const GL_SOURCE1_RGB_EXT                = $8581;
const GL_SOURCE2_RGB_EXT                = $8582;
const GL_SOURCE3_RGB_EXT                = $8583;
const GL_SOURCE4_RGB_EXT                = $8584;
const GL_SOURCE5_RGB_EXT                = $8585;
const GL_SOURCE6_RGB_EXT                = $8586;
const GL_SOURCE7_RGB_EXT                = $8587;
const GL_SOURCE0_ALPHA_EXT              = $8588;
const GL_SOURCE1_ALPHA_EXT              = $8589;
const GL_SOURCE2_ALPHA_EXT              = $858A;
const GL_SOURCE3_ALPHA_EXT              = $858B;
const GL_SOURCE4_ALPHA_EXT              = $858C;
const GL_SOURCE5_ALPHA_EXT              = $858D;
const GL_SOURCE6_ALPHA_EXT              = $858E;
const GL_SOURCE7_ALPHA_EXT              = $858F;
const GL_OPERAND0_RGB_EXT               = $8590;
const GL_OPERAND1_RGB_EXT               = $8591;
const GL_OPERAND2_RGB_EXT               = $8592;
const GL_OPERAND3_RGB_EXT               = $8593;
const GL_OPERAND4_RGB_EXT               = $8594;
const GL_OPERAND5_RGB_EXT               = $8595;
const GL_OPERAND6_RGB_EXT               = $8596;
const GL_OPERAND7_RGB_EXT               = $8597;
const GL_OPERAND0_ALPHA_EXT             = $8598;
const GL_OPERAND1_ALPHA_EXT             = $8599;
const GL_OPERAND2_ALPHA_EXT             = $859A;
const GL_OPERAND3_ALPHA_EXT             = $859B;
const GL_OPERAND4_ALPHA_EXT             = $859C;
const GL_OPERAND5_ALPHA_EXT             = $859D;
const GL_OPERAND6_ALPHA_EXT             = $859E;
const GL_OPERAND7_ALPHA_EXT             = $859F;
{$endc}

{$ifc not undefined GL_EXT_blend_func_separate and GL_EXT_blend_func_separate}
const GL_BLEND_DST_RGB_EXT              = $80C8;
const GL_BLEND_SRC_RGB_EXT              = $80C9;
const GL_BLEND_DST_ALPHA_EXT            = $80CA;
const GL_BLEND_SRC_ALPHA_EXT            = $80CB;
{$endc}

{$ifc not undefined GL_EXT_stencil_wrap and GL_EXT_stencil_wrap}
const GL_INCR_WRAP_EXT                  = $8507;
const GL_DECR_WRAP_EXT                  = $8508;
{$endc}

{$ifc not undefined GL_EXT_422_pixels and GL_EXT_422_pixels}
const GL_422_EXT                        = $80CC;
const GL_422_REV_EXT                    = $80CD;
const GL_422_AVERAGE_EXT                = $80CE;
const GL_422_REV_AVERAGE_EXT            = $80CF;
{$endc}

{$ifc not undefined GL_EXT_texture_cube_map and GL_EXT_texture_cube_map}
const GL_NORMAL_MAP_EXT                 = $8511;
const GL_REFLECTION_MAP_EXT             = $8512;
const GL_TEXTURE_CUBE_MAP_EXT           = $8513;
const GL_TEXTURE_BINDING_CUBE_MAP_EXT   = $8514;
const GL_TEXTURE_CUBE_MAP_POSITIVE_X_EXT = $8515;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_X_EXT = $8516;
const GL_TEXTURE_CUBE_MAP_POSITIVE_Y_EXT = $8517;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_EXT = $8518;
const GL_TEXTURE_CUBE_MAP_POSITIVE_Z_EXT = $8519;
const GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_EXT = $851A;
const GL_PROXY_TEXTURE_CUBE_MAP_EXT     = $851B;
const GL_MAX_CUBE_MAP_TEXTURE_SIZE_EXT  = $851C;
{$endc}

{$ifc not undefined GL_EXT_texture_lod_bias and GL_EXT_texture_lod_bias}
const GL_MAX_TEXTURE_LOD_BIAS_EXT       = $84FD;
const GL_TEXTURE_FILTER_CONTROL_EXT     = $8500;
const GL_TEXTURE_LOD_BIAS_EXT           = $8501;
{$endc}

{$ifc not undefined GL_EXT_texture_filter_anisotropic and GL_EXT_texture_filter_anisotropic}
const GL_TEXTURE_MAX_ANISOTROPY_EXT     = $84FE;
const GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT = $84FF;
{$endc}

{$ifc not undefined GL_EXT_vertex_weighting and GL_EXT_vertex_weighting}
const GL_MODELVIEW0_STACK_DEPTH_EXT     = GL_MODELVIEW_STACK_DEPTH;
const GL_MODELVIEW1_STACK_DEPTH_EXT     = $8502;
const GL_MODELVIEW0_MATRIX_EXT          = GL_MODELVIEW_MATRIX;
const GL_MODELVIEW_MATRIX1_EXT          = $8506;
const GL_VERTEX_WEIGHTING_EXT           = $8509;
const GL_MODELVIEW0_EXT                 = GL_MODELVIEW;
const GL_MODELVIEW1_EXT                 = $850A;
const GL_CURRENT_VERTEX_WEIGHT_EXT      = $850B;
const GL_VERTEX_WEIGHT_ARRAY_EXT        = $850C;
const GL_VERTEX_WEIGHT_ARRAY_SIZE_EXT   = $850D;
const GL_VERTEX_WEIGHT_ARRAY_TYPE_EXT   = $850E;
const GL_VERTEX_WEIGHT_ARRAY_STRIDE_EXT = $850F;
const GL_VERTEX_WEIGHT_ARRAY_POINTER_EXT = $8510;
{$endc}

{$ifc not undefined GL_EXT_texture_compression_s3tc and GL_EXT_texture_compression_s3tc}
const GL_COMPRESSED_RGB_S3TC_DXT1_EXT   = $83F0;
const GL_COMPRESSED_RGBA_S3TC_DXT1_EXT  = $83F1;
const GL_COMPRESSED_RGBA_S3TC_DXT3_EXT  = $83F2;
const GL_COMPRESSED_RGBA_S3TC_DXT5_EXT  = $83F3;
{$endc}

{$ifc not undefined GL_EXT_texture_rectangle and GL_EXT_texture_rectangle}
const GL_TEXTURE_RECTANGLE_EXT          = $84F5;
const GL_TEXTURE_BINDING_RECTANGLE_EXT  = $84F6;
const GL_PROXY_TEXTURE_RECTANGLE_EXT    = $84F7;
const GL_MAX_RECTANGLE_TEXTURE_SIZE_EXT = $84F8;
{$endc}

{$ifc not undefined GL_EXT_texture_sRGB and GL_EXT_texture_sRGB}
const GL_SRGB_EXT                            = $8C40;
const GL_SRGB8_EXT                           = $8C41;
const GL_SRGB_ALPHA_EXT                      = $8C42;
const GL_SRGB8_ALPHA8_EXT                    = $8C43;
const GL_SLUMINANCE_ALPHA_EXT                = $8C44;
const GL_SLUMINANCE8_ALPHA8_EXT              = $8C45;
const GL_SLUMINANCE_EXT                      = $8C46;
const GL_SLUMINANCE8_EXT                     = $8C47;
const GL_COMPRESSED_SRGB_EXT                 = $8C48;
const GL_COMPRESSED_SRGB_ALPHA_EXT           = $8C49;
const GL_COMPRESSED_SLUMINANCE_EXT           = $8C4A;
const GL_COMPRESSED_SLUMINANCE_ALPHA_EXT     = $8C4B;
const GL_COMPRESSED_SRGB_S3TC_DXT1_EXT       = $8C4C;
const GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT = $8C4D;
const GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT = $8C4E;
const GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT = $8C4F;
{$endc}

{$ifc not undefined GL_EXT_vertex_shader and GL_EXT_vertex_shader}
const GL_VERTEX_SHADER_EXT              = $8780;
const GL_VARIANT_VALUE_EXT              = $87E4;
const GL_VARIANT_DATATYPE_EXT           = $87E5;
const GL_VARIANT_ARRAY_STRIDE_EXT       = $87E6;
const GL_VARIANT_ARRAY_TYPE_EXT         = $87E7;
const GL_VARIANT_ARRAY_EXT              = $87E8;
const GL_VARIANT_ARRAY_POINTER_EXT      = $87E9;
const GL_INVARIANT_VALUE_EXT            = $87EA;
const GL_INVARIANT_DATATYPE_EXT         = $87EB;
const GL_LOCAL_CONSTANT_VALUE_EXT       = $87EC;
const GL_LOCAL_CONSTANT_DATATYPE_EXT    = $87Ed;
const GL_OP_INDEX_EXT                   = $8782;
const GL_OP_NEGATE_EXT                  = $8783;
const GL_OP_DOT3_EXT                    = $8784;
const GL_OP_DOT4_EXT                    = $8785;
const GL_OP_MUL_EXT                     = $8786;
const GL_OP_ADD_EXT                     = $8787;
const GL_OP_MADD_EXT                    = $8788;
const GL_OP_FRAC_EXT                    = $8789;
const GL_OP_MAX_EXT                     = $878A;
const GL_OP_MIN_EXT                     = $878B;
const GL_OP_SET_GE_EXT                  = $878C;
const GL_OP_SET_LT_EXT                  = $878D;
const GL_OP_CLAMP_EXT                   = $878E;
const GL_OP_FLOOR_EXT                   = $878F;
const GL_OP_ROUND_EXT                   = $8790;
const GL_OP_EXP_BASE_2_EXT              = $8791;
const GL_OP_LOG_BASE_2_EXT              = $8792;
const GL_OP_POWER_EXT                   = $8793;
const GL_OP_RECIP_EXT                   = $8794;
const GL_OP_RECIP_SQRT_EXT              = $8795;
const GL_OP_SUB_EXT                     = $8796;
const GL_OP_CROSS_PRODUCT_EXT           = $8797;
const GL_OP_MULTIPLY_MATRIX_EXT         = $8798;
const GL_OP_MOV_EXT                     = $8799;
const GL_OUTPUT_VERTEX_EXT              = $879A;
const GL_OUTPUT_COLOR0_EXT              = $879B;
const GL_OUTPUT_COLOR1_EXT              = $879C;
const GL_OUTPUT_TEXTURE_COORD0_EXT      = $879D;
const GL_OUTPUT_TEXTURE_COORD1_EXT      = $879E;
const GL_OUTPUT_TEXTURE_COORD2_EXT      = $879F;
const GL_OUTPUT_TEXTURE_COORD3_EXT      = $87A0;
const GL_OUTPUT_TEXTURE_COORD4_EXT      = $87A1;
const GL_OUTPUT_TEXTURE_COORD5_EXT      = $87A2;
const GL_OUTPUT_TEXTURE_COORD6_EXT      = $87A3;
const GL_OUTPUT_TEXTURE_COORD7_EXT      = $87A4;
const GL_OUTPUT_TEXTURE_COORD8_EXT      = $87A5;
const GL_OUTPUT_TEXTURE_COORD9_EXT      = $87A6;
const GL_OUTPUT_TEXTURE_COORD10_EXT     = $87A7;
const GL_OUTPUT_TEXTURE_COORD11_EXT     = $87A8;
const GL_OUTPUT_TEXTURE_COORD12_EXT     = $87A9;
const GL_OUTPUT_TEXTURE_COORD13_EXT     = $87AA;
const GL_OUTPUT_TEXTURE_COORD14_EXT     = $87AB;
const GL_OUTPUT_TEXTURE_COORD15_EXT     = $87AC;
const GL_OUTPUT_TEXTURE_COORD16_EXT     = $87AD;
const GL_OUTPUT_TEXTURE_COORD17_EXT     = $87AE;
const GL_OUTPUT_TEXTURE_COORD18_EXT     = $87AF;
const GL_OUTPUT_TEXTURE_COORD19_EXT     = $87B0;
const GL_OUTPUT_TEXTURE_COORD20_EXT     = $87B1;
const GL_OUTPUT_TEXTURE_COORD21_EXT     = $87B2;
const GL_OUTPUT_TEXTURE_COORD22_EXT     = $87B3;
const GL_OUTPUT_TEXTURE_COORD23_EXT     = $87B4;
const GL_OUTPUT_TEXTURE_COORD24_EXT     = $87B5;
const GL_OUTPUT_TEXTURE_COORD25_EXT     = $87B6;
const GL_OUTPUT_TEXTURE_COORD26_EXT     = $87B7;
const GL_OUTPUT_TEXTURE_COORD27_EXT     = $87B8;
const GL_OUTPUT_TEXTURE_COORD28_EXT     = $87B9;
const GL_OUTPUT_TEXTURE_COORD29_EXT     = $87BA;
const GL_OUTPUT_TEXTURE_COORD30_EXT     = $87BB;
const GL_OUTPUT_TEXTURE_COORD31_EXT     = $87BC;
const GL_OUTPUT_FOG_EXT                 = $87BD;
const GL_SCALAR_EXT                     = $87BE;
const GL_VECTOR_EXT                     = $87BF;
const GL_MATRIX_EXT                     = $87C0;
const GL_VARIANT_EXT                    = $87C1;
const GL_INVARIANT_EXT                  = $87C2;
const GL_LOCAL_CONSTANT_EXT             = $87C3;
const GL_LOCAL_EXT                      = $87C4;
const GL_MAX_VERTEX_SHADER_INSTRUCTIONS_EXT              = $87C5;
const GL_MAX_VERTEX_SHADER_VARIANTS_EXT                  = $87C6;
const GL_MAX_VERTEX_SHADER_INVARIANTS_EXT                = $87C7;
const GL_MAX_VERTEX_SHADER_LOCAL_CONSTANTS_EXT           = $87C8;
const GL_MAX_VERTEX_SHADER_LOCALS_EXT                    = $87C9;
const GL_MAX_OPTIMIZED_VERTEX_SHADER_INSTRUCTIONS_EXT    = $87CA;
const GL_MAX_OPTIMIZED_VERTEX_SHADER_VARIANTS_EXT        = $87CB;
const GL_MAX_OPTIMIZED_VERTEX_SHADER_LOCAL_CONSTANTS_EXT = $87CC;
const GL_MAX_OPTIMIZED_VERTEX_SHADER_INVARIANTS_EXT      = $87CD;
const GL_MAX_OPTIMIZED_VERTEX_SHADER_LOCALS_EXT          = $87CE;
const GL_VERTEX_SHADER_INSTRUCTIONS_EXT                  = $87CF;
const GL_VERTEX_SHADER_VARIANTS_EXT                      = $87D0;
const GL_VERTEX_SHADER_INVARIANTS_EXT                    = $87D1;
const GL_VERTEX_SHADER_LOCAL_CONSTANTS_EXT               = $87D2;
const GL_VERTEX_SHADER_LOCALS_EXT                        = $87D3;
const GL_VERTEX_SHADER_BINDING_EXT                       = $8781;
const GL_VERTEX_SHADER_OPTIMIZED_EXT                     = $87D4;
const GL_X_EXT                          = $87D5;
const GL_Y_EXT                          = $87D6;
const GL_Z_EXT                          = $87D7;
const GL_W_EXT                          = $87D8;
const GL_NEGATIVE_X_EXT                 = $87D9;
const GL_NEGATIVE_Y_EXT                 = $87DA;
const GL_NEGATIVE_Z_EXT                 = $87DB;
const GL_NEGATIVE_W_EXT                 = $87DC;
const GL_NEGATIVE_ONE_EXT               = $87DF;
const GL_NORMALIZED_RANGE_EXT           = $87E0;
const GL_FULL_RANGE_EXT                 = $87E1;
const GL_CURRENT_VERTEX_EXT             = $87E2;
const GL_MVP_MATRIX_EXT                 = $87E3;
{$endc}

{$ifc not undefined GL_EXT_fragment_shader and GL_EXT_fragment_shader}
const GL_FRAGMENT_SHADER_EXT            = $8920;
const GL_REG_0_EXT                      = $8921;
const GL_REG_1_EXT                      = $8922;
const GL_REG_2_EXT                      = $8923;
const GL_REG_3_EXT                      = $8924;
const GL_REG_4_EXT                      = $8925;
const GL_REG_5_EXT                      = $8926;
const GL_REG_6_EXT                      = $8927;
const GL_REG_7_EXT                      = $8928;
const GL_REG_8_EXT                      = $8929;
const GL_REG_9_EXT                      = $892A;
const GL_REG_10_EXT                     = $892B;
const GL_REG_11_EXT                     = $892C;
const GL_REG_12_EXT                     = $892D;
const GL_REG_13_EXT                     = $892E;
const GL_REG_14_EXT                     = $892F;
const GL_REG_15_EXT                     = $8930;
const GL_REG_16_EXT                     = $8931;
const GL_REG_17_EXT                     = $8932;
const GL_REG_18_EXT                     = $8933;
const GL_REG_19_EXT                     = $8934;
const GL_REG_20_EXT                     = $8935;
const GL_REG_21_EXT                     = $8936;
const GL_REG_22_EXT                     = $8937;
const GL_REG_23_EXT                     = $8938;
const GL_REG_24_EXT                     = $8939;
const GL_REG_25_EXT                     = $893A;
const GL_REG_26_EXT                     = $893B;
const GL_REG_27_EXT                     = $893C;
const GL_REG_28_EXT                     = $893D;
const GL_REG_29_EXT                     = $893E;
const GL_REG_30_EXT                     = $893F;
const GL_REG_31_EXT                     = $8940;
const GL_CON_0_EXT                      = $8941;
const GL_CON_1_EXT                      = $8942;
const GL_CON_2_EXT                      = $8943;
const GL_CON_3_EXT                      = $8944;
const GL_CON_4_EXT                      = $8945;
const GL_CON_5_EXT                      = $8946;
const GL_CON_6_EXT                      = $8947;
const GL_CON_7_EXT                      = $8948;
const GL_CON_8_EXT                      = $8949;
const GL_CON_9_EXT                      = $894A;
const GL_CON_10_EXT                     = $894B;
const GL_CON_11_EXT                     = $894C;
const GL_CON_12_EXT                     = $894D;
const GL_CON_13_EXT                     = $894E;
const GL_CON_14_EXT                     = $894F;
const GL_CON_15_EXT                     = $8950;
const GL_CON_16_EXT                     = $8951;
const GL_CON_17_EXT                     = $8952;
const GL_CON_18_EXT                     = $8953;
const GL_CON_19_EXT                     = $8954;
const GL_CON_20_EXT                     = $8955;
const GL_CON_21_EXT                     = $8956;
const GL_CON_22_EXT                     = $8957;
const GL_CON_23_EXT                     = $8958;
const GL_CON_24_EXT                     = $8959;
const GL_CON_25_EXT                     = $895A;
const GL_CON_26_EXT                     = $895B;
const GL_CON_27_EXT                     = $895C;
const GL_CON_28_EXT                     = $895D;
const GL_CON_29_EXT                     = $895E;
const GL_CON_30_EXT                     = $895F;
const GL_CON_31_EXT                     = $8960;
const GL_MOV_EXT                        = $8961;
const GL_ADD_EXT                        = $8963;
const GL_MUL_EXT                        = $8964;
const GL_SUB_EXT                        = $8965;
const GL_DOT3_EXT                       = $8966;
const GL_DOT4_EXT                       = $8967;
const GL_MAD_EXT                        = $8968;
const GL_LERP_EXT                       = $8969;
const GL_CND_EXT                        = $896A;
const GL_CND0_EXT                       = $896B;
const GL_DOT2_ADD_EXT                   = $896C;
const GL_SECONDARY_INTERPOLATOR_EXT     = $896D;
const GL_NUM_FRAGMENT_REGISTERS_EXT     = $896E;
const GL_NUM_FRAGMENT_CONSTANTS_EXT     = $896F;
const GL_NUM_PASSES_EXT                 = $8970;
const GL_NUM_INSTRUCTIONS_PER_PASS_EXT  = $8971;
const GL_NUM_INSTRUCTIONS_TOTAL_EXT     = $8972;
const GL_NUM_INPUT_INTERPOLATOR_COMPONENTS_EXT = $8973;
const GL_NUM_LOOPBACK_COMPONENTS_EXT    = $8974;
const GL_COLOR_ALPHA_PAIRING_EXT        = $8975;
const GL_SWIZZLE_STR_EXT                = $8976;
const GL_SWIZZLE_STQ_EXT                = $8977;
const GL_SWIZZLE_STR_DR_EXT             = $8978;
const GL_SWIZZLE_STQ_DQ_EXT             = $8979;
const GL_SWIZZLE_STRQ_EXT               = $897A;
const GL_SWIZZLE_STRQ_DQ_EXT            = $897B;
const GL_RED_BIT_EXT                    = $00000001;
const GL_GREEN_BIT_EXT                  = $00000002;
const GL_BLUE_BIT_EXT                   = $00000004;
const GL_2X_BIT_EXT                     = $00000001;
const GL_4X_BIT_EXT                     = $00000002;
const GL_8X_BIT_EXT                     = $00000004;
const GL_HALF_BIT_EXT                   = $00000008;
const GL_QUARTER_BIT_EXT                = $00000010;
const GL_EIGHTH_BIT_EXT                 = $00000020;
const GL_SATURATE_BIT_EXT               = $00000040;
const GL_COMP_BIT_EXT                   = $00000002;
const GL_NEGATE_BIT_EXT                 = $00000004;
const GL_BIAS_BIT_EXT                   = $00000008;
{$endc}

{$ifc not undefined GL_EXT_multisample and GL_EXT_multisample}
const GL_MULTISAMPLE_EXT                = $809D;
const GL_SAMPLE_ALPHA_TO_MASK_EXT       = $809E;
const GL_SAMPLE_ALPHA_TO_ONE_EXT        = $809F;
const GL_SAMPLE_MASK_EXT                = $80A0;
const GL_1PASS_EXT                      = $80A1;
const GL_2PASS_0_EXT                    = $80A2;
const GL_2PASS_1_EXT                    = $80A3;
const GL_4PASS_0_EXT                    = $80A4;
const GL_4PASS_1_EXT                    = $80A5;
const GL_4PASS_2_EXT                    = $80A6;
const GL_4PASS_3_EXT                    = $80A7;
const GL_SAMPLE_BUFFERS_EXT             = $80A8;
const GL_SAMPLES_EXT                    = $80A9;
const GL_SAMPLE_MASK_VALUE_EXT          = $80AA;
const GL_SAMPLE_MASK_INVERT_EXT         = $80AB;
const GL_SAMPLE_PATTERN_EXT             = $80AC;
{$endc}

{$ifc not undefined GL_EXT_stencil_two_side and GL_EXT_stencil_two_side}
const GL_STENCIL_TEST_TWO_SIDE_EXT      = $8910;
const GL_ACTIVE_STENCIL_FACE_EXT        = $8911;
{$endc}

{$ifc not undefined GL_EXT_depth_bounds_test and GL_EXT_depth_bounds_test}
const GL_DEPTH_BOUNDS_TEST_EXT          = $8890;
const GL_DEPTH_BOUNDS_EXT               = $8891;
{$endc}

{$ifc not undefined GL_EXT_blend_equation_separate and GL_EXT_blend_equation_separate}
const GL_BLEND_EQUATION_RGB_EXT         = $8009;
const GL_BLEND_EQUATION_ALPHA_EXT       = $883D;
{$endc}

{$ifc not undefined GL_EXT_texture_mirror_clamp and GL_EXT_texture_mirror_clamp}
const GL_MIRROR_CLAMP_EXT               = $8742;
const GL_MIRROR_CLAMP_TO_EDGE_EXT       = $8743;
const GL_MIRROR_CLAMP_TO_BORDER_EXT     = $8912;
{$endc}

{$ifc not undefined GL_EXT_framebuffer_object and GL_EXT_framebuffer_object}
const GL_FRAMEBUFFER_EXT                 = $8D40;
const GL_RENDERBUFFER_EXT                = $8D41;
const GL_STENCIL_INDEX1_EXT              = $8D46;
const GL_STENCIL_INDEX4_EXT              = $8D47;
const GL_STENCIL_INDEX8_EXT              = $8D48;
const GL_STENCIL_INDEX16_EXT             = $8D49;
const GL_RENDERBUFFER_WIDTH_EXT           = $8D42;
const GL_RENDERBUFFER_HEIGHT_EXT          = $8D43;
const GL_RENDERBUFFER_INTERNAL_FORMAT_EXT = $8D44;
const GL_RENDERBUFFER_RED_SIZE_EXT        = $8D50;
const GL_RENDERBUFFER_GREEN_SIZE_EXT      = $8D51;
const GL_RENDERBUFFER_BLUE_SIZE_EXT       = $8D52;
const GL_RENDERBUFFER_ALPHA_SIZE_EXT      = $8D53;
const GL_RENDERBUFFER_DEPTH_SIZE_EXT      = $8D54;
const GL_RENDERBUFFER_STENCIL_SIZE_EXT    = $8D55;
const GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT           = $8CD0;
const GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME_EXT           = $8CD1;
const GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL_EXT         = $8CD2;
const GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE_EXT = $8CD3;
const GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_EXT    = $8CD4;
const GL_COLOR_ATTACHMENT0_EXT           = $8CE0;
const GL_COLOR_ATTACHMENT1_EXT           = $8CE1;
const GL_COLOR_ATTACHMENT2_EXT           = $8CE2;
const GL_COLOR_ATTACHMENT3_EXT           = $8CE3;
const GL_COLOR_ATTACHMENT4_EXT           = $8CE4;
const GL_COLOR_ATTACHMENT5_EXT           = $8CE5;
const GL_COLOR_ATTACHMENT6_EXT           = $8CE6;
const GL_COLOR_ATTACHMENT7_EXT           = $8CE7;
const GL_COLOR_ATTACHMENT8_EXT           = $8CE8;
const GL_COLOR_ATTACHMENT9_EXT           = $8CE9;
const GL_COLOR_ATTACHMENT10_EXT          = $8CEA;
const GL_COLOR_ATTACHMENT11_EXT          = $8CEB;
const GL_COLOR_ATTACHMENT12_EXT          = $8CEC;
const GL_COLOR_ATTACHMENT13_EXT          = $8CED;
const GL_COLOR_ATTACHMENT14_EXT          = $8CEE;
const GL_COLOR_ATTACHMENT15_EXT          = $8CEF;
const GL_DEPTH_ATTACHMENT_EXT            = $8D00;
const GL_STENCIL_ATTACHMENT_EXT          = $8D20;
const GL_FRAMEBUFFER_COMPLETE_EXT                        = $8CD5;
const GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT           = $8CD6;
const GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT   = $8CD7;
const GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT           = $8CD9;
const GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT              = $8CDA;
const GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT          = $8CDB;
const GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT          = $8CDC;
const GL_FRAMEBUFFER_UNSUPPORTED_EXT                     = $8CDD;
const GL_FRAMEBUFFER_BINDING_EXT         = $8CA6;
const GL_RENDERBUFFER_BINDING_EXT        = $8CA7;
const GL_MAX_COLOR_ATTACHMENTS_EXT       = $8CDF;
const GL_MAX_RENDERBUFFER_SIZE_EXT       = $84E8;
const GL_INVALID_FRAMEBUFFER_OPERATION_EXT = $0506;
{$endc}

{$ifc not undefined GL_EXT_framebuffer_blit and GL_EXT_framebuffer_blit}
const GL_READ_FRAMEBUFFER_EXT                 = $8CA8;
const GL_DRAW_FRAMEBUFFER_EXT                 = $8CA9;
const GL_DRAW_FRAMEBUFFER_BINDING_EXT         = $8CA6;
const GL_READ_FRAMEBUFFER_BINDING_EXT         = $8CAA;
{$endc}

{$ifc not undefined GL_EXT_framebuffer_multisample and GL_EXT_framebuffer_multisample}
const GL_RENDERBUFFER_SAMPLES_EXT               = $8CAB;
const GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE_EXT = $8D56;
const GL_MAX_SAMPLES_EXT                        = $8D57;
{$endc}

{$ifc not undefined GL_EXT_packed_depth_stencil and GL_EXT_packed_depth_stencil}
const GL_DEPTH_STENCIL_EXT                    = $84F9;
const GL_UNSIGNED_INT_24_8_EXT                = $84FA;
const GL_DEPTH24_STENCIL8_EXT                 = $88F0;
const GL_TEXTURE_STENCIL_SIZE_EXT             = $88F1;
{$endc}

{$ifc not undefined GL_EXT_geometry_shader4 and GL_EXT_geometry_shader4}
const GL_GEOMETRY_SHADER_EXT                      = $8DD9;
const GL_GEOMETRY_VERTICES_OUT_EXT                = $8DDA;
const GL_GEOMETRY_INPUT_TYPE_EXT                  = $8DDB;
const GL_GEOMETRY_OUTPUT_TYPE_EXT                 = $8DDC;
const GL_MAX_GEOMETRY_TEXTURE_IMAGE_UNITS_EXT     = $8C29;
const GL_MAX_GEOMETRY_VARYING_COMPONENTS_EXT      = $8DDD;
const GL_MAX_VERTEX_VARYING_COMPONENTS_EXT        = $8DDE;
const GL_MAX_VARYING_COMPONENTS_EXT               = $8B4B;
const GL_MAX_GEOMETRY_UNIFORM_COMPONENTS_EXT      = $8DDF;
const GL_MAX_GEOMETRY_OUTPUT_VERTICES_EXT         = $8DE0;
const GL_MAX_GEOMETRY_TOTAL_OUTPUT_COMPONENTS_EXT = $8DE1;
const GL_LINES_ADJACENCY_EXT                      = $A;
const GL_LINE_STRIP_ADJACENCY_EXT                 = $B;
const GL_TRIANGLES_ADJACENCY_EXT                  = $C;
const GL_TRIANGLE_STRIP_ADJACENCY_EXT             = $D;
const GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS_EXT = $8DA8;
const GL_FRAMEBUFFER_INCOMPLETE_LAYER_COUNT_EXT   = $8DA9;
const GL_FRAMEBUFFER_ATTACHMENT_LAYERED_EXT       = $8DA7;
const GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LAYER_EXT = $8CD4;
const GL_PROGRAM_POINT_SIZE_EXT                   = $8642;
{$endc}

{$ifc not undefined GL_EXT_transform_feedback and GL_EXT_transform_feedback}
const GL_TRANSFORM_FEEDBACK_BUFFER_EXT                      = $8C8E;
const GL_TRANSFORM_FEEDBACK_BUFFER_START_EXT                = $8C84;
const GL_TRANSFORM_FEEDBACK_BUFFER_SIZE_EXT                 = $8C85;
const GL_TRANSFORM_FEEDBACK_BUFFER_BINDING_EXT              = $8C8F;
const GL_INTERLEAVED_ATTRIBS_EXT                            = $8C8C;
const GL_SEPARATE_ATTRIBS_EXT                               = $8C8D;
const GL_PRIMITIVES_GENERATED_EXT                           = $8C87;
const GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN_EXT          = $8C88;
const GL_RASTERIZER_DISCARD_EXT                             = $8C89;
const GL_MAX_TRANSFORM_FEEDBACK_INTERLEAVED_COMPONENTS_EXT  = $8C8A;
const GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS_EXT        = $8C8B;
const GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_COMPONENTS_EXT     = $8C80;
const GL_TRANSFORM_FEEDBACK_VARYINGS_EXT                    = $8C83;
const GL_TRANSFORM_FEEDBACK_BUFFER_MODE_EXT                 = $8C7F;
const GL_TRANSFORM_FEEDBACK_VARYING_MAX_LENGTH_EXT          = $8C76;
{$endc}

{$ifc not undefined GL_EXT_bindable_uniform and GL_EXT_bindable_uniform}
const GL_MAX_VERTEX_BINDABLE_UNIFORMS_EXT                = $8DE2;
const GL_MAX_FRAGMENT_BINDABLE_UNIFORMS_EXT              = $8DE3;
const GL_MAX_GEOMETRY_BINDABLE_UNIFORMS_EXT              = $8DE4;
const GL_MAX_BINDABLE_UNIFORM_SIZE_EXT                   = $8DED;
const GL_UNIFORM_BUFFER_BINDING_EXT                      = $8DEF;
const GL_UNIFORM_BUFFER_EXT                              = $8DEE;
{$endc}

{$ifc not undefined GL_EXT_texture_integer and GL_EXT_texture_integer}
const GL_RGBA_INTEGER_MODE_EXT                           = $8D9E;
const GL_RGBA32UI_EXT                                    = $8D70;
const GL_RGB32UI_EXT                                     = $8D71;
const GL_ALPHA32UI_EXT                                   = $8D72;
const GL_INTENSITY32UI_EXT                               = $8D73;
const GL_LUMINANCE32UI_EXT                               = $8D74;
const GL_LUMINANCE_ALPHA32UI_EXT                         = $8D75;
const GL_RGBA16UI_EXT                                    = $8D76;
const GL_RGB16UI_EXT                                     = $8D77;
const GL_ALPHA16UI_EXT                                   = $8D78;
const GL_INTENSITY16UI_EXT                               = $8D79;
const GL_LUMINANCE16UI_EXT                               = $8D7A;
const GL_LUMINANCE_ALPHA16UI_EXT                         = $8D7B;
const GL_RGBA8UI_EXT                                     = $8D7C;
const GL_RGB8UI_EXT                                      = $8D7D;
const GL_ALPHA8UI_EXT                                    = $8D7E;
const GL_INTENSITY8UI_EXT                                = $8D7F;
const GL_LUMINANCE8UI_EXT                                = $8D80;
const GL_LUMINANCE_ALPHA8UI_EXT                          = $8D81;
const GL_RGBA32I_EXT                                     = $8D82;
const GL_RGB32I_EXT                                      = $8D83;
const GL_ALPHA32I_EXT                                    = $8D84;
const GL_INTENSITY32I_EXT                                = $8D85;
const GL_LUMINANCE32I_EXT                                = $8D86;
const GL_LUMINANCE_ALPHA32I_EXT                          = $8D87;
const GL_RGBA16I_EXT                                     = $8D88;
const GL_RGB16I_EXT                                      = $8D89;
const GL_ALPHA16I_EXT                                    = $8D8A;
const GL_INTENSITY16I_EXT                                = $8D8B;
const GL_LUMINANCE16I_EXT                                = $8D8C;
const GL_LUMINANCE_ALPHA16I_EXT                          = $8D8D;
const GL_RGBA8I_EXT                                      = $8D8E;
const GL_RGB8I_EXT                                       = $8D8F;
const GL_ALPHA8I_EXT                                     = $8D90;
const GL_INTENSITY8I_EXT                                 = $8D91;
const GL_LUMINANCE8I_EXT                                 = $8D92;
const GL_LUMINANCE_ALPHA8I_EXT                           = $8D93;
const GL_RED_INTEGER_EXT                                 = $8D94;
const GL_GREEN_INTEGER_EXT                               = $8D95;
const GL_BLUE_INTEGER_EXT                                = $8D96;
const GL_ALPHA_INTEGER_EXT                               = $8D97;
const GL_RGB_INTEGER_EXT                                 = $8D98;
const GL_RGBA_INTEGER_EXT                                = $8D99;
const GL_BGR_INTEGER_EXT                                 = $8D9A;
const GL_BGRA_INTEGER_EXT                                = $8D9B;
const GL_LUMINANCE_INTEGER_EXT                           = $8D9C;
const GL_LUMINANCE_ALPHA_INTEGER_EXT                     = $8D9D;
{$endc}

{$ifc not undefined GL_EXT_gpu_shader4 and GL_EXT_gpu_shader4}
const GL_VERTEX_ATTRIB_ARRAY_INTEGER_EXT                    = $88FD;
const GL_SAMPLER_1D_ARRAY_EXT                               = $8DC0;
const GL_SAMPLER_2D_ARRAY_EXT                               = $8DC1;
const GL_SAMPLER_BUFFER_EXT                                 = $8DC2;
const GL_SAMPLER_1D_ARRAY_SHADOW_EXT                        = $8DC3;
const GL_SAMPLER_2D_ARRAY_SHADOW_EXT                        = $8DC4;
const GL_SAMPLER_CUBE_SHADOW_EXT                            = $8DC5;
const GL_UNSIGNED_INT_VEC2_EXT                              = $8DC6;
const GL_UNSIGNED_INT_VEC3_EXT                              = $8DC7;
const GL_UNSIGNED_INT_VEC4_EXT                              = $8DC8;
const GL_INT_SAMPLER_1D_EXT                                 = $8DC9;
const GL_INT_SAMPLER_2D_EXT                                 = $8DCA;
const GL_INT_SAMPLER_3D_EXT                                 = $8DCB;
const GL_INT_SAMPLER_CUBE_EXT                               = $8DCC;
const GL_INT_SAMPLER_2D_RECT_EXT                            = $8DCD;
const GL_INT_SAMPLER_1D_ARRAY_EXT                           = $8DCE;
const GL_INT_SAMPLER_2D_ARRAY_EXT                           = $8DCF;
const GL_INT_SAMPLER_BUFFER_EXT                             = $8DD0;
const GL_UNSIGNED_INT_SAMPLER_1D_EXT                        = $8DD1;
const GL_UNSIGNED_INT_SAMPLER_2D_EXT                        = $8DD2;
const GL_UNSIGNED_INT_SAMPLER_3D_EXT                        = $8DD3;
const GL_UNSIGNED_INT_SAMPLER_CUBE_EXT                      = $8DD4;
const GL_UNSIGNED_INT_SAMPLER_2D_RECT_EXT                   = $8DD5;
const GL_UNSIGNED_INT_SAMPLER_1D_ARRAY_EXT                  = $8DD6;
const GL_UNSIGNED_INT_SAMPLER_2D_ARRAY_EXT                  = $8DD7;
const GL_UNSIGNED_INT_SAMPLER_BUFFER_EXT                    = $8DD8;
const GL_MIN_PROGRAM_TEXEL_OFFSET_EXT                       = $8904;
const GL_MAX_PROGRAM_TEXEL_OFFSET_EXT                       = $8905;
{$endc}

{$ifc not undefined GL_EXT_framebuffer_sRGB and GL_EXT_framebuffer_sRGB}
const GL_FRAMEBUFFER_SRGB_EXT                 = $8DB9;
const GL_FRAMEBUFFER_SRGB_CAPABLE_EXT         = $8DBA;
{$endc}

{$ifc not undefined GL_APPLE_vertex_array_range and GL_APPLE_vertex_array_range}
const GL_VERTEX_ARRAY_RANGE_APPLE             = $851D;
const GL_VERTEX_ARRAY_RANGE_LENGTH_APPLE      = $851E;
const GL_MAX_VERTEX_ARRAY_RANGE_ELEMENT_APPLE = $8520;
const GL_VERTEX_ARRAY_RANGE_POINTER_APPLE     = $8521;
const GL_VERTEX_ARRAY_STORAGE_HINT_APPLE      = $851F;
const GL_STORAGE_CLIENT_APPLE                 = $85B4;
const GL_STORAGE_PRIVATE_APPLE                = $85BD;
const GL_STORAGE_CACHED_APPLE                 = $85BE;
const GL_STORAGE_SHARED_APPLE                 = $85BF;
{$endc}

{$ifc not undefined GL_APPLE_vertex_array_object and GL_APPLE_vertex_array_object}
const GL_VERTEX_ARRAY_BINDING_APPLE      = $85B5;
{$endc}

{$ifc not undefined GL_APPLE_element_array and GL_APPLE_element_array}
const GL_ELEMENT_ARRAY_APPLE             = $8A0C;
const GL_ELEMENT_ARRAY_TYPE_APPLE        = $8A0D;
const GL_ELEMENT_ARRAY_POINTER_APPLE     = $8A0E;
{$endc}

{$ifc not undefined GL_APPLE_fence and GL_APPLE_fence}
const GL_DRAW_PIXELS_APPLE              = $8A0A;
const GL_FENCE_APPLE                    = $8A0B;
const GL_BUFFER_OBJECT_APPLE            = $85B3;
{$endc}

{$ifc not undefined GL_APPLE_specular_vector and GL_APPLE_specular_vector}
const GL_LIGHT_MODEL_SPECULAR_VECTOR_APPLE = $85B0;
{$endc}

{$ifc not undefined GL_APPLE_transform_hint and GL_APPLE_transform_hint}
const GL_TRANSFORM_HINT_APPLE           = $85B1;
{$endc}

{$ifc not undefined GL_APPLE_client_storage and GL_APPLE_client_storage}
const GL_UNPACK_CLIENT_STORAGE_APPLE    = $85B2;
{$endc}

{$ifc not undefined GL_APPLE_ycbcr_422 and GL_APPLE_ycbcr_422}
const GL_YCBCR_422_APPLE                 = $85B9;
{$endc}

{$ifc not undefined GL_APPLE_rgb_422 and GL_APPLE_rgb_422}
const GL_RGB_422_APPLE                   = $8A1F;
{$endc}
	
{$ifc not undefined GL_APPLE_ycbcr_422 and GL_APPLE_ycbcr_422 or defined GL_APPLE_rgb_422 and GL_APPLE_rgb_422}
const GL_UNSIGNED_SHORT_8_8_APPLE        = $85BA;
const GL_UNSIGNED_SHORT_8_8_REV_APPLE    = $85BB;
{$endc}

{$ifc not undefined GL_APPLE_texture_range and GL_APPLE_texture_range}
const GL_TEXTURE_RANGE_LENGTH_APPLE      = $85B7;
const GL_TEXTURE_RANGE_POINTER_APPLE     = $85B8;
const GL_TEXTURE_STORAGE_HINT_APPLE      = $85BC;
const GL_TEXTURE_MINIMIZE_STORAGE_APPLE  = $85B6;
{ const GL_STORAGE_PRIVATE_APPLE           = $85BD; }
{ const GL_STORAGE_CACHED_APPLE            = $85BE; }
{ const GL_STORAGE_SHARED_APPLE            = $85BF; }
{$endc}

{$ifc not undefined GL_APPLE_float_pixels and GL_APPLE_float_pixels}
const GL_HALF_APPLE                      = $140B;
const GL_COLOR_FLOAT_APPLE               = $8A0F;
const GL_RGBA_FLOAT32_APPLE              = $8814;
const GL_RGB_FLOAT32_APPLE               = $8815;
const GL_ALPHA_FLOAT32_APPLE             = $8816;
const GL_INTENSITY_FLOAT32_APPLE         = $8817;
const GL_LUMINANCE_FLOAT32_APPLE         = $8818;
const GL_LUMINANCE_ALPHA_FLOAT32_APPLE   = $8819;
const GL_RGBA_FLOAT16_APPLE              = $881A;
const GL_RGB_FLOAT16_APPLE               = $881B;
const GL_ALPHA_FLOAT16_APPLE             = $881C;
const GL_INTENSITY_FLOAT16_APPLE         = $881D;
const GL_LUMINANCE_FLOAT16_APPLE         = $881E;
const GL_LUMINANCE_ALPHA_FLOAT16_APPLE   = $881F;
{$endc}

{$ifc not undefined GL_APPLE_pixel_buffer and GL_APPLE_pixel_buffer}
const GL_MIN_PBUFFER_VIEWPORT_DIMS_APPLE = $8A10;
{$endc}

{$ifc not undefined GL_APPLE_vertex_program_evaluators and GL_APPLE_vertex_program_evaluators}
const GL_VERTEX_ATTRIB_MAP1_APPLE                      = $8A00;
const GL_VERTEX_ATTRIB_MAP2_APPLE                      = $8A01;
const GL_VERTEX_ATTRIB_MAP1_SIZE_APPLE                 = $8A02;
const GL_VERTEX_ATTRIB_MAP1_COEFF_APPLE                = $8A03;
const GL_VERTEX_ATTRIB_MAP1_ORDER_APPLE                = $8A04;
const GL_VERTEX_ATTRIB_MAP1_DOMAIN_APPLE               = $8A05;
const GL_VERTEX_ATTRIB_MAP2_SIZE_APPLE                 = $8A06;
const GL_VERTEX_ATTRIB_MAP2_COEFF_APPLE                = $8A07;
const GL_VERTEX_ATTRIB_MAP2_ORDER_APPLE                = $8A08;
const GL_VERTEX_ATTRIB_MAP2_DOMAIN_APPLE               = $8A09;
{$endc}

{$ifc not undefined GL_APPLE_flush_buffer_range and GL_APPLE_flush_buffer_range}
const GL_BUFFER_SERIALIZED_MODIFY_APPLE = $8A12;
const GL_BUFFER_FLUSHING_UNMAP_APPLE    = $8A13;
{$endc}

{$ifc not undefined GL_APPLE_aux_depth_stencil and GL_APPLE_aux_depth_stencil}
const GL_AUX_DEPTH_STENCIL_APPLE        = $8A14;
{$endc}

{$ifc not undefined GL_APPLE_row_bytes and GL_APPLE_row_bytes}
const GL_PACK_ROW_BYTES_APPLE           = $8A15;
const GL_UNPACK_ROW_BYTES_APPLE         = $8A16;
const GL_PACK_IMAGE_BYTES_APPLE         = $8A17;
const GL_UNPACK_IMAGE_BYTES_APPLE       = $8A18;
{$endc}

{$ifc not undefined GL_APPLE_object_purgeable and GL_APPLE_object_purgeable}
const GL_RELEASED_APPLE                 = $8A19;
const GL_VOLATILE_APPLE                 = $8A1A;
const GL_RETAINED_APPLE                 = $8A1B;
const GL_UNDEFINED_APPLE                = $8A1C;
const GL_PURGEABLE_APPLE                = $8A1D;
{$endc}

{$ifc not undefined GL_APPLE_vertex_point_size and GL_APPLE_vertex_point_size}
const GL_VERTEX_POINT_SIZE_APPLE               = $8A26;
const GL_CURRENT_POINT_SIZE_APPLE              = $8A27;
const GL_POINT_SIZE_ARRAY_APPLE                = $8B9C;
const GL_POINT_SIZE_ARRAY_TYPE_APPLE           = $898A;
const GL_POINT_SIZE_ARRAY_STRIDE_APPLE         = $898B;
const GL_POINT_SIZE_ARRAY_POINTER_APPLE        = $898C;
const GL_POINT_SIZE_ARRAY_BUFFER_BINDING_APPLE = $8B9F;
{$endc}
	
{$ifc not undefined GL_ATI_blend_weighted_minmax and GL_ATI_blend_weighted_minmax}
const GL_MIN_WEIGHTED_ATI               = $877D;
const GL_MAX_WEIGHTED_ATI               = $877E;
{$endc}

{$ifc not undefined GL_ATI_texture_env_combine3 and GL_ATI_texture_env_combine3}
const GL_MODULATE_ADD_ATI               = $8744;
const GL_MODULATE_SIGNED_ADD_ATI        = $8745;
const GL_MODULATE_SUBTRACT_ATI          = $8746;
{$endc}

{$ifc not undefined GL_ATI_separate_stencil and GL_ATI_separate_stencil}
const GL_STENCIL_BACK_FUNC_ATI            = $8800;
const GL_STENCIL_BACK_FAIL_ATI            = $8801;
const GL_STENCIL_BACK_PASS_DEPTH_FAIL_ATI = $8802;
const GL_STENCIL_BACK_PASS_DEPTH_PASS_ATI = $8803;
{$endc}

{$ifc not undefined GL_ATI_array_rev_comps_in_4_bytes and GL_ATI_array_rev_comps_in_4_bytes}
const GL_ARRAY_REV_COMPS_IN_4_BYTES_ATI = $897C;
{$endc}

{$ifc not undefined GL_ATI_texture_mirror_once and GL_ATI_texture_mirror_once}
const GL_MIRROR_CLAMP_ATI                             = $8742;
const GL_MIRROR_CLAMP_TO_EDGE_ATI                     = $8743;
{$endc}

{$ifc not undefined GL_ATI_pn_triangles and GL_ATI_pn_triangles}
const GL_PN_TRIANGLES_ATI                             = $6090;
const GL_MAX_PN_TRIANGLES_TESSELATION_LEVEL_ATI       = $6091;
const GL_PN_TRIANGLES_POINT_MODE_ATI                  = $6092;
const GL_PN_TRIANGLES_NORMAL_MODE_ATI                 = $6093;
const GL_PN_TRIANGLES_TESSELATION_LEVEL_ATI           = $6094;
const GL_PN_TRIANGLES_POINT_MODE_LINEAR_ATI           = $6095;
const GL_PN_TRIANGLES_POINT_MODE_CUBIC_ATI            = $6096;
const GL_PN_TRIANGLES_NORMAL_MODE_LINEAR_ATI          = $6097;
const GL_PN_TRIANGLES_NORMAL_MODE_QUADRATIC_ATI       = $6098;
{$endc}

{$ifc not undefined GL_ATI_text_fragment_shader and GL_ATI_text_fragment_shader}
const GL_TEXT_FRAGMENT_SHADER_ATI                     = $8200;
{$endc}

{$ifc not undefined GL_ATI_blend_equation_separate and GL_ATI_blend_equation_separate}
const GL_ALPHA_BLEND_EQUATION_ATI                     = $883D;
{$endc}

{$ifc not undefined GL_ATI_point_cull_mode and GL_ATI_point_cull_mode}
const GL_POINT_CULL_MODE_ATI                           = $60B3;
const GL_POINT_CULL_CENTER_ATI                         = $60B4;
const GL_POINT_CULL_CLIP_ATI                           = $60B5;
{$endc}

{$ifc not undefined GL_ATIX_pn_triangles and GL_ATIX_pn_triangles}
const GL_PN_TRIANGLES_ATIX                            = $6090;
const GL_MAX_PN_TRIANGLES_TESSELATION_LEVEL_ATIX      = $6091;
const GL_PN_TRIANGLES_POINT_MODE_ATIX                 = $6092;
const GL_PN_TRIANGLES_NORMAL_MODE_ATIX                = $6093;
const GL_PN_TRIANGLES_TESSELATION_LEVEL_ATIX          = $6094;
const GL_PN_TRIANGLES_POINT_MODE_LINEAR_ATIX          = $6095;
const GL_PN_TRIANGLES_POINT_MODE_CUBIC_ATIX           = $6096;
const GL_PN_TRIANGLES_NORMAL_MODE_LINEAR_ATIX         = $6097;
const GL_PN_TRIANGLES_NORMAL_MODE_QUADRATIC_ATIX      = $6098;
{$endc}

{$ifc not undefined GL_ATI_texture_compression_3dc and GL_ATI_texture_compression_3dc}
const GL_COMPRESSED_LUMINANCE_ALPHA_3DC_ATI           = $8837;
{$endc}

{$ifc not undefined GL_ARB_texture_compression_rgtc and GL_ARB_texture_compression_rgtc}
const GL_COMPRESSED_RED_RGTC1                       = $8DBB;
const GL_COMPRESSED_SIGNED_RED_RGTC1                = $8DBC;
const GL_COMPRESSED_RG_RGTC2                        = $8DBD;
const GL_COMPRESSED_SIGNED_RG_RGTC2                 = $8DBE;
{$endc}

{$ifc not undefined GL_ATI_texture_float and GL_ATI_texture_float}
const GL_RGBA_FLOAT32_ATI                             = $8814;
const GL_RGB_FLOAT32_ATI                              = $8815;
const GL_ALPHA_FLOAT32_ATI                            = $8816;
const GL_INTENSITY_FLOAT32_ATI                        = $8817;
const GL_LUMINANCE_FLOAT32_ATI                        = $8818;
const GL_LUMINANCE_ALPHA_FLOAT32_ATI                  = $8819;
const GL_RGBA_FLOAT16_ATI                             = $881A;
const GL_RGB_FLOAT16_ATI                              = $881B;
const GL_ALPHA_FLOAT16_ATI                            = $881C;
const GL_INTENSITY_FLOAT16_ATI                        = $881D;
const GL_LUMINANCE_FLOAT16_ATI                        = $881E;
const GL_LUMINANCE_ALPHA_FLOAT16_ATI                  = $881F;
{$endc}


{$ifc not undefined GL_HP_image_transform and GL_HP_image_transform}
const GL_IMAGE_SCALE_X_HP               = $8155;
const GL_IMAGE_SCALE_Y_HP               = $8156;
const GL_IMAGE_TRANSLATE_X_HP           = $8157;
const GL_IMAGE_TRANSLATE_Y_HP           = $8158;
const GL_IMAGE_ROTATE_ANGLE_HP          = $8159;
const GL_IMAGE_ROTATE_ORIGIN_X_HP       = $815A;
const GL_IMAGE_ROTATE_ORIGIN_Y_HP       = $815B;
const GL_IMAGE_MAG_FILTER_HP            = $815C;
const GL_IMAGE_MIN_FILTER_HP            = $815D;
const GL_IMAGE_CUBIC_WEIGHT_HP          = $815E;
const GL_CUBIC_HP                       = $815F;
const GL_AVERAGE_HP                     = $8160;
const GL_IMAGE_TRANSFORM_2D_HP          = $8161;
const GL_POST_IMAGE_TRANSFORM_COLOR_TABLE_HP = $8162;
const GL_PROXY_POST_IMAGE_TRANSFORM_COLOR_TABLE_HP = $8163;
{$endc}

{$ifc not undefined GL_HP_convolution_border_modes and GL_HP_convolution_border_modes}
const GL_IGNORE_BORDER_HP               = $8150;
const GL_CONSTANT_BORDER_HP             = $8151;
const GL_REPLICATE_BORDER_HP            = $8153;
const GL_CONVOLUTION_BORDER_COLOR_HP    = $8154;
{$endc}

{$ifc not undefined GL_HP_texture_lighting and GL_HP_texture_lighting}
const GL_TEXTURE_LIGHTING_MODE_HP       = $8167;
const GL_TEXTURE_POST_SPECULAR_HP       = $8168;
const GL_TEXTURE_PRE_SPECULAR_HP        = $8169;
{$endc}

{$ifc not undefined GL_HP_occlusion_test and GL_HP_occlusion_test}
const GL_OCCLUSION_TEST_HP              = $8165;
const GL_OCCLUSION_TEST_RESULT_HP       = $8166;
{$endc}


{$ifc not undefined GL_IBM_rasterpos_clip and GL_IBM_rasterpos_clip}
const GL_RASTER_POSITION_UNCLIPPED_IBM  = $19262;
{$endc}

{$ifc not undefined GL_IBM_cull_vertex and GL_IBM_cull_vertex}
const
	GL_CULL_VERTEX_IBM = 103050;
{$endc}

{$ifc not undefined GL_IBM_vertex_array_lists and GL_IBM_vertex_array_lists}
const
	GL_VERTEX_ARRAY_LIST_IBM = 103070;
const
	GL_NORMAL_ARRAY_LIST_IBM = 103071;
const
	GL_COLOR_ARRAY_LIST_IBM = 103072;
const
	GL_INDEX_ARRAY_LIST_IBM = 103073;
const
	GL_TEXTURE_COORD_ARRAY_LIST_IBM = 103074;
const
	GL_EDGE_FLAG_ARRAY_LIST_IBM = 103075;
const
	GL_FOG_COORDINATE_ARRAY_LIST_IBM = 103076;
const
	GL_SECONDARY_COLOR_ARRAY_LIST_IBM = 103077;
const
	GL_VERTEX_ARRAY_LIST_STRIDE_IBM = 103080;
const
	GL_NORMAL_ARRAY_LIST_STRIDE_IBM = 103081;
const
	GL_COLOR_ARRAY_LIST_STRIDE_IBM = 103082;
const
	GL_INDEX_ARRAY_LIST_STRIDE_IBM = 103083;
const
	GL_TEXTURE_COORD_ARRAY_LIST_STRIDE_IBM = 103084;
const
	GL_EDGE_FLAG_ARRAY_LIST_STRIDE_IBM = 103085;
const
	GL_FOG_COORDINATE_ARRAY_LIST_STRIDE_IBM = 103086;
const
	GL_SECONDARY_COLOR_ARRAY_LIST_STRIDE_IBM = 103087;
{$endc}


{$ifc not undefined GL_INGR_color_clamp and GL_INGR_color_clamp}
const GL_RED_MIN_CLAMP_INGR             = $8560;
const GL_GREEN_MIN_CLAMP_INGR           = $8561;
const GL_BLUE_MIN_CLAMP_INGR            = $8562;
const GL_ALPHA_MIN_CLAMP_INGR           = $8563;
const GL_RED_MAX_CLAMP_INGR             = $8564;
const GL_GREEN_MAX_CLAMP_INGR           = $8565;
const GL_BLUE_MAX_CLAMP_INGR            = $8566;
const GL_ALPHA_MAX_CLAMP_INGR           = $8567;
{$endc}

{$ifc not undefined GL_INGR_interlace_read and GL_INGR_interlace_read}
const GL_INTERLACE_READ_INGR            = $8568;
{$endc}


{$ifc not undefined GL_INTEL_parallel_arrays and GL_INTEL_parallel_arrays}
const GL_PARALLEL_ARRAYS_INTEL          = $83F4;
const GL_VERTEX_ARRAY_PARALLEL_POINTERS_INTEL = $83F5;
const GL_NORMAL_ARRAY_PARALLEL_POINTERS_INTEL = $83F6;
const GL_COLOR_ARRAY_PARALLEL_POINTERS_INTEL = $83F7;
const GL_TEXTURE_COORD_ARRAY_PARALLEL_POINTERS_INTEL = $83F8;
{$endc}


{$ifc not undefined GL_NV_texgen_reflection and GL_NV_texgen_reflection}
const GL_NORMAL_MAP_NV                  = $8511;
const GL_REFLECTION_MAP_NV              = $8512;
{$endc}

{$ifc not undefined GL_NV_light_max_exponent and GL_NV_light_max_exponent}
const GL_MAX_SHININESS_NV               = $8504;
const GL_MAX_SPOT_EXPONENT_NV           = $8505;
{$endc}

{$ifc not undefined GL_NV_vertex_array_range and GL_NV_vertex_array_range}
const GL_VERTEX_ARRAY_RANGE_NV          = $851D;
const GL_VERTEX_ARRAY_RANGE_LENGTH_NV   = $851E;
const GL_VERTEX_ARRAY_RANGE_VALID_NV    = $851F;
const GL_MAX_VERTEX_ARRAY_RANGE_ELEMENT_NV = $8520;
const GL_VERTEX_ARRAY_RANGE_POINTER_NV  = $8521;
{$endc}

{$ifc not undefined GL_NV_vertex_array_range2 and GL_NV_vertex_array_range2}
const GL_VERTEX_ARRAY_RANGE_WITHOUT_FLUSH_NV = $8533;
{$endc}

{$ifc not undefined GL_NV_register_combiners and GL_NV_register_combiners}
const GL_REGISTER_COMBINERS_NV          = $8522;
const GL_VARIABLE_A_NV                  = $8523;
const GL_VARIABLE_B_NV                  = $8524;
const GL_VARIABLE_C_NV                  = $8525;
const GL_VARIABLE_D_NV                  = $8526;
const GL_VARIABLE_E_NV                  = $8527;
const GL_VARIABLE_F_NV                  = $8528;
const GL_VARIABLE_G_NV                  = $8529;
const GL_CONSTANT_COLOR0_NV             = $852A;
const GL_CONSTANT_COLOR1_NV             = $852B;
const GL_PRIMARY_COLOR_NV               = $852C;
const GL_SECONDARY_COLOR_NV             = $852D;
const GL_SPARE0_NV                      = $852E;
const GL_SPARE1_NV                      = $852F;
const GL_DISCARD_NV                     = $8530;
const GL_E_TIMES_F_NV                   = $8531;
const GL_SPARE0_PLUS_SECONDARY_COLOR_NV = $8532;
const GL_UNSIGNED_IDENTITY_NV           = $8536;
const GL_UNSIGNED_INVERT_NV             = $8537;
const GL_EXPAND_NORMAL_NV               = $8538;
const GL_EXPAND_NEGATE_NV               = $8539;
const GL_HALF_BIAS_NORMAL_NV            = $853A;
const GL_HALF_BIAS_NEGATE_NV            = $853B;
const GL_SIGNED_IDENTITY_NV             = $853C;
const GL_SIGNED_NEGATE_NV               = $853D;
const GL_SCALE_BY_TWO_NV                = $853E;
const GL_SCALE_BY_FOUR_NV               = $853F;
const GL_SCALE_BY_ONE_HALF_NV           = $8540;
const GL_BIAS_BY_NEGATIVE_ONE_HALF_NV   = $8541;
const GL_COMBINER_INPUT_NV              = $8542;
const GL_COMBINER_MAPPING_NV            = $8543;
const GL_COMBINER_COMPONENT_USAGE_NV    = $8544;
const GL_COMBINER_AB_DOT_PRODUCT_NV     = $8545;
const GL_COMBINER_CD_DOT_PRODUCT_NV     = $8546;
const GL_COMBINER_MUX_SUM_NV            = $8547;
const GL_COMBINER_SCALE_NV              = $8548;
const GL_COMBINER_BIAS_NV               = $8549;
const GL_COMBINER_AB_OUTPUT_NV          = $854A;
const GL_COMBINER_CD_OUTPUT_NV          = $854B;
const GL_COMBINER_SUM_OUTPUT_NV         = $854C;
const GL_MAX_GENERAL_COMBINERS_NV       = $854D;
const GL_NUM_GENERAL_COMBINERS_NV       = $854E;
const GL_COLOR_SUM_CLAMP_NV             = $854F;
const GL_COMBINER0_NV                   = $8550;
const GL_COMBINER1_NV                   = $8551;
const GL_COMBINER2_NV                   = $8552;
const GL_COMBINER3_NV                   = $8553;
const GL_COMBINER4_NV                   = $8554;
const GL_COMBINER5_NV                   = $8555;
const GL_COMBINER6_NV                   = $8556;
const GL_COMBINER7_NV                   = $8557;
{ reuse GL_TEXTURE0_ARB }
{ reuse GL_TEXTURE1_ARB }
{ reuse GL_ZERO }
{ reuse GL_NONE }
{ reuse GL_FOG }
{$endc}

{$ifc not undefined GL_NV_register_combiners2 and GL_NV_register_combiners2}
const GL_PER_STAGE_CONSTANTS_NV         = $8535;
{$endc}

{$ifc not undefined GL_NV_fog_distance and GL_NV_fog_distance}
const GL_FOG_DISTANCE_MODE_NV           = $855A;
const GL_EYE_RADIAL_NV                  = $855B;
const GL_EYE_PLANE_ABSOLUTE_NV          = $855C;
{ reuse GL_EYE_PLANE }
{$endc}

{$ifc not undefined GL_NV_texgen_emboss and GL_NV_texgen_emboss}
const GL_EMBOSS_LIGHT_NV                = $855D;
const GL_EMBOSS_CONSTANT_NV             = $855E;
const GL_EMBOSS_MAP_NV                  = $855F;
{$endc}

{$ifc not undefined GL_NV_vertex_program and GL_NV_vertex_program}
const GL_VERTEX_PROGRAM_NV              = $8620;
const GL_VERTEX_STATE_PROGRAM_NV        = $8621;
const GL_ATTRIB_ARRAY_SIZE_NV           = $8623;
const GL_ATTRIB_ARRAY_STRIDE_NV         = $8624;
const GL_ATTRIB_ARRAY_TYPE_NV           = $8625;
const GL_CURRENT_ATTRIB_NV              = $8626;
const GL_PROGRAM_LENGTH_NV              = $8627;
const GL_PROGRAM_STRING_NV              = $8628;
const GL_MODELVIEW_PROJECTION_NV        = $8629;
const GL_IDENTITY_NV                    = $862A;
const GL_INVERSE_NV                     = $862B;
const GL_TRANSPOSE_NV                   = $862C;
const GL_INVERSE_TRANSPOSE_NV           = $862D;
const GL_MAX_TRACK_MATRIX_STACK_DEPTH_NV = $862E;
const GL_MAX_TRACK_MATRICES_NV          = $862F;
const GL_MATRIX0_NV                     = $8630;
const GL_MATRIX1_NV                     = $8631;
const GL_MATRIX2_NV                     = $8632;
const GL_MATRIX3_NV                     = $8633;
const GL_MATRIX4_NV                     = $8634;
const GL_MATRIX5_NV                     = $8635;
const GL_MATRIX6_NV                     = $8636;
const GL_MATRIX7_NV                     = $8637;
const GL_CURRENT_MATRIX_STACK_DEPTH_NV  = $8640;
const GL_CURRENT_MATRIX_NV              = $8641;
const GL_VERTEX_PROGRAM_POINT_SIZE_NV   = $8642;
const GL_VERTEX_PROGRAM_TWO_SIDE_NV     = $8643;
const GL_PROGRAM_PARAMETER_NV           = $8644;
const GL_ATTRIB_ARRAY_POINTER_NV        = $8645;
const GL_PROGRAM_TARGET_NV              = $8646;
const GL_PROGRAM_RESIDENT_NV            = $8647;
const GL_TRACK_MATRIX_NV                = $8648;
const GL_TRACK_MATRIX_TRANSFORM_NV      = $8649;
const GL_VERTEX_PROGRAM_BINDING_NV      = $864A;
const GL_PROGRAM_ERROR_POSITION_NV      = $864B;
const GL_VERTEX_ATTRIB_ARRAY0_NV        = $8650;
const GL_VERTEX_ATTRIB_ARRAY1_NV        = $8651;
const GL_VERTEX_ATTRIB_ARRAY2_NV        = $8652;
const GL_VERTEX_ATTRIB_ARRAY3_NV        = $8653;
const GL_VERTEX_ATTRIB_ARRAY4_NV        = $8654;
const GL_VERTEX_ATTRIB_ARRAY5_NV        = $8655;
const GL_VERTEX_ATTRIB_ARRAY6_NV        = $8656;
const GL_VERTEX_ATTRIB_ARRAY7_NV        = $8657;
const GL_VERTEX_ATTRIB_ARRAY8_NV        = $8658;
const GL_VERTEX_ATTRIB_ARRAY9_NV        = $8659;
const GL_VERTEX_ATTRIB_ARRAY10_NV       = $865A;
const GL_VERTEX_ATTRIB_ARRAY11_NV       = $865B;
const GL_VERTEX_ATTRIB_ARRAY12_NV       = $865C;
const GL_VERTEX_ATTRIB_ARRAY13_NV       = $865D;
const GL_VERTEX_ATTRIB_ARRAY14_NV       = $865E;
const GL_VERTEX_ATTRIB_ARRAY15_NV       = $865F;
const GL_MAP1_VERTEX_ATTRIB0_4_NV       = $8660;
const GL_MAP1_VERTEX_ATTRIB1_4_NV       = $8661;
const GL_MAP1_VERTEX_ATTRIB2_4_NV       = $8662;
const GL_MAP1_VERTEX_ATTRIB3_4_NV       = $8663;
const GL_MAP1_VERTEX_ATTRIB4_4_NV       = $8664;
const GL_MAP1_VERTEX_ATTRIB5_4_NV       = $8665;
const GL_MAP1_VERTEX_ATTRIB6_4_NV       = $8666;
const GL_MAP1_VERTEX_ATTRIB7_4_NV       = $8667;
const GL_MAP1_VERTEX_ATTRIB8_4_NV       = $8668;
const GL_MAP1_VERTEX_ATTRIB9_4_NV       = $8669;
const GL_MAP1_VERTEX_ATTRIB10_4_NV      = $866A;
const GL_MAP1_VERTEX_ATTRIB11_4_NV      = $866B;
const GL_MAP1_VERTEX_ATTRIB12_4_NV      = $866C;
const GL_MAP1_VERTEX_ATTRIB13_4_NV      = $866D;
const GL_MAP1_VERTEX_ATTRIB14_4_NV      = $866E;
const GL_MAP1_VERTEX_ATTRIB15_4_NV      = $866F;
const GL_MAP2_VERTEX_ATTRIB0_4_NV       = $8670;
const GL_MAP2_VERTEX_ATTRIB1_4_NV       = $8671;
const GL_MAP2_VERTEX_ATTRIB2_4_NV       = $8672;
const GL_MAP2_VERTEX_ATTRIB3_4_NV       = $8673;
const GL_MAP2_VERTEX_ATTRIB4_4_NV       = $8674;
const GL_MAP2_VERTEX_ATTRIB5_4_NV       = $8675;
const GL_MAP2_VERTEX_ATTRIB6_4_NV       = $8676;
const GL_MAP2_VERTEX_ATTRIB7_4_NV       = $8677;
const GL_MAP2_VERTEX_ATTRIB8_4_NV       = $8678;
const GL_MAP2_VERTEX_ATTRIB9_4_NV       = $8679;
const GL_MAP2_VERTEX_ATTRIB10_4_NV      = $867A;
const GL_MAP2_VERTEX_ATTRIB11_4_NV      = $867B;
const GL_MAP2_VERTEX_ATTRIB12_4_NV      = $867C;
const GL_MAP2_VERTEX_ATTRIB13_4_NV      = $867D;
const GL_MAP2_VERTEX_ATTRIB14_4_NV      = $867E;
const GL_MAP2_VERTEX_ATTRIB15_4_NV      = $867F;
{$endc}

{$ifc not undefined GL_NV_texture_shader and GL_NV_texture_shader}
const GL_RGBA_UNSIGNED_DOT_PRODUCT_MAPPING_NV          = $86D9;
const GL_UNSIGNED_INT_S8_S8_8_8_NV                     = $86DA;
const GL_UNSIGNED_INT_8_8_S8_S8_REV_NV                 = $86DB;
const GL_DSDT_MAG_INTENSITY_NV                         = $86DC;
const GL_TEXTURE_SHADER_NV                             = $86DE;
const GL_SHADER_OPERATION_NV                           = $86DF;

const GL_CULL_MODES_NV                                 = $86E0;
const GL_OFFSET_TEXTURE_MATRIX_NV                      = $86E1;
const GL_OFFSET_TEXTURE_SCALE_NV                       = $86E2;
const GL_OFFSET_TEXTURE_BIAS_NV                        = $86E3;
const GL_OFFSET_TEXTURE_2D_MATRIX_NV                   = GL_OFFSET_TEXTURE_MATRIX_NV;
const GL_OFFSET_TEXTURE_2D_SCALE_NV                    = GL_OFFSET_TEXTURE_SCALE_NV;
const GL_OFFSET_TEXTURE_2D_BIAS_NV                     = GL_OFFSET_TEXTURE_BIAS_NV;
const GL_PREVIOUS_TEXTURE_INPUT_NV                     = $86E4;
const GL_CONST_EYE_NV                                  = $86E5;
const GL_SHADER_CONSISTENT_NV                          = $86DD;
const GL_PASS_THROUGH_NV                               = $86E6;
const GL_CULL_FRAGMENT_NV                              = $86E7;
const GL_OFFSET_TEXTURE_2D_NV                          = $86E8;
const GL_OFFSET_TEXTURE_RECTANGLE_NV                   = $864C;
const GL_OFFSET_TEXTURE_RECTANGLE_SCALE_NV             = $864D;
const GL_DEPENDENT_AR_TEXTURE_2D_NV                    = $86E9;
const GL_DEPENDENT_GB_TEXTURE_2D_NV                    = $86EA;
const GL_DOT_PRODUCT_NV                                = $86EC;
const GL_DOT_PRODUCT_DEPTH_REPLACE_NV                  = $86ED;
const GL_DOT_PRODUCT_TEXTURE_2D_NV                     = $86EE;
const GL_DOT_PRODUCT_TEXTURE_RECTANGLE_NV              = $864E;
const GL_DOT_PRODUCT_TEXTURE_CUBE_MAP_NV               = $86F0;
const GL_DOT_PRODUCT_DIFFUSE_CUBE_MAP_NV               = $86F1;
const GL_DOT_PRODUCT_REFLECT_CUBE_MAP_NV               = $86F2;
const GL_DOT_PRODUCT_CONST_EYE_REFLECT_CUBE_MAP_NV     = $86F3;
const GL_HILO_NV                                       = $86F4;
const GL_DSDT_NV                                       = $86F5;
const GL_DSDT_MAG_NV                                   = $86F6;
const GL_DSDT_MAG_VIB_NV                               = $86F7;
const GL_HILO16_NV                                     = $86F8;
const GL_SIGNED_HILO_NV                                = $86F9;
const GL_SIGNED_HILO16_NV                              = $86FA;
const GL_SIGNED_RGBA_NV                                = $86FB;
const GL_SIGNED_RGBA8_NV                               = $86FC;
const GL_SIGNED_RGB_NV                                 = $86FE;
const GL_SIGNED_RGB8_NV                                = $86FF;
const GL_SIGNED_LUMINANCE_NV                           = $8701;
const GL_SIGNED_LUMINANCE8_NV                          = $8702;
const GL_SIGNED_LUMINANCE_ALPHA_NV                     = $8703;
const GL_SIGNED_LUMINANCE8_ALPHA8_NV                   = $8704;
const GL_SIGNED_ALPHA_NV                               = $8705;
const GL_SIGNED_ALPHA8_NV                              = $8706;
const GL_SIGNED_INTENSITY_NV                           = $8707;
const GL_SIGNED_INTENSITY8_NV                          = $8708;
const GL_DSDT8_NV                                      = $8709;
const GL_DSDT8_MAG8_NV                                 = $870A;
const GL_DSDT8_MAG8_INTENSITY8_NV                      = $870B;
const GL_SIGNED_RGB_UNSIGNED_ALPHA_NV                  = $870C;
const GL_SIGNED_RGB8_UNSIGNED_ALPHA8_NV                = $870D;
const GL_HI_SCALE_NV                                   = $870E;
const GL_LO_SCALE_NV                                   = $870F;
const GL_DS_SCALE_NV                                   = $8710;
const GL_DT_SCALE_NV                                   = $8711;
const GL_MAGNITUDE_SCALE_NV                            = $8712;
const GL_VIBRANCE_SCALE_NV                             = $8713;
const GL_HI_BIAS_NV                                    = $8714;
const GL_LO_BIAS_NV                                    = $8715;
const GL_DS_BIAS_NV                                    = $8716;
const GL_DT_BIAS_NV                                    = $8717;
const GL_MAGNITUDE_BIAS_NV                             = $8718;
const GL_VIBRANCE_BIAS_NV                              = $8719;
const GL_TEXTURE_BORDER_VALUES_NV                      = $871A;
const GL_TEXTURE_HI_SIZE_NV                            = $871B;
const GL_TEXTURE_LO_SIZE_NV                            = $871C;
const GL_TEXTURE_DS_SIZE_NV                            = $871D;
const GL_TEXTURE_DT_SIZE_NV                            = $871E;
const GL_TEXTURE_MAG_SIZE_NV                           = $871F;
{$endc}

{$ifc not undefined GL_NV_texture_shader2 and GL_NV_texture_shader2}
const GL_DOT_PRODUCT_TEXTURE_3D_NV                     = $86EF;
{$endc}

{$ifc not undefined GL_NV_texture_shader3 and GL_NV_texture_shader3}
const GL_OFFSET_PROJECTIVE_TEXTURE_2D_NV               = $8850;
const GL_OFFSET_PROJECTIVE_TEXTURE_2D_SCALE_NV         = $8851;
const GL_OFFSET_PROJECTIVE_TEXTURE_RECTANGLE_NV        = $8852;
const GL_OFFSET_PROJECTIVE_TEXTURE_RECTANGLE_SCALE_NV  = $8853;
const GL_OFFSET_HILO_TEXTURE_2D_NV                     = $8854;
const GL_OFFSET_HILO_TEXTURE_RECTANGLE_NV              = $8855;
const GL_OFFSET_HILO_PROJECTIVE_TEXTURE_2D_NV          = $8856;
const GL_OFFSET_HILO_PROJECTIVE_TEXTURE_RECTANGLE_NV   = $8857;
const GL_DEPENDENT_HILO_TEXTURE_2D_NV                  = $8858;
const GL_DEPENDENT_RGB_TEXTURE_3D_NV                   = $8859;
const GL_DEPENDENT_RGB_TEXTURE_CUBE_MAP_NV             = $885A;
const GL_DOT_PRODUCT_PASS_THROUGH_NV                   = $885B;
const GL_DOT_PRODUCT_TEXTURE_1D_NV                     = $885C;
const GL_DOT_PRODUCT_AFFINE_DEPTH_REPLACE_NV           = $885D;
const GL_HILO8_NV                                      = $885E;
const GL_SIGNED_HILO8_NV                               = $885F;
const GL_FORCE_BLUE_TO_ONE_NV                          = $8860;
{$endc}

{$ifc not undefined GL_NV_point_sprite and GL_NV_point_sprite}
const GL_POINT_SPRITE_NV                               = $8861;
const GL_COORD_REPLACE_NV                              = $8862;
const GL_POINT_SPRITE_R_MODE_NV                        = $8863;
{$endc}

{$ifc not undefined GL_NV_depth_clamp and GL_NV_depth_clamp}
const GL_DEPTH_CLAMP_NV                                = $864F;
{$endc}

{$ifc not undefined GL_NV_multisample_filter_hint and GL_NV_multisample_filter_hint}
const GL_MULTISAMPLE_FILTER_HINT_NV                    = $8534;
{$endc}

{$ifc not undefined GL_NV_light_max_exponent and GL_NV_light_max_exponent}
{ const GL_MAX_SHININESS_NV                              = $8504; }
{ const GL_MAX_SPOT_EXPONENT_NV                          = $8505; }
{$endc}

{$ifc not undefined GL_NV_fragment_program2 and GL_NV_fragment_program2 or defined GL_NV_vertex_program2_option and GL_NV_vertex_program2_option}
const GL_MAX_PROGRAM_EXEC_INSTRUCTIONS_NV              = $88F4;
const GL_MAX_PROGRAM_CALL_DEPTH_NV                     = $88F5;
{$endc}

{$ifc not undefined GL_NV_fragment_program2 and GL_NV_fragment_program2}
const GL_MAX_PROGRAM_IF_DEPTH_NV                       = $88F6;
const GL_MAX_PROGRAM_LOOP_DEPTH_NV                     = $88F7;
const GL_MAX_PROGRAM_LOOP_COUNT_NV                     = $88F8;
{$endc}

{$ifc not undefined GL_NV_conditional_render and GL_NV_conditional_render}
const GL_QUERY_WAIT_NV                                 = $8E13;
const GL_QUERY_NO_WAIT_NV                              = $8E14;
const GL_QUERY_BY_REGION_WAIT_NV                       = $8E15;
const GL_QUERY_BY_REGION_NO_WAIT_NV                    = $8E16;
{$endc}

{$ifc not undefined GL_PGI_vertex_hints and GL_PGI_vertex_hints}
const GL_VERTEX_DATA_HINT_PGI           = $1A22A;
const GL_VERTEX_CONSISTENT_HINT_PGI     = $1A22B;
const GL_MATERIAL_SIDE_HINT_PGI         = $1A22C;
const GL_MAX_VERTEX_HINT_PGI            = $1A22D;
const GL_COLOR3_BIT_PGI                 = $00010000;
const GL_COLOR4_BIT_PGI                 = $00020000;
const GL_EDGEFLAG_BIT_PGI               = $00040000;
const GL_INDEX_BIT_PGI                  = $00080000;
const GL_MAT_AMBIENT_BIT_PGI            = $00100000;
const GL_MAT_AMBIENT_AND_DIFFUSE_BIT_PGI = $00200000;
const GL_MAT_DIFFUSE_BIT_PGI            = $00400000;
const GL_MAT_EMISSION_BIT_PGI           = $00800000;
const GL_MAT_COLOR_INDEXES_BIT_PGI      = $01000000;
const GL_MAT_SHININESS_BIT_PGI          = $02000000;
const GL_MAT_SPECULAR_BIT_PGI           = $04000000;
const GL_NORMAL_BIT_PGI                 = $08000000;
const GL_TEXCOORD1_BIT_PGI              = $10000000;
const GL_TEXCOORD2_BIT_PGI              = $20000000;
const GL_TEXCOORD3_BIT_PGI              = $40000000;
const GL_TEXCOORD4_BIT_PGI              = $80000000;
const GL_VERTEX23_BIT_PGI               = $00000004;
const GL_VERTEX4_BIT_PGI                = $00000008;
{$endc}

{$ifc not undefined GL_PGI_misc_hints and GL_PGI_misc_hints}
const GL_PREFER_DOUBLEBUFFER_HINT_PGI   = $1A1F8;
const GL_CONSERVE_MEMORY_HINT_PGI       = $1A1FD;
const GL_RECLAIM_MEMORY_HINT_PGI        = $1A1FE;
const GL_NATIVE_GRAPHICS_HANDLE_PGI     = $1A202;
const GL_NATIVE_GRAPHICS_BEGIN_HINT_PGI = $1A203;
const GL_NATIVE_GRAPHICS_END_HINT_PGI   = $1A204;
const GL_ALWAYS_FAST_HINT_PGI           = $1A20C;
const GL_ALWAYS_SOFT_HINT_PGI           = $1A20D;
const GL_ALLOW_DRAW_OBJ_HINT_PGI        = $1A20E;
const GL_ALLOW_DRAW_WIN_HINT_PGI        = $1A20F;
const GL_ALLOW_DRAW_FRG_HINT_PGI        = $1A210;
const GL_ALLOW_DRAW_MEM_HINT_PGI        = $1A211;
const GL_STRICT_DEPTHFUNC_HINT_PGI      = $1A216;
const GL_STRICT_LIGHTING_HINT_PGI       = $1A217;
const GL_STRICT_SCISSOR_HINT_PGI        = $1A218;
const GL_FULL_STIPPLE_HINT_PGI          = $1A219;
const GL_CLIP_NEAR_HINT_PGI             = $1A220;
const GL_CLIP_FAR_HINT_PGI              = $1A221;
const GL_WIDE_LINE_HINT_PGI             = $1A222;
const GL_BACK_NORMALS_HINT_PGI          = $1A223;
{$endc}


{$ifc not undefined GL_REND_screen_coordinates and GL_REND_screen_coordinates}
const GL_SCREEN_COORDINATES_REND        = $8490;
const GL_INVERTED_SCREEN_W_REND         = $8491;
{$endc}


{$ifc not undefined GL_SGI_color_matrix and GL_SGI_color_matrix}
const GL_COLOR_MATRIX_SGI               = $80B1;
const GL_COLOR_MATRIX_STACK_DEPTH_SGI   = $80B2;
const GL_MAX_COLOR_MATRIX_STACK_DEPTH_SGI = $80B3;
const GL_POST_COLOR_MATRIX_RED_SCALE_SGI = $80B4;
const GL_POST_COLOR_MATRIX_GREEN_SCALE_SGI = $80B5;
const GL_POST_COLOR_MATRIX_BLUE_SCALE_SGI = $80B6;
const GL_POST_COLOR_MATRIX_ALPHA_SCALE_SGI = $80B7;
const GL_POST_COLOR_MATRIX_RED_BIAS_SGI = $80B8;
const GL_POST_COLOR_MATRIX_GREEN_BIAS_SGI = $80B9;
const GL_POST_COLOR_MATRIX_BLUE_BIAS_SGI = $80BA;
const GL_POST_COLOR_MATRIX_ALPHA_BIAS_SGI = $80BB;
{$endc}

{$ifc not undefined GL_SGI_color_table and GL_SGI_color_table}
const GL_COLOR_TABLE_SGI                = $80D0;
const GL_POST_CONVOLUTION_COLOR_TABLE_SGI = $80D1;
const GL_POST_COLOR_MATRIX_COLOR_TABLE_SGI = $80D2;
const GL_PROXY_COLOR_TABLE_SGI          = $80D3;
const GL_PROXY_POST_CONVOLUTION_COLOR_TABLE_SGI = $80D4;
const GL_PROXY_POST_COLOR_MATRIX_COLOR_TABLE_SGI = $80D5;
const GL_COLOR_TABLE_SCALE_SGI          = $80D6;
const GL_COLOR_TABLE_BIAS_SGI           = $80D7;
const GL_COLOR_TABLE_FORMAT_SGI         = $80D8;
const GL_COLOR_TABLE_WIDTH_SGI          = $80D9;
const GL_COLOR_TABLE_RED_SIZE_SGI       = $80DA;
const GL_COLOR_TABLE_GREEN_SIZE_SGI     = $80DB;
const GL_COLOR_TABLE_BLUE_SIZE_SGI      = $80DC;
const GL_COLOR_TABLE_ALPHA_SIZE_SGI     = $80DD;
const GL_COLOR_TABLE_LUMINANCE_SIZE_SGI = $80DE;
const GL_COLOR_TABLE_INTENSITY_SIZE_SGI = $80DF;
{$endc}

{$ifc not undefined GL_SGI_texture_color_table and GL_SGI_texture_color_table}
const GL_TEXTURE_COLOR_TABLE_SGI        = $80BC;
const GL_PROXY_TEXTURE_COLOR_TABLE_SGI  = $80BD;
{$endc}

{$ifc not undefined GL_SGI_depth_pass_instrument and GL_SGI_depth_pass_instrument}
const GL_DEPTH_PASS_INSTRUMENT_SGIX     = $8310;
const GL_DEPTH_PASS_INSTRUMENT_COUNTERS_SGIX = $8311;
const GL_DEPTH_PASS_INSTRUMENT_MAX_SGIX = $8312;
{$endc}

{$ifc not undefined GL_SGIS_texture_filter4 and GL_SGIS_texture_filter4}
const GL_FILTER4_SGIS                   = $8146;
const GL_TEXTURE_FILTER4_SIZE_SGIS      = $8147;
{$endc}

{$ifc not undefined GL_SGIS_pixel_texture and GL_SGIS_pixel_texture}
const GL_PIXEL_TEXTURE_SGIS             = $8353;
const GL_PIXEL_FRAGMENT_RGB_SOURCE_SGIS = $8354;
const GL_PIXEL_FRAGMENT_ALPHA_SOURCE_SGIS = $8355;
const GL_PIXEL_GROUP_COLOR_SGIS         = $8356;
{$endc}

{$ifc not undefined GL_SGIS_texture4D and GL_SGIS_texture4D}
const GL_PACK_SKIP_VOLUMES_SGIS         = $8130;
const GL_PACK_IMAGE_DEPTH_SGIS          = $8131;
const GL_UNPACK_SKIP_VOLUMES_SGIS       = $8132;
const GL_UNPACK_IMAGE_DEPTH_SGIS        = $8133;
const GL_TEXTURE_4D_SGIS                = $8134;
const GL_PROXY_TEXTURE_4D_SGIS          = $8135;
const GL_TEXTURE_4DSIZE_SGIS            = $8136;
const GL_TEXTURE_WRAP_Q_SGIS            = $8137;
const GL_MAX_4D_TEXTURE_SIZE_SGIS       = $8138;
const GL_TEXTURE_4D_BINDING_SGIS        = $814F;
{$endc}

{$ifc not undefined GL_SGIS_detail_texture and GL_SGIS_detail_texture}
const GL_DETAIL_TEXTURE_2D_SGIS         = $8095;
const GL_DETAIL_TEXTURE_2D_BINDING_SGIS = $8096;
const GL_LINEAR_DETAIL_SGIS             = $8097;
const GL_LINEAR_DETAIL_ALPHA_SGIS       = $8098;
const GL_LINEAR_DETAIL_COLOR_SGIS       = $8099;
const GL_DETAIL_TEXTURE_LEVEL_SGIS      = $809A;
const GL_DETAIL_TEXTURE_MODE_SGIS       = $809B;
const GL_DETAIL_TEXTURE_FUNC_POINTS_SGIS = $809C;
{$endc}

{$ifc not undefined GL_SGIS_sharpen_texture and GL_SGIS_sharpen_texture}
const GL_LINEAR_SHARPEN_SGIS            = $80AD;
const GL_LINEAR_SHARPEN_ALPHA_SGIS      = $80AE;
const GL_LINEAR_SHARPEN_COLOR_SGIS      = $80AF;
const GL_SHARPEN_TEXTURE_FUNC_POINTS_SGIS = $80B0;
{$endc}

{$ifc not undefined GL_SGIS_texture_lod and GL_SGIS_texture_lod}
const GL_TEXTURE_MIN_LOD_SGIS           = $813A;
const GL_TEXTURE_MAX_LOD_SGIS           = $813B;
const GL_TEXTURE_BASE_LEVEL_SGIS        = $813C;
const GL_TEXTURE_MAX_LEVEL_SGIS         = $813D;
{$endc}

{$ifc not undefined GL_SGIS_multisample and GL_SGIS_multisample}
const GL_MULTISAMPLE_SGIS               = $809D;
const GL_SAMPLE_ALPHA_TO_MASK_SGIS      = $809E;
const GL_SAMPLE_ALPHA_TO_ONE_SGIS       = $809F;
const GL_SAMPLE_MASK_SGIS               = $80A0;
const GL_1PASS_SGIS                     = $80A1;
const GL_2PASS_0_SGIS                   = $80A2;
const GL_2PASS_1_SGIS                   = $80A3;
const GL_4PASS_0_SGIS                   = $80A4;
const GL_4PASS_1_SGIS                   = $80A5;
const GL_4PASS_2_SGIS                   = $80A6;
const GL_4PASS_3_SGIS                   = $80A7;
const GL_SAMPLE_BUFFERS_SGIS            = $80A8;
const GL_SAMPLES_SGIS                   = $80A9;
const GL_SAMPLE_MASK_VALUE_SGIS         = $80AA;
const GL_SAMPLE_MASK_INVERT_SGIS        = $80AB;
const GL_SAMPLE_PATTERN_SGIS            = $80AC;
{$endc}

{$ifc not undefined GL_SGIS_generate_mipmap and GL_SGIS_generate_mipmap}
const GL_GENERATE_MIPMAP_SGIS           = $8191;
const GL_GENERATE_MIPMAP_HINT_SGIS      = $8192;
{$endc}

{$ifc not undefined GL_SGIS_texture_edge_clamp and GL_SGIS_texture_edge_clamp}
const GL_CLAMP_TO_EDGE_SGIS             = $812F;
{$endc}

{$ifc not undefined GL_SGIS_texture_border_clamp and GL_SGIS_texture_border_clamp}
const GL_CLAMP_TO_BORDER_SGIS           = $812D;
{$endc}

{$ifc not undefined GL_SGIS_texture_select and GL_SGIS_texture_select}
const GL_DUAL_ALPHA4_SGIS               = $8110;
const GL_DUAL_ALPHA8_SGIS               = $8111;
const GL_DUAL_ALPHA12_SGIS              = $8112;
const GL_DUAL_ALPHA16_SGIS              = $8113;
const GL_DUAL_LUMINANCE4_SGIS           = $8114;
const GL_DUAL_LUMINANCE8_SGIS           = $8115;
const GL_DUAL_LUMINANCE12_SGIS          = $8116;
const GL_DUAL_LUMINANCE16_SGIS          = $8117;
const GL_DUAL_INTENSITY4_SGIS           = $8118;
const GL_DUAL_INTENSITY8_SGIS           = $8119;
const GL_DUAL_INTENSITY12_SGIS          = $811A;
const GL_DUAL_INTENSITY16_SGIS          = $811B;
const GL_DUAL_LUMINANCE_ALPHA4_SGIS     = $811C;
const GL_DUAL_LUMINANCE_ALPHA8_SGIS     = $811D;
const GL_QUAD_ALPHA4_SGIS               = $811E;
const GL_QUAD_ALPHA8_SGIS               = $811F;
const GL_QUAD_LUMINANCE4_SGIS           = $8120;
const GL_QUAD_LUMINANCE8_SGIS           = $8121;
const GL_QUAD_INTENSITY4_SGIS           = $8122;
const GL_QUAD_INTENSITY8_SGIS           = $8123;
const GL_DUAL_TEXTURE_SELECT_SGIS       = $8124;
const GL_QUAD_TEXTURE_SELECT_SGIS       = $8125;
{$endc}

{$ifc not undefined GL_SGIS_point_parameters and GL_SGIS_point_parameters}
const GL_POINT_SIZE_MIN_EXT             = $8126;
const GL_POINT_SIZE_MIN_SGIS            = $8126;
const GL_POINT_SIZE_MAX_EXT             = $8127;
const GL_POINT_SIZE_MAX_SGIS            = $8127;
const GL_POINT_FADE_THRESHOLD_SIZE_EXT  = $8128;
const GL_POINT_FADE_THRESHOLD_SIZE_SGIS = $8128;
const GL_DISTANCE_ATTENUATION_EXT       = $8129;
const GL_DISTANCE_ATTENUATION_SGIS      = $8129;
{$endc}

{$ifc not undefined GL_SGIS_fog_function and GL_SGIS_fog_function}
const GL_FOG_FUNC_SGIS                  = $812A;
const GL_FOG_FUNC_POINTS_SGIS           = $812B;
const GL_MAX_FOG_FUNC_POINTS_SGIS       = $812C;
{$endc}

{$ifc not undefined GL_SGIS_point_line_texgen and GL_SGIS_point_line_texgen}
const GL_EYE_DISTANCE_TO_POINT_SGIS     = $81F0;
const GL_OBJECT_DISTANCE_TO_POINT_SGIS  = $81F1;
const GL_EYE_DISTANCE_TO_LINE_SGIS      = $81F2;
const GL_OBJECT_DISTANCE_TO_LINE_SGIS   = $81F3;
const GL_EYE_POINT_SGIS                 = $81F4;
const GL_OBJECT_POINT_SGIS              = $81F5;
const GL_EYE_LINE_SGIS                  = $81F6;
const GL_OBJECT_LINE_SGIS               = $81F7;
{$endc}

{$ifc not undefined GL_SGIS_texture_color_mask and GL_SGIS_texture_color_mask}
const GL_TEXTURE_COLOR_WRITEMASK_SGIS   = $81EF;
{$endc}

{$ifc not undefined GL_SGIX_pixel_texture and GL_SGIX_pixel_texture}
const GL_PIXEL_TEX_GEN_SGIX             = $8139;
const GL_PIXEL_TEX_GEN_MODE_SGIX        = $832B;
{$endc}

{$ifc not undefined GL_SGIX_clipmap and GL_SGIX_clipmap}
const GL_LINEAR_CLIPMAP_LINEAR_SGIX     = $8170;
const GL_TEXTURE_CLIPMAP_CENTER_SGIX    = $8171;
const GL_TEXTURE_CLIPMAP_FRAME_SGIX     = $8172;
const GL_TEXTURE_CLIPMAP_OFFSET_SGIX    = $8173;
const GL_TEXTURE_CLIPMAP_VIRTUAL_DEPTH_SGIX = $8174;
const GL_TEXTURE_CLIPMAP_LOD_OFFSET_SGIX = $8175;
const GL_TEXTURE_CLIPMAP_DEPTH_SGIX     = $8176;
const GL_MAX_CLIPMAP_DEPTH_SGIX         = $8177;
const GL_MAX_CLIPMAP_VIRTUAL_DEPTH_SGIX = $8178;
const GL_NEAREST_CLIPMAP_NEAREST_SGIX   = $844D;
const GL_NEAREST_CLIPMAP_LINEAR_SGIX    = $844E;
const GL_LINEAR_CLIPMAP_NEAREST_SGIX    = $844F;
{$endc}

{$ifc not undefined GL_SGIX_shadow and GL_SGIX_shadow}
const GL_TEXTURE_COMPARE_SGIX           = $819A;
const GL_TEXTURE_COMPARE_OPERATOR_SGIX  = $819B;
const GL_TEXTURE_LEQUAL_R_SGIX          = $819C;
const GL_TEXTURE_GEQUAL_R_SGIX          = $819D;
{$endc}

{$ifc not undefined GL_SGIX_interlace and GL_SGIX_interlace}
const GL_INTERLACE_SGIX                 = $8094;
{$endc}

{$ifc not undefined GL_SGIX_pixel_tiles and GL_SGIX_pixel_tiles}
const GL_PIXEL_TILE_BEST_ALIGNMENT_SGIX = $813E;
const GL_PIXEL_TILE_CACHE_INCREMENT_SGIX = $813F;
const GL_PIXEL_TILE_WIDTH_SGIX          = $8140;
const GL_PIXEL_TILE_HEIGHT_SGIX         = $8141;
const GL_PIXEL_TILE_GRID_WIDTH_SGIX     = $8142;
const GL_PIXEL_TILE_GRID_HEIGHT_SGIX    = $8143;
const GL_PIXEL_TILE_GRID_DEPTH_SGIX     = $8144;
const GL_PIXEL_TILE_CACHE_SIZE_SGIX     = $8145;
{$endc}

{$ifc not undefined GL_SGIX_sprite and GL_SGIX_sprite}
const GL_SPRITE_SGIX                    = $8148;
const GL_SPRITE_MODE_SGIX               = $8149;
const GL_SPRITE_AXIS_SGIX               = $814A;
const GL_SPRITE_TRANSLATION_SGIX        = $814B;
const GL_SPRITE_AXIAL_SGIX              = $814C;
const GL_SPRITE_OBJECT_ALIGNED_SGIX     = $814D;
const GL_SPRITE_EYE_ALIGNED_SGIX        = $814E;
{$endc}

{$ifc not undefined GL_SGIX_texture_multi_buffer and GL_SGIX_texture_multi_buffer}
const GL_TEXTURE_MULTI_BUFFER_HINT_SGIX = $812E;
{$endc}

{$ifc not undefined GL_SGIX_instruments and GL_SGIX_instruments}
const GL_INSTRUMENT_BUFFER_POINTER_SGIX = $8180;
const GL_INSTRUMENT_MEASUREMENTS_SGIX   = $8181;
{$endc}

{$ifc not undefined GL_SGIX_texture_scale_bias and GL_SGIX_texture_scale_bias}
const GL_POST_TEXTURE_FILTER_BIAS_SGIX  = $8179;
const GL_POST_TEXTURE_FILTER_SCALE_SGIX = $817A;
const GL_POST_TEXTURE_FILTER_BIAS_RANGE_SGIX = $817B;
const GL_POST_TEXTURE_FILTER_SCALE_RANGE_SGIX = $817C;
{$endc}

{$ifc not undefined GL_SGIX_framezoom and GL_SGIX_framezoom}
const GL_FRAMEZOOM_SGIX                 = $818B;
const GL_FRAMEZOOM_FACTOR_SGIX          = $818C;
const GL_MAX_FRAMEZOOM_FACTOR_SGIX      = $818D;
{$endc}

{$ifc not undefined GL_SGIX_polynomial_ffd and GL_SGIX_polynomial_ffd}
const GL_GEOMETRY_DEFORMATION_SGIX      = $8194;
const GL_TEXTURE_DEFORMATION_SGIX       = $8195;
const GL_DEFORMATIONS_MASK_SGIX         = $8196;
const GL_MAX_DEFORMATION_ORDER_SGIX     = $8197;
{$endc}

{$ifc not undefined GL_SGIX_reference_plane and GL_SGIX_reference_plane}
const GL_REFERENCE_PLANE_SGIX           = $817D;
const GL_REFERENCE_PLANE_EQUATION_SGIX  = $817E;
{$endc}

{$ifc not undefined GL_SGIX_depth_texture and GL_SGIX_depth_texture}
const GL_DEPTH_COMPONENT16_SGIX         = $81A5;
const GL_DEPTH_COMPONENT24_SGIX         = $81A6;
const GL_DEPTH_COMPONENT32_SGIX         = $81A7;
{$endc}

{$ifc not undefined GL_SGIX_fog_offset and GL_SGIX_fog_offset}
const GL_FOG_OFFSET_SGIX                = $8198;
const GL_FOG_OFFSET_VALUE_SGIX          = $8199;
{$endc}

{$ifc not undefined GL_SGIX_texture_add_env and GL_SGIX_texture_add_env}
const GL_TEXTURE_ENV_BIAS_SGIX          = $80BE;
{$endc}

{$ifc not undefined GL_SGIX_list_priority and GL_SGIX_list_priority}
const GL_LIST_PRIORITY_SGIX             = $8182;
{$endc}

{$ifc not undefined GL_SGIX_ir_instrument1 and GL_SGIX_ir_instrument1}
const GL_IR_INSTRUMENT1_SGIX            = $817F;
{$endc}

{$ifc not undefined GL_SGIX_calligraphic_fragment and GL_SGIX_calligraphic_fragment}
const GL_CALLIGRAPHIC_FRAGMENT_SGIX     = $8183;
{$endc}

{$ifc not undefined GL_SGIX_texture_lod_bias and GL_SGIX_texture_lod_bias}
const GL_TEXTURE_LOD_BIAS_S_SGIX        = $818E;
const GL_TEXTURE_LOD_BIAS_T_SGIX        = $818F;
const GL_TEXTURE_LOD_BIAS_R_SGIX        = $8190;
{$endc}

{$ifc not undefined GL_SGIX_ycrcb and GL_SGIX_ycrcb}
const GL_YCRCB_422_SGIX                 = $81BB;
const GL_YCRCB_444_SGIX                 = $81BC;
{$endc}

{$ifc not undefined GL_SGIX_fragment_lighting and GL_SGIX_fragment_lighting}
const GL_FRAGMENT_LIGHTING_SGIX         = $8400;
const GL_FRAGMENT_COLOR_MATERIAL_SGIX   = $8401;
const GL_FRAGMENT_COLOR_MATERIAL_FACE_SGIX = $8402;
const GL_FRAGMENT_COLOR_MATERIAL_PARAMETER_SGIX = $8403;
const GL_MAX_FRAGMENT_LIGHTS_SGIX       = $8404;
const GL_MAX_ACTIVE_LIGHTS_SGIX         = $8405;
const GL_CURRENT_RASTER_NORMAL_SGIX     = $8406;
const GL_LIGHT_ENV_MODE_SGIX            = $8407;
const GL_FRAGMENT_LIGHT_MODEL_LOCAL_VIEWER_SGIX = $8408;
const GL_FRAGMENT_LIGHT_MODEL_TWO_SIDE_SGIX = $8409;
const GL_FRAGMENT_LIGHT_MODEL_AMBIENT_SGIX = $840A;
const GL_FRAGMENT_LIGHT_MODEL_NORMAL_INTERPOLATION_SGIX = $840B;
const GL_FRAGMENT_LIGHT0_SGIX           = $840C;
const GL_FRAGMENT_LIGHT1_SGIX           = $840D;
const GL_FRAGMENT_LIGHT2_SGIX           = $840E;
const GL_FRAGMENT_LIGHT3_SGIX           = $840F;
const GL_FRAGMENT_LIGHT4_SGIX           = $8410;
const GL_FRAGMENT_LIGHT5_SGIX           = $8411;
const GL_FRAGMENT_LIGHT6_SGIX           = $8412;
const GL_FRAGMENT_LIGHT7_SGIX           = $8413;
{$endc}

{$ifc not undefined GL_SGIX_blend_alpha_minmax and GL_SGIX_blend_alpha_minmax}
const GL_ALPHA_MIN_SGIX                 = $8320;
const GL_ALPHA_MAX_SGIX                 = $8321;
{$endc}

{$ifc not undefined GL_SGIX_async and GL_SGIX_async}
const GL_ASYNC_MARKER_SGIX              = $8329;
{$endc}

{$ifc not undefined GL_SGIX_async_pixel and GL_SGIX_async_pixel}
const GL_ASYNC_TEX_IMAGE_SGIX           = $835C;
const GL_ASYNC_DRAW_PIXELS_SGIX         = $835D;
const GL_ASYNC_READ_PIXELS_SGIX         = $835E;
const GL_MAX_ASYNC_TEX_IMAGE_SGIX       = $835F;
const GL_MAX_ASYNC_DRAW_PIXELS_SGIX     = $8360;
const GL_MAX_ASYNC_READ_PIXELS_SGIX     = $8361;
{$endc}

{$ifc not undefined GL_SGIX_async_histogram and GL_SGIX_async_histogram}
const GL_ASYNC_HISTOGRAM_SGIX           = $832C;
const GL_MAX_ASYNC_HISTOGRAM_SGIX       = $832D;
{$endc}

{$ifc not undefined GL_SGIX_fog_scale and GL_SGIX_fog_scale}
const GL_FOG_SCALE_SGIX                 = $81FC;
const GL_FOG_SCALE_VALUE_SGIX           = $81FD;
{$endc}

{$ifc not undefined GL_SGIX_subsample and GL_SGIX_subsample}
const GL_PACK_SUBSAMPLE_RATE_SGIX       = $85A0;
const GL_UNPACK_SUBSAMPLE_RATE_SGIX     = $85A1;
const GL_PIXEL_SUBSAMPLE_4444_SGIX      = $85A2;
const GL_PIXEL_SUBSAMPLE_2424_SGIX      = $85A3;
const GL_PIXEL_SUBSAMPLE_4242_SGIX      = $85A4;
{$endc}

{$ifc not undefined GL_SGIX_ycrcba and GL_SGIX_ycrcba}
const GL_YCRCB_SGIX                     = $8318;
const GL_YCRCBA_SGIX                    = $8319;
{$endc}

{$ifc not undefined GL_SGIX_vertex_preclip and GL_SGIX_vertex_preclip}
const GL_VERTEX_PRECLIP_SGIX            = $83EE;
const GL_VERTEX_PRECLIP_HINT_SGIX       = $83EF;
{$endc}

{$ifc not undefined GL_SGIX_convolution_accuracy and GL_SGIX_convolution_accuracy}
const GL_CONVOLUTION_HINT_SGIX          = $8316;
{$endc}

{$ifc not undefined GL_SGIX_resample and GL_SGIX_resample}
const GL_PACK_RESAMPLE_SGIX             = $842C;
const GL_UNPACK_RESAMPLE_SGIX           = $842D;
const GL_RESAMPLE_REPLICATE_SGIX        = $842E;
const GL_RESAMPLE_ZERO_FILL_SGIX        = $842F;
const GL_RESAMPLE_DECIMATE_SGIX         = $8430;
{$endc}


{$ifc not undefined GL_SUN_global_alpha and GL_SUN_global_alpha}
const GL_GLOBAL_ALPHA_SUN               = $81D9;
const GL_GLOBAL_ALPHA_FACTOR_SUN        = $81DA;
{$endc}

{$ifc not undefined GL_SUN_triangle_list and GL_SUN_triangle_list}
const GL_RESTART_SUN                    = $01;
const GL_REPLACE_MIDDLE_SUN             = $02;
const GL_REPLACE_OLDEST_SUN             = $03;
const GL_TRIANGLE_LIST_SUN              = $81D7;
const GL_REPLACEMENT_CODE_SUN           = $81D8;
const GL_REPLACEMENT_CODE_ARRAY_SUN     = $85C0;
const GL_REPLACEMENT_CODE_ARRAY_TYPE_SUN = $85C1;
const GL_REPLACEMENT_CODE_ARRAY_STRIDE_SUN = $85C2;
const GL_REPLACEMENT_CODE_ARRAY_POINTER_SUN = $85C3;
const GL_R1UI_V3F_SUN                   = $85C4;
const GL_R1UI_C4UB_V3F_SUN              = $85C5;
const GL_R1UI_C3F_V3F_SUN               = $85C6;
const GL_R1UI_N3F_V3F_SUN               = $85C7;
const GL_R1UI_C4F_N3F_V3F_SUN           = $85C8;
const GL_R1UI_T2F_V3F_SUN               = $85C9;
const GL_R1UI_T2F_N3F_V3F_SUN           = $85CA;
const GL_R1UI_T2F_C4F_N3F_V3F_SUN       = $85CB;
{$endc}

{$ifc not undefined GL_SUN_convolution_border_modes and GL_SUN_convolution_border_modes}
const GL_WRAP_BORDER_SUN                = $81D4;
{$endc}

{$ifc not undefined GL_SUNX_constant_data and GL_SUNX_constant_data}
const GL_UNPACK_CONSTANT_DATA_SUNX      = $81D5;
const GL_TEXTURE_CONSTANT_DATA_SUNX     = $81D6;
{$endc}


{$ifc not undefined GL_WIN_phong_shading and GL_WIN_phong_shading}
const GL_PHONG_WIN                      = $80EA;
const GL_PHONG_HINT_WIN                 = $80EB;
{$endc}

{$ifc not undefined GL_WIN_specular_fog and GL_WIN_specular_fog}
const GL_FOG_SPECULAR_TEXTURE_WIN       = $80EC;
{$endc}


{$ifc not undefined GL_3DFX_texture_compression_FXT1 and GL_3DFX_texture_compression_FXT1}
const GL_COMPRESSED_RGB_FXT1_3DFX       = $86B0;
const GL_COMPRESSED_RGBA_FXT1_3DFX      = $86B1;
{$endc}

{$ifc not undefined GL_3DFX_multisample and GL_3DFX_multisample}
const GL_MULTISAMPLE_3DFX               = $86B2;
const GL_SAMPLE_BUFFERS_3DFX            = $86B3;
const GL_SAMPLES_3DFX                   = $86B4;
const GL_MULTISAMPLE_BIT_3DFX           = $20000000;
{$endc}


{***********************************************************}

{$ifc not undefined GL_ARB_multitexture and GL_ARB_multitexture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glActiveTextureARBProcPtr = procedure( param1 : GLenum );
	glClientActiveTextureARBProcPtr = procedure( param1 : GLenum );
	glMultiTexCoord1dARBProcPtr = procedure( param1 : GLenum; param2 : GLdouble );
	glMultiTexCoord1dvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLdouble );
	glMultiTexCoord1fARBProcPtr = procedure( param1 : GLenum; param2 : GLfloat );
	glMultiTexCoord1fvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glMultiTexCoord1iARBProcPtr = procedure( param1 : GLenum; param2 : GLint );
	glMultiTexCoord1ivARBProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
	glMultiTexCoord1sARBProcPtr = procedure( param1 : GLenum; param2 : GLshort );
	glMultiTexCoord1svARBProcPtr = procedure( param1 : GLenum; const param2 : PGLshort );
	glMultiTexCoord2dARBProcPtr = procedure( param1 : GLenum; GLdouble; param2 : GLdouble );
	glMultiTexCoord2dvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLdouble );
	glMultiTexCoord2fARBProcPtr = procedure( param1 : GLenum; GLfloat; param2 : GLfloat );
	glMultiTexCoord2fvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glMultiTexCoord2iARBProcPtr = procedure( param1 : GLenum; GLint; param2 : GLint );
	glMultiTexCoord2ivARBProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
	glMultiTexCoord2sARBProcPtr = procedure( param1 : GLenum; param2 : GLshort; param3 : GLshort );
	glMultiTexCoord2svARBProcPtr = procedure( param1 : GLenum; const param2 : PGLshort );
	glMultiTexCoord3dARBProcPtr = procedure( param1 : GLenum; param2 : GLdouble; param3 : GLdouble; param4 : GLdouble );
	glMultiTexCoord3dvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLdouble );
	glMultiTexCoord3fARBProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat );
	glMultiTexCoord3fvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glMultiTexCoord3iARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint );
	glMultiTexCoord3ivARBProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
	glMultiTexCoord3sARBProcPtr = procedure( param1 : GLenum; param2 : GLshort; param3 : GLshort; param4 : GLshort );
	glMultiTexCoord3svARBProcPtr = procedure( param1 : GLenum; const param2 : PGLshort );
	glMultiTexCoord4dARBProcPtr = procedure( param1 : GLenum; param2 : GLdouble; param3 : GLdouble; param4 : GLdouble; param5 : GLdouble );
	glMultiTexCoord4dvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLdouble );
	glMultiTexCoord4fARBProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat );
	glMultiTexCoord4fvARBProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glMultiTexCoord4iARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint );
	glMultiTexCoord4ivARBProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
	glMultiTexCoord4sARBProcPtr = procedure( param1 : GLenum; param2 : GLshort; param3 : GLshort; param4 : GLshort; param5 : GLshort );
	glMultiTexCoord4svARBProcPtr = procedure( param1 : GLenum; const param2 : PGLshort );
{$elsec}
procedure glActiveTextureARB( param1: GLenum ); external name '_glActiveTextureARB';
procedure glClientActiveTextureARB( param1 : GLenum ); external name '_glClientActiveTextureARB';
procedure glMultiTexCoord1dARB( param1 : GLenum; param2 : GLdouble ); external name '_glMultiTexCoord1dARB';
procedure glMultiTexCoord1dvARB( param1 : GLenum; const param2 : PGLdouble ); external name '_glMultiTexCoord1dvARB';
procedure glMultiTexCoord1fARB( param1 : GLenum; param2 : GLfloat ); external name '_glMultiTexCoord1fARB';
procedure glMultiTexCoord1fvARB( param1 : GLenum; const param2 : PGLfloat ); external name '_glMultiTexCoord1fvARB';
procedure glMultiTexCoord1iARB( param1 : GLenum; param2 : GLint ); external name '_glMultiTexCoord1iARB';
procedure glMultiTexCoord1ivARB( param1 : GLenum; const param2 : PGLint); external name '_glMultiTexCoord1ivARB';
procedure glMultiTexCoord1sARB( param1 : GLenum; param2 : GLshort ); external name '_glMultiTexCoord1sARB';
procedure glMultiTexCoord1svARB( param1 : GLenum; const param2 : PGLshort ); external name '_glMultiTexCoord1svARB';
procedure glMultiTexCoord2dARB( param1 : GLenum; param2: GLdouble; param3 : GLdouble ); external name '_glMultiTexCoord2dARB';
procedure glMultiTexCoord2dvARB( param1 : GLenum; const param2 : PGLdouble ); external name '_glMultiTexCoord2dvARB';
procedure glMultiTexCoord2fARB( param1 : GLenum; param2 : GLfloat; param3 : GLfloat ); external name '_glMultiTexCoord2fARB';
procedure glMultiTexCoord2fvARB( param1 : GLenum; const param2 : PGLfloat ); external name '_glMultiTexCoord2fvARB';
procedure glMultiTexCoord2iARB( param1 : GLenum; param2 : GLint; param3 : GLint ); external name '_glMultiTexCoord2iARB';
procedure glMultiTexCoord2ivARB( param1 : GLenum; const param2 : PGLint ); external name '_glMultiTexCoord2ivARB';
procedure glMultiTexCoord2sARB( param1 : GLenum; param2 : GLshort; param3 : GLshort ); external name '_glMultiTexCoord2sARB';
procedure glMultiTexCoord2svARB( param1 : GLenum; const param2 : PGLshort ); external name '_glMultiTexCoord2svARB';
procedure glMultiTexCoord3dARB( param1 : GLenum; param2 : GLdouble; param3 : GLdouble; param4 : GLdouble ); external name '_glMultiTexCoord3dARB';
procedure glMultiTexCoord3dvARB( param1 : GLenum; const param2 : PGLdouble ); external name '_glMultiTexCoord3dvARB';
procedure glMultiTexCoord3fARB( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat ); external name '_glMultiTexCoord3fARB';
procedure glMultiTexCoord3fvARB( param1 : GLenum; const param2 : PGLfloat ); external name '_glMultiTexCoord3fvARB';
procedure glMultiTexCoord3iARB( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint ); external name '_glMultiTexCoord3iARB';
procedure glMultiTexCoord3ivARB( param1 : GLenum; const param2 : PGLint ); external name '_glMultiTexCoord3ivARB';
procedure glMultiTexCoord3sARB( param1 : GLenum; param2 : GLshort; param3 : GLshort; param4 : GLshort ); external name '_glMultiTexCoord3sARB';
procedure glMultiTexCoord3svARB( param1 : GLenum; const param2 : PGLshort ); external name '_glMultiTexCoord3svARB';
procedure glMultiTexCoord4dARB( param1 : GLenum; param2 : GLdouble; param3 : GLdouble; param4 : GLdouble; param5 : GLdouble ); external name '_glMultiTexCoord4dARB';
procedure glMultiTexCoord4dvARB( param1 : GLenum; const param2 : PGLdouble ); external name '_glMultiTexCoord4dvARB';
procedure glMultiTexCoord4fARB( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat ); external name '_glMultiTexCoord4fARB';
procedure glMultiTexCoord4fvARB( param1 : GLenum; const param2 : PGLfloat ); external name '_glMultiTexCoord4fvARB';
procedure glMultiTexCoord4iARB( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint ); external name '_glMultiTexCoord4iARB';
procedure glMultiTexCoord4ivARB( param1 : GLenum; const param2 : PGLint ); external name '_glMultiTexCoord4ivARB';
procedure glMultiTexCoord4sARB( param1 : GLenum; param2 : GLshort; param3 : GLshort; param4 : GLshort; param5 : GLshort ); external name '_glMultiTexCoord4sARB';
procedure glMultiTexCoord4svARB( param1 : GLenum; const param2 : PGLshort ); external name '_glMultiTexCoord4svARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_transpose_matrix and GL_ARB_transpose_matrix}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glLoadTransposeMatrixfARBProcPtr = procedure( const param1 : PGLfloat );
	glLoadTransposeMatrixdARBProcPtr = procedure( const param1 : PGLdouble );
	glMultTransposeMatrixfARBProcPtr = procedure( const param1 : PGLfloat );
	glMultTransposeMatrixdARBProcPtr = procedure( const param1 : PGLdouble );
{$elsec}
procedure glLoadTransposeMatrixfARB( const param1 : PGLfloat ); external name '_glLoadTransposeMatrixfARB';
procedure glLoadTransposeMatrixdARB( const param1 : PGLdouble ); external name '_glLoadTransposeMatrixdARB';
procedure glMultTransposeMatrixfARB( const param1 : PGLfloat ); external name '_glMultTransposeMatrixfARB';
procedure glMultTransposeMatrixdARB( const param1 : PGLdouble ); external name '_glMultTransposeMatrixdARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_multisample and GL_ARB_multisample}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glSampleCoverageARBProcPtr = procedure( param1 : GLclampf; param2 : GLboolean );
	glSamplePassARBProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glSampleCoverageARB( param1 : GLclampf; param2 : GLboolean ); external name '_glSampleCoverageARB';
procedure glSamplePassARB( param1 : GLenum ); external name '_glSamplePassARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_texture_compression and GL_ARB_texture_compression}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glCompressedTexImage3DARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLsizei; param7 : GLint; param8 : GLsizei; const param9 : UnivPtr );
	glCompressedTexImage2DARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLint; param7 : GLsizei; const param8 : UnivPtr );
	glCompressedTexImage1DARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLint; param6 : GLsizei; const param7 : UnivPtr );
	glCompressedTexSubImage3DARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLsizei; param8 : GLsizei; param9 : GLenum; param10 : GLsizei; const param11 : UnivPtr );
	glCompressedTexSubImage2DARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLsizei; param6 : GLsizei; param7 : GLenum; param8 : GLsizei; const param9 : UnivPtr );
	glCompressedTexSubImage1DARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLsizei; param5 : GLenum; param6 : GLsizei; const param7 : UnivPtr );
	glGetCompressedTexImageARBProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : UnivPtr );
{$elsec}
procedure glCompressedTexImage3DARB( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLsizei; param7 : GLint; param8 : GLsizei; const param9 : UnivPtr ); external name '_glCompressedTexImage3DARB';
procedure glCompressedTexImage2DARB( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLint; param7 : GLsizei; const param8 : UnivPtr ); external name '_glCompressedTexImage2DARB';
procedure glCompressedTexImage1DARB( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLint; param6 : GLsizei; const param7 : UnivPtr ); external name '_glCompressedTexImage1DARB';
procedure glCompressedTexSubImage3DARB( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLsizei; param8 : GLsizei; param9 : GLenum; param10 : GLsizei; const param11 : UnivPtr ); external name '_glCompressedTexSubImage3DARB';
procedure glCompressedTexSubImage2DARB( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLsizei; param6 : GLsizei; param7 : GLenum; param8 : GLsizei; const param9 : UnivPtr ); external name '_glCompressedTexSubImage2DARB';
procedure glCompressedTexSubImage1DARB( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLsizei; param5 : GLenum; param6 : GLsizei; const param7 : UnivPtr ); external name '_glCompressedTexSubImage1DARB';
procedure glGetCompressedTexImageARB( param1 : GLenum; param2 : GLint; param3 : UnivPtr ); external name '_glGetCompressedTexImageARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_vertex_blend and GL_ARB_vertex_blend}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glWeightbvARBProcPtr = procedure( param1 : GLint; const param2 : PGLbyte );
	glWeightsvARBProcPtr = procedure( param1 : GLint; const param2 : PGLshort );
	glWeightivARBProcPtr = procedure( param1 : GLint; const param2 : PGLint );
	glWeightfvARBProcPtr = procedure( param1 : GLint; const param2 : PGLfloat );
	glWeightdvARBProcPtr = procedure( param1 : GLint; const param2 : PGLdouble );
	glWeightubvARBProcPtr = procedure( param1 : GLint; const param2 : PGLubyte );
	glWeightusvARBProcPtr = procedure( param1 : GLint; const param2 : PGLushort );
	glWeightuivARBProcPtr = procedure( param1 : GLint; const param2 : PGLuint );
	glWeightPointerARBProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLsizei; const param4 : UnivPtr );
	glVertexBlendARBProcPtr = procedure( param1 : GLint );
{$elsec}
procedure glWeightbvARB( param1 : GLint; const param2 : PGLbyte ); external name '_glWeightbvARB';
procedure glWeightsvARB( param1 : GLint; const param2 : PGLshort ); external name '_glWeightsvARB';
procedure glWeightivARB( param1 : GLint; const param2 : PGLint ); external name '_glWeightivARB';
procedure glWeightfvARB( param1 : GLint; const param2 : PGLfloat ); external name '_glWeightfvARB';
procedure glWeightdvARB( param1 : GLint; const param2 : PGLdouble ); external name '_glWeightdvARB';
procedure glWeightubvARB( param1 : GLint; const param2 : PGLubyte ); external name '_glWeightubvARB';
procedure glWeightusvARB( param1 : GLint; const param2 : PGLushort ); external name '_glWeightusvARB';
procedure glWeightuivARB( param1 : GLint; const param2 : PGLuint ); external name '_glWeightuivARB';
procedure glWeightPointerARB( param1 : GLint; param2 : GLenum; param3 : GLsizei; const param4 : UnivPtr ); external name '_glWeightPointerARB';
procedure glVertexBlendARB( param1 : GLint ); external name '_glVertexBlendARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_window_pos and GL_ARB_window_pos}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glWindowPos2dARBProcPtr = procedure( param1 : GLdouble; param2 : GLdouble );
	glWindowPos2dvARBProcPtr = procedure( const param1 : PGLdouble );
	glWindowPos2fARBProcPtr = procedure( param1 : GLfloat; param2 : GLfloat );
	glWindowPos2fvARBProcPtr = procedure( const param1 : PGLfloat );
	glWindowPos2iARBProcPtr = procedure( param1 : GLint; param2 : GLint );
	glWindowPos2ivARBProcPtr = procedure( const param1 : PGLint );
	glWindowPos2sARBProcPtr = procedure( param1 : GLshort; param2 : GLshort );
	glWindowPos2svARBProcPtr = procedure( const param1 : PGLshort );
	glWindowPos3dARBProcPtr = procedure( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble );
	glWindowPos3dvARBProcPtr = procedure( const param1 : PGLdouble );
	glWindowPos3fARBProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat );
	glWindowPos3fvARBProcPtr = procedure( const param1 : PGLfloat );
	glWindowPos3iARBProcPtr = procedure( param1 : GLint; param2 : GLint; param3 : GLint );
	glWindowPos3ivARBProcPtr = procedure( const param1 : PGLint );
	glWindowPos3sARBProcPtr = procedure( param1 : GLshort; param2 : GLshort; param3 : GLshort );
	glWindowPos3svARBProcPtr = procedure( const param1 : PGLshort );
{$elsec}
procedure glWindowPos2dARB( param1 : GLdouble; param2 : GLdouble ); external name '_glWindowPos2dARB';
procedure glWindowPos2dvARB( const param1 : PGLdouble ); external name '_glWindowPos2dvARB';
procedure glWindowPos2fARB( param1 : GLfloat; param2 : GLfloat ); external name '_glWindowPos2fARB';
procedure glWindowPos2fvARB( const param1 : PGLfloat ); external name '_glWindowPos2fvARB';
procedure glWindowPos2iARB( param1 : GLint; param2 : GLint ); external name '_glWindowPos2iARB';
procedure glWindowPos2ivARB( const param1 : PGLint ); external name '_glWindowPos2ivARB';
procedure glWindowPos2sARB( param1 : GLshort; param2 : GLshort ); external name '_glWindowPos2sARB';
procedure glWindowPos2svARB( const param1 : PGLshort ); external name '_glWindowPos2svARB';
procedure glWindowPos3dARB( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble ); external name '_glWindowPos3dARB';
procedure glWindowPos3dvARB( const param1 : PGLdouble ); external name '_glWindowPos3dvARB';
procedure glWindowPos3fARB( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat ); external name '_glWindowPos3fARB';
procedure glWindowPos3fvARB( const param1 : PGLfloat ); external name '_glWindowPos3fvARB';
procedure glWindowPos3iARB( param1 : GLint; param2 : GLint; param3 : GLint ); external name '_glWindowPos3iARB';
procedure glWindowPos3ivARB( const param1 : PGLint ); external name '_glWindowPos3ivARB';
procedure glWindowPos3sARB( param1 : GLshort; param2 : GLshort; param3 : GLshort ); external name '_glWindowPos3sARB';
procedure glWindowPos3svARB( const param1 : PGLshort ); external name '_glWindowPos3svARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_occlusion_query and GL_ARB_occlusion_query}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGenQueriesARBProcPtr = procedure( n: GLsizei; ids: PGLuint );
	glDeleteQueriesARBProcPtr = procedure( n: GLsizei; const ids: PGLuint );
	glIsQueryARBProcPtr = function( id_: GLuint ): GLboolean;
	glBeginQueryARBProcPtr = procedure( target: GLenum; id_: GLuint );
	glEndQueryARBProcPtr = procedure( target: GLenum );
	glGetQueryivARBProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetQueryObjectivARBProcPtr = procedure( id_: GLuint; pname: GLenum; params: PGLint );
	glGetQueryObjectuivARBProcPtr = procedure( id_: GLuint; pname: GLenum; params: PGLuint );
{$elsec}
procedure glGenQueriesARB( n: GLsizei; ids: PGLuint ); external name '_glGenQueriesARB';
procedure glDeleteQueriesARB( n: GLsizei; const ids: PGLuint ); external name '_glDeleteQueriesARB';
function glIsQueryARB( id_: GLuint ): GLboolean; external name '_glIsQueryARB';
procedure glBeginQueryARB( target: GLenum; id_: GLuint ); external name '_glBeginQueryARB';
procedure glEndQueryARB( target: GLenum ); external name '_glEndQueryARB';
procedure glGetQueryivARB( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetQueryivARB';
procedure glGetQueryObjectivARB( id_: GLuint; pname: GLenum; params: PGLint ); external name '_glGetQueryObjectivARB';
procedure glGetQueryObjectuivARB( id_: GLuint; pname: GLenum; params: PGLuint ); external name '_glGetQueryObjectuivARB';
{$endc}
{$endc}

{$ifc not undefined GL_ARB_point_parameters and GL_ARB_point_parameters}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPointParameterfARBProcPtr = procedure( pname: GLenum; param: GLfloat );
	glPointParameterfvARBProcPtr = procedure( pname: GLenum; const params: PGLfloat );
{$elsec}
procedure glPointParameterfARB( pname: GLenum; param: GLfloat ); external name '_glPointParameterfARB';
procedure glPointParameterfvARB( pname: GLenum; const params: PGLfloat ); external name '_glPointParameterfvARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_vertex_program and GL_ARB_vertex_program}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBindProgramARBProcPtr = procedure( target: GLenum; program_: GLuint );
	glDeleteProgramsARBProcPtr = procedure( n: GLsizei; const programs: PGLuint );
	glGenProgramsARBProcPtr = procedure( n: GLsizei; programs: PGLuint );
	glIsProgramARBProcPtr = function( program_: GLuint ): GLboolean;

type
	glProgramEnvParameter4dARBProcPtr = procedure( target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glProgramEnvParameter4dvARBProcPtr = procedure( target: GLenum; index: GLuint; const params: PGLdouble );
	glProgramEnvParameter4fARBProcPtr = procedure( target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glProgramEnvParameter4fvARBProcPtr = procedure( target: GLenum; index: GLuint; const params: PGLfloat );
	glProgramLocalParameter4dARBProcPtr = procedure( target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glProgramLocalParameter4dvARBProcPtr = procedure( target: GLenum; index: GLuint; const params: PGLdouble );
	glProgramLocalParameter4fARBProcPtr = procedure( target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glProgramLocalParameter4fvARBProcPtr = procedure( target: GLenum; index: GLuint; const params: PGLfloat );

{$ifc not undefined GL_EXT_gpu_program_parameters and GL_EXT_gpu_program_parameters}
type
	glProgramEnvParameters4fvEXTProcPtr = procedure( target: GLenum; index: GLuint; count: GLsizei; const params: PGLfloat );
	glProgramLocalParameters4fvEXTProcPtr = procedure( target: GLenum; index: GLuint; count: GLsizei; const params: PGLfloat );
{$endc}

type
	glGetProgramEnvParameterdvARBProcPtr = procedure( target: GLenum; index: GLuint; params: PGLdouble );
	glGetProgramEnvParameterfvARBProcPtr = procedure( target: GLenum; index: GLuint; params: PGLfloat );
	glGetProgramLocalParameterdvARBProcPtr = procedure( target: GLenum; index: GLuint; params: PGLdouble );
	glGetProgramLocalParameterfvARBProcPtr = procedure( target: GLenum; index: GLuint; params: PGLfloat );

type
	glProgramStringARBProcPtr = procedure( target: GLenum; format: GLenum; len: GLsizei; const strng: PChar );
	glGetProgramStringARBProcPtr = procedure( target: GLenum; pname: GLenum; strng: PChar );

type
	glGetProgramivARBProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
{$elsec}
procedure glBindProgramARB( target: GLenum; program_: GLuint ); external name '_glBindProgramARB';
procedure glDeleteProgramsARB( n: GLsizei; const programs: PGLuint ); external name '_glDeleteProgramsARB';
procedure glGenProgramsARB( n: GLsizei; programs: PGLuint ); external name '_glGenProgramsARB';
function glIsProgramARB( program_: GLuint ): GLboolean; external name '_glIsProgramARB';

procedure glProgramEnvParameter4dARB( target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glProgramEnvParameter4dARB';
procedure glProgramEnvParameter4dvARB( target: GLenum; index: GLuint; const params: PGLdouble ); external name '_glProgramEnvParameter4dvARB';
procedure glProgramEnvParameter4fARB( target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glProgramEnvParameter4fARB';
procedure glProgramEnvParameter4fvARB( target: GLenum; index: GLuint; const params: PGLfloat ); external name '_glProgramEnvParameter4fvARB';
procedure glProgramLocalParameter4dARB( target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glProgramLocalParameter4dARB';
procedure glProgramLocalParameter4dvARB( target: GLenum; index: GLuint; const params: PGLdouble ); external name '_glProgramLocalParameter4dvARB';
procedure glProgramLocalParameter4fARB( target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glProgramLocalParameter4fARB';
procedure glProgramLocalParameter4fvARB( target: GLenum; index: GLuint; const params: PGLfloat ); external name '_glProgramLocalParameter4fvARB';

procedure glGetProgramEnvParameterdvARB( target: GLenum; index: GLuint; params: PGLdouble ); external name '_glGetProgramEnvParameterdvARB';
procedure glGetProgramEnvParameterfvARB( target: GLenum; index: GLuint; params: PGLfloat ); external name '_glGetProgramEnvParameterfvARB';
{$ifc not undefined GL_EXT_gpu_program_parameters and GL_EXT_gpu_program_parameters}
procedure glProgramEnvParameters4fvEXT( target: GLenum; index: GLuint; count: GLsizei; const params: PGLfloat ); external name '_glProgramEnvParameters4fvEXT';
procedure glProgramLocalParameters4fvEXT( target: GLenum; index: GLuint; count: GLsizei; const params: PGLfloat ); external name '_glProgramLocalParameters4fvEXT';
{$endc}

procedure glGetProgramLocalParameterdvARB( target: GLenum; index: GLuint; params: PGLdouble ); external name '_glGetProgramLocalParameterdvARB';
procedure glGetProgramLocalParameterfvARB( target: GLenum; index: GLuint; params: PGLfloat ); external name '_glGetProgramLocalParameterfvARB';

procedure glProgramStringARB( target: GLenum; format: GLenum; len: GLsizei; const strng: PChar ); external name '_glProgramStringARB';
procedure glGetProgramStringARB( target: GLenum; pname: GLenum; strng : PChar ); external name '_glGetProgramStringARB';

procedure glGetProgramivARB( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetProgramivARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_ARB_vertex_shader and GL_ARB_vertex_shader or defined GL_ARB_vertex_program and GL_ARB_vertex_program}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}

type
	glVertexAttrib1dARBProcPtr = procedure( index: GLuint; x: GLdouble );
	glVertexAttrib1dvARBProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib1fARBProcPtr = procedure( index: GLuint; x: GLfloat );
	glVertexAttrib1fvARBProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib1sARBProcPtr = procedure( index: GLuint; x: GLshort );
	glVertexAttrib1svARBProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib2dARBProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble );
	glVertexAttrib2dvARBProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib2fARBProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat );
	glVertexAttrib2fvARBProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib2sARBProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort );
	glVertexAttrib2svARBProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib3dARBProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble );
	glVertexAttrib3dvARBProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib3fARBProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat );
	glVertexAttrib3fvARBProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib3sARBProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort; z: GLshort );
	glVertexAttrib3svARBProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib4NbvARBProcPtr = procedure( index: GLuint; const v: PGLbyte );
	glVertexAttrib4NivARBProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttrib4NsvARBProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib4NubARBProcPtr = procedure( index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte );
	glVertexAttrib4NubvARBProcPtr = procedure( index: GLuint; const v: PGLubyte );
	glVertexAttrib4NuivARBProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttrib4NusvARBProcPtr = procedure( index: GLuint; const v: PGLushort );
	glVertexAttrib4bvARBProcPtr = procedure( index: GLuint; const v: PGLbyte );
	glVertexAttrib4dARBProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glVertexAttrib4dvARBProcPtr = procedure( index: GLuint; const v: PGLdouble );
	glVertexAttrib4fARBProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glVertexAttrib4fvARBProcPtr = procedure( index: GLuint; const v: PGLfloat );
	glVertexAttrib4ivARBProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttrib4sARBProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort );
	glVertexAttrib4svARBProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttrib4ubvARBProcPtr = procedure( index: GLuint; const v: PGLubyte );
	glVertexAttrib4uivARBProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttrib4usvARBProcPtr = procedure( index: GLuint; const v: PGLushort );
	glVertexAttribPointerARBProcPtr = procedure( index: GLuint; size: GLint; typ: GLenum; normalized: GLboolean; stride: GLsizei; const pointr: UnivPtr );

type
	glDisableVertexAttribArrayARBProcPtr = procedure( index: GLuint );
	glEnableVertexAttribArrayARBProcPtr = procedure( index: GLuint );

type
	glGetVertexAttribPointervARBProcPtr = procedure( index: GLuint; pname: GLenum; pointr: UnivPtrPtr );
	glGetVertexAttribdvARBProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLdouble );
	glGetVertexAttribfvARBProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLfloat );
	glGetVertexAttribivARBProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLint );

{$elsec}

procedure glVertexAttrib1dARB( index: GLuint; x: GLdouble ); external name '_glVertexAttrib1dARB';
procedure glVertexAttrib1dvARB( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib1dvARB';
procedure glVertexAttrib1fARB( index: GLuint; x: GLfloat ); external name '_glVertexAttrib1fARB';
procedure glVertexAttrib1fvARB( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib1fvARB';
procedure glVertexAttrib1sARB( index: GLuint; x: GLshort ); external name '_glVertexAttrib1sARB';
procedure glVertexAttrib1svARB( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib1svARB';
procedure glVertexAttrib2dARB( index: GLuint; x: GLdouble; y: GLdouble ); external name '_glVertexAttrib2dARB';
procedure glVertexAttrib2dvARB( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib2dvARB';
procedure glVertexAttrib2fARB( index: GLuint; x: GLfloat; y: GLfloat ); external name '_glVertexAttrib2fARB';
procedure glVertexAttrib2fvARB( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib2fvARB';
procedure glVertexAttrib2sARB( index: GLuint; x: GLshort; y: GLshort ); external name '_glVertexAttrib2sARB';
procedure glVertexAttrib2svARB( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib2svARB';
procedure glVertexAttrib3dARB( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glVertexAttrib3dARB';
procedure glVertexAttrib3dvARB( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib3dvARB';
procedure glVertexAttrib3fARB( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glVertexAttrib3fARB';
procedure glVertexAttrib3fvARB( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib3fvARB';
procedure glVertexAttrib3sARB( index: GLuint; x: GLshort; y: GLshort; z: GLshort ); external name '_glVertexAttrib3sARB';
procedure glVertexAttrib3svARB( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib3svARB';
procedure glVertexAttrib4NbvARB( index: GLuint; const v: PGLbyte ); external name '_glVertexAttrib4NbvARB';
procedure glVertexAttrib4NivARB( index: GLuint; const v: PGLint ); external name '_glVertexAttrib4NivARB';
procedure glVertexAttrib4NsvARB( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib4NsvARB';
procedure glVertexAttrib4NubARB( index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte ); external name '_glVertexAttrib4NubARB';
procedure glVertexAttrib4NubvARB( index: GLuint; const v: PGLubyte ); external name '_glVertexAttrib4NubvARB';
procedure glVertexAttrib4NuivARB( index: GLuint; const v: PGLuint ); external name '_glVertexAttrib4NuivARB';
procedure glVertexAttrib4NusvARB( index: GLuint; const v: PGLushort ); external name '_glVertexAttrib4NusvARB';
procedure glVertexAttrib4bvARB( index: GLuint; const v: PGLbyte ); external name '_glVertexAttrib4bvARB';
procedure glVertexAttrib4dARB( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glVertexAttrib4dARB';
procedure glVertexAttrib4dvARB( index: GLuint; const v: PGLdouble ); external name '_glVertexAttrib4dvARB';
procedure glVertexAttrib4fARB( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glVertexAttrib4fARB';
procedure glVertexAttrib4fvARB( index: GLuint; const v: PGLfloat ); external name '_glVertexAttrib4fvARB';
procedure glVertexAttrib4ivARB( index: GLuint; const v: PGLint ); external name '_glVertexAttrib4ivARB';
procedure glVertexAttrib4sARB( index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort ); external name '_glVertexAttrib4sARB';
procedure glVertexAttrib4svARB( index: GLuint; const v: PGLshort ); external name '_glVertexAttrib4svARB';
procedure glVertexAttrib4ubvARB( index: GLuint; const v: PGLubyte ); external name '_glVertexAttrib4ubvARB';
procedure glVertexAttrib4uivARB( index: GLuint; const v: PGLuint ); external name '_glVertexAttrib4uivARB';
procedure glVertexAttrib4usvARB( index: GLuint; const v: PGLushort ); external name '_glVertexAttrib4usvARB';
procedure glVertexAttribPointerARB( index: GLuint; size: GLint; typ: GLenum; normalized: GLboolean; stride: GLsizei; const pointr: UnivPtr ); external name '_glVertexAttribPointerARB';

procedure glDisableVertexAttribArrayARB( index: GLuint ); external name '_glDisableVertexAttribArrayARB';
procedure glEnableVertexAttribArrayARB( index: GLuint ); external name '_glEnableVertexAttribArrayARB';

procedure glGetVertexAttribPointervARB( index: GLuint; pname: GLenum; pointr: UnivPtrPtr ); external name '_glGetVertexAttribPointervARB';
procedure glGetVertexAttribdvARB( index: GLuint; pname: GLenum; params: PGLdouble ); external name '_glGetVertexAttribdvARB';
procedure glGetVertexAttribfvARB( index: GLuint; pname: GLenum; params: PGLfloat ); external name '_glGetVertexAttribfvARB';
procedure glGetVertexAttribivARB( index: GLuint; pname: GLenum; params: PGLint ); external name '_glGetVertexAttribivARB';

{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_shader_objects and GL_ARB_shader_objects}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glDeleteObjectARBProcPtr = procedure( obj: GLhandleARB );
	glGetHandleARBProcPtr = function( pname: GLenum ): GLhandleARB;
	glDetachObjectARBProcPtr = procedure( containerObj: GLhandleARB; attachedObj: GLhandleARB );
	glCreateShaderObjectARBProcPtr = function( shaderType: GLenum ): GLhandleARB;

{GPC-ONLY-START}	
	glShaderSourceARBProcPtr = procedure( shaderObj: GLhandleARB; count: GLsizei; {const} strng: CStringPtrPtr; const length: PGLint );
{GPC-ONLY-FINISH}
	glShaderSourceARBProcPtr = procedure( shaderObj: GLhandleARB; count: GLsizei; {const} strng: PPChar; const length: PGLint );

	glCompileShaderARBProcPtr = procedure( shaderObj: GLhandleARB );
	glCreateProgramObjectARBProcPtr = function: GLhandleARB;
	glAttachObjectARBProcPtr = procedure( containerObj: GLhandleARB; obj: GLhandleARB );
	glLinkProgramARBProcPtr = procedure( programObj: GLhandleARB );
	glUseProgramObjectARBProcPtr = procedure( programObj: GLhandleARB );
	glValidateProgramARBProcPtr = procedure( programObj: GLhandleARB );
	glUniform1fARBProcPtr = procedure( location: GLint; v0: GLfloat );
	glUniform2fARBProcPtr = procedure( location: GLint; v0: GLfloat; v1: GLfloat );
	glUniform3fARBProcPtr = procedure( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat );
	glUniform4fARBProcPtr = procedure( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat; v3: GLfloat );
	glUniform1iARBProcPtr = procedure( location: GLint; v0: GLint );
	glUniform2iARBProcPtr = procedure( location: GLint; v0: GLint; v1: GLint );
	glUniform3iARBProcPtr = procedure( location: GLint; v0: GLint; v1: GLint; v2: GLint );
	glUniform4iARBProcPtr = procedure( location: GLint; v0: GLint; v1: GLint; v2: GLint; v3: GLint );
	glUniform1fvARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform2fvARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform3fvARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform4fvARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLfloat );
	glUniform1ivARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniform2ivARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniform3ivARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniform4ivARBProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLint );
	glUniformMatrix2fvARBProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix3fvARBProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glUniformMatrix4fvARBProcPtr = procedure( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat );
	glGetObjectParameterfvARBProcPtr = procedure( obj: GLhandleARB; pname: GLenum; params: PGLfloat );
	glGetObjectParameterivARBProcPtr = procedure( obj: GLhandleARB; pname: GLenum; params: PGLint );
	glGetInfoLogARBProcPtr = procedure( obj: GLhandleARB; maxLength: GLsizei; length: PGLsizei; infoLog: PChar );
	glGetAttachedObjectsARBProcPtr = procedure( containerObj: GLhandleARB; maxCount: GLsizei; count: PGLsizei; var obj: GLhandleARB );
	glGetUniformLocationARBProcPtr = function( programObj: GLhandleARB; const name: PChar ): GLint;
	glGetActiveUniformARBProcPtr = procedure( programObj: GLhandleARB; index: GLuint; maxLength: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar );
	glGetUniformfvARBProcPtr = procedure( programObj: GLhandleARB; location: GLint; params: PGLfloat );
	glGetUniformivARBProcPtr = procedure( programObj: GLhandleARB; location: GLint; params: PGLint );
	glGetShaderSourceARBProcPtr = procedure( obj: GLhandleARB; maxLength: GLsizei; length: PGLsizei; source: PChar );
{$elsec}
procedure glDeleteObjectARB( obj: GLhandleARB ); external name '_glDeleteObjectARB';
function glGetHandleARB( pname: GLenum ): GLhandleARB; external name '_glGetHandleARB';
procedure glDetachObjectARB( containerObj: GLhandleARB; attachedObj: GLhandleARB ); external name '_glDetachObjectARB';
function glCreateShaderObjectARB( shaderType: GLenum ): GLhandleARB; external name '_glCreateShaderObjectARB';

procedure glShaderSourceARB( shaderObj: GLhandleARB; count: GLsizei; {const} strng: PPChar; const length: PGLint ); external name '_glShaderSourceARB';

procedure glCompileShaderARB( shaderObj: GLhandleARB ); external name '_glCompileShaderARB';
function glCreateProgramObjectARB: GLhandleARB; external name '_glCreateProgramObjectARB';
procedure glAttachObjectARB( containerObj: GLhandleARB; obj: GLhandleARB ); external name '_glAttachObjectARB';
procedure glLinkProgramARB( programObj: GLhandleARB ); external name '_glLinkProgramARB';
procedure glUseProgramObjectARB( programObj: GLhandleARB ); external name '_glUseProgramObjectARB';
procedure glValidateProgramARB( programObj: GLhandleARB ); external name '_glValidateProgramARB';
procedure glUniform1fARB( location: GLint; v0: GLfloat ); external name '_glUniform1fARB';
procedure glUniform2fARB( location: GLint; v0: GLfloat; v1: GLfloat ); external name '_glUniform2fARB';
procedure glUniform3fARB( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat ); external name '_glUniform3fARB';
procedure glUniform4fARB( location: GLint; v0: GLfloat; v1: GLfloat; v2: GLfloat; v3: GLfloat ); external name '_glUniform4fARB';
procedure glUniform1iARB( location: GLint; v0: GLint ); external name '_glUniform1iARB';
procedure glUniform2iARB( location: GLint; v0: GLint; v1: GLint ); external name '_glUniform2iARB';
procedure glUniform3iARB( location: GLint; v0: GLint; v1: GLint; v2: GLint ); external name '_glUniform3iARB';
procedure glUniform4iARB( location: GLint; v0: GLint; v1: GLint; v2: GLint; v3: GLint ); external name '_glUniform4iARB';
procedure glUniform1fvARB( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform1fvARB';
procedure glUniform2fvARB( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform2fvARB';
procedure glUniform3fvARB( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform3fvARB';
procedure glUniform4fvARB( location: GLint; count: GLsizei; const value: PGLfloat ); external name '_glUniform4fvARB';
procedure glUniform1ivARB( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform1ivARB';
procedure glUniform2ivARB( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform2ivARB';
procedure glUniform3ivARB( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform3ivARB';
procedure glUniform4ivARB( location: GLint; count: GLsizei; const value: PGLint ); external name '_glUniform4ivARB';
procedure glUniformMatrix2fvARB( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix2fvARB';
procedure glUniformMatrix3fvARB( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix3fvARB';
procedure glUniformMatrix4fvARB( location: GLint; count: GLsizei; transpose: GLboolean; const value: PGLfloat ); external name '_glUniformMatrix4fvARB';
procedure glGetObjectParameterfvARB( obj: GLhandleARB; pname: GLenum; params: PGLfloat ); external name '_glGetObjectParameterfvARB';
procedure glGetObjectParameterivARB( obj: GLhandleARB; pname: GLenum; params: PGLint ); external name '_glGetObjectParameterivARB';
procedure glGetInfoLogARB( obj: GLhandleARB; maxLength: GLsizei; length: PGLsizei; infoLog: PChar ); external name '_glGetInfoLogARB';
procedure glGetAttachedObjectsARB( containerObj: GLhandleARB; maxCount: GLsizei; count: PGLsizei; var obj: GLhandleARB ); external name '_glGetAttachedObjectsARB';
function glGetUniformLocationARB( programObj: GLhandleARB; const name: PChar ): GLint; external name '_glGetUniformLocationARB';
procedure glGetActiveUniformARB( programObj: GLhandleARB; index: GLuint; maxLength: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar ); external name '_glGetActiveUniformARB';
procedure glGetUniformfvARB( programObj: GLhandleARB; location: GLint; params: PGLfloat ); external name '_glGetUniformfvARB';
procedure glGetUniformivARB( programObj: GLhandleARB; location: GLint; params: PGLint ); external name '_glGetUniformivARB';
procedure glGetShaderSourceARB( obj: GLhandleARB; maxLength: GLsizei; length: PGLsizei; source: PChar ); external name '_glGetShaderSourceARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_vertex_shader and GL_ARB_vertex_shader}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBindAttribLocationARBProcPtr = procedure( programObj: GLhandleARB; index: GLuint; const name: PChar );
	glGetActiveAttribARBProcPtr = procedure( programObj: GLhandleARB; index: GLuint; maxLength: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar );
	glGetAttribLocationARBProcPtr = function( programObj: GLhandleARB; const name: PChar ): GLint;
{$elsec}
procedure glBindAttribLocationARB( programObj: GLhandleARB; index: GLuint; const name: PChar ); external name '_glBindAttribLocationARB';
procedure glGetActiveAttribARB( programObj: GLhandleARB; index: GLuint; maxLength: GLsizei; length: PGLsizei; size: PGLint; typ: PGLenum; name: PChar ); external name '_glGetActiveAttribARB';
function glGetAttribLocationARB( programObj: GLhandleARB; const name: PChar ): GLint; external name '_glGetAttribLocationARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_vertex_buffer_object and GL_ARB_vertex_buffer_object}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBindBufferARBProcPtr = procedure( target: GLenum; buffer: GLuint );
	glDeleteBuffersARBProcPtr = procedure( n: GLsizei; const buffers: PGLuint );
	glGenBuffersARBProcPtr = procedure( n: GLsizei; buffers: PGLuint );
	glIsBufferARBProcPtr = function( buffer: GLuint ): GLboolean;
	glBufferDataARBProcPtr = procedure( target: GLenum; size: GLsizeiptrARB; const data: UnivPtr; usage: GLenum );
	glBufferSubDataARBProcPtr = procedure( target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB; const data: UnivPtr );
	glGetBufferSubDataARBProcPtr = procedure( target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB; data: UnivPtr );
	glMapBufferARBProcPtr = function( target: GLenum; access: GLenum ): UnivPtr;
	glUnmapBufferARBProcPtr = function( target: GLenum ): GLboolean;
	glGetBufferParameterivARBProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetBufferPointervARBProcPtr = procedure( target: GLenum; pname: GLenum; params: UnivPtrPtr );
{$elsec}
procedure glBindBufferARB( target: GLenum; buffer: GLuint ); external name '_glBindBufferARB';
procedure glDeleteBuffersARB( n: GLsizei; const buffers: PGLuint ); external name '_glDeleteBuffersARB';
procedure glGenBuffersARB( n: GLsizei; buffers: PGLuint ); external name '_glGenBuffersARB';
function glIsBufferARB( buffer: GLuint ): GLboolean; external name '_glIsBufferARB';
procedure glBufferDataARB( target: GLenum; size: GLsizeiptrARB; const data: UnivPtr; usage: GLenum ); external name '_glBufferDataARB';
procedure glBufferSubDataARB( target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB; const data: UnivPtr ); external name '_glBufferSubDataARB';
procedure glGetBufferSubDataARB( target: GLenum; offset: GLintptrARB; size: GLsizeiptrARB; data: UnivPtr ); external name '_glGetBufferSubDataARB';
function glMapBufferARB( target: GLenum; access: GLenum ): UnivPtr; external name '_glMapBufferARB';
function glUnmapBufferARB( target: GLenum ): GLboolean; external name '_glUnmapBufferARB';
procedure glGetBufferParameterivARB( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetBufferParameterivARB';
procedure glGetBufferPointervARB( target: GLenum; pname: GLenum; params: UnivPtrPtr ); external name '_glGetBufferPointervARB';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ARB_draw_buffers and GL_ARB_draw_buffers}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glDrawBuffersARBProcPtr = procedure( n: GLsizei; const bufs: PGLenum );
{$elsec}
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
procedure glDrawBuffersARB( n: GLsizei; const bufs: PGLenum ); external name '_glDrawBuffersARB';
{$endc}

{$ifc not undefined GL_ARB_color_buffer_float and GL_ARB_color_buffer_float}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glClampColorARBProcPtr = procedure( target: GLenum; clamp: GLenum );
{$elsec}
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
procedure glClampColorARB( target: GLenum; clamp: GLenum ); external name '_glClampColorARB';
{$endc}


{$ifc not undefined GL_EXT_blend_color and GL_EXT_blend_color}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBlendColorEXTProcPtr = procedure( param1 : GLclampf; param2 : GLclampf; param3 : GLclampf; param4 : GLclampf );
{$elsec}
procedure glBlendColorEXT( param1 : GLclampf; param2 : GLclampf; param3 : GLclampf; param4 : GLclampf ); external name '_glBlendColorEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_polygon_offset and GL_EXT_polygon_offset}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPolygonOffsetEXTProcPtr = procedure( param1 : GLfloat; param2 : GLfloat );
{$elsec}
procedure glPolygonOffsetEXT( param1 : GLfloat; param2 : GLfloat ); external name '_glPolygonOffsetEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_texture3D and GL_EXT_texture3D}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTexImage3DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLsizei; param7 : GLint; param8 : GLenum; param9 : GLenum; const param10 : UnivPtr );
	glTexSubImage3DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLsizei; param8 : GLsizei; param9 : GLenum; param10 : GLenum; const param11 : UnivPtr );
{$elsec}
procedure glTexImage3DEXT( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLsizei; param7 : GLint; param8 : GLenum; param9 : GLenum; const param10 : UnivPtr ); external name '_glTexImage3DEXT';
procedure glTexSubImage3DEXT( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLsizei; param8 : GLsizei; param9 : GLenum; param10 : GLenum; const param11 : UnivPtr ); external name '_glTexSubImage3DEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_subtexture and GL_EXT_subtexture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTexSubImage1DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLsizei; param5 : GLenum; param6 : GLenum; const param7 : UnivPtr );
	glTexSubImage2DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLsizei; param6 : GLsizei; param7 : GLenum; param8 : GLenum; const param9 : UnivPtr );
{$elsec}
procedure glTexSubImage1DEXT( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLsizei; param5 : GLenum; param6 : GLenum; const param7 : UnivPtr ); external name '_glTexSubImage1DEXT';
procedure glTexSubImage2DEXT( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLsizei; param6 : GLsizei; param7 : GLenum; param8 : GLenum; const param9 : UnivPtr ); external name '_glTexSubImage2DEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_copy_texture and GL_EXT_copy_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glCopyTexImage1DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLint );
	glCopyTexImage2DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLsizei; param8 : GLint );
	glCopyTexSubImage1DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLsizei );
	glCopyTexSubImage2DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLint; param7 : GLsizei; param8 : GLsizei );
	glCopyTexSubImage3DEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLint; param7 : GLint; param8 : GLsizei; param9 : GLsizei );
{$elsec}
procedure glCopyTexImage1DEXT( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLint ); external name '_glCopyTexImage1DEXT';
procedure glCopyTexImage2DEXT( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLint; param5 : GLint; param6 : GLsizei; param7 : GLsizei; param8 : GLint ); external name '_glCopyTexImage2DEXT';
procedure glCopyTexSubImage1DEXT( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLsizei ); external name '_glCopyTexSubImage1DEXT';
procedure glCopyTexSubImage2DEXT( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLint; param7 : GLsizei; param8 : GLsizei ); external name '_glCopyTexSubImage2DEXT';
procedure glCopyTexSubImage3DEXT( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLint; param7 : GLint; param8 : GLsizei; param9 : GLsizei ); external name '_glCopyTexSubImage3DEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_histogram and GL_EXT_histogram}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGetHistogramEXTProcPtr = procedure( param1 : GLenum; param2 : GLboolean; param3 : GLenum; param4 : GLenum; param5 : UnivPtr );
	glGetHistogramParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetHistogramParameterivEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glGetMinmaxEXTProcPtr = procedure( param1 : GLenum; param2 : GLboolean; param3 : GLenum; param4 : GLenum; param5 : UnivPtr );
	glGetMinmaxParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetMinmaxParameterivEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glHistogramEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; param3 : GLenum; param4 : GLboolean );
	glMinmaxEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLboolean );
	glResetHistogramEXTProcPtr = procedure( param1 : GLenum );
	glResetMinmaxEXTProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glGetHistogramEXT( param1 : GLenum; param2 : GLboolean; param3 : GLenum; param4 : GLenum; param5 : UnivPtr ); external name '_glGetHistogramEXT';
procedure glGetHistogramParameterfvEXT( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetHistogramParameterfvEXT';
procedure glGetHistogramParameterivEXT( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetHistogramParameterivEXT';
procedure glGetMinmaxEXT( param1 : GLenum; param2 : GLboolean; param3 : GLenum; param4 : GLenum; param5 : UnivPtr ); external name '_glGetMinmaxEXT';
procedure glGetMinmaxParameterfvEXT( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetMinmaxParameterfvEXT';
procedure glGetMinmaxParameterivEXT( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetMinmaxParameterivEXT';
procedure glHistogramEXT( param1 : GLenum; param2 : GLsizei; param3 : GLenum; param4 : GLboolean ); external name '_glHistogramEXT';
procedure glMinmaxEXT( param1 : GLenum; param2 : GLenum; param3 : GLboolean ); external name '_glMinmaxEXT';
procedure glResetHistogramEXT( param1 : GLenum ); external name '_glResetHistogramEXT';
procedure glResetMinmaxEXT( param1 : GLenum ); external name '_glResetMinmaxEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_convolution and GL_EXT_convolution}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glConvolutionFilter1DEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLenum; param5 : GLenum; param6 : const UnivPtr );
	glConvolutionFilter2DEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; param5 : GLenum; param6 : GLenum; param7 : const UnivPtr );
	glConvolutionParameterfEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLfloat );
	glConvolutionParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
	glConvolutionParameteriEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint );
	glConvolutionParameterivEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLint );
	glCopyConvolutionFilter1DEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint; param4 : GLint; param5 : GLsizei );
	glCopyConvolutionFilter2DEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint; param4 : GLint; param5 : GLsizei; param6 : GLsizei );
	glGetConvolutionFilterEXTProcPtr = procedure(( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : UnivPtr ) GLenum; GLenum; GLenum; UnivPtr );
	glGetConvolutionParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetConvolutionParameterivEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glGetSeparableFilterEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : UnivPtr; param5 : UnivPtr; param6 : UnivPtr );
	glSeparableFilter2DEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; param5 : GLenum; param6 : GLenum; param7 : const UnivPtr; param8 : const UnivPtr );
{$elsec}
procedure glConvolutionFilter1DEXT( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLenum; param5 : GLenum; param6 : const UnivPtr ); external name '_glConvolutionFilter1DEXT';
procedure glConvolutionFilter2DEXT( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; param5 : GLenum; param6 : GLenum; param7 : const UnivPtr ); external name '_glConvolutionFilter2DEXT';
procedure glConvolutionParameterfEXT( param1 : GLenum; param2 : GLenum; param3 : GLfloat ); external name '_glConvolutionParameterfEXT';
procedure glConvolutionParameterfvEXT( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glConvolutionParameterfvEXT';
procedure glConvolutionParameteriEXT( param1 : GLenum; param2 : GLenum; param3 : GLint ); external name '_glConvolutionParameteriEXT';
procedure glConvolutionParameterivEXT( param1 : GLenum; param2 : GLenum; const param3 : PGLint ); external name '_glConvolutionParameterivEXT';
procedure glCopyConvolutionFilter1DEXT( param1 : GLenum; param2 : GLenum; param3 : GLint; param4 : GLint; param5 : GLsizei ); external name '_glCopyConvolutionFilter1DEXT';
procedure glCopyConvolutionFilter2DEXT( param1 : GLenum; param2 : GLenum; param3 : GLint; param4 : GLint; param5 : GLsizei; param6 : GLsizei ); external name '_glCopyConvolutionFilter2DEXT';
procedure glGetConvolutionFilterEXT( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : UnivPtr ); external name '_glGetConvolutionFilterEXT';
procedure glGetConvolutionParameterfvEXT( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetConvolutionParameterfvEXT';
procedure glGetConvolutionParameterivEXT( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetConvolutionParameterivEXT';
procedure glGetSeparableFilterEXT( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : UnivPtr; param5 : UnivPtr; param6 : UnivPtr ); external name '_glGetSeparableFilterEXT';
procedure glSeparableFilter2DEXT( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; param5 : GLenum; param6 : GLenum; param7 : const UnivPtr; param8 : const UnivPtr ); external name '_glSeparableFilter2DEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_texture_object and GL_EXT_texture_object}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glAreTexturesResidentEXTProcPtr = function( param1 : GLsizei; const param2 : PGLuint; param3 : PGLboolean ): GLboolean;
	glBindTextureEXTProcPtr = procedure( param1 : GLenum; param2 : GLuint );
	glDeleteTexturesEXTProcPtr = procedure( param1 : GLsizei; const param2 : PGLuint );
	glGenTexturesEXTProcPtr = procedure( param1 : GLsizei; param2 : PGLuint );
	glIsTextureEXTProcPtr = function( param1 : GLuint ): GLboolean;
	glPrioritizeTexturesEXTProcPtr = procedure( param1 : GLsizei; const param2 : PGLuint; const param3 : PGLclampf );
{$elsec}
function glAreTexturesResidentEXT( param1 : GLsizei; const param2 : PGLuint; param3 : PGLboolean ): GLboolean; external name '_glAreTexturesResidentEXT';
procedure glBindTextureEXT( param1 : GLenum; param2 : GLuint ); external name '_glBindTextureEXT';
procedure glDeleteTexturesEXT( param1 : GLsizei; const param2 : PGLuint ); external name '_glDeleteTexturesEXT';
procedure glGenTexturesEXT( param1 : GLsizei; param2 : PGLuint ); external name '_glGenTexturesEXT';
function glIsTextureEXT( param1 : GLuint ): GLboolean; external name '_glIsTextureEXT';
procedure glPrioritizeTexturesEXT( param1 : GLsizei; const param2 : PGLuint; const param3 : PGLclampf ); external name '_glPrioritizeTexturesEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_vertex_array and GL_EXT_vertex_array}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glArrayElementEXTProcPtr = procedure( param1 : GLint );
	glColorPointerEXTProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; const param5 : UnivPtr );
	glDrawArraysEXTProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLsizei );
	glEdgeFlagPointerEXTProcPtr = procedure( param1 : GLsizei; param2 : GLsizei; const param3 : UnivPtr );
	glGetPointervEXTProcPtr = procedure( param1 : GLenum; param2 : UnivPtrPtr );
	glIndexPointerEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; const param4 : UnivPtr );
	glNormalPointerEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; const param4 : UnivPtr );
	glTexCoordPointerEXTProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; const param5 : UnivPtr );
	glVertexPointerEXTProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; const param5 : UnivPtr );
{$elsec}
procedure glArrayElementEXT( param1 : GLint ); external name '_glArrayElementEXT';
procedure glColorPointerEXT( param1 : GLint; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; const param5 : UnivPtr ); external name '_glColorPointerEXT';
procedure glDrawArraysEXT( param1 : GLenum; param2 : GLint; param3 : GLsizei ); external name '_glDrawArraysEXT';
procedure glEdgeFlagPointerEXT( param1 : GLsizei; param2 : GLsizei; const param3 : UnivPtr ); external name '_glEdgeFlagPointerEXT';
procedure glGetPointervEXT( param1 : GLenum; param2 : UnivPtrPtr ); external name '_glGetPointervEXT';
procedure glIndexPointerEXT( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; const param4 : UnivPtr ); external name '_glIndexPointerEXT';
procedure glNormalPointerEXT( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; const param4 : UnivPtr ); external name '_glNormalPointerEXT';
procedure glTexCoordPointerEXT( param1 : GLint; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; const param5 : UnivPtr ); external name '_glTexCoordPointerEXT';
procedure glVertexPointerEXT( param1 : GLint; param2 : GLenum; param3 : GLsizei; param4 : GLsizei; const param5 : UnivPtr ); external name '_glVertexPointerEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_blend_minmax and GL_EXT_blend_minmax}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBlendEquationEXTProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glBlendEquationEXT( param1 : GLenum ); external name '_glBlendEquationEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_color_subtable and GL_EXT_color_subtable}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glColorSubTableEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr );
	glCopyColorSubTableEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; param3 : GLint; param4 : GLint; param5 : GLsizei );
{$elsec}
procedure glColorSubTableEXT( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr ); external name '_glColorSubTableEXT';
procedure glCopyColorSubTableEXT( param1 : GLenum; param2 : GLsizei; param3 : GLint; param4 : GLint; param5 : GLsizei ); external name '_glCopyColorSubTableEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_paletted_texture and GL_EXT_paletted_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glColorTableEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr );
	glColorSubTableEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr );
	glGetColorTableEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; univ param4 : Ptr );
	glGetColorTableParameterivEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glGetColorTableParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
{$elsec}
procedure glColorTableEXT( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr ); external name '_glColorTableEXT';
procedure glColorSubTableEXT( param1 : GLenum; param2 : GLsizei; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr ); external name '_glColorSubTableEXT';
procedure glGetColorTableEXT( param1 : GLenum; param2 : GLenum; param3 : GLenum; univ param4 : Ptr ); external name '_glGetColorTableEXT';
procedure glGetColorTableParameterivEXT( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetColorTableParameterivEXT';
procedure glGetColorTableParameterfvEXT( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetColorTableParameterfvEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_index_material and GL_EXT_index_material}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glIndexMaterialEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum );
{$elsec}
procedure glIndexMaterialEXT( param1 : GLenum; param2 : GLenum ); external name '_glIndexMaterialEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_index_func and GL_EXT_index_func}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glIndexFuncEXTProcPtr = procedure( param1 : GLenum; param2 : GLclampf );
{$elsec}
procedure glIndexFuncEXT( param1 : GLenum; param2 : GLclampf ); external name '_glIndexFuncEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_compiled_vertex_array and GL_EXT_compiled_vertex_array}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glLockArraysEXTProcPtr = procedure( param1 : GLint; param2 : GLsizei );
	glUnlockArraysEXTProcPtr = procedure;
{$elsec}
procedure glLockArraysEXT( param1 : GLint; param2 : GLsizei ); external name '_glLockArraysEXT';
procedure glUnlockArraysEXT; external name '_glUnlockArraysEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_cull_vertex and GL_EXT_cull_vertex}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glCullParameterdvEXTProcPtr = procedure( param1 : GLenum; param2 : PGLdouble );
	glCullParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : PGLfloat );
{$elsec}
procedure glCullParameterdvEXT( param1 : GLenum; param2 : PGLdouble ); external name '_glCullParameterdvEXT';
procedure glCullParameterfvEXT( param1 : GLenum; param2 : PGLfloat ); external name '_glCullParameterfvEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_draw_range_elements and GL_EXT_draw_range_elements}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glDrawRangeElementsEXTProcPtr = procedure( param1 : GLenum; param2 : GLuint; param3 : GLuint; param4 : GLsizei; param5 : GLenum; const param6 : UnivPtr );
{$elsec}
procedure glDrawRangeElementsEXT( param1 : GLenum; param2 : GLuint; param3 : GLuint; param4 : GLsizei; param5 : GLenum; const param6 : UnivPtr ); external name '_glDrawRangeElementsEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_light_texture and GL_EXT_light_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glApplyTextureEXTProcPtr = procedure( param1 : GLenum );
	glTextureLightEXTProcPtr = procedure( param1 : GLenum );
	glTextureMaterialEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum );
{$elsec}
procedure glApplyTextureEXT( param1 : GLenum ); external name '_glApplyTextureEXT';
procedure glTextureLightEXT( param1 : GLenum ); external name '_glTextureLightEXT';
procedure glTextureMaterialEXT( param1 : GLenum; param2 : GLenum ); external name '_glTextureMaterialEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_pixel_transform and GL_EXT_pixel_transform}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPixelTransformParameteriEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint );
	glPixelTransformParameterfEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLfloat );
	glPixelTransformParameterivEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLint );
	glPixelTransformParameterfvEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
{$elsec}
procedure glPixelTransformParameteriEXT( param1 : GLenum; param2 : GLenum; param3 : GLint ); external name '_glPixelTransformParameteriEXT';
procedure glPixelTransformParameterfEXT( param1 : GLenum; param2 : GLenum; param3 : GLfloat ); external name '_glPixelTransformParameterfEXT';
procedure glPixelTransformParameterivEXT( param1 : GLenum; param2 : GLenum; const param3 : PGLint ); external name '_glPixelTransformParameterivEXT';
procedure glPixelTransformParameterfvEXT( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glPixelTransformParameterfvEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_secondary_color and GL_EXT_secondary_color}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glSecondaryColor3bEXTProcPtr = procedure( param1 : GLbyte; param2 : GLbyte; param3 : GLbyte );
	glSecondaryColor3bvEXTProcPtr = procedure( const param1 : PGLbyte );
	glSecondaryColor3dEXTProcPtr = procedure( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble );
	glSecondaryColor3dvEXTProcPtr = procedure( const param1 : PGLdouble );
	glSecondaryColor3fEXTProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat );
	glSecondaryColor3fvEXTProcPtr = procedure( const param1 : PGLfloat );
	glSecondaryColor3iEXTProcPtr = procedure( param1 : GLint; param2 : GLint; param3 : GLint );
	glSecondaryColor3ivEXTProcPtr = procedure( const param1 : PGLint );
	glSecondaryColor3sEXTProcPtr = procedure( param1 : GLshort; param2 : GLshort; param3 : GLshort );
	glSecondaryColor3svEXTProcPtr = procedure( const param1 : PGLshort );
	glSecondaryColor3ubEXTProcPtr = procedure( param1 : GLubyte; param2 : GLubyte; param3 : GLubyte );
	glSecondaryColor3ubvEXTProcPtr = procedure( const param1 : PGLubyte );
	glSecondaryColor3uiEXTProcPtr = procedure( param1 : GLuint; param2 : GLuint; param3 : GLuint );
	glSecondaryColor3uivEXTProcPtr = procedure( const param1 : PGLuint );
	glSecondaryColor3usEXTProcPtr = procedure( param1 : GLushort; param2 : GLushort; param3 : GLushort );
	glSecondaryColor3usvEXTProcPtr = procedure( const param1 : PGLushort );
	glSecondaryColorPointerEXTProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLsizei; const param4 : UnivPtr );
{$elsec}
procedure glSecondaryColor3bEXT( param1 : GLbyte; param2 : GLbyte; param3 : GLbyte ); external name '_glSecondaryColor3bEXT';
procedure glSecondaryColor3bvEXT( const param1 : PGLbyte ); external name '_glSecondaryColor3bvEXT';
procedure glSecondaryColor3dEXT( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble ); external name '_glSecondaryColor3dEXT';
procedure glSecondaryColor3dvEXT( const param1 : PGLdouble ); external name '_glSecondaryColor3dvEXT';
procedure glSecondaryColor3fEXT( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat ); external name '_glSecondaryColor3fEXT';
procedure glSecondaryColor3fvEXT( const param1 : PGLfloat ); external name '_glSecondaryColor3fvEXT';
procedure glSecondaryColor3iEXT( param1 : GLint; param2 : GLint; param3 : GLint ); external name '_glSecondaryColor3iEXT';
procedure glSecondaryColor3ivEXT( const param1 : PGLint ); external name '_glSecondaryColor3ivEXT';
procedure glSecondaryColor3sEXT( param1 : GLshort; param2 : GLshort; param3 : GLshort ); external name '_glSecondaryColor3sEXT';
procedure glSecondaryColor3svEXT( const param1 : PGLshort ); external name '_glSecondaryColor3svEXT';
procedure glSecondaryColor3ubEXT( param1 : GLubyte; param2 : GLubyte; param3 : GLubyte ); external name '_glSecondaryColor3ubEXT';
procedure glSecondaryColor3ubvEXT( const param1 : PGLubyte ); external name '_glSecondaryColor3ubvEXT';
procedure glSecondaryColor3uiEXT( param1 : GLuint; param2 : GLuint; param3 : GLuint ); external name '_glSecondaryColor3uiEXT';
procedure glSecondaryColor3uivEXT( const param1 : PGLuint ); external name '_glSecondaryColor3uivEXT';
procedure glSecondaryColor3usEXT( param1 : GLushort; param2 : GLushort; param3 : GLushort ); external name '_glSecondaryColor3usEXT';
procedure glSecondaryColor3usvEXT( const param1 : PGLushort ); external name '_glSecondaryColor3usvEXT';
procedure glSecondaryColorPointerEXT( param1 : GLint; param2 : GLenum; param3 : GLsizei; const param4 : UnivPtr ); external name '_glSecondaryColorPointerEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_texture_perturb_normal and GL_EXT_texture_perturb_normal}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTextureNormalEXTProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glTextureNormalEXT( param1 : GLenum ); external name '_glTextureNormalEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_multi_draw_arrays and GL_EXT_multi_draw_arrays}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glMultiDrawArraysEXTProcPtr = procedure( param1 : GLenum; const param2 : PGLint; const param3 : PGLsizei; param4 : GLsizei );
	glMultiDrawElementsEXTProcPtr = procedure( param1 : GLenum; const param2 : PGLsizei; param3 : GLenum; const param4 : UnivPtrPtr; param5 : GLsizei );
{$elsec}
procedure glMultiDrawArraysEXT( param1 : GLenum; const param2 : PGLint; const param3 : PGLsizei; param4 : GLsizei ); external name '_glMultiDrawArraysEXT';
procedure glMultiDrawElementsEXT( param1 : GLenum; const param2 : PGLsizei; param3 : GLenum; const param4 : UnivPtrPtr; param5 : GLsizei ); external name '_glMultiDrawElementsEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_fog_coord and GL_EXT_fog_coord}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFogCoordfEXTProcPtr = procedure( param1 : GLfloat );
	glFogCoordfvEXTProcPtr = procedure( const param1 : PGLfloat );
	glFogCoorddEXTProcPtr = procedure( param1 : GLdouble );
	glFogCoorddvEXTProcPtr = procedure( const param1 : PGLdouble );
	glFogCoordPointerEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtr );
{$elsec}
procedure glFogCoordfEXT( param1 : GLfloat ); external name '_glFogCoordfEXT';
procedure glFogCoordfvEXT( const param1 : PGLfloat ); external name '_glFogCoordfvEXT';
procedure glFogCoorddEXT( param1 : GLdouble ); external name '_glFogCoorddEXT';
procedure glFogCoorddvEXT( const param1 : PGLdouble ); external name '_glFogCoorddvEXT';
procedure glFogCoordPointerEXT( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtr ); external name '_glFogCoordPointerEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_coordinate_frame and GL_EXT_coordinate_frame}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTangent3bEXTProcPtr = procedure( param1 : GLbyte; param2 : GLbyte; param3 : GLbyte );
	glTangent3bvEXTProcPtr = procedure( const param1 : PGLbyte );
	glTangent3dEXTProcPtr = procedure( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble );
	glTangent3dvEXTProcPtr = procedure( const param1 : PGLdouble );
	glTangent3fEXTProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat );
	glTangent3fvEXTProcPtr = procedure( const param1 : PGLfloat );
	glTangent3iEXTProcPtr = procedure( param1 : GLint; param2 : GLint; param3 : GLint );
	glTangent3ivEXTProcPtr = procedure( const param1 : PGLint );
	glTangent3sEXTProcPtr = procedure( param1 : GLshort; param2 : GLshort; param3 : GLshort );
	glTangent3svEXTProcPtr = procedure( const param1 : PGLshort );
	glBinormal3bEXTProcPtr = procedure( param1 : GLbyte; param2 : GLbyte; param3 : GLbyte );
	glBinormal3bvEXTProcPtr = procedure( const param1 : PGLbyte );
	glBinormal3dEXTProcPtr = procedure( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble );
	glBinormal3dvEXTProcPtr = procedure( const param1 : PGLdouble );
	glBinormal3fEXTProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat );
	glBinormal3fvEXTProcPtr = procedure( const param1 : PGLfloat );
	glBinormal3iEXTProcPtr = procedure( param1 : GLint; param2 : GLint; param3 : GLint );
	glBinormal3ivEXTProcPtr = procedure( const param1 : PGLint );
	glBinormal3sEXTProcPtr = procedure( param1 : GLshort; param2 : GLshort; param3 : GLshort );
	glBinormal3svEXTProcPtr = procedure( const param1 : PGLshort );
	glTangentPointerEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtr );
	glBinormalPointerEXTProcPtr = procedure( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtr );
{$elsec}
procedure glTangent3bEXT( param1 : GLbyte; param2 : GLbyte; param3 : GLbyte ); external name '_glTangent3bEXT';
procedure glTangent3bvEXT( const param1 : PGLbyte ); external name '_glTangent3bvEXT';
procedure glTangent3dEXT( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble ); external name '_glTangent3dEXT';
procedure glTangent3dvEXT( const param1 : PGLdouble ); external name '_glTangent3dvEXT';
procedure glTangent3fEXT( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat ); external name '_glTangent3fEXT';
procedure glTangent3fvEXT( const param1 : PGLfloat ); external name '_glTangent3fvEXT';
procedure glTangent3iEXT( param1 : GLint; param2 : GLint; param3 : GLint ); external name '_glTangent3iEXT';
procedure glTangent3ivEXT( const param1 : PGLint ); external name '_glTangent3ivEXT';
procedure glTangent3sEXT( param1 : GLshort; param2 : GLshort; param3 : GLshort ); external name '_glTangent3sEXT';
procedure glTangent3svEXT( const param1 : PGLshort ); external name '_glTangent3svEXT';
procedure glBinormal3bEXT( param1 : GLbyte; param2 : GLbyte; param3 : GLbyte ); external name '_glBinormal3bEXT';
procedure glBinormal3bvEXT( const param1 : PGLbyte ); external name '_glBinormal3bvEXT';
procedure glBinormal3dEXT( param1 : GLdouble; param2 : GLdouble; param3 : GLdouble ); external name '_glBinormal3dEXT';
procedure glBinormal3dvEXT( const param1 : PGLdouble ); external name '_glBinormal3dvEXT';
procedure glBinormal3fEXT( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat ); external name '_glBinormal3fEXT';
procedure glBinormal3fvEXT( const param1 : PGLfloat ); external name '_glBinormal3fvEXT';
procedure glBinormal3iEXT( param1 : GLint; param2 : GLint; param3 : GLint ); external name '_glBinormal3iEXT';
procedure glBinormal3ivEXT( const param1 : PGLint ); external name '_glBinormal3ivEXT';
procedure glBinormal3sEXT( param1 : GLshort; param2 : GLshort; param3 : GLshort ); external name '_glBinormal3sEXT';
procedure glBinormal3svEXT( const param1 : PGLshort ); external name '_glBinormal3svEXT';
procedure glTangentPointerEXT( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtr ); external name '_glTangentPointerEXT';
procedure glBinormalPointerEXT( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtr ); external name '_glBinormalPointerEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_blend_func_separate and GL_EXT_blend_func_separate}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBlendFuncSeparateEXTProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum );
{$elsec}
procedure glBlendFuncSeparateEXT( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum ); external name '_glBlendFuncSeparateEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_vertex_weighting and GL_EXT_vertex_weighting}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glVertexWeightfEXTProcPtr = procedure( param1 : GLfloat );
	glVertexWeightfvEXTProcPtr = procedure( const param1 : PGLfloat );
	glVertexWeightPointerEXTProcPtr = procedure( param1 : GLsizei; param2 : GLenum; param3 : GLsizei; const param4 : UnivPtr );
{$elsec}
procedure glVertexWeightfEXT( param1 : GLfloat ); external name '_glVertexWeightfEXT';
procedure glVertexWeightfvEXT( const param1 : PGLfloat ); external name '_glVertexWeightfvEXT';
procedure glVertexWeightPointerEXT( param1 : GLsizei; param2 : GLenum; param3 : GLsizei; const param4 : UnivPtr ); external name '_glVertexWeightPointerEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_vertex_shader and GL_EXT_vertex_shader}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBeginVertexShaderEXTProcPtr = procedure;
	glEndVertexShaderEXTProcPtr = procedure;
	glBindVertexShaderEXTProcPtr = procedure( id_: GLuint );
	glGenVertexShadersEXTProcPtr = function( range: GLuint ): GLuint;
	glDeleteVertexShaderEXTProcPtr = procedure( id_: GLuint );
	glShaderOp1EXTProcPtr = procedure( op: GLenum; res: GLuint; arg1: GLuint );
	glShaderOp2EXTProcPtr = procedure( op: GLenum; res: GLuint; arg1: GLuint; arg2: GLuint );
	glShaderOp3EXTProcPtr = procedure( op: GLenum; res: GLuint; arg1: GLuint; arg2: GLuint; arg3: GLuint );
	glSwizzleEXTProcPtr = procedure( res: GLuint; inp: GLuint; outX: GLenum; outY: GLenum; outZ: GLenum; outW: GLenum );
	glWriteMaskEXTProcPtr = procedure( res: GLuint; inp: GLuint; outX: GLenum; outY: GLenum; outZ: GLenum; outW: GLenum );
	glInsertComponentEXTProcPtr = procedure( res: GLuint; src: GLuint; num: GLuint );
	glExtractComponentEXTProcPtr = procedure( res: GLuint; src: GLuint; num: GLuint );
	glGenSymbolsEXTProcPtr = function( datatype: GLenum; storagetype: GLenum; range: GLenum; components: GLuint ): GLuint;
	glSetInvariantEXTProcPtr = procedure( id_: GLuint; typ: GLenum; addr: UnivPtr );
	glSetLocalConstantEXTProcPtr = procedure( id_: GLuint; typ: GLenum; addr: UnivPtr );
	glVariantbvEXTProcPtr = procedure( id_: GLuint; addr: PGLbyte );
	glVariantdvEXTProcPtr = procedure( id_: GLuint; addr: PGLdouble );
	glVariantfvEXTProcPtr = procedure( id_: GLuint; addr: PGLfloat );
	glVariantivEXTProcPtr = procedure( id_: GLuint; addr: PGLint );
	glVariantsvEXTProcPtr = procedure( id_: GLuint; addr: PGLshort );
	glVariantubvEXTProcPtr = procedure( id_: GLuint; addr: PGLubyte );
	glVariantuivEXTProcPtr = procedure( id_: GLuint; addr: PGLuint );
	glVariantusvEXTProcPtr = procedure( id_: GLuint; addr: PGLushort );
	glVariantPointerEXTProcPtr = procedure( id_: GLuint; typ: GLenum; stride: GLuint; addr: UnivPtr );
	glEnableVariantClientStateEXTProcPtr = procedure( id_: GLuint );
	glDisableVariantClientStateEXTProcPtr = procedure( id_: GLuint );
	glBindLightParameterEXTProcPtr = function( light: GLenum; value: GLenum ): GLuint;
	glBindMaterialParameterEXTProcPtr = function( face: GLenum; value: GLenum ): GLuint;
	glBindTexGenParameterEXTProcPtr = function( unt: GLenum; coord: GLenum; value: GLenum ): GLuint;
	glBindTextureUnitParameterEXTProcPtr = function( unt: GLenum; value: GLenum ): GLuint;
	glBindParameterEXTProcPtr = function( value: GLenum ): GLuint;
	glIsVariantEnabledEXTProcPtr = function( id_: GLuint; cap: GLenum ): GLboolean;
	glGetVariantBooleanvEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLboolean );
	glGetVariantIntegervEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLint );
	glGetVariantFloatvEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLfloat );
	glGetVariantPointervEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: UnivPtrPtr );
	glGetInvariantBooleanvEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLboolean );
	glGetInvariantIntegervEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLint );
	glGetInvariantFloatvEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLfloat );
	glGetLocalConstantBooleanvEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLboolean );
	glGetLocalConstantIntegervEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLint );
	glGetLocalConstantFloatvEXTProcPtr = procedure( id_: GLuint; value: GLenum; data: PGLfloat );
{$elsec}
procedure glBeginVertexShaderEXT; external name '_glBeginVertexShaderEXT';
procedure glEndVertexShaderEXT; external name '_glEndVertexShaderEXT';
procedure glBindVertexShaderEXT( id_: GLuint ); external name '_glBindVertexShaderEXT';
function glGenVertexShadersEXT( range: GLuint ): GLuint; external name '_glGenVertexShadersEXT';
procedure glDeleteVertexShaderEXT( id_: GLuint ); external name '_glDeleteVertexShaderEXT';
procedure glShaderOp1EXT( op: GLenum; res: GLuint; arg1: GLuint ); external name '_glShaderOp1EXT';
procedure glShaderOp2EXT( op: GLenum; res: GLuint; arg1: GLuint; arg2: GLuint ); external name '_glShaderOp2EXT';
procedure glShaderOp3EXT( op: GLenum; res: GLuint; arg1: GLuint; arg2: GLuint; arg3: GLuint ); external name '_glShaderOp3EXT';
procedure glSwizzleEXT( res: GLuint; inp: GLuint; outX: GLenum; outY: GLenum; outZ: GLenum; outW: GLenum ); external name '_glSwizzleEXT';
procedure glWriteMaskEXT( res: GLuint; inp: GLuint; outX: GLenum; outY: GLenum; outZ: GLenum; outW: GLenum ); external name '_glWriteMaskEXT';
procedure glInsertComponentEXT( res: GLuint; src: GLuint; num: GLuint ); external name '_glInsertComponentEXT';
procedure glExtractComponentEXT( res: GLuint; src: GLuint; num: GLuint ); external name '_glExtractComponentEXT';
function glGenSymbolsEXT( datatype: GLenum; storagetype: GLenum; range: GLenum; components: GLuint ): GLuint; external name '_glGenSymbolsEXT';
procedure glSetInvariantEXT( id_: GLuint; typ: GLenum; addr: UnivPtr ); external name '_glSetInvariantEXT';
procedure glSetLocalConstantEXT( id_: GLuint; typ: GLenum; addr: UnivPtr ); external name '_glSetLocalConstantEXT';
procedure glVariantbvEXT( id_: GLuint; addr: PGLbyte ); external name '_glVariantbvEXT';
procedure glVariantdvEXT( id_: GLuint; addr: PGLdouble ); external name '_glVariantdvEXT';
procedure glVariantfvEXT( id_: GLuint; addr: PGLfloat ); external name '_glVariantfvEXT';
procedure glVariantivEXT( id_: GLuint; addr: PGLint ); external name '_glVariantivEXT';
procedure glVariantsvEXT( id_: GLuint; addr: PGLshort ); external name '_glVariantsvEXT';
procedure glVariantubvEXT( id_: GLuint; addr: PGLubyte ); external name '_glVariantubvEXT';
procedure glVariantuivEXT( id_: GLuint; addr: PGLuint ); external name '_glVariantuivEXT';
procedure glVariantusvEXT( id_: GLuint; addr: PGLushort ); external name '_glVariantusvEXT';
procedure glVariantPointerEXT( id_: GLuint; typ: GLenum; stride: GLuint; addr: UnivPtr ); external name '_glVariantPointerEXT';
procedure glEnableVariantClientStateEXT( id_: GLuint ); external name '_glEnableVariantClientStateEXT';
procedure glDisableVariantClientStateEXT( id_: GLuint ); external name '_glDisableVariantClientStateEXT';
function glBindLightParameterEXT( light: GLenum; value: GLenum ): GLuint; external name '_glBindLightParameterEXT';
function glBindMaterialParameterEXT( face: GLenum; value: GLenum ): GLuint; external name '_glBindMaterialParameterEXT';
function glBindTexGenParameterEXT( unt: GLenum; coord: GLenum; value: GLenum ): GLuint; external name '_glBindTexGenParameterEXT';
function glBindTextureUnitParameterEXT( unt: GLenum; value: GLenum ): GLuint; external name '_glBindTextureUnitParameterEXT';
function glBindParameterEXT( value: GLenum ): GLuint; external name '_glBindParameterEXT';
function glIsVariantEnabledEXT( id_: GLuint; cap: GLenum ): GLboolean; external name '_glIsVariantEnabledEXT';
procedure glGetVariantBooleanvEXT( id_: GLuint; value: GLenum; data: PGLboolean ); external name '_glGetVariantBooleanvEXT';
procedure glGetVariantIntegervEXT( id_: GLuint; value: GLenum; data: PGLint ); external name '_glGetVariantIntegervEXT';
procedure glGetVariantFloatvEXT( id_: GLuint; value: GLenum; data: PGLfloat ); external name '_glGetVariantFloatvEXT';
procedure glGetVariantPointervEXT( id_: GLuint; value: GLenum; data: UnivPtrPtr ); external name '_glGetVariantPointervEXT';
procedure glGetInvariantBooleanvEXT( id_: GLuint; value: GLenum; data: PGLboolean ); external name '_glGetInvariantBooleanvEXT';
procedure glGetInvariantIntegervEXT( id_: GLuint; value: GLenum; data: PGLint ); external name '_glGetInvariantIntegervEXT';
procedure glGetInvariantFloatvEXT( id_: GLuint; value: GLenum; data: PGLfloat ); external name '_glGetInvariantFloatvEXT';
procedure glGetLocalConstantBooleanvEXT( id_: GLuint; value: GLenum; data: PGLboolean ); external name '_glGetLocalConstantBooleanvEXT';
procedure glGetLocalConstantIntegervEXT( id_: GLuint; value: GLenum; data: PGLint ); external name '_glGetLocalConstantIntegervEXT';
procedure glGetLocalConstantFloatvEXT( id_: GLuint; value: GLenum; data: PGLfloat ); external name '_glGetLocalConstantFloatvEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_fragment_shader and GL_EXT_fragment_shader}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGenFragmentShadersEXTProcPtr = function( range: GLuint ): GLuint;
	glBindFragmentShaderEXTProcPtr = procedure( id_: GLuint );
	glDeleteFragmentShaderEXTProcPtr = procedure( id_: GLuint );
	glBeginFragmentShaderEXTProcPtr = procedure;
	glEndFragmentShaderEXTProcPtr = procedure;
	glPassTexCoordEXTProcPtr = procedure( dst: GLuint; coord: GLuint; swizzle: GLenum );
	glSampleMapEXTProcPtr = procedure( dst: GLuint; interp: GLuint; swizzle: GLenum );
	glColorFragmentOp1EXTProcPtr = procedure( op: GLenum; dst: GLuint; dstMask: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint );
	glColorFragmentOp2EXTProcPtr = procedure( op: GLenum; dst: GLuint; dstMask: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint );
	glColorFragmentOp3EXTProcPtr = procedure( op: GLenum; dst: GLuint; dstMask: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint; arg3: GLuint; arg3Rep: GLuint; arg3Mod: GLuint );
	glAlphaFragmentOp1EXTProcPtr = procedure( op: GLenum; dst: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint );
	glAlphaFragmentOp2EXTProcPtr = procedure( op: GLenum; dst: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint );
	glAlphaFragmentOp3EXTProcPtr = procedure( op: GLenum; dst: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint; arg3: GLuint; arg3Rep: GLuint; arg3Mod: GLuint );
	glSetFragmentShaderConstantEXTProcPtr = procedure( dst: GLuint; const value: PGLfloat );
{$elsec}
function glGenFragmentShadersEXT( range: GLuint ): GLuint; external name '_glGenFragmentShadersEXT';
procedure glBindFragmentShaderEXT( id_: GLuint ); external name '_glBindFragmentShaderEXT';
procedure glDeleteFragmentShaderEXT( id_: GLuint ); external name '_glDeleteFragmentShaderEXT';
procedure glBeginFragmentShaderEXT; external name '_glBeginFragmentShaderEXT';
procedure glEndFragmentShaderEXT; external name '_glEndFragmentShaderEXT';
procedure glPassTexCoordEXT( dst: GLuint; coord: GLuint; swizzle: GLenum ); external name '_glPassTexCoordEXT';
procedure glSampleMapEXT( dst: GLuint; interp: GLuint; swizzle: GLenum ); external name '_glSampleMapEXT';
procedure glColorFragmentOp1EXT( op: GLenum; dst: GLuint; dstMask: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint ); external name '_glColorFragmentOp1EXT';
procedure glColorFragmentOp2EXT( op: GLenum; dst: GLuint; dstMask: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint ); external name '_glColorFragmentOp2EXT';
procedure glColorFragmentOp3EXT( op: GLenum; dst: GLuint; dstMask: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint; arg3: GLuint; arg3Rep: GLuint; arg3Mod: GLuint ); external name '_glColorFragmentOp3EXT';
procedure glAlphaFragmentOp1EXT( op: GLenum; dst: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint ); external name '_glAlphaFragmentOp1EXT';
procedure glAlphaFragmentOp2EXT( op: GLenum; dst: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint ); external name '_glAlphaFragmentOp2EXT';
procedure glAlphaFragmentOp3EXT( op: GLenum; dst: GLuint; dstMod: GLuint; arg1: GLuint; arg1Rep: GLuint; arg1Mod: GLuint; arg2: GLuint; arg2Rep: GLuint; arg2Mod: GLuint; arg3: GLuint; arg3Rep: GLuint; arg3Mod: GLuint ); external name '_glAlphaFragmentOp3EXT';
procedure glSetFragmentShaderConstantEXT( dst: GLuint; const value: PGLfloat ); external name '_glSetFragmentShaderConstantEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_multisample and GL_EXT_multisample}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glSampleMaskEXTProcPtr = procedure( param1 : GLclampf; param2 : GLboolean );
	glSamplePatternEXTProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glSampleMaskEXT( param1 : GLclampf; param2 : GLboolean ); external name '_glSampleMaskEXT';
procedure glSamplePatternEXT( param1 : GLenum ); external name '_glSamplePatternEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_stencil_two_side and GL_EXT_stencil_two_side}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glActiveStencilFaceEXTProcPtr = procedure( face: GLenum );
{$elsec}
procedure glActiveStencilFaceEXT( face: GLenum ); external name '_glActiveStencilFaceEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_depth_bounds_test and GL_EXT_depth_bounds_test}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glDepthBoundsEXTProcPtr = procedure( zmin: GLclampd; zmax: GLclampd );
{$elsec}
procedure glDepthBoundsEXT( zmin: GLclampd; zmax: GLclampd ); external name '_glDepthBoundsEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_blend_equation_separate and GL_EXT_blend_equation_separate}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBlendEquationSeparateEXTProcPtr = procedure( modeRGB: GLenum; modeAlpha: GLenum );
{$elsec}
procedure glBlendEquationSeparateEXT( modeRGB: GLenum; modeAlpha: GLenum ); external name '_glBlendEquationSeparateEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_framebuffer_object and GL_EXT_framebuffer_object}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glIsRenderbufferEXTProcPtr = function( renderbuffer: GLuint ): GLboolean;
	glBindRenderbufferEXTProcPtr = procedure( target: GLenum; renderbuffer: GLuint );
	glDeleteRenderbuffersEXTProcPtr = procedure( n: GLsizei; const renderbuffers: PGLuint );
	glGenRenderbuffersEXTProcPtr = procedure( n: GLsizei; renderbuffers: PGLuint );
	glRenderbufferStorageEXTProcPtr = procedure( target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei );
	glGetRenderbufferParameterivEXTProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glIsFramebufferEXTProcPtr = function( framebuffer: GLuint ): GLboolean;
	glBindFramebufferEXTProcPtr = procedure( target: GLenum; framebuffer: GLuint );
	glDeleteFramebuffersEXTProcPtr = procedure( n: GLsizei; const framebuffers: PGLuint );
	glGenFramebuffersEXTProcPtr = procedure( n: GLsizei; framebuffers: PGLuint );
	glCheckFramebufferStatusEXTProcPtr = function( target: GLenum ): GLenum;
	glFramebufferTexture1DEXTProcPtr = procedure( target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint );
	glFramebufferTexture2DEXTProcPtr = procedure( target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint );
	glFramebufferTexture3DEXTProcPtr = procedure( target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint; zoffset: GLint );
	glFramebufferRenderbufferEXTProcPtr = procedure( target: GLenum; attachment: GLenum; renderbuffertarget: GLenum; renderbuffer: GLuint );
	glGetFramebufferAttachmentParameterivEXTProcPtr = procedure( target: GLenum; attachment: GLenum; pname: GLenum; params: PGLint );
	glGenerateMipmapEXTProcPtr = procedure( target: GLenum );
{$elsec}
function glIsRenderbufferEXT( renderbuffer: GLuint ): GLboolean; external name '_glIsRenderbufferEXT';
procedure glBindRenderbufferEXT( target: GLenum; renderbuffer: GLuint ); external name '_glBindRenderbufferEXT';
procedure glDeleteRenderbuffersEXT( n: GLsizei; const renderbuffers: PGLuint ); external name '_glDeleteRenderbuffersEXT';
procedure glGenRenderbuffersEXT( n: GLsizei; renderbuffers: PGLuint ); external name '_glGenRenderbuffersEXT';
procedure glRenderbufferStorageEXT( target: GLenum; internalformat: GLenum; width: GLsizei; height: GLsizei ); external name '_glRenderbufferStorageEXT';
procedure glGetRenderbufferParameterivEXT( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetRenderbufferParameterivEXT';
function glIsFramebufferEXT( framebuffer: GLuint ): GLboolean; external name '_glIsFramebufferEXT';
procedure glBindFramebufferEXT( target: GLenum; framebuffer: GLuint ); external name '_glBindFramebufferEXT';
procedure glDeleteFramebuffersEXT( n: GLsizei; const framebuffers: PGLuint ); external name '_glDeleteFramebuffersEXT';
procedure glGenFramebuffersEXT( n: GLsizei; framebuffers: PGLuint ); external name '_glGenFramebuffersEXT';
function glCheckFramebufferStatusEXT( target: GLenum ): GLenum; external name '_glCheckFramebufferStatusEXT';
procedure glFramebufferTexture1DEXT( target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint ); external name '_glFramebufferTexture1DEXT';
procedure glFramebufferTexture2DEXT( target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint ); external name '_glFramebufferTexture2DEXT';
procedure glFramebufferTexture3DEXT( target: GLenum; attachment: GLenum; textarget: GLenum; texture: GLuint; level: GLint; zoffset: GLint ); external name '_glFramebufferTexture3DEXT';
procedure glFramebufferRenderbufferEXT( target: GLenum; attachment: GLenum; renderbuffertarget: GLenum; renderbuffer: GLuint ); external name '_glFramebufferRenderbufferEXT';
procedure glGetFramebufferAttachmentParameterivEXT( target: GLenum; attachment: GLenum; pname: GLenum; params: PGLint ); external name '_glGetFramebufferAttachmentParameterivEXT';
procedure glGenerateMipmapEXT( target: GLenum ); external name '_glGenerateMipmapEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_framebuffer_blit and GL_EXT_framebuffer_blit}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBlitFramebufferEXTProcPtr = procedure( srcX0: GLint; srcY0: GLint; srcX1: GLint; srcY1: GLint; dstX0: GLint; dstY0: GLint; dstX1: GLint; dstY1: GLint; mask: GLbitfield; filter: GLenum );
{$elsec}
procedure glBlitFramebufferEXT( srcX0: GLint; srcY0: GLint; srcX1: GLint; srcY1: GLint; dstX0: GLint; dstY0: GLint; dstX1: GLint; dstY1: GLint; mask: GLbitfield; filter: GLenum ); external name '_glBlitFramebufferEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_framebuffer_multisample and GL_EXT_framebuffer_multisample}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glRenderbufferStorageMultisampleEXTProcPtr = procedure( target: GLenum; samples: GLsizei; internalformat: GLenum; width: GLsizei; height: GLsizei );
{$elsec}
procedure glRenderbufferStorageMultisampleEXT( target: GLenum; samples: GLsizei; internalformat: GLenum; width: GLsizei; height: GLsizei ); external name '_glRenderbufferStorageMultisampleEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_geometry_shader4 and GL_EXT_geometry_shader4}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glProgramParameteriEXTProcPtr = procedure( program_: GLuint; pname: GLenum; value: GLint );
	glFramebufferTextureEXTProcPtr = procedure( target: GLenum; attachment: GLenum; texture: GLuint; level: GLint );
	glFramebufferTextureLayerEXTProcPtr = procedure( target: GLenum; attachment: GLenum; texture: GLuint; level: GLint; layer: GLint );
	glFramebufferTextureFaceEXTProcPtr = procedure( target: GLenum; attachment: GLenum; texture: GLuint; level: GLint; face: GLenum );
{$elsec}
procedure glProgramParameteriEXT( program_: GLuint; pname: GLenum; value: GLint ); external name '_glProgramParameteriEXT';
procedure glFramebufferTextureEXT( target: GLenum; attachment: GLenum; texture: GLuint; level: GLint ); external name '_glFramebufferTextureEXT';
procedure glFramebufferTextureLayerEXT( target: GLenum; attachment: GLenum; texture: GLuint; level: GLint; layer: GLint ); external name '_glFramebufferTextureLayerEXT';
procedure glFramebufferTextureFaceEXT( target: GLenum; attachment: GLenum; texture: GLuint; level: GLint; face: GLenum ); external name '_glFramebufferTextureFaceEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_EXT_transform_feedback and GL_EXT_transform_feedback}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBindBufferRangeEXTProcPtr = procedure( target: GLenum; index: GLuint; buffer: GLuint; offset: GLintptr; size: GLsizeiptr );
	glBindBufferOffsetEXTProcPtr = procedure( target: GLenum; index: GLuint; buffer: GLuint; offset: GLintptr );
	glBindBufferBaseEXTProcPtr = procedure( target: GLenum; index: GLuint; buffer: GLuint );
	glBeginTransformFeedbackEXTProcPtr = procedure( primitiveMode: GLenum );
	glEndTransformFeedbackEXTProcPtr = procedure;
	glTransformFeedbackVaryingsEXTProcPtr = procedure( program_: GLuint; count: GLsizei; {const} varyings: PPChar; bufferMode: GLenum );

	glGetTransformFeedbackVaryingEXTProcPtr = procedure( program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLsizei; typ: PGLenum; name: PChar );
{$elsec}
procedure glBindBufferRangeEXT( target: GLenum; index: GLuint; buffer: GLuint; offset: GLintptr; size: GLsizeiptr ); external name '_glBindBufferRangeEXT';
procedure glBindBufferOffsetEXT( target: GLenum; index: GLuint; buffer: GLuint; offset: GLintptr ); external name '_glBindBufferOffsetEXT';
procedure glBindBufferBaseEXT( target: GLenum; index: GLuint; buffer: GLuint ); external name '_glBindBufferBaseEXT';
procedure glBeginTransformFeedbackEXT( primitiveMode: GLenum ); external name '_glBeginTransformFeedbackEXT';
procedure glEndTransformFeedbackEXT; external name '_glEndTransformFeedbackEXT';

procedure glTransformFeedbackVaryingsEXT( program_: GLuint; count: GLsizei; {const} varyings: PPChar; bufferMode: GLenum ); external name '_glTransformFeedbackVaryingsEXT';

procedure glGetTransformFeedbackVaryingEXT( program_: GLuint; index: GLuint; bufSize: GLsizei; length: PGLsizei; size: PGLsizei; typ: PGLenum; name: PChar ); external name '_glGetTransformFeedbackVaryingEXT';
{$endc}
{$endc} { GL_EXT_transform_feedback }

{$ifc not undefined GL_EXT_transform_feedback and GL_EXT_transform_feedback or defined GL_EXT_draw_buffers2 and GL_EXT_draw_buffers2}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGetIntegerIndexedvEXTProcPtr = procedure( param: GLenum; index: GLint; values: PGLint );
	glGetBooleanIndexedvEXTProcPtr = procedure( param: GLenum; index: GLint; values: PGLboolean );
{$elsec}
procedure glGetIntegerIndexedvEXT( param: GLenum; index: GLuint; values: PGLint ); external name '_glGetIntegerIndexedvEXT';
procedure glGetBooleanIndexedvEXT( param: GLenum; index: GLuint; values: PGLboolean ); external name '_glGetBooleanIndexedvEXT';
{$endc}
{$endc} { GL_EXT_transform_feedback || GL_EXT_draw_buffers2 }

{$ifc not undefined GL_EXT_bindable_uniform and GL_EXT_bindable_uniform}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glUniformBufferEXTProcPtr = procedure( program_: GLuint; location: GLint; buffer: GLuint );
	glGetUniformBufferSizeEXTProcPtr = function( program_: GLuint; location: GLint ): GLint;
	glGetUniformOffsetEXTProcPtr = function( program_: GLuint; location: GLint ): GLintptr;
{$elsec}
procedure glUniformBufferEXT( program_: GLuint; location: GLint; buffer: GLuint ); external name '_glUniformBufferEXT';
function glGetUniformBufferSizeEXT( program_: GLuint; location: GLint ): GLint; external name '_glGetUniformBufferSizeEXT';
function glGetUniformOffsetEXT( program_: GLuint; location: GLint ): GLintptr; external name '_glGetUniformOffsetEXT';
{$endc}
{$endc} { GL_EXT_bindable_uniform }

{$ifc not undefined GL_EXT_texture_integer and GL_EXT_texture_integer}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glClearColorIiEXTProcPtr = procedure( r: GLint; g: GLint; b: GLint; a: GLint );
	glClearColorIuiEXTProcPtr = procedure( r: GLuint; g: GLuint; b: GLuint; a: GLuint );
	glTexParameterIivEXTProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glTexParameterIuivEXTProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLuint );
	glGetTexParameterIivEXTProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLint );
	glGetTexParameterIuivEXTProcPtr = procedure( target: GLenum; pname: GLenum; params: PGLuint );
{$elsec}
procedure glClearColorIiEXT( r: GLint; g: GLint; b: GLint; a: GLint ); external name '_glClearColorIiEXT';
procedure glClearColorIuiEXT( r: GLuint; g: GLuint; b: GLuint; a: GLuint ); external name '_glClearColorIuiEXT';
procedure glTexParameterIivEXT( target: GLenum; pname: GLenum; params: PGLint ); external name '_glTexParameterIivEXT';
procedure glTexParameterIuivEXT( target: GLenum; pname: GLenum; params: PGLuint ); external name '_glTexParameterIuivEXT';
procedure glGetTexParameterIivEXT( target: GLenum; pname: GLenum; params: PGLint ); external name '_glGetTexParameterIivEXT';
procedure glGetTexParameterIuivEXT( target: GLenum; pname: GLenum; params: PGLuint ); external name '_glGetTexParameterIuivEXT';
{$endc}
{$endc} { GL_EXT_texture_integer }

{$ifc not undefined GL_EXT_gpu_shader4 and GL_EXT_gpu_shader4}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glVertexAttribI1iEXTProcPtr = procedure( index: GLuint; x: GLint );
	glVertexAttribI2iEXTProcPtr = procedure( index: GLuint; x: GLint; y: GLint );
	glVertexAttribI3iEXTProcPtr = procedure( index: GLuint; x: GLint; y: GLint; z: GLint );
	glVertexAttribI4iEXTProcPtr = procedure( index: GLuint; x: GLint; y: GLint; z: GLint; w: GLint );
	glVertexAttribI1uiEXTProcPtr = procedure( index: GLuint; x: GLuint );
	glVertexAttribI2uiEXTProcPtr = procedure( index: GLuint; x: GLuint; y: GLuint );
	glVertexAttribI3uiEXTProcPtr = procedure( index: GLuint; x: GLuint; y: GLuint; z: GLuint );
	glVertexAttribI4uiEXTProcPtr = procedure( index: GLuint; x: GLuint; y: GLuint; z: GLuint; w: GLuint );
	glVertexAttribI1ivEXTProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttribI2ivEXTProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttribI3ivEXTProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttribI4ivEXTProcPtr = procedure( index: GLuint; const v: PGLint );
	glVertexAttribI1uivEXTProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttribI2uivEXTProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttribI3uivEXTProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttribI4uivEXTProcPtr = procedure( index: GLuint; const v: PGLuint );
	glVertexAttribI4bvEXTProcPtr = procedure( index: GLuint; const v: PGLbyte );
	glVertexAttribI4svEXTProcPtr = procedure( index: GLuint; const v: PGLshort );
	glVertexAttribI4ubvEXTProcPtr = procedure( index: GLuint; const v: PGLubyte );
	glVertexAttribI4usvEXTProcPtr = procedure( index: GLuint; const v: PGLushort );
	glVertexAttribIPointerEXTProcPtr = procedure( index: GLuint; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glGetVertexAttribIivEXTProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLint );
	glGetVertexAttribIuivEXTProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLuint );
	glUniform1uiEXTProcPtr = procedure( location: GLint; v0: GLuint );
	glUniform2uiEXTProcPtr = procedure( location: GLint; v0: GLuint; v1: GLuint );
	glUniform3uiEXTProcPtr = procedure( location: GLint; v0: GLuint; v1: GLuint; v2: GLuint );
	glUniform4uiEXTProcPtr = procedure( location: GLint; v0: GLuint; v1: GLuint; v2: GLuint; v3: GLuint );
	glUniform1uivEXTProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLuint );
	glUniform2uivEXTProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLuint );
	glUniform3uivEXTProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLuint );
	glUniform4uivEXTProcPtr = procedure( location: GLint; count: GLsizei; const value: PGLuint );
	glGetUniformuivEXTProcPtr = procedure( program_: GLuint; location: GLint; params: PGLuint );
	glBindFragDataLocationEXTProcPtr = procedure( program_: GLuint; colorNumber: GLuint; const name: PChar );
	glGetFragDataLocationEXTProcPtr = function( program_: GLuint; const name: PChar ): GLint;
{$elsec}
procedure glVertexAttribI1iEXT( index: GLuint; x: GLint ); external name '_glVertexAttribI1iEXT';
procedure glVertexAttribI2iEXT( index: GLuint; x: GLint; y: GLint ); external name '_glVertexAttribI2iEXT';
procedure glVertexAttribI3iEXT( index: GLuint; x: GLint; y: GLint; z: GLint ); external name '_glVertexAttribI3iEXT';
procedure glVertexAttribI4iEXT( index: GLuint; x: GLint; y: GLint; z: GLint; w: GLint ); external name '_glVertexAttribI4iEXT';
procedure glVertexAttribI1uiEXT( index: GLuint; x: GLuint ); external name '_glVertexAttribI1uiEXT';
procedure glVertexAttribI2uiEXT( index: GLuint; x: GLuint; y: GLuint ); external name '_glVertexAttribI2uiEXT';
procedure glVertexAttribI3uiEXT( index: GLuint; x: GLuint; y: GLuint; z: GLuint ); external name '_glVertexAttribI3uiEXT';
procedure glVertexAttribI4uiEXT( index: GLuint; x: GLuint; y: GLuint; z: GLuint; w: GLuint ); external name '_glVertexAttribI4uiEXT';
procedure glVertexAttribI1ivEXT( index: GLuint; const v: PGLint ); external name '_glVertexAttribI1ivEXT';
procedure glVertexAttribI2ivEXT( index: GLuint; const v: PGLint ); external name '_glVertexAttribI2ivEXT';
procedure glVertexAttribI3ivEXT( index: GLuint; const v: PGLint ); external name '_glVertexAttribI3ivEXT';
procedure glVertexAttribI4ivEXT( index: GLuint; const v: PGLint ); external name '_glVertexAttribI4ivEXT';
procedure glVertexAttribI1uivEXT( index: GLuint; const v: PGLuint ); external name '_glVertexAttribI1uivEXT';
procedure glVertexAttribI2uivEXT( index: GLuint; const v: PGLuint ); external name '_glVertexAttribI2uivEXT';
procedure glVertexAttribI3uivEXT( index: GLuint; const v: PGLuint ); external name '_glVertexAttribI3uivEXT';
procedure glVertexAttribI4uivEXT( index: GLuint; const v: PGLuint ); external name '_glVertexAttribI4uivEXT';
procedure glVertexAttribI4bvEXT( index: GLuint; const v: PGLbyte ); external name '_glVertexAttribI4bvEXT';
procedure glVertexAttribI4svEXT( index: GLuint; const v: PGLshort ); external name '_glVertexAttribI4svEXT';
procedure glVertexAttribI4ubvEXT( index: GLuint; const v: PGLubyte ); external name '_glVertexAttribI4ubvEXT';
procedure glVertexAttribI4usvEXT( index: GLuint; const v: PGLushort ); external name '_glVertexAttribI4usvEXT';
procedure glVertexAttribIPointerEXT( index: GLuint; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glVertexAttribIPointerEXT';
procedure glGetVertexAttribIivEXT( index: GLuint; pname: GLenum; params: PGLint ); external name '_glGetVertexAttribIivEXT';
procedure glGetVertexAttribIuivEXT( index: GLuint; pname: GLenum; params: PGLuint ); external name '_glGetVertexAttribIuivEXT';
procedure glUniform1uiEXT( location: GLint; v0: GLuint ); external name '_glUniform1uiEXT';
procedure glUniform2uiEXT( location: GLint; v0: GLuint; v1: GLuint ); external name '_glUniform2uiEXT';
procedure glUniform3uiEXT( location: GLint; v0: GLuint; v1: GLuint; v2: GLuint ); external name '_glUniform3uiEXT';
procedure glUniform4uiEXT( location: GLint; v0: GLuint; v1: GLuint; v2: GLuint; v3: GLuint ); external name '_glUniform4uiEXT';
procedure glUniform1uivEXT( location: GLint; count: GLsizei; const value: PGLuint ); external name '_glUniform1uivEXT';
procedure glUniform2uivEXT( location: GLint; count: GLsizei; const value: PGLuint ); external name '_glUniform2uivEXT';
procedure glUniform3uivEXT( location: GLint; count: GLsizei; const value: PGLuint ); external name '_glUniform3uivEXT';
procedure glUniform4uivEXT( location: GLint; count: GLsizei; const value: PGLuint ); external name '_glUniform4uivEXT';
procedure glGetUniformuivEXT( program_: GLuint; location: GLint; params: PGLuint ); external name '_glGetUniformuivEXT';
procedure glBindFragDataLocationEXT( program_: GLuint; colorNumber: GLuint; const name: PChar ); external name '_glBindFragDataLocationEXT';
function glGetFragDataLocationEXT( program_: GLuint; const name: PChar ): GLint; external name '_glGetFragDataLocationEXT';
{$endc}
{$endc} { GL_EXT_gpu_shader4 }

{$ifc not undefined GL_EXT_draw_buffers2 and GL_EXT_draw_buffers2}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glColorMaskIndexedEXTProcPtr = procedure( index: GLuint; r: GLboolean; g: GLboolean; b: GLboolean; a: GLboolean );
	glEnableIndexedEXTProcPtr = procedure( target: GLenum; index: GLuint );
	glDisableIndexedEXTProcPtr = procedure( target: GLenum; index: GLuint );
	glIsEnabledIndexedEXTProcPtr = function( target: GLenum; index: GLuint ): GLboolean;
{$elsec}
procedure glColorMaskIndexedEXT( index: GLuint; r: GLboolean; g: GLboolean; b: GLboolean; a: GLboolean ); external name '_glColorMaskIndexedEXT';
procedure glEnableIndexedEXT( target: GLenum; index: GLuint ); external name '_glEnableIndexedEXT';
procedure glDisableIndexedEXT( target: GLenum; index: GLuint ); external name '_glDisableIndexedEXT';
function glIsEnabledIndexedEXT( target: GLenum; index: GLuint ): GLboolean; external name '_glIsEnabledIndexedEXT';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_texture_range and GL_APPLE_texture_range}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTextureRangeAPPLEProcPtr = procedure( target: GLenum; length: GLsizei; const pointr: UnivPtr );
	glGetTexParameterPointervAPPLEProcPtr = procedure( target: GLenum; pname: GLenum; params: UnivPtrPtr );
{$elsec}
procedure glTextureRangeAPPLE( target: GLenum; length: GLsizei; const pointr: UnivPtr ); external name '_glTextureRangeAPPLE';
procedure glGetTexParameterPointervAPPLE( target: GLenum; pname: GLenum; params: UnivPtrPtr ); external name '_glGetTexParameterPointervAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_vertex_array_range and GL_APPLE_vertex_array_range}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glVertexArrayRangeAPPLEProcPtr = procedure( length: GLsizei; const pointr: UnivPtr );
	glFlushVertexArrayRangeAPPLEProcPtr = procedure( length: GLsizei; const pointr: UnivPtr );
	glVertexArrayParameteriAPPLEProcPtr = procedure( pname: GLenum; param: GLint );
{$elsec}
procedure glVertexArrayRangeAPPLE( length: GLsizei; const pointr: UnivPtr ); external name '_glVertexArrayRangeAPPLE';
procedure glFlushVertexArrayRangeAPPLE( length: GLsizei; const pointr: UnivPtr ); external name '_glFlushVertexArrayRangeAPPLE';
procedure glVertexArrayParameteriAPPLE( pname: GLenum; param: GLint ); external name '_glVertexArrayParameteriAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_vertex_array_object and GL_APPLE_vertex_array_object}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBindVertexArrayAPPLEProcPtr = procedure( id_: GLuint );
	glDeleteVertexArraysAPPLEProcPtr = procedure( n: GLsizei; const ids: PGLuint );
	glGenVertexArraysAPPLEProcPtr = procedure( n: GLsizei; ids: PGLuint );
	glIsVertexArrayAPPLEProcPtr = function( id_: GLuint ): GLboolean;
{$elsec}
procedure glBindVertexArrayAPPLE( id_: GLuint ); external name '_glBindVertexArrayAPPLE';
procedure glDeleteVertexArraysAPPLE( n: GLsizei; const ids: PGLuint ); external name '_glDeleteVertexArraysAPPLE';
procedure glGenVertexArraysAPPLE( n: GLsizei; ids: PGLuint ); external name '_glGenVertexArraysAPPLE';
function glIsVertexArrayAPPLE( id_: GLuint ): GLboolean; external name '_glIsVertexArrayAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_fence and GL_APPLE_fence}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGenFencesAPPLEProcPtr = procedure( n: GLsizei; fences: PGLuint );
	glDeleteFencesAPPLEProcPtr = procedure( n: GLsizei; const fences: PGLuint );
	glSetFenceAPPLEProcPtr = procedure( fence: GLuint );
	glIsFenceAPPLEProcPtr = function( fence: GLuint ): GLboolean;
	glTestFenceAPPLEProcPtr = function( fence: GLuint ): GLboolean;
	glFinishFenceAPPLEProcPtr = procedure( fence: GLuint );
	glTestObjectAPPLEProcPtr = function( objct: GLenum; name: GLuint ): GLboolean;
	glFinishObjectAPPLEProcPtr = procedure( objct: GLenum; name: GLuint );
{$elsec}
procedure glGenFencesAPPLE( n: GLsizei; fences: PGLuint ); external name '_glGenFencesAPPLE';
procedure glDeleteFencesAPPLE( n: GLsizei; const fences: PGLuint ); external name '_glDeleteFencesAPPLE';
procedure glSetFenceAPPLE( fence: GLuint ); external name '_glSetFenceAPPLE';
function glIsFenceAPPLE( fence: GLuint ): GLboolean; external name '_glIsFenceAPPLE';
function glTestFenceAPPLE( fence: GLuint ): GLboolean; external name '_glTestFenceAPPLE';
procedure glFinishFenceAPPLE( fence: GLuint ); external name '_glFinishFenceAPPLE';
function glTestObjectAPPLE( objct: GLenum; name: GLuint ): GLboolean; external name '_glTestObjectAPPLE';
procedure glFinishObjectAPPLE( objct: GLenum; name: GLuint ); external name '_glFinishObjectAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_element_array and GL_APPLE_element_array}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glElementPointerAPPLEProcPtr = procedure( typ: GLenum; const pointr: UnivPtr );
	glDrawElementArrayAPPLEProcPtr = procedure( mode: GLenum; first: GLint; count: GLsizei );
	glDrawRangeElementArrayAPPLEProcPtr = procedure( mode: GLenum; start: GLuint; finish: GLuint; first: GLint; count: GLsizei );
	glMultiDrawElementArrayAPPLEProcPtr = procedure( mode: GLenum; const first: PGLint; const count: PGLsizei; primcount: GLsizei );
	glMultiDrawRangeElementArrayAPPLEProcPtr = procedure( mode: GLenum; start: GLuint; finish: GLuint; const first: PGLint; const count: PGLsizei; primcount: GLsizei );
{$elsec}
procedure glElementPointerAPPLE( typ: GLenum; const pointr: UnivPtr ); external name '_glElementPointerAPPLE';
procedure glDrawElementArrayAPPLE( mode: GLenum; first: GLint; count: GLsizei ); external name '_glDrawElementArrayAPPLE';
procedure glDrawRangeElementArrayAPPLE( mode: GLenum; start: GLuint; finish: GLuint; first: GLint; count: GLsizei ); external name '_glDrawRangeElementArrayAPPLE';
procedure glMultiDrawElementArrayAPPLE( mode: GLenum; const first: PGLint; const count: PGLsizei; primcount: GLsizei ); external name '_glMultiDrawElementArrayAPPLE';
procedure glMultiDrawRangeElementArrayAPPLE( mode: GLenum; start: GLuint; finish: GLuint; const first: PGLint; const count: PGLsizei; primcount: GLsizei ); external name '_glMultiDrawRangeElementArrayAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_flush_render and GL_APPLE_flush_render}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFlushRenderAPPLEProcPtr = procedure;
	glFinishRenderAPPLEProcPtr = procedure;
	glSwapAPPLEProcPtr = procedure;
{$elsec}
procedure glFlushRenderAPPLE; external name '_glFlushRenderAPPLE';
procedure glFinishRenderAPPLE; external name '_glFinishRenderAPPLE';
procedure glSwapAPPLE; external name '_glSwapAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_vertex_program_evaluators and GL_APPLE_vertex_program_evaluators}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glEnableVertexAttribAPPLEProcPtr = procedure( index: GLuint; pname: GLenum );
	glDisableVertexAttribAPPLEProcPtr = procedure( index: GLuint; pname: GLenum );
	glIsVertexAttribEnabledAPPLEProcPtr = function( index: GLuint; pname: GLenum ): GLboolean;
	glMapVertexAttrib1dAPPLEProcPtr = procedure( index: GLuint; size: GLuint; u1: GLdouble; u2: GLdouble; stride: GLint; order: GLint; const points: PGLdouble );
	glMapVertexAttrib1fAPPLEProcPtr = procedure( index: GLuint; size: GLuint; u1: GLfloat; u2: GLfloat; stride: GLint; order: GLint; const points: PGLfloat );
	glMapVertexAttrib2dAPPLEProcPtr = procedure( index: GLuint; size: GLuint; u1: GLdouble; u2: GLdouble; ustride: GLint; uorder: GLint; v1: GLdouble; v2: GLdouble; vstride: GLint; vorder: GLint; const points: PGLdouble );
	glMapVertexAttrib2fAPPLEProcPtr = procedure( index: GLuint; size: GLuint; u1: GLfloat; u2: GLfloat; ustride: GLint; uorder: GLint; v1: GLfloat; v2: GLfloat; vstride: GLint; vorder: GLint; const points: PGLfloat );
{$elsec}
procedure glEnableVertexAttribAPPLE( index: GLuint; pname: GLenum ); external name '_glEnableVertexAttribAPPLE';
procedure glDisableVertexAttribAPPLE( index: GLuint; pname: GLenum ); external name '_glDisableVertexAttribAPPLE';
function glIsVertexAttribEnabledAPPLE( index: GLuint; pname: GLenum ): GLboolean; external name '_glIsVertexAttribEnabledAPPLE';
procedure glMapVertexAttrib1dAPPLE( index: GLuint; size: GLuint; u1: GLdouble; u2: GLdouble; stride: GLint; order: GLint; const points: PGLdouble ); external name '_glMapVertexAttrib1dAPPLE';
procedure glMapVertexAttrib1fAPPLE( index: GLuint; size: GLuint; u1: GLfloat; u2: GLfloat; stride: GLint; order: GLint; const points: PGLfloat ); external name '_glMapVertexAttrib1fAPPLE';
procedure glMapVertexAttrib2dAPPLE( index: GLuint; size: GLuint; u1: GLdouble; u2: GLdouble; ustride: GLint; uorder: GLint; v1: GLdouble; v2: GLdouble; vstride: GLint; vorder: GLint; const points: PGLdouble ); external name '_glMapVertexAttrib2dAPPLE';
procedure glMapVertexAttrib2fAPPLE( index: GLuint; size: GLuint; u1: GLfloat; u2: GLfloat; ustride: GLint; uorder: GLint; v1: GLfloat; v2: GLfloat; vstride: GLint; vorder: GLint; const points: PGLfloat ); external name '_glMapVertexAttrib2fAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_flush_buffer_range and GL_APPLE_flush_buffer_range}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBufferParameteriAPPLEProcPtr = procedure( target: GLenum; pname: GLenum; param: GLint );
	glFlushMappedBufferRangeAPPLEProcPtr = procedure( target: GLenum; offset: GLintptr; size: GLsizeiptr );
{$elsec}
procedure glBufferParameteriAPPLE( target: GLenum; pname: GLenum; param: GLint ); external name '_glBufferParameteriAPPLE';
procedure glFlushMappedBufferRangeAPPLE( target: GLenum; offset: GLintptr; size: GLsizeiptr ); external name '_glFlushMappedBufferRangeAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_object_purgeable and GL_APPLE_object_purgeable}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glObjectPurgeableAPPLEProcPtr = function( objectType: GLenum; name: GLuint; option: GLenum ): GLenum;
	glObjectUnpurgeableAPPLEProcPtr = function( objectType: GLenum; name: GLuint; option: GLenum ): GLenum;
	glGetObjectParameterivAPPLEProcPtr = procedure( objectType: GLenum; name: GLuint; pname: GLenum; params: PGLint );
{$elsec}
function glObjectPurgeableAPPLE( objectType: GLenum; name: GLuint; option: GLenum ): GLenum; external name '_glObjectPurgeableAPPLE';
function glObjectUnpurgeableAPPLE( objectType: GLenum; name: GLuint; option: GLenum ): GLenum; external name '_glObjectUnpurgeableAPPLE';
procedure glGetObjectParameterivAPPLE( objectType: GLenum; name: GLuint; pname: GLenum; params: PGLint ); external name '_glGetObjectParameterivAPPLE';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_APPLE_vertex_point_size and GL_APPLE_vertex_point_size}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPointSizePointerAPPLEProcPtr = procedure( typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glVertexPointSizefAPPLEProcPtr = procedure( size: GLfloat );
{$elsec}
procedure glPointSizePointerAPPLE( typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glPointSizePointerAPPLE';
procedure glVertexPointSizefAPPLE( size: GLfloat ); external name '_glVertexPointSizefAPPLE';
{$endc}
{$endc}

{$ifc not undefined GL_ATI_pn_triangles and GL_ATI_pn_triangles}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPNTrianglesiATIProcPtr = procedure( pname: GLenum; param: GLint );
	glPNTrianglesfATIProcPtr = procedure( pname: GLenum; param: GLfloat );
{$elsec}
procedure glPNTrianglesiATI( pname: GLenum; param: GLint ); external name '_glPNTrianglesiATI';
procedure glPNTrianglesfATI( pname: GLenum; param: GLfloat ); external name '_glPNTrianglesfATI';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ATI_blend_equation_separate and GL_ATI_blend_equation_separate}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBlendEquationSeparateATIProcPtr = procedure( equationRGB: GLenum; equationAlpha: GLenum );
{$elsec}
procedure glBlendEquationSeparateATI( equationRGB: GLenum; equationAlpha: GLenum ); external name '_glBlendEquationSeparateATI';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ATI_separate_stencil and GL_ATI_separate_stencil}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glStencilOpSeparateATIProcPtr = procedure( face: GLenum; sfail: GLenum; dpfail: GLenum; dppass: GLenum );
	glStencilFuncSeparateATIProcPtr = procedure( frontfunc: GLenum; backfunc: GLenum; ref: GLint; mask: GLuint );
{$elsec}
procedure glStencilOpSeparateATI( face: GLenum; sfail: GLenum; dpfail: GLenum; dppass: GLenum ); external name '_glStencilOpSeparateATI';
procedure glStencilFuncSeparateATI( frontfunc: GLenum; backfunc: GLenum; ref: GLint; mask: GLuint ); external name '_glStencilFuncSeparateATI';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_ATIX_pn_triangles and GL_ATIX_pn_triangles}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPNTrianglesiATIXProcPtr = procedure( pname: GLenum; param: GLint );
	glPNTrianglesfATIXProcPtr = procedure( pname: GLenum; param: GLfloat );
{$elsec}
procedure glPNTrianglesiATIX( pname: GLenum; param: GLint ); external name '_glPNTrianglesiATIX';
procedure glPNTrianglesfATIX( pname: GLenum; param: GLfloat ); external name '_glPNTrianglesfATIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_HP_image_transform and GL_HP_image_transform}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glImageTransformParameteriHPProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint );
	glImageTransformParameterfHPProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLfloat );
	glImageTransformParameterivHPProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLint );
	glImageTransformParameterfvHPProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
	glGetImageTransformParameterivHPProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glGetImageTransformParameterfvHPProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
{$elsec}
procedure glImageTransformParameteriHP( param1 : GLenum; param2 : GLenum; param3 : GLint ); external name '_glImageTransformParameteriHP';
procedure glImageTransformParameterfHP( param1 : GLenum; param2 : GLenum; param3 : GLfloat ); external name '_glImageTransformParameterfHP';
procedure glImageTransformParameterivHP( param1 : GLenum; param2 : GLenum; const param3 : PGLint ); external name '_glImageTransformParameterivHP';
procedure glImageTransformParameterfvHP( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glImageTransformParameterfvHP';
procedure glGetImageTransformParameterivHP( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetImageTransformParameterivHP';
procedure glGetImageTransformParameterfvHP( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetImageTransformParameterfvHP';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_IBM_multimode_draw_arrays and GL_IBM_multimode_draw_arrays}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glMultiModeDrawArraysIBMProcPtr = procedure( param1 : GLenum; const param2 : PGLint; const param3 : PGLsizei; param4 : GLsizei; param5 : GLint );
	glMultiModeDrawElementsIBMProcPtr = procedure( const param1 : PGLenum; const param2 : PGLsizei; param3 : GLenum; const param4 : UnivPtrPtr; param5 : GLsizei; param6 : GLint );
{$elsec}
procedure glMultiModeDrawArraysIBM( param1 : GLenum; const param2 : PGLint; const param3 : PGLsizei; param4 : GLsizei; param5 : GLint ); external name '_glMultiModeDrawArraysIBM';
procedure glMultiModeDrawElementsIBM( const param1 : PGLenum; const param2 : PGLsizei; param3 : GLenum; const param4 : UnivPtrPtr; param5 : GLsizei; param6 : GLint ); external name '_glMultiModeDrawElementsIBM';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}
	
{$ifc not undefined GL_IBM_vertex_array_lists and GL_IBM_vertex_array_lists}
type
	PGLbooleanPtr = ^PGLboolean;
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glColorPointerListIBMProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint );
	glSecondaryColorPointerListIBMProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint );
	glEdgeFlagPointerListIBMProcPtr = procedure( param1 : GLint; const param2 : PGLbooleanPtr; param3 : GLint );
	glFogCoordPointerListIBMProcPtr = procedure( param1 : GLenum; param2 : GLint; const param3 : UnivPtrPtr; param4 : GLint );
	glIndexPointerListIBMProcPtr = procedure( param1 : GLenum; param2 : GLint; const param3 : UnivPtrPtr; param4 : GLint );
	glNormalPointerListIBMProcPtr = procedure( param1 : GLenum; param2 : GLint; const param3 : UnivPtrPtr; param4 : GLint );
	glTexCoordPointerListIBMProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint );
	glVertexPointerListIBMProcPtr = procedure( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint );
{$elsec}
procedure glColorPointerListIBM( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint ); external name '_glColorPointerListIBM';
procedure glSecondaryColorPointerListIBM( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint ); external name '_glSecondaryColorPointerListIBM';
procedure glEdgeFlagPointerListIBM( param1 : GLint; const param2 : PGLbooleanPtr; param3 : GLint ); external name '_glEdgeFlagPointerListIBM';
procedure glFogCoordPointerListIBM( param1 : GLenum; param2 : GLint; const param3 : UnivPtrPtr; param4 : GLint ); external name '_glFogCoordPointerListIBM';
procedure glIndexPointerListIBM( param1 : GLenum; param2 : GLint; const param3 : UnivPtrPtr; param4 : GLint ); external name '_glIndexPointerListIBM';
procedure glNormalPointerListIBM( param1 : GLenum; param2 : GLint; const param3 : UnivPtrPtr; param4 : GLint ); external name '_glNormalPointerListIBM';
procedure glTexCoordPointerListIBM( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint ); external name '_glTexCoordPointerListIBM';
procedure glVertexPointerListIBM( param1 : GLint; param2 : GLenum; param3 : GLint; const param4 : UnivPtrPtr; param5 : GLint ); external name '_glVertexPointerListIBM';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_INTEL_parallel_arrays and GL_INTEL_parallel_arrays}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glVertexPointervINTELProcPtr = procedure( param1 : GLint; param2 : GLenum; const param3 : UnivPtrPtr );
	glNormalPointervINTELProcPtr = procedure( param1 : GLenum; const param2 : UnivPtrPtr );
	glColorPointervINTELProcPtr = procedure( param1 : GLint; param2 : GLenum; const param3 : UnivPtrPtr );
	glTexCoordPointervINTELProcPtr = procedure( param1 : GLint; param2 : GLenum; const param3 : UnivPtrPtr );
{$elsec}
procedure glVertexPointervINTEL( param1 : GLint; param2 : GLenum; const param3 : UnivPtrPtr ); external name '_glVertexPointervINTEL';
procedure glNormalPointervINTEL( param1 : GLenum; const param2 : UnivPtrPtr ); external name '_glNormalPointervINTEL';
procedure glColorPointervINTEL( param1 : GLint; param2 : GLenum; const param3 : UnivPtrPtr ); external name '_glColorPointervINTEL';
procedure glTexCoordPointervINTEL( param1 : GLint; param2 : GLenum; const param3 : UnivPtrPtr ); external name '_glTexCoordPointervINTEL';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_MESA_resize_buffers and GL_MESA_resize_buffers}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glResizeBuffersMESAProcPtr = procedure;
{$elsec}
procedure glResizeBuffersMESA; external name '_glResizeBuffersMESA';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_NV_vertex_array_range and GL_NV_vertex_array_range}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFlushVertexArrayRangeNVProcPtr = procedure;
	glVertexArrayRangeNVProcPtr = procedure( param1 : GLsizei; const UnivPtr );
{$elsec}
procedure glFlushVertexArrayRangeNV; external name '_glFlushVertexArrayRangeNV';
procedure glVertexArrayRangeNV( param1 :GLsizei; const param2 :UnivPtr ); external name '_glVertexArrayRangeNV';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_NV_register_combiners and GL_NV_register_combiners}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glCombinerParameterfvNVProcPtr = procedure( param1 : GLenum; const param1 : PGLfloat );
	glCombinerParameterfNVProcPtr = procedure( param1 : GLenum; param1 : GLfloat );
	glCombinerParameterivNVProcPtr = procedure( param1 : GLenum; const param1 : PGLint );
	glCombinerParameteriNVProcPtr = procedure( param1 : GLenum; param1 : GLint );
	glCombinerInputNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : GLenum; param6 : GLenum );
	glCombinerOutputNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : GLenum; param6 : GLenum; param7 : GLenum; param8 : GLboolean; param9 : GLboolean; param10 : GLboolean );
	glFinalCombinerInputNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum );
	glGetCombinerInputParameterfvNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : PGLfloat );
	glGetCombinerInputParameterivNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : PGLint );
	glGetCombinerOutputParameterfvNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : PGLfloat );
	glGetCombinerOutputParameterivNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : PGLint );
	glGetFinalCombinerInputParameterfvNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetFinalCombinerInputParameterivNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
{$elsec}
procedure glCombinerParameterfvNV( param1 : GLenum; const param1 : PGLfloat ); external name '_glCombinerParameterfvNV';
procedure glCombinerParameterfNV( param1 : GLenum; param1 : GLfloat ); external name '_glCombinerParameterfNV';
procedure glCombinerParameterivNV( param1 : GLenum; const param1 : PGLint ); external name '_glCombinerParameterivNV';
procedure glCombinerParameteriNV( param1 : GLenum; param1 : GLint ); external name '_glCombinerParameteriNV';
procedure glCombinerInputNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : GLenum; param6 : GLenum ); external name '_glCombinerInputNV';
procedure glCombinerOutputNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : GLenum; param6 : GLenum; param7 : GLenum; param8 : GLboolean; param9 : GLboolean; param10 : GLboolean ); external name '_glCombinerOutputNV';
procedure glFinalCombinerInputNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum ); external name '_glFinalCombinerInputNV';
procedure glGetCombinerInputParameterfvNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : PGLfloat ); external name '_glGetCombinerInputParameterfvNV';
procedure glGetCombinerInputParameterivNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : GLenum; param5 : PGLint ); external name '_glGetCombinerInputParameterivNV';
procedure glGetCombinerOutputParameterfvNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : PGLfloat ); external name '_glGetCombinerOutputParameterfvNV';
procedure glGetCombinerOutputParameterivNV( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : PGLint ); external name '_glGetCombinerOutputParameterivNV';
procedure glGetFinalCombinerInputParameterfvNV( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetFinalCombinerInputParameterfvNV';
procedure glGetFinalCombinerInputParameterivNV( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetFinalCombinerInputParameterivNV';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_NV_register_combiners2 and GL_NV_register_combiners2}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glCombinerStageParameterfvNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
	glGetCombinerStageParameterfvNVProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
{$elsec}
procedure glCombinerStageParameterfvNV( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glCombinerStageParameterfvNV';
procedure glGetCombinerStageParameterfvNV( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetCombinerStageParameterfvNV';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_NV_vertex_program and GL_NV_vertex_program}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBindProgramNVProcPtr = procedure( target: GLenum; id_: GLuint );
	glDeleteProgramsNVProcPtr = procedure( n: GLsizei; const ids: PGLuint );
	glExecuteProgramNVProcPtr = procedure( target: GLenum; id_: GLuint; const params: PGLfloat );
	glGenProgramsNVProcPtr = procedure( n: GLsizei; ids: PGLuint );
	glAreProgramsResidentNVProcPtr = function( n: GLsizei; const ids: PGLuint; residences: PGLboolean ): GLboolean;
	glRequestResidentProgramsNVProcPtr = procedure( n: GLsizei; ids: PGLuint );
	glGetProgramParameterfvNVProcPtr = procedure( target: GLenum; index: GLuint; pname: GLenum; params: PGLfloat );
	glGetProgramParameterdvNVProcPtr = procedure( target: GLenum; index: GLuint; pname: GLenum; params: PGLdouble );
	glGetProgramivNVProcPtr = procedure( id_: GLuint; pname: GLenum; params: PGLint );
	glGetProgramStringNVProcPtr = procedure( id_: GLuint; pname: GLenum; program_: PGLubyte );
	glGetTrackMatrixivNVProcPtr = procedure( target: GLenum; address: GLuint; pname: GLenum; params: PGLint );
	glGetVertexAttribdvNVProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLdouble );
	glGetVertexAttribfvNVProcPtr = procedure( index: GLuint; pname: GLenum; params: PGLfloat );
	glGetVertexAttribivNVProcPtr = procedure( index: GLuint; pname: GLenum; var params: GLint );
	glGetVertexAttribPointervNVProcPtr = procedure( index: GLuint; pname: GLenum; pointr: UnivPtrPtr );
	glIsProgramNVProcPtr = function( id_: GLuint ): GLboolean;
	glLoadProgramNVProcPtr = procedure( target: GLenum; id_: GLuint; len: GLsizei; const program_: PGLubyte );
	glProgramParameter4fNVProcPtr = procedure( target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glProgramParameter4dNVProcPtr = procedure( target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glProgramParameter4dvNVProcPtr = procedure( target: GLenum; index: GLuint; const params: PGLdouble );
	glProgramParameter4fvNVProcPtr = procedure( target: GLenum; index: GLuint; const params: PGLfloat );
	glProgramParameters4dvNVProcPtr = procedure( target: GLenum; index: GLuint; num: GLuint; const params: PGLdouble );
	glProgramParameters4fvNVProcPtr = procedure( target: GLenum; index: GLuint; num: GLuint; const params: PGLfloat );
	glTrackMatrixNVProcPtr = procedure( target: GLenum; address: GLuint; matrix: GLenum; transform: GLenum );
	glVertexAttribPointerNVProcPtr = procedure( index: GLuint; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr );
	glVertexAttrib1sNVProcPtr = procedure( index: GLuint; x: GLshort );
	glVertexAttrib1fNVProcPtr = procedure( index: GLuint; x: GLfloat );
	glVertexAttrib1dNVProcPtr = procedure( index: GLuint; x: GLdouble );
	glVertexAttrib2sNVProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort );
	glVertexAttrib2fNVProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat );
	glVertexAttrib2dNVProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble );
	glVertexAttrib3sNVProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort; z: GLshort );
	glVertexAttrib3fNVProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat );
	glVertexAttrib3dNVProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble );
	glVertexAttrib4sNVProcPtr = procedure( index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort );
	glVertexAttrib4fNVProcPtr = procedure( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat );
	glVertexAttrib4dNVProcPtr = procedure( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble );
	glVertexAttrib4ubNVProcPtr = procedure( index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte );
	glVertexAttrib1svNVProcPtr = procedure( index: GLuint; v: PGLshort );
	glVertexAttrib1fvNVProcPtr = procedure( index: GLuint; v: PGLfloat );
	glVertexAttrib1dvNVProcPtr = procedure( index: GLuint; v: PGLdouble );
	glVertexAttrib2svNVProcPtr = procedure( index: GLuint; v: PGLshort );
	glVertexAttrib2fvNVProcPtr = procedure( index: GLuint; v: PGLfloat );
	glVertexAttrib2dvNVProcPtr = procedure( index: GLuint; v: PGLdouble );
	glVertexAttrib3svNVProcPtr = procedure( index: GLuint; v: PGLshort );
	glVertexAttrib3fvNVProcPtr = procedure( index: GLuint; v: PGLfloat );
	glVertexAttrib3dvNVProcPtr = procedure( index: GLuint; v: PGLdouble );
	glVertexAttrib4svNVProcPtr = procedure( index: GLuint; v: PGLshort );
	glVertexAttrib4fvNVProcPtr = procedure( index: GLuint; v: PGLfloat );
	glVertexAttrib4dvNVProcPtr = procedure( index: GLuint; v: PGLdouble );
	glVertexAttrib4ubvNVProcPtr = procedure( index: GLuint; v: PGLubyte );
	glVertexAttribs1svNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLshort );
	glVertexAttribs1fvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLfloat );
	glVertexAttribs1dvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLdouble );
	glVertexAttribs2svNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLshort );
	glVertexAttribs2fvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLfloat );
	glVertexAttribs2dvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLdouble );
	glVertexAttribs3svNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLshort );
	glVertexAttribs3fvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLfloat );
	glVertexAttribs3dvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLdouble );
	glVertexAttribs4svNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLshort );
	glVertexAttribs4fvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLfloat );
	glVertexAttribs4dvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLdouble );
	glVertexAttribs4ubvNVProcPtr = procedure( index: GLuint; n: GLsizei; v: PGLubyte );
{$elsec}
procedure glBindProgramNV( target: GLenum; id_: GLuint ); external name '_glBindProgramNV';
procedure glDeleteProgramsNV( n: GLsizei; const ids: PGLuint ); external name '_glDeleteProgramsNV';
procedure glExecuteProgramNV( target: GLenum; id_: GLuint; const params: PGLfloat ); external name '_glExecuteProgramNV';
procedure glGenProgramsNV( n: GLsizei; ids: PGLuint ); external name '_glGenProgramsNV';
function glAreProgramsResidentNV( n: GLsizei; const ids: PGLuint; residences: PGLboolean ): GLboolean; external name '_glAreProgramsResidentNV';
procedure glRequestResidentProgramsNV( n: GLsizei; ids: PGLuint ); external name '_glRequestResidentProgramsNV';
procedure glGetProgramParameterfvNV( target: GLenum; index: GLuint; pname: GLenum; params: PGLfloat ); external name '_glGetProgramParameterfvNV';
procedure glGetProgramParameterdvNV( target: GLenum; index: GLuint; pname: GLenum; params: PGLdouble ); external name '_glGetProgramParameterdvNV';
procedure glGetProgramivNV( id_: GLuint; pname: GLenum; params: PGLint ); external name '_glGetProgramivNV';
procedure glGetProgramStringNV( id_: GLuint; pname: GLenum; program_: PGLubyte ); external name '_glGetProgramStringNV';
procedure glGetTrackMatrixivNV( target: GLenum; address: GLuint; pname: GLenum; params: PGLint ); external name '_glGetTrackMatrixivNV';
procedure glGetVertexAttribdvNV( index: GLuint; pname: GLenum; params: PGLdouble ); external name '_glGetVertexAttribdvNV';
procedure glGetVertexAttribfvNV( index: GLuint; pname: GLenum; params: PGLfloat ); external name '_glGetVertexAttribfvNV';
procedure glGetVertexAttribivNV( index: GLuint; pname: GLenum; params: PGLint ); external name '_glGetVertexAttribivNV';
procedure glGetVertexAttribPointervNV( index: GLuint; pname: GLenum; pointr: UnivPtrPtr ); external name '_glGetVertexAttribPointervNV';
function glIsProgramNV( id_: GLuint ): GLboolean; external name '_glIsProgramNV';
procedure glLoadProgramNV( target: GLenum; id_: GLuint; len: GLsizei; const program_: PGLubyte ); external name '_glLoadProgramNV';
procedure glProgramParameter4fNV( target: GLenum; index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glProgramParameter4fNV';
procedure glProgramParameter4dNV( target: GLenum; index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glProgramParameter4dNV';
procedure glProgramParameter4dvNV( target: GLenum; index: GLuint; const params: PGLdouble ); external name '_glProgramParameter4dvNV';
procedure glProgramParameter4fvNV( target: GLenum; index: GLuint; const params: PGLfloat ); external name '_glProgramParameter4fvNV';
procedure glProgramParameters4dvNV( target: GLenum; index: GLuint; num: GLuint; const params: PGLdouble ); external name '_glProgramParameters4dvNV';
procedure glProgramParameters4fvNV( target: GLenum; index: GLuint; num: GLuint; const params: PGLfloat ); external name '_glProgramParameters4fvNV';
procedure glTrackMatrixNV( target: GLenum; address: GLuint; matrix: GLenum; transform: GLenum ); external name '_glTrackMatrixNV';
procedure glVertexAttribPointerNV( index: GLuint; size: GLint; typ: GLenum; stride: GLsizei; const pointr: UnivPtr ); external name '_glVertexAttribPointerNV';
procedure glVertexAttrib1sNV( index: GLuint; x: GLshort ); external name '_glVertexAttrib1sNV';
procedure glVertexAttrib1fNV( index: GLuint; x: GLfloat ); external name '_glVertexAttrib1fNV';
procedure glVertexAttrib1dNV( index: GLuint; x: GLdouble ); external name '_glVertexAttrib1dNV';
procedure glVertexAttrib2sNV( index: GLuint; x: GLshort; y: GLshort ); external name '_glVertexAttrib2sNV';
procedure glVertexAttrib2fNV( index: GLuint; x: GLfloat; y: GLfloat ); external name '_glVertexAttrib2fNV';
procedure glVertexAttrib2dNV( index: GLuint; x: GLdouble; y: GLdouble ); external name '_glVertexAttrib2dNV';
procedure glVertexAttrib3sNV( index: GLuint; x: GLshort; y: GLshort; z: GLshort ); external name '_glVertexAttrib3sNV';
procedure glVertexAttrib3fNV( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat ); external name '_glVertexAttrib3fNV';
procedure glVertexAttrib3dNV( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble ); external name '_glVertexAttrib3dNV';
procedure glVertexAttrib4sNV( index: GLuint; x: GLshort; y: GLshort; z: GLshort; w: GLshort ); external name '_glVertexAttrib4sNV';
procedure glVertexAttrib4fNV( index: GLuint; x: GLfloat; y: GLfloat; z: GLfloat; w: GLfloat ); external name '_glVertexAttrib4fNV';
procedure glVertexAttrib4dNV( index: GLuint; x: GLdouble; y: GLdouble; z: GLdouble; w: GLdouble ); external name '_glVertexAttrib4dNV';
procedure glVertexAttrib4ubNV( index: GLuint; x: GLubyte; y: GLubyte; z: GLubyte; w: GLubyte ); external name '_glVertexAttrib4ubNV';
procedure glVertexAttrib1svNV( index: GLuint; v: PGLshort ); external name '_glVertexAttrib1svNV';
procedure glVertexAttrib1fvNV( index: GLuint; v: PGLfloat ); external name '_glVertexAttrib1fvNV';
procedure glVertexAttrib1dvNV( index: GLuint; v: PGLdouble ); external name '_glVertexAttrib1dvNV';
procedure glVertexAttrib2svNV( index: GLuint; v: PGLshort ); external name '_glVertexAttrib2svNV';
procedure glVertexAttrib2fvNV( index: GLuint; v: PGLfloat ); external name '_glVertexAttrib2fvNV';
procedure glVertexAttrib2dvNV( index: GLuint; v: PGLdouble ); external name '_glVertexAttrib2dvNV';
procedure glVertexAttrib3svNV( index: GLuint; v: PGLshort ); external name '_glVertexAttrib3svNV';
procedure glVertexAttrib3fvNV( index: GLuint; v: PGLfloat ); external name '_glVertexAttrib3fvNV';
procedure glVertexAttrib3dvNV( index: GLuint; v: PGLdouble ); external name '_glVertexAttrib3dvNV';
procedure glVertexAttrib4svNV( index: GLuint; v: PGLshort ); external name '_glVertexAttrib4svNV';
procedure glVertexAttrib4fvNV( index: GLuint; v: PGLfloat ); external name '_glVertexAttrib4fvNV';
procedure glVertexAttrib4dvNV( index: GLuint; v: PGLdouble ); external name '_glVertexAttrib4dvNV';
procedure glVertexAttrib4ubvNV( index: GLuint; v: PGLubyte ); external name '_glVertexAttrib4ubvNV';
procedure glVertexAttribs1svNV( index: GLuint; n: GLsizei; v: PGLshort ); external name '_glVertexAttribs1svNV';
procedure glVertexAttribs1fvNV( index: GLuint; n: GLsizei; v: PGLfloat ); external name '_glVertexAttribs1fvNV';
procedure glVertexAttribs1dvNV( index: GLuint; n: GLsizei; v: PGLdouble ); external name '_glVertexAttribs1dvNV';
procedure glVertexAttribs2svNV( index: GLuint; n: GLsizei; v: PGLshort ); external name '_glVertexAttribs2svNV';
procedure glVertexAttribs2fvNV( index: GLuint; n: GLsizei; v: PGLfloat ); external name '_glVertexAttribs2fvNV';
procedure glVertexAttribs2dvNV( index: GLuint; n: GLsizei; v: PGLdouble ); external name '_glVertexAttribs2dvNV';
procedure glVertexAttribs3svNV( index: GLuint; n: GLsizei; v: PGLshort ); external name '_glVertexAttribs3svNV';
procedure glVertexAttribs3fvNV( index: GLuint; n: GLsizei; v: PGLfloat ); external name '_glVertexAttribs3fvNV';
procedure glVertexAttribs3dvNV( index: GLuint; n: GLsizei; v: PGLdouble ); external name '_glVertexAttribs3dvNV';
procedure glVertexAttribs4svNV( index: GLuint; n: GLsizei; v: PGLshort ); external name '_glVertexAttribs4svNV';
procedure glVertexAttribs4fvNV( index: GLuint; n: GLsizei; v: PGLfloat ); external name '_glVertexAttribs4fvNV';
procedure glVertexAttribs4dvNV( index: GLuint; n: GLsizei; v: PGLdouble ); external name '_glVertexAttribs4dvNV';
procedure glVertexAttribs4ubvNV( index: GLuint; n: GLsizei; v: PGLubyte ); external name '_glVertexAttribs4ubvNV';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_NV_point_sprite and GL_NV_point_sprite}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPointParameteriNVProcPtr = procedure( pname: GLenum; param: GLint );
	glPointParameterivNVProcPtr = procedure( pname: GLenum; const params: PGLint );
{$elsec}
procedure glPointParameteriNV( pname: GLenum; param: GLint ); external name '_glPointParameteriNV';
procedure glPointParameterivNV( pname: GLenum; const params: PGLint ); external name '_glPointParameterivNV';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_PGI_misc_hints and GL_PGI_misc_hints}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glHintPGIProcPtr = procedure( param1 : GLenum; param2 : GLint );
{$elsec}
procedure glHintPGI( param1 : GLenum; param2 : GLint ); external name '_glHintPGI';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_SGI_color_table and GL_SGI_color_table}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glColorTableSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr );
	glColorTableParameterfvSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
	glColorTableParameterivSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLint );
	glCopyColorTableSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint; param4 : GLint; param5 : GLsizei );
	glGetColorTableSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : UnivPtr );
	glGetColorTableParameterfvSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetColorTableParameterivSGIProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
{$elsec}
procedure glColorTableSGI( param1 : GLenum; param2 : GLenum; param3 : GLsizei; param4 : GLenum; param5 : GLenum; const param6 : UnivPtr ); external name '_glColorTableSGI';
procedure glColorTableParameterfvSGI( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glColorTableParameterfvSGI';
procedure glColorTableParameterivSGI( param1 : GLenum; param2 : GLenum; const param3 : PGLint ); external name '_glColorTableParameterivSGI';
procedure glCopyColorTableSGI( param1 : GLenum; param2 : GLenum; param3 : GLint; param4 : GLint; param5 : GLsizei ); external name '_glCopyColorTableSGI';
procedure glGetColorTableSGI( param1 : GLenum; param2 : GLenum; param3 : GLenum; param4 : UnivPtr ); external name '_glGetColorTableSGI';
procedure glGetColorTableParameterfvSGI( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetColorTableParameterfvSGI';
procedure glGetColorTableParameterivSGI( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetColorTableParameterivSGI';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_texture_filter4 and GL_SGIS_texture_filter4}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGetTexFilterFuncSGISProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glTexFilterFuncSGISProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLsizei; const param4 : PGLfloat );
{$elsec}
procedure glGetTexFilterFuncSGIS( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetTexFilterFuncSGIS';
procedure glTexFilterFuncSGIS( param1 : GLenum; param2 : GLenum; param3 : GLsizei; const param4 : PGLfloat ); external name '_glTexFilterFuncSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_pixel_texture and GL_SGIS_pixel_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPixelTexGenParameteriSGISProcPtr = procedure( param1 : GLenum; param2 : GLint );
	glPixelTexGenParameterivSGISProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
	glPixelTexGenParameterfSGISProcPtr = procedure( param1 : GLenum; param2 : GLfloat );
	glPixelTexGenParameterfvSGISProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glGetPixelTexGenParameterivSGISProcPtr = procedure( param1 : GLenum; param2 : PGLint );
	glGetPixelTexGenParameterfvSGISProcPtr = procedure( param1 : GLenum; param2 : PGLfloat );
{$elsec}
procedure glPixelTexGenParameteriSGIS( param1 : GLenum; param2 : GLint ); external name '_glPixelTexGenParameteriSGIS';
procedure glPixelTexGenParameterivSGIS( param1 : GLenum; const param2 : PGLint ); external name '_glPixelTexGenParameterivSGIS';
procedure glPixelTexGenParameterfSGIS( param1 : GLenum; param2 : GLfloat ); external name '_glPixelTexGenParameterfSGIS';
procedure glPixelTexGenParameterfvSGIS( param1 : GLenum; const param2 : PGLfloat ); external name '_glPixelTexGenParameterfvSGIS';
procedure glGetPixelTexGenParameterivSGIS( param1 : GLenum; param2 : PGLint ); external name '_glGetPixelTexGenParameterivSGIS';
procedure glGetPixelTexGenParameterfvSGIS( param1 : GLenum; param2 : PGLfloat ); external name '_glGetPixelTexGenParameterfvSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_texture4D and GL_SGIS_texture4D}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTexImage4DSGISProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLsizei; param7 : GLsizei; param8 : GLint; param9 : GLenum; param10 : GLenum; const param11 : UnivPtr );
	glTexSubImage4DSGISProcPtr = procedure( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLint; param7 : GLsizei; param8 : GLsizei; param9 : GLsizei; param10 : GLsizei; param11 : GLenum; param12 : GLenum; const param13 : UnivPtr );
{$elsec}
procedure glTexImage4DSGIS( param1 : GLenum; param2 : GLint; param3 : GLenum; param4 : GLsizei; param5 : GLsizei; param6 : GLsizei; param7 : GLsizei; param8 : GLint; param9 : GLenum; param10 : GLenum; const param11 : UnivPtr ); external name '_glTexImage4DSGIS';
procedure glTexSubImage4DSGIS( param1 : GLenum; param2 : GLint; param3 : GLint; param4 : GLint; param5 : GLint; param6 : GLint; param7 : GLsizei; param8 : GLsizei; param9 : GLsizei; param10 : GLsizei; param11 : GLenum; param12 : GLenum; const param13 : UnivPtr ); external name '_glTexSubImage4DSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_detail_texture and GL_SGIS_detail_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glDetailTexFuncSGISProcPtr = procedure( param1 : GLenum; param2 : GLsizei; const param3 : PGLfloat );
	glGetDetailTexFuncSGISProcPtr = procedure( param1 : GLenum; param2 : PGLfloat );
{$elsec}
procedure glDetailTexFuncSGIS( param1 : GLenum; param2 : GLsizei; const param3 : PGLfloat ); external name '_glDetailTexFuncSGIS';
procedure glGetDetailTexFuncSGIS( param1 : GLenum; param2 : PGLfloat ); external name '_glGetDetailTexFuncSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_sharpen_texture and GL_SGIS_sharpen_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glSharpenTexFuncSGISProcPtr = procedure( param1 : GLenum; param2 : GLsizei; const param3 : PGLfloat );
	glGetSharpenTexFuncSGISProcPtr = procedure( param1 : GLenum; param2 : PGLfloat );
{$elsec}
procedure glSharpenTexFuncSGIS( param1 : GLenum; param2 : GLsizei; const param3 : PGLfloat ); external name '_glSharpenTexFuncSGIS';
procedure glGetSharpenTexFuncSGIS( param1 : GLenum; param2 : PGLfloat ); external name '_glGetSharpenTexFuncSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_multisample and GL_SGIS_multisample}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glSampleMaskSGISProcPtr = procedure( Gparam1 : Lclampf; param2 : GLboolean );
	glSamplePatternSGISProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glSampleMaskSGIS( param1 : GLclampf; param2 : GLboolean ); external name '_glSampleMaskSGIS';
procedure glSamplePatternSGIS( param1 : GLenum ); external name '_glSamplePatternSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_fog_function and GL_SGIS_fog_function}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFogFuncSGISProcPtr = procedure( param1 : GLsizei; const param2 : PGLfloat );
	glGetFogFuncSGISProcPtr = procedure( const param1 : PGLfloat );
{$elsec}
procedure glFogFuncSGIS( param1 : GLsizei; const param2 : GLfloat ); external name '_glFogFuncSGIS';
procedure glGetFogFuncSGIS( const param1 : PGLfloat ); external name '_glGetFogFuncSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIS_texture_color_mask and GL_SGIS_texture_color_mask}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTextureColorMaskSGISProcPtr = procedure( param1 : GLboolean; param2 : GLboolean; param3 : GLboolean; param4 : GLboolean );
{$elsec}
procedure glTextureColorMaskSGIS( param1 : GLboolean; param2 : GLboolean; param3 : GLboolean; param4 : GLboolean ); external name '_glTextureColorMaskSGIS';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_pixel_texture and GL_SGIX_pixel_texture}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glPixelTexGenSGIXProcPtr = procedure( param1 : GLenum );
{$elsec}
procedure glPixelTexGenSGIX( param1 : GLenum ); external name '_glPixelTexGenSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_sprite and GL_SGIX_sprite}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glSpriteParameterfSGIXProcPtr = procedure( param1 : GLenum; param1 : GLfloat );
	glSpriteParameterfvSGIXProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glSpriteParameteriSGIXProcPtr = procedure( param1 : GLenum; param2 : GLint );
	glSpriteParameterivSGIXProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
{$elsec}
procedure glSpriteParameterfSGIX( param1 : GLenum; param1 : GLfloat ); external name '_glSpriteParameterfSGIX';
procedure glSpriteParameterfvSGIX( param1 : GLenum; const param2 : PGLfloat ); external name '_glSpriteParameterfvSGIX';
procedure glSpriteParameteriSGIX( param1 : GLenum; param2 : GLint ); external name '_glSpriteParameteriSGIX';
procedure glSpriteParameterivSGIX( param1 : GLenum; const param2 : PGLint ); external name '_glSpriteParameterivSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_instruments and GL_SGIX_instruments}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGetInstrumentsSGIXProcPtr = function: GLint;
	glInstrumentsBufferSGIXProcPtr = procedure( param1 : GLsizei; param2 : PGLint );
	glPollInstrumentsSGIXProcPtr = function( param1 : PGLint ): GLint;
	glReadInstrumentsSGIXProcPtr = procedure( param1 : GLint );
	glStartInstrumentsSGIXProcPtr = procedure;
	glStopInstrumentsSGIXProcPtr = procedure( param1 : GLint );
{$elsec}
function glGetInstrumentsSGIX: GLint; external name '_glGetInstrumentsSGIX';
procedure glInstrumentsBufferSGIX( param1 : GLsizei; param2 : PGLint ); external name '_glInstrumentsBufferSGIX';
function glPollInstrumentsSGIX( param1 : PGLint ): GLint; external name '_glPollInstrumentsSGIX';
procedure glReadInstrumentsSGIX( param1 : GLint ); external name '_glReadInstrumentsSGIX';
procedure glStartInstrumentsSGIX; external name '_glStartInstrumentsSGIX';
procedure glStopInstrumentsSGIX( param1 : GLint ); external name '_glStopInstrumentsSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_framezoom and GL_SGIX_framezoom}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFrameZoomSGIXProcPtr = procedure( param1 : GLint );
{$elsec}
procedure glFrameZoomSGIX( param1 : GLint ); external name '_glFrameZoomSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_tag_sample_buffer and GL_SGIX_tag_sample_buffer}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTagSampleBufferSGIXProcPtr = procedure;
{$elsec}
procedure glTagSampleBufferSGIX; external name '_glTagSampleBufferSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_polynomial_ffd and GL_SGIX_polynomial_ffd}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glDeformationMap3dSGIXProcPtr = procedure( param1 : GLenum; param2 : GLdouble; param3 : GLdouble; param4 : GLint; param5 : GLint; param6 : GLdouble; param7 : GLdouble; param8 : GLint; param9 : GLint; param10 : GLdouble; param11 : GLdouble; param12 : GLint; param13 : GLint; const param14 : PGLdouble );
	glDeformationMap3fSGIXProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLint; param5 : GLint; param6 : GLfloat; param7 : GLfloat; param8 : GLint; param9 : GLint; param10 : GLfloat; param11 : GLfloat; param12 : GLint; param13 : GLint; const param14 : PGLfloat );
	glDeformSGIXProcPtr = procedure( param1 : GLbitfield );
	glLoadIdentityDeformationMapSGIXProcPtr = procedure( param1 : GLbitfield );
{$elsec}
procedure glDeformationMap3dSGIX( param1 : GLenum; param2 : GLdouble; param3 : GLdouble; param4 : GLint; param5 : GLint; param6 : GLdouble; param7 : GLdouble; param8 : GLint; param9 : GLint; param10 : GLdouble; param11 : GLdouble; param12 : GLint; param13 : GLint; const param14 : PGLdouble ); external name '_glDeformationMap3dSGIX';
procedure glDeformationMap3fSGIX( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLint; param5 : GLint; param6 : GLfloat; param7 : GLfloat; param8 : GLint; param9 : GLint; param10 : GLfloat; param11 : GLfloat; param12 : GLint; param13 : GLint; const param14 : PGLfloat ); external name '_glDeformationMap3fSGIX';
procedure glDeformSGIX( param1 : GLbitfield ); external name '_glDeformSGIX';
procedure glLoadIdentityDeformationMapSGIX( param1 : GLbitfield ); external name '_glLoadIdentityDeformationMapSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_reference_plane and GL_SGIX_reference_plane}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glReferencePlaneSGIXProcPtr = procedure( const param1 : PGLdouble );
{$elsec}
procedure glReferencePlaneSGIX( const param1 : PGLdouble ); external name '_glReferencePlaneSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_flush_raster and GL_SGIX_flush_raster}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFlushRasterSGIXProcPtr = procedure;
{$elsec}
procedure glFlushRasterSGIX; external name '_glFlushRasterSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_list_priority and GL_SGIX_list_priority}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGetListParameterfvSGIXProcPtr = procedure( param1 : GLuint; param2 : GLenum; param3 : PGLfloat );
	glGetListParameterivSGIXProcPtr = procedure( param1 : GLuint; param2 : GLenum; param1 : PGLint );
	glListParameterfSGIXProcPtr = procedure( param1 : GLuint; param2 : GLenum; param3 : GLfloat );
	glListParameterfvSGIXProcPtr = procedure( param1 : GLuint; param2 : GLenum; const param3 : PGLfloat );
	glListParameteriSGIXProcPtr = procedure( param1 : GLuint; param2 : GLenum; param3 : GLint );
	glListParameterivSGIXProcPtr = procedure( param1 : GLuint; param2 : GLenum; const param3 : PGLint );
{$elsec}
procedure glGetListParameterfvSGIX( param1 : GLuint; param2 : GLenum; param3 : PGLfloat ); external name '_glGetListParameterfvSGIX';
procedure glGetListParameterivSGIX( param1 : GLuint; param2 : GLenum; param1 : PGLint ); external name '_glGetListParameterivSGIX';
procedure glListParameterfSGIX( param1 : GLuint; param2 : GLenum; param3 : GLfloat ); external name '_glListParameterfSGIX';
procedure glListParameterfvSGIX( param1 : GLuint; param2 : GLenum; const param3 : PGLfloat ); external name '_glListParameterfvSGIX';
procedure glListParameteriSGIX( param1 : GLuint; param2 : GLenum; param3 : GLint ); external name '_glListParameteriSGIX';
procedure glListParameterivSGIX( param1 : GLuint; param2 : GLenum; const param3 : PGLint ); external name '_glListParameterivSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_fragment_lighting and GL_SGIX_fragment_lighting}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFragmentColorMaterialSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum );
	glFragmentLightfSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLfloat );
	glFragmentLightfvSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
	glFragmentLightiSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint );
	glFragmentLightivSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLint );
	glFragmentLightModelfSGIXProcPtr = procedure( param1 : GLenum; param2 : GLfloat );
	glFragmentLightModelfvSGIXProcPtr = procedure( param1 : GLenum; const param2 : PGLfloat );
	glFragmentLightModeliSGIXProcPtr = procedure( param1 : GLenum; param2 : GLint );
	glFragmentLightModelivSGIXProcPtr = procedure( param1 : GLenum; const param2 : PGLint );
	glFragmentMaterialfSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLfloat );
	glFragmentMaterialfvSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat );
	glFragmentMaterialiSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : GLint );
	glFragmentMaterialivSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; const param3 : PGLint );
	glGetFragmentLightfvSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetFragmentLightivSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glGetFragmentMaterialfvSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLfloat );
	glGetFragmentMaterialivSGIXProcPtr = procedure( param1 : GLenum; param2 : GLenum; param3 : PGLint );
	glLightEnviSGIXProcPtr = procedure( param1 : GLenum; param2 : GLint );
{$elsec}
procedure glFragmentColorMaterialSGIX( param1 : GLenum; param2 : GLenum ); external name '_glFragmentColorMaterialSGIX';
procedure glFragmentLightfSGIX( param1 : GLenum; param2 : GLenum; param3 : GLfloat ); external name '_glFragmentLightfSGIX';
procedure glFragmentLightfvSGIX( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glFragmentLightfvSGIX';
procedure glFragmentLightiSGIX( param1 : GLenum; param2 : GLenum; param3 : GLint ); external name '_glFragmentLightiSGIX';
procedure glFragmentLightivSGIX( param1 : GLenum; param2 : GLenum; const param3 : PGLint ); external name '_glFragmentLightivSGIX';
procedure glFragmentLightModelfSGIX( param1 : GLenum; param2 : GLfloat ); external name '_glFragmentLightModelfSGIX';
procedure glFragmentLightModelfvSGIX( param1 : GLenum; const param2 : PGLfloat ); external name '_glFragmentLightModelfvSGIX';
procedure glFragmentLightModeliSGIX( param1 : GLenum; param2 : GLint ); external name '_glFragmentLightModeliSGIX';
procedure glFragmentLightModelivSGIX( param1 : GLenum; const param2 : PGLint ); external name '_glFragmentLightModelivSGIX';
procedure glFragmentMaterialfSGIX( param1 : GLenum; param2 : GLenum; param3 : GLfloat ); external name '_glFragmentMaterialfSGIX';
procedure glFragmentMaterialfvSGIX( param1 : GLenum; param2 : GLenum; const param3 : PGLfloat ); external name '_glFragmentMaterialfvSGIX';
procedure glFragmentMaterialiSGIX( param1 : GLenum; param2 : GLenum; param3 : GLint ); external name '_glFragmentMaterialiSGIX';
procedure glFragmentMaterialivSGIX( param1 : GLenum; param2 : GLenum; const param3 : PGLint ); external name '_glFragmentMaterialivSGIX';
procedure glGetFragmentLightfvSGIX( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetFragmentLightfvSGIX';
procedure glGetFragmentLightivSGIX( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetFragmentLightivSGIX';
procedure glGetFragmentMaterialfvSGIX( param1 : GLenum; param2 : GLenum; param3 : PGLfloat ); external name '_glGetFragmentMaterialfvSGIX';
procedure glGetFragmentMaterialivSGIX( param1 : GLenum; param2 : GLenum; param3 : PGLint ); external name '_glGetFragmentMaterialivSGIX';
procedure glLightEnviSGIX( param1 : GLenum; param2 : GLint ); external name '_glLightEnviSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_async and GL_SGIX_async}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glAsyncMarkerSGIXProcPtr = procedure( param1 : GLuint );
	glFinishAsyncSGIXProcPtr = function( param1 : PGLuint ): GLint;
	glPollAsyncSGIXProcPtr = function( param1 : PGLuint ): GLint;
	glGenAsyncMarkersSGIXProcPtr = function( param1 : GLsizei ): GLuint;
	glDeleteAsyncMarkersSGIXProcPtr = procedure( param1 : GLuint; param2 : GLsizei );
	glIsAsyncMarkerSGIXProcPtr = function( param1 : GLuint ): GLboolean;
{$elsec}
procedure glAsyncMarkerSGIX( param1 : GLuint ); external name '_glAsyncMarkerSGIX';
function glFinishAsyncSGIX( param1 : PGLuint ): GLint; external name '_glFinishAsyncSGIX';
function glPollAsyncSGIX( param1 : PGLuint ): GLint; external name '_glPollAsyncSGIX';
function glGenAsyncMarkersSGIX( param1 : GLsizei ): GLuint; external name '_glGenAsyncMarkersSGIX';
procedure glDeleteAsyncMarkersSGIX( param1 : GLuint; param2 : GLsizei ); external name '_glDeleteAsyncMarkersSGIX';
function glIsAsyncMarkerSGIX( param1 : GLuint ): GLboolean; external name '_glIsAsyncMarkerSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SGIX_igloo_interface and GL_SGIX_igloo_interface}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glIglooInterfaceSGIXProcPtr = procedure( param1 : GLenum; const param2 : UnivPtr );
{$elsec}
procedure glIglooInterfaceSGIX( param1 : GLenum; const param2 : UnivPtr ); external name '_glIglooInterfaceSGIX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_SUN_global_alpha and GL_SUN_global_alpha}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glGlobalAlphaFactorbSUNProcPtr = procedure( param1 : GLbyte );
	glGlobalAlphaFactorsSUNProcPtr = procedure( param1 : GLshort );
	glGlobalAlphaFactoriSUNProcPtr = procedure( param1 : GLint );
	glGlobalAlphaFactorfSUNProcPtr = procedure( param1 : GLfloat );
	glGlobalAlphaFactordSUNProcPtr = procedure( param1 : GLdouble );
	glGlobalAlphaFactorubSUNProcPtr = procedure( param1 : GLubyte );
	glGlobalAlphaFactorusSUNProcPtr = procedure( param1 : GLushort );
	glGlobalAlphaFactoruiSUNProcPtr = procedure( param1 : GLuint );
{$elsec}
procedure glGlobalAlphaFactorbSUN( param1 : GLbyte ); external name '_glGlobalAlphaFactorbSUN';
procedure glGlobalAlphaFactorsSUN( param1 : GLshort ); external name '_glGlobalAlphaFactorsSUN';
procedure glGlobalAlphaFactoriSUN( param1 : GLint ); external name '_glGlobalAlphaFactoriSUN';
procedure glGlobalAlphaFactorfSUN( param1 : GLfloat ); external name '_glGlobalAlphaFactorfSUN';
procedure glGlobalAlphaFactordSUN( param1 : GLdouble ); external name '_glGlobalAlphaFactordSUN';
procedure glGlobalAlphaFactorubSUN( param1 : GLubyte ); external name '_glGlobalAlphaFactorubSUN';
procedure glGlobalAlphaFactorusSUN( param1 : GLushort ); external name '_glGlobalAlphaFactorusSUN';
procedure glGlobalAlphaFactoruiSUN( param1 : GLuint ); external name '_glGlobalAlphaFactoruiSUN';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SUN_triangle_list and GL_SUN_triangle_list}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glReplacementCodeuiSUNProcPtr = procedure( param1 : GLuint );
	glReplacementCodeusSUNProcPtr = procedure( param1 : GLushort );
	glReplacementCodeubSUNProcPtr = procedure( param1 : GLubyte );
	glReplacementCodeuivSUNProcPtr = procedure( const param1 : PGLuint );
	glReplacementCodeusvSUNProcPtr = procedure( const param1 : PGLushort );
	glReplacementCodeubvSUNProcPtr = procedure( const param1 : PGLubyte );
	glReplacementCodePointerSUNProcPtr = procedure( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtrPtr );
{$elsec}
procedure glReplacementCodeuiSUN( param1 : GLuint ); external name '_glReplacementCodeuiSUN';
procedure glReplacementCodeusSUN( param1 : GLushort ); external name '_glReplacementCodeusSUN';
procedure glReplacementCodeubSUN( param1 : GLubyte ); external name '_glReplacementCodeubSUN';
procedure glReplacementCodeuivSUN( const param1 : PGLuint ); external name '_glReplacementCodeuivSUN';
procedure glReplacementCodeusvSUN( const param1 : PGLushort ); external name '_glReplacementCodeusvSUN';
procedure glReplacementCodeubvSUN( const param1 : PGLubyte ); external name '_glReplacementCodeubvSUN';
procedure glReplacementCodePointerSUN( param1 : GLenum; param2 : GLsizei; const param3 : UnivPtrPtr ); external name '_glReplacementCodePointerSUN';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SUN_vertex and GL_SUN_vertex}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glColor4ubVertex2fSUNProcPtr = procedure( param1 : GLubyte; param2 : GLubyte; param3 : GLubyte; param4 : GLubyte; param5 : GLfloat; param6 : GLfloat );
	glColor4ubVertex2fvSUNProcPtr = procedure( const param1 : PGLubyte; const param2 : PGLfloat );
	glColor4ubVertex3fSUNProcPtr = procedure( param1 : GLubyte; param2 : GLubyte; param3 : GLubyte; param4 : GLubyte; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat );
	glColor4ubVertex3fvSUNProcPtr = procedure( const param1 : PGLubyte; const param2 : PGLfloat );
	glColor3fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat );
	glColor3fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat );
	glNormal3fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat );
	glNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat );
	glColor4fNormal3fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat );
	glColor4fNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat );
	glTexCoord2fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat );
	glTexCoord2fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat );
	glTexCoord4fVertex4fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat );
	glTexCoord4fVertex4fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat );
	glTexCoord2fColor4ubVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLubyte; param4 : GLubyte; param5 : GLubyte; param6 : GLubyte; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat );
	glTexCoord2fColor4ubVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLubyte; const param3 : PGLfloat );
	glTexCoord2fColor3fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat );
	glTexCoord2fColor3fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat );
	glTexCoord2fNormal3fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat );
	glTexCoord2fNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat );
	glTexCoord2fColor4fNormal3fVertex3fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat; param12 : GLfloat );
	glTexCoord2fColor4fNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat );
	glTexCoord4fColor4fNormal3fVertex4fSUNProcPtr = procedure( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat; param12 : GLfloat; param13 : GLfloat; param14 : GLfloat; param15 : GLfloat );
	glTexCoord4fColor4fNormal3fVertex4fvSUNProcPtr = procedure( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat );
	glReplacementCodeuiVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat );
	glReplacementCodeuiVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat );
	glReplacementCodeuiColor4ubVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLubyte; param3 : GLubyte; param4 : GLubyte; param5 : GLubyte; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat );
	glReplacementCodeuiColor4ubVertex3fvSUNProcPtr = procedure( param1 : const PGLenum; const param2 : PGLubyte; const param3 : PGLfloat );
	glReplacementCodeuiColor3fVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat );
	glReplacementCodeuiColor3fVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat );
	glReplacementCodeuiNormal3fVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat );
	glReplacementCodeuiNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat );
	glReplacementCodeuiColor4fNormal3fVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat );
	glReplacementCodeuiColor4fNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat );
	glReplacementCodeuiTexCoord2fVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat );
	glReplacementCodeuiTexCoord2fVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat );
	glReplacementCodeuiTexCoord2fNormal3fVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat );
	glReplacementCodeuiTexCoord2fNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat );
	glReplacementCodeuiTexCoord2fColor4fNormal3fVertex3fSUNProcPtr = procedure( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat; param12 : GLfloat; param13 : GLfloat );
	glReplacementCodeuiTexCoord2fColor4fNormal3fVertex3fvSUNProcPtr = procedure( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat; const param5 : PGLfloat );
{$elsec}
procedure glColor4ubVertex2fSUN( param1 : GLubyte; param2 : GLubyte; param3 : GLubyte; param4 : GLubyte; param5 : GLfloat; param6 : GLfloat ); external name '_glColor4ubVertex2fSUN';
procedure glColor4ubVertex2fvSUN( const param1 : PGLubyte; const param2 : PGLfloat ); external name '_glColor4ubVertex2fvSUN';
procedure glColor4ubVertex3fSUN( param1 : GLubyte; param2 : GLubyte; param3 : GLubyte; param4 : GLubyte; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat ); external name '_glColor4ubVertex3fSUN';
procedure glColor4ubVertex3fvSUN( const param1 : PGLubyte; const param2 : PGLfloat ); external name '_glColor4ubVertex3fvSUN';
procedure glColor3fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat ); external name '_glColor3fVertex3fSUN';
procedure glColor3fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat ); external name '_glColor3fVertex3fvSUN';
procedure glNormal3fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat ); external name '_glNormal3fVertex3fSUN';
procedure glNormal3fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat ); external name '_glNormal3fVertex3fvSUN';
procedure glColor4fNormal3fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat ); external name '_glColor4fNormal3fVertex3fSUN';
procedure glColor4fNormal3fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat ); external name '_glColor4fNormal3fVertex3fvSUN';
procedure glTexCoord2fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat ); external name '_glTexCoord2fVertex3fSUN';
procedure glTexCoord2fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat ); external name '_glTexCoord2fVertex3fvSUN';
procedure glTexCoord4fVertex4fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat ); external name '_glTexCoord4fVertex4fSUN';
procedure glTexCoord4fVertex4fvSUN( const param1 : PGLfloat; const param2 : PGLfloat ); external name '_glTexCoord4fVertex4fvSUN';
procedure glTexCoord2fColor4ubVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLubyte; param4 : GLubyte; param5 : GLubyte; param6 : GLubyte; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat ); external name '_glTexCoord2fColor4ubVertex3fSUN';
procedure glTexCoord2fColor4ubVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLubyte; const param3 : PGLfloat ); external name '_glTexCoord2fColor4ubVertex3fvSUN';
procedure glTexCoord2fColor3fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat ); external name '_glTexCoord2fColor3fVertex3fSUN';
procedure glTexCoord2fColor3fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat ); external name '_glTexCoord2fColor3fVertex3fvSUN';
procedure glTexCoord2fNormal3fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat ); external name '_glTexCoord2fNormal3fVertex3fSUN';
procedure glTexCoord2fNormal3fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat ); external name '_glTexCoord2fNormal3fVertex3fvSUN';
procedure glTexCoord2fColor4fNormal3fVertex3fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat; param12 : GLfloat ); external name '_glTexCoord2fColor4fNormal3fVertex3fSUN';
procedure glTexCoord2fColor4fNormal3fVertex3fvSUN( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat ); external name '_glTexCoord2fColor4fNormal3fVertex3fvSUN';
procedure glTexCoord4fColor4fNormal3fVertex4fSUN( param1 : GLfloat; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat; param12 : GLfloat; param13 : GLfloat; param14 : GLfloat; param15 : GLfloat ); external name '_glTexCoord4fColor4fNormal3fVertex4fSUN';
procedure glTexCoord4fColor4fNormal3fVertex4fvSUN( const param1 : PGLfloat; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat ); external name '_glTexCoord4fColor4fNormal3fVertex4fvSUN';
procedure glReplacementCodeuiVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat ); external name '_glReplacementCodeuiVertex3fSUN';
procedure glReplacementCodeuiVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat ); external name '_glReplacementCodeuiVertex3fvSUN';
procedure glReplacementCodeuiColor4ubVertex3fSUN( param1 : GLenum; param2 : GLubyte; param3 : GLubyte; param4 : GLubyte; param5 : GLubyte; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat ); external name '_glReplacementCodeuiColor4ubVertex3fSUN';
procedure glReplacementCodeuiColor4ubVertex3fvSUN( param1 : const PGLenum; const param2 : PGLubyte; const param3 : PGLfloat ); external name '_glReplacementCodeuiColor4ubVertex3fvSUN';
procedure glReplacementCodeuiColor3fVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat ); external name '_glReplacementCodeuiColor3fVertex3fSUN';
procedure glReplacementCodeuiColor3fVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat ); external name '_glReplacementCodeuiColor3fVertex3fvSUN';
procedure glReplacementCodeuiNormal3fVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat ); external name '_glReplacementCodeuiNormal3fVertex3fSUN';
procedure glReplacementCodeuiNormal3fVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat ); external name '_glReplacementCodeuiNormal3fVertex3fvSUN';
procedure glReplacementCodeuiColor4fNormal3fVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat ); external name '_glReplacementCodeuiColor4fNormal3fVertex3fSUN';
procedure glReplacementCodeuiColor4fNormal3fVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat ); external name '_glReplacementCodeuiColor4fNormal3fVertex3fvSUN';
procedure glReplacementCodeuiTexCoord2fVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat ); external name '_glReplacementCodeuiTexCoord2fVertex3fSUN';
procedure glReplacementCodeuiTexCoord2fVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat ); external name '_glReplacementCodeuiTexCoord2fVertex3fvSUN';
procedure glReplacementCodeuiTexCoord2fNormal3fVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat ); external name '_glReplacementCodeuiTexCoord2fNormal3fVertex3fSUN';
procedure glReplacementCodeuiTexCoord2fNormal3fVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat ); external name '_glReplacementCodeuiTexCoord2fNormal3fVertex3fvSUN';
procedure glReplacementCodeuiTexCoord2fColor4fNormal3fVertex3fSUN( param1 : GLenum; param2 : GLfloat; param3 : GLfloat; param4 : GLfloat; param5 : GLfloat; param6 : GLfloat; param7 : GLfloat; param8 : GLfloat; param9 : GLfloat; param10 : GLfloat; param11 : GLfloat; param12 : GLfloat; param13 : GLfloat ); external name '_glReplacementCodeuiTexCoord2fColor4fNormal3fVertex3fSUN';
procedure glReplacementCodeuiTexCoord2fColor4fNormal3fVertex3fvSUN( const param1 : PGLenum; const param2 : PGLfloat; const param3 : PGLfloat; const param4 : PGLfloat; const param5 : PGLfloat ); external name '_glReplacementCodeuiTexCoord2fColor4fNormal3fVertex3fvSUN';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}

{$ifc not undefined GL_SUNX_constant_data and GL_SUNX_constant_data}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glFinishTextureSUNXProcPtr = procedure;
{$elsec}
procedure glFinishTextureSUNX; external name '_glFinishTextureSUNX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_3DFX_tbuffer and GL_3DFX_tbuffer}
{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glTbufferMask3DFXProcPtr = procedure( param1 : GLuint );
{$elsec}
procedure glTbufferMask3DFX( param1 : GLuint ); external name '_glTbufferMask3DFX';
{$endc} { GL_GLEXT_FUNCTION_POINTERS }
{$endc}


{$ifc not undefined GL_GLEXT_FUNCTION_POINTERS and GL_GLEXT_FUNCTION_POINTERS}
type
	glBeginConditionalRenderNVProcPtr = procedure( id_: GLuint; mode: GLenum );
	glEndConditionalRenderNVProcPtr = procedure;
{$elsec}
procedure glBeginConditionalRenderNV( id_: GLuint; mode: GLenum ); external name '_glBeginConditionalRenderNV';
procedure glEndConditionalRenderNV; external name '_glEndConditionalRenderNV';
{$endc}


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
