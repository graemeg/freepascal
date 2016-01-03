<<<<<<< HEAD
<<<<<<< HEAD
{

  Adaption of the delphi3d.net OpenGL units to FreePascal
  Sebastian Guenther (sg@freepascal.org) in 2002
  These units are free to use
=======
=======
>>>>>>> origin/fixes_2_2
{ Adaption of the delphi3d.net OpenGL units to FreePascal
  Sebastian Guenther (sg@freepascal.org) in 2002
  These units are free to use

  19.6.07 : Added GLUT_EXCLUSIVE_FPUMODE to allow for
  unsafe glut-libs, that don't handle FPU-exceptions in
  a compatible way.  Jan Bruns (post@abnuto.de)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
}

{$MACRO ON}
{$MODE Delphi}
{$IFDEF Windows}
  {$DEFINE extdecl := stdcall}
{$ELSE}
  {$DEFINE extdecl := cdecl}
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

{$IFDEF CPU86}
  {$DEFINE GLUT_EXCLUSIVE_FPUMODE}
  {$DEFINE mode_inline := register} //inine or local-calling
{$ENDIF}


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$IFDEF MORPHOS}
{$INLINE ON}
{$DEFINE GLUT_UNIT}
{$ENDIF}

unit Glut;

// Copyright (c) Mark J. Kilgard, 1994, 1995, 1996. */

(* This program is freely distributable without licensing fees  and is
   provided without guarantee or warrantee expressed or  implied. This
   program is -not- in the public domain. *)

{******************************************************************************}
{ Converted to Delphi by Tom Nuydens (tom@delphi3d.net)                        }
{   Contributions by Igor Karpov (glygrik@hotbox.ru)                           }
{   For the latest updates, visit Delphi3D: http://www.delphi3d.net            }
{******************************************************************************}

interface

uses
  SysUtils,
  {$IFDEF Windows}
  Windows, dynlibs,
  {$ELSE}
  {$IFDEF MORPHOS}
  TinyGL,
  {$ELSE}
  dynlibs,
  {$ENDIF}
  {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
  GL;

type
=======
=======
>>>>>>> origin/fixes_2_2
  {$IFDEF GLUT_EXCLUSIVE_FPUMODE}
  math,
  {$ENDIF}
  GL;

type
  PInteger = ^Integer;
  PPChar = ^PChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  TGlutVoidCallback = procedure; cdecl;
  TGlut1IntCallback = procedure(value: Integer); cdecl;
  TGlut2IntCallback = procedure(v1, v2: Integer); cdecl;
  TGlut3IntCallback = procedure(v1, v2, v3: Integer); cdecl;
  TGlut4IntCallback = procedure(v1, v2, v3, v4: Integer); cdecl;
  TGlut1Char2IntCallback = procedure(c: Byte; v1, v2: Integer); cdecl;
  TGlut1UInt3IntCallback = procedure(u: Cardinal; v1, v2, v3: Integer); cdecl;

const
  GLUT_API_VERSION                = 3;
  GLUT_XLIB_IMPLEMENTATION        = 12;
  // Display mode bit masks.
  GLUT_RGB                        = 0;
  GLUT_RGBA                       = GLUT_RGB;
  GLUT_INDEX                      = 1;
  GLUT_SINGLE                     = 0;
  GLUT_DOUBLE                     = 2;
  GLUT_ACCUM                      = 4;
  GLUT_ALPHA                      = 8;
  GLUT_DEPTH                      = 16;
  GLUT_STENCIL                    = 32;
  GLUT_MULTISAMPLE                = 128;
  GLUT_STEREO                     = 256;
  GLUT_LUMINANCE                  = 512;

  // Mouse buttons.
  GLUT_LEFT_BUTTON                = 0;
  GLUT_MIDDLE_BUTTON              = 1;
  GLUT_RIGHT_BUTTON               = 2;

  // Mouse button state.
  GLUT_DOWN                       = 0;
  GLUT_UP                         = 1;

  // function keys
  GLUT_KEY_F1                     = 1;
  GLUT_KEY_F2                     = 2;
  GLUT_KEY_F3                     = 3;
  GLUT_KEY_F4                     = 4;
  GLUT_KEY_F5                     = 5;
  GLUT_KEY_F6                     = 6;
  GLUT_KEY_F7                     = 7;
  GLUT_KEY_F8                     = 8;
  GLUT_KEY_F9                     = 9;
  GLUT_KEY_F10                    = 10;
  GLUT_KEY_F11                    = 11;
  GLUT_KEY_F12                    = 12;
  // directional keys
  GLUT_KEY_LEFT                   = 100;
  GLUT_KEY_UP                     = 101;
  GLUT_KEY_RIGHT                  = 102;
  GLUT_KEY_DOWN                   = 103;
  GLUT_KEY_PAGE_UP                = 104;
  GLUT_KEY_PAGE_DOWN              = 105;
  GLUT_KEY_HOME                   = 106;
  GLUT_KEY_END                    = 107;
  GLUT_KEY_INSERT                 = 108;

  // Entry/exit  state.
  GLUT_LEFT                       = 0;
  GLUT_ENTERED                    = 1;

  // Menu usage state.
  GLUT_MENU_NOT_IN_USE            = 0;
  GLUT_MENU_IN_USE                = 1;

  // Visibility  state.
  GLUT_NOT_VISIBLE                = 0;
  GLUT_VISIBLE                    = 1;

  // Window status  state.
  GLUT_HIDDEN                     = 0;
  GLUT_FULLY_RETAINED             = 1;
  GLUT_PARTIALLY_RETAINED         = 2;
  GLUT_FULLY_COVERED              = 3;

  // Color index component selection values.
  GLUT_RED                        = 0;
  GLUT_GREEN                      = 1;
  GLUT_BLUE                       = 2;

  // Layers for use.
  GLUT_NORMAL                     = 0;
  GLUT_OVERLAY                    = 1;

{$ifdef Windows}
  // Stroke font constants (use these in GLUT program).
  GLUT_STROKE_ROMAN               = Pointer(0);
  GLUT_STROKE_MONO_ROMAN          = Pointer(1);

  // Bitmap font constants (use these in GLUT program).
  GLUT_BITMAP_9_BY_15             = Pointer(2);
  GLUT_BITMAP_8_BY_13             = Pointer(3);
  GLUT_BITMAP_TIMES_ROMAN_10      = Pointer(4);
  GLUT_BITMAP_TIMES_ROMAN_24      = Pointer(5);
  GLUT_BITMAP_HELVETICA_10        = Pointer(6);
  GLUT_BITMAP_HELVETICA_12        = Pointer(7);
  GLUT_BITMAP_HELVETICA_18        = Pointer(8);
{$else Windows}
var
  // Stroke font constants (use these in GLUT program).
  GLUT_STROKE_ROMAN               : Pointer;
  GLUT_STROKE_MONO_ROMAN          : Pointer;

  // Bitmap font constants (use these in GLUT program).
  GLUT_BITMAP_9_BY_15             : Pointer;
  GLUT_BITMAP_8_BY_13             : Pointer;
  GLUT_BITMAP_TIMES_ROMAN_10      : Pointer;
  GLUT_BITMAP_TIMES_ROMAN_24      : Pointer;
  GLUT_BITMAP_HELVETICA_10        : Pointer;
  GLUT_BITMAP_HELVETICA_12        : Pointer;
  GLUT_BITMAP_HELVETICA_18        : Pointer;

const
{$endif Windows}

  // glutGet parameters.
  GLUT_WINDOW_X                   = 100;
  GLUT_WINDOW_Y                   = 101;
  GLUT_WINDOW_WIDTH               = 102;
  GLUT_WINDOW_HEIGHT              = 103;
  GLUT_WINDOW_BUFFER_SIZE         = 104;
  GLUT_WINDOW_STENCIL_SIZE        = 105;
  GLUT_WINDOW_DEPTH_SIZE          = 106;
  GLUT_WINDOW_RED_SIZE            = 107;
  GLUT_WINDOW_GREEN_SIZE          = 108;
  GLUT_WINDOW_BLUE_SIZE           = 109;
  GLUT_WINDOW_ALPHA_SIZE          = 110;
  GLUT_WINDOW_ACCUM_RED_SIZE      = 111;
  GLUT_WINDOW_ACCUM_GREEN_SIZE    = 112;
  GLUT_WINDOW_ACCUM_BLUE_SIZE     = 113;
  GLUT_WINDOW_ACCUM_ALPHA_SIZE    = 114;
  GLUT_WINDOW_DOUBLEBUFFER        = 115;
  GLUT_WINDOW_RGBA                = 116;
  GLUT_WINDOW_PARENT              = 117;
  GLUT_WINDOW_NUM_CHILDREN        = 118;
  GLUT_WINDOW_COLORMAP_SIZE       = 119;
  GLUT_WINDOW_NUM_SAMPLES         = 120;
  GLUT_WINDOW_STEREO              = 121;
  GLUT_WINDOW_CURSOR              = 122;
  GLUT_SCREEN_WIDTH               = 200;
  GLUT_SCREEN_HEIGHT              = 201;
  GLUT_SCREEN_WIDTH_MM            = 202;
  GLUT_SCREEN_HEIGHT_MM           = 203;
  GLUT_MENU_NUM_ITEMS             = 300;
  GLUT_DISPLAY_MODE_POSSIBLE      = 400;
  GLUT_INIT_WINDOW_X              = 500;
  GLUT_INIT_WINDOW_Y              = 501;
  GLUT_INIT_WINDOW_WIDTH          = 502;
  GLUT_INIT_WINDOW_HEIGHT         = 503;
  GLUT_INIT_DISPLAY_MODE          = 504;
  GLUT_ELAPSED_TIME               = 700;
  GLUT_WINDOW_FORMAT_ID		  = 123;

  // glutDeviceGet parameters.
  GLUT_HAS_KEYBOARD               = 600;
  GLUT_HAS_MOUSE                  = 601;
  GLUT_HAS_SPACEBALL              = 602;
  GLUT_HAS_DIAL_AND_BUTTON_BOX    = 603;
  GLUT_HAS_TABLET                 = 604;
  GLUT_NUM_MOUSE_BUTTONS          = 605;
  GLUT_NUM_SPACEBALL_BUTTONS      = 606;
  GLUT_NUM_BUTTON_BOX_BUTTONS     = 607;
  GLUT_NUM_DIALS                  = 608;
  GLUT_NUM_TABLET_BUTTONS         = 609;
  GLUT_DEVICE_IGNORE_KEY_REPEAT   = 610;
  GLUT_DEVICE_KEY_REPEAT          = 611;
  GLUT_HAS_JOYSTICK               = 612;
  GLUT_OWNS_JOYSTICK              = 613;
  GLUT_JOYSTICK_BUTTONS           = 614;
  GLUT_JOYSTICK_AXES              = 615;
  GLUT_JOYSTICK_POLL_RATE         = 616;


  // glutLayerGet parameters.
  GLUT_OVERLAY_POSSIBLE           = 800;
  GLUT_LAYER_IN_USE               = 801;
  GLUT_HAS_OVERLAY                = 802;
  GLUT_TRANSPARENT_INDEX          = 803;
  GLUT_NORMAL_DAMAGED             = 804;
  GLUT_OVERLAY_DAMAGED            = 805;

  // glutVideoResizeGet parameters.
  GLUT_VIDEO_RESIZE_POSSIBLE       = 900;
  GLUT_VIDEO_RESIZE_IN_USE         = 901;
  GLUT_VIDEO_RESIZE_X_DELTA        = 902;
  GLUT_VIDEO_RESIZE_Y_DELTA        = 903;
  GLUT_VIDEO_RESIZE_WIDTH_DELTA    = 904;
  GLUT_VIDEO_RESIZE_HEIGHT_DELTA   = 905;
  GLUT_VIDEO_RESIZE_X              = 906;
  GLUT_VIDEO_RESIZE_Y              = 907;
  GLUT_VIDEO_RESIZE_WIDTH          = 908;
  GLUT_VIDEO_RESIZE_HEIGHT         = 909;

  // glutGetModifiers return mask.
  GLUT_ACTIVE_SHIFT                = 1;
  GLUT_ACTIVE_CTRL                 = 2;
  GLUT_ACTIVE_ALT                  = 4;

  // glutSetCursor parameters.
  // Basic arrows.
  GLUT_CURSOR_RIGHT_ARROW          = 0;
  GLUT_CURSOR_LEFT_ARROW           = 1;
  // Symbolic cursor shapes.
  GLUT_CURSOR_INFO                 = 2;
  GLUT_CURSOR_DESTROY              = 3;
  GLUT_CURSOR_HELP                 = 4;
  GLUT_CURSOR_CYCLE                = 5;
  GLUT_CURSOR_SPRAY                = 6;
  GLUT_CURSOR_WAIT                 = 7;
  GLUT_CURSOR_TEXT                 = 8;
  GLUT_CURSOR_CROSSHAIR            = 9;
  // Directional cursors.
  GLUT_CURSOR_UP_DOWN              = 10;
  GLUT_CURSOR_LEFT_RIGHT           = 11;
  // Sizing cursors.
  GLUT_CURSOR_TOP_SIDE             = 12;
  GLUT_CURSOR_BOTTOM_SIDE          = 13;
  GLUT_CURSOR_LEFT_SIDE            = 14;
  GLUT_CURSOR_RIGHT_SIDE           = 15;
  GLUT_CURSOR_TOP_LEFT_CORNER      = 16;
  GLUT_CURSOR_TOP_RIGHT_CORNER     = 17;
  GLUT_CURSOR_BOTTOM_RIGHT_CORNER  = 18;
  GLUT_CURSOR_BOTTOM_LEFT_CORNER   = 19;
  // Inherit from parent window.
  GLUT_CURSOR_INHERIT              = 100;
  // Blank cursor.
  GLUT_CURSOR_NONE                 = 101;
  // Fullscreen crosshair (if available).
  GLUT_CURSOR_FULL_CROSSHAIR       = 102;

  // GLUT device control sub-API.
  // glutSetKeyRepeat modes.
  GLUT_KEY_REPEAT_OFF      = 0;
  GLUT_KEY_REPEAT_ON       = 1;
  GLUT_KEY_REPEAT_DEFAULT  = 2;

// Joystick button masks.
  GLUT_JOYSTICK_BUTTON_A = 1;
  GLUT_JOYSTICK_BUTTON_B = 2;
  GLUT_JOYSTICK_BUTTON_C = 4;
  GLUT_JOYSTICK_BUTTON_D = 8;

  // GLUT game mode sub-API.
  // glutGameModeGet.
  GLUT_GAME_MODE_ACTIVE           = 0;
  GLUT_GAME_MODE_POSSIBLE         = 1;
  GLUT_GAME_MODE_WIDTH            = 2;
  GLUT_GAME_MODE_HEIGHT           = 3;
  GLUT_GAME_MODE_PIXEL_DEPTH      = 4;
  GLUT_GAME_MODE_REFRESH_RATE     = 5;
  GLUT_GAME_MODE_DISPLAY_CHANGED  = 6;

{$IFDEF MORPHOS}

{ MorphOS GL works differently due to different dynamic-library handling on Amiga-like }
{ systems, so its headers are included here. }
{$INCLUDE tinyglh.inc}

{$ELSE MORPHOS}

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF GLUT_EXCLUSIVE_FPUMODE}
var
OLD_glutInit:procedure(argcp: PInteger; argv: PPChar); extdecl;
OLD_glutInitDisplayMode:procedure(mode: Cardinal); extdecl;
OLD_glutInitDisplayString:procedure(const str: PChar); extdecl;
OLD_glutInitWindowPosition:procedure(x, y: Integer); extdecl;
OLD_glutInitWindowSize:procedure(width, height: Integer); extdecl;
OLD_glutMainLoop:procedure; extdecl;
OLD_glutCreateWindow:function(const title: PChar): Integer; extdecl;
OLD_glutCreateSubWindow:function(win, x, y, width, height: Integer): Integer; extdecl;
OLD_glutDestroyWindow:procedure(win: Integer); extdecl;
OLD_glutPostRedisplay:procedure; extdecl;
OLD_glutPostWindowRedisplay:procedure(win: Integer); extdecl;
OLD_glutSwapBuffers:procedure; extdecl;
OLD_glutGetWindow:function: Integer; extdecl;
OLD_glutSetWindow:procedure(win: Integer); extdecl;
OLD_glutSetWindowTitle:procedure(const title: PChar); extdecl;
OLD_glutSetIconTitle:procedure(const title: PChar); extdecl;
OLD_glutPositionWindow:procedure(x, y: Integer); extdecl;
OLD_glutReshapeWindow:procedure(width, height: Integer); extdecl;
OLD_glutPopWindow:procedure; extdecl;
OLD_glutPushWindow:procedure; extdecl;
OLD_glutIconifyWindow:procedure; extdecl;
OLD_glutShowWindow:procedure; extdecl;
OLD_glutHideWindow:procedure; extdecl;
OLD_glutFullScreen:procedure; extdecl;
OLD_glutSetCursor:procedure(cursor: Integer); extdecl;
OLD_glutWarpPointer:procedure(x, y: Integer); extdecl;
OLD_glutEstablishOverlay:procedure; extdecl;
OLD_glutRemoveOverlay:procedure; extdecl;
OLD_glutUseLayer:procedure(layer: GLenum); extdecl;
OLD_glutPostOverlayRedisplay:procedure; extdecl;
OLD_glutPostWindowOverlayRedisplay:procedure(win: Integer); extdecl;
OLD_glutShowOverlay:procedure; extdecl;
OLD_glutHideOverlay:procedure; extdecl;
OLD_glutCreateMenu:function(callback: TGlut1IntCallback): Integer; extdecl;
OLD_glutDestroyMenu:procedure(menu: Integer); extdecl;
OLD_glutGetMenu:function: Integer; extdecl;
OLD_glutSetMenu:procedure(menu: Integer); extdecl;
OLD_glutAddMenuEntry:procedure(const caption: PChar; value: Integer); extdecl;
OLD_glutAddSubMenu:procedure(const caption: PChar; submenu: Integer); extdecl;
OLD_glutChangeToMenuEntry:procedure(item: Integer; const caption: PChar; value: Integer); extdecl;
OLD_glutChangeToSubMenu:procedure(item: Integer; const caption: PChar; submenu: Integer); extdecl;
OLD_glutRemoveMenuItem:procedure(item: Integer); extdecl;
OLD_glutAttachMenu:procedure(button: Integer); extdecl;
OLD_glutDetachMenu:procedure(button: Integer); extdecl;
OLD_glutDisplayFunc:procedure(f: TGlutVoidCallback); extdecl;
OLD_glutReshapeFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutKeyboardFunc:procedure(f: TGlut1Char2IntCallback); extdecl;
OLD_glutMouseFunc:procedure(f: TGlut4IntCallback); extdecl;
OLD_glutMotionFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutPassiveMotionFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutEntryFunc:procedure(f: TGlut1IntCallback); extdecl;
OLD_glutVisibilityFunc:procedure(f: TGlut1IntCallback); extdecl;
OLD_glutIdleFunc:procedure(f: TGlutVoidCallback); extdecl;
OLD_glutTimerFunc:procedure(millis: Cardinal; f: TGlut1IntCallback; value: Integer); extdecl;
OLD_glutMenuStateFunc:procedure(f: TGlut1IntCallback); extdecl;
OLD_glutSpecialFunc:procedure(f: TGlut3IntCallback); extdecl;
OLD_glutSpaceballMotionFunc:procedure(f: TGlut3IntCallback); extdecl;
OLD_glutSpaceballRotateFunc:procedure(f: TGlut3IntCallback); extdecl;
OLD_glutSpaceballButtonFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutButtonBoxFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutDialsFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutTabletMotionFunc:procedure(f: TGlut2IntCallback); extdecl;
OLD_glutTabletButtonFunc:procedure(f: TGlut4IntCallback); extdecl;
OLD_glutMenuStatusFunc:procedure(f: TGlut3IntCallback); extdecl;
OLD_glutOverlayDisplayFunc:procedure(f:TGlutVoidCallback); extdecl;
OLD_glutWindowStatusFunc:procedure(f: TGlut1IntCallback); extdecl;
OLD_glutKeyboardUpFunc:procedure(f: TGlut1Char2IntCallback); extdecl;
OLD_glutSpecialUpFunc:procedure(f: TGlut3IntCallback); extdecl;
OLD_glutJoystickFunc:procedure(f: TGlut1UInt3IntCallback; pollInterval: Integer); extdecl;
OLD_glutSetColor:procedure(cell: Integer; red, green, blue: GLfloat); extdecl;
OLD_glutGetColor:function(ndx, component: Integer): GLfloat; extdecl;
OLD_glutCopyColormap:procedure(win: Integer); extdecl;
OLD_glutGet:function(t: GLenum): Integer; extdecl;
OLD_glutDeviceGet:function(t: GLenum): Integer; extdecl;
OLD_glutExtensionSupported:function(const name: PChar): Integer; extdecl;
OLD_glutGetModifiers:function: Integer; extdecl;
OLD_glutLayerGet:function(t: GLenum): Integer; extdecl;
OLD_glutBitmapCharacter:procedure(font : pointer; character: Integer); extdecl;
OLD_glutBitmapWidth:function(font : pointer; character: Integer): Integer; extdecl;
OLD_glutStrokeCharacter:procedure(font : pointer; character: Integer); extdecl;
OLD_glutStrokeWidth:function(font : pointer; character: Integer): Integer; extdecl;
OLD_glutBitmapLength:function(font: pointer; const str: PChar): Integer; extdecl;
OLD_glutStrokeLength:function(font: pointer; const str: PChar): Integer; extdecl;
OLD_glutWireSphere:procedure(radius: GLdouble; slices, stacks: GLint); extdecl;
OLD_glutSolidSphere:procedure(radius: GLdouble; slices, stacks: GLint); extdecl;
OLD_glutWireCone:procedure(base, height: GLdouble; slices, stacks: GLint); extdecl;
OLD_glutSolidCone:procedure(base, height: GLdouble; slices, stacks: GLint); extdecl;
OLD_glutWireCube:procedure(size: GLdouble); extdecl;
OLD_glutSolidCube:procedure(size: GLdouble); extdecl;
OLD_glutWireTorus:procedure(innerRadius, outerRadius: GLdouble; sides, rings: GLint); extdecl;
OLD_glutSolidTorus:procedure(innerRadius, outerRadius: GLdouble; sides, rings: GLint); extdecl;
OLD_glutWireDodecahedron:procedure; extdecl;
OLD_glutSolidDodecahedron:procedure; extdecl;
OLD_glutWireTeapot:procedure(size: GLdouble); extdecl;
OLD_glutSolidTeapot:procedure(size: GLdouble); extdecl;
OLD_glutWireOctahedron:procedure; extdecl;
OLD_glutSolidOctahedron:procedure; extdecl;
OLD_glutWireTetrahedron:procedure; extdecl;
OLD_glutSolidTetrahedron:procedure; extdecl;
OLD_glutWireIcosahedron:procedure; extdecl;
OLD_glutSolidIcosahedron:procedure; extdecl;
OLD_glutVideoResizeGet:function(param: GLenum): Integer; extdecl;
OLD_glutSetupVideoResizing:procedure; extdecl;
OLD_glutStopVideoResizing:procedure; extdecl;
OLD_glutVideoResize:procedure(x, y, width, height: Integer); extdecl;
OLD_glutVideoPan:procedure(x, y, width, height: Integer); extdecl;
OLD_glutReportErrors:procedure; extdecl;
OLD_glutIgnoreKeyRepeat:procedure(ignore: Integer); extdecl;
OLD_glutSetKeyRepeat:procedure(repeatMode: Integer); extdecl;
OLD_glutForceJoystickFunc:procedure; extdecl;
OLD_glutGameModeString:procedure(const AString : PChar); extdecl;
OLD_glutEnterGameMode:function: integer; extdecl;
OLD_glutLeaveGameMode:procedure; extdecl;
OLD_glutGameModeGet:function(mode : GLenum): integer; extdecl;

procedure glutInit(argcp: PInteger; argv: PPChar); mode_inline;
procedure glutInitDisplayMode(mode: Cardinal); mode_inline;
procedure glutInitDisplayString(const str: PChar); mode_inline;
procedure glutInitWindowPosition(x, y: Integer); mode_inline;
procedure glutInitWindowSize(width, height: Integer); mode_inline;
procedure glutMainLoop; mode_inline;
function glutCreateWindow(const title: PChar): Integer; mode_inline;
function glutCreateSubWindow(win, x, y, width, height: Integer): Integer; mode_inline;
procedure glutDestroyWindow(win: Integer); mode_inline;
procedure glutPostRedisplay; mode_inline;
procedure glutPostWindowRedisplay(win: Integer); mode_inline;
procedure glutSwapBuffers; mode_inline;
function glutGetWindow: Integer; mode_inline;
procedure glutSetWindow(win: Integer); mode_inline;
procedure glutSetWindowTitle(const title: PChar); mode_inline;
procedure glutSetIconTitle(const title: PChar); mode_inline;
procedure glutPositionWindow(x, y: Integer); mode_inline;
procedure glutReshapeWindow(width, height: Integer); mode_inline;
procedure glutPopWindow; mode_inline;
procedure glutPushWindow; mode_inline;
procedure glutIconifyWindow; mode_inline;
procedure glutShowWindow; mode_inline;
procedure glutHideWindow; mode_inline;
procedure glutFullScreen; mode_inline;
procedure glutSetCursor(cursor: Integer); mode_inline;
procedure glutWarpPointer(x, y: Integer); mode_inline;
procedure glutEstablishOverlay; mode_inline;
procedure glutRemoveOverlay; mode_inline;
procedure glutUseLayer(layer: GLenum); mode_inline;
procedure glutPostOverlayRedisplay; mode_inline;
procedure glutPostWindowOverlayRedisplay(win: Integer); mode_inline;
procedure glutShowOverlay; mode_inline;
procedure glutHideOverlay; mode_inline;
function glutCreateMenu(callback: TGlut1IntCallback): Integer; mode_inline;
procedure glutDestroyMenu(menu: Integer); mode_inline;
function glutGetMenu: Integer; mode_inline;
procedure glutSetMenu(menu: Integer); mode_inline;
procedure glutAddMenuEntry(const caption: PChar; value: Integer); mode_inline;
procedure glutAddSubMenu(const caption: PChar; submenu: Integer); mode_inline;
procedure glutChangeToMenuEntry(item: Integer; const caption: PChar; value: Integer); mode_inline;
procedure glutChangeToSubMenu(item: Integer; const caption: PChar; submenu: Integer); mode_inline;
procedure glutRemoveMenuItem(item: Integer); mode_inline;
procedure glutAttachMenu(button: Integer); mode_inline;
procedure glutDetachMenu(button: Integer); mode_inline;
procedure glutDisplayFunc(f: TGlutVoidCallback); mode_inline;
procedure glutReshapeFunc(f: TGlut2IntCallback); mode_inline;
procedure glutKeyboardFunc(f: TGlut1Char2IntCallback); mode_inline;
procedure glutMouseFunc(f: TGlut4IntCallback); mode_inline;
procedure glutMotionFunc(f: TGlut2IntCallback); mode_inline;
procedure glutPassiveMotionFunc(f: TGlut2IntCallback); mode_inline;
procedure glutEntryFunc(f: TGlut1IntCallback); mode_inline;
procedure glutVisibilityFunc(f: TGlut1IntCallback); mode_inline;
procedure glutIdleFunc(f: TGlutVoidCallback); mode_inline;
procedure glutTimerFunc(millis: Cardinal; f: TGlut1IntCallback; value: Integer); mode_inline;
procedure glutMenuStateFunc(f: TGlut1IntCallback); mode_inline;
procedure glutSpecialFunc(f: TGlut3IntCallback); mode_inline;
procedure glutSpaceballMotionFunc(f: TGlut3IntCallback); mode_inline;
procedure glutSpaceballRotateFunc(f: TGlut3IntCallback); mode_inline;
procedure glutSpaceballButtonFunc(f: TGlut2IntCallback); mode_inline;
procedure glutButtonBoxFunc(f: TGlut2IntCallback); mode_inline;
procedure glutDialsFunc(f: TGlut2IntCallback); mode_inline;
procedure glutTabletMotionFunc(f: TGlut2IntCallback); mode_inline;
procedure glutTabletButtonFunc(f: TGlut4IntCallback); mode_inline;
procedure glutMenuStatusFunc(f: TGlut3IntCallback); mode_inline;
procedure glutOverlayDisplayFunc(f:TGlutVoidCallback); mode_inline;
procedure glutWindowStatusFunc(f: TGlut1IntCallback); mode_inline;
procedure glutKeyboardUpFunc(f: TGlut1Char2IntCallback); mode_inline;
procedure glutSpecialUpFunc(f: TGlut3IntCallback); mode_inline;
procedure glutJoystickFunc(f: TGlut1UInt3IntCallback; pollInterval: Integer); mode_inline;
procedure glutSetColor(cell: Integer; red, green, blue: GLfloat); mode_inline;
function glutGetColor(ndx, component: Integer): GLfloat; mode_inline;
procedure glutCopyColormap(win: Integer); mode_inline;
function glutGet(t: GLenum): Integer; mode_inline;
function glutDeviceGet(t: GLenum): Integer; mode_inline;
function glutExtensionSupported(const name: PChar): Integer; mode_inline;
function glutGetModifiers: Integer; mode_inline;
function glutLayerGet(t: GLenum): Integer; mode_inline;
procedure glutBitmapCharacter(font : pointer; character: Integer); mode_inline;
function glutBitmapWidth(font : pointer; character: Integer): Integer; mode_inline;
procedure glutStrokeCharacter(font : pointer; character: Integer); mode_inline;
function glutStrokeWidth(font : pointer; character: Integer): Integer; mode_inline;
function glutBitmapLength(font: pointer; const str: PChar): Integer; mode_inline;
function glutStrokeLength(font: pointer; const str: PChar): Integer; mode_inline;
procedure glutWireSphere(radius: GLdouble; slices, stacks: GLint); mode_inline;
procedure glutSolidSphere(radius: GLdouble; slices, stacks: GLint); mode_inline;
procedure glutWireCone(base, height: GLdouble; slices, stacks: GLint); mode_inline;
procedure glutSolidCone(base, height: GLdouble; slices, stacks: GLint); mode_inline;
procedure glutWireCube(size: GLdouble); mode_inline;
procedure glutSolidCube(size: GLdouble); mode_inline;
procedure glutWireTorus(innerRadius, outerRadius: GLdouble; sides, rings: GLint); mode_inline;
procedure glutSolidTorus(innerRadius, outerRadius: GLdouble; sides, rings: GLint); mode_inline;
procedure glutWireDodecahedron; mode_inline;
procedure glutSolidDodecahedron; mode_inline;
procedure glutWireTeapot(size: GLdouble); mode_inline;
procedure glutSolidTeapot(size: GLdouble); mode_inline;
procedure glutWireOctahedron; mode_inline;
procedure glutSolidOctahedron; mode_inline;
procedure glutWireTetrahedron; mode_inline;
procedure glutSolidTetrahedron; mode_inline;
procedure glutWireIcosahedron; mode_inline;
procedure glutSolidIcosahedron; mode_inline;
function glutVideoResizeGet(param: GLenum): Integer; mode_inline;
procedure glutSetupVideoResizing; mode_inline;
procedure glutStopVideoResizing; mode_inline;
procedure glutVideoResize(x, y, width, height: Integer); mode_inline;
procedure glutVideoPan(x, y, width, height: Integer); mode_inline;
procedure glutReportErrors; mode_inline;
procedure glutIgnoreKeyRepeat(ignore: Integer); mode_inline;
procedure glutSetKeyRepeat(repeatMode: Integer); mode_inline;
procedure glutForceJoystickFunc; mode_inline;
procedure glutGameModeString(const AString : PChar); mode_inline;
function glutEnterGameMode: integer; mode_inline;
procedure glutLeaveGameMode; mode_inline;
function glutGameModeGet(mode : GLenum): integer; mode_inline;


{$ELSE GLUT_EXCLUSIVE_FPUMODE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
// GLUT initialization sub-API.
  glutInit: procedure(argcp: PInteger; argv: PPChar); extdecl;
  glutInitDisplayMode: procedure(mode: Cardinal); extdecl;
  glutInitDisplayString: procedure(const str: PChar); extdecl;
  glutInitWindowPosition: procedure(x, y: Integer); extdecl;
  glutInitWindowSize: procedure(width, height: Integer); extdecl;
  glutMainLoop: procedure; extdecl;

// GLUT window sub-API.
  glutCreateWindow: function(const title: PChar): Integer; extdecl;
  glutCreateSubWindow: function(win, x, y, width, height: Integer): Integer; extdecl;
  glutDestroyWindow: procedure(win: Integer); extdecl;
  glutPostRedisplay: procedure; extdecl;
  glutPostWindowRedisplay: procedure(win: Integer); extdecl;
  glutSwapBuffers: procedure; extdecl;
  glutGetWindow: function: Integer; extdecl;
  glutSetWindow: procedure(win: Integer); extdecl;
  glutSetWindowTitle: procedure(const title: PChar); extdecl;
  glutSetIconTitle: procedure(const title: PChar); extdecl;
  glutPositionWindow: procedure(x, y: Integer); extdecl;
  glutReshapeWindow: procedure(width, height: Integer); extdecl;
  glutPopWindow: procedure; extdecl;
  glutPushWindow: procedure; extdecl;
  glutIconifyWindow: procedure; extdecl;
  glutShowWindow: procedure; extdecl;
  glutHideWindow: procedure; extdecl;
  glutFullScreen: procedure; extdecl;
  glutSetCursor: procedure(cursor: Integer); extdecl;
  glutWarpPointer: procedure(x, y: Integer); extdecl;

// GLUT overlay sub-API.
  glutEstablishOverlay: procedure; extdecl;
  glutRemoveOverlay: procedure; extdecl;
  glutUseLayer: procedure(layer: GLenum); extdecl;
  glutPostOverlayRedisplay: procedure; extdecl;
  glutPostWindowOverlayRedisplay: procedure(win: Integer); extdecl;
  glutShowOverlay: procedure; extdecl;
  glutHideOverlay: procedure; extdecl;

// GLUT menu sub-API.
  glutCreateMenu: function(callback: TGlut1IntCallback): Integer; extdecl;
  glutDestroyMenu: procedure(menu: Integer); extdecl;
  glutGetMenu: function: Integer; extdecl;
  glutSetMenu: procedure(menu: Integer); extdecl;
  glutAddMenuEntry: procedure(const caption: PChar; value: Integer); extdecl;
  glutAddSubMenu: procedure(const caption: PChar; submenu: Integer); extdecl;
  glutChangeToMenuEntry: procedure(item: Integer; const caption: PChar; value: Integer); extdecl;
  glutChangeToSubMenu: procedure(item: Integer; const caption: PChar; submenu: Integer); extdecl;
  glutRemoveMenuItem: procedure(item: Integer); extdecl;
  glutAttachMenu: procedure(button: Integer); extdecl;
  glutDetachMenu: procedure(button: Integer); extdecl;

// GLUT window callback sub-API.
  glutDisplayFunc: procedure(f: TGlutVoidCallback); extdecl;
  glutReshapeFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutKeyboardFunc: procedure(f: TGlut1Char2IntCallback); extdecl;
  glutMouseFunc: procedure(f: TGlut4IntCallback); extdecl;
  glutMotionFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutPassiveMotionFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutEntryFunc: procedure(f: TGlut1IntCallback); extdecl;
  glutVisibilityFunc: procedure(f: TGlut1IntCallback); extdecl;
  glutIdleFunc: procedure(f: TGlutVoidCallback); extdecl;
  glutTimerFunc: procedure(millis: Cardinal; f: TGlut1IntCallback; value: Integer); extdecl;
  glutMenuStateFunc: procedure(f: TGlut1IntCallback); extdecl;
  glutSpecialFunc: procedure(f: TGlut3IntCallback); extdecl;
  glutSpaceballMotionFunc: procedure(f: TGlut3IntCallback); extdecl;
  glutSpaceballRotateFunc: procedure(f: TGlut3IntCallback); extdecl;
  glutSpaceballButtonFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutButtonBoxFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutDialsFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutTabletMotionFunc: procedure(f: TGlut2IntCallback); extdecl;
  glutTabletButtonFunc: procedure(f: TGlut4IntCallback); extdecl;
  glutMenuStatusFunc: procedure(f: TGlut3IntCallback); extdecl;
  glutOverlayDisplayFunc: procedure(f:TGlutVoidCallback); extdecl;
  glutWindowStatusFunc: procedure(f: TGlut1IntCallback); extdecl;
  glutKeyboardUpFunc: procedure(f: TGlut1Char2IntCallback); extdecl;
  glutSpecialUpFunc: procedure(f: TGlut3IntCallback); extdecl;
  glutJoystickFunc: procedure(f: TGlut1UInt3IntCallback; pollInterval: Integer); extdecl;

// GLUT color index sub-API.
  glutSetColor: procedure(cell: Integer; red, green, blue: GLfloat); extdecl;
  glutGetColor: function(ndx, component: Integer): GLfloat; extdecl;
  glutCopyColormap: procedure(win: Integer); extdecl;

// GLUT state retrieval sub-API.
  glutGet: function(t: GLenum): Integer; extdecl;
  glutDeviceGet: function(t: GLenum): Integer; extdecl;

// GLUT extension support sub-API
  glutExtensionSupported: function(const name: PChar): Integer; extdecl;
  glutGetModifiers: function: Integer; extdecl;
  glutLayerGet: function(t: GLenum): Integer; extdecl;

// GLUT font sub-API
  glutBitmapCharacter: procedure(font : pointer; character: Integer); extdecl;
  glutBitmapWidth: function(font : pointer; character: Integer): Integer; extdecl;
  glutStrokeCharacter: procedure(font : pointer; character: Integer); extdecl;
  glutStrokeWidth: function(font : pointer; character: Integer): Integer; extdecl;
  glutBitmapLength: function(font: pointer; const str: PChar): Integer; extdecl;
  glutStrokeLength: function(font: pointer; const str: PChar): Integer; extdecl;

// GLUT pre-built models sub-API
  glutWireSphere: procedure(radius: GLdouble; slices, stacks: GLint); extdecl;
  glutSolidSphere: procedure(radius: GLdouble; slices, stacks: GLint); extdecl;
  glutWireCone: procedure(base, height: GLdouble; slices, stacks: GLint); extdecl;
  glutSolidCone: procedure(base, height: GLdouble; slices, stacks: GLint); extdecl;
  glutWireCube: procedure(size: GLdouble); extdecl;
  glutSolidCube: procedure(size: GLdouble); extdecl;
  glutWireTorus: procedure(innerRadius, outerRadius: GLdouble; sides, rings: GLint); extdecl;
  glutSolidTorus: procedure(innerRadius, outerRadius: GLdouble; sides, rings: GLint); extdecl;
  glutWireDodecahedron: procedure; extdecl;
  glutSolidDodecahedron: procedure; extdecl;
  glutWireTeapot: procedure(size: GLdouble); extdecl;
  glutSolidTeapot: procedure(size: GLdouble); extdecl;
  glutWireOctahedron: procedure; extdecl;
  glutSolidOctahedron: procedure; extdecl;
  glutWireTetrahedron: procedure; extdecl;
  glutSolidTetrahedron: procedure; extdecl;
  glutWireIcosahedron: procedure; extdecl;
  glutSolidIcosahedron: procedure; extdecl;

// GLUT video resize sub-API.
  glutVideoResizeGet: function(param: GLenum): Integer; extdecl;
  glutSetupVideoResizing: procedure; extdecl;
  glutStopVideoResizing: procedure; extdecl;
  glutVideoResize: procedure(x, y, width, height: Integer); extdecl;
  glutVideoPan: procedure(x, y, width, height: Integer); extdecl;

// GLUT debugging sub-API.
  glutReportErrors: procedure; extdecl;

// GLUT device control sub-API.

  glutIgnoreKeyRepeat: procedure(ignore: Integer); extdecl;
  glutSetKeyRepeat: procedure(repeatMode: Integer); extdecl;
  glutForceJoystickFunc: procedure; extdecl;

// GLUT game mode sub-API.

  //example glutGameModeString('1280x1024:32@75');
  glutGameModeString : procedure (const AString : PChar); extdecl;
  glutEnterGameMode : function : integer; extdecl;
  glutLeaveGameMode : procedure; extdecl;
  glutGameModeGet : function (mode : GLenum) : integer; extdecl;
<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

procedure LoadGlut(const dll: String);
procedure UnloadGlut;
=======
=======
>>>>>>> origin/fixes_2_2

{$ENDIF GLUT_EXCLUSIVE_FPUMODE}
{$ENDIF MORPHOS}

procedure LoadGlut(const dll: String);
procedure FreeGlut;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

{$IFDEF MORPHOS}

{ MorphOS GL works differently due to different dynamic-library handling on Amiga-like }
{ systems, so its functions are included here. }
{$INCLUDE tinygl.inc}

{$ELSE MORPHOS}
<<<<<<< HEAD
<<<<<<< HEAD
uses FreeGlut;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  hDLL: TLibHandle;
{$ENDIF MORPHOS}

<<<<<<< HEAD
<<<<<<< HEAD
procedure UnloadGlut;
=======
procedure FreeGlut;
>>>>>>> graemeg/fixes_2_2
=======
procedure FreeGlut;
>>>>>>> origin/fixes_2_2
begin
{$IFDEF MORPHOS}
  // MorphOS's GL will closed down by TinyGL unit, nothing is needed here.
{$ELSE MORPHOS}

  if (hDLL <> 0) then
    FreeLibrary(hDLL);

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF GLUT_EXCLUSIVE_FPUMODE}

  @OLD_glutInit := nil;
  @OLD_glutInitDisplayMode := nil;
  @OLD_glutInitDisplayString := nil;
  @OLD_glutInitWindowPosition := nil;
  @OLD_glutInitWindowSize := nil;
  @OLD_glutMainLoop := nil;
  @OLD_glutCreateWindow := nil;
  @OLD_glutCreateSubWindow := nil;
  @OLD_glutDestroyWindow := nil;
  @OLD_glutPostRedisplay := nil;
  @OLD_glutPostWindowRedisplay := nil;
  @OLD_glutSwapBuffers := nil;
  @OLD_glutGetWindow := nil;
  @OLD_glutSetWindow := nil;
  @OLD_glutSetWindowTitle := nil;
  @OLD_glutSetIconTitle := nil;
  @OLD_glutPositionWindow := nil;
  @OLD_glutReshapeWindow := nil;
  @OLD_glutPopWindow := nil;
  @OLD_glutPushWindow := nil;
  @OLD_glutIconifyWindow := nil;
  @OLD_glutShowWindow := nil;
  @OLD_glutHideWindow := nil;
  @OLD_glutFullScreen := nil;
  @OLD_glutSetCursor := nil;
  @OLD_glutWarpPointer := nil;
  @OLD_glutEstablishOverlay := nil;
  @OLD_glutRemoveOverlay := nil;
  @OLD_glutUseLayer := nil;
  @OLD_glutPostOverlayRedisplay := nil;
  @OLD_glutPostWindowOverlayRedisplay := nil;
  @OLD_glutShowOverlay := nil;
  @OLD_glutHideOverlay := nil;
  @OLD_glutCreateMenu := nil;
  @OLD_glutDestroyMenu := nil;
  @OLD_glutGetMenu := nil;
  @OLD_glutSetMenu := nil;
  @OLD_glutAddMenuEntry := nil;
  @OLD_glutAddSubMenu := nil;
  @OLD_glutChangeToMenuEntry := nil;
  @OLD_glutChangeToSubMenu := nil;
  @OLD_glutRemoveMenuItem := nil;
  @OLD_glutAttachMenu := nil;
  @OLD_glutDetachMenu := nil;
  @OLD_glutDisplayFunc := nil;
  @OLD_glutReshapeFunc := nil;
  @OLD_glutKeyboardFunc := nil;
  @OLD_glutMouseFunc := nil;
  @OLD_glutMotionFunc := nil;
  @OLD_glutPassiveMotionFunc := nil;
  @OLD_glutEntryFunc := nil;
  @OLD_glutVisibilityFunc := nil;
  @OLD_glutIdleFunc := nil;
  @OLD_glutTimerFunc := nil;
  @OLD_glutMenuStateFunc := nil;
  @OLD_glutSpecialFunc := nil;
  @OLD_glutSpaceballMotionFunc := nil;
  @OLD_glutSpaceballRotateFunc := nil;
  @OLD_glutSpaceballButtonFunc := nil;
  @OLD_glutButtonBoxFunc := nil;
  @OLD_glutDialsFunc := nil;
  @OLD_glutTabletMotionFunc := nil;
  @OLD_glutTabletButtonFunc := nil;
  @OLD_glutMenuStatusFunc := nil;
  @OLD_glutOverlayDisplayFunc := nil;
  @OLD_glutWindowStatusFunc := nil;
  @OLD_glutKeyboardUpFunc := nil;
  @OLD_glutSpecialUpFunc := nil;
  @OLD_glutJoystickFunc := nil;
  @OLD_glutSetColor := nil;
  @OLD_glutGetColor := nil;
  @OLD_glutCopyColormap := nil;
  @OLD_glutGet := nil;
  @OLD_glutDeviceGet := nil;
  @OLD_glutExtensionSupported := nil;
  @OLD_glutGetModifiers := nil;
  @OLD_glutLayerGet := nil;
  @OLD_glutBitmapCharacter := nil;
  @OLD_glutBitmapWidth := nil;
  @OLD_glutStrokeCharacter := nil;
  @OLD_glutStrokeWidth := nil;
  @OLD_glutBitmapLength := nil;
  @OLD_glutStrokeLength := nil;
  @OLD_glutWireSphere := nil;
  @OLD_glutSolidSphere := nil;
  @OLD_glutWireCone := nil;
  @OLD_glutSolidCone := nil;
  @OLD_glutWireCube := nil;
  @OLD_glutSolidCube := nil;
  @OLD_glutWireTorus := nil;
  @OLD_glutSolidTorus := nil;
  @OLD_glutWireDodecahedron := nil;
  @OLD_glutSolidDodecahedron := nil;
  @OLD_glutWireTeapot := nil;
  @OLD_glutSolidTeapot := nil;
  @OLD_glutWireOctahedron := nil;
  @OLD_glutSolidOctahedron := nil;
  @OLD_glutWireTetrahedron := nil;
  @OLD_glutSolidTetrahedron := nil;
  @OLD_glutWireIcosahedron := nil;
  @OLD_glutSolidIcosahedron := nil;
  @OLD_glutVideoResizeGet := nil;
  @OLD_glutSetupVideoResizing := nil;
  @OLD_glutStopVideoResizing := nil;
  @OLD_glutVideoResize := nil;
  @OLD_glutVideoPan := nil;
  @OLD_glutReportErrors := nil;
  @OLD_glutIgnoreKeyRepeat := nil;
  @OLD_glutSetKeyRepeat := nil;
  @OLD_glutForceJoystickFunc := nil;
  @OLD_glutGameModeString := nil;
  @OLD_glutEnterGameMode := nil;
  @OLD_glutLeaveGameMode := nil;
  @OLD_glutGameModeGet := nil;

{$ELSE GLUT_EXCLUSIVE_FPUMODE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  @glutInit := nil;
  @glutInitDisplayMode := nil;
  @glutInitDisplayString := nil;
  @glutInitWindowPosition := nil;
  @glutInitWindowSize := nil;
  @glutMainLoop := nil;
  @glutCreateWindow := nil;
  @glutCreateSubWindow := nil;
  @glutDestroyWindow := nil;
  @glutPostRedisplay := nil;
  @glutPostWindowRedisplay := nil;
  @glutSwapBuffers := nil;
  @glutGetWindow := nil;
  @glutSetWindow := nil;
  @glutSetWindowTitle := nil;
  @glutSetIconTitle := nil;
  @glutPositionWindow := nil;
  @glutReshapeWindow := nil;
  @glutPopWindow := nil;
  @glutPushWindow := nil;
  @glutIconifyWindow := nil;
  @glutShowWindow := nil;
  @glutHideWindow := nil;
  @glutFullScreen := nil;
  @glutSetCursor := nil;
  @glutWarpPointer := nil;
  @glutEstablishOverlay := nil;
  @glutRemoveOverlay := nil;
  @glutUseLayer := nil;
  @glutPostOverlayRedisplay := nil;
  @glutPostWindowOverlayRedisplay := nil;
  @glutShowOverlay := nil;
  @glutHideOverlay := nil;
  @glutCreateMenu := nil;
  @glutDestroyMenu := nil;
  @glutGetMenu := nil;
  @glutSetMenu := nil;
  @glutAddMenuEntry := nil;
  @glutAddSubMenu := nil;
  @glutChangeToMenuEntry := nil;
  @glutChangeToSubMenu := nil;
  @glutRemoveMenuItem := nil;
  @glutAttachMenu := nil;
  @glutDetachMenu := nil;
  @glutDisplayFunc := nil;
  @glutReshapeFunc := nil;
  @glutKeyboardFunc := nil;
  @glutMouseFunc := nil;
  @glutMotionFunc := nil;
  @glutPassiveMotionFunc := nil;
  @glutEntryFunc := nil;
  @glutVisibilityFunc := nil;
  @glutIdleFunc := nil;
  @glutTimerFunc := nil;
  @glutMenuStateFunc := nil;
  @glutSpecialFunc := nil;
  @glutSpaceballMotionFunc := nil;
  @glutSpaceballRotateFunc := nil;
  @glutSpaceballButtonFunc := nil;
  @glutButtonBoxFunc := nil;
  @glutDialsFunc := nil;
  @glutTabletMotionFunc := nil;
  @glutTabletButtonFunc := nil;
  @glutMenuStatusFunc := nil;
  @glutOverlayDisplayFunc := nil;
  @glutWindowStatusFunc := nil;
  @glutKeyboardUpFunc := nil;
  @glutSpecialUpFunc := nil;
  @glutJoystickFunc := nil;
  @glutSetColor := nil;
  @glutGetColor := nil;
  @glutCopyColormap := nil;
  @glutGet := nil;
  @glutDeviceGet := nil;
  @glutExtensionSupported := nil;
  @glutGetModifiers := nil;
  @glutLayerGet := nil;
  @glutBitmapCharacter := nil;
  @glutBitmapWidth := nil;
  @glutStrokeCharacter := nil;
  @glutStrokeWidth := nil;
  @glutBitmapLength := nil;
  @glutStrokeLength := nil;
  @glutWireSphere := nil;
  @glutSolidSphere := nil;
  @glutWireCone := nil;
  @glutSolidCone := nil;
  @glutWireCube := nil;
  @glutSolidCube := nil;
  @glutWireTorus := nil;
  @glutSolidTorus := nil;
  @glutWireDodecahedron := nil;
  @glutSolidDodecahedron := nil;
  @glutWireTeapot := nil;
  @glutSolidTeapot := nil;
  @glutWireOctahedron := nil;
  @glutSolidOctahedron := nil;
  @glutWireTetrahedron := nil;
  @glutSolidTetrahedron := nil;
  @glutWireIcosahedron := nil;
  @glutSolidIcosahedron := nil;
  @glutVideoResizeGet := nil;
  @glutSetupVideoResizing := nil;
  @glutStopVideoResizing := nil;
  @glutVideoResize := nil;
  @glutVideoPan := nil;
  @glutReportErrors := nil;
  @glutIgnoreKeyRepeat := nil;
  @glutSetKeyRepeat := nil;
  @glutForceJoystickFunc := nil;
  @glutGameModeString := nil;
  @glutEnterGameMode := nil;
  @glutLeaveGameMode := nil;
  @glutGameModeGet := nil;
<<<<<<< HEAD
<<<<<<< HEAD
  
  UnloadFreeGlut;
=======
{$ENDIF GLUT_EXCLUSIVE_FPUMODE}
>>>>>>> graemeg/fixes_2_2
=======
{$ENDIF GLUT_EXCLUSIVE_FPUMODE}
>>>>>>> origin/fixes_2_2
{$ENDIF MORPHOS}
end;

procedure LoadGlut(const dll: String);
{$IFDEF MORPHOS}
begin
  // MorphOS's GL has own initialization in TinyGL unit, nothing is needed here.
end;
{$ELSE MORPHOS}
var
  MethodName: string = '';

  function GetGLutProcAddress(Lib: PtrInt; ProcName: PChar): Pointer;
  begin
    MethodName:=ProcName;
    Result:=GetProcAddress(Lib, ProcName);
  end;

begin

<<<<<<< HEAD
<<<<<<< HEAD
  UnloadGlut;
=======
  FreeGlut;
>>>>>>> graemeg/fixes_2_2
=======
  FreeGlut;
>>>>>>> origin/fixes_2_2

  hDLL := LoadLibrary(PChar(dll));
  if hDLL = 0 then raise Exception.Create('Could not load Glut from ' + dll);
  try
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF GLUT_EXCLUSIVE_FPUMODE}
    @OLD_glutInit := GetGLutProcAddress(hDLL, 'glutInit');
    @OLD_glutInitDisplayMode := GetGLutProcAddress(hDLL, 'glutInitDisplayMode');
    @OLD_glutInitDisplayString := GetGLutProcAddress(hDLL, 'glutInitDisplayString');
    @OLD_glutInitWindowPosition := GetGLutProcAddress(hDLL, 'glutInitWindowPosition');
    @OLD_glutInitWindowSize := GetGLutProcAddress(hDLL, 'glutInitWindowSize');
    @OLD_glutMainLoop := GetGLutProcAddress(hDLL, 'glutMainLoop');
    @OLD_glutCreateWindow := GetGLutProcAddress(hDLL, 'glutCreateWindow');
    @OLD_glutCreateSubWindow := GetGLutProcAddress(hDLL, 'glutCreateSubWindow');
    @OLD_glutDestroyWindow := GetGLutProcAddress(hDLL, 'glutDestroyWindow');
    @OLD_glutPostRedisplay := GetGLutProcAddress(hDLL, 'glutPostRedisplay');
    @OLD_glutPostWindowRedisplay := GetGLutProcAddress(hDLL, 'glutPostWindowRedisplay');
    @OLD_glutSwapBuffers := GetGLutProcAddress(hDLL, 'glutSwapBuffers');
    @OLD_glutGetWindow := GetGLutProcAddress(hDLL, 'glutGetWindow');
    @OLD_glutSetWindow := GetGLutProcAddress(hDLL, 'glutSetWindow');
    @OLD_glutSetWindowTitle := GetGLutProcAddress(hDLL, 'glutSetWindowTitle');
    @OLD_glutSetIconTitle := GetGLutProcAddress(hDLL, 'glutSetIconTitle');
    @OLD_glutPositionWindow := GetGLutProcAddress(hDLL, 'glutPositionWindow');
    @OLD_glutReshapeWindow := GetGLutProcAddress(hDLL, 'glutReshapeWindow');
    @OLD_glutPopWindow := GetGLutProcAddress(hDLL, 'glutPopWindow');
    @OLD_glutPushWindow := GetGLutProcAddress(hDLL, 'glutPushWindow');
    @OLD_glutIconifyWindow := GetGLutProcAddress(hDLL, 'glutIconifyWindow');
    @OLD_glutShowWindow := GetGLutProcAddress(hDLL, 'glutShowWindow');
    @OLD_glutHideWindow := GetGLutProcAddress(hDLL, 'glutHideWindow');
    @OLD_glutFullScreen := GetGLutProcAddress(hDLL, 'glutFullScreen');
    @OLD_glutSetCursor := GetGLutProcAddress(hDLL, 'glutSetCursor');
    @OLD_glutWarpPointer := GetGLutProcAddress(hDLL, 'glutWarpPointer');
    @OLD_glutEstablishOverlay := GetGLutProcAddress(hDLL, 'glutEstablishOverlay');
    @OLD_glutRemoveOverlay := GetGLutProcAddress(hDLL, 'glutRemoveOverlay');
    @OLD_glutUseLayer := GetGLutProcAddress(hDLL, 'glutUseLayer');
    @OLD_glutPostOverlayRedisplay := GetGLutProcAddress(hDLL, 'glutPostOverlayRedisplay');
    @OLD_glutPostWindowOverlayRedisplay := GetGLutProcAddress(hDLL, 'glutPostWindowOverlayRedisplay');
    @OLD_glutShowOverlay := GetGLutProcAddress(hDLL, 'glutShowOverlay');
    @OLD_glutHideOverlay := GetGLutProcAddress(hDLL, 'glutHideOverlay');
    @OLD_glutCreateMenu := GetGLutProcAddress(hDLL, 'glutCreateMenu');
    @OLD_glutDestroyMenu := GetGLutProcAddress(hDLL, 'glutDestroyMenu');
    @OLD_glutGetMenu := GetGLutProcAddress(hDLL, 'glutGetMenu');
    @OLD_glutSetMenu := GetGLutProcAddress(hDLL, 'glutSetMenu');
    @OLD_glutAddMenuEntry := GetGLutProcAddress(hDLL, 'glutAddMenuEntry');
    @OLD_glutAddSubMenu := GetGLutProcAddress(hDLL, 'glutAddSubMenu');
    @OLD_glutChangeToMenuEntry := GetGLutProcAddress(hDLL, 'glutChangeToMenuEntry');
    @OLD_glutChangeToSubMenu := GetGLutProcAddress(hDLL, 'glutChangeToSubMenu');
    @OLD_glutRemoveMenuItem := GetGLutProcAddress(hDLL, 'glutRemoveMenuItem');
    @OLD_glutAttachMenu := GetGLutProcAddress(hDLL, 'glutAttachMenu');
    @OLD_glutDetachMenu := GetGLutProcAddress(hDLL, 'glutDetachMenu');
    @OLD_glutDisplayFunc := GetGLutProcAddress(hDLL, 'glutDisplayFunc');
    @OLD_glutReshapeFunc := GetGLutProcAddress(hDLL, 'glutReshapeFunc');
    @OLD_glutKeyboardFunc := GetGLutProcAddress(hDLL, 'glutKeyboardFunc');
    @OLD_glutMouseFunc := GetGLutProcAddress(hDLL, 'glutMouseFunc');
    @OLD_glutMotionFunc := GetGLutProcAddress(hDLL, 'glutMotionFunc');
    @OLD_glutPassiveMotionFunc := GetGLutProcAddress(hDLL, 'glutPassiveMotionFunc');
    @OLD_glutEntryFunc := GetGLutProcAddress(hDLL, 'glutEntryFunc');
    @OLD_glutVisibilityFunc := GetGLutProcAddress(hDLL, 'glutVisibilityFunc');
    @OLD_glutIdleFunc := GetGLutProcAddress(hDLL, 'glutIdleFunc');
    @OLD_glutTimerFunc := GetGLutProcAddress(hDLL, 'glutTimerFunc');
    @OLD_glutMenuStateFunc := GetGLutProcAddress(hDLL, 'glutMenuStateFunc');
    @OLD_glutSpecialFunc := GetGLutProcAddress(hDLL, 'glutSpecialFunc');
    @OLD_glutSpaceballMotionFunc := GetGLutProcAddress(hDLL, 'glutSpaceballMotionFunc');
    @OLD_glutSpaceballRotateFunc := GetGLutProcAddress(hDLL, 'glutSpaceballRotateFunc');
    @OLD_glutSpaceballButtonFunc := GetGLutProcAddress(hDLL, 'glutSpaceballButtonFunc');
    @OLD_glutButtonBoxFunc := GetGLutProcAddress(hDLL, 'glutButtonBoxFunc');
    @OLD_glutDialsFunc := GetGLutProcAddress(hDLL, 'glutDialsFunc');
    @OLD_glutTabletMotionFunc := GetGLutProcAddress(hDLL, 'glutTabletMotionFunc');
    @OLD_glutTabletButtonFunc := GetGLutProcAddress(hDLL, 'glutTabletButtonFunc');
    @OLD_glutMenuStatusFunc := GetGLutProcAddress(hDLL, 'glutMenuStatusFunc');
    @OLD_glutOverlayDisplayFunc := GetGLutProcAddress(hDLL, 'glutOverlayDisplayFunc');
    @OLD_glutWindowStatusFunc := GetGLutProcAddress(hDLL, 'glutWindowStatusFunc');
    @OLD_glutKeyboardUpFunc := GetGLutProcAddress(hDLL, 'glutKeyboardUpFunc');
    @OLD_glutSpecialUpFunc := GetGLutProcAddress(hDLL, 'glutSpecialUpFunc');
    @OLD_glutJoystickFunc := GetGLutProcAddress(hDLL, 'glutJoystickFunc');
    @OLD_glutSetColor := GetGLutProcAddress(hDLL, 'glutSetColor');
    @OLD_glutGetColor := GetGLutProcAddress(hDLL, 'glutGetColor');
    @OLD_glutCopyColormap := GetGLutProcAddress(hDLL, 'glutCopyColormap');
    @OLD_glutGet := GetGLutProcAddress(hDLL, 'glutGet');
    @OLD_glutDeviceGet := GetGLutProcAddress(hDLL, 'glutDeviceGet');
    @OLD_glutExtensionSupported := GetGLutProcAddress(hDLL, 'glutExtensionSupported');
    @OLD_glutGetModifiers := GetGLutProcAddress(hDLL, 'glutGetModifiers');
    @OLD_glutLayerGet := GetGLutProcAddress(hDLL, 'glutLayerGet');
    @OLD_glutBitmapCharacter := GetGLutProcAddress(hDLL, 'glutBitmapCharacter');
    @OLD_glutBitmapWidth := GetGLutProcAddress(hDLL, 'glutBitmapWidth');
    @OLD_glutStrokeCharacter := GetGLutProcAddress(hDLL, 'glutStrokeCharacter');
    @OLD_glutStrokeWidth := GetGLutProcAddress(hDLL, 'glutStrokeWidth');
    @OLD_glutBitmapLength := GetGLutProcAddress(hDLL, 'glutBitmapLength');
    @OLD_glutStrokeLength := GetGLutProcAddress(hDLL, 'glutStrokeLength');
    @OLD_glutWireSphere := GetGLutProcAddress(hDLL, 'glutWireSphere');
    @OLD_glutSolidSphere := GetGLutProcAddress(hDLL, 'glutSolidSphere');
    @OLD_glutWireCone := GetGLutProcAddress(hDLL, 'glutWireCone');
    @OLD_glutSolidCone := GetGLutProcAddress(hDLL, 'glutSolidCone');
    @OLD_glutWireCube := GetGLutProcAddress(hDLL, 'glutWireCube');
    @OLD_glutSolidCube := GetGLutProcAddress(hDLL, 'glutSolidCube');
    @OLD_glutWireTorus := GetGLutProcAddress(hDLL, 'glutWireTorus');
    @OLD_glutSolidTorus := GetGLutProcAddress(hDLL, 'glutSolidTorus');
    @OLD_glutWireDodecahedron := GetGLutProcAddress(hDLL, 'glutWireDodecahedron');
    @OLD_glutSolidDodecahedron := GetGLutProcAddress(hDLL, 'glutSolidDodecahedron');
    @OLD_glutWireTeapot := GetGLutProcAddress(hDLL, 'glutWireTeapot');
    @OLD_glutSolidTeapot := GetGLutProcAddress(hDLL, 'glutSolidTeapot');
    @OLD_glutWireOctahedron := GetGLutProcAddress(hDLL, 'glutWireOctahedron');
    @OLD_glutSolidOctahedron := GetGLutProcAddress(hDLL, 'glutSolidOctahedron');
    @OLD_glutWireTetrahedron := GetGLutProcAddress(hDLL, 'glutWireTetrahedron');
    @OLD_glutSolidTetrahedron := GetGLutProcAddress(hDLL, 'glutSolidTetrahedron');
    @OLD_glutWireIcosahedron := GetGLutProcAddress(hDLL, 'glutWireIcosahedron');
    @OLD_glutSolidIcosahedron := GetGLutProcAddress(hDLL, 'glutSolidIcosahedron');
    @OLD_glutVideoResizeGet := GetGLutProcAddress(hDLL, 'glutVideoResizeGet');
    @OLD_glutSetupVideoResizing := GetGLutProcAddress(hDLL, 'glutSetupVideoResizing');
    @OLD_glutStopVideoResizing := GetGLutProcAddress(hDLL, 'glutStopVideoResizing');
    @OLD_glutVideoResize := GetGLutProcAddress(hDLL, 'glutVideoResize');
    @OLD_glutVideoPan := GetGLutProcAddress(hDLL, 'glutVideoPan');
    @OLD_glutReportErrors := GetGLutProcAddress(hDLL, 'glutReportErrors');
    @OLD_glutIgnoreKeyRepeat := GetGLutProcAddress(hDLL, 'glutIgnoreKeyRepeat');
    @OLD_glutSetKeyRepeat := GetGLutProcAddress(hDLL, 'glutSetKeyRepeat');
    @OLD_glutForceJoystickFunc := GetGLutProcAddress(hDLL, 'glutForceJoystickFunc');
    @OLD_glutGameModeString := GetGLutProcAddress(hDLL, 'glutGameModeString');
    @OLD_glutEnterGameMode := GetGLutProcAddress(hDLL, 'glutEnterGameMode');
    @OLD_glutLeaveGameMode := GetGLutProcAddress(hDLL, 'glutLeaveGameMode');
    @OLD_glutGameModeGet := GetGLutProcAddress(hDLL, 'glutGameModeGet');
{$ELSE GLUT_EXCLUSIVE_FPUMODE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    @glutInit := GetGLutProcAddress(hDLL, 'glutInit');
    @glutInitDisplayMode := GetGLutProcAddress(hDLL, 'glutInitDisplayMode');
    @glutInitDisplayString := GetGLutProcAddress(hDLL, 'glutInitDisplayString');
    @glutInitWindowPosition := GetGLutProcAddress(hDLL, 'glutInitWindowPosition');
    @glutInitWindowSize := GetGLutProcAddress(hDLL, 'glutInitWindowSize');
    @glutMainLoop := GetGLutProcAddress(hDLL, 'glutMainLoop');
    @glutCreateWindow := GetGLutProcAddress(hDLL, 'glutCreateWindow');
    @glutCreateSubWindow := GetGLutProcAddress(hDLL, 'glutCreateSubWindow');
    @glutDestroyWindow := GetGLutProcAddress(hDLL, 'glutDestroyWindow');
    @glutPostRedisplay := GetGLutProcAddress(hDLL, 'glutPostRedisplay');
    @glutPostWindowRedisplay := GetGLutProcAddress(hDLL, 'glutPostWindowRedisplay');
    @glutSwapBuffers := GetGLutProcAddress(hDLL, 'glutSwapBuffers');
    @glutGetWindow := GetGLutProcAddress(hDLL, 'glutGetWindow');
    @glutSetWindow := GetGLutProcAddress(hDLL, 'glutSetWindow');
    @glutSetWindowTitle := GetGLutProcAddress(hDLL, 'glutSetWindowTitle');
    @glutSetIconTitle := GetGLutProcAddress(hDLL, 'glutSetIconTitle');
    @glutPositionWindow := GetGLutProcAddress(hDLL, 'glutPositionWindow');
    @glutReshapeWindow := GetGLutProcAddress(hDLL, 'glutReshapeWindow');
    @glutPopWindow := GetGLutProcAddress(hDLL, 'glutPopWindow');
    @glutPushWindow := GetGLutProcAddress(hDLL, 'glutPushWindow');
    @glutIconifyWindow := GetGLutProcAddress(hDLL, 'glutIconifyWindow');
    @glutShowWindow := GetGLutProcAddress(hDLL, 'glutShowWindow');
    @glutHideWindow := GetGLutProcAddress(hDLL, 'glutHideWindow');
    @glutFullScreen := GetGLutProcAddress(hDLL, 'glutFullScreen');
    @glutSetCursor := GetGLutProcAddress(hDLL, 'glutSetCursor');
    @glutWarpPointer := GetGLutProcAddress(hDLL, 'glutWarpPointer');
    @glutEstablishOverlay := GetGLutProcAddress(hDLL, 'glutEstablishOverlay');
    @glutRemoveOverlay := GetGLutProcAddress(hDLL, 'glutRemoveOverlay');
    @glutUseLayer := GetGLutProcAddress(hDLL, 'glutUseLayer');
    @glutPostOverlayRedisplay := GetGLutProcAddress(hDLL, 'glutPostOverlayRedisplay');
    @glutPostWindowOverlayRedisplay := GetGLutProcAddress(hDLL, 'glutPostWindowOverlayRedisplay');
    @glutShowOverlay := GetGLutProcAddress(hDLL, 'glutShowOverlay');
    @glutHideOverlay := GetGLutProcAddress(hDLL, 'glutHideOverlay');
    @glutCreateMenu := GetGLutProcAddress(hDLL, 'glutCreateMenu');
    @glutDestroyMenu := GetGLutProcAddress(hDLL, 'glutDestroyMenu');
    @glutGetMenu := GetGLutProcAddress(hDLL, 'glutGetMenu');
    @glutSetMenu := GetGLutProcAddress(hDLL, 'glutSetMenu');
    @glutAddMenuEntry := GetGLutProcAddress(hDLL, 'glutAddMenuEntry');
    @glutAddSubMenu := GetGLutProcAddress(hDLL, 'glutAddSubMenu');
    @glutChangeToMenuEntry := GetGLutProcAddress(hDLL, 'glutChangeToMenuEntry');
    @glutChangeToSubMenu := GetGLutProcAddress(hDLL, 'glutChangeToSubMenu');
    @glutRemoveMenuItem := GetGLutProcAddress(hDLL, 'glutRemoveMenuItem');
    @glutAttachMenu := GetGLutProcAddress(hDLL, 'glutAttachMenu');
    @glutDetachMenu := GetGLutProcAddress(hDLL, 'glutDetachMenu');
    @glutDisplayFunc := GetGLutProcAddress(hDLL, 'glutDisplayFunc');
    @glutReshapeFunc := GetGLutProcAddress(hDLL, 'glutReshapeFunc');
    @glutKeyboardFunc := GetGLutProcAddress(hDLL, 'glutKeyboardFunc');
    @glutMouseFunc := GetGLutProcAddress(hDLL, 'glutMouseFunc');
    @glutMotionFunc := GetGLutProcAddress(hDLL, 'glutMotionFunc');
    @glutPassiveMotionFunc := GetGLutProcAddress(hDLL, 'glutPassiveMotionFunc');
    @glutEntryFunc := GetGLutProcAddress(hDLL, 'glutEntryFunc');
    @glutVisibilityFunc := GetGLutProcAddress(hDLL, 'glutVisibilityFunc');
    @glutIdleFunc := GetGLutProcAddress(hDLL, 'glutIdleFunc');
    @glutTimerFunc := GetGLutProcAddress(hDLL, 'glutTimerFunc');
    @glutMenuStateFunc := GetGLutProcAddress(hDLL, 'glutMenuStateFunc');
    @glutSpecialFunc := GetGLutProcAddress(hDLL, 'glutSpecialFunc');
    @glutSpaceballMotionFunc := GetGLutProcAddress(hDLL, 'glutSpaceballMotionFunc');
    @glutSpaceballRotateFunc := GetGLutProcAddress(hDLL, 'glutSpaceballRotateFunc');
    @glutSpaceballButtonFunc := GetGLutProcAddress(hDLL, 'glutSpaceballButtonFunc');
    @glutButtonBoxFunc := GetGLutProcAddress(hDLL, 'glutButtonBoxFunc');
    @glutDialsFunc := GetGLutProcAddress(hDLL, 'glutDialsFunc');
    @glutTabletMotionFunc := GetGLutProcAddress(hDLL, 'glutTabletMotionFunc');
    @glutTabletButtonFunc := GetGLutProcAddress(hDLL, 'glutTabletButtonFunc');
    @glutMenuStatusFunc := GetGLutProcAddress(hDLL, 'glutMenuStatusFunc');
    @glutOverlayDisplayFunc := GetGLutProcAddress(hDLL, 'glutOverlayDisplayFunc');
    @glutWindowStatusFunc := GetGLutProcAddress(hDLL, 'glutWindowStatusFunc');
    @glutKeyboardUpFunc := GetGLutProcAddress(hDLL, 'glutKeyboardUpFunc');
    @glutSpecialUpFunc := GetGLutProcAddress(hDLL, 'glutSpecialUpFunc');
    @glutJoystickFunc := GetGLutProcAddress(hDLL, 'glutJoystickFunc');
    @glutSetColor := GetGLutProcAddress(hDLL, 'glutSetColor');
    @glutGetColor := GetGLutProcAddress(hDLL, 'glutGetColor');
    @glutCopyColormap := GetGLutProcAddress(hDLL, 'glutCopyColormap');
    @glutGet := GetGLutProcAddress(hDLL, 'glutGet');
    @glutDeviceGet := GetGLutProcAddress(hDLL, 'glutDeviceGet');
    @glutExtensionSupported := GetGLutProcAddress(hDLL, 'glutExtensionSupported');
    @glutGetModifiers := GetGLutProcAddress(hDLL, 'glutGetModifiers');
    @glutLayerGet := GetGLutProcAddress(hDLL, 'glutLayerGet');
    @glutBitmapCharacter := GetGLutProcAddress(hDLL, 'glutBitmapCharacter');
    @glutBitmapWidth := GetGLutProcAddress(hDLL, 'glutBitmapWidth');
    @glutStrokeCharacter := GetGLutProcAddress(hDLL, 'glutStrokeCharacter');
    @glutStrokeWidth := GetGLutProcAddress(hDLL, 'glutStrokeWidth');
    @glutBitmapLength := GetGLutProcAddress(hDLL, 'glutBitmapLength');
    @glutStrokeLength := GetGLutProcAddress(hDLL, 'glutStrokeLength');
    @glutWireSphere := GetGLutProcAddress(hDLL, 'glutWireSphere');
    @glutSolidSphere := GetGLutProcAddress(hDLL, 'glutSolidSphere');
    @glutWireCone := GetGLutProcAddress(hDLL, 'glutWireCone');
    @glutSolidCone := GetGLutProcAddress(hDLL, 'glutSolidCone');
    @glutWireCube := GetGLutProcAddress(hDLL, 'glutWireCube');
    @glutSolidCube := GetGLutProcAddress(hDLL, 'glutSolidCube');
    @glutWireTorus := GetGLutProcAddress(hDLL, 'glutWireTorus');
    @glutSolidTorus := GetGLutProcAddress(hDLL, 'glutSolidTorus');
    @glutWireDodecahedron := GetGLutProcAddress(hDLL, 'glutWireDodecahedron');
    @glutSolidDodecahedron := GetGLutProcAddress(hDLL, 'glutSolidDodecahedron');
    @glutWireTeapot := GetGLutProcAddress(hDLL, 'glutWireTeapot');
    @glutSolidTeapot := GetGLutProcAddress(hDLL, 'glutSolidTeapot');
    @glutWireOctahedron := GetGLutProcAddress(hDLL, 'glutWireOctahedron');
    @glutSolidOctahedron := GetGLutProcAddress(hDLL, 'glutSolidOctahedron');
    @glutWireTetrahedron := GetGLutProcAddress(hDLL, 'glutWireTetrahedron');
    @glutSolidTetrahedron := GetGLutProcAddress(hDLL, 'glutSolidTetrahedron');
    @glutWireIcosahedron := GetGLutProcAddress(hDLL, 'glutWireIcosahedron');
    @glutSolidIcosahedron := GetGLutProcAddress(hDLL, 'glutSolidIcosahedron');
    @glutVideoResizeGet := GetGLutProcAddress(hDLL, 'glutVideoResizeGet');
    @glutSetupVideoResizing := GetGLutProcAddress(hDLL, 'glutSetupVideoResizing');
    @glutStopVideoResizing := GetGLutProcAddress(hDLL, 'glutStopVideoResizing');
    @glutVideoResize := GetGLutProcAddress(hDLL, 'glutVideoResize');
    @glutVideoPan := GetGLutProcAddress(hDLL, 'glutVideoPan');
    @glutReportErrors := GetGLutProcAddress(hDLL, 'glutReportErrors');
    @glutIgnoreKeyRepeat := GetGLutProcAddress(hDLL, 'glutIgnoreKeyRepeat');
    @glutSetKeyRepeat := GetGLutProcAddress(hDLL, 'glutSetKeyRepeat');
    @glutForceJoystickFunc := GetGLutProcAddress(hDLL, 'glutForceJoystickFunc');
    @glutGameModeString := GetGLutProcAddress(hDLL, 'glutGameModeString');
    @glutEnterGameMode := GetGLutProcAddress(hDLL, 'glutEnterGameMode');
    @glutLeaveGameMode := GetGLutProcAddress(hDLL, 'glutLeaveGameMode');
    @glutGameModeGet := GetGLutProcAddress(hDLL, 'glutGameModeGet');
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ENDIF GLUT_EXCLUSIVE_FPUMODE}

>>>>>>> graemeg/fixes_2_2
=======
{$ENDIF GLUT_EXCLUSIVE_FPUMODE}

>>>>>>> origin/fixes_2_2
{$ifndef Windows}
    GLUT_STROKE_ROMAN := GetGLutProcAddress(hDll, 'glutStrokeRoman');
    GLUT_STROKE_MONO_ROMAN := GetGLutProcAddress(hDll,'glutStrokeMonoRoman');
    GLUT_BITMAP_9_BY_15 := GetGLutProcAddress(hDll, 'glutBitmap9By15');
    GLUT_BITMAP_8_BY_13 := GetGLutProcAddress(hDll, 'glutBitmap8By13');
    GLUT_BITMAP_TIMES_ROMAN_10 := GetGLutProcAddress(hDll, 'glutBitmapTimesRoman10');
    GLUT_BITMAP_TIMES_ROMAN_24 := GetGLutProcAddress(hDll, 'glutBitmapTimesRoman24');
    GLUT_BITMAP_HELVETICA_10 := GetGLutProcAddress(hDll, 'glutBitmapHelvetica10');
    GLUT_BITMAP_HELVETICA_12 := GetGLutProcAddress(hDll, 'glutBitmapHelvetica12');
    GLUT_BITMAP_HELVETICA_18 := GetGLutProcAddress(hDll, 'glutBitmapHelvetica18');
{$endif Windows}
  except
    raise Exception.Create('Could not load ' + MethodName + ' from ' + dll);
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  LoadFreeGlut(hDLL);
end;
{$ENDIF MORPHOS}


initialization

  {$IFDEF Windows}
  LoadGlut('glut32.dll');
  {$ELSE}
  {$IFDEF OS2}
  LoadGlut('glut.dll');
  {$ELSE OS2}
  {$ifdef darwin}
  LoadGlut('/System/Library/Frameworks/GLUT.framework/GLUT');
  {$else}
  {$IFDEF haiku}
  LoadGlut('libglut.so');
  {$ELSE}
  {$IFNDEF MORPHOS}
  LoadGlut('libglut.so.3');
  {$ENDIF}
  {$ENDIF}
  {$endif}
  {$ENDIF OS2}
  {$ENDIF}

finalization

  UnloadGlut;

=======
=======
>>>>>>> origin/fixes_2_2
end;
{$ENDIF MORPHOS}

{$IFDEF GLUT_EXCLUSIVE_FPUMODE}

VAR
GLUT_EXCLUSIVE_glut_ExceptionMask : TFPUExceptionMask;
GLUT_EXCLUSIVE_fpc__ExceptionMask : TFPUExceptionMask;
GLUT_EXCLUSIVE_glut_PrecisionMode : TFPUPrecisionMode;
GLUT_EXCLUSIVE_fpc__PrecisionMode : TFPUPrecisionMode;
GLUT_EXCLUSIVE_glut_RoundMode : TFPURoundingMode;
GLUT_EXCLUSIVE_fpc__RoundMode : TFPURoundingMode;


PROCEDURE init_fpumode;
BEGIN
  GLUT_EXCLUSIVE_glut_ExceptionMask := GetExceptionMask+[exDenormalized, exInvalidOp, exOverflow, exPrecision, exUnderflow, exZeroDivide];
  GLUT_EXCLUSIVE_glut_PrecisionMode := GetPrecisionMode;
  GLUT_EXCLUSIVE_glut_RoundMode     := GetRoundMode;
END;


PROCEDURE switch_to_glut_fpumode; mode_inline;
BEGIN
  GLUT_EXCLUSIVE_fpc__ExceptionMask := GetExceptionMask;
  GLUT_EXCLUSIVE_fpc__PrecisionMode := GetPrecisionMode;
  GLUT_EXCLUSIVE_fpc__RoundMode     := GetRoundMode;
  SetExceptionMask(GLUT_EXCLUSIVE_glut_ExceptionMask);
  SetPrecisionMode(GLUT_EXCLUSIVE_glut_PrecisionMode);
  SetRoundMode(GLUT_EXCLUSIVE_glut_RoundMode);
END;

PROCEDURE switch_to_FPC_fpumode; mode_inline;
BEGIN
  GLUT_EXCLUSIVE_glut_ExceptionMask := GetExceptionMask;
  GLUT_EXCLUSIVE_glut_PrecisionMode := GetPrecisionMode;
  GLUT_EXCLUSIVE_glut_RoundMode     := GetRoundMode;
  SetExceptionMask(GLUT_EXCLUSIVE_fpc__ExceptionMask);
  SetPrecisionMode(GLUT_EXCLUSIVE_fpc__PrecisionMode);
  SetRoundMode(GLUT_EXCLUSIVE_fpc__RoundMode);
END;


procedure glutInit(argcp: PInteger; argv: PPChar);
begin
  switch_to_glut_fpumode;
  OLD_glutInit(argcp,argv);
  switch_to_FPC_fpumode;
end;

procedure glutInitDisplayMode(mode: Cardinal);
begin
  switch_to_glut_fpumode;
  OLD_glutInitDisplayMode(mode);
  switch_to_FPC_fpumode;
end;

procedure glutInitDisplayString(const str: PChar);
begin
  switch_to_glut_fpumode;
  OLD_glutInitDisplayString(str);
  switch_to_FPC_fpumode;
end;

procedure glutInitWindowPosition(x, y: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutInitWindowPosition(x,y);
  switch_to_FPC_fpumode;
end;

procedure glutInitWindowSize(width, height: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutInitWindowSize(width,height);
  switch_to_FPC_fpumode;
end;

procedure glutMainLoop;
begin
  switch_to_glut_fpumode;
  OLD_glutMainLoop();
  switch_to_FPC_fpumode;
end;

function glutCreateWindow(const title: PChar): Integer;
begin
  switch_to_glut_fpumode;
  glutCreateWindow := OLD_glutCreateWindow(title);
  switch_to_FPC_fpumode;
end;

function glutCreateSubWindow(win, x, y, width, height: Integer): Integer;
begin
  switch_to_glut_fpumode;
  glutCreateSubWindow := OLD_glutCreateSubWindow(win,x,y,width,height);
  switch_to_FPC_fpumode;
end;

procedure glutDestroyWindow(win: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutDestroyWindow(win);
  switch_to_FPC_fpumode;
end;

procedure glutPostRedisplay;
begin
  switch_to_glut_fpumode;
  OLD_glutPostRedisplay();
  switch_to_FPC_fpumode;
end;

procedure glutPostWindowRedisplay(win: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutPostWindowRedisplay(win);
  switch_to_FPC_fpumode;
end;

procedure glutSwapBuffers;
begin
  switch_to_glut_fpumode;
  OLD_glutSwapBuffers();
  switch_to_FPC_fpumode;
end;

function glutGetWindow: Integer;
begin
  switch_to_glut_fpumode;
  glutGetWindow := OLD_glutGetWindow();
  switch_to_FPC_fpumode;
end;

procedure glutSetWindow(win: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutSetWindow(win);
  switch_to_FPC_fpumode;
end;

procedure glutSetWindowTitle(const title: PChar);
begin
  switch_to_glut_fpumode;
  OLD_glutSetWindowTitle(title);
  switch_to_FPC_fpumode;
end;

procedure glutSetIconTitle(const title: PChar);
begin
  switch_to_glut_fpumode;
  OLD_glutSetIconTitle(title);
  switch_to_FPC_fpumode;
end;

procedure glutPositionWindow(x, y: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutPositionWindow(x,y);
  switch_to_FPC_fpumode;
end;

procedure glutReshapeWindow(width, height: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutReshapeWindow(width,height);
  switch_to_FPC_fpumode;
end;

procedure glutPopWindow;
begin
  switch_to_glut_fpumode;
  OLD_glutPopWindow();
  switch_to_FPC_fpumode;
end;

procedure glutPushWindow;
begin
  switch_to_glut_fpumode;
  OLD_glutPushWindow();
  switch_to_FPC_fpumode;
end;

procedure glutIconifyWindow;
begin
  switch_to_glut_fpumode;
  OLD_glutIconifyWindow();
  switch_to_FPC_fpumode;
end;

procedure glutShowWindow;
begin
  switch_to_glut_fpumode;
  OLD_glutShowWindow();
  switch_to_FPC_fpumode;
end;

procedure glutHideWindow;
begin
  switch_to_glut_fpumode;
  OLD_glutHideWindow();
  switch_to_FPC_fpumode;
end;

procedure glutFullScreen;
begin
  switch_to_glut_fpumode;
  OLD_glutFullScreen();
  switch_to_FPC_fpumode;
end;

procedure glutSetCursor(cursor: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutSetCursor(cursor);
  switch_to_FPC_fpumode;
end;

procedure glutWarpPointer(x, y: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutWarpPointer(x,y);
  switch_to_FPC_fpumode;
end;

procedure glutEstablishOverlay;
begin
  switch_to_glut_fpumode;
  OLD_glutEstablishOverlay();
  switch_to_FPC_fpumode;
end;

procedure glutRemoveOverlay;
begin
  switch_to_glut_fpumode;
  OLD_glutRemoveOverlay();
  switch_to_FPC_fpumode;
end;

procedure glutUseLayer(layer: GLenum);
begin
  switch_to_glut_fpumode;
  OLD_glutUseLayer(layer);
  switch_to_FPC_fpumode;
end;

procedure glutPostOverlayRedisplay;
begin
  switch_to_glut_fpumode;
  OLD_glutPostOverlayRedisplay();
  switch_to_FPC_fpumode;
end;

procedure glutPostWindowOverlayRedisplay(win: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutPostWindowOverlayRedisplay(win);
  switch_to_FPC_fpumode;
end;

procedure glutShowOverlay;
begin
  switch_to_glut_fpumode;
  OLD_glutShowOverlay();
  switch_to_FPC_fpumode;
end;

procedure glutHideOverlay;
begin
  switch_to_glut_fpumode;
  OLD_glutHideOverlay();
  switch_to_FPC_fpumode;
end;

function glutCreateMenu(callback: TGlut1IntCallback): Integer;
begin
  switch_to_glut_fpumode;
  glutCreateMenu := OLD_glutCreateMenu(callback);
  switch_to_FPC_fpumode;
end;

procedure glutDestroyMenu(menu: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutDestroyMenu(menu);
  switch_to_FPC_fpumode;
end;

function glutGetMenu: Integer;
begin
  switch_to_glut_fpumode;
  glutGetMenu := OLD_glutGetMenu();
  switch_to_FPC_fpumode;
end;

procedure glutSetMenu(menu: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutSetMenu(menu);
  switch_to_FPC_fpumode;
end;

procedure glutAddMenuEntry(const caption: PChar; value: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutAddMenuEntry(caption,value);
  switch_to_FPC_fpumode;
end;

procedure glutAddSubMenu(const caption: PChar; submenu: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutAddSubMenu(caption,submenu);
  switch_to_FPC_fpumode;
end;

procedure glutChangeToMenuEntry(item: Integer; const caption: PChar; value: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutChangeToMenuEntry(item,caption,value);
  switch_to_FPC_fpumode;
end;

procedure glutChangeToSubMenu(item: Integer; const caption: PChar; submenu: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutChangeToSubMenu(item,caption,submenu);
  switch_to_FPC_fpumode;
end;

procedure glutRemoveMenuItem(item: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutRemoveMenuItem(item);
  switch_to_FPC_fpumode;
end;

procedure glutAttachMenu(button: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutAttachMenu(button);
  switch_to_FPC_fpumode;
end;

procedure glutDetachMenu(button: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutDetachMenu(button);
  switch_to_FPC_fpumode;
end;

procedure glutDisplayFunc(f: TGlutVoidCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutDisplayFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutReshapeFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutReshapeFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutKeyboardFunc(f: TGlut1Char2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutKeyboardFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutMouseFunc(f: TGlut4IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutMouseFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutMotionFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutMotionFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutPassiveMotionFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutPassiveMotionFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutEntryFunc(f: TGlut1IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutEntryFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutVisibilityFunc(f: TGlut1IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutVisibilityFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutIdleFunc(f: TGlutVoidCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutIdleFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutTimerFunc(millis: Cardinal; f: TGlut1IntCallback; value: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutTimerFunc(millis,f,value);
  switch_to_FPC_fpumode;
end;

procedure glutMenuStateFunc(f: TGlut1IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutMenuStateFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutSpecialFunc(f: TGlut3IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutSpecialFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutSpaceballMotionFunc(f: TGlut3IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutSpaceballMotionFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutSpaceballRotateFunc(f: TGlut3IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutSpaceballRotateFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutSpaceballButtonFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutSpaceballButtonFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutButtonBoxFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutButtonBoxFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutDialsFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutDialsFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutTabletMotionFunc(f: TGlut2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutTabletMotionFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutTabletButtonFunc(f: TGlut4IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutTabletButtonFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutMenuStatusFunc(f: TGlut3IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutMenuStatusFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutOverlayDisplayFunc(f:TGlutVoidCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutOverlayDisplayFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutWindowStatusFunc(f: TGlut1IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutWindowStatusFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutKeyboardUpFunc(f: TGlut1Char2IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutKeyboardUpFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutSpecialUpFunc(f: TGlut3IntCallback);
begin
  switch_to_glut_fpumode;
  OLD_glutSpecialUpFunc(f);
  switch_to_FPC_fpumode;
end;

procedure glutJoystickFunc(f: TGlut1UInt3IntCallback; pollInterval: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutJoystickFunc(f,pollInterval);
  switch_to_FPC_fpumode;
end;

procedure glutSetColor(cell: Integer; red, green, blue: GLfloat);
begin
  switch_to_glut_fpumode;
  OLD_glutSetColor(cell,red,green,blue);
  switch_to_FPC_fpumode;
end;

function glutGetColor(ndx, component: Integer): GLfloat;
begin
  switch_to_glut_fpumode;
  glutGetColor := OLD_glutGetColor(ndx,component);
  switch_to_FPC_fpumode;
end;

procedure glutCopyColormap(win: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutCopyColormap(win);
  switch_to_FPC_fpumode;
end;

function glutGet(t: GLenum): Integer;
begin
  switch_to_glut_fpumode;
  glutGet := OLD_glutGet(t);
  switch_to_FPC_fpumode;
end;

function glutDeviceGet(t: GLenum): Integer;
begin
  switch_to_glut_fpumode;
  glutDeviceGet := OLD_glutDeviceGet(t);
  switch_to_FPC_fpumode;
end;

function glutExtensionSupported(const name: PChar): Integer;
begin
  switch_to_glut_fpumode;
  glutExtensionSupported := OLD_glutExtensionSupported(name);
  switch_to_FPC_fpumode;
end;

function glutGetModifiers: Integer;
begin
  switch_to_glut_fpumode;
  glutGetModifiers := OLD_glutGetModifiers();
  switch_to_FPC_fpumode;
end;

function glutLayerGet(t: GLenum): Integer;
begin
  switch_to_glut_fpumode;
  glutLayerGet := OLD_glutLayerGet(t);
  switch_to_FPC_fpumode;
end;

procedure glutBitmapCharacter(font : pointer; character: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutBitmapCharacter(font,character);
  switch_to_FPC_fpumode;
end;

function glutBitmapWidth(font : pointer; character: Integer): Integer;
begin
  switch_to_glut_fpumode;
  glutBitmapWidth := OLD_glutBitmapWidth(font,character);
  switch_to_FPC_fpumode;
end;

procedure glutStrokeCharacter(font : pointer; character: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutStrokeCharacter(font,character);
  switch_to_FPC_fpumode;
end;

function glutStrokeWidth(font : pointer; character: Integer): Integer;
begin
  switch_to_glut_fpumode;
  glutStrokeWidth := OLD_glutStrokeWidth(font,character);
  switch_to_FPC_fpumode;
end;

function glutBitmapLength(font: pointer; const str: PChar): Integer;
begin
  switch_to_glut_fpumode;
  glutBitmapLength := OLD_glutBitmapLength(font,str);
  switch_to_FPC_fpumode;
end;

function glutStrokeLength(font: pointer; const str: PChar): Integer;
begin
  switch_to_glut_fpumode;
  glutStrokeLength := OLD_glutStrokeLength(font,str);
  switch_to_FPC_fpumode;
end;

procedure glutWireSphere(radius: GLdouble; slices, stacks: GLint);
begin
  switch_to_glut_fpumode;
  OLD_glutWireSphere(radius,slices,stacks);
  switch_to_FPC_fpumode;
end;

procedure glutSolidSphere(radius: GLdouble; slices, stacks: GLint);
begin
  switch_to_glut_fpumode;
  OLD_glutSolidSphere(radius,slices,stacks);
  switch_to_FPC_fpumode;
end;

procedure glutWireCone(base, height: GLdouble; slices, stacks: GLint);
begin
  switch_to_glut_fpumode;
  OLD_glutWireCone(base,height,slices,stacks);
  switch_to_FPC_fpumode;
end;

procedure glutSolidCone(base, height: GLdouble; slices, stacks: GLint);
begin
  switch_to_glut_fpumode;
  OLD_glutSolidCone(base,height,slices,stacks);
  switch_to_FPC_fpumode;
end;

procedure glutWireCube(size: GLdouble);
begin
  switch_to_glut_fpumode;
  OLD_glutWireCube(size);
  switch_to_FPC_fpumode;
end;

procedure glutSolidCube(size: GLdouble);
begin
  switch_to_glut_fpumode;
  OLD_glutSolidCube(size);
  switch_to_FPC_fpumode;
end;

procedure glutWireTorus(innerRadius, outerRadius: GLdouble; sides, rings: GLint);
begin
  switch_to_glut_fpumode;
  OLD_glutWireTorus(innerRadius,outerRadius,sides,rings);
  switch_to_FPC_fpumode;
end;

procedure glutSolidTorus(innerRadius, outerRadius: GLdouble; sides, rings: GLint);
begin
  switch_to_glut_fpumode;
  OLD_glutSolidTorus(innerRadius,outerRadius,sides,rings);
  switch_to_FPC_fpumode;
end;

procedure glutWireDodecahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutWireDodecahedron();
  switch_to_FPC_fpumode;
end;

procedure glutSolidDodecahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutSolidDodecahedron();
  switch_to_FPC_fpumode;
end;

procedure glutWireTeapot(size: GLdouble);
begin
  switch_to_glut_fpumode;
  OLD_glutWireTeapot(size);
  switch_to_FPC_fpumode;
end;

procedure glutSolidTeapot(size: GLdouble);
begin
  switch_to_glut_fpumode;
  OLD_glutSolidTeapot(size);
  switch_to_FPC_fpumode;
end;

procedure glutWireOctahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutWireOctahedron();
  switch_to_FPC_fpumode;
end;

procedure glutSolidOctahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutSolidOctahedron();
  switch_to_FPC_fpumode;
end;

procedure glutWireTetrahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutWireTetrahedron();
  switch_to_FPC_fpumode;
end;

procedure glutSolidTetrahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutSolidTetrahedron();
  switch_to_FPC_fpumode;
end;

procedure glutWireIcosahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutWireIcosahedron();
  switch_to_FPC_fpumode;
end;

procedure glutSolidIcosahedron;
begin
  switch_to_glut_fpumode;
  OLD_glutSolidIcosahedron();
  switch_to_FPC_fpumode;
end;

function glutVideoResizeGet(param: GLenum): Integer;
begin
  switch_to_glut_fpumode;
  glutVideoResizeGet := OLD_glutVideoResizeGet(param);
  switch_to_FPC_fpumode;
end;

procedure glutSetupVideoResizing;
begin
  switch_to_glut_fpumode;
  OLD_glutSetupVideoResizing();
  switch_to_FPC_fpumode;
end;

procedure glutStopVideoResizing;
begin
  switch_to_glut_fpumode;
  OLD_glutStopVideoResizing();
  switch_to_FPC_fpumode;
end;

procedure glutVideoResize(x, y, width, height: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutVideoResize(x,y,width,height);
  switch_to_FPC_fpumode;
end;

procedure glutVideoPan(x, y, width, height: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutVideoPan(x,y,width,height);
  switch_to_FPC_fpumode;
end;

procedure glutReportErrors;
begin
  switch_to_glut_fpumode;
  OLD_glutReportErrors();
  switch_to_FPC_fpumode;
end;

procedure glutIgnoreKeyRepeat(ignore: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutIgnoreKeyRepeat(ignore);
  switch_to_FPC_fpumode;
end;

procedure glutSetKeyRepeat(repeatMode: Integer);
begin
  switch_to_glut_fpumode;
  OLD_glutSetKeyRepeat(repeatMode);
  switch_to_FPC_fpumode;
end;

procedure glutForceJoystickFunc;
begin
  switch_to_glut_fpumode;
  OLD_glutForceJoystickFunc();
  switch_to_FPC_fpumode;
end;

procedure glutGameModeString(const AString : PChar);
begin
  switch_to_glut_fpumode;
  OLD_glutGameModeString(AString);
  switch_to_FPC_fpumode;
end;

function glutEnterGameMode: integer;
begin
  switch_to_glut_fpumode;
  glutEnterGameMode := OLD_glutEnterGameMode();
  switch_to_FPC_fpumode;
end;

procedure glutLeaveGameMode;
begin
  switch_to_glut_fpumode;
  OLD_glutLeaveGameMode();
  switch_to_FPC_fpumode;
end;

function glutGameModeGet(mode : GLenum): integer;
begin
  switch_to_glut_fpumode;
  glutGameModeGet := OLD_glutGameModeGet(mode);
  switch_to_FPC_fpumode;
end;


{$ENDIF GLUT_EXCLUSIVE_FPUMODE}

initialization

  {$IFDEF GLUT_EXCLUSIVE_FPUMODE}
  init_fpumode;
  {$ENDIF GLUT_EXCLUSIVE_FPUMODE}

  {$IFDEF Windows}
  LoadGlut('glut32.dll');
  {$ELSE}
  {$ifdef darwin}
  LoadGlut('/System/Library/Frameworks/GLUT.framework/GLUT');
  {$else}
  {$IFNDEF MORPHOS}
  LoadGlut('libglut.so.3');
  {$ENDIF}
  {$endif}
  {$ENDIF}

finalization
  FreeGlut;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
