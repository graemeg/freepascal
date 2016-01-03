{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{$MACRO ON}
{$DEFINE DXCall:=StdCall}
{$PACKRECORDS 1}

Unit DirectDraw;

Interface

Uses
  Windows;

Type
  LPGUID = ^GUID;
  GUID = Packed Record
    Data1 : DWord;
    Data2 : Word;
    Data3 : Word;
    Data4 : Array[0..7] Of Byte;
  End;

{;
; FOURCC codes for DX compressed-texture pixel formats
;

FOURCC_DXT1                     = '1TXD'
FOURCC_DXT2                     = '2TXD'
FOURCC_DXT3                     = '3TXD'
FOURCC_DXT4                     = '4TXD'
FOURCC_DXT5                     = '5TXD'

;
; GUIDS used by DirectDraw objects
;

macro   Define_CLSID_DirectDraw
public  CLSID_DirectDraw
CLSID_DirectDraw        GUID <0D7B70EE0h,04340h,011CFh,\
                        0B0h,063h,000h,020h,0AFh,0C2h,0CDh,035h>
endm

macro   Define_CLSID_DirectDrawClipper
public  CLSID_DirectDrawClipper
CLSID_DirectDrawClipper GUID <0593817A0h,07DB3h,011CFh,\
                        0A2h,0DEh,000h,0AAh,000h,0b9h,033h,056h>
endm

macro   Define_IID_IDirectDraw
public  IID_IDirectDraw
IID_IDirectDraw         GUID <06C14DB80h,0A733h,011CEh,\
                        0A5h,021h,000h,020h,0AFh,00Bh,0E5h,060h>
endm

macro   Define_IID_IDirectDraw2
public  IID_IDirectDraw2
IID_IDirectDraw2        GUID <0B3A6F3E0h,02B43h,011CFh,\
                        0A2h,0DEh,000h,0AAh,000h,0B9h,033h,056h>
endm

macro   Define_IID_IDirectDraw4
public  IID_IDirectDraw4
IID_IDirectDraw4        GUID <09c59509ah,039bdh,011d1h,\
                        08ch,04ah,000h,0c0h,04fh,0d9h,030h,0c5h>
endm

macro   Define_IID_IDirectDrawSurface
public  IID_IDirectDrawSurface
IID_IDirectDrawSurface  GUID <06C14DB81h,0A733h,011CEh,\
                        0A5h,021h,000h,020h,0AFh,00Bh,0E5h,060h>
endm

macro   Define_IID_IDirectDrawSurface2
public  IID_IDirectDrawSurface2
IID_IDirectDrawSurface2 GUID <057805885h,06eech,011cfh,\
                        094h,041h,0a8h,023h,003h,0c1h,00eh,027h>
endm

macro   Define_IID_IDirectDrawSurface3
public  IID_IDirectDrawSurface3
IID_IDirectDrawSurface3 GUID <0DA044E00h,069B2h,011D0h,\
                        0A1h,0D5h,000h,0AAh,000h,0B8h,0DFh,0BBh>
endm

macro   Define_IID_IDirectDrawSurface4
public  IID_IDirectDrawSurface4
IID_IDirectDrawSurface4 GUID <00B2B8630h,0AD35h,011D0h,\
                        08Eh,0A6h,000h,060h,097h,097h,0EAh,05Bh>
endm

macro   Define_IID_IDirectDrawPalette
public  IID_IDirectDrawPalette
IID_IDirectDrawPalette  GUID <06C14DB84h,0A733h,011CEh,\
                        0A5h,021h,000h,020h,0AFh,00Bh,0E5h,060h>
endm

macro   Define_IID_IDirectDrawClipper
public  IID_IDirectDrawClipper
IID_IDirectDrawClipper  GUID <06C14DB85h,0A733h,011CEh,\
                        0A5h,021h,000h,020h,0AFh,00Bh,0E5h,060h>
endm

macro   Define_IID_IDirectDrawColorControl
public  IID_IDirectDrawColorControl
IID_IDirectDrawColorControl GUID <04B9F0EE0h,00D7Eh,011D0h,\
                        09Bh,006h,000h,0A0h,0C9h,003h,0A3h,0B8h>
endm

macro   Define_IID_IDirectDrawGammaControl
public  IID_IDirectDrawGammaControl
IID_IDirectDrawGammaControl GUID <069C11C3Eh,0B46Bh,011D1h,\
                            0ADh,07Ah,000h,0C0h,04Fh,0C2h,09Bh,04Eh>
endm}

Const
  CO_E_NOTINITIALIZED = $800401F0;
  _FACDD = $0876;

{
 Flags for DirectDrawEnumerateEx
 DirectDrawEnumerateEx supercedes DirectDrawEnumerate. You must use GetProcAddress to
 obtain a function pointer (of type LPDIRECTDRAWENUMERATEEX) to DirectDrawEnumerateEx.
 By default, only the primary display device is enumerated.
 DirectDrawEnumerate is equivalent to DirectDrawEnumerate(,,DDENUM_NONDISPLAYDEVICES)
}

  DDENUM_ATTACHEDSECONDARYDEVICES = $00000001;
  DDENUM_DETACHEDSECONDARYDEVICES = $00000002;
  DDENUM_NONDISPLAYDEVICES        = $00000004;

{REGSTR_KEY_DDHW_DESCRIPTION    equ <'Description', 0>
REGSTR_KEY_DDHW_DRIVERNAME      equ <'DriverName', 0>
REGSTR_PATH_DDHW                equ <'Hardware\DirectDrawDrivers', 0>}

  DDCREATE_HARDWAREONLY  = $00000001;
  DDCREATE_EMULATIONONLY = $00000002;

{proctype DDENUMMODESCALLBACK :dword, :dword
proctype DDENUMMODESCALLBACK2 :dword, :dword
proctype DDENUMSURFACESCALLBACK :dword, :dword, :dword
proctype DDENUMSURFACESCALLBACK2 :dword, :dword, :dword}

  DD_ROP_SPACE = (256 Div 32); {space required to store ROP array}

  DDSD_CAPS            = $00000001;    {default}
  DDSD_HEIGHT          = $00000002;
  DDSD_WIDTH           = $00000004;
  DDSD_PITCH           = $00000008;
  DDSD_BACKBUFFERCOUNT = $00000020;
  DDSD_ZBUFFERBITDEPTH = $00000040;
  DDSD_ALPHABITDEPTH   = $00000080;
  DDSD_LPSURFACE       = $00000800;
  DDSD_PIXELFORMAT     = $00001000;
  DDSD_CKDESTOVERLAY   = $00002000;
  DDSD_CKDESTBLT       = $00004000;
  DDSD_CKSRCOVERLAY    = $00008000;
  DDSD_CKSRCBLT        = $00010000;
  DDSD_MIPMAPCOUNT     = $00020000;
  DDSD_REFRESHRATE     = $00040000;
  DDSD_LINEARSIZE      = $00080000;
  DDSD_TEXTURESTAGE    = $00100000;
  DDSD_ALL             = $001ff9ee;

  DDOSD_GUID                 = $00000001;
  DDOSD_COMPRESSION_RATIO    = $00000002;
  DDOSD_SCAPS                = $00000004;
  DDOSD_OSCAPS               = $00000008;
  DDOSD_ALL                  = $0000000f;
  DDOSDCAPS_OPTCOMPRESSED    = $00000001;
  DDOSDCAPS_OPTREORDERED     = $00000002;
  DDOSDCAPS_MONOLITHICMIPMAP = $00000004;
  DDOSDCAPS_VALIDSCAPS       = $30004800;
  DDOSDCAPS_VALIDOSCAPS      = $00000007;

  DDCOLOR_BRIGHTNESS  = $00000001;
  DDCOLOR_CONTRAST    = $00000002;
  DDCOLOR_HUE         = $00000004;
  DDCOLOR_SATURATION  = $00000008;
  DDCOLOR_SHARPNESS   = $00000010;
  DDCOLOR_GAMMA       = $00000020;
  DDCOLOR_COLORENABLE = $00000040;

{============================================================================

  Direct Draw Capability Flags

  These flags are used to describe the capabilities of a given Surface.
  All flags are bit flags.

 ==========================================================================}

{***************************************************************************
 *
 * DIRECTDRAWSURFACE CAPABILITY FLAGS
 *
 ***************************************************************************}


  DDSCAPS_RESERVED1          = $00000001;
  DDSCAPS_ALPHA              = $00000002;
  DDSCAPS_BACKBUFFER         = $00000004;
  DDSCAPS_COMPLEX            = $00000008;
  DDSCAPS_FLIP               = $00000010;
  DDSCAPS_FRONTBUFFER        = $00000020;
  DDSCAPS_OFFSCREENPLAIN     = $00000040;
  DDSCAPS_OVERLAY            = $00000080;
  DDSCAPS_PALETTE            = $00000100;
  DDSCAPS_PRIMARYSURFACE     = $00000200;
  DDSCAPS_PRIMARYSURFACELEFT = $00000400;
  DDSCAPS_SYSTEMMEMORY       = $00000800;
  DDSCAPS_TEXTURE            = $00001000;
  DDSCAPS_3DDEVICE           = $00002000;
  DDSCAPS_VIDEOMEMORY        = $00004000;
  DDSCAPS_VISIBLE            = $00008000;
  DDSCAPS_WRITEONLY          = $00010000;
  DDSCAPS_ZBUFFER            = $00020000;
  DDSCAPS_OWNDC              = $00040000;
  DDSCAPS_LIVEVIDEO          = $00080000;
  DDSCAPS_HWCODEC            = $00100000;
  DDSCAPS_MODEX              = $00200000;
  DDSCAPS_MIPMAP             = $00400000;
  DDSCAPS_RESERVED2          = $00800000;
  DDSCAPS_ALLOCONLOAD        = $04000000;
  DDSCAPS_VIDEOPORT          = $08000000;
  DDSCAPS_LOCALVIDMEM        = $10000000;
  DDSCAPS_NONLOCALVIDMEM     = $20000000;
  DDSCAPS_STANDARDVGAMODE    = $40000000;
  DDSCAPS_OPTIMIZED          = $80000000;

  DDSCAPS2_HARDWAREDEINTERLACE = $00000002;
  DDSCAPS2_HINTDYNAMIC         = $00000004;
  DDSCAPS2_HINTSTATIC          = $00000008;
  DDSCAPS2_TEXTUREMANAGE       = $00000010;
  DDSCAPS2_RESERVED1           = $00000020;
  DDSCAPS2_RESERVED2           = $00000040;
  DDSCAPS2_OPAQUE              = $00000080;
  DDSCAPS2_HINTANTIALIASING    = $00000100;

{***************************************************************************
 *
 * DIRECTDRAW DRIVER CAPABILITY FLAGS
 *
 ***************************************************************************}

  DDCAPS_3D                = $00000001;
  DDCAPS_ALIGNBOUNDARYDEST = $00000002;
  DDCAPS_ALIGNSIZEDEST     = $00000004;
  DDCAPS_ALIGNBOUNDARYSRC  = $00000008;
  DDCAPS_ALIGNSIZESRC      = $00000010;
  DDCAPS_ALIGNSTRIDE       = $00000020;
  DDCAPS_BLT               = $00000040;
  DDCAPS_BLTQUEUE          = $00000080;
  DDCAPS_BLTFOURCC         = $00000100;
  DDCAPS_BLTSTRETCH        = $00000200;
  DDCAPS_GDI               = $00000400;
  DDCAPS_OVERLAY           = $00000800;
  DDCAPS_OVERLAYCANTCLIP   = $00001000;
  DDCAPS_OVERLAYFOURCC     = $00002000;
  DDCAPS_OVERLAYSTRETCH    = $00004000;
  DDCAPS_PALETTE           = $00008000;
  DDCAPS_PALETTEVSYNC      = $00010000;
  DDCAPS_READSCANLINE      = $00020000;
  DDCAPS_STEREOVIEW        = $00040000;
  DDCAPS_VBI               = $00080000;
  DDCAPS_ZBLTS             = $00100000;
  DDCAPS_ZOVERLAYS         = $00200000;
  DDCAPS_COLORKEY          = $00400000;
  DDCAPS_ALPHA             = $00800000;
  DDCAPS_COLORKEYHWASSIST  = $01000000;
  DDCAPS_NOHARDWARE        = $02000000;
  DDCAPS_BLTCOLORFILL      = $04000000;
  DDCAPS_BANKSWITCHED      = $08000000;
  DDCAPS_BLTDEPTHFILL      = $10000000;
  DDCAPS_CANCLIP           = $20000000;
  DDCAPS_CANCLIPSTRETCHED  = $40000000;
  DDCAPS_CANBLTSYSMEM      = $80000000;

{***************************************************************************
 *
 * MORE DIRECTDRAW DRIVER CAPABILITY FLAGS (dwCaps2)
 *
 ***************************************************************************}

  DDCAPS2_CERTIFIED            = $00000001;
  DDCAPS2_NO2DDURING3DSCENE    = $00000002;
  DDCAPS2_VIDEOPORT            = $00000004;
  DDCAPS2_AUTOFLIPOVERLAY      = $00000008;
  DDCAPS2_CANBOBINTERLEAVED    = $00000010;
  DDCAPS2_CANBOBNONINTERLEAVED = $00000020;
  DDCAPS2_COLORCONTROLOVERLAY  = $00000040;
  DDCAPS2_COLORCONTROLPRIMARY  = $00000080;
  DDCAPS2_CANDROPZ16BIT        = $00000100;
  DDCAPS2_NONLOCALVIDMEM       = $00000200;
  DDCAPS2_NONLOCALVIDMEMCAPS   = $00000400;
  DDCAPS2_NOPAGELOCKREQUIRED   = $00000800;
  DDCAPS2_WIDESURFACES         = $00001000;
  DDCAPS2_CANFLIPODDEVEN       = $00002000;
  DDCAPS2_CANBOBHARDWARE       = $00004000;
  DDCAPS2_COPYFOURCC           = $00008000;
  DDCAPS2_PRIMARYGAMMA         = $00020000;
  DDCAPS2_CANRENDERWINDOWED    = $00080000;
  DDCAPS2_CANCALIBRATEGAMMA    = $00100000;
  DDCAPS2_FLIPINTERVAL         = $00200000;
  DDCAPS2_FLIPNOVSYNC          = $00400000;

{***************************************************************************
 *
 * DIRECTDRAW FX ALPHA CAPABILITY FLAGS
 *
 ***************************************************************************}

  DDFXALPHACAPS_BLTALPHAEDGEBLEND       = $00000001;
  DDFXALPHACAPS_BLTALPHAPIXELS          = $00000002;
  DDFXALPHACAPS_BLTALPHAPIXELSNEG       = $00000004;
  DDFXALPHACAPS_BLTALPHASURFACES        = $00000008;
  DDFXALPHACAPS_BLTALPHASURFACESNEG     = $00000010;
  DDFXALPHACAPS_OVERLAYALPHAEDGEBLEND   = $00000020;
  DDFXALPHACAPS_OVERLAYALPHAPIXELS      = $00000040;
  DDFXALPHACAPS_OVERLAYALPHAPIXELSNEG   = $00000080;
  DDFXALPHACAPS_OVERLAYALPHASURFACES    = $00000100;
  DDFXALPHACAPS_OVERLAYALPHASURFACESNEG = $00000200;

{***************************************************************************
 *
 * DIRECTDRAW FX CAPABILITY FLAGS
 *
 ***************************************************************************}

  DDFXCAPS_BLTARITHSTRETCHY       = $00000020;
  DDFXCAPS_BLTARITHSTRETCHYN      = $00000010;
  DDFXCAPS_BLTMIRRORLEFTRIGHT     = $00000040;
  DDFXCAPS_BLTMIRRORUPDOWN        = $00000080;
  DDFXCAPS_BLTROTATION            = $00000100;
  DDFXCAPS_BLTROTATION90          = $00000200;
  DDFXCAPS_BLTSHRINKX             = $00000400;
  DDFXCAPS_BLTSHRINKXN            = $00000800;
  DDFXCAPS_BLTSHRINKY             = $00001000;
  DDFXCAPS_BLTSHRINKYN            = $00002000;
  DDFXCAPS_BLTSTRETCHX            = $00004000;
  DDFXCAPS_BLTSTRETCHXN           = $00008000;
  DDFXCAPS_BLTSTRETCHY            = $00010000;
  DDFXCAPS_BLTSTRETCHYN           = $00020000;
  DDFXCAPS_OVERLAYARITHSTRETCHY   = $00040000;
  DDFXCAPS_OVERLAYARITHSTRETCHYN  = $00000008;
  DDFXCAPS_OVERLAYSHRINKX         = $00080000;
  DDFXCAPS_OVERLAYSHRINKXN        = $00100000;
  DDFXCAPS_OVERLAYSHRINKY         = $00200000;
  DDFXCAPS_OVERLAYSHRINKYN        = $00400000;
  DDFXCAPS_OVERLAYSTRETCHX        = $00800000;
  DDFXCAPS_OVERLAYSTRETCHXN       = $01000000;
  DDFXCAPS_OVERLAYSTRETCHY        = $02000000;
  DDFXCAPS_OVERLAYSTRETCHYN       = $04000000;
  DDFXCAPS_OVERLAYMIRRORLEFTRIGHT = $08000000;
  DDFXCAPS_OVERLAYMIRRORUPDOWN    = $10000000;
  DDFXCAPS_BLTALPHA               = $00000001;
  DDFXCAPS_BLTTRANSFORM           = $00000002;
  DDFXCAPS_BLTFILTER              = DDFXCAPS_BLTARITHSTRETCHY;
  DDFXCAPS_OVERLAYALPHA           = $00000004;
  DDFXCAPS_OVERLAYTRANSFORM       = $20000000;
  DDFXCAPS_OVERLAYFILTER          = DDFXCAPS_OVERLAYARITHSTRETCHY;

{***************************************************************************
 *
 * DIRECTDRAW STEREO VIEW CAPABILITIES
 *
 ***************************************************************************}

  DDSVCAPS_ENIGMA  = $00000001;
  DDSVCAPS_FLICKER = $00000002;
  DDSVCAPS_REDBLUE = $00000004;
  DDSVCAPS_SPLIT   = $00000008;

{***************************************************************************
 *
 * DIRECTDRAWPALETTE CAPABILITIES
 *
 ***************************************************************************}

  DDPCAPS_4BIT               = $00000001;
  DDPCAPS_8BITENTRIES        = $00000002;
  DDPCAPS_8BIT               = $00000004;
  DDPCAPS_INITIALIZE         = $00000008;
  DDPCAPS_PRIMARYSURFACE     = $00000010;
  DDPCAPS_PRIMARYSURFACELEFT = $00000020;
  DDPCAPS_ALLOW256           = $00000040;
  DDPCAPS_VSYNC              = $00000080;
  DDPCAPS_1BIT               = $00000100;
  DDPCAPS_2BIT               = $00000200;
  DDPCAPS_ALPHA              = $00000400;

{***************************************************************************
 *
 * DIRECTDRAWSURFACE SETPRIVATEDATA CONSTANTS
 *
 ***************************************************************************}

  DDSPD_IUNKNOWNPOINTER = $00000001;
  DDSPD_VOLATILE        = $00000002;

{***************************************************************************
 *
 * DIRECTDRAW BITDEPTH CONSTANTS
 *
 * NOTE:  These are only used to indicate supported bit depths.   These
 * are flags only, they are not to be used as an actual bit depth.   The
 * absolute numbers 1, 2, 4, 8, 16, 24 and 32 are used to indicate actual
 * bit depths in a surface or for changing the display mode.
 *
 ***************************************************************************}

  DDBD_1  = $00004000;
  DDBD_2  = $00002000;
  DDBD_4  = $00001000;
  DDBD_8  = $00000800;
  DDBD_16 = $00000400;
  DDBD_24 = $00000200;
  DDBD_32 = $00000100;

{***************************************************************************
 *
 * DIRECTDRAWSURFACE SET/GET COLOR KEY FLAGS
 *
 ***************************************************************************}

  DDCKEY_COLORSPACE  = $00000001;
  DDCKEY_DESTBLT     = $00000002;
  DDCKEY_DESTOVERLAY = $00000004;
  DDCKEY_SRCBLT      = $00000008;
  DDCKEY_SRCOVERLAY  = $00000010;

{***************************************************************************
 *
 * DIRECTDRAW COLOR KEY CAPABILITY FLAGS
 *
 ***************************************************************************}

DDCKEYCAPS_DESTBLT                = $00000001;
DDCKEYCAPS_DESTBLTCLRSPACE        = $00000002;
DDCKEYCAPS_DESTBLTCLRSPACEYUV     = $00000004;
DDCKEYCAPS_DESTBLTYUV             = $00000008;
DDCKEYCAPS_DESTOVERLAY            = $00000010;
DDCKEYCAPS_DESTOVERLAYCLRSPACE    = $00000020;
DDCKEYCAPS_DESTOVERLAYCLRSPACEYUV = $00000040;
DDCKEYCAPS_DESTOVERLAYONEACTIVE   = $00000080;
DDCKEYCAPS_DESTOVERLAYYUV         = $00000100;
DDCKEYCAPS_SRCBLT                 = $00000200;
DDCKEYCAPS_SRCBLTCLRSPACE         = $00000400;
DDCKEYCAPS_SRCBLTCLRSPACEYUV      = $00000800;
DDCKEYCAPS_SRCBLTYUV              = $00001000;
DDCKEYCAPS_SRCOVERLAY             = $00002000;
DDCKEYCAPS_SRCOVERLAYCLRSPACE     = $00004000;
DDCKEYCAPS_SRCOVERLAYCLRSPACEYUV  = $00008000;
DDCKEYCAPS_SRCOVERLAYONEACTIVE    = $00010000;
DDCKEYCAPS_SRCOVERLAYYUV          = $00020000;
DDCKEYCAPS_NOCOSTOVERLAY          = $00040000;

{***************************************************************************
 *
 * DIRECTDRAW PIXELFORMAT FLAGS
 *
 ***************************************************************************}

  DDPF_ALPHAPIXELS       = $00000001;
  DDPF_ALPHA             = $00000002;
  DDPF_FOURCC            = $00000004;
  DDPF_PALETTEINDEXED4   = $00000008;
  DDPF_PALETTEINDEXEDTO8 = $00000010;
  DDPF_PALETTEINDEXED8   = $00000020;
  DDPF_RGB               = $00000040;
  DDPF_COMPRESSED        = $00000080;
  DDPF_RGBTOYUV          = $00000100;
  DDPF_YUV               = $00000200;
  DDPF_ZBUFFER           = $00000400;
  DDPF_PALETTEINDEXED1   = $00000800;
  DDPF_PALETTEINDEXED2   = $00001000;
  DDPF_ZPIXELS           = $00002000;
  DDPF_STENCILBUFFER     = $00004000;
  DDPF_ALPHAPREMULT      = $00008000;
  DDPF_LUMINANCE         = $00020000;
  DDPF_BUMPLUMINANCE     = $00040000;
  DDPF_BUMPDUDV          = $00080000;

{===========================================================================


  DIRECTDRAW CALLBACK FLAGS


 ===========================================================================}

{***************************************************************************
 *
 * DIRECTDRAW ENUMSURFACES FLAGS
 *
 ***************************************************************************}

  DDENUMSURFACES_ALL          = $00000001;
  DDENUMSURFACES_MATCH        = $00000002;
  DDENUMSURFACES_NOMATCH      = $00000004;
  DDENUMSURFACES_CANBECREATED = $00000008;
  DDENUMSURFACES_DOESEXIST    = $00000010;

{***************************************************************************
 *
 * DIRECTDRAW SETDISPLAYMODE FLAGS
 *
 ***************************************************************************}

  DDSDM_STANDARDVGAMODE = $00000001;


{***************************************************************************
 *
 * DIRECTDRAW ENUMDISPLAYMODES FLAGS
 *
 ***************************************************************************}

DDEDM_REFRESHRATES = $00000001;
DDEDM_STANDARDVGAMODES = $00000002;

{***************************************************************************
 *
 * DIRECTDRAW SETCOOPERATIVELEVEL FLAGS
 *
 ***************************************************************************}

  DDSCL_FULLSCREEN         = $00000001;
  DDSCL_ALLOWREBOOT        = $00000002;
  DDSCL_NOWINDOWCHANGES    = $00000004;
  DDSCL_NORMAL             = $00000008;
  DDSCL_EXCLUSIVE          = $00000010;
  DDSCL_ALLOWMODEX         = $00000040;
  DDSCL_SETFOCUSWINDOW     = $00000080;
  DDSCL_SETDEVICEWINDOW    = $00000100;
  DDSCL_CREATEDEVICEWINDOW = $00000200;
  DDSCL_MULTITHREADED      = $00000400;
  DDSCL_FPUSETUP           = $00000800;

{***************************************************************************
 *
 * DIRECTDRAW BLT FLAGS
 *
 ***************************************************************************}

  DDBLT_ALPHADEST                = $00000001;
  DDBLT_ALPHADESTCONSTOVERRIDE   = $00000002;
  DDBLT_ALPHADESTNEG             = $00000004;
  DDBLT_ALPHADESTSURFACEOVERRIDE = $00000008;
  DDBLT_ALPHAEDGEBLEND           = $00000010;
  DDBLT_ALPHASRC                 = $00000020;
  DDBLT_ALPHASRCCONSTOVERRIDE    = $00000040;
  DDBLT_ALPHASRCNEG              = $00000080;
  DDBLT_ALPHASRCSURFACEOVERRIDE  = $00000100;
  DDBLT_ASYNC                    = $00000200;
  DDBLT_COLORFILL                = $00000400;
  DDBLT_DDFX                     = $00000800;
  DDBLT_DDROPS                   = $00001000;
  DDBLT_KEYDEST                  = $00002000;
  DDBLT_KEYDESTOVERRIDE          = $00004000;
  DDBLT_KEYSRC                   = $00008000;
  DDBLT_KEYSRCOVERRIDE           = $00010000;
  DDBLT_ROP                      = $00020000;
  DDBLT_ROTATIONANGLE            = $00040000;
  DDBLT_ZBUFFER                  = $00080000;
  DDBLT_ZBUFFERDESTCONSTOVERRIDE = $00100000;
  DDBLT_ZBUFFERDESTOVERRIDE      = $00200000;
  DDBLT_ZBUFFERSRCCONSTOVERRIDE  = $00400000;
  DDBLT_ZBUFFERSRCOVERRIDE       = $00800000;
  DDBLT_WAIT                     = $01000000;
  DDBLT_DEPTHFILL                = $02000000;

{***************************************************************************
 *
 * BLTFAST FLAGS
 *
 ***************************************************************************}

  DDBLTFAST_NOCOLORKEY   = $00000000;
  DDBLTFAST_SRCCOLORKEY  = $00000001;
  DDBLTFAST_DESTCOLORKEY = $00000002;
  DDBLTFAST_WAIT         = $00000010;

{***************************************************************************
 *
 * FLIP FLAGS
 *
 ***************************************************************************}

  DDFLIP_WAIT      = $00000001;
  DDFLIP_EVEN      = $00000002;
  DDFLIP_ODD       = $00000004;
  DDFLIP_NOVSYNC   = $00000008;
  DDFLIP_INTERVAL2 = $02000000;
  DDFLIP_INTERVAL3 = $03000000;
  DDFLIP_INTERVAL4 = $04000000;

{***************************************************************************
 *
 * DIRECTDRAW SURFACE OVERLAY FLAGS
 *
 ***************************************************************************}

  DDOVER_ALPHADEST                = $00000001;
  DDOVER_ALPHADESTCONSTOVERRIDE   = $00000002;
  DDOVER_ALPHADESTNEG             = $00000004;
  DDOVER_ALPHADESTSURFACEOVERRIDE = $00000008;
  DDOVER_ALPHAEDGEBLEND           = $00000010;
  DDOVER_ALPHASRC                 = $00000020;
  DDOVER_ALPHASRCCONSTOVERRIDE    = $00000040;
  DDOVER_ALPHASRCNEG              = $00000080;
  DDOVER_ALPHASRCSURFACEOVERRIDE  = $00000100;
  DDOVER_HIDE                     = $00000200;
  DDOVER_KEYDEST                  = $00000400;
  DDOVER_KEYDESTOVERRIDE          = $00000800;
  DDOVER_KEYSRC                   = $00001000;
  DDOVER_KEYSRCOVERRIDE           = $00002000;
  DDOVER_SHOW                     = $00004000;
  DDOVER_ADDDIRTYRECT             = $00008000;
  DDOVER_REFRESHDIRTYRECTS        = $00010000;
  DDOVER_REFRESHALL               = $00020000;
  DDOVER_DDFX                     = $00080000;
  DDOVER_AUTOFLIP                 = $00100000;
  DDOVER_BOB                      = $00200000;
  DDOVER_OVERRIDEBOBWEAVE         = $00400000;
  DDOVER_INTERLEAVED              = $00800000;
  DDOVER_BOBHARDWARE              = $01000000;

{***************************************************************************
 *
 * DIRECTDRAWSURFACE LOCK FLAGS
 *
 ***************************************************************************}

  DDLOCK_SURFACEMEMORYPTR = $00000000;    {default}
  DDLOCK_WAIT             = $00000001;
  DDLOCK_EVENT            = $00000002;
  DDLOCK_READONLY         = $00000010;
  DDLOCK_WRITEONLY        = $00000020;
  DDLOCK_NOSYSLOCK        = $00000800;

{***************************************************************************
 *
 * DIRECTDRAWSURFACE BLT FX FLAGS
 *
 ***************************************************************************}

DDBLTFX_ARITHSTRETCHY   = $00000001;
DDBLTFX_MIRRORLEFTRIGHT = $00000002;
DDBLTFX_MIRRORUPDOWN    = $00000004;
DDBLTFX_NOTEARING       = $00000008;
DDBLTFX_ROTATE180       = $00000010;
DDBLTFX_ROTATE270       = $00000020;
DDBLTFX_ROTATE90        = $00000040;
DDBLTFX_ZBUFFERRANGE    = $00000080;
DDBLTFX_ZBUFFERBASEDEST = $00000100;

{***************************************************************************
 *
 * DIRECTDRAWSURFACE OVERLAY FX FLAGS
 *
 ***************************************************************************}

  DDOVERFX_ARITHSTRETCHY   = $00000001;
  DDOVERFX_MIRRORLEFTRIGHT = $00000002;
  DDOVERFX_MIRRORUPDOWN    = $00000004;

{***************************************************************************
 *
 * Flags for dwDDFX member of DDSPRITEFX structure
 *
 ***************************************************************************}

  DDSPRITEFX_AFFINETRANSFORM    = $00000001;
  DDSPRITEFX_RGBASCALING        = $00000002;
  DDSPRITEFX_DEGRADERGBASCALING = $00000004;
  DDSPRITEFX_BILINEARFILTER     = $00000008;
  DDSPRITEFX_BLURFILTER         = $00000010;
  DDSPRITEFX_FLATFILTER         = $00000020;
  DDSPRITEFX_DEGRADEFILTER      = $00000040;

{***************************************************************************
 *
 * DIRECTDRAW WAITFORVERTICALBLANK FLAGS
 *
 ***************************************************************************}

  DDWAITVB_BLOCKBEGIN      = $00000001;
  DDWAITVB_BLOCKBEGINEVENT = $00000002;
  DDWAITVB_BLOCKEND        = $00000004;

{***************************************************************************
 *
 * DIRECTDRAW GETFLIPSTATUS FLAGS
 *
 ***************************************************************************}

  DDGFS_CANFLIP    = $00000001;
  DDGFS_ISFLIPDONE = $00000002;

{***************************************************************************
 *
 * DIRECTDRAW GETBLTSTATUS FLAGS
 *
 ***************************************************************************}

  DDGBS_CANBLT    = $00000001;
  DDGBS_ISBLTDONE = $00000002;

{***************************************************************************
 *
 * DIRECTDRAW ENUMOVERLAYZORDER FLAGS
 *
 ***************************************************************************}

  DDENUMOVERLAYZ_BACKTOFRONT = $00000000;
  DDENUMOVERLAYZ_FRONTTOBACK = $00000001;

{***************************************************************************
 *
 * DIRECTDRAW UPDATEOVERLAYZORDER FLAGS
 *
 ***************************************************************************}

  DDOVERZ_SENDTOFRONT     = $00000000;
  DDOVERZ_SENDTOBACK      = $00000001;
  DDOVERZ_MOVEFORWARD     = $00000002;
  DDOVERZ_MOVEBACKWARD    = $00000003;
  DDOVERZ_INSERTINFRONTOF = $00000004;
  DDOVERZ_INSERTINBACKOF  = $00000005;


{***************************************************************************
 *
 * DIRECTDRAW SETGAMMARAMP FLAGS
 *
 ***************************************************************************}

  DDSGR_CALIBRATE = $00000001;


{===========================================================================


  DIRECTDRAW RETURN CODES

  The return values from DirectDraw Commands and Surface that return an HRESULT
  are codes from DirectDraw concerning the results of the action
  requested by DirectDraw.

 ===========================================================================}

  DD_OK    = 0;
  DD_FALSE = S_FALSE;

{***************************************************************************
 *
 * DIRECTDRAW ENUMCALLBACK RETURN VALUES
 *
 * EnumCallback returns are used to control the flow of the DIRECTDRAW and
 * DIRECTDRAWSURFACE object enumerations.   They can only be returned by
 * enumeration callback routines.
 *
 ***************************************************************************}

  DDENUMRET_CANCEL = 0;
  DDENUMRET_OK     = 1;

{***************************************************************************
 *
 * DIRECTDRAW ERRORS
 *
 * Errors are represented by negative values and cannot be combined.
 *
 ***************************************************************************}

  DDERR_ALREADYINITIALIZED           = ($80000000 + (_FACDD Shl 16) + 5);
  DDERR_CANNOTATTACHSURFACE          = ($80000000 + (_FACDD Shl 16) + 10);
  DDERR_CANNOTDETACHSURFACE          = ($80000000 + (_FACDD Shl 16) + 20);
  DDERR_CURRENTLYNOTAVAIL            = ($80000000 + (_FACDD Shl 16) + 40);
  DDERR_EXCEPTION                    = ($80000000 + (_FACDD Shl 16) + 55);
  DDERR_GENERIC                      = E_FAIL;
  DDERR_HEIGHTALIGN                  = ($80000000 + (_FACDD Shl 16) + 90);
  DDERR_INCOMPATIBLEPRIMARY          = ($80000000 + (_FACDD Shl 16) + 95);
  DDERR_INVALIDCAPS                  = ($80000000 + (_FACDD Shl 16) + 100);
  DDERR_INVALIDCLIPLIST              = ($80000000 + (_FACDD Shl 16) + 110);
  DDERR_INVALIDMODE                  = ($80000000 + (_FACDD Shl 16) + 120);
  DDERR_INVALIDOBJECT                = ($80000000 + (_FACDD Shl 16) + 130);
  DDERR_INVALIDPARAMS                = E_INVALIDARG;
  DDERR_INVALIDPIXELFORMAT           = ($80000000 + (_FACDD Shl 16) + 145);
  DDERR_INVALIDRECT                  = ($80000000 + (_FACDD Shl 16) + 150);
  DDERR_LOCKEDSURFACES               = ($80000000 + (_FACDD Shl 16) + 160);
  DDERR_NO3D                         = ($80000000 + (_FACDD Shl 16) + 170);
  DDERR_NOALPHAHW                    = ($80000000 + (_FACDD Shl 16) + 180);
  DDERR_NOCLIPLIST                   = ($80000000 + (_FACDD Shl 16) + 205);
  DDERR_NOCOLORCONVHW                = ($80000000 + (_FACDD Shl 16) + 210);
  DDERR_NOCOOPERATIVELEVELSET        = ($80000000 + (_FACDD Shl 16) + 212);
  DDERR_NOCOLORKEY                   = ($80000000 + (_FACDD Shl 16) + 215);
  DDERR_NOCOLORKEYHW                 = ($80000000 + (_FACDD Shl 16) + 220);
  DDERR_NODIRECTDRAWSUPPORT          = ($80000000 + (_FACDD Shl 16) + 222);
  DDERR_NOEXCLUSIVEMODE              = ($80000000 + (_FACDD Shl 16) + 225);
  DDERR_NOFLIPHW                     = ($80000000 + (_FACDD Shl 16) + 230);
  DDERR_NOGDI                        = ($80000000 + (_FACDD Shl 16) + 240);
  DDERR_NOMIRRORHW                   = ($80000000 + (_FACDD Shl 16) + 250);
  DDERR_NOTFOUND                     = ($80000000 + (_FACDD Shl 16) + 255);
  DDERR_NOOVERLAYHW                  = ($80000000 + (_FACDD Shl 16) + 260);
  DDERR_OVERLAPPINGRECTS             = ($80000000 + (_FACDD Shl 16) + 270);
  DDERR_NORASTEROPHW                 = ($80000000 + (_FACDD Shl 16) + 280);
  DDERR_NOROTATIONHW                 = ($80000000 + (_FACDD Shl 16) + 290);
  DDERR_NOSTRETCHHW                  = ($80000000 + (_FACDD Shl 16) + 310);
  DDERR_NOT4BITCOLOR                 = ($80000000 + (_FACDD Shl 16) + 316);
  DDERR_NOT4BITCOLORINDEX            = ($80000000 + (_FACDD Shl 16) + 317);
  DDERR_NOT8BITCOLOR                 = ($80000000 + (_FACDD Shl 16) + 320);
  DDERR_NOTEXTUREHW                  = ($80000000 + (_FACDD Shl 16) + 330);
  DDERR_NOVSYNCHW                    = ($80000000 + (_FACDD Shl 16) + 335);
  DDERR_NOZBUFFERHW                  = ($80000000 + (_FACDD Shl 16) + 340);
  DDERR_NOZOVERLAYHW                 = ($80000000 + (_FACDD Shl 16) + 350);
  DDERR_OUTOFCAPS                    = ($80000000 + (_FACDD Shl 16) + 360);
  DDERR_OUTOFMEMORY                  = E_OUTOFMEMORY;
  DDERR_OUTOFVIDEOMEMORY             = ($80000000 + (_FACDD Shl 16) + 380);
  DDERR_OVERLAYCANTCLIP              = ($80000000 + (_FACDD Shl 16) + 382);
  DDERR_OVERLAYCOLORKEYONLYONEACTIVE = ($80000000 + (_FACDD Shl 16) + 384);
  DDERR_PALETTEBUSY                  = ($80000000 + (_FACDD Shl 16) + 387);
  DDERR_COLORKEYNOTSET               = ($80000000 + (_FACDD Shl 16) + 400);
  DDERR_SURFACEALREADYATTACHED       = ($80000000 + (_FACDD Shl 16) + 410);
  DDERR_SURFACEALREADYDEPENDENT      = ($80000000 + (_FACDD Shl 16) + 420);
  DDERR_SURFACEBUSY                  = ($80000000 + (_FACDD Shl 16) + 430);
  DDERR_CANTLOCKSURFACE              = ($80000000 + (_FACDD Shl 16) + 435);
  DDERR_SURFACEISOBSCURED            = ($80000000 + (_FACDD Shl 16) + 440);
  DDERR_SURFACELOST                  = ($80000000 + (_FACDD Shl 16) + 450);
  DDERR_SURFACENOTATTACHED           = ($80000000 + (_FACDD Shl 16) + 460);
  DDERR_TOOBIGHEIGHT                 = ($80000000 + (_FACDD Shl 16) + 470);
  DDERR_TOOBIGSIZE                   = ($80000000 + (_FACDD Shl 16) + 480);
  DDERR_TOOBIGWIDTH                  = ($80000000 + (_FACDD Shl 16) + 490);
  DDERR_UNSUPPORTED                  = E_NOTIMPL;
  DDERR_UNSUPPORTEDFORMAT            = ($80000000 + (_FACDD Shl 16) + 510);
  DDERR_UNSUPPORTEDMASK              = ($80000000 + (_FACDD Shl 16) + 520);
  DDERR_INVALIDSTREAM                = ($80000000 + (_FACDD Shl 16) + 521);
  DDERR_VERTICALBLANKINPROGRESS      = ($80000000 + (_FACDD Shl 16) + 537);
  DDERR_WASSTILLDRAWING              = ($80000000 + (_FACDD Shl 16) + 540);
  DDERR_XALIGN                       = ($80000000 + (_FACDD Shl 16) + 560);
  DDERR_INVALIDDIRECTDRAWGUID        = ($80000000 + (_FACDD Shl 16) + 561);
  DDERR_DIRECTDRAWALREADYCREATED     = ($80000000 + (_FACDD Shl 16) + 562);
  DDERR_NODIRECTDRAWHW               = ($80000000 + (_FACDD Shl 16) + 563);
  DDERR_PRIMARYSURFACEALREADYEXISTS  = ($80000000 + (_FACDD Shl 16) + 564);
  DDERR_NOEMULATION                  = ($80000000 + (_FACDD Shl 16) + 565);
  DDERR_REGIONTOOSMALL               = ($80000000 + (_FACDD Shl 16) + 566);
  DDERR_CLIPPERISUSINGHWND           = ($80000000 + (_FACDD Shl 16) + 567);
  DDERR_NOCLIPPERATTACHED            = ($80000000 + (_FACDD Shl 16) + 568);
  DDERR_NOHWND                       = ($80000000 + (_FACDD Shl 16) + 569);
  DDERR_HWNDSUBCLASSED               = ($80000000 + (_FACDD Shl 16) + 570);
  DDERR_HWNDALREADYSET               = ($80000000 + (_FACDD Shl 16) + 571);
  DDERR_NOPALETTEATTACHED            = ($80000000 + (_FACDD Shl 16) + 572);
  DDERR_NOPALETTEHW                  = ($80000000 + (_FACDD Shl 16) + 573);
  DDERR_BLTFASTCANTCLIP              = ($80000000 + (_FACDD Shl 16) + 574);
  DDERR_NOBLTHW                      = ($80000000 + (_FACDD Shl 16) + 575);
  DDERR_NODDROPSHW                   = ($80000000 + (_FACDD Shl 16) + 576);
  DDERR_OVERLAYNOTVISIBLE            = ($80000000 + (_FACDD Shl 16) + 577);
  DDERR_NOOVERLAYDEST                = ($80000000 + (_FACDD Shl 16) + 578);
  DDERR_INVALIDPOSITION              = ($80000000 + (_FACDD Shl 16) + 579);
  DDERR_NOTAOVERLAYSURFACE           = ($80000000 + (_FACDD Shl 16) + 580);
  DDERR_EXCLUSIVEMODEALREADYSET      = ($80000000 + (_FACDD Shl 16) + 581);
  DDERR_NOTFLIPPABLE                 = ($80000000 + (_FACDD Shl 16) + 582);
  DDERR_CANTDUPLICATE                = ($80000000 + (_FACDD Shl 16) + 583);
  DDERR_NOTLOCKED                    = ($80000000 + (_FACDD Shl 16) + 584);
  DDERR_CANTCREATEDC                 = ($80000000 + (_FACDD Shl 16) + 585);
  DDERR_NODC                         = ($80000000 + (_FACDD Shl 16) + 586);
  DDERR_WRONGMODE                    = ($80000000 + (_FACDD Shl 16) + 587);
  DDERR_IMPLICITLYCREATED            = ($80000000 + (_FACDD Shl 16) + 588);
  DDERR_NOTPALETTIZED                = ($80000000 + (_FACDD Shl 16) + 589);
  DDERR_UNSUPPORTEDMODE              = ($80000000 + (_FACDD Shl 16) + 590);
  DDERR_NOMIPMAPHW                   = ($80000000 + (_FACDD Shl 16) + 591);
  DDERR_INVALIDSURFACETYPE           = ($80000000 + (_FACDD Shl 16) + 592);
  DDERR_NOOPTIMIZEHW                 = ($80000000 + (_FACDD Shl 16) + 600);
  DDERR_NOTLOADED                    = ($80000000 + (_FACDD Shl 16) + 601);
  DDERR_NOFOCUSWINDOW                = ($80000000 + (_FACDD Shl 16) + 602);
  DDERR_DCALREADYCREATED             = ($80000000 + (_FACDD Shl 16) + 620);
  DDERR_NONONLOCALVIDMEM             = ($80000000 + (_FACDD Shl 16) + 630);
  DDERR_CANTPAGELOCK                 = ($80000000 + (_FACDD Shl 16) + 640);
  DDERR_CANTPAGEUNLOCK               = ($80000000 + (_FACDD Shl 16) + 660);
  DDERR_NOTPAGELOCKED                = ($80000000 + (_FACDD Shl 16) + 680);
  DDERR_MOREDATA                     = ($80000000 + (_FACDD Shl 16) + 690);
  DDERR_EXPIRED                      = ($80000000 + (_FACDD Shl 16) + 691);
  DDERR_VIDEONOTACTIVE               = ($80000000 + (_FACDD Shl 16) + 695);
  DDERR_DEVICEDOESNTOWNSURFACE       = ($80000000 + (_FACDD Shl 16) + 699);
  DDERR_NOTINITIALIZED               = CO_E_NOTINITIALIZED;

Type
  PHWND = ^HWND;
  PHDC = ^HDC;
  LPLPDIRECTDRAW = ^LPDIRECTDRAW;
  LPDIRECTDRAW = ^IDIRECTDRAW;
  LPLPDIRECTDRAWSURFACE = ^LPDIRECTDRAWSURFACE;
  LPDIRECTDRAWSURFACE = ^IDirectDrawSurface;
  LPLPDIRECTDRAWCLIPPER = ^LPDIRECTDRAWCLIPPER;
  LPDIRECTDRAWCLIPPER = ^IDirectDrawClipper;
  LPLPDIRECTDRAWPALETTE = ^LPDIRECTDRAWPALETTE;
  LPDIRECTDRAWPALETTE = ^IDirectDrawPalette;

  LPDIRECTDRAW2 = LPDIRECTDRAW;

  LPLPVOID = ^LPVOID;
  LPVOID = Pointer;
  LPDWORD = ^DWord;

{
  Generic pixel format with 8-bit RGB and alpha components
}
  LPDDRGBA = ^DDRGBA;
  DDRGBA = Record
    red, green, blue, alpha : Byte;
  End;

  LPDDCOLORKEY = ^DDCOLORKEY;
  DDCOLORKEY = Record
    dwColorSpaceLowValue : DWord;
    dwColorSpaceHighValue : DWord;
  End;
  LPDDBLTFX = ^DDBLTFX;
  DDBLTFX = Record
    dwSize : DWord;
    dwDDFX : DWord;
    dwROP : DWord;
    dwDDROP : DWord;
    dwRotationAngle : DWord;
    dwZBufferOpCode : DWord;
    dwZBufferLow : DWord;
    dwZBufferHigh : DWord;
    dwZBufferBaseDest : DWord;
    dwZDestConstBitDepth : DWord;
    dwZDestConst : DWord; {union w/: lpDDSZBufferDest : LPDIRECTDRAWSURFACE}
    dwZSrcConstBitDepth : DWord;
    dwZSrcConst : DWord; {union w/: lpDDSZBufferSrc : LPDIRECTDRAWSURFACE}
    dwAlphaEdgeBlendBitDepth : DWord;
    dwAlphaEdgeBlend : DWord;
    dwReserved : DWord;
    dwAlphaDestConstBitDepth : DWord;
    dwAlphaDestConst : DWord; {union w/: lpDDSAlphaDest : LPDIRECTDRAWSURFACE}
    dwAlphaSrcConstBitDepth : DWord;
    dwAlphaSrcConst : DWord; {union w/: lpDDSAlphaSrc : LPDIRECTDRAWSURFACE}
    dwFillColor : DWord;
      {union w/: dwFillDepth : DWord}
      {union w/: lpDDSPattern : LPDIRECTDRAWSURFACE}
    ddckDestColorKey : DDCOLORKEY;
    ddckSrcColorKey : DDCOLORKEY;
  End;
  LPDDPIXELFORMAT = ^DDPIXELFORMAT;
  DDPIXELFORMAT = Record
    dwSize : DWord;
    dwFlags : DWord;
    dwFourCC : DWord;
    dwRGBBitCount : DWord;
    { union w/:
       dwYUVBitCount : DWord;
       dwZBufferBitDepth : DWord;
       dwAlphaBitDepth : DWord;
    }
    dwRBitMask : DWord;
    { union w/: dwYBitMask : DWord;}
    dwGBitMask : DWord;
    { union w/: dwUBitMask : DWord;}
    dwBBitMask : DWord;
    { union w/: dwVBitMask : DWord;}
    dwRGBAlphaBitMask : DWord;
    { union w/: dwYUVAlphaBitMask : DWord;}
  End;
  LPDDSCAPS = ^DDSCAPS;
  DDSCAPS = Record
    dwCaps : DWord;
  End;
  LPDDOSCAPS = ^DDOSCAPS;
  DDOSCAPS = Record
    dwCaps : DWord;
  End;

{ This structure is used internally by DirectDraw.}
  LPDDSCAPSEX = ^DDSCAPSEX;
  DDSCAPSEX = Record
    dwCaps2 : DWord;
    dwCaps3 : DWord;
    dwCaps4 : DWord;
  End;

  LPDDSCAPS2 = ^DDSCAPS2;
  DDSCAPS2 = Record
    dwCaps : DWord;
    dwCaps2 : DWord;
    dwCaps3 : DWord;
    dwCaps4 : DWord;
  End;

{This structure is the DDCAPS structure as it was in version 2 and 3 of Direct X.
 It is present for back compatability.}
  DDCAPS_DX3 = Record
{dwSize                 dd ?            ; size of the DDDRIVERCAPS structure
        dwCaps                  dd ?            ; driver specific capabilities
        dwCaps2                 dd ?            ; more driver specific capabilites
        dwCKeyCaps              dd ?            ; color key capabilities of the surface
        dwFXCaps                dd ?            ; driver specific stretching and effects capabilites
        dwFXAlphaCaps           dd ?            ; alpha driver specific capabilities
        dwPalCaps               dd ?            ; palette capabilities
        dwSVCaps                dd ?            ; stereo vision capabilities
        dwAlphaBltConstBitDepths dd ?           ; DDBD_2,4,8
        dwAlphaBltPixelBitDepths dd ?           ; DDBD_1,2,4,8
        dwAlphaBltSurfaceBitDepths dd ?         ; DDBD_1,2,4,8
        dwAlphaOverlayConstBitDepths dd ?       ; DDBD_2,4,8
        dwAlphaOverlayPixelBitDepths dd ?       ; DDBD_1,2,4,8
        dwAlphaOverlaySurfaceBitDepths dd ?     ; DDBD_1,2,4,8
        dwZBufferBitDepths      dd ?            ; DDBD_8,16,24,32
        dwVidMemTotal           dd ?            ; total amount of video memory
        dwVidMemFree            dd ?            ; amount of free video memory
        dwMaxVisibleOverlays    dd ?            ; maximum number of visible overlays
        dwCurrVisibleOverlays   dd ?            ; current number of visible overlays
        dwNumFourCCCodes        dd ?            ; number of four cc codes
        dwAlignBoundarySrc      dd ?            ; source rectangle alignment
        dwAlignSizeSrc          dd ?            ; source rectangle byte size
        dwAlignBoundaryDest     dd ?            ; dest rectangle alignment
        dwAlignSizeDest         dd ?            ; dest rectangle byte size
        dwAlignStrideAlign      dd ?            ; stride alignment
        dwRops                  dd DD_ROP_SPACE dup (?) ; ROPS supported
        ddsCaps                 DDSCAPS ?       ; DDSCAPS structure has all the general capabilities
        dwMinOverlayStretch     dd ?            ; minimum overlay stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3
        dwMaxOverlayStretch     dd ?            ; maximum overlay stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3
        dwMinLiveVideoStretch   dd ?            ; minimum live video stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3
        dwMaxLiveVideoStretch   dd ?            ; maximum live video stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3
        dwMinHwCodecStretch     dd ?            ; minimum hardware codec stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3
        dwMaxHwCodecStretch     dd ?            ; maximum hardware codec stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3
        dwReserved1             dd ?            ; reserved
        dwReserved2             dd ?            ; reserved
        dwReserved3             dd ?            ; reserved
        dwSVBCaps               dd ?            ; driver specific capabilities for System->Vmem blts
        dwSVBCKeyCaps           dd ?            ; driver color key capabilities for System->Vmem blts
        dwSVBFXCaps             dd ?            ; driver FX capabilities for System->Vmem blts
        dwSVBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for System->Vmem blts
        dwVSBCaps               dd ?            ; driver specific capabilities for Vmem->System blts
        dwVSBCKeyCaps           dd ?            ; driver color key capabilities for Vmem->System blts
        dwVSBFXCaps             dd ?            ; driver FX capabilities for Vmem->System blts
        dwVSBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for Vmem->System blts
        dwSSBCaps               dd ?            ; driver specific capabilities for System->System blts
        dwSSBCKeyCaps           dd ?            ; driver color key capabilities for System->System blts
        dwSSBFXCaps             dd ?            ; driver FX capabilities for System->System blts
        dwSSBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for System->System blts
        dwReserved4             dd ?            ; reserved
        dwReserved5             dd ?            ; reserved
        dwReserved6             dd ?            ; reserved}
  End;

{This structure is the DDCAPS structure as it was in version 5 of Direct X.
 It is present for back compatability.}
  DDCAPS_DX5 = Record
{dwSize                 dd ?            ; size of the DDDRIVERCAPS structure ;  4
        dwCaps                  dd ?            ; driver specific capabilities ;  8
        dwCaps2                 dd ?            ; more driver specific capabilites ;  c
        dwCKeyCaps              dd ?            ; color key capabilities of the surface ; 10
        dwFXCaps                dd ?            ; driver specific stretching and effects capabilites ; 14
        dwFXAlphaCaps           dd ?            ; alpha driver specific capabilities ; 18
        dwPalCaps               dd ?            ; palette capabilities ; 1c
        dwSVCaps                dd ?            ; stereo vision capabilities ; 20
        dwAlphaBltConstBitDepths dd ?           ; DDBD_2,4,8 ; 24
        dwAlphaBltPixelBitDepths dd ?           ; DDBD_1,2,4,8 ; 28
        dwAlphaBltSurfaceBitDepths dd ?         ; DDBD_1,2,4,8 ; 2c
        dwAlphaOverlayConstBitDepths dd ?       ; DDBD_2,4,8 ; 30
        dwAlphaOverlayPixelBitDepths dd ?       ; DDBD_1,2,4,8 ; 34
        dwAlphaOverlaySurfaceBitDepths dd ?     ; DDBD_1,2,4,8 ; 38
        dwZBufferBitDepths      dd ?            ; DDBD_8,16,24,32 ; 3c
        dwVidMemTotal           dd ?            ; total amount of video memory ; 40
        dwVidMemFree            dd ?            ; amount of free video memory ; 44
        dwMaxVisibleOverlays    dd ?            ; maximum number of visible overlays ; 48
        dwCurrVisibleOverlays   dd ?            ; current number of visible overlays ; 4c
        dwNumFourCCCodes        dd ?            ; number of four cc codes ; 50
        dwAlignBoundarySrc      dd ?            ; source rectangle alignment ; 54
        dwAlignSizeSrc          dd ?            ; source rectangle byte size ; 58
        dwAlignBoundaryDest     dd ?            ; dest rectangle alignment ; 5c
        dwAlignSizeDest         dd ?            ; dest rectangle byte size ; 60
        dwAlignStrideAlign      dd ?            ; stride alignment ; 64
        dwRops                  dd DD_ROP_SPACE dup (?) ; ROPS supported ; 84
        ddsCaps                 DDSCAPS ?       ; DDSCAPS structure has all the general capabilities ; 88
        dwMinOverlayStretch     dd ?            ; minimum overlay stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 8c
        dwMaxOverlayStretch     dd ?            ; maximum overlay stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 90
        dwMinLiveVideoStretch   dd ?            ; minimum live video stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 94
        dwMaxLiveVideoStretch   dd ?            ; maximum live video stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 98
        dwMinHwCodecStretch     dd ?            ; minimum hardware codec stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 9c
        dwMaxHwCodecStretch     dd ?            ; maximum hardware codec stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; a0
        dwReserved1             dd ?            ; reserved ; a4
        dwReserved2             dd ?            ; reserved ; a8
        dwReserved3             dd ?            ; reserved ; ac
        dwSVBCaps               dd ?            ; driver specific capabilities for System->Vmem blts ; b0
        dwSVBCKeyCaps           dd ?            ; driver color key capabilities for System->Vmem blts ; b4
        dwSVBFXCaps             dd ?            ; driver FX capabilities for System->Vmem blts ; b8
        dwSVBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for System->Vmem blts ; d8
        dwVSBCaps               dd ?            ; driver specific capabilities for Vmem->System blts ; dc
        dwVSBCKeyCaps           dd ?            ; driver color key capabilities for Vmem->System blts ; e0
        dwVSBFXCaps             dd ?            ; driver FX capabilities for Vmem->System blts ; e4
        dwVSBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for Vmem->System blts ;104
        dwSSBCaps               dd ?            ; driver specific capabilities for System->System blts ;108
        dwSSBCKeyCaps           dd ?            ; driver color key capabilities for System->System blts ;10c
        dwSSBFXCaps             dd ?            ; driver FX capabilities for System->System blts ;110
        dwSSBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for System->System blts ; Members added for DX5: ;130
        dwMaxVideoPorts         dd ?            ; maximum number of usable video ports ;134
        dwCurrVideoPorts        dd ?            ; current number of video ports used ;138
        dwSVBCaps2              dd ?            ; more driver specific capabilities for System->Vmem blts ;13c
        dwNLVBCaps              dd ?            ; driver specific capabilities for non-local->local vidmem blts ;140
        dwNLVBCaps2             dd ?            ; more driver specific capabilities non-local->local vidmem blts ;144
        dwNLVBCKeyCaps          dd ?            ; driver color key capabilities for non-local->local vidmem blts ;148
        dwNLVBFXCaps            dd ?            ; driver FX capabilities for non-local->local blts ;14c
        dwNLVBRops              dd DD_ROP_SPACE dup (?) ; ROPS supported for non-local->local blts}
  End;

  DDCAPS_DX6 = Record
{dwSize                 dd ?            ; size of the DDDRIVERCAPS structure ;  4
        dwCaps                  dd ?            ; driver specific capabilities ;  8
        dwCaps2                 dd ?            ; more driver specific capabilites ;  c
        dwCKeyCaps              dd ?            ; color key capabilities of the surface ; 10
        dwFXCaps                dd ?            ; driver specific stretching and effects capabilites ; 14
        dwFXAlphaCaps           dd ?            ; alpha caps ; 18
        dwPalCaps               dd ?            ; palette capabilities ; 1c
        dwSVCaps                dd ?            ; stereo vision capabilities ; 20
        dwAlphaBltConstBitDepths dd ?           ; DDBD_2,4,8 ; 24
        dwAlphaBltPixelBitDepths dd ?           ; DDBD_1,2,4,8 ; 28
        dwAlphaBltSurfaceBitDepths dd ?         ; DDBD_1,2,4,8 ; 2c
        dwAlphaOverlayConstBitDepths dd ?       ; DDBD_2,4,8 ; 30
        dwAlphaOverlayPixelBitDepths dd ?       ; DDBD_1,2,4,8 ; 34
        dwAlphaOverlaySurfaceBitDepths dd ?     ; DDBD_1,2,4,8 ; 38
        dwZBufferBitDepths      dd ?            ; DDBD_8,16,24,32 ; 3c
        dwVidMemTotal           dd ?            ; total amount of video memory ; 40
        dwVidMemFree            dd ?            ; amount of free video memory ; 44
        dwMaxVisibleOverlays    dd ?            ; maximum number of visible overlays ; 48
        dwCurrVisibleOverlays   dd ?            ; current number of visible overlays ; 4c
        dwNumFourCCCodes        dd ?            ; number of four cc codes ; 50
        dwAlignBoundarySrc      dd ?            ; source rectangle alignment ; 54
        dwAlignSizeSrc          dd ?            ; source rectangle byte size ; 58
        dwAlignBoundaryDest     dd ?            ; dest rectangle alignment ; 5c
        dwAlignSizeDest         dd ?            ; dest rectangle byte size ; 60
        dwAlignStrideAlign      dd ?            ; stride alignment ; 64
        dwRops                  dd DD_ROP_SPACE dup (?) ; ROPS supported ; 84
        ddsOldCaps              DDSCAPS ?       ; Was DDSCAPS   ddsCaps. ddsCaps is of type DDSCAPS2 for DX6 ; 88
        dwMinOverlayStretch     dd ?            ; minimum overlay stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 8c
        dwMaxOverlayStretch     dd ?            ; maximum overlay stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 90
        dwMinLiveVideoStretch   dd ?            ; minimum live video stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 94
        dwMaxLiveVideoStretch   dd ?            ; maximum live video stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 98
        dwMinHwCodecStretch     dd ?            ; minimum hardware codec stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; 9c
        dwMaxHwCodecStretch     dd ?            ; maximum hardware codec stretch factor multiplied by 1000, eg 1000 == 1.0, 1300 == 1.3 ; a0
        dwReserved1             dd ?            ; reserved ; a4
        dwReserved2             dd ?            ; reserved ; a8
        dwReserved3             dd ?            ; reserved ; ac
        dwSVBCaps               dd ?            ; driver specific capabilities for System->Vmem blts ; b0
        dwSVBCKeyCaps           dd ?            ; driver color key capabilities for System->Vmem blts ; b4
        dwSVBFXCaps             dd ?            ; driver FX capabilities for System->Vmem blts ; b8
        dwSVBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for System->Vmem blts ; d8
        dwVSBCaps               dd ?            ; driver specific capabilities for Vmem->System blts ; dc
        dwVSBCKeyCaps           dd ?            ; driver color key capabilities for Vmem->System blts ; e0
        dwVSBFXCaps             dd ?            ; driver FX capabilities for Vmem->System blts ; e4
        dwVSBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for Vmem->System blts ;104
        dwSSBCaps               dd ?            ; driver specific capabilities for System->System blts ;108
        dwSSBCKeyCaps           dd ?            ; driver color key capabilities for System->System blts ;10c
        dwSSBFXCaps             dd ?            ; driver FX capabilities for System->System blts ;110
        dwSSBRops               dd DD_ROP_SPACE dup (?) ; ROPS supported for System->System blts ;130
        dwMaxVideoPorts         dd ?            ; maximum number of usable video ports ;134
        dwCurrVideoPorts        dd ?            ; current number of video ports used ;138
        dwSVBCaps2              dd ?            ; more driver specific capabilities for System->Vmem blts ;13c
        dwNLVBCaps              dd ?            ; driver specific capabilities for non-local->local vidmem blts ;140
        dwNLVBCaps2             dd ?            ; more driver specific capabilities non-local->local vidmem blts ;144
        dwNLVBCKeyCaps          dd ?            ; driver color key capabilities for non-local->local vidmem blts ;148
        dwNLVBFXCaps            dd ?            ; driver FX capabilities for non-local->local blts ;14c
        dwNLVBRops              dd DD_ROP_SPACE dup (?) ; ROPS supported for non-local->local blts ; Members added for DX6 release ;16c
        ddsCaps                 DDSCAPS2 ?      ; Surface Caps}
  End;
  LPDDCAPS = ^DDCAPS;
  DDCAPS = DDCAPS_DX6;
  {DDCAPS = DDCAPS_DX5;}
  {DDCAPS = DDCAPS_DX3;}

  LPDDSURFACEDESC = ^DDSURFACEDESC;
  DDSURFACEDESC = Record
    dwSize : DWord;
    dwFlags : DWord;
    dwHeight : DWord;
    dwWidth : DWord;
    lPitch : LongInt;
    dwBackBufferCount : DWord;
    dwMipMapCount : DWord;
      { union w/:
        dwZBufferBitDepth : DWord;
        dwRefreshRate : DWord;
      }
    dwAlphaBitDepth : DWord;
    dwReserved : DWord;
    lpSurface : LPVOID;
    ddckCKDestOverlay : DDCOLORKEY;
    ddckCKDestBlt : DDCOLORKEY;

    ddckCKSrcOverlay : DDCOLORKEY;
    ddckCKSrcBlt : DDCOLORKEY;
    ddpfPixelFormat : DDPIXELFORMAT;
    ddsCaps : DDSCAPS;
  End;
  LPDDOVERLAYFX = ^DDOVERLAYFX;
  DDOVERLAYFX = Record
    dwSize : DWord;
    dwAlphaEdgeBlendBitDepth : DWord;
    dwAlphaEdgeBlend : DWord;
    dwReserved : DWord;
    dwAlphaDestConstBitDepth : DWord;
    dwAlphaDestConst : DWord;
      {union w/: lpDDSAlphaDest : LPDIRECTDRAWSURFACE;}
    dwAlphaSrcConstBitDepth : DWord;
    dwAlphaSrcConst : DWord;
      {union w/: lpDDSAlphaSrc : LPDIRECTDRAWSURFACE;}
    dckDestColorkey : DDCOLORKEY;
    dckSrcColorkey : DDCOLORKEY;

    dwDDFX : DWord;
    dwFlags : DWord;
  End;
  LPDDBLTBATCH = ^DDBLTBATCH;
  DDBLTBATCH = Record
    lprDest : LPRECT;
    lpDDSSrc : LPDIRECTDRAWSURFACE;
    lprSrc : LPRECT;
    dwFlags : DWord;
    lpDDBltFx : LPDDBLTFX;
  End;
  LPDDGAMMARAMP = ^DDGAMMARAMP;
  DDGAMMARAMP = Record
    red : Array[0..255] Of Word;
    green : Array[0..255] Of Word;
    blue : Array[0..255] Of Word;
  End;
{;
; This is the structure within which DirectDraw returns data about the current graphics driver and chipset
;

MAX_DDDEVICEID_STRING           = 512


struc   DDDEVICEIDENTIFIER
        szDriver                db MAX_DDDEVICEID_STRING dup (?)
        szDescription           db MAX_DDDEVICEID_STRING dup (?)
label   liDriverVersion         qword
        dwDriverVersionLowPart  dd ?
        dwDriverVersionHighPart dd ?
        dwVendorId              dd ?
        dwDeviceId              dd ?
        dwSubSysId              dd ?
        dwRevision              dd ?
        guidDeviceIdentifier    GUID ?
ends}
{;
; Flags for the IDirectDraw4::GetDeviceIdentifier method
;

DDGDI_GETHOSTIDENTIFIER         = 000000001h

proctype CLIPPERCALLBACK :dword, :dword, :dword, :dword
proctype SURFACESTREAMINGCALLBACK :dword}
(*  LPDDCAPS = ^DDCAPS;
  DDCAPS = Record
    dwSize : DWord;
    {todo...}
  End;*)

  LPDDCOLORCONTROL = ^DDCOLORCONTROL;
  DDCOLORCONTROL = Record
    dwSize : DWord;
    dwFlags : DWord;
    lBrightness : LongInt;
    lContrast : LongInt;
    lHue : LongInt;
    lSaturation : LongInt;
    lSharpness : LongInt;
    lGamma : LongInt;
    lColorEnable : LongInt;
    dwReserved1 : DWord;
  End;

  LPDDENUMCALLBACK = Pointer; {CDecl; (maybe)     NO CDECL! STDCALL!}
  LPDDENUMMODESCALLBACK = Pointer; {CDecl; !!!        NO CDECL! STDCALL! todo:check DX SDK}
  LPDDENUMSURFACESCALLBACK = Pointer; {CDecl; (maybe)     NO CDECL! STDCALL!}
  {
  BOOL WINAPI lpCallback(GUID FAR * lpGUID, LPSTR lpDriverDescription, LPSTR lpDriverName, LPVOID lpContext);
  HRESULT WINAPI lpEnumModesCallback(LPDDSURFACEDESC lpDDSurfaceDesc, LPVOID lpContext);
  HRESULT WINAPI lpEnumSurfacesCallback(LPDIRECTDRAWSURFACE2 lpDDSurface, LPDDSURFACEDESC lpDDSurfaceDesc, LPVOID lpContext);
  HRESULT WINAPI lpfnCallback(LPDIRECTDRAWSURFACE lpDDSurface, LPVOID lpContext);
  }


{
; INTERFACES FOLLOW:
;     IDirectDraw
;     IDirectDrawClipper
;     IDirectDrawPalette
;     IDirectDrawSurface
;

;
; IDirectDraw
;

struc   IDirectDraw
;** IUnknown methods **
        QueryInterface          dd ?       0
        AddRef                  dd ?       1
        Release                 dd ?       2
;** IDirectDraw methods **
        Compact                 dd ?       3
        CreateClipper           dd ?       4
        CreatePalette           dd ?       5
        CreateSurface           dd ?       6
        DuplicateSurface        dd ?       7
        EnumDisplayModes        dd ?       8
        EnumSurfaces            dd ?       9
        FlipToGDISurface        dd ?       10
        GetCaps                 dd ?       11
        GetDisplayMode          dd ?       12
        GetFourCCCodes          dd ?       13
        GetGDISurface           dd ?       14
        GetMonitorFrequency     dd ?       15
        GetScanLine             dd ?       16
        GetVerticalBlankStatus  dd ?       17
        Initialize              dd ?       18
        RestoreDisplayMode      dd ?       19
        SetCooperativeLevel     dd ?       20
        SetDisplayMode          dd ?       21
        WaitForVerticalBlank    dd ?       22
;** IDirectDraw2 methods **
        GetAvailableVidMem      dd ?       23
;** IDirectDraw4 methods **
        GetSurfaceFromDC        dd ?       24
        RestoreAllSurfaces      dd ?       25
        TestCooperativeLevel    dd ?       26
        GetDeviceIdentifier     dd ?       27
ends

typedef IDirectDraw2            IDirectDraw
typedef IDirectDraw4            IDirectDraw}
  IDirectDraw = Record
    lpVtbl : ^IDirectDrawVtbl;
  End;
  IDirectDraw2 = IDirectDraw;
  IDirectDraw4 = IDirectDraw;
  IDirectDrawVtbl = Record
    QueryInterface : Function(obj : LPDIRECTDRAW; sht : LPGUID; lplpGUZ : LPLPVOID) : DWord; DXCall;
    AddRef : Function(obj : LPDIRECTDRAW) : DWord; DXCall;
    Release : Function(obj : LPDIRECTDRAW) : DWord; DXCall;
    Compact : Function(obj : LPDIRECTDRAW) : HResult; DXCall;
    CreateClipper : Function(obj : LPDIRECTDRAW;
                 dwFlags : DWord; lplpDDClipper : LPLPDIRECTDRAWCLIPPER;
                 pUnkOther : Pointer) : HResult; DXCall;
    CreatePalette : Function(obj : LPDIRECTDRAW;
                 dwFlags : DWord; lpColorTable : LPPALETTEENTRY;
                 lplpDDPalette : LPLPDIRECTDRAWPALETTE;
                 pUnkOther : Pointer) : HResult; DXCall;
    CreateSurface : Function(obj : LPDIRECTDRAW;
                 lpDDSurfaceDesc : LPDDSURFACEDESC;
                 lplpDDSurface : LPLPDIRECTDRAWSURFACE;
                 pUnkOther : Pointer) : HResult; DXCall;
    DuplicateSurface : Function(obj : LPDIRECTDRAW;
                 lpDDSurface : LPDIRECTDRAWSURFACE;
                 lplpDupDDSurface : LPLPDIRECTDRAWSURFACE) : HResult; DXCall;
    EnumDisplayModes : Function(obj : LPDIRECTDRAW;
                 dwFlags : DWord; lpDDSurfaceDesc : LPDDSURFACEDESC;
                 lpContext : LPVOID;
                 lpEnumModesCallback : LPDDENUMMODESCALLBACK) : HResult; DXCall;
    EnumSurfaces : Function(obj : LPDIRECTDRAW;
                 dwFlags : DWord; lpDDSD : LPDDSURFACEDESC;
                 lpContext : LPVOID;
                 lpEnumSurfacesCallback : LPDDENUMSURFACESCALLBACK) : HResult; DXCall;
{    dummy1, dummy2 : DWord;}
    FlipToGDISurface : Function(obj : LPDIRECTDRAW) : HResult; DXCall;
    GetCaps : Function(obj : LPDIRECTDRAW; lpDDDriverCaps : LPDDCAPS;
                 lpDDHELCaps : LPDDCAPS) : HResult; DXCall;
    GetDisplayMode : Function(obj : LPDIRECTDRAW;
                 lpDDSurfaceDesc : LPDDSURFACEDESC) : HResult; DXCall;
    GetFourCCCodes : Function(obj : LPDIRECTDRAW;
                 lpNumCodes : LPDWORD; lpCodes : LPDWORD) : HResult; DXCall;
    GetGDISurface : Function(obj : LPDIRECTDRAW;
                 lplpGDIDDSSurface : LPLPDIRECTDRAWSURFACE) : HResult; DXCall;
    GetMonitorFrequency : Function(obj : LPDIRECTDRAW;
                 lpdwFrequency : LPDWORD) : HResult; DXCall;
    GetScanLine : Function(obj : LPDIRECTDRAW;
                 lpdwScanLine : LPDWORD) : HResult; DXCall;
    GetVerticalBlankStatus : Function(obj : LPDIRECTDRAW;
                 lpbIsInVB : LPBOOL) : HResult; DXCall;
    {Function DirectDraw_Initialize(obj : LPDIRECTDRAW) : HResult; DXCall;}
    Initialize : DWord;
    RestoreDisplayMode : Function(obj : LPDIRECTDRAW) : HResult; DXCall;
    SetCooperativeLevel : Function(obj : LPDIRECTDRAW;
                 hWnd : HWND; dwFlags : DWord) : HResult; DXCall;
    SetDisplayMode : Function(obj : LPDIRECTDRAW;
                 dwWidth, dwHeight, dwBPP, dwRefreshRate, dwFlags : DWord) : HResult; DXCall;
    WaitForVerticalBlank : Function(obj : LPDIRECTDRAW;
                 dwFlags : DWord; hEvent : HANDLE) : HResult; DXCall;
    GetAvailableVidMem : Function(obj : LPDIRECTDRAW;
                 lpDDSCaps : LPDDSCAPS; lpdwTotal : LPDWORD; lpdwFree : LPDWORD) : HResult; DXCall;
  End;

(*Function DirectDraw_AddRef(obj : LPDIRECTDRAW) : DWord;
Function DirectDraw_Release(obj : LPDIRECTDRAW) : DWord;
Function DirectDraw_Compact(obj : LPDIRECTDRAW) : HResult;
Function DirectDraw_CreateClipper(obj : LPDIRECTDRAW;
             dwFlags : DWord; lplpDDClipper : LPLPDIRECTDRAWCLIPPER;
             pUnkOther : Pointer) : HResult;
Function DirectDraw_CreatePalette(obj : LPDIRECTDRAW;
             dwFlags : DWord; lpColorTable : LPPALETTEENTRY;
             lplpDDPalette : LPLPDIRECTDRAWPALETTE;
             pUnkOther : Pointer) : HResult;
Function DirectDraw_CreateSurface(obj : LPDIRECTDRAW;
             lpDDSurfaceDesc : LPDDSURFACEDESC;
             lplpDDSurface : LPLPDIRECTDRAWSURFACE;
             pUnkOther : Pointer) : HResult;
Function DirectDraw_DuplicateSurface(obj : LPDIRECTDRAW;
             lpDDSurface : LPDIRECTDRAWSURFACE;
             lplpDupDDSurface : LPLPDIRECTDRAWSURFACE) : HResult;
Function DirectDraw_EnumDisplayModes(obj : LPDIRECTDRAW;
             dwFlags : DWord; lpDDSurfaceDesc : LPDDSURFACEDESC;
             lpContext : LPVOID;
             lpEnumModesCallback : LPDDENUMMODESCALLBACK) : HResult;
Function DirectDraw_EnumSurfaces(obj : LPDIRECTDRAW;
             dwFlags : DWord; lpDDSD : LPDDSURFACEDESC;
             lpContext : LPVOID;
             lpEnumSurfacesCallback : LPDDENUMSURFACESCALLBACK) : HResult;
Function DirectDraw_FlipToGDISurface(obj : LPDIRECTDRAW) : HResult;
Function DirectDraw_GetCaps(obj : LPDIRECTDRAW; lpDDDriverCaps : LPDDCAPS;
             lpDDHELCaps : LPDDCAPS) : HResult;
Function DirectDraw_GetDisplayMode(obj : LPDIRECTDRAW;
             lpDDSurfaceDesc : LPDDSURFACEDESC) : HResult;
Function DirectDraw_GetFourCCCodes(obj : LPDIRECTDRAW;
             lpNumCodes : LPDWORD; lpCodes : LPDWORD) : HResult;
Function DirectDraw_GetGDISurface(obj : LPDIRECTDRAW;
             lplpGDIDDSSurface : LPLPDIRECTDRAWSURFACE) : HResult;
Function DirectDraw_GetMonitorFrequency(obj : LPDIRECTDRAW;
             lpdwFrequency : LPDWORD) : HResult;
Function DirectDraw_GetScanLine(obj : LPDIRECTDRAW;
             lpdwScanLine : LPDWORD) : HResult;
Function DirectDraw_GetVerticalBlankStatus(obj : LPDIRECTDRAW;
             lpbIsInVB : LPBOOL) : HResult;
{Function DirectDraw_Initialize(obj : LPDIRECTDRAW) : HResult;}
Function DirectDraw_RestoreDisplayMode(obj : LPDIRECTDRAW) : HResult;
Function DirectDraw_SetCooperativeLevel(obj : LPDIRECTDRAW;
             hWnd : HWND; dwFlags : DWord) : HResult;
Function DirectDraw_SetDisplayMode(obj : LPDIRECTDRAW;
             dwWidth, dwHeight, dwBPP, dwRefreshRate, dwFlags : DWord) : HResult;
Function DirectDraw_WaitForVerticalBlank(obj : LPDIRECTDRAW;
             dwFlags : DWord; hEvent : HANDLE) : HResult;
Function DirectDraw_GetAvailableVidMem(obj : LPDIRECTDRAW;
             lpDDSCaps : LPDDSCAPS; lpdwTotal : LPDWORD; lpdwFree : LPDWORD) : HResult;*)


  IDirectDrawPalette = Record
    lpVtbl : ^IDirectDrawPaletteVtbl;
  End;
  IDirectDrawPaletteVtbl = Record
    q : Pointer;
    AddRef : Function(obj : LPDIRECTDRAWPALETTE) : DWord; DXCall;
    Release : Function(obj : LPDIRECTDRAWPALETTE) : DWord; DXCall;
    GetCaps : Function(obj : LPDIRECTDRAWPALETTE;
                lpdwCaps : LPDWORD) : HRESULT; DXCall;
    GetEntries : Function(obj : LPDIRECTDRAWPALETTE;
                   dwFlags, dwBase, dwNumEntries : DWord;
                   lpEntries : LPPALETTEENTRY) : HRESULT; DXCall;
    Initialize : Function(obj : LPDIRECTDRAWPALETTE;
                   lpDD : LPDIRECTDRAW; dwFlags : DWord;
                   lpDDColorTable : LPPALETTEENTRY) : HRESULT; DXCall;
    SetEntries : Function(obj : LPDIRECTDRAWPALETTE;
                   dwFlags, dwStartingEntry, dwCount : DWord;
                   lpEntries : LPPALETTEENTRY) : HRESULT; DXCall;
  End;

{
;
; IDirectDrawPalette
;

struc   IDirectDrawPalette
;** IUnknown methods **
        QueryInterface          dd ?
        AddRef                  dd ?
        Release                 dd ?
;** IDirectDrawPalette methods **
        GetCaps                 dd ?
        GetEntries              dd ?
        Initialize              dd ?
        SetEntries              dd ?
ends}

  IDirectDrawClipper = Record
    lpVtbl : ^IDirectDrawClipperVtbl;
  End;
  IDirectDrawClipperVtbl = Record
    q : Pointer;
    AddRef : Function(obj : LPDIRECTDRAWCLIPPER) : DWord; DXCall;
    Release : Function(obj : LPDIRECTDRAWCLIPPER) : DWord; DXCall;
    GetClipList : Function(obj : LPDIRECTDRAWCLIPPER;
                    lpRect : LPRECT; lpClipList : LPRGNDATA;
                    lpdwSize : LPDWORD) : HRESULT; DXCall;
    GetHWnd : Function(obj : LPDIRECTDRAWCLIPPER;
                lphWnd : PHWND) : HRESULT; DXCall;
    Initialize : Function(obj : LPDIRECTDRAWCLIPPER;
                   lpDD : LPDIRECTDRAW; dwFlags : DWord) : HRESULT; DXCall;
    IsClipListChanged : Function(obj : LPDIRECTDRAWCLIPPER;
                          lpbChanged : PBoolean) : HRESULT; DXCall;
    SetClipList : Function(obj : LPDIRECTDRAWCLIPPER;
                    lpClipList : LPRGNDATA;
                    dwFlags : DWord) : HRESULT; DXCall;
    SetHWnd : Function(obj : LPDIRECTDRAWCLIPPER;
                dwFlags : DWord; hWnd : HWND) : HRESULT; DXCall;
  End;

{;
; IDirectDrawClipper
;

struc   IDirectDrawClipper
;** IUnknown methods **
        QueryInterface          dd ?
        AddRef                  dd ?
        Release                 dd ?
;** IDirectDrawClipper methods **
        GetClipList             dd ?
        GetHWnd                 dd ?
        Initialize              dd ?
        IsClipListChanged       dd ?
        SetClipList             dd ?
        SetHWnd                 dd ?
ends}

  IDirectDrawSurface = Record
    lpVtbl : ^IDirectDrawSurfaceVtbl;
  End;
  IDirectDrawSurface2 = IDirectDrawSurface;
  IDirectDrawSurface3 = IDirectDrawSurface;
  IDirectDrawSurface4 = IDirectDrawSurface;
  IDirectDrawSurfaceVtbl = Record
    q : Pointer;
    AddRef : Function(obj : LPDIRECTDRAWSURFACE) : DWord; DXCall;
    Release : Function(obj : LPDIRECTDRAWSURFACE) : DWord; DXCall;
    AddAttachedSurface : Function(obj : LPDIRECTDRAWSURFACE;
                           lpDDSAttachedSurface : LPDIRECTDRAWSURFACE{2}) : HRESULT; DXCall;
    AddOverlayDirtyRect : Function(obj : LPDIRECTDRAWSURFACE;
                            lpRect : LPRECT) : HRESULT; DXCall;
    Blt : Function(obj : LPDIRECTDRAWSURFACE; lpDestRect : LPRECT;
            lpDDSrcSurface : LPDIRECTDRAWSURFACE{2}; lpSrcRect : LPRECT;
            dwFlags : DWord; lpDDBltFx : LPDDBLTFX) : HRESULT; DXCall;
    BltBatch : Function(obj : LPDIRECTDRAWSURFACE;
                 lpDDBltBatch : LPDDBLTBATCH; dwCount, dwFlags : DWord) : HRESULT; DXCall;
    BltFast : Function(obj : LPDIRECTDRAWSURFACE; dwX, dwY : DWord;
                lpDDSrcSurface : LPDIRECTDRAWSURFACE{2}; lpSrcRect : LPRECT;
                dwTrans : DWord) : HRESULT; DXCall;
    DeleteAttachedSurface : Function(obj : LPDIRECTDRAWSURFACE;
                              dwFlags : DWord;
                              lpDDSAttachedSurface : LPDIRECTDRAWSURFACE{2}) : HRESULT; DXCall;
    EnumAttachedSurfaces : Function(obj : LPDIRECTDRAWSURFACE;
                             lpContext : LPVOID;
                             lpEnumSurfacesCallback : LPDDENUMSURFACESCALLBACK) : HRESULT; DXCall;
    EnumOverlayZOrders : Function(obj : LPDIRECTDRAWSURFACE;
                           dwFlags : DWord; lpContext : LPVOID;
                           lpfnCallback : LPDDENUMSURFACESCALLBACK) : HRESULT; DXCall;
    Flip : Function(obj : LPDIRECTDRAWSURFACE;
             lpDDSurfaceTargetOverride : LPDIRECTDRAWSURFACE{2};
             dwFlags : DWord) : HRESULT; DXCall;
    GetAttachedSurface : Function(obj : LPDIRECTDRAWSURFACE;
                           lpDDSCaps : LPDDSCAPS;
                           lplpDDAttachedSurface : LPLPDIRECTDRAWSURFACE{2}) : HRESULT; DXCall;
    GetBltStatus : Function(obj : LPDIRECTDRAWSURFACE;
                     dwFlags : DWord) : HRESULT; DXCall;
    GetCaps : Function(obj : LPDIRECTDRAWSURFACE;
                lpDDSCaps : LPDDSCAPS) : HRESULT; DXCall;
    GetClipper : Function(obj : LPDIRECTDRAWSURFACE;
                   lplpDDClipper : LPLPDIRECTDRAWCLIPPER) : HRESULT; DXCall;
    GetColorKey : Function(obj : LPDIRECTDRAWSURFACE;
                    dwFlags : DWord; lpDDColorKey : LPDDCOLORKEY) : HRESULT; DXCall;
    GetDC : Function(obj : LPDIRECTDRAWSURFACE;
              lphDC : PHDC) : HRESULT; DXCall;
    GetFlipStatus : Function(obj : LPDIRECTDRAWSURFACE;
                      dwFlags : DWord) : HRESULT; DXCall;
    GetOverlayPosition : Function(obj : LPDIRECTDRAWSURFACE;
                           lplX, lplY : LPDWORD) : HRESULT; DXCall;
    GetPalette : Function(obj : LPDIRECTDRAWSURFACE;
                   lplpDDPalette : LPLPDIRECTDRAWPALETTE) : HRESULT; DXCall;
    GetPixelFormat : Function(obj : LPDIRECTDRAWSURFACE;
                       lpDDPixelFormat : LPDDPIXELFORMAT) : HRESULT; DXCall;
    GetSurfaceDesc : Function(obj : LPDIRECTDRAWSURFACE;
                       lpDDSurfaceDesc : LPDDSURFACEDESC) : HRESULT; DXCall;
    Initialize : Function(obj : LPDIRECTDRAWSURFACE;
                   lpDD : LPDIRECTDRAW;
                   lpDDSurfaceDesc : LPDDSURFACEDESC) : HRESULT; DXCall;
    IsLost : Function(obj : LPDIRECTDRAWSURFACE) : HRESULT; DXCall;
    Lock : Function(obj : LPDIRECTDRAWSURFACE;
             lpDestRect : LPRECT; lpDDSurfaceDesc : LPDDSURFACEDESC;
             dwFlags : DWord; hEvent : HANDLE) : HRESULT; DXCall;
    ReleaseDC : Function(obj : LPDIRECTDRAWSURFACE;
                  hDC : HDC) : HRESULT; DXCall;
    Restore : Function(obj : LPDIRECTDRAWSURFACE) : HRESULT; DXCall;
    SetClipper : Function(obj : LPDIRECTDRAWSURFACE;
                   lpDDClipper : LPDIRECTDRAWCLIPPER) : HRESULT; DXCall;
    SetColorKey : Function(obj : LPDIRECTDRAWSURFACE;
                    dwFlags : DWord; lpDDColorKey : LPDDCOLORKEY) : HRESULT; DXCall;
    SetOverlayPosition : Function(obj : LPDIRECTDRAWSURFACE;
                           lX, lY : LongInt) : HRESULT; DXCall;
    SetPalette : Function(obj : LPDIRECTDRAWSURFACE;
                   lpDDPalette : LPDIRECTDRAWPALETTE) : HRESULT; DXCall;
    Unlock : Function(obj : LPDIRECTDRAWSURFACE;
               lpSurfaceData : LPVOID) : HRESULT; DXCall;
    UpdateOverlay : Function(obj : LPDIRECTDRAWSURFACE;
                      lpSrcRect : LPRECT; lpDDDestSurface : LPDIRECTDRAWSURFACE{2};
                      lpDestRect : LPRECT; dwFlags : DWord;
                      lpDDOverlayFx : LPDDOVERLAYFX) : HRESULT; DXCall;
    UpdateOverlayDisplay : Function(obj : LPDIRECTDRAWSURFACE;
                             dwFlags : DWord) : HRESULT; DXCall;
    UpdateOverlayZOrder : Function(obj : LPDIRECTDRAWSURFACE;
                            dwFlags : DWord;
                            lpDDSReference : LPDIRECTDRAWSURFACE{2}) : HRESULT; DXCall;
    {v2}
    GetDDInterface : Function(obj : LPDIRECTDRAWSURFACE;
                       lplpDD : LPLPVOID) : HRESULT; DXCall;
    PageLock : Function(obj : LPDIRECTDRAWSURFACE;
                 dwFlags : DWord) : HRESULT; DXCall;
    PageUnlock : Function(obj : LPDIRECTDRAWSURFACE;
                   dwFlags : DWord) : HRESULT; DXCall;
    {v3}
{    SetSurfaceDesc}
    {v4}
{    SetPrivateData
    GetPrivateData
    FreePrivateData
    GetUniquenessValue
    ChangeUniquenessValue}
  End;
{;
; IDirectDrawSurface and related interfaces
;

struc   IDirectDrawSurface
;** IUnknown methods **
        QueryInterface          dd ?
        AddRef                  dd ?
        Release                 dd ?
;** IDirectDrawSurface methods **
        AddAttachedSurface      dd ?
        AddOverlayDirtyRect     dd ?
        Blt                     dd ?
        BltBatch                dd ?
        BltFast                 dd ?
        DeleteAttachedSurface   dd ?
        EnumAttachedSurfaces    dd ?
        EnumOverlayZOrders      dd ?
        Flip                    dd ?
        GetAttachedSurface      dd ?
        GetBltStatus            dd ?
        GetCaps                 dd ?
        GetClipper              dd ?
        GetColorKey             dd ?
        GetDC                   dd ?
        GetFlipStatus           dd ?
        GetOverlayPosition      dd ?
        GetPalette              dd ?
        GetPixelFormat          dd ?
        GetSurfaceDesc          dd ?
        Initialize              dd ?
        IsLost                  dd ?
        _Lock                   dd ?
        ReleaseDC               dd ?
        Restore                 dd ?
        SetClipper              dd ?
        SetColorKey             dd ?
        SetOverlayPosition      dd ?
        SetPalette              dd ?
        Unlock                  dd ?
        UpdateOverlay           dd ?
        UpdateOverlayDisplay    dd ?
        UpdateOverlayZOrder     dd ?
;** Added in the v2 interface **
        GetDDInterface          dd ?
        PageLock                dd ?
        PageUnlock              dd ?
;** Added in the v3 interface **
        SetSurfaceDesc          dd ?
;** Added in the v4 interface **
        SetPrivateData          dd ?
        GetPrivateData          dd ?
        FreePrivateData         dd ?
        GetUniquenessValue      dd ?
        ChangeUniquenessValue   dd ?
ends

typedef IDirectDrawSurface2     IDirectDrawSurface
typedef IDirectDrawSurface3     IDirectDrawSurface
typedef IDirectDrawSurface4     IDirectDrawSurface
}

{
;
; IDirectDrawColorControl
;

struc   IDirectDrawColorControl
;** IUnknown methods **
        QueryInterface          dd ?
        AddRef                  dd ?
        Release                 dd ?
;** IDirectDrawColorControl methods **
        GetColorControls        dd ?
        SetColorControls        dd ?
ends

;
; IDirectDrawGammaControl
;

struc   IDirectDrawGammaControl
;** IUnknown methods **
        QueryInterface          dd ?
        AddRef                  dd ?
        Release                 dd ?
;** IDirectDrawColorControl methods **
        GetGammaRamp            dd ?
        SetGammaRamp            dd ?
ends}

  TDirectDrawCreate = Function(lpGUID : Pointer; lplpDD : LPLPDIRECTDRAW; pUnkOuter : Pointer{IUnknown FAR *}) : HRESULT; DXCall;
  TDirectDrawCreateClipper = Function(dwFlags : DWord; lplpDDClipper : LPLPDIRECTDRAWCLIPPER; pUnkOuter : Pointer{IUnknown FAR *}) : HRESULT; DXCall;
  TDirectDrawEnumerate = Function(lpCallback : LPDDENUMCALLBACK; lpContext : LPVOID) : HRESULT; DXCall; {DirectDrawEnumerateA}
  TDirectDrawEnumerateEx = Function(A, B, C : DWord) : HRESULT; DXCall; {DirectDrawEnumerateExA}

Var
  DirectDrawCreate : TDirectDrawCreate;
  DirectDrawCreateClipper : TDirectDrawCreateClipper;
  DirectDrawEnumerate : TDirectDrawEnumerate;
  DirectDrawEnumerateEx : TDirectDrawEnumerateEx;

Implementation

Begin
  DirectDrawCreate := Nil;
  DirectDrawCreateClipper := Nil;
  DirectDrawEnumerate := Nil;
  DirectDrawEnumerateEx := Nil;
End.
