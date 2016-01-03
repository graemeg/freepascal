{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2002 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
    History:

    Update for AmigaOS 3.9.
    Added some const and a few records.
    Added reaction and workbench.
    31 Jan 2003.

    Changed integer > smallint.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm

}


unit prefs;

INTERFACE
uses exec, iffparse, graphics, timer, intuition;


{ Asl }

const
   ID_ASL = $41534C20;

{ These members correspond directly to the associated
           members of the 'AslSemaphore' data structure defined
           in the <libraries/asl.h> header file by the same names.
          }

  type
     PAslPrefs = ^tAslPrefs;
     tAslPrefs = record
          ap_Reserved : array[0..3] of LONG;
          ap_SortBy : UBYTE;
          ap_SortDrawers : UBYTE;
          ap_SortOrder : UBYTE;
          ap_SizePosition : UBYTE;
          ap_RelativeLeft : WORD;
          ap_RelativeTop : WORD;
          ap_RelativeWidth : UBYTE;
          ap_RelativeHeight : UBYTE;
       end;


{ Font }
{***************************************************************************}

const
 ID_FONT = 1179602516;


 FONTNAMESIZE = 128;

type
 pFontPrefs = ^tFontPrefs;
 tFontPrefs = record
    fp_Reserved     : Array[0..2] of Longint;
    fp_Reserved2    : WORD;
    fp_Type         : WORD;
    fp_FrontPen,
    fp_BackPen,
    fp_DrawMode     : Byte;
    fp_TextAttr     : tTextAttr;
    fp_Name         : Array[0..FONTNAMESIZE-1] of Char;
 end;

const
{ constants for FontPrefs.fp_Type }
 FP_WBFONT     = 0;
 FP_SYSFONT    = 1;
 FP_SCREENFONT = 2;


{***************************************************************************}

{ IControl }
{***************************************************************************}

const
 ID_ICTL = 1229149260;

Type
 pIControlPrefs = ^tIControlPrefs;
 tIControlPrefs = record
    ic_Reserved     : Array[0..3] Of Longint;       { System reserved              }
    ic_TimeOut      : WORD;                         { Verify timeout               }
    ic_MetaDrag     : smallint;                      { Meta drag mouse event        }
    ic_Flags        : ULONG;                        { IControl flags (see below)   }
    ic_WBtoFront,                                   { CKey: WB to front            }
    ic_FrontToBack,                                 { CKey: front screen to back   }
    ic_ReqTrue,                                     { CKey: Requester TRUE         }
    ic_ReqFalse     : Byte;                         { CKey: Requester FALSE        }
 end;

const
{ flags for IControlPrefs.ic_Flags }
 ICB_COERCE_COLORS = 0;
 ICB_COERCE_LACE   = 1;
 ICB_STRGAD_FILTER = 2;
 ICB_MENUSNAP      = 3;
 ICB_MODEPROMOTE   = 4;
 ICB_SQUARE_RATIO  = 5;

 ICF_COERCE_COLORS = 1;
 ICF_COERCE_LACE   = 2;
 ICF_STRGAD_FILTER = 4;
 ICF_MENUSNAP      = 8;
 ICF_MODEPROMOTE   = 16;
 ICF_SQUARE_RATIO  = (1 shl 5);

{***************************************************************************}

  {      File format for input preferences     }


const
 ID_INPT = 1229869140;

Type
 pInputPrefs = ^tInputPrefs;
 tInputPrefs = record
    ip_Keymap      : Array[0..15] of Char;
    ip_PointerTicks : WORD;
    ip_DoubleClick,
    ip_KeyRptDelay,
    ip_KeyRptSpeed : tTimeVal;
    ip_MouseAccel  : smallint;
 end;

 {      File format for locale preferences }

const
 ID_LCLE = 1279478853;
 ID_CTRY = 1129599577;


Type
 pCountryPrefs = ^tCountryPrefs;
 tCountryPrefs = record
    cp_Reserved     : Array[0..3] of ULONG;
    cp_CountryCode  : ULONG;
    cp_TelephoneCode: ULONG;
    cp_MeasuringSystem : Byte;

    cp_DateTimeFormat  : Array[0..79] of Char;
    cp_DateFormat      : Array[0..39] of Char;
    cp_TimeFormat      : Array[0..39] of Char;

    cp_ShortDateTimeFormat  : Array[0..79] of Char;
    cp_ShortDateFormat      : Array[0..39] of Char;
    cp_ShortTimeFormat      : Array[0..39] of Char;

    { for numeric values }
    cp_DecimalPoint,
    cp_GroupSeparator,
    cp_FracGroupSeparator   : Array[0..9] of Char;
    cp_Grouping,
    cp_FracGrouping         : Array[0..9] of Byte;

    { for monetary values }
    cp_MonDecimalPoint,
    cp_MonGroupSeparator,
    cp_MonFracGroupSeparator   : Array[0..9] of Char;
    cp_MonGrouping,
    cp_MonFracGrouping         : Array[0..9] of Byte;
    cp_MonFracDigits,
    cp_MonIntFracDigits        : Byte;

    { for currency symbols }
    cp_MonCS,
    cp_MonSmallCS,
    cp_MonIntCS                : Array[0..9] of Char;

    { for positive monetary values }
    cp_MonPositiveSign         : Array[0..9] of Char;
    cp_MonPositiveSpaceSep,
    cp_MonPositiveSignPos,
    cp_MonPositiveCSPos        : Byte;

    { for negative monetary values }
    cp_MonNegativeSign         : Array[0..9] of Char;
    cp_MonNegativeSpaceSep,
    cp_MonNegativeSignPos,
    cp_MonNegativeCSPos        : Byte;

    cp_CalendarType            : Byte;
 end;

 pLocalePrefs = ^tLocalePrefs;
 tLocalePrefs = record
    lp_Reserved         : Array[0..3] of ULONG;
    lp_CountryName      : Array[0..31] of Char;
    lp_PreferredLanguages : Array[0..9] of Array[0..29] of Char;
    lp_GMTOffset        : Longint;
    lp_Flags            : ULONG;
    lp_CountryData      : tCountryPrefs;
 end;

   {   File format for overscan preferences  }

const
    ID_OSCN = 1330856782;
    OSCAN_MAGIC = $FEDCBA89;


Type
 pOverscanPrefs = ^tOverscanPrefs;
 tOverscanPrefs = record
    os_Reserved,
    os_Magic         : ULONG;
    os_HStart,
    os_HStop,
    os_VStart,
    os_VStop         : WORD;
    os_DisplayID     : ULONG;
    os_ViewPos,
    os_Text          : tPoint;
    os_Standard      : tRectangle;
 end;

{ os_HStart, os_HStop, os_VStart, os_VStop can only be looked at if
 * os_Magic equals OSCAN_MAGIC. If os_Magic is set to any other value,
 * these four fields are undefined
 }


{***************************************************************************}

  {      File format for palette preferences    }


const
    ID_PALT = 1346456660;

Type
 pPalettePrefs = ^tPalettePrefs;
 tPalettePrefs = record
    pap_Reserved     : Array[0..3] of Longint;    { System reserved                }
    pap_4ColorPens   : Array[1..32] of WORD;
    pap_8ColorPens   : Array[1..32] of WORD;
    pap_Colors       : Array[1..32] of tColorSpec;     { Used as full 16-bit RGB values }
 end;


{***************************************************************************}


 {      File format for pointer preferences }

const
      ID_PNTR = 1347310674;


Type
 pPointerPrefs = ^tPointerPrefs;
 tPointerPrefs = record
    pp_Reserved : Array[0..3] of ULONG;
    pp_Which,                             { 0=NORMAL, 1=BUSY }
    pp_Size,                              { see <intuition/pointerclass.h> }
    pp_Width,                             { Width in pixels }
    pp_Height,                            { Height in pixels }
    pp_Depth,                             { Depth }
    pp_YSize,                             { YSize }
    pp_X, pp_Y  : WORD;                   { Hotspot }

    { Color Table:  numEntries = (1 << pp_Depth) - 1 }

    { Data follows }
 end;

{***************************************************************************}

Const
{ constants for PointerPrefs.pp_Which }
 WBP_NORMAL    =  0;
 WBP_BUSY      =  1;

{***************************************************************************}

Type
 pRGBTable = ^tRGBTable;
 tRGBTable = record
    t_Red,
    t_Green,
    t_Blue  : Byte;
 end;

{***************************************************************************}

  {    File format for preferences header }

const
 ID_PREF = 1347568966;
 ID_PRHD = 1347569732;

Type
 pPrefHeader = ^tPrefHeader;
 tPrefHeader = record
    ph_Version,             { version of following data }
    ph_Type     : Byte;     { type of following data    }
    ph_Flags    : ULONG;    { always set to 0 for now   }
 end;

  {     File format for graphics printer preferences }

const
 ID_PGFX = 1346848344;

Type
 pPrinterGfxPrefs = ^tPrinterGfxPrefs;
 tPrinterGfxPrefs = record
    pg_Reserved     : Array[0..3] of Longint;
    pg_Aspect,
    pg_Shade,
    pg_Image        : Word;
    pg_Threshold    : smallint;
    pg_ColorCorrect,
    pg_Dimensions,
    pg_Dithering    : Byte;
    pg_GraphicFlags : WORD;
    pg_PrintDensity : Byte;              { Print density 1 - 7 }
    pg_PrintMaxWidth,
    pg_PrintMaxHeight : WORD;
    pg_PrintXOffset,
    pg_PrintYOffset : Byte;
 end;

const
{ constants for PrinterGfxPrefs.pg_Aspect }
 PA_HORIZONTAL = 0;
 PA_VERTICAL   = 1;

{ constants for PrinterGfxPrefs.pg_Shade }
 PS_BW          = 0;
 PS_GREYSCALE   = 1;
 PS_COLOR       = 2;
 PS_GREY_SCALE2 = 3;

{ constants for PrinterGfxPrefs.pg_Image }
 PI_POSITIVE = 0;
 PI_NEGATIVE = 1;

{ flags for PrinterGfxPrefs.pg_ColorCorrect }
 PCCB_RED   = 1;    { color correct red shades   }
 PCCB_GREEN = 2;    { color correct green shades }
 PCCB_BLUE  = 3;    { color correct blue shades  }

 PCCF_RED   = 1;
 PCCF_GREEN = 2;
 PCCF_BLUE  = 4;

{ constants for PrinterGfxPrefs.pg_Dimensions }
 PD_IGNORE   = 0;  { ignore max width/height settings }
 PD_BOUNDED  = 1;  { use max w/h as boundaries        }
 PD_ABSOLUTE = 2;  { use max w/h as absolutes         }
 PD_PIXEL    = 3;  { use max w/h as prt pixels        }
 PD_MULTIPLY = 4;  { use max w/h as multipliers       }

{ constants for PrinterGfxPrefs.pg_Dithering }
 PD_ORDERED     = 0;  { ordered dithering }
 PD_HALFTONE    = 1;  { halftone dithering        }
 PD_FLOYD       = 2;  { Floyd-Steinberg dithering }

{ flags for PrinterGfxPrefs.pg_GraphicsFlags }
 PGFB_CENTER_IMAGE      = 0;       { center image on paper }
 PGFB_INTEGER_SCALING   = 1;       { force integer scaling }
 PGFB_ANTI_ALIAS        = 2;       { anti-alias image      }

 PGFF_CENTER_IMAGE      = 1;
 PGFF_INTEGER_SCALING   = 2;
 PGFF_ANTI_ALIAS        = 4;


 {      File format for PostScript printer preferences   }

const
 ID_PSPD = 1347637316;

Type
 pPrinterPSPrefs = ^tPrinterPSPrefs;
 tPrinterPSPrefs = record
    ps_Reserved     : Array[0..3] of Longint;               { System reserved }

    { Global printing attributes }
    ps_DriverMode,
    ps_PaperFormat  : Byte;
    ps_Reserved1    : Array[0..1] of Byte;
    ps_Copies,
    ps_PaperWidth,
    ps_PaperHeight,
    ps_HorizontalDPI,
    ps_VerticalDPI  : Longint;

    { Text Options }
    ps_Font,
    ps_Pitch,
    ps_Orientation,
    ps_Tab          : Byte;
    ps_Reserved2    : Array[0..7] of Byte;

    { Text Dimensions }
    ps_LeftMargin,
    ps_RightMargin,
    ps_TopMargin,
    ps_BottomMargin,
    ps_FontPointSize,
    ps_Leading      : Longint;
    ps_Reserved3    : Array[0..7] of Byte;

    { Graphics Options }
    ps_LeftEdge,
    ps_TopEdge,
    ps_Width,
    ps_Height       : Longint;
    ps_Image,
    ps_Shading,
    ps_Dithering    : Byte;
    ps_Reserved4    : Array[0..8] of Byte;

    { Graphics Scaling }
    ps_Aspect,
    ps_ScalingType,
    ps_Reserved5,
    ps_Centering    : Byte;
    ps_Reserved6    : Array[0..7] of byte;
 end;

const
{ All measurements are in Millipoints which is 1/1000 of a point, or
 * in other words 1/72000 of an inch
 }

{ constants for PrinterPSPrefs.ps_DriverMode }
 DM_POSTSCRIPT  = 0;
 DM_PASSTHROUGH = 1;

{ constants for PrinterPSPrefs.ps_PaperFormat }
 PF_USLETTER = 0;
 PF_USLEGAL  = 1;
 PF_A4       = 2;
 PF_CUSTOM   = 3;

{ constants for PrinterPSPrefs.ps_Font }
 FONT_COURIER      = 0;
 FONT_TIMES        = 1;
 FONT_HELVETICA    = 2;
 FONT_HELV_NARROW  = 3;
 FONT_AVANTGARDE   = 4;
 FONT_BOOKMAN      = 5;
 FONT_NEWCENT      = 6;
 FONT_PALATINO     = 7;
 FONT_ZAPFCHANCERY = 8;

{ constants for PrinterPSPrefs.ps_Pitch }
 PITCH_NORMAL     = 0;
 PITCH_COMPRESSED = 1;
 PITCH_EXPANDED   = 2;

{ constants for PrinterPSPrefs.ps_Orientation }
 ORIENT_PORTRAIT  = 0;
 ORIENT_LANDSCAPE = 1;

{ constants for PrinterPSPrefs.ps_Tab }
 TAB_4     = 0;
 TAB_8     = 1;
 TAB_QUART = 2;
 TAB_HALF  = 3;
 TAB_INCH  = 4;

{ constants for PrinterPSPrefs.ps_Image }
 IM_POSITIVE = 0;
 IM_NEGATIVE = 1;

{ constants for PrinterPSPrefs.ps_Shading }
 SHAD_BW        = 0;
 SHAD_GREYSCALE = 1;
 SHAD_COLOR     = 2;

{ constants for PrinterPSPrefs.ps_Dithering }
 DITH_DEFAULT = 0;
 DITH_DOTTY   = 1;
 DITH_VERT    = 2;
 DITH_HORIZ   = 3;
 DITH_DIAG    = 4;

{ constants for PrinterPSPrefs.ps_Aspect }
 ASP_HORIZ = 0;
 ASP_VERT  = 1;

{ constants for PrinterPSPrefs.ps_ScalingType }
 ST_ASPECT_ASIS    = 0;
 ST_ASPECT_WIDE    = 1;
 ST_ASPECT_TALL    = 2;
 ST_ASPECT_BOTH    = 3;
 ST_FITS_WIDE      = 4;
 ST_FITS_TALL      = 5;
 ST_FITS_BOTH      = 6;

{ constants for PrinterPSPrefs.ps_Centering }
 CENT_NONE  = 0;
 CENT_HORIZ = 1;
 CENT_VERT  = 2;
 CENT_BOTH  = 3;


   {   File format for text printer preferences }

const
 ID_PTXT = 1347704916;
 ID_PUNT = 1347767892;
 ID_PDEV = $50444556;

 DRIVERNAMESIZE = 30;               { Filename size     }
 DEVICENAMESIZE = 32;               { .device name size }
 UNITNAMESIZE   = 32;

Type
 pPrinterTxtPrefs = ^tPrinterTxtPrefs;
 tPrinterTxtPrefs = record
    pt_Reserved     : Array[0..3] of Longint;               { System reserved            }
    pt_Driver       : Array[0..DRIVERNAMESIZE-1] of Char;   { printer driver filename    }
    pt_Port         : Byte;                                 { printer port connection    }

    pt_PaperType,
    pt_PaperSize,
    pt_PaperLength,               { Paper length in # of lines }

    pt_Pitch,
    pt_Spacing,
    pt_LeftMargin,                { Left margin                }
    pt_RightMargin,               { Right margin       }
    pt_Quality      : WORD;
 end;

const
{ constants for PrinterTxtPrefs.pt_Port }
 PP_PARALLEL = 0;
 PP_SERIAL   = 1;

{ constants for PrinterTxtPrefs.pt_PaperType }
 PT_FANFOLD  = 0;
 PT_SINGLE   = 1;

{ constants for PrinterTxtPrefs.pt_PaperSize }
 PS_US_LETTER   = 0 ;
 PS_US_LEGAL    = 1 ;
 PS_N_TRACTOR   = 2 ;
 PS_W_TRACTOR   = 3 ;
 PS_CUSTOM      = 4 ;
 PS_EURO_A0     = 5 ;              { European size A0: 841 x 1189 }
 PS_EURO_A1     = 6 ;              { European size A1: 594 x 841  }
 PS_EURO_A2     = 7 ;              { European size A2: 420 x 594  }
 PS_EURO_A3     = 8 ;              { European size A3: 297 x 420  }
 PS_EURO_A4     = 9 ;              { European size A4: 210 x 297  }
 PS_EURO_A5     = 10;              { European size A5: 148 x 210  }
 PS_EURO_A6     = 11;              { European size A6: 105 x 148  }
 PS_EURO_A7     = 12;              { European size A7: 74 x 105   }
 PS_EURO_A8     = 13;              { European size A8: 52 x 74    }

{ constants for PrinterTxtPrefs.pt_PrintPitch }
 PP_PICA  = 0;
 PP_ELITE = 1;
 PP_FINE  = 2;

{ constants for PrinterTxtPrefs.pt_PrintSpacing }
 PS_SIX_LPI   = 0;
 PS_EIGHT_LPI = 1;

{ constants for PrinterTxtPrefs.pt_PrintQuality }
 PQ_DRAFT  = 0;
 PQ_LETTER = 1;


{ PrinterUnitPrefs is used from printer.device to open
   the connection device
}

Type
 pPrinterUnitPrefs = ^tPrinterUnitPrefs;
 tPrinterUnitPrefs = record
    pu_Reserved         : Array[0..3] of Longint;              { System reserved              }
    pu_UnitNum          : Longint;                             { Unit number for OpenDevice() }
    pu_OpenDeviceFlags  : ULONG;                               { Flags for OpenDevice()       }
    pu_DeviceName       : Array[0..DEVICENAMESIZE-1] of Char;  { Name for OpenDevice()        }
 end;


  { PrinterDeviceUnitPrefs is used as descriptor for printer device
     units.
   }
     PPrinterDeviceUnitPrefs = ^tPrinterDeviceUnitPrefs;
     tPrinterDeviceUnitPrefs = record
          pd_Reserved : array[0..3] of LONG;   { System reserved                   }
          pd_UnitNum : LONG;                   { Unit number for OpenDevice()      }
          pd_UnitName : array[0..(UNITNAMESIZE)-1] of UBYTE;  { Symbolic name of the unit  }
       end;

{ Reaction }
  const

     ID_RACT = $52414354;


  type
     PReactionPrefs = ^tReactionPrefs;
     tReactionPrefs = record
          rp_BevelType : UWORD;
          rp_GlyphType : UWORD;
          rp_LayoutSpacing : UWORD;
          rp_3DProp : BOOL;
          rp_LabelPen : UWORD;
          rp_LabelPlace : UWORD;
          rp_3DLabel : BOOL;
          rp_SimpleRefresh : BOOL;
          rp_3DLook : BOOL;
          rp_FallbackAttr : tTextAttr;
          rp_LabelAttr : tTextAttr;
          rp_FallbackName : array[0..(FONTNAMESIZE)-1] of UBYTE;
          rp_LabelName : array[0..(FONTNAMESIZE)-1] of UBYTE;
          rp_Pattern : array[0..255] of UBYTE;
       end;


  {     File format for screen mode preferences }

const
 ID_SCRM = 1396920909;


Type
 pScreenModePrefs = ^tScreenModePrefs;
 tScreenModePrefs = record
    smp_Reserved        : Array[0..3] of ULONG;
    smp_DisplayID       : ULONG;
    smp_Width,
    smp_Height,
    smp_Depth,
    smp_Control         : Word;
 end;

const
{ flags for ScreenModePrefs.smp_Control }
 SMB_AUTOSCROLL = 1;

 SMF_AUTOSCROLL = 1;

 {      File format for serial preferences }

const
 ID_SERL = 1397051980;


Type
 pSerialPrefs = ^tSerialPrefs;
 tSerialPrefs = record
    sp_Reserved     : Array[0..2] of Longint;               { System reserved                  }
    sp_Unit0Map,                  { What unit 0 really refers to     }
    sp_BaudRate,                  { Baud rate                        }

    sp_InputBuffer,               { Input buffer: 0 - 65536          }
    sp_OutputBuffer : ULONG;      { Future: Output: 0 - 65536        }

    sp_InputHandshake,            { Input handshaking                }
    sp_OutputHandshake,           { Future: Output handshaking       }

    sp_Parity,                    { Parity                           }
    sp_BitsPerChar,               { I/O bits per character           }
    sp_StopBits     : Byte;       { Stop bits                        }
 end;

const
{ constants for SerialPrefs.sp_Parity }
 PARITY_NONE     = 0;
 PARITY_EVEN     = 1;
 PARITY_ODD      = 2;
 PARITY_MARK     = 3;               { Future enhancement }
 PARITY_SPACE    = 4;               { Future enhancement }

{ constants for SerialPrefs.sp_Input/OutputHandshaking }
 HSHAKE_XON      = 0;
 HSHAKE_RTS      = 1;
 HSHAKE_NONE     = 2;

    {   File format for sound preferences  }

const
 ID_SOND = 1397706308;

Type
 pSoundPrefs = ^tSoundPrefs;
 tSoundPrefs = record
    sop_Reserved        : Array[0..3] of Longint;            { System reserved            }
    sop_DisplayQueue,               { Flash the display?         }
    sop_AudioQueue      : Boolean;  { Make some sound?           }
    sop_AudioType,                  { Type of sound, see below   }
    sop_AudioVolume,                { Volume of sound, 0..64     }
    sop_AudioPeriod,                { Period of sound, 127..2500 }
    sop_AudioDuration   : WORD;     { Length of simple beep      }
    sop_AudioFileName   : Array[0..255] of Char;     { Filename of 8SVX file      }
 end;

const
{ constants for SoundPrefs.sop_AudioType }
 SPTYPE_BEEP    = 0;       { simple beep sound }
 SPTYPE_SAMPLE  = 1;       { sampled sound     }

   {   File format for wbpattern preferences  }

const
 ID_PTRN = 1347703374;

Type
 pWBPatternPrefs = ^tWBPatternPrefs;
 tWBPatternPrefs = record
    wbp_Reserved    : Array[0..3] of ULONG;
    wbp_Which,                     { Which pattern is it }
    wbp_Flags       : WORD;
    wbp_Revision,                  { Must be set to zero }
    wbp_Depth       : Byte;        { Depth of pattern }
    wbp_DataLength  : WORD;        { Length of following data }
 end;

const
{ constants for WBPatternPrefs.wbp_Which }
 WBP_ROOT       = 0;
 WBP_DRAWER     = 1;
 WBP_SCREEN     = 2;

{ wbp_Flags values }
 WBPF_PATTERN   = $0001;
    { Data contains a pattern }

 WBPF_NOREMAP   = $0010;
    { Don't remap the pattern }
  { PDTA_DitherQuality: see pictureclass.h  }
     WBPF_DITHER_MASK = $0300;
  { DitherQuality: Default  }
     WBPF_DITHER_DEF = $0000;
  { DitherQuality: 0  }
     WBPF_DITHER_BAD = $0100;
  { DitherQuality: 2  }
     WBPF_DITHER_GOOD = $0200;
  { DitherQuality: 4  }
     WBPF_DITHER_BEST = $0300;
  { OBP_Precision: see pictureclass.h  }
     WBPF_PRECISION_MASK = $0C00;
     WBPF_PRECISION_DEF = $0000;
     WBPF_PRECISION_ICON = $0400;
     WBPF_PRECISION_IMAGE = $0800;
     WBPF_PRECISION_EXACT = $0C00;
     WBPF_PLACEMENT_MASK = $3000;
     WBPF_PLACEMENT_TILE = $0000;
     WBPF_PLACEMENT_CENTER = $1000;
     WBPF_PLACEMENT_SCALE = $2000;
     WBPF_PLACEMENT_SCALEGOOD = $3000;

{***************************************************************************}

 MAXDEPTH       = 3;       {  Max depth supported (8 colors) }
 DEFPATDEPTH    = 2;       {  Depth of default patterns }

{  Pattern width & height: }
 PAT_WIDTH      = 16;
 PAT_HEIGHT     = 16;

{ Workbench }

 ID_WBNC = $57424E43;

 type
     PWorkbenchPrefs = ^tWorkbenchPrefs;
     tWorkbenchPrefs = record
          { settings from workbench.library }
          wbp_DefaultStackSize : ULONG;
          wbp_TypeRestartTime : ULONG;
          { settings from icon.library }
          wbp_IconPrecision : ULONG;
          wbp_EmbossRect : tRectangle;
          wbp_Borderless : BOOL;
          wbp_MaxNameLength : LONG;
          wbp_NewIconsSupport : BOOL;
          wbp_ColorIconSupport : BOOL;
           { new for V45 }
          wbp_ImageMemType : ULONG;
          wbp_LockPens : BOOL;
          wbp_NoTitleBar : BOOL;
          wbp_NoGauge : BOOL;
       end;

     PWorkbenchHiddenDevicePrefs = ^tWorkbenchHiddenDevicePrefs;
     tWorkbenchHiddenDevicePrefs = record
          whdp_Name : array[0..0] of UBYTE;  { C String including NULL char  }
       end;

const
     ID_WBHD = $57424844;

IMPLEMENTATION

END.
