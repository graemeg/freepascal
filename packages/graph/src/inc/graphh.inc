{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Interface include file for graph unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{-------------------------------------------------------}
{ Differences with TP Graph unit:                       }
{ -  default putimage and getimage only support a max.  }
{    of 64K colors on screen, because all pixels are    }
{    saved as words.                                    }
{ -  Set RGB Palette is not used, SetPalette must be    }
{    used instead.                                      }
{ -  In the TP graph unit, Clipping is always performed }
{    on strings written with OutText, and this clipping }
{    is done on a character per character basis (for    }
{    example, if ONE part of a character is outside the }
{    viewport , then that character is not written at   }
{    all to the screen. In FPC Pascal, clipping is done }
{    on a PIXEL basis, not a character basis, so part of }
{    characters which are not entirely in the viewport  }
{    may appear on the screen.                          }
{ -  SetTextStyle only conforms to the TP version when  }
{    the correct (and expected) values are used for     }
{    CharSize for stroked fonts (4 = stroked fonts)     }
{ -  InstallUserDriver is not supported, so always      }
{    returns an error.                                  }
{ -  RegisterBGIDriver is not supported, so always      }
{    returns an error.                                  }
{ - DrawPoly XORPut mode is not exactly the same as in  }
{   the TP graph unit.                                  }
{ - Imagesize returns a longint instead of a word       }
{ - ImageSize cannot return an error value              }
{-------------------------------------------------------}
{ AUTHORS:                                                                      }
{   Gernot Tenchio      - original version              }
{   Florian Klaempfl    - major updates                 }
{   Pierre Mueller      - major bugfixes                }
{   Carl Eric Codere    - complete rewrite              }
{   Thomas Schatzl      - optimizations,routines and    }
{                           suggestions.                }
{   Jonas Maebe         - bugfixes and optimizations    }
{ Credits (external):                                   }
{   - Original FloodFill code by                        }
{        Menno Victor van der star                      }
{     (the code has been heavily modified)              }
{-------------------------------------------------------}
{-------------------------------------------------------}
{ For significant speed improvements , is is recommended }
{ that these routines be hooked (otherwise the default,  }
{ slower routines will be used) :                        }
{   HLine()                                              }
{   VLine()                                              }
{   PatternLine()                                        }
{   ClearViewPort()                                      }
{   PutImage()                                           }
{   GetImage()  - ImageSize() should also be changed     }
{   InternalEllipse()                                    }
{   Line()                                               }
{   GetScanLine()                                        }
{--------------------------------------------------------}
{ FPC unit requirements:                                 }
{  All modes should at least have 1 graphics page to     }
{  make it possible to create animation on all supported }
{  systems , this can be done either by double-buffering }
{  yourself in the heap if no hardware is available to do}
{  it.                                                   }
{--------------------------------------------------------}
{ COMPATIBILITY WARNING: Some of the compatibility tests }
{ were done using the CGA and other the VGA drivers.     }
{ Within the BGI drivers themselves the BEHAVIOUR is not }
{ the same, so be warned!!!                              }
{--------------------------------------------------------}
{ History log:                                           }
{   15th February 1999:                                  }
{   + Added support for system font in vertical mode     }
{   + system font is now available for all platforms     }
{   * font support routines now compile                  }
{   * textHeight would not return correct size for system }
{     font                                               }
{   * Alignment of fonts partly fixed                    }
{   17th Feb. 1999:                                      }
{   + First support for stroked fonts                    }
{   18th Feb. 1999:                                      }
{   * bugfix of line drawing which fixes stroked font    }
{     displays.                                          }
{   23rd Feb. 1999:                                      }
{   + Applied Pierre's patches to font                   }
{   + Added scaling of bitmapped fonts                   }
{   + Vertical stroked fonts                             }
{  24th Feb. 1999:                                       }
{   * Scaling of stroked fonts must be done using FPs    }
{     to be 100% compatible with turbo pascal            }
{   + Sped up by 40% stroked font scaling calculations   }
{   + RegisterBGIFont                                    }
{  9th march 1999:                                       }
{   + Starting implementing Fillpoly()                   }
{  15th march 1999:                                      }
{   + SetFillStyle()                                     }
{   + patternLine()                                      }
{   + Bar()                                              }
{   * GraphDefaults would not make the Default color     }
{     of the fill pattern to the Max. Palette entry.     }
{   + SetFillPattern()                                   }
{  20th march 1999:                                      }
{   * GraphDefaults would not reset to the text system   }
{   * DefaultFont would write one character too much to  }
{     the screen                                         }
{   + Sloped thick lines in Line()                       }
{   + Sloped patterned lines in Line()                   }
{   * GraphDefaults would not reset the User Fill pattern}
{     to $ff                                             }
{   + DirectPutPixel takes care of XOR mode writes       }
{     improves speed by about 30% over old method of     }
{     GetPixel XOR CurrentColor                          }
{   * Dashed LineStyle exactly like BP version now       }
{   + Center LineStyle (checked against CGA driver)      }
{   * GraphDefaults() now resets linepattern array       }
{ 1st april  1999:                                       }
{   + First implementation of FillPoly (incomplete)      }
{ 2nd april  1999:                                       }
{   * FillPoly did not Reset PatternLine index           }
{   * FillPoly did not use correct color                 }
{   * PatternLine was writing modes in reverse direction }
{   * PatternLine would not work with non-rectangular    }
{     shapes.                                            }
{   * PatternLine must fill up the ENTIRE pattern,       }
{     with either the foreground or background color.    }
{   * GraphDefaults() would not call SetBkColor()        }
{   * Fixed some memory leaks in FillPoly()              }
{ 11th April  1999:                                      }
{   * PatternLine() was drawing one pixel less then      }
{     requested                                          }
{ 12th April  1999:                                      }
{   + FloodFill - first working implementation           }
{      Horrbly slow even on very fast cpu's              }
{   + Some suggestions of Thomas implemented             }
{ 13th April  1999:                                      }
{   * FloodFill() vertical index was off by one pixel    }
{   * FloodFill() would never draw the last line in the  }
{      list                                              }
{   - Removed ClearViewPort320 which was wrong anyways,  }
{     will need to be implemented later.                 }
{   * PatternLine() would not always restore write mode  }
{   + Circle() uses NormalPut always with NormWidth lines}
{   + FillEllipse() initial version                      }
{   * InternalEllipse() - 0 to 360 now supported as      }
{     angles.                                            }
{ 14th April  1999:                                      }
{   * mod x = and (x-1)(from Thomas Schatzl) gives a     }
{     significant speed improvement.                     }
{ 15th april  1999:                                      }
{   + Arc() ok except for Aspect Ratio, which does not   }
{     give us the correct ratio on a 320x200 screen.     }
{   + Added FillPoly() from Thomas Schatzl               }
{   + More hookable routines                             }
{  16th april  1999:                                     }
{   + Line() checked ok.                                 }
{  17th april  1999:                                     }
{   * GraphDefaults() would not reset CP                 }
{   + GetX(), GetY(), MoveTo() checked for viewports     }
{   * OutTextXY() should not update the CP               }
{   * ClearViewPort() would not update the CP            }
{   * ClearDevice() would not update the CP              }
{   * Sector() would update the CP by calling LineTo     }
{   * Bar3D() would update the CP                        }
{   * PieSlice() would update the CP                     }
{  18th april  1999:                                     }
{   + Clipping algorithm                                 }
{  19th april  1999:                                     }
{   + Adapterinfo structure                              }
{  20th april 1999:                                      }
{   + GetModeName                                        }
{   + GetGraphMode                                       }
{   + GetModeRange                                       }
{--------------------------------------------------------}


type
    smallint = -32768..32767;

    TResolutionRec = record
      x,y: longint;
    end;

    const
       maxsmallint = high(smallint);
       { error codes }
       grOk =  0;
       grNoInitGraph = -1;
       grNotDetected = -2;
       grFileNotFound = -3;
       grInvalidDriver = -4;
       grNoLoadMem = -5;
       grNoScanMem = -6;
       grNoFloodMem = -7;
       grFontNotFound = -8;
       grNoFontMem = -9;
       grInvalidMode = -10;
       grError = -11;
       grIOerror = -12;
       grInvalidFont = -13;
       grInvalidFontNum = -14;
       grInvalidVersion = -18;


       { Color constants for setpalette }
       black     = 0;
       blue      = 1;
       green     = 2;
       cyan      = 3;
       red       = 4;
       magenta   = 5;
       brown     = 6;
       lightgray = 7;
       darkgray  = 8;
       lightblue = 9;
       lightgreen = 10;
       lightcyan = 11;
       lightred  = 12;
       lightmagenta = 13;
       yellow    = 14;
       white     = 15;

       EGABlack    =  0;
       EGABlue     =  1;
       EGAGreen    =  2;
       EGACyan     =  3;
       EGARed      =  4;
       EGAMagenta  =  5;
       EGALightgray=  7;
       EGABrown    = 20;
       EGADarkgray   = 56;
       EGALightblue  = 57;
       EGALightgreen = 58;
       EGALightcyan  = 59;
       EGALightred   = 60;
       EGALightmagenta=61;
       EGAYellow     = 62;
       EGAWhite      = 63;



       { Line styles for GetLineStyle/SetLineStyle }
       SolidLn = 0;
       DottedLn = 1;
       CenterLn = 2;
       DashedLn = 3;
       UserBitLn = 4;

       NormWidth = 1;
       ThickWidth = 3;

       { Set/GetTextStyle Konstanten: }
       DefaultFont = 0;
       TriplexFont = 1;
       SmallFont = 2;
       SansSerifFont = 3;
       GothicFont = 4;
       ScriptFont = 5;
       SimpleFont = 6;
       TSCRFont = 7;
       LCOMFont = 8;
       EuroFont = 9;
       BoldFont = 10;

       HorizDir = 0;
       VertDir = 1;

       UserCharSize = 0;

       ClipOn = true;
       ClipOff = false;

       { Bar3D constants }
       TopOn = true;
       TopOff = false;

       { fill pattern for Get/SetFillStyle: }
       EmptyFill      = 0;
       SolidFill      = 1;
       LineFill       = 2;
       LtSlashFill    = 3;
       SlashFill      = 4;
       BkSlashFill    = 5;
       LtBkSlashFill  = 6;
       HatchFill      = 7;
       XHatchFill     = 8;
       InterleaveFill = 9;
       WideDotFill    = 10;
       CloseDotFill   = 11;
       UserFill       = 12;

       { bitblt operators  }
       NormalPut     = 0;
       CopyPut       = 0;
       XORPut        = 1;
       OrPut         = 2;
       AndPut        = 3;
       NotPut        = 4;

       { SetTextJustify constants }
       LeftText   = 0;
       CenterText = 1;
       RightText  = 2;

       BottomText = 0;
       TopText    = 2;

       { graphic drivers }
       CurrentDriver = -128;
       Detect        = 0;
       LowRes        = 1;
       HercMono      = 7;
       VGA           = 9;
       VESA          = 10;


       D1bit = 11;
       D2bit = 12;
       D4bit = 13;
       D6bit = 14;  { 64 colors Half-brite mode - Amiga }
       D8bit = 15;
       D12bit = 16; { 4096 color modes HAM mode - Amiga }
       D15bit = 17;
       D16bit = 18;
       D24bit = 19; { not yet supported }
       D32bit = 20; { not yet supported }
       D64bit = 21; { not yet supported }

       lowNewDriver = 11;
       highNewDriver = 21;


       { graph modes }
       Default = 0;

       { VGA Driver modes }
       VGALo   = 0;
       VGAMed  = 1;
       VGAHi   = 2;

       { They start at such a high number to make sure they don't clash     }
       { with the mode numbers returned by getmoderange (those mode numbers }
       { are about the same modes, but all supported modes must have a      }
       { consecutive mode number for that, so they are generated on-the-fly }
       { starting from 1)                                                   }

       detectMode = 30000;
       m320x200 = 30001;
       m320x256 = 30002; { amiga resolution (PAL) }
       m320x400 = 30003; { amiga/atari resolution }
       m512x384 = 30004; { mac resolution }
       m640x200 = 30005; { vga resolution }
       m640x256 = 30006; { amiga resolution (PAL) }
       m640x350 = 30007; { vga resolution }
       m640x400 = 30008;
       m640x480 = 30009;
       m800x600 = 30010;
       m832x624 = 30011; { mac resolution }
       m1024x768 = 30012;
       m1280x1024 = 30013;
       m1600x1200 = 30014;
       m2048x1536 = 30015;

       lowNewMode = 30001;
       highNewMode = 30015;

       resolutions: array[lowNewMode..highNewMode] of TResolutionRec =
       ((x:320;y:200),
        (x:320;y:256),
        (x:320;y:400),
        (x:512;y:384),
        (x:640;y:200),
        (x:640;y:256),
        (x:640;y:350),
        (x:640;y:400),
        (x:640;y:480),
        (x:800;y:600),
        (x:832;y:624),
        (x:1024;y:768),
        (x:1280;y:1024),
        (x:1600;y:1200),
        (x:2048;y:1536));


       { Hercules mono card }
       HercMonoHi = 0;

       MaxColors   = 255;   { Maximum possible colors using a palette }
                            { otherwise, direct color encoding        }


    type
       RGBRec = packed record
         Red: smallint;
         Green: smallint;
         Blue : smallint;
       end;

       PaletteType = record
             Size   : longint;
             Colors : array[0..MaxColors] of RGBRec;
       end;

       LineSettingsType = record
             linestyle : word;
             pattern : word;
             thickness : word;
       end;

       TextSettingsType = record
             font : word;
             direction : word;
             charsize : word;
             horiz : word;
             vert : word;
       end;

       FillSettingsType = record
             pattern : word;
             color : word;
       end;

       FillPatternType = array[1..8] of byte;

       PointType = record
             x,y : smallint;
       end;

       ViewPortType = record
             x1,y1,x2,y2 : smallint;
             Clip : boolean;
       end;

       ArcCoordsType = record
             x,y : smallint;
             xstart,ystart : smallint;
             xend,yend : smallint;
       end;

        graph_float = single;   { the platform's preferred floating point size }

  const
       fillpatternTable : array[0..12] of FillPatternType = (
           ($00,$00,$00,$00,$00,$00,$00,$00),     { background color  }
           ($ff,$ff,$ff,$ff,$ff,$ff,$ff,$ff),     { foreground color  }
           ($ff,$ff,$00,$00,$ff,$ff,$00,$00),     { horizontal lines  }
           ($01,$02,$04,$08,$10,$20,$40,$80),     { slashes           }
           ($07,$0e,$1c,$38,$70,$e0,$c1,$83),     { thick slashes     }
           ($07,$83,$c1,$e0,$70,$38,$1c,$0e),     { thick backslashes }
           ($5a,$2d,$96,$4b,$a5,$d2,$69,$b4),     { backslashes       }
           ($ff,$88,$88,$88,$ff,$88,$88,$88),     { small boxes       }
           ($18,$24,$42,$81,$81,$42,$24,$18),     { rhombus           }
           ($cc,$33,$cc,$33,$cc,$33,$cc,$33),     { wall pattern      }
           ($80,$00,$08,$00,$80,$00,$08,$00),     { wide points       }
           ($88,$00,$22,$00,$88,$00,$22,$00),     { dense points      }
           (0,0,0,0,0,0,0,0)                      { user defined line style }
          );



  { ******************** PROCEDURAL VARIABLES ********************* }
  { * These are hooks which have device specific stuff in them,   * }
  { * therefore to add new modes or to redirect these routines    * }
  { * then declare variables of these types as shown below.       * }
  {-----------------------------------------------------------------}

TYPE


       { This is the standard putpixel routine used by all function }
       { drawing routines, it will use the viewport settings, as    }
       { well as clip, and use the current foreground color to plot }
       { the desired pixel.                                         }
       defpixelproc = procedure(X,Y: smallint);

       { standard plot and get pixel                                }
       getpixelproc = function(X,Y: smallint): word;
       putpixelproc = procedure(X,Y: smallint; Color: Word);

       { clears the viewport, also used to clear the device         }
       clrviewproc  = procedure;

       { putimage procedure, can be hooked to accomplish transparency }
       putimageproc = procedure (X,Y: smallint; var Bitmap; BitBlt: Word);
       getimageproc = procedure(X1,Y1,X2,Y2: smallint; Var Bitmap);
       imagesizeproc= function (X1,Y1,X2,Y2: smallint): longint;

       graphfreememprc = procedure (var P: Pointer; size: word);
       graphgetmemprc  = procedure (var P: pointer; size: word);

       { internal routines -- can be hooked for much faster drawing }

       { draw filled horizontal lines using current color }
       { on entry coordinates are already clipped.        }
       hlineproc = procedure (x, x2,y : smallint);
       { on entry coordinates are already clipped.        }
       { draw filled vertical line using current color    }
       vlineproc = procedure (x,y,y2: smallint);

       { this routine is used to draw filled patterns for all routines }
       { that require it. (FillPoly, FloodFill, Sector, etc...         }
       { clipping is verified, uses current Fill settings for drawing  }
       patternlineproc = procedure (x1,x2,y: smallint);

       { this routine is used to draw all circles/ellipses/sectors     }
       { more info... on this later...                                 }
       ellipseproc = procedure (X,Y: smallint;XRadius: word;
         YRadius:word; stAngle,EndAngle: word; fp: PatternLineProc);

       { Line routine - draws lines thick/norm widths with current     }
       { color and line style - LINE must be clipped here.             }
       lineproc = procedure (X1, Y1, X2, Y2 : smallint);

       { this routine is used for FloodFill - it returns an entire      }
       { screen scan line with a word for each pixel in the scanline.   }
       { Also handy for GetImage, so I added x coords as well (JM)      }
       getscanlineproc = procedure (X1, X2, Y : smallint; var data);

       { changes the active display screen where we draw to... }
       setactivepageproc = procedure (page: word);

       { changes the active display screen which we see ... }
       setvisualpageproc = procedure (page: word);

       { this routine actually switches to the desired video mode.     }
       initmodeproc = procedure;

       { this routine is called to save the sate just before a mode set }
       savestateproc = procedure;
       { this routine is called in closegraph to cleanup...             }
       restorestateproc = procedure;

       { This routine is a hook for SetRGBPalette                       }
       setrgbpaletteproc =
         procedure(ColorNum, RedValue, GreenValue, BlueValue: smallint);

       { This routine is a hook for GetRGBPalette                       }
       getrgbpaletteproc =
         procedure(ColorNum: smallint; var
            RedValue, GreenValue, BlueValue: smallint);

       SetAllPaletteProc = procedure(const Palette:PaletteType);

       OutTextXYProc = procedure(x,y : SmallInt;const TextString : string);

       CircleProc = procedure(X, Y: smallint; Radius:Word);


TYPE
    {-----------------------------------}
    { Linked list for mode information  }
    { This list is set up by one of the }
    { following routines:               }
    { It lists all available resolutions}
    { on this display adapter.          }
    {-----------------------------------}
    {   QueryAdapter()                  }
    {   DetectGraph()                   }
    {   InitGraph()                     }
    {-----------------------------------}
    PModeInfo = ^TModeInfo;
    TModeInfo = record
      DriverNumber: smallint;
      ModeNumber: smallint;
      internModeNumber: smallint;
      MaxColor: Longint;            { Maximum colors on screen        }
      PaletteSize : Longint;        { Maximum palette entry we can change }
      XAspect : word;            { XAspect ratio correction factor }
      YAspect : word;            { YAspect ratio correction factor }
      MaxX: word;                { Max-X row                       }
      MaxY: word;                { Max. column.                    }
      DirectColor: boolean;         { Is this a direct color mode??   }
      Hardwarepages: byte;          { total number of image pages - 1 }
      ModeName: String[18];
      { necessary hooks ... }
      DirectPutPixel : DefPixelProc;
      GetPixel       : GetPixelProc;
      PutPixel       : PutPixelProc;
      SetRGBPalette  : SetRGBPaletteProc;
      GetRGBPalette  : GetRGBPaletteProc;
      SetAllPalette  : SetAllPaletteProc;
      { defaults possible ... }
      SetVisualPage  : SetVisualPageProc;
      SetActivePage  : SetActivePageProc;
      ClearViewPort  : ClrViewProc;
      PutImage       : PutImageProc;
      GetImage       : GetImageProc;
      ImageSize      : ImageSizeProc;
      GetScanLine    : GetScanLineProc;
      Line           : LineProc;
      InternalEllipse: EllipseProc;
      PatternLine    : PatternLineProc;
      HLine          : HLineProc;
      VLine          : VLineProc;
      Circle         : CircleProc;
      InitMode       : InitModeProc;
      OutTextXY : OutTextXYProc;
      next: PModeInfo;
    end;

    TNewModeInfo = record
      modeInfo: array[lowNewDriver..highNewDriver] of PModeInfo;
      loHiModeNr: array[lowNewDriver..highNewDriver] of record
        lo,hi: smallint;
      end;
    end;



VAR
  DirectPutPixel : DefPixelProc;
  ClearViewPort  : ClrViewProc;
  PutPixel       : PutPixelProc;
  PutImage       : PutImageProc;
  GetImage       : GetImageProc;
  ImageSize      : ImageSizeProc;
  GetPixel       : GetPixelProc;
  SetVisualPage  : SetVisualPageProc;
  SetActivePage  : SetActivePageProc;
  SetRGBPalette  : SetRGBPaletteProc;
  GetRGBPalette  : GetRGBPaletteProc;
  SetAllPalette  : SetAllPaletteProc;
  OutTextXY      : OutTextXYProc;

  GraphFreeMemPtr: graphfreememprc;
  GraphGetMemPtr : graphgetmemprc;

  GetScanLine    : GetScanLineProc;
  Line           : LineProc;
  InternalEllipse: EllipseProc;
  PatternLine    : PatternLineProc;
  HLine          : HLineProc;
  VLine          : VLineProc;
  Circle         : CircleProc;

  SaveVideoState : SaveStateProc;
  RestoreVideoState: RestoreStateProc;


type
  TCharsetTransTable = array[Char] of Char;
  PCharsetTransTable = ^TCharsetTransTable;

const

  { The following table can be used for translating characters from the
    Ansi charset (ISO8859-1) to the DOS ASCII charset (CP437).
    To use this table, add the following line of code to your program:
    GraphStringTransTable := @AnsiToASCIITransTable;
  }

  AnsiToASCIITransTable: TCharsetTransTable =
    (#$00, #$01, #$02, #$03, #$04, #$05, #$06, #$07,   { $00 - $07 }
     #$08, #$09, #$0a, #$0b, #$0c, #$0d, #$0e, #$0f,   { $08 - $0f }
     #$10, #$11, #$12, #$13, #$14, #$15, #$16, #$17,   { $10 - $17 }
     #$18, #$19, #$1a, #$1b, #$1c, #$1d, #$1e, #$1f,   { $18 - $1f }
     #$20, #$21, #$22, #$23, #$24, #$25, #$26, #$27,   { $20 - $27 }
     #$28, #$29, #$2a, #$2b, #$2c, #$2d, #$2e, #$2f,   { $28 - $2f }
     #$30, #$31, #$32, #$33, #$34, #$35, #$36, #$37,   { $30 - $37 }
     #$38, #$39, #$3a, #$3b, #$3c, #$3d, #$3e, #$3f,   { $38 - $3f }
     #$40, #$41, #$42, #$43, #$44, #$45, #$46, #$47,   { $40 - $47 }
     #$48, #$49, #$4a, #$4b, #$4c, #$4d, #$4e, #$4f,   { $48 - $4f }
     #$50, #$51, #$52, #$53, #$54, #$55, #$56, #$57,   { $50 - $57 }
     #$58, #$59, #$5a, #$5b, #$5c, #$5d, #$5e, #$5f,   { $58 - $5f }
     #$60, #$61, #$62, #$63, #$64, #$65, #$66, #$67,   { $60 - $67 }
     #$68, #$69, #$6a, #$6b, #$6c, #$6d, #$6e, #$6f,   { $68 - $6f }
     #$70, #$71, #$72, #$73, #$74, #$75, #$76, #$77,   { $70 - $77 }
     #$78, #$79, #$7a, #$7b, #$7c, #$7d, #$7e, #$7f,   { $78 - $7f }
     '?' , '?' , '?' , '?' , '?' , '?' , '?' , '?' ,   { $80 - $87 }
     '?' , '?' , '?' , '?' , '?' , '?' , '?' , '?' ,   { $88 - $8f }
     '?' , '?' , '?' , '?' , '?' , '?' , '?' , '?' ,   { $90 - $97 }
     '?' , '?' , '?' , '?' , '?' , '?' , '?' , '?' ,   { $98 - $9f }
     #$ff, #$ad, #$9b, #$9c, '?' , #$9d, '?' , '?' ,   { $a0 - $a7 }
     '?' , '?' , #$a6, #$ae, #$aa, '?' , '?' , '?' ,   { $a8 - $af }
     #$f8, #$f1, #$fd, '?' , '?' , #$e6, '?' , #$fa,   { $b0 - $b7 }
     '?' , '?' , #$a7, #$af, #$ac, #$ab, '?' , #$a8,   { $b8 - $bf }
     '?' , '?' , '?' , '?' , #$8e, #$8f, #$92, #$80,   { $c0 - $c7 }
     '?' , #$90, '?' , '?' , '?' , '?' , '?' , '?' ,   { $c8 - $cf }
     '?' , #$a5, '?' , '?' , '?' , '?' , #$99, '?' ,   { $d0 - $d7 }
     '?' , '?' , '?' , '?' , #$9a, '?' , '?' , #$e1,   { $d8 - $df }
     #$85, #$a0, #$83, '?' , #$84, #$86, #$91, #$87,   { $e0 - $e7 }
     #$8a, #$82, #$88, #$89, #$8d, #$a1, #$8c, #$8b,   { $e8 - $ef }
     '?' , #$a4, #$95, #$a2, #$93, '?' , #$94, #$f6,   { $f0 - $f7 }
     '?' , #$97, #$a3, #$96, #$81, '?' , '?' , #$98);  { $f8 - $ff }


  GraphStringTransTable: PCharsetTransTable = nil;
  DrawTextBackground : boolean = false;

function queryadapterinfo : pmodeinfo;

Procedure Closegraph;
procedure SetLineStyle(LineStyle: word; Pattern: word; Thickness: word);
function  GraphErrorMsg(ErrorCode: smallint): string;
Function  GetMaxX: smallint;
Function  GetMaxY: smallint;
Procedure SetViewPort(X1, Y1, X2, Y2: smallint; Clip: Boolean);
Function  GraphResult: smallint;
function  GetModeName(ModeNumber: smallint): string;
procedure SetGraphMode(Mode: smallint);
function GetGraphMode: smallint;
function GetMaxMode: smallint;
procedure RestoreCrtMode;
procedure GetModeRange(GraphDriver: smallint; var LoMode, HiMode: smallint);
Function  GetX: smallint;
Function  GetY: smallint;
procedure GraphDefaults;
procedure ClearDevice;
procedure GetViewSettings(var viewport : ViewPortType);
procedure SetWriteMode(WriteMode : smallint);
procedure GetFillSettings(var Fillinfo:Fillsettingstype);
procedure GetFillPattern(var FillPattern:FillPatternType);
procedure GetLineSettings(var ActiveLineInfo : LineSettingsType);
procedure InitGraph(var GraphDriver:smallint;var GraphMode:smallint;const PathToDriver:String);
procedure DetectGraph(var GraphDriver:smallint;var GraphMode:smallint);
function InstallUserDriver(Name: string; AutoDetectPtr: Pointer): smallint;
function RegisterBGIDriver(driver: pointer): smallint;
procedure SetFillStyle(Pattern : word; Color: word);
procedure SetFillPattern(Pattern: FillPatternType; Color: word);
Function GetDriverName: string;
 procedure MoveRel(Dx, Dy: smallint);
 procedure MoveTo(X,Y: smallint);

 procedure SetDirectVideo(DirectAccess: boolean);
 function GetDirectVideo: boolean;

 { -------------------- Color/Palette ------------------------------- }
 procedure SetBkColor(ColorNum: Word);
 function  GetColor: Word;
 function  GetBkColor: Word;
 procedure SetColor(Color: Word);
 function  GetMaxColor: word;

 procedure SetPalette(ColorNum: word; Color: shortint);
 procedure GetPalette(var Palette: PaletteType);
 function GetPaletteSize: smallint;
 procedure GetDefaultPalette(var Palette: PaletteType);


 { -------------------- Shapes/Lines -------------------------------- }
 procedure Rectangle(x1,y1,x2,y2:smallint);
 procedure Bar(x1,y1,x2,y2:smallint);
 procedure Bar3D(x1, y1, x2, y2 : smallint;depth : word;top : boolean);
 procedure FillPoly(NumPoints: word; Var PolyPoints);
 procedure DrawPoly(NumPoints : word;var polypoints);
 procedure LineRel(Dx, Dy: smallint);
 procedure LineTo(X,Y : smallint);
 procedure FloodFill(x : smallint; y : smallint; Border: word);

 { -------------------- Circle related routines --------------------- }
 procedure GetAspectRatio(var Xasp,Yasp : word);
 procedure SetAspectRatio(Xasp, Yasp : word);
 procedure GetArcCoords(var ArcCoords: ArcCoordsType);


 procedure Arc(X,Y : smallint; StAngle,EndAngle,Radius: word);
 procedure PieSlice(X,Y,stangle,endAngle:smallint;Radius: Word);
 procedure FillEllipse(X, Y: smallint; XRadius, YRadius: Word);
 procedure Sector(x, y: smallint; StAngle,EndAngle, XRadius, YRadius: Word);
 procedure Ellipse(X,Y : smallint; stAngle, EndAngle: word; XRadius,
   YRadius: word);

 { --------------------- Text related routines --------------------- }
 function  InstallUserFont(const FontFileName : string) : smallint;
 function  RegisterBGIfont(font : pointer) : smallint;
 procedure GetTextSettings(var TextInfo : TextSettingsType);
 function  TextHeight(const TextString : string) : word;
 function  TextWidth(const TextString : string) : word;
 procedure SetTextJustify(horiz,vert : word);
 procedure SetTextStyle(font,direction : word;charsize : word);
 procedure SetUserCharSize(Multx,Divx,Multy,Divy : word);

 procedure OutText(const TextString : string);


