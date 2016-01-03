{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
    History:

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening
    of the library.
    13 Jan 2003.

    Changed cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT colorwheel;

INTERFACE
USES exec, utility;

Type
{ For use with the WHEEL_HSB tag }
 pColorWheelHSB = ^tColorWheelHSB;
 tColorWheelHSB = record
    cw_Hue,
    cw_Saturation,
    cw_Brightness  : ULONG;
 end;

{ For use with the WHEEL_RGB tag }
 pColorWheelRGB = ^tColorWheelRGB;
 tColorWheelRGB = record
    cw_Red,
    cw_Green,
    cw_Blue  : ULONG;
 end;


{***************************************************************************}

const
    WHEEL_Dummy          = (TAG_USER+$04000000);
    WHEEL_Hue            = (WHEEL_Dummy+1) ;  { set/get Hue              }
    WHEEL_Saturation     = (WHEEL_Dummy+2) ;  { set/get Saturation        }
    WHEEL_Brightness     = (WHEEL_Dummy+3) ;  { set/get Brightness        }
    WHEEL_HSB            = (WHEEL_Dummy+4) ;  { set/get ColorWheelHSB     }
    WHEEL_Red            = (WHEEL_Dummy+5) ;  { set/get Red               }
    WHEEL_Green          = (WHEEL_Dummy+6) ;  { set/get Green     }
    WHEEL_Blue           = (WHEEL_Dummy+7) ;  { set/get Blue              }
    WHEEL_RGB            = (WHEEL_Dummy+8) ;  { set/get ColorWheelRGB     }
    WHEEL_Screen         = (WHEEL_Dummy+9) ;  { init screen/enviroment    }
    WHEEL_Abbrv          = (WHEEL_Dummy+10);  { "GCBMRY" if English       }
    WHEEL_Donation       = (WHEEL_Dummy+11);  { colors donated by app     }
    WHEEL_BevelBox       = (WHEEL_Dummy+12);  { inside a bevel box        }
    WHEEL_GradientSlider = (WHEEL_Dummy+13);  { attached gradient slider  }
    WHEEL_MaxPens        = (WHEEL_Dummy+14);  { max # of pens to allocate }


{***************************************************************************}

{--- functions in V39 or higher (Release 3) ---}

VAR ColorWheelBase : pLibrary;

const
    COLORWHEELNAME : Pchar = 'colorwheel.library';

PROCEDURE ConvertHSBToRGB(hsb : pColorWheelHSB; rgb : pColorWheelRGB);
PROCEDURE ConvertRGBToHSB(rgb : pColorWheelRGB; hsb : pColorWheelHSB);

IMPLEMENTATION

uses msgbox;

PROCEDURE ConvertHSBToRGB(hsb : pColorWheelHSB; rgb : pColorWheelRGB);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L hsb,A0
    MOVEA.L rgb,A1
    MOVEA.L ColorWheelBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ConvertRGBToHSB(rgb : pColorWheelRGB; hsb : pColorWheelHSB);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rgb,A0
    MOVEA.L hsb,A1
    MOVEA.L ColorWheelBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

{$I useautoopenlib.inc}
{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of colorwheel.library}

var
    colorwheel_exit : Pointer;

procedure ClosecolorwheelLibrary;
begin
    ExitProc := colorwheel_exit;
    if ColorWheelBase <> nil then begin
        CloseLibrary(ColorWheelBase);
        ColorWheelBase := nil;
    end;
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

begin
    ColorWheelBase := nil;
    ColorWheelBase := OpenLibrary(COLORWHEELNAME,LIBVERSION);
    if ColorWheelBase <> nil then begin
        colorwheel_exit := ExitProc;
        ExitProc := @ClosecolorwheelLibrary
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open colorwheel.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$else}
   {$Warning No autoopening of colorwheel.library compiled}
   {$Info Make sure you open colorwheel.library yourself}
{$endif use_auto_openlib}

END. (* UNIT COLORWHEEL *)



