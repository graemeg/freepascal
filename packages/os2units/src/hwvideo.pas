{
    Copyright (c) 1991, 1992, 1993 International Business Machines Corporation
    Copyright (c) 2002 by Valery Gaynullin
    Copyright (c) 2002-2003 by Yuri Prokushev (prokushev@freemail.ru)

    This is Video Acceleration Interface

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(Video Acceleration Interface)
@author(Valery Gaynullin)
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(29 Nov 2002)
@lastmod(19 Jan 2003)
This is Video Acceleration Interface.
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}

Unit HWVideo;

Interface

Uses
  Os2Def,
  PMWin,
  PMGpi;

// GRADD function class
Const
  VA2_FUNCTION_CLASS='Video Acceleration 2';

Type
  THWVIDEOCAPS=record
    ulLength: Cardinal;
    ulCapsFlags: Cardinal;     //flags, describing HW capability
    szlSrcMax: SIZEL;          //maximum source size (pixels)
    rctlDstMargin: RECTL;      //destination rectangle margins
    fccDstColor: Cardinal;     //screen FOURCC
    ulScanAlign: Cardinal;     //requered scanline aligment-1
    ulNumColors: Cardinal;     //count of supported source FOURCC
    fccColorType: ^Cardinal;   //array of supported FOURCC
    ulAttrCount: Cardinal;     //count of viewport attributes
  end;
  PHWVIDEOCAPS=^THWVIDEOCAPS;

//ulCapsFlag defines
Const
  HWVIDEOCAPS_MINIFY             =  $00000001; //Chip can perform downscaling
  HWVIDEOCAPS_FILTER             =  $00000002; //Image filtering present
  HWVIDEOCAPS_NONINTEGER_SCALE   =  $00000004; //allow scale to noninteger ratio
  HWVIDEOCAPS_COLOR_KEYING       =  $00000008; //allow color keying
  HWVIDEOCAPS_OVERLAY            =  $00000010; //overlay-type HW
  HWVIDEOCAPS_SEPARATE_OUT       =  $00000020; //used separate output connector (like TV out)
  HWVIDEOCAPS_DECODE             =  $00000040; //support non-RAW data
  HWVIDEOCAPS_NEEDPROTECT        =  $00000080; //HW need to lock VRAM

Type
  THWATTRIBUTE=record
    ulLength: Cardinal;                //size of structure in bytes
    szAttrDesc: Array[0..64] of Char;  //string, describing attribute
    ulAttrType: Cardinal;              //type of attribute, check ATTRTYPE_* const
    ulValueSize: Cardinal;             //size in bytes of each value member
    ulValueCount: Cardinal;            //count of value members
    pValueList: Pointer;               //list of supported values
    ulCurrentValue: Cardinal;          //current value
    ulDefaultValue: Cardinal;          //default value
    ulAttribFlags: Cardinal;           //flags to define some additional properties
  end;
  PHWATTRIBUTE=^THWATTRIBUTE;

//types of attributes.
// if ATTRTYPE_BOOLEAN, ATTRTYPE_STATICTEXT or ATTRTYPE_BYTE,
// ulValueCount & ulValueSize undefined,
// pValueList can be NULL,
// else this must be actual value of allocated chunk of memory.

Const
  //attribute of ON/OFF type
  ATTRTYPE_BOOLEAN              = 1;
  //attribute can be member of set string type
  ATTRTYPE_AGGREGATE_STRING     = 2;
  //attribute have no value - this is static text string
  ATTRTYPE_STATICTEXT           = 3;
  //attribute can be any value in 0..255 margins
  ATTRTYPE_BYTE                 = 4;


//defines for ulAttribFlags field
//changing this attribute affect HW capability, so application
//must re-read Caps after changing value of this attribute
//application must not assume preserving any capability when this
//attribute was changed
  ATTRIB_CHANGE_CAPS = 1;

//some common attribute names
Const
  ATTRIBUTE_BRIGHTNESS = 'Brightness';
  ATTRIBUTE_CONTRAST   = 'Contrast';
  ATTRIBUTE_SATURATION = 'Saturation';
  ATTRIBUTE_HUE        = 'Hue';
  ATTRIBUTE_FILTERING  = 'Filtering';
  ATTRIBUTE_TVOUT      = 'Output to TV';
  ATTRIBUTE_COLORKEY   = 'Color Keying';

Type
  THWVIDEOSETUP=record
    ulLength: Cardinal;
    rctlDstRect: RECTL;      //destination screen rectangle
    szlSrcSize: SIZEL;       //source image size
    ulSrcPitch: Cardinal;    //byte offset between image rows
    fccColor: Cardinal;      //image format
    ulKeyColor: Cardinal;    //color key
    rctlSrcRect: RECTL;
  end;
  PHWVIDEOSETUP=^THWVIDEOSETUP;

//return codes
Const
  //no error
  HWVIDEO_DONE = 0;
  //unspecified error
  HWVIDEO_ERROR = 3;
  //FS session active, accelerator not available
  HWVIDEO_ERROR_BACKGROUND = 4;
  //HW not available
  HWVIDEO_ERROR_NO_HW = 6;
  //incorrect parameter
  HWVIDEO_ERROR_PARAMETER = 7;
  //to low offscreen VRAM to handle setup
  HWVIDEO_ERROR_LOW_VRAM = 8;
  //HW already in use
  HWVIDEO_ERROR_USED = 9;

//Init HWVideo subsystem
//check for presence and avilability of accelerated HW, if present and
//available - lock it for this process.
Function HWVIDEOInit: Cardinal; cdecl;

//Get HW capability
//return filled structure. When called, ulNumColors must be
//set to actual allocated size of fccColorType array.
//if call returned with error, then need to check returned
//ulNumColors, allocate larger space and call again.
Function HWVIDEOCaps(pCaps: PHWVIDEOCAPS): Cardinal; cdecl;

//Set HWVideo viewport
//Check HW capability to handle this setup, allocate buffers.
//one special case: pSetup==NULL - disable video and free all
//buffers
Function HWVIDEOSetup(pSetup: PHWVIDEOSETUP): Cardinal; cdecl;

//Get HWVideo buffer pointer
//return linear pointer to overlay buffer and it's physical address
Function HWVIDEOBeginUpdate(var ppBuffer: Pointer; var pulPhysBuffer: Cardinal): Cardinal; cdecl;

//Display HWVideo buffer
//set pending state for last accessed videobuffer, switch
//buffers on next VSYNC
Function HWVIDEOEndUpdate: Cardinal; cdecl;

//Get current HW attributes
//0<=ulAttribNum<pCaps->ulAttrCount, else error returned.
Function HWVIDEOGetAttrib(ulAttribNum: Cardinal; pAttrib: PHWATTRIBUTE): Cardinal; cdecl;

//Set new attribute value
//0<=ulAttribNum<pCaps.ulAttrCount, else error returned.
//pAttrib->ulCurrentValue filled with new value
Function HWVIDEOSetAttrib(ulAttribNum: Cardinal; pAttrib: PHWATTRIBUTE): Cardinal; cdecl;

//Close HWVideo
Function HWVIDEOClose: Cardinal; cdecl;

Implementation

Function HWVIDEOInit: Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOInit';

Function HWVIDEOCaps(pCaps: PHWVIDEOCAPS): Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOCaps';

Function HWVIDEOSetup(pSetup: PHWVIDEOSETUP): Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOSetup';

Function HWVIDEOBeginUpdate(var ppBuffer: Pointer; var pulPhysBuffer: Cardinal): Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOBeginUpdate';

Function HWVIDEOEndUpdate: Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOEndUpdate';

Function HWVIDEOGetAttrib(ulAttribNum: Cardinal; pAttrib: PHWATTRIBUTE): Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOGetAttrib';

Function HWVIDEOSetAttrib(ulAttribNum: Cardinal; pAttrib: PHWATTRIBUTE): Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOSetAttrib';

Function HWVIDEOClose: Cardinal; cdecl;
  external 'hwvideo' name 'HWVIDEOClose';

End.
