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

    Added functions and procedures with array of const.
    For use with fpc 1.0.7. They are in systemvartags.
    11 Nov 2001.

    Update for AmigaOs 3.9.
    Added some new const.
    tadtNewFormatFrame is a new record.
    New procedures and functions.
             FUNCTION ObtainDTDrawInfoA
             FUNCTION DrawDTObjectA
             PROCEDURE ReleaseDTDrawInfo

    New varargs procedures and function, they are
    in systemvartags.
         PROCEDURE RefreshDTObjects
         FUNCTION DoDTMethod
         FUNCTION PrintDTObject
         FUNCTION ObtainDTDrawInfo
         FUNCTION DrawDTObject
    Changed startcode for library.
    28 Jan 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

unit datatypes;

INTERFACE

uses exec, amigados, intuition, utility,
     graphics, iffparse, amigaprinter, prtbase;

const
{***************************************************************************}

 ID_DTYP = 1146378576;

{***************************************************************************}

 ID_DTHD = 1146374212;

Type
 pDataTypeHeader = ^tDataTypeHeader;
 tDataTypeHeader = record
    dth_Name,                                         { Descriptive name of the data type }
    dth_BaseName,                                     { Base name of the data type }
    dth_Pattern  : STRPTR;                            { Match pattern for file name. }
    dth_Mask : Pointer;                               { Comparision mask }
    dth_GroupID,                                      { Group that the DataType is in }
    dth_ID   : ULONG;                                 { ID for DataType (same as IFF FORM type) }
    dth_MaskLen,                                      { Length of comparision mask }
    dth_Pad   : smallint;                              { Unused at present (must be 0) }
    dth_Flags,                                        { Flags }
    dth_Priority  : WORD;                             { Priority }
 end;

const
 DTHSIZE = 32;

{***************************************************************************}

{ Basic type }
 DTF_TYPE_MASK  = $000F;
 DTF_BINARY     = $0000;
 DTF_ASCII      = $0001;
 DTF_IFF        = $0002;
 DTF_MISC       = $0003;

{ Set if case is important }
 DTF_CASE       = $0010;

{ Reserved for system use }
 DTF_SYSTEM1    = $1000;

{****************************************************************************
 *
 * GROUP ID and ID
 *
 * This is used for filtering out objects that you don't want.  For
 * example, you could make a filter for the ASL file requester so
 * that it only showed the files that were pictures, or even to
 * narrow it down to only show files that were ILBM pictures.
 *
 * Note that the Group ID's are in lower case, and always the first
 * four characters of the word.
 *
 * For ID's; If it is an IFF file, then the ID is the same as the
 * FORM type.  If it isn't an IFF file, then the ID would be the
 * first four characters of name for the file type.
 *
 ****************************************************************************}

{ System file, such as; directory, executable, library, device, font, etc. }
 GID_SYSTEM      = 1937339252;

{ Formatted or unformatted text }
 GID_TEXT        = 1952807028;

{ Formatted text with graphics or other DataTypes }
 GID_DOCUMENT    = 1685021557;

{ Sound }
 GID_SOUND       = 1936684398;

{ Musical instruments used for musical scores }
 GID_INSTRUMENT  = 1768846196;

{ Musical score }
 GID_MUSIC       = 1836413801;

{ Still picture }
 GID_PICTURE     = 1885954932;

{ Animated picture }
 GID_ANIMATION   = 1634625901;

{ Animation with audio track }
 GID_MOVIE       = 1836021353;

{***************************************************************************}

{ A code chunk contains an embedded executable that can be loaded
 * with InternalLoadSeg. }
 ID_CODE = 1146372932;

Type
{ DataTypes comparision hook context (Read-Only).  This is the
 * argument that is passed to a custom comparision routine. }

 pDTHookContext = ^tDTHookContext;
 tDTHookContext = record
    { Libraries that are already opened for your use }
    dthc_SysBase,
    dthc_DOSBase,
    dthc_IFFParseBase,
    dthc_UtilityBase             : pLibrary;

    { File context }
    dthc_Lock                    : BPTR;                { Lock on the file }
    dthc_FIB                     : pFileInfoBlock;      { Pointer to a FileInfoBlock }
    dthc_FileHandle              : BPTR;                { Pointer to the file handle (may be NULL) }
    dthc_IFF                     : pIFFHandle;          { Pointer to an IFFHandle (may be NULL) }
    dthc_Buffer                  : STRPTR;              { Buffer }
    dthc_BufferLength            : ULONG;               { Length of the buffer }
 end;

{***************************************************************************}

const
 ID_TOOL = 1146377292;

Type
 pTool = ^tTool;
 tTool = record
    tn_Which,                                      { Which tool is this }
    tn_Flags  : WORD;                              { Flags }
    tn_Program : STRPTR;                           { Application to use }
 end;

const
 TSIZE = 8;

{ defines for tn_Which }
 TW_INFO               =  1;
 TW_BROWSE             =  2;
 TW_EDIT               =  3;
 TW_PRINT              =  4;
 TW_MAIL               =  5;

{ defines for tn_Flags }
 TF_LAUNCH_MASK        =  $000F;
 TF_SHELL              =  $0001;
 TF_WORKBENCH          =  $0002;
 TF_RX                 =  $0003;

{***************************************************************************}

 ID_TAGS = 1146377287;

{***************************************************************************}

Type
 pDataType = ^tDataType;
 tDataType = record
    dtn_Node1,                      { Reserved for system use }
    dtn_Node2   : tNode;            { Reserved for system use }
    dtn_Header  : pDataTypeHeader;  { Pointer to the DataTypeHeader }
    dtn_ToolList: tList;            { List of tool nodes }
    dtn_FunctionName : STRPTR;      { Name of comparision routine }
    dtn_AttrList : pTagItem;         { Object creation tags }
    dtn_Length : ULONG;             { Length of the memory block }
 end;

{***************************************************************************}

 pToolNode = ^tToolNode;
 tToolNode = Record
    tn_Node   : tNode;                               { Embedded node }
    tn_Tool   : tTool;                               { Embedded tool }
    tn_Length : ULONG;                            { Length of the memory block }
 end;

{***************************************************************************}

const
 ID_NAME = 1312902469;

{***************************************************************************}

{ text ID's }
 DTERROR_UNKNOWN_DATATYPE              =  2000;
 DTERROR_COULDNT_SAVE                  =  2001;
 DTERROR_COULDNT_OPEN                  =  2002;
 DTERROR_COULDNT_SEND_MESSAGE          =  2003;

{ new for V40 }
 DTERROR_COULDNT_OPEN_CLIPBOARD        =  2004;
 DTERROR_Reserved                      =  2005;
 DTERROR_UNKNOWN_COMPRESSION           =  2006;
 DTERROR_NOT_ENOUGH_DATA               =  2007;
 DTERROR_INVALID_DATA                  =  2008;

{ New for V44 }
 DTERROR_NOT_AVAILABLE                 =  2009;

{ Offset for types }
 DTMSG_TYPE_OFFSET                     =  2100;

{***************************************************************************}

  DATATYPESCLASS        : Pchar =  'datatypesclass';

{***************************************************************************}

  DTA_Dummy             =  (TAG_USER+$1000);

{ Generic attributes }
  DTA_TextAttr          =  (DTA_Dummy+10);
        { (struct TextAttr ) Pointer to the default TextAttr to use for
         * the text within the object. }

  DTA_TopVert           =  (DTA_Dummy+11);
        { (LONG) Current top vertical unit }

  DTA_VisibleVert       =  (DTA_Dummy+12);
        { (LONG) Number of visible vertical units }

  DTA_TotalVert         =  (DTA_Dummy+13);
        { (LONG) Total number of vertical units }

  DTA_VertUnit          =  (DTA_Dummy+14);
        { (LONG) Number of pixels per vertical unit }

  DTA_TopHoriz          =  (DTA_Dummy+15);
        { (LONG) Current top horizontal unit }

  DTA_VisibleHoriz      =  (DTA_Dummy+16);
        { (LONG)  Number of visible horizontal units }

  DTA_TotalHoriz        =  (DTA_Dummy+17);
        { (LONG) Total number of horizontal units }

  DTA_HorizUnit         =  (DTA_Dummy+18);
        { (LONG) Number of pixels per horizontal unit }

  DTA_NodeName          =  (DTA_Dummy+19);
        { (UBYTE ) Name of the current element within the object. }

  DTA_Title             =  (DTA_Dummy+20);
        { (UBYTE ) Title of the object. }

  DTA_TriggerMethods    =  (DTA_Dummy+21);
        { (struct DTMethod ) Pointer to a NULL terminated array of
         * supported trigger methods. }

  DTA_Data              =  (DTA_Dummy+22);
        { (APTR) Object specific data. }

  DTA_TextFont          =  (DTA_Dummy+23);
        { (struct TextFont ) Default font to use for text within the
         * object. }

  DTA_Methods           =  (DTA_Dummy+24);
        { (ULONG ) Pointer to a ~0 terminated array of supported
         * methods. }

  DTA_PrinterStatus     =  (DTA_Dummy+25);
        { (LONG) Printer error message.  Error numbers are defined in
         * <devices/printer.h> }

  DTA_PrinterProc       =  (DTA_Dummy+26);
        { PRIVATE (struct Process ) Pointer to the print process. }

  DTA_LayoutProc        =  (DTA_Dummy+27);
        { PRIVATE (struct Process ) Pointer to the layout process. }

  DTA_Busy              =  (DTA_Dummy+28);
        { Used to turn the applications' busy pointer off and on }

  DTA_Sync              =  (DTA_Dummy+29);
        { Used to indicate that new information has been loaded into
         * an object.  This is for models that cache the DTA_TopVert-
         * like tags }

  DTA_BaseName          =  (DTA_Dummy+30);
        { The base name of the class }

  DTA_GroupID           =  (DTA_Dummy+31);
        { Group that the object must belong in }

  DTA_ErrorLevel        =  (DTA_Dummy+32);
        { Error level }

  DTA_ErrorNumber       =  (DTA_Dummy+33);
        { datatypes.library error number }

  DTA_ErrorString       =  (DTA_Dummy+34);
        { Argument for datatypes.library error }

  DTA_Conductor         =  (DTA_Dummy+35);
        { New for V40. (UBYTE ) specifies the name of the
         * realtime.library conductor.  Defaults to "Main". }

  DTA_ControlPanel      =  (DTA_Dummy+36);
        { New for V40. (BOOL) Indicate whether a control panel should be
         * embedded within the object (in the animation datatype, for
         * example).  Defaults to TRUE. }

  DTA_Immediate         =  (DTA_Dummy+37);
        { New for V40. (BOOL) Indicate whether the object should
         * immediately begin playing.  Defaults to FALSE. }

  DTA_Repeat            =  (DTA_Dummy+38);
        { New for V40. (BOOL) Indicate that the object should repeat
         * playing.  Defaults to FALSE. }

        { New for V44. Address of a DTST_MEMORY source type
         * object (APTR).
         }
  DTA_SourceAddress     =  (DTA_Dummy+39);

        { New for V44. Size of a DTST_MEMORY source type
         * object (ULONG).
        }
  DTA_SourceSize        =  (DTA_Dummy+40);

        { Reserved tag; DO NOT USE (V44) }
  DTA_Reserved          =  (DTA_Dummy+41);

{ DTObject attributes }
  DTA_Name              =  (DTA_Dummy+100);
  DTA_SourceType        =  (DTA_Dummy+101);
  DTA_Handle            =  (DTA_Dummy+102);
  DTA_DataType          =  (DTA_Dummy+103);
  DTA_Domain            =  (DTA_Dummy+104);

{ DON'T USE THE FOLLOWING FOUR TAGS.  USE THE CORRESPONDING TAGS IN
 * <intuition/gadgetclass.h> }
  DTA_Left              =  (DTA_Dummy+105);
  DTA_Top               =  (DTA_Dummy+106);
  DTA_Width             =  (DTA_Dummy+107);
  DTA_Height            =  (DTA_Dummy+108);

  DTA_ObjName           =  (DTA_Dummy+109);
  DTA_ObjAuthor         =  (DTA_Dummy+110);
  DTA_ObjAnnotation     =  (DTA_Dummy+111);
  DTA_ObjCopyright      =  (DTA_Dummy+112);
  DTA_ObjVersion        =  (DTA_Dummy+113);
  DTA_ObjectID          =  (DTA_Dummy+114);
  DTA_UserData          =  (DTA_Dummy+115);
  DTA_FrameInfo         =  (DTA_Dummy+116);

{ DON'T USE THE FOLLOWING FOUR TAGS.  USE THE CORRESPONDING TAGS IN
 * <intuition/gadgetclass.h> }
  DTA_RelRight          =  (DTA_Dummy+117);
  DTA_RelBottom         =  (DTA_Dummy+118);
  DTA_RelWidth          =  (DTA_Dummy+119);
  DTA_RelHeight         =  (DTA_Dummy+120);

  DTA_SelectDomain      =  (DTA_Dummy+121);
  DTA_TotalPVert        =  (DTA_Dummy+122);
  DTA_TotalPHoriz       =  (DTA_Dummy+123);
  DTA_NominalVert       =  (DTA_Dummy+124);
  DTA_NominalHoriz      =  (DTA_Dummy+125);

{ Printing attributes }
  DTA_DestCols          =  (DTA_Dummy+400);
        { (LONG) Destination X width }

  DTA_DestRows          =  (DTA_Dummy+401);
        { (LONG) Destination Y height }

  DTA_Special           =  (DTA_Dummy+402);
        { (UWORD) Option flags }

  DTA_RastPort          =  (DTA_Dummy+403);
        { (struct RastPort ) RastPort to use when printing. (V40) }

  DTA_ARexxPortName     =  (DTA_Dummy+404);
        { (STRPTR) Pointer to base name for ARexx port (V40) }


{***************************************************************************}

  DTST_RAM              =  1;
  DTST_FILE             =  2;
  DTST_CLIPBOARD        =  3;
  DTST_HOTLINK          =  4;
  DTST_MEMORY           =  5;   { New for V44 }

{***************************************************************************}

{ Attached to the Gadget.SpecialInfo field of the gadget.  Don't access directly,
 * use the Get/Set calls instead.
 }
Type

 pDTSpecialInfo = ^tDTSpecialInfo;
 tDTSpecialInfo = record
    si_Lock            : tSignalSemaphore;       { Locked while in DoAsyncLayout() }
    si_Flags,

    si_TopVert,    { Top row (in units) }
    si_VisVert,    { Number of visible rows (in units) }
    si_TotVert,    { Total number of rows (in units) }
    si_OTopVert,   { Previous top (in units) }
    si_VertUnit,   { Number of pixels in vertical unit }

    si_TopHoriz,   { Top column (in units) }
    si_VisHoriz,   { Number of visible columns (in units) }
    si_TotHoriz,   { Total number of columns (in units) }
    si_OTopHoriz,  { Previous top (in units) }
    si_HorizUnit  : Longint;  { Number of pixels in horizontal unit }
 end;


const
{ Object is in layout processing }
  DTSIF_LAYOUT         =   1;

{ Object needs to be layed out }
  DTSIF_NEWSIZE        =   2;

  DTSIF_DRAGGING       =   4;
  DTSIF_DRAGSELECT     =   8;

  DTSIF_HIGHLIGHT      =   16;

{ Object is being printed }
  DTSIF_PRINTING       =   32;

{ Object is in layout process }
  DTSIF_LAYOUTPROC     =   64;

{***************************************************************************}

Type
 pDTMethod = ^tDTMethod;
 tDTMethod = record
    dtm_Label,
    dtm_Command  : STRPTR;
    dtm_Method   : ULONG;
 end;

{***************************************************************************}

Const
  DTM_Dummy             =  ($600);

{ Inquire what environment an object requires }
  DTM_FRAMEBOX          =  ($601);

{ Same as GM_LAYOUT except guaranteed to be on a process already }
  DTM_PROCLAYOUT        =  ($602);

{ Layout that is occurring on a process }
  DTM_ASYNCLAYOUT       =  ($603);

{ When a RemoveDTObject() is called }
  DTM_REMOVEDTOBJECT    =  ($604);

  DTM_SELECT            =  ($605);
  DTM_CLEARSELECTED     =  ($606);

  DTM_COPY              =  ($607);
  DTM_PRINT             =  ($608);
  DTM_ABORTPRINT        =  ($609);

  DTM_NEWMEMBER         =  ($610);
  DTM_DISPOSEMEMBER     =  ($611);

  DTM_GOTO              =  ($630);
  DTM_TRIGGER           =  ($631);

  DTM_OBTAINDRAWINFO    =  ($640);
  DTM_DRAW              =  ($641);
  DTM_RELEASEDRAWINFO   =  ($642);

  DTM_WRITE             =  ($650);

{ Used to ask the object about itself }
type
 pFrameInfo = ^tFrameInfo;
 tFrameInfo = record
            fri_PropertyFlags : ULONG;
            fri_Resolution : tPoint;
            fri_RedBits :  BYTE;
            fri_GreenBits : BYTE;
            fri_BlueBits : BYTE;
            fri_Dimensions : record
                 Width : ULONG;
                 Height : ULONG;
                 Depth : ULONG;
              end;
            fri_Screen : pScreen;
            fri_ColorMap : pColorMap;
            fri_Flags : ULONG;
         end;


CONST

  FIF_SCALABLE    = $1;
  FIF_SCROLLABLE  = $2;
  FIF_REMAPPABLE  = $4;

{ DTM_REMOVEDTOBJECT, DTM_CLEARSELECTED, DTM_COPY, DTM_ABORTPRINT }
Type

 pdtGeneral = ^tdtGeneral;
 tdtGeneral = record
    MethodID   : ULONG;
    dtg_GInfo  : pGadgetInfo;
 end;

{ DTM_SELECT }
 pdtSelect = ^tdtSelect;
 tdtSelect = record
    MethodID  : ULONG;
    dts_GInfo : pGadgetInfo;
    dts_Select : tRectangle;
 end;

{ DTM_FRAMEBOX }

 pdtFrameBox = ^tdtFrameBox;
 tdtFrameBox = record
    MethodID             : ULONG;
    dtf_GInfo            : pGadgetInfo;
    dtf_ContentsInfo,     { Input }
    dtf_FrameInfo        : pFrameInfo;         { Output }
    dtf_SizeFrameInfo,
    dtf_FrameFlags       : ULONG;
 end;

{ DTM_GOTO }
 pdtGoto = ^tdtGoto;
 tdtGoto = record
    MethodID             : ULONG;
    dtg_GInfo            : pGadgetInfo;
    dtg_NodeName         : STRPTR;          { Node to goto }
    dtg_AttrList         : pTagItem;         { Additional attributes }
 end;

{ DTM_TRIGGER }

 pdtTrigger = ^tdtTrigger;
 tdtTrigger = record
    MethodID             : ULONG;
    dtt_GInfo            : pGadgetInfo;
    dtt_Function         : ULONG;
    dtt_Data             : Pointer;
 end;

const
  STM_PAUSE             =  1 ;
  STM_PLAY              =  2 ;
  STM_CONTENTS          =  3 ;
  STM_INDEX             =  4 ;
  STM_RETRACE           =  5 ;
  STM_BROWSE_PREV       =  6 ;
  STM_BROWSE_NEXT       =  7 ;

  STM_NEXT_FIELD        =  8 ;
  STM_PREV_FIELD        =  9 ;
  STM_ACTIVATE_FIELD    =  10;

  STM_COMMAND           =  11;

{ New for V40 }
  STM_REWIND            =  12;
  STM_FASTFORWARD       =  13;
  STM_STOP              =  14;
  STM_RESUME            =  15;
  STM_LOCATE            =  16;

Type
{ Printer IO request }
 pprinterIO = ^tprinterIO;
 tprinterIO = record
     ios : tIOStdReq;
     iodrp : tIODRPReq;
     iopc : tIOPrtCmdReq;
 end;
{ DTM_PRINT }

        pdtPrint = ^tdtPrint;
        tdtPrint = record
            MethodID : ULONG;
            dtp_GInfo : pGadgetInfo;
            dtp_PIO : pprinterIO;
            dtp_AttrList : pTagItem;
         end;


{ DTM_DRAW }
 pdtDraw = ^tdtDraw;
 tdtDraw = record
    MethodID             : ULONG;
    dtd_RPort            : pRastPort;
    dtd_Left,
    dtd_Top,
    dtd_Width,
    dtd_Height,
    dtd_TopHoriz,
    dtd_TopVert          : Longint;
    dtd_AttrList         : pTagItem;          { Additional attributes }
 end;

{ DTM_WRITE }
 pdtWrite = ^tdtWrite;
 tdtWrite = record
    MethodID             : ULONG;
    dtw_GInfo            : pGadgetInfo;       { Gadget information }
    dtw_FileHandle       : BPTR;              { File handle to write to }
    dtw_Mode             : ULONG;
    dtw_AttrList         : pTagItem;          { Additional attributes }
 end;

const
{ Save data as IFF data }
  DTWM_IFF       = 0;

{ Save data as local data format }
  DTWM_RAW       = 1;

{***************************************************************************}

   PICTUREDTCLASS        : PChar =  'picture.datatype';

{***************************************************************************}

{ Picture attributes }
   PDTA_ModeID           =  (DTA_Dummy + 200);
        { Mode ID of the picture }

   PDTA_BitMapHeader     =  (DTA_Dummy + 201);

   PDTA_BitMap           =  (DTA_Dummy + 202);
        { Pointer to a class-allocated bitmap, that will end
         * up being freed by picture.class when DisposeDTObject()
         * is called }
{ Picture colour table (struct ColorRegister *) }
   PDTA_ColorRegisters   =  (DTA_Dummy + 203);

{ Color table to use with SetRGB32CM() (ULONG *) }
   PDTA_CRegs            =  (DTA_Dummy + 204);

{ Color table; this table is initialized during the layout
 * process and will contain the colours the picture will use
 * after remapping. If no remapping takes place, these colours
 * will match those in the PDTA_CRegs table (ULONG *).
 }
   PDTA_GRegs            =  (DTA_Dummy + 205);

{ Shared pen table; this table is initialized during the layout
 * process while the picture is being remapped (UBYTE *).
 }
   PDTA_ColorTable       =  (DTA_Dummy + 206);

{ Shared pen table; in most places this table will be identical to
 * the PDTA_ColorTable table. Some of the colours in this table might
 * match the original colour palette a little better than the colours
 * picked for the other table. The picture.datatype uses the two tables
 * during remapping, alternating for each pixel (UBYTE *).
 }
   PDTA_ColorTable2      =  (DTA_Dummy + 207);

{ OBSOLETE; DO NOT USE }
   PDTA_Allocated        =  (DTA_Dummy + 208);

{ Number of colors used by the picture. (UWORD) }
   PDTA_NumColors        =  (DTA_Dummy + 209);

{ Number of colors allocated by the picture (UWORD) }
   PDTA_NumAlloc         =  (DTA_Dummy + 210);

   PDTA_Remap            =  (DTA_Dummy + 211);
        { Boolean : Remap picture (defaults to TRUE) }

   PDTA_Screen           =  (DTA_Dummy + 212);
        { Screen to remap to }

   PDTA_FreeSourceBitMap =  (DTA_Dummy + 213);
        { Boolean : Free the source bitmap after remapping }

   PDTA_Grab             =  (DTA_Dummy + 214);
        { Pointer to a Point structure }

   PDTA_DestBitMap       =  (DTA_Dummy + 215);
        { Pointer to the destination (remapped) bitmap }

   PDTA_ClassBitMap      =  (DTA_Dummy + 216);
        { Pointer to class-allocated bitmap, that will end
         * up being freed by the class after DisposeDTObject()
         * is called }

   PDTA_NumSparse        =  (DTA_Dummy + 217);
        { (UWORD) Number of colors used for sparse remapping }

   PDTA_SparseTable      =  (DTA_Dummy + 218);
        { (UBYTE *) Pointer to a table of pen numbers indicating
         * which colors should be used when remapping the image.
         * This array must contain as many entries as there
         * are colors specified with PDTA_NumSparse }

         { Index number of the picture to load (ULONG). (V44) }
   PDTA_WhichPicture     = (DTA_Dummy + 219);

{ Get the number of pictures stored in the file (ULONG *). (V44) }
   PDTA_GetNumPictures   = (DTA_Dummy + 220);

{ Maximum number of colours to use for dithering (ULONG). (V44) }
   PDTA_MaxDitherPens    = (DTA_Dummy + 221);

{ Quality of the dithering algorithm to be used during colour
 * quantization (ULONG). (V44)
 }
   PDTA_DitherQuality    = (DTA_Dummy + 222);

{ Pointer to the allocated pen table (UBYTE *). (V44) }
   PDTA_AllocatedPens    = (DTA_Dummy + 223);

{ Quality for scaling. (V45) }
   PDTA_ScaleQuality     = (DTA_Dummy + 224);

{***************************************************************************}

{ When querying the number of pictures stored in a file, the
 * following value denotes "the number of pictures is unknown".
 }
   PDTANUMPICTURES_Unknown = (0);

{***************************************************************************}

{ V43 extensions (attributes) }

{ Set the sub datatype interface mode (LONG); see "Interface modes" below }
   PDTA_SourceMode              = (DTA_Dummy + 250);

{ Set the app datatype interface mode (LONG); see "Interface modes" below }
   PDTA_DestMode                = (DTA_Dummy + 251);

{ Allocates the resulting bitmap as a friend bitmap (BOOL) }
   PDTA_UseFriendBitMap         = (DTA_Dummy + 255);

{ NULL or mask plane for use with BltMaskBitMapRastPort() (PLANEPTR) }
   PDTA_MaskPlane               = (DTA_Dummy + 258);

{***************************************************************************}

{ Interface modes }
  PMODE_V42 = (0);      { Compatibility mode }
  PMODE_V43 = (1);      { Extended mode }

{***************************************************************************}

{ V43 extensions (methods) }

  PDTM_Dummy = (DTM_Dummy + $60);

{ Transfer pixel data to the picture object in the specified format }
  PDTM_WRITEPIXELARRAY = (PDTM_Dummy + 0);

{ Transfer pixel data from the picture object in the specified format }
  PDTM_READPIXELARRAY = (PDTM_Dummy + 1);

{***************************************************************************}

{  Masking techniques  }
   mskNone                = 0;
   mskHasMask             = 1;
   mskHasTransparentColor = 2;
   mskLasso               = 3;
   mskHasAlpha            = 4;

{  Compression techniques  }
   cmpNone                = 0;
   cmpByteRun1            = 1;
   cmpByteRun2            = 2;

Type
{  Bitmap header (BMHD) structure  }
 pBitMapHeader = ^tBitMapHeader;
 tBitMapHeader = record
    bmh_Width,                         { Width in pixels }
    bmh_Height   : Word;               { Height in pixels }
    bmh_Left,                          { Left position }
    bmh_Top      : smallint;            { Top position }
    bmh_Depth,                         { Number of planes }
    bmh_Masking,                       { Masking type }
    bmh_Compression,                   { Compression type }
    bmh_Pad      : Byte;
    bmh_Transparent : WORD;            { Transparent color }
    bmh_XAspect,
    bmh_YAspect     : Byte;
    bmh_PageWidth,
    bmh_PageHeight  : smallint;
 end;

{***************************************************************************}

{  Color register structure }
 pColorRegister = ^tColorRegister;
 tColorRegister = record
   red, green, blue : Byte;
 end;

{***************************************************************************}

const
{ IFF types that may be in pictures }
   ID_ILBM         = 1229734477;
   ID_BMHD         = 1112361028;
   ID_BODY         = 1112491097;
   ID_CMAP         = 1129136464;
   ID_CRNG         = 1129467463;
   ID_GRAB         = 1196572994;
   ID_SPRT         = 1397772884;
   ID_DEST         = 1145394004;
   ID_CAMG         = 1128353095;

{***************************************************************************}

   SOUNDDTCLASS          : PChar =  'sound.datatype';

{***************************************************************************}

{ Sound attributes }
   SDTA_Dummy            =  (DTA_Dummy + 500);
   SDTA_VoiceHeader      =  (SDTA_Dummy + 1);
   SDTA_Sample           =  (SDTA_Dummy + 2);
   { (UBYTE *) Sample data }

   SDTA_SampleLength     =  (SDTA_Dummy + 3);
   { (ULONG) Length of the sample data in UBYTEs }

   SDTA_Period           =  (SDTA_Dummy + 4);
    { (UWORD) Period }

   SDTA_Volume           =  (SDTA_Dummy + 5);
    { (UWORD) Volume.  Range from 0 to 64 }

   SDTA_Cycles           =  (SDTA_Dummy + 6);

{ The following tags are new for V40 }
   SDTA_SignalTask       =  (SDTA_Dummy + 7);
    { (struct Task *) Task to signal when sound is complete or
        next buffer needed. }

{ (ULONG) Signal mask to use on completion or 0 to disable
 *
 *         NOTE: Due to a bug in sound.datatype V40 SDTA_SignalBit
 *               was actually implemented as a signal mask as opposed
 *               to a bit number. The documentation now reflects
 *               this. If you intend to use a signal bit number
 *               instead of the mask, use the new V44 tag
 *               SDTA_SignalBitNumber below.
 *}
   SDTA_SignalBit        =  (SDTA_Dummy + 8);
   SDTA_SignalBitMask    = SDTA_SignalBit;
    { (BYTE) Signal bit to use on completion or -1 to disable }

   SDTA_Continuous       =  (SDTA_Dummy + 9);
    { (ULONG) Playing a continuous stream of data.  Defaults to
        FALSE. }

{ The following tags are new for V44 }

{ (BYTE) Signal bit to use on completion or -1 to disable }
   SDTA_SignalBitNumber  =  (SDTA_Dummy + 10);

{ (UWORD) Samples per second }
   SDTA_SamplesPerSec    = (SDTA_Dummy + 11);

{ (struct timeval *) Sample replay period }
   SDTA_ReplayPeriod     = (SDTA_Dummy + 12);

{ (BYTE *) Sample data }
   SDTA_LeftSample       = (SDTA_Dummy + 13);
   SDTA_RightSample      = (SDTA_Dummy + 14);

{ (BYTE) Stereo panning }
   SDTA_Pan              = (SDTA_Dummy + 15);

{ (BOOL) FreeVec() all sample data upon OM_DISPOSE. }
   SDTA_FreeSampleData   = (SDTA_Dummy + 16);

{ (BOOL) Wait for the current sample to be played back before
 * switching to the new sample data.
 }
   SDTA_SyncSampleChange = (SDTA_Dummy + 17);

{***************************************************************************}

   CMP_NONE     = 0;
   CMP_FIBDELTA = 1;

Type
 pVoiceHeader = ^tVoiceHeader;
 tVoiceHeader = record
    vh_OneShotHiSamples,
    vh_RepeatHiSamples,
    vh_SamplesPerHiCycle : ULONG;
    vh_SamplesPerSec     : WORD;
    vh_Octaves,
    vh_Compression       : Byte;
    vh_Volume            : ULONG;
 end;

{***************************************************************************}

const

{ Channel allocation }
   SAMPLETYPE_Left      = (2);
   SAMPLETYPE_Right     = (4);
   SAMPLETYPE_Stereo    = (6);

{ IFF types }
   ID_8SVX = 944985688;
   ID_VHDR = 1447576658;
   ID_CHAN = $4348414E;

{***************************************************************************}

{ ***************************************************************************}

   TEXTDTCLASS           : PChar =  'text.datatype';

{ ***************************************************************************}

{  Text attributes }
   TDTA_Buffer           =  (DTA_Dummy + 300);
   TDTA_BufferLen        =  (DTA_Dummy + 301);
   TDTA_LineList         =  (DTA_Dummy + 302);
   TDTA_WordSelect       =  (DTA_Dummy + 303);
   TDTA_WordDelim        =  (DTA_Dummy + 304);
   TDTA_WordWrap         =  (DTA_Dummy + 305);
     {  Boolean. Should the text be word wrapped.  Defaults to false. }

{ ***************************************************************************}

Type
{  There is one Line structure for every line of text in our document.  }
 pLine = ^tLine;
 tLine = record
    ln_Link              : tMinNode;            {  to link the lines together }
    ln_Text              : STRPTR;              {  pointer to the text for this line }
    ln_TextLen           : ULONG;               {  the character length of the text for this line }
    ln_XOffset,                                 {  where in the line the text starts }
    ln_YOffset,                                 {  line the text is on }
    ln_Width,                                   {  Width of line in pixels }
    ln_Height,                                  {  Height of line in pixels }
    ln_Flags             : WORD;                {  info on the line }
    ln_FgPen,                                   {  foreground pen }
    ln_BgPen             : Shortint;            {  background pen }
    ln_Style             : ULONG;               {  Font style }
    ln_Data              : Pointer;             {  Link data... }
 end;

{ ***************************************************************************}

const
{  Line.ln_Flags }

{  Line Feed }
   LNF_LF        = 1;

{  Segment is a link }
   LNF_LINK      = 2;

{  ln_Data is a pointer to an DataTypes object }
   LNF_OBJECT    = 4;

{  Object is selected }
   LNF_SELECTED  = 8;

{ ***************************************************************************}

{  IFF types that may be text }
   ID_FTXT         = 1179932756;
   ID_CHRS         = 1128813139;

{ ***************************************************************************}

   ANIMATIONDTCLASS         : PChar =       'animation.datatype';

{ ***************************************************************************}

{  Animation attributes }
   ADTA_Dummy            =  (DTA_Dummy + 600);
   ADTA_ModeID           =  PDTA_ModeID      ;
   ADTA_KeyFrame         =  PDTA_BitMap      ;
        {  (struct BitMap *) Key frame (first frame) bitmap }

   ADTA_ColorRegisters   =  PDTA_ColorRegisters;
   ADTA_CRegs            =  PDTA_CRegs         ;
   ADTA_GRegs            =  PDTA_GRegs         ;
   ADTA_ColorTable       =  PDTA_ColorTable    ;
   ADTA_ColorTable2      =  PDTA_ColorTable2   ;
   ADTA_Allocated        =  PDTA_Allocated     ;
   ADTA_NumColors        =  PDTA_NumColors     ;
   ADTA_NumAlloc         =  PDTA_NumAlloc      ;

   ADTA_Remap            =  PDTA_Remap;
        {  (BOOL) : Remap animation (defaults to TRUE) }

   ADTA_Screen           =  PDTA_Screen;
        {  (struct Screen *) Screen to remap to }

   ADTA_NumSparse        =  PDTA_NumSparse;
        {  (UWORD) Number of colors used for sparse remapping }

   ADTA_SparseTable      =  PDTA_SparseTable;
        {  (UBYTE *) Pointer to a table of pen numbers indicating
         * which colors should be used when remapping the image.
         * This array must contain as many entries as there
         * are colors specified with ADTA_NumSparse }

   ADTA_Width            =  (ADTA_Dummy + 1);
   ADTA_Height           =  (ADTA_Dummy + 2);
   ADTA_Depth            =  (ADTA_Dummy + 3);
   ADTA_Frames           =  (ADTA_Dummy + 4);
        {  (ULONG) Number of frames in the animation }

   ADTA_Frame            =  (ADTA_Dummy + 5);
        {  (ULONG) Current frame }

   ADTA_FramesPerSecond  =  (ADTA_Dummy + 6);
        {  (ULONG) Frames per second }

   ADTA_FrameIncrement   =  (ADTA_Dummy + 7);
        {  (LONG) Amount to change frame by when fast forwarding or
         * rewinding.  Defaults to 10. }

   ADTA_PreloadFrameCount = (ADTA_Dummy + 8);   { (V44) }
{  Sound attributes }
   ADTA_Sample           =  SDTA_Sample      ;
   ADTA_SampleLength     =  SDTA_SampleLength;
   ADTA_Period           =  SDTA_Period      ;
   ADTA_Volume           =  SDTA_Volume      ;
   ADTA_Cycles           =  SDTA_Cycles      ;

   ADTA_LeftSample       =  SDTA_LeftSample;            { (V44) }
   ADTA_RightSample      =  SDTA_RightSample;   { (V44) }
   ADTA_SamplesPerSec    =  SDTA_SamplesPerSec; { (V44) }

{ ***************************************************************************}

   ID_ANIM   = 1095649613;
   ID_ANHD   = 1095649348;
   ID_DLTA   = 1145852993;

{ ***************************************************************************}

{   Required ANHD structure describes an ANIM frame }
Type
 pAnimHeader = ^tAnimHeader;
 tAnimHeader = record
    ah_Operation   : Byte;  {   The compression method:
                                     0  set directly (normal ILBM BODY),
                                     1  XOR ILBM mode,
                                     2  Long Delta mode,
                                     3  Short Delta mode,
                                     4  Generalized short/long Delta mode,
                                     5  Byte Vertical Delta mode
                                     6  Stereo op 5 (third party)
                                    74  (ascii 'J') reserved for Eric Graham's
                                        compression technique (details to be
                                        released later). }

    ah_Mask        : Byte;      {  (XOR mode only - plane mask where each
                                   bit is set =1 if there is data and =0
                                   if not.) }

    ah_Width,                   {  (XOR mode only - width and height of the }
    ah_Height,                  {  area represented by the BODY to eliminate }
                                {  unnecessary un-changed data) }


    ah_Left,                    {  (XOR mode only - position of rectangular }
    ah_Top         : WORD;      {  area representd by the BODY) }


    ah_AbsTime,                 {  Timing for a frame relative to the time
                                   the first frame was displayed, in
                                   jiffies (1/60 sec) }

    ah_RelTime     : ULONG;    {  Timing for frame relative to time
                                   previous frame was displayed - in
                                   jiffies (1/60 sec) }

    ah_Interleave,              {  Indicates how may frames back this data is to
                                   modify.  0 defaults to indicate two frames back
                                   (for double buffering). n indicates n frames back.
                                   The main intent here is to allow values
                                   of 1 for special applications where
                                   frame data would modify the immediately
                                   previous frame. }

    ah_Pad0        :  Byte;     {  Pad byte, not used at present. }

    ah_Flags       : ULONG;     {  32 option bits used by options=4 and 5.
                                   At present only 6 are identified, but the
                                   rest are set =0 so they can be used to
                                   implement future ideas.  These are defined
                                   for option 4 only at this point.  It is
                                   recommended that all bits be set =0 for
                                   option 5 and that any bit settings
                                   used in the future (such as for XOR mode)
                                   be compatible with the option 4
                                   bit settings.   Player code should check
                                   undefined bits in options 4 and 5 to assure
                                   they are zero.

                                   The six bits for current use are:

                                    bit #       set =0                  set =1
                                    ===============================================
                                    0           short data              long data
                                    1           set                     XOR
                                    2           separate info           one info list
                                                for each plane          for all planes
                                    3           not RLC                 RLC (run length coded)
                                    4           horizontal              vertical
                                    5           short info offsets      long info offsets
                                }

    ah_Pad  : Array[0..15] of Byte;    {  This is a pad for future use for future
                                   compression modes. }
 end;

{ ***************************************************************************}

const
   ADTM_Dummy            =  ($700);

   ADTM_LOADFRAME        =  ($701);
    {  Used to load a frame of the animation }

   ADTM_UNLOADFRAME      =  ($702);
    {  Used to unload a frame of the animation }

   ADTM_START            =  ($703);
    {  Used to start the animation }

   ADTM_PAUSE            =  ($704);
    {  Used to pause the animation (don't reset the timer) }

   ADTM_STOP             =  ($705);
    {  Used to stop the animation }

   ADTM_LOCATE           =  ($706);
    {  Used to locate a frame in the animation (as set by a slider...) }

    { Used to load a new format frame of the animation (V44) }
   ADTM_LOADNEWFORMATFRAME =    ($707);

    { Used to unload a new format frame of the animation (V44) }
   ADTM_UNLOADNEWFORMATFRAME  = ($708);
{ ***************************************************************************}

{  ADTM_LOADFRAME, ADTM_UNLOADFRAME }
Type
 padtFrame = ^tadtFrame;
 tadtFrame = record
    MethodID,
    alf_TimeStamp,         {  Timestamp of frame to load }

    {  The following fields are filled in by the ADTM_LOADFRAME method, }
    {  and are read-only for any other methods. }

    alf_Frame,                        {  Frame number }
    alf_Duration  :  ULONG;           {  Duration of frame }

    alf_BitMap    :  pBitMap;         {  Loaded BitMap }
    alf_CMap      :  pColorMap;       {  Colormap, if changed }

    alf_Sample    :  Pointer;         {  Sound data }
    alf_SampleLength,
    alf_Period    : ULONG;

    alf_UserData  : Pointer;          {  Used by load frame for extra data }
 end;

 { ADTM_LOADNEWFORMATFRAME, ADTM_UNLOADNEWFORMATFRAME }

 PadtNewFormatFrame = ^tadtNewFormatFrame;
     tadtNewFormatFrame = record
          MethodID : ULONG;
          alf_TimeStamp : ULONG;     { Timestamp of frame to load }
          { The following fields are filled in by the ADTM_NEWLOADFRAME method, }
    { and are read-only for any other methods. }
          alf_Frame : ULONG;         { Frame number }
          alf_Duration : ULONG;      { Duration of frame }
          alf_BitMap : PBitMap;      { Loaded BitMap }
          alf_CMap : PColorMap;      { Colormap, if changed }
          alf_Sample : PBYTE;
          alf_SampleLength : ULONG;  { Sound data }
          alf_Period : ULONG;
          alf_UserData : APTR;       { Used by load frame for extra data }
          alf_Size : ULONG;          { Size of this data structure (in bytes) }
          alf_LeftSample : PBYTE;    { Sound for left channel, or NULL if none }
          alf_RightSample : PBYTE;   { Sound for right channel, or NULL if none }
          alf_SamplesPerSec : ULONG; { Replay speed; if > 0, this overrides alf_Period }
       end;


{  ADTM_START, ADTM_PAUSE, ADTM_STOP, ADTM_LOCATE }
 padtStart = ^tadtStart;
 tadtStart = record
    MethodID,
    asa_Frame : ULONG;             {  Frame # to start at }
 end;

{ ***************************************************************************}

VAR DataTypesBase : pLibrary;

const
    DATATYPESNAME : PChar = 'datatypes.library';

{
    Just a note.
    You will see a lot of pObject_ here, pObject is
    defined in intuition.

    In c it's object * but we can't have object in fpc.
    typedef object ULONG

    pObject_ is just pULONG.

}

FUNCTION AddDTObject(win : pWindow; req : pRequester; o : pObject_; pos : LONGINT) : LONGINT;
PROCEDURE DisposeDTObject(o : pObject_);
FUNCTION DoAsyncLayout(o : pObject_; gpl : pgpLayout) : ULONG;
FUNCTION DoDTMethodA(o : pObject_; win : pWindow; req : pRequester; msg : pLONGINT) : ULONG;
FUNCTION GetDTAttrsA(o : pObject_; attrs : pTagItem) : ULONG;
FUNCTION GetDTMethods(obj : pObject_) : Pointer;
FUNCTION GetDTString(id : ULONG) : pCHAR;
FUNCTION GetDTTriggerMethods(obj : pObject_) : pDTMethod;
FUNCTION NewDTObjectA(name : POINTER; attrs : pTagItem): POINTER;
FUNCTION ObtainDataTypeA(typ : ULONG; handle : POINTER; attrs : pTagItem) : pDataType;
FUNCTION PrintDTObjectA(o : pObject_; w : pWindow; r : pRequester; msg : pdtPrint) : ULONG;
PROCEDURE RefreshDTObjectA(o : pObject_; win : pWindow; req : pRequester; attrs : pTagItem);
PROCEDURE ReleaseDataType(dt : pDataType);
FUNCTION RemoveDTObject(win : pWindow; o : pObject_) : LONGINT;
FUNCTION SetDTAttrsA(o : pObject_; win : pWindow; req : pRequester; attrs : pTagItem) : ULONG;

FUNCTION ObtainDTDrawInfoA( o : pObject_; attrs : pTagItem) : POINTER;
FUNCTION DrawDTObjectA(rp : pRastPort; o : pObject_; x : LONGINT; y : LONGINT; w : LONGINT; h : LONGINT; th : LONGINT; tv : LONGINT; attrs : pTagItem) : LONGINT;
PROCEDURE ReleaseDTDrawInfo( o : pObject_; handle : POINTER);

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitDATATYPESLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    DATATYPESIsCompiledHow : longint;


IMPLEMENTATION

{$ifndef dont_use_openlib}
uses msgbox;
{$endif dont_use_openlib}

FUNCTION AddDTObject(win : pWindow; req : pRequester; o : pObject_; pos : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L req,A1
    MOVEA.L o,A2
    MOVE.L  pos,D0
    MOVEA.L DataTypesBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DisposeDTObject(o : pObject_);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L DataTypesBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DoAsyncLayout(o : pObject_; gpl : pgpLayout) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L gpl,A1
    MOVEA.L DataTypesBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DoDTMethodA(o : pObject_; win : pWindow; req : pRequester; msg : pLONGINT) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L win,A1
    MOVEA.L req,A2
    MOVEA.L msg,A3
    MOVEA.L DataTypesBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetDTAttrsA(o : pObject_; attrs : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L attrs,A2
    MOVEA.L DataTypesBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetDTMethods(obj : pObject_) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L obj,A0
    MOVEA.L DataTypesBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetDTString(id : ULONG) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  id,D0
    MOVEA.L DataTypesBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetDTTriggerMethods(obj : pObject_) : pDTMethod;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L obj,A0
    MOVEA.L DataTypesBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NewDTObjectA(name : POINTER; attrs : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  name,D0
    MOVEA.L attrs,A0
    MOVEA.L DataTypesBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ObtainDataTypeA(typ : ULONG; handle : POINTER; attrs : pTagItem) : pDataType;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  typ,D0
    MOVEA.L handle,A0
    MOVEA.L attrs,A1
    MOVEA.L DataTypesBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PrintDTObjectA(o : pObject_; w : pWindow; r : pRequester; msg : pdtPrint) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L w,A1
    MOVEA.L r,A2
    MOVEA.L msg,A3
    MOVEA.L DataTypesBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RefreshDTObjectA(o : pObject_; win : pWindow; req : pRequester; attrs : pTagItem);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L win,A1
    MOVEA.L req,A2
    MOVEA.L attrs,A3
    MOVEA.L DataTypesBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ReleaseDataType(dt : pDataType);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dt,A0
    MOVEA.L DataTypesBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RemoveDTObject(win : pWindow; o : pObject_) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L o,A1
    MOVEA.L DataTypesBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetDTAttrsA(o : pObject_; win : pWindow; req : pRequester; attrs : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L o,A0
    MOVEA.L win,A1
    MOVEA.L req,A2
    MOVEA.L attrs,A3
    MOVEA.L DataTypesBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;


FUNCTION ObtainDTDrawInfoA( o : pObject_; attrs : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L o,A0
        MOVEA.L attrs,A1
        MOVEA.L DataTypesBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DrawDTObjectA(rp : pRastPort;  o : pObject_; x : LONGINT; y : LONGINT; w : LONGINT; h : LONGINT; th : LONGINT; tv : LONGINT; attrs : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A0
        MOVEA.L o,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVE.L  w,D2
        MOVE.L  h,D3
        MOVE.L  th,D4
        MOVE.L  tv,D5
        MOVEA.L attrs,A2
        MOVEA.L DataTypesBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ReleaseDTDrawInfo( o : pObject_; handle : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L o,A0
        MOVEA.L handle,A1
        MOVEA.L DataTypesBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
  END;
END;


const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of datatypes.library}
  {$Info don't forget to use InitDATATYPESLibrary in the beginning of your program}

var
    datatypes_exit : Pointer;

procedure ClosedatatypesLibrary;
begin
    ExitProc := datatypes_exit;
    if DataTypesBase <> nil then begin
        CloseLibrary(DataTypesBase);
        DataTypesBase := nil;
    end;
end;

procedure InitDATATYPESLibrary;
begin
    DataTypesBase := nil;
    DataTypesBase := OpenLibrary(DATATYPESNAME,LIBVERSION);
    if DataTypesBase <> nil then begin
        datatypes_exit := ExitProc;
        ExitProc := @ClosedatatypesLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open datatypes.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    DATATYPESIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of datatypes.library}

var
    datatypes_exit : Pointer;

procedure ClosedatatypesLibrary;
begin
    ExitProc := datatypes_exit;
    if DataTypesBase <> nil then begin
        CloseLibrary(DataTypesBase);
        DataTypesBase := nil;
    end;
end;

begin
    DataTypesBase := nil;
    DataTypesBase := OpenLibrary(DATATYPESNAME,LIBVERSION);
    if DataTypesBase <> nil then begin
        datatypes_exit := ExitProc;
        ExitProc := @ClosedatatypesLibrary;
        DATATYPESIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open datatypes.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    DATATYPESIsCompiledHow := 3;
   {$Warning No autoopening of datatypes.library compiled}
   {$Warning Make sure you open datatypes.library yourself}
{$endif dont_use_openlib}


END. (* UNIT DATATYPES *)





