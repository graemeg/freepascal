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
{$PACKRECORDS 2}

unit datatypes;

interface

uses
  exec, amigados, intuition, utility,
  agraphics, iffparse, amigaprinter, prtbase;

// datatypes
const
  ID_DTYP = Ord('D') shl 24 + Ord('T') shl 16 + Ord('Y') shl 8 + Ord('P'); // DTYP
  ID_DTHD = Ord('D') shl 24 + Ord('T') shl 16 + Ord('H') shl 8 + Ord('D'); // DTHD

Type
  PDataTypeHeader = ^TDataTypeHeader;
  TDataTypeHeader = record
    dth_Name: STRPTR;       // Descriptive name of the data type
    dth_BaseName: STRPTR;   // Base name of the data type
    dth_Pattern: STRPTR;    // Match pattern for file name.
    dth_Mask: PSmallInt;    // Comparision mask
    dth_GroupID: LongWord;  // Group that the DataType is in
    dth_ID: LongWord;       // ID for DataType (same as IFF FORM type)
    dth_MaskLen: SmallInt;  // Length of comparision mask
    dth_Pad: SmallInt;      // Unused at present (must be 0)
    dth_Flags: Word;        // Flags
    dth_Priority: Word;     // Priority
 end;
const
  DTHSIZE = SizeOf(TDataTypeHeader);

// Basic type - TDataTypeHeader.dth_Flags
  DTF_TYPE_MASK = $000F;
  DTF_BINARY    = $0000;
  DTF_ASCII     = $0001;
  DTF_IFF       = $0002;
  DTF_MISC      = $0003;
  DTF_CASE      = $0010; // Set if case is important
  DTF_SYSTEM1   = $1000; // Reserved for system use

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

  // System file, such as; directory, executable, library, device, font, etc.
  GID_SYSTEM    = Ord('s') shl 24 + Ord('y') shl 16 + Ord('s') shl 8 + Ord('t'); // syst
  // Formatted or unformatted text
  GID_TEXT = Ord('t') shl 24 + Ord('e') shl 16 + Ord('x') shl 8 + Ord('t'); // text
  // Formatted text combined with graphics or other DataTypes
  GID_DOCUMENT = Ord('d') shl 24 + Ord('o') shl 16 + Ord('c') shl 8 + Ord('u'); // docu
  // Sound
  GID_SOUND = Ord('s') shl 24 + Ord('o') shl 16 + Ord('u') shl 8 + Ord('n'); // soun
  // Musical instrument
  GID_INSTRUMENT = Ord('i') shl 24 + Ord('n') shl 16 + Ord('s') shl 8 + Ord('t'); // inst
  // Musical score
  GID_MUSIC = Ord('m') shl 24 + Ord('u') shl 16 + Ord('s') shl 8 + Ord('i'); // musi
  // Picture
  GID_PICTURE = Ord('p') shl 24 + Ord('i') shl 16 + Ord('c') shl 8 + Ord('t'); // pict
  //* Animated pictures */
  GID_ANIMATION = Ord('a') shl 24 + Ord('n') shl 16 + Ord('i') shl 8 + Ord('m'); // anim
  //* Animation with audio */
  GID_MOVIE = Ord('m') shl 24 + Ord('o') shl 16 + Ord('v') shl 8 + Ord('i'); // movi

  // A code chunk contains an embedded executable that can be loaded with InternalLoadSeg.
  ID_CODE = Ord('D') shl 24 + Ord('T')  shl 16 + Ord('C') shl 8 + Ord('D'); // DTCD

// DataTypes comparision hook context (Read-Only).  This is the argument that is passed to a custom comparision routine.
type
  PTHookContext = ^TDTHookContext;
  TDTHookContext = record
    dthc_SysBase: PLibrary;
    dthc_DOSBase: PLibrary;
    dthc_IFFParseBase: PLibrary;
    dthc_UtilityBase: PLibrary;
    // File context
    dthc_Lock: BPTR;             // Lock on the file
    dthc_FIB: PFileInfoBlock;    // Pointer to a FileInfoBlock
    dthc_FileHandle: BPTR;       // Pointer to file handle (may be nil)
    dthc_IFF: PIFFHandle;        // Pointer to IFFHandle (may be nil)
    dthc_Buffer: STRPTR;         // Buffer...
    dthc_BufferLength: LongWord; // ... and corresponding length
 end;

const
  ID_TOOL = Ord('D') shl 24 + Ord('T') shl 16 + Ord('T') shl 8 + Ord('L'); // DTTL

type
  PTool = ^TTool;
  TTool = record
    tn_Which: Word;      // Which tool is this TW_*
    tn_Flags: Word;      // Flags TF_*
    tn_Program: STRPTR;  // Application to use
  end;

const
  TOOLSIZE = SizeOf(TTool); // orig TSIZE but clash with type TSize

  // defines for tn_Which
  TW_INFO   = 1;
  TW_BROWSE = 2;
  TW_EDIT   = 3;
  TW_PRINT  = 4;
  TW_MAIL   = 5;

  // defines for tn_Flags
  TF_LAUNCH_MASK = $000F;
  TF_SHELL       = $0001;
  TF_WORKBENCH   = $0002;
  TF_RX          = $0003;

  ID_TAGS = Ord('D') shl 24 + Ord('T') shl 16 + Ord('T') shl 8 + Ord('G'); // DTTG

type
  PDataType = ^TDataType;
  TDataType = record
    dtn_Node1: TNode;            // Reserved for system use
    dtn_Node2: TNode;            // Reserved for system use
    dtn_Header: PDataTypeHeader; // Pointer to the DataTypeHeader
    dtn_ToolList: TList;         // List of tool nodes
    dtn_FunctionName: STRPTR;    // Name of comparison routine
    dtn_AttrList: PTagItem;      // Object creation tags
    dtn_Length: LongWord;        // Length of the memory block
  end;

const
  DTNSIZE = SizeOf(TDataType);

type
  PToolNode = ^TToolNode;
  TToolNode = record
    tn_Node: TNode;      // Embedded node
    tn_Tool: TTool;      // Embedded tool
    tn_Length: LongWord; // Length of the memory block
  end;
const
  TNSIZE = SizeOf(TToolNode);

  ID_NAME = Ord('N') shl 24 + Ord('A') shl 16 + Ord('M') shl 8 + Ord('E');

  // Text ID's
  DTERROR_UNKNOWN_DATATYPE     = 2000;
  DTERROR_COULDNT_SAVE         = 2001;
  DTERROR_COULDNT_OPEN         = 2002;
  DTERROR_COULDNT_SEND_MESSAGE = 2003;
  // new for V40
  DTERROR_COULDNT_OPEN_CLIPBOARD = 2004;
  DTERROR_Reserved               = 2005;
  DTERROR_UNKNOWN_COMPRESSION    = 2006;
  DTERROR_NOT_ENOUGH_DATA        = 2007;
  DTERROR_INVALID_DATA           = 2008;
  // New for V44
  DTERROR_NOT_AVAILABLE = 2009;
  // Offset for types }
  DTMSG_TYPE_OFFSET = 2100;

// datatypesclass
const
  DATATYPESCLASS = 'datatypesclass';

  DTA_Dummy = TAG_USER + $1000;

  DTA_TextAttr       = DTA_Dummy + 10; // (PTextAttr) Default TextAttr to use for text within the object
  DTA_TopVert        = DTA_Dummy + 11; // (LongInt) Top vertical unit
  DTA_VisibleVert    = DTA_Dummy + 12; // (LongInt) Number of visible vertical units
  DTA_TotalVert      = DTA_Dummy + 13; // (LongInt) Total number of vertical units
  DTA_VertUnit       = DTA_Dummy + 14; // (LongInt) Number of pixels per vertical unit
  DTA_TopHoriz       = DTA_Dummy + 15; // (LongInt) Top horizontal unit
  DTA_VisibleHoriz   = DTA_Dummy + 16; // (LongInt) Number of visible horizontal units
  DTA_TotalHoriz     = DTA_Dummy + 17; // (LongInt) Total number of horiziontal units
  DTA_HorizUnit      = DTA_Dummy + 18; // (LongInt) Number of pixels per horizontal unit
  DTA_NodeName       = DTA_Dummy + 19; // (PByte) Name of the current element within the object
  DTA_Title          = DTA_Dummy + 20; // (STRPTR) Object's title
  DTA_TriggerMethods = DTA_Dummy + 21; // (PDTMethod) Pointer to a nil terminated array of trigger methods
  DTA_Data           = DTA_Dummy + 22; // (APTR) Object data
  DTA_TextFont       = DTA_Dummy + 23; // (PTextFont) Default font to use
  DTA_Methods        = DTA_Dummy + 24; // (PLongWord) Pointer to an array (terminated with #0) of methods that the object supports
  DTA_PrinterStatus  = DTA_Dummy + 25; // (LongInt) Printer error message
  DTA_PrinterProc    = DTA_Dummy + 26; // (PProcess) PRIVATE! Pointer to the print process
  DTA_LayoutProc     = DTA_Dummy + 27; // (PProcess) PRIVATE! Pointer to the print process
  DTA_Busy           = DTA_Dummy + 28; // Turns the application's busy pointer on and off
  DTA_Sync           = DTA_Dummy + 29; // Indicate that new information has been loaded into an object.
                                       // (This is used for models that cache the DTA_TopVert-like tags.)
  DTA_BaseName       = DTA_Dummy + 30; // Base name of the class
  DTA_GroupID        = DTA_Dummy + 31; // Group that the object must belong to
  DTA_ErrorLevel     = DTA_Dummy + 32; // Error level
  DTA_ErrorNumber    = DTA_Dummy + 33; // datatypes.library error number
  DTA_ErrorString    = DTA_Dummy + 34; // Argument for datatypes.library error
  DTA_Conductor      = DTA_Dummy + 35; // (PByte) Name of a realtime.library conductor -- defaults to "Main"
  DTA_ControlPanel   = DTA_Dummy + 36; // (LongBool) Specify whether a control panel should be embedded into the object or not
                                       // (for example in the animation datatype) -- defaults to True
  DTA_Immediate      = DTA_Dummy + 37; // (LongBool) Should the object begin playing immediately? -- defaults to False
  DTA_Repeat         = DTA_Dummy + 38; // (LongBool) Indicate that the object should repeat playing -- defaults to False
  // V44
  DTA_SourceAddress  = DTA_Dummy + 39; // (APTR) V44: Address of object if of type DTST_MEMORY
  DTA_SourceSize     = DTA_Dummy + 40; // (LongWord) V44: Size of object if of type DTST_MEMORY

  DTA_Reserved       = DTA_Dummy + 41; // Reserved tag; DO NOT USE (V44)

// DTObject attributes
  DTA_Name          = DTA_Dummy + 100;
  DTA_SourceType    = DTA_Dummy + 101;
  DTA_Handle        = DTA_Dummy + 102;
  DTA_DataType      = DTA_Dummy + 103;
  DTA_Domain        = DTA_Dummy + 104;
  // Left, Top, Width, Height -> use the Gadgetclass Tags
  DTA_ObjName       = DTA_Dummy + 109;
  DTA_ObjAuthor     = DTA_Dummy + 110;
  DTA_ObjAnnotation = DTA_Dummy + 111;
  DTA_ObjCopyright  = DTA_Dummy + 112;
  DTA_ObjVersion    = DTA_Dummy + 113;
  DTA_ObjectID      = DTA_Dummy + 114;
  DTA_UserData      = DTA_Dummy + 115;
  // RelLeft, RelTop, RelWidth, RelHeight -> use the Gadgetclass Tags
  DTA_FrameInfo     = DTA_Dummy + 116;
  DTA_SelectDomain  = DTA_Dummy + 121;
  DTA_TotalPVert    = DTA_Dummy + 122;
  DTA_TotalPHoriz   = DTA_Dummy + 123;
  DTA_NominalVert   = DTA_Dummy + 124;
  DTA_NominalHoriz  = DTA_Dummy + 125;

// Printing attributes
  DTA_DestCols      = DTA_Dummy + 400; // (LongInt) Destination x width
  DTA_DestRows      = DTA_Dummy + 401; // (LongInt) Destination y height
  DTA_Special       = DTA_Dummy + 402; // (Word) Option flags
  DTA_RastPort      = DTA_Dummy + 403; // (PRastPort) RastPort used when printing
  DTA_ARexxPortName = DTA_Dummy + 404; // (STRPTR) Pointer to base name for ARexx port

  DTST_RAM       = 1;
  DTST_FILE      = 2;
  DTST_CLIPBOARD = 3;
  DTST_HOTLINK   = 4;
  DTST_MEMORY    = 5; // V44

// This structure is attached to the Gadget.SpecialInfo field of the gadget. Use Get/Set calls to access it.
type
  PDTSpecialInfo = ^TDTSpecialInfo;
  TDTSpecialInfo = record
    si_Lock: TSignalSemaphore;
    si_Flags: LongWord;

    si_TopVert: LongInt;   // Top row (in units)
    si_VisVert: LongInt;   // Number of visible rows (in units)
    si_TotVert: LongInt;   // Total number of rows (in units)
    si_OTopVert: LongInt;  // Previous top (in units)
    si_VertUnit: LongInt;  // Number of pixels in vertical unit

    si_TopHoriz: LongInt;  // Top column (in units)
    si_VisHoriz: LongInt;  // Number of visible columns (in units)
    si_TotHoriz: LongInt;  // Total number of columns (in units)
    si_OTopHoriz: LongInt; // Previous top (in units)
    si_HorizUnit: LongInt; // Number of pixels in horizontal unit
  end;

const
  DTSIF_LAYOUT     = 1 shl 0; // Object is in layout processing
  DTSIF_NEWSIZE    = 1 shl 1; // Object needs to be layed out
  DTSIF_DRAGGING   = 1 shl 2;
  DTSIF_DRAGSELECT = 1 shl 3;
  DTSIF_HIGHLIGHT  = 1 shl 4;
  DTSIF_PRINTING   = 1 shl 5; // Object is being printed
  DTSIF_LAYOUTPROC = 1 shl 6; // Object is in layout process

type
  PDTMethod = ^TDTMethod;
  TDTMethod = record
    dtm_Label: STRPTR;
    dtm_Command: STRPTR;
    dtm_Method: LongWord;
  end;

const
  DTM_Dummy          = $600;

  DTM_FRAMEBOX       = $601; // Inquire what environment an object requires
  DTM_PROCLAYOUT     = $602; // Same as GM_LAYOUT except guaranteed to be on a process already
  DTM_ASYNCLAYOUT    = $603; // Layout that is occurring on a process
  DTM_REMOVEDTOBJECT = $604; // When RemoveDTObject() is called

  DTM_SELECT         = $605;
  DTM_CLEARSELECTED  = $606;

  DTM_COPY           = $607;
  DTM_PRINT          = $608;
  DTM_ABORTPRINT     = $609;

  DTM_NEWMEMBER      = $610;
  DTM_DISPOSEMEMBER  = $611;

  DTM_GOTO           = $630;
  DTM_TRIGGER        = $631;

  DTM_OBTAINDRAWINFO  = $640;
  DTM_DRAW            = $641;
  DTM_RELEASEDRAWINFO = $642;

  DTM_WRITE           = $650;

type
  // Used to ask the object about itself
  PFrameInfo = ^TFrameInfo;
  TFrameInfo = record
    fri_PropertyFlags: LongWord; // DisplayInfo
    fri_Resolution: TPoint;      // DisplayInfo

    fri_RedBits: Byte;
    fri_GreenBits: Byte;
    fri_BlueBits: Byte;

    fri_Dimensions: record
      Width: LongWord;
      Height: LongWord;
      Depth: LongWord;
    end;

    fri_Screen: PScreen;
    fri_ColorMap: PColorMap;

    fri_Flags: LongWord; // FIF_*
  end;

const
  FIF_SCALABLE   = $1;
  FIF_SCROLLABLE = $2;
  FIF_REMAPPABLE = $4;

type
  // DTM_REMOVEDTOBJECT, DTM_CLEARSELECTED, DTM_COPY, DTM_ABORTPRINT
  PdtGeneral = ^TdtGeneral;
  TdtGeneral = record
    MethodID: LongWord;
    dtg_GInfo: PGadgetInfo;
  end;

  // DTM_SELECT
  PdtSelect = ^TdtSelect;
  TdtSelect = record
    MethodID: LongWord;
    dts_GInfo: PGadgetInfo;
    dts_Select: TRectangle;
  end;

  // DTM_FRAMEBOX
  PdtFrameBox = ^TdtFrameBox;
  TdtFrameBox = record
    MethodID: LongWord;
    dtf_GInfo: PGadgetInfo;
    dtf_ContentsInfo: PFrameInfo;
    dtf_FrameInfo: PFrameInfo;    // Input
    dtf_SizeFrameInfo: LongWord;  // Output
    dtf_FrameFlags: LongWord;
  end;

  // DTM_GOTO
  PdtGoto = ^TdtGoto;
  TdtGoto = record
    MethodID: LongWord;
    dtg_GInfo: PGadgetInfo;
    dtg_NodeName: STRPTR;   // Node to goto
    dtg_AttrList: PTagItem; // Additional attributes
  end;

  //* DTM_TRIGGER */
  PdtTrigger = ^TdtTrigger;
  TdtTrigger = record
    MethodID: LongWord;
    dtt_GInfo: PGadgetInfo;
    dtt_Function: LongWord;
    dtt_Data: APTR;
  end;

const
  STM_PAUSE       = 1;
  STM_PLAY        = 2;
  STM_CONTENTS    = 3;
  STM_INDEX       = 4;
  STM_RETRACE     = 5;
  STM_BROWSE_PREV = 6;
  STM_BROWSE_NEXT = 7;

  STM_NEXT_FIELD     =  8;
  STM_PREV_FIELD     =  9;
  STM_ACTIVATE_FIELD = 10;

  STM_COMMAND = 11;

  // New for V40
  STM_REWIND      = 12;
  STM_FASTFORWARD = 13;
  STM_STOP        = 14;
  STM_RESUME      = 15;
  STM_LOCATE      = 16;

type
  // Printer IO request
  PPrinterIO = ^TPrinterIO;
  TPrinterIO = record
     Ios: TIOStdReq;
     Iodrp: TIODRPReq;
     Iopc: TIOPrtCmdReq;
   end;

  // DTM_PRINT
  PdtPrint = ^TdtPrint;
  TdtPrint = record
    MethodID: LongWord;
    dtp_GInfo: PGadgetInfo;
    dtp_PIO: PPrinterIO;
    dtp_AttrList: PTagItem;
  end;

  // DTM_DRAW
  PdtDraw = ^TdtDraw;
  TdtDraw = record
    MethodID: LongWord;
    dtd_RPort: PRastPort;
    dtd_Left: LongInt;
    dtd_Top: LongInt;
    dtd_Width: LongInt;
    dtd_Height: LongInt;
    dtd_TopHoriz: LongInt;
    dtd_TopVert: LongInt;
    dtd_AttrList: PTagItem; // Additional attributes
  end;

  // DTM_WRITE
  PdtWrite = ^TdtWrite;
  TdtWrite = record
    MethodID: LongWord;
    dtw_GInfo: PGadgetInfo; // Gadget information
    dtw_FileHandle: BPTR;   // File handle to write to
    dtw_Mode: LongWord;
    dtw_AttrList: PTagItem; // Additional attributes
  end;

const
  DTWM_IFF = 0; // Save data as IFF data
  DTWM_RAW = 1; // Save data as local data format

//  pictureclass
const
  PICTUREDTCLASS = 'picture.datatype';

  // Picture attributes
  PDTA_ModeID           = DTA_Dummy + 200; // Mode ID of the picture
  PDTA_BitMapHeader     = DTA_Dummy + 201;
  PDTA_BitMap           = DTA_Dummy + 202; // (PBitmap) Pointer to a class-allocated bitmap, that will end up being freed by picture.class when DisposeDTObject() is called
  PDTA_ColorRegisters   = DTA_Dummy + 203; // (PColorRegister) Picture colour table
  PDTA_CRegs            = DTA_Dummy + 204; // (PLongWord) Color table to use with SetRGB32CM()
  PDTA_GRegs            = DTA_Dummy + 205; // (PLongWord) Color table; this table is initialized during the layout process and will contain the colours the picture will use
                                           // after remapping. If no remapping takes place, these colours will match those in the PDTA_CRegs table.
  PDTA_ColorTable       = DTA_Dummy + 206; // (PByte) Shared pen table; this table is initialized during the layout process while the picture is being remapped
  PDTA_ColorTable2      = DTA_Dummy + 207; // (PByte) Shared pen table; in most places this table will be identical to the PDTA_ColorTable table. Some of the colours in this table might
                                           // match the original colour palette a little better than the colours picked for the other table. The picture.datatype uses the two tables
                                           // during remapping, alternating for each pixel
  PDTA_Allocated        = DTA_Dummy + 208; // OBSOLETE; DO NOT USE
  PDTA_NumColors        = DTA_Dummy + 209; // (Word) Number of colors used by the picture.
  PDTA_NumAlloc         = DTA_Dummy + 210; // (Word) Number of colors allocated by the picture
  PDTA_Remap            = DTA_Dummy + 211; // (Boolean) Remap picture (defaults to True)
  PDTA_Screen           = DTA_Dummy + 212; // (PScreen) Screen to remap to
  PDTA_FreeSourceBitMap = DTA_Dummy + 213; // (Boolean) Free the source bitmap after remapping
  PDTA_Grab             = DTA_Dummy + 214; // Pointer to a Point structure
  PDTA_DestBitMap       = DTA_Dummy + 215; // Pointer to the destination (remapped) bitmap
  PDTA_ClassBitMap      = DTA_Dummy + 216; // Pointer to class-allocated bitmap, that will end up being freed by the class after DisposeDTObject() is called
  PDTA_NumSparse        = DTA_Dummy + 217; // (Word) Number of colors used for sparse remapping
  PDTA_SparseTable      = DTA_Dummy + 218; // (PByte) Pointer to a table of pen numbers indicating which colors should be used when remapping the image.
                                           //  This array must contain as many entries as there are colors specified with PDTA_NumSparse
  // V44
  PDTA_WhichPicture     = DTA_Dummy + 219; // (LongWord) Index number of the picture to load
  PDTA_GetNumPictures   = DTA_Dummy + 220; // (PLongWord) Get the number of pictures stored in the file
  PDTA_MaxDitherPens    = DTA_Dummy + 221; // (LongWord) Maximum number of colours to use for dithering
  PDTA_DitherQuality    = DTA_Dummy + 222; // (LongWord) Quality of the dithering algorithm to be used during colour quantization
  PDTA_AllocatedPens    = DTA_Dummy + 223; // (PByte) Pointer to the allocated pen table
  // V45
  PDTA_ScaleQuality     = DTA_Dummy + 224; // Quality for scaling.


  PDTANUMPICTURES_Unknown = 0; // When querying the number of pictures stored in a file, the following value denotes "the number of pictures is unknown".

  // V43 extensions (attributes)
  PDTA_SourceMode       = DTA_Dummy + 250; // Set the interface mode for the sub datatype. See below.
  PDTA_DestMode         = DTA_Dummy + 251; // Set the interface mode for the app datatype. See below.
  PDTA_UseFriendBitMap  = DTA_Dummy + 255; // Make the allocated bitmap be a "friend" bitmap (LongBool)
  PDTA_MaskPlane        = DTA_Dummy + 258; // (PlantPtr) nil or mask plane for use with BltMaskBitMapRastPort()

  // Interface modes
  PMODE_V42 = 0; // Compatibility mode
  PMODE_V43 = 1; // Extended mode

  // V43 extensions (methods)
  PDTM_Dummy = DTM_Dummy + $60;

  PDTM_WRITEPIXELARRAY = PDTM_Dummy + 0; // Transfer pixel data to the picture object in the specified format
  PDTM_READPIXELARRAY  = PDTM_Dummy + 1; // Transfer pixel data from the picture object in the specified format

  //  Masking techniques
  mskNone                = 0;
  mskHasMask             = 1;
  mskHasTransparentColor = 2;
  mskLasso               = 3;
  mskHasAlpha            = 4;

  //  Compression techniques
  cmpNone     = 0;
  cmpByteRun1 = 1;
  cmpByteRun2 = 2;

type
  // Bitmap header (BMHD) structure
  PBitMapHeader = ^TBitMapHeader;
  TBitMapHeader = record
    bmh_Width: Word;        // Width in pixels
    bmh_Height: Word;       // Height in pixels
    bmh_Left: SmallInt;     // Left position
    bmh_Top: SmallInt;      // Top position
    bmh_Depth: Byte;        // Number of planes
    bmh_Masking: Byte;      // Masking type
    bmh_Compression: Byte;  // Compression type
    bmh_Pad: Byte;
    bmh_Transparent: Word; // Transparent color
    bmh_XAspect: Byte;
    bmh_YAspect: Byte;
    bmh_PageWidth: SmallInt;
    bmh_PageHeight: SmallInt;
  end;

  PColorRegister = ^TColorRegister;
  TColorRegister = record
    red: Byte;
    green: Byte;
    blue: Byte;
  end;

const
  // IFF types that may be in pictures
  ID_ILBM = Ord('I') shl 24 + Ord('L') shl 16 + Ord('B') shl 8 + Ord('M'); // ILBM
  ID_BMHD = Ord('B') shl 24 + Ord('M') shl 16 + Ord('H') shl 8 + Ord('D'); // BMHD
  ID_CMAP = Ord('C') shl 24 + Ord('M') shl 16 + Ord('A') shl 8 + Ord('P'); // CMAP
  ID_CRNG = Ord('C') shl 24 + Ord('R') shl 16 + Ord('N') shl 8 + Ord('G'); // CRNG
  ID_GRAB = Ord('G') shl 24 + Ord('R') shl 16 + Ord('A') shl 8 + Ord('B'); // GRAB
  ID_SPRT = Ord('S') shl 24 + Ord('P') shl 16 + Ord('R') shl 8 + Ord('T'); // SPRT
  ID_DEST = Ord('D') shl 24 + Ord('E') shl 16 + Ord('S') shl 8 + Ord('T'); // DEST
  ID_CAMG = Ord('C') shl 24 + Ord('A') shl 16 + Ord('M') shl 8 + Ord('G'); // CAMG

  ID_BODY = Ord('B') shl 24 + Ord('O') shl 16 + Ord('D') shl 8 + Ord('Y'); // BODY

// soundclass
const
  SOUNDDTCLASS = 'sound.datatype';
  // Sound attributes
  SDTA_Dummy         = DTA_Dummy + 500;
  SDTA_VoiceHeader   = SDTA_Dummy + 1;
  SDTA_Sample        = SDTA_Dummy + 2; // (PByte) Sample data
  SDTA_SampleLength  = SDTA_Dummy + 3; // (LongWord) Length of the sample data in Bytes
  SDTA_Period        = SDTA_Dummy + 4; // (Word) Period
  SDTA_Volume        = SDTA_Dummy + 5; // (Word) Volume.  Range from 0 to 64
  SDTA_Cycles        = SDTA_Dummy + 6;
  // The following tags are new for V40
  SDTA_SignalTask    = SDTA_Dummy + 7; // (PTask) Task to signal when sound is complete or next buffer needed.
  SDTA_SignalBit     = SDTA_Dummy + 8; // (LongWord) Signal mask to use on completion or 0 to disable
  SDTA_SignalBitMask = SDTA_SignalBit; // (Byte) Signal bit to use on completion or -1 to disable
  SDTA_Continuous    = SDTA_Dummy + 9; // (Byte) Playing a continuous stream of data.  Defaults to False.

  // The following tags are new for V44
  SDTA_SignalBitNumber  = SDTA_Dummy + 10; // (Byte) Signal bit to use on completion or -1 to disable
  SDTA_SamplesPerSec    = SDTA_Dummy + 11; // (Word) Samples per second
  SDTA_ReplayPeriod     = SDTA_Dummy + 12; // (PTimeVal) Sample replay period
  SDTA_LeftSample       = SDTA_Dummy + 13; // (PByte) Sample data
  SDTA_RightSample      = SDTA_Dummy + 14; // (PByte) Sample data
  SDTA_Pan              = SDTA_Dummy + 15; // (PByte) Stereo panning
  SDTA_FreeSampleData   = SDTA_Dummy + 16; // (Bool) FreeVec() all sample data upon OM_DISPOSE.
  SDTA_SyncSampleChange = SDTA_Dummy + 17; // (Boot) Wait for the current sample to be played back before switching to the new sample data.

  // Data compression methods
  CMP_NONE     = 0;
  CMP_FIBDELTA = 1;

  // Unity = Fixed 1.0 = maximum volume
  Unity = $10000;

type
  PVoiceHeader = ^TVoiceHeader;
  TVoiceHeader = record
    vh_OneShotHiSamples: LongWord;  // # samples in the high octave 1-shot part
    vh_RepeatHiSamples: LongWord;   // # samples in the high octave repeat part
    vh_SamplesPerHiCycle: LongWord; // # samples/cycle in high octave, else 0
    vh_SamplesPerSec: Word;         // data sampling rate
    vh_Octaves: Byte;               // # of octaves of waveforms
    vh_Compression: Byte;           // data compression technique used
    vh_Volume: LongWord;            // playback nominal volume from 0 to Unity (full volume). Map this value into the output hardware's dynamic range.
  end;

const
  // Channel allocation */
  SAMPLETYPE_Left   = 2;
  SAMPLETYPE_Right  = 4;
  SAMPLETYPE_Stereo = 6;

type
  TSampleType = LongInt;

const
  // IFF types
  ID_8SVX = Ord('8') shl 24 + Ord('S') shl 16 + Ord('V') shl 8 + Ord('X'); // 8SVX
  ID_VHDR = Ord('V') shl 24 + Ord('H') shl 16 + Ord('D') shl 8 + Ord('R'); // VHDR
  ID_CHAN = Ord('C') shl 24 + Ord('H') shl 16 + Ord('A') shl 8 + Ord('N'); // CHAN

// textclass
const
  TEXTDTCLASS = 'text.datatype';

  // attributes
  TDTA_Buffer     = DTA_Dummy + 300;
  TDTA_BufferLen  = DTA_Dummy + 301;
  TDTA_LineList   = DTA_Dummy + 302;
  TDTA_WordSelect = DTA_Dummy + 303;
  TDTA_WordDelim  = DTA_Dummy + 304;
  TDTA_WordWrap   = DTA_Dummy + 305; // Boolean. Should the text be word wrapped. Defaults to false.

Type
  // There is one line structure for every line of text in the document.
  PLine = ^TLine;
  TLine = record
    ln_Link: TMinNode;    // to link the lines together
    ln_Text: STRPTR;      // pointer to the text for this line
    ln_TextLen: LongWord; // the character length of the text for this line
    ln_XOffset: Word;     // where in the line the text starts
    ln_YOffset: Word;     // line the text is on
    ln_Width: Word;       // Width of line in pixels
    ln_Height: Word;      // Height of line in pixels
    ln_Flags: Word;       // info on the line LNF_*
    ln_FgPen: ShortInt;   // foreground pen
    ln_BgPen: ShortInt;   // background pen
    ln_Style: LongWord;   // Font style
    ln_Data: APTR;        // Link data...
  end;
const
  // ln_Flags
  LNF_LF        = 1 shl 0; // Line Feed
  LNF_LINK      = 1 shl 1; // Segment is a link
  LNF_OBJECT    = 1 shl 2; // ln_Data is a pointer to an DataTypes object
  LNF_SELECTED  = 1 shl 3; // Object is selected

  ID_FTXT = Ord('F') shl 24 + Ord('T') shl 16 + Ord('X') shl 8 + Ord('T'); // FTXT
  ID_CHRS = Ord('C') shl 24 + Ord('H') shl 16 + Ord('R') shl 8 + Ord('S'); // CHRS

// animationclass
const
  ANIMATIONDTCLASS = 'animation.datatype';

  // Animation attributes
  ADTA_Dummy           = DTA_Dummy + 600;
  ADTA_ModeID          = PDTA_ModeID;
  ADTA_KeyFrame        = PDTA_BitMap;         // (PBitmap) Key frame (first frame) bitmap
  ADTA_ColorRegisters  = PDTA_ColorRegisters;
  ADTA_CRegs           = PDTA_CRegs;
  ADTA_GRegs           = PDTA_GRegs;
  ADTA_ColorTable      = PDTA_ColorTable;
  ADTA_ColorTable2     = PDTA_ColorTable2;
  ADTA_Allocated       = PDTA_Allocated;
  ADTA_NumColors       = PDTA_NumColors;
  ADTA_NumAlloc        = PDTA_NumAlloc;
  ADTA_Remap           = PDTA_Remap;        // (Bool) Remap animation (defaults to True)
  ADTA_Screen          = PDTA_Screen;       // (PScreen) Screen to remap to
  ADTA_NumSparse       = PDTA_NumSparse;    // (Word) Number of colors used for sparse remapping
  ADTA_SparseTable     = PDTA_SparseTable;  // (PByte) Pointer to a table of pen numbers indicating which colors should be used when remapping the image.
                                            // This array must contain as many entries as there are colors specified with ADTA_NumSparse
  ADTA_Width           = ADTA_Dummy + 1;
  ADTA_Height          = ADTA_Dummy + 2;
  ADTA_Depth           = ADTA_Dummy + 3;
  ADTA_Frames          = ADTA_Dummy + 4;   // (LongWord) Number of frames in the animation
  ADTA_Frame           = ADTA_Dummy + 5;   // (LongWord)  Current frame
  ADTA_FramesPerSecond = ADTA_Dummy + 6;   // (LongWord) Frames per second
  ADTA_FrameIncrement  = ADTA_Dummy + 7;   // (LongInt) Amount to change frame by when fast forwarding or rewinding.  Defaults to 10.

  ADTA_PreloadFrameCount = ADTA_Dummy + 8;   // (V44)

  // Sound attributes
  ADTA_Sample          = SDTA_Sample;
  ADTA_SampleLength    = SDTA_SampleLength;
  ADTA_Period          = SDTA_Period;
  ADTA_Volume          = SDTA_Volume;
  ADTA_Cycles          = SDTA_Cycles;
  // V44
  ADTA_LeftSample    = SDTA_LeftSample;
  ADTA_RightSample   = SDTA_RightSample;
  ADTA_SamplesPerSec = SDTA_SamplesPerSec;

  // IFF ANIM chunks
  ID_ANIM = Ord('A') shl 24 + Ord('N') shl 16 + Ord('I') shl 8 + Ord('M'); // ANIM
  ID_ANHD = Ord('A') shl 24 + Ord('N') shl 16 + Ord('H') shl 8 + Ord('D'); // ANHD
  ID_DLTA = Ord('D') shl 24 + Ord('L') shl 16 + Ord('T') shl 8 + Ord('A'); // DLTA

type
  // Required ANHD structure describes an ANIM frame
  PAnimHeader = ^TAnimHeader;
  TAnimHeader = record
    ah_Operation: Byte; { The compression method:
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
    ah_Mask: Byte;      // (XOR mode only - plane mask where each bit is set =1 if there is data and =0 if not.)
    ah_Width: Word;    // (XOR mode only - width and height of the area represented
    ah_Height: Word;     //   by the BODY to eliminate unnecessary un-changed data)
    ah_Left: SmallInt;  // (XOR mode only - position of rectangular
    ah_Top: SmallInt;   // area representd by the BODY)
    ah_AbsTime: LongWord;  // Timing for a frame relative to the time the first frame was displayed, in jiffies (1/60 sec)
    ah_RelTime: LongWord;  // Timing for frame relative to time previous frame was displayed - in jiffies (1/60 sec)
    ah_Interleave: Byte;   { Indicates how may frames back this data is to modify.  0 defaults to indicate two frames back
                             (for double buffering). n indicates n frames back. The main intent here is to allow values
                             of 1 for special applications where frame data would modify the immediately previous frame. }
    ah_Pad0: Byte;         // Pad byte, not used at present.
    ah_Flags: LongWord;    { 32 option bits used by options=4 and 5. At present only 6 are identified, but the
                             rest are set =0 so they can be used to implement future ideas.  These are defined
                             for option 4 only at this point.  It is recommended that all bits be set =0 for
                             option 5 and that any bit settings used in the future (such as for XOR mode)
                             be compatible with the option 4 bit settings. Player code should check undefined bits
                             in options 4 and 5 to assure they are zero.
                             The six bits for current use are:
                              bit #       set =0                  set =1
                              ===============================================
                              0           short data              long data
                              1           set                     XOR
                              2           separate info           one info list
                                          for each plane          for all planes
                              3           not RLC                 RLC (run length coded)
                              4           horizontal              vertical
                              5           short info offsets      long info offsets}
    ah_Pad: array[0..15] of Byte; // This is a pad for future use for future compression modes
  end;
const
  // Methods
  ADTM_Dummy       = $700;
  ADTM_LOADFRAME   = $701; // Used to load a frame of the animation
  ADTM_UNLOADFRAME = $702; // Used to unload a frame of the animation
  ADTM_START       = $703; // Used to start the animation
  ADTM_PAUSE       = $704; // Used to pause the animation (don't reset the timer)
  ADTM_STOP        = $705; // Used to stop the animation
  ADTM_LOCATE      = $706; // Used to locate a frame in the animation (as set by a slider...)
  // New on V44
  ADTM_LOADNEWFORMATFRAME   = $707; // Used to load a new format frame of the animation
  ADTM_UNLOADNEWFORMATFRAME = $708; // Used to unload a new format frame of the animation

type
  // ADTM_LOADFRAME, ADTM_UNLOADFRAME
  PadtFrame = ^TadtFrame;
  TadtFrame = record
    MethodID: LongWord;
    alf_TimeStamp: LongWord;  // Timestamp of frame to load
    // The following fields are filled in by the ADTM_LOADFRAME method, and are read-only for any other methods.
    alf_Frame: LongWord;      // Frame number
    alf_Duration: LongWord;   // Duration of frame
    alf_BitMap: PBitMap;      // Loaded BitMap
    alf_CMap: PColorMap;      // Colormap, if changed
    alf_Sample: PShortInt;    // Sound data
    alf_SampleLength: LongWord;
    alf_Period: LongWord;
    alf_UserData: APTR;       // Used by load frame for extra data
  end;

  // ADTM_LOADFRAME, ADTM_UNLOADFRAME
  PadtNewFormatFrame = ^TadtNewFormatFrame;
  TadtNewFormatFrame = record
    MethodID: LongWord;
    alf_TimeStamp: LongWord;     // Timestamp of frame to load
    // // The following fields are filled in by the ADTM_NEWLOADFRAME method, and are read-only for any other methods.
    alf_Frame: LongWord;         // Frame number
    alf_Duration: LongWord;      // Duration of frame
    alf_BitMap: PBitMap;         // Loaded BitMap
    alf_CMap: PColorMap;         // Colormap, if changed
    alf_Sample: PShortInt;
    alf_SampleLength: LongWord;  // Sound data
    alf_Period: LongWord;
    alf_UserData: APTR;          // Used by load frame for extra data
    alf_Size: LongWord;          // Size of this data structure (in bytes)
    alf_LeftSample: PShortInt;   // Sound for left channel, or nil if none
    alf_RightSample: PShortInt;  // Sound for right channel, or nil if none
    alf_SamplesPerSec: LongWord; // Replay speed; if > 0, this overrides alf_Period
  end;

  // ADTM_START, ADTM_PAUSE, ADTM_STOP, ADTM_LOCATE
  PadtStart = ^tadtStart;
  TadtStart = record
    MethodID: LongWord;
    asa_Frame: LongWord; // Frame # to start at
  end;



var
  DataTypesBase: PLibrary = nil;

const
  DATATYPESNAME: PChar = 'datatypes.library';

function AddDTObject(Win: PWindow location 'a0'; Req: PRequester location 'a1'; o: PObject_ location 'a2'; Pos: LongInt location 'd0'): LongInt; syscall DataTypesBase 072;
procedure DisposeDTObject(o: PObject_ location 'a0'); syscall DataTypesBase 054;
function DoAsyncLayout(o: PObject_ location 'a0'; gpl: pgpLayout location 'a1'): LongWord; syscall DataTypesBase 084;
function DoDTMethodA(o: PObject_ location 'a0'; Win: PWindow location 'a1'; Req: PRequester location 'a2'; Msg: PLongInt location 'a3'): LongWord; syscall DataTypesBase 090;
function GetDTAttrsA(o: PObject_ location 'a0'; Attrs: PTagItem location 'a2'): LongWord; syscall DataTypesBase 066;
function GetDTMethods(Obj: PObject_ location 'a0'): Pointer; syscall DataTypesBase 102;
function GetDTString(Id: LongWord location 'a0'): STRPTR; syscall DataTypesBase 138;
function GetDTTriggerMethods(Obj: PObject_ location 'a0'): PDTMethod; syscall DataTypesBase 108;
function NewDTObjectA(Name: STRPTR location 'd0'; Attrs: pTagItem location 'a0'): Pointer; syscall DataTypesBase 048;
function ObtainDataTypeA(Typ: LongWord location 'd0'; Handle: Pointer location 'a0'; Attrs: PTagItem location 'a1'): pDataType; syscall DataTypesBase 036;
function PrintDTObjectA(o: PObject_ location 'a0'; w: PWindow location 'a1'; r: PRequester location 'a2'; Msg: PdtPrint location 'a3'): LongWord; syscall DataTypesBase 114;
procedure RefreshDTObjectA(o: PObject_ location 'a0'; Win: PWindow location 'a1'; Req: PRequester location 'a2'; Attrs: PTagItem location 'a3'); syscall DataTypesBase 078;
procedure ReleaseDataType(Dt: PDataType location 'a0'); syscall DataTypesBase 042;
function RemoveDTObject(Win: PWindow location 'a0'; o: PObject_ location 'a1'): LongInt; syscall DataTypesBase 096;
function SetDTAttrsA(o: PObject_ location 'a0'; Win: PWindow location 'a1'; Req: PRequester location 'a2'; Attrs: PTagItem location 'a3'): LongWord; syscall DataTypesBase 060;

function ObtainDTDrawInfoA(o: PObject_ location 'a0'; Attrs: PTagItem location 'a1'): Pointer; syscall DataTypesBase 120;
function DrawDTObjectA(Rp: PRastPort location 'a0';  o: PObject_ location 'a1'; x: LongInt location 'd0'; y: LongInt location 'd1'; w: LongInt location 'd2'; h: LongInt location 'd3'; th: LongInt location 'd4'; tv: LongInt location 'd5'; Attrs: PTagItem location 'a2'): LongInt; syscall DataTypesBase 126;
procedure ReleaseDTDrawInfo(o: PObject_ location 'a0'; Handle: Pointer location 'a1'); syscall DataTypesBase 132;

function ObtainDataType(Typ: LongWord; Handle: APTR; const TagList: array of PtrUInt): PDataType; inline;
function NewDTObject(Name: APTR; const TagList: array of PtrUInt): PObject_; inline;
function SetDTAttrs(O: PObject_; Win: PWindow; Req: PRequester; const TagList: array of PtrUInt): LongWord; inline;
function GetDTAttrs(O: PObject_; const TagList: array of PtrUInt): LongWord; inline;
procedure RefreshDTObject(Obj: PObject_; Window: PWindow; Req: PRequester; const TagList: array of PtrUInt); inline;
function DoDTMethod(O: PObject_; Win: PWindow; Req: PRequester; const ArgList: array of PtrUInt): PtrUInt; inline;
function PrintDTObject(Obj: PObject_; Window: PWindow; Requester: PRequester; const ArgList: array of PtrUInt): LongWord; inline;
function ObtainDTDrawInfo(O: PObject_; const TagList: array of PtrUInt): APTR; inline;
function DrawDTObject(Rp: PRastPort; O: PObject_; x, y, w, h, th, tv: LongInt; const TagList: array of PtrUInt): LongInt; inline;

implementation

function ObtainDataType(Typ: LongWord; Handle: APTR; const TagList: array of PtrUInt): PDataType;
begin
  ObtainDataType := ObtainDataTypeA(Typ, Handle, @TagList);
end;

function NewDTObject(Name: APTR; const TagList: array of PtrUInt): PObject_;
begin
  NewDTObject := NewDTObjectA(Name, @TagList);
end;

function SetDTAttrs(O: PObject_; Win: PWindow; Req: PRequester; const TagList: array of PtrUInt): LongWord;
begin
  SetDTAttrs := SetDTAttrsA(O, Win, Req, @TagList);
end;

function GetDTAttrs(O: PObject_; const TagList: array of PtrUInt): LongWord;
begin
  GetDTAttrs := GetDTAttrsA(O, @TagList);
end;

procedure RefreshDTObject(Obj: PObject_; Window: PWindow; Req: PRequester; const TagList: array of PtrUInt);
begin
  RefreshDTObjectA(Obj, Window, Req, @TagList);
end;

function DoDTMethod(O: PObject_; Win: PWindow; Req: PRequester; const ArgList: array of PtrUInt): PtrUInt;
begin
  DoDTMethod := DoDTMethodA(O, Win, Req, @ArgList);
end;

function PrintDTObject(Obj: PObject_; Window: PWindow; Requester: PRequester; const ArgList: array of PtrUInt): LongWord;
begin
  PrintDTObject := PrintDTObjectA(Obj, Window, Requester, @ArgList);
end;

function ObtainDTDrawInfo(O: PObject_; const TagList: array of PtrUInt): APTR;
begin
  ObtainDTDrawInfo := ObtainDTDrawInfoA(O, @TagList);
end;

function DrawDTObject(Rp: PRastPort; O: PObject_; x, y, w, h, th, tv: LongInt; const TagList: array of PtrUInt): LongInt;
begin
  DrawDTObject := DrawDTObjectA(RP, O, x, y, w, h, th, tv, @TagList);
end;


const
  LIBVERSION : longword = 0;

initialization
  DataTypesBase := OpenLibrary(DATATYPESNAME,LIBVERSION);
finalization
  if Assigned(DataTypesBase) then
    CloseLibrary(DataTypesBase);
END. (* UNIT DATATYPES *)





