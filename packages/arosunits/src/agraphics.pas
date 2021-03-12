{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Free Pascal development team

    graphics.library functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit agraphics;

interface

uses
  Exec, Hardware, Utility;

{$PACKRECORDS C}
const
  BITSET = $8000;
  BITCLR = 0;

type
  TPlanePtr = PByte;

  PPoint = ^TPoint;
  TPoint = record
    x, y: SmallInt;
  end;

  PBitMap = ^TBitMap;
  TBitMap = record
    BytesPerRow: Word;
    Rows: Word;
    Flags: Byte;
    Depth: Byte;
    Pad: Word;
    Planes: array[0..7] of TPlanePtr;
  end;

  PRectangle = ^TRectangle;
  TRectangle = record
    MinX,
    MinY: SmallInt;
    MaxX,
    MaxY: SmallInt;
  end;

  PRect32 = ^TRect32;
  TRect32 = record
    MinX,
    MinY: Longint;
    MaxX,
    MaxY: Longint;
  end;

type
// a structure to contain the 16 collision procedure addresses }
  PCollTable = ^TCollTable;
  TCollTable = array[0..15] of Pointer;

// flags for AllocBitMap, etc.
const
  BMB_CLEAR       = 0;
  BMB_DISPLAYABLE = 1;
  BMB_INTERLEAVED = 2;
  BMB_STANDARD    = 3;
  BMB_MINPLANES   = 4;
  BMB_SPECIALFMT  = 7; // CyberGfx Flag

  BMF_CLEAR       = 1 shl BMB_CLEAR;
  BMF_DISPLAYABLE = 1 shl BMB_DISPLAYABLE;
  BMF_INTERLEAVED = 1 shl BMB_INTERLEAVED;
  BMF_STANDARD    = 1 shl BMB_STANDARD;
  BMF_MINPLANES   = 1 shl BMB_MINPLANES;
  BMF_SPECIALFMT  = 1 shl BMB_SPECIALFMT;

  BMB_PIXFMT_SHIFTUP =  24;

  BMF_REQUESTVMEM = BMF_DISPLAYABLE or BMF_MINPLANES;
// AmigaOS v4 flags
  BMB_HIJACKED    = 7;
  BMF_HIJACKED    = 1 shl 7;
  BMB_RTGTAGS     = 8;
  BMF_RTGTAGS     = 1 shl 8;
  BMB_RTGCHECK    = 9;
  BMF_RTGCHECK    = 1 shl 9;
  BMB_FRIENDISTAG = 10;
  BMF_FRIENDISTAG = 1 shl 10;
  BMB_INVALID     = 11;
  BMF_INVALID     = 1 shl 11;

  BMF_CHECKVALUE  = BMF_RTGTAGS or BMF_RTGCHECK or BMF_FRIENDISTAG;
  BMF_CHECKMASK   = BMF_HIJACKED or BMF_CHECKVALUE or BMF_INVALID;

// tags for AllocBitMap */
  BMATags_Friend              = TAG_USER + 0;
  BMATags_Depth               = TAG_USER + 1;
  BMATags_RGBFormat           = TAG_USER + 2;
  BMATags_Clear               = TAG_USER + 3;
  BMATags_Displayable         = TAG_USER + 4;
  BMATags_Private1            = TAG_USER + 5;
  BMATags_NoMemory            = TAG_USER + 6;
  BMATags_NoSprite            = TAG_USER + 7;
  BMATags_Private2            = TAG_USER + 8;
  BMATags_Private3            = TAG_USER + 9;
  BMATags_ModeWidth           = TAG_USER + 10;
  BMATags_ModeHeight          = TAG_USER + 11;
  BMATags_RenderFunc          = TAG_USER + 12;
  BMATags_SaveFunc            = TAG_USER + 13;
  BMATags_UserData            = TAG_USER + 14;
  BMATags_Alignment           = TAG_USER + 15;
  BMATags_ConstantBytesPerRow = TAG_USER + 16;
  BMATags_UserPrivate         = TAG_USER + 17;
  BMATags_Private4            = TAG_USER + 18;
  BMATags_Private5            = TAG_USER + 19;
  BMATags_Private6            = TAG_USER + 20;
  BMATags_Private7            = TAG_USER + 21;
  BMATags_BitmapColors        = TAG_USER + $29;
  BMATags_DisplayID           = TAG_USER + $32;
  BMATags_BitmapInvisible     = TAG_USER + $37;
  BMATags_BitmapColors32      = TAG_USER + $43;
// the following IDs are for GetBitMapAttr() *}
  BMA_HEIGHT = 0;
  BMA_DEPTH  = 4;
  BMA_WIDTH  = 8;
  BMA_FLAGS  = 12;

type
  PRegionRectangle = ^TRegionRectangle;
  TRegionRectangle = record
    Next, Prev: PRegionRectangle;
    Bounds: TRectangle;
  end;

  PRegion = ^TRegion;
  TRegion = record
    Bounds: TRectangle;
    RegionRectangle: PRegionRectangle;
  end;

const

// internal TClipRect flags
  CR_NEEDS_NO_CONCEALED_RASTERS       = 1;
  CR_NEEDS_NO_LAYERBLIT_DAMAGE        = 2;

// defines for code values for getcode
  ISLESSX     = 1 shl 0;
  ISLESSY     = 1 shl 1;
  ISGRTRX     = 1 shl 2;
  ISGRTRY     = 1 shl 3;

type
// TextAttr node, matches text attributes in RastPort
  PTextAttr = ^TTextAttr;
  TTextAttr = record
    ta_Name: STRPTR;  // name of the font
    ta_YSize: Word;   // height of the font
    ta_Style: Byte;   // intrinsic font style
    ta_Flags: Byte;   // font preferences and flags
  end;
// like TextAttr + Tags
  PTTextAttr = ^TTTextAttr;
  TTTextAttr = record
    tta_Name: STRPTR;   // name of the font
    tta_YSize: Word;    // height of the font
    tta_Style: Byte;    // intrinsic font style
    tta_Flags: Byte;    // font preferences and flags
    tta_Tags: pTagItem; // TTextAttr specific extension -> extended attributes
  end;

const
// ta_Style/tta_Style
  FS_NORMAL           = 0;       // normal text (no style bits set)
  FSB_UNDERLINED      = 0;       // underlined (under baseline)
  FSF_UNDERLINED      = 1 shl 0;
  FSB_BOLD            = 1;       // bold face text (ORed w/ shifted)
  FSF_BOLD            = 1 shl 1;
  FSB_ITALIC          = 2;       // italic (slanted 1:2 right)
  FSF_ITALIC          = 1 shl 2;
  FSB_EXTENDED        = 3;       // extended face (wider than normal)
  FSF_EXTENDED        = 1 shl 3;
  FSB_COLORFONT       = 6;       // this uses ColorTextFont structure
  FSF_COLORFONT       = 1 shl 6;
  FSB_TAGGED          = 7;       // the TextAttr is really an TTextAttr
  FSF_TAGGED          = 1 shl 7;
// ta_Flags/tta_Flags
  FPB_ROMFONT         = 0;      // font is in rom
  FPF_ROMFONT         = 1 shl 0;
  FPB_DISKFONT        = 1;      // font is from diskfont.library
  FPF_DISKFONT        = 1 shl 1;
  FPB_REVPATH         = 2;      // designed path is reversed (e.g. left)
  FPF_REVPATH         = 1 shl 2;
  FPB_TALLDOT         = 3;      // designed for hires non-interlaced
  FPF_TALLDOT         = 1 shl 3;
  FPB_WIDEDOT         = 4;      // designed for lores interlaced
  FPF_WIDEDOT         = 1 shl 4;
  FPB_PROPORTIONAL    = 5;      //character sizes can vary from nominal
  FPF_PROPORTIONAL    = 1 shl 5;
  FPB_DESIGNED        = 6;     // size is "designed", not constructed
  FPF_DESIGNED        = 1 shl 6;
  FPB_REMOVED         = 7;     // the font has been removed
  FPF_REMOVED         = 1 shl 7;
// tta_Tags
  TA_DeviceDPI        =  TAG_USER + 1; // Tag value is Point union: Hi Longint XDPI, Lo Longint YDPI

  MAXFONTMATCHWEIGHT  =  32767;   { perfect match from WeighTAMatch }

type
  PTextFont = ^TTextFont;
  TTextFont = record
    tf_Message: TMessage; // reply message for font removal
                          // font name in LN \    used in this
    tf_YSize: Word;       // font height     |    order to best
    tf_Style: Byte;       // font style      |    match a font
    tf_Flags: Byte;       // preferences and flags /    request.
    tf_XSize: Word;       // nominal font width
    tf_Baseline: Word;    // distance from the top of char to baseline
    tf_BoldSmear: Word;   // smear to affect a bold enhancement
    tf_Accessors: Word;   // access count
    tf_LoChar: Byte;      // the first character described here
    tf_HiChar: Byte;      // the last character described here
    tf_CharData: APTR;    // the bit character data
    tf_Modulo: Word;      // the row modulo for the strike font data
    tf_CharLoc: APTR;     // ptr to location data for the strike font 2 words: bit offset then size
    tf_CharSpace: APTR;   // ptr to words of proportional spacing data
    tf_CharKern: APTR;    // ptr to words of kerning data
    //property tf_extension: PMsgPort read tf_Message.mn_ReplyPort write tf_Message.mn_ReplyPort;
  end;

  PTextFontExtension = ^TTextFontExtension;
  TTextFontExtension = record    // this structure is read-only
    tfe_MatchWord: Word;         // a magic cookie for the extension
    tfe_Flags0: Byte;            // (system private flags)
    tfe_Flags1: Byte;            // (system private flags)

    tfe_BackPtr: PTextFont;      // validation of compilation
    tfe_OrigReplyPort: PMsgPort; // original value in tf_Extension
    tfe_Tags: PTagItem;          // Text Tags for the font

    tfe_OFontPatchS,             // (system private use)
    tfe_OFontPatchK: PWord;      // (system private use)
    // this space is reserved for future expansion
  end;

  PColorFontColors = ^TColorFontColors;
  TColorFontColors = record
    cfc_Reserved,          // *must* be zero
    cfc_Count: Word;       // number of entries in cfc_ColorTable
    cfc_ColorTable: PWord; // 4 bit per component color map packed xRGB
  end;

  PColorTextFont = ^TColorTextFont;
  TColorTextFont = record
    ctf_TF: TTextFont;

    ctf_Flags: Word;      // extended flags
    ctf_Depth,            // number of bit planes
    ctf_FgColor,          // color that is remapped to FgPen
    ctf_Low,              // lowest color represented here }
    ctf_High,             // highest color represented here }
    ctf_PlanePick,        // PlanePick ala Images }
    ctf_PlaneOnOff: Byte; // PlaneOnOff ala Images

    ctf_ColorFontColors: PColorFontColors; // colors for font

    ctf_CharData: array[0..7] of APTR; // pointers to bit planes ala tf_CharData
  end;

  PTextExtent = ^TTextExtent;
  TTextExtent = record
    te_Width,               // same as TextLength
    te_Height: Word;        // same as tf_YSize
    te_Extent: TRectangle;  // relative to CP
  end;

const
// TColorTextFont.ctf_Flags
  CTB_MAPCOLOR  =  0;       // map ctf_FgColor to the rp_FgPen IF it's
  CTF_MAPCOLOR  =  1 shl 0; // is a valid color within ctf_Low..ctf_High
  CT_COLORFONT  =  1 shl 0; // color map contains designer's colors
  CT_GREYFONT   =  1 shl 1; // color map describes even-stepped brightnesses from low to high
  CT_ANTIALIAS  =  1 shl 2; // zero background thru fully saturated char
  CT_COLORMASK  =  $000F;   // mask to get to following color styles

// VSprite flags
  // user-set VSprite flags:
  VSPRITE      = $0001; // set if VSprite, clear if Bob
  SAVEBACK     = $0002; // set if background is to be saved/restored
  OVERLAY      = $0004; // set to mask image of Bob onto background
  MUSTDRAW     = $0008; // set if VSprite absolutely must be drawn
  SUSERFLAGS   = $00FF; // mask of all user-settable VSprite-flags
  // system-set VSprite flags:
  BACKSAVED    = $0100; // this Bob's background has been saved
  BOBUPDATE    = $0200; // temporary flag, useless to outside world
  GELGONE      = $0400; // set if gel is completely clipped (offscreen)
  VSOVERFLOW   = $0800; // VSprite overflow (if MUSTDRAW set we draw!)
// Bob flags
  // these are the user flag bits
  SAVEBOB      = $0001; // set to not erase Bob
  BOBISCOMP    = $0002; // set to identify Bob as AnimComp
  BUSERFLAGS   = $00FF; // mask of all user-settable Bob-flags
  // these are the system flag bits
  BWAITING     = $0100; // set while Bob is waiting on 'after'
  BDRAWN       = $0200; // set when Bob is drawn this DrawG pass
  BOBSAWAY     = $0400; // set to initiate removal of Bob
  BOBNIX       = $0800; // set when Bob is completely removed
  SAVEPRESERVE = $1000; // for back-restore during double-buffer
  OUTSTEP      = $2000; // for double-clearing if double-buffer
// defines for the animation procedures
  ANFRACSIZE   = 6;
  RINGTRIGGER  = $0001;
  ANIMHALF     = $0020;

type
{ UserStuff definitions
  the user can define these to be a single variable or a sub-structure
  if undefined by the user, the system turns these into innocuous variables
  see the manual for a thorough definition of the UserStuff definitions }
  TVUserStuff = SmallInt; // Sprite user stuff
  TBUserStuff = SmallInt; // Bob user stuff
  TAUserStuff = SmallInt; // AnimOb user stuff

  PBob = ^TBob;
  PAnimOb = ^TAnimOb;
  PAnimComp = ^TAnimComp;

{ GEL draw list constructed in the order the Bobs are actually drawn, then
  list is copied to clear list
  must be here in VSprite for system boundary detection
  the VSprite positions are defined in (y,x) order to make sorting
  sorting easier, since (y,x) as a long Longint}
  PVSprite = ^TVSprite;
  TVSprite = record
   // SYSTEM VARIABLES
    NextVSprite: PVSprite; // GEL linked list forward/backward pointers sorted by y,x value
    PrevVSprite: PVSprite;
    IntVSprite: PVSprite;    // pointer of overlay drawing
    ClearPath: PVSprite;   // pointer for overlay clearing
    OldY, OldX: SmallInt;  // previous position
   // COMMON VARIABLES
    Flags: SmallInt;       // VSprite flags
   // COMMON VARIABLES
    Y, X: SmallInt;        // screen position
    Height: SmallInt;
    Width: SmallInt;       // number of words per row of image data
    Depth: SmallInt;       // number of planes of data
    MeMask: SmallInt;      // which types can collide with this VSprite
    HitMask: SmallInt;     // which types this VSprite can collide with
    ImageData: PSmallInt;  // pointer to VSprite image
    BorderLine: PSmallInt; // logical OR of all VSprite bits borderLine is the one-dimensional logical OR of all
                           // the VSprite bits, used for fast collision detection of edge
    CollMask: PSmallInt;   // similar to above except this is a matrix
      //pointer to this VSprite's color definitions (not used by Bobs)
    SprColors: PSmallInt;
    VSBob: PBob;           // points home if this VSprite is part of a Bob
    PlanePick: Shortint;   { planePick flag:  set bit selects a plane from image, clear bit selects
                             use of shadow mask for that plane
                             OnOff flag: if using shadow mask to fill plane, this bit (corresponding
                             to bit in planePick) describes whether to fill with 0's or 1's
                             There are two uses for these flags:
                               - if this is the VSprite of a Bob, these flags describe how the Bob
                                 is to be drawn into memory
                               - if this is a simple VSprite and the user intends on setting the
                                 MUSTDRAW flag of the VSprite, these flags must be set too to describe
                                 which color registers the user wants for the image}
    PlaneOnOff: ShortInt;
    VUserExt: TVUserStuff; // user definable:  see note above
  end;

// dBufPacket defines the values needed to be saved across buffer to buffer when in double-buffer mode
  PDBufPacket = ^TDBufPacket;
  TDBufPacket = record
    BufY,
    BufX: SmallInt;       // save other buffers screen coordinates
    BufPath: PVSprite;    // carry the draw path over the gap
    BufBuffer: PSmallInt; // this pointer must be filled in by the user pointer to other buffer's background save buffer
  end;

// blitter-objects
  TBob = record
   // SYSTEM VARIABLES
   // COMMON VARIABLES
    Flags: SmallInt;        // general purpose flags (see definitions below)
   // USER VARIABLES
    SaveBuffer: PSmallInt;  // pointer to the buffer for background save
    ImageShadow: PSmallInt; // used by Bobs for "cookie-cutting" and multi-plane masking
    Before: PBob;           // draw this Bob before Bob pointed to by before , for correct overlay
    After: PBob;            // draw this Bob after Bob pointed to by after, for correct overlay
    BobVSprite: PVSprite;   // this Bob's VSprite definition
    BobComp: PAnimComp;     // pointer to this Bob's AnimComp def
    DBuffer: PDBufPacket;   // pointer to this Bob's dBuf packet
    BUserExt: TBUserStuff;  // Bob user extension
  end;


  TAnimComp = record
   // SYSTEM VARIABLES
   // COMMON VARIABLES
    Flags: SmallInt; // AnimComp flags for system & user
    Timer: SmallInt; // timer defines how long to keep this component active:
                     //  if set non-zero, timer decrements to zero then switches to nextSeq
                     //  if set to zero, AnimComp never switches
   // USER VARIABLES
    TimeSet: SmallInt;     // initial value for timer when the AnimComp is activated by the system
    NextComp: PAnimComp;   // pointer to next and previous components of animation object
    PrevComp: PAnimComp;
    NextSeq: PAnimComp;    // pointer to component component definition of next image in sequence
    PrevSeq: PAnimComp;
    AnimCRoutine: Pointer; // Pointer of special animation procedure
    YTrans: SmallInt;      // initial y translation (if this is a component) }
    XTrans: SmallInt;      // initial x translation (if this is a component) }
    HeadOb: PAnimOb;
    AnimBob: PBob;
  end;

  TAnimOb = record
   // SYSTEM VARIABLES
    NextOb,
    PrevOb: PAnimOb;
    Clock: Longint;    // number of calls to Animate this AnimOb has endured
    AnOldY,
    AnOldX: SmallInt;  // old y,x coordinates
   // COMMON VARIABLES
    AnY,
    AnX: SmallInt;     // y,x coordinates of the AnimOb
   // USER VARIABLES
    YVel,
    XVel: SmallInt;        // velocities of this object
    YAccel,
    XAccel: SmallInt;      // accelerations of this object
    RingYTrans,
    RingXTrans: SmallInt;  // ring translation values
    AnimORoutine: Pointer; // Pointer of special animation procedure
    HeadComp: PAnimComp;   // pointer to first component
    AUserExt: TAUserStuff; // AnimOb user extension
  end;
   PPAnimOb = ^PAnimOb;

const
  B2NORM   = 0;
  B2SWAP   = 1;
  B2BOBBER = 2;


const
  MAXSUPERSAVECLIPRECTS = 20; // Max. number of cliprects that are kept preallocated in the list

type
  // predefinitions, for record referencing:
  PRastPort = ^tRastPort;
  PLayer_Info = ^TLayer_Info;
  PLayer = ^TLayer;

  PClipRect = ^TClipRect;
  TClipRect = record
    Next: PClipRect;     // roms used to find next ClipRect
    Prev: PClipRect;     // ignored by roms, used by windowlib
    Lobs: PLayer;        // ignored by roms, used by windowlib
    BitMap: PBitMap;
    Bounds: TRectangle;  // set up by windowlib, used by roms
    _p1,
    _p2: Pointer;        // system reserved
    reserved: Longint;   // system use
    Flags: Longint;      // only exists in layer allocation
  end;


// Layer Structure
  TLayer = record
    Front,
    Back: PLayer;         // ignored by roms
    ClipRect: PClipRect;  // read by roms to find first cliprect
    Rp: PRastPort;        // Ignored by roms, I hope
    Bounds: TRectangle;   // ignored by roms
    Parent: PLayer;        // Private!
    Priority: Word;        // system use only
    Flags: Word;           // obscured ?, Virtual BitMap?
    SuperBitMap: PBitMap;
    SuperClipRect: PClipRect; // super bitmap cliprects if VBitMap != 0 else damage cliprect list for refresh
    Window: APTR;             // reserved for user interface use
    Scroll_X,
    Scroll_Y: SmallInt;
    cr,
    cr2,
    crnew: PClipRect;              // used by dedice
    SuperSaveClipRects: PClipRect; // preallocated cr's
    _cliprects: PClipRect;         // system use during refresh
    LayerInfo: PLayer_Info;        // points to head of the list
    Lock: TSignalSemaphore;
    BackFill: PHook;
    VisibleRegion: PRegion;        // Private!
    ClipRegion: PRegion;
    SaveClipRects: PRegion;        // used to back out when in trouble
    Width,
    Height: SmallInt;
    Shape: PRegion;              // Private!
    ShapeRegion: PRegion;        // Private!
    VisibleShape: PRegion;       // Private!
    Nesting: Word;               // Private!
    SuperSaveClipRectCounter: Byte; // Private!
    Visible: Byte;               // Private!
    Reserved2: array[0..1] of Byte;
    { this must stay here }
    DamageList: PRegion;   // list of rectangles to refreshthrough
  end;

  TLayer_Info = record
    Top_Layer: Player;
    check_lp: PLayer;
    Obs: PClipRect;
    FreeClipRects: PClipRect;

    PrivateReserve1: LongInt;
    PrivateReserve2: LongInt;

    Lock: TSignalSemaphore;
    gs_Head: TMinList;

    PrivateReserve3: SmallInt;
    PrivateReserve4: Pointer;

    Flags: Word;               // LIFLG_SUPPORTS_OFFSCREEN_LAYERS
    fatten_count: ShortInt;
    LockLayersCount: ShortInt;
    PrivateReserve5: SmallInt;
    BlankHook: Pointer;
    LayerInfo_extra: Pointer;
  end;

  TChangeLayerShapeMsg = record
    NewShape: PRegion;    // same as passed to ChangeLayerShape()
    ClipRect: PClipRect;
    Shape: PRegion;
  end;

  TCollectPixelsLayerMsg = record
    xSrc: LongInt;
    ySrc: LongInt;
    Width: LongInt;
    Height: LongInt;
    xDest: LongInt;
    yDest: LongInt;
    Bm: PBitmap;
    Layer: PLayer;
    minterm: ULONG;
  end;

// Msg sent through LA_ShapeHook.
  PShapeHookMsg = ^TShapeHookMsg;
  TShapeHookMsg = record
    Action: LongInt;
    NewShape: PRegion;
    OldShape: PRegion;
    NewBounds: PRectangle;
    OldBounds: PRectangle;
  end;
  // Hook for getting LA_ShapeHook and getting this Msg
  TShapeHookProc = function(Hook: PHook; Layer: PLayer; Msg: PShapeHookMsg): PRegion; cdecl;

  PAreaInfo = ^TAreaInfo;
  TAreaInfo = record
    VctrTbl: PSmallInt;  // ptr to start of vector table
    VctrPtr: PSmallInt;  // ptr to current vertex
    FlagTbl: PShortInt;  // ptr to start of vector flag table
    FlagPtr: PShortInt;  // ptrs to areafill flags
    Count: SmallInt;     // number of vertices in list
    MaxCount: SmallInt;  // AreaMove/Draw will not allow Count>MaxCount
    FirstX,
    FirstY: SmallInt;    // first point for this polygon
  end;

  PTmpRas = ^TTmpRas;
  TTmpRas = record
    RasPtr: PShortInt;
    Size: Longint;
  end;

  PGelsInfo = ^TGelsInfo;
  TGelsInfo = record
    sprRsrvd: Shortint;      // flag of which sprites to reserve from vsprite system
    Flags: Byte;             // system use
    gelHead,
    gelTail: PVSprite;       // dummy vSprites for list management
    NextLine: PSmallInt;     // pointer to array of 8 WORDS for sprite available lines
    LastColor: ^PSmallInt;   // pointer to array of 8 pointers for color-last-assigned to vSprites
    CollHandler: PCollTable; // Pointeres of collision routines
    LeftMost,
    RightMost,
    TopMost,
    BottomMost: Smallint;
    FirstBlissObj,
    LastBlissObj: APTR;      // system use only
  end;

  TRastPort = record
    Layer: PLayer;        // LayerPtr
    BitMap: PBitMap;      // BitMapPtr
    AreaPtrn: PWord;      // ptr to areafill pattern
    TmpRas: PTmpRas;
    AreaInfo: PAreaInfo;
    GelsInfo: PGelsInfo;
    Mask: Byte;           // write mask for this raster
    FgPen: ShortInt;      // foreground pen for this raster
    BgPen: ShortInt;      // background pen
    AOlPen: ShortInt;     // areafill outline pen
    DrawMode: ShortInt;   // drawing mode for fill, lines, and text
    AreaPtSz: ShortInt;   // 2^n words for areafill pattern
    LinPatCnt: ShortInt;  // current line drawing pattern preshift
    dummy: ShortInt;
    Flags: Word;          // miscellaneous control bits
    LinePtrn: Word;       // 16 bits for textured lines
    cp_x,
    cp_y: SmallInt;       // current pen position
    minterms: array[0..7] of Byte;
    PenWidth: SmallInt;
    PenHeight: SmallInt;
    Font: PTextFont;       // (TextFontPtr) current font Pointer
    AlgoStyle: Byte;       // the algorithmically generated style
    TxFlags: Byte;         // text specific flags
    TxHeight: Word;        // text height
    TxWidth: Word;         // text nominal width
    TxBaseline: Word;      // text baseline
    TxSpacing: SmallInt;   // text spacing (per character)
    RP_User: APTR;
    {$ifdef AROS_ABIv0}
    longreserved: array[0..1] of IPTR;
    wordreserved: array[0..6] of Word; // used to be a node
    reserved: array[0..7] of Byte;     // for future use
    {$endif}
    {$ifdef AROS_ABIv1}
    RP_Extra: APTR;
    longreserved: array[0..5] of IPTR;
    reserved: array[0..1] of Byte;
    {$endif}
  end;

const
// these are the flag bits for RastPort flags
  FRST_DOT    = $01; // draw the first dot of this line ?
  ONE_DOT     = $02; // use one dot mode for drawing lines
  DBUFFER     = $04; // flag set when RastPorts are double-buffered
// drawing modes
  JAM1        = 0; // jam 1 color into raster
  JAM2        = 1; // jam 2 colors into raster
  COMPLEMENT  = 2; // XOR bits into raster
  INVERSVID   = 4; // inverse video for drawing modes
// only used for bobs
  AREAOUTLINE = $08; // used by areafiller
  NOCROSSFILL = $20; // areafills have no crossovers

// Actions for TShapeHookMsg
  SHAPEHOOKACTION_CREATELAYER   = 0;
  SHAPEHOOKACTION_MOVELAYER     = 1;
  SHAPEHOOKACTION_SIZELAYER     = 2;
  SHAPEHOOKACTION_MOVESIZELAYER = 3;

// Tags for scale layer
  LA_SRCX       = $4000;
  LA_SRCY       = $4001;
  LA_DESTX      = $4002;
  LA_DESTY      = $4003;
  LA_SRCWIDTH   = $4004;
  LA_SRCHEIGHT  = $4005;
  LA_DESTWIDTH  = $4006;
  LA_DESTHEIGHT = $4007;

  ROOTPRIORITY     =  0;
  BACKDROPPRIORITY = 10;
  UPFRONTPRIORITY  = 20;

// Layer constants
  NEWLAYERINFO_CALLED  = 1;

  LAYERSIMPLE          = 1 shl 0;
  LAYERSMART           = 1 shl 1;
  LAYERSUPER           = 1 shl 2;
  LAYERUPDATING        = 1 shl 4;
  LAYERBACKDROP        = 1 shl 6;
  LAYERREFRESH         = 1 shl 7;
  LAYER_CLIPRECTS_LOST = 1 shl 8;
  LAYERIREFRESH        = 1 shl 9;
  LAYERIREFRESH2       = 1 shl 10;
  LAYER_ROOT_LAYER     = 1 shl 14;

  LAYERS_BACKFILL: PHook   = nil;
  LAYERS_NOBACKFILL: PHook = PHook(1);

// LayerInfo Flag
  LIFLG_SUPPORTS_OFFSCREEN_LAYERS = 1 shl 8;  // Same flag as AmigaOS hack PowerWindowsNG

// Tags for CreateLayerTagList
  LA_Dummy       = TAG_USER + 1234;
  LA_Type        = LA_Dummy + 1; // LAYERSIMPLE, LAYERSMART (default) -or LAYERSUPER
  LA_Priority    = LA_Dummy + 2; // -128 .. 127 or LPRI_NORMAL (default) or LPRI_BACKDROP
  LA_Behind      = LA_Dummy + 3; // LongBool. Default is FALSE
  LA_Invisible   = LA_Dummy + 4; // LongBool. Default is FALSE
  LA_BackFill    = LA_Dummy + 5; // PHook. Default is LAYERS_BACKFILL
  LA_SuperBitMap = LA_Dummy + 6; // PBitMap. Default is nil (none)
  LA_Shape       = LA_Dummy + 7; // PRegion. Default is nil (rectangular shape)

  LPRI_NORMAL    = 0;
  LPRI_BACKDROP  = -50;


const
// tfe_Flags0 (partial definition)
  TE0B_NOREMFONT = 0;    // disallow RemFont for this font
  TE0F_NOREMFONT = $01;

Const
    CleanUp     = $40;
    CleanMe     = CleanUp;

    BltClearWait        = 1;    { Waits for blit to finish }
    BltClearXY          = 2;    { Use Row/Bytes per row method }

        { Useful minterms }

    StraightCopy        = $C0;  { Vanilla copy }
    InvertAndCopy       = $30;  { Invert the source before copy }
    InvertDest          = $50;  { Forget source, invert dest }

const
// These flags are passed (in combination) to CoerceMode() to determine the type of coercion required.
  PRESERVE_COLORS = 1; // Ensure that the mode coerced to can display just as many colours as the ViewPort being coerced.
  AVOID_FLICKER   = 2; // Ensure that the mode coerced to is not interlaced.
  IGNORE_MCOMPAT  = 4; // Coercion should ignore monitor compatibility issues.
  BIDTAG_COERCE   = 1; // Private

const
{ These bit descriptors are used by the GEL collide routines.
  These bits are set in the hitMask and meMask variables of
  a GEL to describe whether or not these types of collisions
  can affect the GEL.  BNDRY_HIT is described further below;
  this bit is permanently assigned as the boundary-hit flag.
  The other bit GEL_HIT is meant only as a default to cover
  any GEL hitting any other; the user may redefine this bit.}
  BORDERHIT = 0;
{ These bit descriptors are used by the GEL boundry hit routines.
  When the user's boundry-hit routine is called (via the argument
  set by a call to SetCollision) the first argument passed to
  the user's routine is the Pointer of the GEL involved in the
  boundry-hit, and the second argument has the appropriate bit(s)
  set to describe which boundry was surpassed}
  TOPHIT    = 1;
  BOTTOMHIT = 2;
  LEFTHIT   = 4;
  RIGHTHIT  = 8;

type
  PExtendedNode = ^TExtendedNode;
  TExtendedNode = record
    xln_Succ,
    xln_Pred: PNode;
    xln_Type: Byte;          // NT_GRAPHICS
    xln_Pri: ShortInt;
    xln_Name: PChar;
    xln_Subsystem: Byte;     // see below
    xln_Subtype: Byte;       // SS_GRAPHICS
    xln_Library : Longint;
    xln_Init : Pointer;
  end;

const
// for xln_Subtype
  SS_GRAPHICS   =  $02;
// for xln_Subsystem
  VIEW_EXTRA_TYPE       =  1;
  VIEWPORT_EXTRA_TYPE   =  2;
  SPECIAL_MONITOR_TYPE  =  3;
  MONITOR_SPEC_TYPE     =  4;

type
  PAnalogSignalInterval = ^TAnalogSignalInterval;
  TAnalogSignalInterval = record
    asi_Start,
    asi_Stop: Word;
  end;

  PSpecialMonitor = ^TSpecialMonitor;
  TSpecialMonitor = record
    spm_Node: TExtendedNode;
    spm_Flags: Word;          // Reserved, set to 0
    do_monitor,               // Driver call vector - set up a video mode
    reserved1,                // Private do not touch
    reserved2,
    reserved3: Pointer;
    hblank,                   // Signal timings by themselves
    vblank,
    hsync,
    vsync: TAnalogSignalInterval;
  end;

  PMonitorSpec = ^TMonitorSpec;
  TMonitorSpec = record
    ms_Node: TExtendedNode;
    ms_Flags: Word;  // Flags, see below
    ratioh,
    ratiov: Longint;
    total_rows,              // Total number of scanlines per frame
    total_colorclocks,       // Total number of color clocks per line (in 1/280 ns units)
    DeniseMaxDisplayColumn,
    BeamCon0,                // Value for beamcon0 Amiga(tm) chipset register
    min_row: Word;

    ms_Special: PSpecialMonitor; // Synchro signal timings description (optional)

    ms_OpenCount: Word;         // Driver open count
    ms_transform,
    ms_translate,
    ms_scale: Pointer;
    ms_xoffset,
    ms_yoffset: Word;

    ms_LegalView: TRectangle;  // Allowed range for view positioning (right-bottom position included)

    ms_maxoscan,               // maximum legal overscan
    ms_videoscan: Pointer;     // video display overscan
    DeniseMinDisplayColumn: Word;
    DisplayCompatible: ULONG;

    DisplayInfoDataBase: TList;
    DisplayInfoDataBaseSemaphore: TSignalSemaphore;

    ms_MrgCop,             // Driver call vectors, unused by AROS
    ms_LoadView,
    ms_KillView: Pointer;
  end;

const
// Flags for TMonitorSpec.ms_Flags
  MSB_REQUEST_NTSC      =  0;
  MSB_REQUEST_PAL       =  1;
  MSB_REQUEST_SPECIAL   =  2;
  MSB_REQUEST_A2024     =  3;
  MSB_DOUBLE_SPRITES    =  4;
  MSF_REQUEST_NTSC      =  1 shl MSB_REQUEST_NTSC;
  MSF_REQUEST_PAL       =  1 shl MSB_REQUEST_PAL;
  MSF_REQUEST_SPECIAL   =  1 shl MSB_REQUEST_SPECIAL;
  MSF_REQUEST_A2024     =  1 shl MSB_REQUEST_A2024;
  MSF_DOUBLE_SPRITES    =  1 shl MSB_DOUBLE_SPRITES;

  TO_MONITOR            =  0;
  FROM_MONITOR          =  1;

  STANDARD_XOFFSET      =  9;
  STANDARD_YOFFSET      =  0;

{ obsolete, v37 compatible definitions follow }
  REQUEST_NTSC          =  1;
  REQUEST_PAL           =  2;
  REQUEST_SPECIAL       =  4;
  REQUEST_A2024         =  8;

  DEFAULT_MONITOR_NAME: PChar = 'default.monitor';
  NTSC_MONITOR_NAME: PChar    = 'ntsc.monitor';
  PAL_MONITOR_NAME: PChar     = 'pal.monitor';
  VGA_MONITOR_NAME: PChar     = 'vga.monitor';

  STANDARD_MONITOR_MASK =  REQUEST_NTSC or REQUEST_PAL;

// Some standard/default constants for Amiga(tm) chipset
  STANDARD_NTSC_ROWS    =   262;
  MIN_NTSC_ROW          =    21;
  STANDARD_PAL_ROWS     =   312;
  MIN_PAL_ROW           =    29;
  STANDARD_NTSC_BEAMCON = $0000;
  STANDARD_PAL_BEAMCON  = $0020;
  SPECIAL_BEAMCON        = VARVBLANK or VARHSYNC or VARVSYNC or VARBEAM or VSYNCTRUE or LOLDIS or CSBLANK;
  STANDARD_DENISE_MIN   =    93;
  STANDARD_DENISE_MAX   =   455;
  STANDARD_COLORCLOCKS  =   226;
  STANDARD_VIEW_X =   $81;
  STANDARD_VIEW_Y =   $2C;
  STANDARD_HBSTRT =   $06;
  STANDARD_HSSTRT =   $0B;
  STANDARD_HSSTOP =   $1C;
  STANDARD_HBSTOP =   $2C;
  STANDARD_VBSTRT = $0122;
  STANDARD_VSSTRT = $02A6;
  STANDARD_VSSTOP = $03AA;
  STANDARD_VBSTOP = $1066;

  VGA_COLORCLOCKS = STANDARD_COLORCLOCKS / 2;
  VGA_TOTAL_ROWS  = STANDARD_NTSC_ROWS * 2;
  VGA_DENISE_MIN  =    59;
  MIN_VGA_ROW     =    29;
  VGA_HBSTRT      =   $08;
  VGA_HSSTRT      =   $0E;
  VGA_HSSTOP      =   $1C;
  VGA_HBSTOP      =   $1E;
  VGA_VBSTRT      = $0000;
  VGA_VSSTRT      = $0153;
  VGA_VSSTOP      = $0235;
  VGA_VBSTOP      = $0CCD;

  BROADCAST_HBSTRT  =   $01;
  BROADCAST_HSSTRT  =   $06;
  BROADCAST_HSSTOP  =   $17;
  BROADCAST_HBSTOP  =   $27;
  BROADCAST_VBSTRT  = $0000;
  BROADCAST_VSSTRT  = $02A6;
  BROADCAST_VSSTOP  = $054C;
  BROADCAST_VBSTOP  = $1C40;
  BROADCAST_BEAMCON = LOLDIS OR CSBLANK;

  RATIO_FIXEDPART   = 4;
  RATIO_UNITY       = 1 shl 4;

type
  PCopList = ^TCopList;
  PViewPort = ^TViewPort;
  PColorMap = ^TColorMap;

// Describes playfield(s) (actually bitmaps)
  PRasInfo = ^TRasInfo;
  TRasInfo = record     // used by callers to and InitDspC()
    Next: PRasInfo;     // Pointer to a next playfield (if there's more than one)
    BitMap: PBitMap;    // Actual data to display
    RxOffset,           // Offset of the playfield relative to ViewPort
    RyOffset: SmallInt; // scroll offsets in this BitMap (So that different playfields may be shifted against each other)
  end;

// structure of cprlist that points to list that hardware actually executes
  PCprList = ^TCprList;
  TCprList = record
    Next: PCprList;
    Start: PWord;       // start of copper list
    MaxCount: SmallInt; // number of long instructions
  end;

  PUCopList = ^TUCopList;
  TUCopList = record
    Next: PUCopList;
    FirstCopList: PCopList; // head node of this copper list
    CopList: PCopList;      // node in use
  end;

  PCopInit = ^TCopInit;
  TCopInit = record
    vsync_hblank: array[0..1] of Word;
    diagstrt: array[0..11] of Word;
    fm0: array[0..1] of Word;
    diwstart: array[0..9] of Word;
    bplcon2: array[0..1] of Word;
    sprfix: array[0..15] of Word;
    sprstrtup: array[0..31] of Word;
    wait14: array[0..1] of Word;
    norm_hblank: array[0..1] of Word;
    jump: array[0..1] of Word;
    wait_forever: array[0..5] of Word;
    sprstop: array[0..7] of Word;
  end;

  PView = ^TView;
  TView = record
    ViewPort: PViewPort;  // ViewPortPtr
    LOFCprList: PCprList; // used for interlaced and noninterlaced
    SHFCprList: PCprList; // only used during interlace
    DyOffset,
    DxOffset: SmallInt;   // for complete View positioning offsets are +- adjustments to standard #s
    Modes: Word;          // such as INTERLACE, GENLOC
  end;

// Additional data for Amiga(tm) chipset. Not used by other hardware.
// these structures are obtained via GfxNew and disposed by GfxFree
  PViewExtra = ^TViewExtra;
  TViewExtra = record
    n: TExtendedNode;      // Common Header
    View: PView;           // View it relates to
    Monitor: PMonitorSpec; // Monitor used for displaying this View
    TopLine: Word;
  end;

  // Copper structures
  PCopIns = ^TCopIns;
  TCopIns = record
    OpCode: smallint; // 0 = move, 1 = wait
    case SmallInt of
    0:(
      NxtList: PCopList;
      );
    1:(
      DestAddr: SmallInt; // destination Pointer
      DestData: SmallInt; // data to send
      );
    2:(
      VWaitPos: SmallInt; // vertical wait position
      HWaitPos: SmallInt; // horizontal wait position
      );
  end;

  TCopList = record
    Next: PCopList;       // next block for this copper list
    _CopList: PCopList;   // system use
    _ViewPort: PViewPort; // system use
    CopIns: PCopIns;      // start of this block
    CopPtr: PCopIns;      // intermediate ptr
    CopLStart: PSmallInt; // mrgcop fills this in for Long Frame
    CopSStart: PSmallInt; // mrgcop fills this in for Longint Frame
    Count: Smallint;      // intermediate counter
    MaxCount: SmallInt;   // max # of copins for this block
    DyOffset: SmallInt;   // offset this copper list vertical waits
    SLRepeat: Word;
    Flags: Word;          // EXACT_LINE or HALF_LINE
  end;

// Describes a displayed bitmap (or logical screen). Copperlists are relevant only to Amiga(tm) chipset,
// for other hardware they are NULL.
  TViewPort = record
    Next: PViewPort;
    ColorMap: PColorMap; // table of colors for this viewport if this is nil, MakeVPort assumes default values
    DspIns: PCopList;    // user by MakeView(), Preliminary partial display copperlist
    SprIns: PCopList;    // used by sprite stuff, Preliminary partial sprite copperlist
    ClrIns: PCopList;    // used by sprite stuff
    UCopIns: PUCopList;  // User copper list
    DWidth,              // Width of currently displayed part in pixels
    DHeight: SmallInt;   // Height of currently displayed part in pixels
    DxOffset,            // Displacement from the (0, 0) of the physical screen to (0, 0) of the raster
    DyOffset: SmallInt;
    Modes: Word;         // The same as in View
    SpritePriorities: Byte; // used by makevp
    reserved: Byte;
    RasInfo: PRasInfo;   // Playfield specification
  end;

// Holds additional information about the ViewPort it is associated with
// this structure is obtained via GfxNew and disposed by GfxFree
  PViewPortExtra = ^TViewPortExtra;
  TViewPortExtra = record
    n: TExtendedNode;
    ViewPort: PViewPort;     // ViewPort it relates to (backward link)
    DisplayClip: TRectangle; // makevp display clipping information, Total size of displayable part
    VecTable: APTR;          // Unused in AROS
    DriverData: array[0..1] of APTR; // Private storage for display drivers. Do not touch!
    Flags: Word;             // Flags, see below
    Origin: array[0..1] of TPoint; // First visible point relative to the DClip. One for each possible playfield.
    cop1ptr,                 // private
    cop2ptr: ULONG;          // private
  end;

  PPaletteExtra = ^TPaletteExtra;
  TPaletteExtra = record             // structure may be extended so watch out!
    pe_Semaphore: TSignalSemaphore;  // shared semaphore for arbitration
    pe_FirstFree,                    // *private*
    pe_NFree,                        // number of free colors
    pe_FirstShared,                  // *private*
    pe_NShared: Word;                // *private*
    pe_RefCnt: PByte;                // *private*
    pe_AllocList: PByte;             // *private*
    pe_ViewPort: PViewPort;          // back pointer to viewport
    pe_SharableColors: Word;         // the number of sharable colors.
  end;

{ This structure is the primary storage for palette data.
  Color data itself is stored in two tables: ColorTable and LowColorBits.
  These fields are actually pointer to arrays of Words. Each Word corresponds
  to one color.
  Number of Words in these array is equal to Count value in this structure.
  ColorTable stores upper nibbles of RGB values, LowColorBits stores low nibbles.
  Example:
   color number 4, value: $00ABCDEF
   ColorTable[4] := $0ACE,
   LowColorBits[4] := $0BDF

  SpriteBase fields keep bank number, not a color number. On m68k Amiga colors are divided into
  banks, 16 per each. So bank number is color number divided by 16. Base color is a number which
  is added to all colors of the sprite in order to look up the actual palette entry.
  AROS may run on different hardware where sprites may have base colors that do not divide by 16.
  In order to cover this bank numbers have a form: ((c and $0F) shl 8 ) or (c shr 4), where c is actual
  color number (i. e. remainder is stored in a high byte of Word).}
  TColorMap = record
    Flags: Byte;            // see below (CMF_*)
    Type_: Byte;            // Colormap type (reflects version), see below (COLORMAP_*)
    Count: Word;            // Number of palette entries
    ColorTable: PWord;      // Table of high nibbles of color values (see description above)
      // The following fields are present only if Type_ >= COLORMAP_TYPE_V36
    cm_vpe: PViewPortExtra; // ViewPortExtra, for faster access

    LowColorBits: PWord;    // Table of low nibbles of color values (see above)
    TransparencyPlane,
    SpriteResolution,       // see below
    SpriteResDefault,
    AuxFlags: Byte;

    cm_vp: PViewPort;       // Points back to a ViewPort this colormap belongs to

    NormalDisplayInfo,
    CoerceDisplayInfo : APTR;

    cm_batch_items: PTagItem;
    VPModeID: ULONG;
      // The following fields are present only if Type_ >= COLORMAP_TYPE_V39
    PalExtra: PPaletteExtra; // Structure controlling palette sharing

    SpriteBase_Even,         // Color bank for even sprites (see above)
    SpriteBase_Odd,          // The same for odd sprites
    Bp_0_base,
    Bp_1_base: Word;
  end;

const
// flags for TColorMap.Flags
  CMF_CMTRANS             = 0;
  COLORMAP_TRANSPARENCY   = 1 shl 0;
  CMF_CPTRANS             = 1;
  COLORPLANE_TRANSPARENCY = 1 shl 1;
  CMF_BRDRBLNK            = 2;
  BORDER_BLANKING         = 1 shl 2;
  CMF_BRDNTRAN            = 3;
  BORDER_NOTRANSPARENCY   = 1 shl 3;
  VIDEOCONTROL_BATCH      = 1 shl 4;
  USER_COPPER_CLIP        = 1 shl 5;
  CMF_BRDRSPRT            = 6;
  BORDERSPRITES           = 1 shl 6;
// Types for TColorMap.Type_
  COLORMAP_TYPE_V1_2      = 0;
  COLORMAP_TYPE_V36       = 1;
  COLORMAP_TYPE_V39       = 2;
// SpriteResolution
  SPRITERESN_ECS          = $00;
  SPRITERESN_140NS        = $01; // ^140ns, except in 35ns viewport, where it is 70ns.
  SPRITERESN_70NS         = $02;
  SPRITERESN_35NS         = $03;
  SPRITERESN_DEFAULT      = $ff;

// Private Flags for TCopList.Flags
  EXACT_LINE = 1;
  HALF_LINE  = 2;
// Copper commands
  COPPER_MOVE = 0;     // pseude opcode for move #XXXX,dir
  COPPER_WAIT = 1;     // pseudo opcode for wait y,x
  CPRNXTBUF   = 2;     // continue processing with next buffer
  CPR_NT_SYS  = $2000; // copper user instruction only
  CPR_NT_SHT  = $4000; // copper instruction only for long frames
  CPR_NT_LOF  = $8000; // copper instruction only for Longint frames

  GENLOCK_VIDEO   = 1 shl 1;
  LACE            = 1 shl 2;
  DOUBLESCAN      = 1 shl 3;
  SUPERHIRES      = 1 shl 5;
  PFBA            = 1 shl 6;
  EXTRA_HALFBRITE = 1 shl 7;
  GENLOCK_AUDIO   = 1 shl 8;
  DUALPF          = 1 shl 10;
  HAM             = 1 shl 11;
  EXTENDED_MODE   = 1 shl 12;
  VP_HIDE         = 1 shl 13;
  SPRITES         = 1 shl 14;
  HIRES           = 1 shl 15;
// ViewPortExtra Flags
  VPXB_FREE_ME      = 0;       // Temporary ViewPortExtra allocated during MakeVPort(). ViewPortExtra with this flag
  VPXF_FREE_ME      = 1 shl 0; // will be automatically found and disposed during FreeVPortCopLists().
                               // Private flag in fact, don't set it by hands
  VPXB_LAST         = 1;
  VPXF_LAST         = 1 shl 1;
  VPXB_STRADDLES256 = 4;
  VPXF_STRADDLES256 = 1 shl 4;
  VPXB_STRADDLES512 = 5;
  VPXF_STRADDLES512 = 1 shl 5;
// Private
  VPB_TENHZ      = 4;
  VPF_TENHZ      = 1 shl 4;
  VPB_A2024      = 6;
  VPF_A2024      = 1 shl 6;

  EXTEND_VSTRUCT = $1000;  // unused bit in Modes field of View

// AuxFlags
  CMAB_FULLPALETTE        = 0;
  CMAF_FULLPALETTE        = 1 shl CMAB_FULLPALETTE;
  CMAB_NO_INTERMED_UPDATE = 1;
  CMAF_NO_INTERMED_UPDATE = 1 shl CMAB_NO_INTERMED_UPDATE;
  CMAB_NO_COLOR_LOAD      = 2;
  CMAF_NO_COLOR_LOAD      = 1 shl CMAB_NO_COLOR_LOAD;
  CMAB_DUALPF_DISABLE     = 3;
  CMAF_DUALPF_DISABLE     = 1 shl CMAB_DUALPF_DISABLE;

const
// flags values for ObtainPen
  PENB_EXCLUSIVE   = 0;
  PENB_NO_SETCOLOR = 1;
  PENF_EXCLUSIVE   = 1 shl PENB_EXCLUSIVE;
  PENF_NO_SETCOLOR = 1 shl PENB_NO_SETCOLOR;
// obsolete names for PENF_xxx flags:
  PEN_EXCLUSIVE    = PENF_EXCLUSIVE;
  PEN_NO_SETCOLOR  = PENF_NO_SETCOLOR;
// precision values for ObtainBestPen:
  PRECISION_EXACT  = -1;
  PRECISION_IMAGE  =  0;
  PRECISION_ICON   = 16;
  PRECISION_GUI    = 32;
// tags for ObtainBestPen:
  OBP_Precision    = $84000000;
  OBP_FailIfBad    = $84000001;

{ MakeVPort() will return an error if there is not enough memory,
  or the requested mode cannot be opened with the requested depth with the
  given bitmap (for higher bandwidth alignments).}
  MVP_OK           =  0; // you want to see this one
  MVP_NO_MEM       =  1; // insufficient memory for intermediate workspace
  MVP_NO_VPE       =  2; // ViewPort does not have a ViewPortExtra, and insufficient memory to allocate a temporary one.
  MVP_NO_DSPINS    =  3; // insufficient memory for intermidiate copper instructions.
  MVP_NO_DISPLAY   =  4; // BitMap data is misaligned for this viewport's mode and depth - see AllocBitMap().
  MVP_OFF_BOTTOM   =  5; // PRIVATE - you will never see this.
{ MrgCop() will return an error if there is not enough memory,
  or for some reason MrgCop() did not need to make any copper lists.}
  MCOP_OK         =  0; // you want to see this one
  MCOP_NO_MEM     =  1; // insufficient memory to allocate the system copper lists.
  MCOP_NOP        =  2; // MrgCop() did not merge any copper lists (eg, no ViewPorts in the list, or all marked as hidden).

type
  PDBufInfo = ^TDBufInfo;
  TDBufInfo = record
    dbi_Link1: APTR;
    dbi_Count1: ULONG;
    dbi_SafeMessage: TMessage; // replied to when safe to write to old bitmap
    dbi_UserData1: APTR;       // first user data

    dbi_Link2: APTR;
    dbi_Count2: ULONG;
    dbi_DispMessage: TMessage; // replied to when new bitmap has been displayed at least once
    dbi_UserData2: APTR;       // second user data
    dbi_MatchLong: ULONG;
    dbi_CopPtr1,
    dbi_CopPtr2,
    dbi_CopPtr3: APTR;
    dbi_BeamPos1,
    dbi_BeamPos2: Word;
  end;

const
  INVALID_ID = not 0;

{ With all the new modes that are available under V38 and V39, it is highly
 * recommended that you use either the asl.library screenmode requester,
 * and/or the V39 graphics.library function BestModeIDA().
 *
 * DO NOT interpret the any of the bits in the ModeID for its meaning. For
 * example, do not interpret bit 3 ($4) as meaning the ModeID is interlaced.
 * Instead, use GetDisplayInfoData() with DTAG_DISP, and examine the DIPF_...
 * flags to determine a ModeID's characteristics. The only exception to
 * this rule is that bit 7 ($80) will always mean the ModeID is
 * ExtraHalfBright, and bit 11 ($800) will always mean the ModeID is HAM.
 }

// normal identifiers
   MONITOR_ID_MASK              =   $FFFF1000;

   DEFAULT_MONITOR_ID           =   $00000000;
   NTSC_MONITOR_ID              =   $00011000;
   PAL_MONITOR_ID               =   $00021000;

{ the following 22 composite keys are for Modes on the default Monitor.
  NTSC & PAL "flavors" of these particular keys may be made by or'ing
  the NTSC or PAL MONITOR_ID with the desired MODE_KEY...

  For example, to specifically open a PAL HAM interlaced ViewPort
  (or intuition screen), you would use the modeid of
  (PAL_MONITOR_ID or HAMLACE_KEY)}
   LORES_KEY                     =  $00000000;
   LORESLACE_KEY                 =  $00000004;
   LORESSDBL_KEY                 =  $00000008;
   EXTRAHALFBRITE_KEY            =  $00000080;
   EXTRAHALFBRITELACE_KEY        =  $00000084;
   LORESEHBSDBL_KEY              =  $00000088;
   LORESDPF_KEY                  =  $00000400;
   LORESLACEDPF_KEY              =  $00000404;
   LORESDPF2_KEY                 =  $00000440;
   LORESLACEDPF2_KEY             =  $00000444;
   HAM_KEY                       =  $00000800;
   HAMLACE_KEY                   =  $00000804;
   LORESHAMSDBL_KEY              =  $00000808;
   HIRES_KEY                     =  $00008000;
   HIRESLACE_KEY                 =  $00008004;
   SUPER_KEY                     =  $00008020;
   SUPERLACE_KEY                 =  $00008024;
   HIRESEHB_KEY                  =  $00008080;
   HIRESEHBLACE_KEY              =  $00008084;
   SUPEREHB_KEY                  =  $000080a0;
   SUPEREHBLACE_KEY              =  $000080a4;
   HIRESDPF_KEY                  =  $00008400;
   HIRESLACEDPF_KEY              =  $00008404;
   SUPERDPF_KEY                  =  $00008420;
   SUPERLACEDPF_KEY              =  $00008424;
   HIRESDPF2_KEY                 =  $00008440;
   HIRESLACEDPF2_KEY             =  $00008444;
   SUPERDPF2_KEY                 =  $00008460;
   SUPERLACEDPF2_KEY             =  $00008464;
   HIRESHAM_KEY                  =  $00008800;
   HIRESHAMLACE_KEY              =  $00008804;
   HIRESHAMSDBL_KEY              =  $00008808;
   SUPERHAM_KEY                  =  $00008820;
   SUPERHAMLACE_KEY              =  $00008824;

// VGA identifiers
   VGA_MONITOR_ID                =  $00031000;

   VGAEXTRALORES_KEY             =  $00031004;
   VGALORES_KEY                  =  $00039004;
   VGAPRODUCT_KEY                =  $00039024;
   VGAHAM_KEY                    =  $00031804;
   VGAEXTRALORESLACE_KEY         =  $00031005;
   VGALORESLACE_KEY              =  $00039005;
   VGAPRODUCTLACE_KEY            =  $00039025;
   VGAHAMLACE_KEY                =  $00031805;
   VGAEXTRALORESDPF_KEY          =  $00031404;
   VGALORESDPF_KEY               =  $00039404;
   VGAPRODUCTDPF_KEY             =  $00039424;
   VGAEXTRALORESLACEDPF_KEY      =  $00031405;
   VGALORESLACEDPF_KEY           =  $00039405;
   VGAPRODUCTLACEDPF_KEY         =  $00039425;
   VGAEXTRALORESDPF2_KEY         =  $00031444;
   VGALORESDPF2_KEY              =  $00039444;
   VGAPRODUCTDPF2_KEY            =  $00039464;
   VGAEXTRALORESLACEDPF2_KEY     =  $00031445;
   VGALORESLACEDPF2_KEY          =  $00039445;
   VGAPRODUCTLACEDPF2_KEY        =  $00039465;
   VGAEXTRAHALFBRITE_KEY         =  $00031084;
   VGAEXTRAHALFBRITELACE_KEY     =  $00031085;
   VGAPRODUCTHAM_KEY             =  $00039824;
   VGALORESHAM_KEY               =  $00039804;
   VGAEXTRALORESHAM_KEY          =  VGAHAM_KEY;
   VGAPRODUCTHAMLACE_KEY         =  $00039825;
   VGALORESHAMLACE_KEY           =  $00039805;
   VGAEXTRALORESHAMLACE_KEY      =  VGAHAMLACE_KEY;
   VGAEXTRALORESEHB_KEY          =  VGAEXTRAHALFBRITE_KEY;
   VGAEXTRALORESEHBLACE_KEY      =  VGAEXTRAHALFBRITELACE_KEY;
   VGALORESEHB_KEY               =  $00039084;
   VGALORESEHBLACE_KEY           =  $00039085;
   VGAEHB_KEY                    =  $000390a4;
   VGAEHBLACE_KEY                =  $000390a5;
   VGAEXTRALORESDBL_KEY          =  $00031000;
   VGALORESDBL_KEY               =  $00039000;
   VGAPRODUCTDBL_KEY             =  $00039020;
   VGAEXTRALORESHAMDBL_KEY       =  $00031800;
   VGALORESHAMDBL_KEY            =  $00039800;
   VGAPRODUCTHAMDBL_KEY          =  $00039820;
   VGAEXTRALORESEHBDBL_KEY       =  $00031080;
   VGALORESEHBDBL_KEY            =  $00039080;
   VGAPRODUCTEHBDBL_KEY          =  $000390a0;
// A2024 identifiers
   A2024_MONITOR_ID              =  $00041000;
   A2024TENHERTZ_KEY             =  $00041000;
   A2024FIFTEENHERTZ_KEY         =  $00049000;
// prototype identifiers (private)
   PROTO_MONITOR_ID              =  $00051000;
// Euro72 defines
   EURO72_MONITOR_ID             =  $00061000;
   EURO72EXTRALORES_KEY          =  $00061004;
   EURO72LORES_KEY               =  $00069004;
   EURO72PRODUCT_KEY             =  $00069024;
   EURO72HAM_KEY                 =  $00061804;
   EURO72EXTRALORESLACE_KEY      =  $00061005;
   EURO72LORESLACE_KEY           =  $00069005;
   EURO72PRODUCTLACE_KEY         =  $00069025;
   EURO72HAMLACE_KEY             =  $00061805;
   EURO72EXTRALORESDPF_KEY       =  $00061404;
   EURO72LORESDPF_KEY            =  $00069404;
   EURO72PRODUCTDPF_KEY          =  $00069424;
   EURO72EXTRALORESLACEDPF_KEY   =  $00061405;
   EURO72LORESLACEDPF_KEY        =  $00069405;
   EURO72PRODUCTLACEDPF_KEY      =  $00069425;
   EURO72EXTRALORESDPF2_KEY      =  $00061444;
   EURO72LORESDPF2_KEY           =  $00069444;
   EURO72PRODUCTDPF2_KEY         =  $00069464;
   EURO72EXTRALORESLACEDPF2_KEY  =  $00061445;
   EURO72LORESLACEDPF2_KEY       =  $00069445;
   EURO72PRODUCTLACEDPF2_KEY     =  $00069465;
   EURO72EXTRAHALFBRITE_KEY      =  $00061084;
   EURO72EXTRAHALFBRITELACE_KEY  =  $00061085;
   EURO72PRODUCTHAM_KEY          =  $00069824;
   EURO72PRODUCTHAMLACE_KEY      =  $00069825;
   EURO72LORESHAM_KEY            =  $00069804;
   EURO72LORESHAMLACE_KEY        =  $00069805;
   EURO72EXTRALORESHAM_KEY       =  EURO72HAM_KEY;
   EURO72EXTRALORESHAMLACE_KEY   =  EURO72HAMLACE_KEY ;
   EURO72EXTRALORESEHB_KEY       =  EURO72EXTRAHALFBRITE_KEY;
   EURO72EXTRALORESEHBLACE_KEY   =  EURO72EXTRAHALFBRITELACE_KEY;
   EURO72LORESEHB_KEY            =  $00069084;
   EURO72LORESEHBLACE_KEY        =  $00069085;
   EURO72EHB_KEY                 =  $000690a4;
   EURO72EHBLACE_KEY             =  $000690a5;
{ These ModeIDs are the scandoubled equivalents of the above, with the
  exception of the DualPlayfield modes, as AA does not allow for scandoubling
  dualplayfield.}
   EURO72EXTRALORESDBL_KEY       =  $00061000;
   EURO72LORESDBL_KEY            =  $00069000;
   EURO72PRODUCTDBL_KEY          =  $00069020;
   EURO72EXTRALORESHAMDBL_KEY    =  $00061800;
   EURO72LORESHAMDBL_KEY         =  $00069800;
   EURO72PRODUCTHAMDBL_KEY       =  $00069820;
   EURO72EXTRALORESEHBDBL_KEY    =  $00061080;
   EURO72LORESEHBDBL_KEY         =  $00069080;
   EURO72PRODUCTEHBDBL_KEY       =  $000690a0;
// Euro 36
   EURO36_MONITOR_ID             =  $00071000;
{ Euro36 modeids can be ORed with the default modeids a la NTSC and PAL.
  For example, Euro36 SuperHires is
  (EURO36_MONITOR_ID OR SUPER_KEY)}
// Super 72
   SUPER72_MONITOR_ID            =  $00081000;
{ Super72 modeids can be ORed with the default modeids a la NTSC and PAL.
 For example, Super72 SuperHiresLace (80$600) is
  (SUPER72_MONITOR_ID OR SUPERLACE_KEY).
  The following scandoubled Modes are the exception:}
   SUPER72LORESDBL_KEY           =  $00081008;
   SUPER72HIRESDBL_KEY           =  $00089008;
   SUPER72SUPERDBL_KEY           =  $00089028;
   SUPER72LORESHAMDBL_KEY        =  $00081808;
   SUPER72HIRESHAMDBL_KEY        =  $00089808;
   SUPER72SUPERHAMDBL_KEY        =  $00089828;
   SUPER72LORESEHBDBL_KEY        =  $00081088;
   SUPER72HIRESEHBDBL_KEY        =  $00089088;
   SUPER72SUPEREHBDBL_KEY        =  $000890a8;
// DblNTSC
   DBLNTSC_MONITOR_ID            =  $00091000;

   DBLNTSCLORES_KEY              =  $00091000;
   DBLNTSCLORESFF_KEY            =  $00091004;
   DBLNTSCLORESHAM_KEY           =  $00091800;
   DBLNTSCLORESHAMFF_KEY         =  $00091804;
   DBLNTSCLORESEHB_KEY           =  $00091080;
   DBLNTSCLORESEHBFF_KEY         =  $00091084;
   DBLNTSCLORESLACE_KEY          =  $00091005;
   DBLNTSCLORESHAMLACE_KEY       =  $00091805;
   DBLNTSCLORESEHBLACE_KEY       =  $00091085;
   DBLNTSCLORESDPF_KEY           =  $00091400;
   DBLNTSCLORESDPFFF_KEY         =  $00091404;
   DBLNTSCLORESDPFLACE_KEY       =  $00091405;
   DBLNTSCLORESDPF2_KEY          =  $00091440;
   DBLNTSCLORESDPF2FF_KEY        =  $00091444;
   DBLNTSCLORESDPF2LACE_KEY      =  $00091445;
   DBLNTSCHIRES_KEY              =  $00099000;
   DBLNTSCHIRESFF_KEY            =  $00099004;
   DBLNTSCHIRESHAM_KEY           =  $00099800;
   DBLNTSCHIRESHAMFF_KEY         =  $00099804;
   DBLNTSCHIRESLACE_KEY          =  $00099005;
   DBLNTSCHIRESHAMLACE_KEY       =  $00099805;
   DBLNTSCHIRESEHB_KEY           =  $00099080;
   DBLNTSCHIRESEHBFF_KEY         =  $00099084;
   DBLNTSCHIRESEHBLACE_KEY       =  $00099085;
   DBLNTSCHIRESDPF_KEY           =  $00099400;
   DBLNTSCHIRESDPFFF_KEY         =  $00099404;
   DBLNTSCHIRESDPFLACE_KEY       =  $00099405;
   DBLNTSCHIRESDPF2_KEY          =  $00099440;
   DBLNTSCHIRESDPF2FF_KEY        =  $00099444;
   DBLNTSCHIRESDPF2LACE_KEY      =  $00099445;
   DBLNTSCEXTRALORES_KEY         =  $00091200;
   DBLNTSCEXTRALORESHAM_KEY      =  $00091a00;
   DBLNTSCEXTRALORESEHB_KEY      =  $00091280;
   DBLNTSCEXTRALORESDPF_KEY      =  $00091600;
   DBLNTSCEXTRALORESDPF2_KEY     =  $00091640;
   DBLNTSCEXTRALORESFF_KEY       =  $00091204;
   DBLNTSCEXTRALORESHAMFF_KEY    =  $00091a04;
   DBLNTSCEXTRALORESEHBFF_KEY    =  $00091284;
   DBLNTSCEXTRALORESDPFFF_KEY    =  $00091604;
   DBLNTSCEXTRALORESDPF2FF_KEY   =  $00091644;
   DBLNTSCEXTRALORESLACE_KEY     =  $00091205;
   DBLNTSCEXTRALORESHAMLACE_KEY  =  $00091a05;
   DBLNTSCEXTRALORESEHBLACE_KEY  =  $00091285;
   DBLNTSCEXTRALORESDPFLACE_KEY  =  $00091605;
   DBLNTSCEXTRALORESDPF2LACE_KEY =  $00091645;
// DBLPal
   DBLPAL_MONITOR_ID             =  $000a1000;

   DBLPALLORES_KEY               =  $000a1000;
   DBLPALLORESFF_KEY             =  $000a1004;
   DBLPALLORESHAM_KEY            =  $000a1800;
   DBLPALLORESHAMFF_KEY          =  $000a1804;
   DBLPALLORESEHB_KEY            =  $000a1080;
   DBLPALLORESEHBFF_KEY          =  $000a1084;
   DBLPALLORESLACE_KEY           =  $000a1005;
   DBLPALLORESHAMLACE_KEY        =  $000a1805;
   DBLPALLORESEHBLACE_KEY        =  $000a1085;
   DBLPALLORESDPF_KEY            =  $000a1400;
   DBLPALLORESDPFFF_KEY          =  $000a1404;
   DBLPALLORESDPFLACE_KEY        =  $000a1405;
   DBLPALLORESDPF2_KEY           =  $000a1440;
   DBLPALLORESDPF2FF_KEY         =  $000a1444;
   DBLPALLORESDPF2LACE_KEY       =  $000a1445;
   DBLPALHIRES_KEY               =  $000a9000;
   DBLPALHIRESFF_KEY             =  $000a9004;
   DBLPALHIRESHAM_KEY            =  $000a9800;
   DBLPALHIRESHAMFF_KEY          =  $000a9804;
   DBLPALHIRESLACE_KEY           =  $000a9005;
   DBLPALHIRESHAMLACE_KEY        =  $000a9805;
   DBLPALHIRESEHB_KEY            =  $000a9080;
   DBLPALHIRESEHBFF_KEY          =  $000a9084;
   DBLPALHIRESEHBLACE_KEY        =  $000a9085;
   DBLPALHIRESDPF_KEY            =  $000a9400;
   DBLPALHIRESDPFFF_KEY          =  $000a9404;
   DBLPALHIRESDPFLACE_KEY        =  $000a9405;
   DBLPALHIRESDPF2_KEY           =  $000a9440;
   DBLPALHIRESDPF2FF_KEY         =  $000a9444;
   DBLPALHIRESDPF2LACE_KEY       =  $000a9445;
   DBLPALEXTRALORES_KEY          =  $000a1200;
   DBLPALEXTRALORESHAM_KEY       =  $000a1a00;
   DBLPALEXTRALORESEHB_KEY       =  $000a1280;
   DBLPALEXTRALORESDPF_KEY       =  $000a1600;
   DBLPALEXTRALORESDPF2_KEY      =  $000a1640;
   DBLPALEXTRALORESFF_KEY        =  $000a1204;
   DBLPALEXTRALORESHAMFF_KEY     =  $000a1a04;
   DBLPALEXTRALORESEHBFF_KEY     =  $000a1284;
   DBLPALEXTRALORESDPFFF_KEY     =  $000a1604;
   DBLPALEXTRALORESDPF2FF_KEY    =  $000a1644;
   DBLPALEXTRALORESLACE_KEY      =  $000a1205;
   DBLPALEXTRALORESHAMLACE_KEY   =  $000a1a05;
   DBLPALEXTRALORESEHBLACE_KEY   =  $000a1285;
   DBLPALEXTRALORESDPFLACE_KEY   =  $000a1605;
   DBLPALEXTRALORESDPF2LACE_KEY  =  $000a1645;

// Tags
   BIDTAG_DIPFMustHave     = $80000001; // mask of the DIPF_ flags the ModeID must have Default - 0
   BIDTAG_DIPFMustNotHave  = $80000002; // mask of the DIPF_ flags the ModeID must not have Default - SPECIAL_FLAGS
   BIDTAG_ViewPort         = $80000003; // ViewPort for which a ModeID is sought.  Default - nil
   BIDTAG_NominalWidth     = $80000004; // \ together make the aspect ratio and
   BIDTAG_NominalHeight    = $80000005; // / override the vp^.Width/Height. Default - SourceID NominalDimensionInfo,
                                        // or vp^.DWidth/Height, or (640 * 200), in that preferred order.
   BIDTAG_DesiredWidth     = $80000006; // \ Nominal Width and Height of the
   BIDTAG_DesiredHeight    = $80000007; // / returned ModeID. Default - same as Nominal
   BIDTAG_Depth            = $80000008; // ModeID must support this depth. Default - vp^.RasInfo^.BitMap^.Depth or 1
   BIDTAG_MonitorID        = $80000009; // ModeID must use this monitor. Default - use best monitor available
   BIDTAG_SourceID         = $8000000a; // instead of a ViewPort. Default - VPModeID(vp) if BIDTAG_ViewPort is
                                        // specified, else leave the DIPFMustHave and DIPFMustNotHave values untouched.
   BIDTAG_RedBits          = $8000000b; // \
   BIDTAG_BlueBits         = $8000000c; //  > Match up from the database
   BIDTAG_GreenBits        = $8000000d; // /                 Default - 4
   BIDTAG_GfxPrivate       = $8000000e; // Private

type
// the "public" handle to a DisplayInfoRecord
  DisplayInfoHandle = APTR;

  PQueryHeader = ^TQueryHeader;
  TQueryHeader = record
    StructID,        // datachunk type identifier
    DisplayID,       // copy of display record key
    SkipID,          // TAG_SKIP
    Length:  ULONG;  // length of local data in double-longwords
  end;

  PDisplayInfo = ^TDisplayInfo;
  TDisplayInfo = record
    Header: TQueryHeader;
    NotAvailable: Word;       // If 0 DisplayInfo available, else not available -> see Constants DI_AVAIL_*
    PropertyFlags: ULONG;     // Properties of this mode (DIPF_*)
    Resolution: TPoint;       // ticks-per-pixel X/Y
    PixelSpeed: Word;         // approximation in nanoseconds
    NumStdSprites: Word;      // number of standard amiga sprites
    PaletteRange: Word;       // distinguishable shades available
    SpriteResolution: TPoint; // std sprite ticks-per-pixel X/Y
    pad: array[0..3] of Byte;
    RedBits: Byte;
    GreenBits: Byte;
    BlueBits: Byte;
    pad2: array[0..4] of Byte;
    Reserved: Array[0..1] of IPTR; // terminator
  end;

const
 // availability TDisplayInfo.NotAvailable
  DI_AVAIL_NOCHIPS          = 1 shl 0;
  DI_AVAIL_NOMONITOR        = 1 shl 1;
  DI_AVAIL_NOTWITHGENLOCK   = 1 shl 2;
// Property Flags for TDisplayInfo.PropertyFlags
  DIPF_IS_LACE              = 1 shl 0;
  DIPF_IS_DUALPF            = 1 shl 1;
  DIPF_IS_PF2PRI            = 1 shl 2;
  DIPF_IS_HAM               = 1 shl 3;
  DIPF_IS_ECS               = 1 shl 4; // note: ECS modes (SHIRES, VGA, AND PRODUCTIVITY)
                                       // do not support attached sprites.
  DIPF_IS_PAL               = 1 shl 5;
  DIPF_IS_SPRITES           = 1 shl 6;
  DIPF_IS_GENLOCK           = 1 shl 7;
  DIPF_IS_WB                = 1 shl 8;
  DIPF_IS_DRAGGABLE         = 1 shl 9;
  DIPF_IS_PANELLED          = 1 shl 10;
  DIPF_IS_BEAMSYNC          = 1 shl 11;
  DIPF_IS_EXTRAHALFBRITE    = 1 shl 12;
  DIPF_IS_SPRITES_ATT       = 1 shl 13; // supports attached sprites
  DIPF_IS_SPRITES_CHNG_RES  = 1 shl 14; // supports variable sprite resolution
  DIPF_IS_SPRITES_BORDER    = 1 shl 15; // sprite can be displayed in the border
  DIPF_IS_AA                = 1 shl 16; // AA modes - may only be available if machine has correct memory
                                        // type to support required bandwidth - check availability.
  DIPF_IS_SCANDBL           = 1 shl 17; // scan doubled
  DIPF_IS_SPRITES_CHNG_BASE = 1 shl 18; // can change the sprite base colour
  DIPF_IS_SPRITES_CHNG_PRI  = 1 shl 19; // can change the sprite priority with respect to the playfield(s).
  DIPF_IS_DBUFFER           = 1 shl 20; // can support double buffering
  DIPF_IS_PROGBEAM          = 1 shl 21; // is a programmed beam-sync mode
  DIPF_IS_FOREIGN           = 1 shl 22; // this mode is not native to the Amiga

// Use these tags for passing to BestModeID()
   SPECIAL_FLAGS = DIPF_IS_DUALPF or DIPF_IS_PF2PRI or DIPF_IS_HAM or DIPF_IS_EXTRAHALFBRITE;

type
  PDimensionInfo = ^TDimensionInfo;
  TDimensionInfo = record
    Header: TQueryHeader;
    MaxDepth,              // log2(max number of colors)
    MinRasterWidth,        // minimum width in pixels
    MinRasterHeight,       // minimum height in pixels
    MaxRasterWidth,        // maximum width in pixels
    MaxRasterHeight: Word; // maximum height in pixels
    Nominal,               // "standard" dimensions
    MaxOScan,              // fixed, hardware dependant
    VideoOScan,            // fixed, hardware dependant
    TxtOScan,              // editable via preferences
    StdOScan: TRectangle;  // editable via preferences
    Pad: array[0..13] of Byte;
    Reserved: array[0..1] of IPTR; // terminator
  end;

  PMonitorInfo = ^TMonitorInfo;
  TMonitorInfo = record
    Header: TQueryHeader;
    Mspc: PMonitorSpec;            // pointer to monitor specification
    ViewPosition,                  // editable via preferences
    ViewResolution: TPoint;        // standard monitor ticks-per-pixel
    ViewPositionRange: TRectangle; // fixed, hardware dependant
    TotalRows,                     // display height in scanlines
    TotalColorClocks,              // scanline width in 280 ns units
    MinRow: Word;                  // absolute minimum active scanline
    Compatibility: SmallInt;       // how this coexists with others (MCOMPAT_*)
    Pad: array[0..31] of Byte;
    MouseTicks: TPoint;
    DefaultViewPosition: TPoint;
    PreferredModeID: ULONG;
    Reserved: array[0..1] of IPTR; // terminator
  end;

const
// monitor compatibility TMonitorInfo.Compatibility
  MCOMPAT_NOBODY = -1;  // only one viewport at a time
  MCOMPAT_MIXED  =  0;  // can share display with other MCOMPAT_MIXED
  MCOMPAT_SELF   =  1;  // can share only within same monitor

  DISPLAYNAMELEN = 32;
type
  PNameInfo = ^TNameInfo;
  TNameInfo = record
    Header: TQueryHeader;
    Name: array[0..DISPLAYNAMELEN - 1] of Char;
    Reserved: array[0..1] of IPTR; // terminator
  end;

const
  DTAG_DISP = TAG_USER;
  DTAG_DIMS = TAG_USER + $1000;
  DTAG_MNTR = TAG_USER + $2000;
  DTAG_NAME = TAG_USER + $3000;
  DTAG_VEC  = TAG_USER + $4000;   // internal use only

type
// The following VecInfo structure is PRIVATE, for our use only Touch these, and burn!
  PVecInfo = ^TVecInfo;
  TVecInfo = record
    Header: TQueryHeader;
    Vec: APTR;
    Data: APTR;
    Type_: Word;  // original "Type" in C Includes
    pad: array[0..2] of Word;
    Reserved: array[0..1] of IPTR;
  end;
// AROS-specifics.
const
// Tags for AddDisplayDriverA()
  DDRV_BootMode     = TAG_USER + $01; // (LongBool) Boot mode driver which will be
                                      // unloaded when any next driver comes in, default = False
  DDRV_MonitorID    = TAG_USER + $02; // (ULONG) Monitor ID for this driver, default = next available
  DDRV_ReserveIDs   = TAG_USER + $03; // (ULONG) How many monitor IDs to reserve, default = 1
  DDRV_KeepBootMode = TAG_USER + $04; // (LongBool) Do not shut down boot mode drivers, default = False
  DDRV_ResultID     = TAG_USER + $05; // (PLongWord) Obtain assigned monitor ID
  DDRV_IDMask       = TAG_USER + $06; // (ULONG) Use own mask for monitor ID separation
// Return codes
  DD_OK        = 0; // No error
  DD_NO_MEM    = 1; // Out of memory
  DD_ID_EXISTS = 2; // Specified MonitorID is already allocated

type
// This structure is subject to change! Private!
  PMonitorHandle = ^TMonitorHandle;
  TMonitorHandle = record
    Next: PMonitorHandle;
    id: ULONG;
    mask: ULONG;
    gfxhidd: APTR;
  end;

const
  VTAG_END_CM            = $00000000;
  VTAG_CHROMAKEY_CLR     = $80000000;
  VTAG_CHROMAKEY_SET     = $80000001;
  VTAG_BITPLANEKEY_CLR   = $80000002;
  VTAG_BITPLANEKEY_SET   = $80000003;
  VTAG_BORDERBLANK_CLR   = $80000004;
  VTAG_BORDERBLANK_SET   = $80000005;
  VTAG_BORDERNOTRANS_CLR = $80000006;
  VTAG_BORDERNOTRANS_SET = $80000007;
  VTAG_CHROMA_PEN_CLR    = $80000008;
  VTAG_CHROMA_PEN_SET    = $80000009;
  VTAG_CHROMA_PLANE_SET  = $8000000A;
  VTAG_ATTACH_CM_SET     = $8000000B;
  VTAG_NEXTBUF_CM        = $8000000C;
  VTAG_BATCH_CM_CLR      = $8000000D;
  VTAG_BATCH_CM_SET      = $8000000E;
  VTAG_NORMAL_DISP_GET   = $8000000F;
  VTAG_NORMAL_DISP_SET   = $80000010;
  VTAG_COERCE_DISP_GET   = $80000011;
  VTAG_COERCE_DISP_SET   = $80000012;
  VTAG_VIEWPORTEXTRA_GET = $80000013;
  VTAG_VIEWPORTEXTRA_SET = $80000014;
  VTAG_CHROMAKEY_GET     = $80000015;
  VTAG_BITPLANEKEY_GET   = $80000016;
  VTAG_BORDERBLANK_GET   = $80000017;
  VTAG_BORDERNOTRANS_GET = $80000018;
  VTAG_CHROMA_PEN_GET    = $80000019;
  VTAG_CHROMA_PLANE_GET  = $8000001A;
  VTAG_ATTACH_CM_GET     = $8000001B;
  VTAG_BATCH_CM_GET      = $8000001C;
  VTAG_BATCH_ITEMS_GET   = $8000001D;
  VTAG_BATCH_ITEMS_SET   = $8000001E;
  VTAG_BATCH_ITEMS_ADD   = $8000001F;
  VTAG_VPMODEID_GET      = $80000020;
  VTAG_VPMODEID_SET      = $80000021;
  VTAG_VPMODEID_CLR      = $80000022;
  VTAG_USERCLIP_GET      = $80000023;
  VTAG_USERCLIP_SET      = $80000024;
  VTAG_USERCLIP_CLR      = $80000025;
// The following tags are V39 specific. They will be ignored (returing error -3) by earlier versions
  VTAG_PF1_BASE_GET             =  $80000026;
  VTAG_PF2_BASE_GET             =  $80000027;
  VTAG_SPEVEN_BASE_GET          =  $80000028;
  VTAG_SPODD_BASE_GET           =  $80000029;
  VTAG_PF1_BASE_SET             =  $8000002a;
  VTAG_PF2_BASE_SET             =  $8000002b;
  VTAG_SPEVEN_BASE_SET          =  $8000002c;
  VTAG_SPODD_BASE_SET           =  $8000002d;
  VTAG_BORDERSPRITE_GET         =  $8000002e;
  VTAG_BORDERSPRITE_SET         =  $8000002f;
  VTAG_BORDERSPRITE_CLR         =  $80000030;
  VTAG_SPRITERESN_SET           =  $80000031;
  VTAG_SPRITERESN_GET           =  $80000032;
  VTAG_PF1_TO_SPRITEPRI_SET     =  $80000033;
  VTAG_PF1_TO_SPRITEPRI_GET     =  $80000034;
  VTAG_PF2_TO_SPRITEPRI_SET     =  $80000035;
  VTAG_PF2_TO_SPRITEPRI_GET     =  $80000036;
  VTAG_IMMEDIATE                =  $80000037;
  VTAG_FULLPALETTE_SET          =  $80000038;
  VTAG_FULLPALETTE_GET          =  $80000039;
  VTAG_FULLPALETTE_CLR          =  $8000003A;
  VTAG_DEFSPRITERESN_SET        =  $8000003B;
  VTAG_DEFSPRITERESN_GET        =  $8000003C;

{ all the following tags follow the new, rational standard for videocontrol tags:
  VC_xxx,state         set the state of attribute 'xxx' to value 'state'
  VC_xxx_QUERY,&var    get the state of attribute 'xxx' and store it into the longword
                       pointed to by &var.
  The following are new for V40:}
  VC_IntermediateCLUpdate       =  $80000080; // default = True. When set graphics will update the intermediate copper
  VC_IntermediateCLUpdate_Query =  $80000081; // lists on color changes, etc. When false, it won't, and will be faster.
  VC_NoColorPaletteLoad         =  $80000082; // default = False. When set, graphics will only load color 0
  VC_NoColorPaletteLoad_Query   =  $80000083; // for this ViewPort, and so the ViewPort's colors will come from the previous ViewPort's.
                                              // NB - Using this tag and VTAG_FULLPALETTE_SET together is undefined.
  VC_DUALPF_Disable             =  $80000084; // default = False. When this flag is set, the dual-pf bit
  VC_DUALPF_Disable_Query       =  $80000085; // in Dual-Playfield screens will be turned off. Even bitplanes
                                              // will still come from the first BitMap and odd bitplanes
                                              // from the second BitMap, and both R[xy]Offsets will be
                                              // considered. This can be used (with appropriate palette
                                              // selection) for cross-fades between differently scrolling
                                              // images.
                                              // When this flag is turned on, colors will be loaded for
                                              // the viewport as if it were a single viewport of depth
                                              // depth1+depth2
const
  SPRITE_ATTACHED     = $80;

type
  PPSimpleSprite = ^PSimpleSprite;
  PSimpleSprite = ^TSimpleSprite;
  TSimpleSprite = record
    PosCtlData: PWord;
    Height: Word;
    x, y: Word;   // current position
    Num: Word;
  end;

  PExtSprite = ^TExtSprite;
  TExtSprite = record
    es_SimpleSprite: TSimpleSprite;         { conventional simple sprite structure }
    es_WordWidth: Word;                 { graphics use only, subject to change }
    es_Flags: Word;                 { graphics use only, subject to change }
    es_Bitmap: PBitmap;  // Actual image data.
  end;

const
// Tags for AllocSpriteData()
  SPRITEA_Width          = $81000000;
  SPRITEA_XReplication   = $81000002;
  SPRITEA_YReplication   = $81000004;
  SPRITEA_OutputHeight   = $81000006;
  SPRITEA_Attached       = $81000008;
  SPRITEA_OldDataFormat  = $8100000a; // MUST pass in outputheight if using this tag
// Tags for GetExtSprite()
  GSTAG_SPRITE_NUM       = $82000020;
  GSTAG_ATTACHED         = $82000022;
  GSTAG_SOFTSPRITE       = $82000024;
// Tags valid for either GetExtSprite or ChangeExtSprite
  GSTAG_SCANDOUBLED      = $83000000; // request "NTSC-Like" height if possible.

type
// BitScaleArgs structure used by BitMapScale()
  PBitScaleArgs = ^TBitScaleArgs;
  TBitScaleArgs = record
    bsa_SrcX, bsa_SrcY,             // source origin
    bsa_SrcWidth, bsa_SrcHeight,    // source size
    bsa_XSrcFactor, bsa_YSrcFactor, // scale factor denominators
    bsa_DestX, bsa_DestY,           // destination origin
    bsa_DestWidth, bsa_DestHeight,  // destination size result
    bsa_XDestFactor,                // scale factor numerators
    bsa_YDestFactor: Word;
    bsa_SrcBitMap,                  // source BitMap
    bsa_DestBitMap: PBitMap;        // destination BitMap
    bsa_Flags: ULONG;               // reserved.  Must be zero!
    bsa_XDDA, bsa_YDDA: Word;       // reserved
    bsa_Reserved1,
    bsa_Reserved2: Longint;
  end;

const
// tag definitions for GetRPAttr, SetRPAttr
  RPTAG_Font       = $80000000;  // get/set font
  RPTAG_APen       = $80000002;  // get/set apen
  RPTAG_BPen       = $80000003;  // get/set bpen
  RPTAG_DrMd       = $80000004;  // get/set draw mode
  RPTAG_OutlinePen = $80000005;  // get/set outline pen. corrected case.
  RPTAG_WriteMask  = $80000006;  // get/set WriteMask
  RPTAG_MaxPen     = $80000007;  // get/set maxpen
  RPTAG_DrawBounds = $80000008;  // get only rastport draw bounds. pass @rect
// Extensions taken over from MorphOS
  RPTAG_PenMode    = $80000080;
  RPTAG_FgColor    = $80000081;
  RPTAG_BgColor    = $80000082;

// Extensions invented by AROS
  RPTAG_PatternOriginX     = $800000C0; // SmallInt
  RPTAG_PatternOriginY     = $800000C1; // SmallInt
  RPTAG_ClipRectangle      = $800000C2; // PRectangle Clones PRectangle.
  RPTAG_ClipRectangleFlags = $800000C3; // LongWord
  RPTAG_RemapColorFonts    = $800000C4; // LongBool


// Flags for ClipRectangleFlags
  RPCRF_RELRIGHT  = $01; // ClipRectangle.MaxX is relative to right of layer/bitmap
  RPCRF_RELBOTTOM = $02; // ClipRectangle.MaxY is relative to bottom of layer/bitmap
  RPCRF_VALID     = $04; // private

type
  PGfxBase = ^tGfxBase;
  TGfxBase = record
    LibNode: TLibrary;

    ActiView: PView;      // ViewPtr
    CopInit: PCopInit;    // ptr to copper start up list
    Cia: PLongInt;        // for 8520 resource use
    blitter: PLongInt;    // for future blitter resource use
    LOFlist: PWord;
    SHFlist: PWord;
    blthd,
    blttl: PBltNode;
    bsblthd,
    bsblttl: PBltNode;
    vbsrv,
    timsrv,
    bltsrv: TInterrupt;

    TextFonts: TList;    // Fonts
    DefaultFont: PTextFont;

    Modes: Word;              // copy of current first bplcon0
    VBlank: Shortint;
    Debug: Shortint;
    BeamSync: SmallInt;
    system_bplcon0: SmallInt; // it is ored into each bplcon0 for display
    SpriteReserved: Byte;
    bytereserved: Byte;
    Flags: Word;
    BlitLock: SmallInt;
    BlitNest: SmallInt;

    BlitWaitQ: TList;
    BlitOwner: PTask;
    TOF_WaitQ: tList;

    DisplayFlags: Word;            // NTSC PAL GENLOC etc.  Display flags are determined at power on
    SimpleSprites: PPSimpleSprite; // SimpleSpritePtr ptr

    MaxDisplayRow: Word;       // hardware stuff, do not use
    MaxDisplayColumn: Word;    // hardware stuff, do not use
    NormalDisplayRows: Word;
    NormalDisplayColumns: Word;
     // the following are for standard non interlace, 1/2 wb width
    NormalDPMX: Word;        // Dots per meter on display
    NormalDPMY: Word;        // Dots per meter on display

    LastChanceMemory: PSignalSemaphore;

    LCMptr: PWord;
    MicrosPerLine: Word;    // 256 time usec/line
    MinDisplayColumn: Word;
    ChipRevBits0: Byte;
    MemType: Byte;
    crb_reserved:  array[0..3] of Byte;
    monitor_id: Word;            // normally 0
    hedley: array[0..7] of IPTR;
    hedley_sprites: array[0..7] of IPTR;     // sprite ptrs for intuition mouse
    hedley_sprites1: array[0..7] of IPTR;    // sprite ptrs for intuition mouse
    hedley_count: SmallInt;
    hedley_flags: Word;
    hedley_tmp: SmallInt;

    hash_table: ^IPTR;          // Hashtable used for GfxAssociate() and GfxLookup() (private!)
    current_tot_rows: Word;
    current_tot_cclks: Word;
    hedley_hint: Byte;
    hedley_hint2: Byte;
    nreserved: array[0..3] of ULONG;
    a2024_sync_raster: PLongWord;
    control_delta_pal: Word;
    control_delta_ntsc: Word;

    Current_Monitor: PMonitorSpec;          // MonitorSpec used for current display
    MonitorList: TList;                     // List of all MonitorSpecs in the system
    Default_Monitor: PMonitorSpec;          // MonitorSpec of "default.monitor"
    MonitorListSemaphore: PSignalSemaphore; // Semaphore for MonitorList access

    DisplayInfoDataBase: Pointer;           // nil, unused by AROS
    TopLine: Word;
    ActiViewCprSemaphore: pSignalSemaphore; // Semaphore for active view access

    UtilityBase: PUtilityBase;   // for hook AND tag utilities
    ExecBase: PExecBase;         // to link with rom.lib

    bwshifts: PShortInt;
    StrtFetchMasks,
    StopFetchMasks,
    Overrun: PWord;
    RealStops: PSmallInt;
    SpriteWidth,               // current width (in words) of sprites
    SpriteFMode: Word;         // current sprite fmode bits
    SoftSprites,               // bit mask of size change knowledgeable sprites
    arraywidth: ShortInt;
    DefaultSpriteWidth: Word;  // what width intuition wants
    SprMoveDisable: ShortInt;
    WantChips,
    BoardMemType,
    Bugs: Byte;
    gb_LayersBase: PLongWord; // layers.library base
    ColorMask: ULONG;
    IVector,
    IData: APTR;
    SpecialCounter: ULONG;    // special for double buffering
    DBList: APTR;
    MonitorFlags: Word;
    ScanDoubledSprites,
    BP3Bits: Byte;

    MonitorVBlank: TAnalogSignalInterval;
    Natural_Monitor: PMonitorSpec; // Default MonitorSpec for view without explicit MonitorSpec in ViewExtra

    ProgData: APTR;  // nil not used in AROS
    ExtSprites: Byte;
    pad3: Byte;
    GfxFlags: Word;
    VBCounter: ULONG;

    HashTableSemaphore: PSignalSemaphore;  // Semaphore for hash_table access, private in fact
    case boolean of
      true: ( ChunkyToPlanarPtr: PLongWord;);  // HWEmul[0];
      false: (HWEmul: array[0..8] of PLongWord;);
  end;

type
// for SetDisplayDriverCallback
  TDriverNotifyFunc = function (Obj: APTR; Add: LongBool; userdata: APTR): APTR; cdecl;

const
//DisplayFlags
  // Specify some system-wide options for Amiga(tm) chipset
  NTSC             = 1 shl 0; // Default mode is NTSC
  GENLOC           = 1 shl 1; // Genlock is in use
  PAL              = 1 shl 2; // Default mode is PAL
  TODA_SAFE        = 1 shl 3;
  REALLY_PAL       = 1 shl 4;
  LPEN_SWAP_FRAMES = 1 shl 5; // When light pen is being used on interlaced screens, swap even and odd frames
// bits defs for ChipRevBits
  GFXB_BIG_BLITS = 0 ;
  GFXB_HR_AGNUS  = 0 ;
  GFXB_HR_DENISE = 1 ;
  GFXB_AA_ALICE  = 2 ;
  GFXB_AA_LISA   = 3 ;
  GFXB_AA_MLISA  = 4 ;      { internal use only. }
// Bit Values for ChipRevBits
  GFXF_BIG_BLITS = 1 shl GFXB_BIG_BLITS;
  GFXF_HR_AGNUS  = 1 shl GFXB_HR_AGNUS;
  GFXF_HR_DENISE = 1 shl GFXB_HR_DENISE;
  GFXF_AA_ALICE  = 1 shl GFXB_AA_ALICE;
  GFXF_AA_LISA   = 1 shl GFXB_AA_LISA;
  GFXF_AA_MLISA  = 1 shl GFXB_AA_MLISA;      { internal use only }

//Pass ONE of these to SetChipRev()
  SETCHIPREV_A    = GFXF_HR_AGNUS;
  SETCHIPREV_ECS  = GFXF_HR_AGNUS or GFXF_HR_DENISE;
  SETCHIPREV_AA   = GFXF_AA_ALICE or GFXF_AA_LISA or SETCHIPREV_ECS;
  SETCHIPREV_BEST = $ffffffff;

// memory type
  BUS_16          = 0;
  BUS_32          = 1;
  NML_CAS         = 0;
  DBL_CAS         = 2;

  BANDWIDTH_1X    = BUS_16 or NML_CAS;
  BANDWIDTH_2XNML = BUS_32;
  BANDWIDTH_2XDBL = DBL_CAS;
  BANDWIDTH_4X    = BUS_32 or DBL_CAS;

  BLITMSG_FAULT = 4;

{ GfxFlags (private) }
  NEW_DATABASE   = 1;

  GRAPHICSNAME: PChar  = 'graphics.library';

var
  GfxBase: PGfxBase;



function BltBitMap(const SrcBitMap: PBitMap; xSrc, ySrc: LongInt; DestBitMap: PBitMap; xDest, yDest, xSize, ySize: LongInt; MinTerm : LongWord; Mask: LongWord; TempA: TPlanePtr): LongInt; syscall GfxBase 5;
procedure BltTemplate(const Source: TPlanePtr; xSrc, srcMod: LongInt; DestRP: PRastPort; xDest, yDest, xSize, ySize: LongInt); syscall GfxBase 6;
procedure ClearEOL(Rp: PRastPort); syscall GfxBase 7;
procedure ClearScreen(Rp: PRastPort); syscall GfxBase 8;
function TextLength(Rp: PRastPort; const string_: STRPTR; Count: LongWord): SmallInt; syscall GfxBase 9;
procedure GfxText(Rp: PRastPort; const String_: STRPTR; Count: LongWord); syscall GfxBase 10;
procedure SetFont(Rp: PRastPort; TextFont: PTextFont); syscall GfxBase 11;
function OpenFont(TextAttr: PTextAttr): PTextFont; syscall GfxBase 12;
procedure CloseFont(TextFont: PTextFont); syscall GfxBase 13;
function AskSoftStyle(Rp: PRastPort): LongWord; syscall GfxBase 14;
function SetSoftStyle(Rp: PRastPort; Style: LongWord; Enable: LongWord): LongWord; syscall GfxBase 15;
procedure AddBob(Bob: PBob; Rp: PRastPort); syscall GfxBase 16;
procedure AddVSprite(VSprite: PVSprite; Rp: PRastPort); syscall GfxBase 17;
procedure DoCollision(Rp: PRastPort); syscall GfxBase 18;
procedure DrawGList(Rp: PRastPort; Vp: PViewPort); syscall GfxBase 19;
procedure InitGels(Head: PVSprite; Tail: PVSprite; GelsInfo: PGelsInfo); syscall GfxBase 20;
procedure InitMasks(VSprite: PVSprite); syscall GfxBase 21;
procedure RemIBob(Bob: PBob; Rp: PRastPort; Vp: PViewPort); syscall GfxBase 22;
procedure RemVSprite(VSprite: PVSprite); syscall GfxBase 23;
procedure SetCollision(Num: LongWord; Routine: TProcedure; GInfo: PGelsInfo); syscall GfxBase 24;
procedure SortGList(Rp: PRastPort); syscall GfxBase 25;
procedure AddAnimOb(AnOb: PAnimOb; AnKey: PPAnimOb; Rp: PRastPort); syscall GfxBase 26;
procedure Animate(AnKey: PPAnimOb; Rp: PRastPort); syscall GfxBase 27;
function GetGBuffers(AnOb: PAnimOb; Rp: PRastPort; Db: LongBool): LongBool; syscall GfxBase 28;
procedure InitGMasks(AnOb: PAnimOb); syscall GfxBase 29;
procedure DrawEllipse(Rp: PRastPort; xCenter, yCenter, a, b: LongInt); syscall GfxBase 30;
function AreaEllipse(Rp: PRastPort; xCenter, yCenter, a, b: SmallInt): LongWord; syscall GfxBase 31;
procedure LoadRGB4(Vp: PViewPort; Colors: PWord; Count: LongInt); syscall GfxBase 32;
procedure InitRastPort(Rp: PRastPort); syscall GfxBase 33;
procedure InitVPort(Vp: PViewPort); syscall GfxBase 34;
function MrgCop(View: PView): LongWord; syscall GfxBase 35;
function MakeVPort(View: PView; ViewPort: PViewPort): LongWord; syscall GfxBase 36;
procedure LoadView(View: PView); syscall GfxBase 37;
procedure WaitBlit; syscall GfxBase 38; unimplemented;
procedure SetRast(Rp: PRastPort; Pen: LongWord); syscall GfxBase 39;
procedure GfxMove(Rp: PRastPort; x, y: SmallInt); syscall GfxBase 40;
procedure Draw(Rp: PRastPort; x, y: LongInt); syscall GfxBase 41;
function AreaMove(Rp: PRastPort; x, y: SmallInt): LongWord; syscall GfxBase 42;
function AreaDraw(Rp: PRastPort; x, y: SmallInt): LongWord; syscall GfxBase 43;
function AreaEnd(Rp: PRastPort): LongInt; syscall GfxBase 44;
procedure WaitTOF; syscall GfxBase 45;
procedure QBlit(Blit: PBltNode); syscall GfxBase 46;
procedure InitArea(AreaInfo: PAreaInfo; Buffer: Pointer; MaxVectors: SmallInt); syscall GfxBase 47;
procedure SetRGB4(Vp: PViewPort; n, r, g, b: LongWord); syscall GfxBase 48;
procedure QBSBlit(Blit: PBltNode); syscall GfxBase 49;
procedure BltClear(MemBlock: Pointer; ByteCount: LongWord; Flags: LongWord); syscall GfxBase 50; deprecated;
procedure RectFill(Rp: PRastPort; xMin, yMin, xMax, yMax : LongInt); syscall GfxBase 51;
procedure BltPattern(Rp: PRastPort; mask: TPlanePtr; xMin, yMin, xMax, yMax: LongInt; ByteCnt: LongWord); syscall GfxBase 52;
function ReadPixel(Rp: PRastPort; x, y: LongInt): LongInt; syscall GfxBase 53;
function WritePixel(Rp: PRastPort; x, y: LongInt): LongInt; syscall GfxBase 54;
function Flood(Rp: PRastPort; Mode: LongWord; x, y: LongInt): LongBool; syscall GfxBase 55;
procedure PolyDraw(Rp: PRastPort; Count: LongInt; PolyTable: PSmallInt); syscall GfxBase 56;
procedure SetAPen(Rp: PRastPort; Pen: LongWord); syscall GfxBase 57;
procedure SetBPen(Rp: PRastPort; Pen: LongWord); syscall GfxBase 58;
procedure SetDrMd(Rp: PRastPort; DrawMode: LongWord); syscall GfxBase 59;
procedure InitView(View: PView); syscall GfxBase 60;
procedure CBump(CopList: PUCopList); syscall GfxBase 61;
procedure CMove(CopList: PUCopList; Reg: Pointer; Value: LongInt); syscall GfxBase 62;
procedure CWait(CopList: PUCopList; V: SmallInt; H: SmallInt); syscall GfxBase 63;
function VBeamPos: LongInt; syscall GfxBase 64; platform;
procedure InitBitMap(Bm: PBitMap; Depth: ShortInt; Width, Height: Word); syscall GfxBase 65;
procedure ScrollRaster(Rp: PRastPort; dx, dy, xMin, yMin, xMax, yMax: LongInt); syscall GfxBase 66;
procedure WaitBOVP(Vp: PViewPort); syscall GfxBase 67; unimplemented;
function GetSprite(Sprite: PSimpleSprite; Pick: SmallInt): SmallInt; syscall GfxBase 68;
procedure FreeSprite(Pick: SmallInt); syscall GfxBase 69;
procedure ChangeSprite(Vp: PViewPort; s: PSimpleSprite; NewData: Pointer); syscall GfxBase 70; unimplemented;
procedure MoveSprite(Vp: PViewPort; Sprite: PSimpleSprite; x, y: SmallInt); syscall GfxBase 71;
procedure LockLayerRom(l: PLayer); syscall GfxBase 72;
procedure UnlockLayerRom(l: PLayer); syscall GfxBase 73;
procedure SyncSBitMap(l: PLayer); syscall GfxBase 74;
procedure CopySBitMap(l: PLayer); syscall GfxBase 75;
procedure OwnBlitter; syscall GfxBase 76;
procedure DisownBlitter; syscall GfxBase 77;
function InitTmpRas(TmpRas: PTmpRas; Buffer: Pointer; Size: LongWord): PTmpRas; syscall GfxBase 78;
procedure AskFont(Rp: PRastPort; TextAttr: PTextAttr); syscall GfxBase 79;
procedure AddFont(TextFont: PTextFont); syscall GfxBase 80;
procedure RemFont(TextFont: PTextFont); syscall GfxBase 81;
function AllocRaster(Width, Height: LongWord): TPlanePtr; syscall GfxBase 82;
procedure FreeRaster(p: TPlanePtr; Width, Height: LongWord); syscall GfxBase 83;
procedure AndRectRegion(Reg: PRegion; Rect :PRectangle); syscall GfxBase 84;
function OrRectRegion(Reg: PRegion; Rect: PRectangle): LongBool; syscall GfxBase 85;
function NewRegion: PRegion; syscall GfxBase 86;
function ClearRectRegion(Reg: PRegion; Rect: PRectangle): LongBool; syscall GfxBase 87;
procedure ClearRegion(Region: PRegion); syscall GfxBase 88;
procedure DisposeRegion(Region: PRegion); syscall GfxBase 89;
procedure FreeVPortCopLists(Vp: PViewPort); syscall GfxBase 90;
procedure FreeCopList(CopList: PCopList); syscall GfxBase 91;
procedure ClipBlit(SrcRP: PRastPort; xSrc, ySrc: LongInt; DestRP: PRastPort; xDest, yDest, xSize, ySize: LongInt; MinTerm: Byte); syscall GfxBase 92;
function XorRectRegion(Reg: PRegion; Rect: PRectangle): LongBool; syscall GfxBase 93;
procedure FreeCprList(CprList: PCprList); syscall GfxBase 94;
function GetColorMap(Entries: LongWord): PColorMap; syscall GfxBase 95;
procedure FreeColorMap(ColorMap: PColorMap); syscall GfxBase 96;
function GetRGB4(ColorMap: PColorMap; Entry: LongInt): LongWord; syscall GfxBase 97;
procedure ScrollVPort(Vp: PViewPort); syscall GfxBase 98;
function UCopperListInit(Ucl: PUCopList; n: SmallInt): PCopList; syscall GfxBase 99;
procedure FreeGBuffers(AnOb: PAnimOb; Rp: PRastPort; db: LongBool); syscall GfxBase 100;
procedure BltBitMapRastPort(const SrcBitMap: PBitMap; xSrc, ySrc: LongInt; DestRP: PRastPort; xDest, yDest, xSize, ySize: LongInt; MinTerm: LongWord); syscall GfxBase 101;
function OrRegionRegion(SrcRegion: PRegion; DestRegion: PRegion): LongBool; syscall GfxBase 102;
function XorRegionRegion(SrcRegion: PRegion; DestRegion: PRegion): LongBool; syscall GfxBase 103;
function AndRegionRegion(SrcRegion: PRegion; DestRegion: PRegion): LongBool; syscall GfxBase 104;
procedure SetRGB4CM(Cm: PColorMap; n: SmallInt; r, g, b: Byte); syscall GfxBase 105;
procedure BltMaskBitMapRastPort(SrcBitMap: PBitMap; xSrc, ySrc: LongInt; DestRP: PRastPort; xDest, yDest, xSize, ySize: LongInt; MinTerm: LongWord; bltMask: TPlanePtr); syscall GfxBase 106;
function AttemptLockLayerRom(l: PLayer): LongBool; syscall GfxBase 109;
function GfxNew(Node_Type: LongWord): PExtendedNode; syscall GfxBase 110;
procedure GfxFree(Node: PExtendedNode); syscall GfxBase 111;
procedure GfxAssociate(Pointer_: Pointer; Node: PExtendedNode); syscall GfxBase 112;
procedure BitMapScale(BitScaleArgs: PBitScaleArgs); syscall GfxBase 113;
function ScalerDiv(Factor: LongWord; Numerator: LongWord; Denominator: LongWord): Word; syscall GfxBase 114;
procedure TextExtent(Rp: PRastPort; const String_: STRPTR; Count: LongWord; TextExtent_: PTextExtent); syscall GfxBase 115;
function TextFit(Rp: PRastPort; const String_: STRPTR; StrLen: LongWord; TextExtent_: PTextExtent; ConstrainingExtent: PTextExtent; StrDirection: LongInt; ConstrainingBitWidth: LongWord; ConstrainingBitHeight: LongWord): LongWord; syscall GfxBase 116;
function GfxLookUp(Pointer_: Pointer): PExtendedNode; syscall GfxBase 117;
function VideoControl(Cm: PColorMap; Tags: PTagItem): LongWord; syscall GfxBase 118; unimplemented;
function OpenMonitor(MonitorName: STRPTR; DisplayID: LongWord): PMonitorSpec; syscall GfxBase 119;
function CloseMonitor(Monitor_Spec: PMonitorSpec): LongInt; syscall GfxBase 120;
function FindDisplayInfo(ID: LongWord): DisplayInfoHandle; syscall GfxBase 121;
function NextDisplayInfo(Last_ID: LongWord): LongWord; syscall GfxBase 122;
function GetDisplayInfoData(Handle: DisplayInfoHandle; Buf: PChar; Size: LongWord; TagID: LongWord; ID: LongWord): LongWord; syscall GfxBase 126;
procedure FontExtent(Font: PTextFont; FontExtent: PTextExtent); syscall GfxBase 127;
function ReadPixelLine8(Rp: PRastPort; xStart, yStart, Width: LongWord; Array_: PByte; TempRP: PRastPort): LongInt; syscall GfxBase 128;
function WritePixelLine8(Rp: PRastPort; xStart, yStart, Width: LongWord; Array_: PByte; TempRP: PRastPort): LongInt; syscall GfxBase 129;
function ReadPixelArray8(Rp: PRastPort; xStart, yStart, xStop, yStop: LongWord; Array_: PByte; TempRp: PRastPort): LongInt; syscall GfxBase 130;
function WritePixelArray8(Rp: PRastPort; xStart, yStart, xStop, yStop: LongWord; Array_: PByte; TempRp: PRastPort): LongInt; syscall GfxBase 131;
function GetVPModeID(Vp: PViewPort): LongWord; syscall GfxBase 132;
function ModeNotAvailable(ModeID: LongWord): LongWord; syscall GfxBase 133;
function WeighTAMatch(ReqTextAttr: PTextAttr; TargetTextAttr: PTextAttr; TargetTags: PTagItem): SmallInt; syscall GfxBase 134;
procedure EraseRect(Rp: PRastPort; xMin, yMin, xMax, yMax: LongInt); syscall GfxBase 135;
function ExtendFont(Font: PTextFont; FontTags: PTagItem): LongWord; syscall GfxBase 136;
procedure StripFont(Font: PTextFont); syscall GfxBase 137;
function CalcIVG(View: PView; ViewPort: PViewPort): Word; syscall GfxBase 138; unimplemented;
function AttachPalExtra(Cm: PColorMap; Vp: PViewPort): LongInt; syscall GfxBase 139;
function ObtainBestPenA(Cm: PColorMap; r, g, b: LongWord; Tags: PTagItem): LongInt; syscall GfxBase 140;
procedure SetRGB32(Vp: PViewPort; n, r, g, b : LongWord); syscall GfxBase 142;
function GetAPen(Rp: PRastPort): LongWord; syscall GfxBase 143;
function GetBPen(Rp: PRastPort): LongWord; syscall GfxBase 144;
function GetDrMd(Rp: PRastPort): LongWord; syscall GfxBase 145;
function GetOutlinePen(Rp: PRastPort): LongWord; syscall GfxBase 146;
procedure LoadRGB32(Vp: PViewPort; const Table: PLongWord); syscall GfxBase 147;
function SetChipRev(ChipRev: LongWord): LongWord; syscall GfxBase 148; platform;
procedure SetABPenDrMd(Rp: PRastPort; APen: LongWord; BPen: LongWord; DrawMode: LongWord); syscall GfxBase 149;
procedure GetRGB32(Cm: PColorMap; FirstColor: LongWord; NColors: LongWord; Table: PLongWord); syscall GfxBase 150;
function AllocBitMap(Sizex, Sizey, Depth, Flags: LongWord; Friend_Bitmap: PBitMap): PBitMap; syscall GfxBase 153;
procedure FreeBitMap(Bm: PBitMap); syscall GfxBase 154;
function GetExtSpriteA(Sprite: PExtSprite; Tags: PTagItem): LongInt; syscall GfxBase 155;
function CoerceMode(RealViewPort: PViewPort; MonitorID: LongWord; Flags: LongWord): LongWord; syscall GfxBase 156; unimplemented;
procedure ChangeVPBitMap(Vp: PViewPort; Bm: PBitMap; Db: PDBufInfo); syscall GfxBase 157;
procedure ReleasePen(Cm: PColorMap; n: LongWord); syscall GfxBase 158;
function ObtainPen(Cm: PColorMap; n, r, g, b: LongWord; Flags: LongWord): LongInt; syscall GfxBase 159;
function GetBitMapAttr(BitMap: PBitMap; Attribute: LongWord): IPTR; syscall GfxBase 160;
function AllocDBufInfo(Vp: PViewPort): PDBufInfo; syscall GfxBase 161;
procedure FreeDBufInfo(Dbi: PDBufInfo); syscall GfxBase 162;
function SetOutlinePen(Rp: PRastPort; Pen: LongWord): LongWord; syscall GfxBase 163;
function SetWriteMask(Rp: PRastPort; Mask: LongWord): LongWord; syscall GfxBase 164;
procedure SetMaxPen(Rp: PRastPort; MaxPen: LongWord); syscall GfxBase 165;
procedure SetRGB32CM(Cm: PColorMap; n, r, g, b: LongWord); syscall GfxBase 166;
procedure ScrollRasterBF(Rp: PRastPort; dx, dy, xMin, yMin, xMax, yMax: LongInt); syscall GfxBase 167;
function FindColor(Cm: PColorMap; r, g, b, MaxPen: LongWord): LongWord; syscall GfxBase 168;
function AllocSpriteDataA(Bitmap: PBitMap; TagList: PTagItem): PExtSprite; syscall GfxBase 170;
function ChangeExtSpriteA(Vp: PViewPort; Oldsprite: PExtSprite; NewSprite: PExtSprite; Tags: PTagItem): LongInt; syscall GfxBase 171;
procedure FreeSpriteData(ExtSp: PExtSprite); syscall GfxBase 172;
procedure SetRPAttrsA(Rp: PRastPort; Tags: PTagItem); syscall GfxBase 173;
procedure GetRPAttrsA(Rp: PRastPort; Tags: PTagItem); syscall GfxBase 174;
function BestModeIDA(Tags: PTagItem): LongWord; syscall GfxBase 175;
procedure WriteChunkyPixels(Rp: PRastPort; xStart, yStart, xStop, yStop: LongWord; Array_: PByte; BytesPerRow: LongInt); syscall GfxBase 176;

{$ifdef AROS_ABIv0}
procedure AndRectRegionND(Reg: PRegion; Rect: PRectangle); syscall GfxBase 107;
function AndRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 108;
function OrRectRegionND(Reg: PRegion; Rect: PRectangle): PRegion; syscall GfxBase 123;
function ClearRectRegionND(Reg: PRegion; Rect: PRectangle): PRegion; syscall GfxBase 124;
function OrRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 125;
function ClearRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 141;
function XorRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 151;
function XorRectRegionND(Reg: PRegion; Rect: PRectangle): PRegion; syscall GfxBase 152;
function CreateRastPort: PRastPort; syscall GfxBase 177;
function CloneRastPort(Rp: PRastPort): PRastPort; syscall GfxBase 178;
procedure DeinitRastPort(Rp: PRastPort); syscall GfxBase 179;
procedure FreeRastPort(Rp: PRastPort); syscall GfxBase 180;
function AddDisplayDriverA(GfxHidd: APTR; Tags: PTagItem): LongInt; syscall GfxBase 181;
function WritePixels8(Rp: PRastPort; Array_: PByte; Modulo: LongWord; xStart, yStart, xStop, yStop: LongWord; PixLUT: Pointer{PHIDDT_PixelLUT}; Do_Update: LongBool): LongInt; syscall GfxBase 182;
function FillRectPenDrMd(Rp: PRastPort; x1, y1, x2, y2: LongInt; Pix: Pointer{HIDDT_Pixel}; drmd: Pointer{HIDDT_DrawMode}; Do_Update: LongBool): LongInt; syscall GfxBase 183;
function DoRenderFunc(Rp: PRastPort; Src: PPoint; Rr: PRectangle; Render_Func: Pointer; FuncData: APTR; Do_Update: LongBool): LongInt; syscall GfxBase 184;
function DoPixelFunc(Rp: PRastPort; x, y: LongInt; Render_Func: Pointer; FuncData: APTR; Do_Update: LongBool): LongInt; syscall GfxBase 185;
procedure SetDisplayDriverCallback(CallBack: TDriverNotifyFunc; UserData: APTR); syscall GfxBase 186;
function ClearRegionRegion(R1: PRegion; R2: PRegion): LongBool; syscall GfxBase 187;
function CopyRegion(Region: PRegion): PRegion; syscall GfxBase 188;
function AreRegionsEqual(R1: PRegion; R2: PRegion): LongBool; syscall GfxBase 189;
function IsPointInRegion(Reg: PRegion; x, y: SmallInt): LongBool; syscall GfxBase 190;
function ScrollRegion(Region: PRegion; Rect: PRectangle; Dx, Dy: SmallInt): LongBool; syscall GfxBase 191;
function SwapRegions(Region1: PRegion; Region2: PRegion; Intersect: PRectangle): LongBool; syscall GfxBase 192;
function AndRectRect(Rect1: PRectangle; Rect2: PRectangle; Intersect: PRectangle): LongBool; syscall GfxBase 193;
function NewRectRegion(MinX, MinY, MaxX, MaxY: SmallInt): PRegion; syscall GfxBase 194;
function SetRegion(Src: PRegion; Dest: PRegion): LongBool; syscall GfxBase 195;
procedure BltRastPortBitMap(SrcRastPort: PRastPort; xSrc, ySrc: LongInt; DestBitMap: PBitMap; xDest, yDest, xSize, ySize, MinTerm: LongWord); syscall GfxBase 196;
procedure ShowImminentReset; syscall GfxBase 197;
{$endif}

{$ifdef AROS_ABIv1}
function AddDisplayDriverA(GfxHidd: APTR; Tags: PTagItem): LongInt; syscall GfxBase 107;
procedure SetDisplayDriverCallback(CallBack: TDriverNotifyFunc; UserData: APTR); syscall GfxBase 108;
function SetRegion(Src: PRegion; Dest: PRegion): LongBool; syscall GfxBase 181;
function ClearRegionRegion(R1: PRegion; R2: PRegion): LongBool; syscall GfxBase 182;
function AreRegionsEqual(R1: PRegion; R2: PRegion): LongBool; syscall GfxBase 183;
function IsPointInRegion(Reg: PRegion; x, y: SmallInt): LongBool; syscall GfxBase 184;
function ScrollRegion(Region: PRegion; Rect: PRectangle; Dx, Dy: SmallInt): LongBool; syscall GfxBase 185;
procedure AndRectRegionND(Reg: PRegion; Rect: PRectangle); syscall GfxBase 186;
function AndRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 187;
function OrRectRegionND(Reg: PRegion; Rect: PRectangle): PRegion; syscall GfxBase 188;
function OrRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 189;
function XorRectRegionND(Reg: PRegion; Rect: PRectangle): PRegion; syscall GfxBase 190;
function XorRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 191;
function ClearRectRegionND(Reg: PRegion; Rect: PRectangle): PRegion; syscall GfxBase 192;
function ClearRegionRegionND(R1: PRegion; R2: PRegion): PRegion; syscall GfxBase 193;
function WritePixels8(Rp: PRastPort; Array_: PByte; Modulo: LongWord; xStart, yStart, xStop, yStop: LongWord; PixLUT: Pointer{PHIDDT_PixelLUT}; Do_Update: LongBool): LongInt; syscall GfxBase 197;
function FillRectPenDrMd(Rp: PRastPort; x1, y1, x2, y2: LongInt; Pix: Pointer{HIDDT_Pixel}; drmd: Pointer{HIDDT_DrawMode}; Do_Update: LongBool): LongInt; syscall GfxBase 198;
function DoRenderFunc(Rp: PRastPort; Src: PPoint; Rr: PRectangle; Render_Func: Pointer; FuncData: APTR; Do_Update: LongBool): LongInt; syscall GfxBase 199;
function DoPixelFunc(Rp: PRastPort; x, y: LongInt; Render_Func: Pointer; FuncData: APTR; Do_Update: LongBool): LongInt; syscall GfxBase 200;
procedure UpdateBitMap(Bitmap: PBitmap; x, y, Width, Height: LongWord); syscall GfxBase 201;

function CreateRastPort: PRastPort;
function CloneRastPort(Rp: PRastPort): PRastPort;
procedure DeinitRastPort(Rp: PRastPort);
procedure FreeRastPort(Rp: PRastPort);
{$endif}

function BestModeID(const Tags: array of PtrUInt): LongWord;
function AllocSpriteData(Bitmap: PBitMap; const Tags: array of PtrUInt): PExtSprite;
function ChangeExtSprite(Vp: PViewPort; Oldsprite: PExtSprite; NewSprite: PExtSprite; const Tags: array of PtrUInt): LongInt;
function ExtendFontTags(Font: PTextFont; const Tags: array of PtrUInt): LongWord;
function GetExtSprite(Sprite: PExtSprite; const Tags: array of PtrUInt): LongInt;
procedure GetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt);
function ObtainBestPen(Cm: PColorMap; r, g, b: LongWord; const Tags: array of PtrUInt): LongInt;
procedure SetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt);
function VideoControlTags(Cm: PColorMap; const Tags: array of PtrUInt): LongWord; unimplemented;

// gfxmacros

// This one is used for determining optimal offset for blitting into cliprects
function Align_Offset(x: Pointer): Pointer;
function Is_Visible(l: PLayer): Boolean;
procedure InitAnimate(var Animkey: PAnimOb);
procedure RemBob(B: PBob);

function RasSize(w, h: Word): Integer;
function BitmapFlags_are_Extended(f: LongInt): Boolean;
function GetTextFontReplyPort(Font: PTextFont): PMsgPort;

procedure BNDRYOFF(w: PRastPort);

procedure SetAfPt(w: PRastPort; p: Pointer; n: Byte);
procedure SetDrPt(w: PRastPort; p: Word);
procedure SetOPen(w: PRastPort; c: Byte);
procedure SetWrMsk(w: PRastPort; m: Byte);

function SetAOlPen(Rp: PRastPort; Pen: LongWord): LongWord;

procedure DrawCircle(Rp: PRastPort; xCenter, yCenter, r: LongInt);
function AreaCircle(Rp: PRastPort; xCenter, yCenter, r: SmallInt): LongWord;

// Copper helper
function CINIT(c: PUCopList; n: SmallInt): PCopList;
procedure CMOVE1(c: PUCopList; a: Pointer; b: LongInt);
procedure CWAIT1(c: PUCopList; a: SmallInt; b: SmallInt);
procedure CEND(c: PUCopList);

implementation

function BestModeID(const Tags: array of PtrUInt): LongWord; inline;
begin
  BestModeID := BestModeIDA(@Tags);
end;

function AllocSpriteData(Bitmap: PBitMap; const Tags: array of PtrUInt): PExtSprite; inline;
begin
  AllocSpriteData := AllocSpriteDataA(Bitmap, @Tags);
end;

function ChangeExtSprite(Vp: PViewPort; Oldsprite: PExtSprite; NewSprite: PExtSprite; const Tags: array of PtrUInt): LongInt; inline;
begin
  ChangeExtSprite := ChangeExtSpriteA(Vp, Oldsprite, NewSprite, @Tags);
end;

function ExtendFontTags(Font: PTextFont; const Tags: array of PtrUInt): LongWord; inline;
begin
  ExtendFontTags := ExtendFont(Font, @Tags);
end;

function GetExtSprite(Sprite: PExtSprite; const Tags: array of PtrUInt): LongInt; inline;
begin
  GetExtSprite := GetExtSpriteA(Sprite, @Tags);
end;

procedure GetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt); inline;
begin
  GetRPAttrsA(Rp, @Tags);
end;

function ObtainBestPen(Cm: PColorMap; r, g, b: LongWord; const Tags: array of PtrUInt): LongInt; inline;
begin
  ObtainBestPen := ObtainBestPenA(Cm, r, g, b, @Tags);
end;

procedure SetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt); inline;
begin
  SetRPAttrsA(Rp, @Tags);
end;

function VideoControlTags(Cm: PColorMap; const Tags: array of PtrUInt): LongWord; inline;
begin
  {$WARNINGS OFF} // suppress unimplemented Warning
  VideoControlTags := VideoControl(Cm, @Tags);
  {$WARNINGS ON}
end;

function Align_Offset(x: Pointer): Pointer; inline;
begin
  Align_Offset := Pointer(PtrUInt(x) and $0F);
end;

function Is_Visible(l: PLayer): Boolean; inline;
begin
  Is_Visible := l^.Visible <> 0;
end;

procedure InitAnimate(var AnimKey: PAnimOb); inline;
begin
  AnimKey := nil;
end;

procedure RemBob(B: PBob); inline;
begin
  B^.Flags := B^.Flags or BOBSAWAY;
end;

function RasSize(w, h: Word): Integer; inline;
begin
  RasSize := h * (((w + 15) shr 3) and $FFFE);
end;

function BitmapFlags_are_Extended(f: LongInt): Boolean; inline;
begin
  BitmapFlags_are_Extended := (f and BMF_CHECKMASK) = BMF_CHECKVALUE;
end;

function GetTextFontReplyPort(Font: PTextFont): PMsgPort; inline;
var
  tfe: PTextFontExtension;
begin
   tfe := PTextFontExtension(ExtendFont(Font, nil));
   if Assigned(tfe) then
     GetTextFontReplyPort := tfe^.tfe_OrigReplyPort
   else
     GetTextFontReplyPort :=  Font^.tf_Message.mn_ReplyPort;
end;

function SetAOlPen(Rp: PRastPort; Pen: LongWord): LongWord; inline;
begin
  SetAOlPen := SetOutlinePen(Rp, Pen);
end;

procedure BNDRYOFF (w: PRastPort); inline;
begin
  w^.Flags := w^.Flags and (not AREAOUTLINE);
end;

procedure SetAfPt(w: PRastPort; p: Pointer; n: Byte); inline;
begin
  w^.AreaPtrn := p;
  w^.AreaPtSz := n;
end;

procedure SetDrPt(w: PRastPort; p: Word); inline;
begin
  w^.LinePtrn := p;
  w^.Flags := w^.Flags or (FRST_DOT or $10);
  w^.linpatcnt := 15;
end;

procedure SetOPen(w: PRastPort; c: Byte); inline;
begin
  w^.AOlPen := c;
  w^.Flags := w^.Flags or AREAOUTLINE;
end;

{ This function is fine, but FOR OS39 the SetWriteMask() gfx function
  should be prefered because it SHOULD operate WITH gfx boards as well.
  At least I hope it does.... }
procedure SetWrMsk(w: PRastPort; m: Byte); inline;
begin
  w^.Mask := m;
end;

procedure DrawCircle(Rp: PRastPort; xCenter, yCenter, r: LongInt); inline;
begin
  DrawEllipse(Rp, xCenter, yCenter, r, r);
end;

function AreaCircle(Rp: PRastPort; xCenter, yCenter, r: SmallInt): LongWord; inline;
begin
  AreaCircle := AreaEllipse(Rp, xCenter, yCenter, r, r);
end;

function CINIT(c: PUCopList; n: SmallInt): PCopList; inline;
begin
  CINIT := UCopperListInit(c, n);
end;

procedure CMOVE1(c: PUCopList; a: Pointer; b: LongInt);
begin
  CMove(c, a, b);
  CBump(c);
end;

procedure CWait1(c: PUCopList; a: SmallInt; b: SmallInt); inline;
begin
  CWait(c, a, b);
  CBump(c);
end;

procedure CEND(c: PUCopList); inline;
begin
  CWAIT(c, 10000, 255);
end;
{$ifdef AROS_ABIv1}
function CreateRastPort: PRastPort;
var
  RP: PRastPort;
begin
  RP := AllocMem(SizeOf(TRastPort));
  InitRastPort(RP);
  CreateRastPort := RP;
end;

function CloneRastPort(Rp: PRastPort): PRastPort;
var
  NRP: PRastPort;
begin
  NRP := AllocMem(SizeOf(TRastPort));
  Move(Rp^, NRP^, SizeOf(TRastPort));
  CloneRastPort := NRP;
end;

procedure DeinitRastPort(Rp: PRastPort);
begin
  // nothing to do;
end;

procedure FreeRastPort(Rp: PRastPort);
begin
  if Assigned(Rp) then
    FreeMem(Rp);
end;
{$endif}
initialization
  GfxBase := PGfxBase(OpenLibrary(GRAPHICSNAME, 36));
finalization
  CloseLibrary(PLibrary(GfxBase));
end.






