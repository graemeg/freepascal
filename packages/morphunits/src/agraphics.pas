{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    graphics.library interface unit for MorphOS/PowerPC

    Based on work of Nils Sjoholm member of the Amiga RTL
    development team.

    MorphOS port was done on a free Pegasos II/G4 machine
    provided by Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}

unit agraphics;

interface

uses
  exec, hardware, utility;

const
  BITSET = $8000;
  BITCLR = 0;

type
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

  PPoint = ^TPoint;
  TPoint = record
    x, y: SmallInt;
  end;

  TPlanePtr = PByte;

  PBitMap = ^TBitMap;
  TBitMap = record
    BytesPerRow: Word;
    Rows: Word;
    Flags: Byte;
    Depth: Byte;
    Pad: Word;
    Planes: array[0..7] of TPlanePtr;
  end;

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

// flags for AllocBitMap, etc.
const
  BMB_CLEAR       = 0;
  BMB_DISPLAYABLE = 1;
  BMB_INTERLEAVED = 2;
  BMB_STANDARD    = 3;
  BMB_MINPLANES   = 4;

  BMF_CLEAR       = 1 shl BMB_CLEAR;
  BMF_DISPLAYABLE = 1 shl BMB_DISPLAYABLE;
  BMF_INTERLEAVED = 1 shl BMB_INTERLEAVED;
  BMF_STANDARD    = 1 shl BMB_STANDARD;
  BMF_MINPLANES   = 1 shl BMB_MINPLANES;

// the following IDs are for GetBitMapAttr() *}
  BMA_HEIGHT = 0;
  BMA_DEPTH  = 4;
  BMA_WIDTH  = 8;
  BMA_FLAGS  = 12;

{ structures used by and constructed by windowlib.a }
{ understood by rom software }
type
  PClipRect = ^TClipRect;
  PLayer = ^TLayer;
  PRastPort = ^TRastPort;
  PCopList = ^TCopList;
  PViewPort = ^TViewPort;
  PColorMap = ^TColorMap;

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

  TClipRect = record
    Next    : PClipRect;  // roms used to find next ClipRect
    prev    : PClipRect;  // ignored by roms, used by windowlib
    lobs    : PLayer;     // ignored by roms, used by windowlib
    BitMap  : PBitMap;
    Bounds  : TRectangle; // set up by windowlib, used by roms
    _p1,
    _p2     : APTR;       // system reserved
    reserved: LongInt;    // system use
  end;
  PLayer_Info = ^TLayer_Info;
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
    PrivateReserve4: APTR;

    Flags: Word;               // LIFLG_SUPPORTS_OFFSCREEN_LAYERS
    fatten_count: ShortInt;
    LockLayersCount: ShortInt;
    PrivateReserve5: SmallInt;
    BlankHook: APTR;
    LayerInfo_extra: APTR;
  end;


  TLayer = record
    Front,
    Back            : PLayer;       // ignored by roms
    ClipRect        : PClipRect;    // read by roms to find first cliprect
    Rp              : PRastPort;    // ignored by roms, I hope
    Bounds          : TRectangle;   // ignored by roms
    Reserved        : array[0..3] of Byte;
    Priority        : Word;         // system use only
    Flags           : Word;         // obscured ?, Virtual BitMap?
    SuperBitMap     : PBitMap;
    SuperClipRect   : PClipRect;    // super bitmap cliprects if VBitMap != 0  else damage cliprect list for refresh
    Window          : APTR;         // reserved for user interface use
    Scroll_X,
    Scroll_Y        : Word;
    Cr,
    Cr2,
    CrNew           : PClipRect;    // used by dedice
    SuperSaveClipRects: PClipRect;  // preallocated cr's
    _ClipRects      : PClipRect;    // system use during refresh
    LayerInfo       : Pointer;      // PLayer_Info points to head of the list
    Lock            : TSignalSemaphore;
    BackFill        : PHook;
    Reserved1       : LongWord;
    ClipRegion      : PRegion;
    SaveClipRects   : PRegion;      // used to back out when in trouble
    Width,
    Height          : SmallInt;
    Reserved2       : array[0..17] of Byte;
    { this must stay here }
    DamageList      : PRegion;      // list of rectangles to refresh through
  end;

{***** TextAttr node, matches text attributes in RastPort *********}
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
    tta_Tags: PTagItem; // TTextAttr specific extension -> extended attributes
  end;

{***** TextFonts node *********************************************}
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

{ Note: The combination VWaitAddr and HWaitAddr replace a three way
        union in C.  The three possibilities are:

        nxtList : CopListPtr;  or

        VWaitPos : Longint;
        HWaitPos : Longint;  or

        DestAddr : Longint;
        DestData : Longint;
}
  PCopIns = ^TCopIns;
  PUCopList = ^TUCopList;
  PVSprite = ^TVSprite;
  PPaletteExtra = ^TPaletteExtra;

  TCopIns = record
    OpCode: SmallInt; // 0 = move, 1 = wait
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

  // structure of cprlist that points to list that hardware actually executes
  PCprList = ^TCprList;
  TCprList = record
    Next: PCprList;
    Start: PWord;         // start of copper list
    MaxCount: SmallInt;  // number of long instructions
  end;


  TUCopList = record
    Next: PUCopList;
    FirstCopList: PCopList; // head node of this copper list
    CopList: PCopList;      // node in use
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

  // a structure to contain the 16 collision procedure addresses
  PCollTable = ^TCollTable;
  TCollTable = array[0..15] of Pointer;

{ unoptimized for 32bit alignment of pointers }

  PGelsInfo = ^TGelsInfo;
  TGelsInfo = record
    sprRsrvd: ShortInt;      // flag of which sprites to reserve from vsprite system
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
    longreserved: array[0..1] of LongWord;
    wordreserved: array[0..6] of Word; // used to be a node
    reserved: array[0..7] of Byte;     // for future use
  end;

  PRasInfo = ^TRasInfo;
  TRasInfo = record     // used by callers to and InitDspC()
    Next: PRasInfo;     // Pointer to a next playfield (if there's more than one)
    BitMap: PBitMap;    // Actual data to display
    RxOffset,           // Offset of the playfield relative to ViewPort
    RyOffset: SmallInt; // scroll offsets in this BitMap (So that different playfields may be shifted against each other)
  end;


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
    ratiov: LongInt;
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

  PView = ^TView;
  TView = record
    ViewPort: PViewPort;  // ViewPortPtr
    LOFCprList: PCprList; // used for interlaced and noninterlaced
    SHFCprList: PCprList; // only used during interlace
    DyOffset,
    DxOffset: SmallInt;   // for complete View positioning offsets are +- adjustments to standard #s
    Modes: Word;          // such as INTERLACE, GENLOC
  end;

// these structures are obtained via GfxNew and disposed by GfxFree

// Additional data for Amiga(tm) chipset. Not used by other hardware.
// these structures are obtained via GfxNew and disposed by GfxFree
  PViewExtra = ^TViewExtra;
  TViewExtra = record
    n: TExtendedNode;      // Common Header
    View: PView;           // View it relates to
    Monitor: PMonitorSpec; // Monitor used for displaying this View
    TopLine: Word;
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
    cop2ptr: LongWord;       // private
  end;

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

{ UserStuff definitions
  the user can define these to be a single variable or a sub-structure
  if undefined by the user, the system turns these into innocuous variables
  see the manual for a thorough definition of the UserStuff definitions }

  TVUserStuff = SmallInt; // Sprite user stuff
  TBUserStuff = SmallInt; // Bob user stuff
  TAUserStuff = SmallInt; // AnimOb user stuff

{ GEL draw list constructed in the order the Bobs are actually drawn, then
  list is copied to clear list
  must be here in VSprite for system boundary detection
  the VSprite positions are defined in (y,x) order to make sorting
  sorting easier, since (y,x) as a long Longint}

  PBob = ^TBob;
  PAnimComp = ^TAnimComp;
  PAnimOb = ^TAnimOb;
  TVSprite = record
   // SYSTEM VARIABLES
    NextVSprite: PVSprite; // GEL linked list forward/backward pointers sorted by y,x value
    PrevVSprite: PVSprite;
    DrawPath: PVSprite;    // pointer of overlay drawing
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
  B2NORM      = 0;
  B2SWAP      = 1;
  B2BOBBER    = 2;

const
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

{***** Text Tags **************************************************}
const
  TA_DeviceDPI = TAG_USER + 1; // Tag value is Point union: Hi Longint XDPI, Lo Longint YDPI
  // these tags override struct TextAttr fields
  TA_Name          = TAG_USER+100;
  TA_YSize         = TAG_USER+101;

  TA_Underlined    = TAG_USER+102;
  TA_Bold          = TAG_USER+103;
  TA_Italic        = TAG_USER+104;
  TA_Extended      = TAG_USER+105;
  TA_ColorFont     = TAG_USER+106;

  TA_ROMFont       = TAG_USER+107;
  TA_DiskFont      = TAG_USER+108;
  TA_RevPath       = TAG_USER+109;
  TA_TallDot       = TAG_USER+110;
  TA_WideDot       = TAG_USER+111;
  TA_Proportional  = TAG_USER+112;
  TA_Designed      = TAG_USER+113;

  MAXFONTMATCHWEIGHT = 32767; // perfect match from WeighTAMatch

{----- tfe_Flags0 (partial definition) ----------------------------}
const
  TE0B_NOREMFONT = 0;     // disallow RemFont for this font
  TE0F_NOREMFONT = $01;

{***** ColorTextFont node *****************************************}
{----- ctf_Flags --------------------------------------------------}
  CT_COLORMASK = $000F; // mask to get to following color styles
  CT_COLORFONT = $0001; // color map contains designer's colors }
  CT_GREYFONT  = $0002; // color map describes even-stepped brightnesses from low to high
  CT_ANTIALIAS = $0004; // zero background thru fully saturated char

  CTB_MAPCOLOR = 0;                  // map ctf_FgColor to the rp_FgPen IF it's }
  CTF_MAPCOLOR = 1 shl CTB_MAPCOLOR; // is a valid color within ctf_Low..ctf_High

// graphics copper list instruction definitions
const
  COPPER_MOVE = 0; // pseude opcode for move #XXXX,dir
  COPPER_WAIT = 1; // pseudo opcode for wait y,x
  CPRNXTBUF   = 2; // continue processing with next buffer
  CPR_NT_LOF  = $8000; // copper instruction only for Longint frames
  CPR_NT_SHT  = $4000; // copper instruction only for long frames
  CPR_NT_SYS  = $2000; // copper user instruction only

const
// internal cliprect flags }
  CR_NEEDS_NO_CONCEALED_RASTERS = 1;
  CR_NEEDS_NO_LAYERBLIT_DAMAGE  = 2;
// defines for code values for getcode
  ISLESSX = 1;
  ISLESSY = 2;
  ISGRTRX = 4;
  ISGRTRY = 8;

// ta_Style/tta_Style
  FS_NORMAL      = 0;       // normal text (no style bits set)
  FSB_UNDERLINED = 0;       // underlined (under baseline)
  FSF_UNDERLINED = 1 shl FSB_UNDERLINED;
  FSB_BOLD       = 1;       // bold face text (ORed w/ shifted)
  FSF_BOLD       = 1 shl FSB_BOLD;
  FSB_ITALIC     = 2;       // italic (slanted 1:2 right)
  FSF_ITALIC     = 1 shl FSB_ITALIC;
  FSB_EXTENDED   = 3;       // extended face (wider than normal)
  FSF_EXTENDED   = 1 shl FSB_EXTENDED;
  FSB_COLORFONT  = 6;       // this uses ColorTextFont structure
  FSF_COLORFONT  = 1 shl FSB_COLORFONT;
  FSB_TAGGED     = 7;       // the TextAttr is really an TTextAttr
  FSF_TAGGED     = 1 shl FSB_TAGGED;
// ta_Flags/tta_Flags
  FPB_ROMFONT      = 0;      // font is in rom
  FPF_ROMFONT      = 1 shl FPB_ROMFONT;
  FPB_DISKFONT     = 1;      // font is from diskfont.library
  FPF_DISKFONT     = 1 shl FPB_DISKFONT;
  FPB_REVPATH      = 2;      // designed path is reversed (e.g. left)
  FPF_REVPATH      = 1 shl FPB_REVPATH;
  FPB_TALLDOT      = 3;      // designed for hires non-interlaced
  FPF_TALLDOT      = 1 shl FPB_TALLDOT;
  FPB_WIDEDOT      = 4;      // designed for lores interlaced
  FPF_WIDEDOT      = 1 shl FPB_WIDEDOT;
  FPB_PROPORTIONAL = 5;      //character sizes can vary from nominal
  FPF_PROPORTIONAL = 1 shl FPB_PROPORTIONAL;
  FPB_DESIGNED     = 6;     // size is "designed", not constructed
  FPF_DESIGNED     = 1 shl FPB_DESIGNED;
  FPB_REMOVED      = 7;     // the font has been removed
  FPF_REMOVED      = 1 shl FPB_REMOVED;

  // TCopList.Flags
  EXACT_LINE = 1;
  HALF_LINE = 2;
  // drawing modes
  JAM1       = 0; // jam 1 color into raster
  JAM2       = 1; // jam 2 colors into raster
  COMPLEMENT = 2; // XOR bits into raster
  INVERSVID  = 4; // inverse video for drawing modes

  // these are the flag bits for RastPort flags
  FRST_DOT = $01; // draw the first dot of this line ?
  ONE_DOT  = $02; // use one dot mode for drawing lines
  DBUFFER  = $04; // flag set when RastPorts are double-buffered

  // only used for bobs
  AREAOUTLINE = $08; // used by areafiller
  NOCROSSFILL = $20; // areafills have no crossovers

{ there is only one style of clipping: raster clipping }
{ this preserves the continuity of jaggies regardless of clip window }
{ When drawing into a RastPort, if the ptr to ClipRect is nil then there }
{ is no clipping done, this is dangerous but useful for speed }


Const
    CleanUp     = $40;
    CleanMe     = CleanUp;

    BltClearWait        = 1;    { Waits for blit to finish }
    BltClearXY          = 2;    { Use Row/Bytes per row method }

        { Useful minterms }

    StraightCopy        = $C0;  { Vanilla copy }
    InvertAndCopy       = $30;  { Invert the source before copy }
    InvertDest          = $50;  { Forget source, invert dest }


// mode coercion definitions
const
//  These flags are passed (in combination) to CoerceMode() to determine the type of coercion required.
  PRESERVE_COLORS = 1; // Ensure that the mode coerced to can display just as many colours as the ViewPort being coerced.
  AVOID_FLICKER   = 2; // Ensure that the mode coerced to is not interlaced.
  IGNORE_MCOMPAT  = 4; // Coercion should ignore monitor compatibility issues.

  BIDTAG_COERCE   = 1; //  Private

// graphics GELS definitions
const
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

const

{ These bit descriptors are used by the GEL collide routines.
  These bits are set in the hitMask and meMask variables of
  a GEL to describe whether or not these types of collisions
  can affect the GEL.  BNDRY_HIT is described further below;
  this bit is permanently assigned as the boundary-hit flag.
  The other bit GEL_HIT is meant only as a default to cover
  any GEL hitting any other; the user may redefine this bit. }
  BORDERHIT = 0;

{ These bit descriptors are used by the GEL boundry hit routines.
  When the user's boundry-hit routine is called (via the argument
  set by a call to SetCollision) the first argument passed to
  the user's routine is the Pointer of the GEL involved in the
  boundry-hit, and the second argument has the appropriate bit(s)
  set to describe which boundry was surpassed }
  TOPHIT    = 1;
  BOTTOMHIT = 2;
  LEFTHIT   = 4;
  RIGHTHIT  = 8;

const
  SS_GRAPHICS = $02;

  VIEW_EXTRA_TYPE      = 1;
  VIEWPORT_EXTRA_TYPE  = 2;
  SPECIAL_MONITOR_TYPE = 3;
  MONITOR_SPEC_TYPE    = 4;

type
  // structure used by AddTOFTask
  PIsrvstr = ^TIsrvstr;
  TIsrvstr = record
    is_Node: TNode;
    Iptr: PIsrvstr;   // passed to srvr by os
    code: Pointer;
    ccode: Pointer;
    Carg: APTR;
  end;

const
  TO_MONITOR       = 0;
  FROM_MONITOR     = 1;
  STANDARD_XOFFSET = 9;
  STANDARD_YOFFSET = 0;

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


{ obsolete, v37 compatible definitions follow }
  REQUEST_NTSC          =  1 shl MSB_REQUEST_NTSC;
  REQUEST_PAL           =  1 shl MSB_REQUEST_PAL;
  REQUEST_SPECIAL       =  1 shl MSB_REQUEST_SPECIAL;
  REQUEST_A2024         =  1 shl MSB_REQUEST_A2024;

  DEFAULT_MONITOR_NAME  : PChar = 'default.monitor';
  NTSC_MONITOR_NAME     : PChar = 'ntsc.monitor';
  PAL_MONITOR_NAME      : PChar = 'pal.monitor';
  VGA_MONITOR_NAME      : PChar = 'vga.monitor';
  VGA70_MONITOR_NAME    : PChar = 'vga70.monitor';
  STANDARD_MONITOR_MASK = REQUEST_NTSC or REQUEST_PAL;

  STANDARD_NTSC_ROWS    =  262;
  STANDARD_PAL_ROWS     =  312;
  STANDARD_COLORCLOCKS  =  226;
  STANDARD_DENISE_MAX   =  455;
  STANDARD_DENISE_MIN   =  93 ;
  STANDARD_NTSC_BEAMCON =  $0000;
  STANDARD_PAL_BEAMCON  =  DISPLAYPAL;

  SPECIAL_BEAMCON       = VARVBLANK or LOLDIS or VARVSYNC or VARHSYNC or VARBEAM or CSBLANK or VSYNCTRUE;

  MIN_NTSC_ROW    = 21;
  MIN_PAL_ROW     = 29;
  STANDARD_VIEW_X = $81;
  STANDARD_VIEW_Y = $2C;
  STANDARD_HBSTRT = $06;
  STANDARD_HSSTRT = $0B;
  STANDARD_HSSTOP = $1C;
  STANDARD_HBSTOP = $2C;
  STANDARD_VBSTRT = $0122;
  STANDARD_VSSTRT = $02A6;
  STANDARD_VSSTOP = $03AA;
  STANDARD_VBSTOP = $1066;

  VGA_COLORCLOCKS = STANDARD_COLORCLOCKS / 2;
  VGA_TOTAL_ROWS  = STANDARD_NTSC_ROWS * 2;
  VGA_DENISE_MIN  = 59;
  MIN_VGA_ROW     = 29;
  VGA_HBSTRT      = $08;
  VGA_HSSTRT      = $0E;
  VGA_HSSTOP      = $1C;
  VGA_HBSTOP      = $1E;
  VGA_VBSTRT      = $0000;
  VGA_VSSTRT      = $0153;
  VGA_VSSTOP      = $0235;
  VGA_VBSTOP      = $0CCD;

  VGA70_COLORCLOCKS = STANDARD_COLORCLOCKS / 2;
  VGA70_TOTAL_ROWS  = 449;
  VGA70_DENISE_MIN  = 59;
  MIN_VGA70_ROW     = 35;
  VGA70_HBSTRT      = $08;
  VGA70_HSSTRT      = $0E;
  VGA70_HSSTOP      = $1C;
  VGA70_HBSTOP      = $1E;
  VGA70_VBSTRT      = $0000;
  VGA70_VSSTRT      = $02A6;
  VGA70_VSSTOP      = $0388;
  VGA70_VBSTOP      = $0F73;

  VGA70_BEAMCON     = SPECIAL_BEAMCON xor VSYNCTRUE;

  BROADCAST_HBSTRT  = $01;
  BROADCAST_HSSTRT  = $06;
  BROADCAST_HSSTOP  = $17;
  BROADCAST_HBSTOP  = $27;
  BROADCAST_VBSTRT  = $0000;
  BROADCAST_VSSTRT  = $02A6;
  BROADCAST_VSSTOP  = $054C;
  BROADCAST_VBSTOP  = $1C40;
  BROADCAST_BEAMCON = LOLDIS OR CSBLANK;

  RATIO_FIXEDPART   = 4;
  RATIO_UNITY       = 1 shl RATIO_FIXEDPART;

// if Type == 0 then ColorMap is V1.2/V1.3 compatible
// if Type != 0 then ColorMap is V36       compatible
// the system will never create other than V39 type colormaps when running V39
const
  COLORMAP_TYPE_V1_2 = $00;
  COLORMAP_TYPE_V1_4 = $01;
  COLORMAP_TYPE_V36  = COLORMAP_TYPE_V1_4; // use this definition
  COLORMAP_TYPE_V39  = $02;
  // Flags variable
  COLORMAP_TRANSPARENCY   = $01;
  COLORPLANE_TRANSPARENCY = $02;
  BORDER_BLANKING         = $04;
  BORDER_NOTRANSPARENCY   = $08;
  VIDEOCONTROL_BATCH      = $10;
  USER_COPPER_CLIP        = $20;
  BORDERSPRITES           = $40;

  EXTEND_VSTRUCT = $1000;  { unused bit in Modes field of View }

  CMF_CMTRANS   =  0;
  CMF_CPTRANS   =  1;
  CMF_BRDRBLNK  =  2;
  CMF_BRDNTRAN  =  3;
  CMF_BRDRSPRT  =  6;

  SPRITERESN_ECS       =   0; // ^140ns, except in 35ns viewport, where it is 70ns.
  SPRITERESN_140NS     =   1;
  SPRITERESN_70NS      =   2;
  SPRITERESN_35NS      =   3;
  SPRITERESN_DEFAULT   =   -1;

  // AuxFlags:
  CMAB_FULLPALETTE = 0;
  CMAF_FULLPALETTE = 1 shl CMAB_FULLPALETTE;
  CMAB_NO_INTERMED_UPDATE = 1;
  CMAF_NO_INTERMED_UPDATE = 1 shl CMAB_NO_INTERMED_UPDATE;
  CMAB_NO_COLOR_LOAD = 2;
  CMAF_NO_COLOR_LOAD = 1 shl CMAB_NO_COLOR_LOAD;
  CMAB_DUALPF_DISABLE = 3;
  CMAF_DUALPF_DISABLE = 1 shl CMAB_DUALPF_DISABLE;

// flags values for ObtainPen
const
  PENB_EXCLUSIVE = 0;
  PENB_NO_SETCOLOR = 1;

  PENF_EXCLUSIVE = 1 shl PENB_EXCLUSIVE;
  PENF_NO_SETCOLOR = 1 shl PENB_NO_SETCOLOR;

  // obsolete names for PENF_xxx flags:
  PEN_EXCLUSIVE = PENF_EXCLUSIVE;
  PEN_NO_SETCOLOR = PENF_NO_SETCOLOR;

  // precision values for ObtainBestPen:
  PRECISION_EXACT = -1;
  PRECISION_IMAGE = 0;
  PRECISION_ICON  = 16;
  PRECISION_GUI   = 32;


  // tags for ObtainBestPen:
  OBP_Precision = $84000000;
  OBP_FailIfBad = $84000001;

  // From V39, MakeVPort() will return an error if there is not enough memory,
  // or the requested mode cannot be opened with the requested depth with the
  // given bitmap (for higher bandwidth alignments).
  MVP_OK         = 0; // you want to see this one
  MVP_NO_MEM     = 1; // insufficient memory for intermediate workspace
  MVP_NO_VPE     = 2; // ViewPort does not have a ViewPortExtra, and insufficient memory to allocate a temporary one.
  MVP_NO_DSPINS  = 3; // insufficient memory for intermidiate copper instructions.
  MVP_NO_DISPLAY = 4; // BitMap data is misaligned for this viewport's mode and depth - see AllocBitMap().
  MVP_OFF_BOTTOM = 5; // PRIVATE - you will never see this.

  // From V39, MrgCop() will return an error if there is not enough memory,
  // or for some reason MrgCop() did not need to make any copper lists.
  MCOP_OK     = 0; // you want to see this one
  MCOP_NO_MEM = 1; // insufficient memory to allocate the system copper lists.
  MCOP_NOP    = 2; // MrgCop() did not merge any copper lists (eg, no ViewPorts in the list, or all marked as hidden).

type
  PDBufInfo = ^TDBufInfo;
  TDBufInfo = record
    dbi_Link1: APTR;
    dbi_Count1: LongWord;
    dbi_SafeMessage: TMessage; // replied to when safe to write to old bitmap
    dbi_UserData1: APTR;       // first user data

    dbi_Link2: APTR;
    dbi_Count2: LongWord;
    dbi_DispMessage: TMessage; // replied to when new bitmap has been displayed at least once
    dbi_UserData2: APTR;       // second user data
    dbi_MatchLong: LongWord;
    dbi_CopPtr1,
    dbi_CopPtr2,
    dbi_CopPtr3: APTR;
    dbi_BeamPos1,
    dbi_BeamPos2: Word;
  end;

const
  // include define file for graphics display mode IDs.
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
 * NTSC & PAL "flavors" of these particular keys may be made by or'ing
 * the NTSC or PAL MONITOR_ID with the desired MODE_KEY...
 *
 * For example, to specifically open a PAL HAM interlaced ViewPort
 * (or intuition screen), you would use the modeid of
 * (PAL_MONITOR_ID OR HAMLACE_KEY) }
  LORES_KEY                     =  $00000000;
  HIRES_KEY                     =  $00008000;
  SUPER_KEY                     =  $00008020;
  HAM_KEY                       =  $00000800;
  LORESLACE_KEY                 =  $00000004;
  HIRESLACE_KEY                 =  $00008004;
  SUPERLACE_KEY                 =  $00008024;
  HAMLACE_KEY                   =  $00000804;
  LORESDPF_KEY                  =  $00000400;
  HIRESDPF_KEY                  =  $00008400;
  SUPERDPF_KEY                  =  $00008420;
  LORESLACEDPF_KEY              =  $00000404;
  HIRESLACEDPF_KEY              =  $00008404;
  SUPERLACEDPF_KEY              =  $00008424;
  LORESDPF2_KEY                 =  $00000440;
  HIRESDPF2_KEY                 =  $00008440;
  SUPERDPF2_KEY                 =  $00008460;
  LORESLACEDPF2_KEY             =  $00000444;
  HIRESLACEDPF2_KEY             =  $00008444;
  SUPERLACEDPF2_KEY             =  $00008464;
  EXTRAHALFBRITE_KEY            =  $00000080;
  EXTRAHALFBRITELACE_KEY        =  $00000084;
// New for AA ChipSet (V39)
  HIRESHAM_KEY                  =  $00008800;
  SUPERHAM_KEY                  =  $00008820;
  HIRESEHB_KEY                  =  $00008080;
  SUPEREHB_KEY                  =  $000080a0;
  HIRESHAMLACE_KEY              =  $00008804;
  SUPERHAMLACE_KEY              =  $00008824;
  HIRESEHBLACE_KEY              =  $00008084;
  SUPEREHBLACE_KEY              =  $000080a4;
// Added for V40 - may be useful modes for some games or animations.
  LORESSDBL_KEY                 =  $00000008;
  LORESHAMSDBL_KEY              =  $00000808;
  LORESEHBSDBL_KEY              =  $00000088;
  HIRESHAMSDBL_KEY              =  $00008808;
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
// New for AA ChipSet (V39)
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
// These ModeIDs are the scandoubled equivalents of the above, with the
//  exception of the DualPlayfield modes, as AA does not allow for scandoubling
// dualplayfield.
  VGAEXTRALORESDBL_KEY          =  $00031000;
  VGALORESDBL_KEY               =  $00039000;
  VGAPRODUCTDBL_KEY             =  $00039020;
  VGAEXTRALORESHAMDBL_KEY       =  $00031800;
  VGALORESHAMDBL_KEY            =  $00039800;
  VGAPRODUCTHAMDBL_KEY          =  $00039820;
  VGAEXTRALORESEHBDBL_KEY       =  $00031080;
  VGALORESEHBDBL_KEY            =  $00039080;
  VGAPRODUCTEHBDBL_KEY          =  $000390a0;
// a2024 identifiers
  A2024_MONITOR_ID              =  $00041000;

  A2024TENHERTZ_KEY             =  $00041000;
  A2024FIFTEENHERTZ_KEY         =  $00049000;
// prototype identifiers (private)
  PROTO_MONITOR_ID              =  $00051000;
// These monitors and modes were added for the V38 release.
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
// New AA modes (V39)
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
// These ModeIDs are the scandoubled equivalents of the above, with the
// exception of the DualPlayfield modes, as AA does not allow for scandoubling
// dualplayfield.
  EURO72EXTRALORESDBL_KEY       =  $00061000;
  EURO72LORESDBL_KEY            =  $00069000;
  EURO72PRODUCTDBL_KEY          =  $00069020;
  EURO72EXTRALORESHAMDBL_KEY    =  $00061800;
  EURO72LORESHAMDBL_KEY         =  $00069800;
  EURO72PRODUCTHAMDBL_KEY       =  $00069820;
  EURO72EXTRALORESEHBDBL_KEY    =  $00061080;
  EURO72LORESEHBDBL_KEY         =  $00069080;
  EURO72PRODUCTEHBDBL_KEY       =  $000690a0;

  EURO36_MONITOR_ID             =  $00071000;
// Euro36 modeids can be ORed with the default modeids a la NTSC and PAL.
// For example, Euro36 SuperHires is
// (EURO36_MONITOR_ID OR SUPER_KEY)
   SUPER72_MONITOR_ID            =  $00081000;

// Super72 modeids can be ORed with the default modeids a la NTSC and PAL.
// For example, Super72 SuperHiresLace (80$600) is
// (SUPER72_MONITOR_ID OR SUPERLACE_KEY).
// The following scandoubled Modes are the exception:
  SUPER72LORESDBL_KEY           =  $00081008;
  SUPER72HIRESDBL_KEY           =  $00089008;
  SUPER72SUPERDBL_KEY           =  $00089028;
  SUPER72LORESHAMDBL_KEY        =  $00081808;
  SUPER72HIRESHAMDBL_KEY        =  $00089808;
  SUPER72SUPERHAMDBL_KEY        =  $00089828;
  SUPER72LORESEHBDBL_KEY        =  $00081088;
  SUPER72HIRESEHBDBL_KEY        =  $00089088;
  SUPER72SUPEREHBDBL_KEY        =  $000890a8;
// These monitors and modes were added for the V39 release.
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

  // availability
  DI_AVAIL_NOCHIPS        = $0001;
  DI_AVAIL_NOMONITOR      = $0002;
  DI_AVAIL_NOTWITHGENLOCK = $0004;
  // mode properties }
  DIPF_IS_LACE   = $00000001;
  DIPF_IS_DUALPF = $00000002;
  DIPF_IS_PF2PRI = $00000004;
  DIPF_IS_HAM    = $00000008;
  DIPF_IS_ECS    = $00000010; // ECS modes (SHIRES, VGA, PRODUCTIVITY) do not support attached sprites.
  DIPF_IS_AA     = $00010000; // AA modes - may only be available if machine has correct memory
                              //   type to support required bandwidth - check availability. (V39)
  DIPF_IS_PAL     =  $00000020;
  DIPF_IS_SPRITES =  $00000040;
  DIPF_IS_GENLOCK =  $00000080;

  DIPF_IS_WB        =  $00000100;
  DIPF_IS_DRAGGABLE =  $00000200;
  DIPF_IS_PANELLED  =  $00000400;
  DIPF_IS_BEAMSYNC  =  $00000800;

  DIPF_IS_EXTRAHALFBRITE = $00001000;

  // The following DIPF_IS_... flags are new for V39
  DIPF_IS_SPRITES_ATT       = $00002000; // supports attached sprites
  DIPF_IS_SPRITES_CHNG_RES  = $00004000; // supports variable sprite resolution
  DIPF_IS_SPRITES_BORDER    = $00008000; // sprite can be displayed in the border
  DIPF_IS_SCANDBL           = $00020000; // scan doubled
  DIPF_IS_SPRITES_CHNG_BASE = $00040000; // can change the sprite base colour
  DIPF_IS_SPRITES_CHNG_PRI  = $00080000; // can change the sprite priority with respect to the playfield(s).
  DIPF_IS_DBUFFER           = $00100000; // can support double buffering
  DIPF_IS_PROGBEAM          = $00200000; // is a programmed beam-sync mode
  DIPF_IS_FOREIGN           = $80000000; // this mode is not native to the Amiga

// Use these tags for passing to BestModeID() (V39)
  SPECIAL_FLAGS = DIPF_IS_DUALPF or DIPF_IS_PF2PRI or DIPF_IS_HAM or DIPF_IS_EXTRAHALFBRITE;

  BIDTAG_DIPFMustHave     = $80000001; // mask of the DIPF_ flags the ModeID must have Default - nil
  BIDTAG_DIPFMustNotHave  = $80000002; // mask of the DIPF_ flags the ModeID must not have Default - SPECIAL_FLAGS
  BIDTAG_ViewPort         = $80000003; // ViewPort for which a ModeID is sought. Default - nil

  BIDTAG_NominalWidth  = $80000004; // \ together make the aspect ratio and
  BIDTAG_NominalHeight = $80000005; // / override the vp->Width/Height.
                                       // Default - SourceID NominalDimensionInfo,
                                       // or vp->DWidth/Height, or (640 * 200), in that preferred order.

  BIDTAG_DesiredWidth  = $80000006; // \ Nominal Width and Height of the
  BIDTAG_DesiredHeight = $80000007; // / returned ModeID. Default - same as Nominal

  BIDTAG_Depth            = $80000008; // ModeID must support this depth. Default - vp.^RasInfo^.BitMap^.Depth or 1 }
  BIDTAG_MonitorID        = $80000009; // ModeID must use this monitor. Default - use best monitor available
  BIDTAG_SourceID         = $8000000a; // instead of a ViewPort. Default - VPModeID(vp) if BIDTAG_ViewPort is specified, else leave the DIPFMustHave and DIPFMustNotHave values untouched.

  BIDTAG_RedBits   = $8000000b; // \
  BIDTAG_BlueBits  = $8000000c; //  > Match up from the database. Default - 4
  BIDTAG_GreenBits = $8000000d; // /

  BIDTAG_GfxPrivate = $8000000e; // Private

// graphics display control register definitions
const
  // bplcon0 defines }
  MODE_640    = $8000;
  PLNCNTMSK   = $7;    // how many bit planes? 0 = none, 1->6 = 1->6, 7 = reserved
  PLNCNTSHFT  = 12;    // bits to shift for bplcon0
  PF2PRI      = $40;   // bplcon2 bit
  COLORON     = $0200; // disable color burst
  DBLPF       = $400;
  HOLDNMODIFY = $800;
  INTERLACE   = 4;     // interlace mode for 400
  // bplcon1 defines
  PFA_FINE_SCROLL       = $F;
  PFB_FINE_SCROLL_SHIFT = 4;
  PF_FINE_SCROLL_MASK   = $F;
  // display window start and stop defines
  DIW_HORIZ_POS       = $7F;  // horizontal start/stop
  DIW_VRTCL_POS       = $1FF; // vertical start/stop
  DIW_VRTCL_POS_SHIFT = $7;
  // Data fetch start/stop horizontal position
  DFTCH_MASK  = $FF;
  // vposr bits
  VPOSRLOF    = $8000;

// graphics displayinfo definitions

// the "public" handle to a DisplayInfoRecord
type
  DisplayInfoHandle = APTR;

// datachunk type identifiers

const
  DTAG_DISP = $80000000;
  DTAG_DIMS = $80001000;
  DTAG_MNTR = $80002000;
  DTAG_NAME = $80003000;
  DTAG_VEC  = $80004000; // internal use only

type
  PQueryHeader = ^TQueryHeader;
  TQueryHeader = record
    StructID,         // datachunk type identifier
    DisplayID,        // copy of display record key
    SkipID,           // TAG_SKIP
    Length: LongWord; // length of local data in double-longwords
  end;

  PDisplayInfo = ^TDisplayInfo;
  TDisplayInfo = record
    Header: TQueryHeader;
    NotAvailable: Word;       // If 0 DisplayInfo available, else not available -> see Constants DI_AVAIL_*
    PropertyFlags: LongWord;  // Properties of this mode (DIPF_*)
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
    Reserved: array[0..1] of LongWord; // terminator
  end;

Type
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
    Reserved: array[0..1] of LongWord; // terminator
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
    PreferredModeID: LongWord;
    Reserved: array[0..1] of LongWord; // terminator
  end;

const
  // monitor compatibility
  MCOMPAT_MIXED =  0; // can share display with other MCOMPAT_MIXED
  MCOMPAT_SELF  =  1; // can share only within same monitor
  MCOMPAT_NOBODY= -1; // only one viewport at a time

  DISPLAYNAMELEN = 32;

type
  PNameInfo = ^TNameInfo;
  TNameInfo = record
    Header: TQueryHeader;
    Name: array[0..DISPLAYNAMELEN - 1] of Char;
    Reserved: array[0..1] of LongWord; // terminator
  end;

  // The following VecInfo structure is PRIVATE, for our use only Touch these, and burn!
  PVecInfo = ^TVecInfo;
  TVecInfo = record
    Header: TQueryHeader;
    Vec: APTR;
    Data: APTR;
    Type_: Word;  // original "Type" in C Includes
    pad: array[0..2] of Word;
    Reserved: array[0..1] of LongWord;
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
  VTAG_PF1_BASE_GET          = $80000026;
  VTAG_PF2_BASE_GET          = $80000027;
  VTAG_SPEVEN_BASE_GET       = $80000028;
  VTAG_SPODD_BASE_GET        = $80000029;
  VTAG_PF1_BASE_SET          = $8000002a;
  VTAG_PF2_BASE_SET          = $8000002b;
  VTAG_SPEVEN_BASE_SET       = $8000002c;
  VTAG_SPODD_BASE_SET        = $8000002d;
  VTAG_BORDERSPRITE_GET      = $8000002e;
  VTAG_BORDERSPRITE_SET      = $8000002f;
  VTAG_BORDERSPRITE_CLR      = $80000030;
  VTAG_SPRITERESN_SET        = $80000031;
  VTAG_SPRITERESN_GET        = $80000032;
  VTAG_PF1_TO_SPRITEPRI_SET  = $80000033;
  VTAG_PF1_TO_SPRITEPRI_GET  = $80000034;
  VTAG_PF2_TO_SPRITEPRI_SET  = $80000035;
  VTAG_PF2_TO_SPRITEPRI_GET  = $80000036;
  VTAG_IMMEDIATE             = $80000037;
  VTAG_FULLPALETTE_SET       = $80000038;
  VTAG_FULLPALETTE_GET       = $80000039;
  VTAG_FULLPALETTE_CLR       = $8000003A;
  VTAG_DEFSPRITERESN_SET     = $8000003B;
  VTAG_DEFSPRITERESN_GET     = $8000003C;

{ all the following tags follow the new, rational standard for videocontrol tags:
 * VC_xxx,state         set the state of attribute 'xxx' to value 'state'
 * VC_xxx_QUERY,&var    get the state of attribute 'xxx' and store it into the longword
 *                      pointed to by &var.
 *
 * The following are new for V40:
 }
  VC_IntermediateCLUpdate       =  $80000080; // default=true. When set graphics will update the intermediate copper lists on color changes, etc. When false, it won't, and will be faster.
  VC_IntermediateCLUpdate_Query =  $80000081;
  VC_NoColorPaletteLoad         =  $80000082; // default = false. When set, graphics will only load color 0 for this ViewPort, and so the ViewPort's colors will come
                                              //   from the previous ViewPort's. NB - Using this tag and VTAG_FULLPALETTE_SET together is undefined.
  VC_NoColorPaletteLoad_Query   =  $80000083;
  VC_DUALPF_Disable             =  $80000084;
        { default = false. When this flag is set, the dual-pf bit
           in Dual-Playfield screens will be turned off. Even bitplanes
           will still come from the first BitMap and odd bitplanes
           from the second BitMap, and both R[xy]Offsets will be
           considered. This can be used (with appropriate palette
           selection) for cross-fades between differently scrolling
           images.
           When this flag is turned on, colors will be loaded for
           the viewport as if it were a single viewport of depth
           depth1+depth2 }
  VC_DUALPF_Disable_Query       =  $80000085;

const
  SPRITE_ATTACHED = $80;

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
    es_SimpleSprite: TSimpleSprite;  // conventional simple sprite structure
    es_WordWidth: Word;              // graphics use only, subject to change
    es_Flags: Word;                  // graphics use only, subject to change
  end;

const
  // tags for AllocSpriteData()
  SPRITEA_Width         = $81000000;
  SPRITEA_XReplication  = $81000002;
  SPRITEA_YReplication  = $81000004;
  SPRITEA_OutputHeight  = $81000006;
  SPRITEA_Attached      = $81000008;
  SPRITEA_OldDataFormat = $8100000a; // MUST pass in outputheight if using this tag
  // new v50 defines
  SPRITEA_ViewPort      = $81000020; // viewport this sprite is going to be used with

  // tags for GetExtSprite()
  GSTAG_SPRITE_NUM = $82000020;
  GSTAG_ATTACHED   = $82000022;
  GSTAG_SOFTSPRITE = $82000024;

  // tags valid for either GetExtSprite or ChangeExtSprite
  GSTAG_SCANDOUBLED =  $83000000; // request "NTSC-Like" height if possible

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
    bsa_Flags: LongWord;            // reserved.  Must be zero!
    bsa_XDDA, bsa_YDDA: Word;       // reserved
    bsa_Reserved1,
    bsa_Reserved2: Longint;
  end;

  {    tag definitions for GetRPAttr, SetRPAttr     }

const
  RPTAG_Font            =  $80000000; // get/set font
  RPTAG_APen            =  $80000002; // get/set apen
  RPTAG_BPen            =  $80000003; // get/set bpen
  RPTAG_DrMd            =  $80000004; // get/set draw mode
  RPTAG_OutlinePen      =  $80000005; // get/set outline pen. corrected case.
  RPTAG_WriteMask       =  $80000006; // get/set WriteMask
  RPTAG_MaxPen          =  $80000007; // get/set maxpen

  RPTAG_DrawBounds      =  $80000008; // get only rastport draw bounds. pass @rect
  // V50
  RPTAG_PenMode         = $80000080; // Enable/Disable PenMode (Defaults to True)
  RPTAG_FgColor         = $80000081; // 32bit Foreground Color used when PenMode is False
  RPTAG_BgColor         = $80000082; // 32bit Background Color used when PenMode is False
  RPTAG_AlphaMode       = $80000083; // (NYI) Enable/Disable AlphaMode (Defaults to False) Can only work when PenMode is disabled
  RPTAG_RetainAlpha     = $80000084; // (NYI) Retain target Alpha channel if True (Defaults to False) Only makes sense for target formats with alpha channel
  // V51.13 (MorphOS 3)
  RPTAG_XPos            = $80000085; // Graphics pen X position
  RPTAG_YPos            = $80000086; // Graphics pen Y position

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
    TOF_WaitQ: TList;

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
    hedley: array[0..7] of LongWord;
    hedley_sprites: array[0..7] of LongWord;     // sprite ptrs for intuition mouse
    hedley_sprites1: array[0..7] of LongWord;    // sprite ptrs for intuition mouse
    hedley_count: SmallInt;
    hedley_flags: Word;
    hedley_tmp: SmallInt;

    hash_table: PLongInt;          // Hashtable used for GfxAssociate() and GfxLookup() (private!)
    current_tot_rows: Word;
    current_tot_cclks: Word;
    hedley_hint: Byte;
    hedley_hint2: Byte;
    nreserved: array[0..3] of LongWord;
    a2024_sync_raster: PLongInt;
    control_delta_pal: Word;
    control_delta_ntsc: Word;

    Current_Monitor: PMonitorSpec;          // MonitorSpec used for current display
    MonitorList: TList;                     // List of all MonitorSpecs in the system
    Default_Monitor: PMonitorSpec;          // MonitorSpec of "default.monitor"
    MonitorListSemaphore: PSignalSemaphore; // Semaphore for MonitorList access

    DisplayInfoDataBase: APTR;
    TopLine: Word;
    ActiViewCprSemaphore: PSignalSemaphore; // Semaphore for active view access

    UtilityBase: PLongWord;      // for hook AND tag utilities
    ExecBase: PLongWord;         // to link with rom.lib

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
    ColorMask: LongWord;
    IVector,
    IData: APTR;
    SpecialCounter: LongWord;    // special for double buffering
    DBList: APTR;
    MonitorFlags: Word;
    ScanDoubledSprites,
    BP3Bits: Byte;

    MonitorVBlank: TAnalogSignalInterval;
    Natural_Monitor: PMonitorSpec; // Default MonitorSpec for view without explicit MonitorSpec in ViewExtra

    ProgData: APTR;
    ExtSprites: Byte;
    pad3: Byte;
    GfxFlags: Word;
    VBCounter: LongWord;

    HashTableSemaphore: PSignalSemaphore;  // Semaphore for hash_table access, private in fact
    case boolean of
      true: ( ChunkyToPlanarPtr: PLongWord;);  // HWEmul[0];
      false: (HWEmul: array[0..8] of PLongWord;);
  end;

const
  NTSC       = 1;
  GENLOC     = 2;
  PAL        = 4;
  TODA_SAFE  = 8;
  REALLY_PAL = 16;
  LPEN_SWAP_FRAMES = 32;

  BLITMSG_FAULT = 4;

// bits defs for ChipRevBits
  GFXB_BIG_BLITS = 0 ;
  GFXB_HR_AGNUS  = 0 ;
  GFXB_HR_DENISE = 1 ;
  GFXB_AA_ALICE  = 2 ;
  GFXB_AA_LISA   = 3 ;
  GFXB_AA_MLISA  = 4 ;
// Bit Values for ChipRevBits
  GFXF_BIG_BLITS = 1 shl GFXB_BIG_BLITS;
  GFXF_HR_AGNUS  = 1 shl GFXB_HR_AGNUS;
  GFXF_HR_DENISE = 1 shl GFXB_HR_DENISE;
  GFXF_AA_ALICE  = 1 shl GFXB_AA_ALICE;
  GFXF_AA_LISA   = 1 shl GFXB_AA_LISA;
  GFXF_AA_MLISA  = 1 shl GFXB_AA_MLISA;

// Pass ONE of these to SetChipRev()
  SETCHIPREV_A    = GFXF_HR_AGNUS;
  SETCHIPREV_ECS  = GFXF_HR_AGNUS or GFXF_HR_DENISE;
  SETCHIPREV_AA   = GFXF_AA_ALICE or GFXF_AA_LISA or SETCHIPREV_ECS;
  SETCHIPREV_BEST = $ffffffff;

// memory type }
  BUS_16  = 0;
  NML_CAS = 0;
  BUS_32  = 1;
  DBL_CAS = 2;

  BANDWIDTH_1X    = BUS_16 or NML_CAS;
  BANDWIDTH_2XNML = BUS_32;
  BANDWIDTH_2XDBL = DBL_CAS;
  BANDWIDTH_4X    = BUS_32 or DBL_CAS;

// GfxFlags (private)
  NEW_DATABASE = 1;

  GRAPHICSNAME: PChar  = 'graphics.library';

var
  GfxBase : PGfxBase = nil;

function BltBitMap(srcBitMap: PBitMap location 'a0'; XSrc: LongInt location 'd0'; YSrc: LongInt location 'd1'; DestBitMap: PBitMap location 'a1'; XDest: LongInt location 'd2'; YDest: LongInt location 'd3'; XSize: LongInt location 'd4'; YSize: LongInt location 'd5'; MinTerm: LongWord location 'd6'; Mask: LongWord location 'd7'; TempA: TPlanePtr location 'a2'): LongInt; SysCall GfxBase 030;
procedure BltTemplate(Source: TPlanePtr location 'a0'; XSrc: LongInt location 'd0'; SrcMod: LongInt location 'd1'; DestRP: PRastPort location 'a1'; XDest: LongInt location 'd2'; YDest: LongInt location 'd3'; XSize: LongInt location 'd4'; YSize: LongInt location 'd5'); SysCall GfxBase 036;
procedure ClearEOL(Rp: PRastPort location 'a1'); SysCall GfxBase 042;
procedure ClearScreen(Rp: PRastPort location 'a1'); SysCall GfxBase 048;
function TextLength(Rp: PRastPort location 'a1'; String1: STRPTR location 'a0'; Count: LongWord location 'd0'): LongInt; SysCall GfxBase 054;
function GfxText(Rp: PRastPort location 'a1'; String1: STRPTR location 'a0'; Count: LongWord location 'd0') : LongInt; SysCall GfxBase 060;
function SetFont(Rp: PRastPort location 'a1'; TextFont: PTextFont location 'a0'): LongInt; SysCall GfxBase 066;
function OpenFont(TextAttr: PTextAttr location 'a0'): PTextFont; SysCall GfxBase 072;
procedure CloseFont(TextFont: PTextFont location 'a1'); SysCall GfxBase 078;
function AskSoftStyle(Rp: PRastPort location 'a1'): LongWord; SysCall GfxBase 084;
function SetSoftStyle(Rp: PRastPort location 'a1'; Style: LongWord location 'd0'; Enable: LongWord location 'd1'): LongWord; SysCall GfxBase 090;

procedure AddBob(Bob: PBob location 'a0'; Rp: PRastPort location 'a1'); SysCall GfxBase 096;
procedure AddVSprite(VSprite: PVSprite location 'a0'; Rp: PRastPort location 'a1'); SysCall GfxBase 102;
procedure DoCollision(Rp: PRastPort location 'a1'); SysCall GfxBase 108;
procedure DrawGList(Rp: PRastPort location 'a1'; Vp: PViewPort location 'a0'); SysCall GfxBase 114;
procedure InitGels(Head: PVSprite location 'a0'; Tail: PVSprite location 'a1'; GelsInfo: PGelsInfo location 'a2'); SysCall GfxBase 120;
procedure InitMasks(VSprite: PVSprite location 'a0'); SysCall GfxBase 126;
procedure RemIBob(Bob: PBob location 'a0'; Rp: PRastPort location 'a1'; Vp: PViewPort location 'a2'); SysCall GfxBase 132;
procedure RemVSprite(VSprite: PVSprite location 'a0'); SysCall GfxBase 138;
procedure SetCollision(Num: LongWord location 'd0'; Routine: TProcedure location 'a0'; GInfo: PGelsInfo location 'a1'); syscall GfxBase 144;
procedure SortGList(Rp: PRastPort location 'a1'); SysCall GfxBase 150;
procedure AddAnimOb(AnOb: PAnimOb location 'a0'; AnKey : PPAnimOb location 'a1'; Rp: PRastPort location 'a2'); SysCall GfxBase 156;
procedure Animate(AnKey: PPAnimOb location 'a0'; Rp: PRastPort location 'a1'); SysCall GfxBase 162;
function GetGBuffers(AnOb: PAnimOb location 'a0'; Rp: PRastPort location 'a1'; Flag: LongInt location 'd0'): LongBool; SysCall GfxBase 168;
procedure InitGMasks(AnOb: PAnimOb location 'a0'); SysCall GfxBase 174;

procedure DrawEllipse(Rp: PRastPort location 'a1'; XCenter: LongInt location 'd0'; YCenter: LongInt location 'd1'; a: LongInt location 'd2'; b: LongInt location 'd3'); SysCall GfxBase 180;
function AreaEllipse(Rp: PRastPort location 'a1'; XCenter : LongInt location 'd0'; YCenter : LongInt location 'd1'; a : LongInt location 'd2'; b : LongInt location 'd3'): LongInt; SysCall GfxBase 186;
procedure LoadRGB4(Vp: PViewPort location 'a0'; Colors: PWord location 'a1'; Count: LongInt location 'd0'); SysCall GfxBase 192;
procedure InitRastPort(Rp: PRastPort location 'a1'); SysCall GfxBase 198;
procedure InitVPort(Vp: PViewPort location 'a0'); SysCall GfxBase 204;
function MrgCop(View: PView location 'a1'): LongWord; SysCall GfxBase 210;
function MakeVPort(View: PView location 'a0'; Vp: PViewPort location 'a1'): LongWord; SysCall GfxBase 216;
procedure LoadView(View: PView location 'a1'); SysCall GfxBase 222;
procedure WaitBlit; SysCall GfxBase 228;
procedure SetRast(Rp: PRastPort location 'a1'; Pen: LongWord location 'd0'); SysCall GfxBase 234;
procedure GfxMove(Rp: PRastPort location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'); SysCall GfxBase 240;
procedure Draw(Rp: PRastPort location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'); SysCall GfxBase 246;
function AreaMove(Rp: PRastPort location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'): LongInt; SysCall GfxBase 252;
function AreaDraw(Rp: PRastPort location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'): LongInt; SysCall GfxBase 258;
function AreaEnd(Rp: PRastPort location 'a1'): LongInt; SysCall GfxBase 264;
procedure WaitTOF; SysCall GfxBase 270;
procedure QBlit(Blit: PBltNode location 'a1'); SysCall GfxBase 276;
procedure InitArea(AreaInfo: PAreaInfo location 'a0'; VectorBuffer: APTR location 'a1'; MaxVectors: LongInt location 'd0'); SysCall GfxBase 282;
procedure SetRGB4(Vp: PViewPort location 'a0'; Index: LongInt location 'd0'; Red: LongWord location 'd1'; Green: LongWord location 'd2'; Blue: LongWord location 'd3'); SysCall GfxBase 288;
procedure QBSBlit(Blit: PBltNode location 'a1'); SysCall GfxBase 294;
procedure BltClear(MemBlock: TPlanePtr location 'a1'; ByteCount: LongWord location 'd0'; Flags: LongWord location 'd1'); SysCall GfxBase 300;
procedure RectFill(Rp: PRastPort location 'a1'; XMin: LongInt location 'd0'; YMin: LongInt location 'd1'; XMax: LongInt location 'd2'; YMax: LongInt location 'd3'); SysCall GfxBase 306;
procedure BltPattern(Rp: PRastPort location 'a1'; Mask: TPlanePtr location 'a0'; XMin: LongInt location 'd0'; YMin: LongInt location 'd1'; XMax: LongInt location 'd2'; YMax: LongInt location 'd3'; MaskBPR: LongWord location 'd4'); SysCall GfxBase 312;
function ReadPixel(Rp: PRastPort location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'): LongWord; SysCall GfxBase 318;
function WritePixel(Rp: PRastPort location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'): LongInt; SysCall GfxBase 324;
function Flood(Rp: PRastPort location 'a1'; Mode: LongWord location 'd2'; X: LongInt location 'd0'; Y: LongInt location 'd1'): LongBool; SysCall GfxBase 330;
procedure PolyDraw(Rp: PRastPort location 'a1'; Count: LongInt location 'd0'; PolyTable: PSmallInt location 'a0'); SysCall GfxBase 336;
procedure SetAPen(Rp: PRastPort location 'a1'; Pen: LongWord location 'd0'); SysCall GfxBase 342;
procedure SetBPen(Rp: PRastPort location 'a1'; Pen: LongWord location 'd0'); SysCall GfxBase 348;
procedure SetDrMd(Rp: PRastPort location 'a1'; DrawMode: LongWord location 'd0'); SysCall GfxBase 354;
procedure InitView(View: PView location 'a1'); SysCall GfxBase 360;
procedure CBump(CopList: PUCopList location 'a1'); SysCall GfxBase 366;
procedure CMove(CopList: PUCopList location 'a1'; Destination: APTR location 'd0'; Data: LongInt location 'd1'); SysCall GfxBase 372;
procedure CWait(CopList: PUCopList location 'a1'; V: LongInt location 'd0'; H : LongInt location 'd1'); SysCall GfxBase 378;
function VBeamPos: LongInt; SysCall GfxBase 384;
procedure InitBitMap(BitMap: PBitMap location 'a0'; Depth: LongInt location 'd0'; Width: LongInt location 'd1'; Height: LongInt location 'd2'); SysCall GfxBase 390;
procedure ScrollRaster(Rp: PRastPort location 'a1'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'; XMin: LongInt location 'd2'; YMin: LongInt location 'd3'; XMax: LongInt location 'd4'; YMax: LongInt location 'd5'); SysCall GfxBase 396;
procedure WaitBOVP(Vp: PViewPort location 'a0'); SysCall GfxBase 402;
function GetSprite(Sprite: PSimpleSprite location 'a0'; Num: LongInt location 'd0'): LongInt; SysCall GfxBase 408;
procedure FreeSprite(Num: LongInt location 'd0'); SysCall GfxBase 414;
procedure ChangeSprite(Vp: PViewPort location 'a0'; Sprite: PSimpleSprite location 'a1'; NewData: PWord location 'a2'); SysCall GfxBase 420;
procedure MoveSprite(Vp: PViewPort location 'a0'; Sprite: PSimpleSprite location 'a1'; X: LongInt location 'd0'; Y: LongInt location 'd1'); SysCall GfxBase 426;
procedure LockLayerRom(Layer: PLayer location 'a5'); SysCall GfxBase 432;
procedure UnlockLayerRom(Layer: PLayer location 'a5'); SysCall GfxBase 438;
procedure SyncSBitMap(Layer: PLayer location 'a0'); SysCall GfxBase 444;
procedure CopySBitMap(Layer: PLayer location 'a0'); SysCall GfxBase 450;
procedure OwnBlitter; SysCall GfxBase 456;
procedure DisownBlitter; SysCall GfxBase 462;
function InitTmpRas(TmpRas: PTmpRas location 'a0'; Buffer: TPlanePtr location 'a1'; Size: LongInt location 'd0'): PTmpRas; SysCall GfxBase 468;
procedure AskFont(Rp: PRastPort location 'a1'; TextAttr: PTextAttr location 'a0'); SysCall GfxBase 474;
procedure AddFont(TextFont: PTextFont location 'a1'); SysCall GfxBase 480;
procedure RemFont(TextFont: PTextFont location 'a1'); SysCall GfxBase 486;
function AllocRaster(Width: LongWord location 'd0'; Height: LongWord location 'd1'): TPlanePtr; SysCall GfxBase 492;
procedure FreeRaster(P: TPlanePtr location 'a0'; Width: LongWord location 'd0'; Height: LongWord location 'd1'); SysCall GfxBase 498;
procedure AndRectRegion(Region: PRegion location 'a0'; Rectangle: PRectangle location 'a1'); SysCall GfxBase 504;
function OrRectRegion(Region: PRegion location 'a0'; Rectangle: PRectangle location 'a1'): LongBool; SysCall GfxBase 510;
function NewRegion: PRegion; SysCall GfxBase 516;
function ClearRectRegion(Region: PRegion location 'a0'; Rectangle: PRectangle location 'a1'): LongBool; SysCall GfxBase 522;
procedure ClearRegion(Region: PRegion location 'a0'); SysCall GfxBase 528;
procedure DisposeRegion(Region: PRegion location 'a0'); SysCall GfxBase 534;
procedure FreeVPortCopLists(Vp: PViewPort location 'a0'); SysCall GfxBase 540;
procedure FreeCopList(CopList: PCopList location 'a0'); SysCall GfxBase 546;
procedure ClipBlit(SrcRp: PRastPort location 'a0'; XSrc: LongInt location 'd0'; YSrc: LongInt location 'd1'; DestRp: PRastPort location 'a1'; XDest: LongInt location 'd2'; YDest: LongInt location 'd3'; XSize: LongInt location 'd4'; YSize: LongInt location 'd5'; MinTerm: LongWord location 'd6'); SysCall GfxBase 552;
function XorRectRegion(Region: PRegion location 'a0'; Rectangle: PRectangle location 'a1'): LongBool; SysCall GfxBase 558;
procedure FreeCprList(CprList: PCprList location 'a0'); SysCall GfxBase 564;
function GetColorMap(Entries: LongInt location 'd0'): PColorMap; SysCall GfxBase 570;
procedure FreeColorMap(ColorMap: PColorMap location 'a0'); SysCall GfxBase 576;
function GetRGB4(ColorMap: PColorMap location 'a0'; Entry: LongInt location 'd0'): LongWord; SysCall GfxBase 582;
procedure ScrollVPort(Vp: PViewPort location 'a0'); SysCall GfxBase 588;
function UCopperListInit(UCopList: PUCopList location 'a0'; N: LongInt location 'd0'): PCopList; SysCall GfxBase 594;
procedure FreeGBuffers(AnOb: PAnimOb location 'a0'; Rp: PRastPort location 'a1'; Flag: LongInt location 'd0'); SysCall GfxBase 600;
procedure BltBitMapRastPort(SrcBitMap: PBitMap location 'a0'; XSrc: LongInt location 'd0'; YSrc: LongInt location 'd1'; DestRp: PRastPort location 'a1'; XDest: LongInt location 'd2'; YDest: LongInt location 'd3'; XSize: LongInt location 'd4'; YSize: LongInt location 'd5'; MinTerm: LongWord location 'd6'); SysCall GfxBase 606;
function OrRegionRegion(SrcRegion: PRegion location 'a0'; DestRegion: PRegion location 'a1'): LongBool; SysCall GfxBase 612;
function XorRegionRegion(SrcRegion: PRegion location 'a0'; DestRegion: PRegion location 'a1'): LongBool; SysCall GfxBase 618;
function AndRegionRegion(SrcRegion: PRegion location 'a0'; DestRegion: PRegion location 'a1'): LongBool; SysCall GfxBase 624;
procedure SetRGB4CM(ColorMap: PColorMap location 'a0'; Index: LongInt location 'd0'; Red: LongWord location 'd1'; Green: LongWord location 'd2'; Blue: LongWord location 'd3'); SysCall GfxBase 630;
procedure BltMaskBitMapRastPort(SrcBitMap: PBitMap location 'a0'; XSrc: LongInt location 'd0'; YSrc: LongInt location 'd1'; DestRp: PRastPort location 'a1'; XDest: LongInt location 'd2'; YDest: LongInt location 'd3'; XSize: LongInt location 'd4'; YSize: LongInt location 'd5'; MinTerm: LongWord location 'd6'; BltMask: TPlanePtr location 'a2'); SysCall GfxBase 636;
function AttemptLockLayerRom(Layer: PLayer location 'a5'): LongBool; SysCall GfxBase 654;
function GfxNew(GfxNodeType: LongWord location 'd0'): APTR; SysCall GfxBase 660;
procedure GfxFree(GfxNodePtr: APTR location 'a0'); SysCall GfxBase 666;
procedure GfxAssociate(AssociateNode: APTR location 'a0'; GfxNodePtr: APTR location 'a1'); SysCall GfxBase 672;
procedure BitMapScale(BitScaleArgs: PBitScaleArgs location 'a0'); SysCall GfxBase 678;
function ScalerDiv(Factor: LongWord location 'd0'; Numerator: LongWord location 'd1'; Denominator: LongWord location 'd2'): LongWord; SysCall GfxBase 684;
function TextExtent(Rp: PRastPort location 'a1'; String1: STRPTR location 'a0'; Count: LongInt location 'd0'; TextExtent: PTextExtent location 'a2'): LongInt; SysCall GfxBase 690;
function TextFit(Rp: PRastPort location 'a1'; String1: STRPTR location 'a0'; StrLen: LongWord location 'd0'; TextExtent: PTextExtent location 'a2'; ConstrainingExtent: PTextExtent location 'a3'; StrDirection: LongInt location 'd1'; ConstrainingBitWidth: LongWord location 'd2'; constrainingBitHeight: LongWord location 'd3'): LongWord; SysCall GfxBase 696;
function GfxLookUp(associateNode: APTR location 'a0'): APTR; SysCall GfxBase 702;
function VideoControl(ColorMap: PColorMap location 'a0'; TagArray: PTagItem location 'a1'): LongBool; SysCall GfxBase 708;
function OpenMonitor(MonitorName: PChar location 'a1'; DisplayID: LongWord location 'd0'): PMonitorSpec; SysCall GfxBase 714;
function CloseMonitor(MonitorSpec: PMonitorSpec location 'a0'): LongBool; SysCall GfxBase 720;
function FindDisplayInfo(DisplayID: LongWord location 'd0'): DisplayInfoHandle; SysCall GfxBase 726;
function NextDisplayInfo(DisplayID: LongWord location 'd0'): LongWord; SysCall GfxBase 732;
function GetDisplayInfoData(Handle: DisplayInfoHandle location 'a0'; buf: APTR location 'a1'; Size: LongWord location 'd0'; TagID: LongWord location 'd1'; DisplayID: LongWord location 'd2'): LongWord; SysCall GfxBase 756;
procedure FontExtent(Font: PTextFont location 'a0'; FontExtent: PTextExtent location 'a1'); SysCall GfxBase 762;
function ReadPixelLine8(Rp: PRastPort location 'a0'; XStart: LongWord location 'd0'; YStart: LongWord location 'd1'; Width: LongWord location 'd2'; Array1: PByte location 'a2'; TempRp: PRastPort location 'a1'): LongInt; SysCall GfxBase 768;
function WritePixelLine8(Rp: PRastPort location 'a0'; XStart: LongWord location 'd0'; YStart: LongWord location 'd1'; Width: LongWord location 'd2'; Array1: PByte location 'a2'; TempRp: PRastPort location 'a1'): LongInt; SysCall GfxBase 774;
function ReadPixelArray8(Rp: PRastPort location 'a0'; XStart: LongWord location 'd0'; YStart: LongWord location 'd1'; XStop: LongWord location 'd2'; YStop: LongWord location 'd3'; Array1: PByte location 'a2'; TempRp: PRastPort location 'a1'): LongInt; SysCall GfxBase 780;
function WritePixelArray8(Rp: PRastPort location 'a0'; XStart: LongWord location 'd0'; YStart: LongWord location 'd1'; XStop: LongWord location 'd2'; YStop: LongWord location 'd3'; Array1: PByte location 'a2'; TempRp: PRastPort location 'a1'): LongInt; SysCall GfxBase 786;
function GetVPModeID(Vp: PViewPort location 'a0'): LongInt; SysCall GfxBase 792;
function ModeNotAvailable(ModeID: LongWord location 'd0'): LongInt; SysCall GfxBase 798;
function WeighTAMatch(ReqTextAttr: PTTextAttr location 'a0'; TargetTextAttr: PTextAttr location 'a1'; TargetTags: PTagItem location 'a2'): LongInt; SysCall GfxBase 804;
procedure EraseRect(Rp: PRastPort location 'a1'; XMin: LongInt location 'd0'; YMin: LongInt location 'd1'; XMax: LongInt location 'd2'; YMax: LongInt location 'd3'); SysCall GfxBase 810;
function ExtendFont(Font: PTextFont location 'a0'; FontTags: PTagItem location 'a1'): LongWord; SysCall GfxBase 816;
procedure StripFont(Font: PTextFont location 'a0'); SysCall GfxBase 822;
function CalcIVG(V: PView location 'a0'; Vp: PViewPort location 'a1'): LongWord; SysCall GfxBase 828;
function AttachPalExtra(Cm: PColorMap location 'a0'; Vp: PViewPort location 'a1'): LongInt; SysCall GfxBase 834;
function ObtainBestPenA(Cm: PColorMap location 'a0'; R: LongWord location 'd1'; G: LongWord location 'd2'; B: LongWord location 'd3'; Tags: PTagItem location 'a1'): LongInt; SysCall GfxBase 840;
procedure SetRGB32(Vp: PViewPort location 'a0'; N: LongWord location 'd0'; R: LongWord location 'd1'; G: LongWord location 'd2'; B: LongWord location 'd3'); SysCall GfxBase 852;
function GetAPen(Rp: PRastPort location 'a0'): LongWord; SysCall GfxBase 858;
function GetBPen(Rp: PRastPort location 'a0'): LongWord; SysCall GfxBase 864;
function GetDrMd(Rp: PRastPort location 'a0'): LongWord; SysCall GfxBase 870;
function GetOutlinePen(Rp: PRastPort location 'a0'): LongWord; SysCall GfxBase 876;
procedure LoadRGB32(Vp: PViewPort location 'a0'; var Table: LongWord location 'a1'); SysCall GfxBase 882; overload;
procedure LoadRGB32(Vp: PViewPort location 'a0'; Table: PLongWord location 'a1'); SysCall GfxBase 882; overload;
function SetChipRev(Want: LongWord location 'd0'): LongWord; SysCall GfxBase 888;
procedure SetABPenDrMd(Rp: PRastPort location 'a1'; APen: LongWord location 'd0'; BPen: LongWord location 'd1'; DrawMode: LongWord location 'd2'); SysCall GfxBase 894;
procedure GetRGB32(Cm: PColorMap location 'a0'; FirstColor: LongWord location 'd0'; NColors: LongWord location 'd1'; var Table: LongWord location 'a1'); SysCall GfxBase 900; overload;
procedure GetRGB32(Cm: PColorMap location 'a0'; FirstColor: LongWord location 'd0'; NColors: LongWord location 'd1'; Table: PLongWord location 'a1'); SysCall GfxBase 900; overload;
function AllocBitMap(SizeX: LongWord location 'd0'; SizeY: LongWord location 'd1'; Depth: LongWord location 'd2'; Flags: LongWord location 'd3'; Friend_Bitmap: PBitMap location 'a0'): PBitMap; SysCall GfxBase 918;
procedure FreeBitMap(Bm: PBitMap location 'a0'); SysCall GfxBase 924;
function GetExtSpriteA(Ss: PExtSprite location 'a2'; Tags: PTagItem location 'a1'): LongInt; SysCall GfxBase 930;
function CoerceMode(Vp: PViewPort location 'a0'; MonitorID: LongWord location 'd0'; Flags: LongWord location 'd1'): LongWord; SysCall GfxBase 936;
procedure ChangeVPBitMap(Vp: PViewPort location 'a0'; Bm: PBitMap location 'a1'; Db: PDBufInfo location 'a2'); SysCall GfxBase 942;
procedure ReleasePen(Cm: PColorMap location 'a0'; N: LongWord location 'd0'); SysCall GfxBase 948;
function ObtainPen(Cm: PColorMap location 'a0'; N: LongInt location 'd0'; R: LongWord location 'd1'; G: LongWord location 'd2'; B: LongWord location 'd3'; F: LongInt location 'd4'): LongWord; SysCall GfxBase 954;
function GetBitMapAttr(Bm: PBitMap location 'a0'; AttrNum: LongWord location 'd1'): LongWord; SysCall GfxBase 960;
function AllocDBufInfo(Vp: pViewPort location 'a0'): PDBufInfo; SysCall GfxBase 966;
procedure FreeDBufInfo(Dbi: PDBufInfo location 'a1'); SysCall GfxBase 972;
function SetOutlinePen(Rp: PRastPort location 'a0'; Pen: LongWord location 'd0'): LongWord; SysCall GfxBase 978;
function SetWriteMask(Rp: PRastPort location 'a0'; Msk: LongWord location 'd0'): LongWord; SysCall GfxBase 984;
procedure SetMaxPen(Rp: PRastPort location 'a0'; MaxPen: LongWord location 'd0'); SysCall GfxBase 990;
procedure SetRGB32CM(Cm: PColorMap location 'a0'; N: LongWord location 'd0'; R: LongWord location 'd1'; G: LongWord location 'd2'; B: LongWord location 'd3'); SysCall GfxBase 996;
procedure ScrollRasterBF(Rp: PRastPort location 'a1'; Dx: LongInt location 'd0'; Dy: LongInt location 'd1'; XMin: LongInt location 'd2'; YMin: LongInt location 'd3'; XMax: LongInt location 'd4'; YMax: LongInt location 'd5'); SysCall GfxBase 1002;
function FindColor(Cm: PColorMap location 'a3'; R: LongWord location 'd1'; G: LongWord location 'd2'; B: LongWord location 'd3'; MaxColor: LongInt location 'd4'): LongInt; SysCall GfxBase 1008;
function AllocSpriteDataA(Bm: PBitMap location 'a2'; Tags: PTagItem location 'a1'): PExtSprite; SysCall GfxBase 1020;
function ChangeExtSpriteA(Vp: PViewPort location 'a0'; OldSprite: PExtSprite location 'a1'; NewSprite: PExtSprite location 'a2'; Tags: PTagItem location 'a3'): LongInt; SysCall GfxBase 1026;
procedure FreeSpriteData(Sp: PExtSprite location 'a2'); SysCall GfxBase 1032;
procedure SetRPAttrsA(Rp: PRastPort location 'a0'; Tags: PTagItem location 'a1'); SysCall GfxBase 1038;
procedure GetRPAttrsA(Rp: PRastPort location 'a0'; Tags: PTagItem location 'a1'); SysCall GfxBase 1044;
function BestModeIDA(Tags: PTagItem location 'a0'): LongWord; SysCall GfxBase 1050;
procedure WriteChunkyPixels(Rp: PRastPort location 'a0'; XStart: LongWord location 'd0'; YStart: LongWord location 'd1'; XStop: LongWord location 'd2'; YStop: LongWord location 'd3'; Array1: PByte location 'a2'; BytesPerRow: LongInt location 'd4'); SysCall GfxBase 1056;
function OpenFontTagList(Textattr: PTextAttr location 'a0'; Tags: PTagItem location 'a1'): PTextFont; SysCall GfxBase 1062;

// varargs
function VideoControlTags(ColorMap: PColorMap; const Tags: array of PtrUInt): LongBool; inline;
function WeighTAMatchTags(ReqTextAttr: PTTextAttr; TargetTextAttr: PTextAttr; const TargetTags: array of PtrUInt): LongInt; inline;
function ExtendFontTags(Font: PTextFont; const FontTags: array of PtrUInt): LongWord; inline;
function ObtainBestPen(Cm: PColorMap; r, g, b: LongWord; const Tags: array of PtrUInt): LongInt; inline;
function GetExtSprite(Ss: PExtSprite; const Tags: array of PtrUInt): LongInt; inline;
function AllocSpriteData(Bm: PBitMap; const Tags: array of PtrUInt): PExtSprite; inline;
function ChangeExtSprite(Vp: PViewPort; OldSprite, NewSprite: PExtSprite; const Tags: array of PtrUInt): LongInt; inline;
procedure SetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt); inline;
procedure GetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt); inline;
function BestModeID(const Tags: array of PtrUInt): LongWord; inline;
function OpenFontTags(Textattr: PTextAttr; const Tags: array of PtrUInt): PTextFont; inline;


{ gfxmacros }
procedure BNDRYOFF (w: pRastPort);
procedure InitAnimate (animkey: ppAnimOb);
procedure SetAfPt(w: pRastPort;p: Pointer; n: Byte);
procedure SetDrPt(w: pRastPort;p: Word);
procedure SetOPen(w: pRastPort;c: Byte);
procedure SetWrMsk(w: pRastPort; m: Byte);

procedure SafeSetOutlinePen(w: pRastPort; c: byte);
procedure SafeSetWriteMask( w: pRastPort ; m: smallint ) ;

procedure OFF_DISPLAY (cust: pCustom);
procedure ON_DISPLAY (cust: pCustom);
procedure OFF_SPRITE (cust: pCustom);
procedure ON_SPRITE (cust: pCustom);
procedure OFF_VBLANK (cust: pCustom);
procedure ON_VBLANK (cust: pCustom);

procedure DrawCircle(Rp: PRastPort; xCenter, yCenter, r: LongInt); inline;
function AreaCircle(Rp: PRastPort; xCenter, yCenter, r: SmallInt): LongWord; inline;

function RasSize(w, h: Word): Integer;



{ unit/library initialization }
function InitGraphicsLibrary: boolean;

implementation

function VideoControlTags(ColorMap: PColorMap; const Tags: array of PtrUInt): LongBool; inline;
begin
  VideoControlTags := VideoControl(ColorMap, @Tags);
end;

function WeighTAMatchTags(ReqTextAttr: PTTextAttr; TargetTextAttr: PTextAttr; const TargetTags: array of PtrUInt): LongInt; inline;
begin
  WeighTAMatchTags := WeighTAMatch(ReqTextAttr, TargetTextAttr, @TargetTags);
end;

function ExtendFontTags(Font: PTextFont; const FontTags: array of PtrUInt): LongWord; inline;
begin
  ExtendFontTags := ExtendFont(Font, @FontTags);
end;

function ObtainBestPen(Cm: PColorMap; r, g, b: LongWord; const Tags: array of PtrUInt): LongInt; inline;
begin
  ObtainBestPen := ObtainBestPenA(cm, r, g, b, @Tags);
end;

function GetExtSprite(Ss: PExtSprite; const Tags: array of PtrUInt): LongInt; inline;
begin
  GetExtSprite := GetExtSpriteA(Ss, @Tags);
end;

function AllocSpriteData(Bm: PBitMap; const Tags: array of PtrUInt): PExtSprite; inline;
begin
  AllocSpriteData := AllocSpriteDataA(Bm, @Tags);
end;

function ChangeExtSprite(Vp: PViewPort; OldSprite, NewSprite: PExtSprite; const Tags: array of PtrUInt): LongInt; inline;
begin
  ChangeExtSprite := ChangeExtSpriteA(Vp, OldSprite, NewSprite, @Tags);
end;

procedure SetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt); inline;
begin
  SetRPAttrsA(Rp, @Tags);
end;

procedure GetRPAttrs(Rp: PRastPort; const Tags: array of PtrUInt); inline;
begin
  GetRPAttrsA(Rp, @Tags);
end;

function BestModeID(const Tags: array of PtrUInt): LongWord;
begin
  BestModeID := BestModeIDA(@Tags);
end;

function OpenFontTags(Textattr: PTextAttr; const Tags: array of PtrUInt): PTextFont; inline;
begin
  OpenFontTags := OpenFontTagList(TextAttr, @Tags);
end;

procedure BNDRYOFF (w: pRastPort);
begin
  with w^ do Flags := Flags And (Not AREAOUTLINE);
end;

procedure InitAnimate (animkey: ppAnimOb);
begin
  animkey^ := NIL;
end;

procedure SetAfPt(w: pRastPort;p: Pointer; n: Byte);
begin
  with w^ do begin
    AreaPtrn := p;
    AreaPtSz := n;
  end;
end;

procedure SetDrPt(w: pRastPort;p: Word);
begin
  with w^ do begin
    LinePtrn    := p;
    Flags       := Flags or FRST_doT;
    linpatcnt   := 15;
  end;
end;

procedure SetOPen(w: pRastPort;c: Byte);
begin
  with w^ do begin
    AOlPen  := c;
    Flags   := Flags or AREAOUTLINE;
  end;
end;

{ This function is fine, but For OS39 the SetWriteMask() gfx function
  should be prefered because it SHOULD operate with gfx boards as well.
  At least I hope it does.... }
procedure SetWrMsk(w: pRastPort; m: Byte);
begin
  w^.Mask := m;
end;

procedure SafeSetOutlinePen(w: pRastPort; c: byte);
begin
    IF pGfxBase(GfxBase)^.LibNode.Lib_Version < 39 THEN begin
        w^.AOlPen := c;
        w^.Flags := w^.Flags or AREAOUTLINE;
    end ELSE begin
        c := SetOutlinePen(w,c);
    end;
end;

procedure SafeSetWriteMask( w: pRastPort ; m: smallint ) ;
  VAR x: smallint ;
begin
  IF pGfxBase(GfxBase)^.LibNode.Lib_Version < 39 THEN w^.Mask := BYTE(m)
  ELSE x := SetWriteMask( w, m );
end;

procedure OFF_DISPLAY (cust: pCustom);
begin
    cust^.dmacon := BITCLR or DMAF_RASTER;
end;

procedure ON_DISPLAY (cust: pCustom);
begin
    cust^.dmacon := BITSET or DMAF_RASTER;
end;

procedure OFF_SPRITE (cust: pCustom);
begin
    cust^.dmacon := BITCLR or DMAF_SPRITE;
end;

procedure ON_SPRITE (cust: pCustom);
begin
    cust^.dmacon := BITSET or DMAF_SPRITE;
end;

procedure OFF_VBLANK (cust: pCustom);
begin
    cust^.intena := BITCLR or INTF_VERTB;
end;

procedure ON_VBLANK (cust: pCustom);
begin
    cust^.intena := BITSET or INTF_VERTB;
end;

function RasSize(w, h: Word): Integer; inline;
begin
  RasSize := h * (((w + 15) shr 3) and $FFFE);
end;

procedure DrawCircle(Rp: PRastPort; xCenter, yCenter, r: LongInt); inline;
begin
  DrawEllipse(Rp, xCenter, yCenter, r, r);
end;

function AreaCircle(Rp: PRastPort; xCenter, yCenter, r: SmallInt): LongWord; inline;
begin
  AreaCircle := AreaEllipse(Rp, xCenter, yCenter, r, r);
end;


const
  { Change VERSION and LIBVERSION to proper values }
  VERSION: string[2] = '50';
  LIBVERSION: longword = 50;

function InitGraphicsLibrary: boolean;
begin
  InitGraphicsLibrary := Assigned(GfxBase);
end;

initialization
  GfxBase := PGfxBase(OpenLibrary(GRAPHICSNAME,LIBVERSION));
finalization
  if Assigned(GfxBase) then
    CloseLibrary(PLibrary(GfxBase));
end.
