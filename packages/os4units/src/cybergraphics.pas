{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Free Pascal development team

    cybergraphics.library functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit cybergraphics;

interface

uses
  Exec, agraphics, utility;

const
  CYBERGFXNAME = 'cybergraphics.library';
  CYBERGFX_INCLUDE_VERSION = 41;

  //  Definition of CyberModeNode (Returned in AllocModeList)

type
   PCyberModeNode = ^TCyberModeNode;
   TCyberModeNode = record
     Node: TNode;
     ModeText: array[0..DISPLAYNAMELEN - 1] of Char; // name for this mode
     DisplayID: ULONG;                               // display id associated with the node
     Width: UWORD;                                   // visible width
     Height: UWORD;                                  // visible height
     Depth: UWORD;                                   // display depth
     DisplayTagList: PTagItem;                       // taglist with extended ModeID information
   end;


const
// Parameters for GetCyberMapAttr()
  CYBRMATTR_XMOD         = $80000001; // function returns BytesPerRow if its called with this parameter
  CYBRMATTR_BPPIX        = $80000002; // BytesPerPixel shall be returned
  CYBRMATTR_DISPADR      = $80000003; // do not use this ! private tag
  CYBRMATTR_PIXFMT       = $80000004; // the pixel format is returned
  CYBRMATTR_WIDTH        = $80000005; // returns width in pixels
  CYBRMATTR_HEIGHT       = $80000006; // returns height in lines
  CYBRMATTR_DEPTH        = $80000007; // returns bits per pixel
  CYBRMATTR_ISCYBERGFX   = $80000008; // returns -1 if supplied bitmap is a cybergfx one
  CYBRMATTR_ISLINEARMEM  = $80000009; // returns -1 if supplied bitmap is linear accessable
  CYBRMATTR_PIXFMT_ALPHA = $8000000A;
// Parameters for GetCyberIDAttr()
  CYBRIDATTR_PIXFMT     = $80000001; // the pixel format is returned
  CYBRIDATTR_WIDTH      = $80000002; // returns visible width in pixels
  CYBRIDATTR_HEIGHT     = $80000003; // returns visible height in lines
  CYBRIDATTR_DEPTH      = $80000004; // returns bits per pixel
  CYBRIDATTR_BPPIX      = $80000005; // BytesPerPixel shall be returned
// Tags for CyberModeRequest()
  CYBRMREQ_TB = TAG_USER + $40000;
  //  FilterTags
  CYBRMREQ_MinDepth    = CYBRMREQ_TB + 0; // Minimum depth for displayed screenmode
  CYBRMREQ_MaxDepth    = CYBRMREQ_TB + 1; // Maximum depth  "       "        "
  CYBRMREQ_MinWidth    = CYBRMREQ_TB + 2; // Minumum width  "       "        "
  CYBRMREQ_MaxWidth    = CYBRMREQ_TB + 3; // Maximum width  "       "        "
  CYBRMREQ_MinHeight   = CYBRMREQ_TB + 4; // Minumum height "       "        "
  CYBRMREQ_MaxHeight   = CYBRMREQ_TB + 5; // Minumum height "       "        "
  CYBRMREQ_CModelArray = CYBRMREQ_TB + 6;
  CYBRMREQ_WinTitle    = CYBRMREQ_TB + 20;
  CYBRMREQ_OKText      = CYBRMREQ_TB + 21;
  CYBRMREQ_CancelText  = CYBRMREQ_TB + 22;
  CYBRMREQ_Screen      = CYBRMREQ_TB + 30; // Screen you wish the Requester to open on
// Tags for BestCyberModeID()
  CYBRBIDTG_TB = TAG_USER + $50000;
  // FilterTags
  CYBRBIDTG_Depth         = CYBRBIDTG_TB + 0;
  CYBRBIDTG_NominalWidth  = CYBRBIDTG_TB + 1;
  CYBRBIDTG_NominalHeight = CYBRBIDTG_TB + 2;
  CYBRBIDTG_MonitorID     = CYBRBIDTG_TB + 3;
  CYBRBIDTG_BoardName     = CYBRBIDTG_TB + 5;
// definition of divers pixel formats
  PIXFMT_LUT8    = 0;
  PIXFMT_RGB15   = 1;
  PIXFMT_BGR15   = 2;
  PIXFMT_RGB15PC = 3;
  PIXFMT_BGR15PC = 4;
  PIXFMT_RGB16   = 5;
  PIXFMT_BGR16   = 6;
  PIXFMT_RGB16PC = 7;
  PIXFMT_BGR16PC = 8;
  PIXFMT_RGB24   = 9;
  PIXFMT_BGR24   = 10;
  PIXFMT_ARGB32  = 11;
  PIXFMT_BGRA32  = 12;
  PIXFMT_RGBA32  = 13;
// SrcRectangle formats defines for xxxPixelArray calls()
  RECTFMT_RGB   = 0;
  RECTFMT_RGBA  = 1;
  RECTFMT_ARGB  = 2;
  RECTFMT_LUT8  = 3;
  RECTFMT_GREY8 = 4;
  RECTFMT_RAW   = 5;
  // AROS extensions

// Parameters for CVideoCtrlTagList()
  SETVC_DPMSLevel = $88002001;
  DPMS_ON         = 0; // Full operation
  DPMS_STANDBY    = 1; // Optional state of minimal power reduction
  DPMS_SUSPEND    = 2; // Significant reduction of power consumption
  DPMS_OFF        = 3; // Lowest level of power consumption
// Tags for LockBitMapTagList()
  LBMI_WIDTH       = $84001001;
  LBMI_HEIGHT      = $84001002;
  LBMI_DEPTH       = $84001003;
  LBMI_PIXFMT      = $84001004;
  LBMI_BYTESPERPIX = $84001005;
  LBMI_BYTESPERROW = $84001006;
  LBMI_BASEADDRESS = $84001007;
// Tags for UnLockBitMapTagList()
  UBMI_UPDATERECTS  = $85001001;
  UBMI_REALLYUNLOCK = $85001002;

type
// Message passed to the DoCDrawMethodTagList() hook function
  PCDrawMsg = ^TCDrawMsg;
  TCDrawMsg = record
    cdm_MemPtr: APTR;
    cdm_offx: ULONG;
    cdm_offy: ULONG;
    cdm_xsize: ULONG;
    cdm_ysize: ULONG;
    cdm_BytesPerRow: UWORD;
    cdm_BytesPerPix: UWORD;
    cdm_ColorModel: UWORD;
  end;

// Colour Table source formats for WriteLUTPixelArray()  ULONG [] table
const
  CTABFMT_XRGB8 = 0;
// graphics.library/AllocBitMap() extended flags
  BMB_SPECIALFMT = 7;
  BMF_SPECIALFMT = 1 shl BMB_SPECIALFMT;

var
  CyberGfxBase: PLibrary = nil;
  ICyberGfx: PInterface = nil;

function CyberGfxObtain(): LongWord; syscall ICyberGfx 60;
function CyberGfxRelease(): LongWord; syscall ICyberGfx 64;
procedure CyberGfxExpunge(); syscall ICyberGfx 68;
function CyberGfxClone(): PInterface; syscall ICyberGfx 72;
// 76 - 88 Reserved
function IsCyberModeID(displayID: LongWord): LongBool; syscall ICyberGfx 92;
function BestCModeIDTagList(BestModeIDTags: PTagItem): LongWord; syscall ICyberGfx 96;
// 100 BestCModeIDTags
function CModeRequestTagList(ModeRequest: APTR; ModeRequestTags: PTagItem): LongWord; syscall ICyberGfx 104;
// 108 CModeRequestTags
function AllocCModeListTagList(ModeListTags: PTagItem): PList; syscall ICyberGfx 112;
// 116 AllocCModeListTags
procedure FreeCModeList(ModeList: PList); syscall ICyberGfx 120;
// 120 reserved
function ScalePixelArray(SrcRect: APTR; SrcW, SrcH, SrcMod: LongWord; RastPort: PRastPort; DestX, DestY: LongInt; DestW, DestH, SrcFormat: LongWord): LongInt; syscall ICyberGfx 128;
function GetCyberMapAttr(CyberGfxBitmap: PBitMap; CyberAttrTag: LongWord): LongWord; syscall ICyberGfx 132;
function GetCyberIDAttr(CyberIDAttr, CyberDisplayModeID: LongWord): LongWord; syscall ICyberGfx 136;
function ReadRGBPixel(Rp: PRastPort; X, Y: LongInt): LongWord; syscall ICyberGfx 140;
function WriteRGBPixel(Rp: PRastPort; X, Y: LongInt; Pixel: LongWord): LongInt; syscall ICyberGfx 144;
function ReadPixelArray(DestRect: APTR; DestX, DestY: LongInt; DestMod: LongWord; Rp: PRastPort; SrcX, SrcY: LongInt; Width, Height, DstFormat: LongWord): LongWord; syscall ICyberGfx 148;
function WritePixelArray(SrcRect: APTR; SrcX, SrcY: LongInt; SrcMod: LongWord; Rp: PRastPort; DestX, DestY: LongInt; Width, Height, SrcFormat: LongWord): LongWord; syscall ICyberGfx 152;
function MovePixelArray(SrcX, SrcY: LongWord; RastPort: PRastPort; DstX, DstY, SizeX, SizeY: LongWord): LongWord; syscall ICyberGfx 156;
// 160 reserved
function InvertPixelArray(Rp: PRastPort; DestX, DestY: LongInt; Width, Height: LongWord): LongWord; syscall ICyberGfx 164;
function FillPixelArray(Rp: PRastPort; DestX, DestY: LongInt; Width, Height, Pixel: LongWord): LongWord; syscall ICyberGfx 168;
procedure DoCDrawMethodTagList(Hook: PHook; Rp: PRastPort; Tags: PTagItem); syscall ICyberGfx 172;
// 176 DoCDrawMethodTags
procedure CVideoCtrlTagList(Vp: PViewPort; TagList: PTagItem); syscall ICyberGfx 180;
// 184 CVideoCtrlTags
function LockBitMapTagList(Bitmap: APTR; TagList: PTagItem): APTR; syscall ICyberGfx 188;
// 192 LockBitMapTags
procedure UnLockBitMap(Handle: APTR); syscall ICyberGfx 196;
procedure UnLockBitMapTagList(Handle: APTR; TagList: PTagItem); syscall ICyberGfx 200;
// 204 UnLockBitMapTags
function ExtractColor(RastPort: PRastPort; SingleMap: PBitMap; Colour: LongWord; SX, SY, Width, Height: LongWord): LongWord; syscall ICyberGfx 208;
// 212 reserved
function WriteLUTPixelArray(SrcRect: APTR; SrcX, SrcY: LongInt; SrcMod: LongWord; Rp: PRastPort; CTable: APTR; DestX, DestY: LongInt; SizeX, SizeY, CTabFormat: LongWord): LongInt; syscall ICyberGfx 216;
// 220 reserved
// 224 reserved
function WritePixelArrayAlpha(Src: APTR; SrcX, SrcY, SrcMod: LongInt; Rp: PRastPort; DestX, DestY: LongInt; Width, Height, GlobalAlpha: LongWord): LongWord; syscall ICyberGfx 228;
procedure BltTemplateAlpha(SrcTemplate: APTR; SrcX, SrcY: LongInt; Rp: PRastPort; DestX, DestY: LongInt; Width, Height: LongWord); syscall ICyberGfx 232;

// Functions and procedures with array of const go here
function AllocCModeListTags(const ModeListTags: array of PtrUInt): PList; inline;
function BestCModeIDTags(const BestModeIDTags: array of PtrUInt): LongWord; inline;
function CModeRequestTags(ModeRequest: APTR; const ModeRequestTags : array of PtrUInt): LongWord; inline;
procedure CVideoCtrlTags(ViewPort: PViewPort; const TagList: array of PtrUInt); inline;
procedure DoCDrawMethodTags(Hook: PHook; a1arg: PRastPort; const TagList: array of PtrUInt); inline;
function LockBitMapTags(BitMap: APTR; const TagList: array of PtrUInt): APTR; inline;
procedure UnLockBitMapTags(Handle: APTR; const TagList: array of PtrUInt); inline;

function SHIFT_PIXFMT(fmt: LongInt): LongInt;
function DOWNSHIFT_PIXFMT(fmt: LongInt): LongInt;

implementation

// Functions and procedures with array of const go here
function AllocCModeListTags(const ModeListTags: array of PtrUInt): PList; inline;
begin
  AllocCModeListTags := AllocCModeListTagList(@ModeListTags);
end;

function BestCModeIDTags(const BestModeIDTags: array of PtrUInt): LongWord; inline;
begin
  BestCModeIDTags := BestCModeIDTagList(@BestModeIDTags);
end;

function CModeRequestTags(ModeRequest: APTR; const ModeRequestTags : array of PtrUInt): LongWord;
begin
  CModeRequestTags := CModeRequestTagList(ModeRequest, @ModeRequestTags);
end;

procedure CVideoCtrlTags(ViewPort: PViewPort; const TagList: array of PtrUInt); inline;
begin
  CVideoCtrlTagList(ViewPort, @TagList);
end;

procedure DoCDrawMethodTags(Hook: PHook; a1arg: PRastPort; const TagList: array of PtrUInt); inline;
begin
  DoCDrawMethodTagList(Hook, a1arg, @TagList);
end;

function LockBitMapTags(BitMap: APTR; const TagList: array of PtrUInt): APTR; inline;
begin
  LockBitMapTags := LockBitMapTagList(BitMap, @TagList);
end;

procedure UnLockBitMapTags(Handle: APTR; const TagList: array of PtrUInt); inline;
begin
  UnLockBitMapTagList(Handle, @TagList);
end;

function SHIFT_PIXFMT(fmt: LongInt): LongInt;
begin
  SHIFT_PIXFMT:=(ULONG(fmt)) shl 24;
end;

function DOWNSHIFT_PIXFMT(fmt: LongInt): LongInt;
begin
  DOWNSHIFT_PIXFMT:=(ULONG(fmt)) shr 24;
end;

initialization
  CyberGfxBase := OpenLibrary(CYBERGFXNAME, 0);
  if Assigned(CyberGfxBase) then
    ICyberGfx := GetInterface(CyberGfxBase, 'main', 1, nil);
finalization
  if Assigned(ICyberGfx) then
    DropInterface(ICyberGfx);
  if Assigned(CyberGfxBase) then
    CloseLibrary(CyberGfxBase);
end.



