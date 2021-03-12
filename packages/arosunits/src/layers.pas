{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Free Pascal development team

    layers.library functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit layers;

interface
uses
  exec, agraphics, utility;

const

  LAYERSIMPLE          = 1 shl 0;
  LAYERSMART           = 1 shl 1;
  LAYERSUPER           = 1 shl 2;
  LAYERUPDATING        = 1 shl 4;
  LAYERBACKDROP        = 1 shl 6;
  LAYERREFRESH         = 1 shl 7;
  LAYER_CLIPRECTS_LOST = 1 shl 8;  // during BeginUpdate or during layerop this happens if out of memory
  LAYERIREFRESH        = 1 shl 9;
  LAYERIREFRESH2       = 1 shl 10;

  LMN_REGION           = -1;

type
  PLayer_Info = ^TLayer_Info;
  TLayer_Info = record
    top_layer      : PLayer;
    check_lp       : PLayer;               // Private
    obs            : PClipRect;
    FreeClipRects  : PClipRect;            // Private
    PrivateReserve1,                       // Private
    PrivateReserve2: LongInt;              // Private
    Lock           : TSignalSemaphore;     // Private
    gs_Head        : TMinList;             // Private
    PrivateReserve3: SmallInt;             // Private
    PrivateReserve4: Pointer;              // Private
    Flags          : Word;
    fatten_count   : Shortint;             // Private
    LockLayersCount: Shortint;             // Private
    PrivateReserve5: SmallInt;             // Private
    BlankHook,                             // Private
    LayerInfo_extra: Pointer;              // Private
  end;

  // Backfill hook message
  TBackFillMessage = record
    Layer: PLayer;
    Bounds: TRectangle;
    OffsetX: LongInt;
    OffsetY: LongInt;
  end;
  PBackFillMessage = ^TBackFillMessage;

const
  NEWLAYERINFO_CALLED = 1;

  LAYERS_NOBACKFILL      = 1; // is the value needed to get no backfill hook
  LAYERS_BACKFILL        = 0; // is the value needed to get the default backfill hook

  // LayerInfo Flag
  LIFLG_SUPPORTS_OFFSCREEN_LAYERS = 1 shl 8; // Same flag as AmigaOS hack PowerWindowsNG

  // Tags for CreateLayerTagList

  // AmigaOS4-compatible
  LA_ShapeRegion = TAG_USER + 99 + 105; // ABI_V0 compatibility
  LA_ShapeHook   = TAG_USER + 35;       // PRegion. Default is nil (rectangular shape)
  LA_InFrontOf   = TAG_USER + 99 + 102; // ABI_V0 compatibility
  LA_Hidden      = TAG_USER + 41;       // LongBool. Default is False
  // MorphOS-compatible
  LA_Dummy        = TAG_USER + 1024;
  LA_BackfillHook = LA_Dummy + 1;       // PHook. Default is LAYERS_BACKFILL
  LA_TransRegion  = LA_Dummy + 2;       // PRegion. Default is nil (rectangular shape)
  LA_TransHook    = LA_Dummy + 3;
  LA_WindowPtr    = LA_Dummy + 4;
  LA_SuperBitMap  = TAG_USER + 99 + 14; // PBitMap. Default is nil (none) ABI_V0 compatibility
  // AROS-specific
  LA_AROS    = TAG_USER + 1234;
  LA_Behind  = TAG_USER + 99 + 103; // ABI_V0 compatibility
  LA_ChildOf = TAG_USER + 99 + 101; // ABI_V0 compatibility

  LAYERSNAME : PChar = 'layers.library';

var
  LayersBase : PLibrary;

procedure InitLayers(LayerInfo: PLayer_Info); syscall LayersBase 5;
function CreateUpfrontLayer(LayerInfo: PLayer_Info; Bitmap1: PBitMap; x0: LongInt; y0: LongInt; x1: LongInt; y1: LongInt; Flags: LongInt; SuperBitmap2: PBitMap): PLayer; syscall LayersBase 6;
function CreateBehindLayer(LayerInfo: PLayer_Info; Bitmap1: PBitMap; x0: LongInt; y0: LongInt; x1: LongInt; y1: LongInt; Flags: LongInt; SuperBitmap2: PBitMap): PLayer; syscall LayersBase 7;
function UpfrontLayer(dummy: LongInt; Layer: PLayer): LongInt; syscall LayersBase 8;
function BehindLayer(dummy: LongInt; Layer: PLayer): LongInt; syscall LayersBase 9;
function MoveLayer(dummy: LongInt; Layer: PLayer; dx: LongInt; dy: LongInt): LongInt; syscall LayersBase 10;
function SizeLayer(dummy: LongInt; Layer: PLayer; dx: LongInt; dy: LongInt): LongInt; syscall LayersBase 11;
procedure ScrollLayer(dummy: LongInt; Layer: PLayer; dx: LongInt; dy: LongInt); syscall LayersBase 12;
function BeginUpdate(Layer: PLayer): LongInt; syscall LayersBase 13;
procedure EndUpdate(Layer: PLayer; Flag: LongWord); syscall LayersBase 14;
function DeleteLayer(dummy: LongInt; Layer: PLayer): LongInt; syscall LayersBase 15;
procedure LockLayer(dummy: LongInt; Layer: PLayer); syscall LayersBase 16;
procedure UnlockLayer(Layer: PLayer); syscall LayersBase 17;
procedure LockLayers(LayerInfo: PLayer_Info); syscall LayersBase 18;
procedure UnlockLayers(LayerInfo: PLayer_Info); syscall LayersBase 19;
procedure LockLayerInfo(LayerInfo: PLayer_Info); syscall LayersBase 20;
procedure SwapBitsRastPortClipRect(rp: PRastPort; cr: PClipRect); syscall LayersBase  21;
function WhichLayer(LayerInfo: PLayer_Info; x: LongInt; y: LongInt): PLayer; syscall LayersBase 22;
procedure UnlockLayerInfo(LayerInfo: PLayer_Info); syscall LayersBase 23;
function NewLayerInfo: PLayer_Info; syscall LayersBase 24;
procedure DisposeLayerInfo(LayerInfo: PLayer_Info); syscall LayersBase 25;
function FattenLayerInfo(LayerInfo: PLayer_Info): LongInt; syscall LayersBase 26;
procedure ThinLayerInfo(LayerInfo: PLayer_Info); syscall LayersBase 27;
function MoveLayerInFrontOf(layer_to_move: PLayer; other_layer: PLayer): LongInt; syscall LayersBase 28;
function InstallClipRegion(Layer: PLayer; const Region: PRegion): PRegion;  syscall LayersBase 29;
function MoveSizeLayer(Layer: PLayer; dx: LongInt; dy: LongInt; dw: LongInt; dh: LongInt): LongInt; syscall LayersBase 30;
function CreateUpfrontHookLayer(LayerInfo: PLayer_Info; Bitmap1: PBitMap; x0: LongInt; y0: LongInt; x1: LongInt; y1: LongInt; Flags: LongInt; Hook: PHook; SuperBitmap2: PBitMap): PLayer; syscall LayersBase 31;
function CreateBehindHookLayer(LayerInfo: PLayer_Info; Bitmap1: PBitMap; x0: LongInt; y0: LongInt; x1: LongInt; y1: LongInt; Flags: LongInt; Hook: PHook; SuperBitmap2: PBitMap): PLayer; syscall LayersBase 32;
function InstallLayerHook(Layer: PLayer; Hook: PHook): PHook;  syscall LayersBase 33;
function InstallLayerInfoHook(LayerInfo: PLayer_Info; const Hook: PHook): PHook; syscall LayersBase 34;
procedure SortLayerCR(Layer: PLayer; dx: LongInt; dy: LongInt); syscall LayersBase 35;
procedure DoHookClipRects(Hook: PHook; RPort: PRastPort;const Rect: PRectangle); syscall LayersBase 36;
function ChangeLayerVisibility(Layer: PLayer; Visible: LongInt): LongInt; syscall LayersBase 37;
function ScaleLayer(Layer: PLayer; Taglist: PTagItem): LongWord; syscall LayersBase 38;
function CreateUpfrontLayerTagList(LayerInfo: PLayer_Info; Bitmap1: PBitMap; x0: LongInt; y0: LongInt; x1: LongInt; y1: LongInt; Flags: LongInt; TagList: PTagItem): PLayer; syscall LayersBase 39;
function IsLayerVisible(Layer: PLayer): LongInt; syscall LayersBase 40;
function ChangeLayerShape(Layer: PLayer; NewShape: PRegion; CallBack: PHook): PRegion; syscall LayersBase 41;
function CreateBehindLayer(LayerInfo: PLayer_Info; Bitmap1: PBitMap; x0: LongInt; y0: LongInt; x1: LongInt; y1: LongInt; Flags: LongInt; TagList: PTagItem): PLayer; syscall LayersBase 43;
function IsLayerHiddenBySibling(Layer: PLayer; Check_Visible: WordBool): WordBool; syscall LayersBase 44;
procedure CollectPixelsLayer(Layer: PLayer; Region: PRegion; CallBack: PHook); syscall LayersBase 45;


implementation

initialization
  LayersBase := OpenLibrary(LAYERSNAME, 36);

finalization
  CloseLibrary(LayersBase);

eND. (* UNIT LAYERS *)



