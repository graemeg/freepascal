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
    14 Jan 2003.

    Update for AmigaOS 3.9.
    Changed start code for unit.
    06 Feb 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT layers;

INTERFACE
USES exec, graphics, utility;

const

    LAYERSIMPLE         = 1;
    LAYERSMART          = 2;
    LAYERSUPER          = 4;
    LAYERUPDATING       = $10;
    LAYERBACKDROP       = $40;
    LAYERREFRESH        = $80;
    LAYER_CLIPRECTS_LOST = $100;        { during BeginUpdate }
                                        { or during layerop }
                                        { this happens if out of memory }
    LMN_REGION          = -1;

type
 pLayer_Info = ^tLayer_Info;
 tLayer_Info = record
    top_layer           : pLayer;
    check_lp            : pLayer;              { !! Private !! }
    obs                 : pClipRect;
    FreeClipRects       : pClipRect;              { !! Private !! }
    PrivateReserve1,                            { !! Private !! }
    PrivateReserve2     : Longint;              { !! Private !! }
    Lock                : tSignalSemaphore;      { !! Private !! }
    gs_Head             : tMinList;              { !! Private !! }
    PrivateReserve3     : smallint;                 { !! Private !! }
    PrivateReserve4     : Pointer;              { !! Private !! }
    Flags               : WORD;
    fatten_count        : Shortint;                 { !! Private !! }
    LockLayersCount     : Shortint;                 { !! Private !! }
    PrivateReserve5     : smallint;                 { !! Private !! }
    BlankHook,                                  { !! Private !! }
    LayerInfo_extra     : Pointer;              { !! Private !! }
 end;

const
    NEWLAYERINFO_CALLED = 1;

{
 * LAYERS_NOBACKFILL is the value needed to get no backfill hook
 * LAYERS_BACKFILL is the value needed to get the default backfill hook
 }
 LAYERS_NOBACKFILL      = 1;
 LAYERS_BACKFILL        = 0;

 LAYERSNAME : PChar = 'layers.library';

VAR LayersBase : pLibrary;

FUNCTION BeginUpdate(l : pLayer) : LONGINT;
FUNCTION BehindLayer(dummy : LONGINT; layer : pLayer) : LONGINT;
FUNCTION CreateBehindHookLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; hook : pHook; bm2 : pBitMap) : pLayer;
FUNCTION CreateBehindLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; bm2 : pBitMap) : pLayer;
FUNCTION CreateUpfrontHookLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; hook : pHook; bm2 : pBitMap) : pLayer;
FUNCTION CreateUpfrontLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; bm2 : pBitMap) : pLayer;
FUNCTION DeleteLayer(dummy : LONGINT; layer : pLayer) : LONGINT;
PROCEDURE DisposeLayerInfo(li : pLayer_Info);
PROCEDURE DoHookClipRects(hook : pHook; rport : pRastPort;const rect : pRectangle);
PROCEDURE EndUpdate(layer : pLayer; flag : ULONG);
FUNCTION FattenLayerInfo(li : pLayer_Info) : LONGINT;
PROCEDURE InitLayers(li : pLayer_Info);
FUNCTION InstallClipRegion(layer : pLayer;const region : pRegion) : pRegion;
FUNCTION InstallLayerHook(layer : pLayer; hook : pHook) : pHook;
FUNCTION InstallLayerInfoHook(li : pLayer_Info;const hook : pHook) : pHook;
PROCEDURE LockLayer(dummy : LONGINT; layer : pLayer);
PROCEDURE LockLayerInfo(li : pLayer_Info);
PROCEDURE LockLayers(li : pLayer_Info);
FUNCTION MoveLayer(dummy : LONGINT; layer : pLayer; dx : LONGINT; dy : LONGINT) : LONGINT;
FUNCTION MoveLayerInFrontOf(layer_to_move : pLayer; other_layer : pLayer) : LONGINT;
FUNCTION MoveSizeLayer(layer : pLayer; dx : LONGINT; dy : LONGINT; dw : LONGINT; dh : LONGINT) : LONGINT;
FUNCTION NewLayerInfo : pLayer_Info;
PROCEDURE ScrollLayer(dummy : LONGINT; layer : pLayer; dx : LONGINT; dy : LONGINT);
FUNCTION SizeLayer(dummy : LONGINT; layer : pLayer; dx : LONGINT; dy : LONGINT) : LONGINT;
PROCEDURE SortLayerCR(layer : pLayer; dx : LONGINT; dy : LONGINT);
PROCEDURE SwapBitsRastPortClipRect(rp : pRastPort; cr : pClipRect);
PROCEDURE ThinLayerInfo(li : pLayer_Info);
PROCEDURE UnlockLayer(layer : pLayer);
PROCEDURE UnlockLayerInfo(li : pLayer_Info);
PROCEDURE UnlockLayers(li : pLayer_Info);
FUNCTION UpfrontLayer(dummy : LONGINT; layer : pLayer) : LONGINT;
FUNCTION WhichLayer(li : pLayer_Info; x : LONGINT; y : LONGINT) : pLayer;

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitLAYERSLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    LAYERSIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox;
{$endif dont_use_openlib}

FUNCTION BeginUpdate(l : pLayer) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L l,A0
    MOVEA.L LayersBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION BehindLayer(dummy : LONGINT; layer : pLayer) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVEA.L LayersBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateBehindHookLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; hook : pHook; bm2 : pBitMap) : pLayer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L bm,A1
    MOVE.L  x0,D0
    MOVE.L  y0,D1
    MOVE.L  x1,D2
    MOVE.L  y1,D3
    MOVE.L  flags,D4
    MOVEA.L hook,A3
    MOVEA.L bm2,A2
    MOVEA.L LayersBase,A6
    JSR -192(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateBehindLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; bm2 : pBitMap) : pLayer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L bm,A1
    MOVE.L  x0,D0
    MOVE.L  y0,D1
    MOVE.L  x1,D2
    MOVE.L  y1,D3
    MOVE.L  flags,D4
    MOVEA.L bm2,A2
    MOVEA.L LayersBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateUpfrontHookLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; hook : pHook; bm2 : pBitMap) : pLayer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L bm,A1
    MOVE.L  x0,D0
    MOVE.L  y0,D1
    MOVE.L  x1,D2
    MOVE.L  y1,D3
    MOVE.L  flags,D4
    MOVEA.L hook,A3
    MOVEA.L bm2,A2
    MOVEA.L LayersBase,A6
    JSR -186(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateUpfrontLayer(li : pLayer_Info; bm : pBitMap; x0 : LONGINT; y0 : LONGINT; x1 : LONGINT; y1 : LONGINT; flags : LONGINT; bm2 : pBitMap) : pLayer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L bm,A1
    MOVE.L  x0,D0
    MOVE.L  y0,D1
    MOVE.L  x1,D2
    MOVE.L  y1,D3
    MOVE.L  flags,D4
    MOVEA.L bm2,A2
    MOVEA.L LayersBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DeleteLayer(dummy : LONGINT; layer : pLayer) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVEA.L LayersBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DisposeLayerInfo(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DoHookClipRects(hook : pHook; rport : pRastPort;const rect : pRectangle);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L hook,A0
    MOVEA.L rport,A1
    MOVEA.L rect,A2
    MOVEA.L LayersBase,A6
    JSR -216(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE EndUpdate(layer : pLayer; flag : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer,A0
    MOVE.L  flag,D0
    MOVEA.L LayersBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FattenLayerInfo(li : pLayer_Info) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE InitLayers(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION InstallClipRegion(layer : pLayer;const region : pRegion) : pRegion;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer,A0
    MOVEA.L region,A1
    MOVEA.L LayersBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION InstallLayerHook(layer : pLayer; hook : pHook) : pHook;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer,A0
    MOVEA.L hook,A1
    MOVEA.L LayersBase,A6
    JSR -198(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION InstallLayerInfoHook(li : pLayer_Info;const hook : pHook) : pHook;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L hook,A1
    MOVEA.L LayersBase,A6
    JSR -204(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LockLayer(dummy : LONGINT; layer : pLayer);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVEA.L LayersBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LockLayerInfo(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE LockLayers(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MoveLayer(dummy : LONGINT; layer : pLayer; dx : LONGINT; dy : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L LayersBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MoveLayerInFrontOf(layer_to_move : pLayer; other_layer : pLayer) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer_to_move,A0
    MOVEA.L other_layer,A1
    MOVEA.L LayersBase,A6
    JSR -168(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MoveSizeLayer(layer : pLayer; dx : LONGINT; dy : LONGINT; dw : LONGINT; dh : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer,A0
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVE.L  dw,D2
    MOVE.L  dh,D3
    MOVEA.L LayersBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NewLayerInfo : pLayer_Info;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L LayersBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ScrollLayer(dummy : LONGINT; layer : pLayer; dx : LONGINT; dy : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L LayersBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SizeLayer(dummy : LONGINT; layer : pLayer; dx : LONGINT; dy : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L LayersBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SortLayerCR(layer : pLayer; dx : LONGINT; dy : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer,A0
    MOVE.L  dx,D0
    MOVE.L  dy,D1
    MOVEA.L LayersBase,A6
    JSR -210(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SwapBitsRastPortClipRect(rp : pRastPort; cr : pClipRect);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rp,A0
    MOVEA.L cr,A1
    MOVEA.L LayersBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ThinLayerInfo(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnlockLayer(layer : pLayer);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L layer,A0
    MOVEA.L LayersBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnlockLayerInfo(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnlockLayers(li : pLayer_Info);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVEA.L LayersBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION UpfrontLayer(dummy : LONGINT; layer : pLayer) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L dummy,A0
    MOVEA.L layer,A1
    MOVEA.L LayersBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WhichLayer(li : pLayer_Info; x : LONGINT; y : LONGINT) : pLayer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L li,A0
    MOVE.L  x,D0
    MOVE.L  y,D1
    MOVEA.L LayersBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of layers.library}
  {$Info don't forget to use InitLAYERSLibrary in the beginning of your program}

var
    layers_exit : Pointer;

procedure CloselayersLibrary;
begin
    ExitProc := layers_exit;
    if LayersBase <> nil then begin
        CloseLibrary(LayersBase);
        LayersBase := nil;
    end;
end;

procedure InitLAYERSLibrary;
begin
    LayersBase := nil;
    LayersBase := OpenLibrary(LAYERSNAME,LIBVERSION);
    if LayersBase <> nil then begin
        layers_exit := ExitProc;
        ExitProc := @CloselayersLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open layers.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    LAYERSIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of layers.library}

var
    layers_exit : Pointer;

procedure CloselayersLibrary;
begin
    ExitProc := layers_exit;
    if LayersBase <> nil then begin
        CloseLibrary(LayersBase);
        LayersBase := nil;
    end;
end;

begin
    LayersBase := nil;
    LayersBase := OpenLibrary(LAYERSNAME,LIBVERSION);
    if LayersBase <> nil then begin
        layers_exit := ExitProc;
        ExitProc := @CloselayersLibrary;
        LAYERSIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open layers.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    LAYERSIsCompiledHow := 3;
   {$Warning No autoopening of layers.library compiled}
   {$Warning Make sure you open layers.library yourself}
{$endif dont_use_openlib}


END. (* UNIT LAYERS *)



