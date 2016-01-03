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

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit doublebuffer;


{
        DoubleBuffer.p

        These routines provide a very simple double buffer
        mechanism, mainly by being a bit inflexible with the
        choice of screens and windows.

        The first thing to do is to set up a NewScreen structure,
        just like you would do for OpenScreen.  This can be any
        sort of screen.  Then call OpenDoubleBuffer, which will
        return a pointer to a full-screen, borderless backdrop
        window, or Nil if something went wrong.

        If you write into the window's RastPort, it won't be
        visible until you call SwapBuffers.  By the way, you
        can always write into the same RastPort - you don't
        need to reinitialize after SwapBuffers.  All the
        buffer swapping takes place at the level of BitMaps,
        so it's transparent to RastPorts.

        When you have finished, call CloseDoubleBuffer.  If you
        close the window and screen seperately it might crash
        (I'm not sure), but you'll definitely lose memory.

        One last point: GfxBase must be open before you call
                        OpenDoubleBuffer
}

{
     History:
     This is just an translation of DoubleBuffer.p from PCQ pascal
     to FPC Pascal.
     28 Aug 2000.

     Added the define use_amiga_smartlink.
     13 Jan 2003.

     Changed integer > smallint.
     10 Feb 2003.

     nils.sjoholm@mailbox.swipnet.se
}

interface

uses exec, intuition, graphics;

{
    OpenDoubleBuffer opens the Screen described in "ns" without
    modification, then opens a full screen, borderless backdrop
    window on it.  That way the window and screen normally share
    the same BitMap.

    Assuming all that went OK, it allocates an extra BitMap record
    and the Rasters to go along with it.  Then it points the
    Window's BitMap, in its RastPort, at the extra bitmap.
}

Function OpenDoubleBuffer(ns : pNewScreen) : pWindow;

{
    SwapBuffers swaps the PlanePtrs in the Window's and Screen's
    BitMap structure's, then calls ScrollVPort on the Screen's
    ViewPort to get everything going.
}

Procedure SwapBuffers(w : pWindow);

{
    CloseDoubleBuffer resets the Window's BitMap to the Screen's
    BitMap (just in case), closes the Window and Screen, then
    deallocates the extra BitMap structure and Rasters.
}

Procedure CloseDoubleBuffer(w : pWindow);

implementation

Function OpenDoubleBuffer(ns : pNewScreen) : pWindow;
var
    s : pScreen;
    w : pWindow;
    bm : pBitMap;
    i,j : smallint;
    nw : tNewWindow;
    rp : pRastPort;
begin
    s := OpenScreen(ns);
    if s = Nil then
        OpenDoubleBuffer := Nil;

    ShowTitle(s, 0);

    with s^ do begin
        nw.LeftEdge := LeftEdge;
        nw.TopEdge  := TopEdge;
        nw.Width    := Width;
        nw.Height   := Height;
    end;

    with nw do begin
        DetailPen := 0;
        BlockPen  := 0;
        IDCMPFlags := 0;
        Flags     := WFLG_BACKDROP + WFLG_BORDERLESS + WFLG_ACTIVATE;
        FirstGadget := Nil;
        CheckMark := Nil;
        Title := nil;
        Screen := s;
        BitMap := Nil;
        WType := CUSTOMSCREEN_f;
    end;

    w := OpenWindow(Addr(nw));
    if w = Nil then begin
        CloseScreen(s);
        OpenDoubleBuffer := Nil;
    end;

    bm := AllocMem(SizeOf(tBitMap), MEMF_PUBLIC);
    if bm = Nil then begin
        CloseWindow(w);
        CloseScreen(s);
        OpenDoubleBuffer := Nil;
    end;

    bm^ := s^.BitMap;

    with bm^ do
        for i := 0 to Pred(Depth) do begin
            Planes[i] := AllocRaster(s^.Width, s^.Height);
            if Planes[i] = Nil then begin
                if i > 0 then
                    for j := 0 to Pred(i) do
                        FreeRaster(Planes[j], s^.Width, s^.Height);
                CloseWindow(w);
                CloseScreen(s);
                OpenDoubleBuffer := Nil;
            end;
        end;

    rp := w^.RPort;
    rp^.bitMap := bm;

    OpenDoubleBuffer := w;
end;

{
    SwapBuffers swaps the PlanePtrs in the Window's and Screen's
    BitMap structure's, then calls ScrollVPort on the Screen's
    ViewPort to get everything going.
}

Procedure SwapBuffers(w : pWindow);
var
    s : pScreen;
    bm1,
    bm2 : pBitMap;
    rp : pRastPort;
    Temp : Array [0..7] of PLANEPTR;
begin
    s := w^.WScreen;
    rp := w^.RPort;
    bm1 := rp^.bitMap;
    bm2 := addr(s^.BitMap);
    {Temp := bm2^.Planes;
    This is really stupid I can't assign
    bm2^.Planes to Temp, Sigh
    }
    Temp[0] := bm2^.Planes[0];
    Temp[1] := bm2^.Planes[1];
    Temp[2] := bm2^.Planes[2];
    Temp[3] := bm2^.Planes[3];
    Temp[4] := bm2^.Planes[4];
    Temp[5] := bm2^.Planes[5];
    Temp[6] := bm2^.Planes[6];
    Temp[7] := bm2^.Planes[7];

    bm2^.Planes := bm1^.Planes;
   { bm1^.Planes := Temp;
     And this one to, stupid
   }
    bm1^.Planes[0] := Temp[0];
    bm1^.Planes[1] := Temp[1];
    bm1^.Planes[2] := Temp[2];
    bm1^.Planes[3] := Temp[3];
    bm1^.Planes[4] := Temp[4];
    bm1^.Planes[5] := Temp[5];
    bm1^.Planes[6] := Temp[6];
    bm1^.Planes[7] := Temp[7];

    ScrollVPort(addr(s^.ViewPort));
end;

{
    CloseDoubleBuffer resets the Window's BitMap to the Screen's
    BitMap (just in case), closes the Window and Screen, then
    deallocates the extra BitMap structure and Rasters.
}

Procedure CloseDoubleBuffer(w : pWindow);
var
    s : pScreen;
    bm : pBitMap;
    i  : longint;
    rp : pRastPort;
begin
    s := w^.WScreen;
    rp := w^.RPort;
    bm := rp^.bitMap;
    rp^.bitMap := addr(s^.BitMap);
    with bm^ do
        for i := 0 to Pred(Depth) do
            FreeRaster(Planes[i], s^.Width, s^.Height);
    FreeMem(bm, SizeOf(tBitMap));
    CloseWindow(w);
    CloseScreen(s);
end;

end.
