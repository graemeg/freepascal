{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit conunit;

interface

uses
  exec, console, keymap, inputevent, intuition, agraphics;

const
{ ---- console unit numbers for OpenDevice() }
  CONU_LIBRARY  = -1; // no unit, just fill in IO_DEVICE field
  CONU_STANDARD = 0;  // standard unmapped console

{ ---- New unit numbers for OpenDevice() - (V36) }
  CONU_CHARMAP = 1; // bind character map to console
  CONU_SNIPMAP = 3; // bind character map w/ snip to console

{ ---- New flag defines for OpenDevice() - (V37) }
  CONFLAG_DEFAULT           = 0;
  CONFLAG_NODRAW_ON_NEWSIZE = 1;

  PMB_ASM = M_LNM + 1;   // internal storage bit for AS flag
  PMB_AWM = PMB_ASM + 1; // internal storage bit for AW flag
  MAXTABS = 80;


type
  {$PACKRECORDS 2}
  PConUnit = ^TConUnit;
  TConUnit = record
    cu_MP: TMsgPort;
    { ---- read only variables }
    cu_Window: PWindow;      // Intuition window bound to this unit
    cu_XCP: SmallInt;        // character position
    cu_YCP: SmallInt;
    cu_XMax: SmallInt;       // max character position
    cu_YMax: SmallInt;
    cu_XRSize: SmallInt;     // character raster size
    cu_YRSize: SmallInt;
    cu_XROrigin: SmallInt;   // raster origin
    cu_YROrigin: SmallInt;
    cu_XRExtant: SmallInt;   // raster maxima
    cu_YRExtant: SmallInt;
    cu_XMinShrink: SmallInt; // smallest area intact from resize process
    cu_YMinShrink: SmallInt;
    cu_XCCP: SmallInt;       // cursor position
    cu_YCCP: SmallInt;

    { ---- read/write variables (writes must must be protected) }
    { ---- storage for AskKeyMap and SetKeyMap }
    cu_KeyMapStruct: TKeyMap;
    { ---- tab stops }
    cu_TabStops: array[0..MAXTABS - 1] of Word; // 0 at start, 0xFFFF at end of list

    // ---- console rastport attributes
    cu_Mask: ShortInt;
    cu_FgPen: ShortInt;
    cu_BgPen: ShortInt;
    cu_AOLPen: ShortInt;
    cu_DrawMode: ShortInt;
    cu_Obsolete1: ShortInt; // was cu_AreaPtSz -- not used in V36
    cu_Obsolete2: APTR;     // was cu_AreaPtrn -- not used in V36
    cu_Minterms: array[0..7] of Byte; // console minterms
    cu_Font: PTextFont;
    cu_AlgoStyle: Byte;
    cu_TxFlags: Byte;
    cu_TxHeight: Word;
    cu_TxWidth: Word;
    cu_TxBaseline: Word;
    cu_TxSpacing: Word;

    { ---- console MODES and RAW EVENTS switches }
    cu_Modes: array[0..(PMB_AWM + 7) div 8 - 1] of Byte; // one bit per mode
    cu_RawEvents: array[0..(IECLASS_MAX + 7) div 8 - 1] of Byte;
  end;

implementation

end.
