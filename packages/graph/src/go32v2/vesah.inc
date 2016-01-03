{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Carl Eric Codere

    This include implements VESA basic access.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


TYPE

  pModeList = ^tModeList;
  tModeList = Array [0..255] of word; {list of modes terminated by -1}
                                      {VESA modes are >=100h}

  TVESAinfo = packed record  { VESA Information request }
    signature : array [1..4] of char;     { This should be VESA   }
    version   : word;                     { VESA revision         }
    str       : pChar;                    { pointer to OEM string }
    caps      : longint;                  { video capabilities    }
    modeList  : pModeList;                { pointer to SVGA modes }
(*    pad       : array [18..260] of byte;  { extra padding more then   }
  end;                             { VESA standard because of bugs on }
                                   { some video cards.                }
  *)
    TotalMem     : word;
  { VESA 2.0 }
    OEMversion   : word;
    VendorPtr    : longint;
    ProductPtr   : longint;
    RevisionPtr  : longint;
    filler       : Array[1..478]of Byte;
  end;

  TVESAModeInfo = packed record
    attr           : word;             { mode attributes   (1.0)    }
    winAAttr,
    winBAttr       : byte;             { window attributes (1.0)    }
    winGranularity : word;  {in K}     { Window granularity (1.0)   }
    winSize        : word;  {in K}     { window size       (1.0)    }
    winASeg,                           { Window A Segment address (1.0) }
    winBSeg        : word;             { Window B Segment address (1.0) }
    winFunct       : procedure;        { Function to swtich bank    }
    BytesPerScanLine: word;            {bytes per scan line (1.0)   }
    { extended information }
    xRes, yRes : word;    {pixels}
    xCharSize,
    yCharSize  : byte;
    planes     : byte;
    bitsPixel  : byte;
    banks      : byte;
    memModel   : byte;
    bankSize   : byte;  {in K}
    NumberOfPages: byte;
(*
    pad : array [29..260] of byte; { always put some more space then required}
  end; *)
    reserved       : byte; { pos $1E }
    rm_size        : byte; { pos $1F }
    rf_pos         : byte; { pos $20 }
    gm_size        : byte; { pos $21 }
    gf_pos         : byte; { pos $22 }
    bm_size        : byte; { pos $23 }
    bf_pos         : byte; { pos $24 }
    (* res_mask       : word; { pos $25 }
      here there was an alignment problem !!
      with default alignment
      res_mask was shifted to $26
      and after PhysAddress to $2A !!! PM *)
    res_size       : byte;
    res_pos        : byte;
    DirectColorInfo: byte; { pos $27 }
  { VESA 2.0 }
    PhysAddress    : longint; { pos $28 }
    OffscreenPtr   : longint; { pos $2C }
    OffscreenMem   : word; { pos $30 }
    reserved2      : Array[1..458]of Byte; { pos $32 }
   end;




var
  VESAInfo    : TVESAInfo;         { VESA Driver information  }
  VESAModeInfo    : TVESAModeInfo;     { Current Mode information }
  hasVesa: Boolean;       { true if we have a VESA compatible graphics card}
                          { initialized in QueryAdapterInfo in graph.inc }
