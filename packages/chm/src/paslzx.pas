{ Copyright (C) <2005> <Andrew Haines> paslzx.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  See the file COPYING.LCL, included in this distribution,
  for details about the copyright.
}

{***************************************************************************
 *                     paslzx.pas - LZX decompression routines             *
 *                           -------------------                           *
 *                                                                         *
 *  maintainer: Andrew Haines <andrewd207@aol.com>                         *
 *  source:     modified lzx.c from chmlib 0.37-4                          *
 *  notes:      The lzx.c file was taken from cabextract v0.5, which was,  *
 *              itself, a modified version of the lzx decompression code   *
 *              from unlzx. This file would not be available without the   *
 *              invaluable help from Micha Nelissen fixing my errors.      *
 *                                                                         *
 *              Licensed with permission of Stuart Caie with a modified    *
 *              LGPL.                                                      *
 *                                                                         *
 *  platforms:  Should work on any platform that FreePascal is available   *
 *              on. However it has been tested on only an amd64(Linux) and *
 *              x86(Linux and Windows). Only tested on little endian pc's. *
 ***************************************************************************}

unit paslzx;

{$mode objfpc}{$H+}{$R+}

interface

uses
  Classes, SysUtils;
  
const
  DECR_OK = 0;
  DECR_DATAFORMAT =  1;
  DECR_ILLEGALDATA = 2;
  DECR_NOMEMORY = 3;
  
  
  // some constants defined by the LZX specification
  LZX_MIN_MATCH             =   2;
  LZX_MAX_MATCH             =   257;
  LZX_NUM_CHARS             =   256;
  LZX_BLOCKTYPE_INVALID     =   0;  // also blocktypes 4-7 invalid
  LZX_BLOCKTYPE_VERBATIM    =   1;
  LZX_BLOCKTYPE_ALIGNED     =   2;
  LZX_BLOCKTYPE_UNCOMPRESSED=   3;
  LZX_PRETREE_NUM_ELEMENTS  =   20;
  LZX_ALIGNED_NUM_ELEMENTS  =   8;  // aligned offset tree #elements
  LZX_NUM_PRIMARY_LENGTHS   =   7;  // this one missing from spec!
  LZX_NUM_SECONDARY_LENGTHS =   249;// length tree #elements
  
  // LZX huffman defines: tweak tablebits as desired
  LZX_PRETREE_MAXSYMBOLS    = LZX_PRETREE_NUM_ELEMENTS;
  LZX_PRETREE_TABLEBITS     = 6;
  LZX_MAINTREE_MAXSYMBOLS   = LZX_NUM_CHARS + 50*8;
  LZX_MAINTREE_TABLEBITS    = 12;
  LZX_LENGTH_MAXSYMBOLS     = LZX_NUM_SECONDARY_LENGTHS+1;
  LZX_LENGTH_TABLEBITS      = 12;
  LZX_ALIGNED_MAXSYMBOLS    = LZX_ALIGNED_NUM_ELEMENTS;
  LZX_ALIGNED_TABLEBITS     = 7;

  LZX_LENTABLE_SAFETY       = 64; // we allow length table decoding overruns
  
  extra_bits: array [0..50] of Byte = (
    0,  0,  0,  0,  1,  1,  2,  2,  3,  3,  4,  4,  5,  5,  6,  6,
    7,  7,  8,  8,  9,  9,  10, 10, 11, 11, 12, 12, 13, 13, 14, 14,
    15, 15, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
    17, 17, 17
  );
  
  position_base: array [0..50] of dword = (
          0,       1,       2,      3,      4,      6,      8,     12,     16,     24,     32,       48,      64,      96,     128,     192,
        256,     384,     512,    768,   1024,   1536,   2048,   3072,   4096,   6144,   8192,    12288,   16384,   24576,   32768,   49152,
      65536,   98304,  131072, 196608, 262144, 393216, 524288, 655360, 786432, 917504, 1048576, 1179648, 1310720, 1441792, 1572864, 1703936,
    1835008, 1966080, 2097152
  );
type

  { TBits }

  TBufBits = class
  private
    bitbuf: dword;
    bitsleft: LongInt;
  public
    procedure Init;
    procedure ensure(num: LongInt; var inpos:PByte);
    function peek(numbits: LongInt): dword;
    function remove(numbits: LongInt): dword;
    function read(numbits: LongInt; var inpos: PByte): dword;
  end;

  TLZX_PRETREE_TABLE = record
    Table: array [0..(1 shl LZX_PRETREE_TABLEBITS) + (LZX_PRETREE_MAXSYMBOLS shl 1)-1] of Word;
    Len: array [0..LZX_PRETREE_MAXSYMBOLS + LZX_LENTABLE_SAFETY-1] of Byte;
  end;
  TLZX_MAINTREE_TABLE = record
    Table: array [0..(1 shl LZX_MAINTREE_TABLEBITS) + (LZX_MAINTREE_MAXSYMBOLS shl 1)-1] of Word;
    Len: array [0..LZX_MAINTREE_MAXSYMBOLS + LZX_LENTABLE_SAFETY-1] of Byte;
  end;

  TLZX_LENGTH_TABLE = record
    Table: array [0..(1 shl LZX_LENGTH_TABLEBITS) + (LZX_LENGTH_MAXSYMBOLS shl 1)-1] of Word;
    Len: array [0..LZX_LENGTH_MAXSYMBOLS + LZX_LENTABLE_SAFETY-1] of Byte;
  end;

  TLZX_ALIGNED_TABLE = record
    Table: array [0..(1 shl LZX_ALIGNED_TABLEBITS) + (LZX_ALIGNED_MAXSYMBOLS shl 1)-1] of Word;
    Len: array [0..LZX_ALIGNED_MAXSYMBOLS + LZX_LENTABLE_SAFETY-1] of Byte;
  end;

  PLZXState = ^TLZXState;
  TLZXState = record
    window: PByte;           // the actual decoding window
    window_size,             // window size (32Kb through 2Mb)
    actual_size,             // window size when it was first allocated
    window_posn,             // current offset within the window
    R0, R1, R2: dword;       // for the LRU offset system
    main_elements : Word;    // number of main tree elements
    header_read: LongInt;    // have we started decoding at all yet?
    block_type: Word;        // type of this block
    block_length,            // uncompressed length of this block
    block_remaining,         // uncompressed bytes still left to decode
    frames_read: dword;      // the number of CFDATA blocks
    intel_filesize,          // magic header value used for transform
    intel_curpos: LongInt;   // current offset in transform space
    intel_started: LongInt;  // have we seen any translatable data yet?

    PreTreeTable: TLZX_PRETREE_TABLE;
    MainTreeTable: TLZX_MAINTREE_TABLE;
    LengthTable: TLZX_LENGTH_TABLE;
    AlignedTAble: TLZX_ALIGNED_TABLE;
  end;
  
  // create an lzx state object
  function LZXinit(window: LongInt): PLZXState;
  
  // destroy an lzx state object
  procedure LZXteardown(pState: PLZXState);
  
  // reset an lzx stream
  function LZXreset(pState: PLZXState): LongInt;
  
  function LZXdecompress(pState: PLZXstate; inpos, outpos: PByte; inlen, outlen: LongInt): LongInt;

implementation

const
  ULONG_BITS = sizeof(LongInt)shl 3;
  
function make_decode_table(nsyms: dword; nbits: dword; length: PByte; table: PWord): LongInt;
var
  Sym: Word;
  leaf: dword;
  bit_num: Byte = 1;
  fill: dword;
  pos: dword = 0; //* the current position in the decode table */
  table_mask: dword;
  bit_mask: dword; //* don't do 0 length codes */
  next_symbol: dword; //* base of allocation for long codes */
begin
    Result := 0;
    table_mask :=  1 shl nbits;
    bit_mask := table_mask shr 1;
    next_symbol := bit_mask;
    //* fill entries for codes short enough for a direct mapping */
    while (bit_num <= nbits) do begin
        for sym := 0 to nsyms-1 do begin
            if (length[sym] = bit_num) then begin
                leaf := pos;

                Inc(pos, bit_mask);
                if pos > table_mask then begin
                  Result := 1; //* table overrun */
                  exit;
                end;

                //* fill all possible lookups of this symbol with the symbol itself */
                fill := bit_mask;
                while fill > 0 do
                begin
                  dec(fill);
                  table[leaf] := sym;
                  Inc(leaf);
                end;
            end;
        end;
        bit_mask := bit_mask shr 1;
        Inc(bit_num);
    end;

    //* if there are any codes longer than nbits */
    if pos <> table_mask then begin
        //* clear the remainder of the table */
        for sym := pos to table_mask-1 do table[sym] := 0;

        //* give ourselves room for codes to grow by up to 16 more bits */
        pos := pos shl 16;
        table_mask := table_mask shl 16;
        bit_mask := 1 shl 15;

        while (bit_num <= 16) do begin
            for sym := 0 to nsyms-1 do begin
                if (length[sym] = bit_num) then begin
                    leaf := pos shr 16;
                    for fill := 0 to (bit_num - nbits)-1 do begin
                        //* if this path hasn't been taken yet, 'allocate' two entries */
                        if (table[leaf] = 0) then begin
                            table[(next_symbol shl 1)] := 0;
                            table[(next_symbol shl 1)+1] := 0;
                            table[leaf] := Word(next_symbol);
                            Inc(next_symbol);
                        end;
                        //* follow the path and select either left or right for next bit */
                        leaf := table[leaf] shl 1;
                        if ((pos shr (15-fill)) and 1) > 0 then Inc(leaf);
                    end;
                    table[leaf] := sym;

                    pos := pos + bit_mask;
                    if (pos > table_mask) then begin
                      Result := 1; //* table overflow */
                      exit;
                    end;
                end;
            end;
            bit_mask := bit_mask shr 1;
            Inc(bit_num);
        end;
    end;

    //* full table? */
    if (pos = table_mask) then begin
      Result := 0;
      Exit;
    end;

    //* either erroneous table, or all elements are 0 - let's find out. */
    for sym := 0 to nsyms-1 do begin
      if length[sym] > 0 then begin
        Result := 1;
        Exit;
      end;
    end;
    Result := 0;
end;

type
  PLZX_bits = ^TLzx_bits;
  Tlzx_bits = record
    bb: dword;
    bl: LongInt;
    ip: PByte;
  end;

function READ_HUFFSYM(Table: PWord; Len: PByte; const bits: TBufBits; var inpos: PByte;
             var i, j: DWord; const TableBits, MaxSymbols: DWord; out z: LongInt): LongInt;
var
  hufftbl: PWord;
begin
  bits.ensure(16, inpos);
  hufftbl := Table;
  i := hufftbl[bits.peek(TableBits)];
  if (i) >= MaxSymbols then begin
      j := 1 shl (ULONG_BITS - TableBits);
      repeat
      j := j shr 1;
      i := i shl 1;
      i := i or ord((bits.bitbuf and j) <> 0);
      if j = 0 then begin
         Result := DECR_ILLEGALDATA;
         Exit;
      end;
      i := hufftbl[i];
      until i < MaxSymbols;
  end;
  z := i;
  j := Len[z];
  bits.remove(j);
  Result := 0;
end;

function lzx_read_lens(pState: PLZXState; lens: PByte; first: dword; last: dword; lb: Plzx_bits): LongInt;
var
    i: dword = 0;
    j: dword = 0;
    x,y: dword;
    z: LongInt;

    inpos: PByte;
    bits: TBufBits;
begin
    bits := TBufBits.Create;
    bits.bitbuf := lb^.bb;
    bits.bitsleft := lb^.bl;
    
    inpos := lb^.ip;


    for X := 0 to 19 do begin
        y := bits.read(4, inpos);
        pState^.PreTreeTable.Len[x] := byte(y);
    end;
    if make_decode_table(LZX_PRETREE_MAXSYMBOLS, LZX_PRETREE_TABLEBITS,
                      @pState^.PreTreeTable.Len[0],@pState^.PreTreeTable.Table[0]) >0 then
    begin
       Result := DECR_ILLEGALDATA;
       bits.Free;
       Exit;
    end;


    x := first;
    while x < last do begin
        if READ_HUFFSYM(@pState^.PreTreeTable.Table[0], @pstate^.PreTreeTable.Len[0], bits, inpos, i, j,
                     LZX_PRETREE_TABLEBITS, LZX_PRETREE_MAXSYMBOLS, z) <> 0 then
        begin
           Result := DECR_ILLEGALDATA;
           bits.Free;
           Exit;
        end;
        if (z = 17) then begin
            y := bits.read(4, inpos);
            Inc(y, 4);
            while y > 0 do begin
              dec(y);
              Lens[x] := 0;
              Inc(x);
            end;
        end
        else if (z = 18) then begin
            y := bits.read(5, inpos);
            Inc(y, 20);
            while y > 0 do begin 
              dec(y);
              lens[x] := 0;
              inc(x);
            end;
        end
        else if (z = 19) then begin
            y := bits.read(1, inpos);
            Inc(y, 4);
            if READ_HUFFSYM(@pState^.PreTreeTable.Table[0], @pstate^.PreTreeTable.Len[0], bits, inpos, i, j,
                         LZX_PRETREE_TABLEBITS, LZX_PRETREE_MAXSYMBOLS, z) <> 0 then
            begin
              Result := DECR_ILLEGALDATA;
              bits.Free;
              Exit;
            end;
            z := lens[x] - z;
            if (z < 0) then z := z + 17;
            while y > 0 do begin
              dec(y);
              lens[x] := byte(z);
              inc(x);
            end;
        end
        else begin
            z := lens[x] - z;
            if (z < 0) then  z := z + 17;
            lens[x] := byte(z);
            inc(x);
        end;
    end;

    lb^.bb := bits.bitbuf;
    lb^.bl := bits.bitsleft;
    lb^.ip := inpos;
    Result := 0;
    bits.Free;
end;
  
  
//////////////////////////////////////////////////////////////////////////////////////

function LZXinit(window: LongInt): PLZXState;
var
  pState: PLZXState;
  wndsize: dword;
  i,
  posn_slots: LongInt;
begin
    Result := nil;
    wndsize := 1 shl window;

    //* LZX supports window sizes of 2^15 (32Kb) through 2^21 (2Mb) */
    //* if a previously allocated window is big enough, keep it     */
    if (window < 15) or (window > 21) then begin
      Exit;
    end;

    //* allocate state and associated window */
    New(pState);
    pState^.window := GetMem(wndsize);
    if pState^.window = nil then
    begin
        Dispose(pState);
        Result := nil;
        exit;
    end;
    pState^.actual_size := wndsize;
    pState^.window_size := wndsize;

    //* calculate required position slots */
    if (window = 20) then posn_slots := 42
    else if (window = 21) then posn_slots := 50
    else posn_slots := window shl 1;

    ///** alternatively **/
    ///* posn_slots=i=0; while (i < wndsize) i += 1 << extra_bits[posn_slots++]; */

    ///* initialize other state */
    pState^.R0 := 1;
    pState^.R1 := 1;
    pState^.R2 := 1;
    
    pState^.main_elements   := LZX_NUM_CHARS + (posn_slots shl 3);
    pState^.header_read     := 0;
    pState^.frames_read     := 0;
    pState^.block_remaining := 0;
    pState^.block_type      := LZX_BLOCKTYPE_INVALID;
    pState^.intel_curpos    := 0;
    pState^.intel_started   := 0;
    pState^.window_posn     := 0;

    ///* initialise tables to 0 (because deltas will be applied to them) */
    for i := 0 to LZX_MAINTREE_MAXSYMBOLS-1 do pState^.MainTreeTable.Len[i] := 0;
    for i := 0 to LZX_LENGTH_MAXSYMBOLS-1 do pState^.LengthTable.Len[i] := 0;

    Result := pState;
end;

procedure LZXteardown(pState: PLZXState);
begin
    if pState <> nil then
    begin
        if pState^.window <> nil then
            Freemem(pState^.window);
        Dispose(pState);
    end;
end;

function LZXreset(pState: PLZXState): LongInt;
var
    i: LongInt;
begin
    pState^.R0 := 1;
    pState^.R1 := 1;
    pState^.R2 := 1;
    pState^.header_read     := 0;
    pState^.frames_read     := 0;
    pState^.block_remaining := 0;
    pState^.block_type      := LZX_BLOCKTYPE_INVALID;
    pState^.intel_curpos    := 0;
    pState^.intel_started   := 0;
    pState^.window_posn     := 0;

    for i := 0 to (LZX_MAINTREE_MAXSYMBOLS + LZX_LENTABLE_SAFETY - 1) do pState^.MainTreeTable.Len[i] := 0;
    for i := 0 to LZX_LENGTH_MAXSYMBOLS+LZX_LENTABLE_SAFETY-1 do pState^.LengthTable.Len[i] := 0;
    Result := DECR_OK;
end;

function LZXdecompress(pState: PLZXstate; inpos, outpos: PByte; inlen,
  outlen: LongInt): LongInt;
var
  endinp: PByte;
  window: PByte;
  runsrc,
  rundest: PByte;
  window_posn: dword;
  window_size: dword;
  R0,
  r1,
  R2: dword;
  bits: TBufBits;
  match_offset,
  i,j,k : dword;
  lb: tlzx_bits;
  togo,
  this_run,
  main_element,
  aligned_bits: LongInt;
  match_length,
  length_footer,
  extra,
  verbatim_bits: LongInt;
  data,
  dataend: PByte;
  curpos,
  filesize,
  abs_off,
  rel_off: LongInt;
    function READ_LENGTHS(Len: PByte; first: dword; last: dword): Longint;
    begin
       Result := 0;
       lb.bb := bits.bitbuf;
       lb.bl := bits.bitsleft;
       lb.ip := inpos;
       if (lzx_read_lens(pState, Len,first,last,@lb)) > 0 then begin
           Result := DECR_ILLEGALDATA;
           Exit;
       end;
       bits.bitbuf := lb.bb;
       bits.bitsleft := lb.bl;
       inpos := lb.ip;
    end;

    procedure HandleBlockTypeAligned;
    var
      i, j: dword;
    begin
      for i := 0 to 7 do begin
        j:= bits.read(3, inpos);
        pState^.AlignedTAble.Len[i] := Word(j);
      end;
      if make_decode_table(LZX_ALIGNED_MAXSYMBOLS, LZX_ALIGNED_TABLEBITS,
        @pState^.AlignedTAble.Len[0],@pState^.AlignedTAble.Table[0]) >0 then
      begin
         Result := DECR_ILLEGALDATA;
         Exit;
      end;
    end;

    procedure HandleBlockTypeVerbatim;
    begin
      if (
      READ_LENGTHS(@pState^.MainTreeTable.Len[0], 0, 256) = DECR_ILLEGALDATA)
      or (
      READ_LENGTHS(@pState^.MainTreeTable.Len[0], 256, pState^.main_elements) = DECR_ILLEGALDATA)
      then begin
        Result := DECR_ILLEGALDATA;
        Exit;
      end;
      if make_decode_table(LZX_MAINTREE_MAXSYMBOLS, LZX_MAINTREE_TABLEBITS,
        @pState^.MainTreeTable.Len[0], @pState^.MainTreeTable.Table[0]) >0 then
      begin
         Result := DECR_ILLEGALDATA;
         Exit;
      end;

      if pState^.MainTreeTable.Len[$E8] <> 0 then
        pState^.intel_started := 1;

      if READ_LENGTHS(@pState^.LengthTable.Len[0], 0, LZX_NUM_SECONDARY_LENGTHS) = DECR_ILLEGALDATA then begin
        Result := DECR_ILLEGALDATA;
        Exit;
      end;
      if make_decode_table(LZX_LENGTH_MAXSYMBOLS, LZX_LENGTH_TABLEBITS,
        @pState^.LengthTable.Len[0],@pState^.LengthTable.Table[0]) >0 then
      begin
         Result := DECR_ILLEGALDATA;
         Exit;
      end;
    end;
    
begin
    endinp := inpos + inlen;
    window := pState^.window;

    window_posn := pState^.window_posn;
    window_size := pState^.window_size;
    R0 := pState^.R0;
    R1 := pState^.R1;
    R2 := pState^.R2;
    
    togo := outlen;//, this_run, main_element, aligned_bits;
    bits := TBufBits.Create;
    bits.Init;
    //* read header if necessary */
    if (pState^.header_read) = 0 then begin
        i := 0;
        j := 0;
        k := bits.read(1, inpos);
        if (k) > 0 then begin
            i := bits.read(16, inpos);
            j := bits.read(16, inpos);
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        pState^.intel_filesize := LongInt((i shl 16) or j); ///* or 0 if not encoded */
=======
        pState^.intel_filesize := (i shl 16) or j; ///* or 0 if not encoded */
>>>>>>> graemeg/fixes_2_2
=======
        pState^.intel_filesize := (i shl 16) or j; ///* or 0 if not encoded */
>>>>>>> origin/fixes_2_2
=======
        pState^.intel_filesize := LongInt((i shl 16) or j); ///* or 0 if not encoded */
>>>>>>> origin/fixes_2.4
        pState^.header_read := 1;
    end;

    ///* main decoding loop */
    while (togo > 0) do begin
        ///* last block finished, new block expected */
        if (pState^.block_remaining = 0) then begin
            if (pState^.block_type = LZX_BLOCKTYPE_UNCOMPRESSED) then begin
                if (pState^.block_length and 1) > 0 then Inc(inpos); //* realign bitstream to word */
                bits.Init;
            end;

            pState^.block_type := Word(bits.read(3, inpos));
            i := bits.read(16, inpos);
            j := bits.read(8, inpos);
            
            pState^.block_length := (i shl 8) or j;
            pState^.block_remaining :=  pState^.block_length;

            case (pState^.block_type) of
                LZX_BLOCKTYPE_ALIGNED:
                begin
                    HandleBlockTypeAligned;
                    //* rest of aligned header is same as verbatim */
                    HandleBlockTypeVerbatim;
                end;
                LZX_BLOCKTYPE_VERBATIM:
                begin
                    HandleBlockTypeVerbatim;
                end;
                LZX_BLOCKTYPE_UNCOMPRESSED:
                begin
                    pState^.intel_started := 1; //* because we can't assume otherwise */
                    bits.ensure(16, inpos); //* get up to 16 pad bits into the buffer */
                    if (bits.bitsleft > 16) then Dec(inpos ,2); //* and align the bitstream! */
                    R0 := inpos[0] or (inpos[1]shl 8)or(inpos[2]shl 16)or(inpos[3]shl 24);
                    Inc(inpos,4);
                    R1 := inpos[0] or (inpos[1]shl 8)or(inpos[2]shl 16)or(inpos[3]shl 24);
                    Inc(inpos,4);
                    R2 := inpos[0] or (inpos[1]shl 8)or(inpos[2]shl 16)or(inpos[3]shl 24);
                    Inc(inpos,4);
                end;
            else
                Result := DECR_ILLEGALDATA;
                bits.Free;
                Exit;
            end;
        end;

        //* buffer exhaustion check */
        if (inpos > endinp) then begin
            {* it's possible to have a file where the next run is less than
             * 16 bits in size. In this case, the READ_HUFFSYM() macro used
             * in building the tables will exhaust the buffer, so we should
             * allow for this, but not allow those accidentally read bits to
             * be used (so we check that there are at least 16 bits
             * remaining - in this boundary case they aren't really part of
             * the compressed data)
             *}
            if (inpos > (endinp+2)) or (bits.bitsleft < 16) then begin
              Result := DECR_ILLEGALDATA;
              bits.Free;
              Exit;
            end;
        end;

        this_run := pState^.block_remaining;
        while (this_run > 0) and (togo > 0) do begin

            if (this_run > togo) then this_run := togo;
            Dec(togo, this_run);
            Dec(pState^.block_remaining, this_run);

            //* apply 2^x-1 mask */
            window_posn := window_posn and (window_size - 1);
            //* runs can't straddle the window wraparound */
            if ((window_posn + this_run) > window_size) then begin
                 Result := DECR_DATAFORMAT;
                 bits.Free;
                 Exit;
            end;
            case (pState^.block_type) of

                LZX_BLOCKTYPE_VERBATIM:
                begin
                    while (this_run > 0) do begin
                        if READ_HUFFSYM(@pState^.MainTreeTable.Table[0], @pState^.MainTreeTable.Len[0],
                              bits, inpos, i, j, LZX_MAINTREE_TABLEBITS, LZX_MAINTREE_MAXSYMBOLS,
                              main_element) <> 0 then
                        begin
                          Result := DECR_ILLEGALDATA;
                          bits.Free;
                          Exit;
                        end;

                        if (main_element < LZX_NUM_CHARS) then begin
                            //* literal: 0 to LZX_NUM_CHARS-1 */
                            window[window_posn] := Byte(main_element);
                            Inc(window_posn);
                            Dec(this_run);
                        end
                        else begin
                            //* match: LZX_NUM_CHARS + ((slot<<3) | length_header (3 bits)) */
                            Dec(main_element, LZX_NUM_CHARS);

                            match_length := main_element and LZX_NUM_PRIMARY_LENGTHS;
                            if (match_length = LZX_NUM_PRIMARY_LENGTHS) then begin
                                if READ_HUFFSYM(@pState^.LengthTable.Table[0], @pState^.LengthTable.Len[0],
                                    bits, inpos, i, j, LZX_LENGTH_TABLEBITS, LZX_LENGTH_MAXSYMBOLS,
                                    length_footer) <> 0 then
                                begin
                                  Result := DECR_ILLEGALDATA;
                                  bits.Free;
                                  Exit;
                                end;
                                Inc(match_length, length_footer);
                            end;
                            Inc(match_length, LZX_MIN_MATCH);

                            match_offset := main_element shr 3;

                            if (match_offset > 2) then begin
                                //* not repeated offset */
                                if (match_offset <> 3) then begin
                                    extra := extra_bits[match_offset];
                                    verbatim_bits := bits.read(extra, inpos);
                                    match_offset := position_base[match_offset] - 2 + verbatim_bits;
                                end
                                else begin
                                    match_offset := 1;
                                end;

                                //* update repeated offset LRU queue */
                                R2 := R1;
                                R1 := R0;
                                R0 := match_offset;
                            end
                            else if (match_offset = 0) then begin
                                match_offset := R0;
                            end
                            else if (match_offset = 1) then begin
                                match_offset := R1;
                                R1 := R0;
                                R0 := match_offset;
                            end
                            else begin //* match_offset == 2 */
                                match_offset := R2;
                                R2 := R0;
                                R0 := match_offset;
                            end;

                            rundest := window + window_posn;
                            runsrc  := rundest - match_offset;
                            Inc(window_posn, match_length);
                            if (window_posn > window_size) then begin
                              Result := DECR_ILLEGALDATA;
                              bits.Free;
                              Exit;
                            end;
                            Dec(this_run, match_length);

                            ///* copy any wrapped around source data */
                            while ((runsrc < window) and (match_length > 0)) do begin
                                Dec(match_length);
                                rundest^ := (runsrc + window_size)^;
                                Inc(rundest);
                                Inc(runsrc);
                            end;
                            //* copy match data - no worries about destination wraps */
                            while (match_length > 0) do begin
                              Dec(match_length);
                              rundest^ := runsrc^;
                              Inc(rundest);
                              Inc(runsrc);
                            end;

                        end
                    end;
                end;
                LZX_BLOCKTYPE_ALIGNED:
                begin
                    while (this_run > 0) do begin
                        if READ_HUFFSYM(@pState^.MainTreeTable.Table[0], @pState^.MainTreeTable.Len[0], bits,
                             inpos, i, j, LZX_MAINTREE_TABLEBITS, LZX_MAINTREE_MAXSYMBOLS, main_element) <> 0 then
                        begin
                          Result := DECR_ILLEGALDATA;
                          bits.Free;
                          Exit;
                        end;

                        if (main_element < LZX_NUM_CHARS) then begin
                            //* literal: 0 to LZX_NUM_CHARS-1 */
                            window[window_posn] := Byte(main_element);
                            Inc(window_posn);
                            Dec(this_run);
                        end
                        else begin
                            //* match: LZX_NUM_CHARS + ((slot<<3) | length_header (3 bits)) */
                            Dec(main_element, LZX_NUM_CHARS);

                            match_length := main_element and LZX_NUM_PRIMARY_LENGTHS;
                            if (match_length = LZX_NUM_PRIMARY_LENGTHS) then begin
                                if READ_HUFFSYM(@pState^.LengthTable.Table[0], @pState^.LengthTable.Len[0],
                                     bits, inpos, i, j, LZX_LENGTH_TABLEBITS,
                                     LZX_LENGTH_MAXSYMBOLS, length_footer) <> 0 then
                                begin
                                  Result := DECR_ILLEGALDATA;
                                  bits.Free;
                                  Exit;
                                end;
                                Inc(match_length, length_footer);
                            end;
                            Inc(match_length, LZX_MIN_MATCH);

                            match_offset := main_element shr 3;

                            if (match_offset > 2) then begin
                                //* not repeated offset */
                                extra := extra_bits[match_offset];
                                match_offset := position_base[match_offset] - 2;
                                if (extra > 3) then begin
                                    //* verbatim and aligned bits */
                                    Dec(extra, 3);
                                    verbatim_bits := bits.read(extra, inpos);
                                    Inc(match_offset, (verbatim_bits shl 3));
                                    if READ_HUFFSYM(@pState^.AlignedTAble.Table[0], @pState^.AlignedTAble.Len[0],
                                        bits, inpos, i, j, LZX_ALIGNED_TABLEBITS, LZX_ALIGNED_MAXSYMBOLS,
                                        aligned_bits) <> 0 then
                                    begin
                                      Result := DECR_ILLEGALDATA;
                                      bits.Free;
                                      Exit;
                                    end;
                                    Inc(match_offset, aligned_bits);
                                end
                                else if (extra = 3) then begin
                                    //* aligned bits only */
                                    if READ_HUFFSYM(@pState^.AlignedTAble.Table[0], @pState^.AlignedTAble.Len[0],
                                          bits, inpos, i, j, LZX_ALIGNED_TABLEBITS, LZX_ALIGNED_MAXSYMBOLS,
                                          aligned_bits) <> 0 then
                                    begin
                                      Result := DECR_ILLEGALDATA;
                                      bits.Free;
                                      Exit;
                                    end;
                                    Inc(match_offset, aligned_bits);
                                end
                                else if (extra > 0) then begin //* extra==1, extra==2 */
                                    //* verbatim bits only */
                                    verbatim_bits := bits.read(extra, inpos);
                                    Inc(match_offset, verbatim_bits);
                                end
                                else begin //* extra == 0 */
                                    //* ??? */
                                    match_offset := 1;
                                end;

                                //* update repeated offset LRU queue */
                                R2 := R1;
                                R1 := R0;
                                R0 := match_offset;
                            end
                            else if (match_offset = 0) then begin
                                match_offset := R0;
                            end
                            else if (match_offset = 1) then begin
                                match_offset := R1;
                                R1 := R0;
                                R0 := match_offset;
                            end
                            else begin //* match_offset == 2 */
                                match_offset := R2;
                                R2 := R0;
                                R0 := match_offset;
                            end;

                            rundest := window + window_posn;
                            runsrc  := rundest - match_offset;
                            Inc(window_posn, match_length);
                            if (window_posn > window_size) then begin
                              Result := DECR_ILLEGALDATA;
                              bits.Free;
                              Exit;
                            end;
                            Dec(this_run, match_length);

                            //* copy any wrapped around source data */
                            while ((runsrc < window) and (match_length > 0)) do begin
                                Dec(match_length);
                                rundest^ := (runsrc + window_size)^;
                                Inc(rundest);
                                Inc(runsrc);
                            end;
                            //* copy match data - no worries about destination wraps */
                            while (match_length > 0) do begin
                                Dec(match_length);
                                rundest^ := runsrc^;
                                Inc(rundest);
                                Inc(runsrc);
                            end;
                        end;
                    end;
                end;
                LZX_BLOCKTYPE_UNCOMPRESSED:
                begin
                    if ((inpos + this_run) > endinp) then begin
                        Result := DECR_ILLEGALDATA;
                        bits.Free;
                        Exit;
                    end;
                    Move(inpos^, (window + window_posn)^,  this_run);
                    Inc(inpos, this_run);
                    Inc(window_posn, this_run);
                end;
            else
              Result := DECR_ILLEGALDATA; ///* might as well */
              bits.Free;
              Exit;
            end;
            this_run := pState^.block_remaining;
        end;
    end;

    if (togo <> 0) then begin
        Result := DECR_ILLEGALDATA;
        bits.Free;
        Exit;
    end;
    if window_posn = 0 then
      Move((window + window_size - outlen)^, outpos^, outlen)
    else
      Move((window + window_posn - outlen)^, outpos^, outlen);

    pState^.window_posn := window_posn;
    pState^.R0 := R0;
    pState^.R1 := R1;
    pState^.R2 := R2;

    //* intel E8 decoding */
    if ((pState^.frames_read < 32768) and (pState^.intel_filesize <> 0)) then begin
        if (outlen <= 6 or not pState^.intel_started) then begin
            Inc(pState^.intel_curpos, outlen);
        end
        else begin
            data    := outpos;
            dataend := data + outlen - 10;
            curpos  := pState^.intel_curpos;
            filesize  := pState^.intel_filesize;

            pState^.intel_curpos := curpos + outlen;

            while (data < dataend) do begin
                if data^ <> $E8 then begin
                  Inc(curpos);
                  Inc(Data);
                  continue;
                end;
                Inc(Data);
                abs_off := data[0] or (data[1]shl 8) or (data[2]shl 16) or (data[3]shl 24);

                if (abs_off >= curpos-1) and (abs_off < filesize) then begin
                    if (abs_off >= 0) then
                        rel_off := abs_off - curpos
                    else
                        rel_off := abs_off + filesize;
                    {$IFDEF ENDIAN_BIG}
                    PLongWord(data)^ := Swap(rel_off);
                    {$ELSE}
                    PLongword(data)^ := rel_off;
                    {$ENDIF}
                end;
                Inc(data, 4);
                Inc(curpos, 5);
            end;
        end;
    end;
    Inc(pState^.frames_read);
    bits.Free;
    Result := DECR_OK;
end;

{ TBufBits }

procedure TBufBits.Init;
begin
  bitsleft := 0;
  bitbuf := 0;
end;

procedure TBufBits.ensure(num: LongInt; var inpos:PByte);
begin
  while (bitsleft < num) do begin
    bitbuf := bitbuf or (((inpos[1]shl 8) or inpos[0]) shl (ULONG_BITS-16 - bitsleft));
    Inc(bitsleft, 16);
    Inc(inpos, 2);
  end;
end;

function TBufBits.peek(numbits: LongInt): dword;
begin
  Result := bitbuf shr (ULONG_BITS - numbits);
end;

function TBufBits.remove(numbits: LongInt): dword;
begin
  bitbuf := bitbuf  shl numbits;
  Result := bitbuf;
  Dec(bitsleft, numbits);
end;

function TBufBits.read(numbits: LongInt; var inpos: PByte): dword;
begin
  ensure(numbits, inpos);
  Result := peek(numbits);
  remove(numbits);
end;

end.



