{ Renewed copyright, with permission of the author:
  Copyright (C) 2002 Matthew T. Russotto

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
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit paslznonslide;
{$MODE OBJFPC}

interface
{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}

{$DEFINE LAZY}
{$DEFINE DEBUG_LZ}

type

  u_char = Byte;
  Pu_char  = ^u_char;
  PPu_char = ^Pu_char;

     Plz_info = ^lz_info;


     get_chars_t = function (lzi:plz_info; n:longint; buf:pu_char):longint; cdecl;

     output_match_t = function (lzi:plz_info; match_pos:longint; match_len:longint):longint; cdecl;

     output_literal_t = procedure (lzi:plz_info; ch:u_char); cdecl;
  { window size in bytes  }
  { size of longest match in bytes  }
  { location within stream  }

  lz_info = record
          wsize : longint;
          max_match : longint;
          min_match : longint;
          block_buf : pu_char;
          block_bufe : pu_char;
          block_buf_size : longint;
          chars_in_buf : longint;
          cur_loc : longint;
          block_loc : longint;
          frame_size : longint;
          max_dist : longint;
          prevtab : ppu_char;
          lentab : plongint;
          eofcount : smallint;
          stop : smallint;
          analysis_valid : smallint;
          get_chars : get_chars_t;
          output_match : output_match_t;
          output_literal : output_literal_t;
          user_data : pointer;
       end;

  procedure lz_init(lzi:plz_info; wsize:longint; max_dist:longint; max_match:longint; min_match:longint;
              frame_size:longint; get_chars:get_chars_t; output_match:output_match_t; output_literal:output_literal_t; user_data:pointer);

  procedure lz_release(lzi:plz_info);

  procedure lz_reset(lzi:plz_info);

  procedure lz_stop_compressing(lzi:plz_info);

  function lz_left_to_process(lzi:plz_info):longint;

  { returns # chars read in but unprocessed  }
  function lz_compress(lzi:plz_info; nchars:longint):longint;


implementation
{$IFDEF DEBUG_LZ}
uses Sysutils;
{$ENDIF}

const
  MAX_MATCH = 253;
  MIN_MATCH = 2;

procedure lz_init(lzi:plz_info; wsize:longint; max_dist:longint; max_match:longint; min_match:longint;
              frame_size:longint; get_chars:get_chars_t; output_match:output_match_t; output_literal:output_literal_t; user_data:pointer);
begin
  { the reason for the separate max_dist value is LZX can't reach the
     first three characters in its nominal window.  But using a smaller
     window results in inefficiency when dealing with reset intervals
     which are the length of the nominal window }

  lzi^.wsize := wsize;
  if (max_match > wsize) then
    lzi^.max_match := wsize
  else
    lzi^.max_match := max_match;

  lzi^.min_match := min_match;
  if (lzi^.min_match < 3) then lzi^.min_match := 3;

  lzi^.max_dist := max_dist;
  lzi^.block_buf_size := wsize + lzi^.max_dist;
  lzi^.block_buf := GetMem(lzi^.block_buf_size);
  lzi^.block_bufe := lzi^.block_buf + lzi^.block_buf_size;
  
  
  //assert(lzi^.block_buf != NULL);

  lzi^.cur_loc := 0;
  lzi^.block_loc := 0;
  lzi^.chars_in_buf := 0;
  lzi^.eofcount := 0;
  lzi^.get_chars := get_chars;
  lzi^.output_match := output_match;
  lzi^.output_literal := output_literal;
  lzi^.user_data := user_data;
  lzi^.frame_size := frame_size;
  lzi^.lentab := AllocMem(sizeof(longint)* lzi^.block_buf_size);
  lzi^.prevtab := AllocMem(sizeof(pu_char)* lzi^.block_buf_size);
  lzi^.analysis_valid := 0;
end;

procedure lz_release(lzi:plz_info);
begin
  freemem(lzi^.block_buf);
  freemem(lzi^.lentab);
  freemem(lzi^.prevtab);
end;

procedure lz_reset(lzi: plz_info);
var
  residual: longint;
  
begin
  residual := lzi^.chars_in_buf - lzi^.block_loc;
  move(PByte(lzi^.block_buf)[lzi^.block_loc], lzi^.block_buf[0], residual);

  lzi^.chars_in_buf := residual;
  lzi^.block_loc := 0;
  lzi^.analysis_valid := 0;
end;

function lz_left_to_process(lzi: plz_info): longint;
begin
  lz_left_to_process := lzi^.chars_in_buf - lzi^.block_loc;
end;

procedure fill_blockbuf(lzi: plz_info; maxchars: longint);
var
  toread: longint;
  readhere: pu_char;
  nread: longint;
begin
  if (lzi^.eofcount <> 0) then exit;
  Dec(maxchars, lz_left_to_process(lzi));
  toread := lzi^.block_buf_size - lzi^.chars_in_buf;
  if (toread > maxchars) then toread := maxchars;
  readhere := lzi^.block_buf + lzi^.chars_in_buf;
  nread := lzi^.get_chars(lzi, toread, readhere);
  Inc(lzi^.chars_in_buf, nread);
  if (nread <> toread) then
    Inc(lzi^.eofcount);
end;

procedure lz_analyze_block(lzi: plz_info);
var
  lenp,
  lentab: plongint;
  prevtab, prevp: PPu_char;
  bbp, bbe: Pu_char;
  chartab: array [0..255] of pu_char;
  cursor: pu_char;
  prevlen,
  ch,
  maxlen: longint;
  maxcursor: PtrUInt;
  wasinc: Boolean;
  max_dist: longint;
  I: longint;
begin
  max_dist := lzi^.max_dist;

  FillChar(chartab[0], sizeof(chartab), 0);

  prevtab := lzi^.prevtab;
  prevp := prevtab;
  lentab := lzi^.lentab;
  lenp := lentab;

  FillChar(prevtab[0], sizeof(prevtab) * lzi^.chars_in_buf, 0);
  FillChar(lentab[0], sizeof(prevtab) * lzi^.chars_in_buf, 0);
  
  bbp := lzi^.block_buf;
  bbe := bbp + lzi^.chars_in_buf;
  while (bbp < bbe) do begin
    ch := bbp^;
    if (chartab[ch] <> nil) then begin
      prevp^ := chartab[ch];
      lenp^ := 1;
    end;
    chartab[ch] := bbp;
    Inc(bbp);
    Inc(prevp);
    Inc(lenp);
  end;

  for maxlen := 1 to lzi^.max_match-1 do begin
    wasinc := False;
    bbp := bbe - maxlen;
    lenp := lentab + lzi^.chars_in_buf - maxlen;
    prevp := prevtab + lzi^.chars_in_buf - maxlen;

    //for I := 0 to (bbp-2 - lzi^.block_buf) do begin // we don't use the value of i
    while (bbp > lzi^.block_buf) do begin
      Dec(bbp);
      Dec(prevp);
      Dec(lenp);
      if lenp^ = maxlen then begin
	ch := bbp[maxlen];
	cursor := prevp^;
        while (cursor <> nil) and ((bbp - cursor) <= max_dist) do begin
	  prevlen := (cursor - lzi^.block_buf + lentab)^;
	  if (cursor[maxlen] = ch) then begin
	    prevp^ := cursor;
	    Inc(lenp^);
            wasinc := True;
	    break;
          end;

	  if (prevlen <> maxlen) then break;
          cursor := (cursor - lzi^.block_buf + prevtab)^;
        end;
      end;
    end;
    if not wasinc then break;
  end;

  lzi^.analysis_valid := 1;
end;

procedure lz_stop_compressing(lzi:plz_info);
begin
    lzi^.stop := 1;
end;

function lz_compress(lzi:plz_info; nchars:longint):longint;
var
  bbp, bbe: pu_char;
  lentab, lenp: plongint;
  prevtab, prevp: ppu_char;
  len: longint;
  holdback: longint;
  trimmed: smallint;
  residual: longint;
  bytes_to_move: longint;
begin
  lzi^.stop := 0;
  while ((lz_left_to_process(lzi) <> 0) or  (lzi^.eofcount =0)) and ((lzi^.stop =0) and (nchars > 0)) do begin
    if (lzi^.analysis_valid = 0)
    or ((lzi^.eofcount =0) and (lzi^.chars_in_buf- lzi^.block_loc < nchars)) then begin
      residual := lzi^.chars_in_buf - lzi^.block_loc;
      bytes_to_move := lzi^.max_dist + residual;
      if (bytes_to_move > lzi^.chars_in_buf) then
	bytes_to_move := lzi^.chars_in_buf;
 
      move(PByte(lzi^.block_buf)[lzi^.chars_in_buf - bytes_to_move], lzi^.block_buf, bytes_to_move);

      lzi^.block_loc := bytes_to_move - residual;
      lzi^.chars_in_buf := bytes_to_move;
      fill_blockbuf(lzi, nchars);
      lz_analyze_block(lzi);
    end;
    prevp := lzi^.prevtab + lzi^.block_loc;
    prevtab := prevp;
    lenp := lzi^.lentab + lzi^.block_loc;
    lentab := lenp;
    bbp := lzi^.block_buf + lzi^.block_loc;
    holdback := lzi^.max_match;
    if (lzi^.eofcount <> 0) then holdback := 0;
    if (lzi^.chars_in_buf < (nchars + lzi^.block_loc)) then
      bbe := lzi^.block_buf + lzi^.chars_in_buf - holdback
    else
      bbe := bbp + nchars;
    while ((bbp < bbe) and (lzi^.stop = 0)) do begin
      trimmed := 0;
      len := lenp^;
      if ((lzi^.frame_size <> 0) and (len > (lzi^.frame_size - lzi^.cur_loc mod lzi^.frame_size))) then begin
	trimmed := 1;
	len := (lzi^.frame_size - lzi^.cur_loc mod lzi^.frame_size);
      end;
      if (len > nchars) then begin
	trimmed := 1;
	len := nchars;
      end;
      if (len >= lzi^.min_match) then begin
{$ifdef LAZY}
	if ((bbp < bbe -1) and (trimmed = 0) and
	    ((lenp[1] > (len + 1)))) then begin
	  len := 1;
	  //* this is the lazy eval case */
        end
	else
{$endif}
	  if (lzi^.output_match(lzi, (prevp^ - lzi^.block_buf) - lzi^.block_loc, len) < 0) then begin
	    len := 1; //* match rejected */
          end;
      end
      else
	len := 1;

      if (len < lzi^.min_match) then begin
	//assert(len == 1);
	lzi^.output_literal(lzi, bbp^);
      end;
      Inc(bbp,len);
      Inc(prevp, len);
      Inc(lenp, len);
      Inc(lzi^.cur_loc, len);
      Inc(lzi^.block_loc, len);
      
      //assert(nchars >= len);

      Dec(nchars, len);

    end;
  end;
  lz_compress := 0;
end;

end.
