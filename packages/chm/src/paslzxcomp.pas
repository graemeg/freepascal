{ Copyright (C) <2005> <Andrew Haines> paslzxcomp.pas

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
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit paslzxcomp;
{$MODE OBJFPC}
{$GOTO ON}
interface

uses paslznonslide;

  const
     MIN_MATCH = 2;
     MAX_MATCH = 257;
     NUM_CHARS = 256;
     NUM_PRIMARY_LENGTHS = 7;
     NUM_SECONDARY_LENGTHS = 249;
  { the names of these constants are specific to this library  }
     LZX_MAX_CODE_LENGTH = 16;
     LZX_FRAME_SIZE = 32768;
     LZX_PRETREE_SIZE = 20;
     LZX_ALIGNED_BITS = 3;
     LZX_ALIGNED_SIZE = 8;
     LZX_VERBATIM_BLOCK = 1;
     LZX_ALIGNED_OFFSET_BLOCK = 2;


{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}


  {
      File lzx_compress.h, part of lzxcomp library
      Copyright (C) 2002 Matthew T. Russotto
  
      This program is free software; you can redistribute it and/or modify
      it under the terms of the GNU Lesser General Public License as published by
      the Free Software Foundation; version 2.1 only
  
      This program is distributed in the hope that it will be useful,
      but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      GNU Lesser General Public License for more details.
  
      You should have received a copy of the GNU Lesser General Public License
      along with this program; if not, write to the Free Software
      Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   }
   type
  PPlzx_data = ^Plzx_data;
  Plzx_data  = ^lzx_data;


     TGetBytesFunc = function (arg:pointer; n:longint; buf:pointer):longint; cdecl;

     TWriteBytesFunc = function (arg:pointer; n:longint; buf:pointer):longint; cdecl;

     TMarkFrameFunc = procedure (arg:pointer; uncomp:dword; comp:dword); cdecl;

     TIsEndOfFileFunc = function (arg:pointer): longbool; cdecl;
  { add more here? Error codes, # blocks, # frames, etc?  }

     lzx_results = record
          len_compressed_output : longint;
          len_uncompressed_input : longint;
       end;
       
  phuff_entry = ^huff_entry;
  huff_entry = record
   codelength: smallint;
   code: word;
  end;
       
  lzx_data = record
    in_arg : pointer;
    out_arg: pointer;
    mark_frame_arg: pointer;
    get_bytes: TGetBytesFunc;
    at_eof: TIsEndOfFileFunc;
    put_bytes: TWriteBytesFunc;
    mark_frame: TMarkFrameFunc;
    lzi: plz_info;
    {/* a 'frame' is an 0x8000 byte thing.  Called that because otherwise
     I'd confuse myself overloading 'block' */}
    left_in_frame: longint;
    left_in_block: longint;
    R0, R1, R2: longint;
    num_position_slots: longint;
    //* this is the LZX block size */
    block_size: longint;
    main_freq_table: plongint;
    length_freq_table: array [0..NUM_SECONDARY_LENGTHS-1] of longint;
    aligned_freq_table: array [0..LZX_ALIGNED_SIZE-1] of longint;
    block_codes: plongword;
    block_codesp: plongword;
    main_tree: phuff_entry;
    length_tree: array[0..NUM_SECONDARY_LENGTHS-1] of huff_entry;
    aligned_tree: array[0..LZX_ALIGNED_SIZE-1] of huff_entry;
    main_tree_size: longint;
    bit_buf: word;
    bits_in_buf: longint;
    main_entropy: double;
    last_ratio: double;
    prev_main_treelengths: pbyte;
    prev_length_treelengths: array [0..NUM_SECONDARY_LENGTHS-1] of byte;
    len_uncompressed_input: longword;
    len_compressed_output: longword;
    need_1bit_header: smallint;
    subdivide: smallint; //* 0 = don't subdivide, 1 = allowed, -1 = requested */
  end;
  Plzx_results  = ^lzx_results;

  function lzx_init(lzxdp:Pplzx_data; wsize_code:longint; get_bytes:TGetBytesFunc; get_bytes_arg:pointer; at_eof:TIsEndOfFileFunc;
             put_bytes:TWriteBytesFunc; put_bytes_arg:pointer; mark_frame:TMarkFrameFunc; mark_frame_arg:pointer):longint;

  procedure lzx_reset(lzxd:plzx_data);

  function lzx_compress_block(lzxd:plzx_data; block_size:longint; subdivide: LongBool):longint;

  function lzx_finish(lzxd:plzx_data; lzxr:plzx_results):longint;

implementation
uses math, sysutils;
var
  rloge2: double; // set in initialization section
  
const
  num_position_slots: array [0..6] of smallint = (30, 32, 34, 36, 38, 42, 50);
  
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
  pih_elem = ^ih_elem;
  ih_elem = record
    freq: longint;
    sym: smallint;
    pathlength: smallint;
    parent: pih_elem;
    left: pih_elem;
    right: pih_elem;
  end;
  ph_elem = ^h_elem;
  h_elem = record
    freq: longint;
    sym: smallint;
    pathlength: smallint;
    parent: pih_elem;
    code: word;
  end;

function cmp_leaves(const in_a: ph_elem; const in_b: ph_elem): longint;
begin

  if (in_a^.freq = 0) and (in_b^.freq <> 0) then
    Exit(1);
  if (in_a^.freq <> 0) and (in_b^.freq = 0) then
    Exit(-1);

  if (in_a^.freq = in_b^.freq) then
    Exit(in_a^.sym - in_b^.sym);

  Exit(in_a^.freq - in_b^.freq);
end;

function cmp_pathlengths(const in_a: ph_elem; const in_b: ph_elem): longint;
begin
  if (in_a^.pathlength = in_b^.pathlength) then
  //* see note on canonical pathlengths */
    Exit(in_b^.sym - in_a^.sym);

  Exit(in_b^.pathlength - in_a^.pathlength);
end;

type
  TQSortCompFunc = function(const in_a: ph_elem; const in_b: ph_elem): longint;

procedure qsort(a_array: ph_elem; nelem: integer; cmpfunc: TQSortCompFunc);

var
  tmp: h_elem;

  procedure QuickSort(L, R: Integer);
  var
    I, J, Pivot: Integer;
  begin
    repeat
      I := L;
      J := R;
      Pivot := (L + R) div 2;
      repeat
        while cmpfunc(@a_array[I], @a_array[Pivot]) < 0 do Inc(I);
        while cmpfunc(@a_array[J], @a_array[Pivot]) > 0 do Dec(J);
        if I <= J then
        begin
          // exchange I and J
          tmp := a_array[I];
          a_array[I] := a_array[J];
          a_array[J] := tmp;

          if Pivot = I then
            Pivot := J
          else if Pivot = J then
            Pivot := I;
          Inc(I);
          Dec(j);
        end;
      until I > J;
      if L < J then
        QuickSort(L,J);
      L := I;
    until I >= R;
  end;
begin
  QuickSort(0, nelem - 1);
end;

procedure build_huffman_tree(nelem: longint; max_code_length: longint; freq: plongint; tree: phuff_entry);
var
  leaves: ph_elem;
  inodes: pih_elem;
  next_inode: pih_elem;
  cur_inode: pih_elem;
  cur_leaf :ph_elem;
  leaves_left,
  nleaves,
  pathlength: longint;
  cur_code: word;
  codes_too_long: smallint = 0;
  f1, f2: pih_elem;
  i: longint;
begin
  leaves := GetMem(nelem * sizeof(h_elem));
  for i := 0 to nelem-1 do begin
    leaves[i].freq := freq[i];
    leaves[i].sym := i;
    leaves[i].pathlength := 0;
  end;

  qsort(leaves, nelem, @cmp_leaves);


  leaves_left := 0;
  while leaves_left < nelem do begin
    if (leaves[leaves_left].freq) = 0 then break;
    Inc(leaves_left);
  end;
  nleaves := leaves_left;

  if (nleaves >= 2) then begin
    inodes := AllocMem((nelem-1) * sizeof(ih_elem));
    repeat
      if (codes_too_long <> 0) then begin
        leaves_left := 0;
        while leaves_left < nelem do begin
          if (leaves[leaves_left].freq = 0) then break;
	  if (leaves[leaves_left].freq <> 1) then begin
            leaves[leaves_left].freq := leaves[leaves_left].freq shr 1;
            codes_too_long := 0;
<<<<<<< HEAD
<<<<<<< HEAD
          end;
          Inc(leaves_left);
=======
            Inc(leaves_left);
          end;

>>>>>>> graemeg/fixes_2_2
=======
            Inc(leaves_left);
          end;

>>>>>>> origin/fixes_2_2
        end;
        if codes_too_long <> 0 then
          raise Exception.Create('!codes_too_long');
      end;

      cur_leaf := leaves;
      cur_inode := inodes;
      next_inode := cur_inode;

      repeat
	f1 := nil;
        f2 := nil;
	if (leaves_left <> 0) and
	    ((cur_inode = next_inode) or
	     (cur_leaf^.freq <= cur_inode^.freq)) then begin
          f1 := pih_elem(cur_leaf);
          Inc(cur_leaf);
	  Dec(leaves_left);
        end
	else if (cur_inode <> next_inode) then begin
	  f1 := cur_inode;
          Inc(cur_inode);
        end;

	if ((leaves_left <> 0) and
	    ((cur_inode = next_inode) or
	     (cur_leaf^.freq <= cur_inode^.freq))) then begin
          f2 := pih_elem(cur_leaf);
          Inc(cur_leaf);
	  Dec(leaves_left);
        end
	else if (cur_inode <> next_inode) then begin
          f2 := cur_inode;
          Inc(cur_inode);
        end;

	if (f1 <> nil) and (f2 <> nil) then begin
	  next_inode^.freq := f1^.freq + f2^.freq;
	  next_inode^.sym := -1;
	  next_inode^.left := f1;
	  next_inode^.right := f2;
	  next_inode^.parent := nil;
	  f1^.parent := next_inode;
	  f2^.parent := next_inode;
	  if (f1^.pathlength > f2^.pathlength) then
	    next_inode^.pathlength := f1^.pathlength + 1
	  else
	    next_inode^.pathlength := f2^.pathlength + 1;
	  if (next_inode^.pathlength > max_code_length) then begin
	    codes_too_long := 1;
	    break;
          end;
          Inc(next_inode);
        end;
      until (f1 = nil) and (f2 = nil);
    until codes_too_long = 0;

    //* now traverse tree depth-first */
    cur_inode := next_inode - 1;
    pathlength := 0;
    cur_inode^.pathlength := -1;
    repeat
      //* precondition: at unmarked node*/
      if (cur_inode^.sym = -1) then begin //*&& (cur_inode^.left)*/
	//* left node of unmarked node is unmarked */
	cur_inode := cur_inode^.left;
	cur_inode^.pathlength := -1;
        Inc(pathlength);
      end
      else begin
	//* mark node */
	cur_inode^.pathlength := pathlength;
//#if 0
//	if (cur_inode^.right) {
//	  /* right node of previously unmarked node is unmarked */
//	  cur_inode = cur_inode^.right;
//	  cur_inode^.pathlength = -1;
//	  pathlength++;
//	}
//	else
//#endif
          begin

	    //* time to come up.  Keep coming up until an unmarked node is reached */
	    //* or the tree is exhausted */
            repeat
	      cur_inode := cur_inode^.parent;
	      Dec(pathlength);

	    //while (cur_inode && (cur_inode^.pathlength != -1));
            until (cur_inode = nil) or (cur_inode^.pathlength = -1);
	    if (cur_inode <> nil) then begin
	      //* found unmarked node; mark it and go right */
	      cur_inode^.pathlength := pathlength;
	      cur_inode := cur_inode^.right;
	      cur_inode^.pathlength := -1;
	      Inc(pathlength);
	      //* would be complex if cur_inode could be null here.  It can't */
            end
          end;
      end;
    until cur_inode = nil;

    freemem(inodes);

    ///* the pathlengths are already in order, so this sorts by symbol */
    qsort(leaves, nelem, @cmp_pathlengths);

//#if 0
//    pathlength = leaves[0].pathlength;
//    cur_code = 0;
//    for (i = 0; i < nleaves; i++) {
//      while (leaves[i].pathlength < pathlength) {
// (!(cur_code & 1));
//	cur_code >>= 1;
//	pathlength--;
//      }
//      leaves[i].code = cur_code;
//      cur_code++;
//    }
//#else
    pathlength := leaves[nleaves-1].pathlength;
    if leaves[0].pathlength > 16  then
      raise Exception.Create('leaves[0].pathlength <= 16');
    //* this method cannot deal with bigger codes, though
    //					   the other canonical method can in some cases
    //					   (because it starts with zeros ) */
    cur_code := 0;
    for i := nleaves-1 downto 0 do begin
      while (leaves[i].pathlength > pathlength) do begin
        cur_code := cur_code shl 1;
	Inc(pathlength);
      end;
      leaves[i].code := cur_code;
<<<<<<< HEAD
<<<<<<< HEAD
      {$PUSH}
      {$R-}
      Inc(cur_code); // range error but i = 0 so it's harmless
      {$POP}
=======
      Inc(cur_code);
>>>>>>> graemeg/fixes_2_2
=======
      Inc(cur_code);
>>>>>>> origin/fixes_2_2
    end;
//#endif

  end
  else if (nleaves = 1) then begin
    //* 0 symbols is OK (not according to doc, but according to Caie) */
    //* but if only one symbol is present, two symbols are required */
    nleaves := 2;
    leaves[0].pathlength := 1;
    leaves[1].pathlength := 1;
    if (leaves[1].sym > leaves[0].sym) then begin
      leaves[1].code := 1;
      leaves[0].code := 0;
    end
    else begin
      leaves[0].code := 1;
      leaves[1].code := 0;
    end;
  end;

  Fillchar(tree^, nelem * sizeof(huff_entry), 0);
  for i := 0 to nleaves-1 do begin
    tree[leaves[i].sym].codelength := leaves[i].pathlength;
    tree[leaves[i].sym].code := leaves[i].code;
  end;

  freemem(leaves);
end;

function lzx_get_chars(lzi: plz_info; n: longint; buf: pbyte): longint; cdecl;
var
  //* force lz compression to stop after every block */
  chars_read,
  chars_pad: longint;

  lzud: plzx_data;
begin
  lzud := plzx_data(lzi^.user_data);
  
  chars_read := lzud^.get_bytes(lzud^.in_arg, n, buf);
  Dec(lzud^.left_in_frame, chars_read mod LZX_FRAME_SIZE);
  if (lzud^.left_in_frame < 0) then
    Inc(lzud^.left_in_frame, LZX_FRAME_SIZE);

  if ((chars_read < n) and (lzud^.left_in_frame <> 0)) then begin
    chars_pad := n - chars_read;
    if (chars_pad > lzud^.left_in_frame) then chars_pad := lzud^.left_in_frame;
    //*  never emit a full frame of padding.  This prevents silliness when
    //   lzx_compress is called when at EOF but EOF not yet detected */
    if (chars_pad = LZX_FRAME_SIZE) then chars_pad := 0;
    FillChar(buf[chars_read], chars_pad, 0);
    Dec(lzud^.left_in_frame, chars_pad);
    Inc(chars_read, chars_pad);
  end;
  lzx_get_chars := chars_read;
end;

function find_match_at(lzi: plz_info; loc: longint; match_len: longint; match_locp: plongint): longint;
var
  matchb,
  nmatchb,
  c1, c2: pbyte;
  j: longint;
begin
  if -match_locp^ = loc then Exit(-1);
  if loc < match_len then Exit(-1);

  matchb := lzi^.block_buf + lzi^.block_loc + match_locp^;
  nmatchb := lzi^.block_buf + lzi^.block_loc - loc;
  c1 := matchb;
  c2 := nmatchb;
  j := 0;
  while j < match_len do begin
    if c1^ <> c2^ then begin
      break;
    end;
    Inc(c1);
    Inc(c2);
    Inc(j);
  end;
  
  if (j = match_len) then begin
    match_locp^ := -loc;
    Exit(0);
  end;
  Exit(-1);
end;

procedure check_entropy(lzud: plzx_data; main_index: longint);
var
    freq,
    n_ln_n,
    rn_ln2,
    cur_ratio: double;
    n: longint;
begin
    //* delete old entropy accumulation */
    if (lzud^.main_freq_table[main_index] <> 1) then begin
      freq := double(lzud^.main_freq_table[main_index])-1;
      lzud^.main_entropy := lzud^.main_entropy + (freq * ln(freq));
    end;
    //* add new entropy accumulation */
    freq := double(lzud^.main_freq_table[main_index]);
    lzud^.main_entropy := lzud^.main_entropy - (freq * ln(freq));
    n := lzud^.block_codesp - lzud^.block_codes;

    if (((n and $0FFF) = 0) and (lzud^.left_in_block >= $1000)) then begin
      n_ln_n := (double(n) * ln(double(n)));
      rn_ln2 := (rloge2 / double(n));
      cur_ratio := (n * rn_ln2 *(n_ln_n + lzud^.main_entropy) + 24 + 3 * 80 + NUM_CHARS + (lzud^.main_tree_size-NUM_CHARS)*3 + NUM_SECONDARY_LENGTHS ) / double(n);

      if (cur_ratio > lzud^.last_ratio) then begin
        lzud^.subdivide := -1;
        lz_stop_compressing(lzud^.lzi);
      end;
      lzud^.last_ratio := cur_ratio;

    end;

end;

function lzx_output_match(lzi: plz_info; match_pos, match_len: longint): longint; cdecl;
var
  lzud: plzx_data;
  formatted_offset,
  position_footer: longword;
  length_footer,
  length_header: byte;
  len_pos_header: word;
  position_slot: longint;
  btdt: smallint;
  left, right, mid: longint;
label testforr;
begin
  lzud := plzx_data(lzi^.user_data);

  position_footer := 0;
  btdt := 0;
 testforr:
  if (match_pos = -lzud^.R0) then begin
    match_pos := 0;
    formatted_offset := 0;
    position_slot := 0;
  end
  else if (match_pos = -lzud^.R1) then begin
    lzud^.R1 := lzud^.R0;
    lzud^.R0 := -match_pos;
    match_pos := 1;
    formatted_offset := 1;
    position_slot := 1;
  end
  else if (match_pos = -lzud^.R2) then begin
    lzud^.R2 := lzud^.R0;
    lzud^.R0 := -match_pos;
    match_pos := 2;
    formatted_offset := 2;
    position_slot := 2;
  end
  else begin
    if (btdt = 0) then begin
      btdt := 1;
      if (find_match_at(lzi, lzud^.R0, match_len, @match_pos) = 0) then
	goto testforr;
      if (find_match_at(lzi, lzud^.R1, match_len, @match_pos) = 0) then
	goto testforr;
      if (find_match_at(lzi, lzud^.R2, match_len, @match_pos) = 0) then
        goto testforr;
    end;

    formatted_offset := -match_pos + 2;

    if ((match_len < 3) or
	((formatted_offset >= 64) and (match_len < 4)) or
	((formatted_offset >= 2048) and (match_len < 5)) or
	((formatted_offset >= 65536) and (match_len < 6))) then begin
      //* reject matches where extra_bits will likely be bigger than just outputting
      //  literals.  The numbers are basically derived through guessing
      //  and trial and error */
      Exit(-1); //* reject the match */
    end;

    lzud^.R2 := lzud^.R1;
    lzud^.R1 := lzud^.R0;
    lzud^.R0 := -match_pos;

  ///* calculate position base using binary search of table; if log2 can be
  //   done in hardware, approximation might work;
  //   trunc(log2(formatted_offset*formatted_offset)) gets either the proper
  //   position slot or the next one, except for slots 0, 1, and 39-49

  //   Slots 0-1 are handled by the R0-R1 procedures

  //   Slots 36-49 (formatted_offset >= 262144) can be found by
  //   (formatted_offset/131072) + 34 ==
  //   (formatted_offset >> 17) + 34;
  //*/
    if (formatted_offset >= 262144) then begin
      position_slot := (formatted_offset shr 17) + 34;
    end
    else begin
      left := 3;
      right := lzud^.num_position_slots - 1;
      position_slot := -1;
      while (left <= right) do begin
	mid := (left + right) div 2;
	if (position_base[mid] <= formatted_offset) and
	    (position_base[mid+1] > formatted_offset) then begin
	  position_slot := mid;
	  break;
        end;
	if (formatted_offset > position_base[mid]) then
	  //* too low */
	  left := mid + 1
	else //* too high */
	  right := mid;
      end;
      if not(position_slot >= 0) then
      raise Exception.Create('position_slot >= 0');

      //* FIXME precalc extra_mask table */
    end;
    position_footer := ((LongWord(1) shl extra_bits[position_slot]) - 1) and formatted_offset;
  end;

  //* match length = 8 bits */
  //* position_slot = 6 bits */
  //* position_footer = 17 bits */
  //* total = 31 bits */
  //* plus one to say whether it's a literal or not */
  lzud^.block_codesp^ := $80000000 or //* bit 31 in intelligent bit ordering */
    (position_slot shl 25) or //* bits 30-25 */
    (position_footer shl 8) or //* bits 8-24 */
    (match_len - MIN_MATCH); //* bits 0-7 */
  Inc(lzud^.block_codesp);

  if (match_len < (NUM_PRIMARY_LENGTHS + MIN_MATCH)) then begin
    length_header := match_len - MIN_MATCH;
    //*    length_footer = 255; */ /* not necessary */
  end
  else begin
    length_header := NUM_PRIMARY_LENGTHS;
    length_footer := match_len - (NUM_PRIMARY_LENGTHS + MIN_MATCH);
    Inc(lzud^.length_freq_table[length_footer]);
  end;
  len_pos_header := (position_slot shl 3) or length_header;
  Inc(lzud^.main_freq_table[len_pos_header + NUM_CHARS]);
  if (extra_bits[position_slot] >= 3) then begin
    Inc(lzud^.aligned_freq_table[position_footer and 7]);
  end;

  Dec(lzud^.left_in_block, match_len);

  if (lzud^.subdivide <> 0) then
    check_entropy(lzud, len_pos_header + NUM_CHARS);
  Exit(0); ///* accept the match */
end;

procedure lzx_output_literal(lzi: plz_info; ch: byte); cdecl;
var
  lzud: plzx_data;
begin
  lzud := plzx_data(lzi^.user_data);

  Dec(lzud^.left_in_block);
  lzud^.block_codesp^ := ch;
  Inc(lzud^.block_codesp);
  Inc(lzud^.main_freq_table[ch]);
  if (lzud^.subdivide <> 0) then
    check_entropy(lzud, ch);
end;

procedure lzx_write_bits(lzxd: plzx_data; nbits: longint; bits: longword); cdecl;
var
  cur_bits,
  shift_bits,
  rshift_bits: longint;
  mask_bits: word;
begin
  cur_bits := lzxd^.bits_in_buf;
  while ((cur_bits + nbits) >= 16) do begin
    shift_bits := 16 - cur_bits;
    rshift_bits := nbits - shift_bits;
    if (shift_bits = 16) then begin
      lzxd^.bit_buf := (bits shr rshift_bits) and $FFFF;
    end
    else begin
      mask_bits := (1 shl shift_bits) - 1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      lzxd^.bit_buf := word(lzxd^.bit_buf shl shift_bits);
      lzxd^.bit_buf := word(lzxd^.bit_buf or (bits shr rshift_bits) and mask_bits);
    end;
{$IFDEF ENDIAN_BIG}
    lzxd^.bit_buf := word(((lzxd^.bit_buf and $FF)shl 8) or (lzxd^.bit_buf shr 8));
=======
=======
>>>>>>> origin/fixes_2_2
      lzxd^.bit_buf := lzxd^.bit_buf shl shift_bits;
      lzxd^.bit_buf := lzxd^.bit_buf or (bits shr rshift_bits) and mask_bits;
    end;
{$IFDEF ENDIAN_BIG}
    lzxd^.bit_buf := ((lzxd^.bit_buf and $FF)shl 8) or (lzxd^.bit_buf shr 8);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
      lzxd^.bit_buf := word(lzxd^.bit_buf shl shift_bits);
      lzxd^.bit_buf := word(lzxd^.bit_buf or (bits shr rshift_bits) and mask_bits);
    end;
{$IFDEF ENDIAN_BIG}
    lzxd^.bit_buf := word(((lzxd^.bit_buf and $FF)shl 8) or (lzxd^.bit_buf shr 8));
>>>>>>> origin/fixes_2.4
{$ENDIF}
    lzxd^.put_bytes(lzxd^.out_arg, sizeof(lzxd^.bit_buf), @lzxd^.bit_buf);
    Inc(lzxd^.len_compressed_output, sizeof(lzxd^.bit_buf));
    lzxd^.bit_buf := 0;
    Dec(nbits, shift_bits);
    cur_bits := 0;
  end;
  //* (cur_bits + nbits) < 16.  If nbits := 0, we're done.
  //   otherwise move bits in */
  shift_bits := nbits;
  mask_bits := (1 shl shift_bits) - 1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  lzxd^.bit_buf := word(lzxd^.bit_buf shl shift_bits);
  lzxd^.bit_buf := word(lzxd^.bit_buf or bits and mask_bits);
=======
  lzxd^.bit_buf := lzxd^.bit_buf shl shift_bits;
  lzxd^.bit_buf := lzxd^.bit_buf or bits and mask_bits;
>>>>>>> graemeg/fixes_2_2
=======
  lzxd^.bit_buf := lzxd^.bit_buf shl shift_bits;
  lzxd^.bit_buf := lzxd^.bit_buf or bits and mask_bits;
>>>>>>> origin/fixes_2_2
=======
  lzxd^.bit_buf := word(lzxd^.bit_buf shl shift_bits);
  lzxd^.bit_buf := word(lzxd^.bit_buf or bits and mask_bits);
>>>>>>> origin/fixes_2.4
  Inc(cur_bits, nbits);

  lzxd^.bits_in_buf := cur_bits;
end;

procedure lzx_align_output(lzxd: plzx_data);
begin
  if (lzxd^.bits_in_buf <> 0) then begin
    lzx_write_bits(lzxd, 16 - lzxd^.bits_in_buf, 0);
  end;
  if (lzxd^.mark_frame <> nil) then
    lzxd^.mark_frame(lzxd^.mark_frame_arg, lzxd^.len_uncompressed_input, lzxd^.len_compressed_output);
end;

procedure lzx_write_compressed_literals(lzxd: plzx_data; block_type: longint);
var
  cursor: plongword;
  endp: plongword;
  position_slot: word;
  position_footer,
  match_len_m2, //* match length minus 2, which is MIN_MATCH */
  verbatim_bits,
  block_code: longword;
  length_header,
  length_footer,
  len_pos_header: word;
  huffe: phuff_entry;
  frame_count: longint;
begin
  cursor := lzxd^.block_codes;
  endp := lzxd^.block_codesp;
  frame_count := (lzxd^.len_uncompressed_input mod LZX_FRAME_SIZE);

  Dec(lzxd^.len_uncompressed_input, frame_count); //* will be added back in later */
  while (cursor < endp) do begin
    block_code := cursor^;
    Inc(cursor);
    if (block_code and $80000000) <> 0 then begin
      {*
       *    0x80000000 |                bit 31 in intelligent bit ordering
       * (position_slot shl 25) |        bits 30-25
       * (position_footer shl 8) |       bits 8-24
       * (match_len - MIN_MATCH);       bits 0-7
       *
       *}

      match_len_m2 := block_code and $FF; //* 8 bits */
      position_footer := (block_code shr 8)and $1FFFF; //* 17 bits */
      position_slot := (block_code shr 25) and $3F; //* 6 bits */

      if (match_len_m2 < NUM_PRIMARY_LENGTHS) then begin
	length_header := match_len_m2;
	length_footer := 255; //* personal encoding for NULL */
      end
      else begin
	length_header := NUM_PRIMARY_LENGTHS;
	length_footer := match_len_m2 - NUM_PRIMARY_LENGTHS;
      end;
      len_pos_header := (position_slot shl 3) or length_header;
      huffe := @lzxd^.main_tree[len_pos_header+NUM_CHARS];
      lzx_write_bits(lzxd, huffe^.codelength, huffe^.code);
      if (length_footer <> 255) then begin
	huffe := @lzxd^.length_tree[length_footer];
	lzx_write_bits(lzxd, huffe^.codelength, huffe^.code);
      end;
      if ((block_type = LZX_ALIGNED_OFFSET_BLOCK) and (extra_bits[position_slot] >= 3)) then begin
	//* aligned offset block and code */
	verbatim_bits := position_footer shr 3;
	lzx_write_bits(lzxd, extra_bits[position_slot] - 3, verbatim_bits);
	huffe := @lzxd^.aligned_tree[position_footer and 7];
	lzx_write_bits(lzxd, huffe^.codelength, huffe^.code);
      end
      else begin
	verbatim_bits := position_footer;
	lzx_write_bits(lzxd, extra_bits[position_slot], verbatim_bits);
      end;
      Inc(frame_count, match_len_m2 + 2);
    end
    else begin
      //* literal */
      if not(block_code < NUM_CHARS) then
      raise Exception.Create('block_code < NUM_CHARS');
      
      huffe := @lzxd^.main_tree[block_code];
      lzx_write_bits(lzxd, huffe^.codelength, huffe^.code);
      Inc(frame_count);
    end;
    if (frame_count = LZX_FRAME_SIZE) then begin
      Inc(lzxd^.len_uncompressed_input, frame_count);
      lzx_align_output(lzxd);
      frame_count := 0;
    end;
    if not(frame_count < LZX_FRAME_SIZE) then
    raise Exception.Create('frame_count < LZX_FRAME_SIZE');
  end;
  Inc(lzxd^.len_uncompressed_input, frame_count);
end;

function lzx_write_compressed_tree(lzxd: plzx_data; tree: phuff_entry; prevlengths: pbyte;
			  treesize: longint): longint;
var
  codes,
  runs: pbyte;
  freqs: array [0..LZX_PRETREE_SIZE-1] of longint;
  cur_run: longint;
  last_len: longint;
  pretree: array [0..19] of huff_entry;
  codep,
  codee,
  runp: pbyte;
  excess,
  i,
  cur_code: longint;
begin
  codes := getmem(treesize*sizeof(byte));
  codep := codes;
  runs := getmem(treesize*sizeof(byte));
  runp := runs;
  Fillchar(freqs[0], sizeof(freqs), 0);
  cur_run := 1;
  last_len := tree[0].codelength;
  for i := 1 to treesize do begin
    if ((i = treesize) or (tree[i].codelength <> last_len)) then begin
      if (last_len = 0) then begin
	while (cur_run >= 20) do begin
	  excess := cur_run - 20;
	  if (excess > 31) then excess := 31;
	  codep^ := 18;
          Inc(codep);
	  runp^ := excess;
          Inc(runp);
	  Dec(cur_run, excess + 20);
	  Inc(freqs[18]);
        end;
	while (cur_run >= 4) do begin
	  excess := cur_run - 4;
	  if (excess > 15) then excess := 15;
	  codep^ := 17;
          Inc(codep);
	  runp^ := excess;
          Inc(runp);
	  Dec(cur_run, excess + 4);
	  Inc(freqs[17]);
        end;
	while (cur_run > 0) do begin
	  codep^ := prevlengths[i - cur_run];
	  Inc(freqs[codep^]);
          Inc(codep);
	  runp^ := 0; //* not necessary */
          Inc(runp);
	  Dec(cur_run);
        end;
      end
      else begin
	while (cur_run >= 4) do begin
	  if (cur_run = 4) then excess := 0
	  else excess := 1;
	  codep^ := 19;
          Inc(codep);
	  runp^ := excess;
          Inc(runp);
	  Inc(freqs[19]);
	  //* right, MS lies again.  Code is NOT
	  //   prev_len + len (mod 17), it's prev_len - len (mod 17)*/
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	  codep^ := byte(prevlengths[i-cur_run] - last_len);
	  if (codep^ > 16) then codep^ := byte(codep^ + 17); //Inc(codep^, 17);
=======
	  codep^ := prevlengths[i-cur_run] - last_len;
	  if (codep^ > 16) then Inc(codep^, 17);
>>>>>>> graemeg/fixes_2_2
=======
	  codep^ := prevlengths[i-cur_run] - last_len;
	  if (codep^ > 16) then Inc(codep^, 17);
>>>>>>> origin/fixes_2_2
=======
	  codep^ := byte(prevlengths[i-cur_run] - last_len);
	  if (codep^ > 16) then codep^ := byte(codep^ + 17); //Inc(codep^, 17);
>>>>>>> origin/fixes_2.4
	  Inc(freqs[codep^]);
          Inc(codep);
	  runp^ := 0; //* not necessary */
          Inc(runp);
	  Dec(cur_run, excess+4);
        end;
	while (cur_run > 0) do begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	  codep^ := byte(prevlengths[i-cur_run] - last_len);
	  if (codep^ > 16) then codep^ := byte(codep^ + 17); //Inc(codep^, byte(17));
=======
	  codep^ := prevlengths[i-cur_run] - last_len;
	  if (codep^ > 16) then Inc(codep^, 17);
>>>>>>> graemeg/fixes_2_2
=======
	  codep^ := prevlengths[i-cur_run] - last_len;
	  if (codep^ > 16) then Inc(codep^, 17);
>>>>>>> origin/fixes_2_2
=======
	  codep^ := byte(prevlengths[i-cur_run] - last_len);
	  if (codep^ > 16) then codep^ := byte(codep^ + 17); //Inc(codep^, byte(17));
>>>>>>> origin/fixes_2.4
	  runp^ := 0; //* not necessary */
          Inc(runp);
	  Dec(cur_run);
	  Inc(freqs[codep^]);
          Inc(codep);
        end;
      end;
      if (i <> treesize) then
	last_len := tree[i].codelength;
      cur_run := 0;
    end;
    Inc(cur_run);
  end;
  codee := codep;
  //* now create the huffman table and write out the pretree */
  build_huffman_tree(LZX_PRETREE_SIZE, 16, @freqs[0], pretree);
  for i := 0 to LZX_PRETREE_SIZE-1 do begin
    lzx_write_bits(lzxd, 4, pretree[i].codelength);
  end;
  codep := codes;
  runp := runs;
  cur_run := 0;
  while (codep < codee) do begin
    cur_code := codep^;
    Inc(codep);
    lzx_write_bits(lzxd, pretree[cur_code].codelength, pretree[cur_code].code);
    if (cur_code = 17) then begin
      Inc(cur_run, runp^ + 4);
      lzx_write_bits(lzxd, 4, runp^);
    end
    else if (cur_code = 18) then begin
      Inc(cur_run, runp^ + 20);
      lzx_write_bits(lzxd, 5, runp^);
    end
    else if (cur_code = 19) then begin
      Inc(cur_run, runp^ + 4);
      lzx_write_bits(lzxd, 1, runp^);
      cur_code := codep^;
      Inc(codep);
      lzx_write_bits(lzxd, pretree[cur_code].codelength, pretree[cur_code].code);
      Inc(runp);
    end
    else begin
      Inc(cur_run);
    end;
    Inc(runp);
  end;
  freemem(codes);
  freemem(runs);
  Exit(0);
end;

procedure lzx_reset(lzxd:plzx_data);
begin
  lzxd^.need_1bit_header := 1;
  lzxd^.R0 := 1;
  lzxd^.R1 := 1;
  lzxd^.R2 := 1;
  Fillchar(lzxd^.prev_main_treelengths[0], lzxd^.main_tree_size * sizeof(byte), 0);
  Fillchar(lzxd^.prev_length_treelengths[0], NUM_SECONDARY_LENGTHS * sizeof(byte), 0);
  lz_reset(lzxd^.lzi);
end;

function lzx_compress_block(lzxd:plzx_data; block_size:longint; subdivide:longbool):longint;
var
  i: longint;
  written_sofar: longword = 0;
  block_type: longint;
  uncomp_bits,
  comp_bits,
  comp_bits_ovh,
  uncomp_length: longword;
begin
  if ((lzxd^.block_size <> block_size) or (lzxd^.block_codes = nil)) then begin
    if (lzxd^.block_codes <> nil) then freemem(lzxd^.block_codes);
    lzxd^.block_size := block_size;
    lzxd^.block_codes :=  GetMem(block_size * sizeof(longword));
  end;

  lzxd^.subdivide := Ord(subdivide);

  lzxd^.left_in_block := block_size;
  lzxd^.left_in_frame := LZX_FRAME_SIZE;
  lzxd^.main_entropy := 0.0;
  lzxd^.last_ratio := 9999999.0;
  lzxd^.block_codesp := lzxd^.block_codes;


  Fillchar(lzxd^.length_freq_table[0], NUM_SECONDARY_LENGTHS * sizeof(longint), 0);
  Fillchar(lzxd^.main_freq_table[0], lzxd^.main_tree_size * sizeof(longint), 0);
  Fillchar(lzxd^.aligned_freq_table[0], LZX_ALIGNED_SIZE * sizeof(longint), 0);
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
  while ((lzxd^.left_in_block<>0) and ((lz_left_to_process(lzxd^.lzi)<>0) or not(lzxd^.at_eof(lzxd^.in_arg)))) do begin
    lz_compress(lzxd^.lzi, lzxd^.left_in_block);

    if (lzxd^.left_in_frame = 0) then begin
      lzxd^.left_in_frame := LZX_FRAME_SIZE;
    end;
    
<<<<<<< HEAD
<<<<<<< HEAD
=======
    if lzxd^.at_eof(lzxd^.in_arg) then Sleep(500);
>>>>>>> graemeg/fixes_2_2
=======
    if lzxd^.at_eof(lzxd^.in_arg) then Sleep(500);
>>>>>>> origin/fixes_2_2
    if ((lzxd^.subdivide<0)
      or (lzxd^.left_in_block = 0)
      or ((lz_left_to_process(lzxd^.lzi) = 0) and lzxd^.at_eof(lzxd^.in_arg))) then begin
      //* now one block is LZ-analyzed. */
      //* time to write it out */
      uncomp_length := lzxd^.block_size - lzxd^.left_in_block - written_sofar;
      //* uncomp_length will sometimes be 0 when input length is
      //  an exact multiple of frame size */
      if (uncomp_length = 0) then
        continue;
      if (lzxd^.subdivide < 0) then begin
	lzxd^.subdivide := 1;
      end;

      if (lzxd^.need_1bit_header <> 0) then begin
	//* one bit Intel preprocessing header */
	//* always 0 because this implementation doesn't do Intel preprocessing */
	lzx_write_bits(lzxd, 1, 0);
	lzxd^.need_1bit_header := 0;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
      //* handle extra bits */
      uncomp_bits := 0;
      comp_bits := 0;
      
      build_huffman_tree(LZX_ALIGNED_SIZE, 7, @lzxd^.aligned_freq_table[0], @lzxd^.aligned_tree[0]);
      for i := 0 to LZX_ALIGNED_SIZE-1 do begin
	Inc(uncomp_bits, lzxd^.aligned_freq_table[i]* 3);
	Inc(comp_bits, lzxd^.aligned_freq_table[i]* lzxd^.aligned_tree[i].codelength);
      end;
      comp_bits_ovh := comp_bits + LZX_ALIGNED_SIZE * 3;
      if (comp_bits_ovh < uncomp_bits) then
      	block_type := LZX_ALIGNED_OFFSET_BLOCK
      else
	block_type := LZX_VERBATIM_BLOCK;


      //* block type */
      lzx_write_bits(lzxd, 3, block_type);
      //* uncompressed length */
      lzx_write_bits(lzxd, 24, uncomp_length);

      written_sofar := lzxd^.block_size - lzxd^.left_in_block;

      //* now write out the aligned offset trees if present */
      if (block_type = LZX_ALIGNED_OFFSET_BLOCK) then begin
        for i := 0 to LZX_ALIGNED_SIZE-1 do begin
	  lzx_write_bits(lzxd, 3, lzxd^.aligned_tree[i].codelength);
        end;
      end;
      //* end extra bits */
      build_huffman_tree(lzxd^.main_tree_size, LZX_MAX_CODE_LENGTH,
			 lzxd^.main_freq_table, lzxd^.main_tree);
      build_huffman_tree(NUM_SECONDARY_LENGTHS, 16,
			 @lzxd^.length_freq_table[0], @lzxd^.length_tree[0]);

      //* now write the pre-tree and tree for main 1 */
      lzx_write_compressed_tree(lzxd, lzxd^.main_tree, lzxd^.prev_main_treelengths, NUM_CHARS);

      //* now write the pre-tree and tree for main 2*/
      lzx_write_compressed_tree(lzxd, lzxd^.main_tree + NUM_CHARS,
				lzxd^.prev_main_treelengths + NUM_CHARS,
				lzxd^.main_tree_size - NUM_CHARS);

      //* now write the pre tree and tree for length */
      lzx_write_compressed_tree(lzxd, @lzxd^.length_tree[0], @lzxd^.prev_length_treelengths[0],
				NUM_SECONDARY_LENGTHS);

      //* now write literals */
      lzx_write_compressed_literals(lzxd, block_type);

      //* copy treelengths somewhere safe to do delta compression */
      for i := 0 to lzxd^.main_tree_size-1 do begin
	lzxd^.prev_main_treelengths[i] := lzxd^.main_tree[i].codelength;
      end;
      for i := 0 to NUM_SECONDARY_LENGTHS-1 do begin
        lzxd^.prev_length_treelengths[i] := lzxd^.length_tree[i].codelength;
      end;
      lzxd^.main_entropy := 0.0;
      lzxd^.last_ratio := 9999999.0;
      lzxd^.block_codesp := lzxd^.block_codes;

      Fillchar(lzxd^.length_freq_table[0], NUM_SECONDARY_LENGTHS * sizeof(longint), 0);
      Fillchar(lzxd^.main_freq_table[0], lzxd^.main_tree_size * sizeof(longint), 0);
      Fillchar(lzxd^.aligned_freq_table[0], LZX_ALIGNED_SIZE * sizeof(longint), 0);
    end;
  end;
  Exit(0);
end;

function lzx_init(lzxdp:Pplzx_data; wsize_code:longint; get_bytes:TGetBytesFunc; get_bytes_arg:pointer; at_eof:TIsEndOfFileFunc;
             put_bytes:TWriteBytesFunc; put_bytes_arg:pointer; mark_frame:TMarkFrameFunc; mark_frame_arg:pointer):longint;var
  wsize: longint;
  lzxd: plzx_data;
begin
  if ((wsize_code < 15) or (wsize_code > 21)) then begin
    Exit(-1);
  end;
  
  //lzx_init_static(); I hardcoded this instead

  New(lzxd);
  FillChar(lzxd^, Sizeof(lzxd), 0);
  lzxdp^ := lzxd;

  if (lzxd = nil) then
    Exit(-2);

  lzxd^.in_arg := get_bytes_arg;
  lzxd^.out_arg := put_bytes_arg;
  lzxd^.mark_frame_arg := mark_frame_arg;
  lzxd^.get_bytes := get_bytes;
  lzxd^.put_bytes := put_bytes;
  lzxd^.at_eof := at_eof;
  lzxd^.mark_frame := mark_frame;

  wsize := 1 shl (wsize_code);

  lzxd^.bits_in_buf := 0;
  lzxd^.block_codes := nil;
  lzxd^.num_position_slots := num_position_slots[wsize_code-15];
  lzxd^.main_tree_size := (NUM_CHARS + 8 * lzxd^.num_position_slots);

  lzxd^.main_freq_table := GetMem(sizeof(longint) * lzxd^.main_tree_size);
  lzxd^.main_tree := GetMem(sizeof(huff_entry)* lzxd^.main_tree_size);
  lzxd^.prev_main_treelengths := GetMem(sizeof(byte)*lzxd^.main_tree_size);

  New(lzxd^.lzi);
  //* the -3 prevents matches at wsize, wsize-1, wsize-2, all of which are illegal */
  lz_init(lzxd^.lzi, wsize, wsize - 3, MAX_MATCH, MIN_MATCH, LZX_FRAME_SIZE,
	  @lzx_get_chars, @lzx_output_match, @lzx_output_literal,lzxd);
  lzxd^.len_uncompressed_input := 0;
  lzxd^.len_compressed_output := 0;
  lzx_reset(lzxd);
  Exit(0);
end;

function lzx_finish(lzxd:plzx_data; lzxr:plzx_results):longint;
begin
  if (lzxr <> nil) then begin
    lzxr^.len_compressed_output := lzxd^.len_compressed_output;
    lzxr^.len_uncompressed_input := lzxd^.len_uncompressed_input;
  end;
  lz_release(lzxd^.lzi);
  Dispose(lzxd^.lzi);
  freemem(lzxd^.prev_main_treelengths);
  freemem(lzxd^.main_tree);
  freemem(lzxd^.main_freq_table);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  freemem(lzxd^.block_codes);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  freemem(lzxd^.block_codes);
>>>>>>> origin/fixes_2.4
  dispose(lzxd);
  Exit(0);
end;

initialization
  rloge2 := 1.0 / ln(2);
end.
