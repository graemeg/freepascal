{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Routines to read/write ppu files

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit ppu;

{$i fpcdefs.inc}

interface

  uses
    constexp,entfile;

{ Also write the ppu if only crc if done, this can be used with ppudump to
  see the differences between the intf and implementation }
{ define INTFPPU}

{$ifdef Test_Double_checksum}
const
  CRC_array_Size = 200000;
type
  tcrc_array = array[0..crc_array_size] of dword;
  pcrc_array = ^tcrc_array;
{$endif Test_Double_checksum}

const
  { only update this version if something change in the tppuheader:
     * the unit flags listed below
     * the format of the header itself
    This number cannot become bigger than 255 (it's stored in a byte) }
  CurrentPPUVersion = 208;
  { for any other changes to the ppu format, increase this version number
    (it's a cardinal) }
  CurrentPPULongVersion = 12;

{ unit flags }
  uf_big_endian          = $000004;
  uf_in_library          = $000020; { is the file in another file than <ppufile>.* ? }
  uf_smart_linked        = $000040; { the ppu can be smartlinked }
  uf_static_linked       = $000080; { the ppu can be linked static }
  uf_shared_linked       = $000100; { the ppu can be linked shared }
  uf_lto_linked          = $000200; { the ppu can be used with LTO }
  uf_no_link             = $000400; { unit has no .o generated, but can still have external linking! }
  uf_little_endian       = $001000;
  uf_fpu_emulation       = $008000; { this unit was compiled with fpu emulation on }

type
  { bestreal is defined based on the target architecture }
  ppureal=bestreal;

  { set type aliases. We cast all sets to a corresponding array type so
    that if the set changes size, compilation will fail and we know that
    we have have to increase the ppu version }
  tppuset1 = array[0..0] of byte;
  tppuset2 = array[0..1] of byte;
  { tppuset3 = array[0..2] of byte; (sets of 3 bytes are rounded up to 4 bytes) }
  tppuset4 = array[0..3] of byte;
  tppuset5 = array[0..4] of byte;
  tppuset6 = array[0..5] of byte;
  tppuset7 = array[0..6] of byte;
  tppuset8 = array[0..7] of byte;
  tppuset9 = array[0..8] of byte;
  tppuset10 = array[0..9] of byte;
  tppuset11 = array[0..10] of byte;
  tppuset12 = array[0..11] of byte;
  tppuset13 = array[0..12] of byte;
  tppuset14 = array[0..13] of byte;
  tppuset15 = array[0..14] of byte;
  tppuset16 = array[0..15] of byte;
  tppuset17 = array[0..16] of byte;
  tppuset18 = array[0..17] of byte;
  tppuset19 = array[0..18] of byte;
  tppuset20 = array[0..19] of byte;
  tppuset21 = array[0..20] of byte;
  tppuset22 = array[0..21] of byte;
  tppuset23 = array[0..22] of byte;
  tppuset24 = array[0..23] of byte;
  tppuset25 = array[0..24] of byte;
  tppuset26 = array[0..25] of byte;
  tppuset27 = array[0..26] of byte;
  tppuset28 = array[0..27] of byte;
  tppuset29 = array[0..28] of byte;
  tppuset30 = array[0..29] of byte;
  tppuset31 = array[0..30] of byte;
  tppuset32 = array[0..31] of byte;

  tppuerror=(ppuentrytoobig,ppuentryerror);

  tppuheader=record
    common   : tentryheader;
    checksum : cardinal; { checksum for this ppufile }
    interface_checksum : cardinal;
    deflistsize,
    symlistsize : longint;
    indirect_checksum: cardinal;
  end;

  tppuentry=tentry;

  tunitasmlisttype=(ualt_public,ualt_extern);

  { tppufile }

  tppufile=class(tentryfile)
{$ifdef Test_Double_checksum}
  public
    interface_read_crc_index,
    interface_write_crc_index,
    indirect_read_crc_index,
    indirect_write_crc_index,
    implementation_read_crc_index,
    implementation_write_crc_index : cardinal;
    interface_crc_array,
    indirect_crc_array,
    implementation_crc_array  : pcrc_array;
    CRCFile : text;
  private
{$endif def Test_Double_checksum}
  protected
    procedure newheader;override;
    function readheader: longint;override;
    function outputallowed: boolean;override;
    //procedure doputdata(const b;len:integer);override;
    function getheadersize:longint;override;
    function getheaderaddr:pentryheader;override;
    procedure resetfile;override;
{$ifdef DEBUG_PPU}
    procedure ppu_log(st :string);override;
{$endif}
  public
    header           : tppuheader;
    { crc for the entire unit }
    crc,
    { crc for the interface definitions in this unit }
    interface_crc,
    { crc of all object/class definitions in the interface of this unit, xor'ed
      by the crc's of all object/class definitions in the interfaces of units
      used by this unit. Reason: see mantis #13840 }
    indirect_crc     : cardinal;
    do_crc,
    do_interface_crc,
    do_indirect_crc  : boolean;
    crc_only         : boolean;    { used to calculate interface_crc before implementation }
    constructor Create(const fn:string);
    destructor destroy;override;
    function  CheckPPUId:boolean;
  {read}
  { nothing special currently }
  {write}
    function  createfile:boolean;override;
    procedure writeheader;override;
    procedure putdata(const b;len:integer);override;
  end;

implementation

  uses
{$ifdef Test_Double_checksum}
    comphook,
{$endif def Test_Double_checksum}
    fpccrc;

{$ifdef Test_Double_checksum}
{$ifdef TEST_CRC_ERROR}
const
  CRC_Interface_Change_Message_Level=V_Error;
  CRC_Implementation_Change_Message_Level=V_Error;
  CRC_Indirect_Change_Message_Level=V_Error;
{$else : not  TEST_CRC_ERROR}
const
  CRC_Interface_Change_Message_Level=V_Warning;
  CRC_Implementation_Change_Message_Level=V_Note;
  CRC_Indirect_Change_Message_Level=V_Note;
{$endif : not TEST_CRC_ERROR}
{$endif Test_Double_checksum}

function swapendian_ppureal(d:ppureal):ppureal;

type ppureal_bytes=array[0..sizeof(d)-1] of byte;

var i:0..sizeof(d)-1;

begin
  for i:=low(ppureal_bytes) to high(ppureal_bytes) do
    ppureal_bytes(swapendian_ppureal)[i]:=ppureal_bytes(d)[high(ppureal_bytes)-i];
end;


{*****************************************************************************
                                  TPPUFile
*****************************************************************************}

constructor tppufile.Create(const fn:string);
begin
  inherited Create(fn);
  crc_only:=false;
{$ifdef Test_Double_checksum}
  if not assigned(interface_crc_array) then
    begin
      new(interface_crc_array);
      fillchar(interface_crc_array^,sizeof(interface_crc_array),#$ff);
    end;
  if not assigned(indirect_crc_array) then
    begin
      new(indirect_crc_array);
      fillchar(indirect_crc_array^,sizeof(indirect_crc_array),#$ff);
    end;
  if not assigned(implementation_crc_array) then
    begin
      new(implementation_crc_array);
      fillchar(implementation_crc_array^,sizeof(implementation_crc_array),#$ff);
    end;
{$endif Test_Double_checksum}
end;

destructor tppufile.destroy;
begin
{$ifdef Test_Double_checksum}
  if assigned(interface_crc_array) then
    dispose(interface_crc_array);
  interface_crc_array:=nil;
  if assigned(indirect_crc_array) then
    dispose(indirect_crc_array);
  indirect_crc_array:=nil;
  if assigned(implementation_crc_array) then
    dispose(implementation_crc_array);
  implementation_crc_array:=nil;
{$endif Test_Double_checksum}
  inherited destroy;
end;

function tppufile.CheckPPUId:boolean;
begin
  CheckPPUId:=((Header.common.Id[1]='P') and
                (Header.common.Id[2]='P') and
                (Header.common.Id[3]='U'));
end;


procedure tppufile.newheader;
var
  s : string;
begin
  fillchar(header,sizeof(tppuheader),0);
  str(currentppuversion,s);
  while length(s)<3 do
   s:='0'+s;
  with header.common do
   begin
     Id[1]:='P';
     Id[2]:='P';
     Id[3]:='U';
     Ver[1]:=s[1];
     Ver[2]:=s[2];
     Ver[3]:=s[3];
   end;
end;


function tppufile.readheader: longint;
begin
  if fsize<sizeof(tppuheader) then
    exit(0);
  result:=f.Read(header,sizeof(tppuheader));
  { The header is always stored in little endian order }
  { therefore swap if on a big endian machine          }
{$IFDEF ENDIAN_BIG}
  header.common.compiler := swapendian(header.common.compiler);
  header.common.cpu := swapendian(header.common.cpu);
  header.common.target := swapendian(header.common.target);
  header.common.flags := swapendian(header.common.flags);
  header.common.size := swapendian(header.common.size);
  header.checksum := swapendian(header.checksum);
  header.interface_checksum := swapendian(header.interface_checksum);
  header.indirect_checksum := swapendian(header.indirect_checksum);
  header.deflistsize:=swapendian(header.deflistsize);
  header.symlistsize:=swapendian(header.symlistsize);

  { the PPU DATA is stored in native order }
  change_endian := (header.common.flags and uf_little_endian) = uf_little_endian;
{$ELSE not ENDIAN_BIG}
  change_endian := (header.common.flags and uf_big_endian) = uf_big_endian;
{$ENDIF}
end;

function tppufile.outputallowed: boolean;
begin
  result:=not crc_only;
end;

{$ifdef DEBUG_PPU}
procedure tppufile.ppu_log(st :string);
begin
  inherited ppu_log(st);
  if flog_open then
    begin
      if do_crc and (ppu_log_idx < bufstart+bufidx) then
        begin
          writeln(flog,'New crc : ',hexstr(dword(crc),8));
          writeln(flog,'New interface crc : ',hexstr(dword(interface_crc),8));
          writeln(flog,'New indirect crc : ',hexstr(dword(indirect_crc),8));
          ppu_log_idx:=bufstart+bufidx;
         end;
    end;
{$ifdef IN_PPUDUMP}
  if update_crc then
    begin
      writeln('New crc : ',hexstr(dword(crc),8));
      writeln('New interface crc : ',hexstr(dword(interface_crc),8));
      writeln('New indirect crc : ',hexstr(dword(indirect_crc),8));
    end;
{$endif}
end;
{$endif}

{*****************************************************************************
                                TPPUFile Reading
*****************************************************************************}

{ nothing special currently }

{*****************************************************************************
                                TPPUFile Writing
*****************************************************************************}

function tppufile.createfile:boolean;
begin
{$ifdef INTFPPU}
  if crc_only then
   begin
     fname:=fname+'.intf';
     crc_only:=false;
   end;
{$endif}
  result:=inherited createfile;
end;


procedure tppufile.writeheader;
var
  opos : integer;
begin
  if crc_only then
   exit;
  { flush buffer }
  writebuf;
  { update size (w/o header!) in the header }
  header.common.size:=bufstart-sizeof(tppuheader);
  { set the endian flag }
{$ifndef FPC_BIG_ENDIAN}
    header.common.flags := header.common.flags or uf_little_endian;
{$else not FPC_BIG_ENDIAN}
    header.common.flags := header.common.flags or uf_big_endian;
    { Now swap the header.common in the correct endian (always little endian) }
    header.common.compiler := swapendian(header.common.compiler);
    header.common.cpu := swapendian(header.common.cpu);
    header.common.target := swapendian(header.common.target);
    header.common.flags := swapendian(header.common.flags);
    header.common.size := swapendian(header.common.size);
    header.checksum := swapendian(header.checksum);
    header.interface_checksum := swapendian(header.interface_checksum);
    header.indirect_checksum := swapendian(header.indirect_checksum);
    header.deflistsize:=swapendian(header.deflistsize);
    header.symlistsize:=swapendian(header.symlistsize);
{$endif not FPC_BIG_ENDIAN}
{ write header and restore filepos after it }
  opos:=f.Position;
  f.Position:=0;
  f.Write(header,sizeof(tppuheader));
  f.Position:=opos;
end;


procedure tppufile.putdata(const b;len:integer);
{$ifdef Test_Double_checksum}
  var 
    pb : pbyte;
    ind : integer;
{$endif Test_Double_checksum}
begin
  if do_crc then
   begin
     crc:=UpdateCrc32(crc,b,len);
{$ifdef Test_Double_checksum}
     if crc_only then
       begin
         implementation_crc_array^[implementation_write_crc_index]:=crc;
{$ifdef Test_Double_checksum_write}
         Write(CRCFile,'imp_crc ',implementation_write_crc_index:6,' $',hexstr(crc,8),' ',len);
	 pb:=@b;
	 for ind:=0 to len-1 do
           Write(CRCFile,' ',hexstr(pb[ind],2));
         Writeln(CRCFile);
{$endif Test_Double_checksum_write}
         if implementation_write_crc_index<crc_array_size then
          inc(implementation_write_crc_index);
       end
     else
       begin
         if (implementation_read_crc_index<crc_array_size) and
            (implementation_crc_array^[implementation_read_crc_index]<>crc) then
           begin
             do_comment(CRC_implementation_Change_Message_Level,'implementation CRC changed at index '+tostr(implementation_read_crc_index));
             if CRC_implementation_Change_Message_Level=V_Error then
               do_internalerror(2020113001);
{$ifdef Test_Double_checksum_write}
             Writeln(CRCFile,'!!!imp_crc ',implementation_read_crc_index:5,'$',hexstr(crc,8),'<>$',hexstr(implementation_crc_array^[implementation_read_crc_index],8));
           end
         else
           begin
             Writeln(CRCFile,'imp_crc ',implementation_read_crc_index:5,' OK');
{$endif Test_Double_checksum_write}
           end;
         inc(implementation_read_crc_index);
       end;
{$endif def Test_Double_checksum}
     if do_interface_crc then
       begin
         interface_crc:=UpdateCrc32(interface_crc,b,len);
{$ifdef Test_Double_checksum}
        if crc_only then
          begin
            interface_crc_array^[interface_write_crc_index]:=interface_crc;
{$ifdef Test_Double_checksum_write}
            Write(CRCFile,'int_crc ',interface_write_crc_index:5,' $',hexstr(interface_crc,8),' ',len);
	    pb:=@b;
	    for ind:=0 to len-1 do
              Write(CRCFile,' ',hexstr(pb[ind],2));
            Writeln(CRCFile);
{$endif Test_Double_checksum_write}
            if interface_write_crc_index<crc_array_size then
             inc(interface_write_crc_index);
          end
        else
          begin
            if (interface_read_crc_index<crc_array_size) and
               (interface_crc_array^[interface_read_crc_index]<>interface_crc) then
              begin
                do_comment(CRC_Interface_Change_Message_Level,'interface CRC changed at index '+tostr(interface_read_crc_index));
                if CRC_interface_Change_Message_Level=V_Error then
                  do_internalerror(2020113002);
{$ifdef Test_Double_checksum_write}
                Writeln(CRCFile,'!!!int_crc ',interface_read_crc_index:5,'$',hexstr(interface_crc,8),'<>$',hexstr(interface_crc_array^[interface_read_crc_index],8));
              end
            else
              begin
                Writeln(CRCFile,'int_crc ',interface_read_crc_index:5,' OK');
{$endif Test_Double_checksum_write}
              end;
            inc(interface_read_crc_index);
          end;
{$endif def Test_Double_checksum}
         { indirect crc must only be calculated for the interface; changes
           to a class in the implementation cannot require another unit to
           be recompiled }
         if do_indirect_crc then
           begin
             indirect_crc:=UpdateCrc32(indirect_crc,b,len);
{$ifdef Test_Double_checksum}
             if crc_only then
               begin
                 indirect_crc_array^[indirect_write_crc_index]:=indirect_crc;
{$ifdef Test_Double_checksum_write}
                 Write(CRCFile,'ind_crc ',indirect_write_crc_index:5,' $',hexstr(indirect_crc,8),' ',len);
                 pb:=@b;
                 for ind:=0 to len-1 do
                   Write(CRCFile,' ',hexstr(pb[ind],2));
                 Writeln(CRCFile);
{$endif Test_Double_checksum_write}
                 if indirect_write_crc_index<crc_array_size then
                   inc(indirect_write_crc_index);
               end
             else
               begin
                 if (indirect_read_crc_index<crc_array_size) and
                    (indirect_crc_array^[indirect_read_crc_index]<>indirect_crc) then
                   begin
                     do_comment(CRC_Indirect_Change_Message_Level,'Indirect CRC changed at index '+tostr(indirect_read_crc_index));
                     if CRC_indirect_Change_Message_Level=V_Error then
                       do_internalerror(2020113003);
{$ifdef Test_Double_checksum_write}
                     Writeln(CRCFile,'!!!ind_crc ',indirect_read_crc_index:5,'$',hexstr(indirect_crc,8),'<>$',hexstr(indirect_crc_array^[indirect_read_crc_index],8));
                   end
                 else
                   begin
                     Writeln(CRCFile,'ind_crc ',indirect_read_crc_index:5,' OK');
{$endif Test_Double_checksum_write}
                   end;
                 inc(indirect_read_crc_index);
               end;
{$endif def Test_Double_checksum}
           end;
       end;
    end;
  inherited putdata(b,len);
  (*if not crc_only then
    writedata(b,len);
  inc(entryidx,len);*)
end;

function tppufile.getheadersize: longint;
begin
  result:=sizeof(header);
end;

function tppufile.getheaderaddr: pentryheader;
begin
  result:=@header;
end;

procedure tppufile.resetfile;
begin
{$ifdef Test_Double_checksum_write}
  if (crc<>0) or (interface_crc<>0) or (indirect_crc<>0) then
    Writeln(CRCFile,'!!! tppufile.reset called',
                 ' implementation_crc=$',hexstr(crc,8),
                 ' interface_crc=$',hexstr(interface_crc,8),
                 ' indirect_crc=$',hexstr(indirect_crc,8),
                 ' implementation_crc_size=',implementation_write_crc_index,
                 ' interface_crc_size=',interface_write_crc_index,
                 ' indirect_crc_size=',indirect_write_crc_index);
{$endif Test_Double_checksum_write}
  crc:=0;
  interface_crc:=0;
  indirect_crc:=0;
  do_interface_crc:=true;
  do_indirect_crc:=false;
  do_crc:=true;
end;


end.
