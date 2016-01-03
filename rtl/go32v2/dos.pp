{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Dos unit for BP7 compatible RTL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$inline on}

unit dos;

interface

Uses
  Go32;

Type
  searchrec = packed record
     fill : array[1..21] of byte;
     attr : byte;
     time : longint;
     { reserved : word; not in DJGPP V2 }
     size : longint;
     name : string[255]; { LFN Name, DJGPP uses only [12] but more can't hurt (PFV) }
  end;

{$DEFINE HAS_REGISTERS}
  Registers = Go32.Registers;

{$i dosh.inc}

{$IfDef SYSTEM_DEBUG_STARTUP}
  {$DEFINE FORCE_PROXY}
{$endif SYSTEM_DEBUG_STARTUP}
Const
  { This variable can be set to true
    to force use of !proxy command lines even for short
    strings, for debugging purposes mainly, as
    this might have negative impact if trying to
    call non-go32v2 programs }
  force_go32v2_proxy : boolean =
{$ifdef FORCE_PROXY}
  true;
{$DEFINE DEBUG_PROXY}
{$else not FORCE_PROXY}
  false;
{$endif not FORCE_PROXY}
  { This variable allows to use !proxy if command line is
    longer than 126 characters.
    This will only work if the called program knows how to handle
    those command lines.
    Luckily this is the case for Free Pascal compiled
    programs (even old versions)
    and go32v2 DJGPP programs.
    You can set this to false to get a warning to stderr
    if command line is too long. }
  Use_go32v2_proxy : boolean = true;

{ Added to interface so that there is no need to implement it
  both in dos and sysutils units }

procedure exec_ansistring(path : string;comline : ansistring);

implementation

uses
  strings;

{$DEFINE HAS_GETMSCOUNT}
{$DEFINE HAS_INTR}
{$DEFINE HAS_SETCBREAK}
{$DEFINE HAS_GETCBREAK}
{$DEFINE HAS_SETVERIFY}
{$DEFINE HAS_GETVERIFY}
{$DEFINE HAS_SWAPVECTORS}
{$DEFINE HAS_GETSHORTNAME}
{$DEFINE HAS_GETLONGNAME}

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)

{$I dos.inc}

{******************************************************************************
                           --- Dos Interrupt ---
******************************************************************************}

var
  dosregs : registers;

procedure LoadDosError;
var
  r : registers;
  SimpleDosError : word;
begin
  if (dosregs.flags and fcarry) <> 0 then
   begin
     { I got a extended error = 0
       while CarryFlag was set from Exec function }
     SimpleDosError:=dosregs.ax;
     r.eax:=$5900;
     r.ebx:=$0;
     realintr($21,r);
     { conversion from word to integer !!
       gave a Bound check error if ax is $FFFF !! PM }
     doserror:=integer(r.ax);
     case doserror of
      0  : DosError:=integer(SimpleDosError);
      19 : DosError:=150;
      21 : DosError:=152;
     end;
   end
  else
    doserror:=0;
end;


procedure intr(intno : byte;var regs : registers);
begin
  realintr(intno,regs);
end;


{******************************************************************************
                        --- Info / Date / Time ---
******************************************************************************}

function dosversion : word;
begin
  dosregs.ax:=$3000;
  msdos(dosregs);
  dosversion:=dosregs.ax;
end;


procedure getdate(var year,month,mday,wday : word);
begin
  dosregs.ax:=$2a00;
  msdos(dosregs);
  wday:=dosregs.al;
  year:=dosregs.cx;
  month:=dosregs.dh;
  mday:=dosregs.dl;
end;


procedure setdate(year,month,day : word);
begin
   dosregs.cx:=year;
   dosregs.dh:=month;
   dosregs.dl:=day;
   dosregs.ah:=$2b;
   msdos(dosregs);
end;


procedure gettime(var hour,minute,second,sec100 : word);
begin
  dosregs.ah:=$2c;
  msdos(dosregs);
  hour:=dosregs.ch;
  minute:=dosregs.cl;
  second:=dosregs.dh;
  sec100:=dosregs.dl;
end;


procedure settime(hour,minute,second,sec100 : word);
begin
  dosregs.ch:=hour;
  dosregs.cl:=minute;
  dosregs.dh:=second;
  dosregs.dl:=sec100;
  dosregs.ah:=$2d;
  msdos(dosregs);
end;


function GetMsCount: int64;
begin
  GetMsCount := int64 (MemL [$40:$6c]) * 55;
end;


{******************************************************************************
                               --- Exec ---
******************************************************************************}

const
  DOS_MAX_COMMAND_LINE_LENGTH = 126;

<<<<<<< HEAD
<<<<<<< HEAD
procedure exec_ansistring(path : string;comline : ansistring);
=======
=======
>>>>>>> origin/fixes_2_2
procedure exec(const path : pathstr;const comline : comstr);
>>>>>>> graemeg/fixes_2_2
type
  realptr = packed record
    ofs,seg : word;
  end;
  texecblock = packed record
    envseg    : word;
    comtail   : realptr;
    firstFCB  : realptr;
    secondFCB : realptr;
{    iniStack  : realptr;
    iniCSIP   : realptr;}
  end;
var
  current_dos_buffer_pos,
  arg_ofs,
  i,la_env,
  la_p,la_c,la_e,
  fcb1_la,fcb2_la : longint;
  use_proxy       : boolean;
  proxy_argc      : longint;
  ExecBufSize, TB : longint;
  ExecBufPtr      : PChar;
  execblock       : texecblock;
  c               : ansistring;
  p               : string;

  function paste_to_dos(src : string;add_cr_at_end, include_string_length : boolean) : boolean;
  {Changed by Laaca - added parameter N}
  var
{
    c : pchar;
}
    CLen : cardinal;
    start_pos,ls : longint;
  begin
     paste_to_dos:=false;
<<<<<<< HEAD
<<<<<<< HEAD
     if include_string_length then
       start_pos:=0
     else
       start_pos:=1;
     ls:=Length(src)-start_pos;
{
     if current_dos_buffer_pos+ls+3>transfer_buffer+tb_size then
}
     if Current_Dos_Buffer_Pos + LS + 3 > ExecBufSize then
     begin
      FreeMem (ExecBufPtr);
      RunError(217);
     end;
{
=======
     ls:=Length(src)-n;
     if current_dos_buffer_pos+ls+3>transfer_buffer+tb_size then
      RunError(217);
>>>>>>> graemeg/fixes_2_2
=======
     ls:=Length(src)-n;
     if current_dos_buffer_pos+ls+3>transfer_buffer+tb_size then
      RunError(217);
>>>>>>> origin/fixes_2_2
     getmem(c,ls+3);
}
     Move (Src [Start_Pos], ExecBufPtr [Current_Dos_Buffer_Pos], LS + 1);
     Inc (Current_Dos_Buffer_Pos, LS + 1);
     if add_cr_at_end then
      begin
        ExecBufPtr [Current_Dos_Buffer_Pos] := #13;
        Inc (Current_Dos_Buffer_Pos);
      end;
      ExecBufPtr [Current_Dos_Buffer_Pos] := #0;
      Inc (Current_Dos_Buffer_Pos);
{
      CLen := StrLen (C) + 1;
     seg_move(get_ds,longint(c),dosmemselector,current_dos_buffer_pos,CLen);
     current_dos_buffer_pos:=current_dos_buffer_pos+CLen;
     freemem(c,ls+3);
}
     paste_to_dos:=true;
  end;

  procedure setup_proxy_cmdline;
  const
    MAX_ARGS = 128;
  var
    i : longint;
    quote : char;
    end_of_arg, skip_char : boolean;
    la_proxy_seg    : word;
    la_proxy_ofs    : longint;
    current_arg : string;
    la_argv_ofs : array [0..MAX_ARGS] of word;
  begin
    quote:=#0;
    current_arg:='';
    proxy_argc:=0;
    end_of_arg:=false;
    while (TB + current_dos_buffer_pos) mod 16 <> 0 do
      inc(current_dos_buffer_pos);
    la_proxy_seg:=(TB + current_dos_buffer_pos) shr 4;
    { Also copy parameter 0 }
    la_argv_ofs[0]:=TB+current_dos_buffer_pos-la_proxy_seg*16;
    { Note that this should be done before
      alteriing p value }
    paste_to_dos(p,false,false);
    inc(proxy_argc);
    for i:=1 to length(c) do
      begin
        skip_char:=false;
        case c[i] of
          #1..#32:
            begin
              if quote=#0 then
                end_of_arg:=true;
            end;
          '"' :
            begin
              if quote=#0 then
                begin
                  quote:='"';
                  skip_char:=true;
                end
              else if quote='"' then
                end_of_arg:=true;
            end;
          '''' :
            begin
              if quote=#0 then
                begin
                  quote:='''';
                  skip_char:=true;
                end
              else if quote='''' then
                end_of_arg:=true;
            end;
        end;
        if not end_of_arg and not skip_char then
          current_arg:=current_arg+c[i];
        if i=length(c) then
          end_of_arg:=true;
        if end_of_arg then
          begin
            { Allow empty args using "" or '' }
            if (current_arg<>'') or (quote<>#0) then
              begin
                if proxy_argc>MAX_ARGS then
                  begin
                    writeln(stderr,'Too many arguments in Dos.exec');
                    RunError(217);
                  end;
                la_argv_ofs[proxy_argc]:=TB + current_dos_buffer_pos - la_proxy_seg*16;
    {$ifdef DEBUG_PROXY}
                writeln(stderr,'arg ',proxy_argc,'="',current_arg,'"');
    {$endif DEBUG_PROXY}
                paste_to_dos(current_arg,false,false);
                inc(proxy_argc);
                quote:=#0;
                current_arg:='';
              end;
            { Always reset end_of_arg boolean }
            end_of_arg:=false;
          end;
      end;
    la_proxy_ofs:=TB + current_dos_buffer_pos - la_proxy_seg*16;
{
    seg_move(get_ds,longint(@la_argv_ofs),dosmemselector,
             current_dos_buffer_pos,proxy_argc*sizeof(word));
}
    Move (LA_ArgV_Ofs, ExecBufPtr [Current_Dos_Buffer_Pos],
                                                   Proxy_ArgC * SizeOf (word));
    current_dos_buffer_pos:=current_dos_buffer_pos + proxy_argc*sizeof(word);
    c:='!proxy '+hexstr(proxy_argc,4)+' '+hexstr(la_proxy_seg,4)
       +' '+hexstr(la_proxy_ofs,4);
{$ifdef DEBUG_PROXY}
    writeln(stderr,'Using comline "',c,'"');
{$endif DEBUG_PROXY}
  end;


begin
{ create command line }
<<<<<<< HEAD
  c:=comline; 
  use_proxy:=false;
  if force_go32v2_proxy then
    Use_proxy:=true
  else if length(c)>DOS_MAX_COMMAND_LINE_LENGTH then
    begin
      if Use_go32v2_proxy then
        begin
          Use_Proxy:=true;
        end
      else
        begin
           writeln(stderr,'Dos.exec command line truncated to ',
                   DOS_MAX_COMMAND_LINE_LENGTH,' chars');
           writeln(stderr,'Before: "',c,'"');
           setlength(c, DOS_MAX_COMMAND_LINE_LENGTH);
           writeln(stderr,'After: "',c,'"');
         end;
    end;
=======
  c:=comline;
  if length(c)>DOS_MAX_COMMAND_LINE_LENGTH then
    c[0]:=chr(DOS_MAX_COMMAND_LINE_LENGTH);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ create path }
{$ifdef DEBUG_PROXY}
  writeln(stderr,'Dos.exec path="',path,'"');
{$endif DEBUG_PROXY}
  p:=path;
<<<<<<< HEAD
=======
{ allow slash as backslash }
  DoDirSeparators(p);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  if LFNSupport then
    GetShortName(p);
{ create buffer }
  TB := Transfer_Buffer;
  ExecBufSize := TB_Size;
  GetMem (ExecBufPtr, ExecBufSize);
  if ExecBufPtr = nil then
   begin
    DosError := 8;
    Exit;
   end;
  la_env:=TB;
  while (la_env and 15)<>0 do
   inc(la_env);
  current_dos_buffer_pos:=la_env - TB;
{ copy environment }
  for i:=1 to envcount do
   paste_to_dos(envstr(i),false,false);
  {the behaviour is still suboptimal because variable COMMAND is stripped out}
<<<<<<< HEAD
<<<<<<< HEAD
  paste_to_dos(chr(0),false,false); { adds a double zero at the end }
  if use_proxy then
    setup_proxy_cmdline;
{ allow slash as backslash }
  DoDirSeparators(p);
  { Add program to DosBuffer with
    length at start }
  la_p:=TB + current_dos_buffer_pos;
  paste_to_dos(p,false,true);
  { Add command line args to DosBuffer with
    length at start and Carriage Return at end }
  la_c:=TB + current_dos_buffer_pos;
  paste_to_dos(c,true,true);

  la_e:=TB + current_dos_buffer_pos;
=======
=======
>>>>>>> origin/fixes_2_2
  paste_to_dos(chr(0),false,1); { adds a double zero at the end }
  la_p:=current_dos_buffer_pos;
  paste_to_dos(p,false,0);
  la_c:=current_dos_buffer_pos;
  paste_to_dos(c,true,0);
  la_e:=current_dos_buffer_pos;
>>>>>>> graemeg/fixes_2_2
  fcb1_la:=la_e;
  la_e:=la_e+16;
  fcb2_la:=la_e;
  la_e:=la_e+16;
{$ifdef DEBUG_PROXY}
  flush(stderr);
{$endif DEBUG_PROXY}
  seg_move (get_ds, PtrInt (ExecBufPtr), DosMemSelector, TB, Pred (Current_Dos_Buffer_Pos));
{ allocate FCB see dosexec code }
  arg_ofs:=1;
  while (arg_ofs<length(c)) and (c[arg_ofs] in [' ',#9]) do
    inc(arg_ofs);
  dosregs.ax:=$2901;
  dosregs.ds:=(la_c+arg_ofs) shr 4;
  dosregs.esi:=(la_c+arg_ofs) and 15;
  dosregs.es:=fcb1_la shr 4;
  dosregs.edi:=fcb1_la and 15;
  msdos(dosregs);
{ allocate second FCB see dosexec code }
  dosregs.ax:=$2901;
  dosregs.ds:=(la_c+arg_ofs) shr 4;
  dosregs.esi:=(la_c+arg_ofs) and 15;
  dosregs.es:=fcb2_la shr 4;
  dosregs.edi:=fcb2_la and 15;
  msdos(dosregs);
  with execblock do
   begin
     envseg:=la_env shr 4;
     comtail.seg:=la_c shr 4;
     comtail.ofs:=la_c and 15;
     firstFCB.seg:=fcb1_la shr 4;
     firstFCB.ofs:=fcb1_la and 15;
     secondFCB.seg:=fcb2_la shr 4;
     secondFCB.ofs:=fcb2_la and 15;
   end;
  seg_move(get_ds,longint(@execblock),dosmemselector,la_e,sizeof(texecblock));
  dosregs.edx:=la_p and 15+1;
  dosregs.ds:=la_p shr 4;
  dosregs.ebx:=la_p and 15+la_e-la_p;
  dosregs.es:=la_p shr 4;
  dosregs.ax:=$4b00;
  msdos(dosregs);
  LoadDosError;
  if DosError=0 then
   begin
     dosregs.ax:=$4d00;
     msdos(dosregs);
     LastDosExitCode:=DosRegs.al
   end
  else
   LastDosExitCode:=0;
end;

procedure exec(const path : pathstr;const comline : comstr);
begin
  exec_ansistring(path, comline);
end;


procedure getcbreak(var breakvalue : boolean);
begin
  dosregs.ax:=$3300;
  msdos(dosregs);
  breakvalue:=dosregs.dl<>0;
end;


procedure setcbreak(breakvalue : boolean);
begin
  dosregs.ax:=$3301;
  dosregs.dl:=ord(breakvalue);
  msdos(dosregs);
end;


procedure getverify(var verify : boolean);
begin
  dosregs.ah:=$54;
  msdos(dosregs);
  verify:=dosregs.al<>0;
end;


procedure setverify(verify : boolean);
begin
  dosregs.ah:=$2e;
  dosregs.al:=ord(verify);
  msdos(dosregs);
end;


{******************************************************************************
                               --- Disk ---
******************************************************************************}

type
  ExtendedFat32FreeSpaceRec = packed record
    RetSize           : word;      { $00 }
    Strucversion      : word;      { $02 }
    SecPerClus,                    { $04 }
    BytePerSec,                    { $08 }
    AvailClusters,                 { $0C }
    TotalClusters,                 { $10 }
    AvailPhysSect,                 { $14 }
    TotalPhysSect,                 { $18 }
    AvailAllocUnits,               { $1C }
    TotalAllocUnits   : longword;  { $20 }
    Dummy,                         { $24 }
    Dummy2            : longword;  { $28 }
  end;                             { $2C }

const
  IOCTL_INPUT = 3;       //For request header command field
  CDFUNC_SECTSIZE = 7;   //For cdrom control block func field
  CDFUNC_VOLSIZE  = 8;   //For cdrom control block func field
<<<<<<< HEAD
<<<<<<< HEAD

type
  TRequestHeader = packed record
    length     : byte;         { $00 }
    subunit    : byte;         { $01 }
    command    : byte;         { $02 }
    status     : word;         { $03 }
    reserved1  : longword;     { $05 }
    reserved2  : longword;     { $09 }
    media_desc : byte;         { $0D }
    transf_ofs : word;         { $0E }
    transf_seg : word;         { $10 }
    numbytes   : word;         { $12 }
  end;                         { $14 }

  TCDSectSizeReq = packed record
    func    : byte;            { $00 }
    mode    : byte;            { $01 }
    secsize : word;            { $02 }
  end;                         { $04 }

  TCDVolSizeReq = packed record
    func    : byte;            { $00 }
    size    : longword;        { $01 }
  end;                         { $05 }


function do_diskdata(drive : byte; Free : boolean) : Int64;
var
  blocksize, freeblocks, totblocks : longword;

=======
=======
>>>>>>> origin/fixes_2_2

type
  TRequestHeader = packed record
    length     : byte;         { $00 }
    subunit    : byte;         { $01 }
    command    : byte;         { $02 }
    status     : word;         { $03 }
    reserved1  : longword;     { $05 }
    reserved2  : longword;     { $09 }
    media_desc : byte;         { $0D }
    transf_ofs : word;         { $0E }
    transf_seg : word;         { $10 }
    numbytes   : word;         { $12 }
  end;                         { $14 }

  TCDSectSizeReq = packed record
    func    : byte;            { $00 }
    mode    : byte;            { $01 }
    secsize : word;            { $02 }
  end;                         { $04 }

  TCDVolSizeReq = packed record
    func    : byte;            { $00 }
    size    : longword;        { $01 }
  end;                         { $05 }


function do_diskdata(drive : byte; Free : boolean) : Int64;
var
  blocksize, freeblocks, totblocks : longword;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { Get disk data via old int21/36 (GET FREE DISK SPACE). It's always supported
    even if it returns wrong values for volumes > 2GB and for cdrom drives when
    in pure DOS. Note that it's also the only way to get some data on WinNTs. }
  function DiskData_36 : boolean;
  begin
    DiskData_36:=false;
    dosregs.dl:=drive;
    dosregs.ah:=$36;
    msdos(dosregs);
    if dosregs.ax=$FFFF then exit;

    blocksize:=dosregs.ax*dosregs.cx;
    freeblocks:=dosregs.bx;
    totblocks:=dosregs.dx;
    Diskdata_36:=true;
  end;

  { Get disk data via int21/7303 (FAT32 - GET EXTENDED FREE SPACE ON DRIVE).
    It is supported by win9x even in pure DOS }
  function DiskData_7303 : boolean;
  var
    s : shortstring;
    rec : ExtendedFat32FreeSpaceRec;
  begin
    DiskData_7303:=false;
    s:=chr(drive+$40)+':\'+#0;

    rec.Strucversion:=0;
    rec.RetSize := 0;
    dosmemput(tb_segment,tb_offset,Rec,sizeof(ExtendedFat32FreeSpaceRec));
    dosmemput(tb_segment,tb_offset+Sizeof(ExtendedFat32FreeSpaceRec)+1,s[1],4);
    dosregs.dx:=tb_offset+Sizeof(ExtendedFat32FreeSpaceRec)+1;
    dosregs.ds:=tb_segment;
    dosregs.di:=tb_offset;
    dosregs.es:=tb_segment;
    dosregs.cx:=Sizeof(ExtendedFat32FreeSpaceRec);
    dosregs.ax:=$7303;
    msdos(dosregs);
    if (dosregs.flags and fcarry) <> 0 then
      exit;
    copyfromdos(rec,Sizeof(ExtendedFat32FreeSpaceRec));
    if Rec.RetSize = 0 then
      exit;

    blocksize:=rec.SecPerClus*rec.BytePerSec;
    freeblocks:=rec.AvailAllocUnits;
    totblocks:=rec.TotalAllocUnits;
    DiskData_7303:=true;
  end;

  { Get disk data asking to MSCDEX. Pure DOS returns wrong values with
    int21/7303 or int21/36 if the drive is a CDROM drive }
  function DiskData_CDROM : boolean;
  var req : TRequestHeader;
      sectreq : TCDSectSizeReq;
      sizereq : TCDVolSizeReq;
      i : integer;
      status,byteswritten : word;
      drnum : byte;
  begin
    DiskData_CDROM:=false;
    drnum:=drive-1; //for MSCDEX, 0 = a, 1 = b etc, unlike int21/36

    { Is this a CDROM drive? }
    dosregs.ax:=$150b;
    dosregs.cx:=drnum;
    realintr($2f,dosregs);
    if (dosregs.bx<>$ADAD) or (dosregs.ax=0) then
      exit; // no, it isn't

    { Prepare the request header to send to the cdrom driver }
    FillByte(req,sizeof(req),0);
    req.length:=sizeof(req);
    req.command:=IOCTL_INPUT;
    req.transf_ofs:=tb_offset+sizeof(req); //CDROM control block will follow
    req.transf_seg:=tb_segment;            //the request header
    req.numbytes:=sizeof(sectreq);

    { We're asking the sector size }
    sectreq.func:=CDFUNC_SECTSIZE;
    sectreq.mode:=0; //cooked
    sectreq.secsize:=0;

    for i:=1 to 2 do
    begin
      { Send the request to the cdrom driver }
      dosmemput(tb_segment,tb_offset,req,sizeof(req));
      dosmemput(tb_segment,tb_offset+sizeof(req),sectreq,sizeof(sectreq));
      dosregs.ax:=$1510;
      dosregs.cx:=drnum;
      dosregs.es:=tb_segment;
      dosregs.bx:=tb_offset;
      realintr($2f,dosregs);
      dosmemget(tb_segment,tb_offset+3,status,2);
      { status = $800F means "disk changed". Try once more. }
      if (status and $800F) <> $800F then break;
    end;
    dosmemget(tb_segment,tb_offset+$12,byteswritten,2);
    if (status<>$0100) or (byteswritten<>sizeof(sectreq)) then
      exit; //An error occurred
    dosmemget(tb_segment,tb_offset+sizeof(req),sectreq,sizeof(sectreq));

  { Update the request header for the next request }
    req.numbytes:=sizeof(sizereq);

    { We're asking the volume size (in blocks) }
    sizereq.func:=CDFUNC_VOLSIZE;
    sizereq.size:=0;

    { Send the request to the cdrom driver }
    dosmemput(tb_segment,tb_offset,req,sizeof(req));
    dosmemput(tb_segment,tb_offset+sizeof(req),sizereq,sizeof(sizereq));
    dosregs.ax:=$1510;
    dosregs.cx:=drnum;
    dosregs.es:=tb_segment;
    dosregs.bx:=tb_offset;
    realintr($2f,dosregs);
    dosmemget(tb_segment,tb_offset,req,sizeof(req));
    if (req.status<>$0100) or (req.numbytes<>sizeof(sizereq)) then
      exit; //An error occurred
    dosmemget(tb_segment,tb_offset+sizeof(req)+1,sizereq.size,4);

    blocksize:=sectreq.secsize;
    freeblocks:=0; //always 0 for a cdrom
    totblocks:=sizereq.size;
    DiskData_CDROM:=true;
  end;

begin
  if drive=0 then
  begin
    dosregs.ax:=$1900;    //get current default drive
    msdos(dosregs);
    drive:=dosregs.al+1;
  end;

  if not DiskData_CDROM then
  if not DiskData_7303 then
  if not DiskData_36 then
  begin
    do_diskdata:=-1;
    exit;
  end;
  do_diskdata:=blocksize;
  if free then
    do_diskdata:=do_diskdata*freeblocks
  else
    do_diskdata:=do_diskdata*totblocks;
end;

function diskfree(drive : byte) : int64;
begin
   diskfree:=Do_DiskData(drive,TRUE);
end;

function disksize(drive : byte) : int64;
begin
  disksize:=Do_DiskData(drive,false);
end;


{******************************************************************************
                      --- LFNFindfirst LFNFindNext ---
******************************************************************************}

type
  LFNSearchRec=packed record
    attr,
    crtime,
    crtimehi,
    actime,
    actimehi,
    lmtime,
    lmtimehi,
    sizehi,
    size      : longint;
    reserved  : array[0..7] of byte;
    name      : array[0..259] of byte;
    shortname : array[0..13] of byte;
  end;

procedure LFNSearchRec2Dos(const w:LFNSearchRec;hdl:longint;var d:Searchrec;from_findfirst : boolean);
var
  Len : longint;
begin
  With w do
   begin
     FillChar(d,sizeof(SearchRec),0);
     if DosError=0 then
      len:=StrLen(@Name)
     else
      len:=0;
     d.Name[0]:=chr(len);
     Move(Name[0],d.Name[1],Len);
     d.Time:=lmTime;
     d.Size:=Size;
     d.Attr:=Attr and $FF;
     if (DosError<>0) and from_findfirst then
       hdl:=-1;
     Move(hdl,d.Fill,4);
   end;
end;

{$ifdef DEBUG_LFN}
const
  LFNFileName : string = 'LFN.log';
  LFNOpenNb : longint = 0;
  LogLFN : boolean = false;
var
  lfnfile : text;
{$endif DEBUG_LFN}

procedure LFNFindFirst(path:pchar;attr:longint;var s:searchrec);
var
  i : longint;
  w : LFNSearchRec;
begin
  { allow slash as backslash }
  DoDirSeparators(path);
  dosregs.si:=1; { use ms-dos time }
  { don't include the label if not asked for it, needed for network drives }
  if attr=$8 then
   dosregs.ecx:=8
  else
   dosregs.ecx:=attr and (not 8);
  dosregs.edx:=tb_offset+Sizeof(LFNSearchrec)+1;
  dosmemput(tb_segment,tb_offset+Sizeof(LFNSearchrec)+1,path^,strlen(path)+1);
  dosregs.ds:=tb_segment;
  dosregs.edi:=tb_offset;
  dosregs.es:=tb_segment;
  dosregs.ax:=$714e;
  msdos(dosregs);
  LoadDosError;
  if DosError=2 then
    DosError:=18;
{$ifdef DEBUG_LFN}
  if (DosError=0) and LogLFN then
    begin
      Append(lfnfile);
      inc(LFNOpenNb);
      Writeln(lfnfile,LFNOpenNb,' LFNFindFirst called ',path);
      close(lfnfile);
    end;
{$endif DEBUG_LFN}
  copyfromdos(w,sizeof(LFNSearchRec));
  LFNSearchRec2Dos(w,dosregs.ax,s,true);
end;


procedure LFNFindNext(var s:searchrec);
var
  hdl : longint;
  w   : LFNSearchRec;
begin
  Move(s.Fill,hdl,4);
  dosregs.si:=1; { use ms-dos time }
  dosregs.edi:=tb_offset;
  dosregs.es:=tb_segment;
  dosregs.ebx:=hdl;
  dosregs.ax:=$714f;
  msdos(dosregs);
  LoadDosError;
  copyfromdos(w,sizeof(LFNSearchRec));
  LFNSearchRec2Dos(w,hdl,s,false);
end;


procedure LFNFindClose(var s:searchrec);
var
  hdl : longint;
begin
  Move(s.Fill,hdl,4);
  { Do not call MsDos if FindFirst returned with an error }
  if hdl=-1 then
    begin
      DosError:=0;
      exit;
    end;
  dosregs.ebx:=hdl;
  dosregs.ax:=$71a1;
  msdos(dosregs);
  LoadDosError;
{$ifdef DEBUG_LFN}
  if (DosError=0) and LogLFN  then
    begin
      Append(lfnfile);
      Writeln(lfnfile,LFNOpenNb,' LFNFindClose called ');
      close(lfnfile);
      if LFNOpenNb>0 then
        dec(LFNOpenNb);
    end;
{$endif DEBUG_LFN}
end;


{******************************************************************************
                     --- DosFindfirst DosFindNext ---
******************************************************************************}

procedure dossearchrec2searchrec(var f : searchrec);
var
  len : longint;
begin
  { Check is necessary!! OS/2's VDM doesn't clear the name with #0 if the }
  { file doesn't exist! (JM)                                              }
  if dosError = 0 then
    len:=StrLen(@f.Name)
  else len := 0;
  Move(f.Name[0],f.Name[1],Len);
  f.Name[0]:=chr(len);
end;


procedure DosFindfirst(path : pchar;attr : word;var f : searchrec);
var
   i : longint;
begin
  { allow slash as backslash }
  DoDirSeparators(path);
  copytodos(f,sizeof(searchrec));
  dosregs.edx:=tb_offset;
  dosregs.ds:=tb_segment;
  dosregs.ah:=$1a;
  msdos(dosregs);
  dosregs.ecx:=attr;
  dosregs.edx:=tb_offset+Sizeof(searchrec)+1;
  dosmemput(tb_segment,tb_offset+Sizeof(searchrec)+1,path^,strlen(path)+1);
  dosregs.ds:=tb_segment;
  dosregs.ah:=$4e;
  msdos(dosregs);
  copyfromdos(f,sizeof(searchrec));
  LoadDosError;
  dossearchrec2searchrec(f);
end;


procedure Dosfindnext(var f : searchrec);
begin
  copytodos(f,sizeof(searchrec));
  dosregs.edx:=tb_offset;
  dosregs.ds:=tb_segment;
  dosregs.ah:=$1a;
  msdos(dosregs);
  dosregs.ah:=$4f;
  msdos(dosregs);
  copyfromdos(f,sizeof(searchrec));
  LoadDosError;
  dossearchrec2searchrec(f);
end;


{******************************************************************************
                     --- Findfirst FindNext ---
******************************************************************************}

procedure findfirst(const path : pathstr;attr : word;var f : searchRec);
var
  path0 : array[0..255] of char;
begin
  doserror:=0;
  strpcopy(path0,path);
  if LFNSupport then
   LFNFindFirst(path0,attr,f)
  else
   Dosfindfirst(path0,attr,f);
end;


procedure findnext(var f : searchRec);
begin
  doserror:=0;
  if LFNSupport then
   LFNFindnext(f)
  else
   Dosfindnext(f);
end;


Procedure FindClose(Var f: SearchRec);
begin
  DosError:=0;
  if LFNSupport then
   LFNFindClose(f);
end;


type swap_proc = procedure;

var
  _swap_in  : swap_proc;external name '_swap_in';
  _swap_out : swap_proc;external name '_swap_out';
  _exception_exit : pointer;external name '_exception_exit';
  _v2prt0_exceptions_on : longbool;external name '_v2prt0_exceptions_on';

procedure swapvectors;
begin
  if _exception_exit<>nil then
    if _v2prt0_exceptions_on then
      _swap_out()
    else
      _swap_in();
end;


{******************************************************************************
                               --- File ---
******************************************************************************}


Function FSearch(path: pathstr; dirlist: string): pathstr;
var
  i,p1   : longint;
  s      : searchrec;
  newdir : pathstr;
begin
{ check if the file specified exists }
  findfirst(path,anyfile and not(directory),s);
  if doserror=0 then
   begin
     findclose(s);
     fsearch:=path;
     exit;
   end;
{ No wildcards allowed in these things }
  if (pos('?',path)<>0) or (pos('*',path)<>0) then
    fsearch:=''
  else
    begin
       { allow slash as backslash }
       DoDirSeparators(dirlist);
       repeat
         p1:=pos(';',dirlist);
         if p1<>0 then
          begin
            newdir:=copy(dirlist,1,p1-1);
            delete(dirlist,1,p1);
          end
         else
          begin
            newdir:=dirlist;
            dirlist:='';
          end;
         if (newdir<>'') and (not (newdir[length(newdir)] in ['\',':'])) then
          newdir:=newdir+'\';
         findfirst(newdir+path,anyfile and not(directory),s);
         if doserror=0 then
          newdir:=newdir+path
         else
          newdir:='';
       until (dirlist='') or (newdir<>'');
       fsearch:=newdir;
    end;
  findclose(s);
end;


{ change to short filename if successful DOS call PM }
function GetShortName(var p : String) : boolean;
var
  c : array[0..255] of char;
begin
  move(p[1],c[0],length(p));
  c[length(p)]:=#0;
  copytodos(c,length(p)+1);
  dosregs.ax:=$7160;
  dosregs.cx:=1;
  dosregs.ds:=tb_segment;
  dosregs.si:=tb_offset;
  dosregs.es:=tb_segment;
  dosregs.di:=tb_offset;
  msdos(dosregs);
  LoadDosError;
  if DosError=0 then
   begin
     copyfromdos(c,256);
     move(c[0],p[1],strlen(c));
     p[0]:=char(strlen(c));
     GetShortName:=true;
   end
  else
   GetShortName:=false;
end;


{ change to long filename if successful DOS call PM }
function GetLongName(var p : String) : boolean;
var
  c : array[0..255] of char;
begin
  move(p[1],c[0],length(p));
  c[length(p)]:=#0;
  copytodos(c,length(p)+1);
  dosregs.ax:=$7160;
  dosregs.cx:=2;
  dosregs.ds:=tb_segment;
  dosregs.si:=tb_offset;
  dosregs.es:=tb_segment;
  dosregs.di:=tb_offset;
  msdos(dosregs);
  LoadDosError;
  if DosError=0 then
   begin
     copyfromdos(c,256);
     move(c[0],p[1],strlen(c));
     p[0]:=char(strlen(c));
     GetLongName:=true;
   end
  else
   GetLongName:=false;
end;


{******************************************************************************
                       --- Get/Set File Time,Attr ---
******************************************************************************}

procedure getftime(var f;var time : longint);
begin
  dosregs.bx:=textrec(f).handle;
  dosregs.ax:=$5700;
  msdos(dosregs);
  loaddoserror;
  time:=(dosregs.dx shl 16)+dosregs.cx;
end;


procedure setftime(var f;time : longint);
begin
  dosregs.bx:=textrec(f).handle;
  dosregs.cx:=time and $ffff;
  dosregs.dx:=time shr 16;
  dosregs.ax:=$5701;
  msdos(dosregs);
  loaddoserror;
end;


procedure getfattr(var f;var attr : word);
{$ifndef FPC_ANSI_TEXTFILEREC}
var
  r: rawbytestring;
{$endif not FPC_ANSI_TEXTFILEREC}
begin
{$ifdef FPC_ANSI_TEXTFILEREC}
  copytodos(filerec(f).name,strlen(filerec(f).name)+1);
{$else}
  r:=ToSingleByteFileSystemEncodedFileName(filerec(f).name);
  copytodos(pchar(r)^,length(r)+1);
{$endif}
  dosregs.edx:=tb_offset;
  dosregs.ds:=tb_segment;
  if LFNSupport then
   begin
     dosregs.ax:=$7143;
     dosregs.bx:=0;
   end
  else
   dosregs.ax:=$4300;
  msdos(dosregs);
  LoadDosError;
  Attr:=dosregs.cx;
end;


procedure setfattr(var f;attr : word);
{$ifndef FPC_ANSI_TEXTFILEREC}
var
  r: rawbytestring;
{$endif not FPC_ANSI_TEXTFILEREC}
begin
  { Fail for setting VolumeId. }
  if ((attr and VolumeID)<>0) then
  begin
    doserror:=5;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_ANSI_TEXTFILEREC}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  copytodos(filerec(f).name,strlen(filerec(f).name)+1);
{$else}
  r:=ToSingleByteFileSystemEncodedFileName(filerec(f).name);
  copytodos(pchar(r)^,length(r)+1);
{$endif}
  dosregs.edx:=tb_offset;
  dosregs.ds:=tb_segment;
  if LFNSupport then
   begin
     dosregs.ax:=$7143;
     dosregs.bx:=1;
   end
  else
   dosregs.ax:=$4301;
  dosregs.cx:=attr;
  msdos(dosregs);
  LoadDosError;
end;


{******************************************************************************
                             --- Environment ---
******************************************************************************}

function envcount : longint;
var
  hp : ppchar;
begin
  hp:=envp;
  envcount:=0;
  while assigned(hp^) do
   begin
     inc(envcount);
     inc(hp);
   end;
end;


function envstr (Index: longint): string;
begin
  if (index<=0) or (index>envcount) then
    envstr:=''
  else
    envstr:=strpas(ppchar(pointer(envp)+SizeOf(PChar)*(index-1))^);
end;


Function  GetEnv(envvar: string): string;
var
  hp    : ppchar;
  hs    : string;
  eqpos : longint;
begin
  envvar:=upcase(envvar);
  hp:=envp;
  getenv:='';
  while assigned(hp^) do
   begin
     hs:=strpas(hp^);
     eqpos:=pos('=',hs);
     if upcase(copy(hs,1,eqpos-1))=envvar then
      begin
        getenv:=copy(hs,eqpos+1,length(hs)-eqpos);
        break;
      end;
     inc(hp);
   end;
end;

{$ifdef DEBUG_LFN}
begin
  LogLFN:=(GetEnv('LOGLFN')<>'');
  assign(lfnfile,LFNFileName);
{$I-}
  Reset(lfnfile);
  if IOResult<>0 then
    begin
      Rewrite(lfnfile);
      Writeln(lfnfile,'New lfn.log');
    end;
  close(lfnfile);
{$endif DEBUG_LFN}

end.
