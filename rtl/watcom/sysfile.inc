<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   { Keep Track of open files }
   const
      max_files = 50;
   var
      openfiles : array [0..max_files-1] of boolean;
{$ifdef SYSTEMDEBUG}
      opennames : array [0..max_files-1] of pchar;
   const
      free_closed_names : boolean = true;
{$endif SYSTEMDEBUG}

{****************************************************************************
                        Low level File Routines
 ****************************************************************************}

procedure do_close(handle : longint);
var
  regs : trealregs;
begin
  if Handle<=4 then
   exit;
  regs.realebx:=handle;
  if handle<max_files then
    begin
       openfiles[handle]:=false;
{$ifdef SYSTEMDEBUG}
       if assigned(opennames[handle]) and free_closed_names then
         begin
            sysfreememsize(opennames[handle],strlen(opennames[handle])+1);
            opennames[handle]:=nil;
         end;
{$endif SYSTEMDEBUG}
    end;
  regs.realeax:=$3e00;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_erase(p : pchar; pchangeable: boolean);
var
  regs : trealregs;
  oldp : pchar;
begin
  oldp:=p;
  DoDirSeparators(p,pchangeable);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure do_erase(p : pchar);
var
  regs : trealregs;
begin
  DoDirSeparators(p);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  syscopytodos(longint(p),strlen(p)+1);
  regs.realedx:=tb_offset;
  regs.realds:=tb_segment;
  if LFNSupport then
   regs.realeax:=$7141
  else
   regs.realeax:=$4100;
  regs.realesi:=0;
  regs.realecx:=0;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if p<>oldp then
    freemem(p);
end;

procedure do_rename(p1,p2 : pchar; p1changeable, p2changeable: boolean);
var
  regs : trealregs;
  oldp1, oldp2 : pchar;
begin
  oldp1:=p1;
  oldp2:=p2;
  DoDirSeparators(p1,p1changeable);
  DoDirSeparators(p2,p2changeable);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
end;

procedure do_rename(p1,p2 : pchar);
var
  regs : trealregs;
begin
  DoDirSeparators(p1);
  DoDirSeparators(p2);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  if strlen(p1)+strlen(p2)+3>tb_size then
   HandleError(217);
  sysseg_move(get_ds,sizeuint(p2),dos_selector,tb,strlen(p2)+1);
  sysseg_move(get_ds,sizeuint(p1),dos_selector,tb+strlen(p2)+2,strlen(p1)+1);
  regs.realedi:=tb_offset;
  regs.realedx:=tb_offset + strlen(p2)+2;
  regs.realds:=tb_segment;
  regs.reales:=tb_segment;
  if LFNSupport then
   regs.realeax:=$7156
  else
   regs.realeax:=$5600;
  regs.realecx:=$ff;            { attribute problem here ! }
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if p1<>oldp1 then
    freemem(p1);
  if p2<>oldp2 then
    freemem(p2);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
end;

function do_write(h:longint;addr:pointer;len : longint) : longint;
var
  regs      : trealregs;
  size,
  writesize : longint;
begin
  writesize:=0;
  while len > 0 do
   begin
     if len>tb_size then
      size:=tb_size
     else
      size:=len;
     syscopytodos(ptrint(addr)+writesize,size);
     regs.realecx:=size;
     regs.realedx:=tb_offset;
     regs.realds:=tb_segment;
     regs.realebx:=h;
     regs.realeax:=$4000;
     sysrealintr($21,regs);
     if (regs.realflags and carryflag) <> 0 then
      begin
        GetInOutRes(lo(regs.realeax));
        exit(writesize);
      end;
     inc(writesize,lo(regs.realeax));
     dec(len,lo(regs.realeax));
     { stop when not the specified size is written }
     if lo(regs.realeax)<size then
      break;
   end;
  Do_Write:=WriteSize;
end;

function do_read(h:longint;addr:pointer;len : longint) : longint;
var
  regs     : trealregs;
  size,
  readsize : longint;
begin
  readsize:=0;
  while len > 0 do
   begin
     if len>tb_size then
      size:=tb_size
     else
      size:=len;
     regs.realecx:=size;
     regs.realedx:=tb_offset;
     regs.realds:=tb_segment;
     regs.realebx:=h;
     regs.realeax:=$3f00;
     sysrealintr($21,regs);
     if (regs.realflags and carryflag) <> 0 then
      begin
        GetInOutRes(lo(regs.realeax));
        do_read:=0;
        exit;
      end;
     syscopyfromdos(ptrint(addr)+readsize,lo(regs.realeax));
     inc(readsize,lo(regs.realeax));
     dec(len,lo(regs.realeax));
     { stop when not the specified size is read }
     if lo(regs.realeax)<size then
      break;
   end;
  do_read:=readsize;
end;


function do_filepos(handle : longint) : longint;
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realecx:=0;
  regs.realedx:=0;
  regs.realeax:=$4201;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   Begin
     GetInOutRes(lo(regs.realeax));
     do_filepos:=0;
   end
  else
   do_filepos:=lo(regs.realedx) shl 16+lo(regs.realeax);
end;


procedure do_seek(handle,pos : longint);
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realecx:=pos shr 16;
  regs.realedx:=pos and $ffff;
  regs.realeax:=$4200;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;



function do_seekend(handle:longint):longint;
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realecx:=0;
  regs.realedx:=0;
  regs.realeax:=$4202;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   Begin
     GetInOutRes(lo(regs.realeax));
     do_seekend:=0;
   end
  else
   do_seekend:=lo(regs.realedx) shl 16+lo(regs.realeax);
end;


function do_filesize(handle : longint) : longint;
var
  aktfilepos : longint;
begin
  aktfilepos:=do_filepos(handle);
  do_filesize:=do_seekend(handle);
  do_seek(handle,aktfilepos);
end;


{ truncate at a given position }
procedure do_truncate (handle,pos:longint);
var
  regs : trealregs;
begin
  do_seek(handle,pos);
  regs.realecx:=0;
  regs.realedx:=tb_offset;
  regs.realds:=tb_segment;
  regs.realebx:=handle;
  regs.realeax:=$4000;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

const
  FileHandleCount : longint = 20;

function Increase_file_handle_count : boolean;
var
  regs : trealregs;
begin
  Inc(FileHandleCount,10);
  regs.realebx:=FileHandleCount;
  regs.realeax:=$6700;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   begin
    Increase_file_handle_count:=false;
    Dec (FileHandleCount, 10);
   end
  else
    Increase_file_handle_count:=true;
end;


function dos_version : word;
var
  regs   : trealregs;
begin
  regs.realeax := $3000;
  sysrealintr($21,regs);
  dos_version := regs.realeax
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_open(var f;p:pchar;flags:longint; pchangeable: boolean);
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> graemeg/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> origin/cpstrnew
=======
procedure do_open(var f;p:pchar;flags:longint);
>>>>>>> origin/fixes_2.4
{
  filerec and textrec have both handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}
var
  regs   : trealregs;
  action : longint;
  Avoid6c00 : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldp : pchar;
begin
=======
begin
  DoDirSeparators(p);
>>>>>>> graemeg/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> graemeg/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> graemeg/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> origin/cpstrnew
=======
begin
  DoDirSeparators(p);
>>>>>>> origin/fixes_2.4
{ check if Extended Open/Create API is safe to use }
  Avoid6c00 := lo(dos_version) < 7;
{ close first if opened }
  if ((flags and $10000)=0) then
   begin
     case filerec(f).mode of
      fminput,fmoutput,fminout : Do_Close(filerec(f).handle);
      fmclosed : ;
     else
      begin
        inoutres:=102; {not assigned}
        exit;
      end;
     end;
   end;
{ reset file handle }
  filerec(f).handle:=UnusedHandle;
  action:=$1;
{ convert filemode to filerec modes }
  case (flags and 3) of
   0 : filerec(f).mode:=fminput;
   1 : filerec(f).mode:=fmoutput;
   2 : filerec(f).mode:=fminout;
  end;
  if (flags and $1000)<>0 then
   action:=$12; {create file function}
{ empty name is special }
  if p[0]=#0 then
   begin
     case FileRec(f).mode of
       fminput :
         FileRec(f).Handle:=StdInputHandle;
       fminout, { this is set by rewrite }
       fmoutput :
         FileRec(f).Handle:=StdOutputHandle;
       fmappend :
         begin
           FileRec(f).Handle:=StdOutputHandle;
           FileRec(f).mode:=fmoutput; {fool fmappend}
         end;
     end;
     exit;
   end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  oldp:=p;
  DoDirSeparators(p,pchangeable);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{ real dos call }
  syscopytodos(longint(p),strlen(p)+1);
{$ifndef RTLLITE}
  if LFNSupport then
   regs.realeax := $716c                           { Use LFN Open/Create API }
  else
   regs.realeax:=$6c00;
{$endif RTLLITE}
   if Avoid6c00 then
     regs.realeax := $3d00 + (flags and $ff)      { For now, map to Open API }
   else
     regs.realeax := $6c00;                   { Use Extended Open/Create API }
  if byte(regs.realeax shr 8) = $3d then
    begin  { Using the older Open or Create API's }
      if (action and $00f0) <> 0 then
        regs.realeax := $3c00;                   { Map to Create/Replace API }
      regs.realds := tb_segment;
      regs.realedx := tb_offset;
    end
  else
    begin  { Using LFN or Extended Open/Create API }
      regs.realedx := action;            { action if file does/doesn't exist }
      regs.realds := tb_segment;
      regs.realesi := tb_offset;
      regs.realebx := $2000 + (flags and $ff);              { file open mode }
    end;
  regs.realecx := $20;                                     { file attributes }
  sysrealintr($21,regs);
{$ifndef RTLLITE}
  if (regs.realflags and carryflag) <> 0 then
    if lo(regs.realeax)=4 then
      if Increase_file_handle_count then
        begin
          { Try again }
          if LFNSupport then
            regs.realeax := $716c                    {Use LFN Open/Create API}
          else
            if Avoid6c00 then
              regs.realeax := $3d00+(flags and $ff) {For now, map to Open API}
            else
              regs.realeax := $6c00;            {Use Extended Open/Create API}
          if byte(regs.realeax shr 8) = $3d then
            begin  { Using the older Open or Create API's }
              if (action and $00f0) <> 0 then
                regs.realeax := $3c00;             {Map to Create/Replace API}
              regs.realds := tb_segment;
              regs.realedx := tb_offset;
            end
          else
            begin  { Using LFN or Extended Open/Create API }
              regs.realedx := action;      {action if file does/doesn't exist}
              regs.realds := tb_segment;
              regs.realesi := tb_offset;
              regs.realebx := $2000+(flags and $ff);          {file open mode}
            end;
          regs.realecx := $20;                               {file attributes}
          sysrealintr($21,regs);
        end;
{$endif RTLLITE}
  if (regs.realflags and carryflag) <> 0 then
    begin
      GetInOutRes(lo(regs.realeax));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FileRec(f).mode:=fmclosed;
      if oldp<>p then
        freemem(p);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
      exit;
    end
  else
    begin
      filerec(f).handle:=lo(regs.realeax);
{$ifndef RTLLITE}
      { for systems that have more then 20 by default ! }
      if lo(regs.realeax)>FileHandleCount then
        FileHandleCount:=lo(regs.realeax);
{$endif RTLLITE}
    end;
  if lo(regs.realeax)<max_files then
    begin
{$ifdef SYSTEMDEBUG}
       if openfiles[lo(regs.realeax)] and
          assigned(opennames[lo(regs.realeax)]) then
         begin
            Writeln(stderr,'file ',opennames[lo(regs.realeax)],'(',lo(regs.realeax),') not closed but handle reused!');
            sysfreememsize(opennames[lo(regs.realeax)],strlen(opennames[lo(regs.realeax)])+1);
         end;
{$endif SYSTEMDEBUG}
       openfiles[lo(regs.realeax)]:=true;
{$ifdef SYSTEMDEBUG}
       opennames[lo(regs.realeax)] := sysgetmem(strlen(p)+1);
       move(p^,opennames[lo(regs.realeax)]^,strlen(p)+1);
{$endif SYSTEMDEBUG}
    end;
{ append mode }
  if ((flags and $100) <> 0) and
   (FileRec (F).Handle <> UnusedHandle) then
   begin
     do_seekend(filerec(f).handle);
     filerec(f).mode:=fmoutput; {fool fmappend}
   end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if oldp<>p then
    freemem(p);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
end;

function do_isdevice(handle:THandle):boolean;
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realeax:=$4400;
  sysrealintr($21,regs);
  do_isdevice:=(regs.realedx and $80)<>0;
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
   { Keep Track of open files }
   const
      max_files = 50;
   var
      openfiles : array [0..max_files-1] of boolean;
{$ifdef SYSTEMDEBUG}
      opennames : array [0..max_files-1] of pchar;
   const
      free_closed_names : boolean = true;
{$endif SYSTEMDEBUG}

{****************************************************************************
                        Low level File Routines
 ****************************************************************************}

procedure do_close(handle : longint);
var
  regs : trealregs;
begin
  if Handle<=4 then
   exit;
  regs.realebx:=handle;
  if handle<max_files then
    begin
       openfiles[handle]:=false;
{$ifdef SYSTEMDEBUG}
       if assigned(opennames[handle]) and free_closed_names then
         begin
            sysfreememsize(opennames[handle],strlen(opennames[handle])+1);
            opennames[handle]:=nil;
         end;
{$endif SYSTEMDEBUG}
    end;
  regs.realeax:=$3e00;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

procedure do_erase(p : pchar);
var
  regs : trealregs;
begin
  DoDirSeparators(p);
  syscopytodos(longint(p),strlen(p)+1);
  regs.realedx:=tb_offset;
  regs.realds:=tb_segment;
  if LFNSupport then
   regs.realeax:=$7141
  else
   regs.realeax:=$4100;
  regs.realesi:=0;
  regs.realecx:=0;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

procedure do_rename(p1,p2 : pchar);
var
  regs : trealregs;
begin
  DoDirSeparators(p1);
  DoDirSeparators(p2);
  if strlen(p1)+strlen(p2)+3>tb_size then
   HandleError(217);
  sysseg_move(get_ds,sizeuint(p2),dos_selector,tb,strlen(p2)+1);
  sysseg_move(get_ds,sizeuint(p1),dos_selector,tb+strlen(p2)+2,strlen(p1)+1);
  regs.realedi:=tb_offset;
  regs.realedx:=tb_offset + strlen(p2)+2;
  regs.realds:=tb_segment;
  regs.reales:=tb_segment;
  if LFNSupport then
   regs.realeax:=$7156
  else
   regs.realeax:=$5600;
  regs.realecx:=$ff;            { attribute problem here ! }
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

function do_write(h:longint;addr:pointer;len : longint) : longint;
var
  regs      : trealregs;
  size,
  writesize : longint;
begin
  writesize:=0;
  while len > 0 do
   begin
     if len>tb_size then
      size:=tb_size
     else
      size:=len;
     syscopytodos(ptrint(addr)+writesize,size);
     regs.realecx:=size;
     regs.realedx:=tb_offset;
     regs.realds:=tb_segment;
     regs.realebx:=h;
     regs.realeax:=$4000;
     sysrealintr($21,regs);
     if (regs.realflags and carryflag) <> 0 then
      begin
        GetInOutRes(lo(regs.realeax));
        exit(writesize);
      end;
     inc(writesize,lo(regs.realeax));
     dec(len,lo(regs.realeax));
     { stop when not the specified size is written }
     if lo(regs.realeax)<size then
      break;
   end;
  Do_Write:=WriteSize;
end;

function do_read(h:longint;addr:pointer;len : longint) : longint;
var
  regs     : trealregs;
  size,
  readsize : longint;
begin
  readsize:=0;
  while len > 0 do
   begin
     if len>tb_size then
      size:=tb_size
     else
      size:=len;
     regs.realecx:=size;
     regs.realedx:=tb_offset;
     regs.realds:=tb_segment;
     regs.realebx:=h;
     regs.realeax:=$3f00;
     sysrealintr($21,regs);
     if (regs.realflags and carryflag) <> 0 then
      begin
        GetInOutRes(lo(regs.realeax));
        do_read:=0;
        exit;
      end;
     syscopyfromdos(ptrint(addr)+readsize,lo(regs.realeax));
     inc(readsize,lo(regs.realeax));
     dec(len,lo(regs.realeax));
     { stop when not the specified size is read }
     if lo(regs.realeax)<size then
      break;
   end;
  do_read:=readsize;
end;


function do_filepos(handle : longint) : longint;
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realecx:=0;
  regs.realedx:=0;
  regs.realeax:=$4201;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   Begin
     GetInOutRes(lo(regs.realeax));
     do_filepos:=0;
   end
  else
   do_filepos:=lo(regs.realedx) shl 16+lo(regs.realeax);
end;


procedure do_seek(handle,pos : longint);
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realecx:=pos shr 16;
  regs.realedx:=pos and $ffff;
  regs.realeax:=$4200;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;



function do_seekend(handle:longint):longint;
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realecx:=0;
  regs.realedx:=0;
  regs.realeax:=$4202;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   Begin
     GetInOutRes(lo(regs.realeax));
     do_seekend:=0;
   end
  else
   do_seekend:=lo(regs.realedx) shl 16+lo(regs.realeax);
end;


function do_filesize(handle : longint) : longint;
var
  aktfilepos : longint;
begin
  aktfilepos:=do_filepos(handle);
  do_filesize:=do_seekend(handle);
  do_seek(handle,aktfilepos);
end;


{ truncate at a given position }
procedure do_truncate (handle,pos:longint);
var
  regs : trealregs;
begin
  do_seek(handle,pos);
  regs.realecx:=0;
  regs.realedx:=tb_offset;
  regs.realds:=tb_segment;
  regs.realebx:=handle;
  regs.realeax:=$4000;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

const
  FileHandleCount : longint = 20;

function Increase_file_handle_count : boolean;
var
  regs : trealregs;
begin
  Inc(FileHandleCount,10);
  regs.realebx:=FileHandleCount;
  regs.realeax:=$6700;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   begin
    Increase_file_handle_count:=false;
    Dec (FileHandleCount, 10);
   end
  else
    Increase_file_handle_count:=true;
end;


function dos_version : word;
var
  regs   : trealregs;
begin
  regs.realeax := $3000;
  sysrealintr($21,regs);
  dos_version := regs.realeax
end;


procedure do_open(var f;p:pchar;flags:longint);
{
  filerec and textrec have both handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}
var
  regs   : trealregs;
  action : longint;
  Avoid6c00 : boolean;
begin
  DoDirSeparators(p);
{ check if Extended Open/Create API is safe to use }
  Avoid6c00 := lo(dos_version) < 7;
{ close first if opened }
  if ((flags and $10000)=0) then
   begin
     case filerec(f).mode of
      fminput,fmoutput,fminout : Do_Close(filerec(f).handle);
      fmclosed : ;
     else
      begin
        inoutres:=102; {not assigned}
        exit;
      end;
     end;
   end;
{ reset file handle }
  filerec(f).handle:=UnusedHandle;
  action:=$1;
{ convert filemode to filerec modes }
  case (flags and 3) of
   0 : filerec(f).mode:=fminput;
   1 : filerec(f).mode:=fmoutput;
   2 : filerec(f).mode:=fminout;
  end;
  if (flags and $1000)<>0 then
   action:=$12; {create file function}
{ empty name is special }
  if p[0]=#0 then
   begin
     case FileRec(f).mode of
       fminput :
         FileRec(f).Handle:=StdInputHandle;
       fminout, { this is set by rewrite }
       fmoutput :
         FileRec(f).Handle:=StdOutputHandle;
       fmappend :
         begin
           FileRec(f).Handle:=StdOutputHandle;
           FileRec(f).mode:=fmoutput; {fool fmappend}
         end;
     end;
     exit;
   end;
{ real dos call }
  syscopytodos(longint(p),strlen(p)+1);
{$ifndef RTLLITE}
  if LFNSupport then
   regs.realeax := $716c                           { Use LFN Open/Create API }
  else
   regs.realeax:=$6c00;
{$endif RTLLITE}
   if Avoid6c00 then
     regs.realeax := $3d00 + (flags and $ff)      { For now, map to Open API }
   else
     regs.realeax := $6c00;                   { Use Extended Open/Create API }
  if byte(regs.realeax shr 8) = $3d then
    begin  { Using the older Open or Create API's }
      if (action and $00f0) <> 0 then
        regs.realeax := $3c00;                   { Map to Create/Replace API }
      regs.realds := tb_segment;
      regs.realedx := tb_offset;
    end
  else
    begin  { Using LFN or Extended Open/Create API }
      regs.realedx := action;            { action if file does/doesn't exist }
      regs.realds := tb_segment;
      regs.realesi := tb_offset;
      regs.realebx := $2000 + (flags and $ff);              { file open mode }
    end;
  regs.realecx := $20;                                     { file attributes }
  sysrealintr($21,regs);
{$ifndef RTLLITE}
  if (regs.realflags and carryflag) <> 0 then
    if lo(regs.realeax)=4 then
      if Increase_file_handle_count then
        begin
          { Try again }
          if LFNSupport then
            regs.realeax := $716c                    {Use LFN Open/Create API}
          else
            if Avoid6c00 then
              regs.realeax := $3d00+(flags and $ff) {For now, map to Open API}
            else
              regs.realeax := $6c00;            {Use Extended Open/Create API}
          if byte(regs.realeax shr 8) = $3d then
            begin  { Using the older Open or Create API's }
              if (action and $00f0) <> 0 then
                regs.realeax := $3c00;             {Map to Create/Replace API}
              regs.realds := tb_segment;
              regs.realedx := tb_offset;
            end
          else
            begin  { Using LFN or Extended Open/Create API }
              regs.realedx := action;      {action if file does/doesn't exist}
              regs.realds := tb_segment;
              regs.realesi := tb_offset;
              regs.realebx := $2000+(flags and $ff);          {file open mode}
            end;
          regs.realecx := $20;                               {file attributes}
          sysrealintr($21,regs);
        end;
{$endif RTLLITE}
  if (regs.realflags and carryflag) <> 0 then
    begin
      GetInOutRes(lo(regs.realeax));
      exit;
    end
  else
    begin
      filerec(f).handle:=lo(regs.realeax);
{$ifndef RTLLITE}
      { for systems that have more then 20 by default ! }
      if lo(regs.realeax)>FileHandleCount then
        FileHandleCount:=lo(regs.realeax);
{$endif RTLLITE}
    end;
  if lo(regs.realeax)<max_files then
    begin
{$ifdef SYSTEMDEBUG}
       if openfiles[lo(regs.realeax)] and
          assigned(opennames[lo(regs.realeax)]) then
         begin
            Writeln(stderr,'file ',opennames[lo(regs.realeax)],'(',lo(regs.realeax),') not closed but handle reused!');
            sysfreememsize(opennames[lo(regs.realeax)],strlen(opennames[lo(regs.realeax)])+1);
         end;
{$endif SYSTEMDEBUG}
       openfiles[lo(regs.realeax)]:=true;
{$ifdef SYSTEMDEBUG}
       opennames[lo(regs.realeax)] := sysgetmem(strlen(p)+1);
       move(p^,opennames[lo(regs.realeax)]^,strlen(p)+1);
{$endif SYSTEMDEBUG}
    end;
{ append mode }
  if ((flags and $100) <> 0) and
   (FileRec (F).Handle <> UnusedHandle) then
   begin
     do_seekend(filerec(f).handle);
     filerec(f).mode:=fmoutput; {fool fmappend}
   end;
end;

function do_isdevice(handle:THandle):boolean;
var
  regs : trealregs;
begin
  regs.realebx:=handle;
  regs.realeax:=$4400;
  sysrealintr($21,regs);
  do_isdevice:=(regs.realedx and $80)<>0;
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
