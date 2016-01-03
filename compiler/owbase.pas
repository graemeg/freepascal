{
    Copyright (c) 1998-2002 by Peter Vreman

    Contains the base stuff for writing for object files to disk

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
unit owbase;

{$i fpcdefs.inc}

interface
uses
  cstreams,
  cclasses;

type
  tobjectwriter=class
  private
    f      : TCCustomFileStream;
    opened : boolean;
    buf    : pchar;
    bufidx : longword;
    procedure writebuf;
  protected
    fsize,
    fobjsize  : longword;
  public
    constructor create;
    constructor createAr(const Aarfn:string);virtual;
    destructor  destroy;override;
    function  createfile(const fn:string):boolean;virtual;
    procedure closefile;virtual;
    procedure writesym(const sym:string);virtual;
    procedure write(const b;len:longword);virtual;
    procedure WriteZeros(l:longword);
    procedure writearray(a:TDynamicArray);
    property Size:longword read FSize;
    property ObjSize:longword read FObjSize;
  end;

  tobjectwriterclass = class of tobjectwriter;

  tobjectreader=class
  private
    f      : TCCustomFileStream;
    opened : boolean;
    buf    : pchar;
    ffilename : string;
    bufidx,
    bufmax : longint;
    function readbuf:boolean;
  protected
    function getfilename : string;virtual;
    function GetPos: longint;virtual;
    function GetIsArchive: boolean;virtual;
  public
    constructor create;
    constructor createAr(const Aarfn:string;allow_nonar:boolean=false);virtual;
    destructor  destroy;override;
    function  openfile(const fn:string):boolean;virtual;
    procedure closefile;virtual;
    procedure seek(len:longint);virtual;
    function  read(out b;len:longint):boolean;virtual;
    function  readarray(a:TDynamicArray;len:longint):boolean;
    property filename : string read getfilename;
    property size:longint read bufmax;
    property Pos:longint read GetPos;
    property IsArchive: boolean read GetIsArchive;
  end;

  tobjectreaderclass = class of tobjectreader;

implementation

uses
   SysUtils,
   verbose, globals;

const
  bufsize = 32768;


{****************************************************************************
                              TObjectWriter
****************************************************************************}

constructor tobjectwriter.create;
begin
  getmem(buf,bufsize);
  bufidx:=0;
  opened:=false;
  fsize:=0;
end;


destructor tobjectwriter.destroy;
begin
  if opened then
   closefile;
  freemem(buf,bufsize);
end;

constructor tobjectwriter.createAr(const Aarfn:string);
begin
  InternalError(2015041901);
end;


function tobjectwriter.createfile(const fn:string):boolean;
begin
  createfile:=false;
  f:=CFileStreamClass.Create(fn,fmCreate);
  if CStreamError<>0 then
    begin
       Message2(exec_e_cant_create_objectfile,fn,IntToStr(CStreamError));
       exit;
    end;
  bufidx:=0;
  fsize:=0;
  fobjsize:=0;
  opened:=true;
  createfile:=true;
end;


procedure tobjectwriter.closefile;
var
  fn : string;
begin
  if bufidx>0 then
   writebuf;
  fn:=f.filename;
  f.free;
{ Remove if size is 0 }
  if size=0 then
   DeleteFile(fn);
  opened:=false;
  fsize:=0;
  fobjsize:=0;
end;


procedure tobjectwriter.writebuf;
begin
  f.write(buf^,bufidx);
  bufidx:=0;
end;


procedure tobjectwriter.writesym(const sym:string);
begin
end;


procedure tobjectwriter.write(const b;len:longword);
var
  p   : pchar;
  bufleft,
  idx : longword;
begin
  inc(fsize,len);
  inc(fobjsize,len);
  p:=pchar(@b);
  idx:=0;
  while len>0 do
   begin
     bufleft:=bufsize-bufidx;
     if len>bufleft then
      begin
        move(p[idx],buf[bufidx],bufleft);
        dec(len,bufleft);
        inc(idx,bufleft);
        inc(bufidx,bufleft);
        writebuf;
      end
     else
      begin
        move(p[idx],buf[bufidx],len);
        inc(bufidx,len);
        exit;
      end;
   end;
end;


procedure tobjectwriter.WriteZeros(l:longword);
var
  empty : array[0..1023] of byte;
begin
  if l>sizeof(empty) then
    begin
      fillchar(empty,sizeof(empty),0);
      while l>sizeof(empty) do
        begin
          Write(empty,sizeof(empty));
          Dec(l,sizeof(empty));
        end;
      if l>0 then
        Write(empty,l);
    end
  else if l>0 then
    begin
      fillchar(empty,l,0);
      Write(empty,l);
    end;
end;


procedure tobjectwriter.writearray(a:TDynamicArray);
var
  hp : pdynamicblock;
begin
  hp:=a.firstblock;
  while assigned(hp) do
    begin
      write(hp^.data,hp^.used);
      hp:=hp^.next;
    end;
end;


{****************************************************************************
                              TObjectReader
****************************************************************************}

constructor tobjectreader.create;
begin
  buf:=nil;
  bufidx:=0;
  bufmax:=0;
  ffilename:='';
  opened:=false;
end;


destructor tobjectreader.destroy;
begin
  if opened then
    closefile;
<<<<<<< HEAD
<<<<<<< HEAD
end;


constructor tobjectreader.createAr(const Aarfn:string;allow_nonar:boolean=false);
begin
  InternalError(2015081401);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


function tobjectreader.openfile(const fn:string):boolean;
begin
  openfile:=false;
  f:=CFileStreamClass.Create(fn,fmOpenRead);
  if CStreamError<>0 then
    begin
       Comment(V_Error,'Can''t open object file: '+fn);
       exit;
    end;
  ffilename:=fn;
<<<<<<< HEAD
<<<<<<< HEAD
  bufmax:=f.Size;
  getmem(buf,bufmax);
  f.read(buf^,bufmax);
=======
  getmem(buf,f.Size);
  f.read(buf^,f.Size);
  bufmax:=f.Size;
>>>>>>> graemeg/fixes_2_2
=======
  getmem(buf,f.Size);
  f.read(buf^,f.Size);
  bufmax:=f.Size;
>>>>>>> origin/fixes_2_2
  f.free;
  bufidx:=0;
  opened:=true;
  openfile:=true;
end;


procedure tobjectreader.closefile;
begin
  opened:=false;
  bufidx:=0;
  bufmax:=0;
  freemem(buf);
end;


function tobjectreader.readbuf:boolean;
begin
  result:=bufidx<bufmax;
end;


procedure tobjectreader.seek(len:longint);
begin
  bufidx:=len;
end;


function tobjectreader.read(out b;len:longint):boolean;
begin
  result:=true;
  if bufidx+len>bufmax then
    begin
      result:=false;
      len:=bufmax-bufidx;
    end;
  move(buf[bufidx],b,len);
  inc(bufidx,len);
end;


function tobjectreader.readarray(a:TDynamicArray;len:longint):boolean;
begin
  result:=true;
  if bufidx+len>bufmax then
    begin
      result:=false;
      len:=bufmax-bufidx;
    end;
  a.write(buf[bufidx],len);
  inc(bufidx,len);
end;

function tobjectreader.getfilename : string;
  begin
    result:=ffilename;
<<<<<<< HEAD
<<<<<<< HEAD
  end;

function tobjectreader.GetPos: longint;
  begin
    Result:=bufidx;
  end;


function tobjectreader.GetIsArchive: boolean;
  begin
    Result:=false;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

end.
