{
    Copyright (c) 1999-2002 by the FPC Development Team

    Add multiple FPC units into a static/shared library

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

 ****************************************************************************}
{$ifndef TP}
  {$H+}
{$endif}
Program ppumove;
uses

{$IFNDEF USE_FAKE_SYSUTILS}
  sysutils,
{$ELSE}
  fksysutl,
{$ENDIF}

{$ifdef unix}
  Baseunix,Unix, Dos,
{$else unix}
  dos,
{$endif unix}
  cutils,ppu,entfile,systems,
  getopts;

const
  Version   = 'Version 2.1.1';
  Title     = 'PPU-Mover';
  Copyright = 'Copyright (c) 1998-2007 by the Free Pascal Development Team';

  ShortOpts = 'o:e:d:i:P:qhsvb';
  BufSize = 4096;
  PPUExt = 'ppu';
  ObjExt = 'o';
  StaticLibExt ='a';
{$ifdef unix}
  SharedLibExt ='so';
  BatchExt     ='.sh';
{$else}
  SharedLibExt ='dll';
  BatchExt     ='.bat';
{$endif unix}

  { link options }
  link_none    = $0;
  link_always  = $1;
  link_static  = $2;
  link_smart   = $4;
  link_shared  = $8;
  link_lto     = $10;

Type
  PLinkOEnt = ^TLinkOEnt;
  TLinkOEnt = record
    Name : string;
    Next : PLinkOEnt;
  end;

Var
  ArBin,LDBin,StripBin,
  OutputFileForPPU,
  OutputFile,
  OutputFileForLink,  { the name of the output file needed when linking }
  InputPath,
  DestPath,
  PPLExt,
  LibExt,
  BinutilsPrefix      : string;
  DoStrip,
  Batch,
  Quiet,
  MakeStatic  : boolean;
  Buffer      : Pointer;
  ObjFiles    : PLinkOEnt;
  BatchFile   : Text;
  Libs        : ansistring;

{*****************************************************************************
                                 Helpers
*****************************************************************************}

Procedure Error(const s:string;stop:boolean);
{
  Write an error message to stderr
}
begin
  writeln(stderr,s);
  if stop then
   halt(1);
end;


function Shell(const s:string):longint;
{
  Run a shell commnad and return the exitcode
}
begin
  if Batch then
   begin
     Writeln(BatchFile,s);
     Shell:=0;
     exit;
   end;
{$ifdef unix}
  Shell:=unix.fpsystem(s);
{$else}
  exec(getenv('COMSPEC'),'/C '+s);
  Shell:=DosExitCode;
{$endif}
end;


Function FileExists (Const F : String) : Boolean;
{
  Returns True if the file exists, False if not.
}
Var
{$ifdef unix}
  info : Stat;
{$else}
  info : searchrec;
{$endif}
begin
{$ifdef unix}
  FileExists:=FpStat(F,Info)=0;
{$else}
  FindFirst (F,anyfile,Info);
  FileExists:=DosError=0;
{$endif}
end;


Function ChangeFileExt(Const HStr,ext:String):String;
{
  Return a filename which will have extension ext added if no
  extension is found
}
var
  j : longint;
begin
  j:=length(Hstr);
  while (j>0) and (Hstr[j]<>'.') do
   dec(j);
  if j=0 then
   ChangeFileExt:=Hstr+'.'+Ext
  else
   ChangeFileExt:=HStr;
end;


Function ForceExtension(Const HStr,ext:String):String;
{
  Return a filename which certainly has the extension ext
}
var
  j : longint;
begin
  j:=length(Hstr);
  while (j>0) and (Hstr[j]<>'.') do
   dec(j);
  if j=0 then
   j:=255;
  ForceExtension:=Copy(Hstr,1,j-1)+'.'+Ext;
end;


Procedure AddToLinkFiles(const S : String);
{
  Adds a filename to a list of object files to link to.
  No duplicates allowed.
}
Var
  P : PLinKOEnt;
begin
  P:=ObjFiles;
  { Don't add files twice }
  While (P<>nil) and (p^.name<>s) do
    p:=p^.next;
  if p=nil then
   begin
     new(p);
     p^.next:=ObjFiles;
     p^.name:=s;
     ObjFiles:=P;
   end;
end;


Function ExtractLib(const libfn:string):string;
{
  Extract a static library libfn and return the files with a
  wildcard
}
var
  n : namestr;
  d : dirstr;
  e : extstr;
begin
{ create the temp dir first }
  fsplit(libfn,d,n,e);
  {$push}{$I-}
   mkdir(n+'.sl');
  {$pop}
  if ioresult<>0 then;
{ Extract }
  if Shell(arbin+' x '+libfn)<>0 then
   Error('Fatal: Error running '+arbin,true);
{ Remove the lib file, it's extracted so it can be created with ease }
  if PPLExt=PPUExt then
   Shell('rm '+libfn);
{$ifdef unix}
  ExtractLib:=n+'.sl/*';
{$else}
  ExtractLib:=n+'.sl\*';
{$endif}
end;


Function DoPPU(const PPUFn,PPLFn:String):Boolean;
{
  Convert one file (in Filename) to library format.
  Return true if successful, false otherwise.
}
Var
  inppu,
  outppu : tppufile;
  b,
  untilb : byte;
  l,m    : longint;
  f      : file;
  ext,
  s      : string;
  ppuversion,
  ppulongversion: dword;
  gotltofiles: boolean;
begin
  DoPPU:=false;
  gotltofiles:=false;
  If Not Quiet then
   Write ('Processing ',PPUFn,'...');
  inppu:=tppufile.create(PPUFn);
  if not inppu.openfile then
   begin
     inppu.free;
     Error('Error: Could not open : '+PPUFn,false);
     Exit;
   end;
{ Check the ppufile }
  if not inppu.CheckPPUId then
   begin
     inppu.free;
     Error('Error: Not a PPU File : '+PPUFn,false);
     Exit;
   end;
  ppuversion:=inppu.getversion;
  if ppuversion<CurrentPPUVersion then
   begin
     inppu.free;
     Error('Error: Wrong PPU Version '+tostr(ppuversion)+' in '+PPUFn,false);
     Exit;
   end;
{ No .o file generated for this ppu, just skip }
  if (inppu.header.common.flags and uf_no_link)<>0 then
   begin
     inppu.free;
     If Not Quiet then
      Writeln (' No files.');
     DoPPU:=true;
     Exit;
   end;
{ Already a lib? }
  if (inppu.header.common.flags and uf_in_library)<>0 then
   begin
     inppu.free;
     Error('Error: PPU is already in a library : '+PPUFn,false);
     Exit;
   end;
{ We need a static linked unit }
  if (inppu.header.common.flags and uf_static_linked)=0 then
   begin
     inppu.free;
     Error('Error: PPU is not static linked : '+PPUFn,false);
     Exit;
   end;
{ Check if shared is allowed }
  if tsystem(inppu.header.common.target) in [system_i386_go32v2] then
   begin
     Writeln('Warning: shared library not supported for ppu target, switching to static library');
     MakeStatic:=true;
   end;
{ Create the new ppu }
  if PPUFn=PPLFn then
   outppu:=tppufile.create('ppumove.$$$')
  else
   outppu:=tppufile.create(PPLFn);
  outppu.createfile;
{ Create new header, with the new flags }
  outppu.header:=inppu.header;
  outppu.header.common.flags:=outppu.header.common.flags or uf_in_library;
  if MakeStatic then
   outppu.header.common.flags:=outppu.header.common.flags or uf_static_linked
  else
   outppu.header.common.flags:=outppu.header.common.flags or uf_shared_linked;
{ read until the object files are found }
  untilb:=iblinkunitofiles;
  repeat
    b:=inppu.readentry;
    if b in [ibendinterface,ibend] then
     begin
       inppu.free;
       outppu.free;
       Error('Error: No files to be linked found : '+PPUFn,false);
       Exit;
     end;
    if b<>untilb then
     begin
       if b=ibextraheader then
         begin
           ppulongversion:=cardinal(inppu.getlongint);
           if ppulongversion<>CurrentPPULongVersion then
             begin
               inppu.free;
               outppu.free;
               Error('Error: Wrong PPU Long Version '+tostr(ppulongversion)+' in '+PPUFn,false);
               Exit;
             end;
           outppu.putlongint(longint(ppulongversion));
         end;
       repeat
         inppu.getdatabuf(buffer^,bufsize,l);
         outppu.putdata(buffer^,l);
       until l<bufsize;
       outppu.writeentry(b);
     end;
  until (b=untilb);
{ we have now reached the section for the files which need to be added,
  now add them to the list }
  case b of
    iblinkunitofiles :
      begin
        { add all o files, and save the entry when not creating a static
          library to keep staticlinking possible (also keep possibility
          to link lto) }
        while not inppu.endofentry do
         begin
           s:=inppu.getstring;
           m:=inppu.getlongint;
           if (not MakeStatic) or
              ((m and link_lto)<>0) then
            begin
              gotltofiles:=gotltofiles or ((m and link_lto)<>0);
              outppu.putstring(s);
              outppu.putlongint(m);
            end;
           AddToLinkFiles(s);
         end;
        if not MakeStatic or
           gotltofiles then
         outppu.writeentry(b);
      end;
{    iblinkunitstaticlibs :
      begin
        AddToLinkFiles(ExtractLib(inppu.getstring));
        if not inppu.endofentry then
         begin
           repeat
             inppu.getdatabuf(buffer^,bufsize,l);
             outppu.putdata(buffer^,l);
           until l<bufsize;
           outppu.writeentry(b);
         end;
       end; }
  end;
{ just add a new entry with the new lib }
  if MakeStatic then
   begin
     outppu.putstring(OutputfileForPPU);
     if not gotltofiles then
       outppu.putlongint(link_static)
     else
       outppu.putlongint(link_static or link_lto);
     outppu.writeentry(iblinkunitstaticlibs)
   end
  else
   begin
     outppu.putstring(OutputfileForPPU);
     if not gotltofiles then
       outppu.putlongint(link_shared)
     else
       outppu.putlongint(link_shared or link_lto);
     outppu.writeentry(iblinkunitsharedlibs);
   end;
{ read all entries until the end and write them also to the new ppu }
  repeat
    b:=inppu.readentry;
  { don't write ibend, that's written automatically }
    if b<>ibend then
     begin
       if b=iblinkothersharedlibs then
         begin
           while not inppu.endofentry do
             begin
               s:=inppu.getstring;
               m:=inppu.getlongint;

               outppu.putstring(s);

               { strip lib prefix }
               if copy(s,1,3)='lib' then
                 delete(s,1,3);

               { strip lib prefix }
               if copy(s,1,3)='lib' then
                 delete(s,1,3);
               ext:=ExtractFileExt(s);
               if ext<>'' then
                 delete(s,length(s)-length(ext)+1,length(ext));

               libs:=libs+' -l'+s;

               outppu.putlongint(m);
             end;
         end
       else
         repeat
           inppu.getdatabuf(buffer^,bufsize,l);
           outppu.putdata(buffer^,l);
         until l<bufsize;
       outppu.writeentry(b);
     end;
  until b=ibend;
{ write the last stuff and close }
  outppu.flush;
  outppu.writeheader;
  outppu.free;
  inppu.free;
{ rename }
  if PPUFn=PPLFn then
   begin
     {$push}{$I-}
      assign(f,PPUFn);
      erase(f);
      assign(f,'ppumove.$$$');
      rename(f,PPUFn);
     {$pop}
     if ioresult<>0 then;
   end;
{ the end }
  If Not Quiet then
   Writeln (' Done.');
  DoPPU:=True;
end;


Function DoFile(const FileName:String):Boolean;
{
  Process a file, mainly here for wildcard support under Dos
}
{$ifndef unix}
var
  dir : searchrec;
{$endif}
begin
{$ifdef unix}
  DoFile:=DoPPU(InputPath+FileName,InputPath+ForceExtension(FileName,PPLExt));
{$else}
  DoFile:=false;
  findfirst(filename,$20,dir);
  while doserror=0 do
   begin
     if not DoPPU(InputPath+Dir.Name,InputPath+ForceExtension(Dir.Name,PPLExt)) then
      exit;
     findnext(dir);
   end;
  findclose(dir);
  DoFile:=true;
{$endif}
end;


Procedure DoLink;
{
  Link the object files together to form a (shared) library
}
Var
  Names : ansistring;
  f     : file;
  Err   : boolean;
  P     : PLinkOEnt;
begin
  if not Quiet then
   Write ('Linking ');
  P:=ObjFiles;
  names:='';
  While p<>nil do
   begin
     if Names<>'' then
      Names:=Names+' '+InputPath+P^.name
     else
      Names:=InputPath+p^.Name;
     p:=p^.next;
   end;
  if Names='' then
   begin
     If not Quiet then
      Writeln('Error: no files found to be linked');
     exit;
   end;
  If not Quiet then
    WriteLn(names+Libs);
{ Run ar or ld to create the lib }
  If MakeStatic then
   Err:=Shell(arbin+' rs '+outputfile+' '+names)<>0
  else
   begin
     Err:=Shell(BinutilsPrefix+ldbin+' -shared -E -o '+OutputFile+' '+names+' '+libs)<>0;
     if (not Err) and dostrip then
      Shell(BinutilsPrefix+stripbin+' --strip-unneeded '+OutputFile);
   end;
  If Err then
   Error('Fatal: Library building stage failed.',true);
{ fix permission to 644, so it's not 755 }
{$ifdef unix}
  FPChmod(OutputFile,420);
{$endif}
{ Rename to the destpath }
  if DestPath<>'' then
   begin
     Assign(F, OutputFile);
     Rename(F,DestPath+DirectorySeparator+OutputFile);
   end;
end;


Procedure usage;
{
  Print usage and exit.
}
begin
  Writeln(paramstr(0),': [-qhvbsS] [-e ext] [-o name] [-d path] [-P binutils prefix] file [file ...]');
  Halt(0);
end;



Procedure processopts;
{
  Process command line opions, and checks if command line options OK.
}
var
  C : char;
begin
  if paramcount=0 then
   usage;
{ Reset }
  ObjFiles:=Nil;
  Quiet:=False;
  Batch:=False;
  DoStrip:=False;
  OutputFile:='';
  PPLExt:='ppu';
  ArBin:='ar';
  LdBin:='ld';
  StripBin:='strip';
  BinutilsPrefix:='';
  repeat
    c:=Getopt (ShortOpts);
    Case C of
      EndOfOptions : break;
      'S' : MakeStatic:=True;
      'o' : OutputFile:=OptArg;
      'd' : DestPath:=OptArg;
      'i' : begin
              InputPath:=OptArg;
              if InputPath[length(InputPath)]<>DirectorySeparator then
                InputPath:=InputPath+DirectorySeparator;
            end;
      'e' : PPLext:=OptArg;
      'q' : Quiet:=True;
      'b' : Batch:=true;
      's' : DoStrip:=true;
      '?' : Usage;
      'h' : Usage;
      'P' : BinutilsPrefix:=OptArg;
    end;
  until false;
{ Test filenames on the commandline }
  if (OptInd>Paramcount) then
   Error('Error: no input files',true);
  if (OptInd<ParamCount) and (OutputFile='') then
   Error('Error: when moving multiple units, specify an output name.',true);
{ alloc a buffer }
  GetMem (Buffer,Bufsize);
  If Buffer=Nil then
   Error('Error: could not allocate memory for buffer.',true);
end;


var
  i : longint;
begin
  Libs:='';
  ProcessOpts;
{ Write Header }
  if not Quiet then
   begin
     Writeln(Title+' '+Version);
     Writeln(Copyright);
     Writeln;
   end;
{ fix the libext and outputfilename }
  if Makestatic then
   LibExt:=StaticLibExt
  else
   LibExt:=SharedLibExt;
  if OutputFile='' then
   OutputFile:=Paramstr(OptInd);
  OutputFileForPPU:=OutputFile;
{ fix filename }
{$ifdef unix}
  if Copy(OutputFile,1,3)<>'lib' then
   OutputFile:='lib'+OutputFile;
  { For unix skip replacing the extension if a full .so.X.X if specified }
  i:=pos('.so.',Outputfile);
  if i<>0 then
   OutputFileForLink:=Copy(Outputfile,4,i-4)
  else
   begin
     OutputFile:=ForceExtension(OutputFile,LibExt);
     OutputFileForLink:=Copy(Outputfile,4,length(Outputfile)-length(LibExt)-4);
   end;
{$else}
  OutputFile:=ForceExtension(OutputFile,LibExt);
  OutputFileForLink:=OutputFile;
{$endif}
{ Open BatchFile }
  if Batch then
   begin
     Assign(BatchFile,'pmove'+BatchExt);
     Rewrite(BatchFile);
   end;
{ Process Files }
  i:=OptInd;
  While (i<=ParamCount) and Dofile(ChangeFileExt(Paramstr(i),PPUExt)) do
   Inc(i);
{ Do Linking stage }
  DoLink;
{ Close BatchFile }
  if Batch then
   begin
     if Not Quiet then
      Writeln('Writing pmove'+BatchExt);
     Close(BatchFile);
{$ifdef unix}
  FPChmod('pmove'+BatchExt,493);
{$endif}
   end;
{ The End }
  if Not Quiet then
   Writeln('Done.');
end.
