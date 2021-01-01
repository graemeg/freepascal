{ Copyright (C) <2005> <Andrew Haines> chmls.lpr
  Mostly rewritten by Marco van de Voort 2009-2012

  An util that concentrates on listing and decompiling various sections
   of a CHM.

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
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

  See the file COPYING, included in this distribution,
  for details about the copyright.
}
program chmls;

{$IFDEF MSWINDOWS}
{$apptype console}
{$ENDIF}

{$mode objfpc}{$H+}

uses
  Classes, GetOpts, SysUtils, Types,
  StreamEx,
  chmreader, chmbase, chmsitemap;

{$R-} // CHM spec puts "-1" in dwords etc.
type

  { TListObject }

  TListObject = class
    Section  : Integer;
    count    : integer;
    donotpage: boolean;
    nameonly : boolean;
    procedure OnFileEntry(Name: String; Offset, UncompressedSize, ASection: Integer);
  end;

   TExtractAllObject = class
    basedir : string;
    r       : TChmReader;
    lastone_was_point : boolean;
    procedure OnFileEntry(Name: String; Offset, UncompressedSize, ASection: Integer);
  end;

Type
  TCmdEnum = (cmdList,cmdExtract,cmdExtractall,cmdUnblock,cmdextractalias,cmdextracttoc,cmdextractindex,cmdprintidxhdr,cmdprintsystem,cmdprintwindows,cmdprinttopics,cmdNone);        // One dummy element at the end avoids rangecheck errors.

Const
  CmdNames : array [TCmdEnum] of String = ('LIST','EXTRACT','EXTRACTALL','UNBLOCK','EXTRACTALIAS','EXTRACTTOC','EXTRACTINDEX','PRINTIDXHDR','PRINTSYSTEM','PRINTWINDOWS','PRINTTOPICS','');

var
  theopts : array[1..5] of TOption;


Procedure Usage;

begin
  Writeln(StdErr,'Usage: chmls [switches] [command] [command specific parameters]');
  writeln(stderr);
  writeln(stderr,'Switches : ');
  writeln(stderr,' -h, --help     : this screen');
  writeln(stderr,' -p, --no-page  : do not page list output');
  writeln(stderr,' --no-offset    : do not show "offset" column in list output');
  writeln(stderr,' -n,--name-only : only show "name" column in list output');
  writeln(stderr);
  writeln(stderr,'Where command is one of the following or if omitted, equal to LIST.');
  writeln(stderr,' list       <filename> [section number] ');
  writeln(stderr,'            Shows contents of the archive''s directory');
  writeln(stderr,' extract    <chm filename> <filename to extract> [saveasname]');
  writeln(stderr,'            Extracts file "filename to get" from archive "filename",');
  writeln(stderr,'            and, if specified, saves it to [saveasname]');
  writeln(stderr,' extractall <chm filename> [directory]');
  writeln(stderr,'            Extracts all files from archive "filename" to directory ');
  writeln(stderr,'            "directory"');
  writeln(stderr,' unblockchm <filespec1> [filespec2] ..' );
  writeln(stderr,'            Mass unblocks (XPsp2+) the relevant CHMs. Multiple files');
  writeln(stderr,'            and wildcards allowed');
  writeln(stderr,' extractalias <chmfilename> [basefilename] [symbolprefix]' );
  writeln(stderr,'            Extracts context info from file "chmfilename" ');
  writeln(stderr,'            to a "basefilename".h and "basefilename".ali,');
  writeln(stderr,'            using symbols "symbolprefix"contextnr');
  writeln(stderr,' extracttoc <chmfilename> [filename]');
  writeln(stderr,'            Extracts the toc (mainly to check binary TOC)');
  writeln(stderr,' extractindex <chmfilename> [filename]');
  writeln(stderr,'            Extracts the index (mainly to check binary index)');
  writeln(stderr,' printidxhdr <chmfilename>');
  writeln(stderr,'            prints #IDXHDR in readable format ');
  writeln(stderr,' printsystem <chmfilename>');
  writeln(stderr,'            prints #SYSTEM in readable format ');
  writeln(stderr,' printwindows <chmfilename>');
  writeln(stderr,'            prints #WINDOWS in readable format ');
  writeln(stderr,' printtopics <chmfilename>');
  writeln(stderr,'            prints #TOPICS in readable format ');

  Halt(1);
end;

procedure WrongNrParam(cmd:string;number:integer);

begin
  writeln(stderr,' Wrong number of parameters for ',cmd,' ',number);
  usage;
  halt(1);
end;

procedure InitOptions;
begin
  with theopts[1] do
   begin
    name:='help';
    has_arg:=0;
    flag:=nil;
    value:=#0;
  end;
  with theopts[2] do
   begin
    name:='name-only';
    has_arg:=0;
    flag:=nil;
  end;
  with theopts[3] do
   begin
    name:='no-page';
    has_arg:=0;
    flag:=nil;
  end;
  with theopts[4] do
   begin
    name:='no-offset';
    has_arg:=0;
    flag:=nil;
  end;
  with theopts[5] do
   begin
    name:='';
    has_arg:=0;
    flag:=nil;
  end;
end;

procedure WriteStrAdj(Str: String; CharWidth: Integer);
// Changed to WriteStrADJ (for adjust), since 2.4.0 writestr is a builtin
// Why doesn't Write() allow left aligned columns?, sigh.
  var
    OutString: String;
    Len: Integer;
  begin
    Len := Length(Str);
    SetLength(OutString, CharWidth-Len);
    FillChar(OutString[1], CharWidth-Len, ' ');
    Write(OutString + Str); // to stdout
  end;

function craftpath(pth:string;filename:String):string;

var lenpth,lenfn:integer;
    pthends,filenameends : Boolean;
begin
  lenpth:=length(pth); lenfn :=length(filename);

  if lenpth=0 then
    exit(filename);

  pthends:=false;  filenameends:=false;
  if (lenpth>0) and (pth[lenpth] in ['/','\']) then
    pthends:=true;

  if (lenfn>0) and (filename[1] in ['/','\']) then
    filenameends:=true;

  if pthends and filenameends then
      result:=copy(pth,1,lenpth-1)+filename
  else
    if pthends or filenameends then
        result:=pth+filename
    else
       result:=pth+pathsep+filename;
end;


var donotshowoffset : boolean=false;

procedure TListObject.OnFileEntry(Name: String; Offset, UncompressedSize,
  ASection: Integer);
begin
  Inc(Count);
  if (Section > -1) and (ASection <> Section) then Exit;
  if (Count = 1) or ((Count mod 40 = 0) and not donotpage) then
    begin
      Write(StdErr, '<Section> ');
      if not donotshowoffset then
        Write(StdErr, '<Offset> ');
      Writeln(StdErr, '<UnCompSize>  <Name>');
    end;
  if not nameonly then
    begin
      Write(' ');
      Write(ASection);
      Write('      ');
      if not donotshowoffset then
        begin
          WriteStrAdj(IntToStr(Offset), 10);
          Write('  ');
        end;
      WriteStrAdj(IntToStr(UncompressedSize), 11);
      Write('  ');
    end;
  WriteLn(Name);
end;

procedure TExtractAllObject.OnFileEntry(Name: String; Offset, UncompressedSize,
  ASection: Integer);
var mem : TMemoryStream;
    s   : String;
    len : integer;
procedure wrpoint;
begin
      if lastone_was_point then
        writeln;
      lastone_was_point:=false;
end;
begin
  len:=Length(Name);
  if ((Len>0) and (name[len]='/')) then
    exit; // directory or empty file

  if (UncompressedSize=0) Then
    begin
      WrPoint;
      Writeln(stderr,'Skipping empty file ',Name);
      exit;
    end;
  if ((Len>0) and (name[1]=':')) then
    begin
      WrPoint;
      Writeln(stderr,'Skipping internal file : ',Name);
      exit;
    end;
  mem:=r.getobject(name);
  if assigned(mem) then
    begin
      s:=craftpath(basedir,name);
      ForceDirectories(extractfiledir(s));
      try
         mem.savetofile(s);
         write('.');
         lastone_was_point:=true;
      except
        on e : exception do
          begin
            WrPoint;
            Writeln(Stderr,'Error saving ',name,' to ',s,'.'            );
          end;
       end;
    end
  else
    begin
      Writeln(Stderr,'Can''t extract ',name);
    end;
end;

var donotpage:boolean=false;
    name_only :boolean=false;

procedure ListChm(Const Name:string;Section:Integer);
var
  ITS: TITSFReader;
  Stream: TFileStream;
  JunkObject: TListObject;

begin
  if not Fileexists(name) then
    begin
      writeln(stderr,' Can''t find file ',name);
      halt(1);
    end;

  Stream := TFileStream.Create(name, fmOpenRead);
  JunkObject := TListObject.Create;
  JunkObject.Section:=Section;
  JunkObject.Count:=0;
  JunkObject.DoNotPage:=DoNotPage;
  JunkObject.NameOnly:=Name_Only;

  ITS:= TITSFReader.Create(Stream, True);
  ITS.GetCompleteFileList(@JunkObject.OnFileEntry);

  WriteLn('Total Files in chm: ', JunkObject.Count);
  ITS.Free;
  JunkObject.Free;
end;

procedure ExtractFile(chm,readfrom,saveto:string);
var
  fs: TFileStream;
  m : TMemoryStream;
  r : TChmReader;
begin
  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;

  if (length(readfrom)>1) and (readfrom[1]<>'/') then
    readfrom:='/'+readfrom;

  fs:=TFileStream.create(chm,fmOpenRead or fmShareDenyNone);
  r:=TChmReader.Create(fs,True);
  m:=r.getobject(readfrom);
  if assigned(m) then
    begin
      try
        Writeln('Extracting ms-its:/',chm,'::',readfrom,' to ',saveto);
        m.savetofile(saveto);
      except
        on e : exception do
          writeln('Can''t write to file ',saveto);
        end;
     end
  else
    begin
      writeln(stderr,'Can''t find file ',readfrom,' in ',chm);
      halt(1);
    end;
end;

procedure ExtractFileAll(chm,dirto2:string);
var
  fs: TFileStream;
  m : TMemoryStream;
  r : TChmReader;
  fl : boolean;
  ListAll : TExtractAllObject;
begin
  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;


  if not directoryexists(dirto2) then
    begin
      fl:=false;
      try
        mkdir(dirto2);
        fl:=directoryexists(dirto2);
      except
       on e : exception do ;
       end;
      if not fl then
        begin
          writeln(stderr,'Directory ',dirto2,' doesn''t exist, and trying to create it fails');
          halt(1);
        end;
      end;


  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  Listall:= TExtractAllObject.Create;
  ListAll.basedir:=dirto2;
  ListAll.r:=r;
  ListAll.lastone_was_point:=false;
  r.GetCompleteFileList(@ListAll.OnFileEntry);

  r.free;
end;

procedure ExtractAlias(filespec:TStringDynArray);

var s,
    chm,
    prefixfn,
    symbolname : string;
    i,cnt: integer;
    cl : TList;
    x : PcontextItem;
    f : textfile;
    fs: TFileStream;
    r : TChmReader;

begin
  symbolname:='helpid';
  chm:=filespec[0];
  prefixfn:=changefileext(chm,'');
  if length(filespec)>1 then
    prefixfn:=filespec[1];
  if length(filespec)>2 then
    symbolname:=filespec[2];


  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;
  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  cl:=r.contextlist;
  if assigned(cl) and (cl.count>0) then
    begin
      cnt:=cl.count;
      assignfile(f,changefileext(chm,'.ali'));
      rewrite(f);
      for i:=0 to cnt-1 do
        begin
          x:=pcontextitem(cl[i]);
          s:=x^.url;
          if (length(s)>0) and (s[1]='/') then
            delete(s,1,1);

          writeln(f,symbolname,x^.context,'=',s);
        end;
      closefile(f);
      assignfile(f,changefileext(chm,'.h'));
      rewrite(f);
      for i:=0 to cnt-1 do
        begin
          x:=pcontextitem(cl[i]);
          writeln(f,'#define ',symbolname,x^.context,' ',x^.context);
        end;
      closefile(f);
    end;
   r.free;
end;


procedure unblockchm(s:string);
var f : file;
begin
 writeln('unblocking ',s);
 assignfile(f,s+':Zone.Identifier');
 rewrite(f,1);
 truncate(f);
 closefile(f);
end;

procedure populatefiles(files:TStringlist;filespec:string);
var
  searchResult : TSearchRec;
begin
 if FindFirst(filespec, faAnyFile, searchResult) = 0 then
  begin
    repeat
      files.add(searchresult.name);
    until FindNext(searchResult) <> 0;
    // Must free up resources used by these successful finds
    FindClose(searchResult);
  end;
end;

procedure unblockchms(filespec:TStringDynArray);

var files : TStringList;
    i : Integer;

begin
 files :=TStringList.create;
 try
   for i:=0 to length(filespec)-1 do
    populatefiles(files,filespec[i]);
 except
   writeln(stderr,'Error while scanning directory ',filespec[i]);
   writeln(stderr,'Exiting....');
   halt(1);
  end;
 if files.count>0 then
   for i:=0 to files.count-1 do
     unblockchm(files[i]);
 Files.Free;
end;


procedure readchunk13(m:TMemoryStream;r:TChmReader);

var i,cnt,cnt2: integer;
    s : ansistring;

procedure fetchstring;

begin
  cnt:=m.ReadDWordLE;
  s:='';
  if (cnt>0) then
   s:=r.readstringsentry(cnt);
end;

var dx : dword;
begin
  setlength(s,4);
  for i:=1 to 4 do
    s[i]:=ansichar(m.readbyte);
  Writeln('Identifier tag                                :',s);
  Writeln('Unknown timestamp/checksum                    :',leton(m.readdword));
  Writeln('Always 1                                      :',leton(m.readdword));
  Writeln('Number of topic nodes incl. contents & index  :',leton(m.readdword));
  Writeln('    The following are mostly parameters of the "text/site properties" object of the sitemap contents');
  Writeln('0 (meaning unknown)                           :',leton(m.readdword));
  fetchstring;
  Writeln('Imagelist param index in #strings (0,-1=none) :',cnt);
  if (cnt>0) then
      writeln('    = ',s);
  Writeln('0 (meaning unknown)                           :',leton(m.readdword));
  cnt:=m.ReadDWordLE;
  if cnt=1 then
    s:='Folder'
  else
    if cnt=0 then
      s:='None'
    else
      s:='unknown value!';
  Writeln('imagetype param text/site.                    :',cnt,' = ',s);
  Writeln('Background value                              :',inttohex(leton(m.readdword),8));
  Writeln('Foreground value                              :',inttohex(leton(m.readdword),8));
  fetchstring;
  Writeln('Font  param index in #strings (0,-1=none)     :',cnt);
  if (cnt>0) then
      writeln('    = ',s);
  Writeln('Windows Styles                                :',inttohex(leton(m.readdword),8));
  Writeln('ExWindows Styles                              :',inttohex(leton(m.readdword),8));
  Writeln('Unknown, often -1 or 0                        :',leton(m.readdword));
  FetchString;
  Write  ('Framename                                     :',cnt);
  if (cnt>0) then
      write('    = ',s);
  Writeln;
  FetchString;
  Writeln('Windowname                                    :',cnt);
  if (cnt>0) then
      writeln('    = ',s);
  Writeln('Number of Information Types                   :',leton(m.readdword));
  Writeln('Unknown. Often 1. Also 0, 3.                  :',leton(m.readdword));
  cnt2:=m.ReadDWordLE;
  Writeln('Number of files in the [MERGE FILES] list     :',cnt2);
  dx:=leton(m.readdword);
  Writeln('Unknown. Often 0.                             :',dx,' =$',inttohex(dx,8),'(Non-zero mostly in files with some files in the merge files list)');
  if cnt2>0 then
    for i:=0 to cnt2-1 do
      begin
        fetchstring;
        Writeln(' Offset ', cnt, ' = ',s);
      end;
end;

procedure PrintIDXHDR(filespec:TStringDynArray);

var s,
    chm,
    prefixfn,
    symbolname : ansistring;
    i,cnt,cnt2: integer;
    cl : TList;
    x : PcontextItem;
    f : textfile;
    fs: TFileStream;
    r : TChmReader;
    m : TMemorystream;


begin
  symbolname:='helpid';
  chm:=filespec[0];
  prefixfn:=changefileext(chm,'');
  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;
  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  m:=r.getobject('/#IDXHDR');
  if not assigned(m) then
    begin
      writeln(stderr,'This CHM doesn''t contain a #IDXHDR internal file');
      halt(1);
    end;
  m.position:=0;
  Writeln(' --- #IDXHDR ---');
  readchunk13(m,r);
  m.free;
  r.free;
end;


procedure PrintWindows(filespec:TStringDynArray);

var s,
    chm,
    prefixfn,
    symbolname : ansistring;
    i,cnt,cnt2: integer;
    x : PcontextItem;
    f : textfile;
    fs: TFileStream;
    r : TChmReader;
    m : TMemorystream;

function fetchstring:string;

var xx : longint;
begin
  xx:=m.ReadDWordLE;
  if (xx>0) then
    result:=r.readstringsentry(xx)+ ' (index value = '+inttostr(xx)+')'
  else
    result:='(0)';
end;

function printstructsize(sz:integer):string;

begin
 case sz of
       188 : result:='Compatibility 1.0';
       196 : result:='Compatibility 1.1 or later';
      else
       result:='unknown';
       end;
end;

begin
  chm:=filespec[0];
  prefixfn:=changefileext(chm,'');
  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;
  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  m:=r.getobject('/#WINDOWS');
  if not assigned(m) then
    begin
      writeln(stderr,'This CHM doesn''t contain a #WINDOWS internal file. Odd.');
      halt(1);
    end;
  m.position:=0;
  cnt:=m.ReadDWordLE;
  Writeln('Entries in #Windows                         : ',Cnt);
  cnt2:=m.ReadDWordLE;
  Writeln('Structure size                              : ',cnt2, ' = ',printstructsize(Cnt2));
  writeln;
  i:=0;
  while (i<cnt) do
    begin
      cnt2:=m.ReadDWordLE;
      Writeln('00 Structure size                            : ',cnt2, ' = ',printstructsize(Cnt2));

      Writeln('04 htmlhelp.h indicates "BOOL fUniCodeStrings: ',m.ReadDWordLE);
      Writeln('08 WindowType                                : ',fetchstring);
      cnt2:=m.ReadDWordLE;
      Write  ('0C Which window properties are valid         : ');
      if (cnt2 and $00002)>0 then Write(' "Navigation pane style"');
      if (cnt2 and $00004)>0 then Write(' "Window style flags"');
      if (cnt2 and $00008)>0 then Write(' "Window extended style flags"');
      if (cnt2 and $00010)>0 then Write(' "Initial window position"');
      if (cnt2 and $00020)>0 then Write(' "Navigation pane width"');
      if (cnt2 and $00040)>0 then Write(' "Window show state"');
      if (cnt2 and $00080)>0 then Write(' "Info types"');
      if (cnt2 and $00100)>0 then Write(' "Buttons"');
      if (cnt2 and $00200)>0 then Write(' "Navigation Pane initially closed state"');
      if (cnt2 and $00400)>0 then Write(' "Tab position"');
      if (cnt2 and $00800)>0 then Write(' "Tab order"');
      if (cnt2 and $01000)>0 then Write(' "History count"');
      if (cnt2 and $02000)>0 then Write(' "Default Pane"');
      writeln(' ( = ',inttohex(cnt2,8),')');
      Writeln('10 A bit field of navigation pane styles     : ',inttohex(m.readdwordLE,8));
      Writeln('14 Title Bar Text                            : ',fetchstring);
      Writeln('18 Style Flags                               : ',inttohex(m.readdwordLE,8));
      Writeln('1C Extended Style Flags                      : ',inttohex(m.readdwordLE,8));
      Writeln('20 Initial position (left,top,right,bottom   : ',m.readdwordLE,' ' ,m.readdwordLE,' ',m.readdwordLE,' ',m.readdwordLE);
      Writeln('30 Window ShowState                          : ',inttohex(m.readdwordLE,8));
      Writeln('34 HWND hwndHelp; OUT: window handle"        : ',inttohex(m.readdwordLE,8));
      Writeln('38 HWND hwndCaller; OUT: who called window"  : ',inttohex(m.readdwordLE,8));
      Writeln('3C HH_INFOTYPE* paInfoTypes                  : ',inttohex(m.readdwordLE,8));
      Writeln('40 HWND hwndToolBar;                         : ',inttohex(m.readdwordLE,8));
      Writeln('44 HWND hwndNavigation;                      : ',inttohex(m.readdwordLE,8));
      Writeln('48 HWND hwndHTML;                            : ',inttohex(m.readdwordLE,8));
      Writeln('4C Width of the navigation pane in pixels    : ',inttohex(m.readdwordLE,8));
      Writeln('50 Topic panel coordinates left,top,right,bottom : ',m.readdwordLE,' ' ,m.readdwordLE,' ',m.readdwordLE,' ',m.readdwordLE);
      Writeln('60 TOC File                                  : ',fetchstring);
      Writeln('64 Index File                                : ',fetchstring);
      Writeln('68 Default File                              : ',fetchstring);
      Writeln('6C File when Home button is pressed          : ',fetchstring);
      inc(i);

    end;

  m.free;
  r.free;
end;

procedure PrintTopics(filespec:TStringDynArray);
var s,
    chm,
    prefixfn,
    symbolname : ansistring;
    i,cnt,cnt2: integer;
    cl : TList;
    x : PcontextItem;
    f : textfile;
    fs: TFileStream;
    r : TChmReader;
    m : TMemorystream;
    chunktype,
    chunksize : Word;

    entries : integer;
begin
  chm:=filespec[0];
  prefixfn:=changefileext(chm,'');
  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;
  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  m:=r.getobject('/#TOPICS');
  if not assigned(m) then
    begin
      writeln(stderr,'This CHM doesn''t contain a #SYSTEM internal file. Odd.');
      halt(1);
    end;
  m.position:=0;
  entries:=m.size div 16;
  if entries>0 then
    for i:=0 to entries-1 do
      begin
        writeln('#TOPICS entry : ',i);
        cnt:=m.ReadDWordLE;
        writeln(' TOCIDX index:',cnt,5);
        write  (' Tag name    :');
        cnt2:=m.ReadDWordLE;
        if cnt2=-1 then
          writeln(cnt2)
        else
         begin
           s:=r.ReadStringsEntry(cnt2);
           writeln(s,'(',cnt2,')');
         end;
        write  (' Tag value   :');
        cnt2:=m.ReadDWordLE;
        if cnt2=-1 then
          writeln(cnt2)
        else
         begin
           s:=r.ReadUrlStr(cnt2);
           writeln(s,'(',cnt2,')');
         end;
        cnt2:=m.ReadWordLE;
        writeln(' contents val:',cnt2, '(2=not in contents, 6 in contents, 0/4 unknown)');
        cnt2:=m.ReadWordLE;
        writeln(' unknown val :',cnt2, '(0,2,4,8,10,12,16,32)');
      end;
  m.free;
  r.free;
end;

procedure PrintSystem(filespec:TStringDynArray);

var s,
    chm,
    prefixfn,
    symbolname : ansistring;
    i,cnt,cnt2: integer;
    cl : TList;
    x : PcontextItem;
    f : textfile;
    fs: TFileStream;
    r : TChmReader;
    m : TMemorystream;
    chunktype,
    chunksize : Word;

procedure fetchstring;

begin
  cnt:=m.ReadDWordLE;
  s:='';
  if (cnt>0) then
   s:=r.readstringsentry(cnt);
end;


function printnulterminated(sz:word):string;
begin
 setlength(result,sz);
 if sz>0 then
   begin
     m.read(result[1],sz);
   end;
end;

procedure printentry4(m:TMemoryStream;chsz:dword);
var q : QWord;
    ts : TFileTime;
begin
  writeln('(4)');
  if chsz<32 then
    begin
      Writeln('   is too small', chsz, ' bytes instead of 32');
      m.position:=m.position+chsz;
      exit;
    end;
  writeln(' LCID from HHP file                : ',m.readdwordLE );
  writeln(' One if DBCS in use                : ',m.readdwordLE );
  writeln(' one if fullttext search is on     : ',m.readdwordLE );
  writeln(' Non zero if there are KLinks      : ',m.readdwordLE );
  writeln(' Non zero if there are ALinks      : ',m.readdwordLE );
  ts.dwlowdatetime:=m.readdwordLE;
  ts.dwhighdatetime:=m.readdwordLE;
  writeln(' Timestamp                         : ',ts.dwhighdatetime,':', ts.dwlowdatetime, ' = $',inttohex(ts.dwhighdatetime,8),': $', inttohex(ts.dwlowdatetime,8));
  writeln(' 0/1 except in dsmsdn.chi has 1    : ',m.readdwordLE );
  writeln(' 0 (unknown)                       : ',m.readdwordLE );
end;

procedure printentry8(m:TMemoryStream;chsz:dword);
var q : QWord;
    ts : TFileTime;
begin
  writeln('(8)');
  if chsz<16 then
    begin
      Writeln('   is too small', chsz, ' bytes instead of 16');
      m.position:=m.position+chsz;
      exit;
    end;
  writeln(' 0 (or 4 in some)                  : ',m.readdwordLE );
  fetchstring;
  writeln(' Abbreviation                      : ',cnt,' = ',s);
  writeln(' 3 or 5 depending on 1st field     : ',m.readdwordLE );
  fetchstring;
  writeln(' Abbreviation explanation          : ',cnt,' = ',s);
  if chsz>16 then
    writeln('   x size is larger than 16');
  m.position:=m.position+chsz-16;
end;
var dx : dword;

begin
  symbolname:='helpid';
  chm:=filespec[0];
  prefixfn:=changefileext(chm,'');
  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;
  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  m:=r.getobject('/#SYSTEM');
  if not assigned(m) then
    begin
      writeln(stderr,'This CHM doesn''t contain a #SYSTEM internal file. Odd.');
      halt(1);
    end;
  m.position:=0;
  cnt:=m.ReadDWordLE;
  case cnt of
   2 : s:='Compatibility 1.0';
   3 : s:='Compatibility 1.1 or later';
  else
   s:='unknown';
   end;

  Writeln(' --- #SYSTEM---');

  while (m.size-m.position)>=8 do
    begin
      chunktype := m.readwordle;
      Chunksize := m.readwordle;
      if (m.size-m.position)>=chunksize then
        begin
          case chunktype of
            0 : Writeln('(0)  Contents file from [options]  :',printnulterminated(chunksize));
            1 : Writeln('(1)  Index file from [options]     :',printnulterminated(chunksize));
            2 : Writeln('(2)  Default topic from [options]  :',printnulterminated(chunksize));
            3 : Writeln('(3)  Title from [options]          :',printnulterminated(chunksize));
            4 : printentry4(m,chunksize);
            5 : Writeln('(5)  Default Window from [options] :',printnulterminated(chunksize));
            6 : Writeln('(6)  Compiled file from [options]  :',printnulterminated(chunksize));
            7 : Writeln('(7)  DWord when Binary Index is on :',m.readdwordle, '(= entry in #urltbl has same first dword');
            8 : printentry8(m,chunksize);
            9 : Writeln('(9)  CHM compiler version          :',printnulterminated(chunksize));
            10: begin
                  dx:=m.readdwordle;
                  writeln('(10) Timestamp (32-bit?)           :',dx,' , = $',inttohex(dx,8));
                  m.position:=m.position+chunksize-4;
                end;
            11: Writeln('(11)  DWord when Binary TOC is on   :',m.readdwordle, '(= entry in #urltbl has same first dword');
            12: begin
                  writeln('(12) Number of Information files   :',m.readdwordle);
                  m.position:=m.position+chunksize-4;
                end;
            13: begin
                  cnt:=m.position;
                  Writeln('(13)');
                  readchunk13(m,r);
                  m.position:=chunksize+cnt;
                end;
            14: begin
                  writeln('(14) MS Office related windowing constants ', chunksize,' bytes');
                  m.position:=m.position+chunksize;
                end;
            15: Writeln('(15) Information type checksum     :',m.readdwordle,' (Unknown algorithm & data source)');
            16: Writeln('(16) Default Font from [options]   :',printnulterminated(chunksize));
          else
            begin
              writeln('Not (yet) handled chunk, type ',chunktype,' of size ',chunksize);
              m.position:=m.position+chunksize;
            end;

          end;
        end;
    end;

  m.free;
  r.free;
end;

const
   siteext : array[TSiteMapType] of string = ('.hhc','.hhk');

procedure extracttocindex(filespec:TStringDynArray;sttype:TSiteMapType);
var s,
    chm,
    extractfn     : string;
    i,cnt: integer;
    cl : TList;
    f : textfile;
    fs: TFileStream;
    r : TChmReader;
    x : TCHMSitemap;
begin
  chm:=filespec[0];
  extractfn:=changefileext(chm,siteext[sttype]);
  if length(filespec)>1 then
    extractfn:=filespec[1];

  if not Fileexists(chm) then
    begin
      writeln(stderr,' Can''t find file ',chm);
      halt(1);
    end;
  fs:=TFileStream.create(chm,fmOpenRead);
  r:=TCHMReader.create(fs,true);
  case sttype of
   stindex: x:=r.GetIndexSitemap(false);
   sttoc : x:=r.gettocsitemap(false);
  end;
  if assigned(x) then
   begin
     x.savetofile( extractfn);
     x.free;
   end;
  r.free;
end;

procedure buildarglist(var params: TStringDynArray;var cmd :TCmdEnum);

var s           : ansistring;
    j,k         : Integer;
begin
  s:=uppercase(paramstr(optind));
  cmd:=Low(TCMDEnum);
  While (cmd<>high(TCmdEnum)) and (s<>CmdNames[cmd]) do
    inc(cmd);
  if cmd=CmdNone then
    begin
      writeln(stderr,' Using cmdls without command is deprecated, this may be removed in a future version');
      writeln(stderr,' Please consider using the "list" command');
      cmd:=CmdList;      // no cmd found -> list  In the future we can also do a name check here for the default (symlink on unix)
      k:=optind;
    end
  else
    begin
      k:=optind+1;
    end;
  setlength(params,paramcount-k+1);
  for j:=k to paramcount do
    params[j-k]:=paramstr(j);
end;

Var
  LocalParams : TStringDynArray;
  c:   char;
  i,
  Params,
  OptionIndex : Longint;
  cmd         : TCmdEnum;
  section     : Integer = -1;

// Start of program
begin
  InitOptions;
  Writeln(stderr,'chmls, a CHM utility. (c) 2010 Free Pascal core.');
  Writeln(Stderr);
  repeat
    c:=getlongopts('hnp',@theopts[1],optionindex);
    case c of
      #0 : begin
             case optionindex-1 of
               0 : begin;
                     Usage;
                     Halt;
                   end;
               1 : name_only:=true;
               2 : donotpage:=true;
               3 : donotshowoffset:=true;

                end;
           end;
      'p'     : donotpage:=true;
      'n'     : name_only:=true;
      '?','h' :
            begin
              writeln('unknown option',optopt);
              usage;
              halt;
            end;
   end; { case }
 until c=endofoptions;

 params:=Paramcount-optind+1;

 if params>0 then
  begin
    BuildArgList(localparams,cmd);
    case cmd of
      cmdlist : begin
                  case length(localparams) of
                    1 : ListChm(localparams[0],Section);
                    2 : begin
                          if not TryStrToInt(localparams[1],section) then
                            begin
                              writeln(stderr,' Invalid value for section ',localparams[2]);
                              usage;
                              halt(1);
                            end;
                          ListChm(localparams[0],Section);
                        end;
                  else
                    WrongNrParam(cmdnames[cmd],length(localparams));
                   end; {case}
                end; { cmdlist}
      cmdextract : begin
                     case length(localparams) of
                      2: ExtractFile(localparams[0],localparams[1],extractfilename(localparams[1]));
                      3: ExtractFile(localparams[0],localparams[1],localparams[2]);
                     else
                       WrongNrParam(cmdnames[cmd],length(localparams));
                     end;
                   end;
      cmdextractall: begin
                      if length(localparams)=1 then //extract into current directory
                        ExtractFileAll(localparams[0],GetCurrentDir)
                      else if length(localparams)=2 then //extract into specified dir
                        ExtractFileall(localparams[0],localparams[1])
                      else
                        WrongNrParam(cmdnames[cmd],length(localparams));
                     end;

      cmdunblock   : begin
                      if length(localparams)>0 then
                        Unblockchms(localparams)
                      else
                        WrongNrParam(cmdnames[cmd],length(localparams));
                     end;
      cmdextractalias: begin
                        if length(localparams)>0 then
                          extractalias(localparams)
                        else
                          WrongNrParam(cmdnames[cmd],length(localparams));
                       end;
      cmdextracttoc : begin
                        if length(localparams)>0 then
                          extracttocindex(localparams,sttoc)
                        else
                          WrongNrParam(cmdnames[cmd],length(localparams));
                       end;
      cmdextractindex: begin
                        if length(localparams)>0 then
                          extracttocindex(localparams,stindex)
	                        else
                          WrongNrParam(cmdnames[cmd],length(localparams));
                       end;

      cmdprintidxhdr: begin
                        if length(localparams)=1 then
                          printidxhdr(localparams)
	                else
                          WrongNrParam(cmdnames[cmd],length(localparams));
                       end;
      cmdprintsystem   : begin
                          if length(localparams)=1 then
                            printsystem(localparams)
                          else
                            WrongNrParam(cmdnames[cmd],length(localparams));
                         end;
      cmdprintwindows  : begin
                          if length(localparams)=1 then
                            printwindows(localparams)
                          else
                            WrongNrParam(cmdnames[cmd],length(localparams));
                         end;
      cmdprinttopics   : begin
                          if length(localparams)=1 then
                            printtopics(localparams)
                          else
                            WrongNrParam(cmdnames[cmd],length(localparams));
                         end;
      end; {case cmd of}
  end
 else
   begin
     Usage;
     halt;
   end;

end.

