{
    Copyright (c) 1998-2002 by Daniel Mantione
    Portions Copyright (c) 1998-2002 Eberhard Mattes

    Unit to write out import libraries and def files for OS/2

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
{
   A lot of code in this unit has been ported from C to Pascal from the
   emximp utility, part of the EMX development system. Emximp is copyrighted
   by Eberhard Mattes. Note: Eberhard doesn't know much about the Pascal
   port, please send questions to Tomas Hajny <hajny@freepascal.org> or
   Daniel Mantione <daniel@freepascal.org>.
}
unit t_os2;

{$i fpcdefs.inc}

interface


implementation

  uses
     SysUtils,
     cutils,cfileutl,cclasses,
     globtype,systems,symconst,symdef,
     globals,verbose,fmodule,script,
     import,link,i_os2,ogbase;

  type
    timportlibos2=class(timportlib)
      procedure generatelib;override;
    end;

    tlinkeros2=class(texternallinker)
    private
       Function  WriteResponseFile(isdll:boolean) : Boolean;
    public
       constructor Create;override;
       procedure SetDefaultInfo;override;
       function  MakeExecutable:boolean;override;
    end;


const   profile_flag:boolean=false;

const   n_ext   = 1;
        n_abs   = 2;
        n_text  = 4;
        n_data  = 6;
        n_bss   = 8;
        n_imp1  = $68;
        n_imp2  = $6a;

type    reloc=packed record     {This is the layout of a relocation table
                                 entry.}
            address:longint;    {Fixup location}
            remaining:longint;
            {Meaning of bits for remaining:
             0..23:              Symbol number or segment
             24:                 Self-relative fixup if non-zero
             25..26:             Fixup size (0: 1 byte, 1: 2, 2: 4 bytes)
             27:                 Reference to symbol or segment
             28..31              Not used}
        end;

        nlist=packed record     {This is the layout of a symbol table entry.}
            strofs:longint;     {Offset in string table}
            typ:byte;           {Type of the symbol}
            other:byte;         {Other information}
            desc:word;          {More information}
            value:longint;      {Value (address)}
        end;

        a_out_header=packed record
            magic:word;         {Magic word, must be $0107}
            machtype:byte;      {Machine type}
            flags:byte;         {Flags}
            text_size:longint;  {Length of text, in bytes}
            data_size:longint;  {Length of initialized data, in bytes}
            bss_size:longint;   {Length of uninitialized data, in bytes}
            sym_size:longint;   {Length of symbol table, in bytes}
            entry:longint;      {Start address (entry point)}
            trsize:longint;     {Length of relocation info for text, bytes}
            drsize:longint;     {Length of relocation info for data, bytes}
        end;

        ar_hdr=packed record
            ar_name:array[0..15] of char;
            ar_date:array[0..11] of char;
            ar_uid:array[0..5] of char;
            ar_gid:array[0..5] of char;
            ar_mode:array[0..7] of char;
            ar_size:array[0..9] of char;
            ar_fmag:array[0..1] of char;
        end;

var aout_str_size:longint;
    aout_str_tab:array[0..2047] of char;
    aout_sym_count:longint;
    aout_sym_tab:array[0..5] of nlist;

    aout_text:array[0..63] of byte;
    aout_text_size:longint;

    aout_treloc_tab:array[0..1] of reloc;
    aout_treloc_count:longint;

    aout_size:longint;
    seq_no:longint;

    ar_member_size:longint;

    out_file:file;

procedure PackTime (var T: TSystemTime; var P: longint);

var zs:longint;

begin
    p:=-1980;
    p:=p+t.year and 127;
    p:=p shl 4;
    p:=p+t.month;
    p:=p shl 5;
    p:=p+t.day;
    p:=p shl 16;
    zs:=t.hour;
    zs:=zs shl 6;
    zs:=zs+t.minute;
    zs:=zs shl 5;
    zs:=zs+t.second div 2;
    p:=p+(zs and $ffff);
end;


procedure write_ar(const name:string;size:longint);

var ar:ar_hdr;
    time:TSystemTime;
    numtime:longint;
    tmp:string[19];


begin
    ar_member_size:=size;
    fillchar(ar.ar_name,sizeof(ar.ar_name),' ');
    move(name[1],ar.ar_name,length(name));
    GetLocalTime(time);
    packtime(time,numtime);
    str(numtime,tmp);
    fillchar(ar.ar_date,sizeof(ar.ar_date),' ');
    move(tmp[1],ar.ar_date,length(tmp));
    ar.ar_uid:='0     ';
    ar.ar_gid:='0     ';
    ar.ar_mode:='100666'#0#0;
    str(size,tmp);
    fillchar(ar.ar_size,sizeof(ar.ar_size),' ');
    move(tmp[1],ar.ar_size,length(tmp));
    ar.ar_fmag:='`'#10;
    blockwrite(out_file,ar,sizeof(ar));
end;

procedure finish_ar;

var a:byte;

begin
    a:=0;
    if odd(ar_member_size) then
        blockwrite(out_file,a,1);
end;

procedure aout_init;

begin
  aout_str_size:=sizeof(longint);
  aout_sym_count:=0;
  aout_text_size:=0;
  aout_treloc_count:=0;
end;

function aout_sym(const name:string;typ,other:byte;desc:word;
                  value:longint):longint;

begin
    if aout_str_size+length(name)+1>sizeof(aout_str_tab) then
        internalerror(200504245);
    if aout_sym_count>=sizeof(aout_sym_tab) div sizeof(aout_sym_tab[0]) then
        internalerror(200504246);
    aout_sym_tab[aout_sym_count].strofs:=aout_str_size;
    aout_sym_tab[aout_sym_count].typ:=typ;
    aout_sym_tab[aout_sym_count].other:=other;
    aout_sym_tab[aout_sym_count].desc:=desc;
    aout_sym_tab[aout_sym_count].value:=value;
    strPcopy(@aout_str_tab[aout_str_size],name);
    aout_str_size:=aout_str_size+length(name)+1;
    aout_sym:=aout_sym_count;
    inc(aout_sym_count);
end;

procedure aout_text_byte(b:byte);

begin
    if aout_text_size>=sizeof(aout_text) then
        internalerror(200504247);
    aout_text[aout_text_size]:=b;
    inc(aout_text_size);
end;

procedure aout_text_dword(d:longint);

type li_ar=array[0..3] of byte;

begin
    aout_text_byte(li_ar(d)[0]);
    aout_text_byte(li_ar(d)[1]);
    aout_text_byte(li_ar(d)[2]);
    aout_text_byte(li_ar(d)[3]);
end;

procedure aout_treloc(address,symbolnum,pcrel,len,ext:longint);

begin
    if aout_treloc_count>=sizeof(aout_treloc_tab) div sizeof(reloc) then
        internalerror(200504248);
    aout_treloc_tab[aout_treloc_count].address:=address;
    aout_treloc_tab[aout_treloc_count].remaining:=symbolnum+pcrel shl 24+
     len shl 25+ext shl 27;
    inc(aout_treloc_count);
end;

procedure aout_finish;

begin
    while (aout_text_size and 3)<>0 do
        aout_text_byte ($90);
    aout_size:=sizeof(a_out_header)+aout_text_size+aout_treloc_count*
     sizeof(reloc)+aout_sym_count*sizeof(aout_sym_tab[0])+aout_str_size;
end;

procedure aout_write;

var ao:a_out_header;

begin
    ao.magic:=$0107;
    ao.machtype:=0;
    ao.flags:=0;
    ao.text_size:=aout_text_size;
    ao.data_size:=0;
    ao.bss_size:=0;
    ao.sym_size:=aout_sym_count*sizeof(aout_sym_tab[0]);
    ao.entry:=0;
    ao.trsize:=aout_treloc_count*sizeof(reloc);
    ao.drsize:=0;
    blockwrite(out_file,ao,sizeof(ao));
    blockwrite(out_file,aout_text,aout_text_size);
    blockwrite(out_file,aout_treloc_tab,sizeof(reloc)*aout_treloc_count);
    blockwrite(out_file,aout_sym_tab,sizeof(aout_sym_tab[0])*aout_sym_count);
    plongint(@aout_str_tab)^:=aout_str_size;
    blockwrite(out_file,aout_str_tab,aout_str_size);
end;


procedure AddImport(const module:string;index:longint;const name,mangledname:string);
{mangledname= Assembler label of the function to import.
 module     = Name of DLL to import from.
 index      = Index of function in DLL. Use 0 to import by name.
 name       = Name of function in DLL. Ignored when index=0;}
(*
<<<<<<< HEAD
<<<<<<< HEAD
var tmp1,tmp2,tmp3:string;
*)
=======
>>>>>>> graemeg/fixes_2_2
var tmp1,tmp2,tmp3:string;
*)
=======
var tmp1,tmp2,tmp3:string;
*)
>>>>>>> origin/fixes_2_2
var tmp1,tmp3:string;
    sym_mcount,sym_import:longint;
    fixup_mcount,fixup_import:longint;
begin
    aout_init;
<<<<<<< HEAD
<<<<<<< HEAD
    tmp2:=mangledname;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
(*
    tmp2:=func;
    if profile_flag and not (copy(func,1,4)='_16_') then
*)
<<<<<<< HEAD
<<<<<<< HEAD
    if profile_flag and not (copy(tmp2,1,4)='_16_') then
=======
    if profile_flag and not (copy(Name,1,4)='_16_') then
>>>>>>> graemeg/fixes_2_2
=======
    if profile_flag and not (copy(Name,1,4)='_16_') then
>>>>>>> origin/fixes_2_2
        begin
            {sym_entry:=aout_sym(func,n_text+n_ext,0,0,aout_text_size);}
            sym_mcount:=aout_sym('__mcount',n_ext,0,0,0);
            {Use, say, "_$U_DosRead" for "DosRead" to import the
             non-profiled function.}
(*
            tmp2:='__$U_'+func;
            sym_import:=aout_sym(tmp2,n_ext,0,0,0);
*)
<<<<<<< HEAD
<<<<<<< HEAD
            sym_import:=aout_sym(tmp2,n_ext,0,0,0);
=======
            sym_import:=aout_sym(name,n_ext,0,0,0);
>>>>>>> graemeg/fixes_2_2
=======
            sym_import:=aout_sym(name,n_ext,0,0,0);
>>>>>>> origin/fixes_2_2
            aout_text_byte($55);    {push ebp}
            aout_text_byte($89);    {mov ebp, esp}
            aout_text_byte($e5);
            aout_text_byte($e8);    {call _mcount}
            fixup_mcount:=aout_text_size;
            aout_text_dword(0-(aout_text_size+4));
            aout_text_byte($5d);    {pop ebp}
            aout_text_byte($e9);    {jmp _$U_DosRead}
            fixup_import:=aout_text_size;
            aout_text_dword(0-(aout_text_size+4));

            aout_treloc(fixup_mcount,sym_mcount,1,2,1);
            aout_treloc (fixup_import, sym_import,1,2,1);
        end;
    str(seq_no,tmp1);
    tmp1:='IMPORT#'+tmp1;
(*
    if name='' then
*)
    if index<>0 then
        begin
            str(index,tmp3);
<<<<<<< HEAD
<<<<<<< HEAD
            tmp3:=Name+'='+module+'.'+tmp3;
        end
    else
(*        tmp3:=Name+'='+module+'.'+name;
*)
        tmp3 := MangledName + '=' + module + '.' + target_info.Cprefix + name;

=======
=======
>>>>>>> origin/fixes_2_2
(*
            tmp3:=func+'='+module+'.'+tmp3;
*)
            tmp3:=Name+'='+module+'.'+tmp3;
        end
    else
        tmp3:=Name+'='+module+'.'+name;
(*
        tmp3:=func+'='+module+'.'+name;
>>>>>>> graemeg/fixes_2_2
    aout_sym(tmp2,n_imp1+n_ext,0,0,0);
*)
    aout_sym(Name,n_imp1+n_ext,0,0,0);
    aout_sym(tmp3,n_imp2+n_ext,0,0,0);
    aout_finish;
    write_ar(tmp1,aout_size);
    aout_write;
    finish_ar;
    inc(seq_no);
end;

    procedure timportlibos2.generatelib;
      const
        ar_magic:array[1..8] of char='!<arch>'#10;
      var
          i,j  : longint;
          ImportLibrary : TImportLibrary;
          ImportSymbol  : TImportSymbol;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        seq_no:=1;
        current_module.linkotherstaticlibs.add(Current_Module.ImportLibFilename,link_always);
        assign(out_file,Current_Module.ImportLibFilename);
=======
=======
>>>>>>> origin/fixes_2_2
{        LibName:=FixFileName(Current_Module.ModuleName^ + Target_Info.StaticCLibExt);}
(* Hack to fix the problem without merging larger changes from trunk. *)
        LibName := FixFileName(Current_Module.ObjFileName^);
        LibName := Copy (LibName, 1, Length (LibName) - Length (Target_Info.ObjExt)) + Target_Info.StaticLibExt;
        seq_no:=1;
        current_module.linkotherstaticlibs.add(libname,link_always);
        assign(out_file,libname);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        rewrite(out_file,1);
        blockwrite(out_file,ar_magic,sizeof(ar_magic));

        for i:=0 to current_module.ImportLibraryList.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(current_module.ImportLibraryList[i]);
<<<<<<< HEAD
<<<<<<< HEAD
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                AddImport(ChangeFileExt(ExtractFileName(ImportLibrary.Name),''),
                  ImportSymbol.OrdNr,ImportSymbol.Name,ImportSymbol.MangledName);
=======
{            LibName:=FixFileName(ImportLibrary.Name + Target_Info.StaticCLibExt);}
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                AddImport(ChangeFileExt(ExtractFileName(ImportLibrary.Name),''),ImportSymbol.OrdNr,ImportSymbol.Name);
>>>>>>> graemeg/fixes_2_2
=======
{            LibName:=FixFileName(ImportLibrary.Name + Target_Info.StaticCLibExt);}
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                AddImport(ChangeFileExt(ExtractFileName(ImportLibrary.Name),''),ImportSymbol.OrdNr,ImportSymbol.Name);
>>>>>>> origin/fixes_2_2
              end;
         end;
         close(out_file);
      end;


{****************************************************************************
                               TLinkeros2
****************************************************************************}

Constructor TLinkeros2.Create;
begin
  Inherited Create;
  { allow duplicated libs (PM) }
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkeros2.SetDefaultInfo;
begin
  with Info do
   begin
     ExeCmd[1]:='ld $OPT -o $OUT @$RES';
     ExeCmd[2]:='emxbind -b $STRIP $MAP $APPTYPE $RSRC -k$STACKKB -h1 -q -o $EXE $OUT -ai -s8';
     if Source_Info.Script = script_dos then
      ExeCmd[3]:='del $OUT';
   end;
end;


Function TLinkeros2.WriteResponseFile(isdll:boolean) : Boolean;
Var
  linkres  : TLinkRes;
  i        : longint;
  HPath    : TCmdStrListItem;
  s        : string;
begin
  WriteResponseFile:=False;

  { Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
     LinkRes.Add('-L'+HPath.Str);
     HPath:=TCmdStrListItem(HPath.Next);
   end;
  HPath:=TCmdStrListItem(LibrarySearchPath.First);
  while assigned(HPath) do
   begin
     LinkRes.Add('-L'+HPath.Str);
     HPath:=TCmdStrListItem(HPath.Next);
   end;

  { add objectfiles, start with prt0 always }
  LinkRes.AddFileName(FindObjectFile('prt0','',false));
  while not ObjectFiles.Empty do
   begin
     s:=ObjectFiles.GetFirst;
     if s<>'' then
      LinkRes.AddFileName(s);
   end;

  { Write staticlibraries }
  { No group !! This will not work correctly PM }
  While not StaticLibFiles.Empty do
   begin
     S:=StaticLibFiles.GetFirst;
     LinkRes.AddFileName(s)
   end;

  { Write sharedlibraries like -l<lib>, also add the needed dynamic linker
    here to be sure that it gets linked this is needed for glibc2 systems (PFV) }
  While not SharedLibFiles.Empty do
   begin
     S:=SharedLibFiles.GetFirst;
     i:=Pos(target_info.sharedlibext,S);
     if i>0 then
      Delete(S,i,255);
     LinkRes.Add('-l'+s);
   end;

{ Write and Close response }
  linkres.writetodisk;
  LinkRes.Free;

  WriteResponseFile:=True;
end;


function TLinkeros2.MakeExecutable:boolean;
var
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  i       : longint;
  AppTypeStr,
  StripStr: string[3];
  MapStr: shortstring;
  BaseFilename: TPathStr;
  RsrcStr : string;
  OutName: TPathStr;
  StackSizeKB: cardinal;
begin
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Create some replacements }
  BaseFilename := ChangeFileExt(current_module.exefilename,'');
  OutName := BaseFilename + '.out';
  if (cs_link_strip in current_settings.globalswitches) then
   StripStr := '-s '
  else
   StripStr := '';
  if (cs_link_map in current_settings.globalswitches) then
   MapStr := '-m' + BaseFileName + ' '
  else
   MapStr := '';
  if (usewindowapi) or (AppType = app_gui) then
   AppTypeStr := '-p'
  else if AppType = app_fs then
   AppTypeStr := '-f'
  else AppTypeStr := '-w';
  if not (Current_module.ResourceFiles.Empty) then
   RsrcStr := '-r ' + Current_module.ResourceFiles.GetFirst + ' '
  else
   RsrcStr := '';
(* Only one resource file supported, discard everything else
   (should be already empty anyway, though). *)
  Current_module.ResourceFiles.Clear;
{ Write used files and libraries }
  WriteResponseFile(false);

{ Call linker }
  success:=false;
  for i:=1 to 3 do
   begin
     SplitBinCmd(Info.ExeCmd[i],binstr,cmdstr);
     if binstr<>'' then
      begin
        { Is this really required? Not anymore according to my EMX docs }
        Replace(cmdstr,'$HEAPMB',tostr((1048575) shr 20));
        {Size of the stack when an EMX program runs in OS/2.}
        StackSizeKB := (StackSize + 1023) shr 10;
        (* Ensure a value which might work and is accepted by EMXBIND *)
        if StackSizeKB < 64 then
         StackSizeKB := 64
        else if StackSizeKB > (512 shl 10) then
         StackSizeKB := 512 shl 10;
        Replace(cmdstr,'$STACKKB',tostr(StackSizeKB));
        {When an EMX program runs in DOS, the heap and stack share the
         same memory pool. The heap grows upwards, the stack grows downwards.}
        Replace(cmdstr,'$DOSHEAPKB',tostr(StackSizeKB));
        Replace(cmdstr,'$STRIP ', StripStr);
        Replace(cmdstr,'$MAP ', MapStr);
        Replace(cmdstr,'$APPTYPE',AppTypeStr);
(*
   Arrgh!!! The ancient EMX LD.EXE simply dies without saying anything
   if the full pathname to link.res is quoted!!!!! @#$@@^%@#$^@#$^@^#$
   This means that name of the output directory cannot contain spaces,
   but at least it works otherwise...

        Replace(cmdstr,'$RES',maybequoted(outputexedir+Info.ResName));
*)
        Replace(cmdstr,'$RES',outputexedir+Info.ResName);
<<<<<<< HEAD
<<<<<<< HEAD
        if (Info.ExtraOptions <> '') and
                   (Info.ExtraOptions [Length (Info.ExtraOptions)] <> ' ') then
         Replace(cmdstr,'$OPT',Info.ExtraOptions)
        else
         Replace(cmdstr,'$OPT ',Info.ExtraOptions);
        Replace(cmdstr,'$RSRC ',RsrcStr);
=======
=======
>>>>>>> origin/fixes_2_2
        Replace(cmdstr,'$OPT',Info.ExtraOptions);
        Replace(cmdstr,'$RSRC',RsrcStr);
>>>>>>> graemeg/fixes_2_2
        Replace(cmdstr,'$OUT',maybequoted(OutName));
        Replace(cmdstr,'$EXE',maybequoted(current_module.exefilename));
        if i<>3 then
         success:=DoExec(FindUtil(utilsprefix+binstr),cmdstr,(i=1),false)
        else
         success:=DoExec(binstr,cmdstr,(i=1),true);
      end;
   end;

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_os2,TLinkerOS2);
  RegisterImport(system_i386_os2,TImportLibOS2);
{  RegisterRes(res_wrc_os2_info,TResourceFile);}
  RegisterTarget(system_i386_os2_info);
end.
