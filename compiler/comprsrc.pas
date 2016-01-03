{
    Copyright (c) 1998-2008 by Florian Klaempfl

    Handles the resource files handling

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
unit comprsrc;

{$i fpcdefs.inc}

interface

  uses
<<<<<<< HEAD
<<<<<<< HEAD
    Systems, cstreams, Script;
=======
    Systems, cstreams;
>>>>>>> graemeg/fixes_2_2
=======
    Systems, cstreams;
>>>>>>> origin/fixes_2_2

type
   tresoutput = (roRES, roOBJ);

   tresourcefile = class(TAbstractResourceFile)
   private
      fname : ansistring;
   protected
      function SetupCompilerArguments(output: tresoutput; const OutName :
      ansistring; respath: ansistring; out ObjUsed : boolean) : ansistring; virtual;
   public
      constructor Create(const fn : ansistring);override;
<<<<<<< HEAD
<<<<<<< HEAD
      function Compile(output: tresoutput; const OutName: ansistring) : boolean; virtual;
      procedure PostProcessResourcefile(const s : ansistring);virtual;
      function IsCompiled(const fn : ansistring) : boolean;virtual;
      procedure Collect(const fn : ansistring);virtual;
      procedure EndCollect; virtual;
=======
=======
>>>>>>> origin/fixes_2_2
      procedure Compile(output: tresoutput; const OutName: ansistring);virtual;
      procedure PostProcessResourcefile(const s : ansistring);virtual;
      function IsCompiled(const fn : ansistring) : boolean;virtual;
      procedure Collect(const fn : ansistring);virtual;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   end;
   
   TWinLikeResourceFile = class(tresourcefile)
   private
<<<<<<< HEAD
<<<<<<< HEAD
      fResScript : TScript;
      fScriptName : ansistring;
      fCollectCount : integer;
   protected
      function SetupCompilerArguments(output: tresoutput; const OutName :
        ansistring; respath: ansistring; out ObjUsed : boolean) : ansistring; override;
   public
      constructor Create(const fn : ansistring);override;
      destructor Destroy; override;
      function Compile(output: tresoutput; const OutName: ansistring) : boolean; override;
      function IsCompiled(const fn : ansistring) : boolean;override;
      procedure Collect(const fn : ansistring);override;
      procedure EndCollect; override;
   end;

   TJVMRawResourceFile = class(TWinLikeResourceFile)
   private
   protected
   public
      function Compile(output: tresoutput; const OutName: ansistring) : boolean; override;
      function IsCompiled(const fn : ansistring) : boolean;override;
=======
=======
>>>>>>> origin/fixes_2_2
      FOut: TCFileStream;
      FLastIconID: longint;
      FLastCursorID: longint;
   public
      function IsCompiled(const fn : ansistring) : boolean;override;
      procedure Collect(const fn : ansistring);override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   end;


procedure CompileResourceFiles;
procedure CollectResourceFiles;

Var
  ResCompiler : String;
  RCCompiler  : String;

implementation

uses
  SysUtils,
<<<<<<< HEAD
<<<<<<< HEAD
  cutils,cfileutl,cclasses,
  Globtype,Globals,Verbose,Fmodule, comphook,cpuinfo;
=======
=======
>>>>>>> origin/fixes_2_2
  cutils,cfileutils,cclasses,
  Globtype,Globals,Verbose,Fmodule,
  Script;
  
const
  GlobalResName = 'fpc-res';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{****************************************************************************
                              TRESOURCEFILE
****************************************************************************}

constructor tresourcefile.create(const fn : ansistring);
begin
  fname:=fn;
end;


procedure tresourcefile.PostProcessResourcefile(const s : ansistring);
begin
end;


function tresourcefile.IsCompiled(const fn: ansistring): boolean;
begin
  Result:=CompareText(ExtractFileExt(fn), target_info.resobjext) = 0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
procedure tresourcefile.Collect(const fn: ansistring);
begin
  if fn='' then
    exit;
  fname:=fn;
  Compile(roOBJ, ChangeFileExt(fn, target_info.resobjext));
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure tresourcefile.EndCollect;
begin

end;

function tresourcefile.SetupCompilerArguments(output: tresoutput; const OutName
  : ansistring; respath: ansistring; out ObjUsed : boolean) : ansistring;
var
  s : TCmdStr;
begin
  if output=roRES then
    begin
      s:=target_res.rccmd;
      Replace(s,'$RES',maybequoted(OutName));
      Replace(s,'$RC',maybequoted(fname));
      ObjUsed:=False;
    end
  else
    begin
      s:=target_res.rescmd;
      ObjUsed:=(pos('$OBJ',s)>0);
      Replace(s,'$OBJ',maybequoted(OutName));
      Replace(s,'$RES',maybequoted(fname));
    end;
  Result:=s;
end;

function tresourcefile.compile(output: tresoutput; const OutName: ansistring)
  : boolean;

  Function SelectBin(Const Bin1,Bin2 : String) : String;
=======
=======
>>>>>>> origin/fixes_2_2

procedure tresourcefile.compile(output: tresoutput; const OutName: ansistring);

  Function SelectBin(Const Bin1,Bin2 : String) : String;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  begin
    If (Bin1<>'') then
      SelectBin:=Bin1
    else
<<<<<<< HEAD
<<<<<<< HEAD
      SelectBin:=Bin2;
=======
=======
>>>>>>> origin/fixes_2_2
      SelectBin:=Bin2;  
  end;

  function WindresFileName(filename: TCmdStr): TCmdStr;
  // to be on the safe side, only give short file names with forward slashes to
  // windres
  var
    i: longint;
  begin
    Result := GetShortName(filename);
    for I:=1 to Length(Result) do
    if Result[I] in AllowDirectorySeparators then
      Result[i]:='/';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

var
  respath,
<<<<<<< HEAD
<<<<<<< HEAD
  s,
  bin,
  resbin   : TCmdStr;
=======
  srcfilepath,
  preprocessorbin,
  s,
  bin,
  resbin,
  fnameparam : TCmdStr;
  usewindres,
>>>>>>> graemeg/fixes_2_2
=======
  srcfilepath,
  preprocessorbin,
  s,
  bin,
  resbin,
  fnameparam : TCmdStr;
  usewindres,
>>>>>>> origin/fixes_2_2
  resfound,
  objused  : boolean;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=true;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  if output=roRES then
    Bin:=SelectBin(RCCompiler,target_res.rcbin)
  else
    Bin:=SelectBin(ResCompiler,target_res.resbin);
  if bin='' then
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    Result:=false;
    exit;
  end;
=======
    exit;
>>>>>>> graemeg/fixes_2_2
=======
    exit;
>>>>>>> origin/fixes_2_2
  resfound:=false;
  usewindres:= bin='windres';
  if utilsdirectory<>'' then
    resfound:=FindFile(utilsprefix+bin+source_info.exeext,utilsdirectory,false,resbin);
  if not resfound then
<<<<<<< HEAD
<<<<<<< HEAD
    begin
      resfound:=FindExe(utilsprefix+bin,false,resbin);
      if not resfound and (utilsprefix<>'') and ( (output=roRES) or (Pos('$ARCH', target_res.rescmd)<>0) ) then
        { Search for resource compiler without utilsprefix, if RC->RES compiler is called }
        { or RES->OBJ compiler supports different architectures. }
        resfound:=FindExe(bin,false,resbin);
    end;
=======
    resfound:=FindExe(utilsprefix+bin,false,resbin);
>>>>>>> graemeg/fixes_2_2
=======
    resfound:=FindExe(utilsprefix+bin,false,resbin);
>>>>>>> origin/fixes_2_2
  { get also the path to be searched for the windres.h }
  respath:=ExtractFilePath(resbin);
  if (not resfound) and not(cs_link_nolink in current_settings.globalswitches) then
   begin
     Message1(exec_e_res_not_found, utilsprefix+bin+source_info.exeext);
     current_settings.globalswitches:=current_settings.globalswitches+[cs_link_nolink];
     Result:=false;
   end;
<<<<<<< HEAD
  s:=SetupCompilerArguments(output,OutName,respath,objused);
=======
  srcfilepath:=ExtractFilePath(current_module.mainsource^);
  if usewindres then
    fnameparam:=WindresFileName(fname)
  else
    fnameparam:=maybequoted(fname);
  if output=roRES then
    begin
      s:=target_res.rccmd;
      Replace(s,'$RES',maybequoted(OutName));
      Replace(s,'$RC',fnameparam);
      ObjUsed:=False;
    end
  else
    begin
      s:=target_res.rescmd;
      ObjUsed:=(pos('$OBJ',s)>0);
      Replace(s,'$OBJ',maybequoted(OutName));
      Replace(s,'$RES',fnameparam);
    end;
  { windres doesn't like empty include paths }
  if respath='' then
    respath:='.';
  if usewindres then
    Replace(s,'$INC',WindresFileName(respath))
  else
    Replace(s,'$INC',maybequoted(respath));
  if (target_res.resbin='windres') then
   begin
     if (srcfilepath<>'') then
       s:=s+' --include '+WindresFileName(srcfilepath);
     { try to find a preprocessor }
     preprocessorbin := respath+'cpp'+source_info.exeext;
     if FileExists(preprocessorbin,true) then
       s:=s+' --preprocessor='+maybequoted(preprocessorbin);
   end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ Execute the command }
{ Always try to compile resources. but don't complain if cs_link_nolink }
  if resfound then
   begin
     Message1(exec_i_compilingresource,fname);
     Message2(exec_d_resbin_params,resbin,s);
     FlushOutput;
     try
       if RequotedExecuteProcess(resbin,s) <> 0 then
       begin
         if not (cs_link_nolink in current_settings.globalswitches) then
           Message(exec_e_error_while_compiling_resources);
         current_settings.globalswitches:=current_settings.globalswitches+[cs_link_nolink];
         Result:=false;
       end;
     except
       on E:EOSError do
       begin
         if not (cs_link_nolink in current_settings.globalswitches) then
           Message1(exec_e_cant_call_resource_compiler, resbin);
         current_settings.globalswitches:=current_settings.globalswitches+[cs_link_nolink];
         Result:=false;
       end
     end;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  { Update asmres when externmode is set and resource compiling failed }
  if (not Result) and (cs_link_nolink in current_settings.globalswitches) then
    AsmRes.AddLinkCommand(resbin,s,OutName);
  if Result and (output=roOBJ) and ObjUsed then
    current_module.linkunitofiles.add(OutName,link_always);
end;

constructor TWinLikeResourceFile.Create(const fn : ansistring);
begin
  inherited Create(fn);
  fResScript:=nil;
  fCollectCount:=0;
  if (tf_use_8_3 in target_info.flags) then
    fScriptName:=ChangeFileExt(fn,'.rls')
  else
    fScriptName:=ChangeFileExt(fn,'.reslst');
end;

destructor TWinLikeResourceFile.Destroy;
begin
  if fResScript<>nil then
    fResScript.Free;
  inherited;
end;

function TWinLikeResourceFile.SetupCompilerArguments(output: tresoutput; const
  OutName : ansistring; respath : ansistring; out ObjUsed : boolean) : ansistring;
var
  srcfilepath,
  preprocessorbin,
  s : TCmdStr;
  arch,
  subarch: ansistring;

  function WindresFileName(filename: TCmdStr): TCmdStr;
  // to be on the safe side, for files that are passed to the preprocessor,
  // only give short file names with forward slashes to windres
  var
    i: longint;
  begin
    Result := GetShortName(filename);
    for I:=1 to Length(Result) do
    if Result[I] in AllowDirectorySeparators then
      Result[i]:='/';
  end;

begin
  srcfilepath:=ExtractFilePath(current_module.mainsource);
  if output=roRES then
    begin
      s:=target_res.rccmd;
      if target_res.rcbin = 'windres' then
        Replace(s,'$RC',WindresFileName(fname))
      else
        Replace(s,'$RC',maybequoted(fname));
      Replace(s,'$RES',maybequoted(OutName));
      ObjUsed:=False;
    end
  else
    begin
      s:=target_res.rescmd;
      if (res_external_file in target_res.resflags) then
        ObjUsed:=false
      else
        ObjUsed:=(pos('$OBJ',s)>0);
      Replace(s,'$OBJ',maybequoted(OutName));
      subarch:='all';
      arch:=cpu2str[target_cpu];
      if (target_info.cpu=systems.cpu_arm) then
        begin
          //Differentiate between arm and armeb
          if (target_info.endian=endian_big) then
            arch:=arch+'eb';
        end;
      if target_info.cpu=systems.cpu_powerpc64 then
        begin
          { differentiate between ppc64 and ppc64le }
          if target_info.endian=endian_little then
            arch:=arch+'le';
        end;
      Replace(s,'$ARCH',arch);
      if target_info.system=system_arm_darwin then
        subarch:=lower(cputypestr[current_settings.cputype]);
      Replace(s,'$SUBARCH',subarch);
      case target_info.endian of
        endian_little : Replace(s,'$ENDIAN','littleendian');
        endian_big : Replace(s,'$ENDIAN','bigendian');
      end;
      //call resource compiler with debug switch
      if (status.verbosity and V_Debug)<>0 then
        Replace(s,'$DBG','-v')
      else
        Replace(s,'$DBG','');
      if fCollectCount=0 then
        s:=s+' '+maybequoted(fname)
      else
        s:=s+' '+maybequoted('@'+fScriptName);
    end;
  { windres doesn't like empty include paths }
  if respath='' then
    respath:='.';
  Replace(s,'$INC',maybequoted(respath));
  if (output=roRes) and (target_res.rcbin='windres') then
  begin
    { try to find a preprocessor }
    preprocessorbin := respath+'cpp'+source_info.exeext;
    if FileExists(preprocessorbin,true) then
      s:='--preprocessor='+preprocessorbin+' '+s;
    if (srcfilepath<>'') then
      s:='--include '+WindresFileName(srcfilepath)+' '+s;
  end;
  Result:=s;
end;

function TWinLikeResourceFile.compile(output: tresoutput;
  const OutName: ansistring) : boolean;
begin
  Result:=inherited compile(output,OutName);
  //delete fpc-res.lst file if things went well
  if Result and (output=roOBJ) then
    DeleteFile(fScriptName);
end;
=======
=======
>>>>>>> origin/fixes_2_2
  if output=roOBJ then
    PostProcessResourcefile(OutName);
  { Update asmres when externmode is set }
  if cs_link_nolink in current_settings.globalswitches then
    AsmRes.AddLinkCommand(resbin,s,'');
  if (output=roOBJ) and ObjUsed then
    current_module.linkunitofiles.add(OutName,link_always);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function TWinLikeResourceFile.IsCompiled(const fn: ansistring): boolean;
const
  ResSignature : array [1..32] of byte =
  ($00,$00,$00,$00,$20,$00,$00,$00,$FF,$FF,$00,$00,$FF,$FF,$00,$00,
   $00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,$00);
<<<<<<< HEAD
<<<<<<< HEAD
  knownexts : array[1..4] of string[4] = ('.lfm', '.dfm', '.xfm', '.tlb');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  f : file;
  oldfmode : byte;
  buf: array[1..32] of byte;
  i: longint;
<<<<<<< HEAD
<<<<<<< HEAD
  ext : shortstring;
begin
  ext:=lower(ExtractFileExt(fn));
  Result:=CompareText(ext, target_info.resext) = 0;
  if not Result then
    for i:=1 to high(knownexts) do
    begin
      Result:=CompareText(ext, knownexts[i]) = 0;
      if Result then break;
    end;

=======
begin
  Result:=CompareText(ExtractFileExt(fn), target_info.resext) = 0;
>>>>>>> graemeg/fixes_2_2
=======
begin
  Result:=CompareText(ExtractFileExt(fn), target_info.resext) = 0;
>>>>>>> origin/fixes_2_2
  if Result or not FileExists(fn, False) then exit;
  oldfmode:=Filemode;
  Filemode:=0;
  assign(f,fn);
  reset(f,1);
  BlockRead(f, buf, SizeOf(buf), i);
  close(f);
  Filemode:=oldfmode;
  
  if i<>SizeOf(buf) then
    exit;

  for i:=1 to 32 do
    if buf[i]<>ResSignature[i] then
      exit;
      
  Result:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinLikeResourceFile.Collect(const fn: ansistring);
begin
  if fResScript=nil then
    fResScript:=TScript.Create(fScriptName);
  fResScript.Add(maybequoted_for_script(fn,script_fpcres));
  inc(fCollectCount);
end;

procedure TWinLikeResourceFile.EndCollect;
begin
  if fResScript<>nil then
  begin
    fResScript.WriteToDisk;
    FreeAndNil(fResScript);
    Compile(roOBJ,ChangeFileExt(fname,target_info.resobjext));
  end;
=======
=======
>>>>>>> origin/fixes_2_2

procedure TWinLikeResourceFile.Collect(const fn: ansistring);
const
  zeroes: array[1..3] of byte = (0,0,0);

type
  TResHeader = packed record
    DataSize: dword;
    HeaderSize: dword;
    ResTypeFlag: word;
    ResTypeID: word;
  end;
  
  PIconHeader = ^TIconHeader;
  TIconHeader = packed record
    Reserved: word;
    wType: word;
    wCount: word;
  end;
  
  PIconDir = ^TIconDir;
  TIconDir = packed record
    bWidth: byte;
    bHeight: byte;
    bColorCount: byte;
    bReserved: byte;
    wPlanes: word;
    wBitCount: word;
    lBytesInRes: dword;
    wNameOrdinal: word;
  end;

var
  fs: TCFileStream;
  i, sz, rsz, MaxIconID, MaxCursorID: longint;
  hdr: TResHeader;
  P: pointer;
  PData: PIconHeader;
  PDir: PIconDir;
  ResNameBuf: array[0..1] of word;
begin
  if fn='' then
    begin
      if FOut<>nil then
        begin
          FOut.Free;
          Compile(roOBJ,ChangeFileExt(fname,target_info.resobjext));
        end;
    end
  else
    try
      fs:=TCFileStream.Create(fn,fmOpenRead or fmShareDenyNone);
      if CStreamError<>0 then
        begin
          fs.Free;
          Comment(V_Error,'Can''t open resource file: '+fn);
          Include(current_settings.globalswitches, cs_link_nolink);
          exit;
        end;
      if FOut=nil then
        begin
          FOut:=TCFileStream.Create(fname,fmCreate);
          { writing res signature }
          FOut.CopyFrom(fs, 32);
        end
      else
        fs.Seek(32, soFromBeginning);
      sz:=fs.Size;
      MaxIconID := 0;
      MaxCursorID := 0;
      repeat
        fs.ReadBuffer(hdr, SizeOf(hdr));
        FOut.WriteBuffer(hdr, SizeOf(hdr));
        rsz:=hdr.HeaderSize + hdr.DataSize - SizeOf(hdr);
        if fs.Position + rsz > sz then
          begin
            Comment(V_Error,'Invalid resource file: '+fn);
            Include(current_settings.globalswitches, cs_link_nolink);
            fs.Free;
            exit;
          end;
        { Adjusting cursor and icon IDs }
        if hdr.ResTypeFlag = $FFFF then       { resource type is ordinal }
          case hdr.ResTypeID of
            1, 3:
              { cursor or icon resource }
              begin
                fs.ReadBuffer(ResNameBuf, SizeOf(ResNameBuf));
                if ResNameBuf[0] = $FFFF then   { resource name is ordinal }
                  if hdr.ResTypeID = 1 then
                    begin
                      if ResNameBuf[1] > MaxCursorID then
                        MaxCursorID:=ResNameBuf[1];
                      Inc(ResNameBuf[1], FLastCursorID);
                    end
                  else
                    begin
                      if ResNameBuf[1] > MaxIconID then
                        MaxIconID:=ResNameBuf[1];
                      Inc(ResNameBuf[1], FLastIconID);
                    end;
                FOut.WriteBuffer(ResNameBuf, SizeOf(ResNameBuf));
                Dec(rsz, SizeOf(ResNameBuf));
              end;
            12, 14:
              { cursor or icon group resource }
              begin
                GetMem(P, rsz);
                fs.ReadBuffer(P^, rsz);
                PData := PIconHeader(P + hdr.HeaderSize - sizeof(hdr));
                PDir := PIconDir(Pointer(PData) + sizeof(TIconHeader));
                for i := 0 to PData^.wCount-1 do
                  begin
                    if hdr.ResTypeID = 12 then
                      Inc(PDir^.wNameOrdinal, FLastCursorID)
                    else
                      Inc(PDir^.wNameOrdinal, FLastIconID);
                    Inc(PDir);
                  end;
                FOut.WriteBuffer(P^, rsz);
                rsz:=0;
                FreeMem(P);
              end;
          end;
        { copy rest of the resource data }
        FOut.CopyFrom(fs, rsz);
        { align resource to dword }
        i:=4 - FOut.Position mod 4;
        if i<4 then
          FOut.WriteBuffer(zeroes, i);
        { position to the next resource }
        i:=4 - fs.Position mod 4;
        if i<4 then
          fs.Seek(i, soFromCurrent);
      until fs.Position + SizeOf(hdr) >= sz;
      fs.Free;
      Inc(FLastCursorID, MaxCursorID);
      Inc(FLastIconID, MaxIconID);
    except
      on E:EOSError do begin
        Comment(V_Error,'Error processing resource file: '+fn+': '+E.Message);
        Include(current_settings.globalswitches, cs_link_nolink);
      end;
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{****************************************************************************
                              TJVMRawResourceFile
****************************************************************************}

function TJVMRawResourceFile.Compile(output: tresoutput; const OutName: ansistring): boolean;
  begin
    if output<>roOBJ then
      internalerror(2011081703);
    result:=inherited;
  end;


function TJVMRawResourceFile.IsCompiled(const fn: ansistring): boolean;
  begin
    internalerror(2011081704);
    result:=true;
  end;


function CopyResFile(inf,outf : TCmdStr) : boolean;
var
  src,dst : TCCustomFileStream;
begin
  { Copy .res file to units output dir. }
  Result:=false;
  src:=CFileStreamClass.Create(inf,fmOpenRead or fmShareDenyNone);
  if CStreamError<>0 then
    begin
      Message1(exec_e_cant_open_resource_file, src.FileName);
      Include(current_settings.globalswitches, cs_link_nolink);
      exit;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  dst:=CFileStreamClass.Create(current_module.outputpath+outf,fmCreate);
=======
  dst:=CFileStreamClass.Create(current_module.outputpath^+outf,fmCreate);
>>>>>>> graemeg/cpstrnew
=======
  dst:=CFileStreamClass.Create(current_module.outputpath^+outf,fmCreate);
>>>>>>> graemeg/cpstrnew
=======
  dst:=CFileStreamClass.Create(current_module.outputpath^+outf,fmCreate);
>>>>>>> graemeg/cpstrnew
=======
  dst:=CFileStreamClass.Create(current_module.outputpath^+outf,fmCreate);
>>>>>>> origin/cpstrnew
  if CStreamError<>0 then
    begin
      Message1(exec_e_cant_write_resource_file, dst.FileName);
      Include(current_settings.globalswitches, cs_link_nolink);
      exit;
    end;
  dst.CopyFrom(src,src.Size);
  dst.Free;
  src.Free;
  Result:=true;
end;
 
procedure CompileResourceFiles;
var
  resourcefile : tresourcefile;
  res: TCmdStrListItem;
  p,s : TCmdStr;
<<<<<<< HEAD
<<<<<<< HEAD
  outfmt : tresoutput;
begin
  { Don't do anything for systems supporting resources without using resource
    file classes (e.g. Mac OS). They process resources elsewhere. }
  if ((target_info.res<>res_none) and (target_res.resourcefileclass=nil)) or
     (res_no_compile in target_res.resflags) then
    exit;

  p:=ExtractFilePath(ExpandFileName(current_module.mainsource));
=======
=======
>>>>>>> origin/fixes_2_2
  src,dst : TCFileStream;
  outfmt : tresoutput;
begin
  { OS/2 (EMX) must be processed elsewhere (in the linking/binding stage).
    same with MacOS}
  if target_info.system in [system_i386_os2,system_i386_emx,system_powerpc_macos] then exit;

  p:=ExtractFilePath(ExpandFileName(current_module.mainsource^));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  res:=TCmdStrListItem(current_module.ResourceFiles.First);
  while res<>nil do
    begin
      if target_info.res=res_none then
        Message(scan_e_resourcefiles_not_supported);
      s:=res.FPStr;
      if not path_absolute(s) then
        s:=p+s;
<<<<<<< HEAD
<<<<<<< HEAD
      if not FileExists(s, True) then
        begin
          Message1(exec_e_cant_open_resource_file, s);
          Include(current_settings.globalswitches, cs_link_nolink);
          exit;
        end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      resourcefile:=TResourceFile(resinfos[target_info.res]^.resourcefileclass.create(s));
      if resourcefile.IsCompiled(s) then
        begin
          resourcefile.free;
<<<<<<< HEAD
<<<<<<< HEAD
          if AnsiCompareFileName(IncludeTrailingPathDelimiter(ExpandFileName(current_module.outputpath)), p) <> 0 then
            begin
              { Copy .res file to units output dir. Otherwise .res file will not be found
                when only compiled units path is available }
              res.FPStr:=ExtractFileName(res.FPStr); //store file name only in PPU.
              if not CopyResFile(s,res.FPStr) then exit;
=======
=======
>>>>>>> origin/fixes_2_2
          if AnsiCompareFileName(IncludeTrailingPathDelimiter(ExpandFileName(current_module.outputpath^)), p) <> 0 then
            begin
              { Copy .res file to units output dir }
              res.FPStr:=ExtractFileName(res.FPStr);
              src:=TCFileStream.Create(s,fmOpenRead or fmShareDenyNone);
              if CStreamError<>0 then
                begin
                  Comment(V_Error,'Can''t open resource file: '+src.FileName);
                  Include(current_settings.globalswitches, cs_link_nolink);
                  exit;
                end;
              dst:=TCFileStream.Create(current_module.outputpath^+res.FPStr,fmCreate);
              if CStreamError<>0 then
                begin
                  Comment(V_Error,'Can''t create resource file: '+dst.FileName);
                  Include(current_settings.globalswitches, cs_link_nolink);
                  exit;
                end;
              dst.CopyFrom(src,src.Size);
              dst.Free;
              src.Free;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            end;
        end
      else
        begin
          res.FPStr:=ExtractFileName(res.FPStr);
<<<<<<< HEAD
<<<<<<< HEAD
          if (target_res.rcbin='') and (RCCompiler='') then
=======
          if target_res.rcbin='' then
>>>>>>> graemeg/fixes_2_2
=======
          if target_res.rcbin='' then
>>>>>>> origin/fixes_2_2
            begin
              { if target does not have .rc to .res compiler, create obj }
              outfmt:=roOBJ;
              res.FPStr:=ChangeFileExt(res.FPStr,target_info.resobjext);
            end
          else
            begin
              outfmt:=roRES;
              res.FPStr:=ChangeFileExt(res.FPStr,target_info.resext);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          resourcefile.compile(outfmt, current_module.outputpath+res.FPStr);
=======
          resourcefile.compile(outfmt, current_module.outputpath^+res.FPStr);
>>>>>>> graemeg/fixes_2_2
=======
          resourcefile.compile(outfmt, current_module.outputpath^+res.FPStr);
>>>>>>> origin/fixes_2_2
          resourcefile.free;
        end;
      res:=TCmdStrListItem(res.Next);
    end;
end;


procedure CollectResourceFiles;
var
  resourcefile : tresourcefile;
  
  procedure ProcessModule(u : tmodule);
  var
    res : TCmdStrListItem;
    s   : TCmdStr;
  begin
    res:=TCmdStrListItem(u.ResourceFiles.First);
    while assigned(res) do
      begin
        if path_absolute(res.FPStr) then
          s:=res.FPStr
        else
          begin
<<<<<<< HEAD
<<<<<<< HEAD
            s:=u.path+res.FPStr;
            if not FileExists(s,True) then
              s:=u.outputpath+res.FPStr;
=======
            s:=u.path^+res.FPStr;
            if not FileExists(s,True) then
              s:=u.outputpath^+res.FPStr;
>>>>>>> graemeg/fixes_2_2
=======
            s:=u.path^+res.FPStr;
            if not FileExists(s,True) then
              s:=u.outputpath^+res.FPStr;
>>>>>>> origin/fixes_2_2
          end;
        resourcefile.Collect(s);
        res:=TCmdStrListItem(res.Next);
      end;
  end;
  
var
  hp : tused_unit;
  s : TCmdStr;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if (target_info.res=res_none) or ((target_res.resbin='')
    and (ResCompiler='')) then
      exit;
//  if cs_link_nolink in current_settings.globalswitches then
//    exit;
  s:=ChangeFileExt(current_module.ppufilename,target_info.resobjext);
  if (res_arch_in_file_name in target_res.resflags) then
    s:=ChangeFileExt(s,'.'+cpu2str[target_cpu]+target_info.resobjext);
=======
=======
>>>>>>> origin/fixes_2_2
  if (target_info.res=res_none) or (target_res.rcbin='') then
    exit;
  if cs_link_nolink in current_settings.globalswitches then
    exit;
  s:=main_module.outputpath^+GlobalResName+target_info.resext;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  resourcefile:=TResourceFile(resinfos[target_info.res]^.resourcefileclass.create(s));
  hp:=tused_unit(usedunits.first);
  while assigned(hp) do
    begin
      ProcessModule(hp.u);
      hp:=tused_unit(hp.next);
    end;
  ProcessModule(current_module);
  { Finish collection }
<<<<<<< HEAD
<<<<<<< HEAD
  resourcefile.EndCollect;
=======
  resourcefile.Collect('');
>>>>>>> graemeg/fixes_2_2
=======
  resourcefile.Collect('');
>>>>>>> origin/fixes_2_2
  resourcefile.free;
end;

end.
