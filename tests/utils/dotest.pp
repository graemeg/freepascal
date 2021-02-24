{
    This file is part of the Free Pascal test suite.
    Copyright (c) 1999-2002 by the Free Pascal development team.

    This program makes the compilation and
    execution of individual test sources.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$goto on}
{$H+}

program dotest;
uses
  sysutils,
  strutils,
  dos,
{$ifdef macos}
  macutils,
{$endif}
  teststr,
  testu,
  redir,
  bench,
  classes;

{$ifdef go32v2}
  {$define LIMIT83FS}
{$endif}
{$ifdef os2}
  {$define LIMIT83FS}
{$endif}
{$ifdef msdos}
  {$define LIMIT83FS}
{$endif}

type
  tcompinfo = (compver,comptarget,compcpu);
  tdelexecutable = (deBefore, deAfter);
  tdelexecutables = set of tdelexecutable;

const
  ObjExt='o';
  PPUExt='ppu';
{$ifdef UNIX}
  SrcExeExt='';
{$else UNIX}
{$ifdef MACOS}
  SrcExeExt='';
{$else MACOS}
  SrcExeExt='.exe';
{$endif MACOS}
{$endif UNIX}
  ExeExt : string = '';
  DllExt : string = '.so';
  DllPrefix: string = 'lib';
  DefaultTimeout=60;
  READ_ONLY = 0;

var
  Config : TConfig;
  CompilerLogFile,
  ExeLogFile,
  LongLogfile,
  FailLogfile,
  RTLUnitsDir,
  TestOutputDir,
  OutputDir : string;
  CompilerBin,
{ CompilerCPU and CompilerTarget are lowercased at start
  to avoid need to call lowercase again and again ... }
  CompilerCPU,
  CompilerTarget,
  CompilerVersion,
  DefaultCompilerCPU,
  DefaultCompilerTarget,
  DefaultCompilerVersion : string;
  PPFile : TStringList;
  PPFileInfo : TStringList;
  TestName : string;
  Current : longint;

const
  DoGraph : boolean = false;
  UseOSOnly : boolean = false;
  DoInteractive : boolean = false;
  DoExecute : boolean = false;
  DoKnown : boolean = false;
  DoAll : boolean = false;
  DoUsual : boolean = true;
  { TargetDir : string = ''; unused }
  BenchmarkInfo : boolean = false;
  ExtraCompilerOpts : string = '';
  DelExecutable : TDelExecutables = [];
  RemoteAddr : string = '';
  RemotePathPrefix : string = '';
  RemotePath : string = '/tmp';
  RemotePara : string = '';
  RemoteRshParas : string = '';
  RemoteShell : string = '';
  RemoteShellBase : string = '';
  RemoteShellNeedsExport : boolean = false;
  rshprog : string = 'rsh';
  rcpprog : string = 'rcp';
  rquote : string = '''';
  UseTimeout : boolean = false;
  emulatorname : string = '';
  EmulatorOpts : string = '';
  TargetCanCompileLibraries : boolean = true;
  UniqueSuffix: string = '';


const
  NoSharedLibSupportPattern='$nosharedlib';
  TargetHasNoSharedLibSupport = 'msdos,go32v2';
  NoWorkingUnicodeSupport='$nounicode';
  TargetHasNoWorkingUnicodeSupport = 'msdos';
  NoWorkingThread='$nothread';
  TargetHasNoWorkingThreadSupport = 'go32v2,msdos';

procedure TranslateConfig(var AConfig: TConfig);
begin
  AConfig.SkipTarget:=ReplaceText(AConfig.SkipTarget, NoSharedLibSupportPattern, TargetHasNoSharedLibSupport);
  AConfig.SkipTarget:=ReplaceText(AConfig.SkipTarget, NoWorkingUnicodeSupport, TargetHasNoWorkingUnicodeSupport);
  AConfig.SkipTarget:=ReplaceText(AConfig.SkipTarget, NoWorkingThread, TargetHasNoWorkingThreadSupport);
end;


function ToStr(l:longint):string;
var
  s : string;
begin
  Str(l,s);
  ToStr:=s;
end;


function ToStrZero(l:longint;nbzero : byte):string;
var
  s : string;
begin
  Str(l,s);
  while length(s)<nbzero do
    s:='0'+s;
  ToStrZero:=s;
end;


function trimspace(const s:string):string;
var
  i,j : longint;
begin
  i:=length(s);
  while (i>0) and (s[i] in [#9,' ']) do
   dec(i);
  j:=1;
  while (j<i) and (s[j] in [#9,' ']) do
   inc(j);
  trimspace:=Copy(s,j,i-j+1);
end;


function IsInList(const entry,list:string):boolean;
var
  i,istart : longint;
begin
  IsInList:=false;
  i:=0;
  while (i<length(list)) do
   begin
     { Find list item }
     istart:=i+1;
     while (i<length(list)) and
           (list[i+1]<>',') do
      inc(i);
     if Upcase(entry)=Upcase(TrimSpace(Copy(list,istart,i-istart+1))) then
      begin
        IsInList:=true;
        exit;
      end;
     { skip , }
     inc(i);
   end;
end;


procedure SetPPFileInfo;
Var
  info : searchrec;
  dt : DateTime;
begin
  FindFirst (PPFile[current],anyfile,Info);
  If DosError=0 then
    begin
      UnpackTime(info.time,dt);
      PPFileInfo.Insert(current,PPFile[current]+' '+ToStr(dt.year)+'/'+ToStrZero(dt.month,2)+'/'+
        ToStrZero(dt.day,2)+' '+ToStrZero(dt.Hour,2)+':'+ToStrZero(dt.min,2)+':'+ToStrZero(dt.sec,2));
    end
  else
    PPFileInfo.Insert(current,PPFile[current]);
  FindClose (Info);
end;


function ForceExtension(Const HStr,ext:String):String;
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
   j:=length(Hstr)+1;
  if Ext<>'' then
   begin
     if Ext[1]='.' then
       ForceExtension:=Copy(Hstr,1,j-1)+Ext
     else
       ForceExtension:=Copy(Hstr,1,j-1)+'.'+Ext
   end
  else
   ForceExtension:=Copy(Hstr,1,j-1);
end;

procedure mkdirtree(const s:string);
var
  SErr, hs : string;
  Err: longint;
begin
  if s='' then
    exit;
  if s[length(s)] in ['\','/'{$IFDEF MACOS},':'{$ENDIF}] then
    hs:=Copy(s,1,length(s)-1)
  else
    hs:=s;
  if not PathExists(hs) then
    begin
      { Try parent first }
      mkdirtree(SplitPath(hs));
      { make this dir }
      Verbose(V_Debug,'Making directory '+s);
      {$I-}
       MkDir (HS);
      {$I+}
      Err := IOResult;
      if Err <> 0 then
       begin
        { did another parallel instance create it in the mean time? }
        if not PathExists(hs) then
          begin
            { no -> error }
            Str (Err, SErr);
            Verbose (V_Error, 'Directory creation of "'+HS+'" failed ' + SErr);
          end;
       end;
    end;
end;


    Function RemoveFile(const f:string):boolean;
      var
        g : file;
      begin
        assign(g,f);
        {$I-}
         erase(g);
        {$I+}
        RemoveFile:=(ioresult=0);
      end;


function Copyfile(const fn1,fn2:string;append:boolean) : longint;
const
  bufsize = 16384;
var
  f,g : file;
  oldfilemode : byte;
  st : string;
  addsize,
  i   : longint;
  buf : pointer;
begin
  if Append then
   Verbose(V_Debug,'Appending '+fn1+' to '+fn2)
  else
   Verbose(V_Debug,'Copying '+fn1+' to '+fn2);
  assign(g,fn2);
  if append then
   begin
     {$I-}
      reset(g,1);
     {$I+}
     if ioresult<>0 then
      append:=false
     else
      seek(g,filesize(g));
   end;
  if not append then
   begin
     {$I-}
      rewrite(g,1);
     {$I+}
     if ioresult<>0 then
      Verbose(V_Error,'Can''t open '+fn2+' for output');
   end;
  assign(f,fn1);
  {$I-}
  { Try using read only file mode }
  oldfilemode:=filemode;
  filemode:=READ_ONLY;
  reset(f,1);
  {$I+}
  addsize:=0;
  getmem(buf,bufsize);
  if ioresult<>0 then
   begin
     sleep(1000);
     {$I-}
      reset(f,1);
     {$I+}
      if ioresult<>0 then
        begin
          Verbose(V_Warning,'Can''t open '+fn1);
          st:='Can''t open '+fn1;
          i:=length(st);
          // blocksize is larger than 255, so no check is needed
          move(st[1],buf^,i);
          blockwrite(g,buf^,i);
          freemem(buf,bufsize);
          close(g);
          filemode:=oldfilemode;
          exit;
        end;
   end;
  filemode:=oldfilemode;
  repeat
    blockread(f,buf^,bufsize,i);
    blockwrite(g,buf^,i);
    addsize:=addsize+i;
  until i<bufsize;
  freemem(buf,bufsize);
  close(f);
  close(g);
  CopyFile:=addsize;
end;


procedure AddLog(const logfile,s:string);
var
  t : text;
begin
  assign(t,logfile);
  {$I-}
   append(t);
  {$I+}
  if ioresult<>0 then
   begin
     {$I-}
      rewrite(t);
     {$I+}
     if ioresult<>0 then
       Verbose(V_Abort,'Can''t append to '+logfile);
   end;
  writeln(t,s);
  close(t);
end;


procedure ForceLog(const logfile:string);
var
  t : text;
begin
  assign(t,logfile);
  {$I-}
   append(t);
  {$I+}
  if ioresult<>0 then
   begin
     {$I-}
      rewrite(t);
     {$I+}
     if ioresult<>0 then
       Verbose(V_Abort,'Can''t Create '+logfile);
   end;
  close(t);
end;


function GetCompilerInfo(c:tcompinfo):boolean;
var
  t  : text;
  hs : string;
begin
  GetCompilerInfo:=false;
  { Try to get all information in one call, this is
    supported in 1.1. Older compilers 1.0.x will only
    return the first info }
  case c of
    compver :
      begin
        if DefaultCompilerVersion<>'' then
          begin
            GetCompilerInfo:=true;
            exit;
          end;
        hs:='-iVTPTO';
      end;
    compcpu :
      begin
        if DefaultCompilerCPU<>'' then
          begin
            GetCompilerInfo:=true;
            exit;
          end;
        hs:='-iTPTOV';
      end;
    comptarget :
      begin
        if DefaultCompilerTarget<>'' then
          begin
            GetCompilerInfo:=true;
            exit;
          end;
        hs:='-iTOTPV';
      end;
  end;
  ExecuteRedir(CompilerBin,hs,'','out.'+UniqueSuffix,'');
  assign(t,'out.'+UniqueSuffix);
  {$I-}
   reset(t);
  {$ifdef windows}
    { try to cope with Windows problems related to AntiVirus scanner
      that generate lag time during which access to a given if is forbidden }
   if (inoutres=5) then
     begin
       Sleep(5000);
       ioresult;
       Verbose(V_Warning,'Windows file not accessible out.'+UniqueSuffix);
       reset(t);
     end;
   {$endif windows}
   readln(t,hs);
   close(t);
   erase(t);
  {$I+}
  if ioresult<>0 then
   Verbose(V_Error,'Can''t get Compiler Info')
  else
   begin
     Verbose(V_Debug,'Retrieved Compiler Info: "'+hs+'"');
     case c of
       compver :
         begin
           DefaultCompilerVersion:=GetToken(hs);
           DefaultCompilerCPU:=GetToken(hs);
           DefaultCompilerTarget:=GetToken(hs);
         end;
       compcpu :
         begin
           DefaultCompilerCPU:=GetToken(hs);
           DefaultCompilerTarget:=GetToken(hs);
           DefaultCompilerVersion:=GetToken(hs);
         end;
       comptarget :
         begin
           DefaultCompilerTarget:=GetToken(hs);
           DefaultCompilerCPU:=GetToken(hs);
           DefaultCompilerVersion:=GetToken(hs);
         end;
     end;
     GetCompilerInfo:=true;
   end;
end;


function GetCompilerVersion:boolean;
const
  CompilerVersionDebugWritten : boolean = false;
begin
  if CompilerVersion='' then
    begin
      GetCompilerVersion:=GetCompilerInfo(compver);
      CompilerVersion:=DefaultCompilerVersion;
    end
  else
    GetCompilerVersion:=true;
  if GetCompilerVersion and not CompilerVersionDebugWritten then
    begin
      Verbose(V_Debug,'Compiler Version: "'+CompilerVersion+'"');
      CompilerVersionDebugWritten:=true;
    end;
end;


function GetCompilerCPU:boolean;
const
  CompilerCPUDebugWritten : boolean = false;
begin
  if CompilerCPU='' then
    begin
      GetCompilerCPU:=GetCompilerInfo(compcpu);
      CompilerCPU:=lowercase(DefaultCompilerCPU);
    end
  else
    GetCompilerCPU:=true;
  if GetCompilerCPU and not CompilerCPUDebugWritten then
    begin
      Verbose(V_Debug,'Compiler CPU: "'+CompilerCPU+'"');
      CompilerCPUDebugWritten:=true;
    end;
end;


function GetCompilerTarget:boolean;
const
  CompilerTargetDebugWritten : boolean = false;
begin
  if CompilerTarget='' then
    begin
      GetCompilerTarget:=GetCompilerInfo(comptarget);
      CompilerTarget:=lowercase(DefaultCompilerTarget);
    end
  else
    GetCompilerTarget:=true;
  if GetCompilerTarget and not CompilerTargetDebugWritten then
    begin
      Verbose(V_Debug,'Compiler Target: "'+CompilerTarget+'"');
      CompilerTargetDebugWritten:=true;
    end;
end;


function CompilerFullTarget:string;
begin
  if UseOSOnly then
    CompilerFullTarget:=CompilerTarget
  else
    CompilerFullTarget:=CompilerCPU+'-'+CompilerTarget;
end;

{ Set the three constants above according to
  the current target }

procedure SetTargetDirectoriesStyle;
var
  LTarget : string;
begin
  { Call this first to ensure that CompilerTarget is not empty }
  GetCompilerTarget;
  LTarget := CompilerTarget;
  TargetHasDosStyleDirectories :=
    (LTarget='emx') or
    (LTarget='go32v2') or
    (LTarget='msdos') or
    (LTarget='nativent') or
    (LTarget='os2') or
    (LTarget='symbian') or
    (LTarget='watcom') or
    (LTarget='wdosx') or
    (LTarget='win16') or
    (LTarget='win32') or
    (LTarget='win64');
  TargetAmigaLike:=
    (LTarget='amiga') or
    (LTarget='morphos');
  TargetIsMacOS:=
    (LTarget='macos');
  { Base on whether UNIX is defined as default macro
    in extradefines in systesms/i_XXX.pas units }
  TargetIsUnix:=
    (LTarget='linux') or
    (LTarget='linux6432') or
    (LTarget='freebsd') or
    (LTarget='openbsd') or
    (LTarget='netbsd') or
    (LTarget='beos') or
    (LTarget='haiku') or
    (LTarget='solaris') or
    (LTarget='iphonesim') or
    (LTarget='darwin') or
    (LTarget='aix') or
    (LTarget='android');

  { Set ExeExt for CompilerTarget.
    This list has been set up 2013-01 using the information in
    compiler/system/i_XXX.pas units.
    We should update this list when adding new targets PM }
  if (TargetHasDosStyleDirectories) or (LTarget='wince') then
    begin
      ExeExt:='.exe';
      DllExt:='.dll';
      DllPrefix:='';
    end
  else if LTarget='atari' then
    begin
      ExeExt:='.tpp';
      DllExt:='.dll';
      DllPrefix:='';
    end
  else if LTarget='gba' then
    ExeExt:='.gba'
  else if LTarget='nds' then
    ExeExt:='.bin'
  else if (LTarget='netware') or (LTarget='netwlibc') then
    begin
      ExeExt:='.nlm';
      DllExt:='.nlm';
      DllPrefix:='';
    end
  else if LTarget='wii' then
    ExeExt:='.dol';
end;

{$ifndef LIMIT83FS}
{ Set the UseOSOnly constant above according to
  the current target }

procedure SetUseOSOnly;
var
  LTarget : string;
begin
  { Call this first to ensure that CompilerTarget is not empty }
  GetCompilerTarget;
  LTarget := CompilerTarget;
  UseOSOnly:= (LTarget='emx') or
              (LTarget='go32v2') or
              (LTarget='msdos') or
              (LTarget='os2');
end;
{$endif not LIMIT83FS}

procedure SetTargetCanCompileLibraries;
var
  LTarget : string;
begin
  { Call this first to ensure that CompilerTarget is not empty }
  GetCompilerTarget;
  LTarget := CompilerTarget;
  { Feel free to add other targets here }
  if (LTarget='go32v2') then
    TargetCanCompileLibraries:=false;
end;


function OutputFileName(Const s,ext:String):String;
begin
{$ifndef macos}
  OutputFileName:=OutputDir+'/'+ForceExtension(s,ext);
{$else macos}
  OutputFileName:=ConcatMacPath(OutputDir,ForceExtension(s,ext));
{$endif macos}
end;


function TestOutputFileName(Const pref,base,ext:String):String;
begin
{$ifndef macos}
  TestOutputFileName:=TestOutputDir+'/'+ForceExtension(pref+SplitFileName(base),ext);
{$else macos}
  TestOutputFileName:=ConcatMacPath(TestOutputDir,ForceExtension(pref+SplitFileName(base),ext));
{$endif macos}
end;


function TestLogFileName(Const pref,base,ext:String):String;
var
  LogDir: String;
begin
  LogDir:=TestOutputDir;
{$ifndef macos}
  if UniqueSuffix<>'' then
    LogDir:=LogDir+'/..';
  TestLogFileName:=LogDir+'/'+ForceExtension(pref+SplitFileName(base),ext);
{$else macos}
  if UniqueSuffix<>'' then
    LogDir:=LogDir+'::';
  TestLogFileName:=ConcatMacPath(LogDir,ForceExtension(pref+SplitFileName(base),ext));
{$endif macos}
end;


function ExitWithInternalError(const OutName:string):boolean;
var
  t : text;
  s : string;
begin
  ExitWithInternalError:=false;
  { open logfile }
  assign(t,Outname);
  {$I-}
   reset(t);
  {$I+}
  if ioresult<>0 then
   exit;
  while not eof(t) do
   begin
     readln(t,s);
     if (pos('Fatal: Internal error ',s)>0) or
        (pos('Error: Compilation raised exception internally',s)>0) then
      begin
        ExitWithInternalError:=true;
        break;
      end;
   end;
  close(t);
end;


{ Takes each option from AddOptions list
  considered as a space separated list
  and adds the option to args
  unless option contains a percent sign,
  in that case, the option after % will be added
  to args only if CompilerTarget is listed in
  the string part before %.
  NOTE: this function does not check for
  quoted options...
  The list before % must of course contain no spaces. }

procedure AppendOptions(AddOptions : string;var args : string);
var
  endopt,percentpos : longint;
  opttarget, currentopt : string;
begin
  Verbose(V_Debug,'AppendOptions called with AddOptions="'+AddOptions+'"');
  AddOptions:=trimspace(AddOptions);
  repeat
    endopt:=pos(' ',AddOptions);
    if endopt=0 then
      endopt:=length(AddOptions);
    currentopt:=trimspace(copy(AddOptions,1,endopt));
    AddOptions:=trimspace(copy(Addoptions,endopt+1,length(AddOptions)));
    if currentopt<>'' then
      begin
        percentpos:=pos('%',currentopt);
        if (percentpos=0) then
          begin
            Verbose(V_Debug,'Adding option="'+currentopt+'"');
            args:=args+' '+currentopt;
          end
        else
          begin
            opttarget:=lowercase(copy(currentopt,1,percentpos-1));
            if IsInList(CompilerTarget, opttarget) then
              begin
                Verbose(V_Debug,'Adding target specific option="'+currentopt+'" for '+opttarget);
                args:=args+' '+copy(currentopt,percentpos+1,length(currentopt))
              end
            else
              Verbose(V_Debug,'No matching target "'+currentopt+'"');
          end;
      end;
  until AddOptions='';
end;

{ This function removes some incompatible
  options from TEST_OPT before adding them to
  the list of options passed to the compiler.
  %DELOPT=XYZ  will remove XYZ exactly
  %DELOPT=XYZ* will remove all options starting with XYZ.
  NOTE: This fuinction does not handle quoted options. }
function DelOptions(Pattern, opts : string) : string;
var
  currentopt : string;
  optpos, endopt, startpos, endpos : longint;
  iswild : boolean;
begin
  opts:=trimspace(opts);
  pattern:=trimspace(pattern);
  repeat
    endpos:=pos(' ',pattern);
    if endpos=0 then
      endpos:=length(pattern);
    currentopt:=trimspace(copy(pattern,1,endpos));
    pattern:=trimspace(copy(pattern,endpos+1,length(pattern)));
    if currentopt<>'' then
      begin
        if currentopt[length(currentopt)]='*' then
          begin
            iswild:=true;
            system.delete(currentopt,length(currentopt),1);
          end
        else
          iswild:=false;
        startpos:=1;
        repeat
          optpos:=pos(currentopt,copy(opts,startpos,length(opts)));
          if optpos>0 then
            begin
              { move to index in full opts string }
              optpos:=optpos+startpos-1;
              { compute position of end of opt }
              endopt:=optpos+length(currentopt);
              { use that end as start position for next round }
              startpos:=endopt;
              if iswild then
                begin
                  while (opts[endopt]<>' ') and
                    (endopt<length(opts)) do
                    begin
                      inc(endopt);
                      inc(startpos);
                    end;
                  Verbose(V_Debug,'Pattern match found "'+currentopt+'*" in "'+opts+'"');
                  system.delete(opts,optpos,endopt-optpos+1);
                  Verbose(V_Debug,'After opts="'+opts+'"');
                end
              else
                begin
                  if (endopt>length(opts)) or (opts[endopt]=' ') then
                    begin
                      Verbose(V_Debug,'Exact match found "'+currentopt+'" in "'+opts+'"');
                      system.delete(opts,optpos,endopt-optpos+1);
                      Verbose(V_Debug,'After opts="'+opts+'"');
                    end
                  else
                    begin
                      Verbose(V_Debug,'No exact match "'+currentopt+'" in "'+opts+'"');
                    end;
                end;

            end;
        until optpos=0;
      end;
  until pattern='';
  DelOptions:=opts;
end;

function RunCompiler(const ExtraPara: string):boolean;
var
  args,LocalExtraArgs,
  wpoargs,wposuffix : string;
  passnr,
  passes  : longint;
  execres : boolean;
  EndTicks,
  StartTicks : int64;
begin
  RunCompiler:=false;
  args:='-n -T'+CompilerTarget+' -Fu'+RTLUnitsDir;
  if ExtraPara<>'' then
    args:=args+' '+ExtraPara;
  { the helper object files have been copied to the common directory }
  if UniqueSuffix<>'' then
    args:=args+' -Fo'+TestOutputDir+'/..';
  args:=args+' -FE'+TestOutputDir;
  if TargetIsMacOS then
    args:=args+' -WT ';  {tests should be compiled as MPWTool}
  if Config.DelOptions<>'' then
   LocalExtraArgs:=DelOptions(Config.DelOptions,ExtraCompilerOpts)
  else
    LocalExtraArgs:=ExtraCompilerOpts;

  if LocalExtraArgs<>'' then
   args:=args+' '+LocalExtraArgs;
  if TargetIsUnix then
    begin
      { Add runtime library path to current dir to find .so files }
      if Config.NeedLibrary then
        begin
          if (CompilerTarget='darwin') or
	     (CompilerTarget='aix') then
            args:=args+' -Fl'+TestOutputDir
	  else
          { do not use single quote for -k as they are mishandled on
            Windows Shells }
            args:=args+' -Fl'+TestOutputDir+' -k-rpath -k.'
        end;
    end;
  if Config.NeedOptions<>'' then
   AppendOptions(Config.NeedOptions,args);
  wpoargs:='';
  wposuffix:='';
  if (Config.WpoPasses=0) or
     (Config.WpoParas='') then
    passes:=1
  else
    passes:=config.wpopasses+1;
  args:=args+' '+PPFile[current];

  for passnr:=1 to passes do
    begin
      if (passes>1) then
        begin
          wposuffix:='_'+tostr(passnr);
          wpoargs:=' -OW'+config.wpoparas+' -FW'+TestOutputFileName('',PPFile[current],'wp'+tostr(passnr));
          if (passnr>1) then
            wpoargs:=wpoargs+' -Ow'+config.wpoparas+' -Fw'+TestOutputFileName('',PPFile[current],'wp'+tostr(passnr-1));
        end;
      Verbose(V_Debug,'Executing '+compilerbin+' '+args+wpoargs);
      { also get the output from as and ld that writes to stderr sometimes }
      StartTicks:=GetMicroSTicks;
    {$ifndef macos}
      execres:=ExecuteRedir(CompilerBin,args+wpoargs,'',CompilerLogFile+wposuffix,'stdout');
    {$else macos}
      {Due to that Toolserver is not reentrant, we have to asm and link via script.}
      execres:=ExecuteRedir(CompilerBin,'-s '+args+wpoargs,'',CompilerLogFile+wposuffix,'stdout');
      if execres then
        execres:=ExecuteRedir(TestOutputDir + ':ppas','','',CompilerLogFile+wpo_suffix,'stdout');
    {$endif macos}
      EndTicks:=GetMicroSTicks;
      Verbose(V_Debug,'Exitcode '+ToStr(ExecuteResult));
      if BenchmarkInfo then
        begin
          Verbose(V_Normal,'Compilation took '+ToStr(EndTicks-StartTicks)+' us');
        end;

      if passes > 1 then
        CopyFile(CompilerLogFile+wposuffix,CompilerLogFile,true);
      { Error during execution? }
      if (not execres) and (ExecuteResult=0) then
        begin
          AddLog(FailLogFile,TestName);
          AddLog(ResLogFile,failed_to_compile+PPFileInfo[current]);
          AddLog(LongLogFile,line_separation);
          AddLog(LongLogFile,failed_to_compile+PPFileInfo[current]);
          if CopyFile(CompilerLogFile,LongLogFile,true)=0 then
            AddLog(LongLogFile,'IOStatus'+ToStr(IOStatus));
          { avoid to try again }
          AddLog(ExeLogFile,failed_to_compile+PPFileInfo[current]);
          Verbose(V_Warning,'IOStatus: '+ToStr(IOStatus));
          exit;
        end;

      { Check for internal error }
      if ExitWithInternalError(CompilerLogFile) then
       begin
         AddLog(FailLogFile,TestName);
         if Config.Note<>'' then
          AddLog(FailLogFile,Config.Note);
         AddLog(ResLogFile,failed_to_compile+PPFileInfo[current]+' internalerror generated');
         AddLog(LongLogFile,line_separation);
         AddLog(LongLogFile,failed_to_compile+PPFileInfo[current]);
         if Config.Note<>'' then
          AddLog(LongLogFile,Config.Note);
         if CopyFile(CompilerLogFile,LongLogFile,true)=0 then
           AddLog(LongLogFile,'Internal error in compiler');
         { avoid to try again }
         AddLog(ExeLogFile,failed_to_compile+PPFileInfo[current]);
         Verbose(V_Warning,'Internal error in compiler');
         exit;
       end;
    end;

  { Should the compile fail ? }
  if Config.ShouldFail then
   begin
     if ExecuteResult<>0 then
      begin
        AddLog(ResLogFile,success_compilation_failed+PPFileInfo[current]);
        { avoid to try again }
        AddLog(ExeLogFile,success_compilation_failed+PPFileInfo[current]);
        RunCompiler:=true;
      end
     else
      begin
        AddLog(FailLogFile,TestName);
        if Config.Note<>'' then
          AddLog(FailLogFile,Config.Note);
        AddLog(ResLogFile,failed_compilation_successful+PPFileInfo[current]);
        AddLog(LongLogFile,line_separation);
        AddLog(LongLogFile,failed_compilation_successful+PPFileInfo[current]);
        { avoid to try again }
        AddLog(ExeLogFile,failed_compilation_successful+PPFileInfo[current]);
        if Config.Note<>'' then
          AddLog(LongLogFile,Config.Note);
        CopyFile(CompilerLogFile,LongLogFile,true);
      end;
   end
  else
   begin
     if (ExecuteResult<>0) and
        (((Config.KnownCompileNote<>'') and (Config.KnownCompileError=0)) or
         ((Config.KnownCompileError<>0) and (ExecuteResult=Config.KnownCompileError))) then
      begin
        AddLog(FailLogFile,TestName+known_problem+Config.KnownCompileNote);
        AddLog(ResLogFile,failed_to_compile+PPFileInfo[current]+known_problem+Config.KnownCompileNote);
        AddLog(LongLogFile,line_separation);
        AddLog(LongLogFile,known_problem+Config.KnownCompileNote);
        AddLog(LongLogFile,failed_to_compile+PPFileInfo[current]+' ('+ToStr(ExecuteResult)+')');
        if Copyfile(CompilerLogFile,LongLogFile,true)=0 then
          AddLog(LongLogFile,known_problem+'exitcode: '+ToStr(ExecuteResult));
        Verbose(V_Warning,known_problem+'exitcode: '+ToStr(ExecuteResult));
      end
     else if ExecuteResult<>0 then
      begin
        AddLog(FailLogFile,TestName);
        if Config.Note<>'' then
          AddLog(FailLogFile,Config.Note);
        AddLog(ResLogFile,failed_to_compile+PPFileInfo[current]);
        AddLog(LongLogFile,line_separation);
        AddLog(LongLogFile,failed_to_compile+PPFileInfo[current]);
        if Config.Note<>'' then
          AddLog(LongLogFile,Config.Note);
        if CopyFile(CompilerLogFile,LongLogFile,true)=0 then
          AddLog(LongLogFile,'Exitcode: '+ToStr(ExecuteResult)+' (expected 0)');
        { avoid to try again }
        AddLog(ExeLogFile,failed_to_compile+PPFileInfo[current]);
        Verbose(V_Warning,'Exitcode: '+ToStr(ExecuteResult)+' (expected 0)');
      end
     else
      begin
        AddLog(ResLogFile,successfully_compiled+PPFileInfo[current]);
        RunCompiler:=true;
      end;
   end;
end;


function CheckTestExitCode(const OutName:string):boolean;
var
  t : text;
  s : string;
  i,code : integer;
begin
  CheckTestExitCode:=false;
  { open logfile }
  assign(t,Outname);
  {$I-}
   reset(t);
  {$I+}
  if ioresult<>0 then
   exit;
  while not eof(t) do
   begin
     readln(t,s);
     i:=pos('TestExitCode: ',s);
     if i>0 then
      begin
        delete(s,1,i+14-1);
        val(s,ExecuteResult,code);
        if code=0 then
          CheckTestExitCode:=true;
        break;
      end;
   end;
  close(t);
end;

function LibraryExists(const PPFile : string; out FileName : string) : boolean;
begin
   { Check if a dynamic library XXX was created }
   { Windows XXX.dll style }
  FileName:=TestOutputFilename('',PPFile,'dll');
  if FileExists(FileName) then
    begin
      LibraryExists:=true;
      exit;
    end;
   { Linux libXXX.so style }
  FileName:=TestOutputFilename('lib',PPFile,'so');
  if FileExists(FileName) then
    begin
      LibraryExists:=true;
      exit;
    end;
   { Darwin libXXX.dylib style }
  FileName:=TestOutputFilename('lib',PPFile,'dylib');
  if FileExists(FileName) then
    begin
      LibraryExists:=true;
      exit;
    end;
   { MacOS LibXXX style }
  FileName:=TestOutputFilename('Lib',PPFile,'');
  if FileExists(FileName) then
    begin
      LibraryExists:=true;
      exit;
    end;
   { Netware wlic XXX.nlm style }
  FileName:=TestOutputFilename('',PPFile,'nlm');
  if FileExists(FileName) then
    begin
      LibraryExists:=true;
      exit;
    end;
   { Amiga  XXX.library style }
  FileName:=TestOutputFilename('',PPFile,'library');
  if FileExists(FileName) then
    begin
      LibraryExists:=true;
      exit;
    end;
  LibraryExists:=false;
end;

function ExecuteRemote(prog,args:string;out StartTicks,EndTicks : int64):boolean;
const
  MaxTrials = 5;
var
  Trials : longint;
  Res : boolean;
begin
  if SplitFileExt(prog)='' then
    prog:=prog+SrcExeExt;
  Verbose(V_Debug,'RemoteExecuting '+Prog+' '+args);
  StartTicks:=GetMicroSTicks;
  Res:=false;
  Trials:=0;
  While (Trials<MaxTrials) and not Res do
    begin
      inc(Trials);
      Res:=ExecuteRedir(prog,args,'',EXELogFile,'stdout');
      if not Res then
        Verbose(V_Debug,'Call to '+prog+' failed: '+
          'IOStatus='+ToStr(IOStatus)+
          ' RedirErrorOut='+ToStr(RedirErrorOut)+
          ' RedirErrorIn='+ToStr(RedirErrorIn)+
          ' RedirErrorError='+ToStr(RedirErrorError)+
          ' ExecuteResult='+ToStr(ExecuteResult));
    end;

  if Trials>1 then
    Verbose(V_Debug,'Done in '+tostr(trials)+' trials');
  EndTicks:=GetMicroSTicks;
  ExecuteRemote:=res;
end;

function ExecuteEmulated(const prog,args,FullExeLogFile:string;out StartTicks,EndTicks : int64):boolean;
begin
  Verbose(V_Debug,'EmulatorExecuting '+Prog+' '+args);
  StartTicks:=GetMicroSTicks;
  ExecuteEmulated:=ExecuteRedir(prog,args,'',FullExeLogFile,'stdout');
  EndTicks:=GetMicroSTicks;
end;


function MaybeCopyFiles(const FileToCopy : string) : boolean;
var
  TestRemoteExe,
  pref     : string;
  LocalFile, RemoteFile, s: string;
  LocalPath: string;
  i       : integer;
  execres : boolean;
  EndTicks,
  StartTicks : int64;
  FileList   : TStringList;
  RelativeToConfigMarker : TObject;

  function BuildFileList: TStringList;
    var
      s      : string;
      index  : longint;
    begin
      s:=Config.Files;
      if (length(s) = 0) and (Config.ConfigFileSrc='') then
        begin
          Result:=nil;
          exit;
        end;
      Result:=TStringList.Create;
      if s<>'' then
        repeat
          index:=pos(' ',s);
          if index=0 then
            LocalFile:=s
          else
            LocalFile:=copy(s,1,index-1);
          Result.Add(LocalFile);
          if index=0 then
            break;
          s:=copy(s,index+1,length(s)-index);
        until false;
      if Config.ConfigFileSrc<>'' then
        begin
          if Config.ConfigFileSrc=Config.ConfigFileDst then
            Result.AddObject(Config.ConfigFileSrc,RelativeToConfigMarker)
          else
            Result.AddObject(Config.ConfigFileSrc+'='+Config.ConfigFileDst,RelativeToConfigMarker);
        end;
    end;

begin
  RelativeToConfigMarker:=TObject.Create;
  if RemoteAddr='' then
    begin
      FileList:=BuildFileList;
      if assigned(FileList) then
        begin
          LocalPath:=SplitPath(PPFile[current]);
          if Length(LocalPath) > 0 then
            LocalPath:=LocalPath+'/';
          for i:=0 to FileList.count-1 do
            begin
              if FileList.Names[i]<>'' then
                begin
                  LocalFile:=FileList.Names[i];
                  RemoteFile:=FileList.ValueFromIndex[i];
                end
              else
                begin
                  LocalFile:=FileList[i];
                  RemoteFile:=LocalFile;
                end;
              if FileList.Objects[i]=RelativeToConfigMarker then
                s:='config/'+LocalFile
              else
                s:=LocalPath+LocalFile;
              CopyFile(s,TestOutputDir+'/'+RemoteFile,false);
            end;
          FileList.Free;
        end;
      RelativeToConfigMarker.Free;
      exit(true);
    end;
  execres:=true;
  { Check if library should be deleted. Do not copy to remote target in such case. }
  if (deAfter in DelExecutable) and (Config.DelFiles <> '') then
    if SplitFileName(FileToCopy) = DllPrefix + Trim(Config.DelFiles) + DllExt then
      exit;
  { We don't want to create subdirs, remove paths from the test }
  TestRemoteExe:=RemotePath+'/'+SplitFileName(FileToCopy);
  if deBefore in DelExecutable then
    begin
      s:=RemoteRshParas+' rm ';
      if rshprog <> 'adb' then
        s:=s+'-f ';
      ExecuteRemote(rshprog,s+TestRemoteExe,
                    StartTicks,EndTicks);
    end;
  execres:=ExecuteRemote(rcpprog,RemotePara+' '+FileToCopy+' '+
                         RemotePathPrefix+TestRemoteExe,StartTicks,EndTicks);
  if not execres then
  begin
    Verbose(V_normal, 'Could not copy executable '+FileToCopy);
    RelativeToConfigMarker.Free;
    exit(execres);
  end;
  FileList:=BuildFileList;
  if assigned(FileList) then
  begin
    LocalPath:=SplitPath(PPFile[current]);
    if Length(LocalPath) > 0 then
      LocalPath:=LocalPath+'/';
    for i:=0 to FileList.count-1 do
      begin
        if FileList.Names[i]<>'' then
          begin
            LocalFile:=FileList.Names[i];
            RemoteFile:=FileList.ValueFromIndex[i];
          end
        else
          begin
            LocalFile:=FileList[i];
            RemoteFile:=LocalFile;
          end;
        RemoteFile:=RemotePath+'/'+SplitFileName(RemoteFile);
        if FileList.Objects[i]=RelativeToConfigMarker then
          LocalFile:='config/'+LocalFile
        else
          LocalFile:=LocalPath+LocalFile;
        if DoVerbose and (rcpprog='pscp') then
          pref:='-v '
        else
          pref:='';
        execres:=ExecuteRemote(rcpprog,pref+RemotePara+' '+LocalFile+' '+
                               RemotePathPrefix+RemoteFile,StartTicks,EndTicks);
        if not execres then
        begin
          Verbose(V_normal, 'Could not copy required file '+LocalFile);
          FileList.Free;
          RelativeToConfigMarker.Free;
          exit(false);
        end;
      end;
  end;
  FileList.Free;
  MaybeCopyFiles:=execres;
  RelativeToConfigMarker.Free;
end;

function RunExecutable:boolean;
const
{$ifdef unix}
  CurrDir = './';
{$else}
  CurrDir = '';
{$endif}
var
  OldDir, s, ss,
  execcmd,
  FullExeLogFile,
  TestRemoteExe,
  TestExe  : string;
  execres  : boolean;
  EndTicks,
  StartTicks : int64;
  OldExecuteResult: longint;
begin
  RunExecutable:=false;
  execres:=true;

  TestExe:=TestOutputFilename('',PPFile[current],ExeExt);

  execres:=MaybeCopyFiles(TestExe);
  if EmulatorName<>'' then
    begin
      { Get full name out log file, because we change the directory during
        execution }
      FullExeLogFile:=FExpand(EXELogFile);
      {$I-}
       GetDir(0,OldDir);
       ChDir(TestOutputDir);
      {$I+}
      ioresult;
      s:=CurrDir+SplitFileName(TestExe);
      { Add -Ssource_file_name for dosbox_wrapper }
      if pos('dosbox_wrapper',EmulatorName)>0 then
        s:=s+' -S'+PPFile[current];
      execres:=ExecuteEmulated(EmulatorName,EmulatorOpts+' '+s,FullExeLogFile,StartTicks,EndTicks);
      {$I-}
       ChDir(OldDir);
      {$I+}
    end
  else if RemoteAddr<>'' then
    begin
      TestRemoteExe:=RemotePath+'/'+SplitFileName(TestExe);
      { rsh doesn't pass the exitcode, use a second command to print the exitcode
        on the remoteshell to stdout }
      if DoVerbose and (rshprog='plink') then
        execcmd:='-v '+RemoteRshParas
      else
        execcmd:=RemoteRshParas;
      execcmd:=execcmd+' '+rquote+
         'chmod 755 '+TestRemoteExe+
          ' && cd '+RemotePath+' && { ';
      { Using -rpath . at compile time does not seem
        to work for programs copied over to remote machine,
        at least not for FreeBSD.
        Does this work for all shells? }
      if Config.NeedLibrary then
        begin
          if RemoteShellNeedsExport then
            if CompilerTarget='darwin' then
              execcmd:=execcmd+' DYLD_LIBRARY_PATH=.; export DYLD_LIBRARY_PATH;'
            else
              execcmd:=execcmd+' LD_LIBRARY_PATH=.; export LD_LIBRARY_PATH;'
          else
            if CompilerTarget='darwin' then
              execcmd:=execcmd+' setenv DYLD_LIBRARY_PATH=.; '
            else
              execcmd:=execcmd+' setenv LD_LIBRARY_PATH=.; '
        end;


      if UseTimeout then
      begin
        if Config.Timeout=0 then
          Config.Timeout:=DefaultTimeout;
        str(Config.Timeout,s);
        if (RemoteShellBase='bash') then
          execcmd:=execcmd+'ulimit -t '+s+'; '
        else
          execcmd:=execcmd+'timeout -9 '+s;
      end;
      { as we moved to RemotePath, if path is not absolute
        we need to use ./execfilename only }
      if not isabsolute(TestRemoteExe) then
        execcmd:=execcmd+' ./'+SplitFileName(TestRemoteExe)
      else
        execcmd:=execcmd+' '+TestRemoteExe;
      execcmd:=execcmd+' ; echo TestExitCode: $?';
      if (deAfter in DelExecutable) and
         not Config.NeededAfter then
        begin
          { Delete executable if not needed after }
          execcmd:=execcmd+' ; rm ';
          if rshprog <> 'adb' then
            execcmd:=execcmd+'-f ';
          execcmd:=execcmd+SplitFileName(TestRemoteExe);
        end;
      execcmd:=execcmd+'; }'+rquote;
      execres:=ExecuteRemote(rshprog,execcmd,StartTicks,EndTicks);
      { Check for TestExitCode error in output, sets ExecuteResult }
      if not CheckTestExitCode(EXELogFile) then
        Verbose(V_Debug,'Failed to check exit code for '+execcmd);
      if (deAfter in DelExecutable) and ( (Config.DelFiles <> '') or (Config.Files <> '')) then
        begin
          ss:=Trim(Config.DelFiles + ' ' + Config.Files);
          execcmd:=RemoteRshParas+' ' + rquote + 'cd ' + RemotePath + ' && { ';
          while ss <> '' do
            begin
              s:=Trim(GetToken(ss, [' ',',',';']));
              if s = '' then
                break;
              if ExtractFileExt(s) = '' then
                // If file has no extension, treat it as exe or shared lib
                execcmd:=execcmd + 'rm ' + s + ExeExt + '; rm ' + DllPrefix + s + DllExt
              else
                execcmd:=execcmd + 'rm ' + s;
              execcmd:=execcmd + '; ';
            end;
          execcmd:=execcmd+'}'+rquote;
          // Save ExecuteResult and EXELogFile
          OldExecuteResult:=ExecuteResult;
          s:=EXELogFile;
          // Output results of cleanup commands to stdout
          EXELogFile:='';
          ExecuteRemote(rshprog,execcmd,StartTicks,EndTicks);
          // Restore
          EXELogFile:=s;
          ExecuteResult:=OldExecuteResult;
        end;
    end
  else
    begin
      { Get full name out log file, because we change the directory during
        execution }
      FullExeLogFile:=FExpand(EXELogFile);
      Verbose(V_Debug,'Executing '+TestExe);
      {$I-}
       GetDir(0,OldDir);
       ChDir(TestOutputDir);
      {$I+}
      ioresult;
      { don't redirect interactive and graph programs }
      StartTicks:=GetMicroSTicks;
      if Config.IsInteractive or Config.UsesGraph then
        execres:=ExecuteRedir(CurrDir+SplitFileName(TestExe),'','','','')
      else
        execres:=ExecuteRedir(CurrDir+SplitFileName(TestExe),'','',FullExeLogFile,'stdout');
      EndTicks:=GetMicroSTicks;
      {$I-}
       ChDir(OldDir);
      {$I+}
      ioresult;
    end;

  { Error during execution? }
  Verbose(V_Debug,'Exitcode '+ToStr(ExecuteResult));
  if BenchmarkInfo then
    begin
      Verbose(V_Normal,'Execution took '+ToStr(EndTicks-StartTicks)+' us');
    end;
  if (not execres) and (ExecuteResult=0) then
    begin
      AddLog(FailLogFile,TestName);
      AddLog(ResLogFile,failed_to_run+PPFileInfo[current]);
      AddLog(LongLogFile,line_separation);
      AddLog(LongLogFile,failed_to_run+PPFileInfo[current]);
      if CopyFile(EXELogFile,LongLogFile,true)=0 then
        AddLog(LongLogFile,'IOStatus: '+ToStr(IOStatus));
      { avoid to try again }
      AddLog(ExeLogFile,failed_to_run+PPFileInfo[current]);
      Verbose(V_Warning,'IOStatus: '+ToStr(IOStatus));
      exit;
    end;

  if ExecuteResult<>Config.ResultCode then
   begin
     if (ExecuteResult<>0) and
        (ExecuteResult=Config.KnownRunError) then
       begin
         AddLog(FailLogFile,TestName+known_problem+Config.KnownRunNote);
         AddLog(ResLogFile,failed_to_run+PPFileInfo[current]+known_problem+Config.KnownRunNote);
         AddLog(LongLogFile,line_separation);
         AddLog(LongLogFile,known_problem+Config.KnownRunNote);
         AddLog(LongLogFile,failed_to_run+PPFileInfo[current]+' ('+ToStr(ExecuteResult)+')');
         if Copyfile(EXELogFile,LongLogFile,true)=0 then
           begin
             AddLog(LongLogFile,known_problem+'exitcode: '+ToStr(ExecuteResult)+' (expected '+ToStr(Config.ResultCode)+')');
             AddLog(ExeLogFile,known_problem+'exitcode: '+ToStr(ExecuteResult)+' (expected '+ToStr(Config.ResultCode)+')');
           end;
         Verbose(V_Warning,known_problem+'exitcode: '+ToStr(ExecuteResult)+' (expected '+ToStr(Config.ResultCode)+')');
       end
     else
       begin
         AddLog(FailLogFile,TestName);
         AddLog(ResLogFile,failed_to_run+PPFileInfo[current]);
         AddLog(LongLogFile,line_separation);
         AddLog(LongLogFile,failed_to_run+PPFileInfo[current]+' ('+ToStr(ExecuteResult)+')');
         if Copyfile(EXELogFile,LongLogFile,true)=0 then
           begin
             AddLog(LongLogFile,'Exitcode: '+ToStr(ExecuteResult)+' (expected '+ToStr(Config.ResultCode)+')');
             AddLog(ExeLogFile,'Exitcode: '+ToStr(ExecuteResult)+' (expected '+ToStr(Config.ResultCode)+')');
           end;
         Verbose(V_Warning,'Exitcode: '+ToStr(ExecuteResult)+' (expected '+ToStr(Config.ResultCode)+')');
       end
   end
  else
   begin
     AddLog(ResLogFile,successfully_run+PPFileInfo[current]);
     RunExecutable:=true;
   end;

  if (deAfter in DelExecutable) and not Config.NeededAfter then
    begin
      Verbose(V_Debug,'Deleting executable '+TestExe);
      RemoveFile(TestExe);
      RemoveFile(ForceExtension(TestExe,ObjExt));
      RemoveFile(ForceExtension(TestExe,PPUExt));
    end;
end;


{ Try to collect information concerning the remote configuration
  Currently only records RemoteShell name and sets
  RemoteShellNeedsExport boolean variable }
procedure SetRemoteConfiguration;
var
  f : text;
  StartTicks,EndTicks : int64;
begin
  if RemoteAddr='' then
    exit;
  if rshprog = 'adb' then
    begin
      RemoteShellNeedsExport:=true;
      exit;
    end;
  ExeLogFile:='__remote.tmp';
  ExecuteRemote(rshprog,RemoteRshParas+
                ' "echo SHELL=${SHELL}"',StartTicks,EndTicks);
  Assign(f,ExeLogFile);
  Reset(f);
  While not eof(f) do
    begin
      Readln(f,RemoteShellBase);
      if pos('SHELL=',RemoteShellBase)>0 then
        begin
          RemoteShell:=TrimSpace(Copy(RemoteShellBase,pos('SHELL=',RemoteShellBase)+6,
                                      length(RemoteShellBase)));
          Verbose(V_Debug,'Remote shell is "'+RemoteShell+'"');
          RemoteShellBase:=SplitFileBase(RemoteShell);
          if (RemoteShellBase='bash') or (RemoteShellBase='sh') then
            RemoteShellNeedsExport:=true;
        end;
    end;
  Close(f);
end;

procedure getargs;

  procedure helpscreen;
  begin
    writeln('dotest [Options] <File>');
    writeln;
    writeln('Options can be:');
    writeln('  !ENV_NAME           parse environment variable ENV_NAME for options');
    writeln('  -A                  include ALL tests');
    writeln('  -ADB                use ADB to run tests');
    writeln('  -B                  delete executable before remote upload');
    writeln('  -C<compiler>        set compiler to use');
    writeln('  -D                  display execution time');
    writeln('  -E                  execute test also');
    writeln('  -G                  include graph tests');
    writeln('  -I                  include interactive tests');
    writeln('  -K                  include known bug tests');
    writeln('  -L<ext>             set extension of temporary files (prevent conflicts with parallel invocations)');
    writeln('  -M<emulator>        run the tests using the given emulator');
    writeln('  -N<emulator opts.>  pass options to the emulator');
    writeln('  -O                  use timeout wrapper for (remote) execution');
    writeln('  -P<path>            path to the tests tree on the remote machine');
    writeln('  -R<remote>          run the tests remotely with the given rsh/ssh address');
    writeln('  -S                  use ssh instead of rsh');
    writeln('  -T[cpu-]<os>        run tests for target cpu and os');
    writeln('  -U<remotepara>');
    writeln('                      pass additional parameter to remote program. Multiple -U can be used');
    writeln('  -V                  be verbose');
    writeln('  -W                  use putty compatible file names when testing (plink and pscp)');
    writeln('  -X                  don''t use COMSPEC');
    writeln('  -Y<opts>            extra options passed to the compiler. Several -Y<opt> can be given.');
    writeln('  -Z                  remove temporary files (executable,ppu,o)');
    halt(1);
  end;

  procedure interpret_option (para : string);
  var
    ch : char;
    j : longint;
  begin
   Verbose(V_Debug,'Interpreting  option"'+para+'"');
    ch:=Upcase(para[2]);
    delete(para,1,2);
    case ch of
     'A' :
       if UpperCase(para) = 'DB' then
         begin
           rshprog:='adb';
           rcpprog:='adb';
           rquote:='"';
           if RemoteAddr = '' then
             RemoteAddr:='1'; // fake remote addr (default device will be used)
         end
       else
         begin
           DoGraph:=true;
           DoInteractive:=true;
           DoKnown:=true;
           DoAll:=true;
         end;

     'B' : Include(DelExecutable,deBefore);

     'C' : CompilerBin:=Para;

     'D' : BenchMarkInfo:=true;

     'E' : DoExecute:=true;

     'G' : begin
             DoGraph:=true;
             if para='-' then
               DoUsual:=false;
           end;

     'I' : begin
             DoInteractive:=true;
             if para='-' then
               DoUsual:=false;
           end;

     'K' : begin
             DoKnown:=true;
             if para='-' then
               DoUsual:=false;
           end;

     'L' : begin
             UniqueSuffix:=Para;
             if UniqueSuffix='' then
               UniqueSuffix:=toStr(system.GetProcessID);
           end;

     'M' : EmulatorName:=Para;

     'N' : EmulatorOpts:=Para;

     'O' : UseTimeout:=true;

     'P' : RemotePath:=Para;

     'R' : RemoteAddr:=Para;

     'S' :
       begin
         rshprog:='ssh';
         rcpprog:='scp';
       end;

     'T' :
       begin
         j:=Pos('-',Para);
         if j>0 then
           begin
             CompilerCPU:=Copy(Para,1,j-1);
             CompilerTarget:=Copy(Para,j+1,length(para));
           end
         else
           CompilerTarget:=Para
       end;

     'U' :
       RemotePara:=RemotePara+' '+Para;

     'V' : DoVerbose:=true;

     'W' :
       begin
         rshprog:='plink';
         rcpprog:='pscp';
         rquote:='"';
       end;

     'X' : UseComSpec:=false;

     'Y' : ExtraCompilerOpts:= ExtraCompilerOpts +' '+ Para;

     'Z' : Include(DelExecutable,deAfter);
    end;
 end;

 procedure interpret_env(arg : string);
 var
   para : string;
   pspace : longint;
 begin
   Verbose(V_Debug,'Interpreting environment option"'+arg+'"');
   { Get rid of leading '!' }
   delete(arg,1,1);
   arg:=getenv(arg);
   Verbose(V_Debug,'Environment value is "'+arg+'"');
   while (length(arg)>0) do
     begin
       while (length(arg)>0) and (arg[1]=' ') do
         delete(arg,1,1);
       pspace:=pos(' ',arg);
       if pspace=0 then
         pspace:=length(arg)+1;
       para:=copy(arg,1,pspace-1);
       if (length(para)>0) and (para[1]='-') then
         interpret_option (para)
       else
         begin
           PPFile.Insert(current,ForceExtension(Para,'pp'));
           inc(current);
         end;
       delete(arg,1,pspace);
     end;
 end;

var
  param : string;
  i  : longint;

begin
  CompilerBin:='ppc386'+srcexeext;
  for i:=1 to paramcount do
   begin
     param:=Paramstr(i);
     if (param[1]='-') then
      interpret_option(param)
     else if (param[1]='!') then
       interpret_env(param)
     else
       begin
         PPFile.Insert(current,ForceExtension(Param,'pp'));
         inc(current);
       end;
   end;
  if current=0 then
    HelpScreen;
  { disable graph,interactive when running remote }
  if RemoteAddr<>'' then
    begin
      DoGraph:=false;
      DoInteractive:=false;
    end;
  { If we use PuTTY plink program with -load option,
    the IP address or name should not be added to
    the command line }
  if (rshprog='plink') and (pos('-load',RemotePara)>0) then
    RemoteRshParas:=RemotePara
  else
    if rshprog='adb' then
      begin
        if RemoteAddr <> '1' then
          RemotePara:=Trim('-s ' + RemoteAddr + ' ' + RemotePara);
        RemoteRshParas:=Trim(RemotePara + ' shell');
      end
    else
      RemoteRshParas:=RemotePara+' '+RemoteAddr;
  if rcpprog = 'adb' then
    begin
      RemotePathPrefix:='';
      RemotePara:=Trim(RemotePara + ' push');
    end
  else
    RemotePathPrefix:=RemoteAddr + ':';
end;


procedure RunTest;
var
  PPDir,LibraryName,LogSuffix,PPPrefix : string;
  Res : boolean;
begin
  Res:=GetConfig(PPFile[current],Config);
  TranslateConfig(Config);

  if Res then
    begin
      Res:=GetCompilerCPU;
      Res:=GetCompilerTarget;
{$ifndef MACOS}
      RTLUnitsDir:='tstunits/'+CompilerFullTarget;
{$else MACOS}
      RTLUnitsDir:=':tstunits:'+CompilerFullTarget;
{$endif MACOS}
      if not PathExists(RTLUnitsDir) then
        Verbose(V_Abort,'Unit path "'+RTLUnitsDir+'" does not exists');
{$ifndef MACOS}
      OutputDir:='output/'+CompilerFullTarget;
{$else MACOS}
      OutputDir:=':output:'+CompilerFullTarget;
{$endif MACOS}
      if not PathExists(OutputDir) then
        Verbose(V_Abort,'Output path "'+OutputDir+'" does not exists');
      { Make subdir in output if needed }
      PPDir:=SplitPath(PPFile[current]);
      if PPDir[length(PPDir)] in ['/','\'{$ifdef MACOS},':'{$endif MACOS}] then
        Delete(PPDir,length(PPDir),1);
      if PPDir<>'' then
        begin
{$ifndef MACOS}
          { handle paths that are parallel to the tests directory (let's hope
            that noone uses ../../ -.- ) }
          { ToDo: check relative paths on MACOS }
          PPPrefix:=Copy(PPDir,1,3);
          if (PPPrefix='../') or (PPPrefix='..\') then
            PPDir:='root/'+Copy(PPDir,4,length(PPDir));
          TestOutputDir:=OutputDir+'/'+PPDir;
          if UniqueSuffix<>'' then
            TestOutputDir:=TestOutputDir+'/'+UniqueSuffix;
{$else MACOS}
          TestOutputDir:=OutputDir+PPDir;
          if UniqueSuffix<>'' then
            TestOutputDir:=TestOutputDir+':'+UniqueSuffix;
{$endif MACOS}
          mkdirtree(TestOutputDir);
        end
      else
        TestOutputDir:=OutputDir;
      if UniqueSuffix<>'' then
        LogSuffix:=UniqueSuffix
      else
        LogSuffix:=SplitBasePath(PPDir)+'log';
      ResLogFile:=OutputFileName('log',LogSuffix);
      LongLogFile:=OutputFileName('longlog',LogSuffix);
      FailLogFile:=OutputFileName('faillist',LogSuffix);
      ForceLog(ResLogFile);
      ForceLog(LongLogFile);
      ForceLog(FailLogFile);
      { Per test logfiles }
      CompilerLogFile:=TestLogFileName('',SplitFileName(PPFile[current]),'log');
      ExeLogFile:=TestLogFileName('',SplitFileName(PPFile[current]),'elg');
      Verbose(V_Debug,'Using Compiler logfile: '+CompilerLogFile);
      Verbose(V_Debug,'Using Execution logfile: '+ExeLogFile);
    end;

  if Res then
   begin
     if Config.UsesGraph and (not DoGraph) then
      begin
        AddLog(ResLogFile,skipping_graph_test+PPFileInfo[current]);
        { avoid a second attempt by writing to elg file }
        AddLog(EXELogFile,skipping_graph_test+PPFileInfo[current]);
        Verbose(V_Warning,skipping_graph_test);
        Res:=false;
      end;
   end;

  if Res then
   begin
     if Config.IsInteractive and (not DoInteractive) then
      begin
        { avoid a second attempt by writing to elg file }
        AddLog(EXELogFile,skipping_interactive_test+PPFileInfo[current]);
        AddLog(ResLogFile,skipping_interactive_test+PPFileInfo[current]);
        Verbose(V_Warning,skipping_interactive_test);
        Res:=false;
      end;
   end;

  if Res then
   begin
     if Config.IsKnownCompileError and (not DoKnown) then
      begin
        { avoid a second attempt by writing to elg file }
        AddLog(EXELogFile,skipping_known_bug+PPFileInfo[current]);
        AddLog(ResLogFile,skipping_known_bug+PPFileInfo[current]);
        Verbose(V_Warning,skipping_known_bug);
        Res:=false;
      end;
   end;

  if Res and not DoUsual then
    res:=(Config.IsInteractive and DoInteractive) or
         (Config.IsKnownRunError and DoKnown) or
         (Config.UsesGraph and DoGraph);

  if Res then
   begin
     if (Config.MinVersion<>'') and not DoAll then
      begin
        Verbose(V_Debug,'Required compiler version: '+Config.MinVersion);
        Res:=GetCompilerVersion;
        if CompilerVersion<Config.MinVersion then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_compiler_version_too_low+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_compiler_version_too_low+PPFileInfo[current]);
           Verbose(V_Warning,'Compiler version too low '+CompilerVersion+' < '+Config.MinVersion);
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     if (Config.MaxVersion<>'') and not DoAll then
      begin
        Verbose(V_Debug,'Highest compiler version: '+Config.MaxVersion);
        Res:=GetCompilerVersion;
        if CompilerVersion>Config.MaxVersion then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_compiler_version_too_high+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_compiler_version_too_high+PPFileInfo[current]);
           Verbose(V_Warning,'Compiler version too high '+CompilerVersion+' > '+Config.MaxVersion);
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     if Config.NeedCPU<>'' then
      begin
        Verbose(V_Debug,'Required compiler cpu: '+Config.NeedCPU);
        if not IsInList(CompilerCPU,Config.NeedCPU) then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_other_cpu+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_other_cpu+PPFileInfo[current]);
           Verbose(V_Warning,'Compiler cpu "'+CompilerCPU+'" is not in list "'+Config.NeedCPU+'"');
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     if Config.SkipCPU<>'' then
      begin
        Verbose(V_Debug,'Skip compiler cpu: '+Config.SkipCPU);
        if IsInList(CompilerCPU,Config.SkipCPU) then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_other_cpu+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_other_cpu+PPFileInfo[current]);
           Verbose(V_Warning,'Compiler cpu "'+CompilerCPU+'" is in list "'+Config.SkipCPU+'"');
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     if Config.SkipEmu<>'' then
      begin
        Verbose(V_Debug,'Skip emulator: '+emulatorname);
        if IsInList(emulatorname,Config.SkipEmu) then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_other_cpu+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_other_cpu+PPFileInfo[current]);
           Verbose(V_Warning,'Emulator "'+emulatorname+'" is in list "'+Config.SkipEmu+'"');
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     if Config.NeedTarget<>'' then
      begin
        Verbose(V_Debug,'Required compiler target: '+Config.NeedTarget);
        if not IsInList(CompilerTarget,Config.NeedTarget) then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_other_target+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_other_target+PPFileInfo[current]);
           Verbose(V_Warning,'Compiler target "'+CompilerTarget+'" is not in list "'+Config.NeedTarget+'"');
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     if Config.SkipTarget<>'' then
      begin
        Verbose(V_Debug,'Skip compiler target: '+Config.SkipTarget);
        if IsInList(CompilerTarget,Config.SkipTarget) then
         begin
           { avoid a second attempt by writing to elg file }
           AddLog(EXELogFile,skipping_other_target+PPFileInfo[current]);
           AddLog(ResLogFile,skipping_other_target+PPFileInfo[current]);
           Verbose(V_Warning,'Compiler target "'+CompilerTarget+'" is in list "'+Config.SkipTarget+'"');
           Res:=false;
         end;
      end;
   end;

  if Res then
   begin
     { Use known bug, to avoid adding a new entry for this PM 2011-06-24 }
     if Config.NeedLibrary and not TargetCanCompileLibraries then
      begin
        AddLog(EXELogFile,skipping_known_bug+PPFileInfo[current]);
        AddLog(ResLogFile,skipping_known_bug+PPFileInfo[current]);
        Verbose(V_Warning,'Compiler target "'+CompilerTarget+'" does not support library compilation');
        Res:=false;
      end;
   end;

  if Res then
   begin
     Res:=RunCompiler('');
     if Res and Config.NeedRecompile then
      Res:=RunCompiler(Config.RecompileOpt);
   end;

  if Res and (not Config.ShouldFail) then
   begin
     if (Config.NoRun) then
      begin
        { avoid a second attempt by writing to elg file }
        AddLog(EXELogFile,skipping_run_test+PPFileInfo[current]);
        AddLog(ResLogFile,skipping_run_test+PPFileInfo[current]);
        Verbose(V_Debug,skipping_run_test);
        if LibraryExists(PPFile[current],LibraryName) then
          MaybeCopyFiles(LibraryName);
      end
     else if Config.IsKnownRunError and (not DoKnown) then
      begin
        { avoid a second attempt by writing to elg file }
        AddLog(EXELogFile,skipping_known_bug+PPFileInfo[current]);
        AddLog(ResLogFile,skipping_known_bug+PPFileInfo[current]);
        Verbose(V_Warning,skipping_known_bug);
      end
     else
      begin
        if DoExecute then
         begin
           if FileExists(TestOutputFilename('',PPFile[current],'ppu')) or
              FileExists(TestOutputFilename('',PPFile[current],'ppo')) or
              FileExists(TestOutputFilename('',PPFile[current],'ppw')) then
             begin
               AddLog(ExeLogFile,skipping_run_unit+PPFileInfo[current]);
               AddLog(ResLogFile,skipping_run_unit+PPFileInfo[current]);
               Verbose(V_Debug,'Unit found, skipping run test')
             end
           else if LibraryExists(PPFile[current],LibraryName) then
             begin
               Verbose(V_Debug,'Library found, skipping run test');
               MaybeCopyFiles(LibraryName);
             end
           else
             Res:=RunExecutable;
         end;
      end;
   end;
end;


begin
  Current:=0;
  PPFile:=TStringList.Create;
  PPFile.Capacity:=10;
  PPFileInfo:=TStringList.Create;
  PPFileInfo.Capacity:=10;
  GetArgs;
  SetTargetDirectoriesStyle;
  SetTargetCanCompileLibraries;
  SetRemoteConfiguration;
{$ifdef LIMIT83fs}
  UseOSOnly:=true;
{$else not LIMIT83fs}
  SetUseOSOnly;
{$endif not LIMIT83fs}
  Verbose(V_Debug,'Found '+ToStr(PPFile.Count)+' tests to run');
  if current>0 then
    for current:=0 to PPFile.Count-1 do
      begin
        SetPPFileInfo;
        TestName:=Copy(PPFile[current],1,Pos('.pp',PPFile[current])-1);
        Verbose(V_Normal,'Running test '+TestName+', file '+PPFile[current]);
        RunTest;
      end;
  PPFile.Free;
  PPFileInfo.Free;
end.
