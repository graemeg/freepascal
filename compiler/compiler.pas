{
    This unit is the interface of the compiler which can be used by
    external programs to link in the compiler

    Copyright (c) 1998-2005 by Florian Klaempfl

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

unit compiler;

{$i fpcdefs.inc}

interface

uses
{$ifdef GO32V2}
  emu387,
{$endif GO32V2}
{$ifdef WATCOM}
  emu387,
{$endif WATCOM}
{$if defined(unix) and (FPC_FULLVERSION>20700)}
  { system code page stuff for unix }
  unixcp,
{$endif}
{$IFNDEF USE_FAKE_SYSUTILS}
  sysutils,math,
{$ELSE}
  fksysutl,
{$ENDIF}
  verbose,comphook,systems,
  cutils,cfileutl,cclasses,globals,options,fmodule,parser,symtable,
  assemble,link,dbgbase,import,export,tokens,pass_1,wpobase,wpo
  { cpu parameter handling }
  ,cpupara
  { procinfo stuff }
  ,cpupi
  { cpu codegenerator }
  ,cgcpu
{$ifndef NOPASS2}
  ,cpunode
{$endif}
  { cpu targets }
  ,cputarg
{$ifdef llvm}
  ,llvmtarg
{$endif llvm}
  { system information for source system }
  { the information about the target os  }
  { are pulled in by the t_* units       }
{$ifdef amiga}
  ,i_amiga
{$endif amiga}
{$ifdef android}
  ,i_android
{$endif android}
{$ifdef aros}
  ,i_aros
{$endif}
{$ifdef atari}
  ,i_atari
{$endif atari}
{$ifdef beos}
  ,i_beos
{$endif beos}
{$ifdef fbsd}
  ,i_fbsd
{$endif fbsd}
{$ifdef gba}
  ,i_gba
{$endif gba}
{$ifdef go32v2}
  ,i_go32v2
{$endif go32v2}
{$ifdef linux}
  ,i_linux
{$endif linux}
{$ifdef macos}
  ,i_macos
{$endif macos}
{$ifdef nds}
  ,i_nds
{$endif nds}
{$ifdef nwm}
  ,i_nwm
{$endif nwm}
{$ifdef nwl}
  ,i_nwl
{$endif nwm}
{$ifdef os2}
 {$ifdef emx}
  ,i_emx
 {$else emx}
  ,i_os2
 {$endif emx}
{$endif os2}
{$ifdef palmos}
  ,i_palmos
{$endif palmos}
{$ifdef solaris}
  ,i_sunos
{$endif solaris}
{$ifdef wdosx}
  ,i_wdosx
{$endif wdosx}
{$ifdef wii}
  ,i_wii
{$endif wii}
{$ifdef win32}
  ,i_win
{$endif win32}
{$ifdef symbian}
  ,i_symbian
{$endif symbian}
<<<<<<< HEAD
{$ifdef nativent}
  ,i_nativent
{$endif nativent}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef aix}
  ,i_aix
{$endif aix}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  ,globtype;

function Compile(const cmd:TCmdStr):longint;
=======
  ;

function Compile(const cmd:string):longint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

uses
  aasmcpu;

{$if defined(EXTDEBUG) or defined(MEMDEBUG)}
  {$define SHOWUSEDMEM}
{$endif}

var
  CompilerInitedAfterArgs,
  CompilerInited : boolean;


{****************************************************************************
                                Compiler
****************************************************************************}

procedure DoneCompiler;
begin
  if not CompilerInited then
   exit;
{ Free compiler if args are read }
  if CompilerInitedAfterArgs then
   begin
     CompilerInitedAfterArgs:=false;
     DoneParser;
     DoneImport;
     DoneExport;
     DoneLinker;
     DoneAsm;
     DoneWpo;
   end;
{ Free memory for the others }
  CompilerInited:=false;
  do_doneSymbolInfo;
  DoneSymtable;
  DoneGlobals;
  DoneFileUtils;
  donetokens;
end;


procedure InitCompiler(const cmd:TCmdStr);
begin
  if CompilerInited then
   DoneCompiler;
{$if defined(unix) and (FPC_FULLVERSION>20700)}
  { Set default code page for ansistrings on unix-like systems }
  DefaultSystemCodePage:=GetSystemCodePage;
{$endif}
{ inits which need to be done before the arguments are parsed }
  InitSystems;
  { fileutils depends on source_info so it must be after systems }
  InitFileUtils;
  { globals depends on source_info so it must be after systems }
  InitGlobals;
  { verbose depends on exe_path and must be after globals }
  InitVerbose;
  inittokens;
  IniTSymtable; {Must come before read_arguments, to enable macrosymstack}
  do_initSymbolInfo;
  CompilerInited:=true;
{ this is needed here for the IDE
  in case of compilation failure
  at the previous compile }
  set_current_module(nil);
{ read the arguments }
  read_arguments(cmd);
{ inits which depend on arguments }
  InitParser;
  InitImport;
  InitExport;
  InitLinker;
  InitAsm;
  InitWpo;

  CompilerInitedAfterArgs:=true;
end;


function Compile(const cmd:TCmdStr):longint;

{$maxfpuregisters 0}

  procedure writepathlist(w:longint;l:TSearchPathList);
  var
    hp : TCmdStrListItem;
  begin
    hp:=TCmdStrListItem(l.first);
    while assigned(hp) do
     begin
       Message1(w,hp.str);
       hp:=TCmdStrListItem(hp.next);
     end;
  end;

var
  timestr    : string[20];
  linkstr    : string[64];
{$ifdef SHOWUSEDMEM}
  hstatus : TFPCHeapStatus;
{$endif SHOWUSEDMEM}
  ExceptionMask : TFPUExceptionMask;
  totaltime : real;
begin
  try
    try
       ExceptionMask:=GetExceptionMask;
       SetExceptionMask([exInvalidOp, exDenormalized, exZeroDivide,
                         exOverflow, exUnderflow, exPrecision]);

       starttime:=getrealtime;

       { Initialize the compiler }
       InitCompiler(cmd);

       { show some info }
       Message1(general_t_compilername,FixFileName(system.paramstr(0)));
       Message1(general_d_sourceos,source_info.name);
       Message1(general_i_targetos,target_info.name);
       Message1(general_t_exepath,exepath);
       WritePathList(general_t_unitpath,unitsearchpath);
       WritePathList(general_t_includepath,includesearchpath);
       WritePathList(general_t_librarypath,librarysearchpath);
       WritePathList(general_t_objectpath,objectsearchpath);

       { Compile the program }
  {$ifdef PREPROCWRITE}
       if parapreprocess then
        parser.preprocess(inputfilepath+inputfilename)
       else
  {$endif PREPROCWRITE}
        parser.compile(inputfilepath+inputfilename);

       { Show statistics }
       if status.errorcount=0 then
        begin
          totaltime:=getrealtime-starttime;
          if totaltime<0 then
            totaltime:=totaltime+3600.0*24.0;
          if round(frac(totaltime)*10) >= 10 then
            totaltime:=trunc(totaltime) + 1;
          timestr:=tostr(trunc(totaltime))+'.'+tostr(round(frac(totaltime)*10));
          if status.codesize<>aword(-1) then
            linkstr:=', '+tostr(status.codesize)+' ' +strpas(MessagePChar(general_text_bytes_code))+', '+tostr(status.datasize)+' '+strpas(MessagePChar(general_text_bytes_data))
          else
            linkstr:='';
          Message3(general_i_abslines_compiled,tostr(status.compiledlines),timestr,linkstr);
          if (Status.Verbosity and V_Warning = V_Warning) and
                                               (Status.CountWarnings <> 0) then
           Message1 (general_i_number_of_warnings, tostr (Status.CountWarnings));
          if (Status.Verbosity and V_Hint = V_Hint) and
                                                  (Status.CountHints <> 0) then
           Message1 (general_i_number_of_hints, tostr (Status.CountHints));
          if (Status.Verbosity and V_Note = V_Note) and
                                               (Status.CountNotes <> 0) then
           Message1 (general_i_number_of_notes, tostr (Status.CountNotes));
        end;
     finally
       { no message possible after this !!    }
       DoneCompiler;

       SetExceptionMask(ExceptionMask);
     end;
     DoneVerbose;
  except
    on EControlCAbort do
      begin
        try
          { in case of 50 errors, this could cause another exception,
            suppress this exception
          }
          Message(general_f_compilation_aborted);
        except
          on ECompilerAbort do
            ;
        end;
        DoneVerbose;
      end;
    on ECompilerAbort do
      begin
        try
          { in case of 50 errors, this could cause another exception,
            suppress this exception
          }
          Message(general_f_compilation_aborted);
        except
          on ECompilerAbort do
            ;
        end;
        DoneVerbose;
      end;
    on ECompilerAbortSilent do
      begin
        DoneVerbose;
      end;
    on EOutOfMemory do
      begin
        try
          Message(general_f_no_memory_left);
        except
          on ECompilerAbort do
            ;
        end;
        DoneVerbose;
      end;
    on e : EInOutError do
      begin
        try
          Message1(general_f_ioerror,e.message);
        except
          on ECompilerAbort do
            ;
        end;
        DoneVerbose;
      end;
    on e : EOSError do
      begin
        try
          Message1(general_f_oserror,e.message);
        except
          on ECompilerAbort do
            ;
        end;
        DoneVerbose;
      end;
    on Exception do
      begin
        { General catchall, normally not used }
        try
          { in case of 50 errors, this could cause another exception,
            suppress this exception
          }
          if not exception_raised then
            begin
              exception_raised:=true;
              Message(general_e_exception_raised);
            end
          else
            Message(general_f_compilation_aborted);
        except
          on ECompilerAbort do
            ;
        end;
        DoneVerbose;
        Raise;
      end;
  end;
{$ifdef SHOWUSEDMEM}
      hstatus:=GetFPCHeapStatus;
      Writeln('Max Memory used/heapsize: ',DStr(hstatus.MaxHeapUsed shr 10),'/',DStr(hstatus.MaxHeapSize shr 10),' Kb');
{$endif SHOWUSEDMEM}

  { Set the return value if an error has occurred }
  if status.errorcount=0 then
    result:=0
  else
    result:=1;
end;

end.
