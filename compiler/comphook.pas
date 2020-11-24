{
    Copyright (c) 1998-2002 by Peter Vreman

    This unit handles the compilerhooks for output to external programs

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
unit comphook;

{$i fpcdefs.inc}

interface

uses
{$IFNDEF USE_FAKE_SYSUTILS}
  sysutils,
{$ELSE}
  fksysutl,
{$ENDIF}
  globtype,
  finput;

Const
  { Levels }
  V_None         = $0;
  V_Fatal        = $1;
  V_Error        = $2;
  V_Normal       = $4; { doesn't show a text like Error: }
  V_Warning      = $8;
  V_Note         = $10;
  V_Hint         = $20;
  V_LineInfoMask = $fff;
  { From here by default no line info }
  V_Info         = $1000;
  V_Status       = $2000;
  V_Used         = $4000;
  V_Tried        = $8000;
  V_Conditional  = $10000;
  V_Debug        = $20000;
  V_Executable   = $40000;
  V_TimeStamps   = $80000;
  V_LevelMask    = $fffffff;
  V_All          = V_LevelMask;
  V_Default      = V_Fatal + V_Error + V_Normal;
  { Flags }
  V_LineInfo     = $10000000;

const
  { RHIDE expect gcc like error output }
  fatalstr      : string[20] = 'Fatal:';
  errorstr      : string[20] = 'Error:';
  warningstr    : string[20] = 'Warning:';
  notestr       : string[20] = 'Note:';
  hintstr       : string[20] = 'Hint:';

type
  PCompilerStatus = ^TCompilerStatus;
  TCompilerStatus = record
  { Current status }
    currentmodule,
    currentsourceppufilename, { the name of the ppu where the source file
                                comes from where the error location is given }
    currentsourcepath,
    currentsource : string;   { filename }
    currentline,
    currentcolumn : longint;  { current line and column }
    currentmodulestate : string[20];
  { Total Status }
    compiledlines : longint;  { the number of lines which are compiled }
    errorcount,               { this field should never be increased directly,
                                use Verbose.GenerateError procedure to do this,
                                this allows easier error catching using GDB by
                                adding a single breakpoint at this procedure }
    countWarnings,
    countNotes,
    countHints    : longint;  { number of found errors/warnings/notes/hints }
    codesize,
    datasize      : qword;
  { program info }
    isexe,
    ispackage,
    islibrary     : boolean;
  { Settings for the output }
    showmsgnrs    : boolean;
    verbosity     : longint;
    maxerrorcount : longint;
    errorwarning,
    errornote,
    errorhint,
    skip_error,
    use_stderr,
    use_redir,
    use_bugreport,
    use_gccoutput,
    sources_avail,
    print_source_path : boolean;
  { Redirection support }
    redirfile : text;
  { Special file for bug report }
    reportbugfile : text;
  end;

type
  EControlCAbort=class(Exception)
    constructor Create;
  end;
  ECompilerAbort=class(Exception)
    constructor Create;
  end;
  ECompilerAbortSilent=class(Exception)
    constructor Create;
  end;

var
  status : tcompilerstatus;

{ Default Functions }
Function  def_status:boolean;
Function  def_comment(Level:Longint;const s:ansistring):boolean;
function  def_internalerror(i:longint):boolean;
function  def_CheckVerbosity(v:longint):boolean;
procedure def_initsymbolinfo;
procedure def_donesymbolinfo;
procedure def_extractsymbolinfo;
function  def_openinputfile(const filename: TPathStr): tinputfile;
Function  def_getnamedfiletime(Const F : TPathStr) : Longint;
{ Function redirecting for IDE support }
type
  tstopprocedure         = procedure(err:longint);
  tstatusfunction        = function:boolean;
  tcommentfunction       = function(Level:Longint;const s:ansistring):boolean;
  tinternalerrorfunction = function(i:longint):boolean;
  tcheckverbosityfunction = function(i:longint):boolean;

  tinitsymbolinfoproc = procedure;
  tdonesymbolinfoproc = procedure;
  textractsymbolinfoproc = procedure;
  topeninputfilefunc = function(const filename: TPathStr): tinputfile;
  tgetnamedfiletimefunc = function(const filename: TPathStr): longint;

const
  do_status        : tstatusfunction  = @def_status;
  do_comment       : tcommentfunction = @def_comment;
  do_internalerror : tinternalerrorfunction = @def_internalerror;
  do_checkverbosity : tcheckverbosityfunction = @def_checkverbosity;

  do_initsymbolinfo : tinitsymbolinfoproc = @def_initsymbolinfo;
  do_donesymbolinfo : tdonesymbolinfoproc = @def_donesymbolinfo;
  do_extractsymbolinfo : textractsymbolinfoproc = @def_extractsymbolinfo;
  needsymbolinfo : boolean =false;

  do_openinputfile : topeninputfilefunc = @def_openinputfile;
  do_getnamedfiletime : tgetnamedfiletimefunc = @def_getnamedfiletime;

implementation

  uses
   cutils, systems, globals, comptty;

{****************************************************************************
                          Helper Routines
****************************************************************************}

function gccfilename(const s : string) : string;
var
  i : longint;
begin
  for i:=1to length(s) do
   begin
     case s[i] of
      '\' : gccfilename[i]:='/';
 'A'..'Z' : if not (tf_files_case_aware in source_info.flags) and
               not (tf_files_case_sensitive in source_info.flags) then
              gccfilename[i]:=chr(ord(s[i])+32)
            else
              gccfilename[i]:=s[i];
     else
      gccfilename[i]:=s[i];
     end;
   end;
  gccfilename[0]:=s[0];
end;


function tostr(i : longint) : string;
var
  hs : string;
begin
  str(i,hs);
  tostr:=hs;
end;

type
  TOutputColor = (oc_black,oc_red,oc_green,oc_orange,og_blue,oc_magenta,oc_cyan,oc_lightgray);

procedure WriteColoredOutput(var t: Text;color: TOutputColor;const s : AnsiString);
  begin
     if TTYCheckSupported and IsATTY(t) then
       begin
         case color of
           oc_black:
             write(t,#27'[1m'#27'[30m');
           oc_red:
             write(t,#27'[1m'#27'[31m');
           oc_green:
             write(t,#27'[1m'#27'[32m');
           oc_orange:
             write(t,#27'[1m'#27'[33m');
           og_blue:
             write(t,#27'[1m'#27'[34m');
           oc_magenta:
             write(t,#27'[1m'#27'[35m');
           oc_cyan:
             write(t,#27'[1m'#27'[36m');
           oc_lightgray:
             write(t,#27'[1m'#27'[37m');
         end;
       end;
    write(t,s);
    if TTYCheckSupported and IsATTY(t) then
      write(t,#27'[0m');
  end;
{****************************************************************************
                          Stopping the compiler
****************************************************************************}

constructor EControlCAbort.Create;
  begin
    inherited Create('Ctrl-C Signaled!');
  end;


constructor ECompilerAbort.Create;
  begin
    inherited Create('Compilation Aborted');
  end;


constructor ECompilerAbortSilent.Create;
  begin
    inherited Create('Compilation Aborted');
  end;


{****************************************************************************
                         Predefined default Handlers
****************************************************************************}

function def_status:boolean;
var
  hstatus : TFPCHeapStatus;
begin
  def_status:=false; { never stop }
{ Status info?, Called every line }
  if ((status.verbosity and V_Status)<>0) then
   begin
     if (status.compiledlines=1) or
        (status.currentline mod 100=0) then
       begin
         if status.currentline>0 then
           Write(status.currentline,' ');
         hstatus:=GetFPCHeapStatus;
         WriteLn(DStr(hstatus.CurrHeapUsed shr 10),'/',DStr(hstatus.CurrHeapSize shr 10),' Kb Used');
       end;
   end;
{$ifdef macos}
  Yield;
{$endif}
end;


Function def_comment(Level:Longint;const s:ansistring):boolean;
const
  rh_errorstr   = 'error:';
  rh_warningstr = 'warning:';

  procedure WriteMsgTypeColored(var t : text;const s : AnsiString);
    begin
      case (status.verbosity and Level) of
        V_Warning:
          WriteColoredOutput(t,oc_magenta,s);
        V_Error,
        V_Fatal:
          WriteColoredOutput(t,oc_red,s);
        else
          write(t,s);
      end;
    end;

var
  hs2,
  MsgTypeStr,
  MsgLocStr,
  MsgTimeStr: AnsiString;
begin
  def_comment:=false; { never stop }
  MsgTypeStr:='';
  MsgLocStr:='';
  MsgTimeStr:='';
  if not(status.use_gccoutput) then
    begin
      if (status.verbosity and Level)=V_Hint then
        MsgTypeStr:=hintstr;
      if (status.verbosity and Level)=V_Note then
        MsgTypeStr:=notestr;
      if (status.verbosity and Level)=V_Warning then
        MsgTypeStr:=warningstr;
      if (status.verbosity and Level)=V_Error then
        MsgTypeStr:=errorstr;
      if (status.verbosity and Level)=V_Fatal then
        MsgTypeStr:=fatalstr;
      if (status.verbosity and Level)=V_Used then
        MsgTypeStr:=PadSpace('('+status.currentmodule+')',10);
    end
  else
    begin
      if (status.verbosity and Level)=V_Hint then
        MsgTypeStr:=rh_warningstr;
      if (status.verbosity and Level)=V_Note then
        MsgTypeStr:=rh_warningstr;
      if (status.verbosity and Level)=V_Warning then
        MsgTypeStr:=rh_warningstr;
      if (status.verbosity and Level)=V_Error then
        MsgTypeStr:=rh_errorstr;
      if (status.verbosity and Level)=V_Fatal then
        MsgTypeStr:=rh_errorstr;
    end;
  { Generate line prefix }
  if ((Level and V_LineInfo)=V_LineInfo) and
     (status.currentsource<>'') and
     (status.currentline>0) then
    begin
{$ifndef macos}
      { Adding the column should not confuse RHIDE,
      even if it does not yet use it PM
      but only if it is after error or warning !! PM }
      if status.currentcolumn>0 then
        begin
          if status.use_gccoutput then
            MsgLocStr:=gccfilename(status.currentsource)+':'+tostr(status.currentline)+':'+tostr(status.currentcolumn)+':'
          else
            MsgLocStr:=status.currentsource+'('+tostr(status.currentline)+','+tostr(status.currentcolumn)+')';
          if status.print_source_path then
            if status.sources_avail then
              MsgLocStr:=status.currentsourcepath+MsgLocStr
            else
              MsgLocStr:=status.currentsourceppufilename+':'+MsgLocStr;
        end
      else
        begin
          if status.use_gccoutput then
            MsgLocStr:=gccfilename(status.currentsource)+':'+tostr(status.currentline)+':'
          else
            MsgLocStr:=status.currentsource+'('+tostr(status.currentline)+')';
        end;
 {$else macos}
      { MPW style error }
      if status.currentcolumn>0 then
        MsgLocStr:='File "'+status.currentsourcepath+status.currentsource+'"; Line '+tostr(status.currentline)+' #[' + tostr(status.currentcolumn) + ']'
      else
        MsgLocStr:='File "'+status.currentsourcepath+status.currentsource+'"; Line '+tostr(status.currentline)+' # ';
 {$endif macos}
    end;
  if MsgLocStr<>'' then
    MsgLocStr:=MsgLocStr+' ';
  if MsgTypeStr<>'' then
    MsgTypeStr:=MsgTypeStr+' ';
  if (status.verbosity and V_TimeStamps)<>0 then
    begin
      system.str(getrealtime-starttime:0:3,hs2);
      MsgTimeStr:='['+hs2+'] ';
    end;

  { Display line }
  if (Level<>V_None) and
     ((status.verbosity and (Level and V_LevelMask))=(Level and V_LevelMask)) then
   begin
     if status.use_stderr then
       begin
         write(StdErr,MsgTimeStr+MsgLocStr);
         WriteMsgTypeColored(StdErr,MsgTypeStr);
         writeln(StdErr,s);
         flush(StdErr);
       end
     else
       begin
         if status.use_redir then
           writeln(status.redirfile,MsgTimeStr+MsgLocStr+MsgTypeStr+s)
         else
           begin
             write(MsgTimeStr+MsgLocStr);
             WriteMsgTypeColored(Output,MsgTypeStr);
             writeln(s);
           end;
       end;
   end;
  { include everything in the bugreport file }
  if status.use_bugreport then
    begin
      Write(status.reportbugfile,hexstr(level,8)+':');
      Writeln(status.reportbugfile,MsgTimeStr+MsgLocStr+MsgTypeStr+s);
    end;
end;


function def_internalerror(i : longint) : boolean;
begin
  do_comment(V_Fatal+V_LineInfo,'Internal error '+tostr(i));
{$ifdef EXTDEBUG}
  { Internalerror() and def_internalerror() do not
    have a stackframe }
  dump_stack(stdout,get_caller_frame(get_frame));
{$endif EXTDEBUG}
  def_internalerror:=true;
end;

function def_CheckVerbosity(v:longint):boolean;
begin
  result:=status.use_bugreport or
          ((v<>V_None) and
           ((status.verbosity and (v and V_LevelMask))=(v and V_LevelMask)));
end;

procedure def_initsymbolinfo;
begin
end;

procedure def_donesymbolinfo;
begin
end;

procedure def_extractsymbolinfo;
begin
end;

function  def_openinputfile(const filename: TPathStr): tinputfile;
begin
  def_openinputfile:=tdosinputfile.create(filename);
end;


Function def_GetNamedFileTime (Const F : TPathStr) : Longint;
begin
  Result:=FileAge(F);
end;

end.
