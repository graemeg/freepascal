{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000 by Peter Vreman

    Stabs Line Info Retriever

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
  This unit should not be compiled in objfpc mode, since this would make it
  dependent on objpas unit.
}
unit lineinfo;
interface

{$S-}
{$Q-}

<<<<<<< HEAD
<<<<<<< HEAD
{$IF FPC_VERSION<3}
Type 
  CodePointer = Pointer;
{$ENDIF}

function GetLineInfo(addr:ptruint;var func,source:string;var line:longint) : boolean;
function StabBackTraceStr(addr:CodePointer):string;
procedure CloseStabs;
=======
function GetLineInfo(addr:ptruint;var func,source:string;var line:longint) : boolean;
>>>>>>> graemeg/fixes_2_2
=======
function GetLineInfo(addr:ptruint;var func,source:string;var line:longint) : boolean;
>>>>>>> origin/fixes_2_2

implementation

uses
  exeinfo,strings;

const
  N_Function    = $24;
  N_TextLine    = $44;
  N_DataLine    = $46;
  N_BssLine     = $48;
  N_SourceFile  = $64;
  N_IncludeFile = $84;

  maxstabs = 40; { size of the stabs buffer }

var
  { GDB after 4.18 uses offset to function begin
    in text section but OS/2 version still uses 4.16 PM }
  StabsFunctionRelative: boolean;

type
  pstab=^tstab;
  tstab=packed record
    strpos  : longint;
    ntype   : byte;
    nother  : byte;
    ndesc   : word;
    nvalue  : dword;
  end;

{ We use static variable so almost no stack is required, and is thus
  more safe when an error has occured in the program }
{$WARNING This code is not thread-safe, and needs improvement }  
var
  e          : TExeFile;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  staberr    : boolean;
>>>>>>> graemeg/fixes_2_2
=======
  staberr    : boolean;
>>>>>>> origin/fixes_2_2
  stabcnt,              { amount of stabs }
  stablen,
  stabofs,              { absolute stab section offset in executable }
  stabstrlen,
  stabstrofs : longint; { absolute stabstr section offset in executable }
  dirlength  : longint; { length of the dirctory part of the source file }
  stabs      : array[0..maxstabs-1] of tstab;  { buffer }
  funcstab,             { stab with current function info }
  linestab,             { stab with current line info }
  dirstab,              { stab with current directory info }
  filestab   : tstab;   { stab with current file info }
  filename,
<<<<<<< HEAD
<<<<<<< HEAD
  lastfilename,         { store last processed file }
  dbgfn : string;
  lastopenstabs: Boolean; { store last result of processing a file }


function OpenStabs(addr : pointer) : boolean;
  var
    baseaddr : pointer;
begin
  // False by default
  OpenStabs:=false;

  // Empty so can test if GetModuleByAddr has worked
  filename := '';

  // Get filename by address using GetModuleByAddr
  GetModuleByAddr(addr,baseaddr,filename);
{$ifdef DEBUG_LINEINFO}
  writeln(stderr,filename,' Baseaddr: ',hexstr(ptruint(baseaddr),sizeof(baseaddr)*2));
{$endif DEBUG_LINEINFO}

  // Check if GetModuleByAddr has worked
  if filename = '' then
    exit;

  // If target filename same as previous, then re-use previous result
  if filename = lastfilename then
  begin
    OpenStabs:=lastopenstabs;
    exit;
  end;

  // Close previously opened stabs
  CloseStabs;

  // Reset last open stabs result
  lastopenstabs := false;

  // Save newly processed filename
  lastfilename := filename;

  // Open exe file or debug link
  if not OpenExeFile(e,filename) then
    exit;
  if ReadDebugLink(e,dbgfn) then
    begin
      CloseExeFile(e);
      if not OpenExeFile(e,dbgfn) then
        exit;
    end;

  // Find stab section
{$ifdef BeOS}
  { Do not change ProcessAddress field for BeOS/Haiku
    if baseAddr is lower than ProcessAdress }
  if ptruint(baseaddr)>ptruint(e.processaddress) then
{$endif BeOS}
    e.processaddress:=ptruint(baseaddr)-e.processaddress;
  StabsFunctionRelative := E.FunctionRelative;
  if FindExeSection(e,'.stab',stabofs,stablen) and
     FindExeSection(e,'.stabstr',stabstrofs,stabstrlen) then
    begin
      stabcnt:=stablen div sizeof(tstab);
      lastopenstabs:=true;
      OpenStabs:=true;
    end
  else
    begin
      CloseExeFile(e);
=======
  dbgfn : string;

=======
  dbgfn : string;

>>>>>>> origin/fixes_2_2

function OpenStabs(addr : pointer) : boolean;
  var
    baseaddr : pointer;
begin
  OpenStabs:=false;
  if staberr then
    exit;

  GetModuleByAddr(addr,baseaddr,filename);
{$ifdef DEBUG_LINEINFO}
  writeln(stderr,filename,' Baseaddr: ',hexstr(ptruint(baseaddr),sizeof(baseaddr)*2));
{$endif DEBUG_LINEINFO}

  if not OpenExeFile(e,filename) then
    exit;
  if ReadDebugLink(e,dbgfn) then
    begin
      CloseExeFile(e);
      if not OpenExeFile(e,dbgfn) then
        exit;
    end;
  e.processaddress:=e.processaddress+dword(baseaddr);
  StabsFunctionRelative := E.FunctionRelative;
  if FindExeSection(e,'.stab',stabofs,stablen) and
     FindExeSection(e,'.stabstr',stabstrofs,stabstrlen) then
    begin
      stabcnt:=stablen div sizeof(tstab);
      OpenStabs:=true;
    end
  else
    begin
      staberr:=true;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      exit;
    end;
end;


procedure CloseStabs;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if e.isopen then
  begin
    CloseExeFile(e);

    // Reset last processed filename
    lastfilename := '';
  end;
end;


=======
  CloseExeFile(e);
end;


>>>>>>> graemeg/fixes_2_2
=======
  CloseExeFile(e);
end;


>>>>>>> origin/fixes_2_2
function GetLineInfo(addr:ptruint;var func,source:string;var line:longint) : boolean;
var
  res,
  stabsleft,
  stabscnt,i : longint;
  found : boolean;
  lastfunc : tstab;
begin
  GetLineInfo:=false;
{$ifdef DEBUG_LINEINFO}
  writeln(stderr,'GetLineInfo called');
{$endif DEBUG_LINEINFO}
  fillchar(func,high(func)+1,0);
  fillchar(source,high(source)+1,0);
  line:=0;
<<<<<<< HEAD
<<<<<<< HEAD

  if not OpenStabs(pointer(addr)) then
    exit;

  { correct the value to the correct address in the file }
  { processaddress is set in OpenStabs                   }
  addr := dword(addr - e.processaddress);
=======
=======
>>>>>>> origin/fixes_2_2
  if staberr then
    exit;
  if not e.isopen then
   begin
     if not OpenStabs(pointer(addr)) then
      exit;
   end;

  { correct the value to the correct address in the file }
  { processaddress is set in OpenStabs                   }
  addr := addr - e.processaddress;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifdef DEBUG_LINEINFO}
  writeln(stderr,'Addr: ',hexstr(addr,sizeof(addr)*2));
{$endif DEBUG_LINEINFO}

  fillchar(funcstab,sizeof(tstab),0);
  fillchar(filestab,sizeof(tstab),0);
  fillchar(dirstab,sizeof(tstab),0);
  fillchar(linestab,sizeof(tstab),0);
  fillchar(lastfunc,sizeof(tstab),0);
  found:=false;
  seek(e.f,stabofs);
  stabsleft:=stabcnt;
  repeat
    if stabsleft>maxstabs then
     stabscnt:=maxstabs
    else
     stabscnt:=stabsleft;
    blockread(e.f,stabs,stabscnt*sizeof(tstab),res);
    stabscnt:=res div sizeof(tstab);
    for i:=0 to stabscnt-1 do
     begin
       case stabs[i].ntype of
         N_BssLine,
         N_DataLine,
         N_TextLine :
           begin
             if (stabs[i].ntype=N_TextLine) and StabsFunctionRelative then
               inc(stabs[i].nvalue,lastfunc.nvalue);
             if (stabs[i].nvalue<=addr) and
                (stabs[i].nvalue>linestab.nvalue) then
              begin
                { if it's equal we can stop and take the last info }
                if stabs[i].nvalue=addr then
                 found:=true
                else
                 linestab:=stabs[i];
              end;
           end;
         N_Function :
           begin
             lastfunc:=stabs[i];
             if (stabs[i].nvalue<=addr) and
                (stabs[i].nvalue>funcstab.nvalue) then
              begin
                funcstab:=stabs[i];
                fillchar(linestab,sizeof(tstab),0);
              end;
           end;
         N_SourceFile,
         N_IncludeFile :
           begin
             if (stabs[i].nvalue<=addr) and
                (stabs[i].nvalue>=filestab.nvalue) then
              begin
                { if same value and type then the first one
                  contained the directory PM }
                if (stabs[i].nvalue=filestab.nvalue) and
                   (stabs[i].ntype=filestab.ntype) then
                  dirstab:=filestab
                else
                  fillchar(dirstab,sizeof(tstab),0);
                filestab:=stabs[i];
                fillchar(linestab,sizeof(tstab),0);
                { if new file then func is not valid anymore PM }
                if stabs[i].ntype=N_SourceFile then
                  begin
                    fillchar(funcstab,sizeof(tstab),0);
                    fillchar(lastfunc,sizeof(tstab),0);
                  end;
              end;
           end;
       end;
     end;
    dec(stabsleft,stabscnt);
  until found or (stabsleft=0);

{ get the line,source,function info }
  line:=linestab.ndesc;
  if dirstab.ntype<>0 then
   begin
     seek(e.f,stabstrofs+dirstab.strpos);
     blockread(e.f,source[1],high(source)-1,res);
     dirlength:=strlen(@source[1]);
     source[0]:=chr(dirlength);
   end
  else
   dirlength:=0;
  if filestab.ntype<>0 then
   begin
     seek(e.f,stabstrofs+filestab.strpos);
     blockread(e.f,source[dirlength+1],high(source)-(dirlength+1),res);
     source[0]:=chr(strlen(@source[1]));
   end;
  if funcstab.ntype<>0 then
   begin
     seek(e.f,stabstrofs+funcstab.strpos);
     blockread(e.f,func[1],high(func)-1,res);
     func[0]:=chr(strlen(@func[1]));
     i:=pos(':',func);
     if i>0 then
      Delete(func,i,255);
   end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  if e.isopen then
    CloseStabs;
>>>>>>> graemeg/fixes_2_2
=======
  if e.isopen then
    CloseStabs;
>>>>>>> origin/fixes_2_2
  GetLineInfo:=true;
end;


function StabBackTraceStr(addr:CodePointer):string;
var
  func,
  source : string;
  hs     : string;
  line   : longint;
  Store  : TBackTraceStrFunc;
  Success : boolean;
begin
{$ifdef DEBUG_LINEINFO}
  writeln(stderr,'StabBackTraceStr called');
{$endif DEBUG_LINEINFO}
  { reset to prevent infinite recursion if problems inside the code PM }
  Success:=false;
  Store:=BackTraceStrFunc;
  BackTraceStrFunc:=@SysBackTraceStr;
  Success:=GetLineInfo(ptruint(addr),func,source,line);
{ create string }
{$ifdef netware}
  { we need addr relative to code start on netware }
  dec(addr,ptruint(system.NWGetCodeStart));
  StabBackTraceStr:='  CodeStart + $'+HexStr(ptruint(addr),sizeof(ptruint)*2);
{$else}
  StabBackTraceStr:='  $'+HexStr(ptruint(addr),sizeof(ptruint)*2);
{$endif}
  if func<>'' then
    StabBackTraceStr:=StabBackTraceStr+'  '+func;
  if source<>'' then
   begin
     if func<>'' then
      StabBackTraceStr:=StabBackTraceStr+', ';
     if line<>0 then
      begin
        str(line,hs);
        StabBackTraceStr:=StabBackTraceStr+' line '+hs;
      end;
     StabBackTraceStr:=StabBackTraceStr+' of '+source;
   end;
<<<<<<< HEAD
<<<<<<< HEAD
  BackTraceStrFunc:=Store;
=======
=======
>>>>>>> origin/fixes_2_2
  if Success then
    BackTraceStrFunc:=Store;
>>>>>>> graemeg/fixes_2_2
end;


initialization
  lastfilename := '';
  lastopenstabs := false;
  BackTraceStrFunc:=@StabBackTraceStr;

finalization
<<<<<<< HEAD
<<<<<<< HEAD
  CloseStabs;

=======
  if e.isopen then
   CloseStabs;
>>>>>>> graemeg/fixes_2_2
=======
  if e.isopen then
   CloseStabs;
>>>>>>> origin/fixes_2_2
end.
