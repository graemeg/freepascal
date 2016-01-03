{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Carl Eric Codere
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$define ATARI}
unit system;

{--------------------------------------------------------------------}
{ LEFT TO DO:                                                        }
{--------------------------------------------------------------------}
{ o SBrk                                                             }
{ o Implement truncate                                               }
{ o Implement paramstr(0)                                            }
{--------------------------------------------------------------------}


{$I os.inc}

  interface

    {$I systemh.inc}

type
 THandle = longint;

    {$I heaph.inc}

{Platform specific information}
const
 LineEnding = #10;
 LFNSupport = true;
 CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)
 DirectorySeparator = '/';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [':'];
 FileNameCaseSensitive = false;
 maxExitCode = 255;
 MaxPathLen = 255;
 AllFilesMask = '*';

 sLineBreak: string [1] = LineEnding;
    { used for single computations }
    const BIAS4 = $7f-1;

const
  UnusedHandle    = $ffff;
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = $ffff;



  implementation

    {$I system.inc}
    {$I lowmath.inc}


function GetProcessID:SizeUInt;
begin
{$WARNING To be checked by platform maintainer}
 GetProcessID := 1;
end;

    const
      argc : longint = 0;


    var
      errno : integer;

{$S-}
    procedure Stack_Check; assembler;
    { Check for local variable allocation }
    { On Entry -> d0 : size of local stack we are trying to allocate }
         asm
          XDEF STACKCHECK
           move.l  sp,d1            { get value of stack pointer            }
           sub.l   d0,d1            {  sp - stack_size                      }
           sub.l   #2048,d1
           cmp.l   __BREAK,d1
           bgt     @st1nosweat
           move.l  #202,d0
           jsr     HALT_ERROR
         @st1nosweat:
         end;


    Procedure Error2InOut;
    Begin
     if (errno <= -2) and (errno >= -11) then
       InOutRes:=150-errno  { 150+errno }
     else
      Begin
        case errno of
          -32 : InOutRes:=1;
          -33 : InOutRes:=2;
          -34 : InOutRes:=3;
          -35 : InOutRes:=4;
          -36 : InOutRes:=5;
          -37 : InOutRes:=8;
          -39 : InOutRes:=8;
          -40 : InOutRes:=9;
          -46 : InOutRes:=15;
          -67..-64 : InOutRes:=153;
          -15 : InOutRes:=151;
          -13 : InOutRes:=150;
        else
           InOutres := word(errno);
         end;
     end;
     errno:=0;
    end;



    procedure halt(errnum : byte);

      begin
         do_exit;
         flush(stderr);
         asm
            clr.l   d0
            move.b  errnum,d0
            move.w  d0,-(sp)
            move.w  #$4c,-(sp)
            trap    #1
         end;
      end;


      function args : pointer; assembler;
      asm
         move.l __ARGS,d0
      end;




   Function GetParamCount(const p: pchar): longint;
   var
    i: word;
    count: word;
   Begin
    i:=0;
    count:=0;
    while p[count] <> #0 do
     Begin
       if (p[count] <> ' ') and (p[count] <> #9) and (p[count] <> #0) then
       Begin
          i:=i+1;
          while (p[count] <> ' ') and (p[count] <> #9) and (p[count] <> #0) do
           count:=count+1;
       end;
       if p[count] = #0 then break;
       count:=count+1;
     end;
     GetParamCount:=longint(i);
   end;


   Function GetParam(index: word; const p : pchar): string;
   { On Entry: index = string index to correct parameter  }
   { On exit:  = correct character index into pchar array }
   { Returns correct index to command line argument }
   var
    count: word;
    localindex: word;
    l: byte;
    temp: string;
   Begin
     temp:='';
     count := 0;
     { first index is one }
     localindex := 1;
     l:=0;
     While p[count] <> #0 do
       Begin
         if (p[count] <> ' ') and (p[count] <> #9) then
           Begin
             if localindex = index then
              Begin
               while (p[count] <> #0) and (p[count] <> ' ') and (p[count] <> #9) and (l < 256) do
                Begin
                  temp:=temp+p[count];
                  l:=l+1;
                  count:=count+1;
                end;
                temp[0]:=char(l);
                GetParam:=temp;
                exit;
              end;
             { Point to next argument in list }
             while (p[count] <> #0) and (p[count] <> ' ') and (p[count] <> #9) do
               Begin
                 count:=count+1;
               end;
             localindex:=localindex+1;
           end;
         if p[count] = #0 then break;
         count:=count+1;
       end;
     GetParam:=temp;
   end;


    function paramstr(l : longint) : string;
      var
       p : pchar;
       s1 : string;
      begin
         if l = 0 then
         Begin
           s1 := '';
         end
         else
         if (l>0) and (l<=paramcount) then
           begin
             p:=args;
             paramstr:=GetParam(word(l),p);
           end
         else paramstr:='';
      end;

      function paramcount : longint;
      Begin
        paramcount := argc;
      end;




    procedure randomize;

      var
         hl : longint;

      begin
         asm
           movem.l d2/d3/a2/a3, -(sp)     { save OS registers }
           move.w #17,-(sp)
           trap   #14         { call xbios - random number }
           add.l  #2,sp
           movem.l (sp)+,d2/d3/a2/a3
           move.l d0,hl       { result in d0 }
         end;
         randseed:=hl;
      end;

function getheapstart:pointer;assembler;
asm
        lea.l   HEAP,a0
        move.l  a0,d0
end;


function getheapsize:longint;assembler;
asm
       move.l   HEAP_SIZE,d0
end ['D0'];

  { This routine is used to grow the heap.  }
  { But here we do a trick, we say that the }
  { heap cannot be regrown!                 }
  function sbrk( size: longint): pointer;
  { on exit nil = if fails.               }
  Begin
   sbrk:=nil;
  end;

{$I heap.inc}


{****************************************************************************
                          Low Level File Routines
 ****************************************************************************}

procedure DoDirSeparators(p:pchar);
var
  i : longint;
begin
{ allow slash as backslash }
  for i:=0 to strlen(p) do
   if p[i] in AllowDirectorySeparators then p[i]:=DirectorySeparator;
end;


procedure do_close(h : longint);
begin
  asm
        movem.l d2/d3/a2/a3,-(sp)
        move.l  h,d0
        move.w  d0,-(sp)
        move.w  #$3e,-(sp)
        trap    #1
        add.l   #4,sp      { restore stack ... }
        movem.l (sp)+,d2/d3/a2/a3
  end;
end;


procedure do_erase(p : pchar);
begin
  DoDirSeparators(p);
  asm
        move.l  d2,d6            { save d2   }
        movem.l d3/a2/a3,-(sp)   { save regs }
        move.l  p,-(sp)
        move.w #$41,-(sp)
        trap   #1
        add.l  #6,sp
        move.l d6,d2       { restore d2 }
        movem.l (sp)+,d3/a2/a3
        tst.w  d0
        beq    @doserend
        move.w d0,errno
        @doserend:
  end;
  if errno <> 0 then
     Error2InOut;
end;


procedure do_rename(p1,p2 : pchar);
begin
  DoDirSeparators(p1);
  DoDirSeparators(p2);
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)
            move.l  p1,-(sp)
            move.l  p2,-(sp)
            clr.w   -(sp)
            move.w  #$56,-(sp)
            trap    #1
            lea     12(sp),sp
            move.l  d6,d2       { restore d2 }
            movem.l (sp)+,d3/a2/a3
            tst.w   d0
            beq     @dosreend
            move.w  d0,errno    { error ... }
         @dosreend:
  end;
  if errno <> 0 then
     Error2InOut;
end;

function do_isdevice(handle:word):boolean;
begin
  if (handle=stdoutputhandle) or (handle=stdinputhandle) or
  (handle=stderrorhandle) then
    do_isdevice:=FALSE
  else
    do_isdevice:=TRUE;
end;


function do_write(h,addr,len : longint) : longint;
begin
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)
            move.l  addr,-(sp)
            move.l  len,-(sp)
            move.l  h,d0
            move.w  d0,-(sp)
            move.w  #$40,-(sp)
            trap    #1
            lea     12(sp),sp
            move.l d6,d2       { restore d2 }
            movem.l (sp)+,d3/a2/a3
            tst.l   d0
            bpl     @doswrend
            move.w  d0,errno    { error ... }
          @doswrend:
            move.l  d0,@RESULT
  end;
  if errno <> 0 then
     Error2InOut;
end;


function do_read(h,addr,len : longint) : longint;
begin
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)
            move.l addr,-(sp)
            move.l len,-(sp)
            move.l h,d0
            move.w d0,-(sp)
            move.w #$3f,-(sp)
            trap   #1
            lea    12(sp),sp
            move.l d6,d2       { restore d2 }
            movem.l (sp)+,d3/a2/a3
            tst.l   d0
            bpl     @dosrdend
            move.w  d0,errno    { error ... }
          @dosrdend:
            move.l  d0,@Result
  end;
  if errno <> 0 then
     Error2InOut;
end;


function do_filepos(handle : longint) : longint;
begin
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)
            move.w #1,-(sp)     { seek from current position }
            move.l handle,d0
            move.w d0,-(sp)
            move.l #0,-(sp)     { with a seek offset of zero }
            move.w #$42,-(sp)
            trap   #1
            lea    10(sp),sp
            move.l d6,d2       { restore d2 }
            movem.l (sp)+,d3/a2/a3
            move.l d0,@Result
  end;
end;


procedure do_seek(handle,pos : longint);
begin
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)
            move.w #0,-(sp)     { seek from start of file    }
            move.l handle,d0
            move.w d0,-(sp)
            move.l pos,-(sp)
            move.w #$42,-(sp)
            trap   #1
            lea    10(sp),sp
            move.l d6,d2       { restore d2 }
            movem.l (sp)+,d3/a2/a3
  end;
end;


function do_seekend(handle:longint):longint;
var
 t: longint;
begin
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)
            move.w #2,-(sp)     { seek from end of file        }
            move.l handle,d0
            move.w d0,-(sp)
            move.l #0,-(sp)     { with an offset of 0 from end }
            move.w #$42,-(sp)
            trap   #1
            lea    10(sp),sp
            move.l d6,d2       { restore d2 }
            movem.l (sp)+,d3/a2/a3
            move.l d0,t
  end;
   do_seekend:=t;
end;


function do_filesize(handle : longint) : longint;
var
   aktfilepos : longint;
begin
   aktfilepos:=do_filepos(handle);
   do_filesize:=do_seekend(handle);
   do_seek(handle,aktfilepos);
end;


procedure do_truncate (handle,pos:longint);
begin
  do_seek(handle,pos);
  {!!!!!!!!!!!!}
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
  i : word;
  oflags: longint;
begin
  DoDirSeparators(p);
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
  oflags:=$02; { read/write mode }
{ convert filemode to filerec modes }
  case (flags and 3) of
   0 : begin
         filerec(f).mode:=fminput;
         oflags:=$00; { read mode only }
       end;
   1 : filerec(f).mode:=fmoutput;
   2 : filerec(f).mode:=fminout;
  end;
  if (flags and $1000)<>0 then
   begin
     filerec(f).mode:=fmoutput;
     oflags:=$04;  { read/write with create }
   end
  else
   if (flags and $100)<>0 then
    begin
      filerec(f).mode:=fmoutput;
      oflags:=$02;  { read/write             }
    end;
{ empty name is special }
  if p[0]=#0 then
   begin
     case filerec(f).mode of
       fminput : filerec(f).handle:=StdInputHandle;
      fmappend,
      fmoutput : begin
                   filerec(f).handle:=StdOutputHandle;
                   filerec(f).mode:=fmoutput; {fool fmappend}
                 end;
     end;
     exit;
   end;
   asm
      movem.l d2/d3/a2/a3,-(sp)    { save used registers }

      cmp.l   #4,oflags    { check if rewrite mode ... }
      bne     @opencont2
      { rewrite mode - create new file }
      move.w  #0,-(sp)
      move.l  p,-(sp)
      move.w  #$3c,-(sp)
      trap    #1
      add.l   #8,sp       { restore stack of os call }
      bra     @end
      { reset - open existing files     }
    @opencont2:
      move.l  oflags,d0    { use flag as source  ...    }
    @opencont1:
      move.w  d0,-(sp)
      move.l  p,-(sp)
      move.w  #$3d,-(sp)
      trap    #1
      add.l   #8,sp       { restore stack of os call }
   @end:
      movem.l (sp)+,d2/d3/a2/a3

      tst.w   d0
      bpl     @opennoerr  { if positive return values then ok }
      cmp.w   #-1,d0      { if handle is -1 CON:              }
      beq     @opennoerr
      cmp.w   #-2,d0      { if handle is -2 AUX:              }
      beq     @opennoerr
      cmp.w   #-3,d0      { if handle is -3 PRN:              }
      beq     @opennoerr
      move.w  d0,errno    { otherwise normal error            }
    @opennoerr:
      move.w  d0,i        { get handle as SIGNED VALUE...     }
    end;
  if errno <> 0 then
     Error2InOut;
  filerec(f).handle:=i;
  if ((flags and $100) <> 0) and
       (FileRec (F).Handle <> UnusedHandle) then
   do_seekend(filerec(f).handle);
end;

{*****************************************************************************
                           UnTyped File Handling
*****************************************************************************}

{$i file.inc}

{*****************************************************************************
                           Typed File Handling
*****************************************************************************}

{$i typefile.inc}

{*****************************************************************************
                           Text File Handling
*****************************************************************************}

{$i text.inc}

{*****************************************************************************
                           Directory Handling
*****************************************************************************}

procedure DosDir(func:byte;const s:string);
var
  buffer : array[0..255] of char;
  c : word;
begin
  move(s[1],buffer,length(s));
  buffer[length(s)]:=#0;
  DoDirSeparators(pchar(@buffer));
  c:=word(func);
  asm
        move.l  d2,d6      { save d2 }
        movem.l d3/a2/a3,-(sp)
        pea     buffer
        move.w  c,-(sp)
        trap    #1
        add.l   #6,sp
        move.l  d6,d2       { restore d2 }
        movem.l (sp)+,d3/a2/a3
        tst.w   d0
        beq     @dosdirend
        move.w  d0,errno
     @dosdirend:
  end;
  if errno <> 0 then
     Error2InOut;
end;


procedure mkdir(const s : string);[IOCheck];
begin
  If InOutRes <> 0 then exit;
  DosDir($39,s);
end;


procedure rmdir(const s : string);[IOCheck];
begin
  If InOutRes <> 0 then exit;
  DosDir($3a,s);
end;


procedure chdir(const s : string);[IOCheck];
begin
  If InOutRes <> 0 then exit;
  DosDir($3b,s);
end;


function GetDirIO (DriveNr: byte; var Dir: ShortString): word;
                                               [public, alias: 'FPC_GETDIRIO'];
var
  temp : array[0..255] of char;
  i    : longint;
  j: byte;
  drv: word;
begin
  GetDirIO := 0;
  drv:=word(drivenr);
  asm
            move.l  d2,d6      { save d2 }
            movem.l d3/a2/a3,-(sp)

            { Get dir from drivenr : 0=default, 1=A etc... }
            move.w drv,-(sp)

            { put (previously saved) offset in si }
{            move.l temp,-(sp)}
             pea   temp

            { call attos function 47H : Get dir }
            move.w #$47,-(sp)

            { make the call }
            trap   #1
            add.l  #8,sp

            move.l d6,d2         { restore d2 }
            movem.l (sp)+,d3/a2/a3
  end;
  { conversion to pascal string }
  i:=0;
  while (temp[i]<>#0) do
   begin
     if temp[i] in AllowDirectorySeparators then
       temp[i]:=DirectorySeparator;
     dir[i+3]:=temp[i];
     inc(i);
   end;
  dir[2]:=':';
  dir[3]:='\';
  dir[0]:=char(i+2);
{ upcase the string (FPC Pascal function) }
  dir:=upcase(dir);
  if drivenr<>0 then   { Drive was supplied. We know it }
   dir[1]:=chr(65+drivenr-1)
  else
   begin
      asm
        move.l  d2,d6      { save d2 }
        movem.l d3/a2/a3,-(sp)
        move.w #$19,-(sp)
        trap   #1
        add.l  #2,sp
        move.w d0,drv
        move.l d6,d2        { restore d2 }
        movem.l (sp)+,d3/a2/a3
     end;
     dir[1]:=chr(byte(drv)+ord('A'));
   end;
end;

procedure GetDir (DriveNr: byte; var Dir: ShortString);

begin
  InOutRes := GetDirIO (DriveNr, Dir);
end;


{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}
Procedure system_exit;
begin
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

function CheckInitialStkLen (StkLen: SizeUInt): SizeUInt;
begin
  CheckInitialStkLen := StkLen;
end;

begin
  StackLength := CheckInitialStkLen (InitialStkLen);
{ Initialize ExitProc }
  ExitProc:=Nil;
{ Setup heap }
  InitHeap;
{ Setup stdin, stdout and stderr }
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
{ Reset IO Error }
  InOutRes:=0;
(* This should be changed to a real value during *)
(* thread driver initialization if appropriate.  *)
  ThreadID := 1;
  errno := 0;
{ Setup command line arguments }
  argc:=GetParamCount(args);
  InitVariantManager;
{$ifdef HASWIDESTRING}
  InitWideStringManager;
{$endif HASWIDESTRING}
end.
