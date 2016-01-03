{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    Low leve file functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************

                          Low Level File Routines

****************************************************************************}


procedure do_close (H: THandle);

begin
{ Only three standard handles under real OS/2 }
  if (h > 4) or
     ((os_MODE = osOS2) and (h > 2)) then
   begin
     asm
        pushl %ebx
        movb $0x3e,%ah
        movl h,%ebx
        call syscall
        jnc  .Lnoerror           { error code?            }
        movw  %ax, InOutRes       { yes, then set InOutRes }
     .Lnoerror:
        popl %ebx
     end ['eax'];
   end;
end;

procedure do_erase(p:Pchar; pchangeable: boolean);
var
  oldp: pchar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
    oldp:=p;
    DoDirSeparators(p,pchangeable);
=======
    DoDirSeparators(p);
>>>>>>> graemeg/fixes_2_2
=======
    DoDirSeparators(p);
>>>>>>> origin/fixes_2_2
    asm
        movl P,%edx
        movb $0x41,%ah
        call syscall
        jnc .LERASE1
        movw %ax,inoutres
    .LERASE1:
    end ['eax', 'edx'];
    if p<>oldp then
      freemem(p);
end;

procedure do_rename(p1,p2:Pchar; p1changeable, p2changeable: boolean);
var
  oldp1, oldp2 : pchar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
    oldp1:=p1;
    oldp2:=p2;
    DoDirSeparators(p1,p1changeable);
    DoDirSeparators(p2,p2changeable);
=======
    DoDirSeparators(p1);
    DoDirSeparators(p2);
>>>>>>> graemeg/fixes_2_2
=======
    DoDirSeparators(p1);
    DoDirSeparators(p2);
>>>>>>> origin/fixes_2_2
    asm
        movl P1, %edx
        movl P2, %edi
        movb $0x56,%ah
        call syscall
        jnc .LRENAME1
        movw %ax,inoutres
    .LRENAME1:
    end ['eax', 'edx', 'edi'];
  if p1<>oldp1 then
    freemem(p1);
  if p2<>oldp2 then
    freemem(p2);
end;

function do_read (H: THandle; Addr: pointer; Len: longint): longint; assembler;
asm
    pushl %ebx
{$IFNDEF REGCALL}
    movl len,%ecx
    movl addr,%edx
    movl %eax,%ebx
{$ELSE REGCALL}
    movl h,%ebx
{$ENDIF REGCALL}
    movb $0x3f,%ah
    call syscall
    jnc .LDOSREAD1
    movw %ax,inoutres
    xorl %eax,%eax
.LDOSREAD1:
    popl %ebx
end {['eax', 'ebx', 'ecx', 'edx']};

function do_write (H: THandle; Addr: pointer; Len: longint): longint;
                                                                     assembler;
asm
    pushl %ebx
{$IFDEF REGCALL}
    movl %eax,%ebx
{$ENDIF REGCALL}
    xorl %eax,%eax
    cmpl $0,len    { 0 bytes to write is undefined behavior }
    jz   .LDOSWRITE1
{$IFNDEF REGCALL}
    movl len,%ecx
    movl addr,%edx
    movl h,%ebx
{$ENDIF REGCALL}
    movb $0x40,%ah
    call syscall
    jnc .LDOSWRITE1
    movw %ax,inoutres
.LDOSWRITE1:
    popl %ebx
end {['eax', 'ebx', 'ecx', 'edx']};

function do_filepos (Handle: THandle): longint; assembler;
asm
    pushl %ebx
{$IFDEF REGCALL}
    movl %eax,%ebx
{$ELSE REGCALL}
    movl handle,%ebx
{$ENDIF REGCALL}
    movw $0x4201,%ax
    xorl %edx,%edx
    call syscall
    jnc .LDOSFILEPOS
    movw %ax,inoutres
    xorl %eax,%eax
.LDOSFILEPOS:
    popl %ebx
end {['eax', 'ebx', 'ecx', 'edx']};

procedure do_seek (Handle: THandle; Pos: longint); assembler;
asm
    pushl %ebx
{$IFDEF REGCALL}
    movl %eax,%ebx
{$ELSE REGCALL}
    movl handle,%ebx
    movl pos,%edx
{$ENDIF REGCALL}
    movw $0x4200,%ax
    call syscall
    jnc .LDOSSEEK1
    movw %ax,inoutres
.LDOSSEEK1:
    popl %ebx
end {['eax', 'ebx', 'ecx', 'edx']};

function do_seekend (Handle: THandle): longint; assembler;
asm
    pushl %ebx
{$IFDEF REGCALL}
    movl %eax,%ebx
{$ELSE REGCALL}
    movl handle,%ebx
{$ENDIF REGCALL}
    movw $0x4202,%ax
    xorl %edx,%edx
    call syscall
    jnc .Lset_at_end1
    movw %ax,inoutres;
    xorl %eax,%eax
.Lset_at_end1:
    popl %ebx
end {['eax', 'ebx', 'ecx', 'edx']};

function do_filesize (Handle: THandle): longint;

var aktfilepos:longint;

begin
    aktfilepos:=do_filepos(handle);
    do_filesize:=do_seekend(handle);
    do_seek(handle,aktfilepos);
end;

procedure do_truncate (Handle: THandle; Pos: longint); assembler;
asm
    pushl %ebx
(* DOS function 40h isn't safe for this according to EMX documentation *)
{$IFDEF REGCALL}
    movl %eax,%ebx
    pushl %eax
{$ELSE REGCALL}
    movl Handle,%ebx
    movl Pos,%edx
{$ENDIF REGCALL}
    movl $0x7F25,%eax
    call syscall
    incl %eax
    movl %ecx, %eax
{$IFDEF REGCALL}
    popl %ebx
{$ENDIF REGCALL}
    jnz .LTruncate1      { compare the value of EAX to verify error }
(* File position is undefined after truncation, move to the end. *)
    movl $0x4202,%eax
{$IFNDEF REGCALL}
    movl Handle,%ebx
{$ENDIF REGCALL}
    movl $0,%edx
    call syscall
    jnc .LTruncate2
.LTruncate1:
    movw %ax,inoutres
.LTruncate2:
    popl %ebx
end {['eax', 'ebx', 'ecx', 'edx']};

const
    FileHandleCount: cardinal = 20;

function Increase_File_Handle_Count: boolean;
var Err: word;
    L1: longint;
    L2: cardinal;
begin
    if os_mode = osOS2 then
        begin
            L1 := 10;
            if DosSetRelMaxFH (L1, L2) <> 0 then
                Increase_File_Handle_Count := false
            else
                if L2 > FileHandleCount then
                    begin
                        FileHandleCount := L2;
                        Increase_File_Handle_Count := true;
                    end
                else
                    Increase_File_Handle_Count := false;
        end
    else
        begin
            Inc (FileHandleCount, 10);
            Err := 0;
            asm
                pushl %ebx
                movl $0x6700, %eax
                movl FileHandleCount, %ebx
                call syscall
                jnc .LIncFHandles
                movw %ax, Err
.LIncFHandles:
                popl %ebx
            end ['eax'];
            if Err <> 0 then
                begin
                    Increase_File_Handle_Count := false;
                    Dec (FileHandleCount, 10);
                end
            else
                Increase_File_Handle_Count := true;
        end;
end;

procedure do_open(var f;p:pchar;flags:longint; pchangeable: boolean);

{
  filerec and textrec have both handle and mode as the first items so
  they could use the same routine for opening/creating.
  when (flags and $100)   the file will be append
  when (flags and $1000)  the file will be truncate/rewritten
  when (flags and $10000) there is no check for close (needed for textfiles)
}

var
  Action: cardinal;
  oldp : pchar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
    DoDirSeparators(p);
>>>>>>> graemeg/fixes_2_2
=======
    DoDirSeparators(p);
>>>>>>> origin/fixes_2_2
    { close first if opened }
    if ((flags and $10000)=0) then
        begin
            case filerec(f).mode of
                fminput,fmoutput,fminout : Do_Close(filerec(f).handle);
                fmclosed:;
            else
                begin
                    inoutres:=102; {not assigned}
                    exit;
                end;
            end;
       end;
    { reset file handle }
    filerec(f).handle := UnusedHandle;
    Action := 0;
    { convert filemode to filerec modes }
    case (flags and 3) of
        0 : filerec(f).mode:=fminput;
        1 : filerec(f).mode:=fmoutput;
        2 : filerec(f).mode:=fminout;
    end;
    if (flags and $1000)<>0 then
        Action := $50000; (* Create / replace *)
    { empty name is special }
    if p[0]=#0 then
        begin
          case FileRec(f).mode of
            fminput :
              FileRec(f).Handle:=StdInputHandle;
            fminout, { this is set by rewrite }
            fmoutput :
              FileRec(f).Handle:=StdOutputHandle;
            fmappend :
              begin
                FileRec(f).Handle:=StdOutputHandle;
                FileRec(f).mode:=fmoutput; {fool fmappend}
              end;
            end;
            exit;
        end;
    oldp:=p;
    DoDirSeparators(p,pchangeable);
    Action := Action or (Flags and $FF);
(* DenyNone if sharing not specified. *)
    if Flags and 112 = 0 then
        Action := Action or 64;
    asm
        pushl %ebx
        movl $0x7f2b, %eax
        movl Action, %ecx
        movl p, %edx
        call syscall
        cmpl $0xffffffff, %eax
        jnz .LOPEN1
        movw %cx, InOutRes
        movl UnusedHandle, %eax
.LOPEN1:
        movl f,%edx         { Warning : This assumes Handle is first }
        movl %eax,(%edx)    { field of FileRec                       }
        popl %ebx
    end ['eax', 'ecx', 'edx'];
    if (InOutRes = 4) and Increase_File_Handle_Count then
(* Trying again after increasing amount of file handles *)
        asm
            pushl %ebx
            movl $0x7f2b, %eax
            movl Action, %ecx
            movl p, %edx
            call syscall
            cmpl $0xffffffff, %eax
            jnz .LOPEN2
            movw %cx, InOutRes
            movl UnusedHandle, %eax
.LOPEN2:
            movl f,%edx
            movl %eax,(%edx)
            popl %ebx
        end ['eax', 'ecx', 'edx'];
      { for systems that have more handles }
    if (FileRec (F).Handle <> UnusedHandle) then
        begin
            if (FileRec (F).Handle > FileHandleCount) then
                                         FileHandleCount := FileRec (F).Handle;
            if ((Flags and $100) <> 0) then
                begin
                    do_seekend (FileRec (F).Handle);
                    FileRec (F).Mode := fmOutput; {fool fmappend}
                end;
        end
    else
      FileRec(f).mode:=fmclosed;
    if oldp<>p then
      freemem(p);
end;

{$ASMMODE INTEL}
function do_isdevice (Handle: THandle): boolean; assembler;
(*
var HT, Attr: longint;
begin
    if os_mode = osOS2 then
        begin
            if DosQueryHType (Handle, HT, Attr) <> 0 then HT := 1;
        end
    else
*)
asm
    push ebx
{$IFDEF REGCALL}
    mov ebx, eax
{$ELSE REGCALL}
    mov ebx, Handle
{$ENDIF REGCALL}
    mov eax, 4400h
    call syscall
    mov eax, 1
    jc @IsDevEnd
    test edx, 80h           { verify if it is a file  }
    jnz @IsDevEnd
    dec eax                 { nope, so result is zero }
@IsDevEnd:
    pop ebx
end {['eax', 'ebx', 'edx']};
{$ASMMODE ATT}





