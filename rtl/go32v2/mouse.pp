{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Mouse unit for Go32v2

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Mouse;
interface

{$i mouseh.inc}

{ tells the mouse unit to draw the mouse cursor itself }
procedure DoCustomMouse(b : boolean);

const
  MouseIsVisible: boolean = false;


implementation

uses
  video,go32;

{$i mouse.inc}


var
  RealSeg : Word;                                    { Real mode segment }
  RealOfs : Word;                                    { Real mode offset }
  CurrentMask : word;
  MouseCallback : Pointer;                           { Mouse call back ptr }
  UnderNT: boolean;
{$ifdef DEBUG}
  EntryEDI,EntryESI : longint;
  EntryDS,EntryES : word;
{$endif DEBUG}
  { Real mode registers in text segment below $ffff limit
    for Windows NT
    NOTE this might cause problem if someone want to
    protect text section against writing (would be possible
    with CWSDPMI under raw dos, not implemented yet !) }
  ActionRegs    : TRealRegs;external name '___v2prt0_rmcb_regs';
  v2prt0_ds_alias : word;external name '___v2prt0_ds_alias';
const
  MousePresent : boolean = false;
  First_try    : boolean = true;
{$ifdef DEBUG}
  MouseError   : longint = 0;
  CallCounter  : longint = 0;
{$endif DEBUG}
  drawmousecursor : boolean = false;
  { position where the mouse was drawn the last time }
  oldmousex : longint = -1;
  oldmousey : longint = -1;
  mouselock : boolean = false;

{ if the cursor is drawn by this the unit, we must be careful }
{ when drawing while the interrupt handler is called          }
procedure lockmouse;assembler;

  asm
  .Ltrylockagain:
     movb    $1,%al
     xchgb   mouselock,%al
     orb     %al,%al
     jne     .Ltrylockagain
  end;

procedure unlockmouse;

  begin
     mouselock:=false;
  end;


{$ASMMODE ATT}
procedure MouseInt;assembler;
asm
        pushl   %edi
        pushl   %ebx
        movb    %bl,mousebuttons
        movw    %cx,mousewherex
        movw    %dx,mousewherey
        shrw    $3,%cx
        shrw    $3,%dx
        { should we draw the mouse cursor? }
        cmpb    $0,drawmousecursor
        je      .Lmouse_nocursor
        cmpb    $0,mouseisvisible
        je      .Lmouse_nocursor
        pushw   %fs
        pushl   %eax
        pushl   %edi
        { check lock }
        movb    $1,%al
        xchgb   mouselock,%al
        orb     %al,%al
        { don't update the cursor yet, because hide/showcursor is called }
        jne    .Ldont_draw

        { load start of video buffer }
        movzwl  videoseg,%edi
        shll    $4,%edi
        movw    dosmemselector,%fs

        { calculate address of old mouse cursor }
        movl    oldmousey,%eax
        imulw   screenwidth,%ax
        addl    oldmousex,%eax
        leal    1(%edi,%eax,2),%eax
        { remove old cursor }
        xorb    $0x7f,%fs:(%eax)

        { store position of old cursor }
        movzwl  %cx,%ecx
        movl    %ecx,oldmousex
        movzwl  %dx,%edx
        movl    %edx,oldmousey

        { calculate address of new cursor }
        movl    %edx,%eax
        imulw   screenwidth,%ax
        addl    %ecx,%eax
        leal    1(%edi,%eax,2),%eax
        { draw new cursor }
        xorb    $0x7f,%fs:(%eax)

        { unlock mouse }
        movb    $0,mouselock

.Ldont_draw:
        popl    %edi
        popl    %eax
        popw    %fs
.Lmouse_nocursor:
        cmpb    MouseEventBufSize,PendingMouseEvents
        je      .Lmouse_exit
        movl    PendingMouseTail,%edi
        movw    %bx,(%edi)
        movw    %cx,2(%edi)
        movw    %dx,4(%edi)
        movw    $0,6(%edi)
        addl    $8,%edi
        leal    PendingMouseEvent,%eax
        addl    MouseEventBufSize*8,%eax
        cmpl    %eax,%edi
        jne     .Lmouse_nowrap
        leal    PendingMouseEvent,%edi
.Lmouse_nowrap:
        movl    %edi,PendingMouseTail
        incb    PendingMouseEvents
.Lmouse_exit:
        popl   %ebx
        popl   %edi
end;



PROCEDURE Mouse_Trap; ASSEMBLER;
ASM
   PUSH %ES;                                          { Save ES register }
   PUSH %DS;                                          { Save DS register }
   PUSHL %EDI;                                        { Save register }
   PUSHL %ESI;                                        { Save register }
   { ; caution : ds is not the selector for our data !! }
{$ifdef DEBUG}
   MOVL  %EDI,%ES:EntryEDI
   MOVL  %ESI,%ES:EntryESI
   MOVW  %DS,%AX
   MOVW  %AX,%ES:EntryDS
   MOVW  %ES,%AX
   MOVW  %AX,%ES:EntryES
{$endif DEBUG}
 {  movw  %cs:v2prt0_ds_alias,%ax v2prt0 is not locked !!
   movw  %ax,%ds
   movw  %ax,%es }
   PUSH %ES;                                          { Push data seg }
   POP %DS;                                           { Load data seg }
{$ifdef DEBUG}
   incl callcounter
   CMPL $ACTIONREGS,%edi
   JE  .L_ActionRegsOK
   INCL MouseError
   JMP  .L_NoCallBack
.L_ActionRegsOK:
{$endif DEBUG}
   MOVL MOUSECALLBACK, %EAX;                          { Fetch callback addr }
   CMPL $0, %EAX;                                     { Check for nil ptr }
   JZ .L_NoCallBack;                                  { Ignore if nil }
   MOVL %EDI,%EAX;                                    { %EAX = @actionregs }
   MOVL (%EAX), %EDI;                                 { EDI from actionregs }
   MOVL 4(%EAX), %ESI;                                { ESI from actionregs }
   MOVL 16(%EAX), %EBX;                               { EBX from actionregs }
   MOVL 20(%EAX), %EDX;                               { EDX from actionregs }
   MOVL 24(%EAX), %ECX;                               { ECX from actionregs }
   MOVL 28(%EAX), %EAX;                               { EAX from actionregs }
   CALL *MOUSECALLBACK;                               { Call callback proc }
.L_NoCallBack:
   POPL %ESI;                                         { Recover register }
   POPL %EDI;                                         { Recover register }
   POP %DS;                                           { Restore DS register }
   POP %ES;                                           { Restore ES register }
   {  This works for WinNT
   movzwl %si,%eax
   but CWSDPMI need this }
   movl %esi,%eax
   MOVL %ds:(%Eax), %EAX;
   MOVL %EAX, %ES:42(%EDI);                           { Set as return addr }
   ADDW $4, %ES:46(%EDI);                             { adjust stack }
   IRET;                                              { Interrupt return }
END;

PROCEDURE Mouse_Trap_NT; ASSEMBLER;
ASM
   pushl %eax;
   PUSH %ES;                                          { Save ES register }
   PUSH %DS;                                          { Save DS register }
   PUSH %FS;                                          { Save FS register }
   PUSHL %EDI;                                        { Save register }
   PUSHL %ESI;                                        { Save register }
   pushl %ebx;
   pushl %ecx;
   pushl %edx;
   { ; caution : ds is not the selector for our data !! }
   MOVW %cs:v2prt0_ds_alias,%ax
   movw %ax,%es
   { ES now has dataseg  alias that is never invalid }
{$ifdef DEBUG}
   MOVL  %EDI,%ES:EntryEDI
   MOVL  %ESI,%ES:EntryESI
   MOVW  %DS,%AX
   MOVW  %AX,%ES:EntryDS
   MOVW  %ES,%AX
   MOVW  %AX,%ES:EntryES
{$endif DEBUG}
 {  movw  %cs:v2prt0_ds_alias,%ax v2prt0 is not locked !!
   movw  %ax,%ds
   movw  %ax,%es }
   PUSH %ES;                                          { Push data seg }
   POP %DS;                                           { Load data seg }
{$ifdef DEBUG}
   incl callcounter
   CMPL $ACTIONREGS,%edi
   JE  .L_ActionRegsOK
   INCL MouseError
   JMP  .L_NoCallBack
.L_ActionRegsOK:
{$endif DEBUG}
   MOVL MOUSECALLBACK, %EAX;                          { Fetch callback addr }
   CMPL $0, %EAX;                                     { Check for nil ptr }
   JZ .L_NoCallBack;                                  { Ignore if nil }
   MOVL %EDI,%EAX;                                    { %EAX = @actionregs }
   MOVL (%EAX), %EDI;                                 { EDI from actionregs }
   MOVL 4(%EAX), %ESI;                                { ESI from actionregs }
   MOVL 16(%EAX), %EBX;                               { EBX from actionregs }
   MOVL 20(%EAX), %EDX;                               { EDX from actionregs }
   MOVL 24(%EAX), %ECX;                               { ECX from actionregs }
   MOVL 28(%EAX), %EAX;                               { EAX from actionregs }
   CALL *MOUSECALLBACK;                               { Call callback proc }
.L_NoCallBack:
   popl %edx;
   popl %ecx;
   popl %ebx;
   POPL %ESI;                                         { Recover register }
   POPL %EDI;                                         { Recover register }
   POP %FS;                                           { Restore FS register }
   POP %DS;                                           { Restore DS register }
   POP %ES;                                           { Restore ES register }
   movw %es,%ax
   cmpw $0,%ax
   jne .Lesisok
   { ; caution : ds is not the selector for our data !! }
   MOVW %cs:v2prt0_ds_alias,%ax
   movw %ax,%es
.Lesisok:
   lsl  %eax,%eax
   cmpl %edi,%eax
   ja   .Ldontzeroedi
   movzwl %di,%edi
.Ldontzeroedi:
   movw %ds,%ax
   lsl  %eax,%eax
   cmpl %esi,%eax
   ja   .Lsimplecopy
   movzwl %si,%eax
   jmp  .Lcopyend
.Lsimplecopy:
   movl %esi,%eax
.Lcopyend:
   MOVL %ds:(%Eax), %EAX
   MOVL %EAX, %ES:42(%EDI)                           { Set as return addr }
   ADDW $4, %ES:46(%EDI)                             { adjust stack }
   popl %eax
   IRET                                              { Interrupt return }
END;

Function Allocate_mouse_bridge : boolean;
var
  error : word;
begin
  ASM
    pushl %edi
    pushl %esi
    LEAL ACTIONREGS, %EDI;                       { Addr of actionregs }
    LEAL MOUSE_TRAP, %ESI;                       { Procedure address }
    CMPB $0, UnderNT
    JZ  .LGo32
    LEAL MOUSE_TRAP_NT, %ESI;                       { Procedure address }
  .LGo32:
    PUSH %DS;                                    { Save DS segment }
    PUSH %ES;                                    { Save ES segment }
    MOVW v2prt0_ds_alias,%ES;                    { ES now has dataseg  alias that is never invalid }
    PUSH %CS;
    POP  %DS;                                    { DS now has codeseg }
    MOVW $0x303, %AX;                            { Function id }
    INT  $0x31;                                  { Call DPMI bridge }
    JNC .L_call_ok;                              { Branch if ok }
    POP  %ES;                                    { Restore ES segment }
    POP  %DS;                                    { Restore DS segment }
    MOVW $0,REALSEG;
    MOVW $0,REALOFS;
    JMP  .L_exit
  .L_call_ok:
    POP  %ES;                                    { Restore ES segment }
    POP  %DS;                                    { Restore DS segment }
    MOVW %CX,REALSEG;                            { Transfer real seg }
    MOVW %DX,REALOFS;                            { Transfer real ofs }
    MOVW $0, %AX;                                { Force error to zero }
  .L_exit:
    MOVW %AX, ERROR;                             { Return error state }
    popl %esi
    popl %edi
  END;
  Allocate_mouse_bridge:=error=0;
end;

Procedure Release_mouse_bridge;
begin
  ASM
     MOVW $0x304, %AX;                            { Set function id }
     MOVW REALSEG, %CX;                           { Bridged real seg }
     MOVW REALOFS, %DX;                           { Bridged real ofs }
     INT $0x31;                                   { Release bridge }
     MOVW $0,REALSEG;
     MOVW $0,REALOFS;
  END;
end;

PROCEDURE Mouse_Action (Mask : Word; P : Pointer);
VAR
  Error : Word;
  Rg    : TRealRegs;
BEGIN
  Error := 0;                                         { Preset no error }
  If (P <> MouseCallBack) or (Mask<>CurrentMask) Then                        { Check func different }
   Begin
   { Remove old calback }
     If (CurrentMask <> 0) Then
      Begin
        Rg.AX := 12;                                   { Function id }
        Rg.CX := 0;                                    { Zero mask register }
        Rg.ES := 0;                                    { Zero proc seg }
        Rg.DX := 0;                                    { Zero proc ofs }
        RealIntr($33, Rg);                             { Stop INT 33 callback }
      End;
     if RealSeg=0 then
       error:=1;
    { test addresses for Windows NT }
    if (longint(@actionregs)>$ffff) {or
       (longint(@mouse_trap)>$ffff)} then
      begin
         error:=1;
      end
    else If (P = Nil) Then
     Begin
       Mask := 0;                                    { Zero mask register }
     End;
    If (Error = 0) Then
     Begin
       MouseCallback := P;                            { Set call back addr }
       if Mask<>0 then
         begin
           Rg.AX := 12;                                   { Set function id }
           Rg.CX := Mask;                                 { Set mask register }
           If Mask<>0 then
             begin
               Rg.ES := RealSeg;                              { Real mode segment }
               Rg.DX := RealOfs;                              { Real mode offset }
             end
           else
             begin
               Rg.ES:=0;
               Rg.DX:=0;
             end;
           RealIntr($33, Rg);                             { Set interrupt 33 }
         end;
       CurrentMask:=Mask;
     End;
   End;
  If (Error <> 0) Then
   Begin
     Writeln('GO32V2 mouse handler set failed !!');
     ReadLn;                                          { Wait for user to see }
   End;
END;


{ We need to remove the mouse callback before exiting !! PM }

const StoredExit : Pointer = Nil;
      FirstMouseInitDone : boolean = false;

procedure MouseSafeExit;
begin
  ExitProc:=StoredExit;
  if MouseCallBack<>Nil then
    Mouse_Action(0, Nil);
  if not FirstMouseInitDone then
    exit;
  FirstMouseInitDone:=false;
  Unlock_Code(Pointer(@Mouse_Trap), 400);            { Release trap code }
  Unlock_Code(Pointer(@Mouse_Trap_NT), 400);            { Release trap code }
  Unlock_Code(Pointer(@MouseInt), 400);               { Lock MouseInt code  }
  Unlock_Data(ActionRegs, SizeOf(TRealRegs));        { Release registers }
  UnLock_Data(MouseCallBack,SizeOf(Pointer));
  { unlock Mouse Queue and related stuff ! }
  Unlock_Data(PendingMouseEvent,
        MouseEventBufSize*Sizeof(TMouseEvent));
  Unlock_Data(PendingMouseTail,SizeOf(longint));
  Unlock_Data(PendingMouseEvents,sizeof(byte));
  Unlock_Data(MouseButtons,SizeOf(byte));
  Unlock_Data(MouseWhereX,SizeOf(word));
  Unlock_Data(MouseWhereY,SizeOf(word));
  Unlock_Data(drawmousecursor,SizeOf(boolean));
  Unlock_Data(mouseisvisible,SizeOf(boolean));
  Unlock_Data(mouselock,SizeOf(boolean));
  Unlock_Data(videoseg,SizeOf(word));
  Unlock_Data(dosmemselector,SizeOf(word));
  Unlock_Data(screenwidth,SizeOf(word));
  Unlock_Data(OldMouseX,SizeOf(longint));
  Unlock_Data(OldMouseY,SizeOf(longint));
{$ifdef DEBUG}
  Unlock_Data(EntryEDI, SizeOf(longint));
  Unlock_Data(EntryESI, SizeOf(longint));
  Unlock_Data(EntryDS, SizeOf(word));
  Unlock_Data(EntryES, SizeOf(word));
  Unlock_Data(MouseError, SizeOf(longint));
  Unlock_Data(callcounter, SizeOf(longint));
{$endif DEBUG}
  Release_mouse_bridge;
end;

function RunningUnderWINNT: boolean;
var r: trealregs;
begin
  fillchar(r,sizeof(r),0);
  r.ax:=$3306;
  realintr($21,r);
  RunningUnderWINNT:=(r.bx=$3205);
end;

procedure SysInitMouse;
begin
  UnderNT:=RunningUnderWINNT;
  if not MousePresent then
    begin
      if DetectMouse=0 then
        begin
          if First_try then
            begin
              Writeln('No mouse driver found ');
              First_try:=false;
            end;
          exit;
        end
      else
        MousePresent:=true;
    end;
  { don't do this twice !! PM }

  If not FirstMouseInitDone then
    begin
      StoredExit:=ExitProc;
      ExitProc:=@MouseSafeExit;
      Lock_Code(Pointer(@Mouse_Trap), 400);              { Lock trap code }
      Lock_Code(Pointer(@Mouse_Trap_NT), 400);              { Lock trap code }
      Lock_Code(Pointer(@MouseInt), 400);               { Lock MouseInt code  }
      Lock_Data(ActionRegs, SizeOf(TRealRegs));          { Lock registers }
      Lock_Data(MouseCallBack, SizeOf(pointer));
      { lock Mouse Queue and related stuff ! }
      Lock_Data(PendingMouseEvent,
        MouseEventBufSize*Sizeof(TMouseEvent));
      Lock_Data(PendingMouseTail,SizeOf(longint));
      Lock_Data(PendingMouseEvents,sizeof(byte));
      Lock_Data(MouseButtons,SizeOf(byte));
      Lock_Data(MouseWhereX,SizeOf(word));
      Lock_Data(MouseWhereY,SizeOf(word));
      Lock_Data(drawmousecursor,SizeOf(boolean));
      Lock_Data(mouseisvisible,SizeOf(boolean));
      Lock_Data(mouselock,SizeOf(boolean));
      Lock_Data(videoseg,SizeOf(word));
      Lock_Data(dosmemselector,SizeOf(word));
      Lock_Data(screenwidth,SizeOf(word));
      Lock_Data(OldMouseX,SizeOf(longint));
      Lock_Data(OldMouseY,SizeOf(longint));
{$ifdef DEBUG}
      Lock_Data(EntryEDI, SizeOf(longint));
      Lock_Data(EntryESI, SizeOf(longint));
      Lock_Data(EntryDS, SizeOf(word));
      Lock_Data(EntryES, SizeOf(word));
      Lock_Data(MouseError, SizeOf(longint));
      Lock_Data(callcounter, SizeOf(longint));
{$endif DEBUG}
      Allocate_mouse_bridge;
      FirstMouseInitDone:=true;
    end;
  If MouseCallBack=Nil then
    Mouse_Action($ffff, @MouseInt);                    { Set masks/interrupt }
  drawmousecursor:=false;
  mouseisvisible:=false;
  if (screenwidth>80) or (screenheight>50) then
    DoCustomMouse(true);
  ShowMouse;
end;


procedure SysDoneMouse;
begin
  HideMouse;
  If (MouseCallBack <> Nil) Then
    Mouse_Action(0, Nil);                            { Clear mask/interrupt }
end;


function SysDetectMouse:byte;assembler;
asm
        pushl   %ebx
        movl    $0x200,%eax
        movl    $0x33,%ebx
        int     $0x31
        movw    %cx,%ax
        orw     %ax,%dx
        jz      .Lno_mouse
        xorl    %eax,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        orw     %ax,%ax
        jz      .Lno_mouse
        movl    %ebx,%eax
.Lno_mouse:
        popl    %ebx
end;


procedure SysShowMouse;

begin
   if drawmousecursor then
     begin
        lockmouse;
        if not(mouseisvisible) then
          begin
             oldmousex:=getmousex-1;
             oldmousey:=getmousey-1;
             mem[videoseg:(((screenwidth*oldmousey)+oldmousex)*2)+1]:=
               mem[videoseg:(((screenwidth*oldmousey)+oldmousex)*2)+1] xor $7f;
             mouseisvisible:=true;
          end;
        unlockmouse;
     end
   else
     asm
             cmpb    $1,MousePresent
             jne     .LShowMouseExit
             movl    $1,%eax
             pushl   %ebp
             int     $0x33
             popl    %ebp
     .LShowMouseExit:
     end;
  MouseIsVisible := true;
end;


procedure SysHideMouse;

begin
   if drawmousecursor then
     begin
        lockmouse;
        if mouseisvisible then
          begin
             mouseisvisible:=false;
             mem[videoseg:(((screenwidth*oldmousey)+oldmousex)*2)+1]:=
               mem[videoseg:(((screenwidth*oldmousey)+oldmousex)*2)+1] xor $7f;
             oldmousex:=-1;
             oldmousey:=-1;
          end;
        unlockmouse;
     end
   else
     asm
             cmpb    $1,MousePresent
             jne     .LHideMouseExit
             movl    $2,%eax
             pushl   %ebp
             int     $0x33
             popl    %ebp
     .LHideMouseExit:
     end;
  MouseIsVisible := false;
end;


function SysGetMouseX:word;assembler;
asm
        pushl   %ebx
        cmpb    $1,MousePresent
        jne     .LGetMouseXError
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        movzwl  %cx,%eax
        shrl    $3,%eax
        incl    %eax
        jmp .Lexit
.LGetMouseXError:
        xorl    %eax,%eax
.Lexit:
        popl    %ebx
end;


function SysGetMouseY:word;assembler;
asm
        pushl   %ebx
        cmpb    $1,MousePresent
        jne     .LGetMouseYError
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        movzwl  %dx,%eax
        shrl    $3,%eax
        incl    %eax
        jmp .Lexit
.LGetMouseYError:
        xorl    %eax,%eax
.Lexit:
        popl    %ebx
end;


function SysGetMouseButtons:word;assembler;
asm
        pushl   %ebx
        cmpb    $1,MousePresent
        jne     .LGetMouseButtonsError
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        movw    %bx,%ax
        jmp     .Lexit
.LGetMouseButtonsError:
        xorl    %eax,%eax
.Lexit:
        popl    %ebx
end;


procedure SysSetMouseXY(x,y:word);assembler;
asm
        pushl   %ebx
        cmpb    $1,MousePresent
        jne     .LSetMouseXYExit
        movw    x,%cx
        movw    y,%dx
        movl    $4,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
.LSetMouseXYExit:
        popl    %ebx
end;

Procedure SetMouseXRange (Min,Max:Longint);
begin
  If Not(MousePresent) Then Exit;
  asm
        movl    $7,%eax
        movl    min,%ecx
        movl    max,%edx
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
end;

Procedure SetMouseYRange (min,max:Longint);
begin
  If Not(MousePresent) Then Exit;
  asm
        movl    $8,%eax
        movl    min,%ecx
        movl    max,%edx
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
end;

procedure DoCustomMouse(b : boolean);

  begin
     HideMouse;
     lockmouse;
     oldmousex:=-1;
     oldmousey:=-1;
     SetMouseXRange(0,(screenwidth-1)*8);
     SetMouseYRange(0,(screenheight-1)*8);
     if b then
       begin
          mouseisvisible:=false;
          drawmousecursor:=true;
       end
     else
       drawmousecursor:=false;
     unlockmouse;
  end;

const
  LastCallcounter : longint = 0;

procedure SysGetMouseEvent(var MouseEvent: TMouseEvent);
var
 RR: TRealRegs;
begin
  if not MousePresent then
    begin
      Fillchar(MouseEvent,SizeOf(TMouseEvent),#0);
    end;
{$ifdef DEBUG}
  if mouseError>0 then
    Writeln('Errors in mouse Handler ',MouseError);
{$ifdef EXTMOUSEDEBUG}
  if callcounter>LastCallcounter then
    Writeln('Number of calls in mouse Handler ',Callcounter);
{$endif EXTMOUSEDEBUG}
  LastCallcounter:=Callcounter;
{$endif DEBUG}
  while PendingMouseEvents = 0 do
   begin
(* Give up time slices while waiting for mouse events. *)
    RealIntr ($28, RR);
   end;
  MouseEvent:=PendingMouseHead^;
  inc(PendingMouseHead);
  if PendingMouseHead=@PendingMouseEvent[0]+MouseEventBufsize then
   PendingMouseHead:=@PendingMouseEvent[0];
  dec(PendingMouseEvents);
  if (LastMouseEvent.x<>MouseEvent.x) or (LastMouseEvent.y<>MouseEvent.y) then
   MouseEvent.Action:=MouseActionMove;
  if (LastMouseEvent.Buttons<>MouseEvent.Buttons) then
   begin
     if (LastMouseEvent.Buttons and MouseEvent.buttons<>LastMouseEvent.Buttons) then
       MouseEvent.Action:=MouseActionUp
     else
       MouseEvent.Action:=MouseActionDown;
   end;
  LastMouseEvent:=MouseEvent;
end;


Const
  SysMouseDriver : TMouseDriver = (
    useDefaultQueue : true;
    InitDriver      : @SysInitMouse;
    DoneDriver      : @SysDoneMouse;
    DetectMouse     : @SysDetectMouse;
    ShowMouse       : @SysShowMouse;
    HideMouse       : @SysHideMouse;
    GetMouseX       : @SysGetMouseX;
    GetMouseY       : @SysGetMouseY;
    GetMouseButtons : @SysGetMouseButtons;
    SetMouseXY      : @SysSetMouseXY;
    GetMouseEvent   : @SysGetMouseEvent;
    PollMouseEvent  : Nil;
    PutMouseEvent  : Nil;
  );

Begin
  SetMouseDriver(SysMouseDriver);
end.
