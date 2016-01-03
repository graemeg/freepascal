{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Keyboard unit for linux

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit keyboard;

{$inline on}

{*****************************************************************************}
                                  interface
{*****************************************************************************}

{$i keybrdh.inc}

const
  AltPrefix : byte = 0;
  ShiftPrefix : byte = 0;
  CtrlPrefix : byte = 0;

type
  Tprocedure = procedure;

  PTreeElement = ^TTreeElement;
  TTreeElement = record
    Next,Parent,Child :  PTreeElement;
    CanBeTerminal : boolean;
    char : byte;
    ScanValue : byte;
    CharValue : byte;
    SpecialHandler : Tprocedure;
  end;

function RawReadKey:char;
function RawReadString : String;
function KeyPressed : Boolean;
procedure AddSequence(const St : String; AChar,AScan :byte);inline;
function FindSequence(const St : String;var AChar, Ascan : byte) : boolean;
procedure RestoreStartMode;

function AddSpecialSequence(const St : string;Proc : Tprocedure) : PTreeElement; platform;


{*****************************************************************************}
                               implementation
{*****************************************************************************}

uses
  Mouse,  Strings,
  termio,baseUnix
  {$ifdef linux},linuxvcs{$endif};

{$i keyboard.inc}

var OldIO,StartTio : TermIos;
{$ifdef linux}
    is_console:boolean;
    vt_switched_away:boolean;
{$endif}
{$ifdef logging}
    f : text;
{$endif logging}

const
  KeyBufferSize = 20;
var
  KeyBuffer : Array[0..KeyBufferSize-1] of Char;
  KeyPut,
  KeySend   : longint;

{ Buffered Input routines }
const
  InSize=256;
var
  InBuf  : array [0..InSize-1] of char;
{  InCnt,}
  InHead,
  InTail : longint;

{$i keyscan.inc}

{Some internal only scancodes}
const KbShiftUp    = $f0;
      KbShiftLeft  = $f1;
      KbShiftRight = $f2;
      KbShiftDown  = $f3;
      KbShiftHome  = $f4;
      KbShiftEnd   = $f5;

      double_esc_hack_enabled : boolean = false;

{$ifdef Unused}
type
   TKeyState = Record
      Normal, Shift, Ctrl, Alt : word;
     end;

const
  KeyStates : Array[0..255] of TKeyState
    (

    );

{$endif Unused}

procedure SetRawMode(b:boolean);

var Tio:Termios;

begin
  TCGetAttr(1,Tio);
  if b then
   begin
     {Standard output now needs #13#10.}
     settextlineending(output,#13#10);
     OldIO:=Tio;
     CFMakeRaw(Tio);
   end
  else
    begin
      Tio := OldIO;
      {Standard output normally needs just a linefeed.}
      settextlineending(output,#10);
    end;
  TCsetattr(1,TCSANOW,Tio);
end;

{$ifdef linux}

{The Linux console can do nice things: we can get the state of the shift keys,
 and reprogram the keys. That's nice since it allows excellent circumvention
 of VT100 limitations, we can make the keyboard work 100%...

 A 100% working keyboard seems to be a pretty basic requirement, but we're
 one of the few guys providing such an outrageous luxury (DM).}

type
  chgentry=packed record
    tab,
    idx,
    oldtab,
    oldidx : byte;
    oldval,
    newval : word;
  end;
  kbentry=packed record
    kb_table,
    kb_index : byte;
    kb_value : word;
  end;
  kbsentry=packed record
    kb_func:byte;
    kb_string:array[0..511] of char;
  end;
  vt_mode=packed record
    mode,          {vt mode}
    waitv:byte;    {if set, hang on writes if not active}
    relsig,        {signal to raise on release req}
    acqsig,        {signal to raise on acquisition}
    frsig:word;    {unused (set to 0)}
 end;

const
  kbdchange:array[0..23] of chgentry=(
    {This prevents the alt+function keys from switching consoles.
     We code the F1..F12 sequences into ALT+F1..ALT+12, we check
     the shiftstates separetely anyway.}
    (tab:8; idx:$3b; oldtab:0; oldidx:$3b; oldval:0; newval:0),
    (tab:8; idx:$3c; oldtab:0; oldidx:$3c; oldval:0; newval:0),
    (tab:8; idx:$3d; oldtab:0; oldidx:$3d; oldval:0; newval:0),
    (tab:8; idx:$3e; oldtab:0; oldidx:$3e; oldval:0; newval:0),
    (tab:8; idx:$3f; oldtab:0; oldidx:$3f; oldval:0; newval:0),
    (tab:8; idx:$40; oldtab:0; oldidx:$40; oldval:0; newval:0),
    (tab:8; idx:$41; oldtab:0; oldidx:$41; oldval:0; newval:0),
    (tab:8; idx:$42; oldtab:0; oldidx:$42; oldval:0; newval:0),
    (tab:8; idx:$43; oldtab:0; oldidx:$43; oldval:0; newval:0),
    (tab:8; idx:$44; oldtab:0; oldidx:$44; oldval:0; newval:0),
    (tab:8; idx:$45; oldtab:0; oldidx:$45; oldval:0; newval:0),
    (tab:8; idx:$46; oldtab:0; oldidx:$46; oldval:0; newval:0),
    {This prevents the shift+function keys outputting strings, so
     the kernel will the codes for the non-shifted function
     keys. This is desired because normally shift+f1/f2 will output the
     same string as f11/12. We will get the shift state separately.}
    (tab:1; idx:$3b; oldtab:0; oldidx:$3b; oldval:0; newval:0),
    (tab:1; idx:$3c; oldtab:0; oldidx:$3c; oldval:0; newval:0),
    (tab:1; idx:$3d; oldtab:0; oldidx:$3d; oldval:0; newval:0),
    (tab:1; idx:$3e; oldtab:0; oldidx:$3e; oldval:0; newval:0),
    (tab:1; idx:$3f; oldtab:0; oldidx:$3f; oldval:0; newval:0),
    (tab:1; idx:$40; oldtab:0; oldidx:$40; oldval:0; newval:0),
    (tab:1; idx:$41; oldtab:0; oldidx:$41; oldval:0; newval:0),
    (tab:1; idx:$42; oldtab:0; oldidx:$42; oldval:0; newval:0),
    (tab:1; idx:$43; oldtab:0; oldidx:$43; oldval:0; newval:0),
    (tab:1; idx:$44; oldtab:0; oldidx:$44; oldval:0; newval:0),
    (tab:1; idx:$45; oldtab:0; oldidx:$45; oldval:0; newval:0),
    (tab:1; idx:$46; oldtab:0; oldidx:$46; oldval:0; newval:0)
  );

 KDGKBENT=$4B46;
 KDSKBENT=$4B47;
 KDGKBSENT=$4B48;
 KDSKBSENT=$4B49;
 KDGKBMETA=$4B62;
 KDSKBMETA=$4B63;
 K_ESCPREFIX=$4;
 K_METABIT=$3;
 VT_GETMODE=$5601;
 VT_SETMODE=$5602;
 VT_RELDISP=$5605;
 VT_PROCESS=1;

const
  oldmeta : longint = 0;
  meta : longint = 0;

var oldesc0,oldesc1,oldesc2,oldesc4,oldesc8:word;

procedure prepare_patching;

var entry : kbentry;
    i:longint;

begin
  for i:=low(kbdchange) to high(kbdchange) do
   with kbdchange[i] do
     begin
       entry.kb_table:=tab;
       entry.kb_index:=idx;
       fpIoctl(stdinputhandle,KDGKBENT,@entry);
       oldval:=entry.kb_value;
       entry.kb_table:=oldtab;
       entry.kb_index:=oldidx;
       fpioctl(stdinputhandle,KDGKBENT,@entry);
       newval:=entry.kb_value;
     end;
  {Save old escape code.}
  entry.kb_index:=1;
  entry.kb_table:=0;
  fpioctl(stdinputhandle,KDGKBENT,@entry);
  oldesc0:=entry.kb_value;
  entry.kb_table:=1;
  fpioctl(stdinputhandle,KDGKBENT,@entry);
  oldesc1:=entry.kb_value;
  entry.kb_table:=2;
  fpioctl(stdinputhandle,KDGKBENT,@entry);
  oldesc2:=entry.kb_value;
  entry.kb_table:=4;
  fpioctl(stdinputhandle,KDGKBENT,@entry);
  oldesc4:=entry.kb_value;
  entry.kb_table:=8;
  fpioctl(stdinputhandle,KDGKBENT,@entry);
  oldesc8:=entry.kb_value;
end;

procedure PatchKeyboard;
var
  entry : kbentry;
  sentry : kbsentry;
  i:longint;
begin
  fpIoctl(stdinputhandle,KDGKBMETA,@oldmeta);
  meta:=K_ESCPREFIX;
  fpIoctl(stdinputhandle,KDSKBMETA,@meta);
  for i:=low(kbdchange) to high(kbdchange) do
    with kbdchange[i] do
      begin
        entry.kb_table:=tab;
        entry.kb_index:=idx;
        entry.kb_value:=newval;
        fpioctl(stdinputhandle,KDSKBENT,@entry);
      end;

  {Map kernel escape key code to symbol F32.}
  entry.kb_index:=1;
  entry.kb_value:=$011f;
  entry.kb_table:=0;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=1;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=2;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=4;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=8;
  fpioctl(stdinputhandle,KDSKBENT,@entry);

  {F32 (the escape key) will generate ^[[0~ .}
  sentry.kb_func:=31;
  sentry.kb_string:=#27'[0~';
  fpioctl(stdinputhandle,KDSKBSENT,@sentry);
end;


procedure UnpatchKeyboard;
var
  e : ^chgentry;
  entry : kbentry;
  i : longint;
begin
  if oldmeta in [K_ESCPREFIX,K_METABIT] then
    fpioctl(stdinputhandle,KDSKBMETA,@oldmeta);
  for i:=low(kbdchange) to high(kbdchange) do
    with kbdchange[i] do
      begin
        entry.kb_table:=tab;
        entry.kb_index:=idx;
        entry.kb_value:=oldval;
        fpioctl(stdinputhandle,KDSKBENT,@entry);
      end;

  entry.kb_index:=1;
  entry.kb_table:=0;
  entry.kb_value:=oldesc0;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=1;
  entry.kb_value:=oldesc1;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=2;
  entry.kb_value:=oldesc2;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=4;
  entry.kb_value:=oldesc4;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
  entry.kb_table:=8;
  entry.kb_value:=oldesc8;
  fpioctl(stdinputhandle,KDSKBENT,@entry);
end;

{A problem of patching the keyboard is that it no longer works as expected
 when working on another console. So we unpatch it when the user switches
 away.}

const switches:longint=0;

procedure vt_handler(sig:longint);cdecl;

begin
  if vt_switched_away then
    begin
      {Confirm the switch.}
      fpioctl(stdoutputhandle,VT_RELDISP,pointer(2));
      {Switching to program, patch keyboard.}
      patchkeyboard;
    end
  else
    begin
      {Switching away from program, unpatch the keyboard.}
      unpatchkeyboard;
      fpioctl(stdoutputhandle,VT_RELDISP,pointer(1));
    end;
  vt_switched_away:=not vt_switched_away;
  {Clear buffer.}
  intail:=inhead;
end;

procedure install_vt_handler;

var mode:vt_mode;

begin
{  ioctl(vt_fd,KDSETMODE,KD_GRAPHICS);}
    fpioctl(stdoutputhandle,VT_GETMODE,@mode);
    mode.mode:=VT_PROCESS;
    mode.relsig:=SIGUSR1;
    mode.acqsig:=SIGUSR1;
    vt_switched_away:=false;
    fpsignal(SIGUSR1,@vt_handler);
    fpioctl(stdoutputhandle,VT_SETMODE,@mode);
end;
{$endif}

function ttyRecvChar:char;

var Readed,i : longint;

begin
  {Buffer empty? Yes, input from stdin}
  if (InHead=InTail) then
    begin
      {Calc Amount of Chars to Read}
      i:=InSize-InHead;
      if InTail>InHead then
        i:=InTail-InHead;
      {Read}
      repeat
        Readed:=fpRead(StdInputHandle,InBuf[InHead],i);
      until readed<>-1;
      {Increase Counters}
      inc(InHead,Readed);
      {Wrap if End has Reached}
      if InHead>=InSize then
        InHead:=0;
    end;
  {Check Buffer}
  ttyRecvChar:=InBuf[InTail];
  inc(InTail);
  if InTail>=InSize then
    InTail:=0;
end;

procedure PushKey(Ch:char);
var
  Tmp : Longint;
begin
  Tmp:=KeyPut;
  Inc(KeyPut);
  If KeyPut>=KeyBufferSize Then
   KeyPut:=0;
  If KeyPut<>KeySend Then
   KeyBuffer[Tmp]:=Ch
  Else
   KeyPut:=Tmp;
End;


function PopKey:char;
begin
  If KeyPut<>KeySend Then
   begin
     PopKey:=KeyBuffer[KeySend];
     Inc(KeySend);
     If KeySend>=KeyBufferSize Then
      KeySend:=0;
   End
  Else
   PopKey:=#0;
End;


procedure PushExt(b:byte);
begin
  PushKey(#0);
  PushKey(chr(b));
end;


const
  AltKeyStr  : string[38]='qwertyuiopasdfghjklzxcvbnm1234567890-=';
  AltCodeStr : string[38]=#016#017#018#019#020#021#022#023#024#025#030#031#032#033#034#035#036#037#038+
                          #044#045#046#047#048#049#050#120#121#122#123#124#125#126#127#128#129#130#131;
function FAltKey(ch:char):byte;
var
  Idx : longint;
begin
  Idx:=Pos(ch,AltKeyStr);
  if Idx>0 then
   FAltKey:=byte(AltCodeStr[Idx])
  else
   FAltKey:=0;
End;


{ This one doesn't care about keypresses already processed by readkey  }
{ and waiting in the KeyBuffer, only about waiting keypresses at the   }
{ TTYLevel (including ones that are waiting in the TTYRecvChar buffer) }
function sysKeyPressed: boolean;
var
  fdsin : tfdSet;
begin
  if (inhead<>intail) then
   sysKeyPressed:=true
  else
   begin
     fpFD_ZERO(fdsin);
     fpFD_SET(StdInputHandle,fdsin);
     sysKeypressed:=(fpSelect(StdInputHandle+1,@fdsin,nil,nil,0)>0);
   end;
end;

function KeyPressed:Boolean;
begin
  Keypressed := (KeySend<>KeyPut) or sysKeyPressed;
End;


const
  LastMouseEvent : TMouseEvent =
  (
    Buttons : 0;
    X : 0;
    Y : 0;
    Action : 0;
  );

  procedure GenMouseEvent;
  var MouseEvent: TMouseEvent;
      ch : char;
      fdsin : tfdSet;
      buttonval:byte;
  begin
    fpFD_ZERO(fdsin);
    fpFD_SET(StdInputHandle,fdsin);
{    Fillchar(MouseEvent,SizeOf(TMouseEvent),#0);}
    MouseEvent.action:=0;
    if inhead=intail then
      fpSelect(StdInputHandle+1,@fdsin,nil,nil,10);
    ch:=ttyRecvChar;
    { Other bits are used for Shift, Meta and Ctrl modifiers PM }
    buttonval:=byte(ch)-byte(' ');
    {bits 0..1: button status
     bit  5   : mouse movement while button down.
     bit  6   : interpret button 1 as button 4
                interpret button 2 as button 5}
    case buttonval and 3 of
      0 : {left button press}
        MouseEvent.buttons:=1;
      1 : {middle button pressed }
        MouseEvent.buttons:=2;
      2 : { right button pressed }
        MouseEvent.buttons:=4;
      3 : { no button pressed }
        MouseEvent.buttons:=0;
    end;
     if inhead=intail then
       fpSelect(StdInputHandle+1,@fdsin,nil,nil,10);
     ch:=ttyRecvChar;
     MouseEvent.x:=Ord(ch)-ord(' ')-1;
     if inhead=intail then
      fpSelect(StdInputHandle+1,@fdsin,nil,nil,10);
     ch:=ttyRecvChar;
     MouseEvent.y:=Ord(ch)-ord(' ')-1;
     mouseevent.action:=MouseActionMove;
     if (lastmouseevent.buttons=0) and (mouseevent.buttons<>0) then
       MouseEvent.action:=MouseActionDown;
     if (lastmouseevent.buttons<>0) and (mouseevent.buttons=0) then
       MouseEvent.action:=MouseActionUp;
(*
     else

       begin
         if (LastMouseEvent.Buttons<>0) and
            ((LastMouseEvent.X<>MouseEvent.X) or (LastMouseEvent.Y<>MouseEvent.Y)) then
           begin
             MouseEvent.Action:=MouseActionMove;
             MouseEvent.Buttons:=LastMouseEvent.Buttons;
{$ifdef DebugMouse}
             Writeln(system.stderr,' Mouse Move (',MouseEvent.X,',',MouseEvent.Y,')');
{$endif DebugMouse}
             PutMouseEvent(MouseEvent);
             MouseEvent.Buttons:=0;
           end;
         MouseEvent.Action:=MouseActionUp;
       end;
*)
     PutMouseEvent(MouseEvent);
{$ifdef DebugMouse}
     if MouseEvent.Action=MouseActionDown then
       Write(system.stderr,'Button down : ')
     else
       Write(system.stderr,'Button up : ');
     Writeln(system.stderr,'buttons = ',MouseEvent.Buttons,' (',MouseEvent.X,',',MouseEvent.Y,')');
{$endif DebugMouse}
     LastMouseEvent:=MouseEvent;
  end;

var roottree:array[char] of PTreeElement;

procedure FreeElement (PT:PTreeElement);
var next : PTreeElement;
begin
  while PT <> nil do
  begin
    FreeElement(PT^.Child);
    next := PT^.Next;
    dispose(PT);
    PT := next;
  end;
end;

procedure FreeTree;

var i:char;

begin
  for i:=low(roottree) to high(roottree) do
    begin
      FreeElement(RootTree[i]);
      roottree[i]:=nil;
    end;
end;

function NewPTree(ch : byte;Pa : PTreeElement) : PTreeElement;
begin
  newPtree:=allocmem(sizeof(Ttreeelement));
  newPtree^.char:=ch;
  newPtree^.Parent:=Pa;
  if Assigned(Pa) and (Pa^.Child=nil) then
    Pa^.Child:=newPtree;
end;

function DoAddSequence(const St : String; AChar,AScan :byte) : PTreeElement;
var
  CurPTree,NPT : PTreeElement;
  c : byte;
  i : longint;
begin
  if St='' then
    begin
      DoAddSequence:=nil;
      exit;
    end;
  CurPTree:=RootTree[st[1]];
  if CurPTree=nil then
    begin
      CurPTree:=NewPTree(ord(st[1]),nil);
      RootTree[st[1]]:=CurPTree;
    end;
  for i:=2 to Length(St) do
    begin
      NPT:=CurPTree^.Child;
      c:=ord(St[i]);
      if NPT=nil then
        NPT:=NewPTree(c,CurPTree);
      CurPTree:=nil;
      while assigned(NPT) and (NPT^.char<c) do
        begin
          CurPTree:=NPT;
          NPT:=NPT^.Next;
        end;

      if assigned(NPT) and (NPT^.char=c) then
        CurPTree:=NPT
      else
        begin
          if CurPTree=nil then
            begin
              NPT^.Parent^.child:=NewPTree(c,NPT^.Parent);
              CurPTree:=NPT^.Parent^.Child;
              CurPTree^.Next:=NPT;
            end
          else
            begin
              CurPTree^.Next:=NewPTree(c,CurPTree^.Parent);
              CurPTree:=CurPTree^.Next;
              CurPTree^.Next:=NPT;
            end;
        end;
    end;
  if CurPTree^.CanBeTerminal then
    begin
      { here we have a conflict !! }
      { maybe we should claim }
      with CurPTree^ do
        begin
{$ifdef DEBUG}
          if (ScanValue<>AScan) or (CharValue<>AChar) then
            Writeln(system.stderr,'key "',st,'" changed value');
          if (ScanValue<>AScan) then
            Writeln(system.stderr,'Scan was ',ScanValue,' now ',AScan);
          if (CharValue<>AChar) then
            Writeln(system.stderr,'Char was ',chr(CharValue),' now ',chr(AChar));
{$endif DEBUG}
          ScanValue:=AScan;
          CharValue:=AChar;
        end;
    end
  else with CurPTree^ do
    begin
      CanBeTerminal:=True;
      ScanValue:=AScan;
      CharValue:=AChar;
    end;
  DoAddSequence:=CurPTree;
end;


procedure AddSequence(const St : String; AChar,AScan :byte);inline;
begin
  DoAddSequence(St,AChar,AScan);
end;

{ Returns the Child that as c as char if it exists }
function FindChild(c : byte;Root : PTreeElement) : PTreeElement;
var
  NPT : PTreeElement;
begin
  NPT:=Root^.Child;
  while assigned(NPT) and (NPT^.char<c) do
    NPT:=NPT^.Next;
  if assigned(NPT) and (NPT^.char=c) then
    FindChild:=NPT
  else
    FindChild:=nil;
end;

function AddSpecialSequence(const St : string;Proc : Tprocedure) : PTreeElement;
var
  NPT : PTreeElement;
begin
  NPT:=DoAddSequence(St,0,0);
  NPT^.SpecialHandler:=Proc;
  AddSpecialSequence:=NPT;
end;

function FindSequence(const St : String;var AChar,AScan :byte) : boolean;
var
  NPT : PTreeElement;
  i,p : byte;
begin
  FindSequence:=false;
  AChar:=0;
  AScan:=0;
  if St='' then
    exit;
  p:=1;
  {This is a distusting hack for certain even more disgusting xterms: Some of
   them send two escapes for an alt-key. If we wouldn't do this, we would need
   to put a lot of entries twice in the table.}
  if double_esc_hack_enabled and (st[1]=#27) and (st[2]='#27') and
     (st[3] in ['a'..'z','A'..'Z','0'..'9','-','+','_','=']) then
    inc(p);
  NPT:=RootTree[St[p]];

  if npt<>nil then
    begin
      for i:=p+1 to Length(St) do
        begin
          NPT:=FindChild(ord(St[i]),NPT);
          if NPT=nil then
            exit;
        end;
      if NPT^.CanBeTerminal then
        begin
          FindSequence:=true;
          AScan:=NPT^.ScanValue;
          AChar:=NPT^.CharValue;
        end;
    end;
end;

type  key_sequence=packed record
        char,scan:byte;
        st:string[7];
      end;

const key_sequences:array[0..276] of key_sequence=(
       (char:0;scan:kbAltA;st:#27'A'),
       (char:0;scan:kbAltA;st:#27'a'),
       (char:0;scan:kbAltB;st:#27'B'),
       (char:0;scan:kbAltB;st:#27'b'),
       (char:0;scan:kbAltC;st:#27'C'),
       (char:0;scan:kbAltC;st:#27'c'),
       (char:0;scan:kbAltD;st:#27'D'),
       (char:0;scan:kbAltD;st:#27'd'),
       (char:0;scan:kbAltE;st:#27'E'),
       (char:0;scan:kbAltE;st:#27'e'),
       (char:0;scan:kbAltF;st:#27'F'),
       (char:0;scan:kbAltF;st:#27'f'),
       (char:0;scan:kbAltG;st:#27'G'),
       (char:0;scan:kbAltG;st:#27'g'),
       (char:0;scan:kbAltH;st:#27'H'),
       (char:0;scan:kbAltH;st:#27'h'),
       (char:0;scan:kbAltI;st:#27'I'),
       (char:0;scan:kbAltI;st:#27'i'),
       (char:0;scan:kbAltJ;st:#27'J'),
       (char:0;scan:kbAltJ;st:#27'j'),
       (char:0;scan:kbAltK;st:#27'K'),
       (char:0;scan:kbAltK;st:#27'k'),
       (char:0;scan:kbAltL;st:#27'L'),
       (char:0;scan:kbAltL;st:#27'l'),
       (char:0;scan:kbAltM;st:#27'M'),
       (char:0;scan:kbAltM;st:#27'm'),
       (char:0;scan:kbAltN;st:#27'N'),
       (char:0;scan:kbAltN;st:#27'n'),
       (char:0;scan:kbAltO;st:#27'O'),
       (char:0;scan:kbAltO;st:#27'o'),
       (char:0;scan:kbAltP;st:#27'P'),
       (char:0;scan:kbAltP;st:#27'p'),
       (char:0;scan:kbAltQ;st:#27'Q'),
       (char:0;scan:kbAltQ;st:#27'q'),
       (char:0;scan:kbAltR;st:#27'R'),
       (char:0;scan:kbAltR;st:#27'r'),
       (char:0;scan:kbAltS;st:#27'S'),
       (char:0;scan:kbAltS;st:#27's'),
       (char:0;scan:kbAltT;st:#27'T'),
       (char:0;scan:kbAltT;st:#27't'),
       (char:0;scan:kbAltU;st:#27'U'),
       (char:0;scan:kbAltU;st:#27'u'),
       (char:0;scan:kbAltV;st:#27'V'),
       (char:0;scan:kbAltV;st:#27'v'),
       (char:0;scan:kbAltW;st:#27'W'),
       (char:0;scan:kbAltW;st:#27'w'),
       (char:0;scan:kbAltX;st:#27'X'),
       (char:0;scan:kbAltX;st:#27'x'),
       (char:0;scan:kbAltY;st:#27'Y'),
       (char:0;scan:kbAltY;st:#27'y'),
       (char:0;scan:kbAltZ;st:#27'Z'),
       (char:0;scan:kbAltZ;st:#27'z'),
       (char:0;scan:kbAltMinus;st:#27'-'),
       (char:0;scan:kbAltEqual;st:#27'='),
       (char:0;scan:kbAlt0;st:#27'0'),
       (char:0;scan:kbAlt1;st:#27'1'),
       (char:0;scan:kbAlt2;st:#27'2'),
       (char:0;scan:kbAlt3;st:#27'3'),
       (char:0;scan:kbAlt4;st:#27'4'),
       (char:0;scan:kbAlt5;st:#27'5'),
       (char:0;scan:kbAlt6;st:#27'6'),
       (char:0;scan:kbAlt7;st:#27'7'),
       (char:0;scan:kbAlt8;st:#27'8'),
       (char:0;scan:kbAlt9;st:#27'9'),

       (char:0;scan:kbF1;st:#27'[[A'),           {linux,konsole,xterm}
       (char:0;scan:kbF2;st:#27'[[B'),           {linux,konsole,xterm}
       (char:0;scan:kbF3;st:#27'[[C'),           {linux,konsole,xterm}
       (char:0;scan:kbF4;st:#27'[[D'),           {linux,konsole,xterm}
       (char:0;scan:kbF5;st:#27'[[E'),           {linux,konsole}
       (char:0;scan:kbF1;st:#27'[11~'),          {Eterm,rxvt}
       (char:0;scan:kbF2;st:#27'[12~'),          {Eterm,rxvt}
       (char:0;scan:kbF3;st:#27'[13~'),          {Eterm,rxvt}
       (char:0;scan:kbF4;st:#27'[14~'),          {Eterm,rxvt}
       (char:0;scan:kbF5;st:#27'[15~'),          {xterm,Eterm,gnome,rxvt}
       (char:0;scan:kbF6;st:#27'[17~'),          {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF7;st:#27'[18~'),          {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF8;st:#27'[19~'),          {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF9;st:#27'[20~'),          {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF10;st:#27'[21~'),         {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF11;st:#27'[23~'),         {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF12;st:#27'[24~'),         {linux,xterm,Eterm,konsole,gnome,rxvt}
       (char:0;scan:kbF1;st:#27'[M'),            {FreeBSD}
       (char:0;scan:kbF2;st:#27'[N'),            {FreeBSD}
       (char:0;scan:kbF3;st:#27'[O'),            {FreeBSD}
       (char:0;scan:kbF4;st:#27'[P'),            {FreeBSD}
       (char:0;scan:kbF5;st:#27'[Q'),            {FreeBSD}
       (char:0;scan:kbF6;st:#27'[R'),            {FreeBSD}
       (char:0;scan:kbF7;st:#27'[S'),            {FreeBSD}
       (char:0;scan:kbF8;st:#27'[T'),            {FreeBSD}
       (char:0;scan:kbF9;st:#27'[U'),            {FreeBSD}
       (char:0;scan:kbF10;st:#27'[V'),           {FreeBSD}
       (char:0;scan:kbF11;st:#27'[W'),           {FreeBSD}
       (char:0;scan:kbF12;st:#27'[X'),           {FreeBSD}
       (char:0;scan:kbF1;st:#27'OP'),            {vt100,gnome,konsole}
       (char:0;scan:kbF2;st:#27'OQ'),            {vt100,gnome,konsole}
       (char:0;scan:kbF3;st:#27'OR'),            {vt100,gnome,konsole}
       (char:0;scan:kbF4;st:#27'OS'),            {vt100,gnome,konsole}
       (char:0;scan:kbF5;st:#27'Ot'),            {vt100}
       (char:0;scan:kbF6;st:#27'Ou'),            {vt100}
       (char:0;scan:kbF7;st:#27'Ov'),            {vt100}
       (char:0;scan:kbF8;st:#27'Ol'),            {vt100}
       (char:0;scan:kbF9;st:#27'Ow'),            {vt100}
       (char:0;scan:kbF10;st:#27'Ox'),           {vt100}
       (char:0;scan:kbF11;st:#27'Oy'),           {vt100}
       (char:0;scan:kbF12;st:#27'Oz'),           {vt100}
       (char:0;scan:kbEsc;st:#27'[0~'),          {if linux keyboard patched, escape
                                                  returns this}
       (char:0;scan:kbIns;st:#27'[2~'),          {linux,Eterm,rxvt}
       (char:0;scan:kbDel;st:#27'[3~'),          {linux,Eterm,rxvt}
       (char:0;scan:kbHome;st:#27'[1~'),         {linux}
       (char:0;scan:kbHome;st:#27'[7~'),         {Eterm,rxvt}
       (char:0;scan:kbHome;st:#27'[H'),          {FreeBSD}
       (char:0;scan:kbHome;st:#27'OH'),          {some xterm configurations}
       (char:0;scan:kbEnd;st:#27'[4~'),          {linux,Eterm}
       (char:0;scan:kbEnd;st:#27'[8~'),          {rxvt}
       (char:0;scan:kbEnd;st:#27'[F'),           {FreeBSD}
       (char:0;scan:kbEnd;st:#27'OF'),           {some xterm configurations}
       (char:0;scan:kbPgUp;st:#27'[5~'),         {linux,Eterm,rxvt}
       (char:0;scan:kbPgUp;st:#27'[I'),          {FreeBSD}
       (char:0;scan:kbPgDn;st:#27'[6~'),         {linux,Eterm,rxvt}
       (char:0;scan:kbPgDn;st:#27'[G'),          {FreeBSD}
       (char:0;scan:kbUp;st:#27'[A'),            {linux,FreeBSD,rxvt}
       (char:0;scan:kbDown;st:#27'[B'),          {linux,FreeBSD,rxvt}
       (char:0;scan:kbRight;st:#27'[C'),         {linux,FreeBSD,rxvt}
       (char:0;scan:kbLeft;st:#27'[D'),          {linux,FreeBSD,rxvt}
       (char:0;scan:kbUp;st:#27'OA'),            {xterm}
       (char:0;scan:kbDown;st:#27'OB'),          {xterm}
       (char:0;scan:kbRight;st:#27'OC'),         {xterm}
       (char:0;scan:kbLeft;st:#27'OD'),          {xterm}
(* Already recognized above as F11!
       (char:0;scan:kbShiftF1;st:#27'[23~'),     {rxvt}
       (char:0;scan:kbShiftF2;st:#27'[24~'),     {rxvt}
*)
       (char:0;scan:kbShiftF3;st:#27'[25~'),     {linux,rxvt}
       (char:0;scan:kbShiftF4;st:#27'[26~'),     {linux,rxvt}
       (char:0;scan:kbShiftF5;st:#27'[28~'),     {linux,rxvt}
       (char:0;scan:kbShiftF6;st:#27'[29~'),     {linux,rxvt}
       (char:0;scan:kbShiftF7;st:#27'[31~'),     {linux,rxvt}
       (char:0;scan:kbShiftF8;st:#27'[32~'),     {linux,rxvt}
       (char:0;scan:kbShiftF9;st:#27'[33~'),     {linux,rxvt}
       (char:0;scan:kbShiftF10;st:#27'[34~'),    {linux,rxvt}
       (char:0;scan:kbShiftF11;st:#27'[23$'),    {rxvt}
       (char:0;scan:kbShiftF12;st:#27'[24$'),    {rxvt}
       (char:0;scan:kbShiftF1;st:#27'[11;2~'),   {konsole in vt420pc mode}
       (char:0;scan:kbShiftF2;st:#27'[12;2~'),   {konsole in vt420pc mode}
       (char:0;scan:kbShiftF3;st:#27'[13;2~'),   {konsole in vt420pc mode}
       (char:0;scan:kbShiftF4;st:#27'[14;2~'),   {konsole in vt420pc mode}
       (char:0;scan:kbShiftF5;st:#27'[15;2~'),   {xterm}
       (char:0;scan:kbShiftF6;st:#27'[17;2~'),   {xterm}
       (char:0;scan:kbShiftF7;st:#27'[18;2~'),   {xterm}
       (char:0;scan:kbShiftF8;st:#27'[19;2~'),   {xterm}
       (char:0;scan:kbShiftF9;st:#27'[20;2~'),   {xterm}
       (char:0;scan:kbShiftF10;st:#27'[21;2~'),  {xterm}
       (char:0;scan:kbShiftF11;st:#27'[23;2~'),  {xterm}
       (char:0;scan:kbShiftF12;st:#27'[24;2~'),  {xterm}
       (char:0;scan:kbShiftF1;st:#27'O5P'),      {xterm}
       (char:0;scan:kbShiftF2;st:#27'O5Q'),      {xterm}
       (char:0;scan:kbShiftF3;st:#27'O5R'),      {xterm}
       (char:0;scan:kbShiftF4;st:#27'O5S'),      {xterm}
       (char:0;scan:kbShiftF1;st:#27'O2P'),      {konsole,xterm}
       (char:0;scan:kbShiftF2;st:#27'O2Q'),      {konsole,xterm}
       (char:0;scan:kbShiftF3;st:#27'O2R'),      {konsole,xterm}
       (char:0;scan:kbShiftF4;st:#27'O2S'),      {konsole,xterm}
       (char:0;scan:kbCtrlF1;st:#27'[11;5~'),    {none, but expected}
       (char:0;scan:kbCtrlF2;st:#27'[12;5~'),    {none, but expected}
       (char:0;scan:kbCtrlF3;st:#27'[13;5~'),    {none, but expected}
       (char:0;scan:kbCtrlF4;st:#27'[14;5~'),    {none, but expected}
       (char:0;scan:kbCtrlF5;st:#27'[15;5~'),    {xterm}
       (char:0;scan:kbCtrlF6;st:#27'[17;5~'),    {xterm}
       (char:0;scan:kbCtrlF7;st:#27'[18;5~'),    {xterm}
       (char:0;scan:kbCtrlF8;st:#27'[19;5~'),    {xterm}
       (char:0;scan:kbCtrlF9;st:#27'[20;5~'),    {xterm}
       (char:0;scan:kbCtrlF10;st:#27'[21;5~'),   {xterm}
       (char:0;scan:kbCtrlF11;st:#27'[23;5~'),   {xterm}
       (char:0;scan:kbCtrlF12;st:#27'[24;5~'),   {xterm}
       (char:0;scan:kbCtrlF1;st:#27'[11^'),      {rxvt}
       (char:0;scan:kbCtrlF2;st:#27'[12^'),      {rxvt}
       (char:0;scan:kbCtrlF3;st:#27'[13^'),      {rxvt}
       (char:0;scan:kbCtrlF4;st:#27'[14^'),      {rxvt}
       (char:0;scan:kbCtrlF5;st:#27'[15^'),      {rxvt}
       (char:0;scan:kbCtrlF6;st:#27'[17^'),      {rxvt}
       (char:0;scan:kbCtrlF7;st:#27'[18^'),      {rxvt}
       (char:0;scan:kbCtrlF8;st:#27'[19^'),      {rxvt}
       (char:0;scan:kbCtrlF9;st:#27'[20^'),      {rxvt}
       (char:0;scan:kbCtrlF10;st:#27'[21^'),     {rxvt}
       (char:0;scan:kbCtrlF11;st:#27'[23^'),     {rxvt}
       (char:0;scan:kbCtrlF12;st:#27'[24^'),     {rxvt}
       (char:0;scan:kbShiftIns;st:#27'[2;2~'),   {should be the code, but shift+ins
                                                  is paste X clipboard in many
                                                  terminal emulators :(}
       (char:0;scan:kbShiftDel;st:#27'[3;2~'),   {xterm,konsole}
       (char:0;scan:kbCtrlIns;st:#27'[2;5~'),    {xterm}
       (char:0;scan:kbCtrlDel;st:#27'[3;5~'),    {xterm}
       (char:0;scan:kbShiftDel;st:#27'[3$'),     {rxvt}
       (char:0;scan:kbCtrlIns;st:#27'[2^'),      {rxvt}
       (char:0;scan:kbCtrlDel;st:#27'[3^'),      {rxvt}
       (char:0;scan:kbAltF1;st:#27#27'[[A'),
       (char:0;scan:kbAltF2;st:#27#27'[[B'),
       (char:0;scan:kbAltF3;st:#27#27'[[C'),
       (char:0;scan:kbAltF4;st:#27#27'[[D'),
       (char:0;scan:kbAltF5;st:#27#27'[[E'),
       (char:0;scan:kbAltF1;st:#27#27'[11~'),    {rxvt}
       (char:0;scan:kbAltF2;st:#27#27'[12~'),    {rxvt}
       (char:0;scan:kbAltF3;st:#27#27'[13~'),    {rxvt}
       (char:0;scan:kbAltF4;st:#27#27'[14~'),    {rxvt}
       (char:0;scan:kbAltF5;st:#27#27'[15~'),    {rxvt}
       (char:0;scan:kbAltF6;st:#27#27'[17~'),    {rxvt}
       (char:0;scan:kbAltF7;st:#27#27'[18~'),    {rxvt}
       (char:0;scan:kbAltF8;st:#27#27'[19~'),    {rxvt}
       (char:0;scan:kbAltF9;st:#27#27'[20~'),    {rxvt}
       (char:0;scan:kbAltF10;st:#27#27'[21~'),   {rxvt}
       (char:0;scan:kbAltF11;st:#27#27'[23~'),   {rxvt}
       (char:0;scan:kbAltF12;st:#27#27'[24~'),   {rxvt}
       (char:0;scan:kbAltF1;st:#27#27'OP'),      {xterm}
       (char:0;scan:kbAltF2;st:#27#27'OQ'),      {xterm}
       (char:0;scan:kbAltF3;st:#27#27'OR'),      {xterm}
       (char:0;scan:kbAltF4;st:#27#27'OS'),      {xterm}
       (char:0;scan:kbAltF5;st:#27#27'Ot'),      {xterm}
       (char:0;scan:kbAltF6;st:#27#27'Ou'),      {xterm}
       (char:0;scan:kbAltF7;st:#27#27'Ov'),      {xterm}
       (char:0;scan:kbAltF8;st:#27#27'Ol'),      {xterm}
       (char:0;scan:kbAltF9;st:#27#27'Ow'),      {xterm}
       (char:0;scan:kbAltF10;st:#27#27'Ox'),     {xterm}
       (char:0;scan:kbAltF11;st:#27#27'Oy'),     {xterm}
       (char:0;scan:kbAltF12;st:#27#27'Oz'),     {xterm}
       (char:0;scan:kbAltF1;st:#27'O3P'),        {xterm on FreeBSD}
       (char:0;scan:kbAltF2;st:#27'O3Q'),        {xterm on FreeBSD}
       (char:0;scan:kbAltF3;st:#27'O3R'),        {xterm on FreeBSD}
       (char:0;scan:kbAltF4;st:#27'O3S'),        {xterm on FreeBSD}
       (char:0;scan:kbAltF5;st:#27'[15;3~'),     {xterm on FreeBSD}
       (char:0;scan:kbAltF6;st:#27'[17;3~'),     {xterm on FreeBSD}
       (char:0;scan:kbAltF7;st:#27'[18;3~'),     {xterm on FreeBSD}
       (char:0;scan:kbAltF8;st:#27'[19;3~'),     {xterm on FreeBSD}
       (char:0;scan:kbAltF9;st:#27'[20;3~'),     {xterm on FreeBSD}
       (char:0;scan:kbAltF10;st:#27'[21;3~'),    {xterm on FreeBSD}
       (char:0;scan:kbAltF11;st:#27'[23;3~'),    {xterm on FreeBSD}
       (char:0;scan:kbAltF12;st:#27'[24;3~'),    {xterm on FreeBSD}

       (char:0;scan:kbShiftTab;st:#27'[Z'),
       (char:0;scan:kbShiftUp;st:#27'[1;2A'),    {xterm}
       (char:0;scan:kbShiftDown;st:#27'[1;2B'),  {xterm}
       (char:0;scan:kbShiftRight;st:#27'[1;2C'), {xterm}
       (char:0;scan:kbShiftLeft;st:#27'[1;2D'),  {xterm}
       (char:0;scan:kbShiftUp;st:#27'[a'),       {rxvt}
       (char:0;scan:kbShiftDown;st:#27'[b'),     {rxvt}
       (char:0;scan:kbShiftRight;st:#27'[c'),    {rxvt}
       (char:0;scan:kbShiftLeft;st:#27'[d'),     {rxvt}
       (char:0;scan:kbShiftEnd;st:#27'[1;2F'),   {xterm}
       (char:0;scan:kbShiftEnd;st:#27'[8$'),     {rxvt}
       (char:0;scan:kbShiftHome;st:#27'[1;2H'),  {xterm}
       (char:0;scan:kbShiftHome;st:#27'[7$'),    {rxvt}

       (char:0;scan:kbCtrlUp;st:#27'[1;5A'),     {xterm}
       (char:0;scan:kbCtrlDown;st:#27'[1;5B'),   {xterm}
       (char:0;scan:kbCtrlRight;st:#27'[1;5C'),  {xterm}
       (char:0;scan:kbCtrlLeft;st:#27'[1;5D'),   {xterm}
       (char:0;scan:kbCtrlUp;st:#27'[Oa'),       {rxvt}
       (char:0;scan:kbCtrlDown;st:#27'[Ob'),     {rxvt}
       (char:0;scan:kbCtrlRight;st:#27'[Oc'),    {rxvt}
       (char:0;scan:kbCtrlLeft;st:#27'[Od'),     {rxvt}
       (char:0;scan:kbCtrlEnd;st:#27'[1;5F'),    {xterm}
       (char:0;scan:kbCtrlEnd;st:#27'[8^'),      {rxvt}
       (char:0;scan:kbCtrlHome;st:#27'[1;5H'),   {xterm}
       (char:0;scan:kbCtrlHome;st:#27'[7^'),     {rxvt}

       (char:0;scan:kbAltUp;st:#27#27'[A'),      {rxvt}
       (char:0;scan:kbAltDown;st:#27#27'[B'),    {rxvt}
       (char:0;scan:kbAltLeft;st:#27#27'[D'),    {rxvt}
       (char:0;scan:kbAltRight;st:#27#27'[C'),   {rxvt}
{$ifdef HAIKU}
       (char:0;scan:kbAltUp;st:#27#27'OA'),
       (char:0;scan:kbAltDown;st:#27#27'OB'),
       (char:0;scan:kbAltRight;st:#27#27'OC'),
{$else}
       (char:0;scan:kbAltUp;st:#27'OA'),
       (char:0;scan:kbAltDown;st:#27'OB'),
       (char:0;scan:kbAltRight;st:#27'OC'),
{$endif}
       (char:0;scan:kbAltLeft;st:#27#27'OD'),
       (char:0;scan:kbAltPgUp;st:#27#27'[5~'),   {rxvt}
       (char:0;scan:kbAltPgDn;st:#27#27'[6~'),   {rxvt}
       (char:0;scan:kbAltEnd;st:#27#27'[4~'),
       (char:0;scan:kbAltEnd;st:#27#27'[8~'),    {rxvt}
       (char:0;scan:kbAltHome;st:#27#27'[1~'),
       (char:0;scan:kbAltHome;st:#27#27'[7~'),   {rxvt}
       (char:0;scan:kbAltIns;st:#27#27'[2~'),    {rxvt}
       (char:0;scan:kbAltDel;st:#27#27'[3~'),    {rxvt}

  { xterm default values }
  { xterm alternate default values }
  { ignored sequences }
       (char:0;scan:0;st:#27'[?1;0c'),
       (char:0;scan:0;st:#27'[?1l'),
       (char:0;scan:0;st:#27'[?1h'),
       (char:0;scan:0;st:#27'[?1;2c'),
       (char:0;scan:0;st:#27'[?7l'),
       (char:0;scan:0;st:#27'[?7h')
      );

procedure LoadDefaultSequences;

var i:cardinal;

begin
  AddSpecialSequence(#27'[M',@GenMouseEvent);
  {Unix backspace/delete hell... Is #127 a backspace or delete?}
  if copy(fpgetenv('TERM'),1,4)='cons' then
    begin
      {FreeBSD is until now only terminal that uses it for delete.}
      DoAddSequence(#127,0,kbDel);        {Delete}
      DoAddSequence(#27#127,0,kbAltDel);  {Alt+delete}
    end
  else
    begin
      DoAddSequence(#127,8,0);            {Backspace}
      DoAddSequence(#27#127,0,kbAltBack); {Alt+backspace}
    end;
  { all Esc letter }
  for i:=low(key_sequences) to high(key_sequences) do
    with key_sequences[i] do
      DoAddSequence(st,char,scan);
end;

function RawReadKey:char;
var
  fdsin    : tfdSet;
begin
  {Check Buffer first}
  if KeySend<>KeyPut then
   begin
     RawReadKey:=PopKey;
     exit;
   end;
  {Wait for Key}
  if not sysKeyPressed then
   begin
     fpFD_ZERO (fdsin);
     fpFD_SET (StdInputHandle,fdsin);
     fpSelect (StdInputHandle+1,@fdsin,nil,nil,nil);
   end;
  RawReadKey:=ttyRecvChar;
end;


function RawReadString : String;
var
  ch : char;
  fdsin : tfdSet;
  St : String;
begin
  St:=RawReadKey;
  fpFD_ZERO (fdsin);
  fpFD_SET (StdInputHandle,fdsin);
  Repeat
     if inhead=intail then
       fpSelect(StdInputHandle+1,@fdsin,nil,nil,10);
     if SysKeyPressed then
       ch:=ttyRecvChar
     else
       ch:=#0;
     if ch<>#0 then
       St:=St+ch;
  Until ch=#0;
  RawReadString:=St;
end;


function ReadKey(var IsAlt : boolean):char;
var
  ch       : char;
  fdsin    : tfdSet;
  store    : array [0..8] of char;
  arrayind : byte;
  NPT,NNPT : PTreeElement;


    procedure RestoreArray;
      var
        i : byte;
      begin
        for i:=0 to arrayind-1 do
          PushKey(store[i]);
      end;

begin
  IsAlt:=false;
{Check Buffer first}
  if KeySend<>KeyPut then
   begin
     ReadKey:=PopKey;
     exit;
   end;
{Wait for Key}
  if not sysKeyPressed then
   begin
     fpFD_ZERO (fdsin);
     fpFD_SET (StdInputHandle,fdsin);
     fpSelect (StdInputHandle+1,@fdsin,nil,nil,nil);
   end;
  ch:=ttyRecvChar;
  NPT:=RootTree[ch];
  if not assigned(NPT) then
    PushKey(ch)
  else
    begin
      fpFD_ZERO(fdsin);
      fpFD_SET(StdInputHandle,fdsin);
      store[0]:=ch;
      arrayind:=1;
      while assigned(NPT) and syskeypressed do
        begin
          if inhead=intail then
            fpSelect(StdInputHandle+1,@fdsin,nil,nil,10);
          ch:=ttyRecvChar;
          if (ch=#27) and double_esc_hack_enabled then
            begin
              {This is the same hack as in findsequence; see findsequence for
               explanation.}
              ch:=ttyrecvchar;
              {Alt+O cannot be used in this situation, it can be a function key.} 
              if not(ch in ['a'..'z','A'..'N','P'..'Z','0'..'9','-','+','_','=']) then
                begin
                  if intail=0 then
                    intail:=insize
                  else
                    dec(intail);
                  inbuf[intail]:=ch;
                  ch:=#27;
                end
              else
                begin
                  write(#27'[?1036l');
                  double_esc_hack_enabled:=false;
                end;
            end;
           NNPT:=FindChild(ord(ch),NPT);
           if assigned(NNPT) then
             begin
               NPT:=NNPT;
               if NPT^.CanBeTerminal and
                  assigned(NPT^.SpecialHandler) then
                 break;
             End;
           if ch<>#0 then
             begin
               store[arrayind]:=ch;
               inc(arrayind);
             end;
           if not assigned(NNPT) then
             begin
               if ch<>#0 then
                 begin
                   { Put that unused char back into InBuf }
                   If InTail=0 then
                     InTail:=InSize-1
                   else
                     Dec(InTail);
                   InBuf[InTail]:=ch;
                 end;
               break;
             end;
        end;
      if assigned(NPT) and NPT^.CanBeTerminal then
        begin
          if assigned(NPT^.SpecialHandler) then
            begin
              NPT^.SpecialHandler;
              PushExt(0);
            end
          else if NPT^.CharValue<>0 then
            PushKey(chr(NPT^.CharValue))
          else if NPT^.ScanValue<>0 then
            PushExt(NPT^.ScanValue);
        end
      else
        RestoreArray;
   end
{$ifdef logging}
       writeln(f);
{$endif logging}
    ;
  ReadKey:=PopKey;
End;

{$ifdef linux}
function ShiftState:byte;

var arg:longint;

begin
  shiftstate:=0;
  arg:=6;
  if fpioctl(StdInputHandle,TIOCLINUX,@arg)=0 then
   begin
     if (arg and 8)<>0 then
      shiftstate:=kbAlt;
     if (arg and 4)<>0 then
      inc(shiftstate,kbCtrl);
     { 2 corresponds to AltGr so set both kbAlt and kbCtrl PM }
     if (arg and 2)<>0 then
      shiftstate:=shiftstate or (kbAlt or kbCtrl);
     if (arg and 1)<>0 then
      inc(shiftstate,kbShift);
   end;
end;

procedure force_linuxtty;

var s:string[15];
    handle:sizeint;
    thistty:string;

begin
  is_console:=false;
  if vcs_device<>-1 then
    begin
       { running on a tty, find out whether locally or remotely }
      thistty:=ttyname(stdinputhandle);
      if (copy(thistty,1,8)<>'/dev/tty') or not (thistty[9] in ['0'..'9']) then
        begin
          {Running from Midnight Commander or something... Bypass it.}
          str(vcs_device,s);
          handle:=fpopen('/dev/tty'+s,O_RDWR);
          fpioctl(stdinputhandle,TIOCNOTTY,nil);
          {This will currently only work when the user is root :(}
          fpioctl(handle,TIOCSCTTY,nil);
          if errno<>0 then
            exit;
          fpclose(stdinputhandle);
          fpclose(stdoutputhandle);
          fpclose(stderrorhandle);
          fpdup2(handle,stdinputhandle);
          fpdup2(handle,stdoutputhandle);
          fpdup2(handle,stderrorhandle);
          fpclose(handle);
        end;
      is_console:=true;
    end;
end;
{$endif linux}

{ Exported functions }

procedure SysInitKeyboard;
begin
  SetRawMode(true);
{$ifdef logging}
     assign(f,'keyboard.log');
     rewrite(f);
{$endif logging}
{$ifdef linux}
  force_linuxtty;
  prepare_patching;
  patchkeyboard;
  if is_console then
    install_vt_handler
  else
    begin
{$endif}
      { default for Shift prefix is ^ A}
      if ShiftPrefix = 0 then
        ShiftPrefix:=1;
      {default for Alt prefix is ^Z }
      if AltPrefix=0 then
        AltPrefix:=26;
      { default for Ctrl Prefix is ^W }
      if CtrlPrefix=0 then
        CtrlPrefix:=23;
      if copy(fpgetenv('TERM'),1,5)='xterm' then
          {The alt key should generate an escape prefix. Save the old setting
           make make it send that escape prefix.}
        begin
          write(#27'[?1036s'#27'[?1036h');
          double_esc_hack_enabled:=true;
        end;
{$ifdef linux}
    end;
{$endif}
  LoadDefaultSequences;
{  LoadTerminfoSequences;}
end;


procedure SysDoneKeyboard;
begin
{$ifdef linux}
  if is_console then
  unpatchkeyboard;
{$endif linux}

  if copy(fpgetenv('TERM'),1,5)='xterm' then
     {Restore the old alt key behaviour.}
     write(#27'[?1036r');

  SetRawMode(false);

  FreeTree;
{$ifdef logging}
  close(f);
{$endif logging}
end;


function SysGetKeyEvent: TKeyEvent;

  function EvalScan(b:byte):byte;
  const
    DScan:array[0..31] of byte = (
      $39, $02, $28, $04, $05, $06, $08, $28,
      $0A, $0B, $09, $0D, $33, $0C, $34, $35,
      $0B, $02, $03, $04, $05, $06, $07, $08,
      $09, $0A, $27, $27, $33, $0D, $34, $35);
   LScan:array[0..31] of byte = (
      $29, $1E, $30, $2E, $20, $12, $21, $22,
      $23, $17, $24, $25, $26, $32, $31, $18,
      $19, $10, $13, $1F, $14, $16, $2F, $11,
      $2D, $15, $2C, $1A, $2B, $1B, $29, $0C);
  begin
    if (b and $E0)=$20  { digits / leters } then
     EvalScan:=DScan[b and $1F]
    else
     case b of
      $08:EvalScan:=$0E; { backspace }
      $09:EvalScan:=$0F; { TAB }
      $0D:EvalScan:=$1C; { CR }
      $1B:EvalScan:=$01; { esc }
      $40:EvalScan:=$03; { @ }
      $5E:EvalScan:=$07; { ^ }
      $60:EvalScan:=$29; { ` }
     else
      EvalScan:=LScan[b and $1F];
     end;
  end;

  function EvalScanZ(b:byte):byte;
  begin
    EvalScanZ:=b;
    if b in [$3B..$44] { F1..F10 -> Alt-F1..Alt-F10} then
     EvalScanZ:=b+$2D;
  end;

const
   {kbHome, kbUp, kbPgUp,Missing, kbLeft,
    kbCenter, kbRight, kbAltGrayPlus, kbend,
    kbDown, kbPgDn, kbIns, kbDel }
  CtrlArrow : array [kbHome..kbDel] of byte =
   {($77,$8d,$84,$8e,$73,$8f,$74,$90,$75,$91,$76);}
   (kbCtrlHome,kbCtrlUp,kbCtrlPgUp,kbNoKey,kbCtrlLeft,
    kbCtrlCenter,kbCtrlRight,kbAltGrayPlus,kbCtrlEnd,
    kbCtrlDown,kbCtrlPgDn,kbCtrlIns,kbCtrlDel);
  AltArrow : array [kbHome..kbDel] of byte =
   (kbAltHome,kbAltUp,kbAltPgUp,kbNoKey,kbAltLeft,
    kbCenter,kbAltRight,kbAltGrayPlus,kbAltEnd,
    kbAltDown,kbAltPgDn,kbAltIns,kbAltDel);
  ShiftArrow : array [kbShiftUp..kbShiftEnd] of byte =
   (kbUp,kbLeft,kbRight,kbDown,kbHome,kbEnd);

var
  MyScan:byte;
  MyChar : char;
  EscUsed,AltPrefixUsed,CtrlPrefixUsed,ShiftPrefixUsed,IsAlt,Again : boolean;
  SState:byte;

begin {main}
  MyChar:=Readkey(IsAlt);
  MyScan:=ord(MyChar);
{$ifdef linux}
  if is_console then
    SState:=ShiftState
  else
{$endif}
    Sstate:=0;
  CtrlPrefixUsed:=false;
  AltPrefixUsed:=false;
  ShiftPrefixUsed:=false;
  EscUsed:=false;
  if IsAlt then
    SState:=SState or kbAlt;
  repeat
    again:=false;
    if Mychar=#0 then
      begin
        MyScan:=ord(ReadKey(IsAlt));
        if myscan=$01 then
          mychar:=#27;
        { Handle Ctrl-<x>, but not AltGr-<x> }
        if ((SState and kbCtrl)<>0) and ((SState and kbAlt) = 0)  then
          case MyScan of
            kbHome..kbDel : { cArrow }
              MyScan:=CtrlArrow[MyScan];
            kbF1..KbF10 : { cF1-cF10 }
              MyScan:=MyScan+kbCtrlF1-kbF1;
            kbF11..KbF12 : { cF11-cF12 }
              MyScan:=MyScan+kbCtrlF11-kbF11;
          end
        { Handle Alt-<x>, but not AltGr }
        else if ((SState and kbAlt)<>0) and ((SState and kbCtrl) = 0) then
          case MyScan of
            kbHome..kbDel : { AltArrow }
              MyScan:=AltArrow[MyScan];
            kbF1..KbF10 : { aF1-aF10 }
              MyScan:=MyScan+kbAltF1-kbF1;
            kbF11..KbF12 : { aF11-aF12 }
              MyScan:=MyScan+kbAltF11-kbF11;
          end
        else if (SState and kbShift)<>0 then
          case MyScan of
            kbIns: MyScan:=kbShiftIns;
            kbDel: MyScan:=kbShiftDel;
            kbF1..KbF10 : { sF1-sF10 }
              MyScan:=MyScan+kbShiftF1-kbF1;
            kbF11..KbF12 : { sF11-sF12 }
              MyScan:=MyScan+kbShiftF11-kbF11;
          end;
        if myscan in [kbShiftUp..kbShiftEnd] then
          begin
            myscan:=ShiftArrow[myscan];
            sstate:=sstate or kbshift;
          end;
        if myscan=kbAltBack then
          sstate:=sstate or kbalt;
        if (MyChar<>#0) or (MyScan<>0) or (SState<>0) then
          SysGetKeyEvent:=$3000000 or ord(MyChar) or (MyScan shl 8) or (SState shl 16)
        else
          SysGetKeyEvent:=0;
        exit;
      end
    else if MyChar=#27 then
      begin
        if EscUsed then
          SState:=SState and not kbAlt
        else
          begin
            SState:=SState or kbAlt;
            Again:=true;
            EscUsed:=true;
          end;
      end
    else if (AltPrefix<>0) and (MyChar=chr(AltPrefix)) then
      begin { ^Z - replace Alt for Linux OS }
        if AltPrefixUsed then
          begin
            SState:=SState and not kbAlt;
          end
        else
          begin
            AltPrefixUsed:=true;
            SState:=SState or kbAlt;
            Again:=true;
          end;
      end
    else if (CtrlPrefix<>0) and (MyChar=chr(CtrlPrefix)) then
      begin
        if CtrlPrefixUsed then
          SState:=SState and not kbCtrl
        else
          begin
            CtrlPrefixUsed:=true;
            SState:=SState or kbCtrl;
            Again:=true;
          end;
      end
    else if (ShiftPrefix<>0) and (MyChar=chr(ShiftPrefix)) then
      begin
        if ShiftPrefixUsed then
          SState:=SState and not kbShift
        else
          begin
            ShiftPrefixUsed:=true;
            SState:=SState or kbShift;
            Again:=true;
          end;
      end;
    if not again then
      begin
        MyScan:=EvalScan(ord(MyChar));
        if ((SState and kbAlt)<>0) and ((SState and kbCtrl) = 0) then
          begin
            if MyScan in [$02..$0D] then
              inc(MyScan,$76);
            MyChar:=chr(0);
          end
        else if (SState and kbShift)<>0 then
          if MyChar=#9 then
            begin
              MyChar:=#0;
              MyScan:=kbShiftTab;
            end;
      end
    else
      begin
        MyChar:=Readkey(IsAlt);
        MyScan:=ord(MyChar);
        if IsAlt then
          SState:=SState or kbAlt;
      end;
    until not Again;
  if (MyChar<>#0) or (MyScan<>0) or (SState<>0) then
    SysGetKeyEvent:=$3000000 or ord(MyChar) or (MyScan shl 8) or (SState shl 16)
  else
    SysGetKeyEvent:=0;
end;


function SysPollKeyEvent: TKeyEvent;
var
  KeyEvent : TKeyEvent;
begin
  if keypressed then
    begin
      KeyEvent:=SysGetKeyEvent;
      PutKeyEvent(KeyEvent);
      SysPollKeyEvent:=KeyEvent
    end
  else
    SysPollKeyEvent:=0;
end;


function SysGetShiftState  : Byte;
begin
{$ifdef linux}
  if is_console then
    SysGetShiftState:=ShiftState
  else
{$else}
    SysGetShiftState:=0;
{$endif}
end;


procedure RestoreStartMode;
begin
  TCSetAttr(1,TCSANOW,StartTio);
end;


const
  SysKeyboardDriver : TKeyboardDriver = (
    InitDriver : @SysInitKeyBoard;
    DoneDriver : @SysDoneKeyBoard;
    GetKeyevent : @SysGetKeyEvent;
    PollKeyEvent : @SysPollKeyEvent;
    GetShiftState : @SysGetShiftState;
    TranslateKeyEvent : Nil;
    TranslateKeyEventUnicode : Nil;
  );

begin
  SetKeyBoardDriver(SysKeyBoardDriver);
  TCGetAttr(1,StartTio);
end.
