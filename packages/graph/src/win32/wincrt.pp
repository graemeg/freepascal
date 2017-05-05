{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    This is unit implements some of the crt functionality
    for the gui win32 graph unit implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit wincrt;

  interface

    function readkey : char;
    function keypressed : boolean;
    procedure delay(ms : word);

    { dummy }
    procedure textmode(mode : integer);

    { plays the windows standard sound }
    { hz is ignored (at least on win95 }
    procedure sound(hz : word);

    { dummy }
    procedure nosound;


  var
     directvideo : boolean;

     { dummy }
     lastmode : word;

  implementation

    uses
       windows,graph;

    const
       keybuffersize = 32;

    var
       keyboardhandling : TCriticalSection;
       keybuffer : array[1..keybuffersize] of char;
       nextfree,nexttoread : longint;

    procedure inccyclic(var i : longint);

      begin
         inc(i);
         if i>keybuffersize then
           i:=1;
      end;

    procedure addchar(c : char);

      begin
         EnterCriticalSection(keyboardhandling);
         keybuffer[nextfree]:=c;
         inccyclic(nextfree);
         { skip old chars }
         if nexttoread=nextfree then
           begin
              // special keys are started by #0
              // so we've to remove two chars
              if keybuffer[nexttoread]=#0 then
                inccyclic(nexttoread);
              inccyclic(nexttoread);
           end;
         LeaveCriticalSection(keyboardhandling);
      end;

    function readkey : char;

      begin
         while true do
           begin
              EnterCriticalSection(keyboardhandling);
              if nexttoread<>nextfree then
                begin
                   readkey:=keybuffer[nexttoread];
                   inccyclic(nexttoread);
                   LeaveCriticalSection(keyboardhandling);
                   exit;
                end;
              LeaveCriticalSection(keyboardhandling);
              { give other threads a chance }
              Windows.Sleep(10);
           end;
      end;

    function keypressed : boolean;

      begin
         EnterCriticalSection(keyboardhandling);
         keypressed:=nexttoread<>nextfree;
         LeaveCriticalSection(keyboardhandling);
      end;

    procedure delay(ms : word);

      begin
         Sleep(ms);
      end;

    procedure textmode(mode : integer);

      begin
      end;

    procedure sound(hz : word);

      begin
         Windows.Beep(hz,500);
      end;

    procedure nosound;

      begin
      end;

    procedure addextchar(c : char);

      begin
         addchar(#0);
         addchar(c);
      end;

    const
       altkey : boolean = false;
       ctrlkey : boolean = false;
       shiftkey : boolean = false;

    function msghandler(Window: HWnd; AMessage:UInt; WParam : WParam; LParam: LParam): LResult; stdcall;

      begin
         case amessage of
           WM_CHAR:
             begin
                addchar(chr(wparam));
             end;
           WM_KEYDOWN:
             begin
                case wparam of
                   49..57:
                     if ctrlkey then addextchar(chr(wparam-47));
                   48:
                     if ctrlkey then addextchar(#11);
                   189:
                     if ctrlkey then addextchar(#12);
                   187:
                     if ctrlkey then addextchar(#13);
                   222:
                     if ctrlkey then addextchar(#39);
                   186:
                     if ctrlkey then addextchar(#40);
                   192:
                     if ctrlkey then addextchar(#41);
                   188:
                     if ctrlkey then addextchar(#51);
                   190:
                     if ctrlkey then addextchar(#52);
                   191,111:
                     if ctrlkey then addextchar(#149);
                   106:
                     if ctrlkey then addextchar(#150);
                   109:
                     if ctrlkey then addextchar(#142);
                   103:
                     if ctrlkey then addextchar(#119);
                   104:
                     if ctrlkey then addextchar(#141);
                   105:
                     if ctrlkey then addextchar(#132);
                   107:
                     if ctrlkey then addextchar(#78);
                   100:
                     if ctrlkey then addextchar(#115);
                   101:
                     if ctrlkey then addextchar(#143);
                   102:
                     if ctrlkey then addextchar(#116);
                    97:
                     if ctrlkey then addextchar(#117);
                    98:
                     if ctrlkey then addextchar(#145);
                    99:
                     if ctrlkey then addextchar(#118);
                    96:
                     if ctrlkey then addextchar(#146);
                   110:
                     if ctrlkey then addextchar(#147);
                   VK_LEFT:
                     begin
                        if ctrlkey then
                         addextchar(#115)
                        else
                         addextchar(#75);
                      end;
                   VK_RIGHT:
                     begin
                        if ctrlkey then
                         addextchar(#116)
                        else
                         addextchar(#77);
                      end;
                   VK_DOWN:
                     begin
                        if ctrlkey then
                         addextchar(#145)
                        else
                         addextchar(#80);
                      end;
                   VK_UP:
                     begin
                        if ctrlkey then
                         addextchar(#141)
                        else
                         addextchar(#72);
                      end;
                   VK_INSERT:
                     begin
                        if ctrlkey then
                         addextchar(#146)
                        else
                         addextchar(#82);
                      end;
                   VK_DELETE:
                     begin
                        if ctrlkey then
                         addextchar(#147)
                        else
                         addextchar(#83);
                      end;
                   VK_END:
                     begin
                        if ctrlkey then
                         addextchar(#117)
                        else
                         addextchar(#79);
                      end;
                   VK_HOME:
                     begin
                        if ctrlkey then
                         addextchar(#119)
                        else
                         addextchar(#71);
                      end;
                   VK_PRIOR:
                     begin
                        if ctrlkey then
                         addextchar(#132)
                        else
                         addextchar(#73);
                      end;
                   VK_NEXT:
                     begin
                        if ctrlkey then
                         addextchar(#118)
                        else
                         addextchar(#81);
                      end;
                   VK_F1..VK_F12:
                     begin
                        if ctrlkey then
                          addextchar(chr(wparam+24))
                        else if altkey then
                          addextchar(chr(wparam+34))
                        else
                          addextchar(chr(wparam-11));
                     end;
                   VK_CONTROL:
                     ctrlkey:=true;
                   VK_MENU:
                     altkey:=true;
                   VK_SHIFT:
                     shiftkey:=true;
                end;
             end;
           WM_KEYUP:
             begin
                case wparam of
                   VK_CONTROL:
                     ctrlkey:=false;
                   VK_MENU:
                     altkey:=false;
                   VK_SHIFT:
                     shiftkey:=false;
                end;
             end;
         end;
         msghandler:=0;
      end;

    var
       oldexitproc : pointer;

    procedure myexitproc;

      begin
         exitproc:=oldexitproc;
         charmessagehandler:=nil;
         DeleteCriticalSection(keyboardhandling);
      end;

begin
   charmessagehandler:=@msghandler;
   nextfree:=1;
   nexttoread:=1;
   InitializeCriticalSection(keyboardhandling);
   oldexitproc:=exitproc;
   exitproc:=@myexitproc;
   lastmode:=0;
end.
