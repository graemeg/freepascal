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

    function msghandler(Window: HWnd; AMessage:UInt; WParam : WParam; LParam: LParam): Longint; stdcall;

      begin
         case amessage of
           WM_CHAR:
             begin
                addchar(chr(wparam));
             end;
           WM_KEYDOWN:
             begin
                case wparam of
                   VK_LEFT:
                     addextchar(#75);
                   VK_RIGHT:
                     addextchar(#77);
                   VK_DOWN:
                     addextchar(#80);
                   VK_UP:
                     addextchar(#72);
                   VK_INSERT:
                     addextchar(#82);
                   VK_DELETE:
                     addextchar(#83);
                   VK_END:
                     addextchar(#79);
                   VK_HOME:
                     addextchar(#71);
                   VK_PRIOR:
                     addextchar(#73);
                   VK_NEXT:
                     addextchar(#81);
                   VK_F1..VK_F10:
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
