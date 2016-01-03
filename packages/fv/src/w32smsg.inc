{
   System independent system interface for win32

   Copyright (c) 2000 by Pierre Muller

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.


   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the Free
   Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
}

uses
   windows,dos,winevent;

var
   ChangeSystemEvents : TCriticalSection;
Const
  SystemEventActive : Boolean = false;

procedure SystemEventHandler(var ir:INPUT_RECORD);

  var
     e : TSystemEvent;

  begin
  { WINDOW_BUFFER_SIZE_EVENT is triggered by buffer size changes
    but we are interested in console size changes, thus its handled below
    in PollSystemEvent }
     if (ir.EventType in [FOCUS_EVENT{,WINDOW_BUFFER_SIZE_EVENT}]) then
       begin
          EnterCriticalSection(ChangeSystemEvents);
          if (ir.EventType=FOCUS_EVENT) then
            begin
              if ir.Event.FocusEvent.bSetFocus then
                e.typ:=SysSetFocus
              else
                e.typ:=SysReleaseFocus;
            end
          else
            begin
              e.typ:=SysResize;
              e.x:=ir.Event.WindowBufferSizeEvent.dwSize.x;
              e.y:=ir.Event.WindowBufferSizeEvent.dwSize.y;
            end;
          PutSystemEvent(e);
          LeaveCriticalSection(ChangeSystemEvents);
       end;
  end;


var
  Xsize, YSize : longint;

function HandleConsoleCtrl(typ : dword) : BOOL; stdcall;
var
  SE :   TSystemEvent;
begin
  HandleConsoleCtrl:=false;
  case typ of
    CTRL_CLOSE_EVENT,
    CTRL_LOGOFF_EVENT,
    CTRL_SHUTDOWN_EVENT :
      begin
        SE.typ:=SysClose;
        SE.CloseTyp:=typ;
        PutSystemEvent(SE);
        HandleConsoleCtrl:=true;
      end;
  end;
end;


procedure InitSystemMsg;

var
   mode : dword;
   ConsoleScreenBufferInfo : Console_screen_buffer_info;

begin
  if SystemEventActive then
    exit;
  // enable Window events
  GetConsoleMode(TextRec(Input).Handle,@mode);
  mode:=mode or ENABLE_WINDOW_INPUT;
  SetConsoleMode(TextRec(Input).Handle,mode);
  GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE),
    @ConsoleScreenBufferInfo);
  XSize:=ConsoleScreenBufferInfo.srWindow.right-ConsoleScreenBufferInfo.srWindow.left+1;
  YSize:=ConsoleScreenBufferInfo.srWindow.bottom-ConsoleScreenBufferInfo.srWindow.top+1;
  PendingSystemHead:=@PendingSystemEvent;
  PendingSystemTail:=@PendingSystemEvent;
  PendingSystemEvents:=0;
  FillChar(LastSystemEvent,sizeof(TSystemEvent),0);
  InitializeCriticalSection(ChangeSystemEvents);
  SetResizeEventHandler(@SystemEventHandler);
  SetFocusEventHandler(@SystemEventHandler);
  SetConsoleCtrlHandler(@HandleConsoleCtrl,true);
  SystemEventActive:=true;
end;


procedure DoneSystemMsg;
var
   mode : dword;
begin
  if not SystemEventActive then
    exit;
  // disable System events
  GetConsoleMode(TextRec(Input).Handle,@mode);
  mode:=mode and (not ENABLE_WINDOW_INPUT);
  SetConsoleMode(TextRec(Input).Handle,mode);

  SetResizeEventHandler(nil);
  SetFocusEventHandler(nil);
  DeleteCriticalSection(ChangeSystemEvents);
  SetConsoleCtrlHandler(@HandleConsoleCtrl,false);
  SystemEventActive:=false;
end;



procedure GetSystemEvent(var SystemEvent: TSystemEvent);

var
   b : byte;

begin
  repeat
    EnterCriticalSection(ChangeSystemEvents);
    b:=PendingSystemEvents;
    LeaveCriticalSection(ChangeSystemEvents);
    if b>0 then
      break
    else
      sleep(50);
  until false;
  EnterCriticalSection(ChangeSystemEvents);
  SystemEvent:=PendingSystemHead^;
  inc(PendingSystemHead);
  if longint(PendingSystemHead)=longint(@PendingSystemEvent)+sizeof(PendingSystemEvent) then
   PendingSystemHead:=@PendingSystemEvent;
  dec(PendingSystemEvents);
  LastSystemEvent:=SystemEvent;
  LeaveCriticalSection(ChangeSystemEvents);
end;


function PollSystemEvent(var SystemEvent: TSystemEvent):boolean;
var
   ConsoleScreenBufferInfo : Console_screen_buffer_info;
   NewXSize, NewYSize : longint;
begin
  EnterCriticalSection(ChangeSystemEvents);
  if PendingSystemEvents>0 then
   begin
     SystemEvent:=PendingSystemHead^;
     PollSystemEvent:=true;
   end
  else
   begin
     GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE),
       @ConsoleScreenBufferInfo);
     NewXSize:=ConsoleScreenBufferInfo.srWindow.right-ConsoleScreenBufferInfo.srWindow.left+1;
     NewYSize:=ConsoleScreenBufferInfo.srWindow.bottom-ConsoleScreenBufferInfo.srWindow.top+1;
     if (XSize<>NewXSize) or (YSize<>NewYSize) then
       begin
         SystemEvent.typ:=SysResize;
         SystemEvent.x:=NewXSize;
         SystemEvent.y:=NewYSize;
         PutSystemEvent(SystemEvent);
         XSize:=NewXSize;
         YSize:=NewYSize;
         PollSystemEvent:=true;
       end
     else
       PollSystemEvent:=false;
   end;
  LeaveCriticalSection(ChangeSystemEvents);
end;

