{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Event Handling unit for setting Keyboard and Mouse Handlers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit WinEvent;
interface

{
   We need this unit to implement keyboard and mouse,
   because win32 uses only one message queue for mouse and key events
}

    uses
       Windows;

    type
       TEventProcedure = Procedure(var ir:INPUT_RECORD);

    { these procedures must be used to set the event handlers }
    { these doesn't do something, they signal only the        }
    { the upper layer that an event occured, this event       }
    { must be handled with Win32-API function by the upper    }
    { layer                                                   }
    Procedure SetMouseEventHandler(p : TEventProcedure);
    Procedure SetKeyboardEventHandler(p : TEventProcedure);
    Procedure SetFocusEventHandler(p : TEventProcedure);
    Procedure SetMenuEventHandler(p : TEventProcedure);
    Procedure SetResizeEventHandler(p : TEventProcedure);
    Procedure SetUnknownEventHandler(p : TEventProcedure);

    { these procedures must be used to get the event handlers }
    Function GetMouseEventHandler : TEventProcedure;
    Function GetKeyboardEventHandler : TEventProcedure;
    Function GetFocusEventHandler : TEventProcedure;
    Function GetMenuEventHandler : TEventProcedure;
    Function GetResizeEventHandler : TEventProcedure;
    Function GetUnknownEventHandler : TEventProcedure;

  implementation

    const
       { these procedures are called if an event occurs }
       MouseEventHandler : TEventProcedure = nil;
       KeyboardEventHandler : TEventProcedure = nil;
       FocusEventHandler : TEventProcedure = nil;
       MenuEventHandler : TEventProcedure = nil;
       ResizeEventHandler : TEventProcedure = nil;
       UnknownEventHandler  : TEventProcedure = nil;

       { if this counter is zero, the event handler thread is killed }
       InstalledHandlers : Byte = 0;

    var
       HandlerChanging : TCriticalSection;
       EventThreadHandle : Handle;
       EventThreadID : DWord;

       { true, if the event handler should be stoped }
       ExitEventHandleThread : boolean;

    Function GetMouseEventHandler : TEventProcedure;
      begin
         GetMouseEventHandler:=MouseEventHandler;
      end;


    Function GetKeyboardEventHandler : TEventProcedure;
      begin
         GetKeyboardEventHandler:=KeyboardEventHandler;
      end;


    Function GetFocusEventHandler : TEventProcedure;
      begin
         GetFocusEventHandler:=FocusEventHandler;
      end;


    Function GetMenuEventHandler : TEventProcedure;
      begin
         GetMenuEventHandler:=MenuEventHandler;
      end;


    Function GetResizeEventHandler : TEventProcedure;
      begin
         GetResizeEventHandler:=ResizeEventHandler;
      end;


    Function GetUnknownEventHandler : TEventProcedure;
      begin
         GetUnknownEventHandler:=UnknownEventHandler;
      end;


    Function EventHandleThread(p : pointer) : DWord;StdCall;
      const
        irsize = 10;
      var
         ir : array[0..irsize-1] of TInputRecord;
         i,dwRead : DWord;
      begin
         while not(ExitEventHandleThread) do
           begin
              { wait for an event }
              WaitForSingleObject(StdInputHandle,INFINITE);
              { guard this code, else it is doomed to crash, if the
                thread is switched between the assigned test and
                the call and the handler is removed
              }
              if not(ExitEventHandleThread) then
                begin
                   if ReadConsoleInput(StdInputHandle,ir[0],irsize,dwRead) then
                    begin
                      i:=0;
                      EnterCriticalSection(HandlerChanging);
                      while i<dwRead do
                        begin
                          { call the handler }
                          case ir[i].EventType of
                            KEY_EVENT:
                              begin
                                 if assigned(KeyboardEventHandler) then
                                   KeyboardEventHandler(ir[i]);
                              end;

                            _MOUSE_EVENT:
                              begin
                                 if assigned(MouseEventHandler) then
                                   MouseEventHandler(ir[i]);
                              end;

                            WINDOW_BUFFER_SIZE_EVENT:
                              begin
                                 if assigned(ResizeEventHandler) then
                                   ResizeEventHandler(ir[i]);
                              end;

                            MENU_EVENT:
                              begin
                                 if assigned(MenuEventHandler) then
                                   MenuEventHandler(ir[i]);
                              end;

                            FOCUS_EVENT:
                              begin
                                 if assigned(FocusEventHandler) then
                                   FocusEventHandler(ir[i]);
                              end;

                            else
                              begin
                                 if assigned(UnknownEventHandler) then
                                   UnknownEventHandler(ir[i]);
                              end;
                           end;
                         inc(i);
                      end;
                      LeaveCriticalSection(HandlerChanging);
                    end;
                end;
           end;
        EventHandleThread:=0;
      end;

    Procedure NewEventHandlerInstalled(p,oldp : TEventProcedure);
      var
         oldcount : Byte;
         ir : TInputRecord;
         written : DWord;
      begin
         oldcount:=InstalledHandlers;
         if Pointer(oldp)<>nil then
           dec(InstalledHandlers);
         if Pointer(p)<>nil then
           inc(InstalledHandlers);
         { start event handler thread }
         if (oldcount=0) and (InstalledHandlers=1) then
           begin
              ExitEventHandleThread:=false;
              EventThreadHandle:=CreateThread(nil,0,@EventHandleThread,
                nil,0,EventThreadID);
           end
         { stop and destroy event handler thread }
         else if (oldcount=1) and (InstalledHandlers=0) then
           begin
              ExitEventHandleThread:=true;
              { create a dummy event and sent it to the thread, so
                we can leave WaitForSingleObject }
              ir.EventType:=KEY_EVENT;
              { mouse event can be disabled by mouse.inc code
                in DoneMouse
                so use a key event instead PM }
              WriteConsoleInput(StdInputHandle,ir,1,written);
              { wait, til the thread is ready }
              WaitForSingleObject(EventThreadHandle,INFINITE);
              CloseHandle(EventThreadHandle);
           end;
      end;


    Procedure SetMouseEventHandler(p : TEventProcedure);
      var
         oldp : TEventProcedure;
      begin
         EnterCriticalSection(HandlerChanging);
         oldp:=MouseEventHandler;
         MouseEventHandler:=p;
         LeaveCriticalSection(HandlerChanging);
         NewEventHandlerInstalled(MouseEventHandler,oldp);
      end;


    Procedure SetKeyboardEventHandler(p : TEventProcedure);
      var
         oldp : TEventProcedure;
      begin
         EnterCriticalSection(HandlerChanging);
         oldp:=KeyboardEventHandler;
         KeyboardEventHandler:=p;
         LeaveCriticalSection(HandlerChanging);
         NewEventHandlerInstalled(KeyboardEventHandler,oldp);
      end;


    Procedure SetFocusEventHandler(p : TEventProcedure);
      var
         oldp : TEventProcedure;
      begin
         EnterCriticalSection(HandlerChanging);
         oldp:=FocusEventHandler;
         FocusEventHandler:=p;
         LeaveCriticalSection(HandlerChanging);
         NewEventHandlerInstalled(FocusEventHandler,oldp);
      end;


    Procedure SetMenuEventHandler(p : TEventProcedure);
      var
         oldp : TEventProcedure;
      begin
         EnterCriticalSection(HandlerChanging);
         oldp:=MenuEventHandler;
         MenuEventHandler:=p;
         LeaveCriticalSection(HandlerChanging);
         NewEventHandlerInstalled(MenuEventHandler,oldp);
      end;


    Procedure SetResizeEventHandler(p : TEventProcedure);
      var
         oldp : TEventProcedure;
      begin
         EnterCriticalSection(HandlerChanging);
         oldp:=ResizeEventHandler;
         ResizeEventHandler:=p;
         LeaveCriticalSection(HandlerChanging);
         NewEventHandlerInstalled(ResizeEventHandler,oldp);
      end;


    Procedure SetUnknownEventHandler(p : TEventProcedure);
      var
         oldp : TEventProcedure;
      begin
         EnterCriticalSection(HandlerChanging);
         oldp:=UnknownEventHandler;
         UnknownEventHandler:=p;
         LeaveCriticalSection(HandlerChanging);
         NewEventHandlerInstalled(UnknownEventHandler,oldp);
      end;


initialization
   InitializeCriticalSection(HandlerChanging);

finalization
  { Uninstall all handlers                   }
  { this stops also the event handler thread }
  SetMouseEventHandler(nil);
  SetKeyboardEventHandler(nil);
  SetFocusEventHandler(nil);
  SetMenuEventHandler(nil);
  SetResizeEventHandler(nil);
  SetUnknownEventHandler(nil);
  { delete the critical section object }
  DeleteCriticalSection(HandlerChanging);

end.
