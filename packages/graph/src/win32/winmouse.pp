{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    This is unit implements a subset of the msmouse unit functionality
    for the gui win32 graph unit implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit winmouse;

  interface
    { initializes the mouse with the default values for the current screen mode }
    Function InitMouse:Boolean;

    { shows mouse pointer,text+graphics screen support }
    Procedure ShowMouse;

    { hides mouse pointer }
    Procedure HideMouse;

    { reads mouse position in pixels (divide by 8 to get text position in standard
      text mode) and reads the buttons state:
         bit 1 set -> left button pressed
         bit 2 set -> right button pressed
         bit 3 set -> middle button pressed
      Have a look at the example program in the manual to see how you can use this }
    Procedure GetMouseState(var x,y, buttons :Longint);

    { returns true if the left button is pressed }
    Function LPressed:Boolean;

    { returns true if the right button is pressed }
    Function RPressed:Boolean;

    { returns true if the middle button is pressed }
    Function MPressed:Boolean;

(*!!!!! the following functions aren't implemented yet:
    { positions the mouse pointer }
    Procedure SetMousePos(x,y:Longint);

    { returns at which position "button" was last pressed in x,y and returns the
      number of times this button has been pressed since the last time this
      function was called with "button" as parameter. For button you can use the
      LButton, RButton and MButton constants for resp. the left, right and middle
      button }
    Function GetLastButtonPress(button:Longint;var x,y:Longint): Longint;

    { returns at which position "button" was last released in x,y and returns the
      number of times this button has been re since the last time. For button
      you can use the LButton, RButton and MButton constants for resp. the left,
      right and middle button
    }
    Function GetLastButtonRelease (button : Longint; var x,y:Longint): Longint;

    { sets mouse's x range, with Min and Max resp. the higest and the lowest
      column (in pixels) in between which the mouse cursor can move }
    Procedure SetMouseXRange (Min,Max:Longint);

    { sets mouse's y range, with Min and Max resp. the higest and the lowest
      row (in pixels) in between which the mouse cursor can move}
    Procedure SetMouseYRange (Min,Max:Longint);

    { set the window coordinates in which the mouse cursor can move }
    Procedure SetMouseWindow(x1,y1,x2,y2:Longint);

    { sets the mouse shape in text mode: background and foreground color and the
      Ascii value with which the character on screen is XOR'ed when the cursor
      moves over it. Set to 0 for a "transparent" cursor}
    Procedure SetMouseShape(ForeColor,BackColor,Ascii:Byte);

    { sets the mouse ascii in text mode. The difference between this one and
      SetMouseShape, is that the foreground and background colors stay the same
      and that the Ascii code you enter is the character that you will get on
      screen; there's no XOR'ing }
    Procedure SetMouseAscii(Ascii:Byte);

    { set mouse speed in mickey's/pixel; default: horizontal: 8; vertical: 16 }
    Procedure SetMouseSpeed(Horizontal ,Vertical:Longint);

    { set a rectangle on screen that mouse will disappear if it is moved into }
    Procedure SetMouseHideWindow(x1,y1,x2,y2:Longint);
*)

    Const
       LButton = 1; { left button   }
       RButton = 2; { right button  }
       MButton = 4; { middle button }

    Var
       MouseFound: Boolean;

  implementation

    uses
       windows,graph;

    var
       oldexitproc : pointer;
       mousebuttonstate : byte;

    function InitMouse : boolean;

      begin
         InitMouse:=MouseFound;
      end;

    procedure ShowMouse;

      begin
         Windows.ShowCursor(true);
      end;

    procedure HideMouse;

      begin
         Windows.ShowCursor(false);
      end;

    function msghandler(Window: HWnd; AMessage:UInt; WParam : WParam; LParam: LParam): Longint; stdcall;

      begin
         { we catch the double click messages here too, }
         { even if they never appear because the graph  }
         { windows doesn't have the cs_dblclks flags    }
         case amessage of
            wm_lbuttondblclk,
            wm_lbuttondown:
              mousebuttonstate:=mousebuttonstate or LButton;
            wm_rbuttondblclk,
            wm_rbuttondown:
              mousebuttonstate:=mousebuttonstate or RButton;
            wm_mbuttondblclk,
            wm_mbuttondown:
              mousebuttonstate:=mousebuttonstate or MButton;
            wm_lbuttonup:
              mousebuttonstate:=mousebuttonstate and not(LButton);
            wm_rbuttonup:
              mousebuttonstate:=mousebuttonstate and not(RButton);
            wm_mbuttonup:
              mousebuttonstate:=mousebuttonstate and not(MButton);
         end;
         msghandler:=0;
      end;

    Function LPressed : Boolean;

      begin
         LPressed:=(mousebuttonstate and LButton)<>0;
      end;

    Function RPressed : Boolean;

      begin
         RPressed:=(mousebuttonstate and RButton)<>0;
      end;

    Function MPressed : Boolean;

      begin
         MPressed:=(mousebuttonstate and MButton)<>0;
      end;

    Procedure GetMouseState(var x,y,buttons : Longint);

      var
         pos : POINT;

      begin
         buttons:=mousebuttonstate;
         GetCursorPos(@pos);
         ScreenToClient(GraphWindow,@pos);
         x:=pos.x;
         y:=pos.y;
      end;

    procedure myexitproc;

      begin
         exitproc:=oldexitproc;
         mousemessagehandler:=nil;
      end;

  begin
     mousemessagehandler:=@msghandler;
     oldexitproc:=exitproc;
     exitproc:=@myexitproc;
     mousebuttonstate:=0;
     MouseFound:=GetSystemMetrics(SM_MOUSEPRESENT)<>0;
  end.
