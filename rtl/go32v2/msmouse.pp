{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Mouse unit for microsoft mouse compatible drivers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************
}
Unit MSMouse;
Interface

{$calling oldfpccall}

{
  Mouse support functions and procedures, with error checking: if mouse
  isn't present then the routine ends. If you want to remove error checking,
  remove the next define.
}


{initializes the mouse with the default values for the current screen mode}
  Function InitMouse:Boolean;

{shows mouse pointer,text+graphics screen support}
  Procedure ShowMouse;

{hides mouse pointer}
  Procedure HideMouse;

{reads mouse position in pixels (divide by 8 to get text position in standard
 text mode) and reads the buttons state:
    bit 1 set -> left button pressed
    bit 2 set -> right button pressed
    bit 3 set -> middle button pressed
 Have a look at the example program in the manual to see how you can use this}
  Procedure GetMouseState(var x,y, buttons :Longint);

{returns true if the left button is pressed}
  Function LPressed:Boolean;

{returns true if the right button is pressed}
  Function RPressed:Boolean;

{returns true if the middle button is pressed}
  Function MPressed:Boolean;

{positions the mouse pointer}
  Procedure SetMousePos(x,y:Longint);

{returns at which position "button" was last pressed in x,y and returns the
 number of times this button has been pressed since the last time this
 function was called with "button" as parameter. For button you can use the
 LButton, RButton and MButton constants for resp. the left, right and middle
 button}
  Function GetLastButtonPress(button:Longint;var x,y:Longint): Longint;

{returns at which position "button" was last released in x,y and returns the
 number of times this button has been re since the last time. For button
 you can use the LButton, RButton and MButton constants for resp. the left,
 right and middle button}
Function GetLastButtonRelease (button : Longint; var x,y:Longint): Longint;

{sets mouse's x range, with Min and Max resp. the higest and the lowest
 column (in pixels) in between which the mouse cursor can move}
  Procedure SetMouseXRange (Min,Max:Longint);

{sets mouse's y range, with Min and Max resp. the higest and the lowest
 row (in pixels) in between which the mouse cursor can move}
  Procedure SetMouseYRange (Min,Max:Longint);

{set the window coordinates in which the mouse cursor can move}
  Procedure SetMouseWindow(x1,y1,x2,y2:Longint);

{sets the mouse shape in text mode: background and foreground color and the
 Ascii value with which the character on screen is XOR'ed when the cursor
 moves over it. Set to 0 for a "transparent" cursor}
  Procedure SetMouseShape(ForeColor,BackColor,Ascii:Byte);

{sets the mouse ascii in text mode. The difference between this one and
 SetMouseShape, is that the foreground and background colors stay the same
 and that the Ascii code you enter is the character that you will get on
 screen; there's no XOR'ing}
  Procedure SetMouseAscii(Ascii:Byte);

{set mouse speed in mickey's/pixel; default: horizontal: 8; vertical: 16}
  Procedure SetMouseSpeed(Horizontal ,Vertical:Longint);

{set a rectangle on screen that mouse will disappear if it is moved into}
  Procedure SetMouseHideWindow(x1,y1,x2,y2:Longint);

Const LButton = 1; {left button}
      RButton = 2; {right button}
      MButton = 4; {middle button}

Var
  MouseFound: Boolean;

Implementation

{$asmmode ATT}

Function InitMouse: Boolean;
begin
  asm
        xorl    %eax,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        cmpw    $0xffff,%ax
        setz    %al
        movb    %al,__RESULT
  end;
end;


Procedure ShowMouse;
begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $1,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
end;

Procedure HideMouse;
begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $2,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
end;

Procedure GetMouseState(var x,y,buttons:Longint);
begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        andl    $0xffff,%ebx
        andl    $0xffff,%ecx
        andl    $0xffff,%edx
        movl    x,%eax
        movl    %ecx,(%eax)
        movl    y,%eax
        movl    %edx,(%eax)
        movl    buttons,%eax
        movl    %ebx,(%eax)
  end;
end;

Function LPressed:Boolean;
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        movl    %ebx,%eax
        andl    $1,%eax
        movb    %al,__RESULT
  end;
end;

Function RPressed:Boolean;
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        movl    %ebx,%eax
        shrl    $1,%eax
        andl    $1,%eax
        movb    %al,__RESULT
  end;
end;

Function MPressed:Boolean;
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $3,%eax
        pushl   %ebp
        int     $0x33
        popl    %ebp
        movl    %ebx,%eax
        shrl    $2,%eax
        andl    $1,%eax
        movb    %al,__RESULT
  end;
end;

Procedure SetMousePos(x,y:Longint);
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $4,%eax
        movl    x,%ecx
        movl    y,%edx
        pushl   %ebp
        int     $0x33
        popl    %ebp
  End;
End;

Function GetLastButtonPress(Button: Longint;var x,y:Longint):Longint;
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $5,%eax
        movl    button,%ebx
        shrl    $1, %ebx        {0 = left, 1 = right, 2 = middle}
        pushl   %ebp
        int     $0x33
        popl    %ebp
        andl    $0xffff,%ebx
        andl    $0xffff,%edx
        andl    $0xffff,%ecx
        movl    %ebx, __RESULT
        movl    x,%eax
        movl    %ecx,(%eax)
        movl    y,%eax
        movl    %edx,(%eax)
  end;
end;

Function GetLastButtonRelease (button : Longint; var x,y:Longint): Longint;
begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $6,%eax
        movl    button,%ebx
        shrl    $1, %ebx        {0 = left, 1 = right, 2 = middle}
        pushl   %ebp
        int     $0x33
        popl    %ebp
        andl    $0xffff,%ebx
        andl    $0xffff,%ecx
        andl    $0xffff,%edx
        movl    %ebx,__RESULT
        movl    x,%eax
        movl    %ecx,(%eax)
        movl    y,%eax
        movl    %edx,(%eax)
  end;
end;

Procedure SetMouseXRange (Min,Max:Longint);
begin
  If (Not MouseFound) Then Exit;
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
  If (Not MouseFound) Then Exit;
  asm
        movl    $8,%eax
        movl    min,%ecx
        movl    max,%edx
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
end;

Procedure SetMouseWindow(x1,y1,x2,y2:Longint);
Begin
  If (Not MouseFound) Then Exit;
  SetMouseXRange(x1,x2);
  SetMouseYRange(y1,y2);
End;

Procedure SetMouseShape(ForeColor,BackColor,Ascii:Byte);
Begin
  If (Not MouseFound) Then Exit;
  asm
        xorl    %ebx,%ebx
        movl    $0xa,%eax
        movl    $0xffff,%ecx
        xorl    %edx,%edx
        movb    BackColor,%dh
        shlb    $4,%dh
        addb    ForeColor,%dh
        movb    Ascii,%dl
        pushl   %ebp
        int     $0x33
        popl    %ebp
  End;
End;

Procedure SetMouseAscii(Ascii:byte);
Begin
  If (Not MouseFound) Then Exit;
  asm
        xorl    %ebx,%ebx
        movl    $0xa,%eax
        movl    $0xff00,%ecx
        xorl    %edx,%edx
        movb    Ascii,%dl
        pushl   %ebp
        int     $0x33
        popl    %ebp
  End;
End;

Procedure SetMouseHideWindow(x1,y1,x2,y2:Longint);
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $0x0010,%eax
        movl    x1,%ecx
        movl    y1,%edx
        movl    x2,%esi
        movl    y2,%edi
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
End;

Procedure SetMouseSpeed(Horizontal,Vertical:Longint);
Begin
  If (Not MouseFound) Then Exit;
  asm
        movl    $0x0f,%eax
        movl    Horizontal,%ecx
        movl    Vertical,%edx
        pushl   %ebp
        int     $0x33
        popl    %ebp
  end;
End;

Begin
  MouseFound := InitMouse;
End.
