Program Bezier;


{
   This program draws Bezier curves using the degree elevation
   method.  For large numbers of points (more than 10, for
   example) this is faster than the recursive way.
}

{
   History:
   Changed the source to use 2.0+.
   Looks a lot better.
   Added CloseWindowSafely.
   Made the window dynamic, it will
   adjust the size after the screen size.
   9 May 1998.

   Translated the source to fpc.
   20 Aug 1998.

   Changed to use TAGS and pas2c.
   31 Oct 1998.

   Removed Opening of graphics.library,
   handled by graphics.pas.
   21 Mar 2001.

   Uses systemvartags and
   OpenScreenTags
   OpenWindowTags
   Text to GText.
   09 Nov 2002.

   nils.sjoholm@mailbox.swipnet.se
}

uses exec, intuition, graphics, utility,pastoc, systemvartags;

type
    PointRec = packed Record
        X, Y : Real;
    end;

Const
    w  : pWindow  = Nil;
    s  : pScreen   = Nil;

{
    This will make the new look for screen.
    SA_Pens, Integer(pens)
}
    pens : array [0..0] of integer = (not 0);

Var
    rp : pRastPort;

    PointCount : Word;
    Points : Array [1..200] of PointRec;

    LastX, LastY : Word;

Procedure CleanUpAndDie;
begin
    if assigned(w) then CloseWindow(w);
    if assigned(s) then CloseScreen(s);
    Halt(0);
end;

Procedure DrawLine;
begin
    Move(rp, Trunc(Points[PointCount].X), Trunc(Points[PointCount].Y));
    Draw(rp, LastX, LastY);
end;

Procedure GetPoints;
var
    LastSeconds,
    LastMicros  : Longint;
    IM : pIntuiMessage;
    StoreMsg : tIntuiMessage;
    Leave : Boolean;
    OutOfBounds : Boolean;
    BorderLeft, BorderRight,
    BorderTop, BorderBottom : Word;
    dummy : Boolean;

    Procedure AddPoint;
    begin
    Inc(PointCount);
    with Points[PointCount] do begin
        X := Real(StoreMsg.MouseX);
        Y := Real(StoreMsg.MouseY);
    end;
    with StoreMsg do begin
        LastX := MouseX;
        LastY := MouseY;
        LastSeconds := Seconds;
        LastMicros := Micros;
    end;
    SetAPen(rp, 2);
    SetDrMd(rp, JAM1);
    DrawEllipse(rp, LastX, LastY, 5, 3);
    SetAPen(rp, 3);
    SetDrMd(rp, COMPLEMENT);
    DrawLine;
    end;

    Function CheckForExit : Boolean;
    {   This function determines whether the user wanted to stop
    entering points.  I added the position tests because my
    doubleclick time is too long, and I was too lazy to dig
    out Preferences to change it. }
    begin
    with StoreMsg do
        CheckForExit := DoubleClick(LastSeconds, LastMicros,
                    Seconds, Micros) and
                (Abs(MouseX - Trunc(Points[PointCount].X)) < 5) and
                (Abs(MouseY - TRunc(Points[PointCount].Y)) < 3);
    end;

    Procedure ClearIt;
    {  This just clears the screen when you enter your first point }
    begin
    SetDrMd(rp, JAM1);
    SetAPen(rp, 0);
    RectFill(rp, BorderLeft, BorderTop,
             BorderRight, BorderBottom);
    SetDrMd(rp, COMPLEMENT);
    SetAPen(rp, 3);
    end;

begin
    dummy := ModifyIDCMP(w, IDCMP_CLOSEWINDOW or IDCMP_MOUSEBUTTONS or
IDCMP_MOUSEMOVE);
    SetDrMd(rp, COMPLEMENT);
    PointCount := 0;
    Leave := False;
    OutOfBounds := False;
    BorderLeft := w^.BorderLeft;
    BorderRight := (w^.Width - w^.BorderRight) -1;
    BorderTop := w^.BorderTop;
    BorderBottom := (w^.Height - w^.BorderBottom) -1;
    repeat
        IM := pIntuiMessage(WaitPort(w^.UserPort));
        IM := pIntuiMessage(GetMsg(w^.UserPort));
        StoreMsg := IM^;
        ReplyMsg(pMessage(IM));
        case StoreMsg.IClass of
           IDCMP_MOUSEMOVE : if PointCount > 0 then begin
                 if not OutOfBounds then
                 DrawLine;
                     LastX := StoreMsg.MouseX;
                     LastY := StoreMsg.MouseY;
                 if (LastX > BorderLeft) and
                (LastX < BorderRight) and
                (LastY > BorderTop) and
                (LastY < BorderBottom) then begin
                 DrawLine;
                 OutOfBounds := False;
                 end else
                 OutOfBounds := True;
                 end;
           IDCMP_MOUSEBUTTONS : if StoreMsg.Code = SELECTUP then begin
                    if PointCount > 0 then
                    Leave := CheckForExit
                else
                    ClearIt;
                    if (not Leave) and (not OutOfBounds) then
                    AddPoint;
                    end;
           IDCMP_CLOSEWINDOW : CleanUpAndDie;
        end;
    until Leave or (PointCount > 50);
    if not Leave then
        DrawLine;
    dummy := ModifyIDCMP(w, IDCMP_CLOSEWINDOW);
    SetDrMd(rp, JAM1);
    SetAPen(rp, 1);
end;

Procedure Elevate;
var
    t, tprime,
    RealPoints : Real;
    i : Integer;
begin
    Inc(PointCount);
    RealPoints := Real(PointCount);
    Points[PointCount] := Points[Pred(PointCount)];
    for i := Pred(PointCount) downto 2 do
    with Points[i] do begin
        t := Real(i) / RealPoints;
        tprime := 1.0 - t;
        X := t * Points[Pred(i)].X + tprime * X;
        Y := t * Points[Pred(i)].Y + tprime * Y;
    end;
end;

Procedure DrawCurve;
var
    i : Integer;
begin
    Move(rp, Trunc(Points[1].X), Trunc(Points[1].Y));
    for i := 2 to PointCount do
    Draw(rp, Round(Points[i].X), Round(Points[i].Y));
end;

Procedure DrawBezier;
begin
    SetAPen(rp, 2);
    while PointCount < 100 do begin
    Elevate;
    DrawCurve;
    if GetMsg(w^.UserPort) <> Nil then
        CleanUpAndDie;
    end;
    SetAPen(rp, 1);
    DrawCurve;
end;

begin

   s := OpenScreenTags(nil,[SA_Pens,@pens,
      SA_Depth,     2,
      SA_DisplayID, HIRES_KEY,
      SA_Title,     'Simple Bezier Curves',
      TAG_END]);

    if s = NIL then CleanUpAndDie;

      w := OpenWindowTags(nil,[
      WA_IDCMP,        IDCMP_CLOSEWINDOW,
      WA_Left,         0,
      WA_Top,          s^.BarHeight +1,
      WA_Width,        s^.Width,
      WA_Height,       s^.Height - (s^.BarHeight + 1),
      WA_DepthGadget,  ltrue,
      WA_DragBar,      ltrue,
      WA_CloseGadget,  ltrue,
      WA_ReportMouse,  ltrue,
      WA_SmartRefresh, ltrue,
      WA_Activate,     ltrue,
      WA_Title,        'Close the Window to Quit',
      WA_CustomScreen, s,
      TAG_END]);

    IF w=NIL THEN CleanUpAndDie;

    rp := w^.RPort;
    Move(rp, 252, 30);
    GText(rp, pas2c('Enter points by pressing the left mouse button'), 46);
    Move(rp, 252, 40);
    GText(rp, pas2c('Double click on the last point to begin drawing'), 47);
    repeat
        GetPoints;  { Both these routines will quit if }
        DrawBezier; { the window is closed. }
    until False;
    CleanUpAndDie;
end.
