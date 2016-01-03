Program Moire;

{
      Will now open a default screen (can be any size) with
      the new look. The window get it's size depending on
      the screen size.
      14 May 1998

      Translated to FPC from PCQ Pascal.
      15 Aug 1998.

      Changed to use vartags and pas2c.
      18 Jan 2000.

      Removed opening of graphics.library.
      21 Mar 2001.

      Reworked to use systemvartags.
      28 Nov 2002.

      nils.sjoholm@mailbox.swipnet.se
}

uses Exec, Intuition, Graphics, Utility, systemvartags;


const
    pens : array [0..0] of Integer = ( not 0);


var
    w  : pWindow;
    s  : pScreen;
    m  : pMessage;


Procedure DoDrawing(RP : pRastPort);
var
    x  : word;
    Pen : Byte;
    Stop : word;
begin
    Pen := 1;
    while true do begin
    with w^ do begin
        x := 0;
        while x < Pred(Width - BorderRight - BorderLeft) do begin
        Stop := Pred(Width - BorderRight);
        SetAPen(RP, Pen);
        Move(RP, Succ(x + BorderLeft), BorderTop);
        Draw(RP, Stop - x, Pred(Height - BorderBottom));
        Pen := (Pen + 1) mod 4;
        Inc(x);
        end;
        m := GetMsg(UserPort);
        if m <> Nil then
        Exit;
        x := 0;
        while x < Pred(Height - BorderBottom - BorderTop) do begin
        Stop := Pred(Height - BorderBottom);
        SetAPen(RP, Pen);
        Move(RP, Pred(Width - BorderRight), Succ(x + BorderTop));
        Draw(RP, Succ(BorderLeft), Stop - x);
        Pen := (Pen + 1) mod 4;
        Inc(x);
        end;
        m := GetMsg(UserPort);
        if m <> Nil then
        Exit;
    end;
    end;
end;

begin
    { Note that the startup code of all FPC programs depends on
      Intuition, so if we got to this point Intuition must be
      open, so the run time library just uses the pointer that
      the startup code created.  Same with DOS, although we don't
      use that here. }



    s := OpenScreenTags(NIL, [
    SA_Pens,      @pens,
    SA_Depth,     2,
    SA_DisplayID, HIRES_KEY,
    SA_Title,     'Close the Window to End This Demonstration',
    TAG_END]);

    if s <> NIL then begin

    w := OpenWindowTags(NIL, [
    WA_IDCMP,        IDCMP_CLOSEWINDOW,
    WA_Left,         20,
    WA_Top,          50,
    WA_Width,        336,
    WA_Height,       100,
    WA_MinWidth,     50,
    WA_MinHeight,    20,
    WA_MaxWidth,     -1,
    WA_MaxHeight,    -1,
    WA_DepthGadget,  ltrue,
    WA_DragBar,      -1,
    WA_CloseGadget,  -1,
    WA_SizeGadget,   -1,
    WA_SmartRefresh, -1,
    WA_Activate,     -1,
    WA_Title,        'Feel Free to Re-Size the Window',
    WA_CustomScreen, s,
    TAG_END]);

    IF w <> NIL THEN begin

        DoDrawing(w^.RPort);
        Forbid;
        repeat
            m := GetMsg(w^.UserPort);
        until m = nil;
        CloseWindow(w);
        Permit;
        end else
        writeln('Could not open the window');
        CloseScreen(s);
    end else
        writeln('Could not open the screen.');
end.
