Program Snowflake;

{ This program draws a fractal snowflake pattern.  I think I got it out
of some magazine years ago.  It was written, as I remember it, for the
PC in BASIC, which I converted to AmigaBASIC.  I have long since
forgotten the details of how it worked, so I could not give the
variables meaningful names.  To the original author, by the way, goes
the credit for those names.  Invoke the program with the line "Snow
<level>", where <level> is a digit between 1 and 6.  In order to get a
feel for what's going on, try running the levels in order.  Level 6
takes a long time, and frankly doesn't look as good as level 5.  }

{
   Translated to fpc pascal from pcq pascal.
   Updated the source to the new style. Will
   now also open a screen.
   04 Apr 2001.

   Reworked to use systemvartags.
   28 Nov 2002.

   nils.sjoholm@mailbox.swipnet.se
}


uses exec,intuition,graphics,utility,systemvartags;



var
    dx : array [0..11] of real;
    dy : array [0..11] of real;
    sd : array [0..6] of Longint;
    rd : array [0..6] of Longint;
    sn : array [0..6] of Longint;
    ln : array [0..6] of real;
    a  : real;
    nc : Longint;
    x, y, t : real;
    w  : pWindow;
    s  : pScreen;
    rp : pRastPort;
    n  : Longint;
    d, ns, i, j : Longint;
    l : real;
    m : pMessage;

const
     pens : array [0..0] of integer = (not 0);

Procedure usage;
begin
    writeln('Usage: Snow <level>');
    writeln('       where <level> is between 1 and 6');
    halt(20);
end;

procedure CleanUp(why : string; err : longint);
begin
    if assigned(w) then CloseWindow(w);
    if assigned(s) then CloseScreen(s);
    if why <> '' then writeln(why);
    halt(err);
end;

Function readcycles: Longint;
var
    cycles : Longint;
begin
    if paramcount <> 1 then usage;
    cycles := ord(paramstr(1)[1]) - ord('0');
    if (cycles > 6) or (cycles < 1) then
        usage;
    readcycles := cycles;
end;


procedure initarrays;
begin
    sd[0] := 0;
    rd[0] := 0;
    sd[1] := 1;
    rd[1] := 0;
    sd[2] := 1;
    rd[2] := 7;
    sd[3] := 0;
    rd[3] := 10;
    sd[4] := 0;
    rd[4] := 0;
    sd[5] := 0;
    rd[5] := 2;
    sd[6] := 1;
    rd[6] := 2;

    for n := 0 to 6 do
        ln[n] := 1.0 / 3.0;
    ln[2] := sqrt(ln[1]);
    a := 0.0;
    for n := 6 to 11 do begin
        dy[n] := sin(a);
        dx[n] := cos(a);
        a := a + 0.52359;
    end;
    for n := 0 to 5 do begin
        dx[n] := -(dx[n + 6]);
        dy[n] := -(dy[n + 6]);
    end;
    x := 534.0;
    y := 151.0;
    t := 324.0;
end;

begin
    nc := readcycles();
    initarrays;

    s := OpenScreenTags(nil, [SA_Pens,   @pens,
      SA_Depth,     2,
      SA_DisplayID, HIRES_KEY,
      SA_Title,     'Simple Fractal SnowFlakes',
      TAG_END]);

    if s = NIL then CleanUp('No screen',20);

      w := OpenWindowTags(nil, [
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

    if w = nil then CleanUp('No window',20);

        rp := w^.RPort;
        SetAPen(rp,2);
        for n := 0 to nc do
            sn[n] := 0;

        Move(rp, trunc(x), trunc(y));

        repeat
            d := 0;
            l := t;
            ns := 0;

            for n := 1 to nc do begin
                i := sn[n];
                l := l * ln[i];
                j := sn[n - 1];
                ns := ns + sd[j];
                if odd(ns) then
                    d := (d + 12 - rd[i]) mod 12
                else
                    d := (d + rd[i]) mod 12;
            end;

            x := x + 1.33 * l * dx[d];
            y := y - 0.5 * l * dy[d];

            Draw(rp, trunc(x), trunc(y));
            sn[nc] := sn[nc] + 1;
            n := nc;
            while (n >= 1) and (sn[n] = 7) do begin
                sn[n] := 0;
                sn[n - 1] := sn[n - 1] + 1;
                n := n - 1;
            end;
        until sn[0] <> 0;
        m := WaitPort(w^.UserPort);
        forbid;
        repeat
            m := GetMsg(w^.UserPort);
        until m = nil;
        permit;
        CleanUp('',0);

end.
