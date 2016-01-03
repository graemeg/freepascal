PROGRAM Talk2Boopsi;

{ This example creates a Boopsi prop gadget and integer string gadget, connecting them
so they }
{ update each other when the user changes their value.  The example program only
initializes   }
{ the gadgets and puts them on the window; it doesn't have to interact with them to
make them  }
{ talk to each other.
}

{
    History:

    Changed to use TAGS.
    16 Jul 2000.

    Added MessageBox for report.
    31 Jul 2000.

    Changed to use array of const.
    OpenWindowTags,
    NewObject and
    SetGadgetAttrs
    12 Nov 2002.

    nils.sjoholm@mailbox.swipnet.se

}

uses Exec, Intuition, Utility,msgbox, systemvartags;



VAR
   w      : pWindow;
   mymsg  : pIntuiMessage;
   prop,
   int    : pGadget;
   done   : BOOLEAN;
   dummy  : Word;
   temp   : Longint;
   prop2intmap : array[0..1] of tTagItem;
   int2propmap : array[0..1] of tTagItem;

CONST

   vers  : PChar = '$VER: Talk2boopsi 37.1';

   PROPGADGET_ID       = 1;
   INTGADGET_ID        = 2;
   PROPGADGETWIDTH     = 10;
   PROPGADGETHEIGHT    = 80;
   INTGADGETHEIGHT     = 18;
   VISIBLE             = 10;
   TOTAL               = 100;
   INITIALVAL          = 25;
   MINWINDOWWIDTH      = 80;
   MINWINDOWHEIGHT     = (PROPGADGETHEIGHT + 70);
   MAXCHARS            = 3;

PROCEDURE CleanUp(Why : STRING; err: Word);
BEGIN
    IF assigned(prop) THEN DisposeObject(prop);
    IF assigned(int) THEN DisposeObject(int);
    IF assigned(w) THEN CloseWindow(w);
    IF Why <> '' THEN MessageBox('Boopsi Report',Why,'OK');
    Halt(err);
END;

BEGIN

    done := FALSE;

    prop2intmap[0].ti_Tag := PGA_Top;
    prop2intmap[0].ti_Data := STRINGA_LongVal;
    prop2intmap[1].ti_Tag := TAG_END;

    int2propmap[0].ti_Tag := STRINGA_LongVal;
    int2propmap[0].ti_Data := PGA_Top;
    int2propmap[1].ti_Tag := TAG_END;

    w := OpenWindowTags(NIL,[
    WA_Flags,     WFLG_DEPTHGADGET + WFLG_DRAGBAR +
                               WFLG_CLOSEGADGET + WFLG_SIZEGADGET + WFLG_ACTIVATE,
    WA_IDCMP,     IDCMP_CLOSEWINDOW,
    WA_Width,     MINWINDOWWIDTH + 10,
    WA_Height,    MINWINDOWHEIGHT + 10,
    WA_MinWidth,  MINWINDOWWIDTH,
    WA_MinHeight, MINWINDOWHEIGHT,
    TAG_END]);

    IF w=NIL THEN CleanUp('No window',20);

    prop := NewObject(NIL, 'propgclass',[
    GA_ID,       PROPGADGET_ID,
    GA_Top,      (w^.BorderTop) + 5,
    GA_Left,     (w^.BorderLeft) + 5,
    GA_Width,    PROPGADGETWIDTH,
    GA_Height,   PROPGADGETHEIGHT,
    ICA_MAP,     @prop2intmap,
    PGA_Total,   TOTAL,
    PGA_Top,     INITIALVAL,
    PGA_Visible, VISIBLE,
    PGA_NewLook, ltrue,
    TAG_END]);

    IF prop = NIL THEN CleanUp('No propgadget',20);

    int := NewObject(NIL, 'strgclass',[
    GA_ID,      INTGADGET_ID,
    GA_Top,     (w^.BorderTop) + 5,
    GA_Left,    (w^.BorderLeft) + PROPGADGETWIDTH + 10,
    GA_Width,   MINWINDOWWIDTH -
                                  (w^.BorderLeft + w^.BorderRight +
                                  PROPGADGETWIDTH + 15),
    GA_Height,  INTGADGETHEIGHT,

    ICA_MAP,    @int2propmap,
    ICA_TARGET, prop,
    GA_Previous, prop,

    STRINGA_LongVal,  INITIALVAL,
    STRINGA_MaxChars, MAXCHARS,
    TAG_END]);

    temp := SetGadgetAttrs(prop, w, NIL,[
    ICA_TARGET, int,
    TAG_END]);

    IF int = NIL THEN CleanUp('No INTEGER gadget',20);

    dummy := AddGList(w, prop, -1, -1, NIL);
    RefreshGList(prop, w, NIL, -1);

    WHILE (NOT done) DO BEGIN
        mymsg := pIntuiMessage(WaitPort(W^.UserPort));
        mymsg := pIntuiMessage(GetMsg(W^.UserPort));
        IF mymsg^.IClass = IDCMP_CLOSEWINDOW THEN done := True;
        ReplyMsg(pMessage(mymsg));
    END;

    dummy := RemoveGList(w, prop, -1);
    CleanUp('',0);
END.
