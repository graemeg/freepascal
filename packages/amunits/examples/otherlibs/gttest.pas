program gttest;

{

    This is just a small test of gtlayout.library.
    It's from gtlayout.doc.

    No problems so far.
    16 Jul 2000.

    Added MessageBox for report.
    31 Jul 2000.

    Updated to use fpc 1.0.7
    07 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se

}

uses intuition, exec, gadtools, utility, gtlayout, msgbox;

const
    ltrue : longint = 1;
    lfalse : longint = 0;

var
    handle : pLayoutHandle;
    win : pWindow;
    msg : pIntuiMessage;
    msgQuali : ulong;
    msgclass : ulong;
    msgcode : word;
    msggadget : pGadget;
    done : boolean;

procedure CleanUp(why : string; rc : integer);
begin
    LT_DeleteHandle(handle);
    if why <> '' then MessageBox('GTLayout Report',why,'OK');
    halt(rc);
end;

begin
    done := false;
    handle := LT_CreateHandleTags(nil,[
                    LAHN_AutoActivate, lfalse,
                    TAG_DONE]);

    if handle = nil then CleanUp('Could''t create a handle',20);

    LT_New(handle,[LA_Type,VERTICAL_KIND,       { A vertical group. }
                   LA_LabelText,'Main Group',
                   TAG_DONE]);

    LT_New(handle,[LA_Type,BUTTON_KIND,         { A plain button. }
                   LA_LabelText,'A button',
                   LA_ID,11,
                   TAG_DONE]);

    LT_New(handle,[LA_Type,XBAR_KIND,TAG_DONE]); { A separator bar. }

    LT_New(handle,[LA_Type,BUTTON_KIND,          { A plain button. }
                   LA_LabelText,'Another button',
                   LA_ID,22,
                   TAG_DONE]);

    LT_New(handle,[LA_Type,CHECKBOX_KIND,LA_LabelText,'test',LA_ID,33,LA_BOOL,1,TAG_DONE]);

    LT_New(handle,[La_Type,END_KIND,TAG_DONE]);  { This ends the current group. }

    win := LT_Build(handle,[LAWN_Title,'Window title',
                            LAWN_IDCMP, IDCMP_CLOSEWINDOW,
                            WA_CloseGadget, ltrue,
                            TAG_DONE]);

    if win = nil then CleanUp('Can''t open the window',20);

    repeat
        msg := pIntuiMessage(WaitPort(win^.UserPort));
        msg := GT_GetIMsg(win^.UserPort);
        while msg <> nil do begin
            msgclass := msg^.IClass;
            msgcode := msg^.Code;
            msgQuali := msg^.Qualifier;
            msggadget := msg^.IAddress;
            GT_ReplyIMsg(msg);
            LT_HandleInput(handle,msgQuali,@msgclass,@msgcode,@msggadget);
            case msgclass of
                 IDCMP_CLOSEWINDOW : begin
                                     writeln(LT_GetAttributesA(handle,33,nil));
                                     done := true;
                                     end;
                 IDCMP_GADGETUP: begin
                                 case msggadget^.GadgetId of
                                      11 : writeln('First gadget');
                                      22 : writeln('Second gadget');
                                 end;
                                 end;
            end;
            msg := GT_GetIMsg(win^.UserPort);
         end;
     until done;
     CleanUp('all ok',0);
end.
