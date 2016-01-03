PROGRAM EasyGadtools;

{
    This is just a test on how to make a unit EasyGadtools.

    Feel free to make any changes or improvements on this
    example. If you make a unit or have a unit to handle
    gadtools in an easy way let me know.
    24 Jul 2000.

    Changed to use systemvartags.
    25 Nov 2002.

    nils.sjoholm@mailbox.swipnet.se

}

USES Intuition, Exec, Graphics, GadTools, Utility, pastoc,systemvartags;

CONST

     strarray : array[0..4] of PChar = ('A cycle',
                                        'test',
                                        'for',
                                        'FPC Pascal',
                                        nil);


VAR
  ps                : pScreen;
  vi                : Pointer;
  ng                : tNewGadget;
  glist,gad         : pGadget;
  wp                : pWindow;
  HFont             : word;
  HGadget           : word;
  DistGad           : word;
  HG                : word;
  attr              : pTextAttr;

function NewGadget(left,top,width,height : Integer; txt : PChar; txtattr: pTextAttr;
                   id : word; flags: Longint; visinfo, userdata : Pointer): tNewGadget;
var
    ng : tNewGadget;
begin
    with ng do begin
        ng_LeftEdge   := left;
        ng_TopEdge    := top;
        ng_Width      := width;
        ng_Height     := height;
        ng_GadgetText := txt;
        ng_TextAttr   := txtattr;
        ng_GadgetID   := id;
        ng_Flags      := flags;
        ng_VisualInfo := visinfo;
        ng_UserData   := userdata;
    END;
    NewGadget := ng;
end;

PROCEDURE CleanUp(why : string; rc : BYTE);
BEGIN
  IF assigned(wp) THEN CloseWindow(wp);
  IF assigned(glist) THEN FreeGadgets(glist);
  IF assigned(vi) THEN FreeVisualInfo(vi);
  if why <> '' then writeln(why);
  HALT(rc);
END;

{ Clones some datas from default pubscreen for fontsensitive
  placing of gadgets. }
PROCEDURE CloneDatas;
BEGIN
  ps := LockPubScreen(NIL);
  IF ps = NIL THEN CleanUp('Can''t get a lock on public screen',20)
  ELSE
  BEGIN
     HFont := ps^.Font^.ta_YSize;
     attr := ps^.Font;
     vi := GetVisualInfoA(ps,NIL);
     UnLockPubScreen(NIL, ps);
     IF vi = NIL THEN CleanUp('Can''t get VisualInfo', 20);
  END;
END;

function ButtonGadget(id,left,top,width,height:word; txt:pchar): pGadget;
begin
   ng := NewGadget(left,top,width,height,txt,attr,id,PLACETEXT_IN,vi,nil);
   gad := CreateGadgetA(BUTTON_KIND,gad,@ng,nil);
   ButtonGadget := gad;
end;

function ButtonGadget(id,left,top,width,height:word; txt: string): pGadget;
begin
   ButtonGadget := ButtonGadget(id,left,top,width,height,pas2c(txt));
end;

function CycleGadget(id,left,top,width,height:word; txt:pchar ; thearr : Pointer): pGadget;
begin
   ng := NewGadget(left,top,width,height,txt,attr,id,PLACETEXT_LEFT,vi,nil);
   gad := CreateGadget(CYCLE_KIND,gad,@ng,[
                                         GTCY_Labels,thearr,
                                         TAG_END]);
   CycleGadget := gad;
end;

PROCEDURE GenerateWindow;
BEGIN
  glist := NIL; gad := CreateContext(addr(glist));
  IF gad = NIL THEN CleanUp('Can''t create GadList', 20);

  gad := ButtonGadget(0,10,HG,200,HGadget,'File Requester');
  HG := HG + DistGad;

  gad := ButtonGadget(1,10,HG,200,HGadget,'Font Requester');
  HG := HG + DistGad;

  gad := ButtonGadget(2,10,HG,200,HGadget,'Screen Requester');
  HG := HG + DistGad + 3;

  gad := CycleGadget(3,100,HG,100,HGadget,'Cycle me',@strarray);
  HG := HG + DistGad+4;

  gad := ButtonGadget(4,10,HG,96,HGadget,'OK');
  gad := ButtonGadget(5,115,HG,96,HGadget,'Cancel');

  HG := HG + 5;

  if gad = nil then CleanUp('Can''t create gadgets',20);

  wp := OpenWindowTags(NIL,[
                WA_Gadgets, glist,
                WA_Title, 'Test of EasyGadtools',
                WA_Left,100,
                WA_Top,100,
                WA_Flags, WFLG_SMART_REFRESH OR WFLG_NOCAREREFRESH OR
                                WFLG_DEPTHGADGET OR WFLG_DRAGBAR OR WFLG_CLOSEGADGET OR
                                WFLG_ACTIVATE,
                WA_Idcmp, IDCMP_GADGETUP OR IDCMP_CLOSEWINDOW,
                WA_InnerWidth, 215,
                WA_InnerHeight, HG,
                TAG_DONE]);

  IF wp = NIL THEN CleanUp('Can''t open window', 20);
END;

PROCEDURE MainWait;
VAR
  msg : pIntuiMessage;
  iclass : LONG;
  ende : BOOLEAN;
BEGIN
  ende := FALSE;
  REPEAT
    msg := pIntuiMessage(WaitPort(wp^.UserPort));
     msg := GT_GetIMsg(wp^.UserPort);
     WHILE msg <> NIL DO
     BEGIN
        iclass := msg^.IClass;
        GT_ReplyIMsg(msg);
        CASE iclass OF
          IDCMP_CLOSEWINDOW : ende := TRUE;
          IDCMP_GADGETUP : writeln('You clicked on a gadget');
        ELSE END;
       msg := GT_GetIMsg(wp^.UserPort);
     END;
  UNTIL ende;
END;

BEGIN
  CloneDatas;
  HGadget := HFont +6;
  DistGad := HGadget +4;
  HG := HFont + 10;
  GenerateWindow;
  MainWait;
  CleanUp('',0);
END.
