PROGRAM ImageGadget;

{
   An example on how to use GadTools gadgets,
   on the same time how to use images.
   20 Sep 1998.

   Changed the code to use TAGS, now also use
   pas2c for strings-pchar.
   1 Nov 1998.

   Removed opening of gadtools.library.
   Will be opened by unit gadtools.
   16 Jul 2000.

   Update to use systemvartags. Not a
   very nice demo, needs to rewrite to
   handle more bitplanes.
   28 Nov 2002.

   nils.sjoholm@mailbox.swipnet.se
}

USES Intuition, Exec, Graphics, GadTools, Utility, systemvartags,pastoc;


CONST
  MSG_NO_PS            : PChar = 'Can''t lock Public Screen';
  MSG_NO_VI            : PChar = 'Can''t get Visual Info';
  MSG_NO_MEM           : PChar = 'Not enough memory free';
  MSG_NO_WP            : PChar = 'Can''t open window';

  WIN_TITLE            : PChar = 'Images-Example';
  OK_TEXT              : PChar = 'OK';

  type
      data = array[1..176] of word;
      pdata = ^data;

  const
    renderd : data = (
    {* Plane 0 *}
        $0000,$0000,
        $0000,$0010,
        $0000,$0010,
        $0000,$0010,
        $01C0,$0010,
        $03E0,$0010,
        $07F0,$0010,
        $0000,$0010,
        $0000,$0810,
        $039A,$C810,
        $0000,$0810,
        $031E,$0810,
        $0000,$4810,
        $03E6,$0810,
        $0000,$0810,
        $0000,$0810,
        $07FF,$F810,
        $0000,$0010,
        $0000,$0010,
        $0000,$0010,
        $0000,$0010,
        $7FFF,$FFF0,
    {* Plane 1 *}
        $FFFF,$FFE0,
        $8000,$0000,
        $8000,$0000,
        $8000,$0000,
        $81C0,$0000,
        $83E0,$0000,
        $87F0,$0000,
        $8000,$0000,
        $87FF,$E000,
        $8465,$2000,
        $87FF,$E000,
        $84E1,$E000,
        $87FF,$A000,
        $8419,$E000,
        $87FF,$E000,
        $8400,$0000,
        $8000,$0000,
        $8000,$0000,
        $8000,$0000,
        $8000,$0000,
        $8000,$0000,
        $0000,$0000,
    {* Plane 2 *}
        $0000,$0000,
        $0000,$0020,
        $0000,$0020,
        $0000,$0020,
        $0000,$0020,
        $01C0,$0020,
        $03E0,$0020,
        $0FFF,$F820,
        $0800,$1020,
        $0800,$1020,
        $0800,$1020,
        $0800,$1020,
        $0800,$1020,
        $0800,$1020,
        $0800,$1020,
        $0BFF,$F020,
        $0800,$0020,
        $0000,$0020,
        $0000,$0020,
        $0000,$0020,
        $7FFF,$FFE0,
        $0000,$0000,

        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000
    );

     selectd : data = (
        { Plane 0 }
                $FFFF,$FFE0,
                $8000,$0000,
                $8000,$0000,
                $8000,$0000,
                $8000,$0000,
                $80E0,$0000,
                $81F0,$0000,
                $83F8,$0000,
                $8000,$0000,
                $8000,$0400,
                $81CD,$6400,
                $8000,$0400,
                $818F,$0400,
                $8000,$2400,
                $81F3,$0400,
                $8000,$0400,
                $8000,$0400,
                $83FF,$FC00,
                $8000,$0000,
                $8000,$0000,
                $8000,$0000,
                $0000,$0000,
        { Plane 1 }
                $0000,$0000,
                $0000,$0010,
                $0000,$0010,
                $0000,$0010,
                $0000,$0010,
                $00E0,$0010,
                $01F0,$0010,
                $03F8,$0010,
                $0000,$0010,
                $03FF,$F010,
                $0232,$9010,
                $03FF,$F010,
                $0270,$F010,
                $03FF,$D010,
                $020C,$F010,
                $03FF,$F010,
                $0200,$0010,
                $0000,$0010,
                $0000,$0010,
                $0000,$0010,
                $0000,$0010,
                $7FFF,$FFF0,
        { Plane 2 }
                $0000,$0000,
                $0000,$0020,
                $0000,$0020,
                $0000,$0020,
                $0000,$0020,
                $0000,$0020,
                $00E0,$0020,
                $01F0,$0020,
                $07FF,$FC20,
                $0400,$0820,
                $0400,$0820,
                $0400,$0820,
                $0400,$0820,
                $0400,$0820,
                $0400,$0820,
                $0400,$0820,
                $05FF,$F820,
                $0400,$0020,
                $0000,$0020,
                $0000,$0020,
                $7FFF,$FFE0,
                $0000,$0000,

        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000,
        $0000,$0000
                     );


VAR
  ps                : pScreen;
  vi                : Pointer;
  ng                : tNewGadget;
  xoff, yoff,i      : Longint;
  gl,g              : pGadget;
  firstimage        : pdata;
  secondimage       : pdata;
  renderi,
  selecti           : tImage;
  wp                : pWindow;


function NewGadget(left,top,width,height : Integer; txt : PChar; txtattr: pTextAttr;
                   id : word; flags: Longint; visinfo, userdata : Pointer):
tNewGadget;
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

function Image(left,top,width,height,depth: Integer; imdata : pointer;
               ppick, ponoff: byte; nextim : pImage): tImage;
var
    im : tImage;
begin

        im.LeftEdge    := left;
        im.TopEdge     := top;
        im.Width       := width;
        im.Height      := height;
        im.Depth       := depth;
        im.ImageData   := imdata;

        im.PlanePick   := ppick;
        im.PlaneOnOff  := ponoff;

        im.NextImage   := nextim;

    Image := im;
end;



FUNCTION EasyReq(wp : pWindow; title,body,gad : PChar) : Longint;
VAR
  es : tEasyStruct;
BEGIN
  es.es_StructSize:=SizeOf(tEasyStruct);
  es.es_Flags:=0;
  es.es_Title:=title;
  es.es_TextFormat:=body;
  es.es_GadgetFormat:=gad;

  EasyReq := EasyRequestArgs(wp,@es,0,NIL);
END;

PROCEDURE CleanUp(why : PChar; rc : BYTE);
BEGIN
  IF assigned(wp) THEN CloseWindow(wp);
  IF assigned(gl) THEN FreeGadgets(gl);
  IF assigned(vi) THEN FreeVisualInfo(vi);
  IF assigned(firstimage) THEN FreeVec(firstimage);
  IF assigned(secondimage) THEN FreeVec(secondimage);
   IF why <> nil THEN i := EasyReq(NIL,WIN_TITLE,why,OK_TEXT);
  HALT(rc);
END;

{ Clones some datas from default pubscreen for fontsensitive
  placing of gadgets. }
PROCEDURE CloneDatas;
BEGIN
  ps := LockPubScreen(NIL);
  IF ps = NIL THEN CleanUp(MSG_NO_PS,20)
  ELSE
  BEGIN
     xoff := ps^.WBorLeft;
     yoff := ps^.WBorTop + ps^.Font^.ta_YSize + 1;
     vi := GetVisualInfoA(ps,NIL);
     UnLockPubScreen(NIL, ps);
     IF vi = NIL THEN CleanUp(MSG_NO_VI, 20);
  END;
END;

procedure AllocateImages;
begin
  firstimage := Pointer(AllocVec(SizeOf(data),MEMF_CLEAR OR MEMF_CHIP));
  if firstimage = nil then CleanUp(MSG_NO_MEM,20);

  firstimage^ := renderd;

  renderi := Image(0,0,28,22,3,firstimage,$ff,$0,nil);

  secondimage := Pointer(AllocVec(SizeOf(data),MEMF_CLEAR OR MEMF_CHIP));
  if secondimage = nil then CleanUp(MSG_NO_MEM,20);

  secondimage^ := selectd;

  selecti := Image(0,0,28,22,3,secondimage,$ff,$0,nil);

END;

PROCEDURE GenerateWindow;
BEGIN
  gl := NIL; gl := CreateContext(addr(gl));
  IF gl = NIL THEN CleanUp(MSG_NO_MEM, 20);
  ng := NewGadget(xoff+1,yoff+1,28,22,nil,nil,1,0,vi,nil);

  g := CreateGadgetA(GENERIC_KIND,gl,@ng,NIL);
  IF g = NIL THEN CleanUp(MSG_NO_MEM, 20);

  g^.GadgetType := GTYP_BOOLGADGET;
  g^.Flags := GFLG_GADGIMAGE OR GFLG_GADGHIMAGE; { 2 Images }
  g^.Activation := GACT_RELVERIFY; { Verhalten wie ein BUTTON_KIND-Gadget }
  g^.GadgetRender := @renderi;
  g^.SelectRender := @selecti;

  wp := OpenWindowTags(NIL,[
                WA_Gadgets,gl,
                WA_Title, 'Images in Gadgets',
                WA_Flags, WFLG_SMART_REFRESH OR WFLG_NOCAREREFRESH OR
                                WFLG_DEPTHGADGET OR WFLG_DRAGBAR OR WFLG_CLOSEGADGET OR
                                WFLG_ACTIVATE,
                WA_Idcmp, IDCMP_GADGETUP OR IDCMP_CLOSEWINDOW,
                WA_InnerWidth, 100,
                WA_InnerHeight, 50,
                TAG_DONE]);

  IF wp = NIL THEN CleanUp(MSG_NO_WP, 20);
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
          IDCMP_GADGETUP :
             i := EasyReq(wp,WIN_TITLE,pas2c('You have clicked on the Gadget!'),pas2c('Wheeew!'));
        ELSE END;
       msg := GT_GetIMsg(wp^.UserPort);
     END;
  UNTIL ende;
END;

BEGIN
  new(gl);
  CloneDatas;
  AllocateImages;
  GenerateWindow;
  MainWait;
  CleanUp(nil,0);
END.
