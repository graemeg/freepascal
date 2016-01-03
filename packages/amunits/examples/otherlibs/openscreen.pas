
PROGRAM OpenScreen;

{***********************************************************************
* This is an example that shows how to open a p96 Screen and a Window
* to get input events and how to paint on that screen.
* Program terminates when space bar or any mouse button is pressed!
*
* alex (Sun Dec 29 01:42:59 1996)
***********************************************************************}

{
    Translated to fpc pascal.
    14 Mars 2001.

    Updated for fpc 1.0.7
    08 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}


uses exec, amigados, graphics, intuition, picasso96api, utility,systemvartags;

Const

    Pens        :   Array [0..0] Of integer = (NOT(0));
    template    :   pchar  =   'Width=W/N,Height=H/N,Depth=D/N';
    ScreenTitle :   pchar  =   'Picasso96 API Test';
    vecarray    :   Array[0..2] of longint = (0,0,0);
    ltrue       :   longint = 1;

Var

    sc      :   pScreen;
    wdf,
    wdp     :   pWindow;
    rpf,
    rpp     :   pRastPort;
    terminate   :   Boolean;
    signals     :   longint;
    format      :   RGBFTYPE;
    x1, y1,
    x2, y2,
    x3, y3      :   word;
    imsg        :   pIntuiMessage;
    msg         :   pMessage;
    Dimensions  :   Array [0..3] Of word;
    Width,
    Height,
    Depth       :   longint;
    rda         :   pRDArgs;

procedure CleanUp(str : string);
begin
    if assigned(wdp) then CloseWindow(wdp);
    if assigned(wdf) then CloseWindow(wdf);
    if assigned(sc) then p96CloseScreen(sc);
    if str <> '' then writeln(str);
    halt;
end;

BEGIN
    Width:=640;
    Height:=480;
    Depth:=8;

    rda := ReadArgs(template,@vecarray,Nil);
    If rda<>Nil Then Begin
       If vecarray[0] <> 0 then Width := long(@vecarray[0]);
       If vecarray[1] <> 0 then Height := long(@vecarray[1]);
       If vecarray[2] <> 0 then Depth := long(@vecarray[2]);
       FreeArgs(rda);
    End;

    sc:=p96OpenScreenTags([P96SA_Width, Width,
                           P96SA_Height, Height,
                           P96SA_Depth, Depth,
                           P96SA_AutoScroll, lTRUE,
                           P96SA_Pens, @Pens,
                           P96SA_Title, ScreenTitle,
                           TAG_DONE]);


    If sc=Nil Then CleanUp('Unable to open screen.');

    Dimensions[0]:=0;
    Dimensions[1]:=sc^.BarHeight+1;
    Dimensions[2]:=sc^.Width;
    Dimensions[3]:=sc^.Height-sc^.BarHeight-1;

    wdp:=OpenWindowTags(NIL,[WA_CustomScreen, sc,
                             WA_Title,'Writepixel',
                             WA_Left, (sc^.Width DIV 2-200) DIV 2+sc^.Width DIV 2,
                             WA_Top, (sc^.Height-sc^.BarHeight-300) DIV 2,
                             WA_Zoom, @Dimensions,
                             WA_Width, 200,
                             WA_Height, 300,
                             WA_MinWidth, 100,
                             WA_MinHeight, 100,
                             WA_MaxWidth, -1,
                             WA_MaxHeight, -1,
                             WA_SimpleRefresh, lTRUE,
                             WA_RMBTrap, lTRUE,
                             WA_Activate, lTRUE,
                             WA_CloseGadget, lTRUE,
                             WA_DepthGadget, lTRUE,
                             WA_DragBar, lTRUE,
                             WA_SizeGadget, lTRUE,
                             WA_SizeBBottom, lTRUE,
                             WA_GimmeZeroZero, lTRUE,
                             WA_ScreenTitle,ScreenTitle,
                             WA_IDCMP, IDCMP_RAWKEY + IDCMP_CLOSEWINDOW,
                             TAG_DONE]);

    If wdp = Nil Then CleanUp('Unable to open window 1.');

    wdf:=OpenWindowTags(NIL,[WA_CustomScreen,sc,
                             WA_Title, 'FillRect',
                             WA_Left,(sc^.Width div 2-200) div 2,
                             WA_Top,(sc^.Height-sc^.BarHeight-300)div 2,
                             WA_Zoom, @Dimensions,
                             WA_Width, 200,
                             WA_Height, 300,
                             WA_MinWidth, 100,
                             WA_MinHeight, 100,
                             WA_MaxWidth, -1,
                             WA_MaxHeight, -1,
                             WA_SimpleRefresh, lTRUE,
                             WA_RMBTrap, lTRUE,
                             WA_Activate, lTRUE,
                             WA_CloseGadget, lTRUE,
                             WA_DepthGadget, lTRUE,
                             WA_DragBar, lTRUE,
                             WA_SizeGadget, lTRUE,
                             WA_SizeBBottom, lTRUE,
                             WA_GimmeZeroZero, lTRUE,
                             WA_ScreenTitle, ScreenTitle,
                             WA_IDCMP, IDCMP_RAWKEY or IDCMP_CLOSEWINDOW,
                             TAG_DONE]);

    If wdf = Nil Then CleanUp('Unable to open window 2.');

    rpf:=wdf^.RPort;
    rpp:=wdp^.RPort;
    terminate:=False;
    signals:= longint((1 shl wdf^.UserPort^.mp_SigBit) or (1 shl wdp^.UserPort^.mp_SigBit));
    format:= RGBFTYPE(p96GetBitMapAttr (sc^.RastPort.BitMap, P96BMA_RGBFORMAT));

    Randomize;

    Repeat
       x1:=Random (wdf^.Width);
       y1:=Random (wdf^.Height);
       x2:=Random (wdf^.Width);
       y2:=Random (wdf^.Height);
       If x2<x1 Then Begin
         x3:=x2;
         x2:=x1;
         x1:=x3;
       End;
       If y2<y1 Then Begin
         y3:=y2;
         y2:=y1;
         y1:=y3;
       End;

       x3:=Random (wdp^.Width);
       y3:=Random (wdp^.Height);

       If format=RGBFB_CLUT Then Begin
          SetAPen (rpf, Random (255));
          RectFill (rpf,x1,y1,x2,y2);
          SetAPen (rpp, Random (255));
          WritePixel (rpp,x3,y3);
       End Else Begin
          p96RectFill (rpf, x1, y1, x2, y2,(Random(255) shl 16)+(Random(255) shl 8)+(Random (255)));
          p96WritePixel (rpp, x3, y3, ((Random(255)) shl 16) or ((Random(255)) shl 8) or (Random(255)));
       End;

       Repeat
         imsg:=pIntuiMessage(GetMsg (wdf^.UserPort));
         If imsg<>Nil Then Begin
            If ((imsg^.IClass=IDCMP_CLOSEWINDOW) Or ((imsg^.IClass=IDCMP_RAWKEY) And ((imsg^.Code=$40) or (imsg^.Code=$45)))) Then
                terminate:=True;
            ReplyMsg (pMessage(imsg));
         End;
       Until imsg=Nil;
       Repeat
          imsg:=pIntuiMessage(GetMsg (wdp^.UserPort));
          If imsg<>Nil Then Begin
             If ((imsg^.IClass=IDCMP_CLOSEWINDOW) Or ((imsg^.IClass=IDCMP_RAWKEY) And ((imsg^.Code=$40) or (imsg^.Code=$45)))) Then
                terminate:=True;
             ReplyMsg (pMessage(imsg));
          End;
       Until imsg=Nil;

      Until terminate;

      Forbid;
      Repeat
         msg:=GetMsg (wdf^.UserPort);
         If msg<>Nil Then
            ReplyMsg (msg);
      Until msg=Nil;
      Repeat
         msg:=GetMsg (wdp^.UserPort);
         If msg<>Nil Then
            ReplyMsg (msg);
      Until msg=Nil;
      Permit;

    CleanUp('');
END.
