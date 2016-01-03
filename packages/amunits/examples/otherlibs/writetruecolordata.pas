Program WriteTrueColorData;

{ ***********************************************************************
  * This is an example that shows how to use p96WriteTrueColorData
  * Program terminates when space bar or any mouse button is pressed!
  *
  * alx (Mon Dec 30 12:09:35 1996)
  *********************************************************************** }

{
    Translated to fpc pascal.
    14 Mars 2001.

    Updated for fpc 1.0.7
    08 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}

uses exec, amigados, intuition, graphics, picasso96api, utility,systemvartags;


Const
    DataWidth   =   160;
    DataHeight  =   160;
    template    :   PChar = 'Width=W/N,Height=H/N,Depth=D/N';
    vecarray    :   Array[0..2] of long = (0,0,0);
    ltrue       :   longint = 1;
Var
    rda         :   pRDArgs;

{ p96WriteTrueColorData only works on True- and HiColorModes }

Const
    HiColorFormats      =   (RGBFF_R5G6B5 or RGBFF_R5G5B5 or RGBFF_R5G6B5PC or RGBFF_R5G5B5PC or RGBFF_B5G6R5PC or RGBFF_B5G5R5PC);
    TrueColorFormats    =   (RGBFF_R8G8B8 or RGBFF_B8G8R8);
    TrueAlphaFormats    =   (RGBFF_R8G8B8A8 or RGBFF_B8G8R8A8 or RGBFF_A8R8G8B8 or RGBFF_A8B8G8R8);
    UsefulFormats       =   (HiColorFormats or TrueColorFormats or TrueAlphaFormats);

    Pens    :   Array [0..0] Of integer = (NOT(0));

Var
    sc          :   pScreen;
    win         :   pWindow;
    i,
    DisplayID   :   Longint;
    width,
    height,
    depth       :   Longint;

    quit        :   Boolean;
    reddata,
    greendata,
    bluedata    :   Pointer;
    tci         :   tTrueColorInfo;
    fh          :   FileHandle;
    imsg        :   pIntuiMessage;


procedure CleanUp(why : string);
begin
    if assigned(win) then CloseWindow(win);
    if assigned(sc) then p96CloseScreen(sc);
    if why <> '' then writeln(why);
end;

Begin

 width:=640;
 height:=480;
 depth:=24;

 rda:=ReadArgs (template,@vecarray,Nil);
 If rda<>Nil Then
 Begin
  If vecarray[0]<>0 then width := long(@vecarray[0]);
  If vecarray[1]<>0 then height := long(@vecarray[1]);
  If vecarray[2]<>0 then depth := long(@vecarray[2]);
  FreeArgs(rda);
 End;

 DisplayID := p96BestModeIDTags([P96BIDTAG_NominalWidth, width,
                                 P96BIDTAG_NominalHeight, height,
                                 P96BIDTAG_Depth, depth,
                                 P96BIDTAG_FormatsAllowed, UsefulFormats,
                                 TAG_DONE]);


 sc := p96OpenScreenTags([P96SA_DisplayID, DisplayID,
                          P96SA_Width, width,
                          P96SA_Height, height,
                          P96SA_Depth, depth,
                          P96SA_AutoScroll, lTRUE,
                          P96SA_Pens, @Pens,
                          P96SA_Title, 'WriteTrueColorData Test',
                          TAG_DONE]);

if sc = nil then CleanUp('Can''t open screen');




 win := OpenWindowTags(Nil,[WA_CustomScreen, sc,
                            WA_Backdrop, lTRUE,
                            WA_Borderless, lTRUE,
                            WA_SimpleRefresh, lTRUE,
                            WA_RMBTrap, lTRUE,
                            WA_Activate, lTRUE,
                            WA_IDCMP, IDCMP_RAWKEY or IDCMP_MOUSEBUTTONS,
                            TAG_END]);

 if win = nil then CleanUp('Can''t open window');

 quit:=False;
 reddata:=AllocVec(DataWidth*DataHeight, MEMF_ANY);
 greendata:=AllocVec(DataWidth*DataHeight, MEMF_ANY);
 bluedata:=AllocVec(DataWidth*DataHeight, MEMF_ANY);
 If (reddata<>Nil) And (greendata<>Nil) And (bluedata<>Nil) Then Begin
      tci.PixelDistance:=1;
      tci.BytesPerRow:=DataWidth;
      tci.RedData:=reddata;
      tci.GreenData:=greendata;
      tci.BlueData:=bluedata;

      fh:=DOSOpen ('Symbol.red',MODE_OLDFILE);
      If fh = 0 Then Begin
       i:=DOSRead(fh, reddata, DataWidth*DataHeight);
       DOSClose(fh);
      End;

      fh:=DOSOpen ('Symbol.green',MODE_OLDFILE);
      If fh = 0 Then Begin
       i:=DOSRead(fh, greendata, DataWidth*DataHeight);
       DOSClose(fh);
      End;

      fh:=DOSOpen ('Symbol.blue',MODE_OLDFILE);
      If fh = 0 Then Begin
       i:=DOSRead(fh, bluedata, DataWidth*DataHeight);
       DOSClose(fh);
      End;

      { paint something on the screen }

      p96WriteTrueColorData(@tci,0,0,win^.RPort,50,50,DataWidth,DataHeight);

     End;

     FreeVec(reddata);
     FreeVec(greendata);
     FreeVec(bluedata);

     { wait for input }

     While Not(quit) Do Begin
         WaitPort(win^.UserPort);
         imsg:=pIntuiMessage(GetMsg (win^.UserPort));
         While(imsg<>Nil) Do Begin
            If ((imsg^.IClass=IDCMP_MOUSEBUTTONS) or ((imsg^.IClass=IDCMP_RAWKEY) And (imsg^.Code=$40))) Then Begin
        { press MOUSEBUTTONS or SPACE bar to end program }
            quit:=True;
            End;
            ReplyMsg(pMessage(imsg));
            imsg:=pIntuiMessage(GetMsg (win^.UserPort));
         End;
     End;

  CleanUp('');
End.
