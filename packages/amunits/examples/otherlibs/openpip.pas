Program OpenPIP;


{ ***********************************************************************
  * This is an example that shows how to open a p96 PIP Window
  * to get input events and how to paint in that window.
  *
  *********************************************************************** }

{
    Translated to fpc pascal.
    3 Mars 2001.

    Updated for fpc 1.0.7
    08 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}

uses exec, amigados, graphics, intuition, picasso96api, utility,strings;


Const
    WB          :   Pchar = 'Workbench';
    template    :   Pchar = 'Width=W/N,Height=H/N,Pubscreen=PS/K';
    vecarray    :   Array[0..2] of long = (0,0,0);
    ltrue       :   longint = 1;
Var
    PubScreenName   :   Array [0..80] Of Char;
    height,
    width           :   longint;
    wd              :   pWindow;
    imsg            :   pIntuiMessage;
    goahead         :   Boolean;
    rp              :   pRastPort;
    x,
    y               :   Word;
    rda             :   pRDArgs;

Begin
    width := 256;
    height := 256;
    StrCopy(@PubScreenName,WB);

    rda:=ReadArgs(template,@vecarray,Nil);
    If rda<>Nil Then Begin
       If vecarray[0] <> 0 then width := long(@vecarray[0]);
       If vecarray[1] <> 0 then height := long(@vecarray[1]);
       If vecarray[2] <> 0 then StrCopy(@PubScreenName,@vecarray[2]);
       FreeArgs(rda);
    End;


    wd := p96PIP_OpenTags([P96PIP_SourceFormat, long(RGBFB_R5G5B5),
                           P96PIP_SourceWidth,256,
                           P96PIP_SourceHeight,256,
                           WA_Title,'Picasso96 API PIP Test',
                           WA_Activate,lTRUE,
                           WA_RMBTrap,lTRUE,
                           WA_Width,Width,
                           WA_Height,Height,
                           WA_DragBar, lTRUE,
                           WA_DepthGadget,lTRUE,
                           WA_SimpleRefresh,lTRUE,
                           WA_SizeGadget,lTRUE,
                           WA_CloseGadget,lTRUE,
                           WA_IDCMP,IDCMP_CLOSEWINDOW,
                           WA_PubScreenName,@PubScreenName,
                           TAG_DONE]);

    If wd <> Nil Then Begin
        goahead:=True;
        rp:=Nil;

        p96PIP_GetTags(wd,[P96PIP_SourceRPort, @rp, TAG_END]);
        If rp<>Nil Then Begin
            For y:=0 To (Height-1) Do
            For x:=0 To (Width-1) Do
               p96WritePixel (rp,x,y,(x*256+y)*256);
        End Else Writeln ('No PIP rastport.');
        While goahead Do Begin
            WaitPort (wd^.UserPort);
            imsg := pIntuiMessage(GetMsg (wd^.UserPort));
            While imsg<>Nil Do Begin
                If imsg^.IClass=IDCMP_CLOSEWINDOW Then goahead:=False;
                ReplyMsg (pMessage(imsg));
                imsg:=pIntuiMessage(GetMsg (wd^.UserPort));
            End;
        End;
        p96PIP_Close(wd);
    End Else Writeln ('Unable to open PIP.');
End.
