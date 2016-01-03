Program RequestModeID;


{ ***********************************************************************
  * This is example shows how to use p96RequestModeIDTagList()
  *
  * tabt (Sat Dec 28 03:44:35 1996)
  *********************************************************************** }

{
    Translated to fpc pascal.
    3 Mars 2001.

    Updated for fpc 1.0.7
    08 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}

uses exec, amigados, graphics, intuition, picasso96api, utility;

Const
    template    :   pchar = 'Width=W/N,Height=H/N,Depth=D/N';

    vecarray    :   Array[0..2] of longint = (0,0,0);

Var
    width,
    height,
    depth,
    DisplayID   :   longint;
    dim         :   tDimensionInfo;
    rda         :   pRDArgs;


Begin
    width:=640;
    height:=480;
    depth:=15;

    rda:=ReadArgs (template,@vecarray,Nil);
    If rda<>Nil Then Begin
       If vecarray[0] <> 0 then width := long(@vecarray[0]);
       If vecarray[1] <> 0 then height := long(@vecarray[1]);
       If vecarray[2] <> 0 then depth := long(@vecarray[2]);
       FreeArgs(rda);
    End;

    DisplayID := p96RequestModeIDTags([P96MA_MinWidth, width,
                                       P96MA_MinHeight, height,
                                       P96MA_MinDepth, depth,
                                       P96MA_WindowTitle, 'RequestModeID Test',
                                       P96MA_FormatsAllowed, (RGBFF_CLUT or RGBFF_R5G6B5 or RGBFF_R8G8B8 or RGBFF_A8R8G8B8),
                                       TAG_DONE]);

    Writeln ('DisplayID:', hexstr(DisplayID,8));

    If DisplayID <> INVALID_ID Then Begin
        If GetDisplayInfoData(Nil, @dim ,SizeOf(tDimensionInfo),DTAG_DIMS,DisplayID) <> 0 Then
            Writeln('Dimensions: ',dim.Nominal.MaxX-dim.Nominal.MinX+1,'x',dim.Nominal.MaxY-dim.Nominal.MinY+1,'x',dim.MaxDepth)
        Else
            Writeln('No Dimensioninfo.');
    End Else Writeln('DisplayID invalid.');
End.
