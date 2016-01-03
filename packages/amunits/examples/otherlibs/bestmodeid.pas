
PROGRAM BestModeID;

{***********************************************************************
* This is example shows how to use p96BestModeIDTagList()
*
* tabt (Mon Aug 28 14:07:40 1995)
***********************************************************************}

{
    Translated to fpc pascal.
    1 Mars 2001.

    Updated for fpc 1.0.7
    08 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}

uses exec, amigados, graphics, picasso96api, utility;

Const

    template    :   pchar  =    'Width=W/N,Height=H/N,Depth=D/N';
    vecarray    :   Array[0..2] of long = (0,0,0);

    fmtstrings  :   Array [1..(Ord(RGBFB_MaxFormats)-2)] OF pchar = (
                    'RGBFB_NONE',
                    'RGBFB_CLUT',
                    'RGBFB_R8G8B8',
                    'RGBFB_B8G8R8',
                    'RGBFB_R5G6B5PC',
                    'RGBFB_R5G5B5PC',
                    'RGBFB_A8R8G8B8',
                    'RGBFB_A8B8G8R8',
                    'RGBFB_R8G8B8A8',
                    'RGBFB_B8G8R8A8',
                    'RGBFB_R5G6B5',
                    'RGBFB_R5G5B5',
                    'RGBFB_B5G6R5PC',
                    'RGBFB_B5G5R5PC');
Var
    DisplayID,
    width,
    height,
    depth       :   longint;
    rda         :   pRDArgs;

Begin

   width:=640;
   height:=480;
   depth:=24;

   rda:=ReadArgs (template,Addr(vecarray),Nil);
   If rda<>Nil Then Begin
      If vecarray[0]<> 0 then width := long(@vecarray[0]);
      If vecarray[1]<> 0 then height := long(@vecarray[1]);
      If vecarray[2]<> 0 then depth := long(@vecarray[2]);
      FreeArgs(rda);
   End;



   DisplayID:=p96BestModeIDTags([P96BIDTAG_NominalWidth, width,
                                 P96BIDTAG_NominalHeight, height,
                                 P96BIDTAG_Depth, depth,
                                 P96BIDTAG_FormatsForbidden, (RGBFF_R5G5B5 or RGBFF_R5G5B5PC or RGBFF_B5G5R5PC),
                                 TAG_DONE]);;
   If DisplayID>0 Then Begin
      Writeln ('DisplayID: ', hexstr(DisplayID,8));
      If DisplayID<>INVALID_ID Then Begin
         Writeln ('Width: ', p96GetModeIDAttr(DisplayID, P96IDA_WIDTH));
         Writeln ('Height: ', p96GetModeIDAttr(DisplayID, P96IDA_HEIGHT));
         Writeln ('Depth: ', p96GetModeIDAttr(DisplayID, P96IDA_DEPTH));
         Writeln ('BytesPerPixel: ', p96GetModeIDAttr(DisplayID, P96IDA_BYTESPERPIXEL));
         Writeln ('BitsPerPixel: ', p96GetModeIDAttr(DisplayID, P96IDA_BITSPERPIXEL));
         Writeln ('RGBFormat: ', fmtstrings[p96GetModeIDAttr(DisplayID,P96IDA_RGBFORMAT)+1]);
         If p96GetModeIDAttr(DisplayID, P96IDA_ISP96)<>0 Then
            Writeln ('Is P96: yes')
         Else
           Writeln ('Is P96: no');
      End;
   End Else
       Writeln ('DisplayID is 0.');
End.
