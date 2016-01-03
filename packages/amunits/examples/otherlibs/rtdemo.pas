PROGRAM RTDemo;


(*
**  This is a straight translation from demo.c
**  in the reqtools archive.
**
**  Check this demo for tips on how to use
**  reqtools in FPC Pascal.
**
**  nils.sjoholm@mailbox.swipnet.se  (Nils Sjoholm)
**
*)

{
    History:

    Changed to use the new stuff in unit reqtools.pas.
    Removed the array values, will use unit longarray
    instead. Cleaned up the code a bit.

    09 Dec 2002.
    nils.sjoholm@mailbox.swipnet.se

}

uses reqtools, strings, utility,longarray;



CONST
    DISKINSERTED=$00008000;


VAR
    filereq         : prtFileRequester;
    fontreq         : prtFontRequester;
    scrnreq         : prtScreenModeRequester;
    filelist        : prtFileList;
    buffer          : PChar;
    filename        : PChar;
    dummy           : PChar;
    dummy2          : PChar;
    longnum         : Longint;
    ret             : Longint;
    color           : Longint;
    undertag        : Array [0..1] of tTagItem;

FUNCTION GetScrollValue(value : INTEGER): STRING;
BEGIN
    IF value = 0 THEN GetScrollValue := 'Off'
    ELSE GetScrollValue := 'On';
END;

PROCEDURE CleanUp;
BEGIN
    if assigned(dummy) then StrDispose(dummy);
    if assigned(dummy2) then StrDispose(dummy2);
    if assigned(buffer) then StrDispose(buffer);
    if assigned(filename) then StrDispose(filename);
END;

BEGIN
    dummy:= StrAlloc(400);
    dummy2 := StrAlloc(200);

    undertag[0].ti_Tag  := RT_UnderScore;
    undertag[0].ti_Data := Longint(byte('_'));
    undertag[1].ti_Tag  := TAG_END;

    rtEZRequestA('ReqTools 2.0 Demo' + #10 +
           '~~~~~~~~~~~~~~~~~' + #10 +
           '''reqtools.library'' offers several' + #10 +
           'different types of requesters:','Let''s see them', NIL, NIL, NIL);

    rtEZRequestA('NUMBER 1:' + #10 + 'The larch :-)',
                     'Be serious!', NIL, NIL, NIL);

    rtEZRequestA('NUMBER 1:' + #10 + 'String requester' + #10 + 'function:rtGetString()',
                     'Show me', NIL, NIL, NIL);

    buffer:= StrAlloc(128);      { This should alloc'd to maxchars + 1 }

    StrPCopy(buffer, 'A bit of text');

    ret := rtGetStringA (buffer, 127, 'Enter anything:', NIL, NIL);

    IF (ret=0) THEN
        rtEZRequestA('You entered nothing','I''m sorry', NIL, NIL, NIL)
    ELSE
        rtEZRequestA('You entered this string:' + #10 + '%s','So I did', NIL,
        readinlongs([buffer]),NIL);

    ret := rtGetString(buffer, 127, 'Enter anything:', NIL,[
                RTGS_GadFmt, ' _Ok |New _2.0 feature!|_Cancel',
                RTGS_TextFmt,'These are two new features of ReqTools 2.0:' + #10
                + 'Text above the entry gadget and more than' + #10 + 'one response gadget.',
                TAG_MORE, @undertag]);



    IF ret=2 THEN
        rtEZRequestA('Yep, this is a new' + #10 + 'ReqTools 2.0 feature!',
                       'Oh boy!',NIL,NIL,NIL);

    ret := rtGetString(buffer, 127, 'Enter anything:',NIL,[
                        RTGS_GadFmt,' _Ok | _Abort |_Cancel',
                        RTGS_TextFmt,'New is also the ability to switch off the' + #10 +
                        'backfill pattern.  You can also center the' + #10 +
                        'text above the entry gadget.' + #10 +
                        'These new features are also available in' + #10 +
                        'the rtGetLong() requester.',
                        RTGS_BackFill, FALSE,
                        RTGS_Flags, GSREQF_CENTERTEXT + GSREQF_HIGHLIGHTTEXT,
                        TAG_MORE, @undertag]);

    IF ret = 2 THEN
        rtEZRequestA('What!! You pressed abort!?!' + #10 + 'You must be joking :-)',
                             'Ok, Continue',NIL,NIL,NIL);

    rtEZRequestA ('NUMBER 2:' + #10 + 'Number requester' + #10 + 'function:rtGetLong()',
                     'Show me', NIL, NIL, NIL);

    ret := rtGetLong(longnum, 'Enter a number:',NIL,[
                      RTGL_ShowDefault, FALSE,
                      RTGL_Min, 0,
                      RTGL_Max, 666,
                      TAG_DONE]);

    IF(ret=0) THEN
        rtEZRequestA('You entered nothing','I''m sorry', NIL, NIL, NIL)
    ELSE
        rtEZRequestA('The number You entered was:'  + #10 + '%ld' ,
                     'So it was', NIL, readinlongs([longnum]), NIL);

    rtEZRequestA ('NUMBER 3:' + #10 + 'Notification requester, the requester' + #10 +
                         'you''ve been using all the time!' + #10 +
                         'function: rtEZRequestA()','Show me more', NIL, NIL, NIL);

    rtEZRequestA ('Simplest usage: some body text and' + #10 + 'a single centered gadget.',
                         'Got it', NIL, NIL, NIL);

    ret := 0;
    WHILE ret = 0 DO BEGIN
        ret := rtEZRequestA ('You can also use two gadgets to' + #10 +
                             'ask the user something' + #10 +
                             'Do you understand?',
                             'Of course|Not really', NIL, NIL, NIL);
        IF ret = 0 THEN rtEZRequestA ('You are not one of the brightest are you?' +
                                       #10 + 'We''ll try again...',
                                      'Ok', NIL, NIL, NIL);
    END;

    rtEZRequestA ('Great, we''ll continue then.', 'Fine', NIL, NIL, NIL);

    ret:=rtEZRequestA ('You can also put up a requester with' + #10 +
                       'three choices.' + #10 +
                       'How do you like the demo so far ?',
                       'Great|So so|Rubbish', NIL, NIL, NIL);
    CASE ret OF
        0:  rtEZRequestA ('Too bad, I really hoped you' + #10 + 'would like it better.',
                               'So what', NIL, NIL, NIL);

        1:  rtEZRequestA ('I''m glad you like it so much.','Fine', NIL, NIL, NIL);

        2:  rtEZRequestA ('Maybe if you run the demo again' + #10 + 'you''ll REALLY like it.',
                               'Perhaps', NIL, NIL, NIL);
    END;

    ret := rtEZRequest('The number of responses is not limited to three' + #10 +
                            'as you can see.  The gadgets are labeled with' + #10 +
                            'the ''Return'' code from rtEZRequestA().' + #10 +
                            'Pressing ''Return'' will choose 4, note that' + #10 +
                            '4''s button text is printed in boldface.',
                            '1|2|3|4|5|0', NIL, NIL,[
                            RTEZ_DefaultResponse, 4,
                            TAG_DONE]);

    rtEZRequestA('You picked ''%ld''.', 'How true', NIL, readinlongs([ret]),NIL);

    {
      If i used just a string for this text is will be truncated
      after 255 chars. There are no strpcat in strings so we
      have to use two buffers and then use strcat.
      Could also try ansistring.
    }
    strpcopy(dummy,'New for Release 2.0 of ReqTools (V38) is' + #10 +
                   'the possibility to define characters in the' + #10 +
                   'buttons as keyboard shortcuts.' + #10 +
                   'As you can see these characters are underlined.' + #10 +
                   'Pressing shift while still holding down the key' + #10 +
                   'will cancel the shortcut.' + #10);
    {
      The second buffer.
    }
    strpcopy(dummy2,'Note that in other requesters a string gadget may' + #10 +
                   'be active.  To use the keyboard shortcuts there' + #10 +
                   'you have to keep the Right Amiga key pressed down.');
    {
      Now put them together
    }
    strcat(dummy,dummy2);

    rtEZRequestA(dummy,'_Great|_Fantastic|_Swell|Oh _Boy',NIL,NIL,@undertag);

    rtEZRequestA('You may also use C-style formatting codes in the body text.' + #10 +
                        'Like this:' + #10 +  + #10 +
                        'The number %%ld is written %%s. will give:' + #10 +  + #10 +
                        'The number %ld is written %s.' + #10 +  + #10 +
                        'if you also pass ''5'' and ''five'' to rtEZRequestA().',
                        '_Proceed',NIL,readinlongs([5,'five']),@undertag);

    ret := rtEZRequest('It is also possible to pass extra IDCMP flags' + #10 +
                        'that will satisfy rtEZRequest(). This requester' + #10 +
                        'has had DISKINSERTED passed to it.' + #10 +
                        '(Try inserting a disk).', '_Continue', NIL,NIL,[
                        RT_IDCMPFlags, DISKINSERTED,
                        TAG_MORE,@undertag]);

    IF ((ret = DISKINSERTED)) THEN
        rtEZRequestA('You inserted a disk.', 'I did', NIL, NIL, NIL)
    ELSE
        rtEZRequestA('You Used the ''Continue'' gadget' + #10 +
                          'to satisfy the requester.','I did', NIL, NIL, NIL);

    rtEZRequest('Finally, it is possible to specify the position' + #10 +
                        'of the requester.' + #10 +
                        'E.g. at the top left of the screen, like this.' + #10 +
                        'This works for all requesters, not just rtEZRequest()!',
                        '_Amazing', NIL,NIL,[
                        RT_ReqPos, REQPOS_TOPLEFTSCR,
                        TAG_MORE,@undertag]);

    rtEZRequest('Alternatively, you can center the' + #10 +
                        'requester on the screen.' + #10 +
                        'Check out ''reqtools.doc'' for all the possibilities.',
                        'I''ll do that', NIL,NIL,[
                        RT_ReqPos, REQPOS_CENTERSCR,
                        TAG_MORE,@undertag]);


    ret := rtEZRequestA('NUMBER 4:' + #10 + 'File requester' + #10 + 'function: rtFileRequest()',
                          '_Demonstrate', NIL, NIL, @undertag);

    filereq := rtAllocRequestA(RT_FILEREQ, NIL);

    IF (filereq<>NIL) THEN BEGIN
        filename := StrAlloc(80);
        strpcopy (filename, '');
        {
          We have to cast rtFileRequester to an Longint
          to keep the compiler happy.
        }
        ret := Longint(rtFileRequestA(filereq, filename, 'Pick a file', NIL));
        IF (ret)<>0 THEN begin
            rtEZRequestA('You picked the file:' + #10 + '%s' + #10 + 'in directory:'
                                + #10 + '%s', 'Right', NIL, readinlongs([
                                                          filename,filereq^.Dir]),NIL);
        END
        ELSE
            rtEZRequestA('You didn''t pick a file.', 'No', NIL, NIL, NIL);

        rtEZRequestA('The file requester has the ability' + #10 +
                     'to allow you to pick more than one' + #10 +
                     'file (use Shift to extend-select).' + #10 +
                     'Note the extra gadgets you get.',
                     '_Interesting', NIL,NIL, @undertag);

        filelist := rtFileRequest(filereq,filename,'Pick some files',[
                                   RTFI_Flags, FREQF_MULTISELECT,
                                   TAG_END]);

        IF filelist <> NIL THEN BEGIN
            rtEZRequestA('You selected some files, this is' + #10 +
                          'the first one:' + #10 +
                          '"%s"' + #10 +
                          'All the files are returned as a linked' + #10 +
                          'list (see demo.c and reqtools.h).',
                          'Aha', NIL, readinlongs([filelist^.Name]),NIL);
            (* Traverse all selected files *)
            (*
            tempflist = flist;
            while (tempflist) {
                DoSomething (tempflist->Name, tempflist->StrLen);
                tempflist = tempflist->Next;
                }
            *)
            (* Free filelist when no longer needed! *)
            rtFreeFileList(filelist);
        END;
        rtFreeRequest(filereq);
    END
    ELSE
        rtEZRequestA('Out of memory!', 'Oh boy!', NIL, NIL, NIL);

    rtEZRequestA('The file requester can be used' + #10 + 'as a directory requester as well.',
                    'Let''s _see that', NIL, NIL, @undertag);

    filereq := rtAllocRequestA(RT_FILEREQ, NIL);
    IF (filereq<>NIL) THEN BEGIN

         ret := Longint(rtFileRequest(filereq, filename, 'Pick a directory',[
                                       RTFI_Flags, FREQF_NOFILES,
                                       TAG_END]));

         IF(ret=1) THEN begin
             rtEZRequestA('You picked the directory:' + #10 +'%s',
                          'Right', NIL, readinlongs([filereq^.Dir]), NIL);
         end ELSE
             rtEZRequestA('You didn''t pick a directory.', 'No', NIL, NIL, NIL);

         rtFreeRequest(filereq);
    END
    ELSE
         ret := rtEZRequestA('Out of memory','No',NIL,NIL,NIL);

    rtEZRequestA('NUMBER 5:' + #10 +' Font requester' + #10 + 'function:rtFontRequest()',
                          'Show', NIL, NIL, NIL);

    fontreq := rtAllocRequestA(RT_FONTREQ, NIL);
    IF (fontreq<>NIL) THEN BEGIN
         fontreq^.Flags := FREQF_STYLE OR FREQF_COLORFONTS;
         ret := rtFontRequestA (fontreq, 'Pick a font', NIL);
         IF(ret<>0) THEN begin
             rtEZRequestA('You picked the font:' + #10 + '%s' + #10 + 'with size:' +
                          #10 + '%ld',
                         'Right', NIL, readinlongs([fontreq^.Attr.ta_Name,
                                                    fontreq^.Attr.ta_YSize]),NIL);
         end ELSE
             ret := rtEZRequestA('You didn''t pick a font','I know', NIL, NIL, NIL);
         rtFreeRequest(fontreq);
    END
    ELSE
         rtEZRequestA('Out of memory!', 'Oh boy!', NIL, NIL, NIL);

    rtEZRequestA('NUMBER 6:' + #10 + 'Palette requester' + #10 + 'function:rtPaletteRequest()',
                 '_Proceed', NIL,NIL, @undertag);

    color := rtPaletteRequestA('Change palette',NIL,NIL);
    IF (color = -1) THEN
        rtEZRequestA('You canceled.' + #10 + 'No nice colors to be picked ?',
                         'Nah', NIL, NIL, NIL)
    ELSE begin
        rtEZRequestA('You picked color number %ld.', 'Sure did',
                         NIL, readinlongs([color]), NIL);
    END;

    rtEZRequestA('NUMBER 7: (ReqTools 2.0)' + #10 +
                 'Volume requester' + #10 +
                 'function: rtFileRequest() with' + #10 +
                 'RTFI_VolumeRequest tag.',
                 '_Show me', NIL, NIL, @undertag);

    filereq := rtAllocRequestA(RT_FILEREQ,NIL);
    IF (filereq <> NIL) THEN BEGIN

        ret := Longint(rtFileRequest(filereq,NIL,'Pick a volume!',[
                                      RTFI_VolumeRequest,0,
                                      TAG_END]));
        IF (ret = 1) THEN begin
            rtEZRequestA('You picked the volume:' + #10 + '%s',
                        'Right',NIL,readinlongs([filereq^.Dir]),NIL);
        end
        ELSE
            rtEZRequestA('You didn''t pick a volume.','I did not',NIL,NIL,NIL);
        rtFreeRequest(filereq);
    END
    ELSE
        rtEZRequestA('Out of memory','Oh boy!',NIL,NIL,NIL);

    rtEZRequestA('NUMBER 8: (ReqTools 2.0)' + #10 +
                 'Screen mode requester' + #10 +
                 'function: rtScreenModeRequest()' + #10 +
                 'Only available on Kickstart 2.0!',
                 '_Proceed', NIL, NIL, @undertag);

    scrnreq := rtAllocRequestA (RT_SCREENMODEREQ, NIL);
    IF (scrnreq<>NIL) THEN BEGIN

        ret := rtScreenModeRequest( scrnreq, 'Pick a screen mode:',[
                                     RTSC_Flags, SCREQF_DEPTHGAD OR SCREQF_SIZEGADS OR
                                     SCREQF_AUTOSCROLLGAD OR SCREQF_OVERSCANGAD,
                                     TAG_END]);

        IF(ret=1) THEN BEGIN
            rtEZRequestA('You picked this mode:' + #10 +
                         'ModeID  : 0x%lx' + #10 +
                         'Size    : %ld x %ld' + #10 +
                         'Depth   : %ld' + #10 +
                         'Overscan: %ld' + #10 +
                         'AutoScroll %s',
                         'Right', NIL,
                         readinlongs([scrnreq^.DisplayID,
                                      scrnreq^.DisplayWidth,
                                      scrnreq^.DisplayHeight,
                                      scrnreq^.DisplayDepth,
                                      scrnreq^.OverscanType,
                                      GetScrollValue(scrnreq^.AutoScroll)]),NIL);
        END
        ELSE
            rtEZRequestA('You didn''t pick a screen mode.', 'Sorry', NIL, NIL, NIL);
        rtFreeRequest (scrnreq);
    END
    ELSE
    rtEZRequestA('Out of memory!', 'Oh boy!', NIL, NIL, NIL);


    rtEZRequestA('That''s it!' + #10 + 'Hope you enjoyed the demo', '_Sure did', NIL,
                    NIL,@undertag);
    CleanUp;
END.
