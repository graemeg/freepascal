Program Edit_Demo;
{---------------------------------------------------------------------------
                                 CncWare
                         (c) Copyright 1999-2000
 ---------------------------------------------------------------------------
  Filename..: edit_demo.pp
  Programmer: Ken J. Wright, ken@cncware.com
  Date......: 12/12/99

  Purpose - Demonstrate the use of the oCrt unit.

-------------------------------<< REVISIONS >>--------------------------------
  Ver  |   Date   | Prog| Description
-------+----------+-----+-----------------------------------------------------
  1.00 | 12/12/99 | kjw | Initial Release.
  1.01 | 12/13/99 | kjw | Changed to use oCrt.
  1.02 | 06/16/00 | kjw | Added help & goto line pop-up screens.
                        | Changes for control keys.
  1.03 | 07/25/00 | kjw | Added use of new tnMenu object.
------------------------------------------------------------------------------
}
uses oCrt;

const
   MAXLINES = 52;                 { allow for long screens }
   CURLINES : Integer = MAXLINES; { adjusted later }
   FRAMED = true;
   NOFRAME = false;
   bg = 16;                       { background color multiplier }

type
   { doubly linked list of strings to edit }
   pLine = ^tLine;
   tLine = Record
      s : ^string;
      next,
      prev : pLine;
   End;
   s80 = string[80];

var
   hdr,                            { list head }
   line,                           { current position in list }
   line1 : pLine;                  { first list item of current page }
   ss : array[1..MAXLINES] of s80; { a sliding screen buffer }
   xp,yp : string;                 { x & y positions for the status line }
   EdWin,                          { main edit window }
   StatWin : tnWindow;             { status line }
   mnu0 : tnMenu;                  { main menu }
   mnu1 : pnMenu;                  { dynamic menu for sub menus }
   xi,                             { integer scratch pad }
   cv,                             { edit character return value }
   idx : integer;                  { current screen buffer row index }
   cline,                          { current line number }
   dlines : integer;               { number of displayed lines }
   lines : longint;                { total number of lines in the list }
   mactive,                        { is the menu active? }
   Finished : boolean;             { exit when finished }
   tf : text;                      { the text file we are reading/writing }
   fnam : string;                  { name of the current file, tf }


{ replace the old string with a new one }
Procedure ReallocateLine(var p : pLine; s : string);
Begin
   If p = Nil Then Exit;
   If p^.s^ <> s Then Begin
      FreeMem(p^.s,Length(p^.s^)+1);
      GetMem(p^.s,Length(s)+1);
      p^.s^ := s;
   End;
End;

{ insert a new pline into the edit list before p }
Procedure InsertLine(var p : pLine; s : string);
Var
   tmp : pLine;
Begin
   New(tmp);
   GetMem(tmp^.s,Length(s)+1);
   tmp^.s^ := s;
   tmp^.prev := p^.prev;
   tmp^.next := p;
   p^.prev := tmp;
   tmp^.prev^.next := tmp;
   inc(lines);
End;

{ delete a pline from the edit list }
Procedure DeleteLine(var p : pLine);
Var
   tmp : pLine;
Begin
   FreeMem(p^.s,Length(p^.s^));
   tmp := p^.next;
   tmp^.prev := p^.prev;
   p^.prev^.next := tmp;
   Dispose(p);
   p := tmp;
   dec(lines);
   If cline > lines Then cline := lines;
End;

{ return the minimum of two integer values }
Function Min(i1,i2 : integer) : integer;
Begin
  If i1 < i2 Then
     Min := i1
  Else
     Min := i2;
End;

{ fill the edit buffer starting with position h in the edit list }
Procedure LoadLines(var h : pLine);
Var
   tmp : pLine;
   i : integer;
Begin
   FillChar(ss,SizeOf(ss),#0);
   tmp := h;
   If tmp = hdr Then tmp := tmp^.Next;
   For i := 1 to CURLINES Do Begin
      If (tmp <> Nil) and (tmp <> hdr) Then Begin
         ss[i] := tmp^.s^;
         tmp := tmp^.next;
         dlines := i;
      End;
   End;
End;

{ display the edit buffer in the edit window }
Procedure DisplayLines;
Var
   i : integer;
Begin
   With EdWin Do Begin
      For i := 1 to CURLINES Do Begin
         FWrite(1,i,GetColor,Cols,ss[i]);
      End;
   End;
End;

{ free the entire edit list }
Procedure ClearLines(var h : pLine);
Var
   tmp : pLine;
Begin
   If h <> Nil Then Begin
      tmp := h^.prev;
      If (tmp <> h) and (tmp^.s <> Nil) Then Begin
         FreeMem(tmp^.s,Length(tmp^.s^)+1);
         tmp^.next := h;
         Dispose(tmp);
      End;
   End;
   New(h);
   h^.next := h;
   h^.prev := h;
   h^.s := nil;
End;

Function PromptFile(hs : string; var s : string) : integer;
Var
   win : pnWindow;
   ret : integer;
Begin
   New(win,Init(1,1,EdWin.Cols,3,cyan*bg,FRAMED,cyan*bg+white));
   With win^ Do Begin
      PutHeader(hs,GetFrameColor,center);
      FWrite(2,1,GetColor,0,'Filename: ');
      Align(center,center);
      Show;
      s := Edit(12,1,GetColor+white,Cols,12,fnam,ret);
      PromptFile := ret;
      Hide;
   End;
   Dispose(win,Done);
End;

{ prompt for, and open a text file }
Function OpenFile(var f : text; prompt : boolean) : boolean;
Var
   s : string;
   tst : text;
   ret : integer;
Begin
   If prompt Then
      ret := PromptFile('Open File',s)
   Else Begin
      s := fnam;
      ret := nkEnter;
   End;
   If ret = nkEnter Then Begin
      Assign(tst,s);
      {$I-}
      Reset(tst);
      {$I+}
      If IoResult = 0 Then Begin
         Close(tst);
         Assign(f,s);
         Reset(f);
         OpenFile := true;
         fnam := s;
      End Else Begin
         nShowMessage('Could not open file "'+s+'"',79,' Error ',78,true);
         OpenFile := false;
      End;
   End Else
      OpenFile := false;
End;

{ read a file line by line into the edit list }
Procedure ReadFile(var f : text; prompt : boolean);
Var
   err : boolean;
   s : string;
   win : pnWindow;
Begin
   If Not OpenFile(f,prompt) Then Exit;
   ClearLines(hdr);
   lines := 0;
   win := nShowMessage('Reading "'+fnam+'"...',47,' Open File ',46,false);
   {$I-}
   Repeat
      If Not Eof(f) Then Begin
         Readln(f,s);
         err := (IoResult <> 0);
         If Not Err Then InsertLine(hdr,s);
      End;
   Until Eof(f) or err;
   Close(f);
   {$I+}
   win^.Hide;
   win^.Done;
   line1 := hdr^.next;
   line := line1;
   LoadLines(line1);
   DisplayLines;
   idx := 1;
End;

{ save the edit list to disk }
Procedure SaveFile(var f : text);
Var
   tmp : text;
   s,
   tnam : string;
   cur : pLine;
   win : pnWindow;
Begin
   If PromptFile('Save File',s) = nkEsc Then
      Exit
   Else
      fnam := s;
   tnam := fnam+'~';
   Assign(tmp,tnam);
   Assign(f,fnam);
   win := nShowMessage('Saving "'+fnam+'"...',47,' Save File ',46,false);
   {$I-}
   Reset(tmp);
   If IoResult = 0 Then Begin
      Close(tmp);
      Erase(tmp);
      Rename(f,tnam);
      Assign(f,fnam);
   End;
   ReWrite(f);
   cur := hdr^.next;
   Repeat
      If cur <> hdr Then Writeln(f,cur^.s^);
      cur := cur^.next;
   Until cur = hdr;
   Close(f);
   {$I+}
   win^.Hide;
   win^.Done;
End;

{ make the menu appear active }
Procedure MenuUp;
Begin
   With mnu0 Do Begin
      SetColor(48);
      SetCursorColor(79);
      Show;
   End;
   StatWin.FWrite(1,1,StatWin.GetColor,0,'Esc=Edit');
End;

{ make the menu appear inactive }
Procedure MenuDown;
Begin
   With mnu0 Do Begin
      SetColor(56);
      SetCursorColor(56);
      Show;
   End;
   StatWin.FWrite(1,1,StatWin.GetColor,0,'Esc=Menu');
End;

{ execute the File submenu }
Procedure Menu_File;
Begin
   mnu0.SetIndex(1);
   MenuUp;
   New(mnu1,Init(1,1,0,3,1,48,79,8,FRAMED,62));
   With mnu1^ Do Begin
      Add('Open');
      Add('Save');
      Add('Exit - F10');
      Post; { need the item count for move }
      Move(1,nMaxRows-Count-2);
      Start;
      Case Index of
         1 : ReadFile(tf,true);
         2 : SaveFile(tf);
         3 : Finished := true;
      End;
      Hide;
   End;
   Dispose(mnu1,Done);
   MenuDown;
End;

{ display the help screen }
Procedure Help;
Var
   hwin : pnWindow;
Begin
   mnu0.SetIndex(4);
   MenuUp;
   New(hwin,Init(1,1,40,20,62,FRAMED,49));
   With hwin^ Do Begin
      Align(center,center);
      PutHeader('Edit_Demo Help',15,center);
      FWrite(2, 2,63,0,'Ctrl/Q    - Move to column 1');
      FWrite(2, 3,63,0,'Ctrl/W    - Move to end of line');
      FWrite(2, 4,63,0,'Ctrl/A    - Move to previous word');
      FWrite(2, 5,63,0,'Ctrl/F    - Move to next word');
      FWrite(2, 6,63,0,'Ctrl/G    - Delete character');
      FWrite(2, 7,63,0,'Ctrl/H    - Destructive Backspace');
      FWrite(2, 8,63,0,'Ctrl/D    - Move forward one column');
      FWrite(2, 9,63,0,'Ctrl/S    - Move back one column');
      FWrite(2,10,63,0,'Ctrl/I    - Toggle Insert/Overwrite');
      FWrite(2,11,63,0,'Ctrl/P    - Embed control character');
      FWrite(2,12,63,0,'Ctrl/L    - Goto line number');
      FWrite(2,13,63,0,'Ctrl/N    - Insert new line');
      FWrite(2,14,63,0,'Ctrl/Y    - Delete current line');
      FWrite(2,15,63,0,'Ctrl/X    - Move down one line');
      FWrite(2,16,63,0,'Ctrl/E    - Move up one line');
      FWrite(2,17,63,0,'Esc/1..0  - F1..F10');
      Show;
      Repeat Until Keypressed;
      While KeyPressed Do ReadKey;
      Hide;
   End;
   Dispose(hwin,Done);
   MenuDown;
End;

{ goto the specified line in the edit buffer }
Function GotoLine : boolean;
Var
   gwin : pnWindow;
   l,
   ii : longint;
   esc : boolean;
   aline : pline;
Begin
   New(gwin,Init(1,1,40,3,62,FRAMED,49));
   With gwin^ Do Begin
      Align(center,center);
      PutHeader('Goto Line Number',15,center);
      FWrite(2,1,63,0,'Line: ');
      Show;
      ec.ClearMode := true;
      ii := EditNumber(8,1,63,8,0,'',cline,1,lines,esc);
{      If esc or not (i in [1..lines]) Then i := ii;}
      Hide;
   End;
   Dispose(gwin,Done);
   If Not esc Then Begin
      l := 0;
      aline := hdr;
      Repeat
         inc(l);
         aline := aline^.next;
      Until (l = ii);
      line1 := aline;
      cline := l;
   End;
   GotoLine := (Not esc);
End;

{ initialize the global stuff }
Procedure EditInit;
Begin
   With mnu0 Do Begin
      Init(1,1,45,1,5,56,56,7,NOFRAME,0);
      Add('File');
      Add('InsLn');
      Add('DelLn');
      Add('Help');
      Add('Exit');
      Post;
      Align(left,bottom);
   End;
   With StatWin Do Begin
      Init(1,1,nStdScr.Cols-(mnu0.Wind^.Cols),1,48,NOFRAME,0);
      Align(right,bottom);
      Show;
   End;
   MenuDown;
   With EdWin Do Begin
      Init(1,1,nStdScr.Cols,nStdScr.Rows-1,30,FRAMED,31);
      PutHeader(' oCrt Editor Demonstration ',15,center);
      Show;
      GotoXY(1,1);
      {--------------------------------------------------------------------
        The next line causes sedit to exit after every keystroke so we can
        capture the insert mode and cursor positions for display update.
        Alternatively, we could setup an ec.Special string to exit only on
        certain keystrokes of interest.
       --------------------------------------------------------------------}
      ec.ExitMode := true;
      { too re-assign a built-in key, put it in ec.special,
        then use it in the case statement below

      EdWin.ec.Special := EdWin.ec.Special + #5;
      }
      { now let's bind some keystrokes to the editor window }
      ec.AddChMap(^a#0#0+chr(nKeyCtrlLeft));
      ec.AddChMap(^s#0#0+chr(nKeyLeft));
      ec.AddChMap(^f#0#0+chr(nKeyCtrlRight));
      ec.AddChMap(^d#0#0+chr(nKeyRight));
      ec.AddChMap(^e#0#0+chr(nKeyUp));
      ec.AddChMap(^x#0#0+chr(nKeyDown));
      ec.AddChMap(^q#0#0+chr(nKeyHome));
      ec.AddChMap(^w#0#0+chr(nKeyEnd));
      { define the number of edit window rows }
      CURLINES := Min(MAXLINES,Rows);
   End;
   FillChar(ss,SizeOf(ss),#0);
   nEscDelay(250);
   idx := 1;
   Finished := false;
   mactive := false;
   ClearLines(hdr);
   If ParamCount > 0 Then Begin
      fnam := ParamStr(1);
      ReadFile(tf,false);
   End Else
      fnam := '';
   { an empty list? }
   If hdr^.next = hdr Then Begin
      InsertLine(hdr,'');
      line1 := hdr^.next;
      line := line1;
      dlines := 1;
   End;
   cline := 1;
End;

Begin
   EditInit;
   Repeat
      With EdWin Do Begin
         Case ec.InsMode of
            true : StatWin.FWrite(11,1,StatWin.GetColor,0,'Ins');
            false: StatWin.FWrite(11,1,StatWin.GetColor,0,'Ovr');
         End;
         Str(WhereX:0,xp);
         Str(cline:0,yp);
         StatWin.FWrite(16,1,StatWin.GetColor,StatWin.Cols,'Col:'+xp+'  Row:'+yp);
         If mactive Then Begin
            With mnu0 Do Begin
               MenuUp;
               Start;
               Case Index Of
                  1 : cv := nkAltF;
                  2 : cv := nkF1;
                  3 : cv := nkF2;
                  4 : cv := nkF3;
                  5 : cv := nkF10;
                  Else cv := 0;
               End;
               MenuDown;
               Show;
            End;
            mactive := false;
            Active;
            GotoXY(WhereX,WhereY);
         End Else Begin
            ss[idx] := Edit(1,idx,26,Cols,WhereX,ss[idx],cv);
            FWrite(1,idx,GetColor,Cols,ss[idx]);
            ReallocateLine(line,ss[idx]);
         End;
         Case cv of
                12 : If GotoLine Then Begin
                        idx := 1;
                        LoadLines(line1);
                        DisplayLines;
                     End;
            {5,}
            nkUp   : Begin
                        dec(idx);
                        dec(cline);
                        If (idx < 1) and (line1^.prev <> hdr) Then Begin
                           line1 := line1^.prev;
                           LoadLines(line1);
                           DisplayLines;
                        End;
                     End;
            nkDown : Begin
                        inc(idx);
                        inc(cline);
                        If idx > CURLINES Then Begin
                           line1 := line1^.next;
                           LoadLines(line1);
                           DisplayLines;
                        End;
                     End;
            nkPgUp : Begin
                        For xi := 1 to CURLINES Do Begin
                           line1 := line1^.prev;
                           dec(cline);
                           If line1 = hdr Then
                              line1 := line1^.next;
                        End;
                        LoadLines(line1);
                        DisplayLines;
                     End;
            nkPgDn : Begin
                        If dlines = CURLINES Then Begin
                           For xi := 1 to CURLINES Do Begin
                              inc(cline);
                              line1 := line1^.next;
                              If line1 = hdr Then
                                 line1 := line1^.prev;
                           End;
                           LoadLines(line1);
                           DisplayLines;
                        End;
                     End;
            nkEnter: Begin
                        GotoXY(1,WhereY);
                        If line^.next = hdr Then Begin
                           InsertLine(hdr,'');
                           If dlines < CURLINES Then inc(dlines);
                        End;
                        If idx < CURLINES Then
                           inc(idx)
                        Else Begin
                           line1 := line1^.next;
                           LoadLines(line1);
                           DisplayLines;
                        End;
                        inc(cline);
                     End;
            14, { ctrl/n }
            nkF1   : Begin
                        { first displayed line? }
                        If line1 = line Then Begin
                           line1 := line1^.prev;
                           InsertLine(line,'');
                           line1 := line1^.next;
                        End Else
                           InsertLine(line,'');
                        LoadLines(line1);
                        DisplayLines;
                     End;
            25, { ctrl/y }
            nkF2   : Begin
                        { first displayed line? }
                        If line1 = line Then line1 := line^.next;
                        DeleteLine(line);
                        LoadLines(line1);
                        DisplayLines;
                     End;
            nkAltH,
            nkF3   : Help;
            nkEsc  : mactive := true;
            nkF10  : Finished := true;
            nkAltF : menu_file;
         End;
         Active;
         If idx > CURLINES Then idx := CURLINES; { keep in window, }
         If idx > dlines Then idx := dlines;     { but not below last }
         If idx < 1 Then idx := 1;
         If cline < 1 Then cline := 1;
         If cline > lines Then cline := lines;
         GotoXY(WhereX,idx);
         line := line1;
         For xi := 1 to idx-1 Do Begin
            line := line^.next;
         End;
      End;
   Until Finished;
   ClearLines(hdr);
   EdWin.Done;
   StatWin.Done;
   ClrScr;
End.
