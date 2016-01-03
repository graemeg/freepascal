{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl

    This file implements the win32 gui support for the graph unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit Graph;
<<<<<<< HEAD
<<<<<<< HEAD

interface

{ used to create a file containing all calls to WM_PAINT
  WARNING this probably creates HUGE files PM }
{ $define DEBUG_WM_PAINT}

{ debug child window handling }
{ $define DEBUGCHILDS}

{
  To be able to use standard file handles in the graph thread,
  we need to use the system functions handling threads,
  to ensure that thread varaibles are correctly initialized.
  This new default setting can be overridden by defining
  USE_WINDOWS_API_THREAD_FUNCTIONS macro.

  Use API thread functions by default, to avoid interferences due to
  initialization of threadvars, this solves e.g. #27508 (which does not
  mean though that interworking with CRT is guranteed in any way)

  undefine this when debugging the graph unit due to writelns in the
  debug code }
{$if not(defined(DEBUG_WM_PAINT)) and not(defined(DEBUGCHILDS))}
{$define USE_WINDOWS_API_THREAD_FUNCTIONS}
{$endif not(defined(DEBUG_WM_PAINT)) and not(defined(DEBUGCHILDS))}

{$ifndef USE_WINDOWS_API_THREAD_FUNCTIONS}
  {$define USE_SYSTEM_BEGIN_THREAD}
{$endif ndef USE_WINDOWS_API_THREAD_FUNCTIONS}

=======
interface

>>>>>>> graemeg/fixes_2_2
=======
interface

>>>>>>> origin/fixes_2_2
uses
  windows;

{$i graphh.inc}

  var
    { this procedure allows to hook keyboard messages }
    charmessagehandler : WndProc;
    { this procedure allows to hook mouse messages }
    mousemessagehandler : WndProc;
    { this procedure allows to wm_command messages }
    commandmessagehandler : WndProc;
    NotifyMessageHandler : WndProc;

    OnGraphWindowCreation : procedure;

    GraphWindow,ParentWindow : HWnd;
    // this allows direct drawing to the window
    bitmapdc : hdc;
    windc : hdc;

  const
    { predefined window style }
    { we shouldn't set CS_DBLCLKS here }
    { because most dos applications    }
    { handle double clicks on it's own }
    graphwindowstyle : DWord = cs_hRedraw or cs_vRedraw;

    windowtitle : pchar = 'Graph window application';
    menu : hmenu = 0;
    icon : hicon = 0;
    drawtoscreen : boolean = true;
    drawtobitmap : boolean = true;
    // the graph window can be a child window, this allows to add toolbars
    // to the main window
    UseChildWindow : boolean = false;
    // this allows to specify an offset for the child child window
    ChildOffset : rect = (left:0;top:0;right:0;bottom:0);

CONST

  m640x200x16       = VGALo;
  m640x400x16       = VGAMed;
  m640x480x16       = VGAHi;

  { VESA Specific video modes. }
  m320x200x32k      = $10D;
  m320x200x64k      = $10E;

  m640x400x256      = $100;

  m640x480x256      = $101;
  m640x480x32k      = $110;
  m640x480x64k      = $111;

  m800x600x16       = $102;
  m800x600x256      = $103;
  m800x600x32k      = $113;
  m800x600x64k      = $114;

  m1024x768x16      = $104;
  m1024x768x256     = $105;
  m1024x768x32k     = $116;
  m1024x768x64k     = $117;

  m1280x1024x16     = $106;
  m1280x1024x256    = $107;
  m1280x1024x32k    = $119;
  m1280x1024x64k    = $11A;

  { some extra modes which applies only to GUI }
  mLargestWindow16  = $f0;
  mLargestWindow256 = $f1;
  mLargestWindow32k = $f2;
  mLargestWindow64k = $f3;
  mLargestWindow16M = $f4;
  mMaximizedWindow16 = $f5;
  mMaximizedWindow256 = $f6;
  mMaximizedWindow32k = $f7;
  mMaximizedWindow64k = $f8;
  mMaximizedWindow16M = $f9;


implementation

uses
  strings;

{
   Remarks:
      Colors in 16 color mode:
      ------------------------
        - the behavior of xor/or/and put isn't 100%:
          it is done using the RGB color getting from windows
          instead of the palette index!
        - palette operations aren't supported
      To solve these drawbacks, setpalette must be implemented
      by exchanging the colors in the DCs, further GetPaletteEntry
      must be used when doing xor/or/and operations
}


const
   InternalDriverName = 'WIN32GUI';

{$i graph.inc}

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

{ used to create a file containing all calls to WM_PAINT
  WARNING this probably creates HUGE files PM }
{ $define DEBUG_WM_PAINT}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
   savedscreen : hbitmap;
   graphrunning : boolean;
   graphdrawing : tcriticalsection;
   pens : array[0..15] of HPEN;
{$ifdef DEBUG_WM_PAINT}
   graphdebug : text;

const
   wm_paint_count : longint = 0;
var
{$endif DEBUG_WM_PAINT}
   oldbitmap : hgdiobj;
   pal : ^rgbrec;
//   SavePtr : pointer; { we don't use that pointer }
   MessageThreadHandle : Handle;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef WIN64}
  {$ifdef USE_SYSTEM_BEGIN_THREAD}
     MessageThreadId : Qword;
  {$else}
     MessageThreadId : DWord;
  {$endif}
{$else not WIN64}
   MessageThreadID : DWord;
{$endif not WIN64}
=======
   MessageThreadID : DWord;
>>>>>>> graemeg/fixes_2_2
=======
   MessageThreadID : DWord;
>>>>>>> origin/fixes_2_2

function GetPaletteEntry(r,g,b : word) : word;

  var
     dist,i,index,currentdist : longint;

  begin
     dist:=$7fffffff;
     index:=0;
     for i:=0 to maxcolors do
       begin
          currentdist:=abs(r-pal[i].red)+abs(g-pal[i].green)+
            abs(b-pal[i].blue);
          if currentdist<dist then
            begin
               index:=i;
               dist:=currentdist;
               if dist=0 then
                 break;
            end;
       end;
     GetPaletteEntry:=index;
  end;

procedure PutPixel16Win32GUI(x,y : smallint;pixel : word);

  var
     c : colorref;

  begin
    x:=x+startxviewport;
    y:=y+startyviewport;
    { convert to absolute coordinates and then verify clipping...}
    if clippixels then
      begin
         if (x<startxviewport) or (x>(startxviewport+viewwidth)) or
           (y<StartyViewPort) or (y>(startyviewport+viewheight)) then
           exit;
      end;
    if graphrunning then
      begin
         c:=RGB(pal[pixel].red,pal[pixel].green,pal[pixel].blue);
         EnterCriticalSection(graphdrawing);
         if drawtobitmap then
           SetPixelV(bitmapdc,x,y,c);
         if drawtoscreen then
           SetPixelV(windc,x,y,c);
         LeaveCriticalSection(graphdrawing);
      end;
  end;

function GetPixel16Win32GUI(x,y : smallint) : word;

  var
     c : COLORREF;

  begin
    x:=x+startxviewport;
    y:=y+startyviewport;
    { convert to absolute coordinates and then verify clipping...}
    if clippixels then
      begin
         if (x<startxviewport) or (x>(startxviewport+viewwidth)) or
           (y<StartyViewPort) or (y>(startyviewport+viewheight)) then
           exit;
      end;
    if graphrunning then
      begin
         EnterCriticalSection(graphdrawing);
         c:=Windows.GetPixel(bitmapdc,x,y);
         LeaveCriticalSection(graphdrawing);
         GetPixel16Win32GUI:=GetPaletteEntry(GetRValue(c),GetGValue(c),GetBValue(c));
      end
    else
      begin
        _graphresult:=grerror;
        exit;
      end;
  end;

procedure DirectPutPixel16Win32GUI(x,y : smallint);

  var
     col : longint;
     c,c2 : COLORREF;

  begin
    if graphrunning then
      begin
         EnterCriticalSection(graphdrawing);
         col:=CurrentColor;
         case currentwritemode of
           XorPut:
             Begin
                c2:=Windows.GetPixel(windc,x,y);
                c:=RGB(pal[col].red,pal[col].green,pal[col].blue) xor c2;
                if drawtobitmap then
                  SetPixelV(bitmapdc,x,y,c);
                if drawtoscreen then
                  SetPixelV(windc,x,y,c);
             End;
           AndPut:
             Begin
                c2:=Windows.GetPixel(windc,x,y);
                c:=RGB(pal[col].red,pal[col].green,pal[col].blue) and c2;
                if drawtobitmap then
                  SetPixelV(bitmapdc,x,y,c);
                if drawtoscreen then
                  SetPixelV(windc,x,y,c);
             End;
           OrPut:
             Begin
                c2:=Windows.GetPixel(windc,x,y);
                c:=RGB(pal[col].red,pal[col].green,pal[col].blue) or c2;
                if drawtobitmap then
                  SetPixelV(bitmapdc,x,y,c);
                if drawtoscreen then
                  SetPixelV(windc,x,y,c);
             End
           else
             Begin
                If CurrentWriteMode<>NotPut Then
                  col:=CurrentColor
                Else col := Not(CurrentColor);
                c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                if drawtobitmap then
                  SetPixelV(bitmapdc,x,y,c);
                if drawtoscreen then
                  SetPixelV(windc,x,y,c);
             End
         end;
         LeaveCriticalSection(graphdrawing);
      end;
  end;

var
   bitmapfontverticalcache : array[0..255] of HBITMAP;
   bitmapfonthorizoncache : array[0..255] of HBITMAP;

procedure OutTextXYWin32GUI(x,y : smallint;const TextString : string);

  type
   Tpoint = record
     X,Y: smallint;
   end;
  var
     i,j,k,c       : longint;
     xpos,ypos     : longint;
     counter       : longint;
     cnt1,cnt2     : smallint;
     cnt3,cnt4     : smallint;
     charsize      : word;
     WriteMode     : word;
     curX2, curY2, xpos2, ypos2, x2, y2: graph_float;
     oldvalues     : linesettingstype;
     fontbitmap    : TBitmapChar;
<<<<<<< HEAD
<<<<<<< HEAD
     fontbitmapbyte: byte;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     chr           : char;
     curx2i,cury2i,
     xpos2i,ypos2i : longint;
     charbitmap,oldcharbitmap : HBITMAP;
     chardc : HDC;
     color : longint;
     brushwin,oldbrushwin,brushbitmap,oldbrushbitmap : HBRUSH;
     bitmaprgn,winrgn : HRGN;

  begin
     { save current write mode }
     WriteMode := CurrentWriteMode;
     CurrentWriteMode := NormalPut;
     GetTextPosition(xpos,ypos,textstring);
     X:=X-XPos; Y:=Y+YPos;
     XPos:=X; YPos:=Y;
     CharSize := CurrentTextInfo.Charsize;
     if Currenttextinfo.font=DefaultFont then
     begin
       if CurrentTextInfo.direction=HorizDir then
       { Horizontal direction }
         begin
            if (x>viewwidth) or (y>viewheight) or
              (x<0) or (y<0) then
              begin
                 CurrentWriteMode:=WriteMode;
                 exit;
              end;
            EnterCriticalSection(graphdrawing);
            c:=length(textstring);
            chardc:=CreateCompatibleDC(windc);
            if currentcolor<>white then
              begin
                 color:=RGB(pal[currentcolor].red,pal[currentcolor].green,
                   pal[currentcolor].blue);

                 if drawtoscreen then
                   begin
                      brushwin:=CreateSolidBrush(color);
                      oldbrushwin:=SelectObject(windc,brushwin);
                   end;

                 if drawtobitmap then
                   begin
                      brushbitmap:=CreateSolidBrush(color);
                      oldbrushbitmap:=SelectObject(bitmapdc,brushbitmap);
                   end;
              end;
            inc(x,startxviewport);
            inc(y,startyviewport);

            { let windows do the clipping }
            if drawtobitmap then
              begin
                 bitmaprgn:=CreateRectRgn(startxviewport,startyviewport,
                   startxviewport+viewwidth+1,startyviewport+viewheight+1);
                 SelectClipRgn(bitmapdc,bitmaprgn);
              end;

            if drawtoscreen then
              begin
                 winrgn:=CreateRectRgn(startxviewport,startyviewport,
                   startxviewport+viewwidth+1,startyviewport+viewheight+1);
                 SelectClipRgn(windc,winrgn);
              end;

            for i:=0 to c-1 do
              begin
                 xpos:=x+(i*8)*Charsize;
                 if bitmapfonthorizoncache[byte(textstring[i+1])]=0 then
                   begin
                      charbitmap:=CreateCompatibleBitmap(windc,8,8);
                      if charbitmap=0 then
                        writeln('Bitmap konnte nicht erzeugt werden!');
                      oldcharbitmap:=SelectObject(chardc,charbitmap);
                      Fontbitmap:=TBitmapChar(DefaultFontData[textstring[i+1]]);

                      for j:=0 to 7 do
<<<<<<< HEAD
<<<<<<< HEAD
                        begin
                          fontbitmapbyte:=Fontbitmap[j];
                          for k:=0 to 7 do
                            begin
                              if (fontbitmapbyte and $80)<>0 then
                                SetPixelV(chardc,k,j,$ffffff)
                              else
                                SetPixelV(chardc,k,j,0);
                              fontbitmapbyte:=fontbitmapbyte shl 1;
                            end;
                        end;
=======
=======
>>>>>>> origin/fixes_2_2
                         for k:=0 to 7 do
                           if Fontbitmap[j,k]<>0 then
                             SetPixelV(chardc,k,j,$ffffff)
                           else
                             SetPixelV(chardc,k,j,0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                      bitmapfonthorizoncache[byte(textstring[i+1])]:=charbitmap;
                      SelectObject(chardc,oldcharbitmap);
                   end;
                 oldcharbitmap:=SelectObject(chardc,bitmapfonthorizoncache[byte(textstring[i+1])]);
                 if CharSize=1 then
                   begin
                      if currentcolor=white then
                        begin
                           if drawtoscreen then
                             BitBlt(windc,xpos,y,8,8,chardc,0,0,SRCPAINT);
                           if drawtobitmap then
                             BitBlt(bitmapdc,xpos,y,8,8,chardc,0,0,SRCPAINT);
                        end
                      else
                        begin
                           { could we do this with one pattern operation ?? }
                           { we would need something like DSnaSPao }
                           if drawtoscreen then
                             begin
                                // ROP $00220326=DSna
                                BitBlt(windc,xpos,y,8,8,chardc,0,0,$00220326);
                                // ROP $00EA02E9 = DPSao
                                BitBlt(windc,xpos,y,8,8,chardc,0,0,$00EA02E9);
                             end;

                           if drawtobitmap then
                             begin
                                BitBlt(bitmapdc,xpos,y,8,8,chardc,0,0,$00220326);
                                BitBlt(bitmapdc,xpos,y,8,8,chardc,0,0,$00EA02E9);
                             end;
                        end;
                   end
                 else
                   begin
                      if currentcolor=white then
                        begin
                           if drawtoscreen then
                             StretchBlt(windc,xpos,y,8*charsize,8*charsize,chardc,0,0,8,8,SRCPAINT);
                           if drawtobitmap then
                             StretchBlt(bitmapdc,xpos,y,8*charsize,8*charsize,chardc,0,0,8,8,SRCPAINT);
                        end
                      else
                        begin
                           { could we do this with one pattern operation ?? }
                           { we would need something like DSnaSPao }
                           if drawtoscreen then
                             begin
                                // ROP $00220326=DSna
                                StretchBlt(windc,xpos,y,8*charsize,8*charsize,chardc,0,0,8,8,$00220326);
                                // ROP $00EA02E9 = DPSao
                                StretchBlt(windc,xpos,y,8*charsize,8*charsize,chardc,0,0,8,8,$00EA02E9);
                             end;
                           if drawtobitmap then
                             begin
                                StretchBlt(bitmapdc,xpos,y,8*charsize,8*charsize,chardc,0,0,8,8,$00220326);
                                StretchBlt(bitmapdc,xpos,y,8*charsize,8*charsize,chardc,0,0,8,8,$00EA02E9);
                             end;
                        end;
                   end;
                 SelectObject(chardc,oldcharbitmap);
              end;
           if currentcolor<>white then
             begin
                 if drawtoscreen then
                   begin
                      SelectObject(windc,oldbrushwin);
                      DeleteObject(brushwin);
                   end;

                 if drawtobitmap then
                   begin
                      SelectObject(bitmapdc,oldbrushbitmap);
                      DeleteObject(brushbitmap);
                   end;
             end;
           { release clip regions }
           if drawtobitmap then
             begin
               SelectClipRgn(bitmapdc,0);
               DeleteObject(bitmaprgn);
             end;
           if drawtoscreen then
             begin
                SelectClipRgn(windc,0);
                DeleteObject(winrgn);
             end;
           DeleteDC(chardc);
           LeaveCriticalSection(graphdrawing);
         end
       else
       { Vertical direction }
         begin
            if (x>viewwidth) or (y>viewheight) or
              (x<0) or (y<0) then
              begin
                 CurrentWriteMode:=WriteMode;
                 exit;
              end;
            EnterCriticalSection(graphdrawing);
            c:=length(textstring);
            chardc:=CreateCompatibleDC(windc);
            if currentcolor<>white then
              begin
                 color:=RGB(pal[currentcolor].red,pal[currentcolor].green,
                   pal[currentcolor].blue);

                 if drawtoscreen then
                   begin
                      brushwin:=CreateSolidBrush(color);
                      oldbrushwin:=SelectObject(windc,brushwin);
                   end;

                 if drawtobitmap then
                   begin
                      brushbitmap:=CreateSolidBrush(color);
                      oldbrushbitmap:=SelectObject(bitmapdc,brushbitmap);
                   end;
              end;
            inc(x,startxviewport);
            inc(y,startyviewport);
            { let windows do the clipping }
            if drawtoscreen then
              begin
                 winrgn:=CreateRectRgn(startxviewport,startyviewport,
                   startxviewport+viewwidth+1,startyviewport+viewheight+1);
                 SelectClipRgn(windc,winrgn);
              end;

            if drawtobitmap then
              begin
                 bitmaprgn:=CreateRectRgn(startxviewport,startyviewport,
                   startxviewport+viewwidth+1,startyviewport+viewheight+1);
                 SelectClipRgn(bitmapdc,bitmaprgn);
              end;
            for i:=0 to c-1 do
              begin
                 ypos:=y+1-((i+1)*8)*CharSize;
                 if bitmapfontverticalcache[byte(textstring[i+1])]=0 then
                   begin
                      charbitmap:=CreateCompatibleBitmap(windc,8,8);
                      if charbitmap=0 then
                        writeln('Bitmap konnte nicht erzeugt werden!');
                      oldcharbitmap:=SelectObject(chardc,charbitmap);
                      Fontbitmap:=TBitmapChar(DefaultFontData[textstring[i+1]]);

                      for j:=0 to 7 do
<<<<<<< HEAD
<<<<<<< HEAD
                        begin
                          fontbitmapbyte:=Fontbitmap[j];
                          for k:=0 to 7 do
                            begin
                              if (fontbitmapbyte and $80)<>0 then
                                SetPixelV(chardc,j,7-k,$ffffff)
                              else
                                SetPixelV(chardc,j,7-k,0);
                              fontbitmapbyte:=fontbitmapbyte shl 1;
                            end;
                        end;
=======
=======
>>>>>>> origin/fixes_2_2
                         for k:=0 to 7 do
                           if Fontbitmap[j,k]<>0 then
                             SetPixelV(chardc,j,7-k,$ffffff)
                           else
                             SetPixelV(chardc,j,7-k,0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                      bitmapfontverticalcache[byte(textstring[i+1])]:=charbitmap;
                      SelectObject(chardc,oldcharbitmap);
                   end;
                 oldcharbitmap:=SelectObject(chardc,bitmapfontverticalcache[byte(textstring[i+1])]);
                 if CharSize=1 then
                   begin
                      if currentcolor=white then
                        begin
                           if drawtoscreen then
                             BitBlt(windc,x,ypos,8,8,chardc,0,0,SRCPAINT);
                           if drawtobitmap then
                             BitBlt(bitmapdc,x,ypos,8,8,chardc,0,0,SRCPAINT);
                        end
                      else
                        begin
                           { could we do this with one pattern operation ?? }
                           { we would need something like DSnaSPao }
                           if drawtoscreen then
                             begin
                                // ROP $00220326=DSna
                                BitBlt(windc,x,ypos,8,8,chardc,0,0,$00220326);
                                // ROP $00EA02E9 = DPSao
                                BitBlt(windc,x,ypos,8,8,chardc,0,0,$00EA02E9);
                             end;
                           if drawtobitmap then
                             begin
                                BitBlt(bitmapdc,x,ypos,8,8,chardc,0,0,$00220326);
                                BitBlt(bitmapdc,x,ypos,8,8,chardc,0,0,$00EA02E9);
                             end;
                        end;
                   end
                 else
                   begin
                      if currentcolor=white then
                        begin
                           if drawtoscreen then
                             StretchBlt(windc,x,ypos,8*charsize,8*charsize,chardc,0,0,8,8,SRCPAINT);
                           if drawtobitmap then
                             StretchBlt(bitmapdc,x,ypos,8*charsize,8*charsize,chardc,0,0,8,8,SRCPAINT);
                        end
                      else
                        begin
                           { could we do this with one pattern operation ?? }
                           { we would need something like DSnaSPao }
                           if drawtoscreen then
                             begin
                                // ROP $00220326=DSna
                                StretchBlt(windc,x,ypos,8*charsize,8*charsize,chardc,0,0,8,8,$00220326);
                                // ROP $00EA02E9 = DPSao
                                StretchBlt(windc,x,ypos,8*charsize,8*charsize,chardc,0,0,8,8,$00EA02E9);
                             end;
                           if drawtobitmap then
                             begin
                                StretchBlt(bitmapdc,x,ypos,8*charsize,8*charsize,chardc,0,0,8,8,$00220326);
                                StretchBlt(bitmapdc,x,ypos,8*charsize,8*charsize,chardc,0,0,8,8,$00EA02E9);
                             end;
                        end;
                   end;
                 SelectObject(chardc,oldcharbitmap);
              end;
           if currentcolor<>white then
             begin
                if drawtoscreen then
                  begin
                     SelectObject(windc,oldbrushwin);
                     DeleteObject(brushwin);
                  end;

                if drawtobitmap then
                  begin
                     SelectObject(bitmapdc,oldbrushbitmap);
                     DeleteObject(brushbitmap);
                  end;
             end;
           { release clip regions }
           if drawtoscreen then
             begin
                SelectClipRgn(windc,0);
                DeleteObject(winrgn);
             end;
           if drawtobitmap then
             begin
                SelectClipRgn(bitmapdc,0);
                DeleteObject(bitmaprgn);
             end;
           DeleteDC(chardc);
           LeaveCriticalSection(graphdrawing);
        end;
     end else
     { This is a stroked font which is already loaded into memory }
       begin
          getlinesettings(oldvalues);
          { reset line style to defaults }
          setlinestyle(solidln,oldvalues.pattern,normwidth);
          if Currenttextinfo.direction=vertdir then
             xpos:=xpos + Textheight(textstring);
          CurX2:=xpos; xpos2 := curX2; x2 := xpos2;
          CurY2:=ypos; ypos2 := curY2; y2 := ypos2;
{              x:=xpos; y:=ypos;}

          for i:=1 to length(textstring) do
            begin
               c:=byte(textstring[i]);
{                   Stroke_Count[c] := }
               unpack( fonts[CurrentTextInfo.font].instr,
                 fonts[CurrentTextInfo.font].Offsets[c], Strokes );
               counter:=0;
               while true do
                 begin
                     if CurrentTextInfo.direction=VertDir then
                       begin
                         xpos2:=x2-(Strokes[counter].Y*CurrentYRatio);
                         ypos2:=y2-(Strokes[counter].X*CurrentXRatio);
                       end
                     else
                       begin
                         xpos2:=x2+(Strokes[counter].X*CurrentXRatio) ;
                         ypos2:=y2-(Strokes[counter].Y*CurrentYRatio) ;
                       end;
                     case opcodes(Strokes[counter].opcode) of
                       _END_OF_CHAR: break;
                       _DO_SCAN: begin
                                { Currently unsupported };
                                end;
                       _MOVE : Begin
                                 CurX2 := XPos2;
                                 CurY2 := YPos2;
                               end;
                       _DRAW: Begin
                                curx2i:=trunc(CurX2);
                                cury2i:=trunc(CurY2);
                                xpos2i:=trunc(xpos2);
                                ypos2i:=trunc(ypos2);
                                { this optimization doesn't matter that much
                                if (curx2i=xpos2i) then
                                  begin
                                     if (cury2i=ypos2i) then
                                       putpixel(curx2i,cury2i,currentcolor)
                                     else if (cury2i+1=ypos2i) or
                                       (cury2i=ypos2i+1) then
                                        begin
                                           putpixel(curx2i,cury2i,currentcolor);
                                           putpixel(curx2i,ypos2i,currentcolor);
                                        end
                                      else
                                        Line(curx2i,cury2i,xpos2i,ypos2i);
                                  end
                                else if (cury2i=ypos2i) then
                                  begin
                                     if (curx2i+1=xpos2i) or
                                       (curx2i=xpos2i+1) then
                                        begin
                                           putpixel(curx2i,cury2i,currentcolor);
                                           putpixel(xpos2i,cury2i,currentcolor);
                                        end
                                      else
                                        Line(curx2i,cury2i,xpos2i,ypos2i);
                                  end
                                else
                                }
                                Line(curx2i,cury2i,xpos2i,ypos2i);
                                CurX2:=xpos2;
                                CurY2:=ypos2;
                              end;
                         else
                           Begin
                           end;
                        end;
                    Inc(counter);
                 end; { end while }
               if Currenttextinfo.direction=VertDir then
                 y2:=y2-(byte(fonts[CurrenttextInfo.font].widths[c])*CurrentXRatio)
               else
                 x2:=x2+(byte(fonts[Currenttextinfo.font].widths[c])*CurrentXRatio);
            end;
          setlinestyle( oldvalues.linestyle, oldvalues.pattern, oldvalues.thickness);
       end;
    { restore write mode }
    CurrentWriteMode := WriteMode;
  end;

procedure HLine16Win32GUI(x,x2,y: smallint);

   var
      c,c2 : COLORREF;
      col,i : longint;
      oldpen,pen : HPEN;

   Begin
      if graphrunning then
        begin
           { must we swap the values? }
           if x>x2 then
             Begin
               x:=x xor x2;
               x2:=x xor x2;
               x:=x xor x2;
             end;
           if ClipPixels then
             begin
                if (x>ViewWidth) or (y<0) or (y>ViewHeight) or (x2<0) then
                  exit;
                if x<0 then
                  x:=0;
                if x2>ViewWidth then
                  x2:=ViewWidth;
             end;
           X:=X+StartXViewPort;
           X2:=X2+StartXViewPort;
           Y:=Y+StartYViewPort;
           Case CurrentWriteMode of
             AndPut:
               Begin
                  EnterCriticalSection(graphdrawing);
                  col:=CurrentColor;
                  for i:=x to x2 do
                    begin
                       c2:=Windows.GetPixel(windc,i,y);
                       c:=RGB(pal[col].red,pal[col].green,pal[col].blue) and c2;
                       if drawtobitmap then
                         SetPixelV(bitmapdc,i,y,c);

                       if drawtoscreen then
                         SetPixelV(windc,i,y,c);
                    end;
                  LeaveCriticalSection(graphdrawing);
               End;
             XorPut:
               Begin
                  EnterCriticalSection(graphdrawing);
                  col:=CurrentColor;
                  for i:=x to x2 do
                    begin
                       c2:=Windows.GetPixel(windc,i,y);
                       c:=RGB(pal[col].red,pal[col].green,pal[col].blue) xor c2;

                       if drawtobitmap then
                         SetPixelV(bitmapdc,i,y,c);

                       if drawtoscreen then
                         SetPixelV(windc,i,y,c);
                    end;
                  LeaveCriticalSection(graphdrawing);
               End;
             OrPut:
               Begin
                  EnterCriticalSection(graphdrawing);
                  col:=CurrentColor;
                  for i:=x to x2 do
                    begin
                       c2:=Windows.GetPixel(windc,i,y);
                       c:=RGB(pal[col].red,pal[col].green,pal[col].blue) or c2;

                       if drawtobitmap then
                         SetPixelV(bitmapdc,i,y,c);

                       if drawtoscreen then
                         SetPixelV(windc,i,y,c);
                    end;
                  LeaveCriticalSection(graphdrawing);
               End
             Else
               Begin
                  If CurrentWriteMode<>NotPut Then
                    col:=CurrentColor
                  Else col:=Not(CurrentColor);
                  EnterCriticalSection(graphdrawing);
                  if x2-x<=2 then
                    begin
                       c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                       for x := x to x2 do
                         begin
                            if drawtobitmap then
                              SetPixelV(bitmapdc,x,y,c);
                            if drawtoscreen then
                              SetPixelV(windc,x,y,c);
                         end;
                    end
                  else
                    begin
                       if (col>=0) and (col<=high(pens)) then
                         begin
                            if pens[col]=0 then
                              begin
                                 c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                                 pens[col]:=CreatePen(PS_SOLID,1,c);
                              end;
                            pen:=pens[col];
                         end
                       else
                         begin
                            c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                            pen:=CreatePen(PS_SOLID,1,c);
                         end;

                       if drawtobitmap then
                         begin
                            oldpen:=SelectObject(bitmapdc,pen);
                            Windows.MoveToEx(bitmapdc,x,y,nil);
                            Windows.LineTo(bitmapdc,x2+1,y);
                            SelectObject(bitmapdc,oldpen);
                         end;

                       if drawtoscreen then
                         begin
                            oldpen:=SelectObject(windc,pen);
                            Windows.MoveToEx(windc,x,y,nil);
                            Windows.LineTo(windc,x2+1,y);
                            SelectObject(windc,oldpen);
                         end;

                       if (col<0) or (col>high(pens)) then
                         DeleteObject(pen);
                    end;
                   LeaveCriticalSection(graphdrawing);
               End;
           End;
        end;
   end;

procedure VLine16Win32GUI(x,y,y2: smallint); {$ifndef fpc}far;{$endif fpc}

 var
  ytmp: smallint;
  col,c : longint;
  oldpen,pen : HPEN;

Begin
  { must we swap the values? }
  if y >= y2 then
   Begin
     ytmp := y2;
     y2 := y;
     y:= ytmp;
   end;
 if ClipPixels then
   begin
      if (x>ViewWidth) or (x<0) or (y>ViewHeight) or (y2<0) then
        exit;
      if y<0 then
        y:=0;
      if y2>ViewHeight then
        y2:=ViewHeight;
   end;
  { First convert to global coordinates }
  X   := X + StartXViewPort;
  Y2  := Y2 + StartYViewPort;
  Y   := Y + StartYViewPort;
  if currentwritemode=normalput then
    begin
       col:=CurrentColor;
       EnterCriticalSection(graphdrawing);
       if y2-y<=2 then
         begin
            c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
            for y := y to y2 do
              begin
                 if drawtobitmap then
                   SetPixelV(bitmapdc,x,y,c);
                 if drawtoscreen then
                   SetPixelV(windc,x,y,c);
              end;
         end
       else
         begin
            if (col>=0) and (col<=high(pens)) then
              begin
                 if pens[col]=0 then
                   begin
                      c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                      pens[col]:=CreatePen(PS_SOLID,1,c);
                   end;
                 pen:=pens[col];
              end
            else
              begin
                 c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                 pen:=CreatePen(PS_SOLID,1,c);
              end;

            if drawtobitmap then
              begin
                 oldpen:=SelectObject(bitmapdc,pen);
                 Windows.MoveToEx(bitmapdc,x,y,nil);
                 Windows.LineTo(bitmapdc,x,y2+1);
                 SelectObject(bitmapdc,oldpen);
              end;

            if drawtoscreen then
              begin
                 oldpen:=SelectObject(windc,pen);
                 Windows.MoveToEx(windc,x,y,nil);
                 Windows.LineTo(windc,x,y2+1);
                 SelectObject(windc,oldpen);
              end;
            if (col<0) or (col>high(pens)) then
              DeleteObject(pen);
         end;
       LeaveCriticalSection(graphdrawing);
    end
  else
    for y := y to y2 do Directputpixel(x,y)
End;

procedure Circle16Win32GUI(X, Y: smallint; Radius:Word);

  var
     bitmaprgn,winrgn : HRGN;
     col,c : longint;
     oldpen,pen : HPEN;
     OriginalArcInfo: ArcCoordsType;
     OldWriteMode: word;

  begin
     if (Radius = 0) then
          Exit;

     if (Radius = 1) then
       begin
          { only normal put mode is supported by a call to PutPixel }
          PutPixel(X, Y, CurrentColor);
          Exit;
       end;

     if (Radius = 2) then
       begin
          { only normal put mode is supported by a call to PutPixel }
          PutPixel(X-1, Y, CurrentColor);
          PutPixel(X+1, Y, CurrentColor);
          PutPixel(X, Y-1, CurrentColor);
          PutPixel(X, Y+1, CurrentColor);
          Exit;
       end;

     if LineInfo.Thickness = Normwidth then
       begin
          EnterCriticalSection(graphdrawing);
          { let windows do the clipping }
          if drawtobitmap then
            begin
               bitmaprgn:=CreateRectRgn(startxviewport,startyviewport,
                 startxviewport+viewwidth+1,startyviewport+viewheight+1);
               SelectClipRgn(bitmapdc,bitmaprgn);
            end;

          if drawtoscreen then
            begin
               winrgn:=CreateRectRgn(startxviewport,startyviewport,
                 startxviewport+viewwidth+1,startyviewport+viewheight+1);
               SelectClipRgn(windc,winrgn);
            end;

          inc(x,StartXViewPort);
          inc(y,StartYViewPort);
          col:=CurrentColor;

          if (col>=0) and (col<=high(pens)) then
            begin
               if pens[col]=0 then
                 begin
                    c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
                    pens[col]:=CreatePen(PS_SOLID,1,c);
                 end;
               pen:=pens[col];
            end
          else
            begin
               c:=RGB(pal[col].red,pal[col].green,pal[col].blue);
               pen:=CreatePen(PS_SOLID,1,c);
            end;

          if drawtobitmap then
            begin
               oldpen:=SelectObject(bitmapdc,pen);
               windows.arc(bitmapdc,x-radius,y-radius,x+radius,y+radius,
                 x,y-radius,x,y-radius);
               SelectObject(bitmapdc,oldpen);
            end;

          if drawtoscreen then
            begin
               oldpen:=SelectObject(windc,pen);
               windows.arc(windc,x-radius,y-radius,x+radius,y+radius,
                 x,y-radius,x,y-radius);
               SelectObject(windc,oldpen);
            end;

          if (col<0) or (col>high(pens)) then
            DeleteObject(pen);
          { release clip regions }
          if drawtoscreen then
            begin
               SelectClipRgn(windc,0);
               DeleteObject(winrgn);
            end;
          if drawtobitmap then
            begin
               SelectClipRgn(bitmapdc,0);
               DeleteObject(bitmaprgn);
            end;
          LeaveCriticalSection(graphdrawing);
       end
     else
       begin
          { save state of arc information }
          { because it is not needed for  }
          { a circle call.                }
          move(ArcCall,OriginalArcInfo, sizeof(ArcCall));
          InternalEllipse(X,Y,Radius,Radius,0,360,{$ifdef fpc}@{$endif}DummyPatternLine);
          { restore arc information }
          move(OriginalArcInfo, ArcCall,sizeof(ArcCall));
       end;
 end;

{
Procedure PutImageWin32GUI(X,Y: smallint; var Bitmap; BitBlt: Word); {$ifndef fpc}far;{$endif fpc}
type
  pt = array[0..$fffffff] of word;
  ptw = array[0..2] of longint;
var
  k: longint;
  oldCurrentColor: word;
  oldCurrentWriteMode, i, j, y1, x1, deltaX, deltaX1, deltaY: smallint;
Begin
{$ifdef logging}
  LogLn('putImage at ('+strf(x)+','+strf(y)+') with width '+strf(ptw(Bitmap)[0])+
    ' and height '+strf(ptw(Bitmap)[1]));
  deltaY := 0;
{$endif logging}
  inc(x,startXViewPort);
  inc(y,startYViewPort);
  x1 := ptw(Bitmap)[0]+x; { get width and adjust end coordinate accordingly }
  y1 := ptw(Bitmap)[1]+y; { get height and adjust end coordinate accordingly }

  deltaX := 0;
  deltaX1 := 0;
  k := 3 * sizeOf(Longint) div sizeOf(Word); { Three reserved longs at start of bitmap }
 { check which part of the image is in the viewport }
  if clipPixels then
    begin
      if y < startYViewPort then
        begin
          deltaY := startYViewPort - y;
          inc(k,(x1-x+1)*deltaY);
          y := startYViewPort;
         end;
      if y1 > startYViewPort+viewHeight then
        y1 := startYViewPort+viewHeight;
      if x < startXViewPort then
        begin
          deltaX := startXViewPort-x;
          x := startXViewPort;
        end;
      if x1 > startXViewPort + viewWidth then
        begin
          deltaX1 := x1 - (startXViewPort + viewWidth);
          x1 := startXViewPort + viewWidth;
        end;
    end;
{$ifdef logging}
  LogLn('deltax: '+strf(deltax)+', deltax1: '+strf(deltax1)+',deltay: '+strf(deltay));
{$endif logging}
  case bitBlt of
  end;
  oldCurrentColor := currentColor;
  oldCurrentWriteMode := currentWriteMode;
  currentWriteMode := bitBlt;
  for j:=Y to Y1 do
   Begin
     inc(k,deltaX);
     for i:=X to X1 do
      begin
        currentColor := pt(bitmap)[k];
        directPutPixel(i,j);
        inc(k);
     end;
     inc(k,deltaX1);
   end;
  currentWriteMode := oldCurrentWriteMode;
  currentColor := oldCurrentColor;
end;
}
procedure SetRGBPaletteWin32GUI(colorNum,redValue,greenvalue,
      bluevalue : smallint);

  begin
     if directcolor or (colornum<0) or (colornum>=maxcolor) then
       begin
         _graphresult:=grerror;
         exit;
       end;
     pal[colorNum].red:=redValue;
     pal[colorNum].green:=greenValue;
     pal[colorNum].blue:=blueValue;
     if (colorNum>=0) and (colorNum<=high(pens)) and (pens[colorNum]<>0) then
       begin
          DeleteObject(pens[colorNum]);
          pens[colorNum]:=0;
       end;
  end;

procedure GetRGBPaletteWin32GUI(colorNum : smallint;
      var redValue,greenvalue,bluevalue : smallint);

  begin
     if directcolor or (colornum<0) or (colornum>=maxcolor) then
       begin
         _graphresult:=grerror;
         exit;
       end;
     redValue:=pal[colorNum].red;
     greenValue:=pal[colorNum].green;
     blueValue:=pal[colorNum].blue;
  end;

procedure savestate;

  begin
  end;


procedure restorestate;

  begin
  end;

function WindowProcGraph(Window: HWnd; AMessage:UInt; WParam : WParam;
                    LParam: LParam): Longint; stdcall;

  var
     dc : hdc;
     ps : paintstruct;
     r : rect;
     oldbrush : hbrush;
     oldpen : hpen;
     i : longint;

begin
  WindowProcGraph := 0;

  case AMessage of
    wm_lbuttondown,
    wm_rbuttondown,
    wm_mbuttondown,
    wm_lbuttonup,
    wm_rbuttonup,
    wm_mbuttonup,
    wm_lbuttondblclk,
    wm_rbuttondblclk,
    wm_mbuttondblclk:
    {
    This leads to problem, i.e. the menu etc doesn't work any longer
    wm_nclbuttondown,
    wm_ncrbuttondown,
    wm_ncmbuttondown,
    wm_nclbuttonup,
    wm_ncrbuttonup,
    wm_ncmbuttonup,
    wm_nclbuttondblclk,
    wm_ncrbuttondblclk,
    wm_ncmbuttondblclk:
    }
      begin
         if assigned(mousemessagehandler) then
           WindowProcGraph:=mousemessagehandler(window,amessage,wparam,lparam);
      end;
    wm_notify:
      begin
         if assigned(notifymessagehandler) then
           WindowProcGraph:=notifymessagehandler(window,amessage,wparam,lparam);
      end;
    wm_command:
      if assigned(commandmessagehandler) then
        WindowProcGraph:=commandmessagehandler(window,amessage,wparam,lparam);
    wm_keydown,
    wm_keyup,
    wm_char:
      begin
         if assigned(charmessagehandler) then
           WindowProcGraph:=charmessagehandler(window,amessage,wparam,lparam);
      end;
    wm_paint:
      begin
{$ifdef DEBUG_WM_PAINT}
         inc(wm_paint_count);
{$endif DEBUG_WM_PAINT}
{$ifdef DEBUGCHILDS}
         writeln('Start child painting');
{$endif DEBUGCHILDS}
         if not GetUpdateRect(Window,@r,false) then
           exit;
         EnterCriticalSection(graphdrawing);
         graphrunning:=true;
         dc:=BeginPaint(Window,@ps);
{$ifdef DEBUG_WM_PAINT}
         Writeln(graphdebug,'WM_PAINT in ((',r.left,',',r.top,
           '),(',r.right,',',r.bottom,'))');
{$endif def DEBUG_WM_PAINT}
         if graphrunning then
           {BitBlt(dc,0,0,maxx+1,maxy+1,bitmapdc,0,0,SRCCOPY);}
           BitBlt(dc,r.left,r.top,r.right-r.left+1,r.bottom-r.top+1,bitmapdc,r.left,r.top,SRCCOPY);
         EndPaint(Window,ps);
         LeaveCriticalSection(graphdrawing);
         Exit;
      end;
    wm_create:
      begin
{$ifdef DEBUG_WM_PAINT}
         assign(graphdebug,'wingraph.log');
         rewrite(graphdebug);
{$endif DEBUG_WM_PAINT}
{$ifdef DEBUGCHILDS}
         writeln('Creating window (HWND: ',window,')... ');
{$endif DEBUGCHILDS}
         GraphWindow:=window;
         EnterCriticalSection(graphdrawing);
         dc:=GetDC(window);
{$ifdef DEBUGCHILDS}
         writeln('Window DC: ',dc);
{$endif DEBUGCHILDS}
         bitmapdc:=CreateCompatibleDC(dc);
         savedscreen:=CreateCompatibleBitmap(dc,maxx+1,maxy+1);
         ReleaseDC(window,dc);
         oldbitmap:=SelectObject(bitmapdc,savedscreen);
         windc:=GetDC(window);
         // clear everything
         oldpen:=SelectObject(bitmapdc,GetStockObject(BLACK_PEN));
         oldbrush:=SelectObject(bitmapdc,GetStockObject(BLACK_BRUSH));
         Windows.Rectangle(bitmapdc,0,0,maxx,maxy);
         SelectObject(bitmapdc,oldpen);
         SelectObject(bitmapdc,oldbrush);
         // ... the window too
         oldpen:=SelectObject(windc,GetStockObject(BLACK_PEN));
         oldbrush:=SelectObject(windc,GetStockObject(BLACK_BRUSH));
         Windows.Rectangle(windc,0,0,maxx,maxy);
         SelectObject(windc,oldpen);
         SelectObject(windc,oldbrush);
         // clear font cache
         fillchar(bitmapfonthorizoncache,sizeof(bitmapfonthorizoncache),0);
         fillchar(bitmapfontverticalcache,sizeof(bitmapfontverticalcache),0);

         // clear predefined pens
         fillchar(pens,sizeof(pens),0);
         if assigned(OnGraphWindowCreation) then
           OnGraphWindowCreation;
         LeaveCriticalSection(graphdrawing);
{$ifdef DEBUGCHILDS}
         writeln('done');
         GetClientRect(window,@r);
         writeln('Window size: ',r.right,',',r.bottom);
{$endif DEBUGCHILDS}
      end;
    wm_Destroy:
      begin
         EnterCriticalSection(graphdrawing);
         graphrunning:=false;
         ReleaseDC(GraphWindow,windc);
         SelectObject(bitmapdc,oldbitmap);
         DeleteObject(savedscreen);
         DeleteDC(bitmapdc);
         // release font cache
         for i:=0 to 255 do
           if bitmapfonthorizoncache[i]<>0 then
             DeleteObject(bitmapfonthorizoncache[i]);
         for i:=0 to 255 do
           if bitmapfontverticalcache[i]<>0 then
             DeleteObject(bitmapfontverticalcache[i]);

         for i:=0 to high(pens) do
           if pens[i]<>0 then
             DeleteObject(pens[i]);

         LeaveCriticalSection(graphdrawing);
{$ifdef DEBUG_WM_PAINT}
         close(graphdebug);
{$endif DEBUG_WM_PAINT}
         PostQuitMessage(0);
         Exit;
      end
    else
<<<<<<< HEAD
<<<<<<< HEAD
      WindowProcGraph := DefWindowProcA(Window, AMessage, WParam, LParam);
=======
      WindowProcGraph := DefWindowProc(Window, AMessage, WParam, LParam);
>>>>>>> graemeg/fixes_2_2
=======
      WindowProcGraph := DefWindowProc(Window, AMessage, WParam, LParam);
>>>>>>> origin/fixes_2_2
  end;
end;

function WindowProcParent(Window: HWnd; AMessage:UInt; WParam : WParam;
                    LParam: LParam): Longint; stdcall;

begin
  WindowProcParent := 0;
  case AMessage of
    wm_keydown,
    wm_keyup,
    wm_char:
      begin
         if assigned(charmessagehandler) then
           WindowProcParent:=charmessagehandler(window,amessage,wparam,lparam);
      end;
    wm_notify:
      begin
         if assigned(notifymessagehandler) then
           WindowProcParent:=notifymessagehandler(window,amessage,wparam,lparam);
      end;
    wm_command:
      if assigned(commandmessagehandler) then
        WindowProcParent:=commandmessagehandler(window,amessage,wparam,lparam);
    else
<<<<<<< HEAD
<<<<<<< HEAD
      WindowProcParent := DefWindowProcA(Window, AMessage, WParam, LParam);
=======
      WindowProcParent := DefWindowProc(Window, AMessage, WParam, LParam);
>>>>>>> graemeg/fixes_2_2
=======
      WindowProcParent := DefWindowProc(Window, AMessage, WParam, LParam);
>>>>>>> origin/fixes_2_2
  end;
end;

function WinRegister: Boolean;
var
  WindowClass: WndClass;
begin
  WindowClass.Style := graphwindowstyle;
  WindowClass.lpfnWndProc := WndProc(@WindowProcGraph);
  WindowClass.cbClsExtra := 0;
  WindowClass.cbWndExtra := 0;
  WindowClass.hInstance := system.MainInstance;
  if icon<>0 then
    WindowClass.hIcon := icon
  else
    WindowClass.hIcon := LoadIcon(0, idi_Application);
  WindowClass.hCursor := LoadCursor(0, idc_Arrow);
  WindowClass.hbrBackground := GetStockObject(BLACK_BRUSH);
  if menu<>0 then
    WindowClass.lpszMenuName := MAKEINTRESOURCE(menu)
  else
    WindowClass.lpszMenuName := nil;
  WindowClass.lpszClassName := 'FPCGraphWindow';

  winregister:=RegisterClass(WindowClass) <> 0;
end;

function WinRegisterWithChild: Boolean;
var
  WindowClass: WndClass;
begin
  WindowClass.Style := graphwindowstyle;
  WindowClass.lpfnWndProc := WndProc(@WindowProcParent);
  WindowClass.cbClsExtra := 0;
  WindowClass.cbWndExtra := 0;
  WindowClass.hInstance := system.MainInstance;
  if icon<>0 then
    WindowClass.hIcon := icon
  else
    WindowClass.hIcon := LoadIcon(0, idi_Application);
  WindowClass.hCursor := LoadCursor(0, idc_Arrow);
  WindowClass.hbrBackground := GetStockObject(BLACK_BRUSH);
  if menu<>0 then
    WindowClass.lpszMenuName := MAKEINTRESOURCE(menu)
  else
    WindowClass.lpszMenuName := nil;
  WindowClass.lpszClassName := 'FPCGraphWindowMain';

  WinRegisterWithChild:=RegisterClass(WindowClass) <> 0;
{$ifdef DEBUGCHILDS}
  writeln('Main window successfully registered: WinRegisterWithChild is ',WinRegisterWithChild);
{$endif DEBUGCHILDS}
  if WinRegisterWithChild then
    begin
       WindowClass.Style := CS_HREDRAW or CS_VREDRAW;
       WindowClass.lpfnWndProc := WndProc(@WindowProcGraph);
       WindowClass.cbClsExtra := 0;
       WindowClass.cbWndExtra := 0;
       WindowClass.hInstance := system.MainInstance;
       WindowClass.hIcon := 0;
       WindowClass.hCursor := LoadCursor(0, idc_Arrow);
       WindowClass.hbrBackground := GetStockObject(BLACK_BRUSH);
       WindowClass.lpszMenuName := nil;
       WindowClass.lpszClassName := 'FPCGraphWindowChild';
       WinRegisterWithChild:=RegisterClass(WindowClass)<>0;
{$ifdef DEBUGCHILDS}
       writeln('Child window registered: WinRegisterWithChild is ',WinRegisterWithChild);
{$endif DEBUGCHILDS}
    end;
end;

var
   // here we can force the creation of a maximized window }
   extrastyle : cardinal;

 { Create the Window Class }
function WinCreate : HWnd;
var
  hWindow: HWnd;
begin
  WinCreate:=0;
  if UseChildWindow then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
       ParentWindow:=CreateWindowA('FPCGraphWindowMain', windowtitle,
=======
       ParentWindow:=CreateWindow('FPCGraphWindowMain', windowtitle,
>>>>>>> graemeg/fixes_2_2
=======
       ParentWindow:=CreateWindow('FPCGraphWindowMain', windowtitle,
>>>>>>> origin/fixes_2_2
                  WS_OVERLAPPEDWINDOW or WS_CLIPCHILDREN or extrastyle, longint(CW_USEDEFAULT), 0,
                  maxx+ChildOffset.Left+ChildOffset.Right+1+
                    2*GetSystemMetrics(SM_CXFRAME),
                  maxy+ChildOffset.Top+ChildOffset.Bottom+1+
                    2*GetSystemMetrics(SM_CYFRAME)+
                  GetSystemMetrics(SM_CYCAPTION),
                  0, 0, system.MainInstance, nil);
       if ParentWindow<>0 then
         begin
            ShowWindow(ParentWindow, SW_SHOW);
            UpdateWindow(ParentWindow);
         end
       else
         exit;
<<<<<<< HEAD
<<<<<<< HEAD
       hWindow:=CreateWindowA('FPCGraphWindowChild',nil,
=======
       hWindow:=CreateWindow('FPCGraphWindowChild',nil,
>>>>>>> graemeg/fixes_2_2
=======
       hWindow:=CreateWindow('FPCGraphWindowChild',nil,
>>>>>>> origin/fixes_2_2
                  WS_CHILD, ChildOffset.Left,ChildOffset.Top,
                  maxx+1,maxy+1,
                  ParentWindow, 0, system.MainInstance, nil);
       if hwindow<>0 then
         begin
            ShowWindow(hwindow, SW_SHOW);
            UpdateWindow(hwindow);
         end
       else
         exit;
       WinCreate:=hWindow;
    end
  else
    begin
<<<<<<< HEAD
<<<<<<< HEAD
       hWindow:=CreateWindowA('FPCGraphWindow', windowtitle,
=======
       hWindow:=CreateWindow('FPCGraphWindow', windowtitle,
>>>>>>> graemeg/fixes_2_2
=======
       hWindow:=CreateWindow('FPCGraphWindow', windowtitle,
>>>>>>> origin/fixes_2_2
                  ws_OverlappedWindow or extrastyle, longint(CW_USEDEFAULT), 0,
                  maxx+1+2*GetSystemMetrics(SM_CXFRAME),
                  maxy+1+2*GetSystemMetrics(SM_CYFRAME)+
                  GetSystemMetrics(SM_CYCAPTION),
                  0, 0, system.MainInstance, nil);
       if hWindow <> 0 then
         begin
            ShowWindow(hWindow, SW_SHOW);
            UpdateWindow(hWindow);
            WinCreate:=hWindow;
         end;
    end;
end;

const
   winregistered : boolean = false;

<<<<<<< HEAD
<<<<<<< HEAD
   { Thread functions have different return type and calling convention
     for system unit funcitons andfor windows API. }
{$ifdef USE_SYSTEM_BEGIN_THREAD}
function MessageHandleThread(p : pointer) : ptrint;
{$else not USE_SYSTEM_BEGIN_THREAD}
function MessageHandleThread(p : pointer) : DWord; stdcall;
{$endif not USE_SYSTEM_BEGIN_THREAD}
=======
function MessageHandleThread(p : pointer) : DWord;StdCall;
>>>>>>> graemeg/fixes_2_2
=======
function MessageHandleThread(p : pointer) : DWord;StdCall;
>>>>>>> origin/fixes_2_2

  var
     AMessage: Msg;

  begin
     if not(winregistered) then
       begin
          if UseChildWindow then
            begin
               if not(WinRegisterWithChild) then
                 begin
                    MessageBox(0, 'Window registration failed', nil, mb_Ok);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef USE_SYSTEM_BEGIN_THREAD}
                    System.EndThread(1);
{$else not USE_SYSTEM_BEGIN_THREAD}
                    Windows.ExitThread(1);
{$endif not USE_SYSTEM_BEGIN_THREAD}
=======
                    ExitThread(1);
>>>>>>> graemeg/fixes_2_2
=======
                    ExitThread(1);
>>>>>>> origin/fixes_2_2
                 end;
            end
          else
            begin
               if not(WinRegister) then
                 begin
                    MessageBox(0, 'Window registration failed', nil, mb_Ok);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef USE_SYSTEM_BEGIN_THREAD}
                    System.EndThread(1);
{$else not USE_SYSTEM_BEGIN_THREAD}
                    Windows.ExitThread(1);
{$endif not USE_SYSTEM_BEGIN_THREAD}
=======
                    ExitThread(1);
>>>>>>> graemeg/fixes_2_2
=======
                    ExitThread(1);
>>>>>>> origin/fixes_2_2
                 end;
            end;
          winregistered:=true;
       end;
     GraphWindow:=WinCreate;
     if longint(GraphWindow) = 0 then begin
       MessageBox(0, 'Window creation failed', nil, mb_Ok);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef USE_SYSTEM_BEGIN_THREAD}
       System.EndThread(1);
{$else not USE_SYSTEM_BEGIN_THREAD}
       Windows.ExitThread(1);
{$endif not USE_SYSTEM_BEGIN_THREAD}
=======
       ExitThread(1);
>>>>>>> graemeg/fixes_2_2
=======
       ExitThread(1);
>>>>>>> origin/fixes_2_2
     end;
     while longint(GetMessage(@AMessage, 0, 0, 0))=longint(true) do
       begin
          TranslateMessage(AMessage);
          DispatchMessage(AMessage);
       end;
     MessageHandleThread:=0;
  end;

procedure InitWin32GUI16colors;

  var
     threadexitcode : longint;
  begin
     getmem(pal,sizeof(RGBrec)*maxcolor);
     move(DefaultColors,pal^,sizeof(RGBrec)*maxcolor);
     if (IntCurrentMode=mMaximizedWindow16) or
       (IntCurrentMode=mMaximizedWindow256) or
       (IntCurrentMode=mMaximizedWindow32k) or
       (IntCurrentMode=mMaximizedWindow64k) or
       (IntCurrentMode=mMaximizedWindow16M) then
       extrastyle:=ws_maximize
     else
       extrastyle:=0;
     { start graph subsystem }
     InitializeCriticalSection(graphdrawing);
     graphrunning:=false;
<<<<<<< HEAD
<<<<<<< HEAD
     {Use system BeginThread instead of CreteThreead
     function BeginThread(sa : Pointer;stacksize : SizeUInt;
  ThreadFunction : tthreadfunc;p : pointer;creationFlags : dword;
  var ThreadId : TThreadID) : TThreadID;}
{$ifdef USE_SYSTEM_BEGIN_THREAD}
     MessageThreadHandle:=System.BeginThread(nil,0,@MessageHandleThread,
       nil,0,MessageThreadID);
{$else not USE_SYSTEM_BEGIN_THREAD}
     MessageThreadHandle:=CreateThread(nil,0,@MessageHandleThread,
       nil,0,MessageThreadID);
{$endif not USE_SYSTEM_BEGIN_THREAD}
=======
     MessageThreadHandle:=CreateThread(nil,0,@MessageHandleThread,
       nil,0,MessageThreadID);
>>>>>>> graemeg/fixes_2_2
=======
     MessageThreadHandle:=CreateThread(nil,0,@MessageHandleThread,
       nil,0,MessageThreadID);
>>>>>>> origin/fixes_2_2
     repeat
       GetExitCodeThread(MessageThreadHandle,@threadexitcode);
     until graphrunning or (threadexitcode<>STILL_ACTIVE);
     if threadexitcode<>STILL_ACTIVE then
        _graphresult := grerror;
  end;

procedure CloseGraph;

  begin
     If not isgraphmode then
       begin
         _graphresult := grnoinitgraph;
         exit
       end;
     if UseChildWindow then
       begin
          { if the child window isn't destroyed }
          { the main window can't be closed     }
          { I don't know any other way (FK)     }
          PostMessage(GraphWindow,wm_destroy,0,0);
          PostMessage(ParentWindow,wm_destroy,0,0)
       end
     else
       PostMessage(GraphWindow,wm_destroy,0,0);

     PostThreadMessage(MessageThreadHandle,wm_quit,0,0);
     WaitForSingleObject(MessageThreadHandle,Infinite);
     CloseHandle(MessageThreadHandle);
     DeleteCriticalSection(graphdrawing);
     freemem(pal,sizeof(RGBrec)*maxcolor);

     MessageThreadID := 0;
     MessageThreadHandle := 0;
     isgraphmode := false;
  end;

procedure LineWin32GUI(X1, Y1, X2, Y2: smallint); {$ifndef fpc}far;{$endif fpc}

  var X, Y :           smallint;
      deltax, deltay : smallint;
      d, dinc1, dinc2: smallint;
      xinc1          : smallint;
      xinc2          : smallint;
      yinc1          : smallint;
      yinc2          : smallint;
      i              : smallint;
      Flag           : Boolean; { determines pixel direction in thick lines }
      NumPixels      : smallint;
      PixelCount     : smallint;
      OldCurrentColor: Word;
      swtmp          : smallint;
      TmpNumPixels   : smallint;
      col : longint;
      pen,oldpen : hpen;

 begin
    if graphrunning then
      begin
         {******************************************}
         {  SOLID LINES                             }
         {******************************************}
         if lineinfo.LineStyle = SolidLn then
           Begin
              { Convert to global coordinates. }
              x1 := x1 + StartXViewPort;
              x2 := x2 + StartXViewPort;
              y1 := y1 + StartYViewPort;
              y2 := y2 + StartYViewPort;
              { if fully clipped then exit... }
              if ClipPixels then
                begin
                   if LineClipped(x1,y1,x2,y2,StartXViewPort, StartYViewPort,
                     StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
                       exit;
                  If LineInfo.Thickness=NormWidth then
                   Begin
                      EnterCriticalSection(graphdrawing);
                      {
                      if currentwritemode<>normalput then
                        begin
                           case currentwritemode of
                              XORPut:
                                begin
                                   SetROP2(windc,R2_XORPEN);
                                   SetROP2(bitmapdc,R2_XORPEN);
                                end;
                              AndPut:
                                begin
                                   SetROP2(windc,R2_MASKPEN);
                                   SetROP2(bitmapdc,R2_MASKPEN);
                                end;
                              OrPut:
                                begin
                                   SetROP2(windc,R2_MERGEPEN);
                                   SetROP2(bitmapdc,R2_MERGEPEN);
                                end;
                           end;
                        end;
                      }
                      col:=RGB(pal[CurrentColor].red,pal[CurrentColor].green,pal[CurrentColor].blue);
                      pen:=CreatePen(PS_SOLID,1,col);
                      if pen=0 then
                        writeln('Pen konnte nicht erzeugt werden!');

                      oldpen:=SelectObject(windc,pen);
                      MoveToEx(windc,x1,y1,nil);
                      Windows.LineTo(windc,x2,y2);
                      SetPixel(windc,x2,y2,col);
                      SelectObject(windc,oldpen);

                      oldpen:=SelectObject(bitmapdc,pen);
                      MoveToEx(bitmapdc,x1,y1,nil);
                      Windows.LineTo(bitmapdc,x2,y2);
                      SetPixel(bitmapdc,x2,y2,col);
                      SelectObject(bitmapdc,oldpen);

                      DeleteObject(pen);
                      {
                      if currentwritemode<>normalput then
                        begin
                           SetROP2(windc,R2_COPYPEN);
                           SetROP2(bitmapdc,R2_COPYPEN);
                        end;
                      }
                      LeaveCriticalSection(graphdrawing);
                   end
                 else
                  { Thick width lines }
                   begin
                     { Draw the pixels }
                      for i := 1 to numpixels do
                        begin
                         { all depending on the slope, we can determine         }
                         { in what direction the extra width pixels will be put }
                         If Flag then
                           Begin
                             DirectPutPixelClip(x-1,y);
                             DirectPutPixelClip(x,y);
                             DirectPutPixelClip(x+1,y);
                           end
                         else
                           Begin
                             DirectPutPixelClip(x, y-1);
                             DirectPutPixelClip(x, y);
                             DirectPutPixelClip(x, y+1);
                           end;
                         if d < 0 then
                           begin
                             d := d + dinc1;
                             x := x + xinc1;
                             y := y + yinc1;
                           end
                         else
                           begin
                             d := d + dinc2;
                             x := x + xinc2;
                             y := y + yinc2;
                           end;
                        end;
                   end;
                 end;
           end
          else
       {******************************************}
       {  begin patterned lines                   }
       {******************************************}
           Begin
             { Convert to global coordinates. }
             x1 := x1 + StartXViewPort;
             x2 := x2 + StartXViewPort;
             y1 := y1 + StartYViewPort;
             y2 := y2 + StartYViewPort;
             { if fully clipped then exit... }
             if ClipPixels then
              begin
              if LineClipped(x1,y1,x2,y2,StartXViewPort, StartYViewPort,
                  StartXViewPort+ViewWidth, StartYViewPort+ViewHeight) then
                     exit;
              end;

             OldCurrentColor := CurrentColor;
             PixelCount:=0;
             if y1 = y2 then
                   Begin
                    { Check if we must swap }
                if x1 >= x2 then
                      Begin
                        swtmp := x1;
                        x1 := x2;
                        x2 := swtmp;
                      end;
                if LineInfo.Thickness = NormWidth then
                     Begin
                      for PixelCount:=x1 to x2 do
                            { optimization: PixelCount mod 16 }
                            if LinePatterns[PixelCount and 15] = TRUE then
                             begin
                               DirectPutPixel(PixelCount,y2);
                             end;
                     end
                    else
                     Begin
                      for i:=-1 to 1 do
                            Begin
                              for PixelCount:=x1 to x2 do
                                { Optimization from Thomas - mod 16 = and 15 }
                                {this optimization has been performed by the compiler
                                 for while as well (JM)}
                                if LinePatterns[PixelCount and 15] = TRUE then
                                  begin
                                        DirectPutPixelClip(PixelCount,y2+i);
                                  end;
                            end;
                     end;
               end
             else
             if x1 = x2 then
                  Begin
                   { Check if we must swap }
                   if y1 >= y2 then
                     Begin
                       swtmp := y1;
                       y1 := y2;
                       y2 := swtmp;
                     end;
                   if LineInfo.Thickness = NormWidth then
                     Begin
                       for PixelCount:=y1 to y2 do
                           { compare if we should plot a pixel here , compare }
                           { with predefined line patterns...                 }
                           if LinePatterns[PixelCount and 15] = TRUE then
                             begin
                           DirectPutPixel(x1,PixelCount);
                             end;
                     end
                   else
                     Begin
                       for i:=-1 to 1 do
                            Begin
                              for PixelCount:=y1 to y2 do
                              { compare if we should plot a pixel here , compare }
                              { with predefined line patterns...                 }
                                if LinePatterns[PixelCount and 15] = TRUE then
                                  begin
                                    DirectPutPixelClip(x1+i,PixelCount);
                                  end;
                            end;
                     end;
                  end
             else
                  Begin
                    oldCurrentColor := CurrentColor;
                    { Calculate deltax and deltay for initialisation }
                    deltax := abs(x2 - x1);
                    deltay := abs(y2 - y1);

                    { Initialize all vars based on which is the independent variable }
                    if deltax >= deltay then
                      begin

                        Flag := FALSE;
                        { x is independent variable }
                        numpixels := deltax + 1;
                        d := (2 * deltay) - deltax;
                        dinc1 := deltay Shl 1;
                        dinc2 := (deltay - deltax) shl 1;
                        xinc1 := 1;
                        xinc2 := 1;
                        yinc1 := 0;
                        yinc2 := 1;
                     end
                   else
                     begin

                       Flag := TRUE;
                       { y is independent variable }
                       numpixels := deltay + 1;
                       d := (2 * deltax) - deltay;
                       dinc1 := deltax Shl 1;
                       dinc2 := (deltax - deltay) shl 1;
                       xinc1 := 0;
                       xinc2 := 1;
                       yinc1 := 1;
                       yinc2 := 1;
                     end;

                   { Make sure x and y move in the right directions }
                   if x1 > x2 then
                     begin
                       xinc1 := - xinc1;
                       xinc2 := - xinc2;
                     end;
                   if y1 > y2 then
                     begin
                       yinc1 := - yinc1;
                       yinc2 := - yinc2;
                     end;

                   { Start drawing at <x1, y1> }
                   x := x1;
                   y := y1;

                   If LineInfo.Thickness=ThickWidth then

                    Begin
                      TmpNumPixels := NumPixels-1;
                      { Draw the pixels }
                      for i := 0 to TmpNumPixels do
                        begin
                            { all depending on the slope, we can determine         }
                            { in what direction the extra width pixels will be put }
                              If Flag then
                                 Begin
                                   { compare if we should plot a pixel here , compare }
                                   { with predefined line patterns...                 }
                                   if LinePatterns[i and 15] = TRUE then
                                     begin
                                       DirectPutPixelClip(x-1,y);
                                       DirectPutPixelClip(x,y);
                                       DirectPutPixelClip(x+1,y);
                                     end;
                                 end
                              else
                                 Begin
                                   { compare if we should plot a pixel here , compare }
                                   { with predefined line patterns...                 }
                                   if LinePatterns[i and 15] = TRUE then
                                    begin
                                      DirectPutPixelClip(x,y-1);
                                      DirectPutPixelClip(x,y);
                                      DirectPutPixelClip(x,y+1);
                                    end;
                                 end;
                          if d < 0 then
                                begin
                                  d := d + dinc1;
                                  x := x + xinc1;
                                  y := y + yinc1;
                                end
                          else
                                begin
                          d := d + dinc2;
                          x := x + xinc2;
                          y := y + yinc2;
                                end;
                       end;
                   end
                  else
                   Begin
                    { instead of putting in loop , substract by one now }
                    TmpNumPixels := NumPixels-1;
                   { NormWidth }
                    for i := 0 to TmpNumPixels do
                    begin
                         if LinePatterns[i and 15] = TRUE then
                           begin
                                 DirectPutPixel(x,y);
                           end;
                    if d < 0 then
                        begin
                          d := d + dinc1;
                          x := x + xinc1;
                          y := y + yinc1;
                        end
                    else
                        begin
                          d := d + dinc2;
                          x := x + xinc2;
                          y := y + yinc2;
                        end;
                    end;
                   end
               end;
       {******************************************}
       {  end patterned lines                     }
       {******************************************}
              { restore color }
              CurrentColor:=OldCurrentColor;
          end;
    end;
 end;  { Line }

{ multipage support could be done by using more than one background bitmap }
procedure SetVisualWin32GUI(page: word);

  begin
  end;

procedure SetActiveWin32GUI(page: word);
  begin
  end;

function queryadapterinfo : pmodeinfo;

  var
     mode: TModeInfo;
     ScreenWidth,ScreenHeight : longint;
     ScreenWidthMaximized,ScreenHeightMaximized : longint;

  procedure SetupWin32GUIDefault;

    begin
       mode.DirectPutPixel:={$ifdef fpc}@{$endif}DirectPutPixel16Win32GUI;
       mode.PutPixel:={$ifdef fpc}@{$endif}PutPixel16Win32GUI;
       mode.GetPixel:={$ifdef fpc}@{$endif}GetPixel16Win32GUI;
       mode.HLine := {$ifdef fpc}@{$endif}HLine16Win32GUI;
       mode.SetRGBPalette := {$ifdef fpc}@{$endif}SetRGBPaletteWin32GUI;
       mode.GetRGBPalette := {$ifdef fpc}@{$endif}GetRGBPaletteWin32GUI;
       mode.SetVisualPage := {$ifdef fpc}@{$endif}SetVisualWin32GUI;
       mode.SetActivePage := {$ifdef fpc}@{$endif}SetActiveWin32GUI;
       mode.InitMode := {$ifdef fpc}@{$endif}InitWin32GUI16colors;
       mode.OuttextXY:={$ifdef fpc}@{$endif}OuttextXYWin32GUI;
       mode.VLine := {$ifdef fpc}@{$endif}VLine16Win32GUI;
       // mode.circle := {$ifdef fpc}@{$endif}Circle16Win32GUI;
       // doesn't work yet
       // mode.Line:={$ifdef fpc}@{$endif}LineWin32GUI;
    end;

  begin
     SaveVideoState:={$ifdef fpc}@{$endif}savestate;
     RestoreVideoState:={$ifdef fpc}@{$endif}restorestate;
     { we must take care of the border and caption }
     ScreenWidth:=GetSystemMetrics(SM_CXSCREEN)-
       2*GetSystemMetrics(SM_CXFRAME);
     ScreenHeight:=GetSystemMetrics(SM_CYSCREEN)-
       2*GetSystemMetrics(SM_CYFRAME)-
       GetSystemMetrics(SM_CYCAPTION);
     { for maximozed windows it's again different }
     { here we've only a caption }
     ScreenWidthMaximized:=GetSystemMetrics(SM_CXFULLSCREEN);
     { neither GetSystemMetrics(SM_CYFULLSCREEN nor     }
     { SystemParametersInfo(SPI_GETWORKAREA)            }
     { takes a hidden try into account :( FK            }
     ScreenHeightMaximized:=GetSystemMetrics(SM_CYFULLSCREEN);

     QueryAdapterInfo := ModeList;
     { If the mode listing already exists... }
     { simply return it, without changing    }
     { anything...                           }
     if assigned(ModeList) then
       exit;
     { the first one becomes the standard mode }
     if (ScreenWidth>=640) and (ScreenHeight>=480) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VGA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=VGAHi;
          mode.ModeName:='640 x 480 x 16 Win32GUI';
          mode.MaxColor := 16;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 639;
          mode.MaxY := 479;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     if (ScreenWidth>=640) and (ScreenHeight>=200) then
       begin
          InitMode(mode);
          { now add all standard VGA modes...       }
          mode.DriverNumber:= VGA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=VGALo;
          mode.ModeName:='640 x 200 x 16 Win32GUI';
          mode.MaxColor := 16;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 639;
          mode.MaxY := 199;
          SetupWin32GUIDefault;
          mode.XAspect := 4500;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     if (ScreenWidth>=640) and (ScreenHeight>=350) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VGA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=VGAMed;
          mode.ModeName:='640 x 350 x 16 Win32GUI';
          mode.MaxColor := 16;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 639;
          mode.MaxY := 349;
          SetupWin32GUIDefault;
          mode.XAspect := 7750;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     if (ScreenWidth>=640) and (ScreenHeight>=400) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m640x400x256;
          mode.ModeName:='640 x 400 x 256 Win32GUI';
          mode.MaxColor := 256;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 639;
          mode.MaxY := 399;
          SetupWin32GUIDefault;
          mode.XAspect := 8333;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     if (ScreenWidth>=640) and (ScreenHeight>=480) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m640x480x256;
          mode.ModeName:='640 x 480 x 256 Win32GUI';
          mode.MaxColor := 256;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 639;
          mode.MaxY := 479;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     { add 800x600 only if screen is large enough }
     If (ScreenWidth>=800) and (ScreenHeight>=600) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m800x600x16;
          mode.ModeName:='800 x 600 x 16 Win32GUI';
          mode.MaxColor := 16;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 799;
          mode.MaxY := 599;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m800x600x256;
          mode.ModeName:='800 x 600 x 256 Win32GUI';
          mode.MaxColor := 256;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 799;
          mode.MaxY := 599;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     { add 1024x768 only if screen is large enough }
     If (ScreenWidth>=1024) and (ScreenHeight>=768) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m1024x768x16;
          mode.ModeName:='1024 x 768 x 16 Win32GUI';
          mode.MaxColor := 16;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 1023;
          mode.MaxY := 767;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m1024x768x256;
          mode.ModeName:='1024 x 768 x 256 Win32GUI';
          mode.MaxColor := 256;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 1023;
          mode.MaxY := 768;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     { add 1280x1024 only if screen is large enough }
     If (ScreenWidth>=1280) and (ScreenHeight>=1024) then
       begin
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m1280x1024x16;
          mode.ModeName:='1280 x 1024 x 16 Win32GUI';
          mode.MaxColor := 16;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 1279;
          mode.MaxY := 1023;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
          InitMode(mode);
          mode.DriverNumber:= VESA;
          mode.HardwarePages:= 0;
          mode.ModeNumber:=m1280x1024x256;
          mode.ModeName:='1280 x 1024 x 256 Win32GUI';
          mode.MaxColor := 256;
          mode.PaletteSize := mode.MaxColor;
          mode.DirectColor := FALSE;
          mode.MaxX := 1279;
          mode.MaxY := 1023;
          SetupWin32GUIDefault;
          mode.XAspect := 10000;
          mode.YAspect := 10000;
          AddMode(mode);
       end;
     { at least we add a mode with the largest possible window }
      InitMode(mode);
      mode.DriverNumber:= VESA;
      mode.HardwarePages:= 0;
      mode.ModeNumber:=mLargestWindow16;
      mode.ModeName:='Largest Window x 16';
      mode.MaxColor := 16;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectColor := FALSE;
      mode.MaxX := ScreenWidth-1;
      mode.MaxY := ScreenHeight-1;
      SetupWin32GUIDefault;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
      AddMode(mode);
      InitMode(mode);
      mode.DriverNumber:= VESA;
      mode.HardwarePages:= 0;
      mode.ModeNumber:=mLargestWindow256;
      mode.ModeName:='Largest Window x 256';
      mode.MaxColor := 256;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectColor := FALSE;
      mode.MaxX := ScreenWidth-1;
      mode.MaxY := ScreenHeight-1;
      SetupWin32GUIDefault;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
      AddMode(mode);
      { .. and a maximized window }
      InitMode(mode);
      mode.DriverNumber:= VESA;
      mode.HardwarePages:= 0;
      mode.ModeNumber:=mMaximizedWindow16;
      mode.ModeName:='Maximized Window x 16';
      mode.MaxColor := 16;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectColor := FALSE;
      mode.MaxX := ScreenWidthMaximized-1;
      mode.MaxY := ScreenHeightMaximized-1;
      SetupWin32GUIDefault;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
      AddMode(mode);
      InitMode(mode);
      mode.DriverNumber:= VESA;
      mode.HardwarePages:= 0;
      mode.ModeNumber:=mMaximizedWindow256;
      mode.ModeName:='Maximized Window x 256';
      mode.MaxColor := 256;
      mode.PaletteSize := mode.MaxColor;
      mode.DirectColor := FALSE;
      mode.MaxX := ScreenWidthMaximized-1;
      mode.MaxY := ScreenHeightMaximized-1;
      SetupWin32GUIDefault;
      mode.XAspect := 10000;
      mode.YAspect := 10000;
      AddMode(mode);
  end;

begin
  InitializeGraph;
  charmessagehandler:=nil;
  mousemessagehandler:=nil;
  commandmessagehandler:=nil;
  notifymessagehandler:=nil;
  OnGraphWindowCreation:=nil;
end.
