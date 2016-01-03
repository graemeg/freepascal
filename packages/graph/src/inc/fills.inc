{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Thomas Schatzl and Carl Eric Codere

    This include implements polygon filling and flood filling.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ simple descriptive name }
function max(a, b : Longint) : Longint;
begin
  max := b;
  if (a > b) then max := a;
end;

{ here too }
function min(a, b : Longint) : Longint;
begin
  min := b;
  if (a < b) then min := a;
end;

procedure fillpoly(numpoints : Word; var polypoints);

{ disable range check mode }
<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$R-}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifopt R+}
{$define OPT_R_WAS_ON}
{$R-}
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
  pedge = ^tedge;
  tedge = packed record
    yMin, yMax, x, dX, dY, frac : Longint;
  end;

  pedgearray = ^tedgearray;
  tedgearray = array[0..0] of tedge;

  ppedgearray = ^tpedgearray;
  tpedgearray = array[0..0] of pedge;

var
  nActive, nNextEdge : Longint;
  p0, p1 : pointtype;
  i, j, gap, x0, x1, y, nEdges : Longint;
  ET : pedgearray;
  GET, AET : ppedgearray;
  t : pedge;

  ptable : ^pointtype;

<<<<<<< HEAD
<<<<<<< HEAD
  LastPolygonStart : Longint;
  Closing, PrevClosing : Boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
{ /********************************************************************
  * Add entries to the global edge table.  The global edge table has a
  * bucket for each scan line in the polygon. Each bucket contains all
  * the edges whose yMin == yScanline.  Each bucket contains the yMax,
  * the x coordinate at yMax, and the denominator of the slope (dX)
*/}
  getmem(et, sizeof(tedge) * numpoints);
  getmem(get, sizeof(pedge) * numpoints);
  getmem(aet, sizeof(pedge) * numpoints);

  ptable := @polypoints;

 { check for getmem success }

  nEdges := 0;
<<<<<<< HEAD
<<<<<<< HEAD
  LastPolygonStart := 0;
  Closing := false;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  for i := 0 to (numpoints-1) do begin
    p0 := ptable[i];
    if (i+1) >= numpoints then p1 := ptable[0]
    else p1 := ptable[i+1];
<<<<<<< HEAD
<<<<<<< HEAD
    { save the 'closing' flag for the previous edge }
    PrevClosing := Closing;
    { check if the current edge is 'closing'. This means that it 'closes'
      the polygon by going back to the first point of the polygon.
      Also, 0-length edges are never considered 'closing'. }
    if ((p1.x <> ptable[i].x) or
        (p1.y <> ptable[i].y)) and
        (LastPolygonStart < i) and
       ((p1.x = ptable[LastPolygonStart].x) and
        (p1.y = ptable[LastPolygonStart].y)) then
    begin
      Closing := true;
      LastPolygonStart := i + 2;
    end
    else
      Closing := false;
    { skip current edge if the previous edge was 'closing'. This is TP7 compatible }
    if PrevClosing then
      continue;
   { draw the edges }
{    nickysn: moved after drawing the filled area
    Line(p0.x,p0.y,p1.x,p1.y);}
   { ignore if this is a horizontal edge}
    if (p0.y = p1.y) then continue;
   { swap ptable if necessary to ensure p0 contains yMin}
=======
   { ignore if this is a horizontal edge}
    if (p0.y = p1.y) then continue;
    {swap ptable if necessary to ensure p0 contains yMin}
>>>>>>> graemeg/fixes_2_2
=======
   { ignore if this is a horizontal edge}
    if (p0.y = p1.y) then continue;
    {swap ptable if necessary to ensure p0 contains yMin}
>>>>>>> origin/fixes_2_2
    if (p0.y > p1.y) then begin
      p0 := p1;
      p1 := ptable[i];
    end;
   { create the new edge }
    et^[nEdges].ymin := p0.y;
    et^[nEdges].ymax := p1.y;
    et^[nEdges].x := p0.x;
    et^[nEdges].dX := p1.x-p0.x;
    et^[nEdges].dy := p1.y-p0.y;
    et^[nEdges].frac := 0;
    get^[nEdges] :=  @et^[nEdges];
    inc(nEdges);
  end;
 { sort the GET on ymin }
  gap := 1;
  while (gap < nEdges) do gap := 3*gap+1;
  gap := gap div 3;
  while (gap > 0) do begin
    for i := gap to (nEdges-1) do begin
      j := i - gap;
      while (j >= 0) do begin
        if (GET^[j]^.ymin <= GET^[j+gap]^.yMin) then break;
        t := GET^[j];
        GET^[j] := GET^[j+gap];
        GET^[j+gap] := t;
        dec(j, gap);
      end;
    end;
    gap := gap div 3;
  end;
  { initialize the active edge table, and set y to first entering edge}
  nActive := 0;
  nNextEdge := 0;

  y := GET^[nNextEdge]^.ymin;
  { Now process the edges using the scan line algorithm.  Active edges
  will be added to the Active Edge Table (AET), and inactive edges will
  be deleted.  X coordinates will be updated with incremental integer
  arithmetic using the slope (dY / dX) of the edges. }
  while (nNextEdge < nEdges) or (nActive <> 0) do begin
    {Move from the ET bucket y to the AET those edges whose yMin == y
    (entering edges) }
    while (nNextEdge < nEdges) and (GET^[nNextEdge]^.ymin = y) do begin
      AET^[nActive] := GET^[nNextEdge];
      inc(nActive);
      inc(nNextEdge);
    end;
    { Remove from the AET those entries for which yMax == y (leaving
    edges) }
    i := 0;
    while (i < nActive) do begin
      if (AET^[i]^.yMax = y) then begin
        dec(nActive);
        System.move(AET^[i+1], AET^[i], (nActive-i)*sizeof(pedge));
      end else
        inc(i);
    end;

    if (y >= 0) then begin
    {Now sort the AET on x.  Since the list is usually quite small,
    the sort is implemented as a simple non-recursive shell sort }

    gap := 1;
    while (gap < nActive) do gap := 3*gap+1;

    gap := gap div 3;
    while (gap > 0) do begin
      for i := gap to (nActive-1) do begin
        j := i - gap;
        while (j >= 0) do begin
          if (AET^[j]^.x <= AET^[j+gap]^.x) then break;
          t := AET^[j];
          AET^[j] := AET^[j+gap];
          AET^[j+gap] := t;
          dec(j, gap);
        end;
      end;
      gap := gap div 3;
    end;

    { Fill in desired pixels values on scan line y by using pairs of x
    coordinates from the AET }
    i := 0;
<<<<<<< HEAD
<<<<<<< HEAD
    while (i < (nActive - 1)) do begin
      x0 := AET^[i]^.x;
      x1 := AET^[i+1]^.x;
      {Left edge adjustment for positive fraction.  0 is interior. }
      if (AET^[i]^.frac >= 0) then inc(x0);
=======
=======
>>>>>>> origin/fixes_2_2
    while (i < nActive) do begin
      x0 := AET^[i]^.x;
      x1 := AET^[i+1]^.x;
      {Left edge adjustment for positive fraction.  0 is interior. }
      if (AET^[i]^.frac > 0) then inc(x0);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      {Right edge adjustment for negative fraction.  0 is exterior. }
      if (AET^[i+1]^.frac <= 0) then dec(x1);

      x0 := max(x0, 0);
      x1 := min(x1, viewWidth);
      { Draw interior spans}
      if (x1 >= x0) then begin
        PatternLine(x0, x1, y);
      end;

      inc(i, 2);
    end;

    end;

    { Update all the x coordinates.  Edges are scan converted using a
    modified midpoint algorithm (Bresenham's algorithm reduces to the
    midpoint algorithm for two dimensional lines) }
    for i := 0 to (nActive-1) do begin
      t := AET^[i];
      { update the fraction by dX}
      inc(t^.frac, t^.dX);

      if (t^.dX < 0) then
        while ( -(t^.frac) >= t^.dY) do begin
          inc(t^.frac, t^.dY);
          dec(t^.x);
        end
      else
        while (t^.frac >= t^.dY) do begin
          dec(t^.frac, t^.dY);
          inc(t^.x);
        end;
    end;
    inc(y);
    if (y >= ViewHeight) then break;
  end;
<<<<<<< HEAD
<<<<<<< HEAD

  { finally, draw the edges }
  LastPolygonStart := 0;
  Closing := false;
  for i := 0 to (numpoints-1) do begin
    p0 := ptable[i];
    if (i+1) >= numpoints then p1 := ptable[0]
    else p1 := ptable[i+1];
    { save the 'closing' flag for the previous edge }
    PrevClosing := Closing;
    { check if the current edge is 'closing'. This means that it 'closes'
      the polygon by going back to the first point of the polygon.
      Also, 0-length edges are never considered 'closing'. }
    if ((p1.x <> p0.x) or
        (p1.y <> p0.y)) and
        (LastPolygonStart < i) and
       ((p1.x = ptable[LastPolygonStart].x) and
        (p1.y = ptable[LastPolygonStart].y)) then
    begin
      Closing := true;
      LastPolygonStart := i + 2;
    end
    else
      Closing := false;
    { skip edge if the previous edge was 'closing'. This is TP7 compatible }
    if PrevClosing then
      continue;
   { draw the edges }
    Line(p0.x,p0.y,p1.x,p1.y);
  end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  System.freemem(et, sizeof(tedge) * numpoints);
  System.freemem(get, sizeof(pedge) * numpoints);
  System.freemem(aet, sizeof(pedge) * numpoints);
end;


{ maximum supported Y resultion }
const
  MaxYRes = 2048;
  { changing this to 1 or 2 doesn't improve performance noticably }
  YResDiv = 4;

type
  PFloodLine = ^TFloodLine;
  TFloodLine = record
    next: PFloodLine;
    x1 : smallint;
    x2 : smallint;
    y  : smallint;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  PDrawnList  = ^TDrawnList;
  TDrawnList  = Array[0..(MaxYRes - 1) div YResDiv] of PFloodLine;

var
   DrawnList : PDrawnList;
=======
=======
>>>>>>> origin/fixes_2_2
  TDrawnList  = Array[0..(MaxYRes - 1) div YResDiv] of PFloodLine;

var
   DrawnList : TDrawnList;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   Buffer : Record                         { Union for byte and word addressing of buffer }
     ByteIndex : Word;
     WordIndex : Word;
     Case Boolean Of
        False : (Bytes : Array [0..StdBufferSize-1] Of Byte);
        True  : (Words : Array [0..(StdBufferSize DIV 2)-1] Of Word);
     End;

  s1, s2, s3 : PWordArray;                { Three buffers for scanlines                 }


  Procedure PushPoint (x, y : smallint);
  {********************************************************}
  { Adds a  point to the list of points to check if we     }
  { need to draw. Doesn't add the point if there is a      }
  { buffer overflow.                                       }
  {********************************************************}
  Begin
    If Buffer.WordIndex<(StdBufferSize DIV 2)-3 then
     Begin
       Buffer.Words[Buffer.WordIndex]:=x;
       Buffer.Words[Buffer.WordIndex+1]:=y;
       Inc (Buffer.WordIndex,2);
     End
  End;

  Procedure PopPoint (Var x, y : smallint);
  {********************************************************}
  { Removes a point from the list of points to check, if   }
  { we try to access an illegal point, then the routine    }
  { returns -1,-1 as a coordinate pair.                    }
  {********************************************************}
  Begin
   If Buffer.WordIndex>1 then
    Begin
      x:=Buffer.Words[Buffer.WordIndex-2];
      y:=Buffer.Words[Buffer.WordIndex-1];
      Dec (Buffer.WordIndex,2);
    End
   Else
    Begin
      x:=-1;
      y:=-1;
    End;
  End;






  {********************************************************}
  { Procedure AddLinePoints()                              }
  {--------------------------------------------------------}
  { Adds a line segment to the list of lines which will be }
  { drawn to the screen. The line added is on the specified}
  { Y axis, from the x1 to x2 coordinates.                 }
  {********************************************************}
  Procedure AddLinePoints(x1,x2,y: smallint);
   var temp: PFloodLine;
   begin
     new(temp);
     temp^.x1 := x1;
     temp^.x2 := x2;
     temp^.y := y;
<<<<<<< HEAD
<<<<<<< HEAD
     temp^.next := DrawnList^[y div YResDiv];
     DrawnList^[y div YResDiv] := temp;
=======
     temp^.next := DrawnList[y div YResDiv];
     DrawnList[y div YResDiv] := temp;
>>>>>>> graemeg/fixes_2_2
=======
     temp^.next := DrawnList[y div YResDiv];
     DrawnList[y div YResDiv] := temp;
>>>>>>> origin/fixes_2_2
   end;

  {********************************************************}
  { Procedure AlreadyDrawn()                               }
  {--------------------------------------------------------}
  { This routine searches through the list of segments     }
  { which will be drawn to the screen, and determines  if  }
  { the specified point (x,y) will already be drawn.       }
  { i.e : Checks if the x,y point lies within a known      }
  { segment which will be drawn to the screen. This makes  }
  { sure that we don't draw some segments two times.       }
  { Return TRUE if the point is already in the segment list}
  { to draw, otherwise returns FALSE.                      }
  {********************************************************}
  Function AlreadyDrawn(x, y: smallint): boolean;
  var
    temp : PFloodLine;
   begin
    AlreadyDrawn := false;
<<<<<<< HEAD
<<<<<<< HEAD
    temp := DrawnList^[y div YResDiv];
=======
    temp := DrawnList[y div YResDiv];
>>>>>>> graemeg/fixes_2_2
=======
    temp := DrawnList[y div YResDiv];
>>>>>>> origin/fixes_2_2
    while assigned(temp) do
      begin
        if (temp^.y = y) and
           (temp^.x1 <= x) and
           (temp^.x2 >= x) then
          begin
            AlreadyDrawn := true;
            exit;
          end;
        temp := temp^.next;
      end;
   end;

  {********************************************************}
  { Procedure CleanUpDrawnList                             }
  {--------------------------------------------------------}
  { removes all elements from the DrawnList. Doesn't init  }
  { elements of it with NILL                               }
  {********************************************************}
  Procedure CleanUpDrawnList;
  var
<<<<<<< HEAD
<<<<<<< HEAD
    l: smallint;
    temp1, temp2: PFloodLine;
  begin
    for l := 0 to ViewHeight div YResDiv do
      begin
        temp1 := DrawnList^[l];
=======
=======
>>>>>>> origin/fixes_2_2
    l: longint;
    temp1, temp2: PFloodLine;
  begin
    for l := 0 to high(DrawnList) do
      begin
        temp1 := DrawnList[l];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        while assigned(temp1) do
          begin
            temp2 := temp1;
            temp1 := temp1^.next;
            dispose(temp2);
          end;
      end;
  end;


  Procedure FloodFill (x, y : smallint; Border: word);
  {********************************************************}
  { Procedure FloodFill()                                  }
  {--------------------------------------------------------}
  { This routine fills a region of the screen bounded by   }
  { the <Border> color. It uses the current fillsettings   }
  { for the flood filling. Clipping is supported, and      }
  { coordinates are local/viewport relative.               }
  {********************************************************}
  Var
   stemp: PWordArray;
   Beginx : smallint;
   d, e : Byte;
   Cont : Boolean;
   BackupColor : Word;
   x1, x2, prevy: smallint;
  Begin
<<<<<<< HEAD
<<<<<<< HEAD
    GetMem(DrawnList,sizeof(PFloodLine)*((ViewHeight div YResDiv) + 1));
    FillChar(DrawnList^,sizeof(PFloodLine)*((ViewHeight div YResDiv) + 1),0);
=======
    FillChar(DrawnList,sizeof(DrawnList),0);
>>>>>>> graemeg/fixes_2_2
=======
    FillChar(DrawnList,sizeof(DrawnList),0);
>>>>>>> origin/fixes_2_2
    { init prevy }
    prevy := 32767;
    { Save current drawing color }
    BackupColor := CurrentColor;
    CurrentColor := FillSettings.Color;
    { MaxX is based on zero index }
    GetMem (s1,(ViewWidth+1)*2);  { A pixel color represents a word }
    GetMem (s2,(ViewWidth+1)*2);  { A pixel color represents a word }
    GetMem (s3,(ViewWidth+1)*2);  { A pixel color represents a word }
    if (not assigned(s1)) or (not assigned(s2)) or (not assigned(s3)) then
      begin
        _GraphResult := grNoFloodMem;
        exit;
      end;
    If (x<0) Or (y<0) Or
       (x>ViewWidth) Or (y>ViewHeight) then Exit;
    { Index of points to check  }
    Buffer.WordIndex:=0;
    PushPoint (x,y);
    While Buffer.WordIndex>0 Do
     Begin
       PopPoint (x,y);
       { Get the complete lines for the following }
       If y <> prevy then
         begin
           If (prevy - y = 1) then
             { previous line was one below the new one, so the previous s2 }
             { = new s1                                                    }
             Begin
               stemp := s3;
               s3 := s1;
               s1 := s2;
               s2 := stemp;
               GetScanline(0,ViewWidth,y-1,s2^);
             End
           Else If (y - prevy = 1) then
             { previous line was one above the new one, so the previous s3 }
             { = new s1                                                    }
             Begin
               stemp := s2;
               s2 := s1;
               s1 := s3;
               s3 := stemp;
               GetScanline(0,ViewWidth,y+1,s3^);
             End
           Else
             begin
               GetScanline(0,ViewWidth,y-1,s2^);
               GetScanline(0,ViewWidth,y,s1^);
               GetScanline(0,ViewWidth,y+1,s3^);
             end;
         end;
       prevy := y;
       { check the current scan line }
       While (s1^[x]<>Border) And (x<=ViewWidth) Do Inc (x);
       d:=0;
       e:=0;
       dec(x);
       Beginx:=x;
       REPEAT
         { check the above line }
         If y<ViewHeight then
           Begin
              Cont:=(s3^[x]<>Border) and (not AlreadyDrawn(x,y+1));
              If (e=0) And Cont then
                Begin
                  PushPoint (x,y+1);
                  e:=1;
                End
              Else
                If (e=1) And Not Cont then e:=0;
           End;
        { check the line below }
        If (y>0) then
          Begin
            Cont:=(s2^[x]<>Border) and (not AlreadyDrawn(x,y-1));
            If (d=0) And Cont then
              Begin
                PushPoint (x,y-1);
                d:=1;
              End
            Else
              If (d=1) And Not Cont then d:=0;
          End;
        Dec (x);
       Until (x<0) Or (s1^[x]=Border);
       { swap the values }
       x1:=x+1;
       x2:=BeginX;
       if x1 > x2 then
         Begin
           x:=x1;
           x1:=x2;
           x2:=x;
         end;
       { Add to the list of drawn lines }
       AddLinePoints(x1,x2,y);
       PatternLine (x1,x2,y);
     End; { end while }

    System.FreeMem (s1,(ViewWidth+1)*2);
    System.FreeMem (s2,(ViewWidth+1)*2);
    System.FreeMem (s3,(ViewWidth+1)*2);
    CleanUpDrawnList;
<<<<<<< HEAD
<<<<<<< HEAD
    System.FreeMem(DrawnList,sizeof(PFloodLine)*((ViewHeight div YResDiv) + 1));
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    CurrentColor := BackUpColor;
  End;

{ restore previous range check mode }
<<<<<<< HEAD
<<<<<<< HEAD
{$pop}
=======
{$ifdef OPT_R_WAS_ON}
{$R+}
{$endif}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef OPT_R_WAS_ON}
{$R+}
{$endif}
>>>>>>> origin/fixes_2_2
