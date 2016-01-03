{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1998 - 2005 by the Free Pascal development team.

    This file implements platform independent routines for Crt.
    It should be modified later to use routines from Keyboard and
    Video instead of code in platform-specific crt.pas.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

var
  ScanCode: byte;
  SpecialKey: boolean;

procedure GotoXY (X: byte; Y: byte);
begin
 GotoXY32 (X, Y);
end;

procedure Window (X1, Y1, X2, Y2: byte);
begin
 Window32 (X1, Y1, X2, Y2);
end;

function WhereX: byte;
var
 X1: dword;
begin
 X1 := WhereX32;
 if X1 > 255 then
  WhereX := 255
 else
  WhereX := X1;
end;

function WhereY: byte;
var
 Y1: dword;
begin
 Y1 := WhereY32;
 if Y1 > 255 then
  WhereY := 255
 else
  WhereY := Y1;
end;


procedure ClrScr;
{Clears the current window.}
begin
 RemoveLines (0, Succ (WindMaxY - WindMinY));
 GotoXY32 (1, 1);
end;


procedure GotoXY32 (X, Y: dword);
(* Positions cursor on (X, Y) (1-based) relative to window origin; for TP/BP
   compatibility call completely ignored in case of incorrect parameters.    *)
begin
 if (X > 0) and (Y > 0) then
  begin
   Dec (X);
   Dec (Y);
   if (X <= WindMaxX - WindMinX) and (Y <= WindMaxY - WindMinY) then
    SetScreenCursor (X + WindMinX, Y + WindMinY);
  end;
end;


function WhereX32: dword;
(* Returns the X position of the cursor (1-based). *)
var
 X, Y: dword;
begin
 GetScreenCursor (X, Y);
 WhereX32 := Succ (X - WindMinX);
end;


function WhereY32: dword;
(* Returns the Y position of the cursor (1-based). *)
var
 X, Y: dword;
begin
 GetScreenCursor (X, Y);
 WhereY32 := Succ (Y - WindMinY);
end;


procedure ClrEol;
(* Clears the line where cursor is located from current position up to end. *)
var
 X, Y: dword;
begin
 GetScreenCursor (X, Y);
 ClearCells (X, Y, Succ (WindMaxX - X));
end;


procedure DelLine;
(* Deletes the line at cursor. *)
begin
 RemoveLines (Pred (WhereY32), 1);
end;


procedure TextMode (Mode: word);
{ Use this procedure to set-up a specific text-mode.}
begin
 TextAttr := $07;
 LastMode := Mode;
 SetScreenMode (Mode);
 WindMin := 0;
 WindMaxX := Pred (ScreenWidth);
 WindMaxY := Pred (ScreenHeight);
 if WindMaxX >= 255 then
  WindMax := 255
 else
  WindMax := WindMaxX;
 if WindMaxY >= 255 then
  WindMax := WindMax or $FF00
 else
  WindMax := WindMax or (WindMaxY shl 8);
 ClrScr;
end;


procedure TextColor (Color: byte);
{All text written after calling this will have Color as foreground colour.}
begin
 TextAttr := (TextAttr and $70) or (Color and $f);
 if Color > 15 then
  TextAttr := TextAttr or 128;
end;


procedure TextBackground (Color: byte);
{All text written after calling this will have Color as background colour.}
begin
 TextAttr := (TextAttr and $8F) or ((Color and $7) shl 4);
end;


procedure NormVideo;
{Changes the text-background to black and the foreground to white.}
begin
 TextAttr := $7;
end;


procedure LowVideo;
{All text written after this will have low intensity.}
begin
 TextAttr := TextAttr and $F7;
end;


procedure HighVideo;
{All text written after this will have high intensity.}
begin
 TextAttr := TextAttr or $8;
end;


procedure Window32 (X1, Y1, X2, Y2: dword);
{Change the write window to the given coordinates.}
begin
 if (X1 > 0) and (Y1 > 0) and (X2 <= ScreenWidth) and (Y2 <= ScreenHeight)
                                             and (X1 <= X2) and (Y1 <= Y2) then
  begin
   WindMinX := Pred (X1);
   WindMinY := Pred (Y1);
   if WindMinX >= 255 then
    WindMin := 255
   else
    WindMin := WindMinX;
   if WindMinY >= 255 then
    WindMin := WindMin or $FF00
   else
    WindMin := WindMin or (WindMinY shl 8);
   WindMaxX := Pred (X2);
   WindMaxY := Pred (Y2);
   if WindMaxX >= 255 then
    WindMax := 255
   else
    WindMax := WindMaxX;
   if WindMaxY >= 255 then
    WindMax := WindMax or $FF00
   else
    WindMax := WindMaxX or (WindMaxY shl 8);
   GotoXY32 (1, 1);
  end;
end;


threadvar
 CurrX, CurrY: dword;


procedure WriteChar (C: char);
begin
 case C of
  #7: WriteBell;
  #8: if CurrX >= WindMinX then
       Dec (CurrX);
{      #9: x:=(x-lo(windmin)) and $fff8+8+lo(windmin);}
  #10: Inc (CurrY);
  #13: CurrX := WindMinX;
  else
   begin
    WriteNormal (C, CurrX, CurrY);
    Inc (CurrX);
   end;
 end;
 if CurrX > WindMaxX then
  begin
   CurrX := WindMinX;
   Inc (CurrY);
  end;
 if CurrY > WindMaxY then
  begin
   RemoveLines (0, 1);
   CurrY := WindMaxY;
  end;
end;


function CrtWrite (var F: TextRec): integer;
var
 I: dword;
{Write a series of characters to the console.}
begin
 if F.BufPos > 0 then
  begin
   GetScreenCursor (CurrX, CurrY);
   for I := 0 to Pred (F.BufPos) do
    WriteChar ((PChar (F.BufPtr) + I)^);
   SetScreenCursor (CurrX, CurrY);
   F.BufPos := 0;
  end;
 CrtWrite := 0;
end;


function CrtRead (var F: TextRec): integer;
{Read a series of characters from the console.}
var
 C: char;
begin
 GetScreenCursor (CurrX, CurrY);
 F.BufPos := 0;
 F.BufEnd := 0;
 repeat
  if F.BufPos > F.BufEnd then
   F.BufEnd := F.BufPos;
  SetScreenCursor (CurrX, CurrY);
  C := ReadKey;
  case C of
   #0: ReadKey;
(* The following code to support input editing is incomplete anyway
   - no handling of line breaks, no possibility to insert characters
   or delete characters inside the string, etc.

   #0 : case readkey of
          #71 : while f.bufpos>0 do
                 begin
                   dec(f.bufpos);
                   WriteChar(#8);
                 end;
          #75 : if f.bufpos>0 then
                 begin
                   dec(f.bufpos);
                   WriteChar(#8);
                 end;
          #77 : if f.bufpos<f.bufend then
                 begin
                   WriteChar(f.bufptr^[f.bufpos]);
                   inc(f.bufpos);
                 end;
          #79 : while f.bufpos<f.bufend do
                 begin
                   WriteChar(f.bufptr^[f.bufpos]);
                   inc(f.bufpos);
                 end;
         end;
*)

   #8: if (F.BufPos > 0) and (F.BufPos = F.BufEnd) then
        begin
{$WARNING CrtRead doesn't handle line breaks correctly (same bug as TP/BP)!}
         WriteChar (#8);
         WriteChar (' ');
         WriteChar (#8);
         Dec (F.BufPos);
         Dec (F.BufEnd);
        end;
   #13: begin
         WriteChar(#13);
         WriteChar(#10);
         F.BufPtr^ [F.BufEnd] := #13;
         Inc (F.BufEnd);
         F.BufPtr^ [F.BufEnd] := #10;
         Inc (F.BufEnd);
         break;
        end;
   #26: if CheckEOF then
         begin
          F.BufPtr^ [F.BufEnd] := #26;
          Inc (F.BufEnd);
          break;
         end;
   #32..#255: if F.BufPos < F.BufSize - 2 then
               begin
                F.BufPtr^ [F.BufPos] := C;
                Inc (F.BufPos);
                WriteChar (C);
               end;
  end
 until false;
 CrtRead := 0;
end;


function CrtReturn (var F: TextRec): integer;
begin
 CrtReturn:=0;
end;


function CrtClose (var F: TextRec): integer;
begin
 F.Mode := fmClosed;
 CrtClose := 0;
end;


function CrtOpen (var F: TextRec): integer;
begin
 if F.Mode = fmOutput then
  begin
   TextRec(F).InOutFunc := @CrtWrite;
   TextRec(F).FlushFunc := @CrtWrite;
  end
 else
  begin
   F.Mode := fmInput;
   TextRec(F).InOutFunc := @CrtRead;
   TextRec(F).FlushFunc := @CrtReturn;
  end;
 TextRec(F).CloseFunc := @CrtClose;
 CrtOpen := 0;
end;


procedure AssignCrt (var F: text);
{Assigns a file to the crt console.}
begin
 Assign (F, '');
 TextRec (F).OpenFunc := @CrtOpen;
end;


{$IFNDEF HAS_SOUND}
procedure Sound (Hz: word);
(* Dummy Sound implementation - for platforms requiring both frequence
   and duration at the beginning instead of start and stop procedures. *)
begin
end;
{$ENDIF HAS_SOUND}


{$IFNDEF HAS_NOSOUND}
procedure NoSound;
(* Dummy NoSound implementation - for platforms requiring both frequence
   and duration at the beginning instead of start and stop procedures.   *)
begin
end;
{$ENDIF HAS_NOSOUND}


var
  PrevCtrlBreakHandler: TCtrlBreakHandler;


function CrtCtrlBreakHandler (CtrlBreak: boolean): boolean;
begin
(* Earlier registered handlers (e.g. FreeVision) have priority. *)
  if Assigned (PrevCtrlBreakHandler) then
    if PrevCtrlBreakHandler (CtrlBreak) then
      begin
        CrtCtrlBreakHandler := true;
        Exit;
      end;
(* If Ctrl-Break was pressed, either ignore it or allow default processing. *)
  if CtrlBreak then
    CrtCtrlBreakHandler := not (CheckBreak)
  else (* Ctrl-C pressed *)
    begin
      if not (SpecialKey) and (ScanCode = 0) then
        ScanCode := 3;
      CrtCtrlBreakHandler := true;
    end;
end;


procedure CrtInit;
(* Common part of unit initialization. *)
begin
 TextAttr := LightGray;
 WindMin := 0;
 WindMaxX := Pred (ScreenWidth);
 WindMaxY := Pred (ScreenHeight);
 if WindMaxX >= 255 then
  WindMax := 255
 else
  WindMax := WindMaxX;
 if WindMaxY >= 255 then
  WindMax := WindMax or $FF00
 else
  WindMax := WindMax or (WindMaxY shl 8);
 ScanCode := 0;
 SpecialKey := false;
 AssignCrt (Input);
 Reset (Input);
 AssignCrt (Output);
 Rewrite (Output);
 PrevCtrlBreakHandler := SysSetCtrlBreakHandler (@CrtCtrlBreakHandler);
 if PrevCtrlBreakHandler = TCtrlBreakHandler (pointer (-1)) then
   PrevCtrlBreakHandler := nil;
 CheckBreak := true;
end;
