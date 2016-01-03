{

   Timed dialogs for Free Vision

   Copyright (c) 2004 by Free Pascal core team

<<<<<<< HEAD
<<<<<<< HEAD
   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.
=======
=======
>>>>>>> origin/fixes_2_2
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the Free
<<<<<<< HEAD
<<<<<<< HEAD
   Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301, USA.
=======
   Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> origin/fixes_2_2

 ****************************************************************************}
UNIT timeddlg;

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                  INTERFACE
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{====Include file to sort compiler platform out =====================}
{$I platform.inc}
{====================================================================}

{==== Compiler directives ===========================================}

{$IFNDEF PPC_FPC}{ FPC doesn't support these switches }
  {$F-} { Near calls are okay }
  {$A+} { Word Align Data }
  {$B-} { Allow short circuit boolean evaluations }
  {$O+} { This unit may be overlaid }
  {$G+} { 286 Code optimization - if you're on an 8088 get a real computer }
  {$P-} { Normal string variables }
  {$N-} { No 80x87 code generation }
  {$E+} { Emulation is on }
{$ENDIF}

{$X+} { Extended syntax is ok }
{$R-} { Disable range checking }
{$S-} { Disable Stack Checking }
{$I-} { Disable IO Checking }
{$Q-} { Disable Overflow Checking }
{$V-} { Turn off strict VAR strings }
{====================================================================}

USES objects, dialogs, fvconsts, drivers, views; { Standard GFV unit }

type
  TTimedDialog = object (TDialog)
    Secs: longint;
    constructor Init (var Bounds: TRect; ATitle: TTitleStr; ASecs: word);
    constructor Load (var S: TStream);
    procedure   GetEvent (var Event: TEvent); virtual;
    procedure   Store (var S: TStream); virtual;
  private
    Secs0: longint;
    Secs2: longint;
    DayWrap: boolean;
  end;
  PTimedDialog = ^TTimedDialog;

(* Must be always included in TTimeDialog! *)
  TTimedDialogText = object (TStaticText)
    constructor Init (var Bounds: TRect);
    procedure   GetText (var S: string); virtual;
  end;
  PTimedDialogText = ^TTimedDialogText;

const
  RTimedDialog: TStreamRec = (
    ObjType: idTimedDialog;
{$IFDEF BP_VMTLink}                              { BP style VMT link }
    VmtLink: Ofs (TypeOf (TTimedDialog)^);
{$ELSE}                                          { Alt style VMT link }
    VmtLink: TypeOf (TTimedDialog);
{$ENDIF BP_VMTLink}
    Load:    @TTimedDialog.Load;
    Store:   @TTimedDialog.Store
  );

  RTimedDialogText: TStreamRec = (
    ObjType: idTimedDialogText;
{$IFDEF BP_VMTLink}                              { BP style VMT link }
    VmtLink: Ofs (TypeOf (TTimedDialogText)^);
{$ELSE}                                          { Alt style VMT link }
    VmtLink: TypeOf (TTimedDialogText);
{$ENDIF BP_VMTLink}
    Load:    @TTimedDialogText.Load;
    Store:   @TTimedDialogText.Store
  );

procedure RegisterTimedDialog;

FUNCTION TimedMessageBox (Const Msg: String; Params: Pointer;
  AOptions: Word; ASecs: Word): Word;

{-TimedMessageBoxRect------------------------------------------------
TimedMessageBoxRect allows the specification of a TRect for the message box
to occupy.
---------------------------------------------------------------------}
FUNCTION TimedMessageBoxRect (Var R: TRect; Const Msg: String; Params: Pointer;
  AOptions: Word; ASecs: Word): Word;


{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

USES
  dos,
  app, {resource,} msgbox;   { Standard GFV units }


{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

constructor TTimedDialogText.Init (var Bounds: TRect);
begin
  inherited Init (Bounds, '');
end;


procedure TTimedDialogText.GetText (var S: string);
begin
  if Owner <> nil
(* and (TypeOf (Owner^) = TypeOf (TTimedDialog)) *)
                  then
   begin
    Str (PTimedDialog (Owner)^.Secs, S);
    S := #3 + S;
   end
  else
   S := '';
end;



constructor TTimedDialog.Init (var Bounds: TRect; ATitle: TTitleStr;
  ASecs: word);
var
  H, M, S, S100: word;
begin
  inherited Init (Bounds, ATitle);
  GetTime (H, M, S, S100);
  Secs0 := H * 3600 + M * 60 + S;
  Secs2 := Secs0 + ASecs;
  Secs := ASecs;
  DayWrap := Secs2 > 24 * 3600;
end;


procedure TTimedDialog.GetEvent (var Event: TEvent);
var
  H, M, S, S100: word;
  Secs1: longint;
begin
  inherited GetEvent (Event);
  GetTime (H, M, S, S100);
  Secs1 := H * 3600 + M * 60 + S;
  if DayWrap then Inc (Secs1, 24 * 3600);
  if Secs2 - Secs1 <> Secs then
   begin
    Secs := Secs2 - Secs1;
    if Secs < 0 then
     Secs := 0;
(* If remaining seconds are displayed in one of included views, update them. *)
    Redraw;
   end;
  with Event do
   if (Secs = 0) and (What = evNothing) then
    begin
     What := evCommand;
     Command := cmCancel;
    end;
end;


constructor TTimedDialog.Load (var S: TStream);
begin
  inherited Load (S);
  S.Read (Secs, SizeOf (Secs));
  S.Read (Secs0, SizeOf (Secs0));
  S.Read (Secs2, SizeOf (Secs2));
  S.Read (DayWrap, SizeOf (DayWrap));
end;


procedure TTimedDialog.Store (var S: TStream);
begin
  inherited Store (S);
  S.Write (Secs, SizeOf (Secs));
  S.Write (Secs0, SizeOf (Secs0));
  S.Write (Secs2, SizeOf (Secs2));
  S.Write (DayWrap, SizeOf (DayWrap));
end;



function TimedMessageBox (const Msg: string; Params: pointer;
  AOptions: word; ASecs: word): word;
var
  R: TRect;
begin
  R.Assign(0, 0, 40, 10);                            { Assign area }
  if (AOptions AND mfInsertInApp = 0) then           { Non app insert }
   R.Move((Desktop^.Size.X - R.B.X) div 2,
      (Desktop^.Size.Y - R.B.Y) div 2)               { Calculate position }
  else
   R.Move((Application^.Size.X - R.B.X) div 2,
      (Application^.Size.Y - R.B.Y) div 2);          { Calculate position }
  TimedMessageBox := TimedMessageBoxRect (R, Msg, Params,
    AOptions, ASecs);                                { Create message box }
end;


function TimedMessageBoxRect (var R: TRect; const Msg: string; Params: pointer;
  AOptions: word; ASecs: word): word;
var
  Dlg: PTimedDialog;
  TimedText: PTimedDialogText;
begin
  Dlg := New (PTimedDialog, Init (R, MsgBoxTitles [AOptions
    and $3], ASecs));                                { Create dialog }
  with Dlg^ do
   begin
    R.Assign (3, Size.Y - 5, Size.X - 2, Size.Y - 4);
    New (TimedText, Init (R));
    Insert (TimedText);
    R.Assign (3, 2, Size.X - 2, Size.Y - 5);         { Assign area for text }
   end;
  TimedMessageBoxRect := MessageBoxRectDlg (Dlg, R, Msg, Params, AOptions);
  Dispose (Dlg, Done);                               { Dispose of dialog }
end;



procedure RegisterTimedDialog;
begin
  RegisterType (RTimedDialog);
  RegisterType (RTimedDialogText);
end;


begin
  RegisterTimedDialog;
end.
