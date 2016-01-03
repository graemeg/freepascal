{ Control stack

<<<<<<< HEAD
<<<<<<< HEAD
  CopyRight (C) 2004-2008 Ales Katona
=======
  CopyRight (C) 2004-2007 Ales Katona
>>>>>>> graemeg/fixes_2_2
=======
  CopyRight (C) 2004-2007 Ales Katona
>>>>>>> origin/fixes_2_2

  This library is Free software; you can rediStribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is diStributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; withOut even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a Copy of the GNU Library General Public License
  along with This library; if not, Write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

  This license has been modified. See File LICENSE for more inFormation.
  Should you find these sources withOut a LICENSE File, please contact
  me at ales@chello.sk
}

unit lControlStack;

{$mode objfpc}

interface

const
  TL_CSLENGTH = 3;

type
  TLOnFull = procedure of object;
  
  TLControlStack = class
   private
    FItems: array of Char;
    FIndex: Byte;
    FAllowInflation: Boolean;
    FOnFull: TLOnFull;
    function GetFull: Boolean;
    function GetItem(const i: Byte): Char;
    procedure SetItem(const i: Byte; const Value: Char);
    procedure SetAllowInflation(const b: boolean);
   public
    constructor Create;
    procedure Clear;
    procedure Push(const Value: Char);
    property ItemIndex: Byte read FIndex;
    property AllowInflation: Boolean read FAllowInflation write SetAllowInflation;
    property Items[i: Byte]: Char read GetItem write SetItem; default;
    property Full: Boolean read GetFull;
    property OnFull: TLOnFull read FOnFull write FOnFull;
  end;

implementation

uses
  lTelnet;

(* The normal situation is that there are up to TL_CSLENGTH items on the stack. *)
(* However this may be relaxed in cases (assumed to be rare) where subcommand   *)
(* parameters are being accumulated.                                            *)
  
constructor TLControlStack.Create;
begin
  FOnFull:=nil;
  FIndex:=0;                            (* Next insertion point, [0] when empty *)
  FAllowInflation := false;
  SetLength(FItems, TL_CSLENGTH);
end;

function TLControlStack.GetFull: Boolean;
begin
  Result:=False;                        (* It's full when it has a complete     *)
  if FIndex >= TL_CSLENGTH then         (* command, irrespective of whether the *)
    Result:=True;                       (* stack's inflated by a subcommand.    *)
end;

function TLControlStack.GetItem(const i: Byte): Char;
begin
  Result:=TS_NOP;
  if not FAllowInflation then begin
    if i < TL_CSLENGTH then
      Result:=FItems[i]
  end else
    if i < Length(FItems) then
      Result:=FItems[i]
end;

procedure TLControlStack.SetItem(const i: Byte; const Value: Char);
begin
  if not FAllowInflation then begin
    if i < TL_CSLENGTH then
      FItems[i]:=Value
  end else begin
    while i >= Length(FItems) do begin
      SetLength(FItems, Length(FItems) + 1);
      FItems[Length(FItems) - 1] := TS_NOP
    end;
    FItems[i] := Value
  end
end;

procedure TLControlStack.SetAllowInflation(const b: boolean);

begin
  FAllowInflation := b;
  if not b then                         (* No more funny stuff please           *)
    Clear
end;

procedure TLControlStack.Clear;
begin
  FIndex:=0;
  FAllowInflation := false;
  SetLength(FItems, TL_CSLENGTH)        (* In case inflation was allowed        *)
end;

procedure TLControlStack.Push(const Value: Char);
begin
  if not FAllowInflation then
    if FIndex < TL_CSLENGTH then begin
      FItems[FIndex]:=Value;
      Inc(FIndex)
    end else begin end
  else begin
    SetLength(FItems, Length(FItems) + 1);
    FItems[Length(FItems) - 1] := Value;
    FIndex := Length(FItems)
  end;
  if Full and Assigned(FOnFull) then
    FOnFull;
end;

end.

