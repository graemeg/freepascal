{
  This file is part of the Free Pascal run time library.
  Copyright (C) 2020 Michael Van Canneyt
  member of the Free Pascal development team.

  Nullable generic type.

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
unit nullable;

{$mode objfpc}
{$modeswitch advancedrecords}

interface

uses sysutils;

Type

  { TNullable }

  generic TNullable<T> = record
  private
    FValue: T;
    FHasValue: Boolean; // Default False
    function GetIsNull: Boolean;
    function GetValue: T;
    function GetValueOrDefault: T;
    procedure SetHasValue(AValue: Boolean);
    procedure SetValue(AValue: T);
  Public
    // Make things more readable
    Type
      TMyType = specialize TNullable<T>;
    // Clear value, no value present after this.
    procedure Clear;
    // Is a value present ?
    property HasValue: Boolean read FHasValue write SetHasValue;
    // Is No value present
    property IsNull: Boolean read GetIsNull;
    // return the value.
    property Value: T read GetValue write SetValue;
    // If a value is present, return it, otherwise return the default.
    property ValueOrDefault: T read GetValueOrDefault;
    // Return an empty value
    class function Empty: TMyType; static;
    // management operator
    class operator Initialize(var aSelf : TNullable);
    // Conversion.
    class operator Explicit(aValue: T): TMyType;
    class operator Explicit(aValue: TMyType): T;
    class operator := (aValue: T): TMyType;
    class operator := (aValue: TMyType): T;
   end;

implementation

uses rtlconsts,typinfo;

{ TNullable }

function TNullable.GetIsNull: Boolean;
begin
  Result:=Not HasValue;
end;

function TNullable.GetValue: T;
begin
  if not FHasValue then
    raise EConvertError.CreateFmt(SErrCannotConvertNullToType,[PtypeInfo(TypeInfo(T))^.Name]);
  Result:=FValue;
end;

function TNullable.GetValueOrDefault: T;
begin
  if HasValue then
    Result:=Value
  else
    Result:=Default(T);
end;

procedure TNullable.SetHasValue(AValue: Boolean);
begin
  if FHasValue=AValue then Exit;
  if aValue then
    Value:=Default(T)
  else
    FHasValue:=False;
end;

procedure TNullable.SetValue(AValue: T);
begin
  FValue:=aValue;
  FHasValue:=True;
end;

procedure TNullable.Clear;
begin
  HasValue:=False;
end;

class operator TNullable.Initialize(var aSelf: TNullable);
begin
  aSelf.FHasValue:=False;
end;

class function TNullable.Empty: TMyType; static;

begin
  Result.HasValue:=False;
end;

class operator TNullable.Explicit(aValue: T): TMyType;

begin
  Result.Value:=aValue;
end;

class operator TNullable.Explicit(aValue: TMyType): T;

begin
  Result:=aValue.Value;
end;

class operator TNullable.:= (aValue: T): TMyType;
begin
  Result.Value:=aValue;
end;

class operator TNullable.:= (aValue: TMyType): T;

begin
  // We could use :=This is in line with TField's behaviour.
  Result:=aValue.Value;
end;

end.
