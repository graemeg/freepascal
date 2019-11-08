{ %OPT=-gh }

program tw36196;

{$mode objfpc}{$H+}
{$modeswitch prefixedattributes}

uses
  {heaptrc, }SysUtils, typinfo, {%H-}rtti;

type

  IntRangeAttribute = class(TCustomAttribute)
  private
    FMinValue,
    FMaxValue: Integer;
  public
    constructor Create(aMin, aMax: Integer);
    property MinValue: Integer read FMinValue;
    property MaxValue: Integer read FMaxValue;
  end;

  DefaultStrAttribute = class(TCustomAttribute)
  private
    FValue: string;
  public
    constructor Create;
    constructor Create(const aValue: string);
    property Value: string read FValue;
  end;

  CheckAttribute = class(TCustomAttribute)
  private
    FChecked: Boolean;
  public
    constructor Create;
    constructor Create(aValue: Boolean);
    property Checked: Boolean read FChecked;
  end;

  [Check(True)]
  TMyClass = class
  private
    FName,
    FDescr: string;
    FId: Integer;
  published
    [DefaultStr]
    property Name: string read FName write FName;
    [DefaultStr('nice description')]
    property Description: string read FDescr write FDescr;
    [IntRange(100, 10000)]
    property Id: Integer read FId write FId;
  end;

  [DefaultStr('this is TMyRec')][IntRange(0, 100)]
  TMyRec = record
    Name: string;
    Value: Integer;
  end;

  [Check][IntRange(-10000, 10000)]
  TMyInt = type Integer;

{ CheckAttribute }

constructor CheckAttribute.Create;
begin
  FChecked := False;
end;

constructor CheckAttribute.Create(aValue: Boolean);
begin
  FChecked := aValue;
end;

{ DefaultStrAttribute }

constructor DefaultStrAttribute.Create;
begin
  FValue := 'Unassigned';
end;

constructor DefaultStrAttribute.Create(const aValue: string);
begin
  FValue := aValue;
end;

{ TIntRangeAttribute }

constructor IntRangeAttribute.Create(aMin, aMax: Integer);
begin
  FMinValue := aMin;
  FMaxValue := aMax;
end;

procedure PrintAttribute(Attr: TCustomAttribute);
begin
  if not Assigned(Attr) then
    exit;
  WriteLn('  Found attribute ', Attr.ClassName, ':');
  if Attr is DefaultStrAttribute then
    WriteLn('    property Value has value "', DefaultStrAttribute(Attr).Value, '"')
  else
    if Attr is IntRangeAttribute then
      begin
        WriteLn('    property MinValue has value ', IntRangeAttribute(Attr).MinValue);
        WriteLn('    property MaxValue has value ', IntRangeAttribute(Attr).MaxValue);
      end
    else
      if Attr is CheckAttribute then
        WriteLn('    property Checked has value ', CheckAttribute(Attr).Checked);
end;

procedure PrintClassAttributes(aClass: TClass);
var
 RCtx: TRttiContext;
 RType: TRttiType;
 Prop: TRttiProperty;
 Attr: TCustomAttribute;
begin
  RCtx := TRttiContext.Create;
  try
    RType := RCtx.GetType(aClass);
    WriteLn(RType.Name, ' attributes:');
    for Attr in RType.GetAttributes do
      PrintAttribute(Attr);
    for Prop in RType.GetProperties do
      for Attr in Prop.GetAttributes do
        PrintAttribute(Attr);
  finally
    RCtx.Free;
  end;
end;

procedure PrintTypeAttributes(aInfo: PTypeInfo);
var
 RCtx: TRttiContext;
 RType: TRttiType;
 Attr: TCustomAttribute;
begin
  RCtx := TRttiContext.Create;
  try
    RType := RCtx.GetType(aInfo);
    WriteLn(RType.Name, ' attributes:');
    for Attr in RType.GetAttributes do
      PrintAttribute(Attr);
  finally
    RCtx.Free;
  end;
end;

begin
  HaltOnNotReleased:=True;
  //SetHeapTraceOutput('heap.log');
  PrintClassAttributes(TMyClass);
  PrintTypeAttributes(TypeInfo(TMyRec));
  PrintTypeAttributes(TypeInfo(TMyInt));
end.
