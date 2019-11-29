program tb;

{$mode objfpc}
{$H+}

uses typinfo;

{$M+}
Type
  TBooleanArray = Array of Boolean;

  TMyObject = Class(TObject)
  Private
    FBooleans : TBooleanArray;
  Published
   Property Booleans: TBooleanArray Read FBooleans Write FBooleans;
  end;


Var
  O : TMyObject;
  P : Pointer;
  Info : PPropInfo;
  I : Integer;

begin
  O:=TMyObject.Create;
  try
    Info:=GetPropINfo(O,'Booleans');
    // Get property using RTTI (returns Nil, as expected)
    P:=GetObjectProp(O,Info);
    // Clear array
    I:=0;
    DynArraySetLength(P,Info^.PropType,1,@i);
    // Now, set new length
    I:=2;
    DynArraySetLength(P,Info^.PropType,1,@i);
    // Set some values
    TBooleanArray(P)[0]:=True;
    TBooleanArray(P)[1]:=False;
    // This is OK
    Writeln('Length  : ',Length(TBooleanArray(P)));
    // Set property using RTTI
    SetObjectProp(O,Info,TObject(P));
    // This goes wrong.
    Writeln('Correct pointer : ',HexStr(GetObjectProp(O,Info)),' = ',HexStr(P),' ? ',Pointer(GetObjectProp(O,Info))=P);
    // Crash !!
    Writeln('Length array : ',Length(O.Booleans));
  finally
    O.Free;
  end;
end.
