program tw36738;

{$mode objfpc}
{$modeswitch advancedrecords}

uses
  SysUtils;

type

  TMyManagedRec = record
    Field1: Integer;
    Field2: Int64;
    class operator Initialize(var r: TMyManagedRec);
    class operator Copy(constref aSrc: TMyManagedRec; var aDst: TMyManagedRec);
  end;

  generic TGenericRec<T> = record
    SomeField: Integer;
    GenField: T;
  end;

  TSimpleRec = record
    SomeField: Integer;
    MngField: TMyManagedRec;
  end;

  TMyRecSpec = specialize TGenericRec<TMyManagedRec>;

class operator TMyManagedRec.Initialize(var r: TMyManagedRec);
begin
  r.Field1 := 101;
  r.Field2 := 1001;
end;

class operator TMyManagedRec.Copy(constref aSrc: TMyManagedRec; var aDst: TMyManagedRec);
begin
  if @aSrc <> @aDst then
    begin
      aDst.Field1 := aSrc.Field1 + 100;
      aDst.Field2 := aSrc.Field2 + 1000;
      Writeln(aDst.Field1);
      Writeln(aDst.Field2);
    end;
end;

var
  MyGenRec, MyGenRec2: TMyRecSpec;
  MyRec, MyRec2: TSimpleRec;

begin
  if IsManagedType(TMyRecSpec) then
    begin
      WriteLn('Yes, TMyRecSpec is a managed type');
      WriteLn('MyGenRec.GenField.Field1 = ', MyGenRec.GenField.Field1);
      if MyGenRec.GenField.Field1 <> 101 then
        Halt(1);
      WriteLn('MyGenRec.GenField.Field2 = ', MyGenRec.GenField.Field2);
      if MyGenRec.GenField.Field2 <> 1001 then
        Halt(2);
      WriteLn('MyGenRec2.GenField.Field1 = ', MyGenRec2.GenField.Field1);
      if MyGenRec2.GenField.Field1 <> 101 then
        Halt(3);
      WriteLn('MyGenRec2.GenField.Field2 = ', MyGenRec2.GenField.Field2);
      if MyGenRec2.GenField.Field2 <> 1001 then
        Halt(4);
      MyGenRec2 := MyGenRec;
      WriteLn('MyGenRec2.GenField.Field1 = ', MyGenRec2.GenField.Field1);
      if MyGenRec2.GenField.Field1 <> 201 then
        Halt(5);
      WriteLn('MyGenRec2.GenField.Field2 = ', MyGenRec2.GenField.Field2);
      if MyGenRec2.GenField.Field2 <> 2001 then
        Halt(6);
    end
  else begin
    WriteLn('No, TMyRecSpec is not a managed type');
    Halt(7);
  end;

  WriteLn;

  if IsManagedType(TSimpleRec) then
    begin
      WriteLn('Yes, TSimpleRec is a managed type');
      WriteLn('MyRec.MngField.Field1 = ', MyRec.MngField.Field1);
      if MyRec.MngField.Field1 <> 101 then
        Halt(8);
      WriteLn('MyRec.MngField.Field2 = ', MyRec.MngField.Field2);
      if MyRec.MngField.Field2 <> 1001 then
        Halt(9);
      WriteLn('MyRec2.MngField.Field1 = ', MyRec2.MngField.Field1);
      if MyRec2.MngField.Field1 <> 101 then
        Halt(10);
      WriteLn('MyRec2.MngField.Field2 = ', MyRec2.MngField.Field2);
      if MyRec.MngField.Field2 <> 1001 then
        Halt(11);
      MyRec2 := MyRec;
      WriteLn('MyRec2.MngField.Field1 = ', MyRec2.MngField.Field1);
      if MyRec2.MngField.Field1 <> 201 then
        Halt(12);
      WriteLn('MyRec2.MngField.Field2 = ', MyRec2.MngField.Field2);
      if MyRec2.MngField.Field2 <> 2001 then
        Halt(13);
    end
  else begin
    WriteLn('No, TSimpleRec is not a managed type');
    Halt(14);
  end;
  //ReadLn;
  Writeln('ok');
end.
