program project1;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Classes
  { you can add units after this };

type
  IIntf1 = interface
    ['{87776F0F-8CE0-4881-B969-C76F5A9CA517}']
    procedure M1;
  end;

  IIntf2 = interface
    ['{923C47DF-0A7E-4698-98B8-45175306CDF2}']
    procedure M2;
  end;

  { TObjIntf2 }

  TObjIntf2 = class(TInterfacedObject, IIntf2)
    procedure M2;
  end;

  { TObj }

  TObj = class(TInterfacedObject, IIntf1, IIntf2)
    private
      FObjIntf2:IIntf2;
    public
      constructor Create;

      procedure M1;

      //when implementing IIntf2 using delegation,
      //TObj1.M1 is called instead of TObjIntf2
      property I2:IIntf2 read FObjIntf2 implements IIntf2;

      //when implementing M2 directly it works right.
      //procedure M2;
  end;

{ TObjIntf2 }

procedure TObjIntf2.M2;
begin
  Writeln('TObjIntf2.M2 called');
end;

{ TObj }

constructor TObj.Create;
begin
  FObjIntf2:=TObjIntf2.Create;
end;

procedure TObj.M1;
begin
  Writeln('TObj.M1 called');
<<<<<<< HEAD
  halt(1);
=======
>>>>>>> origin/fixes_2.4
end;

{
procedure TObj.M2;
begin
  Writeln('TObj.M2 called');
end;
}

var O:TObj;
    i1:IIntf1;
    i2:IIntf2;
begin
  O:=TObj.Create;
  i1:=O;

  //all tries are unsuccessful
  //i2:=O as IIntf2;
  //(O as IIntf1).QueryInterface(IIntf2, i2);
  i1.QueryInterface(IIntf2, i2);

  //still calls TObj1.M1
  i2.M2;
end.

