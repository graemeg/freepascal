program corbainterface;
{$ifdef FPC}{$mode objfpc}{$h+}{$endif}
{$ifdef mswindows}{$apptype console}{$endif}
uses
 sysutils;

type
 {$interfaces corba}
 icorbainterface1 = interface ['{9E8B9751-7779-4484-B6B7-960D18ACE7AB}']
  procedure iproc1;
 end;
 icorbainterface2 = interface ['MSE1']
  procedure iproc2;
 end;

 {$interfaces com}
 icominterface = interface ['{BC9EF8D0-2B67-4E5C-9952-05DF15A71567}']
  procedure iproc3;
 end;

 ttestclasscorba = class(tobject,icorbainterface1,icorbainterface2)
  public
   procedure iproc1;
   procedure iproc2;
 end;

 ttestclasscom = class(tinterfacedobject,icominterface)
  public
   procedure iproc3;
 end;

{ ttestclass }

procedure ttestclasscorba.iproc1;
begin
end;

procedure ttestclasscorba.iproc2;
begin
end;

{ ttestclasscom }

procedure ttestclasscom.iproc3;
begin
end;


var
 testclass1: ttestclasscorba;
 testclass2: ttestclasscom;
 po1: pointer;

begin
 testclass1:= ttestclasscorba.create;
 testclass2:= ttestclasscom.create;

 if testclass1.getinterface(icorbainterface1,po1) then begin
  writeln('getinterface icorbainterface1 found');
 end
 else begin
  writeln('getinterface icorbainterface1 not found');
 end;

<<<<<<< HEAD
<<<<<<< HEAD
 if testclass1.getinterface(icorbainterface2,po1) then begin
  writeln('getinterface icorbainterface2 found');
 end
 else begin
  writeln('getinterface icorbainterface2 not found');
 end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 if testclass2.getinterface(icominterface,po1) then begin
  writeln('getinterface icominterface found');
 end
 else begin
  writeln('getinterface icominterface not found');
 end;

 if testclass1.getinterfacebystr('MSE1',po1) then begin
  writeln('getinterfacebystr MSE1 found');
 end
 else begin
  writeln('getinterfacebystr MSE1 not found');
 end;

 testclass1.free;
 testclass2._Release;
end.

