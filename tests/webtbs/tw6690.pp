program inheritedcorba;
{$mode objfpc}{$h+}
uses
 typinfo;

type
 {$interfaces corba}
 iinterface1 = interface
  procedure proc1;
 end;
 {$interfaces com}
 iinterface2 = interface
  procedure proc2;
 end;

 iinterface3 = interface(iinterface1)
  procedure proc3;
 end;
 iinterface4 = interface(iinterface2)
  procedure proc4;
 end;

 {$interfaces corba}
 iinterface5 = interface(iinterface1)
  procedure proc5;
 end;
 iinterface6 = interface(iinterface2)
  procedure proc6;
 end;

 tclass1 = class(iinterface1)
  public
   procedure proc1;
 end;

{tclass6 = class(iinterface6)
  public
   procedure proc6;
 end;
}
{ does not compile because it is com style interface:
 inheritedcorba.pas(36,12) Error: No matching implementation for
 interface method "IUnknown.QueryInterface(const TGuid,out <Formal type>):
  LongInt;StdCall" found  ...
}
procedure writeinterfacetype(po: ptypeinfo);
begin
 case po^.kind of
  tkinterfaceraw: if (po^.name<>'iinterface1') and
                  (po^.name<>'iinterface3') and
                  (po^.name<>'iinterface5') then
                  halt(1);
  tkinterface: if (po^.name<>'iinterface2') and
                  (po^.name<>'iinterface4') and
                  (po^.name<>'iinterface6') then
<<<<<<< HEAD
<<<<<<< HEAD
                  halt(2);
  else
    halt(3);
=======
                  halt(1);
  else
    halt(1);
>>>>>>> graemeg/fixes_2_2
=======
                  halt(1);
  else
    halt(1);
>>>>>>> origin/fixes_2_2
 end;
end;

{ tclass1 }

procedure tclass1.proc1;
begin
end;

begin
 writeinterfacetype(typeinfo(iinterface1));
 writeinterfacetype(typeinfo(iinterface2));
 writeinterfacetype(typeinfo(iinterface3));
 writeinterfacetype(typeinfo(iinterface4));
 writeinterfacetype(typeinfo(iinterface5));
 writeinterfacetype(typeinfo(iinterface6));
 writeln('ok');
end.
