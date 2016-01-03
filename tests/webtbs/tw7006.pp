program av;
{$ifdef FPC}{$mode objfpc}{$h+}{$INTERFACES CORBA}{$endif}
{$ifdef mswindows}{$apptype console}{$endif}
uses
<<<<<<< HEAD
<<<<<<< HEAD
 {$ifdef FPC}{$ifdef unix}cthreads,{$ifdef darwin}iosxwstr{$else}cwstring{$endif},{$endif}{$endif}sysutils;
=======
 {$ifdef FPC}{$ifdef unix}cthreads,cwstring,{$endif}{$endif}sysutils;
>>>>>>> graemeg/fixes_2_2
=======
 {$ifdef FPC}{$ifdef unix}cthreads,cwstring,{$endif}{$endif}sysutils;
>>>>>>> origin/fixes_2_2
type
 testrecty = record
  str: widestring;
  int: integer;
 end;

var
 tr1,tr2: testrecty;
 
begin
 tr1.str:= 'abc';
 tr1.int:= 0;
 tr2:= tr1;
end.
