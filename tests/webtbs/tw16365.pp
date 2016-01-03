program delegation;
{$ifdef FPC}{$mode objfpc}{$h+}{$endif}
{$ifdef mswindows}{$apptype console}{$endif}
uses
 sysutils;

type
 itest = interface
  function test: longint;
 end;
 
 timpclass = class(tobject,itest)
  protected
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   function _addref: longint; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
   function _release: longint; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
   function _addref: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
   function _release: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
   function _addref: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
   function _release: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
   function _addref: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
   function _release: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
   function _addref: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
   function _release: integer; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> origin/cpstrnew
   function QueryInterface(constref IID: TGUID; out Obj): HResult; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
  public
   function test: longint;
 end;
 
 ttestclass = class(tobject,itest)
  private
   fimp: timpclass;
   property imp: timpclass read fimp implements itest;
  public
   constructor create;
   destructor destroy; override;
 end;

{ timpclass }

function timpclass.test: longint;
begin
 writeln('test');
 result:=123456;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function timpclass._addref: longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
function timpclass._addref: integer; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function timpclass._addref: integer; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function timpclass._addref: integer; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function timpclass._addref: integer; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> origin/cpstrnew
begin
 result:= -1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function timpclass._release: longint;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
function timpclass._release: integer;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function timpclass._release: integer;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function timpclass._release: integer;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function timpclass._release: integer;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> origin/cpstrnew
begin
 result:= -1;
end;

function timpclass.QueryInterface(constref IID: TGUID; out Obj): HResult;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
begin
 if GetInterface(IID, Obj) then begin
   Result:=0
 end
 else begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  result:= HResult(e_nointerface);
=======
  result:= integer(e_nointerface);
>>>>>>> graemeg/cpstrnew
=======
  result:= integer(e_nointerface);
>>>>>>> graemeg/cpstrnew
=======
  result:= integer(e_nointerface);
>>>>>>> graemeg/cpstrnew
=======
  result:= integer(e_nointerface);
>>>>>>> origin/cpstrnew
 end;
end;

{ ttestclass }

constructor ttestclass.create;
begin
 fimp:= timpclass.create;
end;

destructor ttestclass.destroy;
begin
 inherited;
 fimp.free;
end;

var
 testclass: ttestclass;
begin
 testclass:= ttestclass.create;
 if itest(testclass).test<>123456 then  //<<<<---- AV
   halt(1);
 testclass.free;
end.


