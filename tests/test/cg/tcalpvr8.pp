{ %maxversion=1.9.4 }

{****************************************************************}
{  CODE GENERATOR TEST PROGRAM                                   }
{****************************************************************}
{ NODE TESTED : secondcalln()                                    }
{****************************************************************}
{ PRE-REQUISITES: secondload()                                   }
{                 secondassign()                                 }
{                 secondcalln()                                  }
{                 secondadd()                                    }
{                 secondtypeconv()                               }
{****************************************************************}
{ DEFINES:                                                       }
{****************************************************************}
{ REMARKS: This tests a subset of the secondcalln() , it         }
{          verifies procedural variables for saveregisters       }
{          calling conventions.                                  }
{****************************************************************}
program tcalpvr3;
{$MODE OBJFPC}
{$R+}

const
   RESULT_U8BIT = $55;
   RESULT_U16BIT = $500F;
   RESULT_S32BIT = $500F0000;
   RESULT_S64BIT = -12000;

type

  troutine = procedure (x: longint;  y: byte);saveregisters;
  troutineresult = function (x: longint; y: byte): int64;saveregisters;

  tsimpleobject = object
    constructor init;
    procedure test_normal(x: byte);saveregisters;
    procedure test_static(x: byte);static;saveregisters;
    procedure test_virtual(x: byte);virtual;saveregisters;
  end;

  tsimpleclass = class
    constructor create;
    procedure test_normal(x: byte);saveregisters;
    class procedure test_static(x: byte);saveregisters;
    procedure test_virtual(x: byte);virtual;saveregisters;
  end;

  tobjectmethod = procedure (x: byte) of object ;saveregisters;
  tclassmethod = procedure (x: byte) of object;saveregisters;

var
  proc : troutine;
  func : troutineresult;
  obj_method : tobjectmethod;
  cla_method : tclassmethod;
  global_s32bit : longint;
  global_s64bit : int64;
  global_u8bit : byte;
  value_s32bit : longint;
  value_u8bit : byte;
  obj : tsimpleobject;
  cla : tsimpleclass;




  procedure fail;
   begin
     WriteLn('Failed!');
     halt(1);
   end;

  procedure clear_globals;
   begin
     global_s32bit := 0;
     global_u8bit := 0;
     global_s64bit := 0;
   end;

  procedure clear_values;
    begin
      value_s32bit := 0;
      value_u8bit := 0;
    end;


  procedure testroutine(x: longint; y: byte);saveregisters;
   begin
     global_s32bit := x;
     global_u8bit := y;
   end;

  function testroutineresult(x: longint; y: byte): int64;saveregisters;
   begin
     global_s32bit := x;
     global_u8bit := y;
     testroutineresult := RESULT_S64BIT;
   end;


  function getroutine: troutine;
    begin
      getroutine:=proc;
    end;

  function getroutineresult : troutineresult;
   begin
     getroutineresult := func;
   end;

{ IMPOSSIBLE TO DO CURRENTLY !
  function get_object_method_static : tnormalmethod;
   begin
     get_object_method_static := @obj.test_static;
   end;
}

  { objects access }
  function get_object_method_normal : tobjectmethod;
   begin
     get_object_method_normal := @obj.test_normal;
   end;

  function get_object_type_method_virtual : tobjectmethod;
   begin
     get_object_type_method_virtual := @obj.test_virtual;
   end;

  function get_object_method_virtual : tobjectmethod;
   begin
     get_object_method_virtual := @obj.test_virtual;
   end;

{
  HOW CAN WE GET THIS ADDRESS???
  function get_class_method_static_self : tclassmethodself;
   begin
     get_class_method_static_self := @cla.test_static_self;
   end;
}

  function get_class_method_normal : tclassmethod;
   begin
     get_class_method_normal := @cla.test_normal;
   end;
{
  function get_class_method_static : tclassmethod;
   begin
     get_class_method_static := @cla.test_static;
   end;}

  function get_class_method_virtual : tclassmethod;
   begin
     get_class_method_virtual := @cla.test_virtual;
   end;

 {****************************************************************************************************}

  constructor tsimpleobject.init;
   begin
   end;

  procedure tsimpleobject.test_normal(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;

  procedure tsimpleobject.test_static(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;

  procedure tsimpleobject.test_virtual(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;

 {****************************************************************************************************}
  constructor tsimpleclass.create;
   begin
    inherited create;
   end;

  procedure tsimpleclass. test_normal(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;

  class procedure tsimpleclass.test_static(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;

  procedure tsimpleclass.test_virtual(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;


var
 failed : boolean;
Begin
 { setup variables }
 proc := @testroutine;
 func := @testroutineresult;
 obj.init;
 cla:=tsimpleclass.create;

 {****************************************************************************************************}

 Write('Testing procedure variable call (LOC_REGISTER)..');

 clear_globals;
 clear_values;
 failed := false;

 { parameters in LOC_CONSTANT, routine address in LOC_REGISTER }
 troutine(getroutine)(RESULT_S32BIT,RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;

 clear_globals;
 clear_values;
 { parameters in LOC_REFERENCE,routine address in LOC_REGISTER }
 value_s32bit := RESULT_S32BIT;
 value_u8bit := RESULT_U8BIT;
 troutine(getroutine)(value_s32bit , value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;

 If failed then
   fail
 else
   WriteLn('Passed!');


 Write('Testing procedure variable call (LOC_REFERENCE)..');

 clear_globals;
 clear_values;
 failed := false;

 { parameters in LOC_CONSTANT, routine address in LOC_REGISTER }
 proc(RESULT_S32BIT,RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;

 clear_globals;
 clear_values;
 { parameters in LOC_REFERENCE,routine address in LOC_REGISTER }
 value_s32bit := RESULT_S32BIT;
 value_u8bit := RESULT_U8BIT;
 proc(value_s32bit , value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;

 If failed then
   fail
 else
   WriteLn('Passed!');
 {****************************************************************************************************}
 Write('Testing function variable call (LOC_REGISTER)..');

 clear_globals;
 clear_values;
 failed := false;

 { parameters in LOC_CONSTANT, routine address in LOC_REGISTER }
 global_s64bit := troutineresult(getroutineresult)(RESULT_S32BIT,RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;
 if global_s64bit <> RESULT_S64BIT then
   failed := true;

 clear_globals;
 clear_values;
 { parameters in LOC_REFERENCE,routine address in LOC_REGISTER }
 value_s32bit := RESULT_S32BIT;
 value_u8bit := RESULT_U8BIT;
 global_s64bit := troutineresult(getroutineresult)(value_s32bit , value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;
 if global_s64bit <> RESULT_S64BIT then
   failed := true;

 If failed then
   fail
 else
   WriteLn('Passed!');


 Write('Testing function variable call (LOC_REFERENCE)..');

 clear_globals;
 clear_values;
 failed := false;

 { parameters in LOC_CONSTANT, routine address in LOC_REGISTER }
 global_s64bit := func(RESULT_S32BIT,RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;
 if global_s64bit <> RESULT_S64BIT then
   failed := true;

 clear_globals;
 clear_values;
 { parameters in LOC_REFERENCE,routine address in LOC_REGISTER }
 value_s32bit := RESULT_S32BIT;
 value_u8bit := RESULT_U8BIT;
 global_s64bit := func(value_s32bit , value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;
 if global_s32bit <> RESULT_S32BIT then
   failed := true;
 if global_s64bit <> RESULT_S64BIT then
   failed := true;

 If failed then
   fail
 else
   WriteLn('Passed!');
 {****************************************************************************************************}
 Write('Testing object method variable call (LOC_REGISTER) ..');

 clear_globals;
 clear_values;
 failed := false;

 tobjectmethod(get_object_method_normal)(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 tobjectmethod(get_object_type_method_virtual)(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 tobjectmethod(get_object_method_virtual)(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 value_u8bit := RESULT_U8BIT;
 tobjectmethod(get_object_method_normal)(value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 value_u8bit := RESULT_U8BIT;
 tobjectmethod(get_object_type_method_virtual)(value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 value_u8bit := RESULT_U8BIT;
 tobjectmethod(get_object_method_virtual)(value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;


 If failed then
   fail
 else
   WriteLn('Passed!');

 Write('Testing object method variable call (LOC_REFERENCE) ..');

 clear_globals;
 clear_values;
 failed := false;

 obj_method:=@obj.test_normal;
 obj_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 obj_method:=@obj.test_virtual;
 obj_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 obj_method:=@obj.test_virtual;
 obj_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 value_u8bit := RESULT_U8BIT;
 obj_method:=@obj.test_normal;
 obj_method(value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 value_u8bit := RESULT_U8BIT;
 obj_method:=@obj.test_virtual;
 obj_method(value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 value_u8bit := RESULT_U8BIT;
 obj_method:=@obj.test_normal;
 obj_method(value_u8bit);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;


 If failed then
   fail
 else
   WriteLn('Passed!');

 {****************************************************************************************************}
 Write('Testing class method variable call (LOC_REGISTER) ..');

 clear_globals;
 clear_values;
 failed := false;

 tclassmethod(get_class_method_normal)(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;


 tclassmethod(get_class_method_virtual)(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 If failed then
   fail
 else
   WriteLn('Passed!');

 Write('Testing class method variable call (LOC_REFERENCE)...');

 clear_globals;
 clear_values;
 failed := false;


 cla_method := @cla.test_normal;
 cla_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;


 cla_method := @cla.test_virtual;
 cla_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

 cla_method := @cla.test_virtual;
 cla_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;

 clear_globals;
 clear_values;

{ cla_method := @cla.test_static;
 cla_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;}

 clear_globals;
 clear_values;


{ cla_method := @cla.test_static;
 cla_method(RESULT_U8BIT);
 if global_u8bit <> RESULT_U8BIT then
   failed := true;}

 If failed then
   fail
 else
   WriteLn('Passed!');

end.
