{ %maxversion=1.9.4 }

{****************************************************************}
{  CODE GENERATOR TEST PROGRAM                                   }
{  Copyright (c) 2002 Carl Eric Codere                           }
{****************************************************************}
{ NODE TESTED : secondcalln()                                    }
{****************************************************************}
{ PRE-REQUISITES: secondload()                                   }
{                 secondassign()                                 }
{                 secondtypeconv()                               }
{                 secondtryexcept()                              }
{****************************************************************}
{ DEFINES:                                                       }
{            FPC     = Target is FreePascal compiler             }
{****************************************************************}
{ REMARKS: This tests secondcalln(), genentrycode() and          }
{ genexitcode() for standard object with the saveregisters       }
{ calling convention.                                            }
{                                                                }
{****************************************************************}
program tcalobj8;
{$R+}

 const
 { should be defined depending on CPU target }
 {$ifdef cpu68k}
   BIG_INDEX = 8000;
   SMALL_INDEX  = 13;
 {$else}
   BIG_INDEX = 33000;
   SMALL_INDEX = 13;     { value should not be aligned! }
 {$endif}
   RESULT_U8BIT = $55;
   RESULT_U16BIT = 2*RESULT_U8BIT;
   RESULT_S32BIT = $500F0000;
   RESULT_S64BIT = $500F0000;
   RESULT_S32REAL = 1777.12;
   RESULT_S64REAL = 3444.24;
   RESULT_BOOL8BIT = 1;
   RESULT_BOOL16BIT = 1;
   RESULT_BOOL32BIT = 1;
   RESULT_PCHAR = 'Hello world';
   RESULT_BIGSTRING = 'Hello world';
   RESULT_SMALLSTRING = 'H';
   RESULT_CHAR = 'I';
   RESULT_BOOLEAN = TRUE;

 type

   tprocedure = procedure;

   tsmallrecord = packed record
     b: byte;
     w: word;
   end;

   tlargerecord = packed record
     b: array[1..BIG_INDEX] of byte;
   end;

   tsmallarray = packed array[1..SMALL_INDEX] of byte;

   tsmallsetenum =
   (A_A,A_B,A_C,A_D);

   tsmallset = set of tsmallsetenum;
   tlargeset = set of char;

   tsmallstring = string[2];


 var
  global_u8bit : byte;
  global_u16bit : word;
  global_s32bit : longint;
  global_s32real : single;
  global_s64real : double;
  global_ptr : pchar;
  global_proc : tprocedure;
  global_bigstring : shortstring;
  global_boolean : boolean;
  global_char : char;
  global_s64bit : int64;
  value_s64bit : int64;
  value_ansistring : ansistring;
  value_u8bit : byte;
  value_u16bit : word;
  value_s32bit : longint;
  value_s32real : single;
  value_s64real  : double;
  value_proc : tprocedure;
  value_ptr : pchar;
  value_smallrec : tsmallrecord;
  value_largerec : tlargerecord;
  value_smallset : tsmallset;
  value_smallstring : tsmallstring;
  value_bigstring   : shortstring;
  value_largeset : tlargeset;
  value_smallarray : tsmallarray;
  value_boolean : boolean;
  value_char : char;

     procedure fail;
     begin
       WriteLn('Failure.');
       halt(1);
     end;


     procedure clear_globals;
      begin
       global_u8bit := 0;
       global_u16bit := 0;
       global_s32bit := 0;
       global_s32real := 0.0;
       global_s64real := 0.0;
       global_ptr := nil;
       global_proc := nil;
       global_bigstring := '';
       global_boolean := false;
       global_char := #0;
       global_s64bit := 0;
      end;


     procedure clear_values;
      begin
       value_u8bit := 0;
       value_u16bit := 0;
       value_s32bit := 0;
       value_s32real := 0.0;
       value_s64real  := 0.0;
       value_proc := nil;
       value_ptr := nil;
       fillchar(value_smallrec, sizeof(value_smallrec), #0);
       fillchar(value_largerec, sizeof(value_largerec), #0);
       value_smallset := [];
       value_smallstring := '';
       value_bigstring   := '';
       value_largeset := [];
       fillchar(value_smallarray, sizeof(value_smallarray), #0);
       value_boolean := false;
       value_char:=#0;
       value_ansistring := '';
       value_s64bit := 0;
      end;


      function getu8: byte;
       begin
         getu8 := RESULT_U8BIT;
       end;


type

 { object without vmt }
 pnovmtobject = ^tnovmtobject;
 tnovmtobject = object
 public
   object_bigstring : shortstring;
   object_u16bit : word;
   { no parameter testing }
   procedure method_public_none;saveregisters;
   procedure method_public_static_none; static;saveregisters;
   procedure method_call_private_none;saveregisters;
   procedure method_call_private_static_none; static;saveregisters;
   { simple value parameter testing }
   procedure method_public_u8(x : byte);saveregisters;
   procedure method_public_static_u8(x: byte); static;saveregisters;
   procedure method_call_private_u8(x: byte);saveregisters;
   procedure method_call_private_static_u8(x: byte); static;saveregisters;
   function  func_array_mixed_nested(b: byte): tsmallarray;saveregisters;
 private
   procedure method_private_none;saveregisters;
   procedure method_private_static_none; static;saveregisters;
   function func_getu16bit : word;saveregisters;
   { simple value parameter testing }
   procedure method_private_u8(x: byte);saveregisters;
   procedure method_private_static_u8(x: byte); static;saveregisters;
 end;


 { object with vmt }
 pvmtobject = ^tvmtobject;
 tvmtobject = object
 public
   object_u8bit : byte;
   object_u16bit : word;
   object_bigstring : shortstring;
   object_s32bit : longint;
   object_s64bit : int64;
   constructor constructor_params_mixed(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);
   constructor constructor_init;
   destructor destructor_params_done;
   procedure method_normal_params_mixed(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   procedure method_virtual_params_mixed(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_virtual_overridden_params_mixed(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_static_params_mixed(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);static;saveregisters;
   procedure method_normal_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;

   { virtual methods which call other methods }
   procedure method_virtual_call_static_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_virtual_call_virtual_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_virtual_call_overridden_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_virtual_call_normal_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_virtual_call_constructor_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;
   procedure method_virtual_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;

 end;

 pheritedvmtobject = ^theritedvmtobject;
 theritedvmtobject = object(tvmtobject)
   constructor constructor_params_mixed_call_virtual(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);
   constructor constructor_params_mixed_call_overridden(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);
   constructor constructor_params_mixed_call_static(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);
   constructor constructor_params_mixed_call_normal(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);
   constructor constructor_params_mixed_call_inherited(u8 :byte; u16: word;
      bigstring: shortstring; s32: longint; s64: int64);
   procedure method_virtual_overridden_params_mixed(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;

   { normal methods which call other methods }
   procedure method_normal_call_static_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   procedure method_normal_call_virtual_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   procedure method_normal_call_overridden_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   procedure method_normal_call_normal_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   procedure method_normal_call_constructor_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   procedure method_normal_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;

   { virtual methods which call other methods }
   procedure method_virtual_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);virtual;saveregisters;

 end;

 pfailvmtobject = ^tfailvmtobject;
 tfailvmtobject = object(tvmtobject)
 public
    constructor constructor_public_none;
 end;



{**************************************************************************}
{                             NO VMT OBJECT                                }
{**************************************************************************}

  {****************** NO PARAMETERS ******************}
 procedure tnovmtobject.method_public_none;saveregisters;
  begin
    global_u8bit := RESULT_U8BIT;
  end;


 procedure tnovmtobject.method_public_static_none;saveregisters;
  begin
    global_u8bit := RESULT_U8BIT;
  end;


 procedure tnovmtobject.method_call_private_none;saveregisters;
   begin
       method_private_none;
       method_private_static_none;
   end;

 procedure tnovmtobject.method_call_private_static_none;saveregisters;
   begin
     method_private_static_none;
   end;


 procedure tnovmtobject.method_private_none;saveregisters;
  begin
    Inc(global_u16bit, RESULT_U8BIT);
  end;


 procedure tnovmtobject.method_private_static_none;saveregisters;
  begin
    Inc(global_u16bit, RESULT_U8BIT);
  end;

  {******************** PARAMETERS ******************}

  procedure tnovmtobject.method_public_u8(x : byte);saveregisters;
   begin
     global_u8bit := x;
   end;

  procedure tnovmtobject.method_public_static_u8(x: byte);saveregisters;
   begin
     global_u8bit := x;
   end;

  procedure tnovmtobject.method_call_private_u8(x: byte);saveregisters;
   begin
     method_private_static_u8(x);
     method_private_u8(x);
   end;

  procedure tnovmtobject. method_call_private_static_u8(x: byte);saveregisters;
   begin
     method_private_static_u8(x);
   end;

   procedure tnovmtobject.method_private_u8(x: byte);saveregisters;
    begin
      Inc(global_u16bit,x);
    end;

   procedure tnovmtobject.method_private_static_u8(x: byte);saveregisters;
    begin
      Inc(global_u16bit,x);
    end;


  function tnovmtobject.func_getu16bit : word;saveregisters;
   begin
     func_getu16bit := object_u16bit;
   end;

  {
    complex testing, nested field access, with parameters and
    comple return value.

    On exit : global_u8bit := x;
              global_u16bit := object_u16bit (from func_getu16bit);
              global_s32bit :=  RESULT_S32BIT
              global_bigstring := object_bigstring
              global_s64bit := x;
  }
  function tnovmtobject.func_array_mixed_nested(b: byte): tsmallarray;saveregisters;

    procedure nested_one_proc(l: longint);
     begin
       global_u16bit := func_getu16bit;
       global_s32bit := l;
     end;

    procedure nested_two_proc(l : longint);
     begin
       global_s64bit := l;
     end;



   function nested_one_func(level1_b : byte; s: shortstring): byte;
     var
      s1 : shortstring;

      function nested_two_func(level2_b : byte; s :shortstring): byte;
        begin
          nested_two_func:=level2_b;
          global_bigstring := s;
          nested_one_proc(RESULT_S32BIT);
        end;

    begin
      s1:=s;
      nested_one_func := nested_two_func(level1_b,s1);
      nested_two_proc(level1_b);
    end;


 var
  local_b: byte;
  smallarray: tsmallarray;
 begin
  fillchar(smallarray, sizeof(smallarray), #0);
  smallarray[1] := RESULT_U8BIT;
  smallarray[SMALL_INDEX] := RESULT_U8BIT;
  func_array_mixed_nested := smallarray;
  local_b:=b;
  global_u8bit := nested_one_func(local_b, object_bigstring);
 end;

{**************************************************************************}
{                             FAILED OBJECT                                }
{**************************************************************************}
constructor tfailvmtobject.constructor_public_none;
 begin
    { this calls the constructor fail special keyword }
    fail;
 end;

{**************************************************************************}
{                               VMT  OBJECT                                }
{**************************************************************************}
constructor tvmtobject.constructor_params_mixed(u8 :byte; u16: word;
   bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := u8;
   object_u16bit := u16;
   object_bigstring := bigstring;
   object_s32bit := s32;
   object_s64bit := s64;
 end;


constructor tvmtobject.constructor_init;
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
 end;

destructor tvmtobject.destructor_params_done;
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
 end;


procedure tvmtobject.method_normal_params_mixed(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
 begin
   object_u8bit := u8;
   object_u16bit := u16;
   object_bigstring := bigstring;
   object_s32bit := s32;
   object_s64bit := s64;
 end;

procedure tvmtobject.method_virtual_params_mixed(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
 begin
   object_u8bit := u8;
   object_u16bit := u16;
   object_bigstring := bigstring;
   object_s32bit := s32;
   object_s64bit := s64;
 end;

{ this one should be overridden }
procedure tvmtobject.method_virtual_overridden_params_mixed(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
 begin
    RunError(211);
 end;

{ can't access field of instances in static methods }
procedure tvmtobject.method_static_params_mixed(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
 begin
   global_u8bit := u8;
   global_u16bit := u16;
   global_bigstring := bigstring;
   global_s32bit := s32;
   global_s64bit := s64;
 end;

procedure tvmtobject.method_normal_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
  begin
   object_u8bit := u8;
   object_u16bit := u16;
   object_bigstring := bigstring;
   object_s32bit := s32;
   object_s64bit := s64;
  end;


procedure tvmtobject.method_virtual_call_static_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
  begin
    method_static_params_mixed(u8, u16, bigstring, s32, s64);
  end;

procedure tvmtobject.method_virtual_call_virtual_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
    method_virtual_params_mixed(u8, u16, bigstring, s32, s64);
   end;

procedure tvmtobject.method_virtual_call_overridden_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
    method_virtual_overridden_params_mixed(u8, u16, bigstring, s32, s64);
   end;


procedure tvmtobject.method_virtual_call_normal_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
    method_normal_params_mixed(u8, u16, bigstring, s32, s64);
   end;

procedure tvmtobject.method_virtual_call_constructor_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
     constructor_params_mixed(u8, u16, bigstring, s32, s64);
   end;

procedure tvmtobject.method_virtual_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
  begin
   object_u8bit := u8;
   object_u16bit := u16;
   object_bigstring := bigstring;
   object_s32bit := s32;
   object_s64bit := s64;
  end;


{**************************************************************************}
{                          INHERITED VMT OBJECT                            }
{**************************************************************************}
constructor theritedvmtobject.constructor_params_mixed_call_virtual(
   u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
   method_virtual_params_mixed(u8, u16, bigstring, s32, s64);
 end;

constructor theritedvmtobject.constructor_params_mixed_call_overridden(
   u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
   method_virtual_overridden_params_mixed(u8, u16, bigstring, s32, s64);
 end;

constructor theritedvmtobject.constructor_params_mixed_call_static(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
   method_static_params_mixed(u8, u16, bigstring, s32, s64);
 end;

constructor theritedvmtobject.constructor_params_mixed_call_normal(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
   method_normal_params_mixed(u8, u16, bigstring, s32, s64);
 end;

constructor theritedvmtobject.constructor_params_mixed_call_inherited
   (u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := 0;
   object_u16bit := 0;
   object_bigstring := '';
   object_s32bit := 0;
   object_s64bit := 0;
   inherited constructor_params_mixed(u8, u16, bigstring, s32, s64);
 end;

{ this one should be overridden }
procedure theritedvmtobject.method_virtual_overridden_params_mixed(
    u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);
 begin
   object_u8bit := u8;
   object_u16bit := u16;
   object_bigstring := bigstring;
   object_s32bit := s32;
   object_s64bit := s64;
 end;

procedure theritedvmtobject.method_normal_call_static_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
  begin
    method_static_params_mixed(u8, u16, bigstring, s32, s64);
  end;

procedure theritedvmtobject.method_normal_call_virtual_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
    method_virtual_params_mixed(u8, u16, bigstring, s32, s64);
   end;

procedure theritedvmtobject.method_normal_call_overridden_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
    method_virtual_overridden_params_mixed(u8, u16, bigstring, s32, s64);
   end;


procedure theritedvmtobject.method_normal_call_normal_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
    method_normal_params_mixed(u8, u16, bigstring, s32, s64);
   end;

procedure theritedvmtobject.method_normal_call_constructor_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
   begin
     constructor_params_mixed(u8, u16, bigstring, s32, s64);
   end;

procedure theritedvmtobject.method_normal_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
  begin
   Inherited method_normal_call_inherited_params_mixed(u8, u16, bigstring,
     s32, s64);
  end;

procedure theritedvmtobject.method_virtual_call_inherited_params_mixed(
      u8 :byte; u16: word; bigstring: shortstring; s32: longint; s64: int64);saveregisters;
  begin
   Inherited method_virtual_call_inherited_params_mixed(u8, u16, bigstring,
     s32, s64);
  end;


procedure testnovmtobject;
var
  novmtobject : tnovmtobject;
  failed : boolean;
begin
  {******************** STATIC / METHOD SIMPLE CALL **********************}
  Write('No parameter / method call testing...');
  failed := false;

  clear_globals;
  clear_values;

  tnovmtobject.method_public_static_none;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  novmtobject.method_public_static_none;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  tnovmtobject.method_call_private_static_none;
  if global_u16bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_call_private_static_none;
  if global_u16bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_public_none;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_call_private_none;
  if global_u16bit <> (RESULT_U16BIT) then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');

  Write('Simple parameter (LOC_CONSTANT) / method call testing...');
  failed := false;

  clear_globals;
  clear_values;

  { parameter is LOC_CONSTANT }
  novmtobject.method_public_u8(RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  tnovmtobject.method_public_static_u8(RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_public_static_u8(RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_call_private_u8(RESULT_U8BIT);
  if global_u16bit <> (RESULT_U16BIT) then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_call_private_static_u8(RESULT_U8BIT);
  if global_u16bit <> (RESULT_U8BIT) then
    failed := true;


  if failed then
    fail
  else
    WriteLn('Passed!');


  Write('Simple parameter (LOC_REFERENCE) / method call testing...');
  failed := false;

  clear_globals;
  clear_values;

  value_u8bit := RESULT_U8BIT;
  novmtobject.method_public_u8(value_u8bit);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  value_u8bit := RESULT_U8BIT;
  tnovmtobject.method_public_static_u8(value_u8bit);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  value_u8bit := RESULT_U8BIT;
  novmtobject.method_public_static_u8(value_u8bit);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  value_u8bit := RESULT_U8BIT;
  novmtobject.method_call_private_u8(value_u8bit);
  if global_u16bit <> (RESULT_U16BIT) then
    failed := true;

  clear_globals;
  clear_values;

  value_u8bit := RESULT_U8BIT;
  novmtobject.method_call_private_static_u8(value_u8bit);
  if global_u16bit <> (RESULT_U8BIT) then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');

  Write('Simple parameter (LOC_REGISTER) / method call testing...');
  failed := false;

  clear_globals;
  clear_values;

  novmtobject.method_public_u8(getu8);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  tnovmtobject.method_public_static_u8(getu8);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_public_static_u8(getu8);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_call_private_u8(getu8);
  if global_u16bit <> (RESULT_U16BIT) then
    failed := true;

  clear_globals;
  clear_values;

  novmtobject.method_call_private_static_u8(getu8);
  if global_u16bit <> (RESULT_U8BIT) then
    failed := true;

 if failed then
   fail
 else
   WriteLn('Passed!');

  Write('Simple parameter / complex return / nested method access testing...');

  clear_globals;
  clear_values;
  failed := false;
  novmtobject.object_bigstring := RESULT_BIGSTRING;
  novmtobject.object_u16bit := RESULT_U16BIT;

  value_smallarray := novmtobject.func_array_mixed_nested(RESULT_U8BIT);
  if (value_smallarray[1] <> RESULT_U8BIT) or (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) then
    failed := true;
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if global_bigstring <> RESULT_BIGSTRING then
    failed := true;
  if global_u16bit <> RESULT_U16BIT then
    failed := true;
  if global_s32bit <> RESULT_S32BIT then
    failed := true;
  if global_s64bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');
end;


procedure testfailedobject;
var
  failedobject : tfailvmtobject;
 begin
  Write('Testing constructor return value...');
  if failedobject.constructor_public_none then
    fail
  else
    Writeln('Passed!');
 end;


 procedure testvmtobject;
  var
   vmtobject : tvmtobject;
   failed : boolean;
  begin

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_CONSTANT) constructor call...');
    vmtobject.constructor_params_mixed(RESULT_U8BIT, RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_REFERENCE) constructor call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.constructor_params_mixed(value_u8bit, value_u16bit, value_bigstring,
       value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

  end;


 procedure testheritedvmtobject;
  var
   vmtobject : theritedvmtobject;
   failed : boolean;
  begin
    {********************** CONSTRUCTOR TESTING ************************}
    {********************** DESTRUCTOR  TESTING ************************}
    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_CONSTANT) inherited constructor call...');
    vmtobject.constructor_params_mixed_call_inherited(RESULT_U8BIT, RESULT_U16BIT, RESULT_BIGSTRING,
       RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_REFERENCE) inherited constructor call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.constructor_params_mixed_call_inherited(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/virtual call...');
    vmtobject.constructor_params_mixed_call_virtual(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/virtual call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.constructor_params_mixed_call_virtual(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/virtual call...');
    vmtobject.constructor_params_mixed_call_overridden(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/virtual call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.constructor_params_mixed_call_overridden(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/method call...');
    vmtobject.constructor_params_mixed_call_normal(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/method call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.constructor_params_mixed_call_normal(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/static call...');
    vmtobject.constructor_params_mixed_call_static(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/static call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.constructor_params_mixed_call_static(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    {************************* METHOD TESTING **************************}
    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual method call...');
    vmtobject.method_virtual_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual method call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual method call...');
    vmtobject.method_virtual_overridden_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual method call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_overridden_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call...');
    vmtobject.method_normal_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) static method call...');
    vmtobject.method_static_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) static method call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_static_params_mixed(value_u8bit,
      value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    { ********************************************************************
      This calls methods which in turn call other methods, or a constructor
      or a destructor.
      *********************************************************************
    }
    clear_globals;
    clear_values;
    failed := false;
    { Calls the ancestor virtual method }
    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call w/virtual call...');
    vmtobject.method_normal_call_virtual_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call w/virtual call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_call_virtual_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    { The virtual method has been overridden by the object in this case }
    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call w/virtual call...');
    vmtobject.method_normal_call_overridden_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call w/virtual call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_call_overridden_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call w/normal call...');
    vmtobject.method_normal_call_normal_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call w/normal call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_call_normal_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    (* constructor call inside a normal method *)

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call w/constructor call...');
    vmtobject.method_normal_call_constructor_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call w/constructor call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_call_constructor_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    { static method call }
    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call w/static call...');
    vmtobject.method_normal_call_static_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call w/static call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_call_static_params_mixed(value_u8bit,
      value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    (* calls the inherited method *)
    clear_globals;
    clear_values;
    failed := false;
    { Calls the ancestor virtual method }
    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) method call w/inherited call...');
    vmtobject.method_normal_call_inherited_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) method call w/inherited call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_normal_call_inherited_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

 { ********************************************************************
      This calls virtual methods which in turn call other methods,
      or a constructor  or a destructor.
   *********************************************************************
    }
    clear_globals;
    clear_values;
    failed := false;
    { Calls the ancestor virtual method }
    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/virtual call...');
    vmtobject.method_virtual_call_virtual_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/virtual call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_call_virtual_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    { The virtual method has been overridden by the object in this case }
    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/virtual call...');
    vmtobject.method_virtual_call_overridden_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/virtual call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_call_overridden_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/normal call...');
    vmtobject.method_virtual_call_normal_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/normal call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_call_normal_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    (* constructor call inside a normal method *)

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/constructor call...');
    vmtobject.method_virtual_call_constructor_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/constructor call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_call_constructor_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    { static virtual call }
    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/static call...');
    vmtobject.method_virtual_call_static_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/static call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_call_static_params_mixed(value_u8bit,
      value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if global_u8bit <> RESULT_U8BIT then
      failed := true;
    if global_u16bit <> RESULT_U16BIT then
      failed := true;
    if global_s32bit <> RESULT_S32BIT then
      failed := true;
    if global_s64bit <> RESULT_S64BIT then
      failed := true;
    if global_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    (* calls the inherited method *)
    clear_globals;
    clear_values;
    failed := false;
    { Calls the ancestor virtual method }
    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/inherited call...');
    vmtobject.method_virtual_call_inherited_params_mixed(RESULT_U8BIT,
       RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');

    clear_globals;
    clear_values;
    failed := false;

    vmtobject.constructor_init;
    Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/inherited call...');
    value_u8bit := RESULT_U8BIT;
    value_u16bit := RESULT_U16BIT;
    value_bigstring := RESULT_BIGSTRING;
    value_s32bit := RESULT_S32BIT;
    value_s64bit := RESULT_S64BIT;
    vmtobject.method_virtual_call_inherited_params_mixed(value_u8bit,
       value_u16bit, value_bigstring, value_s32bit, value_s64bit);
    if vmtobject.object_u8bit <> RESULT_U8BIT then
      failed := true;
    if vmtobject.object_u16bit <> RESULT_U16BIT then
      failed := true;
    if vmtobject.object_s32bit <> RESULT_S32BIT then
      failed := true;
    if vmtobject.object_s64bit <> RESULT_S64BIT then
      failed := true;
    if vmtobject.object_bigstring <> RESULT_BIGSTRING then
      failed := true;
    vmtobject.destructor_params_done;

    if failed then
      fail
    else
      Writeln('Passed!');


  end;


  { same as testvmtherited, except uses with statement }
  procedure testwith;
  var
   vmtobject : theritedvmtobject;
   failed : boolean;
  begin
    with vmtobject do
     begin
        {********************** CONSTRUCTOR TESTING ************************}
        {********************** DESTRUCTOR  TESTING ************************}
        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_CONSTANT) inherited constructor call...');
        constructor_params_mixed_call_inherited(RESULT_U8BIT, RESULT_U16BIT, RESULT_BIGSTRING,
           RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_REFERENCE) inherited constructor call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        constructor_params_mixed_call_inherited(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/virtual call...');
        constructor_params_mixed_call_virtual(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/virtual call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        constructor_params_mixed_call_virtual(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/virtual call...');
        constructor_params_mixed_call_overridden(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/virtual call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        constructor_params_mixed_call_overridden(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/method call...');
        constructor_params_mixed_call_normal(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/method call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        constructor_params_mixed_call_normal(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_CONSTANT) constructor call w/static call...');
        constructor_params_mixed_call_static(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        Write('Testing mixed parameter (LOC_REFERENCE) constructor call w/static call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        constructor_params_mixed_call_static(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        {************************* METHOD TESTING **************************}
        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual method call...');
        method_virtual_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual method call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual method call...');
        method_virtual_overridden_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual method call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_overridden_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call...');
        method_normal_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) static method call...');
        method_static_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) static method call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_static_params_mixed(value_u8bit,
          value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        { ********************************************************************
          This calls methods which in turn call other methods, or a constructor
          or a destructor.
          *********************************************************************
        }
        clear_globals;
        clear_values;
        failed := false;
        { Calls the ancestor virtual method }
        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call w/virtual call...');
        method_normal_call_virtual_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call w/virtual call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_call_virtual_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        { The virtual method has been overridden by the object in this case }
        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call w/virtual call...');
        method_normal_call_overridden_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call w/virtual call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_call_overridden_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call w/normal call...');
        method_normal_call_normal_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call w/normal call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_call_normal_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        (* constructor call inside a normal method *)

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call w/constructor call...');
        method_normal_call_constructor_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call w/constructor call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_call_constructor_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        { static method call }
        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call w/static call...');
        method_normal_call_static_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call w/static call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_call_static_params_mixed(value_u8bit,
          value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        (* calls the inherited method *)
        clear_globals;
        clear_values;
        failed := false;
        { Calls the ancestor virtual method }
        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) method call w/inherited call...');
        method_normal_call_inherited_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) method call w/inherited call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_normal_call_inherited_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

    { ********************************************************************
        This calls virtual methods which in turn call other methods,
        or a constructor  or a destructor.
       *********************************************************************
    }
        clear_globals;
        clear_values;
        failed := false;
        { Calls the ancestor virtual method }
        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/virtual call...');
        method_virtual_call_virtual_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/virtual call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_call_virtual_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        { The virtual method has been overridden by the object in this case }
        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/virtual call...');
        method_virtual_call_overridden_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/virtual call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_call_overridden_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/normal call...');
        method_virtual_call_normal_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/normal call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_call_normal_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        (* constructor call inside a normal method *)

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/constructor call...');
        method_virtual_call_constructor_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/constructor call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_call_constructor_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        { static virtual call }
        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/static call...');
        method_virtual_call_static_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/static call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_call_static_params_mixed(value_u8bit,
          value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if global_u8bit <> RESULT_U8BIT then
          failed := true;
        if global_u16bit <> RESULT_U16BIT then
          failed := true;
        if global_s32bit <> RESULT_S32BIT then
          failed := true;
        if global_s64bit <> RESULT_S64BIT then
          failed := true;
        if global_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        (* calls the inherited method *)
        clear_globals;
        clear_values;
        failed := false;
        { Calls the ancestor virtual method }
        constructor_init;
        Write('Testing mixed parameter (LOC_CONSTANT) virtual call w/inherited call...');
        method_virtual_call_inherited_params_mixed(RESULT_U8BIT,
           RESULT_U16BIT, RESULT_BIGSTRING, RESULT_S32BIT, RESULT_S64BIT);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');

        clear_globals;
        clear_values;
        failed := false;

        constructor_init;
        Write('Testing mixed parameter (LOC_REFERENCE) virtual call w/inherited call...');
        value_u8bit := RESULT_U8BIT;
        value_u16bit := RESULT_U16BIT;
        value_bigstring := RESULT_BIGSTRING;
        value_s32bit := RESULT_S32BIT;
        value_s64bit := RESULT_S64BIT;
        method_virtual_call_inherited_params_mixed(value_u8bit,
           value_u16bit, value_bigstring, value_s32bit, value_s64bit);
        if object_u8bit <> RESULT_U8BIT then
          failed := true;
        if object_u16bit <> RESULT_U16BIT then
          failed := true;
        if object_s32bit <> RESULT_S32BIT then
          failed := true;
        if object_s64bit <> RESULT_S64BIT then
          failed := true;
        if object_bigstring <> RESULT_BIGSTRING then
          failed := true;
        destructor_params_done;

        if failed then
          fail
        else
          Writeln('Passed!');
   end; { end with }
  end;


begin
  WriteLN('*********************** NO VMT OBJECT TESTS ********************');
  testnovmtobject;
  WriteLN('************************ VMT OBJECT FAIL  **********************');
  testfailedobject;
  WriteLN('************************* VMT OBJECT TESTS *********************');
  testvmtobject;
  testheritedvmtobject;
  WriteLN('**************** VMT OBJECT TESTS USING WITH *******************');
  testwith;
end.
