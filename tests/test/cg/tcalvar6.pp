{****************************************************************}
{  CODE GENERATOR TEST PROGRAM                                   }
{  By Carl Eric Codere                                           }
{****************************************************************}
{ NODE TESTED : secondcallparan()                                }
{****************************************************************}
{ PRE-REQUISITES: secondload()                                   }
{                 secondassign()                                 }
{                 secondtypeconv()                               }
{                 secondtryexcept()                              }
{                 secondcalln()                                  }
{                 secondadd()                                    }
{****************************************************************}
{ DEFINES:                                                       }
{            FPC     = Target is FreePascal compiler             }
{****************************************************************}
{ REMARKS: This tests a subset of the secondcalln() node         }
{          (var   parameters with safecall calling convention)   }
{****************************************************************}
program tcalvar6;
{$ifdef fpc}
  {$mode objfpc}
  {$INLINE ON}
{$endif}
{$R+}
{$P-}
{$V+}

{$ifdef VER70}
  {$define tp}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ On linux/i386 and linux/x86-64 afecall is the same as cdecl, so it does not       }
{ support all parameter types.                                      }
{$if ((defined(linux) or defined(android)) and (defined(cpui386) or defined(cpux86_64)))}
  {$define safecall_is_cdecl}
{$endif}

{$ifdef cpu68k}
  {$define cpusmall}
{$endif}
{$ifdef cpu8086}
  {$define cpusmall}
{$endif}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ On linux/i386 safecall is the same as cdecl, so it does not       }
{ support all parameter types.                                      }
{$if (defined(linux) and defined(cpui386))}
  {$define safecall_is_cdecl}
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

 { REAL should map to single or double }
 { so it is not checked, since single  }
 { double nodes are checked.           }

 { assumes that enumdef is the same as orddef (same storage format) }

 const
{ should be defined depending on CPU target }
{$ifdef fpc}
  {$ifdef cpusmall}
    BIG_INDEX = 8000;
    SMALL_INDEX  = 13;
  {$else}
    BIG_INDEX = 33000;
    SMALL_INDEX = 13;     { value should not be aligned! }
  {$endif}
{$else}
  BIG_INDEX = 33000;
  SMALL_INDEX = 13;     { value should not be aligned! }
{$endif}
  RESULT_U8BIT = $55;
  RESULT_U16BIT = $500F;
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
{$ifdef fpc}
  pbytearr=^byte;
{$else}
  pbytearr=^tbytearr;
  tbytearr=array[0..$fffffff] of byte;
{$endif}

  tclass1 = class
  end;

  tprocedure = procedure;

  tsmallrecord =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
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
 global_s64bit : int64;
 global_s32real : single;
 global_s64real : double;
 global_ptr : pchar;
 global_proc : tprocedure;
 global_class : tclass1;
 global_bigstring : shortstring;
 global_boolean : boolean;
 global_char : char;
 value_u8bit : byte;
 value_u16bit : word;
 value_s32bit : longint;
 value_s64bit : int64;
 value_s32real : single;
 value_s64real  : double;
 value_proc : tprocedure;
 value_ptr : pchar;
 value_class : tclass1;
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
      global_s64bit := 0;
      global_s32real := 0.0;
      global_s64real := 0.0;
      global_ptr := nil;
      global_proc := nil;
      global_class := nil;
      global_bigstring := '';
      global_boolean := false;
      global_char := #0;
     end;


    procedure clear_values;
     begin
      value_u8bit := 0;
      value_u16bit := 0;
      value_s32bit := 0;
      value_s64bit := 0;
      value_s32real := 0.0;
      value_s64real  := 0.0;
      value_proc := nil;
      value_ptr := nil;
      value_class := nil;
      fillchar(value_smallrec, sizeof(value_smallrec), #0);
      fillchar(value_largerec, sizeof(value_largerec), #0);
      value_smallset := [];
      value_smallstring := '';
      value_bigstring   := '';
      value_largeset := [];
      fillchar(value_smallarray, sizeof(value_smallarray), #0);
      value_boolean := false;
      value_char:=#0;
     end;


  procedure testprocedure;
   begin
   end;

   function getu8bit : byte;
    begin
      getu8bit:=RESULT_U8BIT;
    end;

   function getu16bit: word;
     begin
       getu16bit:=RESULT_U16BIT;
     end;

   function gets32bit: longint;
    begin
      gets32bit:=RESULT_S32BIT;
    end;

   function gets64bit: int64;
    begin
      gets64bit:=RESULT_S64BIT;
    end;


   function gets32real: single;
    begin
      gets32real:=RESULT_S32REAL;
    end;

   function gets64real: double;
    begin
      gets64real:=RESULT_S64REAL;
    end;

  {************************************************************************}
  {                           VAR   PARAMETERS                             }
  {************************************************************************}
  procedure proc_var_s32bit(var v : longint);safecall;
   begin
     v:=RESULT_S32BIT;
   end;

  procedure proc_var_s64bit(var v: int64);safecall;
   begin
     v:=RESULT_S64BIT;
   end;


  procedure proc_var_u8bit(var v: byte);safecall;
   begin
     v:=RESULT_U8BIT;
   end;

  procedure proc_var_smallrecord(var smallrec : tsmallrecord);safecall;
   begin
     smallrec.b := RESULT_U8BIT;
     smallrec.w := RESULT_U16BIT;
   end;


  procedure proc_var_largerecord(var largerec : tlargerecord);safecall;
   begin
     largerec.b[1] := RESULT_U8BIT;
     largerec.b[2] := RESULT_U8BIT;
   end;


  procedure proc_var_smallset(var smallset : tsmallset);safecall;
   begin
     smallset := [A_A,A_D];
   end;


  procedure proc_var_largeset(var largeset : tlargeset);safecall;
   begin
     largeset:= largeset + ['I'];
   end;


  procedure proc_var_smallstring(var s:tsmallstring);safecall;
   begin
     s:=RESULT_SMALLSTRING;
   end;


  procedure proc_var_bigstring(var s:shortstring);safecall;
   begin
     s:=RESULT_BIGSTRING;
   end;


  procedure proc_var_openstring(var s: OpenString);{$ifndef safecall_is_cdecl} safecall; {$endif}
   begin
    global_u8bit := high(s);
    s:=RESULT_SMALLSTRING;
   end;

  procedure proc_var_smallarray(var arr : tsmallarray);safecall;
  begin
    arr[SMALL_INDEX] := RESULT_U8BIT;
    arr[1] := RESULT_U8BIT;
  end;

  procedure proc_var_smallarray_open(var arr : array of byte);{$ifndef safecall_is_cdecl} safecall; {$endif}
  begin
    arr[high(arr)] := RESULT_U8BIT;
    arr[low(arr)] := RESULT_U8BIT;
  end;

  procedure proc_var_smallarray_const_1(var arr : array of const);{$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
    for i:=0 to high(arr) do
     begin
       case arr[i].vtype of
        vtInteger : arr[i].vinteger := RESULT_U8BIT;
        vtBoolean : arr[i].vboolean := RESULT_BOOLEAN;
        else
          RunError(255);
       end;
     end; {endfor}
  end;


  procedure proc_var_smallarray_const_2(var arr : array of const);{$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
     if high(arr)<0 then
       global_u8bit := RESULT_U8BIT;
  end;


  procedure proc_var_formaldef_array(var buf);safecall;
  var
   p: pbytearr;
  begin
    { array is indexed from 1 }
    p := @buf;
    p[SMALL_INDEX-1] := RESULT_U8BIT;
    p[0] := RESULT_U8BIT;
  end;


procedure proc_var_formaldef_string(var buf);safecall;
  var
   p: pbytearr;
  begin
    { array is indexed from 1 }
    p := @buf;
    p[SMALL_INDEX-1] := RESULT_U8BIT;
    p[0] := RESULT_U8BIT;
  end;


  {************************************************************************}
  {                     MIXED   VAR PARAMETERS                             }
  {************************************************************************}
  procedure proc_var_s32bit_mixed(b1 : byte;var v : longint; b2: byte);safecall;
   begin
     v:=RESULT_S32BIT;
     value_u8bit := RESULT_U8BIT;
   end;

  procedure proc_var_s64bit_mixed(b1 : byte;var v: int64; b2: byte);safecall;
   begin
     v:=RESULT_S64BIT;
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_u8bit_mixed(b1 : byte;var v: byte; b2: byte);safecall;
   begin
     v:=RESULT_U8BIT;
     value_u8bit := RESULT_U8BIT;
   end;

  procedure proc_var_smallrecord_mixed(b1 : byte; var smallrec : tsmallrecord; b2: byte);safecall;
   begin
     smallrec.b := RESULT_U8BIT;
     smallrec.w := RESULT_U16BIT;
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_largerecord_mixed(b1 : byte; var largerec : tlargerecord; b2: byte);safecall;
   begin
     largerec.b[1] := RESULT_U8BIT;
     largerec.b[2] := RESULT_U8BIT;
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_smallset_mixed(b1 : byte; var smallset : tsmallset; b2: byte);safecall;
   begin
     smallset := [A_A,A_D];
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_largeset_mixed(b1 : byte; var largeset : tlargeset; b2: byte);safecall;
   begin
     largeset:= largeset + ['I'];
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_smallstring_mixed(b1 : byte; var s:tsmallstring; b2: byte);safecall;
   begin
     s:=RESULT_SMALLSTRING;
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_bigstring_mixed(b1 : byte; var s:shortstring; b2: byte);safecall;
   begin
     s:=RESULT_BIGSTRING;
     value_u8bit := RESULT_U8BIT;
   end;


  procedure proc_var_openstring_mixed(b1 : byte; var s: OpenString; b2: byte);{$ifndef safecall_is_cdecl} safecall; {$endif}
   begin
    global_u8bit := high(s);
    s:=RESULT_SMALLSTRING;
    value_u8bit := RESULT_U8BIT;
   end;

  procedure proc_var_smallarray_mixed(b1 : byte; var arr : tsmallarray; b2: byte);safecall;
  begin
    arr[SMALL_INDEX] := RESULT_U8BIT;
    arr[1] := RESULT_U8BIT;
    value_u8bit := RESULT_U8BIT;
  end;

  procedure proc_var_smallarray_open_mixed(b1 : byte; var arr : array of byte; b2: byte);{$ifndef safecall_is_cdecl} safecall; {$endif}
  begin
    arr[high(arr)] := RESULT_U8BIT;
    arr[low(arr)] := RESULT_U8BIT;
    value_u8bit := RESULT_U8BIT;
  end;

  procedure proc_var_smallarray_const_1_mixed(b1 : byte; var arr : array of const; b2: byte);{$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
    for i:=0 to high(arr) do
     begin
       case arr[i].vtype of
        vtInteger : arr[i].vinteger := RESULT_U8BIT;
        vtBoolean : arr[i].vboolean := RESULT_BOOLEAN;
        else
          RunError(255);
       end;
     end; {endfor}
     value_u8bit := RESULT_U8BIT;
 end;


  procedure proc_var_smallarray_const_2_mixed(b1 : byte; var arr : array of const; b2: byte);{$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
     if high(arr)<0 then
       global_u8bit := RESULT_U8BIT;
     value_u8bit := RESULT_U8BIT;
end;


  procedure proc_var_formaldef_array_mixed(b1 : byte; var buf; b2: byte);safecall;
  var
   p: pbytearr;
  begin
    { array is indexed from 1 }
    p := @buf;
    p[SMALL_INDEX-1] := RESULT_U8BIT;
    p[0] := RESULT_U8BIT;
    value_u8bit := RESULT_U8BIT;
  end;


procedure proc_var_formaldef_string_mixed(b1 : byte; var buf; b2: byte);safecall;
  var
   p: pbytearr;
  begin
    { array is indexed from 1 }
    p := @buf;
    p[SMALL_INDEX-1] := RESULT_U8BIT;
    p[0] := RESULT_U8BIT;
    value_u8bit := RESULT_U8BIT;
  end;

var
  failed: boolean;
  pp : ^pchar;
begin
  {***************************** NORMAL TESTS *******************************}
  clear_globals;
  clear_values;
  failed:=false;

  write('Var parameter test (src : LOC_REFERENCE (orddef)))...');
  proc_var_s32bit(global_s32bit);
  if global_s32bit <> RESULT_S32BIT then
    failed:=true;

  clear_globals;
  clear_values;
  proc_var_s64bit(global_s64bit);
  if global_s64bit <> RESULT_S64BIT then
    failed:=true;

  clear_globals;
  clear_values;
  proc_var_u8bit(global_u8bit);
  if global_u8bit <> RESULT_U8BIT then
    failed:=true;


  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Var parameter test (src : LOC_REFERENCE (recorddef)))...');
  clear_globals;
  clear_values;
  failed := false;

  proc_var_smallrecord(value_smallrec);
  if (value_smallrec.b <> RESULT_U8BIT) or (value_smallrec.w <> RESULT_U16BIT) then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_largerecord(value_largerec);
  if (value_largerec.b[1] <> RESULT_U8BIT) or (value_largerec.b[2] <> RESULT_U8BIT) then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');


  write('var parameter test (src : LOC_REFERENCE (setdef)))...');
  clear_globals;
  clear_values;
  failed := false;

  proc_var_smallset(value_smallset);
  if (not (A_A in value_smallset)) or (not (A_D in value_smallset)) then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_largeset(value_largeset);
  if not ('I' in value_largeset) then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');


  write('var parameter test (src : LOC_REFERENCE (stringdef)))...');
  clear_globals;
  clear_values;
  failed := false;
  proc_var_smallstring(value_smallstring);
  if value_smallstring <> RESULT_SMALLSTRING then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_bigstring(value_bigstring);
  if value_bigstring <> RESULT_BIGSTRING then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_openstring(value_smallstring);
  if (value_smallstring <> RESULT_SMALLSTRING) or (global_u8bit <> high(value_smallstring)) then
    failed := true;


  if failed then
    fail
  else
    WriteLn('Passed!');


  write('Var parameter test (src : LOC_REFERENCE (formaldef)))...');
  clear_globals;
  clear_values;
  failed:=false;

  proc_var_formaldef_array(value_smallarray);
  if (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) or (value_smallarray[1] <> RESULT_U8BIT) then
    failed := true;


  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Var parameter test (src : LOC_REFERENCE (arraydef)))...');

  clear_globals;
  clear_values;
  failed:=false;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_var_smallarray(value_smallarray);
  if (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) or (value_smallarray[1] <> RESULT_U8BIT) then
    failed := true;



  clear_globals;
  clear_values;

  proc_var_smallarray_open(value_smallarray);
  if (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) or (value_smallarray[1] <> RESULT_U8BIT) then
    failed := true;

(*   HOW CAN ARRAY OF CONST VAR PARAMETERS BE TESTED?
  clear_globals;
  clear_values;
  value_u8bit := RESULT_U8BIT;
  value_ptr := RESULT_PCHAR;
  value_s64bit := RESULT_S64BIT;
  value_smallstring := RESULT_SMALLSTRING;
  value_class := tclass1.create;
  value_boolean := RESULT_BOOLEAN;
  value_char := RESULT_CHAR;
  value_s64real:=RESULT_S64REAL;
  proc_var_smallarray_var_1([value_u8bit,value_ptr,value_s64bit,value_char,value_smallstring,value_s64real,value_boolean,value_class]);

  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  if global_char <> RESULT_CHAR then
    failed := true;
  if global_boolean <> RESULT_BOOLEAN then
    failed:=true;
  if trunc(global_s64real) <> trunc(RESULT_S64REAL) then
     failed := true;
  if global_bigstring <> RESULT_SMALLSTRING then
     failed := true;
  if global_ptr <> value_ptr then
     failed := true;
{  if value_class <> global_class then
     failed := true;!!!!!!!!!!!!!!!!!!!!}
  if global_s64bit <> RESULT_S64BIT then
     failed := true;
  if assigned(value_class) then
    value_class.destroy;
  global_u8bit := 0;
  proc_var_smallarray_const_2([]);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
*)


  if failed then
    fail
  else
    WriteLn('Passed!');

  {***************************** MIXED  TESTS *******************************}
  clear_globals;
  clear_values;
  failed:=false;

  write('Var parameter test (src : LOC_REFERENCE (orddef)))...');
  proc_var_s32bit_mixed(RESULT_U8BIT, global_s32bit, RESULT_U8BIT);
  if global_s32bit <> RESULT_S32BIT then
    failed:=true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_s64bit_mixed(RESULT_U8BIT, global_s64bit, RESULT_U8BIT);
  if global_s64bit <> RESULT_S64BIT then
    failed:=true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_u8bit_mixed(RESULT_U8BIT, global_u8bit, RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed:=true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;


  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Var parameter test (src : LOC_REFERENCE (recorddef)))...');
  clear_globals;
  clear_values;
  failed := false;

  proc_var_smallrecord_mixed(RESULT_U8BIT,value_smallrec, RESULT_U8BIT);
  if (value_smallrec.b <> RESULT_U8BIT) or (value_smallrec.w <> RESULT_U16BIT) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_largerecord_mixed(RESULT_U8BIT, value_largerec, RESULT_U8BIT);
  if (value_largerec.b[1] <> RESULT_U8BIT) or (value_largerec.b[2] <> RESULT_U8BIT) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');


  write('var parameter test (src : LOC_REFERENCE (setdef)))...');
  clear_globals;
  clear_values;
  failed := false;

  proc_var_smallset_mixed(RESULT_U8BIT, value_smallset, RESULT_U8BIT);
  if (not (A_A in value_smallset)) or (not (A_D in value_smallset)) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_largeset_mixed(RESULT_U8BIT, value_largeset, RESULT_U8BIT);
  if not ('I' in value_largeset) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');


  write('var parameter test (src : LOC_REFERENCE (stringdef)))...');
  clear_globals;
  clear_values;
  failed := false;
  proc_var_smallstring_mixed(RESULT_U8BIT, value_smallstring, RESULT_U8BIT);
  if value_smallstring <> RESULT_SMALLSTRING then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_bigstring_mixed(RESULT_U8BIT, value_bigstring,RESULT_U8BIT);
  if value_bigstring <> RESULT_BIGSTRING then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  proc_var_openstring_mixed(RESULT_U8BIT, value_smallstring, RESULT_U8BIT);
  if (value_smallstring <> RESULT_SMALLSTRING) or (global_u8bit <> high(value_smallstring)) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;


  if failed then
    fail
  else
    WriteLn('Passed!');


  write('Var parameter test (src : LOC_REFERENCE (formaldef)))...');
  clear_globals;
  clear_values;
  failed:=false;

  proc_var_formaldef_array_mixed(RESULT_U8BIT, value_smallarray, RESULT_U8BIT);
  if (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) or (value_smallarray[1] <> RESULT_U8BIT) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;


  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Var parameter test (src : LOC_REFERENCE (arraydef)))...');

  clear_globals;
  clear_values;
  failed:=false;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_var_smallarray_mixed(RESULT_U8BIT, value_smallarray, RESULT_U8BIT);
  if (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) or (value_smallarray[1] <> RESULT_U8BIT) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;



  clear_globals;
  clear_values;

  proc_var_smallarray_open_mixed(RESULT_U8BIT, value_smallarray, RESULT_U8BIT);
  if (value_smallarray[SMALL_INDEX] <> RESULT_U8BIT) or (value_smallarray[1] <> RESULT_U8BIT) then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');

end.
