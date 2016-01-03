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
{          (const parameters with safecall calling convention)   }
{****************************************************************}
program tcalcst6;
{$ifdef fpc}
  {$mode objfpc}
  {$INLINE ON}
{$endif}
{$R+}

{$ifdef VER70}
  {$define tp}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ On linux/i386 and linux/x86-64safecall is the same as cdecl, so it does not       }
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
{$ifndef tp}
  tclass1 = class
  end;
{$else}
  shortstring = string;
{$endif}

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
 global_s32real : single;
 global_s64real : double;
 global_ptr : pchar;
 global_proc : tprocedure;
 global_bigstring : shortstring;
 global_boolean : boolean;
 global_char : char;
 value_u8bit : byte;
 value_u16bit : word;
 value_s32bit : longint;
{$ifndef tp}
 global_class : tclass1;
 global_s64bit : int64;
 value_s64bit : int64;
 value_class : tclass1;
{$endif}
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
{$ifndef tp}
      global_s64bit := 0;
      global_class := nil;
{$endif}
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
{$ifndef tp}
      value_s64bit := 0;
      value_class := nil;
{$endif}
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
  {                           CONST  PARAMETERS                            }
  {************************************************************************}
  function proc_const_s32bit(const v : longint): longint; safecall;
   begin
     global_s32bit := v;
     proc_const_s32bit := 0;
   end;

{$ifndef tp}
  function proc_const_s64bit(const v: int64): longint; safecall;
   begin
     global_s64bit:= v;
     proc_const_s64bit := 0;
   end;

  function proc_const_smallarray_const_1(const arr : array of const): longint; {$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
    for i:=0 to high(arr) do
     begin
       case arr[i].vtype of
        vtInteger : global_u8bit := arr[i].vinteger and $ff;
        vtBoolean : global_boolean := arr[i].vboolean;
        vtChar : global_char := arr[i].vchar;
        vtExtended : global_s64real := arr[i].VExtended^;
        vtString :  global_bigstring := arr[i].VString^;
        vtPointer : ;
        vtPChar : global_ptr := arr[i].VPchar;
        vtObject : ;
{        vtClass : global_class := (arr[i].VClass) as tclass1;}
        vtAnsiString : ;
        vtInt64 :  global_s64bit := arr[i].vInt64^;
        else
          RunError(255);
       end;
     end; {endfor}
     proc_const_smallarray_const_1 := 0;
  end;


  function proc_const_smallarray_const_2(const arr : array of const): longint; {$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
     if high(arr)<0 then
       global_u8bit := RESULT_U8BIT;
     proc_const_smallarray_const_2 := 0;
  end;

{$endif}


  function proc_const_smallrecord(const smallrec : tsmallrecord): longint;safecall;
   begin
     if (smallrec.b = RESULT_U8BIT) and (smallrec.w = RESULT_U16BIT) then
       global_u8bit := RESULT_U8BIT;
     proc_const_smallrecord := 0;
   end;


  function proc_const_largerecord(const largerec : tlargerecord): longint;safecall;
   begin
     if (largerec.b[1] = RESULT_U8BIT) and (largerec.b[2] = RESULT_U8BIT) then
       global_u8bit := RESULT_U8BIT;
     proc_const_largerecord := 0;
   end;

  function proc_const_smallset(const smallset : tsmallset): longint;safecall;
   begin
     if A_D in smallset then
       global_u8bit := RESULT_U8BIT;
     proc_const_smallset := 0;
   end;


  function proc_const_largeset(const largeset : tlargeset): longint;safecall;
   begin
     if 'I' in largeset then
       global_u8bit := RESULT_U8BIT;
     proc_const_largeset := 0;
   end;


  function proc_const_smallstring(const s:tsmallstring): longint;safecall;
   begin
     if s = RESULT_SMALLSTRING then
       global_u8bit := RESULT_u8BIT;
     proc_const_smallstring := 0;
   end;


  function proc_const_bigstring(const s:shortstring): longint;safecall;
   begin
     if s = RESULT_BIGSTRING then
       global_u8bit := RESULT_u8BIT;
     proc_const_bigstring := 0;
   end;


  function proc_const_smallarray(const arr : tsmallarray): longint;safecall;
  begin
    if arr[SMALL_INDEX] = RESULT_U8BIT then
      global_u8bit := RESULT_U8BIT;
    proc_const_smallarray := 0;
  end;

  function proc_const_smallarray_open(const arr : array of byte): longint;safecall;
  begin
    { form 0 to N-1 indexes in open arrays }
    if arr[SMALL_INDEX-1] = RESULT_U8BIT then
      global_u8bit := RESULT_U8BIT;
    proc_const_smallarray_open := 0;
  end;




  function proc_const_formaldef_array(const buf): longint;safecall;
  var
   p: pchar;
  begin
    { array is indexed from 1 }
    p := @buf;
    global_u8bit := byte(p[SMALL_INDEX-1]);
    proc_const_formaldef_array := 0;
  end;


  {************************************************************************}
  {                   MIXED   CONST  PARAMETERS                            }
  {************************************************************************}
  function proc_const_s32bit_mixed(b1: byte; const v : longint; b2: byte): longint;safecall;
   begin
     global_s32bit := v;
     value_u8bit := b2;
     proc_const_s32bit_mixed := 0;
   end;

{$ifndef tp}
  function proc_const_s64bit_mixed(b1 : byte; const v: int64; b2: byte): longint;safecall;
   begin
     global_s64bit:= v;
     value_u8bit := b2;
     proc_const_s64bit_mixed := 0;
   end;

  function proc_const_smallarray_const_1_mixed(b1 : byte; const arr : array of const; b2: byte): longint;{$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
    for i:=0 to high(arr) do
     begin
       case arr[i].vtype of
        vtInteger : global_u8bit := arr[i].vinteger and $ff;
        vtBoolean : global_boolean := arr[i].vboolean;
        vtChar : global_char := arr[i].vchar;
        vtExtended : global_s64real := arr[i].VExtended^;
        vtString :  global_bigstring := arr[i].VString^;
        vtPointer : ;
        vtPChar : global_ptr := arr[i].VPchar;
        vtObject : ;
{        vtClass : global_class := (arr[i].VClass) as tclass1;}
        vtAnsiString : ;
        vtInt64 :  global_s64bit := arr[i].vInt64^;
        else
          RunError(255);
       end;
     end; {endfor}
     value_u8bit := b2;
     proc_const_smallarray_const_1_mixed := 0;
  end;


  function proc_const_smallarray_const_2_mixed(b1: byte; const arr : array of const; b2: byte): longint;{$ifndef safecall_is_cdecl} safecall; {$endif}
  var
   i: integer;
  begin
     if high(arr)<0 then
       global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
     proc_const_smallarray_const_2_mixed := 0;
  end;
{$endif}


  function proc_const_smallrecord_mixed(b1 : byte; const smallrec : tsmallrecord; b2: byte): longint;safecall;
   begin
     if (smallrec.b = RESULT_U8BIT) and (smallrec.w = RESULT_U16BIT) then
       global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
     proc_const_smallrecord_mixed := 0;
   end;


  function proc_const_largerecord_mixed(b1: byte; const largerec : tlargerecord; b2: byte): longint; safecall;
   begin
     if (largerec.b[1] = RESULT_U8BIT) and (largerec.b[2] = RESULT_U8BIT) then
       global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
     proc_const_largerecord_mixed := 0;
   end;

  function proc_const_smallset_mixed(b1: byte; const smallset : tsmallset; b2: byte): longint; safecall;
   begin
     if A_D in smallset then
       global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
     proc_const_smallset_mixed := 0;
   end;


  function proc_const_largeset_mixed(b1: byte; const largeset : tlargeset; b2: byte): longint; safecall;
   begin
     if 'I' in largeset then
       global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
     proc_const_largeset_mixed := 0;
   end;


  function proc_const_smallstring_mixed(b1: byte; const s:tsmallstring; b2: byte): longint; safecall;
   begin
     if s = RESULT_SMALLSTRING then
       global_u8bit := RESULT_u8BIT;
     value_u8bit := b2;
     proc_const_smallstring_mixed := 0;
   end;


  function proc_const_bigstring_mixed(b1: byte; const s:shortstring; b2: byte): longint; safecall;
   begin
     if s = RESULT_BIGSTRING then
       global_u8bit := RESULT_u8BIT;
     value_u8bit := b2;
     proc_const_bigstring_mixed := 0;
   end;


  function proc_const_smallarray_mixed(b1: byte; const arr : tsmallarray; b2: byte): longint; safecall;
  begin
    if arr[SMALL_INDEX] = RESULT_U8BIT then
      global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
     proc_const_smallarray_mixed := 0;
  end;

  function proc_const_smallarray_open_mixed(b1: byte; const arr : array of byte; b2: byte): longint; {$ifndef safecall_is_cdecl} safecall; {$endif}
  begin
    { form 0 to N-1 indexes in open arrays }
    if arr[high(arr)] = RESULT_U8BIT then
      global_u8bit := RESULT_U8BIT;
     value_u8bit := b2;
    proc_const_smallarray_open_mixed := 0;
  end;




  function proc_const_formaldef_array_mixed(b1: byte; const buf; b2: byte): longint; safecall;
  var
   p: pchar;
  begin
    { array is indexed from 1 }
    p := @buf;
    global_u8bit := byte(p[SMALL_INDEX-1]);
    value_u8bit := b2;
    proc_const_formaldef_array_mixed := 0;
  end;


var
  failed: boolean;
  pp : ^pchar;
begin
  {***************************** NORMAL TESTS *******************************}
  write('Const parameter test (src : LOC_REGISTER (orddef)))...');
  clear_globals;
  clear_values;
  failed:=false;

  proc_const_s32bit(gets32bit);
  if global_s32bit <> RESULT_S32BIT then
    failed:=true;
{$ifndef tp}
  proc_const_s64bit(gets64bit);
  if global_s64bit <> RESULT_S64BIT then
    failed:=true;
{$endif}

  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Const parameter test (src : LOC_REFERENCE (recorddef)))...');
  clear_globals;
  clear_values;
  failed := false;

  value_smallrec.b := RESULT_U8BIT;
  value_smallrec.w := RESULT_U16BIT;
  proc_const_smallrecord(value_smallrec);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  fillchar(value_largerec,sizeof(value_largerec),RESULT_U8BIT);
  proc_const_largerecord(value_largerec);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');



  write('const parameter test (src : LOC_REFERENCE (setdef)))...');
  clear_globals;
  clear_values;
  failed := false;

  value_smallset := [A_A,A_D];
  proc_const_smallset(value_smallset);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  value_largeset := ['I'];
  proc_const_largeset(value_largeset);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');





  write('const parameter test (src : LOC_REFERENCE (stringdef)))...');
  clear_globals;
  clear_values;
  failed := false;
  value_smallstring := RESULT_SMALLSTRING;

  proc_const_smallstring(value_smallstring);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  value_bigstring := RESULT_BIGSTRING;
  proc_const_bigstring(value_bigstring);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');




  write('Const parameter test (src : LOC_REFERENCE (formaldef)))...');
  clear_globals;
  clear_values;
  failed:=false;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_const_formaldef_array(value_smallarray);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');



  write('Const parameter test (src : LOC_REFERENCE (arraydef)))...');

  clear_globals;
  clear_values;
  failed:=false;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_const_smallarray(value_smallarray);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_const_smallarray_open(value_smallarray);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;

{$ifndef tp}
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
  proc_const_smallarray_const_1([value_u8bit,value_ptr,value_s64bit,value_char,value_smallstring,value_s64real,
    value_boolean,value_class]);

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
  proc_const_smallarray_const_2([]);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
{$endif}

  if failed then
    fail
  else
    WriteLn('Passed!');


  {***************************** MIXED  TESTS *******************************}
  write('Mixed const parameter test (src : LOC_REGISTER (orddef)))...');
  clear_globals;
  clear_values;
  failed:=false;

  proc_const_s32bit_mixed(RESULT_U8BIT,gets32bit,RESULT_U8BIT);
  if global_s32bit <> RESULT_S32BIT then
    failed:=true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;
{$ifndef tp}
  proc_const_s64bit_mixed(RESULT_U8BIT,gets64bit,RESULT_U8BIT);
  if global_s64bit <> RESULT_S64BIT then
    failed:=true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;
{$endif}
  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Mixed const parameter test (src : LOC_REFERENCE (recorddef)))...');
  clear_globals;
  clear_values;
  failed := false;

  value_smallrec.b := RESULT_U8BIT;
  value_smallrec.w := RESULT_U16BIT;
  proc_const_smallrecord_mixed(RESULT_U8BIT,value_smallrec,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  fillchar(value_largerec,sizeof(value_largerec),RESULT_U8BIT);
  proc_const_largerecord_mixed(RESULT_U8BIT,value_largerec,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');



  write('Mixed const parameter test (src : LOC_REFERENCE (setdef)))...');
  clear_globals;
  clear_values;
  failed := false;

  value_smallset := [A_A,A_D];
  proc_const_smallset_mixed(RESULT_U8BIT,value_smallset,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  value_largeset := ['I'];
  proc_const_largeset_mixed(RESULT_U8BIT,value_largeset,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');


  write('Mixed const parameter test (src : LOC_REFERENCE (stringdef)))...');
  clear_globals;
  clear_values;
  failed := false;
  value_smallstring := RESULT_SMALLSTRING;

  proc_const_smallstring_mixed(RESULT_U8BIT,value_smallstring,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;
  value_bigstring := RESULT_BIGSTRING;
  proc_const_bigstring_mixed(RESULT_U8BIT,value_bigstring,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');

  write('Mixed const parameter test (src : LOC_REFERENCE (formaldef)))...');
  clear_globals;
  clear_values;
  failed:=false;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_const_formaldef_array_mixed(RESULT_U8BIT,value_smallarray,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  if failed then
    fail
  else
    WriteLn('Passed!');



  write('Mixed const parameter test (src : LOC_REFERENCE (arraydef)))...');

  clear_globals;
  clear_values;
  failed:=false;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_const_smallarray_mixed(RESULT_U8BIt,value_smallarray,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  clear_globals;
  clear_values;

  value_smallarray[SMALL_INDEX] := RESULT_U8BIT;
  proc_const_smallarray_open_mixed(RESULT_U8BIT,value_smallarray,RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

{$ifndef tp}
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
  proc_const_smallarray_const_1_mixed(RESULT_U8BIT, [value_u8bit,value_ptr,value_s64bit,value_char,value_smallstring,
    value_s64real,value_boolean,value_class],RESULT_U8BIT);

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
  if value_u8bit <> RESULT_U8BIT then
    failed := true;

  global_u8bit := 0;
  proc_const_smallarray_const_2_mixed(RESULT_U8BIT,[],RESULT_U8BIT);
  if global_u8bit <> RESULT_U8BIT then
    failed := true;
  if value_u8bit <> RESULT_U8BIT then
    failed := true;
{$endif}

  if failed then
    fail
  else
    WriteLn('Passed!');
end.
