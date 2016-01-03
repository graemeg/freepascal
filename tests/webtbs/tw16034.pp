program Hello;

{$ifdef fpc}
{$mode delphi}
{$endif}

{$APPTYPE CONSOLE}
{$O-}

type
  ptr = pointer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef fpc}
  codeptr = codepointer;
{$else}
  codeptr = pointer;
{$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Int = ptrint;
  pPtr = ^ptr;
  UInt = ptruint;
  Bool = Boolean;

  // Object woth VMT at offset 0.
  TObj0 =
    object
      Constructor Init;
      Function Value(p: UInt): UInt; Virtual;
    enD;

  // Object with VMT at offset 0, and size = 5.
  TObj1 =
    object (TObj0)
      f1: Byte; // UInt;

      Constructor Init;
      Function Value(p: UInt): UInt; Virtual;
    enD;

  // Object with VMT at offset 0, but size = 8. (???)
  TObj2 =
    object
      f1{, f2, f3, f4}: Byte; // UInt;

      Constructor Init;
      Function Value(p: UInt): UInt; Virtual;
    enD;

{ Implmentation }

Constructor TObj0.Init;
begin
enD;

Function TObj0.Value(p: UInt): UInt;
begin
  Result := 0;
enD;

Constructor TObj1.Init;
begin
enD;

Function TObj1.Value(p: UInt): UInt;
begin
  Result := 0;
enD;

Constructor TObj2.Init;
begin
enD;

Function TObj2.Value(p: UInt): UInt;
begin
  Result := 0;
enD;

{ Low Level VMT Routines }

type
  pObjVMT = ^TObjVMT;
  TObjVMT =
    record
      fInstanceSize: UInt;
      fInstanceSize2: Int;
      fParent: pObjVMT;
    enD;

Function GetInstanceSize(AVMT: pObjVMT): UInt;
begin
  Result := AVMT.fInstanceSize;
enD;

Function GetVMTPtrOffset(AVMT: pObjVMT): UInt;
begin
  if (AVMT.fParent = nil) then
    Result := GetInstanceSize(AVMT) - SizeOf(ptr) else
    Result := GetVMTPtrOffset(AVMT.fParent);
enD;

Function SetVMT(Obj: ptr; AVMT: ptr): Bool;
begin
  Result := (AVMT <> nil);

  if (Result) then
    pPtr(UInt(Obj) + GetVMTPtrOffset(AVMT))^ := AVMT;
enD;


{ Main }

var
  O0: TObj0;
  O1: TObj1;
  O2: TObj2;

  s0, s1, s2: UInt;
  v0, v1, v2: ptr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  cn0, cn1, cn2: codeptr;
=======
  cn0, cn1, cn2: ptr;
>>>>>>> graemeg/cpstrnew
=======
  cn0, cn1, cn2: ptr;
>>>>>>> graemeg/cpstrnew
=======
  cn0, cn1, cn2: ptr;
>>>>>>> graemeg/cpstrnew
=======
  cn0, cn1, cn2: ptr;
>>>>>>> origin/cpstrnew

begin
  // VMT Pointers
  v0 := TypeOf(TObj0);
  v1 := TypeOf(TObj1);
  v2 := TypeOf(TObj2);

  // Object sizes
  s0 := SizeOf(TObj0); // = 4
  s1 := SizeOf(TObj1); // = 5
  s2 := SizeOf(TObj2); // = 8 (???)
  writeln(s0);
  writeln(s1);
  writeln(s2);

  // Method pointers
  cn0 := @TObj0.Value;
  cn1 := @TObj1.Value;
  cn2 := @TObj2.Value;

  // VMT offsets (use in watches - need in program!)
// Int(@o0._vptr$) - Int(@o0)     = 0
// Int(@o1._vptr$) - Int(@o1)     = 0
// Int(@o2._vptr$) - Int(@o2)     = 1 (???)

{
  // Constructors - skipping
  O0.Init;
  O1.Init;
  O2.Init;
}

  // Store VMT (emulate constructor)
  SetVMT(@O0, TypeOf(TObj0));
  SetVMT(@O1, TypeOf(TObj1));
  SetVMT(@O2, TypeOf(TObj2));

  // Call Virtual Functions
  O2.f1 := O0.Value(0);
  O2.f1 := O1.Value(0);
  O2.f1 := O2.Value(0); {CRASHES !!!}
  { SizeOf(TObj2) must be 5,
  or ptr(Int(@o2._vptr$) - Int(@o2)) must be 4! }

  // MessageBox will be displayed, if all was successfull
  writeln(O2.f1, 'Hello, FPC uWorld!', 'Hello', 0);
end.

