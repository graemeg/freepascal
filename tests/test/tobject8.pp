{ %OPT=-gh }
// Validate that objects with parent are finalized when statically allocated
type
  pobj = ^tobj;
  tobj = object
  public
    foo: ansistring;
    constructor init(const s: ansistring);
    procedure test; virtual;
    destructor done; virtual;
  end;

  pobj1 = ^tobj1;
  tobj1 = object(tobj)
    bar: ansistring;
    constructor init(const s1,s2: ansistring);
    procedure test; virtual;
    destructor done; virtual;
  end;

constructor tobj.init(const s: ansistring);
begin
  foo:=s;
end;

destructor tobj.done;
begin
end;

constructor tobj1.init(const s1,s2: ansistring);
begin
  inherited init(s1);
  bar:=s2;
end;

destructor tobj1.done;
begin
  inherited done;
end;

procedure tobj.test;
begin
end;

procedure tobj1.test;
begin
end;

var
  s1, s2, s3, s4: ansistring;
  obj: tobj1;

procedure local;
var
  instance: tobj1;
begin
  instance.init(s3,s4);
  
end;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  HaltOnNotReleased:=true;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  s1 := 'string1';
  s2 := 'string2';
  s3 := 'string3';
  s4 := 'string4';
  UniqueString(s1);  // make it actually allocate memory for strings
  UniqueString(s2);
  UniqueString(s3);
  UniqueString(s4);
  local;
  obj.init(s1,s2);
end.
