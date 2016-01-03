{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    This unit makes Free Pascal as much as possible Delphi compatible

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$Mode ObjFpc}
{$I-}
{$ifndef Unix}
  {$S-}
{$endif}
unit objpas;

interface

  { first, in object pascal, the integer type must be redefined }
{$ifdef CPU16}
   const
       MaxInt  = MaxSmallint;
    type
       Integer  = smallint;
       PInteger = ^Integer;
{$else CPU16}
    const
       MaxInt  = MaxLongint;
    type
       Integer  = longint;
       PInteger = ^Integer;
{$endif CPU16}

       { Ansistring are the default }
       PString = PAnsiString;

       { array types }
{$ifdef CPU16}
       IntegerArray  = array[0..(32768 div SizeOf(Integer))-2] of Integer;
{$else CPU16}
       IntegerArray  = array[0..$effffff] of Integer;
{$endif CPU16}
       TIntegerArray = IntegerArray;
       PIntegerArray = ^IntegerArray;
{$ifdef CPU16}
       PointerArray  = array [0..(32768 div SizeOf(Pointer))-2] of Pointer;
{$else CPU16}
       PointerArray  = array [0..512*1024*1024-2] of Pointer;
{$endif CPU16}
       TPointerArray = PointerArray;
       PPointerArray = ^PointerArray;
       TBoundArray = array of integer;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{$if FPC_FULLVERSION >= 20701}

      { Generic array type. 
        Slightly Less useful in FPC, since dyn array compatibility is at the element level. 
        But still useful for generic methods and of course Delphi compatibility}
      
      Generic TArray<T> = Array of T;
      
      { Generic support for enumerator interfaces. These are added here, because
        mode (Obj)FPC does currently not allow the overloading of types with
        generic types (this will need a modeswitch...) }

      { Note: In Delphi these two generic types inherit from the two interfaces
              above, but in FPC as well as in Delphi(!) this leads to problems,
              because of method hiding and method implementation. E.g.
              consider a class which enumerates integers one needs to implement
              a GetCurrent for TObject as well... }
       generic IEnumerator<T> = interface
         function GetCurrent: T;
         function MoveNext: Boolean;
         procedure Reset;
         property Current: T read GetCurrent;
       end;

       generic IEnumerable<T> = interface
         function GetEnumerator: specialize IEnumerator<T>;
       end;
{$endif}


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef FPC_HAS_FEATURE_CLASSES}
Var
   ExceptionClass: TClass; { Exception base class (must actually be Exception, defined in sysutils ) }
{$endif FPC_HAS_FEATURE_CLASSES}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
Var
   ExceptObjProc: Pointer; { Used to convert OS exceptions to exceptions in Delphi. Unused in FPC}
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
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
=======
Var
   ExceptionClass: TClass; { Exception base class (must actually be Exception, defined in sysutils ) }
   ExceptObjProc: Pointer; { Used to convert OS exceptions to exceptions in Delphi. Unused in FPC}
>>>>>>> graemeg/fixes_2_2
=======
Var
   ExceptionClass: TClass; { Exception base class (must actually be Exception, defined in sysutils ) }
   ExceptObjProc: Pointer; { Used to convert OS exceptions to exceptions in Delphi. Unused in FPC}
>>>>>>> origin/fixes_2_2

{****************************************************************************
                             Compatibility routines.
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
    { Untyped file support }
<<<<<<< HEAD
<<<<<<< HEAD
     Procedure AssignFile(out f:File;p:pchar);
     Procedure AssignFile(out f:File;c:char);
  {$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
     Procedure AssignFile(out f:File;const Name:UnicodeString);
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
     Procedure AssignFile(out f:File;const Name:RawByteString);
  {$endif FPC_HAS_FEATURE_ANSISTRINGS}
     Procedure CloseFile(var f:File);
{$endif FPC_HAS_FEATURE_FILEIO}

{$ifdef FPC_HAS_FEATURE_TEXTIO}
     { Text file support }
     Procedure AssignFile(out t:Text;p:pchar);
     Procedure AssignFile(out t:Text;c:char);
  {$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
     Procedure AssignFile(out t:Text;const Name:UnicodeString);
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
     Procedure AssignFile(out t:Text;const Name:RawByteString);
  {$endif FPC_HAS_FEATURE_ANSISTRINGS}
=======
=======
>>>>>>> origin/fixes_2_2

     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;const Name:string);
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;p:pchar);
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;c:char);
     Procedure CloseFile(var f:File);

     { Text file support }
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;const s:string);
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;p:pchar);
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;c:char);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     Procedure CloseFile(Var t:Text);
{$endif FPC_HAS_FEATURE_TEXTIO}

{$ifdef FPC_HAS_FEATURE_FILEIO}
     { Typed file supoort }
<<<<<<< HEAD
<<<<<<< HEAD
     Procedure AssignFile(out f:TypedFile;p:pchar);
     Procedure AssignFile(out f:TypedFile;c:char);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
     Procedure AssignFile(out f:TypedFile;const Name:UnicodeString);
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
     Procedure AssignFile(out f:TypedFile;const Name:RawByteString);
  {$endif FPC_HAS_FEATURE_ANSISTRINGS}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_FEATURE_FILEIO}

{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
=======
=======
>>>>>>> origin/fixes_2_2

     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;const Name:string);
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;p:pchar);
     Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;c:char);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     { ParamStr should return also an ansistring }
     Function ParamStr(Param : Integer) : Ansistring;
{$endif FPC_HAS_FEATURE_COMMANDARGS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifdef FPC_HAS_FEATURE_FILEIO}
     Procedure MkDir(const s:ansistring);overload;
     Procedure RmDir(const s:ansistring);overload;
     Procedure ChDir(const s:ansistring);overload;
{$endif FPC_HAS_FEATURE_FILEIO}
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

{****************************************************************************
                             Resource strings.
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_RESOURCES}
   type
     TResourceIterator = Function (Name,Value : AnsiString; Hash : Longint; arg:pointer) : AnsiString;

   Function Hash(S : AnsiString) : LongWord;
   Procedure ResetResourceTables;
   Procedure FinalizeResourceTables;
   Procedure SetResourceStrings (SetFunction :  TResourceIterator;arg:pointer);
   Procedure SetUnitResourceStrings (const UnitName:string;SetFunction :  TResourceIterator;arg:pointer);

   { Delphi compatibility }
   type
     PResStringRec=^AnsiString;
     TResStringRec=AnsiString;
   Function LoadResString(p:PResStringRec):AnsiString;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_RESOURCES}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  implementation

{****************************************************************************
                             Compatibility routines.
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_FILEIO}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure MkDirpchar(s: pchar;len:sizeuint);[IOCheck]; external name 'FPC_SYS_MKDIR';
Procedure ChDirpchar(s: pchar;len:sizeuint);[IOCheck]; external name 'FPC_SYS_CHDIR';
Procedure RmDirpchar(s: pchar;len:sizeuint);[IOCheck]; external name 'FPC_SYS_RMDIR';

{ Untyped file support }
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
<<<<<<< HEAD
{ Untyped file support }
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;const Name:string);
>>>>>>> graemeg/fixes_2_2
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;const Name:string);
>>>>>>> origin/fixes_2_2

Procedure AssignFile(out f:File;p:pchar);
begin
  System.Assign (F,p);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure AssignFile(out f:File;c:char);
=======
=======
>>>>>>> origin/fixes_2_2
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;p:pchar);

>>>>>>> graemeg/fixes_2_2
begin
  System.Assign (F,c);
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure AssignFile(out f:File;const Name:RawBytestring);
begin
  System.Assign (F,Name);
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;c:char);
>>>>>>> graemeg/fixes_2_2
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:File;c:char);
>>>>>>> origin/fixes_2_2

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure AssignFile(out f:File;const Name:UnicodeString);
begin
  System.Assign (F,Name);
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

Procedure CloseFile(Var f:File); [IOCheck];

begin
  { Catch Runtime error/Exception }
  System.Close(f);
end;
{$endif FPC_HAS_FEATURE_FILEIO}

{$ifdef FPC_HAS_FEATURE_TEXTIO}
{ Text file support }

<<<<<<< HEAD
<<<<<<< HEAD
Procedure AssignFile(out t:Text;p:pchar);
=======
=======
>>>>>>> origin/fixes_2_2
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;const s:string);

>>>>>>> graemeg/fixes_2_2
begin
  System.Assign (T,p);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Procedure AssignFile(out t:Text;c:char);
=======
=======
>>>>>>> origin/fixes_2_2
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;p:pchar);

>>>>>>> graemeg/fixes_2_2
begin
  System.Assign (T,c);
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure AssignFile(out t:Text;const Name:RawBytestring);
begin
  System.Assign (T,Name);
end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;c:char);
>>>>>>> graemeg/fixes_2_2
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} t:Text;c:char);
>>>>>>> origin/fixes_2_2

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure AssignFile(out t:Text;const Name:UnicodeString);
begin
  System.Assign (T,Name);
end;
{$endif FPC_HAS_FEATURE_WIDESTRINGS}

Procedure CloseFile(Var t:Text); [IOCheck];

begin
  { Catch Runtime error/Exception }
  System.Close(T);
end;
{$endif FPC_HAS_FEATURE_TEXTIO}

{$ifdef FPC_HAS_FEATURE_FILEIO}
{ Typed file support }

<<<<<<< HEAD
<<<<<<< HEAD
Procedure AssignFile(out f:TypedFile;p:pchar);
begin
  System.Assign (F,p);
end;
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;const Name:string);
>>>>>>> graemeg/fixes_2_2
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;const Name:string);
>>>>>>> origin/fixes_2_2

Procedure AssignFile(out f:TypedFile;c:char);
begin
  System.Assign (F,c);
end;
{$endif FPC_HAS_FEATURE_FILEIO}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Procedure AssignFile(out f:TypedFile;const Name:RawBytestring);
=======
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
Function ParamStr(Param : Integer) : Ansistring;

Var Len : longint;
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;p:pchar);
>>>>>>> origin/fixes_2_2

>>>>>>> graemeg/cpstrnew
begin
  System.Assign (F,Name);
end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;p:pchar);
>>>>>>> graemeg/fixes_2_2

{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
Procedure AssignFile(out f:TypedFile;const Name:UnicodeString);
begin
  System.Assign (F,Name);
end;
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
{$endif FPC_HAS_FEATURE_FILEIO}

{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
Function ParamStr(Param : Integer) : ansistring;
  begin
  {
    Paramstr(0) should return the name of the binary.
    Since this functionality is included in the system unit,
    we fetch it from there.
    Normally, pathnames are less than 255 chars anyway,
    so this will work correct in 99% of all cases.
    In time, the system unit should get a GetExeName call.
  }
    if (Param=0) then
      Result:=System.Paramstr(0)
    else if (Param>0) and (Param<argc) then
      Result:=Argv[Param]
    else
      Result:='';
  end;
{$endif FPC_HAS_FEATURE_COMMANDARGS}
=======
{$endif FPC_HAS_FEATURE_COMMANDARGS}
=======
{$endif FPC_HAS_FEATURE_FILEIO}

=======
{$endif FPC_HAS_FEATURE_FILEIO}

>>>>>>> graemeg/cpstrnew
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
Function ParamStr(Param : Integer) : Ansistring;
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;c:char);
>>>>>>> origin/fixes_2_2

<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_FILEIO}
Procedure MkDir(const s:ansistring);
begin
  mkdirpchar(pchar(s),length(s));
end;
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_COMMANDARGS}
=======
Procedure AssignFile({$ifdef PARAOUTFILE}out{$else}var{$endif} f:TypedFile;c:char);
>>>>>>> graemeg/fixes_2_2

Procedure RmDir(const s:ansistring);
begin
  RmDirpchar(pchar(s),length(s));
end;
=======
{$endif FPC_HAS_FEATURE_FILEIO}

{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
Function ParamStr(Param : Integer) : Ansistring;
>>>>>>> origin/cpstrnew

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
{$ifdef FPC_HAS_FEATURE_FILEIO}
Procedure MkDir(const s:ansistring);
begin
  mkdirpchar(pchar(s),length(s));
end;
<<<<<<< HEAD

Procedure RmDir(const s:ansistring);
begin
  RmDirpchar(pchar(s),length(s));
end;

>>>>>>> graemeg/cpstrnew
Procedure ChDir(const s:ansistring);
begin
  ChDirpchar(pchar(s),length(s));
end;
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_FILEIO}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
{$endif FPC_HAS_FEATURE_COMMANDARGS}
>>>>>>> origin/cpstrnew

=======

<<<<<<< HEAD
=======
{$ifdef FPC_HAS_FEATURE_FILEIO}
Procedure MkDir(const s:ansistring);
begin
  mkdirpchar(pchar(s),length(s));
end;

>>>>>>> origin/cpstrnew
Procedure RmDir(const s:ansistring);
begin
  RmDirpchar(pchar(s),length(s));
end;

Procedure ChDir(const s:ansistring);
begin
  ChDirpchar(pchar(s),length(s));
end;
{$endif FPC_HAS_FEATURE_FILEIO}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef FPC_HAS_FEATURE_RESOURCES}
{ ---------------------------------------------------------------------
    ResourceString support
  ---------------------------------------------------------------------}
Function Hash(S : AnsiString) : LongWord;
Var
  thehash,g,I : LongWord;
begin
   thehash:=0;
   For I:=1 to Length(S) do { 0 terminated }
     begin
     thehash:=thehash shl 4;
     inc(theHash,Ord(S[i]));
     g:=thehash and LongWord($f shl 28);
     if g<>0 then
       begin
       thehash:=thehash xor (g shr 24);
       thehash:=thehash xor g;
       end;
     end;
   If theHash=0 then
     Hash:=$ffffffff
   else
     Hash:=TheHash;
end;

Type
   TResourceStringTableList = Packed Record
     Count : sizeint;
     Tables : Array[{$ifdef cpu16}Byte{$else cpu16}Word{$endif cpu16}] of record
       TableStart,
       TableEnd   : PResourceStringRecord;
     end;
   end;

{ Support for string constants initialized with resourcestrings }
{$ifdef FPC_HAS_RESSTRINITS}
   PResStrInitEntry = ^TResStrInitEntry;
   TResStrInitEntry = record
     Addr: PPointer;
     Data: PResourceStringRecord;
   end;

   TResStrInitTable = packed record
     Count: {$ifdef VER2_6}longint{$else}sizeint{$endif};
     Tables: packed array[1..{$ifdef cpu16}8191{$else cpu16}32767{$endif cpu16}] of PResStrInitEntry;
   end;

var
  ResStrInitTable : TResStrInitTable; external name 'FPC_RESSTRINITTABLES';

procedure UpdateResourceStringRefs;
var
  i: integer;
  ptable: PResStrInitEntry;
begin
  for i:=1 to ResStrInitTable.Count do
    begin
      ptable:=ResStrInitTable.Tables[i];
      while Assigned(ptable^.Addr) do
        begin
          AnsiString(ptable^.Addr^):=ptable^.Data^.CurrentValue;
          Inc(ptable);
        end;
    end;
end;
{$endif FPC_HAS_RESSTRINITS}

Var
  ResourceStringTable : TResourceStringTableList; External Name 'FPC_RESOURCESTRINGTABLES';

Procedure SetResourceStrings (SetFunction :  TResourceIterator;arg:pointer);
Var
  ResStr : PResourceStringRecord;
  i      : integer;
  s      : AnsiString;
begin
  With ResourceStringTable do
    begin
      For i:=0 to Count-1 do
        begin
          ResStr:=Tables[I].TableStart;
          { Skip first entry (name of the Unit) }
          inc(ResStr);
          while ResStr<Tables[I].TableEnd do
            begin
              s:=SetFunction(ResStr^.Name,ResStr^.DefaultValue,Longint(ResStr^.HashValue),arg);
              if s<>'' then
                ResStr^.CurrentValue:=s;
              inc(ResStr);
            end;
        end;
    end;
{$ifdef FPC_HAS_RESSTRINITS}
  UpdateResourceStringRefs;
{$endif FPC_HAS_RESSTRINITS}
end;


Procedure SetUnitResourceStrings (const UnitName:string;SetFunction :  TResourceIterator;arg:pointer);
Var
  ResStr : PResourceStringRecord;
  i      : integer;
  s,
  UpUnitName : AnsiString;
begin
  With ResourceStringTable do
    begin
      UpUnitName:=UpCase(UnitName);
      For i:=0 to Count-1 do
        begin
          ResStr:=Tables[I].TableStart;
          { Check name of the Unit }
          if ResStr^.Name<>UpUnitName then
            continue;
          inc(ResStr);
          while ResStr<Tables[I].TableEnd do
            begin
              s:=SetFunction(ResStr^.Name,ResStr^.DefaultValue,Longint(ResStr^.HashValue),arg);
              if s<>'' then
                ResStr^.CurrentValue:=s;
              inc(ResStr);
            end;
        end;
    end;
{$ifdef FPC_HAS_RESSTRINITS}
  { Resourcestrings of one unit may be referenced from other units,
    so updating everything is the only option. }
  UpdateResourceStringRefs;
{$endif FPC_HAS_RESSTRINITS}
end;


Procedure ResetResourceTables;
Var
  ResStr : PResourceStringRecord;
  i      : integer;
begin
  With ResourceStringTable do
    begin
      For i:=0 to Count-1 do
        begin
          ResStr:=Tables[I].TableStart;
          { Skip first entry (name of the Unit) }
          inc(ResStr);
          while ResStr<Tables[I].TableEnd do
            begin
              ResStr^.CurrentValue:=ResStr^.DefaultValue;
              inc(ResStr);
            end;
        end;
    end;
end;


Procedure FinalizeResourceTables;
Var
  ResStr : PResourceStringRecord;
  i      : integer;
begin
  With ResourceStringTable do
    begin
      For i:=0 to Count-1 do
        begin
          ResStr:=Tables[I].TableStart;
          { Skip first entry (name of the Unit) }
          inc(ResStr);
          while ResStr<Tables[I].TableEnd do
            begin
              ResStr^.CurrentValue:='';
              inc(ResStr);
            end;
        end;
    end;
end;


Function LoadResString(p:PResStringRec):AnsiString;
begin
  Result:=p^;
end;
{$endif FPC_HAS_FEATURE_RESOURCES}


{$ifdef FPC_HAS_FEATURE_RESOURCES}
Initialization
{  ResetResourceTables;}
finalization
  FinalizeResourceTables;
{$endif FPC_HAS_FEATURE_RESOURCES}
end.
