program test;

{$mode objfpc}
{$r+,q+}
{$inline on}

const
  DBIDMASK = $FFFFFFFFFFFF;

type
  TmydbID = type Longword;
  TmydbCLSID = type Word;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TmydbDBID   = 0..(qword(1) shl 48)-1;  // Unique ID of the database
=======
  TmydbDBID   = 0..(1 shl 48)-1;  // Unique ID of the database
>>>>>>> graemeg/cpstrnew
=======
  TmydbDBID   = 0..(1 shl 48)-1;  // Unique ID of the database
>>>>>>> graemeg/cpstrnew
=======
  TmydbDBID   = 0..(1 shl 48)-1;  // Unique ID of the database
>>>>>>> graemeg/cpstrnew
=======
  TmydbDBID   = 0..(1 shl 48)-1;  // Unique ID of the database
>>>>>>> origin/cpstrnew
  TmydbDBTYPE = type Byte;

  tarr = bitpacked array[0..10] of TmydbDBID;

  TmydbUID = bitpacked record
    DBID  : TmydbDBID;            // Database Identifier
    PROID : TmydbID;              // Profile Identifier
    OID   : TmydbID;              // Object Identifier
    CLSID : TmydbCLSID;           // Object Class
  end;

function mydbMakeUID(const DBID: TmydbDBID; const PROID: TmydbID; const CLSID: TmydbCLSID; const OID: TmydbID): TmydbUID; inline;
begin
  Result.CLSID := CLSID;
  Result.DBID := DBID and DBIDMASK;
  Result.PROID := PROID;
  Result.OID := OID;
end;

var
  uid: TmydbUID;
  arr: tarr;
  i: longint;
begin
  uid:=mydbMakeUID($987654321654,$12345678,$5432,$18273645);
  if (uid.CLSID<>$5432) then
    halt(1);
  if (uid.DBID<>($987654321654 and DBIDMASK)) then
    halt(2);
  if (uid.PROID<>$12345678) then
    halt(3);
  if (uid.OID<>$18273645) then
    halt(4);
  i:=2;
  arr[2]:=$987654321654;
  if (arr[i]<>$987654321654) or
     (arr[1]<>0) or
     (arr[3]<>0) then
    halt(5);
  arr[2]:=0;
  arr[i]:=$987654321654;
  if (arr[i]<>$987654321654) or
     (arr[1]<>0) or
     (arr[3]<>0) then
    halt(6);
end.
