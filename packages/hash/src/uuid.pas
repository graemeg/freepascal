{
    This file is part of the Free Pascal packages.
    Copyright (c) 1999-2006 by the Free Pascal development team

    Implements a UUID generation algorithm (RFC 4122)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit uuid;

interface

{$mode objfpc}
{$h+}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  SysUtils, DateUtils, md5, sha1;
=======
  SysUtils, DateUtils, md5;
>>>>>>> graemeg/fixes_2_2
=======
  SysUtils, DateUtils, md5;
>>>>>>> origin/fixes_2_2



(******************************************************************************
 * types and constants
 ******************************************************************************)

type
{$ifdef VER2_0}
       uuid_t = packed record
          case integer of
             1 : (
                  Data1 : DWord;
                  Data2 : word;
                  Data3 : word;
                  Data4 : array[0..7] of byte;
                 );
             2 : (
                  D1 : DWord;
                  D2 : word;
                  D3 : word;
                  D4 : array[0..7] of byte;
                 );
             3 : ( { uuid fields according to RFC4122 }
                  time_low : dword;     // The low field of the timestamp
                  time_mid : word;                      // The middle field of the timestamp
                  time_hi_and_version : word;           // The high field of the timestamp multiplexed with the version
                  clock_seq_hi_and_reserved : byte;     // The high field of the clock sequence multiplexed with the var
                  clock_seq_low : byte;                 // The low field of the clock sequence
                  node : array[0..5] of byte;           // The spatially unique node identifier
                 );
       end;
{$else VER2_0}
  uuid_t          = TGuid;
{$endif VER2_0}
  uuid_time_t     = qword;
  uuid_node_t     = array[0..5] of byte;
  unsigned16      = word;

  uuid_state      = record
    ts   : uuid_time_t;   // saved timestamp
    node : uuid_node_t;   // saved node ID
    cs   : unsigned16;    // saved clock sequence
  end;

const
  UUID_VERSION_1  = $1;   // The time-based version specified in this document.
  UUID_VERSION_2  = $2;   // DCE Security version, with embedded POSIX UIDs.
  UUID_VERSION_3  = $3;   // The name-based version specified in this document that uses MD5 hashing.
  UUID_VERSION_4  = $4;   // The randomly or pseudo-randomly generated version specified in this document.
  UUID_VERSION_5  = $5;   // The name-based version specified in this document that uses SHA-1 hashing.

{ set the following to the number of 100ns ticks of the actual resolution of your system's clock }
  UUIDS_PER_TICK  = 1024;



(******************************************************************************
 * core uuid functions
 ******************************************************************************)

{ uuid_initialize -- used to initialize the uuid_create function }
procedure uuid_initialize(const state: uuid_state);

{ uuid_create -- generator a UUID }
<<<<<<< HEAD
<<<<<<< HEAD
function uuid_create(out uuid: uuid_t): boolean;

{ uuid_finalize -- returns the current state }
procedure uuid_finalize(out state: uuid_state);

{ uuid_create_md5_from_name -- create a version 3 (MD5) UUID using a "name" from a "name space" }
procedure uuid_create_md5_from_name(out uuid: uuid_t; const nsid: uuid_t; const name: string);

{ uuid_create_sha1_from_name -- create a version 5 (SHA-1) UUID using a "name" from a "name space" }
procedure uuid_create_sha1_from_name(out uuid: uuid_t; const nsid: uuid_t; const name: string);
=======
=======
>>>>>>> origin/fixes_2_2
function uuid_create(var uuid: uuid_t): boolean;

{ uuid_finalize -- returns the current state }
procedure uuid_finalize(var state: uuid_state);

{ uuid_create_md5_from_name -- create a version 3 (MD5) UUID using a "name" from a "name space" }
procedure uuid_create_md5_from_name(var uuid: uuid_t; const nsid: uuid_t; const name: string);

{ uuid_create_sha1_from_name -- create a version 5 (SHA-1) UUID using a "name" from a "name space" }
procedure uuid_create_sha1_from_name(var uuid: uuid_t; const nsid: uuid_t; const name: string);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ uuid_compare --  Compare two UUID's "lexically" }
function uuid_compare(const u1, u2: uuid_t): integer;



(******************************************************************************
 * auxilary functions
 ******************************************************************************)

{ read_state -- read UUID generator state from non-volatile store }
<<<<<<< HEAD
<<<<<<< HEAD
function read_state(out clockseq: unsigned16; out timestamp: uuid_time_t; out node: uuid_node_t): boolean;
=======
function read_state(var clockseq: unsigned16; var timestamp: uuid_time_t; var node: uuid_node_t): boolean;
>>>>>>> graemeg/fixes_2_2
=======
function read_state(var clockseq: unsigned16; var timestamp: uuid_time_t; var node: uuid_node_t): boolean;
>>>>>>> origin/fixes_2_2

{ write_state -- save UUID generator state back to non-volatile storage }
procedure write_state(var clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);

{ format_uuid_v1 -- make a UUID from the timestamp, clockseq, and node ID }
<<<<<<< HEAD
<<<<<<< HEAD
procedure format_uuid_v1(out uuid: uuid_t; const clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);

{ format_uuid_v3or5 -- make a UUID from a (pseudo)random 128-bit number }
procedure format_uuid_v3or5(out uuid: uuid_t; const hash: pointer; const v: integer);

{ get_current_time -- get time as 60-bit 100ns ticks since UUID epoch. Compensate for the fact that real clock resolution is less than 100ns. }
procedure get_current_time(out timestamp: uuid_time_t);
=======
=======
>>>>>>> origin/fixes_2_2
procedure format_uuid_v1(var uuid: uuid_t; const clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);

{ format_uuid_v3or5 -- make a UUID from a (pseudo)random 128-bit number }
procedure format_uuid_v3or5(var uuid: uuid_t; const hash: pointer; const v: integer);

{ get_current_time -- get time as 60-bit 100ns ticks since UUID epoch. Compensate for the fact that real clock resolution is less than 100ns. }
procedure get_current_time(var timestamp: uuid_time_t);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2



(******************************************************************************
 * system functions
 ******************************************************************************)

{ get_system_time -- system dependent call to get the current system time. Returned as 100ns ticks since UUID epoch, but resolution may be less than 100ns. }
<<<<<<< HEAD
<<<<<<< HEAD
procedure get_system_time(out timestamp: uuid_time_t);
=======
procedure get_system_time(var timestamp: uuid_time_t);
>>>>>>> graemeg/fixes_2_2
=======
procedure get_system_time(var timestamp: uuid_time_t);
>>>>>>> origin/fixes_2_2

{ true_random -- generate a crypto-quality random number. }
function true_random: unsigned16;

implementation


{ uuid_initialize }

var
  current_state : uuid_state;
  current_node  : uuid_node_t;

procedure uuid_initialize(const state: uuid_state);
begin
  Randomize;
  current_node[0] := Random($100);
  current_node[1] := Random($100);
  current_node[2] := Random($100);
  current_node[3] := Random($100);
  current_node[4] := Random($100);
  current_node[5] := Random($100);
  current_state := state;
end;


{ uuid_finalize }

<<<<<<< HEAD
<<<<<<< HEAD
procedure uuid_finalize(out state: uuid_state);
=======
procedure uuid_finalize(var state: uuid_state);
>>>>>>> graemeg/fixes_2_2
=======
procedure uuid_finalize(var state: uuid_state);
>>>>>>> origin/fixes_2_2
begin
  state := current_state;
end;


{ uuid_create }

<<<<<<< HEAD
<<<<<<< HEAD
function uuid_create(out uuid: uuid_t): boolean;
=======
function uuid_create(var uuid: uuid_t): boolean;
>>>>>>> graemeg/fixes_2_2
=======
function uuid_create(var uuid: uuid_t): boolean;
>>>>>>> origin/fixes_2_2
var
  timestamp: uuid_time_t;
  last_time: uuid_time_t;
  clockseq: unsigned16;
  last_node: uuid_node_t;
  f: boolean;
begin
  (* acquire system-wide lock so we're alone *)
// LOCK;

  (* get time, node ID, saved state from non-volatile storage *)
  get_current_time(timestamp);
  f := read_state(clockseq, last_time, last_node);

  (* if no NV state, or if clock went backwards, or node ID
     changed (e.g., new network card) change clockseq *)
   if not f or not CompareMem(@current_node, @last_node, sizeof(uuid_node_t)) then
     clockseq := true_random() else
   if timestamp < last_time then
     clockseq := clockseq + 1;

   (* save the state for next time *)
   write_state(clockseq, timestamp, current_node);

// UNLOCK;

   (* stuff fields into the UUID *)
   format_uuid_v1(uuid, clockseq, timestamp, current_node);

   Result := true;
end;


{ uuid_create_md5_from_name }

<<<<<<< HEAD
<<<<<<< HEAD
procedure uuid_create_md5_from_name(out uuid: uuid_t; const nsid: uuid_t; const name: string);
=======
procedure uuid_create_md5_from_name(var uuid: uuid_t; const nsid: uuid_t; const name: string);
>>>>>>> graemeg/fixes_2_2
=======
procedure uuid_create_md5_from_name(var uuid: uuid_t; const nsid: uuid_t; const name: string);
>>>>>>> origin/fixes_2_2
var
  net_nsid: uuid_t;
  c: TMDContext;
  hash: TMDDigest;
begin
  (* put name space ID in network byte order so it hashes the same
     no matter what endian machine we're on *)
  net_nsid := nsid;
  net_nsid.time_low := ntobe(net_nsid.time_low);
  net_nsid.time_mid := ntobe(net_nsid.time_mid);
  net_nsid.time_hi_and_version := ntobe(net_nsid.time_hi_and_version);

  MDInit(c, MD_VERSION_5);
  MDUpdate(c, net_nsid, sizeof(net_nsid));
  MDUpdate(c, pchar(name)^, Length(name));
  MDFinal(c, hash);

  (* the hash is in network byte order at this point *)
  format_uuid_v3or5(uuid, @hash, UUID_VERSION_3);
end;


{ uuid_create_sha1_from_name }

<<<<<<< HEAD
<<<<<<< HEAD
procedure uuid_create_sha1_from_name(out uuid: uuid_t; const nsid: uuid_t; const name: string);
var
  net_nsid: uuid_t;
  c: TSHA1Context;
  hash: TSHA1Digest;
=======
=======
>>>>>>> origin/fixes_2_2
procedure uuid_create_sha1_from_name(var uuid: uuid_t; const nsid: uuid_t; const name: string);
var
  net_nsid: uuid_t;
{  c: TMDContext;
  hash: TMDDigest;}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  (* put name space ID in network byte order so it hashes the same
     no matter what endian machine we're on *)
  net_nsid := nsid;
  net_nsid.time_low := ntobe(net_nsid.time_low);
  net_nsid.time_mid := ntobe(net_nsid.time_mid);
  net_nsid.time_hi_and_version := ntobe(net_nsid.time_hi_and_version);

<<<<<<< HEAD
<<<<<<< HEAD
  SHA1Init(c);
  SHA1Update(c, net_nsid, sizeof(net_nsid));
  SHA1Update(c, pchar(name)^, Length(name));
  SHA1Final(c, hash);
=======
=======
>>>>>>> origin/fixes_2_2
  {SHAInit(c, SHA_VERSION_1);
  SHAUpdate(c, net_nsid, sizeof(net_nsid));
  SHAUpdate(c, pchar(name)^, Length(name));
  SHAFinal(c, hash);}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  (* the hash is in network byte order at this point *)
  format_uuid_v3or5(uuid, @hash, UUID_VERSION_5);
end;


{ uuid_compare }

function uuid_compare(const u1, u2: uuid_t): integer;
begin
  Result := pinteger(@u1)[0] - pinteger(@u2)[0];
  if Result <> 0 then Exit;
  Result := pinteger(@u1)[1] - pinteger(@u2)[1];
  if Result <> 0 then Exit;
  Result := pinteger(@u1)[2] - pinteger(@u2)[2];
  if Result <> 0 then Exit;
  Result := pinteger(@u1)[3] - pinteger(@u2)[3];
end;


{ read_state }

<<<<<<< HEAD
<<<<<<< HEAD
function read_state(out clockseq: unsigned16; out timestamp: uuid_time_t; out node: uuid_node_t): boolean;
=======
function read_state(var clockseq: unsigned16; var timestamp: uuid_time_t; var node: uuid_node_t): boolean;
>>>>>>> graemeg/fixes_2_2
=======
function read_state(var clockseq: unsigned16; var timestamp: uuid_time_t; var node: uuid_node_t): boolean;
>>>>>>> origin/fixes_2_2
begin
  clockseq := current_state.cs;
  timestamp := current_state.ts;
  node := current_state.node;
  Result := true;
end;


{ write_state }

procedure write_state(var clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);
begin
  (* always save state to volatile shared state *)
  current_state.cs := clockseq;
  current_state.ts := timestamp;
  current_state.node := node;
end;


{ format_uuid_v1 }

<<<<<<< HEAD
<<<<<<< HEAD
procedure format_uuid_v1(out uuid: uuid_t; const clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);
=======
procedure format_uuid_v1(var uuid: uuid_t; const clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);
>>>>>>> graemeg/fixes_2_2
=======
procedure format_uuid_v1(var uuid: uuid_t; const clockseq: unsigned16; const timestamp: uuid_time_t; const node: uuid_node_t);
>>>>>>> origin/fixes_2_2
begin
  uuid.time_low := timestamp and $FFFFFFFF;
  uuid.time_mid := (timestamp shr 32) and $FFFF;
  uuid.time_hi_and_version := (timestamp shr 48) and $0FFF;
  uuid.time_hi_and_version := uuid.time_hi_and_version or (UUID_VERSION_1 shl 12);
  uuid.clock_seq_low := clockseq and $FF;
  uuid.clock_seq_hi_and_reserved := (clockseq shr 8) and $3F;
  uuid.clock_seq_hi_and_reserved := uuid.clock_seq_hi_and_reserved or $80;
  uuid.node := node;
end;


{ format_uuid_v3or5 }

<<<<<<< HEAD
<<<<<<< HEAD
procedure format_uuid_v3or5(out uuid: uuid_t; const hash: pointer; const v: integer);
=======
procedure format_uuid_v3or5(var uuid: uuid_t; const hash: pointer; const v: integer);
>>>>>>> graemeg/fixes_2_2
=======
procedure format_uuid_v3or5(var uuid: uuid_t; const hash: pointer; const v: integer);
>>>>>>> origin/fixes_2_2
begin
  (* convert UUID to local byte order *)
  move(hash^, uuid, sizeof(uuid));
  uuid.time_low := beton(uuid.time_low);
  uuid.time_mid := beton(uuid.time_mid);
  uuid.time_hi_and_version := beton(uuid.time_hi_and_version);

  (* put in the variant and version bits *)
  uuid.time_hi_and_version := uuid.time_hi_and_version and $0FFF;
  uuid.time_hi_and_version := uuid.time_hi_and_version or (v shl 12);
  uuid.clock_seq_hi_and_reserved := $3F;
  uuid.clock_seq_hi_and_reserved := uuid.clock_seq_hi_and_reserved or $80;
end;


{ get_current_time }

var
  time_last: uuid_time_t;
  uuids_this_tick: unsigned16 = UUIDS_PER_TICK;

<<<<<<< HEAD
<<<<<<< HEAD
procedure get_current_time(out timestamp: uuid_time_t);
=======
procedure get_current_time(var timestamp: uuid_time_t);
>>>>>>> graemeg/fixes_2_2
=======
procedure get_current_time(var timestamp: uuid_time_t);
>>>>>>> origin/fixes_2_2
var
  time_now: uuid_time_t;
begin
  while true do
  begin
    get_system_time(time_now);

    (* if clock reading changed since last UUID generated, *)
    if time_last <> time_now then
    begin
      (* reset count of uuids gen'd with this clock reading *)
      uuids_this_tick := 0;
      time_last := time_now;
      Break;
    end;

    if uuids_this_tick < UUIDS_PER_TICK then
    begin
      uuids_this_tick := uuids_this_tick + 1;
      Break;
    end;

    (* going too fast for our clock; spin *)
  end;

  (* add the count of uuids to low order bits of the clock reading *)
  timestamp := time_now + uuids_this_tick;
end;


{ get_system_time }

<<<<<<< HEAD
<<<<<<< HEAD
procedure get_system_time(out timestamp: uuid_time_t);
=======
procedure get_system_time(var timestamp: uuid_time_t);
>>>>>>> graemeg/fixes_2_2
=======
procedure get_system_time(var timestamp: uuid_time_t);
>>>>>>> origin/fixes_2_2
var
  Epoch:TDateTime;
begin
  Epoch := EncodeDateTime(1582, 10, 15, 0, 0, 0, 0);
  timestamp := 10000*MilliSecondsBetween(Epoch, Now);
end;


{ true_random }

function true_random: unsigned16;
begin
  Result := Random($10000);
end;

end.
