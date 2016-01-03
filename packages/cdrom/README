This directory contains some CD-ROM support routines for Free Pascal.

The routines are demonstrated by the 'getdiscid' and 'showcds' programs.
The latter shows the available CD-rom drives on the system, the former
calculates a disc-id and query for use with a freecddb server.

Unit cdrom
----------

The unit 'cdrom' implements a cross-platform interface for handling CD-roms.
Currently, it is implemented for Linux and Windows only.
The other units are support units, needed by various implementations.

The unit presents 2 calls:

Type 
  TTocEntry = Record 
    min, sec, frame : Integer
  end;
  PTocEntry = ^TTocEntry;

Function GetCDRomDevices(Var Devices : Array of string) : Integer;
Function ReadCDTOC(Device : String; Var CDTOC : Array of TTocEntry) : Integer;

The GetCDRomDevices call retrieves a list of available CD-ROM devices/drives
on the system. The algorithm used to detect these devices depends on the
system. The function returns the number of devices found, or a negative
number on error. The 'Devices' array is filled with strings that describe
the devices. For unix like systems, these are the device files (/dev/hdc
etc), for windows, these can be drive letters (D: etc) or ASPI Adaptor,
Target, lun IDs triples, for example ASPI[1;0;0].

(an algorithm mapping these to drive letters would be appreciated)

The ReadCDTOC call retrieves the table of contents of a CD in the CD-rom
drive (Device) This returns the number of tracks on the CD-ROM. On return,
The CDTOC array will be filled with the length of the tracks, and the frame
number, manipulated to be usable in a disc ID calculation. There will be one
extra entry at the end of the array, describing the lead-out.

unit discid
-----------

The DISCID unit calculates a DISC-ID as needed by a freecddb server to 
query for the contents of a disc.

Function CDDBDiscID(Const CDTOC : Array of TTocEntry; Count : Integer) : integer ;

The count is the number of tracks, the array should contain Count+1
elements, the last one being the lead out. The output of the ReadCDToc
function can be used as input for this function.

Function GetCDDBQueryString(Const Tracks : Array of TTocEntry; Count : Integer) : String;

This function returns a query string as expected by a freecddb server.
It consists of the disc id, number of tracks, and the frame fields of the 
TCDToc records in the tracks array.

unit lincd
----------

Essentially translation of the linux/cdrom.h include file, plus 2 functions

Function IsCDDevice(Device : String) : Boolean;

Returns True if the device file is a cdrom.

Function DetectCd : String;

Detects the first cd in the system; it does this by checking a number of
well-known common device files; the first once for which IsCDDevice returns
True is returned. Note that this can take some time.

unit major
----------

Definitions of the major device numbers under linux. used by the lincd unit.

unit wincd
----------
Some CD-routines. Essentially the low-level counterpart of the cd-rom unit.
The unit tries to use the windows ASPI layer to detect cd-roms. If the layer
is not present, the common IOCTL or SPTI interfaces are used.

TCDAccessMethod = (camNone,camASPI,camSPTI,camIOCTL);

Determines which access method is used.

Records used in the low-level calls:

TTOCTrack = packed record
  rsvd,
  ADR,
  trackNumber,
  rsvd2 : Byte;
  addr : Array[0..3] of byte;
end;  

TTOC = packed Record
  toclen : word;
  firsttrack,
  lastTrack  : byte;
  toctrack:  Array[0..99] of TTocTrack;
end;

Const
  AccessMethodNames : Array[TCDAccessMethod] of string
                    = ('None','ASPI','SPTI','IOCTL');
                    
Function  GetCDAccessMethod : TCDAccessMethod;

Returns the current CD-rom detection method.

Procedure SetCDAccessMethod (Value : TCDAccessMethod);

Sets the current CD-rom detection method.

Function  ReadTOC(Device : String; Var TOC : TTOc) : Integer;

Reads the CD-Rom devices. Device is a drive letter, or if ASPI is used
a string of the form [Aid,TgID,LunID] (Adapter ID, Target ID, LUN id)

Function  EnumCDDrives(Var Drives : Array of String) : Integer;

Returns the CD-ROM drive letters of the system, or strings describing ASPI
drives.

Function  GetNumDrives : Integer;

Returns the number of drives.

NOTE: A function mapping an ASPI triple to a drive letter would be
appreciated.

unit Wnaspi32:
--------------

Low-level ASPI unit. Contains all structures, plus the following:

  TSendASPI32Command = function( LPSRB : Pointer ) : DWORD; cdecl;
  TGetASPI32SupportInfo = function : DWORD; cdecl;

  SendASPI32Command : TSendASPI32Command = nil;
  GetASPI32SupportInfo : TGetASPI32SupportInfo = nil;

These procedural variables map to the actual API calls. They are only valid
and usable if ASPILoaded returns TRUE.

Function ASPILoaded : Boolean;

Returns TRUE if ASPI is available and loaded.

Procedure CheckASPI;

Checks whether ASPI is available. After calling this it is possible to use
the ASPIAvailable function. This procedure is called in the initialization
section of the unit.

procedure UnloadASPI;

Unloads the ASPI library if it was loaded. This procedure is called in the
finalization section of the unit.

scsidefs.pp
-----------

Bare translation of the scsidefs.h windows SDK header.

cdromioctl.pp
-------------

This unit contains some records and constants defined in window's 
cdromioctl.h


Michael.
