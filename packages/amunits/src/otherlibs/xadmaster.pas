{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 1998-2002 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for xadmaster.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  First version of this unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT XADMASTER;

INTERFACE
USES Exec,utility,amigados;

const
    XADMASTERNAME : PChar = 'xadmaster.library';


  {
        $VER: xadmaster.h 12.0 (24.06.2002)
        xadmaster.library defines and structures

        Copyright © 1998-2002 by Dirk Stöcker
        All Rights Reserved.
   }


  const
     XADNAME = 'xadmaster.library';
  { NOTE: Nearly all structures need to be allocated using the
     xadAllocObject function.  }
  {

       library base structure

                                                                          }



  type
     PxadMasterBase = ^txadMasterBase;
     txadMasterBase = record
          xmb_LibNode : tLibrary;
          xmb_SysBase : PExecBase;
          xmb_DOSBase : PDosLibrary;
          xmb_UtilityBase : PUtilityBase;
          xmb_RecogSize : ULONG;      { read only  }
          xmb_DefaultName : STRPTR;   { name for XADFIF_NOFILENAME (V6)  }
       end;

  {

       tag-function call flags

                                                                          }
  { input tags for xadGetInfo, only one can be specified per call  }
  const
   { input data size  }
     XAD_INSIZE = TAG_USER + 1;
     XAD_INFILENAME = TAG_USER + 2;
     XAD_INFILEHANDLE = TAG_USER + 3;
     XAD_INMEMORY = TAG_USER + 4;
     XAD_INHOOK = TAG_USER + 5;
  { (V2)  }
     XAD_INSPLITTED = TAG_USER + 6;
  { (V4)  }
     XAD_INDISKARCHIVE = TAG_USER + 7;
  { (V8)  }
     XAD_INXADSTREAM = TAG_USER + 8;
  { (V11)  }
     XAD_INDEVICE = TAG_USER + 9;
  { output tags, only one can be specified per call, xadXXXXUnArc  }
  { output data size  }
     XAD_OUTSIZE = TAG_USER + 10;
     XAD_OUTFILENAME = TAG_USER + 11;
     XAD_OUTFILEHANDLE = TAG_USER + 12;
     XAD_OUTMEMORY = TAG_USER + 13;
     XAD_OUTHOOK = TAG_USER + 14;
     XAD_OUTDEVICE = TAG_USER + 15;
  { (V8)  }
     XAD_OUTXADSTREAM = TAG_USER + 16;
  { object allocation tags for xadAllocObjectA  }
  { XADOBJ_FILEINFO, size of needed name space  }
     XAD_OBJNAMESIZE = TAG_USER + 20;
  { XADOBJ_FILEINFO, size of needed comment space  }
     XAD_OBJCOMMENTSIZE = TAG_USER + 21;
  { XADOBJ_FILEINFO & XADOBJ_DISKINFO, self use size  }
     XAD_OBJPRIVINFOSIZE = TAG_USER + 22;
  { XADOBJ_DISKINFO, number of needed entries  }
     XAD_OBJBLOCKENTRIES = TAG_USER + 23;
  { tags for xadGetInfo, xadFileUnArc and xadDiskUnArc  }
  { do not use extern clients  }
     XAD_NOEXTERN = TAG_USER + 50;
  { password when needed  }
     XAD_PASSWORD = TAG_USER + 51;
  { number of wanted entry  }
     XAD_ENTRYNUMBER = TAG_USER + 52;
  { the progress hook  }
     XAD_PROGRESSHOOK = TAG_USER + 53;
  { overwrite file ?  }
     XAD_OVERWRITE = TAG_USER + 54;
  { create directory tree  }
     XAD_MAKEDIRECTORY = TAG_USER + 55;
  { ignore drive geometry ?  }
     XAD_IGNOREGEOMETRY = TAG_USER + 56;
  { lowest cylinder  }
     XAD_LOWCYLINDER = TAG_USER + 57;
  { highest cylinder  }
     XAD_HIGHCYLINDER = TAG_USER + 58;
  { verify for disk hook  }
     XAD_VERIFY = TAG_USER + 59;
  { do not delete partial/corrupt files (V3.3)  }
     XAD_NOKILLPARTIAL = TAG_USER + 60;
  { format output device (V5)  }
     XAD_FORMAT = TAG_USER + 61;
  { sector labels are stored on disk (V9)  }
     XAD_USESECTORLABELS = TAG_USER + 62;
  { ignore the client, if certain flags are set (V11)  }
     XAD_IGNOREFLAGS = TAG_USER + 63;
  { ignore the client, if certain flags are NOT set (V11)  }
     XAD_ONLYFLAGS = TAG_USER + 64;
  { input tags for xadConvertDates, only one can be passed  }
  { unix date variable  }
     XAD_DATEUNIX = TAG_USER + 70;
  { amiga date variable  }
     XAD_DATEAMIGA = TAG_USER + 71;
  { struct DateStamp  }
     XAD_DATEDATESTAMP = TAG_USER + 72;
  { struct xadDate  }
     XAD_DATEXADDATE = TAG_USER + 73;
  { struct ClockData  }
     XAD_DATECLOCKDATA = TAG_USER + 74;
  { input is system time  }
     XAD_DATECURRENTTIME = TAG_USER + 75;
  { MS-DOS packed format (V2)  }
     XAD_DATEMSDOS = TAG_USER + 76;
  { Mac date variable (V8)  }
     XAD_DATEMAC = TAG_USER + 77;
  { CP/M data structure (V10)  }
     XAD_DATECPM = TAG_USER + 78;
  { CP/M data structure type 2 (V10)  }
     XAD_DATECPM2 = TAG_USER + 79;
  { ISO9660 date structure (V11)  }
     XAD_DATEISO9660 = TAG_USER + 300;
  { output tags, there can be specified multiple tags for one call  }
  { unix date variable  }
     XAD_GETDATEUNIX = TAG_USER + 80;
  { amiga date variable  }
     XAD_GETDATEAMIGA = TAG_USER + 81;
  { struct DateStamp  }
     XAD_GETDATEDATESTAMP = TAG_USER + 82;
  { struct xadDate  }
     XAD_GETDATEXADDATE = TAG_USER + 83;
  { struct ClockData  }
     XAD_GETDATECLOCKDATA = TAG_USER + 84;
  { MS-DOS packed format (V2)  }
     XAD_GETDATEMSDOS = TAG_USER + 86;
  { Mac date variable (V8)  }
     XAD_GETDATEMAC = TAG_USER + 87;
  { CP/M data structure (V10)  }
     XAD_GETDATECPM = TAG_USER + 88;
  { CP/M data structure type 2 (V10)  }
     XAD_GETDATECPM2 = TAG_USER + 89;
  { ISO9660 date structure (V11)  }
     XAD_GETDATEISO9660 = TAG_USER + 320;
  { following tags need locale.library to be installed  }
  { make local to GMT time  }
     XAD_MAKEGMTDATE = TAG_USER + 90;
  { make GMT to local time  }
     XAD_MAKELOCALDATE = TAG_USER + 91;
  { tags for xadHookTagAccess (V3)  }
  { the hook uses xadSkipInfo (V3)  }
     XAD_USESKIPINFO = TAG_USER + 104;
  { pass sector labels with XADAC_WRITE (V9)  }
     XAD_SECTORLABELS = TAG_USER + 105;
  { pointer to UWORD value (V3)  }
     XAD_GETCRC16 = TAG_USER + 120;
  { pointer to ULONG value (V3)  }
     XAD_GETCRC32 = TAG_USER + 121;
  { ID for crc calculation (V3)  }
     XAD_CRC16ID = TAG_USER + 130;
  { ID for crc calculation (V3)  }
     XAD_CRC32ID = TAG_USER + 131;
  { tags for xadConvertProtection (V4)  }
  { Amiga type protection bits (V4)  }
     XAD_PROTAMIGA = TAG_USER + 160;
  { protection bits in UNIX mode (V4)  }
     XAD_PROTUNIX = TAG_USER + 161;
  { MSDOS type protection bits (V4)  }
     XAD_PROTMSDOS = TAG_USER + 162;
  { input is a xadFileInfo structure (V11)  }
     XAD_PROTFILEINFO = TAG_USER + 163;
  { return Amiga protection bits (V4)  }
     XAD_GETPROTAMIGA = TAG_USER + 170;
  { return UNIX protection bits (V11)  }
     XAD_GETPROTUNIX = TAG_USER + 171;
  { return MSDOS protection bits (V11)  }
     XAD_GETPROTMSDOS = TAG_USER + 172;
  { fill xadFileInfo protection fields (V11)  }
     XAD_GETPROTFILEINFO = TAG_USER + 173;
  { tags for xadGetDiskInfo (V7)  }
  { the client to start with (V7)  }
     XAD_STARTCLIENT = TAG_USER + 180;
  { do not create XADERR_EMPTY (V8)  }
     XAD_NOEMPTYERROR = TAG_USER + 181;
  { tags for xadFreeHookAccess (V8)  }
  { error occured, call abort method (V8)  }
     XAD_WASERROR = TAG_USER + 190;
  { tags for miscellaneous stuff  }
  { xadArchiveInfo for stream hooks (V8)  }
     XAD_ARCHIVEINFO = TAG_USER + 200;
  { error code of function (V12)  }
     XAD_ERRORCODE = TAG_USER + 201;
  { tags for xadAddFileEntry and xadAddDiskEntry (V10)  }
  { set xai_InPos after call (V10)  }
     XAD_SETINPOS = TAG_USER + 240;
  { insert dirs at list start (V10)  }
     XAD_INSERTDIRSFIRST = TAG_USER + 241;
  { tags for xadConvertName (V12)  }
  (* UWORD  , default is {'/','\\',0} in source charset (V12)  *)
     XAD_PATHSEPERATOR = TAG_USER + 260;
  { the characterset of string (V12)  }
     XAD_CHARACTERSET = TAG_USER + 261;
  { maximum size of following (V12)  }
     XAD_STRINGSIZE = TAG_USER + 262;
  { zero-terminated string (V12)  }
     XAD_CSTRING = TAG_USER + 263;
  { lengthed Pascal string (V12)  }
     XAD_PSTRING = TAG_USER + 264;
  { an xad string (V12)  }
     XAD_XADSTRING = TAG_USER + 265;
  { default is TRUE (V12)  }
     XAD_ADDPATHSEPERATOR = TAG_USER + 266;
  { tags for xadGetFilename (V12)  }
  { default is FALSE (V12)  }
     XAD_NOLEADINGPATH = TAG_USER + 280;
  { default is FALSE (V12)  }
     XAD_NOTRAILINGPATH = TAG_USER + 281;
  { default are #?()[]~% :|",1-31,127-160 (V12)  }
     XAD_MASKCHARACTERS = TAG_USER + 282;
  { default is '_' (V12)  }
     XAD_MASKINGCHAR = TAG_USER + 283;
  { pointer which should hold buf size (V12)  }
     XAD_REQUIREDBUFFERSIZE = TAG_USER + 284;
  { Places 300-339 used for dates!  }
  {

       objects for xadAllocObjectA

                                                                          }
  { struct xadArchiveInfo  }
     XADOBJ_ARCHIVEINFO = $0001;
  { struct xadFileInfo  }
     XADOBJ_FILEINFO = $0002;
  { struct xadDiskInfo  }
     XADOBJ_DISKINFO = $0003;
  { struct HookParam  }
     XADOBJ_HOOKPARAM = $0004;
  { struct xadDeviceInfo  }
     XADOBJ_DEVICEINFO = $0005;
  { struct xadProgressInfo  }
     XADOBJ_PROGRESSINFO = $0006;
  { struct xadTextInfo  }
     XADOBJ_TEXTINFO = $0007;
  { struct xadSplitFile (V2)  }
     XADOBJ_SPLITFILE = $0008;
  { struct xadSkipInfo (V3)  }
     XADOBJ_SKIPINFO = $0009;
  { struct xadImageInfo (V4)  }
     XADOBJ_IMAGEINFO = $000A;
  { struct xadSpecial (V11)  }
     XADOBJ_SPECIAL = $000B;
  { result type of xadAllocVec  }
  { memory of requested size and type  }
     XADOBJ_MEMBLOCK = $0100;
  { private type  }
  { an typed XAD string (V12)  }
     XADOBJ_STRING = $0101;
  {

       modes for xadCalcCRC126 and xadCalcCRC32

                                                                          }
     XADCRC16_ID1 = $A001;
     XADCRC32_ID1 = $EDB88320;
  {

       hook related stuff

                                                                          }
  { read data into buffer  }
     XADHC_READ = 1;
  { write buffer data to file/memory  }
     XADHC_WRITE = 2;
  { seek in file  }
     XADHC_SEEK = 3;
  { initialize the hook  }
     XADHC_INIT = 4;
  { end up hook work, free stuff  }
     XADHC_FREE = 5;
  { an error occured, delete partial stuff  }
     XADHC_ABORT = 6;
  { complete input size is needed  }
     XADHC_FULLSIZE = 7;
  { return disk image info (V4)  }
     XADHC_IMAGEINFO = 8;

  type
     PxadHookParam = ^txadHookParam;
     txadHookParam = record
          xhp_Command : ULONG;
          xhp_CommandData : LONG;
          xhp_BufferPtr : APTR;
          xhp_BufferSize : ULONG;
          xhp_DataPos : ULONG;     { current seek position  }
          xhp_PrivatePtr : APTR;
          xhp_TagList : APTR;      { allows to transport tags to hook (V9)  }
       end;

  { xadHookAccess commands  }
  { get data  }

  const
     XADAC_READ = 10;
  { write data  }
     XADAC_WRITE = 11;
  { copy input to output  }
     XADAC_COPY = 12;
  { seek in input file  }
     XADAC_INPUTSEEK = 13;
  { seek in output file  }
     XADAC_OUTPUTSEEK = 14;
  {

       support structures

                                                                          }
  { Own date structure to cover all possible dates in a human friendly
     format. xadConvertDates may be used to convert between different date
     structures and variables.  }

  type
     PxadDate = ^txadDate;
     txadDate = record
          xd_Micros : ULONG;  { values 0 to 999999         }
          xd_Year : LONG;     { values 1 to 2147483648  }
          xd_Month : UBYTE;   { values 1 to 12     }
          xd_WeekDay : UBYTE; { values 1 to 7      }
          xd_Day : UBYTE;     { values 1 to 31     }
          xd_Hour : UBYTE;    { values 0 to 23     }
          xd_Minute : UBYTE;  { values 0 to 59     }
          xd_Second : UBYTE;  { values 0 to 59     }
       end;

  { monday is the first day and  }

  const
     XADDAY_MONDAY = 1;
     XADDAY_TUESDAY = 2;
     XADDAY_WEDNESDAY = 3;
     XADDAY_THURSDAY = 4;
     XADDAY_FRIDAY = 5;
     XADDAY_SATURDAY = 6;
  { sunday the last day of a week  }
     XADDAY_SUNDAY = 7;

  type
     PxadDeviceInfo = ^txadDeviceInfo;
     txadDeviceInfo = record         { for XAD_OUTDEVICE tag  }
          xdi_DeviceName : STRPTR;   { name of device  }
          xdi_Unit : ULONG;          { unit of device  }
          xdi_DOSName : STRPTR;      { instead of Device+Unit, dos name without ':'  }
       end;

     PxadSplitFile = ^txadSplitFile;
     txadSplitFile = record           { for XAD_INSPLITTED  }
          xsf_Next : PxadSplitFile;
          xsf_Type : ULONG;           { XAD_INFILENAME, XAD_INFILEHANDLE, XAD_INMEMORY, XAD_INHOOK  }
          xsf_Size : ULONG;           { necessary for XAD_INMEMORY, useful for others  }
          xsf_Data : ULONG;           { FileName, Filehandle, Hookpointer or Memory  }
       end;

     PxadSkipInfo = ^txadSkipInfo;
     txadSkipInfo = record
          xsi_Next : PxadSkipInfo;
          xsi_Position : ULONG;     { position, where it should be skipped  }
          xsi_SkipSize : ULONG;     { size to skip  }
       end;

     PxadImageInfo = ^txadImageInfo;
     txadImageInfo = record          { for XADHC_IMAGEINFO  }
          xii_SectorSize : ULONG;    { usually 512  }
          xii_FirstSector : ULONG;   { of the image file  }
          xii_NumSectors : ULONG;    { of the image file  }
          xii_TotalSectors : ULONG;  { of this device type  }
       end;

   type
     PxadClient = ^txadClient;
     txadClient = record
          xc_Next : PxadClient;
          xc_Version : UWORD;         { set to XADCLIENT_VERSION  }
          xc_MasterVersion : UWORD;
          xc_ClientVersion : UWORD;
          xc_ClientRevision : UWORD;
          xc_RecogSize : ULONG;       { needed size to recog the type  }
          xc_Flags : ULONG;           { see XADCF_xxx defines  }
          xc_Identifier : ULONG;      { ID of internal clients  }
          xc_ArchiverName : STRPTR;
          xc_RecogData : function :BOOL;
          xc_GetInfo : function :LONG;
          xc_UnArchive : function :LONG;
          xc_Free : procedure ;
       end;

  { function interface
  ASM(BOOL) xc_RecogData(REG(d0, ULONG size), REG(a0, STRPTR data),
                REG(a6, struct xadMasterBase  xadMasterBase));
  ASM(LONG) xc_GetInfo(REG(a0, struct xadArchiveInfo  ai),
                REG(a6, struct xadMasterBase  xadMasterBase));
  ASM(LONG) xc_UnArchive(REG(a0, struct xadArchiveInfo  ai),
                REG(a6, struct xadMasterBase  xadMasterBase));
  ASM(void) xc_Free(REG(a0, struct xadArchiveInfo  ai),
                REG(a6, struct xadMasterBase  xadMasterBase));
   }
  { xc_RecogData returns 1 when recognized and 0 when not, all the others
     return 0 when ok and XADERR values on error. xc_Free has no return
     value.

     Filesystem clients need to clear xc_RecogSize and xc_RecogData. The
     recognition is automatically done by GetInfo. XADERR_FILESYSTEM is
     returned in case of unknown format. If it is known detection should
     go on and any other code may be returned, if it fails.
     The field xc_ArchiverName means xc_FileSystemName for filesystem
     clients.
   }

  type
     PxadSpecialUnixDevice = ^txadSpecialUnixDevice;
     txadSpecialUnixDevice = record
          xfis_MajorVersion : ULONG;    { major device version  }
          xfis_MinorVersion : ULONG;    { minor device version  }
       end;

     PxadSpecialAmigaAddress = ^txadSpecialAmigaAddress;
     txadSpecialAmigaAddress = record
          xfis_JumpAddress : ULONG;      { code executaion start address  }
          xfis_DecrunchAddress : ULONG;  { decrunch start of code  }
       end;

     PxadSpecialCBM8bit = ^txadSpecialCBM8bit;
     txadSpecialCBM8bit = record
          xfis_FileType : UBYTE;       { File type XADCBM8BITTYPE_xxx  }
          xfis_RecordLength : UBYTE;   { record length if relative file  }
       end;

  type
     PxadSpecial = ^txadSpecial;
     txadSpecial = record
          xfis_Type : ULONG;           { XADSPECIALTYPE to define type of block (V11)  }
          xfis_Next : PxadSpecial;     { pointer to next entry  }
          xfis_Data : record
              case longint of
                 0 : ( xfis_UnixDevice : txadSpecialUnixDevice );
                 1 : ( xfis_AmigaAddress : txadSpecialAmigaAddress );
                 2 : ( xfis_CBM8bit : txadSpecialCBM8bit );
              end;
       end;

  { Multiuser fields (xfi_OwnerUID, xfi_OwnerUID, xfi_UserName, xfi_GroupName)
     and multiuser bits (see <dos/dos.h>) are currently not supported with normal
     Amiga filesystem. But the clients support them, if archive format holds
     such information.

     The protection bits (all 3 fields) should always be set using the
     xadConvertProtection procedure. Call it with as much protection information
     as possible. It extracts the relevant data at best (and also sets the 2 flags).
     DO NOT USE these fields directly, but always through xadConvertProtection
     call.
   }

  type
     PxadFileInfo = ^txadFileInfo;
     txadFileInfo = record
          xfi_Next : PxadFileInfo;
          xfi_EntryNumber : ULONG;       { number of entry  }
          xfi_EntryInfo : STRPTR;        { additional archiver text  }
          xfi_PrivateInfo : APTR;        { client private, see XAD_OBJPRIVINFOSIZE  }
          xfi_Flags : ULONG;             { see XADFIF_xxx defines  }
          xfi_FileName : STRPTR;         { see XAD_OBJNAMESIZE tag  }
          xfi_Comment : STRPTR;          { see XAD_OBJCOMMENTSIZE tag  }
          xfi_Protection : ULONG;        { OS 3 bits (including multiuser)  }
          xfi_OwnerUID : ULONG;          { user ID  }
          xfi_OwnerGID : ULONG;          { group ID  }
          xfi_UserName : STRPTR;         { user name  }
          xfi_GroupName : STRPTR;        { group name  }
          xfi_Size : ULONG;              { size of this file  }
          xfi_GroupCrSize : ULONG;       { crunched size of group  }
          xfi_CrunchSize : ULONG;        { crunched size  }
          xfi_LinkName : STRPTR;         { name and path of link  }
          xfi_Date : txadDate;
          xfi_Generation : UWORD;        { File Generation [0...0xFFFF] (V3)  }
          xfi_DataPos : ULONG;           { crunched data position (V3)  }
          xfi_MacFork : PxadFileInfo;    { pointer to 2nd fork for Mac (V7)  }
          xfi_UnixProtect : UWORD;       { protection bits for Unix (V11)  }
          xfi_DosProtect : UBYTE;        { protection bits for MS-DOS (V11)  }
          xfi_FileType : UBYTE;          { XADFILETYPE to define type of exe files (V11)  }
          xfi_Special : PxadSpecial;     { pointer to special data (V11)  }
       end;

 { NOTE: the texts passed with that structure must not always be printable.
     Although the clients should add an additional (not counted) zero at the text
     end, the whole file may contain other unprintable stuff (e.g. for DMS).
     So when printing this texts do it on a byte for byte base including
     printability checks.
   }

 type
     PxadTextInfo = ^txadTextInfo;
     txadTextInfo = record
          xti_Next : PxadTextInfo;
          xti_Size : ULONG;      { maybe zero - no text - e.g. when crypted  }
          xti_Text : STRPTR;     { and there is no password in xadGetInfo()  }
          xti_Flags : ULONG;     { see XADTIF_xxx defines  }

       end;

  type
     PxadDiskInfo = ^txadDiskInfo;
     txadDiskInfo = record
          xdi_Next : PxadDiskInfo;
          xdi_EntryNumber : ULONG;    { number of entry  }
          xdi_EntryInfo : STRPTR;     { additional archiver text  }
          xdi_PrivateInfo : APTR;     { client private, see XAD_OBJPRIVINFOSIZE  }
          xdi_Flags : ULONG;          { see XADDIF_xxx defines  }
          xdi_SectorSize : ULONG;
          xdi_TotalSectors : ULONG;   { see devices/trackdisk.h  }
          xdi_Cylinders : ULONG;      { to find out what these  }
          xdi_CylSectors : ULONG;     { fields mean, they are equal  }
          xdi_Heads : ULONG;          { to struct DriveGeometry  }
          xdi_TrackSectors : ULONG;
          xdi_LowCyl : ULONG;         { lowest cylinder stored  }
          xdi_HighCyl : ULONG;        { highest cylinder stored  }
          xdi_BlockInfoSize : ULONG;  { number of BlockInfo entries  }
          xdi_BlockInfo : Pointer;    { see XADBIF_xxx defines and XAD_OBJBLOCKENTRIES tag  }
          xdi_TextInfo : PxadTextInfo;{ linked list with info texts  }
          xdi_DataPos : ULONG;        { crunched data position (V3)  }
       end;

  { BlockInfo points to a UBYTE field for every track from first sector of
     lowest cylinder to last sector of highest cylinder. When not used,
     pointer must be 0. Do not use it, when there are no entries!
     This is just for information. The applications still asks the client
     to unarchive whole cylinders and not archived blocks are cleared for
     unarchiving.
   }

  { If the image file holds total data of disk xii_TotalSectors equals
     xii_NumSectors and xii_FirstSector is zero. Addition of xii_FirstSector
     and xii_NumSectors cannot exceed xii_TotalSectors value!
   }
  {

       information structures

                                                                          }
     PxadArchiveInfo = ^txadArchiveInfo;
     txadArchiveInfo = record
          xai_Client : PxadClient;         { pointer to unarchiving client  }
          xai_PrivateClient : APTR;        { private client data  }
          xai_Password : STRPTR;           { password for crypted archives  }
          xai_Flags : ULONG;               { read only XADAIF_ flags  }
          xai_LowCyl : ULONG;              { lowest cylinder to unarchive  }
          xai_HighCyl : ULONG;             { highest cylinder to unarchive  }
          xai_InPos : ULONG;               { input position, read only  }
          xai_InSize : ULONG;              { input size, read only  }
          xai_OutPos : ULONG;              { output position, read only  }
          xai_OutSize : ULONG;             { output file size, read only  }
          xai_FileInfo : PxadFileInfo;     { data pointer for file arcs  }
          xai_DiskInfo : PxadDiskInfo;     { data pointer for disk arcs  }
          xai_CurFile : PxadFileInfo;      { data pointer for current file arc  }
          xai_CurDisk : PxadDiskInfo;      { data pointer for current disk arc  }
          xai_LastError : LONG;            { last error, when XADAIF_FILECORRUPT (V2)  }
          xai_MultiVolume : PULONG;        { array of start offsets from parts (V2)  }
          xai_SkipInfo : PxadSkipInfo;     { linked list of skip entries (V3)  }
          xai_ImageInfo : PxadImageInfo;   { for filesystem clients (V5)  }
          xai_InName : STRPTR;             { Input archive name if available (V7)  }
       end;



  { This structure is nearly complete private to either xadmaster or its
  clients. An application program may access for reading only xai_Client,
  xai_Flags, xai_FileInfo and xai_DiskInfo. For xai_Flags only XADAIF_CRYPTED
  and XADAIF_FILECORRUPT are useful. All the other stuff is private and should
  not be accessed!  }
  { archive entries are encrypted  }

  const
     XADAIB_CRYPTED = 0;
  { file is corrupt, but valid entries are in the list  }
     XADAIB_FILECORRUPT = 1;
  { unarchive file entry  }
     XADAIB_FILEARCHIVE = 2;
  { unarchive disk entry  }
     XADAIB_DISKARCHIVE = 3;
  { overwrite the file (PRIVATE)  }
     XADAIB_OVERWRITE = 4;
  { create directory when missing (PRIVATE)  }
     XADAIB_MAKEDIRECTORY = 5;
  { ignore drive geometry (PRIVATE)  }
     XADAIB_IGNOREGEOMETRY = 6;
  { verify is turned on for disk hook (PRIVATE)  }
     XADAIB_VERIFY = 7;
  { do not delete partial files (PRIVATE)  }
     XADAIB_NOKILLPARTIAL = 8;
  { is disk image extraction (V5)  }
     XADAIB_DISKIMAGE = 9;
  { format in disk hook (PRIVATE)  }
     XADAIB_FORMAT = 10;
  { do not create empty error (PRIVATE)  }
     XADAIB_NOEMPTYERROR = 11;
  { in stuff only (PRIVATE)  }
     XADAIB_ONLYIN = 12;
  { out stuff only (PRIVATE)  }
     XADAIB_ONLYOUT = 13;
  { use SectorLabels (PRIVATE)  }
     XADAIB_USESECTORLABELS = 14;
     XADAIF_CRYPTED = 1 shl XADAIB_CRYPTED;
     XADAIF_FILECORRUPT = 1 shl XADAIB_FILECORRUPT;
     XADAIF_FILEARCHIVE = 1 shl XADAIB_FILEARCHIVE;
     XADAIF_DISKARCHIVE = 1 shl XADAIB_DISKARCHIVE;
     XADAIF_OVERWRITE = 1 shl XADAIB_OVERWRITE;
     XADAIF_MAKEDIRECTORY = 1 shl XADAIB_MAKEDIRECTORY;
     XADAIF_IGNOREGEOMETRY = 1 shl XADAIB_IGNOREGEOMETRY;
     XADAIF_VERIFY = 1 shl XADAIB_VERIFY;
     XADAIF_NOKILLPARTIAL = 1 shl XADAIB_NOKILLPARTIAL;
     XADAIF_DISKIMAGE = 1 shl XADAIB_DISKIMAGE;
     XADAIF_FORMAT = 1 shl XADAIB_FORMAT;
     XADAIF_NOEMPTYERROR = 1 shl XADAIB_NOEMPTYERROR;
     XADAIF_ONLYIN = 1 shl XADAIB_ONLYIN;
     XADAIF_ONLYOUT = 1 shl XADAIB_ONLYOUT;
     XADAIF_USESECTORLABELS = 1 shl XADAIB_USESECTORLABELS;

  { These are used for xfi_FileType to define file type. (V11)  }
  { infile was only one data file  }

  const
     XADFILETYPE_DATACRUNCHER = 1;
  { infile was text-linked  }
     XADFILETYPE_TEXTLINKER = 2;
  { infile was an Amiga exe cruncher  }
     XADFILETYPE_AMIGAEXECRUNCHER = 11;
  { infile was an Amiga exe linker  }
     XADFILETYPE_AMIGAEXELINKER = 12;
  { infile was an Amiga text-exe linker  }
     XADFILETYPE_AMIGATEXTLINKER = 13;
  { infile was an Amiga address cruncher  }
     XADFILETYPE_AMIGAADDRESS = 14;
  { this file is a block device  }
     XADFILETYPE_UNIXBLOCKDEVICE = 21;
  { this file is a character device  }
     XADFILETYPE_UNIXCHARDEVICE = 22;
  { this file is a named pipe  }
     XADFILETYPE_UNIXFIFO = 23;
  { this file is a socket  }
     XADFILETYPE_UNIXSOCKET = 24;
  { infile was an MSDOS exe cruncher  }
     XADFILETYPE_MSDOSEXECRUNCHER = 31;
  { xadSpecial entry is xadSpecialUnixDevice  }
     XADSPECIALTYPE_UNIXDEVICE = 1;
  { xadSpecial entry is xadSpecialAmigaAddress  }
     XADSPECIALTYPE_AMIGAADDRESS = 2;
  { xadSpecial entry is xadSpecialCBM8bit  }
     XADSPECIALTYPE_CBM8BIT = 3;



  {       Unknown / Unused  }

  const
     XADCBM8BITTYPE_UNKNOWN = $00;
  { Tape - BASIC program file  }
     XADCBM8BITTYPE_BASIC = $01;
  { Tape - Data block (SEQ file)  }
     XADCBM8BITTYPE_DATA = $02;
  { Tape - Fixed addres program file  }
     XADCBM8BITTYPE_FIXED = $03;
  { Tape - Sequential data file  }
     XADCBM8BITTYPE_SEQDATA = $04;
  { Disk - Sequential file "SEQ"  }
     XADCBM8BITTYPE_SEQ = $81;
  { Disk - Program file "PRG"  }
     XADCBM8BITTYPE_PRG = $82;
  { Disk - User-defined file "USR"  }
     XADCBM8BITTYPE_USR = $83;
  { Disk - Relative records file "REL"  }
     XADCBM8BITTYPE_REL = $84;
  { Disk - CBM (partition) "CBM"  }
     XADCBM8BITTYPE_CBM = $85;



  { entry is crypted  }

  const
     XADFIB_CRYPTED = 0;
  { entry is a directory  }
     XADFIB_DIRECTORY = 1;
  { entry is a link  }
     XADFIB_LINK = 2;
  { file is an information text  }
     XADFIB_INFOTEXT = 3;
  { file is in a crunch group  }
     XADFIB_GROUPED = 4;
  { crunch group ends here  }
     XADFIB_ENDOFGROUP = 5;
  { no date supported, CURRENT date is set  }
     XADFIB_NODATE = 6;
  { file is marked as deleted (V3)  }
     XADFIB_DELETED = 7;
  { before unarchiving the datapos is set (V3)  }
     XADFIB_SEEKDATAPOS = 8;
  { there was no filename, using internal one (V6)  }
     XADFIB_NOFILENAME = 9;
  { file size is unknown and thus set to zero (V6)  }
     XADFIB_NOUNCRUNCHSIZE = 10;
  { file is only partial (V6)  }
     XADFIB_PARTIALFILE = 11;
  { file is Apple data fork (V7)  }
     XADFIB_MACDATA = 12;
  { file is Apple resource fork (V7)  }
     XADFIB_MACRESOURCE = 13;
  { allows extract file during scanning (V10)  }
     XADFIB_EXTRACTONBUILD = 14;
  { UNIX protection bits are present (V11)  }
     XADFIB_UNIXPROTECTION = 15;
  { MSDOS protection bits are present (V11)  }
     XADFIB_DOSPROTECTION = 16;
  { this entry may change until GetInfo is finished (V11)  }
     XADFIB_ENTRYMAYCHANGE = 17;
  { the xfi_FileName fields is an XAD string (V12)  }
     XADFIB_XADSTRFILENAME = 18;
  { the xfi_LinkName fields is an XAD string (V12)  }
     XADFIB_XADSTRLINKNAME = 19;
  { the xfi_Comment fields is an XAD string (V12)  }
     XADFIB_XADSTRCOMMENT = 20;
     XADFIF_CRYPTED = 1 shl XADFIB_CRYPTED;
     XADFIF_DIRECTORY = 1 shl XADFIB_DIRECTORY;
     XADFIF_LINK = 1 shl XADFIB_LINK;
     XADFIF_INFOTEXT = 1 shl XADFIB_INFOTEXT;
     XADFIF_GROUPED = 1 shl XADFIB_GROUPED;
     XADFIF_ENDOFGROUP = 1 shl XADFIB_ENDOFGROUP;
     XADFIF_NODATE = 1 shl XADFIB_NODATE;
     XADFIF_DELETED = 1 shl XADFIB_DELETED;
     XADFIF_SEEKDATAPOS = 1 shl XADFIB_SEEKDATAPOS;
     XADFIF_NOFILENAME = 1 shl XADFIB_NOFILENAME;
     XADFIF_NOUNCRUNCHSIZE = 1 shl XADFIB_NOUNCRUNCHSIZE;
     XADFIF_PARTIALFILE = 1 shl XADFIB_PARTIALFILE;
     XADFIF_MACDATA = 1 shl XADFIB_MACDATA;
     XADFIF_MACRESOURCE = 1 shl XADFIB_MACRESOURCE;
     XADFIF_EXTRACTONBUILD = 1 shl XADFIB_EXTRACTONBUILD;
     XADFIF_UNIXPROTECTION = 1 shl XADFIB_UNIXPROTECTION;
     XADFIF_DOSPROTECTION = 1 shl XADFIB_DOSPROTECTION;
     XADFIF_ENTRYMAYCHANGE = 1 shl XADFIB_ENTRYMAYCHANGE;
     XADFIF_XADSTRFILENAME = 1 shl XADFIB_XADSTRFILENAME;
     XADFIF_XADSTRLINKNAME = 1 shl XADFIB_XADSTRLINKNAME;
     XADFIF_XADSTRCOMMENT = 1 shl XADFIB_XADSTRCOMMENT;



  { entry is empty, as data was crypted  }

  const
     XADTIB_CRYPTED = 0;
  { text is a banner  }
     XADTIB_BANNER = 1;
  { text is a file description  }
     XADTIB_FILEDIZ = 2;
     XADTIF_CRYPTED = 1 shl XADTIB_CRYPTED;
     XADTIF_BANNER = 1 shl XADTIB_BANNER;
     XADTIF_FILEDIZ = 1 shl XADTIB_FILEDIZ;

  { entry is crypted  }

  const
     XADDIB_CRYPTED = 0;
  { before unarchiving the datapos is set (V3)  }
     XADDIB_SEEKDATAPOS = 1;
  { the clients delivers sector labels (V9)  }
     XADDIB_SECTORLABELS = 2;
  { allows extract disk during scanning (V10)  }
     XADDIB_EXTRACTONBUILD = 3;
  { this entry may change until GetInfo is finished (V11)  }
     XADDIB_ENTRYMAYCHANGE = 4;
  { Some of the crunchers do not store all necessary information, so it
  may be needed to guess some of them. Set the following flags in that case
  and geometry check will ignore these fields.  }
  { sectorsize is guessed (V10)  }
     XADDIB_GUESSSECTORSIZE = 5;
  { totalsectors number is guessed (V10)  }
     XADDIB_GUESSTOTALSECTORS = 6;
  { cylinder number is guessed  }
     XADDIB_GUESSCYLINDERS = 7;
  { cylsectors is guessed  }
     XADDIB_GUESSCYLSECTORS = 8;
  { number of heads is guessed  }
     XADDIB_GUESSHEADS = 9;
  { tracksectors is guessed  }
     XADDIB_GUESSTRACKSECTORS = 10;
  { lowcyl is guessed  }
     XADDIB_GUESSLOWCYL = 11;
  { highcyl is guessed  }
     XADDIB_GUESSHIGHCYL = 12;
  { If it is impossible to set some of the fields, you need to set some of
  these flags. NOTE: XADDIB_NOCYLINDERS is really important, as this turns
  of usage of lowcyl and highcyl keywords. When you have cylinder information,
  you should not use these and instead use guess flags and calculate
  possible values for the missing fields.  }
  { cylinder number is not set  }
     XADDIB_NOCYLINDERS = 15;
  { cylsectors is not set  }
     XADDIB_NOCYLSECTORS = 16;
  { number of heads is not set  }
     XADDIB_NOHEADS = 17;
  { tracksectors is not set  }
     XADDIB_NOTRACKSECTORS = 18;
  { lowcyl is not set  }
     XADDIB_NOLOWCYL = 19;
  { highcyl is not set  }
     XADDIB_NOHIGHCYL = 20;
     XADDIF_CRYPTED = 1 shl XADDIB_CRYPTED;
     XADDIF_SEEKDATAPOS = 1 shl XADDIB_SEEKDATAPOS;
     XADDIF_SECTORLABELS = 1 shl XADDIB_SECTORLABELS;
     XADDIF_EXTRACTONBUILD = 1 shl XADDIB_EXTRACTONBUILD;
     XADDIF_ENTRYMAYCHANGE = 1 shl XADDIB_ENTRYMAYCHANGE;
     XADDIF_GUESSSECTORSIZE = 1 shl XADDIB_GUESSSECTORSIZE;
     XADDIF_GUESSTOTALSECTORS = 1 shl XADDIB_GUESSTOTALSECTORS;
     XADDIF_GUESSCYLINDERS = 1 shl XADDIB_GUESSCYLINDERS;
     XADDIF_GUESSCYLSECTORS = 1 shl XADDIB_GUESSCYLSECTORS;
     XADDIF_GUESSHEADS = 1 shl XADDIB_GUESSHEADS;
     XADDIF_GUESSTRACKSECTORS = 1 shl XADDIB_GUESSTRACKSECTORS;
     XADDIF_GUESSLOWCYL = 1 shl XADDIB_GUESSLOWCYL;
     XADDIF_GUESSHIGHCYL = 1 shl XADDIB_GUESSHIGHCYL;
     XADDIF_NOCYLINDERS = 1 shl XADDIB_NOCYLINDERS;
     XADDIF_NOCYLSECTORS = 1 shl XADDIB_NOCYLSECTORS;
     XADDIF_NOHEADS = 1 shl XADDIB_NOHEADS;
     XADDIF_NOTRACKSECTORS = 1 shl XADDIB_NOTRACKSECTORS;
     XADDIF_NOLOWCYL = 1 shl XADDIB_NOLOWCYL;
     XADDIF_NOHIGHCYL = 1 shl XADDIB_NOHIGHCYL;
  { defines for BlockInfo  }
  { this block was cleared for archiving  }
     XADBIB_CLEARED = 0;
  { this block was not archived  }
     XADBIB_UNUSED = 1;
     XADBIF_CLEARED = 1 shl XADBIB_CLEARED;
     XADBIF_UNUSED = 1 shl XADBIB_UNUSED;
  {

       progress report stuff
                                                                         }
  type
     PxadProgressInfo = ^txadProgressInfo;
     txadProgressInfo = record
          xpi_Mode : ULONG;                { work modus  }
          xpi_Client : PxadClient;         { the client doing the work  }
          xpi_DiskInfo : PxadDiskInfo;     { current diskinfo, for disks  }
          xpi_FileInfo : PxadFileInfo;     { current info for files  }
          xpi_CurrentSize : ULONG;         { current filesize  }
          xpi_LowCyl : ULONG;              { for disks only  }
          xpi_HighCyl : ULONG;             { for disks only  }
          xpi_Status : ULONG;              { see XADPIF flags  }
          xpi_Error : LONG;                { any of the error codes  }
          xpi_FileName : STRPTR;           { name of file to overwrite (V2)  }
          xpi_NewName : STRPTR;            { new name buffer, passed by hook (V2)  }
       end;

  { NOTE: For disks CurrentSize is Sector SectorSize, where SectorSize can
  be found in xadDiskInfo structure. So you may output the sector value.  }
  { different progress modes  }

  const
     XADPMODE_ASK = 1;
     XADPMODE_PROGRESS = 2;
     XADPMODE_END = 3;
     XADPMODE_ERROR = 4;
  { (V10)  }
     XADPMODE_NEWENTRY = 5;
  { (V11)  }
     XADPMODE_GETINFOEND = 6;
  { flags for progress hook and ProgressInfo status field  }
  { overwrite the file  }
     XADPIB_OVERWRITE = 0;
  { create the directory  }
     XADPIB_MAKEDIRECTORY = 1;
  { ignore drive geometry  }
     XADPIB_IGNOREGEOMETRY = 2;
  { destination is a directory (V10)  }
     XADPIB_ISDIRECTORY = 3;
  { rename the file (V2)  }
     XADPIB_RENAME = 10;
  { all ok, proceed  }
     XADPIB_OK = 16;
  { skip file  }
     XADPIB_SKIP = 17;
     XADPIF_OVERWRITE = 1 shl XADPIB_OVERWRITE;
     XADPIF_MAKEDIRECTORY = 1 shl XADPIB_MAKEDIRECTORY;
     XADPIF_IGNOREGEOMETRY = 1 shl XADPIB_IGNOREGEOMETRY;
     XADPIF_ISDIRECTORY = 1 shl XADPIB_ISDIRECTORY;
     XADPIF_RENAME = 1 shl XADPIB_RENAME;
     XADPIF_OK = 1 shl XADPIB_OK;
     XADPIF_SKIP = 1 shl XADPIB_SKIP;
  {

       errors

                                                                          }
  { no error  }
     XADERR_OK = $0000;
  { unknown error  }
     XADERR_UNKNOWN = $0001;
  { input data buffers border exceeded  }
     XADERR_INPUT = $0002;
  { output data buffers border exceeded  }
     XADERR_OUTPUT = $0003;
  { function called with illegal parameters  }
     XADERR_BADPARAMS = $0004;
  { not enough memory available  }
     XADERR_NOMEMORY = $0005;
  { data is corrupted  }
     XADERR_ILLEGALDATA = $0006;
  { command is not supported  }
     XADERR_NOTSUPPORTED = $0007;
  { required resource missing  }
     XADERR_RESOURCE = $0008;
  { error on decrunching  }
     XADERR_DECRUNCH = $0009;
  { unknown file type  }
     XADERR_FILETYPE = $000A;
  { opening file failed  }
     XADERR_OPENFILE = $000B;
  { file, disk has been skipped  }
     XADERR_SKIP = $000C;
  { user break in progress hook  }
     XADERR_BREAK = $000D;
  { file already exists  }
     XADERR_FILEEXISTS = $000E;
  { missing or wrong password  }
     XADERR_PASSWORD = $000F;
  { could not create directory  }
     XADERR_MAKEDIR = $0010;
  { wrong checksum  }
     XADERR_CHECKSUM = $0011;
  { verify failed (disk hook)  }
     XADERR_VERIFY = $0012;
  { wrong drive geometry  }
     XADERR_GEOMETRY = $0013;
  { unknown data format  }
     XADERR_DATAFORMAT = $0014;
  { source contains no files  }
     XADERR_EMPTY = $0015;
  { unknown filesystem  }
     XADERR_FILESYSTEM = $0016;
  { name of file exists as directory  }
     XADERR_FILEDIR = $0017;
  { buffer was to short  }
     XADERR_SHORTBUFFER = $0018;
  { text encoding was defective  }
     XADERR_ENCODING = $0019;
  {

       characterset and filename conversion

                                                                          }
  { this is the ONLY destination setting for clients!  }
     CHARSET_HOST = 0;
  { 16bit Unicode (usually no source type)  }
     CHARSET_UNICODE_UCS2_HOST = 10;
  { 16bit Unicode big endian storage  }
     CHARSET_UNICODE_UCS2_BIGENDIAN = 11;
  { 16bit Unicode little endian storage  }
     CHARSET_UNICODE_UCS2_LITTLEENDIAN = 12;
  { variable size unicode encoding  }
     CHARSET_UNICODE_UTF8 = 13;
  { all the 1xx types are generic types which also maybe a bit dynamic  }
  { the default Amiga charset  }
     CHARSET_AMIGA = 100;
  { the default MSDOS charset  }
     CHARSET_MSDOS = 101;
  { the default MacOS charset  }
     CHARSET_MACOS = 102;
  { the default C64 charset  }
     CHARSET_C64 = 103;
  { the default Atari ST charset  }
     CHARSET_ATARI_ST = 104;
  { the default Windows charset  }
     CHARSET_WINDOWS = 105;
  { all the 2xx to 9xx types are real charsets, use them whenever you know
     what the data really is  }
  { the lower 7 bits of ASCII charsets  }
     CHARSET_ASCII = 200;
  { the base charset  }
     CHARSET_ISO_8859_1 = 201;
  { Euro-sign fixed ISO variant  }
     CHARSET_ISO_8859_15 = 215;
  { Atari ST (US) charset  }
     CHARSET_ATARI_ST_US = 300;
  { C64 lower case charset  }
     CHARSET_PETSCII_C64_LC = 301;
  { IBM Codepage 437 charset  }
     CHARSET_CODEPAGE_437 = 400;
  { Windows Codepage 1252 charset  }
     CHARSET_CODEPAGE_1252 = 401;
  {

       client related stuff

                                                                          }
  type
     PxadForeman = ^txadForeman;
     txadForeman = record
          xfm_Security : ULONG;      { should be XADFOREMAN_SECURITY  }
          xfm_ID : ULONG;            { must be XADFOREMAN_ID  }
          xfm_Version : UWORD;       { set to XADFOREMAN_VERSION  }
          xfm_Reserved : UWORD;
          xfm_VersString : STRPTR;   { pointer to $VER: string  }
          xfm_FirstClient : PxadClient; { pointer to first client  }
       end;

  { MOVEQ #-1,D0 and RTS  }

  const
     XADFOREMAN_SECURITY = $70FF4E75;
  { 'XADF' identification ID  }
     XADFOREMAN_ID = $58414446;
     XADFOREMAN_VERSION = 1;



  const
     XADCLIENT_VERSION = 1;
  { archiver is a file archiver  }
     XADCB_FILEARCHIVER = 0;
  { archiver is a disk archiver  }
     XADCB_DISKARCHIVER = 1;
  { external client, set by xadmaster  }
     XADCB_EXTERN = 2;
  { filesystem clients (V5)  }
     XADCB_FILESYSTEM = 3;
  { do not check size for recog call (V6)  }
     XADCB_NOCHECKSIZE = 4;
  { file archiver is plain data file (V11)  }
     XADCB_DATACRUNCHER = 5;
  { file archiver is executable file (V11)  }
     XADCB_EXECRUNCHER = 6;
  { file archiver is address crunched file (V11)  }
     XADCB_ADDRESSCRUNCHER = 7;
  { file archiver is a linker file (V11)  }
     XADCB_LINKER = 8;
  { master frees XAD strings (V12)  }
     XADCB_FREEXADSTRINGS = 25;
  { master frees xadSpecial  structures (V11)  }
     XADCB_FREESPECIALINFO = 26;
  { master frees xadSkipInfo structures (V3)  }
     XADCB_FREESKIPINFO = 27;
  { master frees xadTextInfo structures (V2)  }
     XADCB_FREETEXTINFO = 28;
  { master frees xadTextInfo text block (V2)  }
     XADCB_FREETEXTINFOTEXT = 29;
  { master frees xadFileInfo structures (V2)  }
     XADCB_FREEFILEINFO = 30;
  { master frees xadDiskInfo structures (V2)  }
     XADCB_FREEDISKINFO = 31;
     XADCF_FILEARCHIVER = 1 shl XADCB_FILEARCHIVER;
     XADCF_DISKARCHIVER = 1 shl XADCB_DISKARCHIVER;
     XADCF_EXTERN = 1 shl XADCB_EXTERN;
     XADCF_FILESYSTEM = 1 shl XADCB_FILESYSTEM;
     XADCF_NOCHECKSIZE = 1 shl XADCB_NOCHECKSIZE;
     XADCF_DATACRUNCHER = 1 shl XADCB_DATACRUNCHER;
     XADCF_EXECRUNCHER = 1 shl XADCB_EXECRUNCHER;
     XADCF_ADDRESSCRUNCHER = 1 shl XADCB_ADDRESSCRUNCHER;
     XADCF_LINKER = 1 shl XADCB_LINKER;
     XADCF_FREEXADSTRINGS = 1 shl XADCB_FREEXADSTRINGS;
     XADCF_FREESPECIALINFO = 1 shl XADCB_FREESPECIALINFO;
     XADCF_FREESKIPINFO = 1 shl XADCB_FREESKIPINFO;
     XADCF_FREETEXTINFO = 1 shl XADCB_FREETEXTINFO;
     XADCF_FREETEXTINFOTEXT = 1 shl XADCB_FREETEXTINFOTEXT;
     XADCF_FREEFILEINFO = 1 shl XADCB_FREEFILEINFO;
     XADCF_FREEDISKINFO = 1 shl XADCB_FREEDISKINFO;
  { The types 5 to 9 always need XADCB_FILEARCHIVER set also. These only specify
  the type of the archiver somewhat better. Do not mix real archivers and these
  single file data clients.  }
  {

       client ID's

                                                                          }
  { If an external client has set the xc_Identifier field, the internal
  client is replaced.  }
  { disk archivers start with 1000  }
     XADCID_XMASH = 1000;
     XADCID_SUPERDUPER3 = 1001;
     XADCID_XDISK = 1002;
     XADCID_PACKDEV = 1003;
     XADCID_ZOOM = 1004;
     XADCID_ZOOM5 = 1005;
     XADCID_CRUNCHDISK = 1006;
     XADCID_PACKDISK = 1007;
     XADCID_MDC = 1008;
     XADCID_COMPDISK = 1009;
     XADCID_LHWARP = 1010;
     XADCID_SAVAGECOMPRESSOR = 1011;
     XADCID_WARP = 1012;
     XADCID_GDC = 1013;
     XADCID_DCS = 1014;
  { file archivers start with 5000  }
     XADCID_TAR = 5000;
     XADCID_SDSSFX = 5001;
     XADCID_LZX = 5002;
     XADCID_MXMSIMPLEARC = 5003;
     XADCID_LHPAK = 5004;
     XADCID_AMIGAPLUSUNPACK = 5005;
     XADCID_AMIPACK = 5006;
     XADCID_LHA = 5007;
     XADCID_LHASFX = 5008;
     XADCID_PCOMPARC = 5009;
     XADCID_SOMNI = 5010;
     XADCID_LHSFX = 5011;
     XADCID_XPKARCHIVE = 5012;
     XADCID_SHRINK = 5013;
     XADCID_SPACK = 5014;
     XADCID_SPACKSFX = 5015;
     XADCID_ZIP = 5016;
     XADCID_WINZIPEXE = 5017;
     XADCID_GZIP = 5018;
     XADCID_ARC = 5019;
     XADCID_ZOO = 5020;
     XADCID_LHAEXE = 5021;
     XADCID_ARJ = 5022;
     XADCID_ARJEXE = 5023;
     XADCID_ZIPEXE = 5024;
     XADCID_LHF = 5025;
     XADCID_COMPRESS = 5026;
     XADCID_ACE = 5027;
     XADCID_ACEEXE = 5028;
     XADCID_GZIPSFX = 5029;
     XADCID_HA = 5030;
     XADCID_SQ = 5031;
     XADCID_LHAC64SFX = 5032;
     XADCID_SIT = 5033;
     XADCID_SIT5 = 5034;
     XADCID_SIT5EXE = 5035;
     XADCID_MACBINARY = 5036;
     XADCID_CPIO = 5037;
     XADCID_PACKIT = 5038;
  { filesystem client start with 8000  }
     XADCID_FSAMIGA = 8000;
     XADCID_FSSANITYOS = 8001;
     XADCID_FSFAT = 8002;
  { mixed archivers start with 9000  }
     XADCID_DMS = 9000;
     XADCID_DMSSFX = 9001;

VAR xadMasterBase : pxadMasterBase;


FUNCTION xadAddDiskEntryA(di : pxadDiskInfo; ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadAddFileEntryA(fi : pxadFileInfo; ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadAllocObjectA(_type : LONGINT; CONST tags : pTagItem) : POINTER;
FUNCTION xadAllocVec(size : longword; flags : longword) : POINTER;
FUNCTION xadCalcCRC16(id : longword; init : longword; size : longword; buffer : pCHAR) : WORD;
FUNCTION xadCalcCRC32(id : longword; init : longword; size : longword; buffer : pCHAR) : longword;
FUNCTION xadConvertDatesA(CONST tags : pTagItem) : LONGINT;
FUNCTION xadConvertNameA(charset : longword; CONST tags : pTagItem) : pCHAR;
FUNCTION xadConvertProtectionA(CONST tags : pTagItem) : LONGINT;
PROCEDURE xadCopyMem(src : POINTER; dest : POINTER; size : longword);
FUNCTION xadDiskFileUnArcA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadDiskUnArcA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadFileUnArcA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadFreeHookAccessA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
PROCEDURE xadFreeInfo(ai : pxadArchiveInfo);
PROCEDURE xadFreeObjectA(obj : POINTER; CONST tags : pTagItem);
FUNCTION xadGetClientInfo : pxadClient;
FUNCTION xadGetDiskInfoA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadGetErrorText(errnum : longword) : pCHAR;
FUNCTION xadGetFilenameA(buffersize : longword; buffer : pCHAR; path : pCHAR; name : pCHAR; CONST tags : pTagItem) : LONGINT;
FUNCTION xadGetHookAccessA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadGetInfoA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadHookAccess(command : longword; data : LONGINT; buffer : POINTER; ai : pxadArchiveInfo) : LONGINT;
FUNCTION xadHookTagAccessA(command : longword; data : LONGINT; buffer : POINTER; ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
FUNCTION xadRecogFileA(size : longword; memory : POINTER; CONST tags : pTagItem) : pxadClient;
{
 Functions and procedures with array of const go here
}
FUNCTION xadAddDiskEntry(di : pxadDiskInfo; ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadAddFileEntry(fi : pxadFileInfo; ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadAllocObject(_type : LONGINT; const tags : Array Of Const) : POINTER;
FUNCTION xadConvertDates(const tags : Array Of Const) : LONGINT;
FUNCTION xadConvertName(charset : longword; const tags : Array Of Const) : pCHAR;
FUNCTION xadConvertProtection(const tags : Array Of Const) : LONGINT;
FUNCTION xadDiskFileUnArc(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadDiskUnArc(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadFileUnArc(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadFreeHookAccess(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
PROCEDURE xadFreeObject(obj : POINTER; const tags : Array Of Const);
FUNCTION xadGetDiskInfo(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadGetFilename(buffersize : longword; buffer : pCHAR; path : pCHAR; name : pCHAR; const tags : Array Of Const) : LONGINT;
FUNCTION xadGetHookAccess(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadGetInfo(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadHookTagAccess(command : longword; data : LONGINT; buffer : POINTER; ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
FUNCTION xadRecogFile(size : longword; memory : POINTER; const tags : Array Of Const) : pxadClient;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitXADMASTERLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    XADMASTERIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;



FUNCTION xadAddDiskEntryA(di : pxadDiskInfo; ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L di,A0
        MOVEA.L ai,A1
        MOVEA.L tags,A2
        MOVEA.L xadMasterBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadAddFileEntryA(fi : pxadFileInfo; ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L fi,A0
        MOVEA.L ai,A1
        MOVEA.L tags,A2
        MOVEA.L xadMasterBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadAllocObjectA(_type : LONGINT; CONST tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  _type,D0
        MOVEA.L tags,A0
        MOVEA.L xadMasterBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadAllocVec(size : longword; flags : longword) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  size,D0
        MOVE.L  flags,D1
        MOVEA.L xadMasterBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadCalcCRC16(id : longword; init : longword; size : longword; buffer : pCHAR) : WORD;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  id,D0
        MOVE.L  init,D1
        MOVE.L  size,D2
        MOVEA.L buffer,A0
        MOVEA.L xadMasterBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadCalcCRC32(id : longword; init : longword; size : longword; buffer : pCHAR) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  id,D0
        MOVE.L  init,D1
        MOVE.L  size,D2
        MOVEA.L buffer,A0
        MOVEA.L xadMasterBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadConvertDatesA(CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tags,A0
        MOVEA.L xadMasterBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadConvertNameA(charset : longword; CONST tags : pTagItem) : pCHAR;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  charset,D0
        MOVEA.L tags,A0
        MOVEA.L xadMasterBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadConvertProtectionA(CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tags,A0
        MOVEA.L xadMasterBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE xadCopyMem(src : POINTER; dest : POINTER; size : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L src,A0
        MOVEA.L dest,A1
        MOVE.L  size,D0
        MOVEA.L xadMasterBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION xadDiskFileUnArcA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadDiskUnArcA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadFileUnArcA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadFreeHookAccessA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE xadFreeInfo(ai : pxadArchiveInfo);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L xadMasterBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE xadFreeObjectA(obj : POINTER; CONST tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L obj,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION xadGetClientInfo : pxadClient;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L xadMasterBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadGetDiskInfoA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadGetErrorText(errnum : longword) : pCHAR;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  errnum,D0
        MOVEA.L xadMasterBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadGetFilenameA(buffersize : longword; buffer : pCHAR; path : pCHAR; name : pCHAR; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  buffersize,D0
        MOVEA.L buffer,A0
        MOVEA.L path,A1
        MOVEA.L name,A2
        MOVEA.L tags,A3
        MOVEA.L xadMasterBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadGetHookAccessA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadGetInfoA(ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ai,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadHookAccess(command : longword; data : LONGINT; buffer : POINTER; ai : pxadArchiveInfo) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  command,D0
        MOVE.L  data,D1
        MOVEA.L buffer,A0
        MOVEA.L ai,A1
        MOVEA.L xadMasterBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadHookTagAccessA(command : longword; data : LONGINT; buffer : POINTER; ai : pxadArchiveInfo; CONST tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  command,D0
        MOVE.L  data,D1
        MOVEA.L buffer,A0
        MOVEA.L ai,A1
        MOVEA.L tags,A2
        MOVEA.L xadMasterBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION xadRecogFileA(size : longword; memory : POINTER; CONST tags : pTagItem) : pxadClient;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  size,D0
        MOVEA.L memory,A0
        MOVEA.L tags,A1
        MOVEA.L xadMasterBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION xadAddDiskEntry(di : pxadDiskInfo; ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadAddDiskEntry := xadAddDiskEntryA(di , ai , readintags(tags));
end;

FUNCTION xadAddFileEntry(fi : pxadFileInfo; ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadAddFileEntry := xadAddFileEntryA(fi , ai , readintags(tags));
end;

FUNCTION xadAllocObject(_type : LONGINT; const tags : Array Of Const) : POINTER;
begin
    xadAllocObject := xadAllocObjectA(_type , readintags(tags));
end;

FUNCTION xadConvertDates(const tags : Array Of Const) : LONGINT;
begin
    xadConvertDates := xadConvertDatesA(readintags(tags));
end;

FUNCTION xadConvertName(charset : longword; const tags : Array Of Const) : pCHAR;
begin
    xadConvertName := xadConvertNameA(charset , readintags(tags));
end;

FUNCTION xadConvertProtection(const tags : Array Of Const) : LONGINT;
begin
    xadConvertProtection := xadConvertProtectionA(readintags(tags));
end;

FUNCTION xadDiskFileUnArc(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadDiskFileUnArc := xadDiskFileUnArcA(ai , readintags(tags));
end;

FUNCTION xadDiskUnArc(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadDiskUnArc := xadDiskUnArcA(ai , readintags(tags));
end;

FUNCTION xadFileUnArc(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadFileUnArc := xadFileUnArcA(ai , readintags(tags));
end;

FUNCTION xadFreeHookAccess(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadFreeHookAccess := xadFreeHookAccessA(ai , readintags(tags));
end;

PROCEDURE xadFreeObject(obj : POINTER; const tags : Array Of Const);
begin
    xadFreeObjectA(obj , readintags(tags));
end;

FUNCTION xadGetDiskInfo(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadGetDiskInfo := xadGetDiskInfoA(ai , readintags(tags));
end;

FUNCTION xadGetFilename(buffersize : longword; buffer : pCHAR; path : pCHAR; name : pCHAR; const tags : Array Of Const) : LONGINT;
begin
    xadGetFilename := xadGetFilenameA(buffersize , buffer , path , name , readintags(tags));
end;

FUNCTION xadGetHookAccess(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadGetHookAccess := xadGetHookAccessA(ai , readintags(tags));
end;

FUNCTION xadGetInfo(ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadGetInfo := xadGetInfoA(ai , readintags(tags));
end;

FUNCTION xadHookTagAccess(command : longword; data : LONGINT; buffer : POINTER; ai : pxadArchiveInfo; const tags : Array Of Const) : LONGINT;
begin
    xadHookTagAccess := xadHookTagAccessA(command , data , buffer , ai , readintags(tags));
end;

FUNCTION xadRecogFile(size : longword; memory : POINTER; const tags : Array Of Const) : pxadClient;
begin
    xadRecogFile := xadRecogFileA(size , memory , readintags(tags));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of xadmaster.library}
  {$Info don't forget to use InitXADMASTERLibrary in the beginning of your program}

var
    xadmaster_exit : Pointer;

procedure ClosexadmasterLibrary;
begin
    ExitProc := xadmaster_exit;
    if xadMasterBase <> nil then begin
        CloseLibrary(pLibrary(xadMasterBase));
        xadMasterBase := nil;
    end;
end;

procedure InitXADMASTERLibrary;
begin
    xadMasterBase := nil;
    xadMasterBase := pxadMasterBase(OpenLibrary(XADMASTERNAME,LIBVERSION));
    if xadMasterBase <> nil then begin
        xadmaster_exit := ExitProc;
        ExitProc := @ClosexadmasterLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open xadmaster.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    XADMASTERIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of xadmaster.library}

var
    xadmaster_exit : Pointer;

procedure ClosexadmasterLibrary;
begin
    ExitProc := xadmaster_exit;
    if xadMasterBase <> nil then begin
        CloseLibrary(pLibrary(xadMasterBase));
        xadMasterBase := nil;
    end;
end;

begin
    xadMasterBase := nil;
    xadMasterBase := pxadMasterBase(OpenLibrary(XADMASTERNAME,LIBVERSION));
    if xadMasterBase <> nil then begin
        xadmaster_exit := ExitProc;
        ExitProc := @ClosexadmasterLibrary;
        XADMASTERIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open xadmaster.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    XADMASTERIsCompiledHow := 3;
   {$Warning No autoopening of xadmaster.library compiled}
   {$Warning Make sure you open xadmaster.library yourself}
{$endif dont_use_openlib}


END. (* UNIT XADMASTER *)
