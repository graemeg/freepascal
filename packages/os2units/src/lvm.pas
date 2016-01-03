{
   Copyright (c) International Business Machines  Corp., 2000
   Copyright (c) 2003 Yuri Prokushev

   This module defines the interface to LVM.DLL, which is the
   engine that performs all of the disk partitioning/volume
   creation work.

   This program is free software;  you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;  without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
   the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program;  if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

}
Unit LVM;

{$PACKRECORDS C}

Interface

{$ifdef os2}
  {$define lvm1}
{$endif}

{$ifdef linux}
  {$define lvm2}
{$endif}

// The number of bytes in a sector on the disk.
const
  BYTES_PER_SECTOR=512;

//The maximum number of cylinders, heads, and sectors that a partition table entry can accomodate.
//Cylinders are numbered 0 - 1023, for a maximum of 1024 cylinders.
//eads are numbered 0 - 255, for a maximum of 256 heads.
//Sectors are numbered 1 - 63, for a maximum of 63 sectors per track.
const
  MAX_CYLINDERS= 1024;
  MAX_HEADS    = 256;
  MAX_SECTORS  = 63;

// The following define the values used to indicate that a partition table entry is for an EBR, not a partition.
const
  EBR_BOOT_INDICATOR = 0;
  EBR_FORMAT_INDICATOR = 5;

// The following define is used as the default Format_Indicator for new non-primary partitions.
const
  NEW_LOGICAL_DRIVE_FORMAT_INDICATOR = $6;

// The following define is used as the default Format_Indicator for a new non-active primary partitions.
const
  NEW_PRIMARY_PARTITION_FORMAT_INDICATOR = $16;

// The following define is used as the default Format_Indicator for a new active primary partition.
const
  NEW_ACTIVE_PRIMARY_PARTITION_FORMAT_INDICATOR = $06;

// The following define is used to hold the value of the Boot_Indicator for active partitions.
const
  ACTIVE_PARTITION = $80;

// Define the size of a Partition Name.  Partition Names are user defined names given to a partition.
const
  PARTITION_NAME_SIZE = 20;

// Define the size of a volume name.  Volume Names are user defined names given to a volume.
const
  VOLUME_NAME_SIZE = 20;

// Define the size of a disk name.  Disk Names are user defined names given to physical disk drives in the system.
const
  DISK_NAME_SIZE = 20;

// The name of the filesystem in use on a partition.  This name may be up to 12 ( + NULL terminator) characters long.
const
  FILESYSTEM_NAME_SIZE = 20;

// The comment field is reserved but is not currently used.  This is for future expansion and use.
const
  COMMENT_SIZE = 81;

// Define the minimum number of sectors to reserve on the disk for Boot Manager.
const
  BOOT_MANAGER_SIZE = 2048;

// An INTEGER number is a whole number, either + or -.
//The number appended to the INTEGER key word indicates the number of bits
//used to represent an INTEGER of that type.
type
  INTEGER16=SmallInt;
  INTEGER32=LongInt;
  INTEGER=LongInt;

// A CARDINAL number is a positive integer >= 0.
//The number appended to the CARDINAL key word indicates the number of bits
//used to represent a CARDINAL of that type.
type
  CARDINAL16=Word;
  CARDINAL32 = Cardinal;

(*
/* A REAL number is a floating point number. */
typedef float   REAL32;
typedef double  REAL64;
*)
  REAL32 = double;
  REAL64 = double;


// An ADDRESS variable is one which holds an address.  The address can contain
//anything, or even be invalid.  It is just an address which is presumed to
//hold some kind of data.
Type
  ADDRESS=Pointer;

Type
  pSTRING=PChar;

// 4 bytes
Type
  DoubleWord=Cardinal;

//* The following types are used in the declaration of disk structures.  Disk structures
//have a defined size and internal structure which must be matched exactly.              */

//* 8 bytes. */
type
  QuadWord=QWord;

// The following types are used internally by LVM.  */

// Define a Partition Sector Number.  A Partition Sector Number is relative to the start of a partition.
//The first sector in a partition is PSN 0.
type
  PSN=Cardinal;

// Define a Logical Sector Number.  A Logical Sector Number is relative to the start of a volume.
//The first sector in a volume is LSN 0.
type
  LSN=Cardinal;

// Define a Logical Block Address.  A Logical Block Address is relative to the start of a
//physical device - a disk drive.  The first sector on a disk drive is LBA 0.
type
  LBA=Cardinal;

// The following define sets the maximum number of LVM classes for which structures and storage will be reserved.
const
  MAXIMUM_LVM_CLASSES = 3;

// The following enum defines the various LVM classes to which a "feature" may belong.
// An LVM Plugin is used to implement a "feature", so "plugin" and "feature" are really synonyms.

type
  _LVM_Classes = (
    Partition_Class, // For "features" which must operate on a partition level - i.e. Bad Block Relocation.
    Aggregate_Class, // For "features" which combine partitions into a single logical entity - i.e. Drive Linking.
    Volume_Class     // For "features" which operate best on a volume level - i.e. encryption, mirroring etc.
       );
  LVM_Classes = _LVM_Classes;


// An LVM plugin may belong to one or more classes.  For each class to which it belongs, certain attributes must be defined.
//This structure tracks those attributes for a class.
type
  _LVM_Class_Attributes=record
    ClassMember: BOOLEAN;      // TRUE if a member of this class, FALSE otherwise.
    GlobalExclusive: BOOLEAN;  // TRUE if this plugin can not work with any other plugin - i.e. it
//must be the only "feature" on the volume, besides the built in feature of BBR.
    TopExclusive: BOOLEAN;     // TRUE if this plugin must be the topmost plugin in this class.
    BottomExclusive: BOOLEAN;  // TRUE if this plugin must be the bottommost plugin in this class.
    ClassExclusive: BOOLEAN;   // TRUE if this plugin will not work with any other plugin in this class.
    Weight_Factor: CARDINAL32;    // A value between 1 and 100 which is used to guide the LVM interfaces when attempting to
//establish a default ordering for plugins within this class.  A value of 1
//indicates that this plugin wants to be as close to the bottom of the plugins
//in this class as possible.  A value of 100 means that this plugin wants to
//be as close to being the topmost plugin in this class as possible.  This value
//is only used if none of the "exclusive" flags are set.
  end;
  LVM_Class_Attributes=_LVM_Class_Attributes;

// The following enum specifies the interface types that LVM supports, and hence any plugin must support.
  _LVM_Interface_Types= (
    PM_Interface,
    VIO_Interface,           // LVM.EXE is a VIO app. since it is used during install, and during recovery scenarios where PM/Java may not be available.
    Java_Interface          // The LVM GUI is written in Java.
  );
  LVM_Interface_Types=_LVM_Interface_Types;

const
  MAXIMUM_LVM_INTERFACE_TYPES = 3;


//* The following structures define what functions must be supported for each interface type.
type
  PADDRESS=^ADDRESS;
  PCARDINAL32=^CARDINAL32;

  _LVM_OS2_Native_Support=record
//void (* _System Create_and_Configure) ( CARDINAL32 ID, ADDRESS InputBuffer, CARDINAL32 InputBufferSize, ADDRESS * OutputBuffer, CARDINAL32 * OutputBufferSize, CARDINAL32 * Error_Code);
    Create_and_Configure: procedure(ID: CARDINAL32; InputBuffer: ADDRESS; InputBufferSize: CARDINAL32; OutputBuffer: PADDRESS; OutputBufferSize: PCARDINAL32; Error_Code: PCARDINAL32);
//void (* _System Display_Status) ( ADDRESS Volume_Handle, CARDINAL32 * Error_Code );
    Display_Status: procedure(Volume_Handle: ADDRESS; Error_Code: PCARDINAL32 );
//void (* _System Control_Panel) (ADDRESS Volume_Handle, CARDINAL32 * Error_Code );
    Control_Panel: procedure(Volume_Handle: ADDRESS; Error_Code: PCARDINAL32 );
//void (* _System Help_Panel) (CARDINAL32 Help_Index, CARDINAL32 * Error_Code);
    Help_Panel: procedure(Help_Index: CARDINAL32; Error_Code: PCARDINAL32);
  end;
  LVM_OS2_Native_Support=_LVM_OS2_Native_Support;


type
  _LVM_Interface_Support=record
    Interface_Supported: BOOLEAN;
  case longint of
      0 : ( Java_Interface_Class : ^char );
      1 : ( VIO_PM_Calls : LVM_OS2_Native_Support );
  end;
  LVM_Interface_Support=_LVM_Interface_Support;

//* The following define the default help indicies which must be supported by the Help_Panel function.  NOTE:  Index
//values from 0 to 100 are reserved by LVM.  The Plugin may, for its own use, use any values above 100.
const
  HELP_PLUGIN_DESCRIPTION   = 0;

// The following define the maximum length of the names which can be used to represent a feature in LVM.  The
//maximum name length for a feature is 30 characters plus the trailing NULL character.  For command line parsing,
//though, a shorter name is preferable!  Thus, the "short" name for a feature will be limited to 10 characters
//plus the trailing NULL character.  The "short" name will be used for command line purposes, while the regular
//name will be used by all other interfaces.
const
  MAX_FEATURE_NAME_LENGTH       = 31;
  MAX_FEATURE_SHORT_NAME_LENGTH = 11;
  MAX_OEM_INFO_LENGTH           =255;

// The following definitions are used to control and access the various "features" available through the LVM Engine, such as Drive Linking and BBR.
type
  _Feature_ID_Data=record
    Name: Array[0..MAX_FEATURE_NAME_LENGTH-1] of char;             // Feature Name, for use in menus and command line parsing.
    Short_Name: Array[0..MAX_FEATURE_SHORT_NAME_LENGTH-1] of char; // The name/code used to represent this feature during command line parsing.
    OEM_Info: Array[0..MAX_OEM_INFO_LENGTH-1] of char;             // Name and copyright info. of the manufacturer, i.e. IBM, Vinca, etc.
    ID: CARDINAL32;                                        // Numeric Feature ID.
    Major_Version_Number: CARDINAL32;                      // The version number of this feature.
    Minor_Version_Number: CARDINAL32;                      // The version number of this feature.
    LVM_Major_Version_Number: CARDINAL32;                  // The version of LVM that this feature was designed to work with.
    LVM_Minor_Version_Number: CARDINAL32;                  // The version of LVM that this feature was designed to work with.
    Preferred_Class: LVM_Classes;                           // The class from which this "feature" prefers to be chosen.  Encryption can be performed
//at the partition level or the volume level, and may therefore belong to both the
//Partition_Class and the Volume_Class.  However, it is preferrable for it to be used
//on the volume level instead of at the partition level.  Thus, its perferred class would
//be the Volume_Class, but it would still be a member of both the Volume_Class and the
//Partition_Class.
    ClassData: Array[0..MAXIMUM_LVM_CLASSES-1] of LVM_Class_Attributes;  // The attributes for each of the LVM classes that this "feature" is in.
    Interface_Support: Array[0..MAXIMUM_LVM_INTERFACE_TYPES-1] of LVM_Interface_Support;  // The functions and classes for each of the video modes that LVM can run it.
  end;
  Feature_ID_Data=_Feature_ID_Data;

// The following defines the TAG value used to identify an item of type Feature_ID_Data in a DLIST.
const
  FEATURE_ID_DATA_TAG = 354385972;

// The following are invariant for a disk drive.
Type
  Drive_Control_Record = record
    Drive_Number: CARDINAL32;                   // OS/2 Drive Number for this drive.
    Drive_Size: CARDINAL32;                     // The total number of sectors on the drive.
    Drive_Serial_Number: DoubleWord;            // The serial number assigned to this drive.  For info. purposes only.
    Drive_Handle: ADDRESS;                      // Handle used for operations on the disk that this record corresponds to.
    Cylinder_Count: CARDINAL32;                 // The number of cylinders on the drive.
    Heads_Per_Cylinder: CARDINAL32;             // The number of heads per cylinder for this drive.
    Sectors_Per_Track: CARDINAL32;              // The number of sectors per track for this drive.
    Drive_Is_PRM: BOOLEAN;                      // Set to TRUE if this drive is a PRM.
    Reserved: Array[0..3-1] of BYTE;              // Alignment.
  end;

// The following structure is returned by the Get_Drive_Control_Data function.
  Drive_Control_Array=record
    Drive_Control_Data: ^Drive_Control_Record;       // An array of drive control records.
    Count: CARDINAL32;                               // The number of entries in the array of drive control records.
  end;

// The following structure defines the information that can be changed for a specific disk drive.
  Drive_Information_Record=record
    Total_Available_Sectors: CARDINAL32;        // The number of sectors on the disk which are not currently assigned to a partition.
    Largest_Free_Block_Of_Sectors: CARDINAL32;  // The number of sectors in the largest contiguous block of available sectors.
    Corrupt_Partition_Table: BOOLEAN;           // If TRUE, then the partitioning information found on the drive is incorrect!
    Unusable: BOOLEAN;                          // If TRUE, the drive's MBR is not accessible and the drive can not be partitioned.
    IO_Error: BOOLEAN;                          // If TRUE, then the last I/O operation on this drive failed!
    Is_Big_Floppy: BOOLEAN;                     // If TRUE, then the drive is a PRM formatted as a big floppy (i.e. the old style removable media support).
    Drive_Name: Array[0..DISK_NAME_SIZE-1] of Char; // User assigned name for this disk drive.
  end;

  Partition_Information_Record=record
    Partition_Handle: ADDRESS;                      // The handle used to perform operations on this partition.
    Volume_Handle: ADDRESS;                         // If this partition is part of a volume, this will be the handle of
                                                    //the volume.  If this partition is NOT part of a volume, then this
                                                    //handle will be 0.
    Drive_Handle: ADDRESS;                          // The handle for the drive this partition resides on.
    Partition_Serial_Number: DoubleWord;            // The serial number assigned to this partition.
    Partition_Start: CARDINAL32;                    // The LBA of the first sector of the partition.
    True_Partition_Size: CARDINAL32;                // The total number of sectors comprising the partition.
    Usable_Partition_Size: CARDINAL32;              // The size of the partition as reported to the IFSM.  This is the
                                                    //size of the partition less any LVM overhead.
    Boot_Limit: CARDINAL32;                         // The maximum number of sectors from this block of free space that can be used to
                                                    //create a bootable partition if you allocate from the beginning of the block of
                                                    //free space.
    Spanned_Volume: BOOLEAN;                        // TRUE if this partition is part of a multi-partition volume.
    Primary_Partition: BOOLEAN;                     // True or False.  Any non-zero value here indicates that
                                                    //this partition is a primary partition.  Zero here indicates
                                                    //that this partition is a "logical drive" - i.e. it resides
                                                    //inside of an extended partition.
    Active_Flag: BYTE;                              // 80 = Partition is marked as being active.
                                                    // 0 = Partition is not active.
    OS_Flag: BYTE;                                  // This field is from the partition table.  It is known as the
                                                    //OS flag, the Partition Type Field, Filesystem Type, and
                                                    //various other names.

                                                    //Values of interest

                                                    //If this field is: (values are in hex)

                                                    //07 = The partition is a compatibility partition formatted for use
                                                    //with an installable filesystem, such as HPFS or JFS.
                                                    //00 = Unformatted partition
                                                    //01 = FAT12 filesystem is in use on this partition.
                                                    //04 = FAT16 filesystem is in use on this partition.
                                                    //0A = OS/2 Boot Manager Partition
                                                    //35 = LVM partition
                                                    //84 = OS/2 FAT16 partition which has been relabeled by Boot Manager to "Hide" it.
    Partition_Type: BYTE;                           // 0 = Free Space
                                                    //1 = LVM Partition (Part of an LVM Volume.)
                                                    //2 = Compatibility Partition
                                                    //All other values are reserved for future use.
    Partition_Status: BYTE;                         // 0 = Free Space
                                                    //1 = In Use - i.e. already assigned to a volume.
                                                    //2 = Available - i.e. not currently assigned to a volume.
    On_Boot_Manager_Menu: BOOLEAN;                  // Set to TRUE if this partition is not part of a Volume yet is on the Boot Manager Menu.
    Reserved: BYTE;                                 // Alignment.
    Volume_Drive_Letter: char;                      // The drive letter assigned to the volume that this partition is a part of.
    Drive_Name: Array[0..DISK_NAME_SIZE-1] of char;   // User assigned name for this disk drive.
    File_System_Name: Array[0..FILESYSTEM_NAME_SIZE-1] of char;// The name of the filesystem in use on this partition, if it is known.
    Partition_Name: Array[0..PARTITION_NAME_SIZE-1] of char;   // The user assigned name for this partition.
    Volume_Name: Array[0..VOLUME_NAME_SIZE-1] of char;         // If this partition is part of a volume, then this will be the
                                                             //name of the volume that this partition is a part of.  If this
                                                             //record represents free space, then the Volume_Name will be
                                                             //"FREE SPACE xx", where xx is a unique numeric ID generated by
                                                             //LVM.DLL.  Otherwise it will be an empty string.
  end;

// The following defines are for use with the Partition_Type field in the Partition_Information_Record.
const
  pt_FREE_SPACE_PARTITION = 0;
  pt_LVM_PARTITION        = 1;
  pt_COMPATIBILITY_PARTITION = 2;

// The following defines are for use with the Partition_Status field in the Partition_Information_Record.
const
  PARTITION_IS_IN_USE    = 1;
  PARTITION_IS_AVAILABLE = 2;
  PARTITION_IS_FREE_SPACE = 0;

// The following structure is returned by various functions in the LVM Engine.
type
  Partition_Information_Array=record
    Partition_Array: ^Partition_Information_Record; // An array of Partition_Information_Records.
    Count: CARDINAL32;           // The number of entries in the Partition_Array.
  end;

// The following items are invariant for a volume.
type
  Volume_Control_Record=record
    Volume_Serial_Number: DoubleWord;            // The serial number assigned to this volume.
    Volume_Handle: ADDRESS;                   // The handle used to perform operations on this volume.
    Compatibility_Volume: BOOLEAN;            // TRUE indicates that this volume is compatible with older versions of OS/2.
//FALSE indicates that this is an LVM specific volume and can not be used without OS2LVM.DMD.
    Device_Type: BYTE;                     // Indicates what type of device the Volume resides on:
//0 = Hard Drive under LVM Control
//1 = PRM under LVM Control
//2 = CD-ROM
//3 = Network drive
//4 = Unknown device NOT under LVM Control
    Reserved: Array[0..2-1] of BYTE;                     // Alignment.
  end;

// The following define the device types used in the Device_Type field of the Volume_Control_Record.
const
  LVM_HARD_DRIVE = 0;
  LVM_PRM        = 1;
  NON_LVM_CDROM  = 2;
  NETWORK_DRIVE  = 3;
  NON_LVM_DEVICE = 4;

// The following structure is returned by the Get_Volume_Control_Data function.
type
  Volume_Control_Array=record
    Volume_Control_Data: ^Volume_Control_Record;      // An array of volume control records.
    Count: CARDINAL32;                    // The number of entries in the array of volume control records.
  end;

// The following information about a volume can (and often does) vary.
type
  Volume_Information_Record=record
    Volume_Size: CARDINAL32;                           // The number of sectors comprising the volume.
    Partition_Count: CARDINAL32;                       // The number of partitions which comprise this volume.
    Drive_Letter_Conflict: CARDINAL32;                 // 0 indicates that the drive letter preference for this volume is unique.
//1 indicates that the drive letter preference for this volume
//is not unique, but this volume got its preferred drive letter anyway.
//2 indicates that the drive letter preference for this volume
//is not unique, and this volume did NOT get its preferred drive letter.
//4 indicates that this volume is currently "hidden" - i.e. it has
//no drive letter preference at the current time.
    Compatibility_Volume: BOOLEAN;                  // TRUE if this is for a compatibility volume, FALSE otherwise.
    Bootable: BOOLEAN;                              // Set to TRUE if this volume appears on the Boot Manager menu, or if it is
//a compatibility volume and its corresponding partition is the first active
//primary partition on the first drive.
    Drive_Letter_Preference: char;               // The drive letter that this volume desires to be.
    Current_Drive_Letter: char;                  // The drive letter currently used to access this volume.  May be different than
//Drive_Letter_Preference if there was a conflict ( i.e. Drive_Letter_Preference
//is already in use by another volume ).
    Initial_Drive_Letter: char;                  // The drive letter assigned to this volume by the operating system when LVM was started.
//This may be different from the Drive_Letter_Preference if there were conflicts, and
//may be different from the Current_Drive_Letter.  This will be 0x0 if the Volume did
//not exist when the LVM Engine was opened (i.e. it was created during this LVM session).
    New_Volume: BOOLEAN;                            // Set to FALSE if this volume existed before the LVM Engine was opened.  Set to
//TRUE if this volume was created after the LVM Engine was opened.
    Status: BYTE;                                // 0 = None.
//1 = Bootable
//2 = Startable
//3 = Installable.
    Reserved_1: BYTE;
    Volume_Name: Array[0..VOLUME_NAME_SIZE-1] of char;         // The user assigned name for this volume.
    File_System_Name: Array[0..FILESYSTEM_NAME_SIZE-1] of char;// The name of the filesystem in use on this partition, if it is known.
  end;

// The following structure is used to return the feature information for the installed features, or the features on a volume.
type
  Feature_Information_Array=record
    Count: CARDINAL32;
    Feature_Data: ^Feature_ID_Data;
  end;

// The following structure defines an item on the Boot Manager Menu.
type
  Boot_Manager_Menu_Item=record
    Handle: ADDRESS;            // A Volume or Partition handle.
    Volume: BOOLEAN;            // If TRUE, then Handle is the handle of a Volume.  Otherwise, Handle is the handle of a partition.
  end;

// The following structure is used to get a list of the items on the partition manager menu.
type
  Boot_Manager_Menu=record
    Menu_Items: ^Boot_Manager_Menu_Item;
    Count: CARDINAL32;
  end;

// The following structure is used to specify an LVM Feature when creating a volume.  Since LVM Features may be part of
//more than one LVM Class, the specific class to be used with the feature must also be specified.
type
  LVM_Feature_Specification_Record=record
    Feature_ID: CARDINAL32;      // The feature ID of the feature to use.
    Actual_Class: LVM_Classes;   // The LVM Class (supported by the specified feature) to use.
    Init_Data: ADDRESS;          // The address of a buffer containing initialization data for this feature.
                                 //NULL if there is no initialization data being provided for this feature.
  end;

// The following structure is used with the Get_Child_Handles function.
Type
  LVM_Handle_Array_Record=record
    Count: CARDINAL32;
    Handles: ^ADDRESS;
  end;

// The following preprocessor directives define the operations that can be performed on a partition, volume, or a block of free space.
// These definitions represent bits in a 32 bit value returned by the Get_Valid_Options function.
const
  CREATE_PRIMARY_PARTITION          = 1;
  CREATE_LOGICAL_DRIVE              = 2;
  DELETEPARTITION                   = 4;
  SET_ACTIVE_PRIMARY                = 8;
  SET_PARTITION_ACTIVE             = $10;
  SET_PARTITION_INACTIVE           = $20;
  SETSTARTABLE                    = $40;
  INSTALLBOOTMANAGER             = $80;
  REMOVEBOOTMANAGER              = $100;
  SET_BOOT_MANAGER_DEFAULTS        = $200;
  ADD_TO_BOOT_MANAGER_MENU         = $400;
  REMOVE_FROM_BOOT_MANAGER_MENU    = $800;
  DELETEVOLUME                    = $1000;
  HIDEVOLUME                      = $2000;
  EXPANDVOLUME                    = $4000;
  SET_VOLUME_INSTALLABLE           = $8000;
  ASSIGNDRIVELETTER              = $10000;
  CAN_BOOT_PRIMARY                 = $20000;      // If a primary is created from this block of free space, then it can be made bootable.
  CAN_BOOT_LOGICAL                 = $40000;      // If a logical drive is created from this block of free space, then OS/2 can boot from it by adding it to the boot manager menu.
  CAN_SET_NAME                     = $80000;
  SET_BOOT_MANAGER_STARTABLE       = $100000;

// The following enumeration defines the allocation strategies used by the Create_Partition function.
type
  _Allocation_Algorithm =(
Automatic,               // Let LVM decide which block of free space to use to create the partition.
Best_Fit,                // Use the block of free space which is closest in size to the partition being created.
First_Fit,               // Use the first block of free space on the disk which is large enough to hold a partition of the specified size.
Last_Fit,                // Use the last block of free space on the disk which is large enough to hold a partition of the specified size.
From_Largest,            // Find the largest block of free space and allocate the partition from that block of free space.
From_Smallest,           // Find the smallest block of free space that can accommodate a partition of the size specified.
All                      // Turn the specified drive or block of free space into a single partition.
);
  Allocation_Algorithm=_Allocation_Algorithm;

// Error codes returned by the LVM Engine.
const
  LVM_ENGINE_NO_ERROR                          =  0;
  LVM_ENGINE_OUT_OF_MEMORY                     =  1;
  LVM_ENGINE_IO_ERROR                          =  2;
  LVM_ENGINE_BAD_HANDLE                        =  3;
  LVM_ENGINE_INTERNAL_ERROR                    =  4;
  LVM_ENGINE_ALREADY_OPEN                      =  5;
  LVM_ENGINE_NOT_OPEN                          =  6;
  LVM_ENGINE_NAME_TOO_BIG                      =  7;
  LVM_ENGINE_OPERATION_NOT_ALLOWED             =  8;
  LVM_ENGINE_DRIVE_OPEN_FAILURE                =  9;
  LVM_ENGINE_BAD_PARTITION                     = 10;
  LVM_ENGINE_CAN_NOT_MAKE_PRIMARY_PARTITION    = 11;
  LVM_ENGINE_TOO_MANY_PRIMARY_PARTITIONS       = 12;
  LVM_ENGINE_CAN_NOT_MAKE_LOGICAL_DRIVE        = 13;
  LVM_ENGINE_REQUESTED_SIZE_TOO_BIG            = 14;
  LVM_ENGINE_1024_CYLINDER_LIMIT               = 15;
  LVM_ENGINE_PARTITION_ALIGNMENT_ERROR         = 16;
  LVM_ENGINE_REQUESTED_SIZE_TOO_SMALL          = 17;
  LVM_ENGINE_NOT_ENOUGH_FREE_SPACE             = 18;
  LVM_ENGINE_BAD_ALLOCATION_ALGORITHM          = 19;
  LVM_ENGINE_DUPLICATE_NAME                    = 20;
  LVM_ENGINE_BAD_NAME                          = 21;
  LVM_ENGINE_BAD_DRIVE_LETTER_PREFERENCE       = 22;
  LVM_ENGINE_NO_DRIVES_FOUND                   = 23;
  LVM_ENGINE_WRONG_VOLUME_TYPE                 = 24;
  LVM_ENGINE_VOLUME_TOO_SMALL                  = 25;
  LVM_ENGINE_BOOT_MANAGER_ALREADY_INSTALLED    = 26;
  LVM_ENGINE_BOOT_MANAGER_NOT_FOUND            = 27;
  LVM_ENGINE_INVALID_PARAMETER                 = 28;
  LVM_ENGINE_BAD_FEATURE_SET                   = 29;
  LVM_ENGINE_TOO_MANY_PARTITIONS_SPECIFIED     = 30;
  LVM_ENGINE_LVM_PARTITIONS_NOT_BOOTABLE       = 31;
  LVM_ENGINE_PARTITION_ALREADY_IN_USE          = 32;
  LVM_ENGINE_SELECTED_PARTITION_NOT_BOOTABLE   = 33;
  LVM_ENGINE_VOLUME_NOT_FOUND                  = 34;
  LVM_ENGINE_DRIVE_NOT_FOUND                   = 35;
  LVM_ENGINE_PARTITION_NOT_FOUND               = 36;
  LVM_ENGINE_TOO_MANY_FEATURES_ACTIVE          = 37;
  LVM_ENGINE_PARTITION_TOO_SMALL               = 38;
  LVM_ENGINE_MAX_PARTITIONS_ALREADY_IN_USE     = 39;
  LVM_ENGINE_IO_REQUEST_OUT_OF_RANGE           = 40;
  LVM_ENGINE_SPECIFIED_PARTITION_NOT_STARTABLE = 41;
  LVM_ENGINE_SELECTED_VOLUME_NOT_STARTABLE     = 42;
  LVM_ENGINE_EXTENDFS_FAILED                   = 43;
  LVM_ENGINE_REBOOT_REQUIRED                   = 44;
  LVM_ENGINE_CAN_NOT_OPEN_LOG_FILE             = 45;
  LVM_ENGINE_CAN_NOT_WRITE_TO_LOG_FILE         = 46;
  LVM_ENGINE_REDISCOVER_FAILED                 = 47;
  LVM_ENGINE_INTERNAL_VERSION_FAILURE          = 48;
  LVM_ENGINE_PLUGIN_OPERATION_INCOMPLETE       = 49;
  LVM_ENGINE_BAD_FEATURE_ID                    = 50;
  LVM_ENGINE_NO_INIT_DATA                      = 51;
  LVM_ENGINE_NO_CONTEXT_DATA                   = 52;
  LVM_ENGINE_WRONG_CLASS_FOR_FEATURE           = 53;
  LVM_ENGINE_INCOMPATIBLE_FEATURES_SELECTED    = 54;
  LVM_ENGINE_NO_CHILDREN                       = 55;
  LVM_ENGINE_FEATURE_NOT_SUPPORTED_BY_INTERFACE= 56;
  LVM_ENGINE_NO_PARENT                         = 57;
  LVM_ENGINE_VOLUME_HAS_NOT_BEEN_COMMITTED_YET = 58;
  LVM_ENGINE_UNABLE_TO_REFERENCE_VOLUME        = 59;
  LVM_ENGINE_PARSING_ERROR                     = 60;
  LVM_ENGINE_INTERNAL_FEATURE_ERROR            = 61;
  LVM_ENGINE_VOLUME_NOT_CONVERTED              = 62;


// The following definitions are used for command line processing.  As the command line is processed,
//the command line is first broken up into tokens.  Each token has a "characterization", which indicates
//what the token is thought to be.

type
  Token_Characterizations=(
LVM_AcceptableCharsStr,
LVM_All,
LVM_BestFit,
LVM_BootDOS,
LVM_BootOS2,
LVM_Bootable,
LVM_CR,
LVM_CRI,
LVM_Compatibility,
LVM_Drive,
LVM_Existing,
LVM_Expand,
LVM_FS,
LVM_FirstFit,
LVM_Freespace,
LVM_FromEnd,
LVM_FromLargest,
LVM_FromSmallest,
LVM_FromStart,
LVM_LVM,
LVM_LastFit,
LVM_Logical,
LVM_New,
LVM_NoBoot,
LVM_NonBootable,
LVM_NotBootable,
LVM_Partition,
LVM_Primary,
LVM_RB,
LVM_Size,
LVM_Unusable,
LVM_Unused,
LVM_Volume,
LVM_Volumes,
LVM_Comma,
LVM_Number,
LVM_Colon,
LVM_Space,
LVM_Tab,
LVM_MultiSpace,
LVM_MultiTab,
LVM_String,
LVM_FileNameStr,
LVM_SemiColon,
LVM_Eof,
LVM_Separator,
LVM_Open_Paren,                    //* ( */
LVM_Close_Paren,                   //* ) */
LVM_Open_Bracket,                  //* [ */
LVM_Close_Bracket,                 //* ] */
LVM_Open_Brace,                    //* { */
LVM_Close_Brace,                   //* } */
LVM_EQ_Sign,                       //* = */
LVM_Bootmgr,
LVM_Create,
LVM_Delete,
LVM_DriveLetter,
LVM_File,
LVM_Hide,
LVM_Install,
LVM_NewMBR,
LVM_Query,
LVM_RediscoverPRM,
LVM_SetName,
LVM_SetStartable,
LVM_SI,
LVM_SlashSize,
LVM_StartLog
);

type
  _LVM_Token=record
    TokenText: PChar;  // The actual text of the token.
    TokenType: Token_Characterizations;  // What the token is thought to be.
    Position: CARDINAL32;   // The position of the first character of the token on the command line.
  end;
  LVM_Token=_LVM_Token;

const
  LVM_TOKEN_TAG = 28387473;

// Function Prototypes

//***************************************************************************
//
// Functions relating to the LVM Engine itself
//
//***************************************************************************

//****************************************************************************************************/
//*                                                                                                  */
//*   Function Name: Open_LVM_Engine                                                                 */
//*                                                                                                  */
//*   Descriptive Name: Opens the LVM Engine and readies it for use.                                 */
//*                                                                                                  */
//*   Input: BOOLEAN Ignore_CHS : If TRUE, then the LVM engine will not check the CHS values in the  */
//*                               MBR/EBR partition tables for validity.  This is useful if there    */
//*                               are drive geometry problems, such as the drive was partitioned and */
//*                               formatted with one geometry and then moved to a different machine  */
//*                               which uses a different geometry for the drive.  This would cause   */
//*                               the starting and ending CHS values in the partition tables to      */
//*                               be inconsistent with the size and partition offset entries in the  */
//*                               partition tables.  Setting Ignore_CHS to TRUE will disable the     */
//*                               LVM Engine's CHS consistency checks, thereby allowing the drive    */
//*                               to be partitioned.                                                 */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in which to store an error code   */
//*                                    should an error occur.                                        */
//*                                                                                                  */
//*   Output:  *Error_Code will be 0 if this function completes successfully.  If an error occurs,   */
//*            *Error_Code will contain a non-zero error code.                                       */
//*                                                                                                  */
//*   Error Handling: If this function aborts with an error, all memory allocated during the course  */
//*                   of this function will be released.  Disk read errors will be reported to the   */
//*                   user via pop-up error messages.  Disk read errors will only cause this         */
//*                   function to abort if none of the disk drives in the system could be            */
//*                   successfully read.                                                             */
//*                                                                                                  */
//*   Side Effects:  The LVM Engine will be initialized.  The partition tables for all OS2DASD       */
//*                  controlled disk drives will be read into memory.  Memory will be allocated for  */
//*                  the data structures used by the LVM Engine.                                     */
//*                                                                                                  */
//*   Notes:  This is provided for programs that used LVM Version 1.  This function assumes an       */
//*           LVM_Interface_Type of VIO_Interface.                                                   */
//*                                                                                                  */
//****************************************************************************************************/
procedure Open_LVM_Engine(Ignore_CHS: BOOLEAN; Error_Code: PCARDINAL32); external 'lvm' name 'Open_LVM_Engine';

//****************************************************************************************************/
//*                                                                                                  */
//*   Function Name: Open_LVM_Engine2                                                                */
//*                                                                                                  */
//*   Descriptive Name: Opens the LVM Engine and readies it for use.                                 */
//*                                                                                                  */
//*   Input: BOOLEAN Ignore_CHS : If TRUE, then the LVM engine will not check the CHS values in the  */
//*                               MBR/EBR partition tables for validity.  This is useful if there    */
//*                               are drive geometry problems, such as the drive was partitioned and */
//*                               formatted with one geometry and then moved to a different machine  */
//*                               which uses a different geometry for the drive.  This would cause   */
//*                               the starting and ending CHS values in the partition tables to      */
//*                               be inconsistent with the size and partition offset entries in the  */
//*                               partition tables.  Setting Ignore_CHS to TRUE will disable the     */
//*                               LVM Engine's CHS consistency checks, thereby allowing the drive    */
//*                               to be partitioned.                                                 */
//*          LVM_Interface_Types Interface_Type - Indicate the type of user interface being used:    */
//*                               PM_Interface, VIO_Interface, or Java_Interface.  This lets the     */
//*                               LVM Engine know which interface support routines to call in any    */
//*                               plugin modules which may be loaded.                                */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in which to store an error code   */
//*                                    should an error occur.                                        */
//*                                                                                                  */
//*   Output:  *Error_Code will be 0 if this function completes successfully.  If an error occurs,   */
//*            *Error_Code will contain a non-zero error code.                                       */
//*                                                                                                  */
//*   Error Handling: If this function aborts with an error, all memory allocated during the course  */
//*                   of this function will be released.  Disk read errors will be reported to the   */
//*                   user via pop-up error messages.  Disk read errors will only cause this         */
//*                   function to abort if none of the disk drives in the system could be            */
//*                   successfully read.                                                             */
//*                                                                                                  */
//*   Side Effects:  The LVM Engine will be initialized.  The partition tables for all OS2DASD       */
//*                  controlled disk drives will be read into memory.  Memory will be allocated for  */
//*                  the data structures used by the LVM Engine.                                     */
//*                                                                                                  */
//*   Notes:  New in LVM Version 2                                                                   */
//*                                                                                                  */
//****************************************************************************************************/
{$ifdef lvm2}
procedure Open_LVM_Engine2(Ignore_CHS: BOOLEAN; Interface_Type: LVM_Interface_Types; Error_Code: PCARDINAL32); external 'lvm' name 'Open_LVM_Engine2';
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Commit_Changes                                   */
//*                                                                   */
//*   Descriptive Name: Saves any changes made to the partitioning    */
//*                     information of the OS2DASD controlled disk    */
//*                     drives in the system.                         */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output:  The function return value will be TRUE if all of the   */
//*            partitioning/volume changes made were successfully     */
//*            written to disk.  Also, *Error_Code will be 0 if no    */
//*            errors occur.                                          */
//*                                                                   */
//*            If an error occurs, then the furnction return value    */
//*            will be FALSE and *Error_Code will contain a non-zero  */
//*            error code.                                            */
//*                                                                   */
//*   Error Handling:  If an error occurs, the function return value  */
//*                    will be false and *Error_Code will be > 0.     */
//*                                                                   */
//*                    Disk read and write errors will be indicated by*/
//*                    setting the IO_Error field of the              */
//*                    Drive_Information_Record to TRUE.  Thus, if    */
//*                    the function return value is FALSE, and        */
//*                    *Error_Code indicates an I/O error, the caller */
//*                    of this function should call the               */
//*                    Get_Drive_Status function on each drive to     */
//*                    determine which drives had I/O errors.         */
//*                                                                   */
//*                    If a read or write error occurs, then the      */
//*                    engine may not have been able to create a      */
//*                    partition or volume.  Thus, the caller         */
//*                    may want to refresh all partition and volume   */
//*                    data to see what the engine was and was not    */
//*                    able to create.                                */
//*                                                                   */
//*   Side Effects:  The partitioning information of the disk drives  */
//*                  in the system may be altered.                    */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Commit_Changes(Error_Code: PCARDINAL32): BOOLEAN; external 'lvm' name 'Commit_Changes';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Java_Call_Back                               */
//*                                                                   */
//*   Descriptive Name: This function allows the calling Java program */
//*                     to set the call back address.  The call back  */
//*                     address is used when the LVM Engine or one of */
//*                     its plug-ins, needs to run a Java class to    */
//*                     gather information from the user.             */
//*                                                                   */
//*   Input: void ( * Execute_Java_Class) ... - The address of a       */
//*                                            function that the LVM  */
//*                                            engine may call when   */
//*                                            it needs a Java class  */
//*                                            to be executed.  This  */
//*                                            is only required if the*/
//*                                            user interface being   */
//*                                            used is written in Java*/
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output: If the function completes successfully, then *Error_Code*/
//*           will be set to LVM_ENGINE_NO_ERROR.  Otherwise,         */
//*           *Error_Code will be set to a non-zero error code.       */
//*                                                                   */
//*   Error Handling:  If an error occurs, the function will abort and*/
//*                    *Error_Code will be set to a non-zero error    */
//*                    code.                                          */
//*                                                                   */
//*   Side Effects:  The Java call back address is set to point to the*/
//*                  specified function.  Once the Java call back     */
//*                  address is set, LVM plug-ins which require the   */
//*                  Java call back will be enabled and can be used   */
//*                  during the creation of LVM Volumes.              */
//*                                                                   */
//*   Notes:  If a Java interface is in use (as specified on the      */
//*           Open_LVM_Engine call), then this function must be called*/
//*           in order to enable those LVM plug-ins which require     */
//*           initialization information during the creation of an    */
//*           LVM Volume.  If these plug-ins are not enabled, then    */
//*           they will not be reported by the Get_Available_Features */
//*           API, nor can they be used or accessed by any other LVM  */
//*           Engine APIs.  Thus, this function should be called      */
//*           immediately after the Open_LVM_Engine API is called.    */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
type
  TJavaExecProc=procedure(
    Class_Name: PChar;
    InputBuffer: ADDRESS;
    InputBufferSize: CARDINAL32;
    OutputBuffer: PADDRESS;
    OutputBufferSize,
    Error_Code: PCARDINAL32);

procedure Set_Java_Call_Back(
  Execute_Java_Class: TJAvaExecProc;
  Error_Code: PCARDINAL32); external 'lvm' name 'Set_Java_Call_Back';

{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Close_LVM_Engine                                 */
//*                                                                   */
//*   Descriptive Name: Closes the LVM Engine and frees any memory    */
//*                     held by the LVM Engine.                       */
//*                                                                   */
//*   Input: None.                                                    */
//*                                                                   */
//*   Output:  None.                                                  */
//*                                                                   */
//*   Error Handling: N/A                                             */
//*                                                                   */
//*   Side Effects:  Any memory held by the LVM Engine is released.   */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Close_LVM_Engine; external 'lvm' name 'Close_LVM_Engine';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Available_Features                           */
//*                                                                   */
//*   Descriptive Name: Returns the feature ID information for each of*/
//*                     the features that the LVM Engine knows about. */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output: If successful, a Feature_Information_Array structure is */
//*           returned with a non-zero Count.  Also, *Error_Code will */
//*           be set to LVM_ENGINE_NO_ERROR.  If an error occurs,     */
//*           then the Count field in the structure will be 0 and     */
//*           ( *Error_Code) will contain a non-zero error code.       */
//*                                                                   */
//*   Error Handling: The only expected error is if this function is  */
//*                   called while the LVM Engine is not open.  This  */
//*                   should be the only error condition.             */
//*                                                                   */
//*   Side Effects:  Memory is allocated using the LVM Engine's memory*/
//*                  manager for the array of Feature_ID_Data items   */
//*                  being returned.                                  */
//*                                                                   */
//*   Notes:  This function seems to be presented since LVM Version 2 */
//*                                                                   */
//*********************************************************************/
{$ifdef LVM2}
function Get_Available_Features(Error_Code: PCARDINAL32): Feature_Information_Array; external 'lvm' name 'Get_Available_Features';
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Issue_Feature_Command                            */
//*                                                                   */
//*   Descriptive Name: Issues a feature specific command to either   */
//*                     the Ring 0 or Ring 3 portion of the feature.  */
//*                                                                   */
//*   Input: CARDINAL32 Feature_ID - The numeric ID assigned to the   */
//*                                  feature which is to receive the  */
//*                                  command being issued.            */
//*          ADDRESS Handle - The handle of the volume, partition, or */
//*                           aggregate to which the feature command  */
//*                           is to be directed.                      */
//*          BOOLEAN Ring0 - If TRUE, then the command will be sent   */
//*                          to the Ring 0 portion of the feature.    */
//*                          If FALSE, then the command will be sent  */
//*                          to the Ring 3 portion of the feature.    */
//*          ADDRESS InputBuffer - A buffer containing the command and*/
//*                                any necessary information for the  */
//*                                feature to process the command.    */
//*          CARDINAL32 InputSize - The number of bytes in the        */
//*                                 InputBuffer.                      */
//*          ADDRESS * OutputBuffer - The address of a variable used  */
//*                                   to hold the location of the     */
//*                                   output buffer created by the    */
//*                                   feature in response to the      */
//*                                   command in InputBuffer.         */
//*          CARDINAL32 * OutputSize - The address of a variable used */
//*                                    to hold the number of bytes in */
//*                                    *OutputBuffer.                 */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output: If successful, then *Error_Code will be set to          */
//*           LVM_ENGINE_NO_ERROR.  If unsuccessful, then *Error_Code */
//*           will be set to a non-zero error code.  *OutputBuffer and*/
//*           *OutputSize are set by the feature.  If the feature     */
//*           specified does not exist, then *OutputBuffer will be    */
//*           NULL and *Outputsize will be 0.  If the feature does    */
//*           exist, then the value of *OutputBuffer and *OutputSize  */
//*           depend upon the feature.                                */
//*                                                                   */
//*   Error Handling: If the specified feature does not exist, then   */
//*                   *Error_Code will contain a non-zero error code, */
//*                   *OutputBuffer will be NULL, and *OutputSize will*/
//*                   be set to 0.  All other error conditions are    */
//*                   feature dependent.                              */
//*                                                                   */
//*   Side Effects:  Side effects are feature dependent.              */
//*                                                                   */
//*   Notes:  New in LVM Version 2                                    */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
void _System Issue_Feature_Command( CARDINAL32 Feature_ID,
ADDRESS Handle,
BOOLEAN Ring0,
ADDRESS InputBuffer,
CARDINAL32 InputSize,
ADDRESS * OutputBuffer,
CARDINAL32 * OutputSize,
CARDINAL32 * Error_Code );
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Parse_Feature_Parameters                         */
//*                                                                   */
//*   Descriptive Name: This function allows access to the parsing    */
//*                     function of an LVM Plug-in Feature.  The      */
//*                     specified feature will be passed a list of    */
//*                     tokens to parse, and, if it parses the tokens */
//*                     successfully, will produce a buffer containing*/
//*                     initialization data that can be used with the */
//*                     Create_Volume API.  If it encounters an error,*/
//*                     the current item in the Tokens list will be   */
//*                     the offending item, and a pointer to an error */
//*                     message will be returned.                     */
//*                                                                   */
//*   Input: DLIST  Tokens - A DLIST of tokens to parse.  Parsing will*/
//*                          begin with the current entry in the list */
//*                          and proceed until there is an error or   */
//*                          until the specified feature has found a  */
//*                          complete command.  Each feature defines  */
//*                          what commands it will accept.            */
//*          LVM_Feature_Specification_Record * Feature_Data - A      */
//*                          pointer to a record which contains the   */
//*                          ID of the feature which is to parse the  */
//*                          DLIST of Tokens.  The other fields in    */
//*                          this record will be filled in by the     */
//*                          feature if it successfully parses the    */
//*                          tokens.                                  */
//*          char ** Error_Message - The address of a pointer to char.*/
//*                          This will be set to NULL if the feature  */
//*                          successfully parses the list of tokens,  */
//*                          or it will be set to point to an error   */
//*                          message if an error occurs.              */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32    */
//*                          variable into which an error code may    */
//*                          be placed.  The error code will be       */
//*                          LVM_ENGINE_NO_ERROR if this function     */
//*                          completes successfully, or a non-zero    */
//*                          error code if an error occurs.           */
//*                                                                   */
//*   Output: If there are no errors, the Actual_Class and Init_Data  */
//*           fields of *Feature_Data will be set, *Error_Message will*/
//*           be set to NULL, and Error_Code will be set to           */
//*           LVM_ENGINE_NO_ERROR.  The current item in the Tokens    */
//*           list will be the first token that was not parsed by the */
//*           feature (i.e. the first token after the command accepted*/
//*           by the plug-in).                                        */
//*                                                                   */
//*           If an error occurs, the values of Actual_Class and      */
//*           Init_Data in *Feature_Data are undefined.  *Error_Code  */
//*           will be set to LVM_ENGINE_PARSING_ERROR if the error is */
//*           parsing related, or some other non-zero value if the    */
//*           error is not parsing related.  If the error is parsing  */
//*           related, then *Error_Message will point to an error     */
//*           message which can be displayed for the user.  The       */
//*           current item in the Tokens list will be the token which */
//*           failed.                                                 */
//*                                                                   */
//*   Error Handling: If an parsing related error occurs, i.e. the    */
//*                   tokens in the Tokens list do not form a valid   */
//*                   command accepted by the specified feature, then */
//*                   the current item in the Tokens list will be the */
//*                   offending token, *Error_Message will be set to  */
//*                   point to an error message, and *Error_Code will */
//*                   be set to LVM_ENGINE_PARSING_ERROR.             */
//*                   If any other error occurs, the current item in  */
//*                   the Tokens list will be the token that was being*/
//*                   processed when the error occurs, and *Error_Code*/
//*                   will be set to a non-zero value.                */
//*                                                                   */
//*   Side Effects:  The current item in the Tokens list may change.  */
//*                                                                   */
//*   Notes:  Each feature defines which commands it will accept, and */
//*           therefore which commands it will successfully parse.    */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
void _System Parse_Feature_Parameters( DLIST                               Tokens,
LVM_Feature_Specification_Record  * Feature_Data,
char **                             Error_Message,
CARDINAL32 *                        Error_Code);
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Refresh_LVM_Engine                               */
//*                                                                   */
//*   Descriptive Name: This function causes the LVM Engine to look   */
//*                     for changes in the current system             */
//*                     configuration and update its internal tables  */
//*                     accordingly.                                  */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output:  None.                                                  */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be set to  */
//*                   a non-zero value.                               */
//*                                                                   */
//*   Side Effects: Volumes which represent non-LVM devices may have  */
//*                 their handles changed!                            */
//*                                                                   */
//*   Notes:  After calling this function, Get_Volume_Control_Data    */
//*           should be called to get the updated list of volumes.    */
//*           This is necessary as the handles of some volumes may    */
//*           have changed.                                           */
//*                                                                   */
//*********************************************************************/
procedure Refresh_LVM_Engine(Error_Code: PCARDINAL32); external 'lvm' name 'Refresh_LVM_Engine';

//****************************************************************************
//
// Functions relating to Drives
//
//***************************************************************************

//*********************************************************************/
//*                                                                   */
//*   Function Name:  Get_Drive_Control_Data                          */
//*                                                                   */
//*   Descriptive Name:  This function returns an array of            */
//*                      Drive_Control_Records.  These records provide*/
//*                      important information about the drives in the*/
//*                      system and provide the handles required to   */
//*                      access them.                                 */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output:  A Drive_Control_Array structure is returned.  If no    */
//*            errors occur, Drive_Control_Data will be non-NULL,     */
//*            Count will be greater than zero, and *Error_Code will  */
//*            be 0.                                                  */
//*                                                                   */
//*   Error Handling:  If an error occurs, the Drive_Control_Array    */
//*                    returned by this function will have NULL for   */
//*                    Drive_Control_Data, and 0 for Count.           */
//*                    *Error_Code will be greater than 0.            */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The caller becomes responsible for the memory allocated */
//*           for the array of Drive_Control_Records pointed to by    */
//*           Drive_Control_Data pointer in the Drive_Control_Array   */
//*           structure returned by this function.  The caller should */
//*           free this memory when they are done using it.           */
//*                                                                   */
//*********************************************************************/
function Get_Drive_Control_Data(Error_Code: PCARDINAL32): Drive_Control_Array; external 'lvm' name 'Get_Drive_Control_Data';

//*********************************************************************/
//*                                                                   */
//*   Function Name:  Get_Drive_Status                                */
//*                                                                   */
//*   Descriptive Name:  Returns the Drive_Information_Record for the */
//*                      drive specified by Drive_Handle.             */
//*                                                                   */
//*   Input: ADDRESS Drive_Handle - The handle of the drive to use.   */
//*                             Drive handles are obtained through the*/
//*                             Get_Drive_Control_Data function.      */
//*         CARDINAL32 * Error_Code - The address of a CARDINAL32 in  */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output: This function returns the Drive_Information_Record for  */
//*           the drive associated with the specified Drive_Handle.   */
//*           If no errors occur, *Error_Code will be set to 0.  If an*/
//*           error does occur, then *Error_Code will be non-zero.    */
//*                                                                   */
//*   Error Handling:  If an error occurs, then *Error_Code will be   */
//*                    non-zero.                                      */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Drive_Status(Drive_Handle: ADDRESS; Error_Code: PCARDINAL32): Drive_Information_Record; external 'lvm' name 'Get_Drive_Status';

//****************************************************************************
//
// Functions relating to Partitions
//
//***************************************************************************

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Partitions                                   */
//*                                                                   */
//*   Descriptive Name: Returns an array of partitions associated     */
//*                     with the object specified by Handle.          */
//*                                                                   */
//*   Input:ADDRESS Handle - This is the handle of a drive or volume. */
//*                      Drive handles are obtained through the       */
//*                      Get_Drive_Control_Data function.  Volume     */
//*                      handles are obtained through the             */
//*                      Get_Volume_Control_Data function.            */
//*         CARDINAL32 * Error_Code - The address of a CARDINAL32 in  */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output: This function returns a structure.  The structure has   */
//*           two components: an array of partition information       */
//*           records and the number of entries in the array.  If     */
//*           Handle is the handle of a disk drive, then the returned */
//*           array will contain a partition information record for   */
//*           each partition and block of free space on that drive.   */
//*           If Handle is the handle of a volume, then the returned  */
//*           array will contain a partition information record for   */
//*           each partition which is part of the specified volume.   */
//*           If no errors occur, then *Error_Code will be 0.  If an  */
//*           error does occur, then *Error_Code will be non-zero.    */
//*                                                                   */
//*   Error Handling: Any memory allocated for the return value will  */
//*                   be freed.  The Partition_Information_Array      */
//*                   returned by this function will contain a NULL   */
//*                   pointer for Partition_Array, and have a Count of*/
//*                   0.  *Error_Code will be non-zero.               */
//*                                                                   */
//*                   If Handle is non-NULL and is invalid, a trap    */
//*                   is likely.                                      */
//*                                                                   */
//*   Side Effects:  Memory will be allocated to hold the array       */
//*                  returned by this function.                       */
//*                                                                   */
//*   Notes:  The caller becomes responsible for the memory allocated */
//*           for the array of Partition_Information_Records pointed  */
//*           to by Partition_Array pointer in the                    */
//*           Partition_Information_Array structure returned by this  */
//*           function.  The caller should free this memory when they */
//*           are done using it.                                      */
//*                                                                   */
//*********************************************************************/
function Get_Partitions(Handle: ADDRESS; Error_Code: PCARDINAL32): Partition_Information_Array; external 'lvm' name 'Get_Partitions';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Partition_Handle                             */
//*                                                                   */
//*   Descriptive Name: Returns the handle of the partition whose     */
//*                     serial number matches the one provided.       */
//*                                                                   */
//*   Input: CARDINAL32 Serial_Number - This is the serial number to  */
//*                                     look for.  If a partition with*/
//*                                     a matching serial number is   */
//*                                     found, its handle will be     */
//*                                     returned.                     */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If a partition with a matching serial number is found,  */
//*              then the function return value will be the handle    */
//*              of the partition found.  If no matching partition is */
//*              found, then the function return value will be NULL.  */
//*                                                                   */
//*   Error Handling:  If no errors occur, *Error_Code will be        */
//*                    LVM_ENGINE_NO_ERROR.  If an error occurs, then */
//*                    *Error_Code will be a non-zero error code.     */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Partition_Handle(Serial_Number: CARDINAL32; Error_Code: PCARDINAL32): ADDRESS; external 'lvm' name 'Get_Partition_Handle';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Partition_Information                        */
//*                                                                   */
//*   Descriptive Name: Returns the Partition_Information_Record for  */
//*                     the partition specified by Partition_Handle.  */
//*                                                                   */
//*   Input: ADDRESS Partition_Handle - The handle associated with the*/
//*                                     partition for which the       */
//*                                     Partition_Information_Record  */
//*                                     is desired.                   */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: A Partition_Information_Record is returned.  If there   */
//*           is no error, then *Error_Code will be 0.  If an error   */
//*           occurs, *Error_Code will be non-zero.                   */
//*                                                                   */
//*   Error Handling:  If the Partition_Handle is not a valid handle, */
//*                    a trap could result.  If it is a handle for    */
//*                    something other than a partition, an error code*/
//*                    will be returned in *Error_Code.               */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Partition_Information(Partition_Handle: ADDRESS; Error_Code: PCARDINAL32): Partition_Information_Record; external 'lvm' name 'Get_Partition_Information';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Create_Partition                                 */
//*                                                                   */
//*   Descriptive Name: Creates a partition on a disk drive.          */
//*                                                                   */
//*   Input: ADDRESS         Handle - The handle of a disk drive or   */
//*                                   a block of free space.          */
//*          CARDINAL32      Size - The size, in sectors, of the      */
//*                                 partition to create.              */
//*          char            Name[] - The name to give to the newly   */
//*                                   created partition.              */
//*          Allocation_Algorithm algorithm - If Handle is a drive,   */
//*                                           then the engine will    */
//*                                           find a block of free    */
//*                                           space to use to create  */
//*                                           the partition.  This    */
//*                                           tells the engine which  */
//*                                           memory management       */
//*                                           algorithm to use.       */
//*          BOOLEAN         Bootable - If TRUE, then the engine will */
//*                                     only create the partition if  */
//*                                     it can be booted from.  If    */
//*                                     Primary_Partition is FALSE,   */
//*                                     then it is assumed that OS/2  */
//*                                     is the operating system that  */
//*                                     will be booted.               */
//*          BOOLEAN         Primary_Partition - If TRUE, then the    */
//*                                              engine will create   */
//*                                              a primary partition. */
//*                                              If FALSE, then the   */
//*                                              engine will create a */
//*                                              logical drive.       */
//*          BOOLEAN         Allocate_From_Start - If TRUE, then the  */
//*                                                engine will        */
//*                                                allocate the new   */
//*                                                partition from the */
//*                                                beginning of the   */
//*                                                selected block of  */
//*                                                free space.  If    */
//*                                                FALSE, then the    */
//*                                                partition will be  */
//*                                                allocated from the */
//*                                                end of the selected*/
//*                                                block of free      */
//*                                                space.             */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: The function return value will be the handle of the     */
//*           partition created.  If the partition could not be       */
//*           created, then NULL will be returned.                    */
//*           *Error_Code will be 0 if the partition was created.     */
//*           *Error_Code will be > 0 if the partition could not be   */
//*           created.                                                */
//*                                                                   */
//*   Error Handling:  If the partition can not be created, then any  */
//*                    memory allocated by this function will be      */
//*                    freed and the partitioning of the disk in      */
//*                    question will be unchanged.                    */
//*                                                                   */
//*                    If Handle is not a valid handle, then a trap   */
//*                    may result.                                    */
//*                                                                   */
//*                    If Handle represents a partition or volume,    */
//*                    then the function will abort and set           */
//*                    *Error_Code to a non-zero value.               */
//*                                                                   */
//*   Side Effects:  A partition may be created on a disk drive.      */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
type
  TPartitionName=Array[0..PARTITION_NAME_SIZE-1] of char;

function Create_Partition(Handle: ADDRESS;
Size: CARDINAL32;
Name: TPartitionName;
algorithm: Allocation_Algorithm;
Bootable: BOOLEAN;
Primary_Partition: BOOLEAN;
Allocate_From_Start: BOOLEAN;
Error_Code: PCARDINAL32
): ADDRESS; external 'lvm' name 'Create_Partition';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Delete_Partition                                 */
//*                                                                   */
//*   Descriptive Name: Deletes the partition specified by            */
//*                     Partition_Handle.                             */
//*                                                                   */
//*   Input: ADDRESS Partition_Handle - The handle associated with the*/
//*                                     partition to be deleted.      */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the partition was deleted      */
//*           successfully.  *Error_Code will be > 0 if the partition */
//*           could not be deleted.                                   */
//*                                                                   */
//*   Error Handling: If the partition can not be deleted, then       */
//*                   *Error_Code will be > 0.                        */
//*                                                                   */
//*                   If Partition_Handle is not a valid handle, a    */
//*                   trap may result.                                */
//*                                                                   */
//*                   If Partition_Handle is a volume or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*   Side Effects: A partition on a disk drive may be deleted.       */
//*                                                                   */
//*   Notes:  A partition can not be deleted if it is part of a       */
//*           volume!                                                 */
//*                                                                   */
//*********************************************************************/
procedure Delete_Partition(Partition_Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'Delete_Partition';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Active_Flag                                  */
//*                                                                   */
//*   Descriptive Name: Sets the Active Flag field for a partition.   */
//*                                                                   */
//*   Input: ADDRESS Partition_Handle - The handle of the partition   */
//*                                     whose Active Flag is to be set*/
//*          BYTE Active_Flag - The new value for the Active Flag.    */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the Active Flag was            */
//*           successfully set, otherwise *Error_Code will contain a  */
//*           non-zero error code indicating what went wrong.         */
//*                                                                   */
//*   Error Handling: If the Active Flag can not be set, this function*/
//*                   will abort without changing any disk structures.*/
//*                                                                   */
//*                   If Partition_Handle is not a valid handle, a    */
//*                   trap may result.                                */
//*                                                                   */
//*                   If Partition_Handle is a volume or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*                                                                   */
//*   Side Effects:  The Active Flag for a partition may be modified. */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Active_Flag(Partition_Handle: ADDRESS;
Active_Flag: BYTE;
Error_Code: PCARDINAL32
); external 'lvm' name 'Set_Active_Flag';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_OS_Flag                                      */
//*                                                                   */
//*   Descriptive Name: Sets the OS Flag field for a partition.  This */
//*                     field is typically used to indicate the       */
//*                     filesystem used on the partition, which       */
//*                     generally gives an indication of which OS is  */
//*                     using that partition.                         */
//*                                                                   */
//*   Input: ADDRESS Partition_Handle - The handle of the partition   */
//*                                     whose Active Flag is to be set*/
//*          BYTE OS_Flag - The new value for the OS Flag.            */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the OS Flag was successfully   */
//*           set, otherwise *Error_Code will contain a non-zero error*/
//*           code indicating what went wrong.                        */
//*                                                                   */
//*   Error Handling: If the OS Flag can not be set, this function    */
//*                   will abort without changing any disk structures.*/
//*                                                                   */
//*                   If Partition_Handle is not a valid handle, a    */
//*                   trap may result.                                */
//*                                                                   */
//*                   If Partition_Handle is a volume or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*                                                                   */
//*   Side Effects:  The OS Flag for a partition may be modified.     */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_OS_Flag(Partition_Handle: ADDRESS;
OS_Flag: BYTE;
Error_Code: PCARDINAL32
); external 'lvm' name 'Set_OS_Flag';

//****************************************************************************
//
// Functions relating to Volumes
//
//***************************************************************************

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Volume_Control_Data                          */
//*                                                                   */
//*   Descriptive Name: This function returns a structure containing  */
//*                     an array of Volume_Control_Records.  These    */
//*                     records contain information about volumes     */
//*                     which is invariant - i.e. will not change for */
//*                     as long as the volume exists.  One of the     */
//*                     items in the Volume_Control_Record is the     */
//*                     handle for the volume.  This handle must be   */
//*                     used on all accesses to the volume.           */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: A Volume_Control_Array structure is returned.           */
//*                                                                   */
//*           If there are no errors, then the Volume_Control_Data    */
//*           pointer in the Volume_Control_Array will be non-NULL,   */
//*           the Count field of the Volume_Control_Array will be     */
//*           >= 0, and *Error_Code will be 0.                        */
//*                                                                   */
//*           If an error does occur, then the Volume_Control_Data    */
//*           pointer in the the Volume_Control_Array will be NULL,   */
//*           the Count field of the Volume_Control_Array will be 0,  */
//*           and *Error_Code will be > 0.                            */
//*                                                                   */
//*   Error Handling: If an error occurs, then any memory allocated by*/
//*                   this function will be freed.                    */
//*                                                                   */
//*   Side Effects:  Memory for the returned array is allocated.      */
//*                                                                   */
//*   Notes:  The caller becomes responsible for the memory allocated */
//*           for the array of Volume_Control_Records pointed to by   */
//*           Volume_Control_Data pointer in the Volume_Control_Array */
//*           structure returned by this function.  The caller should */
//*           free this memory when they are done using it.           */
//*                                                                   */
//*********************************************************************/
function Get_Volume_Control_Data(Error_Code: PCARDINAL32): Volume_Control_Array; external 'lvm' name 'Get_Volume_Control_Data';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Volume_Information                           */
//*                                                                   */
//*   Descriptive Name:  This function returns the                    */
//*                      Volume_Information_Record for the volume     */
//*                      associated with Volume_Handle.               */
//*                                                                   */
//*   Input: ADDRESS Volume_Handle - The handle of the volume about   */
//*                                  which information is desired.    */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: This function returns a Volume_Information_Record.      */
//*                                                                   */
//*           If this function is successful, then *Error_Code will be*/
//*              0.                                                   */
//*                                                                   */
//*           If this function fails, then *Error_Code will be > 0.   */
//*                                                                   */
//*   Error Handling:  If Volume_Handle is not a valid handle, a trap */
//*                    will be likely.  If Volume_Handle is a drive or*/
//*                    partition handle, *Error_Code will be > 0.     */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Volume_Information(Volume_Handle: ADDRESS; Error_Code: PCARDINAL32): Volume_Information_Record; external 'lvm' name 'Get_Volume_Information';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Create_Volume                                    */
//*                                                                   */
//*   Descriptive Name:  This function creates a volume from a list of*/
//*                      partitions.  The partitions are specified by */
//*                      their corresponding handles.                 */
//*                                                                   */
//*   Input: char         Name[] - The name to assign to the newly    */
//*                                created volume.                    */
//*          BOOLEAN      Create_LVM_Volume - If TRUE, then an LVM    */
//*                                           volume is created,      */
//*                                           otherwise a             */
//*                                           compatibility volume is */
//*                                           created.                */
//*          BOOLEAN      Bootable - If TRUE, the volume will not be  */
//*                                  created unless OS/2 can be booted*/
//*                                  from it.                         */
//*          char         Drive_Letter_Preference - This is the drive */
//*                                                 letter to use for */
//*                                                 accessing the     */
//*                                                 newly created     */
//*                                                 volume.           */
//*          CARDINAL32   FeaturesToUse - This is currently reserved  */
//*                                       for future use and should   */
//*                                       always be set to 0.         */
//*          CARDINAL32   Partition_Count - The number of partitions  */
//*                                         to link together to form  */
//*                                         the volume being created. */
//*          ADDRESS      Partition_Handles[] - An array of partition */
//*                                             handles with one entry*/
//*                                             for each partition    */
//*                                             that is to become part*/
//*                                             of the volume being   */
//*                                             created.              */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the volume was created.        */
//*           *Error_Code will be > 0 if the volume could not be      */
//*              created.                                             */
//*                                                                   */
//*   Error Handling: If any of the handles in the partition handles  */
//*                   array is not valid, then a trap is likely.  If  */
//*                   Partition_Count is greater than the number of   */
//*                   entries in the partition handles array, then a  */
//*                   trap is likely.  If any of the handles in the   */
//*                   partition array are not partition handles, then */
//*                   *Error_Code will be > 0.  If the volume can NOT */
//*                   be created, then *Error_Code will be > 0 and any*/
//*                   memory allocated by this function will be freed.*/
//*                   If the volume can NOT be created, then the      */
//*                   existing partition/volume structure of the disk */
//*                   will be unchanged.                              */
//*                                                                   */
//*   Side Effects:  A volume may be created.                         */
//*                                                                   */
//*   Notes:  This function provides limited compatibility for        */
//*           programs written to use the LVM Version 1 interface.    */
//*           Specifically, this function will only allow the         */
//*           creation of compatibility volumes.  Any attempt to      */
//*           create an LVM volume will result in an error code being */
//*           returned.                                               */
//*                                                                   */
//*********************************************************************/
type
  TVolumeName=Array[0..VOLUME_NAME_SIZE-1] of char;

procedure Create_Volume(Name: TVolumeName;
Create_LVM_Volume: BOOLEAN;
Bootable: BOOLEAN;
Drive_Letter_Preference: char;
FeaturesToUse: CARDINAL32;
Partition_Count: CARDINAL32;
Partition_Handles: Array of ADDRESS;
Error_Code: PCARDINAL32
); external 'lvm' name 'Create_Volume';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Create_Volume2                                   */
//*                                                                   */
//*   Descriptive Name:  This function creates a volume from a list of*/
//*                      partitions.  The partitions are specified by */
//*                      their corresponding handles.                 */
//*                                                                   */
//*   Input: char         Name[] - The name to assign to the newly    */
//*                                created volume.                    */
//*          BOOLEAN      Create_LVM_Volume - If TRUE, then an LVM    */
//*                                           volume is created,      */
//*                                           otherwise a             */
//*                                           compatibility volume is */
//*                                           created.                */
//*          BOOLEAN      Bootable - If TRUE, the volume will not be  */
//*                                  created unless OS/2 can be booted*/
//*                                  from it.                         */
//*          char         Drive_Letter_Preference - This is the drive */
//*                                                 letter to use for */
//*                                                 accessing the     */
//*                                                 newly created     */
//*                                                 volume.           */
//*          CARDINAL32   Feature_Count - The number of features to   */
//*                                       install on the volume being */
//*                                       created.  This field is     */
//*                                       ignored if Create_LVM_Volume*/
//*                                       is FALSE.                   */
//*          LVM_Feature_Specification_Record FeaturesToUse[] - An    */
//*                                         array of feature IDs and  */
//*                                         their associated LVM      */
//*                                         classes used to designate */
//*                                         which features to install */
//*                                         on the volume being       */
//*                                         created and the order in  */
//*                                         which to install them.    */
//*                                         This field is ignored if  */
//*                                         Create_LVM_Volume is      */
//*                                         FALSE.                    */
//*          CARDINAL32   Partition_Count - The number of partitions  */
//*                                         to link together to form  */
//*                                         the volume being created. */
//*          ADDRESS      Partition_Handles[] - An array of partition */
//*                                             handles with one entry*/
//*                                             for each partition    */
//*                                             that is to become part*/
//*                                             of the volume being   */
//*                                             created.              */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the volume was created.        */
//*           *Error_Code will be > 0 if the volume could not be      */
//*              created.                                             */
//*                                                                   */
//*   Error Handling: If any of the handles in the partition handles  */
//*                   array is not valid, then a trap is likely.  If  */
//*                   Partition_Count is greater than the number of   */
//*                   entries in the partition handles array, then a  */
//*                   trap is likely.  If any of the handles in the   */
//*                   partition array are not partition handles, then */
//*                   *Error_Code will be > 0.  If the volume can NOT */
//*                   be created, then *Error_Code will be > 0 and any*/
//*                   memory allocated by this function will be freed.*/
//*                   If the volume can NOT be created, then the      */
//*                   existing partition/volume structure of the disk */
//*                   will be unchanged.                              */
//*                                                                   */
//*   Side Effects:  A volume may be created.                         */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
void _System Create_Volume2( char                               Name[VOLUME_NAME_SIZE],
BOOLEAN                            Create_LVM_Volume,
BOOLEAN                            Bootable,
char                               Drive_Letter_Preference,
CARDINAL32                         Feature_Count,
LVM_Feature_Specification_Record   FeaturesToUse[],
CARDINAL32                         Partition_Count,
ADDRESS                            Partition_Handles[],
CARDINAL32 *                       Error_Code
);
{$endif}

//*********************************************************************/
//*                                                                   */
//*   Function Name: Delete_Volume                                    */
//*                                                                   */
//*   Descriptive Name: Deletes the volume specified by Volume_Handle.*/
//*                                                                   */
//*   Input: ADDRESS Volume_Handle - The handle of the volume to      */
//*                                  delete.  All partitions which are*/
//*                                  part of the specified volume will*/
//*                                  be deleted also.                 */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the volume and its partitions  */
//*           are successfully deleted.  Otherwise, *Error_Code will  */
//*           be > 0.                                                 */
//*                                                                   */
//*   Error Handling: *Error_Code will be > 0 if an error occurs.  If */
//*                   the volume or any of its partitions can not be  */
//*                   deleted, then any changes made by this function */
//*                   will be undone.                                 */
//*                                                                   */
//*                   If Volume_Handle is not a valid handle, a trap  */
//*                   may result.                                     */
//*                                                                   */
//*                   If Volume_Handle is a partition or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*   Side Effects:  A volume and its partitions may be deleted.      */
//*                  System memory may be freed as the internal       */
//*                  structures used to track the deleted volume      */
//*                  are no longer required.                          */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Delete_Volume(Volume_Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'Delete_Volume';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Hide_Volume                                      */
//*                                                                   */
//*   Descriptive Name: Hide volume "hides" a volume from OS/2 by     */
//*                     removing its drive letter assignment.  Without*/
//*                     a drive letter assignment, OS/2 can not access*/
//*                     (or "see") the volume.                        */
//*                                                                   */
//*   Input: ADDRESS Volume_Handle - The handle of the volume to hide.*/
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the volume was successfully    */
//*           hidden.  If the volume could not be hidden, then        */
//*           *Error_Code will be > 0.                                */
//*                                                                   */
//*   Error Handling: *Error_Code will be > 0 if the volume can not be*/
//*                   hidden.  If the volume can not be hidden, then  */
//*                   nothing will be altered.                        */
//*                                                                   */
//*                   If Volume_Handle is not a valid handle, a trap  */
//*                   may result.                                     */
//*                                                                   */
//*                   If Volume_Handle is a partition or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Hide_Volume(Volume_Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'Hide_Volume';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Expand_Volume                                    */
//*                                                                   */
//*   Descriptive Name: This function expands an existing volume by   */
//*                     linking additional partitions to it.          */
//*                                                                   */
//*   Input: ADDRESS Volume_Handle - The handle of the volume to be   */
//*                                  expanded.                        */
//*          CARDINAL32 Partition_Count - The number of partitions or */
//*                                       volumes to be added to the  */
//*                                       volume being expanded.      */
//*          ADDRESS Partition_Handles[] - An array of handles.  Each */
//*                                        handle in the array is the */
//*                                        handle of a partition      */
//*                                        which is to be added to    */
//*                                        the volume being expanded. */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the volume is successfully     */
//*           expanded.  If the volume can not be expanded,           */
//*           *Error_Code will be > 0.                                */
//*                                                                   */
//*   Error Handling: If the volume can not be expanded, the state of */
//*                   the volume is unchanged and any memory allocated*/
//*                   by this function is freed.                      */
//*                                                                   */
//*                   If Volume_Handle is not a valid handle, a trap  */
//*                   may result.                                     */
//*                                                                   */
//*                   If Volume_Handle is a partition or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*                   If any of the partition handles in the          */
//*                   Partition_handles array are not valid handles,  */
//*                   then a trap may result.                         */
//*                                                                   */
//*                   If any of the partition handles in the          */
//*                   Partition_Handles array are actually drive      */
//*                   handles, then this function will abort and      */
//*                   set *Error_Code to a non-zero value.            */
//*                                                                   */
//*                   If Partition_Count is greater than the number of*/
//*                   entries in the Partition_Handles array, a trap  */
//*                   may result.                                     */
//*                                                                   */
//*   Side Effects:  A volume may be expanded.  If the volume is      */
//*                  expanded using another volume, the partitions    */
//*                  on the second volume will be linked to those of  */
//*                  the first volume and all data on the second      */
//*                  volume will be lost.                             */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Expand_Volume(Volume_Handle: ADDRESS;
Partition_Count: CARDINAL32;
Partition_Handles: Array of ADDRESS;
Error_Code: PCARDINAL32
); external 'lvm' name 'Expand_Volume';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Assign_Drive_Letter                              */
//*                                                                   */
//*   Descriptive Name: Assigns a drive letter to a volume.           */
//*                                                                   */
//*   Input: ADDRESS Volume_Handle - The handle of the volume which   */
//*                                  is to have its assigned drive    */
//*                                  letter changed.                  */
//*          char  New_Drive_Preference - The new drive letter to     */
//*                                       assign to the volume.       */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the drive letter was assigned  */
//*           successfully; otherwise *Error_Code will be > 0.        */
//*                                                                   */
//*   Error Handling: If the drive letter assignment can not be made, */
//*                   the volume will not be altered.                 */
//*                                                                   */
//*                   If Volume_Handle is not a valid handle, a trap  */
//*                   may result.                                     */
//*                                                                   */
//*                   If Volume_Handle is a partition or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*   Side Effects:  A volume may have its drive letter assignment    */
//*                  changed.                                         */
//*                                                                   */
//*   Notes:  If the drive letter being assigned is already in use by */
//*           volume which does not lie on removable media, then the  */
//*           drive assignment will NOT be made.                      */
//*                                                                   */
//*********************************************************************/
procedure Assign_Drive_Letter(Volume_Handle: ADDRESS;
New_Drive_Preference: char;
Error_Code: PCARDINAL32
); external 'lvm' name 'Assign_Drive_Letter';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Installable                                  */
//*                                                                   */
//*   Descriptive Name: Marks a volume as being the volume to install */
//*                     OS/2 on.                                      */
//*                                                                   */
//*   Input: ADDRESS Volume_Handle - The handle of the volume to which*/
//*                                  OS/2 should be installed.        */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If the volume is successfully marked as installable,    */
//*           *Error_Code will be 0; otherwise *Error_Code will       */
//*           be > 0.                                                 */
//*                                                                   */
//*   Error Handling: If Volume_Handle is not a valid handle, a trap  */
//*                   may result.                                     */
//*                                                                   */
//*                   If Volume_Handle is a partition or drive handle,*/
//*                   then this function will abort and set           */
//*                   *Error_Code to a non-zero value.                */
//*                                                                   */
//*   Side Effects:  The specified volume may be marked as            */
//*                  installable.                                     */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Installable(Volume_Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'Set_Installable';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Installable_Volume                           */
//*                                                                   */
//*   Descriptive Name: Marks a volume as being the volume to install */
//*                     OS/2 on.                                      */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If a volume is mared installable, its information will  */
//*           be returned and *Error_Code will be LVM_ENGINE_NO_ERROR.*/
//*           If there is no volume marked installable, then          */
//*           *Error_Code will be > 0.                                */
//*                                                                   */
//*   Error Handling: An error code is returned if there is an error. */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Installable_Volume(Error_Code: PCARDINAL32): Volume_Information_Record; external 'lvm' name 'Get_Installable_Volume';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Convert_Volumes_To_V1                            */
//*                                                                   */
//*   Descriptive Name: This function attempts to convert all LVM     */
//*                     volumes in the system into a format that can  */
//*                     be used by LVM Version 1, which was shipped   */
//*                     with Warp Server for e-business.  This        */
//*                     function returns a bitmap of the drive letters*/
//*                     corresponding to Volumes that can not be      */
//*                     converted.                                    */
//*                                                                   */
//*   Input: BOOLEAN * Hidden_Volume_Conversion_Failure - The address */
//*                                 of a BOOLEAN variable in which    */
//*                                 to store a flag indicating if     */
//*                                 there were hidden volumes that    */
//*                                 could not be converted.  If       */
//*                                 *Hidden_Volume_Conversion_Failure */
//*                                 is TRUE, then there were hidden   */
//*                                 volumes that could not be         */
//*                                 converted.  If FALSE, then there  */
//*                                 were no hidden volumes, or the    */
//*                                 hidden volumes that existed were  */
//*                                 converted successfully.           */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: This function returns a bitmap of the drive letters     */
//*           corresponding to volumes that could not be converted to */
//*           LVM Version 1 format.  If this function is successful   */
//*           and all volumes were converted, then *Error_Code will be*/
//*           set to LVM_ENGINE_NO_ERROR and the bitmap returned will */
//*           have no bits set.  If this function failes, *Error_Code */
//*           will contain a non-zero error code and the bitmap       */
//*           returned by this function may be non-zero.              */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  All LVM volumes that can be converted to LVM     */
//*                  Version 1 format will be.                        */
//*                                                                   */
//*   Notes:  Bit 0 in the bitmap returned by this function represents*/
//*           drive letter 'A'.                                       */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
CARDINAL32 _System Convert_Volumes_To_V1 ( BOOLEAN *    Hidden_Volume_Conversion_Failure,
CARDINAL32 * Error_Code ) ;
{$endif}

//***************************************************************************
//
// Functions relating to Partitions, Drives, and Volumes.
//
//***************************************************************************

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Name                                         */
//*                                                                   */
//*   Descriptive Name: Sets the name of a volume, drive, or partition*/
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of the drive, partition, or  */
//*                           volume which is to have its name set.   */
//*          char New_Name[] - The new name for the drive/partition/  */
//*                            volume.                                */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the name is set as specified.  */
//*           If the name can not be set, *Error_Code will be > 0.    */
//*                                                                   */
//*   Error Handling: If the name can not be set, then drive/volume/  */
//*                   partition is not modified.                      */
//*                                                                   */
//*                   If Handle is not a valid handle, a trap may     */
//*                   result.                                         */
//*                                                                   */
//*   Side Effects:  A drive/volume/partition may have its name set.  */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Name(Handle: ADDRESS;
New_Name: Array of char;
Error_Code: PCARDINAL32
); external 'lvm' name 'Set_Name';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Startable                                    */
//*                                                                   */
//*   Descriptive Name: Sets the specified volume or partition        */
//*                     startable.  If a volume is specified, it must */
//*                     be a compatibility volume whose partition is  */
//*                     a primary partition on the first drive.  If a */
//*                     partition is specified, it must be a primary  */
//*                     partition on the first drive in the system.   */
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of the partition or volume   */
//*                           which is to be set startable.           */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the specified volume or        */
//*           partition was set startable.                            */
//*           If the name can not be set, *Error_Code will be > 0.    */
//*                                                                   */
//*   Error Handling: If the volume or partition could not be set     */
//*                   startable, then nothing in the system is        */
//*                   changed.                                        */
//*                                                                   */
//*                   If Handle is not a valid handle, a trap may     */
//*                   result.                                         */
//*                                                                   */
//*   Side Effects:  Any other partition or volume which is marked    */
//*                  startable will have its startable flag cleared.  */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Startable(Handle: ADDRESS;
Error_Code: PCARDINAL32
); external 'lvm' name 'Set_Startable';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Valid_Options                                */
//*                                                                   */
//*   Descriptive Name: Returns a bitmap where each bit in the bitmap */
//*                     corresponds to a possible operation that the  */
//*                     LVM Engine can perform.  Those bits which are */
//*                     1 represent operations which can be performed */
//*                     on the item specified by Handle.  Those bits  */
//*                     which are 0 are not allowed on the item       */
//*                     specified by Handle.                          */
//*                                                                   */
//*   Input: ADDRESS Handle - This is any valid drive, volume, or     */
//*                           partition handle.                       */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output:  A bitmap indicating which operations are valid on the  */
//*            item specified by Handle.                              */
//*                                                                   */
//*            If no errors occur, *Error_Code will be 0, otherwise   */
//*            *Error_Code will be > 0.                               */
//*                                                                   */
//*   Error Handling:  If Handle is not valid, a trap will be likely. */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The values of the various bits in the bitmap returned   */
//*           by this function are defined near the beginning of this */
//*           file, immediately after all of the structure            */
//*           definitions.                                            */
//*                                                                   */
//*********************************************************************/
function Get_Valid_Options(Handle: ADDRESS; Error_Code: PCARDINAL32): CARDINAL32; external 'lvm' name 'Get_Valid_Options';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Child_Handles                                */
//*                                                                   */
//*   Descriptive Name: Given the handle of a volume or aggregate,    */
//*                     this function will return the handles of the  */
//*                     children of the volume or aggregate.  This    */
//*                     allows the entire tree representation of a    */
//*                     volume to be traversed, a level at a time.    */
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of the volume or aggregate   */
//*                           whose children are required.            */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output:  If successful, an LVM_Handle_Array_Record is returned  */
//*            with a non-zero Count.  Also, *Error_Code will be set  */
//*            to LVM_ENGINE_NO_ERROR.  If an error occurs, then      */
//*            the Count field will be 0 and *Error_Code will contain */
//*            a non-zero error code.                                 */
//*                                                                   */
//*   Error Handling: If Handle is not a valid handle, then a trap is */
//*                   likely.  If Handle is the handle of partition,  */
//*                   then *Error_Code will be set to                 */
//*                   LVM_ENGINE_NO_CHILDREN.  If Handle is not a     */
//*                   volume or aggregate handle, then *Error_Code    */
//*                   will be set to LVM_ENGINE_BAD_HANDLE.           */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
LVM_Handle_Array_Record  _System Get_Child_Handles( ADDRESS Handle, CARDINAL32 * Error_Code);
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Parent_Handle                                */
//*                                                                   */
//*   Descriptive Name: Given the handle of a partition or aggregate, */
//*                     this function will return the handle of the   */
//*                     parent of the partition or aggregate.         */
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of the partition or aggregate*/
//*                           whose parent is required.               */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output:  If successful, the handle of the parent is returned    */
//*            as the function result and *Error_Code will be set to  */
//*            LVM_ENGINE_NO_ERROR.                                   */
//*            If an error occurs, then NULL will be the function     */
//*            result and *Error_Code will contain a non-zero error   */
//*            code.                                                  */
//*                                                                   */
//*   Error Handling: If Handle is not a valid handle, then a trap is */
//*                   likely.  If Handle is the handle of volume,     */
//*                   then *Error_Code will be set to                 */
//*                   LVM_ENGINE_NO_PARENT.  If Handle is not the     */
//*                   handle of a volume, partition, or aggregate     */
//*                   then *Error_Code will be set to                 */
//*                   LVM_ENGINE_BAD_HANDLE.                          */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
ADDRESS _System Get_Parent_Handle( ADDRESS Handle, CARDINAL32 * Error_Code);
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Features                                     */
//*                                                                   */
//*   Descriptive Name: Returns the feature ID information for each of*/
//*                     the features that are installed on the        */
//*                     item specified by Handle.                     */
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of the object to use.        */
//*                                                                   */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                   in which to store an error code */
//*                                   should an error occur.          */
//*                                                                   */
//*   Output: If successful, a Feature_Information_Array structure is */
//*           returned with a non-zero Count.  Also, *Error_Code will */
//*           be set to LVM_ENGINE_NO_ERROR.  If an error occurs,     */
//*           then the Count field in the structure will be 0 and     */
//*           ( *Error_Code) will contain a non-zero error code.       */
//*                                                                   */
//*   Error Handling:  If Handle is not a valid handle, a trap        */
//*                    will be likely.                                */
//*                                                                   */
//*   Side Effects:  Memory is allocated using the LVM Engine's memory*/
//*                  manager for the array of Feature_ID_Data items   */
//*                  being returned.                                  */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
Feature_Information_Array _System Get_Features( ADDRESS Handle, CARDINAL32 * Error_Code );
{$endif}
//***************************************************************************
//
// Functions relating to Boot Manager
//
//***************************************************************************

//*********************************************************************/
//*                                                                   */
//*   Function Name: Boot_Manager_Is_Installed                        */
//*                                                                   */
//*   Descriptive Name: Indicates whether or not Boot Manager is      */
//*                     installed on the first or second hard drives  */
//*                     in the system.                                */
//*                                                                   */
//*   Input: BOOLEAN * Active - *Active is set to TRUE if LVM found an*/
//*                             active copy of Boot Manager on the    */
//*                             system.  If LVM could not find an     */
//*                             active copy of Boot Manager on the    */
//*                             system, but did find an inactive copy */
//*                             of Boot Manager, then *Active will be */
//*                             set to FALSE.                         */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: TRUE is returned if Boot Manager is found.  If this     */
//*           copy of Boot Manager is Active, then *Active will be set*/
//*           to TRUE.  If the copy of Boot Manager is not currently  */
//*           active, then *Active will be set to FALSE.              */
//*                                                                   */
//*           FALSE is returned if Boot Manager is not found or if an */
//*           error occurs.  In this case, *Active is undefined.      */
//*                                                                   */
//*           *Error_Code will be 0 if no errors occur; otherwise it  */
//*           will be > 0.                                            */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
type
  PBOOLEAN=^BOOLEAN;

function Boot_Manager_Is_Installed(Active: PBOOLEAN; Error_Code: PCARDINAL32): BOOLEAN; external 'lvm' name 'Boot_Manager_Is_Installed';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Boot_Manager_Handle                          */
//*                                                                   */
//*   Descriptive Name: Returns the handle of the partition containing*/
//*                     Boot Manager.                                 */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If Boot Manager is NOT installed, NULL is returned.     */
//*           If Boot Manager is installed, whether it is active or   */
//*           not, the handle of the partition it resides in is       */
//*           returned.                                               */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:                                                          */
//*                                                                   */
//*********************************************************************/
function Get_Boot_Manager_Handle(Error_Code: PCARDINAL32): ADDRESS; external 'lvm' name 'Get_Boot_Manager_Handle';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Add_To_Boot_Manager                              */
//*                                                                   */
//*   Descriptive Name: Adds the volume/partition to the Boot Manager */
//*                     menu.                                         */
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of a partition or volume that*/
//*                           is to be added to the Boot Manager menu.*/
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the partition or volume was    */
//*           successfully added to the Boot Manager menu; otherwise  */
//*           *Error_Code will be > 0.                                */
//*                                                                   */
//*   Error Handling: If the partition/volume can not be added to the */
//*                   Boot Manager menu, no action is taken and       */
//*                   *Error_Code will contain a non-zero error code. */
//*                                                                   */
//*                   If Handle is not a valid handle, a trap may     */
//*                   result.                                         */
//*                                                                   */
//*                   If Handle represents a drive, then this function*/
//*                   will abort and set *Error_Code to a non-zero    */
//*                   value.                                          */
//*                                                                   */
//*   Side Effects:  The Boot Manager menu may be altered.            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Add_To_Boot_Manager(Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'Add_To_Boot_Manager';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Remove_From_Boot_Manager                         */
//*                                                                   */
//*   Descriptive Name: Removes the specified partition or volume     */
//*                     from the Boot Manager menu.                   */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the partition or volume was    */
//*           successfully removed to the Boot Manager menu;          */
//*           otherwise *Error_Code will be > 0.                      */
//*                                                                   */
//*   Error Handling: If Handle is not a valid handle, a trap may     */
//*                   result.                                         */
//*                                                                   */
//*                   If Handle represents a drive, or if Handle      */
//*                   represents a volume or partition which is NOT on*/
//*                   the boot manager menu, then this function       */
//*                   will abort and set *Error_Code to a non-zero    */
//*                   value.                                          */
//*                                                                   */
//*   Side Effects:  The Boot Manager menu may be altered.            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Remove_From_Boot_Manager(Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'Remove_From_Boot_Manager';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Boot_Manager_Menu                            */
//*                                                                   */
//*   Descriptive Name: Returns an array containing the handles of the*/
//*                     partitions and volumes appearing on the       */
//*                     Boot Manager menu.                            */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: The function returns a Boot_Manager_Menu structure.     */
//*           This structure contains two items: a pointer to an array*/
//*           of Boot_Manager_Menu_Items and a count of how many items*/
//*           are in the array.  Each Boot_Manager_Menu_Item contains */
//*           a handle and a BOOLEAN variable to indicate whether the */
//*           handle is for a partition or a volume.                  */
//*                                                                   */
//*           If this function is successful, then *Error_Code will   */
//*           be 0.                                                   */
//*                                                                   */
//*           If an error occurs, the Count field in the              */
//*           Boot_Manager_Menu will be 0 and the corresponding       */
//*           pointer will be NULL.  *Error_Code will be > 0.         */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                   any memory allocated by this function will be   */
//*                   freed.                                          */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Boot_Manager_Menu(Error_Code: PCARDINAL32): Boot_Manager_Menu; external 'lvm' name 'Get_Boot_Manager_Menu';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Install_Boot_Manager                             */
//*                                                                   */
//*   Descriptive Name: This function installs Boot Manager.  It can  */
//*                     be used to replace an existing Boot Manager   */
//*                     as well.                                      */
//*                                                                   */
//*   Input: CARDINAL32  Drive_Number - The number of the drive to    */
//*                                     install Boot Manager on.  Must*/
//*                                     be 1 or 2.                    */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If this function is successful, then *Error_Code will be*/
//*           0; otherwise it will be > 0.                            */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be set to a*/
//*                   non-zero value.  Depending upon the error, it   */
//*                   is possible that the Boot Manager partition can */
//*                   be left in an unusuable state (such as for a    */
//*                   write error).                                   */
//*                                                                   */
//*   Side Effects: Boot Manager may be installed on drive 1 or 2.    */
//*                 The MBR for drive 1 may be altered.               */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
// Only drives 0 and 1 are acceptable.
procedure Install_Boot_Manager(Drive_Number: CARDINAL32; Error_Code: PCARDINAL32); external 'lvm' name 'Install_Boot_Manager';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Remove_Boot_Manager                              */
//*                                                                   */
//*   Descriptive Name: Removes Boot Manager from the system.         */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if Boot Manager was successfully  */
//*           removed from the system; otherwise *Error_Code will     */
//*           be 0.                                                   */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  Boot Manager will be removed from the system.    */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Remove_Boot_Manager(Error_Code: PCARDINAL32); external 'lvm' name 'Remove_Boot_Manager';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Boot_Manager_Options                         */
//*                                                                   */
//*   Descriptive Name: Sets the Boot Managers Options.  The options  */
//*                     that can be set are: whether or not the time- */
//*                     out timer is active, how long the timer-out   */
//*                     is, the partition to boot by default, and     */
//*                     whether or not Boot Manager should display its*/
//*                     menu using default mode or advanced mode.     */
//*                                                                   */
//*   Input: ADDRESS Handle - The handle of the partition or volume   */
//*                           to boot if the time-out timer is active */
//*                           and the time-out value is reached.      */
//*          BOOLEAN Timer_Active - If TRUE, then the time-out timer  */
//*                                 is active.                        */
//*          CARDINAL32 Time_Out_Value - If the time-out timer is     */
//*                                      active, this is the time-out */
//*                                      value, in seconds.           */
//*          BOOLEAN Advanced_Mode - If TRUE, then Boot Manager will  */
//*                                  operate in advanced mode.  If    */
//*                                  FALSE, then normal mode will be  */
//*                                  in effect.                       */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if no errors occur.  If an error  */
//*           does occur, then *Error_Code will be > 0.               */
//*                                                                   */
//*   Error Handling: If an error occurs, no changes will be made to  */
//*                   Boot Manager and *Error_Code will be set a      */
//*                   non-zero error code.                            */
//*                                                                   */
//*   Side Effects:  Boot Manager may be modified.                    */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Boot_Manager_Options(Handle: ADDRESS;
Timer_Active: BOOLEAN;
Time_Out_Value: CARDINAL32;
Advanced_Mode: BOOLEAN;
Error_Code: PCARDINAL32
); external 'lvm' name 'Set_Boot_Manager_Options';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Boot_Manager_Options                         */
//*                                                                   */
//*   Descriptive Name: This function returns the current Boot Manager*/
//*                     settings for the various Boot Manager options.*/
//*                                                                   */
//*   Input: ADDRESS * Handle - The handle for the default boot volume*/
//*                             or partition.                         */
//*          BOOLEAN * Handle_Is_Volume - If TRUE, then Handle        */
//*                                       represents a volume.  If    */
//*                                       FALSE, then Handle          */
//*                                       represents a partition.     */
//*          BOOLEAN * Timer_Active - If TRUE, then the time-out timer*/
//*                                   is active.  If FALSE, then the  */
//*                                   time-out timer is not active.   */
//*          CARDINAL32 * Time_Out_Value - If the time-out timer is   */
//*                                        active, then this is the   */
//*                                        number of seconds that Boot*/
//*                                        Manager will wait for user */
//*                                        input before booting the   */
//*                                        default volume/partition.  */
//*          BOOLEAN * Advanced_Mode - If TRUE, the Boot Manager is   */
//*                                    operating in advanced mode.  If*/
//*                                    FALSE, then Boot Manager is    */
//*                                    operating in normal mode.      */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Handle, *Handle_Is_Volume, *Timer_Active,              */
//*           *Time_out_value, *Advanced_Mode, and *Error_Code are all*/
//*           set by this function.  If there are no errors, then     */
//*           *Error_Code will be set to 0.  If any errors occur, then*/
//*           *Error_Code will be > 0.                                */
//*                                                                   */
//*   Error Handling: If any of the parameters are invalid, then a    */
//*                   trap is likely.  If Boot Manager is not         */
//*                   installed, then *Error_Code will be > 0.        */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Get_Boot_Manager_Options(Handle: PADDRESS;
Handle_Is_Volume: PBOOLEAN;
Timer_Active: PBOOLEAN;
Time_Out_Value: PCARDINAL32;
Advanced_Mode: PBOOLEAN;
Error_Code: PCARDINAL32
); external 'lvm' name 'Get_Boot_Manager_Options';

//****************************************************************************
//
// Other Functions
//
//***************************************************************************

//*********************************************************************/
//*                                                                   */
//*   Function Name:  Allocate_Engine_Memory                          */
//*                                                                   */
//*   Descriptive Name:  Allocates a block of memory using LVM.DLL's  */
//*                      memory management functions.                 */
//*                                                                   */
//*   Input: CARDINAL32 Size - The number of bytes of memory to       */
//*                            allocate.                              */
//*                                                                   */
//*   Output: The address of the block of memory which was allocated, */
//*           or NULL if the requested amount of memory could not be  */
//*           allocated.                                              */
//*                                                                   */
//*   Error Handling: None.                                           */
//*                                                                   */
//*   Side Effects:  The specified number of bytes is allocated from  */
//*                  the memory manager imbedded in LVM.DLL.  Memory  */
//*                  allocated by this function must be freed using   */
//*                  Free_Engine_Memory function.  The use of any     */
//*                  memory manager to free the memory could result in*/
//*                  Bad Things Happening!                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
{$ifdef lvm2}
ADDRESS _System Allocate_Engine_Memory( CARDINAL32 Size );
{$endif}
//*********************************************************************/
//*                                                                   */
//*   Function Name:  Free_Engine_Memory                              */
//*                                                                   */
//*   Descriptive Name: Frees a memory object created by LVM.DLL and  */
//*                     returned to a user of LVM.DLL.                */
//*                                                                   */
//*   Input: ADDRESS Object : The address of the memory object to     */
//*                           free.  This could be the                */
//*                           Drive_Control_Data field of a           */
//*                           Drive_Control_Record, the               */
//*                           Partition_Array field of a              */
//*                           Partition_Information_Array structure,  */
//*                           or any other dynamically allocated      */
//*                           memory object created by LVM.DLL and    */
//*                           returned by a function in LVM.DLL.      */
//*                                                                   */
//*   Output: None.                                                   */
//*                                                                   */
//*   Error Handling: None.                                           */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  A trap or exception could occur if a bad address is     */
//*           passed into this function.                              */
//*                                                                   */
//*********************************************************************/
procedure Free_Engine_Memory(Object_: ADDRESS); external 'lvm' name 'Free_Engine_Memory';

//*********************************************************************/
//*                                                                   */
//*   Function Name: New_MBR                                          */
//*                                                                   */
//*   Descriptive Name: This function lays down a new MBR on the      */
//*                     specified drive.                              */
//*                                                                   */
//*   Input: ADDRESS Drive_Handle - The handle of the drive on which  */
//*                                 the new MBR is to be placed.      */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if the new MBR was successfully   */
//*           placed on the specified drive.  If the operation failed */
//*           for any reason, then *Error_Code will contain a non-zero*/
//*           error code.                                             */
//*                                                                   */
//*   Error Handling: If an error occurs, then the existing MBR is not*/
//*                   altered and *Error_Code will be > 0.            */
//*                                                                   */
//*   Side Effects:  A new MBR may be placed on the specified drive.  */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure New_MBR(Drive_Handle: ADDRESS; Error_Code: PCARDINAL32); external 'lvm' name 'New_MBR';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Available_Drive_Letters                      */
//*                                                                   */
//*   Descriptive Name: This function returns a bitmap indicating     */
//*                     which drive letters are available for use.    */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: This function returns a bitmap of the available drive   */
//*           letters.  If this function is successful, then          */
//*           *Error_Code will be set to 0.  Otherwise, *Error_Code   */
//*           will be > 0 and the bitmap returned will have all bits  */
//*           set to 0.                                               */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  A drive letter is available if it is not associated     */
//*           with a volume located on a disk drive controlled        */
//*           by OS2DASD.                                             */
//*                                                                   */
//*********************************************************************/
function Get_Available_Drive_Letters(Error_Code: PCARDINAL32): CARDINAL32; external 'lcm' name 'Get_Available_Drive_Letters';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Reserved_Drive_Letters                       */
//*                                                                   */
//*   Descriptive Name: This function returns a bitmap indicating     */
//*                     which drive letters are reserved for use by   */
//*                     devices NOT under the control of LVM.         */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: This function returns a bitmap of the drive letters     */
//*           which are being used by devices which are NOT controlled*/
//*           by LVM.  While a Volume CAN be assigned a drive letter  */
//*           from this list, a reboot will almost always be required */
//*           in order for the assignment to take place.              */
//*           If this function is successful, then *Error_Code will be*/
//*           set to 0.  Otherwise, *Error_Code will be > 0 and the   */
//*           bitmap returned will have all bits set to 0.            */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  Devices which are assigned drive letters but which are  */
//*           NOT under LVM control include:  CD-ROM, Network drives, */
//*           parallel port attached devices, and any DASD devices    */
//*           not controlled by OS2DASD.                              */
//*                                                                   */
//*********************************************************************/
function Get_Reserved_Drive_Letters(Error_Code: CARDINAL32): CARDINAL32; external 'lvm' name 'Get_Reserved_Drive_Letters';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Reboot_Required                                  */
//*                                                                   */
//*   Descriptive Name: This function indicates whether or not any    */
//*                     changes were made to the partitioning of the  */
//*                     disks in the system which would require a     */
//*                     reboot to make functional.                    */
//*                                                                   */
//*   Input: None.                                                    */
//*                                                                   */
//*   Output: The function return value will be TRUE if the system    */
//*           must be rebooted as a result of disk partitioning       */
//*           changes.                                                */
//*                                                                   */
//*   Error Handling: None required.                                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Reboot_Required: BOOLEAN; external 'lvm' name 'Reboot_Required';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Changes_Pending                                  */
//*                                                                   */
//*   Descriptive Name: This function indicates whether or not any    */
//*                     changes were made to the partitioning of the  */
//*                     disks in the system which have not yet been   */
//*                     comitted to disk.                             */
//*                                                                   */
//*   Input: None.                                                    */
//*                                                                   */
//*   Output: The function return value will be TRUE if there are     */
//*           uncomitted changes to the partitioning of one or more of*/
//*           the drives in the system.                               */
//*                                                                   */
//*   Error Handling: None required.                                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Changes_Pending: BOOLEAN; external 'lvm' name 'Changes_Pending';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Reboot_Flag                                  */
//*                                                                   */
//*   Descriptive Name: This function sets the Reboot Flag.  The      */
//*                     Reboot Flag is a special flag on the boot     */
//*                     disk used by the install program to keep      */
//*                     track of whether or not the system was just   */
//*                     rebooted.  It is used by the various phases   */
//*                     of install.                                   */
//*                                                                   */
//*   Input: BOOLEAN Reboot - The new value for the Reboot Flag.  If  */
//*                           TRUE, then the reboot flag will be set. */
//*                           If FALSE, then the reboot flag will be  */
//*                           cleared.                                */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be set to 0 if there are no errors.    */
//*           *Error_Code will be > 0 if an error occurs.             */
//*                                                                   */
//*   Error Handling: If an error occurs, then the value of the Reboot*/
//*                   Flag will be unchanged.                         */
//*                                                                   */
//*   Side Effects:  The value of the Reboot Flag may be changed.     */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Reboot_Flag(Reboot: BOOLEAN; Error_Code: PCARDINAL32); external 'lvm' name 'Set_Reboot_Flag';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Reboot_Flag                                  */
//*                                                                   */
//*   Descriptive Name: This function returns the value of the Reboot */
//*                     Flag.  The Reboot Flag is a special flag on   */
//*                     the boot disk used by the install program to  */
//*                     keep track of whether or not the system was   */
//*                     just rebooted.  It is used by the various     */
//*                     phases of install.                            */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: The function return value will be TRUE if no errors     */
//*           occur and the Reboot Flag is set.  *Error_Code will be  */
//*           0 under these conditions.  If an error occurs, the      */
//*           function return value will be FALSE and *Error_Code     */
//*           will be > 0.                                            */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                   The value of the reboot flag will be unchanged. */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Reboot_Flag(Error_Code: PCARDINAL32): BOOLEAN; external 'lvm' name 'Get_Reboot_Flag';

//*********************************************************************/
//*                                                                   */
//*                                                                   */
//*   Function Name: Set_Install_Flags                                */
//*                                                                   */
//*   Descriptive Name: This function sets the value of the Install   */
//*                     Flags.  The Install Flags reside in a 32 bit  */
//*                     field in the LVM dataspace.  These flags are  */
//*                     not used by LVM, thereby leaving Install free */
//*                     to use them for whatever it wants.            */
//*                                                                   */
//*   Input: CARDINAL32 Install_Flags - The new value for the Install */
//*                                     Flags.                        */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32    */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be set to 0 if there are no errors.    */
//*           *Error_Code will be > 0 if an error occurs.             */
//*                                                                   */
//*   Error Handling: If an error occurs, then the value of the       */
//*                   Install Flags will be unchanged.                */
//*                                                                   */
//*   Side Effects:  The value of the Install Flags may be changed.   */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Install_Flags(Install_Flags: CARDINAL32; Error_Code: PCARDINAL32); external 'lvm' name 'Set_Install_Flags';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_Install_Flags                                */
//*                                                                   */
//*   Descriptive Name: This function returns the value of the Install*/
//*                     Flags.  The Install Flags reside in a 32 bit  */
//*                     field in the LVM dataspace.  These flags are  */
//*                     not used by LVM, thereby leaving Install free */
//*                     to use them for whatever it wants.            */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: The function returns the current value of the Install   */
//*           Flags stored in the LVM Dataspace.                      */
//*           *Error_Code will be LVM_ENGINE_NO_ERROR if the function */
//*           is successful.  If an error occurs, the function will   */
//*           return 0 and *Error_Code will be > 0.                   */
//*                                                                   */
//*   Error Handling: If an error occurs, *Error_Code will be > 0.    */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
function Get_Install_Flags(Error_Code: PCARDINAL32): CARDINAL32; external 'lvm' name 'Get_Install_Flags';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Min_Install_Size                             */
//*                                                                   */
//*   Descriptive Name: This function tells the LVM Engine how big a  */
//*                     partition/volume must be in order for it to   */
//*                     marked installable.  If this function is not  */
//*                     used to set the minimum size for an           */
//*                     installable partition/volume, the LVM Engine  */
//*                     will use a default value of 300 MB.           */
//*                                                                   */
//*   Input: CARDINAL32 Min_Sectors - The minimum size, in sectors,   */
//*                                   that a partition must be in     */
//*                                   order for it to be marked as    */
//*                                   installable.                    */
//*                                                                   */
//*   Output: None.                                                   */
//*                                                                   */
//*   Error Handling: None required.                                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Min_Install_Size(Min_Sectors: CARDINAL32); external 'lvm' name 'Set_Min_Install_Size';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Set_Free_Space_Threshold                         */
//*                                                                   */
//*   Descriptive Name: This function tells the LVM Engine not to     */
//*                     report blocks of free space which are less    */
//*                     than the size specified.  The engine defaults */
//*                     to not reporting blocks of free space which   */
//*                     are smaller than 2048 sectors (1 MB).         */
//*                                                                   */
//*   Input: CARDINAL32 Min_Sectors - The minimum size, in sectors,   */
//*                                   that a block of free space must */
//*                                   be in order for the LVM engine  */
//*                                   to report it.                   */
//*                                                                   */
//*   Output: None.                                                   */
//*                                                                   */
//*   Error Handling: None required.                                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Set_Free_Space_Threshold(Min_Sectors: CARDINAL32); external 'lvm' name 'Set_Free_Space_Threshold';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Read_Sectors                                     */
//*                                                                   */
//*   Descriptive Name: This function reads one or more sectors from  */
//*                     the specified drive and places the data read  */
//*                     in Buffer.                                    */
//*                                                                   */
//*   Input: CARDINAL32 Drive_Number : The number of the hard drive to*/
//*                                    read from.  The drives in the  */
//*                                    system are numbered from 1 to  */
//*                                    n, where n is the total number */
//*                                    of hard drives in the system.  */
//*          LBA Starting_Sector : The first sector to read from.     */
//*          CARDINAL32 Sectors_To_Read : The number of sectors to    */
//*                                       read into memory.           */
//*          ADDRESS Buffer : The location to put the data read into. */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return code.              */
//*                                                                   */
//*   Output: If Successful, then the data read will be placed in     */
//*              memory starting at Buffer, and *Error will be        */
//*              LVM_ENGINE_NO_ERROR.                                 */
//*           If Unsuccessful, then *Error will be > 0 and the        */
//*              contents of memory starting at Buffer is undefined.  */
//*                                                                   */
//*   Error Handling: *Error will be > 0 if an error occurs.          */
//*                                                                   */
//*   Side Effects: Data may be read into memory starting at Buffer.  */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Read_Sectors(Drive_Number: CARDINAL32;
Starting_Sector: LBA;
Sectors_To_Read: CARDINAL32;
Buffer: ADDRESS;
Error: PCARDINAL32); external 'lvm' name 'Read_Sectors';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Write_Sectors                                    */
//*                                                                   */
//*   Descriptive Name: This function writes data from memory to one  */
//*                     or more sectors on the specified drive.       */
//*                                                                   */
//*   Input: CARDINAL32 Drive_Number : The number of the hard drive to*/
//*                                    write to.  The drives in the   */
//*                                    system are numbered from 1 to  */
//*                                    n, where n is the total number */
//*                                    of hard drives in the system.  */
//*          LBA Starting_Sector : The first sector to write to.      */
//*          CARDINAL32 Sectors_To_Read : The number of sectors to    */
//*                                       be written.                 */
//*          ADDRESS Buffer : The location of the data to be written  */
//*                           to disk.                                */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return code.              */
//*                                                                   */
//*   Output: If Successful, then the data at Buffer will be placed   */
//*              on the disk starting at the sector specified, and    */
//*              *Error will be LVM_ENGINE_NO_ERROR.                  */
//*           If Unsuccessful, then *Error will be > 0 and the        */
//*              contents of the disk starting at sector              */
//*              Starting_Sector is undefined.                        */
//*                                                                   */
//*   Error Handling: *Error will be > 0 if an error occurs.          */
//*                                                                   */
//*   Side Effects: Data may be written to disk.                      */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Write_Sectors(Drive_Number: CARDINAL32;
Starting_Sector: LBA;
Sectors_To_Write: CARDINAL32;
Buffer: ADDRESS;
Error: PCARDINAL32); external 'lvm' name 'Write_Sectors';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Rediscover_PRMs                                  */
//*                                                                   */
//*   Descriptive Name: Causes OS2LVM and OS2DASD to check PRMs for   */
//*                     new or changed media.                         */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If the Rediscover PRM operation was successful, then    */
//*           *Error_Code will be LVM_ENGINE_NO_ERROR.  If there      */
//*           was an error, then *Error_Code will be > 0.             */
//*                                                                   */
//*   Error Handling: None.                                           */
//*                                                                   */
//*   Side Effects:  New volumes may be discovered and assigned drive */
//*                  letters by OS2LVM and OS2DASD.                   */
//*                                                                   */
//*   Notes: The LVM Engine must be CLOSED when this function is      */
//*          called as this function is disabled while it is open!    */
//*                                                                   */
//*********************************************************************/
procedure Rediscover_PRMs(Error_Code: PCARDINAL32); external 'lvm' name 'Rediscover_PRMs';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Get_LVM_View                                     */
//*                                                                   */
//*   Descriptive Name:  This function gets the OS2LVM data for the   */
//*                      specified drive letter.  The intent is to    */
//*                      allow the determination of what drive letter */
//*                      a volume really has given the possibilities  */
//*                      of conflict or a drive preference of '*'.    */
//*                                                                   */
//*   Input:  char  IFSM_Drive_Letter : The drive letter for which the*/
//*                                     OS2LVM data is requested.     */
//*           CARDINAL32 * Drive_Number : The address of a variable   */
//*                                       to hold the OS/2 drive      */
//*                                       number of the drive         */
//*                                       containing the first        */
//*                                       partition of the volume     */
//*                                       currently assigned to the   */
//*                                       requested drive letter.     */
//*           CARDINAL32 * Partition_LBA : The address of a variable  */
//*                                        to hold the LBA of the     */
//*                                        first partition of the     */
//*                                        volume currently assigned  */
//*                                        to the requested drive     */
//*                                        letter.                    */
//*           char * LVM_Drive_Letter : The address of a variable to  */
//*                                     hold the drive letter that    */
//*                                     OS2LVM thinks the volume      */
//*                                     assigned to the requested     */
//*                                     drive letter should have.     */
//*           BYTE * UnitID : The address of a variable to hold the   */
//*                           OS2LVM unit ID for the volume associated*/
//*                           with the requested drive letter.        */
//*                                                                   */
//*   Output:  The function return value will be TRUE if the function */
//*            completed successfully.                                */
//*                                                                   */
//*   Error Handling: If this function fails, the specified drive     */
//*                   letter is either not in use, or is in use by a  */
//*                   device not controlled by OS2LVM.                */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes: This function can be used with the LVM Engine open or    */
//*          closed.                                                  */
//*                                                                   */
//*********************************************************************/
function Get_LVM_View(IFSM_Drive_Letter: char;
Drive_Number: PCARDINAL32;
Partition_LBA: PCARDINAL32;
LVM_Drive_Letter: Pchar;
UnitID: PBYTE): BOOLEAN; external 'lvm' name 'Get_LVM_View';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Start_Logging                                    */
//*                                                                   */
//*   Descriptive Name: Enables the LVM Engine logging.  Once enabled,*/
//*                     the LVM Engine logging function will log all  */
//*                     LVM Engine activity to the specified log file.*/
//*                     The data is logged in a binary format for     */
//*                     compactness and speed.                        */
//*                                                                   */
//*   Input: char * Filename - The filename of the file to use as the */
//*                            log file.                              */
//*          CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: If the logging file was successfully created, then      */
//*           *Error_Code will be 0.  If the log file could not be    */
//*           created, then *Error_Code will be > 0.                  */
//*                                                                   */
//*   Error Handling: If the log file can not be created, then        */
//*                   *Error_Code will be > 0.                        */
//*                                                                   */
//*   Side Effects:  A file may be created/opened for logging of      */
//*                  LVM Engine actions.                              */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Start_Logging(Filename: Pchar; Error_Code: PCARDINAL32); external 'lvm' name 'Start_Logging';

//*********************************************************************/
//*                                                                   */
//*   Function Name: Stop_Logging                                     */
//*                                                                   */
//*   Descriptive Name: This function ends LVM Engine logging and     */
//*                     closes the log file.                          */
//*                                                                   */
//*   Input: CARDINAL32 * Error_Code - The address of a CARDINAL32 in */
//*                                    in which to store an error code*/
//*                                    should an error occur.         */
//*                                                                   */
//*   Output: *Error_Code will be 0 if this function completes        */
//*           successfully; otherwise it will be > 0.                 */
//*                                                                   */
//*   Error Handling: If the log file is not currently opened, or if  */
//*                   the close operation fails on the log file, then */
//*                   *Error_Code will be > 0.                        */
//*                                                                   */
//*   Side Effects:  The log file may be closed.                      */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
procedure Stop_Logging(Error_Code: PCARDINAL32); external 'lvm' name 'Stop_Logging';


{$ifdef lists}
(*
* Description:  This module implements a simple, generic, doubly linked list.
*               Data objects of any type can be placed into a linked list
*               created by this module.  Futhermore, data objects of different
*               types may be placed into the same linked list.
*
* Notes:  This linked list implementation makes use of the concept of the
*         current item.  In any non-empty list, one item in the list will
*         be designated as the current item.  When any of the following
*         functions are called, they will operate upon the current item
*         only: GetItem, ReplaceItem, DeleteItem, GetTag, NextItem,
*         PreviousItem, GetObject, ExtractItem, and ExtractObject.  The
*         user of this module may set the current item through the use of
*         the GoToStartOfList, GoToEndOfList, NextItem, PreviousItem,
*         and GoToSpecifiedItem functions.
*
*         Since a linked list created by this module may contain items
*         of different types, the user will need a way to identify items
*         of different types which may be in the same list.  To allow users
*         to do this, the concept of an item tag is used.  When an item is
*         added to the list, the user must enter an item tag.  The item
*         tag is merely some identifier that the user wishes to associate
*         with the item being placed into the list.  When used as intended,
*         each type of data item will have a unique tag associated with it.
*         This way, all data items of the same type will have the same tag
*         while data items of different types will have different tags.
*         Thus, by using the GetTag function, the user can get the item
*         tag for the current item without having to get the item from the
*         list.  This allows the user to differentiate between items of
*         different types which reside in the same list.
*
*         This module is single threaded.  If used in a multi-threaded
*         environment, the user must implement appropriate access controls.
*
*         When an item is inserted or appended to a list, this module
*         allocates memory on the heap to hold the item and then copies
*         the item to the memory that it allocated.  This allows local
*         variables to be safely inserted or appended to a list.  However,
*         it should be noted that under certain circumstances a copy of the
*         entire data item will NOT be made.  Specifically, if the data item
*         is a structure or array containing pointers, then the data pointed
*         to by the pointers will NOT be copied even though the structure or
*         array is!  This results from the fact that, when an item is being
*         inserted or appended to a list, the user provides just an address
*         and size.  This module assumes that the item to inserted or append
*         lies in a contiguous block of memory at the address provided by the
*         user.  This module has no way of knowing the structure of the data
*         at the specified address, and therefore can not know about any
*         embedded pointers which may lie within that block of memory.
*
*         This module now employs the concept of a handle.  A handle is a
*         reference to a specific item in a list which allows that item to
*         be made the current item in the list quickly.  Example:  If you
*         use the GetHandle function to get a handle for the current item
*         (lets call the item B1), then, regardless of where you are in the
*         list (or any reodering of the items in the list), you can make item
*         B1 the current item by passing its handle to the GoToSpecifiedItem
*         function.  Alternatively, you could operate directly on B1 using
*         the other handle based functions, such as GetItem_By_Handle, for
*         example.  GetItem_By_Handle gets the item associated with the
*         specified handle without changing which item in the list is the
*         current item in the list.
*
*         The functions of this module refer to user data as either items or
*         objects.  The difference between the two is simple, yet subtle.  It
*         deals with who is responsible for the memory used to hold the data.
*         In the case of an item, this module is responsible for the memory
*         used to hold the user data.  In the case of an object, the user
*         is responsible for the memory used to hold the data.
*
*         What this means is that, for functions adding ITEMS to a list,
*         this module will be responsible for allocating memory to hold
*         the user data and then copying the user data into the memory
*         that was allocated.  For functions which return items, this
*         module will COPY the user data from the LIST into a buffer
*         specified by the user.  For functions which add objects to a
*         list, the user provides a pointer to a block of memory holding
*         user data.  This block of memory was allocated by the user, and
*         becomes the "property" of this module once it has been added to
*         a LIST.  For functions which return objects, a pointer to the
*         memory where the data is stored is returned.  As long as an item/object
*         is in a LIST, this module will be responsible for the memory that
*         is used to store the data associated with that item.  This means that
*         users of this module should not call free on an object returned by this
*         module as long as that object is still within a list.
*
*
*)

typedef unsigned long TAG;

typedef ADDRESS DLIST;

//*--------------------------------------------------
//* Type definitions
//--------------------------------------------------*/

typedef enum _Insertion_Modes {
InsertAtStart,
InsertBefore,
InsertAfter,
AppendToList
} Insertion_Modes;

//************************************************
//*           Functions Available                *
//************************************************/

//*
//* The parameter *Error is set by every function in this module.  It
//* will be set to 0 to indicate success, and will be > 0 if an
//* error occurs.  The following table lists the possible error codes:
//*     0 : No error.
//*     1 : Out of memory
//*     2 : Memory has been corrupted!
//*     3 : Bad List Record!
//*     4 : List Record not initialized yet!
//*     5 : List is empty!
//*     6 : Item size mismatch!
//*     7 : Bad item pointer!
//*     8 : Item has zero size!
//*     9 : Item tag mismatch!
//*    10 : Already at end of list!
//*    11 : Already at start of list!
//*    12 : Bad Handle!
//*    13 : Invalid Insertion Mode!
//*/
const
  DLIST_SUCCESS                   = 0;
  DLIST_OUT_OF_MEMORY             = 1;
  DLIST_CORRUPTED                 = 2;
  DLIST_BAD                       = 3;
  DLIST_NOT_INITIALIZED           = 4;
  DLIST_EMPTY                     = 5;
  DLIST_ITEM_SIZE_WRONG           = 6;
  DLIST_BAD_ITEM_POINTER          = 7;
  DLIST_ITEM_SIZE_ZERO            = 8;
  DLIST_ITEM_TAG_WRONG            = 9;
  DLIST_END_OF_LIST               =10;
  DLIST_ALREADY_AT_START          =11;
  DLIST_BAD_HANDLE                =12;
  DLIST_INVALID_INSERTION_MODE    =13;

///* The following code is special.  It is for use with the PruneList and ForEachItem functions.  Basically, these functions
//can be thought of as "searching" a list.  They present each item in the list to a user supplied function which can then
//operate on the items.  If the user supplied function returns a non-zero error code, ForEachItem and PruneList abort and
//return an error to the caller.  This may be undesirable.  If the user supplied function used with PruneList and ForEachItem
//returns the code below, PruneList/ForEachItem will abort and return DLIST_SUCCESS.  This allows PruneList and ForEachItem
//to be used to search a list and terminate the search when the desired item is found without having to traverse the
//remaining items in the list.                                                                                                  */
#define DLIST_SEARCH_COMPLETE  0xFF

#ifdef USE_POOLMAN

//*********************************************************************/
//*                                                                   */
//*   Function Name:  CreateList                                      */
//*                                                                   */
//*   Descriptive Name: This function allocates and initializes the   */
//*                     data structures associated with a list and    */
//*                     then returns a pointer to these structures.   */
//*                                                                   */
//*   Input: CARDINAL32 InitialPoolSize - Each List gets a pool of    */
//*                                     link nodes.  When items are   */
//*                                     added to the List, a link node*/
//*                                     is removed from the pool.     */
//*                                     When an item is removed from  */
//*                                     the List, the link node used  */
//*                                     for that item is returned to  */
//*                                     the pool.  InitialPoolSize is */
//*                                     the number of link nodes to   */
//*                                     place in the pool when the    */
//*                                     pool is created.              */
//*          CARDINAL32 MaximumPoolSize - When the pool runs out of   */
//*                                     link nodes, new nodes are     */
//*                                     allocated by the pool.  When  */
//*                                     these links start being       */
//*                                     returned to the pool, the pool*/
//*                                     will grow.  This parameter    */
//*                                     puts a limit on how big the   */
//*                                     pool may grow to.  Once the   */
//*                                     pool reaches this size, any   */
//*                                     link nodes being returned to  */
//*                                     the pool will be deallocated. */
//*          CARDINAL32 PoolIncrement - When the pool runs out of link*/
//*                                   nodes and more are required,    */
//*                                   the pool will allocate one or   */
//*                                   more link nodes.  This tells the*/
//*                                   pool how many link nodes to     */
//*                                   allocate at one time.           */
//*                                                                   */
//*   Output: If Success : The function return value will be non-NULL */
//*                                                                   */
//*           If Failure : The function return value will be NULL.    */
//*                                                                   */
//*   Error Handling:  The function will only fail if it can not      */
//*                    allocate enough memory to create the new list  */
//*                    and its associated pool of link nodes.         */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
DLIST _System CreateList(CARDINAL32 InitialPoolSize,
CARDINAL32 MaximumPoolSize,
CARDINAL32 PoolIncrement);

#else

//*********************************************************************/
//*                                                                   */
//*   Function Name:  CreateList                                      */
//*                                                                   */
//*   Descriptive Name: This function allocates and initializes the   */
//*                     data structures associated with a list and    */
//*                     then returns a pointer to these structures.   */
//*                                                                   */
//*   Input: None.                                                    */
//*                                                                   */
//*   Output: If Success : The function return value will be non-NULL */
//*                                                                   */
//*           If Failure : The function return value will be NULL.    */
//*                                                                   */
//*   Error Handling:  The function will only fail if it can not      */
//*                    allocate enough memory to create the new list. */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  None.                                                   */
//*                                                                   */
//*********************************************************************/
DLIST _System CreateList( void );

#endif

//*********************************************************************/
//*                                                                   */
//*   Function Name: InsertItem                                       */
//*                                                                   */
//*   Descriptive Name:  This function inserts an item into a DLIST.  */
//*                      The item can be placed either before or      */
//*                      after the current item in the DLIST.         */
//*                                                                   */
//*   Input:  DLIST          ListToAddTo : The list to which the      */
//*                                        data item is to be         */
//*                                        added.                     */
//*           CARDINAL32    ItemSize : The size of the data item, in  */
//*                                    bytes.                         */
//*           ADDRESS       ItemLocation : The address of the data    */
//*                                        to append to the list      */
//*           TAG           ItemTag : The item tag to associate with  */
//*                                   item being appended to the list */
//*           ADDRESS TargetHandle : The item in ListToAddTo which    */
//*                                   is used to determine where      */
//*                                   the item being transferred will */
//*                                   be placed.  If this is NULL,    */
//*                                   then the current item in        */
//*                                   ListToAddTo will be used.       */
//*           Insertion_Modes InsertMode : This indicates where,      */
//*                                   relative to the item in         */
//*                                   ListToAddTo specified by        */
//*                                   Target_Handle, the item being   */
//*                                   inserted can be placed.         */
//*           BOOLEAN MakeCurrent : If TRUE, the item being inserted  */
//*                                 into ListToAddTo becomes the      */
//*                                 current item in ListToAddTo.      */
//*           CARDINAL32 *  Error : The address of a variable to hold */
//*                                 the error return code.            */
//*                                                                   */
//*   Output:  If the operation is successful, then *Error will be    */
//*            set to 0 and the function return value will be the     */
//*            handle for the item that was appended to the list.     */
//*            If the operation fails, then *Error will contain an    */
//*            error code and the function return value will be NULL. */
//*                                                                   */
//*   Error Handling: This function will fail under the following     */
//*                   conditions:                                     */
//*                       ListToAddTo does not point to a valid       */
//*                           list                                    */
//*                       ItemSize is 0                               */
//*                       ItemLocation is NULL                        */
//*                       The memory required to hold a copy of the   */
//*                           item can not be allocated.              */
//*                       The memory required to create a LINK NODE   */
//*                           can not be allocated.                   */
//*                       TargetHandle is invalid or is for an item   */
//*                           in another list.                        */
//*                   If this routine fails, an error code is returned*/
//*                   and any memory allocated by this function is    */
//*                   freed.                                          */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes:  The item to add is copied to the heap to                */
//*           avoid possible conflicts with the usage of              */
//*           local variables in functions which process              */
//*           DLISTs.  However, a pointer to a local variable         */
//*           should not be appended to the DLIST.                    */
//*                                                                   */
//*           It is assumed that TargetHandle is valid, or is at least*/
//*           the address of an accessible block of storage.  If      */
//*           TargetHandle is invalid, or is not the address of an    */
//*           accessible block of storage, then a trap or exception   */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*                                                                   */
//*********************************************************************/
ADDRESS _System InsertItem ( DLIST           ListToAddTo,
CARDINAL32      ItemSize,
ADDRESS         ItemLocation,
TAG             ItemTag,
ADDRESS         TargetHandle,
Insertion_Modes Insert_Mode,
BOOLEAN         MakeCurrent,
CARDINAL32 *    Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name: InsertObject                                     */
//*                                                                   */
//*   Descriptive Name:  This function inserts an object into a DLIST.*/
//*                      The object can be inserted before or after   */
//*                      the current item in the list.                */
//*                                                                   */
//*   Input:  DLIST          ListToAddTo : The list to which the      */
//*                                          data object is to be     */
//*                                          inserted.                */
//*           CARDINAL32    ItemSize : The size of the data item, in  */
//*                                    bytes.                         */
//*           ADDRESS       ItemLocation : The address of the data    */
//*                                        to append to the list      */
//*           TAG           ItemTag : The item tag to associate with  */
//*                                   the item being appended to the  */
//*                                   list                            */
//*           ADDRESS TargetHandle : The item in ListToAddTo which    */
//*                                   is used to determine where      */
//*                                   the item being transferred will */
//*                                   be placed.  If this is NULL,    */
//*                                   then the current item in        */
//*                                   ListToAddTo will be used.       */
//*           Insertion_Modes InsertMode : This indicates where,      */
//*                                   relative to the item in         */
//*                                   ListToAddTo specified by        */
//*                                   Target_Handle, the item being   */
//*                                   inserted can be placed.         */
//*           BOOLEAN MakeCurrent : If TRUE, the item being inserted  */
//*                                 into ListToAddTo becomes the      */
//*                                 current item in ListToAddTo.      */
//*           CARDINAL32 *  Error : The address of a variable to hold */
//*                                 the error return code.            */
//*                                                                   */
//*   Output:  If the operation is successful, then *Error will be    */
//*            set to 0 and the function return value will be the     */
//*            handle for the item that was appended to the list.     */
//*            If the operation fails, then *Error will contain an    */
//*            error code and the function return value will be NULL. */
//*                                                                   */
//*   Error Handling: This function will fail under the following     */
//*                   conditions:                                     */
//*                       ListToAddTo does not point to a valid       */
//*                           list                                    */
//*                       ItemSize is 0                               */
//*                       ItemLocation is NULL                        */
//*                       The memory required for a LINK NODE can not */
//*                           be allocated.                           */
//*                       TargetHandle is invalid or is for an item   */
//*                           in another list.                        */
//*                   If this routine fails, an error code is returned*/
//*                   and any memory allocated by this function is    */
//*                   freed.                                          */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes:  The item to insert is NOT copied to the heap.  Instead, */
//*           the location of the item is stored in the list.  This   */
//*           is the major difference between InsertObject and        */
//*           InsertItem.  InsertItem allocates memory on the heap,   */
//*           copies the item to the memory it allocated, and stores  */
//*           the address of the memory it allocated in the list.     */
//*           InsertObject stores the address provided by the user.   */
//*                                                                   */
//*           It is assumed that TargetHandle is valid, or is at least*/
//*           the address of an accessible block of storage.  If      */
//*           TargetHandle is invalid, or is not the address of an    */
//*           accessible block of storage, then a trap or exception   */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*                                                                   */
//*********************************************************************/
ADDRESS _System InsertObject ( DLIST           ListToAddTo,
CARDINAL32      ItemSize,
ADDRESS         ItemLocation,
TAG             ItemTag,
ADDRESS         TargetHandle,
Insertion_Modes Insert_Mode,
BOOLEAN         MakeCurrent,
CARDINAL32 *    Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  DeleteItem                                      */
//*                                                                   */
//*   Descriptive Name:  This function removes the specified item from*/
//*                      the list and optionally frees the memory     */
//*                      associated with it.                          */
//*                                                                   */
//*   Input:  DLIST       ListToDeleteFrom : The list whose current   */
//*                                         item is to be deleted.    */
//*           BOOLEAN    FreeMemory : If TRUE, then the memory        */
//*                                   associated with the current     */
//*                                   item will be freed.  If FALSE   */
//*                                   then the current item will be   */
//*                                   removed from the list but its   */
//*                                   memory will not be freed.       */
//*           ADDRESS Handle : The handle of the item to get.  This   */
//*                            handle must be of an item which resides*/
//*                            in ListToDeleteFrom, or NULL.  If      */
//*                            NULL is used, then the current item    */
//*                            in ListToDeleteFrom will be deleted.   */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                 the error return code.            */
//*                                                                   */
//*   Output:  If the operation is successful, then *Error will be    */
//*            set to 0.  If the operation fails, then *Error will    */
//*            contain an error code.                                 */
//*                                                                   */
//*   Error Handling: This function will fail if ListToDeleteFrom is  */
//*                   not a valid list, or if ListToDeleteFrom is     */
//*                   empty, or if Handle is invalid.                 */
//*                   If this routine fails, an error code is returned*/
//*                   in *Error.                                      */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  Items in a list can be accessed in two ways:  A copy of */
//*           the item can be obtained using GetItem and its related  */
//*           calls, or a pointer to the item can be obtained using   */
//*           GetObject and its related calls.  If you have a copy of */
//*           the data and wish to remove the item from the list, set */
//*           FreeMemory to TRUE.  This will remove the item from the */
//*           list and deallocate the memory used to hold it.  If you */
//*           have a pointer to the item in the list (from one of the */
//*           GetObject style functions) and wish to remove the item  */
//*           from the list, set FreeMemory to FALSE.  This removes   */
//*           the item from the list without freeing its memory, so   */
//*           that the pointer obtained with the GetObject style      */
//*           functions is still useable.                             */
//*                                                                   */
//*           It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is considered a valid     */
//*                 handle which refers to the current item in        */
//*                 ListToDeleteFrom.                                 */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list, unless the handle specified belongs   */
//*           to the current item in the list, in which case the      */
//*           item following the current item becomes the current     */
//*           item in the list.  If there is no item following the    */
//*           current item in the list, then the item preceeding the  */
//*           current item will become the current item in the list.  */
//*                                                                   */
//*********************************************************************/
void _System DeleteItem (DLIST        ListToDeleteFrom,
BOOLEAN      FreeMemory,
ADDRESS      Handle,
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  DeleteAllItems                                  */
//*                                                                   */
//*   Descriptive Name:  This function deletes all of the items in the*/
//*                      specified list and optionally frees the      */
//*                      memory associated with each item deleted.    */
//*                                                                   */
//*   Input:  DLIST       ListToDeleteFrom : The list whose items     */
//*                                          are to be deleted.       */
//*           BOOLEAN    FreeMemory : If TRUE, then the memory        */
//*                                   associated with each item in the*/
//*                                   list will be freed.  If FALSE   */
//*                                   then the each item will be      */
//*                                   removed from the list but its   */
//*                                   memory will not be freed.       */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                 the error return code.            */
//*                                                                   */
//*   Output:  If the operation is successful, then *Error will be    */
//*            set to 0.  If the operation fails, then *Error will    */
//*            contain an error code.                                 */
//*                                                                   */
//*   Error Handling: This function will fail if ListToDeleteFrom is  */
//*                   not a valid list, or if ListToDeleteFrom is     */
//*                   empty.                                          */
//*                   If this routine fails, an error code is returned*/
//*                   in *Error.                                      */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  Items in a list can be accessed in two ways:  A copy of */
//*           the item can be obtained using GetItem and its related  */
//*           calls, or a pointer to the item can be obtained using   */
//*           GetObject and its related calls.  If you have a copy of */
//*           the data and wish to remove the item from the list, set */
//*           FreeMemory to TRUE.  This will remove the item from the */
//*           list and deallocate the memory used to hold it.  If you */
//*           have a pointer to the item in the list (from one of the */
//*           GetObject style functions) and wish to remove the item  */
//*           from the list, set FreeMemory to FALSE.  This removes   */
//*           the item from the list without freeing its memory, so   */
//*           that the pointer obtained with the GetObject style      */
//*           functions is still useable.                             */
//*                                                                   */
//*           It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System DeleteAllItems (DLIST        ListToDeleteFrom,
BOOLEAN      FreeMemory,
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetItem                                         */
//*                                                                   */
//*   Descriptive Name:  This function copies the specified item in   */
//*                      the list to a buffer provided by the caller. */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                               the current item is.                */
//*           ADDRESS     ItemLocation : This is the location of the  */
//*                                      buffer into which the current*/
//*                                      item is to be copied.        */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           ADDRESS Handle : The handle of the item to get.  This   */
//*                            handle must be of an item which resides*/
//*                            in ListToGetItemFrom, or NULL.  If     */
//*                            NULL, then the current item in the list*/
//*                            will be used.                          */
//*           BOOLEAN MakeCurrent : If TRUE, the item to get will     */
//*                                 become the current item in the    */
//*                                 list.                             */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                              the error return code.               */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The buffer at ItemLocation will contain a copy of */
//*                    the current item from ListToGetItemFrom.       */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                                                                   */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemLocation is NULL                      */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         Handle is invalid, or is for an item      */
//*                             which is not in ListToGetItemFrom     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is considered a valid     */
//*                 handle corresponding to the current item in the   */
//*                 list.                                             */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list.                                       */
//*                                                                   */
//*********************************************************************/
void _System GetItem( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
ADDRESS        ItemLocation,
TAG            ItemTag,
ADDRESS        Handle,
BOOLEAN        MakeCurrent,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetNextItem                                     */
//*                                                                   */
//*   Descriptive Name:  This function advances the current item      */
//*                      pointer and then copies the current item in  */
//*                      the list to a buffer provided by the caller. */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                                 the current item is.              */
//*           ADDRESS     ItemLocation : This is the location of the  */
//*                                      buffer into which the current*/
//*                                      item is to be copied.        */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The buffer at ItemLocation will contain a copy of */
//*                    the current item from ListToGetItemFrom.       */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                 The current item pointer will NOT be advanced.    */
//*                     The current item in the list will be the same */
//*                     as before the call to this function.          */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemLocation is NULL                      */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         The current item in the list before this  */
//*                             function is called is the last item   */
//*                             item in the list.                     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System GetNextItem( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
ADDRESS        ItemLocation,
TAG            ItemTag,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetPreviousItem                                 */
//*                                                                   */
//*   Descriptive Name:  This function makes the previous item in the */
//*                      list the current item in the list and then   */
//*                      copies that item to a buffer provided by the */
//*                      user.                                        */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                                 the current item is.              */
//*           ADDRESS    ItemLocation : This is the location of the   */
//*                                      buffer into which the current*/
//*                                      item is to be copied.        */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The buffer at ItemLocation will contain a copy of */
//*                    the current item from ListToGetItemFrom.       */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                 The current item pointer will NOT be advanced.    */
//*                     The current item in the list will be the same */
//*                     as before the call to this function.          */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemLocation is NULL                      */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         The current item in the list before this  */
//*                             function is called is the last item   */
//*                             item in the list.                     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System GetPreviousItem( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
ADDRESS        ItemLocation,
TAG            ItemTag,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetObject                                       */
//*                                                                   */
//*   Descriptive Name:  This function returns the address of the data*/
//*                      associated with the specified item in the    */
//*                      list.                                        */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to have its address       */
//*                                      returned to the caller.      */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                               the current item is.                */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           ADDRESS Handle : The handle of the item to get.  This   */
//*                            handle must be of an item which resides*/
//*                            in ListToGetItemFrom, or NULL.  If     */
//*                            NULL, then the current item in the list*/
//*           BOOLEAN MakeCurrent : If TRUE, the item to get will     */
//*                                 become the current item in the    */
//*                                 list.                             */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The function return value will be the address of  */
//*                 the data associated with the current item in the  */
//*                 list.                                             */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                 The function return value will be NULL.           */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         Handle is invalid, or is for an item      */
//*                             which is not in ListToGetItemFrom     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The user should not free the memory associated with     */
//*           the address returned by this function as the object is  */
//*           still in the list.                                      */
//*                                                                   */
//*           It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap may   */
//*           occur.                                                  */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is considered a valid     */
//*                 handle designating the current item in the list.  */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list.                                       */
//*                                                                   */
//*********************************************************************/
ADDRESS _System GetObject( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
TAG            ItemTag,
ADDRESS        Handle,
BOOLEAN        MakeCurrent,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetNextObject                                   */
//*                                                                   */
//*   Descriptive Name:  This function advances the current item      */
//*                      pointer and then returns the address of the  */
//*                      data associated with the current item in the */
//*                      list.                                        */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                                 the current item is.              */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The function return value will be the address of  */
//*                 the data associated with the current item in the  */
//*                 list.                                             */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                 The function return value will be NULL.           */
//*                 The current item pointer will NOT be advanced.    */
//*                     The current item in the list will be the same */
//*                     as before the call to this function.          */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         The current item in the list before this  */
//*                             function is called is the last item   */
//*                             item in the list.                     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The user should not free the memory associated with     */
//*           the address returned by this function as the object is  */
//*           still in the list.                                      */
//*                                                                   */
//*           It is assumed that Error contains a valid address. If   */
//*           this assumption are violated, an exception or trap may  */
//*           occur.                                                  */
//*                                                                   */
//*********************************************************************/
ADDRESS _System GetNextObject( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
TAG            ItemTag,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetPreviousObject                               */
//*                                                                   */
//*   Descriptive Name:  This function makes the previous item in the */
//*                      list the current item and then returns the   */
//*                      address of the data associated with the      */
//*                      current item in the list.                    */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                                 the current item is.              */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The function return value will be the address of  */
//*                 the data associated with the current item in the  */
//*                 list.                                             */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                 The function return value will be NULL.           */
//*                 The current item pointer will NOT be advanced.    */
//*                     The current item in the list will be the same */
//*                     as before the call to this function.          */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         The current item in the list before this  */
//*                             function is called is the last item   */
//*                             item in the list.                     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The user should not free the memory associated with     */
//*           the address returned by this function as the object is  */
//*           still in the list.                                      */
//*                                                                   */
//*           It is assumed that Error contains a valid address. If   */
//*           this assumption are violated, an exception or trap may  */
//*           occur.                                                  */
//*                                                                   */
//*********************************************************************/
ADDRESS _System GetPreviousObject( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
TAG            ItemTag,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  ExtractItem                                     */
//*                                                                   */
//*   Descriptive Name:  This function copies the specified item in   */
//*                      the list to a buffer provided by the caller  */
//*                      and removes the item from the list.          */
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                                 the current item is.              */
//*           ADDRESS     ItemLocation : This is the location of the  */
//*                                      buffer into which the current*/
//*                                      item is to be copied.        */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           ADDRESS Handle : The handle of the item to get.  This   */
//*                            handle must be of an item which resides*/
//*                            in ListToGetItemFrom, or NULL.  If     */
//*                            NULL, then the current item in the list*/
//*                            will be used.                          */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The buffer at ItemLocation will contain a copy of */
//*                    the current item from ListToGetItemFrom.       */
//*                 The item will have been removed from the list and */
//*                    its memory deallocated.                        */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemLocation is NULL                      */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         Handle is invalid, or is for an item      */
//*                             which is not in ListToGetItemFrom     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is considered a valid     */
//*                 handle which refers to the current item in the    */
//*                 list.                                             */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list, unless the handle specified belongs   */
//*           to the current item in the list, in which case the      */
//*           item following the current item becomes the current     */
//*           item in the list.  If there is no item following the    */
//*           current item in the list, then the item preceeding the  */
//*           current item will become the current item in the list.  */
//*                                                                   */
//*********************************************************************/
void _System ExtractItem( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
ADDRESS        ItemLocation,
TAG            ItemTag,
ADDRESS        Handle,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  ExtractObject                                   */
//*                                                                   */
//*   Descriptive Name:  This function returns the address of the data*/
//*                      associated with the specified item in the    */
//*                      list and then removes that item from the list*/
//*                                                                   */
//*   Input:  DLIST   ListToGetItemFrom : The list whose current item */
//*                                      is to be copied and returned */
//*                                      to the caller.               */
//*           CARDINAL32 ItemSize : What the caller thinks the size of*/
//*                                 the current item is.              */
//*           TAG     ItemTag : What the caller thinks the item tag   */
//*                             of the current item is.               */
//*           ADDRESS Handle : The handle of the item to get.  This   */
//*                            handle must be of an item which resides*/
//*                            in ListToGetItemFrom, or NULL.  If     */
//*                            NULL, then the current item in the     */
//*                            list will be used.                     */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code.             */
//*                                                                   */
//*   Output:  If Successful :                                        */
//*                 *Error will be set to 0.                          */
//*                 The function return value will be the address of  */
//*                 the data associated with the current item in the  */
//*                 list.                                             */
//*                 The current item is removed from the list.        */
//*            If Failure :                                           */
//*                 *Error will contain an error code.                */
//*                 The function return value will be NULL.           */
//*                                                                   */
//*   Error Handling: This function will fail under any of the        */
//*                   following conditions:                           */
//*                         ListToGetItemFrom is not a valid list     */
//*                         ItemSize does not match the size of the   */
//*                             current item in the list              */
//*                         ItemTag does not match the item tag       */
//*                             of the current item in the list       */
//*                         Handle is invalid, or is for an item      */
//*                             which is not in ListToGetItemFrom     */
//*                   If any of these conditions occur, *Error will   */
//*                   contain a non-zero error code.                  */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The user is responsible for the memory associated with  */
//*           the address returned by this function since this        */
//*           function removes that object from the list.  This means */
//*           that, when the user is through with the object, they    */
//*           should free it.                                         */
//*                                                                   */
//*           It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap may   */
//*           occur.                                                  */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is considered a valid     */
//*                 handle which refers to the current item in the    */
//*                 list.                                             */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list, unless the handle specified belongs   */
//*           to the current item in the list, in which case the      */
//*           item following the current item becomes the current     */
//*           item in the list.  If there is no item following the    */
//*           current item in the list, then the item preceeding the  */
//*           current item will become the current item in the list.  */
//*                                                                   */
//*********************************************************************/
ADDRESS _System ExtractObject( DLIST          ListToGetItemFrom,
CARDINAL32     ItemSize,
TAG            ItemTag,
ADDRESS        Handle,
CARDINAL32 *   Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  ReplaceItem                                     */
//*                                                                   */
//*   Descriptive Name:  This function replaces the specified item in */
//*                      the list with the one provided as its        */
//*                      argument.                                    */
//*                                                                   */
//*   Input: DLIST   ListToReplaceItemIn : The list whose current item*/
//*                                       is to be replaced           */
//*          CARDINAL32 ItemSize : The size, in bytes, of the         */
//*                              replacement item                     */
//*          ADDRESS     ItemLocation : The address of the replacement*/
//*                                     item                          */
//*          TAG     ItemTag : The item tag that the user wishes to   */
//*                            associate with the replacement item    */
//*          ADDRESS Handle : The handle of the item to get.  This    */
//*                           handle must be of an item which resides */
//*                           in ListToGetItemFrom, or NULL.  If NULL */
//*                           then the current item in the list will  */
//*                           used.                                   */
//*          BOOLEAN MakeCurrent : If TRUE, the item to get will      */
//*                                become the current item in the     */
//*                                list.                              */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return code               */
//*                                                                   */
//*   Output:  If Successful then *Error will be set to 0.            */
//*            If Unsuccessful, then *Error will be set to a non-zero */
//*              error code.                                          */
//*                                                                   */
//*   Error Handling:  This function will fail under the following    */
//*                    conditions:                                    */
//*                         ListToReplaceItemIn is empty              */
//*                         ItemSize is 0                             */
//*                         ItemLocation is NULL                      */
//*                         The memory required can not be allocated. */
//*                         Handle is invalid, or is for an item      */
//*                             which is not in ListToGetItemFrom     */
//*                    If any of these conditions occurs, *Error      */
//*                    will contain a non-zero error code.            */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is a valid handle which   */
//*                 refers to the current item in the list.           */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list.                                       */
//*                                                                   */
//*********************************************************************/
void _System ReplaceItem( DLIST         ListToReplaceItemIn,
CARDINAL32    ItemSize,
ADDRESS       ItemLocation,
TAG           ItemTag,
ADDRESS       Handle,
BOOLEAN       MakeCurrent,
CARDINAL32 *  Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name: ReplaceObject                                    */
//*                                                                   */
//*   Descriptive Name:  This function replaces the specified object  */
//*                      in the list with the one provided as its     */
//*                      argument.                                    */
//*                                                                   */
//*   Input: DLIST   ListToReplaceItemIn : The list whose current     */
//*                                       object is to be replaced    */
//*          CARDINAL32 ItemSize : The size, in bytes, of the         */
//*                              replacement object                   */
//*          ADDRESS     ItemLocation : The address of the replacement*/
//*                                     item                          */
//*          TAG     ItemTag : The item tag that the user wishes to   */
//*                            associate with the replacement item    */
//*          ADDRESS Handle : The handle of the item to get.  This    */
//*                           handle must be of an item which resides */
//*                           in ListToGetItemFrom, or NULL.  If NULL */
//*                           then the current item in the list will  */
//*                           be used.                                */
//*          BOOLEAN MakeCurrent : If TRUE, the item to get will      */
//*                                become the current item in the     */
//*                                list.                              */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return code               */
//*                                                                   */
//*   Output:  If Successful then *Error will be set to 0 and the     */
//*              return value of the function will be the address     */
//*              of the object that was replaced.                     */
//*            If Unsuccessful, then *Error will be set to a non-zero */
//*              error code and the function return value will be     */
//*              NULL.                                                */
//*                                                                   */
//*   Error Handling:  This function will fail under the following    */
//*                    conditions:                                    */
//*                         ListToReplaceItemIn is empty              */
//*                         ItemSize is 0                             */
//*                         ItemLocation is NULL                      */
//*                         The memory required can not be allocated. */
//*                         Handle is invalid, or is for an item      */
//*                             which is not in ListToGetItemFrom     */
//*                    If any of these conditions occurs, *Error      */
//*                    will contain a non-zero error code.            */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  The user is responsible for the memory associated with  */
//*           the object returned by this function as that object is  */
//*           removed from the list.  This means that, when the user  */
//*           is through with the object returned by this function,   */
//*           they should free it.                                    */
//*                                                                   */
//*           It is assumed that Error contains a valid address. It   */
//*           is also assumed that if ItemLocation is not NULL, then  */
//*           it is a valid address that can be dereferenced.  If     */
//*           these assumptions are violated, an exception or trap    */
//*           may occur.                                              */
//*                                                                   */
//*           It is assumed that Handle is valid, or is at least the  */
//*           address of an accessible block of storage.  If Handle   */
//*           is invalid, or is not the address of an accessible block*/
//*           of storage, then a trap or exception may occur.         */
//*           NOTE: For this function, NULL is a valid handle for the */
//*                 current item in the list.                         */
//*                                                                   */
//*           This function does not alter which item is the current  */
//*           item in the list.                                       */
//*                                                                   */
//*********************************************************************/
ADDRESS _System ReplaceObject( DLIST         ListToReplaceItemIn,
CARDINAL32 *  ItemSize,             /* On input - size of new object.  On return = size of old object. */
ADDRESS       ItemLocation,
TAG        *  ItemTag,              /* On input - TAG of new object.  On return = TAG of old object. */
ADDRESS       Handle,
BOOLEAN       MakeCurrent,
CARDINAL32 *  Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetTag                                          */
//*                                                                   */
//*   Descriptive Name:  This function returns the item tag associated*/
//*                      with the current item in the list.           */
//*                                                                   */
//*   Input:  DLIST   ListToGetTagFrom : The list from which the item */
//*                                     tag of the current item is to */
//*                                     be returned                   */
//*           ADDRESS Handle : The handle of the item whose TAG and   */
//*                            size we are to get.  This handle must  */
//*                            be of an item which resides in         */
//*                            in ListToGetTagFrom, or NULL.  If NULL */
//*                            then the current item in the list will */
//*                            be used.                               */
//*           CARDINAL32 * ItemSize : The size, in bytes, of the      */
//*                                   current item in the list.       */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, the function returns the item tag & size*/
//*               associated with the current item in ListToGetTagFrom*/
//*               and *Error is set to 0.                             */
//*            If unsuccessful, the function returns 0 and *Error is  */
//*               set to a non-zero error code.                       */
//*                                                                   */
//*   Error Handling: This function will fail if ListToGetTagFrom is  */
//*                   not a valid list or is an empty list.  In either*/
//*                   of these cases, *Error is set to a non-zero     */
//*                   error code.                                     */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*                                                                   */
//*********************************************************************/
TAG _System GetTag( DLIST  ListToGetTagFrom,
ADDRESS Handle,
CARDINAL32 * ItemSize,
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetHandle                                       */
//*                                                                   */
//*   Descriptive Name:  This function returns a handle for the       */
//*                      current item in the list.  This handle is    */
//*                      then associated with that item regardless of */
//*                      its position in the list.  This handle can be*/
//*                      used to make its associated item the current */
//*                      item in the list.                            */
//*                                                                   */
//*   Input:  DLIST   ListToGetHandleFrom : The list from which a     */
//*                                        handle is needed.          */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, the function returns a handle for the   */
//*               the current item in ListToGetHandleFrom, and *Error */
//*               is set to 0.                                        */
//*            If unsuccessful, the function returns 0 and *Error is  */
//*               set to a non-zero error code.                       */
//*                                                                   */
//*   Error Handling: This function will fail if ListToGetHandleFrom  */
//*                   is not a valid list or is an empty list.  In    */
//*                   either of these cases, *Error is set to a       */
//*                   non-zero error code.                            */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*           The handle returned is a pointer to an internal         */
//*           structure within the list.  If the item associated      */
//*           with this handle is removed from the list, the handle   */
//*           will be invalid and should not be used as the internal  */
//*           structure it points to will nolonger exist!             */
//*                                                                   */
//*********************************************************************/
ADDRESS _System GetHandle ( DLIST ListToGetHandleFrom, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  GetListSize                                     */
//*                                                                   */
//*   Descriptive Name:  This function returns the number of items in */
//*                      a list.                                      */
//*                                                                   */
//*   Input:  DLIST   ListToGetSizeOf : The list whose size we wish to*/
//*                                    know                           */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, the function returns the a count of the */
//*               number of items in the list, and *Error is set to 0.*/
//*            If unsuccessful, the function returns 0 and *Error is  */
//*               set to a non-zero error code.                       */
//*                                                                   */
//*   Error Handling: This function will fail if ListToGetSizeOf is   */
//*                   not a valid list.  If this happens, then *Error */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
CARDINAL32 _System GetListSize( DLIST ListToGetSizeOf, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  ListEmpty                                       */
//*                                                                   */
//*   Descriptive Name:  This function returns TRUE if the            */
//*                      specified list is empty, otherwise it returns*/
//*                      FALSE.                                       */
//*                                                                   */
//*   Input:  DLIST       ListToCheck : The list to check to see if it*/
//*                                    is empty                       */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, the function returns TRUE if the        */
//*               number of items in the list is 0, otherwise it      */
//*               returns FALSE.  Also, *Error is set to 0.           */
//*            If unsuccessful, the function returns TRUE and         */
//*               *Error is set to a non-zero error code.             */
//*                                                                   */
//*   Error Handling: This function will fail if ListToCheck is not   */
//*                   a valid list.  If this happens, then *Error     */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
BOOLEAN _System ListEmpty( DLIST ListToCheck, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  AtEndOfList                                     */
//*                                                                   */
//*   Descriptive Name:  This function returns TRUE if the            */
//*                      current item in the list is the last item    */
//*                      in the list.  Returns FALSE otherwise.       */
//*                                                                   */
//*   Input:  DLIST       ListToCheck : The list to check.            */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, the function returns TRUE if the        */
//*               current item in the list is the last item in the    */
//*               list.  If it is not the last item in the list,      */
//*               FALSE is returned.  *Error_Code is set to           */
//*               DLIST_SUCCESS.                                      */
//*            If unsuccessful, the function returns FALSE and        */
//*               *Error is set to a non-zero error code.             */
//*                                                                   */
//*   Error Handling: This function will fail if ListToCheck is not   */
//*                   a valid list.  If this happens, then *Error     */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
BOOLEAN _System AtEndOfList( DLIST ListToCheck, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  AtStartOfList                                   */
//*                                                                   */
//*   Descriptive Name:  This function returns TRUE if the            */
//*                      current item in the list is the first item   */
//*                      in the list.  Returns FALSE otherwise.       */
//*                                                                   */
//*   Input:  DLIST       ListToCheck : The list to check.            */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, the function returns TRUE if the        */
//*               current item in the list is the first item in the   */
//*               list.  If it is not the first item in the list,     */
//*               FALSE is returned.  *Error_Code is set to           */
//*               DLIST_SUCCESS.                                      */
//*            If unsuccessful, the function returns FALSE and        */
//*               *Error is set to a non-zero error code.             */
//*                                                                   */
//*   Error Handling: This function will fail if ListToCheck is not   */
//*                   a valid list.  If this happens, then *Error     */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
BOOLEAN _System AtStartOfList( DLIST ListToCheck, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  DestroyList                                     */
//*                                                                   */
//*   Descriptive Name:  This function releases the memory associated */
//*                      with the internal data structures of a DLIST.*/
//*                      Once a DLIST has been eliminated by this     */
//*                      function, it must be reinitialized before it */
//*                      can be used again.                           */
//*                                                                   */
//*   Input:  DLIST       ListToDestroy : The list to be eliminated   */
//*                                      from memory.                 */
//*           BOOLEAN FreeItemMemory : If TRUE, all items in the list */
//*                                    will be freed.  If FALSE, all  */
//*                                    items in the list are not      */
//*                                    freed, only the list structures*/
//*                                    associated with them are.      */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, *Error will be set to 0.                */
//*            If unsuccessful, *Error will be set to a non-zero error*/
//*               code.                                               */
//*                                                                   */
//*   Error Handling: This function will fail if ListToDestroy is not */
//*                   a valid list.  If this happens, then *Error     */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*           If FreeItemMemory is TRUE, then this function will try  */
//*           to delete any items which may be in the list.  However, */
//*           since this function has no way of knowing the internal  */
//*           structure of an item, items which contain embedded      */
//*           pointers will not be entirely freed.  This can lead to  */
//*           memory leaks.  The programmer should ensure that any    */
//*           list passed to this function when the FreeItemMemory    */
//*           parameter is TRUE is empty or does not contain any      */
//*           items with embedded pointers.                           */
//*                                                                   */
//*********************************************************************/
void _System DestroyList( DLIST *  ListToDestroy, BOOLEAN FreeItemMemory, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  NextItem                                        */
//*                                                                   */
//*   Descriptive Name:  This function makes the next item in the list*/
//*                      the current item in the list (i.e. it        */
//*                      advances the current item pointer).          */
//*                                                                   */
//*   Input:  DLIST       ListToAdvance : The list whose current item */
//*                                      pointer is to be advanced    */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, *Error will be set to 0.                */
//*            If unsuccessful, *Error will be set to a non-zero error*/
//*               code.                                               */
//*                                                                   */
//*   Error Handling: This function will fail under the following     */
//*                   conditions:                                     */
//*                        ListToAdvance is not a valid list          */
//*                        ListToAdvance is empty                     */
//*                        The current item is the last item in the   */
//*                           list                                    */
//*                   If any of these conditions occurs, then *Error  */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System NextItem( DLIST  ListToAdvance, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  PreviousItem                                    */
//*                                                                   */
//*   Descriptive Name:  This function makes the previous item in the */
//*                      list the current item in the list.           */
//*                                                                   */
//*   Input:  DLIST       ListToChange : The list whose current item  */
//*                                      pointer is to be changed     */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, *Error will be set to 0.                */
//*            If unsuccessful, *Error will be set to a non-zero error*/
//*               code.                                               */
//*                                                                   */
//*   Error Handling: This function will fail under the following     */
//*                   conditions:                                     */
//*                        ListToChange is not a valid list           */
//*                        ListToChange is empty                      */
//*                        The current item is the first item in the  */
//*                           list                                    */
//*                   If any of these conditions occurs, then *Error  */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System PreviousItem( DLIST  ListToChange, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name: GoToStartOfList                                  */
//*                                                                   */
//*   Descriptive Name:  This function makes the first item in the    */
//*                      list the current item in the list.           */
//*                                                                   */
//*   Input:  DLIST       ListToReset : The list whose current item   */
//*                                    is to be set to the first item */
//*                                    in the list                    */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, *Error will be set to 0.                */
//*            If unsuccessful, *Error will be set to a non-zero error*/
//*               code.                                               */
//*                                                                   */
//*   Error Handling: This function will fail if ListToReset is not   */
//*                   a valid list.  If this occurs, then *Error      */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System GoToStartOfList( DLIST ListToReset, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name: GoToEndOfList                                    */
//*                                                                   */
//*   Descriptive Name:  This function makes the last item in the     */
//*                      list the current item in the list.           */
//*                                                                   */
//*   Input:  DLIST       ListToSet : The list whose current item     */
//*                                    is to be set to the last item  */
//*                                    in the list                    */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, *Error will be set to 0.                */
//*            If unsuccessful, *Error will be set to a non-zero error*/
//*               code.                                               */
//*                                                                   */
//*   Error Handling: This function will fail if ListToSet is not     */
//*                   a valid list.  If this occurs, then *Error      */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System GoToEndOfList( DLIST ListToSet, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name: GoToSpecifiedItem                                */
//*                                                                   */
//*   Descriptive Name:  This function makes the item associated with */
//*                      Handle the current item in the list.         */
//*                                                                   */
//*   Input:  DLIST  ListToReposition:  The list whose current item   */
//*                                    is to be set to the item       */
//*                                    associated with Handle.        */
//*           ADDRESS Handle : A handle obtained by using the         */
//*                            GetHandle function.  This handle       */
//*                            identifies a unique item in the list.  */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return code              */
//*                                                                   */
//*   Output:  If successful, *Error will be set to 0.                */
//*            If unsuccessful, *Error will be set to a non-zero error*/
//*               code.                                               */
//*                                                                   */
//*   Error Handling: This function will fail if ListToReposition is  */
//*                   not a valid list.  If this occurs, then *Error  */
//*                   is set to a non-zero error code.                */
//*                                                                   */
//*   Side Effects:  None.                                            */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*                                                                   */
//*           It is assumed that Handle is a valid handle and that    */
//*           the item associated with Handle is still in the list.   */
//*           If these conditions are not met, an exception or trap   */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System GoToSpecifiedItem( DLIST ListToReposition, ADDRESS Handle, CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  SortList                                        */
//*                                                                   */
//*   Descriptive Name:  This function sorts the contents of a list.  */
//*                      The sorting algorithm used is a stable sort  */
//*                      whose performance is not dependent upon the  */
//*                      initial order of the items in the list.      */
//*                                                                   */
//*   Input: DLIST ListToSort : The DLIST that is to be sorted.       */
//*                                                                   */
//*          INTEGER32 ( *Compare) ( ... )                             */
//*                                                                   */
//*              This is a pointer to a function that can compare any */
//*              two items in the list.  It should return -1 if       */
//*              Object1 is less than Object2, 0 if Object1 is equal  */
//*              to Object2, and 1 if Object1 is greater than Object2.*/
//*              This function will be called during the sort whenever*/
//*              the sorting algorithm needs to compare two objects.  */
//*                                                                   */
//*              The Compare function takes the following parameters: */
//*                                                                   */
//*              ADDRESS Object1 : The address of the data for the    */
//*                                first object to be compared.       */
//*              TAG Object1Tag : The user assigned TAG value for the */
//*                               first object to be compared.        */
//*              ADDRESS Object2 : The address of the data for the    */
//*                                second object to be compared.      */
//*              TAG Object2Tag : The user assigned TAG value for the */
//*                               second object to be compared.       */
//*              CARDINAL32 * Error : The address of a variable to    */
//*                                   hold the error return value.    */
//*                                                                   */
//*              If this function ever sets *Error to a non-zero value*/
//*              the sort will terminate and the error code will be   */
//*              returned to the caller of the SortList function.     */
//*                                                                   */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return value.             */
//*                                                                   */
//*   Output:  If successful, this function will set *Error to        */
//*               DLIST_SUCCESS and ListToSort will have been sorted. */
//*            If unsuccessful, *Error will contain an error code.    */
//*               The order of the items in ListToSort is undefined   */
//*               and may have changed.                               */
//*                                                                   */
//*   Error Handling: This function will terminate if *Compare sets   */
//*                   *Error to a non-zero value, or if ListToSort    */
//*                   is invalid.  If this function does terminate in */
//*                   the middle of a sort, the order of the items in */
//*                   ListToSort may be different than it was before  */
//*                   the function was called.                        */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes:  It is assumed that Error contains a valid address. If   */
//*           this assumption is violated, an exception or trap       */
//*           may occur.                                              */
//*                                                                   */
//*********************************************************************/
void _System SortList(DLIST        ListToSort,
INTEGER32    ( * _System Compare) (ADDRESS Object1, TAG Object1Tag, ADDRESS Object2, TAG Object2Tag,CARDINAL32 * Error),
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  ForEachItem                                     */
//*                                                                   */
//*   Descriptive Name:  This function passes a pointer to each item  */
//*                      in a list to a user provided function for    */
//*                      processing by the user provided function.    */
//*                                                                   */
//*   Input:  DLIST ListToProcess : The DLIST whose items are to be   */
//*                                processed by the user provided     */
//*                                function.                          */
//*                                                                   */
//*           void ( * ProcessItem) (...)                               */
//*                                                                   */
//*               This is a pointer to the user provided function.    */
//*               This user provided function takes the following     */
//*                  parameters:                                      */
//*                                                                   */
//*                  ADDRESS Object : A pointer to an item in         */
//*                                   ListToProcess.                  */
//*                  TAG Object1Tag : The user assigned TAG value for */
//*                                   the item pointed to by Object.  */
//*                  ADDRESS Parameter : The address of a block of    */
//*                                      memory containing any        */
//*                                      parameters that the user     */
//*                                      wishes to have passed to this*/
//*                                      function.                    */
//*                  CARDINAL32 * Error : The address of a variable to*/
//*                                       hold the error return value.*/
//*                                                                   */
//*           ADDRESS Parameters : This field is passed through to    */
//*                                *ProcessItem.  This function does  */
//*                                not even look at the contents of   */
//*                                this field.  This field is here to */
//*                                provide the user a way to pass     */
//*                                additional data to *ProcessItem    */
//*                                that *ProcessItem may need to      */
//*                                function correctly.                */
//*                                                                   */
//*           BOOLEAN Forward : If TRUE, then the list is traversed   */
//*                             from the start of the list to the end */
//*                             of the list.  If FALSE, then the list */
//*                             is traversed from the end of the list */
//*                             to the beginning.                     */
//*                                                                   */
//*           CARDINAL32 * Error : The address of a variable to hold  */
//*                                the error return value.            */
//*                                                                   */
//*   Output:  If successful, this function will set *Error to        */
//*               DLIST_SUCCESS.                                      */
//*            If unsuccessful, then this function will set *Error to */
//*               a non-zero error code.                              */
//*                                                                   */
//*   Error Handling: This function aborts immediately when an error  */
//*                   is detected, and any remaining items in the list*/
//*                   will not be processed.                          */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes: This function allows the user to access all of the items */
//*          in a list and perform an operation on them.  The         */
//*          operation performed must not free any items in the list, */
//*          or perform any list operations on the list being         */
//*          processed.                                               */
//*                                                                   */
//*          As an example of when this would be useful, consider a   */
//*          a list of graphic objects (rectangles, triangles, circles*/
//*          etc.)  which comprise a drawing.  To draw the picture    */
//*          that these graphic objects represent, one could build a  */
//*          loop which gets and draws each item.  Another way to     */
//*          do this would be to build a drawing function which can   */
//*          draw any of the graphic objects, and then use that       */
//*          function as the ProcessItem function in a call to        */
//*          ForEachItem.                                             */
//*                                                                   */
//*          If the ProcessItem function sets *Error to something     */
//*          other than DLIST_SUCCESS, then ForEachItem will terminate*/
//*          and return an error to whoever called it.  The single    */
//*          exception to this is if ProcessItem sets *Error to       */
//*          DLIST_SEARCH_COMPLETE, in which case ForEachItem         */
//*          terminates and sets *Error to DLIST_SUCCESS.  This is    */
//*          usefull for using ForEachItem to search a list and then  */
//*          terminating the search once the desired item is found.   */
//*                                                                   */
//*          A word about the Parameters parameter.  This parameter   */
//*          is passed through to *ProcessItem and is never looked at */
//*          by this function.  This means that the user can put any  */
//*          value they desire into Parameters as long as it is the   */
//*          same size (in bytes) as Parameters.  The intended use of */
//*          Parameters is to allow the user to pass information to   */
//*          *ProcessItem that *ProcessItem may need.  Either way,    */
//*          how Parameters is used is literally up to the user.      */
//*                                                                   */
//*********************************************************************/
void _System ForEachItem(DLIST        ListToProcess,
void         ( * _System ProcessItem) (ADDRESS Object, TAG ObjectTag, CARDINAL32 ObjectSize, ADDRESS ObjectHandle, ADDRESS Parameters, CARDINAL32 * Error),
ADDRESS      Parameters,
BOOLEAN      Forward,
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  PruneList                                       */
//*                                                                   */
//*   Descriptive Name:  This function allows the caller to examine   */
//*                      each item in a list and optionally delete    */
//*                      it from the list.                            */
//*                                                                   */
//*   Input:  DLIST ListToProcess : The DLIST to be pruned.           */
//*                                                                   */
//*           BOOLEAN ( * KillItem) (...)                               */
//*                                                                   */
//*               This is a pointer to a user provided function.      */
//*               This user provided function takes the following     */
//*                  parameters:                                      */
//*                                                                   */
//*                  ADDRESS Object : A pointer to an item in         */
//*                                   ListToProcess.                  */
//*                  TAG Object1Tag : The user assigned TAG value for */
//*                                   the item pointed to by Object.  */
//*                  ADDRESS Parameter : The address of a block of    */
//*                                      memory containing any        */
//*                                      parameters that the user     */
//*                                      wishes to have passed to this*/
//*                                      function.                    */
//*                  BOOLEAN * FreeMemory : The address of a BOOLEAN  */
//*                                         variable which this       */
//*                                         function will set to      */
//*                                         either TRUE or FALSE.     */
//*                                         If the function return    */
//*                                         value is TRUE, then the   */
//*                                         value in *FreeMemory will */
//*                                         be examined.  If it is    */
//*                                         TRUE, then PruneList will */
//*                                         free the memory associated*/
//*                                         with the item being       */
//*                                         deleted.  If *FreeMemory  */
//*                                         is FALSE, then the item   */
//*                                         being removed from the    */
//*                                         DLIST will not be freed,  */
//*                                         and it is up to the user  */
//*                                         to ensure that this memory*/
//*                                         is handled properly.      */
//*                  CARDINAL32 * Error : The address of a variable to*/
//*                                       hold the error return value.*/
//*                                                                   */
//*           ADDRESS Parameters : This field is passed through to    */
//*                                *ProcessItem.  This function does  */
//*                                not even look at the contents of   */
//*                                this field.  This field is here to */
//*                                provide the user a way to pass     */
//*                                additional data to *ProcessItem    */
//*                                that *ProcessItem may need to      */
//*                                function correctly.                */
//*                                                                   */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return value.             */
//*                                                                   */
//*   Output:  If successful, this function will set *Error to        */
//*               DLIST_SUCCESS.                                      */
//*            If unsuccessful, then this function will set *Error to */
//*               a non-zero error code.                              */
//*                                                                   */
//*   Error Handling: This function aborts immediately when an error  */
//*                   is detected, and any remaining items in the list*/
//*                   will not be processed.                          */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes: This function allows the user to access all of the items */
//*          in a list, perform an operation on them, and then        */
//*          optionally delete ("remove") them from the DLIST.  The   */
//*          operation performed must not free any items in the list, */
//*          or perform any list operations on the list being         */
//*          processed.                                               */
//*                                                                   */
//*          If the KillItem function sets *Error to something other  */
//*          than DLIST_SUCCESS, then PruneList will terminate and    */
//*          return an error to whoever called it.  The single        */
//*          exception to this is if KillItem sets *Error to          */
//*          DLIST_SEARCH_COMPLETE, in which case KillItem            */
//*          terminates and sets *Error to DLIST_SUCCESS.  This is    */
//*          usefull for using KillItem to search a list and then     */
//*          terminating the search once the desired item is found.   */
//*                                                                   */
//*          A word about the Parameters parameter.  This parameter   */
//*          is passed through to *ProcessItem and is never looked at */
//*          by this function.  This means that the user can put any  */
//*          value they desire into Parameters as long as it is the   */
//*          same size (in bytes) as Parameters.  The intended use of */
//*          Parameters is to allow the user to pass information to   */
//*          *ProcessItem that *ProcessItem may need.  Either way,    */
//*          how Parameters is used is literally up to the user.      */
//*                                                                   */
//*********************************************************************/
void _System PruneList(DLIST        ListToProcess,
BOOLEAN      ( * _System KillItem) (ADDRESS Object, TAG ObjectTag, CARDINAL32 ObjectSize, ADDRESS ObjectHandle, ADDRESS Parameters, BOOLEAN * FreeMemory, CARDINAL32 * Error),
ADDRESS      Parameters,
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  AppendList                                      */
//*                                                                   */
//*   Descriptive Name: Removes the items in SourceList and appends   */
//*                     them to TargetList.                           */
//*                                                                   */
//*   Input:  DLIST TargetList : The DLIST which is to have the items */
//*                             from SourceList appended to it.       */
//*           DLIST SourceList : The DLIST whose items are to be      */
//*                              removed and appended to TargetList.  */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return value.             */
//*                                                                   */
//*   Output: If successful, *Error will be set to DLIST_SUCCESS,     */
//*              SourceList will be empty, and TargetList will contain*/
//*              all of its original items and all of the items that  */
//*              were in SourceList.                                  */
//*           If unsuccessful, *Error will be set to a non-zero value */
//*              and SourceList and TargetList will be unmodified.    */
//*                                                                   */
//*   Error Handling:  This function will abort immediately upon      */
//*                    detection of an error.  All errors that can be */
//*                    detected are detected before the contents of   */
//*                    SourceList are appended to TargetList, so if an*/
//*                    error is detected and the function aborts,     */
//*                    SourceList and TargetList are unaltered.       */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes: None.                                                    */
//*                                                                   */
//*********************************************************************/
void _System AppendList(DLIST        TargetList,
DLIST        SourceList,
CARDINAL32 * Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  TransferItem                                    */
//*                                                                   */
//*   Descriptive Name: Removes an item in SourceList and places in   */
//*                     TargetList.                                   */
//*                                                                   */
//*   Input:  DLIST SourceList : The DLIST containing the item which  */
//*                              is to be transferred.                */
//*           ADDRESS SourceHandle : The handle of the item in        */
//*                                   SourceList which is to be       */
//*                                   transferred to another DLIST.   */
//*                                   If this is NULL, then the       */
//*                                   current item in SourceList will */
//*                                   be used.                        */
//*           DLIST TargetList : The DLIST which is to receive the    */
//*                              item being transferred.              */
//*           ADDRESS TargetHandle : The item in TargetList which     */
//*                                   is used to determine where      */
//*                                   the item being transferred will */
//*                                   be placed.  If this is NULL,    */
//*                                   then the current item in        */
//*                                   TargetList will be used.        */
//*           Insertion_Modes TransferMode : This indicates where,    */
//*                                   relative to the item in         */
//*                                   TargetList specified by         */
//*                                   Target_Handle, the item being   */
//*                                   transferred can be placed.      */
//*          BOOLEAN MakeCurrent : If TRUE, the item transferred to   */
//*                                 TargetList becomes the current    */
//*                                 item in TargetList.               */
//*          CARDINAL32 * Error : The address of a variable to hold   */
//*                               the error return value.             */
//*                                                                   */
//*   Output: If successful, *Error will be set to DLIST_SUCCESS,     */
//*              SourceList will be empty, and TargetList will contain*/
//*              all of its original items and all of the items that  */
//*              were in SourceList.                                  */
//*           If unsuccessful, *Error will be set to a non-zero value */
//*              and SourceList and TargetList will be unmodified.    */
//*                                                                   */
//*   Error Handling:  This function will abort immediately upon      */
//*                    detection of an error.  All errors that can be */
//*                    detected are detected before the contents of   */
//*                    SourceList are appended to TargetList, so if an*/
//*                    error is detected and the function aborts,     */
//*                    SourceList and TargetList are unaltered.       */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes: None.                                                    */
//*                                                                   */
//*********************************************************************/
void _System TransferItem(DLIST             SourceList,
ADDRESS           SourceHandle,
DLIST             TargetList,
ADDRESS           TargetHandle,
Insertion_Modes   TransferMode,
BOOLEAN           MakeCurrent,
CARDINAL32 *      Error);

//*********************************************************************/
//*                                                                   */
//*   Function Name:  CheckListIntegrity                              */
//*                                                                   */
//*   Descriptive Name: Checks the integrity of a DLIST.  All link    */
//*                     nodes in the list are checked, as are all     */
//*                     fields in the list control block.             */
//*                                                                   */
//*   Input:  DLIST ListToCheck - The list whose integrity is to be   */
//*                               checked.                            */
//*                                                                   */
//*   Output: The function return value will be TRUE if all of the    */
//*           elements in the DLIST are correct.  If this function    */
//*           returns FALSE, then the DLIST being checked has been    */
//*           corrupted!                                              */
//*                                                                   */
//*   Error Handling: If this function encounters an error in a DLIST,*/
//*                   it will return FALSE.                           */
//*                                                                   */
//*   Side Effects: None.                                             */
//*                                                                   */
//*   Notes: None.                                                    */
//*                                                                   */
//*********************************************************************/
BOOLEAN _System CheckListIntegrity(DLIST ListToCheck);

{$endif}

Implementation

End.
