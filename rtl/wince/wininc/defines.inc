{
    This file is part of the Free Pascal run time library.
    This unit contains the constant definitions for the Win32 API
    Copyright (c) 1999-2000 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  Defines.h

  Windows32 API definitions

  Copyright (C) 1996, 1997 Free Software Foundation, Inc.

  Author: Scott Christley <scottc@net-community.com>

  This file is part of the Windows32 API Library.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Library General Public
  License as published by the Free Software Foundation; either
  version 2 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Library General Public License for more details.

  If you are interested in a warranty or support for this source code,
  contact Scott Christley <scottc@net-community.com> for more information.

  License along with this library; see the file COPYING.LIB.
  If not, write to the Free Software Foundation,
  59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.


}


{$ifdef read_interface}


{$PACKRECORDS C}


  const
     UNICODE_NULL  = widechar(#0);
     NULL=0;                            //+windef
     //FALSE=0;                         //+windef, removed can't compile
     //TRUE=1;                          //+windef, removed can't compile
     MAX_PATH = 260;                    //windef
     LF_FACESIZE = 32;
     LF_FULLFACESIZE = 64;
     ELF_VENDOR_SIZE = 4;
     SECURITY_STATIC_TRACKING = 0;
     SECURITY_DYNAMIC_TRACKING = 1;
     MAX_DEFAULTCHAR = 2;
     MAX_LEADBYTES = 12;
     EXCEPTION_MAXIMUM_PARAMETERS = 15; //winnt
     CCHDEVICENAME = 32;
     CCHFORMNAME = 32;
     MENU_TEXT_LEN = 40;
     MAX_LANA = 254;
     NCBNAMSZ = 16;
     NETBIOS_NAME_LEN = 16;
     OFS_MAXPATHNAME = 128;
     MAX_TAB_STOPS = 32;
     RAS_MaxCallbackNumber = 128;
     RAS_MaxDeviceName = 128;
     RAS_MaxDeviceType = 16;
     RAS_MaxEntryName = 256;
     RAS_MaxIpAddress = 15;
     RAS_MaxIpxAddress = 21;
     RAS_MaxPhoneNumber = 128;
     UNLEN = 256;
     PWLEN = 256;
     CNLEN = 15;
     DNLEN = 15;
  { Unsigned types max  }
     MAXDWORD = $FFFFFFFF;     //winnt
     MAXWORD = $FFFF;          //winnt
     MAXBYTE = $FF;            //winnt
  { Signed types max/min  }
     MINCHAR = $80;            //winnt
     MAXCHAR = $7F;            //winnt
     MINSHORT = $8000;         //winnt
     MAXSHORT = $7FFF;         //winnt
     MINLONG = $80000000;      //winnt
     MAXLONG = $7FFFFFFF;      //winnt
  { _llseek  }
     FILE_BEGIN = 0;
     FILE_CURRENT = 1;
     FILE_END = 2;
  { _lopen, LZOpenFile, OpenFile  }
     OF_READ = 0;
     OF_READWRITE = 2;
     OF_WRITE = 1;
     OF_SHARE_COMPAT = 0;
     OF_SHARE_DENY_NONE = 64;
     OF_SHARE_DENY_READ = 48;
     OF_SHARE_DENY_WRITE = 32;
     OF_SHARE_EXCLUSIVE = 16;
     OF_CANCEL = 2048;
     OF_CREATE = 4096;
     OF_DELETE = 512;
     OF_EXIST = 16384;
     OF_PARSE = 256;
     OF_PROMPT = 8192;
     OF_REOPEN = 32768;
     OF_VERIFY = 1024;
  { ActivateKeyboardLayout, LoadKeyboardLayout  }
     HKL_NEXT = 1;
     HKL_PREV = 0;
     KLF_REORDER = 8;
     KLF_UNLOADPREVIOUS = 4;
     KLF_ACTIVATE = 1;
     KLF_NOTELLSHELL = 128;
     KLF_REPLACELANG = 16;
     KLF_SUBSTITUTE_OK = 2;
  { AppendMenu  }
     MF_BITMAP = $4;
     MF_DISABLED = $2;
     MF_ENABLED = 0;
     MF_GRAYED = $1;
     MF_HELP = $4000;
     MF_MENUBARBREAK = $20;
     MF_MENUBREAK = $40;
     MF_MOUSESELECT = $8000;
     MF_OWNERDRAW = $100;
     MF_POPUP = $10;
     MF_SEPARATOR = $800;
     MF_STRING = 0;
     MF_SYSMENU = $2000;
     MF_USECHECKBITMAPS = $200;
  { Ternary Raster Operations - BitBlt  }
     BLACKNESS = $00000042;
     NOTSRCERASE = $001100A6;
     NOTSRCCOPY = $00330008;
     SRCERASE = $00440328;
     DSTINVERT = $00550009;
     PATINVERT = $005A0049;
     SRCINVERT = $00660046;
     SRCAND = $008800C6;
     MERGEPAINT = $00BB0226;
     MERGECOPY = $00C000CA;
     SRCCOPY = $00CC0020;
     SRCPAINT = $00EE0086;
     PATCOPY = $00F00021;
     PATPAINT = $00FB0A09;
     WHITENESS = $00FF0062;
  { Binary Raster Operations  }
     R2_BLACK = 1;
     R2_COPYPEN = 13;
     R2_MASKNOTPEN = 3;
     R2_MASKPEN = 9;
     R2_MASKPENNOT = 5;
     R2_MERGENOTPEN = 12;
     R2_MERGEPEN = 15;
     R2_MERGEPENNOT = 14;
     R2_NOP = 11;
     R2_NOT = 6;
     R2_NOTCOPYPEN = 4;
     R2_NOTMASKPEN = 8;
     R2_NOTMERGEPEN = 2;
     R2_NOTXORPEN = 10;
     R2_WHITE = 16;
     R2_XORPEN = 7;
  { BroadcastSystemMessage  }
     BSF_FLUSHDISK = 4;
     BSF_FORCEIFHUNG = 32;
     BSF_IGNORECURRENTTASK = 2;
     BSF_NOHANG = 8;
     BSF_POSTMESSAGE = 16;
     BSF_QUERY = 1;
     BSM_ALLCOMPONENTS = 0;
     BSM_APPLICATIONS = 8;
     BSM_INSTALLABLEDRIVERS = 4;
     BSM_NETDRIVER = 2;
     BSM_VXDS = 1;
     BROADCAST_QUERY_DENY = 1112363332;
  { CascadeWindows, TileWindows  }
     MDITILE_SKIPDISABLED = 2;
     MDITILE_HORIZONTAL = 1;
     MDITILE_VERTICAL = 0;
  { CBTProc  }
     HCBT_ACTIVATE = 5;
     HCBT_CLICKSKIPPED = 6;
     HCBT_CREATEWND = 3;
     HCBT_DESTROYWND = 4;
     HCBT_KEYSKIPPED = 7;
     HCBT_MINMAX = 1;
     HCBT_MOVESIZE = 0;
     HCBT_QS = 2;
     HCBT_SETFOCUS = 9;
     HCBT_SYSCOMMAND = 8;
  { ChangeDisplaySettings  }
     DM_BITSPERPEL              = $40000;
     DM_PELSWIDTH               = $80000;
     DM_PELSHEIGHT              = $100000;
     DM_DISPLAYFLAGS            = $200000;
     DM_DISPLAYFREQUENCY        = $400000;
     DM_DISPLAYORIENTATION      = $00800000;
     DM_DISPLAYQUERYORIENTATION = $01000000;
  { rotation angle for screen rotation }
     DMDO_0       = 0;
     DMDO_90      = 1;
     DMDO_180     = 2;
     DMDO_270     = 4;
     DMDO_DEFAULT = DMDO_0;

     CDS_UPDATEREGISTRY = 1;
     CDS_TEST = 2;
     CDS_FULLSCREEN = 4;
     CDS_GLOBAL = 8;
     CDS_SET_PRIMARY = $10;
     CDS_RESET = $40000000;
     CDS_SETRECT = $20000000;
     CDS_NORESET = $10000000;
     DISP_CHANGE_SUCCESSFUL = 0;
     DISP_CHANGE_RESTART = 1;
     DISP_CHANGE_BADFLAGS = -(4);
     DISP_CHANGE_FAILED = -(1);
     DISP_CHANGE_BADMODE = -(2);
     DISP_CHANGE_NOTUPDATED = -(3);
  { ChangeServiceConfig  }
     SERVICE_NO_CHANGE = -(1);
     SERVICE_WIN32_OWN_PROCESS = 16;
     SERVICE_WIN32_SHARE_PROCESS = 32;
     SERVICE_KERNEL_DRIVER = 1;
     SERVICE_FILE_SYSTEM_DRIVER = 2;
     SERVICE_INTERACTIVE_PROCESS = 256;
     SERVICE_BOOT_START = 0;
     SERVICE_SYSTEM_START = 1;
     SERVICE_AUTO_START = 2;
     SERVICE_DEMAND_START = 3;
     SERVICE_DISABLED = 4;
  { SERVICE_STATUS structure  }
     SERVICE_STOPPED = 1;
     SERVICE_START_PENDING = 2;
     SERVICE_STOP_PENDING = 3;
     SERVICE_RUNNING = 4;
     SERVICE_CONTINUE_PENDING = 5;
     SERVICE_PAUSE_PENDING = 6;
     SERVICE_PAUSED = 7;
     SERVICE_ACCEPT_STOP = 1;
     SERVICE_ACCEPT_PAUSE_CONTINUE = 2;
     SERVICE_ACCEPT_SHUTDOWN = 4;
  { CheckDlgButton  }
     BST_CHECKED = 1;
     BST_INDETERMINATE = 2;
     BST_UNCHECKED = 0;
     BST_FOCUS = 8;
     BST_PUSHED = 4;
  { CheckMenuItem, HiliteMenuItem  }
     MF_BYCOMMAND = 0;
     MF_BYPOSITION = $400;
     MF_CHECKED = $8;
     MF_UNCHECKED = 0;
     MF_HILITE = $80;
     MF_UNHILITE = 0;
  { ChildWindowFromPointEx  }
     CWP_ALL = 0;
     CWP_SKIPINVISIBLE = 1;
     CWP_SKIPDISABLED = 2;
     CWP_SKIPTRANSPARENT = 4;
  { ClearCommError  }
     CE_BREAK = 16;
     CE_DNS = 2048;
     CE_FRAME = 8;
     CE_IOE = 1024;
     CE_MODE = 32768;
     CE_OOP = 4096;
     CE_OVERRUN = 2;
     CE_PTO = 512;
     CE_RXOVER = 1;
     CE_RXPARITY = 4;
     CE_TXFULL = 256;
  { ChooseMatchToTarget  }
  { CombineRgn  }
     RGN_AND = 1;
     RGN_COPY = 5;
     RGN_DIFF = 4;
     RGN_OR = 2;
     RGN_XOR = 3;
     NULLREGION = 1;
     SIMPLEREGION = 2;
     COMPLEXREGION = 3;
     ERROR = 0;
  { CommonDlgExtendedError  }
     CDERR_DIALOGFAILURE = $ffff;
     CDERR_FINDRESFAILURE = 6;
     CDERR_INITIALIZATION = 2;
     CDERR_LOADRESFAILURE = 7;
     CDERR_LOADSTRFAILURE = 5;
     CDERR_LOCKRESFAILURE = 8;
     CDERR_MEMALLOCFAILURE = 9;
     CDERR_MEMLOCKFAILURE = 10;
     CDERR_NOHINSTANCE = 4;
     CDERR_NOHOOK = 11;
     CDERR_NOTEMPLATE = 3;
     CDERR_REGISTERMSGFAIL = 12;
     CDERR_STRUCTSIZE = 1;
     PDERR_CREATEICFAILURE = $1000 + 10;
     PDERR_DEFAULTDIFFERENT = $1000 + 12;
     PDERR_DNDMMISMATCH = $1000 + 9;
     PDERR_GETDEVMODEFAIL = $1000 + 5;
     PDERR_INITFAILURE = $1000 + 6;
     PDERR_LOADDRVFAILURE = $1000 + 4;
     PDERR_NODEFAULTPRN = $1000 + 8;
     PDERR_NODEVICES = $1000 + 7;
     PDERR_PARSEFAILURE = $1000 + 2;
     PDERR_PRINTERNOTFOUND = $1000 + 11;
     PDERR_RETDEFFAILURE = $1000 + 3;
     PDERR_SETUPFAILURE = $1000 + 1;
     CFERR_MAXLESSTHANMIN = $2000 + 2;
     CFERR_NOFONTS = $2000 + 1;
     FNERR_BUFFERTOOSMALL = $3000 + 3;
     FNERR_INVALIDFILENAME = $3000 + 2;
     FNERR_SUBCLASSFAILURE = $3000 + 1;
     FRERR_BUFFERLENGTHZERO = $4000 + 1;
  { CompareString, LCMapString  }
     //LOCALE_SYSTEM_DEFAULT = $800; //-winnt, replaced by function
     //LOCALE_USER_DEFAULT = $400;   //-winnt, replaced by function
     NORM_IGNORECASE = 1;
     NORM_IGNOREKANATYPE = 65536;
     NORM_IGNORENONSPACE = 2;
     NORM_IGNORESYMBOLS = 4;
     NORM_IGNOREWIDTH = 131072;
     SORT_STRINGSORT = 4096;
     LCMAP_BYTEREV = 2048;
     LCMAP_FULLWIDTH = 8388608;
     LCMAP_HALFWIDTH = 4194304;
     LCMAP_HIRAGANA = 1048576;
     LCMAP_KATAKANA = 2097152;
     LCMAP_LOWERCASE = 256;
     LCMAP_SORTKEY = 1024;
     LCMAP_UPPERCASE = 512;
  { ContinueDebugEvent  }
     DBG_CONTINUE = $10002;
     DBG_CONTROL_BREAK = $40010008;
     DBG_CONTROL_C = $40010005;
     DBG_EXCEPTION_NOT_HANDLED = $80010001;
     DBG_TERMINATE_THREAD = $40010003;
     DBG_TERMINATE_PROCESS = $40010004;
  { ControlService  }
     SERVICE_CONTROL_STOP = 1;
     SERVICE_CONTROL_PAUSE = 2;
     SERVICE_CONTROL_CONTINUE = 3;
     SERVICE_CONTROL_INTERROGATE = 4;
     SERVICE_CONTROL_SHUTDOWN = 5;
  { CopyImage, LoadImage  }
     IMAGE_BITMAP = 0;
     IMAGE_CURSOR = 2;
     IMAGE_ENHMETAFILE = 1;
     IMAGE_ICON = 1;
     LR_COPYDELETEORG = 8;
     LR_COPYRETURNORG = 4;
     LR_MONOCHROME = 1;
     LR_CREATEDIBSECTION = 8192;
     LR_DEFAULTSIZE = 64;
  { CreateDesktop  }
     DF_ALLOWOTHERACCOUNTHOOK = $1;
     DESKTOP_CREATEMENU = $4;
     DESKTOP_CREATEWINDOW = $2;
     DESKTOP_ENUMERATE = $40;
     DESKTOP_HOOKCONTROL = $8;
     DESKTOP_JOURNALPLAYBACK = $20;
     DESKTOP_JOURNALRECORD = $10;
     DESKTOP_READOBJECTS = $1;
     DESKTOP_SWITCHDESKTOP = $100;
     DESKTOP_WRITEOBJECTS = $80;
     WSF_VISIBLE = $1;
  { CreateDIBitmap  }
     CBM_INIT = $4;
     DIB_PAL_COLORS = 1;
     DIB_RGB_COLORS = 0;
  { file & pipe  }
     FILE_READ_DATA = $0001;
  { directory  }
     FILE_LIST_DIRECTORY = $0001;
  { file & pipe  }
     FILE_WRITE_DATA = $0002;
  { directory  }
     FILE_ADD_FILE = $0002;
  { file  }
     FILE_APPEND_DATA = $0004;
  { directory  }
     FILE_ADD_SUBDIRECTORY = $0004;
  { named pipe  }
     FILE_CREATE_PIPE_INSTANCE = $0004;
  { file & directory  }
     FILE_READ_EA = $0008;
     FILE_READ_PROPERTIES = FILE_READ_EA;
  { file & directory  }
     FILE_WRITE_EA = $0010;
     FILE_WRITE_PROPERTIES = FILE_WRITE_EA;
  { file  }
     FILE_EXECUTE = $0020;
  { directory  }
     FILE_TRAVERSE = $0020;
  { directory  }
     FILE_DELETE_CHILD = $0040;
  { all  }
     FILE_READ_ATTRIBUTES = $0080;
  { all  }
     FILE_WRITE_ATTRIBUTES = $0100;
  { displaced lower
  #define FILE_ALL_ACCESS (STANDARD_RIGHTS_REQUIRED | SYNCHRONIZE | 0x1FF)

  #define FILE_GENERIC_READ         (STANDARD_RIGHTS_READ     |\
                                     FILE_READ_DATA           |\
                                     FILE_READ_ATTRIBUTES     |\
                                     FILE_READ_EA             |\
                                     SYNCHRONIZE)


  #define FILE_GENERIC_WRITE        (STANDARD_RIGHTS_WRITE    |\
                                     FILE_WRITE_DATA          |\
                                     FILE_WRITE_ATTRIBUTES    |\
                                     FILE_WRITE_EA            |\
                                     FILE_APPEND_DATA         |\
                                     SYNCHRONIZE)


  #define FILE_GENERIC_EXECUTE      (STANDARD_RIGHTS_EXECUTE  |\
                                     FILE_READ_ATTRIBUTES     |\
                                     FILE_EXECUTE             |\
                                     SYNCHRONIZE)
   }
     FILE_SHARE_DELETE = 4;
     FILE_SHARE_READ = 1;
     FILE_SHARE_WRITE = 2;
     CONSOLE_TEXTMODE_BUFFER = 1;
     CREATE_NEW = 1;
     CREATE_ALWAYS = 2;
     OPEN_EXISTING = 3;
     OPEN_ALWAYS = 4;
     TRUNCATE_EXISTING = 5;
     FILE_ATTRIBUTE_ARCHIVE = 32;
     FILE_ATTRIBUTE_COMPRESSED = 2048;
     FILE_ATTRIBUTE_NORMAL = 128;
     FILE_ATTRIBUTE_DIRECTORY = 16;
     FILE_ATTRIBUTE_HIDDEN = 2;
     FILE_ATTRIBUTE_READONLY = 1;
     FILE_ATTRIBUTE_SYSTEM = 4;
     FILE_ATTRIBUTE_TEMPORARY = 256;
     FILE_FLAG_WRITE_THROUGH = $80000000;
     FILE_FLAG_OVERLAPPED = 1073741824;
     FILE_FLAG_NO_BUFFERING = 536870912;
     FILE_FLAG_RANDOM_ACCESS = 268435456;
     FILE_FLAG_SEQUENTIAL_SCAN = 134217728;
     FILE_FLAG_DELETE_ON_CLOSE = 67108864;
     FILE_FLAG_BACKUP_SEMANTICS = 33554432;
     FILE_FLAG_POSIX_SEMANTICS = 16777216;
     SECURITY_ANONYMOUS = 0;
     SECURITY_IDENTIFICATION = 65536;
     SECURITY_IMPERSONATION = 131072;
     SECURITY_DELEGATION = 196608;
     SECURITY_CONTEXT_TRACKING = 262144;
     SECURITY_EFFECTIVE_ONLY = 524288;
     SECURITY_SQOS_PRESENT = 1048576;
  { CreateFileMapping, VirtualAlloc, VirtualFree, VirtualProtect  }
     SEC_COMMIT = 134217728;
     SEC_IMAGE = 16777216;
     SEC_NOCACHE = 268435456;
     SEC_RESERVE = 67108864;
     PAGE_READONLY = 2;
     PAGE_READWRITE = 4;
     PAGE_WRITECOPY = 8;
     PAGE_EXECUTE = 16;
     PAGE_EXECUTE_READ = 32;
     PAGE_EXECUTE_READWRITE = 64;
     PAGE_EXECUTE_WRITECOPY = 128;
     PAGE_GUARD = 256;
     PAGE_NOACCESS = 1;
     PAGE_NOCACHE = 512;
     MEM_COMMIT = 4096;
     MEM_FREE = 65536;
     MEM_RESERVE = 8192;
     MEM_IMAGE = 16777216;
     MEM_MAPPED = 262144;
     MEM_PRIVATE = 131072;
     MEM_DECOMMIT = 16384;
     MEM_RELEASE = 32768;
     MEM_TOP_DOWN = 1048576;
     EXCEPTION_GUARD_PAGE = $80000001;
     SECTION_EXTEND_SIZE = $10;
     SECTION_MAP_READ = $4;
     SECTION_MAP_WRITE = $2;
     SECTION_QUERY = $1;
     SECTION_ALL_ACCESS = $f001f;
  { CreateFont  }
     FW_DONTCARE = 0;
     FW_THIN = 100;
     FW_EXTRALIGHT = 200;
     FW_LIGHT = 300;
     FW_NORMAL = 400;
     FW_REGULAR = FW_NORMAL;
     FW_MEDIUM = 500;
     FW_SEMIBOLD = 600;
     FW_BOLD = 700;
     FW_EXTRABOLD = 800;
     FW_HEAVY = 900;
     ANSI_CHARSET = 0;
     DEFAULT_CHARSET = 1;
     SYMBOL_CHARSET = 2;
     SHIFTJIS_CHARSET = 128;
     HANGEUL_CHARSET = 129;
     GB2312_CHARSET = 134;
     CHINESEBIG5_CHARSET = 136;
     GREEK_CHARSET = 161;
     TURKISH_CHARSET = 162;
     HEBREW_CHARSET = 177;
     ARABIC_CHARSET = 178;
     BALTIC_CHARSET = 186;
     RUSSIAN_CHARSET = 204;
     THAI_CHARSET = 222;
     EASTEUROPE_CHARSET = 238;

     OEM_CHARSET = 255;
     OEM_CERTIFY_TRUST = 2;   //+windef
     OEM_CERTIFY_RUN = 1;     //+windef
     OEM_CERTIFY_FALSE = 0;   //+windef

     OUT_DEFAULT_PRECIS = 0;
     OUT_STRING_PRECIS = 1;
     OUT_CHARACTER_PRECIS = 2;
     OUT_STROKE_PRECIS = 3;
     OUT_TT_PRECIS = 4;
     OUT_DEVICE_PRECIS = 5;
     OUT_RASTER_PRECIS = 6;
     OUT_TT_ONLY_PRECIS = 7;
     OUT_OUTLINE_PRECIS = 8;
     CLIP_DEFAULT_PRECIS = 0;
     CLIP_CHARACTER_PRECIS = 1;
     CLIP_STROKE_PRECIS = 2;
     CLIP_MASK = 15;
     CLIP_LH_ANGLES = 16;
     CLIP_TT_ALWAYS = 32;
     CLIP_EMBEDDED = 128;
     DEFAULT_QUALITY = 0;
     DRAFT_QUALITY = 1;
     PROOF_QUALITY = 2;
     NONANTIALIASED_QUALITY = 3;
     ANTIALIASED_QUALITY = 4;
     DEFAULT_PITCH = 0;
     FIXED_PITCH = 1;
     VARIABLE_PITCH = 2;
     MONO_FONT = 8;
     FF_DECORATIVE = 80;
     FF_DONTCARE = 0;
     FF_MODERN = 48;
     FF_ROMAN = 16;
     FF_SCRIPT = 64;
     FF_SWISS = 32;
  { CreateHatchBrush  }
     HS_BDIAGONAL = 3;
     HS_CROSS = 4;
     HS_DIAGCROSS = 5;
     HS_FDIAGONAL = 2;
     HS_HORIZONTAL = 0;
     HS_VERTICAL = 1;
  { CreateIconFromResourceEx  }
     LR_DEFAULTCOLOR = 0;
     LR_LOADREALSIZE = 128;
  { already defined above !!
  #define LR_MONOCHROME (1)
   }
  { CreateMappedBitmap  }
     CMB_MASKED = 2;
  { CreateNamedPipe  }
     PIPE_ACCESS_DUPLEX = 3;
     PIPE_ACCESS_INBOUND = 1;
     PIPE_ACCESS_OUTBOUND = 2;
     //WRITE_DAC = $40000;     //~winnt, moved to ACCESS_TYPES
     //WRITE_OWNER = $80000;   //~winnt, moved to ACCESS_TYPES
     //ACCESS_SYSTEM_SECURITY = $1000000;  //~winnt, moved to ACCESS_TYPES
     PIPE_TYPE_BYTE = 0;
     PIPE_TYPE_MESSAGE = 4;
     PIPE_READMODE_BYTE = 0;
     PIPE_READMODE_MESSAGE = 2;
     PIPE_WAIT = 0;
     PIPE_NOWAIT = 1;
  { CreatePen, ExtCreatePen  }
     PS_GEOMETRIC = 65536;
     PS_COSMETIC = 0;
     PS_ALTERNATE = 8;
     PS_SOLID = 0;
     PS_DASH = 1;
     PS_DOT = 2;
     PS_DASHDOT = 3;
     PS_DASHDOTDOT = 4;
     PS_NULL = 5;
     PS_USERSTYLE = 7;
     PS_INSIDEFRAME = 6;
     PS_ENDCAP_ROUND = 0;
     PS_ENDCAP_SQUARE = 256;
     PS_ENDCAP_FLAT = 512;
     PS_JOIN_BEVEL = 4096;
     PS_JOIN_MITER = 8192;
     PS_JOIN_ROUND = 0;
     PS_STYLE_MASK = 15;
     PS_ENDCAP_MASK = 3840;
     PS_TYPE_MASK = 983040;
  { CreateProcess  }
     CREATE_DEFAULT_ERROR_MODE = 67108864;
     CREATE_NEW_CONSOLE = 16;
     CREATE_NEW_PROCESS_GROUP = 512;
     CREATE_SEPARATE_WOW_VDM = 2048;
     CREATE_SUSPENDED = 4;
     CREATE_UNICODE_ENVIRONMENT = 1024;
     DEBUG_PROCESS = 1;
     DEBUG_ONLY_THIS_PROCESS = 2;
     DETACHED_PROCESS = 8;
     HIGH_PRIORITY_CLASS = 128;
     IDLE_PRIORITY_CLASS = 64;
     NORMAL_PRIORITY_CLASS = 32;
     REALTIME_PRIORITY_CLASS = 256;
  { CreateService  }
     SERVICE_ALL_ACCESS = $f01ff;
     SERVICE_CHANGE_CONFIG = 2;
     SERVICE_ENUMERATE_DEPENDENTS = 8;
     SERVICE_INTERROGATE = 128;
     SERVICE_PAUSE_CONTINUE = 64;
     SERVICE_QUERY_CONFIG = 1;
     SERVICE_QUERY_STATUS = 4;
     SERVICE_START = 16;
     SERVICE_STOP = 32;
     SERVICE_USER_DEFINED_CONTROL = 256;
     SERVICE_DELETE = $10000;
     SERVICE_READ_CONTROL = $20000;
     SERVICE_GENERIC_EXECUTE = $20000000;
  { already defined above !!
  #define SERVICE_WIN32_OWN_PROCESS     (16)
  #define SERVICE_WIN32_SHARE_PROCESS   (32)
  #define SERVICE_KERNEL_DRIVER (1)
  #define SERVICE_FILE_SYSTEM_DRIVER    (2)
  #define SERVICE_INTERACTIVE_PROCESS   (256)
  #define SERVICE_BOOT_START    (0)
  #define SERVICE_SYSTEM_START  (1)
  #define SERVICE_AUTO_START    (2)
  #define SERVICE_DEMAND_START  (3)
  #define SERVICE_DISABLED      (4)
   }
     SERVICE_ERROR_IGNORE = 0;
     SERVICE_ERROR_NORMAL = 1;
     SERVICE_ERROR_SEVERE = 2;
     SERVICE_ERROR_CRITICAL = 3;
  { CreateTapePartition, WriteTapemark  }
     TAPE_FIXED_PARTITIONS = 0;
     TAPE_INITIATOR_PARTITIONS = $2;
     TAPE_SELECT_PARTITIONS = $1;
     TAPE_FILEMARKS = $1;
     TAPE_LONG_FILEMARKS = $3;
     TAPE_SETMARKS = 0;
     TAPE_SHORT_FILEMARKS = $2;
  { CreateWindow  }
     CW_USEDEFAULT = longint($80000000);
     WS_BORDER = $800000;
     WS_CAPTION = $c00000;
     WS_CHILD = $40000000;
{     WS_CHILDWINDOW = $40000000; // Not declared in WinCE }
     WS_CLIPCHILDREN = $2000000;
     WS_CLIPSIBLINGS = $4000000;
     WS_DISABLED = $8000000;
     WS_DLGFRAME = $400000;
     WS_GROUP = $20000;
     WS_HSCROLL = $100000;
     WS_MAXIMIZEBOX = $20000;
     WS_MINIMIZEBOX = $10000;
     WS_OVERLAPPED = WS_BORDER or WS_CAPTION;
     WS_POPUP = $80000000;
     WS_SIZEBOX = $40000;
     WS_SYSMENU = $80000;
     WS_TABSTOP = $10000;
     WS_THICKFRAME = $40000;
     WS_VISIBLE = $10000000;
     WS_VSCROLL = $200000;

     BS_3STATE = $5;
     BS_AUTO3STATE = $6;
     BS_AUTOCHECKBOX = $3;
     BS_AUTORADIOBUTTON = $9;
     BS_BITMAP = $80;
     BS_BOTTOM = $800;
     BS_CENTER = $300;
     BS_CHECKBOX = $2;
     BS_DEFPUSHBUTTON = $1;
     BS_GROUPBOX = $7;
     BS_ICON = $40;
     BS_LEFT = $100;
     BS_LEFTTEXT = $20;
     BS_MULTILINE = $2000;
     BS_NOTIFY = $4000;
     BS_OWNERDRAW = $b;
     BS_PUSHBUTTON = 0;
     BS_PUSHLIKE = $1000;
     BS_RADIOBUTTON = $4;
     BS_RIGHT = $200;
     BS_RIGHTBUTTON = BS_LEFTTEXT;
     BS_TEXT = 0;
     BS_TOP = $400;
{     BS_USERBUTTON = $8; // Unsupported under WinCE. }
     BS_VCENTER = $c00;
{     BS_FLAT = $8000; // Unsupported under WinCE. }

     CBS_AUTOHSCROLL = $40;
     CBS_DISABLENOSCROLL = $800;
     CBS_DROPDOWN = $2;
     CBS_DROPDOWNLIST = $3;
     CBS_HASSTRINGS = $200;
     CBS_LOWERCASE = $4000;
     CBS_NOINTEGRALHEIGHT = $400;
     CBS_OEMCONVERT = $80;
{     CBS_OWNERDRAWFIXED = $10; // Unsupported under WinCE. }
{     CBS_OWNERDRAWVARIABLE = $20; // Unsupported under WinCE. }
{     CBS_SIMPLE = $1; // Unsupported under WinCE. }
     CBS_SORT = $100;
     CBS_UPPERCASE = $2000;

{ Extended combo box styles }
    CBS_EX_CONSTSTRINGDATA  = $00000002;
{ If you specify the CBS_EX_CONSTSTRINGDATA style when the application 
  inserts a string into the list part of a combo box, the combo box 
  stores the pointer passed to it by the application rather than copying 
  the string. This saves RAM resources when you have a large table of 
  strings in ROM that you want to insert into a combo box. }



{ Edit Control Styles }
     ES_AUTOHSCROLL = $80;
     ES_AUTOVSCROLL = $40;
     ES_CENTER = $1;
     ES_LEFT = 0;
     ES_LOWERCASE = $10;
     ES_MULTILINE = $4;
     ES_NOHIDESEL = $100;
     ES_NUMBER = $2000;
     ES_OEMCONVERT = $400;
     ES_PASSWORD = $20;
     ES_READONLY = $800;
     ES_RIGHT = $2;
     ES_UPPERCASE = $8;
     ES_WANTRETURN = $1000;
     ES_COMBOBOX   = $0200; { Indicates that the edit control is part of a combo box. }

{ List box styles }
     LBS_DISABLENOSCROLL = $1000;
     LBS_EXTENDEDSEL = $800;
     LBS_HASSTRINGS = $40;
     LBS_MULTICOLUMN = $200;
     LBS_MULTIPLESEL = $8;
     LBS_NODATA = $2000;
     LBS_NOINTEGRALHEIGHT = $100;
     LBS_NOREDRAW = $4;
     LBS_NOSEL = $4000;
     LBS_NOTIFY = $1;
     LBS_OWNERDRAWFIXED = $10;
     LBS_OWNERDRAWVARIABLE = $20;
     LBS_SORT = $2;
//     LBS_STANDARD = $a00003;
     LBS_USETABSTOPS = $80;
     LBS_WANTKEYBOARDINPUT = $400;
     LBS_STANDARD = LBS_NOTIFY or LBS_SORT or WS_VSCROLL or WS_BORDER;

{ Extended list box styles }
     LBS_EX_CONSTSTRINGDATA   = $00000002;
{ Windows CE supports the LBS_EX_CONSTSTRINGDATA style that saves RAM 
  resources when you have a large table of strings in ROM that you want 
  to insert into a list box. }

{ Scroll bar styles }
{     SBS_BOTTOMALIGN = $4; // Unsupported under WinCE. }
     SBS_HORZ = 0;
{     SBS_LEFTALIGN = $2; // Unsupported under WinCE. }
{     SBS_RIGHTALIGN = $4; // Unsupported under WinCE. }
{     SBS_SIZEBOX = $8; // Unsupported under WinCE. }
{     SBS_SIZEBOXBOTTOMRIGHTALIGN = $4; // Unsupported under WinCE. }
{     SBS_SIZEBOXTOPLEFTALIGN = $2; // Unsupported under WinCE. }
{     SBS_SIZEGRIP = $10; // Unsupported under WinCE. }
{     SBS_TOPALIGN = $2; // Unsupported under WinCE. }
     SBS_VERT = $1;

{ Static control styles }
     SS_BITMAP = $e;
{     SS_BLACKFRAME = $7; // Unsupported under WinCE. }
{     SS_BLACKRECT = $4; // Unsupported under WinCE. }
     SS_CENTER = $1;
     SS_CENTERIMAGE = $200;
{     SS_ENHMETAFILE = $f; // Unsupported under WinCE. }
{     SS_ETCHEDFRAME = $12; // Unsupported under WinCE. }
{     SS_ETCHEDHORZ = $10; // Unsupported under WinCE. }
{     SS_ETCHEDVERT = $11; // Unsupported under WinCE. }
{     SS_GRAYFRAME = $8; // Unsupported under WinCE. }
{     SS_GRAYRECT = $5; // Unsupported under WinCE. }
     SS_ICON = $3;
     SS_LEFT = 0;
     SS_LEFTNOWORDWRAP = $c;
     SS_NOPREFIX = $80;
     SS_NOTIFY = $100;
{     SS_OWNERDRAW = $d; // Unsupported under WinCE. }
{     SS_REALSIZEIMAGE = $800; // Unsupported under WinCE. }
     SS_RIGHT = $2;
{     SS_RIGHTJUST = $400; // Unsupported under WinCE. }
{     SS_SIMPLE = $b; // Unsupported under WinCE. }
{     SS_SUNKEN = $1000; // Unsupported under WinCE. }
{     SS_USERITEM = $a; // Unsupported under WinCE. }
{     SS_WHITEFRAME = $9; // Unsupported under WinCE. }
{     SS_WHITERECT = $6; // Unsupported under WinCE. }

    { Dialog styles }
{     DS_ABSALIGN      = $1; // Unsupported under WinCE. }
{     DS_SYSMODAL      = $2; // Unsupported under WinCE. }
     DS_3DLOOK        = $4;
{     DS_FIXEDSYS      = $8; // Unsupported under WinCE. }
{     DS_NOFAILCREATE  = $10; // Unsupported under WinCE. }
     DS_LOCALEDIT     = $20; { Edit items get Local storage. }
     DS_SETFONT       = $40; { User specified font for Dlg controls. }
     DS_MODALFRAME    = $80; { Can be combined with WS_CAPTION. }
{     DS_NOIDLEMSG     = $100; // Unsupported under WinCE. }
     DS_SETFOREGROUND = $200; { not in Windows 3.1 }
     DS_CONTROL       = $400;
     DS_CENTER        = $800;
{     DS_CENTERMOUSE   = $1000; // Unsupported under WinCE. }
{     DS_CONTEXTHELP   = $2000; // Unsupported under WinCE. }
     DS_NOCLOSEBTN    = $4000;

  { CreateWindowEx  }
{     WS_EX_LEFT             = 0; // Not declared. }
{     WS_EX_LTRREADING       = 0; // Not declared. }
{     WS_EX_RIGHTSCROLLBAR   = 0; // Not declared. }
     WS_EX_DLGMODALFRAME    = $1;
{     WS_EX_NOPARENTNOTIFY   = $4; // Unsupported under WinCE. }
     WS_EX_TOPMOST          = $8;
{     WS_EX_ACCEPTFILES      = $10; // Unsupported under WinCE. }
{     WS_EX_TRANSPARENT      = $20; // Unsupported under WinCE. }
{     WS_EX_MDICHILD         = $40; // Unsupported under WinCE. }
     WS_EX_TOOLWINDOW       = $80;
{     WS_EX_PALETTEWINDOW    = $188; // Unsupported under WinCE. }
     WS_EX_WINDOWEDGE       = $100;
     WS_EX_CLIENTEDGE       = $200;
     WS_EX_OVERLAPPEDWINDOW = WS_EX_WINDOWEDGE or WS_EX_CLIENTEDGE;
     WS_EX_CONTEXTHELP      = $400;
     WS_EX_RIGHT            = $1000;
     WS_EX_RTLREADING       = $2000;
     WS_EX_LEFTSCROLLBAR    = $4000;
{     WS_EX_CONTROLPARENT    = $10000; // Unsupported under WinCE. }
     WS_EX_STATICEDGE       = $20000;
{     WS_EX_APPWINDOW        = $40000; // Unsupported under WinCE. }
     WS_EX_NOANIMATION      = $4000000;
     WS_EX_NOACTIVATE       = $8000000;
     WS_EX_INK              = $10000000;
     WS_EX_ABOVESTARTUP     = $20000000;
     WS_EX_NODRAG           = $40000000;
     WS_EX_CAPTIONOKBTN     = $80000000;

     WS_EX_NOINHERITLAYOUT  = $00100000; // Disable inheritence of mirroring by children
     WS_EX_LAYOUTRTL        = $00400000; // Right to left mirroring
     

  { CreateWindowStation  }
     WINSTA_ACCESSCLIPBOARD = $4;
     WINSTA_ACCESSGLOBALATOMS = $20;
     WINSTA_CREATEDESKTOP = $8;
     WINSTA_ENUMDESKTOPS = $1;
     WINSTA_ENUMERATE = $100;
     WINSTA_EXITWINDOWS = $40;
     WINSTA_READATTRIBUTES = $2;
     WINSTA_READSCREEN = $200;
     WINSTA_WRITEATTRIBUTES = $10;
  { DdeCallback  }
  { DdeClientTransaction  }
  { DdeEnableCallback  }
  { DdeGetLastError  }
  { DdeInitialize  }
  { DdeNameService  }
  { DebugProc  }
     WH_CALLWNDPROC = 4;
     WH_CALLWNDPROCRET = 12;
     WH_CBT = 5;
     WH_DEBUG = 9;
     WH_GETMESSAGE = 3;
     WH_JOURNALPLAYBACK = 1;
     WH_JOURNALRECORD = 0;
     WH_KEYBOARD = 2;
     WH_MOUSE = 7;
     WH_MSGFILTER = -(1);
     WH_SHELL = 10;
     WH_SYSMSGFILTER = 6;
  { already defined above !!
  #define WH_MSGFILTER  (-1)  }
     WH_FOREGROUNDIDLE = 11;
  { DefineDosDevice  }
     DDD_RAW_TARGET_PATH = 1;
     DDD_REMOVE_DEFINITION = 2;
     DDD_EXACT_MATCH_ON_REMOVE = 4;
  { DeviceCapbilities  }
     DC_BINNAMES = 12;          //windef
     DC_BINS = 6;               //windef
     DC_COPIES = 18;            //windef
     DC_DRIVER = 11;            //windef
     DC_DATATYPE_PRODUCED = 21;
     DC_DUPLEX = 7;             //windef
     DC_EMF_COMPLIANT = 20;
     DC_ENUMRESOLUTIONS = 13;   //windef
     DC_EXTRA = 9;              //windef
     DC_FIELDS = 1;             //windef
     DC_FILEDEPENDENCIES = 14;  //windef
     DC_MAXEXTENT = 5;          //windef
     DC_MINEXTENT = 4;          //windef
     DC_ORIENTATION = 17;       //windef
     DC_PAPERNAMES = 16;        //windef
     DC_PAPERS = 2;             //windef
     DC_PAPERSIZE = 3;          //windef
     DC_SIZE = 8;               //windef
     DC_TRUETYPE = 15;          //windef
     DCTT_BITMAP = $1;
     DCTT_DOWNLOAD = $2;
     DCTT_SUBDEV = $4;
     DC_VERSION = 10;           //windef
     DC_BINADJUST = 19;
  { already defined above !!
  #define DC_DATATYPE_PRODUCED  (21)
   }
  { DeviceIoControl  }
  { DlgDirList  }
     DDL_ARCHIVE = 32;
     DDL_DIRECTORY = 16;
     DDL_DRIVES = 16384;
     DDL_EXCLUSIVE = 32768;
     DDL_HIDDEN = 2;
     DDL_READONLY = 1;
     DDL_READWRITE = 0;
     DDL_SYSTEM = 4;
     DDL_POSTMSGS = 8192;
  { DllEntryPoint  }
     DLL_PROCESS_ATTACH = 1;
     DLL_THREAD_ATTACH = 2;
     DLL_PROCESS_DETACH = 0;
     DLL_THREAD_DETACH = 3;
  { DocumentProperties  }
     DM_IN_BUFFER = 8;   //windef
     DM_MODIFY = 8;      //windef
     DM_IN_PROMPT = 4;   //windef
     DM_PROMPT = 4;      //windef
     DM_OUT_BUFFER = 2;  //windef
     DM_COPY = 2;        //windef
     DM_UPDATE = 1;      //windef
  { DrawAnimatedRects  }
     IDANI_OPEN = 1;
     IDANI_CLOSE = 2;
  { DrawCaption  }
     DC_ACTIVE = 1;
     DC_SMALLCAP = 2;
  { DrawEdge  }
     BDR_RAISEDINNER = 4;
     BDR_SUNKENINNER = 8;
     BDR_RAISEDOUTER = 1;
     BDR_SUNKENOUTER = 1;
     EDGE_BUMP = 9;
     EDGE_ETCHED = 6;
     EDGE_RAISED = 5;
     EDGE_SUNKEN = 10;
     BF_ADJUST = 8192;
     BF_BOTTOM = 8;
     BF_BOTTOMLEFT = 9;
     BF_BOTTOMRIGHT = 12;
     BF_DIAGONAL = 16;
     BF_DIAGONAL_ENDBOTTOMLEFT = 25;
     BF_DIAGONAL_ENDBOTTOMRIGHT = 28;
     BF_DIAGONAL_ENDTOPLEFT = 19;
     BF_DIAGONAL_ENDTOPRIGHT = 22;
     BF_FLAT = 16384;
     BF_LEFT = 1;
     BF_MIDDLE = 2048;
     BF_MONO = 32768;
     BF_RECT = 15;
     BF_RIGHT = 4;
     BF_SOFT = 4096;
     BF_TOP = 2;
     BF_TOPLEFT = 3;
     BF_TOPRIGHT = 6;
  { DrawFrameControl  }
     DFC_BUTTON = 4;
     DFC_CAPTION = 1;
     DFC_MENU = 2;
     DFC_SCROLL = 3;
     DFCS_BUTTON3STATE = 8;
     DFCS_BUTTONCHECK = 0;
     DFCS_BUTTONPUSH = 16;
     DFCS_BUTTONRADIO = 4;
     DFCS_BUTTONRADIOIMAGE = 1;
     DFCS_BUTTONRADIOMASK = 2;
     DFCS_CAPTIONCLOSE = 0;
     DFCS_CAPTIONHELP = 4;
     DFCS_CAPTIONMAX = 2;
     DFCS_CAPTIONMIN = 1;
     DFCS_CAPTIONRESTORE = 3;
     DFCS_MENUARROW = 0;
     DFCS_MENUBULLET = 2;
     DFCS_MENUCHECK = 1;
     DFCS_SCROLLCOMBOBOX = 5;
     DFCS_SCROLLDOWN = 1;
     DFCS_SCROLLLEFT = 2;
     DFCS_SCROLLRIGHT = 3;
     DFCS_SCROLLSIZEGRIP = 8;
     DFCS_SCROLLUP = 0;
     DFCS_ADJUSTRECT = 8192;
     DFCS_CHECKED = 1024;
     DFCS_FLAT = 16384;
     DFCS_INACTIVE = 256;
     DFCS_MONO = 32768;
     DFCS_PUSHED = 512;
  { DrawIconEx  }
     DI_COMPAT = 4;
     DI_DEFAULTSIZE = 8;
     DI_IMAGE = 2;
     DI_MASK = 1;
     DI_NORMAL = 3;
  { DrawState  }
     DST_BITMAP = 4;
     DST_COMPLEX = 0;
     DST_ICON = 3;
     DST_PREFIXTEXT = 2;
     DST_TEXT = 1;
     DSS_NORMAL = 0;
     DSS_UNION = 16;
     DSS_DISABLED = 32;
     DSS_MONO = 128;
  { DrawStatusText  }
     SBT_NOBORDERS = 256;
     SBT_OWNERDRAW = 4096;
     SBT_POPOUT = 512;
     SBT_RTLREADING = 1024;
  { DrawText, DrawTextEx  }
     DT_BOTTOM = 8;
     DT_CALCRECT = 1024;
     DT_CENTER = 1;
     DT_EDITCONTROL = 8192;
     DT_END_ELLIPSIS = 32768;
     DT_PATH_ELLIPSIS = 16384;
     DT_EXPANDTABS = 64;
     DT_EXTERNALLEADING = 512;
     DT_LEFT = 0;
     DT_MODIFYSTRING = 65536;
     DT_NOCLIP = 256;
     DT_NOPREFIX = 2048;
     DT_RIGHT = 2;
     DT_RTLREADING = 131072;
     DT_SINGLELINE = 32;
     DT_TABSTOP = 128;
     DT_TOP = 0;
     DT_VCENTER = 4;
     DT_WORDBREAK = 16;
     DT_INTERNAL = 4096;
  { DuplicateHandle, MapViewOfFile  }
     DUPLICATE_CLOSE_SOURCE = 1;
     DUPLICATE_SAME_ACCESS = 2;
     FILE_MAP_ALL_ACCESS = $f001f;
     FILE_MAP_READ = 4;
     FILE_MAP_WRITE = 2;
     FILE_MAP_COPY = 1;
     MUTEX_ALL_ACCESS = $1f0001;
     MUTEX_MODIFY_STATE = 1;
     //SYNCHRONIZE = $100000;     //~winnt, move to ACCESS_TYPE
     SEMAPHORE_ALL_ACCESS = $1f0003;
     SEMAPHORE_MODIFY_STATE = 2;
     EVENT_ALL_ACCESS = $1f0003;
     EVENT_MODIFY_STATE = 2;
     KEY_ALL_ACCESS = $f003f;
     KEY_CREATE_LINK = 32;
     KEY_CREATE_SUB_KEY = 4;
     KEY_ENUMERATE_SUB_KEYS = 8;
     KEY_EXECUTE = $20019;
     KEY_NOTIFY = 16;
     KEY_QUERY_VALUE = 1;
     KEY_READ = $20019;
     KEY_SET_VALUE = 2;
     KEY_WRITE = $20006;
     PROCESS_ALL_ACCESS = $1f0fff;
     PROCESS_CREATE_PROCESS = 128;
     PROCESS_CREATE_THREAD = 2;
     PROCESS_DUP_HANDLE = 64;
     PROCESS_QUERY_INFORMATION = 1024;
     PROCESS_SET_INFORMATION = 512;
     PROCESS_TERMINATE = 1;
     PROCESS_VM_OPERATION = 8;
     PROCESS_VM_READ = 16;
     PROCESS_VM_WRITE = 32;
     THREAD_ALL_ACCESS = $1f03ff;
     THREAD_DIRECT_IMPERSONATION = 512;
     THREAD_GET_CONTEXT = 8;
     THREAD_IMPERSONATE = 256;
     THREAD_QUERY_INFORMATION = 64;
     THREAD_SET_CONTEXT = 16;
     THREAD_SET_INFORMATION = 32;
     THREAD_SET_THREAD_TOKEN = 128;
     THREAD_SUSPEND_RESUME = 2;
     THREAD_TERMINATE = 1;
  { EditWordBreakProc  }
     WB_ISDELIMITER = 2;
     WB_LEFT = 0;
     WB_RIGHT = 1;
  { EnableScrollBar  }
     SB_BOTH = 3;
     SB_CTL = 2;
     SB_HORZ = 0;
     SB_VERT = 1;
     ESB_DISABLE_BOTH = 3;
     ESB_DISABLE_DOWN = 2;
     ESB_DISABLE_LEFT = 1;
     ESB_DISABLE_LTUP = 1;
     ESB_DISABLE_RIGHT = 2;
     ESB_DISABLE_RTDN = 2;
     ESB_DISABLE_UP = 1;
     ESB_ENABLE_BOTH = 0;
  { Scroll Bar notifications }
     SB_LINEUP = 0;
     SB_LINEDOWN = 1;
     SB_LINELEFT = 0;
     SB_LINERIGHT = 1;
     SB_PAGEUP = 2;
     SB_PAGEDOWN = 3;
     SB_PAGELEFT = 2;
     SB_PAGERIGHT = 3;
     SB_THUMBPOSITION = 4;
     SB_THUMBTRACK = 5;
     SB_ENDSCROLL = 8;
     SB_LEFT = 6;
     SB_RIGHT = 7;
     SB_BOTTOM = 7;
     SB_TOP = 6;
  { EnumCalendarInfo  }
     ENUM_ALL_CALENDARS = -(1);
  { EnumDateFormats  }
     DATE_SHORTDATE = 1;
     DATE_LONGDATE = 2;
  { EnumDependentServices  }
     SERVICE_ACTIVE = 1;
     SERVICE_INACTIVE = 2;
  { EnumFontFamExProc  }
     DEVICE_FONTTYPE = 2;
     RASTER_FONTTYPE = 1;
     TRUETYPE_FONTTYPE = 4;
  { EnumObjects, GetCurrentObject, GetObjectType  }
     OBJ_BRUSH = 2;
     OBJ_PEN = 1;
     OBJ_PAL = 5;
     OBJ_FONT = 6;
     OBJ_BITMAP = 7;
     OBJ_EXTPEN = 11;
     OBJ_REGION = 8;
     OBJ_DC = 3;
     OBJ_MEMDC = 10;
     OBJ_METAFILE = 9;
     OBJ_METADC = 4;
     OBJ_ENHMETAFILE = 13;
     OBJ_ENHMETADC = 12;
  { EnumPrinters  }
  { EnumProtocols  }
  { EnumResLangProc  }
     RT_ACCELERATOR = MAKEINTRESOURCE(9);
     RT_BITMAP = MAKEINTRESOURCE(2);
     RT_DIALOG = MAKEINTRESOURCE(5);
     RT_FONT = MAKEINTRESOURCE(8);
     RT_FONTDIR = MAKEINTRESOURCE(7);
     RT_MENU = MAKEINTRESOURCE(4);
     RT_RCDATA = MAKEINTRESOURCE(10);
     RT_STRING = MAKEINTRESOURCE(6);
     RT_MESSAGETABLE = MAKEINTRESOURCE(11);
     RT_CURSOR = MAKEINTRESOURCE(1);
     RT_GROUP_CURSOR = MAKEINTRESOURCE(12);
     RT_ICON = MAKEINTRESOURCE(3);
     RT_GROUP_ICON = MAKEINTRESOURCE(13);
     RT_VERSION = MAKEINTRESOURCE(16);

  { EnumServicesStatus  }

  const
     SERVICE_WIN32 = 48;
     SERVICE_DRIVER = 11;
  { EnumSystemCodePages  }
     CP_INSTALLED = 1;
     CP_SUPPORTED = 2;
  { EnumSystemLocales  }
     LCID_INSTALLED = 1;
     LCID_SUPPORTED = 2;
  { EraseTape  }
     TAPE_ERASE_LONG = $1;
     TAPE_ERASE_SHORT = 0;
  { Escape  }
     SP_ERROR = -(1);
     SP_OUTOFDISK = -(4);
     SP_OUTOFMEMORY = -(5);
     SP_USERABORT = -(3);
     PHYSICALWIDTH = 110;
     PHYSICALHEIGHT = 111;
     PHYSICALOFFSETX = 112;
     PHYSICALOFFSETY = 113;
     SCALINGFACTORX = 114;
     SCALINGFACTORY = 115;
     QUERYESCSUPPORT = 8;
     {ABORTDOC = 2; conflicts with AbortDoc function }
     cABORTDOC = 2;
     {ENDDOC = 11; conflicts with AbortDoc function }
     cENDDOC = 11;
     GETPHYSPAGESIZE = 12;
     GETPRINTINGOFFSET = 13;
     GETSCALINGFACTOR = 14;
     NEWFRAME = 1;
     NEXTBAND = 3;
     PASSTHROUGH = 19;
     {SETABORTPROC = 9; conflicts with AbortDoc function }
     cSETABORTPROC = 9;
     {STARTDOC = 10; conflicts with AbortDoc function }
     cSTARTDOC = 10;
  { EscapeCommFunction  }
     CLRDTR = 6;          //winbase
     CLRRTS = 4;          //winbase
     SETDTR = 5;          //winbase
     SETRTS = 3;          //winbase 
     SETXOFF = 1;         //winbase
     SETXON = 2;          //winbase
     SETBREAK = 8;          //winbase
     CLRBREAK = 9;                                //winbase
     // Some CE devices share a UART between an IRDA port and a serial port.
     // These escape functions allow control over the mode.
     SETIR = 10;  // Set the port to IR mode.     //+winbase
     CLRIR = 11;  // Set the port to non-IR mode. //+winbase
  { ExitWindowsEx  }
     EWX_FORCE = 4;
     EWX_LOGOFF = 0;
     EWX_POWEROFF = 8;
     EWX_REBOOT = 2;
     EWX_SHUTDOWN = 1;
  { ExtFloodFill  }
     FLOODFILLBORDER = 0;
     FLOODFILLSURFACE = 1;
  { ExtTextOut  }
     ETO_CLIPPED = 4;
     ETO_GLYPH_INDEX = 16;
     ETO_OPAQUE = 2;
     ETO_RTLREADING = 128;
  { FindFirstChangeNotification  }
     FILE_NOTIFY_CHANGE_FILE_NAME = 1;
     FILE_NOTIFY_CHANGE_DIR_NAME = 2;
     FILE_NOTIFY_CHANGE_ATTRIBUTES = 4;
     FILE_NOTIFY_CHANGE_SIZE = 8;
     FILE_NOTIFY_CHANGE_LAST_WRITE = 16;
     FILE_NOTIFY_CHANGE_SECURITY = 256;
  { FindFirstPrinterChangeNotification  }
  { FindNextPrinterNotification  }
  { FMExtensionProc  }
  { FoldString  }
     MAP_FOLDCZONE = 16;
     MAP_FOLDDIGITS = 128;
     MAP_PRECOMPOSED = 32;
     MAP_COMPOSITE = 64;
  { ForegroundIdleProc  }
     HC_ACTION = 0;
  { FormatMessage  }
     FORMAT_MESSAGE_ALLOCATE_BUFFER = 256;
     FORMAT_MESSAGE_IGNORE_INSERTS = 512;
     FORMAT_MESSAGE_FROM_STRING = 1024;
     FORMAT_MESSAGE_FROM_HMODULE = 2048;
     FORMAT_MESSAGE_FROM_SYSTEM = 4096;
     FORMAT_MESSAGE_ARGUMENT_ARRAY = 8192;
     FORMAT_MESSAGE_MAX_WIDTH_MASK = 255;
  { GdiComment  }
     GDICOMMENT_WINDOWS_METAFILE = -(2147483647);
     GDICOMMENT_BEGINGROUP = 2;
     GDICOMMENT_ENDGROUP = 3;
     GDICOMMENT_MULTIFORMATS = 1073741828;
     GDICOMMENT_IDENTIFIER = 1128875079;
  { GenerateConsoleCtrlEvent, HandlerRoutine  }
     CTRL_C_EVENT = 0;
     CTRL_BREAK_EVENT = 1;
     CTRL_CLOSE_EVENT = 2;
     CTRL_LOGOFF_EVENT = 5;
     CTRL_SHUTDOWN_EVENT = 6;
  { GetAddressByName  }
  { GetArcDirection  }
     AD_COUNTERCLOCKWISE = 1;
     AD_CLOCKWISE = 2;
  { GetClassLong, GetClassWord  }
     GCW_ATOM = -(32);
     GCL_CBCLSEXTRA = -(20);
     GCL_CBWNDEXTRA = -(18);
     GCL_HBRBACKGROUND = -(10);
     GCL_HCURSOR = -(12);
     GCL_HICON = -(14);
     GCL_HICONSM = -(34);
     GCL_HMODULE = -(16);
     GCL_MENUNAME = -(8);
     GCL_STYLE = -(26);
     GCL_WNDPROC = -(24);
  { GetClipboardFormat, SetClipboardData  }
     CF_BITMAP = 2;
     CF_DIB = 8;
     CF_PALETTE = 9;
     CF_ENHMETAFILE = 14;
     CF_METAFILEPICT = 3;
     CF_OEMTEXT = 7;
     CF_TEXT = 1;
     CF_UNICODETEXT = 13;
     CF_DIF = 5;
     CF_DSPBITMAP = 130;
     CF_DSPENHMETAFILE = 142;
     CF_DSPMETAFILEPICT = 131;
     CF_DSPTEXT = 129;
     CF_GDIOBJFIRST = 768;
     CF_GDIOBJLAST = 1023;
     CF_HDROP = 15;
     CF_LOCALE = 16;
     CF_OWNERDISPLAY = 128;
     CF_PENDATA = 10;
     CF_PRIVATEFIRST = 512;
     CF_PRIVATELAST = 767;
     CF_RIFF = 11;
     CF_SYLK = 4;
     CF_WAVE = 12;
     CF_TIFF = 6;
  { GetCommMask  }
     EV_BREAK = 64;
     EV_CTS = 8;
     EV_DSR = 16;
     EV_ERR = 128;
     EV_EVENT1 = 2048;
     EV_EVENT2 = 4096;
     EV_PERR = 512;
     EV_RING = 256;
     EV_RLSD = 32;
     EV_RX80FULL = 1024;
     EV_RXCHAR = 1;
     EV_RXFLAG = 2;
     EV_TXEMPTY = 4;
     EV_POWER = $2000;  // WINCE Power event.  //+winbase

  { GetCommModemStatus  }
     MS_CTS_ON = $10;
     MS_DSR_ON = $20;
     MS_RING_ON = $40;
     MS_RLSD_ON = $80;
  { GetComputerName  }
     MAX_COMPUTERNAME_LENGTH = 15;
  { GetCPInfo  }
     CP_ACP = 0;
     CP_MACCP = 2;
     CP_OEMCP = 1;
     { WinCE 5.0+}
     CP_UTF7 = 65000;
     CP_UTF8 = 65001;
     
  { GetDateFormat  }
  { already defined above !!
  #define DATE_SHORTDATE        (1)
  #define DATE_LONGDATE (2)
   }
     DATE_USE_ALT_CALENDAR = 4;
  { GetDCEx  }
     DCX_WINDOW = $1;
     DCX_CACHE = $2;
     DCX_PARENTCLIP = $20;
     DCX_CLIPSIBLINGS = $10;
     DCX_CLIPCHILDREN = $8;
     DCX_NORESETATTRS = $4;
     DCX_LOCKWINDOWUPDATE = $400;
     DCX_EXCLUDERGN = $40;
     DCX_INTERSECTRGN = $80;
     DCX_VALIDATE = $200000;
  { GetDeviceCaps  }
     DRIVERVERSION = 0;
     TECHNOLOGY = 2;
     DT_PLOTTER = 0;
     DT_RASDISPLAY = 1;
     DT_RASPRINTER = 2;
     DT_RASCAMERA = 3;
     DT_CHARSTREAM = 4;
     DT_METAFILE = 5;
     DT_DISPFILE = 6;
     HORZSIZE = 4;
     VERTSIZE = 6;
     HORZRES = 8;
     VERTRES = 10;
     LOGPIXELSX = 88;
     LOGPIXELSY = 90;
     BITSPIXEL = 12;
     PLANES = 14;
     NUMBRUSHES = 16;
     NUMPENS = 18;
     NUMFONTS = 22;
     NUMCOLORS = 24;
     ASPECTX = 40;
     ASPECTY = 42;
     ASPECTXY = 44;
     PDEVICESIZE = 26;
     CLIPCAPS = 36;
     SIZEPALETTE = 104;
     NUMRESERVED = 106;
     COLORRES = 108;
  { already defined above !!
  #define PHYSICALWIDTH (110)
  #define PHYSICALHEIGHT        (111)
  #define PHYSICALOFFSETX       (112)
  #define PHYSICALOFFSETY       (113)
  #define SCALINGFACTORX        (114)
  #define SCALINGFACTORY        (115)
   }
     VREFRESH = 116;
     DESKTOPHORZRES = 118;
     DESKTOPVERTRES = 117;
     BLTALIGNMENT = 119;
     RASTERCAPS = 38;
     RC_BANDING = 2;
     RC_BITBLT = 1;
     RC_BITMAP64 = 8;
     RC_DI_BITMAP = 128;
     RC_DIBTODEV = 512;
     RC_FLOODFILL = 4096;
     RC_GDI20_OUTPUT = 16;
     RC_PALETTE = 256;
     RC_SCALING = 4;
     RC_STRETCHBLT = 2048;
     RC_STRETCHDIB = 8192;
     CURVECAPS = 28;
     CC_NONE = 0;
     CC_CIRCLES = 1;
     CC_PIE = 2;
     CC_CHORD = 4;
     CC_ELLIPSES = 8;
     CC_WIDE = 16;
     CC_STYLED = 32;
     CC_WIDESTYLED = 64;
     CC_INTERIORS = 128;
     CC_ROUNDRECT = 256;
     LINECAPS = 30;
     LC_NONE = 0;
     LC_POLYLINE = 2;
     LC_MARKER = 4;
     LC_POLYMARKER = 8;
     LC_WIDE = 16;
     LC_STYLED = 32;
     LC_WIDESTYLED = 64;
     LC_INTERIORS = 128;
     POLYGONALCAPS = 32;
     PC_NONE = 0;
     PC_POLYGON = 1;
     PC_RECTANGLE = 2;
     PC_WINDPOLYGON = 4;
     PC_SCANLINE = 8;
     PC_WIDE = 16;
     PC_STYLED = 32;
     PC_WIDESTYLED = 64;
     PC_INTERIORS = 128;
     TEXTCAPS = 34;
     TC_OP_CHARACTER = 1;
     TC_OP_STROKE = 2;
     TC_CP_STROKE = 4;
     TC_CR_90 = 8;
     TC_CR_ANY = 16;
     TC_SF_X_YINDEP = 32;
     TC_SA_DOUBLE = 64;
     TC_SA_INTEGER = 128;
     TC_SA_CONTIN = 256;
     TC_EA_DOUBLE = 512;
     TC_IA_ABLE = 1024;
     TC_UA_ABLE = 2048;
     TC_SO_ABLE = 4096;
     TC_RA_ABLE = 8192;
     TC_VA_ABLE = 16384;
     TC_RESERVED = 32768;
     TC_SCROLLBLT = 65536;
     PC_PATHS = 512;
  { GetDriveType  }
     DRIVE_REMOVABLE = 2;
     DRIVE_FIXED = 3;
     DRIVE_REMOTE = 4;
     DRIVE_CDROM = 5;
     DRIVE_RAMDISK = 6;
     DRIVE_UNKNOWN = 0;
     DRIVE_NO_ROOT_DIR = 1;
  { GetExceptionCode  }
     EXCEPTION_ACCESS_VIOLATION = $c0000005;
     EXCEPTION_BREAKPOINT = $80000003;
     EXCEPTION_DATATYPE_MISALIGNMENT = $80000002;
     EXCEPTION_SINGLE_STEP = $80000004;
     EXCEPTION_ARRAY_BOUNDS_EXCEEDED = $c000008c;
     EXCEPTION_FLT_DENORMAL_OPERAND = $c000008d;
     EXCEPTION_FLT_DIVIDE_BY_ZERO = $c000008e;
     EXCEPTION_FLT_INEXACT_RESULT = $c000008f;
     EXCEPTION_FLT_INVALID_OPERATION = $c0000090;
     EXCEPTION_FLT_OVERFLOW = $c0000091;
     EXCEPTION_FLT_STACK_CHECK = $c0000092;
     EXCEPTION_FLT_UNDERFLOW = $c0000093;
     EXCEPTION_INT_DIVIDE_BY_ZERO = $c0000094;
     EXCEPTION_INT_OVERFLOW = $c0000095;
     EXCEPTION_INVALID_HANDLE = $c0000008;
     EXCEPTION_PRIV_INSTRUCTION = $c0000096;
     EXCEPTION_NONCONTINUABLE_EXCEPTION = $c0000025;
     EXCEPTION_NONCONTINUABLE = $1;                  //winnt
     EXCEPTION_UNWINDING = $2;                       //+winnt
     EXCEPTION_EXIT_UNWIND = $4;                     //+winnt
     EXCEPTION_STACK_INVALID = $8;                   //+winnt
     EXCEPTION_NESTED_CALL = $10;                    //+winnt
     EXCEPTION_TARGET_UNWIND = $20;                  //+winnt
     EXCEPTION_COLLIDED_UNWIND = $40;                //+winnt
     EXCEPTION_UNWIND = EXCEPTION_UNWINDING or EXCEPTION_EXIT_UNWIND or
                        EXCEPTION_TARGET_UNWIND or EXCEPTION_COLLIDED_UNWIND; //+winnt
     EXCEPTION_STACK_OVERFLOW = $c00000fd;
     EXCEPTION_INVALID_DISPOSITION = $c0000026;
     EXCEPTION_IN_PAGE_ERROR = $c0000006;
     EXCEPTION_ILLEGAL_INSTRUCTION = $c000001d;
     EXCEPTION_POSSIBLE_DEADLOCK = $c0000194;

     function IS_UNWINDING( Flag : Longint) : boolean;     //+winnt
     function IS_DISPATCHING( Flag : Longint) : boolean;   //+winnt
     function IS_TARGET_UNWIND( Flag : Longint) : Longint; //+winnt

  { GetFileType  }
  const
     FILE_TYPE_UNKNOWN = 0;
     FILE_TYPE_DISK = 1;
     FILE_TYPE_CHAR = 2;
     FILE_TYPE_PIPE = 3;
     GDI_ERROR = $ffffffff;
  { GetGraphicsMode  }
     GM_COMPATIBLE = 1;
     GM_ADVANCED = 2;
  { GetIconInfo  }
     IDC_ARROW = MAKEINTRESOURCE(32512);
     IDC_IBEAM = MAKEINTRESOURCE(32513);
     IDC_WAIT = MAKEINTRESOURCE(32514);
     IDC_CROSS = MAKEINTRESOURCE(32515);
     IDC_UPARROW = MAKEINTRESOURCE(32516);
     IDC_SIZENWSE = MAKEINTRESOURCE(32642);
     IDC_SIZENESW = MAKEINTRESOURCE(32643);
     IDC_SIZEWE = MAKEINTRESOURCE(32644);
     IDC_SIZENS = MAKEINTRESOURCE(32645);
     IDC_SIZEALL = MAKEINTRESOURCE(32646);
     IDC_NO = MAKEINTRESOURCE(32648);
     IDC_APPSTARTING = MAKEINTRESOURCE(32650);
     IDC_HELP = MAKEINTRESOURCE(32651);
     IDI_APPLICATION = MAKEINTRESOURCE(32512);
     IDI_HAND = MAKEINTRESOURCE(32513);
     IDI_QUESTION = MAKEINTRESOURCE(32514);
     IDI_EXCLAMATION = MAKEINTRESOURCE(32515);
     IDI_ASTERISK = MAKEINTRESOURCE(32516);
     IDI_WINLOGO = MAKEINTRESOURCE(32517);
     IDC_SIZE = MAKEINTRESOURCE(32640);
     IDC_ICON = MAKEINTRESOURCE(32641);
     IDC_HAND = MAKEINTRESOURCE(32649);

  { GetMapMode  }

  const
     MM_ANISOTROPIC = 8;
     MM_HIENGLISH = 5;
     MM_HIMETRIC = 3;
     MM_ISOTROPIC = 7;
     MM_LOENGLISH = 4;
     MM_LOMETRIC = 2;
     MM_TEXT = 1;
     MM_TWIPS = 6;
  { GetMenuDefaultItem  }
     GMDI_GOINTOPOPUPS = $2;
     GMDI_USEDISABLED = $1;
  { PeekMessage  }
     PM_NOREMOVE = 0;
     PM_REMOVE = 1;
     PM_NOYIELD = 2;
  { GetNextWindow, GetWindow  }
     GW_HWNDNEXT = 2;
     GW_HWNDPREV = 3;
     GW_CHILD = 5;
     GW_HWNDFIRST = 0;
     GW_HWNDLAST = 1;
     GW_OWNER = 4;
     GW_MAX = 5; //+winuser

  { GetQueueStatus  }
     QS_ALLEVENTS = 191;
     QS_ALLINPUT = 255;
     QS_HOTKEY = 128;
     QS_INPUT = 7;
     QS_KEY = 1;
     QS_MOUSE = 6;
     QS_MOUSEBUTTON = 4;
     QS_MOUSEMOVE = 2;
     QS_PAINT = 32;
     QS_POSTMESSAGE = 8;
     QS_SENDMESSAGE = 64;
     QS_TIMER = 16;
  { GetScrollInfo, SetScrollInfo  }
{     SIF_ALL = 23; }
     SIF_PAGE = 2;
     SIF_POS = 4;
     SIF_RANGE = 1;
     SIF_DISABLENOSCROLL = 8;
     SIF_TRACKPOS = $10;
     SIF_ALL = SIF_RANGE or SIF_PAGE or SIF_POS or SIF_TRACKPOS;
  { GetStdHandle  }


  { was #define dname def_expr }
     STD_INPUT_HANDLE = DWORD(-(10));
     STD_OUTPUT_HANDLE = DWORD(-(11));
     STD_ERROR_HANDLE = DWORD(-(12));

    { was #define dname def_expr }
     INVALID_HANDLE_VALUE = HANDLE(-1);

  { GetStockObject  }

     BLACK_BRUSH = 4;
     DKGRAY_BRUSH = 3;
     GRAY_BRUSH = 2;
     HOLLOW_BRUSH = 5;
     LTGRAY_BRUSH = 1;
     NULL_BRUSH = 5;
     WHITE_BRUSH = 0;
     BLACK_PEN = 7;
     NULL_PEN = 8;
     WHITE_PEN = 6;
     ANSI_FIXED_FONT = 11;
     ANSI_VAR_FONT = 12;
     DEVICE_DEFAULT_FONT = 14;
<<<<<<< HEAD
<<<<<<< HEAD
//     DEFAULT_GUI_FONT = 17;  Returns zero in the emulator and
                            // isn't listed in MSDN, see bug 15062
=======
     DEFAULT_GUI_FONT = 17;
>>>>>>> graemeg/fixes_2_2
=======
     DEFAULT_GUI_FONT = 17;
>>>>>>> origin/fixes_2_2
     OEM_FIXED_FONT = 10;
     SYSTEM_FONT = 13;
     SYSTEM_FIXED_FONT = 16;
     DEFAULT_PALETTE = 15;
  { GetStringTypeA  }
     CT_CTYPE1 = 1;
     CT_CTYPE2 = 2;
     CT_CTYPE3 = 4;
     C1_UPPER = 1;
     C1_LOWER = 2;
     C1_DIGIT = 4;
     C1_SPACE = 8;
     C1_PUNCT = 16;
     C1_CNTRL = 32;
     C1_BLANK = 64;
     C1_XDIGIT = 128;
     C1_ALPHA = 256;
     C2_LEFTTORIGHT = 1;
     C2_RIGHTTOLEFT = 2;
     C2_EUROPENUMBER = 3;
     C2_EUROPESEPARATOR = 4;
     C2_EUROPETERMINATOR = 5;
     C2_ARABICNUMBER = 6;
     C2_COMMONSEPARATOR = 7;
     C2_BLOCKSEPARATOR = 8;
     C2_SEGMENTSEPARATOR = 9;
     C2_WHITESPACE = 10;
     C2_OTHERNEUTRAL = 11;
     C2_NOTAPPLICABLE = 0;
     C3_NONSPACING = 1;
     C3_DIACRITIC = 2;
     C3_VOWELMARK = 4;
     C3_SYMBOL = 8;
     C3_KATAKANA = 16;
     C3_HIRAGANA = 32;
     C3_HALFWIDTH = 64;
     C3_FULLWIDTH = 128;
     C3_IDEOGRAPH = 256;
     C3_KASHIDA = 512;
     C3_ALPHA = 32768;
     C3_NOTAPPLICABLE = 0;
  { GetSysColor }
{$IFDEF WINCE}
     SYS_COLOR_INDEX_FLAG =$40000000;
{$ELSE}
     SYS_COLOR_INDEX_FLAG =0;
{$ENDIF WINCE}
     COLOR_SCROLLBAR               = 0 or SYS_COLOR_INDEX_FLAG;
     COLOR_BACKGROUND              = 1 or SYS_COLOR_INDEX_FLAG;
     COLOR_ACTIVECAPTION           = 2 or SYS_COLOR_INDEX_FLAG;
     COLOR_INACTIVECAPTION         = 3 or SYS_COLOR_INDEX_FLAG;
     COLOR_MENU                    = 4 or SYS_COLOR_INDEX_FLAG;
     COLOR_WINDOW                  = 5 or SYS_COLOR_INDEX_FLAG;
     COLOR_WINDOWFRAME             = 6 or SYS_COLOR_INDEX_FLAG;
     COLOR_MENUTEXT                = 7 or SYS_COLOR_INDEX_FLAG;
     COLOR_WINDOWTEXT              = 8 or SYS_COLOR_INDEX_FLAG;
     COLOR_CAPTIONTEXT             = 9 or SYS_COLOR_INDEX_FLAG;
     COLOR_ACTIVEBORDER            =10 or SYS_COLOR_INDEX_FLAG;
     COLOR_INACTIVEBORDER          =11 or SYS_COLOR_INDEX_FLAG;
     COLOR_APPWORKSPACE            =12 or SYS_COLOR_INDEX_FLAG;
     COLOR_HIGHLIGHT               =13 or SYS_COLOR_INDEX_FLAG;
     COLOR_HIGHLIGHTTEXT           =14 or SYS_COLOR_INDEX_FLAG;
     COLOR_BTNFACE                 =15 or SYS_COLOR_INDEX_FLAG;
     COLOR_BTNSHADOW               =16 or SYS_COLOR_INDEX_FLAG;
     COLOR_GRAYTEXT                =17 or SYS_COLOR_INDEX_FLAG;
     COLOR_BTNTEXT                 =18 or SYS_COLOR_INDEX_FLAG;
     COLOR_INACTIVECAPTIONTEXT     =19 or SYS_COLOR_INDEX_FLAG;
     COLOR_BTNHIGHLIGHT            =20 or SYS_COLOR_INDEX_FLAG;

     COLOR_3DDKSHADOW              =21 or SYS_COLOR_INDEX_FLAG;
     COLOR_3DLIGHT                 =22 or SYS_COLOR_INDEX_FLAG;
     COLOR_INFOTEXT                =23 or SYS_COLOR_INDEX_FLAG;
     COLOR_INFOBK                  =24 or SYS_COLOR_INDEX_FLAG;
     COLOR_STATIC                  =25 or SYS_COLOR_INDEX_FLAG;
     COLOR_STATICTEXT              =26 or SYS_COLOR_INDEX_FLAG;
     COLOR_GRADIENTACTIVECAPTION   =27 or SYS_COLOR_INDEX_FLAG;
     COLOR_GRADIENTINACTIVECAPTION =28 or SYS_COLOR_INDEX_FLAG;
     C_SYS_COLOR_TYPES       =29;

     COLOR_DESKTOP           =COLOR_BACKGROUND;
     COLOR_3DFACE            =COLOR_BTNFACE;
     COLOR_3DSHADOW          =COLOR_BTNSHADOW;
     COLOR_3DHIGHLIGHT       =COLOR_BTNHIGHLIGHT;
     COLOR_3DHILIGHT         =COLOR_BTNHIGHLIGHT;

  { GetSystemMetrics  }
     SM_CYMIN = 29;
     SM_CXMIN = 28;
     SM_ARRANGE = 56;
     SM_CLEANBOOT = 67;
  { The right value for SM_CEMETRICS for NT 3.5 is 75.  For Windows 95
     and NT 4.0, it is 76.  The meaning is undocumented, anyhow.   }
     SM_CMETRICS = 76;
     SM_CMOUSEBUTTONS = 43;
     SM_CXBORDER = 5;
     SM_CYBORDER = 6;
     SM_CXCURSOR = 13;
     SM_CYCURSOR = 14;
     SM_CXDLGFRAME = 7;
     SM_CYDLGFRAME = 8;
     SM_CXDOUBLECLK = 36;
     SM_CYDOUBLECLK = 37;
     SM_CXDRAG = 68;
     SM_CYDRAG = 69;
     SM_CXEDGE = 45;
     SM_CYEDGE = 46;
     SM_CXFIXEDFRAME = 7;
     SM_CYFIXEDFRAME = 8;
     SM_CXFRAME = 32;
     SM_CYFRAME = 33;
     SM_CXFULLSCREEN = 16;
     SM_CYFULLSCREEN = 17;
     SM_CXHSCROLL = 21;
     SM_CYHSCROLL = 3;
     SM_CXHTHUMB = 10;
     SM_CXICON = 11;
     SM_CYICON = 12;
     SM_CXICONSPACING = 38;
     SM_CYICONSPACING = 39;
     SM_CXMAXIMIZED = 61;
     SM_CYMAXIMIZED = 62;
     SM_CXMAXTRACK = 59;
     SM_CYMAXTRACK = 60;
     SM_CXMENUCHECK = 71;
     SM_CYMENUCHECK = 72;
     SM_CXMENUSIZE = 54;
     SM_CYMENUSIZE = 55;
     SM_CXMINIMIZED = 57;
     SM_CYMINIMIZED = 58;
     SM_CXMINSPACING = 47;
     SM_CYMINSPACING = 48;
     SM_CXMINTRACK = 34;
     SM_CYMINTRACK = 35;
     SM_CXSCREEN = 0;
     SM_CYSCREEN = 1;
     SM_CXSIZE = 30;
     SM_CYSIZE = 31;
     SM_CXSIZEFRAME = 32;
     SM_CYSIZEFRAME = 33;
     SM_CXSMICON = 49;
     SM_CYSMICON = 50;
     SM_CXSMSIZE = 52;
     SM_CYSMSIZE = 53;
     SM_CXVSCROLL = 2;
     {SM_CYHSCROLL = 3;already above }
     {SM_CXHSCROLL = 21;already above }
     SM_CYVSCROLL = 20;
     SM_CYVTHUMB = 9;
     SM_CYCAPTION = 4;
     SM_CYKANJIWINDOW = 18;
     SM_CYMENU = 15;
     SM_CYSMCAPTION = 51;
     SM_DBCSENABLED = 42;
     SM_DEBUG = 22;
     SM_MENUDROPALIGNMENT = 40;
     SM_MIDEASTENABLED = 74;
     SM_MOUSEPRESENT = 19;
     SM_MOUSEWHEELPRESENT = 75;
     SM_NETWORK = 63;
     SM_PENWINDOWS = 41;
     SM_SECURE = 44;
     SM_SHOWSOUNDS = 70;
     SM_SLOWMACHINE = 73;
     SM_SWAPBUTTON = 23;
     ARW_BOTTOMLEFT = 0;
     ARW_BOTTOMRIGHT = $1;
     ARW_HIDE = $8;
     ARW_TOPLEFT = $2;
     ARW_TOPRIGHT = $3;
     ARW_DOWN = $4;
     ARW_LEFT = 0;
     ARW_RIGHT = 0;
     ARW_UP = $4;
  { GetSystemPaletteUse  }
     SYSPAL_NOSTATIC = 2;
     SYSPAL_STATIC = 1;
     SYSPAL_ERROR = 0;
  { GetTextAlign  }
     TA_BASELINE = 24;
     TA_BOTTOM = 8;
     TA_TOP = 0;
     TA_CENTER = 6;
     TA_LEFT = 0;
     TA_RIGHT = 2;
     TA_RTLREADING = 256;
     TA_NOUPDATECP = 0;
     TA_UPDATECP = 1;
     VTA_BASELINE = 24;
     VTA_CENTER = 6;
  { GetThreadPriority  }
     THREAD_PRIORITY_TIME_CRITICAL = 0;
     THREAD_PRIORITY_HIGHEST       = 1;
     THREAD_PRIORITY_ABOVE_NORMAL  = 2;
     THREAD_PRIORITY_NORMAL        = 3;
     THREAD_PRIORITY_BELOW_NORMAL  = 4;
     THREAD_PRIORITY_LOWEST        = 5;
     THREAD_PRIORITY_ABOVE_IDLE    = 6;
     THREAD_PRIORITY_IDLE          = 7;
     THREAD_PRIORITY_ERROR_RETURN  = 2147483647;
     TLS_MINIMUM_AVAILABLE = 64;
  { GetTimeFormat  }
     TIME_NOMINUTESORSECONDS = 1;
     TIME_NOSECONDS = 2;
     TIME_NOTIMEMARKER = 4;
     TIME_FORCE24HOURFORMAT = 8;
  { GetTimeZoneInformation  }
    { was #define dname def_expr }


  const
     TIME_ZONE_ID_INVALID = DWORD(-1);
     TIME_ZONE_ID_UNKNOWN = 0;
     TIME_ZONE_ID_STANDARD = 1;
     TIME_ZONE_ID_DAYLIGHT = 2;
  { GetWindowLong  }
     GWL_EXSTYLE = -(20);
     GWL_STYLE = -(16);
     GWL_WNDPROC = -(4);
     GWL_HINSTANCE = -(6);
     GWL_HWNDPARENT = -(8);
     GWL_ID = -(12);
     GWL_USERDATA = -(21);
     DWL_DLGPROC = 4;
     DWL_MSGRESULT = 0;
     DWL_USER = 8;
  { LocalAlloc, LocalFlags  }
     LMEM_FIXED          = $0000;
     LMEM_MOVEABLE       = $0002;
     LMEM_NOCOMPACT      = $0010;      //**** Used for Moveable Memory  ***/
     LMEM_NODISCARD      = $0020;      //**** Ignored *****/
     LMEM_ZEROINIT       = $0040;
     LMEM_MODIFY         = $0080;      //*** Used only in LocalReAlloc() **/
     LMEM_DISCARDABLE    = $0F00;      //**** Ignored ****/
     LMEM_VALID_FLAGS    = $0F72;
     LMEM_INVALID_HANDLE = $8000;
     LMEM_DDESHARE       = $0000;  //**** Ignored ****/
     LMEM_LOWER          = $0000;  //**** Ignored ****/
     LMEM_NOT_BANKED     = $0000;  //**** Ignored ****/
     LMEM_NOTIFY         = $0000;  //**** Ignored ****/
     LMEM_SHARE          = $0000;  //**** Ignored ****/
     LPTR                = LMEM_FIXED or LMEM_ZEROINIT;
     LHND                = LMEM_MOVEABLE or LMEM_ZEROINIT;
     NONZEROLHND         = LMEM_MOVEABLE;
     NONZEROLPTR         = LMEM_FIXED;
//* Flags returned by LocalFlags (in addition to LMEM_DISCARDABLE) */
     LMEM_DISCARDED      = $4000;
     LMEM_LOCKCOUNT      = $00FF;
  { GlobalAlloc, GlobalFlags  }
     GMEM_FIXED          = LMEM_FIXED;
     GMEM_MOVEABLE       = LMEM_MOVEABLE;
     GPTR                = LPTR;
     GHND                = LHND;
     GMEM_DDESHARE       = LMEM_DDESHARE;
     GMEM_DISCARDABLE    = LMEM_DISCARDABLE;
     GMEM_LOWER          = LMEM_LOWER;
     GMEM_NOCOMPACT      = LMEM_NOCOMPACT;
     GMEM_NODISCARD      = LMEM_NODISCARD;
     GMEM_NOT_BANKED     = LMEM_NOT_BANKED;
     GMEM_NOTIFY         = LMEM_NOTIFY;
     GMEM_SHARE          = LMEM_SHARE;
     GMEM_ZEROINIT       = LMEM_ZEROINIT;
  { HeapAlloc, HeapReAlloc  }
     HEAP_GENERATE_EXCEPTIONS = 4;
     HEAP_NO_SERIALIZE = 1;
     HEAP_ZERO_MEMORY = 8;
     STATUS_NO_MEMORY = $c0000017;
     STATUS_ACCESS_VIOLATION = $c0000005;
     HEAP_REALLOC_IN_PLACE_ONLY = 16;
  { ImageList_Create  }
     ILC_COLOR = 0;
     ILC_COLOR4 = 4;
     ILC_COLOR8 = 8;
     ILC_COLOR16 = 16;
     ILC_COLOR24 = 24;
     ILC_COLOR32 = 32;
     ILC_COLORDDB = 254;
     ILC_MASK = 1;
     ILC_PALETTE = 2048;
  { ImageList_Draw, ImageList_DrawEx  }
     ILD_BLEND25 = 2;
     ILD_BLEND50 = 4;
     ILD_SELECTED = 4;
     ILD_BLEND = 4;
     ILD_FOCUS = 2;
     ILD_MASK = 16;
     ILD_NORMAL = 0;
     ILD_TRANSPARENT = 1;
     CLR_NONE = $ffffffff;
     CLR_DEFAULT = $ff000000;
     CLR_INVALID = $FFFFFFFF;
  { ImageList_LoadImage  }
     {LR_DEFAULTCOLOR = 0;already above }
     LR_LOADFROMFILE = 16;
     LR_LOADMAP3DCOLORS = 4096;
     LR_LOADTRANSPARENT = 32;
     {LR_MONOCHROME = 1;already above }
  { ImmConfigureIME  }
     IME_CONFIG_GENERAL = 1;
     IME_CONFIG_REGISTERWORD = 2;
     IME_CONFIG_SELECTDICTIONARY = 3;
  { ImmGetConversionList  }
     GCL_CONVERSION = 1;
     GCL_REVERSECONVERSION = 2;
     GCL_REVERSE_LENGTH = 3;
  { ImmGetGuideLine  }
     GGL_LEVEL = 1;
     GGL_INDEX = 2;
     GGL_STRING = 3;
     GGL_PRIVATE = 4;
     GL_LEVEL_ERROR = 2;
     GL_LEVEL_FATAL = 1;
     GL_LEVEL_INFORMATION = 4;
     GL_LEVEL_NOGUIDELINE = 0;
     GL_LEVEL_WARNING = 3;
     GL_ID_CANNOTSAVE = 17;
     GL_ID_NOCONVERT = 32;
     GL_ID_NODICTIONARY = 16;
     GL_ID_NOMODULE = 1;
     GL_ID_READINGCONFLICT = 35;
     GL_ID_TOOMANYSTROKE = 34;
     GL_ID_TYPINGERROR = 33;
     GL_ID_UNKNOWN = 0;
     GL_ID_INPUTREADING = 36;
     GL_ID_INPUTRADICAL = 37;
     GL_ID_INPUTCODE = 38;
     GL_ID_CHOOSECANDIDATE = 40;
     GL_ID_REVERSECONVERSION = 41;
  { ImmGetProperty  }
     IGP_PROPERTY = 4;
     IGP_CONVERSION = 8;
     IGP_SENTENCE = 12;
     IGP_UI = 16;
     IGP_SETCOMPSTR = 20;
     IGP_SELECT = 24;
     IME_PROP_AT_CARET = 65536;
     IME_PROP_SPECIAL_UI = 131072;
     IME_PROP_CANDLIST_START_FROM_1 = 262144;
     IME_PROP_UNICODE = 524288;
     UI_CAP_2700 = 1;
     UI_CAP_ROT90 = 2;
     UI_CAP_ROTANY = 4;
     SCS_CAP_COMPSTR = 1;
     SCS_CAP_MAKEREAD = 2;
     SELECT_CAP_CONVERSION = 1;
     SELECT_CAP_SENTENCE = 2;
  { ImmNotifyIME  }
     NI_CHANGECANDIDATELIST = 19;
     NI_CLOSECANDIDATE = 17;
     NI_COMPOSITIONSTR = 21;
     NI_OPENCANDIDATE = 16;
     NI_SELECTCANDIDATESTR = 18;
     NI_SETCANDIDATE_PAGESIZE = 23;
     NI_SETCANDIDATE_PAGESTART = 22;
     CPS_CANCEL = 4;
     CPS_COMPLETE = 1;
     CPS_CONVERT = 2;
     CPS_REVERT = 3;
  { ImmSetCompositionString  }
     SCS_SETSTR = 9;
     SCS_CHANGEATTR = 18;
     SCS_CHANGECLAUSE = 36;
  { ImmUnregisterWord  }
     IME_REGWORD_STYLE_EUDC = 1;
     IME_REGWORD_STYLE_USER_FIRST = $80000000;
     IME_REGWORD_STYLE_USER_LAST = -(1);
  { InitCommonControlEx }  //+commctrl
     I_IMAGENONE             = -2; // Desktop listview uses this same value for I_IMAGENONE when (_WIN32_IE >= 0x0501)

     ICC_LISTVIEW_CLASSES  = $00000001; // listview, header
     ICC_TREEVIEW_CLASSES  = $00000002; // treeview, tooltips
     ICC_BAR_CLASSES       = $00000004; // toolbar, statusbar, trackbar, tooltips
     ICC_TAB_CLASSES       = $00000008; // tab, tooltips
     ICC_UPDOWN_CLASS      = $00000010; // updown
     ICC_PROGRESS_CLASS    = $00000020; // progress
     ICC_WIN95_CLASSES     = $0000007F; // ! was ICC_WIN95_CLASSES     = $000000FF
     ICC_DATE_CLASSES      = $00000100; // month picker, date picker, time picker, updown
     ICC_COOL_CLASSES      = $00000400; // rebar (coolbar) control
     ICC_INTERNET_CLASSES  = $00000800; // IP Address control
     ICC_TOOLTIP_CLASSES   = $00001000; // Tooltip static & button
     ICC_CAPEDIT_CLASS     = $00002000; // All-caps edit control
     ICC_FE_CLASSES        = $40000000; // FE specific input subclasses

  { InitializeSecurityDescriptor  }
     SECURITY_DESCRIPTOR_REVISION = 1;
  { IsTextUnicode  }
     IS_TEXT_UNICODE_ASCII16 = 1;
     IS_TEXT_UNICODE_REVERSE_ASCII16 = 16;
     IS_TEXT_UNICODE_STATISTICS = 2;
     IS_TEXT_UNICODE_REVERSE_STATISTICS = 32;
     IS_TEXT_UNICODE_CONTROLS = 4;
     IS_TEXT_UNICODE_REVERSE_CONTROLS = 64;
     IS_TEXT_UNICODE_SIGNATURE = 8;
     IS_TEXT_UNICODE_REVERSE_SIGNATURE = 128;
     IS_TEXT_UNICODE_ILLEGAL_CHARS = 256;
     IS_TEXT_UNICODE_ODD_LENGTH = 512;
     IS_TEXT_UNICODE_NULL_BYTES = 4096;
     IS_TEXT_UNICODE_UNICODE_MASK = 15;
     IS_TEXT_UNICODE_REVERSE_MASK = 240;
     IS_TEXT_UNICODE_NOT_UNICODE_MASK = 3840;
     IS_TEXT_UNICODE_NOT_ASCII_MASK = 61440;
  { JournalPlaybackProc, KeyboardProc  }
     HC_GETNEXT = 1;
     HC_SKIP = 2;
     HC_SYSMODALOFF = 5;
     HC_SYSMODALON = 4;
     HC_NOREMOVE = 3;
  { keybd_event  }
     KEYEVENTF_EXTENDEDKEY = 1;
     KEYEVENTF_KEYUP = 2;
     KEYEVENTF_SILENT = 4;
  { LoadBitmap  }
     OBM_BTNCORNERS = 32758;
     OBM_BTSIZE = 32761;
     OBM_CHECK = 32760;
     OBM_CHECKBOXES = 32759;
     OBM_CLOSE = 32754;
     OBM_COMBO = 32738;
     OBM_DNARROW = 32752;
     OBM_DNARROWD = 32742;
     OBM_DNARROWI = 32736;
     OBM_LFARROW = 32750;
     OBM_LFARROWI = 32734;
     OBM_LFARROWD = 32740;
     OBM_MNARROW = 32739;
     OBM_OLD_CLOSE = 32767;
     OBM_OLD_DNARROW = 32764;
     OBM_OLD_LFARROW = 32762;
     OBM_OLD_REDUCE = 32757;
     OBM_OLD_RESTORE = 32755;
     OBM_OLD_RGARROW = 32763;
     OBM_OLD_UPARROW = 32765;
     OBM_OLD_ZOOM = 32756;
     OBM_REDUCE = 32749;
     OBM_REDUCED = 32746;
     OBM_RESTORE = 32747;
     OBM_RESTORED = 32744;
     OBM_RGARROW = 32751;
     OBM_RGARROWD = 32741;
     OBM_RGARROWI = 32735;
     OBM_SIZE = 32766;
     OBM_UPARROW = 32753;
     OBM_UPARROWD = 32743;
     OBM_UPARROWI = 32737;
     OBM_ZOOM = 32748;
     OBM_ZOOMD = 32745;
  { LoadLibraryEx  }
     DONT_RESOLVE_DLL_REFERENCES = 1;
     LOAD_LIBRARY_AS_DATAFILE = 2;
     LOAD_WITH_ALTERED_SEARCH_PATH = 8;
  { MessageBeep, MessageBox  }
     MB_USERICON = $80;
     MB_ICONASTERISK = $40;
     MB_ICONEXCLAMATION = $30;
     MB_ICONWARNING = $30;
     MB_ICONERROR = $10;
     MB_ICONHAND = $10;
     MB_ICONQUESTION = $20;
     MB_OK = 0;
     MB_ABORTRETRYIGNORE = $2;
     MB_APPLMODAL = 0;
     MB_DEFAULT_DESKTOP_ONLY = $20000;
     MB_HELP = $4000;
     MB_RIGHT = $80000;
     MB_RTLREADING = $100000;
     MB_TOPMOST = $40000;
     MB_DEFBUTTON1 = 0;
     MB_DEFBUTTON2 = $100;
     MB_DEFBUTTON3 = $200;
     MB_DEFBUTTON4 = $300;
     MB_ICONINFORMATION = $40;
     MB_ICONSTOP = $10;
     MB_OKCANCEL = $1;
     MB_RETRYCANCEL = $5;
     MB_SERVICE_NOTIFICATION = $40000;
     MB_SETFOREGROUND = $10000;
     MB_SYSTEMMODAL = $1000;
     MB_TASKMODAL = $2000;
     MB_YESNO = $4;
     MB_YESNOCANCEL = $3;
     IDABORT = 3;
     IDCANCEL = 2;
     IDCLOSE = 8;
     IDHELP = 9;
     IDIGNORE = 5;
     IDNO = 7;
     IDOK = 1;
     IDRETRY = 4;
     IDYES = 6;
  { MessageProc  }
     MSGF_DIALOGBOX = 0;
     MSGF_MENU = 2;
     MSGF_NEXTWINDOW = 6;
     MSGF_SCROLLBAR = 5;
     MSGF_MAINLOOP = 8;
     MSGF_USER = 4096;
  { ModifyWorldTransform  }
     MWT_IDENTITY = 1;
     MWT_LEFTMULTIPLY = 2;
     MWT_RIGHTMULTIPLY = 3;
  { mouse_event  }
     MOUSEEVENTF_ABSOLUTE = 32768;
     MOUSEEVENTF_MOVE = 1;
     MOUSEEVENTF_LEFTDOWN = 2;
     MOUSEEVENTF_LEFTUP = 4;
     MOUSEEVENTF_RIGHTDOWN = 8;
     MOUSEEVENTF_RIGHTUP = 16;
     MOUSEEVENTF_MIDDLEDOWN = 32;
     MOUSEEVENTF_MIDDLEUP = 64;
  { MoveFileEx  }
     MOVEFILE_REPLACE_EXISTING = 1;
     MOVEFILE_COPY_ALLOWED = 2;
     MOVEFILE_DELAY_UNTIL_REBOOT = 4;
  { MsgWaitForMultipleObjects, WaitForMultipleObjectsEx  }
     WAIT_OBJECT_0 = 0;
     WAIT_ABANDONED_0 = $80;
     WAIT_TIMEOUT = $102;
     WAIT_IO_COMPLETION = $c0;
     WAIT_ABANDONED = $80;
     WAIT_FAILED = $ffffffff;
     MAXIMUM_WAIT_OBJECTS = $40;     //winnt
     MAXIMUM_SUSPEND_COUNT = MAXCHAR;    //~winnt
  { MultiByteToWideChar  }
     MB_PRECOMPOSED = 1;
     MB_COMPOSITE = 2;
     MB_ERR_INVALID_CHARS = 8;
     MB_USEGLYPHCHARS = 4;
  { PostMessage  }
    { was #define dname def_expr }
    HWND_BROADCAST              = HWND($FFFF);

  const
  { PropertySheet  }
     IS_PSREBOOTSYSTEM = 3;
     IS_PSRESTARTWINDOWS = 2;
  { PropSheetPageProc  }
     PSPCB_CREATE = 2;
     PSPCB_RELEASE = 1;
  { PurgeComm  }
     PURGE_TXABORT = 1;
     PURGE_RXABORT = 2;
     PURGE_TXCLEAR = 4;
     PURGE_RXCLEAR = 8;
  { QueryServiceObjectSecurity  }
     OWNER_SECURITY_INFORMATION = $1;
     GROUP_SECURITY_INFORMATION = $2;
     DACL_SECURITY_INFORMATION = $4;
     SACL_SECURITY_INFORMATION = $8;
  { RedrawWindow  }
     RDW_ERASE = 4;
     RDW_FRAME = 1024;
     RDW_INTERNALPAINT = 2;
     RDW_INVALIDATE = 1;
     RDW_NOERASE = 32;
     RDW_NOFRAME = 2048;
     RDW_NOINTERNALPAINT = 16;
     RDW_VALIDATE = 8;
     RDW_ERASENOW = 512;
     RDW_UPDATENOW = 256;
     RDW_ALLCHILDREN = 128;
     RDW_NOCHILDREN = 64;
  { RegCreateKey  }
     HKEY_CLASSES_ROOT           = HKEY($80000000);
     HKEY_CURRENT_USER           = HKEY($80000001);
     HKEY_LOCAL_MACHINE          = HKEY($80000002);
     HKEY_USERS                  = HKEY($80000003);
     HKEY_PERFORMANCE_DATA       = HKEY($80000004);
     HKEY_CURRENT_CONFIG         = HKEY($80000005);
     HKEY_DYN_DATA               = HKEY($80000006);

  { RegCreateKeyEx  }

  const
     REG_OPTION_VOLATILE = $1;
     REG_OPTION_NON_VOLATILE = 0;
     REG_CREATED_NEW_KEY = $1;
     REG_OPENED_EXISTING_KEY = $2;
  { RegEnumValue  }
     REG_BINARY = 3;
     REG_DWORD = 4;
     REG_DWORD_LITTLE_ENDIAN = 4;
     REG_DWORD_BIG_ENDIAN = 5;
     REG_EXPAND_SZ = 2;
     REG_FULL_RESOURCE_DESCRIPTOR = 9;
     REG_LINK = 6;
     REG_MULTI_SZ = 7;
     REG_NONE = 0;
     REG_RESOURCE_LIST = 8;
     REG_RESOURCE_REQUIREMENTS_LIST = 10;
     REG_SZ = 1;
  { RegisterHotKey  }
     MOD_ALT = 1;
     MOD_CONTROL = 2;
     MOD_SHIFT = 4;
     MOD_WIN = 8;
<<<<<<< HEAD
<<<<<<< HEAD
     MOD_KEYUP = $1000;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     IDHOT_SNAPDESKTOP = -(2);
     IDHOT_SNAPWINDOW = -(1);
  { RegNotifyChangeKeyValue  }
     REG_NOTIFY_CHANGE_NAME = $1;
     REG_NOTIFY_CHANGE_ATTRIBUTES = $2;
     REG_NOTIFY_CHANGE_LAST_SET = $4;
     REG_NOTIFY_CHANGE_SECURITY = $8;
  { ScrollWindowEx  }
     SW_ERASE = 4;
     SW_INVALIDATE = 2;
     SW_SCROLLCHILDREN = 1;
  { SendMessageTimeout  }
     SMTO_ABORTIFHUNG = 2;
     SMTO_BLOCK = 1;
     SMTO_NORMAL = 0;
  { SetBkMode  }
     OPAQUE = 2;
     TRANSPARENT = 1;
  { SetDebugErrorLevel  }
     SLE_ERROR = 1;
     SLE_MINORERROR = 2;
     SLE_WARNING = 3;
  { SetErrorMode  }
     SEM_FAILCRITICALERRORS = 1;
     SEM_NOALIGNMENTFAULTEXCEPT = 4;
     SEM_NOGPFAULTERRORBOX = 2;
     SEM_NOOPENFILEERRORBOX = 32768;
  { SetICMMode  }
     ICM_ON = 2;
     ICM_OFF = 1;
     ICM_QUERY = 3;
  { SetJob  }
  { Locale Information  }
     LOCALE_ILANGUAGE = 1;
     LOCALE_SLANGUAGE = 2;
     LOCALE_SENGLANGUAGE = 4097;
     LOCALE_SABBREVLANGNAME = 3;
     LOCALE_SNATIVELANGNAME = 4;
     LOCALE_ICOUNTRY = 5;
     LOCALE_SCOUNTRY = 6;
     LOCALE_SENGCOUNTRY = 4098;
     LOCALE_SABBREVCTRYNAME = 7;
     LOCALE_SNATIVECTRYNAME = 8;
     LOCALE_IDEFAULTLANGUAGE = 9;
     LOCALE_IDEFAULTCOUNTRY = 10;
     LOCALE_IDEFAULTANSICODEPAGE = 4100;
     LOCALE_IDEFAULTCODEPAGE = 11;
     LOCALE_SLIST = 12;
     LOCALE_IMEASURE = 13;
     LOCALE_SDECIMAL = 14;
     LOCALE_STHOUSAND = 15;
     LOCALE_SGROUPING = 16;
     LOCALE_IDIGITS = 17;
     LOCALE_ILZERO = 18;
     LOCALE_INEGNUMBER = 4112;
     LOCALE_SCURRENCY = 20;
     LOCALE_SMONDECIMALSEP = 22;
     LOCALE_SMONTHOUSANDSEP = 23;
     LOCALE_SMONGROUPING = 24;
     LOCALE_ICURRDIGITS = 25;
     LOCALE_ICURRENCY = 27;
     LOCALE_INEGCURR = 28;
     LOCALE_SDATE = 29;
     LOCALE_STIME = 30;
     LOCALE_STIMEFORMAT = 4099;
     LOCALE_SSHORTDATE = 31;
     LOCALE_SLONGDATE = 32;
     LOCALE_IDATE = 33;
     LOCALE_ILDATE = 34;
     LOCALE_ITIME = 35;
     LOCALE_ITLZERO = 37;
     LOCALE_IDAYLZERO = 38;
     LOCALE_IMONLZERO = 39;
     LOCALE_S1159 = 40;
     LOCALE_S2359 = 41;
     LOCALE_ICALENDARTYPE = 4105;
     LOCALE_IOPTIONALCALENDAR = 4107;
     LOCALE_IFIRSTDAYOFWEEK = 4108;
     LOCALE_IFIRSTWEEKOFYEAR = 4109;
     LOCALE_SDAYNAME1 = 42;
     LOCALE_SDAYNAME2 = 43;
     LOCALE_SDAYNAME3 = 44;
     LOCALE_SDAYNAME4 = 45;
     LOCALE_SDAYNAME5 = 46;
     LOCALE_SDAYNAME6 = 47;
     LOCALE_SDAYNAME7 = 48;
     LOCALE_SABBREVDAYNAME1 = 49;
     LOCALE_SABBREVDAYNAME2 = 50;
     LOCALE_SABBREVDAYNAME3 = 51;
     LOCALE_SABBREVDAYNAME4 = 52;
     LOCALE_SABBREVDAYNAME5 = 53;
     LOCALE_SABBREVDAYNAME6 = 54;
     LOCALE_SABBREVDAYNAME7 = 55;
     LOCALE_SMONTHNAME1 = 56;
     LOCALE_SMONTHNAME2 = 57;
     LOCALE_SMONTHNAME3 = 58;
     LOCALE_SMONTHNAME4 = 59;
     LOCALE_SMONTHNAME5 = 60;
     LOCALE_SMONTHNAME6 = 61;
     LOCALE_SMONTHNAME7 = 62;
     LOCALE_SMONTHNAME8 = 63;
     LOCALE_SMONTHNAME9 = 64;
     LOCALE_SMONTHNAME10 = 65;
     LOCALE_SMONTHNAME11 = 66;
     LOCALE_SMONTHNAME12 = 67;
     LOCALE_SMONTHNAME13 = 4110;
     LOCALE_SABBREVMONTHNAME1 = 68;
     LOCALE_SABBREVMONTHNAME2 = 69;
     LOCALE_SABBREVMONTHNAME3 = 70;
     LOCALE_SABBREVMONTHNAME4 = 71;
     LOCALE_SABBREVMONTHNAME5 = 72;
     LOCALE_SABBREVMONTHNAME6 = 73;
     LOCALE_SABBREVMONTHNAME7 = 74;
     LOCALE_SABBREVMONTHNAME8 = 75;
     LOCALE_SABBREVMONTHNAME9 = 76;
     LOCALE_SABBREVMONTHNAME10 = 77;
     LOCALE_SABBREVMONTHNAME11 = 78;
     LOCALE_SABBREVMONTHNAME12 = 79;
     LOCALE_SABBREVMONTHNAME13 = 4111;
     LOCALE_SPOSITIVESIGN = 80;
     LOCALE_SNEGATIVESIGN = 81;
     LOCALE_IPOSSIGNPOSN = 82;
     LOCALE_INEGSIGNPOSN = 83;
     LOCALE_IPOSSYMPRECEDES = 84;
     LOCALE_IPOSSEPBYSPACE = 85;
     LOCALE_INEGSYMPRECEDES = 86;
     LOCALE_INEGSEPBYSPACE = 87;
     LOCALE_NOUSEROVERRIDE = $80000000;
     LOCALE_USE_CP_ACP     = $40000000;   // use the system ACP
     LOCALE_RETURN_NUMBER  = $20000000;   // return number instead
  { Calendar Type Information  }
     CAL_ICALINTVALUE = 1;
     CAL_IYEAROFFSETRANGE = 3;
     CAL_SABBREVDAYNAME1 = 14;
     CAL_SABBREVDAYNAME2 = 15;
     CAL_SABBREVDAYNAME3 = 16;
     CAL_SABBREVDAYNAME4 = 17;
     CAL_SABBREVDAYNAME5 = 18;
     CAL_SABBREVDAYNAME6 = 19;
     CAL_SABBREVDAYNAME7 = 20;
     CAL_SABBREVMONTHNAME1 = 34;
     CAL_SABBREVMONTHNAME2 = 35;
     CAL_SABBREVMONTHNAME3 = 36;
     CAL_SABBREVMONTHNAME4 = 37;
     CAL_SABBREVMONTHNAME5 = 38;
     CAL_SABBREVMONTHNAME6 = 39;
     CAL_SABBREVMONTHNAME7 = 40;
     CAL_SABBREVMONTHNAME8 = 41;
     CAL_SABBREVMONTHNAME9 = 42;
     CAL_SABBREVMONTHNAME10 = 43;
     CAL_SABBREVMONTHNAME11 = 44;
     CAL_SABBREVMONTHNAME12 = 45;
     CAL_SABBREVMONTHNAME13 = 46;
     CAL_SCALNAME = 2;
     CAL_SDAYNAME1 = 7;
     CAL_SDAYNAME2 = 8;
     CAL_SDAYNAME3 = 9;
     CAL_SDAYNAME4 = 10;
     CAL_SDAYNAME5 = 11;
     CAL_SDAYNAME6 = 12;
     CAL_SDAYNAME7 = 13;
     CAL_SERASTRING = 4;
     CAL_SLONGDATE = 6;
     CAL_SMONTHNAME1 = 21;
     CAL_SMONTHNAME2 = 22;
     CAL_SMONTHNAME3 = 23;
     CAL_SMONTHNAME4 = 24;
     CAL_SMONTHNAME5 = 25;
     CAL_SMONTHNAME6 = 26;
     CAL_SMONTHNAME7 = 27;
     CAL_SMONTHNAME8 = 28;
     CAL_SMONTHNAME9 = 29;
     CAL_SMONTHNAME10 = 30;
     CAL_SMONTHNAME11 = 31;
     CAL_SMONTHNAME12 = 32;
     CAL_SMONTHNAME13 = 33;
     CAL_SSHORTDATE = 5;
  { SetProcessWorkingSetSize  }
     PROCESS_SET_QUOTA = 256;
  { SetPrinter  }
  { SetService  }
  { SetStretchBltMode  }
     BLACKONWHITE = 1;
     COLORONCOLOR = 3;
     HALFTONE = 4;
     STRETCH_ANDSCANS = 1;
     STRETCH_DELETESCANS = 3;
     STRETCH_HALFTONE = 4;
     STRETCH_ORSCANS = 2;
     WHITEONBLACK = 2;
  { SetSystemCursor  }
     OCR_NORMAL = 32512;
     OCR_IBEAM = 32513;
     OCR_WAIT = 32514;
     OCR_CROSS = 32515;
     OCR_UP = 32516;
     OCR_SIZE = 32640;
     OCR_ICON = 32641;
     OCR_SIZENWSE = 32642;
     OCR_SIZENESW = 32643;
     OCR_SIZEWE = 32644;
     OCR_SIZENS = 32645;
     OCR_SIZEALL = 32646;
     OCR_NO = 32648;
     OCR_APPSTARTING = 32650;
  { SetTapePosition  }
     TAPE_ABSOLUTE_BLOCK = $1;
     TAPE_LOGICAL_BLOCK = $2;
     TAPE_REWIND = 0;
     TAPE_SPACE_END_OF_DATA = $4;
     TAPE_SPACE_FILEMARKS = $6;
     TAPE_SPACE_RELATIVE_BLOCKS = $5;
     TAPE_SPACE_SEQUENTIAL_FMKS = $7;
     TAPE_SPACE_SEQUENTIAL_SMKS = $9;
     TAPE_SPACE_SETMARKS = $8;
  { SetUnhandledExceptionFilter  }
     EXCEPTION_EXECUTE_HANDLER = 1;
     EXCEPTION_CONTINUE_EXECUTION = -(1);
     EXCEPTION_CONTINUE_SEARCH = 0;
  { SetWindowPos, DeferWindowPos  }
     HWND_BOTTOM = HWND(1);
     HWND_NOTOPMOST = HWND(-(2));
     HWND_TOP = HWND(0);
     HWND_TOPMOST = HWND(-(1));


  const
     SWP_DRAWFRAME = 32;
     SWP_FRAMECHANGED = 32;
     SWP_HIDEWINDOW = 128;
     SWP_NOACTIVATE = 16;
     SWP_NOCOPYBITS = 256;
     SWP_NOMOVE = 2;
     SWP_NOSIZE = 1;
     SWP_NOREDRAW = 8;
     SWP_NOZORDER = 4;
     SWP_SHOWWINDOW = 64;
     SWP_NOOWNERZORDER = 512;
     SWP_NOREPOSITION = 512;
     SWP_NOSENDCHANGING = 1024;
  { SHAddToRecentDocs  }
  { SHAppBarMessage  }
  { SHChangeNotify  }
  { ShellProc  }
     HSHELL_ACTIVATESHELLWINDOW = 3;
     HSHELL_GETMINRECT = 5;
     HSHELL_LANGUAGE = 8;
     HSHELL_REDRAW = 6;
     HSHELL_TASKMAN = 7;
     HSHELL_WINDOWACTIVATED = 4;
     HSHELL_WINDOWCREATED = 1;
     HSHELL_WINDOWDESTROYED = 2;
  { SHGetFileInfo  }
  // supported SHGetFileInfo input flags
     SHGFI_ICON              = $000000100;     // get icon
     SHGFI_DISPLAYNAME       = $000000200;     // get display name
     SHGFI_TYPENAME          = $000000400;     // get type name
     SHGFI_ATTRIBUTES        = $000000800;     // get attributes
     SHGFI_SYSICONINDEX      = $000004000;     // get system icon index
     SHGFI_LARGEICON         = $000000000;     // get large icon
     SHGFI_SMALLICON         = $000000001;     // get small icon
     SHGFI_USEFILEATTRIBUTES = $000000010;     // use passed dwFileAttribute
     SHGFI_PIDL              = $000000008;     // pszPath is a pidl
// supported SHGetFileInfo output attributes
     SFGAO_LINK              = $00010000;     // Shortcut (link)
     SFGAO_READONLY          = $00040000;     // read-only
     SFGAO_FOLDER            = $20000000;     // It's a folder.
  { SHGetSpecialFolderLocation  }
  { ShowWindow  }
     SW_HIDE = 0;
     SW_MAXIMIZE = 3;
     SW_MINIMIZE = 6;
     SW_NORMAL = 1;
     SW_RESTORE = 9;
     SW_SHOW = 5;
     SW_SHOWDEFAULT = 10;
     SW_SHOWMAXIMIZED = 3;
     SW_SHOWMINIMIZED = 2;
     SW_SHOWMINNOACTIVE = 7;
     SW_SHOWNA = 8;
     SW_SHOWNOACTIVATE = 4;
     SW_SHOWNORMAL = 1;
     WPF_RESTORETOMAXIMIZED = 2;
     WPF_SETMINPOSITION = 1;
  { SID }
     ////////////////////////////////////////////////////////////////////////
     //                                                                    //
     //              Security Id     (SID)                                 //
     //                                                                    //
     ////////////////////////////////////////////////////////////////////////
     //
     //
     // Pictorially the structure of an SID is as follows:
     //
     //         1   1   1   1   1   1
     //         5   4   3   2   1   0   9   8   7   6   5   4   3   2   1   0
     //      +---------------------------------------------------------------+
     //      |      SubAuthorityCount        |Reserved1 (SBZ)|   Revision    |
     //      +---------------------------------------------------------------+
     //      |                   IdentifierAuthority[0]                      |
     //      +---------------------------------------------------------------+
     //      |                   IdentifierAuthority[1]                      |
     //      +---------------------------------------------------------------+
     //      |                   IdentifierAuthority[2]                      |
     //      +---------------------------------------------------------------+
     //      |                                                               |
     //      +- -  -  -  -  -  -  -  SubAuthority[]  -  -  -  -  -  -  -  - -+
     //      |                                                               |
     //      +---------------------------------------------------------------+
     //
     //
     SID_REVISION =1;    // Current revision level
     SID_MAX_SUB_AUTHORITIES = 15;
     SID_RECOMMENDED_SUB_AUTHORITIES = 1;    // Will change to around 6
                                             // in a future release.


  { Sleep  }
     INFINITE = $FFFFFFFF;
  { SystemParametersInfo  }
     SPI_GETACCESSTIMEOUT = 60;
     SPI_GETANIMATION = 72;
     SPI_GETBEEP = 1;
     SPI_GETBORDER = 5;
     SPI_GETDEFAULTINPUTLANG = 89;
     SPI_GETDRAGFULLWINDOWS = 38;
     SPI_GETFASTTASKSWITCH = 35;
     SPI_GETFILTERKEYS = 50;
     SPI_GETFONTSMOOTHING = 74;
     SPI_GETGRIDGRANULARITY = 18;
     SPI_GETHIGHCONTRAST = 66;
     SPI_GETICONMETRICS = 45;
     SPI_GETICONTITLELOGFONT = 31;
     SPI_GETICONTITLEWRAP = 25;
     SPI_GETKEYBOARDDELAY = 22;
     SPI_GETKEYBOARDPREF = 68;
     SPI_GETKEYBOARDSPEED = 10;
     SPI_GETLOWPOWERACTIVE = 83;
     SPI_GETLOWPOWERTIMEOUT = 79;
     SPI_GETMENUDROPALIGNMENT = 27;
     SPI_GETMINIMIZEDMETRICS = 43;
     SPI_GETMOUSE = 3;
     SPI_GETMOUSEKEYS = 54;
     SPI_GETMOUSETRAILS = 94;
     SPI_GETNONCLIENTMETRICS = 41;
     SPI_GETPOWEROFFACTIVE = 84;
     SPI_GETPOWEROFFTIMEOUT = 80;
     SPI_GETSCREENREADER = 70;
     SPI_GETSCREENSAVEACTIVE = 16;
     SPI_GETSCREENSAVETIMEOUT = 14;
     SPI_GETSERIALKEYS = 62;
     SPI_GETSHOWSOUNDS = 56;
     SPI_GETSOUNDSENTRY = 64;
     SPI_GETSTICKYKEYS = 58;
     SPI_GETTOGGLEKEYS = 52;
     SPI_GETWINDOWSEXTENSION = 92;
     SPI_GETWORKAREA = 48;
     SPI_ICONHORIZONTALSPACING = 13;
     SPI_ICONVERTICALSPACING = 24;
     SPI_LANGDRIVER = 12;
     SPI_SCREENSAVERRUNNING = 97;
     SPI_SETACCESSTIMEOUT = 61;
     SPI_SETANIMATION = 73;
     SPI_SETBEEP = 2;
     SPI_SETBORDER = 6;
     SPI_SETDEFAULTINPUTLANG = 90;
     SPI_SETDESKPATTERN = 21;
     SPI_SETDESKWALLPAPER = 20;
     SPI_SETDOUBLECLICKTIME = 32;
     SPI_SETDOUBLECLKHEIGHT = 30;
     SPI_SETDOUBLECLKWIDTH = 29;
     SPI_SETDRAGFULLWINDOWS = 37;
     SPI_SETDRAGHEIGHT = 77;
     SPI_SETDRAGWIDTH = 76;
     SPI_SETFASTTASKSWITCH = 36;
     SPI_SETFILTERKEYS = 51;
     SPI_SETFONTSMOOTHING = 75;
     SPI_SETGRIDGRANULARITY = 19;
     SPI_SETHANDHELD = 78;
     SPI_SETHIGHCONTRAST = 67;
     SPI_SETICONMETRICS = 46;
     SPI_SETICONTITLELOGFONT = 34;
     SPI_SETICONTITLEWRAP = 26;
     SPI_SETKEYBOARDDELAY = 23;
     SPI_SETKEYBOARDPREF = 69;
     SPI_SETKEYBOARDSPEED = 11;
     SPI_SETLANGTOGGLE = 91;
     SPI_SETLOWPOWERACTIVE = 85;
     SPI_SETLOWPOWERTIMEOUT = 81;
     SPI_SETMENUDROPALIGNMENT = 28;
     SPI_SETMINIMIZEDMETRICS = 44;
     SPI_SETMOUSE = 4;
     SPI_SETMOUSEBUTTONSWAP = 33;
     SPI_SETMOUSEKEYS = 55;
     SPI_SETMOUSETRAILS = 93;
     SPI_SETNONCLIENTMETRICS = 42;
     SPI_SETPENWINDOWS = 49;
     SPI_SETPOWEROFFACTIVE = 86;
     SPI_SETPOWEROFFTIMEOUT = 82;
     SPI_SETSCREENREADER = 71;
     SPI_SETSCREENSAVEACTIVE = 17;
     SPI_SETSCREENSAVETIMEOUT = 15;
     SPI_SETSERIALKEYS = 63;
     SPI_SETSHOWSOUNDS = 57;
     SPI_SETSOUNDSENTRY = 65;
     SPI_SETSTICKYKEYS = 59;
     SPI_SETTOGGLEKEYS = 53;
     SPI_SETWORKAREA = 47;
     SPI_SETBATTERYIDLETIMEOUT = 251;
     SPI_GETBATTERYIDLETIMEOUT = 252;
     SPI_SETEXTERNALIDLETIMEOUT = 253;
     SPI_GETEXTERNALIDLETIMEOUT = 254;
     SPI_SETWAKEUPIDLETIMEOUT = 255;
     SPI_GETWAKEUPIDLETIMEOUT = 256;
     SPI_GETPLATFORMTYPE = 257;
     SPI_GETOEMINFO = 258;

     SPIF_UPDATEINIFILE = 1;
     SPIF_SENDWININICHANGE = 2;
     SPIF_SENDCHANGE = 2;
  { TrackPopupMenu, TrackPopMenuEx  }
     TPM_CENTERALIGN = $4;
     TPM_LEFTALIGN = 0;
     TPM_RIGHTALIGN = $8;
     TPM_LEFTBUTTON = 0;
     TPM_RIGHTBUTTON = $2;
     TPM_HORIZONTAL = 0;
     TPM_BOTTOMALIGN = $20;
     TPM_VERTICAL = $40;
<<<<<<< HEAD
<<<<<<< HEAD
     TPM_NONOTIFY        = $0080;     //* Don't send any notification msgs */
     TPM_RETURNCMD       = $0100;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TranslateCharsetInfo  }
     TCI_SRCCHARSET = 1;
     TCI_SRCCODEPAGE = 2;
     TCI_SRCFONTSIG = 3;
  { WideCharToMultiByte  }
     WC_COMPOSITECHECK = 512;
     WC_DISCARDNS = 16;
     WC_SEPCHARS = 32;
     WC_DEFAULTCHAR = 64;
  { WinHelp  }
     HELP_COMMAND = $102;
     HELP_CONTENTS = $3;
     HELP_CONTEXT = $1;
     HELP_CONTEXTPOPUP = $8;
     HELP_FORCEFILE = $9;
     HELP_HELPONHELP = $4;
     HELP_INDEX = $3;
     HELP_KEY = $101;
     HELP_MULTIKEY = $201;
     HELP_PARTIALKEY = $105;
     HELP_QUIT = $2;
     HELP_SETCONTENTS = $5;
     HELP_SETINDEX = $5;
     HELP_CONTEXTMENU = $a;
     HELP_FINDER = $b;
     HELP_WM_HELP = $c;
     HELP_TCARD = $8000;
     HELP_TCARD_DATA = $10;
     HELP_TCARD_OTHER_CALLER = $11;
  { WNetAddConnectino2  }
     CONNECT_UPDATE_PROFILE = 1;
  { WNetConnectionDialog, WNetDisconnectDialog, WNetOpenEnum  }
     RESOURCETYPE_DISK = 1;
     RESOURCETYPE_PRINT = 2;
     RESOURCETYPE_ANY = 0;
     RESOURCE_CONNECTED = 1;
     RESOURCE_GLOBALNET = 2;
     RESOURCE_REMEMBERED = 3;
     RESOURCEUSAGE_CONNECTABLE = 1;
     RESOURCEUSAGE_CONTAINER = 2;
  { WNetGetResourceInformation, WNetGetResourceParent  }
     WN_BAD_NETNAME = $43;
     WN_EXTENDED_ERROR = $4b8;
     WN_MORE_DATA = $ea;
     WN_NO_NETWORK = $4c6;
     WN_SUCCESS = 0;
     WN_ACCESS_DENIED = $5;
     WN_BAD_PROVIDER = $4b4;
     WN_NOT_AUTHENTICATED = $4dc;
  { WNetGetUniversalName  }
     UNIVERSAL_NAME_INFO_LEVEL = 1;
     REMOTE_NAME_INFO_LEVEL = 2;
  { GetExitCodeThread  }
     STILL_ACTIVE = $103;

  { kfuncs consts }  //+kfuncs

    {$ifdef CPUARM}
      PUserKData = $FFFFC800;
    {$else}
      PUserKData = $00005800;
    {$endif CPUARM}

     EVENT_PULSE = 1;
     EVENT_RESET = 2;
     EVENT_SET = 3;

     SYSHANDLE_OFFSET = $004;
     NUM_SYS_HANDLES  = 32;
     SYS_HANDLE_BASE  = 64;
     SH_WIN32         = 0;
     SH_CURTHREAD     = 1;
     SH_CURPROC       = 2;
     SH_LAST_NOTIFY   = 16; // Last set notified on Thread/Process Termination
     SH_GDI           = 16;
     SH_WMGR          = 17;
     SH_WNET        = 18; // WNet APIs for network redirector
     SH_COMM          = 19; // Communications not "COM"
     SH_FILESYS_APIS  = 20; // File system APIS
     SH_SHELL         = 21;
     SH_DEVMGR_APIS   = 22; // File system device manager
     SH_TAPI        = 23;
     SH_PATCHER       = 24;
     SH_SERVICES      = 26;
     SH_LASTRESERVED  = 26;

     TLS_FUNCALLOC    = 0;
     TLS_FUNCFREE     = 1;

  { COMMPROP structure  }
     SP_SERIALCOMM = $1;
     BAUD_075 = $1;
     BAUD_110 = $2;
     BAUD_134_5 = $4;
     BAUD_150 = $8;
     BAUD_300 = $10;
     BAUD_600 = $20;
     BAUD_1200 = $40;
     BAUD_1800 = $80;
     BAUD_2400 = $100;
     BAUD_4800 = $200;
     BAUD_7200 = $400;
     BAUD_9600 = $800;
     BAUD_14400 = $1000;
     BAUD_19200 = $2000;
     BAUD_38400 = $4000;
     BAUD_56K = $8000;
     BAUD_57600 = $40000;
     BAUD_115200 = $20000;
     BAUD_128K = $10000;
     BAUD_USER = $10000000;
     PST_FAX = $21;
     PST_LAT = $101;
     PST_MODEM = $6;
     PST_NETWORK_BRIDGE = $100;
     PST_PARALLELPORT = $2;
     PST_RS232 = $1;
     PST_RS422 = $3;
     PST_RS423 = $4;
     PST_RS449 = $5;
     PST_SCANNER = $22;
     PST_TCPIP_TELNET = $102;
     PST_UNSPECIFIED = 0;
     PST_X25 = $103;
     PCF_16BITMODE = $200;
     PCF_DTRDSR = $1;
     PCF_INTTIMEOUTS = $80;
     PCF_PARITY_CHECK = $8;
     PCF_RLSD = $4;
     PCF_RTSCTS = $2;
     PCF_SETXCHAR = $20;
     PCF_SPECIALCHARS = $100;
     PCF_TOTALTIMEOUTS = $40;
     PCF_XONXOFF = $10;
     SP_BAUD = $2;
     SP_DATABITS = $4;
     SP_HANDSHAKING = $10;
     SP_PARITY = $1;
     SP_PARITY_CHECK = $20;
     SP_RLSD = $40;
     SP_STOPBITS = $8;
     DATABITS_5 = 1;
     DATABITS_6 = 2;
     DATABITS_7 = 4;
     DATABITS_8 = 8;
     DATABITS_16 = 16;
     DATABITS_16X = 32;
     STOPBITS_10 = 1;
     STOPBITS_15 = 2;
     STOPBITS_20 = 4;
     PARITY_NONE = 256;
     PARITY_ODD = 512;
     PARITY_EVEN = 1024;
     PARITY_MARK = 2048;
     PARITY_SPACE = 4096;
     COMMPROP_INITIALIZED = $e73cf52e;
  { DCB structure  }
     CBR_110 = 110;
     CBR_300 = 300;
     CBR_600 = 600;
     CBR_1200 = 1200;
     CBR_2400 = 2400;
     CBR_4800 = 4800;
     CBR_9600 = 9600;
     CBR_14400 = 14400;
     CBR_19200 = 19200;
     CBR_38400 = 38400;
     CBR_56000 = 56000;
     CBR_57600 = 57600;
     CBR_115200 = 115200;
     CBR_128000 = 128000;
     CBR_256000 = 256000;
     DTR_CONTROL_DISABLE = 0;
     DTR_CONTROL_ENABLE = 1;
     DTR_CONTROL_HANDSHAKE = 2;
     RTS_CONTROL_DISABLE = 0;
     RTS_CONTROL_ENABLE = 1;
     RTS_CONTROL_HANDSHAKE = 2;
     RTS_CONTROL_TOGGLE = 3;
     EVENPARITY = 2;
     MARKPARITY = 3;
     NOPARITY = 0;
     ODDPARITY = 1;
     SPACEPARITY = 4;
     ONESTOPBIT = 0;
     ONE5STOPBITS = 1;
     TWOSTOPBITS = 2;
  { Debugging events  }
     CREATE_PROCESS_DEBUG_EVENT = 3;
     CREATE_THREAD_DEBUG_EVENT = 2;
     EXCEPTION_DEBUG_EVENT = 1;
     EXIT_PROCESS_DEBUG_EVENT = 5;
     EXIT_THREAD_DEBUG_EVENT = 4;
     LOAD_DLL_DEBUG_EVENT = 6;
     OUTPUT_DEBUG_STRING_EVENT = 8;
     UNLOAD_DLL_DEBUG_EVENT = 7;
     RIP_EVENT = 9;
  { PROCESS_HEAP_ENTRY structure  }
     PROCESS_HEAP_REGION = 1;
     PROCESS_HEAP_UNCOMMITTED_RANGE = 2;
     PROCESS_HEAP_ENTRY_BUSY = 4;
     PROCESS_HEAP_ENTRY_MOVEABLE = 16;
     PROCESS_HEAP_ENTRY_DDESHARE = 32;
  { Win32s  }
     HINSTANCE_ERROR = 32;
  { WIN32_STREAM_ID structure  }
     BACKUP_DATA = 1;
     BACKUP_EA_DATA = 2;
     BACKUP_SECURITY_DATA = 3;
     BACKUP_ALTERNATE_DATA = 4;
     BACKUP_LINK = 5;
     STREAM_MODIFIED_WHEN_READ = 1;
     STREAM_CONTAINS_SECURITY = 2;
  { STARTUPINFO structure  }
     STARTF_USESHOWWINDOW = 1;
     STARTF_USEPOSITION = 4;
     STARTF_USESIZE = 2;
     STARTF_USECOUNTCHARS = 8;
     STARTF_USEFILLATTRIBUTE = 16;
     STARTF_RUNFULLSCREEN = 32;
     STARTF_FORCEONFEEDBACK = 64;
     STARTF_FORCEOFFFEEDBACK = 128;
     STARTF_USESTDHANDLES = 256;
     STARTF_USEHOTKEY = 512;
  { OSVERSIONINFO structure  }
     VER_PLATFORM_WIN32s = 0;
     VER_PLATFORM_WIN32_WINDOWS = 1;
     VER_PLATFORM_WIN32_NT = 2;
  { More versions }
     VER_SERVER_NT                      = $80000000;  //winnt
     VER_WORKSTATION_NT                 = $40000000;  //winnt
     VER_SUITE_SMALLBUSINESS            = $00000001;  //winnt
     VER_SUITE_ENTERPRISE               = $00000002;  //winnt
     VER_SUITE_BACKOFFICE               = $00000004;  //winnt
     VER_SUITE_COMMUNICATIONS           = $00000008;  //winnt
     VER_SUITE_TERMINAL                 = $00000010;  //winnt
     VER_SUITE_SMALLBUSINESS_RESTRICTED = $00000020;  //winnt
     VER_SUITE_EMBEDDEDNT               = $00000040;
     VER_SUITE_DATACENTER               = $00000080;
     VER_SUITE_SINGLEUSERTS             = $00000100;
     VER_SUITE_PERSONAL                 = $00000200;
     VER_SUITE_BLADE                    = $00000400;
     VER_SUITE_EMBEDDED_RESTRICTED      = $00000800;
  { PROPSHEETPAGE structure  }
     MAXPROPPAGES = 100;
     PSP_DEFAULT = 0;
     PSP_DLGINDIRECT = 1;
     PSP_HASHELP = 32;
     PSP_USECALLBACK = 128;
     PSP_USEHICON = 2;
     PSP_USEICONID = 4;
     PSP_USEREFPARENT = 64;
     PSP_USETITLE = 8;
     PSP_RTLREADING = 16;
     PSP_PREMATURE = $0400; //prsht

  { PROPSHEETHEADER structure  }  //prsht
     PSH_DEFAULT = 0;
     PSH_PROPTITLE = 1;
     PSH_USEHICON = 2;
     PSH_USEICONID = 4;
     PSH_PROPSHEETPAGE = 8;
     PSH_WIZARDHASFINISH = $0010;
     PSH_WIZARD = 32;
     PSH_USEPSTARTPAGE = 64;
     PSH_NOAPPLYNOW = 128;
     PSH_USECALLBACK = 256;
     PSH_HASHELP = 512;
     PSH_MODELESS = 1024;
     PSH_RTLREADING = 2048;
     PSH_WIZARDCONTEXTHELP = $1000;
     PSH_MAXIMIZE          = $2000;
     PSH_NOCANCELBUTTON    = $80000000; //specific ppc

     PSCB_INITIALIZED = 1;
     PSCB_PRECREATE = 2;
     PSCB_GETVERSION = 3;
     PSCB_GETTITLE   = 4;
     PSCB_GETLINKTEXT = 5;

  { PSN_APPLY message  } //prsht
     PSNRET_NOERROR = 0;
     PSNRET_INVALID  = 1;
     PSNRET_INVALID_NOCHANGEPAGE = 2;

  { Property Sheet  }
     PROP_SM_CXDLG = 212;
     PROP_SM_CYDLG = 188;
     PROP_MED_CXDLG = 227;
     PROP_MED_CYDLG = 215;
     PROP_LG_CXDLG = 252;
     PROP_LG_CYDLG = 218;

     PSBTN_APPLYNOW = 4;
     PSBTN_BACK = 0;
     PSBTN_CANCEL = 5;
     PSBTN_FINISH = 2;
     PSBTN_HELP = 6;
     PSBTN_MAX  = 6; //prsht
     PSBTN_NEXT = 1;
     PSBTN_OK = 3;

     PSWIZB_BACK = 1;
     PSWIZB_NEXT = 2;
     PSWIZB_FINISH = 4;
     PSWIZB_DISABLEDFINISH = 8;

     ID_PSREBOOTSYSTEM = 3;
     ID_PSRESTARTWINDOWS = 2;
     WIZ_BODYCX = 184;
     WIZ_BODYX = 92;
     WIZ_CXBMP = 80;
     WIZ_CXDLG = 276;
     WIZ_CYDLG = 140;

  { VX_FIXEDFILEINFO structure  }
     VS_FILE_INFO = MAKEINTRESOURCE(16);


  const
     VS_VERSION_INFO = 1;
     VS_FF_DEBUG = $1;
     VS_FF_INFOINFERRED = $10;
     VS_FF_PATCHED = $4;
     VS_FF_PRERELEASE = $2;
     VS_FF_PRIVATEBUILD = $8;
     VS_FF_SPECIALBUILD = $20;
     VOS_UNKNOWN = 0;
     VOS_DOS = $10000;
     VOS_OS216 = $20000;
     VOS_OS232 = $30000;
     VOS_NT = $40000;
     VOS_DOS_WINDOWS16 = $10001;
     VOS_DOS_WINDOWS32 = $10004;
     VOS_OS216_PM16 = $20002;
     VOS_OS232_PM32 = $30003;
     VOS_NT_WINDOWS32 = $40004;
     VFT_UNKNOWN = 0;
     VFT_APP = $1;
     VFT_DLL = $2;
     VFT_DRV = $3;
     VFT_FONT = $4;
     VFT_VXD = $5;
     VFT_STATIC_LIB = $7;
     VFT2_UNKNOWN = 0;
     VFT2_DRV_PRINTER = $1;
     VFT2_DRV_KEYBOARD = $2;
     VFT2_DRV_LANGUAGE = $3;
     VFT2_DRV_DISPLAY = $4;
     VFT2_DRV_MOUSE = $5;
     VFT2_DRV_NETWORK = $6;
     VFT2_DRV_SYSTEM = $7;
     VFT2_DRV_INSTALLABLE = $8;
     VFT2_DRV_SOUND = $9;
     VFT2_FONT_RASTER = $1;
     VFT2_FONT_VECTOR = $2;
     VFT2_FONT_TRUETYPE = $3;
  { PANOSE structure  }
     PAN_ANY = 0;
     PAN_NO_FIT = 1;
     PAN_FAMILY_TEXT_DISPLAY = 2;
     PAN_FAMILY_SCRIPT = 3;
     PAN_FAMILY_DECORATIVE = 4;
     PAN_FAMILY_PICTORIAL = 5;
     PAN_SERIF_COVE = 2;
     PAN_SERIF_OBTUSE_COVE = 3;
     PAN_SERIF_SQUARE_COVE = 4;
     PAN_SERIF_OBTUSE_SQUARE_COVE = 5;
     PAN_SERIF_SQUARE = 6;
     PAN_SERIF_THIN = 7;
     PAN_SERIF_BONE = 8;
     PAN_SERIF_EXAGGERATED = 9;
     PAN_SERIF_TRIANGLE = 10;
     PAN_SERIF_NORMAL_SANS = 11;
     PAN_SERIF_OBTUSE_SANS = 12;
     PAN_SERIF_PERP_SANS = 13;
     PAN_SERIF_FLARED = 14;
     PAN_SERIF_ROUNDED = 15;
     PAN_WEIGHT_VERY_LIGHT = 2;
     PAN_WEIGHT_LIGHT = 3;
     PAN_WEIGHT_THIN = 4;
     PAN_WEIGHT_BOOK = 5;
     PAN_WEIGHT_MEDIUM = 6;
     PAN_WEIGHT_DEMI = 7;
     PAN_WEIGHT_BOLD = 8;
     PAN_WEIGHT_HEAVY = 9;
     PAN_WEIGHT_BLACK = 10;
     PAN_WEIGHT_NORD = 11;
     PAN_PROP_OLD_STYLE = 2;
     PAN_PROP_MODERN = 3;
     PAN_PROP_EVEN_WIDTH = 4;
     PAN_PROP_EXPANDED = 5;
     PAN_PROP_CONDENSED = 6;
     PAN_PROP_VERY_EXPANDED = 7;
     PAN_PROP_VERY_CONDENSED = 8;
     PAN_PROP_MONOSPACED = 9;
     PAN_CONTRAST_NONE = 2;
     PAN_CONTRAST_VERY_LOW = 3;
     PAN_CONTRAST_LOW = 4;
     PAN_CONTRAST_MEDIUM_LOW = 5;
     PAN_CONTRAST_MEDIUM = 6;
     PAN_CONTRAST_MEDIUM_HIGH = 7;
     PAN_CONTRAST_HIGH = 8;
     PAN_CONTRAST_VERY_HIGH = 9;
     PAN_STROKE_GRADUAL_DIAG = 2;
     PAN_STROKE_GRADUAL_TRAN = 3;
     PAN_STROKE_GRADUAL_VERT = 4;
     PAN_STROKE_GRADUAL_HORZ = 5;
     PAN_STROKE_RAPID_VERT = 6;
     PAN_STROKE_RAPID_HORZ = 7;
     PAN_STROKE_INSTANT_VERT = 8;
     PAN_STRAIGHT_ARMS_HORZ = 2;
     PAN_STRAIGHT_ARMS_WEDGE = 3;
     PAN_STRAIGHT_ARMS_VERT = 4;
     PAN_STRAIGHT_ARMS_SINGLE_SERIF = 5;
     PAN_STRAIGHT_ARMS_DOUBLE_SERIF = 6;
     PAN_BENT_ARMS_HORZ = 7;
     PAN_BENT_ARMS_VERT = 9;
     PAN_BENT_ARMS_WEDGE = 8;
     PAN_BENT_ARMS_SINGLE_SERIF = 10;
     PAN_BENT_ARMS_DOUBLE_SERIF = 11;
     PAN_LETT_NORMAL_CONTACT = 2;
     PAN_LETT_NORMAL_WEIGHTED = 3;
     PAN_LETT_NORMAL_BOXED = 4;
     PAN_LETT_NORMAL_FLATTENED = 5;
     PAN_LETT_NORMAL_ROUNDED = 6;
     PAN_LETT_NORMAL_OFF_CENTER = 7;
     PAN_LETT_NORMAL_SQUARE = 8;
     PAN_LETT_OBLIQUE_CONTACT = 9;
     PAN_LETT_OBLIQUE_WEIGHTED = 10;
     PAN_LETT_OBLIQUE_BOXED = 11;
     PAN_LETT_OBLIQUE_FLATTENED = 12;
     PAN_LETT_OBLIQUE_ROUNDED = 13;
     PAN_LETT_OBLIQUE_OFF_CENTER = 14;
     PAN_LETT_OBLIQUE_SQUARE = 15;
     PAN_MIDLINE_STANDARD_TRIMMED = 2;
     PAN_MIDLINE_STANDARD_POINTED = 3;
     PAN_MIDLINE_STANDARD_SERIFED = 4;
     PAN_MIDLINE_HIGH_TRIMMED = 5;
     PAN_MIDLINE_HIGH_POINTED = 6;
     PAN_MIDLINE_HIGH_SERIFED = 7;
     PAN_MIDLINE_CONSTANT_TRIMMED = 8;
     PAN_MIDLINE_CONSTANT_POINTED = 9;
     PAN_MIDLINE_CONSTANT_SERIFED = 10;
     PAN_MIDLINE_LOW_TRIMMED = 11;
     PAN_MIDLINE_LOW_POINTED = 12;
     PAN_MIDLINE_LOW_SERIFED = 13;
     PAN_XHEIGHT_CONSTANT_SMALL = 2;
     PAN_XHEIGHT_CONSTANT_STD = 3;
     PAN_XHEIGHT_CONSTANT_LARGE = 4;
     PAN_XHEIGHT_DUCKING_SMALL = 5;
     PAN_XHEIGHT_DUCKING_STD = 6;
     PAN_XHEIGHT_DUCKING_LARGE = 7;
  { PALETTENTRY structure  }
     PC_EXPLICIT = 2;
     PC_NOCOLLAPSE = 4;
     PC_RESERVED = 1;
  { LOGBRUSH structure  }
     BS_DIBPATTERN = 5;
     BS_DIBPATTERN8X8 = 8;
     BS_DIBPATTERNPT = 6;
     BS_HATCHED = 2;
     BS_HOLLOW = 1;
     BS_NULL = 1;
     BS_PATTERN = 3;
     BS_PATTERN8X8 = 7;
     BS_SOLID = 0;
  { DEVMODE structure  }
     DM_ORIENTATION = $1;
     DM_PAPERSIZE = $2;
     DM_PAPERLENGTH = $4;
     DM_PAPERWIDTH = $8;
     DM_SCALE = $10;
     DM_COPIES = $100;
     DM_DEFAULTSOURCE = $200;
     DM_PRINTQUALITY = $400;
     DM_COLOR = $800;
     DM_DUPLEX = $1000;
     DM_YRESOLUTION = $2000;
     DM_TTOPTION = $4000;
     DM_COLLATE = $8000;
     DM_FORMNAME = $10000;
     DM_LOGPIXELS = $20000;
     {DM_BITSPERPEL = $40000;
     DM_PELSWIDTH = $80000;
     DM_PELSHEIGHT = $100000;
     DM_DISPLAYFLAGS = $200000;
     DM_DISPLAYFREQUENCY = $400000;already above }
     DM_ICMMETHOD = $800000;
     DM_ICMINTENT = $1000000;
     DM_MEDIATYPE = $2000000;
     DM_DITHERTYPE = $4000000;
   { orientation selections }
     DMORIENT_LANDSCAPE = 2;
     DMORIENT_PORTRAIT = 1;
   { paper selections }
     DMPAPER_LETTER = 1;
     DMPAPER_LEGAL = 5;
     DMPAPER_A4 = 9;
     DMPAPER_CSHEET = 24;
     DMPAPER_DSHEET = 25;
     DMPAPER_ESHEET = 26;
     DMPAPER_LETTERSMALL = 2;
     DMPAPER_TABLOID = 3;
     DMPAPER_LEDGER = 4;
     DMPAPER_STATEMENT = 6;
     DMPAPER_EXECUTIVE = 7;
     DMPAPER_A3 = 8;
     DMPAPER_A4SMALL = 10;
     DMPAPER_A5 = 11;
     DMPAPER_B4 = 12;
     DMPAPER_B5 = 13;
     DMPAPER_FOLIO = 14;
     DMPAPER_QUARTO = 15;
     DMPAPER_10X14 = 16;
     DMPAPER_11X17 = 17;
     DMPAPER_NOTE = 18;
     DMPAPER_ENV_9 = 19;
     DMPAPER_ENV_10 = 20;
     DMPAPER_ENV_11 = 21;
     DMPAPER_ENV_12 = 22;
     DMPAPER_ENV_14 = 23;
     DMPAPER_ENV_DL = 27;
     DMPAPER_ENV_C5 = 28;
     DMPAPER_ENV_C3 = 29;
     DMPAPER_ENV_C4 = 30;
     DMPAPER_ENV_C6 = 31;
     DMPAPER_ENV_C65 = 32;
     DMPAPER_ENV_B4 = 33;
     DMPAPER_ENV_B5 = 34;
     DMPAPER_ENV_B6 = 35;
     DMPAPER_ENV_ITALY = 36;
     DMPAPER_ENV_MONARCH = 37;
     DMPAPER_ENV_PERSONAL = 38;
     DMPAPER_FANFOLD_US = 39;
     DMPAPER_FANFOLD_STD_GERMAN = 40;
     DMPAPER_FANFOLD_LGL_GERMAN = 41;
  { print qualities }
     DMRES_HIGH = -(4);
     DMRES_MEDIUM = -(3);
     DMRES_LOW = -(2);
     DMRES_DRAFT = -(1);
  { color enable/disable for color printers }
     DMCOLOR_COLOR = 2;
     DMCOLOR_MONOCHROME = 1;
     DMDUP_SIMPLEX = 1;
     DMDUP_HORIZONTAL = 3;
     DMDUP_VERTICAL = 2;
     DMTT_BITMAP = 1;
     DMTT_DOWNLOAD = 2;
     DMTT_SUBDEV = 3;
     DMCOLLATE_TRUE = 1;
     DMCOLLATE_FALSE = 0;
     DM_GRAYSCALE = 1;
     DM_INTERLACED = 2;
     DMICMMETHOD_NONE = 1;
     DMICMMETHOD_SYSTEM = 2;
     DMICMMETHOD_DRIVER = 3;
     DMICMMETHOD_DEVICE = 4;
     DMICMMETHOD_USER = 256;
     DMICM_SATURATE = 1;
     DMICM_CONTRAST = 2;
     DMICM_COLORMETRIC = 3;
     DMICM_USER = 256;
     DMMEDIA_STANDARD = 1;
     DMMEDIA_GLOSSY = 3;
     DMMEDIA_TRANSPARENCY = 2;
     DMMEDIA_USER = 256;
     DMDITHER_NONE = 1;
     DMDITHER_COARSE = 2;
     DMDITHER_FINE = 3;
     DMDITHER_LINEART = 4;
     DMDITHER_GRAYSCALE = 10;
     DMDITHER_USER = 256;
  { RGNDATAHEADER structure  }
     RDH_RECTANGLES = 1;
  { TTPOLYGONHEADER structure  }
     TT_POLYGON_TYPE = 24;
  { TTPOLYCURVE structure  }
     TT_PRIM_LINE = 1;
     TT_PRIM_QSPLINE = 2;
  { GCP_RESULTS structure  }
     GCPCLASS_ARABIC = 2;
     GCPCLASS_HEBREW = 2;
     GCPCLASS_LATIN = 1;
     GCPCLASS_LATINNUMBER = 5;
     GCPCLASS_LOCALNUMBER = 4;
     GCPCLASS_LATINNUMERICSEPARATOR = 7;
     GCPCLASS_LATINNUMERICTERMINATOR = 6;
     GCPCLASS_NEUTRAL = 3;
     GCPCLASS_NUMERICSEPARATOR = 8;
     GCPCLASS_PREBOUNDLTR = 128;
     GCPCLASS_PREBOUNDRTL = 64;
     GCPCLASS_POSTBOUNDLTR = 32;
     GCPCLASS_POSTBOUNDRTL = 16;
     GCPGLYPH_LINKBEFORE = 32768;
     GCPGLYPH_LINKAFTER = 16384;
  { RASTERIZER_STATUS structure  }
     TT_AVAILABLE = 1;
     TT_ENABLED = 2;
  { COLORADJUSTMENT structure  }
     CA_NEGATIVE = 1;
     CA_LOG_FILTER = 2;
     ILLUMINANT_DEVICE_DEFAULT = 0;
     ILLUMINANT_A = 1;
     ILLUMINANT_B = 2;
     ILLUMINANT_C = 3;
     ILLUMINANT_D50 = 4;
     ILLUMINANT_D55 = 5;
     ILLUMINANT_D65 = 6;
     ILLUMINANT_D75 = 7;
     ILLUMINANT_F2 = 8;
     ILLUMINANT_TUNGSTEN = 1;
     ILLUMINANT_DAYLIGHT = 3;
     ILLUMINANT_FLUORESCENT = 8;
     ILLUMINANT_NTSC = 3;
  { DOCINFO structure  }
     DI_APPBANDING = 1;
  { EMRMETAHEADER structure  }
     EMR_HEADER = 1;
     ENHMETA_SIGNATURE = 1179469088;
  { RTF event masks  }
     ENM_CHANGE = 1;
     ENM_CORRECTTEXT = 4194304;
     ENM_DROPFILES = 1048576;
     ENM_KEYEVENTS = 65536;
     ENM_MOUSEEVENTS = 131072;
     ENM_PROTECTED = 2097152;
     ENM_REQUESTRESIZE = 262144;
     ENM_SCROLL = 4;
     ENM_SELCHANGE = 524288;
     ENM_UPDATE = 2;
     ENM_NONE = 0;
  { RTF styles  }
     ES_DISABLENOSCROLL = 8192;
     ES_EX_NOCALLOLEINIT = 16777216;
     ES_NOIME = 524288;
     ES_SAVESEL = 32768;
     ES_SELFIME = 262144;
     ES_SUNKEN = 16384;
     ES_VERTICAL = 4194304;
     ES_SELECTIONBAR = 16777216;
  { EM_SETOPTIONS message  }
     ECOOP_SET = 1;
     ECOOP_OR = 2;
     ECOOP_AND = 3;
     ECOOP_XOR = 4;
     ECO_AUTOWORDSELECTION = 1;
     ECO_AUTOVSCROLL = 64;
     ECO_AUTOHSCROLL = 128;
     ECO_NOHIDESEL = 256;
     ECO_READONLY = 2048;
     ECO_WANTRETURN = 4096;
     ECO_SAVESEL = 32768;
     ECO_SELECTIONBAR = 16777216;
     ECO_VERTICAL = 4194304;
  { EM_SETCHARFORMAT message  }
     SCF_WORD = 2;
     SCF_SELECTION = 1;
  { EM_STREAMOUT message  }
     SF_TEXT = 1;
     SF_RTF = 2;
     SF_RTFNOOBJS = 3;
     SF_TEXTIZED = 4;
     SFF_SELECTION = 32768;
     SFF_PLAINRTF = 16384;
  { EM_FINDWORDBREAK message  }
     WB_CLASSIFY = 3;
     {WB_ISDELIMITER = 2;
     WB_LEFT = 0; already above }
     WB_LEFTBREAK = 6;
     WB_PREVBREAK = 6;
     WB_MOVEWORDLEFT = 4;
     WB_MOVEWORDPREV = 4;
     WB_MOVEWORDRIGHT = 5;
     WB_MOVEWORDNEXT = 5;
     {WB_RIGHT = 1;already above }
     WB_RIGHTBREAK = 7;
     WB_NEXTBREAK = 7;
  { EM_GETPUNCTUATION message  }
     PC_LEADING = 2;
     PC_FOLLOWING = 1;
     PC_DELIMITER = 4;
     PC_OVERFLOW = 3;
  { EM_SETWORDWRAPMODE message  }
     WBF_WORDWRAP = 16;
     WBF_WORDBREAK = 32;
     WBF_OVERFLOW = 64;
     WBF_LEVEL1 = 128;
     WBF_LEVEL2 = 256;
     WBF_CUSTOM = 512;
     WBF_BREAKAFTER = 64;
     WBF_BREAKLINE = 32;
     WBF_ISWHITE = 16;
  { CHARFORMAT structure  }
     CFM_BOLD = 1;
     CFM_COLOR = 1073741824;
     CFM_FACE = 536870912;
     CFM_ITALIC = 2;
     CFM_OFFSET = 268435456;
     CFM_PROTECTED = 16;
     CFM_SIZE = $80000000;
     CFM_STRIKEOUT = 8;
     CFM_UNDERLINE = 4;
     CFE_AUTOCOLOR = 1073741824;
     CFE_BOLD = 1;
     CFE_ITALIC = 2;
     CFE_STRIKEOUT = 8;
     CFE_UNDERLINE = 4;
     CFE_PROTECTED = 16;
  { PARAFORMAT structure  }
     PFM_ALIGNMENT = 8;
     PFM_NUMBERING = 32;
     PFM_OFFSET = 4;
     PFM_OFFSETINDENT = $80000000;
     PFM_RIGHTINDENT = 2;
     PFM_STARTINDENT = 1;
     PFM_TABSTOPS = 16;
     PFN_BULLET = 1;
     PFA_LEFT = 1;
     PFA_RIGHT = 2;
     PFA_CENTER = 3;
  { SELCHANGE structure  }
     SEL_EMPTY = 0;
     SEL_TEXT = 1;
     SEL_OBJECT = 2;
     SEL_MULTICHAR = 4;
     SEL_MULTIOBJECT = 8;
  { RTF clipboard formats  }
     CF_RTF = 'Rich Text Format';
     CF_RETEXTOBJ = 'RichEdit Text and Objects';
  { DRAWITEMSTRUCT structure  }
     ODT_BUTTON = 4;
     ODT_COMBOBOX = 3;
     ODT_LISTBOX = 2;
     ODT_LISTVIEW = 102;
     ODT_MENU = 1;
     ODT_STATIC = 5;
     ODT_TAB = 101;
     ODT_HEADER = 100;
     ODA_DRAWENTIRE = 1;
     ODA_FOCUS = 4;
     ODA_SELECT = 2;
     ODS_CHECKED = 8;
     ODS_COMBOBOXEDIT = 4096;
     ODS_DEFAULT = 32;
     ODS_DISABLED = 4;
     ODS_FOCUS = 16;
     ODS_GRAYED = 2;
     ODS_SELECTED = 1;
  { Common control window classes  }
     ANIMATE_CLASSW = 'SysAnimate32';
     HOTKEY_CLASSW = 'msctls_hotkey32';
     PROGRESS_CLASSW = 'msctls_progress32';
     STATUSCLASSNAMEW = 'msctls_statusbar32';
     TOOLBARCLASSNAMEW = 'ToolbarWindow32';
     TOOLTIPS_CLASSW = 'tooltips_class32';
     TRACKBAR_CLASSW = 'msctls_trackbar32';
     UPDOWN_CLASSW = 'msctls_updown32';
     WC_HEADERW = 'SysHeader32';
     WC_LISTVIEWW = 'SysListView32';
     WC_TABCONTROLW = 'SysTabControl32';
     WC_TREEVIEWW = 'SysTreeView32';
  { Common control styles  }
     CCS_ADJUSTABLE = $20;
     CCS_BOTTOM = $3;
     CCS_NODIVIDER = $40;
     CCS_NOMOVEY = $2;
     CCS_NOPARENTALIGN = $8;
     CCS_NORESIZE = $4;
     CCS_TOP = $1;
     CCS_VERT = $80;
     ANIMATE_CLASSA = 'SysAnimate32';
     HOTKEY_CLASSA = 'msctls_hotkey32';
     PROGRESS_CLASSA = 'msctls_progress32';
     STATUSCLASSNAMEA = 'msctls_statusbar32';
     TOOLBARCLASSNAMEA = 'ToolbarWindow32';
     TOOLTIPS_CLASSA = 'tooltips_class32';
     TRACKBAR_CLASSA = 'msctls_trackbar32';
     UPDOWN_CLASSA = 'msctls_updown32';
     WC_HEADERA = 'SysHeader32';
     WC_LISTVIEWA = 'SysListView32';
     WC_TABCONTROLA = 'SysTabControl32';
     WC_TREEVIEWA = 'SysTreeView32';
{$ifdef UNICODE}

  const
     ANIMATE_CLASS = ANIMATE_CLASSW;
     HOTKEY_CLASS = HOTKEY_CLASSW;
     PROGRESS_CLASS = PROGRESS_CLASSW;
     STATUSCLASSNAME = STATUSCLASSNAMEW;
     TOOLBARCLASSNAME = TOOLBARCLASSNAMEW;
     TOOLTIPS_CLASS = TOOLTIPS_CLASSW;
     TRACKBAR_CLASS = TRACKBAR_CLASSW;
     UPDOWN_CLASS = UPDOWN_CLASSW;
     WC_HEADER = WC_HEADERW;
     WC_LISTVIEW = WC_LISTVIEWW;
     WC_TABCONTROL = WC_TABCONTROLW;
     WC_TREEVIEW = WC_TREEVIEWW;
{$else}

  const
     ANIMATE_CLASS = ANIMATE_CLASSA;
     HOTKEY_CLASS = HOTKEY_CLASSA;
     PROGRESS_CLASS = PROGRESS_CLASSA;
     STATUSCLASSNAME = STATUSCLASSNAMEA;
     TOOLBARCLASSNAME = TOOLBARCLASSNAMEA;
     TOOLTIPS_CLASS = TOOLTIPS_CLASSA;
     TRACKBAR_CLASS = TRACKBAR_CLASSA;
     UPDOWN_CLASS = UPDOWN_CLASSA;
     WC_HEADER = WC_HEADERA;
     WC_LISTVIEW = WC_LISTVIEWA;
     WC_TABCONTROL = WC_TABCONTROLA;
     WC_TREEVIEW = WC_TREEVIEWA;
{$endif}
  { UNICODE  }
  { Header control styles  }

  const
     HDS_BUTTONS = 2;
     HDS_HIDDEN = 8;
     HDS_HORZ = 0;
  { HD_ITEM structure  }
     HDI_BITMAP = 16;
     HDI_FORMAT = 4;
     HDI_HEIGHT = 1;
     HDI_LPARAM = 8;
     HDI_TEXT = 2;
     HDI_WIDTH = 1;
     HDF_CENTER = 2;
     HDF_LEFT = 0;
     HDF_RIGHT = 1;
     HDF_RTLREADING = 4;
     HDF_BITMAP = 8192;
     HDF_OWNERDRAW = 32768;
     HDF_STRING = 16384;
     HDF_JUSTIFYMASK = 3;
  { HD_HITTESTINFO structure  }
     HHT_NOWHERE = 1;
     HHT_ONDIVIDER = 4;
     HHT_ONDIVOPEN = 8;
     HHT_ONHEADER = 2;
     HHT_TOLEFT = 2048;
     HHT_TORIGHT = 1024;
  { TBADDBITMAP structure  }
     HINST_COMMCTRL = HINST(-(1));


  const
     IDB_STD_LARGE_COLOR = 1;
     IDB_STD_SMALL_COLOR = 0;
     IDB_VIEW_LARGE_COLOR = 5;
     IDB_VIEW_SMALL_COLOR = 4;
     STD_COPY = 1;
     STD_CUT = 0;
     STD_DELETE = 5;
     STD_FILENEW = 6;
     STD_FILEOPEN = 7;
     STD_FILESAVE = 8;
     STD_FIND = 12;
     STD_HELP = 11;
     STD_PASTE = 2;
     STD_PRINT = 14;
     STD_PRINTPRE = 9;
     STD_PROPERTIES = 10;
     STD_REDOW = 4;
     STD_REPLACE = 13;
     STD_UNDO = 3;
     VIEW_LARGEICONS = 0;
     VIEW_SMALLICONS = 1;
     VIEW_LIST = 2;
     VIEW_DETAILS = 3;
     VIEW_SORTNAME = 4;
     VIEW_SORTSIZE = 5;
     VIEW_SORTDATE = 6;
     VIEW_SORTTYPE = 7;
  { Toolbar styles  }
     TBSTYLE_ALTDRAG = 1024;
     TBSTYLE_TOOLTIPS = 256;
     TBSTYLE_WRAPABLE = 512;
     TBSTYLE_BUTTON = 0;
     TBSTYLE_CHECK = 2;
     TBSTYLE_CHECKGROUP = 6;
     TBSTYLE_GROUP = 4;
     TBSTYLE_SEP = 1;
     TBSTYLE_DROPDOWN        = $0008;
     TBSTYLE_AUTOSIZE        = $0010; // automatically calculate the cx of the button
     TBSTYLE_FLAT            = $0800;
     TBSTYLE_LIST            = $1000;
     TBSTYLE_CUSTOMERASE     = $2000;
     TBSTYLE_TRANSPARENT     = $8000;
  { Toolbar states  }
     TBSTATE_CHECKED = 1;
     TBSTATE_ENABLED = 4;
     TBSTATE_HIDDEN = 8;
     TBSTATE_INDETERMINATE = 16;
     TBSTATE_PRESSED = 2;
     TBSTATE_WRAP = 32;
     TBSTATE_ELLIPSES        = $40;
     TBSTATE_HIGHLIGHTED     = $80;
  { Tooltip styles  }
     TTS_ALWAYSTIP = 1;
     TTS_NOPREFIX = 2;
  { TOOLINFO structure  }
     TTF_IDISHWND = 1;
     TTF_CENTERTIP = 2;
     TTF_RTLREADING = 4;
     TTF_SUBCLASS = 16;
  { TTM_SETDELAYTIME message  }
     TTDT_AUTOMATIC = 0;
     TTDT_AUTOPOP = 2;
     TTDT_INITIAL = 3;
     TTDT_RESHOW = 1;
  { Status window  }
     SBARS_SIZEGRIP = 256;
     {SBARS_SIZEGRIP = 256;already above }
  { DL_DRAGGING message  }
     DL_MOVECURSOR = 3;
     DL_COPYCURSOR = 2;
     DL_STOPCURSOR = 1;
  { Up-down control styles  }
     UDS_ALIGNLEFT = 8;
     UDS_ALIGNRIGHT = 4;
     UDS_ARROWKEYS = 32;
     UDS_AUTOBUDDY = 16;
     UDS_HORZ = 64;
     UDS_NOTHOUSANDS = 128;
     UDS_SETBUDDYINT = 2;
     UDS_WRAP = 1;
  { UDM_SETRANGE message  }
     UD_MAXVAL = 32767;
     UD_MINVAL = -(32767);
  { HKM_GETHOTKEY message  }
     HOTKEYF_ALT = 4;
     HOTKEYF_CONTROL = 2;
     HOTKEYF_EXT = 8;
     HOTKEYF_SHIFT = 1;
  { HKM_SETRULES message  }
     HKCOMB_A = 8;
     HKCOMB_C = 4;
     HKCOMB_CA = 64;
     HKCOMB_NONE = 1;
     HKCOMB_S = 2;
     HKCOMB_SA = 32;
     HKCOMB_SC = 16;
     HKCOMB_SCA = 128;
  { Trackbar styles  }
     TBS_HORZ = 0;
     TBS_VERT = 2;
     TBS_AUTOTICKS = 1;
     TBS_NOTICKS = 16;
     TBS_TOP = 4;
     TBS_BOTTOM = 0;
     TBS_LEFT = 4;
     TBS_RIGHT = 0;
     TBS_BOTH = 8;
     TBS_ENABLESELRANGE = 32;
     TBS_FIXEDLENGTH = 64;
     TBS_NOTHUMB = 128;
     TB_BOTTOM = 7;
     TB_ENDTRACK = 8;
     TB_LINEDOWN = 1;
     TB_LINEUP = 0;
     TB_PAGEDOWN = 3;
     TB_PAGEUP = 2;
     TB_THUMBPOSITION = 4;
     TB_THUMBTRACK = 5;
     TB_TOP = 6;
  { List view styles  }
     LVS_ALIGNLEFT = 2048;
     LVS_ALIGNTOP = 0;
     LVS_AUTOARRANGE = 256;
     LVS_EDITLABELS = 512;
     LVS_ICON = 0;
     LVS_LIST = 3;
     LVS_NOCOLUMNHEADER = 16384;
     LVS_NOLABELWRAP = 128;
     LVS_NOSCROLL = 8192;
     LVS_NOSORTHEADER = 32768;
     LVS_OWNERDRAWFIXED = 1024;
     LVS_OWNERDATA = $1000;
     LVS_REPORT = 1;
     LVS_SHAREIMAGELISTS = 64;
     LVS_SHOWSELALWAYS = 8;
     LVS_SINGLESEL = 4;
     LVS_SMALLICON = 2;
     LVS_SORTASCENDING = 16;
     LVS_SORTDESCENDING = 32;
     LVS_TYPESTYLEMASK = 64512;
     //extended styles
     LVS_EX_GRIDLINES        = $00000001;
     LVS_EX_SUBITEMIMAGES    = $00000002;
     LVS_EX_CHECKBOXES       = $00000004;
     LVS_EX_TRACKSELECT      = $00000008;
     LVS_EX_HEADERDRAGDROP   = $00000010;
     LVS_EX_FULLROWSELECT    = $00000020; // applies to report mode only
     LVS_EX_ONECLICKACTIVATE = $00000040;
     LVS_EX_NOHSCROLL        = $10000000;
     LVS_EX_GRADIENT         = $20000000; // draw a background gradient

     LVSIL_NORMAL = 0;
     LVSIL_SMALL = 1;
     LVSIL_STATE = 2;
     LVIS_CUT = 4;
     LVIS_DROPHILITED = 8;
     LVIS_FOCUSED = 1;
     LVIS_SELECTED = 2;
     LVIS_OVERLAYMASK = 3840;
     LVIS_STATEIMAGEMASK = 61440;

     // progress bar styles
     PBS_SMOOTH              = 01;
     PBS_VERTICAL            = 04;

     LPSTR_TEXTCALLBACKW = LPWSTR(-(1));
     LPSTR_TEXTCALLBACKA = LPSTR(-(1));


  {$ifdef UNICODE}

     {const this is a function in fact !!
              LPSTR_TEXTCALLBACK = LPSTR_TEXTCALLBACKW;}
     LPSTR_TEXTCALLBACK          = LPWSTR(-(1));

  {$else}

   {const
   LPSTR_TEXTCALLBACK = LPSTR_TEXTCALLBACKA; }
     LPSTR_TEXTCALLBACK          =LPSTR(-(1));
  {$endif}

   { LV_ITEM structure  }

  const
     LVIF_TEXT = 1;
     LVIF_IMAGE = 2;
     LVIF_PARAM = 4;
     LVIF_STATE = 8;
     LVIF_INDENT = $10;
     LVIF_DI_SETITEM = 4096;
  { LVM_GETNEXTITEM structure  }
     LVNI_ABOVE = 256;
     LVNI_ALL = 0;
     LVNI_BELOW = 512;
     LVNI_TOLEFT = 1024;
     LVNI_TORIGHT = 2048;
     LVNI_CUT = 4;
     LVNI_DROPHILITED = 8;
     LVNI_FOCUSED = 1;
     LVNI_SELECTED = 2;
  { LV_FINDINFO structure  }
     LVFI_PARAM = 1;
     LVFI_PARTIAL = 8;
     LVFI_STRING = 2;
     LVFI_WRAP = 32;
     LVFI_NEARESTXY = 64;
  { LV_HITTESTINFO structure  }
     LVHT_ABOVE = 8;
     LVHT_BELOW = 16;
     LVHT_NOWHERE = 1;
     LVHT_ONITEMICON = 2;
     LVHT_ONITEMLABEL = 4;
     LVHT_ONITEMSTATEICON = 8;
     LVHT_TOLEFT = 64;
     LVHT_TORIGHT = 32;
     LVHT_ONITEM = LVHT_ONITEMICON or LVHT_ONITEMLABEL or LVHT_ONITEMSTATEICON;
  { LV_COLUMN structure  }
     LVCF_FMT = 1;
     LVCF_SUBITEM = 8;
     LVCF_TEXT = 4;
     LVCF_WIDTH = 2;
     LVCF_IMAGE = $10;
     LVCF_ORDER = $20;
     LVCFMT_CENTER = 2;
     LVCFMT_LEFT = 0;
     LVCFMT_RIGHT = 1;
     LVCFMT_JUSTIFYMASK      = $0003;
     LVCFMT_IMAGE            = $0800;
     LVCFMT_BITMAP_ON_RIGHT  = $1000;
  { ListView_GetItemRect  }
     LVIR_BOUNDS = 0;
     LVIR_ICON = 1;
     LVIR_LABEL = 2;
     LVIR_SELECTBOUNDS = 3;
  { LVM_ARRANGE message  }
     LVA_ALIGNLEFT = 1;
     LVA_ALIGNTOP = 2;
     LVA_DEFAULT = 0;
     LVA_SNAPTOGRID = 5;
  { LVM_SETCOLUMNWIDTH message  }
     LVSCW_AUTOSIZE = -(1);
     LVSCW_AUTOSIZE_USEHEADER = -(2);
  { Tree View styles  }
     TVS_DISABLEDRAGDROP = 16;
     TVS_EDITLABELS = 8;
     TVS_HASBUTTONS = 1;
     TVS_HASLINES = 2;
     TVS_LINESATROOT = 4;
     TVS_SHOWSELALWAYS = 32;
     TVS_CHECKBOXES = $0100;
     TVS_SINGLEEXPAND = $0400;
  { Tree View states  }
     TVIS_BOLD = 16;
     TVIS_CUT = 4;
     TVIS_DROPHILITED = 8;
     TVIS_EXPANDED = 32;
     TVIS_EXPANDEDONCE = 64;
     TVIS_FOCUSED = 1;
     TVIS_OVERLAYMASK = 3840;
     TVIS_SELECTED = 2;
     TVIS_STATEIMAGEMASK = 61440;
     TVIS_USERMASK = 61440;
  { TV_ITEM structure  }
     TVIF_CHILDREN = 64;
     TVIF_HANDLE = 16;
     TVIF_IMAGE = 2;
     TVIF_PARAM = 4;
     TVIF_SELECTEDIMAGE = 32;
     TVIF_STATE = 8;
     TVIF_TEXT = 1;
     I_CHILDRENCALLBACK = -(1);
     I_IMAGECALLBACK = -(1);
  { TV_INSERTSTRUCT structure  }
    { added manually PM, TREEITEM is not defined in the C headers }
     type
       HTREEITEM = THandle;

  const
    TVI_ROOT = HTREEITEM($FFFF0000);
    TVI_FIRST =HTREEITEM($FFFF0001);
    TVI_LAST = HTREEITEM($FFFF0002);
    TVI_SORT = HTREEITEM($FFFF0003);

  { TV_HITTESTINFO structure  }

  const
     TVHT_ABOVE = 256;
     TVHT_BELOW = 512;
     TVHT_NOWHERE = 1;
     TVHT_ONITEM = 70;
     TVHT_ONITEMBUTTON = 16;
     TVHT_ONITEMICON = 2;
     TVHT_ONITEMINDENT = 8;
     TVHT_ONITEMLABEL = 4;
     TVHT_ONITEMRIGHT = 32;
     TVHT_ONITEMSTATEICON = 64;
     TVHT_TOLEFT = 2048;
     TVHT_TORIGHT = 1024;
  { TVM_EXPAND message  }
     TVE_COLLAPSE = 1;
     TVE_COLLAPSERESET = 32768;
     TVE_EXPAND = 2;
     TVE_TOGGLE = 3;
  { TVM_GETIMAGELIST message  }
     TVSIL_NORMAL = 0;
     TVSIL_STATE = 2;
  { TVM_GETNEXTITEM message  }
     TVGN_CARET = 9;
     TVGN_CHILD = 4;
     TVGN_DROPHILITE = 8;
     TVGN_FIRSTVISIBLE = 5;
     TVGN_NEXT = 1;
     TVGN_NEXTVISIBLE = 6;
     TVGN_PARENT = 3;
     TVGN_PREVIOUS = 2;
     TVGN_PREVIOUSVISIBLE = 7;
     TVGN_ROOT = 0;
  { TVN_SELCHANGED message  }
     TVC_BYKEYBOARD = 2;
     TVC_BYMOUSE = 1;
     TVC_UNKNOWN = 0;
  { Tab control styles  } //commctrl
     TCS_RIGHTJUSTIFY = 0;
     TCS_SINGLELINE = 0;
     TCS_TABS = 0;
     TCS_SCROLLOPPOSITE = $0001;  // assumes multiline tab
     TCS_BOTTOM         = $0002;
     TCS_RIGHT          = $0002;
     TCS_MULTISELECT    = $0004;  // allow multi-select in button mode
     TCS_FLATBUTTONS    = $0008;
     TCS_FORCEICONLEFT  = 16;
     TCS_FORCELABELLEFT = 32;
     TCS_HOTTRACK       = $0040;
     TCS_VERTICAL       = $0080;
     TCS_BUTTONS = 256;
     TCS_MULTILINE = 512;
     TCS_FIXEDWIDTH = 1024;
     TCS_RAGGEDRIGHT = 2048;
     TCS_FOCUSONBUTTONDOWN = 4096;
     TCS_OWNERDRAWFIXED = 8192;
     TCS_TOOLTIPS= 16384;
     TCS_FOCUSNEVER = 32768;

     TCS_EX_FLATSEPARATORS = $00000001;

  { TC_ITEM structure  }
     TCIF_TEXT = 1;
     TCIF_IMAGE = 2;
     TCIF_PARAM = 8;
     TCIF_RTLREADING = 4;
  { TC_HITTESTINFO structure  }
     TCHT_NOWHERE = 1;
     TCHT_ONITEM = 6;
     TCHT_ONITEMICON = 2;
     TCHT_ONITEMLABEL = 4;
  { Animation control styles  }
     ACS_AUTOPLAY = 4;
     ACS_CENTER = 1;
     ACS_TRANSPARENT = 2;
<<<<<<< HEAD
<<<<<<< HEAD
     ACS_TIMER       = 8;  // don't use threads... use timers
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { MODEMDEVCAPS structure  }
     DIALOPTION_BILLING = 64;
     DIALOPTION_QUIET = 128;
     DIALOPTION_DIALTONE = 256;
     MDMVOLFLAG_LOW = 1;
     MDMVOLFLAG_MEDIUM = 2;
     MDMVOLFLAG_HIGH = 4;
     MDMVOL_LOW = 0;
     MDMVOL_MEDIUM = 1;
     MDMVOL_HIGH = 2;
     MDMSPKRFLAG_OFF = 1;
     MDMSPKRFLAG_DIAL = 2;
     MDMSPKRFLAG_ON = 4;
     MDMSPKRFLAG_CALLSETUP = 8;
     MDMSPKR_OFF = 0;
     MDMSPKR_DIAL = 1;
     MDMSPKR_ON = 2;
     MDMSPKR_CALLSETUP = 3;
     MDM_BLIND_DIAL = 512;
     MDM_CCITT_OVERRIDE = 64;
     MDM_CELLULAR = 8;
     MDM_COMPRESSION = 1;
     MDM_ERROR_CONTROL = 2;
     MDM_FLOWCONTROL_HARD = 16;
     MDM_FLOWCONTROL_SOFT = 32;
     MDM_FORCED_EC = 4;
     MDM_SPEED_ADJUST = 128;
     MDM_TONE_DIAL = 256;
     MDM_V23_OVERRIDE = 1024;
  { Languages  }
   //
   //  Language IDs.
   //
   //  The following two combinations of primary language ID and
   //  sublanguage ID have special semantics:
   //
   //    Primary Language ID   Sublanguage ID      Result
   //    -------------------   ---------------     ------------------------
   //    LANG_NEUTRAL          SUBLANG_NEUTRAL     Language neutral
   //    LANG_NEUTRAL          SUBLANG_DEFAULT     User default language
   //    LANG_NEUTRAL          SUBLANG_SYS_DEFAULT System default language
   //    LANG_INVARIANT        SUBLANG_NEUTRAL     Invariant locale
   //

   //
   //  Primary language IDs.
   //

     LANG_NEUTRAL    = $00;
     LANG_INVARIANT  = $7f;

     LANG_AFRIKAANS  = $36;
     LANG_ALBANIAN   = $1c;
     LANG_ARABIC     = $01;
     LANG_ARMENIAN   = $2b;
     LANG_ASSAMESE   = $4d;
     LANG_AZERI      = $2c;
     LANG_BASQUE     = $2d;
     LANG_BELARUSIAN = $23;
     LANG_BENGALI    = $45;
     LANG_BULGARIAN  = $02;
     LANG_CATALAN    = $03;
     LANG_CHINESE    = $04;
     LANG_CROATIAN   = $1a;
     LANG_CZECH      = $05;
     LANG_DANISH     = $06;
     LANG_DIVEHI     = $65;
     LANG_DUTCH      = $13;
     LANG_ENGLISH    = $09;
     LANG_ESTONIAN   = $25;
     LANG_FAEROESE   = $38;
     LANG_FARSI      = $29;
     LANG_FINNISH    = $0b;
     LANG_FRENCH     = $0c;
     LANG_GALICIAN   = $56;
     LANG_GEORGIAN   = $37;
     LANG_GERMAN     = $07;
     LANG_GREEK      = $08;
     LANG_GUJARATI   = $47;
     LANG_HEBREW     = $0d;
     LANG_HINDI      = $39;
     LANG_HUNGARIAN  = $0e;
     LANG_ICELANDIC  = $0f;
     LANG_INDONESIAN = $21;
     LANG_ITALIAN    = $10;
     LANG_JAPANESE   = $11;
     LANG_KANNADA    = $4b;
     LANG_KASHMIRI   = $60;
     LANG_KAZAK      = $3f;
     LANG_KONKANI    = $57;
     LANG_KOREAN     = $12;
     LANG_KYRGYZ     = $40;
     LANG_LATVIAN    = $26;
     LANG_LITHUANIAN = $27;
     LANG_MACEDONIAN = $2f;            // the Former Yugoslav Republic of Macedonia
     LANG_MALAY      = $3e;
     LANG_MALAYALAM  = $4c;
     LANG_MANIPURI   = $58;
     LANG_MARATHI    = $4e;
     LANG_MONGOLIAN  = $50;
     LANG_NEPALI     = $61;
     LANG_NORWEGIAN  = $14;
     LANG_ORIYA      = $48;
     LANG_POLISH     = $15;
     LANG_PORTUGUESE = $16;
     LANG_PUNJABI    = $46;
     LANG_ROMANIAN   = $18;
     LANG_RUSSIAN    = $19;
     LANG_SANSKRIT   = $4f;
     LANG_SERBIAN    = $1a;
     LANG_SINDHI     = $59;
     LANG_SLOVAK     = $1b;
     LANG_SLOVENIAN  = $24;
     LANG_SPANISH    = $0a;
     LANG_SWAHILI    = $41;
     LANG_SWEDISH    = $1d;
     LANG_SYRIAC     = $5a;
     LANG_TAMIL      = $49;
     LANG_TATAR      = $44;
     LANG_TELUGU     = $4a;
     LANG_THAI       = $1e;
     LANG_TURKISH    = $1f;
     LANG_UKRAINIAN  = $22;
     LANG_URDU       = $20;
     LANG_UZBEK      = $43;
     LANG_VIETNAMESE = $2a;

   //
   //  Sublanguage IDs.
   //
   //  The name immediately following SUBLANG_ dictates which primary
   //  language ID that sublanguage ID can be combined with to form a
   //  valid language ID.
   //

     SUBLANG_NEUTRAL     = $00; // language neutral
     SUBLANG_DEFAULT     = $01; // user default
     SUBLANG_SYS_DEFAULT = $02; // system default

     SUBLANG_ARABIC_SAUDI_ARABIA        = $01; // Arabic (Saudi Arabia)
     SUBLANG_ARABIC_IRAQ                = $02; // Arabic (Iraq)
     SUBLANG_ARABIC_EGYPT               = $03; // Arabic (Egypt)
     SUBLANG_ARABIC_LIBYA               = $04; // Arabic (Libya)
     SUBLANG_ARABIC_ALGERIA             = $05; // Arabic (Algeria)
     SUBLANG_ARABIC_MOROCCO             = $06; // Arabic (Morocco)
     SUBLANG_ARABIC_TUNISIA             = $07; // Arabic (Tunisia)
     SUBLANG_ARABIC_OMAN                = $08; // Arabic (Oman)
     SUBLANG_ARABIC_YEMEN               = $09; // Arabic (Yemen)
     SUBLANG_ARABIC_SYRIA               = $0a; // Arabic (Syria)
     SUBLANG_ARABIC_JORDAN              = $0b; // Arabic (Jordan)
     SUBLANG_ARABIC_LEBANON             = $0c; // Arabic (Lebanon)
     SUBLANG_ARABIC_KUWAIT              = $0d; // Arabic (Kuwait)
     SUBLANG_ARABIC_UAE                 = $0e; // Arabic (U.A.E)
     SUBLANG_ARABIC_BAHRAIN             = $0f; // Arabic (Bahrain)
     SUBLANG_ARABIC_QATAR               = $10; // Arabic (Qatar)
     SUBLANG_AZERI_LATIN                = $01; // Azeri (Latin)
     SUBLANG_AZERI_CYRILLIC             = $02; // Azeri (Cyrillic)
     SUBLANG_CHINESE_TRADITIONAL        = $01; // Chinese (Taiwan)
     SUBLANG_CHINESE_SIMPLIFIED         = $02; // Chinese (PR China)
     SUBLANG_CHINESE_HONGKONG           = $03; // Chinese (Hong Kong S.A.R., P.R.C.)
     SUBLANG_CHINESE_SINGAPORE          = $04; // Chinese (Singapore)
     SUBLANG_CHINESE_MACAU              = $05; // Chinese (Macau S.A.R.)
     SUBLANG_DUTCH                      = $01; // Dutch
     SUBLANG_DUTCH_BELGIAN              = $02; // Dutch (Belgian)
     SUBLANG_ENGLISH_US                 = $01; // English (USA)
     SUBLANG_ENGLISH_UK                 = $02; // English (UK)
     SUBLANG_ENGLISH_AUS                = $03; // English (Australian)
     SUBLANG_ENGLISH_CAN                = $04; // English (Canadian)
     SUBLANG_ENGLISH_NZ                 = $05; // English (New Zealand)
     SUBLANG_ENGLISH_EIRE               = $06; // English (Irish)
     SUBLANG_ENGLISH_SOUTH_AFRICA       = $07; // English (South Africa)
     SUBLANG_ENGLISH_JAMAICA            = $08; // English (Jamaica)
     SUBLANG_ENGLISH_CARIBBEAN          = $09; // English (Caribbean)
     SUBLANG_ENGLISH_BELIZE             = $0a; // English (Belize)
     SUBLANG_ENGLISH_TRINIDAD           = $0b; // English (Trinidad)
     SUBLANG_ENGLISH_ZIMBABWE           = $0c; // English (Zimbabwe)
     SUBLANG_ENGLISH_PHILIPPINES        = $0d; // English (Philippines)
     SUBLANG_FRENCH                     = $01; // French
     SUBLANG_FRENCH_BELGIAN             = $02; // French (Belgian)
     SUBLANG_FRENCH_CANADIAN            = $03; // French (Canadian)
     SUBLANG_FRENCH_SWISS               = $04; // French (Swiss)
     SUBLANG_FRENCH_LUXEMBOURG          = $05; // French (Luxembourg)
     SUBLANG_FRENCH_MONACO              = $06; // French (Monaco)
     SUBLANG_GERMAN                     = $01; // German
     SUBLANG_GERMAN_SWISS               = $02; // German (Swiss)
     SUBLANG_GERMAN_AUSTRIAN            = $03; // German (Austrian)
     SUBLANG_GERMAN_LUXEMBOURG          = $04; // German (Luxembourg)
     SUBLANG_GERMAN_LIECHTENSTEIN       = $05; // German (Liechtenstein)
     SUBLANG_ITALIAN                    = $01; // Italian
     SUBLANG_ITALIAN_SWISS              = $02; // Italian (Swiss)
     SUBLANG_KASHMIRI_SASIA             = $02; // Kashmiri (South Asia)
     SUBLANG_KASHMIRI_INDIA             = $02; // For app compatibility only
     SUBLANG_KOREAN                     = $01; // Korean (Extended Wansung)
     SUBLANG_LITHUANIAN                 = $01; // Lithuanian
     SUBLANG_MALAY_MALAYSIA             = $01; // Malay (Malaysia)
     SUBLANG_MALAY_BRUNEI_DARUSSALAM    = $02; // Malay (Brunei Darussalam)
     SUBLANG_NEPALI_INDIA               = $02; // Nepali (India)
     SUBLANG_NORWEGIAN_BOKMAL           = $01; // Norwegian (Bokmal)
     SUBLANG_NORWEGIAN_NYNORSK          = $02; // Norwegian (Nynorsk)
     SUBLANG_PORTUGUESE                 = $02; // Portuguese
     SUBLANG_PORTUGUESE_BRAZILIAN       = $01; // Portuguese (Brazilian)
     SUBLANG_SERBIAN_LATIN              = $02; // Serbian (Latin)
     SUBLANG_SERBIAN_CYRILLIC           = $03; // Serbian (Cyrillic)
     SUBLANG_SPANISH                    = $01; // Spanish (Castilian)
     SUBLANG_SPANISH_MEXICAN            = $02; // Spanish (Mexican)
     SUBLANG_SPANISH_MODERN             = $03; // Spanish (Spain)
     SUBLANG_SPANISH_GUATEMALA          = $04; // Spanish (Guatemala)
     SUBLANG_SPANISH_COSTA_RICA         = $05; // Spanish (Costa Rica)
     SUBLANG_SPANISH_PANAMA             = $06; // Spanish (Panama)
     SUBLANG_SPANISH_DOMINICAN_REPUBLIC = $07; // Spanish (Dominican Republic)
     SUBLANG_SPANISH_VENEZUELA          = $08; // Spanish (Venezuela)
     SUBLANG_SPANISH_COLOMBIA           = $09; // Spanish (Colombia)
     SUBLANG_SPANISH_PERU               = $0a; // Spanish (Peru)
     SUBLANG_SPANISH_ARGENTINA          = $0b; // Spanish (Argentina)
     SUBLANG_SPANISH_ECUADOR            = $0c; // Spanish (Ecuador)
     SUBLANG_SPANISH_CHILE              = $0d; // Spanish (Chile)
     SUBLANG_SPANISH_URUGUAY            = $0e; // Spanish (Uruguay)
     SUBLANG_SPANISH_PARAGUAY           = $0f; // Spanish (Paraguay)
     SUBLANG_SPANISH_BOLIVIA            = $10; // Spanish (Bolivia)
     SUBLANG_SPANISH_EL_SALVADOR        = $11; // Spanish (El Salvador)
     SUBLANG_SPANISH_HONDURAS           = $12; // Spanish (Honduras)
     SUBLANG_SPANISH_NICARAGUA          = $13; // Spanish (Nicaragua)
     SUBLANG_SPANISH_PUERTO_RICO        = $14; // Spanish (Puerto Rico)
     SUBLANG_SWEDISH                    = $01; // Swedish
     SUBLANG_SWEDISH_FINLAND            = $02; // Swedish (Finland)
     SUBLANG_URDU_PAKISTAN              = $01; // Urdu (Pakistan)
     SUBLANG_URDU_INDIA                 = $02; // Urdu (India)
     SUBLANG_UZBEK_LATIN                = $01; // Uzbek (Latin)
     SUBLANG_UZBEK_CYRILLIC             = $02; // Uzbek (Cyrillic)

   //
   //  Sorting IDs.
   //

     SORT_DEFAULT = $0; // sorting default

     SORT_JAPANESE_XJIS    = $0; // Japanese XJIS order
     SORT_JAPANESE_UNICODE = $1; // Japanese Unicode order

     SORT_CHINESE_BIG5     = $0; // Chinese BIG5 order
     SORT_CHINESE_PRCP     = $0; // PRC Chinese Phonetic order
     SORT_CHINESE_UNICODE  = $1; // Chinese Unicode order
     SORT_CHINESE_PRC      = $2; // PRC Chinese Stroke Count order
     SORT_CHINESE_BOPOMOFO = $3; // Traditional Chinese Bopomofo order

     SORT_KOREAN_KSC     = $0; // Korean KSC order
     SORT_KOREAN_UNICODE = $1; // Korean Unicode order

     SORT_GERMAN_PHONE_BOOK = $1; // German Phone Book order

     SORT_HUNGARIAN_DEFAULT   = $0; // Hungarian Default order
     SORT_HUNGARIAN_TECHNICAL = $1; // Hungarian Technical order

     SORT_GEORGIAN_TRADITIONAL = $0; // Georgian Traditional order
     SORT_GEORGIAN_MODERN      = $1; // Georgian Modern order


  { SYSTEM_INFO structure  }
     PROCESSOR_INTEL_386 = 386;
     PROCESSOR_INTEL_486 = 486;
     PROCESSOR_INTEL_PENTIUM = 586;
     PROCESSOR_MIPS_R4000 = 4000; // incl R4101 & R3910 for Windows CE
     PROCESSOR_ALPHA_21064 = 21064;

     PROCESSOR_INTEL_PENTIUMII = 686;
     PROCESSOR_MIPS_R5000      = 5000;  // incl R5432 for Windows CE
     PROCESSOR_PPC_403         = 403;
     PROCESSOR_PPC_601         = 601;
     PROCESSOR_PPC_603         = 603;
     PROCESSOR_PPC_604         = 604;
     PROCESSOR_PPC_620         = 620;
     PROCESSOR_HITACHI_SH3     = 10003;   // Windows CE
     PROCESSOR_HITACHI_SH3E    = 10004;   // Windows CE
     PROCESSOR_HITACHI_SH4     = 10005;   // Windows CE
     PROCESSOR_MOTOROLA_821    = 821;     // Windows CE
     PROCESSOR_SHx_SH3         = 103;     // Windows CE
     PROCESSOR_SHx_SH3DSP      = 105;     // Windows CE
     PROCESSOR_SHx_SH4         = 104;     // Windows CE
     PROCESSOR_STRONGARM       = 2577;    // Windows CE - 0xA11
     PROCESSOR_ARM720          = 1824;    // Windows CE - 0x720
     PROCESSOR_ARM820          = 2080;    // Windows CE - 0x820
     PROCESSOR_ARM920          = 2336;    // Windows CE - 0x920
     PROCESSOR_ARM_7TDMI       = 70001;   // Windows CE

     PROCESSOR_ARCHITECTURE_INTEL = 0;
     PROCESSOR_ARCHITECTURE_MIPS  = 1;
     PROCESSOR_ARCHITECTURE_ALPHA = 2;
     PROCESSOR_ARCHITECTURE_PPC   = 3;
     PROCESSOR_ARCHITECTURE_SHX   = 4;
     PROCESSOR_ARCHITECTURE_ARM   = 5;
     PROCESSOR_ARCHITECTURE_IA64  = 6;
     PROCESSOR_ARCHITECTURE_ALPHA64 = 7;
     PROCESSOR_ARCHITECTURE_UNKNOWN = $FFFF;

     PROCESSOR_X86_32BIT_CORE                = 1;

     PROCESSOR_MIPS16_CORE                   = 1;
     PROCESSOR_MIPSII_CORE                   = 2;
     PROCESSOR_MIPSIV_CORE                   = 3;

     PROCESSOR_HITACHI_SH3_CORE              = 1;
     PROCESSOR_HITACHI_SH4_CORE              = 2;

     PROCESSOR_ARM_V4_CORE                   = 1;
     PROCESSOR_ARM_V4I_CORE                  = 2;
     PROCESSOR_ARM_V4T_CORE                  = 3;

     PROCESSOR_FEATURE_NOFP                  = 0;
     PROCESSOR_FEATURE_FP                    = 1;
     PROCESSOR_FEATURE_DSP                   = PROCESSOR_FEATURE_FP;

     PROCESSOR_QUERY_INSTRUCTION             = $00000000;

{
#define PROCESSOR_INSTRUCTION_CODE(arch, core, feature)      \
    ((arch) << 24 | (core) << 16 | (feature))
}
     
     PROCESSOR_X86_32BIT_INSTRUCTION = (PROCESSOR_ARCHITECTURE_INTEL shl 24) or
                                       (PROCESSOR_X86_32BIT_CORE shl 16) or
                                       PROCESSOR_FEATURE_FP;

     PROCESSOR_MIPS_MIPS16_INSTRUCTION = (PROCESSOR_ARCHITECTURE_MIPS shl 24) or
                                         (PROCESSOR_MIPS16_CORE shl 16) or
                                         PROCESSOR_FEATURE_NOFP;

     PROCESSOR_MIPS_MIPSII_INSTRUCTION = (PROCESSOR_ARCHITECTURE_MIPS shl 24) or
                                         (PROCESSOR_MIPSII_CORE shl 16) or
                                         PROCESSOR_FEATURE_NOFP;

     PROCESSOR_MIPS_MIPSIIFP_INSTRUCTION = (PROCESSOR_ARCHITECTURE_MIPS shl 24) or
                                           (PROCESSOR_MIPSII_CORE shl 16) or
                                           PROCESSOR_FEATURE_FP;

     PROCESSOR_MIPS_MIPSIV_INSTRUCTION = (PROCESSOR_ARCHITECTURE_MIPS shl 24) or
                                         (PROCESSOR_MIPSIV_CORE shl 16) or
                                         PROCESSOR_FEATURE_NOFP;

     PROCESSOR_MIPS_MIPSIVFP_INSTRUCTION = (PROCESSOR_ARCHITECTURE_MIPS shl 24) or
                                           (PROCESSOR_MIPSIV_CORE shl 16) or
                                           PROCESSOR_FEATURE_FP;

     PROCESSOR_HITACHI_SH3_INSTRUCTION = (PROCESSOR_ARCHITECTURE_SHX shl 24) or
                                         (PROCESSOR_HITACHI_SH3_CORE shl 16) or
                                         PROCESSOR_FEATURE_NOFP;

     PROCESSOR_HITACHI_SH3DSP_INSTRUCTION = (PROCESSOR_ARCHITECTURE_SHX shl 24) or
                                            (PROCESSOR_HITACHI_SH3_CORE shl 16) or
                                            PROCESSOR_FEATURE_DSP;

     PROCESSOR_HITACHI_SH4_INSTRUCTION = (PROCESSOR_ARCHITECTURE_SHX shl 24) or
                                         (PROCESSOR_HITACHI_SH4_CORE shl 16) or
                                         PROCESSOR_FEATURE_FP;

     PROCESSOR_ARM_V4_INSTRUCTION = (PROCESSOR_ARCHITECTURE_ARM shl 24) or
                                    (PROCESSOR_ARM_V4_CORE shl 16) or
                                    PROCESSOR_FEATURE_NOFP;

     PROCESSOR_ARM_V4FP_INSTRUCTION = (PROCESSOR_ARCHITECTURE_ARM shl 24) or
                                      (PROCESSOR_ARM_V4_CORE shl 16) or
                                      PROCESSOR_FEATURE_FP;

     PROCESSOR_ARM_V4I_INSTRUCTION = (PROCESSOR_ARCHITECTURE_ARM shl 24) or
                                     (PROCESSOR_ARM_V4I_CORE shl 16) or
                                     PROCESSOR_FEATURE_NOFP;

     PROCESSOR_ARM_V4IFP_INSTRUCTION = (PROCESSOR_ARCHITECTURE_ARM shl 24) or
                                       (PROCESSOR_ARM_V4I_CORE shl 16) or
                                       PROCESSOR_FEATURE_FP;

     PROCESSOR_ARM_V4T_INSTRUCTION = (PROCESSOR_ARCHITECTURE_ARM shl 24) or
                                     (PROCESSOR_ARM_V4T_CORE shl 16) or
                                     PROCESSOR_FEATURE_NOFP;

     PROCESSOR_ARM_V4TFP_INSTRUCTION = (PROCESSOR_ARCHITECTURE_ARM shl 24) or
                                       (PROCESSOR_ARM_V4T_CORE shl 16) or
                                       PROCESSOR_FEATURE_FP;

     PF_FLOATING_POINT_PRECISION_ERRATA  = 0;
     PF_FLOATING_POINT_EMULATED          = 1;
     PF_COMPARE_EXCHANGE_DOUBLE          = 2;
     PF_MMX_INSTRUCTIONS_AVAILABLE       = 3;
     PF_PPC_MOVEMEM_64BIT_OK             = 4;
     PF_ALPHA_BYTE_INSTRUCTIONS          = 5;
     PF_XMMI_INSTRUCTIONS_AVAILABLE      = 6;
     PF_3DNOW_INSTRUCTIONS_AVAILABLE     = 7;
     PF_RDTSC_INSTRUCTION_AVAILABLE      = 8;
     PF_PAE_ENABLED                      = 9;
     PF_XMMI64_INSTRUCTIONS_AVAILABLE    = 10;

{ Table from MSDN. Instruction set compatibility.
OS instructions set                    Supported instruction sets
--------------------------------------------------------------------------------
PROCESSOR_X86_32BIT_INSTRUCTION        PROCESSOR_X86_32BIT_INSTRUCTION

PROCESSOR_MIPS_MIPS16_INSTRUCTION      PROCESSOR_MIPS_MIPS16_INSTRUCTION
                                       PROCESSOR_MIPS_MIPSII_INSTRUCTION

PROCESSOR_MIPS_MIPSII_INSTRUCTION      PROCESSOR_MIPS_MIPSII_INSTRUCTION

PROCESSOR_MIPS_MIPSIIFP_INSTRUCTION    PROCESSOR_MIPS_MIPSIIFP_INSTRUCTION

PROCESSOR_MIPS_MIPSIV_INSTRUCTION      PROCESSOR_MIPS_MIPSIV_INSTRUCTION

PROCESSOR_MIPS_MIPSIVFP_INSTRUCTION    PROCESSOR_MIPS_MIPSIVFP_INSTRUCTION

PROCESSOR_HITACHI_SH3_INSTRUCTION      PROCESSOR_HITACHI_SH3_INSTRUCTION

PROCESSOR_HITACHI_SH3DSP_INSTRUCTION   PROCESSOR_HITACHI_SH3DSP_INSTRUCTION
                                       PROCESSOR_HITACHI_SH3_INSTRUCTION

PROCESSOR_HITACHI_SH4_INSTRUCTION      PROCESSOR_HITACHI_SH4_INSTRUCTION

PROCESSOR_ARM_V4_INSTRUCTION           PROCESSOR_ARM_V4_INSTRUCTION

PROCESSOR_ARM_V4FP_INSTRUCTION         PROCESSOR_ARM_V4FP_INSTRUCTION
                                       PROCESSOR_ARM_V4_INSTRUCTION

PROCESSOR_ARM_V4I_INSTRUCTION          PROCESSOR_ARM_V4I_INSTRUCTION
                                       PROCESSOR_ARM_V4T_INSTRUCTION

PROCESSOR_ARM_V4IFP_INSTRUCTION        PROCESSOR_ARM_V4IFP_INSTRUCTION
                                       PROCESSOR_ARM_V4TFP_INSTRUCTION
                                       PROCESSOR_ARM_V4I_INSTRUCTION
                                       PROCESSOR_ARM_V4T_INSTRUCTION

PROCESSOR_ARM_V4T_INSTRUCTION          PROCESSOR_ ARM_V4I_INSTRUCTION
                                       PROCESSOR_ARM_V4T_INSTRUCTION

PROCESSOR_ARM_V4TFP_INSTRUCTION        PROCESSOR_ARM_V4TFP_INSTRUCTION
                                       PROCESSOR_ARM_V4IFP_INSTRUCTION
                                       PROCESSOR_ARM_V4I_INSTRUCTION
                                       PROCESSOR_ARM_V4T_INSTRUCTION
}



  { FSCTL_SET_COMPRESSION  }
     COMPRESSION_FORMAT_NONE = 0;
     COMPRESSION_FORMAT_DEFAULT = 1;
     COMPRESSION_FORMAT_LZNT1 = 2;
  { TAPE_GET_DRIVE_PARAMETERS structure  }
     TAPE_DRIVE_COMPRESSION = 131072;
     TAPE_DRIVE_ECC = 65536;
     TAPE_DRIVE_ERASE_BOP_ONLY = 64;
     TAPE_DRIVE_ERASE_LONG = 32;
     TAPE_DRIVE_ERASE_IMMEDIATE = 128;
     TAPE_DRIVE_ERASE_SHORT = 16;
     TAPE_DRIVE_FIXED = 1;
     TAPE_DRIVE_FIXED_BLOCK = 1024;
     TAPE_DRIVE_INITIATOR = 4;
     TAPE_DRIVE_PADDING = 262144;
     TAPE_DRIVE_GET_ABSOLUTE_BLK = 1048576;
     TAPE_DRIVE_GET_LOGICAL_BLK = 2097152;
     TAPE_DRIVE_REPORT_SMKS = 524288;
     TAPE_DRIVE_SELECT = 2;
     TAPE_DRIVE_SET_EOT_WZ_SIZE = 4194304;
     TAPE_DRIVE_TAPE_CAPACITY = 256;
     TAPE_DRIVE_TAPE_REMAINING = 512;
     TAPE_DRIVE_VARIABLE_BLOCK = 2048;
     TAPE_DRIVE_WRITE_PROTECT = 4096;
     TAPE_DRIVE_ABS_BLK_IMMED = -(2147475456);
     TAPE_DRIVE_ABSOLUTE_BLK = -(2147479552);
     TAPE_DRIVE_END_OF_DATA = -(2147418112);
     TAPE_DRIVE_FILEMARKS = -(2147221504);
     TAPE_DRIVE_LOAD_UNLOAD = -(2147483647);
     TAPE_DRIVE_LOAD_UNLD_IMMED = -(2147483616);
     TAPE_DRIVE_LOCK_UNLOCK = -(2147483644);
     TAPE_DRIVE_LOCK_UNLK_IMMED = -(2147483520);
     TAPE_DRIVE_LOG_BLK_IMMED = -(2147450880);
     TAPE_DRIVE_LOGICAL_BLK = -(2147467264);
     TAPE_DRIVE_RELATIVE_BLKS = -(2147352576);
     TAPE_DRIVE_REVERSE_POSITION = -(2143289344);
     TAPE_DRIVE_REWIND_IMMEDIATE = -(2147483640);
     TAPE_DRIVE_SEQUENTIAL_FMKS = -(2146959360);
     TAPE_DRIVE_SEQUENTIAL_SMKS = -(2145386496);
     TAPE_DRIVE_SET_BLOCK_SIZE = -(2147483632);
     TAPE_DRIVE_SET_COMPRESSION = -(2147483136);
     TAPE_DRIVE_SET_ECC = -(2147483392);
     TAPE_DRIVE_SET_PADDING = -(2147482624);
     TAPE_DRIVE_SET_REPORT_SMKS = -(2147481600);
     TAPE_DRIVE_SETMARKS = -(2146435072);
     TAPE_DRIVE_SPACE_IMMEDIATE = -(2139095040);
     TAPE_DRIVE_TENSION = -(2147483646);
     TAPE_DRIVE_TENSION_IMMED = -(2147483584);
     TAPE_DRIVE_WRITE_FILEMARKS = -(2113929216);
     TAPE_DRIVE_WRITE_LONG_FMKS = -(2013265920);
     TAPE_DRIVE_WRITE_MARK_IMMED = -(1879048192);
     TAPE_DRIVE_WRITE_SETMARKS = -(2130706432);
     TAPE_DRIVE_WRITE_SHORT_FMKS = -(2080374784);
  { ACCESS_MASK, ACCESS TYPES  }
//     DELETE = $00010000;       //+winnt  // The name conflicts with Delete procedure
     READ_CONTROL = $00020000; //+winnt
     WRITE_DAC = $00040000;    //winnt
     WRITE_OWNER = $00080000;  //winnt
     SYNCHRONIZE = $00100000;  //winnt
     //
     // MaximumAllowed access type
     //
     MAXIMUM_ALLOWED = $2000000;   //winnt
     GENERIC_READ = $80000000;     //winnt
     GENERIC_WRITE = $40000000;    //winnt
     GENERIC_EXECUTE = $20000000;  //+winnt
     GENERIC_ALL = $10000000;      //winnt
  { Standard rights  }
     STANDARD_RIGHTS_REQUIRED = $f0000;      //winnt
     STANDARD_RIGHTS_WRITE = READ_CONTROL;   //~winnt
     STANDARD_RIGHTS_READ = READ_CONTROL;    //~winnt
     STANDARD_RIGHTS_EXECUTE = READ_CONTROL; //~winnt
     STANDARD_RIGHTS_ALL = $1f0000;
     SPECIFIC_RIGHTS_ALL = $ffff;
     //
     // AccessSystemAcl access type
     //
     ACCESS_SYSTEM_SECURITY = $1000000;
  { SID  }
     /////////////////////////////////////////////////////////////////////////////
     //                                                                         //
     // Universal well-known SIDs                                               //
     //                                                                         //
     //     Null SID                     S-1-0-0                                //
     //     World                        S-1-1-0                                //
     //     Local                        S-1-2-0                                //
     //     Creator Owner ID             S-1-3-0                                //
     //     Creator Group ID             S-1-3-1                                //
     //     Creator Owner Server ID      S-1-3-2                                //
     //     Creator Group Server ID      S-1-3-3                                //
     //                                                                         //
     //     (Non-unique IDs)             S-1-4                                  //
     //                                                                         //
     /////////////////////////////////////////////////////////////////////////////

     SECURITY_NULL_SID_AUTHORITY : SID_IDENTIFIER_AUTHORITY_REC = ( 0,0,0,0,0,0);  //+winnt
     SECURITY_WORLD_SID_AUTHORITY : SID_IDENTIFIER_AUTHORITY_REC = (0,0,0,0,0,1);  //+winnt
     SECURITY_LOCAL_SID_AUTHORITY : SID_IDENTIFIER_AUTHORITY_REC = (0,0,0,0,0,2);  //+winnt
     SECURITY_CREATOR_SID_AUTHORITY : SID_IDENTIFIER_AUTHORITY_REC = (0,0,0,0,0,3);//+winnt
     SECURITY_NON_UNIQUE_AUTHORITY : SID_IDENTIFIER_AUTHORITY_REC = (0,0,0,0,0,4); //+winnt
     SECURITY_NULL_RID = 0;                  //winnt
     SECURITY_WORLD_RID = 0;                 //winnt
     SECURITY_LOCAL_RID = 0;                 //winnt
     SECURITY_CREATOR_OWNER_RID = 0;         //winnt
     SECURITY_CREATOR_GROUP_RID = $1;        //winnt
     SECURITY_CREATOR_OWNER_SERVER_RID = $2; //+winnt
     SECURITY_CREATOR_GROUP_SERVER_RID = $3; //+winnt
     /////////////////////////////////////////////////////////////////////////////
     //                                                                         //
     // NT well-known SIDs                                                      //
     //                                                                         //
     //     NT Authority          S-1-5                                         //
     //     Dialup                S-1-5-1                                       //
     //                                                                         //
     //     Network               S-1-5-2                                       //
     //     Batch                 S-1-5-3                                       //
     //     Interactive           S-1-5-4                                       //
     //     Service               S-1-5-6                                       //
     //     AnonymousLogon        S-1-5-7       (aka null logon session)        //
     //     Proxy                 S-1-5-8                                       //
     //     ServerLogon           S-1-5-9       (aka domain controller account) //
     //     Self                  S-1-5-10      (self RID)                      //
     //     Authenticated User    S-1-5-11      (Authenticated user somewhere)  //
     //     Restricted Code       S-1-5-12      (Running restricted code)       //
     //                                                                         //
     //     (Logon IDs)           S-1-5-5-X-Y                                   //
     //                                                                         //
     //     (NT non-unique IDs)   S-1-5-0x15-...                                //
     //                                                                         //
     //     (Built-in domain)     s-1-5-0x20                                    //
     //                                                                         //
     /////////////////////////////////////////////////////////////////////////////
     SECURITY_NT_AUTHORITY : SID_IDENTIFIER_AUTHORITY_REC = (0,0,0,0,0,5); //+winnt

     SECURITY_DIALUP_RID                 = $1;        //winnt
     SECURITY_NETWORK_RID                = $2;        //winnt
     SECURITY_BATCH_RID                  = $3;        //winnt
     SECURITY_INTERACTIVE_RID            = $4;        //winnt
     SECURITY_LOGON_IDS_RID              = $5;        //winnt
     SECURITY_LOGON_IDS_RID_COUNT        = $3;        //winnt
     SECURITY_SERVICE_RID                = $6;        //winnt
     SECURITY_ANONYMOUS_LOGON_RID        = $00000007; //+winnt
     SECURITY_PROXY_RID                  = $00000008; //+winnt
     SECURITY_ENTERPRISE_CONTROLLERS_RID = $00000009; //+winnt
     SECURITY_SERVER_LOGON_RID           = SECURITY_ENTERPRISE_CONTROLLERS_RID; //+winnt
     SECURITY_PRINCIPAL_SELF_RID         = $0000000A; //+winnt
     SECURITY_AUTHENTICATED_USER_RID     = $0000000B; //+winnt
     SECURITY_RESTRICTED_CODE_RID        = $0000000C; //+winnt

     SECURITY_LOCAL_SYSTEM_RID           = $12;       //winnt
     SECURITY_NT_NON_UNIQUE              = $00000015; //+winnt
     SECURITY_BUILTIN_DOMAIN_RID         = $20;       //winnt
     // Well-known users ...
     DOMAIN_USER_RID_ADMIN               = $1f4;      //winnt
     DOMAIN_USER_RID_GUEST               = $1f5;      //winnt
     DOMAIN_USER_RID_KRBTGT              = $000001F6; //+winnt
     // well-known groups ...
     DOMAIN_GROUP_RID_ADMINS             = $200;      //winnt
     DOMAIN_GROUP_RID_USERS              = $201;      //winnt
     DOMAIN_GROUP_RID_GUESTS             = $00000202; //+winnt
     DOMAIN_GROUP_RID_COMPUTERS          = $00000203; //+winnt
     DOMAIN_GROUP_RID_CONTROLLERS        = $00000204; //+winnt
     DOMAIN_GROUP_RID_CERT_ADMINS        = $00000205; //+winnt
     DOMAIN_GROUP_RID_SCHEMA_ADMINS      = $00000206; //+winnt
     DOMAIN_GROUP_RID_ENTERPRISE_ADMINS  = $00000207; //+winnt
     // well-known aliases ...
     DOMAIN_ALIAS_RID_ADMINS = $220;                  //winnt
     DOMAIN_ALIAS_RID_USERS = $221;                   //winnt
     DOMAIN_ALIAS_RID_GUESTS = $222;                  //winnt
     DOMAIN_ALIAS_RID_POWER_USERS = $223;             //winnt
     DOMAIN_ALIAS_RID_ACCOUNT_OPS = $224;             //winnt
     DOMAIN_ALIAS_RID_SYSTEM_OPS = $225;              //winnt
     DOMAIN_ALIAS_RID_PRINT_OPS = $226;               //winnt
     DOMAIN_ALIAS_RID_BACKUP_OPS = $227;              //winnt
     DOMAIN_ALIAS_RID_REPLICATOR = $228;              //winnt

     //
     // Allocate the System Luid.  The first 1000 LUIDs are reserved.
     // Use #999 here (0x3E7 = 999)
     //
     SYSTEM_LUID : LUID = ( LowPart: $3E7; HighPart: $0 );
     ANONYMOUS_LOGON_LUID : LUID = (LowPart: $3e6; HighPart: $0 );

     //
     // Group attributes
     //
     SE_GROUP_MANDATORY = $1;                         //winnt
     SE_GROUP_ENABLED_BY_DEFAULT = $2;                //winnt
     SE_GROUP_ENABLED = $4;                           //winnt
     SE_GROUP_OWNER = $8;                             //winnt
     SE_GROUP_USE_FOR_DENY_ONLY = $00000010;          //+winnt
     SE_GROUP_LOGON_ID = $c0000000;                   //winnt

  { ACL Defines  }
     ACL_REVISION = 2;
     ACL_REVISION_DS = 4;             //+winnt
     // This is the history of ACL revisions.  Add a new one whenever
     // ACL_REVISION is updated
     ACL_REVISION1=1;                 //+winnt
     ACL_REVISION2=2;                 //+winnt
     ACL_REVISION3=3;                 //+winnt
     ACL_REVISION4=4;                 //+winnt
     MIN_ACL_REVISION=ACL_REVISION2;  //+winnt
     MAX_ACL_REVISION=ACL_REVISION4;  //+winnt
  { ACE_HEADER structure  }
     ACCESS_MIN_MS_ACE_TYPE  = $0;          //+winnt
     ACCESS_ALLOWED_ACE_TYPE = $0;
     ACCESS_DENIED_ACE_TYPE = $1;
     SYSTEM_AUDIT_ACE_TYPE = $2;
     SYSTEM_ALARM_ACE_TYPE = $3;
     ACCESS_MAX_MS_V2_ACE_TYPE = $3;        //+winnt
     ACCESS_ALLOWED_COMPOUND_ACE_TYPE = $4; //+winnt
     ACCESS_MAX_MS_V3_ACE_TYPE = $4;

     ACCESS_MIN_MS_OBJECT_ACE_TYPE = $5;    //+winnt
     ACCESS_ALLOWED_OBJECT_ACE_TYPE = $5;   //+winnt
     ACCESS_DENIED_OBJECT_ACE_TYPE = $6;    //+winnt
     SYSTEM_AUDIT_OBJECT_ACE_TYPE = $7;     //+winnt
     SYSTEM_ALARM_OBJECT_ACE_TYPE = $8;     //+winnt
     ACCESS_MAX_MS_OBJECT_ACE_TYPE = $8;    //+winnt

     ACCESS_MAX_MS_V4_ACE_TYPE = $8;        //+winnt
     ACCESS_MAX_MS_ACE_TYPE = $8;           //+winnt

  { ACE flags in the ACE_HEADER structure  }
     //
     //  The following are the inherit flags that go into the AceFlags field
     //  of an Ace header.
     //
     OBJECT_INHERIT_ACE = $1;              //winnt
     CONTAINER_INHERIT_ACE = $2;           //winnt
     NO_PROPAGATE_INHERIT_ACE = $4;        //winnt
     INHERIT_ONLY_ACE = $8;                //winnt
     INHERITED_ACE = $10;                  //+winnt
     VALID_INHERIT_FLAGS = $1F;            //+winnt
     //  The following are the currently defined ACE flags that go into the
     //  AceFlags field of an ACE header.  Each ACE type has its own set of
     //  AceFlags.
     //
     //  SUCCESSFUL_ACCESS_ACE_FLAG - used only with system audit and alarm ACE
     //  types to indicate that a message is generated for successful accesses.
     //
     //  FAILED_ACCESS_ACE_FLAG - used only with system audit and alarm ACE types
     //  to indicate that a message is generated for failed accesses.
     //
     //
     //  SYSTEM_AUDIT and SYSTEM_ALARM AceFlags
     //
     //  These control the signaling of audit and alarms for success or failure.
     //
     SUCCESSFUL_ACCESS_ACE_FLAG = $40;     //winnt
     FAILED_ACCESS_ACE_FLAG = $80;         //winnt
  { SECURITY_DESCRIPTOR_CONTROL  }
     {SECURITY_DESCRIPTOR_REVISION = 1;already defined above }
     SECURITY_DESCRIPTOR_MIN_LENGTH = 20;
     SE_OWNER_DEFAULTED = 1;
     SE_GROUP_DEFAULTED = 2;
     SE_DACL_PRESENT = 4;
     SE_DACL_DEFAULTED = 8;
     SE_SACL_PRESENT = 16;
     SE_SACL_DEFAULTED = 32;
     SE_SELF_RELATIVE = 32768;
  { PRIVILEGE_SET  }
     SE_PRIVILEGE_ENABLED_BY_DEFAULT = $1;
     SE_PRIVILEGE_ENABLED = $2;
     SE_PRIVILEGE_USED_FOR_ACCESS = $80000000;
     PRIVILEGE_SET_ALL_NECESSARY = $1;
  { OPENFILENAME structure  }
     OFN_ALLOWMULTISELECT = $200;
     OFN_CREATEPROMPT = $2000;
     OFN_ENABLEHOOK = $20;
     OFN_ENABLETEMPLATE = $40;
     OFN_ENABLETEMPLATEHANDLE = $80;
     OFN_EXPLORER = $80000;
     OFN_EXTENSIONDIFFERENT = $400;
     OFN_FILEMUSTEXIST = $1000;
     OFN_HIDEREADONLY = $4;
     OFN_LONGNAMES = $200000;
     OFN_NOCHANGEDIR = $8;
     OFN_NODEREFERENCELINKS = $100000;
     OFN_NOLONGNAMES = $40000;
     OFN_NONETWORKBUTTON = $20000;
     OFN_NOREADONLYRETURN = $8000;
     OFN_NOTESTFILECREATE = $10000;
     OFN_NOVALIDATE = $100;
     OFN_OVERWRITEPROMPT = $2;
     OFN_PATHMUSTEXIST = $800;
     OFN_READONLY = $1;
     OFN_SHAREAWARE = $4000;
     OFN_SHOWHELP = $10;
     OFN_PROJECT  = $00400000;
     OFN_PROPERTY = $00800000;
     OFN_SHOW_ALL = $01000000;
  { SHAREVISTRING message  }
     OFN_SHAREFALLTHROUGH = $2;
     OFN_SHARENOWARN = $1;
     OFN_SHAREWARN = 0;
  { Open/Save notifications  }
     CDN_INITDONE = $fffffda7;
     CDN_SELCHANGE = $fffffda6;
     CDN_FOLDERCHANGE = $fffffda5;
     CDN_SHAREVIOLATION = $fffffda4;
     CDN_HELP = $fffffda3;
     CDN_FILEOK = $fffffda2;
     CDN_TYPECHANGE = $fffffda1;
  { Open/Save messages  }
     CDM_GETFILEPATH = $465;
     CDM_GETFOLDERIDLIST = $467;
     CDM_GETFOLDERPATH = $466;
     CDM_GETSPEC = $464;
     CDM_HIDECONTROL = $469;
     CDM_SETCONTROLTEXT = $468;
     CDM_SETDEFEXT = $46a;
  { CHOOSECOLOR structure  }
     CC_ENABLEHOOK = $10;
     CC_ENABLETEMPLATE = $20;
     CC_ENABLETEMPLATEHANDLE = $40;
     CC_FULLOPEN = $2;
     CC_PREVENTFULLOPEN = $4;
     CC_RGBINIT = $1;
     CC_SHOWHELP = $8;
     CC_SOLIDCOLOR = $80;
  { FINDREPLACE structure  }
     FR_DIALOGTERM = $40;
     FR_DOWN = $1;
     FR_ENABLEHOOK = $100;
     FR_ENABLETEMPLATE = $200;
     FR_ENABLETEMPLATEHANDLE = $2000;
     FR_FINDNEXT = $8;
     FR_HIDEUPDOWN = $4000;
     FR_HIDEMATCHCASE = $8000;
     FR_HIDEWHOLEWORD = $10000;
     FR_MATCHCASE = $4;
     FR_NOMATCHCASE = $800;
     FR_NOUPDOWN = $400;
     FR_NOWHOLEWORD = $1000;
     FR_REPLACE = $10;
     FR_REPLACEALL = $20;
     FR_SHOWHELP = $80;
     FR_WHOLEWORD = $2;
  { CHOOSEFONT structure  }
     CF_APPLY = $200;
     CF_ANSIONLY = $400;
     CF_BOTH = $3;
     CF_TTONLY = $40000;
     CF_EFFECTS = $100;
     CF_ENABLEHOOK = $8;
     CF_ENABLETEMPLATE = $10;
     CF_ENABLETEMPLATEHANDLE = $20;
     CF_FIXEDPITCHONLY = $4000;
     CF_FORCEFONTEXIST = $10000;
     CF_INITTOLOGFONTSTRUCT = $40;
     CF_LIMITSIZE = $2000;
     CF_NOOEMFONTS = $800;
     CF_NOFACESEL = $80000;
     CF_NOSCRIPTSEL = $800000;
     CF_NOSTYLESEL = $100000;
     CF_NOSIZESEL = $200000;
     CF_NOSIMULATIONS = $1000;
     CF_NOVECTORFONTS = $800;
     CF_NOVERTFONTS = $1000000;
     CF_PRINTERFONTS = $2;
     CF_SCALABLEONLY = $20000;
     CF_SCREENFONTS = $1;
     CF_SCRIPTSONLY = $400;
     CF_SELECTSCRIPT = $400000;
     CF_SHOWHELP = $4;
     CF_USESTYLE = $80;
     CF_WYSIWYG = $8000;
     BOLD_FONTTYPE = $100;
     ITALIC_FONTTYPE = $200;
     PRINTER_FONTTYPE = $4000;
     REGULAR_FONTTYPE = $400;
     SCREEN_FONTTYPE = $2000;
     SIMULATED_FONTTYPE = $8000;
  { Common dialog messages  }
     COLOROKSTRINGW = 'commdlg_ColorOK';
     FILEOKSTRINGW = 'commdlg_FileNameOK';
     FINDMSGSTRINGW = 'commdlg_FindReplace';
     HELPMSGSTRINGW = 'commdlg_help';
     LBSELCHSTRINGW = 'commdlg_LBSelChangedNotify';
     SETRGBSTRINGW = 'commdlg_SetRGBColor';
     SHAREVISTRINGW = 'commdlg_ShareViolation';
     COLOROKSTRINGA = 'commdlg_ColorOK';
     FILEOKSTRINGA = 'commdlg_FileNameOK';
     FINDMSGSTRINGA = 'commdlg_FindReplace';
     HELPMSGSTRINGA = 'commdlg_help';
     LBSELCHSTRINGA = 'commdlg_LBSelChangedNotify';
     SETRGBSTRINGA = 'commdlg_SetRGBColor';
     SHAREVISTRINGA = 'commdlg_ShareViolation';
{$ifdef UNICODE}

  const
     COLOROKSTRING = COLOROKSTRINGW;
     FILEOKSTRING = FILEOKSTRINGW;
     FINDMSGSTRING = FINDMSGSTRINGW;
     HELPMSGSTRING = HELPMSGSTRINGW;
     LBSELCHSTRING = LBSELCHSTRINGW;
     SETRGBSTRING = SETRGBSTRINGW;
     SHAREVISTRING = SHAREVISTRINGW;
{$else}

  const
     COLOROKSTRING = COLOROKSTRINGA;
     FILEOKSTRING = FILEOKSTRINGA;
     FINDMSGSTRING = FINDMSGSTRINGA;
     HELPMSGSTRING = HELPMSGSTRINGA;
     LBSELCHSTRING = LBSELCHSTRINGA;
     SETRGBSTRING = SETRGBSTRINGA;
     SHAREVISTRING = SHAREVISTRINGA;
{$endif}
  { LBSELCHSTRING message  }

  const
     CD_LBSELCHANGE = 0;
     CD_LBSELADD = 2;
     CD_LBSELSUB = 1;
     CD_LBSELNOITEMS = -(1);
  { DEVNAMES structure  }
     DN_DEFAULTPRN = 1;
  { PRINTDLG structure  }
     PD_ALLPAGES = 0;
     PD_COLLATE = 16;
     PD_DISABLEPRINTTOFILE = 524288;
     PD_ENABLEPRINTHOOK = 4096;
     PD_ENABLEPRINTTEMPLATE = 16384;
     PD_ENABLEPRINTTEMPLATEHANDLE = 65536;
     PD_ENABLESETUPHOOK = 8192;
     PD_ENABLESETUPTEMPLATE = 32768;
     PD_ENABLESETUPTEMPLATEHANDLE = 131072;
     PD_HIDEPRINTTOFILE = 1048576;
     PD_NOPAGENUMS = 8;
     PD_NOSELECTION = 4;
     PD_NOWARNING = 128;
     PD_PAGENUMS = 2;
     PD_PRINTSETUP = 64;
     PD_PRINTTOFILE = 32;
     PD_RETURNDC = 256;
     PD_RETURNDEFAULT = 1024;
     PD_RETURNIC = 512;
     PD_SELECTION = 1;
     PD_SHOWHELP = 2048;
     PD_USEDEVMODECOPIES = 262144;
     PD_USEDEVMODECOPIESANDCOLLATE = 262144;
  { PAGESETUPDLG structure  }
     PSD_DEFAULTMINMARGINS = 0;
     PSD_DISABLEMARGINS = 16;
     PSD_DISABLEORIENTATION = 256;
     PSD_DISABLEPAGEPAINTING = 524288;
     PSD_DISABLEPAPER = 512;
     PSD_DISABLEPRINTER = 32;
     PSD_ENABLEPAGEPAINTHOOK = 262144;
     PSD_ENABLEPAGESETUPHOOK = 8192;
     PSD_ENABLEPAGESETUPTEMPLATE = 32768;
     PSD_ENABLEPAGESETUPTEMPLATEHANDLE = 131072;
     PSD_INHUNDREDTHSOFMILLIMETERS = 8;
     PSD_INTHOUSANDTHSOFINCHES = 4;
     PSD_INWININIINTLMEASURE = 0;
     PSD_MARGINS = 2;
     PSD_MINMARGINS = 1;
     PSD_NOWARNING = 128;
     PSD_RETURNDEFAULT = 1024;
     PSD_SHOWHELP = 2048;
  { WM_SHOWWINDOW message  }
     SW_OTHERUNZOOM = 4;
     SW_OTHERZOOM = 2;
     SW_PARENTCLOSING = 1;
     SW_PARENTOPENING = 3;
  { Virtual Key codes  }
     VK_LBUTTON = 1;
     VK_RBUTTON = 2;
     VK_CANCEL = 3;
     VK_MBUTTON = 4;
     VK_BACK = 8;
     VK_TAB = 9;
     VK_CLEAR = 12;
     VK_RETURN = 13;
     VK_SHIFT = 16;
     VK_CONTROL = 17;
     VK_MENU = 18;
     VK_PAUSE = 19;
     VK_CAPITAL = 20;
     VK_ESCAPE = 27;
     VK_SPACE = 32;
     VK_PRIOR = 33;
     VK_NEXT = 34;
     VK_END = 35;
     VK_HOME = 36;
     VK_LEFT = 37;
     VK_UP = 38;
     VK_RIGHT = 39;
     VK_DOWN = 40;
     VK_SELECT = 41;
     VK_PRINT = 42;
     VK_EXECUTE = 43;
     VK_SNAPSHOT = 44;
     VK_INSERT = 45;
     VK_DELETE = 46;
     VK_HELP = 47;
     VK_0 = 48;
     VK_1 = 49;
     VK_2 = 50;
     VK_3 = 51;
     VK_4 = 52;
     VK_5 = 53;
     VK_6 = 54;
     VK_7 = 55;
     VK_8 = 56;
     VK_9 = 57;
     VK_A = 65;
     VK_B = 66;
     VK_C = 67;
     VK_D = 68;
     VK_E = 69;
     VK_F = 70;
     VK_G = 71;
     VK_H = 72;
     VK_I = 73;
     VK_J = 74;
     VK_K = 75;
     VK_L = 76;
     VK_M = 77;
     VK_N = 78;
     VK_O = 79;
     VK_P = 80;
     VK_Q = 81;
     VK_R = 82;
     VK_S = 83;
     VK_T = 84;
     VK_U = 85;
     VK_V = 86;
     VK_W = 87;
     VK_X = 88;
     VK_Y = 89;
     VK_Z = 90;
     VK_LWIN = 91;
     VK_RWIN = 92;
     VK_APPS = 93;
     VK_NUMPAD0 = 96;
     VK_NUMPAD1 = 97;
     VK_NUMPAD2 = 98;
     VK_NUMPAD3 = 99;
     VK_NUMPAD4 = 100;
     VK_NUMPAD5 = 101;
     VK_NUMPAD6 = 102;
     VK_NUMPAD7 = 103;
     VK_NUMPAD8 = 104;
     VK_NUMPAD9 = 105;
     VK_MULTIPLY = 106;
     VK_ADD = 107;
     VK_SEPARATOR = 108;
     VK_SUBTRACT = 109;
     VK_DECIMAL = 110;
     VK_DIVIDE = 111;
     VK_F1 = 112;
     VK_F2 = 113;
     VK_F3 = 114;
     VK_F4 = 115;
     VK_F5 = 116;
     VK_F6 = 117;
     VK_F7 = 118;
     VK_F8 = 119;
     VK_F9 = 120;
     VK_F10 = 121;
     VK_F11 = 122;
     VK_F12 = 123;
     VK_F13 = 124;
     VK_F14 = 125;
     VK_F15 = 126;
     VK_F16 = 127;
     VK_F17 = 128;
     VK_F18 = 129;
     VK_F19 = 130;
     VK_F20 = 131;
     VK_F21 = 132;
     VK_F22 = 133;
     VK_F23 = 134;
     VK_F24 = 135;
  { GetAsyncKeyState  }
     VK_NUMLOCK = 144;
     VK_SCROLL = 145;
     VK_LSHIFT = 160;
     VK_LCONTROL = 162;
     VK_LMENU = 164;
     VK_RSHIFT = 161;
     VK_RCONTROL = 163;
     VK_RMENU = 165;
  { ImmGetVirtualKey  }
     VK_PROCESSKEY = 229;

     // these constants are xda-specific:
     VK_TOUCHPAD   = $01; // VK_LBUTTON
     VK_ANSWER     = $72; // VK_F5
     VK_HANGUP     = $73; // VK_F4
     VK_VOLUME     = $75; // VK_F6

     VK_TOPBUTTONS = $5b; // VK_LWIN
     // VK_TOPBUTTONS is sent together with one of these:
     VK_CALENDER   = $C1;
     VK_CONTACTS   = $C2;

     // note that when holding the navigator button down continuously,
     // occasionally the OS will report no keys pressed down.

     VK_NAVIGATOR_MOVE = $84; // VK_F21
     // VK_NAVIGATOR_MOVE is sent together with one of these:
     VK_NAVIGATE_LEFT  = $25; // VK_LEFT
     VK_NAVIGATE_UP    = $26; // VK_UP
     VK_NAVIGATE_RIGHT = $27; // VK_RIGHT
     VK_NAVIGATE_DOWN  = $28; // VK_DOWN

     VK_NAVIGATOR_SELECT = $86; // VK_F23

     VK_ONOFF            = $DF; // VK_OFF

  { Keystroke Message Flags  }
     KF_ALTDOWN = 8192;
     KF_DLGMODE = 2048;
     KF_EXTENDED = 256;
     KF_MENUMODE = 4096;
     KF_REPEAT = 16384;
     KF_UP = 32768;
  { GetKeyboardLayoutName  }
     KL_NAMELENGTH = 9;
  { WM_ACTIVATE message  }
     WA_ACTIVE = 1;
     WA_CLICKACTIVE = 2;
     WA_INACTIVE = 0;
  { WM_ACTIVATE message  }
     PWR_CRITICALRESUME = 3;
     PWR_SUSPENDREQUEST = 1;
     PWR_SUSPENDRESUME = 2;
     PWR_FAIL = -(1);
     PWR_OK = 1;
  { WM_NOTIFYFORMAT message  }
     NF_QUERY = 3;
     NF_REQUERY = 4;
     NFR_ANSI = 1;
     NFR_UNICODE = 2;
  { WM_SIZING message  }
     WMSZ_BOTTOM = 6;
     WMSZ_BOTTOMLEFT = 7;
     WMSZ_BOTTOMRIGHT = 8;
     WMSZ_LEFT = 1;
     WMSZ_RIGHT = 2;
     WMSZ_TOP = 3;
     WMSZ_TOPLEFT = 4;
     WMSZ_TOPRIGHT = 5;
  { WM_MOUSEACTIVATE message  }
     MA_ACTIVATE = 1;
     MA_ACTIVATEANDEAT = 2;
     MA_NOACTIVATE = 3;
     MA_NOACTIVATEANDEAT = 4;
  { WM_SIZE message  }
     SIZE_MAXHIDE = 4;
     SIZE_MAXIMIZED = 2;
     SIZE_MAXSHOW = 3;
     SIZE_MINIMIZED = 1;
     SIZE_RESTORED = 0;
  { WM_NCCALCSIZE message  }
     WVR_ALIGNTOP = 16;
     WVR_ALIGNLEFT = 32;
     WVR_ALIGNBOTTOM = 64;
     WVR_ALIGNRIGHT = 128;
     WVR_HREDRAW = 256;
     WVR_VREDRAW = 512;
     WVR_REDRAW = 768;
     WVR_VALIDRECTS = 1024;
  { WM_NCHITTEST message  }
     HTBOTTOM = 15;
     HTBOTTOMLEFT = 16;
     HTBOTTOMRIGHT = 17;
     HTCAPTION = 2;
     HTCLIENT = 1;
     HTERROR = -(2);
     HTGROWBOX = 4;
     HTHSCROLL = 6;
     HTLEFT = 10;
     HTMENU = 5;
     HTNOWHERE = 0;
     HTREDUCE = 8;
     HTRIGHT = 11;
     HTSIZE = 4;
     HTSYSMENU = 3;
     HTTOP = 12;
     HTTOPLEFT = 13;
     HTTOPRIGHT = 14;
     HTTRANSPARENT = -(1);
     HTVSCROLL = 7;
     HTZOOM = 9;
  { Mouse messages  }
     MK_CONTROL = 8;
     MK_LBUTTON = 1;
     MK_MBUTTON = 16;
     MK_RBUTTON = 2;
     MK_SHIFT = 4;
  { WNDCLASS structure  }
     CS_VREDRAW = 1;
     CS_HREDRAW = 2;
     CS_DBLCLKS = 8;
     CS_PARENTDC = 128;
     CS_NOCLOSE = 512;
     CS_SAVEBITS = 2048;
     CS_GLOBALCLASS = 16384;
     CS_IME = $10000;

     DLGWINDOWEXTRA = 32;
  { ACCEL structure  }
     FALT = 16;
     FCONTROL = 8;
     FNOINVERT = 2;
     FSHIFT = 4;
     FVIRTKEY = 1;
  { MENUITEMINFO structure  }
     MIIM_CHECKMARKS = 8;
     MIIM_DATA = 32;
     MIIM_ID = 2;
     MIIM_STATE = 1;
     MIIM_SUBMENU = 4;
     MIIM_TYPE = 16;
     MFT_BITMAP = $4;
     MFT_MENUBARBREAK = $20;
     MFT_MENUBREAK = $40;
     MFT_OWNERDRAW = $100;
     MFT_RADIOCHECK = $200;
     MFT_RIGHTJUSTIFY = $4000;
     MFT_SEPARATOR = $800;
     MFT_STRING = 0;
     MFS_CHECKED = $8;
     MFS_DEFAULT = $1000;
     MFS_DISABLED = $3;
     MFS_ENABLED = 0;
     MFS_GRAYED = $3;
     MFS_HILITE = $80;
     MFS_UNCHECKED = 0;
     MFS_UNHILITE = 0;
  { SERIALKEYS structure  }
     SERKF_AVAILABLE = 2;
     SERKF_INDICATOR = 4;
     SERKF_SERIALKEYSON = 1;
  { FILTERKEYS structure  }
     FKF_AVAILABLE = 2;
     FKF_CLICKON = 64;
     FKF_FILTERKEYSON = 1;
     FKF_HOTKEYACTIVE = 4;
     FKF_HOTKEYSOUND = 16;
     FKF_CONFIRMHOTKEY = 8;
     FKF_INDICATOR = 32;
  { HELPINFO structure  }
     HELPINFO_MENUITEM = 2;
     HELPINFO_WINDOW = 1;
  { WM_PRINT message  }
     PRF_CHECKVISIBLE = $1;
     PRF_CHILDREN = $10;
     PRF_CLIENT = $4;
     PRF_ERASEBKGND = $8;
     PRF_NONCLIENT = $2;
     PRF_OWNED = $20;
  { MapWindowPoints  }
    { was #define dname def_expr }
    HWND_DESKTOP = HWND(0);

  { WM_SYSCOMMAND message  }

  const
     SC_CLOSE = 61536;
     SC_CONTEXTHELP = 61824;
     SC_DEFAULT = 61792;
     SC_HOTKEY = 61776;
     SC_HSCROLL = 61568;
     SC_KEYMENU = 61696;
     SC_MAXIMIZE = 61488;
     SC_ZOOM = 61488;
     SC_MINIMIZE = 61472;
     SC_ICON = 61472;
     SC_MONITORPOWER = 61808;
     SC_MOUSEMENU = 61584;
     SC_MOVE = 61456;
     SC_NEXTWINDOW = 61504;
     SC_PREVWINDOW = 61520;
     SC_RESTORE = 61728;
     SC_SCREENSAVE = 61760;
     SC_SIZE = 61440;
     SC_TASKLIST = 61744;
     SC_VSCROLL = 61552;
  { DM_GETDEFID message  }
     DC_HASDEFID = 21323;
  { WM_GETDLGCODE message  }
     DLGC_BUTTON = 8192;
     DLGC_DEFPUSHBUTTON = 16;
     DLGC_HASSETSEL = 8;
     DLGC_RADIOBUTTON = 64;
     DLGC_STATIC = 256;
     DLGC_UNDEFPUSHBUTTON = 32;
     DLGC_WANTALLKEYS = 4;
     DLGC_WANTARROWS = 1;
     DLGC_WANTCHARS = 128;
     DLGC_WANTMESSAGE = 4;
     DLGC_WANTTAB = 2;
  { EM_SETMARGINS message  }
     EC_LEFTMARGIN = 1;
     EC_RIGHTMARGIN = 2;
     EC_USEFONTINFO = 65535;
  { LB_SETCOUNT message  }
     LB_ERR = -(1);
     LB_ERRSPACE = -(2);
     LB_OKAY = 0;
  { CB_DIR message  }
     CB_OKAY = 0;
     CB_ERR = -(1);
     CB_ERRSPACE = -(2);
  { WM_IME_CONTROL message  }
     IMC_GETCANDIDATEPOS = 7;
     IMC_GETCOMPOSITIONFONT = 9;
     IMC_GETCOMPOSITIONWINDOW = 11;
     IMC_GETSTATUSWINDOWPOS = 15;
     IMC_CLOSESTATUSWINDOW = 33;
     IMC_OPENSTATUSWINDOW = 34;
     IMC_SETCANDIDATEPOS = 8;
     IMC_SETCOMPOSITIONFONT = 10;
     IMC_SETCOMPOSITIONWINDOW = 12;
     IMC_SETSTATUSWINDOWPOS = 16;
  { WM_IME_CONTROL message  }
     IMN_CHANGECANDIDATE = 3;
     IMN_CLOSECANDIDATE = 4;
     IMN_CLOSESTATUSWINDOW = 1;
     IMN_GUIDELINE = 13;
     IMN_OPENCANDIDATE = 5;
     IMN_OPENSTATUSWINDOW = 2;
     IMN_SETCANDIDATEPOS = 9;
     IMN_SETCOMPOSITIONFONT = 10;
     IMN_SETCOMPOSITIONWINDOW = 11;
     IMN_SETCONVERSIONMODE = 6;
     IMN_SETOPENSTATUS = 8;
     IMN_SETSENTENCEMODE = 7;
     IMN_SETSTATUSWINDOWPOS = 12;
     IMN_PRIVATE = 14;
  { STICKYKEYS structure  }
     SKF_AUDIBLEFEEDBACK = 64;
     SKF_AVAILABLE = 2;
     SKF_CONFIRMHOTKEY = 8;
     SKF_HOTKEYACTIVE = 4;
     SKF_HOTKEYSOUND = 16;
     SKF_INDICATOR = 32;
     SKF_STICKYKEYSON = 1;
     SKF_TRISTATE = 128;
     SKF_TWOKEYSOFF = 256;
  { MOUSEKEYS structure  }
     MKF_AVAILABLE = 2;
     MKF_CONFIRMHOTKEY = 8;
     MKF_HOTKEYACTIVE = 4;
     MKF_HOTKEYSOUND = 16;
     MKF_INDICATOR = 32;
     MKF_MOUSEKEYSON = 1;
     MKF_MODIFIERS = 64;
     MKF_REPLACENUMBERS = 128;
  { SOUNDSENTRY structure  }
     SSF_AVAILABLE = 2;
     SSF_SOUNDSENTRYON = 1;
     SSTF_BORDER = 2;
     SSTF_CHARS = 1;
     SSTF_DISPLAY = 3;
     SSTF_NONE = 0;
     SSGF_DISPLAY = 3;
     SSGF_NONE = 0;
     SSWF_CUSTOM = 4;
     SSWF_DISPLAY = 3;
     SSWF_NONE = 0;
     SSWF_TITLE = 1;
     SSWF_WINDOW = 2;
  { ACCESSTIMEOUT structure  }
     ATF_ONOFFFEEDBACK = 2;
     ATF_TIMEOUTON = 1;
  { HIGHCONTRAST structure  }
     HCF_AVAILABLE = 2;
     HCF_CONFIRMHOTKEY = 8;
     HCF_HIGHCONTRASTON = 1;
     HCF_HOTKEYACTIVE = 4;
     HCF_HOTKEYAVAILABLE = 64;
     HCF_HOTKEYSOUND = 16;
     HCF_INDICATOR = 32;
  { TOGGLEKEYS structure  }
     TKF_AVAILABLE = 2;
     TKF_CONFIRMHOTKEY = 8;
     TKF_HOTKEYACTIVE = 4;
     TKF_HOTKEYSOUND = 16;
     TKF_TOGGLEKEYSON = 1;
  { Installable Policy  }
     PP_DISPLAYERRORS = 1;
  { SERVICE_INFO structure  }
     RESOURCEDISPLAYTYPE_DOMAIN = 1;
     RESOURCEDISPLAYTYPE_FILE = 4;
     RESOURCEDISPLAYTYPE_GENERIC = 0;
     RESOURCEDISPLAYTYPE_GROUP = 5;
     RESOURCEDISPLAYTYPE_SERVER = 2;
     RESOURCEDISPLAYTYPE_SHARE = 3;
  { KEY_EVENT_RECORD structure  }
     CAPSLOCK_ON = 128;
     ENHANCED_KEY = 256;
     LEFT_ALT_PRESSED = 2;
     LEFT_CTRL_PRESSED = 8;
     NUMLOCK_ON = 32;
     RIGHT_ALT_PRESSED = 1;
     RIGHT_CTRL_PRESSED = 4;
     SCROLLLOCK_ON = 64;
     SHIFT_PRESSED = 16;
  { MOUSE_EVENT_RECORD structure  }
     FROM_LEFT_1ST_BUTTON_PRESSED = 1;
     RIGHTMOST_BUTTON_PRESSED = 2;
     FROM_LEFT_2ND_BUTTON_PRESSED = 4;
     FROM_LEFT_3RD_BUTTON_PRESSED = 8;
     FROM_LEFT_4TH_BUTTON_PRESSED = 16;
     DOUBLE_CLICK = 2;
     MOUSE_MOVED = 1;
  { INPUT_RECORD structure  }
     KEY_EVENT = 1;
     _MOUSE_EVENT = 2; {conflict with function mouse_event}
     cMOUSE_EVENT = 2;
     WINDOW_BUFFER_SIZE_EVENT = 4;
     MENU_EVENT = 8;
     FOCUS_EVENT = 16;
  { BITMAPINFOHEADER structure  }
     BI_RGB = 0;
     BI_RLE8 = 1;
     BI_RLE4 = 2;
     BI_BITFIELDS = 3;
  { Extensions to OpenGL  }
  { wglUseFontOutlines  }
     WGL_FONT_LINES = 0;
     WGL_FONT_POLYGONS = 1;
  { LAYERPLANEDESCRIPTOR structure  }
  { PIXELFORMATDESCRIPTOR structure  }
     PFD_GENERIC_FORMAT = $40;
     PFD_NEED_PALETTE = $80;
     PFD_NEED_SYSTEM_PALETTE = $100;
     PFD_SWAP_EXCHANGE = $200;
     PFD_SWAP_COPY = $400;
     PFD_SWAP_LAYER_BUFFERS = $800;
     PFD_GENERIC_ACCELERATED = $1000;
     PFD_SUPPORT_DIRECTDRAW = $2000;
  { TEXTMETRIC structure  }
     TMPF_FIXED_PITCH = $1;
     TMPF_VECTOR = $2;
     TMPF_TRUETYPE = $4;
     TMPF_DEVICE = $8;
     WM_CTLCOLOR = 25;

  { --------------------- old stuff, need to organize! ---------------  }
  { BEGINNING of windowsx.h stuff from old headers:  }
  {  Not convertable by H2PAS
  #define __CRACK_VOID_F(fn,args) (void)(fn args)
  #define __CRACK_BOOL_F(fn,args) (BOOL)(fn args)
  #define __CRACK_HMENU_F(fn,args) (HMENU)(fn args)
  #define __CRACK_HWND_F(fn,args) (HWND)(fn args)
  #define __CRACK_LONG_F(fn, args) (LRESULT)(fn args)
  #define __CRACK_ZERO_F(fn, args)  (fn args,0)
   }
  { was #define dname(params) def_expr }
  function GetFirstChild(h:HWND):HWND;

  { was #define dname(params) def_expr }
  function GetNextSibling(h:HWND):HWND;

  { was #define dname(params) def_expr }
  function GetWindowID(h:HWND):longint;

  { was #define dname(params) def_expr }
  function SubclassWindow(h:HWND; p:LONG):LONG;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_COMMAND_CMD(w,l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_COMMAND_ID(w,l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_CTLCOLOR_HDC(w,l,msg : longint) : HDC;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_CTLCOLOR_HWND(w,l,msg : longint) : HWND;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_HSCROLL_CODE(w,l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_HSCROLL_HWND(w,l : longint) : HWND;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_HSCROLL_POS(w,l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_MDIACTIVATE_FACTIVATE(h,a,b : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_MDIACTIVATE_HWNDACTIVATE(a,b : longint) : HWND;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_MDIACTIVATE_HWNDDEACT(a,b : longint) : HWND;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_VSCROLL_CODE(w,l : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_VSCROLL_HWND(w,l : longint) : HWND;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_VSCROLL_POS(w,l : longint) : longint;
    { return type might be wrong }

  {  Not convertable by H2PAS
  #define FORWARD_WM_CLOSE(h, fn)                 __CRACK_VOID_F(fn,(h, WM_CLOSE, 0, 0))
  #define FORWARD_WM_COMMAND(h, id, c, n, fn)     __CRACK_VOID_F(fn,(h, WM_COMMAND, MAKEWPARAM(id,n), (LPARAM)c))
  #define FORWARD_WM_CREATE(h, p, fn)             __CRACK_BOOL_F(fn,(h, WM_CREATE, 0, (LPARAM)p))
  #define FORWARD_WM_DESTROY(h, fn)               __CRACK_VOID_F(fn,(h, WM_DESTROY, 0, 0))
  #define FORWARD_WM_ENABLE(h, e, fn)             __CRACK_VOID_F(fn,(h, WM_ENABLE, (WPARAM)e, 0))
  #define FORWARD_WM_INITDIALOG(h, c, l, fn)      __CRACK_BOOL_F(fn,(h, WM_INITDIALOG, (WPARAM)c, l))
  #define FORWARD_WM_MDICASCADE(h, c, fn)         __CRACK_BOOL_F(fn,(h, WM_MDICASCADE, (WPARAM)c, 0))
  #define FORWARD_WM_MDIDESTROY(h, d, fn)         __CRACK_VOID_F(fn,(h, WM_MDIDESTROY, (WPARAM)d, 0))
  #define FORWARD_WM_MDIGETACTIVE(h, fn)          __CRACK_HWND_F(fn,(h, WM_MDIGETACTIVE, 0, 0))
  #define FORWARD_WM_MDIICONARRANGE(h, fn)        __CRACK_VOID_F(fn,(h, WM_MDIICONARRANGE, 0, 0))
  #define FORWARD_WM_MDISETMENU(h, fr, hf, hw, fn) __CRACK_HMENU_F(fn,(h, WM_MDISETMENU, (WPARAM)((fr) ? (hf) : 0), (LPARAM)(hw)))
  #define FORWARD_WM_MDITILE(h, c, fn)            __CRACK_BOOL_F(fn,(h, WM_MDITILE, (WPARAM)(c), 0))
  #define FORWARD_WM_PAINT(h, fn)                 __CRACK_VOID_F(fn,(h, WM_PAINT, 0, 0))
  #define FORWARD_WM_QUERYENDSESSION(h, fn)       __CRACK_BOOL_F(fn,(h, WM_QUERYENDSESSION, 0, 0))
  #define FORWARD_WM_SIZE(h, state, cx, cy, fn)   __CRACK_VOID_F(fn,(h, WM_SIZE, (WPARAM)state, MAKELPARAM(cx, cy)))
  #define FORWARD_WM_SYSCOMMAND(h, c, x, y, fn)   __CRACK_VOID_F(fn,(h, WM_SYSCOMMAND, (WPARAM)c, MAKELPARAM(x, y)))

  #define HANDLE_WM_CLOSE(h, w, l, fn)            __CRACK_ZERO_F(fn,(h));
  #define HANDLE_WM_COMMAND(h, w, l, fn)          __CRACK_ZERO_F(fn,(h, SEXT_LOWORD(w), (HWND)l, HIWORD(w)))
  #define HANDLE_WM_CREATE(h, w, l, fn)           (LRESULT)((fn(h, (CREATESTRUCT  )l)) ? 0 : -1)
  #define HANDLE_WM_DESTROY(h, w, l, fn)          __CRACK_ZERO_F(fn,(h))
  #define HANDLE_WM_ENABLE(h, w, l, fn)           __CRACK_ZERO_F(fn,(h, (BOOL)w))
  #define HANDLE_WM_INITDIALOG(h, w, l, fn)       __CRACK_LONG_F(fn,(h, (HWND)w, l))
  #define HANDLE_WM_MDICASCADE(h, w, l, fn)       __CRACK_LONG_F(fn, (h, (UINT)w)
  #define HANDLE_WM_MDIDESTROY(h, w, l, fn)       __CRACK_ZERO_F(fn,(h, (HWND)w))
  #define HANDLE_WM_MDIGETACTIVE(h, w, l, fn)     __CRACK_LONG_F(fn,(h))
  #define HANDLE_WM_MDIICONARRANGE(h, w, l, fn)   __CRACK_ZERO_F(fn,(h))
  #define HANDLE_WM_MDISETMENU(h, w, l, fn)       __CRACK_LONG_F(fn,(h, (BOOL)w, (HMENU)w, (HMENU)l)
  #define HANDLE_WM_MDITILE(h, w, l, fn)          __CRACK_LONG_F(fn,(h, (UINT)w))
  #define HANDLE_WM_PAINT(h, w, l, fn)            __CRACK_ZERO_F(fn,(h))
  #define HANDLE_WM_QUERYENDSESSION(h, w, l, fn)  MAKELRESULT(fn(h), 0)
  #define HANDLE_WM_SIZE(h, w, l, fn)             __CRACK_ZERO_F(fn,(h, (UINT)w, SEXT_LOWORD(l), SEXT_HIWORD(l)))
  #define HANDLE_WM_SYSCOMMAND(h, w, l, fn)       __CRACK_ZERO_F(fn,(h, (UINT)w, SEXT_LOWORD(l), SEXT_HIWORD(l)))
   }
  { Totally disgusting! get wParam and lParam from the environment !  }
  {  Not convertable by H2PAS
  #define HANDLE_MSG(h, message, fn) case message: return HANDLE_##message(h, wParam, lParam, fn)
   }
  { END OF windowsx.h stuff from old headers  }
  { ------------------------------------------------------------------  }
  { BEGINNING of shellapi.h stuff from old headers  }

  const
     SE_ERR_SHARE = 26;
     SE_ERR_ASSOCINCOMPLETE = 27;
     SE_ERR_DDETIMEOUT = 28;
     SE_ERR_DDEFAIL = 29;
     SE_ERR_DDEBUSY = 30;
     SE_ERR_NOASSOC = 31;
  { END OF shellapi.h stuff from old headers  }
  { ------------------------------------------------------------------  }
  { From ddeml.h in old Cygnus headers  }
     XCLASS_BOOL = $1000;
     XCLASS_DATA = $2000;
     XCLASS_FLAGS = $4000;
     XCLASS_MASK = $fc00;
     XCLASS_NOTIFICATION = $8000;
     XTYPF_NOBLOCK = $0002;
     XTYP_ADVDATA = $4010;
     XTYP_ADVREQ = $2022;
     XTYP_ADVSTART = $1030;
     XTYP_ADVSTOP = $8040;
     XTYP_CONNECT = $1062;
     XTYP_CONNECT_CONFIRM = $8072;
     XTYP_DISCONNECT = $80c2;
     XTYP_EXECUTE = $4050;
     XTYP_POKE = $4090;
     XTYP_REQUEST = $20b0;
     XTYP_WILDCONNECT = $20E2;
     XTYP_REGISTER = $80A2;
     XTYP_ERROR = $8002;
     XTYP_XACT_COMPLETE = $8080;
     XTYP_UNREGISTER = $80D2;
     DMLERR_DLL_USAGE = $4004;
     DMLERR_INVALIDPARAMETER = $4006;
     DMLERR_NOTPROCESSED = $4009;
     DMLERR_POSTMSG_FAILED = $400c;
     DMLERR_SERVER_DIED = $400e;
     DMLERR_SYS_ERROR = $400f;
     DMLERR_BUSY = $4001;
     DMLERR_DATAACKTIMEOUT = $4002;
     DMLERR_ADVACKTIMEOUT = $4000;
     DMLERR_DLL_NOT_INITIALIZED = $4003;
     DMLERR_LOW_MEMORY = $4007;
     DMLERR_MEMORY_ERROR = $4008;
     DMLERR_POKEACKTIMEOUT = $400b;
     DMLERR_NO_CONV_ESTABLISHED = $400a;
     DMLERR_REENTRANCY = $400d;
     DMLERR_UNFOUND_QUEUE_ID = $4011;
     DMLERR_UNADVACKTIMEOUT = $4010;
     DMLERR_EXECACKTIMEOUT = $4005;
     DDE_FACK = $8000;
     DDE_FNOTPROCESSED = $0000;
     DNS_REGISTER = $0001;
     DNS_UNREGISTER = $0002;
     CP_WINANSI = 1004;
     CP_WINUNICODE = 1200;
  {  Not convertable by H2PAS
  #define EXPENTRY CALLBACK
   }
     APPCLASS_STANDARD = $00000000;
  { End of stuff from ddeml.h in old Cygnus headers  }
  { -----------------------------------------------  }
     BKMODE_LAST = 2;
     CTLCOLOR_MSGBOX = 0;
     CTLCOLOR_EDIT = 1;
     CTLCOLOR_LISTBOX = 2;
     CTLCOLOR_BTN = 3;
     CTLCOLOR_DLG = 4;
     CTLCOLOR_SCROLLBAR = 5;
     CTLCOLOR_STATIC = 6;
     CTLCOLOR_MAX = 7;
     META_SETMAPMODE = $0103;
     META_SETWINDOWORG = $020B;
     META_SETWINDOWEXT = $020C;
     POLYFILL_LAST = 2;
     STATUS_WAIT_0 = $00000000;
     STATUS_ABANDONED_WAIT_0 = $00000080;
     STATUS_USER_APC = $000000C0;
     STATUS_TIMEOUT = $00000102;
     STATUS_PENDING = $00000103;
     STATUS_GUARD_PAGE_VIOLATION = $80000001;
     STATUS_DATATYPE_MISALIGNMENT = $80000002;
     STATUS_BREAKPOINT = $80000003;
     STATUS_SINGLE_STEP = $80000004;
     STATUS_IN_PAGE_ERROR = $C0000006;
     STATUS_INVALID_HANDLE = $C0000008;
     STATUS_ILLEGAL_INSTRUCTION = $C000001D;
     STATUS_NONCONTINUABLE_EXCEPTION = $C0000025;
     STATUS_INVALID_DISPOSITION = $C0000026;
     STATUS_ARRAY_BOUNDS_EXCEEDED = $C000008C;
     STATUS_FLOAT_DENORMAL_OPERAND = $C000008D;
     STATUS_FLOAT_DIVIDE_BY_ZERO = $C000008E;
     STATUS_FLOAT_INEXACT_RESULT = $C000008F;
     STATUS_FLOAT_INVALID_OPERATION = $C0000090;
     STATUS_FLOAT_OVERFLOW = $C0000091;
     STATUS_FLOAT_STACK_CHECK = $C0000092;
     STATUS_FLOAT_UNDERFLOW = $C0000093;
     STATUS_INTEGER_DIVIDE_BY_ZERO = $C0000094;
     STATUS_INTEGER_OVERFLOW = $C0000095;
     STATUS_PRIVILEGED_INSTRUCTION = $C0000096;
     STATUS_STACK_OVERFLOW = $C00000FD;
     STATUS_CONTROL_C_EXIT = $C000013A;
{$define EXCEPTION_CTRL_C}
{ Declared above.
     PROCESSOR_ARCHITECTURE_INTEL = 0;
     PROCESSOR_ARCHITECTURE_MIPS = 1;
     PROCESSOR_ARCHITECTURE_ALPHA = 2;
     PROCESSOR_ARCHITECTURE_PPC = 3;
}
  { was #define dname(params) def_expr }
  function FreeModule(h:HINST):WINBOOL;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function MakeProcInstance(p,i : longint) : longint;
    { return type might be wrong }

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function FreeProcInstance(p : longint) : longint;
    { return type might be wrong }


  const
    { _fmemcpy = memcpy; these are functions }
  { Used by wxwindows.  }
     SIZEFULLSCREEN = SIZE_MAXIMIZED;
     SIZENORMAL = SIZE_RESTORED;
     SIZEICONIC = SIZE_MINIMIZED;
     { NPLOGPALETTE = PLOGPALETTE; probably a type }
  { In the old winnt.h  }
  (*  Not convertable by H2PAS anyhow with if 0
  #if 0
  #ifdef __ANAL__
  #define DECLARE_HANDLE(h) struct h##__ { int dummy; }; typedef struct h##__  h
  #else
  #define DECLARE_HANDLE(h)  typedef void  h
  #endif
  DECLARE_HANDLE(HANDLE);
  #endif
   *)

{$ifdef i386}                              //+winnt

  { x86  }
  { The doc refered me to winnt.h, so I had to look...  }

  const
     SIZE_OF_80387_REGISTERS = 80;         //winnt
     MAXIMUM_SUPPORTED_EXTENSION = 512;    //winnt
  { Values for contextflags  }
     CONTEXT_i386 = $10000;
     CONTEXT_i486 = $10000;                //+winnt
     CONTEXT_CONTROL = CONTEXT_i386 or 1;
     CONTEXT_INTEGER = CONTEXT_i386 or 2;
     CONTEXT_SEGMENTS = CONTEXT_i386 or 4;
     CONTEXT_FLOATING_POINT = CONTEXT_i386 or 8;
     CONTEXT_DEBUG_REGISTERS = CONTEXT_i386 or $10;
     CONTEXT_EXTENDED_REGISTERS = CONTEXT_i386 or $20; //+winnt
     CONTEXT_FULL = (CONTEXT_CONTROL or CONTEXT_INTEGER) or CONTEXT_SEGMENTS;
  { our own invention  }
     FLAG_TRACE_BIT = $100;
     CONTEXT_DEBUGGER = CONTEXT_FULL or CONTEXT_FLOATING_POINT;
{$endif i386}

{$ifdef _MIPS_} //+winnt all block added
//
// Processor Feature Values used in IsProcessorFeaturePresent API
//
     PF_MIPS_MIPSII          = $81000001; // MIPSII  instruction set
     PF_MIPS_MIPSIII         = $81000002; // MIPSIII instruction set
     PF_MIPS_MIPSIV          = $81000003; // MIPSIV  instruction set
     PF_MIPS_SMART_ASE       = $81000004; // MIPS smart card arch. specific ext.
     PF_MIPS_MIPS16          = $81000005; // MIPS16  instruction set
     PF_MIPS_MIPS32          = $81000006; // MIPS32  instruction set
     PF_MIPS_MIPS64          = $81000007; // MIPS64  instruction set
     PF_MIPS_FPU             = $81000008; // FPU support
     PF_MIPS_CPU_4KEX        = $81000009; // "R4K" exception model
     PF_MIPS_CPU_4KTLB       = $8100000A; // "R4K" TLB handler
     PF_MIPS_CPU_32FPR       = $8100000B; // 32 dbl. prec. FP registers
     PF_MIPS_CPU_COUNTER     = $8100000C; // Cycle count/compare
     PF_MIPS_CPU_WATCH       = $8100000D; // watchpoint registers
     PF_MIPS_CPU_DIVEC       = $8100000E; // dedicated interrupt vector
     PF_MIPS_CPU_VCE         = $8100000F; // virt. coherence conflict possible
     PF_MIPS_CPU_CACHE_CDEX  = $81000010; // Create_Dirty_Exclusive CACHE op
     PF_MIPS_CPU_MCHECK      = $81000011; // Machine check exception
     PF_MIPS_CPU_EJTAG       = $81000012; // EJTAG exception
     PF_MIPS_PERF_COUNTER    = $81000013; // perf counter
     PF_MIPS_ARCH_2          = $81000014; // arch. release 2

     CONTEXT_R4000 = $00010000;    // r4000 context

     CONTEXT_CONTROL          = CONTEXT_R4000 or $00000001;
     CONTEXT_FLOATING_POINT   = CONTEXT_R4000 or $00000002;
     CONTEXT_INTEGER          = CONTEXT_R4000 or $00000004;
     CONTEXT_EXTENDED_FLOAT   = CONTEXT_FLOATING_POINT or $00000008;
     CONTEXT_EXTENDED_INTEGER = CONTEXT_INTEGER or $00000010;
     CONTEXT_FULL             = CONTEXT_CONTROL or CONTEXT_FLOATING_POINT or
                            CONTEXT_INTEGER or CONTEXT_EXTENDED_INTEGER;
{$ifdef _MIPS64} //+winnt
 CONTEXT32_LENGTH=$1B8;
{$else}
 CONTEXT32_LENGTH=$130;  // The original 32-bit Context length (pre NT 4.0)
{$endif _MIPS64}

{$endif _MIPS_} //+winnt

{$ifdef _PPC_} //~winnt, now _PPC_ instead of __PPC__, moved after mips
  const
     CONTEXT_CONTROL = 1;
     CONTEXT_FLOATING_POINT = 2;
     CONTEXT_INTEGER = 4;
     CONTEXT_DEBUG_REGISTERS = 8;
     CONTEXT_FULL = (CONTEXT_CONTROL or CONTEXT_FLOATING_POINT) or CONTEXT_INTEGER;
     CONTEXT_DEBUGGER = CONTEXT_FULL;
{$endif _PPC_} //~winnt

{$ifdef _MPPC_} //+winnt all block
  const
     CONTEXT_CONTROL = 1;
     CONTEXT_FLOATING_POINT = 2;
     CONTEXT_INTEGER = 4;
     CONTEXT_DEBUG_REGISTERS = 8;
     CONTEXT_FULL = (CONTEXT_CONTROL or CONTEXT_FLOATING_POINT) or CONTEXT_INTEGER;
{$endif _MPPC_} //+winnt

{$ifdef _IA64_} //+winnt all block
     SIZE_OF_80387_REGISTERS = 80;
     CONTEXT_IA64 = $00080000;    // IA64 context
     CONTEXT_CONTROL = CONTEXT_IA64 or $00000001;
     CONTEXT_LOWER_FLOATING_POINT = CONTEXT_IA64 or $00000002;
     CONTEXT_HIGHER_FLOATING_POINT = CONTEXT_IA64 or $00000004;
     CONTEXT_INTEGER = CONTEXT_IA64 or $00000008;
     CONTEXT_DEBUG = CONTEXT_IA64 or $00000010;

     CONTEXT_FLOATING_POINT = CONTEXT_LOWER_FLOATING_POINT or CONTEXT_HIGHER_FLOATING_POINT;
     CONTEXT_FULL = CONTEXT_CONTROL or CONTEXT_FLOATING_POINT or CONTEXT_INTEGER;

     CONTEXT_i386 = $00010000;    // this assumes that i386 and
     CONTEXT_i486 = $00010000;    // i486 have identical context records
     CONTEXT_X86 = $00010000;    // X86 have identical context records

     CONTEXT86_CONTROL = CONTEXT_i386 or $00000001; // SS:SP, CS:IP, FLAGS, BP
     CONTEXT86_INTEGER = CONTEXT_i386 or $00000002; // AX, BX, CX, DX, SI, DI
     CONTEXT86_SEGMENTS = CONTEXT_i386 or $00000004; // DS, ES, FS, GS
     CONTEXT86_FLOATING_POINT = CONTEXT_i386 or $00000008; // 387 state
     CONTEXT86_DEBUG_REGISTERS = CONTEXT_i386 or $00000010; // DB 0-3,6,7

     CONTEXT86_FULL = CONTEXT86_CONTROL or CONTEXT86_INTEGER or CONTEXT86_SEGMENTS;    // context corresponding to set flags will be returned.
{$endif _IA64_} //+winnt

{$ifdef SHx} //+winnt all block added
//
// Processor Feature Values used in IsProcessorFeaturePresent API
//
     PF_SHX_SH3 = $82000001;
     PF_SHX_SH4 = $82000002;
     PF_SHX_SH5 = $82000003;
     PF_SHX_DSP = $82000004;
     PF_SHX_FPU = $82000005;
//
// The following flags control the contents of the CONTEXT structure.
//
     CONTEXT_SH3 = $00000040;
     CONTEXT_SH4  = $000000c0;  // CONTEXT_SH3 | 0x80 - must contain the SH3 bits

{$ifdef SH3}
     CONTEXT_CONTROL = CONTEXT_SH3 or $00000001;
     CONTEXT_INTEGER = CONTEXT_SH3 or $00000002;
     CONTEXT_DEBUG_REGISTERS = CONTEXT_SH3 or $00000008;
     CONTEXT_DSP_REGISTERS = CONTEXT_SH3 or $00000010;
     CONTEXT_FULL = CONTEXT_CONTROL or CONTEXT_INTEGER or CONTEXT_DEBUG_REGISTERS or CONTEXT_DSP_REGISTERS;
{$else} // SH4
     CONTEXT_CONTROL = CONTEXT_SH4 or $00000001;
     CONTEXT_INTEGER = CONTEXT_SH4 or $00000002;
     CONTEXT_DEBUG_REGISTERS = CONTEXT_SH4 or $00000008;
     CONTEXT_FLOATING_POINT = CONTEXT_SH4 or $00000004;
     CONTEXT_FULL = CONTEXT_CONTROL or CONTEXT_INTEGER or CONTEXT_DEBUG_REGISTERS or CONTEXT_FLOATING_POINT;
{$endif SH3}

{$endif SHx} //+winnt

{$ifdef CPUARM} //+winnt all block added
//
// Processor Feature Values used in IsProcessorFeaturePresent API
//
     PF_ARM_V4                       = $80000001;
     PF_ARM_V5                       = $80000002;
     PF_ARM_V6                       = $80000003;
     PF_ARM_V7                       = $80000004;
     PF_ARM_THUMB                    = $80000005;
     PF_ARM_JAZELLE                  = $80000006;
     PF_ARM_DSP                      = $80000007;
     PF_ARM_MOVE_CP                  = $80000008;
     PF_ARM_VFP10                    = $80000009;
     PF_ARM_MPU                      = $8000000A;
     PF_ARM_WRITE_BUFFER             = $8000000B;
     PF_ARM_MBX                      = $8000000C;
     PF_ARM_L2CACHE                  = $8000000D;
     PF_ARM_PHYSICALLY_TAGGED_CACHE  = $8000000E;
     PF_ARM_VFP_SINGLE_PRECISION     = $8000000F;
     PF_ARM_VFP_DOUBLE_PRECISION     = $80000010;
     PF_ARM_ITCM                     = $80000011;
     PF_ARM_DTCM                     = $80000012;
     PF_ARM_UNIFIED_CACHE            = $80000013;
     PF_ARM_WRITE_BACK_CACHE         = $80000014;
     PF_ARM_CACHE_CAN_BE_LOCKED_DOWN = $80000015;
     PF_ARM_L2CACHE_MEMORY_MAPPED    = $80000016;
     PF_ARM_L2CACHE_COPROC           = $80000017;


<<<<<<< HEAD
<<<<<<< HEAD
// Specific OEM extensions
=======
// Specific OEM extentions
>>>>>>> graemeg/fixes_2_2
=======
// Specific OEM extentions
>>>>>>> origin/fixes_2_2
     PF_ARM_INTEL_XSCALE             = $80010001;
     PF_ARM_INTEL_PMU                = $80010002;
     PF_ARM_INTEL_WMMX               = $80010003;

     CONTEXT_ARM = $0000040;
     CONTEXT_CONTROL = CONTEXT_ARM or $00000001;
     CONTEXT_INTEGER = CONTEXT_ARM or $00000002;
     CONTEXT_FLOATING_POINT = CONTEXT_ARM or $00000004;

     CONTEXT_FULL = CONTEXT_CONTROL or CONTEXT_INTEGER or CONTEXT_FLOATING_POINT;

     NUM_VFP_REGS = 32;
     NUM_EXTRA_CONTROL_REGS = 8;

{$endif CPUARM} //+winnt

  const
     { ASCIICHAR = AsciiChar; this is the kind of thing that can
     make problems for FPC !! }
  {  ignored in H2PAS
  #define FAR
     handled in H2PAS
  #define PACKED __attribute__((packed))
   }
     FILTER_TEMP_DUPLICATE_ACCOUNT = $0001;
     FILTER_NORMAL_ACCOUNT = $0002;
     FILTER_INTERDOMAIN_TRUST_ACCOUNT = $0008;
     FILTER_WORKSTATION_TRUST_ACCOUNT = $0010;
     FILTER_SERVER_TRUST_ACCOUNT = $0020;
     LOGON32_LOGON_INTERACTIVE = $02;
     LOGON32_LOGON_BATCH = $04;
     LOGON32_LOGON_SERVICE = $05;
     LOGON32_PROVIDER_DEFAULT = $00;
     LOGON32_PROVIDER_WINNT35 = $01;
     QID_SYNC = $FFFFFFFF;
  { Magic numbers in PE executable header.   }
  { e_magic field  }
     IMAGE_DOS_SIGNATURE = $5a4d;
  { nt_signature field  }
     IMAGE_NT_SIGNATURE = $4550;


  { Severity values }
    SEVERITY_SUCCESS = 0;
    SEVERITY_ERROR = 1;

 { Variant type codes (wtypes.h).
    Some, not all though }
    VT_EMPTY           = 0;
    VT_NULL            = 1;
    VT_I2              = 2;
    VT_I4              = 3;
    VT_R4              = 4;
    VT_R8              = 5;
    VT_BSTR            = 8;
    VT_ERROR           = 10;
    VT_BOOL            = 11;
    VT_UI1             = 17;
    VT_BYREF        = $4000;
    VT_RESERVED     = $8000;

{ Define the facility codes }

const
  FACILITY_WINDOWS                     = 8;
  FACILITY_STORAGE                     = 3;
  FACILITY_RPC                         = 1;
  FACILITY_SSPI                        = 9;
  FACILITY_WIN32                       = 7;
  FACILITY_CONTROL                     = 10;
  FACILITY_NULL                        = 0;
  FACILITY_INTERNET                    = 12;
  FACILITY_ITF                         = 4;
  FACILITY_DISPATCH                    = 2;
  FACILITY_CERT                        = 11;

{ Manually added, bug 2672}
  ICON_SMALL                           = 0;
  ICON_BIG                             = 1;

// Manually added, bug 3270
  COLOR_HOTLIGHT                = 26;
  //COLOR_GRADIENTACTIVECAPTION   = 27; moved to GetSysColor
  //COLOR_GRADIENTINACTIVECAPTION = 28;
  COLOR_MENUHILIGHT             = 29;
  COLOR_MENUBAR                 = 30;

<<<<<<< HEAD
<<<<<<< HEAD
=======
  WM_APP                        = $8000;

>>>>>>> graemeg/fixes_2_2
=======
  WM_APP                        = $8000;

>>>>>>> origin/fixes_2_2
  // ShellExecuteEx mask
  SEE_MASK_NOCLOSEPROCESS =  $00000040;
  SEE_MASK_FLAG_NO_UI     =  $00000400;


const
      INPUT_MOUSE     = 0;
      INPUT_KEYBOARD  = 1;
      INPUT_HARDWARE  = 2;

// Constants used by GetMessageSource function.
const
      MSGSRC_UNKNOWN              = 0;
      MSGSRC_SOFTWARE_POST        = 1;
      MSGSRC_HARDWARE_KEYBOARD    = 2;

<<<<<<< HEAD
<<<<<<< HEAD
// define dwFlag values for LoadFSDEx function...
const
      LOADFSD_ASYNCH        = $0000;
      LOADFSD_SYNCH        = $0001;

// flag values for ResourceRequestEx()
const
      RREXF_REQUEST_EXCLUSIVE         = $0001;


{* File:    ResMgr.h
 *
 * Purpose: Resource manager definitions
 * The resource manager itself is part of the device manager (device.exe)
 * and its affliated entry points are declared in winbase.h.
 *
 *}
// Resource ids less than BASE_OEM are reserved by Microsoft;
// Resource ids between these limits are for OEM use.
const
      RESMGR_BASE_OEM     = $00080000;
      RESMGR_MAX_OEM      = $000FFFFF;
// Resource ids greater than MAX_OEM are reserved for future use by Microsoft.

// --- FIVE hex digits (not four) in these constants!
const
      RESMGR_IRQ          = $00001;
      RESMGR_IOSPACE      = $00002;
// End-of-file ResMgr.h

const
      MONITOR_DEFAULTTONULL    = $00000000;
      MONITOR_DEFAULTTOPRIMARY = $00000001;
      MONITOR_DEFAULTTONEAREST = $00000002;

const
      MONITORINFOF_PRIMARY        = $00000001;

const
      ENUM_CURRENT_SETTINGS  = DWORD(-1);
      ENUM_REGISTRY_SETTINGS = DWORD(-2);

// For EnumDisplayDevices
const
      DISPLAY_DEVICE_ATTACHED_TO_DESKTOP = $00000001;
      DISPLAY_DEVICE_MULTI_DRIVER        = $00000002;
      DISPLAY_DEVICE_PRIMARY_DEVICE      = $00000004;
      DISPLAY_DEVICE_MIRRORING_DRIVER    = $00000008;
      DISPLAY_DEVICE_VGA_COMPATIBLE      = $00000010;
      DISPLAY_DEVICE_REMOVABLE           = $00000020;
      DISPLAY_DEVICE_MODESPRUNED         = $08000000;
      DISPLAY_DEVICE_REMOTE              = $04000000;
      DISPLAY_DEVICE_DISCONNECT          = $02000000;

// Child device state
const
      DISPLAY_DEVICE_ACTIVE              = $00000001;
      DISPLAY_DEVICE_ATTACHED            = $00000002;

// winuserm.h
// Mobile extensions to winuser.h

//***  VK_T* Virtual keycodes for telephone keyboard
{
// the phone keypad maps to VK_* as shown below.
// some keys might not be present, e.g. VK_TFLIP
    sofkey1     softkey2    VK_TSOFT1, VK_TSOFT2
            ^               VK_TUP
        <   +   >           VK_TLEFT, VK_TACTION, VK_TRIGHT
            v               VK_TDOWN
    home        back        VK_THOME, VK_TBACK
    talk        end         VK_TTALK, VK_TEND
    1       2       3       VK_T0..VK_T9
    4       5       6       ...
    7       8       9       ...
    *       0       #       VK_TSTAR, VK_TPOUND
// other buttons include
    VK_TRECORD
    VK_TPOWER, VK_TVOLUMEUP, VK_TVOLUMEDOWN
    VK_TFLIP
}
const
      VK_TSOFT1               = VK_F1;               // Softkey 1
      VK_TSOFT2               = VK_F2;               // Softkey 2
      VK_TTALK                = VK_F3;               // Talk
      VK_TEND                 = VK_F4;               // End
      VK_THOME                = VK_LWIN;             // Home
      VK_TBACK                = VK_ESCAPE;           // Back

      VK_TACTION              = VK_RETURN;       // Action
      VK_TRECORD              = VK_F10;          // Record
      VK_TFLIP                = VK_F17;          // Flip
      VK_TPOWER               = VK_F18;          // Power
      VK_TVOLUMEUP            = VK_F6;           // Volume Up
      VK_TVOLUMEDOWN          = VK_F7;           // Volume Down

      VK_TSPEAKERPHONE_TOGGLE = VK_F16;
      VK_END_ALL_DATA_CALLS   = VK_F15;


      VK_TUP                  = VK_UP;               // Up
      VK_TDOWN                = VK_DOWN;             // Down
      VK_TLEFT                = VK_LEFT;             // Left
      VK_TRIGHT               = VK_RIGHT;            // Right

      VK_T0                   = longint(WideChar('0'));                // 0 .. 9 keys
      VK_T1                   = longint(WideChar('1'));
      VK_T2                   = longint(WideChar('2'));
      VK_T3                   = longint(WideChar('3'));
      VK_T4                   = longint(WideChar('4'));
      VK_T5                   = longint(WideChar('5'));
      VK_T6                   = longint(WideChar('6'));
      VK_T7                   = longint(WideChar('7'));
      VK_T8                   = longint(WideChar('8'));
      VK_T9                   = longint(WideChar('9'));
      VK_TSTAR                = VK_F8;               // *
      VK_TPOUND               = VK_F9;               // #

      VK_SYMBOL               = VK_F11;              // Symbol (SYM) key

      VK_REDKEY               = VK_F19;              // Sent by a keypad that has a special red function key
      VK_ROCKER               = VK_F20;              // Arrow keys came from a Rocker
      VK_DPAD                 = VK_F21;              // Arrow keys came from a DPAD
      VK_ACTION               = VK_F23;              // Sent with VK_RETURN when doing Action on PPC rockers

      VK_VOICEDIAL            = VK_F24;            // Key used to kick off voice dial recognition
      VK_KEYLOCK              = VK_F22;            // Key used to lock the device

// Chorded keys.  DONE is VK_RWIN + VK_F6 and MOJI is VK_RWIN + VK_F7
const
      VK_DONE     = VK_F6;                 // <OK/Close> hotkey code
      MOD_DONE    = MOD_WIN or MOD_KEYUP; // <OK/Close> hotkey modifiers

      VK_MOJI     = VK_F7;                 // Key used to switch between FE language layouts
      MOD_MOJI    = MOD_WIN or MOD_KEYUP;  //Modifiers so MOJI goes to the shell


// App keys
const
      VK_APP_FIRST    = $C1;
      VK_APP1         = $C1;
      VK_APP2         = $C2;
      VK_APP3         = $C3;
      VK_APP4         = $C4;
      VK_APP5         = $C5;
      VK_APP6         = $C6;
      VK_APP_LAST     = $C6;

// Specifies keys whose autorepeats should be silent. (Used by the keyboard driver.)
// All keys except the nav keys should have silent autorepeats.
// #define SILENT_REPEAT(uVKey) (uVKey < VK_LEFT || uVKey > VK_DOWN)
function SILENT_REPEAT(uVKey:longint):BOOL; inline;

// #define Edit_GetInputMode(hwndCtl, fActual)     ((int)(DWORD)SNDMSG((hwndCtl), EM_GETINPUTMODE, 0L, (LPARAM)(BOOL)(fActual)))
function Edit_GetInputMode(hwndCtl:HWND; fActual:BOOL):longint;
// #define Edit_SetInputMode(hwndCtl, nInputMode)  ((BOOL)(DWORD)SNDMSG((hwndCtl), EM_SETINPUTMODE, 0L, (LPARAM)(int)(nInputMode)))
function Edit_SetInputMode(hwndCtl:HWND; nInputMode:longint):Bool;

// #define Edit_SetSymbols(hwndCtl, pszSymbols)    ((BOOL)(DWORD)SNDMSG((hwndCtl), EM_SETSYMBOLS, 0L, (LPARAM)(LPCTSTR)(pszSymbols)))
function Edit_SetSymbols(hwndCtl:HWND; pszSymbols:LPCTSTR):BOOL;

// #define Edit_SetExtendedStyle(hwndCtl, dwMask, dwExStyle)    ((DWORD)SNDMSG((hwndCtl), EM_SETEXTENDEDSTYLE, (WPARAM)(DWORD)(dwMask), (LPARAM)(DWORD)(dwExStyle)))
function Edit_SetExtendedStyle(hwndCtl:HWND; dwMask:DWORD; dwExStyle:DWORD):DWORD;

// #define Edit_GetExtendedStyle(hwndCtl)                       ((DWORD)SNDMSG((hwndCtl), EM_GETEXTENDEDSTYLE, 0L, 0L))
function Edit_GetExtendedStyle(hwndCtl:HWND):DWORD;

// Edit control extended styles
const
      ES_EX_CLEARONBACKPRESSHOLD      = $00000001;  // Clear all contents when user does Press&Hold of Back key.
      ES_EX_FOCUSBORDERDISABLED       = $00000002;  // Prevent control from drawing a border when it gains focus.

// IME Input Modes
const
      IM_SPELL           = 0;
      IM_AMBIG           = 1;
      IM_NUMBERS         = 2;
      IM_LAST            = IM_NUMBERS;
      IM_MASK            = $0000FFFF;       // Mask to access mode.

// IME Input Mode Modifier Flags
      IMMF_SETCLR_SHIFT       = $00010000;  // In. Used to set/clear Shift.
      IMMF_SETCLR_CAPSLOCK    = $00020000;  // In. Used to set/clear Capslock.
      IMMF_SHIFT              = $01000000;  // In/Out. Shift state.
      IMMF_CAPSLOCK           = $02000000;  // In/Out. CapsLock state.
      IMMF_MASK               = $FFFF0000;  // Mask to access modifier flags.

// Input modes for EM_SETINPUTMODE/EM_GETINPUTMODE
      EIM_SPELL                   = IM_SPELL;       // Spell input mode
      EIM_AMBIG                   = IM_AMBIG;       // Ambiguous mode if input engine supports this mode. Else defaults to Spell mode.
      EIM_NUMBERS                 = IM_NUMBERS;     // Numbers mode.
      EIM_TEXT                    = IM_LAST + 1;    // User's preferred mode. This is the user's last Spell or Ambig mode selection
                                                    // in an EIM_TEXT edit control.
      EIM_MASK                    = IM_MASK;        // Mask to access mode.

// Macro to access mode
// #define EIM_MODE(x)                 (x & EIM_MASK)
function EIM_MODE(x:DWORD):DWORD;

// Input mode modifier flags
const
      EIMMF_SETCLR_SHIFT          = IMMF_SETCLR_SHIFT;       // In. Used to set/clear Shift.
      EIMMF_SETCLR_CAPSLOCK       = IMMF_SETCLR_CAPSLOCK;    // In. Used to set/clear Capslock.
      EIMMF_SHIFT                 = IMMF_SHIFT;              // In/Out. Shift state.
      EIMMF_CAPSLOCK              = IMMF_CAPSLOCK;           // In/Out. CapsLock state.
      EIMMF_MASK                  = IMMF_MASK;               // Mask to access modifier flags.

// Macro to access modifier flags
// #define EIM_MODIFIERS(x)            (x & EIMMF_MASK)
function EIM_MODIFIERS(x:DWORD):DWORD;


// #define ListBox_GetInputMode(hwndCtl, fActual)              ((int)(DWORD)SNDMSG((hwndCtl), LB_GETINPUTMODE, 0L, (WPARAM)(BOOL)(fActual)))
function ListBox_GetInputMode(hwndCtl:HWND; fActual:BOOL):longint;

// #define ListBox_SetInputMode(hwndCtl, nInputMode)           ((BOOL)(DWORD)SNDMSG((hwndCtl), LB_SETINPUTMODE, 0L, (LPARAM)(int)(nInputMode)))
function ListBox_SetInputMode(hwndCtl:HWND; nInputMode:longint):BOOL;

// Input modes for LB_SETINPUTMODE/LB_GETINPUTMODE
const
      LIM_SPELL                   = IM_SPELL;                // Spell input mode
      LIM_NUMBERS                 = IM_NUMBERS;              // Numbers mode.

// wParam of  WM_IME_REQUEST. Extension for Mobile.
const
      IMR_ISIMEAWARE           = $1000;  // Is window IME aware?

// Return values of IMR_ISIMEAWARE
const
      IMEAF_AWARE         = $00000001; // Window is IME aware
      IMEAF_SMART_CAPS    = $00000002; // Window uses smart caps.

// Extension of WM_HOTKEY MOD_* values for Mobile
const
      MOD_HOLD                = $8000; // Was WM_HOTKEY caused by Press&Hold

const
      MIIM_FULLSTR            = $10000000;

const
      SS_REALSIZEIMAGE    = $00000800;
      
// End-of-winuserm.h

const
      MAXINTATOM = ATOM($C000);
      INVALID_ATOM = ATOM(0);

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif read_interface}


{$ifdef read_implementation}





    { was #define dname(params) def_expr }
    { argument types are unknown }
    { return type might be wrong }
    function IS_UNWINDING( Flag : Longint) : boolean;
    begin
     IS_UNWINDING:=(Flag and EXCEPTION_UNWIND)<>0;
    end;

    { was #define dname(params) def_expr }
    { argument types are unknown }
    { return type might be wrong }
    function IS_DISPATCHING( Flag : Longint) : boolean;
    begin
     IS_DISPATCHING:=(Flag and EXCEPTION_UNWIND)=0;
    end;

    { was #define dname(params) def_expr }
    { argument types are unknown }
    { return type might be wrong }
    function IS_TARGET_UNWIND( Flag : Longint) : Longint;
    begin
     IS_TARGET_UNWIND:=(Flag and EXCEPTION_TARGET_UNWIND);
    end;

    { was #define dname(params) def_expr }
    { argument types are unknown }
    { return type might be wrong }
    function MakeProcInstance(p,i : longint) : longint;
      { return type might be wrong }
      begin
         MakeProcInstance:=p;
      end;

    { was #define dname(params) def_expr }
    { argument types are unknown }
    { return type might be wrong }
    function FreeProcInstance(p : longint) : longint;
      { return type might be wrong }
      begin
         FreeProcInstance:=p;
      end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_COMMAND_CMD(w,l : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_COMMAND_CMD:=HIWORD(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_COMMAND_ID(w,l : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_COMMAND_ID:=LOWORD(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_CTLCOLOR_HDC(w,l,msg : longint) : HDC;
    begin
       GET_WM_CTLCOLOR_HDC:=HDC(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_CTLCOLOR_HWND(w,l,msg : longint) : HWND;
    begin
       GET_WM_CTLCOLOR_HWND:=HWND(l);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_HSCROLL_CODE(w,l : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_HSCROLL_CODE:=LOWORD(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_HSCROLL_HWND(w,l : longint) : HWND;
    begin
       GET_WM_HSCROLL_HWND:=HWND(l);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_HSCROLL_POS(w,l : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_HSCROLL_POS:=HIWORD(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_MDIACTIVATE_FACTIVATE(h,a,b : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_MDIACTIVATE_FACTIVATE:=longint(b = LONG(h));
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_MDIACTIVATE_HWNDACTIVATE(a,b : longint) : HWND;
    begin
       GET_WM_MDIACTIVATE_HWNDACTIVATE:=HWND(b);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_MDIACTIVATE_HWNDDEACT(a,b : longint) : HWND;
    begin
       GET_WM_MDIACTIVATE_HWNDDEACT:=HWND(a);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_VSCROLL_CODE(w,l : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_VSCROLL_CODE:=LOWORD(w);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  function GET_WM_VSCROLL_HWND(w,l : longint) : HWND;
    begin
       GET_WM_VSCROLL_HWND:=HWND(l);
    end;

  { was #define dname(params) def_expr }
  { argument types are unknown }
  { return type might be wrong }
  function GET_WM_VSCROLL_POS(w,l : longint) : longint;
    { return type might be wrong }
    begin
       GET_WM_VSCROLL_POS:=HIWORD(w);
    end;

  { was #define dname(params) def_expr }
  function FreeModule(h:HINST):WINBOOL;
    begin
       FreeModule:=FreeLibrary(h);
    end;

  { was #define dname(params) def_expr }
  function GetNextSibling(h:HWND):HWND;
    begin
       GetNextSibling:=GetWindow(h,GW_HWNDNEXT);
    end;
  { was #define dname(params) def_expr }
  function GetWindowID(h:HWND):longint;
    begin
       GetWindowID:=GetDlgCtrlID(h);
    end;

  { was #define dname(params) def_expr }
  function SubclassWindow(h:HWND; p:LONG):LONG;
    begin
       SubclassWindow:=SetWindowLong(h,GWL_WNDPROC,p);
    end;

  { was #define dname(params) def_expr }
  function GetFirstChild(h:HWND):HWND;
    begin
       //GetFirstChild:=GetTopWindow(h);
       GetFirstChild:=GetWindow(h,GW_CHILD);
    end;

<<<<<<< HEAD
<<<<<<< HEAD
// winuserm.h
// Mobile extensions to winuser.h

// Specifies keys whose autorepeats should be silent. (Used by the keyboard driver.)
// All keys except the nav keys should have silent autorepeats.
// #define SILENT_REPEAT(uVKey) (uVKey < VK_LEFT || uVKey > VK_DOWN)
function SILENT_REPEAT(uVKey:longint):BOOL; inline;
begin
  SILENT_REPEAT:=(uVKey<VK_LEFT) or (uVKey>VK_DOWN);
end;


function Edit_GetInputMode(hwndCtl:HWND; fActual:BOOL):longint; inline;
begin
  Edit_GetInputMode:=longint(SendMessage(hwndCtl,EM_GETINPUTMODE,WPARAM(0),LPARAM(fActual)));
end;

function Edit_SetInputMode(hwndCtl:HWND; nInputMode:longint):Bool; inline;
begin
  Edit_SetInputMode:=BOOL(DWORD(SendMessage(hwndCtl,EM_SETINPUTMODE,WPARAM(0),LPARAM(nInputMode))));
end;

function Edit_SetSymbols(hwndCtl:HWND; pszSymbols:LPCTSTR):BOOL; inline;
begin
  Edit_SetSymbols:=BOOL(SendMessage(hwndCtl,EM_SETSYMBOLS,WPARAM(0),LPARAM(pszSymbols)));
end;

function Edit_SetExtendedStyle(hwndCtl:HWND; dwMask:DWORD; dwExStyle:DWORD):DWORD; inline;
begin
  Edit_SetExtendedStyle:=DWORD(SendMessage(hwndCtl,EM_SETEXTENDEDSTYLE,WPARAM(dwMask),LPARAM(dwExStyle)));
end;

function Edit_GetExtendedStyle(hwndCtl:HWND):DWORD; inline;
begin
  Edit_GetExtendedStyle:=DWORD(SendMessage(hwndCtl,EM_GETEXTENDEDSTYLE,WPARAM(0),LPARAM(0)));
end;


function EIM_MODE(x:DWORD):DWORD; inline;
begin
  EIM_MODE:=x and EIM_MASK;
end;

function EIM_MODIFIERS(x:DWORD):DWORD; inline;
begin
  EIM_MODIFIERS:=x and EIMMF_MASK;
end;

function ListBox_GetInputMode(hwndCtl:HWND; fActual:BOOL):longint; inline;
begin
  ListBox_GetInputMode:=longint(SendMessage(hwndCtl,LB_GETINPUTMODE,WPARAM(0),LPARAM(fActual)));
end;

function ListBox_SetInputMode(hwndCtl:HWND; nInputMode:longint):BOOL; inline;
begin
  ListBox_SetInputMode:=BOOL(SendMessage(hwndCtl,LB_SETINPUTMODE,WPARAM(0),LPARAM(nInputMode)));
end;

// End-of-winuserm.h

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef WINCE}
{$endif WINCE}

{$ifdef WIN32}
{$endif WIN32}

{$endif read_implementation}


