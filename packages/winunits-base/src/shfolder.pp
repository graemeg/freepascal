{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Interface to shfolder.dll

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ---------------------------------------------------------------------
  shfolder.dll is distributed standard with IE5.5, so it should ship
  with 2000/XP or higher but is likely to be installed on NT/95/98 or
  ME as well.  It works on all these systems.

  The info found here is also in the registry:
  HKCU\Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders\
  HKCU\Software\Microsoft\Windows\CurrentVersion\Explorer\User Shell Folders\

  Note that not all CSIDL_* constants are supported by shlfolder.dll,
  they should be supported by the shell32.dll, though again not on all
  systems.
  ---------------------------------------------------------------------}

Unit shfolder;

Interface

Uses
  windows;

Const
  LibName = 'SHFolder.dll';

Const

  CSIDL_PROGRAMS                = $0002; { %SYSTEMDRIVE%\Program Files                                      }
  CSIDL_PERSONAL                = $0005; { %USERPROFILE%\My Documents                                       }
  CSIDL_FAVORITES               = $0006; { %USERPROFILE%\Favorites                                          }
  CSIDL_STARTUP                 = $0007; { %USERPROFILE%\Start menu\Programs\Startup                        }
  CSIDL_RECENT                  = $0008; { %USERPROFILE%\Recent                                             }
  CSIDL_SENDTO                  = $0009; { %USERPROFILE%\Sendto                                             }
  CSIDL_STARTMENU               = $000B; { %USERPROFILE%\Start menu                                         }
  CSIDL_MYMUSIC                 = $000D; { %USERPROFILE%\Documents\My Music                                 }
  CSIDL_MYVIDEO                 = $000E; { %USERPROFILE%\Documents\My Videos                                }
  CSIDL_DESKTOPDIRECTORY        = $0010; { %USERPROFILE%\Desktop                                            }
  CSIDL_NETHOOD                 = $0013; { %USERPROFILE%\NetHood                                            }
  CSIDL_TEMPLATES               = $0015; { %USERPROFILE%\Templates                                          }
  CSIDL_COMMON_STARTMENU        = $0016; { %PROFILEPATH%\All users\Start menu                               }
  CSIDL_COMMON_PROGRAMS         = $0017; { %PROFILEPATH%\All users\Start menu\Programs                      }
  CSIDL_COMMON_STARTUP          = $0018; { %PROFILEPATH%\All users\Start menu\Programs\Startup              }
  CSIDL_COMMON_DESKTOPDIRECTORY = $0019; { %PROFILEPATH%\All users\Desktop                                  }
  CSIDL_APPDATA                 = $001A; { %USERPROFILE%\Application Data (roaming)                         }
  CSIDL_PRINTHOOD               = $001B; { %USERPROFILE%\Printhood                                          }
  CSIDL_LOCAL_APPDATA           = $001C; { %USERPROFILE%\Local Settings\Application Data (non roaming)      }
  CSIDL_COMMON_FAVORITES        = $001F; { %PROFILEPATH%\All users\Favorites                                }
  CSIDL_INTERNET_CACHE          = $0020; { %USERPROFILE%\Local Settings\Temporary Internet Files            }
  CSIDL_COOKIES                 = $0021; { %USERPROFILE%\Cookies                                            }
  CSIDL_HISTORY                 = $0022; { %USERPROFILE%\Local settings\History                             }
  CSIDL_COMMON_APPDATA          = $0023; { %PROFILESPATH%\All Users\Application Data                        }
  CSIDL_WINDOWS                 = $0024; { %SYSTEMROOT%                                                     }
  CSIDL_SYSTEM                  = $0025; { %SYSTEMROOT%\SYSTEM32 (may be system on 95/98/ME)                }
  CSIDL_PROGRAM_FILES           = $0026; { %SYSTEMDRIVE%\Program Files                                      }
  CSIDL_MYPICTURES              = $0027; { %USERPROFILE%\My Documents\My Pictures                           }
  CSIDL_PROFILE                 = $0028; { %USERPROFILE%                                                    }
  CSIDL_PROGRAM_FILES_COMMON    = $002B; { %SYSTEMDRIVE%\Program Files\Common                               }
  CSIDL_COMMON_TEMPLATES        = $002D; { %PROFILEPATH%\All Users\Templates                                }
  CSIDL_COMMON_DOCUMENTS        = $002E; { %PROFILEPATH%\All Users\Documents                                }
  CSIDL_COMMON_ADMINTOOLS       = $002F; { %PROFILEPATH%\All Users\Start Menu\Programs\Administrative Tools }
  CSIDL_ADMINTOOLS              = $0030; { %USERPROFILE%\Start Menu\Programs\Administrative Tools           }
  CSIDL_COMMON_MUSIC            = $0035; { %PROFILEPATH%\All Users\Documents\my music                       }
  CSIDL_COMMON_PICTURES         = $0036; { %PROFILEPATH%\All Users\Documents\my pictures                    }
  CSIDL_COMMON_VIDEO            = $0037; { %PROFILEPATH%\All Users\Documents\my videos                      }
  CSIDL_CDBURN_AREA             = $003B; { %USERPROFILE%\Local Settings\Application Data\Microsoft\CD Burning }
  CSIDL_PROFILES                = $003E; { %PROFILEPATH%                                                    }

  CSIDL_FLAG_CREATE             = $8000; { (force creation of requested folder if it doesn't exist yet)     }

{ Original entry points }

Function SHGetFolderPathA(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall; external LibName name 'SHGetFolderPathA';
Function SHGetFolderPathW(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall; external LibName name 'SHGetFolderPathW';

Function SHGetFolderPath (Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall; external LibName name 'SHGetFolderPathA';


{ For Delphi compatibility }

type
  PFNSHGetFolderPathA = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall;
  PFNSHGetFolderPathW = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall;
  PFNSHGetFolderPath = PFNSHGetFolderPathA;

  TSHGetFolderPathA = PFNSHGetFolderPathA;
  TSHGetFolderPathW = PFNSHGetFolderPathW;
  TSHGetFolderPath = TSHGetFolderPathA;

implementation

const
  SHFolderDll = 'SHFolder.dll';

end.
