unit windirs;

{$mode objfpc}
{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  windows;
=======
  windows,
  strings;
>>>>>>> graemeg/cpstrnew
=======
  windows,
  strings;
>>>>>>> graemeg/cpstrnew
=======
  windows,
  strings;
>>>>>>> graemeg/cpstrnew
=======
  windows,
  strings;
>>>>>>> origin/cpstrnew

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

Function GetWindowsSpecialDir(ID :  Integer) : String;

implementation

uses
  sysutils;

Type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  PFNSHGetFolderPath = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: {$ifdef FPC_UNICODE_RTL}PWideChar{$ELSE}PChar{$ENDIF}): HRESULT; stdcall;
=======
  PFNSHGetFolderPath = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall;

>>>>>>> graemeg/cpstrnew
=======
  PFNSHGetFolderPath = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall;

>>>>>>> graemeg/cpstrnew
=======
  PFNSHGetFolderPath = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall;

>>>>>>> graemeg/cpstrnew
=======
  PFNSHGetFolderPath = Function(Ahwnd: HWND; Csidl: Integer; Token: THandle; Flags: DWord; Path: PChar): HRESULT; stdcall;

>>>>>>> origin/cpstrnew

var
  SHGetFolderPath : PFNSHGetFolderPath = Nil;
  CFGDLLHandle : THandle = 0;

Procedure InitDLL;

Var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  pathBuf: array[0..MAX_PATH-1] of {$ifdef FPC_UNICODE_RTL}WideChar{$else}Ansichar{$endif};
=======
  pathBuf: array[0..MAX_PATH-1] of char;
>>>>>>> graemeg/cpstrnew
=======
  pathBuf: array[0..MAX_PATH-1] of char;
>>>>>>> graemeg/cpstrnew
=======
  pathBuf: array[0..MAX_PATH-1] of char;
>>>>>>> graemeg/cpstrnew
=======
  pathBuf: array[0..MAX_PATH-1] of char;
>>>>>>> origin/cpstrnew
  pathLength: Integer;
begin
  { Load shfolder.dll using a full path, in order to prevent spoofing (Mantis #18185)
    Don't bother loading shell32.dll because shfolder.dll itself redirects SHGetFolderPath
    to shell32.dll whenever possible. }
  pathLength:=GetSystemDirectory(pathBuf, MAX_PATH);
  if (pathLength>0) and (pathLength<MAX_PATH-14) then { 14=length('\shfolder.dll'#0) }
  begin
    StrLCopy(@pathBuf[pathLength],'\shfolder.dll',MAX_PATH-pathLength-1);
    CFGDLLHandle:=LoadLibrary(pathBuf);

    if (CFGDLLHandle<>0) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Pointer(ShGetFolderPath):=GetProcAddress(CFGDLLHandle,{$ifdef FPC_UNICODE_RTL}'SHGetFolderPathW'{$else}'SHGetFolderPathA'{$endif});
=======
      Pointer(ShGetFolderPath):=GetProcAddress(CFGDLLHandle,'SHGetFolderPathA');
>>>>>>> graemeg/cpstrnew
=======
      Pointer(ShGetFolderPath):=GetProcAddress(CFGDLLHandle,'SHGetFolderPathA');
>>>>>>> graemeg/cpstrnew
=======
      Pointer(ShGetFolderPath):=GetProcAddress(CFGDLLHandle,'SHGetFolderPathA');
>>>>>>> graemeg/cpstrnew
=======
      Pointer(ShGetFolderPath):=GetProcAddress(CFGDLLHandle,'SHGetFolderPathA');
>>>>>>> origin/cpstrnew
      If @ShGetFolderPath=nil then
      begin
        FreeLibrary(CFGDLLHandle);
        CFGDllHandle:=0;
      end;
    end;
  end;
  If (@ShGetFolderPath=Nil) then
    Raise Exception.Create('Could not determine SHGetFolderPath Function');
end;

Function GetWindowsSpecialDir(ID :  Integer) : String;

Var
  APath : Array[0..MAX_PATH] of char;

begin
  Result:='';
  if (CFGDLLHandle=0) then
    InitDLL;
  If (SHGetFolderPath<>Nil) then
    begin
    if SHGetFolderPath(0,ID or CSIDL_FLAG_CREATE,0,0,@APATH[0])=S_OK then
      Result:=IncludeTrailingPathDelimiter(StrPas(@APath[0]));
    end;
end;

Initialization
Finalization
  if CFGDLLHandle<>0 then
   FreeLibrary(CFGDllHandle);
end.

