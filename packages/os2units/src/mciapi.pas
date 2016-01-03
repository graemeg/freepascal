{
    Copyright (c) 1992, 1993 by International Business Machines Corporation
    Copyright (c) 2002 by Andry Svirgunov (cool2@ngs.ru)
    Copyright (c) 2002-2003 by Yuri Prokushev (prokushev@freemail.ru)

    High-Level MCI Interfaces of OS/2 Multimedia subsystem

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(High-Level MCI Interfaces of OS/2 Multimedia subsystem)
@author(Andry Svirgunov (cool2@ngs.ru))
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(01 Oct 2002)
@lastmod(19 Jan 2003)
This is the High-Level Macro Service API Routines of OS/2 Multimedia subsystem.
All functions are from MCIAPI dll (which also contains REXX functions.
See "Multimedia with REXX" for more information.).
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
Unit mciapi;

{$MODE ObjFPC}

Interface

// Flags for mciPlayFile
Const
  // digital and overlay
  MCI_OWNERISPARENT   = $0001;
  // stop playing whatever is playing
  MCI_STOPACTIVE      = $0002;
  // play and return immediately
  MCI_ASYNCRENDEZVOUS = $0004;
  // wait til prev is finished then play
  MCI_RENDEZVOUS      = $0008;
  // no syncup will be done
  MCI_ASYNC           = $0010;

{
This function plays a multimedia data file (video, audio) using MCI commands.
hwndOwner is handle of owner window. If hwndOwner equial to zero used active
          window.
pszFile is pointer to asciiz name of data file. For compound files can be
        used 'filename+element'.
ulFlags is MCI_* flags.
pszTitle is title of generated window (e.g. for video). Ignored if not generated.
hwndViewport is handle of window for displaying video. If none, then default
             window displayed.
}
Function mciPlayFile(hwndOwner: Cardinal;               // Ownerwindow
                     pszFile: PChar;                    // File
                     ulFlags: Cardinal;                 // Flags
                     pszTitle: PChar;                   // Title
                     hwndViewport: Cardinal):           // Viewport Window
                       Cardinal; cdecl;

Function mciPlayResource(hwndOwner: Cardinal;           // Owner Window
                         hmod: LongInt;                 // Module
                         resType: LongInt;              // Resource Type
                         resID: LongInt;                // Resource ID
                         ulFlags: Cardinal;             // Flags
                         pszTitle: PChar;               // Title
                         hwndViewport: Cardinal):       // Viewport Window
                           Cardinal; cdecl;

Function mciRecordAudioFile(hwndOwner: Cardinal;
                            pszFile,
                            pszTitle: PChar;
                            ulFlags: Cardinal):
                              Cardinal; cdecl;

// Audio Macro Service Constants and Routines

Const
  MMIO_FE_FINDFIRST        = 1;
  MMIO_FE_FINDNEXT         = 2;
  MMIO_FE_FINDEND          = 3;
  MMIO_FE_FINDELEMENT      = 4;

  MMIO_RE_COMPACT          = 1;

Function mmioRemoveElement(pszFileElement: pChar;
                           ulFlag: LongInt):
                             Cardinal; cdecl;

Function mmioFindElement(ulCode: LongInt;              // Find Code
                         pszElement: PChar;            // Element
                         ulElementLen: LongInt;        // Element Buffer Length
                         pszFile: PChar;
                         ulReserved: LongInt):         // Compound File
                           Cardinal; cdecl;



Implementation

Function mciPlayFile(hwndOwner: Cardinal; pszFile: PChar; ulFlags: Cardinal; pszTitle: PChar; hwndViewport: Cardinal): Cardinal; cdecl;
    external 'MCIAPI' index 10;
Function mciPlayResource(hwndOwner: Cardinal; hmod: LongInt; resType: LongInt; resID: LongInt; ulFlags: Cardinal; pszTitle: PChar; hwndViewport: Cardinal): Cardinal;  cdecl;
    external 'MCIAPI' index 11;
Function mciRecordAudioFile(hwndOwner: Cardinal; pszFile, pszTitle: PChar; ulFlags: Cardinal): Cardinal; cdecl;
    external 'MCIAPI' index 12;
Function mmioRemoveElement(pszFileElement: PChar; ulFlag: LongInt): Cardinal; cdecl;
    external 'MCIAPI' index 16;
Function mmioFindElement(ulCode: LongInt; pszElement: PChar; ulElementLen: LongInt; pszFile: PChar; ulReserved: LongInt): Cardinal; cdecl;
    external 'MCIAPI' index 18;

end.
