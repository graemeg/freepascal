{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    An easy way to use asl.library, no need to open asl.library,
    unit asl will open it for you.
    A lot of overlay functions here.:)

    One remark, be aware of that GetMultiFiles use linklist for the
    linked list of files, you can't use your own list with ordinary
    nodes.
    26 Oct 1998

    Removed amigaoverlays, use smartlink instead.
    05 Nov 2002.

    Added the define use_amiga_smartlink.
    13 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit easyasl;


interface

uses exec, asl, utility, amigautils,strings, workbench, linklist;


TYPE

    pFPCFontInfo = ^tFPCFontInfo;

    tFPCFontInfo = RECORD
    nfi_Name       : String[40];
    nfi_Size       : Word;
    nfi_Style      : Byte;
    nfi_Flags      : Byte;
    nfi_FrontPen   : Byte;
    nfi_BackPen    : Byte;
    nfi_DrawMode   : Byte;
    END;


FUNCTION GetFileAsl(title : PChar; VAR path, fname : PChar; thepatt : PChar;win : Pointer): Boolean;
FUNCTION GetFontAsl(title : PChar;VAR finfo : tFPCFontInfo; win : Pointer): Boolean;
FUNCTION GetMultiAsl(title : PChar; VAR path : PChar;  VAR Thelist : pList; thepatt : PChar;win : Pointer): Boolean;
FUNCTION GetPathAsl(title : PChar; VAR path : PChar; win : Pointer): Boolean;
FUNCTION SaveFileAsl(title : PChar; VAR path, fname : PChar; win : Pointer): Boolean;


FUNCTION GetFileAsl(title : PChar; VAR path, fname : PChar; thepatt : String;win : Pointer): Boolean;
FUNCTION GetFileAsl(title : String; VAR path, fname : PChar; thepatt : PChar;win : Pointer): Boolean;
FUNCTION GetFileAsl(title : String; VAR path, fname : PChar; thepatt : String;win : Pointer): Boolean;
FUNCTION GetFontAsl(title : String;VAR finfo : tFPCFontInfo; win : Pointer): Boolean;
FUNCTION GetMultiAsl(title : PChar; VAR path : PChar;  VAR Thelist : pList; thepatt : String;win : Pointer): Boolean;
FUNCTION GetMultiAsl(title : String; VAR path : PChar;  VAR Thelist : pList; thepatt : PChar;win : Pointer): Boolean;
FUNCTION GetMultiAsl(title : String; VAR path : PChar;  VAR Thelist : pList; thepatt : String;win : Pointer): Boolean;
FUNCTION GetPathAsl(title : String; VAR path : PChar; win : Pointer): Boolean;
FUNCTION SaveFileAsl(title : String; VAR path, fname : PChar; win : Pointer): Boolean;


implementation


 uses pastoc;



FUNCTION GetFileAsl(title : PChar; VAR path, fname : PChar; thepatt : PChar;win : Pointer): Boolean;

VAR
    fr : pFileRequester;
    result : Boolean;
    mytags : ARRAY[0..7] OF tTagItem;

BEGIN
    result := false;
    IF strlen(fname) >0 THEN begin
        mytags[0].ti_Tag  := ASLFR_InitialFile;
        mytags[0].ti_Data := Longint(fname);
    END ELSE begin
        mytags[0].ti_Tag := TAG_IGNORE;
    END;
    IF (strlen(path) > 0) and (FileType(path) = 2) THEN begin
        mytags[1].ti_Tag  := ASLFR_InitialDrawer;
        mytags[1].ti_Data := Longint(path);
    END ELSE begin
        mytags[1].ti_Tag  := ASLFR_InitialDrawer;
        mytags[1].ti_Data := Longint(pas2c('Sys:'));
    END;
    IF win <> nil THEN begin
        mytags[2].ti_Tag  := ASLFR_Window;
        mytags[2].ti_Data := Longint(win);
    END ELSE begin
        mytags[2].ti_Tag  := TAG_IGNORE;
    END;
    IF win <> nil THEN begin
        mytags[3].ti_Tag  := ASLFR_SleepWindow;
        mytags[3].ti_Data := Longint(Byte(true));
    END ELSE begin
        mytags[3].ti_Tag  := TAG_IGNORE;
    END;
    IF title <> nil THEN begin
        mytags[4].ti_Tag  := ASLFR_TitleText;
        mytags[4].ti_Data := Longint(title);
    END ELSE begin
        mytags[4].ti_Tag  := TAG_IGNORE;
    END;
    IF thepatt <> nil THEN begin
        mytags[5].ti_Tag  := ASLFR_InitialPattern;
        mytags[5].ti_Data := Longint(thepatt);
    END ELSE begin
        mytags[5].ti_Tag  := TAG_IGNORE;
    END;
    IF thepatt <> nil THEN begin
        mytags[6].ti_Tag  := ASLFR_DoPatterns;
        mytags[6].ti_Data := Longint(Byte(true));
    END ELSE begin
        mytags[6].ti_Tag  := TAG_IGNORE;
    END;
    mytags[7].ti_Tag  := TAG_DONE;

    fr := AllocAslRequest(ASL_FileRequest,@mytags);
    IF fr <> NIL THEN BEGIN
       IF AslRequest(fr,NIL) THEN BEGIN
          IF (strlen(fr^.rf_Dir) >0) and (strlen(fr^.rf_File) > 0) THEN begin
             strcopy(path,fr^.rf_Dir);
             strcopy(fname,fr^.rf_File);
             result := true;
          END ELSE begin
             result := false;
          end;
       END ELSE BEGIN
          result := false;
       END;
       FreeAslRequest(fr);
    END ELSE BEGIN
       result := false;
    END;
    GetFileAsl := result;
END;

FUNCTION GetFontAsl(title : PChar;VAR finfo : tFPCFontInfo; win : Pointer): Boolean;

VAR
    fr : pFontRequester;
    result : boolean;
    mytags : ARRAY[0..14] OF tTagItem;

BEGIN

    result := false;
    IF win <> nil THEN begin
        mytags[0].ti_Tag  := ASLFR_Window;
        mytags[0].ti_Data := Longint(win);
    END ELSE begin
        mytags[0].ti_Tag  := TAG_IGNORE;
    END;
    IF win <> nil THEN begin
        mytags[1].ti_Tag  := ASLFR_SleepWindow;
        mytags[1].ti_Data := Longint(Byte(true));
    END ELSE begin
        mytags[1].ti_Tag  := TAG_IGNORE;
    END;
    IF title <> nil THEN begin
        mytags[2].ti_Tag  := ASLFR_TitleText;
        mytags[2].ti_Data := Longint(title);
    END ELSE begin
        mytags[2].ti_Tag  := TAG_IGNORE;
    END;
    IF length(finfo.nfi_Name) > 0 THEN BEGIN
        mytags[3].ti_Tag  := ASLFO_InitialName;
        mytags[3].ti_Data := Longint(pas2c(finfo.nfi_Name));
    END ELSE BEGIN
        finfo.nfi_Name := 'topaz.font';
        mytags[3].ti_Tag  := ASLFO_InitialName;
        mytags[3].ti_Data := Longint(pas2c('topaz.font'));
    END;
    IF finfo.nfi_Size <= 4 THEN BEGIN
        mytags[4].ti_Tag  := ASLFO_InitialSize;
        mytags[4].ti_Data := 9;
    END ELSE BEGIN
        mytags[4].ti_Tag  := ASLFO_InitialSize;
        mytags[4].ti_Data := Longint(finfo.nfi_Size);
    END;
    IF finfo.nfi_Style >= 0 THEN BEGIN
        mytags[5].ti_Tag  := ASLFO_InitialStyle;
        mytags[5].ti_Data := Longint(finfo.nfi_Style);
    END ELSE BEGIN
        mytags[5].ti_Tag  := TAG_IGNORE;
    END;
    IF finfo.nfi_Flags >= 0 THEN BEGIN
        mytags[6].ti_Tag  := ASLFO_InitialFlags;
        mytags[6].ti_Data := Longint(finfo.nfi_Flags);
    END ELSE BEGIN
        mytags[6].ti_Tag := TAG_IGNORE;
    END;
    IF finfo.nfi_BackPen >=0 THEN BEGIN
        mytags[7].ti_Tag := ASLFO_InitialBackPen;
        mytags[7].ti_Data := Longint(finfo.nfi_BackPen);
    END ELSE BEGIN
        mytags[7].ti_Tag := ASLFO_InitialBackPen;
        mytags[7].ti_Data := 0;
    END;
    IF (finfo.nfi_FrontPen = 0) and (finfo.nfi_BackPen = 0) THEN BEGIN
        mytags[8].ti_Tag := ASLFO_InitialFrontPen;
        mytags[8].ti_Data := 1;
    END ELSE BEGIN
        mytags[8].ti_Tag := ASLFO_InitialFrontPen;
        mytags[8].ti_Data := Longint(finfo.nfi_FrontPen);
    END;
    IF finfo.nfi_DrawMode >= 0 THEN BEGIN
        mytags[9].ti_Tag := ASLFO_InitialDrawMode;
        mytags[9].ti_Data := Longint(finfo.nfi_DrawMode);
    END ELSE BEGIN
        mytags[9].ti_Tag := ASLFO_InitialDrawMode;
        mytags[9].ti_Data := 0;
    END;
    mytags[10].ti_Tag := ASLFO_DoFrontPen;
    mytags[10].ti_Data := Longint(Byte(true));
    mytags[11].ti_Tag := ASLFO_DoBackPen;
    mytags[11].ti_Data := Longint(Byte(true));
    mytags[12].ti_Tag := ASLFO_DoStyle;
    mytags[12].ti_Data := Longint(Byte(true));
    mytags[13].ti_Tag := ASLFO_DoDrawMode;
    mytags[13].ti_Data := Longint(Byte(true));
    mytags[14].ti_Tag := TAG_DONE;

    fr := AllocAslRequest(ASL_FontRequest,@mytags);
    IF fr <> NIL THEN BEGIN
         IF AslRequest(fr,NIL) THEN BEGIN
              WITH finfo DO BEGIN
                  nfi_Name := strpas(fr^.fo_Attr.ta_Name);
                  nfi_Size       := fr^.fo_Attr.ta_YSize;
                  nfi_Style      := fr^.fo_Attr.ta_Style;
                  nfi_Flags      := fr^.fo_Attr.ta_Flags;
                  nfi_FrontPen   := fr^.fo_FrontPen;
                  nfi_BackPen    := fr^.fo_BackPen;
                  nfi_DrawMode   := fr^.fo_DrawMode;
             END;
             result := true;
         END ELSE BEGIN
             result := false;
         END;
         FreeAslRequest(fr);
    END ELSE BEGIN
        result := false;
    END;
    GetFontAsl := result;
END;

FUNCTION GetMultiAsl(title : PChar; VAR path : PChar;  VAR Thelist : pList; thepatt : PChar;win : Pointer): Boolean;

VAR
    fr : pFileRequester;
    result : Boolean;
    mytags : ARRAY[0..7] OF tTagItem;
    index : Longint;
    tempnode : pFPCNode;

BEGIN
        IF (strlen(path) > 0) and (FileType(path) = 2) THEN begin
            mytags[0].ti_Tag  := ASLFR_InitialDrawer;
            mytags[0].ti_Data := Longint(path);
        END ELSE begin
            mytags[0].ti_Tag  := ASLFR_InitialDrawer;
            mytags[0].ti_Data := Longint(pas2c('Sys:'));
        END;
        IF win <> nil THEN begin
            mytags[1].ti_Tag  := ASLFR_Window;
            mytags[1].ti_Data := Longint(win);
        END ELSE begin
            mytags[1].ti_Tag  := TAG_IGNORE;
        END;
        IF win <> nil THEN begin
            mytags[2].ti_Tag  := ASLFR_SleepWindow;
            mytags[2].ti_Data := Longint(Byte(true));
        END ELSE begin
            mytags[2].ti_Tag  := TAG_IGNORE;
        END;
        IF title <> nil THEN begin
            mytags[3].ti_Tag  := ASLFR_TitleText;
            mytags[3].ti_Data := Longint(title);
        END ELSE begin
            mytags[3].ti_Tag  := TAG_IGNORE;
        END;
        IF thepatt <> nil THEN begin
            mytags[4].ti_Tag  := ASLFR_InitialPattern;
            mytags[4].ti_Data := Longint(thepatt);
        END ELSE begin
            mytags[4].ti_Tag  := TAG_IGNORE;
        END;
        IF thepatt <> nil THEN begin
            mytags[5].ti_Tag  := ASLFR_DoPatterns;
            mytags[5].ti_Data := Longint(Byte(true));
        END ELSE begin
            mytags[5].ti_Tag  := TAG_IGNORE;
        END;
        mytags[6].ti_Tag := ASLFR_DoMultiSelect;
        mytags[6].ti_Data := Longint(Byte(true));
        mytags[7].ti_Tag  := TAG_DONE;

        fr := AllocAslRequest(ASL_FileRequest,@mytags);
        IF fr <> NIL THEN BEGIN
             IF AslRequest(fr,NIL) THEN BEGIN
                 IF (strlen(fr^.rf_Dir) >0) THEN begin
                    strcopy(path,fr^.rf_Dir);
                    result := true;
                    FOR index := 1 to (fr^.rf_NumArgs) do begin
                        tempnode := AddNewnode(TheList,fr^.rf_ArgList^[index].wa_Name);
                    end;
                 END ELSE begin
                    result := false;
                 end;
             END ELSE BEGIN
                 result := false;
             END;
             FreeAslRequest(fr);
        END ELSE BEGIN
            result := false;
        END;
        GetMultiAsl := result;
END;

FUNCTION GetPathAsl(title : PChar; VAR path : PChar; win : Pointer): Boolean;

VAR
    fr : pFileRequester;
    result : Boolean;
    mytags : ARRAY[0..5] OF tTagItem;

BEGIN
    result := false;
    IF (strlen(path) > 0) and (FileType(path) = 2) THEN begin
        mytags[0].ti_Tag  := ASLFR_InitialDrawer;
        mytags[0].ti_Data := Longint(path);
    END ELSE begin
        mytags[0].ti_Tag  := ASLFR_InitialDrawer;
        mytags[0].ti_Data := Longint(pas2c('Sys:'));
    END;
    IF win <> nil THEN begin
        mytags[1].ti_Tag  := ASLFR_Window;
        mytags[1].ti_Data := Longint(win);
    END ELSE begin
        mytags[1].ti_Tag  := TAG_IGNORE;
    END;
    IF win <> nil THEN begin
        mytags[2].ti_Tag  := ASLFR_SleepWindow;
        mytags[2].ti_Data := Longint(Byte(true));
    END ELSE begin
        mytags[2].ti_Tag  := TAG_IGNORE;
    END;
    IF title <> nil THEN begin
        mytags[3].ti_Tag  := ASLFR_TitleText;
        mytags[3].ti_Data := Longint(title);
    END ELSE begin
        mytags[3].ti_Tag  := TAG_IGNORE;
    END;
    mytags[4].ti_Tag  := ASLFR_DrawersOnly;
    mytags[4].ti_Data := Longint(Byte(true));
    mytags[5].ti_Tag  := TAG_DONE;

    fr := AllocAslRequest(ASL_FileRequest,@mytags);
    IF fr <> NIL THEN BEGIN
         IF AslRequest(fr,NIL) THEN BEGIN
             IF (strlen(fr^.rf_Dir) >0) THEN begin
                strcopy(path,fr^.rf_Dir);
                result := true;
             END ELSE begin
                result := false;
             end;
         END ELSE BEGIN
             result := false;
         END;
         FreeAslRequest(fr);
    END ELSE BEGIN
        result := false;
    END;
    GetPathAsl := result;
END;

FUNCTION SaveFileAsl(title : PChar; VAR path, fname : PChar; win : Pointer): Boolean;

VAR
    fr : pFileRequester;
    result : Boolean;
    mytags : ARRAY[0..6] OF tTagItem;

BEGIN
    result := false;
    IF strlen(fname) >0 THEN begin
        mytags[0].ti_Tag  := ASLFR_InitialFile;
        mytags[0].ti_Data := Longint(fname);
    END ELSE begin
        mytags[0].ti_Tag := TAG_IGNORE;
    END;
    IF (strlen(path) > 0) and (FileType(path) = 2) THEN begin
        mytags[1].ti_Tag  := ASLFR_InitialDrawer;
        mytags[1].ti_Data := Longint(path);
    END ELSE begin
        mytags[1].ti_Tag  := ASLFR_InitialDrawer;
        mytags[1].ti_Data := Longint(pas2c('Sys:'));
    END;
    IF win <> nil THEN begin
        mytags[2].ti_Tag  := ASLFR_Window;
        mytags[2].ti_Data := Longint(win);
    END ELSE begin
        mytags[2].ti_Tag  := TAG_IGNORE;
    END;
    IF win <> nil THEN begin
        mytags[3].ti_Tag  := ASLFR_SleepWindow;
        mytags[3].ti_Data := Longint(Byte(true));
    END ELSE begin
        mytags[3].ti_Tag  := TAG_IGNORE;
    END;
    IF title <> nil THEN begin
        mytags[4].ti_Tag  := ASLFR_TitleText;
        mytags[4].ti_Data := Longint(title);
    END ELSE begin
        mytags[4].ti_Tag  := TAG_IGNORE;
    END;
    mytags[5].ti_Tag := ASLFR_DoSaveMode;
    mytags[5].ti_Data := Longint(Byte(true));
    mytags[6].ti_Tag  := TAG_DONE;

    fr := AllocAslRequest(ASL_FileRequest,@mytags);
    IF fr <> NIL THEN BEGIN
         IF AslRequest(fr,NIL) THEN BEGIN
             IF (strlen(fr^.rf_Dir) >0) and (strlen(fr^.rf_File) > 0) THEN begin
                strcopy(path,fr^.rf_Dir);
                strcopy(fname,fr^.rf_File);
                result := true;
             END ELSE begin
                result := false;
             end;
         END ELSE BEGIN
             result := false;
         END;
         FreeAslRequest(fr);
    END ELSE BEGIN
        result := false;
    END;
    SaveFileAsl := result;
END;


FUNCTION GetFileAsl(title : String; VAR path, fname : PChar; thepatt : PChar;win : Pointer): Boolean;
begin
    GetFileAsl := GetFileAsl(pas2c(title),path,fname,thepatt,win);
end;

FUNCTION GetFileAsl(title : String; VAR path, fname : PChar; thepatt : String;win : Pointer): Boolean;
begin
    GetFileAsl := GetFileAsl(pas2c(title),path,fname,pas2c(thepatt),win);
end;

FUNCTION GetFileAsl(title : PChar; VAR path, fname : PChar; thepatt : String;win : Pointer): Boolean;
begin
    GetFileAsl := GetFileAsl(title,path,fname,pas2c(thepatt),win);
end;

FUNCTION GetFontAsl(title : String;VAR finfo : tFPCFontInfo; win : Pointer): Boolean;
begin
    GetFontAsl := GetFontAsl(pas2c(title),finfo,win);
end;

FUNCTION GetMultiAsl(title : String; VAR path : PChar;  VAR Thelist : pList; thepatt : PChar;win : Pointer): Boolean;
begin
    GetMultiAsl := GetMultiAsl(pas2c(title),path,TheList,thepatt,win);
end;

FUNCTION GetMultiAsl(title : String; VAR path : PChar;  VAR Thelist : pList; thepatt : String;win : Pointer): Boolean;
begin
    GetMultiAsl := GetMultiAsl(pas2c(title),path,TheList,pas2c(thepatt),win);
end;

FUNCTION GetMultiAsl(title : PChar; VAR path : PChar;  VAR Thelist : pList; thepatt : String;win : Pointer): Boolean;
begin
    GetMultiAsl := GetMultiAsl(title,path,TheList,pas2c(thepatt),win);
end;

FUNCTION GetPathAsl(title : String; VAR path : PChar; win : Pointer): Boolean;
begin
    GetPathAsl := GetPathAsl(pas2c(title),path,win);
end;

FUNCTION SaveFileAsl(title : String; VAR path, fname : PChar; win : Pointer): Boolean;
begin
    SaveFileAsl := SaveFileAsl(pas2c(title),path,fname,win);
end;


end.
