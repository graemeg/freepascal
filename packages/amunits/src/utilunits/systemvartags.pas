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

{$mode objfpc}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit systemvartags;

interface

uses exec,amigados, amigaguide, asl, bullet, intuition, datatypes ,
     gadtools, graphics, locale, lowlevel, realtime,
     workbench, utility, tagsarray;

{    As of today boolean and char doesn't function in
     array of const. Use ltrue and lfalse instead. You
     can just cast a char.

     Added the define use_amiga_smartlink.
     13 Jan 2003.

     Changed integer > smallint.
     Moved ltrue and lfalse to exec.
     10 Feb 2003.

     nils.sjoholm@mailbox.swipnet.se
}


{
     This is functions and procedures with array of const.
     For use with fpc 1.0 and above.
}

{ functions from amigados. }
FUNCTION AllocDosObjectTags(type_ : ULONG; Const argv : Array of Const) : POINTER;
FUNCTION CreateNewProcTags(Const argv : Array of Const) : pProcess;
FUNCTION NewLoadSegTags(file_ : pCHAR; Const argv : Array of Const) : LONGINT;
FUNCTION SystemTags(command : pCHAR; Const argv : Array of Const) : LONGINT;
   {  This one as well, an overlay function }
FUNCTION SystemTags(command : string; Const argv : Array of Const) : LONGINT;

{ functions from amigaguide. }
FUNCTION AddAmigaGuideHost(h : pHook; name : pCHAR; Const argv : Array Of Const) : POINTER;
FUNCTION OpenAmigaGuide(nag : pNewAmigaGuide; Const argv : Array Of Const) : POINTER;
FUNCTION OpenAmigaGuideAsync(nag : pNewAmigaGuide; Const argv : Array Of Const) : POINTER;
FUNCTION RemoveAmigaGuideHost(hh : POINTER; Const argv : Array Of Const) : LONGINT;
FUNCTION SendAmigaGuideCmd(cl : POINTER; cmd : pCHAR; Const argv : Array Of Const) : LONGINT;
FUNCTION SendAmigaGuideContext(cl : POINTER; Const argv : Array Of Const) : LONGINT;
FUNCTION SetAmigaGuideAttrs(cl : POINTER; Const argv : Array Of Const) : LONGINT;
FUNCTION SetAmigaGuideContext(cl : POINTER; id : ULONG; Const argv : Array Of Const) : LONGINT;

{ functions from asl. }
FUNCTION AllocAslRequestTags(reqType : ULONG; Const argv : Array Of Const) : POINTER;
FUNCTION AslRequestTags(requester : POINTER; Const argv : Array Of Const) : BOOLEAN;

{ functions from bullet }
FUNCTION ObtainInfo(glyphEngine : pGlyphEngine; Const argv : Array Of Const) : ULONG;
FUNCTION ReleaseInfo(glyphEngine : pGlyphEngine; Const argv : Array Of Const) : ULONG;
FUNCTION SetInfo(glyphEngine : pGlyphEngine; Const argv : Array Of Const) : ULONG;

{ functions from datatypes }
FUNCTION GetDTAttrs(o : pObject_; Const argv : Array Of Const) : ULONG;
FUNCTION NewDTObject(name : POINTER; Const argv : Array Of Const): POINTER;
FUNCTION ObtainDataType(typ : ULONG; handle : POINTER; Const argv : Array Of Const) : pDataType;
PROCEDURE RefreshDTObject(o : pObject_; win : pWindow; req : pRequester; Const argv : Array Of Const);
FUNCTION SetDTAttrs(o : pObject_; win : pWindow; req : pRequester; Const argv : Array Of Const) : ULONG;

{ functions from gadtools }
FUNCTION CreateGadget(kind : ULONG; gad : pGadget; ng : pNewGadget; Const argv : Array Of Const) : pGadget;
FUNCTION CreateMenus(newmenu : pNewMenu; Const argv : Array Of Const) : pMenu;
PROCEDURE DrawBevelBox(rport : pRastPort; left : LONGINT; top : LONGINT; width : LONGINT; height : LONGINT; Const argv : Array Of Const);
FUNCTION GetVisualInfo(screen : pScreen; Const argv : Array Of Const) : POINTER;
FUNCTION GT_GetGadgetAttrs(gad : pGadget; win : pWindow; req : pRequester; Const argv : Array Of Const) : LONGINT;
PROCEDURE GT_SetGadgetAttrs(gad : pGadget; win : pWindow; req : pRequester; Const argv : Array Of Const);
FUNCTION LayoutMenuItems(firstitem : pMenuItem; vi : POINTER; Const argv : Array Of Const) : BOOLEAN;
FUNCTION LayoutMenus(firstmenu : pMenu; vi : POINTER; Const argv : Array Of Const) : BOOLEAN;

{ functions from graphics }
FUNCTION AllocSpriteData(bm : pBitMap; Const argv : Array Of Const) : pExtSprite;
FUNCTION BestModeID(Const argv : Array Of Const) : ULONG;
FUNCTION ChangeExtSprite(vp : pViewPort; oldsprite : pExtSprite; newsprite : pExtSprite; Const argv : Array Of Const) : LONGINT;
FUNCTION ExtendFontTags(font : pTextFont; Const argv : Array Of Const) : ULONG;
FUNCTION GetExtSprite(ss : pExtSprite; Const argv : Array Of Const) : LONGINT;
PROCEDURE GetRPAttrs(rp : pRastPort; Const argv : Array Of Const);
FUNCTION ObtainBestPen(cm : pColorMap; r : ULONG; g : ULONG; b : ULONG; Const argv : Array Of Const) : LONGINT;
PROCEDURE SetRPAttrs(rp : pRastPort; Const argv : Array Of Const);
FUNCTION VideoControlTags(colorMap : pColorMap; Const argv : Array Of Const) : BOOLEAN;
FUNCTION WeighTAMatchTags(reqTextAttr : pTextAttr; targetTextAttr : pTextAttr; Const argv : Array Of Const) : smallint;

{ functions from intuition. }
FUNCTION OpenScreenTags(newScreen : pNewScreen; tagList : array of const) : pScreen;
FUNCTION OpenWindowTags(newWindow : pNewWindow; tagList : array of const) : pWindow;
FUNCTION NewObject(classPtr : pIClass; classID : pCHAR; Const argv : Array Of Const) : POINTER;
FUNCTION SetGadgetAttrs(gadget : pGadget; window : pWindow; requester : pRequester; Const argv : Array Of Const) : ULONG;
FUNCTION NewObject(classPtr : pIClass; classID : string; Const argv : array of const ) : POINTER;

{ from locale }
FUNCTION OpenCatalog(locale : pLocale; name : pCHAR; Const argv : Array Of Const) : pCatalog;

{ functions from lowlevel }
FUNCTION SetJoyPortAttrs(portNumber : ULONG; Const argv : Array Of Const) : BOOLEAN;
FUNCTION SystemControl(Const argv : Array Of Const) : ULONG;

{ functions from realtime }
FUNCTION CreatePlayer(Const argv : Array Of Const) : pPlayer;
FUNCTION GetPlayerAttrs(player : pPlayer; Const argv : Array Of Const) : ULONG;
FUNCTION SetPlayerAttrs(player : pPlayer; Const argv : Array Of Const) : BOOLEAN;

{ from utility }
function AllocNamedObject(name : STRPTR; Const argv : Array Of Const) : pNamedObject;

{ functions from workbench }
FUNCTION AddAppMenuItem(id : ULONG; userdata : ULONG; text_ : pCHAR; msgport : pMsgPort; Const argv : Array Of Const) : pAppMenuItem;
FUNCTION AddAppWindow(id : ULONG; userdata : ULONG; window : pWindow; msgport : pMsgPort; Const argv : Array Of Const) : pAppWindow;

implementation

uses pastoc;

FUNCTION AllocDosObjectTags(type_ : ULONG; Const argv : Array of Const) : POINTER;
begin
     AllocDosObjectTags := AllocDosObjectTagList(type_, readintags(argv));
end;

FUNCTION CreateNewProcTags(Const argv : Array of Const) : pProcess;
begin
     CreateNewProcTags := CreateNewProcTagList(readintags(argv));
end;

FUNCTION NewLoadSegTags(file_ : pCHAR; Const argv : Array of Const) : LONGINT;
begin
     NewLoadSegTags := NewLoadSegTagList(file_,readintags(argv));
end;

FUNCTION SystemTags(command : pCHAR; Const argv : Array of Const) : LONGINT;
begin
     SystemTags := SystemTagList(command,readintags(argv));
end;

FUNCTION SystemTags(command : string; Const argv : Array of Const) : LONGINT;
begin
     SystemTags := SystemTagList(command,readintags(argv));
end;

FUNCTION OpenScreenTags(newScreen : pNewScreen; tagList : array of const) : pScreen;
begin
    OpenScreenTags := OpenScreenTagList(newScreen, readintags(tagList));
end;

FUNCTION OpenWindowTags(newWindow : pNewWindow; tagList : array of const) : pWindow;
begin
    OpenWindowTags := OpenWindowTagList(newWindow, readintags(tagList));
end;

FUNCTION NewObject(classPtr : pIClass; classID : pCHAR; Const argv : Array Of Const) : POINTER;
begin
    NewObject := NewObjectA(classPtr,classID, readintags(argv));
end;

FUNCTION NewObject(classPtr : pIClass; classID : string; Const argv : array of const ) : POINTER;
begin
      NewObject := NewObjectA(classPtr,pas2c(classID),readintags(argv));
end;

FUNCTION SetGadgetAttrs(gadget : pGadget; window : pWindow; requester : pRequester; Const argv : Array Of Const) : ULONG;
begin
    SetGadgetAttrs := SetGadgetAttrsA(gadget,window,requester,readintags(argv));
end;

FUNCTION AddAmigaGuideHost(h : pHook; name : pCHAR; Const argv : Array Of Const) : POINTER;
begin
    AddAmigaGuideHost := AddAmigaGuideHostA(h,name,readintags(argv));
end;

FUNCTION OpenAmigaGuide(nag : pNewAmigaGuide; Const argv : Array Of Const) : POINTER;
begin
    OpenAmigaGuide := OpenAmigaGuideA(nag,readintags(argv));
end;

FUNCTION OpenAmigaGuideAsync(nag : pNewAmigaGuide; Const argv : Array Of Const) : POINTER;
begin
    OpenAmigaGuideAsync := OpenAmigaGuideAsyncA(nag,readintags(argv));
end;

FUNCTION RemoveAmigaGuideHost(hh : POINTER; Const argv : Array Of Const) : LONGINT;
begin
    RemoveAmigaGuideHost := RemoveAmigaGuideHostA(hh,readintags(argv));
end;

FUNCTION SendAmigaGuideCmd(cl : POINTER; cmd : pCHAR; Const argv : Array Of Const) : LONGINT;
begin
    SendAmigaGuideCmd := SendAmigaGuideCmdA(cl,cmd,readintags(argv));
end;

FUNCTION SendAmigaGuideContext(cl : POINTER; Const argv : Array Of Const) : LONGINT;
begin
    SendAmigaGuideContext := SendAmigaGuideContextA(cl,readintags(argv));
end;

FUNCTION SetAmigaGuideAttrs(cl : POINTER; Const argv : Array Of Const) : LONGINT;
begin
    SetAmigaGuideAttrs := SetAmigaGuideAttrsA(cl,readintags(argv));
end;

FUNCTION SetAmigaGuideContext(cl : POINTER; id : ULONG; Const argv : Array Of Const) : LONGINT;
begin
    SetAmigaGuideContext := SetAmigaGuideContextA(cl,id,readintags(argv));
end;

FUNCTION AllocAslRequestTags(reqType : ULONG; Const argv : Array Of Const) : POINTER;
begin
    AllocAslRequestTags := AllocAslRequest(reqType,readintags(argv));
end;

FUNCTION AslRequestTags(requester : POINTER; Const argv : Array Of Const) : BOOLEAN;
begin
    AslRequestTags := AslRequest(requester,readintags(argv));
end;

FUNCTION ObtainInfo(glyphEngine : pGlyphEngine; Const argv : Array Of Const) : ULONG;
begin
    ObtainInfo := ObtainInfoA(glyphEngine,readintags(argv));
end;

FUNCTION ReleaseInfo(glyphEngine : pGlyphEngine; Const argv : Array Of Const) : ULONG;
begin
    ReleaseInfo := releaseInfoA(glyphEngine,readintags(argv));
end;

FUNCTION SetInfo(glyphEngine : pGlyphEngine; Const argv : Array Of Const) : ULONG;
begin
    SetInfo := SetInfoA(glyphEngine,readintags(argv));
end;

FUNCTION GetDTAttrs(o : pObject_; Const argv : Array Of Const) : ULONG;
begin
    GetDTAttrs := GetDTAttrsA(o,readintags(argv));
end;

FUNCTION NewDTObject(name : POINTER; Const argv : Array Of Const): POINTER;
begin
    NewDTObject := NewDTObjectA(name,readintags(argv));
end;

FUNCTION ObtainDataType(typ : ULONG; handle : POINTER; Const argv : Array Of Const) : pDataType;
begin
    ObtainDataType := ObtainDataTypeA(typ,handle,readintags(argv));
end;
PROCEDURE RefreshDTObject(o : pObject_; win : pWindow; req : pRequester; Const argv : Array Of Const);
begin
    RefreshDTObjectA(o,win,req,readintags(argv));
end;

FUNCTION SetDTAttrs(o : pObject_; win : pWindow; req : pRequester; Const argv : Array Of Const) : ULONG;
begin
    SetDTAttrs := SetDTAttrsA(o,win,req,readintags(argv));
end;

FUNCTION CreateGadget(kind : ULONG; gad : pGadget; ng : pNewGadget; Const argv : Array Of Const) : pGadget;
begin
    CreateGadget := CreateGadgetA(kind,gad,ng,readintags(argv));
end;

FUNCTION CreateMenus(newmenu : pNewMenu; Const argv : Array Of Const) : pMenu;
begin
    CreateMenus := CreateMenusA(newmenu,readintags(argv));
end;

PROCEDURE DrawBevelBox(rport : pRastPort; left : LONGINT; top : LONGINT; width : LONGINT; height : LONGINT; Const argv : Array Of Const);
begin
    DrawBevelBoxA(rport,left,top,width,height,readintags(argv));
end;

FUNCTION GetVisualInfo(screen : pScreen; Const argv : Array Of Const) : POINTER;
begin
    GetVisualInfo := GetVisualInfoA(screen,readintags(argv));
end;

FUNCTION GT_GetGadgetAttrs(gad : pGadget; win : pWindow; req : pRequester; Const argv : Array Of Const) : LONGINT;
begin
    GT_GetGadgetAttrs := GT_GetGadgetAttrsA(gad,win,req,readintags(argv));
end;

PROCEDURE GT_SetGadgetAttrs(gad : pGadget; win : pWindow; req : pRequester; Const argv : Array Of Const);
begin
    GT_SetGadgetAttrsA(gad,win,req,readintags(argv));
end;

FUNCTION LayoutMenuItems(firstitem : pMenuItem; vi : POINTER; Const argv : Array Of Const) : BOOLEAN;
begin
    LayoutMenuItems := LayoutMenuItemsA(firstitem,vi,readintags(argv));
end;

FUNCTION LayoutMenus(firstmenu : pMenu; vi : POINTER; Const argv : Array Of Const) : BOOLEAN;
begin
    LayoutMenus := LayoutMenusA(firstmenu,vi,readintags(argv));
end;

FUNCTION AllocSpriteData(bm : pBitMap; Const argv : Array Of Const) : pExtSprite;
begin
    AllocSpriteData := AllocSpriteDataA(bm,readintags(argv));
end;

FUNCTION BestModeID(Const argv : Array Of Const) : ULONG;
begin
    BestModeID := BestModeIDA(readintags(argv));
end;

FUNCTION ChangeExtSprite(vp : pViewPort; oldsprite : pExtSprite; newsprite : pExtSprite; Const argv : Array Of Const) : LONGINT;
begin
    ChangeExtSprite := ChangeExtSpriteA(vp,oldsprite,newsprite,readintags(argv));
end;

FUNCTION ExtendFontTags(font : pTextFont; Const argv : Array Of Const) : ULONG;
begin
    ExtendFontTags := ExtendFont(font,readintags(argv));
end;

FUNCTION GetExtSprite(ss : pExtSprite; Const argv : Array Of Const) : LONGINT;
begin
    GetExtSprite := GetExtSpriteA(ss,readintags(argv));
end;

PROCEDURE GetRPAttrs(rp : pRastPort; Const argv : Array Of Const);
begin
    GetRPAttrsA(rp,readintags(argv));
end;

FUNCTION ObtainBestPen(cm : pColorMap; r : ULONG; g : ULONG; b : ULONG; Const argv : Array Of Const) : LONGINT;
begin
    ObtainBestPen := ObtainBestPenA(cm,r,g,b,readintags(argv));
end;

PROCEDURE SetRPAttrs(rp : pRastPort; Const argv : Array Of Const);
begin
    SetRPAttrsA(rp,readintags(argv));
end;

FUNCTION VideoControlTags(colorMap : pColorMap; Const argv : Array Of Const) : BOOLEAN;
begin
    VideoControlTags := VideoControl(colorMap,readintags(argv));
end;

FUNCTION WeighTAMatchTags(reqTextAttr : pTextAttr; targetTextAttr : pTextAttr; Const argv : Array Of Const) : smallint;
begin
    WeighTAMatchTags := WeighTAMatch(reqTextAttr,targetTextAttr,readintags(argv));
end;

FUNCTION OpenCatalog(locale : pLocale; name : pCHAR; Const argv : Array Of Const) : pCatalog;
begin
    OpenCatalog := OpenCatalogA(locale,name,readintags(argv));
end;

FUNCTION SetJoyPortAttrs(portNumber : ULONG; Const argv : Array Of Const) : BOOLEAN;
begin
    SetJoyPortAttrs := SetJoyPortAttrsA(portNumber,readintags(argv));
end;

FUNCTION SystemControl(Const argv : Array Of Const) : ULONG;
begin
    SystemControl := SystemControlA(readintags(argv));
end;

FUNCTION CreatePlayer(Const argv : Array Of Const) : pPlayer;
begin
    CreatePlayer := CreatePlayerA(readintags(argv));
end;

FUNCTION GetPlayerAttrs(player : pPlayer; Const argv : Array Of Const) : ULONG;
begin
    GetPlayerAttrs := GetPlayerAttrsA(player,readintags(argv));
end;

FUNCTION SetPlayerAttrs(player : pPlayer; Const argv : Array Of Const) : BOOLEAN;
begin
    SetPlayerAttrs := SetPlayerAttrsA(player,readintags(argv));
end;

function AllocNamedObject(name : STRPTR; Const argv : Array Of Const) : pNamedObject;
begin
    AllocNamedObject := AllocNamedObjectA(name,readintags(argv));
end;

FUNCTION AddAppMenuItem(id : ULONG; userdata : ULONG; text_ : pCHAR; msgport : pMsgPort; Const argv : Array Of Const) : pAppMenuItem;
begin
    AddAppMenuItem := AddAppMenuItemA(id,userdata,text_,msgport,readintags(argv));
end;

FUNCTION AddAppWindow(id : ULONG; userdata : ULONG; window : pWindow; msgport : pMsgPort; Const argv : Array Of Const) : pAppWindow;
begin
    AddAppWindow := AddAppWindowA(id,userdata,window,msgport,readintags(argv));
end;



end.
