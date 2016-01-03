PROGRAM Main;

(*
 *  OpenTriton -- A free release of the triton.library source code
 *  Copyright (C) 1993-1998  Stefan Zeiger
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  demo.c - Triton demo program
 *
 *)

uses exec, triton, tritonmacros, utility, amigalib, amigados,workbench;

{
   A demo in FPC Pascal using triton.library

   This is the main demo for triton.

   Updated for fpc 1.0.7.
   Added const NULL. Instead of typing longint(nil) just type null.
   After changes to tritonmacros (more overlays with SetTRTag)
   I could remove all longstr and other casts to longint.
   09 Jan 2003.

   nils.sjoholm@mailbox.swipnet.se
}



VAR
    App            : pTR_App;
    Main_Project   : pTR_Project;

const
    NULL : longint = longint(nil);

Function IntToStr (I : Longint) : String;

     Var S : String;

     begin
      Str (I,S);
      IntToStr:=S;
     end;


PROCEDURE do_text;
VAR
    close_me     : BOOLEAN;
    trmsg        : pTR_Message;
    text_project : pTR_Project;

BEGIN
    ProjectStart;
    WindowID(5); WindowTitle('Text'); WindowPosition(TRWP_CENTERDISPLAY);
    VertGroupA;
    Space; CenteredText('Normal text');
    Space; CenteredTextH('Highlighted text');
    Space; CenteredText3('3-dimensional text');
    Space; CenteredTextB('Bold text');
    Space; CenteredText('A _shortcut');
    Space; CenteredInteger(42);
    Space; HorizGroupAC;
             Space;
             ClippedText('This is a very long text which is truncated to fit with TRTX_CLIPPED.');
             Space; EndGroup;
    Space; EndGroup; EndProject;

    text_project := TR_OpenProject(App,@tritontags);
    IF text_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = text_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(text_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;



PROCEDURE do_groups;
VAR
    close_me       : BOOLEAN;
    trmsg          : pTR_Message;
    groups_project : pTR_Project;

BEGIN
    ProjectStart;
    WindowTitle('Groups'); WindowPosition(TRWP_CENTERDISPLAY); WindowUnderscore('~'); WindowID(1);

    HorizGroupA; Space; VertGroupA;
    Space;

    NamedFrameBox('TRGR_PROPSHARE (default)'); ObjectBackfillWin; VertGroupA; Space; HorizGroupC;
      Space;
      Button('Short',1);
      Space;
      Button('And much, much longer...',2);
      Space;
      EndGroup; Space; EndGroup;

    Space;

    NamedFrameBox('TRGR_EQUALSHARE'); ObjectBackfillWin; VertGroupA; Space; HorizGroupEC;
      Space;
      Button('Short',3);
      Space;
      Button('And much, much longer...',4);
      Space;
      EndGroup; Space; EndGroup;

    Space;

    NamedFrameBox('TRGR_PROPSPACES'); ObjectBackfillWin; VertGroupA; Space; HorizGroupSC;
      Space;
      Button('Short',5);
      Space;
      Button('And much, much longer...',6);
      Space;
      EndGroup; Space; EndGroup;

    Space;

    NamedFrameBox('TRGR_ARRAY'); ObjectBackfillWin; VertGroupA; Space; LineArray;
      BeginLine;
        Space;
        Button('Short',7);
        Space;
        Button('And much, much longer...',8);
        Space;
        EndLine;
      Space;
      BeginLine;
        Space;
        Button('Not so short',9);
        Space;
        Button('And a bit longer...',10);
        Space;
        EndLine;
      Space;
      BeginLineI;
        NamedSeparator('An independent line');
        EndLine;
      Space;
      BeginLine;
        Space;
        Button('foo bar',12);
        Space;
        Button('42',13);
        Space;
        EndLine;
      EndArray; Space; EndGroup;

    Space;
    EndGroup; Space; EndGroup;
    EndProject;

    groups_project := TR_OpenProject(App,@tritontags);
    IF groups_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = groups_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                  TRMS_CLOSEWINDOW : close_me := True;
                  TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(groups_project);
      END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;

PROCEDURE do_menus;
VAR
    close_me      : BOOLEAN;
    trmsg         : pTR_Message;
    menus_project : pTR_Project;

BEGIN
    ProjectStart;
       SetTRTag(TRWI_ID,                 2);
       SetTRTag(TRWI_Title,              'Menus');
       SetTRTag(TRMN_Title,              'A menu');
        SetTRTag(TRMN_Item,              'A simple item'); SetTRTag(TRAT_ID, 1);
        SetTRTag(TRMN_Item,              'Another item'); SetTRTag(TRAT_ID, 2);
        SetTRTag(TRMN_Item,              'And now... a barlabel'); SetTRTag(TRAT_ID, 3);
        SetTRTag(TRMN_Item,              TRMN_BARLABEL);
        SetTRTag(TRMN_Item,              '1_An item with a shortcut'); SetTRTag(TRAT_ID, 4);
        SetTRTag(TRMN_Item,              '2_Another one'); SetTRTag(TRAT_ID, 5);
        SetTRTag(TRMN_Item,              '3_And number 3'); SetTRTag(TRAT_ID, 6);
        SetTRTag(TRMN_Item,              TRMN_BARLABEL);
        SetTRTag(TRMN_Item,              '_F1_And under OS3.0: Extended command keys'); SetTRTag(TRAT_ID, 6);
        SetTRTag(TRMN_Item,              '_F2_Another one'); SetTRTag(TRAT_ID, 7);
        SetTRTag(TRMN_Item,              TRMN_BARLABEL);
        SetTRTag(TRMN_Item,              'How do you like submenus?');
         SetTRTag(TRMN_Sub,              'G_Great!'); SetTRTag(TRAT_ID, 8);
         SetTRTag(TRMN_Sub,              'F_Fine'); SetTRTag(TRAT_ID, 9);
         SetTRTag(TRMN_Sub,              'D_Don''t know'); SetTRTag(TRAT_ID, 10);
         SetTRTag(TRMN_Sub,              'N_Not so fine'); SetTRTag(TRAT_ID, 11);
         SetTRTag(TRMN_Sub,              'P_Puke!'); SetTRTag(TRAT_ID, 12);

       SetTRTag(TRMN_Title,              'Another menu');
        SetTRTag(TRMN_Item,              'This item is ghosted'); SetTRTag(TRMN_Flags, TRMF_DISABLED); SetTRTag(TRAT_ID, 100);
        SetTRTag(TRMN_Item,              TRMN_BARLABEL);
        SetTRTag(TRMN_Item,              'Item 1 is checked'); SetTRTag(TRMN_Flags, TRMF_CHECKED); SetTRTag(TRAT_ID, 13);
        SetTRTag(TRMN_Item,              'Item 2 can be checked, too'); SetTRTag(TRMN_Flags, TRMF_CHECKIT); SetTRTag(TRAT_ID, 14);

       SetTRTag(TRMN_Title,              'Ghosted menu');
       SetTRTag(TRMN_Flags,              TRMF_DISABLED);
        SetTRTag(TRMN_Item,              'Item 1'); SetTRTag(TRAT_ID, 101);
        SetTRTag(TRMN_Item,              'Item 2'); SetTRTag(TRAT_ID, 102);

       EndProject;

    menus_project := TR_OpenProject(App,@tritontags);
    IF menus_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = menus_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR       : WriteLN(TR_GetErrorString(trmsg^.trm_Data));
                 TRMS_NEWVALUE    : writeln('The new value of object ',trmsg^.trm_ID,' is ',trmsg^.trm_Data);
                 TRMS_ACTION      : writeln('Object ',trmsg^.trm_ID,' has triggered an action.');
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(menus_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;

PROCEDURE do_gadgets;
CONST
  cycle_entries : ARRAY [0..10] OF PChar = (
                  'Entry 0',
                  '1',
                  '2',
                  '3',
                  '4',
                  '5',
                  '6',
                  '7',
                  '8',
                  '9',
                  NIL);


  mx_entries : ARRAY [0..3] OF PChar = (
                  'Choice 0',
                  'Choice 1',
                  'Choice 2',
                  NIL);
VAR
    close_me        : BOOLEAN;
    trmsg           : pTR_Message;
    gadgets_project : pTR_Project;

BEGIN
    ProjectStart;
    SetTRTag(TRWI_ID,  3);
    SetTRTag(TRWI_Title,'Gadgets');
    SetTRTag(TRWI_Position,TRWP_CENTERDISPLAY);

  SetTRTag(TRGR_Vert,                   TRGR_PROPSHARE OR TRGR_ALIGN);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Line,               TROF_HORIZ);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  'GadTools'); SetTRTag(TRAT_Flags,  TRTX_TITLE);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Line,               TROF_HORIZ);
      SetTRTag(TROB_Space,              NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_CheckBox,         NULL);
          SetTRTag(TRAT_ID,             1);
          SetTRTag(TRAT_Value,          true);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           '_Checkbox');
          SetTRTag(TRAT_ID,             1);
        SetTRTag(TROB_Space,            NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_Slider,           NULL);
          SetTRTag(TRAT_ID,             4);
          SetTRTag(TRSL_Min,            1);
          SetTRTag(TRSL_Max,            3);
          SetTRTag(TRAT_Value,          1);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           '_Slider: ');
          SetTRTag(TRAT_ID,             4);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Value,          1);
          SetTRTag(TRAT_ID,             4);
          SetTRTag(TRAT_MinWidth,       3);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_Scroller,         NULL);
          SetTRTag(TRAT_ID,             5);
          SetTRTag(TRAT_Value,          2);
          SetTRTag(TRSC_Total,          7);
          SetTRTag(TRSC_Visible,        3);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           'Sc_roller: ');
          SetTRTag(TRAT_ID,             5);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Value,          2);
          SetTRTag(TRAT_ID,             5);
          SetTRTag(TRAT_MinWidth,       3);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_Palette,          NULL);
          SetTRTag(TRAT_ID,             3);
          SetTRTag(TRAT_Value,          1);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           '_Palette: ');
          SetTRTag(TRAT_ID,             3);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Value,          1);
          SetTRTag(TRAT_ID,             3);
          SetTRTag(TRAT_MinWidth,       3);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_Cycle,            @cycle_entries);
          SetTRTag(TRAT_ID,             6);
          SetTRTag(TRAT_Value,          4);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           'C_ycle: ');
          SetTRTag(TRAT_ID,             6);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Value,          4);
          SetTRTag(TRAT_ID,             6);
          SetTRTag(TRAT_MinWidth,       3);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_Cycle,            @mx_entries);
          SetTRTag(TRAT_ID,             13);
          SetTRTag(TRAT_Value,          1);
          SetTRTag(TRAT_Flags,          TRCY_MX);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           '_MX: ');
          SetTRTag(TRAT_ID,             13);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Value,          1);
          SetTRTag(TRAT_ID,             13);
          SetTRTag(TRAT_MinWidth,       3);
        SetTRTag(TROB_Space,            NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_String,           'foo bar');
          SetTRTag(TRAT_ID,             7);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           'S_tring');
          SetTRTag(TRAT_ID,             7);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_String,           '');
          SetTRTag(TRAT_Flags,          TRST_INVISIBLE);
          SetTRTag(TRAT_ID,             15);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           'Pass_word');
          SetTRTag(TRAT_ID,             15);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TROB_String,           '0.42');
          SetTRTag(TRAT_Flags,          TRST_FLOAT);
          SetTRTag(TRST_Filter,         '01234567.,');
          SetTRTag(TRAT_ID,             16);
      SetTRTag(TRGR_End,                NULL);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
        SetTRTag(TROB_Text,             NULL);
          SetTRTag(TRAT_Text,           '_Octal float');
          SetTRTag(TRAT_ID,             16);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Line,               TROF_HORIZ);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  'BOOPSI'); SetTRTag(TRAT_Flags,  TRTX_TITLE);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Line,               TROF_HORIZ);
      SetTRTag(TROB_Space,              NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Button,             NULL); SetTRTag(TRAT_ID, 2); SetTRTag(TRAT_Text,  '_Button');
      SetTRTag(TROB_Space,              NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_PROPSPACES OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  '_File:'); SetTRTag(TRAT_ID, 10);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Button,             TRBT_GETFILE); SetTRTag(TRAT_ID, 10); SetTRTag(TRAT_Text,  '');
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  '_Drawer:'); SetTRTag(TRAT_ID, 11);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Button,             TRBT_GETDRAWER); SetTRTag(TRAT_ID, 11); SetTRTag(TRAT_Text,  '');
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  '_Entry:'); SetTRTag(TRAT_ID, 12);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Button,             TRBT_GETENTRY); SetTRTag(TRAT_ID, 12); SetTRTag(TRAT_Text,  '');
      SetTRTag(TROB_Space,              NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_End,                  NULL);

  EndProject;
    gadgets_project := TR_OpenProject(App,@tritontags);
    IF gadgets_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = gadgets_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(gadgets_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;


PROCEDURE do_backfill;
VAR
    close_me         : BOOLEAN;
    trmsg            : pTR_Message;
    backfill_project : pTR_Project;

BEGIN
ProjectStart;
  WindowID(7); WindowTitle('Backfill'); WindowPosition(TRWP_CENTERDISPLAY);
  VertGroupA;
    Space;  CenteredText('Each window and');
    SpaceS; CenteredText('FrameBox can have');
    SpaceS; CenteredText('one of the following');
    SpaceS; CenteredText('backfill patterns');
    Space;  HorizGroupA;
              Space; GroupBox; ObjectBackfillS; SpaceB;
              Space; GroupBox; ObjectBackfillSA; SpaceB;
              Space; GroupBox; ObjectBackfillSF; SpaceB;
              Space; EndGroup;
    Space;  HorizGroupA;
              Space; GroupBox; ObjectBackfillSB; SpaceB;
              Space; GroupBox; ObjectBackfillA; SpaceB;
              Space; GroupBox; ObjectBackfillAF; SpaceB;
              Space; EndGroup;
    Space;  HorizGroupA;
              Space; GroupBox; ObjectBackfillAB; SpaceB;
              Space; GroupBox; ObjectBackfillF; SpaceB;
              Space; GroupBox; ObjectBackfillFB; SpaceB;
              Space; EndGroup;
    Space; EndGroup; EndProject;

    backfill_project := TR_OpenProject(App,@tritontags);
    IF backfill_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = backfill_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(backfill_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;


PROCEDURE do_disabling;
VAR
    close_me          : BOOLEAN;
    trmsg             : pTR_Message;
    disabling_project : pTR_Project;

BEGIN
  ProjectStart;
  SetTRTag(TRWI_ID,4); SetTRTag(TRWI_Title,'Disabling'); SetTRTag(TRWI_Position,TRWP_CENTERDISPLAY);
  SetTRTag(TRGR_Vert,                   TRGR_PROPSHARE OR TRGR_ALIGN);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_CheckBox,           NULL); SetTRTag(TRAT_ID, 1); SetTRTag(TRAT_Value, true);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  '_Disabled'); SetTRTag(TRAT_ID, 1);
      SetTRTag(TRGR_Horiz,              TRGR_PROPSPACES);
        SetTRTag(TROB_Space,            NULL);
        SetTRTag(TRGR_End,              NULL);
      SetTRTag(TRGR_End,                NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_EQUALSHARE OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Line,               TROF_HORIZ);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TRGR_End,                NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_CheckBox,           NULL); SetTRTag(TRAT_Value, true); SetTRTag(TRAT_ID, 2); SetTRTag(TRAT_Disabled, true);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Text,               NULL); SetTRTag(TRAT_Text,  '_Checkbox'); SetTRTag(TRAT_ID, 2);
      SetTRTag(TROB_Space,              NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

    SetTRTag(TRGR_Horiz,                TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_CENTER);
      SetTRTag(TROB_Space,              NULL);
      SetTRTag(TROB_Button,             NULL); SetTRTag(TRAT_Text,  '_Button'); SetTRTag(TRAT_ID, 3); SetTRTag(TRAT_Disabled, true);
      SetTRTag(TROB_Space,              NULL);
    SetTRTag(TRGR_End,                  NULL);

    SetTRTag(TROB_Space,                NULL);

  SetTRTag(TRGR_End,                    NULL);

  EndProject;

    disabling_project := TR_OpenProject(App,@tritontags);
    IF disabling_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = disabling_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
                 TRMS_NEWVALUE: BEGIN
                                  IF trmsg^.trm_ID =1 THEN BEGIN
                                    TR_SetAttribute(disabling_project,2,TRAT_Disabled,trmsg^.trm_Data);
                                    TR_SetAttribute(disabling_project,3,TRAT_Disabled,trmsg^.trm_Data);
                                  END;
                                END;
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(disabling_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;


PROCEDURE do_notification;
VAR
    close_me             : BOOLEAN;
    trmsg                : pTR_Message;
    notification_project : pTR_Project;

BEGIN
ProjectStart;
  WindowID(6); WindowTitle('Notification'); WindowPosition(TRWP_CENTERDISPLAY);
  VertGroupA;
    Space;
    NamedSeparatorI('_Checkmarks',1);
    Space;

    HorizGroupSAC;
      Space; CheckBox(1);
      Space; CheckBox(1);
      Space; CheckBox(1);
      Space; EndGroup;

    Space;

    HorizGroupSAC;
      Space; CheckBox(1);
      Space; CheckBox(1);
      Space; CheckBox(1);
      Space; EndGroup;

    Space;
    HorizGroupSAC;
      Space; CheckBox(1);
      Space; CheckBox(1);
      Space; CheckBox(1);
      Space; EndGroup;

    Space;
    NamedSeparatorI('_Slider and Progress indicator',2);
    Space;

    HorizGroupAC;
      Space;
      SliderGadget(0,10,8,2);
      Space;
      Integer3(8);SetTRTag(TRAT_ID,2);SetTRTag(TRAT_MinWidth,3);
      Space;
      EndGroup;

    Space;

    HorizGroupAC;
      Space;
      TextN('0%');
      Space;
      Progress(10,8,2);
      Space;
      TextN('100%');
      Space;
      EndGroup;

    Space;
  EndGroup; EndProject;

    notification_project := TR_OpenProject(App,@tritontags);
    IF notification_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = notification_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(notification_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;


PROCEDURE do_lists;
CONST

    LVList1Strings : ARRAY[0..18] OF PChar = (
                       'This is a' ,
                       'READ ONLY' ,
                       'Listview' ,
                       'gadget using' ,
                       'the fixed-' ,
                       'width font.' ,
                       '' ,
                       'This window' ,
                       'will remember' ,
                       'its position' ,
                       'even without' ,
                       'the Preferences' ,
                       'system, when' ,
                       'you reopen it,' ,
                       'because it has' ,
                       'got a dimension' ,
                       'structure' ,
                       'attached' ,
                       'to it.');



    LVList2Strings : ARRAY [0..8] OF PChar = (
                       'This is a' ,
                       'SELECT' ,
                       'Listview' ,
                       'gadget.' ,
                       'Use the' ,
                       'numeric' ,
                       'key pad to' ,
                       'move' ,
                       'around.');



    LVList3Strings : ARRAY [0..12] OF PChar = (
                       'This is a' ,
                       'SHOW' ,
                       'SELECTED' ,
                       'Listview' ,
                       'gadget.' ,
                       'This list' ,
                       'is a bit' ,
                       'longer, so' ,
                       'that you' ,
                       'can try the' ,
                       'other' ,
                       'keyboard' ,
                       'shortcuts.');

VAR
    close_me      : BOOLEAN;
    trmsg         : pTR_Message;
    lists_project : pTR_Project;
    i             : Longint;
    LVList1,
    LVList2,
    LVList3       : pList;
    MyNode        : pNode;

BEGIN

    New(LVList1);
    NewList(LVList1);
    FOR i := 0 TO 18 DO BEGIN
        New(MyNode);
        MyNode^.ln_Name := LVList1Strings[i];
        AddTail(LVList1,MyNode);
    END;

    New(LVList2);
    NewList(LVList2);
    FOR i := 0 TO 8 DO BEGIN
        New(MyNode);
        MyNode^.ln_Name := LVList2Strings[i];
        AddTail(LVList2,MyNode);
    END;

    New(LVList3);
    NewList(LVList3);
    FOR i := 0 TO 12 DO BEGIN
        New(MyNode);
        MyNode^.ln_Name := LVList3Strings[i];
        AddTail(LVList3,MyNode);
    END;

ProjectStart;
  WindowID(9); WindowTitle('Lists'); WindowPosition(TRWP_CENTERDISPLAY);
  HorizGroupA; Space; VertGroupA;
    Space;
    NamedSeparatorIN('_Read only',1);
    Space;
    FWListROCN(LVList1,1,0);
    Space;
    NamedSeparatorIN('_Select',2);
    Space;
    ListSelC(LVList2,2,0);
    Space;
    NamedSeparatorIN('S_how selected',3);
    Space;
    ListSSN(LVList3,3,0,1);
    Space;
  EndGroup; Space; EndGroup;
  EndProject;

    lists_project := TR_OpenProject(App,@tritontags);
    IF lists_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = lists_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(lists_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;

PROCEDURE do_appwindow;
VAR
    close_me          : BOOLEAN;
    trmsg             : pTR_Message;
    appwindow_project : pTR_Project;
    chararray         : ARRAY [0..100] OF Char;
    dirname           : PChar;
    temp              : BOOLEAN;
    reqstr            : STRING[200];

BEGIN
    dirname := @chararray;
ProjectStart;
  WindowID(8); WindowTitle('AppWindow'); WindowPosition(TRWP_CENTERDISPLAY);
  VertGroupA;
    Space;  CenteredText('This window is an application window.');
    SpaceS; CenteredText('Drop icons into the window or into');
    SpaceS; CenteredText('the icon drop boxes below and see');
    SpaceS; CenteredText('what will happen...');
    Space;  HorizGroupA;
              Space; DropBox(1);
              Space; DropBox(2);
              Space; EndGroup;
    Space; EndGroup; EndProject;

  appwindow_project := TR_OpenProject(App,@tritontags);
    IF appwindow_project <> NIL THEN BEGIN
      TR_LockProject(Main_Project);
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = appwindow_project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
                 TRMS_ICONDROPPED:  BEGIN
                                      dirname[0] := #0;
                                      temp := NameFromLock(Longint(pAppMessage(trmsg^.trm_Data)^.am_ArgList^[1].wa_Lock),dirname,100);
                                      temp := AddPart(dirname,(pAppMessage(trmsg^.trm_Data)^.am_ArgList^[1].wa_Name),100);
                                      case trmsg^.trm_ID of
                                         1: reqstr := 'Icon(s) dropped into the left box.' + #9 + 'Name of first dropped icon:' + #10 + '%3' + strpas(dirname);
                                         2: reqstr := 'Icon(s) dropped into the right box.' + #9 + 'Name of first dropped icon:' + #10 + '%3' + strpas(dirname);
                                         ELSE reqstr := 'Icon(s) dropped into the window.' + #9 + 'Name of first dropped icon:' + #10 + '%3' + strpas(dirname);
                                      END;
                                      TR_EasyRequestTags(App,reqstr,'_Ok',[
                                                     TREZ_LockProject, appwindow_project,
                                                     TREZ_Title,'AppWindow report',
                                                     TREZ_Activate,1,
                                                     TAG_END]);

                                    END;
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_UnlockProject(Main_Project);
      TR_CloseProject(appwindow_project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;

PROCEDURE Do_Main;
VAR
    trmsg      : pTR_Message;
    quit       : BOOLEAN;
    reqstr     : string[255];
    helpstring : PChar;
    abouttags  : Pointer;

BEGIN

    ProjectStart;
    BeginRequester('About...',TRWP_CENTERDISPLAY);

    VertGroupA; Space;  CenteredText3('Triton Demo 2.0');
              SpaceS; CenteredText('� 1993-1996 by Stefan Zeiger');
              Space;  HorizSeparator;
              Space;  CenteredText('This program is using the');
              SpaceS; CenteredText('Triton GUI creation system');
              SpaceS; CenteredText('which is � by Stefan Zeiger');
              Space;  EndGroup;

    BeginRequesterGads;
    CenteredButtonRE('_Ok',1);
    EndRequester;

    abouttags := CloneTagItems(@tritontags);

    ProjectStart;
    WindowID(10); WindowTitle('Triton Demo');
    WindowPosition(TRWP_CENTERDISPLAY);
    WindowFlags(TRWF_HELP);
    QuickHelpOn(1);
    BeginMenu('Project');
      MenuItem('?_About...',101);
      ItemBarlabel;
      MenuItem('H_Help',102);
      MenuItemCC('I_QuickHelp',104);
      ItemBarlabel;
      MenuItem('Q_Quit',103);
    VertGroupA;
      Space;  CenteredText3('T � r � i � t � o � n');
      Space;  CenteredText3('The object oriented GUI creation system');
      Space;  CenteredText('Demo program for release 2.0');
      Space;  CenteredText('Written and � 1993-1997 by Stefan Zeiger');
      Space;  CenteredText('This demo made in FPC Pascal');
      Space;  HorizSeparator;
      Space;  HorizGroupEA;
              Space; Button('_Gadgets',1); QuickHelp('Show some fancy gadgets');
              Space; Button('G_roups',2); QuickHelp('Groupies?'+#10+'Huh huh...');
              Space; Button('_Text',3); QuickHelp('You know what ''text'' means, huh?');
              Space; EndGroup;
      Space; HorizGroupEA;
              Space; Button('_Connections',4); QuickHelp('So you''re super-connected now...');
              Space; Button('_Backfill',5); QuickHelp('United colors of Triton');
              Space; Button('_Disabling',6); QuickHelp('To be or not to be');
              Space; EndGroup;
      Space; HorizGroupEA;
              Space; Button('_AppWindow',7); QuickHelp('Demonstrate AppWindow feature');
              Space; Button('_Menus',8); QuickHelp('A fancy pull-down menu');
              Space; Button('_Lists',9); QuickHelp('� 4 eggs'+#10+'� 1/2lbs bread'+#10+'� 1l milk'+#9+'%3PS: Don''t be late');
              Space; EndGroup;
      Space; EndGroup; EndProject;

    Main_Project := TR_OpenProject(App,@tritontags);
    IF Main_Project <> NIL THEN BEGIN
      quit := FALSE;
      WHILE NOT quit DO BEGIN
        TR_Wait(app,0);
        REPEAT
          trmsg := TR_GetMsg(app);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = Main_Project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : quit := True;
                 TRMS_NEWVALUE    : IF (trmsg^.trm_ID=104) THEN TR_SetAttribute(Main_Project,0,TRWI_QuickHelp,trmsg^.trm_Data);
                 TRMS_ACTION      : BEGIN
                                      CASE trmsg^.trm_ID OF
                                          1: do_gadgets;
                                          2: do_groups;
                                          3: do_text;
                                          4: do_notification;
                                          5: do_backfill;
                                          6: do_disabling;
                                          7: do_appwindow;
                                          8: do_menus;
                                          9: do_lists;
                                        101: TR_AutoRequest(App,Main_Project,abouttags);
                                        102: begin
                                                TR_EasyRequestTags(App,'To get help, move the mouse pointer over' + #10 +
                                                'any gadget or menu item and press <Help>'+#10+
                                                'or turn on QuickHelp before.','_Ok',[
                                                TREZ_LockProject,Main_Project,
                                                TREZ_Title,'Triton help',
                                                TAG_END]);
                                             end;
                                        103: quit := True;
                                       END;
                                    END;
                 TRMS_HELP        : BEGIN
                                      helpstring := PChar(TR_GetAttribute(Main_Project,trmsg^.trm_ID,TRDO_QuickHelpString));
                                      IF helpstring <> NIL THEN BEGIN
                                         reqstr := 'Help for object ' + IntToStr(trmsg^.trm_ID) + ':' + #10 + '%h' + strpas(helpstring);
                                      END ELSE BEGIN
                                         reqstr := 'No help available for object ' + IntToStr(trmsg^.trm_ID);
                                      END;
                                      TR_EasyRequestTags(App,reqstr,'_Ok',[
                                                     TREZ_LockProject,Main_Project,
                                                     TREZ_Title,'Triton help',
                                                     TAG_END]);
                                    END;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
                 ELSE
                 END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL quit OR (trmsg = NIL);
      END;
      TR_CloseProject(Main_Project);
      FreeTagItems(abouttags);
      END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(App)));
END;

BEGIN

    App := TR_CreateAppTags([
              TRCA_Name,'TritonDemo',
              TRCA_LongName,'Triton Demo',
              TRCA_Version,'2.0',
              TAG_DONE]);

    if App <> nil then begin
       Do_Main;
       TR_DeleteApp(App);;
    end;
END.
