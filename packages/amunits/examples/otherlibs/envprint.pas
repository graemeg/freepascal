Program EnvPrint;

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
 *  envprint.c - Envprint 2.0 GUI created with Triton
 *
 *  As you can see below, it is possible to mix the tag format with
 *  the C pre-processor macro format. (Actually I was just too lazy
 *  to transform the whole project definition from tags to macros ;)
 *
 *)

{
   A demo in FPC Pascal using triton.library

   Updated for fpc 1.0.7.
   Added const null and made use of the updated
   tritonmacros.
   09 Jan 2003.

   nils.sjoholm@mailbox.swipnet.se
}


uses triton, tritonmacros,utility;

const
    null : longint = longint(nil);

var
    Triton_App : pTR_App;

PROCEDURE do_demo;
VAR
    close_me   : BOOLEAN;
    trmsg      : pTR_Message;
    dummy      : Longint;
    Project    : pTR_Project;

BEGIN
  ProjectStart;
  WindowID(1); WindowPosition(TRWP_CENTERDISPLAY);
  WindowTitle('EnvPrint 2.0 <THIS IS ONLY A NON-FUNCTIONAL GUI DEMO>');

  BeginMenu('Project');
    BeginSub('Load');
      SubItem('S_Load sender...',1);
      SubItem('D_Load addressee...',2);
      SubItem('C_Load comment...',3);
    BeginSub('Save');
      SubItem('O_Load sender',4);
      SubItem('E_Load addressee',5);
      SubItem('M_Load comment',6);
    BeginSub('Sace as');
      SubItem('U_Load sender as...',7);
      SubItem('T_Load addressee as...',8);
      SubItem('N_Load comment as...',9);
    MenuItem('F_Delete file...',10);
    ItemBarlabel;
    MenuItem('P_Print...',11);
    MenuItem('R_Preferences...',12);
    ItemBarlabel;
    MenuItem('?_About...',13);
    ItemBarlabel;
    MenuItem('Q_Quit',14);

  BeginMenu('Edit');
    MenuItem('W_Swap',15);
    MenuItem('X_Clear',16);

  HorizGroupA;
    Space;
    VertGroupA;
      HorizGroupEAC;
        VertGroupA;

          Space;

          NamedSeparatorI('Se_nder',101);

          SetTRTag(TROB_Space,          null);

          HorizGroup;
            StringGadget(NIL,101);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_1',1101);
              EndGroup;
            EndGroup;

          SpaceS;

          HorizGroup;
            StringGadget(NIL,102);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_2',1102);
              EndGroup;
            EndGroup;

          SpaceS;

          HorizGroup;
            StringGadget(NIL,103);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_3',1103);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,104);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_4',1104);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,105);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_5',1105);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,106);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_6',1106);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,107);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_7',1107);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,108);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_8',1108);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          null);

          SetTRTag(TRGR_End,            null);

        SetTRTag(TROB_Space,            null);

        SetTRTag(TRGR_Vert,             TRGR_PROPSHARE OR TRGR_ALIGN);

          SetTRTag(TROB_Space,          null);

          NamedSeparatorI('Add_ressee',201);

          SetTRTag(TROB_Space,          null);

          HorizGroup;
            StringGadget(NIL,201);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_!',1201);
              EndGroup;
            EndGroup;

          SpaceS;

          HorizGroup;
            StringGadget(NIL,202);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_''',1202);
              EndGroup;
            EndGroup;

          SpaceS;

          HorizGroup;
            StringGadget(NIL,203);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_§',1203);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,204);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_$',1204);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,205);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_%%',1205);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,206);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_&',1206);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,207);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_/',1207);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          TRST_SMALL);

          HorizGroup;
            StringGadget(NIL,208);
            SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
              GetEntryButtonS('_(',1208);
              EndGroup;
            EndGroup;

          SetTRTag(TROB_Space,          null);

          SetTRTag(TRGR_End,            null);

        SetTRTag(TRGR_End,              null);

      NamedSeparatorI('Co_mment',301);

      SetTRTag(TROB_Space,              null);

      HorizGroup;
        StringGadget(NIL,301);
        SetTRTag(TRGR_Horiz,0 OR TRGR_FIXHORIZ);;
          GetEntryButtonS('_0',1301);
          EndGroup;
        EndGroup;

      SetTRTag(TROB_Space,              null);

      SetTRTag(TRGR_End,                null);

    SetTRTag(TROB_Space,                null);
    SetTRTag(TROB_Line,                 TROF_VERT OR TROF_RAISED);

    SetTRTag(TROB_Space,                TRST_BIG);

    SetTRTag(TRGR_Vert,                 TRGR_PROPSHARE OR TRGR_ALIGN OR TRGR_FIXHORIZ);
      SetTRTag(TROB_Space,              null);
      SetTRTag(TRGR_Horiz,              TRGR_EQUALSHARE OR TRGR_CENTER);
        SetTRTag(TROB_Line,             TROF_HORIZ);
        SetTRTag(TROB_Space,            null);
        SetTRTag(TROB_Text,             null); SetTRTag(TRAT_Text,  'Load'); SetTRTag(TRAT_Flags, TRTX_TITLE);
        SetTRTag(TROB_Space,            null);
        SetTRTag(TROB_Line,             TROF_HORIZ);
        SetTRTag(TRGR_End,              null);
      SetTRTag(TROB_Space,              null);
      SetTRTag(TROB_Button,             null); SetTRTag(TRAT_Text,  '_Sender...'); SetTRTag(TRAT_ID, 501);
      SetTRTag(TROB_Space,              TRST_SMALL);
      SetTRTag(TROB_Button,             null); SetTRTag(TRAT_Text,  '_Addressee...'); SetTRTag(TRAT_ID, 502);
      SetTRTag(TROB_Space,              TRST_SMALL);
      SetTRTag(TROB_Button,             null); SetTRTag(TRAT_Text,  '_Comment...'); SetTRTag(TRAT_ID, 503);
      SetTRTag(TROB_Space,              null);
      SetTRTag(TRGR_Horiz,              TRGR_EQUALSHARE OR TRGR_CENTER);
        SetTRTag(TROB_Line,             TROF_HORIZ);
        SetTRTag(TROB_Space,            null);
        SetTRTag(TROB_Text,             null); SetTRTag(TRAT_Text,  'Save'); SetTRTag(TRAT_Flags, TRTX_TITLE);
        SetTRTag(TROB_Space,            null);
        SetTRTag(TROB_Line,             TROF_HORIZ);
        SetTRTag(TRGR_End,              null);
      SetTRTag(TROB_Space,              null);
      SetTRTag(TROB_Button,             null); SetTRTag(TRAT_Text,  'S_ender...'); SetTRTag(TRAT_ID, 504);
      SetTRTag(TROB_Space,              TRST_SMALL);
      SetTRTag(TROB_Button,             null); SetTRTag(TRAT_Text,  'A_ddressee...'); SetTRTag(TRAT_ID, 505);
      SetTRTag(TROB_Space,              TRST_SMALL);
      SetTRTag(TROB_Button,             null); SetTRTag(TRAT_Text,  'C_omment...'); SetTRTag(TRAT_ID, 506);
      SetTRTag(TROB_Space,              TRST_BIG);
      SetTRTag(TROB_Line,               TROF_HORIZ);
      SetTRTag(TROB_Space,              TRST_BIG);
      SetTRTag(TRGR_Horiz,              TRGR_EQUALSHARE);
        SetTRTag(TROB_Button,           null); SetTRTag(TRAT_Text,  '_Print...'); SetTRTag(TRAT_ID, 507);
        SetTRTag(TROB_Space,            TRST_SMALL);
        SetTRTag(TROB_Button,           null); SetTRTag(TRAT_Text,  'S_wap'); SetTRTag(TRAT_ID, 508);
        SetTRTag(TRGR_End,              null);
      SetTRTag(TROB_Space,              TRST_SMALL);
      SetTRTag(TRGR_Horiz,              TRGR_EQUALSHARE);
        SetTRTag(TROB_Button,           null); SetTRTag(TRAT_Text,  'Pre_fs...'); SetTRTag(TRAT_ID, 509);
        SetTRTag(TROB_Space,            TRST_SMALL);
        SetTRTag(TROB_Button,           null); SetTRTag(TRAT_Text,  'C_lear'); SetTRTag(TRAT_ID, 510);
        SetTRTag(TRGR_End,              null);
      SetTRTag(TROB_Space,              null);
    SetTRTag(TRGR_End,                  null);

    SetTRTag(TROB_Space,                null);

  SetTRTag(TRGR_End,                    null);

  EndProject;

  Project := TR_OpenProject(Triton_App,@tritontags);
    IF Project <> NIL THEN BEGIN
      close_me := FALSE;
      WHILE NOT close_me DO BEGIN
        dummy := TR_Wait(Triton_App,0);
        REPEAT
          trmsg := TR_GetMsg(Triton_App);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = Project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_CLOSEWINDOW : close_me := True;
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
      END;
      TR_CloseProject(Project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(Triton_App)));
END;

begin
  Triton_App := TR_CreateAppTags([
                TRCA_Name,'Envprint',
                TRCA_LongName,'EnvPrint GUI demo',
                TRCA_Version,'2.0',
                TAG_END]);

  if Triton_App <> nil then begin
     do_demo;
     TR_DeleteApp(Triton_App);
  END
  ELSE writeln('Can''t create application');
END.
