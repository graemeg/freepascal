program ProgIndex;

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
 *  progind.c - Progress indicator demo
 *
 *)

{
   A demo in FPC Pascal using triton.library

   Updated for fpc 1.0.7
   11 Jan 2003.

   nils.sjoholm@mailbox.swipnet.se
}

uses triton, tritonmacros, amigados, utility;



const
     ID_MAIN_GADGET_STOP = 1;
     ID_MAIN_PROGIND     = 2;

var
   Triton_App : pTR_App;

procedure do_main;
var
  close_me  : boolean;
  trmsg     : pTR_Message;
  project   : pTR_Project;
  i         : Longint;

begin
    close_me := false;
    i := 0;

    ProjectStart;
    WindowID(1);
    WindowTitle('Progress Indicator Demo');
    WindowPosition(TRWP_CENTERDISPLAY);
    WindowFlags(TRWF_NOCLOSEGADGET OR TRWF_NOESCCLOSE);

    VertGroupA;
      Space;  CenteredText('Working...');
      Space;  HorizGroupA;
                Space; Progress(100,0,ID_MAIN_PROGIND); (* A per cent progress indicator *)
                Space; EndGroup;
      SpaceS;HorizGroupA;
                Space; HorizGroupSA; TextN('000%'); Space; TextN('050%'); Space; TextN('100%'); EndGroup;
                Space; EndGroup;
      Space; HorizGroupSA;
                Space; ButtonE('_Stop',ID_MAIN_GADGET_STOP);
                Space; EndGroup;
      Space; EndGroup;

    EndProject;

    project := TR_OpenProject(Triton_App,@tritontags);

    IF Project <> NIL THEN BEGIN
      WHILE NOT close_me DO BEGIN
        TR_SetAttribute(project,ID_MAIN_PROGIND,TRAT_Value,i);
        DOSDelay(10);
        REPEAT
          trmsg := TR_GetMsg(Triton_App);
          IF trmsg <> NIL THEN BEGIN
            IF (trmsg^.trm_Project = Project) THEN BEGIN
               CASE trmsg^.trm_Class OF
                 TRMS_ERROR:        WriteLN(TR_GetErrorString(trmsg^.trm_Data));
                 TRMS_ACTION :
                 BEGIN
                 CASE trmsg^.trm_ID OF
                   ID_MAIN_GADGET_STOP : close_me := True;
                 END;
               END;
               ELSE
               END;
            END;
            TR_ReplyMsg(trmsg);
          END;
        UNTIL close_me OR (trmsg = NIL);
        inc(i);
        if i = 101 then close_me := true;
      END;
      TR_CloseProject(project);
    END ELSE WriteLN(TR_GetErrorString(TR_GetLastError(Triton_App)));
end;


(* /////////////////////////////////////////////////////////////////////////////////////////////////////// *)
(* ////////////////////////////////////////////////////////////////////////////////////// Main function // *)
(* /////////////////////////////////////////////////////////////////////////////////////////////////////// *)

begin
  Triton_App := TR_CreateAppTags([
                TRCA_Name,'trProgIndDemo',
                TRCA_Version,'1.0',
                TAG_END]);

  if Triton_App <> nil then begin
    do_main;
    TR_DeleteApp(Triton_App);
  end
  else writeln('Can''t create application');
end.
