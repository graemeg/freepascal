Program ToolManager1;

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
 *  Toolmanager1.c - Looks like the original ToolManager
 *
 *)

{
   A demo in FPC Pascal using triton.library

   Updated for fpc 1.0.7
   11 Jan 2003.

   nils.sjoholm@mailbox.swipnet.se
}

uses exec, triton, tritonmacros, amigalib, utility, linklist;



const
     cycle_entries : array [0..7] of PChar = ('Exec','Image','Sound','Menu','Icon','Dock','Access',NIL);

     liststrings : array [0..8] of PChar = (
                     '2024view' ,
                     'Add to archive',
                     'Delete',
                     'Edit text',
                     'Env',
                     'Exchange',
                     'Global Help System',
                     'Multiview',
                     'Paint');

var
   i : Longint;
   LVList : pList;
   MyNode : pFPCNode;
   Triton_App : pTR_App;

procedure CleanUp(why : string; err : longint);
begin
    if assigned(Triton_App) then  TR_DeleteApp(Triton_App);
    if assigned(LVList) then DestroyList(LVList);
    if why <> '' then writeln(why);
    halt(err);
end;

begin
    CreateList(LVList);
    FOR i := 0 TO 8 DO BEGIN
        MyNode := AddNewNode(LVList,liststrings[i]);
    END;

    Triton_App := TR_CreateAppTags([
                               TRCA_Name,'ToolManagerGUIDemo1',
                               TRCA_LongName,'ToolManager GUI demo 1',
                               TRCA_Info,'Looks like the original ToolManager',
                               TAG_END]);

    if Triton_App = nil then CleanUp('Can''t create application',20);

      ProjectStart;
      WindowID(0); WindowPosition(TRWP_BELOWTITLEBAR);
      WindowTitle('ToolManager GUI demo 1'); WindowFlags(TRWF_NOSIZEGADGET OR TRWF_NODELZIP OR TRWF_NOZIPGADGET OR TRWF_NOESCCLOSE);
      WindowBackfillNone;

      VertGroupA;

        Space;

        HorizGroupAC;
          Space;
          TextID('_Object Type',1);
          Space;
          CycleGadget(@cycle_entries,0,1);
          Space;
        EndGroup;

        Space;

        HorizGroupAC;
          Space;
            VertGroupAC;
              CenteredTextID('Object List',2);
              Space;
              ListSSCN(LVList,2,0,0);
            EndGroup;
          Space;
            VertGroupA;
              TextN('');
              Space;
              Button('Top',3);
              Space;
              Button('Up',4);
              Space;
              Button('Down',5);
              Space;
              Button('Bottom',6);
              Space;
              Button('So_rt',7);
            EndGroup;
          Space;
        EndGroup;

        Space;

        HorizGroupEA;
          Space;
          Button('_New...',8);
          Space;
          Button('_Edit...',9);
          Space;
          Button('Co_py',10);
          Space;
          Button('Remove',11);
          Space;
        EndGroup;

        Space;

        HorizGroupEA;
          Space;
          Button('_Save',12);
          Space;
          Button('_Use',13);
          Space;
          Button('_Test',14);
          Space;
          Button('_Cancel',15);
          Space;
        EndGroup;

        Space;

      EndGroup;

      EndProject;

    i := TR_AutoRequest(Triton_App,NIL,@tritontags);
    CleanUp('',0);
end.
