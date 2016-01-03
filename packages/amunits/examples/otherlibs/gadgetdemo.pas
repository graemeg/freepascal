PROGRAM GadgetDemo;

uses exec, triton, tritonmacros, utility, amigalib,amigautils, linklist;

{
   A demo in FPC Pascal using triton.library

   Update for fpc 1.0.7.
   Removed the use of vartags and pas2c.
   09 Jan 2003.

   nils.sjoholm@mailbox.swipnet.se
}



CONST


    NumInList   =  7;
    cyclenum    =  4;

    mxstrings : ARRAY[0..NumInList-1] OF PChar = (
                                        'Amiga 500',
                                        'Amiga 600',
                                        'Amiga 1200',
                                        'Amiga 2000',
                                        'Amiga 3000',
                                        'Amiga 4000',
                                         NIL);

    cyclestrings : ARRAY[0..cyclenum-1] OF PChar = (
                                        'Hallo',
                                        'Moin',
                                        'Tach',
                                         NIL);
CONST

    ButtonGadID      = 1;
    CheckGadID       = 2;
    ScrollGadID      = 3;
    ScrollGadTextID  = 4;
    SlidGadID        = 5;
    SlidGadTextID    = 6;
    CycleGadID       = 7;
    CycleGadTextID   = 8;
    StringGadID      = 9;
    EntryGadID       = 10;
    PassGadID        = 11;
    MxGadID          = 12;
    MxGadTextID      = 13;
    ListGadID        = 14;

VAR
    Triton_App  : pTR_App;
    Project     : pTR_Project;
    trmsg       : pTR_Message;
    quit        : Boolean;
    z           : Longint;
    dummy       : Longint;
    Mylist      : pList;
    MyNode      : pFPCNode;
    i           : Longint;
    GadID       : Longint;

PROCEDURE CleanExit(errstring : STRING; rc : Longint);
BEGIN
    IF assigned(Project) THEN TR_CloseProject(Project);
    if assigned(Triton_App) then TR_DeleteApp(Triton_App);
    if assigned(MyList) then DestroyList(MyList);
    IF errstring <> '' THEN WriteLn(errstring);
    Halt(rc)
END;

BEGIN

    CreateList(MyList);
    FOR i := 0 TO NumInList-2 DO BEGIN
        MyNode := AddNewNode(MyList,mxstrings[i]);
    END;

    Triton_App := TR_CreateAppTags([
                     TRCA_Name,'FPC Pascal Demo',
                     TRCA_LongName,'FPC Pascal Application Demo :)',
                     TRCA_Version,'0.01',
                     TRCA_Info,'Just a test of Triton',
                     TRCA_Release,'1.0',
                     TRCA_Date,'01-05-1996',
                     TAG_DONE]);

    if Triton_App = NIL then CleanExit('Can''t create application',20);

ProjectStart;
    WindowID(1);
    WindowPosition(TRWP_CENTERDISPLAY);
    WindowTitle('Gadgets');
       HorizGroupAC;
          Space;
          VertGroupA;
             Space;
             NamedSeparator('Gadget deactivate');
             Space;
             Button('_Button',ButtonGadID);
             Space;
             HorizGroupSC;
                Space;
                HorizGroup;
                   TextID('_Gadget activ?',CheckGadID);
                   Space;
                   CheckBoxCLEFT(CheckGadID);
                EndGroup;
                Space;
             EndGroup;
             SpaceB;
             NamedSeparator('Pick value');
             Space;
             LineArray;
                BeginLine;
                   TextID('Sc_roller',ScrollGadID);
                   SetTRTag(TRAT_Flags,TROF_RIGHTALIGN);
                   Space;
                   SetTRTag(TROB_Scroller,TROF_HORIZ);
                   SetTRTag(TRSC_Total,40);
                   SetTRTag(TRSC_Visible,10);
                   SetTRTag(TRAT_Value,5);
                   SetTRTag(TRAT_ID,ScrollGadID);
                   Space;
                   ClippedTextBoxMW('5',ScrollGadTextID,2);
                EndLine;
                Space;
                BeginLine;
                   TextID('S_lider',SlidGadID);
                   SetTRTag(TRAT_Flags,TROF_RIGHTALIGN);
                   Space;
                   SliderGadget(1,50,25,SlidGadID);
                   Space;
                   ClippedTextBoxMW('25',SlidGadTextID,2);
                EndLine;
                Space;
                BeginLine;
                   TextID('C_ycle',CycleGadID);
                   SetTRTag(TRAT_Flags,TROF_RIGHTALIGN);
                   Space;
                   CycleGadget(@cyclestrings,0,CycleGadID);
                   Space;
                   ClippedTextBoxMW(cyclestrings[0],CycleGadTextID,5);
                EndLine;
             EndArray;
             SpaceB;
              NamedSeparator('Type some Text');
             Space;
             LineArray;
                BeginLine;
                   TextID('_String',StringGadID);
                   SetTRTag(TRAT_Flags,TROF_RIGHTALIGN);
                   Space;
                   StringGadget('Please change',StringGadID);
                   GetEntryButton(EntryGadID);
                EndLine;
                Space;
                BeginLine;
                   TextID('_Password',PassGadID);
                   SetTRTag(TRAT_Flags,TROF_RIGHTALIGN);
                   Space;
                   PasswordGadget('',PassGadID);
                EndLine;
             EndArray;
             Space;
          EndGroup;
          Space;
          VertSeparator;
          Space;
          VertGroupAC;
             Space;
             NamedSeparatorI('C_hoose',MxGadID);
             Space;
             MXGadget(@mxstrings,4,MxGadID);
             Space;
             ClippedTextBox(mxstrings[4],MxGadTextID);
             SpaceB;
             NamedSeparatorI('D_oubleclick!',ListGadID);
             Space;
             ListSS(Mylist,ListGadID,0,0);
          EndGroup;
          Space;
       EndGroup;
    EndProject;

    Project := TR_OpenProject(Triton_App,@tritontags);

    IF Project = NIL THEN CleanExit('No project',20);

    quit := False;
    WHILE NOT quit DO BEGIN
      dummy := TR_Wait(Triton_App,0);
      REPEAT
        trmsg := TR_GetMsg(Triton_App);
        IF trmsg <> NIL THEN BEGIN
          IF (trmsg^.trm_Project = Project) THEN BEGIN
             CASE trmsg^.trm_Class OF

               TRMS_CLOSEWINDOW :
               BEGIN
                 TR_LockProject(project);
                 dummy := TR_EasyRequest(Triton_App,'%3Sure you want to end this demo?','Yes|No',NIL);
                 TR_UnlockProject(project);
                 IF dummy = 1 THEN quit := True;
               END;

               TRMS_NEWVALUE :
               BEGIN
                 GadID := trmsg^.trm_ID;
                 CASE GadID OF

                   CheckGadID :
                   BEGIN
                     dummy := trmsg^.trm_Data;
                     IF dummy = 1 THEN BEGIN
                        TR_Disable(project,ButtonGadID);
                     END ELSE BEGIN
                        TR_Enable(project,ButtonGadID);
                        TR_SetString(project,StringGadID,'hello test');
                     END;
                   END;

                   ScrollGadID : TR_SetText(project,ScrollGadTextID,LongToStr(trmsg^.trm_Data));

                   SlidGadID   : TR_SetText(project,SlidGadTextID,LongToStr(trmsg^.trm_Data));

                   CycleGadID  : TR_SetText(project,CycleGadTextID,cyclestrings[trmsg^.trm_Data]);

                   StringGadID : ;

                   EntryGadID  : ;

                   PassGadID   : ;

                   MxGadID:
                   BEGIN
                     TR_SetText(project,MxGadTextID,mxstrings[trmsg^.trm_Data]);
                     TR_SetValue(project,ListGadID,(trmsg^.trm_Data));
                   END;

                   ListGadID :
                   BEGIN
                     TR_SetValue(project,MxGadID,(trmsg^.trm_Data));
                     TR_SetText(project,MxGadTextID,mxstrings[trmsg^.trm_Data]);
                   END;
                 END;
               END;

               TRMS_ACTION :
               BEGIN
                 GadID := trmsg^.trm_ID;
                 CASE GadID OF

                   ButtonGadID :
                   begin
                     TR_LockProject(project);
                     TR_EasyRequest(Triton_App,'You clicked on button ' + LongToStr(trmsg^.trm_ID),'_Aha',nil);
                     TR_UnlockProject(Project);
                   END;

                   EntryGadID:
                   BEGIN
                     TR_LockProject(project);
                     z := TR_EasyRequest(Triton_App,TR_GetSTRPTR(project,StringGadID),'OK',NIL);
                     TR_UnlockProject(Project);
                   END;
                 END;
               END;
               ELSE
             END;
           END;
           TR_ReplyMsg(trmsg);
         END;
       UNTIL quit OR (trmsg = NIL);
    END;
    CleanExit('',0);
END.
