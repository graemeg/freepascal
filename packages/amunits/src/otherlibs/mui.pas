
{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2000-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:

    Added MessageBox for error report.
    31 Jul 2000.

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening
    of the library.
    15 Jan 2003.

    Changed integer > smallint.
    Changed cardinal > longword.
    Changed startcode for unit.
    12 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}
{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit mui;

interface

  {

     MUI - MagicUserInterface
     (c) 1993-1997 Stefan Stuntz

     Main Header File


     Class Tree


     rootclass                    (BOOPSI's base class)
     +--Notify                   (implements notification mechanism)
     !  +--Family                (handles multiple children)
     !  !  +--Menustrip          (describes a complete menu strip)
     !  !  +--Menu               (describes a single menu)
     !  !  \--Menuitem           (describes a single menu item)
     !  +--Application           (main class for all applications)
     !  +--Window                (main class for all windows)
     !  !  \--Aboutmui           (About window of MUI preferences)
     !  +--Area                  (base class for all GUI elements)
     !     +--Rectangle          (spacing object)
     !     +--Balance            (balancing separator bar)
     !     +--Image              (image display)
     !     +--Bitmap             (draws bitmaps)
     !     !  \--Bodychunk       (makes bitmap from ILBM body chunk)
     !     +--Text               (text display)
     !     +--Gadget             (base class for intuition gadgets)
     !     !  +--String          (string gadget)
     !     !  +--Boopsi          (interface to BOOPSI gadgets)
     !     !  \--Prop            (proportional gadget)
     !     +--Gauge              (fule gauge)
     !     +--Scale              (percentage scale)
     !     +--Colorfield         (field with changeable color)
     !     +--List               (line-oriented list)
     !     !  +--Floattext       (special list with floating text)
     !     !  +--Volumelist      (special list with volumes)
     !     !  +--Scrmodelist     (special list with screen modes)
     !     !  \--Dirlist         (special list with files)
     !     +--Numeric            (base class for slider gadgets)
     !     !  +--Knob            (turning knob)
     !     !  +--Levelmeter      (level display)
     !     !  +--Numericbutton   (space saving popup slider)
     !     !  \--Slider          (traditional slider)
     !     +--Framedisplay       (private)
     !     !  \--Popframe        (private)
     !     +--Imagedisplay       (private)
     !     !  \--Popimage        (private)
     !     +--Pendisplay         (displays a pen specification)
     !     !  \--Poppen          (popup button to adjust a pen spec)
     !     +--Group              (groups other GUI elements)
     !        +--Mccprefs        (private)
     !        +--Register        (handles page groups with titles)
     !        !  \--Penadjust    (group to adjust a pen)
     !        +--Settingsgroup   (private)
     !        +--Settings        (private)
     !        +--Frameadjust     (private)
     !        +--Imageadjust     (private)
     !        +--Virtgroup       (handles virtual groups)
     !        +--Scrollgroup     (virtual groups with scrollbars)
     !        +--Scrollbar       (traditional scrollbar)
     !        +--Listview        (listview)
     !        +--Radio           (radio button)
     !        +--Cycle           (cycle gadget)
     !        +--Coloradjust     (several gadgets to adjust a color)
     !        +--Palette         (complete palette gadget)
     !        +--Popstring       (base class for popup objects)
     !           +--Popobject    (popup aynthing in a separate window)
     !           !  +--Poplist   (popup a simple listview)
     !           !  \--Popscreen (popup a list of public screens)
     !           \--Popasl       (popup an asl requester)
     +--Semaphore                (semaphore equipped objects)
        +--Applist               (private)
        +--Dataspace             (handles general purpose data spaces)
           \--Configdata         (private)


     General Header File Information


     All macro and structure definitions follow these rules:

     Name                       Meaning

     MUIC_<class>               Name of a class
     MUIM_<class>_<method>      Method
     MUIP_<class>_<method>      Methods parameter structure
     MUIV_<class>_<method>_<x>  Special method value
     MUIA_<class>_<attrib>      Attribute
     MUIV_<class>_<attrib>_<x>  Special attribute value
     MUIE_<error>               Error return code from MUI_Error()
     MUII_<name>                Standard MUI image
     MUIX_<code>                Control codes for text strings
     MUIO_<name>                Object type for MUI_MakeObject()

     MUIA_... attribute definitions are followed by a comment
     consisting of the three possible letters I, S and G.
     I: it's possible to specify this attribute at object creation time.
     S: it's possible to change this attribute with SetAttrs().
     G: it's possible to get this attribute with GetAttr().

     Items marked with "Custom Class" are for use in custom classes only!
   }

uses exec, intuition,utility,graphics,iffparse;


  const
     MUIMASTER_NAME  : PChar = 'muimaster.library';
     MUIMASTER_VMIN = 11;
     MUIMASTER_VLATEST = 19;
  {
     !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     Warning, some of the macros in this header file work only with
     muimaster.library V11 and above. If you recompile your programs,
     be sure to open muimaster.library with MUIMASTER_VMIN as version number.
     !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   }

  {
     Config items for MUIM_GetConfigItem
                                                                           }
     MUICFG_PublicScreen = 36;
  {
     Black box specification structures for images, pens, frames
                                                                           }

  type
     plongword = ^longword;

     tMUI_PenSpec = record
          buf : array[0..31] of char;
       end;
     pMUI_PenSpec = ^tMUI_PenSpec;

  {
     Public Screen Stuff
                                                                           }
  {
     NOTE: This stuff is only included to allow compilation of the supplied
           public screen manager for educational purposes. Everything
           here is subject to change without notice and I guarantee to
           do that just for fun!
           More info can be found in the screen manager source file.
   }

  const
     PSD_INITIAL_NAME : PChar = '(unnamed)';
     PSD_INITIAL_TITLE : PChar = 'MUI Public Screen';



  const
     PSD_NAME_FRONTMOST : PChar = '«Frontmost»';
     PSD_FILENAME_SAVE : PChar = 'envarc:mui/PublicScreens.iff';
     PSD_FILENAME_USE : PChar = 'env:mui/PublicScreens.iff';
     PSD_MAXLEN_NAME = 32;
     PSD_MAXLEN_TITLE = 128;
     PSD_MAXLEN_FONT = 48;
     PSD_MAXLEN_BACKGROUND = 256;
     PSD_NUMCOLS = 8;
     PSD_MAXSYSPENS = 20;
     PSD_NUMSYSPENS = 12;
     PSD_MAXMUIPENS = 10;
     PSD_NUMMUIPENS = 8;

  type
     tMUI_RGBcolor = record
          red : ULONG;
          green : ULONG;
          blue : ULONG;
       end;

     pMUI_RGBColor = ^tMUI_RGBColor;

     tMUI_PubScreenDesc = record
          Version : LONG;
          Name : array[0..(PSD_MAXLEN_NAME)-1] of char;
          Title : array[0..(PSD_MAXLEN_TITLE)-1] of char;
          Font : array[0..(PSD_MAXLEN_FONT)-1] of char;
          Background : array[0..(PSD_MAXLEN_BACKGROUND)-1] of char;
          DisplayID : ULONG;
          DisplayWidth : WORD;
          DisplayHeight : WORD;
          DisplayDepth : BYTE;
          OverscanType : BYTE;
          AutoScroll : BYTE;
          NoDrag : BYTE;
          Exclusive : BYTE;
          Interleaved : BYTE;
          SysDefault : BYTE;
          Behind : BYTE;
          AutoClose : BYTE;
          CloseGadget : BYTE;
          DummyWasForeign : BYTE;
          SystemPens : array[0..(PSD_MAXSYSPENS)-1] of BYTE;
          Reserved : array[0..((1 + (7 * 4)) - PSD_MAXSYSPENS)-1] of BYTE;
          Palette : array[0..(PSD_NUMCOLS)-1] of tMUI_RGBcolor;
          rsvd : array[0..(PSD_MAXSYSPENS - PSD_NUMCOLS)-1] of tMUI_RGBcolor;
          rsvd2 : array[0..(PSD_MAXMUIPENS)-1] of tMUI_PenSpec;
          Changed : LONG;
          UserData : APTR;
       end;
     pMUI_PubScreenDesc = ^tMUI_PubScreenDesc;

     tMUIS_InfoClient = record
          node : tMinNode;
          task : PTask;
          sigbit : ULONG;
       end;
     pMUIS_InfoClient = ^tMUIS_InfoClient;

  {
     Object Types for MUI_MakeObject()
                                                                             }
  { STRPTR label, ULONG flags  }

  const

     MUIO_Label = 1;
  { STRPTR label  }
     MUIO_Button = 2;
  { STRPTR label  }
     MUIO_Checkmark = 3;
  { STRPTR label, STRPTR  entries  }
     MUIO_Cycle = 4;
  { STRPTR label, STRPTR  entries  }
     MUIO_Radio = 5;
  { STRPTR label, LONG min, LONG max  }
     MUIO_Slider = 6;
  { STRPTR label, LONG maxlen  }
     MUIO_String = 7;
  { STRPTR imagespec  }
     MUIO_PopButton = 8;
  { LONG space    }
     MUIO_HSpace = 9;
  { LONG space    }
     MUIO_VSpace = 10;
  { LONG space    }
     MUIO_HBar = 11;
  { LONG space    }
     MUIO_VBar = 12;
  { struct NewMenu  nm, ULONG flags  }
     MUIO_MenustripNM = 13;
  { STRPTR label, STRPTR shortcut, ULONG flags, ULONG data   }
     MUIO_Menuitem = 14;
  { STRPTR label  }
     MUIO_BarTitle = 15;
  { STRPTR label, LONG min, LONG max, STRPTR format  }
     MUIO_NumericButton = 16;
     MUIO_Menuitem_CopyStrings = 1 shl 30;
     MUIO_Label_SingleFrame = 1 shl 8;
     MUIO_Label_DoubleFrame = 1 shl 9;
     MUIO_Label_LeftAligned = 1 shl 10;
     MUIO_Label_Centered = 1 shl 11;
     MUIO_Label_FreeVert = 1 shl 12;
  { check for "localized" menu items such as "O\0Open"  }
     MUIO_MenustripNM_CommandKeyCheck = 1 shl 0;
  {
     ARexx Interface
                                                                             }

  type
     tMUI_Command = record
          mc_Name : Pchar;
          mc_Template : Pchar;
          mc_Parameters : LONG;
          mc_Hook : PHook;
          mc_Reserved : array[0..4] of LONG;
       end;
     pMUI_Command = ^tMUI_Command;



    const
     {  MC_TEMPLATE_ID : PCHar = not(0); }
       MC_TEMPLATE_ID  = -1;
       MUI_RXERR_BADDEFINITION = -(1);
       MUI_RXERR_OUTOFMEMORY = -(2);
       MUI_RXERR_UNKNOWNCOMMAND = -(3);
       MUI_RXERR_BADSYNTAX = -(4);
    {
       Return values for MUI_Error()
                                                                               }
       MUIE_OK = 0;
       MUIE_OutOfMemory = 1;
       MUIE_OutOfGfxMemory = 2;
       MUIE_InvalidWindowObject = 3;
       MUIE_MissingLibrary = 4;
       MUIE_NoARexx = 5;
       MUIE_SingleTask = 6;
    {
       Standard MUI Images & Backgrounds
                                                                               }
    { These images are configured    }
       MUII_WindowBack = 0;
    { with the preferences program.  }
       MUII_RequesterBack = 1;
       MUII_ButtonBack = 2;
       MUII_ListBack = 3;
       MUII_TextBack = 4;
       MUII_PropBack = 5;
       MUII_PopupBack = 6;
       MUII_SelectedBack = 7;
       MUII_ListCursor = 8;
       MUII_ListSelect = 9;
       MUII_ListSelCur = 10;
       MUII_ArrowUp = 11;
       MUII_ArrowDown = 12;
       MUII_ArrowLeft = 13;
       MUII_ArrowRight = 14;
       MUII_CheckMark = 15;
       MUII_RadioButton = 16;
       MUII_Cycle = 17;
       MUII_PopUp = 18;
       MUII_PopFile = 19;
       MUII_PopDrawer = 20;
       MUII_PropKnob = 21;
       MUII_Drawer = 22;
       MUII_HardDisk = 23;
       MUII_Disk = 24;
       MUII_Chip = 25;
       MUII_Volume = 26;
       MUII_RegisterBack = 27;
       MUII_Network = 28;
       MUII_Assign = 29;
       MUII_TapePlay = 30;
       MUII_TapePlayBack = 31;
       MUII_TapePause = 32;
       MUII_TapeStop = 33;
       MUII_TapeRecord = 34;
       MUII_GroupBack = 35;
       MUII_SliderBack = 36;
       MUII_SliderKnob = 37;
       MUII_TapeUp = 38;
       MUII_TapeDown = 39;
       MUII_PageBack = 40;
       MUII_ReadListBack = 41;
       MUII_Count = 42;
    { These are direct color     }
       MUII_BACKGROUND = 128;
    { combinations and are not   }
       MUII_SHADOW = 129;
    { affected by users prefs.   }
       MUII_SHINE = 130;
       MUII_FILL = 131;
    { Generally, you should      }
       MUII_SHADOWBACK = 132;
    { avoid using them. Better   }
       MUII_SHADOWFILL = 133;
    { use one of the customized  }
       MUII_SHADOWSHINE = 134;
    { images above.              }
       MUII_FILLBACK = 135;
       MUII_FILLSHINE = 136;
       MUII_SHINEBACK = 137;
       MUII_FILLBACK2 = 138;
       MUII_HSHINEBACK = 139;
       MUII_HSHADOWBACK = 140;
       MUII_HSHINESHINE = 141;
       MUII_HSHADOWSHADOW = 142;
       MUII_MARKSHINE = 143;
       MUII_MARKHALFSHINE = 144;
       MUII_MARKBACKGROUND = 145;
       MUII_LASTPAT = 145;
    {
       Special values for some methods
                                                                               }
       MUIV_TriggerValue = $49893131;
       MUIV_NotTriggerValue = $49893133;
       MUIV_EveryTime = $49893131;
       MUIV_Notify_Self = 1;
       MUIV_Notify_Window = 2;
       MUIV_Notify_Application = 3;
       MUIV_Notify_Parent = 4;


    const
       MUIV_Application_ReturnID_Quit = -(1);
       MUIV_List_Insert_Top = 0;
       MUIV_List_Insert_Active = -(1);
       MUIV_List_Insert_Sorted = -(2);
       MUIV_List_Insert_Bottom = -(3);
       MUIV_List_Remove_First = 0;
       MUIV_List_Remove_Active = -(1);
       MUIV_List_Remove_Last = -(2);
       MUIV_List_Remove_Selected = -(3);
       MUIV_List_Select_Off = 0;
       MUIV_List_Select_On = 1;
       MUIV_List_Select_Toggle = 2;
       MUIV_List_Select_Ask = 3;
       MUIV_List_GetEntry_Active = -(1);
       MUIV_List_Select_Active = -(1);
       MUIV_List_Select_All = -(2);
       MUIV_List_Redraw_Active = -(1);
       MUIV_List_Redraw_All = -(2);
       MUIV_List_Move_Top = 0;
       MUIV_List_Move_Active = -(1);
       MUIV_List_Move_Bottom = -(2);
    { only valid for second parameter  }
       MUIV_List_Move_Next = -(3);
    { only valid for second parameter  }
       MUIV_List_Move_Previous = -(4);
       MUIV_List_Exchange_Top = 0;
       MUIV_List_Exchange_Active = -(1);
       MUIV_List_Exchange_Bottom = -(2);
    { only valid for second parameter  }
       MUIV_List_Exchange_Next = -(3);
    { only valid for second parameter  }
       MUIV_List_Exchange_Previous = -(4);
       MUIV_List_Jump_Top = 0;
       MUIV_List_Jump_Active = -(1);
       MUIV_List_Jump_Bottom = -(2);
       MUIV_List_Jump_Up = -(4);
       MUIV_List_Jump_Down = -(3);
       MUIV_List_NextSelected_Start = -(1);
       MUIV_List_NextSelected_End = -(1);
       MUIV_DragQuery_Refuse = 0;
       MUIV_DragQuery_Accept = 1;
       MUIV_DragReport_Abort = 0;
       MUIV_DragReport_Continue = 1;
       MUIV_DragReport_Lock = 2;
       MUIV_DragReport_Refresh = 3;
    {
       Control codes for text strings
                                                                               }
    { right justified  }
       MUIX_R : PChar = '\033r';
    { centered         }
       MUIX_C : PChar = '\033c';
    { left justified   }
       MUIX_L : PChar = '\033l';
    { normal      }
       MUIX_N : PChar = '\033n';
    { bold        }
       MUIX_B : PChar = '\033b';
    { italic      }
       MUIX_I : PChar = '\033i';
    { underlined  }
       MUIX_U : PChar = '\033u';
    { text pen            }
       MUIX_PT : PChar = '\0332';
    { highlight text pen  }
       MUIX_PH : PChar = '\0338';
    {
       Parameter structures for some classes
                                                                               }

    type
       tMUI_Palette_Entry = record
            mpe_ID : LONG;
            mpe_Red : ULONG;
            mpe_Green : ULONG;
            mpe_Blue : ULONG;
            mpe_Group : LONG;
         end;
       pMUI_Palette_Entry = ^tMUI_Palette_Entry;

    const
       MUIV_Palette_Entry_End = (-1);
    {                            }
    { Application Input Handler  }
    {                            }
    { see below  }

    type
       tMUI_InputHandlerNode = record
            ihn_Node : tMinNode;
            ihn_Object : pObject_;
            ihn_stuff : record
                case longint of
                   0 : ( ihn_sigs : ULONG );
                   1 : ( ihn_timer : record
                        ihn_millis : WORD;
                        ihn_current : WORD;
                     end );
                end;
            ihn_Flags : ULONG;
            ihn_Method : ULONG;
         end;
    pMUI_InputHandlerNode = ^tMUI_InputHandlerNode;

    const
      { ihn_Signals = ihn_stuff.ihn_sigs;
       ihn_Millis = ihn_stuff.(ihn_timer.ihn_millis);
       ihn_Current = ihn_stuff.(ihn_timer.ihn_current); }
    { Flags for ihn_Flags  }
    { set ihn_Ticks to number of 1/100 sec ticks you want to be triggered  }
       MUIIHNF_TIMER = 1 shl 0;
    {                       }
    { Window Event Handler  }
    {                       }
    { don't touch!  }
    { event handlers are inserted according to their priority.  }
    { certain flags, see below for definitions.  }
    { object which should receive MUIM_HandleEvent.  }
    { if !=NULL, MUIM_HandleEvent is invoked on exactly this class with CoerceMethod().  }
    { one or more IDCMP flags this handler should react on.  }

    type
       tMUI_EventHandlerNode = record
            ehn_Node : tMinNode;
            ehn_Reserved : BYTE;
            ehn_Priority : BYTE;
            ehn_Flags : WORD;
            ehn_Object : pObject_;
            ehn_Class : PIClass;
            ehn_Events : ULONG;
         end;
       pMUI_EventHandlerNode = ^tMUI_EventHandlerNode;
    { flags for ehn_Flags  }

    const
       MUI_EHF_ALWAYSKEYS = 1 shl 0;
    { other values reserved for future use  }
    { return values for MUIM_HandleEvent (bit-masked, all other bits must be 0)  }
    { stop MUI from calling other handlers  }
       MUI_EventHandlerRC_Eat = 1 shl 0;
    {                     }
    { List Position Test  }
    {                     }
    { number of entry, -1 if mouse not over valid entry  }
    { numer of column, -1 if no valid column  }
    { see below  }
    { x offset of mouse click relative to column start  }
    { y offset of mouse click from center of line
                          (negative values mean click was above center,
                           positive values mean click was below center)  }

    type
       tMUI_List_TestPos_Result = record
            entry : LONG;
            column : WORD;
            flags : WORD;
            xoffset : WORD;
            yoffset : WORD;
         end;
       pMUI_List_TestPos_Result = ^tMUI_List_TestPos_Result;

    const
       MUI_LPR_ABOVE = 1 shl 0;
       MUI_LPR_BELOW = 1 shl 1;
       MUI_LPR_LEFT = 1 shl 2;
       MUI_LPR_RIGHT = 1 shl 3;

    {

       For Boopsi Image Implementors Only:

       If MUI is using a boopsi image object, it will send a special method
       immediately after object creation. This method has a parameter structure
       where the boopsi can fill in its minimum and maximum size and learn if
       its used in a horizontal or vertical context.

       The boopsi image must use the method id (MUIM_BoopsiQuery) as return
       value. That's how MUI sees that the method is implemented.

       Note: MUI does not depend on this method. If the boopsi image doesn't
             implement it, minimum size will be 0 and maximum size unlimited.

                                                                               }
    { this is send to the boopsi and  }

    const
       MUIM_BoopsiQuery = $80427157;
    { must be used as return value    }
    { parameter structure  }
    { always MUIM_BoopsiQuery  }
    { obsolete, use mbq_RenderInfo  }
    { read only, see below  }
    { write only, fill in min width   }
    { write only, fill in min height  }
    { write only, fill in max width   }
    { write only, fill in max height  }
    { write only, fill in def width   }
    { write only, fill in def height  }
    { read only, display context  }
    { may grow in future ...  }

 type
       tMUI_RenderInfo = record
            mri_WindowObject : pObject_;
            mri_Screen : PScreen;
            mri_DrawInfo : PDrawInfo;
            mri_Pens : ^WORD;
            mri_Window : PWindow;
            mri_RastPort : PRastPort;
            mri_Flags : ULONG;
         end;
       pMUI_RenderInfo = ^tMUI_RenderInfo;

    type
       tMUI_BoopsiQuery = record
            mbq_MethodID : ULONG;
            mbq_Screen : PScreen;
            mbq_Flags : ULONG;
            mbq_MinWidth : LONG;
            mbq_MinHeight : LONG;
            mbq_MaxWidth : LONG;
            mbq_MaxHeight : LONG;
            mbq_DefWidth : LONG;
            mbq_DefHeight : LONG;
            mbq_RenderInfo : PMUI_RenderInfo;
         end;
       pMUI_BoopsiQuery = ^tMUI_BoopsiQuery;
    { old structure name  }


       MUIP_BoopsiQuery = tMUI_BoopsiQuery;

     const
    { object used in a horizontal  }
       MBQF_HORIZ = 1 shl 0;
    { context (else vertical)      }
    { use this for unlimited MaxWidth/Height  }
       MBQ_MUI_MAXMAX = 10000;
    {                                          }
    { Begin of automatic header file creation  }
    {                                          }
    {                                                                           }
    {  Notify                                                                   }
    {                                                                           }


    const
       MUIC_Notify : PChar = 'Notify.mui';

    { Methods  }
    { V4   }

    const
       MUIM_CallHook = $8042b96b;
    { V12  }
       MUIM_Export = $80420f1c;
    { V8   }
       MUIM_FindUData = $8042c196;
    { V11  }
       MUIM_GetConfigItem = $80423edb;
    { V8   }
       MUIM_GetUData = $8042ed0c;
    { V12  }
       MUIM_Import = $8042d012;
    { V4   }
       MUIM_KillNotify = $8042d240;
    { V16  }
       MUIM_KillNotifyObj = $8042b145;
    { V7   }
       MUIM_MultiSet = $8042d356;
    { V9   }
       MUIM_NoNotifySet = $8042216f;
    { V4   }
       MUIM_Notify = $8042c9cb;
    { V4   }
       MUIM_Set = $8042549a;
    { V4   }
       MUIM_SetAsString = $80422590;
    { V8   }
       MUIM_SetUData = $8042c920;
    { V11  }
       MUIM_SetUDataOnce = $8042ca19;
    { V6   }
       MUIM_WriteLong = $80428d86;
    { V6   }
       MUIM_WriteString = $80424bf4;
    { ...  }

    type
       tMUIP_CallHook = record
            MethodID : ULONG;
            Hook : PHook;
            param1 : ULONG;
         end;
       pMUIP_CallHook = ^tMUIP_CallHook;

       tMUIP_Export = record
            MethodID : ULONG;
            dataspace : pObject_;
         end;
       pMUIP_Export = ^tMUIP_Export;

       tMUIP_FindUData = record
            MethodID : ULONG;
            udata : ULONG;
         end;
       pMUIP_FindUData = ^tMUIP_FindUData;

       tMUIP_GetConfigItem = record
            MethodID : ULONG;
            id : ULONG;
            storage : PULONG;
         end;
       pMUIP_GetConfigItem = ^tMUIP_GetConfigItem;

       tMUIP_GetUData = record
            MethodID : ULONG;
            udata : ULONG;
            attr : ULONG;
            storage : PULONG;
         end;
       pMUIP_GetUData = ^tMUIP_GetUData;

       tMUIP_Import = record
            MethodID : ULONG;
            dataspace : pObject_;
         end;
       pMUIP_Import = ^tMUIP_Import;

       tMUIP_KillNotify = record
            MethodID : ULONG;
            TrigAttr : ULONG;
         end;
       pMUIP_KillNotify = ^tMUIP_KillNotify;

       tMUIP_KillNotifyObj = record
            MethodID : ULONG;
            TrigAttr : ULONG;
            dest : pObject_;
         end;
       pMUIP_KillNotifyObj = ^tMUIP_KillNotifyObj;

    { ...  }
       tMUIP_MultiSet = record
            MethodID : ULONG;
            attr : ULONG;
            val : ULONG;
            obj : APTR;
         end;
       pMUIP_MultiSet = ^tMUIP_MultiSet;

    { ...  }
       tMUIP_NoNotifySet = record
            MethodID : ULONG;
            attr : ULONG;
            format : Pchar;
            val : ULONG;
         end;
       pMUIP_NoNotifySet = ^tMUIP_NoNotifySet;

    { ...  }
       tMUIP_Notify = record
            MethodID : ULONG;
            TrigAttr : ULONG;
            TrigVal : ULONG;
            DestObj : APTR;
            FollowParams : ULONG;
         end;
       pMUIP_Notify = ^tMUIP_Notify;

       tMUIP_Set = record
            MethodID : ULONG;
            attr : ULONG;
            val : ULONG;
         end;
       pMUIP_Set = ^tMUIP_Set;

    { ...  }
       tMUIP_SetAsString = record
            MethodID : ULONG;
            attr : ULONG;
            format : Pchar;
            val : ULONG;
         end;
       pMUIP_SetAsString = ^tMUIP_SetAsString;

       tMUIP_SetUData = record
            MethodID : ULONG;
            udata : ULONG;
            attr : ULONG;
            val : ULONG;
         end;
       pMUIP_SetUData = ^tMUIP_SetUData;

       tMUIP_SetUDataOnce = record
            MethodID : ULONG;
            udata : ULONG;
            attr : ULONG;
            val : ULONG;
         end;
       pMUIP_SetUDataOnce = ^tMUIP_SetUDataOnce;

       tMUIP_WriteLong = record
            MethodID : ULONG;
            val : ULONG;
            memory : PULONG;
         end;
       pMUIP_WriteLong = ^tMUIP_WriteLong;

       tMUIP_WriteString = record
            MethodID : ULONG;
            str : Pchar;
            memory : Pchar;
         end;
       pMUIP_WriteString = ^tMUIP_WriteString;

    { Attributes  }
    { V4  ..g Object             }

    const
       MUIA_ApplicationObject = $8042d3ee;
    { V5  ..g struct AppMessage    }
       MUIA_AppMessage = $80421955;
    { V4  isg LONG               }
       MUIA_HelpLine = $8042a825;
    { V4  isg STRPTR             }
       MUIA_HelpNode = $80420b85;
    { V7  .s. BOOL               }
       MUIA_NoNotify = $804237f9;
    { V11 isg ULONG              }
       MUIA_ObjectID = $8042d76e;
    { V11 ..g Object             }
       MUIA_Parent = $8042e35f;
    { V4  ..g LONG               }
       MUIA_Revision = $80427eaa;
    { V4  isg ULONG              }
       MUIA_UserData = $80420313;
    { V4  ..g LONG               }
       MUIA_Version = $80422301;
    {                                                                           }
    {  Family                                                                   }
    {                                                                           }


    const
       MUIC_Family : PChar = 'Family.mui';

    { Methods  }
    { V8   }

    const
       MUIM_Family_AddHead = $8042e200;
    { V8   }
       MUIM_Family_AddTail = $8042d752;
    { V8   }
       MUIM_Family_Insert = $80424d34;
    { V8   }
       MUIM_Family_Remove = $8042f8a9;
    { V8   }
       MUIM_Family_Sort = $80421c49;
    { V8   }
       MUIM_Family_Transfer = $8042c14a;

    type
       tMUIP_Family_AddHead = record
            MethodID : ULONG;
            obj : pObject_;
         end;
       pMUIP_Family_AddHead = ^tMUIP_Family_AddHead;

       tMUIP_Family_AddTail = record
            MethodID : ULONG;
            obj : pObject_;
         end;
       pMUIP_Family_AddTail = ^tMUIP_Family_AddTail;

       tMUIP_Family_Insert = record
            MethodID : ULONG;
            obj : pObject_;
            pred : pObject_;
         end;
       pMUIP_Family_Insert = ^tMUIP_Family_Insert;

       tMUIP_Family_Remove = record
            MethodID : ULONG;
            obj : pObject_;
         end;
       pMUIP_Family_Remove = ^tMUIP_Family_Remove;

       tMUIP_Family_Sort = record
            MethodID : ULONG;
            obj : array[0..0] of pObject_;
         end;
       pMUIP_Family_Sort = ^tMUIP_Family_Sort;

       tMUIP_Family_Transfer = record
            MethodID : ULONG;
            family : pObject_;
         end;
       pMUIP_Family_Transfer = ^tMUIP_Family_Transfer;

    { Attributes  }
    { V8  i.. Object             }

    const
       MUIA_Family_Child = $8042c696;
    { V8  ..g struct MinList     }
       MUIA_Family_List = $80424b9e;
    {                                                                           }
    {  Menustrip                                                                }
    {                                                                           }


    const
       MUIC_Menustrip : PChar = 'Menustrip.mui';

    { Methods  }
    { Attributes  }
    { V8  isg BOOL               }

    const
       MUIA_Menustrip_Enabled = $8042815b;
    {                                                                           }
    {  Menu                                                                     }
    {                                                                           }


    const
       MUIC_Menu : PChar = 'Menu.mui';

    { Methods  }
    { Attributes  }
    { V8  isg BOOL               }

    const
       MUIA_Menu_Enabled = $8042ed48;
    { V8  isg STRPTR             }
       MUIA_Menu_Title = $8042a0e3;
    {                                                                           }
    {  Menuitem                                                                 }
    {                                                                           }


    const
       MUIC_Menuitem : PChar = 'Menuitem.mui';

    { Methods  }
    { Attributes  }
    { V8  isg BOOL               }

    const
       MUIA_Menuitem_Checked = $8042562a;
    { V8  isg BOOL               }
       MUIA_Menuitem_Checkit = $80425ace;
    { V16 isg BOOL               }
       MUIA_Menuitem_CommandString = $8042b9cc;
    { V8  isg BOOL               }
       MUIA_Menuitem_Enabled = $8042ae0f;
    { V8  isg LONG               }
       MUIA_Menuitem_Exclude = $80420bc6;
    { V8  isg STRPTR             }
       MUIA_Menuitem_Shortcut = $80422030;
    { V8  isg STRPTR             }
       MUIA_Menuitem_Title = $804218be;
    { V8  isg BOOL               }
       MUIA_Menuitem_Toggle = $80424d5c;
    { V8  ..g struct MenuItem    }
       MUIA_Menuitem_Trigger = $80426f32;
       MUIV_Menuitem_Shortcut_Check = -(1);
    {                                                                           }
    {  Application                                                              }
    {                                                                           }


    const
       MUIC_Application : PChar = 'Application.mui';

    { Methods  }
    { V14  }

    const
       MUIM_Application_AboutMUI = $8042d21d;
    { V11  }
       MUIM_Application_AddInputHandler = $8042f099;
    { V11  }
       MUIM_Application_CheckRefresh = $80424d68;

    { MUI_OBSOLETE  }
    { V4   }

    const
       MUIM_Application_InputBuffered = $80427e59;
    { V4   }
       MUIM_Application_Load = $8042f90d;
    { V11  }
       MUIM_Application_NewInput = $80423ba6;
    { V11  }
       MUIM_Application_OpenConfigWindow = $804299ba;
    { V4   }
       MUIM_Application_PushMethod = $80429ef8;
    { V11  }
       MUIM_Application_RemInputHandler = $8042e7af;
    { V4   }
       MUIM_Application_ReturnID = $804276ef;
    { V4   }
       MUIM_Application_Save = $804227ef;
    { V11  }
       MUIM_Application_SetConfigItem = $80424a80;

    { V4   }

    const
       MUIM_Application_ShowHelp = $80426479;

    type
       tMUIP_Application_AboutMUI = record
            MethodID : ULONG;
            refwindow : pObject_;
         end;
       pMUIP_Application_AboutMUI = ^tMUIP_Application_AboutMUI;

       tMUIP_Application_AddInputHandler = record
            MethodID : ULONG;
            ihnode : PMUI_InputHandlerNode;
         end;
       pMUIP_Application_AddInputHandler = ^tMUIP_Application_AddInputHandler;

       tMUIP_Application_CheckRefresh = record
            MethodID : ULONG;
         end;
       pMUIP_Application_CheckRefresh = ^tMUIP_Application_CheckRefresh;

       tMUIP_Application_GetMenuCheck = record
            MethodID : ULONG;
            MenuID : ULONG;
         end;
       pMUIP_Application_GetMenuCheck = ^tMUIP_Application_GetMenuCheck;

       tMUIP_Application_GetMenuState = record
            MethodID : ULONG;
            MenuID : ULONG;
         end;
       pMUIP_Application_GetMenuState = ^tMUIP_Application_GetMenuState;

       tMUIP_Application_Input = record
            MethodID : ULONG;
            signal : PLONGBITS;
         end;
       pMUIP_Application_Input = ^tMUIP_Application_Input;

       tMUIP_Application_InputBuffered = record
            MethodID : ULONG;
         end;
       pMUIP_Application_InputBuffered = ^tMUIP_Application_InputBuffered;

       tMUIP_Application_Load = record
            MethodID : ULONG;
            name : STRPTR;
         end;

       tMUIP_Application_NewInput = record
            MethodID : ULONG;
            signal : PLONGBITS;
         end;
       pMUIP_Application_NewInput = ^tMUIP_Application_NewInput;

       tMUIP_Application_OpenConfigWindow = record
            MethodID : ULONG;
            flags : ULONG;
         end;
       pMUIP_Application_OpenConfigWindow = ^tMUIP_Application_OpenConfigWindow;

    { ...  }
       tMUIP_Application_PushMethod = record
            MethodID : ULONG;
            dest : pObject_;
            count : LONG;
         end;
       pMUIP_Application_PushMethod = ^tMUIP_Application_PushMethod;

       tMUIP_Application_RemInputHandler = record
            MethodID : ULONG;
            ihnode : PMUI_InputHandlerNode;
         end;
       pMUIP_Application_RemInputHandler = ^tMUIP_Application_RemInputHandler;

       tMUIP_Application_ReturnID = record
            MethodID : ULONG;
            retid : ULONG;
         end;
       pMUIP_Application_ReturnID = ^tMUIP_Application_ReturnID;

       tMUIP_Application_Save = record
            MethodID : ULONG;
            name : STRPTR;
         end;

       tMUIP_Application_SetConfigItem = record
            MethodID : ULONG;
            item : ULONG;
            data : APTR;
         end;
       pMUIP_Application_SetConfigItem = ^tMUIP_Application_SetConfigItem;

       tMUIP_Application_SetMenuCheck = record
            MethodID : ULONG;
            MenuID : ULONG;
            stat : LONG;
         end;
       pMUIP_Application_SetMenuCheck = ^tMUIP_Application_SetMenuCheck;

       tMUIP_Application_SetMenuState = record
            MethodID : ULONG;
            MenuID : ULONG;
            stat : LONG;
         end;
       pMUIP_Application_SetMenuState = ^tMUIP_Application_SetMenuState;

       tMUIP_Application_ShowHelp = record
            MethodID : ULONG;
            window : pObject_;
            name : Pchar;
            node : Pchar;
            line : LONG;
         end;
       pMUIP_Application_ShowHelp = ^tMUIP_Application_ShowHelp;

    { Attributes  }
    { V4  isg BOOL               }

    const
       MUIA_Application_Active = $804260ab;
    { V4  i.g STRPTR             }
       MUIA_Application_Author = $80424842;
    { V4  i.g STRPTR             }
       MUIA_Application_Base = $8042e07a;
    { V4  ..g Broker             }
       MUIA_Application_Broker = $8042dbce;
    { V4  isg struct Hook        }
       MUIA_Application_BrokerHook = $80428f4b;
    { V6  ..g struct MsgPort     }
       MUIA_Application_BrokerPort = $8042e0ad;
    { V6  i.g LONG               }
       MUIA_Application_BrokerPri = $8042c8d0;
    { V4  isg struct MUI_Command    }
       MUIA_Application_Commands = $80428648;
    { V4  i.g STRPTR             }
       MUIA_Application_Copyright = $8042ef4d;
    { V4  i.g STRPTR             }
       MUIA_Application_Description = $80421fc6;
    { V4  isg struct DiskObject    }
       MUIA_Application_DiskObject = $804235cb;
    { V4  ..g BOOL               }
       MUIA_Application_DoubleStart = $80423bc6;
    { V5  is. Object             }
       MUIA_Application_DropObject = $80421266;
    { V8  ..g BOOL               }
       MUIA_Application_ForceQuit = $804257df;
    { V8  isg STRPTR             }
       MUIA_Application_HelpFile = $804293f4;
    { V4  .sg BOOL               }
       MUIA_Application_Iconified = $8042a07f;


    const
       MUIA_Application_MenuAction = $80428961;
    { V4  ..g ULONG              }
       MUIA_Application_MenuHelp = $8042540b;
    { V8  i.. Object             }
       MUIA_Application_Menustrip = $804252d9;
    { V7  isg struct Hook        }
       MUIA_Application_RexxHook = $80427c42;
    { V4  ..g struct RxMsg       }
       MUIA_Application_RexxMsg = $8042fd88;
    { V4  .s. STRPTR             }
       MUIA_Application_RexxString = $8042d711;
    { V4  i.. BOOL               }
       MUIA_Application_SingleTask = $8042a2c8;
    { V4  .s. BOOL               }
       MUIA_Application_Sleep = $80425711;
    { V4  i.g STRPTR             }
       MUIA_Application_Title = $804281b8;
    { V10 i.. BOOL               }
       MUIA_Application_UseCommodities = $80425ee5;
    { V10 i.. BOOL               }
       MUIA_Application_UseRexx = $80422387;
    { V4  i.g STRPTR             }
       MUIA_Application_Version = $8042b33f;
    { V4  i.. Object             }
       MUIA_Application_Window = $8042bfe0;
    { V13 ..g struct List        }
       MUIA_Application_WindowList = $80429abe;
       MUIV_Application_Package_NetConnect = $a3ff7b49;
    {                                                                           }
    {  Window                                                                   }
    {                                                                           }

    const
       MUIC_Window : PChar = 'Window.mui';

    { V16  }

    const
       MUIM_Window_AddEventHandler = $804203b7;

    { V16  }

    const
       MUIM_Window_RemEventHandler = $8042679e;
    { V4   }
       MUIM_Window_ScreenToBack = $8042913d;
    { V4   }
       MUIM_Window_ScreenToFront = $804227a4;

    { V11  }

    const
       MUIM_Window_Snapshot = $8042945e;
    { V4   }
       MUIM_Window_ToBack = $8042152e;
    { V4   }
       MUIM_Window_ToFront = $8042554f;

    type
       tMUIP_Window_AddEventHandler = record
            MethodID : ULONG;
            ehnode : PMUI_EventHandlerNode;
         end;
       pMUIP_Window_AddEventHandler = ^tMUIP_Window_AddEventHandler;

       tMUIP_Window_GetMenuCheck = record
            MethodID : ULONG;
            MenuID : ULONG;
         end;
       pMUIP_Window_GetMenuCheck = ^tMUIP_Window_GetMenuCheck;

       tMUIP_Window_GetMenuState = record
            MethodID : ULONG;
            MenuID : ULONG;
         end;
        pMUIP_Window_GetMenuState =  ^tMUIP_Window_GetMenuState;

       tMUIP_Window_RemEventHandler = record
            MethodID : ULONG;
            ehnode : PMUI_EventHandlerNode;
         end;
       pMUIP_Window_RemEventHandler = ^tMUIP_Window_RemEventHandler;

       tMUIP_Window_ScreenToBack = record
            MethodID : ULONG;
         end;
       pMUIP_Window_ScreenToBack = ^tMUIP_Window_ScreenToBack;

       tMUIP_Window_ScreenToFront = record
            MethodID : ULONG;
         end;
       pMUIP_Window_ScreenToFront = ^tMUIP_Window_ScreenToFront;

       tMUIP_Window_SetCycleChain = record
            MethodID : ULONG;
            obj : array[0..0] of pObject_;
         end;
       pMUIP_Window_SetCycleChain = ^tMUIP_Window_SetCycleChain;

       tMUIP_Window_SetMenuCheck = record
            MethodID : ULONG;
            MenuID : ULONG;
            stat : LONG;
         end;
       pMUIP_Window_SetMenuCheck = ^tMUIP_Window_SetMenuCheck;

       tMUIP_Window_SetMenuState = record
            MethodID : ULONG;
            MenuID : ULONG;
            stat : LONG;
         end;
       pMUIP_Window_SetMenuState = ^tMUIP_Window_SetMenuState;

       tMUIP_Window_Snapshot = record
            MethodID : ULONG;
            flags : LONG;
         end;
       pMUIP_Window_Snapshot = ^tMUIP_Window_Snapshot;

       tMUIP_Window_ToBack = record
            MethodID : ULONG;
         end;
       pMUIP_Window_ToBack = ^tMUIP_Window_ToBack;

       tMUIP_Window_ToFront = record
            MethodID : ULONG;
         end;
       pMUIP_Window_ToFront = ^tMUIP_Window_ToFront;

    { Attributes  }
    { V4  isg BOOL               }

    const
       MUIA_Window_Activate = $80428d2f;
    { V4  .sg Object             }
       MUIA_Window_ActiveObject = $80427925;
    { V4  i.g LONG               }
       MUIA_Window_AltHeight = $8042cce3;
    { V4  i.g LONG               }
       MUIA_Window_AltLeftEdge = $80422d65;
    { V4  i.g LONG               }
       MUIA_Window_AltTopEdge = $8042e99b;
    { V4  i.g LONG               }
       MUIA_Window_AltWidth = $804260f4;
    { V5  i.. BOOL               }
       MUIA_Window_AppWindow = $804280cf;
    { V4  i.. BOOL               }
       MUIA_Window_Backdrop = $8042c0bb;
    { V4  i.. BOOL               }
       MUIA_Window_Borderless = $80429b79;
    { V4  i.. BOOL               }
       MUIA_Window_CloseGadget = $8042a110;
    { V4  ..g BOOL               }
       MUIA_Window_CloseRequest = $8042e86e;
    { V4  isg Object             }
       MUIA_Window_DefaultObject = $804294d7;
    { V4  i.. BOOL               }
       MUIA_Window_DepthGadget = $80421923;
    { V4  i.. BOOL               }
       MUIA_Window_DragBar = $8042045d;
    { V8  isg BOOL               }
       MUIA_Window_FancyDrawing = $8042bd0e;
    { V4  i.g LONG               }
       MUIA_Window_Height = $80425846;
    { V4  isg ULONG              }
       MUIA_Window_ID = $804201bd;
    { V4  ..g struct InputEvent    }
       MUIA_Window_InputEvent = $804247d8;
    { V4  isg BOOL               }
       MUIA_Window_IsSubWindow = $8042b5aa;
    { V4  i.g LONG               }
       MUIA_Window_LeftEdge = $80426c65;
    { MUI_OBSOLETE  }
    { V8  isg ULONG              }

    const
       MUIA_Window_MenuAction = $80427521;
    { V8  i.g Object             }
       MUIA_Window_Menustrip = $8042855e;
    { V10 ..g Object             }
       MUIA_Window_MouseObject = $8042bf9b;
    { V10 i.. BOOL               }
       MUIA_Window_NeedsMouseObject = $8042372a;
    { V4  is. BOOL               }
       MUIA_Window_NoMenus = $80429df5;
    { V4  .sg BOOL               }
       MUIA_Window_Open = $80428aa0;
    { V6  isg STRPTR             }
       MUIA_Window_PublicScreen = $804278e4;
    { V4  is. Object             }
       MUIA_Window_RefWindow = $804201f4;
    { V4  isg Object             }
       MUIA_Window_RootObject = $8042cba5;
    { V4  isg struct Screen      }
       MUIA_Window_Screen = $8042df4f;
    { V5  isg STRPTR             }
       MUIA_Window_ScreenTitle = $804234b0;
    { V4  i.. BOOL               }
       MUIA_Window_SizeGadget = $8042e33d;
    { V4  i.. BOOL               }
       MUIA_Window_SizeRight = $80424780;
    { V4  .sg BOOL               }
       MUIA_Window_Sleep = $8042e7db;
    { V4  isg STRPTR             }
       MUIA_Window_Title = $8042ad3d;
    { V4  i.g LONG               }
       MUIA_Window_TopEdge = $80427c66;
    { V13 isg BOOL               }
       MUIA_Window_UseBottomBorderScroller = $80424e79;
    { V13 isg BOOL               }
       MUIA_Window_UseLeftBorderScroller = $8042433e;
    { V13 isg BOOL               }
       MUIA_Window_UseRightBorderScroller = $8042c05e;
    { V4  i.g LONG               }
       MUIA_Window_Width = $8042dcae;
    { V4  ..g struct Window      }
       MUIA_Window_Window = $80426a42;
       MUIV_Window_ActiveObject_None = 0;
       MUIV_Window_ActiveObject_Next = -(1);
       MUIV_Window_ActiveObject_Prev = -(2);


    const
       MUIV_Window_AltHeight_Scaled = -(1000);
       MUIV_Window_AltLeftEdge_Centered = -(1);
       MUIV_Window_AltLeftEdge_Moused = -(2);
       MUIV_Window_AltLeftEdge_NoChange = -(1000);
       MUIV_Window_AltTopEdge_Centered = -(1);
       MUIV_Window_AltTopEdge_Moused = -(2);


    const
       MUIV_Window_AltTopEdge_NoChange = -(1000);


    const
       MUIV_Window_AltWidth_Scaled = -(1000);



    const
       MUIV_Window_Height_Scaled = -(1000);
       MUIV_Window_Height_Default = -(1001);
       MUIV_Window_LeftEdge_Centered = -(1);
       MUIV_Window_LeftEdge_Moused = -(2);


    const
       MUIV_Window_TopEdge_Centered = -(1);
       MUIV_Window_TopEdge_Moused = -(2);


    const
       MUIV_Window_Width_Scaled = -(1000);
       MUIV_Window_Width_Default = -(1001);
    {                                                                           }
    {  Aboutmui                                                                 }
    {                                                                           }


    const
       MUIC_Aboutmui : PChar = 'Aboutmui.mui';

    { Methods  }
    { Attributes  }
    { V11 i.. Object             }

    const
       MUIA_Aboutmui_Application = $80422523;
    {                                                                           }
    {  Area                                                                     }
    {                                                                           }


    const
       MUIC_Area : PChar = 'Area.mui';

    { Methods  }
    { Custom Class  }
    { V4   }

    const
       MUIM_AskMinMax = $80423874;
    { Custom Class  }
    { V4   }
       MUIM_Cleanup = $8042d985;
    { V11  }
       MUIM_ContextMenuBuild = $80429d2e;
    { V11  }
       MUIM_ContextMenuChoice = $80420f0e;
    { V18  }
       MUIM_CreateBubble = $80421c41;
    { V11  }
       MUIM_CreateShortHelp = $80428e93;
    { V18  }
       MUIM_DeleteBubble = $804211af;
    { V11  }
       MUIM_DeleteShortHelp = $8042d35a;
    { V11  }
       MUIM_DragBegin = $8042c03a;
    { V11  }
       MUIM_DragDrop = $8042c555;
    { V11  }
       MUIM_DragFinish = $804251f0;
    { V11  }
       MUIM_DragQuery = $80420261;
    { V11  }
       MUIM_DragReport = $8042edad;
    { Custom Class  }
    { V4   }
       MUIM_Draw = $80426f3f;
    { V11  }
       MUIM_DrawBackground = $804238ca;
    { Custom Class  }
    { V16  }
       MUIM_HandleEvent = $80426d66;
    { Custom Class  }
    { V4   }
       MUIM_HandleInput = $80422a1a;
    { Custom Class  }
    { V4   }
       MUIM_Hide = $8042f20f;
    { Custom Class  }
    { V4   }
       MUIM_Setup = $80428354;
    { Custom Class  }
    { V4   }
       MUIM_Show = $8042cc84;

    type

     { MUI_MinMax structure holds information about minimum, maximum
       and default dimensions of an object.  }
       tMUI_MinMax = record
            MinWidth : WORD;
            MinHeight : WORD;
            MaxWidth : WORD;
            MaxHeight : WORD;
            DefWidth : WORD;
            DefHeight : WORD;
         end;
       pMUI_MinMax = ^tMUI_MinMax;

       tMUIP_AskMinMax = record
            MethodID : ULONG;
            MinMaxInfo : PMUI_MinMax;
         end;
       pMUIP_AskMinMax = ^tMUIP_AskMinMax;

    { Custom Class  }
       tMUIP_Cleanup = record
            MethodID : ULONG;
         end;
       pMUIP_Cleanup  = ^tMUIP_Cleanup;

    { Custom Class  }
       tMUIP_ContextMenuBuild = record
            MethodID : ULONG;
            mx : LONG;
            my : LONG;
         end;
       pMUIP_ContextMenuBuild = ^tMUIP_ContextMenuBuild;

       tMUIP_ContextMenuChoice = record
            MethodID : ULONG;
            item : pObject_;
         end;
       pMUIP_ContextMenuChoice = ^tMUIP_ContextMenuChoice;

       tMUIP_CreateBubble = record
            MethodID : ULONG;
            x : LONG;
            y : LONG;
            txt : Pchar;
            flags : ULONG;
         end;
       pMUIP_CreateBubble = ^tMUIP_CreateBubble;

       tMUIP_CreateShortHelp = record
            MethodID : ULONG;
            mx : LONG;
            my : LONG;
         end;
       pMUIP_CreateShortHelp = ^tMUIP_CreateShortHelp;

       tMUIP_DeleteBubble = record
            MethodID : ULONG;
            bubble : APTR;
         end;
       pMUIP_DeleteBubble = ^tMUIP_DeleteBubble;

       tMUIP_DeleteShortHelp = record
            MethodID : ULONG;
            help : STRPTR;
         end;
       pMUIP_DeleteShortHelp = ^tMUIP_DeleteShortHelp;

       tMUIP_DragBegin = record
            MethodID : ULONG;
            obj : pObject_;
         end;
       pMUIP_DragBegin = ^tMUIP_DragBegin;

       tMUIP_DragDrop = record
            MethodID : ULONG;
            obj : pObject_;
            x : LONG;
            y : LONG;
         end;
       pMUIP_DragDrop = ^tMUIP_DragDrop;

       tMUIP_DragFinish = record
            MethodID : ULONG;
            obj : pObject_;
         end;
       pMUIP_DragFinish = ^tMUIP_DragFinish;

       tMUIP_DragQuery = record
            MethodID : ULONG;
            obj : pObject_;
         end;
       pMUIP_DragQuery = ^tMUIP_DragQuery;

       tMUIP_DragReport = record
            MethodID : ULONG;
            obj : pObject_;
            x : LONG;
            y : LONG;
            update : LONG;
         end;
       pMUIP_DragReport = ^tMUIP_DragReport;

       tMUIP_Draw = record
            MethodID : ULONG;
            flags : ULONG;
         end;
       pMUIP_Draw = ^tMUIP_Draw;

    { Custom Class  }
       tMUIP_DrawBackground = record
            MethodID : ULONG;
            left : LONG;
            top : LONG;
            width : LONG;
            height : LONG;
            xoffset : LONG;
            yoffset : LONG;
            flags : LONG;
         end;
       pMUIP_DrawBackground = ^tMUIP_DrawBackground;

       tMUIP_HandleEvent = record
            MethodID : ULONG;
            imsg : PIntuiMessage;
            muikey : LONG;
         end;
       pMUIP_HandleEvent = ^tMUIP_HandleEvent;

    { Custom Class  }
       tMUIP_HandleInput = record
            MethodID : ULONG;
            imsg : PIntuiMessage;
            muikey : LONG;
         end;
       pMUIP_HandleInput = ^tMUIP_HandleInput;

    { Custom Class  }
       tMUIP_Hide = record
            MethodID : ULONG;
         end;
       pMUIP_Hide = ^tMUIP_Hide;

    { Custom Class  }
       tMUIP_Setup = record
            MethodID : ULONG;
            RenderInfo : PMUI_RenderInfo;
         end;
       pMUIP_Setup = ^tMUIP_Setup;

    { Custom Class  }
       tMUIP_Show = record
            MethodID : ULONG;
         end;
       pMUIP_Show = ^tMUIP_Show;

    { Custom Class  }
    { Attributes  }
    { V4  is. LONG               }

    const
       MUIA_Background = $8042545b;
    { V4  ..g LONG               }
       MUIA_BottomEdge = $8042e552;
    { V11 isg Object             }
       MUIA_ContextMenu = $8042b704;
    { V11 ..g Object             }
       MUIA_ContextMenuTrigger = $8042a2c1;
    { V4  isg char               }
       MUIA_ControlChar = $8042120b;
    { V11 isg LONG               }
       MUIA_CycleChain = $80421ce7;
    { V4  isg BOOL               }
       MUIA_Disabled = $80423661;
    { V11 isg BOOL               }
       MUIA_Draggable = $80420b6e;
    { V11 isg BOOL               }
       MUIA_Dropable = $8042fbce;

    { V4  is. BOOL               }

    const
       MUIA_FillArea = $804294a3;
    { V4  i.. LONG               }
       MUIA_FixHeight = $8042a92b;
    { V4  i.. STRPTR             }
       MUIA_FixHeightTxt = $804276f2;
    { V4  i.. LONG               }
       MUIA_FixWidth = $8042a3f1;
    { V4  i.. STRPTR             }
       MUIA_FixWidthTxt = $8042d044;
    { V4  i.g struct TextFont    }
       MUIA_Font = $8042be50;
    { V4  i.. LONG               }
       MUIA_Frame = $8042ac64;
    { V4  i.. BOOL               }
       MUIA_FramePhantomHoriz = $8042ed76;
    { V4  i.. STRPTR             }
       MUIA_FrameTitle = $8042d1c7;
    { V4  ..g LONG               }
       MUIA_Height = $80423237;
    { V11 isg LONG               }
       MUIA_HorizDisappear = $80429615;
    { V4  isg WORD               }
       MUIA_HorizWeight = $80426db9;
    { V4  i.g LONG               }
       MUIA_InnerBottom = $8042f2c0;
    { V4  i.g LONG               }
       MUIA_InnerLeft = $804228f8;
    { V4  i.g LONG               }
       MUIA_InnerRight = $804297ff;
    { V4  i.g LONG               }
       MUIA_InnerTop = $80421eb6;
    { V4  i.. LONG               }
       MUIA_InputMode = $8042fb04;
    { V4  ..g LONG               }
       MUIA_LeftEdge = $8042bec6;
    { V11 i.. LONG               }
       MUIA_MaxHeight = $804293e4;
    { V11 i.. LONG               }
       MUIA_MaxWidth = $8042f112;
    { V4  ..g BOOL               }
       MUIA_Pressed = $80423535;
    { V4  ..g LONG               }
       MUIA_RightEdge = $8042ba82;
    { V4  isg BOOL               }
       MUIA_Selected = $8042654b;
    { V11 isg STRPTR             }
       MUIA_ShortHelp = $80428fe3;
    { V4  isg BOOL               }
       MUIA_ShowMe = $80429ba8;
    { V4  i.. BOOL               }
       MUIA_ShowSelState = $8042caac;
    { V4  ..g LONG               }
       MUIA_Timer = $80426435;
    { V4  ..g LONG               }
       MUIA_TopEdge = $8042509b;
    { V11 isg LONG               }
       MUIA_VertDisappear = $8042d12f;
    { V4  isg WORD               }
       MUIA_VertWeight = $804298d0;
    { V4  i.. WORD               }
       MUIA_Weight = $80421d1f;
    { V4  ..g LONG               }
       MUIA_Width = $8042b59c;
    { V4  ..g struct Window      }
       MUIA_Window = $80421591;
    { V4  ..g Object             }
       MUIA_WindowObject = $8042669e;
       MUIV_Font_Inherit = 0;
       MUIV_Font_Normal = -(1);
       MUIV_Font_List = -(2);
       MUIV_Font_Tiny = -(3);
       MUIV_Font_Fixed = -(4);
       MUIV_Font_Title = -(5);
       MUIV_Font_Big = -(6);
       MUIV_Font_Button = -(7);
       MUIV_Frame_None = 0;
       MUIV_Frame_Button = 1;
       MUIV_Frame_ImageButton = 2;
       MUIV_Frame_Text = 3;
       MUIV_Frame_String = 4;
       MUIV_Frame_ReadList = 5;
       MUIV_Frame_InputList = 6;
       MUIV_Frame_Prop = 7;
       MUIV_Frame_Gauge = 8;
       MUIV_Frame_Group = 9;
       MUIV_Frame_PopUp = 10;
       MUIV_Frame_Virtual = 11;
       MUIV_Frame_Slider = 12;
       MUIV_Frame_Count = 13;
       MUIV_InputMode_None = 0;
       MUIV_InputMode_RelVerify = 1;
       MUIV_InputMode_Immediate = 2;
       MUIV_InputMode_Toggle = 3;
    {                                                                           }
    {  Rectangle                                                                }
    {                                                                           }


    const
       MUIC_Rectangle : PChar = 'Rectangle.mui';

    { Attributes  }
    { V11 i.g STRPTR             }

    const
       MUIA_Rectangle_BarTitle = $80426689;
    { V7  i.g BOOL               }
       MUIA_Rectangle_HBar = $8042c943;
    { V7  i.g BOOL               }
       MUIA_Rectangle_VBar = $80422204;
    {                                                                           }
    {  Balance                                                                  }
    {                                                                           }


    const
       MUIC_Balance : PChar = 'Balance.mui';

    {                                                                           }
    {  Image                                                                    }
    {                                                                           }


    const
       MUIC_Image : PChar = 'Image.mui';

    { Attributes  }
    { V4  i.. BOOL               }

    const
       MUIA_Image_FontMatch = $8042815d;
    { V4  i.. BOOL               }
       MUIA_Image_FontMatchHeight = $80429f26;
    { V4  i.. BOOL               }
       MUIA_Image_FontMatchWidth = $804239bf;
    { V4  i.. BOOL               }
       MUIA_Image_FreeHoriz = $8042da84;
    { V4  i.. BOOL               }
       MUIA_Image_FreeVert = $8042ea28;
    { V4  i.. struct Image       }
       MUIA_Image_OldImage = $80424f3d;
    { V4  i.. char               }
       MUIA_Image_Spec = $804233d5;
    { V4  is. LONG               }
       MUIA_Image_State = $8042a3ad;
    {                                                                           }
    {  Bitmap                                                                   }
    {                                                                           }


    const
       MUIC_Bitmap : PChar = 'Bitmap.mui';

    { Attributes  }
    { V8  isg struct BitMap      }

    const
       MUIA_Bitmap_Bitmap = $804279bd;
    { V8  isg LONG               }
       MUIA_Bitmap_Height = $80421560;
    { V8  isg UBYTE              }
       MUIA_Bitmap_MappingTable = $8042e23d;
    { V11 isg LONG               }
       MUIA_Bitmap_Precision = $80420c74;
    { V11 ..g struct BitMap      }
       MUIA_Bitmap_RemappedBitmap = $80423a47;
    { V8  isg ULONG              }
       MUIA_Bitmap_SourceColors = $80425360;
    { V8  isg LONG               }
       MUIA_Bitmap_Transparent = $80422805;
    { V11 i.. BOOL               }
       MUIA_Bitmap_UseFriend = $804239d8;
    { V8  isg LONG               }
       MUIA_Bitmap_Width = $8042eb3a;
    {                                                                           }
    {  Bodychunk                                                                }
    {                                                                           }


    const
       MUIC_Bodychunk : PChar = 'Bodychunk.mui';

    { Attributes  }
    { V8  isg UBYTE              }

    const
       MUIA_Bodychunk_Body = $8042ca67;
    { V8  isg UBYTE              }
       MUIA_Bodychunk_Compression = $8042de5f;
    { V8  isg LONG               }
       MUIA_Bodychunk_Depth = $8042c392;
    { V8  isg UBYTE              }
       MUIA_Bodychunk_Masking = $80423b0e;
    {                                                                           }
    {  Text                                                                     }
    {                                                                           }

    const
       MUIC_Text : PChar = 'Text.mui';

    { Attributes  }
    { V4  isg STRPTR             }

    const
       MUIA_Text_Contents = $8042f8dc;
    { V4  i.. char               }
       MUIA_Text_HiChar = $804218ff;
    { V4  isg STRPTR             }
       MUIA_Text_PreParse = $8042566d;
    { V4  i.. BOOL               }
       MUIA_Text_SetMax = $80424d0a;
    { V4  i.. BOOL               }
       MUIA_Text_SetMin = $80424e10;
    { V11 i.. BOOL               }
       MUIA_Text_SetVMax = $80420d8b;
    {                                                                           }
    {  Gadget                                                                   }
    {                                                                           }

    const
       MUIC_Gadget : PChar = 'Gadget.mui';

    { Attributes  }
    { V11 ..g struct Gadget      }

    const
       MUIA_Gadget_Gadget = $8042ec1a;
    {                                                                           }
    {  String                                                                   }
    {                                                                           }


    const
       MUIC_String : PChar = 'String.mui';

    { Methods  }
    { Attributes  }
    { V4  isg STRPTR             }

    const
       MUIA_String_Accept = $8042e3e1;
    { V4  ..g STRPTR             }
       MUIA_String_Acknowledge = $8042026c;
    { V11 isg BOOL               }
       MUIA_String_AdvanceOnCR = $804226de;
    { V4  isg Object             }
       MUIA_String_AttachedList = $80420fd2;
    { V4  .sg LONG               }
       MUIA_String_BufferPos = $80428b6c;
    { V4  isg STRPTR             }
       MUIA_String_Contents = $80428ffd;
    { V4  .sg LONG               }
       MUIA_String_DisplayPos = $8042ccbf;
    { V7  isg struct Hook        }
       MUIA_String_EditHook = $80424c33;
    { V4  i.g LONG               }
       MUIA_String_Format = $80427484;
    { V4  isg ULONG              }
       MUIA_String_Integer = $80426e8a;
    { V11 isg BOOL               }
       MUIA_String_LonelyEditHook = $80421569;
    { V4  i.g LONG               }
       MUIA_String_MaxLen = $80424984;
    { V4  isg STRPTR             }
       MUIA_String_Reject = $8042179c;
    { V4  i.g BOOL               }
       MUIA_String_Secret = $80428769;
       MUIV_String_Format_Left = 0;
       MUIV_String_Format_Center = 1;
       MUIV_String_Format_Right = 2;
    {                                                                           }
    {  Boopsi                                                                   }
    {                                                                           }


    const
       MUIC_Boopsi : PChar = 'Boopsi.mui';

    { Attributes  }
    { V4  isg struct IClass      }

    const
       MUIA_Boopsi_Class = $80426999;
    { V4  isg char               }
       MUIA_Boopsi_ClassID = $8042bfa3;
    { V4  isg ULONG              }
       MUIA_Boopsi_MaxHeight = $8042757f;
    { V4  isg ULONG              }
       MUIA_Boopsi_MaxWidth = $8042bcb1;
    { V4  isg ULONG              }
       MUIA_Boopsi_MinHeight = $80422c93;
    { V4  isg ULONG              }
       MUIA_Boopsi_MinWidth = $80428fb2;
    { V4  ..g Object             }
       MUIA_Boopsi_Object = $80420178;
    { V4  i.. ULONG              }
       MUIA_Boopsi_Remember = $8042f4bd;
    { V9  i.. BOOL               }
       MUIA_Boopsi_Smart = $8042b8d7;
    { V4  isg ULONG              }
       MUIA_Boopsi_TagDrawInfo = $8042bae7;
    { V4  isg ULONG              }
       MUIA_Boopsi_TagScreen = $8042bc71;
    { V4  isg ULONG              }
       MUIA_Boopsi_TagWindow = $8042e11d;
    {                                                                           }
    {  Prop                                                                     }
    {                                                                           }


    const
       MUIC_Prop : PChar = 'Prop.mui';

    { Methods  }
    { V16  }

    const
       MUIM_Prop_Decrease = $80420dd1;
    { V16  }
       MUIM_Prop_Increase = $8042cac0;

    type
       tMUIP_Prop_Decrease = record
            MethodID : ULONG;
            amount : LONG;
         end;
       pMUIP_Prop_Decrease = ^tMUIP_Prop_Decrease;

       tMUIP_Prop_Increase = record
            MethodID : ULONG;
            amount : LONG;
         end;
       pMUIP_Prop_Increase = ^tMUIP_Prop_Increase;

    { Attributes  }
    { V4  isg LONG               }

    const
       MUIA_Prop_Entries = $8042fbdb;
    { V4  isg LONG               }
       MUIA_Prop_First = $8042d4b2;
    { V4  i.g BOOL               }
       MUIA_Prop_Horiz = $8042f4f3;
    { V4  isg BOOL               }
       MUIA_Prop_Slider = $80429c3a;
    { V13 i.. LONG               }
       MUIA_Prop_UseWinBorder = $8042deee;
    { V4  isg LONG               }
       MUIA_Prop_Visible = $8042fea6;
       MUIV_Prop_UseWinBorder_None = 0;
       MUIV_Prop_UseWinBorder_Left = 1;
       MUIV_Prop_UseWinBorder_Right = 2;
       MUIV_Prop_UseWinBorder_Bottom = 3;
    {                                                                           }
    {  Gauge                                                                    }
    {                                                                           }


    const
       MUIC_Gauge : PChar = 'Gauge.mui';

    { Attributes  }
    { V4  isg LONG               }

    const
       MUIA_Gauge_Current = $8042f0dd;
    { V4  isg BOOL               }
       MUIA_Gauge_Divide = $8042d8df;
    { V4  i.. BOOL               }
       MUIA_Gauge_Horiz = $804232dd;
    { V7  isg STRPTR             }
       MUIA_Gauge_InfoText = $8042bf15;
    { V4  isg LONG               }
       MUIA_Gauge_Max = $8042bcdb;
    {                                                                           }
    {  Scale                                                                    }
    {                                                                           }


    const
       MUIC_Scale : PChar = 'Scale.mui';

    { Attributes  }
    { V4  isg BOOL               }

    const
       MUIA_Scale_Horiz = $8042919a;
    {                                                                           }
    {  Colorfield                                                               }
    {                                                                           }


    const
       MUIC_Colorfield : PChar = 'Colorfield.mui';

    { Attributes  }
    { V4  isg ULONG              }

    const
       MUIA_Colorfield_Blue = $8042d3b0;
    { V4  isg ULONG              }
       MUIA_Colorfield_Green = $80424466;
    { V4  ..g ULONG              }
       MUIA_Colorfield_Pen = $8042713a;
    { V4  isg ULONG              }
       MUIA_Colorfield_Red = $804279f6;
    { V4  isg ULONG              }
       MUIA_Colorfield_RGB = $8042677a;
    {                                                                           }
    {  List                                                                     }
    {                                                                           }


    const
       MUIC_List : PChar = 'List.mui';

    { Methods  }
    { V4   }

    const
       MUIM_List_Clear = $8042ad89;
    { V11  }
       MUIM_List_CreateImage = $80429804;
    { V11  }
       MUIM_List_DeleteImage = $80420f58;
    { V4   }
       MUIM_List_Exchange = $8042468c;
    { V4   }
       MUIM_List_GetEntry = $804280ec;
    { V4   }
       MUIM_List_Insert = $80426c87;
    { V7   }
       MUIM_List_InsertSingle = $804254d5;
    { V4   }
       MUIM_List_Jump = $8042baab;
    { V9   }
       MUIM_List_Move = $804253c2;
    { V6   }
       MUIM_List_NextSelected = $80425f17;
    { V4   }
       MUIM_List_Redraw = $80427993;
    { V4   }
       MUIM_List_Remove = $8042647e;
    { V4   }
       MUIM_List_Select = $804252d8;
    { V4   }
       MUIM_List_Sort = $80422275;
    { V11  }
       MUIM_List_TestPos = $80425f48;

    type
       tMUIP_List_Clear = record
            MethodID : ULONG;
         end;
       pMUIP_List_Clear = ^tMUIP_List_Clear;

       tMUIP_List_CreateImage = record
            MethodID : ULONG;
            obj : pObject_;
            flags : ULONG;
         end;
       pMUIP_List_CreateImage = ^tMUIP_List_CreateImage;

       tMUIP_List_DeleteImage = record
            MethodID : ULONG;
            listimg : APTR;
         end;
       pMUIP_List_DeleteImage = ^tMUIP_List_DeleteImage;

       tMUIP_List_Exchange = record
            MethodID : ULONG;
            pos1 : LONG;
            pos2 : LONG;
         end;
       pMUIP_List_Exchange = ^tMUIP_List_Exchange;

       tMUIP_List_GetEntry = record
            MethodID : ULONG;
            pos : LONG;
            entry : PAPTR;
         end;
       pMUIP_List_GetEntry = ^tMUIP_List_GetEntry;

       tMUIP_List_Insert = record
            MethodID : ULONG;
            entries : PAPTR;
            count : LONG;
            pos : LONG;
         end;
       pMUIP_List_Insert = ^tMUIP_List_Insert;

       tMUIP_List_InsertSingle = record
            MethodID : ULONG;
            entry : APTR;
            pos : LONG;
         end;
       pMUIP_List_InsertSingle = ^tMUIP_List_InsertSingle;

       tMUIP_List_Jump = record
            MethodID : ULONG;
            pos : LONG;
         end;
       pMUIP_List_Jump = ^tMUIP_List_Jump;

       tMUIP_List_Move = record
            MethodID : ULONG;
            from : LONG;
            too : LONG;
         end;
       pMUIP_List_Move = ^tMUIP_List_Move;

       tMUIP_List_NextSelected = record
            MethodID : ULONG;
            pos : PLONG;
         end;
       pMUIP_List_NextSelected = ^tMUIP_List_NextSelected;

       tMUIP_List_Redraw = record
            MethodID : ULONG;
            pos : LONG;
         end;
       pMUIP_List_Redraw = ^tMUIP_List_Redraw;

       tMUIP_List_Remove = record
            MethodID : ULONG;
            pos : LONG;
         end;
       pMUIP_List_Remove = ^tMUIP_List_Remove;

       tMUIP_List_Select = record
            MethodID : ULONG;
            pos : LONG;
            seltype : LONG;
            state : PLONG;
         end;
       pMUIP_List_Select = ^tMUIP_List_Select;

       tMUIP_List_Sort = record
            MethodID : ULONG;
         end;
       pMUIP_List_Sort = ^tMUIP_List_Sort;

       tMUIP_List_TestPos = record
            MethodID : ULONG;
            x : LONG;
            y : LONG;
            res : PMUI_List_TestPos_Result;
         end;
       pMUIP_List_TestPos = ^tMUIP_List_TestPos;

    { Attributes  }
    { V4  isg LONG               }

    const
       MUIA_List_Active = $8042391c;
    { V4  i.. BOOL               }
       MUIA_List_AdjustHeight = $8042850d;
    { V4  i.. BOOL               }
       MUIA_List_AdjustWidth = $8042354a;
    { V11 isg BOOL               }
       MUIA_List_AutoVisible = $8042a445;
    { V4  is. struct Hook        }
       MUIA_List_CompareHook = $80425c14;
    { V4  is. struct Hook        }
       MUIA_List_ConstructHook = $8042894f;
    { V4  is. struct Hook        }
       MUIA_List_DestructHook = $804297ce;
    { V4  is. struct Hook        }
       MUIA_List_DisplayHook = $8042b4d5;
    { V11 isg BOOL               }
       MUIA_List_DragSortable = $80426099;
    { V11 ..g LONG               }
       MUIA_List_DropMark = $8042aba6;
    { V4  ..g LONG               }
       MUIA_List_Entries = $80421654;
    { V4  ..g LONG               }
       MUIA_List_First = $804238d4;
    { V4  isg STRPTR             }
       MUIA_List_Format = $80423c0a;
    { V9  ..g LONG               }
       MUIA_List_InsertPosition = $8042d0cd;
    { V4  i.. LONG               }
       MUIA_List_MinLineHeight = $8042d1c3;
    { V4  is. struct Hook        }
       MUIA_List_MultiTestHook = $8042c2c6;
    { V13 i.. APTR               }
       MUIA_List_Pool = $80423431;
    { V13 i.. ULONG              }
       MUIA_List_PoolPuddleSize = $8042a4eb;
    { V13 i.. ULONG              }
       MUIA_List_PoolThreshSize = $8042c48c;
    { V4  .s. BOOL               }
       MUIA_List_Quiet = $8042d8c7;
    { V11 isg BOOL               }
       MUIA_List_ShowDropMarks = $8042c6f3;
    { V4  i.. APTR               }
       MUIA_List_SourceArray = $8042c0a0;
    { V6  isg char               }
       MUIA_List_Title = $80423e66;
    { V4  ..g LONG               }
       MUIA_List_Visible = $8042191f;
       MUIV_List_Active_Off = -(1);
       MUIV_List_Active_Top = -(2);
       MUIV_List_Active_Bottom = -(3);
       MUIV_List_Active_Up = -(4);
       MUIV_List_Active_Down = -(5);
       MUIV_List_Active_PageUp = -(6);
       MUIV_List_Active_PageDown = -(7);
       MUIV_List_ConstructHook_String = -(1);
       MUIV_List_CopyHook_String = -(1);
       MUIV_List_CursorType_None = 0;
       MUIV_List_CursorType_Bar = 1;
       MUIV_List_CursorType_Rect = 2;
       MUIV_List_DestructHook_String = -(1);
    {                                                                           }
    {  Floattext                                                                }
    {                                                                           }


    const
       MUIC_Floattext : PChar = 'Floattext.mui';

    { Attributes  }
    { V4  isg BOOL               }

    const
       MUIA_Floattext_Justify = $8042dc03;
    { V4  is. STRPTR             }
       MUIA_Floattext_SkipChars = $80425c7d;
    { V4  is. LONG               }
       MUIA_Floattext_TabSize = $80427d17;
    { V4  isg STRPTR             }
       MUIA_Floattext_Text = $8042d16a;
    {                                                                           }
    {  Volumelist                                                               }
    {                                                                           }


    const
       MUIC_Volumelist : PChar = 'Volumelist.mui';

    {                                                                           }
    {  Scrmodelist                                                              }
    {                                                                           }

    const
       MUIC_Scrmodelist : PChar = 'Scrmodelist.mui';

    { Attributes  }
    {                                                                           }
    {  Dirlist                                                                  }
    {                                                                           }


    const
       MUIC_Dirlist : PChar = 'Dirlist.mui';

    { Methods  }
    { V4   }

    const
       MUIM_Dirlist_ReRead = $80422d71;

    type
       MUIP_Dirlist_ReRead = record
            MethodID : ULONG;
         end;

    { Attributes  }
    { V4  is. STRPTR             }

    const
       MUIA_Dirlist_AcceptPattern = $8042760a;
    { V4  isg STRPTR             }
       MUIA_Dirlist_Directory = $8042ea41;
    { V4  is. BOOL               }
       MUIA_Dirlist_DrawersOnly = $8042b379;
    { V4  is. BOOL               }
       MUIA_Dirlist_FilesOnly = $8042896a;
    { V4  is. BOOL               }
       MUIA_Dirlist_FilterDrawers = $80424ad2;
    { V4  is. struct Hook        }
       MUIA_Dirlist_FilterHook = $8042ae19;
    { V6  is. BOOL               }
       MUIA_Dirlist_MultiSelDirs = $80428653;
    { V4  ..g LONG               }
       MUIA_Dirlist_NumBytes = $80429e26;
    { V4  ..g LONG               }
       MUIA_Dirlist_NumDrawers = $80429cb8;
    { V4  ..g LONG               }
       MUIA_Dirlist_NumFiles = $8042a6f0;
    { V4  ..g STRPTR             }
       MUIA_Dirlist_Path = $80426176;
    { V4  is. BOOL               }
       MUIA_Dirlist_RejectIcons = $80424808;
    { V4  is. STRPTR             }
       MUIA_Dirlist_RejectPattern = $804259c7;
    { V4  is. LONG               }
       MUIA_Dirlist_SortDirs = $8042bbb9;
    { V4  is. BOOL               }
       MUIA_Dirlist_SortHighLow = $80421896;
    { V4  is. LONG               }
       MUIA_Dirlist_SortType = $804228bc;
    { V4  ..g LONG               }
       MUIA_Dirlist_Status = $804240de;
       MUIV_Dirlist_SortDirs_First = 0;
       MUIV_Dirlist_SortDirs_Last = 1;
       MUIV_Dirlist_SortDirs_Mix = 2;
       MUIV_Dirlist_SortType_Name = 0;
       MUIV_Dirlist_SortType_Date = 1;
       MUIV_Dirlist_SortType_Size = 2;
       MUIV_Dirlist_Status_Invalid = 0;
       MUIV_Dirlist_Status_Reading = 1;
       MUIV_Dirlist_Status_Valid = 2;
    {                                                                           }
    {  Numeric                                                                  }
    {                                                                           }

    const
       MUIC_Numeric : PChar = 'Numeric.mui';

    { Methods  }
    { V11  }

    const
       MUIM_Numeric_Decrease = $804243a7;
    { V11  }
       MUIM_Numeric_Increase = $80426ecd;
    { V11  }
       MUIM_Numeric_ScaleToValue = $8042032c;
    { V11  }
       MUIM_Numeric_SetDefault = $8042ab0a;
    { V11  }
       MUIM_Numeric_Stringify = $80424891;
    { V11  }
       MUIM_Numeric_ValueToScale = $80423e4f;

    type
       tMUIP_Numeric_Decrease = record
            MethodID : ULONG;
            amount : LONG;
         end;
       pMUIP_Numeric_Decrease = ^tMUIP_Numeric_Decrease;

       tMUIP_Numeric_Increase = record
            MethodID : ULONG;
            amount : LONG;
         end;
       pMUIP_Numeric_Increase = ^tMUIP_Numeric_Increase;

       tMUIP_Numeric_ScaleToValue = record
            MethodID : ULONG;
            scalemin : LONG;
            scalemax : LONG;
            scale : LONG;
         end;
       pMUIP_Numeric_ScaleToValue = ^tMUIP_Numeric_ScaleToValue;

       tMUIP_Numeric_SetDefault = record
            MethodID : ULONG;
         end;
       pMUIP_Numeric_SetDefault = ^tMUIP_Numeric_SetDefault;

       tMUIP_Numeric_Stringify = record
            MethodID : ULONG;
            value : LONG;
         end;
       pMUIP_Numeric_Stringify = ^tMUIP_Numeric_Stringify;

       tMUIP_Numeric_ValueToScale = record
            MethodID : ULONG;
            scalemin : LONG;
            scalemax : LONG;
         end;
       pMUIP_Numeric_ValueToScale = ^tMUIP_Numeric_ValueToScale;

    { Attributes  }
    { V11 isg BOOL               }

    const
       MUIA_Numeric_CheckAllSizes = $80421594;
    { V11 isg LONG               }
       MUIA_Numeric_Default = $804263e8;
    { V11 isg STRPTR             }
       MUIA_Numeric_Format = $804263e9;
    { V11 isg LONG               }
       MUIA_Numeric_Max = $8042d78a;
    { V11 isg LONG               }
       MUIA_Numeric_Min = $8042e404;
    { V11 isg BOOL               }
       MUIA_Numeric_Reverse = $8042f2a0;
    { V11 isg BOOL               }
       MUIA_Numeric_RevLeftRight = $804294a7;
    { V11 isg BOOL               }
       MUIA_Numeric_RevUpDown = $804252dd;
    { V11 isg LONG               }
       MUIA_Numeric_Value = $8042ae3a;
    {                                                                           }
    {  Knob                                                                     }
    {                                                                           }


    const
       MUIC_Knob : PChar = 'Knob.mui';

    {                                                                           }
    {  Levelmeter                                                               }
    {                                                                           }


    const
       MUIC_Levelmeter : PChar = 'Levelmeter.mui';

    { Attributes  }
    { V11 isg STRPTR             }

    const
       MUIA_Levelmeter_Label = $80420dd5;
    {                                                                           }
    {  Numericbutton                                                            }
    {                                                                           }


    const
       MUIC_Numericbutton : PChar = 'Numericbutton.mui';

    {                                                                           }
    {  Slider                                                                   }
    {                                                                           }


    const
       MUIC_Slider : PChar = 'Slider.mui';

    { Attributes  }
    { V11 isg BOOL               }

    const
       MUIA_Slider_Horiz = $8042fad1;

    { V6  i.. BOOL               }

    const
       MUIA_Slider_Quiet = $80420b26;

    {                                                                           }
    {  Framedisplay                                                             }
    {                                                                           }


    const
       MUIC_Framedisplay : PChar = 'Framedisplay.mui';

    { Attributes  }
    {                                                                           }
    {  Popframe                                                                 }
    {                                                                           }


    const
       MUIC_Popframe : PChar = 'Popframe.mui';

    {                                                                           }
    {  Imagedisplay                                                             }
    {                                                                           }

    const
       MUIC_Imagedisplay : PChar = 'Imagedisplay.mui';

    { Attributes  }
    {                                                                           }
    {  Popimage                                                                 }
    {                                                                           }


    const
       MUIC_Popimage : PChar = 'Popimage.mui';

    {                                                                           }
    {  Pendisplay                                                               }
    {                                                                           }


    const
       MUIC_Pendisplay : PChar = 'Pendisplay.mui';

    { Methods  }
    { V13  }

    const
       MUIM_Pendisplay_SetColormap = $80426c80;
    { V13  }
       MUIM_Pendisplay_SetMUIPen = $8042039d;
    { V13  }
       MUIM_Pendisplay_SetRGB = $8042c131;

    type
       MUIP_Pendisplay_SetColormap = record
            MethodID : ULONG;
            colormap : LONG;
         end;

       MUIP_Pendisplay_SetMUIPen = record
            MethodID : ULONG;
            muipen : LONG;
         end;

       MUIP_Pendisplay_SetRGB = record
            MethodID : ULONG;
            red : ULONG;
            green : ULONG;
            blue : ULONG;
         end;

    { Attributes  }
    { V13 ..g Object             }

    const
       MUIA_Pendisplay_Pen = $8042a748;
    { V13 isg Object             }
       MUIA_Pendisplay_Reference = $8042dc24;
    { V11 isg struct MUI_RGBcolor    }
       MUIA_Pendisplay_RGBcolor = $8042a1a9;
    { V11 isg struct MUI_PenSpec     }
       MUIA_Pendisplay_Spec = $8042a204;
    {                                                                           }
    {  Poppen                                                                   }
    {                                                                           }


    const
       MUIC_Poppen : PChar = 'Poppen.mui';

    {                                                                           }
    {  Group                                                                    }
    {                                                                           }


    const
       MUIC_Group : PChar = 'Group.mui';

    { Methods  }
    { V11  }

    const
       MUIM_Group_ExitChange = $8042d1cc;
    { V11  }
       MUIM_Group_InitChange = $80420887;
    { V4   }
       MUIM_Group_Sort = $80427417;

    type
       tMUIP_Group_ExitChange = record
            MethodID : ULONG;
         end;
       pMUIP_Group_ExitChange = ^tMUIP_Group_ExitChange;

       tMUIP_Group_InitChange = record
            MethodID : ULONG;
         end;
       pMUIP_Group_InitChange = ^tMUIP_Group_InitChange;

       tMUIP_Group_Sort = record
            MethodID : ULONG;
            obj : array[0..0] of pObject_;
         end;
       pMUIP_Group_Sort = ^tMUIP_Group_Sort;

    { Attributes  }
    { V5  isg LONG               }

    const
       MUIA_Group_ActivePage = $80424199;
    { V4  i.. Object             }
       MUIA_Group_Child = $804226e6;
    { V4  ..g struct List        }
       MUIA_Group_ChildList = $80424748;
    { V4  is. LONG               }
       MUIA_Group_Columns = $8042f416;
    { V4  i.. BOOL               }
       MUIA_Group_Horiz = $8042536b;
    { V4  isg LONG               }
       MUIA_Group_HorizSpacing = $8042c651;
    { V11 i.. struct Hook        }
       MUIA_Group_LayoutHook = $8042c3b2;
    { V5  i.. BOOL               }
       MUIA_Group_PageMode = $80421a5f;
    { V4  is. LONG               }
       MUIA_Group_Rows = $8042b68f;
    { V4  i.. BOOL               }
       MUIA_Group_SameHeight = $8042037e;
    { V4  i.. BOOL               }
       MUIA_Group_SameSize = $80420860;
    { V4  i.. BOOL               }
       MUIA_Group_SameWidth = $8042b3ec;
    { V4  is. LONG               }
       MUIA_Group_Spacing = $8042866d;
    { V4  isg LONG               }
       MUIA_Group_VertSpacing = $8042e1bf;
       MUIV_Group_ActivePage_First = 0;
       MUIV_Group_ActivePage_Last = -(1);
       MUIV_Group_ActivePage_Prev = -(2);
       MUIV_Group_ActivePage_Next = -(3);
       MUIV_Group_ActivePage_Advance = -(4);
    {                                                                           }
    {  Mccprefs                                                                 }
    {                                                                           }


    const
       MUIC_Mccprefs : PChar = 'Mccprefs.mui';

    {                                                                           }
    {  Register                                                                 }
    {                                                                           }

    const
       MUIC_Register : PChar = 'Register.mui';

    { Attributes  }
    { V7  i.g BOOL               }

    const
       MUIA_Register_Frame = $8042349b;
    { V7  i.g STRPTR             }
       MUIA_Register_Titles = $804297ec;
    {                                                                           }
    {  Penadjust                                                                }
    {                                                                           }


    const
       MUIC_Penadjust : PChar= 'Penadjust.mui';

    { Methods  }
    { Attributes  }
    { V11 i.. BOOL               }

    const
       MUIA_Penadjust_PSIMode = $80421cbb;
    {                                                                           }
    {  Settingsgroup                                                            }
    {                                                                           }


    const
       MUIC_Settingsgroup : PChar = 'Settingsgroup.mui';

    { Methods  }
    { V11  }

    const
       MUIM_Settingsgroup_ConfigToGadgets = $80427043;
    { V11  }
       MUIM_Settingsgroup_GadgetsToConfig = $80425242;

    type
       tMUIP_Settingsgroup_ConfigToGadgets = record
            MethodID : ULONG;
            configdata : pObject_;
         end;
       pMUIP_Settingsgroup_ConfigToGadgets = ^tMUIP_Settingsgroup_ConfigToGadgets;

       tMUIP_Settingsgroup_GadgetsToConfig = record
            MethodID : ULONG;
            configdata : pObject_;
         end;
       pMUIP_Settingsgroup_GadgetsToConfig = ^tMUIP_Settingsgroup_GadgetsToConfig;

    { Attributes  }
    {                                                                           }
    {  Settings                                                                 }
    {                                                                           }

    const
       MUIC_Settings : PChar = 'Settings.mui';

    { Methods  }
    { Attributes  }
    {                                                                           }
    {  Frameadjust                                                              }
    {                                                                           }


    const
       MUIC_Frameadjust : PChar = 'Frameadjust.mui';

    { Methods  }
    { Attributes  }
    {                                                                           }
    {  Imageadjust                                                              }
    {                                                                           }


    const
       MUIC_Imageadjust : PChar = 'Imageadjust.mui';

    { Methods  }
    { Attributes  }

    const
       MUIV_Imageadjust_Type_All = 0;
       MUIV_Imageadjust_Type_Image = 1;
       MUIV_Imageadjust_Type_Background = 2;
       MUIV_Imageadjust_Type_Pen = 3;
    {                                                                           }
    {  Virtgroup                                                                }
    {                                                                           }


    const
       MUIC_Virtgroup : PChar = 'Virtgroup.mui';

    { Methods  }
    { Attributes  }
    { V6  ..g LONG               }

    const
       MUIA_Virtgroup_Height = $80423038;
    { V11 i.. BOOL               }
       MUIA_Virtgroup_Input = $80427f7e;
    { V6  isg LONG               }
       MUIA_Virtgroup_Left = $80429371;
    { V6  isg LONG               }
       MUIA_Virtgroup_Top = $80425200;
    { V6  ..g LONG               }
       MUIA_Virtgroup_Width = $80427c49;
    {                                                                           }
    {  Scrollgroup                                                              }
    {                                                                           }


    const
       MUIC_Scrollgroup : PChar = 'Scrollgroup.mui';

    { Methods  }
    { Attributes  }
    { V4  i.g Object             }

    const
       MUIA_Scrollgroup_Contents = $80421261;
    { V9  i.. BOOL               }
       MUIA_Scrollgroup_FreeHoriz = $804292f3;
    { V9  i.. BOOL               }
       MUIA_Scrollgroup_FreeVert = $804224f2;
    { V16 ..g Object             }
       MUIA_Scrollgroup_HorizBar = $8042b63d;
    { V13 i.. BOOL               }
       MUIA_Scrollgroup_UseWinBorder = $804284c1;
    { V16 ..g Object             }
       MUIA_Scrollgroup_VertBar = $8042cdc0;
    {                                                                           }
    {  Scrollbar                                                                }
    {                                                                           }


    const
       MUIC_Scrollbar : PChar = 'Scrollbar.mui';

    { Attributes  }
    { V11 i.. LONG               }

    const
       MUIA_Scrollbar_Type = $8042fb6b;
       MUIV_Scrollbar_Type_Default = 0;
       MUIV_Scrollbar_Type_Bottom = 1;
       MUIV_Scrollbar_Type_Top = 2;
       MUIV_Scrollbar_Type_Sym = 3;
    {                                                                           }
    {  Listview                                                                 }
    {                                                                           }


    const
       MUIC_Listview : PChar = 'Listview.mui';

    { Attributes  }
    { V7  ..g LONG               }

    const
       MUIA_Listview_ClickColumn = $8042d1b3;
    { V7  isg LONG               }
       MUIA_Listview_DefClickColumn = $8042b296;
    { V4  i.g BOOL               }
       MUIA_Listview_DoubleClick = $80424635;
    { V11 isg LONG               }
       MUIA_Listview_DragType = $80425cd3;
    { V4  i.. BOOL               }
       MUIA_Listview_Input = $8042682d;
    { V4  i.g Object             }
       MUIA_Listview_List = $8042bcce;
    { V7  i.. LONG               }
       MUIA_Listview_MultiSelect = $80427e08;
    { V10 i.. BOOL               }
       MUIA_Listview_ScrollerPos = $8042b1b4;
    { V4  ..g BOOL               }
       MUIA_Listview_SelectChange = $8042178f;
       MUIV_Listview_DragType_None = 0;
       MUIV_Listview_DragType_Immediate = 1;
       MUIV_Listview_MultiSelect_None = 0;
       MUIV_Listview_MultiSelect_Default = 1;
       MUIV_Listview_MultiSelect_Shifted = 2;
       MUIV_Listview_MultiSelect_Always = 3;
       MUIV_Listview_ScrollerPos_Default = 0;
       MUIV_Listview_ScrollerPos_Left = 1;
       MUIV_Listview_ScrollerPos_Right = 2;
       MUIV_Listview_ScrollerPos_None = 3;
    {                                                                           }
    {  Radio                                                                    }
    {                                                                           }


    const
       MUIC_Radio : PChar = 'Radio.mui';

    { Attributes  }
    { V4  isg LONG               }

    const
       MUIA_Radio_Active = $80429b41;
    { V4  i.. STRPTR             }
       MUIA_Radio_Entries = $8042b6a1;
    {                                                                           }
    {  Cycle                                                                    }
    {                                                                           }


    const
       MUIC_Cycle : PChar = 'Cycle.mui';

    { Attributes  }
    { V4  isg LONG               }

    const
       MUIA_Cycle_Active = $80421788;
    { V4  i.. STRPTR             }
       MUIA_Cycle_Entries = $80420629;
       MUIV_Cycle_Active_Next = -(1);
       MUIV_Cycle_Active_Prev = -(2);
    {                                                                           }
    {  Coloradjust                                                              }
    {                                                                           }


    const
       MUIC_Coloradjust : PChar = 'Coloradjust.mui';

    { Methods  }
    { Attributes  }
    { V4  isg ULONG              }

    const
       MUIA_Coloradjust_Blue = $8042b8a3;
    { V4  isg ULONG              }
       MUIA_Coloradjust_Green = $804285ab;
    { V4  isg ULONG              }
       MUIA_Coloradjust_ModeID = $8042ec59;
    { V4  isg ULONG              }
       MUIA_Coloradjust_Red = $80420eaa;
    { V4  isg ULONG              }
       MUIA_Coloradjust_RGB = $8042f899;
    {                                                                           }
    {  Palette                                                                  }
    {                                                                           }


    const
       MUIC_Palette : PChar = 'Palette.mui';

    { Attributes  }
    { V6  i.g struct MUI_Palette_Entry    }

    const
       MUIA_Palette_Entries = $8042a3d8;
    { V6  isg BOOL               }
       MUIA_Palette_Groupable = $80423e67;
    { V6  isg char               }
       MUIA_Palette_Names = $8042c3a2;
    {                                                                           }
    {  Popstring                                                                }
    {                                                                           }


    const
       MUIC_Popstring : PChar = 'Popstring.mui';

    { Methods  }
    { V7   }

    const
       MUIM_Popstring_Close = $8042dc52;
    { V7   }
       MUIM_Popstring_Open = $804258ba;

    type
       tMUIP_Popstring_Close = record
            MethodID : ULONG;
            result : LONG;
         end;
       pMUIP_Popstring_Close = ^tMUIP_Popstring_Close;

       tMUIP_Popstring_Open = record
            MethodID : ULONG;
         end;
       pMUIP_Popstring_Open = ^tMUIP_Popstring_Open;

    { Attributes  }
    { V7  i.g Object             }

    const
       MUIA_Popstring_Button = $8042d0b9;
    { V7  isg struct Hook        }
       MUIA_Popstring_CloseHook = $804256bf;
    { V7  isg struct Hook        }
       MUIA_Popstring_OpenHook = $80429d00;
    { V7  i.g Object             }
       MUIA_Popstring_String = $804239ea;
    { V7  isg BOOL               }
       MUIA_Popstring_Toggle = $80422b7a;
    {                                                                           }
    {  Popobject                                                                }
    {                                                                           }


    const
       MUIC_Popobject : PChar = 'Popobject.mui';

    { Attributes  }
    { V7  isg BOOL               }

    const
       MUIA_Popobject_Follow = $80424cb5;
    { V7  isg BOOL               }
       MUIA_Popobject_Light = $8042a5a3;
    { V7  i.g Object             }
       MUIA_Popobject_Object = $804293e3;
    { V7  isg struct Hook        }
       MUIA_Popobject_ObjStrHook = $8042db44;
    { V7  isg struct Hook        }
       MUIA_Popobject_StrObjHook = $8042fbe1;
    { V7  isg BOOL               }
       MUIA_Popobject_Volatile = $804252ec;
    { V9  isg struct Hook        }
       MUIA_Popobject_WindowHook = $8042f194;
    {                                                                           }
    {  Poplist                                                                  }
    {                                                                           }


    const
       MUIC_Poplist : PChar = 'Poplist.mui';

    { Attributes  }
    { V8  i.. char               }

    const
       MUIA_Poplist_Array = $8042084c;
    {                                                                           }
    {  Popscreen                                                                }
    {                                                                           }


    const
       MUIC_Popscreen : PChar = 'Popscreen.mui';

    { Attributes  }
    {                                                                           }
    {  Popasl                                                                   }
    {                                                                           }

    const
       MUIC_Popasl : PChar = 'Popasl.mui';

    { Attributes  }
    { V7  ..g BOOL               }

    const
       MUIA_Popasl_Active = $80421b37;
    { V7  isg struct Hook        }
       MUIA_Popasl_StartHook = $8042b703;
    { V7  isg struct Hook        }
       MUIA_Popasl_StopHook = $8042d8d2;
    { V7  i.g ULONG              }
       MUIA_Popasl_Type = $8042df3d;
    {                                                                           }
    {  Semaphore                                                                }
    {                                                                           }


    const
       MUIC_Semaphore : PChar = 'Semaphore.mui';

    { Methods  }
    { V11  }

    const
       MUIM_Semaphore_Attempt = $80426ce2;
    { V11  }
       MUIM_Semaphore_AttemptShared = $80422551;
    { V11  }
       MUIM_Semaphore_Obtain = $804276f0;
    { V11  }
       MUIM_Semaphore_ObtainShared = $8042ea02;
    { V11  }
       MUIM_Semaphore_Release = $80421f2d;

    type
       tMUIP_Semaphore_Attempt = record
            MethodID : ULONG;
         end;
       pMUIP_Semaphore_Attempt = ^tMUIP_Semaphore_Attempt;

       tMUIP_Semaphore_AttemptShared = record
            MethodID : ULONG;
         end;
       pMUIP_Semaphore_AttemptShared = ^tMUIP_Semaphore_AttemptShared;

       tMUIP_Semaphore_Obtain = record
            MethodID : ULONG;
         end;
       pMUIP_Semaphore_Obtain = ^tMUIP_Semaphore_Obtain;

       tMUIP_Semaphore_ObtainShared = record
            MethodID : ULONG;
         end;
       pMUIP_Semaphore_ObtainShared = ^tMUIP_Semaphore_ObtainShared;

       tMUIP_Semaphore_Release = record
            MethodID : ULONG;
         end;
       pMUIP_Semaphore_Release = ^tMUIP_Semaphore_Release;

    {                                                                           }
    {  Applist                                                                  }
    {                                                                           }

    const
       MUIC_Applist : PChar = 'Applist.mui';
    { Methods  }
    {                                                                           }
    {  Cclist                                                                   }
    {                                                                           }


    const
       MUIC_Cclist : PChar = 'Cclist.mui';

    { Methods  }
    {                                                                           }
    {  Dataspace                                                                }
    {                                                                           }


    const
       MUIC_Dataspace : PChar = 'Dataspace.mui';

    { Methods  }
    { V11  }

    const
       MUIM_Dataspace_Add = $80423366;
    { V11  }
       MUIM_Dataspace_Clear = $8042b6c9;
    { V11  }
       MUIM_Dataspace_Find = $8042832c;
    { V11  }
       MUIM_Dataspace_Merge = $80423e2b;
    { V11  }
       MUIM_Dataspace_ReadIFF = $80420dfb;
    { V11  }
       MUIM_Dataspace_Remove = $8042dce1;
    { V11  }
       MUIM_Dataspace_WriteIFF = $80425e8e;

    type
       tMUIP_Dataspace_Add = record
            MethodID : ULONG;
            data : APTR;
            len : LONG;
            id : ULONG;
         end;
       pMUIP_Dataspace_Add = ^tMUIP_Dataspace_Add;

       tMUIP_Dataspace_Clear = record
            MethodID : ULONG;
         end;
       pMUIP_Dataspace_Clear = ^tMUIP_Dataspace_Clear;

       tMUIP_Dataspace_Find = record
            MethodID : ULONG;
            id : ULONG;
         end;
       pMUIP_Dataspace_Find = ^tMUIP_Dataspace_Find;

       tMUIP_Dataspace_Merge = record
            MethodID : ULONG;
            dataspace : pObject_;
         end;
       pMUIP_Dataspace_Merge = ^tMUIP_Dataspace_Merge;

       tMUIP_Dataspace_ReadIFF = record
            MethodID : ULONG;
            handle : PIFFHandle;
         end;
       pMUIP_Dataspace_ReadIFF = ^tMUIP_Dataspace_ReadIFF;

       tMUIP_Dataspace_Remove = record
            MethodID : ULONG;
            id : ULONG;
         end;
       pMUIP_Dataspace_Remove = ^tMUIP_Dataspace_Remove;

       tMUIP_Dataspace_WriteIFF = record
            MethodID : ULONG;
            handle : PIFFHandle;
            type_ : ULONG;
            id : ULONG;
         end;
       pMUIP_Dataspace_WriteIFF = ^tMUIP_Dataspace_WriteIFF;

    { Attributes  }
    { V11 i.. APTR               }

    const
       MUIA_Dataspace_Pool = $80424cf9;
    {                                                                           }
    {  Configdata                                                               }
    {                                                                           }


    const
       MUIC_Configdata : PChar = 'Configdata.mui';

    { Methods  }
    { Attributes  }
    {                                                                           }
    {  Dtpic                                                                    }
    {                                                                           }


    const
       MUIC_Dtpic : PChar = 'Dtpic.mui';
    { Attributes  }
    {                                        }
    { End of automatic header file creation  }
    {                                        }
    {
       Structures and Macros for creating custom classes.
                                                                             }
    {
       GENERAL NOTES:

       - Everything described in this header file is only valid within
         MUI classes. You may never use any of these things out of
         a class, e.g. in a traditional MUI application.

       - Except when otherwise stated, all structures are strictly read only.
     }
    { Global information for every object  }
    { ... private data follows ...  }

    type
       tMUI_GlobalInfo = record
            priv0 : ULONG;
            mgi_ApplicationObject : pObject_;
         end;
       pMUI_GlobalInfo = ^tMUI_GlobalInfo;

    { Instance data of notify class  }
       tMUI_NotifyData = record
            mnd_GlobalInfo : PMUI_GlobalInfo;
            mnd_UserData : ULONG;
            mnd_ObjectID : ULONG;
            priv1 : ULONG;
            priv2 : ULONG;
            priv3 : ULONG;
            priv4 : ULONG;
         end;
       pMUI_NotifyData = ^tMUI_NotifyData;


    { use this if a dimension is not limited.  }

    const
       MUI_MAXMAX = 10000;
    { Hook message for custom layout  }
    { type of message (see defines below)                       }
    { list of this groups children, traverse with NextObject()  }
    { results for MUILM_MINMAX                                  }
    { size (and result) for MUILM_LAYOUT                        }

    type
       tMUI_LayoutMsg = record
            lm_Type : ULONG;
            lm_Children : PMinList;
            lm_MinMax : tMUI_MinMax;
            lm_Layout : record
                 Width : LONG;
                 Height : LONG;
                 priv5 : ULONG;
                 priv6 : ULONG;
              end;
         end;
       pMUI_LayoutMsg = ^tMUI_LayoutMsg;

    { MUI wants you to calc your min & max sizes  }

    const
       MUILM_MINMAX = 1;
    { MUI wants you to layout your children       }
       MUILM_LAYOUT = 2;
    { return this if your hook doesn't implement lm_Type  }
       MUILM_UNKNOWN = -(1);
    { (partial) instance data of area class  }
    { RenderInfo for this object  }
    { Font  }
    { min/max/default sizes  }
    { position and dimension  }
    { frame & innerspacing left offset  }
    { frame & innerspacing top offset   }
    { frame & innerspacing add. width   }
    { frame & innerspacing add. height  }
    { see definitions below  }
    { ... private data follows ...  }

    type
       tMUI_AreaData = record
            mad_RenderInfo : PMUI_RenderInfo;
            priv7 : ULONG;
            mad_Font : PTextFont;
            mad_MinMax : tMUI_MinMax;
            mad_Box : tIBox;
            mad_addleft : BYTE;
            mad_addtop : BYTE;
            mad_subwidth : BYTE;
            mad_subheight : BYTE;
            mad_Flags : ULONG;
         end;
      pMUI_AreaData = ^tMUI_AreaData;

    { Definitions for mad_Flags, other flags are private  }
    { completely redraw yourself  }

    const
       MADF_DRAWOBJECT = 1 shl 0;
    { only update yourself  }
       MADF_DRAWUPDATE = 1 shl 1;
    { MUI's draw pens  }
       MPEN_SHINE = 0;
       MPEN_HALFSHINE = 1;
       MPEN_BACKGROUND = 2;
       MPEN_HALFSHADOW = 3;
       MPEN_SHADOW = 4;
       MPEN_TEXT = 5;
       MPEN_FILL = 6;
       MPEN_MARK = 7;
       MPEN_COUNT = 8;
    { Mask for pens from MUI_ObtainPen()  }
       MUIPEN_MASK = $0000ffff;


    { Information on display environment  }
    { valid between MUIM_Setup/MUIM_Cleanup  }
    { valid between MUIM_Setup/MUIM_Cleanup  }
    { valid between MUIM_Setup/MUIM_Cleanup  }
    { valid between MUIM_Setup/MUIM_Cleanup  }
    { valid between MUIM_Show/MUIM_Hide  }
    { valid between MUIM_Show/MUIM_Hide  }
    { valid between MUIM_Setup/MUIM_Cleanup  }
    { ... private data follows ...  }



    {
       If mri_Flags & MUIMRI_RECTFILL, RectFill() is quicker
       than Move()/Draw() for horizontal or vertical lines.
       on the current display.
     }

    const
       MUIMRI_RECTFILL = 1 shl 0;
    {
       If mri_Flags & MUIMRI_TRUECOLOR, display environment is a
       cybergraphics emulated hicolor or true color display.
     }
       MUIMRI_TRUECOLOR = 1 shl 1;
    {
       If mri_Flags & MUIMRI_THINFRAMES, MUI uses thin frames
       (1:1) apsect ratio instead of standard 2:1 frames.
     }
       MUIMRI_THINFRAMES = 1 shl 2;
    {
       If mri_Flags & MUIMRI_REFRESHMODE, MUI is currently
       refreshing a WFLG_SIMPLEREFRESH window and is between
       a BeginRefresh()/EndRefresh() pair.
     }
       MUIMRI_REFRESHMODE = 1 shl 3;
    { the following macros can be used to get pointers to an objects
       GlobalInfo and RenderInfo structures.  }

    type
       t__dummyXFC2__ = record
            mnd : tMUI_NotifyData;
            mad : tMUI_AreaData;
         end;
       p__dummyXFC2__ = ^t__dummyXFC2__;


    const

        MUIKEY_RELEASE = -2;
        MUIKEY_NONE    = -1;
        MUIKEY_PRESS   = 0;
        MUIKEY_TOGGLE  = 1;
        MUIKEY_UP      = 2;
        MUIKEY_DOWN    = 3;
        MUIKEY_PAGEUP  = 4;
        MUIKEY_PAGEDOWN = 5;
        MUIKEY_TOP      = 6;
        MUIKEY_BOTTOM   = 7;
        MUIKEY_LEFT     = 8;
        MUIKEY_RIGHT    = 9;
        MUIKEY_WORDLEFT = 10;
        MUIKEY_WORDRIGHT = 11;
        MUIKEY_LINESTART = 12;
        MUIKEY_LINEEND   = 13;
        MUIKEY_GADGET_NEXT = 14;
        MUIKEY_GADGET_PREV = 15;
        MUIKEY_GADGET_OFF  = 16;
        MUIKEY_WINDOW_CLOSE = 17;
        MUIKEY_WINDOW_NEXT  = 18;
        MUIKEY_WINDOW_PREV  = 19;
        MUIKEY_HELP         = 20;
        MUIKEY_POPUP        = 21;
        MUIKEY_COUNT        = 22;

       MUIKEYF_PRESS = 1 shl MUIKEY_PRESS;
       MUIKEYF_TOGGLE = 1 shl MUIKEY_TOGGLE;
       MUIKEYF_UP = 1 shl MUIKEY_UP;
       MUIKEYF_DOWN = 1 shl MUIKEY_DOWN;
       MUIKEYF_PAGEUP = 1 shl MUIKEY_PAGEUP;
       MUIKEYF_PAGEDOWN = 1 shl MUIKEY_PAGEDOWN;
       MUIKEYF_TOP = 1 shl MUIKEY_TOP;
       MUIKEYF_BOTTOM = 1 shl MUIKEY_BOTTOM;
       MUIKEYF_LEFT = 1 shl MUIKEY_LEFT;
       MUIKEYF_RIGHT = 1 shl MUIKEY_RIGHT;
       MUIKEYF_WORDLEFT = 1 shl MUIKEY_WORDLEFT;
       MUIKEYF_WORDRIGHT = 1 shl MUIKEY_WORDRIGHT;
       MUIKEYF_LINESTART = 1 shl MUIKEY_LINESTART;
       MUIKEYF_LINEEND = 1 shl MUIKEY_LINEEND;
       MUIKEYF_GADGET_NEXT = 1 shl MUIKEY_GADGET_NEXT;
       MUIKEYF_GADGET_PREV = 1 shl MUIKEY_GADGET_PREV;
       MUIKEYF_GADGET_OFF = 1 shl MUIKEY_GADGET_OFF;
       MUIKEYF_WINDOW_CLOSE = 1 shl MUIKEY_WINDOW_CLOSE;
       MUIKEYF_WINDOW_NEXT = 1 shl MUIKEY_WINDOW_NEXT;
       MUIKEYF_WINDOW_PREV = 1 shl MUIKEY_WINDOW_PREV;
       MUIKEYF_HELP = 1 shl MUIKEY_HELP;
       MUIKEYF_POPUP = 1 shl MUIKEY_POPUP;

    { MUI_CustomClass returned by MUI_CreateCustomClass()  }
    { use for whatever you want  }
    { MUI has opened these libraries  }
    { for you automatically. You can  }
    { use them or decide to open      }
    { your libraries yourself.        }
    { pointer to super class    }
    { pointer to the new class  }
    { ... private data follows ...  }

    type
       tMUI_CustomClass = record
            mcc_UserData : APTR;
            mcc_UtilityBase : PLibrary;
            mcc_DOSBase : PLibrary;
            mcc_GfxBase : PLibrary;
            mcc_IntuitionBase : PLibrary;
            mcc_Super : PIClass;
            mcc_Class : PIClass;
         end;
       pMUI_CustomClass = ^tMUI_CustomClass;

FUNCTION MUI_NewObjectA(class_ : pChar; tags : pTagItem) : pObject_;
PROCEDURE MUI_DisposeObject(obj : pObject_);
FUNCTION MUI_RequestA(app : POINTER; win : POINTER; flags : LONGBITS; title : pChar; gadgets : pChar; format : pChar; params : POINTER) : LONGINT;
FUNCTION MUI_AllocAslRequest(typ : ULONG; tags : pTagItem) : POINTER;
FUNCTION MUI_AslRequest(req : POINTER; tags : pTagItem) : BOOLEAN;
PROCEDURE MUI_FreeAslRequest(req : POINTER);
FUNCTION MUI_Error : LONGINT;
FUNCTION MUI_SetError(errnum : LONGINT) : LONGINT;
FUNCTION MUI_GetClass(name : pCHar) : pIClass;
PROCEDURE MUI_FreeClass(cl : pIClass);
PROCEDURE MUI_RequestIDCMP(obj : pObject_; flags : ULONG);
PROCEDURE MUI_RejectIDCMP(obj : pObject_; flags : ULONG);
PROCEDURE MUI_Redraw(obj : pObject_; flags : ULONG);
FUNCTION MUI_CreateCustomClass(base : pLibrary; supername : pChar; supermcc : pMUI_CustomClass; datasize : LONGINT; dispatcher : POINTER) : pMUI_CustomClass;
FUNCTION MUI_DeleteCustomClass(mcc : pMUI_CustomClass) : BOOLEAN;
FUNCTION MUI_MakeObjectA(typ: LONGINT; params : pULONG) : pULONG;
FUNCTION MUI_Layout(obj : pObject_; l : LONGINT; t : LONGINT; w : LONGINT; h : LONGINT; flags : ULONG) : BOOLEAN;
FUNCTION MUI_ObtainPen(mri : pMUI_RenderInfo; spec : pMUI_PenSpec; flags : ULONG) : LONGINT;
PROCEDURE MUI_ReleasePen(mri : pMUI_RenderInfo; pen : LONGINT);
FUNCTION MUI_AddClipping(mri : pMUI_RenderInfo; l : smallint; t : smallint; w : smallint; h : smallint) : POINTER;
PROCEDURE MUI_RemoveClipping(mri : pMUI_RenderInfo; h : POINTER);
FUNCTION MUI_AddClipRegion(mri : pMUI_RenderInfo; region : pRegion) : POINTER;
PROCEDURE MUI_RemoveClipRegion(mri : pMUI_RenderInfo; region : POINTER);
FUNCTION MUI_BeginRefresh(mri : pMUI_RenderInfo; flags : ULONG) : BOOLEAN;
PROCEDURE MUI_EndRefresh(mri : pMUI_RenderInfo; flags : ULONG);


(*
** some procedures to get some information about our object
*)

function MUINotifyData(obj : APTR) : pMUI_NotifyData;
function MUIAreaData(obj : APTR) : pMUI_AreaData;
function MUIGlobalInfo(obj : APTR) : pMUI_GlobalInfo;
function MUIUserData(obj : APTR) : Pointer ;
function MUIRenderInfo(obj : APTR) : pMUI_RenderInfo;
function MUIPen(pen : longint): longint;
(*
** some more specialized functions to retain information about special
** object-data like rastport, window, etc.
**
** NOTE: These macros may only be used in custom classes and are
** only valid if your class is inbetween the specified methods!
*)

function OBJ_App(obj : APTR) : pObject_;       (* valid between MUIM_Setup/Cleanup *)
function OBJ_Win(obj : APTR) : pObject_;       (* valid between MUIM_Setup/Cleanup *)
function OBJ_Dri(obj : APTR) : pDrawInfo;          (* valid between MUIM_Setup/Cleanup *)
function OBJ_Screen(obj : APTR) : pScreen;         (* valid between MUIM_Setup/Cleanup *)
function OBJ_Pens(obj : APTR) : pWord;             (* valid between MUIM_Setup/Cleanup *)
function OBJ_Window(obj : APTR) : pWindow;         (* valid between MUIM_Show/Hide *)
function OBJ_Rp(obj : APTR) : pRastPort;           (* valid between MUIM_Show/Hide *)
function OBJ_Left(obj : APTR) : smallint;           (* valid during MUIM_Draw *)
function OBJ_Top(obj : APTR) : smallint;            (* valid during MUIM_Draw *)
function OBJ_Width(obj : APTR) : smallint;          (* valid during MUIM_Draw *)
function OBJ_Height(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
function OBJ_Right(obj : APTR) : smallint;          (* valid during MUIM_Draw *)
function OBJ_Bottom(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
function OBJ_AddLeft(obj : APTR) : smallint;        (* valid during MUIM_Draw *)
function OBJ_AddTop(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
function OBJ_SubWidth(obj : APTR) : smallint;       (* valid during MUIM_Draw *)
function OBJ_SubHeight(obj : APTR) : smallint;      (* valid during MUIM_Draw *)
function OBJ_MLeft(obj : APTR) : smallint;          (* valid during MUIM_Draw *)
function OBJ_MTop(obj : APTR) : smallint;           (* valid during MUIM_Draw *)
function OBJ_MWidth(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
function OBJ_MHeight(obj : APTR) : smallint;        (* valid during MUIM_Draw *)
function OBJ_MRight(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
function OBJ_MBottom(obj : APTR) : smallint;        (* valid during MUIM_Draw *)
function OBJ_Font(obj : APTR) : pTextFont;         (* valid between MUIM_Setup/Cleanup *)
function OBJ_MinWidth(obj : APTR) : ULONG;         (* valid between MUIM_Show/Hide *)
function OBJ_MinHeight(obj : APTR) : ULONG;        (* valid between MUIM_Show/Hide *)
function OBJ_MaxWidth(obj : APTR) : ULONG;         (* valid between MUIM_Show/Hide *)
function OBJ_MaxHeight(obj : APTR) : ULONG;        (* valid between MUIM_Show/Hide *)
function OBJ_DefWidth(obj : APTR) : ULONG;         (* valid between MUIM_Show/Hide *)
function OBJ_DefHeight(obj : APTR) : ULONG;        (* valid between MUIM_Show/Hide *)
function OBJ_Flags(obj : APTR) : ULONG;

function OBJ_Between(a,x,b : smallint): boolean;
function OBJ_IsInObject(x,y : smallint; obj : pObject_): boolean;

function MUIV_Window_AltHeight_MinMax(p : longint) : longint;
function MUIV_Window_AltHeight_Visible(p : longint) : longint;
function MUIV_Window_AltHeight_Screen(p : longint) : longint;
function MUIV_Window_AltTopEdge_Delta(p : longint) : longint;
function MUIV_Window_AltWidth_MinMax(p : longint) : longint;
function MUIV_Window_AltWidth_Visible(p : longint) : longint;
function MUIV_Window_AltWidth_Screen(p : longint) : longint;
function MUIV_Window_Height_MinMax(p : longint) : longint;
function MUIV_Window_Height_Visible(p : longint) : longint;
function MUIV_Window_Height_Screen(p : longint) : longint;
function MUIV_Window_TopEdge_Delta(p : longint) : longint;
function MUIV_Window_Width_MinMax(p : longint) : longint;
function MUIV_Window_Width_Visible(p : longint) : longint;
function MUIV_Window_Width_Screen(p : longint) : longint;

{
 Functions and procedures with array of const go here
}
FUNCTION MUI_AllocAslRequestTags(_type : longword; const tags : Array Of Const) : POINTER;
FUNCTION MUI_AslRequestTags(req : POINTER; const tags : Array Of Const) : BOOLEAN;
FUNCTION MUI_MakeObject(_type : LONGINT; const params : Array Of Const) : pULONG;
FUNCTION MUI_NewObject(a0arg : pCHAR; const tags : Array Of Const) : pULONG;
FUNCTION MUI_Request(app : POINTER; win : POINTER; flags : longword; title : pCHAR; gadgets : pCHAR; format : pCHAR; const params : Array Of Const) : LONGINT;

VAR MUIMasterBase : pLibrary;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitMUIMASTERLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    MUIMASTERIsCompiledHow : longint;

implementation

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray,longarray;

function MUINotifyData(obj : APTR) : pMUI_NotifyData;
begin
    MUINotifyData := pMUI_NotifyData(@p__dummyXFC2__(obj)^.mnd);
end;

function MUIAreaData(obj : APTR) : pMUI_AreaData;
begin
    MUIAreaData := pMUI_AreaData(@p__dummyXFC2__(obj)^.mad);
end;

function MUIGlobalInfo(obj : APTR) : pMUI_GlobalInfo;
begin
    MUIGlobalInfo := pMUI_GlobalInfo(p__dummyXFC2__(obj)^.mnd.mnd_GlobalInfo);
end;

function MUIUserData(obj : APTR) : Pointer ;
begin
    MUIUserData := Pointer(p__dummyXFC2__(obj)^.mnd.mnd_GlobalInfo);
end;

function MUIRenderInfo(obj : APTR) : pMUI_RenderInfo;
begin
    MUIRenderInfo := pMUI_RenderInfo(p__dummyXFC2__(obj)^.mad.mad_RenderInfo);
end;

function MUIPen(pen : longint): longint;
begin
    MUIPen := longint(pen*MUIPEN_Mask);
end;

function OBJ_App(obj : APTR) : pObject_;       (* valid between MUIM_Setup/Cleanup *)
begin
    OBJ_App := pMUI_GlobalInfo(obj)^.mgi_ApplicationObject;
end;

function OBJ_Win(obj : APTR) : pObject_;       (* valid between MUIM_Setup/Cleanup *)
begin
    OBJ_Win := pMUI_RenderInfo(obj)^.mri_WindowObject;
end;

function OBJ_Dri(obj : APTR) : pDrawInfo;          (* valid between MUIM_Setup/Cleanup *)
begin
    OBJ_Dri := pMUI_RenderInfo(obj)^.mri_DrawInfo;
end;

function OBJ_Screen(obj : APTR) : pScreen;         (* valid between MUIM_Setup/Cleanup *)
begin
    OBJ_Screen := pMUI_RenderInfo(obj)^.mri_Screen;
end;

function OBJ_Pens(obj : APTR) : pWord;      (* valid between MUIM_Setup/Cleanup *)
begin
    OBJ_Pens := pMUI_RenderInfo(obj)^.mri_Pens;
end;

function OBJ_Window(obj : APTR) : pWindow;         (* valid between MUIM_Show/Hide *)
begin
    OBJ_Window := PMUI_RenderInfo(obj)^.mri_Window;
end;

function OBJ_Rp(obj : APTR) : pRastPort;           (* valid between MUIM_Show/Hide *)
begin
    OBJ_Rp := pMUI_RenderInfo(obj)^.mri_RastPort;
end;

function OBJ_Left(obj : APTR) : smallint;           (* valid during MUIM_Draw *)
begin
    OBJ_Left := pMUI_AreaData(obj)^.mad_Box.Left;
end;

function OBJ_Top(obj : APTR) : smallint;            (* valid during MUIM_Draw *)
begin
    OBJ_Top := pMUI_AreaData(obj)^.mad_Box.Top;
end;

function OBJ_Width(obj : APTR) : smallint;          (* valid during MUIM_Draw *)
begin
    OBJ_Width := pMUI_AreaData(obj)^.mad_Box.Width;
end;

function OBJ_Height(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
begin
    OBJ_Height := pMUI_AreaData(obj)^.mad_Box.Height;
end;

function OBJ_Right(obj : APTR) : smallint;          (* valid during MUIM_Draw *)
begin
    OBJ_Right := OBJ_Left(obj) + OBJ_Width(obj) -1;
end;

function OBJ_Bottom(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
begin
    OBJ_Bottom := OBJ_Top(obj) + OBJ_Height(obj) -1;
end;

function OBJ_AddLeft(obj : APTR) : smallint;        (* valid during MUIM_Draw *)
begin
    OBJ_AddLeft := pMUI_AreaData(obj)^.mad_AddLeft;
end;

function OBJ_AddTop(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
begin
    OBJ_AddTop := pMUI_AreaData(obj)^.mad_AddTop;
end;

function OBJ_SubWidth(obj : APTR) : smallint;       (* valid during MUIM_Draw *)
begin
    OBJ_SubWidth := pMUI_AreaData(obj)^.mad_SubWidth;
end;

function OBJ_SubHeight(obj : APTR) : smallint;      (* valid during MUIM_Draw *)
begin
    OBJ_SubHeight := pMUI_AreaData(obj)^.mad_SubHeight;
end;

function OBJ_MLeft(obj : APTR) : smallint;          (* valid during MUIM_Draw *)
begin
    OBJ_MLeft := OBJ_Left(obj) + OBJ_AddLeft(obj);
end;

function OBJ_MTop(obj : APTR) : smallint;           (* valid during MUIM_Draw *)
begin
    OBJ_MTop := OBJ_Top(obj) + OBJ_AddTop(obj);
end;

function OBJ_MWidth(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
begin
    OBJ_MWidth := OBJ_Width(obj) -OBJ_SubWidth(obj);
end;

function OBJ_MHeight(obj : APTR) : smallint;        (* valid during MUIM_Draw *)
begin
    OBJ_MHeight := OBJ_Height(obj) - OBJ_SubHeight(obj);
end;

function OBJ_MRight(obj : APTR) : smallint;         (* valid during MUIM_Draw *)
begin
    OBJ_MRight := OBJ_MLeft(obj) + OBJ_MWidth(obj) -1;
end;

function OBJ_MBottom(obj : APTR) : smallint;        (* valid during MUIM_Draw *)
begin
    OBJ_MBottom := OBJ_MTop(obj) + OBJ_MHeight(obj) -1;
end;

function OBJ_Font(obj : APTR) : pTextFont;         (* valid between MUIM_Setup/Cleanup *)
begin
    OBJ_Font := pMUI_AreaData(obj)^.mad_Font;
end;

function OBJ_MinWidth(obj : APTR) : ULONG;         (* valid between MUIM_Show/Hide *)
begin
    OBJ_MinWidth := pMUI_AreaData(obj)^.mad_MinMax.MinWidth;
end;

function OBJ_MinHeight(obj : APTR) : ULONG;        (* valid between MUIM_Show/Hide *)
begin
    OBJ_MinHeight := pMUI_AreaData(obj)^.mad_MinMax.MinHeight;
end;

function OBJ_MaxWidth(obj : APTR) : ULONG;         (* valid between MUIM_Show/Hide *)
begin
    OBJ_maxWidth := pMUI_AreaData(obj)^.mad_MinMax.MaxWidth;
end;

function OBJ_MaxHeight(obj : APTR) : ULONG;        (* valid between MUIM_Show/Hide *)
begin
    OBJ_maxHeight := pMUI_AreaData(obj)^.mad_MinMax.MaxHeight;
end;

function OBJ_DefWidth(obj : APTR) : ULONG;         (* valid between MUIM_Show/Hide *)
begin
    OBJ_DefWidth := pMUI_AreaData(obj)^.mad_MinMax.DefWidth;
end;

function OBJ_DefHeight(obj : APTR) : ULONG;        (* valid between MUIM_Show/Hide *)
begin
    OBJ_DefHeight := pMUI_AreaData(obj)^.mad_MinMax.DefHeight;
end;

function OBJ_Flags(obj : APTR) : ULONG;
begin
    OBJ_Flags := pMUI_AreaData(obj)^.mad_Flags;
end;

(*
** 2 useful procedures for testing if some coordinates are inside your object
** (converted from the ones in class3.c. So look there how to use... )
*)

function OBJ_Between(a,x,b : smallint): boolean;
begin
    OBJ_Between := ((x>=a) and (x<=b));
end;

function OBJ_IsInObject(x,y : smallint; obj : pObject_): boolean;
begin
    OBJ_IsInObject := (OBJ_Between(OBJ_MLeft(obj),x,OBJ_MRight(obj))
        and OBJ_Between(OBJ_MTop(obj),y,OBJ_MBottom(obj)));
end;

function MUIV_Window_AltHeight_MinMax(p : longint) : longint;
begin
    MUIV_Window_AltHeight_MinMax := (0 - p);
end;

function MUIV_Window_AltHeight_Visible(p : longint) : longint;
begin
    MUIV_Window_AltHeight_Visible := (-100 - (p));
end;

function MUIV_Window_AltHeight_Screen(p : longint) : longint;
begin
    MUIV_Window_AltHeight_Screen := (-200 - (p));
end;

function MUIV_Window_AltTopEdge_Delta(p : longint) : longint;
begin
    MUIV_Window_AltTopEdge_Delta := (-3 - (p));
end;

function MUIV_Window_AltWidth_MinMax(p : longint) : longint;
begin
    MUIV_Window_AltWidth_MinMax := 0 - p;
end;

function MUIV_Window_AltWidth_Visible(p : longint) : longint;
begin
    MUIV_Window_AltWidth_Visible := (-100 - (p));
end;

function MUIV_Window_AltWidth_Screen(p : longint) : longint;
begin
    MUIV_Window_AltWidth_Screen := (-200 - (p));
end;

function MUIV_Window_Height_MinMax(p : longint) : longint;
begin
    MUIV_Window_Height_MinMax := 0 - p;
end;

function MUIV_Window_Height_Visible(p : longint) : longint;
begin
    MUIV_Window_Height_Visible := (-100 - (p));
end;

function MUIV_Window_Height_Screen(p : longint) : longint;
begin
    MUIV_Window_Height_Screen := (-200 - (p));
end;

function MUIV_Window_TopEdge_Delta(p : longint) : longint;
begin
    MUIV_Window_TopEdge_Delta := (-3 - (p));
end;

function MUIV_Window_Width_MinMax(p : longint) : longint;
begin
    MUIV_Window_Width_MinMax := 0 - p;
end;

function MUIV_Window_Width_Visible(p : longint) : longint;
begin
    MUIV_Window_Width_Visible := (-100 - (p));
end;

function MUIV_Window_Width_Screen(p : longint) : longint;
begin
    MUIV_Window_Width_Screen := (-200 - (p));
end;

FUNCTION MUI_NewObjectA(class_ : pCHar; tags : pTagItem) : pObject_;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L class_,A0
        MOVEA.L tags,A1
        MOVEA.L MUIMasterBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MUI_DisposeObject(obj : pObject_);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L obj,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MUI_RequestA(app : POINTER; win : POINTER; flags : LONGBITS; title : pCHar; gadgets : pChar; format : pChar; params : POINTER) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  app,D0
        MOVE.L  win,D1
        MOVE.L  flags,D2
        MOVEA.L title,A0
        MOVEA.L gadgets,A1
        MOVEA.L format,A2
        MOVEA.L params,A3
        MOVEA.L MUIMasterBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MUI_AllocAslRequest(typ : ULONG; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  typ,D0
        MOVEA.L tags,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MUI_AslRequest(req : POINTER; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L req,A0
        MOVEA.L tags,A1
        MOVEA.L MUIMasterBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE MUI_FreeAslRequest(req : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L req,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MUI_Error : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L MUIMasterBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MUI_SetError(errnum : LONGINT) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  errnum,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MUI_GetClass(name : pChar) : pIClass;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MUI_FreeClass(cl : pIClass);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L cl,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MUI_RequestIDCMP(obj : pObject_; flags : ULONG);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L obj,A0
        MOVE.L  flags,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MUI_RejectIDCMP(obj : pObject_; flags : ULONG);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L obj,A0
        MOVE.L  flags,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MUI_Redraw(obj : pObject_; flags : ULONG);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L obj,A0
        MOVE.L  flags,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MUI_CreateCustomClass(base : pLibrary; supername : pChar; supermcc : pMUI_CustomClass; datasize : LONGINT; dispatcher : POINTER) : pMUI_CustomClass;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L base,A0
        MOVEA.L supername,A1
        MOVEA.L supermcc,A2
        MOVE.L  datasize,D0
        MOVEA.L dispatcher,A3
        MOVEA.L MUIMasterBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MUI_DeleteCustomClass(mcc : pMUI_CustomClass) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mcc,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MUI_MakeObjectA(typ : LONGINT; params : pULONG) : pObject_;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  typ,D0
        MOVEA.L params,A0
        MOVEA.L MUIMasterBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MUI_Layout(obj : pObject_; l : LONGINT; t : LONGINT; w : LONGINT; h : LONGINT; flags : ULONG) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L obj,A0
        MOVE.L  l,D0
        MOVE.L  t,D1
        MOVE.L  w,D2
        MOVE.L  h,D3
        MOVE.L  flags,D4
        MOVEA.L MUIMasterBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MUI_ObtainPen(mri : pMUI_RenderInfo; spec : pMUI_PenSpec; flags : ULONG) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVEA.L spec,A1
        MOVE.L  flags,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MUI_ReleasePen(mri : pMUI_RenderInfo; pen : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVE.L  pen,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MUI_AddClipping(mri : pMUI_RenderInfo; l : smallint; t : smallint; w : smallint; h : smallint) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVE.L  l,D0
        MOVE.L  t,D1
        MOVE.L  w,D2
        MOVE.L  h,D3
        MOVEA.L MUIMasterBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MUI_RemoveClipping(mri : pMUI_RenderInfo; h : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVEA.L h,A1
        MOVEA.L MUIMasterBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MUI_AddClipRegion(mri : pMUI_RenderInfo; region : pRegion) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVEA.L region,A1
        MOVEA.L MUIMasterBase,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MUI_RemoveClipRegion(mri : pMUI_RenderInfo; region : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVEA.L region,A1
        MOVEA.L MUIMasterBase,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MUI_BeginRefresh(mri : pMUI_RenderInfo; flags : ULONG) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVE.L  flags,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -192(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE MUI_EndRefresh(mri : pMUI_RenderInfo; flags : ULONG);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mri,A0
        MOVE.L  flags,D0
        MOVEA.L MUIMasterBase,A6
        JSR     -198(A6)
        MOVEA.L (A7)+,A6
  END;
END;


{
 Functions and procedures with array of const go here
}
FUNCTION MUI_AllocAslRequestTags(_type : longword; const tags : Array Of Const) : POINTER;
begin
    MUI_AllocAslRequestTags := MUI_AllocAslRequest(_type , readintags(tags));
end;

FUNCTION MUI_AslRequestTags(req : POINTER; const tags : Array Of Const) : BOOLEAN;
begin
    MUI_AslRequestTags := MUI_AslRequest(req , readintags(tags));
end;

FUNCTION MUI_MakeObject(_type : LONGINT; const params : Array Of Const) : pULONG;
begin
    MUI_MakeObject := MUI_MakeObjectA(_type , readinlongs(params));
end;

FUNCTION MUI_NewObject(a0arg : pCHAR; const tags : Array Of Const) : pULONG;
begin
    MUI_NewObject := MUI_NewObjectA(a0arg , readintags(tags));
end;

FUNCTION MUI_Request(app : POINTER; win : POINTER; flags : longword; title : pCHAR; gadgets : pCHAR; format : pCHAR; const params : Array Of Const) : LONGINT;
begin
    MUI_Request := MUI_RequestA(app , win , flags , title , gadgets , format , readintags(params));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of muimaster.library}
  {$Info don't forget to use InitMUIMASTERLibrary in the beginning of your program}

var
    muimaster_exit : Pointer;

procedure ClosemuimasterLibrary;
begin
    ExitProc := muimaster_exit;
    if MUIMasterBase <> nil then begin
        CloseLibrary(MUIMasterBase);
        MUIMasterBase := nil;
    end;
end;

procedure InitMUIMASTERLibrary;
begin
    MUIMasterBase := nil;
    MUIMasterBase := OpenLibrary(MUIMASTER_NAME,LIBVERSION);
    if MUIMasterBase <> nil then begin
        muimaster_exit := ExitProc;
        ExitProc := @ClosemuimasterLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open muimaster.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    MUIMASTERIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of muimaster.library}

var
    muimaster_exit : Pointer;

procedure ClosemuimasterLibrary;
begin
    ExitProc := muimaster_exit;
    if MUIMasterBase <> nil then begin
        CloseLibrary(MUIMasterBase);
        MUIMasterBase := nil;
    end;
end;

begin
    MUIMasterBase := nil;
    MUIMasterBase := OpenLibrary(MUIMASTER_NAME,LIBVERSION);
    if MUIMasterBase <> nil then begin
        muimaster_exit := ExitProc;
        ExitProc := @ClosemuimasterLibrary;
        MUIMASTERIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open muimaster.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    MUIMASTERIsCompiledHow := 3;
   {$Warning No autoopening of muimaster.library compiled}
   {$Warning Make sure you open muimaster.library yourself}
{$endif dont_use_openlib}


end.
