{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2002 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
    History:
    Added overlay functions for Pchar->Strings, functions
    and procedures.
    14 Jul 2000.

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening
    of the library.
    13 Jan 2003.

    changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

unit commodities;

INTERFACE


uses exec, inputevent, keymap;



{    **************
 * Broker stuff
 **************}

CONST
{     buffer sizes   }
      CBD_NAMELEN   =  24;
      CBD_TITLELEN  =  40;
      CBD_DESCRLEN  =  40;

{     CxBroker errors   }
      CBERR_OK      =  0;        {     No error                         }
      CBERR_SYSERR  =  1;        {     System error , no memory, etc    }
      CBERR_DUP     =  2;        {     uniqueness violation             }
      CBERR_VERSION =  3;        {     didn't understand nb_VERSION     }

      NB_VERSION    =  5;        {     Version of NewBroker structure   }

Type
  pNewBroker = ^tNewBroker;
  tNewBroker = record
   nb_Version   : Shortint;  {     set to NB_VERSION                }
   nb_Name,
   nb_Title,
   nb_Descr     : STRPTR;
   nb_Unique,
   nb_Flags     : smallint;
   nb_Pri       : Shortint;
   {     new in V5   }
   nb_Port      : pMsgPort;
   nb_ReservedChannel  : smallint;  {     plans for later port sharing     }
  END;

CONST
{     Flags for nb_Unique }
      NBU_DUPLICATE  = 0;
      NBU_UNIQUE     = 1;        {     will not allow duplicates        }
      NBU_NOTIFY     = 2;        {     sends CXM_UNIQUE to existing broker }

{     Flags for nb_Flags }
        COF_SHOW_HIDE = 4;

{    *******
 * cxusr
 *******}

{    * Fake data types for system private objects   }
Type
  CxObj = Longint;
  pCxObj = ^CxObj;
  CxMsg = Longint;
  pCXMsg = ^CxMsg;


CONST
{    ******************************}
{    * Commodities Object Types   *}
{    ******************************}
      CX_INVALID     = 0;     {     not a valid object (probably null)  }
      CX_FILTER      = 1;     {     input event messages only           }
      CX_TYPEFILTER  = 2;     {     filter on message type      }
      CX_SEND        = 3;     {     sends a message                     }
      CX_SIGNAL      = 4;     {     sends a signal              }
      CX_TRANSLATE   = 5;     {     translates IE into chain            }
      CX_BROKER      = 6;     {     application representative          }
      CX_DEBUG       = 7;     {     dumps kprintf to serial port        }
      CX_CUSTOM      = 8;     {     application provids function        }
      CX_ZERO        = 9;     {     system terminator node      }

{    ***************}
{    * CxMsg types *}
{    ***************}
      CXM_UNIQUE     = 16;    {     sent down broker by CxBroker()      }
{     Obsolete: subsumed by CXM_COMMAND (below)   }

{     Messages of this type rattle around the Commodities input network.
 * They will be sent to you by a Sender object, and passed to you
 * as a synchronous function call by a Custom object.
 *
 * The message port or function entry point is stored in the object,
 * and the ID field of the message will be set to what you arrange
 * issuing object.
 *
 * The Data field will point to the input event triggering the
 * message.
 }
      CXM_IEVENT     = 32;

{     These messages are sent to a port attached to your Broker.
 * They are sent to you when the controller program wants your
 * program to do something.  The ID field identifies the command.
 *
 * The Data field will be used later.
 }
      CXM_COMMAND    = 64;

{     ID values   }
      CXCMD_DISABLE   = (15);   {     please disable yourself       }
      CXCMD_ENABLE    = (17);   {     please enable yourself        }
      CXCMD_APPEAR    = (19);   {     open your window, if you can  }
      CXCMD_DISAPPEAR = (21);   {     go dormant                    }
      CXCMD_KILL      = (23);   {     go away for good              }
      CXCMD_UNIQUE    = (25);   {     someone tried to create a broker
                               * with your name.  Suggest you Appear.
                               }
      CXCMD_LIST_CHG  = (27);  {     Used by Exchange program. Someone }
                              {     has changed the broker list       }

{     return values for BrokerCommand(): }
      CMDE_OK        = (0);
      CMDE_NOBROKER  = (-1);
      CMDE_NOPORT    = (-2);
      CMDE_NOMEM     = (-3);

{     IMPORTANT NOTE: for V5:
 * Only CXM_IEVENT messages are passed through the input network.
 *
 * Other types of messages are sent to an optional port in your broker.
 *
 * This means that you must test the message type in your message handling,
 * if input messages and command messages come to the same port.
 *
 * Older programs have no broker port, so processing loops which
 * make assumptions about type won't encounter the new message types.
 *
 * The TypeFilter CxObject is hereby obsolete.
 *
 * It is less convenient for the application, but eliminates testing
 * for type of input messages.
 }

{    ********************************************************}
{    * CxObj Error Flags (return values from CxObjError())  *}
{    ********************************************************}
      COERR_ISNULL      = 1;  {     you called CxError(NULL)            }
      COERR_NULLATTACH  = 2;  {     someone attached NULL to my list    }
      COERR_BADFILTER   = 4;  {     a bad filter description was given  }
      COERR_BADTYPE     = 8;  {     unmatched type-specific operation   }


{    ****************************}
{     Input Expression structure }
{    ****************************}

      IX_VERSION        = 2;

Type
  pInputXpression = ^tInputXpression;
  tInputXpression = record
   ix_Version,               {     must be set to IX_VERSION  }
   ix_Class    : Byte;       {     class must match exactly   }

   ix_Code     : Word;      {     Bits that we want  }

   ix_CodeMask : Word;      {     Set bits here to indicate  }
                             {     which bits in ix_Code are  }
                             {     don't care bits.           }

   ix_Qualifier: Word;      {     Bits that we want  }

   ix_QualMask : Word;      {     Set bits here to indicate  }
                           {     which bits in ix_Qualifier }
                                                   {     are don't care bits        }

   ix_QualSame : Word;    {     synonyms in qualifier      }
  END;

   IX = tInputXpression;
   pIX = ^IX;

CONST
{     QualSame identifiers }
      IXSYM_SHIFT = 1;     {     left- and right- shift are equivalent     }
      IXSYM_CAPS  = 2;     {     either shift or caps lock are equivalent  }
      IXSYM_ALT   = 4;     {     left- and right- alt are equivalent       }

{     corresponding QualSame masks }
      IXSYM_SHIFTMASK = (IEQUALIFIER_LSHIFT + IEQUALIFIER_RSHIFT);
      IXSYM_CAPSMASK  = (IXSYM_SHIFTMASK    + IEQUALIFIER_CAPSLOCK);
      IXSYM_ALTMASK   = (IEQUALIFIER_LALT   + IEQUALIFIER_RALT);

      IX_NORMALQUALS  = $7FFF;   {     for QualMask field: avoid RELATIVEMOUSE }


VAR CxBase : pLibrary;

const
    COMMODITIESNAME : PChar = 'commodities.library';

FUNCTION ActivateCxObj(co : pCxObj; tru : LONGINT) : LONGINT;
PROCEDURE AddIEvents(events : pInputEvent);
PROCEDURE AttachCxObj(headObj : pCxObj; co : pCxObj);
PROCEDURE ClearCxObjError(co : pCxObj);
FUNCTION CreateCxObj(typ : ULONG; arg1 : LONGINT; arg2 : LONGINT): pCxObj;
FUNCTION CxBroker(nb : pNewBroker; error : pCxObj) : pCxObj;
FUNCTION CxMsgData(cxm : pCxMsg) : POINTER;
FUNCTION CxMsgID(cxm : pCxMsg) : LONGINT;
FUNCTION CxMsgType(cxm : pCxMsg) : ULONG;
FUNCTION CxObjError(co : pCxObj) : LONGINT;
FUNCTION CxObjType(co : pCxObj) : ULONG;
PROCEDURE DeleteCxObj(co : pCxObj);
PROCEDURE DeleteCxObjAll(co : pCxObj);
PROCEDURE DisposeCxMsg(cxm : pCxMsg);
PROCEDURE DivertCxMsg(cxm : pCxMsg; headObj : pCxObj; returnObj : pCxObj);
PROCEDURE EnqueueCxObj(headObj : pCxObj; co : pCxObj);
PROCEDURE InsertCxObj(headObj : pCxObj; co : pCxObj; pred : pCxObj);
FUNCTION InvertKeyMap(ansiCode : ULONG; event : pInputEvent; km : pKeyMap) : BOOLEAN;
FUNCTION MatchIX(event : pInputEvent; ix : pInputXpression) : BOOLEAN;
FUNCTION ParseIX(description : pCHAR; ix : pInputXpression) : LONGINT;
PROCEDURE RemoveCxObj(co : pCxObj);
PROCEDURE RouteCxMsg(cxm : pCxMsg; co : pCxObj);
FUNCTION SetCxObjPri(co : pCxObj; pri : LONGINT) : LONGINT;
PROCEDURE SetFilter(filter : pCxObj; text : pCHAR);
PROCEDURE SetFilterIX(filter : pCxObj; ix : pInputXpression);
PROCEDURE SetTranslate(translator : pCxObj; events : pInputEvent);

{ overlay functions}

FUNCTION ParseIX(description : string; ix : pInputXpression) : LONGINT;
PROCEDURE SetFilter(filter : pCxObj; text : string);


IMPLEMENTATION

uses pastoc,msgbox;

FUNCTION ActivateCxObj(co : pCxObj; tru : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVE.L  tru,D0
    MOVEA.L CxBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE AddIEvents(events : pInputEvent);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L events,A0
    MOVEA.L CxBase,A6
    JSR -180(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AttachCxObj(headObj : pCxObj; co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L headObj,A0
    MOVEA.L co,A1
    MOVEA.L CxBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ClearCxObjError(co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVEA.L CxBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CreateCxObj(typ : ULONG; arg1 : LONGINT; arg2 : LONGINT) : pCxObj;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  typ,D0
    MOVEA.L arg1,A0
    MOVEA.L arg2,A1
    MOVEA.L CxBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CxBroker(nb : pNewBroker; error : pCxObj) : pCxObj;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L nb,A0
    MOVE.L  error,D0
    MOVEA.L CxBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CxMsgData(cxm : pCxMsg) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L cxm,A0
    MOVEA.L CxBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CxMsgID(cxm : pCxMsg) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L cxm,A0
    MOVEA.L CxBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CxMsgType(cxm : pCxMsg) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L cxm,A0
    MOVEA.L CxBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CxObjError(co : pCxObj) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVEA.L CxBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CxObjType(co : pCxObj) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVEA.L CxBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DeleteCxObj(co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVEA.L CxBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteCxObjAll(co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVEA.L CxBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DisposeCxMsg(cxm : pCxMsg);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L cxm,A0
    MOVEA.L CxBase,A6
    JSR -168(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DivertCxMsg(cxm : pCxMsg; headObj : pCxObj; returnObj : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L cxm,A0
    MOVEA.L headObj,A1
    MOVEA.L returnObj,A2
    MOVEA.L CxBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE EnqueueCxObj(headObj : pCxObj; co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L headObj,A0
    MOVEA.L co,A1
    MOVEA.L CxBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE InsertCxObj(headObj : pCxObj; co : pCxObj; pred : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L headObj,A0
    MOVEA.L co,A1
    MOVEA.L pred,A2
    MOVEA.L CxBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION InvertKeyMap(ansiCode : ULONG; event : pInputEvent; km : pKeyMap) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  ansiCode,D0
    MOVEA.L event,A0
    MOVEA.L km,A1
    MOVEA.L CxBase,A6
    JSR -174(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MatchIX(event : pInputEvent; ix : pInputXpression) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L event,A0
    MOVEA.L ix,A1
    MOVEA.L CxBase,A6
    JSR -204(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION ParseIX(description : pCHAR; ix : pInputXpression) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L description,A0
    MOVEA.L ix,A1
    MOVEA.L CxBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RemoveCxObj(co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVEA.L CxBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RouteCxMsg(cxm : pCxMsg; co : pCxObj);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L cxm,A0
    MOVEA.L co,A1
    MOVEA.L CxBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetCxObjPri(co : pCxObj; pri : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L co,A0
    MOVE.L  pri,D0
    MOVEA.L CxBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SetFilter(filter : pCxObj; text : pCHAR);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L filter,A0
    MOVEA.L text,A1
    MOVEA.L CxBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SetFilterIX(filter : pCxObj; ix : pInputXpression);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L filter,A0
    MOVEA.L ix,A1
    MOVEA.L CxBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SetTranslate(translator : pCxObj; events : pInputEvent);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L translator,A0
    MOVEA.L events,A1
    MOVEA.L CxBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;


FUNCTION ParseIX(description : string; ix : pInputXpression) : LONGINT;
begin
      ParseIX := ParseIX(pas2c(description),ix);
end;

PROCEDURE SetFilter(filter : pCxObj; text : string);
begin
      SetFilter(filter,pas2c(text));
end;

{$I useautoopenlib.inc}
{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of commodities.library}

var
    commodities_exit : Pointer;

procedure ClosecommoditiesLibrary;
begin
    ExitProc := commodities_exit;
    if CxBase <> nil then begin
        CloseLibrary(CxBase);
        CxBase := nil;
    end;
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

begin
    CxBase := nil;
    CxBase := OpenLibrary(COMMODITIESNAME,LIBVERSION);
    if CxBase <> nil then begin
        commodities_exit := ExitProc;
        ExitProc := @ClosecommoditiesLibrary
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open commodities.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$else}
   {$Warning No autoopening of commodities.library compiled}
   {$Info Make sure you open commodities.library yourself}
{$endif use_auto_openlib}

END. (* UNIT COMMODITIES *)



