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
    History:

    Added functions and procedures with array of const.
    For use with fpc 1.0.7. Thay are in systemvartags.
    11 Nov 2002.


    Added the define use_amiga_smartlink.
    13 Jan 2003.

    Update for AmigaOS 3.9.
    Added a few overlays.
    06 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit utility;

INTERFACE
uses exec;


Type
      pClockData = ^tClockData;
      tClockData = record
        sec   : Word;
        min   : Word;
        hour  : Word;
        mday  : Word;
        month : Word;
        year  : Word;
        wday  : Word;
      END;

      pHook = ^tHook;
      tHook = record
        h_MinNode  : tMinNode;
        h_Entry    : Pointer;   { assembler entry point        }
        h_SubEntry : Pointer;   { often HLL entry point        }
        h_Data     : Pointer;   { owner specific               }
      END;

{
 * Hook calling conventions:
 *      A0 - pointer to hook data structure itself
 *      A1 - pointer to parameter structure ("message") typically
 *           beginning with a longword command code, which makes
 *           sense in the context in which the hook is being used.
 *      A2 - Hook specific address data ("object," e.g, GadgetInfo)
 *
 * Control will be passed to the routine h_Entry.  For many
 * High-Level Languages (HLL), this will be an assembly language
 * stub which pushes registers on the stack, does other setup,
 * and then calls the function at h_SubEntry.
 *
 * The C standard receiving code is:
 * CDispatcher( hook, object, message )
 *     struct Hook      *hook;
 *     APTR             object;
 *     APTR             message;
 *
 * NOTE that register natural order differs from this convention
 * for C parameter order, which is A0,A2,A1.
 *
 * The assembly language stub for "vanilla" C parameter conventions
 * could be:

 _hookEntry:
        move.l  a1,-(sp)                ; push message packet pointer
        move.l  a2,-(sp)                ; push object pointer
        move.l  a0,-(sp)                ; push hook pointer
        move.l  h_SubEntry(a0),a0       ; fetch C entry point ...
        jsr     (a0)                    ; ... and call it
        lea     12(sp),sp               ; fix stack
        rts

 * with this function as your interface stub, you can write
 * a Hook setup function as:

 SetupHook( hook, c_function, userdata )
 struct Hook    *hook;
 ULONG          (*c_function)();
 VOID           *userdata;

        ULONG   (*hookEntry)();

        hook->h_Entry =         hookEntry;
        hook->h_SubEntry =      c_function;
        hook->h_Data =                  userdata;


 * with Lattice C pragmas, you can put the C function in the
 * h_Entry field directly if you declare the function:

ULONG __saveds __asm
CDispatcher(    register __a0 struct Hook       *hook,
                register __a2 VOID              *object,
                register __a1 ULONG             *message );
 *
 ***}

 {      Namespace definitions      }


Type
{ The named object structure }
 pNamedObject = ^tNamedObject;
 tNamedObject = record
    no_Object  : Pointer;       { Your pointer, for whatever you want }
 END;

const
{ Tags for AllocNamedObject() }
 ANO_NameSpace  = 4000;    { Tag to define namespace      }
 ANO_UserSpace  = 4001;    { tag to define userspace      }
 ANO_Priority   = 4002;    { tag to define priority       }
 ANO_Flags      = 4003;    { tag to define flags          }

{ Flags for tag ANO_Flags }
 NSB_NODUPS     = 0;
 NSB_CASE       = 1;

 NSF_NODUPS     = 1;      { Default allow duplicates }
 NSF_CASE       = 2;      { Default to caseless... }


   {    Control attributes for Pack/UnpackStructureTags() }


{ PackTable definition:
 *
 * The PackTable is a simple array of LONGWORDS that are evaluated by
 * PackStructureTags() and UnpackStructureTags().
 *
 * The table contains compressed information such as the tag offset from
 * the base tag. The tag offset has a limited range so the base tag is
 * defined in the first longword.
 *
 * After the first longword, the fields look as follows:
 *
 *      +--------- 1 = signed, 0 = unsigned (for bits, 1=inverted boolean)
 *      |
 *      |  +------ 00 = Pack/Unpack, 10 = Pack, 01 = Unpack, 11 = special
 *      | / \
 *      | | |  +-- 00 = Byte, 01 = Integer, 10 = Long, 11 = Bit
 *      | | | / \
 *      | | | | | /----- For bit operations: 1 = TAG_EXISTS is TRUE
 *      | | | | | |
 *      | | | | | | /-------------------- Tag offset from base tag value
 *      | | | | | | |                 \
 *      m n n o o p q q q q q q q q q q r r r s s s s s s s s s s s s s
 *                                      \   | |               |
 *      Bit offset (for bit operations) ----/ |               |
 *                                            \                       |
 *      Offset into data structure -----------------------------------/
 *
 * A -1 longword signifies that the next longword will be a new base tag
 *
 * A 0 longword signifies that it is the end of the pack table.
 *
 * What this implies is that there are only 13-bits of address offset
 * and 10 bits for tag offsets from the base tag.  For most uses this
 * should be enough, but when this is not, either multiple pack tables
 * or a pack table with extra base tags would be able to do the trick.
 * The goal here was to make the tables small and yet flexible enough to
 * handle most cases.
 }

const
 PSTB_SIGNED =31;
 PSTB_UNPACK =30;    { Note that these are active low... }
 PSTB_PACK   =29;    { Note that these are active low... }
 PSTB_EXISTS =26;    { Tag exists bit true flag hack...  }

 PSTF_SIGNED = $80000000;
 PSTF_UNPACK = $40000000;
 PSTF_PACK   = $20000000;

 PSTF_EXISTS = $4000000;


{***************************************************************************}


 PKCTRL_PACKUNPACK = $00000000;
 PKCTRL_PACKONLY   = $40000000;
 PKCTRL_UNPACKONLY = $20000000;

 PKCTRL_BYTE       = $80000000;
 PKCTRL_WORD       = $88000000;
 PKCTRL_LONG       = $90000000;

 PKCTRL_UBYTE      = $00000000;
 PKCTRL_UWORD      = $08000000;
 PKCTRL_ULONG      = $10000000;

 PKCTRL_BIT        = $18000000;
 PKCTRL_FLIPBIT    = $98000000;


{***************************************************************************}


{ Macros used by the next batch of macros below. Normally, you don't use
 * this batch directly. Then again, some folks are wierd
 }



{***************************************************************************}


{ Some handy dandy macros to easily create pack tables
 *
 * Use PACK_STARTTABLE() at the start of a pack table. You pass it the
 * base tag value that will be handled in the following chunk of the pack
 * table.
 *
 * PACK_ENDTABLE() is used to mark the end of a pack table.
 *
 * PACK_NEWOFFSET() lets you change the base tag value used for subsequent
 * entries in the table
 *
 * PACK_ENTRY() lets you define an entry in the pack table. You pass it the
 * base tag value, the tag of interest, the type of the structure to use,
 * the field name in the structure to affect and control bits (combinations of
 * the various PKCTRL_XXX bits)
 *
 * PACK_BYTEBIT() lets you define a bit-control entry in the pack table. You
 * pass it the same data as PACK_ENTRY, plus the flag bit pattern this tag
 * affects. This macro should be used when the field being affected is byte
 * sized.
 *
 * PACK_WORDBIT() lets you define a bit-control entry in the pack table. You
 * pass it the same data as PACK_ENTRY, plus the flag bit pattern this tag
 * affects. This macro should be used when the field being affected is Integer
 * sized.
 *
 * PACK_LONGBIT() lets you define a bit-control entry in the pack table. You
 * pass it the same data as PACK_ENTRY, plus the flag bit pattern this tag
 * affects. This macro should be used when the field being affected is longword
 * sized.
 *
 * EXAMPLE:
 *
 *    ULONG packTable[] =
 *    (
 *         PACK_STARTTABLE(GA_Dummy),
 *         PACK_ENTRY(GA_Dummy,GA_Left,Gadget,LeftEdge,PKCTRL_WORD|PKCTRL_PACKUNPACK),
 *         PACK_ENTRY(GA_Dummy,GA_Top,Gadget,TopEdge,PKCTRL_WORD|PKCTRL_PACKUNPACK),
 *         PACK_ENTRY(GA_Dummy,GA_Width,Gadget,Width,PKCTRL_UWORD|PKCTRL_PACKUNPACK),
 *         PACK_ENTRY(GA_Dummy,GA_Height,Gadget,Height,PKCTRL_UWORD|PKCTRL_PACKUNPACK),
 *         PACK_WORDBIT(GA_Dummy,GA_RelVerify,Gadget,Activation,PKCTRL_BIT|PKCTRL_PACKUNPACK,GACT_RELVERIFY)
 *         PACK_ENDTABLE
 *    );
 }


{ ======================================================================= }
{ ==== TagItem ========================================================== }
{ ======================================================================= }
{ This data type may propagate through the system for more general use.
 * In the meantime, it is used as a general mechanism of extensible data
 * arrays for parameter specification and property inquiry (coming soon
 * to a display controller near you).
 *
 * In practice, an array (or chain of arrays) of TagItems is used.
 }
Type
    Tag = LongInt;
    pTag = ^Tag;

    pTagItem = ^tTagItem;
    tTagItem = record
     ti_Tag  : Tag;
     ti_Data : LongInt;
    END;

    ppTagItem = ^pTagItem;

{ ---- system tag values ----------------------------- }
CONST
 TAG_DONE          = 0; { terminates array of TagItems. ti_Data unused }
 TAG_END           = TAG_DONE;
 TAG_IGNORE        = 1; { ignore this item, not END of array           }
 TAG_MORE          = 2; { ti_Data is pointer to another array of TagItems
                         * note that this tag terminates the current array
                         }
 TAG_SKIP          = 3; { skip this AND the next ti_Data items         }

{ differentiates user tags from control tags }
 TAG_USER          = $80000000;    { differentiates user tags from system tags}

{* If the TAG_USER bit is set in a tag number, it tells utility.library that
 * the tag is not a control tag (like TAG_DONE, TAG_IGNORE, TAG_MORE) and is
 * instead an application tag. "USER" means a client of utility.library in
 * general, including system code like Intuition or ASL, it has nothing to do
 * with user code.
 *}


{ Tag filter logic specifiers for use with FilterTagItems() }
 TAGFILTER_AND     = 0;       { exclude everything but filter hits   }
 TAGFILTER_NOT     = 1;       { exclude only filter hits             }

{ Mapping types for use with MapTags() }
 MAP_REMOVE_NOT_FOUND = 0;  { remove tags that aren't in mapList }
 MAP_KEEP_NOT_FOUND   = 1;  { keep tags that aren't in mapList   }



Type
 pUtilityBase = ^tUtilityBase;
 tUtilityBase = record
    ub_LibNode   : tLibrary;
    ub_Language  : Byte;
    ub_Reserved  : Byte;
 END;

function AddNamedObject(nameSpace,obj : pNamedObject) : Boolean;
function AllocateTagItems(num : ULONG) : pTagItem;
function AllocNamedObjectA(const name : STRPTR;const TagList : pTagItem) : pNamedObject;
procedure Amiga2Date(amigatime : ULONG;resultat : pClockData);
procedure ApplyTagChanges(TagList : pTagItem; const ChangeList : pTagItem);
function AttemptRemNamedObject(obj : pNamedObject) : LongInt;
function CallHookPkt(h : pHook;obj, paramPkt : APTR) : ULONG;
function CheckDate(const date : pClockData) : ULONG;
function CloneTagItems(const tagList : pTagItem) : pTagItem;
function Date2Amiga(const date : pClockData) : ULONG;
procedure FilterTagChanges(changelist, oldvalues : pTagItem;apply : ULONG);
function FilterTagItems(taglist : pTagItem ;const tagArray : pULONG;logic : ULONG) : ULONG;
function FindNamedObject(nameSpace : pNamedObject;const name : STRPTR;lastobject: pNamedObject) : pNamedObject;
function FindTagItem(TagVal : Tag;const TagList : pTagItem) : pTagItem;
procedure FreeNamedObject(Obj : pNamedObject);
procedure FreeTagItems(TagList : pTagItem);
function GetTagData(tagval : Tag;default : ULONG;const TagList : pTagItem) : ULONG;
function GetUniqueID : ULONG;
procedure MapTags(TagList : pTagItem;const maplist : pTagItem;IncludeMiss : ULONG);
function NamedObjectName(Obj : pNamedObject) : STRPTR;
function NextTagItem(Item : ppTagItem) : pTagItem;
function PackBoolTags(InitialFlags : ULONG;const TagList, boolmap : pTagItem) : ULONG;
function PackStructureTags(packk: APTR;const packTable : pULONG;const TagList : pTagItem) : ULONG;
procedure RefreshTagItemClones(cloneTagItem : pTagItem; const OriginalTagItems : pTagItem);
procedure ReleaseNamedObject(Obj : pNamedObject);
procedure RemNamedObject(Obj : pNamedObject;Msg : pointer);
function SDivMod32( dividend , divisor : LongInt) : LongInt;
function SMult32(Arg1, Arg2 : LongInt) : LongInt;
function SMult64(Arg1, Arg2 : LongInt) : LongInt;
function Stricmp(const Str1: STRPTR;const Str2 : STRPTR) : LongInt;
function Strnicmp(const Str1: STRPTR;const Str2 : STRPTR;len : LongInt) : LongInt;
function TagInArray(t : Tag;const TagArray : pULONG) : Boolean;
function ToLower(c : ULONG) : Char;
function ToUpper(c : ULONG) : Char;
function UDivMod32( dividend , divisor : ULONG) : ULONG;
function UMult32(Arg1, Arg2 : ULONG) : ULONG;
function UMult64(Arg1, Arg2 : ULONG) : ULONG;
function UnpackStructureTags(const pac: APTR;const packTable: pULONG;TagList : pTagItem) : ULONG;

function AllocNamedObjectA(const name : string;const TagList : pTagItem) : pNamedObject;
FUNCTION FindNamedObject(nameSpace : pNamedObject; CONST name : string; lastObject : pNamedObject) : pNamedObject;
FUNCTION Stricmp(CONST string1 : string; CONST string2 : pCHAR) : LONGINT;
FUNCTION Stricmp(CONST string1 : pCHAR; CONST string2 : string) : LONGINT;
FUNCTION Stricmp(CONST string1 : string; CONST string2 : string) : LONGINT;
FUNCTION Strnicmp(CONST string1 : string; CONST string2 : pCHAR; length : LONGINT) : LONGINT;
FUNCTION Strnicmp(CONST string1 : pCHAR; CONST string2 : string; length : LONGINT) : LONGINT;
FUNCTION Strnicmp(CONST string1 : string; CONST string2 : string; length : LONGINT) : LONGINT;


IMPLEMENTATION

uses pastoc;

function AddNamedObject(nameSpace,obj : pNamedObject) : Boolean;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  nameSpace,a0
       MOVE.L  obj,a1
       MOVE.L  _UtilityBase,A6
       JSR -222(A6)
       MOVE.L  (A7)+,A6
       TST.L   d0
       bne     @success
       bra     @end
   @success:
       move.b  #1,d0
   @end:
       move.b  d0,@RESULT
   end;
end;

function AllocateTagItems(num : ULONG) : pTagItem;
begin
  asm
      MOVE.L  A6,-(A7)
      MOVE.L  num,d0
      MOVE.L  _UtilityBase,A6
      JSR -066(A6)
      MOVE.L  (A7)+,A6
      MOVE.L  d0,@RESULT
  end;
end;

function AllocNamedObjectA(const name : STRPTR;const TagList : pTagItem) : pNamedObject;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  name,a0
       MOVE.L  TagList,a1
       MOVE.L  _UtilityBase,A6
       JSR -228(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

procedure Amiga2Date(amigatime : ULONG;resultat : pClockData);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  amigatime,d0
       MOVE.L  resultat,a0
       MOVE.L  _UtilityBase,A6
       JSR -120(A6)
       MOVE.L  (A7)+,A6
   end;
end;

procedure ApplyTagChanges(TagList : pTagItem;const ChangeList : pTagItem);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  TagList,a0
       MOVE.L  ChangeList,a1
       MOVE.L  _UtilityBase,A6
       JSR -186(A6)
       MOVE.L  (A7)+,A6
   end;
end;

function AttemptRemNamedObject(obj : pNamedObject) : LongInt;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  obj,a0
       MOVE.L  _UtilityBase,A6
       JSR -234(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function CallHookPkt(h : pHook;obj, paramPkt : APTR) : ULONG;
begin
   asm
       MOVEM.L a2/a6,-(A7)
       MOVE.L  h,a0
       MOVE.L  obj,a2
       MOVE.L  paramPkt,a1
       MOVE.L  _UtilityBase,A6
       JSR -102(A6)
       MOVEM.L (A7)+,a2/a6
       MOVE.L  d0,@RESULT
   end;
end;

function CheckDate(const date : pClockData) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  date,a0
       MOVE.L  _UtilityBase,A6
       JSR -132(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function CloneTagItems(const tagList : pTagItem) : pTagItem;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  taglist,a0
       MOVE.L  _UtilityBase,A6
       JSR -072(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function Date2Amiga(const date : pClockData) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  date,a0
       MOVE.L  _UtilityBase,A6
       JSR -126(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

procedure FilterTagChanges(changelist, oldvalues : pTagItem;apply : ULONG);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  changelist,a0
       MOVE.L  oldvalues,a1
       MOVE.L  apply,d0
       MOVE.L  _UtilityBase,A6
       JSR -054(A6)
       MOVE.L  (A7)+,A6
   end;
end;

function FilterTagItems(taglist : pTagItem ;const tagArray : pULONG;logic : ULONG) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  taglist,a0
       MOVE.L  tagArray,a1
       MOVE.L  logic,d0
       MOVE.L  _UtilityBase,A6
       JSR -096(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function FindNamedObject(nameSpace : pNamedObject;const name : STRPTR;lastobject: pNamedObject) : pNamedObject;
begin
   asm
       MOVEM.L a2/a6,-(A7)
       MOVE.L  nameSpace,a0
       MOVE.L  name,a1
       MOVE.L  lastobject,a2
       MOVE.L  _UtilityBase,A6
       JSR -240(A6)
       MOVEM.L (A7)+,a2/a6
       MOVE.L  d0,@RESULT
   end;
end;

function FindTagItem(TagVal : Tag;const TagList : pTagItem) : pTagItem;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  TagVal,d0
       MOVE.L  TagList,a0
       MOVE.L  _UtilityBase,A6
       JSR -030(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

procedure FreeNamedObject(Obj : pNamedObject);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Obj,a0
       MOVE.L  _UtilityBase,A6
       JSR -246(A6)
       MOVE.L  (A7)+,A6
   end;
end;

procedure FreeTagItems(TagList : pTagItem);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  TagList,a0
       MOVE.L  _UtilityBase,A6
       JSR -078(A6)
       MOVE.L  (A7)+,A6
   end;
end;

function GetTagData(tagval : Tag;default : ULONG;const TagList : pTagItem) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  tagval,d0
       MOVE.L  default,d1
       MOVE.L  TagList,a0
       MOVE.L  _UtilityBase,A6
       JSR -036(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function GetUniqueID : ULONG;
begin
   asm
      MOVE.L  A6,-(A7)
      MOVE.L  _UtilityBase,A6
      JSR -270(A6)
      MOVE.L  (A7)+,A6
      MOVE.L  d0,@RESULT
   end;
end;

procedure MapTags(TagList : pTagItem;const maplist : pTagItem;IncludeMiss : ULONG);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  TagList,a0
       MOVE.L  maplist,a1
       MOVE.L  IncludeMiss,d0
       MOVE.L  _UtilityBase,A6
       JSR -060(A6)
       MOVE.L  (A7)+,A6
   end;
end;

function NamedObjectName(Obj : pNamedObject) : STRPTR;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Obj,a0
       MOVE.L  _UtilityBase,A6
       JSR -252(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function NextTagItem(Item : ppTagItem) : pTagItem;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Item,a0
       MOVE.L  _UtilityBase,A6
       JSR -048(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function PackBoolTags(InitialFlags : ULONG;const TagList, boolmap : pTagItem) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  InitialFlags,d0
       MOVE.L  TagList,a0
       MOVE.L  boolmap,a1
       MOVE.L  _UtilityBase,A6
       JSR -042(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function PackStructureTags(packk: APTR;const packTable : pULONG;const TagList : pTagItem) : ULONG;
begin
   asm
       MOVEM.L a2/a6,-(A7)
       MOVE.L  packk,a0
       MOVE.L  packTable,a1
       MOVE.L  TagList,a2
       MOVE.L  _UtilityBase,A6
       JSR -210(A6)
       MOVEM.L (A7)+,a2/a6
       MOVE.L  d0,@RESULT
   end;
end;

procedure RefreshTagItemClones(cloneTagItem : pTagItem; const OriginalTagItems : pTagItem);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  cloneTagItem,a0
       MOVE.L  OriginalTagItems,a1
       MOVE.L  _UtilityBase,A6
       JSR -084(A6)
       MOVE.L  (A7)+,A6
   end;
end;

procedure ReleaseNamedObject(Obj : pNamedObject);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Obj,a0
       MOVE.L  _UtilityBase,A6
       JSR -258(A6)
       MOVE.L  (A7)+,A6
   end;
end;

procedure RemNamedObject(Obj : pNamedObject;Msg : pointer);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Obj,a0
       MOVE.L  Msg,a1
       MOVE.L  _UtilityBase,A6
       JSR -264(A6)
       MOVE.L  (A7)+,A6
   end;
end;

function SDivMod32( dividend , divisor : LongInt) : LongInt;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  dividend,d0
       MOVE.L  divisor,d1
       MOVE.L  _UtilityBase,A6
       JSR -150(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function SMult32(Arg1, Arg2 : LongInt) : LongInt;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Arg1,d0
       MOVE.L  Arg2,d1
       MOVE.L  _UtilityBase,A6
       JSR -138(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function SMult64(Arg1, Arg2 : LongInt) : LongInt;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Arg1,d0
       MOVE.L  Arg2,d1
       MOVE.L  _UtilityBase,A6
       JSR -198(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function Stricmp(const Str1: STRPTR;const Str2 : STRPTR) : LongInt;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Str1,a0
       MOVE.L  Str2,a1
       MOVE.L  _UtilityBase,A6
       JSR -162(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function Strnicmp(const Str1: STRPTR;const Str2 : STRPTR;len : LongInt) : LongInt;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Str1,a0
       MOVE.L  Str2,a1
       MOVE.L  len,d0
       MOVE.L  _UtilityBase,A6
       JSR -168(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function TagInArray(t : Tag;const TagArray : pULONG) : Boolean;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  t,d0
       MOVE.L  TagArray,a0
       MOVE.L  _UtilityBase,A6
       JSR -090(A6)
       MOVE.L  (A7)+,A6
       TST.L   d0
       bne     @success
       bra     @end
   @success:
       move.b  #1,d0
   @end:
       move.b  d0,@RESULT
   end;
end;

function ToLower(c : ULONG) : Char;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  c,d0
       MOVE.L  _UtilityBase,A6
       JSR -180(A6)
       MOVE.L  (A7)+,A6
       MOVE.B  d0,@RESULT
   end;
end;

function ToUpper(c : ULONG) : Char;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  c,d0
       MOVE.L  _UtilityBase,A6
       JSR -174(A6)
       MOVE.L  (A7)+,A6
       MOVE.B  d0,@RESULT
   end;
end;

function UDivMod32( dividend , divisor : ULONG) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  dividend,d0
       MOVE.L  divisor,d1
       MOVE.L  _UtilityBase,A6
       JSR -156(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function UMult32(Arg1, Arg2 : ULONG) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Arg1,d0
       MOVE.L  Arg2,d1
       MOVE.L  _UtilityBase,A6
       JSR -144(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function UMult64(Arg1, Arg2 : ULONG) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Arg1,d0
       MOVE.L  Arg2,d1
       MOVE.L  _UtilityBase,A6
       JSR -204(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

function UnpackStructureTags(const pac: APTR;const packTable: pULONG;TagList : pTagItem) : ULONG;
begin
   asm
       MOVEM.L a2/a6,-(A7)
       MOVE.L  pac,a0
       MOVE.L  packTable,a1
       MOVE.L  TagList,a2
       MOVE.L  _UtilityBase,A6
       JSR -216(A6)
       MOVEM.L (A7)+,a2/a6
       MOVE.L  d0,@RESULT
   end;
end;


function AllocNamedObjectA(const name : string;const TagList : pTagItem) : pNamedObject;
begin
       AllocNamedObjectA := AllocNamedObjectA(pas2c(name),TagList);
end;

FUNCTION FindNamedObject(nameSpace : pNamedObject; CONST name : string; lastObject : pNamedObject) : pNamedObject;
begin
       FindNamedObject := FindNamedObject(nameSpace,pas2c(name),lastObject);
end;

FUNCTION Stricmp(CONST string1 : string; CONST string2 : pCHAR) : LONGINT;
begin
       Stricmp := Stricmp(pas2c(string1),string2);
end;

FUNCTION Stricmp(CONST string1 : pCHAR; CONST string2 : string) : LONGINT;
begin
       Stricmp := Stricmp(string1,pas2c(string2));
end;

FUNCTION Stricmp(CONST string1 : string; CONST string2 : string) : LONGINT;
begin
       Stricmp := Stricmp(pas2c(string1),pas2c(string2));
end;

FUNCTION Strnicmp(CONST string1 : string; CONST string2 : pCHAR; length : LONGINT) : LONGINT;
begin
       Strnicmp := Strnicmp(pas2c(string1),string2,length);
end;

FUNCTION Strnicmp(CONST string1 : pCHAR; CONST string2 : string; length : LONGINT) : LONGINT;
begin
       Strnicmp := Strnicmp(string1,pas2c(string2),length);
end;

FUNCTION Strnicmp(CONST string1 : string; CONST string2 : string; length : LONGINT) : LONGINT;
begin
       Strnicmp := Strnicmp(pas2c(string1),pas2c(string2),length);
end;


end.
