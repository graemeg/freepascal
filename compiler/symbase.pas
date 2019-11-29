{
    Copyright (c) 1998-2002 by Florian Klaempfl, Pierre Muller

    This unit handles the symbol tables

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ****************************************************************************
}
unit symbase;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cutils,cclasses,
       { global }
       globtype,globals,
       { symtable }
       symconst
       ;

{************************************************
            Needed forward pointers
************************************************}

    type
       TSymtable = class;

       { THashedIDString }

       THashedIDString=object
       private
         FId   : TIDString;
         FHash : Longword;
         procedure SetId(const s:TIDString);
       public
         property Id:TIDString read FId write SetId;
         property Hash:longword read FHash;
       end;


{************************************************
                 TDefEntry
************************************************}

      TDefEntry = class
         typ   : tdeftyp;
         defid : longint;
         owner : TSymtable;
      end;


{************************************************
                   TSymEntry
************************************************}

      { this object is the base for all symbol objects }
      TSymEntry = class(TFPHashObject)
      private
         FRealName : pshortstring;
         function  GetRealname:shortstring;
         procedure SetRealname(const ANewName:shortstring);
      public
         typ   : tsymtyp;
         SymId : longint;
         Owner : TSymtable;
         destructor destroy;override;
         property RealName:shortstring read GetRealName write SetRealName;
      end;

{************************************************
                 TSymtable
************************************************}

       TSymtable = class
       public
          name      : pshortstring;
          realname  : pshortstring;
          DefList   : TFPObjectList;
          SymList   : TFPHashObjectList;
          defowner  : TDefEntry; { for records and objects }
          moduleid  : longint;
          refcount  : smallint;
          currentvisibility : tvisibility;
          currentlyoptional : boolean;
          tableoptions : tsymtableoptions;
          { level of symtable, used for nested procedures }
          symtablelevel : byte;
          { do not allow to add new definitions, can be extended to symbols probably }
          sealed : boolean;
          symtabletype  : TSymtabletype;
          constructor Create(const s:string);
          { attention: only execute the a child's destructor if refcount is 1! }
          destructor  destroy;override;
          procedure freeinstance;override;
          function  getcopy:TSymtable;
          procedure clear;virtual;
          function  checkduplicate(var s:THashedIDString;sym:TSymEntry):boolean;virtual;
          procedure insert(sym:TSymEntry;checkdup:boolean=true);virtual;
          procedure Delete(sym:TSymEntry);virtual;
          function  Find(const s:TIDString) : TSymEntry;
          function  FindWithHash(const s:THashedIDString) : TSymEntry;virtual;
          procedure insertdef(def:TDefEntry);virtual;
          procedure deletedef(def:TDefEntry);
          function  iscurrentunit:boolean;virtual;
          { includes the flag in this symtable and all parent symtables; if
            it's already set the flag is not set again }
          procedure includeoption(option:tsymtableoption);
       end;

       psymtablestackitem = ^TSymtablestackitem;
       TSymtablestackitem = record
         symtable : TSymtable;
         next     : psymtablestackitem;
       end;

       TSymtablestack = class
         stack : psymtablestackitem;
         constructor create;
         destructor destroy;override;
         procedure clear;
         function finditem(st:TSymtable):psymtablestackitem;
         procedure push(st:TSymtable); virtual;
         procedure pushafter(st,afterst:TSymtable); virtual;
         procedure pop(st:TSymtable); virtual;
         function  top:TSymtable;
         function getcopyuntil(finalst: TSymtable): TSymtablestack;
       end;


    var
       initialmacrosymtable: TSymtable;   { macros initially defined by the compiler or
                                            given on the command line. Is common
                                            for all files compiled and do not change. }
       macrosymtablestack,
       symtablestack        : TSymtablestack;

{$ifdef MEMDEBUG}
    var
      memrealnames : tmemdebug;
{$endif MEMDEBUG}


implementation

    uses
       verbose;

{****************************************************************************
                              THashedIDString
****************************************************************************}

    procedure THashedIDString.SetId(const s:TIDString);
      begin
        FId:=s;
        FHash:=FPHash(s);
      end;


{****************************************************************************
                                TSymEntry
****************************************************************************}

    destructor TSymEntry.destroy;
      begin
{$ifdef MEMDEBUG}
        memrealnames.start;
{$endif MEMDEBUG}
        stringdispose(Frealname);
{$ifdef MEMDEBUG}
        memrealnames.stop;
{$endif MEMDEBUG}
        inherited destroy;
      end;


    function TSymEntry.GetRealname:shortstring;
      begin
        if not assigned(FRealname) then
          internalerror(200611011);
        result:=FRealname^;
      end;


    procedure TSymEntry.SetRealname(const ANewName:shortstring);
      begin
        stringdispose(FRealname);
        FRealname:=stringdup(ANewName);
        if Hash<>$ffffffff then
          begin
            if FRealname^[1]='$' then
              Rename(Copy(FRealname^,2,255))
            else
              Rename(Upper(FRealname^));
          end;
      end;


{****************************************************************************
                                TSymtable
****************************************************************************}

    constructor TSymtable.Create(const s:string);
      begin
         if s<>'' then
           begin
             name:=stringdup(upper(s));
             realname:=stringdup(s);
           end
         else
           begin
             name:=nil;
             realname:=nil;
           end;
         symtabletype:=abstractsymtable;
         symtablelevel:=0;
         defowner:=nil;
         DefList:=TFPObjectList.Create(true);
         SymList:=TFPHashObjectList.Create(true);
         refcount:=1;
         currentvisibility:=vis_public;
         currentlyoptional:=false;
      end;


    destructor TSymtable.destroy;
      begin
        { freeinstance decreases refcount }
        if refcount>1 then
          exit;
        Clear;
        DefList.Free;
        { SymList can already be disposed or set to nil for withsymtable, }
        { but in that case Free does nothing                              }
        SymList.Free;
        stringdispose(name);
        stringdispose(realname);
      end;


    procedure TSymtable.freeinstance;
      begin
        dec(refcount);
        if refcount=0 then
          inherited freeinstance;
      end;


    function TSymtable.getcopy:TSymtable;
      begin
        inc(refcount);
        result:=self;
      end;


    function TSymtable.iscurrentunit:boolean;
      begin
        result:=false;
      end;

    procedure TSymtable.includeoption(option: tsymtableoption);
      var
        st: tsymtable;
      begin
        if option in tableoptions then
          exit;
        include(tableoptions,option);
        { iterative approach should be faster than recursion based on calls }
        st:=self;
        while assigned(st.defowner) do
          begin
            st:=st.defowner.owner;
            { this can happen for specializations of routines that are not yet
              owned cause they might be thrown away again }
            if not assigned(st) then
              break;
            { the flag is already set, so by definition it is set in the
              owning symtables as well }
            if option in st.tableoptions then
              break;
            include(st.tableoptions,option);
          end;
      end;


    procedure TSymtable.clear;
      var
        i : integer;
      begin
         SymList.Clear;
         { Prevent recursive calls between TDef.destroy and TSymtable.Remove }
         if DefList.OwnsObjects then
           begin
             for i := 0 to DefList.Count-1 do
               TDefEntry(DefList[i]).Owner:=nil;
           end;
         DefList.Clear;
      end;


    function TSymtable.checkduplicate(var s:THashedIDString;sym:TSymEntry):boolean;
      begin
        result:=(FindWithHash(s)<>nil);
      end;


    procedure TSymtable.insert(sym:TSymEntry;checkdup:boolean=true);
      var
        hashedid : THashedIDString;
      begin
         if checkdup then
           begin
             if sym.realname[1]='$' then
               hashedid.id:=Copy(sym.realname,2,255)
             else
               hashedid.id:=Upper(sym.realname);
             { First check for duplicates, this can change the symbol name
               in case of a duplicate entry }
             checkduplicate(hashedid,sym);
           end;
         { Now we can insert the symbol, any duplicate entries
           are renamed to an unique (and for users unaccessible) name }
         if sym.realname[1]='$' then
           sym.ChangeOwnerAndName(SymList,Copy(sym.realname,2,255))
         else
           sym.ChangeOwnerAndName(SymList,Upper(sym.realname));
         sym.Owner:=self;
      end;


    procedure TSymtable.Delete(sym:TSymEntry);
      begin
        if sym.Owner<>self then
          internalerror(200611121);
        SymList.Remove(sym);
      end;


    procedure TSymtable.insertdef(def:TDefEntry);
      begin
         DefList.Add(def);
         def.owner:=self;
      end;


    procedure TSymtable.deletedef(def:TDefEntry);
      begin
        if def.Owner<>self then
          internalerror(200611122);
        def.Owner:=nil;
        DefList.Remove(def);
      end;


    function TSymtable.Find(const s : TIDString) : TSymEntry;
      begin
        result:=TSymEntry(SymList.Find(s));
      end;


    function TSymtable.FindWithHash(const s:THashedIDString) : TSymEntry;
      begin
        result:=TSymEntry(SymList.FindWithHash(s.id,s.hash));
      end;


{****************************************************************************
                            Symtable Stack
****************************************************************************}

    constructor TSymtablestack.create;
      begin
        stack:=nil;
      end;


    destructor TSymtablestack.destroy;
      begin
        clear;
      end;


    procedure TSymtablestack.clear;
      var
        hp : psymtablestackitem;
      begin
        while assigned(stack) do
          begin
            hp:=stack;
            stack:=hp^.next;
            dispose(hp);
          end;
      end;

    function TSymtablestack.finditem(st: TSymtable): psymtablestackitem;
      begin
        if not assigned(stack) then
          internalerror(200601233);
        result:=stack;
        while assigned(result)and(result^.symtable<>st) do
          result:=result^.next;
      end;

    procedure TSymtablestack.push(st:TSymtable);
      var
        hp : psymtablestackitem;
      begin
        new(hp);
        hp^.symtable:=st;
        hp^.next:=stack;
        stack:=hp;
      end;

    procedure TSymtablestack.pushafter(st,afterst:TSymtable);
      var
        hp,afteritem: psymtablestackitem;
      begin
        afteritem:=finditem(afterst);
        if assigned(afteritem) then
          begin
            new(hp);
            hp^.symtable:=st;
            hp^.next:=afteritem^.next;
            afteritem^.next:=hp;
          end
        else
          internalerror(201309171);
      end;

    procedure TSymtablestack.pop(st:TSymtable);
      var
        hp : psymtablestackitem;
      begin
        if not assigned(stack) then
          internalerror(200601231);
        if stack^.symtable<>st then
          internalerror(200601232);
        hp:=stack;
        stack:=hp^.next;
        dispose(hp);
      end;


    function TSymtablestack.top:TSymtable;
      begin
        if not assigned(stack) then
          internalerror(200601233);
        result:=stack^.symtable;
      end;


  function addstitemreverse(st: TSymtablestack; finalst: tsymtable; curitem: psymtablestackitem): boolean;
    begin
      if not assigned(curitem) then
        begin
          result:=true;
          exit;
        end;
      if addstitemreverse(st,finalst,curitem^.next) then
        begin
          st.push(curitem^.symtable);
          result:=curitem^.symtable<>finalst
        end
      else
        result:=false
    end;


  function TSymtablestack.getcopyuntil(finalst: TSymtable): TSymtablestack;
    begin
      result:=TSymtablestack.create;
      addstitemreverse(result,finalst,stack);
    end;


{$ifdef MEMDEBUG}
initialization
  memrealnames:=TMemDebug.create('Realnames');
  memrealnames.stop;

finalization
  memrealnames.free;
{$endif MEMDEBUG}
end.
