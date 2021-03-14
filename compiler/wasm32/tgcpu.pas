{
    Copyright (C) 2019 Dmitry Boyarintsev

    This unit handles the temporary variables for the WebAssembly

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
unit tgcpu;

{$i fpcdefs.inc}

  interface

    uses
       globtype,
       aasmdata,
       cgutils, cpubase,
       symtype,tgobj;

    type

      { TWasmLocal }

      TWasmLocal = class
        inuse    : Boolean;
        noreuse  : Boolean;
        index    : integer;
        typ      : TWasmBasicType;
        next     : TWasmLocal; // next in the same basic type
        nextseq  : TWasmLocal; // from 0 to max
        constructor create(atype: TWasmBasicType; aindex: integer);
      end;

      { TWasmLocalVars }

      TWasmLocalVars = class
      private
        last: TWasmLocal; // need public?
      public
        locv: array[TWasmBasicType] of TWasmLocal;
        ordered: array of integer;
        first: TWasmLocal; // first in sequence
        varindex: integer;
        constructor Create(astartindex: Integer = 0);
        destructor Destroy; override;
        function alloc(bt: TWasmBasicType): integer;
        function allocnoreuse(bt: TWasmBasicType): integer;
        procedure dealloc(bt: TWasmBasicType; index: integer);
        procedure dealloc(index: integer);
      end;

       { ttgwasm }

       ttgwasm = class(ttgobj)
        private
         procedure updateFirstTemp;

         procedure allocLocalVarToRef(wbt: TWasmBasicType; out ref: treference);
         procedure allocLocalVarNoReuseToRef(wbt: TWasmBasicType; out ref: treference);
         procedure LocalVarToRef(idx: integer; size: Integer; out ref: treference);
        public
         localvars: TWasmLocalVars;

         constructor create; override;
         destructor destroy; override;
         procedure setfirsttemp(l : asizeint); override;
         procedure gethltemp(list: TAsmList; def: tdef; forcesize: asizeint; temptype: ttemptype; out ref: treference); override;
         procedure gethltempmanaged(list: TAsmList; def: tdef; temptype: ttemptype; out ref: treference); override;
         procedure ungettemp(list: TAsmList; const ref : treference); override;
         procedure allocframepointer(list: TAsmList; out ref: treference);
         procedure allocbasepointer(list: TAsmList; out ref: treference);
       end;

    function defToWasmBasic(def: tdef; var wbt: TWasmBasicType): Boolean;

  implementation

    uses
       verbose,
       cgbase,
       symconst,symtable,symdef,symsym,symcpu,defutil,
       aasmbase,aasmcpu,
       hlcgobj,hlcgcpu, procinfo;

    function defToWasmBasic(def: tdef; var wbt: TWasmBasicType): Boolean;
    begin
      Result := assigned(def);
      if not Result then Exit;

      if is_pointer(def) then
        wbt := wbt_i32 // wasm32
      else if is_currency(def) then
        wbt := wbt_i64
      else if is_ordinal(def) then begin
        if is_64bit(def) then wbt := wbt_i64
        else wbt := wbt_i32;
      end else if is_real(def) then begin
        if is_single(def) then wbt := wbt_f32
        else wbt := wbt_f64; // real/double/extended
      end else
        Result := false;
    end;

        { TWasmLocal }

                constructor TWasmLocal.create(atype: TWasmBasicType;
                  aindex: integer);
        begin
          typ:=atype;
          index:=aindex;
        end;

    { TWasmLocalVars }

        constructor TWasmLocalVars.Create(astartindex: Integer = 0);
          begin
            inherited Create;
            varindex := astartindex;
          end;

        destructor TWasmLocalVars.Destroy;
          var
            t : TWasmLocal;
            n : TWasmLocal;
          begin
            t := first;
            while Assigned(t) do
              begin
                n:=t;
                t:=t.nextseq;
                n.Free;
              end;
            inherited Destroy;
          end;


    { ttgwasm }

    procedure ttgwasm.updateFirstTemp;
    begin
      firsttemp := localvars.varindex;
      if lasttemp<firsttemp then lasttemp := firsttemp;
    end;

    constructor ttgwasm.create;
      begin
        inherited create;
        localvars:=TWasmLocalVars.Create;
      end;

    destructor ttgwasm.destroy;
      begin
        localvars.Free;
        inherited destroy;
      end;

    procedure ttgwasm.setfirsttemp(l: asizeint);
      begin
        firsttemp:=l;
        lasttemp:=l;
        localvars.varindex := l; //?
      end;


    procedure ttgwasm.gethltemp(list: TAsmList; def: tdef; forcesize: asizeint; temptype: ttemptype; out ref: treference);
      var
        wbt: TWasmBasicType;
      begin
        if temptype=tt_regallocator then
          begin
            if Assigned(def) and defToWasmBasic(def, wbt) then
              allocLocalVarToRef(wbt, ref)
            else
              internalerror(2020121801);
          end
        else
          inherited;
      end;

    procedure ttgwasm.gethltempmanaged(list: TAsmList; def: tdef; temptype: ttemptype; out ref: treference);
      begin
        inherited;
      end;

    procedure ttgwasm.ungettemp(list: TAsmList; const ref: treference);
      begin
        if ref.base=NR_LOCAL_STACK_POINTER_REG then
          localvars.dealloc(ref.offset)
        else
          inherited;
      end;

    procedure ttgwasm.allocframepointer(list: TAsmList; out ref: treference);
      begin
        allocLocalVarNoReuseToRef(wbt_i32,ref);
      end;

    procedure ttgwasm.allocbasepointer(list: TAsmList; out ref: treference);
      begin
        allocLocalVarNoReuseToRef(wbt_i32,ref);
      end;

    procedure ttgwasm.allocLocalVarToRef(wbt: TWasmBasicType; out ref: treference);
      var
        idx : integer;
      begin
        idx := localvars.alloc(wbt);
        localVarToRef(idx, 1, ref);
      end;

    procedure ttgwasm.allocLocalVarNoReuseToRef(wbt: TWasmBasicType; out ref: treference);
      var
        idx : integer;
      begin
        idx := localvars.allocnoreuse(wbt);
        localVarToRef(idx, 1, ref);
      end;

    procedure ttgwasm.localVarToRef(idx: integer; size: integer; out ref: treference);
      begin
        reference_reset_base(ref,NR_LOCAL_STACK_POINTER_REG,idx,ctempposinvalid,size,[]);
        updateFirstTemp;
      end;

    function TWasmLocalVars.alloc(bt: TWasmBasicType): integer;
      var
        i : integer;
        lc : TWasmLocal;
        t  : TWasmLocal;
      begin
        lc := locv[bt];
        t := nil;
        while Assigned(lc) and ((lc.inuse) or (lc.noreuse)) do begin
          t := lc;
          lc := lc.next;
        end;
        if Assigned(lc) then begin
          lc.inuse := true;
        end else begin
          lc := TWasmLocal.Create(bt, varindex);
          if Assigned(t)
            then t.next := lc
            else locv[bt]:=lc;
          lc.inuse:=true;
          inc(varindex);

          if Assigned(last) then last.nextseq := lc;
          if not Assigned(first) then first := lc;
          last := lc;
        end;
        alloc := lc.index;

      end;

    function TWasmLocalVars.allocnoreuse(bt: TWasmBasicType): integer;
      var
        i : integer;
        lc : TWasmLocal;
        t  : TWasmLocal;
      begin
        lc := locv[bt];
        t := nil;
        while Assigned(lc) do
          begin
            t := lc;
            lc := lc.next;
          end;
        lc := TWasmLocal.Create(bt, varindex);
        if Assigned(t) then
          t.next := lc
        else
          locv[bt]:=lc;
        lc.inuse:=true;
        lc.noreuse:=true;
        inc(varindex);
        if Assigned(last) then
          last.nextseq := lc;
        if not Assigned(first) then
          first := lc;
        last := lc;
        allocnoreuse := lc.index;
      end;

    procedure TWasmLocalVars.dealloc(bt: TWasmBasicType; index: integer);
      var
        lc : TWasmLocal;
      begin
        lc := locv[bt];
        while Assigned(lc) and (lc.index <> index) do
          lc := lc.next;
        if Assigned(lc) then lc.inuse := false;
      end;

    procedure TWasmLocalVars.dealloc(index: integer);
      var
        lc : TWasmLocal;
      begin
        lc := first;
        while Assigned(lc) and (lc.index <> index) do
          lc := lc.nextseq;
        if Assigned(lc) then lc.inuse := false;
      end;


initialization
  tgobjclass:=ttgwasm;
end.
