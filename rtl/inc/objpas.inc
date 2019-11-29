{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    This unit makes Free Pascal as much as possible Delphi compatible

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ifdef FPC_HAS_FEATURE_VARIANTS}
    procedure DoDispCallByIDError(res : Pointer; const disp : IDispatch;desc : PDispDesc; params : Pointer);
      begin
        handleerroraddrframeind(RuntimeErrorExitCodes[reVarDispatch],
          get_pc_addr,get_frame);
      end;


    procedure fpc_dispatch_by_id(Result: Pointer; const Dispatch: pointer;
      DispDesc: Pointer; Params: Pointer); compilerproc;
      type
        TDispProc = procedure(res : Pointer; const disp : IDispatch;desc : PDispDesc; params : Pointer);
      begin
        TDispProc(DispCallByIDProc)(Result,IDispatch(Dispatch),DispDesc,Params);
      end;
{$endif FPC_HAS_FEATURE_VARIANTS}


{****************************************************************************
                  Internal Routines called from the Compiler
****************************************************************************}

    { the reverse order of the parameters make code generation easier }
    function fpc_do_is(aclass : tclass;aobject : tobject) : boolean;[public,alias: 'FPC_DO_IS']; compilerproc;
      begin
         fpc_do_is:=assigned(aobject) and assigned(aclass) and
           aobject.inheritsfrom(aclass);
      end;


    { the reverse order of the parameters make code generation easier }
    function fpc_do_as(aclass : tclass;aobject : tobject): tobject;[public,alias: 'FPC_DO_AS']; compilerproc;
      begin
         if assigned(aobject) and not(aobject.inheritsfrom(aclass)) then
           handleerroraddrframeInd(219,get_pc_addr,get_frame);
         result := aobject;
      end;

    { interface helpers }
    procedure fpc_intf_decr_ref(var i: pointer);[public,alias: 'FPC_INTF_DECR_REF']; compilerproc;
      begin
        if assigned(i) then
          begin
            IUnknown(i)._Release;
            i:=nil;
          end;
      end;

    { local declaration for intf_decr_ref for local access }
    procedure intf_decr_ref(var i: pointer); [external name 'FPC_INTF_DECR_REF'];


    procedure fpc_intf_incr_ref(i: pointer);[public,alias: 'FPC_INTF_INCR_REF']; compilerproc;
      begin
         if assigned(i) then
           IUnknown(i)._AddRef;
      end;

    { local declaration of intf_incr_ref for local access }
    procedure intf_incr_ref(i: pointer); [external name 'FPC_INTF_INCR_REF'];

    procedure fpc_intf_assign(var D: pointer; const S: pointer);[public,alias: 'FPC_INTF_ASSIGN']; compilerproc;
      begin
         if assigned(S) then
           IUnknown(S)._AddRef;
         if assigned(D) then
           IUnknown(D)._Release;
         D:=S;
      end;

    procedure fpc_intf_assign(var D: pointer; const s: pointer); [external name 'FPC_INTF_ASSIGN'];

    {procedure fpc_intf_assign_by_iid(var D: pointer; const S: pointer; const iid: TGUID);[public,alias: 'FPC_INTF_ASSIGN2']; compilerproc;
      var
        tmp : pointer;
      begin
         if assigned(S) then
           begin
             tmp:=nil;
             if IUnknown(S).QueryInterface(iid,tmp)<>S_OK then
               handleerror(219);
             if assigned(D) then
               IUnknown(D)._Release;
             D:=tmp;
           end
         else
           begin
             if assigned(D) then
               IUnknown(D)._Release;
             D:=nil;
           end;
      end;}


    function fpc_intf_is(const S: pointer; const iid: TGUID): Boolean;[public,alias: 'FPC_INTF_IS']; compilerproc;
      var
        tmpi: pointer;
      begin
        tmpi:=nil;
        fpc_intf_is:=Assigned(S) and (IUnknown(S).QueryInterface(iid,tmpi)=S_OK);
        if Assigned(tmpi) then
          IUnknown(tmpi)._Release;
      end;


    function fpc_intf_is_class(const S: pointer; const aclass: tclass): Boolean;[public,alias: 'FPC_INTF_IS_CLASS']; compilerproc;
      var
        tmpo: tobject;
      begin
        fpc_intf_is_class:=Assigned(S) and (IUnknown(S).QueryInterface(IObjectInstance,tmpo)=S_OK)  and tmpo.InheritsFrom(aclass);
      end;


    function fpc_class_is_intf(const S: pointer; const iid: TGUID): Boolean;[public,alias: 'FPC_CLASS_IS_INTF']; compilerproc;
      var
        tmpi: pointer;
        tmpi2: pointer; // weak!
      begin
        tmpi:=nil;
        tmpi2:=nil;
        fpc_class_is_intf:=Assigned(S) and ((TObject(S).GetInterfaceWeak(IUnknown,tmpi2) and (IUnknown(tmpi2).QueryInterface(IID,tmpi)=S_OK)) or
            TObject(S).GetInterface(IID,tmpi));
        if Assigned(tmpi) then
          IUnknown(tmpi)._Release;
      end;


    function fpc_class_is_corbaintf(const S: pointer; const iid: Shortstring): Boolean;[public,alias: 'FPC_CLASS_IS_CORBAINTF']; compilerproc;
      begin
        fpc_class_is_corbaintf:=Assigned(S) and Assigned(TObject(S).GetInterfaceEntryByStr(iid));
      end;


    function fpc_intf_cast(const S: pointer; const iid: TGUID): IInterface;[public,alias: 'FPC_INTF_CAST']; compilerproc;
      var
        tmpi: pointer;
      begin
        tmpi:=nil;
        if Assigned(S) and (IUnknown(S).QueryInterface(iid,tmpi)=S_OK) then
          pointer(fpc_intf_cast):=tmpi
        else
          fpc_intf_cast:= nil;
      end;


    function fpc_intf_cast_class(const S: pointer; const aclass: tclass): pointer;[public,alias: 'FPC_INTF_CAST_CLASS']; compilerproc;
      var
        tmpo: tobject;
      begin
        if Assigned(S) and (IUnknown(S).QueryInterface(IObjectInstance,tmpo)=S_OK) and tmpo.InheritsFrom(aclass) then
          fpc_intf_cast_class:=tmpo
        else
          fpc_intf_cast_class:=nil;
      end;


    function fpc_class_cast_intf(const S: pointer; const iid: TGUID): IInterface;[public,alias: 'FPC_CLASS_CAST_INTF']; compilerproc;
      var
        tmpi: pointer;
        tmpi2: pointer; // weak!
      begin
        tmpi:=nil;
        tmpi2:=nil;
        if Assigned(S) and ((TObject(S).GetInterfaceWeak(IUnknown,tmpi2) and (IUnknown(tmpi2).QueryInterface(IID,tmpi)=S_OK)) or
            TObject(S).GetInterface(IID,tmpi)) then
          begin
            // decrease reference count
            fpc_class_cast_intf:=nil;
            pointer(fpc_class_cast_intf):=tmpi
          end
        else
          fpc_class_cast_intf:=nil;
      end;


    function fpc_class_cast_corbaintf(const S: pointer; const iid: Shortstring): Pointer;[public,alias: 'FPC_CLASS_CAST_CORBAINTF']; compilerproc;
      var
        tmpi: pointer;
      begin
        if Assigned(S) and TObject(S).GetInterface(iid,tmpi) then
          fpc_class_cast_corbaintf:=tmpi
        else
          fpc_class_cast_corbaintf:=nil;
      end;


    function fpc_intf_as(const S: pointer; const iid: TGUID): IInterface;[public,alias: 'FPC_INTF_AS']; compilerproc;
      var
        tmpi: pointer; // _AddRef before _Release
      begin
        if assigned(S) then
          begin
             tmpi:=nil;
             if IUnknown(S).QueryInterface(iid,tmpi)<>S_OK then
               handleerror(219);
             // decrease reference count
             fpc_intf_as:=nil;
             pointer(fpc_intf_as):=tmpi;
          end
        else
          fpc_intf_as:=nil;
      end;


    function fpc_intf_as_class(const S: pointer; const aclass: tclass): pointer;[public,alias: 'FPC_INTF_AS_CLASS']; compilerproc;
      var
        tmpo: tobject;
      begin
        if assigned(S) then
          begin
            if not ((IUnknown(S).QueryInterface(IObjectInstance,tmpo)=S_OK) and tmpo.inheritsfrom(aclass)) then
              handleerror(219);
            fpc_intf_as_class:=tmpo;
          end
        else
          fpc_intf_as_class:=nil;
      end;


    function fpc_class_as_intf(const S: pointer; const iid: TGUID): IInterface;[public,alias: 'FPC_CLASS_AS_INTF']; compilerproc;
      var
        tmpi: pointer; // _AddRef before _Release
        tmpi2: pointer; // weak!
      begin
        if assigned(S) then
          begin
             tmpi:=nil;
             tmpi2:=nil;
             if not ((TObject(S).GetInterfaceWeak(IUnknown,tmpi2) and (IUnknown(tmpi2).QueryInterface(IID,tmpi)=S_OK)) or TObject(S).GetInterface(IID,tmpi)) then
               handleerror(219);
             // decrease reference count
             fpc_class_as_intf:=nil;
             pointer(fpc_class_as_intf):=tmpi;
          end
        else
          fpc_class_as_intf:=nil;
      end;


    function fpc_class_as_corbaintf(const S: pointer; const iid: Shortstring): Pointer;[public,alias: 'FPC_CLASS_AS_CORBAINTF']; compilerproc;
      var
        tmpi: pointer; // _AddRef before _Release
      begin
        if assigned(S) then
          begin
             tmpi:=nil;
             if not TObject(S).GetInterface(iid,tmpi) then
               handleerror(219);
             fpc_class_as_corbaintf:=tmpi;
          end
        else
          fpc_class_as_corbaintf:=nil;
      end;

{****************************************************************************
                               TVMT
****************************************************************************}


    function TVmt.GetvParent: PVmt;
      begin
        {$ifdef VER3_0}
        GetvParent:=vParentRef;
        {$else VER3_0}
        if Assigned(vParentRef) then
          GetvParent:=vParentRef^
        else
          GetvParent:=Nil;
        {$endif VER3_0}
      end;


{****************************************************************************
                           TINTERFACEENTRY
****************************************************************************}


    function tinterfaceentry.GetIID: pguid;
      begin
        {$ifdef VER3_0}
        GetIID:=IIDRef;
        {$else VER3_0}
        if Assigned(IIDRef) then
          GetIID:=IIDRef^
        else
          GetIID:=Nil;
        {$endif VER3_0}
      end;


    function tinterfaceentry.GetIIDStr: pshortstring;
      begin
        {$ifdef VER3_0}
        GetIIDStr:=IIDStrRef;
        {$else VER3_0}
        if Assigned(IIDStrRef) then
          GetIIDStr:=IIDStrRef^
        else
          GetIIDStr:=Nil;
        {$endif VER3_0}
      end;


{****************************************************************************
                               TOBJECT
****************************************************************************}

      constructor TObject.Create;
        begin
        end;

      destructor TObject.Destroy;
        begin
        end;

      procedure TObject.Free;

        begin
           // the call via self avoids a warning
           if self<>nil then
             self.destroy;
        end;

      class function TObject.InstanceSize : SizeInt;

        begin
           InstanceSize := PVmt(Self)^.vInstanceSize;
        end;

      {$ifdef VER3_0}
      var
        emptyintf: ptruint; public name 'FPC_EMPTYINTF';
      {$endif VER3_0}

      procedure InitInterfacePointers(objclass: tclass;instance : pointer);

        var
          ovmt: PVmt;
          i: longint;
          intftable: pinterfacetable;
          Res: pinterfaceentry;
        begin
          ovmt := PVmt(objclass);
          while assigned(ovmt) and {$ifdef VER3_0}(ovmt^.vIntfTable <> @emptyintf){$else}assigned(ovmt^.vIntfTable){$endif} do
            begin
              intftable:=ovmt^.vIntfTable;
              {$ifdef VER3_0}
              if assigned(intftable) then
              {$endif VER3_0}
              begin
                i:=intftable^.EntryCount;
                Res:=@intftable^.Entries[0];
                while i>0 do begin
                  if Res^.IType = etStandard then
                    ppointer(@(pbyte(instance)[Res^.IOffset]))^:=
                      pointer(Res^.VTable);
                  inc(Res);
                  dec(i);
                end;
              end;
              ovmt:=ovmt^.vParent;
            end;
        end;

      class function TObject.InitInstance(instance : pointer) : tobject; {$ifdef SYSTEMINLINE} inline; {$ENDIF}

{$ifndef VER3_0}
        var
           vmt  : PVmt;
           inittable : pointer;
           mopinittable : PRTTIRecordOpOffsetTable;
           i : longint;
{$endif VER3_0}
        begin
           { the size is saved at offset 0 }
           fillchar(instance^, InstanceSize, 0);
           { insert VMT pointer into the new created memory area }
           { (in class methods self contains the VMT!)           }
           ppointer(instance)^:=pointer(self);
           if {$ifdef VER3_0}PVmt(self)^.vIntfTable <> @emptyintf{$else}assigned(PVmt(self)^.vIntfTable){$endif} then
             InitInterfacePointers(self,instance);

{$ifndef VER3_0}
           { for management operators like initialize call int_initialize }
           vmt := PVmt(self);
           if assigned(vmt) then
             begin
               inittable:=vmt^.vInitTable;
               if assigned(inittable) then
                 begin
                   mopinittable:=RTTIRecordMopInitTable(inittable);
                   if assigned(mopinittable) then
                     begin
                       {$push}
                       { ensure that no range check errors pop up with the [0..0] array }
                       {$R-}
                       for i:=0 to mopinittable^.Count-1 do
                         TRTTIRecVarOp(mopinittable^.Entries[i].ManagmentOperator)(PByte(Instance)+mopinittable^.Entries[i].FieldOffset);
                       {$pop}
                     end;
                 end;
             end;
{$endif VER3_0}

           InitInstance:=TObject(Instance);
        end;

      class function TObject.ClassParent : tclass;

        begin
           { type of self is class of tobject => it points to the vmt }
           { the parent vmt is saved at offset vmtParent              }
           classparent:=tclass(PVmt(Self)^.vParent);
        end;

      class function TObject.NewInstance : tobject;

        var
           p : pointer;

        begin
           getmem(p, InstanceSize);
           if p <> nil then
              InitInstance(p);
           NewInstance:=TObject(p);
        end;

      procedure TObject.FreeInstance;

        begin
           CleanupInstance;
           FreeMem(Pointer(Self));
        end;

      class function TObject.ClassType : TClass;

        begin
           ClassType:=TClass(Pointer(Self))
        end;

      type
         tmethodnamerec = packed record
            name : pshortstring;
            addr : codepointer;
         end;

         tmethodnametable = packed record
           count : dword;
           entries : packed array[0..0] of tmethodnamerec;
         end;

         pmethodnametable =  ^tmethodnametable;

      class function TObject.MethodAddress(const name : shortstring) : codepointer;

        var
           methodtable : pmethodnametable;
           i : dword;
           ovmt : PVmt;

        begin
           ovmt:=PVmt(self);
           while assigned(ovmt) do
             begin
                methodtable:=pmethodnametable(ovmt^.vMethodTable);
                if assigned(methodtable) then
                  begin
                     for i:=0 to methodtable^.count-1 do
                       if ShortCompareText(methodtable^.entries[i].name^, name)=0 then
                         begin
                            MethodAddress:=methodtable^.entries[i].addr;
                            exit;
                         end;
                  end;
                ovmt := ovmt^.vParent;
             end;
           MethodAddress:=nil;
        end;


      class function TObject.MethodName(address : codepointer) : shortstring;
        var
           methodtable : pmethodnametable;
           i : dword;
           ovmt : PVmt;
        begin
           ovmt:=PVmt(self);
           while assigned(ovmt) do
             begin
                methodtable:=pmethodnametable(ovmt^.vMethodTable);
                if assigned(methodtable) then
                  begin
                     for i:=0 to methodtable^.count-1 do
                       if methodtable^.entries[i].addr=address then
                         begin
                            MethodName:=methodtable^.entries[i].name^;
                            exit;
                         end;
                  end;
                ovmt := ovmt^.vParent;
             end;
           MethodName:='';
        end;


      function TObject.FieldAddress(const name : shortstring) : pointer;
        type
           PFieldInfo = ^TFieldInfo;
           TFieldInfo =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
           packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
           record
             FieldOffset: SizeUInt;
             ClassTypeIndex: Word;
             Name: ShortString;
           end;

           PFieldTable = ^TFieldTable;
           TFieldTable =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
           packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
           record
             FieldCount: Word;
             ClassTable: Pointer;
             { should be array[Word] of TFieldInfo;  but
               Elements have variant size! force at least proper alignment }
             Fields: array[0..0] of TFieldInfo
           end;

        var
           ovmt: PVmt;
           FieldTable: PFieldTable;
           FieldInfo: PFieldInfo;
           i: longint;

        begin
           if Length(name) > 0 then
           begin
             ovmt := PVmt(ClassType);
             while ovmt <> nil do
             begin
               FieldTable := PFieldTable(ovmt^.vFieldTable);
               if FieldTable <> nil then
               begin
                 FieldInfo := @FieldTable^.Fields[0];
                 for i := 0 to FieldTable^.FieldCount - 1 do
                 begin
                   if ShortCompareText(FieldInfo^.Name, name) = 0 then
                   begin
                     fieldaddress := Pointer(Self) + FieldInfo^.FieldOffset;
                     exit;
                   end;
                   FieldInfo := PFieldInfo(PByte(@FieldInfo^.Name) + 1 + Length(FieldInfo^.Name));
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
                   { align to largest field of TFieldInfo }
                   FieldInfo := Align(FieldInfo, SizeOf(PtrUInt));
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
                 end;
               end;
               { Try again with the parent class type }
               ovmt:=ovmt^.vParent;
             end;
           end;

           fieldaddress:=nil;
        end;

      function TObject.SafeCallException(exceptobject : tobject;
        exceptaddr : codepointer) : HResult;

        begin
          safecallexception:=E_UNEXPECTED;
        end;

      class function TObject.ClassInfo : pointer;

        begin
          ClassInfo := PVmt(Self)^.vTypeInfo;
        end;

      class function TObject.ClassName : ShortString;

        begin
          ClassName := PVmt(Self)^.vClassName^;
        end;

      class function TObject.ClassNameIs(const name : string) : boolean;

        begin
        // call to ClassName inlined here, this eliminates stack and string copying.
           ClassNameIs:=ShortCompareText(PVmt(Self)^.vClassName^, name) = 0;
        end;

      class function TObject.InheritsFrom(aclass : TClass) : Boolean;

        var
           vmt: PVmt;

        begin
           if assigned(aclass) then
             begin
               vmt:=PVmt(self);
               while assigned(vmt) and (vmt <> PVmt(aclass)) do
                 vmt := vmt^.vParent;
               InheritsFrom := (vmt = PVmt(aclass));
             end
           else
             inheritsFrom := False;
        end;

      class function TObject.stringmessagetable : pstringmessagetable;

        begin
          stringmessagetable:=PVmt(Self)^.vMsgStrPtr;
        end;

      type
         tmessagehandler = procedure(var msg) of object;


      procedure TObject.Dispatch(var message);

        type
{$PUSH}
{$PACKRECORDS NORMAL}
           PMsgIntTable = ^TMsgIntTable;
           TMsgIntTable = record
              index : dword;
              method : codepointer;
           end;

           PMsgInt = ^TMsgInt;
           TMsgInt = record
              count : longint;
              msgs : array[0..0] of TMsgIntTable;
           end;
{$POP}
        var
           index : dword;
           count,i : longint;
           msgtable : PMsgIntTable;
           p : PMsgInt;
           ovmt : PVmt;
           msghandler : tmessagehandler;

        begin
           index:=dword(message);
           ovmt := PVmt(ClassType);
           while assigned(ovmt) do
             begin
                // See if we have messages at all in this class.
                p:=PMsgInt(ovmt^.vDynamicTable);
                If Assigned(p) then
                  begin
                     msgtable:=@p^.msgs;
                     count:=p^.count;
                  end
                else
                  Count:=0;
                { later, we can implement a binary search here }
                for i:=0 to count-1 do
                  begin
                     if index=msgtable[i].index then
                       begin
                          TMethod(msghandler).Code:=msgtable[i].method;
                          TMethod(msghandler).Data:=self;
                          msghandler(message);
                          exit;
                       end;
                  end;
                ovmt:=ovmt^.vParent;
             end;
           DefaultHandler(message);
        end;

      procedure TObject.DispatchStr(var message);

        var
           name : shortstring;
           count,i : longint;
           msgstrtable : pmsgstrtable;
           p: pstringmessagetable;
           ovmt : PVmt;
           msghandler : tmessagehandler;

        begin
           name:=pshortstring(@message)^;
           ovmt:=PVmt(ClassType);
           while assigned(ovmt) do
           begin
                p := ovmt^.vMsgStrPtr;
                if (P<>Nil) and (p^.count<>0) then
                  begin
                  count:=p^.count;
                  msgstrtable:=@p^.msgstrtable;
                  end
                else
                  Count:=0;
                { later, we can implement a binary search here }
                for i:=0 to count-1 do
                  begin
                     if name=msgstrtable[i].name^ then
                       begin
                          TMethod(msghandler).Code:=msgstrtable[i].method;
                          TMethod(msghandler).Data:=self;
                          msghandler(message);
                          exit;
                       end;
                  end;
                ovmt:=ovmt^.vParent;
           end;
           DefaultHandlerStr(message);
        end;

      procedure TObject.DefaultHandler(var message);

        begin
        end;

      procedure TObject.DefaultHandlerStr(var message);

        begin
        end;

      procedure TObject.CleanupInstance;

        var
           vmt  : PVmt;
           temp : pointer;
        begin
           vmt := PVmt(ClassType);
           while vmt<>nil do
             begin
               Temp:= vmt^.vInitTable;
               { The RTTI format matches one for records, except the type is tkClass.
                 Since RecordRTTI does not check the type, calling it yields the desired result. }
               if Assigned(Temp) then
                 RecordRTTI(Self,Temp,@int_finalize);
               vmt:= vmt^.vParent;
             end;
        end;

      procedure TObject.AfterConstruction;

        begin
        end;

      procedure TObject.BeforeDestruction;

        begin
        end;

      function IsGUIDEqual(const guid1, guid2: tguid): boolean;
        begin
          IsGUIDEqual:=
            (guid1.D1=guid2.D1) and
            (PDWORD(@guid1.D2)^=PDWORD(@guid2.D2)^) and
            (PDWORD(@guid1.D4[0])^=PDWORD(@guid2.D4[0])^) and
            (PDWORD(@guid1.D4[4])^=PDWORD(@guid2.D4[4])^);
        end;

      // Use of managed types should be avoided here; implicit _Addref/_Release
      // will end up in unpredictable behaviour if called on CORBA interfaces.
      type
        TInterfaceGetter = procedure(out Obj) of object;
        TClassGetter = function: TObject of object;

      function GetInterfaceByEntry(Instance: pointer; IEntry: pinterfaceentry; out obj): boolean;
        var
          Getter: TMethod;
        begin
          Pointer(Obj) := nil;
          Getter.Data := Instance;
          if Assigned(IEntry) and Assigned(Instance) then
          begin
            case IEntry^.IType of
              etStandard:
                  Pointer(Obj) := PByte(instance)+IEntry^.IOffset;
              etFieldValue, etFieldValueClass:
                  Pointer(obj) := PPointer(PByte(Instance)+IEntry^.IOffset)^;
              etVirtualMethodResult:
                begin
                  // IOffset is relative to the VMT, not to instance.
                  Getter.code := PCodePointer(PByte(PPointer(Instance)^) + IEntry^.IOffset)^;
                  TInterfaceGetter(Getter)(obj);
                end;
              etVirtualMethodClass:
                begin
                  // IOffset is relative to the VMT, not to instance.
                  Getter.code := PCodePointer(PByte(PPointer(Instance)^) + IEntry^.IOffset)^;
                  TObject(obj) := TClassGetter(Getter)();
                end;
              etStaticMethodResult:
                begin
                  Getter.code := IEntry^.IOffsetAsCodePtr;
                  TInterfaceGetter(Getter)(obj);
                end;
              etStaticMethodClass:
                begin
                  Getter.code := IEntry^.IOffsetAsCodePtr;
                  TObject(obj) := TClassGetter(Getter)();
                end;
            end;
          end;
          result := assigned(pointer(obj));
        end;

      function TObject.GetInterface(const iid : tguid;out obj) : boolean;
        var
          IEntry: PInterfaceEntry;
          Instance: TObject;
        begin
          if IsGUIDEqual(IObjectInstance,iid) then
          begin
            TObject(Obj) := Self;
            Result := True;
            Exit;
          end;

          Instance := self;
          repeat
            IEntry := Instance.GetInterfaceEntry(iid);
            result := GetInterfaceByEntry(Instance, IEntry, obj);

            if (not result) or
              (IEntry^.IType in [etStandard, etFieldValue,
               etStaticMethodResult, etVirtualMethodResult]) then
              Break;

            { if interface is implemented by a class-type property or field,
              continue search }
            Instance := TObject(obj);
          until False;

          { Getter function will normally AddRef, so adding another reference here
            will cause memleak. }
          if result and (IEntry^.IType in [etStandard, etFieldValue]) then
            IInterface(obj)._AddRef;
        end;

      function TObject.GetInterfaceWeak(const iid : tguid; out obj) : boolean;
        var
          IEntry: PInterfaceEntry;
          Instance: TObject;
        begin
          if IsGUIDEqual(IObjectInstance,iid) then
          begin
            TObject(Obj) := Self;
            Result := True;
            Exit;
          end;

          Instance := self;
          repeat
            IEntry := Instance.GetInterfaceEntry(iid);
            result := GetInterfaceByEntry(Instance, IEntry, obj);

            if (not result) or
              (IEntry^.IType in [etStandard, etFieldValue,
               etStaticMethodResult, etVirtualMethodResult]) then
              Break;

            { if interface is implemented by a class-type property or field,
              continue search }
            Instance := TObject(obj);
          until False;

          { Getter function will normally AddRef, so we have to release it,
            else the ref is not weak. }
          if result and not (IEntry^.IType in [etStandard, etFieldValue]) then
            IInterface(obj)._Release;
        end;

      function TObject.GetInterfaceByStr(const iidstr : shortstring;out obj) : boolean;
        var
          IEntry: PInterfaceEntry;
          Instance: TObject;
        begin
          Instance := self;
          repeat
            IEntry := Instance.GetInterfaceEntryByStr(iidstr);
            result := GetInterfaceByEntry(Instance, IEntry, obj);

            if (not result) or
              (IEntry^.IType in [etStandard, etFieldValue,
               etStaticMethodResult, etVirtualMethodResult]) then
              Break;

            { if interface is implemented by a class-type property or field,
              continue search }
            Instance := TObject(obj);
          until False;

          { Getter function will normally AddRef, so adding another reference here
            will cause memleak. (com interfaces only!) }
          if result and Assigned(IEntry^.IID) and (IEntry^.IType in [etStandard, etFieldValue]) then
            IInterface(obj)._AddRef;
        end;

      function TObject.GetInterface(const iidstr : shortstring;out obj) : boolean;
        begin
          Result := GetInterfaceByStr(iidstr,obj);
        end;

      class function TObject.GetInterfaceEntry(const iid : tguid) : pinterfaceentry;
        var
          i: longint;
          intftable: pinterfacetable;
          ovmt: PVmt;
        begin
          ovmt := PVmt(Self);
          while Assigned(ovmt) and {$ifdef VER3_0}(ovmt^.vIntfTable <> @emptyintf){$else}Assigned(ovmt^.vIntftable){$endif} do
          begin
            intftable:=ovmt^.vIntfTable;
            {$ifdef VER3_0}
            if assigned(intftable) then
            {$endif VER3_0}
            begin
              for i:=0 to intftable^.EntryCount-1 do
              begin
                result:=@intftable^.Entries[i];
                if assigned(Result^.iid) and IsGUIDEqual(Result^.iid^,iid) then
                  Exit;
              end;
            end;
            ovmt := ovmt^.vParent;
          end;
          result := nil;
        end;

      class function TObject.GetInterfaceEntryByStr(const iidstr : shortstring) : pinterfaceentry;
        var
          i: longint;
          intftable: pinterfacetable;
          ovmt: PVmt;
        begin
          ovmt := PVmt(Self);
          while Assigned(ovmt) and {$ifdef VER3_0}(ovmt^.vIntfTable <> @emptyintf){$else}Assigned(ovmt^.vIntfTable){$endif} do
          begin
            intftable:=ovmt^.vIntfTable;
            {$ifdef VER3_0}
            if assigned(intftable) then
            {$endif VER3_0}
            begin
              for i:=0 to intftable^.EntryCount-1 do
              begin
                result:=@intftable^.Entries[i];
                if assigned(result^.iidstr) and (result^.iidstr^ = iidstr) then
                  Exit;
              end;
            end;
            ovmt := ovmt^.vParent;
          end;
          result:=nil;
        end;

      class function TObject.GetInterfaceTable : pinterfacetable;
        begin
          getinterfacetable:=PVmt(Self)^.vIntfTable;
        end;

      class function TObject.UnitName : {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}ansistring{$else FPC_HAS_FEATURE_ANSISTRINGS}shortstring{$endif FPC_HAS_FEATURE_ANSISTRINGS};
        type
          TClassTypeInfo = {$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}packed{$endif}record
            Attributes: Pointer;
            case TTypeKind of
              tkClass: (
                ClassType: TClass;
                ParentInfo: Pointer;
                PropCount: SmallInt;
                UnitName: ShortString;
              );
              { include for proper alignment }
              tkInt64: (
                Dummy: Int64;
              );
          end;
          PClassTypeInfo = ^TClassTypeInfo;
        var
          classtypeinfo: PClassTypeInfo;
        begin
          classtypeinfo:=ClassInfo;
          if Assigned(classtypeinfo) then
          begin
            // offset PTypeInfo by Length(Name) + 2 (ShortString length byte + SizeOf(Kind))
            inc(Pointer(classtypeinfo), PByte(Pointer(classtypeinfo)+1)^ + 2);
            {$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
            classtypeinfo:=aligntoqword(classtypeinfo);
            {$endif}
            result:=classtypeinfo^.UnitName;
          end
          else
            result:='';
        end;

      class function TObject.QualifiedClassName: {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}ansistring{$else FPC_HAS_FEATURE_ANSISTRINGS}shortstring{$endif FPC_HAS_FEATURE_ANSISTRINGS};
        var
          uname: {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}ansistring{$else FPC_HAS_FEATURE_ANSISTRINGS}shortstring{$endif FPC_HAS_FEATURE_ANSISTRINGS};
        begin
          uname := UnitName; //TODO: change 'UnitName' to 'UnitScope' as soon as RTL implement it
          if uname='' then
            result:=ClassName
          else
            result:=Concat(uname, '.', ClassName);
        end;

      function TObject.Equals(Obj: TObject) : boolean;
        begin
          result:=Obj=Self;
        end;

      function TObject.GetHashCode: PtrInt;
        begin
          result:=PtrInt(Self);
        end;

      function TObject.ToString: {$ifdef FPC_HAS_FEATURE_ANSISTRINGS}ansistring{$else FPC_HAS_FEATURE_ANSISTRINGS}shortstring{$endif FPC_HAS_FEATURE_ANSISTRINGS};
        begin
          result:=ClassName;
        end;

{****************************************************************************
                               TINTERFACEDOBJECT
****************************************************************************}

    function TInterfacedObject.QueryInterface(
      {$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

      begin
         if getinterface(iid,obj) then
           result:=S_OK
         else
           result:=longint(E_NOINTERFACE);
      end;

    function TInterfacedObject._AddRef : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

      begin
         _addref:=interlockedincrement(frefcount);
      end;

    function TInterfacedObject._Release : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

      begin
         _Release:=interlockeddecrement(frefcount);
         if _Release=0 then
           begin
           if interlockedincrement(fdestroycount)=1 then
             self.destroy;
           end;
      end;

   destructor TInterfacedObject.Destroy;

   begin
     // We must explicitly reset. Bug ID 32353
     FRefCount:=0;
     FDestroyCount:=0;
     inherited Destroy;
   end;

    procedure TInterfacedObject.AfterConstruction;

      begin
         { we need to fix the refcount we forced in newinstance }
         { further, it must be done in a thread safe way        }
         declocked(frefcount);
      end;

    procedure TInterfacedObject.BeforeDestruction;

      begin
         if frefcount<>0 then
           HandleError(204);
      end;

    class function TInterfacedObject.NewInstance : TObject;

      begin
         NewInstance:=inherited NewInstance;
         if NewInstance<>nil then
           TInterfacedObject(NewInstance).frefcount:=1;
      end;

{****************************************************************************
                               TAGGREGATEDOBJECT
****************************************************************************}

    constructor TAggregatedObject.Create(const aController: IUnknown);

      begin
        inherited Create;
        { do not keep a counted reference to the controller! }
        fcontroller := Pointer(aController);
      end;

    function TAggregatedObject.QueryInterface(
      {$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

      begin
         Result := IUnknown(fcontroller).QueryInterface(iid, obj);
      end;

    function TAggregatedObject._AddRef : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

      begin
         Result := IUnknown(fcontroller)._AddRef;
      end;

    function TAggregatedObject._Release : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

      begin
         Result := IUnknown(fcontroller)._Release;
      end;

    function TAggregatedObject.GetController : IUnknown;

      begin
         Result := IUnknown(fcontroller);
      end;

{****************************************************************************
                               TContainedOBJECT
****************************************************************************}

    function TContainedObject.QueryInterface(
            {$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};

    begin
      if getinterface(iid,obj) then
        result:=S_OK
      else
        result:=longint(E_NOINTERFACE);
    end;

{****************************************************************************
                               TCustomAttribute
****************************************************************************}


    constructor TCustomAttribute.Create;
    begin
      inherited;
    end;


{****************************************************************************
                             Exception Support
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
{$i except.inc}
{$endif FPC_HAS_FEATURE_EXCEPTIONS}
