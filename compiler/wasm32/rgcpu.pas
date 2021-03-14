{
    Copyright (c) 2019 by Dmitry Boyarintsev

    This unit implements the WebAssembly specific class for the register
    allocator

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

 ****************************************************************************}
unit rgcpu;

{$i fpcdefs.inc}

  interface

    uses
      cclasses,
      aasmbase,aasmcpu,aasmtai,aasmdata,
      cgbase,cgutils, procinfo,
      cpubase,
      rgobj;

    type
      tspilltemps = array[tregistertype] of ^Tspill_temp_list;

      { trgcpu }

      trgcpu=class(trgobj)
       protected
        class procedure do_spill_replace_all(list:TAsmList;instr:taicpu;const spilltemps: tspilltemps);
        class procedure remove_dummy_load_stores(list: TAsmList; headertai: tai);
       public
        { performs the register allocation for *all* register types }
        class procedure do_all_register_allocation(list: TAsmList; headertai: tai);
      end;


implementation

    uses
      verbose,cutils,
      globtype,globals,
      cgobj,
      tgobj,
      symtype,symdef,symcpu;

    { trgcpu }

    class procedure trgcpu.do_spill_replace_all(list:TAsmList;instr:taicpu;const spilltemps: tspilltemps);
      var
        l: longint;
        reg: tregister;
      begin
        { WebAssebly instructions never have more than one memory (virtual register)
          operand, so there is no danger of superregister conflicts }
        for l:=0 to instr.ops-1 do
          if (instr.oper[l]^.typ=top_reg) and (instr.oper[l]^.reg<>NR_LOCAL_FRAME_POINTER_REG) then
            begin
              reg:=instr.oper[l]^.reg;
              instr.loadref(l,spilltemps[getregtype(reg)]^[getsupreg(reg)]);
            end;
      end;


    class procedure trgcpu.remove_dummy_load_stores(list: TAsmList; headertai: tai);

      type
        taitypeset =  set of taitype;

      function nextskipping(p: tai; const skip: taitypeset): tai;
        begin
          result:=p;
          if not assigned(result) then
            exit;
          repeat
            result:=tai(result.next);
          until not assigned(result) or
                not(result.typ in skip);
        end;

      function issimpleregstore(p: tai; var reg: tregister; doubleprecisionok: boolean): boolean;
        const
          simplestoressp = [a_f32_store];
          simplestoresdp = [a_f64_store];
        begin
          result:=
            assigned(p) and
            (p.typ=ait_instruction) and
            ((taicpu(p).opcode in simplestoressp) or
             (doubleprecisionok and
              (taicpu(p).opcode in simplestoresdp))) and
            ((reg=NR_NO) or
             (taicpu(p).oper[0]^.typ=top_reg) and
             (taicpu(p).oper[0]^.reg=reg));
          if result and
             (reg=NR_NO) then
            reg:=taicpu(p).oper[0]^.reg;
        end;

      function issimpleregload(p: tai; var reg: tregister; doubleprecisionok: boolean): boolean;
        const
          simpleloadssp = [a_f32_load];
          simpleloadsdp = [a_f64_load];
        begin
          result:=
            assigned(p) and
            (p.typ=ait_instruction) and
            ((taicpu(p).opcode in simpleloadssp) or
             (doubleprecisionok and
              (taicpu(p).opcode in simpleloadsdp))) and
            ((reg=NR_NO) or
             (taicpu(p).oper[0]^.typ=top_reg) and
             (taicpu(p).oper[0]^.reg=reg));
          if result and
             (reg=NR_NO) then
            reg:=taicpu(p).oper[0]^.reg;
        end;

      function isregallocoftyp(p: tai; typ: TRegAllocType;var reg: tregister): boolean;
        begin
          result:=
            assigned(p) and
            (p.typ=ait_regalloc) and
            (tai_regalloc(p).ratype=typ);
          if result then
            if reg=NR_NO then
              reg:=tai_regalloc(p).reg
            else
              result:=tai_regalloc(p).reg=reg;
        end;

      function regininstruction(p: tai; reg: tregister): boolean;
        var
          sr: tsuperregister;
          i: longint;
        begin
          result:=false;
          if p.typ<>ait_instruction then
            exit;
          sr:=getsupreg(reg);
          for i:=0 to taicpu(p).ops-1 do
            case taicpu(p).oper[0]^.typ of
              top_reg:
                if (getsupreg(taicpu(p).oper[0]^.reg)=sr) then
                  exit(true);
              top_ref:
                begin
                  if (getsupreg(taicpu(p).oper[0]^.ref^.base)=sr) then
                    exit(true);
                  if (getsupreg(taicpu(p).oper[0]^.ref^.index)=sr) then
                    exit(true);
                end;
              else
                ;
            end;
        end;

      function try_remove_store_dealloc_load(var p: tai): boolean;
        var
          dealloc,
          load: tai;
          reg: tregister;
        begin
          result:=false;
          { check for:
              store regx
              dealloc regx
              load regx
            and remove. We don't have to check that the load/store
            types match, because they have to for this to be
            valid WebAssembly code }
          dealloc:=nextskipping(p,[ait_comment,ait_tempalloc]);
          load:=nextskipping(dealloc,[ait_comment,ait_tempalloc]);
          reg:=NR_NO;
          if issimpleregstore(p,reg,true) and
             isregallocoftyp(dealloc,ra_dealloc,reg) and
             issimpleregload(load,reg,true) then
            begin
              { remove the whole sequence: the store }
              list.remove(p);
              p.free;
              p:=Tai(load.next);
              { the load }
              list.remove(load);
              load.free;

              result:=true;
            end;
        end;


     function try_swap_store_x_load(var p: tai): boolean;
       var
         insertpos,
         storex,
         deallocy,
         loady,
         deallocx,
         loadx: tai;
         swapxy: taicpu;
         regx, regy: tregister;
       begin
         result:=false;
         { check for:
             alloc regx (optional)
             store regx (p)
             dealloc regy
             load regy
             dealloc regx
             load regx
           and change to
             dealloc regy
             load regy
             swap
             alloc regx (if it existed)
             store regx
             dealloc regx
             load  regx

           This will create opportunities to remove the store/load regx
           (and possibly also for regy)
         }
         //regx:=NR_NO;
         //regy:=NR_NO;
         //if not issimpleregstore(p,regx,false) then
         //  exit;
         //storex:=p;
         //deallocy:=nextskipping(storex,[ait_comment,ait_tempalloc]);
         //loady:=nextskipping(deallocy,[ait_comment,ait_tempalloc]);
         //deallocx:=nextskipping(loady,[ait_comment,ait_tempalloc]);
         //loadx:=nextskipping(deallocx,[ait_comment,ait_tempalloc]);
         //if not assigned(loadx) then
         //  exit;
         //if not issimpleregload(loady,regy,false) then
         //  exit;
         //if not issimpleregload(loadx,regx,false) then
         //  exit;
         //if not isregallocoftyp(deallocy,ra_dealloc,regy) then
         //  exit;
         //if not isregallocoftyp(deallocx,ra_dealloc,regx) then
         //  exit;
         //insertpos:=tai(p.previous);
         //if not assigned(insertpos) or
         //   not isregallocoftyp(insertpos,ra_alloc,regx) then
         //  insertpos:=storex;
         //list.remove(deallocy);
         //list.insertbefore(deallocy,insertpos);
         //list.remove(loady);
         //list.insertbefore(loady,insertpos);
         //swapxy:=taicpu.op_none(a_swap);
         //swapxy.fileinfo:=taicpu(loady).fileinfo;
         //list.insertbefore(swapxy,insertpos);
         //result:=true;
       end;


      var
        p,next,nextnext: tai;
        reg: tregister;
        removedsomething: boolean;
      begin
        repeat
          removedsomething:=false;
          p:=headertai;
          while assigned(p) do
            begin
              case p.typ of
                ait_regalloc:
                  begin
                    reg:=NR_NO;
                    next:=nextskipping(p,[ait_comment,ait_tempalloc]);
                    nextnext:=nextskipping(next,[ait_comment,ait_regalloc]);
                    if assigned(nextnext) then
                      begin
                        { remove
                            alloc reg
                            dealloc reg

                          (can appear after optimisations, necessary to prevent
                           useless stack slot allocations) }
                        if isregallocoftyp(p,ra_alloc,reg) and
                           isregallocoftyp(next,ra_dealloc,reg) and
                           not regininstruction(nextnext,reg) then
                          begin
                            list.remove(p);
                            p.free;
                            p:=tai(next.next);
                            list.remove(next);
                            next.free;
                            removedsomething:=true;
                            continue;
                          end;
                      end;
                  end;
                ait_instruction:
                  begin
                    if try_remove_store_dealloc_load(p) or
                       try_swap_store_x_load(p) then
                      begin
                        removedsomething:=true;
                        continue;
                      end;
                  end;
                else
                  ;
              end;
              p:=tai(p.next);
            end;
        until not removedsomething;
      end;

    class procedure trgcpu.do_all_register_allocation(list: TAsmList; headertai: tai);
      var
        spill_temps : tspilltemps;
        templist : TAsmList;
        intrg,
        fprg     : trgcpu;
        p,q      : tai;
        size     : longint;

        insbefore : TLinkedListItem;
        lastins   : TLinkedListItem;
        //locavail  : array[TWasmBasicType] of tlocalalloc; // used or not

        ra      : tai_regalloc;
        idx     : integer;
        fidx    : integer;
        pidx    : integer;
        t: treftemppos;
        def: tdef;

      begin
        { Since there are no actual registers, we simply spill everything. We
          use tt_regallocator temps, which are not used by the temp allocator
          during code generation, so that we cannot accidentally overwrite
          any temporary values }

        { get references to all register allocators }
        intrg:=trgcpu(cg.rg[R_INTREGISTER]);
        fprg:=trgcpu(cg.rg[R_FPUREGISTER]);
        { determine the live ranges of all registers }
        intrg.insert_regalloc_info_all(list);
        fprg.insert_regalloc_info_all(list);
        { Don't do the actual allocation when -sr is passed }
        if (cs_no_regalloc in current_settings.globalswitches) then
          exit;
        { remove some simple useless store/load sequences }
        remove_dummy_load_stores(list,headertai);
        { allocate room to store the virtual register -> temp mapping }
        spill_temps[R_INTREGISTER]:=allocmem(sizeof(treference)*intrg.maxreg);
        spill_temps[R_FPUREGISTER]:=allocmem(sizeof(treference)*fprg.maxreg);
        { List to insert temp allocations into }
        templist:=TAsmList.create;
        { allocate/replace all registers }
        p:=headertai;
        insbefore := nil;
        while assigned(p) do
          begin
            case p.typ of
              ait_regalloc:
                  begin
                    ra := tai_regalloc(p);
                    case getregtype(ra.reg) of
                      R_INTREGISTER:
                        case getsubreg(ra.reg) of
                          R_SUBD:
                            begin
                              size:=4;
                              def:=s32inttype;
                            end;
                          R_SUBQ:
                            begin
                              size:=8;
                              def:=s64inttype;
                            end;
                          else
                            internalerror(2020120803);
                        end;
                      R_ADDRESSREGISTER:
                        begin
                          size:=4;
                          def:=voidpointertype;
                        end;
                      R_FPUREGISTER:
                        case getsubreg(ra.reg) of
                          R_SUBFS:
                            begin
                              size:=4;
                              def:=s32floattype;
                            end;
                          R_SUBFD:
                            begin
                              size:=8;
                              def:=s64floattype;
                            end;
                          else
                            internalerror(2020120804);
                        end;
                      else
                        internalerror(2010122912);
                    end;
                    case ra.ratype of
                      ra_alloc :
                        tg.gethltemp(templist,def,
                                     size,
                                     tt_regallocator,spill_temps[getregtype(ra.reg)]^[getsupreg(ra.reg)]);
                      ra_dealloc :
                        begin
                          tg.ungettemp(templist,spill_temps[getregtype(ra.reg)]^[getsupreg(ra.reg)]);
                          { don't invalidate the temp reference, may still be used one instruction
                            later }
                        end;
                      else
                        ;
                    end;
                    { insert the tempallocation/free at the right place }
                    { remove the register allocation info for the register
                      (p.previous is valid because we just inserted the temp
                       allocation/free before p) }
                    q:=Tai(p.previous);
                    list.remove(p);
                    p.free;
                    p:=q;
                  end;
              ait_instruction:
                do_spill_replace_all(list,taicpu(p),spill_temps);
              else
                ;
            end;
            p:=Tai(p.next);
          end;
        if templist.count>0 then
          list.insertListBefore(nil, templist);
        freemem(spill_temps[R_INTREGISTER]);
        freemem(spill_temps[R_FPUREGISTER]);
        templist.free;
      end;

end.
