{
    Copyright (c) 2002 by Florian Klaempfl

    Generates the argument location information for x86-64 target

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
unit cpupara;

{$i fpcdefs.inc}

  interface

    uses
      globtype,
      cpubase,cgbase,cgutils,
      symconst,symtype,symsym,symdef,
      aasmtai,aasmdata,
      parabase,paramgr;

    type
       tcpuparamanager = class(tparamanager)
       private
          procedure create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                               var intparareg,mmparareg,parasize:longint;varargsparas: boolean);
       public
          function param_use_paraloc(const cgpara:tcgpara):boolean;override;
          function push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
          function ret_in_param(def:tdef;pd:tabstractprocdef):boolean;override;
          function get_volatile_registers_int(calloption : tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_mm(calloption : tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_fpu(calloption : tproccalloption):tcpuregisterset;override;
          function create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;override;
          function create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> graemeg/cpstrnew
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> graemeg/cpstrnew
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> graemeg/cpstrnew
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> origin/cpstrnew
       end;

  implementation

    uses
       cutils,verbose,
       systems,
       defutil,
       symtable;

    const
      paraintsupregs : array[0..5] of tsuperregister = (RS_RDI,RS_RSI,RS_RDX,RS_RCX,RS_R8,RS_R9);
      parammsupregs : array[0..7] of tsuperregister = (RS_XMM0,RS_XMM1,RS_XMM2,RS_XMM3,RS_XMM4,RS_XMM5,RS_XMM6,RS_XMM7);

      paraintsupregs_winx64 : array[0..3] of tsuperregister = (RS_RCX,RS_RDX,RS_R8,RS_R9);
      parammsupregs_winx64 : array[0..3] of tsuperregister = (RS_XMM0,RS_XMM1,RS_XMM2,RS_XMM3);

<<<<<<< HEAD
<<<<<<< HEAD
{
   The argument classification code largely comes from libffi:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

   ffi64.c - Copyright (c) 2002, 2007  Bo Thorsen <bo@suse.de>
             Copyright (c) 2008  Red Hat, Inc.

   x86-64 Foreign Function Interface

   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   ``Software''), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED ``AS IS'', WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------- *)
}

    const
      MAX_PARA_CLASSES = 4;

    type
<<<<<<< HEAD
<<<<<<< HEAD
      tx64paraclasstype = (
=======
      tx64paraclass = (
>>>>>>> graemeg/cpstrnew
=======
      tx64paraclass = (
>>>>>>> origin/cpstrnew
        X86_64_NO_CLASS,
        X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS,
        X86_64_SSE_CLASS,X86_64_SSESF_CLASS,X86_64_SSEDF_CLASS,X86_64_SSEUP_CLASS,
        X86_64_X87_CLASS,X86_64_X87UP_CLASS,
        X86_64_COMPLEX_X87_CLASS,
        X86_64_MEMORY_CLASS
      );
<<<<<<< HEAD
<<<<<<< HEAD

      tx64paraclass = record
        def: tdef;
        typ: tx64paraclasstype;
=======
=======
>>>>>>> graemeg/cpstrnew

   ffi64.c - Copyright (c) 2002, 2007  Bo Thorsen <bo@suse.de>
             Copyright (c) 2008  Red Hat, Inc.

   x86-64 Foreign Function Interface

   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   ``Software''), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED ``AS IS'', WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------- *)
}

    const
      MAX_PARA_CLASSES = 4;

    type
      tx64paraclass = (
        X86_64_NO_CLASS,
        X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS,
        X86_64_SSE_CLASS,X86_64_SSESF_CLASS,X86_64_SSEDF_CLASS,X86_64_SSEUP_CLASS,
        X86_64_X87_CLASS,X86_64_X87UP_CLASS,
        X86_64_COMPLEX_X87_CLASS,
        X86_64_MEMORY_CLASS
      );
      tx64paraclasses = array[0..MAX_PARA_CLASSES-1] of tx64paraclass;

    { Win64-specific helper }
    function aggregate_in_registers_win64(varspez:tvarspez;size:longint):boolean;
      begin
    { TODO: Temporary hack: vs_const parameters are always passed by reference for win64}
        result:=(varspez=vs_value) and (size in [1,2,4,8])
      end;

    (* x86-64 register passing implementation.  See x86-64 ABI for details.  Goal
       of this code is to classify each 8bytes of incoming argument by the register
       class and assign registers accordingly.  *)

    (* Return the union class of CLASS1 and CLASS2.
       See the x86-64 PS ABI for details.  *)

    function merge_classes(class1, class2: tx64paraclass): tx64paraclass;
      begin
        (* Rule #1: If both classes are equal, this is the resulting class.  *)
        if (class1=class2) then
          exit(class1);

        (* Rule #2: If one of the classes is NO_CLASS, the resulting class is
           the other class.  *)
        if (class1=X86_64_NO_CLASS) then
          exit(class2);
        if (class2=X86_64_NO_CLASS) then
          exit(class1);

        (* Rule #3: If one of the classes is MEMORY, the result is MEMORY.  *)
        if (class1=X86_64_MEMORY_CLASS) or
           (class2=X86_64_MEMORY_CLASS) then
          exit(X86_64_MEMORY_CLASS);

        (* Rule #4: If one of the classes is INTEGER, the result is INTEGER.  *)
        { 32 bit }
        if ((class1=X86_64_INTEGERSI_CLASS) and
            (class2=X86_64_SSESF_CLASS)) or
           ((class2=X86_64_INTEGERSI_CLASS) and
            (class1=X86_64_SSESF_CLASS)) then
          exit(X86_64_INTEGERSI_CLASS);
        { 64 bit }
        if (class1 in [X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS]) or
           (class2 in [X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS]) then
          exit(X86_64_INTEGER_CLASS);

        (* Rule #5: If one of the classes is X87, X87UP, or COMPLEX_X87 class,
           MEMORY is used.  *)
        if (class1 in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS]) or
           (class2 in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS]) then
          exit(X86_64_MEMORY_CLASS);

        (* Rule #6: Otherwise class SSE is used.  *)
        result:=X86_64_SSE_CLASS;
      end;

    (* Classify the argument of type TYPE and mode MODE.
       CLASSES will be filled by the register class used to pass each word
       of the operand.  The number of words is returned.  In case the parameter
       should be passed in memory, 0 is returned. As a special case for zero
       sized containers, classes[0] will be NO_CLASS and 1 is returned.

       real_size contains either def.size, or a value derived from
       def.bitpackedsize and the field offset denoting the number of bytes
       spanned by a bitpacked field

       See the x86-64 PS ABI for details.
    *)
    function classify_as_integer_argument(real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        size: aint;
      begin
        size:=byte_offset+real_size;
        if size<=4 then
          classes[0]:=X86_64_INTEGERSI_CLASS
        else
          classes[0]:=X86_64_INTEGER_CLASS;
        if size<=8 then
          result:=1
        else
          begin
             if size<=12 then
               classes[1]:=X86_64_INTEGERSI_CLASS
             else if (size<=16) then
               classes[1]:=X86_64_INTEGER_CLASS
             else
               internalerror(2010021401);
            result:=2;
          end
      end;

<<<<<<< HEAD

    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint; forward;

    function init_aggregate_classification(def: tdef; varspez: tvarspez; out words: longint; out classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
=======
=======
>>>>>>> origin/cpstrnew
      tx64paraclasses = array[0..MAX_PARA_CLASSES-1] of tx64paraclass;

    { Win64-specific helper }
    function aggregate_in_registers_win64(varspez:tvarspez;size:longint):boolean;
      begin
    { TODO: Temporary hack: vs_const parameters are always passed by reference for win64}
        result:=(varspez=vs_value) and (size in [1,2,4,8])
      end;

    (* x86-64 register passing implementation.  See x86-64 ABI for details.  Goal
       of this code is to classify each 8bytes of incoming argument by the register
       class and assign registers accordingly.  *)

    (* Return the union class of CLASS1 and CLASS2.
       See the x86-64 PS ABI for details.  *)

    function merge_classes(class1, class2: tx64paraclass): tx64paraclass;
      begin
        (* Rule #1: If both classes are equal, this is the resulting class.  *)
        if (class1=class2) then
          exit(class1);

        (* Rule #2: If one of the classes is NO_CLASS, the resulting class is
           the other class.  *)
        if (class1=X86_64_NO_CLASS) then
          exit(class2);
        if (class2=X86_64_NO_CLASS) then
          exit(class1);

        (* Rule #3: If one of the classes is MEMORY, the result is MEMORY.  *)
        if (class1=X86_64_MEMORY_CLASS) or
           (class2=X86_64_MEMORY_CLASS) then
          exit(X86_64_MEMORY_CLASS);

        (* Rule #4: If one of the classes is INTEGER, the result is INTEGER.  *)
        { 32 bit }
        if ((class1=X86_64_INTEGERSI_CLASS) and
            (class2=X86_64_SSESF_CLASS)) or
           ((class2=X86_64_INTEGERSI_CLASS) and
            (class1=X86_64_SSESF_CLASS)) then
          exit(X86_64_INTEGERSI_CLASS);
        { 64 bit }
        if (class1 in [X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS]) or
           (class2 in [X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS]) then
          exit(X86_64_INTEGER_CLASS);

        (* Rule #5: If one of the classes is X87, X87UP, or COMPLEX_X87 class,
           MEMORY is used.  *)
        if (class1 in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS]) or
           (class2 in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS]) then
          exit(X86_64_MEMORY_CLASS);

        (* Rule #6: Otherwise class SSE is used.  *)
        result:=X86_64_SSE_CLASS;
      end;

    (* Classify the argument of type TYPE and mode MODE.
       CLASSES will be filled by the register class used to pass each word
       of the operand.  The number of words is returned.  In case the parameter
       should be passed in memory, 0 is returned. As a special case for zero
       sized containers, classes[0] will be NO_CLASS and 1 is returned.

       real_size contains either def.size, or a value derived from
       def.bitpackedsize and the field offset denoting the number of bytes
       spanned by a bitpacked field

       See the x86-64 PS ABI for details.
    *)
    function classify_as_integer_argument(real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        size: aint;
      begin
        size:=byte_offset+real_size;
        if size<=4 then
          classes[0]:=X86_64_INTEGERSI_CLASS
        else
          classes[0]:=X86_64_INTEGER_CLASS;
        if size<=8 then
          result:=1
        else
          begin
             if size<=12 then
               classes[1]:=X86_64_INTEGERSI_CLASS
             else if (size<=16) then
               classes[1]:=X86_64_INTEGER_CLASS
             else
               internalerror(2010021401);
            result:=2;
          end
      end;

<<<<<<< HEAD

    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint; forward;

    function init_aggregate_classification(def: tdef; varspez: tvarspez; out words: longint; out classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
>>>>>>> graemeg/cpstrnew
        words:=0;
        { win64 follows a different convention here }
        if (target_info.system=system_x86_64_win64) then
          begin
            if aggregate_in_registers_win64(varspez,def.size) then
              begin
                classes[0]:=X86_64_INTEGER_CLASS;
                result:=1;
              end
            else
              result:=0;
            exit;
          end;

        (* If the struct is larger than 32 bytes, pass it on the stack.  *)
        if def.size > 32 then
          exit(0);

        words:=(def.size+7) div 8;

        (* Zero sized arrays or structures are NO_CLASS.  We return 0 to
           signal memory class, so handle it as special case.  *)
        if (words=0) then
          begin
            classes[0]:=X86_64_NO_CLASS;
            exit(1);
          end;

        { we'll be merging the classes elements with the subclasses
          elements, so initialise them first }
        for i:=low(classes) to high(classes) do
          classes[i]:=X86_64_NO_CLASS;
        result:=words;
      end;


    function classify_aggregate_element(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; new_byte_offset: aint): longint;
      var
        subclasses: tx64paraclasses;
        i,
        pos: longint;
      begin
        result:=classify_argument(def,varspez,real_size,subclasses,new_byte_offset mod 8);
        if (result=0) then
          exit;
        pos:=new_byte_offset div 8;
        if result-1+pos>high(classes) then
          internalerror(2010053108);
        for i:=0 to result-1 do
          begin
            classes[i+pos] :=
              merge_classes(subclasses[i],classes[i+pos]);
          end;
      end;


    function finalize_aggregate_classification(def: tdef; words: longint; var classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
        if (words>2) then
          begin
            (* When size > 16 bytes, if the first one isn't
               X86_64_SSE_CLASS or any other ones aren't
               X86_64_SSEUP_CLASS, everything should be passed in
               memory.  *)
            if (classes[0]<>X86_64_SSE_CLASS) then
              exit(0);

            for i:=1 to words-1 do
              if (classes[i]<>X86_64_SSEUP_CLASS) then
                exit(0);
          end;

        (* Final merger cleanup.  *)
        (* The first one must never be X86_64_SSEUP_CLASS or
           X86_64_X87UP_CLASS.  *)
        if (classes[0]=X86_64_SSEUP_CLASS) or
           (classes[0]=X86_64_X87UP_CLASS) then
          internalerror(2010021402);
        for i:=0 to words-1 do
          begin
            (* If one class is MEMORY, everything should be passed in
               memory.  *)
            if (classes[i]=X86_64_MEMORY_CLASS) then
              exit(0);

            (* The X86_64_SSEUP_CLASS should be always preceded by
               X86_64_SSE_CLASS or X86_64_SSEUP_CLASS.  *)
            if (classes[i]=X86_64_SSEUP_CLASS) and
               (classes[i-1]<>X86_64_SSE_CLASS) and
               (classes[i-1]<>X86_64_SSEUP_CLASS) then
              classes[i]:=X86_64_SSE_CLASS;

            (*  If X86_64_X87UP_CLASS isn't preceded by X86_64_X87_CLASS,
                everything should be passed in memory.  *)
            if (classes[i]=X86_64_X87UP_CLASS) and
               (classes[i-1]<>X86_64_X87_CLASS) then
              exit(0);
          end;

          { FIXME: in case a record contains empty padding space, e.g. a
            "single" field followed by a "double", then we have a problem
            because the cgpara helpers cannot figure out that they should
            skip 4 bytes after storing the single (LOC_MMREGISTER with size
            OS_F32) to memory before storing the double -> for now scale
            such locations always up to 64 bits, although this loads/stores
            some superfluous data }
          { 1) the first part is 32 bit while there is still a second part }
          if (classes[1]<>X86_64_NO_CLASS) then
            case classes[0] of
              X86_64_INTEGERSI_CLASS:
                classes[0]:=X86_64_INTEGER_CLASS;
              X86_64_SSESF_CLASS:
                classes[0]:=X86_64_SSE_CLASS;
            end;
          { 2) the second part is 32 bit, but the total size is > 12 bytes }
          if (def.size>12) then
            case classes[1] of
              X86_64_INTEGERSI_CLASS:
                classes[1]:=X86_64_INTEGER_CLASS;
              X86_64_SSESF_CLASS:
                classes[1]:=X86_64_SSE_CLASS;
            end;

          result:=words;
      end;


    function classify_record(def: tdef; varspez: tvarspez; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        vs: tfieldvarsym;
        size,
        new_byte_offset: aint;
        i,
        words,
        num: longint;
      begin
        result:=init_aggregate_classification(def,varspez,words,classes);
        if (words=0) then
          exit;

        (* Merge the fields of the structure.  *)
        for i:=0 to tabstractrecorddef(def).symtable.symlist.count-1 do
          begin
            if tsym(tabstractrecorddef(def).symtable.symlist[i]).typ<>fieldvarsym then
              continue;
            vs:=tfieldvarsym(tabstractrecorddef(def).symtable.symlist[i]);
            num:=-1;
            if not tabstractrecordsymtable(tabstractrecorddef(def).symtable).is_packed then
              begin
                new_byte_offset:=byte_offset+vs.fieldoffset;
                size:=vs.vardef.size;
              end
            else
              begin
                new_byte_offset:=byte_offset+vs.fieldoffset div 8;
                if (vs.vardef.typ in [orddef,enumdef]) then
                  { calculate the number of bytes spanned by
                    this bitpacked field }
                  size:=((vs.fieldoffset+vs.vardef.packedbitsize+7) div 8)-(vs.fieldoffset div 8)
                else
                  size:=vs.vardef.size
              end;
            num:=classify_aggregate_element(vs.vardef,varspez,size,classes,new_byte_offset);
            if (num=0) then
              exit(0);
          end;

        result:=finalize_aggregate_classification(def,words,classes);
      end;


    function classify_normal_array(def: tarraydef; varspez: tvarspez; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        i, elecount: aword;
        size,
        elesize,
        new_byte_offset,
        bitoffset: aint;
        words,
        num: longint;
        isbitpacked: boolean;
      begin
        result:=init_aggregate_classification(def,varspez,words,classes);
        if (words=0) then
          exit;

        isbitpacked:=is_packed_array(def);
        if not isbitpacked then
          begin
            elesize:=def.elesize;
            size:=elesize;
          end
        else
          begin
            elesize:=def.elepackedbitsize;
            bitoffset:=0;
          end;

        (* Merge the elements of the array.  *)
        i:=0;
        elecount:=def.elecount;
        repeat
          if not isbitpacked then
            begin
              { size does not change }
              new_byte_offset:=byte_offset+i*elesize;
            end
          else
            begin
              { calculate the number of bytes spanned by this bitpacked
                element }
              size:=((bitoffset+elesize+7) div 8)-(bitoffset div 8);
              new_byte_offset:=byte_offset+(elesize*i) div 8;
              { bit offset of next element }
              inc(bitoffset,elesize);
            end;
          num:=classify_aggregate_element(def.elementdef,varspez,size,classes,new_byte_offset);
          if (num=0) then
            exit(0);
          inc(i);
        until (i=elecount);

        result:=finalize_aggregate_classification(def,words,classes);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
      end;

      tx64paraclasses = array[0..MAX_PARA_CLASSES-1] of tx64paraclass;

<<<<<<< HEAD
<<<<<<< HEAD
    { Win64-specific helper }
    function aggregate_in_registers_win64(varspez:tvarspez;size:longint):boolean;
      begin
    { TODO: Temporary hack: vs_const parameters are always passed by reference for win64}
        result:=(varspez=vs_value) and (size in [1,2,4,8])
      end;

    (* x86-64 register passing implementation.  See x86-64 ABI for details.  Goal
       of this code is to classify each 8bytes of incoming argument by the register
       class and assign registers accordingly.  *)


       function classify_representative_def(def1, def2: tdef): tdef;
         var
           def1size, def2size: asizeint;
         begin
           if not assigned(def1) then
             result:=def2
           else if not assigned(def2) then
             result:=def1
           else
             begin
               def1size:=def1.size;
               def2size:=def2.size;
               if def1size>def2size then
                 result:=def1
               else if def2size>def1size then
                 result:=def2
               else if def1.alignment>def2.alignment then
                 result:=def1
               else
                 result:=def2;
             end;
         end;

       (* Classify the argument of type TYPE and mode MODE.
          CLASSES will be filled by the register class used to pass each word
          of the operand.  The number of words is returned.  In case the parameter
          should be passed in memory, 0 is returned. As a special case for zero
          sized containers, classes[0] will be NO_CLASS and 1 is returned.

          real_size contains either def.size, or a value derived from
          def.bitpackedsize and the field offset denoting the number of bytes
          spanned by a bitpacked field

          See the x86-64 PS ABI for details.
       *)
       procedure classify_single_integer_class(def: tdef; size,real_size: aint; var cl: tx64paraclass; byte_offset: aint);
         begin
           if (byte_offset=0) and
              (real_size in [1,2,4,8]) and
              (not assigned(cl.def) or
               (def.alignment>=cl.def.alignment)) then
             cl.def:=def;
           if size<=4 then
             begin
               cl.typ:=X86_64_INTEGERSI_CLASS;
               { gcc/clang sign/zero-extend all values to 32 bits, except for
                 _Bool (= Pascal boolean), which is only zero-extended to 8 bits
                 as per the x86-64 ABI -> do the same }
               if not assigned(cl.def) or
                  not is_pasbool(cl.def) or
                  (size>1) then
                 cl.def:=u32inttype;
             end
           else
             begin
               cl.typ:=X86_64_INTEGER_CLASS;
               if not assigned(cl.def) or
                  (cl.def.size<size) or
                  (not(cl.def.typ in [orddef,floatdef,pointerdef,classrefdef]) and
                   not is_implicit_pointer_object_type(cl.def) and
                   not is_dynamicstring(cl.def) and
                   not is_dynamic_array(cl.def)) then
                 cl.def:=u64inttype;
             end;
         end;


       function classify_as_integer_argument(def: tdef; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint;
         var
           size: aint;
         begin
           size:=byte_offset+real_size;
           classify_single_integer_class(def,size,real_size,classes[0],byte_offset);
           if size<=8 then
             result:=1
           else
             begin
               classify_single_integer_class(def,size-8,real_size,classes[1],byte_offset-8);
               if size>16 then
                 internalerror(2010021401);
               result:=2;
             end
         end;


    (* Return the union class of CLASS1 and CLASS2.
       See the x86-64 PS ABI for details.  *)

    function merge_classes(class1, class2: tx64paraclass): tx64paraclass;
      begin
        (* Rule #1: If both classes are equal, this is the resulting class.  *)
        if (class1.typ=class2.typ) then
          begin
            result.typ:=class1.typ;
            result.def:=classify_representative_def(class1.def,class2.def);
            exit;
          end;

        (* Rule #2: If one of the classes is NO_CLASS, the resulting class is
           the other class.  *)
        if (class1.typ=X86_64_NO_CLASS) then
          exit(class2);
        if (class2.typ=X86_64_NO_CLASS) then
          exit(class1);

        (* Rule #3: If one of the classes is MEMORY, the result is MEMORY.  *)
        if (class1.typ=X86_64_MEMORY_CLASS) then
          exit(class1)
        else if (class2.typ=X86_64_MEMORY_CLASS) then
          exit(class2);

        (* Rule #4: If one of the classes is INTEGER, the result is INTEGER.  *)
        { 32 bit }
        if ((class1.typ=X86_64_INTEGERSI_CLASS) and
            (class2.typ=X86_64_SSESF_CLASS)) then
          exit(class1)
        else if ((class2.typ=X86_64_INTEGERSI_CLASS) and
            (class1.typ=X86_64_SSESF_CLASS)) then
          exit(class2);
        { 64 bit }
        if (class1.typ in [X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS]) then
          begin
            result:=class1;
            if result.def.size<8 then
              begin
                result.typ:=X86_64_INTEGER_CLASS;
                result.def:=s64inttype;
              end;
            exit
          end
        else if (class2.typ in [X86_64_INTEGER_CLASS,X86_64_INTEGERSI_CLASS]) then
          begin
            result:=class2;
            if result.def.size<8 then
              begin
                result.typ:=X86_64_INTEGER_CLASS;
                result.def:=s64inttype;
              end;
            exit
          end;

        (* Rule #5: If one of the classes is X87, X87UP, or COMPLEX_X87 class,
           MEMORY is used.  *)
        if (class1.typ in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS]) then
          begin
            result:=class1;
            result.typ:=X86_64_MEMORY_CLASS;
            exit;
          end
        else if (class2.typ in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS]) then
          begin
            result:=class2;
            result.typ:=X86_64_MEMORY_CLASS;
            exit;
          end;

        (* Rule #6: Otherwise class SSE is used.  *)
        if class1.def.size>class2.def.size then
          result:=class1
        else
          result:=class2;
        result.typ:=X86_64_SSE_CLASS;
        result.def:=s64floattype;
=======
    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint;
      begin
=======
    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint;
      begin
>>>>>>> graemeg/cpstrnew
        case def.typ of
          orddef,
          enumdef,
          pointerdef,
          classrefdef:
            result:=classify_as_integer_argument(real_size,classes,byte_offset);
          formaldef:
            result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset);
          floatdef:
            begin
              case tfloatdef(def).floattype of
                s32real:
                  begin
                    if byte_offset=0 then
                      classes[0]:=X86_64_SSESF_CLASS
                    else
                      { if we have e.g. a record with two successive "single"
                        fields, we need a 64 bit rather than a 32 bit load }
                      classes[0]:=X86_64_SSE_CLASS;
                    result:=1;
                  end;
                s64real:
                  begin
                    classes[0]:=X86_64_SSEDF_CLASS;
                    result:=1;
                  end;
                s80real,
                sc80real:
                  begin
                    classes[0]:=X86_64_X87_CLASS;
                    classes[1]:=X86_64_X87UP_CLASS;
                    result:=2;
                  end;
                s64comp,
                s64currency:
                  begin
                    classes[0]:=X86_64_INTEGER_CLASS;
                    result:=1;
                  end;
                s128real:
                  begin
                    classes[0]:=X86_64_SSE_CLASS;
                    classes[1]:=X86_64_SSEUP_CLASS;
                    result:=2;
                  end;
                else
                  internalerror(2010060301);
              end;
            end;
          recorddef:
            result:=classify_record(def,varspez,classes,byte_offset);
          objectdef:
            begin
              if is_object(def) then
                { pass by reference, like ppc and i386 }
                result:=0
              else
                { all kinds of pointer types: class, objcclass, interface, ... }
                result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset);
            end;
          setdef:
            begin
              if is_smallset(def) then
                result:=classify_as_integer_argument(def.size,classes,byte_offset)
              else
                result:=0;
            end;
          stringdef:
            begin
              if (tstringdef(def).stringtype in [st_shortstring,st_longstring]) then
                result:=0
              else
                result:=classify_as_integer_argument(def.size,classes,byte_offset);
            end;
          arraydef:
            begin
              { a dynamic array is treated like a pointer }
              if is_dynamic_array(def) then
                result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset)
              { other special arrays are passed on the stack }
              else if is_open_array(def) or
                      is_array_of_const(def) then
                result:=0
              else
              { normal array }
                result:=classify_normal_array(tarraydef(def),varspez,classes,byte_offset);
            end;
          { the file record is definitely too big }
          filedef:
            result:=0;
          procvardef:
            begin
              if (po_methodpointer in tprocvardef(def).procoptions) then
                begin
                  { treat as TMethod record }
                  def:=search_system_type('TMETHOD').typedef;
                  result:=classify_argument(def,varspez,def.size,classes,byte_offset);
                end
              else
                { pointer }
                result:=classify_as_integer_argument(def.size,classes,byte_offset);
            end;
          variantdef:
            begin
              { same as tvardata record }
              def:=search_system_type('TVARDATA').typedef;
              result:=classify_argument(def,varspez,def.size,classes,byte_offset);
            end;
          else
            internalerror(2010021405);
<<<<<<< HEAD
=======
        end;
      end;


    procedure getvalueparaloc(varspez:tvarspez;def:tdef;var loc1,loc2:tx64paraclass);
      var
        size: aint;
        i: longint;
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        { init the classes array, because even if classify_argument inits only
          one element we copy both to loc1/loc2 in case "1" is returned }
        for i:=low(classes) to high(classes) do
          classes[i]:=X86_64_NO_CLASS;
        { def.size internalerrors for open arrays and dynamic arrays, since
          their size cannot be determined at compile-time.
          classify_argument does not look at the realsize argument for arrays
          cases, but we obviously do have to pass something... }
        if is_special_array(def) then
          size:=-1
        else
          size:=def.size;
        numclasses:=classify_argument(def,varspez,size,classes,0);
        case numclasses of
          0:
           begin
             loc1:=X86_64_MEMORY_CLASS;
             loc2:=X86_64_NO_CLASS;
           end;
          1,2:
            begin
              { If the class is X87, X87UP or COMPLEX_X87, it is passed in memory }
              if classes[0] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[0]:=X86_64_MEMORY_CLASS;
              if classes[1] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[1]:=X86_64_MEMORY_CLASS;
              loc1:=classes[0];
              loc2:=classes[1];
            end
          else
            { 4 can only happen for _m256 vectors, not yet supported }
            internalerror(2010021501);
>>>>>>> graemeg/cpstrnew
        end;
      end;


    procedure getvalueparaloc(varspez:tvarspez;def:tdef;var loc1,loc2:tx64paraclass);
      var
<<<<<<< HEAD
        size: aint;
        i: longint;
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        { init the classes array, because even if classify_argument inits only
          one element we copy both to loc1/loc2 in case "1" is returned }
        for i:=low(classes) to high(classes) do
          classes[i]:=X86_64_NO_CLASS;
        { def.size internalerrors for open arrays and dynamic arrays, since
          their size cannot be determined at compile-time.
          classify_argument does not look at the realsize argument for arrays
          cases, but we obviously do have to pass something... }
        if is_special_array(def) then
          size:=-1
        else
          size:=def.size;
        numclasses:=classify_argument(def,varspez,size,classes,0);
        case numclasses of
          0:
           begin
             loc1:=X86_64_MEMORY_CLASS;
             loc2:=X86_64_NO_CLASS;
           end;
          1,2:
            begin
              { If the class is X87, X87UP or COMPLEX_X87, it is passed in memory }
              if classes[0] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[0]:=X86_64_MEMORY_CLASS;
              if classes[1] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[1]:=X86_64_MEMORY_CLASS;
              loc1:=classes[0];
              loc2:=classes[1];
            end
          else
            { 4 can only happen for _m256 vectors, not yet supported }
            internalerror(2010021501);
        end;
>>>>>>> graemeg/cpstrnew
      end;


    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint; forward;

    function init_aggregate_classification(def: tdef; varspez: tvarspez; byte_offset: aint; out words: longint; out classes: tx64paraclasses): longint;
      var
<<<<<<< HEAD
        i: longint;
      begin
        words:=0;
        { win64 follows a different convention here }
        if (target_info.system=system_x86_64_win64) then
          begin
            if aggregate_in_registers_win64(varspez,def.size) then
              begin
                classes[0].typ:=X86_64_INTEGER_CLASS;
                classes[0].def:=def;
                result:=1;
              end
            else
              result:=0;
            exit;
          end;

        (* If the struct is larger than 32 bytes, pass it on the stack.  *)
        if def.size > 32 then
          exit(0);

        { if a struct starts an offset not divisible by 8, it can span extra
          words }
        words:=(def.size+byte_offset mod 8+7) div 8;

        (* Zero sized arrays or structures are NO_CLASS.  We return 0 to
           signal memory class, so handle it as special case.  *)
        if (words=0) then
          begin
            classes[0].typ:=X86_64_NO_CLASS;
            classes[0].def:=def;
            exit(1);
          end;

        { we'll be merging the classes elements with the subclasses
          elements, so initialise them first }
        for i:=low(classes) to high(classes) do
          begin
            classes[i].typ:=X86_64_NO_CLASS;
            classes[i].def:=nil;
          end;
        result:=words;
      end;


    function classify_aggregate_element(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; new_byte_offset: aint): longint;
      var
        subclasses: tx64paraclasses;
        i,
        pos: longint;
      begin
        fillchar(subclasses,sizeof(subclasses),0);
        result:=classify_argument(def,varspez,real_size,subclasses,new_byte_offset mod 8);
        if (result=0) then
          exit;
        pos:=new_byte_offset div 8;
        if result-1+pos>high(classes) then
          internalerror(2010053108);
        for i:=0 to result-1 do
          begin
            classes[i+pos] :=
              merge_classes(subclasses[i],classes[i+pos]);
          end;
        inc(result,pos);
      end;


    function finalize_aggregate_classification(def: tdef; words: longint; var classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
        if (words>2) then
          begin
            (* When size > 16 bytes, if the first one isn't
               X86_64_SSE_CLASS or any other ones aren't
               X86_64_SSEUP_CLASS, everything should be passed in
               memory.  *)
            if (classes[0].typ<>X86_64_SSE_CLASS) then
              exit(0);

            for i:=1 to words-1 do
              if (classes[i].typ<>X86_64_SSEUP_CLASS) then
                exit(0);
          end;

        (* Final merger cleanup.  *)
        (* The first one must never be X86_64_SSEUP_CLASS or
           X86_64_X87UP_CLASS.  *)
        if (classes[0].typ=X86_64_SSEUP_CLASS) or
           (classes[0].typ=X86_64_X87UP_CLASS) then
          internalerror(2010021402);
        for i:=0 to words-1 do
          begin
            (* If one class is MEMORY, everything should be passed in
               memory.  *)
            if (classes[i].typ=X86_64_MEMORY_CLASS) then
              exit(0);

            (* The X86_64_SSEUP_CLASS should be always preceded by
               X86_64_SSE_CLASS or X86_64_SSEUP_CLASS.  *)
            if (classes[i].typ=X86_64_SSEUP_CLASS) and
               (classes[i-1].typ<>X86_64_SSE_CLASS) and
               (classes[i-1].typ<>X86_64_SSEUP_CLASS) then
              begin
                classes[i].typ:=X86_64_SSE_CLASS;
                classes[i].def:=carraydef.getreusable_no_free(s32floattype,2);
              end;

            (*  If X86_64_X87UP_CLASS isn't preceded by X86_64_X87_CLASS,
                everything should be passed in memory.  *)
            if (classes[i].typ=X86_64_X87UP_CLASS) and
               (classes[i-1].typ<>X86_64_X87_CLASS) then
              exit(0);

            (* FPC addition: because we store an extended in 10 bytes, the
               X86_64_X87UP_CLASS can be replaced with e.g. INTEGER if an
               extended is followed by e.g. an array [0..5] of byte -> we also
               have to check whether each X86_64_X87_CLASS is followed by
               X86_64_X87UP_CLASS -- if not, pass in memory

               This cannot happen in the original ABI, because there
               sizeof(extended) = 16 and hence nothing can be merged with
               X86_64_X87UP_CLASS and change it into something else *)
            if (classes[i].typ=X86_64_X87_CLASS) and
               ((i=(words-1)) or
                (classes[i+1].typ<>X86_64_X87UP_CLASS)) then
              exit(0);
          end;

{$ifndef llvm}
          { FIXME: in case a record contains empty padding space, e.g. a
            "single" field followed by a "double", then we have a problem
            because the cgpara helpers cannot figure out that they should
            skip 4 bytes after storing the single (LOC_MMREGISTER with size
            OS_F32) to memory before storing the double -> for now scale
            such locations always up to 64 bits, although this loads/stores
            some superfluous data }
          { 1) the first part is 32 bit while there is still a second part }
          if (classes[1].typ<>X86_64_NO_CLASS) then
            case classes[0].typ of
              X86_64_INTEGERSI_CLASS:
                begin
                  classes[0].typ:=X86_64_INTEGER_CLASS;
                  classes[0].def:=s64inttype;
                end;
              X86_64_SSESF_CLASS:
                begin
                  classes[0].typ:=X86_64_SSE_CLASS;
                  classes[0].def:=carraydef.getreusable_no_free(s32floattype,2);
                end;
            end;
          { 2) the second part is 32 bit, but the total size is > 12 bytes }
          if (def.size>12) then
            case classes[1].typ of
              X86_64_INTEGERSI_CLASS:
                begin
                  classes[1].typ:=X86_64_INTEGER_CLASS;
                  classes[1].def:=s64inttype;
                end;
              X86_64_SSESF_CLASS:
                begin
                  classes[1].typ:=X86_64_SSE_CLASS;
                  classes[1].def:=carraydef.getreusable_no_free(s32floattype,2);
                end;
            end;
{$endif not llvm}
          result:=words;
      end;


    function classify_record(def: tdef; varspez: tvarspez; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        vs: tfieldvarsym;
        size,
        new_byte_offset: aint;
        i,
        words,
        num: longint;
        checkalignment: boolean;
      begin
        result:=init_aggregate_classification(def,varspez,byte_offset,words,classes);
=======

    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint; forward;

    function init_aggregate_classification(def: tdef; varspez: tvarspez; out words: longint; out classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
        words:=0;
        { win64 follows a different convention here }
        if (target_info.system=system_x86_64_win64) then
          begin
            if aggregate_in_registers_win64(varspez,def.size) then
              begin
                classes[0]:=X86_64_INTEGER_CLASS;
                result:=1;
              end
            else
              result:=0;
            exit;
          end;

        (* If the struct is larger than 32 bytes, pass it on the stack.  *)
        if def.size > 32 then
          exit(0);

        words:=(def.size+7) div 8;

        (* Zero sized arrays or structures are NO_CLASS.  We return 0 to
           signal memory class, so handle it as special case.  *)
        if (words=0) then
          begin
            classes[0]:=X86_64_NO_CLASS;
            exit(1);
          end;

        { we'll be merging the classes elements with the subclasses
          elements, so initialise them first }
        for i:=low(classes) to high(classes) do
          classes[i]:=X86_64_NO_CLASS;
        result:=words;
      end;


    function classify_aggregate_element(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; new_byte_offset: aint): longint;
      var
        subclasses: tx64paraclasses;
        i,
        pos: longint;
      begin
        result:=classify_argument(def,varspez,real_size,subclasses,new_byte_offset mod 8);
        if (result=0) then
          exit;
        pos:=new_byte_offset div 8;
        if result-1+pos>high(classes) then
          internalerror(2010053108);
        for i:=0 to result-1 do
          begin
            classes[i+pos] :=
              merge_classes(subclasses[i],classes[i+pos]);
          end;
      end;


    function finalize_aggregate_classification(def: tdef; words: longint; var classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
=======

    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint; forward;

    function init_aggregate_classification(def: tdef; varspez: tvarspez; out words: longint; out classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
        words:=0;
        { win64 follows a different convention here }
        if (target_info.system=system_x86_64_win64) then
          begin
            if aggregate_in_registers_win64(varspez,def.size) then
              begin
                classes[0]:=X86_64_INTEGER_CLASS;
                result:=1;
              end
            else
              result:=0;
            exit;
          end;

        (* If the struct is larger than 32 bytes, pass it on the stack.  *)
        if def.size > 32 then
          exit(0);

        words:=(def.size+7) div 8;

        (* Zero sized arrays or structures are NO_CLASS.  We return 0 to
           signal memory class, so handle it as special case.  *)
        if (words=0) then
          begin
            classes[0]:=X86_64_NO_CLASS;
            exit(1);
          end;

        { we'll be merging the classes elements with the subclasses
          elements, so initialise them first }
        for i:=low(classes) to high(classes) do
          classes[i]:=X86_64_NO_CLASS;
        result:=words;
      end;


    function classify_aggregate_element(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; new_byte_offset: aint): longint;
      var
        subclasses: tx64paraclasses;
        i,
        pos: longint;
      begin
        result:=classify_argument(def,varspez,real_size,subclasses,new_byte_offset mod 8);
        if (result=0) then
          exit;
        pos:=new_byte_offset div 8;
        if result-1+pos>high(classes) then
          internalerror(2010053108);
        for i:=0 to result-1 do
          begin
            classes[i+pos] :=
              merge_classes(subclasses[i],classes[i+pos]);
          end;
      end;


    function finalize_aggregate_classification(def: tdef; words: longint; var classes: tx64paraclasses): longint;
      var
        i: longint;
      begin
>>>>>>> origin/cpstrnew
        if (words>2) then
          begin
            (* When size > 16 bytes, if the first one isn't
               X86_64_SSE_CLASS or any other ones aren't
               X86_64_SSEUP_CLASS, everything should be passed in
               memory.  *)
            if (classes[0]<>X86_64_SSE_CLASS) then
              exit(0);

            for i:=1 to words-1 do
              if (classes[i]<>X86_64_SSEUP_CLASS) then
                exit(0);
          end;

        (* Final merger cleanup.  *)
        (* The first one must never be X86_64_SSEUP_CLASS or
           X86_64_X87UP_CLASS.  *)
        if (classes[0]=X86_64_SSEUP_CLASS) or
           (classes[0]=X86_64_X87UP_CLASS) then
          internalerror(2010021402);
        for i:=0 to words-1 do
          begin
            (* If one class is MEMORY, everything should be passed in
               memory.  *)
            if (classes[i]=X86_64_MEMORY_CLASS) then
              exit(0);

            (* The X86_64_SSEUP_CLASS should be always preceded by
               X86_64_SSE_CLASS or X86_64_SSEUP_CLASS.  *)
            if (classes[i]=X86_64_SSEUP_CLASS) and
               (classes[i-1]<>X86_64_SSE_CLASS) and
               (classes[i-1]<>X86_64_SSEUP_CLASS) then
              classes[i]:=X86_64_SSE_CLASS;

            (*  If X86_64_X87UP_CLASS isn't preceded by X86_64_X87_CLASS,
                everything should be passed in memory.  *)
            if (classes[i]=X86_64_X87UP_CLASS) and
               (classes[i-1]<>X86_64_X87_CLASS) then
              exit(0);
          end;

          { FIXME: in case a record contains empty padding space, e.g. a
            "single" field followed by a "double", then we have a problem
            because the cgpara helpers cannot figure out that they should
            skip 4 bytes after storing the single (LOC_MMREGISTER with size
            OS_F32) to memory before storing the double -> for now scale
            such locations always up to 64 bits, although this loads/stores
            some superfluous data }
          { 1) the first part is 32 bit while there is still a second part }
          if (classes[1]<>X86_64_NO_CLASS) then
            case classes[0] of
              X86_64_INTEGERSI_CLASS:
                classes[0]:=X86_64_INTEGER_CLASS;
              X86_64_SSESF_CLASS:
                classes[0]:=X86_64_SSE_CLASS;
            end;
          { 2) the second part is 32 bit, but the total size is > 12 bytes }
          if (def.size>12) then
            case classes[1] of
              X86_64_INTEGERSI_CLASS:
                classes[1]:=X86_64_INTEGER_CLASS;
              X86_64_SSESF_CLASS:
                classes[1]:=X86_64_SSE_CLASS;
            end;

          result:=words;
      end;


    function classify_record(def: tdef; varspez: tvarspez; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        vs: tfieldvarsym;
        size,
        new_byte_offset: aint;
        i,
        words,
        num: longint;
      begin
        result:=init_aggregate_classification(def,varspez,words,classes);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if (words=0) then
          exit;

        (* Merge the fields of the structure.  *)
        for i:=0 to tabstractrecorddef(def).symtable.symlist.count-1 do
          begin
            if tsym(tabstractrecorddef(def).symtable.symlist[i]).typ<>fieldvarsym then
              continue;
            vs:=tfieldvarsym(tabstractrecorddef(def).symtable.symlist[i]);
            num:=-1;
<<<<<<< HEAD
<<<<<<< HEAD
            checkalignment:=true;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            if not tabstractrecordsymtable(tabstractrecorddef(def).symtable).is_packed then
              begin
                new_byte_offset:=byte_offset+vs.fieldoffset;
                size:=vs.vardef.size;
              end
            else
              begin
                new_byte_offset:=byte_offset+vs.fieldoffset div 8;
                if (vs.vardef.typ in [orddef,enumdef]) then
<<<<<<< HEAD
<<<<<<< HEAD
                  begin
                    { calculate the number of bytes spanned by
                      this bitpacked field }
                    size:=((vs.fieldoffset+vs.vardef.packedbitsize+7) div 8)-(vs.fieldoffset div 8);
                    { our bitpacked fields are interpreted as always being
                      aligned, because unlike in C we don't have char:1, int:1
                      etc (so everything is basically a char:x) }
                    checkalignment:=false;
                  end
                else
                  size:=vs.vardef.size;
              end;
            { If [..] an object [..] contains unaligned fields, it has class
              MEMORY }
            if checkalignment and
               (align(new_byte_offset,vs.vardef.structalignment)<>new_byte_offset) then
              begin
                result:=0;
                exit;
=======
=======
>>>>>>> origin/cpstrnew
                  { calculate the number of bytes spanned by
                    this bitpacked field }
                  size:=((vs.fieldoffset+vs.vardef.packedbitsize+7) div 8)-(vs.fieldoffset div 8)
                else
                  size:=vs.vardef.size
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              end;
            num:=classify_aggregate_element(vs.vardef,varspez,size,classes,new_byte_offset);
            if (num=0) then
              exit(0);
          end;

        result:=finalize_aggregate_classification(def,words,classes);
      end;


    function classify_normal_array(def: tarraydef; varspez: tvarspez; var classes: tx64paraclasses; byte_offset: aint): longint;
      var
        i, elecount: aword;
        size,
        elesize,
        new_byte_offset,
        bitoffset: aint;
        words,
        num: longint;
        isbitpacked: boolean;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        size:=0;
        bitoffset:=0;
        result:=init_aggregate_classification(def,varspez,byte_offset,words,classes);

=======
        result:=init_aggregate_classification(def,varspez,words,classes);
>>>>>>> graemeg/cpstrnew
=======
        result:=init_aggregate_classification(def,varspez,words,classes);
>>>>>>> origin/cpstrnew
        if (words=0) then
          exit;

        isbitpacked:=is_packed_array(def);
        if not isbitpacked then
          begin
            elesize:=def.elesize;
            size:=elesize;
          end
        else
          begin
            elesize:=def.elepackedbitsize;
            bitoffset:=0;
          end;

        (* Merge the elements of the array.  *)
        i:=0;
        elecount:=def.elecount;
        repeat
          if not isbitpacked then
            begin
              { size does not change }
              new_byte_offset:=byte_offset+i*elesize;
<<<<<<< HEAD
<<<<<<< HEAD
              { If [..] an object [..] contains unaligned fields, it has class
                MEMORY }
              if align(new_byte_offset,def.alignment)<>new_byte_offset then
                begin
                  result:=0;
                  exit;
                end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            end
          else
            begin
              { calculate the number of bytes spanned by this bitpacked
                element }
              size:=((bitoffset+elesize+7) div 8)-(bitoffset div 8);
              new_byte_offset:=byte_offset+(elesize*i) div 8;
              { bit offset of next element }
              inc(bitoffset,elesize);
            end;
          num:=classify_aggregate_element(def.elementdef,varspez,size,classes,new_byte_offset);
          if (num=0) then
            exit(0);
          inc(i);
        until (i=elecount);

        result:=finalize_aggregate_classification(def,words,classes);
      end;


    function classify_argument(def: tdef; varspez: tvarspez; real_size: aint; var classes: tx64paraclasses; byte_offset: aint): longint;
      begin
        case def.typ of
          orddef,
          enumdef,
          pointerdef,
          classrefdef:
<<<<<<< HEAD
<<<<<<< HEAD
            result:=classify_as_integer_argument(def,real_size,classes,byte_offset);
          formaldef:
            result:=classify_as_integer_argument(voidpointertype,voidpointertype.size,classes,byte_offset);
          floatdef:
            begin
              classes[0].def:=def;
=======
=======
>>>>>>> origin/cpstrnew
            result:=classify_as_integer_argument(real_size,classes,byte_offset);
          formaldef:
            result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset);
          floatdef:
            begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              case tfloatdef(def).floattype of
                s32real:
                  begin
                    if byte_offset=0 then
<<<<<<< HEAD
<<<<<<< HEAD
                      classes[0].typ:=X86_64_SSESF_CLASS
                    else
                      begin
                        { if we have e.g. a record with two successive "single"
                          fields, we need a 64 bit rather than a 32 bit load }
                        classes[0].typ:=X86_64_SSE_CLASS;
                        classes[0].def:=carraydef.getreusable_no_free(s32floattype,2);
                      end;
=======
=======
>>>>>>> origin/cpstrnew
                      classes[0]:=X86_64_SSESF_CLASS
                    else
                      { if we have e.g. a record with two successive "single"
                        fields, we need a 64 bit rather than a 32 bit load }
                      classes[0]:=X86_64_SSE_CLASS;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    result:=1;
                  end;
                s64real:
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    classes[0].typ:=X86_64_SSEDF_CLASS;
=======
                    classes[0]:=X86_64_SSEDF_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    classes[0]:=X86_64_SSEDF_CLASS;
>>>>>>> origin/cpstrnew
                    result:=1;
                  end;
                s80real,
                sc80real:
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    classes[0].typ:=X86_64_X87_CLASS;
                    classes[1].typ:=X86_64_X87UP_CLASS;
                    classes[1].def:=def;
=======
                    classes[0]:=X86_64_X87_CLASS;
                    classes[1]:=X86_64_X87UP_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    classes[0]:=X86_64_X87_CLASS;
                    classes[1]:=X86_64_X87UP_CLASS;
>>>>>>> origin/cpstrnew
                    result:=2;
                  end;
                s64comp,
                s64currency:
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    classes[0].typ:=X86_64_INTEGER_CLASS;
=======
                    classes[0]:=X86_64_INTEGER_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    classes[0]:=X86_64_INTEGER_CLASS;
>>>>>>> origin/cpstrnew
                    result:=1;
                  end;
                s128real:
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    classes[0].typ:=X86_64_SSE_CLASS;
                    classes[0].def:=carraydef.getreusable_no_free(s32floattype,2);
                    classes[1].typ:=X86_64_SSEUP_CLASS;
                    classes[1].def:=carraydef.getreusable_no_free(s32floattype,2);
=======
                    classes[0]:=X86_64_SSE_CLASS;
                    classes[1]:=X86_64_SSEUP_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    classes[0]:=X86_64_SSE_CLASS;
                    classes[1]:=X86_64_SSEUP_CLASS;
>>>>>>> origin/cpstrnew
                    result:=2;
                  end;
                else
                  internalerror(2010060301);
              end;
            end;
          recorddef:
            result:=classify_record(def,varspez,classes,byte_offset);
          objectdef:
            begin
              if is_object(def) then
                { pass by reference, like ppc and i386 }
                result:=0
              else
                { all kinds of pointer types: class, objcclass, interface, ... }
<<<<<<< HEAD
<<<<<<< HEAD
                result:=classify_as_integer_argument(def,voidpointertype.size,classes,byte_offset);
=======
                result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset);
>>>>>>> graemeg/cpstrnew
=======
                result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset);
>>>>>>> origin/cpstrnew
            end;
          setdef:
            begin
              if is_smallset(def) then
<<<<<<< HEAD
<<<<<<< HEAD
                result:=classify_as_integer_argument(def,def.size,classes,byte_offset)
=======
                result:=classify_as_integer_argument(def.size,classes,byte_offset)
>>>>>>> graemeg/cpstrnew
=======
                result:=classify_as_integer_argument(def.size,classes,byte_offset)
>>>>>>> origin/cpstrnew
              else
                result:=0;
            end;
          stringdef:
            begin
              if (tstringdef(def).stringtype in [st_shortstring,st_longstring]) then
                result:=0
              else
<<<<<<< HEAD
<<<<<<< HEAD
                result:=classify_as_integer_argument(def,def.size,classes,byte_offset);
=======
                result:=classify_as_integer_argument(def.size,classes,byte_offset);
>>>>>>> graemeg/cpstrnew
=======
                result:=classify_as_integer_argument(def.size,classes,byte_offset);
>>>>>>> origin/cpstrnew
            end;
          arraydef:
            begin
              { a dynamic array is treated like a pointer }
              if is_dynamic_array(def) then
<<<<<<< HEAD
<<<<<<< HEAD
                result:=classify_as_integer_argument(def,voidpointertype.size,classes,byte_offset)
=======
                result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset)
>>>>>>> graemeg/cpstrnew
=======
                result:=classify_as_integer_argument(voidpointertype.size,classes,byte_offset)
>>>>>>> origin/cpstrnew
              { other special arrays are passed on the stack }
              else if is_open_array(def) or
                      is_array_of_const(def) then
                result:=0
              else
              { normal array }
                result:=classify_normal_array(tarraydef(def),varspez,classes,byte_offset);
            end;
          { the file record is definitely too big }
          filedef:
            result:=0;
          procvardef:
            begin
              if (po_methodpointer in tprocvardef(def).procoptions) then
                begin
                  { treat as TMethod record }
                  def:=search_system_type('TMETHOD').typedef;
                  result:=classify_argument(def,varspez,def.size,classes,byte_offset);
                end
              else
                { pointer }
<<<<<<< HEAD
<<<<<<< HEAD
                result:=classify_as_integer_argument(def,def.size,classes,byte_offset);
=======
                result:=classify_as_integer_argument(def.size,classes,byte_offset);
>>>>>>> graemeg/cpstrnew
=======
                result:=classify_as_integer_argument(def.size,classes,byte_offset);
>>>>>>> origin/cpstrnew
            end;
          variantdef:
            begin
              { same as tvardata record }
              def:=search_system_type('TVARDATA').typedef;
              result:=classify_argument(def,varspez,def.size,classes,byte_offset);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          undefineddef:
            { show shall we know?
              since classify_argument is called during parsing, see tw27685.pp,
              we handle undefineddef here }
            result:=0;
=======
>>>>>>> graemeg/cpstrnew
          else
            internalerror(2010021405);
=======
          else
            internalerror(2010021405);
        end;
      end;


    procedure getvalueparaloc(varspez:tvarspez;def:tdef;var loc1,loc2:tx64paraclass);
      var
        size: aint;
        i: longint;
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        { init the classes array, because even if classify_argument inits only
          one element we copy both to loc1/loc2 in case "1" is returned }
        for i:=low(classes) to high(classes) do
          classes[i]:=X86_64_NO_CLASS;
        { def.size internalerrors for open arrays and dynamic arrays, since
          their size cannot be determined at compile-time.
          classify_argument does not look at the realsize argument for arrays
          cases, but we obviously do have to pass something... }
        if is_special_array(def) then
          size:=-1
        else
          size:=def.size;
        numclasses:=classify_argument(def,varspez,size,classes,0);
        case numclasses of
          0:
           begin
             loc1:=X86_64_MEMORY_CLASS;
             loc2:=X86_64_NO_CLASS;
           end;
          1,2:
            begin
              { If the class is X87, X87UP or COMPLEX_X87, it is passed in memory }
              if classes[0] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[0]:=X86_64_MEMORY_CLASS;
              if classes[1] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[1]:=X86_64_MEMORY_CLASS;
              loc1:=classes[0];
              loc2:=classes[1];
            end
          else
            { 4 can only happen for _m256 vectors, not yet supported }
            internalerror(2010021501);
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2

    function structure_in_registers(varspez:tvarspez;size:longint):boolean;
      begin
        if (target_info.system=system_x86_64_win64) then
{$warning Temporary hack: vs_const parameters are always passed by reference for win64}
          result:=(varspez=vs_value) and (size in [1,2,4,8])
        else
          result:=(size<=16);
      end;


    procedure getvalueparaloc(varspez:tvarspez;p : tdef;var loc1,loc2:tcgloc);
      begin
        loc1:=LOC_INVALID;
        loc2:=LOC_INVALID;
        case p.typ of
           orddef:
             begin
               loc1:=LOC_REGISTER;
               {$warning TODO 128bit also needs lochigh}
             end;
           floatdef:
             begin
               case tfloatdef(p).floattype of
                  s80real:
                    loc1:=LOC_REFERENCE;
                  s32real,
                  s64real :
                    loc1:=LOC_MMREGISTER;
                  s64currency,
                  s64comp :
                    loc1:=LOC_REGISTER;
                  s128real:
                    begin
                      loc1:=LOC_MMREGISTER;
                      loc2:=LOC_MMREGISTER;
                      {$warning TODO float 128bit needs SSEUP lochigh}
                    end;
               end;
             end;
           recorddef:
             begin
               if structure_in_registers(varspez,p.size) then
                 begin
                   loc1:=LOC_REGISTER;
                   if p.size>8 then
                     loc2:=LOC_REGISTER;
                 end
               else
                 loc1:=LOC_REFERENCE;
             end;
           objectdef:
             begin
               if is_object(p) then
                 begin
                   if structure_in_registers(varspez,p.size) then
                     loc1:=LOC_REGISTER
                   else
                     loc1:=LOC_REFERENCE;
                 end
               else
                 loc1:=LOC_REGISTER;
             end;
           arraydef:
             begin
               if not(is_special_array(p)) and
                  (target_info.system=system_x86_64_win64) and
                  structure_in_registers(varspez,p.size) then
                 begin
                   loc1:=LOC_REGISTER;
                   if p.size>8 then
                     loc2:=LOC_REGISTER;
                 end
               else
                 loc1:=LOC_REFERENCE;
             end;
           variantdef:
             { linux abi }
             if target_info.system<>system_x86_64_win64 then
               loc1:=LOC_REGISTER
             else
               loc1:=LOC_REFERENCE;
           stringdef:
             if is_shortstring(p) or is_longstring(p) then
               begin
                 { handle long and shortstrings like arrays }
                 if structure_in_registers(varspez,p.size) then
                   begin
                     loc1:=LOC_REGISTER;
                     if p.size>8 then
                       loc2:=LOC_REGISTER;
                   end
                 else
                   loc1:=LOC_REFERENCE;
               end
             else
               loc1:=LOC_REGISTER;
           setdef:
             if is_smallset(p) then
               loc1:=LOC_REGISTER
             else
               loc1:=LOC_REFERENCE;
           procvardef:
             begin
               if (po_methodpointer in tprocvardef(p).procoptions) then
                 begin
                   { This is a record of 16 bytes }
                   if structure_in_registers(varspez,p.size) then
                     begin
                       loc1:=LOC_REGISTER;
                       loc2:=LOC_REGISTER;
                     end
                   else
                     loc1:=LOC_REFERENCE;
                 end
               else
                 loc1:=LOC_REGISTER;
             end;
           else
             begin
               { default for pointers,enums,etc }
               loc1:=LOC_REGISTER;
             end;
>>>>>>> graemeg/fixes_2_2
        end;
      end;


    procedure getvalueparaloc(varspez:tvarspez;def:tdef;var loc1,loc2:tx64paraclass);
      var
<<<<<<< HEAD
        size: aint;
        i: longint;
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        { init the classes array, because even if classify_argument inits only
          one element we copy both to loc1/loc2 in case "1" is returned }
        for i:=low(classes) to high(classes) do
<<<<<<< HEAD
          begin
            classes[i].typ:=X86_64_NO_CLASS;
            classes[i].def:=nil;
          end;
=======
          classes[i]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
        { def.size internalerrors for open arrays and dynamic arrays, since
          their size cannot be determined at compile-time.
          classify_argument does not look at the realsize argument for arrays
          cases, but we obviously do have to pass something... }
        if is_special_array(def) then
          size:=-1
        else
          size:=def.size;
        numclasses:=classify_argument(def,varspez,size,classes,0);
        case numclasses of
          0:
           begin
<<<<<<< HEAD
             loc1.typ:=X86_64_MEMORY_CLASS;
             loc1.def:=def;
             loc2.typ:=X86_64_NO_CLASS;
=======
             loc1:=X86_64_MEMORY_CLASS;
             loc2:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
           end;
          1,2:
            begin
              { If the class is X87, X87UP or COMPLEX_X87, it is passed in memory }
<<<<<<< HEAD
              if classes[0].typ in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[0].typ:=X86_64_MEMORY_CLASS;
              if classes[1].typ in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[1].typ:=X86_64_MEMORY_CLASS;
=======
              if classes[0] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[0]:=X86_64_MEMORY_CLASS;
              if classes[1] in [X86_64_X87_CLASS,X86_64_X87UP_CLASS,X86_64_COMPLEX_X87_CLASS] then
                classes[1]:=X86_64_MEMORY_CLASS;
>>>>>>> graemeg/cpstrnew
              loc1:=classes[0];
              loc2:=classes[1];
            end
          else
            { 4 can only happen for _m256 vectors, not yet supported }
            internalerror(2010021501);
<<<<<<< HEAD
=======
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        if ((target_info.system=system_x86_64_win64) and
            (calloption=pocall_safecall)) then
          exit(true);
        case def.typ of
          { for records it depends on their contents and size }
          recorddef,
          { make sure we handle 'procedure of object' correctly }
          procvardef:
            begin
              numclasses:=classify_argument(def,vs_value,def.size,classes,0);
              result:=(numclasses=0);
            end;
          else
            result:=inherited ret_in_param(def,calloption);
>>>>>>> graemeg/cpstrnew
=======
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        if ((target_info.system=system_x86_64_win64) and
            (calloption=pocall_safecall)) then
          exit(true);
        case def.typ of
          { for records it depends on their contents and size }
          recorddef,
          { make sure we handle 'procedure of object' correctly }
          procvardef:
            begin
              numclasses:=classify_argument(def,vs_value,def.size,classes,0);
              result:=(numclasses=0);
            end;
          else
            result:=inherited ret_in_param(def,calloption);
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
        end;
      end;


    function tcpuparamanager.ret_in_param(def:tdef;pd:tabstractprocdef):boolean;
      var
        classes: tx64paraclasses;
        numclasses: longint;
      begin
<<<<<<< HEAD
        if handle_common_ret_in_param(def,pd,result) then
          exit;
        fillchar(classes,sizeof(classes),0);
=======
        if ((target_info.system=system_x86_64_win64) and
            (calloption=pocall_safecall)) then
          exit(true);
>>>>>>> graemeg/cpstrnew
        case def.typ of
          { for records it depends on their contents and size }
          recorddef,
          { make sure we handle 'procedure of object' correctly }
          procvardef:
            begin
              numclasses:=classify_argument(def,vs_value,def.size,classes,0);
              result:=(numclasses=0);
            end;
          else
<<<<<<< HEAD
            result:=inherited ret_in_param(def,pd);
=======
            result:=inherited ret_in_param(def,calloption);
>>>>>>> graemeg/cpstrnew
=======
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        if ((target_info.system=system_x86_64_win64) and
            (calloption=pocall_safecall)) then
          exit(true);
        case def.typ of
          { for records it depends on their contents and size }
          recorddef,
          { make sure we handle 'procedure of object' correctly }
          procvardef:
            begin
              numclasses:=classify_argument(def,vs_value,def.size,classes,0);
              result:=(numclasses=0);
            end;
          else
            result:=inherited ret_in_param(def,calloption);
>>>>>>> origin/cpstrnew
        end;
      end;


    function tcpuparamanager.param_use_paraloc(const cgpara:tcgpara):boolean;
      var
        paraloc : pcgparalocation;
      begin
        if not assigned(cgpara.location) then
          internalerror(200410102);
        result:=true;
        { All locations are LOC_REFERENCE }
        paraloc:=cgpara.location;
        while assigned(paraloc) do
          begin
            if (paraloc^.loc<>LOC_REFERENCE) then
              begin
                result:=false;
                exit;
              end;
            paraloc:=paraloc^.next;
          end;
      end;


    { true if a parameter is too large to copy and only the address is pushed }
<<<<<<< HEAD
    function tcpuparamanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
=======
    function tx86_64paramanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      var
        classes: tx64paraclasses;
        numclasses: longint;
      begin
        fillchar(classes,sizeof(classes),0);
        result:=false;
        { var,out,constref always require address }
        if varspez in [vs_var,vs_out,vs_constref] then
          begin
            result:=true;
            exit;
          end;
        { Only vs_const, vs_value here }
        case def.typ of
          formaldef :
            result:=true;
          recorddef :
            begin
<<<<<<< HEAD
<<<<<<< HEAD
              { MetroWerks Pascal: const records always passed by reference
                (for Mac OS X interfaces) }
              if (calloption=pocall_mwpascal) and
                 (varspez=vs_const) then
                result:=true
              { Win ABI depends on size to pass it in a register or not }
              else if (target_info.system=system_x86_64_win64) then
                result:=not aggregate_in_registers_win64(varspez,def.size)
              { pass constant parameters that would be passed via memory by
                reference for non-cdecl/cppdecl, and make sure that the tmethod
                record (size=16) is passed the same way as a complex procvar }
              else if ((varspez=vs_const) and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       not(calloption in cdecl_pocalls)) or
=======
                       not(calloption in [pocall_cdecl,pocall_cppdecl])) or
>>>>>>> graemeg/cpstrnew
=======
                       not(calloption in [pocall_cdecl,pocall_cppdecl])) or
>>>>>>> graemeg/cpstrnew
=======
                       not(calloption in [pocall_cdecl,pocall_cppdecl])) or
>>>>>>> graemeg/cpstrnew
=======
                       not(calloption in [pocall_cdecl,pocall_cppdecl])) or
>>>>>>> origin/cpstrnew
                      (def.size=16) then
                begin
                  numclasses:=classify_argument(def,vs_value,def.size,classes,0);
                  result:=numclasses=0;
                end
              else
              { SysV ABI always passes it as value parameter }
=======
=======
>>>>>>> origin/fixes_2_2
              { Win ABI depends on size to pass it in a register or not }
              if (target_info.system=system_x86_64_win64) then
                result:=not structure_in_registers(varspez,def.size)
              else
              { linux ABI always passes it as value parameter }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                result:=false;
            end;
          arraydef :
            begin
              { cdecl array of const need to be ignored and therefor be puhsed
                as value parameter with length 0 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              if ((calloption in cdecl_pocalls) and
=======
              if ((calloption in [pocall_cdecl,pocall_cppdecl]) and
>>>>>>> graemeg/cpstrnew
=======
              if ((calloption in [pocall_cdecl,pocall_cppdecl]) and
>>>>>>> graemeg/cpstrnew
=======
              if ((calloption in [pocall_cdecl,pocall_cppdecl]) and
>>>>>>> graemeg/cpstrnew
=======
              if ((calloption in [pocall_cdecl,pocall_cppdecl]) and
>>>>>>> origin/cpstrnew
                  is_array_of_const(def)) or
                 is_dynamic_array(def) then
                result:=false
              else
                { pass all arrays by reference to be compatible with C (passing
                  an array by value (= copying it on the stack) does not exist,
                  because an array is the same as a pointer there }
                result:=true
            end;
          objectdef :
            begin
              { don't treat objects like records, because we only know wheter
                or not they'll have a VMT after the entire object is parsed
                -> if they are used as function result from one of their own
                methods, their size can still change after we've determined
                whether this function result should be returned by reference or
                by value }
              if is_object(def) then
                result:=true;
            end;
          variantdef,
          stringdef,
          procvardef,
=======
=======
>>>>>>> origin/fixes_2_2
              if (calloption in [pocall_cdecl,pocall_cppdecl]) and
                 (is_array_of_const(def) or
                  is_dynamic_array(def)) then
                result:=false
              else
                result:=true;
            end;
          objectdef :
            begin
              if is_object(def) then
                result:=not structure_in_registers(varspez,def.size);
            end;
          stringdef :
            begin
              if (tstringdef(def).stringtype in [st_shortstring,st_longstring]) then
                result:=not structure_in_registers(varspez,def.size);
            end;
          procvardef :
            begin
              if (po_methodpointer in tprocvardef(def).procoptions) then
                result:=not structure_in_registers(varspez,def.size);
            end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          setdef :
            begin
              numclasses:=classify_argument(def,vs_value,def.size,classes,0);
              result:=numclasses=0;
            end;
        end;
      end;


    function tcpuparamanager.get_volatile_registers_int(calloption : tproccalloption):tcpuregisterset;
      begin
        if target_info.system=system_x86_64_win64 then
          result:=[RS_RAX,RS_RCX,RS_RDX,RS_R8,RS_R9,RS_R10,RS_R11]
        else
          result:=[RS_RAX,RS_RCX,RS_RDX,RS_RSI,RS_RDI,RS_R8,RS_R9,RS_R10,RS_R11];
      end;


    function tcpuparamanager.get_volatile_registers_mm(calloption : tproccalloption):tcpuregisterset;
      begin
        if target_info.system=system_x86_64_win64 then
          result:=[RS_XMM0..RS_XMM5]
        else
          result:=[RS_XMM0..RS_XMM15];
      end;


    function tcpuparamanager.get_volatile_registers_fpu(calloption : tproccalloption):tcpuregisterset;
      begin
        result:=[RS_ST0..RS_ST7];
      end;


    function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
      const
        intretregs: array[0..1] of tregister = (NR_FUNCTION_RETURN_REG,NR_FUNCTION_RETURN_REG_HIGH);
        mmretregs: array[0..1] of tregister = (NR_MM_RESULT_REG,NR_MM_RESULT_REG_HIGH);
      var
        classes: tx64paraclasses;
        i,
        numclasses: longint;
        intretregidx,
        mmretregidx: longint;
        retcgsize : tcgsize;
        paraloc : pcgparalocation;
      begin
        if set_common_funcretloc_info(p,forcetempdef,retcgsize,result) then
          exit;

<<<<<<< HEAD
        { Return in FPU register? -> don't use classify_argument(), because
          currency and comp need special treatment here (they are integer class
          when passing as parameter, but LOC_FPUREGISTER as function result) }
        if result.def.typ=floatdef then
          begin
            paraloc:=result.add_location;
            paraloc^.def:=result.def;
            case tfloatdef(result.def).floattype of
              s32real:
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMS);
                  paraloc^.size:=OS_F32;
                end;
              s64real:
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMD);
                  paraloc^.size:=OS_F64;
                end;
=======
    procedure tx86_64paramanager.create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;


<<<<<<< HEAD
=======
    procedure tx86_64paramanager.create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function tx86_64paramanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;
      const
        intretregs: array[0..1] of tregister = (NR_FUNCTION_RETURN_REG,NR_FUNCTION_RETURN_REG_HIGH);
        mmretregs: array[0..1] of tregister = (NR_MM_RESULT_REG,NR_MM_RESULT_REG_HIGH);
      var
        classes: tx64paraclasses;
        i,
        numclasses: longint;
        intretregidx,
        mmretregidx: longint;
        retcgsize : tcgsize;
        paraloc : pcgparalocation;
      begin
        result.init;
        result.alignment:=get_para_align(p.proccalloption);
        { void has no location }
        if is_void(def) then
          begin
            paraloc:=result.add_location;
            result.size:=OS_NO;
            result.intsize:=0;
            paraloc^.size:=OS_NO;
            paraloc^.loc:=LOC_VOID;
            exit;
          end;
        { Constructors return self instead of a boolean }
        if (p.proctypeoption=potype_constructor) then
          begin
            retcgsize:=OS_ADDR;
            result.intsize:=sizeof(pint);
          end
        else
          begin
            retcgsize:=def_cgsize(def);
            result.intsize:=def.size;
          end;
        result.size:=retcgsize;
        { Return is passed as var parameter }
        if ret_in_param(def,p.proccalloption) then
          begin
            paraloc:=result.add_location;
            paraloc^.loc:=LOC_REFERENCE;
            paraloc^.size:=retcgsize;
            exit;
          end;
<<<<<<< HEAD

        { Return in FPU register? -> don't use classify_argument(), because
          currency and comp need special treatment here (they are integer class
          when passing as parameter, but LOC_FPUREGISTER as function result) }
        if def.typ=floatdef then
=======
        { Return is passed as var parameter }
        if ret_in_param(p.returndef,p.proccalloption) then
          begin
            p.funcretloc[side].loc:=LOC_REFERENCE;
            p.funcretloc[side].size:=retcgsize;
            exit;
          end;
        { Return is passed as var parameter }
        if ret_in_param(p.returndef,p.proccalloption) then
          begin
            p.funcretloc[side].loc:=LOC_REFERENCE;
            p.funcretloc[side].size:=retcgsize;
            exit;
          end;
        { Return in FPU register? }
        if p.returndef.typ=floatdef then
>>>>>>> graemeg/fixes_2_2
          begin
            paraloc:=result.add_location;
            case tfloatdef(def).floattype of
              s32real:
<<<<<<< HEAD
<<<<<<< HEAD
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMS);
                  paraloc^.size:=OS_F32;
                end;
              s64real:
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMD);
                  paraloc^.size:=OS_F64;
                end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMS);
                  paraloc^.size:=OS_F32;
                end;
              s64real:
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMD);
                  paraloc^.size:=OS_F64;
                end;
>>>>>>> graemeg/cpstrnew
=======
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMS);
                  paraloc^.size:=OS_F32;
                end;
              s64real:
                begin
                  paraloc^.loc:=LOC_MMREGISTER;
                  paraloc^.register:=newreg(R_MMREGISTER,RS_MM_RESULT_REG,R_SUBMMD);
                  paraloc^.size:=OS_F64;
                end;
>>>>>>> origin/cpstrnew
              { the first two only exist on targets with an x87, on others
                they are replace by int64 }
              s64currency,
              s64comp,
              s80real,
              sc80real:
                begin
                  paraloc^.loc:=LOC_FPUREGISTER;
                  paraloc^.register:=NR_FPU_RESULT_REG;
                  paraloc^.size:=retcgsize;
                end;
              else
                internalerror(200405034);
            end;
          end
        else
         { Return in register }
          begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            fillchar(classes,sizeof(classes),0);
            numclasses:=classify_argument(result.def,vs_value,result.def.size,classes,0);
=======
            numclasses:=classify_argument(def,vs_value,def.size,classes,0);
>>>>>>> graemeg/cpstrnew
=======
            numclasses:=classify_argument(def,vs_value,def.size,classes,0);
>>>>>>> graemeg/cpstrnew
=======
            numclasses:=classify_argument(def,vs_value,def.size,classes,0);
>>>>>>> graemeg/cpstrnew
=======
            numclasses:=classify_argument(def,vs_value,def.size,classes,0);
>>>>>>> origin/cpstrnew
            { this would mean a memory return }
            if (numclasses=0) then
              internalerror(2010021502);
            { this would mean an _m256 vector (valid, but not yet supported) }
            if (numclasses>2) then
              internalerror(2010021503);
            intretregidx:=0;
            mmretregidx:=0;
            for i:=0 to numclasses-1 do
              begin
                paraloc:=result.add_location;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                paraloc^.def:=classes[i].def;
                case classes[i].typ of
=======
                case classes[i] of
>>>>>>> graemeg/cpstrnew
=======
                case classes[i] of
>>>>>>> graemeg/cpstrnew
                  X86_64_INTEGERSI_CLASS,
                  X86_64_INTEGER_CLASS:
                    begin
                      paraloc^.loc:=LOC_REGISTER;
                      paraloc^.register:=intretregs[intretregidx];
<<<<<<< HEAD
<<<<<<< HEAD
                      if classes[i].typ=X86_64_INTEGER_CLASS then
                        begin
                          paraloc^.size:=OS_64;
                          if paraloc^.def.size<>8 then
                            paraloc^.def:=u64inttype;
                        end
                      else if result.intsize in [1,2,4] then
                        begin
                          paraloc^.size:=def_cgsize(paraloc^.def);
                        end
                      else
                        begin
                          paraloc^.size:=OS_32;
                          if paraloc^.def.size<>4 then
                            paraloc^.def:=u32inttype;
                        end;
=======
                      if classes[i]=X86_64_INTEGER_CLASS then
                        paraloc^.size:=OS_64
                      else if result.intsize in [1,2,4] then
                        paraloc^.size:=retcgsize
                      else
                        paraloc^.size:=OS_32;
>>>>>>> graemeg/cpstrnew
                      setsubreg(paraloc^.register,cgsize2subreg(R_INTREGISTER,paraloc^.size));
                      inc(intretregidx);
                    end;
                  X86_64_SSE_CLASS,
                  X86_64_SSEUP_CLASS,
                  X86_64_SSESF_CLASS,
                  X86_64_SSEDF_CLASS:
                    begin
                      paraloc^.loc:=LOC_MMREGISTER;
                      paraloc^.register:=mmretregs[mmretregidx];
<<<<<<< HEAD
                      case classes[i].typ of
=======
                      case classes[i] of
>>>>>>> graemeg/cpstrnew
                        X86_64_SSESF_CLASS:
                          begin
                            setsubreg(paraloc^.register,R_SUBMMS);
                            paraloc^.size:=OS_F32;
                          end;
                        X86_64_SSEDF_CLASS:
                          begin
                            setsubreg(paraloc^.register,R_SUBMMD);
                            paraloc^.size:=OS_F64;
                          end;
                        else
                          begin
<<<<<<< HEAD
                            setsubreg(paraloc^.register,R_SUBQ);
=======
                            setsubreg(paraloc^.register,R_SUBMMWHOLE);
>>>>>>> graemeg/cpstrnew
                            paraloc^.size:=OS_M64;
                          end;
                      end;
                      inc(mmretregidx);
<<<<<<< HEAD
                    end;
                  X86_64_X87_CLASS:
                    begin
                      { must be followed by X86_64_X87UP_CLASS and that must be
                        the last class }
                      if (i<>(numclasses-2)) or
                         (classes[i+1].typ<>X86_64_X87UP_CLASS) then
                        internalerror(2014110401);
                      paraloc^.loc:=LOC_FPUREGISTER;
                      paraloc^.register:=NR_FPU_RESULT_REG;
                      paraloc^.size:=OS_F80;
                      break;
=======
>>>>>>> graemeg/cpstrnew
=======
                case classes[i] of
                  X86_64_INTEGERSI_CLASS,
                  X86_64_INTEGER_CLASS:
                    begin
                      paraloc^.loc:=LOC_REGISTER;
                      paraloc^.register:=intretregs[intretregidx];
=======
>>>>>>> graemeg/cpstrnew
=======
                case classes[i] of
                  X86_64_INTEGERSI_CLASS,
                  X86_64_INTEGER_CLASS:
                    begin
                      paraloc^.loc:=LOC_REGISTER;
                      paraloc^.register:=intretregs[intretregidx];
>>>>>>> origin/cpstrnew
                      if classes[i]=X86_64_INTEGER_CLASS then
                        paraloc^.size:=OS_64
                      else if result.intsize in [1,2,4] then
                        paraloc^.size:=retcgsize
                      else
                        paraloc^.size:=OS_32;
                      setsubreg(paraloc^.register,cgsize2subreg(R_INTREGISTER,paraloc^.size));
                      inc(intretregidx);
                    end;
                  X86_64_SSE_CLASS,
                  X86_64_SSEUP_CLASS,
                  X86_64_SSESF_CLASS,
                  X86_64_SSEDF_CLASS:
                    begin
                      paraloc^.loc:=LOC_MMREGISTER;
                      paraloc^.register:=mmretregs[mmretregidx];
                      case classes[i] of
                        X86_64_SSESF_CLASS:
                          begin
                            setsubreg(paraloc^.register,R_SUBMMS);
                            paraloc^.size:=OS_F32;
                          end;
                        X86_64_SSEDF_CLASS:
                          begin
                            setsubreg(paraloc^.register,R_SUBMMD);
                            paraloc^.size:=OS_F64;
                          end;
                        else
                          begin
                            setsubreg(paraloc^.register,R_SUBMMWHOLE);
                            paraloc^.size:=OS_M64;
                          end;
                      end;
                      inc(mmretregidx);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    end;
                  X86_64_NO_CLASS:
                    begin
                      { empty record/array }
                      if (i<>0) or
                         (numclasses<>1) then
                        internalerror(2010060302);
                      paraloc^.loc:=LOC_VOID;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      paraloc^.def:=voidtype;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    end;
                  else
                    internalerror(2010021504);
                end;
              end;
=======
            p.funcretloc[side].loc:=LOC_REGISTER;
            p.funcretloc[side].size:=retcgsize;
            if side=callerside then
              p.funcretloc[side].register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(retcgsize))
            else
              p.funcretloc[side].register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(retcgsize));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          end;
      end;


    procedure tcpuparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                                            var intparareg,mmparareg,parasize:longint;varargsparas: boolean);
      var
        hp         : tparavarsym;
        fdef,
        paradef    : tdef;
        paraloc    : pcgparalocation;
        subreg     : tsubregister;
        pushaddr   : boolean;
        paracgsize : tcgsize;
        loc        : array[1..2] of tx64paraclass;
        needintloc,
        needmmloc,
        paralen,
        locidx,
        i,
        varalign,
        paraalign  : longint;
      begin
        paraalign:=get_para_align(p.proccalloption);
        { Register parameters are assigned from left to right }
        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            paradef:=hp.vardef;
            { on win64, if a record has only one field and that field is a
              single or double, it has to be handled like a single/double }
            if (target_info.system=system_x86_64_win64) and
               ((paradef.typ=recorddef) {or
               is_object(paradef)}) and
               tabstractrecordsymtable(tabstractrecorddef(paradef).symtable).has_single_field(fdef) and
               (fdef.typ=floatdef) and
               (tfloatdef(fdef).floattype in [s32real,s64real]) then
              paradef:=fdef;

            pushaddr:=push_addr_param(hp.varspez,paradef,p.proccalloption);
            if pushaddr then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                loc[1].typ:=X86_64_INTEGER_CLASS;
                loc[2].typ:=X86_64_NO_CLASS;
=======
                loc[1]:=X86_64_INTEGER_CLASS;
                loc[2]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                loc[1]:=X86_64_INTEGER_CLASS;
                loc[2]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                loc[1]:=X86_64_INTEGER_CLASS;
                loc[2]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                loc[1]:=X86_64_INTEGER_CLASS;
                loc[2]:=X86_64_NO_CLASS;
>>>>>>> origin/cpstrnew
                paracgsize:=OS_ADDR;
                paralen:=sizeof(pint);
                paradef:=cpointerdef.getreusable_no_free(paradef);
                loc[1].def:=paradef;
              end
            else
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                getvalueparaloc(hp.varspez,paradef,loc[1],loc[2]);
                paralen:=push_size(hp.varspez,paradef,p.proccalloption);
                paracgsize:=def_cgsize(paradef);
=======
=======
>>>>>>> origin/fixes_2_2
                getvalueparaloc(hp.varspez,hp.vardef,loc[1],loc[2]);
                paralen:=push_size(hp.varspez,hp.vardef,p.proccalloption);
                paracgsize:=def_cgsize(hp.vardef);
>>>>>>> graemeg/fixes_2_2
              end;

            { cheat for now, we should copy the value to an mm reg as well (FK) }
            if varargsparas and
               (target_info.system = system_x86_64_win64) and
               (paradef.typ = floatdef) then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                loc[2].typ:=X86_64_NO_CLASS;
                if paracgsize=OS_F64 then
                  begin
                    loc[1].typ:=X86_64_INTEGER_CLASS;
                    paracgsize:=OS_64;
                    paradef:=u64inttype;
                  end
                else
                  begin
                    loc[1].typ:=X86_64_INTEGERSI_CLASS;
                    paracgsize:=OS_32;
                    paradef:=u32inttype;
                  end;
                loc[1].def:=paradef;
=======
                loc[2]:=X86_64_NO_CLASS;
                if paracgsize=OS_F64 then
                  begin
                    loc[1]:=X86_64_INTEGER_CLASS;
                    paracgsize:=OS_64
                  end
                else
                  begin
                    loc[1]:=X86_64_INTEGERSI_CLASS;
                    paracgsize:=OS_32;
                  end;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                loc[2]:=X86_64_NO_CLASS;
                if paracgsize=OS_F64 then
                  begin
                    loc[1]:=X86_64_INTEGER_CLASS;
                    paracgsize:=OS_64
                  end
                else
                  begin
                    loc[1]:=X86_64_INTEGERSI_CLASS;
                    paracgsize:=OS_32;
                  end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              end;

            hp.paraloc[side].reset;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].intsize:=paralen;
            hp.paraloc[side].Alignment:=paraalign;
            hp.paraloc[side].def:=paradef;
            if paralen>0 then
              begin
                { Enough registers free? }
                needintloc:=0;
                needmmloc:=0;
                for locidx:=low(loc) to high(loc) do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  case loc[locidx].typ of
=======
                  case loc[locidx] of
>>>>>>> graemeg/cpstrnew
=======
                  case loc[locidx] of
>>>>>>> graemeg/cpstrnew
=======
                  case loc[locidx] of
>>>>>>> graemeg/cpstrnew
=======
                  case loc[locidx] of
>>>>>>> origin/cpstrnew
                    X86_64_INTEGER_CLASS,
                    X86_64_INTEGERSI_CLASS:
                      inc(needintloc);
                    X86_64_SSE_CLASS,
                    X86_64_SSESF_CLASS,
                    X86_64_SSEDF_CLASS,
                    X86_64_SSEUP_CLASS:
                      inc(needmmloc);
                  end;
                { the "-1" is because we can also use the current register }
                if ((target_info.system=system_x86_64_win64) and
                    ((intparareg+needintloc-1 > high(paraintsupregs_winx64)) or
                     (mmparareg+needmmloc-1 > high(parammsupregs_winx64)))) or
                   ((target_info.system<>system_x86_64_win64) and
                    ((intparareg+needintloc-1 > high(paraintsupregs)) or
                     (mmparareg+needmmloc-1 > high(parammsupregs)))) then
                  begin
                    { If there are no registers available for any
                      eightbyte of an argument, the whole argument is
                      passed on the stack. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    loc[low(loc)].typ:=X86_64_MEMORY_CLASS;
                    loc[low(loc)].def:=paradef;
                    for locidx:=succ(low(loc)) to high(loc) do
                      loc[locidx].typ:=X86_64_NO_CLASS;
=======
                    loc[low(loc)]:=X86_64_MEMORY_CLASS;
                    for locidx:=succ(low(loc)) to high(loc) do
                      loc[locidx]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    loc[low(loc)]:=X86_64_MEMORY_CLASS;
                    for locidx:=succ(low(loc)) to high(loc) do
                      loc[locidx]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    loc[low(loc)]:=X86_64_MEMORY_CLASS;
                    for locidx:=succ(low(loc)) to high(loc) do
                      loc[locidx]:=X86_64_NO_CLASS;
>>>>>>> graemeg/cpstrnew
=======
                    loc[low(loc)]:=X86_64_MEMORY_CLASS;
                    for locidx:=succ(low(loc)) to high(loc) do
                      loc[locidx]:=X86_64_NO_CLASS;
>>>>>>> origin/cpstrnew
                  end;

                locidx:=1;
                while (paralen>0) and
                      (locidx<=2) and
                      (loc[locidx].typ<>X86_64_NO_CLASS) do
                  begin
<<<<<<< HEAD
                    { Allocate }
                    case loc[locidx].typ of
=======
                    if locidx>2 then
                      internalerror(200501283);
                    { Allocate }
                    case loc[locidx] of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                      X86_64_INTEGER_CLASS,
                      X86_64_INTEGERSI_CLASS:
                        begin
                          paraloc:=hp.paraloc[side].add_location;
                          paraloc^.loc:=LOC_REGISTER;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                          paraloc^.def:=loc[locidx].def;
                          if (paracgsize=OS_NO) or (loc[2].typ<>X86_64_NO_CLASS) then
                            begin
                              if loc[locidx].typ=X86_64_INTEGER_CLASS then
                                begin
                                  paraloc^.size:=OS_INT;
                                  paraloc^.def:=u64inttype;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                          if (paracgsize=OS_NO) or (loc[2]<>X86_64_NO_CLASS) then
                            begin
                              if loc[locidx]=X86_64_INTEGER_CLASS then
                                begin
                                  paraloc^.size:=OS_INT;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
                          if (paracgsize=OS_NO) or (loc[2]<>X86_64_NO_CLASS) then
                            begin
                              if loc[locidx]=X86_64_INTEGER_CLASS then
                                begin
                                  paraloc^.size:=OS_INT;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                                  subreg:=R_SUBWHOLE;
                                end
                              else
                                begin
                                  paraloc^.size:=OS_32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                  paraloc^.def:=u32inttype;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                                  subreg:=R_SUBD;
                                end;
                            end
                          else
                            begin
                              paraloc^.size:=def_cgsize(paraloc^.def);
                              { s64comp is pushed in an int register }
                              if paraloc^.size=OS_C64 then
                                begin
                                  paraloc^.size:=OS_64;
                                  paraloc^.def:=u64inttype;
                                end;
                              subreg:=cgsize2subreg(R_INTREGISTER,paraloc^.size);
                            end;

                          { winx64 uses different registers }
                          if target_info.system=system_x86_64_win64 then
                            paraloc^.register:=newreg(R_INTREGISTER,paraintsupregs_winx64[intparareg],subreg)
                          else
                            paraloc^.register:=newreg(R_INTREGISTER,paraintsupregs[intparareg],subreg);

                          { matching mm register must be skipped }
                          if target_info.system=system_x86_64_win64 then
                            inc(mmparareg);

                          inc(intparareg);
                          dec(paralen,tcgsize2size[paraloc^.size]);
                        end;
                      X86_64_SSE_CLASS,
                      X86_64_SSESF_CLASS,
                      X86_64_SSEDF_CLASS,
                      X86_64_SSEUP_CLASS:
                        begin
                          paraloc:=hp.paraloc[side].add_location;
                          paraloc^.loc:=LOC_MMREGISTER;
                          paraloc^.def:=loc[locidx].def;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                          case loc[locidx].typ of
=======
                          case loc[locidx] of
>>>>>>> graemeg/cpstrnew
=======
                          case loc[locidx] of
>>>>>>> graemeg/cpstrnew
=======
                          case loc[locidx] of
>>>>>>> graemeg/cpstrnew
=======
                          case loc[locidx] of
>>>>>>> origin/cpstrnew
                            X86_64_SSESF_CLASS:
                              begin
                                subreg:=R_SUBMMS;
                                paraloc^.size:=OS_F32;
                              end;
                            X86_64_SSEDF_CLASS:
                              begin
                                subreg:=R_SUBMMD;
                                paraloc^.size:=OS_F64;
                              end;
                            else
                              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                subreg:=R_SUBQ;
=======
                                subreg:=R_SUBMMWHOLE;
>>>>>>> graemeg/cpstrnew
=======
                                subreg:=R_SUBMMWHOLE;
>>>>>>> graemeg/cpstrnew
=======
                                subreg:=R_SUBMMWHOLE;
>>>>>>> graemeg/cpstrnew
=======
                                subreg:=R_SUBMMWHOLE;
>>>>>>> origin/cpstrnew
                                paraloc^.size:=OS_M64;
                              end;
                          end;

                          { winx64 uses different registers }
                          if target_info.system=system_x86_64_win64 then
                            paraloc^.register:=newreg(R_MMREGISTER,parammsupregs_winx64[mmparareg],subreg)
                          else
                            paraloc^.register:=newreg(R_MMREGISTER,parammsupregs[mmparareg],subreg);

                          { matching int register must be skipped }
                          if target_info.system=system_x86_64_win64 then
                            inc(intparareg);

                          inc(mmparareg);
                          dec(paralen,tcgsize2size[paraloc^.size]);
                        end;
                      X86_64_MEMORY_CLASS :
                        begin
                          paraloc:=hp.paraloc[side].add_location;
                          paraloc^.loc:=LOC_REFERENCE;
<<<<<<< HEAD
<<<<<<< HEAD
                          paraloc^.def:=loc[locidx].def;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                          {Hack alert!!! We should modify int_cgsize to handle OS_128,
                           however, since int_cgsize is called in many places in the
                           compiler where only a few can already handle OS_128, fixing it
                           properly is out of the question to release 2.2.0 in time. (DM)}
                          if paracgsize=OS_128 then
                            if paralen=8 then
                              paraloc^.size:=OS_64
                            else if paralen=16 then
                              paraloc^.size:=OS_128
                            else
                              internalerror(200707143)
                          else if paracgsize in [OS_F32,OS_F64,OS_F80,OS_F128] then
                            paraloc^.size:=int_float_cgsize(paralen)
                          else
                            paraloc^.size:=int_cgsize(paralen);
                          if side=callerside then
                            paraloc^.reference.index:=NR_STACK_POINTER_REG
                          else
                            paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                          varalign:=used_align(size_2_align(paralen),paraalign,paraalign);
                          paraloc^.reference.offset:=parasize;
                          parasize:=align(parasize+paralen,varalign);
                          paralen:=0;
                        end;
                      else
                        internalerror(2010053113);
                    end;
<<<<<<< HEAD
                    inc(locidx);
=======
                    if (locidx<2) and
                       (loc[locidx+1]<>X86_64_NO_CLASS) then
                      inc(locidx);
>>>>>>> graemeg/cpstrnew
                  end;
              end
            else
              begin
                paraloc:=hp.paraloc[side].add_location;
                paraloc^.loc:=LOC_VOID;
                paraloc^.def:=paradef;
              end;
          end;
        { Register parameters are assigned from left-to-right, but the
          offsets on the stack are right-to-left. There is no need
          to reverse the offset, only adapt the calleeside with the
          start offset of the first param on the stack }
        if side=calleeside then
          begin
            for i:=0 to paras.count-1 do
              begin
                hp:=tparavarsym(paras[i]);
                paraloc:=hp.paraloc[side].location;
                while paraloc<>nil do
                  begin
                    with paraloc^ do
                     if (loc=LOC_REFERENCE) then
                       inc(reference.offset,target_info.first_parm_offset);
                    paraloc:=paraloc^.next;
                  end;
              end;
          end;
      end;


    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;
      var
        intparareg,mmparareg,
        parasize : longint;
      begin
        intparareg:=0;
        mmparareg:=0;
        if target_info.system=system_x86_64_win64 then
          parasize:=4*8
        else
          parasize:=0;
        { calculate the registers for the normal parameters }
        create_paraloc_info_intern(p,callerside,p.paras,intparareg,mmparareg,parasize,false);
        { append the varargs }
        create_paraloc_info_intern(p,callerside,varargspara,intparareg,mmparareg,parasize,true);
        { store used no. of SSE registers, that needs to be passed in %AL }
        varargspara.mmregsused:=mmparareg;
        result:=parasize;
      end;


    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      var
        intparareg,mmparareg,
        parasize : longint;
      begin
        intparareg:=0;
        mmparareg:=0;
        if target_info.system=system_x86_64_win64 then
          parasize:=4*8
        else
          parasize:=0;
        create_paraloc_info_intern(p,side,p.paras,intparareg,mmparareg,parasize,false);
        { Create Function result paraloc }
        create_funcretloc_info(p,side);
        { We need to return the size allocated on the stack }
        result:=parasize;
      end;


begin
   paramanager:=tcpuparamanager.create;
end.
