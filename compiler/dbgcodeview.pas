{
    Copyright (c) 2018 by Nikolay Nikolov

    This units contains support for Microsoft CodeView debug info generation

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
{ documentation for the format, available on the internet:

  earlier versions:
    http://pagesperso-orange.fr/pierrelib/exec_formats/MS_Symbol_Type_v1.0.pdf
    http://ftp.openwatcom.org/devel/docs/CodeView.pdf

  modern versions:
    https://llvm.org/docs/PDB/index.html
    https://llvm.org/devmtg/2016-11/Slides/Kleckner-CodeViewInLLVM.pdf
}

unit dbgcodeview;

{$i fpcdefs.inc}

interface

    uses
      aasmdata,
      dbgbase;

    type
      TSymbolIndex = (
        S_COMPILE    = $0001, { Compile flags symbol }
        S_REGISTER   = $0002, { Register variable }
        S_CONSTANT   = $0003, { Constant symbol }
        S_UDT        = $0004, { User-defined Type }
        S_SSEARCH    = $0005, { Start search }
        S_END        = $0006, { End block, procedure, with, or thunk }
        S_SKIP       = $0007, { Skip - Reserve symbol space }
        S_CVRESERVE  = $0008, { Reserved for internal use by the Microsoft debugger }
        S_OBJNAME    = $0009, { Specify name of object file }
        S_ENDARG     = $000a, { Specify end of arguments in function symbols }
        S_COBOLUDT   = $000b, { Microfocus COBOL user-defined type }
        S_MANYREG    = $000c, { Many register symbol }
        S_RETURN     = $000d, { Function return description }
        S_ENTRYTHIS  = $000e, { Description of this pointer at entry }
        S_BPREL16    = $0100, { BP relative 16:16 }
        S_LDATA16    = $0101, { Local data 16:16 }
        S_GDATA16    = $0102, { Global data 16:16 }
        S_PUB16      = $0103, { Public symbol 16:16 }
        S_LPROC16    = $0104, { Local procedure start 16:16 }
        S_GPROC16    = $0105, { Global procedure start 16:16 }
        S_THUNK16    = $0106, { Thunk start 16:16 }
        S_BLOCK16    = $0107, { Block start 16:16 }
        S_WITH16     = $0108, { With start 16:16 }
        S_LABEL16    = $0109, { Code label 16:16 }
        S_CEXMODEL16 = $010a, { Change execution model 16:16 }
        S_VFTPATH16  = $010b, { Virtual function table path descriptor 16:16 }
        S_REGREL16   = $010c, { Specify 16:16 offset relative to arbitrary register }
        S_BPREL32    = $0200, { BP relative 16:32 }
        S_LDATA32    = $0201, { Local data 16:32 }
        S_GDATA32    = $0202, { Global data 16:32 }
        S_PUB32      = $0203, { Public symbol 16:32 }
        S_LPROC32    = $0204, { Local procedure start 16:32 }
        S_GPROC32    = $0205, { Global procedure start 16:32 }
        S_THUNK32    = $0206, { Thunk start 16:32 }
        S_BLOCK32    = $0207, { Block start 16:32 }
        S_VFTPATH32  = $020b, { Virtual function table path descriptor 16:32 }
        S_REGREL32   = $020c, { 16:32 offset relative to arbitrary register }
        S_LTHREAD32  = $020d, { Local Thread Storage data }
        S_GTHREAD32  = $020e, { Global Thread Storage data }
        S_LPROCMIPS  = $0300, { Local procedure start MIPS }
        S_GPROCMIPS  = $0301, { Global procedure start MIPS }
        S_PROCREF    = $0400, { Reference to a procedure }
        S_DATAREF    = $0401, { Reference to data }
        S_ALIGN      = $0402  { Page align symbols }
      );
      TLeafIndex=(
        LF_MODIFIER   = $0001, { Type Modifier (const, volatile, unaligned) }
        LF_POINTER    = $0002, { Pointer }
        LF_ARRAY      = $0003, { Simple Array }
        LF_CLASS      = $0004, { Class (C++ class declaration) }
        LF_STRUCTURE  = $0005, { Structure (C and C++ struct declaration) }
        LF_UNION      = $0006, { Union }
        LF_ENUM       = $0007, { Enumeration }
        LF_PROCEDURE  = $0008, { Procedure }
        LF_MFUNCTION  = $0009, { Member Function }
        LF_VTSHAPE    = $000a, { Virtual Function Table Shape }
        LF_COBOL0     = $000b, { reserved for Microfocus COBOL }
        LF_COBOL1     = $000c, { reserved for Microfocus COBOL }
        LF_BARRAY     = $000d, { Basic Array }
        LF_LABEL      = $000e, { Label }
        LF_NULL       = $000f, { Null }
        LF_NOTTRAN    = $0010, { Not Translated }
        LF_DIMARRAY   = $0011, { Multiply Dimensioned Array }
        LF_VFTPATH    = $0012, { Path to Virtual Function Table }
        LF_PRECOMP    = $0013, { Reference Precompiled Types }
        LF_ENDPRECOMP = $0014, { End of Precompiled Types }
        LF_OEM        = $0015, { OEM Generic Type }
        LF_Reserved   = $0016, { Reserved }

        LF_PAD0       = $f0,
        LF_PAD1       = $f1,
        LF_PAD2       = $f2,
        LF_PAD3       = $f3,
        LF_PAD4       = $f4,
        LF_PAD5       = $f5,
        LF_PAD6       = $f6,
        LF_PAD7       = $f7,
        LF_PAD8       = $f8,
        LF_PAD9       = $f9,
        LF_PAD10      = $fa,
        LF_PAD11      = $fb,
        LF_PAD12      = $fc,
        LF_PAD13      = $fd,
        LF_PAD14      = $fe,
        LF_PAD15      = $ff,

        LF_SKIP       = $0200, { Skip (used by incremental compilers to reserve space for future indexes) }
        LF_ARGLIST    = $0201, { Argument List }
        LF_DEFARG     = $0202, { Default Argument }
        LF_LIST       = $0203, { Arbitrary List }
        LF_FIELDLIST  = $0204, { Field List }
        LF_DERIVED    = $0205, { Derived Classes }
        LF_BITFIELD   = $0206, { Bit Fields }
        LF_METHODLIST = $0207, { Method List }
        LF_DIMCONU    = $0208, { Dimensioned Array with Constant Upper Bound }
        LF_DIMCONLU   = $0209, { Dimensioned Array with Constant Lower and Upper Bounds }
        LF_DIMVARU    = $020a, { Dimensioned Array with Variable Upper Bound }
        LF_DIMVARLU   = $020b, { Dimensioned Array with Variable Lower and Upper Bounds }
        LF_REFSYM     = $020c, { Referenced Symbol }

        LF_BCLASS     = $0400, { Real Base Class }
        LF_VBCLASS    = $0401, { Direct Virtual Base Class }
        LF_IVBCLASS   = $0402, { Indirect Virtual Base Class }
        LF_ENUMERATE  = $0403, { Enumeration Name and Value }
        LF_FRIENDFCN  = $0404, { Friend Function }
        LF_INDEX      = $0405, { Index to Another Type Record }
        LF_MEMBER     = $0406, { Data Member }
        LF_STMEMBER   = $0407, { Static Data Member }
        LF_METHOD     = $0408, { Method }
        LF_NESTTYPE   = $0409, { Nested Type Definition }
        LF_VFUNCTAB   = $040a, { Virtual Function Table Pointer }
        LF_FRIENDCLS  = $040b, { Friend Class }
        LF_ONEMETHOD  = $040c, { One Method }
        LF_VFUNCOFF   = $040d, { Virtual Function Offset }

        LF_CHAR       = $8000, { Signed Char (8-bit value) }
        LF_SHORT      = $8001, { Signed Short (16-bit signed value) }
        LF_USHORT     = $8002, { Unsigned Short (16-bit unsigned value) }
        LF_LONG       = $8003, { Signed Long (32-bit signed value) }
        LF_ULONG      = $8004, { Unsigned Long (32-bit unsigned value) }
        LF_REAL32     = $8005, { 32-bit Float }
        LF_REAL64     = $8006, { 64-bit Float }
        LF_REAL80     = $8007, { 80-bit Float }
        LF_REAL128    = $8008, { 128-bit Float }
        LF_QUADWORD   = $8009, { Signed Quad Word (64-bit signed value) }
        LF_UQUADWORD  = $800a, { Unsigned Quad Word (64-bit unsigned value) }
        LF_REAL48     = $800b, { 48-bit Float }
        LF_COMPLEX32  = $800c, { 32-bit Complex }
        LF_COMPLEX64  = $800d, { 64-bit Complex }
        LF_COMPLEX80  = $800e, { 80-bit Complex }
        LF_COMPLEX128 = $800f, { 128-bit Complex }
        LF_VARSTRING  = $8010  { Variable-length String }
      );

    const
      LF_NUMERIC = LF_CHAR;

    type

      { TDebugInfoCodeView }

      TDebugInfoCodeView = class(TDebugInfo)
      public
        procedure insertlineinfo(list:TAsmList);override;
      end;

    procedure InsertLineInfo_OMF_LINNUM_MsLink(list: TAsmList);

implementation

    uses
      globtype,
      cutils,
      aasmtai,
      fmodule,
      systems;

    procedure InsertLineInfo_OMF_LINNUM_MsLink(list: TAsmList);
      var
        currfileinfo,
        lastfileinfo : tfileposinfo;
        nolineinfolevel : Integer;
        currfuncname : pshortstring;
        hp : tai;
      begin
        FillChar(lastfileinfo,sizeof(lastfileinfo),0);
        hp:=Tai(list.first);
        nolineinfolevel:=0;
        while assigned(hp) do
          begin
            case hp.typ of
              ait_function_name :
                begin
                  currfuncname:=tai_function_name(hp).funcname;
                  list.concat(tai_comment.Create(strpnew('function: '+currfuncname^)));
                end;
              ait_force_line :
                begin
                  lastfileinfo.line:=-1;
                end;
              ait_marker :
                begin
                  case tai_marker(hp).kind of
                    mark_NoLineInfoStart:
                      inc(nolineinfolevel);
                    mark_NoLineInfoEnd:
                      dec(nolineinfolevel);
                    else
                      ;
                  end;
                end;
              else
                ;
            end;

            { OMF LINNUM records do not support multiple source files }
            if (hp.typ=ait_instruction) and
               (nolineinfolevel=0) and
               (tailineinfo(hp).fileinfo.fileindex=main_module.unit_index) then
              begin
                currfileinfo:=tailineinfo(hp).fileinfo;

                { line changed ? }
                if (lastfileinfo.line<>currfileinfo.line) and (currfileinfo.line<>0) then
                  begin
                    { line directive }
                    list.insertbefore(tai_directive.Create(asd_omf_linnum_line,tostr(currfileinfo.line)),hp);
                  end;
                lastfileinfo:=currfileinfo;
              end;

            hp:=tai(hp.next);
          end;
      end;


{****************************************************************************
                             TDebugInfoCodeView
****************************************************************************}

    procedure TDebugInfoCodeView.insertlineinfo(list: TAsmList);
      begin
        InsertLineInfo_OMF_LINNUM_MsLink(list);
      end;

{****************************************************************************
****************************************************************************}
    const
      dbg_codeview_info : tdbginfo =
         (
           id     : dbg_codeview;
           idtxt  : 'CODEVIEW';
         );

initialization
  RegisterDebugInfo(dbg_codeview_info,TDebugInfoCodeView);

end.

