{
    Copyright (c) 2010, 2013, 2015 by Jonas Maebe

    Basic Processor information for LLVM

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

{$i fpcdefs.inc}

Unit llvminfo;

Interface

uses
  globtype;

Type
   { possible supported processors for this target }
   tllvmversion =
      (llvmver_invalid,
       { Xcode versions use snapshots of LLVM and don't correspond to released
         versions of llvm (they don't ship with the llvm utilities either, but
         they do come with Clang, which can be used instead of opt/llc) }
       llvmver_xc_10_0,
       llvmver_xc_10_1,
       llvmver_7_0,
       llvmver_7_1,
       llvmver_8_0,
       llvmver_9_0,
       llvmver_10_0
      );

type
   tllvmversionflag = (
     llvmflag_memcpy_indiv_align,           { memcpy intrinsic supports separate alignment for source and dest }
     llvmflag_null_pointer_valid,           { supports "llvmflag_null_pointer_valid" attribute, which indicates access to nil should not be optimized as undefined behaviour }
     llvmflag_constrained_fptrunc_fpext,    { supports constrained fptrunc and fpext intrinsics }
     llvmflag_constrained_fptoi_itofp,      { supports constrained fptosi/fptoui/uitofp/sitofp instrinsics }
     llvmflag_generic_constrained_si64tofp  { supports sitofp for 64 bit signed integers on all targets }
   );
   tllvmversionflags = set of tllvmversionflag;

Const
   llvmversionstr : array[tllvmversion] of string[14] = (
     '',
     'Xcode-10.0',
     'Xcode-10.1',
     '7.0',
     '7.1',
     '8.0',
     '9.0',
     '10.0'
   );

   llvmversion_properties: array[tllvmversion] of tllvmversionflags =
     (
       { invalid         } [],
       { llvmver_xc_10_0 } [],
       { llvmver_xc_10_1 } [],
       { llvmver_7_0     } [llvmflag_memcpy_indiv_align,llvmflag_null_pointer_valid],
       { llvmver_7_1     } [llvmflag_memcpy_indiv_align,llvmflag_null_pointer_valid],
       { llvmver_8_0     } [llvmflag_memcpy_indiv_align,llvmflag_null_pointer_valid],
       { llvmver_9_0     } [llvmflag_memcpy_indiv_align,llvmflag_null_pointer_valid,llvmflag_constrained_fptrunc_fpext],
       { llvmver_10_0    } [llvmflag_memcpy_indiv_align,llvmflag_null_pointer_valid,llvmflag_constrained_fptrunc_fpext,llvmflag_constrained_fptoi_itofp]
     );

   { Supported optimizations, only used for information }
   supported_optimizerswitches = genericlevel1optimizerswitches+
                                 genericlevel2optimizerswitches+
                                 genericlevel3optimizerswitches-
                                 { no need to write info about those }
                                 [cs_opt_level1,cs_opt_level2,cs_opt_level3]+
                                 [cs_opt_loopunroll,cs_opt_stackframe,
				  cs_opt_nodecse,cs_opt_reorder_fields,cs_opt_fastmath];

   level1optimizerswitches = genericlevel1optimizerswitches;
   level2optimizerswitches = genericlevel2optimizerswitches + level1optimizerswitches + [cs_opt_nodecse,cs_opt_stackframe];
   level3optimizerswitches = genericlevel3optimizerswitches + level2optimizerswitches + [];
   level4optimizerswitches = genericlevel4optimizerswitches + level3optimizerswitches + [];

   function llvmversion2enum(const s: string): tllvmversion;

Implementation

  function llvmversion2enum(const s: string): tllvmversion;
    begin
      for result:=succ(low(llvmversionstr)) to high(llvmversionstr) do
        begin
          if s=llvmversionstr[result] then
            exit;
        end;
      result:=llvmver_invalid;
    end;

end.
