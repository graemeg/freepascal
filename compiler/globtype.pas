{
    Copyright (c) 1998-2002 by Florian Klaempfl, Pierre Muller

    Global types

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
unit globtype;

{$i fpcdefs.inc}

interface

    const
       maxidlen = 127;

    type
       { TCmdStr is used to pass command line parameters to an external program to be
         executed from the FPC application. In some circomstances, this can be more
         than 255 characters. That's why using Ansi Strings}
       TCmdStr = AnsiString;
       TPathStr = AnsiString;

{$ifdef symansistr}
       TSymStr = AnsiString;
{$else symansistr}
       TSymStr = ShortString;
{$endif symansistr}
       PSymStr = ^TSymStr;

       Int32 = Longint;

       { Integer type corresponding to pointer size }
{$ifdef cpu64bitaddr}
       PUint = qword;
       PInt = int64;
{$endif cpu64bitaddr}
{$ifdef cpu32bitaddr}
       PUint = cardinal;
       PInt = longint;
{$endif cpu32bitaddr}
{$ifdef cpu16bitaddr}
       PUint = word;
       PInt = Smallint;
{$endif cpu16bitaddr}

       { Natural integer register type and size for the target machine }
{$ifdef cpu64bitalu}
       AWord = qword;
       AInt = Int64;

     Const
       AIntBits = 64;
{$endif cpu64bitalu}
{$ifdef cpu32bitalu}
       AWord = longword;
       AInt = longint;

     Const
       AIntBits = 32;
{$endif cpu32bitalu}
{$ifdef cpu16bitalu}
       AWord = Word;
       AInt = Smallint;

     Const
       AIntBits = 16;
{$endif cpu16bitalu}
{$ifdef cpu8bitalu}
       AWord = Byte;
       AInt = Shortint;

     Const
       AIntBits = 8;
{$endif cpu8bitalu}

     { Maximum possible size of locals space (stack frame) }
     Const
{$if defined(cpu16bitaddr)}
       MaxLocalsSize = High(PUint);
{$else}
       MaxLocalsSize = High(longint) - 15;
{$endif}

     Type
       PAWord = ^AWord;
       PAInt = ^AInt;

       { target cpu specific type used to store data sizes }
{$ifdef cpu16bitaddr}
       { on small CPUs such as i8086, we use LongInt to support data structures
         larger than 32767 bytes and up to 65535 bytes in size. Since asizeint
         must be signed, we use LongInt/LongWord. }
       ASizeInt = LongInt;
       ASizeUInt = LongWord;
{$else cpu16bitaddr}
       ASizeInt = PInt;
       ASizeUInt = PUInt;
{$endif cpu16bitaddr}

       { type used for handling constants etc. in the code generator }
       TCGInt = Int64;

       { This must be an ordinal type with the same size as a pointer
         Note: Must be unsigned! Otherwise, ugly code like
         pointer(-1) will result in a pointer with the value
         $fffffffffffffff on a 32bit machine if the compiler uses
         int64 constants internally (JM) }
{$ifdef i8086}
       TConstPtrUInt = LongWord;  { 32-bit for far pointers support }
{$else i8086}
       TConstPtrUInt = PUint;
{$endif i8086}

       { Use a variant record to be sure that the array if aligned correctly }
       tcompdoublerec=record
         case byte of
           0 : (bytes:array[0..7] of byte);
           1 : (value:double);
       end;
       { Use a variant record to be sure that the array if aligned correctly }
       tcompsinglerec=record
         case byte of
           0 : (bytes:array[0..3] of byte);
           1 : (value:single);
       end;
       tcompextendedrec=record
         case byte of
           0 : (bytes:array[0..9] of byte);
           1 : (value:extended);
       end;

       pconstset = ^tconstset;
       tconstset = set of 0..255;

       { Switches which can be changed locally }
       tlocalswitch = (cs_localnone,
         { codegen }
         cs_check_overflow,cs_check_range,cs_check_object,
         cs_check_io,cs_check_stack,
         cs_checkpointer,cs_check_ordinal_size,
         cs_generate_stackframes,cs_do_assertion,cs_generate_rtti,
         cs_full_boolean_eval,cs_typed_const_writable,cs_allow_enum_calc,
         cs_do_inline,cs_fpu_fwait,cs_ieee_errors,
         cs_check_low_addr_load,cs_imported_data,
         cs_excessprecision,cs_check_fpu_exceptions,
         cs_check_all_case_coverage,
         { mmx }
         cs_mmx,cs_mmx_saturation,
         { parser }
         cs_typed_addresses,cs_strict_var_strings,cs_refcountedstrings,
         cs_bitpacking,cs_varpropsetter,cs_scopedenums,cs_pointermath,
         { macpas specific}
         cs_external_var, cs_externally_visible,
         { jvm specific }
         cs_check_var_copyout,
         cs_zerobasedstrings,
         { i8086 specific }
         cs_force_far_calls,
         cs_hugeptr_arithmetic_normalization,
         cs_hugeptr_comparison_normalization
       );
       tlocalswitches = set of tlocalswitch;

       { Switches which can be changed only at the beginning of a new module }
       tmoduleswitch = (cs_modulenone,
         { parser }
         cs_fp_emulation,cs_extsyntax,cs_openstring,
         { support }
         cs_support_goto,cs_support_macro,
         cs_support_c_operators,
         { generation }
         cs_profile,cs_debuginfo,cs_compilesystem,
         cs_lineinfo,cs_implicit_exceptions,
         cs_explicit_codepage,cs_system_codepage,
         { linking }
         cs_create_smart,cs_create_dynamic,cs_create_pic,
         { browser switches are back }
         cs_browser,cs_local_browser,
         { target specific }
         cs_executable_stack,
         { i8086 specific }
         cs_huge_code,
         cs_win16_smartcallbacks,
         { Record usage of checkpointer experimental feature }
         cs_checkpointer_called,
         { enable link time optimisation (both unit code generation and optimising the whole program/library) }
         cs_lto
       );
       tmoduleswitches = set of tmoduleswitch;

       { Switches which can be changed only for a whole program/compilation,
         mostly set with commandline }
       tglobalswitch = (cs_globalnone,
         { parameter switches }
         cs_check_unit_name,cs_constructor_name,cs_support_exceptions,
         cs_support_c_objectivepas,
         cs_transparent_file_names,
         { units }
         cs_load_objpas_unit,
         cs_load_gpc_unit,
         cs_load_fpcylix_unit,
         cs_support_vectors,
         { debuginfo }
         cs_use_heaptrc,cs_use_lineinfo,
         cs_gdb_valgrind,cs_no_regalloc,cs_stabs_preservecase,
         { assembling }
         cs_asm_leave,cs_asm_extern,cs_asm_pipe,cs_asm_source,
         cs_asm_regalloc,cs_asm_tempalloc,cs_asm_nodes,cs_asm_pre_binutils_2_25,
         { linking }
         cs_link_nolink,cs_link_static,cs_link_smart,cs_link_shared,cs_link_deffile,
         cs_link_strip,cs_link_staticflag,cs_link_on_target,cs_link_extern,cs_link_opt_vtable,
         cs_link_opt_used_sections,cs_link_separate_dbg_file,
         cs_link_map,cs_link_pthread,cs_link_no_default_lib_order,
         cs_link_native,
         cs_link_pre_binutils_2_19,
         cs_link_vlink,
         { disable LTO for the system unit (needed to work around linker bugs on macOS) }
         cs_lto_nosystem,
         cs_assemble_on_target,
         { use a memory model which allows large data structures, e.g. > 2 GB static data on x86-64 targets
           this not supported on all OSes }
         cs_large
       );
       tglobalswitches = set of tglobalswitch;

       { global switches specific to debug information }
       tdebugswitch = (ds_none,
          { enable set support in dwarf debug info, breaks gdb versions }
          { without support for that tag (they refuse to parse the rest }
          { of the debug information)                                   }
          ds_dwarf_sets,
          { use absolute paths for include files in stabs. Pro: gdb     }
          { always knows full path to file. Con: doesn't work anymore   }
          { if the include file is moved (otherwise, things still work  }
          { if your source hierarchy is the same, but has a different   }
          { base path)                                                  }
          ds_stabs_abs_include_files,
          { prefix method names by "classname__" in DWARF (like is done }
          { for Stabs); not enabled by default, because otherwise once  }
          { support for calling methods has been added to gdb, you'd    }
          { always have to type classinstance.classname__methodname()   }
          ds_dwarf_method_class_prefix,
          { Simulate C++ debug information in DWARF. It can be used for }
          { debuggers, which do not support Pascal.                     }
          ds_dwarf_cpp,
          { emit line number information in LINNUM/LINNUM32 records,    }
          { using the MS LINK format, for targets that use the OMF      }
          { object format. This option is useful for compatibility with }
          { the Open Watcom Debugger and the Open Watcom Linker. Even   }
          { though, they support and use dwarf debug information in the }
          { final executable file, they expect LINNUM records in the    }
          { object modules for the line number information.             }
          ds_dwarf_omf_linnum
       );
       tdebugswitches = set of tdebugswitch;

       { global target-specific switches }
       ttargetswitch = (ts_none,
         { generate code that results in smaller TOCs than normal (AIX) }
         ts_small_toc,
         { for the JVM target: generate integer array initializations via string
           constants in order to reduce the generated code size (Java routines
           are limited to 64kb of bytecode) }
         ts_compact_int_array_init,
         { for the JVM target: intialize enum fields in constructors with the
           enum class instance corresponding to ordinal value 0 (not done by
           default because this initialization can only be performed after the
           inherited constructors have run, and if they call a virtual method
           of the current class, then this virtual method may already have
           initialized that field with another value and the constructor
           initialization will result in data loss }
         ts_jvm_enum_field_init,
         { when automatically generating getters/setters for properties, use
           these strings as prefixes for the generated getters/setter names }
         ts_auto_getter_prefix,
         ts_auto_setter_predix,
         ts_thumb_interworking,
         { lowercase the first character of routine names, used to generate
           names that are compliant with Java coding standards from code
           written according to Delphi coding standards }
         ts_lowercase_proc_start,
         { initialise local variables on the JVM target so you won't get
           accidental uses of uninitialised values }
         ts_init_locals,
         { emit a CLD instruction before using the x86 string instructions }
         ts_cld,
         { increment BP before pushing it in the function prologue and decrement
           it after popping it in the function epilogue, iff the function is
           going to terminate with a far ret. Thus, the BP value pushed on the
           stack becomes odd if the function is far and even if the function is
           near. This allows walking the BP chain on the stack and e.g.
           obtaining a stack trace even if the program uses a mixture of near
           and far calls. This is also required for Win16 real mode, because it
           allows Windows to move code segments around (in order to defragment
           memory) and then walk through the stacks of all running programs and
           update the segment values of the segment that has moved. }
         ts_x86_far_procs_push_odd_bp
       );
       ttargetswitches = set of ttargetswitch;


       { adding a new entry here requires also adding the appropriate define in
         systemh.inc (FK)
       }
       tfeature = (
         f_heap,f_init_final,f_rtti,f_classes,f_exceptions,f_exitcode,
         f_ansistrings,f_widestrings,f_textio,f_consoleio,f_fileio,
         f_random,f_variants,f_objects,f_dynarrays,f_threading,f_commandargs,
         f_processes,f_stackcheck,f_dynlibs,f_softfpu,f_objectivec1,f_resources,
         f_unicodestring
       );
       tfeatures = set of tfeature;

     type
       { optimizer }
       toptimizerswitch = (cs_opt_none,
         cs_opt_level1,cs_opt_level2,cs_opt_level3,cs_opt_level4,
         cs_opt_regvar,cs_opt_uncertain,cs_opt_size,cs_opt_stackframe,
         cs_opt_peephole,cs_opt_loopunroll,cs_opt_tailrecursion,cs_opt_nodecse,
         cs_opt_nodedfa,cs_opt_loopstrength,cs_opt_scheduler,cs_opt_autoinline,cs_useebp,cs_userbp,
         cs_opt_reorder_fields,cs_opt_fastmath,
         { Allow removing expressions whose result is not used, even when this
           can change program behaviour (range check errors disappear,
           access violations due to invalid pointer derefences disappear, ...).
           Note: it does not (and must not) remove expressions that have
             explicit side-effects, only implicit side-effects (like the ones
             mentioned before) can disappear.
         }
         cs_opt_dead_values,
         { compiler checks for empty procedures/methods and removes calls to them if possible }
         cs_opt_remove_empty_proc,
         cs_opt_constant_propagate,
         cs_opt_dead_store_eliminate,
         cs_opt_forcenostackframe,
         cs_opt_use_load_modify_store,
         cs_opt_unused_para
       );
       toptimizerswitches = set of toptimizerswitch;

       { whole program optimizer }
       twpoptimizerswitch = (
         cs_wpo_devirtualize_calls,cs_wpo_optimize_vmts,
         cs_wpo_symbol_liveness
       );
       twpoptimizerswitches = set of twpoptimizerswitch;

       { platform triplet style }
       ttripletstyle = (
         triplet_llvm
         { , triple_gnu }
       );

       { module flags (extra unit flags not in ppu header) }
       tmoduleflag = (
         mf_init,                     { unit has initialization section }
         mf_finalize,                 { unit has finalization section   }
         mf_checkpointer_called,      { Unit uses experimental checkpointer test code }
         mf_has_resourcestrings,      { unit has resource string section }
         mf_release,                  { unit was compiled with -Ur option }
         mf_threadvars,               { unit has threadvars }
         mf_has_stabs_debuginfo,      { this unit has stabs debuginfo generated }
         mf_local_symtable,           { this unit has a local symtable stored }
         mf_uses_variants,            { this unit uses variants }
         mf_has_resourcefiles,        { this unit has external resources (using $R directive)}
         mf_has_exports,              { this module or a used unit has exports }
         mf_has_dwarf_debuginfo,      { this unit has dwarf debuginfo generated }
         mf_wideinits,                { this unit has winlike widestring typed constants }
         mf_classinits,               { this unit has class constructors/destructors }
         mf_resstrinits,              { this unit has string consts referencing resourcestrings }
         mf_i8086_far_code,           { this unit uses an i8086 memory model with far code (i.e. medium, large or huge) }
         mf_i8086_far_data,           { this unit uses an i8086 memory model with far data (i.e. compact or large) }
         mf_i8086_huge_data,          { this unit uses an i8086 memory model with huge data (i.e. huge) }
         mf_i8086_cs_equals_ds,       { this unit uses an i8086 memory model with CS=DS (i.e. tiny) }
         mf_i8086_ss_equals_ds,       { this unit uses an i8086 memory model with SS=DS (i.e. tiny, small or medium) }
         mf_package_deny,             { this unit must not be part of a package }
         mf_package_weak,             { this unit may be completely contained in a package }
         mf_llvm,                     { compiled for LLVM code generator, not compatible with regular compiler because of different nodes in inline functions }
         mf_symansistr                { symbols are ansistrings (for ppudump) }
       );
       tmoduleflags = set of tmoduleflag;

    type
       ttargetswitchinfo = record
          name: string[22];
          { target switch can have an arbitratry value, not only on/off }
          hasvalue: boolean;
          { target switch can be used only globally }
          isglobal: boolean;
          define: string[25];
       end;

    const
       OptimizerSwitchStr : array[toptimizerswitch] of string[18] = ('',
         'LEVEL1','LEVEL2','LEVEL3','LEVEL4',
         'REGVAR','UNCERTAIN','SIZE','STACKFRAME',
         'PEEPHOLE','LOOPUNROLL','TAILREC','CSE',
         'DFA','STRENGTH','SCHEDULE','AUTOINLINE','USEEBP','USERBP',
         'ORDERFIELDS','FASTMATH','DEADVALUES','REMOVEEMPTYPROCS',
         'CONSTPROP',
         'DEADSTORE','FORCENOSTACKFRAME','USELOADMODIFYSTORE',
         'UNUSEDPARA'
       );
       WPOptimizerSwitchStr : array [twpoptimizerswitch] of string[14] = (
         'DEVIRTCALLS','OPTVMTS','SYMBOLLIVENESS'
       );

       DebugSwitchStr : array[tdebugswitch] of string[22] = ('',
         'DWARFSETS','STABSABSINCLUDES','DWARFMETHODCLASSPREFIX','DWARFCPP','DWARFOMFLINNUM');

       TargetSwitchStr : array[ttargetswitch] of ttargetswitchinfo = (
         (name: '';                    hasvalue: false; isglobal: true ; define: ''),
         (name: 'SMALLTOC';            hasvalue: false; isglobal: true ; define: ''),
         (name: 'COMPACTINTARRAYINIT'; hasvalue: false; isglobal: true ; define: ''),
         (name: 'ENUMFIELDINIT';       hasvalue: false; isglobal: true ; define: ''),
         (name: 'AUTOGETTERPREFIX';    hasvalue: true ; isglobal: false; define: ''),
         (name: 'AUTOSETTERPREFIX';    hasvalue: true ; isglobal: false; define: ''),
         (name: 'THUMBINTERWORKING';   hasvalue: false; isglobal: true ; define: ''),
         (name: 'LOWERCASEPROCSTART';  hasvalue: false; isglobal: true ; define: ''),
         (name: 'INITLOCALS';          hasvalue: false; isglobal: true ; define: ''),
         (name: 'CLD';                 hasvalue: false; isglobal: true ; define: 'FPC_ENABLED_CLD'),
         (name: 'FARPROCSPUSHODDBP';   hasvalue: false; isglobal: false; define: 'FPC_FAR_PROCS_PUSH_ODD_BP')
       );

       { switches being applied to all CPUs at the given level }
       genericlevel1optimizerswitches = [cs_opt_level1,cs_opt_peephole];
       genericlevel2optimizerswitches = [cs_opt_level2,cs_opt_remove_empty_proc,cs_opt_unused_para];
       genericlevel3optimizerswitches = [cs_opt_level3,cs_opt_constant_propagate,cs_opt_nodedfa{$ifndef llvm},cs_opt_use_load_modify_store{$endif},cs_opt_loopunroll];
       genericlevel4optimizerswitches = [cs_opt_level4,cs_opt_reorder_fields,cs_opt_dead_values,cs_opt_fastmath];

       { whole program optimizations whose information generation requires
         information from all loaded units
       }
       WPOptimizationsNeedingAllUnitInfo = [cs_wpo_devirtualize_calls,cs_wpo_optimize_vmts];

       featurestr : array[tfeature] of string[14] = (
         'HEAP','INITFINAL','RTTI','CLASSES','EXCEPTIONS','EXITCODE',
         'ANSISTRINGS','WIDESTRINGS','TEXTIO','CONSOLEIO','FILEIO',
         'RANDOM','VARIANTS','OBJECTS','DYNARRAYS','THREADING','COMMANDARGS',
         'PROCESSES','STACKCHECK','DYNLIBS','SOFTFPU','OBJECTIVEC1','RESOURCES',
         'UNICODESTRINGS'
       );

    type
       { Switches which can be changed by a mode (fpc,tp7,delphi) }
       tmodeswitch = (m_none,
         { generic }
         m_fpc,m_objfpc,m_delphi,m_tp7,m_mac,m_iso,m_extpas,
         {$ifdef gpc_mode}m_gpc,{$endif}
         { more specific }
         m_class,               { delphi class model }
         m_objpas,              { load objpas unit }
         m_result,              { result in functions }
         m_string_pchar,        { pchar 2 string conversion }
         m_cvar_support,        { cvar variable directive }
         m_nested_comment,      { nested comments }
         m_tp_procvar,          { tp style procvars (no @ needed) }
         m_mac_procvar,         { macpas style procvars }
         m_repeat_forward,      { repeating forward declarations is needed }
         m_pointer_2_procedure, { allows the assignement of pointers to
                                  procedure variables                     }
         m_autoderef,           { does auto dereferencing of struct. vars }
         m_initfinal,           { initialization/finalization for units }
         m_default_ansistring,  { ansistring turned on by default }
         m_out,                 { support the calling convention OUT }
         m_default_para,        { support default parameters }
         m_hintdirective,       { support hint directives }
         m_duplicate_names,     { allow locals/paras to have duplicate names of globals }
         m_property,            { allow properties }
         m_default_inline,      { allow inline proc directive }
         m_except,              { allow exception-related keywords }
         m_objectivec1,         { support interfacing with Objective-C (1.0) }
         m_objectivec2,         { support interfacing with Objective-C (2.0) }
         m_nested_procvars,     { support nested procedural variables }
         m_non_local_goto,      { support non local gotos (like iso pascal) }
         m_advanced_records,    { advanced record syntax with visibility sections, methods and properties }
         m_isolike_unary_minus, { unary minus like in iso pascal: same precedence level as binary minus/plus }
         m_systemcodepage,      { use system codepage as compiler codepage by default, emit ansistrings with system codepage }
         m_final_fields,        { allows declaring fields as "final", which means they must be initialised
                                  in the (class) constructor and are constant from then on (same as final
                                  fields in Java) }
         m_default_unicodestring, { makes the default string type in $h+ mode unicodestring rather than
                                    ansistring; similarly, char becomes unicodechar rather than ansichar }
         m_type_helpers,        { allows the declaration of "type helper" for all supported types
                                  (primitive types, records, classes, interfaces) }
         m_blocks,              { support for http://en.wikipedia.org/wiki/Blocks_(C_language_extension) }
         m_isolike_io,          { I/O as it required by an ISO compatible compiler }
         m_isolike_program_para, { program parameters as it required by an ISO compatible compiler }
         m_isolike_mod,         { mod operation as it is required by an iso compatible compiler }
         m_array_operators,     { use Delphi compatible array operators instead of custom ones ("+") }
         m_multi_helpers,       { helpers can appear in multiple scopes simultaneously }
         m_array2dynarray,      { regular arrays can be implicitly converted to dynamic arrays }
         m_prefixed_attributes  { enable attributes that are defined before the type they belong to }
       );
       tmodeswitches = set of tmodeswitch;

    const
       alllanguagemodes = [m_fpc,m_objfpc,m_delphi,m_tp7,m_mac,m_iso,m_extpas];

    type
       { Application types (platform specific) }
       tapptype = (
         app_none,
         app_native,    { native for Windows and NativeNT targets }
         app_gui,       { graphic user-interface application }
         app_cui,       { console application }
         app_fs,        { full-screen type application (OS/2 and EMX only) }
         app_tool,      { tool application, (MPW tool for MacOS, MacOS only) }
         app_arm7,      { for Nintendo DS target }
         app_arm9,      { for Nintendo DS target }
         app_bundle,    { dynamically loadable bundle, Darwin only }
         app_com        { DOS .COM file }
       );

       { interface types }
       tinterfacetypes = (
         it_interfacecom,
         it_interfacecorba,
         it_interfacejava
       );

       { currently parsed block type }
       tblock_type = (
         bt_none,        { not assigned                              }
         bt_general,     { default                                   }
         bt_type,        { type section                              }
         bt_const,       { const section                             }
         bt_const_type,  { const part of type. e.g.: ": Integer = 1" }
         bt_var,         { variable declaration                      }
         bt_var_type,    { type of variable                          }
         bt_except,      { except section                            }
         bt_body         { procedure body                            }
       );

       { Temp types }
       ttemptype = (tt_none,
                    { free temp location, can be reused for something else }
                    tt_free,
                    { temp location that will be freed when ttgobj.UnGetTemp/
                      ttgobj.UnGetIfTemp is called on it }
                    tt_normal,
                    { temp location that will not be freed; if it has to be
                      freed, first ttgobj.changetemptype() it to tt_normal,
                      or call ttgobj.UnGetLocal() instead (for local variables,
                      since they are also persistent temps) }
                    tt_persistent,
                    { temp location that can never be reused anymore, even
                      after it has been freed }
                    tt_noreuse,
                    { freed version of the above }
                    tt_freenoreuse,
                    { temp location that has been allocated by the register
                      allocator and that can be reallocated only by the
                      register allocator }
                    tt_regallocator,
                    { freed version of the above }
                    tt_freeregallocator);
       ttemptypeset = set of ttemptype;

       { calling convention for tprocdef and tprocvardef }
       tproccalloption=(pocall_none,
         { procedure uses C styled calling }
         pocall_cdecl,
         { C++ calling conventions }
         pocall_cppdecl,
         { Far16 for OS/2 }
         pocall_far16,
         { Old style FPC default calling }
         pocall_oldfpccall,
         { Procedure has compiler magic}
         pocall_internproc,
         { procedure is a system call, applies e.g. to MorphOS and PalmOS }
         pocall_syscall,
         { pascal standard left to right }
         pocall_pascal,
         { procedure uses register (fastcall) calling }
         pocall_register,
         { safe call calling conventions }
         pocall_safecall,
         { procedure uses stdcall call }
         pocall_stdcall,
         { Special calling convention for cpus without a floating point
           unit. Floating point numbers are passed in integer registers
           instead of floating point registers. Depending on the other
           available calling conventions available for the cpu
           this replaces either pocall_fastcall or pocall_stdcall.
         }
         pocall_softfloat,
         { Metrowerks Pascal. Special case on Mac OS (X): passes all }
         { constant records by reference.                            }
         pocall_mwpascal,
         { Special interrupt handler for embedded systems }
         pocall_interrupt,
         { Directive for arm: pass floating point values in (v)float registers
           regardless of the actual calling conventions }
         pocall_hardfloat,
         { for x86-64: force sysv ABI (Pascal resp. C) }
         pocall_sysv_abi_default,
         pocall_sysv_abi_cdecl,
         { for x86-64: forces Microsoft ABI (Pascal resp. C) }
         pocall_ms_abi_default,
         pocall_ms_abi_cdecl,
         { for x86-64: Microsoft's "vectorcall" ABI }
         pocall_vectorcall
       );
       tproccalloptions = set of tproccalloption;

     const
       proccalloptionStr : array[tproccalloption] of string[16]=('',
           'CDecl',
           'CPPDecl',
           'Far16',
           'OldFPCCall',
           'InternProc',
           'SysCall',
           'Pascal',
           'Register',
           'SafeCall',
           'StdCall',
           'SoftFloat',
           'MWPascal',
           'Interrupt',
           'HardFloat',
           'SysV_ABI_Default',
           'SysV_ABI_CDecl',
           'MS_ABI_Default',
           'MS_ABI_CDecl',
           'VectorCall'
         );

       { Default calling convention }
{$if defined(i8086)}
       pocall_default = pocall_pascal;
{$elseif defined(i386) or defined(x86_64)}
       pocall_default = pocall_register;
{$elseif defined(m68k)}
       pocall_default = pocall_register;
{$else}
       pocall_default = pocall_stdcall;
{$endif}

       cstylearrayofconst = [pocall_cdecl,pocall_cppdecl,pocall_mwpascal,pocall_sysv_abi_cdecl,pocall_ms_abi_cdecl];

       modeswitchstr : array[tmodeswitch] of string[18] = ('',
         '','','','','','','',
         {$ifdef gpc_mode}'',{$endif}
         { more specific }
         'CLASS',
         'OBJPAS',
         'RESULT',
         'PCHARTOSTRING',
         'CVAR',
         'NESTEDCOMMENTS',
         'CLASSICPROCVARS',
         'MACPROCVARS',
         'REPEATFORWARD',
         'POINTERTOPROCVAR',
         'AUTODEREF',
         'INITFINAL',
         'ANSISTRINGS',
         'OUT',
         'DEFAULTPARAMETERS',
         'HINTDIRECTIVE',
         'DUPLICATELOCALS',
         'PROPERTIES',
         'ALLOWINLINE',
         'EXCEPTIONS',
         'OBJECTIVEC1',
         'OBJECTIVEC2',
         'NESTEDPROCVARS',
         'NONLOCALGOTO',
         'ADVANCEDRECORDS',
         'ISOUNARYMINUS',
         'SYSTEMCODEPAGE',
         'FINALFIELDS',
         'UNICODESTRINGS',
         'TYPEHELPERS',
         'CBLOCKS',
         'ISOIO',
         'ISOPROGRAMPARAS',
         'ISOMOD',
         'ARRAYOPERATORS',
         'MULTIHELPERS',
         'ARRAYTODYNARRAY',
         'PREFIXEDATTRIBUTES'
         );


     type
       tprocinfoflag=(
         { procedure has at least one assembler block }
         pi_has_assembler_block,
         { procedure does a call }
         pi_do_call,
         { procedure has a try statement = no register optimization }
         pi_uses_exceptions,
         { procedure is declared as @var(assembler), don't optimize}
         pi_is_assembler,
         { procedure contains data which needs to be finalized }
         pi_needs_implicit_finally,
         { procedure has the implicit try..finally generated }
         pi_has_implicit_finally,
         { procedure uses fpu}
         pi_uses_fpu,
         { procedure uses GOT for PIC code }
         pi_needs_got,
         { references var/proc/type/const in static symtable,
           i.e. not allowed for inlining from other units }
         pi_uses_static_symtable,
         { set if the procedure has to push parameters onto the stack }
         pi_has_stackparameter,
         { set if the procedure has at least one label }
         pi_has_label,
         { calls itself recursive }
         pi_is_recursive,
         { stack frame optimization not possible (only on x86 probably) }
         pi_needs_stackframe,
         { set if the procedure has at least one register saved on the stack }
         pi_has_saved_regs,
         { dfa was generated for this proc }
         pi_dfaavailable,
         { subroutine contains interprocedural used labels }
         pi_has_interproclabel,
         { subroutine has unwind info (win64) }
         pi_has_unwind_info,
         { subroutine contains interprocedural gotos }
         pi_has_global_goto,
         { subroutine contains inherited call }
         pi_has_inherited,
         { subroutine has nested exit }
         pi_has_nested_exit,
         { allocates memory on stack, so stack is unbalanced on exit }
         pi_has_stack_allocs,
         { set if the stack frame of the procedure is estimated }
         pi_estimatestacksize,
         { the routine calls a C-style varargs function }
         pi_calls_c_varargs,
         { the routine has an open array parameter,
           for i8086 cpu huge memory model,
           as this changes SP register it requires special handling
           to restore DS segment register  }
         pi_has_open_array_parameter,
         { subroutine uses threadvars }
         pi_uses_threadvar,
         { set if the procedure has generated data which shall go in an except table }
         pi_has_except_table_data,
         { subroutine needs to load and maintain a tls register }
         pi_needs_tls,
         { subroutine uses get_frame }
         pi_uses_get_frame,
         { x86 only: subroutine uses ymm registers, requires vzeroupper call }
         pi_uses_ymm
       );
       tprocinfoflags=set of tprocinfoflag;

       ttlsmodel = (tlsm_none,
         { elf tls model: works for all kind of code and thread vars }
         tlsm_global_dynamic,
         { elf tls model: works only if the thread vars are declared and used in the same module,
           regardless when the module is loaded }
         tlsm_local_dynamic,
         { elf tls model: works only if the thread vars are declared and used in modules and executables loaded at startup }
         tlsm_initial_exec,
         { elf tls model: works only if the thread vars are declared and used in the same executable }
         tlsm_local_exec
       );

    type
      { float types -- warning, this enum/order is used internally by the RTL
        as well in rtl/inc/real2str.inc }
      tfloattype = (
        s32real,s64real,s80real,sc80real { the C "long double" type on x86 },
        s64comp,s64currency,s128real
      );

    type
      { register allocator live range extension direction }
      TRADirection = (rad_forward, rad_backwards, rad_backwards_reinit);

    type
{$ifndef symansistr}
      TIDString = string[maxidlen];
{$else}
      TIDString = TSymStr;
{$endif}

      tnormalset = set of byte; { 256 elements set }
      pnormalset = ^tnormalset;

      pboolean   = ^boolean;
      pdouble    = ^double;
      pbyte      = ^byte;
      pword      = ^word;
      plongint   = ^longint;
      plongintarray = plongint;

      pfileposinfo = ^tfileposinfo;
      tfileposinfo = record
        { if types of column or fileindex are changed, modify tcompilerppufile.putposinfo }
        line      : longint;
        column    : word;
        fileindex : word;
        moduleindex : word;
      end;

  {$ifndef xFPC}
    type
      pguid = ^tguid;
      tguid = packed record
        D1: LongWord;
        D2: Word;
        D3: Word;
        D4: array[0..7] of Byte;
      end;
  {$endif}

       tstringencoding = Word;
       tcodepagestring = string[20];

    const
       { link options }
       link_none    = $0;
       link_always  = $1;
       link_static  = $2;
       link_smart   = $4;
       link_shared  = $8;
       link_lto     = $10;

    type
      { a message state }
      tmsgstate = (
        ms_on := 1,
        ms_off := 2,
        ms_error := 3,

        ms_on_global := $11,    // turn on output
        ms_off_global := $22,   // turn off output
        ms_error_global := $33  // cast to error
      );
    const
      { Mask for current value of message state }
      ms_local_mask = $0f;
      { Mask for global value of message state
        that needs to be restored when changing units }
      ms_global_mask = $f0;
      { Shift used to convert global to local message state }
      ms_shift = 4;

    type
      pmessagestaterecord = ^tmessagestaterecord;
      tmessagestaterecord = record
        next : pmessagestaterecord;
        value : longint;
        state : tmsgstate;
      end;

    type
      tx86memorymodel = (mm_tiny,mm_small,mm_medium,mm_compact,mm_large,mm_huge);
    const
      x86memorymodelstr : array[tx86memorymodel] of string[7]=(
        'TINY',
        'SMALL',
        'MEDIUM',
        'COMPACT',
        'LARGE',
        'HUGE');

  { hide Sysutils.ExecuteProcess in units using this one after SysUtils}
  const
    ExecuteProcess = 'Do not use' deprecated 'Use cfileutil.RequotedExecuteProcess instead, ExecuteProcess cannot deal with single quotes as used by Unix command lines';

implementation

end.
