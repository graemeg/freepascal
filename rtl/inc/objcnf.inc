{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by the Free Pascal development team

    This unit provides an interface to the Objective-C non-fragile
    run time (1.5+/2.x) as defined by Apple

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

interface

uses
  ctypes
{$ifdef unix}
  ,unixtype
{$endif}
  ;

{$packrecords c}

{$ifdef darwin}
const
  libname = 'objc';
  {$linkframework Foundation}
  {$define targetok}
{$endif}

{$ifndef targetok}
  {$error Add support for the current target to the objc1 unit }
{$endif}

const
  CLS_CLASS        = 0;
  CLS_META         = 1;
  CLS_ROOT         = 2;
  OBJC2_CLS_HIDDEN = $10;
  CLS_EXCEPTION    = $20;

type
  { make all opaque types assignment-incompatible with other typed pointers by
    declaring them as pointers to empty records

    WARNING: do NOT change the names, types or field names/types of these
      types, as many are used internally by the compiler.
  }

  { BOOL is one byte and uses 0/1, just like Pascal }
  BOOL = boolean;

  tobjc_class = record
  end;
  pobjc_class = ^tobjc_class;
  _Class = pobjc_class;

  id = ^objc_object;
  pobjc_object = id;

  _fpc_objc_sel_type = record
  end;
  SEL = ^_fpc_objc_sel_type;

  IMP = function(target: id; msg: SEL): id; varargs; cdecl;
  pIMP = ^IMP;

(*
  From Clang:
  // struct _class_t {
  //   struct _class_t *isa;
  //   struct _class_t * const superclass;
  //   void *cache;
  //   IMP *vtable;
  //   struct class_ro_t *ro;
  // }
  
  // Full definition required by the compiler, do not make opaque!
*)
  objc_object = record
    isa: pobjc_class;
    superclass: pobjc_class;
    cache: pointer;
    vtable: pIMP;
    ro: pointer;
  end;

  
  // Full definition required by the compiler, do not make opaque!
  objc_super = record
    receiver: id;
    _class: pobjc_class;
  end;
  pobjc_super = ^objc_super;

(* From Clang:
  // struct _protocol_t {
  //   id isa;  // NULL
  //   const char * const protocol_name;
  //   const struct _protocol_list_t * protocol_list; // super protocols
  //   const struct method_list_t * const instance_methods;
  //   const struct method_list_t * const class_methods;
  //   const struct method_list_t *optionalInstanceMethods;
  //   const struct method_list_t *optionalClassMethods;
  //   const struct _prop_list_t * properties;
  //   const uint32_t size;  // sizeof(struct _protocol_t)
  //   const uint32_t flags;  // = 0
  // }
  
  // Full definition required by the compiler, do not make opaque!
*)
  objc_protocol = record
    isa: id;
    protocol_name: pchar;
    protocol_list: pointer;
    instance_methods,
    class_methods,
    optionalInstanceMethods,
    optionalClassMethods: pointer;
    properties: pointer;
    size: cuint32;
    flags: cuint32;
  end;
  pobjc_protocol = ^objc_protocol;
  ppobjc_protocol = ^pobjc_protocol;

(* From Clang:
///  struct _ivar_t {
///   unsigned long int *offset;  // pointer to ivar offset location
///   char *name;
///   char *type;
///   uint32_t alignment;
///   uint32_t size;
  
  // Full definition required by the compiler, do not make opaque!
*)
  objc_ivar = record
    offset: pculong;
    name: pchar;
    ttype: pchar;
    alignment: cuint32;
    size: cuint32;
  end;
  Pobjc_ivar = ^objc_ivar;
  Ivar = Pobjc_ivar;
  PIvar = ^Ivar;

  // Full definition required by the compiler, do not make opaque!
  objc_method = record
    _cmd: SEL;
    method_type: pchar;
    _imp: pchar;
  end;
  Pobjc_method = ^objc_method;
  Method = Pobjc_method;
  PMethod = ^Method;

  { type that certainly will be returned by address }
  tdummyrecbyaddrresult = record
    a: array[0..1000] of shortstring;
  end;

  TEnumerationMutationHandler = procedure(obj: id); cdecl;

  ptrdiff_t = ptrint;

{ sending messages }
  function  objc_msgSend(self: id; op: SEL): id; cdecl; varargs; external libname;
  function  objc_msgSendSuper(const super: pobjc_super; op: SEL): id; cdecl; varargs; external libname;
  function  objc_msgSendSuper2(const super: pobjc_super; op: SEL): id; cdecl; varargs; weakexternal libname; { Mac OS X 10.6 and later }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the AArch64 ABI does not require special handling of struct returns, so no
  special handlers are provided/required }
{$ifndef cpuaarch64}
{ The following two are declared as procedures with the hidden result pointer
  as their first parameter. This corresponds to the declaration below as far
  as the code generator is concerned (and is easier to handle in the compiler).  }
  function  objc_msgSend_stret(self: id; op: SEL): tdummyrecbyaddrresult; cdecl; varargs; external libname;
  function  objc_msgSendSuper_stret(const super: pobjc_super; op: SEL): tdummyrecbyaddrresult; cdecl; varargs; external libname;
  function  objc_msgSendSuper2_stret(const super: pobjc_super; op: SEL): tdummyrecbyaddrresult; cdecl; varargs; weakexternal libname;
{$endif cpuaarch64}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { The following two are declared as procedures with the hidden result pointer
    as their first parameter. This corresponds to the declaration below as far
    as the code generator is concerned (and is easier to handle in the compiler).  }
  function  objc_msgSend_stret(self: id; op: SEL): tdummyrecbyaddrresult; cdecl; varargs; external libname;
  function  objc_msgSendSuper_stret(const super: pobjc_super; op: SEL): tdummyrecbyaddrresult; cdecl; varargs; external libname;
  function  objc_msgSendSuper2_stret(const super: pobjc_super; op: SEL): tdummyrecbyaddrresult; cdecl; varargs; weakexternal libname;
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
  { This one actually also exists to return extended on x86_64, but
    we don't support that yet
  }
{$ifdef cpui386}
  function  objc_msgSend_fpret (self: id; op: SEL): double; cdecl; varargs; external libname;
{$else cpui386}
  function  objc_msgSend_fpret (self: id; op: SEL): double; cdecl; varargs; external libname name 'objc_msgSend';
{$endif cpui386}

  function sel_getName(sel: SEL): PChar; cdecl; external libname;
  function sel_registerName(str: PChar): SEL; cdecl; external libname;
  function object_getClassName(obj: id): PChar; cdecl; external libname;
  function object_getIndexedIvars(obj: id ): Pointer; cdecl; external libname;

  function sel_getUid(const str: PChar): SEL; cdecl; external libname;

  function object_copy(obj:id; size:size_t):id; cdecl; external libname;
  function object_dispose(obj:id):id; cdecl; external libname;

  function object_getClass(obj:id): pobjc_class; cdecl; external libname;
  function object_setClass(obj:id; cls: pobjc_class):pobjc_class; cdecl; external libname;

  function object_getIvar(obj:id; _ivar:Ivar):id; cdecl; external libname;
  procedure object_setIvar(obj:id; _ivar:Ivar; value:id); cdecl; external libname;

  function object_setInstanceVariable(obj:id; name:pchar; value:pointer):Ivar; cdecl; external libname;
  function object_getInstanceVariable(obj:id; name:pchar; var outValue: Pointer):Ivar; cdecl; external libname;

  function objc_getClass(name:pchar):id; cdecl; external libname;
  function objc_getMetaClass(name:pchar):id; cdecl; external libname;
  function objc_lookUpClass(name:pchar):id; cdecl; external libname;
  function objc_getClassList(buffer:pClass; bufferCount:cint):cint; cdecl; external libname;

  function objc_getProtocol(name:pchar): pobjc_protocol; cdecl; external libname;
  function objc_copyProtocolList(outCount:pdword):ppobjc_protocol; cdecl; external libname;

  function class_getName(cls:pobjc_class):PChar; cdecl; external libname;
  function class_isMetaClass(cls:pobjc_class):BOOL; cdecl; external libname;
  function class_getSuperclass(cls:pobjc_class):pobjc_class; cdecl; external libname;

  function class_getVersion(cls:pobjc_class):longint; cdecl; external libname;
  procedure class_setVersion(cls:pobjc_class; version:longint); cdecl; external libname;

  function class_getInstanceSize(cls:pobjc_class):size_t; cdecl; external libname;

  function class_getInstanceVariable(cls:pobjc_class; name:pchar):Ivar; cdecl; external libname;
  function class_getClassVariable(cls:pobjc_class; name:pchar):Ivar; cdecl; external libname;
  function class_copyIvarList(cls:pobjc_class; outCount:pdword):PIvar; cdecl; external libname;

  function class_getInstanceMethod(cls:pobjc_class; name:SEL):Method; cdecl; external libname;
  function class_getClassMethod(cls:pobjc_class; name:SEL):Method; cdecl; external libname;
  function class_getMethodImplementation(cls:pobjc_class; name:SEL):IMP; cdecl; external libname;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef cpuaarch64}
  function class_getMethodImplementation_stret(cls:pobjc_class; name:SEL):IMP; cdecl; external libname;
{$endif cpuaarch64}
=======
  function class_getMethodImplementation_stret(cls:pobjc_class; name:SEL):IMP; cdecl; external libname;
>>>>>>> graemeg/cpstrnew
=======
  function class_getMethodImplementation_stret(cls:pobjc_class; name:SEL):IMP; cdecl; external libname;
>>>>>>> graemeg/cpstrnew
=======
  function class_getMethodImplementation_stret(cls:pobjc_class; name:SEL):IMP; cdecl; external libname;
>>>>>>> graemeg/cpstrnew
=======
  function class_getMethodImplementation_stret(cls:pobjc_class; name:SEL):IMP; cdecl; external libname;
>>>>>>> origin/cpstrnew
  function class_respondsToSelector(cls:pobjc_class; sel:SEL):BOOL; cdecl; external libname;
  function class_copyMethodList(cls:pobjc_class; outCount:pdword):PMethod; cdecl; external libname;

  function class_conformsToProtocol(cls:pobjc_class; var protocol: pobjc_protocol):BOOL; cdecl; external libname;
  function class_copyProtocolList(cls:pobjc_class; var outCount: dword):ppobjc_protocol; cdecl; external libname;

  function class_createInstance(cls:pobjc_class; extraBytes:size_t):id; cdecl; external libname;

  function objc_allocateClassPair(superclass:pobjc_class; name:pchar; extraBytes:size_t):pobjc_class; cdecl; external libname;
  procedure objc_registerClassPair(cls:pobjc_class); cdecl; external libname;
  function objc_duplicateClass(original:pobjc_class; name:pchar; extraBytes:size_t):pobjc_class; cdecl; external libname;
  procedure objc_disposeClassPair(cls:pobjc_class); cdecl; external libname;

  function class_addMethod(cls:pobjc_class; name:SEL; _imp:IMP; types:pchar):BOOL; cdecl; external libname;
  function class_addIvar(cls:pobjc_class; name:pchar; size:size_t; alignment:cuint8; types:pchar):BOOL; cdecl; external libname;
  function class_addProtocol(cls:pobjc_class; protocol:pobjc_protocol):BOOL; cdecl; external libname;

  function method_getName(m:Method):SEL; cdecl; external libname;
  function method_getImplementation(m:Method):IMP; cdecl; external libname;
  function method_getTypeEncoding(m:Method):Pchar; cdecl; external libname;

  function method_getNumberOfArguments(m:Method):dword; cdecl; external libname;
  function method_copyReturnType(m:Method):Pchar; cdecl; external libname;
  function method_copyArgumentType(m:Method; index:dword):Pchar; cdecl; external libname;
  procedure method_getReturnType(m:Method; dst:pchar; dst_len:size_t); cdecl; external libname;

  function method_setImplementation(m:Method; imp:IMP):IMP; cdecl; external libname;

  function ivar_getName(v:Ivar):Pchar; cdecl; external libname;
  function ivar_getTypeEncoding(v:Ivar):Pchar; cdecl; external libname;
  function ivar_getOffset(v:Ivar):ptrdiff_t; cdecl; external libname;

  function sel_isEqual(lhs:SEL; rhs:SEL):BOOL; cdecl; external libname;

  { fast enumeration support (available on Mac OS X 10.5 and later) }
  procedure objc_enumerationMutation(obj: id); cdecl; external libname;
  procedure objc_setEnumerationMutationHandler(handler: TEnumerationMutationHandler); cdecl; external libname;

implementation

end.
