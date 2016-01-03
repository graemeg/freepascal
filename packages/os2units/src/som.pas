{
    Copyright (c) 1994-1996 by International Business Machines Corporation
    Copyright (c) 1997 Antony T Curtis.
    Copyright (c) 2002-2005 by Yuri Prokushev (prokushev@freemail.ru)

    System Object Model Run-time library API (SOM.DLL)

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

Unit SOM;

Interface

{$mode objfpc}
{$warning This units doesn't work because FPC/2 doesn't implements external vars}
{$warning This code is alpha!}

//uses
//  SOMTypes;

var
{$warning support of external vars required}
  SOM_MajorVersion, SOM_MinorVersion    :Longint;       (*  SOM Version Numbers  *)
//³ 00070 ³ SOM_MajorVersion
//³ 00071 ³ SOM_MinorVersion
{$warning support of external vars required}
  SOM_MaxThreads                        :Longint; // ³ 00095 ³ SOM_MaxThreads   (*  SOM Thread Support   *)

type
  Flags                 =Longint;

type
  TSOMObject                         = Pointer;
  SOMClassType                          = Pointer;
  SOMMSingleInstanceType                = Pointer;
  SOMClassMgrType                       = Pointer;
  SOMClassPtr                           = ^SOMClassType;
  PSOMClass                             = ^SOMClassType;
  PSOMObject                          = ^TSOMObject;
  CORBAObjectType                       = TSOMObject;    (* in SOM, a CORBA object is a SOM object *)

  somToken              =Pointer;       (* Uninterpretted value   *)
  somId                 =^PChar;
  somIdPtr              =^somId;
  PsomToken             =^somToken;       (* Uninterpretted value   *)

  somMToken             =somToken;
  somDToken             =somToken;
  somMTokenPtr          =^somMToken;
  somDTokenPtr          =^somDToken;

type
  ImplId                =^PChar;
  RepositoryId          = PChar;
  AttributeDef_AttributeMode    = Cardinal;
  OperationDef_OperationMode = Longint;
  ParameterDef_ParameterMode    = Cardinal;

  somMethodPtr          =Pointer;
  somBooleanVector      =^Byte;
  somCtrlInfo           =somToken;

  somSharedMethodData   =somToken;
  somSharedMethodDataPtr=^somSharedMethodData;

  somClassInfoPtr       =^somClassInfo;
  somClassInfo          =somToken;


  Identifier            =PChar;         (* CORBA 7.5.1, p. 129 *)

  TypeCode              = pointer;

(* CORBA 5.7, p.89 *)
  any                   = record
    _type               : TypeCode;
    _value              : Pointer;
  end;

  NamedValue            =record
    name                : Identifier;
    argument            : any;
    len                 : Longint;
    arg_modes           : Flags;
  end;

(* -- Method/Data Tokens -- For locating methods and data members. *)

  somRdAppType          =LongInt;       (* method signature code -- see def below *)
  somFloatMap           =Array[0..13] of LongInt; (* float map -- see def below *)
  somFloatMapPtr        =^somFloatMapPtr;

  somMethodInfoStruct   =record
    callType            :somRdAppType;
    va_listSize         :Longint;
    float_map           :somFloatMapPtr;
  end;
  somMethodInfo         =somMethodInfoStruct;
  somMethodInfoPtr      =^somMethodInfo;

  somMethodDataStruct   =record
    id                  :somId;
    ctype               :Longint;               (* 0=static, 1=dynamic 2=nonstatic *)
    descriptor          :somId;                 (* for use with IR interfaces *)
    mToken              :somMToken;             (* NULL for dynamic methods *)
    method              :somMethodPtr;          (* depends on resolution context *)
    shared              :somSharedMethodDataPtr;
  end;
  somMethodData         =somMethodDataStruct;
  somMethodDataPtr      =^somMethodDataStruct;

  somMethodProc         =Procedure(somSelf:TSOMObject);
  somMethodProcPtr      =^somMethodProc;


(*---------------------------------------------------------------------
 * C++-style constructors are called initializers in SOM. Initializers
 * are methods that receive a pointer to a somCtrlStruct as an argument.
 *)

  somInitInfo           =record
    cls                 :SOMClassType;(* the class whose introduced data is to be initialized *)
    defaultInit         :somMethodProc;
    defaultCopyInit     :somMethodProc;
    defaultConstCopyInit:somMethodProc;
    defaultNCArgCopyInit:somMethodProc;
    dataOffset          :Longint;
    legacyInit          :somMethodProc;
  end;

  somDestructInfo       =record
    cls                 :SOMClassType;(* the class whose introduced data is to be destroyed *)
    defaultDestruct     :somMethodProc;
    dataOffset          :Longint;
    legacyUninit        :somMethodProc;
  end;

  somAssignInfo         =record
    cls                 :SOMClassType;(* the class whose introduced data is to be assigned *)
    defaultAssign       :somMethodProc;
    defaultConstAssign  :somMethodProc;
    defaultNCArgAssign  :somMethodProc;
    udaAssign           :somMethodProc;
    udaConstAssign      :somMethodProc;
    dataOffset          :Longint;
  end;

  _IDL_SEQUENCE_octet   = record
    _maximum            : Cardinal;
    _length             : Cardinal;
    _buffer             : ^Byte;
  end;
  ReferenceData         =_IDL_SEQUENCE_octet;

(*
 * A special info access structure pointed to by
 * the parentMtab entry of somCClassDataStructure.
 *)
   somTD_somRenewNoInitNoZeroThunk      =Procedure(var buf); cdecl;

  somInitInfoPtr        =^somInitInfo;

  somInitCtrlStruct     =record
    mask                :somBooleanVector;(* an array of booleans to control ancestor calls *)
    info                :somInitInfoPtr;  (* an array of structs *)
    infoSize            :Longint;         (* increment for info access *)
    ctrlInfo            :somCtrlInfo;
  end;
  somInitCtrl           =somInitCtrlStruct;
  som3InitCtrl          =somInitCtrlStruct;

  somDestructInfoPtr    =^somDestructInfo;
  somDestructCtrlStruct =record
    mask                :somBooleanVector;(* an array of booleans to control ancestor calls *)
    info                :somDestructInfoPtr;(* an array of structs *)
    infoSize            :Longint;         (* increment for info access *)
    ctrlInfo            :somCtrlInfo;
  end;
  somDestructCtrl       =somDestructCtrlStruct;
  som3DestructCtrl      =somDestructCtrlStruct;

  somAssignInfoPtr      =^somAssignInfo;
  somAssignCtrlStruct   =record
    mask                :somBooleanVector;(* an array of booleans to control ancestor calls *)
    info                :somAssignInfoPtr;(* an array of structs *)
    infoSize            :Longint;         (* increment for info access *)
    ctrlInfo            :somCtrlInfo;
  end;
  somAssignCtrl         =somAssignCtrlStruct;
  som3AssignCtrl        =somAssignCtrlStruct;

(*----------------------------------------------------------------------
 *  The Class Data Structures -- these are used to implement static
 *  method and data interfaces to SOM objects.
 *)

type
(* -- (Generic) Class data Structure *)
  somClassDataStructure =record
    classObject         :SOMClassType;                 (* changed by shadowing *)
    tokens              :Array[0..0] of somToken;      (* method tokens, etc. *)
  end;
  somClassDataStructurePtr=^somClassDataStructure;

  somInitCtrlPtr        =^somInitCtrl;
  somDestructCtrlPtr    =^somDestructCtrl;
  somAssignCtrlPtr      =^somAssignCtrl;

(* -- For building lists of method tables *)
  somMethodTabPtr       =^somMethodTab;

  somMethodTabs         =^somMethodTabList;
  somMethodTabList      =record
    mtab                :somMethodTabPtr;
    next                :somMethodTabs;
  end;

  somParentMtabStruct   =record
    mtab                :somMethodTabPtr;       (* this class' mtab -- changed by shadowing *)
    next                :somMethodTabs;         (* the parent mtabs -- unchanged by shadowing *)
    classObject         :SOMClassType;          (* unchanged by shadowing *)
    somRenewNoInitNoZeroThunk:somTD_somRenewNoInitNoZeroThunk; (* changed by shadowing *)
    instanceSize        :Longint;               (* changed by shadowing *)
    initializers        :somMethodProcPtr;      (* resolved initializer array in releaseorder *)
    resolvedMTokens     :somMethodProcPtr;      (* resolved methods *)
    initCtrl            :somInitCtrl;           (* these fields are filled in if somDTSClass&2 is on *)
    destructCtrl        :somDestructCtrl;
    assignCtrl          :somAssignCtrl;
    embeddedTotalCount  :Longint;
    hierarchyTotalCount :Longint;
    unused              :Longint;
  end;
  somParentMtabStructPtr=^somParentMtabStruct;

(*
 * (Generic) Auxiliary Class Data Structure
 *)
  somCClassDataStructure=record
    parentMtab          :somParentMtabStructPtr;
    instanceDataToken   :somDToken;
    wrappers            :Array[0..0] of somMethodProc;  (* for valist methods *)
  end;
  somCClassDataStructurePtr=^somCClassDataStructure;


(*----------------------------------------------------------------------
 *  The Method Table Structure
 *)

(* -- to specify an embedded object (or array of objects). *)
  somEmbeddedObjStructPtr=^somEmbeddedObjStruct;
  somEmbeddedObjStruct  =record
    copp                :SOMClassType;  (* address of class of object ptr *)
    cnt                 :Longint;       (* object count *)
    offset              :Longint;       (* Offset to pointer (to embedded objs) *)
  end;

  somMethodTabStruct    =record
    classObject         :SOMClassType;
    classInfo           :somClassInfoPtr;
    className           :PChar;
    instanceSize        :Longint;
    dataAlignment       :Longint;
    mtabSize            :Longint;
    protectedDataOffset :Longint;       (* from class's introduced data *)
    protectedDataToken  :somDToken;
    embeddedObjs        :somEmbeddedObjStructPtr;
    (* remaining structure is opaque *)
    entries             :Array[0..0] of somMethodProc;
  end;
  somMethodTab          =somMethodTabStruct;

(* -- For building lists of class objects *)
  somClasses            =^somClassList;
  somClassList          =record
    cls                 :SOMClassType;
    next                :somClasses;
  end;

(* -- For building lists of objects *)
  somObjects            =^somObjectList;
  somObjectList         =record
    obj                 :TSOMObject;
    next                :somObjects;
  end;



(*----------------------------------------------------------------------
 * Method Stubs -- Signature Support
 *
 *
 * This section defines the structures used to pass method signature
 * ingo to the runtime. This supports selection of generic apply stubs
 * and runtime generation of redispatchstubs when these are needed. The
 * information is registered with the runtime when methods are defined.
 *
 * When calling somAddStaticMethod, if the redispatchStub is -1, then a
 * pointer to a struct of type somApRdInfo is passed as the applyStub.
 * Otherwise, the passed redispatchstub and applystub are taken as given.
 * When calling somAddDynamicMethod, an actual apply stub must be passed.
 * Redispatch stubs for dynamic methods are not available, nor is
 * automated support for dynamic method apply stubs. The following
 * atructures only appropriate in relation to static methods.
 *
 * In SOMr2, somAddStaticMethod can be called with an actual redispatchstub
 * and applystub *ONLY* if the method doesn't return a structure. Recall
 * that no SOMr1 methods returned structures, so SOMr1 binaries obey this
 * restriction. The reason for this rule is that SOMr2 *may* use thunks,
 * and thunks need to know if a structure is returned. We therefore assume
 * that if no signature information is provided for a method through the
 * somAddStaticMethod interface, then the method returns a scalar.
 *
 * If a structure is returned, then a -1 *must* be passed to
 * somAddStaticMethod as a redispatchstub. In any case, if a -1 is passed,
 * then this means that the applystub actually points to a structure of type
 * somApRdInfo. This structure is used to hold and access signature
 * information encoded as follows.
 *
 * If the somApRdInfo pointer is NULL, then, if the runtime was built with
 * SOM_METHOD_STUBS defined, a default signature is assumed (no arguments,
 * and no structure returned); otherwise, the stubs are taken as
 * somDefaultMethod (which produces a runtime error when used) if dynamic
 * stubs are not available.
 *
 * If the somApRdInfo pointer is not NULL, then the structure it points to can
 * either include (non-null) redispatch and applystubs (the method is then
 * assumed to return a structure), or null stubs followed by information needed
 * to generate necessary stubs dynamically.
 *)

  somApRdInfoStruct     =record
    rdStub              :somMethodProc;
    apStub              :somMethodProc;
    stubInfo            :somMethodInfoPtr;
  end;
  somApRdInfo           =somApRdInfoStruct;


(*
 * Values for somRdAppType are generated by summing one from column A and one
 * from column B of the following constants:
 *)
(* Column A: return type *)
const
  SOMRdRetsimple        = 0; (* Return type is a non-float fullword *)
  SOMRdRetfloat         = 2; (* Return type is (single) float *)
  SOMRdRetdouble        = 4; (* Return type is double *)
  SOMRdRetlongdouble    = 6; (* Return type is long double *)
  SOMRdRetaggregate     = 8; (* Return type is struct or union *)
  SOMRdRetbyte          =10; (* Return type is a byte *)
  SOMRdRethalf          =12; (* Return type is a (2 byte) halfword *)
(* Column B: are there any floating point scalar arguments? *)
  SOMRdNoFloatArgs      = 0;
  SOMRdFloatArgs        = 1;

(* A somFloatMap is only needed on RS/6000 *)
(*
 * This is an array of offsets for up to the first 13 floating point arguments.
 * If there are fewer than 13 floating point arguments, then there will be
 * zero entries following the non-zero entries which represent the float args.
 * A non-zero entry signals either a single- or a double-precision floating point
 * argument. For a double-precision argument, the entry is the stack
 * frame offset. For a single-precision argument the entry is the stack
 * frame offset + 1. For the final floating point argument, add 2 to the
 * code that would otherwise be used.
 *)
  SOMFMSingle           = 1; (* add to indicate single-precision *)
  SOMFMLast             = 2; (* add to indicate last floating point arg *)

const
  SOM_SCILEVEL          = 4;


(* The SCI includes the following information:
 *
 * The address of a class's ClassData structure is passed.
 * This structure should have the external name,
 * <className>ClassData. The classObject field should be NULL
 * (if it is not NULL, then a new class will not be built). somBuildClass will
 * set this field to the address of the new class object when it is built.
 *
 * The address of the class's auxiliary ClassData structure is passed.
 * Thi structure should have the external name,
 * <className>CClassData. The parentMtab field will be set by somBuildClass.
 * This field often allows method calls to a class object to be avoided.
 *
 * The other structures referenced by the static class information (SCI)
 * are used to:
 *)

(*
 * to specify a static method. The methodId used here must be
 * a simple name (i.e., no colons). In all other cases,
 * where a somId is used to identify a registered method,
 * the somId can include explicit scoping. An explicitly-scoped
 * method name is called a method descriptor. For example,
 * the method introduced by TSOMObject as somGetClass has the
 * method descriptor "TSOMObject::somGetClass". When a
 * class is contained in an IDL module, the descriptor syntax
 * <moduleName>::<className>::<methodName> can be used. Method
 * descriptors can be useful when a class supports different methods
 * that have the same name (note: IDL prevents this from occuring
 * statically, but SOM itself has no problems with this).
 *)

type
  somStaticMethodStruct =record
    classData           :somMTokenPtr;
    methodId            :somIdPtr;        (* this must be a simple name (no colons) *)
    methodDescriptor    :somIdPtr;
    method              :somMethodPtr;//somMethodProc;
    redispatchStub      :somMethodPtr;//somMethodProc;
    applyStub           :somMethodPtr;//somMethodProc;
  end;
  somStaticMethod_t     =somStaticMethodStruct;
  somStaticMethod_p     =^somStaticMethod_t;

(* to specify an overridden method *)
  somOverideMethodStruct=record
    methodId            :somIdPtr;        (* this can be a method descriptor *)
    method              :somMethodPtr;//somMethodProc;
  end;
  somOverrideMethod_t   =somOverideMethodStruct;
  somOverrideMethod_p   =^somOverrideMethod_t;

(* to inherit a specific parent's method implementation *)
  somInheritedMethodStruct=record
    methodId            :somIdPtr;      (* identify the method *)
    parentNum           :Longint;       (* identify the parent *)
    mToken              :somMTokenPtr;  (* for parentNumresolve *)
  end;
  somInheritedMethod_t  =somInheritedMethodStruct;
  somInheritedMethod_p  =^somInheritedMethod_t;

(* to register a method that has been moved from this *)
(* class <cls> upwards in the class hierachy to class <dest> *)
  somMigratedMethodStruct=record
    clsMToken           :somMTokenPtr;
                                (* points into the <cls> classdata structure *)
                                (* the method token in <dest> will copied here *)
    destMToken          :somMTokenPtr;
                                (* points into the <dest> classdata structure *)
                                (* the method token here will be copied to <cls> *)
  end;
  somMigratedMethod_t   =somMigratedMethodStruct;
  somMigratedMethod_p        =^somMigratedMethod_t;

(* to specify non-internal data *)
  somNonInternalDataStruct=record
    classData           :somDTokenPtr;
    basisForDataOffset  :PChar;
  end;
  somNonInternalData_t  =somNonInternalDataStruct;
  somNonInternalData_p  =^somNonInternalData_t;

(* to specify a "procedure" or "classdata" *)
  somProcMethodsStruct  =record
    classData           :somMethodProcPtr;
    pEntry              :somMethodProc;
  end;
  somProcMethods_t      =somProcMethodsStruct;
  somProcMethods_p      =^somProcMethods_t;

(* to specify a general method "action" using somMethodStruct *)
(*
  the type of action is specified by loading the type field of the
  somMethodStruct. There are three bit fields in the overall type:

  action (in type & 0xFF)
   0: static -- (i.e., virtual) uses somAddStaticMethod
   1: dynamic -- uses somAddDynamicMethod (classData==0)
   2: nonstatic -- (i.e., nonvirtual) uses somAddMethod
   3: udaAssign -- registers a method as the udaAssign (but doesn't add the method)
   4: udaConstAssign -- like 3, this doesn't add the method
   5: somClassResolve Override (using the class pointed to by *classData)
   6: somMToken Override (using the method token pointed to by methodId)
                         (note: classData==0 for this)
   7: classAllocate -- indicates the default heap allocator for this class.
                       If classData == 0, then method is the code address (or NULL)
                       If classData != 0, then *classData is the code address.
                       No other info required (or used)
   8: classDeallocate -- like 7, but indicates the default heap deallocator.
   9: classAllocator -- indicates a non default heap allocator for this class.
                        like 7, but a methodDescriptor can be given.

   === the following is not currently supported ===
   binary data access -- in (type & 0x100), valid for actions 0,1,2,5,6
   0: the method procedure doesn't want binary data access
   1: the method procedure does want binary data access

   aggregate return -- in (type & 0x200), used when binary data access requested
   0: method procedure doesn't return a structure
   1: method procedure does return a structure
*)

  somMethodStruct       =record
    mtype               :Longint;
    classData           :somMTokenPtr;
    methodId            :somIdPtr;
    methodDescriptor    :somIdPtr;
    method              :somMethodProc;
    redispatchStub      :somMethodProc;
    applyStub           :somMethodProc;
  end;
  somMethods_t          =somMethodStruct;
  somMethods_p          =^somMethods_t;

(* to specify a varargs function *)
  somVarargsFuncsStruct =record
    classData           :somMethodProcPtr;
    vEntry              :somMethodProc;
  end;
  somVarargsFuncs_t     =somVarargsFuncsStruct;
  somVarargsFuncs_p     =^somVarargsFuncs_t;

(* to specify dynamically computed information (incl. embbeded objs) *)
  somDynamicSCIPtr      =^somDynamicSciPtr;
  somDynamicSCI         =record
    version             :Longint;       (* 1 for now *)
    instanceDataSize    :Longint;       (* true size (incl. embedded objs) *)
    dataAlignment       :Longint;       (* true alignment *)
    embeddedObjs        :somEmbeddedObjStructPtr; (* array end == null copp *)
  end;


(*
   to specify a DTS class, use the somDTSClass entry in the following
   data structure. This entry is a bit vector interpreted as follows:

   (somDTSClass & 0x0001) == the class is a DTS C++ class
   (somDTSClass & 0x0002) == the class wants the initCtrl entries
                             of the somParentMtabStruct filled in.

*)



(*
 *  The Static Class Info Structure passed to somBuildClass
 *)

  somStaticClassInfoStruct=record
    layoutVersion       :Longint;  (* this struct defines layout version SOM_SCILEVEL *)
    numStaticMethods    :Longint;   (* count of smt entries *)
    numStaticOverrides  :Longint; (* count of omt entries *)
    numNonInternalData  :Longint; (* count of nit entries *)
    numProcMethods      :Longint;     (* count of pmt entries *)
    numVarargsFuncs     :Longint;    (* count of vft entries *)
    majorVersion        :Longint;
    minorVersion        :Longint;
    instanceDataSize    :Longint;   (* instance data introduced by this class *)
    maxMethods          :Longint;         (* count numStaticMethods and numMethods *)
    numParents          :Longint;
    classId             :somId;
    explicitMetaId      :somId;
    implicitParentMeta  :Longint;
    parents             :somIdPtr;
    cds                 :somClassDataStructurePtr;
    ccds                :somCClassDataStructurePtr;
    smt                 :somStaticMethod_p; (* basic "static" methods for mtab *)
    omt                 :somOverrideMethod_p; (* overrides for mtab *)
    nitReferenceBase    :PChar;
    nit                 :somNonInternalData_p; (* datatokens for instance data *)
    pmt                 :somProcMethods_p; (* Arbitrary ClassData members *)
    vft                 :somVarargsFuncs_p; (* varargs stubs *)
    cif                 :pointer{^somTP_somClassInitFunc}; (* class init function *)
    (* end of layout version 1 *)

    (* begin layout version 2 extensions *)
    dataAlignment       :Longint; (* the desired byte alignment for instance data *)
    (* end of layout version 2 *)

//#define SOMSCIVERSION 1

    (* begin layout version 3 extensions *)
    numDirectInitClasses:Longint;
    directInitClasses   :somIdPtr;
    numMethods          :Longint; (* general (including nonstatic) methods for mtab *)
    mt                  :somMethods_p;
    protectedDataOffset :Longint; (* access = resolve(instanceDataToken) + offset *)
    somSCIVersion       :Longint;  (* used during development. currently = 1 *)
    numInheritedMethods :Longint;
    imt                 :somInheritedMethod_p; (* inherited method implementations *)
    numClassDataEntries :Longint; (* should always be filled in *)
    classDataEntryNames :somIdPtr; (* either NULL or ptr to an array of somIds *)
    numMigratedMethods  :Longint;
    mmt                 :somMigratedMethod_p; (* migrated method implementations *)
    numInitializers     :Longint; (* the initializers for this class *)
    initializers        :somIdPtr;     (* in order of release *)
    somDTSClass         :Longint; (* used to identify a DirectToSOM class *)
    dsci                :somDynamicSCIPtr;  (* used to register dynamically computed info *)
    (* end of layout version 3 *)
  end;
  somStaticClassInfo    =somStaticClassInfoStruct;
  somStaticClassInfoPtr =^somStaticClassInfoStruct;


type
(*----------------------------------------------------------------------
 *  Typedefs for pointers to functions
 *)

  Contained_Description         = record
    name                        : Identifier;
    value                       : any;
  end;

  InterfaceDef_FullInterfaceDescription = record
    name                : Identifier;
    id, defined_in      : RepositoryId;
    {operation           : IDL_SEQUENCE_OperationDef_OperationDescription;
    attributes          : IDL_SEQUENCE_AttributeDef_AttributeDescription;}
  end;

  InterfaceDef_InterfaceDescription = record
    name                : Identifier;
    id, defined_in      : RepositoryId;
  end;

(* CORBA 7.6.1, p.139 plus 5.7, p.89 enum Data Type Mapping *)
type
  TCKind                = Cardinal;
const
  TypeCode_tk_null      = 1;
  TypeCode_tk_void      = 2;
  TypeCode_tk_short     = 3;
  TypeCode_tk_long      = 4;
  TypeCode_tk_ushort    = 5;
  TypeCode_tk_ulong     = 6;
  TypeCode_tk_float     = 7;
  TypeCode_tk_double    = 8;
  TypeCode_tk_boolean   = 9;
  TypeCode_tk_char      = 10;
  TypeCode_tk_octet     = 11;
  TypeCode_tk_any       = 12;
  TypeCode_tk_TypeCode  = 13;
  TypeCode_tk_Principal = 14;
  TypeCode_tk_objref    = 15;
  TypeCode_tk_struct    = 16;
  TypeCode_tk_union     = 17;
  TypeCode_tk_enum      = 18;
  TypeCode_tk_string    = 19;
  TypeCode_tk_sequence  = 20;
  TypeCode_tk_array     = 21;

  TypeCode_tk_pointer   = 101; (* SOM extension *)
  TypeCode_tk_self      = 102; (* SOM extension *)
  TypeCode_tk_foreign   = 103; (* SOM extension *)

(* Short forms of tk_<x> enumerators *)

  tk_null       = TypeCode_tk_null;
  tk_void       = TypeCode_tk_void;
  tk_short      = TypeCode_tk_short;
  tk_long       = TypeCode_tk_long;
  tk_ushort     = TypeCode_tk_ushort;
  tk_ulong      = TypeCode_tk_ulong;
  tk_float      = TypeCode_tk_float;
  tk_double     = TypeCode_tk_double;
  tk_boolean    = TypeCode_tk_boolean;
  tk_char       = TypeCode_tk_char;
  tk_octet      = TypeCode_tk_octet;
  tk_any        = TypeCode_tk_any;
  tk_TypeCode   = TypeCode_tk_TypeCode;
  tk_Principal  = TypeCode_tk_Principal;
  tk_objref     = TypeCode_tk_objref;
  tk_struct     = TypeCode_tk_struct;
  tk_union      = TypeCode_tk_union;
  tk_enum       = TypeCode_tk_enum;
  tk_string     = TypeCode_tk_string;
  tk_sequence   = TypeCode_tk_sequence;
  tk_array      = TypeCode_tk_array;
  tk_pointer    = TypeCode_tk_pointer;
  tk_self       = TypeCode_tk_self;
  tk_foreign    = TypeCode_tk_foreign;

type
  SOMClass_somOffsets           = record
    cls                         : SOMClassType;
    offset                      : Longint;
  end;

  _IDL_SEQUENCE_SOMClass        = record
    _maximum                    : Cardinal;
    _length                     : Cardinal;
    _buffer                     : SOMClassPtr;
  end;
  _IDL_SEQUENCE_SOMObject       = record
    _maximum                    : Cardinal;
    _length                     : Cardinal;
    _buffer                     : PSOMObject;
  end;
  SOMClass_SOMClassSequence     = _IDL_SEQUENCE_SOMClass;

(*----------------------------------------------------------------------
 *  Windows extra procedures:
 *)

(*
 *  Replaceable character output handler.
 *  Points to the character output routine to be used in development
 *  support.  Initialized to <somOutChar>, but may be reset at anytime.
 *  Should return 0 (false) if an error occurs and 1 (true) otherwise.
 *)
type
  somTD_SOMOutCharRoutine       =Function(ch:Char):Longint; cdecl;

var
{$warning support of external vars required}
  SOMOutCharRoutine     :somTD_SOMOutCharRoutine;//³ 00015 ³ SOMOutCharRoutine

Procedure somSetOutChar(outch:somTD_SOMOutCharRoutine); cdecl;
  external 'som' name 'somSetOutChar'; {index 85}

Function  somMainProgram:SOMClassMgrType; cdecl;
  external 'som' name 'somMainProgram'; {index 88}
Procedure somEnvironmentEnd; cdecl;
  external 'som' name 'somEnvironmentEnd'; {index 83}
Function  somAbnormalEnd:Boolean; cdecl;
  external 'som' name 'somAbnormalEnd'; {index 84}

(*--------------------------------------------------------*)


(*---------------------------------------------------------------------
 *  Offset-based method resolution.
 *)

Function  somResolve(obj:TSOMObject; mdata:somMToken):{somMethodProc}pointer; cdecl;
  external 'som' name 'somResolve'; {index 37}
Function  somParentResolve(parentMtabs:somMethodTabs;
                           mToken:somMToken):somMethodProc; cdecl;
  external 'som' name 'somParentResolve'; {index 33}
Function  somParentNumResolve(parentMtabs:somMethodTabs;
                              parentNum:Longint;mToken:somMToken):{somMethodProc}pointer; cdecl;
  external 'som' name 'somParentNumResolve'; {index 50}
Function  somClassResolve(obj:SOMClassType; mdata:somMToken):{somMethodProc}pointer; cdecl;
  external 'som' name 'somClassResolve'; {index 48}
Function  somAncestorResolve(obj:TSOMObject;                 (* the object *)
                             var ccds:somCClassDataStructure;   (* id the ancestor *)
                             mToken:somMToken):{somMethodProc}pointer; cdecl;
  external 'som' name 'somAncestorResolve'; {index 74}
Function  somResolveByName(obj:TSOMObject;
                           methodName:PChar):{somMethodProc}pointer; cdecl;
  external 'som' name 'somResolveByName'; {index 61}
(*------------------------------------------------------------------------------
 * Offset-based data resolution
 *)
Function  somDataResolve(obj:TSOMObject; dataId:somDToken):somToken; cdecl;
  external 'som' name 'somDataResolve'; {index 47}
Function  somDataResolveChk(obj:TSOMObject; dataId:somDToken):somToken; cdecl;
  external 'som' name 'somDataResolveChk'; {index 72}


(*----------------------------------------------------------------------
 *  Misc. procedures:
 *)

(*
 *  Create and initialize the SOM environment
 *
 *  Can be called repeatedly
 *
 *  Will be called automatically when first object (including a class
 *  object) is created, if it has not already been done.
 *
 *  Returns the SOMClassMgrObject
 *)
Function  somEnvironmentNew:SOMClassMgrType; cdecl;
  external 'som' name 'somEnvironmentNew'; {index 30}

(*
 * Test whether <obj> is a valid SOM object. This test is based solely on
 * the fact that (on this architecture) the first word of a SOM object is a
 * pointer to its method table. The test is therefore most correctly understood
 * as returning true if and only if <obj> is a pointer to a pointer to a
 * valid SOM method table. If so, then methods can be invoked on <obj>.
 *)
Function  somIsObj(obj:somToken):Boolean; cdecl;
  external 'som' name 'somIsObj'; {index 60}

(*
 * Return the class that introduced the method represented by a given method token.
 *)
Function  somGetClassFromMToken(mToken:somMToken):SOMClassType; cdecl;
  external 'som' name 'somGetClassFromMToken'; {index 82}


(*----------------------------------------------------------------------
 *  String Manager: stem <somsm>
 *)
Function  somCheckID(id:somId):somId; cdecl;
  external 'som' name 'somCheckId'; {index 26}
(* makes sure that the id is registered and in normal form, returns *)
(* the id *)

Function  somRegisterId(id:somId):Longint; cdecl;
  external 'som' name 'somRegisterId'; {index 36}
(* Same as somCheckId except returns 1 (true) if this is the first *)
(* time the string associated with this id has been registered, *)
(* returns 0 (false) otherwise *)

Function  somIDFromString(aString:PChar):somId; cdecl;
  external 'som' name 'somIdFromString'; {index 31}
(* caller is responsible for freeing the returned id with SOMFree *)

// Not found
//Function  somIdFromStringNoFree(aString:PChar):somId; cdecl;
(* call is responsible for *not* freeing the returned id *)

Function  somStringFromId(id:somId):PChar; cdecl;
  external 'som' name 'somStringFromId'; {index 40}

Function  somCompareIds(id1,id2:somId):Longint; cdecl;
  external 'som' name 'somCompareIds'; {index 27}
(* returns true (1) if the two ids are equal, else false (0) *)

Function  somTotalRegIds:Longint; cdecl;
  external 'som' name 'somTotalRegIds'; {index 43}
(* Returns the total number of ids that have been registered so far, *)
(* you can use this to advise the SOM runtime concerning expected *)
(* number of ids in later executions of your program, via a call to *)
(* somSetExpectedIds defined below *)

Procedure somSetExpectedIds(numIds:Longint{ulong}); cdecl;
  external 'som' name 'somSetExpectedIds'; {index 39}
(* Tells the SOM runtime how many unique ids you expect to use during *)
(* the execution of your program, this can improve space and time *)
(* utilization slightly, this routine must be called before the SOM *)
(* environment is created to have any effect *)

Function  somUniqueKey(id:somId):Longint{ulong}; cdecl;
  external 'som' name 'somUniqueKey'; {index 44}
(* Returns the unique key for this id, this key will be the same as the *)
(* key in another id if and only if the other id refers to the same *)
(* name as this one *)

Procedure somBeginPersistentIds; cdecl;
  external 'som' name 'somBeginPersistentIds'; {index 24}
(* Tells the id manager that strings for any new ids that are *)
(* registered will never be freed or otherwise modified. This allows *)
(* the id manager to just use a pointer to the string in the *)
(* unregistered id as the master copy of the ids string. Thus saving *)
(* space *)
(* Under normal use (where ids are static varibles) the string *)
(* associated with an id would only be freed if the code module in *)
(* which it occured was unloaded *)

Procedure somEndPersistentIds; cdecl;
  external 'som' name 'somEndPersistentIds'; {index 29}
(* Tells the id manager that strings for any new ids that are *)
(* registered may be freed or otherwise modified.  Therefore the id *)
(* manager must copy the strings inorder to remember the name of an *)
(* id. *)

(*----------------------------------------------------------------------
 *  Class Manager: SOMClassMgrType, stem <somcm>
 *)

(* Global class manager object *)
var
{$warning support of external vars required}
  SOMClassMgrObject     : SOMClassMgrType;//³ 00007 ³ SOMClassMgrObject

(* The somRegisterClassLibrary function is provided for use
 * in SOM class libraries on platforms that have loader-invoked
 * entry points associated with shared libraries (DLLs).
 *
 * This function registers a SOM Class Library with the SOM Kernel.
 * The library is identified by its file name and a pointer
 * to its initialization routine.  Since this call may occur
 * prior to the invocation of somEnvironmentNew, its actions
 * are deferred until the SOM environment has been initialized.
 * At that time, the SOMClassMgrObject is informed of all
 * pending library initializations via the _somRegisterClassLibrary
 * method.  The actual invocation of the library's initialization
 * routine will occur during the execution of the SOM_MainProgram
 * macro (for statically linked libraries), or during the _somFindClass
 * method (for libraries that are dynamically loaded).
 *)
Procedure somRegisterClassLibrary(libraryName:PChar;
                                  libraryInitRun:somMethodProc); cdecl;
  external 'som' name 'somRegisterClassLibrary'; {index 86}


(*----------------------------------------------------------------------
 * -- somApply --
 *
 * This routine replaces direct use of applyStubs in SOMr1. The reason
 * for the replacement is that the SOMr1 style of applyStub is not
 * generally available in SOMr2, which uses a fixed set of applyStubs,
 * according to method information in the somMethodData. In particular,
 * neither the redispatch stub nor the apply stub found in the method
 * data structure are necessarily useful as such. The method somGetRdStub
 * is the way to get a redispatch stub, and the above function is the
 * way to call an apply stub. If an appropriate apply stub for the
 * method indicated by md is available, then this is invoked and TRUE is
 * returned; otherwise FALSE is returned.
 *
 * The va_list passed to somApply *must* include the target object,
 * somSelf, as its first entry, and any single precision floating point
 * arguments being passed to the the method procedure must be
 * represented on the va_list using double precision values. retVal cannot
 * be NULL.
 *)

Function  somApply(var somSelf:TSOMObject;
                   var retVal:somToken;
                   mdPtr:somMethodDataPtr;
                   var ap):Boolean; cdecl;
  external 'som' name 'somApply'; {index 69}

(*---------------------------------------------------------------------
 * -- somBuildClass --
 *
 * This procedure automates construction of a new class object. A variety of
 * special structures are used to allow language bindings to statically define
 * the information necessary to specify a class. Pointers to these static
 * structures are accumulated into an overall "static class information"
 * structure or SCI, passed to somBuildClass. The SCI has evolved over time.
 * The current version is defined here.
 *)


Function  somBuildClass(inherit_vars:Longint;
                        var sci:somStaticClassInfo;
                        majorVersion,minorVersion:Longint):SOMClassType; cdecl;
  external 'som' name 'somBuildClass'; {index 49}

  (*
  The arguments to somBuildClass are as follows:

   inherit_vars: a bit mask used to control inheritance of implementation
   Implementation is inherited from parent i iff the bit 1<<i is on, or i>=32.

   sci: the somStaticClassInfo defined above.

   majorVersion, minorVersion: the version of the class implementation.

   *)


(*---------------------------------------------------------------------
 *  Used by old single-inheritance emitters to make class creation
 *  an atomic operation. Kept for backwards compatability.
 *)
type
  somTD_classInitRoutine=Procedure(var a,b:SOMClassType); cdecl;

Procedure somConstructClass(classInitRoutine:somTD_ClassInitRoutine;
                            parentClass,metaClass:SOMClassType;
                            var cds :somClassDataStructure); cdecl;
  external 'som' name 'somConstructClass'; {index 28}


(*
 * Uses <SOMOutCharRoutine> to output its arguments under control of the ANSI C
 * style format.  Returns the number of characters output.
 *)
Function  somPrintf(fnt:PChar;buf:pointer):Longint; cdecl;
  external 'som' name 'somPrintf'; {index 35}

// vprint form of somPrintf
Function  somVPrintf(fnt:PChar;var ap):Longint; cdecl;
  external 'som' name 'somVprintf'; {index 45}

// Outputs (via somPrintf) blanks to prefix a line at the indicated level
Procedure somPrefixLevel(level:Longint); cdecl;
  external 'som' name 'somPrefixLevel'; {index 34}

// Combines somPrefixLevel and somPrintf
Procedure somLPrintf(level:Longint;fmt:PChar;var buf); cdecl;
  external 'som' name 'somLPrintf'; {index 32}

Function SOMObjectNewClass(majorVersion,minorVersion:Longint):SOMClassType; cdecl;
  external 'som' name 'SOMObjectNewClass'; {index 22}
Function SOMClassNewClass(majorVersion,minorVersion:Longint):SOMClassType; cdecl;
  external 'som' name 'SOMClassNewClass'; {index 21}

Function SOMClassMgrNewClass(majorVersion,minorVersion:Longint):SOMClassType; cdecl;
  external 'som' name 'SOMClassMgrNewClass'; {index 20}

(*----------------------------------------------------------------------
 * Pointers to routines used to do dynamic code loading and deleting
 *)
type
  somTD_SOMLoadModule           =Function({IN}Module:PChar      (* className *);
                                          {IN}FileName:PChar    (* fileName *);
                                          {IN}FuncName:PChar    (* functionName *);
                                          {IN}MajorVer:Longint  (* majorVersion *);
                                          {IN}MinorVer:Longint  (* minorVersion *);
                                          {OUT}var ref:somToken (* modHandle *)):Longint; cdecl;
  somTD_SOMDeleteModule         =Function({IN} ref:somToken     (* modHandle *)):Longint; cdecl;
  somTD_SOMClassInitFuncName    =Function:PChar; cdecl;

var
{$warning support of external vars required}
  SOMLoadModule         :somTD_SOMLoadModule;//³ 00011 ³ SOMLoadModule
{$warning support of external vars required}
  SOMDeleteModule       :somTD_SOMDeleteModule;//³ 00008 ³ SOMDeleteModule
{$warning support of external vars required}
  SOMClassInitFuncName  :somTD_SOMClassInitFuncName; //³ 00004 ³ SOMClassInitFuncName

(*----------------------------------------------------------------------
 *  Replaceable SOM Memory Management Interface
 *
 *  External procedure variables SOMCalloc, SOMFree, SOMMalloc, SOMRealloc
 *  have the same interface as their standard C-library analogs.
 *)

type
  somTD_SOMMalloc               =Function({IN} size_t:Longint   (* nbytes *)):somToken; cdecl;
  somTD_SOMCalloc               =Function({IN} size_c:Longint   (* element_count *);
                                          {IN} size_e:Longint   (* element_size *)):somToken; cdecl;
  somTD_SOMRealloc              =Function({IN} ref:somToken     (* memory *);
                                          {IN} size:Longint     (* nbytes *)):somToken; cdecl;
  somTD_SOMFree                 =Procedure({IN} ref:somToken    (* memory *)); cdecl;

var
{$warning support of external vars required}
  SOMCalloc             :somTD_SOMCalloc; // ³ 00001 ³ SOMCalloc
{$warning support of external vars required}
  SOMFree               :somTD_SOMFree; //³ 00010 ³ SOMFree
{$warning support of external vars required}
  SOMMalloc             :somTD_SOMMalloc;//³ 00012 ³ SOMMalloc
{$warning support of external vars required}
  SOMRealloc            :somTD_SOMRealloc;//³ 00016 ³ SOMRealloc

(*----------------------------------------------------------------------
 *  Replaceable SOM Error handler
 *)

type
  somTD_SOMError                =Procedure({IN} code:Longint    (* code *);
                                           {IN} fn:PChar        (* fileName *);
                                           {IN} ln:Longint      (* linenum *)); cdecl;

var
{$warning support of external vars required}
  SOMError              :somTD_SOMError;//³ 00009 ³ SOMError

(*----------------------------------------------------------------------
 *  Replaceable SOM Semaphore Operations
 *
 *  These operations are used by the SOM Kernel to make thread-safe
 *  state changes to internal resources.
 *)

type
  somTD_SOMCreateMutexSem       =Function({OUT}var sem:somToken ):Longint; cdecl;
  somTD_SOMRequestMutexSem      =Function({IN}sem:somToken ):Longint; cdecl;
  somTD_SOMReleaseMutexSem      =Function({IN}sem:somToken ):Longint; cdecl;
  somTD_SOMDestroyMutexSem      =Function({IN}sem:somToken ):Longint; cdecl;

var
{$warning support of external vars required}
  SOMCreateMutexSem     :somTD_SOMCreateMutexSem;//³ 00090 ³ SOMCreateMutexSem
{$warning support of external vars required}
  SOMRequestMutexSem    :somTD_SOMRequestMutexSem;//³ 00091 ³ SOMRequestMutexSem
{$warning support of external vars required}
  SOMReleaseMutexSem    :somTD_SOMReleaseMutexSem;//³ 00092 ³ SOMReleaseMutexSem
{$warning support of external vars required}
  SOMDestroyMutexSem    :somTD_SOMDestroyMutexSem;//³ 00093 ³ SOMDestroyMutexSem

(*----------------------------------------------------------------------
 *  Replaceable SOM Thread Identifier Operation
 *
 *  This operation is used by the SOM Kernel to index data unique to the
 *  currently executing thread.  It must return a small integer that
 *  uniquely represents the current thread within the current process.
 *)

type
  somTD_SOMGetThreadId          =Function:Longint; cdecl;

var
{$warning support of external vars required}
  SOMGetThreadId        :somTD_SOMGetThreadId;//³ 00094 ³ SOMGetThreadId


(*----------------------------------------------------------------------
 * Externals used in the implementation of SOM, but not part of the
 * SOM API.
 *)

Function  somTestCls(obj:TSOMObject; classObj:SOMClassType;
                     fileName:PChar; lineNumber:Longint):TSOMObject; cdecl;
  external 'som' name 'somTestCls'; {index 42}
Procedure somTest(condition,severity:Longint;fileName:PChar;
                  lineNum:Longint;msg:PChar); cdecl;
  external 'som' name 'somTest'; {index 41}
Procedure somAssert(condition,ecode:Longint;
                    fileName:PChar;lineNum:Longint;msg:PChar); cdecl;
  external 'som' name 'somAssert'; {index 23}

type
  exception_type        = (NO_EXCEPTION, USER_EXCEPTION, SYSTEM_EXCEPTION);
  completion_status     = (YES, NO, MAYBE);
  StExcep               = record
    minot               : Cardinal;
    completed           : completion_status;
  end;

  Environment           =^EnvironmentType;
  EnvironmentType       = record
    _major              : exception_type;
    exception           : record
      _exception_name   : PChar;
      _params           : Pointer;
    end;
    _somdAnchor         : pointer;
  end;

Function  somExceptionId(ev:Environment):PChar; cdecl;
  external 'som' name 'somExceptionId'; {index 52}
Function  somExceptionValue(ev:Environment):Pointer; cdecl;
  external 'som' name 'somExceptionValue'; {index 53}
Procedure somExceptionFree(ev:Environment); cdecl;
  external 'som' name 'somExceptionFree'; {index 54}
Procedure somSetException(ev:Environment;major:exception_type;exception_name:PChar;params:pointer); cdecl;
  external 'som' name 'somSetException'; {index 55}
Function  somGetGlobalEnvironment:Environment; cdecl;
  external 'som' name 'somGetGlobalEnvironment'; {index 58}

(* Exception function names per CORBA 5.19, p.99 *)
Function  exception_id(ev:Environment):PChar; cdecl;
Function  exception_value(ev:Environment):Pointer; cdecl;
Procedure exception_free(ev:Environment); cdecl;


(*  Convenience macros for manipulating environment structures
 *
 *  SOM_CreateLocalEnvironment returns a pointer to an Environment.
 *  The other 3 macros all expect a single argument that is also
 *  a pointer to an Environment.  Use the create/destroy forms for
 *  a dynamic local environment and the init/uninit forms for a stack-based
 *  local environment.
 *
 *  For example
 *
 *      Environment *ev;
 *      ev = SOM_CreateLocalEnvironment ();
 *      ... Use ev in methods
 *      SOM_DestroyLocalEnvironment (ev);
 *
 *  or
 *
 *      Environment ev;
 *      SOM_InitEnvironment (&ev);
 *      ... Use &ev in methods
 *      SOM_UninitEnvironment (&ev);
 *)
Function SOM_CreateLocalEnvironment:Environment; cdecl;

Procedure SOM_DestroyLocalEnvironment(ev:Environment); cdecl;

Procedure SOM_InitEnvironment(ev:Environment); cdecl;

Procedure SOM_UninitEnvironment(ev:Environment); cdecl;

(*----------------------------------------------------------------------
 * Macros are used in the C implementation of SOM... However, Pascal
 * doesn't have macro capability... (from SOMCDEV.H)
 *)

{ Change SOM_Resolve(o,ocn,mn) to...
  somTD_ocn_mn(somResolve(SOM_TestCls(o, ocnClassData.classObject), ocnClassData.mn)))

  Change SOM_ResolveNoCheck(o,ocn,mn) to...
  somTD_ocn_mn(somResolve(o,ocnClassData,mn))

  Change SOM_ParentNumResolveCC(pcn,pcp,ocn,mn) to...
  somTD_pcn_mn(somParentNumResolve(ocn_CClassData.parentMtab,pcp,pcnClassData.mn))

  Change SOM_ParentNumResolve(pcn,pcp,mtabs,mn) to...
  somTD_pcn_mn(somParentNumResolve(mtabs,pcp,pcnClassData.mn))

  Change SOM_ClassResolve(cn,class,mn) to...
  somTD_cn_mn(somClassResolve(class,cnClassData.mn))

  Change SOM_ResolveD(o,tdc,cdc,mn) to...
  somTD_tdc_mn(somResolve(SOM_TestCls(o,cdcClassData.classObject), cdcClassData.mn)))

  Change SOM_ParentResolveE(pcn,mtbls,mn) to...
  somTD_pcn_mn(somParentResolve(mtbls,pcnClassData.mn))

  Change SOM_DataResolve(obj,dataId) to...
  somDataResolve(obj, dataId)

  Change SOM_ClassLibrary(name) to...
  somRegisterClassLibrary(name,somMethodProc(SOMInitModule))
}

type
  SOMClassCClassDataStructure   = record
    parentMtab                  : somMethodTabs;
    instanceDataToken           : somDToken;
  end;

var
{$warning support of external vars required}
  SOMClassCClassData            : SOMClassCClassDataStructure;//³ 00002 ³ SOMClassCClassData

type
  SOMClassClassDataStructure    = record
    classObject                 : SOMClassType;
    somNew                      : somMToken;
    somRenew                    : somMToken;
    somInitClass                : somMToken;
    somClassReady               : somMToken;
    somGetName                  : somMToken;
    somGetParent                : somMToken;
    somDescendedFrom            : somMToken;
    somCheckVersion             : somMToken;
    somFindMethod               : somMToken;
    somFindMethodOk             : somMToken;
    somSupportsMethod           : somMToken;
    somGetNumMethods            : somMToken;
    somGetInstanceSize          : somMToken;
    somGetInstanceOffset        : somMToken;
    somGetInstancePartSize      : somMToken;
    somGetMethodIndex           : somMToken;
    somGetNumStaticMethods      : somMToken;
    somGetPClsMtab              : somMToken;
    somGetClassMtab             : somMToken;
    somAddStaticMethod          : somMToken;
    somOverrideSMethod          : somMToken;
    somAddDynamicMethod         : somMToken;
    somcPrivate0                : somMToken;
    somGetApplyStub             : somMToken;
    somFindSMethod              : somMToken;
    somFindSMethodOk            : somMToken;
    somGetMethodDescriptor      : somMToken;
    somGetNthMethodInfo         : somMToken;
    somSetClassData             : somMToken;
    somGetClassData             : somMToken;
    somNewNoInit                : somMToken;
    somRenewNoInit              : somMToken;
    somGetInstanceToken         : somMToken;
    somGetMemberToken           : somMToken;
    somSetMethodDescriptor      : somMToken;
    somGetMethodData            : somMToken;
    somOverrideMtab             : somMToken;
    somGetMethodToken           : somMToken;
    somGetParents               : somMToken;
    somGetPClsMtabs             : somMToken;
    somInitMIClass              : somMToken;
    somGetVersionNumbers        : somMToken;
    somLookupMethod             : somMToken;
    _get_somInstanceDataOffsets : somMToken;
    somRenewNoZero              : somMToken;
    somRenewNoInitNoZero        : somMToken;
    somAllocate                 : somMToken;
    somDeallocate               : somMToken;
    somGetRdStub                : somMToken;
    somGetNthMethodData         : somMToken;
    somcPrivate1                : somMToken;
    somcPrivate2                : somMToken;
    _get_somDirectInitClasses   : somMToken;
    _set_somDirectInitClasses   : somMToken;
    somGetInstanceInitMask      : somMToken;
    somGetInstanceDestructionMask : somMToken;
    somcPrivate3                : somMToken;
    somcPrivate4                : somMToken;
    somcPrivate5                : somMToken;
    somcPrivate6                : somMToken;
    somcPrivate7                : somMToken;
    somDefinedMethod            : somMToken;
    somcPrivate8                : somMToken;
    somcPrivate9                : somMToken;
    somcPrivate10               : somMToken;
    somcPrivate11               : somMToken;
    somcPrivate12               : somMToken;
    somcPrivate13               : somMToken;
    somcPrivate14               : somMToken;
    somcPrivate15               : somMToken;
    _get_somDataAlignment       : somMToken;
    somGetInstanceAssignmentMask  : somMToken;
    somcPrivate16               : somMToken;
    somcPrivate17               : somMToken;
    _get_somClassAllocate       : somMToken;
    _get_somClassDeallocate     : somMToken;
  end;

var
{$warning support of external vars required}
  SOMClassClassData             : SOMClassClassDataStructure;//³ 00003 ³ SOMClassClassData
{$warning support of external vars required}
  SOMClassMgrCClassData         : somCClassDataStructure;//³ 00005 ³ SOMClassMgrCClassData

type
  SOMClassMgrClassDataStructure = record
    classObject                 : SOMClassType;
    somFindClsInFile            : somMToken;
    somFindClass                : somMToken;
    somClassFromId              : somMToken;
    somRegisterClass            : somMToken;
    somUnregisterClass          : somMToken;
    somLocateClassFile          : somMToken;
    somLoadClassFile            : somMToken;
    somUnloadClassFile          : somMToken;
    somGetInitFunction          : somMToken;
    somMergeInto                : somMToken;
    somGetRelatedClasses        : somMToken;
    somSubstituteClass          : somMToken;
    _get_somInterfaceRepository : somMToken;
    _set_somInterfaceRepository : somMToken;
    _get_somRegisteredClasses   : somMToken;
    somBeginPersistentClasses   : somMToken;
    somEndPersistentClasses     : somMToken;
    somcmPrivate1               : somMToken;
    somcmPrivate2               : somMToken;
    somRegisterClassLibrary     : somMToken;
    somJoinAffinityGroup        : somMToken;
  end;

var
{$warning support of external vars required}
  SOMClassMgrClassData          : SOMClassMgrClassDataStructure;//³ 00006 ³ SOMClassMgrClassData

type
  SOMObjectCClassDataStructure  = record
    parentMtab                  :somMethodTabs;
    instanceDataToken           :somDToken;
  end;

var
{$warning support of external vars required}
  SOMObjectCClassData           : SOMObjectCClassDataStructure;//³ 00013 ³ SOMObjectCClassData

type
  SOMObjectClassDataStructure   = record
    classObject                 : SOMClassType;
    somInit                     : somMToken;
    somUninit                   : somMToken;
    somFree                     : somMToken;
    somDefaultVCopyInit         : somMToken;
    somGetClassName             : somMToken;
    somGetClass                 : somMToken;
    somIsA                      : somMToken;
    somRespondsTo               : somMToken;
    somIsInstanceOf             : somMToken;
    somGetSize                  : somMToken;
    somDumpSelf                 : somMToken;
    somDumpSelfInt              : somMToken;
    somPrintSelf                : somMToken;
    somDefaultConstVCopyInit    : somMToken;
    somDispatchV                : somMToken;
    somDispatchL                : somMToken;
    somDispatchA                : somMToken;
    somDispatchD                : somMToken;
    somDispatch                 : somMToken;
    somClassDispatch            : somMToken;
    somCastObj                  : somMToken;
    somResetObj                 : somMToken;
    somDefaultInit              : somMToken;
    somDestruct                 : somMToken;
    somPrivate1                 : somMToken;
    somPrivate2                 : somMToken;
    somDefaultCopyInit          : somMToken;
    somDefaultConstCopyInit     : somMToken;
    somDefaultAssign            : somMToken;
    somDefaultConstAssign       : somMToken;
    somDefaultVAssign           : somMToken;
    somDefaultConstVAssign      : somMToken;
  end;

var
{$warning support of external vars required}
  SOMObjectClassData            : SOMObjectClassDataStructure;//³ 00014 ³ SOMObjectClassData

(* Another not ported vars *)
// Control the printing of method and procedure entry messages,
// 0-none, 1-user, 2-core&user */
  SOM_TraceLevel: Longint; //³ 00018 ³ SOM_TraceLevel

// Control the printing of warning messages, 0-none, 1-all
  SOM_WarnLevel: Longint; //³ 00019 ³ SOM_WarnLevel

// Control the printing of successful assertions, 0-none, 1-user,
// 2-core&user
  SOM_AssertLevel: Longint; //³ 00017 ³ SOM_AssertLevel

// ToDo: Move this to corresponding place
Procedure somCheckArgs(argc: longint; argv: array of pchar); cdecl;
  external 'som' name 'somCheckArgs'; {index 25}
Procedure somUnregisterClassLibrary (libraryName: PChar); cdecl;
  external 'som' name 'somUnregisterClassLibrary'; {index 89}
Function somResolveTerminal(x : SOMClassPtr; mdata: somMToken): somMethodProcPtr; cdecl;
  external 'som' name 'somResolveTerminal'; {index 133}
Function somPCallResolve(obj: PSOMObject; callingCls: SOMClassPtr; method: somMToken): somMethodProcPtr; cdecl;
  external 'som' name 'somPCallResolve'; {index 362}
Function va_SOMObject_somDispatchA(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const): Pointer; cdecl;
  external 'som' name 'va_SOMObject_somDispatchA'; {index 64}
Function somva_SOMObject_somDispatchA(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const): Pointer; cdecl;
  external 'som' name 'somva_SOMObject_somDispatchA'; {index 96}
Function va_SOMObject_somDispatchL(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const): Longint; cdecl;
  external 'som' name 'va_SOMObject_somDispatchL'; {index 66}
Function somva_SOMObject_somDispatchL(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const): Longint; cdecl;
  external 'som' name 'somva_SOMObject_somDispatchL'; {index 98}

Function va_SOMObject_somDispatch(somSelf: PSOMObject;
                retValue: PsomToken;
                methodId: somId;
                args: array of const): Boolean; cdecl;
  external 'som' name 'va_SOMObject_somDispatch'; {index 68}

Procedure va_SOMObject_somDispatchV(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const); cdecl;
  external 'som' name 'va_SOMObject_somDispatchV'; {index 67}

Procedure somva_SOMObject_somDispatchV(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const); cdecl;
  external 'som' name 'somva_SOMObject_somDispatchV'; {index 99}

Function va_SOMObject_somDispatchD(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const): double; cdecl;
  external 'som' name 'va_SOMObject_somDispatchD'; {index 65}

Function somva_SOMObject_somDispatchD(somSelf: PSOMObject;
                methodId: somId;
                descriptor: somId;
                args: array of const): double; cdecl;
  external 'som' name 'somva_SOMObject_somDispatchD'; {index 97}
Function somva_SOMObject_somDispatch(somSelf: PSOMObject;
                retValue: PsomToken;
                methodId: somId;
                args: array of const): boolean; cdecl;
  external 'som' name 'somva_SOMObject_somDispatch'; {index 100}
Function somva_SOMObject_somClassDispatch(somSelf: PSOMObject;
                clsObj: PSOMClass;
                retValue: PsomToken;
                methodId: somId;
                args: array of const): boolean; cdecl;
  external 'som' name 'somva_SOMObject_somClassDispatch'; {index 101}

Implementation

Function exception_id(ev:Environment):PChar; cdecl;
begin
  Result := somExceptionId(ev)
end;

Function  exception_value(ev:Environment):Pointer; cdecl;
begin
  Result := somExceptionValue(ev)
end;

Procedure exception_free(ev:Environment); cdecl;
begin
  somExceptionFree(ev)
end;

Function SOM_CreateLocalEnvironment:Environment; cdecl;
begin
  Result:=SOMCalloc(1, sizeof(EnvironmentType))
end;

Procedure SOM_DestroyLocalEnvironment(ev:Environment); cdecl;
begin
  somExceptionFree(ev);
  if somGetGlobalEnvironment<>ev then SOMFree(ev);
end;

Procedure SOM_InitEnvironment(ev:Environment); cdecl;
begin
  if somGetGlobalEnvironment<>ev then FillChar(ev^,sizeof(EnvironmentType),0);
end;

Procedure SOM_UninitEnvironment(ev:Environment); cdecl;
begin
  somExceptionFree(ev);
end;


End.
(*
³ 00038 ³ somSaveMetrics // not found
³ 00046 ³ somWriteMetrics // not found
³ 00051 ³ somCreateDynamicClass // not found
³ 00056 ³ SOM_IdTable // not found
³ 00057 ³ SOM_IdTableSize // not found
³ 00062 ³ somStartCriticalSection // not found
³ 00063 ³ somEndCriticalSection // not found
³ 00080 ³ somfixMsgTemplate // not found
³ 00087 ³ SOMParentDerivedMetaclassClassData // not found
³ 00132 ³ somFreeThreadData // not found
³ 00135 ³ somIdMarshal  // not found
³ 00361 ³ somMakeUserRdStub // Not found
*)

