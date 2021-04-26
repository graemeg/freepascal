{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2020 by Michael Van Canneyt

    Pascal to Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************
}(*
Abstract:
  Converts TPasElements into TJSElements.

Works:
- units, programs
- unit interface function
- uses list
- use $impl for implementation declarations, can be disabled
- option to disable "use strict"
- interface vars
  - only double, no other float type
  - only string, no other string type
  - modifier public to protect from removing by optimizer
- implementation vars
- external vars
- initialization section
- procedures
  - params
  - local vars
  - default values
  - function results
  - modifier external 'name'
  - local const: declare in singleton parent function as local var
  - give procedure overloads in module unique names by appending $1, $2, ...
  - give nested procedure overloads unique names by appending $1, $2, ...
  - untyped parameter
  - varargs
  - modifier public to protect from removing by optimizer
  - choose overloads based on type and precision
  - fail overload on multiple with loss of precision or one used default param
  - FuncName:=, auto rename lower lvl Result variables
  - var modifier 'absolute' for local vars
- assign statements
- char
  - literals
  - ord(char)  ->  char.charCodeAt()
  - chr(integer)  -> String.fromCharCode(integer)
- string
  - literals
  - setlength(s,newlen) -> s = rtl.strSetLength(s,newlen)
  - read and write char aString[]
  - allow only String, no ShortString, AnsiString, UnicodeString,...
  - allow type casting string to external class name 'String'
- for int/enum do, for char do, for bool do
- repeat..until
- while..do
- try..finally
- try..except, try..except on else
- raise, raise E
- asm..end
- assembler; asm..end;
- break
- continue
- procedure str, function str
- type alias
- inc/dec to += -=
- case-of
- convert "a div b" to "Math.floor(a / b)"
- and, or, xor, not: logical and bitwise
- typecast boolean to integer and back with unary plus:  +bool  and int!=0
- rename name conflicts with js identifiers: apply, bind, call, prototype, ...
- record
  - types and vars
  - assign, copy members, not record reference, needed by ^record
  - assign record member
  - clone set member
  - clone static array member
  - clone when passing as argument
  - equal, not equal
  - const
  - array of record-const
  - skip clone record of new record
  - use rtl.recNewT to create a record type
  - use TRec.$new to instantiate records, using Object.create to instantiate
  - record field external name
  - advanced records:
    - public, private, strict private
    - class var
    - const member
    - sub types
    - functions
    - properties
    - class properties
    - default property
    - rtti
    - constructor
- assign: copy values, do not create new JS object, needed by ^record
- classes
  - declare using createClass
  - constructor
  - destructor
  - vars, init on create, clear references on destroy
  - class vars
  - ancestor
  - virtual, override, abstract
  - "is" operator
  - "as" operator
  - call inherited
    - "inherited;",
    - "inherited funcname(params);"
    - in nested proc
  - call class method
  - read/write class var
  - property
    - param list
    - property of type array
  - class property
    - accessors non static
  - Assigned()
  - default property
  - type casts
  - overloads, reintroduce  append $1, $2, ...
  - reintroduced variables
  - external vars and methods
  - const
  - bracket accessor, getter/setter has external name '[]'
  - TObject.Free sets variable to nil
  - property stored and index modifier
  - option verify method calls -CR, bsObjectChecks
- dynamic arrays
  - arrays can be null
  - init as "arr = []"  so typeof works
  - SetLength(arr,dim1,...) becomes  arr = rtl.arraySetLength(arr,defaultvalue,dim1,dim2,...)
  - length(), low(), high(), assigned(), concat()
  - assign nil -> []  so typeof works
  - read, write element arr[index]
  - multi dimensional [index1,index2] -> [index1][index2]
  - array of record
  - equal, unequal nil -> rtl.length(array)==0  or >0
  - when passing nil to an array argument, pass []
  - allow type casting array to external class name 'Array'
  - type cast array to array of same dimensions and compatible element type
  - function copy(array,start=0,count=max): array
  - procedure insert(item,var array,const position)
  - procedure delete(var array,const start,count)
  - const c: dynarray = (a,b,...)
  - mode delphi: var B: TBytes = [1,2,3]; // square bracket initialization
  - a:=[];
  - a:=[1,2,3]; // assignation using constant array
  - a:=[[],[]]  // nested constant array
  - string like operations: modeswitch arrayoperators  a:=A+[4,5];
  - Insert(Arr,MultiDimArr,0-based-pos);
  - a := Concat([1,2,3],[4,5,6]);
  - copy, concat for static arrays, creating dynamic arrays
- static arrays
  - range: enumtype, boolean, int, char, custom int
  - init as arr = rtl.arraySetLength(null,value,dim1,dim2,...)
  - init with expression
  - length(1-dim array)
  - low(1-dim array), high(1-dim array)
  - "=" operator for records with static array fields
  - of record
- open arrays
  - as dynamic arrays
- enums
  - type with values and names
  - option to write numbers instead of variables
  - ord(), low(), high(), pred(), succ(), str(), writestr()
  - type cast alias to enumtype
  - type cast number to enumtype, enumtype to number
  - const aliasname = enumvalue
- sets
  - set of enum
  - include, exclude, clone when referenced
  - assign :=   set state referenced
  - constant set: enums, enum vars, ranges
  - set operators +, -, *, ><, =, <>, >=, <=
  - in-operator
  - low(), high()
  - when passing as argument set state referenced
  - set of (enum,enum2)  - anonymous enumtype
  - set of char, boolean, integer range, char range, enum range
- with-do  using local var
  - with record do i:=v;
  - with classinstance do begin create; i:=v; f(); i:=a[]; end;
- pass by reference
  - pass local var to a var/out parameter
  - pass variable to a var/out parameter
  - pass reference to a var/out parameter
  - pass array element to a var/out parameter
- procedure types
  - implemented as immutable wrapper function
  - assign := nil, proctype (not clone), @function, @method
  - call  explicit and implicit
  - compare equal and notequal with nil, proctype, address, function
  - assigned(proctype)
  - pass as argument
  - methods
  - mode delphi: proctype:=proc
  - mode delphi: functype=funcresulttype
  - nested functions
  - reference to
  - @@ compare method in delphi mode
- class-of
  - assign :=   nil, var
  - call class method
  - call constructor
  - operators =, <>
  - class var, property, method
  - Self in class method
  - typecast
- class external
  - JS object or function as ancestor
  - does not descend from TObject
  - all members become external. case sensitive
  - has no hidden values like $class, $ancestor, $unitname, $init, $final
  - can be ancestor of a pascal class (not descend from TObject).
  - pascal class descendant can override methods
  - property works as normal, replaced by getter and setter
  - class-of
  - class var/function: works as in JS.
  - is and as operators
  - destructor forbidden
  - constructor must not be virtual
  - constructor 'new' -> new extclass(params)
  - constructor Name -> new extclass.name(params)
  - constructor Name external name '{}' -> {}
  - identifiers are renamed to avoid clashes with external names
  - call inherited
  - Pascal descendant can override newinstance
  - any class can be typecasted to any root class
  - class instances cannot access external class members (e.g. static class functions)
  - external class 'Array' bracket operator [integer] type jsvalue
  - external class 'Object' bracket operator [string] type jsvalue
  - typecast class type to JS Object, e.g. TJSObject(TObject)
  - typecast record type to JS Object, e.g. TJSObject(TPoint)
  - typecast interface type to JS Object, e.g. TJSObject(IUnknown)
  - for i in tjsobject do
  - nested classes
- jsvalue
  - init as undefined
  - assign to jsvalue := integer, string, boolean, double, char
  - type cast base types to jsvalue
  - type cast jsvalue to base type
     integer: Math.floor(jsvalue)   may return NaN
     boolean: !(jsvalue == false)   works for numbers too 0==false
     double: rtl.getNumber(jsvalue)    typeof(n)=="number"?n:NaN;
     string: ""+jsvalue
     char: rtl.getChar(jsvalue)   ((typeof(c)!="string") && (c.length==1)) ? c : ""
  - enums: assign to jsvalue, typecast jsvalue to enum
  - class instance: assign to jsvalue, typecast jsvalue to a class
  - class of: assign to jsvalue, typecast jsvalue to a class-of
  - array of jsvalue,
    allow to assign any array to an array of jsvalue
    allow type casting to any array
  - parameter, result type, assign from/to untyped
  - operators equal, not equal
  - callback: assign to jsvalue, equal, not equal
  - jsvalue is class-type, jsvalue is class-of-type
  - for i in jsvalue do
- RTTI
  - base types
  - $mod.$rtti
  - enum type tkEnumeration
  - set type tkSet
  - procedure type  tkProcVar, tkMethod
  - class type tkClass
    - fields,
    - methods,
    - properties  no params, no index, no defaultvalue
    - class forward
  - class-of type tkClassRef
  - dyn array type  tkDynArray
  - static array type  tkArray
  - record type  tkRecord
  - no typeinfo for local types
  - built-in function typeinfo(): Pointer/TTypeInfo/...;
    - typeinfo(class) -> class.$rtti
  - WPO skip not used typeinfo
  - open array param
  - property stored and index modifier
  - property default value, nodefault
- pointer
  - compare with and assign nil
  - typecast class, class-of, interface, array
- ECMAScript6:
  - use 0b for binary literals
  - use 0o for octal literals
- dotted unit names, namespaces
- resourcestring
- custom ranges
  - enum, int, char
  - low(), high(), pred(), succ(), ord(),
  - rg(int), int(rg), enum:=rg,
  - rg:=rg, rg1:=rg2, rg:=enum, =, <>,
  - set of int/enum/char range, in
  - array[rg], low(array), high(array), length(array)
- enumeration  for..in..do
  - enum, enum range, set of enum, set of enum range
  - int, int range, set of int, set of int range
  - char, char range, set of char, set of char range
  - array
  - class
  - for key in JSObject do
  - for value in JSArray do
- Assert(bool[,string])
  - without sysutils: if(!bool) throw string
  - with sysutils: if(!bool) throw pas.sysutils.EAssertionFailed.$create("Create",[string])
- Object checks:
  - Method call EInvalidCast, rtl.checkMethodCall
  - type cast to class-type and class-of-type, rtl.asExt, EInvalidCast
- Range checks:
  - compile time: warnings to errors
  - assign int:=, int+=, enum:=, enum+=, intrange:=, intrange+=,
      enumrange:=, enumrange+=, char:=, char+=, charrange:=, charrange+=
  - procedure argument int, enum, intrange, enumrange, char, charrange
  - array[index1,index2,...]  read and assign
  - string[index]  read and assign
- Interfaces:
  - autogenerate GUID
  - method resolution
  - delegation, property implements: intf or object, field or function,
    class field, class function
  - default property
  - Assigned(intfvar)
  - TGUID record
    - GuidVar:='{guid}', StringVar:=GuidVar, GuidVar:=IntfTypeOrVar,
    - GuidVar=IntfTypeOrVar, GuidVar=s
    - pass IntfTypeOrVar to GuidVar parameter
  - TGUIDString
    - GuidString:=IntfTypeOrVar, GuidString=IntfTypeOrVar
    - pass IntfTypeOrVar to GuidString parameter
  - CORBA: IntfVar:=nil, IntfVar:=IntfVar, IntfVar:=ObjVar;
  - CORBA: IntfVar=IntfVar2, IntfVar<>IntfVar2,
  - CORBA: IntfVar is IBird, IntfVar is TBird, ObjVar is IBird
  - CORBA: IntfVar2 as IBird, IntfVar2 as TBird, ObjVar as IBird
  - CORBA: IntfVar:=IBird(IntfVar2);',
  - CORBA: pass IntfVar as argument, pass classinstvar to intf argument
  - CORBA: IEnumerable
  - COM: IntfVar:=nil, IntfVar:=IntfVar, IntfVar:=ObjVar, IntfArg:=, IntfLocalVar:=
  - COM: IntfVar=IntfVar2, IntfVar<>IntfVar2,
  - COM: IntfVar is IBird, IntfVar is TBird, ObjVar is IBird
  - COM: IntfVar2 as IBird, IntfVar2 as TBird, ObjVar as IBird
  - COM: IntfVar:=IBird(IntfVar2);',
  - COM: pass IntfVar as argument, pass classinstvar to intf argument
  - COM: function result, release on exception
  - COM: addref/release for function call in expression
  - COM: delegation
  - COM: property in class, property in interface
  - COM: with interface do
  - COM: for interface in ... do
  - COM: pass IntfVar to untyped parameter
- currency:
  - as nativeint*10000
  - CurA+CurB -> CurA+CurB
  - CurA-CurB -> CurA-CurB
  - CurA*CurB -> CurA*CurB/10000
  - CurA/CurB -> Math.floor(CurA/CurB*10000)
  - CurA^^CurB -> Math.floor(Math.pow(CurA/10000,CurB/10000)*10000)
  - Double:=Currency  ->  Double:=Currency/10000
  - Currency:=Double  -> Currency:=Math.floor(Double*10000)
  - jsvalue := currency  ->  jsvalue:=currency/10000
- simplify Math.floor(constnumber) to truncated constnumber
- Pointer of record
  - p:=@r, p^:=r
  - p^.x, p.x
  - dispose, new
- typecast byte(longword) -> value & $ff
- typecast TJSFunction(func)
- modeswitch OmitRTTI
- debugger;
- anonymous functions
  - assign
  - pass as argument
- procedure val(const string; var enumtype; out int)
- move all local types to global
- class helpers:
  - ancestor
  - class var, const, sub type
  - method, class method, static class method
  - call methods, @method
  - constructor, not for external class
  - inherited, inherited name
  - property, class property
  - for in
- record helpers:
  - in function allow assign Self
- type helpers:
  - pass var, const, read only const
  - pass arg default, arg const, arg var, arg out
  - pass result element
  - pass function result
  - pass field, class field
  - pass property getter field, property getter function,
  - pass class property, static class property
  - pass array property
- array of const, TVarRec
- attributes
- overflow check:
  -Co   : Overflow checking of integer operations
- generics
- async procedure modifier
- function await(const expr: T): T
- function await(T; p: TJSPromise): T
- constref
- generics

ToDos:
- range check:
   type helper self:=
- cmd line param to set modeswitch
- Result:=inherited;
- asm-block annotate/reference
  - pas()  test or use or read or write
  - trailing [,,,]
- bug: DoIt(typeinfo(i))  where DoIt is in another unit and has TTypeInfo
- $OPTIMIZATION ON|OFF
- $optimization REMOVEEMPTYPROCS
- $optimization REMOVEEMPTYPROCS,RemoveNotUsedDeclarations-
- static arrays
  - clone multi dim static array
- RTTI
  - class property field/static/nonstatic
- interfaces
  - array of interface
  - record member interface
- 1 as TEnum, ERangeError
- ifthen<T>
- stdcall of methods: pass original 'this' as first parameter
- property read Arr[0]  https://bugs.freepascal.org/view.php?id=33416
- write, writeln
- call array of proc element without ()
- enums with custom values
- library
- option overflow checking -Co
  +, -, *, Succ, Pred, Inc, Dec
  -CO   : Check for possible overflow of integer operations
-C3 : Turn on ieee error checking for constants
- optimizations:
  see https://wiki.lazarus.freepascal.org/Pas2js_optimizations
- objects
- operator overloading
  - operator enumerator
- inline
- extended RTTI

Debugging this unit: -d<x>
   VerbosePas2JS
*)
unit FPPas2Js;

{$mode objfpc}{$H+}
{$inline on}

{$ifdef fpc}
  {$define UsePChar}
  {$define HasInt64}
{$endif}

interface

uses
  {$ifdef pas2js}
  {$else}
  AVL_Tree,
  {$endif}
  Classes, SysUtils, math, contnrs,
  jsbase, jstree, jswriter,
  PasTree, PScanner, PasResolveEval, PasResolver;

// message numbers
const
  nPasElementNotSupported = 4001;
  nNotSupportedX = 4002;
  nUnaryOpcodeNotSupported = 4003;
  nBinaryOpcodeNotSupported = 4004;
  nInvalidNumber = 4005;
  nInitializedArraysNotSupported = 4006;
  nMemberExprMustBeIdentifier = 4007;
  nCantWriteSetLiteral = 4008;
  nInvalidAbsoluteLocation = 4009;
  nForInJSArrDefaultGetterNotExtBracketAccessor = 4010;
  nInvalidFunctionReference = 4011;
  nMissingExternalName = 4012;
  nVirtualMethodNameMustMatchExternal = 4013;
  nPublishedNameMustMatchExternal = 4014;
  nInvalidVariableModifier = 4015;
  nAWaitOnlyInAsyncProcedure = 4016;
  nNewInstanceFunctionMustBeVirtual = 4017;
  nNewInstanceFunctionMustHaveTwoParameters = 4018;
  nNewInstanceFunctionMustNotHaveOverloadAtX = 4019;
  nBracketAccessorOfExternalClassMustHaveOneParameter = 4020;
  nTypeXCannotBePublished = 4021;
  nNestedInheritedNeedsParameters = 4022;
  nFreeNeedsVar = 4023;
  nDuplicateGUIDXInYZ = 4024;
  nCantCallExtBracketAccessor = 4025;
  nJSNewNotSupported = 4026;
  nHelperClassMethodForExtClassMustBeStatic = 4027;
  nBitWiseOperationIs32Bit = 4028;
  nDuplicateMessageIdXAtY = 4029;
  nDispatchRequiresX = 4030;
  nConstRefNotForXAsConst = 4031;
  nSymbolCannotBeExportedFromALibrary = 4032;
// resourcestring patterns of messages
resourcestring
  sPasElementNotSupported = 'Pascal element not supported: %s';
  sNotSupportedX = 'Not supported: %s';
  sUnaryOpcodeNotSupported = 'Unary OpCode not yet supported "%s"';
  sBinaryOpcodeNotSupported = 'Binary OpCode not yet supported "%s"';
  sInvalidNumber = 'Invalid number "%s"';
  sInitializedArraysNotSupported = 'Initialized array variables not yet supported';
  sMemberExprMustBeIdentifier = 'Member expression must be an identifier';
  sCantWriteSetLiteral = 'Cannot write set literal';
  sInvalidAbsoluteLocation = 'Invalid absolute location';
  sForInJSArrDefaultGetterNotExtBracketAccessor = 'for-in-JS-array needs as default getter an external bracket accessor';
  sInvalidFunctionReference = 'Invalid function reference';
  sMissingExternalName = 'Missing external name';
  sVirtualMethodNameMustMatchExternal = 'Virtual method name must match external';
  sInvalidVariableModifier = 'Invalid variable modifier "%s"';
  sPublishedNameMustMatchExternal = 'Published name must match external';
  sAWaitOnlyInAsyncProcedure = 'await only available in async procedure';
  sNewInstanceFunctionMustBeVirtual = 'NewInstance function must be virtual';
  sNewInstanceFunctionMustHaveTwoParameters = 'NewInstance function must have two parameters';
  sNewInstanceFunctionMustNotHaveOverloadAtX = 'NewInstance function must not have overload at %s';
  sBracketAccessorOfExternalClassMustHaveOneParameter = 'Bracket accessor of external class must have one parameter';
  sTypeXCannotBePublished = 'Type "%s" cannot be published';
  sNestedInheritedNeedsParameters = 'nested inherited needs parameters';
  sFreeNeedsVar = 'Free needs a variable';
  sDuplicateGUIDXInYZ = 'Duplicate GUID %s in %s and %s';
  sCantCallExtBracketAccessor = 'cannot call external bracket accessor, use a property instead';
  sJSNewNotSupported = 'Pascal class does not support the "new" constructor';
  sHelperClassMethodForExtClassMustBeStatic = 'Helper class method for external class must be static';
  sBitWiseOperationIs32Bit = 'Bitwise operation is 32-bit';
  sDuplicateMessageIdXAtY = 'Duplicate message id "%s" at %s';
  sDispatchRequiresX = 'Dispatch requires %s';
  sConstRefNotForXAsConst = 'ConstRef not yet implemented for %s. Treating as Const';
  sSymbolCannotBeExportedFromALibrary = 'The symbol cannot be exported from a library';

const
  ExtClassBracketAccessor = '[]'; // external name '[]' marks the array param getter/setter
  IsExtModePasClassInstance = 1;
  IsExtModePasClass = 2;
  LocalVarHide = '-';

type
  TPas2JSBuiltInName = (
    // functions
    pbifnArray_Concat,
    pbifnArray_ConcatN,
    pbifnArray_Copy,
    pbifnArray_Equal,
    pbifnArray_Insert,
    pbifnArray_Length,
    pbifnArray_Reference,
    pbifnArray_SetLength,
    pbifnArray_Static_Clone,
    pbifnAs,
    pbifnAsExt,
    pbifnBitwiseLongwordFix,
    pbifnBitwiseNativeIntAnd,
    pbifnBitwiseNativeIntOr,
    pbifnBitwiseNativeIntShl,
    pbifnBitwiseNativeIntShr,
    pbifnBitwiseNativeIntXor,
    pbifnCheckMethodCall,
    pbifnCheckVersion,
    pbifnClassAncestorFunc,
    pbifnClassInstanceFree,
    pbifnClassInstanceNew,
    pbifnClassInitSpecialize,
    pbifnCreateClass,
    pbifnCreateClassExt,
    pbifnCreateHelper,
    pbifnGetChar,
    pbifnGetNumber,
    pbifnGetObject,
    pbifnGetResourcestring,
    pbifnHelperNew,
    pbifnIntf_AddRef,
    pbifnIntf_Release,
    pbifnIntfAddMap,
    pbifnIntfAsClass,
    pbifnIntfAsIntfT, // COM intfvar as intftype
    pbifnIntfCreate,
    pbifnIntfCreateTGUID,
    pbifnIntfExprRefsAdd,
    pbifnIntfExprRefsCreate,
    pbifnIntfExprRefsFree,
    pbifnIntfGetGUIDR,
    pbifnIntfGetIntfT,
    pbifnIntfGuidRToStr,
    pbifnIntfIsClass,
    pbifnIntfIsIntf, // COM intfvar is intftype
    pbifnIntfToClass,
    pbifnIntfSetIntfL,
    pbifnIntfSetIntfP,
    pbifnIntfStrToGUIDR,
    pbifnIntfQueryIntfIsT,
    pbifnIntfQueryIntfT,
    pbifnIs,
    pbifnIsExt,
    pbifnFloatToStr,
    pbifnValEnum,
    pbifnFreeLocalVar,
    pbifnFreeVar,
    pbifnLibraryMain,
    pbifnOverflowCheckInt,
    pbifnProcType_Create,
    pbifnProcType_CreateSafe,
    pbifnProcType_Equal,
    pbifnProgramMain,
    pbifnRaiseException, // rtl.raiseE
    pbifnRangeCheckArrayRead,
    pbifnRangeCheckArrayWrite,
    pbifnRangeCheckChar,
    pbifnRangeCheckInt,
    pbifnRangeCheckGetCharAt,
    pbifnRangeCheckSetCharAt,
    pbifnRecordAssign,
    pbifnRecordClone,
    pbifnRecordCreateType,
    pbifnRecordEqual,
    pbifnRecordNew,
    pbifnRTTIAddField, // typeinfos of tkclass and tkrecord have addField
    pbifnRTTIAddFields, // typeinfos of tkclass and tkrecord have addFields
    pbifnRTTIAddMethod,//   "   "
    pbifnRTTIAddProperty,//   "   "
    pbifnRTTIInherited, // typeinfo for type alias type $inherited
    pbifnRTTINewClass,// typeinfo creator of tkClass $Class
    pbifnRTTINewClassRef,// typeinfo of tkClassRef $ClassRef
    pbifnRTTINewDynArray,// typeinfo of tkDynArray $DynArray
    pbifnRTTINewEnum,// typeinfo of tkEnumeration $Enum
    pbifnRTTINewExtClass,// typeinfo creator of tkExtClass $ExtClass
    pbifnRTTINewInt,// typeinfo of tkInt $Int
    pbifnRTTINewInterface,// typeinfo creator of tkInterface $Interface
    pbifnRTTINewMethodVar,// typeinfo of tkMethod $MethodVar
    pbifnRTTINewPointer,// typeinfo of tkPointer $Pointer
    pbifnRTTINewProcSig,// rtl.newTIProcSig
    pbifnRTTINewProcVar,// typeinfo of tkProcVar $ProcVar
    pbifnRTTINewRecord,// typeinfo creator of tkRecord $Record
    pbifnRTTINewRefToProcVar,// typeinfo of tkRefToProcVar $RefToProcVar
    pbifnRTTINewSet,// typeinfo of tkSet $Set
    pbifnRTTINewStaticArray,// typeinfo of tkArray $StaticArray
    pbifnSetCharAt,
    pbifnSet_Clone,
    pbifnSet_Create,
    pbifnSet_Difference,
    pbifnSet_Equal,
    pbifnSet_Exclude,
    pbifnSet_GreaterEqual,
    pbifnSet_Include,
    pbifnSet_Intersect,
    pbifnSet_LowerEqual,
    pbifnSet_NotEqual,
    pbifnSet_Reference,
    pbifnSet_SymDiffSet,
    pbifnSet_Union,
    pbifnSpaceLeft,
    pbifnStringSetLength,
    pbifnTrunc, // rtl.trunc
    pbifnUnitInit,
    // variables
    pbivnExceptObject,
    pbivnIntfExprRefs,
    pbivnIntfGUID,
    pbivnIntfKind,
    pbivnIntfMaps,
    pbivnImplementation,
    pbivnImplCode,
    pbivnMessageInt,
    pbivnMessageStr,
    pbivnLibrary, // library
    pbivnLocalModuleRef,
    pbivnLocalProcRef,
    pbivnLocalTypeRef,
    pbivnLoop,
    pbivnLoopEnd,
    pbivnLoopIn,
    pbivnModule,
    pbivnModules,
    pbivnPtrClass,
    pbivnPtrRecord,
    pbivnProcOk,
    pbivnProgram,  // program
    pbivnResourceStrings,
    pbivnResourceStringOrig,
    pbivnRTL,
    pbivnRTTI, // $rtti
    pbivnRTTIArray_Dims,
    pbivnRTTIArray_ElType,
    pbivnRTTIClassRef_InstanceType,
    pbivnRTTIEnum_EnumType,
    pbivnRTTIInt_MaxValue,
    pbivnRTTIInt_MinValue,
    pbivnRTTIInt_OrdType,
    pbivnRTTILocal, // $r
    pbivnRTTIMemberAttributes, // attr
    pbivnRTTIMethodKind, // tTypeInfoMethodVar has methodkind
    pbivnRTTIPointer_RefType, // reftype
    pbivnRTTIProcFlags, // flags
    pbivnRTTIProc_InitSpec, // init
    pbivnRTTIProcVar_ProcSig, // procsig
    pbivnRTTIPropDefault, // Default
    pbivnRTTIPropIndex, // index
    pbivnRTTIPropStored, // stored
    pbivnRTTISet_CompType, // comptype
    pbivnRTTITypeAttributes, // attr
    pbivnRTTIExtClass_Ancestor, // ancestor
    pbivnRTTIExtClass_JSClass, // jsclass
    pbivnSelf,
    pbivnTObjectDestroy,
    pbivnWith,
    // types
    pbitnAnonymousPostfix,
    pbitnIntDouble,
    pbitnTI,
    pbitnTIClass,
    pbitnTIClassRef,
    pbitnTIDynArray,
    pbitnTIEnum,
    pbitnTIExtClass,
    pbitnTIHelper,
    pbitnTIInteger,
    pbitnTIInterface,
    pbitnTIMethodVar,
    pbitnTIPointer,
    pbitnTIProcVar,
    pbitnTIRecord,
    pbitnTIRefToProcVar,
    pbitnTISet,
    pbitnTIStaticArray,
    pbitnUIntDouble
    );

const
  Pas2JSBuiltInNames: array[TPas2JSBuiltInName] of string = (
    'arrayConcat', // rtl.arrayConcat    pbifnArray_Concat
    'arrayConcatN', // rtl.arrayConcatN   pbifnArray_ConcatN
    'arrayCopy', // rtl.arrayCopy      pbifnArray_Copy
    'arrayEq', // rtl.arrayEq          pbifnArray_Equal
    'arrayInsert', // rtl.arrayCopy      pbifnArray_Insert
    'length', // rtl.length    pbifnArray_Length
    'arrayRef', // rtl.arrayRef  pbifnArray_Reference
    'arraySetLength', // rtl.arraySetLength  pbifnArray_SetLength
    '$clone', // pbifnArray_Static_Clone
    'as', // rtl.as  pbifnAs
    'asExt', // rtl.asExt  pbifnAsExt
    'lw', // pbifnBitwiseLongwordFix
    'and', // pbifnBitwiseNativeIntAnd,
    'or', // pbifnBitwiseNativeIntOr,
    'shl', // pbifnBitwiseNativeIntShl,
    'shr', // pbifnBitwiseNativeIntShr,
    'xor', // pbifnBitwiseNativeIntXor,
    'checkMethodCall', // pbifnCheckMethodCall
    'checkVersion', // pbifnCheckVersion
    '$ancestorfunc', // pbifnClassAncestorFunc
    '$destroy', // pbifnClassInstanceFree
    '$create', // pbifnClassInstanceNew
    '$initSpec', // pbifnClassInitSpecialize
    'createClass', // pbifnCreateClass   rtl.createClass
    'createClassExt', // pbifnCreateClassExt  rtl.createClassExt
    'createHelper', // pbifnCreateHelper  rtl.createHelper
    'getChar', // rtl.getChar
    'getNumber', // rtl.getNumber
    'getObject', // rtl.getObject
    'getResStr', // rtl.getResStr
    '$new', // helpertype.$new
    '_AddRef', // rtl._AddRef
    '_Release', // rtl._Release
    'addIntf', // rtl.addIntf  pbifnIntfAddMap
    'intfAsClass', // rtl.intfAsClass
    'intfAsIntfT', // rtl.intfAsIntfT
    'createInterface', // rtl.createInterface
    'createTGUID', // rtl.createTGUID
    'ref', // $ir.ref  pbifnIntfExprRefsAdd
    'createIntfRefs', // rtl.createIntfRefs
    'free', // $ir.free
    'getIntfGUIDR', // rtl.getIntfGUIDR
    'getIntfT',   // rtl.getIntfT
    'guidrToStr', // rtl.guidrToStr
    'intfIsClass', // rtl.intfIsClass
    'intfIsIntfT', // rtl.intfIsIntfT
    'intfToClass', // rtl.intfToClass
    'setIntfL', // rtl.setIntfL
    'setIntfP', // rtl.setIntfP
    'strToGUIDR', // rtl.strToGUIDR
    'queryIntfIsT', // rtl.queryIntfIsT
    'queryIntfT', // rtl.queryIntfT
    'is', // pbifnIs  rtl.is
    'isExt', // pbifnIsExt  rtl.isExt
    'floatToStr', // pbifnFloatToStr  rtl.floatToStr
    'valEnum', // pbifnValEnum  rtl.valEnum
    'freeLoc', // pbifnFreeLocalVar  rtl.freeLoc
    'free', // pbifnFreeVar  rtl.free
    '$main', // pbifnLibraryMain
    'oc', //  pbifnOverflowCheckInt rtl.oc
    'createCallback', // pbifnProcType_Create  rtl.createCallback
    'createSafeCallback', // pbifnProcType_CreateSafe  rtl.createSafeCallback
    'eqCallback', // pbifnProcType_Equal  rtl.eqCallback
    '$main', // pbifnProgramMain
    'raiseE', // pbifnRaiseException  rtl.raiseE
    'rcArrR',  // pbifnRangeCheckArrayRead  rtl.rcArrR
    'rcArrW',  // pbifnRangeCheckArrayWrite  rtl.rcArrW
    'rcc', // pbifnRangeCheckChar  rtl.rcc
    'rc',  // pbifnRangeCheckInt  rtl.rc
    'rcCharAt',  // pbifnRangeCheckGetCharAt  rtl.rcCharAt
    'rcSetCharAt',  // pbifnRangeCheckSetCharAt  rtl.rcSetCharAt
    '$assign', // pbifnRecordAssign
    '$clone', // pbifnRecordClone
    'recNewT', // pbifnRecordCreateType
    '$eq', // pbifnRecordEqual
    '$new', // pbifnRecordNew
    'addField', // pbifnRTTIAddField
    'addFields', // pbifnRTTIAddFields
    'addMethod', // pbifnRTTIAddMethod
    'addProperty', // pbifnRTTIAddProperty
    '$inherited', // pbifnRTTIInherited
    '$Class', // pbifnRTTINewClass  tkClass
    '$ClassRef', // pbifnRTTINewClassRef
    '$DynArray', // pbifnRTTINewDynArray
    '$Enum', // pbifnRTTINewEnum
    '$ExtClass', // pbifnRTTINewExtClass
    '$Int', // pbifnRTTINewInt
    '$Interface', // pbifnRTTINewInterface
    '$MethodVar', // pbifnRTTINewMethodVar
    '$Pointer', // pbifnRTTINewPointer
    'newTIProcSig', // pbifnRTTINewProcSig
    '$ProcVar', // pbifnRTTINewProcVar
    '$Record', // pbifnRTTINewRecord
    '$RefToProcVar', // pbifnRTTINewRefToProcVar
    '$Set', // pbifnRTTINewSet
    '$StaticArray', // pbifnRTTINewStaticArray
    'setCharAt', // pbifnSetCharAt  rtl.setCharAt
    'cloneSet', // pbifnSet_Clone  rtl.cloneSet
    'createSet', // pbifnSet_Create  rtl.createSet [...]
    'diffSet', // pbifnSet_Difference  rtl.diffSet -
    'eqSet', // pbifnSet_Equal  rtl.eqSet =
    'excludeSet', // pbifnSet_Exclude  rtl.excludeSet
    'geSet', // pbifnSet_GreaterEqual  rtl.geSet superset >=
    'includeSet', // pbifnSet_Include  rtl.includeSet
    'intersectSet', // pbifnSet_Intersect  rtl.intersectSet *
    'leSet', // pbifnSet_LowerEqual  rtl.leSet subset <=
    'neSet', // pbifnSet_NotEqual  rtl.neSet <>
    'refSet', // pbifnSet_Reference  rtl.refSet
    'symDiffSet', // pbifnSet_SymDiffSet  rtl.symDiffSet >< (symmetrical difference)
    'unionSet', // pbifnSet_Union  rtl.unionSet +
    'spaceLeft', // pbifnSpaceLeft  rtl.spaceLeft
    'strSetLength', // pbifnStringSetLength  rtl.strSetLength
    'trunc', // pbifnTrunc
    '$init', // pbifnUnitInit
    '$e', // pbivnExceptObject
    '$ir',  // pbivnIntfExprRefs
    '$guid',// pbivnIntfGUID
    '$kind', // pbivnIntfKind
    '$intfmaps', // pbivnIntfMaps
    '$impl', // pbivnImplementation
    '$implcode', // pbivnImplCode
    '$msgint', // pbivnMessageInt
    '$msgstr', // pbivnMessageStr
    'library', //  pbivnLibrary  pas.library
    '$lm', // pbivnLocalModuleRef
    '$lp', // pbivnLocalProcRef
    '$lt', // pbivnLocalTypeRef
    '$l', // pbivnLoop
    '$end', // pbivnLoopEnd
    '$in', // pbivnLoopIn
    '$mod', // pbivnModule
    'pas', // pbivnModules
    '$class', // pbivnPtrClass, ClassType
    '$record', // pbivnPtrRecord, hidden recordtype
    '$ok', // pbivnProcOk
    'program', // pbivnProgram  pas.program
    '$resourcestrings', // pbivnResourceStrings
    'org', // pbivnResourceStringOrig
    'rtl', // pbivnRTL
    '$rtti', // pbivnRTTI
    'dims', // pbivnRTTIArray_Dims
    'eltype', // pbivnRTTIArray_ElType
    'instancetype', // pbivnRTTIClassRef_InstanceType
    'enumtype', // pbivnRTTIEnum_EnumType
    'maxvalue', // pbivnRTTIInt_MaxValue
    'minvalue', // pbivnRTTIInt_MinValue
    'ordtype', // pbivnRTTIInt_OrdType
    '$r', // pbivnRTTILocal
    'attr', // pbivnRTTIMemberAttributes
    'methodkind', // pbivnRTTIMethodKind
    'reftype', // pbivnRTTIPointer_RefType
    'flags', // pbivnRTTIProcFlags
    'init', // pbivnRTTIProc_InitSpec
    'procsig', // pbivnRTTIProcVar_ProcSig
    'Default', // pbivnRTTIPropDefault
    'index', // pbivnRTTIPropIndex
    'stored', // pbivnRTTIPropStored
    'comptype', // pbivnRTTISet_CompType
    'attr', // pbivnRTTITypeAttributes
    'ancestor', // pbivnRTTIExtClass_Ancestor
    'jsclass', // pbivnRTTIExtClass_JSClass
    '$Self', // pbivnSelf
    'tObjectDestroy', // pbivnTObjectDestroy rtl.tObjectDestroy
    '$with', // pbivnWith
    '$a', // pbitnAnonymousPostfix
    'NativeInt', // pbitnIntDouble
    'tTypeInfo', // pbitnTI
    'tTypeInfoClass', // pbitnTIClass
    'tTypeInfoClassRef', // pbitnTIClassRef
    'tTypeInfoDynArray', // pbitnTIDynArray
    'tTypeInfoEnum', // pbitnTIEnum
    'tTypeInfoExtClass', // pbitnTIExtClass
    'tTypeInfoHelper', // pbitnTIHelper
    'tTypeInfoInteger', // pbitnTIInteger
    'tTypeInfoInterface', // pbitnTIInterface
    'tTypeInfoMethodVar', // pbitnTIMethodVar
    'tTypeInfoPointer', // pbitnTIPointer
    'tTypeInfoProcVar', // pbitnTIProcVar
    'tTypeInfoRecord', // pbitnTIRecord
    'tTypeInfoRefToProcVar', // pbitnTIRefToProcVar
    'tTypeInfoSet', // pbitnTISet
    'tTypeInfoStaticArray', // pbitnTIStaticArray
    'NativeUInt' // pbitnUIntDouble
    );

  // reserved words, not usable as identifiers, not even as sub identifiers
  // pas2js will avoid name clashes, by changing the casing
  JSReservedWords: array[0..59] of string = (
     // keep sorted, first uppercase, then lowercase !
     '__extends',
     '_super',
     'anonymous',
     'apply',
     'array',
     'await',
     'bind',
     'break',
     'call',
     'case',
     'catch',
     'class',
     'constructor',
     'continue',
     'default',
     'delete',
     'do',
     'each',
     'else',
     'enum',
     'escape',
     'eval',
     'export',
     'extends',
     'false',
     'for',
     'function',
     'getPrototypeOf',
     'hasOwnProperty',
     'if',
     'implements',
     'import',
     'in',
     'instanceof',
     'interface',
     'isPrototypeOf',
     'let',
     'new',
     'null',
     'package',
     'private',
     'propertyIsEnumerable',
     'protected',
     'prototype',
     'public',
     'return',
     'static',
     'super',
     'switch',
     'this',
     'throw',
     'toLocaleString',
     'toString',
     'true',
     'try',
     'undefined',
     'var',
     'while',
     'with',
     'yield'
    );
  // reserved words, not usable as global identifiers, can be used as sub identifiers
  JSReservedGlobalWords: array[0..52] of string = (
     // keep sorted, first uppercase, then lowercase !
     'Array',
     'ArrayBuffer',
     'Boolean',
     'DataView',
     'Date',
     'Error',
     'EvalError',
     'Float32Array',
     'Float64Array',
     'FormData',
     'Generator',
     'GeneratorFunction',
     'Infinity',
     'Int16Array',
     'Int32Array',
     'Int8Array',
     'InternalError',
     'JSON',
     'Map',
     'Math',
     'NaN',
     'Number',
     'Object',
     'Promise',
     'Proxy',
     'RangeError',
     'ReferenceError',
     'Reflect',
     'RegExp',
     'Set',
     'String',
     'Symbol',
     'SyntaxError',
     'TypeError',
     'URIError',
     'Uint16Array',
     'Uint32Array',
     'Uint8Array',
     'Uint8ClampedArray',
     'WeakMap',
     'WeakSet',
     'arguments',
     'decodeURI',
     'decodeURIComponent',
     'encodeURI',
     'encodeURIComponent',
     'isFinite',
     'isNaN',
     'parseFloat',
     'parseInt',
     'unescape',
     'uneval',
     'valueOf'
    );

type

  { EPas2JS }

  EPas2JS = Class(Exception)
  public
    PasElement: TPasElement;
    MsgNumber: integer;
    Args: TMessageArgs;
    Id: TMaxPrecInt;
    MsgType: TMessageType;
  end;

type
  TPasToJsPlatform = (
    PlatformBrowser,
    PlatformNodeJS,
    PlatformElectron
    );
  TPasToJsPlatforms = set of TPasToJsPlatform;
const
  PasToJsPlatformNames: array[TPasToJsPlatform] of string = (
   'Browser',
   'NodeJS',
   'Electron'
    );
type
  TPasToJsProcessor = (
    ProcessorECMAScript5,
    ProcessorECMAScript6
    );
  TPasToJsProcessors = set of TPasToJsProcessor;
const
  PasToJsProcessorNames: array[TPasToJsProcessor] of string = (
   'ECMAScript5',
   'ECMAScript6'
    );

//------------------------------------------------------------------------------
// Pas2js built-in types
type
  TPas2jsBaseType = (
    pbtNone,
    pbtJSValue
    );
  TPas2jsBaseTypes = set of TPas2jsBaseType;

const
  Pas2jsBaseTypeNames: array[TPas2jsBaseType] of string = (
    'None',
    'JSValue'
    );

type
  TPas2jsBuiltInProc = (
    pbpDebugger,
    pbpAWait
    );
const
  Pas2jsBuiltInProcNames: array[TPas2jsBuiltInProc] of string = (
    'Debugger',
    'AWait'
    );

const
  ClassVarModifiersType = [vmClass,vmStatic];
  LowJSNativeInt = MinSafeIntDouble;
  HighJSNativeInt = MaxSafeIntDouble;
  LowJSBoolean = false;
  HighJSBoolean = true;

//------------------------------------------------------------------------------
// Element CustomData
type

  { TPas2JsElementData }

  TPas2JsElementData = Class(TPasElementBase)
  private
    FElement: TPasElement;
    procedure SetElement(const AValue: TPasElement);
  public
    Owner: TObject; // e.g. a TPasToJSConverter
    Next: TPas2JsElementData; // TPasToJSConverter uses this for its memory chain
    constructor Create; virtual;
    destructor Destroy; override;
    property Element: TPasElement read FElement write SetElement; // can be TPasElement
  end;
  TPas2JsElementDataClass = class of TPas2JsElementData;

  TPas2JSStoredLocalVar = class(TPasElementBase)
  public
    Name: string;
    Element: TPasElement;
  end;
  TPas2JSStoredLocalVarArray = array of TPas2JSStoredLocalVar;

  TPas2JSModuleScopeFlag = (
    p2msfPromiseSearched // TJSPromise searched
    );
  TPas2JSModuleScopeFlags = set of TPas2JSModuleScopeFlag;

  { TPas2JSModuleScope }

  TPas2JSModuleScope = class(TPasModuleScope)
  private
    FJSPromiseClass: TPasClassType;
    procedure SetJSPromiseClass(const AValue: TPasClassType);
  public
    FlagsJS: TPas2JSModuleScopeFlags;
    SystemVarRecs: TPasFunction;
    StoreJSLocalVars: TPas2JSStoredLocalVarArray; // only with coStoreImplJS
    procedure ClearStoreJSLocalVars;
    destructor Destroy; override;
    property JSPromiseClass: TPasClassType read FJSPromiseClass write SetJSPromiseClass;
  end;

  { TPas2jsElevatedLocals }

  TPas2jsElevatedLocals = class
  private
    FElevatedLocals: TPasResHashList; // list of TPasIdentifier, case insensitive
    procedure InternalAdd(Item: TPasIdentifier);
    procedure OnClear(Item, Dummy: pointer);
  public
    constructor Create;
    destructor Destroy; override;
    function Find(const Identifier: String): TPasIdentifier; inline;
    function Add(const Identifier: String; El: TPasElement): TPasIdentifier; virtual;
  end;

  { TPas2JSPrecompiledJS - Option coStoreImplJS }

  TPas2JSPrecompiledJS = class
  public
    BodyJS: string;
    EmptyJS: boolean; // true if Body.Body=nil
    GlobalJS: TStringList;
    ShortRefs: TFPList; // list of TPasElement needing a SectionContext.AddLocalVar
    procedure AddShortRef(El: TPasElement);
    destructor Destroy; override;
  end;

  { TPas2JSSectionScope
    JSElement is TJSSourceElements }

  TPas2JSSectionScope = class(TPasSectionScope)
  public
    ElevatedLocals: TPas2jsElevatedLocals;
    Renamed: boolean;
    constructor Create; override;
    destructor Destroy; override;
    procedure WriteElevatedLocals(Prefix: string); virtual;
  end;

  { TPas2JSInitialFinalizationScope }

  TPas2JSInitialFinalizationScope = class(TPasInitialFinalizationScope)
  public
    ImplJS: TPas2JSPrecompiledJS; // Option coStoreImplJS
    destructor Destroy; override;
  end;

  TMessageIdToProc_List = TStringList;

  { TPas2JSClassScope }

  TPas2JSClassScope = class(TPasClassScope)
  public
    JSName: string;
    NewInstanceFunction: TPasClassFunction;
    GUID: string;
    ElevatedLocals: TPas2jsElevatedLocals;
    MemberOverloadsRenamed: boolean;
    // Dispatch and message modifiers:
    DispatchField: String;
    DispatchStrField: String;
    MsgIntToProc, MsgStrToProc: TMessageIdToProc_List; // not stored by filer
  public
    constructor Create; override;
    destructor Destroy; override;
  end;

  { TPas2JSRecordScope }

  TPas2JSRecordScope = class(TPasRecordScope)
  public
    JSName: string;
    MemberOverloadsRenamed: boolean;
  end;

  { TPas2JSProcedureScope }

  TPas2JSProcedureScope = class(TPasProcedureScope)
  public
    OverloadName: string;
    JSName: string;
    ResultVarName: string; // valid in implementation ProcScope, empty means use ResolverResultVar
    BodyOverloadsRenamed: boolean;
    ImplJS: TPas2JSPrecompiledJS; // Option coStoreImplJS: stored in ImplScope
    procedure AddGlobalJS(const JS: string);
    destructor Destroy; override;
  end;

  { TPas2JSArrayScope }

  TPas2JSArrayScope = Class(TPasArrayScope)
  public
    JSName: string;
  end;

  { TPas2JSProcTypeScope }

  TPas2JSProcTypeScope = Class(TPasProcTypeScope)
  public
    JSName: string;
  end;

  { TPas2JSWithExprScope }

  TPas2JSWithExprScope = class(TPasWithExprScope)
  public
    WithVarName: string;
  end;

  { TPas2JSOverloadChgThisScope
    Dummy scope to signal a change of the "this" on the overload scope stack }

  TPas2JSOverloadChgThisScope = class(TPasIdentifierScope)
  end;

  { TResElDataPas2JSBaseType - CustomData for compiler built-in types (TPasUnresolvedSymbolRef), e.g. jsvalue }

  TResElDataPas2JSBaseType = class(TResElDataBaseType)
  public
    JSBaseType: TPas2jsBaseType;
  end;

//------------------------------------------------------------------------------
// TPas2JSResolver
const
  msAllPas2jsModeSwitchesReadOnly = [
    msClass,
    msResult,
    msRepeatForward,
    msInitFinal,
    msOut,
    msDefaultPara,
    msProperty,
    msExcept,
    msDefaultUnicodestring,
    msCBlocks
    ];
  msAllPas2jsModeSwitches = msAllPas2jsModeSwitchesReadOnly+[
    msDelphi,msObjfpc,
    msNestedComment,
    msAutoDeref,
    msHintDirective,
    msAdvancedRecords,
    msExternalClass,
    msTypeHelpers,
    msArrayOperators,
    msPrefixedAttributes,
    msOmitRTTI,
    msMultiHelpers,
    msImplicitFunctionSpec,
    msMultilineStrings];

  bsAllPas2jsBoolSwitchesReadOnly = [
    bsLongStrings
    ];
  bsAllPas2jsBoolSwitches = bsAllPas2jsBoolSwitchesReadOnly+[
    bsAssertions,
    bsRangeChecks,
    bsWriteableConst,
    bsTypeInfo,
    bsOverflowChecks,
    bsHints,
    bsNotes,
    bsWarnings,
    bsMacro,
    bsScopedEnums,
    bsObjectChecks
    ];

  vsAllPas2jsValueSwitchesReadOnly = [];
  vsAllPas2jsValueSwitches = vsAllPas2jsValueSwitchesReadOnly+[
    vsInterfaces,
    vsDispatchField,
    vsDispatchStrField
    ];

  // default parser+scanner options
  po_Pas2js = po_Resolver+[
    po_AsmWhole,
    po_ResolveStandardTypes,
    po_ExtConstWithoutExpr,
    po_StopOnUnitInterface,
    po_AsyncProcs];

  btAllJSBaseTypes = [
    btChar,
    btWideChar,
    btString,
    btUnicodeString,
    btDouble,
    btCurrency, // nativeint*10000 truncated
    btBoolean,
    btByteBool,
    btWordBool,
    btLongBool,
    btByte,
    btShortInt,
    btWord,
    btSmallInt,
    btLongWord,
    btLongint,
    btUIntDouble,
    btIntDouble,
    btPointer
    ];
  bfAllJSBaseProcs = bfAllStandardProcs;

  btAllJSStrings = [btString,btUnicodeString];
  btAllJSChars = [btChar,btWideChar];
  btAllJSStringAndChars = btAllJSStrings+btAllJSChars;
  btAllJSFloats = [btDouble];
  btAllJSBooleans = [btBoolean,btByteBool,btWordBool,btLongBool];
  btAllJSInteger = [btByte,btShortInt,btWord,btSmallInt,btLongWord,btLongint,
    btIntDouble,btUIntDouble,
    btCurrency  // in pas2js currency is more like an integer, instead of float
    ];
  btAllJSValueSrcTypes = [btNil,btUntyped,btPointer,btSet]+btAllJSInteger
      +btAllJSStringAndChars+btAllJSFloats+btAllJSBooleans;
  btAllJSValueTypeCastTo = btAllJSInteger
      +btAllJSStringAndChars+btAllJSFloats+btAllJSBooleans+[btPointer];
  btAllJSRangeCheckTypes = btAllJSInteger + btAllJSChars;
  btAllJSOverflowAddSubType = [btIntDouble,btUIntDouble,btCurrency];
  btAllJSOverflowMultType = [btLongWord,btLongint,btIntDouble,btUIntDouble,btCurrency];

  DefaultPasResolverOptions = [
    proFixCaseOfOverrides,
    proClassPropertyNonStatic,
    proPropertyAsVarParam,
    proClassOfIs,
    proExtClassInstanceNoTypeMembers,
    proOpenAsDynArrays,
    proProcTypeWithoutIsNested,
    proMethodAddrAsPointer,
    proSafecallAllowsDefault
    ];

type
  TPasToJsConverterOption = (
    coLowerCase, // lowercase all identifiers, except conflicts with JS reserved words
    coSwitchStatement, // convert case-of into switch instead of if-then-else
    coEnumNumbers, // use enum numbers instead of names
    coUseStrict,   // insert 'use strict'
    coNoTypeInfo,  // do not generate RTTI
    coEliminateDeadCode,  // skip code that is never executed
    coStoreImplJS,  // store references to JS code in procscopes
    coRTLVersionCheckMain, // insert rtl version check into main
    coRTLVersionCheckSystem, // insert rtl version check into system unit init
    coRTLVersionCheckUnit, // insert rtl version check into every unit init
    coShortRefGlobals // use short local variables for global identifiers
    );
  TPasToJsConverterOptions = set of TPasToJsConverterOption;
const
  DefaultPasToJSOptions = [coLowerCase];

type
  TPas2JSResolver = class;

  { TPas2jsPasScanner }

  TPas2jsPasScanner = class(TPascalScanner)
  private
    FCompilerVersion: string;
    FResolver: TPas2JSResolver;
    FTargetPlatform: TPasToJsPlatform;
    FTargetProcessor: TPasToJsProcessor;
  protected
    function HandleInclude(const Param: String): TToken; override;
    procedure DoHandleOptimization(OptName, OptValue: string); override;
  public
    GlobalConvOptsEnabled: TPasToJsConverterOptions;
    GlobalConvOptsDisabled: TPasToJsConverterOptions;
    function ReadNonPascalTillEndToken(StopAtLineEnd: boolean): TToken;
      override;
    property CompilerVersion: string read FCompilerVersion write FCompilerVersion;
    property Resolver: TPas2JSResolver read FResolver write FResolver;
    property TargetPlatform: TPasToJsPlatform read FTargetPlatform write FTargetPlatform;
    property TargetProcessor: TPasToJsProcessor read FTargetProcessor write FTargetProcessor;
  end;

  { TPas2JSResolverHub }

  TPas2JSResolverHub = class(TPasResolverHub)
  private
    FJSDelaySpecialize: TFPList;// list of TPasGenericType
    function GetJSDelaySpecializes(Index: integer): TPasGenericType;
  public
    constructor Create(TheOwner: TObject); override;
    destructor Destroy; override;
    procedure Reset; override;
    // delayed type specialization
    procedure AddJSDelaySpecialize(SpecType: TPasGenericType);
    function IsJSDelaySpecialize(SpecType: TPasGenericType): boolean;
    function JSDelaySpecializeCount: integer;
    property JSDelaySpecializes[Index: integer]: TPasGenericType read GetJSDelaySpecializes;
  end;

  { TPas2JSResolver }

  TPas2JSResolver = class(TPasResolver)
  private
    FJSBaseTypes: array[TPas2jsBaseType] of TPasUnresolvedSymbolRef;
    FJSBuiltInProcs: array[TPas2jsBuiltInProc] of TResElDataBuiltInProc;
    FExternalNames: TPasResHashList; // list of TPasIdentifier, case sensitive
    FFirstElementData, FLastElementData: TPas2JsElementData;
    function GetJSBaseTypes(aBaseType: TPas2jsBaseType): TPasUnresolvedSymbolRef; inline;
    function GetJSBuiltInProcs(aProc: TPas2jsBuiltInProc): TResElDataBuiltInProc; inline;
    procedure InternalAdd(Item: TPasIdentifier);
    procedure OnClearHashItem(Item, Dummy: pointer);
  protected
    type
      THasAnoFuncData = record
        Expr: TProcedureExpr;
      end;
      PHasAnoFuncData = ^THasAnoFuncData;
    procedure OnHasAnonymousEl(El: TPasElement; arg: pointer);
  protected
    type
      THasElReadingDeclData = record
        Decl: TPasElement;
        El: TPasElement;
      end;
      PHasElReadingDeclData = ^THasElReadingDeclData;
    procedure OnHasElReadingDecl(El: TPasElement; arg: pointer);
  protected
    type
      TPRFindExtSystemClass = record
        JSName: string;
        ErrorPosEl: TPasElement;
        Found: TPasClassType;
        ElScope: TPasScope; // Where Found was found
        StartScope: TPasScope; // where the search started
      end;
      PPRFindExtSystemClass = ^TPRFindExtSystemClass;
    procedure OnFindExtSystemClass(El: TPasElement; ElScope, StartScope: TPasScope;
      FindExtSystemClassData: Pointer; var Abort: boolean); virtual;
  protected
    // overloads: fix name clashes in JS
    FOverloadScopes: TFPList; // list of TPasIdentifierScope
    function HasOverloadIndex(El: TPasElement; WithElevatedLocal: boolean = false): boolean; virtual;
    function GetOverloadIndex(Identifier: TPasIdentifier;
      StopAt: TPasElement): integer;
    function GetOverloadAt(Identifier: TPasIdentifier; var Index: integer): TPasIdentifier;
    function GetOverloadIndex(El: TPasElement): integer;
    function GetOverloadAt(const aName: String; Index: integer): TPasIdentifier;
    function GetElevatedLocals(Scope: TPasScope): TPas2jsElevatedLocals;
    function RenameOverload(El: TPasElement): boolean;
    procedure RenameOverloadsInSection(aSection: TPasSection);
    procedure RenameOverloads(DeclEl: TPasElement; Declarations: TFPList);
    procedure RenameSubOverloads(Declarations: TFPList);
    procedure RenameMembers(El: TPasMembersType);
    procedure RenameSpecialized(SpecializedItem: TPRSpecializedItem);
    procedure PushOverloadScopeSkip;
    procedure PushOverloadScope(Scope: TPasIdentifierScope);
    function PushOverloadClassOrRecScopes(Scope: TPasClassOrRecordScope; WithParents: boolean): integer;
    procedure PopOverloadScope;
    procedure RestoreOverloadScopeLvl(OldScopeCount: integer);
    procedure ClearOverloadScopes;
  protected
    procedure AddType(El: TPasType); override;
    procedure AddRecordType(El: TPasRecordType; TypeParams: TFPList); override;
    procedure AddClassType(El: TPasClassType; TypeParams: TFPList); override;
    procedure AddEnumType(El: TPasEnumType); override;
    procedure ResolveImplAsm(El: TPasImplAsmStatement); override;
    procedure ResolveNameExpr(El: TPasExpr; const aName: string;
      Access: TResolvedRefAccess); override;
    procedure ResolveFuncParamsExpr(Params: TParamsExpr;
      Access: TResolvedRefAccess); override;
    procedure FinishInterfaceSection(Section: TPasSection); override;
    procedure FinishTypeSectionEl(El: TPasType); override;
    procedure FinishModule(CurModule: TPasModule); override;
    procedure FinishEnumType(El: TPasEnumType); override;
    procedure FinishSetType(El: TPasSetType); override;
    procedure FinishRecordType(El: TPasRecordType); override;
    procedure FinishClassType(El: TPasClassType); override;
    procedure FinishArrayType(El: TPasArrayType); override;
    procedure FinishAncestors(aClass: TPasClassType); override;
    procedure FinishVariable(El: TPasVariable); override;
    procedure FinishArgument(El: TPasArgument); override;
    procedure FinishProcedureType(El: TPasProcedureType); override;
    procedure FinishProperty(PropEl: TPasProperty); override;
    procedure FinishProcParamAccess(ProcType: TPasProcedureType;
      Params: TParamsExpr); override;
    procedure FinishPropertyParamAccess(Params: TParamsExpr; Prop: TPasProperty
      ); override;
    procedure FinishExportSymbol(El: TPasExportSymbol); override;
    procedure FindCreatorArrayOfConst(Args: TFPList; ErrorEl: TPasElement);
    function FindProc_ArrLitToArrayOfConst(ErrorEl: TPasElement): TPasFunction; virtual;
    function FindSystemExternalClassType(const aClassName, JSName: string;
      ErrorEl: TPasElement): TPasClassType; virtual;
    function FindTJSPromise(ErrorEl: TPasElement): TPasClassType; virtual;
    procedure CheckExternalClassConstructor(Ref: TResolvedReference); virtual;
    procedure CheckConditionExpr(El: TPasExpr;
      const ResolvedEl: TPasResolverResult); override;
    procedure CheckNewInstanceFunction(ClassScope: TPas2JSClassScope); virtual;
    function AddExternalName(const aName: string; El: TPasElement): TPasIdentifier; virtual;
    function FindExternalName(const aName: String): TPasIdentifier; virtual;
    procedure AddExternalPath(aName: string; El: TPasElement);
    procedure AddElevatedLocal(El: TPasElement); virtual;
    procedure ClearElementData; virtual;
    function GenerateGUID(El: TPasClassType): string; virtual;
    function CheckCallAsyncFuncResult(Param: TPasExpr; out ResolvedEl: TPasResolverResult): boolean; virtual;
  protected
    // generic/specialize
    procedure SpecializeGenericIntf(SpecializedItem: TPRSpecializedItem);
      override;
    procedure SpecializeGenericImpl(SpecializedItem: TPRSpecializedItem);
      override;
    procedure SpecializeProcedure(GenEl, SpecEl: TPasProcedure;
      SpecializedItem: TPRSpecializedItem); override;
    function SpecializeParamsNeedDelay(SpecializedItem: TPRSpecializedItem): TPasElement; virtual;
    function IsSpecializedNonStaticMethod(ProcType: TPasProcedureType): boolean;
  protected
    const
      cJSValueConversion = 2*cTypeConversion;
    // additional base types
    function AddJSBaseType(const aName: string; Typ: TPas2jsBaseType): TResElDataPas2JSBaseType;
    function CheckAssignCompatibilityCustom(const LHS,
      RHS: TPasResolverResult; ErrorEl: TPasElement;
      RaiseOnIncompatible: boolean; var Handled: boolean): integer; override;
    function CheckTypeCastClassInstanceToClass(const FromClassRes,
      ToClassRes: TPasResolverResult; ErrorEl: TPasElement): integer; override;
    function CheckEqualCompatibilityCustomType(const LHS,
      RHS: TPasResolverResult; ErrorEl: TPasElement;
      RaiseOnIncompatible: boolean): integer; override;
    function CheckForIn(Loop: TPasImplForLoop; const VarResolved,
      InResolved: TPasResolverResult): boolean; override;
    procedure ComputeUnaryNot(El: TUnaryExpr;
      var ResolvedEl: TPasResolverResult; Flags: TPasResolverComputeFlags);
      override;
    procedure ComputeBinaryExprRes(Bin: TBinaryExpr; out
      ResolvedEl: TPasResolverResult; Flags: TPasResolverComputeFlags;
      var LeftResolved, RightResolved: TPasResolverResult); override;
    // built-in functions
    function BI_Exit_OnGetCallCompatibility(Proc: TResElDataBuiltInProc;
      Expr: TPasExpr; RaiseOnError: boolean): integer; override;
    function BI_Val_OnGetCallCompatibility(Proc: TResElDataBuiltInProc;
      Expr: TPasExpr; RaiseOnError: boolean): integer; override;
    procedure BI_TypeInfo_OnGetCallResult(Proc: TResElDataBuiltInProc;
      Params: TParamsExpr; out ResolvedEl: TPasResolverResult); override;
    function BI_Debugger_OnGetCallCompatibility(Proc: TResElDataBuiltInProc;
      Expr: TPasExpr; RaiseOnError: boolean): integer; virtual;
    function BI_AWait_OnGetCallCompatibility(Proc: TResElDataBuiltInProc;
      Expr: TPasExpr; RaiseOnError: boolean): integer; virtual;
    procedure BI_AWait_OnGetCallResult(Proc: TResElDataBuiltInProc;
      Params: TParamsExpr; out ResolvedEl: TPasResolverResult); virtual;
    procedure BI_AWait_OnEval(Proc: TResElDataBuiltInProc;
      Params: TParamsExpr; Flags: TResEvalFlags; out Evaluated: TResEvalValue); virtual;
    procedure BI_AWait_OnFinishParamsExpr(Proc: TResElDataBuiltInProc;
      Params: TParamsExpr); virtual;
  public
    constructor Create; reintroduce;
    destructor Destroy; override;
    procedure ClearBuiltInIdentifiers; override;
    // base types
    function IsJSBaseType(TypeEl: TPasType; Typ: TPas2jsBaseType): boolean;
    function IsJSBaseType(const TypeResolved: TPasResolverResult;
      Typ: TPas2jsBaseType; HasValue: boolean = false): boolean;
    procedure AddObjFPCBuiltInIdentifiers(
      const TheBaseTypes: TResolveBaseTypes;
      const TheBaseProcs: TResolverBuiltInProcs); override;
    function CheckTypeCastRes(const FromResolved,
      ToResolved: TPasResolverResult; ErrorEl: TPasElement;
      RaiseOnError: boolean): integer; override;
    function FindLocalBuiltInSymbol(El: TPasElement): TPasElement; override;
    property JSBaseTypes[aBaseType: TPas2jsBaseType]: TPasUnresolvedSymbolRef read GetJSBaseTypes;
    property JSBuiltInProcs[aProc: TPas2jsBuiltInProc]: TResElDataBuiltInProc read GetJSBuiltInProcs;
    // compute literals and constants
    function ExtractPasStringLiteral(El: TPasElement; const S: String): TJSString; virtual;
    function ResolverToJSValue(Value: TResEvalValue; ErrorEl: TPasElement): TJSValue; virtual;
    function ComputeConstString(Expr: TPasExpr; StoreCustomData, NotEmpty: boolean): String; virtual;
    procedure CheckAssignExprRangeToCustom(
      const LeftResolved: TPasResolverResult; RValue: TResEvalValue;
      RHS: TPasExpr); override;
    function CheckAssignCompatibilityClasses(LType, RType: TPasClassType
      ): integer; override;
    function HasStaticArrayCloneFunc(Arr: TPasArrayType): boolean;
    function IsTGUID(TypeEl: TPasRecordType): boolean; override;
    function GetAssignGUIDString(TypeEl: TPasRecordType; Expr: TPasExpr; out GUID: TGuid): boolean;
    procedure CheckDispatchField(Proc: TPasProcedure; Switch: TValueSwitch);
    procedure AddMessageStr(var MsgToProc: TMessageIdToProc_List; const S: string; Proc: TPasProcedure);
    procedure AddMessageIdToClassScope(Proc: TPasProcedure; EmitHints: boolean); virtual;
    procedure ComputeResultElement(El: TPasResultElement; out
      ResolvedEl: TPasResolverResult; Flags: TPasResolverComputeFlags;
      StartEl: TPasElement = nil); override;
    // CustomData
    function GetElementData(El: TPasElementBase;
      DataClass: TPas2JsElementDataClass): TPas2JsElementData; virtual;
    procedure AddElementData(Data: TPas2JsElementData); virtual;
    function CreateElementData(DataClass: TPas2JsElementDataClass;
      El: TPasElement): TPas2JsElementData; virtual;
    // checking compatibilility
    function CheckEqualCompatibilityUserType(const LHS,
      RHS: TPasResolverResult; ErrorEl: TPasElement;
      RaiseOnIncompatible: boolean): integer; override;
    // utility
    procedure RaiseMsg(const Id: TMaxPrecInt; MsgNumber: integer; const Fmt: String;
      Args: array of {$IFDEF pas2js}jsvalue{$ELSE}const{$ENDIF}; ErrorPosEl: TPasElement); override;
    function GetOverloadName(El: TPasElement): string;
    function GetBaseDescription(const R: TPasResolverResult; AddPath: boolean=
      false): string; override;
    function HasTypeInfo(El: TPasType): boolean; override;
    function ProcHasImplElements(Proc: TPasProcedure): boolean; override;
    function HasAnonymousFunctions(El: TPasImplElement): boolean;
    function GetTopLvlProcScope(El: TPasElement): TPas2JSProcedureScope;
    function ProcCanBePrecompiled(DeclProc: TPasProcedure): boolean; virtual;
    function IsTObjectFreeMethod(El: TPasExpr): boolean; virtual;
    function IsExternalBracketAccessor(El: TPasElement): boolean;
    function IsExternalClassConstructor(El: TPasElement): boolean;
    function IsForInExtArray(Loop: TPasImplForLoop; const VarResolved,
      InResolved: TPasResolverResult; out ArgResolved, LengthResolved,
      PropResultResolved: TPasResolverResult): boolean;
    function IsHelperMethod(El: TPasElement): boolean; override;
    function IsHelperForMember(El: TPasElement): boolean; virtual;
    function ImplBlockReadsDecl(Block: TPasImplBlock; Decl: TPasElement): boolean; virtual;
  end;

//------------------------------------------------------------------------------
// TConvertContext
type
  TCtxAccess = (
    caRead,  // normal read
    caAssign, // needs setter, aContext.AccessContext is TAssignContext
    caByReference // needs path, getter and setter, aContext.AccessContext is TParamContext
    );
  TCtxVarKind = (
    cvkNone,
    cvkGlobal, // e.g. $mod, $impl, class type
    cvkCurType, // e.g. PasElement is a class, js var is the current class (Self in a class method)
    cvkInstance, // e.g. PasElement is a class, js var is the current instance (Self in method)
    cvkHelperTemp // e.g. helper-for getter/setter
    );
  TCtxVarKinds = set of TCtxVarKind;
const
  cvkAll = [cvkNone..cvkHelperTemp];

type
  TFunctionContext = Class;

  { TConvertContext }

  TConvertContextClass = Class of TConvertContext;
  TConvertContext = Class(TObject)
  public
    PasElement: TPasElement;
    JSElement: TJSElement;
    Resolver: TPas2JSResolver;
    Parent: TConvertContext;
    IsGlobal: boolean; // can hold constants and types
    Access: TCtxAccess;
    AccessContext: TConvertContext;
    TmpVarCount: integer;
    ScannerBoolSwitches: TBoolSwitches;
    ScannerModeSwitches: TModeSwitches;
    constructor Create(PasEl: TPasElement; JSEl: TJSElement; aParent: TConvertContext); virtual;
    function GetRootModule: TPasModule;
    function GetRootContext: TConvertContext;
    function GetNonDotContext: TConvertContext;
    function GetFunctionContext: TFunctionContext;
    function GetLocalName(El: TPasElement; const Filter: TCtxVarKinds): string; virtual;
    function GetSelfContext: TFunctionContext;
    function GetContextOfPasElement(El: TPasElement): TConvertContext;
    function GetFuncContextOfPasElement(El: TPasElement): TFunctionContext;
    function GetContextOfType(aType: TConvertContextClass): TConvertContext;
    function GetMainSectionContext: TFunctionContext;
    function CurrentModeSwitches: TModeSwitches;
    function GetGlobalFunc: TFunctionContext;
    procedure WriteStack;
    procedure DoWriteStack(Index: integer); virtual;
    function ToString: string; override;
  end;

  { TRootContext }

  TRootContext = Class(TConvertContext)
  public
    ResourceStrings: TJSVarDeclaration;
    GlobalClassMethods: TArrayOfPasProcedure;
    procedure AddGlobalClassMethod(p: TPasProcedure);
    destructor Destroy; override;
  end;

  { TFCLocalIdentifier }

  TFCLocalIdentifier = class
  public
    Element: TPasElement;
    Name: string;
    Kind: TCtxVarKind;
    constructor Create(const aName: string; TheEl: TPasElement; aKind: TCtxVarKind);
  end;
  TFCLocalVars = array of TFCLocalIdentifier;

  { TFunctionContext
    Module Function: PasElement is TPasProcedure (ImplProc), ThisPas=nil
    Method: PasElement is TPasProcedure (ImplProc), ThisPas is TPasMembersType }

  TFunctionContext = Class(TConvertContext)
  public
    LocalVars: TFCLocalVars;
    ThisVar: TFCLocalIdentifier;
    IntfElReleases: TFPList; // list of TPasElement, that needs rtl._Release(<El>)
    ResultNeedsIntfRelease: boolean;
    IntfExprReleaseCount: integer; // >0 means needs $ir
    BodySt: TJSElement;
    TrySt: TJSTryFinallyStatement;
    FinallyFirst, FinallyLast: TJSStatementList;
    constructor Create(PasEl: TPasElement; JSEl: TJSElement;
      aParent: TConvertContext); override;
    destructor Destroy; override;
    function AddLocalVar(aName: string; El: TPasElement; aKind: TCtxVarKind; AutoUnique: boolean): TFCLocalIdentifier;
    function AddLocalJSVar(aName: string; AutoUnique: boolean): TFCLocalIdentifier;
    procedure Add_InterfaceRelease(El: TPasElement);
    function CreateLocalIdentifier(const Prefix: string; El: TPasElement; aKind: TCtxVarKind): string; virtual;
    function ToString: string; override;
    function GetLocalName(El: TPasElement; const Filter: TCtxVarKinds): string; override;
    function IndexOfLocalVar(const aName: string): integer;
    function IndexOfLocalVar(El: TPasElement; const Filter: TCtxVarKinds): integer;
    function FindLocalVar(const aName: string; WithParents: boolean): TFCLocalIdentifier;
    function FindPrecompiledVar(const aName: string; WithParents: boolean): TPas2JSStoredLocalVar; virtual;
    function FindPrecompiledVar(El: TPasElement; WithParents: boolean): TPas2JSStoredLocalVar; virtual;
    procedure DoWriteStack(Index: integer); override;
  end;

  { TObjectContext }

  TObjectContext  = Class(TConvertContext)
  end;

  { TSectionContext - interface/implementation/program/library
    interface/program/library: PasElement is TPasModule, ThisPas is TPasModule
    implementation: PasElement is TImplementationSection, ThisPas=nil }

  TSectionContext = Class(TFunctionContext)
  public
    SrcElements: TJSSourceElements;
    HeaderIndex: integer; // index in TJSSourceElements(JSElement).Statements
    PrecompiledVars: TPas2JSStoredLocalVarArray; // copy from TPas2JSModuleScope, do not free
    constructor Create(PasEl: TPasElement; JSEl: TJSElement; aParent: TConvertContext); override;
    procedure AddHeaderStatement(JS: TJSElement);
    function FindPrecompiledVar(const aName: string; WithParents: boolean
      ): TPas2JSStoredLocalVar; override;
    function FindPrecompiledVar(El: TPasElement; WithParents: boolean
      ): TPas2JSStoredLocalVar; override;
  end;

  { TInterfaceSectionContext }

  TInterfaceSectionContext = Class(TSectionContext)
  public
    ImplContext: TSectionContext;
    ImplHeaderStatements: TFPList;
    ImplSrcElements: TJSSourceElements;
    ImplHeaderIndex: integer; // index in ImplSrcElements.Statements
    destructor Destroy; override;
    procedure AddImplHeaderStatement(JS: TJSElement);
  end;

  { TDotContext - used for converting eopSubIdent }

  TDotContext = Class(TConvertContext)
  public
    LeftResolved: TPasResolverResult;
    // created by ConvertElement if subident needs special translation:
    JS: TJSElement;
  end;

  { TAssignContext - used for left side of an assign statement }

  TAssignContext = Class(TConvertContext)
  public
    // set when creating:
    LeftResolved: TPasResolverResult;
    RightResolved: TPasResolverResult;
    RightSide: TJSElement;
    // created by ConvertElement if assign needs a call:
    PropertyEl: TPasProperty;
    Call: TJSCallExpression;
    constructor Create(PasEl: TPasElement; JSEl: TJSElement; aParent: TConvertContext); override;
  end;

  { TParamContext }

  TParamContext = Class(TConvertContext)
  public
    // set when creating:
    Arg: TPasArgument;
    Expr: TPasExpr;
    ResolvedExpr: TPasResolverResult;
    // created by ConvertElement:
    Setter: TJSElement;
    ReusingReference: boolean; // true = result is a reference, do not create another
    constructor Create(PasEl: TPasElement; JSEl: TJSElement; aParent: TConvertContext); override;
  end;

//------------------------------------------------------------------------------
// TPasToJSConverter
const
  DefaultJSWriterOptions = [
    {$IFDEF FPC_HAS_CPSTRING}
    woUseUTF8,
    {$ENDIF}
    woCompactArrayLiterals,
    woCompactObjectLiterals,
    woCompactArguments];

type

  { TPasToJSConverterGlobals - shared options }

  TPasToJSConverterGlobals = class
  private
    FOwner: TObject;
  public
    BuiltInNames: array[TPas2JSBuiltInName] of string;
    RTLVersion: TJSNumber;
    TargetPlatform: TPasToJsPlatform;
    TargetProcessor: TPasToJsProcessor;
    constructor Create(TheOwner: TObject);
    procedure Reset;
    procedure ResetBuiltInNames;
    property Owner: TObject read FOwner;
  end;

  TPas2JSIsElementUsedEvent = function(Sender: TObject; El: TPasElement): boolean of object;
  TJSReservedWordList = array of String;

  TRefPathKind = (
    rpkPath,      // e.g. "TObject"
    rpkPathWithDot, // e.g. "TObject."
    rpkPathAndName // e.g. "TObject.ClassName"
    );

  { TPasToJSConverter }

  TPasToJSConverter = Class(TObject)
  private
    // inline at ttop, because fpc 3.1 requires inline implementation in front of use
    function GetUseEnumNumbers: boolean; inline;
    function GetUseLowerCase: boolean; inline;
    function GetUseSwitchStatement: boolean; inline;
    function GetBIName(bin: TPas2JSBuiltInName): string; inline;
  private
    {$IFDEF EnableForLoopRunnerCheck}
    type
      TForLoopFindData = record
        ForLoop: TPasImplForLoop;
        LoopVar: TPasElement;
        FoundLoop: boolean;
        LoopVarWrite: boolean; // true if first access of LoopVar after loop is a write
        LoopVarRead: boolean; // true if first access of LoopVar after loop is a read
      end;
      PForLoopFindData = ^TForLoopFindData;
    procedure ForLoop_OnProcBodyElement(El: TPasElement; arg: pointer);
    {$ENDIF}
  private
    FGlobals: TPasToJSConverterGlobals;
    FOnIsElementUsed: TPas2JSIsElementUsedEvent;
    FOnIsTypeInfoUsed: TPas2JSIsElementUsedEvent;
    FOptions: TPasToJsConverterOptions;
    FReservedWords: TJSReservedWordList; // sorted with CompareStr
    Procedure AddGlobalClassMethod(aContext: TConvertContext; P: TPasProcedure);
    Function CreatePrimitiveDotExpr(Path: string; PosEl: TPasElement): TJSElement;
    Function CreateSubDeclJSNameExpr(El: TPasElement; JSName: string;
      AContext: TConvertContext; PosEl: TPasElement): TJSElement;
    Function CreateSubDeclPasNameExpr(El: TPasElement; const PasName: string;
      AContext: TConvertContext; PosEl: TPasElement = nil): TJSElement;
    Function CreateSubDeclNameExpr(El: TPasElement;
      AContext: TConvertContext; PosEl: TPasElement = nil): TJSElement;
    Function CreateIdentifierExpr(El: TPasElement; AContext: TConvertContext): TJSElement;
    Function CreateIdentifierExpr(AName: string; CheckGlobal: boolean; PosEl: TPasElement; AContext: TConvertContext): TJSElement;
    Function CreateSwitchStatement(El: TPasImplCaseOf; AContext: TConvertContext): TJSElement;
    Function CreateTypeDecl(El: TPasType; AContext: TConvertContext): TJSElement;
    Function CreateVarDecl(El: TPasVariable; AContext: TConvertContext): TJSElement;
    Procedure AddToSourceElements(Src: TJSSourceElements; El: TJSElement);
    procedure RemoveFromSourceElements(Src: TJSSourceElements;
      El: TJSElement);
    procedure SetGlobals(const AValue: TPasToJSConverterGlobals);
    procedure SetReservedWords(const AValue: TJSReservedWordList);
    procedure SetUseEnumNumbers(const AValue: boolean);
    procedure SetUseLowerCase(const AValue: boolean);
    procedure SetUseSwitchStatement(const AValue: boolean);
  protected
    type
      TMemberFunc = (mfInit, mfFinalize);
      TConvertJSEvent = function(El: TPasElement; AContext: TConvertContext; Data: Pointer): TJSElement of object;
      TCreateRefPathData = record
        El: TPasElement;
        Full: boolean;
        Ref: TResolvedReference;
      end;
      PCreateRefPathData = ^TCreateRefPathData;
    Function OnCreateReferencePathExpr(El: TPasElement; AContext : TConvertContext;
      CreateRefPathData: Pointer): TJSElement;
  protected
    // Error functions
    Procedure DoError(Id: TMaxPrecInt; Const Msg : String);
    Procedure DoError(Id: TMaxPrecInt; Const Msg : String;
      const Args: array of {$IFDEF pas2js}jsvalue{$ELSE}const{$ENDIF});
    Procedure DoError(Id: TMaxPrecInt; MsgNumber: integer; const MsgPattern: string;
      const Args: array of {$IFDEF pas2js}jsvalue{$ELSE}const{$ENDIF}; El: TPasElement);
    procedure RaiseNotSupported(El: TPasElement; AContext: TConvertContext; Id: TMaxPrecInt; const Msg: string = '');
    procedure RaiseIdentifierNotFound(Identifier: string; El: TPasElement; Id: TMaxPrecInt);
    procedure RaiseInconsistency(Id: TMaxPrecInt; El: TPasElement);
    // Computation, value conversions
    Function GetExpressionValueType(El: TPasExpr; AContext: TConvertContext ): TJSType; virtual;
    Function GetPasIdentValueType(AName: String; AContext: TConvertContext): TJSType; virtual;
    Function ComputeConstString(Expr: TPasExpr; AContext: TConvertContext; NotEmpty: boolean): String; virtual;
    Function IsLiteralInteger(El: TJSElement; out Number: TMaxPrecInt): boolean;
    Function IsLiteralNumber(El: TJSElement; out n: TJSNumber): boolean;
    // Name mangling
    Function GetOverloadName(El: TPasElement; AContext: TConvertContext): string;
    Function CanClashWithGlobal(El: TPasElement): boolean;
    Function TransformToJSName(ErrorEl: TPasElement; Const AName: String; CheckGlobal: boolean; AContext : TConvertContext): String; virtual;
    Function TransformElToJSName(El: TPasElement; AContext : TConvertContext) : String; virtual;
    Function TransformModuleName(El: TPasModule; AddModulesPrefix: boolean; AContext : TConvertContext) : String; virtual;
    Function IsReservedWord(const aName: string; CheckGlobal: boolean): boolean; virtual;
    Function GetTypeInfoName(El: TPasType; AContext: TConvertContext;
      ErrorEl: TPasElement; Full: boolean = false): String; virtual;
    Function TransformArgName(Arg: TPasArgument; AContext: TConvertContext): string; virtual;
    Function CreateGlobalAliasForeign(El: TPasElement; JSPath: string; AContext: TConvertContext): string; virtual; // El in other module
    Function CreateGlobalAliasNull(El: TPasElement; Prefix: TPas2JSBuiltInName;
      SectionContext: TSectionContext): TFCLocalIdentifier; virtual;
    Procedure CreateGlobalAlias_List(ElRefList: TFPList; AContext: TConvertContext); virtual;
    Function ElNeedsGlobalAlias(El: TPasElement): boolean; virtual;
    // utility functions for creating stuff
    Function IsElementUsed(El: TPasElement): boolean; virtual;
    Function IsSystemUnit(aModule: TPasModule): boolean; virtual;
    Function HasTypeInfo(El: TPasType; AContext: TConvertContext): boolean; virtual;
    Function IsClassRTTICreatedBefore(aClass: TPasClassType; Before: TPasElement; AConText: TConvertContext): boolean;
    Function IsExprTemporaryVar(Expr: TPasExpr): boolean; virtual;
    Function IsExprPropertySetterConst(Expr: TPasExpr; AContext: TConvertContext): boolean; virtual;
    Procedure FindAvailableLocalName(var aName: string; JSExpr: TJSElement);
    Function GetImplJSProcScope(El: TPasElement; Src: TJSSourceElements;
      AContext: TConvertContext): TPas2JSProcedureScope;
    Function SpecializeNeedsDelay(El: TPasGenericType; AContext: TConvertContext): boolean; virtual;
    // Never create an element manually, always use the below functions
    Function CreateElement(C: TJSElementClass; Src: TPasElement): TJSElement; virtual;
    Function CreateFreeOrNewInstanceExpr(Ref: TResolvedReference;
      AContext : TConvertContext): TJSCallExpression; virtual;
    Function CreateFunctionSt(El: TPasElement; WithBody: boolean = true;
      WithSrc: boolean = false): TJSFunctionDeclarationStatement;
    Function CreateFunctionDef(El: TPasElement; WithBody: boolean = true;
      WithSrc: boolean = false): TJSFuncDef;
    Procedure CreateProcedureCall(var Call: TJSCallExpression; Args: TParamsExpr;
      TargetProc: TPasProcedureType; AContext: TConvertContext); virtual;
    Procedure CreateProcedureCallArgs(Elements: TJSArrayLiteralElements;
      Args: TParamsExpr; TargetProc: TPasProcedureType; AContext: TConvertContext); virtual;
    Function CreateProcCallArg(El: TPasExpr; TargetArg: TPasArgument;
      AContext: TConvertContext): TJSElement; virtual;
    Function CreateProcCallArgRef(El: TPasExpr; ResolvedEl: TPasResolverResult;
      TargetArg: TPasArgument;  AContext: TConvertContext): TJSElement; virtual;
    Function CreateArrayEl(El: TPasExpr; JS: TJSElement; AContext: TConvertContext): TJSElement; virtual;
    Function CreateArgumentAccess(Arg: TPasArgument; AContext: TConvertContext;
      PosEl: TPasElement): TJSElement; virtual;
    Function CreateUnary(Members: array of string; E: TJSElement): TJSUnary;
    Function CreateUnaryPlus(Expr: TJSElement; El: TPasElement): TJSUnaryPlusExpression;
    Function CreateMemberExpression(Members: array of string): TJSElement;
    Function CreateCallExpression(El: TPasElement): TJSCallExpression;
    Function CreateCallCharCodeAt(Arg: TJSElement; aNumber: integer; El: TPasElement): TJSCallExpression; virtual;
    Function CreateCallFromCharCode(Arg: TJSElement; El: TPasElement): TJSCallExpression; virtual;
    Function CreateUsesList(UsesSection: TPasSection; AContext : TConvertContext): TJSArrayLiteral;
    Procedure AddToStatementList(var First, Last: TJSStatementList;
      Add: TJSElement; Src: TPasElement); overload;
    Procedure AddToStatementList(St: TJSStatementList; Add: TJSElement; Src: TPasElement); overload;
    Procedure PrependToStatementList(var St: TJSElement; Add: TJSElement; PosEl: TPasElement);
    Procedure AddToVarStatement(VarStat: TJSVariableStatement; Add: TJSElement;
      Src: TPasElement);
    Function CreateValInit(PasType: TPasType; Expr: TPasExpr; El: TPasElement;
      AContext: TConvertContext): TJSElement; virtual;
    Function CreateVarInit(El: TPasVariable; AContext: TConvertContext): TJSElement; virtual;
    Function CreateVarStatement(const aName: String; Init: TJSElement;
      El: TPasElement): TJSVariableStatement; virtual;
    Function CreateVarDecl(const aName: String; Init: TJSElement; El: TPasElement): TJSVarDeclaration; virtual;
    // JS literals
    Function CreateLiteralNumber(El: TPasElement; const n: TJSNumber): TJSLiteral; virtual;
    Function CreateLiteralHexNumber(El: TPasElement; const n: TMaxPrecInt; Digits: byte): TJSLiteral; virtual;
    Function CreateLiteralString(El: TPasElement; const s: string): TJSLiteral; virtual;
    Function CreateLiteralJSString(El: TPasElement; const s: TJSString): TJSLiteral; virtual;
    Function CreateLiteralBoolean(El: TPasElement; b: boolean): TJSLiteral; virtual;
    Function CreateLiteralNull(El: TPasElement): TJSLiteral; virtual;
    Function CreateLiteralUndefined(El: TPasElement): TJSLiteral; virtual;
    Function CreateLiteralCustomValue(El: TPasElement; const s: TJSString): TJSLiteral; virtual;
    Function CreateSetLiteralElement(Expr: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function CreateUnaryNot(El: TJSElement; Src: TPasElement): TJSUnaryNotExpression; virtual;
    Procedure ConvertCharLiteralToInt(Lit: TJSLiteral; ErrorEl: TPasElement; AContext: TConvertContext); virtual;
    Function ClonePrimaryExpression(El: TJSPrimaryExpression; Src: TPasElement): TJSPrimaryExpression;
    // simple JS expressions
    Function CreateMulNumber(El: TPasElement; JS: TJSElement; n: TMaxPrecInt): TJSElement; virtual;
    Function CreateDivideNumber(El: TPasElement; JS: TJSElement; n: TMaxPrecInt): TJSElement; virtual;
    Function CreateTruncFloor(El: TPasElement; JS: TJSElement; FloorAndCeil: boolean): TJSElement; virtual;
    Function CreateDotNameExpr(PosEl: TPasElement; MExpr: TJSElement;
      const aName: TJSString): TJSDotMemberExpression; virtual;
    Function CreateDotExpression(aParent: TPasElement; Left, Right: TJSElement;
      CheckRightIntfRef: boolean = false): TJSElement; virtual;
    // range and overflow checks
    Function CreateOverflowCheckCall(GetExpr: TJSElement; PosEl: TPasElement): TJSCallExpression; virtual;
    Function CreateRangeCheckCall(GetExpr: TJSElement; MinVal, MaxVal: TMaxPrecInt;
      RTLFunc: TPas2JSBuiltInName; PosEl: TPasElement): TJSCallExpression; virtual;
    Function CreateRangeCheckCall_TypeRange(aType: TPasType; GetExpr: TJSElement;
      AContext: TConvertContext; PosEl: TPasElement): TJSCallExpression; virtual;
    // reference
    Function CreateReferencePath(El: TPasElement; AContext: TConvertContext;
      Kind: TRefPathKind; Full: boolean = false; Ref: TResolvedReference = nil): string; virtual;
    Function CreateReferencePathExpr(El: TPasElement; AContext: TConvertContext;
      Full: boolean = false; Ref: TResolvedReference = nil): TJSElement; virtual;
    Function CreateGlobalTypePath(El: TPasType; AContext: TConvertContext): string; virtual;
    Function CreateStaticProcPath(El: TPasProcedure; AContext: TConvertContext): string; virtual;
    Function CreateGlobalElPath(El: TPasElement; AContext: TConvertContext): string; virtual;
    Function GetLocalName(El: TPasElement; const Filter: TCtxVarKinds; AContext: TConvertContext): string;
    Function ProcCanHaveShortRef(Proc: TPasProcedure): boolean;
    Procedure StoreImplJSLocal(El: TPasElement; AContext: TConvertContext); virtual;
    Procedure StoreImplJSLocals(ModScope: TPas2JSModuleScope; IntfContext: TSectionContext); virtual;
    Procedure RestoreImplJSLocals(ModScope: TPas2JSModuleScope; IntfContext: TSectionContext); virtual;
    // section
    Function CreateImplementationSection(El: TPasModule; IntfContext: TInterfaceSectionContext): TJSFunctionDeclarationStatement; virtual;
    Procedure CreateInitSection(El: TPasModule; Src: TJSSourceElements; AContext: TConvertContext); virtual;
    Procedure AddHeaderStatement(JS: TJSElement; PosEl: TPasElement; aContext: TConvertContext); virtual;
    Procedure AddImplHeaderStatement(JS: TJSElement; PosEl: TPasElement; aContext: TConvertContext); virtual;
    function AddDelayedInits(El: TPasModule; Src: TJSSourceElements; AContext: TConvertContext): boolean; virtual;
    function CreateDelaySpecializeInit(El: TPasGenericType; AContext: TConvertContext): TJSElement; virtual;
    // enum and sets
    Function CreateReferencedSet(El: TPasElement; SetExpr: TJSElement): TJSElement; virtual;
    // record
    Function CreateRecordInit(aRecord: TPasRecordType; Expr: TPasExpr;
      El: TPasElement; AContext: TConvertContext): TJSElement; virtual;
    Function CreateRecordCallNew(PosEl: TPasElement; RecTypeEl: TPasRecordType;
      AContext: TConvertContext): TJSCallExpression; virtual;
    Function CreateRecordCallClone(PosEl: TPasElement; RecTypeEl: TPasRecordType;
      RecordExpr: TJSElement; AContext: TConvertContext): TJSCallExpression; virtual;
    Function CreateRecordFunctionNew(El: TPasRecordType; AContext: TConvertContext;
      Fields: TFPList): TJSElement; virtual;
    Function CreateRecordFunctionEqual(El: TPasRecordType; AContext: TConvertContext;
      Fields: TFPList): TJSElement; virtual;
    Function CreateRecordFunctionAssign(El: TPasRecordType; AContext: TConvertContext;
      Fields: TFPList): TJSElement; virtual;
    Procedure CreateRecordRTTI(El: TPasRecordType; Src: TJSSourceElements;
      FuncContext: TFunctionContext; MembersSrc: TJSSourceElements;
      MembersFuncContext: TFunctionContext); virtual;
    Function CreateDelayedInitMembersFunction(PosEl: TPasElement; Src: TJSSourceElements;
      FuncContext: TFunctionContext; out DelaySrc: TJSSourceElements): TFunctionContext; virtual;
    // array
    Function CreateArrayConcat(ElTypeResolved: TPasResolverResult; PosEl: TPasElement;
      AContext: TConvertContext): TJSCallExpression; overload; virtual;
    Function CreateArrayConcat(ArrayType: TPasArrayType; PosEl: TPasElement;
      AContext: TConvertContext): TJSCallExpression; overload; virtual;
    Function CreateArrayInit(ArrayType: TPasArrayType; Expr: TPasExpr;
      El: TPasElement; AContext: TConvertContext): TJSElement; virtual;
    Function CreateArrayRef(El: TPasElement; ArrayExpr: TJSElement): TJSElement; virtual;
    Function CreateCmpArrayWithNil(El: TPasElement; JSArray: TJSElement;
      OpCode: TExprOpCode): TJSElement; virtual;
    Function CreateCloneStaticArray(El: TPasElement; ArrTypeEl: TPasArrayType;
      ArrayExpr: TJSElement; AContext: TConvertContext): TJSElement; virtual;
    // class
    Procedure AddClassConDestructorFunction(El: TPasClassType; Src: TJSSourceElements;
      ClassContext: TConvertContext; IsTObject: boolean; Ancestor: TPasType;
      Kind: TMemberFunc); virtual;
    Procedure AddClassRTTI(El: TPasClassType; Src: TJSSourceElements;
      FuncContext: TFunctionContext); virtual;
    Procedure AddClassConstructors(FuncContext: TFunctionContext; PosEl: TPasElement); virtual;
    Procedure AddClassMessageIds(El: TPasClassType; Src: TJSSourceElements;
      FuncContext: TFunctionContext; pbivn: TPas2JSBuiltInName); virtual;
    // misc
    Function CreateCallback(Expr: TPasExpr; ResolvedEl: TPasResolverResult;
      aSafeCall: boolean; AContext: TConvertContext): TJSElement; virtual;
    Function CreateSafeCallback(Expr: TPasExpr; JS: TJSElement; AContext: TConvertContext): TJSElement; virtual;
    Function CreateExternalBracketAccessorCall(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function CreateAssignStatement(LeftEl: TPasExpr; AssignContext: TAssignContext): TJSElement; virtual;
    Function CreateGetEnumeratorLoop(El: TPasImplForLoop;
      AContext: TConvertContext): TJSElement; virtual;
    Function CreateCallRTLFreeLoc(Setter, Getter: TJSElement; Src: TPasElement): TJSElement; virtual;
    Function CreatePropertyGet(Prop: TPasProperty; Expr: TPasExpr;
      AContext: TConvertContext; PosEl: TPasElement): TJSElement; virtual;
    Function AppendPropertyAssignArgs(Call: TJSCallExpression; Prop: TPasProperty;
      AssignContext: TAssignContext; PosEl: TPasElement): TJSCallExpression; virtual;
    Function AppendPropertyReadArgs(Call: TJSCallExpression; Prop: TPasProperty;
      aContext: TConvertContext; PosEl: TPasElement): TJSCallExpression; virtual;
    Function CreateDotSplit(El: TPasElement; Expr: TJSElement): TJSElement; virtual;
    Function CreatePrecompiledJS(El: TJSElement): string; virtual;
    Function CreateRaisePropReadOnly(PosEl: TPasElement): TJSElement; virtual;
    Procedure AddRTLVersionCheck(FuncContext: TFunctionContext; PosEl: TPasElement);
    // create elements for RTTI
    Function CreateTypeInfoRef(El: TPasType; AContext: TConvertContext;
      ErrorEl: TPasElement): TJSElement; virtual;
    Function CreateRTTIArgList(Parent: TPasElement; Args: TFPList;
      AContext: TConvertContext): TJSElement; virtual;
    Procedure AddRTTIArgument(Arg: TPasArgument; TargetParams: TJSArrayLiteral;
      AContext: TConvertContext); virtual;
    Function GetClassBIName(El: TPasClassType; AContext: TConvertContext): string; virtual;
    Function CreateRTTINewType(El: TPasType; const CallFuncName: string;
      IsForward: boolean; AContext: TConvertContext; out ObjLit: TJSObjectLiteral): TJSCallExpression; virtual;
    Function CreateRTTIAttributes(const Attr: TPasExprArray; PosEl: TPasElement; aContext: TConvertContext): TJSElement; virtual;
    Function CreateRTTIMemberField(Members: TFPList; Index: integer;
      AContext: TConvertContext): TJSElement; virtual;
    Function CreateRTTIMemberMethod(Members: TFPList; Index: integer;
      AContext: TConvertContext): TJSElement; virtual;
    Function CreateRTTIMemberProperty(Members: TFPList; Index: integer;
      AContext: TConvertContext): TJSElement; virtual;
    Procedure CreateRTTIAnonymous(El: TPasType; AContext: TConvertContext); virtual; // needed by precompiled files from 2.0.0
    Function CreateRTTIAnonymousArray(El: TPasArrayType; AContext: TConvertContext): TJSCallExpression; virtual;
    Function CreateRTTIMembers(El: TPasMembersType; Src: TJSSourceElements;
      FuncContext: TFunctionContext; MembersSrc: TJSSourceElements;
      MembersFuncContext: TFunctionContext; RTTIExpr: TJSElement;
      NeedLocalVar: boolean): boolean; virtual;
    // create elements for interfaces
    Procedure AddIntfDelegations(ClassEl: TPasElement; Prop: TPasProperty;
      FinishedGUIDs: TStringList; ObjLit: TJSObjectLiteral; aContext: TFunctionContext);
    Function CreateGUIDObjLit(aTGUIDRecord: TPasRecordType; const GUID: TGUID;
      PosEl: TPasElement; AContext: TConvertContext): TJSObjectLiteral;
    Function CreateAssignComIntfVar(const LeftResolved: TPasResolverResult;
      var LHS, RHS: TJSElement; AContext: TConvertContext; PosEl: TPasElement): TJSElement; virtual;
    Function IsInterfaceRef(Expr: TJSElement): boolean;
    Function CreateIntfRef(Expr: TJSElement; aContext: TConvertContext;
      PosEl: TPasElement): TJSCallExpression; virtual;
    Function RemoveIntfRef(Call: TJSCallExpression; AContext: TConvertContext): TJSElement;
    Procedure CreateFunctionTryFinally(FuncContext: TFunctionContext);
    Procedure AddFunctionFinallySt(NewEl: TJSElement; PosEl: TPasElement;
      FuncContext: TFunctionContext);
    Procedure AddFunctionFinallyRelease(SubEl: TPasElement; FuncContext: TFunctionContext);
    Procedure AddInFrontOfFunctionTry(NewEl: TJSElement; PosEl: TPasElement;
      FuncContext: TFunctionContext);
    Procedure AddInterfaceReleases(FuncContext: TFunctionContext; PosEl: TPasElement);
    Procedure AddClassSupportedInterfaces(El: TPasClassType; Src: TJSSourceElements;
      FuncContext: TFunctionContext);
    // create elements for helpers
    Function CreateCallHelperMethod(Proc: TPasProcedure; Expr: TPasExpr;
      AContext: TConvertContext; Implicit: boolean = false): TJSCallExpression; virtual;
    Procedure AddHelperConstructor(El: TPasClassType; Src: TJSSourceElements;
      AContext: TConvertContext); virtual;
    // Statements
    Function ConvertImplBlockElements(El: TPasImplBlock; AContext: TConvertContext; NilIfEmpty: boolean): TJSElement; virtual;
    Function ConvertBeginEndStatement(El: TPasImplBeginBlock; AContext: TConvertContext; NilIfEmpty: boolean): TJSElement; virtual;
    Function ConvertStatement(El: TPasImplStatement; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertAssignStatement(El: TPasImplAssign; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertRaiseStatement(El: TPasImplRaise; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertIfStatement(El: TPasImplIfElse; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertWhileStatement(El: TPasImplWhileDo; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertRepeatStatement(El: TPasImplRepeatUntil; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertForStatement(El: TPasImplForLoop; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertFinalizationSection(El: TFinalizationSection; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertInitializationSection(El: TPasModule; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertSimpleStatement(El: TPasImplSimple; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertWithStatement(El: TPasImplWithDo; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertTryStatement(El: TPasImplTry; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertExceptOn(El: TPasImplExceptOn; AContext: TConvertContext): TJSElement;
    Function ConvertCaseOfStatement(El: TPasImplCaseOf; AContext: TConvertContext): TJSElement;
    Function ConvertAsmStatement(El: TPasImplAsmStatement; AContext: TConvertContext): TJSElement;
    // Expressions
    Function ConvertConstValue(Value: TResEvalValue; AContext: TConvertContext; El: TPasElement): TJSElement; virtual;
    Function ConvertArrayValues(El: TArrayValues; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertInheritedExpr(El: TInheritedExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertNilExpr(El: TNilExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertCharToInt(Arg: TJSElement; PosEl: TPasElement; ArgContext: TConvertContext): TJSElement; virtual;
    Function ConvertIntToInt(Arg: TJSElement; FromBT, ToBT: TResolverBaseType; PosEl: TPasElement; ArgContext: TConvertContext): TJSElement; virtual;
    Function CreateBitWiseAnd(El: TPasElement; Value: TJSElement; const Mask: TMaxPrecInt; Shift: integer): TJSElement; virtual;
    Function CreateBitWiseLongword(El: TPasElement; Value: TJSElement): TJSElement; virtual;
    Function ConvertParamsExpr(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertArrayParams(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertFuncParams(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertExternalConstructor(Left: TPasExpr; Ref: TResolvedReference;
      ParamsExpr: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertTObjectFree_Bin(Bin: TBinaryExpr; NameExpr: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertTObjectFree_With(NameExpr: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertTypeCastToBaseType(El: TParamsExpr; AContext: TConvertContext; ToBaseTypeData: TResElDataBaseType): TJSElement; virtual;
    Function ConvertArrayOrSetLiteral(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Length(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_SetLength(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_ExcludeInclude(El: TParamsExpr; AContext: TConvertContext; IsInclude: boolean): TJSElement; virtual;
    Function ConvertBuiltInContinue(El: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltInBreak(El: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Exit(El: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_IncDec(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Assigned(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Chr(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Ord(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_LowHigh(El: TParamsExpr; AContext: TConvertContext; IsLow: boolean): TJSElement; virtual;
    Function ConvertBuiltIn_PredSucc(El: TParamsExpr; AContext: TConvertContext; IsPred: boolean): TJSElement; virtual;
    Function ConvertBuiltIn_StrProc(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_StrFunc(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltInStrParam(El: TPasExpr; AContext: TConvertContext; IsStrFunc, IsFirst: boolean): TJSElement; virtual;
    Function ConvertBuiltIn_WriteStr(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Val(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_LoHi(El: TParamsExpr; AContext: TConvertContext; IsLoFunc: Boolean): TJSElement; virtual;
    Function ConvertBuiltIn_ConcatArray(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_ConcatString(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_CopyArray(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_InsertArray(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_DeleteArray(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_TypeInfo(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_GetTypeKind(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Assert(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_New(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Dispose(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Default(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_Debugger(El: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBuiltIn_AWait(El: TParamsExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertRecordValues(El: TRecordValues; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertSelfExpression(El: TSelfExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBinaryExpression(El: TBinaryExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertBinaryExpressionRes(El: TBinaryExpr; AContext: TConvertContext;
      const LeftResolved, RightResolved: TPasResolverResult; var A,B: TJSElement): TJSElement; virtual;
    function ConvertBinaryExpressionMultiAdd(El: TBinaryExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertSubIdentExpression(El: TBinaryExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertSubIdentExprCustom(El: TBinaryExpr; AContext: TConvertContext;
      const OnConvertRight: TConvertJSEvent = nil; Data: Pointer = nil): TJSElement; virtual;
    Function ConvertBoolConstExpression(El: TBoolConstExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertPrimitiveExpression(El: TPrimitiveExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertIdentifierExpr(El: TPasExpr; const aName: string; AContext : TConvertContext): TJSElement; virtual;
    Function ConvertUnaryExpression(El: TUnaryExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertInlineSpecializeExpr(El: TInlineSpecializeExpr; AContext: TConvertContext): TJSElement; virtual;
    // Convert declarations
    Function ConvertElement(El : TPasElement; AContext: TConvertContext) : TJSElement; virtual;
    Function ConvertProperty(El: TPasProperty; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertConst(El: TPasConst; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertDeclarations(El: TPasDeclarations; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertExportSymbol(El: TPasExportSymbol; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertExpression(El: TPasExpr; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertImplBlock(El: TPasImplBlock; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertImplCommand(El: TPasImplCommand; AContext: TConvertContext ): TJSElement; virtual;
    Function ConvertLabelMark(El: TPasImplLabelMark; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertLabels(El: TPasLabels; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertModule(El: TPasModule; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertPackage(El: TPasPackage; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertProcedure(El: TPasProcedure; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertResString(El: TPasResString; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertVariable(El: TPasVariable; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertRecordType(El: TPasRecordType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertClassType(El: TPasClassType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertClassForwardType(El: TPasClassType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertClassOfType(El: TPasClassOfType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertExtClassType(El: TPasClassType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertEnumType(El: TPasEnumType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertSetType(El: TPasSetType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertRangeType(El: TPasRangeType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertTypeAliasType(El: TPasTypeAliasType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertPointerType(El: TPasPointerType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertProcedureType(El: TPasProcedureType; AContext: TConvertContext): TJSElement; virtual;
    Function ConvertArrayType(El: TPasArrayType; AContext: TConvertContext): TJSElement; virtual;
  Public
    // RTTI, TypeInfo constants
    const
      // TParamFlag
      pfVar = 1;
      pfConst = 2;
      pfOut = 4;
      pfArray = 8;
      // TProcedureFlag
      pfStatic = 1;
      pfVarargs = 2;
      pfExternal = 4;
      pfSafeCall = 8;
      pfAsync = $10;
      // PropertyFlag
      pfGetFunction = 1; // getter is a function
      pfSetProcedure = 2; // setter is a function
      pfStoredTrue = 0; // stored true, always
      pfStoredFalse = 4; // stored false, never
      pfStoredField = 8; // stored field, field name is in Stored
      pfStoredFunction = 12; // stored function, function name is in Stored
      pfHasIndex = 16; { if getter is function, append Index as last param
                         if setter is function, append Index as second last param }
    type
      TMethodKind = (
        mkProcedure,      // 0  default
        mkFunction,       // 1
        mkConstructor,    // 2
        mkDestructor,     // 3
        mkClassProcedure, // 4
        mkClassFunction   // 5
        );
      TOrdType  = (
        otSByte,      // 0
        otUByte,      // 1
        otSWord,      // 2
        otUWord,      // 3
        otSLong,      // 4
        otULong,      // 5
        otSIntDouble, // 6 NativeInt
        otUIntDouble  // 7 NativeUInt
        );
    Function GetOrdType(MinValue, MaxValue: TMaxPrecInt; ErrorEl: TPasElement): TOrdType; virtual;
  Public
    // array of const, TVarRec
    const
      pas2js_vtInteger       = 0;
      pas2js_vtBoolean       = 1;
      //vtChar          = 2; // Delphi/FPC: ansichar
      pas2js_vtExtended      = 3; // Note: double in pas2js, PExtended in Delphi/FPC
      //vtString        = 4; // Delphi/FPC: PShortString
      pas2js_vtPointer       = 5;
      //vtPChar         = 6;
      pas2js_vtObject        = 7;
      pas2js_vtClass         = 8;
      pas2js_vtWideChar      = 9;
      //vtPWideChar     = 10;
      //vtAnsiString    = 11;
      pas2js_vtCurrency      = 12; // Note: currency in pas2js, PCurrency in Delphi/FPC
      //vtVariant       = 13;
      pas2js_vtInterface     = 14;
      //vtWideString    = 15;
      //vtInt64         = 16;
      //vtQWord         = 17;
      pas2js_vtUnicodeString = 18;
      // only pas2js, not in Delphi/FPC:
      pas2js_vtNativeInt     = 19;
      pas2js_vtJSValue       = 20;
  Public
    Constructor Create;
    Destructor Destroy; override;
    Function ConvertPasElement(El: TPasElement; Resolver: TPas2JSResolver) : TJSElement;
    // options
    Property Options: TPasToJsConverterOptions read FOptions write FOptions default DefaultPasToJSOptions;
    Property Globals: TPasToJSConverterGlobals read FGlobals write SetGlobals;
    Property UseLowerCase: boolean read GetUseLowerCase write SetUseLowerCase default true;
    Property UseSwitchStatement: boolean read GetUseSwitchStatement write SetUseSwitchStatement;// default false, because slower than "if" in many engines
    Property UseEnumNumbers: boolean read GetUseEnumNumbers write SetUseEnumNumbers; // default false
    Property OnIsElementUsed: TPas2JSIsElementUsedEvent read FOnIsElementUsed write FOnIsElementUsed;
    Property OnIsTypeInfoUsed: TPas2JSIsElementUsedEvent read FOnIsTypeInfoUsed write FOnIsTypeInfoUsed;
    Property ReservedWords: TJSReservedWordList read FReservedWords write SetReservedWords;
  end;

var
  JSTypeCaptions: array[TJSType] of string = (
    'undefined',
    'null',
    'boolean',
    'number',
    'string',
    'object',
    'reference',
    'completion'
    );

function CodePointToJSString(u: longword): TJSString;
function PosLast(c: char; const s: string): integer;

function JSEquals(A, B: TJSElement): boolean;

function dbgs(opts: TPasToJsConverterOptions): string; overload;

implementation

const
  TempRefObjGetterName = 'get';
  TempRefObjSetterName = 'set';
  TempRefObjSetterArgName = 'v';
  TempRefGetPathName = 'p';
  TempRefSetPathName = 's';
  TempRefParamName = 'a';
  IdentChars = ['0'..'9', 'A'..'Z', 'a'..'z','_'];
  AwaitSignature2 = 'function await(aType,TJSPromise):aType';

function CodePointToJSString(u: longword): TJSString;
begin
  if u < $10000 then
    // Note: codepoints $D800 - $DFFF are reserved
    Result:=WideChar(u)
  else
    Result:=WideChar($D800+((u - $10000) shr 10))+WideChar($DC00+((u - $10000) and $3ff));
end;

function PosLast(c: char; const s: string): integer;
begin
  Result:=length(s);
  while (Result>0) and (s[Result]<>c) do dec(Result);
end;

function JSEquals(A, B: TJSElement): boolean;
begin
  if A=nil then
    exit(B=nil)
  else if B=nil then
    exit(false)
  else if A.ClassType<>B.ClassType then
    exit(false);
  if A.ClassType=TJSPrimaryExpressionIdent then
    exit(TJSPrimaryExpressionIdent(A).Name=TJSPrimaryExpressionIdent(B).Name)
  else if A.ClassType=TJSPrimaryExpressionThis then
  else if A.ClassType=TJSDotMemberExpression then
    Result:=JSEquals(TJSDotMemberExpression(A).MExpr,TJSDotMemberExpression(B).MExpr)
        and (TJSDotMemberExpression(A).Name=TJSDotMemberExpression(B).Name)
  else if A.ClassType=TJSBracketMemberExpression then
    Result:=JSEquals(TJSBracketMemberExpression(A).MExpr,TJSBracketMemberExpression(B).MExpr)
        and (TJSBracketMemberExpression(A).Name=TJSBracketMemberExpression(B).Name)
  else
    exit(false);
end;

function dbgs(opts: TPasToJsConverterOptions): string;
var
  o: TPasToJsConverterOption;
  h: string;
begin
  Result:='';
  for o in opts do
    begin
    if Result<>'' then Result:=Result+',';
    str(o,h);
    Result:=Result+h;
    end;
  Result:='['+Result+']';
end;

{ TPas2JSInitialFinalizationScope }

destructor TPas2JSInitialFinalizationScope.Destroy;
begin
  FreeAndNil(ImplJS);
  inherited Destroy;
end;

{ TInterfaceSectionContext }

destructor TInterfaceSectionContext.Destroy;
var
  i: Integer;
  El: TJSElement;
begin
  if ImplHeaderStatements<>nil then
    begin
    for i:=0 to ImplHeaderStatements.Count-1 do
      begin
      El:=TJSElement(ImplHeaderStatements[i]);
      El.Free;
      end;
    FreeAndNil(ImplHeaderStatements);
    end;
  inherited Destroy;
end;

procedure TInterfaceSectionContext.AddImplHeaderStatement(JS: TJSElement);
begin
  if JS=nil then exit;
  if ImplContext<>nil then
    begin
    // unit impl is currently created
    ImplContext.AddHeaderStatement(JS);
    end
  else if ImplSrcElements<>nil then
    begin
    // unit impl finished -> e.g. during the initialization section
    ImplSrcElements.Statements.InsertNode(ImplHeaderIndex).Node:=JS;
    inc(ImplHeaderIndex);
    end
  else
    begin
    // unit impl not yet created
    if ImplHeaderStatements=nil then
      ImplHeaderStatements:=TFPList.Create;
    ImplHeaderStatements.Add(JS);
    end;
end;

{ TPas2JSResolverHub }

function TPas2JSResolverHub.GetJSDelaySpecializes(Index: integer
  ): TPasGenericType;
begin
  Result:=TPasGenericType(FJSDelaySpecialize[Index]);
end;

constructor TPas2JSResolverHub.Create(TheOwner: TObject);
begin
  inherited Create(TheOwner);
  FJSDelaySpecialize:=TFPList.Create;
end;

destructor TPas2JSResolverHub.Destroy;
begin
  FreeAndNil(FJSDelaySpecialize);
  inherited Destroy;
end;

procedure TPas2JSResolverHub.Reset;
begin
  inherited Reset;
  FJSDelaySpecialize.Clear;
end;

procedure TPas2JSResolverHub.AddJSDelaySpecialize(SpecType: TPasGenericType);
begin
  if FJSDelaySpecialize.IndexOf(SpecType)>=0 then
    raise EPas2JS.Create('TPas2JSResolverHub.AddJSDelaySpecialize '+GetObjPath(SpecType));
  FJSDelaySpecialize.Add(SpecType);
end;

function TPas2JSResolverHub.IsJSDelaySpecialize(SpecType: TPasGenericType): boolean;
begin
  Result:=FJSDelaySpecialize.IndexOf(SpecType)>=0;
end;

function TPas2JSResolverHub.JSDelaySpecializeCount: integer;
begin
  Result:=FJSDelaySpecialize.Count;
end;

{ TPas2JSModuleScope }

procedure TPas2JSModuleScope.SetJSPromiseClass(const AValue: TPasClassType);
begin
  if FJSPromiseClass=AValue then Exit;
  if FJSPromiseClass<>nil then
    FJSPromiseClass.Release{$IFDEF CheckPasTreeRefCount}('TPas2JSModuleScope.SetJSPromiseClass'){$ENDIF};
  FJSPromiseClass:=AValue;
  if FJSPromiseClass<>nil then
    FJSPromiseClass.AddRef{$IFDEF CheckPasTreeRefCount}('TPas2JSModuleScope.SetJSPromiseClass'){$ENDIF};
end;

procedure TPas2JSModuleScope.ClearStoreJSLocalVars;
var
  i: Integer;
begin
  for i:=0 to length(StoreJSLocalVars)-1 do
    FreeAndNil(StoreJSLocalVars[i]);
  StoreJSLocalVars:=nil;
end;

destructor TPas2JSModuleScope.Destroy;
begin
  ClearStoreJSLocalVars;
  JSPromiseClass:=nil;
  inherited Destroy;
end;

{ TPas2JSClassScope }

constructor TPas2JSClassScope.Create;
begin
  inherited Create;
  ElevatedLocals:=TPas2jsElevatedLocals.Create;
end;

destructor TPas2JSClassScope.Destroy;
begin
  FreeAndNil(ElevatedLocals);
  FreeAndNil(MsgIntToProc);
  FreeAndNil(MsgStrToProc);
  inherited Destroy;
end;

{ TRootContext }

procedure TRootContext.AddGlobalClassMethod(p: TPasProcedure);
begin
  {$IF defined(fpc) and (FPC_FULLVERSION<30101)}
  SetLength(GlobalClassMethods,length(GlobalClassMethods)+1);
  GlobalClassMethods[length(GlobalClassMethods)-1]:=P;
  {$ELSE}
  Insert(P,GlobalClassMethods,length(GlobalClassMethods));
  {$ENDIF}
end;

destructor TRootContext.Destroy;
begin
  inherited Destroy;
end;

{ TPasToJSConverterGlobals }

constructor TPasToJSConverterGlobals.Create(TheOwner: TObject);
begin
  FOwner:=TheOwner;
  ResetBuiltInNames;
end;

procedure TPasToJSConverterGlobals.Reset;
begin
  RTLVersion:=1;
  TargetPlatform:=PlatformBrowser;
  TargetProcessor:=ProcessorECMAScript5;
  ResetBuiltInNames;
end;

procedure TPasToJSConverterGlobals.ResetBuiltInNames;
var
  n: TPas2JSBuiltInName;
begin
  for n in TPas2JSBuiltInName do
    BuiltInNames[n]:=Pas2JSBuiltInNames[n];
end;

{ TPas2jsElevatedLocals }

procedure TPas2jsElevatedLocals.InternalAdd(Item: TPasIdentifier);
var
  {$IFDEF fpc}
  Index: Integer;
  {$ENDIF}
  OldItem: TPasIdentifier;
  LoName: string;
begin
  LoName:=lowercase(Item.Identifier);
  {$IFDEF VerbosePasResolver}
  if Item.Owner<>nil then
    raise Exception.Create('20160925184110');
  Item.Owner:=Self;
  {$ENDIF}
  {$IFDEF pas2js}
  OldItem:=TPasIdentifier(FElevatedLocals.Find(LoName));
  if OldItem<>nil then
    begin
    // insert LIFO - last in, first out
    {$IFDEF VerbosePasResolver}
    if lowercase(OldItem.Identifier)<>LoName then
      raise Exception.Create('20181025113922');
    {$ENDIF}
    Item.NextSameIdentifier:=OldItem;
    FElevatedLocals.Remove(LoName);
    end;
  FElevatedLocals.Add(LoName, Item);
  {$ELSE}
  Index:=FElevatedLocals.FindIndexOf(LoName);
  //writeln('  Index=',Index);
  if Index>=0 then
    begin
    // insert LIFO - last in, first out
    OldItem:=TPasIdentifier(FElevatedLocals.List^[Index].Data);
    {$IFDEF VerbosePasResolver}
    if lowercase(OldItem.Identifier)<>LoName then
      raise Exception.Create('20160925183438');
    {$ENDIF}
    Item.NextSameIdentifier:=OldItem;
    FElevatedLocals.List^[Index].Data:=Item;
    end
  else
    begin
    FElevatedLocals.Add(LoName, Item);
    end;
  {$ENDIF}
  {$IFDEF VerbosePasResolver}
  if Find(Item.Identifier)<>Item then
    raise Exception.Create('20160925183849');
  {$ENDIF}
end;

procedure TPas2jsElevatedLocals.OnClear(Item, Dummy: pointer);
var
  PasIdentifier: TPasIdentifier absolute Item;
  Ident: TPasIdentifier;
begin
  if Dummy=nil then ;
  //writeln('TPasIdentifierScope.OnClearItem ',PasIdentifier.Identifier+':'+PasIdentifier.ClassName);
  while PasIdentifier<>nil do
    begin
    Ident:=PasIdentifier;
    PasIdentifier:=PasIdentifier.NextSameIdentifier;
    Ident.Free;
    end;
end;

constructor TPas2jsElevatedLocals.Create;
begin
  inherited Create;
  FElevatedLocals:=TPasResHashList.Create;
end;

destructor TPas2jsElevatedLocals.Destroy;
begin
  FElevatedLocals.ForEachCall(@OnClear,nil);
  {$IFDEF pas2js}
  FElevatedLocals:=nil;
  {$ELSE}
  FreeAndNil(FElevatedLocals);
  {$ENDIF}
  inherited Destroy;
end;

// inline
function TPas2jsElevatedLocals.Find(const Identifier: String
  ): TPasIdentifier;
begin
  Result:=TPasIdentifier(FElevatedLocals.Find(lowercase(Identifier)));
end;

function TPas2jsElevatedLocals.Add(const Identifier: String;
  El: TPasElement): TPasIdentifier;
var
  Item: TPasIdentifier;
begin
  //writeln('TPas2jsElevatedLocals.Add Identifier="',Identifier,'" El=',GetObjName(El));
  Item:=TPasIdentifier.Create;
  Item.Identifier:=Identifier;
  Item.Element:=El;

  InternalAdd(Item);
  //writeln('TPas2jsElevatedLocals.Add END');
  Result:=Item;
end;

{ TPas2JSSectionScope }

constructor TPas2JSSectionScope.Create;
begin
  inherited Create;
  ElevatedLocals:=TPas2jsElevatedLocals.Create;
end;

destructor TPas2JSSectionScope.Destroy;
begin
  FreeAndNil(ElevatedLocals);
  inherited Destroy;
end;

procedure TPas2JSSectionScope.WriteElevatedLocals(Prefix: string);
begin
  Prefix:=Prefix+'  ';
  ElevatedLocals.FElevatedLocals.ForEachCall(@OnWriteItem,Pointer(Prefix));
end;

{ TPas2JSPrecompiledJS }

procedure TPas2JSPrecompiledJS.AddShortRef(El: TPasElement);
begin
  if ShortRefs=nil then
    ShortRefs:=TFPList.Create;
  if ShortRefs.IndexOf(El)<0 then
    ShortRefs.Add(El);
end;

destructor TPas2JSPrecompiledJS.Destroy;
begin
  FreeAndNil(GlobalJS);
  FreeAndNil(ShortRefs);
  inherited Destroy;
end;

{ TPas2JSProcedureScope }

procedure TPas2JSProcedureScope.AddGlobalJS(const JS: string);
begin
  if ImplJS=nil then
    raise Exception.Create('[20201018120133] TPas2JSProcedureScope.AddGlobalJS');
  if ImplJS.GlobalJS=nil then
    ImplJS.GlobalJS:=TStringList.Create;
  ImplJS.GlobalJS.Add(Js);
end;

destructor TPas2JSProcedureScope.Destroy;
begin
  FreeAndNil(ImplJS);
  inherited Destroy;
end;

{ TFCLocalIdentifier }

constructor TFCLocalIdentifier.Create(const aName: string; TheEl: TPasElement;
  aKind: TCtxVarKind);
begin
  Name:=aName;
  Element:=TheEl;
  Kind:=aKind;
end;

{ TPas2jsPasScanner }

function TPas2jsPasScanner.HandleInclude(const Param: String): TToken;

  procedure SetStr(s: string);
  var
    i: Integer;
    h: String;
  begin
    Result:=tkString;
    if s='' then
      s:=''''''
    else
      for i:=length(s) downto 1 do
        case s[i] of
        #0..#31,#127:
          begin
          h:='#'+IntToStr(ord(s[i]));
          if i>1 then h:=''''+h;
          if (i<length(s)) and (s[i+1]<>'#') then
            h:=h+'''';
          s:=LeftStr(s,i-1)+h+copy(s,i+1,length(s));
          end;
        else
          if i=length(s) then
            s:=s+'''';
          if s[i]='''' then
            Insert('''',s,i);
          if i=1 then
            s:=''''+s;
        end;
    SetCurTokenString(s);
  end;

  procedure SetInteger(const i: TMaxPrecInt);
  begin
    Result:=tkNumber;
    SetCurTokenString(IntToStr(i));
  end;

var
  Year, Month, Day, Hour, Minute, Second, MilliSecond: word;
  i: Integer;
  Scope: TPasScope;
begin
  if (Param<>'') and (Param[1]='%') then
  begin
    if (length(Param)<3) or (Param[length(Param)]<>'%') then
      begin
      SetStr('');
      DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,SWarnIllegalCompilerDirectiveX,
        ['$i '+Param]);
      exit;
      end;
    if length(Param)>255 then
      begin
      SetStr('');
      DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,SWarnIllegalCompilerDirectiveX,
        ['$i '+copy(Param,1,255)+'...']);
      exit;
      end;
    case lowercase(Param) of
    '%date%':
      begin
        // 'Y/M/D'
        DecodeDate(Now,Year,Month,Day);
        SetStr(IntToStr(Year)+'/'+IntToStr(Month)+'/'+IntToStr(Day));
        exit;
      end;
    '%time%':
      begin
        // 'hh:mm:ss'
        DecodeTime(Now,Hour,Minute,Second,MilliSecond);
        SetStr(Format('%2d:%2d:%2d',[Hour,Minute,Second]));
        exit;
      end;
    '%pas2jstarget%','%fpctarget%',
    '%pas2jstargetos%','%fpctargetos%':
      begin
        SetStr(PasToJsPlatformNames[TargetPlatform]);
        exit;
      end;
    '%pas2jstargetcpu%','%fpctargetcpu%':
      begin
        SetStr(PasToJsProcessorNames[TargetProcessor]);
        exit;
      end;
    '%pas2jsversion%','%fpcversion%':
      begin
        SetStr(CompilerVersion);
        exit;
      end;
    '%file%':
      begin
        SetStr(CurFilename);
        exit;
      end;
    '%filename%':
      begin
        SetStr(ExtractFileName(CurFilename));
        exit;
      end;
    '%unit%',
    '%module%':
      begin
        SetStr(CurModuleName);
        exit;
      end;
    '%line%':
      begin
        SetStr(IntToStr(CurRow));
        exit;
      end;
    '%linenum%':
      begin
        SetInteger(CurRow);
        exit;
      end;
    '%currentroutine%':
      begin
        if Resolver<>nil then
          for i:=Resolver.ScopeCount-1 downto 0 do
          begin
            Scope:=Resolver.Scopes[i];
            if (Scope.Element is TPasProcedure)
                and (Scope.Element.Name<>'') then
            begin
              SetStr(Scope.Element.Name);
              exit;
            end;
          end;
        SetStr('<anonymous>');
        exit;
      end;
    else
      SetStr(GetEnvironmentVariable(copy(Param,2,length(Param)-2)));
      exit;
    end;
  end;
  Result:=inherited HandleInclude(Param);
end;

procedure TPas2jsPasScanner.DoHandleOptimization(OptName, OptValue: string);

  procedure HandleBoolean(o: TPasToJsConverterOption; IsGlobalSwitch: boolean);
  var
    Enable: Boolean;
  begin
    Enable:=false;
    case lowercase(OptValue) of
    '','on','+': Enable:=true;
    'off','-': Enable:=false;
    else
      Error(nErrWrongSwitchToggle,SErrWrongSwitchToggle,[]);
    end;
    if IsGlobalSwitch and SkipGlobalSwitches then
      begin
      DoLog(mtWarning,nMisplacedGlobalCompilerSwitch,SMisplacedGlobalCompilerSwitch,[]);
      exit;
      end;
    if Enable then
      begin
      Include(GlobalConvOptsEnabled,o);
      Exclude(GlobalConvOptsDisabled,o);
      end
    else
      begin
      Include(GlobalConvOptsDisabled,o);
      Exclude(GlobalConvOptsEnabled,o);
      end;
  end;

begin
  case lowercase(OptName) of
  'enumnumbers':
    HandleBoolean(coEnumNumbers,true);
  'usestrict':
    HandleBoolean(coUseStrict,true);
  'jsshortrefglobals':
    HandleBoolean(coShortRefGlobals,true);
  else
    DoLog(mtWarning,nWarnIllegalCompilerDirectiveX,sWarnIllegalCompilerDirectiveX,['optimization '+OptName]);
  end;
end;

function TPas2jsPasScanner.ReadNonPascalTillEndToken(StopAtLineEnd: boolean
  ): TToken;
var
  StartPos, MyTokenPos: integer;
  s: string;
  l: integer;

  Procedure CommitTokenPos;
  begin
    {$IFDEF Pas2js}
    TokenPos:=MyTokenPos;
    {$ELSE}
    TokenPos:=PChar(s)+MyTokenPos-1;
    {$ENDIF}
  end;

  Procedure Add;
  var
    AddLen: PtrInt;
  begin
    AddLen:=MyTokenPos-StartPos;
    if AddLen=0 then
      SetCurTokenString('')
    else
      begin
      SetCurTokenString(CurTokenString+copy(CurLine,StartPos,AddLen));
      StartPos:=MyTokenPos;
      end;
  end;

  function DoEndOfLine: boolean;
  begin
    Add;
    if StopAtLineEnd then
      begin
      ReadNonPascalTillEndToken := tkLineEnding;
      CommitTokenPos;
      SetCurToken(tkLineEnding);
      FetchLine;
      exit(true);
      end;
    if not FetchLine then
      begin
      ReadNonPascalTillEndToken := tkEOF;
      SetCurToken(tkEOF);
      exit(true);
      end;
    s:=CurLine;
    l:=length(s);
    MyTokenPos:=1;
    StartPos:=MyTokenPos;
    Result:=false;
  end;

  procedure HandleEscape;
  begin
    inc(MyTokenPos);
    if (MyTokenPos<=l) and (s[MyTokenPos]>#31) then
      inc(MyTokenPos);
  end;

begin
  SetCurTokenString('');
  s:=CurLine;
  l:=length(s);
  {$IFDEF Pas2js}
  MyTokenPos:=TokenPos;
  {$ELSE}
  {$IFDEF VerbosePas2JS}
  if (TokenPos<PChar(s)) or (TokenPos>PChar(s)+length(s)) then
    Error(nErrRangeCheck,'[20181109104812]');
  {$ENDIF}
  MyTokenPos:=TokenPos-PChar(s)+1;
  {$ENDIF}
  StartPos:=MyTokenPos;
  repeat
    if MyTokenPos>l then
      if DoEndOfLine then exit;
    case s[MyTokenPos] of
    '\':
      HandleEscape;
    '''':
      begin
      inc(MyTokenPos);
      repeat
        if MyTokenPos>l then
          Error(nErrOpenString,SErrOpenString);
        case s[MyTokenPos] of
        '\':
          HandleEscape;
        '''':
          begin
          inc(MyTokenPos);
          break;
          end;
        #10,#13:
          begin
          // string literal missing closing apostroph
          break;
          end
        else
          inc(MyTokenPos);
        end;
      until false;
      end;
    '"':
      begin
      inc(MyTokenPos);
      repeat
        if MyTokenPos>l then
          Error(nErrOpenString,SErrOpenString);
        case s[MyTokenPos] of
        '\':
          HandleEscape;
        '"':
          begin
          inc(MyTokenPos);
          break;
          end;
        #10,#13:
          begin
          // string literal missing closing quote
          break;
          end
        else
          inc(MyTokenPos);
        end;
      until false;
      end;
    '`': // template literal
      begin
      inc(MyTokenPos);
      repeat
        while MyTokenPos>l do
          if DoEndOfLine then
            begin
            if not StopAtLineEnd then
              Error(nErrOpenString,SErrOpenString);
            exit;
            end;
        case s[MyTokenPos] of
        '\':
          HandleEscape;
        '`':
          begin
          inc(MyTokenPos);
          break;
          end;
        // Note: template literals can span multiple lines
        else
          inc(MyTokenPos);
        end;
      until false;
      end;
    '/':
      begin
      inc(MyTokenPos);
      if (MyTokenPos<=l) and (s[MyTokenPos]='/') then
        begin
        // skip Delphi comment //, see Note above
        repeat
          inc(MyTokenPos);
        until (MyTokenPos>l) or (s[MyTokenPos] in [#10,#13]);
        end;
      end;
    '0'..'9', 'A'..'Z', 'a'..'z','_':
      begin
      // number or identifier
      if (CompareText(copy(s,MyTokenPos,3),'end')=0)
          and ((MyTokenPos+3>l) or not (s[MyTokenPos+3] in IdentChars)) then
        begin
        // 'end' found
        Add;
        if CurTokenString<>'' then
          begin
          // return characters in front of 'end'
          Result:=tkWhitespace;
          CommitTokenPos;
          SetCurToken(Result);
          exit;
          end;
        // return 'end'
        Result := tkend;
        SetCurTokenString(copy(s,MyTokenPos,3));
        inc(MyTokenPos,3);
        CommitTokenPos;
        SetCurToken(Result);
        exit;
        end
      else
        begin
        // skip identifier
        while (MyTokenPos<=l) and (s[MyTokenPos] in IdentChars) do
          inc(MyTokenPos);
        end;
      end;
    else
      inc(MyTokenPos);
    end;
  until false;
end;

{ TPas2JSResolver }

// inline
function TPas2JSResolver.GetJSBaseTypes(aBaseType: TPas2jsBaseType
  ): TPasUnresolvedSymbolRef;
begin
  Result:=FJSBaseTypes[aBaseType];
end;

function TPas2JSResolver.GetJSBuiltInProcs(aProc: TPas2jsBuiltInProc
  ): TResElDataBuiltInProc;
begin
  Result:=FJSBuiltInProcs[aProc];
end;

procedure TPas2JSResolver.InternalAdd(Item: TPasIdentifier);
var
  {$IFDEF fpc}
  Index: Integer;
  {$ENDIF}
  OldItem: TPasIdentifier;
  aName: String;
begin
  aName:=Item.Identifier;
  {$IFDEF VerbosePasResolver}
  if Item.Owner<>nil then
    raise Exception.Create('20170322235419');
  Item.Owner:=Self;
  {$ENDIF}
  {$IFDEF pas2js}
  OldItem:=TPasIdentifier(FExternalNames.Find(aName));
  if OldItem<>nil then
    begin
    // insert LIFO - last in, first out
    {$IFDEF VerbosePasResolver}
    if OldItem.Identifier<>aName then
      raise Exception.Create('20181025114714');
    {$ENDIF}
    Item.NextSameIdentifier:=OldItem;
    FExternalNames.Remove(aName);
    end;
  FExternalNames.Add(aName,Item);
  {$ELSE}
  Index:=FExternalNames.FindIndexOf(aName);
  //writeln('  Index=',Index);
  if Index>=0 then
    begin
    // insert LIFO - last in, first out
    OldItem:=TPasIdentifier(FExternalNames.List^[Index].Data);
    {$IFDEF VerbosePasResolver}
    if OldItem.Identifier<>aName then
      raise Exception.Create('20170322235429');
    {$ENDIF}
    Item.NextSameIdentifier:=OldItem;
    FExternalNames.List^[Index].Data:=Item;
    end
  else
    FExternalNames.Add(aName, Item);
  {$ENDIF}
  {$IFDEF VerbosePasResolver}
  if FindExternalName(Item.Identifier)<>Item then
    raise Exception.Create('20170322235433');
  {$ENDIF}
end;

procedure TPas2JSResolver.OnClearHashItem(Item, Dummy: pointer);
var
  PasIdentifier: TPasIdentifier absolute Item;
  Ident: TPasIdentifier;
begin
  if Dummy=nil then ;
  //writeln('TPas2JSResolver.OnClearItem ',PasIdentifier.Identifier+':'+PasIdentifier.ClassName);
  while PasIdentifier<>nil do
    begin
    Ident:=PasIdentifier;
    PasIdentifier:=PasIdentifier.NextSameIdentifier;
    Ident.Free;
    end;
end;

procedure TPas2JSResolver.OnHasAnonymousEl(El: TPasElement; arg: pointer);
var
  Data: PHasAnoFuncData absolute arg;
begin
  if (El=nil) or (Data^.Expr<>nil) or (El.ClassType<>TProcedureExpr) then exit;
  Data^.Expr:=TProcedureExpr(El);
end;

procedure TPas2JSResolver.OnHasElReadingDecl(El: TPasElement; arg: pointer);
var
  Data: PHasElReadingDeclData absolute arg;
  Ref: TResolvedReference;
begin
  if Data^.El<>nil then exit;
  if El.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(El.CustomData);
    if (Ref.Declaration=Data^.Decl) and (Ref.Access in rraAllRead) then
      begin
      Data^.El:=El;
      end;
    end;
end;

procedure TPas2JSResolver.OnFindExtSystemClass(El: TPasElement; ElScope,
  StartScope: TPasScope; FindExtSystemClassData: Pointer; var Abort: boolean);
var
  Data: PPRFindExtSystemClass absolute FindExtSystemClassData;
  aClass: TPasClassType;
begin
  if Data^.Found<>nil then exit;
  if not (El is TPasClassType) then exit;
  aClass:=TPasClassType(El);
  if not aClass.IsExternal then exit;
  if aClass.Parent is TPasMembersType then
    exit; // nested class
  if aClass.ExternalName<>Data^.JSName then exit;
  Data^.Found:=aClass;
  Data^.ElScope:=ElScope;
  Data^.StartScope:=StartScope;
  Abort:=true;
end;

function TPas2JSResolver.HasOverloadIndex(El: TPasElement;
  WithElevatedLocal: boolean): boolean;
var
  C: TClass;
  ProcScope: TPasProcedureScope;
begin
  C:=El.ClassType;
  if C=TPasProperty then
    exit(false)
  else if (C=TPasConst)
      or C.InheritsFrom(TPasType) then
    begin
    if (not WithElevatedLocal) and (El.Parent is TProcedureBody) then
      exit(false); // local const/type is counted via ElevatedLocals
    if (C=TPasClassType) and TPasClassType(El).IsForward then
      exit(false);
    end
  else if C.InheritsFrom(TPasProcedure) then
    begin
    if TPasProcedure(El).IsOverride then
      exit(true); // using name of overridden
    if El.Visibility=visPublished then
      exit(false);

    // Note: external proc pollutes the name space
    ProcScope:=TPasProcedureScope(El.CustomData);
    if ProcScope.DeclarationProc<>nil then
      // implementation proc -> only count the header -> skip
      exit(false);
    if ProcScope.SpecializedFromItem<>nil then
      exit(false);
    end;
  Result:=true;
end;

function TPas2JSResolver.GetOverloadIndex(Identifier: TPasIdentifier;
  StopAt: TPasElement): integer;
// if not found return number of overloads
// if found return index in overloads
var
  El: TPasElement;
begin
  Result:=0;
  // iterate from last added to first added
  // Note: the first added has Index=0
  while Identifier<>nil do
    begin
    El:=Identifier.Element;
    Identifier:=Identifier.NextSameIdentifier;
    if El=StopAt then
      Result:=0
    else if HasOverloadIndex(El) then
      inc(Result);
    end;
end;

function TPas2JSResolver.GetOverloadAt(Identifier: TPasIdentifier;
  var Index: integer): TPasIdentifier;
// if found Result<>nil and Index=0
// if not found Result=nil and Index is reduced by number of overloads
var
  El: TPasElement;
  CurIdent: TPasIdentifier;
  Count: Integer;
begin
  if Identifier=nil then exit(nil);
  // Note: the Identifier chain is from last added to first added
  // -> get length of chain
  Count:=0;
  CurIdent:=Identifier;
  while CurIdent<>nil do
    begin
    El:=CurIdent.Element;
    CurIdent:=CurIdent.NextSameIdentifier;
    if HasOverloadIndex(El) then
      inc(Count);
    end;
  if Count<=Index then
    begin
    // Index is not in this scope
    dec(Index);
    exit(nil);
    end;
  // Index is in this scope -> find it
  CurIdent:=Identifier;
  while CurIdent<>nil do
    begin
    if HasOverloadIndex(CurIdent.Element) then
      begin
      dec(Count);
      if (Index=Count) then
        begin
        Index:=0;
        Result:=CurIdent;
        exit;
        end;
      end;
    CurIdent:=CurIdent.NextSameIdentifier;
    end;
end;

function TPas2JSResolver.GetOverloadIndex(El: TPasElement): integer;
var
  i, j, MaxDepth: Integer;
  Identifier: TPasIdentifier;
  Scope: TPasIdentifierScope;
  CurEl: TPasElement;
  ThisChanged: Boolean;
  ElevatedLocals: TPas2jsElevatedLocals;
begin
  Result:=0;
  //if SameText(El.Name,'ci') then writeln('TPas2JSResolver.GetOverloadIndex ',GetObjPath(El),' HasOverloadIndex=',HasOverloadIndex(El,true));
  if not HasOverloadIndex(El,true) then exit;

  ThisChanged:=false;
  MaxDepth:=FOverloadScopes.Count-1;
  for i:=MaxDepth downto 0 do
    begin
    Scope:=TPasIdentifierScope(FOverloadScopes[i]);
    if Scope.ClassType=TPas2JSOverloadChgThisScope then
      begin
      ThisChanged:=true;
      continue;
      end;
    if i<MaxDepth then
      begin
      // Reason for "if i<MaxDepth":
      // Because the elevated locals have their index after their global scope
      // and before the next deeper (local) scope

      // check elevated locals
      ElevatedLocals:=GetElevatedLocals(Scope);
      //if SameText(El.Name,'ci') then writeln('TPas2JSResolver.GetOverloadIndex ',GetObjPath(El),' Scope.Element=',GetObjName(Scope.Element),' ',ElevatedLocals<>nil);
      if ElevatedLocals<>nil then
        begin
        Identifier:=ElevatedLocals.Find(El.Name);
        j:=0;
        // add count or index
        while Identifier<>nil do
          begin
          CurEl:=Identifier.Element;
          Identifier:=Identifier.NextSameIdentifier;
          if CurEl=El then
            j:=0
          else
            inc(j);
          end;
        inc(Result,j);
        end;
      end;
    if not ThisChanged then
      begin
      // add count or index of this scope
      Identifier:=Scope.FindLocalIdentifier(El.Name);
      inc(Result,GetOverloadIndex(Identifier,El));
      end;
    end;
  if ThisChanged then exit;
  // element in global scope
  // -> add count or index of the external scope
  Identifier:=FindExternalName(El.Name);
  inc(Result,GetOverloadIndex(Identifier,El));
end;

function TPas2JSResolver.GetOverloadAt(const aName: String; Index: integer
  ): TPasIdentifier;
var
  i, MaxDepth: Integer;
  Scope: TPasIdentifierScope;
  Skip: Boolean;
  ElevatedLocals: TPas2jsElevatedLocals;
begin
  Result:=nil;
  Skip:=false;
  MaxDepth:=FOverloadScopes.Count-1;
  for i:=MaxDepth downto 0 do
    begin
    // find last added
    Scope:=TPasIdentifierScope(FOverloadScopes[i]);
    if Scope.ClassType=TPas2JSOverloadChgThisScope then
      begin
      Skip:=true;
      continue;
      end;
    if i<MaxDepth then
      begin
      // check elevated locals
      // Note: the elevated locals are after the section scope and
      //       before the next deeper scope
      ElevatedLocals:=GetElevatedLocals(Scope);
      if ElevatedLocals<>nil then
        begin
        Result:=ElevatedLocals.Find(aName);
        Result:=GetOverloadAt(Result,Index);
        if Result<>nil then
          exit;
        end;
      end;
    if not Skip then
      begin
      Result:=Scope.FindLocalIdentifier(aName);
      Result:=GetOverloadAt(Result,Index);
      if Result<>nil then
        exit;
      end;
    end;
  if Skip then exit;
  // find in external names
  Result:=FindExternalName(aName);
  Result:=GetOverloadAt(Result,Index);
end;

function TPas2JSResolver.GetElevatedLocals(Scope: TPasScope
  ): TPas2jsElevatedLocals;
var
  C: TClass;
begin
  C:=Scope.ClassType;
  if C=TPas2JSSectionScope then
    Result:=TPas2JSSectionScope(Scope).ElevatedLocals
  else if C=TPas2JSClassScope then
    Result:=TPas2JSClassScope(Scope).ElevatedLocals
  else
    Result:=nil;
end;

function TPas2JSResolver.RenameOverload(El: TPasElement): boolean;
var
  OverloadIndex: Integer;

  function GetDuplicate: TPasElement;
  var
    Duplicate: TPasIdentifier;
  begin
    Duplicate:=GetOverloadAt(El.Name,0);
    Result:=Duplicate.Element;
  end;

var
  NewName: String;
  Duplicate: TPasElement;
  ProcScope: TPas2JSProcedureScope;
begin
  // => count overloads in this section
  OverloadIndex:=GetOverloadIndex(El);
  //if SameText(El.Name,'ci') then writeln('TPas2JSResolver.RenameOverload ',GetObjPath(El),' ',OverloadIndex);
  if OverloadIndex=0 then
    Result:=false // there is no overload
  else
    begin
    if (El.ClassType=TPasClassFunction)
        and (El.Parent.ClassType=TPasClassType)
        and (TPas2JSClassScope(TPasClassType(El.Parent).CustomData).NewInstanceFunction=El) then
      begin
      Duplicate:=GetDuplicate;
      RaiseMsg(20170324234324,nNewInstanceFunctionMustNotHaveOverloadAtX,
        sNewInstanceFunctionMustNotHaveOverloadAtX,[GetElementSourcePosStr(Duplicate)],El);
      end;
    if El.Visibility=visPublished then
      begin
      Duplicate:=GetDuplicate;
      RaiseMsg(20170413220924,nDuplicateIdentifier,sDuplicateIdentifier,
        [Duplicate.Name,GetElementSourcePosStr(Duplicate)],El);
      end;

    NewName:=El.Name+'$'+IntToStr(OverloadIndex);
    {$IFDEF VerbosePas2JS}
    writeln('TPas2JSResolver.RenameOverload "',El.Name,'" has overload. NewName="',NewName,'"');
    {$ENDIF}
    if (El.CustomData is TPas2JSProcedureScope) then
      begin
      ProcScope:=TPas2JSProcedureScope(El.CustomData);
      ProcScope.OverloadName:=NewName;
      if ProcScope.DeclarationProc<>nil then
        RaiseInternalError(20180322233222,GetElementDbgPath(El));
      if ProcScope.ImplProc<>nil then
        TPas2JSProcedureScope(ProcScope.ImplProc.CustomData).OverloadName:=NewName;
      if ProcScope.SpecializedFromItem<>nil then
        RenameSpecialized(ProcScope.SpecializedFromItem);
      end
    else
      El.Name:=NewName;
    Result:=true;
    end;
end;

procedure TPas2JSResolver.RenameOverloadsInSection(aSection: TPasSection);
var
  IntfSection: TInterfaceSection;
  OldScopeCount: Integer;
  Scope: TPas2JSSectionScope;
begin
  if aSection=nil then exit;
  Scope:=aSection.CustomData as TPas2JSSectionScope;
  if Scope.Renamed then
    RaiseNotYetImplemented(20200601231236,aSection);

  IntfSection:=nil;
  OldScopeCount:=FOverloadScopes.Count;
  if aSection.ClassType=TImplementationSection then
    begin
    IntfSection:=RootElement.InterfaceSection;
    PushOverloadScope(IntfSection.CustomData as TPasIdentifierScope);
    end;
  PushOverloadScope(aSection.CustomData as TPasIdentifierScope);
  RenameOverloads(aSection,aSection.Declarations);
  RenameSubOverloads(aSection.Declarations);
  RestoreOverloadScopeLvl(OldScopeCount);
  Scope.Renamed:=true;
  {$IFDEF VerbosePas2JS}
  //writeln('TPas2JSResolver.RenameOverloadsInSection END ',GetObjName(aSection));
  {$ENDIF}
end;

procedure TPas2JSResolver.RenameOverloads(DeclEl: TPasElement;
  Declarations: TFPList);
var
  i: Integer;
  El: TPasElement;
  Proc: TPasProcedure;
  ProcScope, OvrProcScope, ImplProcScope: TPas2JSProcedureScope;
  C: TClass;
begin
  //IsExternalClass:=(DeclEl is TPasClassType) and (TPasClassType(DeclEl).IsExternal);
  if DeclEl=nil then;
  for i:=0 to Declarations.Count-1 do
    begin
    El:=TPasElement(Declarations[i]);
    C:=El.ClassType;
    if C.InheritsFrom(TPasProcedure) then
      begin
      Proc:=TPasProcedure(El);
      ProcScope:=Proc.CustomData as TPas2JSProcedureScope;
      //writeln('TPas2JSResolver.RenameOverloads Proc=',Proc.Name,' DeclarationProc=',GetObjName(ProcScope.DeclarationProc),' ImplProc=',GetObjName(ProcScope.ImplProc),' ClassScope=',GetObjName(ProcScope.ClassOrRecordScope));
      if ProcScope.DeclarationProc<>nil then
        // DeclarationProc already propagates to ImplProc
        continue
      else if Proc.IsOverride then
        begin
        // override -> copy name from overridden proc
        if ProcScope.OverriddenProc=nil then
          RaiseInternalError(20171205183502);
        OvrProcScope:=TPas2JSProcedureScope(ProcScope.OverriddenProc.CustomData);
        if OvrProcScope.OverloadName<>'' then
          begin
          ProcScope.OverloadName:=OvrProcScope.OverloadName;
          if ProcScope.ImplProc<>nil then
            begin
            ImplProcScope:=TPas2JSProcedureScope(ProcScope.ImplProc.CustomData);
            ImplProcScope.OverloadName:=ProcScope.OverloadName;
            ImplProcScope.JSName:=ProcScope.JSName;
            end;
          end;
        continue;
        end
      else if Proc.IsExternal then
        begin
        // Note: Pascal names of external procs are not in the generated JS,
        // so no need to rename them
        continue;
        end
      else
        begin
        // proc declaration (header, not body)
        RenameOverload(Proc);
        end;
      end
    else if C.InheritsFrom(TPasType) then
      begin
      if El.Parent is TProcedureBody then
        RenameOverload(El);
      end
    else if C=TPasConst then
      RenameOverload(El)
    else if C.InheritsFrom(TPasVariable) then
      begin
      // class fields can have name clashes, record fields cannot
      if El.Parent.ClassType=TPasClassType then
        RenameOverload(El);
      end;
    end;
  {$IFDEF VerbosePas2JS}
  //writeln('TPas2JSResolver.RenameOverloads END ',GetObjName(DeclEl));
  {$ENDIF}
end;

procedure TPas2JSResolver.RenameSubOverloads(Declarations: TFPList);
var
  i, OldScopeCount: Integer;
  El: TPasElement;
  Proc, ImplProc: TPasProcedure;
  ProcScope, ImplProcScope: TPas2JSProcedureScope;
  C: TClass;
  ProcBody: TProcedureBody;
begin
  for i:=0 to Declarations.Count-1 do
    begin
    El:=TPasElement(Declarations[i]);
    C:=El.ClassType;
    if C.InheritsFrom(TPasProcedure) then
      begin
      Proc:=TPasProcedure(El);
      ProcScope:=Proc.CustomData as TPas2JSProcedureScope;

      ImplProc:=ProcScope.ImplProc;
      if ImplProc<>nil then
        ImplProcScope:=TPas2JSProcedureScope(ImplProc.CustomData)
      else
        begin
        ImplProc:=Proc;
        ImplProcScope:=ProcScope;
        end;
      {$IFDEF VerbosePas2JS}
      //writeln('TPas2JSResolver.RenameSubOverloads ImplProc=',ImplProc.Name,' DeclarationProc=',GetObjName(ProcScope.DeclarationProc),' ClassScope=',GetObjName(ImplProcScope.ClassOrRecordScope));
      {$ENDIF}
      ProcBody:=ImplProc.Body;
      if (ProcBody<>nil) and (not ImplProcScope.BodyOverloadsRenamed) then
        begin
        ImplProcScope.BodyOverloadsRenamed:=true;
        OldScopeCount:=FOverloadScopes.Count;
        if (ImplProcScope.ClassRecScope<>nil)
            and not (Proc.Parent is TPasMembersType) then
          begin
          // push class scopes
          PushOverloadClassOrRecScopes(ImplProcScope.ClassRecScope,true);
          end;
        PushOverloadScope(ImplProcScope);
        // first rename all overloads on this level
        RenameOverloads(ProcBody,ProcBody.Declarations);
        // then process nested procedures
        RenameSubOverloads(ProcBody.Declarations);
        PopOverloadScope;
        RestoreOverloadScopeLvl(OldScopeCount);
        end;
      end
    else if (C=TPasClassType) or (C=TPasRecordType) then
      RenameMembers(TPasMembersType(El));
    end;
  {$IFDEF VerbosePas2JS}
  //writeln('TPas2JSResolver.RenameSubOverloads END');
  {$ENDIF}
end;

procedure TPas2JSResolver.RenameMembers(El: TPasMembersType);
var
  OldScopeCount: Integer;
  ClassEl: TPasClassType;
  ClassOrRecScope: TPasClassOrRecordScope;
  RecScope: TPas2JSRecordScope;
  ClassScope: TPas2JSClassScope;
begin
  OldScopeCount:=FOverloadScopes.Count;
  if El.ClassType=TPasClassType then
    begin
    ClassEl:=TPasClassType(El);
    if ClassEl.IsForward then exit;
    // add class and ancestor scopes
    ClassScope:=El.CustomData as TPas2JSClassScope;
    if ClassScope.MemberOverloadsRenamed then exit;
    ClassScope.MemberOverloadsRenamed:=true;
    ClassOrRecScope:=ClassScope;
    end
  else
    begin
    // add record scope
    RecScope:=El.CustomData as TPas2JSRecordScope;
    if RecScope.MemberOverloadsRenamed then exit;
    RecScope.MemberOverloadsRenamed:=true;
    ClassOrRecScope:=RecScope;
    end;
  PushOverloadClassOrRecScopes(ClassOrRecScope,false);

  // first rename all overloads on this level
  RenameOverloads(El,El.Members);
  // then process nested procedures
  RenameSubOverloads(El.Members);

  // restore scope
  RestoreOverloadScopeLvl(OldScopeCount);
end;

procedure TPas2JSResolver.RenameSpecialized(SpecializedItem: TPRSpecializedItem
  );
var
  GenScope: TPasGenericScope;
  NewName: String;
begin
  if SpecializedItem=nil then exit;
  NewName:=SpecializedItem.GenericEl.Name+'$G'+IntToStr(SpecializedItem.Index+1);
  GenScope:=TPasGenericScope(SpecializedItem.SpecializedEl.CustomData);
  if GenScope is TPas2JSClassScope then
    TPas2JSClassScope(GenScope).JSName:=NewName
  else if GenScope is TPas2JSRecordScope then
    TPas2JSRecordScope(GenScope).JSName:=NewName
  else if GenScope is TPas2JSArrayScope then
    TPas2JSArrayScope(GenScope).JSName:=NewName
  else if GenScope is TPas2JSProcTypeScope then
    TPas2JSProcTypeScope(GenScope).JSName:=NewName
  else if GenScope is TPas2JSProcedureScope then
    // handled in GetOverloadName
  else
    RaiseNotYetImplemented(20200906203342,SpecializedItem.SpecializedEl,GetObjName(GenScope));
  {$IFDEF VerbosePas2JS}
  writeln('TPas2JSResolver.RenameSpecialized GenericEl=',GetObjPath(SpecializedItem.GenericEl),' Spec=',GetObjPath(SpecializedItem.SpecializedEl),' JSName="',NewName,'"');
  {$ENDIF}
end;

procedure TPas2JSResolver.PushOverloadScopeSkip;
begin
  FOverloadScopes.Add(TPas2JSOverloadChgThisScope.Create);
end;

procedure TPas2JSResolver.PushOverloadScope(Scope: TPasIdentifierScope);
begin
  if (FOverloadScopes.Count>0) and (TObject(FOverloadScopes[FOverloadScopes.Count-1])=Scope) then
    RaiseNotYetImplemented(20200602000045,Scope.Element);
  FOverloadScopes.Add(Scope);
end;

function TPas2JSResolver.PushOverloadClassOrRecScopes(
  Scope: TPasClassOrRecordScope; WithParents: boolean): integer;
var
  CurScope: TPasClassOrRecordScope;
  aParent: TPasElement;
begin
  Result:=FOverloadScopes.Count;
  repeat
    PushOverloadScopeSkip;
    // push class and ancestors
    CurScope:=Scope;
    repeat
      PushOverloadScope(CurScope);
      if CurScope is TPas2JSClassScope then
        CurScope:=TPas2JSClassScope(CurScope).AncestorScope
      else
        break;
    until CurScope=nil;

    if not WithParents then
      exit;
    aParent:=Scope.Element.Parent;
    if not (aParent is TPasMembersType) then
      exit;
    // nested class -> push parent class scope...
    Scope:=aParent.CustomData as TPasClassOrRecordScope;
  until false;
end;

procedure TPas2JSResolver.PopOverloadScope;
var
  i: Integer;
  Scope: TPasIdentifierScope;
begin
  i:=FOverloadScopes.Count-1;
  if i<0 then
    RaiseInternalError(20200723125456);
  Scope:=TPasIdentifierScope(FOverloadScopes[i]);
  if Scope.ClassType=TPas2JSOverloadChgThisScope then
    Scope.Free;
  FOverloadScopes.Delete(i);
end;

procedure TPas2JSResolver.RestoreOverloadScopeLvl(OldScopeCount: integer);
begin
  while FOverloadScopes.Count>OldScopeCount do
    PopOverloadScope;
end;

procedure TPas2JSResolver.ClearOverloadScopes;
begin
  if FOverloadScopes=nil then exit;
  while FOverloadScopes.Count>0 do
    PopOverloadScope;
  FreeAndNil(FOverloadScopes);
end;

procedure TPas2JSResolver.AddType(El: TPasType);
begin
  inherited AddType(El);
  if El.Parent is TProcedureBody then
    // local type
    AddElevatedLocal(El);
end;

procedure TPas2JSResolver.AddRecordType(El: TPasRecordType; TypeParams: TFPList
  );
begin
  inherited;
  if (El.Name='') and (El.Parent.ClassType<>TPasVariant) then
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPas2JSResolver.AddRecordType ',GetObjName(El.Parent));
    {$ENDIF}
    RaiseNotYetImplemented(20190408224556,El,'anonymous record type');
    end;
  if El.Parent is TProcedureBody then
    // local record
    AddElevatedLocal(El);
end;

procedure TPas2JSResolver.AddClassType(El: TPasClassType; TypeParams: TFPList);
begin
  inherited AddClassType(El,TypeParams);
end;

procedure TPas2JSResolver.AddEnumType(El: TPasEnumType);
begin
  inherited AddEnumType(El);
  if El.Parent is TProcedureBody then
    // local enum type
    AddElevatedLocal(El);
end;

procedure TPas2JSResolver.ResolveImplAsm(El: TPasImplAsmStatement);
{type
  TAsmToken = (
    atNone,
    atWord,
    atDot,
    atRoundBracketOpen,
    atRoundBracketClose
    );

  procedure Next;
  begin

  end;}

var
  Lines: TStrings;
begin
  Lines:=El.Tokens;
  if Lines=nil then exit;
  // ToDo: resolve explicit references
end;

procedure TPas2JSResolver.ResolveNameExpr(El: TPasExpr; const aName: string;
  Access: TResolvedRefAccess);

  procedure CheckTObjectFree(Ref: TResolvedReference);
  // Ref is the ComputeElement of El
  var
    Bin: TBinaryExpr;
    Left: TPasExpr;
    LeftResolved: TPasResolverResult;
    IdentEl: TPasElement;
    C: TClass;
  begin
    if not IsTObjectFreeMethod(El) then exit;
    // El is the TPrimitiveExpr of "Free"
    if Ref.WithExprScope<>nil then
      begin
      // with expr do free
      if GetNewInstanceExpr(Ref.WithExprScope.Expr)<>nil then
        exit; // with TSomeClass.Create do Free  -> ok
      RaiseMsg(20170517092407,nFreeNeedsVar,sFreeNeedsVar,[],El);
      end;
    C:=El.Parent.ClassType;
    if (C=TBinaryExpr) then
      begin
      Bin:=TBinaryExpr(El.Parent);
      if (Bin.right<>El) or (Bin.OpCode<>eopSubIdent) then
        RaiseMsg(20170516151950,nFreeNeedsVar,sFreeNeedsVar,[],El);
      // expr.Free
      if rrfImplicitCallWithoutParams in Ref.Flags then
        // ".Free;" -> ok
      else if Bin.Parent is TParamsExpr then
        begin
        if Bin.Parent.Parent is TPasExpr then
          RaiseMsg(20170516161345,nFreeNeedsVar,sFreeNeedsVar,[],El);
        // ".Free();" -> ok
        end
      else if Bin.Parent is TPasImplElement then
        // ok
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JSResolver.ResolveNameExpr.CheckTObjectFree Bin.Parent=',GetObjName(Bin.Parent));
        {$ENDIF}
        RaiseMsg(20170516160347,nFreeNeedsVar,sFreeNeedsVar,[],El);
        end;

      Left:=Bin.left;
      ComputeElement(Left,LeftResolved,[]);
      if not (rrfReadable in LeftResolved.Flags) then
        RaiseMsg(20170516152300,nFreeNeedsVar,sFreeNeedsVar,[],El);
      if not (rrfWritable in LeftResolved.Flags) then
        RaiseMsg(20170516152307,nFreeNeedsVar,sFreeNeedsVar,[],El);
      IdentEl:=LeftResolved.IdentEl;
      if IdentEl=nil then
        RaiseMsg(20170516152401,nFreeNeedsVar,sFreeNeedsVar,[],El);
      if IdentEl.ClassType=TPasArgument then
        exit; // readable and writable argument -> ok
      if (IdentEl.ClassType=TPasVariable)
         or (IdentEl.ClassType=TPasConst) then
        exit; // readable and writable variable -> ok
      if (IdentEl.ClassType=TPasResultElement)
          and (Left is TPrimitiveExpr) then
        begin
        // "Result.Free" -> ok
        exit;
        end;
      {$IFDEF VerbosePas2JS}
      writeln('CheckTObjectFree LeftResolved=',GetResolverResultDbg(LeftResolved));
      {$ENDIF}
      RaiseMsg(20170516152455,nFreeNeedsVar,sFreeNeedsVar,[],El);
      end
    else if C.InheritsFrom(TPasImplBlock) then
      begin
      // e.g.  "begin Free end;"  OR  "if expr then Free;"  -> ok
      exit;
      end;
    RaiseMsg(20170516152454,nFreeNeedsVar,sFreeNeedsVar,[],El);
  end;

  procedure CheckResultEl(Ref: TResolvedReference);
  // Ref.Declaration is TPasResultElement
  var
    CurEl: TPasElement;
    Lvl: Integer;
    ProcScope, CurProcScope: TPas2JSProcedureScope;
    FuncType: TPasFunctionType;
  begin
    // result refers to a function result
    // -> check if it is referring to a parent function result
    Lvl:=0;
    CurEl:=El;
    CurProcScope:=nil;
    while CurEl<>nil do
      begin
      if (CurEl is TPasProcedure)
          and (TPasProcedure(CurEl).ProcType is TPasFunctionType) then
        begin
        inc(Lvl);
        if not (CurEl.CustomData is TPas2JSProcedureScope) then
          RaiseInternalError(20181210231858);
        ProcScope:=TPas2JSProcedureScope(CurEl.CustomData);
        if ProcScope.DeclarationProc is TPasFunction then
          FuncType:=TPasFunctionType(ProcScope.DeclarationProc.ProcType)
        else
          FuncType:=TPasFunctionType(TPasProcedure(CurEl).ProcType);
        if Lvl=1 then
          begin
          // current function (where the statement of El is)
          if (FuncType.ResultEl=Ref.Declaration) then
            exit; // accessing current function -> ok
          // accessing Result variable of higher function -> need rename
          // Note: ProcScope.ResultVarName only valid in implementation ProcScope
          if ProcScope.ResultVarName<>'' then
            exit; // is already renamed
          CurProcScope:=ProcScope;
          end;
        end;
      CurEl:=CurEl.Parent;
      end;
    if Lvl<2 then
      RaiseNotYetImplemented(20171003112020,El);
    // El refers to a higher Result variable
    // -> current function needs another name for its Result variable
    CurProcScope.ResultVarName:=ResolverResultVar+'$'+IntToStr(Lvl-1);
  end;

var
  Ref: TResolvedReference;
begin
  inherited ResolveNameExpr(El, aName, Access);
  if El.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(El.CustomData);
    if (CompareText(aName,'free')=0) then
      CheckTObjectFree(Ref)
    else if (Ref.Declaration is TPasResultElement) then
      CheckResultEl(Ref)
    else if IsExternalClassConstructor(Ref.Declaration) then
      CheckExternalClassConstructor(Ref);
    end;
end;

procedure TPas2JSResolver.ResolveFuncParamsExpr(Params: TParamsExpr;
  Access: TResolvedRefAccess);
var
  Value: TPasExpr;
  Ref: TResolvedReference;
begin
  inherited ResolveFuncParamsExpr(Params, Access);
  Value:=Params.Value;
  if Value.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(Value.CustomData);
    if IsExternalClassConstructor(Ref.Declaration) then
      CheckExternalClassConstructor(Ref);
    end;
end;

procedure TPas2JSResolver.FinishInterfaceSection(Section: TPasSection);
begin
  inherited FinishInterfaceSection(Section);
  if FOverloadScopes=nil then
    begin
    FOverloadScopes:=TFPList.Create;
    RenameOverloadsInSection(Section);
    end;
end;

procedure TPas2JSResolver.FinishTypeSectionEl(El: TPasType);
var
  C: TClass;
  TypeEl: TPasType;
begin
  inherited FinishTypeSectionEl(El);

  C:=El.ClassType;
  if C=TPasPointerType then
    begin
    TypeEl:=ResolveAliasType(TPasPointerType(El).DestType);
    if TypeEl.ClassType=TPasRecordType then
      // ^record
    else
      RaiseMsg(20180423105726,nNotSupportedX,sNotSupportedX,['pointer of '+TPasPointerType(El).DestType.Name],El);
    end;
end;

procedure TPas2JSResolver.FinishModule(CurModule: TPasModule);
var
  ModuleClass: TClass;
begin
  inherited FinishModule(CurModule);
  if FOverloadScopes=nil then
    FOverloadScopes:=TFPList.Create;
  try
    ModuleClass:=CurModule.ClassType;
    if ModuleClass=TPasModule then
      RenameOverloadsInSection(CurModule.ImplementationSection)
    else if ModuleClass=TPasProgram then
      RenameOverloadsInSection(TPasProgram(CurModule).ProgramSection)
    else if CurModule.ClassType=TPasLibrary then
      RenameOverloadsInSection(TPasLibrary(CurModule).LibrarySection)
    else
      RaiseNotYetImplemented(20170221000032,CurModule);
  finally
    ClearOverloadScopes;
  end;
end;

procedure TPas2JSResolver.FinishEnumType(El: TPasEnumType);
var
  i: Integer;
  EnumValue: TPasEnumValue;
begin
  inherited FinishEnumType(El);
  for i:=0 to El.Values.Count-1 do
    begin
    EnumValue:=TPasEnumValue(El.Values[i]);
    if EnumValue.Value<>nil then
      RaiseNotYetImplemented(20180126202434,EnumValue,'enum const');
    end;
end;

procedure TPas2JSResolver.FinishSetType(El: TPasSetType);
var
  TypeEl: TPasType;
  C: TClass;
  RangeValue: TResEvalValue;
  bt: TResolverBaseType;
begin
  inherited FinishSetType(El);
  TypeEl:=ResolveAliasType(El.EnumType);
  C:=TypeEl.ClassType;
  if C=TPasEnumType then
    exit
  else if C=TPasUnresolvedSymbolRef then
    begin
    if TypeEl.CustomData is TResElDataBaseType then
      begin
      bt:=TResElDataBaseType(TypeEl.CustomData).BaseType;
      if bt in [btBoolean,btByte,btShortInt,btSmallInt,btWord,btChar,btWideChar] then
        exit; // ok
      {$IFDEF VerbosePas2JS}
      writeln('TPas2JSResolver.FinishSetType El='+GetObjName(El)+' TypeEl=',GetObjName(TypeEl),' ',bt);
      {$ENDIF}
      RaiseMsg(20171110150000,nNotSupportedX,sNotSupportedX,['set of '+TypeEl.Name],El);
      end;
    end
  else if C=TPasRangeType then
    begin
    RangeValue:=Eval(TPasRangeType(TypeEl).RangeExpr,[refConst]);
    try
      case RangeValue.Kind of
      revkRangeInt:
        begin
        if TResEvalRangeInt(RangeValue).RangeEnd-TResEvalRangeInt(RangeValue).RangeStart>$ffff then
          begin
          {$IFDEF VerbosePas2JS}
          writeln('TPas2JSResolver.FinishSetType El='+GetObjName(El)+' Range='+RangeValue.AsDebugString,' ',bt);
          {$ENDIF}
          RaiseMsg(20171110150159,nNotSupportedX,sNotSupportedX,['set of '+TypeEl.Name],El);
          end;
        exit;
        end;
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JSResolver.FinishSetType El='+GetObjName(El)+' Range='+RangeValue.AsDebugString);
        {$ENDIF}
        RaiseMsg(20171110145211,nNotSupportedX,sNotSupportedX,['set of '+TypeEl.Name],El);
        end;
      end;
    finally
      ReleaseEvalValue(RangeValue);
    end;
    end;
  {$IFDEF VerbosePas2JS}
  writeln('TPas2JSResolver.FinishSetType El='+GetObjName(El)+' TypeEl=',GetObjName(TypeEl));
  {$ENDIF}
  RaiseMsg(20170415182320,nNotSupportedX,sNotSupportedX,['set of '+TypeEl.Name],El);
end;

procedure TPas2JSResolver.FinishRecordType(El: TPasRecordType);
begin
  if (El.Variants<>nil) and (El.Variants.Count>0) then
    RaiseMsg(20180104205309,nXIsNotSupported,sXIsNotSupported,['variant record'],TPasElement(El.Variants[0]));
  inherited FinishRecordType(El);
end;

procedure TPas2JSResolver.FinishClassType(El: TPasClassType);
var
  Scope, CurScope: TPas2JSClassScope;
  Value: TResEvalValue;
begin
  inherited FinishClassType(El);
  if El.IsExternal then
    begin
    if El.ExternalName='' then
      RaiseMsg(20170321151109,nMissingExternalName,sMissingExternalName,[],El);
    AddExternalPath(El.ExternalName,El);
    end;
  if El.IsPacked then
    RaiseMsg(20180326155616,nPasElementNotSupported,sPasElementNotSupported,
      ['packed'],El);
  if El.IsForward then
    exit;

  //writeln('TPas2JSResolver.FinishClassType START ',GetObjName(El));
  Scope:=El.CustomData as TPas2JSClassScope;
  case El.ObjKind of
  okInterface:
    begin
    if not (El.InterfaceType in [citCom,citCorba]) then
      RaiseMsg(20180326155612,nPasElementNotSupported,sPasElementNotSupported,
        [InterfaceTypeNames[El.InterfaceType]],El);
    if El.GUIDExpr<>nil then
      begin
      Value:=Eval(El.GUIDExpr,[refConst]);
      try
        case Value.Kind of
        {$IFDEF FPC_HAS_CPSTRING}
        revkString:
          Scope.GUID:=TResEvalString(Value).S;
        revkUnicodeString:
          Scope.GUID:=UTF8Encode(TResEvalUTF16(Value).S);
        {$ELSE}
        revkUnicodeString:
          Scope.GUID:=TResEvalUTF16(Value).S;
        {$ENDIF}
        else
          RaiseXExpectedButYFound(20180326160602,'string literal',El.GUIDExpr.ElementTypeName,El.GUIDExpr);
        end;
        // test format?
      finally
        ReleaseEvalValue(Value);
      end;
      end
    else
      begin
      // autogenerate GUID
      Scope.GUID:=GenerateGUID(El);
      end;

    CurScope:=Scope;
    repeat
      CurScope:=TPas2JSClassScope(CurScope.AncestorScope);
      if CurScope=nil then break;
      if SameText(CurScope.GUID,Scope.GUID) then
        RaiseMsg(20180330232206,nDuplicateGUIDXInYZ,sDuplicateGUIDXInYZ,
          [Scope.GUID,El.Name,CurScope.Element.Name],El);
    until false;
    end;
  end;

  // clear
  Scope.MsgIntToProc:=nil;
  Scope.MsgStrToProc:=nil;
  //writeln('TPas2JSResolver.FinishClassType END ',GetObjName(El));
end;

procedure TPas2JSResolver.FinishArrayType(El: TPasArrayType);
var
  ElType: TPasType;
begin
  inherited FinishArrayType(El);
  ElType:=ResolveAliasType(El.ElType);
  while ElType is TPasArrayType do
    ElType:=ResolveAliasType(TPasArrayType(ElType).ElType);
  if IsInterfaceType(ElType,citCom) then
    RaiseMsg(20180404134515,nNotSupportedX,sNotSupportedX,['array of COM-interface'],El);
end;

procedure TPas2JSResolver.FinishAncestors(aClass: TPasClassType);
var
  IntfList: TFPList;
  i, j: Integer;
  Scope, IntfScope: TPas2JSClassScope;
  IntfType, OrigIntfType: TPasType;
  GUIDs: TStringList;
begin
  inherited FinishAncestors(aClass);

  if aClass.Parent is TPasRecordType then
    begin
    if not (aClass.ObjKind in ([okClass]+okAllHelpers)) then
      RaiseNotYetImplemented(20190105143752,aClass,GetElementTypeName(aClass)+' inside record');
    end;

  Scope:=TPas2JSClassScope(aClass.CustomData);
  if Scope=nil then exit;

  Scope.DispatchField:=CurrentParser.Scanner.CurrentValueSwitch[vsDispatchField];
  Scope.DispatchStrField:=CurrentParser.Scanner.CurrentValueSwitch[vsDispatchStrField];

  IntfList:=aClass.Interfaces;
  GUIDs:=TStringList.Create;
  try
    for i:=0 to IntfList.Count-1 do
      begin
      OrigIntfType:=TPasType(IntfList[i]);
      IntfType:=ResolveAliasType(OrigIntfType);
      IntfScope:=TPas2JSClassScope(IntfType.CustomData);
      j:=GUIDs.IndexOf(IntfScope.GUID);
      if j>=0 then
        RaiseMsg(20180330231220,nDuplicateGUIDXInYZ,sDuplicateGUIDXInYZ,
          [IntfScope.GUID,OrigIntfType.Name,TpasElement(GUIDs.Objects[j]).Name],aClass); // ToDo: jump to interface expr
      GUIDs.AddObject(IntfScope.GUID,OrigIntfType);
      end;
  finally
    GUIDs.Free;
  end;
end;

procedure TPas2JSResolver.FinishVariable(El: TPasVariable);
const
  ClassFieldModifiersAllowed = [vmClass,vmStatic,vmExternal,vmPublic];
  RecordVarModifiersAllowed = [vmClass,vmStatic,vmExternal,vmPublic];
  LocalVarModifiersAllowed = [];
  ImplementationVarModifiersAllowed = [vmExternal];
  SectionVarModifiersAllowed = [vmExternal,vmPublic];

  procedure RaiseVarModifierNotSupported(const Allowed: TVariableModifiers);
  var
    s: String;
    m: TVariableModifier;
  begin
    s:='';
    for m in TVariableModifiers do
      if (m in El.VarModifiers) and not (m in Allowed) then
        begin
        str(m,s);
        RaiseMsg(20170322134418,nInvalidVariableModifier,
          sInvalidVariableModifier,[VariableModifierNames[m]],El);
        end;
  end;

var
  ExtName: String;
  ParentC: TClass;
  AbsExpr: TPasExpr;
  ResolvedAbsol: TPasResolverResult;
  AbsIdent: TPasElement;
  TypeEl, ElTypeEl: TPasType;
  GUID: TGUID;
begin
  inherited FinishVariable(El);

  ParentC:=El.Parent.ClassType;

  if El.AbsoluteExpr<>nil then
    begin
    // check 'absolute' alias
    if vmExternal in El.VarModifiers then
      RaiseMsg(20171226105002,nXModifierMismatchY,sXModifierMismatchY,
        ['absolute','external'],El.AbsoluteExpr);
    AbsExpr:=El.AbsoluteExpr;
    ComputeElement(AbsExpr,ResolvedAbsol,[rcNoImplicitProc]);
    AbsIdent:=ResolvedAbsol.IdentEl;
    if ParentC=TProcedureBody then
      begin
      // local var
      if (AbsIdent.Parent is TProcedureBody)
          or (AbsIdent is TPasArgument) then
        // ok
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JSResolver.FinishVariable absolute: El.Parent=',GetObjName(El.Parent),'.Parent=',GetObjName(El.Parent.Parent),' AbsParent=',GetObjName(AbsIdent.Parent),'.Parent=',GetObjName(AbsIdent.Parent.Parent));
        {$ENDIF}
        RaiseMsg(20171226102424,nInvalidAbsoluteLocation,sInvalidAbsoluteLocation,[],El.AbsoluteExpr);
        end;
      end
    else
      begin
      RaiseMsg(20170728133340,nInvalidVariableModifier,
        sInvalidVariableModifier,['absolute'],El);
      end;
    end;

  if (ParentC=TPasClassType) then
    begin
    // class member
    RaiseVarModifierNotSupported(ClassFieldModifiersAllowed);
    if TPasClassType(El.Parent).IsExternal then
      begin
      // external class
      if El.Visibility=visPublished then
        // Note: an external class has no typeinfo
        RaiseMsg(20170413221516,nSymbolCannotBePublished,sSymbolCannotBePublished,
          [],El);
      if not (vmExternal in El.VarModifiers) then
        begin
        // make variable external
        if (El.ClassType=TPasVariable) or (El.ClassType=TPasConst) then
          begin
          if El.ExportName<>nil then
            RaiseMsg(20170322134321,nInvalidVariableModifier,
              sInvalidVariableModifier,['export name'],El.ExportName);
          El.ExportName:=TPrimitiveExpr.Create(El,pekString,''''+El.Name+'''');
          end;
        Include(El.VarModifiers,vmExternal);
        end;
      if (El.ClassType=TPasConst) and (TPasConst(El).Expr<>nil) then
        // external const with expression is not writable
        TPasConst(El).IsConst:=true;
      end;
    end
  else if ParentC=TPasRecordType then
    begin
    // record member
    RaiseVarModifierNotSupported(RecordVarModifiersAllowed);
    if IsInterfaceType(El.VarType,citCom) then
      RaiseMsg(20180404135105,nNotSupportedX,sNotSupportedX,['COM-interface as record member'],El);
    if (El.ClassType=TPasConst) and (TPasConst(El).Expr<>nil)
        and (vmExternal in TPasConst(El).VarModifiers) then
      // external const with expression is not writable
      TPasConst(El).IsConst:=true;
    end
  else if ParentC=TProcedureBody then
    begin
    // local var
    RaiseVarModifierNotSupported(LocalVarModifiersAllowed);
    if El.ClassType=TPasConst then
      begin
      // local const. Can be writable!
      AddElevatedLocal(El);
      end;
    end
  else if ParentC=TPasImplExceptOn then
    // except on var
    RaiseVarModifierNotSupported(LocalVarModifiersAllowed)
  else if ParentC=TImplementationSection then
    // implementation var
    RaiseVarModifierNotSupported(ImplementationVarModifiersAllowed)
  else if ParentC.InheritsFrom(TPasSection) then
    begin
    // interface/program/library var
    RaiseVarModifierNotSupported(SectionVarModifiersAllowed);
    end
  else
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPas2JSResolver.FinishVariable ',GetObjPath(El));
    {$ENDIF}
    RaiseNotYetImplemented(20170324151259,El);
    end;

  if vmExternal in El.VarModifiers then
    begin
    // compute constant
    if El.LibraryName<>nil then
      RaiseMsg(20170227094227,nPasElementNotSupported,sPasElementNotSupported,
        ['library'],El.ExportName);
    if El.ExportName=nil then
      RaiseMsg(20170227100750,nMissingExternalName,sMissingExternalName,[],El);
    ExtName:=ComputeConstString(El.ExportName,true,true);
    if (El.Visibility=visPublished) and (ExtName<>El.Name) then
      RaiseMsg(20170407002940,nPublishedNameMustMatchExternal,
        sPublishedNameMustMatchExternal,[],El.ExportName);
    // add external name to FExternalNames
    if (El.Parent is TPasSection)
        or ((El.ClassType=TPasConst) and (El.Parent is TPasProcedure)) then
      AddExternalPath(ExtName,El.ExportName);
    end;

  if El.VarType<>nil then
    begin
    TypeEl:=ResolveAliasType(El.VarType);

    if TypeEl.ClassType=TPasPointerType then
      begin
      ElTypeEl:=ResolveAliasType(TPasPointerType(TypeEl).DestType);
      if ElTypeEl.ClassType=TPasRecordType then
        // ^record
      else
        RaiseMsg(20180423110113,nNotSupportedX,sNotSupportedX,['pointer'],El);
      end;

    if El.Expr<>nil then
      begin
      if (TypeEl.ClassType=TPasRecordType) then
        begin
        if GetAssignGUIDString(TPasRecordType(TypeEl),El.Expr,GUID) then
          // e.g. IObjectInstance: TGuid = '{D91C9AF4-3C93-420F-A303-BF5BA82BFD23}'
        else
          ;
        end;
      end;
    end;
end;

procedure TPas2JSResolver.FinishArgument(El: TPasArgument);
var
  TypeEl, ElTypeEl: TPasType;
  C: TClass;
begin
  inherited FinishArgument(El);
  if El.ArgType<>nil then
    begin
    TypeEl:=ResolveAliasType(El.ArgType);
    C:=TypeEl.ClassType;

    if C=TPasPointerType then
      begin
      ElTypeEl:=ResolveAliasType(TPasPointerType(TypeEl).DestType);
      if ElTypeEl.ClassType=TPasRecordType then
        // ^record
      else
        RaiseMsg(20180423110239,nNotSupportedX,sNotSupportedX,['pointer'],El);
      end;

    if El.Access=argConstRef then
      begin
      if (C=TPasRecordType) or (C=TPasArrayType) then
        // argConstRef works same as argConst for records -> ok
      else
        LogMsg(20191215133912,mtWarning,nConstRefNotForXAsConst,sConstRefNotForXAsConst,
          [GetElementTypeName(TypeEl)],El);
      end;
    end;
end;

procedure TPas2JSResolver.FinishProcedureType(El: TPasProcedureType);
var
  Proc: TPasProcedure;
  pm: TProcedureModifier;
  ExtName: String;
  C: TClass;
  AClassOrRec: TPasMembersType;
  ClassOrRecScope: TPasClassOrRecordScope;
  AClass: TPasClassType;
  ClassScope: TPas2JSClassScope;
  ptm: TProcTypeModifier;
  TypeEl, ElTypeEl, HelperForType: TPasType;
  FuncType: TPasFunctionType;
begin
  inherited FinishProcedureType(El);

  if El is TPasFunctionType then
    begin
    FuncType:=TPasFunctionType(El);
    if FuncType.ResultEl<>nil then
      begin
      TypeEl:=ResolveAliasType(FuncType.ResultEl.ResultType);
      if TypeEl.ClassType=TPasPointerType then
        begin
        ElTypeEl:=ResolveAliasType(TPasPointerType(TypeEl).DestType);
        if ElTypeEl.ClassType=TPasRecordType then
          // ^record
        else
          RaiseMsg(20180423110824,nNotSupportedX,sNotSupportedX,['pointer'],El);
        end;
      end;
    end;

  if El.Parent is TPasProcedure then
    begin
    Proc:=TPasProcedure(El.Parent);

    // calling convention
    if El.CallingConvention<>ccDefault then
      RaiseMsg(20170211214731,nNotSupportedX,sNotSupportedX,
        [cCallingConventions[El.CallingConvention]],Proc);

    for pm in Proc.Modifiers do
      if (not (pm in [pmVirtual, pmAbstract, pmOverride,
                      pmOverload, pmMessage, pmReintroduce,
                      pmInline, pmAssembler, pmPublic,
                      pmExternal, pmForward])) then
        RaiseNotYetImplemented(20170208142159,El,'modifier '+ModifierNames[pm]);
    for ptm in Proc.ProcType.Modifiers do
      if (not (ptm in [ptmOfObject,ptmVarargs,ptmStatic,ptmAsync])) then
        RaiseNotYetImplemented(20170411171454,El,'modifier '+ProcTypeModifiers[ptm]);

    // check pmPublic
    if [pmPublic,pmExternal]<=Proc.Modifiers then
      RaiseMsg(20170324150149,nInvalidXModifierY,
        sInvalidXModifierY,[Proc.ElementTypeName,'public, external'],Proc);
    if (Proc.PublicName<>nil) then
      RaiseMsg(20170324150417,nPasElementNotSupported,sPasElementNotSupported,
        ['public name'],Proc.PublicName);

    // modifier dispid
    if Proc.DispIDExpr<>nil then
      RaiseMsg(20190303225224,nPasElementNotSupported,sPasElementNotSupported,
        ['dispid'],Proc.DispIDExpr);

    // modifier message
    if Proc.MessageExpr<>nil then
      begin
      if (not (Proc.Parent is TPasClassType))
          or (TPasClassType(Proc.Parent).ObjKind<>okClass) then
        RaiseMsg(20190303231445,nInvalidXModifierY,sInvalidXModifierY,['message','at non class method'],Proc.MessageExpr);
      if TPasClassType(Proc.Parent).IsExternal then
        RaiseMsg(20190304002235,nInvalidXModifierY,sInvalidXModifierY,['message','in external class'],Proc.MessageExpr);
      AddMessageIdToClassScope(Proc,true);
      end;

    if Proc.Parent is TPasMembersType then
      begin
      // class/record member
      AClassOrRec:=TPasMembersType(Proc.Parent);
      ClassOrRecScope:=AClassOrRec.CustomData as TPasClassOrRecordScope;
      if ClassOrRecScope is TPas2JSClassScope then
        begin
        AClass:=TPasClassType(AClassOrRec);
        ClassScope:=TPas2JSClassScope(ClassOrRecScope);
        if AClass.IsExternal then
          begin
          // external class -> make method external
          if not (pmExternal in Proc.Modifiers) then
            begin
            if Proc.LibrarySymbolName<>nil then
              RaiseMsg(20170322142158,nInvalidXModifierY,
                sInvalidXModifierY,[Proc.ElementTypeName,'symbol name'],Proc.LibrarySymbolName);
            Proc.Modifiers:=Proc.Modifiers+[pmExternal];
            Proc.LibrarySymbolName:=TPrimitiveExpr.Create(Proc,pekString,''''+Proc.Name+'''');
            end;

          if Proc.Visibility=visPublished then
            // Note: an external class has no typeinfo
            RaiseMsg(20170413221327,nSymbolCannotBePublished,sSymbolCannotBePublished,
              [],Proc);

          C:=Proc.ClassType;
          if (C=TPasProcedure) or (C=TPasFunction) then
            // ok
          else if (C=TPasClassProcedure) or (C=TPasClassFunction) then
            // ok
          else if C=TPasConstructor then
            begin
            if Proc.IsVirtual then
              // constructor of external class can't be overriden -> forbid virtual
              RaiseMsg(20170323100447,nInvalidXModifierY,sInvalidXModifierY,
                [Proc.ElementTypeName,'virtual,external'],Proc);
            ComputeConstString(Proc.LibrarySymbolName,true,true);
            end
          else
            RaiseMsg(20170322163210,nPasElementNotSupported,sPasElementNotSupported,
              [Proc.ElementTypeName],Proc);

          end
        else
          // Pascal class, not external
          case AClass.ObjKind of
          okClass:
            begin
            if (ClassScope.NewInstanceFunction=nil)
                and (Proc.ClassType=TPasClassFunction)
                and (ClassScope.AncestorScope<>nil)
                and (TPasClassType(ClassScope.AncestorScope.Element).IsExternal)
                and (Proc.Visibility in [visProtected,visPublic,visPublished])
                and (TPasClassFunction(Proc).FuncType.ResultEl.ResultType=AClassOrRec)
                and (Proc.Modifiers-[pmVirtual,pmAssembler]=[])
                and (Proc.ProcType.Modifiers*[ptmOfObject]=[ptmOfObject]) then
              begin
              // The first non private class function in a Pascal class descending
              // from an external class
              // -> this is the NewInstance function
              ClassScope.NewInstanceFunction:=TPasClassFunction(Proc);
              CheckNewInstanceFunction(ClassScope);
              end;
            end;
          okInterface:
            begin
            for pm in Proc.Modifiers do
              if not (pm in [pmOverload, pmReintroduce]) then
                RaiseMsg(20180329141108,nInvalidXModifierY,
                  sInvalidXModifierY,[Proc.ElementTypeName,ModifierNames[pm]],Proc);
            end;
          okClassHelper,okRecordHelper,okTypeHelper:
            begin
            HelperForType:=ResolveAliasType(AClass.HelperForType);
            if HelperForType.ClassType=TPasClassType then
              begin
              if TPasClassType(HelperForType).IsExternal then
                begin
                // method of a class helper for external class
                if IsClassMethod(Proc) and not (ptmStatic in El.Modifiers) then
                  RaiseMsg(20190201165259,nHelperClassMethodForExtClassMustBeStatic,
                    sHelperClassMethodForExtClassMustBeStatic,[],El);
                if Proc.ClassType=TPasConstructor then
                  RaiseNotYetImplemented(20190206153655,El);
                end;
              end;
            if Proc.IsExternal then
              begin
              if not (HelperForType is TPasMembersType) then
                RaiseMsg(20190314225457,nNotSupportedX,sNotSupportedX,['external method in type helper'],El);
              end;
            end;
          end;
        end
      else
        begin
        AClass:=nil;
        ClassScope:=nil;
        end;
      end;

    if pmExternal in Proc.Modifiers then
      begin
      // external proc

      // external override -> unneeded information, probably a bug
      if Proc.IsOverride then
        RaiseMsg(20170321101715,nInvalidXModifierY,sInvalidXModifierY,
          [Proc.ElementTypeName,'override,external'],Proc);

      if Proc.LibraryExpr<>nil then
        RaiseMsg(20170211220712,nPasElementNotSupported,sPasElementNotSupported,
          ['external library name'],Proc.LibraryExpr);
      if Proc.LibrarySymbolName=nil then
        RaiseMsg(20170227095454,nMissingExternalName,sMissingExternalName,
          ['missing external name'],Proc);

      for pm in [pmAssembler,pmForward,pmNoReturn,pmInline] do
        if pm in Proc.Modifiers then
          RaiseMsg(20170323100842,nInvalidXModifierY,sInvalidXModifierY,
            [Proc.ElementTypeName,ModifierNames[pm]],Proc);

      // compute external name
      ExtName:=ComputeConstString(Proc.LibrarySymbolName,true,true);

      // a virtual must have the external name, so that override works
      if Proc.IsVirtual and (Proc.Name<>ExtName) then
        RaiseMsg(20170321090049,nVirtualMethodNameMustMatchExternal,
          sVirtualMethodNameMustMatchExternal,[],Proc.LibrarySymbolName);

      // a published must have the external name, so that streaming works
      if (Proc.Visibility=visPublished) then
        begin
        if (Proc.Name<>ExtName) then
          RaiseMsg(20170407002940,nPublishedNameMustMatchExternal,
            sPublishedNameMustMatchExternal,[],Proc.LibrarySymbolName);
        if ExtName=ExtClassBracketAccessor then
          RaiseMsg(20170409211805,nSymbolCannotBePublished,
            sSymbolCannotBePublished,[],Proc.LibrarySymbolName);
        end;

      if Proc.Parent is TPasSection then
        AddExternalPath(ExtName,Proc.LibrarySymbolName);

      end;
    end
  else
    begin
    // proc type, not proc
    if not (El.CallingConvention in [ccDefault,ccSafeCall]) then
      RaiseMsg(20200516134717,nNotSupportedX,sNotSupportedX,
        [cCallingConventions[El.CallingConvention]],El);
    end;
end;

procedure TPas2JSResolver.FinishProperty(PropEl: TPasProperty);
var
  Getter, Setter: TPasElement;
  GetterIsBracketAccessor, SetterIsBracketAccessor: Boolean;
  Arg: TPasArgument;
  ArgResolved: TPasResolverResult;
  ParentC: TClass;
  IndexExpr: TPasExpr;
  PropArgs: TFPList;
begin
  inherited FinishProperty(PropEl);

  ParentC:=PropEl.Parent.ClassType;
  if (ParentC=TPasClassType) then
    begin
    // class member
    if TPasClassType(PropEl.Parent).IsExternal then
      begin
      // external class
      if PropEl.Visibility=visPublished then
        // Note: an external class has no typeinfo
        RaiseMsg(20170413221703,nSymbolCannotBePublished,sSymbolCannotBePublished,
          [],PropEl);
      end;
    end
  else if ParentC=TPasRecordType then
    // record member
  else
    RaiseNotYetImplemented(20190105144817,PropEl);

  Getter:=GetPasPropertyGetter(PropEl);
  GetterIsBracketAccessor:=IsExternalBracketAccessor(Getter);
  Setter:=GetPasPropertySetter(PropEl);
  SetterIsBracketAccessor:=IsExternalBracketAccessor(Setter);
  IndexExpr:=GetPasPropertyIndex(PropEl);
  PropArgs:=GetPasPropertyArgs(PropEl);
  if GetterIsBracketAccessor then
    begin
    if (PropArgs.Count<>1) or (IndexExpr<>nil) then
      RaiseMsg(20170403001743,nBracketAccessorOfExternalClassMustHaveOneParameter,
        sBracketAccessorOfExternalClassMustHaveOneParameter,
        [],PropEl);
    end;
  if SetterIsBracketAccessor then
    begin
    if (PropArgs.Count<>1) or (IndexExpr<>nil) then
      RaiseMsg(20170403001806,nBracketAccessorOfExternalClassMustHaveOneParameter,
        sBracketAccessorOfExternalClassMustHaveOneParameter,
        [],PropEl);
    end;
  if GetterIsBracketAccessor or SetterIsBracketAccessor then
    begin
    Arg:=TPasArgument(PropArgs[0]);
    if not (Arg.Access in [argDefault,argConst]) then
      RaiseMsg(20170403090225,nXExpectedButYFound,sXExpectedButYFound,
        ['default or "const"',AccessNames[Arg.Access]],PropEl);
    ComputeElement(Arg,ArgResolved,[rcType],Arg);
    if not (ArgResolved.BaseType in (btAllJSInteger+btAllJSStringAndChars+btAllJSBooleans+btAllJSFloats)) then
      RaiseMsg(20170403090628,nIncompatibleTypesGotExpected,
        sIncompatibleTypesGotExpected,
        [GetResolverResultDescription(ArgResolved,true),'string'],Arg);
    end;
end;

procedure TPas2JSResolver.FinishProcParamAccess(ProcType: TPasProcedureType;
  Params: TParamsExpr);
begin
  inherited FinishProcParamAccess(ProcType, Params);
  FindCreatorArrayOfConst(ProcType.Args,Params);
end;

procedure TPas2JSResolver.FinishPropertyParamAccess(Params: TParamsExpr;
  Prop: TPasProperty);
var
  Args: TFPList;
begin
  inherited FinishPropertyParamAccess(Params, Prop);
  Args:=GetPasPropertyArgs(Prop);
  if Args=nil then
    RaiseNotYetImplemented(20190215210914,Params,GetObjName(Prop));
  FindCreatorArrayOfConst(Args,Params);
end;

procedure TPas2JSResolver.FinishExportSymbol(El: TPasExportSymbol);
var
  ResolvedEl: TPasResolverResult;
  DeclEl: TPasElement;
  Proc: TPasProcedure;
begin
  if El.Parent is TLibrarySection then
    // ok
  else
    // everywhere else: not supported
    RaiseMsg(20210106224720,nNotSupportedX,sNotSupportedX,['non library export'],El.ExportIndex);
  if El.ExportIndex<>nil then
    RaiseMsg(20210106223403,nNotSupportedX,sNotSupportedX,['export index'],El.ExportIndex);

  inherited FinishExportSymbol(El);

  ComputeElement(El,ResolvedEl,[]);
  DeclEl:=ResolvedEl.IdentEl;
  if DeclEl=nil then
    RaiseMsg(20210106223620,nSymbolCannotBeExportedFromALibrary,
      sSymbolCannotBeExportedFromALibrary,[],El)
  else if DeclEl is TPasProcedure then
    begin
    Proc:=TPasProcedure(DeclEl);
    if Proc.Parent is TPasSection then
      // ok
    else
      RaiseMsg(20210106224436,nSymbolCannotBeExportedFromALibrary,
        sSymbolCannotBeExportedFromALibrary,[],El);
    end
  else
    RaiseMsg(20210106223621,nSymbolCannotBeExportedFromALibrary,
      sSymbolCannotBeExportedFromALibrary,[],El);
end;

procedure TPas2JSResolver.FindCreatorArrayOfConst(Args: TFPList;
  ErrorEl: TPasElement);
var
  i: Integer;
  Arg: TPasArgument;
begin
  for i:=0 to Args.Count-1 do
    begin
    Arg:=TPasArgument(Args[i]);
    if not IsArrayOfConst(Arg.ArgType) then continue;
    FindProc_ArrLitToArrayOfConst(ErrorEl);
    end;
end;

function TPas2JSResolver.FindProc_ArrLitToArrayOfConst(ErrorEl: TPasElement
  ): TPasFunction;
var
  aMod, UtilsMod: TPasModule;
  ModScope: TPas2JSModuleScope;
  SectionScope: TPasSectionScope;
  Identifier: TPasIdentifier;
  El: TPasElement;
  FuncType: TPasFunctionType;
begin
  aMod:=RootElement;
  ModScope:=aMod.CustomData as TPas2JSModuleScope;
  Result:=ModScope.SystemVarRecs;
  if Result<>nil then exit;

  // find unit in uses clauses
  UtilsMod:=FindUsedUnitname('system',aMod);
  if UtilsMod=nil then
    RaiseIdentifierNotFound(20190215211531,'System.VarRecs',ErrorEl);

  // find class in interface
  if UtilsMod.InterfaceSection=nil then
    RaiseIdentifierNotFound(20190215211538,'System.VarRecs',ErrorEl);

  // find function VarRecs
  SectionScope:=NoNil(UtilsMod.InterfaceSection.CustomData) as TPasSectionScope;
  Identifier:=SectionScope.FindLocalIdentifier('VarRecs');
  if Identifier=nil then
    RaiseIdentifierNotFound(20190215211551,'System.VarRecs',ErrorEl);
  El:=Identifier.Element;
  if El.ClassType<>TPasFunction then
    RaiseXExpectedButYFound(20190215211559,'function System.VarRecs',GetElementTypeName(El),ErrorEl);
  Result:=TPasFunction(El);
  ModScope.SystemVarRecs:=Result;

  // check signature
  FuncType:=Result.ProcType as TPasFunctionType;
  if FuncType.Args.Count>0 then
    RaiseXExpectedButYFound(20190215211953,'function System.VarRecs with 0 args',
      IntToStr(FuncType.Args.Count),ErrorEl);
  if FuncType.Modifiers<>[ptmVarargs] then
    RaiseXExpectedButYFound(20190215212151,'function System.VarRecs; varargs',
      '?',ErrorEl);
  if FuncType.CallingConvention<>ccDefault then
    RaiseXExpectedButYFound(20190215211824,'function System.VarRecs with default calling convention',
      cCallingConventions[FuncType.CallingConvention],ErrorEl);
end;

function TPas2JSResolver.FindSystemExternalClassType(const aClassName,
  JSName: string; ErrorEl: TPasElement): TPasClassType;
var
  Data: TPRFindExtSystemClass;
  Abort: boolean;
begin
  Data:=Default(TPRFindExtSystemClass);
  Data.ErrorPosEl:=ErrorEl;
  Data.JSName:=JSName;
  Abort:=false;
  IterateElements(aClassName,@OnFindExtSystemClass,@Data,Abort);
  Result:=Data.Found;
  if (ErrorEl<>nil) and (Result=nil) then
    RaiseIdentifierNotFound(20200526095647,aClassName+' = class external name '''+JSName+'''',ErrorEl);
end;

function TPas2JSResolver.FindTJSPromise(ErrorEl: TPasElement): TPasClassType;
var
  aMod: TPasModule;
  ModScope: TPas2JSModuleScope;
begin
  aMod:=RootElement;
  ModScope:=aMod.CustomData as TPas2JSModuleScope;
  Result:=ModScope.JSPromiseClass;
  if p2msfPromiseSearched in ModScope.FlagsJS then
    exit; // use cache
  Result:=FindSystemExternalClassType('TJSPromise','Promise',ErrorEl);
  ModScope.JSPromiseClass:=Result;
  Include(ModScope.FlagsJS,p2msfPromiseSearched);
end;

procedure TPas2JSResolver.CheckExternalClassConstructor(Ref: TResolvedReference
  );
var
  TypeEl: TPasType;
begin
  if not (Ref.Context is TResolvedRefCtxConstructor) then
    RaiseMsg(20180511165144,nJSNewNotSupported,sJSNewNotSupported,[],Ref.Element);
  TypeEl:=TResolvedRefCtxConstructor(Ref.Context).Typ;
  if TypeEl.ClassType=TPasClassType then
    begin
    // ClassType.new
    if not TPasClassType(TypeEl).IsExternal then
      RaiseMsg(20180511165316,nJSNewNotSupported,sJSNewNotSupported,[],Ref.Element);
    end
  else if TypeEl.ClassType=TPasClassOfType then
    begin
    TypeEl:=ResolveAliasType(TPasClassOfType(TypeEl).DestType);
    if TypeEl.ClassType=TPasClassType then
      begin
      // ClassOfVar.new
      if not TPasClassType(TypeEl).IsExternal then
        RaiseMsg(20180511175309,nJSNewNotSupported,sJSNewNotSupported,[],Ref.Element);
      end;
    end;
end;

procedure TPas2JSResolver.CheckConditionExpr(El: TPasExpr;
  const ResolvedEl: TPasResolverResult);
begin
  if (ResolvedEl.BaseType=btCustom) and (IsJSBaseType(ResolvedEl,pbtJSValue)) then
    exit;
  inherited CheckConditionExpr(El, ResolvedEl);
end;

procedure TPas2JSResolver.CheckNewInstanceFunction(ClassScope: TPas2JSClassScope
  );
var
  Proc: TPasClassFunction;
  Args: TFPList;
  Arg: TPasArgument;
  ResolvedArg: TPasResolverResult;
begin
  Proc:=ClassScope.NewInstanceFunction;
  // proc modifiers override and external were already checked
  // visibility was already checked
  // function result type was already checked
  if not Proc.IsVirtual then
    RaiseMsg(20170324231040,nNewInstanceFunctionMustBeVirtual,
      sNewInstanceFunctionMustBeVirtual,[],Proc);
  Args:=Proc.ProcType.Args;
  if Args.Count<2 then
    RaiseMsg(20170324232247,nNewInstanceFunctionMustHaveTwoParameters,
      sNewInstanceFunctionMustHaveTwoParameters,[],Proc.ProcType);

  // first param must be a string
  Arg:=TPasArgument(Args[0]);
  if Arg.Access<>argDefault then
    RaiseMsg(20170324232655,nIncompatibleTypeArgNo,sIncompatibleTypeArgNo,
      ['1',AccessNames[Arg.Access],'default (none)'],Arg);
  if Arg.ArgType=nil then
    RaiseMsg(20170324233201,nIncompatibleTypeArgNo,sIncompatibleTypeArgNo,
      ['1','untyped','String'],Arg);
  ComputeElement(Arg.ArgType,ResolvedArg,[rcType]);
  if ResolvedArg.BaseType<>btString then
    RaiseMsg(20170324233348,nIncompatibleTypeArgNo,sIncompatibleTypeArgNo,
      ['1',GetResolverResultDescription(ResolvedArg),'String'],Arg);

  // second param must be const untyped
  Arg:=TPasArgument(Args[1]);
  if Arg.Access<>argConst then
    RaiseMsg(20170324233457,nIncompatibleTypeArgNo,sIncompatibleTypeArgNo,
      ['2',AccessNames[Arg.Access],'const'],Arg);
  if Arg.ArgType<>nil then
    RaiseMsg(20170324233508,nIncompatibleTypeArgNo,sIncompatibleTypeArgNo,
      ['2','type','untyped'],Arg);
end;

function TPas2JSResolver.AddExternalName(const aName: string; El: TPasElement
  ): TPasIdentifier;
var
  Item: TPasIdentifier;
begin
  //writeln('TPas2JSResolver.AddExternalIdentifier Name="',aName,'" El=',GetObjName(El));
  Item:=TPasIdentifier.Create;
  Item.Identifier:=aName;
  Item.Element:=El;

  InternalAdd(Item);
  //writeln('TPas2JSResolver.AddExternalIdentifier END');
  Result:=Item;
end;

function TPas2JSResolver.FindExternalName(const aName: String
  ): TPasIdentifier;
begin
  Result:=TPasIdentifier(FExternalNames.Find(aName));
  {$IFDEF VerbosePasResolver}
  {AllowWriteln}
  if (Result<>nil) and (Result.Owner<>Self) then
    begin
    writeln('TPas2JSResolver.FindExternalName Result.Owner<>Self Owner='+GetObjName(Result.Owner));
    raise Exception.Create('20170322235814');
    end;
  {AllowWriteln-}
  {$ENDIF}
end;

procedure TPas2JSResolver.AddExternalPath(aName: string; El: TPasElement);
// add aName and the first identifier of aName
var
  p: integer;
begin
  aName:=Trim(aName);
  if aName='' then exit;
  AddExternalName(aName,El);
  p:=1;
  while (p<=length(aName)) and (aName[p] in ['a'..'z','A'..'Z','0'..'9','_','$']) do
    inc(p);
  if p>length(aName) then exit;
  AddExternalName(LeftStr(aName,p-1),El);
end;

procedure TPas2JSResolver.AddElevatedLocal(El: TPasElement);
var
  i: Integer;
  ElevatedLocals: TPas2jsElevatedLocals;
  Scope: TPasScope;
  ProcScope: TPas2JSProcedureScope;
begin
  i:=ScopeCount-1;
  while (i>=0) do
    begin
    Scope:=Scopes[i];
    if Scope is TPas2JSProcedureScope then
      begin
      ProcScope:=TPas2JSProcedureScope(Scope);
      if ProcScope.ClassRecScope<>nil then
        Scope:=ProcScope.ClassRecScope;
      end;
    ElevatedLocals:=GetElevatedLocals(Scope);
    if ElevatedLocals<>nil then
      begin
      ElevatedLocals.Add(El.Name,El);
      exit;
      end;
    dec(i);
    end;
  RaiseNotYetImplemented(20180420131358,El);
end;

procedure TPas2JSResolver.ClearElementData;
var
  Data, Next: TPas2JsElementData;
begin
  Data:=FFirstElementData;
  while Data<>nil do
    begin
    Next:=Data.Next;
    Data.Free;
    Data:=Next;
    end;
  FFirstElementData:=nil;
  FLastElementData:=nil;

  FExternalNames.ForEachCall(@OnClearHashItem,nil);
  FExternalNames.Clear;
end;

function TPas2JSResolver.GenerateGUID(El: TPasClassType): string;
var
  Name: String;
  i, BytePos, BitPos, v: Integer;
  Member: TPasElement;
  Bytes: array[0..15] of byte;
  List: TStringList;
  Scope: TPas2JSClassScope;
begin
  Name:=El.PathName;
  Scope:=TPas2JSClassScope(El.CustomData);
  if Scope.AncestorScope<>nil then
    begin
    // use ancestor GUID as start
    Name:=TPas2JSClassScope(Scope.AncestorScope).GUID+Name;
    end;
  List:=TStringList.Create;
  for i:=0 to El.Members.Count-1 do
    begin
    Member:=TPasElement(El.Members[i]);
    if Member is TPasProcedure then
      List.Add(Member.Name);
    end;
  List.Sort;
  for i:=0 to List.Count-1 do
    Name:=Name+','+List[i];
  List.Free;

  BytePos:=0;
  BitPos:=0;
  {$IFDEF fpc}
  FillByte({%H-}Bytes[0],16,0);
  {$ENDIF}
  for i:=1 to length(Name) do
    begin
    // read 16-bit
    v:=(Bytes[BytePos] shl 8)+Bytes[(BytePos+1) and 15];
    // change some bits
    v:=v+integer((ord(Name[i]) shl (11-BitPos)));
    // write 16 bit
    Bytes[BytePos]:=(v shr 8) and $ff;
    Bytes[(BytePos+1) and 15]:=v and $ff;
    inc(BitPos,5);
    if BitPos>7 then
      begin
      dec(BitPos,8);
      BytePos:=(BytePos+1) and 15;
      end;
    end;
  // set version 3
  Bytes[6]:=(Bytes[6] and $f)+(3 shl 4);
  // set variant 2
  Bytes[8]:=(Bytes[8] and $3f)+(2 shl 6);

  Result:='{';
  for i:=0 to 3 do Result:=Result+HexStr(Bytes[i],2);
  Result:=Result+'-';
  for i:=4 to 5 do Result:=Result+HexStr(Bytes[i],2);
  Result:=Result+'-';
  for i:=6 to 7 do Result:=Result+HexStr(Bytes[i],2);
  Result:=Result+'-';
  for i:=8 to 9 do Result:=Result+HexStr(Bytes[i],2);
  Result:=Result+'-';
  for i:=10 to 15 do Result:=Result+HexStr(Bytes[i],2);
  Result:=Result+'}';
end;

function TPas2JSResolver.CheckCallAsyncFuncResult(Param: TPasExpr; out
  ResolvedEl: TPasResolverResult): boolean;
var
  PathEnd: TPasExpr;
  Ref: TResolvedReference;
  Decl: TPasElement;
  DeclFunc: TPasFunction;
begin
  Result:=false;
  PathEnd:=GetPathEndIdent(Param,true);
  if (PathEnd<>nil) and (PathEnd.CustomData is TResolvedReference) then
    begin
    Ref:=TResolvedReference(PathEnd.CustomData);
    Decl:=Ref.Declaration;
    if Decl is TPasFunction then
      begin
      DeclFunc:=TPasFunction(Decl);
      if DeclFunc.IsAsync then
        begin
        // await(CallAsyncFunction)  ->  use Pascal result type (not TJSPromise)
        // Note the missing rcCall flag
        ComputeResultElement(DeclFunc.FuncType.ResultEl,ResolvedEl,[],PathEnd);
        exit(true);
        end;
      end;
    end;
  ResolvedEl:=Default(TPasResolverResult);
end;

procedure TPas2JSResolver.SpecializeGenericIntf(
  SpecializedItem: TPRSpecializedItem);
var
  El: TPasElement;
begin
  inherited SpecializeGenericIntf(SpecializedItem);
  RenameSpecialized(SpecializedItem);
  El:=SpecializedItem.SpecializedEl;
  if (El is TPasGenericType)
      and IsFullySpecialized(TPasGenericType(El))
      and (SpecializeParamsNeedDelay(SpecializedItem)<>nil) then
    TPas2JSResolverHub(Hub).AddJSDelaySpecialize(TPasGenericType(El));
end;

procedure TPas2JSResolver.SpecializeGenericImpl(
  SpecializedItem: TPRSpecializedItem);
var
  El: TPasElement;
begin
  inherited SpecializeGenericImpl(SpecializedItem);

  El:=SpecializedItem.SpecializedEl;
  if El is TPasMembersType then
    begin
    if FOverloadScopes=nil then
      begin
      FOverloadScopes:=TFPList.Create;
      try
        RenameMembers(TPasMembersType(El));
      finally
        ClearOverloadScopes;
      end;
      end;
    end;
end;

procedure TPas2JSResolver.SpecializeProcedure(GenEl, SpecEl: TPasProcedure;
  SpecializedItem: TPRSpecializedItem);
var
  GenProcScope, SpecProcScope: TPas2JSProcedureScope;
begin
  GenProcScope:=GenEl.CustomData as TPas2JSProcedureScope;
  SpecProcScope:=SpecEl.CustomData as TPas2JSProcedureScope;
  if SpecializedItem=nil then
    begin
    SpecProcScope.OverloadName:=GenProcScope.OverloadName;
    SpecProcScope.JSName:=GenProcScope.JSName;
    // SpecProcScope.ResultVarName is set on demand
    end;
  inherited SpecializeProcedure(GenEl, SpecEl, SpecializedItem);
end;

function TPas2JSResolver.SpecializeParamsNeedDelay(
  SpecializedItem: TPRSpecializedItem): TPasElement;
// finds first specialize param defined later than the generic
// For example: generic in the unit interface, param in implementation
// or param in another unit, not used by the generic
var
  Gen: TPasElement;
  GenMod, ParamMod: TPasModule;
  Params: TPasTypeArray;
  Param: TPasType;
  i: Integer;
  GenSection, ParamSection: TPasSection;
  ParamResolver, GenResolver: TPasResolver;
begin
  Result:=nil;
  if SpecializedItem=nil then exit;
  Gen:=SpecializedItem.GenericEl;
  GenSection:=GetParentSection(Gen);
  if not (GenSection is TInterfaceSection) then
    exit; // generic in unit implementation/program/library -> params cannot be defined in a later section -> no delay needed
  GenMod:=nil;
  GenResolver:=nil;

  // ToDo: delay only, if either RTTI or class var using a param

  Params:=SpecializedItem.Params;
  for i:=0 to length(Params)-1 do
    begin
    Param:=ResolveAliasType(Params[i],false);
    if Param.ClassType=TPasUnresolvedSymbolRef then
      continue; // built-in type -> no delay needed
    if (Param.CustomData is TPasGenericScope)
        and (TPasGenericScope(Param.CustomData).GenericStep<psgsInterfaceParsed) then
      exit(Param); // specialization is within param itself -> needs delay
    ParamSection:=GetParentSection(Param);
    if ParamSection=GenSection then
      continue; // same section -> no delay needed
    // not in same section
    ParamMod:=ParamSection.GetModule;
    if GenMod=nil then
      GenMod:=GenSection.GetModule;
    if ParamMod=GenMod then
      exit(Param); // generic in unit interface, param in implementation
    // param in another unit
    if ParamSection is TImplementationSection then
      exit(Param); // generic in unit interface, param in another implementation
    // param in another unit interface
    if GenResolver=nil then
      GenResolver:=GetResolver(GenMod);
    ParamResolver:=GetResolver(ParamMod);
    if (ParamResolver.FinishedInterfaceIndex>GenResolver.FinishedInterfaceIndex)
        or (ParamResolver.FinishedInterfaceIndex=0) // 0 means currently parsing
        then
      exit(Param); // param in a later unit interface
    // generic in a later unit interface -> no delay needed
    end;
end;

function TPas2JSResolver.IsSpecializedNonStaticMethod(
  ProcType: TPasProcedureType): boolean;
var
  Proc: TPasProcedure;
  Scope: TPas2JSProcedureScope;
begin
  if not (ProcType.Parent is TPasProcedure) then
    exit(false); // not a method
  Proc:=TPasProcedure(ProcType.Parent);
  if Proc.IsStatic or Proc.IsExternal then
    exit(false);
  if not (Proc.Parent is TPasMembersType) then
    exit(false); // not a method
  Scope:=TPas2JSProcedureScope(Proc.CustomData);
  if Scope.SpecializedFromItem=nil then
    exit(false);
  Result:=true;
end;

function TPas2JSResolver.AddJSBaseType(const aName: string; Typ: TPas2jsBaseType
  ): TResElDataPas2JSBaseType;
var
  El: TPasUnresolvedSymbolRef;
begin
  El:=AddCustomBaseType(aName,TResElDataPas2JSBaseType);
  if Typ<>pbtNone then
    FJSBaseTypes[Typ]:=El;
  Result:=TResElDataPas2JSBaseType(El.CustomData);
  Result.JSBaseType:=Typ;
end;

function TPas2JSResolver.CheckAssignCompatibilityCustom(const LHS,
  RHS: TPasResolverResult; ErrorEl: TPasElement; RaiseOnIncompatible: boolean;
  var Handled: boolean): integer;
var
  LeftBaseType: TPas2jsBaseType;
  LArray: TPasArrayType;
  ElTypeResolved: TPasResolverResult;
  LTypeEl, RTypeEl: TPasType;
  TIName: String;
begin
  Result:=cIncompatible;
  //writeln('TPas2JSResolver.CheckAssignCompatibilityCustom LHS=',GetResolverResultDbg(LHS),' RHS=',GetResolverResultDbg(RHS));
  if LHS.BaseType=btCustom then
    begin
    if not (LHS.LoTypeEl is TPasUnresolvedSymbolRef) then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPas2JSResolver.CheckAssignCompatibilityCustomBaseType LHS=',GetResolverResultDbg(LHS));
      {$ENDIF}
      RaiseInternalError(20170325114554);
      end;
    if not (LHS.LoTypeEl.CustomData is TResElDataPas2JSBaseType) then
      exit;
    Handled:=true;
    LeftBaseType:=TResElDataPas2JSBaseType(LHS.LoTypeEl.CustomData).JSBaseType;
    if LeftBaseType=pbtJSValue then
      begin
      // assign to a JSValue
      if rrfReadable in RHS.Flags then
        begin
        // RHS is a value
        if (RHS.BaseType in btAllJSValueSrcTypes) then
          Result:=cJSValueConversion // type cast to JSValue
        else if RHS.BaseType=btCustom then
          begin
          if IsJSBaseType(RHS,pbtJSValue) then
            Result:=cExact;
          end
        else if RHS.BaseType=btContext then
          Result:=cJSValueConversion
        else if (RHS.BaseType=btProc) and (RHS.IdentEl=nil) then
          begin
          // JSValue:=anonymousproc
          Result:=cExact;
          end;
        end
      else if RHS.BaseType=btContext then
        begin
        // RHS is not a value
        if RHS.IdentEl<>nil then
          begin
          if RHS.IdentEl.ClassType=TPasClassType then
            Result:=cJSValueConversion; // RHS is a class type
          end;
        end;
      end;
    end
  else if (LHS.BaseType=btContext) then
    begin
    LTypeEl:=LHS.LoTypeEl;
    RTypeEl:=RHS.LoTypeEl;
    if (LTypeEl.ClassType=TPasArrayType)
        and (rrfReadable in RHS.Flags) then
      begin
      LArray:=TPasArrayType(LTypeEl);
      if length(LArray.Ranges)>0 then
        exit;
      if (RHS.BaseType<>btContext) or (RTypeEl.ClassType<>TPasArrayType) then
        exit;
      ComputeElement(GetArrayElType(LArray),ElTypeResolved,[rcType]);
      if IsJSBaseType(ElTypeResolved,pbtJSValue) then
        begin
        // array of jsvalue := array
        Handled:=true;
        Result:=cJSValueConversion;
        end;
      end
    else if (LTypeEl.ClassType=TPasClassType)
        and (rrfReadable in RHS.Flags)
        and (RHS.BaseType=btPointer)
        and IsSameType(RTypeEl,BaseTypes[btPointer],prraNone)
        then
      begin
      TIName:=Pas2JSBuiltInNames[pbivnRTL]+'.'+Pas2JSBuiltInNames[pbitnTI];
      if IsExternalClass_Name(TPasClassType(LTypeEl),TIName) then
        begin
        // aTTypeInfo:=aPointer
        Handled:=true;
        Result:=cTypeConversion;
        end;
      end;
    end;

  if RaiseOnIncompatible then ;
  if ErrorEl=nil then ;
end;

function TPas2JSResolver.CheckTypeCastClassInstanceToClass(const FromClassRes,
  ToClassRes: TPasResolverResult; ErrorEl: TPasElement): integer;
// type cast not related classes
var
  ToClass, FromClass: TPasClassType;
  ToClassScope, FromClassScope: TPas2JSClassScope;
  ToSpecItem, FromSpecItem: TPRSpecializedItem;
  i: Integer;
  ToParam, FromParam: TPasType;
begin
  if FromClassRes.BaseType=btNil then exit(cExact);
  ToClass:=ToClassRes.LoTypeEl as TPasClassType;
  ToClassScope:=ToClass.CustomData as TPas2JSClassScope;
  if ToClassScope.AncestorScope=nil then
    // type cast to root class
    exit(cTypeConversion+1);

  ToSpecItem:=ToClassScope.SpecializedFromItem;
  if ToSpecItem<>nil then
    begin
    FromClass:=FromClassRes.LoTypeEl as TPasClassType;
    FromClassScope:=FromClass.CustomData as TPas2JSClassScope;
    FromSpecItem:=FromClassScope.SpecializedFromItem;
    if FromSpecItem<>nil then
      begin
      // typecast a specialized instance to a specialized type TA<>(aB<>)
      if FromSpecItem.GenericEl=ToSpecItem.GenericEl then
        begin
        // typecast to same generic class
        Result:=cTypeConversion+1;
        for i:=0 to length(FromSpecItem.Params)-1 do
          begin
          FromParam:=FromSpecItem.Params[i];
          ToParam:=ToSpecItem.Params[i];
          if IsSameType(FromParam,ToParam,prraAlias)
              or IsJSBaseType(FromParam,pbtJSValue)
              or IsJSBaseType(ToParam,pbtJSValue) then
            // ok
          else
            begin
            Result:=cIncompatible;
            break;
            end;
          end;
        if Result<cIncompatible then
          exit; // e.g. TGen<JSValue>(aGen<Word>) or TGen<Word>(aGen<JSValue>)
        end;
      end;
    end;

  Result:=cIncompatible;
  if ErrorEl=nil then ;
end;

function TPas2JSResolver.CheckEqualCompatibilityCustomType(const LHS,
  RHS: TPasResolverResult; ErrorEl: TPasElement; RaiseOnIncompatible: boolean
  ): integer;
var
  LeftBaseType: TPas2jsBaseType;
begin
  Result:=cIncompatible;
  if LHS.BaseType=btCustom then
    begin
    if not (LHS.LoTypeEl is TPasUnresolvedSymbolRef) then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPas2JSResolver.CheckEqualCompatibilityCustomType LHS=',GetResolverResultDbg(LHS));
      {$ENDIF}
      RaiseInternalError(20170330005841);
      end;
    if not (LHS.LoTypeEl.CustomData is TResElDataPas2JSBaseType) then
      exit;
    LeftBaseType:=TResElDataPas2JSBaseType(LHS.LoTypeEl.CustomData).JSBaseType;
    if LeftBaseType=pbtJSValue then
      begin
      if (rrfReadable in LHS.Flags) then
        begin
        if (rrfReadable in RHS.Flags) then
          begin
          if RHS.BaseType in btAllJSValueSrcTypes then
            Result:=cJSValueConversion
          else if RHS.BaseType=btCustom then
            begin
            if IsJSBaseType(RHS,pbtJSValue) then
              Result:=cExact;
            end
          else if RHS.BaseType=btContext then
            Result:=cJSValueConversion;
          end
        else if RHS.BaseType=btContext then
          begin
          // right side is not a value
          if RHS.IdentEl<>nil then
            begin
            if RHS.IdentEl.ClassType=TPasClassType then
              Result:=cJSValueConversion; // RHS is a class
            end;
          end;
        end;
      end;
    end
  else if RHS.BaseType=btCustom then
    exit(CheckEqualCompatibilityCustomType(RHS,LHS,ErrorEl,RaiseOnIncompatible))
  else
    RaiseInternalError(20170330005725);
end;

function TPas2JSResolver.CheckForIn(Loop: TPasImplForLoop; const VarResolved,
  InResolved: TPasResolverResult): boolean;
var
  TypeEl: TPasType;
  ArgResolved, LengthResolved, PropResultResolved: TPasResolverResult;
begin
  if InResolved.BaseType=btCustom then
    begin
    if IsJSBaseType(InResolved,pbtJSValue,true) then
      begin
      // for string in jsvalue do ...
      if not (VarResolved.BaseType in btAllStrings) then
        RaiseXExpectedButYFound(20180423185800,'string',GetResolverResultDescription(VarResolved,true),Loop.StartExpr);
      exit(true);
      end;
    end
  else if InResolved.BaseType=btContext then
    begin
    TypeEl:=InResolved.LoTypeEl;
    if (TypeEl.ClassType=TPasClassType) and TPasClassType(TypeEl).IsExternal then
      begin
      // for key in JSClass do ...
      if IsForInExtArray(Loop,VarResolved,InResolved,ArgResolved,
          LengthResolved,PropResultResolved) then
        exit(true);
      // for key in JSObject do
      if not (VarResolved.BaseType in btAllStrings) then
        RaiseXExpectedButYFound(20180423191611,'string',GetResolverResultDescription(VarResolved,true),Loop.StartExpr);
      exit(true);
      end;
    end;
  Result:=false;
end;

procedure TPas2JSResolver.ComputeUnaryNot(El: TUnaryExpr;
  var ResolvedEl: TPasResolverResult; Flags: TPasResolverComputeFlags);
begin
  if ResolvedEl.BaseType=btCustom then
    begin
    if IsJSBaseType(ResolvedEl,pbtJSValue,true) then
      begin
      SetResolverValueExpr(ResolvedEl,btBoolean,BaseTypes[btBoolean],BaseTypes[btBoolean],
                           El,[rrfReadable]);
      exit;
      end;
    end;
  inherited ComputeUnaryNot(El, ResolvedEl, Flags);
end;

procedure TPas2JSResolver.ComputeBinaryExprRes(Bin: TBinaryExpr; out
  ResolvedEl: TPasResolverResult; Flags: TPasResolverComputeFlags;
  var LeftResolved, RightResolved: TPasResolverResult);

  procedure SetBaseType(BaseType: TResolverBaseType);
  begin
    SetResolverValueExpr(ResolvedEl,BaseType,BaseTypes[BaseType],BaseTypes[BaseType],
                         Bin,[rrfReadable]);
  end;

var
  RightTypeEl: TPasType;
begin
  if (LeftResolved.BaseType=btCustom)
      or (RightResolved.BaseType=btCustom) then
    case Bin.OpCode of
    eopIs:
      if IsJSBaseType(LeftResolved,pbtJSValue,true) then
        begin
        // aJSValue is x
        if (RightResolved.IdentEl is TPasType)
            and (ResolveAliasType(TPasType(RightResolved.IdentEl)) is TPasClassType) then
          begin
          // e.g. if aJSValue is TObject then ;
          SetBaseType(btBoolean);
          exit;
          end;
        RightTypeEl:=RightResolved.LoTypeEl;
        if (RightTypeEl is TPasClassOfType) then
          begin
          // e.g. if aJSValue is TClass then ;
          // or  if aJSValue is ImageClass then ;
          SetBaseType(btBoolean);
          exit;
          end;
        end;
    end;

  inherited ComputeBinaryExprRes(Bin, ResolvedEl, Flags, LeftResolved,
    RightResolved);
end;

function TPas2JSResolver.BI_Exit_OnGetCallCompatibility(
  Proc: TResElDataBuiltInProc; Expr: TPasExpr; RaiseOnError: boolean): integer;
var
  Params: TParamsExpr;
  CtxProc: TPasProcedure;
  ParamResolved: TPasResolverResult;
  Param: TPasExpr;
begin
  if (Expr is TParamsExpr) and (length(TParamsExpr(Expr).Params)=1) then
    begin
    Params:=TParamsExpr(Expr);

    CtxProc:=GetParentProc(Expr,true);
    if (CtxProc<>nil) and CtxProc.IsAsync then
      begin
      // inside async proc
      Param:=Params.Params[0];
      ComputeElement(Param,ParamResolved,[]);

      if (rrfReadable in ParamResolved.Flags)
          and (ParamResolved.BaseType=btContext)
          and (ParamResolved.LoTypeEl is TPasClassType)
          and IsExternalClass_Name(TPasClassType(ParamResolved.LoTypeEl),'Promise') then
        begin
        // "exit(aPromise)"  inside async proc
        exit(cCompatible);
        end;
      end;
    end;

  Result:=inherited BI_Exit_OnGetCallCompatibility(Proc, Expr, RaiseOnError);
end;

function TPas2JSResolver.BI_Val_OnGetCallCompatibility(
  Proc: TResElDataBuiltInProc; Expr: TPasExpr; RaiseOnError: boolean): integer;
var
  Params: TParamsExpr;
  Param: TPasExpr;
  ParamResolved: TPasResolverResult;
  bt: TResolverBaseType;
  C: TClass;
begin
  Result:=inherited;
  Params:=TParamsExpr(Expr);
  Param:=Params.Params[1];
  ComputeElement(Param,ParamResolved,[]);
  Result:=cIncompatible;
  bt:=ParamResolved.BaseType;
  if bt=btRange then
    bt:=ParamResolved.SubType;
  if bt=btContext then
    begin
    C:=ParamResolved.LoTypeEl.ClassType;
    if (C=TPasEnumType) or (C=TPasRangeType) then
      Result:=cExact
    end;
  if Result=cIncompatible then
    exit(CheckRaiseTypeArgNo(20181214142349,2,Param,ParamResolved,
         'enum variable',RaiseOnError));
end;

procedure TPas2JSResolver.BI_TypeInfo_OnGetCallResult(
  Proc: TResElDataBuiltInProc; Params: TParamsExpr; out
  ResolvedEl: TPasResolverResult);
// if an external type with the right name and external name is in scope return
// that, otherwise btPointer
var
  Param: TPasExpr;
  ParamResolved: TPasResolverResult;
  C: TClass;
  TIName: String;
  FindData: TPRFindData;
  Abort: boolean;
  bt: TResolverBaseType;
  jbt: TPas2jsBaseType;
  TypeEl: TPasType;
  FoundClass: TPasClassType;
  ScopeDepth: Integer;
  TemplType: TPasGenericTemplateType;
  ConEl: TPasElement;
  ConToken: TToken;
  ResultEl: TPasResultElement;
begin
  Param:=Params.Params[0];
  ComputeElement(Param,ParamResolved,[rcNoImplicitProc]);
  if ParamResolved.LoTypeEl=nil then
    RaiseInternalError(20170413090726);
  if (ParamResolved.BaseType=btProc) and (ParamResolved.IdentEl is TPasFunction) then
    begin
    // typeinfo of function result -> resolve once
    ResultEl:=TPasFunction(ParamResolved.IdentEl).FuncType.ResultEl;
    ComputeResultElement(ResultEl,ParamResolved,[]);
    Include(ParamResolved.Flags,rrfReadable);
    if ParamResolved.LoTypeEl=nil then
      RaiseInternalError(20170421124923);
    end;

  TypeEl:=ParamResolved.LoTypeEl;
  C:=TypeEl.ClassType;
  TIName:='';
  //writeln('TPas2JSResolver.BI_TypeInfo_OnGetCallResult TypeEl=',GetObjName(TypeEl));

  if C=TPasUnresolvedSymbolRef then
    begin
    if TypeEl.CustomData is TResElDataPas2JSBaseType then
      begin
      jbt:=TResElDataPas2JSBaseType(TypeEl.CustomData).JSBaseType;
      if jbt=pbtJSValue then
        TIName:=Pas2JSBuiltInNames[pbitnTI];
      end
    else if TypeEl.CustomData is TResElDataBaseType then
      begin
      bt:=TResElDataBaseType(TypeEl.CustomData).BaseType;
      if bt in (btAllJSInteger+[btCurrency]) then
        TIName:=Pas2JSBuiltInNames[pbitnTIInteger]
      else if bt in [btString,btChar,btDouble,btBoolean] then
        TIName:=Pas2JSBuiltInNames[pbitnTI]
      else if bt=btPointer then
        TIName:=Pas2JSBuiltInNames[pbitnTIPointer];
      end;
    end
  else if ParamResolved.BaseType=btContext then
    begin
    if C=TPasEnumType then
      TIName:=Pas2JSBuiltInNames[pbitnTIEnum]
    else if C=TPasSetType then
      TIName:=Pas2JSBuiltInNames[pbitnTISet]
    else if C.InheritsFrom(TPasProcedureType) then
      begin
      if TPasProcedureType(TypeEl).IsReferenceTo then
        TIName:=Pas2JSBuiltInNames[pbitnTIRefToProcVar]
      else if TPasProcedureType(TypeEl).IsOfObject then
        TIName:=Pas2JSBuiltInNames[pbitnTIMethodVar]
      else
        TIName:=Pas2JSBuiltInNames[pbitnTIProcVar];
      end
    else if C=TPasRecordType then
      TIName:=Pas2JSBuiltInNames[pbitnTIRecord]
    else if C=TPasClassType then
      case TPasClassType(TypeEl).ObjKind of
      okClass:
        if TPasClassType(TypeEl).IsExternal then
          TIName:=Pas2JSBuiltInNames[pbitnTIExtClass]
        else
          TIName:=Pas2JSBuiltInNames[pbitnTIClass];
      okInterface: TIName:=Pas2JSBuiltInNames[pbitnTIInterface];
      okClassHelper,okRecordHelper,okTypeHelper: TIName:=Pas2JSBuiltInNames[pbitnTIHelper];
      else
        RaiseNotYetImplemented(20180328195807,Param);
      end
    else if C=TPasClassOfType then
      begin
      if rrfReadable in ParamResolved.Flags then
        TIName:=Pas2JSBuiltInNames[pbitnTIClass]
      else
        TIName:=Pas2JSBuiltInNames[pbitnTIClassRef];
      end
    else if C=TPasArrayType then
      begin
      if length(TPasArrayType(TypeEl).Ranges)>0 then
        TIName:=Pas2JSBuiltInNames[pbitnTIStaticArray]
      else
        TIName:=Pas2JSBuiltInNames[pbitnTIDynArray];
      end
    else if C=TPasPointerType then
      TIName:=Pas2JSBuiltInNames[pbitnTIPointer]
    else if C=TPasGenericTemplateType then
      begin
      TemplType:=TPasGenericTemplateType(TypeEl);
      if length(TemplType.Constraints)>0 then
        begin
        ConEl:=TemplType.Constraints[0];
        ConToken:=GetGenericConstraintKeyword(ConEl);
        case ConToken of
        tkrecord: TIName:=Pas2JSBuiltInNames[pbitnTIRecord];
        tkclass,tkConstructor: TIName:=Pas2JSBuiltInNames[pbitnTIClass];
        else
          if not (ConEl is TPasType) then
            RaiseNotYetImplemented(20191018180031,ConEl,GetObjPath(Param));
          TypeEl:=ResolveAliasType(TPasType(ConEl));
          if TypeEl is TPasClassType then
            case TPasClassType(TypeEl).ObjKind of
            okClass:
              if TPasClassType(TypeEl).IsExternal then
                TIName:=Pas2JSBuiltInNames[pbitnTIExtClass]
              else
                TIName:=Pas2JSBuiltInNames[pbitnTIClass];
            okInterface:
              TIName:=Pas2JSBuiltInNames[pbitnTIInterface];
            else
              RaiseNotYetImplemented(20200927100825,ConEl,GetObjPath(Param));
            end
          else
            RaiseNotYetImplemented(20191018180131,ConEl,GetObjPath(Param));
        end;
        end;
      if TIName='' then
        begin
        // generic template without constraints
        TIName:=Pas2JSBuiltInNames[pbitnTI];
        end;
      end;
    end
  else if ParamResolved.BaseType=btSet then
    begin
    if ParamResolved.IdentEl is TPasSetType then
      TIName:=Pas2JSBuiltInNames[pbitnTISet];
    end
  else if ParamResolved.BaseType=btRange then
    begin
    ConvertRangeToElement(ParamResolved);
    if ParamResolved.BaseType in btAllJSInteger then
      TIName:=Pas2JSBuiltInNames[pbitnTIInteger]
    else if ParamResolved.BaseType in [btChar,btBoolean] then
      TIName:=Pas2JSBuiltInNames[pbitnTI]
    else if ParamResolved.BaseType=btContext then
      begin
      TypeEl:=ParamResolved.LoTypeEl;
      C:=TypeEl.ClassType;
      if C=TPasEnumType then
        TIName:=Pas2JSBuiltInNames[pbitnTIEnum];
      end;
    end
  else if C=TPasRangeType then
    begin
    if ParamResolved.BaseType in btAllJSInteger then
      TIName:=Pas2JSBuiltInNames[pbitnTIInteger]
    else if ParamResolved.BaseType in [btChar,btBoolean] then
      TIName:=Pas2JSBuiltInNames[pbitnTI]
    end;
  //writeln('TPas2JSResolver.BI_TypeInfo_OnGetCallResult TIName=',TIName,' ',GetObjName(TypeEl));
  if TIName='' then
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPas2JSResolver.BI_TypeInfo_OnGetCallResult ',GetResolverResultDbg(ParamResolved));
    {$ENDIF}
    RaiseNotYetImplemented(20170413091852,Param);
    end;

  // search for TIName
  ScopeDepth:=StashSubExprScopes;
  FindData:=Default(TPRFindData);
  FindData.ErrorPosEl:=Params;
  Abort:=false;
  IterateElements(TIName,@OnFindFirst,@FindData,Abort);
  RestoreStashedScopes(ScopeDepth);
  {$IFDEF VerbosePas2JS}
  writeln('TPas2JSResolver.BI_TypeInfo_OnGetCallResult TIName="',TIName,'" FindData.Found="',GetObjName(FindData.Found),'"');
  {$ENDIF}
  if FindData.Found is TPasType then
    begin
    TypeEl:=ResolveAliasType(TPasType(FindData.Found));
    if TypeEl.ClassType=TPasClassType then
      begin
      FoundClass:=TPasClassType(FindData.Found);
      if FoundClass.IsExternal
          and (FoundClass.ExternalName=Pas2JSBuiltInNames[pbivnRTL]+'.'+TIName) then
        begin
        // use external class definition
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JSResolver.BI_TypeInfo_OnGetCallResult FindData.Found="',FindData.Found.ParentPath,'"');
        {$ENDIF}
        SetResolverTypeExpr(ResolvedEl,btContext,FoundClass,TPasType(FindData.Found),[rrfReadable]);
        exit;
        end;
      end;
    end;

  // default: btPointer
  SetResolverTypeExpr(ResolvedEl,btPointer,BaseTypes[btPointer],BaseTypes[btPointer],[rrfReadable]);

  if Proc=nil then ;
end;

function TPas2JSResolver.BI_Debugger_OnGetCallCompatibility(
  Proc: TResElDataBuiltInProc; Expr: TPasExpr; RaiseOnError: boolean): integer;
// debugger;
begin
  if Expr is TParamsExpr then
    Result:=CheckBuiltInMaxParamCount(Proc,TParamsExpr(Expr),0,RaiseOnError)
  else
    Result:=cExact;
end;

function TPas2JSResolver.BI_AWait_OnGetCallCompatibility(
  Proc: TResElDataBuiltInProc; Expr: TPasExpr; RaiseOnError: boolean): integer;
// await(T; p: TJSPromise): T
// await(T; jsvalue): T
// await(AsyncFuncWithResultT): T
// await(AsyncProc);
var
  Params: TParamsExpr;
  Param: TPasExpr;
  ParamResolved, Param2Resolved: TPasResolverResult;
  ParentProc: TPasProcedure;
  TypeEl: TPasType;
begin
  Result:=cIncompatible;

  // check if inside async proc
  ParentProc:=GetParentProc(Expr,true);
  if (ParentProc=nil) or not ParentProc.IsAsync then
    begin
    if RaiseOnError then
      RaiseMsg(20200519153349,nAWaitOnlyInAsyncProcedure,sAWaitOnlyInAsyncProcedure,[],Expr);
    exit;
    end;

  if not CheckBuiltInMinParamCount(Proc,Expr,1,RaiseOnError) then
    exit;
  Params:=TParamsExpr(Expr);
  Param:=Params.Params[0];
  ComputeElement(Param,ParamResolved,[]);
  if (rrfReadable in ParamResolved.Flags) then
    begin
    // function await(value)
    // must be the only parameter
    Result:=CheckBuiltInMaxParamCount(Proc,Params,1,RaiseOnError);
    if Result=cIncompatible then exit;

    TypeEl:=ParamResolved.LoTypeEl;
    if (ParamResolved.IdentEl is TPasResultElement) then
      begin
      // await(AsyncFuncCall)
      if not TPasFunctionType(ParamResolved.IdentEl.Parent).IsAsync then
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JSResolver.BI_AWait_OnGetCallCompatibility ',GetResolverResultDbg(ParamResolved));
        {$ENDIF}
        if RaiseOnError then
          RaiseMsg(20201229232446,nXExpectedButYFound,sXExpectedButYFound,['async function',GetResolverResultDescription(ParamResolved)],Expr)
        else
          exit(cIncompatible);
        end;
      end
    else if (ParamResolved.BaseType=btContext)
        and (TypeEl is TPasProcedureType) then
      begin
      // await(AsyncFuncTypeVar)
      if not TPasProcedureType(TypeEl).IsAsync then
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JSResolver.BI_AWait_OnGetCallCompatibility ',GetResolverResultDbg(ParamResolved));
        {$ENDIF}
        if RaiseOnError then
          RaiseMsg(20201229232541,nXExpectedButYFound,sXExpectedButYFound,['async function',GetResolverResultDescription(ParamResolved)],Expr)
        else
          exit(cIncompatible);
        end;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPas2JSResolver.BI_AWait_OnGetCallCompatibility ',GetResolverResultDbg(ParamResolved));
      {$ENDIF}
      if RaiseOnError then
        RaiseMsg(20201229224920,nXExpectedButYFound,sXExpectedButYFound,['async function',GetResolverResultDescription(ParamResolved)],Expr)
      else
        exit(cIncompatible);
      end;

    end
  else if ParamResolved.BaseType=btProc then
    begin
    // e.g.  await(Proc)
    if Expr.Parent is TPasExpr then
      begin
      if RaiseOnError then
        RaiseMsg(20200523232827,nXExpectedButYFound,sXExpectedButYFound,['async function',GetResolverResultDescription(ParamResolved)],Expr);
      exit;
      end;
    Result:=CheckBuiltInMaxParamCount(Proc,Params,1,RaiseOnError);
    end
  else
    begin
    TypeEl:=ParamResolved.LoTypeEl;
    if (TypeEl is TPasUnresolvedSymbolRef)
        and (TypeEl.CustomData is TResElDataBaseType) then
      // base type
    else if (TypeEl<>nil) and (ParamResolved.IdentEl is TPasType) then
      begin
      // custom type
      if (ParamResolved.BaseType=btContext)
          and (ParamResolved.LoTypeEl is TPasClassType)
          and IsExternalClass_Name(TPasClassType(ParamResolved.LoTypeEl),'Promise') then
        begin
        // awit(TJSPromise,x) ->  await resolves all promises
        exit(CheckRaiseTypeArgNo(20201120001741,1,Param,ParamResolved,'non Promise type',RaiseOnError));
        end;
      end
    else
      exit(CheckRaiseTypeArgNo(20200519151816,1,Param,ParamResolved,'jsvalue',RaiseOnError));

    // function await(type,...)
    if length(Params.Params)<2 then
      begin
      if RaiseOnError then
        RaiseMsg(20200520090749,nWrongNumberOfParametersForCallTo,
          sWrongNumberOfParametersForCallTo,[AwaitSignature2],Params);
      exit(cIncompatible);
      end;

    // check second param TJSPromise
    Param:=Params.Params[1];
    if CheckCallAsyncFuncResult(Param,Param2Resolved) then
      begin
      // await(T,CallAsyncFuncResultS)
      if (Param2Resolved.BaseType=btContext)
          and (Param2Resolved.LoTypeEl is TPasClassType)
          and IsExternalClass_Name(TPasClassType(Param2Resolved.LoTypeEl),'Promise') then
        begin
        // await(T,CallAsyncFuncReturningPromise) -> good
        end
      else
        begin
        // await(T,CallAsyncFuncResultS)
        // Note: Actually this case is not needed, as you can simply write await(AsyncCall)
        //       but it helps some parsers and some people find it more readable
        // make sure you cannot shoot yourself in the foot: -> check T=S OR S is T
        ParamResolved.Flags:=[rrfReadable,rrfWritable];
        ParamResolved.IdentEl:=nil;
        Result:=CheckParamResCompatibility(Param,Param2Resolved,ParamResolved,1,RaiseOnError,false);
        exit;
        end;
      end
    else
      begin
      ComputeElement(Param,Param2Resolved,[]);
      if not (rrfReadable in Param2Resolved.Flags) then
        exit(CheckRaiseTypeArgNo(20200520091707,2,Param,Param2Resolved,
           'instance of TJSPromise',RaiseOnError));

      if (Param2Resolved.BaseType=btContext)
          and (Param2Resolved.LoTypeEl is TPasClassType)
          and IsExternalClass_Name(TPasClassType(Param2Resolved.LoTypeEl),'Promise') then
        // await(T,aPromise)
      else if IsJSBaseType(Param2Resolved,pbtJSValue) then
        // await(T,jsvalue)
      else if (Param2Resolved.IdentEl is TPasArgument)
          and (Param2Resolved.LoTypeEl=nil) then
        // await(T,UntypedArg)
      else
        exit(CheckRaiseTypeArgNo(20200520091708,2,Param,Param2Resolved,
           'TJSPromise',RaiseOnError));
      end;

    Result:=CheckBuiltInMaxParamCount(Proc,Params,2,RaiseOnError,AwaitSignature2);
    end;
end;

procedure TPas2JSResolver.BI_AWait_OnGetCallResult(Proc: TResElDataBuiltInProc;
  Params: TParamsExpr; out ResolvedEl: TPasResolverResult);
// function await(const Expr: T): T
// function await(T; p: TJSPromise): T
// await(Proc());
var
  Param: TPasExpr;
begin
  Param:=Params.Params[0];
  if length(Params.Params)=1 then
    begin
    // await(AsyncFuncCall)
    if CheckCallAsyncFuncResult(Param,ResolvedEl) then
      begin
      // await(CallAsynFuncResultT): T
      if (ResolvedEl.BaseType=btContext)
          and (ResolvedEl.LoTypeEl is TPasClassType)
          and IsExternalClass_Name(TPasClassType(ResolvedEl.LoTypeEl),'Promise') then
        // async function returns a promise, await resolve all promises -> need final type as first param
        RaiseMsg(20201229235932,nWrongNumberOfParametersForCallTo,
          sWrongNumberOfParametersForCallTo,[AwaitSignature2],Param);
      exit;
      end;
    end
  else
    begin
    // await(T;promise):T
    end;
  ComputeElement(Param,ResolvedEl,[]);
  Include(ResolvedEl.Flags,rrfReadable);
  if Proc=nil then ;
end;

procedure TPas2JSResolver.BI_AWait_OnEval(Proc: TResElDataBuiltInProc;
  Params: TParamsExpr; Flags: TResEvalFlags; out Evaluated: TResEvalValue);
var
  Param: TPasExpr;
  ParamResolved: TPasResolverResult;
begin
  Evaluated:=nil;
  if length(Params.Params)<>1 then
    exit;

  Param:=Params.Params[0];
  ComputeElement(Param,ParamResolved,[]);
  Evaluated:=Eval(Param,Flags);
  if Proc=nil then ;
end;

procedure TPas2JSResolver.BI_AWait_OnFinishParamsExpr(
  Proc: TResElDataBuiltInProc; Params: TParamsExpr);
var
  P: TPasExprArray;
  Param, PathEnd: TPasExpr;
  Ref: TResolvedReference;
  Decl, IdentEl, SubEl: TPasElement;
  ResolvedEl, ParamResolved: TPasResolverResult;
  Implicit, IsPromise: Boolean;
  TypeEl: TPasType;
begin
  if Proc=nil then ;
  P:=Params.Params;
  if P=nil then ;
  Param:=P[0];
  FinishCallArgAccess(Param,rraRead);
  if length(P)=1 then
    begin
    // await(expr)
    PathEnd:=GetPathEndIdent(Param,false);
    if (PathEnd<>nil) and (PathEnd.CustomData is TResolvedReference) then
      begin
      // await(a.b)
      Ref:=TResolvedReference(PathEnd.CustomData);
      Decl:=Ref.Declaration;
      Implicit:=false;
      if (Decl is TPasVariable) or (Decl.ClassType=TPasArgument) then
        begin
        ComputeElement(Decl,ResolvedEl,[rcNoImplicitProcType]);
        if IsProcedureType(ResolvedEl,true) then
          Implicit:=true;
        end
      else if (Decl is TPasProcedure) then
        Implicit:=true;
      if Implicit then begin
        // implicit call
        Exclude(Ref.Flags,rrfNoImplicitCallWithoutParams);
        Include(Ref.Flags,rrfImplicitCallWithoutParams);
        end;
      end
    else
      begin
      ComputeElement(Param,ParamResolved,[]);
      IsPromise:=false;
      TypeEl:=ParamResolved.LoTypeEl;
      IdentEl:=ParamResolved.IdentEl;
      if TypeEl.ClassType=TPasClassType then
        IsPromise:=IsExternalClass_Name(TPasClassType(TypeEl),'Promise')
      else if (ParamResolved.BaseType=btProc) and (IdentEl=nil)
          and (TypeEl is TPasProcedureType) then
        IsPromise:=TPasProcedureType(TypeEl).IsAsync
      else if IdentEl is TPasProcedure then
        IsPromise:=TPasProcedure(ParamResolved.IdentEl).IsAsync
      else if IdentEl is TPasResultElement then
        begin
        SubEl:=TPasResultElement(IdentEl).Parent;
        if (SubEl is TPasFunctionType) then
          IsPromise:=TPasFunctionType(SubEl).IsAsync;
        end;
      {$IFDEF VerbosePas2JS}
      writeln('TPas2JSResolver.BI_AWait_OnFinishParamsExpr Param=',GetObjPath(Param),' ParamResolved=',GetResolverResultDbg(ParamResolved));
      {$ENDIF}
      if not IsPromise then
        LogMsg(20201116000324,mtHint,nAwaitWithoutPromise,sAwaitWithoutPromise,[],Param);
      end;
    end;

  if length(P)>1 then
    FinishCallArgAccess(P[1],rraRead);
  if length(P)>2 then
    RaiseNotYetImplemented(20200525142451,Params);
end;

constructor TPas2JSResolver.Create;
var
  bt: TPas2jsBaseType;
begin
  inherited;
  // prefer overloads of GUID with string
  cInterfaceToTGUID:=cTypeConversion+2;
  cInterfaceToString:=cTypeConversion+1;

  {$IFDEF FPC_HAS_CPSTRING}
  ExprEvaluator.DefaultSourceCodePage:=CP_UTF8;
  ExprEvaluator.DefaultStringCodePage:=CP_UTF16;
  {$ENDIF}
  FExternalNames:=TPasResHashList.Create;
  StoreSrcColumns:=true;
  Options:=Options+DefaultPasResolverOptions;
  ScopeClass_Class:=TPas2JSClassScope;
  ScopeClass_InitialFinalization:=TPas2JSInitialFinalizationScope;
  ScopeClass_Module:=TPas2JSModuleScope;
  ScopeClass_Procedure:=TPas2JSProcedureScope;
  ScopeClass_Record:=TPas2JSRecordScope;
  ScopeClass_Array:=TPas2JSArrayScope;
  ScopeClass_ProcType:=TPas2JSProcTypeScope;
  ScopeClass_Section:=TPas2JSSectionScope;
  ScopeClass_WithExpr:=TPas2JSWithExprScope;
  for bt in [pbtJSValue] do
    AddJSBaseType(Pas2jsBaseTypeNames[bt],bt);
  AnonymousElTypePostfix:=Pas2JSBuiltInNames[pbitnAnonymousPostfix];
  BaseTypeChar:=btWideChar;
  BaseTypeString:=btUnicodeString;
  BaseTypeLength:=btIntDouble;
end;

destructor TPas2JSResolver.Destroy;
begin
  ClearElementData;
  {$IFDEF pas2js}
  FExternalNames:=nil;
  {$ELSE}
  FreeAndNil(FExternalNames);
  {$ENDIF}
  ClearOverloadScopes;
  inherited Destroy;
end;

procedure TPas2JSResolver.ClearBuiltInIdentifiers;
var
  bt: TPas2jsBaseType;
  pbp: TPas2jsBuiltInProc;
begin
  inherited ClearBuiltInIdentifiers;
  for bt in TPas2jsBaseType do
    ReleaseAndNil(TPasElement(FJSBaseTypes[bt]){$IFDEF CheckPasTreeRefCount},'TPasResolver.AddCustomBaseType'{$ENDIF});
  for pbp in TPas2jsBuiltInProc do
    FJSBuiltInProcs[pbp]:=nil;
end;

function TPas2JSResolver.IsJSBaseType(TypeEl: TPasType; Typ: TPas2jsBaseType
  ): boolean;
begin
  Result:=(TypeEl is TPasUnresolvedSymbolRef)
    and (CompareText(TypeEl.Name,Pas2jsBaseTypeNames[Typ])=0)
    and (TypeEl.CustomData is TResElDataPas2JSBaseType);
end;

function TPas2JSResolver.IsJSBaseType(const TypeResolved: TPasResolverResult;
  Typ: TPas2jsBaseType; HasValue: boolean): boolean;
begin
  if (TypeResolved.BaseType<>btCustom) or not IsJSBaseType(TypeResolved.LoTypeEl,Typ) then
    exit(false);
  if HasValue and not (rrfReadable in TypeResolved.Flags) then
    exit(false);
  Result:=true;
end;

procedure TPas2JSResolver.AddObjFPCBuiltInIdentifiers(
  const TheBaseTypes: TResolveBaseTypes;
  const TheBaseProcs: TResolverBuiltInProcs);
var
  InvalidTypes: TResolveBaseTypes;
  bt: TResolverBaseType;
  InvalidProcs: TResolverBuiltInProcs;
  bf: TResolverBuiltInProc;
begin
  InvalidTypes:=TheBaseTypes-btAllJSBaseTypes;
  if InvalidTypes<>[] then
    for bt in InvalidTypes do
      RaiseInternalError(20170409180202,BaseTypeNames[bt]);
  InvalidProcs:=TheBaseProcs-bfAllJSBaseProcs;
  if InvalidProcs<>[] then
    for bf in InvalidProcs do
      RaiseInternalError(20170409180246,ResolverBuiltInProcNames[bf]);
  inherited AddObjFPCBuiltInIdentifiers(TheBaseTypes-[btUIntDouble,btIntDouble],TheBaseProcs);
  if btUIntDouble in TheBaseTypes then
    AddBaseType(Pas2JSBuiltInNames[pbitnUIntDouble],btUIntDouble);
  if btIntDouble in TheBaseTypes then
    AddBaseType(Pas2JSBuiltInNames[pbitnIntDouble],btIntDouble);
  FJSBuiltInProcs[pbpDebugger]:=AddBuiltInProc(Pas2jsBuiltInProcNames[pbpDebugger],
      'procedure Debugger',
      @BI_Debugger_OnGetCallCompatibility,nil,
      nil,nil,bfCustom,[bipfCanBeStatement]);
  FJSBuiltInProcs[pbpAWait]:=AddBuiltInProc(Pas2jsBuiltInProcNames[pbpAWait],
      'function await(const Expr: T): T',
      @BI_AWait_OnGetCallCompatibility,@BI_AWait_OnGetCallResult,
      @BI_AWait_OnEval,@BI_AWait_OnFinishParamsExpr,bfCustom,[bipfCanBeStatement]);
end;

function TPas2JSResolver.CheckTypeCastRes(const FromResolved,
  ToResolved: TPasResolverResult; ErrorEl: TPasElement; RaiseOnError: boolean
  ): integer;

  function Incompatible(Id: TMaxPrecInt): integer;
  begin
    if RaiseOnError then
      RaiseIncompatibleTypeRes(Id,nIllegalTypeConversionTo,
        [],FromResolved,ToResolved,ErrorEl);
    Result:=cIncompatible;
  end;

var
  JSBaseType: TPas2jsBaseType;
  C: TClass;
  ToClass: TPasClassType;
  ToTypeEl, FromTypeEl: TPasType;
begin
  Result:=cIncompatible;
  {$IFDEF VerbosePas2JS}
  writeln('TPas2JSResolver.CheckTypeCastRes To=',GetResolverResultDbg(ToResolved),' From=',GetResolverResultDbg(FromResolved));
  {$ENDIF}
  if rrfReadable in FromResolved.Flags then
    begin
    if (ToResolved.BaseType=btCustom) then
      begin
      ToTypeEl:=ToResolved.LoTypeEl;
      if not (ToTypeEl is TPasUnresolvedSymbolRef) then
        RaiseInternalError(20170325142826);
      if (ToTypeEl.CustomData is TResElDataPas2JSBaseType) then
        begin
        // type cast to pas2js type, e.g. JSValue(V)
        JSBaseType:=TResElDataPas2JSBaseType(ToTypeEl.CustomData).JSBaseType;
        if JSBaseType=pbtJSValue then
          begin
          if (FromResolved.BaseType in btAllJSValueSrcTypes) then
            Result:=cCompatible // type cast to JSValue
          else if FromResolved.BaseType=btCustom then
            begin
            if IsJSBaseType(FromResolved,pbtJSValue) then
              Result:=cExact;
            end
          else if FromResolved.BaseType=btContext then
            Result:=cCompatible;
          end;
        exit;
        end;
      end
    else if FromResolved.BaseType=btCustom then
      begin
      FromTypeEl:=FromResolved.LoTypeEl;
      if not (FromTypeEl is TPasUnresolvedSymbolRef) then
        RaiseInternalError(20170325143016);
      if (FromTypeEl.CustomData is TResElDataPas2JSBaseType) then
        begin
        // type cast a pas2js value, e.g. T(jsvalue)
        JSBaseType:=TResElDataPas2JSBaseType(FromTypeEl.CustomData).JSBaseType;
        if JSBaseType=pbtJSValue then
          begin
          if (ToResolved.BaseType in btAllJSValueTypeCastTo) then
            Result:=cCompatible // type cast JSValue to simple base type
          else if ToResolved.BaseType=btContext then
            begin
            // typecast JSValue to user type
            Result:=cCompatible;
            end;
          end;
        exit;
        end;
      end
    else if ToResolved.BaseType=btContext then
      begin
      ToTypeEl:=ToResolved.LoTypeEl;
      C:=ToTypeEl.ClassType;
      if C=TPasClassType then
        begin
        ToClass:=TPasClassType(ToTypeEl);
        if ToClass.IsExternal then
          begin
          if (FromResolved.BaseType in btAllJSStringAndChars) then
            begin
            if IsExternalClass_Name(ToClass,'String') then
              // TJSString(aString)
              exit(cExact);
            end
          else if (FromResolved.BaseType=btContext) then
            begin
            FromTypeEl:=FromResolved.LoTypeEl;
            if FromTypeEl.ClassType=TPasArrayType then
              begin
              if IsExternalClass_Name(ToClass,'Array')
                  or IsExternalClass_Name(ToClass,'Object') then
                // TJSArray(AnArray)  or  TJSObject(AnArray)
                exit(cExact);
              end
            else if FromTypeEl.ClassType=TPasRecordType then
              begin
              if IsExternalClass_Name(ToClass,'Object') then
                // TJSObject(aRecord)
                exit(cExact);
              end
            else if FromTypeEl.ClassType=TPasClassOfType then
              begin
              if IsExternalClass_Name(ToClass,'Object') then
                // TJSObject(ImgClass)
                exit(cExact);
              end
            else if FromTypeEl.InheritsFrom(TPasProcedureType) then
              begin
              if IsExternalClass_Name(ToClass,'Function')
                  or IsExternalClass_Name(ToClass,'Object') then
                // TJSFunction(@Proc) or TJSFunction(ProcVar)
                exit(cExact);
              end
            else if FromTypeEl.ClassType=TPasClassType then
              begin
              if TPasClassType(FromTypeEl).IsExternal
                  and (msDelphi in CurrentParser.CurrentModeswitches)
                  and not (bsObjectChecks in CurrentParser.Scanner.CurrentBoolSwitches) then
                // ExtClass(ExtClass)  -> allow in mode delphi and no objectchecks
                exit(cAliasExact); // $mode delphi
              end;
            end;
          end;
        end
      else if C=TPasArrayType then
        begin
        if (FromResolved.BaseType=btContext) then
          begin
          FromTypeEl:=FromResolved.LoTypeEl;
          if (FromTypeEl.ClassType=TPasClassType)
              and TPasClassType(FromTypeEl).IsExternal
              and (IsExternalClass_Name(TPasClassType(FromTypeEl),'Array')
                or IsExternalClass_Name(TPasClassType(FromTypeEl),'Object')) then
            begin
            // type cast external Array/Object to an array
            exit(cCompatible);
            end;
          end;
        end
      else if C=TPasRecordType then
        begin
        // typecast to recordtype
        if FromResolved.BaseType=btUntyped then
          // recordtype(untyped) -> ok
        else if FromResolved.BaseType=btContext then
          begin
          FromTypeEl:=FromResolved.LoTypeEl;
          if FromTypeEl=ToTypeEl then
            exit(cAliasExact)
          else
            // FPC/Delphi allow typecasting records of same size, pas2js does not
            exit(Incompatible(20180503134526));
          end
        else
          exit(Incompatible(20180503134528));
        end
      else if C.InheritsFrom(TPasProcedureType) then
        begin
        // typecast to proctype
        if FromResolved.BaseType=btContext then
          begin
          FromTypeEl:=FromResolved.LoTypeEl;
          if FromTypeEl.ClassType=TPasClassType then
            begin
            if IsExternalClass_Name(TPasClassType(FromTypeEl),'Function') then
              // TProcType(aJSFunction)
              exit(cCompatible);
            end;
          end;
        end;
      end;
    end
  else if FromResolved.IdentEl is TPasType then
    begin
    // FromResolved is a type
    FromTypeEl:=ResolveAliasType(TPasType(FromResolved.IdentEl));
    if ToResolved.BaseType=btContext then
      begin
      ToTypeEl:=ToResolved.LoTypeEl;
      if (ToTypeEl.ClassType=TPasClassType)
          and TPasClassType(ToTypeEl).IsExternal
          and (TPasClassType(ToTypeEl).ExternalName='Object') // do not allow typecast to a descendant!
          then
        begin
        // type cast to JS Object, not a descendant
        if (FromTypeEl.ClassType=TPasClassType)
            or (FromTypeEl.ClassType=TPasRecordType) then
          // e.g. TJSObject(TObject)
          exit(cTypeConversion+1);
        end;
      end;
    end;
  Result:=inherited CheckTypeCastRes(FromResolved,ToResolved,ErrorEl,RaiseOnError);
end;

function TPas2JSResolver.FindLocalBuiltInSymbol(El: TPasElement): TPasElement;
var
  Data: TObject;
  pbp: TPas2jsBuiltInProc;
begin
  Result:=inherited FindLocalBuiltInSymbol(El);
  if Result<>nil then exit;
  Data:=El.CustomData;
  if Data is TResElDataPas2JSBaseType then
    Result:=JSBaseTypes[TResElDataPas2JSBaseType(Data).JSBaseType]
  else if (Data.ClassType=TResElDataBuiltInProc)
      and (TResElDataBuiltInProc(Data).BuiltIn=bfCustom) then
    for pbp in TPas2jsBuiltInProc do
      if El.Name=Pas2jsBuiltInProcNames[pbp] then
        Result:=FJSBuiltInProcs[pbp].Element;
end;

function TPas2JSResolver.ExtractPasStringLiteral(El: TPasElement;
  const S: String): TJSString;
{ Extracts the value from a Pascal string literal

  S is a Pascal string literal e.g. 'Line'#10
    ''  empty string
    '''' => "'"
    #decimal
    #$hex
    ^l  l is a letter a-z
    Note that invalid UTF-8 sequences are checked by the scanner
}
var
  p, StartP, i, l: integer;
  c: Char;
begin
  Result:='';
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ExtractPasStringLiteral S="',S,'" ',{$IFDEF pas2js}copy(s,100){$ELSE}RawStrToCaption(S,100){$ENDIF},' ',length(S));
  {$ENDIF}
  if S='' then
    RaiseInternalError(20170207154543);
  p:=1;
  l:=length(S);
  while p<=l do
    case S[p] of
    '''':
      begin
      inc(p);
      StartP:=p;
      repeat
        if p>l then
          RaiseInternalError(20170207155120);
        c:=S[p];
        case c of
        '''':
          begin
          if p>StartP then
            Result:=Result+StrToJSString(copy(S,StartP,p-StartP)); // todo error on invalid UTF-8 sequence
          inc(p);
          StartP:=p;
          if (p>l) or (S[p]<>'''') then
            break;
          Result:=Result+'''';
          inc(p);
          StartP:=p;
          end;
        else
          inc(p);
        end;
      until false;
      if p>StartP then
        Result:=Result+StrToJSString(copy(S,StartP,p-StartP)); // todo error on invalid UTF-8 sequence
      end;
    '#':
      begin
      // word sequence
      inc(p);
      if p>l then
        RaiseInternalError(20170207155121);
      if S[p]='$' then
        begin
        // #$hexnumber
        inc(p);
        StartP:=p;
        i:=0;
        while p<=l do
          begin
          c:=S[p];
          case c of
          '0'..'9': i:=i*16+ord(c)-ord('0');
          'a'..'f': i:=i*16+ord(c)-ord('a')+10;
          'A'..'F': i:=i*16+ord(c)-ord('A')+10;
          else break;
          end;
          if i>$10ffff then
            RaiseNotYetImplemented(20170207164657,El,'maximum codepoint is $10ffff');
          inc(p);
          end;
        if p=StartP then
          RaiseInternalError(20170207164956);
        end
      else
        begin
        // #decimalnumber
        StartP:=p;
        i:=0;
        while p<=l do
          begin
          c:=S[p];
          case c of
          '0'..'9': i:=i*10+ord(c)-ord('0');
          else break;
          end;
          if i>$10ffff then
            RaiseNotYetImplemented(20170207171140,El,'maximum codepoint is $10ffff');
          inc(p);
          end;
        if p=StartP then
          RaiseInternalError(20170207171148);
        end;
      Result:=Result+CodePointToJSString(i);
      end;
    '^':
      begin
      // ^A is #1
      inc(p);
      if p>l then
        RaiseInternalError(20181025125920);
      c:=S[p];
      case c of
      'a'..'z': Result:=Result+TJSChar(ord(c)-ord('a')+1);
      'A'..'Z': Result:=Result+TJSChar(ord(c)-ord('A')+1);
      else RaiseInternalError(20170207160412);
      end;
      inc(p);
      end;
    else
      RaiseNotYetImplemented(20170207154653,El,'ord='+IntToStr(ord(S[p])));
    end;
  {$IFDEF VerbosePas2JS}
  {AllowWriteln}
  writeln('TPasToJSConverter.ExtractPasStringLiteral Result="',Result,'"');
  //for i:=1 to length(Result) do
  //  writeln('  Result[',i,']',HexStr(ord(Result[i]),4));
  {AllowWriteln-}
  {$ENDIF}
end;

function TPas2JSResolver.ResolverToJSValue(Value: TResEvalValue;
  ErrorEl: TPasElement): TJSValue;
begin
  Result:=nil;
  if Value=nil then exit;
  case Value.Kind of
  revkBool: Result:=TJSValue.Create(TResEvalBool(Value).B);
  revkInt: Result:=TJSValue.Create(TJSNumber(TResEvalInt(Value).Int));
  revkUInt: Result:=TJSValue.Create(TJSNumber(TResEvalUInt(Value).UInt));
  revkFloat: Result:=TJSValue.Create(TJSNumber(TResEvalFloat(Value).FloatValue));
  {$IFDEF FPC_HAS_CPSTRING}
  revkString: Result:=TJSValue.Create(TJSString(
    ExprEvaluator.GetUnicodeStr(TResEvalString(Value).S,ErrorEl)));
  {$ENDIF}
  revkUnicodeString: Result:=TJSValue.Create(TJSString(TResEvalUTF16(Value).S));
  else
    {$IFDEF VerbosePas2JS}
    writeln('TPas2JSResolver.ResolverToJSValue ',Value.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170914092413,ErrorEl,'');
  end;
end;

function TPas2JSResolver.ComputeConstString(Expr: TPasExpr; StoreCustomData,
  NotEmpty: boolean): String;
var
  Value: TResEvalValue;
begin
  Result:='';
  if Expr=nil then
    RaiseInternalError(20170215123600);
  Value:=Eval(Expr,[refAutoConst],StoreCustomData);
  if Value<>nil then
    try
      case Value.Kind of
      {$IFDEF FPC_HAS_CPSTRING}
      revkString: Result:=ExprEvaluator.GetUTF8Str(TResEvalString(Value).S,Expr);
      revkUnicodeString: Result:=UTF8Encode(TResEvalUTF16(Value).S);
      {$ELSE}
      revkUnicodeString: Result:=TResEvalUTF16(Value).S;
      {$ENDIF}
      else
        str(Value.Kind,Result);
        RaiseXExpectedButYFound(20170211221121,'string literal',Result,Expr);
      end;
    finally
      ReleaseEvalValue(Value);
    end;

  if NotEmpty and (Result='') then
    RaiseXExpectedButYFound(20170321085318,'string literal','empty',Expr);
end;

procedure TPas2JSResolver.CheckAssignExprRangeToCustom(
  const LeftResolved: TPasResolverResult; RValue: TResEvalValue; RHS: TPasExpr);
var
  LeftBaseType: TPas2jsBaseType;
begin
  if (LeftResolved.BaseType<>btCustom) then
    exit;
  if not (LeftResolved.LoTypeEl is TPasUnresolvedSymbolRef) then
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPas2JSResolver.CheckAssignExprRangeToCustom LeftResolved=',GetResolverResultDbg(LeftResolved));
    {$ENDIF}
    RaiseInternalError(20170902165913);
    end;
  if not (LeftResolved.LoTypeEl.CustomData is TResElDataPas2JSBaseType) then
    exit;
  LeftBaseType:=TResElDataPas2JSBaseType(LeftResolved.LoTypeEl.CustomData).JSBaseType;
  if LeftBaseType=pbtJSValue then
    // jsvalue:=someconst   ->  ok
  else
    RaiseNotYetImplemented(20170902170153,RHS);

  if RHS=nil then ;
  if RValue=nil then ;
end;

function TPas2JSResolver.CheckAssignCompatibilityClasses(LType,
  RType: TPasClassType): integer;
// LType and RType are not related
var
  LeftScope, RightScope: TPas2JSClassScope;
  LeftSpecItem, RightSpecItem: TPRSpecializedItem;
  i: Integer;
  LeftParam, RightParam: TPasType;
begin
  Result:=cIncompatible;
  if LType.IsExternal and RType.IsExternal then
    begin
    LeftScope:=TPas2JSClassScope(LType.CustomData);
    RightScope:=TPas2JSClassScope(RType.CustomData);
    LeftSpecItem:=LeftScope.SpecializedFromItem;
    RightSpecItem:=RightScope.SpecializedFromItem;
    if (LeftSpecItem<>nil) and (RightSpecItem<>nil)
        and (LeftSpecItem.GenericEl=RightSpecItem.GenericEl) then
      begin
      Result:=cExact;
      for i:=0 to length(LeftSpecItem.Params)-1 do
        begin
        LeftParam:=LeftSpecItem.Params[i];
        RightParam:=RightSpecItem.Params[i];
        if IsSameType(LeftParam,RightParam,prraAlias)
            or IsJSBaseType(LeftParam,pbtJSValue) then
          // e.g. TExt<jsvalue>:=aExt<word>
        else
          begin
          Result:=cIncompatible;
          break;
          end;
        end;
      end;
    end;
end;

function TPas2JSResolver.HasStaticArrayCloneFunc(Arr: TPasArrayType): boolean;
var
  l: Integer;
  ElType: TPasType;
begin
  l:=length(Arr.Ranges);
  if l=0 then exit(false);
  if l>1 then exit(false ); // ToDo: return true when cloning multi dims is implemented
  ElType:=ResolveAliasType(Arr.ElType);
  if ElType is TPasArrayType then
    Result:=length(TPasArrayType(ElType).Ranges)>0
  else if ElType is TPasRecordType then
    Result:=true
  else if ElType is TPasSetType then
    Result:=true
  else
    Result:=false;
end;

function TPas2JSResolver.IsTGUID(TypeEl: TPasRecordType): boolean;
var
  Members: TFPList;
  El: TPasElement;
  MemberIndex, i: Integer;
begin
  Result:=false;
  if not SameText(TypeEl.Name,'TGUID') then exit;
  Members:=TypeEl.Members;
  i:=1;
  for MemberIndex:=0 to Members.Count-1 do
    begin
    El:=TPasElement(Members[MemberIndex]);
    if (El.ClassType<>TPasVariable) then continue;
    if SameText(El.Name,'D'+IntToStr(i)) then
      begin
      if i=4 then exit(true);
      inc(i);
      end;
    end;
end;

function TPas2JSResolver.GetAssignGUIDString(TypeEl: TPasRecordType;
  Expr: TPasExpr; out GUID: TGuid): boolean;
var
  Value: TResEvalValue;
  GUIDStr: String;
begin
  Result:=false;
  if Expr=nil then exit;
  if not IsTGUID(TypeEl) then exit;
  Value:=Eval(Expr,[refAutoConst]);
  try
    case Value.Kind of
    {$IFDEF FPC_HAS_CPSTRING}
    revkString: GUIDStr:=ExprEvaluator.GetUTF8Str(TResEvalString(Value).S,Expr);
    revkUnicodeString: GUIDStr:=UTF8Encode(TResEvalUTF16(Value).S);
    {$ELSE}
    revkUnicodeString: GUIDStr:=TResEvalUTF16(Value).S;
    {$ENDIF}
    else
      RaiseXExpectedButYFound(20180415092350,'GUID string literal',Value.AsString,Expr);
    end;
    if not TryStringToGUID(GUIDStr,GUID) then
      RaiseXExpectedButYFound(20180415092351,'GUID string literal',Value.AsString,Expr);
    Result:=true;
  finally
    ReleaseEvalValue(Value);
  end;
end;

procedure TPas2JSResolver.CheckDispatchField(Proc: TPasProcedure;
  Switch: TValueSwitch);
var
  ProcScope: TPas2JSProcedureScope;
  ClassScope: TPas2JSClassScope;
  FieldName: String;
  Args, Members: TFPList;
  Arg: TPasArgument;
  ArgType: TPasType;
  i: Integer;
  Member: TPasElement;
  MemberResolved: TPasResolverResult;
begin
  Args:=Proc.ProcType.Args;
  if Args.Count<>1 then
    RaiseNotYetImplemented(20190311213959,Proc);
  Arg:=TPasArgument(Args[0]);
  if Arg.ArgType=nil then
    exit; // untyped arg

  ProcScope:=TPas2JSProcedureScope(Proc.CustomData);
  ClassScope:=TPas2JSClassScope(ProcScope.ClassRecScope);
  FieldName:='';
  while ClassScope<>nil do
    begin
    case Switch of
    vsDispatchField:
      if ClassScope.DispatchField<>'' then
        begin
        FieldName:=ClassScope.DispatchField;
        break;
        end;
    vsDispatchStrField:
      if ClassScope.DispatchStrField<>'' then
        begin
        FieldName:=ClassScope.DispatchStrField;
        break;
        end;
    else
      RaiseNotYetImplemented(20190311213650,Proc,'');
    end;
    ClassScope:=ClassScope.AncestorScope as TPas2JSClassScope;
    end;
  if FieldName='' then exit;

  // there is a Dispatch(str) method with a directive -> check field
  ArgType:=ResolveAliasType(Arg.ArgType);
  if not (ArgType is TPasMembersType) then
    begin
    LogMsg(20190311214257,mtWarning,nDispatchRequiresX,sDispatchRequiresX,['record type'],Arg);
    exit;
    end;
  Members:=TPasMembersType(ArgType).Members;
  for i:=0 to Members.Count-1 do
    begin
    Member:=TPasElement(Members[i]);
    if SameText(Member.Name,FieldName) then
      begin
      if Member.ClassType<>TPasVariable then
        begin
        LogMsg(20190311215218,mtWarning,nDispatchRequiresX,sDispatchRequiresX,['field variable "'+FieldName+'"'],Arg);
        exit;
        end;
      // field found -> check type
      ComputeElement(TPasVariable(Member).VarType,MemberResolved,[rcType],Arg);
      case Switch of
      vsDispatchField:
        if not (MemberResolved.BaseType in btAllJSInteger) then
          begin
          LogMsg(20190311215215,mtWarning,nDispatchRequiresX,sDispatchRequiresX,['integer field "'+FieldName+'"'],Arg);
          exit;
          end;
      vsDispatchStrField:
        if not (MemberResolved.BaseType in btAllJSStrings) then
          begin
          LogMsg(20190312125025,mtWarning,nDispatchRequiresX,sDispatchRequiresX,['string field "'+FieldName+'"'],Arg);
          exit;
          end;
      end;
      // check name case
      if Member.Name<>FieldName then
        begin
        LogMsg(20190311221651,mtWarning,nDispatchRequiresX,sDispatchRequiresX,['field name to match exactly "'+FieldName+'"'],Arg);
        exit;
        end;
      exit;
      end;
    end;
  LogMsg(20190311214710,mtWarning,nDispatchRequiresX,sDispatchRequiresX,['record field "'+FieldName+'"'],Arg);
end;

procedure TPas2JSResolver.AddMessageStr(var MsgToProc: TMessageIdToProc_List;
  const S: string; Proc: TPasProcedure);
var
  i: Integer;
begin
  if MsgToProc=nil then
    MsgToProc:=TMessageIdToProc_List.Create
  else
    begin
    // check duplicate
    for i:=0 to MsgToProc.Count-1 do
      if MsgToProc[i]=S then
        RaiseMsg(20190303233647,nDuplicateMessageIdXAtY,sDuplicateMessageIdXAtY,
          [S,GetElementSourcePosStr(TPasProcedure(MsgToProc.Objects[i]).MessageExpr)],Proc.MessageExpr);
    end;
  MsgToProc.AddObject(S,Proc);
end;

procedure TPas2JSResolver.AddMessageIdToClassScope(Proc: TPasProcedure;
  EmitHints: boolean);
var
  AClass: TPasClassType;
  ClassScope: TPas2JSClassScope;
  Expr: TPasExpr;
  Value: TResEvalValue;
begin
  AClass:=TPasClassType(Proc.Parent);
  ClassScope:=TPas2JSClassScope(AClass.CustomData);
  Expr:=Proc.MessageExpr;
  Value:=Eval(Expr,[refConst]);
  if Value=nil then
    RaiseMsg(20190303225651,nIllegalExpressionAfterX,sIllegalExpressionAfterX,['message modifier'],Expr);
  try
    case Value.Kind of
    {$ifdef FPC_HAS_CPSTRING}
    revkString:
      begin
      AddMessageStr(ClassScope.MsgStrToProc,ExprEvaluator.GetUTF8Str(TResEvalString(Value).S,Expr),Proc);
      if EmitHints then
        CheckDispatchField(Proc,vsDispatchStrField);
      end;
    {$ENDIF}
    revkUnicodeString:
      begin
      AddMessageStr(ClassScope.MsgStrToProc,String(TResEvalUTF16(Value).S),Proc);
      if EmitHints then
        CheckDispatchField(Proc,vsDispatchStrField);
      end;
    revkInt:
      begin
      AddMessageStr(ClassScope.MsgIntToProc,IntToStr(TResEvalInt(Value).Int),Proc);
      if EmitHints then
        CheckDispatchField(Proc,vsDispatchField);
      end
    else
      RaiseXExpectedButYFound(20190303225849,'integer constant',Value.AsString,Expr);
    end;
  finally
    ReleaseEvalValue(Value);
  end;
end;

procedure TPas2JSResolver.ComputeResultElement(El: TPasResultElement; out
  ResolvedEl: TPasResolverResult; Flags: TPasResolverComputeFlags;
  StartEl: TPasElement);
var
  FuncType: TPasFunctionType;
  Proc: TPasProcedure;
  JSPromiseClass: TPasClassType;
begin
  if (rcCall in Flags) and (El.Parent is TPasFunctionType) then
    begin
    FuncType:=TPasFunctionType(El.Parent);
    if FuncType.Parent is TPasProcedure then
      begin
      Proc:=TPasProcedure(FuncType.Parent);
      if Proc.IsAsync then
        begin
        // an async function call returns a TJSPromise
        JSPromiseClass:=FindTJSPromise(StartEl);
        SetResolverIdentifier(ResolvedEl,btContext,El,
                       JSPromiseClass,JSPromiseClass,[rrfReadable,rrfWritable]);
        exit;
        end;
      end;
    end;
  inherited ComputeResultElement(El, ResolvedEl, Flags, StartEl);
end;

function TPas2JSResolver.GetElementData(El: TPasElementBase;
  DataClass: TPas2JsElementDataClass): TPas2JsElementData;
begin
  Result:=nil;
  repeat
    if El.InheritsFrom(DataClass) then
      exit(TPas2JsElementData(El));
    if El.CustomData=nil then exit;
    El:=El.CustomData as TPasElementBase;
  until false;
end;

procedure TPas2JSResolver.AddElementData(Data: TPas2JsElementData);
begin
  Data.Owner:=Self;
  if FFirstElementData<>nil then
    begin
    FLastElementData.Next:=Data;
    FLastElementData:=Data;
    end
  else
    begin
    FFirstElementData:=Data;
    FLastElementData:=Data;
    end;
end;

function TPas2JSResolver.CreateElementData(DataClass: TPas2JsElementDataClass;
  El: TPasElement): TPas2JsElementData;
begin
  Result:=DataClass.Create;
  Result.Element:=El;
  AddElementData(Result);
end;

function TPas2JSResolver.CheckEqualCompatibilityUserType(const LHS,
  RHS: TPasResolverResult; ErrorEl: TPasElement; RaiseOnIncompatible: boolean
  ): integer;
begin
  Result:=inherited CheckEqualCompatibilityUserType(LHS,RHS,ErrorEl,RaiseOnIncompatible);
  if Result=cIncompatible then exit;
  if (LHS.LoTypeEl is TPasArrayType)
      and (length(TPasArrayType(LHS.LoTypeEl).Ranges)>0) then
    RaiseMsg(20200508103543,nXIsNotSupported,sXIsNotSupported,['compare static array'],ErrorEl);
  if (RHS.LoTypeEl is TPasArrayType)
      and (length(TPasArrayType(RHS.LoTypeEl).Ranges)>0) then
    RaiseMsg(20200508103544,nXIsNotSupported,sXIsNotSupported,['compare static array'],ErrorEl);
end;

procedure TPas2JSResolver.RaiseMsg(const Id: TMaxPrecInt; MsgNumber: integer;
  const Fmt: String; Args: array of {$IFDEF pas2js}jsvalue{$ELSE}const{$ENDIF};
  ErrorPosEl: TPasElement);
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPas2JSResolver.RaiseMsg [',Id,']');
  {$ENDIF}
  inherited RaiseMsg(Id, MsgNumber, Fmt, Args, ErrorPosEl);
end;

function TPas2JSResolver.GetOverloadName(El: TPasElement): string;
var
  Data: TObject;
  ProcScope, GenScope: TPas2JSProcedureScope;
  GenEl: TPasElement;
begin
  Data:=El.CustomData;
  if Data is TPasGenericScope then
    begin
    if Data is TPas2JSProcedureScope then
      begin
      ProcScope:=TPas2JSProcedureScope(Data);
      if ProcScope.SpecializedFromItem<>nil then
        begin
        // specialized proc -> generic name + '$G' + index
        GenEl:=ProcScope.SpecializedFromItem.GenericEl;
        GenScope:=TPas2JSProcedureScope(GenEl.CustomData);
        Result:=GenScope.OverloadName;
        if Result='' then
          Result:=GenEl.Name+'$';
        Result:=Result+'G'+IntToStr(ProcScope.SpecializedFromItem.Index+1);
        end
      else
        Result:=ProcScope.OverloadName;
      end
    else if Data is TPas2JSArrayScope then
      Result:=TPas2JSArrayScope(Data).JSName
    else if Data is TPas2JSProcTypeScope then
      Result:=TPas2JSProcTypeScope(Data).JSName
    else if Data is TPas2JSRecordScope then
      Result:=TPas2JSRecordScope(Data).JSName
    else if Data is TPas2JSClassScope then
      Result:=TPas2JSClassScope(Data).JSName
    else
      Result:='';
    if Result<>'' then exit;
    end;
  Result:=El.Name;
end;

function TPas2JSResolver.GetBaseDescription(const R: TPasResolverResult;
  AddPath: boolean): string;
begin
  if (R.BaseType=btCustom) and (R.LoTypeEl.CustomData is TResElDataPas2JSBaseType) then
    Result:=Pas2jsBaseTypeNames[TResElDataPas2JSBaseType(R.LoTypeEl.CustomData).JSBaseType]
  else
    Result:=inherited GetBaseDescription(R, AddPath);
end;

function TPas2JSResolver.HasTypeInfo(El: TPasType): boolean;
begin
  Result:=inherited HasTypeInfo(El);
  if not Result then exit;
  if El.Parent is TProcedureBody then
    Result:=false;
end;

function TPas2JSResolver.ProcHasImplElements(Proc: TPasProcedure): boolean;
var
  Scope: TPas2JSProcedureScope;
begin
  Result:=inherited ProcHasImplElements(Proc);
  if Result then exit;
  // no body elements found -> check precompiled
  Scope:=Proc.CustomData as TPas2JSProcedureScope;
  if Scope.ImplProc<>nil then
    Scope:=Scope.ImplProc.CustomData as TPas2JSProcedureScope;
  if (Scope.ImplJS<>nil) and (Scope.ImplJS.BodyJS<>'') then
    Result:=not Scope.ImplJS.EmptyJS;
end;

function TPas2JSResolver.HasAnonymousFunctions(El: TPasImplElement): boolean;
var
  Data: THasAnoFuncData;
begin
  if El=nil then
    exit(false);
  Data:=default(THasAnoFuncData);
  El.ForEachCall(@OnHasAnonymousEl,@Data);
  Result:=Data.Expr<>nil;
end;

function TPas2JSResolver.GetTopLvlProcScope(El: TPasElement
  ): TPas2JSProcedureScope;
var
  Proc: TPasProcedure;
begin
  Result:=nil;
  while El<>nil do
    begin
    if El is TPasProcedure then
      begin
      Proc:=TPasProcedure(El);
      if Proc.CustomData is TPas2JSProcedureScope then
        Result:=TPas2JSProcedureScope(Proc.CustomData);
      exit;
      end;
    El:=El.Parent;
    end;
end;

function TPas2JSResolver.ProcCanBePrecompiled(DeclProc: TPasProcedure): boolean;
var
  El: TPasElement;
  TemplTypes: TFPList;
  ProcScope: TPas2JSProcedureScope;
  GenScope: TPasGenericScope;
begin
  if GetProcTemplateTypes(DeclProc)<>nil then
    exit(false); // generic DeclProc
  ProcScope:=DeclProc.CustomData as TPas2JSProcedureScope;
  if ProcScope.SpecializedFromItem<>nil then
    exit(false); // specialized generic DeclProc
  El:=DeclProc;
  repeat
    El:=El.Parent;
    if El=nil then
      exit(true); // ok
    if El is TPasProcedure then
      exit(false); // DeclProc is a local DeclProc
    if El is TPasGenericType then
      begin
      TemplTypes:=TPasGenericType(El).GenericTemplateTypes;
      if (TemplTypes<>nil) and (TemplTypes.Count>0) then
        exit(false); // method of a generic class/record type
      GenScope:=El.CustomData as TPasGenericScope;
      if GenScope.SpecializedFromItem<>nil then
        exit(false); // method of a specialized class/record type
      end;
  until false;
end;

function TPas2JSResolver.IsTObjectFreeMethod(El: TPasExpr): boolean;
var
  Ref: TResolvedReference;
  Decl: TPasElement;
begin
  Result:=false;
  if El=nil then exit;
  if El.ClassType<>TPrimitiveExpr then exit;
  if not (El.CustomData is TResolvedReference) then exit;
  Ref:=TResolvedReference(El.CustomData);
  if CompareText(TPrimitiveExpr(El).Value,'free')<>0 then exit;
  Decl:=Ref.Declaration;
  if not (Decl.ClassType=TPasProcedure)
      or (Decl.Parent.ClassType<>TPasClassType)
      or (CompareText(Decl.Parent.Name,'tobject')<>0)
      or (pmExternal in TPasProcedure(Decl).Modifiers)
      or (TPasProcedure(Decl).ProcType.Args.Count>0) then
    exit;
  Result:=true;
end;

function TPas2JSResolver.IsExternalBracketAccessor(El: TPasElement): boolean;
var
  ExtName: String;
begin
  if (not (El is TPasProcedure)) or (TPasProcedure(El).LibrarySymbolName=nil) then
    exit(false);
  ExtName:=ComputeConstString(TPasProcedure(El).LibrarySymbolName,false,false);
  Result:=ExtName=ExtClassBracketAccessor;
end;

function TPas2JSResolver.IsExternalClassConstructor(El: TPasElement): boolean;
var
  P: TPasElement;
begin
  if (El.ClassType=TPasConstructor)
      and (pmExternal in TPasConstructor(El).Modifiers) then
    begin
    P:=El.Parent;
    if (P<>nil) and (P.ClassType=TPasClassType) and TPasClassType(P).IsExternal then
      exit(true);
    end;
  Result:=false;
end;

function TPas2JSResolver.IsForInExtArray(Loop: TPasImplForLoop;
  const VarResolved, InResolved: TPasResolverResult; out ArgResolved,
  LengthResolved, PropResultResolved: TPasResolverResult): boolean;
var
  TypeEl: TPasType;
  aClass: TPasClassType;
  ClassScope: TPas2JSClassScope;
  DefProp: TPasProperty;
  Arg0: TPasArgument;
  Getter: TPasElement;
  ClassDotScope: TPasDotClassScope;
  Ident: TPasIdentifier;
  LengthVar: TPasVariable;
begin
  Result:=false;
  ArgResolved:=Default(TPasResolverResult);
  LengthResolved:=Default(TPasResolverResult);
  PropResultResolved:=Default(TPasResolverResult);

  TypeEl:=InResolved.LoTypeEl;
  if (TypeEl.ClassType<>TPasClassType) or not TPasClassType(TypeEl).IsExternal then
    begin
    {$IFDEF VerboseIsForInExtArray}
    writeln('TPas2JSResolver.IsForInExtArray TypeEl ',GetObjName(TypeEl));
    {$ENDIF}
    exit;
    end;
  // for key in JSClass do ...
  aClass:=TPasClassType(TypeEl);
  ClassScope:=TPas2JSClassScope(aClass.CustomData);
  // check has default property
  DefProp:=ClassScope.DefaultProperty;
  if (DefProp=nil) or (DefProp.Args.Count<>1) then
    begin
    {$IFDEF VerboseIsForInExtArray}
    writeln('TPas2JSResolver.IsForInExtArray DefProp ');
    {$ENDIF}
    exit;
    end;
  // check default property is array property
  Arg0:=TPasArgument(DefProp.Args[0]);
  if not (Arg0.Access in [argDefault,argConst]) then
    begin
    {$IFDEF VerboseIsForInExtArray}
    writeln('TPas2JSResolver.IsForInExtArray Arg0 ');
    {$ENDIF}
    exit;
    end;
  // check default array property has an integer as parameter
  ComputeElement(Arg0,ArgResolved,[]);
  if not (ArgResolved.BaseType in btAllJSInteger) then
    begin
    {$IFDEF VerboseIsForInExtArray}
    writeln('TPas2JSResolver.IsForInExtArray ArgResolved=',GetResolverResultDbg(ArgResolved));
    {$ENDIF}
    exit;
    end;

  // find aClass.Length
  ClassDotScope:=PushClassDotScope(aClass);
  Ident:=ClassDotScope.FindIdentifier('length');
  PopScope;
  // check 'length' is const/variable/property
  if (Ident=nil) or not (Ident.Element is TPasVariable) then
    begin
    {$IFDEF VerboseIsForInExtArray}
    writeln('TPas2JSResolver.IsForInExtArray Length ');
    {$ENDIF}
    exit;
    end;

  LengthVar:=TPasVariable(Ident.Element);
  // check 'length' is same type as Arg0
  ComputeElement(LengthVar,LengthResolved,[]);
  if not IsSameType(LengthResolved.LoTypeEl,ArgResolved.LoTypeEl,prraNone) then
    begin
    {$IFDEF VerboseIsForInExtArray}
    writeln('TPas2JSResolver.IsForInExtArray LengthResolved=',GetResolverResultDbg(LengthResolved),' ArgResolved=',GetResolverResultDbg(ArgResolved));
    {$ENDIF}
    exit;
    end;

  // InResolved has default getter and length -> use array enumerator
  Result:=true;

  // check getter is external bracket accessor
  Getter:=GetPasPropertyGetter(DefProp);
  if not IsExternalBracketAccessor(Getter) then
    RaiseMsg(20180519141636,nForInJSArrDefaultGetterNotExtBracketAccessor,
      sForInJSArrDefaultGetterNotExtBracketAccessor,[],Loop.StartExpr);

  // check var fits the property type
  ComputeElement(DefProp.VarType,PropResultResolved,[]);
  Include(PropResultResolved.Flags,rrfReadable);

  //writeln('IsForInExtArray VarResolved=',GetResolverResultDbg(VarResolved),' PropResultResolved=',GetResolverResultDbg(PropResultResolved));
  CheckAssignResCompatibility(VarResolved,PropResultResolved,Loop.VariableName,true);
end;

function TPas2JSResolver.IsHelperMethod(El: TPasElement): boolean;
begin
  Result:=inherited IsHelperMethod(El);
  if not Result then exit;
  Result:=not TPasProcedure(El).IsExternal;
end;

function TPas2JSResolver.IsHelperForMember(El: TPasElement): boolean;
var
  Parent: TPasElement;
begin
  if El=nil then
    exit(false);
  Parent:=El.Parent;
  if (Parent=nil) or (Parent.ClassType<>TPasClassType)
      or (TPasClassType(Parent).HelperForType=nil) then
    exit(false);
  if El is TPasProcedure then
    Result:=TPasProcedure(El).IsExternal
  else if El is TPasVariable then
    Result:=vmExternal in TPasVariable(El).VarModifiers
  else
    Result:=false;
end;

function TPas2JSResolver.ImplBlockReadsDecl(Block: TPasImplBlock;
  Decl: TPasElement): boolean;
var
  Data: THasElReadingDeclData;
begin
  Data.Decl:=Decl;
  Data.El:=nil;
  Block.ForEachCall(@OnHasElReadingDecl,@Data);
  Result:=Data.El<>nil;
end;

{ TParamContext }

constructor TParamContext.Create(PasEl: TPasElement; JSEl: TJSElement;
  aParent: TConvertContext);
begin
  inherited Create(PasEl, JSEl, aParent);
  Access:=caAssign;
  AccessContext:=Self;
end;

{ TPas2JsElementData }

procedure TPas2JsElementData.SetElement(const AValue: TPasElement);
var
  Data: TPasElementBase;
begin
  if FElement=AValue then Exit;
  if FElement<>nil then
    begin
    Data:=FElement;
    while Data.CustomData<>Self do
      if Data.CustomData is TPasElementBase then
        Data:=TPasElementBase(Data.CustomData)
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPas2JsElementData.SetElement REMOVE ',ClassName);
        writeln('  ',GetObjName(Data.CustomData));
        {$ENDIF}
        raise EPas2JS.Create('');
        end;
    Data.CustomData:=CustomData;
    TPasElement(FElement).Release{$IFDEF CheckPasTreeRefCount}('TPas2JsElementData.SetElement'){$ENDIF};
    end;
  FElement:=AValue;
  if FElement<>nil then
    begin
    TPasElement(FElement).AddRef{$IFDEF CheckPasTreeRefCount}('TPas2JsElementData.SetElement'){$ENDIF};
    Data:=FElement;
    while Data.CustomData is TPasElementBase do
      Data:=TPasElementBase(Data.CustomData);
    if Data.CustomData<>nil then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPas2JsElementData.SetElement INSERT ',ClassName);
      writeln('  ',GetObjName(Data.CustomData));
      {$ENDIF}
      raise EPas2JS.Create('');
      end;
    Data.CustomData:=Self;
    end;
end;

constructor TPas2JsElementData.Create;
begin

end;

destructor TPas2JsElementData.Destroy;
begin
  Element:=nil;
  Next:=nil;
  Owner:=nil;
  inherited Destroy;
end;

{ TAssignContext }

constructor TAssignContext.Create(PasEl: TPasElement; JSEl: TJSElement;
  aParent: TConvertContext);
begin
  inherited Create(PasEl, JSEl, aParent);
  Access:=caAssign;
  AccessContext:=Self;
end;

{ TSectionContext }

constructor TSectionContext.Create(PasEl: TPasElement; JSEl: TJSElement;
  aParent: TConvertContext);
begin
  inherited;
  IsGlobal:=true;
  SrcElements:=JSEl as TJSSourceElements;
end;

procedure TSectionContext.AddHeaderStatement(JS: TJSElement);
begin
  if JS=nil then exit;
  SrcElements.Statements.InsertNode(HeaderIndex).Node:=JS;
  inc(HeaderIndex);
end;

function TSectionContext.FindPrecompiledVar(const aName: string;
  WithParents: boolean): TPas2JSStoredLocalVar;
var
  i: Integer;
begin
  for i:=0 to length(PrecompiledVars)-1 do
    begin
    Result:=PrecompiledVars[i];
    if Result.Name=aName then
      exit;
    end;
  if not WithParents then
    exit(nil);
  Result:=inherited FindPrecompiledVar(aName,WithParents);
end;

function TSectionContext.FindPrecompiledVar(El: TPasElement;
  WithParents: boolean): TPas2JSStoredLocalVar;
var
  i: Integer;
begin
  for i:=0 to length(PrecompiledVars)-1 do
    begin
    Result:=PrecompiledVars[i];
    if Result.Element=El then
      exit;
    end;
  if not WithParents then
    exit(nil);
  Result:=inherited FindPrecompiledVar(El, WithParents);
end;

{ TFunctionContext }

constructor TFunctionContext.Create(PasEl: TPasElement; JSEl: TJSElement;
  aParent: TConvertContext);
begin
  inherited Create(PasEl, JSEl, aParent);
  ThisVar:=TFCLocalIdentifier.Create('this',nil,cvkNone);
  SetLength(LocalVars,1);
  LocalVars[0]:=ThisVar;
end;

destructor TFunctionContext.Destroy;
var
  i: Integer;
begin
  FreeAndNil(IntfElReleases);
  for i:=0 to length(LocalVars)-1 do
    FreeAndNil(LocalVars[i]);
  ThisVar:=nil;
  inherited Destroy;
end;

function TFunctionContext.AddLocalVar(aName: string; El: TPasElement;
  aKind: TCtxVarKind; AutoUnique: boolean): TFCLocalIdentifier;
var
  l: Integer;
  Ident, V: TFCLocalIdentifier;
  PV: TPas2JSStoredLocalVar;
begin
  Ident:=FindLocalVar(aName,true);
  if Ident<>nil then
    begin
    if AutoUnique then
      aName:=CreateLocalIdentifier(aName,El,aKind)
    else
      begin
      V:=FindLocalVar(aName,false);
      if V=nil then
        // overriding parent context
      else if El<>V.Element then
        // adding an alias, e.g. "this" for classtype and SelfArg
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TFunctionContext.AddLocalVar [20200608131330] Duplicate "'+aName+'" El='+GetObjPath(El),' Old=',GetObjPath(Ident.Element));
        {$ENDIF}
        raise EPas2JS.Create('[20200608131330] "'+aName+'" El='+GetObjPath(El));
        end;
      end;
    end
  else if aKind=cvkGlobal then
    begin
    // check precompiled names
    PV:=FindPrecompiledVar(El,true);
    if PV<>nil then
      aName:=PV.Name;
    end;
  // add
  l:=length(LocalVars);
  SetLength(LocalVars,l+1);
  Result:=TFCLocalIdentifier.Create(aName,El,aKind);
  LocalVars[l]:=Result;
end;

function TFunctionContext.AddLocalJSVar(aName: string; AutoUnique: boolean
  ): TFCLocalIdentifier;
begin
  Result:=AddLocalVar(aName,nil,cvkNone,AutoUnique);
end;

procedure TFunctionContext.Add_InterfaceRelease(El: TPasElement);
begin
  if IntfElReleases=nil then
    IntfElReleases:=TFPList.Create;
  if IntfElReleases.IndexOf(El)>=0 then exit;
  IntfElReleases.Add(El);
end;

function TFunctionContext.CreateLocalIdentifier(const Prefix: string;
  El: TPasElement; aKind: TCtxVarKind): string;
var
  l: Integer;
  PV: TPas2JSStoredLocalVar;
begin
  // check precompiled names
  if aKind=cvkGlobal then
    begin
    PV:=FindPrecompiledVar(El,true);
    if PV<>nil then
      exit(PV.Name);
    end;
  // find new name
  Result:=Prefix;
  l:=0;
  while (FindLocalVar(Result,true)<>nil)
      or ((aKind=cvkGlobal) and (FindPrecompiledVar(Result,true)<>nil)) do
    begin
    inc(l);
    Result:=Prefix+IntToStr(l);
    end;
end;

function TFunctionContext.ToString: string;
var
  s: string;
begin
  Result:=inherited ToString;
  if ThisVar.Element<>nil then
    begin
    str(ThisVar.Kind,s);
    Result:=Result+' this,Kind='+s+',El='+GetObjPath(ThisVar.Element);
    end;
end;

function TFunctionContext.GetLocalName(El: TPasElement;
  const Filter: TCtxVarKinds): string;

  function Check(V: TFCLocalIdentifier; FuncCtx: TFunctionContext): boolean;
  begin
    Result:=false;
    if (V.Name<>'') and (V.Element=El) and (V.Kind in Filter) then
      begin
      // found a candidate
      if (V.Name='this') and (FuncCtx<>Self) then
        exit;
      if (Filter<>cvkAll) then
        begin
        if FindLocalVar(V.Name,true)<>V then
          exit; // another var in a lower context hides this var
        end;
      Result:=true;
      end;
  end;

var
  V: TFCLocalIdentifier;
  FuncCtx: TFunctionContext;
  i: Integer;
begin
  if El=nil then exit('');
  FuncCtx:=Self;
  repeat
    if Check(FuncCtx.ThisVar,FuncCtx) then
      exit('this');
    for i:=0 to length(FuncCtx.LocalVars)-1 do
      begin
      V:=FuncCtx.LocalVars[i];
      if Check(V,FuncCtx) then
        exit(V.Name);
      end;
    FuncCtx:=FuncCtx.Parent.GetFunctionContext;
  until FuncCtx=nil;
  Result:='';
end;

function TFunctionContext.IndexOfLocalVar(const aName: string): integer;
var
  i: Integer;
begin
  for i:=0 to length(LocalVars)-1 do
    if LocalVars[i].Name=aName then exit(i);
  Result:=-1;
end;

function TFunctionContext.IndexOfLocalVar(El: TPasElement;
  const Filter: TCtxVarKinds): integer;
var
  i: Integer;
begin
  if El=nil then exit(-1);
  for i:=0 to length(LocalVars)-1 do
    if (LocalVars[i].Element=El) and (LocalVars[i].Kind in Filter) then
      exit(i);
  Result:=-1;
end;

function TFunctionContext.FindLocalVar(const aName: string; WithParents: boolean
  ): TFCLocalIdentifier;
var
  i: Integer;
  ParentFC: TFunctionContext;
begin
  i:=IndexOfLocalVar(aName);
  if i>=0 then
    exit(LocalVars[i]);
  if (not WithParents) or (Parent=nil) then
    exit(nil);
  ParentFC:=Parent.GetFunctionContext;
  if ParentFC=nil then
    exit(nil);
  Result:=ParentFC.FindLocalVar(aName,true);
end;

function TFunctionContext.FindPrecompiledVar(const aName: string;
  WithParents: boolean): TPas2JSStoredLocalVar;
var
  ParentFC: TFunctionContext;
begin
  if (not WithParents) or (Parent=nil) then
    exit(nil);
  ParentFC:=Parent.GetFunctionContext;
  if ParentFC=nil then
    exit(nil);
  Result:=ParentFC.FindPrecompiledVar(aName,true);
end;

function TFunctionContext.FindPrecompiledVar(El: TPasElement;
  WithParents: boolean): TPas2JSStoredLocalVar;
var
  ParentFC: TFunctionContext;
begin
  if (not WithParents) or (Parent=nil) then
    exit(nil);
  ParentFC:=Parent.GetFunctionContext;
  if ParentFC=nil then
    exit(nil);
  Result:=ParentFC.FindPrecompiledVar(El,true);
end;

procedure TFunctionContext.DoWriteStack(Index: integer);
var
  i: Integer;
begin
  inherited DoWriteStack(Index);
  {AllowWriteln}
  for i:=0 to length(LocalVars)-1 do
    writeln('    ',i,' ',LocalVars[i].Name,': ',GetObjName(LocalVars[i].Element),' ',LocalVars[i].Kind);
  {AllowWriteln-}
end;

{ TConvertContext }

constructor TConvertContext.Create(PasEl: TPasElement; JSEl: TJSElement;
  aParent: TConvertContext);
begin
  PasElement:=PasEl;
  JSElement:=JsEl;
  Parent:=aParent;
  if Parent<>nil then
    begin
    Resolver:=Parent.Resolver;
    Access:=aParent.Access;
    AccessContext:=aParent.AccessContext;
    ScannerBoolSwitches:=aParent.ScannerBoolSwitches;
    ScannerModeSwitches:=aParent.ScannerModeSwitches;
    end;
end;

function TConvertContext.GetRootModule: TPasModule;
var
  aContext: TConvertContext;
begin
  aContext:=Self;
  while aContext.Parent<>nil do
    aContext:=aContext.Parent;
  if aContext.PasElement is TPasModule then
    Result:=TPasModule(aContext.PasElement)
  else
    Result:=nil;
end;

function TConvertContext.GetRootContext: TConvertContext;
begin
  Result:=Self;
  while Result.Parent<>nil do
    Result:=Result.Parent;
end;

function TConvertContext.GetNonDotContext: TConvertContext;
begin
  Result:=Self;
  while Result is TDotContext do
    Result:=Result.Parent;
end;

function TConvertContext.GetFunctionContext: TFunctionContext;
begin
  Result:=TFunctionContext(GetContextOfType(TFunctionContext));
end;

function TConvertContext.GetLocalName(El: TPasElement;
  const Filter: TCtxVarKinds): string;
begin
  if Parent<>nil then
    Result:=Parent.GetLocalName(El,Filter)
  else
    Result:='';
end;

function TConvertContext.GetSelfContext: TFunctionContext;
var
  Ctx: TConvertContext;
  FuncContext: TFunctionContext;
  V: TFCLocalIdentifier;
begin
  Ctx:=Self;
  while Ctx<>nil do
    begin
    if (Ctx is TFunctionContext) then
      begin
      FuncContext:=TFunctionContext(Ctx);
      V:=FuncContext.ThisVar;
      if (V.Element is TPasMembersType)
          and (V.Kind in [cvkGlobal,cvkCurType,cvkInstance]) then
        exit(FuncContext);
      end;
    Ctx:=Ctx.Parent;
    end;
  Result:=nil;
end;

function TConvertContext.GetContextOfPasElement(El: TPasElement
  ): TConvertContext;
var
  ctx: TConvertContext;
begin
  Result:=nil;
  ctx:=Self;
  repeat
    if ctx.PasElement=El then
      exit(ctx);
    ctx:=ctx.Parent;
  until ctx=nil;
end;

function TConvertContext.GetFuncContextOfPasElement(El: TPasElement
  ): TFunctionContext;
var
  ctx: TConvertContext;
  Scope: TPas2JSProcedureScope;
begin
  Result:=nil;
  if El is TPasProcedure then
    begin
    Scope:=TPas2JSProcedureScope(El.CustomData);
    if Scope.ImplProc<>nil then
      El:=Scope.ImplProc;
    end;
  ctx:=Self;
  repeat
    if (ctx.PasElement=El) and (ctx is TFunctionContext) then
      exit(TFunctionContext(ctx));
    ctx:=ctx.Parent;
  until ctx=nil;
end;

function TConvertContext.GetContextOfType(aType: TConvertContextClass
  ): TConvertContext;
var
  ctx: TConvertContext;
begin
  Result:=nil;
  ctx:=Self;
  repeat
    if ctx is aType then
      exit(ctx);
    ctx:=ctx.Parent;
  until ctx=nil;
end;

function TConvertContext.GetMainSectionContext: TFunctionContext;
var
  Ctx: TConvertContext;
begin
  Ctx:=Self;
  repeat
    if Ctx is TSectionContext then
      Result:=TSectionContext(Ctx);
    Ctx:=Ctx.Parent;
  until Ctx=nil;
end;

function TConvertContext.CurrentModeSwitches: TModeSwitches;
begin
  if Resolver=nil then
    Result:=OBJFPCModeSwitches
  else
    Result:=Resolver.CurrentParser.CurrentModeswitches;
end;

function TConvertContext.GetGlobalFunc: TFunctionContext;
var
  Ctx: TConvertContext;
begin
  Ctx:=Self;
  while (Ctx<>nil) do
    begin
    if Ctx.IsGlobal and (Ctx.JSElement<>nil) and (Ctx is TFunctionContext) then
      exit(TFunctionContext(Ctx));
    Ctx:=Ctx.Parent;
    end;
  Result:=nil;
end;

procedure TConvertContext.WriteStack;
{AllowWriteln}
var
  SelfCtx: TFunctionContext;

  procedure W(Index: integer; AContext: TConvertContext);
  begin
    if AContext=SelfCtx then
      writeln('  SelfContext:');
    AContext.DoWriteStack(Index);
    if AContext.Parent<>nil then
      W(Index+1,AContext.Parent);
  end;

begin
  SelfCtx:=GetSelfContext;
  writeln('TConvertContext.WriteStack: START');
  W(1,Self);
  writeln('TConvertContext.WriteStack: END');
end;
{AllowWriteln-}

procedure TConvertContext.DoWriteStack(Index: integer);
begin
  {AllowWriteln}
  writeln('  ',Index,' ',ToString);
  {AllowWriteln-}
end;

function TConvertContext.ToString: string;
begin
  Result:='['+ClassName+']'
    +' pas='+GetObjName(PasElement)
    +' js='+GetObjName(JSElement)
    +' Global='+BoolToStr(IsGlobal,true);
end;

{ TPasToJSConverter }

// inline
function TPasToJSConverter.GetUseEnumNumbers: boolean;
begin
  Result:=coEnumNumbers in FOptions;
end;

// inline
function TPasToJSConverter.GetUseLowerCase: boolean;
begin
  Result:=coLowerCase in FOptions;
end;

// inline
function TPasToJSConverter.GetUseSwitchStatement: boolean;
begin
  Result:=coSwitchStatement in FOptions;
end;

// inline
function TPasToJSConverter.GetBIName(bin: TPas2JSBuiltInName): string;
begin
  Result:=FGlobals.BuiltInNames[bin];
end;

procedure TPasToJSConverter.AddGlobalClassMethod(aContext: TConvertContext;
  P: TPasProcedure);
var
  RootContext: TConvertContext;
begin
  RootContext:=aContext.GetRootContext;
  if not (RootContext is TRootContext) then
    DoError(20190226232141,RootContext.ClassName);
  TRootContext(RootContext).AddGlobalClassMethod(P);
end;

procedure TPasToJSConverter.AddToSourceElements(Src: TJSSourceElements;
  El: TJSElement);

Var
  List : TJSStatementList;
  AddEl : TJSElement;

begin
  While El<>nil do
  begin
    if El is TJSStatementList then
      begin
      List:=El as TJSStatementList;
      // List.A is first statement, List.B is next in list, chained.
      // -> add A, continue with B and free List
      AddEl:=List.A;
      El:=List.B;
      List.A:=Nil;
      List.B:=Nil;
      FreeAndNil(List);
      end
    else
      begin
      AddEl:=El;
      El:=Nil;
      end;
    Src.Statements.AddNode.Node:=AddEl;
  end;
end;

procedure TPasToJSConverter.RemoveFromSourceElements(Src: TJSSourceElements;
  El: TJSElement);
var
  Statements: TJSElementNodes;
  i: Integer;
begin
  Statements:=Src.Statements;
  for i:=Statements.Count-1 downto 0 do
    if Statements[i].Node=El then
      Statements.Delete(i);
end;

procedure TPasToJSConverter.SetGlobals(const AValue: TPasToJSConverterGlobals);
begin
  if FGlobals=AValue then Exit;
  if (FGlobals<>nil) and (FGlobals.Owner=Self) then
    FreeAndNil(FGlobals);
  FGlobals:=AValue;
end;

procedure TPasToJSConverter.SetReservedWords(const AValue: TJSReservedWordList
  );
var
  i: Integer;
begin
  if FReservedWords=AValue then Exit;
  for i:=0 to length(AValue)-2 do
    if CompareStr(AValue[i],AValue[i+1])>=0 then
      raise Exception.Create('TPasToJSConverter.SetPreservedWords "'+AValue[i]+'" >= "'+AValue[i+1]+'"');
  FReservedWords:=AValue;
end;

function TPasToJSConverter.ConvertModule(El: TPasModule;
  AContext: TConvertContext): TJSElement;
(*
Program:
 rtl.module('program',
    [<uses1>,<uses2>, ...],
    function(){
      var $mod = this;
      <programsection>
      this.$main=function(){
        <initialization>
        };
    });

Library:
 rtl.module('library',
    [<uses1>,<uses2>, ...],
    function(){
      var $mod = this;
      <librarysection>
      this.$main=function(){
        <initialization>
        };
    });
  export1 = pas.unit1.func1;

Unit without implementation:
 rtl.module('<unitname>',
    [<interface uses1>,<uses2>, ...],
    function(){
      var $mod = this;
      this.$impl = $impl;
      <interface>
      this.$init=function(){
        <initialization>
        };
    });

Unit with implementation:
 rtl.module('<unitname>',
    [<interface uses1>,<uses2>, ...],
    function(){
      var $mod = this;
      var $impl = $mod.$impl;
      <interface>
      $impl.$code=function(){
        };
      this.$init=function(){
        <initialization>
        };
    },
    [<implementation uses1>,<uses2>, ...],
    );
*)
Var
  aResolver: TPas2JSResolver;
  OuterSrc , Src: TJSSourceElements;
  RegModuleCall, Call: TJSCallExpression;
  ArgArray: TJSArguments;
  FunDecl, ImplFunc: TJSFunctionDeclarationStatement;
  UsesSection: TPasSection;
  ModuleName, ModVarName: String;
  IntfContext: TSectionContext;
  ImplVarSt: TJSVariableStatement;
  HasImplCode, ok, NeedRTLCheckVersion: Boolean;
  Prg: TPasProgram;
  Lib: TPasLibrary;
  ImplFuncAssignSt: TJSSimpleAssignStatement;
  IntfSecCtx: TInterfaceSectionContext;
  ModScope: TPas2JSModuleScope;
begin
  Result:=Nil;
  aResolver:=AContext.Resolver;
  if aResolver<>nil then
    ModScope:=El.CustomData as TPas2JSModuleScope
  else
    ModScope:=nil;
  OuterSrc:=TJSSourceElements(CreateElement(TJSSourceElements, El));
  Result:=OuterSrc;
  IntfContext:=nil;
  ok:=false;
  try
    // create 'rtl.module(...)'
    RegModuleCall:=CreateCallExpression(El);
    AddToSourceElements(OuterSrc,RegModuleCall);
    RegModuleCall.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),'module']);
    ArgArray := RegModuleCall.Args;
    RegModuleCall.Args:=ArgArray;

    // add module name parameter
    ModuleName:=TransformModuleName(El,false,AContext);
    ArgArray.Elements.AddElement.Expr:=CreateLiteralString(El,ModuleName);

    // add interface-uses-section parameter: [<interface uses1>,<uses2>, ...]
    UsesSection:=nil;
    if (El is TPasProgram) then
      UsesSection:=TPasProgram(El).ProgramSection
    else if (El is TPasLibrary) then
      UsesSection:=TPasLibrary(El).LibrarySection
    else
      UsesSection:=El.InterfaceSection;
    ArgArray.Elements.AddElement.Expr:=CreateUsesList(UsesSection,AContext);

    // add interface parameter: function(){}
    FunDecl:=CreateFunctionSt(El,true,true);
    ArgArray.AddElement(FunDecl);
    Src:=FunDecl.AFunction.Body.A as TJSSourceElements;

    if coUseStrict in Options then
      // "use strict" must be the first statement in a function
      AddToSourceElements(Src,CreateLiteralString(El,'use strict'));

    NeedRTLCheckVersion:=(coRTLVersionCheckUnit in Options)
        or ((coRTLVersionCheckSystem in Options) and IsSystemUnit(El));
    if NeedRTLCheckVersion then
      begin
      Call:=CreateCallExpression(El);
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnCheckVersion)]);
      Call.AddArg(CreateLiteralNumber(El,FGlobals.RTLVersion));
      AddToSourceElements(Src,Call);
      end;

    ImplVarSt:=nil;
    if El.ClassType=TPasModule then
      IntfContext:=TInterfaceSectionContext.Create(El,Src,AContext)
    else
      IntfContext:=TSectionContext.Create(El,Src,AContext);
    // add "var $mod = this;"
    IntfContext.ThisVar.Element:=El;
    IntfContext.ThisVar.Kind:=cvkGlobal;
    if El.CustomData is TPasModuleScope then
      IntfContext.ScannerBoolSwitches:=TPasModuleScope(El.CustomData).BoolSwitches;
    ModVarName:=GetBIName(pbivnModule);
    IntfContext.AddLocalVar(ModVarName,El,cvkGlobal,false);
    AddToSourceElements(Src,CreateVarStatement(ModVarName,
      CreatePrimitiveDotExpr('this',El),El));

    if (ModScope<>nil) then
      RestoreImplJSLocals(ModScope,IntfContext);

    if (El is TPasProgram) then
      begin // program
      Prg:=TPasProgram(El);
      if Assigned(Prg.ProgramSection) then
        AddToSourceElements(Src,ConvertDeclarations(Prg.ProgramSection,IntfContext));
      HasImplCode:=AddDelayedInits(Prg,Src,IntfContext);
      CreateInitSection(Prg,Src,IntfContext);
      end
    else if El is TPasLibrary then
      begin // library
      Lib:=TPasLibrary(El);
      if Assigned(Lib.LibrarySection) then
        AddToSourceElements(Src,ConvertDeclarations(Lib.LibrarySection,IntfContext));
      HasImplCode:=AddDelayedInits(Lib,Src,IntfContext);
      CreateInitSection(Lib,Src,IntfContext);
      // ToDo: append exports
      end
    else
      begin // unit
      IntfSecCtx:=TInterfaceSectionContext(IntfContext);
      if Assigned(El.ImplementationSection) then
        begin
        // add var $impl = $mod.$impl
        ImplVarSt:=CreateVarStatement(GetBIName(pbivnImplementation),
          CreateMemberExpression([ModVarName,GetBIName(pbivnImplementation)]),El);
        AddToSourceElements(Src,ImplVarSt);
        // register local var $impl
        IntfSecCtx.AddLocalVar(GetBIName(pbivnImplementation),El.ImplementationSection,cvkGlobal,false);
        end;
      if Assigned(El.InterfaceSection) then
        AddToSourceElements(Src,ConvertDeclarations(El.InterfaceSection,IntfSecCtx));

      ImplFunc:=CreateImplementationSection(El,IntfSecCtx);
      // add $mod.$implcode = ImplFunc;
      ImplFuncAssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      ImplFuncAssignSt.LHS:=CreateMemberExpression([ModVarName,GetBIName(pbivnImplCode)]);
      ImplFuncAssignSt.Expr:=ImplFunc;
      AddToSourceElements(Src,ImplFuncAssignSt);

      // append initialization section
      CreateInitSection(El,Src,IntfSecCtx);

      if TJSSourceElements(ImplFunc.AFunction.Body.A).Statements.Count>0 then
        HasImplCode:=true
      else
        begin
        // empty implementation

        // remove unneeded $impl from interface
        RemoveFromSourceElements(Src,ImplVarSt);
        // remove unneeded $mod.$implcode = function(){}
        RemoveFromSourceElements(Src,ImplFuncAssignSt);
        // keep impl uses section
        HasImplCode:=(El.ImplementationSection<>nil)
                       and (length(El.ImplementationSection.UsesClause)>0);
        end;

      if HasImplCode then
        // add implementation uses list: [<implementation uses1>,<uses2>, ...]
        ArgArray.AddElement(CreateUsesList(El.ImplementationSection,AContext));
      end; // end unit

    if (ModScope<>nil) and (coStoreImplJS in Options) then
      StoreImplJSLocals(ModScope,IntfContext);
    ok:=true;
  finally
    IntfContext.Free;
    if not ok then
      FreeAndNil(Result);
  end;
end;

function TPasToJSConverter.CreateElement(C: TJSElementClass; Src: TPasElement
  ): TJSElement;

var
  Line, Col: Integer;
begin
  if Assigned(Src) then
    begin
    TPasResolver.UnmangleSourceLineNumber(Src.SourceLinenumber,Line,Col);
    Result:=C.Create(Line,Col,Src.SourceFilename);
    end
  else
    Result:=C.Create(0,0);
end;

function TPasToJSConverter.CreateFreeOrNewInstanceExpr(Ref: TResolvedReference;
  AContext: TConvertContext): TJSCallExpression;
// class: create "$create("ProcName")"
// record: create "$new().ProcName()"
var
  C, SubCall: TJSCallExpression;
  Proc: TPasProcedure;
  ProcScope: TPasProcedureScope;
  ClassRecScope: TPasClassOrRecordScope;
  ClassOrRec: TPasElement;
  ArgEx: TJSLiteral;
  FunName, ProcName: String;
  DotExpr: TJSDotMemberExpression;
begin
  Result:=nil;
  //writeln('TPasToJSConverter.CreateFreeOrNewInstanceExpr Ref.Declaration=',GetObjName(Ref.Declaration));
  Proc:=Ref.Declaration as TPasProcedure;
  if Proc.Name='' then
    RaiseInconsistency(20170125191914,Proc);
  //writeln('TPasToJSConverter.CreateFreeOrNewInstanceExpr Proc.Name=',Proc.Name);
  ProcScope:=Proc.CustomData as TPasProcedureScope;
  //writeln('TPasToJSConverter.CreateFreeOrNewInstanceExpr ProcScope.Element=',GetObjName(ProcScope.Element),' ProcScope.ClassScope=',GetObjName(ProcScope.ClassOrRecordScope),' ProcScope.DeclarationProc=',GetObjName(ProcScope.DeclarationProc),' ProcScope.ImplProc=',GetObjName(ProcScope.ImplProc),' ProcScope.CustomData=',GetObjName(ProcScope.CustomData));
  ClassRecScope:=ProcScope.ClassRecScope;
  ClassOrRec:=ClassRecScope.Element;
  if ClassOrRec.Name='' then
    RaiseInconsistency(20170125191923,ClassOrRec);
  C:=CreateCallExpression(Ref.Element);
  try
    ProcName:=TransformElToJSName(Proc,AContext);
    if ClassOrRec.ClassType=TPasRecordType then
      begin
      // create "path.$new()"
      FunName:=CreateReferencePath(Proc,AContext,rpkPathWithDot,false,Ref)+GetBIName(pbifnRecordNew);
      SubCall:=CreateCallExpression(Ref.Element);
      SubCall.Expr:=CreatePrimitiveDotExpr(FunName,Ref.Element);
      // append ".ProcName"
      DotExpr:=CreateDotNameExpr(Ref.Element,SubCall,TJSString(ProcName));
      // as call: "path.$new().ProcName()"
      C.Expr:=DotExpr;
      end
    else
      begin
      // add "$create()"
      if rrfNewInstance in Ref.Flags then
        FunName:=GetBIName(pbifnClassInstanceNew)
      else
        FunName:=GetBIName(pbifnClassInstanceFree);
      FunName:=CreateReferencePath(Proc,AContext,rpkPathWithDot,false,Ref)+FunName;
      C.Expr:=CreatePrimitiveDotExpr(FunName,Ref.Element);
      // parameter: "ProcName"
      ArgEx := CreateLiteralString(Ref.Element,ProcName);
      C.AddArg(ArgEx);
      end;
    Result:=C;
  finally
    if Result=nil then
      C.Free;
  end;
end;

function TPasToJSConverter.CreateFunctionSt(El: TPasElement; WithBody: boolean;
  WithSrc: boolean): TJSFunctionDeclarationStatement;
var
  FuncSt: TJSFunctionDeclarationStatement;
begin
  FuncSt:=TJSFunctionDeclarationStatement(CreateElement(TJSFunctionDeclarationStatement,El));
  Result:=FuncSt;
  FuncSt.AFunction:=CreateFunctionDef(El,WithBody,WithSrc);
end;

function TPasToJSConverter.CreateFunctionDef(El: TPasElement;
  WithBody: boolean; WithSrc: boolean): TJSFuncDef;
begin
  Result:=TJSFuncDef.Create;
  if WithBody then
    begin
    Result.Body:=TJSFunctionBody(CreateElement(TJSFunctionBody,El));
    if WithSrc then
      Result.Body.A:=TJSSourceElements(CreateElement(TJSSourceElements, El));
    end;
end;

function TPasToJSConverter.ConvertUnaryExpression(El: TUnaryExpr;
  AContext: TConvertContext): TJSElement;

  procedure NotSupported(Id: TMaxPrecInt);
  var
    ResolvedEl: TPasResolverResult;
  begin
    if AContext.Resolver<>nil then
      begin
      AContext.Resolver.ComputeElement(El.Operand,ResolvedEl,[],El);
      DoError(Id,nIllegalQualifierInFrontOf,sIllegalQualifierInFrontOf,
        [OpcodeStrings[El.OpCode],AContext.Resolver.GetResolverResultDescription(ResolvedEl)],El);
      end
    else
      DoError(Id,nUnaryOpcodeNotSupported,sUnaryOpcodeNotSupported,
              [OpcodeStrings[El.OpCode]],El);
  end;

  function DerefPointer(TypeEl: TPasType): boolean;
  begin
    if TypeEl.ClassType=TPasRecordType then
      begin
      // PRecordVar^ -> PRecordVar
      ConvertUnaryExpression:=ConvertExpression(El.Operand,AContext);
      exit(true);
      end;
    Result:=false;
  end;

Var
  U : TJSUnaryExpression;
  E : TJSElement;
  ResolvedEl: TPasResolverResult;
  BitwiseNot, NeedLongWordBitFix: Boolean;
  aResolver: TPas2JSResolver;
  TypeEl, SubTypeEl: TPasType;
begin
  if AContext=nil then ;
  aResolver:=AContext.Resolver;
  Result:=Nil;
  U:=nil;
  Case El.OpCode of
    eopAdd:
      begin
      E:=ConvertExpression(El.Operand,AContext);
      U:=CreateUnaryPlus(E,El);
      U.A:=E;
      end;
    eopSubtract:
      begin
      E:=ConvertExpression(El.Operand,AContext);
      U:=TJSUnaryMinusExpression(CreateElement(TJSUnaryMinusExpression,El));
      U.A:=E;
      end;
    eopNot:
      begin
      E:=ConvertExpression(El.Operand,AContext);
      BitwiseNot:=true;
      if aResolver<>nil then
        begin
        aResolver.ComputeElement(El.Operand,ResolvedEl,[]);
        BitwiseNot:=ResolvedEl.BaseType in btAllJSInteger;
        NeedLongWordBitFix:=ResolvedEl.BaseType=btLongWord;
        end
      else
        NeedLongWordBitFix:=false;
      if BitwiseNot then
        begin
        U:=TJSUnaryInvExpression(CreateElement(TJSUnaryInvExpression,El));
        U.A:=E;
        if NeedLongWordBitFix then
          exit(CreateBitWiseLongword(El,U));
        end
      else
        U:=CreateUnaryNot(E,El);
      end;
    eopAddress:
      begin
      if aResolver=nil then
        NotSupported(20180423162321);
      aResolver.ComputeElement(El.Operand,ResolvedEl,[rcNoImplicitProc]);
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertUnaryExpression ',GetResolverResultDbg(ResolvedEl));
      {$ENDIF}
      if ResolvedEl.BaseType=btProc then
        begin
        if ResolvedEl.IdentEl is TPasProcedure then
          begin
          Result:=CreateCallback(El.Operand,ResolvedEl,false,AContext);
          exit;
          end;
        end
      else if (ResolvedEl.BaseType=btContext) then
        begin
        TypeEl:=ResolvedEl.LoTypeEl;
        if TypeEl.ClassType=TPasRecordType then
          begin
          // @RecVar -> RecVar
          Result:=ConvertExpression(El.Operand,AContext);
          exit;
          end;
        end;
      end;
    eopDeref:
      begin
      if aResolver=nil then
        NotSupported(20180423162350);
      aResolver.ComputeElement(El.Operand,ResolvedEl,[rcNoImplicitProc]);
      if ResolvedEl.BaseType=btPointer then
        begin
        TypeEl:=ResolvedEl.LoTypeEl;
        if DerefPointer(TypeEl) then exit;
        end
      else if (ResolvedEl.BaseType=btContext) then
        begin
        TypeEl:=ResolvedEl.LoTypeEl;
        if TypeEl.ClassType=TPasPointerType then
          begin
          SubTypeEl:=aResolver.ResolveAliasType(TPasPointerType(TypeEl).DestType);
          if DerefPointer(SubTypeEl) then exit;
          end;
        end;
      end;
    eopMemAddress:
      begin
      // @@ProcVar -> ProcVar
      Result:=ConvertExpression(El.Operand,AContext);
      exit;
      end;
  end;
  if U=nil then
    NotSupported(20180423162324);
  Result:=U;
end;

function TPasToJSConverter.ConvertInlineSpecializeExpr(
  El: TInlineSpecializeExpr; AContext: TConvertContext): TJSElement;
begin
  Result:=ConvertExpression(El.NameExpr,AContext);
end;

function TPasToJSConverter.GetExpressionValueType(El: TPasExpr;
  AContext: TConvertContext): TJSType;

  Function CombineValueType(A,B : TJSType) : TJSType;

  begin
    If (A=jstUNDEFINED) then
      Result:=B
    else if (B=jstUNDEFINED) then
      Result:=A
    else
      Result:=A; // pick the first
  end;

Var
  A,B : TJSType;

begin
  if (El is TBoolConstExpr) then
    Result:=jstBoolean
  else if (El is TPrimitiveExpr) then
    begin
    Case El.Kind of
      pekIdent : Result:=GetPasIdentValueType(El.Name,AContext);
      pekNumber : Result:=jstNumber;
      pekString : Result:=jstString;
      pekSet : Result:=jstUNDEFINED;
      pekNil : Result:=jstNull;
      pekBoolConst : Result:=jstBoolean;
      pekRange : Result:=jstUNDEFINED;
      pekFuncParams : Result:=jstUNDEFINED;
      pekArrayParams : Result:=jstUNDEFINED;
      pekListOfExp : Result:=jstUNDEFINED;
      pekInherited : Result:=jstUNDEFINED;
      pekSelf : Result:=jstObject;
    end
    end
  else if (El is TUnaryExpr) then
    Result:=GetExpressionValueType(TUnaryExpr(El).Operand,AContext)
  else if (El is TBinaryExpr) then
    begin
    A:=GetExpressionValueType(TBinaryExpr(El).Left,AContext);
    B:=GetExpressionValueType(TBinaryExpr(El).Right,AContext);
    Result:=CombineValueType(A,B);
    end
  else
    result:=jstUndefined
end;

function TPasToJSConverter.GetPasIdentValueType(AName: String;
  AContext: TConvertContext): TJSType;

begin
  if AContext=nil then ;
  if AName='' then ;
  Result:=jstUNDEFINED;
end;

function TPasToJSConverter.ComputeConstString(Expr: TPasExpr;
  AContext: TConvertContext; NotEmpty: boolean): String;
var
  Prim: TPrimitiveExpr;
begin
  if AContext.Resolver<>nil then
    Result:=AContext.Resolver.ComputeConstString(Expr,false,NotEmpty)
  else
    begin
    // fall back:
    Result:='';
    if Expr is TPrimitiveExpr then
      begin
      Prim:=TPrimitiveExpr(Expr);
      if Prim.Kind=pekString then
        Result:=Prim.Value
      else
        RaiseNotSupported(Prim,AContext,20170215124733);
      end
    else
      RaiseNotSupported(Expr,AContext,20170322121331);
    end;
end;

function TPasToJSConverter.IsLiteralInteger(El: TJSElement; out
  Number: TMaxPrecInt): boolean;
var
  Value: TJSValue;
begin
  Result:=false;
  if not (El is TJSLiteral) then exit;
  Value:=TJSLiteral(El).Value;
  if (Value.ValueType=jstNumber) then
    try
      Number:=Round(Value.AsNumber);
      if Number=Value.AsNumber then
        exit(true);
    except
    end;
end;

function TPasToJSConverter.IsLiteralNumber(El: TJSElement; out n: TJSNumber
  ): boolean;
var
  Value: TJSValue;
begin
  if not (El is TJSLiteral) then exit(false);
  Value:=TJSLiteral(El).Value;
  if Value.ValueType<>jstNumber then exit(false);
  Result:=true;
  n:=Value.AsNumber;
end;

function TPasToJSConverter.GetOverloadName(El: TPasElement;
  AContext: TConvertContext): string;
begin
  if AContext.Resolver<>nil then
    Result:=AContext.Resolver.GetOverloadName(El)
  else
    Result:=El.Name;
end;

function TPasToJSConverter.CanClashWithGlobal(El: TPasElement): boolean;
// returns true for JS variables accessed directly, i.e. without dot prefix
// which therefore must be checked if they clash with global JS identifiers.
var
  C: TClass;
begin
  C:=El.ClassType;
  if C=TPasArgument then
    Result:=true
  else if El.Parent is TProcedureBody then
    Result:=true
  else if El.Parent is TPasImplExceptOn then
    Result:=true
  else
    Result:=false;
end;

function TPasToJSConverter.ConvertBinaryExpression(El: TBinaryExpr;
  AContext: TConvertContext): TJSElement;
Const
  BinClasses : Array [TExprOpCode] of TJSBinaryClass = (
   Nil, //eopEmpty,
   TJSAdditiveExpressionPlus, // +
   TJSAdditiveExpressionMinus, // -
   TJSMultiplicativeExpressionMul, // *
   TJSMultiplicativeExpressionDiv, // /
   TJSMultiplicativeExpressionDiv, // div
   TJSMultiplicativeExpressionMod, // mod
   Nil, //eopPower
   TJSURShiftExpression, // shr
   TJSLShiftExpression, // shl
   Nil, // Not
   Nil, // And
   Nil, // Or
   Nil, // XOr
   TJSEqualityExpressionSEQ,
   TJSEqualityExpressionSNE,
   TJSRelationalExpressionLT,
   TJSRelationalExpressionGT,
   TJSRelationalExpressionLE,
   TJSRelationalExpressionGE,
   Nil, // In
   TJSRelationalExpressionInstanceOf, // is
   Nil, // As
   Nil, // Symmetrical diff
   Nil, // Address,
   Nil, // Deref
   Nil, // MemAddress
   Nil  // SubIndent,
  );

Var
  LeftResolved, RightResolved: TPasResolverResult;

  procedure NotSupportedRes(id: TMaxPrecInt);
  begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertBinaryExpression.NotSupportedRes',
      ' Left=',GetResolverResultDbg(LeftResolved),
      ' Op=',ExprKindNames[El.Kind],
      ' Right=',GetResolverResultDbg(RightResolved));
    {$ENDIF}
    RaiseNotSupported(El,AContext,id,
      GetResolverResultDbg(LeftResolved)+ExprKindNames[El.Kind]
        +GetResolverResultDbg(RightResolved));
  end;

  function BitwiseOpNeedLongwordFix: boolean;
  begin
    Result:=((LeftResolved.BaseType=btLongWord) and (RightResolved.BaseType<=btLongWord))
        or ((RightResolved.BaseType=btLongWord) and (LeftResolved.BaseType<=btLongWord));
  end;

  function CreateBitwiseLongwordOp(A, B: TJSElement; C: TJSBinaryClass): TJSElement;
  var
    R: TJSBinary;
  begin
    R:=TJSBinary(CreateElement(C,El));
    R.A:=A;
    R.B:=B;
    Result:=CreateBitWiseLongword(El,R);
  end;

var
  R : TJSBinary;
  C : TJSBinaryClass;
  A,B: TJSElement;
  UseBitwiseOp: Boolean;
  Call: TJSCallExpression;
  Flags: TPasResolverComputeFlags;
  ModeSwitches: TModeSwitches;
  aResolver: TPas2JSResolver;
  LeftTypeEl, RightTypeEl: TPasType;
  OldAccess: TCtxAccess;
begin
  Result:=Nil;
  aResolver:=AContext.Resolver;

  case El.OpCode of
  eopAdd:
    begin
    Result:=ConvertBinaryExpressionMultiAdd(El,aContext);
    exit;
    end;
  eopSubIdent:
    begin
    Result:=ConvertSubIdentExpression(El,AContext);
    exit;
    end;
  eopNone:
    if El.left is TInheritedExpr then
      begin
      Result:=ConvertInheritedExpr(TInheritedExpr(El.left),AContext);
      exit;
      end;
  end;

  OldAccess:=AContext.Access;
  AContext.Access:=caRead;
  Call:=nil;
  A:=nil;
  B:=nil;
  try
    A:=ConvertExpression(El.left,AContext);
    B:=ConvertExpression(El.right,AContext);

    if aResolver<>nil then
      begin
      ModeSwitches:=AContext.CurrentModeSwitches;
      // compute left
      Flags:=[];
      if El.OpCode in [eopEqual,eopNotEqual] then
        if not (msDelphi in ModeSwitches) then
          Flags:=[rcNoImplicitProcType];
      aResolver.ComputeElement(El.left,LeftResolved,Flags);

      // compute right
      Flags:=[];
      if (El.OpCode in [eopEqual,eopNotEqual])
          and not (msDelphi in ModeSwitches) then
        begin
        if LeftResolved.BaseType=btNil then
          Flags:=[rcNoImplicitProcType]
        else if aResolver.IsProcedureType(LeftResolved,true) then
          Flags:=[rcNoImplicitProcType]
        else
          Flags:=[];
        end;
      aResolver.ComputeElement(El.right,RightResolved,Flags);

      Result:=ConvertBinaryExpressionRes(El,AContext,LeftResolved,RightResolved,A,B);
      if Result<>nil then exit;
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertBinaryExpression Left=',GetResolverResultDbg(LeftResolved),' Right=',GetResolverResultDbg(RightResolved));
      {$ENDIF}
      end;

    C:=BinClasses[El.OpCode];
    if C=nil then
      Case El.OpCode of
      eopAs :
        begin
        // "A as B"
        Call:=CreateCallExpression(El);
        LeftTypeEl:=LeftResolved.LoTypeEl;
        RightTypeEl:=RightResolved.LoTypeEl;
        if LeftTypeEl is TPasClassType then
          begin
          if RightTypeEl is TPasClassType then
            case TPasClassType(LeftTypeEl).ObjKind of
            okClass:
              case TPasClassType(RightTypeEl).ObjKind of
              okClass:
                // ClassInstVar is ClassType
                if TPasClassType(RightResolved.LoTypeEl).IsExternal then
                  // B is external class -> "rtl.asExt(A,B)"
                  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnAsExt),El)
                else
                  // otherwise -> "rtl.as(A,B)"
                  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnAs),El);
              okInterface:
                begin
                // ClassInstVar as IntfType
                case TPasClassType(RightTypeEl).InterfaceType of
                citCom:
                  begin
                  // COM:  $ir.ref(rtl.queryIntfT(objVar,intftype),"id")
                  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfQueryIntfT),El);
                  Call.AddArg(A);
                  Call.AddArg(B);
                  Call:=CreateIntfRef(Call,AContext,El);
                  Result:=Call;
                  exit;
                  end;
                citCorba:
                  // CORBA:  rtl.getIntfT(objVar,intftype)
                  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGetIntfT),El);
                else RaiseNotSupported(El,AContext,20180401225752);
                end;
                end
              else
                NotSupportedRes(20180327214535);
              end;
            okInterface:
              case TPasClassType(RightTypeEl).ObjKind of
              okClass:
                // IntfVar as ClassType ->  rtl.intfAsClass(intfvar,classtype)
                Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfAsClass),El);
              okInterface:
                // IntfVar as IntfType
                if TPasClassType(LeftTypeEl).InterfaceType=citCom then
                  // COM -> "rtl.intfAsIntfT(A,B)"
                  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfAsIntfT),El)
                else
                  // CORBA -> "rtl.as(A,B)"
                  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnAs),El);
              else
                NotSupportedRes(20180327214545);
              end;
            else
              NotSupportedRes(20180327214559);
            end
          else if RightTypeEl is TPasClassOfType then
            begin
            // ClassInstVar is ClassOfType -> "rtl.as(A,B)"
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnAs),El);
            end;
          end;
        Call.AddArg(A);
        Call.AddArg(B);
        Result:=Call;
        exit;
        end;
      eopAnd:
        begin
        if aResolver<>nil then
          begin
          UseBitwiseOp:=((LeftResolved.BaseType in btAllJSInteger)
                     or (RightResolved.BaseType in btAllJSInteger));
          if UseBitwiseOp then
            begin
            if (LeftResolved.BaseType in [btIntDouble,btUIntDouble])
                and (RightResolved.BaseType in [btIntDouble,btUIntDouble]) then
              begin
              Call:=CreateCallExpression(El);
              Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnBitwiseNativeIntAnd)]);
              Call.AddArg(A);
              Call.AddArg(B);
              Result:=Call;
              exit;
              end
            else if BitwiseOpNeedLongwordFix then
              begin
              Result:=CreateBitwiseLongwordOp(A,B,TJSBitwiseAndExpression);
              exit;
              end;
            end;
          end
        else
          UseBitwiseOp:=(GetExpressionValueType(El.left,AContext)=jstNumber)
            or (GetExpressionValueType(El.right,AContext)=jstNumber);
        if UseBitwiseOp then
          C:=TJSBitwiseAndExpression
        else
          C:=TJSLogicalAndExpression;
        end;
      eopOr:
        begin
        if aResolver<>nil then
          begin
          UseBitwiseOp:=((LeftResolved.BaseType in btAllJSInteger)
                     or (RightResolved.BaseType in btAllJSInteger));
          if UseBitwiseOp then
            begin
            if ((LeftResolved.BaseType in [btIntDouble,btUIntDouble])
                or (RightResolved.BaseType in [btIntDouble,btUIntDouble])) then
              begin
              Call:=CreateCallExpression(El);
              Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnBitwiseNativeIntOr)]);
              Call.AddArg(A);
              Call.AddArg(B);
              Result:=Call;
              exit;
              end
            else if BitwiseOpNeedLongwordFix then
              begin
              Result:=CreateBitwiseLongwordOp(A,B,TJSBitwiseOrExpression);
              exit;
              end;
            end;
          end
        else
          UseBitwiseOp:=(GetExpressionValueType(El.left,AContext)=jstNumber)
            or (GetExpressionValueType(El.right,AContext)=jstNumber);
        if UseBitwiseOp then
          C:=TJSBitwiseOrExpression
        else
          C:=TJSLogicalOrExpression;
        end;
      eopXor:
        begin
        if aResolver<>nil then
          begin
          UseBitwiseOp:=((LeftResolved.BaseType in btAllJSInteger)
                     or (RightResolved.BaseType in btAllJSInteger));
          if UseBitwiseOp then
            begin
            if ((LeftResolved.BaseType in [btIntDouble,btUIntDouble])
                or (RightResolved.BaseType in [btIntDouble,btUIntDouble])) then
              begin
              Call:=CreateCallExpression(El);
              Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnBitwiseNativeIntXor)]);
              Call.AddArg(A);
              Call.AddArg(B);
              Result:=Call;
              exit;
              end
            else if BitwiseOpNeedLongwordFix then
              begin
              Result:=CreateBitwiseLongwordOp(A,B,TJSBitwiseXOrExpression);
              exit;
              end;
            end;
          end
        else
          UseBitwiseOp:=(GetExpressionValueType(El.left,AContext)=jstNumber)
            or (GetExpressionValueType(El.right,AContext)=jstNumber);
        if UseBitwiseOp then
          C:=TJSBitwiseXOrExpression
        else
          C:=TJSBitwiseXOrExpression; // no logical xor in JS. bitwise works for boolean too
        end;
      eopPower:
        begin
        Call:=CreateCallExpression(El);
        Call.Expr:=CreatePrimitiveDotExpr('Math.pow',El);
        Call.AddArg(A);
        Call.AddArg(B);
        Result:=Call;
        end;
      else
        if C=nil then
          DoError(20161024191244,nBinaryOpcodeNotSupported,sBinaryOpcodeNotSupported,[OpcodeStrings[El.OpCode]],El);
      end;
    if (Result=Nil) and (C<>Nil) then
      begin
      R:=TJSBinary(CreateElement(C,El));
      R.A:=A; A:=nil;
      R.B:=B; B:=nil;
      Result:=R;

      case El.OpCode of
      eopDiv:
        begin
        // convert "a div b" to "rtl.trunc(a/b)"
        Result:=CreateTruncFloor(El,Result,true);
        end;
      end;

      if (bsOverflowChecks in AContext.ScannerBoolSwitches) and (aResolver<>nil) then
        case El.OpCode of
        eopAdd,eopSubtract:
          if (LeftResolved.BaseType in btAllJSOverflowAddSubType)
              or (RightResolved.BaseType in btAllJSOverflowAddSubType) then
            Result:=CreateOverflowCheckCall(Result,El);
        eopMultiply:
          if (LeftResolved.BaseType in btAllJSOverflowMultType)
              or (RightResolved.BaseType in btAllJSOverflowMultType) then
            Result:=CreateOverflowCheckCall(Result,El);
        end;
      end;
  finally
    AContext.Access:=OldAccess;
    if Result=nil then
      begin
      A.Free;
      B.Free;
      end;
  end;
end;

function TPasToJSConverter.ConvertBinaryExpressionRes(El: TBinaryExpr;
  AContext: TConvertContext; const LeftResolved,
  RightResolved: TPasResolverResult; var A, B: TJSElement): TJSElement;

  procedure NotSupported(id: TMaxPrecInt);
  begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertBinaryExpressionRes.NotSupported',
      ' Left=',GetResolverResultDbg(LeftResolved),
      ' Op=',ExprKindNames[El.Kind],
      ' Right=',GetResolverResultDbg(RightResolved));
    {$ENDIF}
    RaiseNotSupported(El,AContext,id,
      GetResolverResultDbg(LeftResolved)+ExprKindNames[El.Kind]
        +GetResolverResultDbg(RightResolved));
  end;

  function CreateEqualCallback: TJSElement;
  var
    Call: TJSCallExpression;
  begin
    // convert "proctypeA = proctypeB" to "rtl.eqCallback(proctypeA,proctypeB)"
    Call:=CreateCallExpression(El);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnProcType_Equal)]);
    Call.AddArg(A);
    A:=nil;
    Call.AddArg(B);
    B:=nil;
    if El.OpCode=eopNotEqual then
      begin
      // convert "proctypeA <> proctypeB" to "!rtl.eqCallback(proctypeA,proctypeB)"
      Result:=CreateUnaryNot(Call,El);
      end
    else
      Result:=Call;
  end;

  procedure ConcatArray(ArrayType: TPasArrayType);
  var
    Call: TJSCallExpression;
  begin
    Call:=CreateArrayConcat(ArrayType,El,AContext);
    Result:=Call;
    Call.AddArg(A); A:=nil;
    Call.AddArg(B); B:=nil;
  end;

var
  aResolver: TPas2JSResolver;
  FunName: String;
  Call: TJSCallExpression;
  InOp: TJSRelationalExpressionIn;
  TypeEl, LeftTypeEl, RightTypeEl: TPasType;
  SNE: TJSEqualityExpressionSNE;
  JSBinClass: TJSBinaryClass;
  ResolvedEl: TPasResolverResult;
  AInt, BInt: TMaxPrecInt;
  LArrType: TPasArrayType;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertBinaryExpressionRes OpCode="',OpcodeStrings[El.OpCode],'" Left=',GetResolverResultDbg(LeftResolved),' Right=',GetResolverResultDbg(RightResolved));
  {$ENDIF}
  Result:=nil;
  aResolver:=AContext.Resolver;
  LeftTypeEl:=LeftResolved.LoTypeEl;
  RightTypeEl:=RightResolved.LoTypeEl;
  if (LeftResolved.BaseType in [btSet,btArrayOrSet])
      and (RightResolved.BaseType in [btSet,btArrayOrSet]) then
    begin
    // set operators -> rtl.operatorfunction(a,b)
    case El.OpCode of
    eopAdd: FunName:=GetBIName(pbifnSet_Union);
    eopSubtract: FunName:=GetBIName(pbifnSet_Difference);
    eopMultiply: FunName:=GetBIName(pbifnSet_Intersect);
    eopSymmetricaldifference: FunName:=GetBIName(pbifnSet_SymDiffSet);
    eopEqual: FunName:=GetBIName(pbifnSet_Equal);
    eopNotEqual: FunName:=GetBIName(pbifnSet_NotEqual);
    eopGreaterThanEqual: FunName:=GetBIName(pbifnSet_GreaterEqual);
    eopLessthanEqual: FunName:=GetBIName(pbifnSet_LowerEqual);
    else
      DoError(20170209151300,nBinaryOpcodeNotSupported,sBinaryOpcodeNotSupported,[OpcodeStrings[El.OpCode]],El);
    end;
    Call:=CreateCallExpression(El);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),FunName]);
    Call.AddArg(A); A:=nil;
    Call.AddArg(B); B:=nil;
    Result:=Call;
    exit;
    end
  else if (El.OpCode=eopIn) and (RightResolved.BaseType in [btSet,btArrayOrSet])  then
    begin
    // a in b -> a in b
    if not (A is TJSLiteral) or (TJSLiteral(A).Value.ValueType<>jstNumber) then
      begin
      FreeAndNil(A);
      A:=CreateSetLiteralElement(El.left,AContext);
      end;
    InOp:=TJSRelationalExpressionIn(CreateElement(TJSRelationalExpressionIn,El));
    InOp.A:=A; A:=nil;
    InOp.B:=B; B:=nil;
    Result:=InOp;
    exit;
    end
  else if (El.OpCode=eopAdd)
        and ((LeftResolved.BaseType=btContext) and (LeftResolved.LoTypeEl.ClassType=TPasArrayType)) then
    begin
    // Arr+Arr  Arr+[]  Arr+[...]
    ConcatArray(TPasArrayType(LeftResolved.LoTypeEl));
    exit;
    end
  else if (El.OpCode=eopAdd)
        and ((RightResolved.BaseType=btContext) and (RightResolved.LoTypeEl.ClassType=TPasArrayType)) then
    begin
    // []+Arr  [...]+Arr
    ConcatArray(TPasArrayType(RightResolved.LoTypeEl));
    exit;
    end
  else if (El.OpCode=eopAdd)
        and (LeftResolved.BaseType=btArrayLit) then
    begin
    // [...]+[]  [...]+[...]
    SetResolverValueExpr(ResolvedEl,LeftResolved.SubType,LeftResolved.LoTypeEl,
      LeftResolved.HiTypeEl,El.left,LeftResolved.Flags);
    Call:=CreateArrayConcat(ResolvedEl,El,AContext);
    Result:=Call;
    Call.AddArg(A); A:=nil;
    Call.AddArg(B); B:=nil;
    exit;
    end
  else if El.OpCode in [eopShl,eopShr] then
    begin
    if LeftResolved.BaseType in [btIntDouble,btUIntDouble] then
      begin
      // BigInt shl/shr   JavaScript bitwise operators only supports 32bit
      if IsLiteralInteger(B,BInt) then
        begin
        // BigInt shl/shr const
        if BInt>=54 then
          begin
          // A shl 54 -> 0
          // A shr 54 -> 0
          Result:=CreateLiteralNumber(El,0);
          FreeAndNil(A);
          FreeAndNil(B);
          exit;
          end
        else if BInt<=0 then
          begin
          // A shl 0 -> A
          // A shr 0 -> A
          Result:=A;
          A:=nil;
          FreeAndNil(B);
          exit;
          end
        else if IsLiteralInteger(A,AInt) then
          begin
          // const shl const  ->  const
          if El.OpCode=eopShl then
            AInt:=AInt shl BInt
          else
            AInt:=AInt shr BInt;
          if (AInt>=0) and (AInt<=MaxSafeIntDouble) then
            begin
            TJSLiteral(A).Value.AsNumber:=AInt;
            Result:=A;
            FreeAndNil(B);
            exit;
            end;
          end
        else if El.OpCode=eopShr then
          begin
          // BigInt shr const -> Math.floor(A/otherconst)
          Result:=CreateTruncFloor(El,CreateDivideNumber(El,A,TMaxPrecInt(1) shl BInt),false);
          A:=nil;
          FreeAndNil(B);
          exit;
          end;
        end;
      // use rtl.shl(a,b)
      Call:=CreateCallExpression(El);
      Result:=Call;
      if El.OpCode=eopShl then
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnBitwiseNativeIntShl)])
      else
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnBitwiseNativeIntShr)]);
      Call.AddArg(A); A:=nil;
      Call.AddArg(B); B:=nil;
      exit;
      end
    else if LeftResolved.BaseType=btLongWord then
      begin
      // aLongWord shl b  ->  rtl.lw(a << b)
      if El.OpCode=eopShl then
        JSBinClass:=TJSLShiftExpression
      else
        JSBinClass:=TJSURShiftExpression;
      Result:=TJSBinaryExpression(CreateElement(JSBinClass,El));
      TJSBinaryExpression(Result).A:=A; A:=nil;
      TJSBinaryExpression(Result).B:=B; B:=nil;
      Result:=CreateBitWiseLongword(El,Result);
      exit;
      end;
    end
  else if (LeftResolved.BaseType=btCurrency) or (RightResolved.BaseType=btCurrency) then
    begin
    case El.OpCode of
    eopAdd,eopSubtract,
    eopEqual, eopNotEqual,  // Logical
    eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual: // ordering
      begin
      // currency + currency  ->  currency + currency
      // currency + number  ->  currency + number*10000
      // number + currency  ->  number*10000 + currency
      case El.OpCode of
      eopAdd: JSBinClass:=TJSAdditiveExpressionPlus;
      eopSubtract: JSBinClass:=TJSAdditiveExpressionMinus;
      eopEqual: JSBinClass:=TJSEqualityExpressionSEQ;
      eopNotEqual: JSBinClass:=TJSEqualityExpressionSNE;
      eopLessThan: JSBinClass:=TJSRelationalExpressionLT;
      eopGreaterThan: JSBinClass:=TJSRelationalExpressionGT;
      eopLessthanEqual: JSBinClass:=TJSRelationalExpressionLE;
      eopGreaterThanEqual: JSBinClass:=TJSRelationalExpressionGE;
      end;
      Result:=TJSBinary(CreateElement(JSBinClass,El));
      if LeftResolved.BaseType<>btCurrency then
        A:=CreateMulNumber(El,A,10000);
      TJSBinary(Result).A:=A; A:=nil;
      if RightResolved.BaseType<>btCurrency then
        B:=CreateMulNumber(El,B,10000);
      TJSBinary(Result).B:=B; B:=nil;
      if (bsOverflowChecks in AContext.ScannerBoolSwitches)
        and (El.OpCode in [eopAdd,eopSubtract]) then
          Result:=CreateOverflowCheckCall(Result,El);
      exit;
      end;
    eopMultiply:
      begin
      // currency * currency  ->  (currency * currency)/10000
      // currency * number  ->  currency * number
      // number * currency  ->  number * currency
      Result:=TJSMultiplicativeExpressionMul(CreateElement(TJSMultiplicativeExpressionMul,El));
      TJSBinaryExpression(Result).A:=A; A:=nil;
      TJSBinaryExpression(Result).B:=B; B:=nil;
      if (LeftResolved.BaseType=btCurrency) and (RightResolved.BaseType=btCurrency) then
        Result:=CreateDivideNumber(El,Result,10000);
      if (bsOverflowChecks in AContext.ScannerBoolSwitches) then
        Result:=CreateOverflowCheckCall(Result,El);
      exit;
      end;
    eopDivide:
      begin
      // currency / currency  ->  rtl.trunc((currency/currency)*10000)
      // currency / number  ->  rtl.trunc(currency/number)
      // number / currency  ->  rtl.trunc(number/currency)
      Result:=TJSMultiplicativeExpressionDiv(CreateElement(TJSMultiplicativeExpressionDiv,El));
      TJSBinaryExpression(Result).A:=A; A:=nil;
      TJSBinaryExpression(Result).B:=B; B:=nil;
      if (LeftResolved.BaseType=btCurrency) and (RightResolved.BaseType=btCurrency) then
        Result:=CreateMulNumber(El,Result,10000);
      Result:=CreateTruncFloor(El,Result,true);
      exit;
      end;
    eopPower:
      begin
      // currency^^currency  ->  rtl.trunc(Math.pow(currency/10000,currency/10000)*10000)
      // currency^^number  ->  rtl.trunc(Math.pow(currency/10000,number)*10000)
      // number^^currency  ->  rtl.trunc(Math.pow(number,currency/10000)*10000)
      if LeftResolved.BaseType=btCurrency then
        A:=CreateDivideNumber(El,A,10000);
      if RightResolved.BaseType=btCurrency then
        B:=CreateDivideNumber(El,B,10000);
      Call:=CreateCallExpression(El);
      Call.Expr:=CreatePrimitiveDotExpr('Math.pow',El);
      Call.AddArg(A); A:=nil;
      Call.AddArg(B); B:=nil;
      Result:=CreateMulNumber(El,Call,10000);
      Result:=CreateTruncFloor(El,Result,true);
      end
    else
      RaiseNotSupported(El,AContext,20180422104215);
    end;
    end
  else if (LeftResolved.BaseType=btPointer)
      or ((LeftResolved.BaseType=btContext) and (LeftTypeEl.ClassType=TPasPointerType)) then
    case El.OpCode of
    eopEqual,eopNotEqual: ;
    else
      DoError(20180423114054,nIllegalQualifierAfter,sIllegalQualifierAfter,
        [OpcodeStrings[El.OpCode],aResolver.GetResolverResultDescription(LeftResolved,true)],El);
    end
  else if (RightResolved.BaseType=btPointer)
      or ((RightResolved.BaseType=btContext) and (RightTypeEl.ClassType=TPasPointerType)) then
    case El.OpCode of
    eopEqual,eopNotEqual: ;
    else
      DoError(20180423114246,nIllegalQualifierInFrontOf,sIllegalQualifierInFrontOf,
        [OpcodeStrings[El.OpCode],aResolver.GetResolverResultDescription(RightResolved,true)],El);
    end
  else if (El.OpCode=eopIs) then
    begin
    // "A is B"
    Call:=CreateCallExpression(El);
    Result:=Call;
    Call.AddArg(A); A:=nil;
    if (RightResolved.IdentEl is TPasType) then
      TypeEl:=aResolver.ResolveAliasType(TPasType(RightResolved.IdentEl))
    else
      TypeEl:=nil;

    if (TypeEl is TPasClassOfType) then
      begin
      // "A is class-of-type" -> use the class
      FreeAndNil(B);
      TypeEl:=aResolver.ResolveAliasType(TPasClassOfType(TypeEl).DestType);
      B:=CreateReferencePathExpr(TypeEl,AContext);
      end;

    if (LeftResolved.BaseType=btCustom) then
      begin
      // aJSValue is ... -> "rtl.isExt(A,B,mode)"
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIsExt)]);
      Call.AddArg(B); B:=nil;
      if RightTypeEl is TPasClassType then
        Call.AddArg(CreateLiteralNumber(El.right,IsExtModePasClassInstance))
      else if RightTypeEl is TPasClassOfType then
        Call.AddArg(CreateLiteralNumber(El.right,IsExtModePasClass))
      else
        NotSupported(20180119005904);
      end
    else if (RightTypeEl is TPasClassType) and TPasClassType(RightTypeEl).IsExternal then
      begin
      // B is an external class -> "rtl.isExt(A,B)"
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIsExt)]);
      Call.AddArg(B); B:=nil;
      end
    else if LeftTypeEl is TPasClassOfType then
      begin
      // A is a TPasClassOfType -> "rtl.is(A,B)"
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIs)]);
      Call.AddArg(B); B:=nil;
      end
    else
      begin
      if LeftTypeEl is TPasClassType then
        begin
        if RightTypeEl is TPasClassType then
          case TPasClassType(LeftTypeEl).ObjKind of
          okClass:
            case TPasClassType(RightTypeEl).ObjKind of
            okClass: ;
            okInterface:
              begin
              // ClassInstVar is IntfType
              case TPasClassType(RightTypeEl).InterfaceType of
              citCom:
                begin
                // COM:  rtl.queryIntfIsT(A,B)
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfQueryIntfIsT)]);
                Call.AddArg(B); B:=nil;
                end;
              citCorba:
                begin
                // CORBA:  rtl.getIntfT(A,B)!==null
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfGetIntfT)]);
                Call.AddArg(B); B:=nil;
                SNE:=TJSEqualityExpressionSNE(CreateElement(TJSEqualityExpressionSNE,El));
                Result:=SNE;
                SNE.A:=Call;
                SNE.B:=CreateLiteralNull(El);
                end;
              else
                RaiseNotSupported(El,AContext,20180401225502,InterfaceTypeNames[TPasClassType(RightTypeEl).InterfaceType]);
              end;
              exit;
              end;
            else
              NotSupported(20180327210501);
            end;
          okInterface:
            case TPasClassType(RightTypeEl).ObjKind of
            okClass:
              begin
              // IntfVar is ClassType  ->  rtl.intfIsClass(A,B)
              Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfIsClass)]);
              Call.AddArg(B); B:=nil;
              exit;
              end;
            okInterface:
              if TPasClassType(LeftTypeEl).InterfaceType=citCom then
                begin
                // COM: IntfVar is IntfType  ->  rtl.intfIsIntfT(A,B)
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfIsIntf)]);
                Call.AddArg(B); B:=nil;
                exit;
                end;
            else
              NotSupported(20180327210741);
            end;
          else
            NotSupported(20180327210251);
          end;
        end;

      // use directly "B.isPrototypeOf(A)"
      Call.Expr:=CreateDotNameExpr(El,B,'isPrototypeOf');
      B:=nil;
      end;
    exit;
    end
  else if (El.OpCode in [eopEqual,eopNotEqual]) then
    begin
    if aResolver.IsProcedureType(LeftResolved,true) then
      begin
      if RightResolved.BaseType=btNil then
      else if aResolver.IsProcedureType(RightResolved,true)
          or aResolver.IsJSBaseType(RightResolved,pbtJSValue,true) then
        exit(CreateEqualCallback);
      end
    else if aResolver.IsProcedureType(RightResolved,true) then
      begin
      if LeftResolved.BaseType=btNil then
      else if aResolver.IsJSBaseType(LeftResolved,pbtJSValue,true) then
        exit(CreateEqualCallback);
      end
    else if LeftResolved.BaseType=btNil then
      begin
      if RightResolved.BaseType=btContext then
        begin
        RightTypeEl:=RightResolved.LoTypeEl;
        if RightTypeEl.ClassType=TPasArrayType then
          begin
          // convert "nil = array" to "rtl.length(array) > 0"
          FreeAndNil(A);
          Result:=CreateCmpArrayWithNil(El,B,El.OpCode);
          B:=nil;
          exit;
          end;
        end;
      end
    else if LeftResolved.BaseType in btAllStrings then
      begin
      if RightResolved.BaseType=btContext then
        begin
        RightTypeEl:=RightResolved.LoTypeEl;
        if RightTypeEl.ClassType=TPasRecordType then
          begin
          if aResolver.IsTGUID(TPasRecordType(RightTypeEl)) then
            begin
            // "aString=GuidVar"  ->  "GuidVar.$eq(rtl.createTGUID(aString))"
            Call:=CreateCallExpression(El);
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfCreateTGUID),El);
            Call.AddArg(A);
            A:=Call;
            Call:=CreateCallExpression(El);
            Call.Expr:=CreateDotNameExpr(El,B,TJSString(GetBIName(pbifnRecordEqual)));
            B:=nil;
            Call.AddArg(A);
            A:=nil;
            if El.OpCode=eopNotEqual then
              Result:=CreateUnaryNot(Call,El)
            else
              Result:=Call;
            exit;
            end;
          end
        else if RightTypeEl.ClassType=TPasClassType then
          begin
          if TPasClassType(RightTypeEl).ObjKind=okInterface then
            begin
            // "aString=IntfTypeOrVar"  ->  "aString===IntfTypeOrVar.$guid"
            B:=CreateDotNameExpr(El.left,B,TJSString(GetBIName(pbivnIntfGUID)));
            end;
          end;
        end;
      end
    else if LeftResolved.BaseType=btContext then
      begin
      LeftTypeEl:=LeftResolved.LoTypeEl;
      if LeftTypeEl.ClassType=TPasRecordType then
        begin
        // LHS is a record
        if RightResolved.BaseType=btContext then
          begin
          RightTypeEl:=RightResolved.LoTypeEl;
          if RightTypeEl.ClassType=TPasRecordType then
            begin
            // convert "recordA = recordB" to "recordA.$eq(recordB)"
            Call:=CreateCallExpression(El);
            Call.Expr:=CreateDotNameExpr(El,A,TJSString(GetBIName(pbifnRecordEqual)));
            A:=nil;
            Call.AddArg(B);
            B:=nil;
            if El.OpCode=eopNotEqual then
              begin
              // convert "recordA <> recordB" to "!recordA.$eq(recordB)"
              Result:=CreateUnaryNot(Call,El);
              end
            else
              Result:=Call;
            exit;
            end
          else if (RightTypeEl.ClassType=TPasClassType)
              and (TPasClassType(RightTypeEl).ObjKind=okInterface)
              and aResolver.IsTGUID(TPasRecordType(LeftTypeEl)) then
            begin
            // "GuidVar = intfTypeOrVar"  ->  "GuidVar.$eq(rtl.getIntfGUIDR(intfTypeOrVar))"
            Call:=CreateCallExpression(El);
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGetGUIDR),El);
            Call.AddArg(B);
            B:=Call;
            Call:=CreateCallExpression(El);
            Call.Expr:=CreateDotNameExpr(El,A,TJSString(GetBIName(pbifnRecordEqual)));
            A:=nil;
            Call.AddArg(B);
            B:=nil;
            if El.OpCode=eopNotEqual then
              Result:=CreateUnaryNot(Call,El)
            else
              Result:=Call;
            exit;
            end;
          end
        else if (RightResolved.BaseType in btAllStrings)
            and aResolver.IsTGUID(TPasRecordType(LeftTypeEl)) then
          begin
          // "GuidVar = aString"  ->  "GuidVar.$eq(rtl.createTGUID(aString))"
          Call:=CreateCallExpression(El);
          Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfCreateTGUID),El);
          Call.AddArg(B);
          B:=Call;
          Call:=CreateCallExpression(El);
          Call.Expr:=CreateDotNameExpr(El,A,TJSString(GetBIName(pbifnRecordEqual)));
          A:=nil;
          Call.AddArg(B);
          B:=nil;
          if El.OpCode=eopNotEqual then
            Result:=CreateUnaryNot(Call,El)
          else
            Result:=Call;
          exit;
          end;
        end
      else if LeftTypeEl.ClassType=TPasClassType then
        begin
        if RightResolved.BaseType in btAllStrings then
          begin
          if (TPasClassType(LeftTypeEl).ObjKind=okInterface) then
            begin
            // "IntfTypeOrVar=aString"  ->  "IntfTypeOrVar.$guid === aString"
            A:=CreateDotNameExpr(El.left,A,TJSString(GetBIName(pbivnIntfGUID)));
            end;
          end
        else if RightResolved.BaseType=btContext then
          begin
          RightTypeEl:=RightResolved.LoTypeEl;
          if RightTypeEl.ClassType=TPasRecordType then
            begin
            if (TPasClassType(LeftTypeEl).ObjKind=okInterface)
                and aResolver.IsTGUID(TPasRecordType(RightTypeEl)) then
              begin
              // "IntfTypeOrVar=GuidVar"  ->  "GuidVar.$eq(rtl.getIntfGUIDR(intfTypeOrVar))"
              Call:=CreateCallExpression(El);
              Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGetGUIDR),El);
              Call.AddArg(A);
              A:=Call;
              Call:=CreateCallExpression(El);
              Call.Expr:=CreateDotNameExpr(El,B,TJSString(GetBIName(pbifnRecordEqual)));
              B:=nil;
              Call.AddArg(A);
              A:=nil;
              if El.OpCode=eopNotEqual then
                Result:=CreateUnaryNot(Call,El)
              else
                Result:=Call;
              exit;
              end;
            end;
          end;
        end
      else if LeftTypeEl.ClassType=TPasArrayType then
        begin
        LArrType:=TPasArrayType(LeftTypeEl);
        if RightResolved.BaseType=btNil then
          begin
          // convert "array = nil" to "rtl.length(array) === 0"
          FreeAndNil(B);
          Result:=CreateCmpArrayWithNil(El,A,El.OpCode);
          A:=nil;
          exit;
          end
        else if length(LArrType.Ranges)>0 then
          begin
          // LHS is static array
          aResolver.RaiseMsg(20200508102656,nXIsNotSupported,sXIsNotSupported,['compare static array'],TPasElement(El));
          end;
        end;
      end;
    if aResolver.IsJSBaseType(LeftResolved,pbtJSValue)
        or aResolver.IsJSBaseType(RightResolved,pbtJSValue) then
      begin
        // convert "jsvalue = something" to "jsvalue == something" (not strict)
        // Note: default "=" is converted to "===" (strict equal)
        if El.OpCode=eopEqual then
          Result:=TJSEqualityExpressionEQ(CreateElement(TJSEqualityExpressionEQ,El))
        else
          Result:=TJSEqualityExpressionNE(CreateElement(TJSEqualityExpressionNE,El));
        TJSBinaryExpression(Result).A:=A; A:=nil;
        TJSBinaryExpression(Result).B:=B; B:=nil;
        exit;
      end;
    end;
end;

function TPasToJSConverter.ConvertBinaryExpressionMultiAdd(El: TBinaryExpr;
  AContext: TConvertContext): TJSElement;
// handle multi add without stack
// Note: The parser generates a list of TBinaryExpr.Lefts
var
  aResolver: TPas2JSResolver;
  Left: TPasExpr;
  SubBin: TBinaryExpr;
  A, B: TJSElement;
  LeftResolved, RightResolved, ResultResolved: TPasResolverResult;
  Flags: TPasResolverComputeFlags;
  R: TJSBinary;
  OldAccess: TCtxAccess;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  Left:=El;
  while Left.ClassType=TBinaryExpr do
    begin
    SubBin:=TBinaryExpr(Left);
    if SubBin.OpCode<>eopAdd then break;
    Left:=SubBin.left;
    if Left.Parent<>SubBin then
      begin
      if aResolver<>nil then
        RaiseNotSupported(SubBin,AContext,20210321220458)
      else if Left.Parent=nil then
        Left.Parent:=SubBin
      else
        RaiseNotSupported(SubBin,AContext,20210321221135);
      end;
    end;
  if Left=El then
    RaiseNotSupported(El,AContext,20210321221047);
  OldAccess:=AContext.Access;
  AContext.Access:=caRead;
  A:=nil;
  B:=nil;
  try
    A:=ConvertExpression(Left,AContext);
    Flags:=[];
    if aResolver<>nil then
      aResolver.ComputeElement(Left,LeftResolved,Flags);
    repeat
      SubBin:=TBinaryExpr(Left.Parent);
      B:=ConvertExpression(SubBin.right,AContext);
      if aResolver<>nil then
        begin
        aResolver.ComputeElement(SubBin.right,RightResolved,Flags);
        Result:=ConvertBinaryExpressionRes(SubBin,AContext,LeftResolved,RightResolved,A,B);
        if (Result<>nil) then
          begin
          A:=nil;
          B:=nil;
          if SubBin=El then exit;
          end;
        aResolver.ComputeBinaryExprRes(SubBin,ResultResolved,Flags,LeftResolved,RightResolved);
        end;
      if Result=nil then
        begin
        // +
        R:=TJSBinary(CreateElement(TJSAdditiveExpressionPlus,SubBin));
        R.A:=A; A:=nil;
        R.B:=B; B:=nil;
        Result:=R;

        if (bsOverflowChecks in AContext.ScannerBoolSwitches) and (aResolver<>nil) then
          case El.OpCode of
          eopAdd,eopSubtract:
            if (LeftResolved.BaseType in btAllJSOverflowAddSubType)
                or (RightResolved.BaseType in btAllJSOverflowAddSubType) then
              Result:=CreateOverflowCheckCall(Result,SubBin);
          eopMultiply:
            if (LeftResolved.BaseType in btAllJSOverflowMultType)
                or (RightResolved.BaseType in btAllJSOverflowMultType) then
              Result:=CreateOverflowCheckCall(Result,SubBin);
          end;

        if SubBin=El then exit;
        end;
      // next
      A:=Result;
      Result:=nil;
      if aResolver<>nil then
        LeftResolved:=ResultResolved;
      Left:=SubBin;
    until false;
  finally
    AContext.Access:=OldAccess;
    if Result=nil then
      begin
      A.Free;
      B.Free;
      end;
  end;
end;

function TPasToJSConverter.ConvertSubIdentExpression(El: TBinaryExpr;
  AContext: TConvertContext): TJSElement;
// connect El.left and El.right with a dot.
var
  RightRef: TResolvedReference;
  RightEl: TPasExpr;
  RightRefDecl: TPasElement;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;

  // Note: TPasParser guarantees that there is at most one TBinaryExpr
  //       and/or one TInlineSpecializeExpr between
  //       TParamsExpr and its NameExpr. E.g. a.b.c() = ((a.b).c)()

  RightEl:=El.right;
  if RightEl is TInlineSpecializeExpr then
    RightEl:=TInlineSpecializeExpr(RightEl).NameExpr;
  if (RightEl.ClassType<>TPrimitiveExpr) then
    RaiseNotSupported(RightEl,AContext,20190131162250,'Left='+GetObjName(El.left)+' right='+GetObjName(RightEl));
  if not (RightEl.CustomData is TResolvedReference) then
    RaiseNotSupported(RightEl,AContext,20190131162301);

  RightRef:=TResolvedReference(RightEl.CustomData);
  RightRefDecl:=RightRef.Declaration;
  if aResolver.IsTObjectFreeMethod(RightEl) then
    begin
    // e.g. Obj.Free;
    Result:=ConvertTObjectFree_Bin(El,RightEl,AContext);
    exit;
    end
  else if aResolver.IsExternalClassConstructor(RightRefDecl) then
    begin
    // e.g. mod.ExtClass.new;
    if (El.Parent is TParamsExpr) and (TParamsExpr(El.Parent).Value=El) then
      // Note: ExtClass.new() is handled in ConvertFuncParams
      RaiseNotSupported(El,AContext,20190116135818);
    Result:=ConvertExternalConstructor(El.left,RightRef,nil,AContext);
    exit;
    end;

  Result:=ConvertSubIdentExprCustom(El,AContext);
end;

function TPasToJSConverter.ConvertSubIdentExprCustom(El: TBinaryExpr;
  AContext: TConvertContext; const OnConvertRight: TConvertJSEvent;
  Data: Pointer): TJSElement;
var
  OldAccess: TCtxAccess;
  LeftJS, RightJS: TJSElement;
  DotContext: TDotContext;
  aResolver: TPas2JSResolver;
  LeftResolved: TPasResolverResult;
  RightEl: TPasExpr;
  RightRef: TResolvedReference;
  RightRefDecl: TPasElement;
  Proc: TPasProcedure;
begin
  aResolver:=AContext.Resolver;

  // Note: TPasParser guarantees that there is at most one TBinaryExpr
  //       and/or one TInlineSpecializeExpr between
  //       TParamsExpr and its NameExpr. E.g. a.b.c() = ((a.b).c)()

  RightEl:=El.right;
  if RightEl is TInlineSpecializeExpr then
    RightEl:=TInlineSpecializeExpr(RightEl).NameExpr;
  if (RightEl.ClassType<>TPrimitiveExpr) then
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertSubIdentExprCustom Bin=',El.OpCode,' El.Right=',GetObjName(RightEl));
    {$ENDIF}
    RaiseNotSupported(RightEl,AContext,20190131164529);
    end;
  if not (RightEl.CustomData is TResolvedReference) then
    RaiseNotSupported(RightEl,AContext,20190131164530);

  RightRef:=TResolvedReference(RightEl.CustomData);
  RightRefDecl:=RightRef.Declaration;
  if RightRefDecl.ClassType=TPasProperty then
    begin
    // redirect to Getter/Setter
    case AContext.Access of
    caAssign:
      begin
      RightRefDecl:=aResolver.GetPasPropertySetter(TPasProperty(RightRefDecl));
      if RightRefDecl=nil then
        DoError(20190211111137,nNoMemberIsProvidedToAccessProperty,sNoMemberIsProvidedToAccessProperty,[],RightEl);
      end;
    caRead:
      begin
      RightRefDecl:=aResolver.GetPasPropertyGetter(TPasProperty(RightRefDecl));
      if RightRefDecl=nil then
        DoError(20190211111038,nNoMemberIsProvidedToAccessProperty,sNoMemberIsProvidedToAccessProperty,[],RightEl);
      end;
    end;
    end
  else if RightRefDecl.ClassType=TPasEnumValue then
    begin
    // enum value
    Result:=ConvertIdentifierExpr(RightEl,'',aContext);
    exit;
    end;
  if (AContext.Access=caAssign)
      and aResolver.IsClassField(RightRefDecl) then
    begin
    // e.g. "Something.aClassVar:=" -> "aClass.aClassVar:="
    LeftJS:=CreateReferencePathExpr(RightRefDecl.Parent,AContext);
    Result:=CreateDotNameExpr(El,LeftJS,TJSString(TransformElToJSName(RightRefDecl,AContext)));
    exit;
    end;
  if RightRefDecl is TPasProcedure then
    begin
    Proc:=TPasProcedure(RightRefDecl);
    if not aResolver.ProcHasSelf(Proc) then
      begin
      // a.StaticProc  ->  pas.unit1.aclass.StaticProc(defaultargs)
      // ToDo: check if left side has only types (no call nor field)
      Result:=ConvertIdentifierExpr(RightEl,TPrimitiveExpr(RightEl).Value,aContext);
      exit;
      end;
    end;

  LeftJS:=nil;
  if aResolver.IsHelper(RightRefDecl.Parent) then
    begin
    // LeftJS.HelperMember
    if (RightRefDecl is TPasVariable)
        and not (vmExternal in TPasVariable(RightRefDecl).VarModifiers) then
      begin
      // LeftJS.HelperField  -> HelperType.HelperField
      if Assigned(OnConvertRight) then
        Result:=OnConvertRight(RightEl,AContext,Data)
      else
        Result:=ConvertIdentifierExpr(RightEl,TPrimitiveExpr(RightEl).Value,AContext);
      exit;
      end
    else if RightRefDecl is TPasProcedure then
      begin
      Proc:=TPasProcedure(RightRefDecl);
      if Proc.IsExternal then
        // normal call
      else if rrfNoImplicitCallWithoutParams in RightRef.Flags then
        begin
        Result:=CreateReferencePathExpr(RightRefDecl,AContext);
        exit;
        end
      else
        begin
        // call helper method
        Result:=CreateCallHelperMethod(Proc,El,AContext);
        exit;
        end;
      end
    else
      RaiseNotSupported(El,AContext,20190131170119,GetObjName(RightRefDecl));
    end;

  if LeftJS=nil then
    begin
    // check Left

    if aResolver<>nil then
      aResolver.ComputeElement(El.left,LeftResolved,[])
    else
      LeftResolved:=Default(TPasResolverResult);
    if LeftResolved.BaseType=btModule then
      begin
      // e.g. system.inttostr()
      // module path is created automatically
      if Assigned(OnConvertRight) then
        Result:=OnConvertRight(RightEl,AContext,Data)
      else
        Result:=ConvertIdentifierExpr(RightEl,TPrimitiveExpr(RightEl).Value,AContext);
      exit;
      end;

    // convert LeftJS side
    OldAccess:=AContext.Access;
    AContext.Access:=caRead;
    LeftJS:=ConvertExpression(El.left,AContext);
    if LeftJS=nil then
      RaiseNotSupported(El,AContext,20190116110446);
    AContext.Access:=OldAccess;
    end;

  // convert RightJS side
  DotContext:=TDotContext.Create(El,LeftJS,AContext);
  RightJS:=nil;
  try
    DotContext.LeftResolved:=LeftResolved;
    if Assigned(OnConvertRight) then
      RightJS:=OnConvertRight(RightEl,DotContext,Data)
    else
      RightJS:=ConvertIdentifierExpr(RightEl,TPrimitiveExpr(RightEl).Value,DotContext);
    if DotContext.JS<>nil then
      begin
      LeftJS:=nil;
      RightJS:=nil;
      exit(DotContext.JS);
      end;
  finally
    if (RightJS=nil) and (DotContext.JSElement=LeftJS) then
      LeftJS.Free;
    DotContext.Free;
  end;
  if RightJS is TJSLiteral then
    begin
    LeftJS.Free;
    exit(RightJS);
    end;
  // connect via dot
  Result:=CreateDotExpression(El,LeftJS,RightJS,true);
end;

function TPasToJSConverter.CreateIdentifierExpr(El: TPasElement;
  AContext: TConvertContext): TJSElement;
begin
  Result:=CreatePrimitiveDotExpr(TransformElToJSName(El,AContext),El);
end;

function TPasToJSConverter.CreateIdentifierExpr(AName: string;
  CheckGlobal: boolean; PosEl: TPasElement; AContext: TConvertContext
  ): TJSElement;
// CheckGlobal: check name clashes with global identifiers too
begin
  Result:=CreatePrimitiveDotExpr(TransformToJSName(PosEl,AName,CheckGlobal,AContext),PosEl);
end;

function TPasToJSConverter.CreateSubDeclJSNameExpr(El: TPasElement;
  JSName: string; AContext: TConvertContext; PosEl: TPasElement): TJSElement;
var
  C: TClass;
  VarKinds: TCtxVarKinds;
  ParentName: String;
begin
  C:=El.ClassType;
  if C.InheritsFrom(TPasType) or (C=TPasConst) then
    VarKinds:=[cvkGlobal]
  else if C.InheritsFrom(TPasVariable) then
    begin
    VarKinds:=[cvkCurType];
    if ([vmClass, vmStatic]*TPasVariable(El).VarModifiers<>[]) then
      VarKinds:=[cvkGlobal]
    else if El.Parent is TPasMembersType then
      VarKinds:=[cvkCurType]
    else
      VarKinds:=[cvkGlobal];
    end
  else if (El.Parent is TProcedureBody) then
    VarKinds:=[]
  else
    VarKinds:=[cvkGlobal];
  if VarKinds<>[] then
    begin
    ParentName:=GetLocalName(El.Parent,VarKinds,AContext);
    if ParentName='' then
      ParentName:='this';
    if JSName[1]='[' then
      JSName:=ParentName+JSName
    else
      JSName:=ParentName+'.'+JSName;
    end;
  Result:=CreatePrimitiveDotExpr(JSName,PosEl);
end;

function TPasToJSConverter.CreateSubDeclPasNameExpr(El: TPasElement;
  const PasName: string; AContext: TConvertContext; PosEl: TPasElement
  ): TJSElement;
var
  JSName: String;
begin
  JSName:=TransformToJSName(El,PasName,false,AContext);
  Result:=CreateSubDeclJSNameExpr(El,JSName,AContext,PosEl);
end;

function TPasToJSConverter.CreateSubDeclNameExpr(El: TPasElement;
  AContext: TConvertContext; PosEl: TPasElement): TJSElement;
var
  JSName: String;
begin
  JSName:=TransformElToJSName(El,AContext);
  Result:=CreateSubDeclJSNameExpr(El,JSName,AContext,PosEl);
end;

function TPasToJSConverter.ConvertPrimitiveExpression(El: TPrimitiveExpr;
  AContext: TConvertContext): TJSElement;

  function DeleteLeadingZeroes(const s: string): string;
  // Note: 01 is in JS octal, and in strict mode forbidden
  // $00ff00 -> $ff00
  // 00E001 -> 0E1
  // 0.001 -> 0.001
  // 0.00E1 -> 0.00E1
  var
    i: Integer;
  begin
    Result:=s;
    i:=1;
    if Result[1]='$' then
      // hexadecimal -> can not be a float, 'E' is a hexdigit
      while i<length(Result) do
        begin
        if (Result[i]='0') and (Result[i+1] in ['0'..'9','A'..'F','a'..'f'])
            and ((i=1) or not (Result[i-1] in ['0'..'9','A'..'F','a'..'f'])) then
          Delete(Result,i,1)
        else
          inc(i);
        end
    else
      // decimal, can be a float, 'E' is a start of a new number
      while i<length(Result) do
        begin
        if (Result[i]='0') and (Result[i+1] in ['0'..'9'])
            and ((i=1) or not (Result[i-1] in ['.','0'..'9'])) then
          Delete(Result,i,1)
        else
          inc(i);
        end;
  end;

Var
  L : TJSLiteral;
  Number : TJSNumber;
  ConversionError , Code: Integer;
  i: TMaxPrecInt;
  S: String;
begin
  {$IFDEF VerbosePas2JS}
  str(El.Kind,S);
  writeln('TPasToJSConverter.ConvertPrimitiveExpression El=',GetObjName(El),' Context=',GetObjName(AContext),' El.Kind=',S);
  {$ENDIF}
  Result:=Nil;
  case El.Kind of
    pekString:
      begin
      if AContext.Resolver<>nil then
        Result:=CreateLiteralJSString(El,
          AContext.Resolver.ExtractPasStringLiteral(El,El.Value))
      else
        begin
        S:={$IFDEF pas2js}DeQuoteString{$ELSE}AnsiDequotedStr{$ENDIF}(El.Value,'''');
        Result:=CreateLiteralString(El,S);
        end;
      //writeln('TPasToJSConverter.ConvertPrimitiveExpression Result="',TJSLiteral(Result).Value.AsString,'" ',GetObjName(AContext.Resolver));
      end;
    pekNumber:
      begin
      case El.Value[1] of
      '0'..'9':
        begin
        Val(El.Value,Number,ConversionError);
        if ConversionError<>0 then
          DoError(20161024191248,nInvalidNumber,sInvalidNumber,[El.Value],El);
        L:=CreateLiteralNumber(El,Number);
        L.Value.CustomValue:=TJSString(DeleteLeadingZeroes(El.Value));
        end;
      '$','&','%':
        begin
          val(El.Value,i,Code);
          if Code<>0 then
            DoError(20161024224442,nInvalidNumber,sInvalidNumber,[El.Value],El);
          Number:=i;
          if Number<>i then
            // number was rounded -> we lost precision
            DoError(20161024230812,nInvalidNumber,sInvalidNumber,[El.Value],El);
          L:=CreateLiteralNumber(El,Number);
          S:=DeleteLeadingZeroes(El.Value);
          S:=copy(S,2,length(S));
          case El.Value[1] of
          '$': S:='0x'+S;
          '&': if FGlobals.TargetProcessor=ProcessorECMAScript5 then
                 S:='' // in strict mode 01 is forbidden
               else
                 S:='0o'+S;
          '%': if FGlobals.TargetProcessor=ProcessorECMAScript5 then
                 S:='' // use decimal
               else
                 S:='0b'+S;
          end;
          L.Value.CustomValue:=TJSString(S);
        end;
      else
        DoError(20161024223232,nInvalidNumber,sInvalidNumber,[El.Value],El);
      end;
      Result:=L;
      end;
    pekIdent:
      Result:=ConvertIdentifierExpr(El,El.Value,AContext);
    else
      RaiseNotSupported(El,AContext,20161024222543);
  end;
end;

function TPasToJSConverter.ConvertIdentifierExpr(El: TPasExpr;
  const aName: string; AContext: TConvertContext): TJSElement;
var
  AssignContext: TAssignContext;
  ApplyParam: TJSElement;

  procedure CallImplicit(Decl: TPasElement);
  var
    ProcType: TPasProcedureType;
    ResolvedEl: TPasResolverResult;
    Call: TJSCallExpression;
    NeedIntfRef: Boolean;
  begin
    // create a call with default parameters
    ProcType:=nil;
    if Decl is TPasProcedure then
      ProcType:=TPasProcedure(Decl).ProcType
    else
      begin
      AContext.Resolver.ComputeElement(El,ResolvedEl,[rcNoImplicitProc]);
      if ResolvedEl.LoTypeEl is TPasProcedureType then
        ProcType:=TPasProcedureType(ResolvedEl.LoTypeEl)
      else
        RaiseNotSupported(El,AContext,20170217005025);
      end;

    NeedIntfRef:=false;
    if (ProcType is TPasFunctionType)
        and not ProcType.IsAsync
        and AContext.Resolver.IsInterfaceType(
          TPasFunctionType(ProcType).ResultEl.ResultType,citCom)
    then
      NeedIntfRef:=true;

    Call:=nil;
    try
      CreateProcedureCall(Call,nil,ProcType,AContext);
      if ApplyParam<>nil then
        begin
        if Call.Args=nil then
          Call.Args:=TJSArguments(CreateElement(TJSArguments,ProcType));
        Call.InsertArg(0,ApplyParam);
        ApplyParam:=nil;
        if AContext is TDotContext then
          TDotContext(AContext).JS:=Call;
        end;
      Call.Expr:=Result;
      if NeedIntfRef then
        // $ir.ref(id,fnname())
        Call:=CreateIntfRef(Call,AContext,El);
      Result:=Call;
    finally
      if Result<>Call then
        begin
        Call.Free;
        ApplyParam.Free;
        end;
    end;
  end;

  function CreateShortRefImplictCall_Apply(TargetProc: TPasProcedure;
    Ref: TResolvedReference): string;
  var
    ApplyPath: String;
  begin
    // ProcName; -> "$lp.apply(this,args);"  or  "$lp.apply($with,args);"
    Result:=CreateStaticProcPath(TargetProc,AContext)+'.apply';

    ApplyPath:=CreateReferencePath(TargetProc,AContext,rpkPath,false,Ref);
    if AContext is TDotContext then
      begin
      ApplyParam:=AContext.JSElement;
      AContext.JSElement:=nil;
      if ApplyPath<>'' then
        // e.g. "$class"
        ApplyParam:=CreateDotNameExpr(El,ApplyParam,TJSString(ApplyPath));
      end
    else
      begin
      if ApplyPath='' then
        RaiseNotSupported(El,AContext,20201101022637);
      ApplyParam:=CreatePrimitiveDotExpr(ApplyPath,El);
      end;
    if ApplyParam=nil then
      RaiseNotSupported(El,AContext,20201101021136);
  end;

  procedure CallTypeSetter;
  var
    Call: TJSCallExpression;
  begin
    if AssignContext<>nil then
      begin
      if AssignContext.LeftResolved.LoTypeEl is TPasRecordType then
        begin
        // aRecord:=right  ->  aRecord.$assign(right)
        Call:=CreateCallExpression(El);
        AssignContext.Call:=Call;
        Call.Expr:=CreateDotNameExpr(El,Result,TJSString(GetBIName(pbifnRecordAssign)));
        Call.AddArg(AssignContext.RightSide);
        AssignContext.RightSide:=nil;
        Result:=Call;
        end;
      end;
  end;

var
  Decl: TPasElement;
  Name: String;
  Ref: TResolvedReference;
  Call: TJSCallExpression;
  BuiltInProc: TResElDataBuiltInProc;
  Prop: TPasProperty;
  IsImplicitCall: Boolean;
  TargetProcType: TPasProcedureType;
  ArrLit: TJSArrayLiteral;
  FuncScope: TPas2JSProcedureScope;
  Value: TResEvalValue;
  aResolver: TPas2JSResolver;
  BracketExpr: TJSBracketMemberExpression;
  PathExpr: TJSElement;
  Proc: TPasProcedure;
begin
  Result:=nil;
  if not (El.CustomData is TResolvedReference) then
    begin
    if AContext.Resolver<>nil then
      RaiseIdentifierNotFound(aName,El,20161024191306)
    else
      // simple mode
      Result:=CreateIdentifierExpr(aName,true,El,AContext);
    exit;
    end;

  aResolver:=AContext.Resolver;
  Ref:=TResolvedReference(El.CustomData);
  Decl:=Ref.Declaration;

  if aResolver.IsExternalClassConstructor(Decl) then
    begin
    // create external object/function
    Result:=ConvertExternalConstructor(nil,Ref,nil,AContext);
    exit;
    end;
  if aResolver.IsExternalBracketAccessor(Decl) then
    DoError(20180511154132,nCantCallExtBracketAccessor,sCantCallExtBracketAccessor,[],El);

  if [rrfNewInstance,rrfFreeInstance]*Ref.Flags<>[] then
    begin
    Call:=CreateFreeOrNewInstanceExpr(Ref,AContext);
    Result:=Call;
    TargetProcType:=TPasProcedure(Decl).ProcType;
    if TargetProcType.Args.Count>0 then
      begin
      // add default parameters:
      if Decl.Parent.ClassType=TPasRecordType then
        // insert default parameters, e.g. TRecord.$new().create(1,2,3)
        CreateProcedureCallArgs(Call.Args.Elements,nil,TargetProcType,AContext)
      else
        begin
        // insert array parameter [], e.g. TObject.$create("create",[])
        ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
        CreateProcedureCallArgs(ArrLit.Elements,nil,TargetProcType,AContext);
        Call.AddArg(ArrLit);
        end;
      end;
    exit;
    end;

  if (Ref.WithExprScope<>nil) and aResolver.IsTObjectFreeMethod(El) then
    begin
    Result:=ConvertTObjectFree_With(El,AContext);
    exit;
    end;

  Prop:=nil;
  AssignContext:=nil;
  ApplyParam:=nil;
  IsImplicitCall:=rrfImplicitCallWithoutParams in Ref.Flags;
  if AContext.Access=caAssign then
    AssignContext:=AContext.AccessContext as TAssignContext;

  if Decl.ClassType=TPasArgument then
    begin
    Result:=CreateArgumentAccess(TPasArgument(Decl),AContext,El);
    if IsImplicitCall then
      CallImplicit(Decl);
    exit;
    end;

  if Decl.ClassType=TPasProperty then
    begin
    // Decl is a property -> redirect to getter/setter
    Prop:=TPasProperty(Decl);
    case AContext.Access of
      caAssign:
        begin
        if AssignContext.Call<>nil then
          RaiseNotSupported(El,AContext,20170206000310);
        Decl:=aResolver.GetPasPropertySetter(Prop);
        if Decl is TPasProcedure then
          begin
          if aResolver.IsHelperMethod(Decl) then
            begin
            Result:=CreateCallHelperMethod(TPasProcedure(Decl),El,AContext);
            exit;
            end;
          // Setter
          Call:=CreateCallExpression(El);
          Call.Expr:=CreateReferencePathExpr(Decl,AContext,false,Ref);
          Result:=AppendPropertyAssignArgs(Call,Prop,AssignContext,El);
          exit;
          end;
        end;
      caRead:
        begin
        Result:=CreatePropertyGet(Prop,El,AContext,El);
        if Result is TJSCallExpression then exit;
        if not IsImplicitCall then exit;
        end;
      else
        RaiseNotSupported(El,AContext,20170213212623);
    end;
    end; // property redirect

  if aResolver.IsClassField(Decl)
      and (AContext.Access in [caAssign,caByReference]) then
    begin
    // writing a class var  -> aClass.VarName
    PathExpr:=CreateReferencePathExpr(Decl.Parent,AContext);
    Result:=CreateDotNameExpr(El,PathExpr,TJSString(TransformElToJSName(Decl,AContext)));
    CallTypeSetter;
    exit;
    end
  else if Decl.ClassType=TPasConst then
    begin
    if TPasConst(Decl).IsConst and (TPasConst(Decl).Expr<>nil) then
      begin
      // const with expression
      Value:=aResolver.Eval(TPasConst(Decl).Expr,[refConst]);
      if Value<>nil then
        try
          if Value.Kind in [revkNil,revkBool,revkInt,revkUInt,revkFloat,revkEnum] then
            begin
            Result:=ConvertConstValue(Value,AContext,El);
            exit;
            end;
        finally
          ReleaseEvalValue(Value);
        end;
      if vmExternal in TPasConst(Decl).VarModifiers then
        begin
        // external constant with expression is always added by value, not by reference
        Result:=ConvertExpression(TPasConst(Decl).Expr,AContext);
        CallTypeSetter;
        exit;
        end;
      end;
    end
  else if Decl.ClassType=TPasResString then
    begin
    // read resourcestring -> rtl.getResStr(pas.modulename,"name")
    Call:=CreateCallExpression(El);
    Result:=Call;
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnGetResourcestring)]);
    Call.AddArg(CreatePrimitiveDotExpr(TransformModuleName(Decl.GetModule,true,AContext),El));
    Call.AddArg(CreateLiteralString(El,TransformElToJSName(Decl,AContext)));
    exit;
    end
  else if aResolver.IsHelperMethod(Decl)
      and not (rrfNoImplicitCallWithoutParams in Ref.Flags) then
    begin
    Result:=CreateCallHelperMethod(TPasProcedure(Decl),El,AContext);
    exit;
    end
  else if Decl.CustomData is TResElDataBuiltInProc then
    begin
    BuiltInProc:=TResElDataBuiltInProc(Decl.CustomData);
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertIdentifierExpr ',Decl.Name,' ',ResolverBuiltInProcNames[BuiltInProc.BuiltIn]);
    {$ENDIF}
    case BuiltInProc.BuiltIn of
      bfBreak: Result:=ConvertBuiltInBreak(El,AContext);
      bfContinue: Result:=ConvertBuiltInContinue(El,AContext);
      bfExit: Result:=ConvertBuiltIn_Exit(El,AContext);
      bfCustom:
        case BuiltInProc.Element.Name of
        'Debugger': Result:=ConvertBuiltIn_Debugger(El,AContext);
        else
          RaiseNotSupported(El,AContext,20181126102554,'built in custom proc '+BuiltInProc.Element.Name);
        end
    else
      RaiseNotSupported(El,AContext,20161130164955,'built in proc '+ResolverBuiltInProcNames[BuiltInProc.BuiltIn]);
    end;
    if Result=nil then
      RaiseInconsistency(20170214120048,Decl);
    exit;
    end;

  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertIdentifierExpr ',GetObjName(El),' Decl=',GetObjName(Decl),' Decl.Parent=',GetObjName(Decl.Parent));
  //if CompareText(aName,'Self')=0 then
  //  begin
  //  writeln('TPasToJSConverter.ConvertIdentifierExpr AContext=',GetObjName(AContext),' SelfContext=',GetObjName(AContext.GetSelfContext),' LocalVar=',AContext.GetLocalName(Decl),' ',GetObjName(Decl));
  //  AContext.WriteStack;
  //  end;
  {$ENDIF}

  try
    if Decl is TPasModule then
      Name:=TransformModuleName(TPasModule(Decl),true,AContext)
    else if (Decl is TPasResultElement) then
      begin
      Name:=ResolverResultVar;
      Proc:=Decl.Parent.Parent as TPasProcedure;
      FuncScope:=Proc.CustomData as TPas2JSProcedureScope;
      if FuncScope.ImplProc<>nil then
        FuncScope:=FuncScope.ImplProc.CustomData as TPas2JSProcedureScope;
      if FuncScope.ResultVarName<>'' then
        Name:=FuncScope.ResultVarName;
      end
    else if Decl.ClassType=TPasEnumValue then
      begin
      if UseEnumNumbers then
        begin
        Result:=CreateLiteralNumber(El,(Decl.Parent as TPasEnumType).Values.IndexOf(Decl));
        exit;
        end
      else
        begin
        // enums always need the full path
        Name:=CreateReferencePath(Decl,AContext,rpkPathAndName,true);
        end;
      end
    else if Decl.ClassType=TPasArgument then
      Name:=TransformArgName(TPasArgument(Decl),AContext)
    else if Decl is TPasProcedure then
      begin
      Proc:=TPasProcedure(Decl);
      if (coShortRefGlobals in Options)
          and aResolver.IsSpecializedNonStaticMethod(Proc.ProcType) then
        Name:=CreateShortRefImplictCall_Apply(Proc,Ref)
      else
        Name:=CreateReferencePath(Decl,AContext,rpkPathAndName,false,Ref);
      end
    else
      Name:=CreateReferencePath(Decl,AContext,rpkPathAndName,false,Ref);
    if Name='' then
      RaiseNotSupported(El,AContext,20180509134804,GetObjName(Decl));

    if Result=nil then
      begin
      if (Name[1]='[') and (Name[length(Name)]=']')
          and (AContext is TDotContext)
          and (AContext.JSElement<>nil) then
        begin
        // e.g. Obj.A  with A having an external name '["name"]';
        // -> Obj["name"]
        if IsImplicitCall then
          RaiseNotSupported(El,AContext,20180509134951,Name);
        BracketExpr:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
        TDotContext(AContext).JS:=BracketExpr;
        BracketExpr.MExpr:=AContext.JSElement;
        Result:=CreateLiteralCustomValue(El,TJSString(copy(Name,2,length(Name)-2)));
        BracketExpr.Name:=Result;
        exit;
        end;
      Result:=CreatePrimitiveDotExpr(Name,El);
      end;

    if IsImplicitCall then
      CallImplicit(Decl);
    CallTypeSetter;
  finally
    ApplyParam.Free;
  end;
end;

function TPasToJSConverter.ConvertBoolConstExpression(El: TBoolConstExpr;
  AContext: TConvertContext): TJSElement;

begin
  if AContext=nil then ;
  Result:=CreateLiteralBoolean(El,El.Value);
end;

function TPasToJSConverter.ConvertNilExpr(El: TNilExpr;
  AContext: TConvertContext): TJSElement;
begin
  if AContext=nil then ;
  Result:=CreateLiteralNull(El);
end;

function TPasToJSConverter.ConvertCharToInt(Arg: TJSElement;
  PosEl: TPasElement; ArgContext: TConvertContext): TJSElement;
begin
  if (Arg is TJSLiteral) and (TJSLiteral(Arg).Value.ValueType=jstString) then
    begin
    // convert char literal to int
    ConvertCharLiteralToInt(TJSLiteral(Arg),PosEl,ArgContext);
    Result:=Arg;
    end
  else
    begin
    // convert char to int  ->  Arg.charCodeAt(0)
    Result:=CreateCallCharCodeAt(Arg,0,PosEl);
    end;
end;

function TPasToJSConverter.ConvertIntToInt(Arg: TJSElement; FromBT,
  ToBT: TResolverBaseType; PosEl: TPasElement; ArgContext: TConvertContext
  ): TJSElement;
var
  aResolver: TPas2JSResolver;
  MinVal, MaxVal: TMaxPrecInt;
  Call: TJSCallExpression;
  ShiftEx: TJSURShiftExpression;
begin
  Result:=Arg;
  aResolver:=ArgContext.Resolver;
  if FromBT=btCurrency then
    begin
    if ToBT<>btCurrency then
      // currency to integer -> rtl.trunc(value/10000)
      Result:=CreateTruncFloor(PosEl,CreateDivideNumber(PosEl,Result,10000),true);
    end
  else if ToBT=btCurrency then
    // integer to currency -> value*10000
    Result:=CreateMulNumber(PosEl,Result,10000);
  if (ToBT<>btIntDouble) and not (Result is TJSLiteral) then
    begin
    if bsRangeChecks in ArgContext.ScannerBoolSwitches then
      begin
      // rtl.rc(param,MinInt,MaxInt)
      if not aResolver.GetIntegerRange(ToBT,MinVal,MaxVal) then
        RaiseNotSupported(PosEl,ArgContext,20180425131839);
      Call:=CreateCallExpression(PosEl);
      Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnRangeCheckInt),PosEl);
      Call.AddArg(Result);
      Result:=Call;
      Call.AddArg(CreateLiteralNumber(PosEl,MinVal));
      Call.AddArg(CreateLiteralNumber(PosEl,MaxVal));
      end
    else
      case ToBT of
      btByte:
        // value to byte  ->  value & 255
        if FromBT<>btByte then
          Result:=CreateBitWiseAnd(PosEl,Result,255,0);
      btShortInt:
        // value to shortint  ->  value & 255 << 24 >> 24
        if FromBT<>btShortInt then
          Result:=CreateBitWiseAnd(PosEl,Result,255,24);
      btWord:
        // value to word  ->  value & 65535
        if not (FromBT in [btByte,btWord]) then
          Result:=CreateBitWiseAnd(PosEl,Result,65535,0);
      btSmallInt:
        // value to smallint  ->  value & 65535 << 16 >> 16
        if not (FromBT in [btShortInt,btSmallInt]) then
          Result:=CreateBitWiseAnd(PosEl,Result,65535,16);
      btLongWord:
        // value to longword  ->  value >>> 0
        if not (FromBT in [btByte,btWord,btLongWord,btUIntSingle]) then
          begin
          ShiftEx:=TJSURShiftExpression(CreateElement(TJSURShiftExpression,PosEl));
          ShiftEx.A:=Result;
          ShiftEx.B:=CreateLiteralNumber(PosEl,0);
          Result:=ShiftEx;
          end;
      btLongint:
        // value to longint  ->  value & 0xffffffff
        if not (FromBT in [btShortInt,btSmallInt,btLongint,btIntSingle]) then
          Result:=CreateBitWiseAnd(PosEl,Result,$ffffffff,0);
      end;
    end;
end;

function TPasToJSConverter.CreateBitWiseAnd(El: TPasElement; Value: TJSElement;
  const Mask: TMaxPrecInt; Shift: integer): TJSElement;
// if sign=false: Value & Mask
// if sign=true:  Value & Mask << ZeroBits >> ZeroBits
var
  AndEx: TJSBitwiseAndExpression;
  Hex: String;
  i: Integer;
  ShiftEx: TJSShiftExpression;
begin
  AndEx:=TJSBitwiseAndExpression(CreateElement(TJSBitwiseAndExpression,El));
  Result:=AndEx;
  AndEx.A:=Value;
  AndEx.B:=CreateLiteralNumber(El,Mask);
  if Mask>999999 then
    begin
    Hex:=HexStr(Mask,8);
    i:=1;
    while i<8 do
      if Hex[i]='0' then
        inc(i)
      else
        break;
    Hex:=Copy(Hex,i,8);
    TJSLiteral(AndEx.B).Value.CustomValue:=TJSString('0x'+Hex);
    end;
  if Shift>0 then
    begin
    // value << ZeroBits
    ShiftEx:=TJSLShiftExpression(CreateElement(TJSLShiftExpression,El));
    ShiftEx.A:=Result;
    Result:=ShiftEx;
    ShiftEx.B:=CreateLiteralNumber(El,Shift);
    // value << ZeroBits >> ZeroBits
    ShiftEx:=TJSRShiftExpression(CreateElement(TJSRShiftExpression,El));
    ShiftEx.A:=Result;
    Result:=ShiftEx;
    ShiftEx.B:=CreateLiteralNumber(El,Shift);
    end;
end;

function TPasToJSConverter.CreateBitWiseLongword(El: TPasElement;
  Value: TJSElement): TJSElement;
var
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(El);
  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnBitwiseLongwordFix),El);
  Call.AddArg(Value);
  Result:=Call;
end;

function TPasToJSConverter.ConvertInheritedExpr(El: TInheritedExpr;
  AContext: TConvertContext): TJSElement;

  function CreateAncestorCall(ParentEl: TPasElement; Apply: boolean;
    AncestorProc: TPasProcedure; ParamsExpr: TParamsExpr): TJSElement;
  var
    FunName, SelfName: String;
    Call: TJSCallExpression;
    SelfContext: TFunctionContext;
    ClassScope, AncestorScope: TPasClassScope;
    AncestorClass, aClass: TPasClassType;
  begin
    Result:=nil;
    SelfContext:=AContext.GetSelfContext;
    if SelfContext=nil then
      RaiseInconsistency(20170418114702,El);
    SelfName:=GetLocalName(SelfContext.ThisVar.Element,[cvkCurType,cvkInstance],AContext);

    if Apply and (SelfContext<>AContext) then
      DoError(20170418204325,nNestedInheritedNeedsParameters,sNestedInheritedNeedsParameters,
        [],El);

    Call:=nil;
    try
      Call:=CreateCallExpression(ParentEl);
      if (AncestorProc.Parent is TPasClassType)
          and TPasClassType(AncestorProc.Parent).IsExternal then
        begin
        // ancestor is in an external class
        // They could be overriden, without a Pascal declaration
        // -> use the direct ancestor class of the current proc
        aClass:=SelfContext.ThisVar.Element as TPasClassType;
        if aClass.CustomData=nil then
          RaiseInconsistency(20170323111252,aClass);
        ClassScope:=TPasClassScope(aClass.CustomData);
        AncestorScope:=ClassScope.AncestorScope;
        if AncestorScope=nil then
          RaiseInconsistency(20170323111306,aClass);
        AncestorClass:=AncestorScope.Element as TPasClassType;
        if (AncestorProc.ClassType=TPasConstructor) and SameText(AncestorProc.Name,'new')
            and AContext.Resolver.IsExternalClass_Name(TPasClassType(AncestorProc.Parent),'Function') then
          begin
          // calling ancestor new constructor
          // this.$func(param1,param2,...)
          FunName:='this.'+GetBIName(pbifnClassAncestorFunc);
          Call.Expr:=CreatePrimitiveDotExpr(FunName,ParentEl);
          CreateProcedureCall(Call,ParamsExpr,AncestorProc.ProcType,AContext);
          Result:=Call;
          exit;
          end
        else
          FunName:=CreateReferencePath(AncestorClass,AContext,rpkPathAndName,true)
            +'.'+TransformElToJSName(AncestorProc,AContext);
        end
      else
        FunName:=CreateReferencePath(AncestorProc,AContext,rpkPathAndName,true);
      if AncestorProc.ProcType.Args.Count=0 then
        Apply:=false;
      if Apply and (SelfContext=AContext) then
        // create "ancestor.funcname.apply(this,arguments)"
        FunName:=FunName+'.apply'
      else
        // create "ancestor.funcname.call(this,param1,param2,...)"
        FunName:=FunName+'.call';
      Call.Expr:=CreatePrimitiveDotExpr(FunName,ParentEl);
      Call.AddArg(CreatePrimitiveDotExpr(SelfName,ParentEl));
      if Apply then
        // "inherited;" -> pass the arguments
        Call.AddArg(CreatePrimitiveDotExpr('arguments',ParentEl))
      else
        // "inherited Name(...)" -> pass the user arguments
        CreateProcedureCall(Call,ParamsExpr,AncestorProc.ProcType,AContext);

      if (AncestorProc is TPasFunction)
          and not AncestorProc.IsAsync
          and AContext.Resolver.IsInterfaceType(
              TPasFunction(AncestorProc).FuncType.ResultEl.ResultType,citCom) then
        Call:=CreateIntfRef(Call,AContext,El);

      Result:=Call;
    finally
      if Result=nil then
        Call.Free;
    end;
  end;

var
  Right: TPasExpr;
  Ref: TResolvedReference;
  PrimExpr: TPrimitiveExpr;
  AncestorProc: TPasProcedure;
  ParamsExpr: TParamsExpr;
begin
  Result:=nil;
  if (El.Parent is TBinaryExpr) and (TBinaryExpr(El.Parent).OpCode=eopNone)
      and (TBinaryExpr(El.Parent).left=El) then
    begin
    // "inherited <name>"
    AncestorProc:=nil;
    ParamsExpr:=nil;
    Right:=TBinaryExpr(El.Parent).right;
    if Right.ClassType=TPrimitiveExpr then
      begin
      PrimExpr:=TPrimitiveExpr(Right);
      Ref:=PrimExpr.CustomData as TResolvedReference;
      if rrfImplicitCallWithoutParams in Ref.Flags then
        begin
        // inherited <function>
        // -> create "AncestorProc.call(this,defaultargs)"
        AncestorProc:=Ref.Declaration as TPasProcedure;
        end
      else
        begin
        // inherited <varname>
        // all variables have unique names -> simply access it
        Result:=ConvertPrimitiveExpression(PrimExpr,AContext);
        exit;
        end;
      end
    else if Right.ClassType=TParamsExpr then
      begin
      ParamsExpr:=TParamsExpr(Right);
      if ParamsExpr.Kind=pekFuncParams then
        begin
        if ParamsExpr.Value is TPrimitiveExpr then
          begin
          // inherited <function>(args)
          // -> create "AncestorProc.call(this,args,defaultargs)"
          PrimExpr:=TPrimitiveExpr(ParamsExpr.Value);
          Ref:=PrimExpr.CustomData as TResolvedReference;
          AncestorProc:=Ref.Declaration as TPasProcedure;
          end
        else
          DoError(20170418205802,nXExpectedButYFound,sXExpectedButYFound,
            ['inherited name()',ParamsExpr.Value.ElementTypeName],ParamsExpr.Value);
        end
      else
        begin
        // inherited <varname>[]
        // all variables have unique names -> simply access it
        Result:=ConvertExpression(Right,AContext);
        exit;
        end;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertInheritedExpression Parent=',GetTreeDbg(El.Parent,2));
      {$ENDIF}
      DoError(20170418205955,nXExpectedButYFound,sXExpectedButYFound,
        ['inherited name()',Right.ElementTypeName],Right);
      end;
    if AncestorProc=nil then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertInheritedExpression Right=',GetObjName(Right));
      {$ENDIF}
      RaiseNotSupported(El,AContext,20170201190824);
      end;
    //writeln('TPasToJSConverter.ConvertInheritedExpression Func=',GetObjName(FuncContext.PasElement));
    Result:=CreateAncestorCall(Right,false,AncestorProc,ParamsExpr);
    end
  else
    begin
    // "inherited;"
    if El.CustomData=nil then
      exit; // "inherited;" when there is no AncestorProc proc -> silently ignore
    // create "AncestorProc.apply(this,arguments)"
    Ref:=TResolvedReference(El.CustomData);
    AncestorProc:=Ref.Declaration as TPasProcedure;
    Result:=CreateAncestorCall(El,true,AncestorProc,nil);
    end;
end;

function TPasToJSConverter.ConvertSelfExpression(El: TSelfExpr;
  AContext: TConvertContext): TJSElement;
begin
  Result:=ConvertIdentifierExpr(El,'Self',AContext);
end;

function TPasToJSConverter.ConvertParamsExpr(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
begin
  Result:=Nil;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertParamsExpression ',GetObjName(El),' El.Kind=',ExprKindNames[El.Kind]);
  {$ENDIF}
  Case El.Kind of
    pekFuncParams:
      Result:=ConvertFuncParams(El,AContext);
    pekArrayParams:
      Result:=ConvertArrayParams(El,AContext);
    pekSet:
      Result:=ConvertArrayOrSetLiteral(El,AContext);
  else
    RaiseNotSupported(El,AContext,20170209103235,ExprKindNames[El.Kind]);
  end;
end;

function TPasToJSConverter.ConvertArrayParams(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  ArgContext: TConvertContext;

  procedure RaiseIllegalBrackets(id: TMaxPrecInt; const ResolvedEl: TPasResolverResult);
  begin
    DoError(id,nIllegalQualifierAfter,sIllegalQualifierAfter,
      ['[',AContext.Resolver.GetResolverResultDescription(ResolvedEl,true)],El);
  end;

  function GetValueReference: TResolvedReference;
  var
    Value: TPasExpr;
  begin
    Result:=nil;
    Value:=El.Value;
    if (Value.ClassType=TPrimitiveExpr)
        and (Value.CustomData is TResolvedReference) then
      exit(TResolvedReference(Value.CustomData));
  end;

  function ConvertIndexMinus1(Param: TPasExpr): TJSElement;
  var
    NeedMinus1: Boolean;
    JSVal: TJSValue;
    MinusJS: TJSAdditiveExpressionMinus;
  begin
    Result:=ConvertExpression(Param,ArgContext);
    NeedMinus1:=true;
    if Result is TJSLiteral then
      begin
      JSVal:=TJSLiteral(Result).Value;
      if (JSVal.ValueType=jstNumber) then
        begin
        // simply subtract 1 from constant
        JSVal.AsNumber:=JSVal.AsNumber-1;
        NeedMinus1:=false;
        end;
      end;
    if NeedMinus1 then
      begin
      // index-1
      MinusJS:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,Param));
      MinusJS.A:=Result;
      MinusJS.B:=CreateLiteralNumber(Param,1);
      Result:=MinusJS;
      end;
  end;

  procedure ConvertStringBracket(const ResolvedValue: TPasResolverResult);
  var
    CallEx, SetStrCall: TJSCallExpression;
    Param: TPasExpr;
    DotExpr: TJSDotMemberExpression;
    AssignContext: TAssignContext;
    AssignSt: TJSSimpleAssignStatement;
    OldAccess: TCtxAccess;
    IndexExpr: TJSElement;
    Arg: TPasArgument;
    IsRangeCheck: Boolean;
  begin
    Result:=nil;
    IsRangeCheck:=(bsRangeChecks in AContext.ScannerBoolSwitches)
              and (AContext.Access in [caRead,caAssign]);
    Param:=El.Params[0];
    case AContext.Access of
    caAssign:
      begin
      // s[index] := value
      AssignContext:=AContext.AccessContext as TAssignContext;
      if AssignContext.RightSide=nil then
        RaiseInconsistency(20180123192020,El);

      AssignSt:=nil;
      SetStrCall:=nil;
      CallEx:=nil;
      try
        // CallEx: rtl.setCharAt(s,index,value)

        // rtl.setCharAt
        CallEx:=CreateCallExpression(El);
        if IsRangeCheck then
          CallEx.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnRangeCheckSetCharAt)])
        else
          CallEx.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnSetCharAt)]);
        // first param  s
        OldAccess:=AContext.Access;
        AContext.Access:=caRead;
        CallEx.AddArg(ConvertExpression(El.Value,AContext));
        // second param  index-1
        CallEx.AddArg(ConvertIndexMinus1(Param));
        AContext.Access:=OldAccess;
        // third param  value
        CallEx.AddArg(AssignContext.RightSide);
        AssignContext.RightSide:=nil;

        if ResolvedValue.IdentEl is TPasArgument then
          begin
          Arg:=TPasArgument(ResolvedValue.IdentEl);
          if Arg.Access in [argVar,argOut] then
            begin
            // call by reference
            // s[index] := value  ->  s.set(CallEx)
            SetStrCall:=CreateCallExpression(El.Value);
            SetStrCall.Expr:=CreateMemberExpression([TransformArgName(Arg,AContext),TempRefObjSetterName]);
            SetStrCall.AddArg(CallEx);
            AssignContext.Call:=CallEx;
            CallEx:=nil;
            Result:=SetStrCall;
            end;
          end
        else if ResolvedValue.IdentEl is TPasProperty then
          RaiseNotSupported(El,AContext,20180124115924);
        if Result=nil then
          begin
          // s[index] := value  ->  s = CallEx
          AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
          AssignSt.Expr:=CallEx;
          AssignContext.Call:=CallEx;
          CallEx:=nil;
          OldAccess:=AContext.Access;
          AContext.Access:=caRead;
          AssignSt.LHS:=ConvertExpression(El.Value,AContext);
          Result:=AssignSt;
          end;
      finally
        if Result=nil then
          begin
          CallEx.Free;
          SetStrCall.Free;
          AssignSt.Free;
          end;
      end;
      end;
    caRead:
      begin

      CallEx:=CreateCallExpression(El);
      try
        if IsRangeCheck and not TBinaryExpr.IsRightSubIdent(El) then
          begin
          // read s[index]  ->  rtl.rcCharAt(s,index-1)
          CallEx.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnRangeCheckGetCharAt),El);
          CallEx.AddArg(ConvertExpression(El.Value,AContext));
          end
        else
          begin
          // s[index]  ->  s.charAt(index-1)
          // add string accessor
          DotExpr:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,El));
          CallEx.Expr:=DotExpr;
          DotExpr.MExpr:=ConvertExpression(El.Value,AContext);
          DotExpr.Name:='charAt';
          end;

        // add parameter "index-1"
        IndexExpr:=ConvertIndexMinus1(Param);
        CallEx.AddArg(IndexExpr);
        Result:=CallEx;
      finally
        if Result=nil then
          CallEx.Free;
      end;
      end;
    else
      RaiseNotSupported(El,AContext,20170213213101);
    end;
  end;

  procedure ConvertArrayBracket(ArrayEl: TPasArrayType);
  var
    BracketEx, Sub: TJSBracketMemberExpression;
    i, ArgNo: Integer;
    Arg, ArrJS: TJSElement;
    OldAccess: TCtxAccess;
    Ranges: TPasExprArray;
    Int: TMaxPrecInt;
    Param: TPasExpr;
    JSAdd: TJSAdditiveExpression;
    LowRg: TResEvalValue;
    IsRangeCheck, ok, NeedRangeCheck: Boolean;
    CallEx, CallAssign: TJSCallExpression;
    AssignContext: TAssignContext;
    ArgList: TFPList;
    IsAssignRecord: boolean;
    {$IFDEF FPC_HAS_CPSTRING}
    w: WideChar;
    {$ENDIF}
  begin
    Result:=nil;
    Arg:=nil;
    ArrJS:=nil;
    ArgList:=TFPList.Create;
    NeedRangeCheck:=false;

    ok:=false;
    try
      // add read accessor
      OldAccess:=AContext.Access;
      AContext.Access:=caRead;
      ArrJS:=ConvertExpression(El.Value,AContext);
      AContext.Access:=OldAccess;

      ArgNo:=0;
      repeat
        // Note: dynamic array has length(ArrayEl.Ranges)=0
        Ranges:=ArrayEl.Ranges;
        for i:=1 to Max(length(Ranges),1) do
          begin
          // add parameter
          Param:=El.Params[ArgNo];
          ArgContext.Access:=caRead;
          Arg:=ConvertExpression(Param,ArgContext);
          ArgContext.Access:=OldAccess;
          if not (Arg is TJSLiteral) then
            NeedRangeCheck:=true;

          if i<=length(Ranges) then
            begin
            // static array
            LowRg:=ArgContext.Resolver.EvalRangeLimit(Ranges[i-1],[refConst],true,El);
            if LowRg=nil then
              RaiseNotSupported(Param,ArgContext,20170910163341);
            try
              Int:=0;
              case LowRg.Kind of
              revkBool:
                if TResEvalBool(LowRg).B=false then
                  begin
                  // array starts at 'false'
                  if (Arg is TJSLiteral) and (TJSLiteral(Arg).Value.ValueType=jstBoolean) then
                    begin
                    // convert Pascal boolean literal to JS number
                    if TJSLiteral(Arg).Value.AsBoolean then
                      TJSLiteral(Arg).Value.AsNumber:=1
                    else
                      TJSLiteral(Arg).Value.AsNumber:=0;
                    end
                  else
                    begin
                    // -> convert bool to int with unary plus:  +bool
                    Arg:=CreateUnaryPlus(Arg,Param);
                    end;
                  end
                else
                  begin
                  // array starts at 'true'
                  if (Arg is TJSLiteral) and (TJSLiteral(Arg).Value.ValueType=jstBoolean) then
                    begin
                    if TJSLiteral(Arg).Value.AsBoolean then
                      TJSLiteral(Arg).Value.AsNumber:=0
                    else
                      ArgContext.Resolver.ExprEvaluator.EmitRangeCheckConst(
                        20170910203312,'false','true','true',Param,mtError);
                    end
                  else
                    begin
                    // convert bool to int with offset: 1-bool
                    JSAdd:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,Param));
                    JSAdd.A:=CreateLiteralNumber(Param,1);
                    JSAdd.B:=Arg;
                    Arg:=JSAdd;
                    end;
                  end;
              revkEnum:
                Int:=TResEvalEnum(LowRg).Index;
              revkInt:
                Int:=TResEvalInt(LowRg).Int;
              {$IFDEF FPC_HAS_CPSTRING}
              revkString:
                begin
                if length(TResEvalString(LowRg).S)<>1 then
                  begin
                  if ArgContext.Resolver.ExprEvaluator.GetWideChar(TResEvalString(LowRg).S,w) then
                    Int:=ord(w)
                  else
                    ArgContext.Resolver.RaiseXExpectedButYFound(20170910213203,'char','string',Param);
                  end
                else
                  Int:=ord(TResEvalString(LowRg).S[1]);
                Arg:=ConvertCharToInt(Arg,Param,ArgContext);
                end;
              {$ENDIF}
              revkUnicodeString:
                begin
                if length(TResEvalUTF16(LowRg).S)<>1 then
                  ArgContext.Resolver.RaiseXExpectedButYFound(20170910213247,'char','string',Param)
                else
                  Int:=ord(TResEvalUTF16(LowRg).S[1]);
                Arg:=ConvertCharToInt(Arg,Param,ArgContext);
                end
              else
                RaiseNotSupported(Param,ArgContext,20170910170446);
              end;
              if Int<>0 then
                begin
                if (Arg is TJSLiteral) and (TJSLiteral(Arg).Value.ValueType=jstNumber) then
                  // parameter is single number -> simply subtract the offset
                  TJSLiteral(Arg).Value.AsNumber:=TJSLiteral(Arg).Value.AsNumber-Int
                else
                  begin
                  // parameter is an expression -> add offset
                  if Int>0 then
                    begin
                    // Arg-Offset
                    JSAdd:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,Param));
                    JSAdd.A:=Arg;
                    JSAdd.B:=CreateLiteralNumber(Param,Int);
                    Arg:=JSAdd;
                    end
                  else
                    begin
                    // Arg+Offset
                    JSAdd:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,Param));
                    JSAdd.A:=Arg;
                    JSAdd.B:=CreateLiteralNumber(Param,-Int);
                    Arg:=JSAdd;
                    end;
                  end;
                end;
            finally
              ReleaseEvalValue(LowRg);
            end;
            end;

          ArgList.Add(Arg);
          Arg:=nil;
          inc(ArgNo);
          if ArgNo>length(El.Params) then
            RaiseInconsistency(20170206180553,El);
          end;
        if ArgNo=length(El.Params) then
          break;
        // continue in sub array
        ArrayEl:=AContext.Resolver.ResolveAliasType(ArrayEl.ElType) as TPasArrayType;
      until ArrayEl=nil;

      IsRangeCheck:=NeedRangeCheck
                and (bsRangeChecks in AContext.ScannerBoolSwitches)
                and (AContext.Access in [caRead,caAssign]);
      AssignContext:=nil;
      IsAssignRecord:=false;
      if AContext.Access=caAssign then
        begin
        AssignContext:=AContext.AccessContext as TAssignContext;
        if AssignContext.Call<>nil then
          RaiseNotSupported(El,AContext,20180424192155);
        IsAssignRecord:=AssignContext.LeftResolved.LoTypeEl is TPasRecordType;
        end;

      if IsRangeCheck and not TBinaryExpr.IsRightSubIdent(El) then
        begin
        // read a[i,j,k]  ->  rtl.rcArrR(a,i,j,k)
        // assign a[i,j,k]:=RHS  ->  rtl.rcArrW(a,i,j,k,RHS)
        // assign ArrOfRecord[i,j]:=RHS  ->  rtl.rcArrR(a,i,j,k).$assign(RHS)
        CallEx:=CreateCallExpression(El);
        Result:=CallEx;
        if (AContext.Access=caRead) or IsAssignRecord then
          CallEx.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnRangeCheckArrayRead),El)
        else
          CallEx.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnRangeCheckArrayWrite),El);
        CallEx.AddArg(ArrJS); ArrJS:=nil;
        for i:=0 to ArgList.Count-1 do
          CallEx.AddArg(TJSElement(ArgList[i]));
        ArgList.Clear;
        if AContext.Access=caAssign then
          begin
          // a[i,j,k]:=
          if IsAssignRecord then
            begin
            // rtl.rcArrR(a,i,j,k).$assign(RHS)
            CallAssign:=CreateCallExpression(El);
            CallAssign.Expr:=CreateDotNameExpr(El,CallEx,
                                       TJSString(GetBIName(pbifnRecordAssign)));
            CallEx:=CallAssign;
            end;
          CallEx.AddArg(AssignContext.RightSide);
          AssignContext.RightSide:=nil;
          AssignContext.Call:=CallEx;
          // ToDo: range check value
          Result:=CallEx;
          end;
        end
      else
        begin
        BracketEx:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
        BracketEx.MExpr:=ArrJS; ArrJS:=nil;
        for i:=0 to ArgList.Count-1 do
          begin
          if BracketEx.Name<>nil then
            begin
            // nested [][]
            Sub:=BracketEx;
            BracketEx:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
            BracketEx.MExpr:=Sub;
            end;
          BracketEx.Name:=TJSElement(ArgList[i]);
          end;
        Result:=BracketEx;
        ArgList.Clear;
        if IsAssignRecord then
          begin
          // assign ArrOfRecord[i,j]:=RHS  ->  a[i][j].$assign(RHS)
          CallAssign:=CreateCallExpression(El);
          CallAssign.Expr:=CreateDotNameExpr(El,Result,
                                       TJSString(GetBIName(pbifnRecordAssign)));
          Result:=CallAssign;
          CallAssign.AddArg(AssignContext.RightSide);
          AssignContext.RightSide:=nil;
          AssignContext.Call:=CallAssign;
          end;
        end;

      ok:=true;
    finally
      if not ok then
        begin
        ArrJS.Free;
        for i:=0 to ArgList.Count-1 do
          TJSElement(ArgList[i]).{$IFDEF pas2js}Destroy{$ELSE}Free{$ENDIF};
        Arg.Free;
        Result.Free;
        end;
      ArgList.Free;
    end;
  end;

  function IsJSBracketAccessorAndConvert(Prop: TPasProperty;
    AccessEl: TPasElement;
    AContext: TConvertContext; ChompPropName: boolean): boolean;
  // If El.Value contains property name set ChompPropName = true
  var
    Bracket: TJSBracketMemberExpression;
    OldAccess: TCtxAccess;
    PathEl: TPasExpr;
    Ref: TResolvedReference;
    Path: String;
  begin
    if not AContext.Resolver.IsExternalBracketAccessor(AccessEl) then
      exit(false);
    Result:=true;
    // bracket accessor of external class
    if AContext.Resolver.GetPasPropertyArgs(Prop).Count<>1 then
      RaiseInconsistency(20170403003753,Prop);
    // bracket accessor of external class  -> create  PathEl[param]
    Bracket:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El.Params[0]));
    try
      PathEl:=El.Value;
      if ChompPropName then
        begin
        if (PathEl is TPrimitiveExpr)
            and (TPrimitiveExpr(PathEl).Kind=pekIdent)
            and (PathEl.CustomData is TResolvedReference) then
          begin
          // propname without path, e.g.  propname[param]
          Ref:=TResolvedReference(PathEl.CustomData);
          Path:=CreateReferencePath(Prop,AContext,rpkPath,false,Ref);
          if Path<>'' then
            Bracket.MExpr:=CreatePrimitiveDotExpr(Path,PathEl);
          PathEl:=nil;
          end
        else if (PathEl is TBinaryExpr)
            and (TBinaryExpr(PathEl).OpCode=eopSubIdent)
            and (TBinaryExpr(PathEl).right is TPrimitiveExpr)
            and (TPrimitiveExpr(TBinaryExpr(PathEl).right).Kind=pekIdent) then
          begin
          // instance.propname[param]  ->  instance[param]
          PathEl:=TBinaryExpr(PathEl).left;
          end
        else
          RaiseNotSupported(El.Value,AContext,20170402225050);
        end;

      if (PathEl<>nil) and (Bracket.MExpr=nil) then
        begin
        OldAccess:=AContext.Access;
        AContext.Access:=caRead;
        Bracket.MExpr:=ConvertExpression(PathEl,AContext);
        AContext.Access:=OldAccess;
        end;

      OldAccess:=ArgContext.Access;
      ArgContext.Access:=caRead;
      Bracket.Name:=ConvertExpression(El.Params[0],ArgContext);
      ArgContext.Access:=OldAccess;
      ConvertArrayParams:=Bracket;
      Bracket:=nil;
    finally
      Bracket.Free;
    end;
  end;

  procedure ConvertIndexedProperty(Prop: TPasProperty; AContext: TConvertContext;
    CheckPath: boolean);
  var
    Call: TJSCallExpression;
    i: Integer;
    TargetArg: TPasArgument;
    Elements: TJSArrayLiteralElements;
    Arg: TJSElement;
    AccessEl: TPasElement;
    AssignContext: TAssignContext;
    OldAccess: TCtxAccess;
    IndexExpr: TPasExpr;
    Value: TResEvalValue;
    PropArgs: TFPList;
    aResolver: TPas2JSResolver;
    TypeEl: TPasType;
    Bin: TBinaryExpr;
    CreateRefPathData: TCreateRefPathData;
  begin
    Result:=nil;
    AssignContext:=nil;
    aResolver:=AContext.Resolver;
    Call:=nil;
    try
      // find getter/setter
      case AContext.Access of
      caAssign:
        AccessEl:=aResolver.GetPasPropertySetter(Prop);
      caRead:
        AccessEl:=aResolver.GetPasPropertyGetter(Prop);
      else
        RaiseNotSupported(El,AContext,20170213213317);
      end;
      if IsJSBracketAccessorAndConvert(Prop,AccessEl,AContext,true) then
        exit;

      // create call
      if aResolver.IsHelperMethod(AccessEl) then
        begin
        if CheckPath then
          Call:=CreateCallHelperMethod(TPasProcedure(AccessEl),El.Value,AContext)
        else
          Call:=CreateCallHelperMethod(TPasProcedure(AccessEl),El,AContext)
        end
      else
        Call:=CreateCallExpression(El);

      if AContext.Access=caAssign then
        begin
        AssignContext:=AContext.AccessContext as TAssignContext;
        AssignContext.PropertyEl:=Prop;
        AssignContext.Call:=Call;
        end;

      if CheckPath and (Call.Expr=nil) then
        if aResolver.IsNameExpr(El.Value) then
          // no special context
        else if El.Value is TBinaryExpr then
          begin
          // convert left
          Bin:=TBinaryExpr(El.Value);
          if Bin.OpCode<>eopSubIdent then
            RaiseNotSupported(El,AContext,20190116100510);
          CreateRefPathData.El:=AccessEl;
          CreateRefPathData.Full:=false;
          CreateRefPathData.Ref:=GetValueReference;
          Call.Expr:=ConvertSubIdentExprCustom(Bin,AContext,
            @OnCreateReferencePathExpr,@CreateRefPathData);
          end
        else
          begin
          {$IFDEF VerbosePas2JS}
          writeln('ConvertFuncParams.ConvertIndexedProperty ',GetObjName(El.Value));
          {$ENDIF}
          RaiseNotSupported(El,AContext,20190116100431);
          end;

      if Call.Expr=nil then
        Call.Expr:=CreateReferencePathExpr(AccessEl,AContext,false,GetValueReference);

      Elements:=Call.Args.Elements;
      OldAccess:=ArgContext.Access;
      // add params
      PropArgs:=aResolver.GetPasPropertyArgs(Prop);
      i:=0;
      while i<PropArgs.Count do
        begin
        TargetArg:=TPasArgument(PropArgs[i]);
        Arg:=CreateProcCallArg(El.Params[i],TargetArg,ArgContext);
        Elements.AddElement.Expr:=Arg;
        inc(i);
        end;
      // fill up default values
      while i<PropArgs.Count do
        begin
        TargetArg:=TPasArgument(PropArgs[i]);
        if TargetArg.ValueExpr=nil then
          begin
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.ConvertArrayParams.ConvertIndexedProperty missing default value: Prop=',Prop.Name,' i=',i);
          {$ENDIF}
          RaiseInconsistency(20170206185126,TargetArg);
          end;
        AContext.Access:=caRead;
        Arg:=ConvertExpression(TargetArg.ValueExpr,ArgContext);
        Elements.AddElement.Expr:=Arg;
        inc(i);
        end;
      // add index specifier
      IndexExpr:=aResolver.GetPasPropertyIndex(Prop);
      if IndexExpr<>nil then
        begin
        Value:=aResolver.Eval(IndexExpr,[refConst]);
        try
          Elements.AddElement.Expr:=ConvertConstValue(Value,ArgContext,El);
        finally
          ReleaseEvalValue(Value);
        end;
        end;
      // finally add as last parameter the value
      if AssignContext<>nil then
        begin
        Elements.AddElement.Expr:=AssignContext.RightSide;
        AssignContext.RightSide:=nil;
        end;

      ArgContext.Access:=OldAccess;

      // add interface reference
      if AContext.Access=caRead then
        begin
        TypeEl:=aResolver.GetPasPropertyType(Prop);
        if aResolver.IsInterfaceType(TypeEl,citCom) then
          Call:=CreateIntfRef(Call,AContext,El);
        end;

      Result:=Call;
    finally
      if Result=nil then
        begin
        if (AssignContext<>nil) and (AssignContext.Call=Call) then
          AssignContext.Call:=nil;
        Call.Free;
        end;
    end;
  end;

  procedure ConvertDefaultProperty(const ResolvedEl: TPasResolverResult;
    Prop: TPasProperty);
  var
    DotContext: TDotContext;
    Left, Right: TJSElement;
    OldAccess: TCtxAccess;
    AccessEl, SetAccessEl: TPasElement;
    aResolver: TPas2JSResolver;
  begin
    aResolver:=AContext.Resolver;
    case AContext.Access of
    caAssign:
      begin
      AccessEl:=aResolver.GetPasPropertySetter(Prop);
      if IsJSBracketAccessorAndConvert(Prop,AccessEl,AContext,false) then
        exit;
      end;
    caRead:
      begin
      AccessEl:=aResolver.GetPasPropertyGetter(Prop);
      if IsJSBracketAccessorAndConvert(Prop,AccessEl,AContext,false) then
        exit;
      end;
    caByReference:
      begin
      AccessEl:=aResolver.GetPasPropertyGetter(Prop);
      SetAccessEl:=aResolver.GetPasPropertySetter(Prop);
      if aResolver.IsExternalBracketAccessor(AccessEl) then
        begin
        if aResolver.IsExternalBracketAccessor(SetAccessEl) then
          begin
          // read and write are brackets -> easy
          if not IsJSBracketAccessorAndConvert(Prop,AccessEl,AContext,false) then
            RaiseNotSupported(El,AContext,20170405090845);
          exit;
          end;
        end;
      RaiseNotSupported(El,AContext,20170403000550);
      end;
    else
      RaiseNotSupported(El,AContext,20170402233834);
    end;

    if aResolver.IsHelperMethod(AccessEl) then
      begin
      ConvertIndexedProperty(Prop,AContext,false);
      exit;
      end;

    DotContext:=nil;
    Left:=nil;
    Right:=nil;
    try
      OldAccess:=AContext.Access;
      AContext.Access:=caRead;
      Left:=ConvertExpression(El.Value,AContext);
      AContext.Access:=OldAccess;

      DotContext:=TDotContext.Create(El.Value,Left,AContext);
      DotContext.LeftResolved:=ResolvedEl;
      ConvertIndexedProperty(Prop,DotContext,false);
      if DotContext.JS<>nil then
        RaiseNotSupported(El,AContext,20180509134226,GetObjName(DotContext.JS));
      Right:=Result;
      Result:=nil;
    finally
      DotContext.Free;
      if Right=nil then
        Left.Free;
    end;
    Result:=CreateDotExpression(El,Left,Right,true);
  end;

Var
  ResolvedEl: TPasResolverResult;
  TypeEl: TPasType;
  B: TJSBracketMemberExpression;
  OldAccess: TCtxAccess;
  aResolver: TPas2JSResolver;
  Ref: TResolvedReference;
begin
  if El.Kind<>pekArrayParams then
    RaiseInconsistency(20170209113713,El);
  ArgContext:=AContext.GetNonDotContext;
  aResolver:=AContext.Resolver;
  if aResolver=nil then
    begin
    // without Resolver
    if Length(El.Params)>1 then
      RaiseNotSupported(El,AContext,20170207151325,'Cannot convert 2-dim arrays');
    B:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
    try
      // add reference
      OldAccess:=AContext.Access;
      AContext.Access:=caRead;
      B.MExpr:=ConvertExpression(El.Value,AContext);

      // add parameter
      OldAccess:=ArgContext.Access;
      ArgContext.Access:=caRead;
      B.Name:=ConvertExpression(El.Params[0],ArgContext);
      ArgContext.Access:=OldAccess;

      Result:=B;
    finally
      if Result=nil then
        B.Free;
    end;
    exit;
    end;

  // has Resolver
  aResolver.ComputeElement(El.Value,ResolvedEl,[]);

  if El.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(El.CustomData);
    if Ref.Declaration is TPasProperty then
      begin
      ConvertDefaultProperty(ResolvedEl,TPasProperty(Ref.Declaration));
      exit;
      end;
    end;

  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertArrayParams Value=',GetResolverResultDbg(ResolvedEl));
  {$ENDIF}
  if ResolvedEl.BaseType in btAllJSStrings then
    // aString[]
    ConvertStringBracket(ResolvedEl)
  else if (ResolvedEl.IdentEl is TPasProperty)
      and (aResolver.GetPasPropertyArgs(TPasProperty(ResolvedEl.IdentEl)).Count>0) then
    // aProperty[]
    ConvertIndexedProperty(TPasProperty(ResolvedEl.IdentEl),AContext,true)
  else if ResolvedEl.BaseType=btContext then
    begin
    TypeEl:=ResolvedEl.LoTypeEl;
    if TypeEl.ClassType=TPasArrayType then
      // anArray[]
      ConvertArrayBracket(TPasArrayType(TypeEl))
    else
      RaiseIllegalBrackets(20170206181220,ResolvedEl);
    end
  else
    RaiseIllegalBrackets(20170206180222,ResolvedEl);
end;

function TPasToJSConverter.ConvertFuncParams(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  aResolver: TPas2JSResolver;
  DotBin: TBinaryExpr;
  Call: TJSCallExpression;
  Elements: TJSArrayLiteralElements;

  procedure CreateFreeOrNewInstanceCall(Ref: TResolvedReference);
  var
    JsArrLit: TJSArrayLiteral;
    LeftResolved: TPasResolverResult;
    OldAccess: TCtxAccess;
    Left, DotExpr: TJSElement;
    DotContext: TDotContext;
  begin
    if DotBin<>nil then
      begin
      aResolver.ComputeElement(DotBin.left,LeftResolved,[]);

      // convert left side
      OldAccess:=AContext.Access;
      AContext.Access:=caRead;
      Left:=ConvertExpression(DotBin.left,AContext);
      if Left=nil then
        RaiseInconsistency(20190116132530,El);
      AContext.Access:=OldAccess;

      DotContext:=TDotContext.Create(DotBin,Left,AContext);
      try
        DotContext.LeftResolved:=LeftResolved;
        Call:=CreateFreeOrNewInstanceExpr(Ref,DotContext);
        if DotContext.JS<>nil then
          RaiseNotSupported(El,AContext,20190116132748);
      finally
        DotContext.Free;
        if Call=nil then
          Left.Free;
      end;
      // connect via dot
      DotExpr:=CreateDotExpression(DotBin,Left,Call,true);
      if DotExpr<>Call then
        RaiseNotSupported(El,AContext,20190116133841);
      end;
    if Call=nil then
      Call:=CreateFreeOrNewInstanceExpr(Ref,AContext);
    if (rrfNewInstance in Ref.Flags)
        and (Ref.Declaration.Parent.ClassType=TPasClassType) then
      begin
      // insert array parameter [], e.g. this.TObject.$create("create",[])
      JsArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
      Call.AddArg(JsArrLit);
      Elements:=JsArrLit.Elements;
      end
    else
      Elements:=Call.Args.Elements;
  end;

  procedure CreateShortRefApply(Value: TPasExpr; TargetProcType: TPasProcedureType);
  var
    TargetProc: TPasProcedure;
    aName: String;
    LeftJS: TJSElement;
    Ref: TResolvedReference;
  begin
    // create  "$lp.apply(LeftJS,args);"
    TargetProc:=TPasProcedure(TargetProcType.Parent);
    aName:=CreateStaticProcPath(TargetProc,AContext);
    Call.Expr:=CreatePrimitiveDotExpr(aName+'.apply',Value);
    if DotBin<>nil then
      begin
      // a.b() -> "$lp.apply(a,args);"
      LeftJS:=ConvertExpression(DotBin.left,AContext);
      if LeftJS=nil then
        RaiseNotSupported(DotBin,AContext,20201030235816);
      end
    else if Value.CustomData is TResolvedReference then
      begin
      // a() -> "$lp.apply(this,args);"  or  "$lp.apply($with,args);"
      Ref:=TResolvedReference(Value.CustomData);
      aName:=CreateReferencePath(TargetProc,AContext,rpkPath,false,Ref);
      LeftJS:=CreatePrimitiveDotExpr(aName,Value);
      if LeftJS=nil then
        RaiseNotSupported(DotBin,AContext,20201031003202);
      end
    else
      RaiseNotSupported(DotBin,AContext,202010310032046);
    Elements.AddElement.Expr:=LeftJS;
  end;

var
  Decl: TPasElement;
  Ref: TResolvedReference;
  BuiltInProc: TResElDataBuiltInProc;
  TargetProc: TPasProcedure;
  TargetProcType: TPasProcedureType;
  JsArrLit: TJSArrayLiteral;
  OldAccess: TCtxAccess;
  DeclResolved, ParamResolved, ValueResolved: TPasResolverResult;
  Param, Value: TPasExpr;
  JSBaseType: TPas2jsBaseType;
  C: TClass;
  aName, ArgName: String;
  aClassTypeEl: TPasClassType;
  ParamTypeEl: TPasType;
  NeedIntfRef: Boolean;
  DestRange, SrcRange: TResEvalValue;
  LastArg: TJSArrayLiteralElement;
  CallArgs: TJSArguments;
begin
  Result:=nil;
  if El.Kind<>pekFuncParams then
    RaiseInconsistency(20170209113515,El);
  aResolver:=AContext.Resolver;
  //writeln('TPasToJSConverter.ConvertFuncParams START pekFuncParams ',GetObjName(El.CustomData),' ',GetObjName(El.Value.CustomData));
  Call:=nil;
  Elements:=nil;
  TargetProcType:=nil;
  DotBin:=nil;

  Value:=El.Value;
  if (not (Value.CustomData is TResolvedReference))
      and (aResolver<>nil)
      and (Value is TBinaryExpr) and (TBinaryExpr(Value).OpCode=eopSubIdent) then
    begin
    //  path.Value()
    DotBin:=TBinaryExpr(Value);
    Value:=DotBin.right;
    end;
  if (not (Value.CustomData is TResolvedReference))
      and (aResolver<>nil)
      and (Value is TInlineSpecializeExpr) then
    begin
    //  Value<>()
    Value:=TInlineSpecializeExpr(Value).NameExpr;
    end;

  if Value.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(Value.CustomData);
    Decl:=Ref.Declaration;
    if Decl is TPasType then
      Decl:=aResolver.ResolveAliasType(TPasType(Decl));
    //writeln('TPasToJSConverter.ConvertFuncParams pekFuncParams TResolvedReference ',GetObjName(Ref.Declaration),' ',GetObjName(Ref.Declaration.CustomData));
    C:=Decl.ClassType;

    if C=TPasUnresolvedSymbolRef then
      begin
      if Decl.CustomData is TResElDataBuiltInProc then
        begin
        BuiltInProc:=TResElDataBuiltInProc(Decl.CustomData);
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.ConvertFuncParams BuiltInProc ',Decl.Name,' ',ResolverBuiltInProcNames[BuiltInProc.BuiltIn]);
        {$ENDIF}
        case BuiltInProc.BuiltIn of
          bfLength: Result:=ConvertBuiltIn_Length(El,AContext);
          bfSetLength: Result:=ConvertBuiltIn_SetLength(El,AContext);
          bfInclude: Result:=ConvertBuiltIn_ExcludeInclude(El,AContext,true);
          bfExclude: Result:=ConvertBuiltIn_ExcludeInclude(El,AContext,false);
          bfExit: Result:=ConvertBuiltIn_Exit(El,AContext);
          bfInc,
          bfDec: Result:=ConvertBuiltIn_IncDec(El,AContext);
          bfAssigned: Result:=ConvertBuiltIn_Assigned(El,AContext);
          bfChr: Result:=ConvertBuiltIn_Chr(El,AContext);
          bfOrd: Result:=ConvertBuiltIn_Ord(El,AContext);
          bfLow: Result:=ConvertBuiltIn_LowHigh(El,AContext,true);
          bfHigh: Result:=ConvertBuiltIn_LowHigh(El,AContext,false);
          bfPred: Result:=ConvertBuiltIn_PredSucc(El,AContext,true);
          bfSucc: Result:=ConvertBuiltIn_PredSucc(El,AContext,false);
          bfStrProc: Result:=ConvertBuiltIn_StrProc(El,AContext);
          bfStrFunc: Result:=ConvertBuiltIn_StrFunc(El,AContext);
          bfWriteStr: Result:=ConvertBuiltIn_WriteStr(El,AContext);
          bfVal: Result:=ConvertBuiltIn_Val(El,AContext);
          bfLo: Result := ConvertBuiltIn_LoHi(El,AContext,True);
          bfHi: Result := ConvertBuiltIn_LoHi(El,AContext,False);
          bfConcatArray: Result:=ConvertBuiltIn_ConcatArray(El,AContext);
          bfConcatString: Result:=ConvertBuiltIn_ConcatString(El,AContext);
          bfCopyArray: Result:=ConvertBuiltIn_CopyArray(El,AContext);
          bfInsertArray: Result:=ConvertBuiltIn_InsertArray(El,AContext);
          bfDeleteArray: Result:=ConvertBuiltIn_DeleteArray(El,AContext);
          bfTypeInfo: Result:=ConvertBuiltIn_TypeInfo(El,AContext);
          bfGetTypeKind: Result:=ConvertBuiltIn_GetTypeKind(El,AContext);
          bfAssert:
            begin
            Result:=ConvertBuiltIn_Assert(El,AContext);
            if Result=nil then exit;
            end;
          bfNew: Result:=ConvertBuiltIn_New(El,AContext);
          bfDispose:
            begin
            Result:=ConvertBuiltIn_Dispose(El,AContext);
            if Result=nil then exit;
            end;
          bfDefault: Result:=ConvertBuiltIn_Default(El,AContext);
          bfCustom:
            case BuiltInProc.Element.Name of
            'Debugger': Result:=ConvertBuiltIn_Debugger(El,AContext);
            'AWait': Result:=ConvertBuiltIn_AWait(El,AContext);
            else
              RaiseNotSupported(El,AContext,20181126101801,'built in custom proc '+BuiltInProc.Element.Name);
            end;
        else
          RaiseNotSupported(El,AContext,20161130164955,'built in proc '+ResolverBuiltInProcNames[BuiltInProc.BuiltIn]);
        end;
        if Result=nil then
          RaiseInconsistency(20170210121932,El);
        exit;
        end
      else if Decl.CustomData is TResElDataBaseType then
        begin
        // typecast to base type
        Result:=ConvertTypeCastToBaseType(El,AContext,TResElDataBaseType(Decl.CustomData));
        exit;
        end
      else
        RaiseNotSupported(El,AContext,20170325160624);
      end
    else if aResolver.IsExternalClassConstructor(Decl) then
      begin
      // create external object/function
      if DotBin<>nil then
        Param:=DotBin.left
      else
        Param:=nil;
      Result:=ConvertExternalConstructor(Param,Ref,El,AContext);
      exit;
      end
    else if aResolver.IsTObjectFreeMethod(Value) then
      begin
      if DotBin<>nil then
        Result:=ConvertTObjectFree_Bin(DotBin,Value,AContext)
      else
        RaiseNotSupported(El,AContext,20190115215224);
      exit;
      end
    else if C.InheritsFrom(TPasProcedure) then
      begin
      TargetProc:=TPasProcedure(Decl);
      if aResolver.IsHelperMethod(TargetProc) then
        begin
        // calling a helper method
        Result:=CreateCallHelperMethod(TargetProc,El.Value,AContext);
        exit;
        end;
      if aResolver.IsExternalBracketAccessor(TargetProc) then
        exit(CreateExternalBracketAccessorCall(El,AContext));
      TargetProcType:=TargetProc.ProcType;
      end
    else if (C=TPasClassType)
        or (C=TPasClassOfType)
        or (C=TPasRecordType)
        or (C=TPasEnumType)
        or (C=TPasRangeType)
        or (C=TPasArrayType)
        or (C=TPasPointerType) then
      begin
      // typecast
      // default is to simply replace  "aType(param)" with "param"
      Param:=El.Params[0];
      aResolver.ComputeElement(Param,ParamResolved,[]);
      ParamTypeEl:=ParamResolved.LoTypeEl;

      if (C=TPasRecordType) and (ParamResolved.BaseType=btUntyped)
          and (ParamResolved.IdentEl is TPasArgument) then
        begin
        // RecordType(UntypedArg) -> UntypedArg
        ArgName:=TransformArgName(TPasArgument(ParamResolved.IdentEl),AContext);
        Result:=CreatePrimitiveDotExpr(ArgName,El);
        exit;
        end
      else if (C=TPasClassType)
          and aResolver.IsExternalClass_Name(TPasClassType(Decl),'Function') then
        begin
        // TJSFunction(param)
        if (Param is TPasExpr) and (TPasExpr(Param).OpCode=eopAddress) then
          begin
          aResolver.ComputeElement(TUnaryExpr(Param).Operand,ValueResolved,[rcNoImplicitProc]);
          if (ValueResolved.BaseType=btProc)
              and (ValueResolved.IdentEl is TPasProcedure) then
            begin
            // TJSFunction(@procname)  -> procname
            Result:=CreateReferencePathExpr(TPasProcedure(ValueResolved.IdentEl),AContext);
            exit;
            end;
          end;
        end;

      Result:=ConvertExpression(Param,AContext);

      if C=TPasRangeType then
        begin
        DestRange:=aResolver.EvalTypeRange(TPasRangeType(Decl),[refConst]);
        SrcRange:=nil;
        try
          if DestRange=nil then
            RaiseNotSupported(El,AContext,20180424124708);
          SrcRange:=aResolver.EvalTypeRange(ParamResolved.LoTypeEl,[]);
          if SrcRange=nil then
            RaiseNotSupported(El,AContext,20180424125331);
          case DestRange.Kind of
          revkRangeInt:
            case TResEvalRangeInt(DestRange).ElKind of
            revskEnum, revskInt:
              // type cast to integer-range
              case SrcRange.Kind of
              revkRangeInt:
                case TResEvalRangeInt(SrcRange).ElKind of
                  revskEnum, revskInt:
                    ; // ToDo: higher precision to lower precision -> modulo
                else
                  RaiseNotSupported(El,AContext,20180424130705);
                end;
              revkRangeUInt: ;
              else
                RaiseNotSupported(El,AContext,20180424125608);
              end;
            else
              RaiseNotSupported(El,AContext,20180424125419);
            end;
          else
            RaiseNotSupported(El,AContext,20180424124814);
          end;
        finally
          ReleaseEvalValue(SrcRange);
          ReleaseEvalValue(DestRange);
        end;
        end
      else if C=TPasClassType then
        begin
        if ParamTypeEl is TPasClassType then
          case TPasClassType(Decl).ObjKind of
          okClass:
            case TPasClassType(ParamTypeEl).ObjKind of
            okClass:;
            okInterface:
              if not TPasClassType(Decl).IsExternal then
                begin
                // classtype(intfvar)  ->  rtl.intfToClass(intfvar,classtype)
                Call:=CreateCallExpression(El);
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfToClass)]);
                Call.AddArg(Result);
                Result:=Call;
                Call.AddArg(CreateReferencePathExpr(Decl,AContext));
                exit; // bsObjectChecks not needed
                end;
            else
              RaiseNotSupported(El,AContext,20180327221211,ObjKindNames[TPasClassType(ParamTypeEl).ObjKind]);
            end;
          okInterface:
            case TPasClassType(ParamTypeEl).ObjKind of
            okClass:
              begin
              case TPasClassType(Decl).InterfaceType of
              citCom:
                // IntfType(ClassInstVar)  ->  queryIntfT(ClassInstVar,IntfType)
                begin
                Call:=CreateCallExpression(El);
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfQueryIntfT)]);
                Call.AddArg(Result);
                Result:=Call;
                Call.AddArg(CreateReferencePathExpr(Decl,AContext));
                Result:=CreateIntfRef(Result,AContext,El);
                end;
              citCorba:
                // IntfType(ClassInstVar)  ->  getIntfT(ClassInstVar,IntfType)
                begin
                Call:=CreateCallExpression(El);
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfGetIntfT)]);
                Call.AddArg(Result);
                Result:=Call;
                Call.AddArg(CreateReferencePathExpr(Decl,AContext));
                end;
              else
                RaiseNotSupported(El,AContext,20180416102614,InterfaceTypeNames[TPasClassType(Decl).InterfaceType]);
              end;
              exit; // bsObjectChecks not needed
              end;
            okInterface:;
            else
              RaiseNotSupported(El,AContext,20180327221233,ObjKindNames[TPasClassType(ParamTypeEl).ObjKind]);
            end;
          else
            RaiseNotSupported(El,AContext,20180327221130,ObjKindNames[TPasClassType(Decl).ObjKind]);
          end;
        end;

      if bsObjectChecks in AContext.ScannerBoolSwitches then
        begin
        if (C=TPasClassType)
            or (C=TPasClassOfType) then
          begin
          // TObject(param) -> rtl.asExt(param,type,mode)
          if C=TPasClassOfType then
            aClassTypeEl:=aResolver.ResolveAliasType(TPasClassOfType(Decl).DestType) as TPasClassType
          else
            aClassTypeEl:=TPasClassType(Decl);
          aName:=CreateReferencePath(aClassTypeEl,AContext,rpkPathAndName);
          Call:=CreateCallExpression(El);
          Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnAsExt)]);
          Call.AddArg(Result);
          Call.AddArg(CreatePrimitiveDotExpr(aName,Value));
          if aClassTypeEl.IsExternal then
          else if C=TPasClassOfType then
            Call.AddArg(CreateLiteralNumber(Value,IsExtModePasClass))
          else
            Call.AddArg(CreateLiteralNumber(Value,IsExtModePasClassInstance));
          Result:=Call;
          end;
        end
      else if (ParamResolved.BaseType=btCustom)
          and (ParamTypeEl.CustomData is TResElDataPas2JSBaseType) then
        begin
        JSBaseType:=TResElDataPas2JSBaseType(ParamTypeEl.CustomData).JSBaseType;
        if JSBaseType=pbtJSValue then
          begin
          if (C=TPasClassType)
              or (C=TPasClassOfType)
              or (C=TPasRecordType) then
            begin
            // TObject(jsvalue)  ->  rtl.getObject(jsvalue)
            Call:=CreateCallExpression(El);
            Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnGetObject)]);
            Call.AddArg(Result);
            Result:=Call;
            end;
          end;
        end;

      exit;
      end
    else if C.InheritsFrom(TPasVariable) then
      begin
      aResolver.ComputeElement(Decl,DeclResolved,[rcType]);
      if DeclResolved.LoTypeEl is TPasProcedureType then
        // e.g. OnClick()
        TargetProcType:=TPasProcedureType(DeclResolved.LoTypeEl)
      else
        RaiseNotSupported(El,AContext,20170217115244);
      end
    else if (C=TPasArgument) then
      begin
      aResolver.ComputeElement(Decl,DeclResolved,[rcType]);
      if DeclResolved.LoTypeEl is TPasProcedureType then
        TargetProcType:=TPasProcedureType(DeclResolved.LoTypeEl)
      else
        RaiseNotSupported(El,AContext,20170328224020);
      end
    else if (C=TPasProcedureType)
        or (C=TPasFunctionType) then
      begin
      aResolver.ComputeElement(Value,ValueResolved,[rcNoImplicitProc]);
      if (ValueResolved.IdentEl is TPasType)
          and (aResolver.ResolveAliasType(TPasType(ValueResolved.IdentEl)) is TPasProcedureType) then
        begin
        // type cast to proc type
        Param:=El.Params[0];
        Result:=ConvertExpression(Param,AContext);
        exit;
        end
      else
        begin
        // calling proc var
        TargetProcType:=TPasProcedureType(Decl);
        end;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertFuncParams El=',GetObjName(El),' Decl=',GetObjName(Decl));
      {$ENDIF}
      RaiseNotSupported(El,AContext,20170215114337);
      end;
    if [rrfNewInstance,rrfFreeInstance]*Ref.Flags<>[] then
      begin
      // call constructor, destructor
      CreateFreeOrNewInstanceCall(Ref);
      end;
    end;

  // BEWARE: TargetProcType can be nil, if called without resolver

  NeedIntfRef:=false;
  if (TargetProcType is TPasFunctionType) and (aResolver<>nil) then
    begin
    if aResolver.IsInterfaceType(TPasFunctionType(TargetProcType).ResultEl.ResultType,citCom)
        and not TargetProcType.IsAsync then
      NeedIntfRef:=true;
    end;

  if Call=nil then
    begin
    Call:=CreateCallExpression(El);
    Elements:=Call.Args.Elements;
    end;
  OldAccess:=AContext.Access;
  try
    AContext.Access:=caRead;
    if Call.Args=nil then
      begin
      // append ()
      Call.Args:=TJSArguments(CreateElement(TJSArguments,El));
      Elements:=Call.Args.Elements;
      end
    else if Elements=nil then
      RaiseInconsistency(20180720154413,El);

    if Call.Expr=nil then
      begin
      if (coShortRefGlobals in Options)
          and aResolver.IsSpecializedNonStaticMethod(TargetProcType) then
        CreateShortRefApply(Value,TargetProcType)
      else if DotBin<>nil then
        Call.Expr:=ConvertSubIdentExprCustom(DotBin,AContext)
      else
        Call.Expr:=ConvertExpression(Value,AContext);
      end;
    //if Call.Expr is TPrimitiveExpr then
    //  writeln('TPasToJSConverter.ConvertFuncParams ',TPrimitiveExpr(Call.Expr).GetDeclaration(true));
    CreateProcedureCallArgs(Elements,El,TargetProcType,AContext);
    CallArgs:=Call.Args;

    if (Elements.Count=0)
        and (CallArgs.Elements.Count>0) then
      begin
      // for example: rrfNewInstance
      LastArg:=CallArgs.Elements[CallArgs.Elements.Count-1];
      if not (LastArg.Expr is TJSArrayLiteral) then
        RaiseNotSupported(El,AContext,20180720161317);
      JsArrLit:=TJSArrayLiteral(LastArg.Expr);
      if JsArrLit.Elements<>Elements then
        RaiseNotSupported(El,AContext,20180720161324);
      LastArg.Free;
      end;

    if CallArgs.Elements.Count=0 then
      begin
      CallArgs.Free;
      Call.Args:=nil;
      end;
    if NeedIntfRef then
      // $ir.ref(id,path.fnname())
      Call:=CreateIntfRef(Call,AContext,El);

    Result:=Call;
  finally
    AContext.Access:=OldAccess;
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertExternalConstructor(Left: TPasExpr;
  Ref: TResolvedReference; ParamsExpr: TParamsExpr; AContext: TConvertContext
  ): TJSElement;
var
  Proc: TPasConstructor;
  ExtName: String;
  NewExpr: TJSNewMemberExpression;
  LeftResolved: TPasResolverResult;
  OldAccess: TCtxAccess;
  ExtNameEl: TJSElement;
  WithData: TPas2JSWithExprScope;
  PosEl: TPasElement;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  NewExpr:=nil;
  ExtName:='';
  ExtNameEl:=nil;
  try
    Proc:=Ref.Declaration as TPasConstructor;
    PosEl:=Ref.Element;

    if CompareText(Proc.Name,'new')=0 then
      begin
      if Proc.LibrarySymbolName<>nil then
        begin
        ExtName:=ComputeConstString(Proc.LibrarySymbolName,AContext,true);
        if not SameText(ExtName,'new') then
          ExtNameEl:=CreatePrimitiveDotExpr(ExtName,PosEl);
        end;
      if (ExtNameEl=nil) and (Left<>nil) then
        begin
        if aResolver<>nil then
          begin
          aResolver.ComputeElement(Left,LeftResolved,[]);
          if LeftResolved.BaseType=btModule then
            begin
            // e.g. Unit.TExtA
            // ExtName is global -> omit unit
            Left:=nil;
            end
          else ;
          end;
        if Left<>nil then
          begin
          // convert left side
          OldAccess:=AContext.Access;
          AContext.Access:=caRead;
          ExtNameEl:=ConvertExpression(Left,AContext);
          AContext.Access:=OldAccess;
          end;
        end;
      if ExtNameEl=nil then
        begin
        if Ref.WithExprScope<>nil then
          begin
          // using local WITH var
          WithData:=Ref.WithExprScope as TPas2JSWithExprScope;
          ExtName:=WithData.WithVarName;
          if ExtName='' then
            RaiseNotSupported(ParamsExpr,AContext,20190209092049);
          end
        else
          // use external class name
          ExtName:=(Proc.Parent as TPasClassType).ExternalName;
        if ExtName='' then
          DoError(20180511163944,nJSNewNotSupported,sJSNewNotSupported,[],ParamsExpr);
        ExtNameEl:=CreatePrimitiveDotExpr(ExtName,PosEl);
        end;
      end
    else
      begin
      // external constructor ProcName
      ExtName:='';
      if aResolver<>nil then
        ExtName:=aResolver.ComputeConstString(Proc.LibrarySymbolName,true,true);
      if ExtName='{}' then
        begin
        // external constructor {} -> "{}"
        Result:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,PosEl));
        exit;
        end;
      // external constructor ProcName -> "new ExtA.ProcName()"
      ExtNameEl:=CreateReferencePathExpr(Proc,AContext,true);
      end;

    NewExpr:=TJSNewMemberExpression(CreateElement(TJSNewMemberExpression,PosEl));
    NewExpr.MExpr:=ExtNameEl;
    ExtNameEl:=nil;
    NewExpr.Args:=TJSArguments(CreateElement(TJSArguments,PosEl));
    if ParamsExpr<>nil then
      CreateProcedureCallArgs(NewExpr.Args.Elements,ParamsExpr,Proc.ProcType,AContext);
    Result:=NewExpr;
    NewExpr:=nil;
  finally
    ExtNameEl.Free;
    NewExpr.Free;
  end;
end;

function TPasToJSConverter.ConvertTObjectFree_Bin(Bin: TBinaryExpr;
  NameExpr: TPasExpr; AContext: TConvertContext): TJSElement;

  function CreateCallRTLFree(Obj, Prop: TJSElement): TJSElement;
  // create "rtl.free(obj,prop)"
  var
    Call: TJSCallExpression;
  begin
    Call:=CreateCallExpression(Bin.right);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnFreeVar)]);
    Call.Args.AddElement(Obj);
    Call.Args.AddElement(Prop);
    Result:=Call;
  end;

var
  LeftJS, Obj, Prop, Getter, Setter: TJSElement;
  DotExpr: TJSDotMemberExpression;
  BracketJS: TJSBracketMemberExpression;
  aName: TJSString;
  Call: TJSCallExpression;
  AssignContext: TAssignContext;
begin
  Result:=nil;

  LeftJS:=ConvertExpression(Bin.left,AContext);
  try
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertTObjectFree_Bin ',GetObjName(LeftJS));
    {$ENDIF}

    if LeftJS is TJSPrimaryExpressionIdent then
      begin
      aName:=TJSPrimaryExpressionIdent(LeftJS).Name;
      if Pos('.',aName)>0 then
        RaiseInconsistency(20170516173832,Bin.left);
      // v.free
      // -> v=rtl.freeLoc(v);
      Getter:=LeftJS;
      Setter:=ClonePrimaryExpression(TJSPrimaryExpressionIdent(LeftJS),Bin.left);
      Result:=CreateCallRTLFreeLoc(Setter,Getter,NameExpr);
      end
    else if LeftJS is TJSDotMemberExpression then
      begin
      // obj.prop.free
      // ->  rtl.free(obj,"prop");
      DotExpr:=TJSDotMemberExpression(LeftJS);
      Obj:=DotExpr.MExpr;
      DotExpr.MExpr:=nil;
      Prop:=CreateLiteralJSString(Bin.right,DotExpr.Name);
      FreeAndNil(LeftJS);
      Result:=CreateCallRTLFree(Obj,Prop);
      end
    else if LeftJS is TJSBracketMemberExpression then
      begin
      // obj[prop].free
      // ->  rtl.free(obj,prop);
      BracketJS:=TJSBracketMemberExpression(LeftJS);
      Obj:=BracketJS.MExpr;
      BracketJS.MExpr:=nil;
      Prop:=BracketJS.Name;
      BracketJS.Name:=nil;
      FreeAndNil(LeftJS);
      Result:=CreateCallRTLFree(Obj,Prop);
      end
    else if LeftJS is TJSCallExpression then
      begin
      // getter().free
      // -> setter(rtl.freeLoc(getter()))
      AssignContext:=TAssignContext.Create(Bin.Left,nil,AContext);
      try
        Call:=CreateCallExpression(Bin.Left);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnFreeLocalVar)]);
        Call.Args.AddElement(LeftJS);
        LeftJS:=nil;
        AssignContext.RightSide:=Call;
        AContext.Resolver.ComputeElement(Bin.Left,AssignContext.LeftResolved,[rcNoImplicitProc]);
        AssignContext.RightResolved:=AssignContext.LeftResolved;
        Result:=CreateAssignStatement(Bin.Left,AssignContext);
      finally
        AssignContext.RightSide.Free;
        AssignContext.Free;
      end;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertTObjectFree_Bin ',GetObjName(LeftJS));
      {$ENDIF}
      RaiseNotSupported(Bin.left,AContext,20170516164659,'invalid scope for Free');
      end;
  finally
    if Result=nil then
      LeftJS.Free;
  end;
end;

function TPasToJSConverter.ConvertTObjectFree_With(NameExpr: TPasExpr;
  AContext: TConvertContext): TJSElement;
var
  WithExprScope: TPas2JSWithExprScope;
  Getter, Setter: TJSElement;
begin
  Result:=nil;
  WithExprScope:=TResolvedReference(NameExpr.CustomData).WithExprScope as TPas2JSWithExprScope;
  if WithExprScope=nil then
    RaiseInconsistency(20181027133210,NameExpr);
  if AContext.Resolver.GetNewInstanceExpr(WithExprScope.Expr)<>nil then
    begin
    // "with TSomeClass.Create do Free"
    // -> "$with1=rtl.freeLoc($with1);
    if WithExprScope.WithVarName='' then
     RaiseNotSupported(NameExpr,AContext,20190209092220);
    Getter:=CreatePrimitiveDotExpr(WithExprScope.WithVarName,WithExprScope.Expr);
    Setter:=CreatePrimitiveDotExpr(WithExprScope.WithVarName,WithExprScope.Expr);
    Result:=CreateCallRTLFreeLoc(Setter,Getter,NameExpr);
    exit;
    end;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertTObjectFree_With With=',GetObjName(WithExprScope.Expr));
  {$ENDIF}
  RaiseInconsistency(20170517092248,NameExpr);
end;

function TPasToJSConverter.ConvertTypeCastToBaseType(El: TParamsExpr;
  AContext: TConvertContext; ToBaseTypeData: TResElDataBaseType): TJSElement;
var
  to_bt: TResolverBaseType;
  Param: TPasExpr;
  ParamResolved: TPasResolverResult;
  JSBaseType: TPas2jsBaseType;
  JSBaseTypeData: TResElDataPas2JSBaseType;

  function IsParamPas2JSBaseType: boolean;
  var
    TypeEl: TPasType;
  begin
    if ParamResolved.BaseType<>btCustom then exit(false);
    TypeEl:=ParamResolved.LoTypeEl;
    if TypeEl.ClassType<>TPasUnresolvedSymbolRef then exit(false);
    if not (TypeEl.CustomData is TResElDataPas2JSBaseType) then exit(false);
    Result:=true;
    JSBaseTypeData:=TResElDataPas2JSBaseType(TypeEl.CustomData);
    JSBaseType:=JSBaseTypeData.JSBaseType;
  end;

var
  NotEqual: TJSEqualityExpressionNE;
  CondExpr: TJSConditionalExpression;
  Call: TJSCallExpression;
  NotExpr: TJSUnaryNotExpression;
  AddExpr: TJSAdditiveExpressionPlus;
  Int: TMaxPrecInt;
  aResolver: TPas2JSResolver;
  from_bt: TResolverBaseType;
  FromTypeEl: TPasType;
  ElTypeResolved: TPasResolverResult;
begin
  Result:=nil;
  Param:=El.Params[0];
  aResolver:=AContext.Resolver;
  aResolver.ComputeElement(Param,ParamResolved,[]);
  JSBaseTypeData:=nil;
  JSBaseType:=pbtNone;

  from_bt:=ParamResolved.BaseType;
  FromTypeEl:=ParamResolved.LoTypeEl;
  if from_bt=btRange then
    begin
    from_bt:=ParamResolved.SubType;
    aResolver.ComputeElement(TPasRangeType(FromTypeEl).RangeExpr.left,ElTypeResolved,[rcConstant]);
    FromTypeEl:=ElTypeResolved.LoTypeEl;
    end;
  to_bt:=ToBaseTypeData.BaseType;
  if from_bt=to_bt then
    begin
    Result:=ConvertExpression(Param,AContext);
    exit;
    end;

  if to_bt in btAllJSInteger then
    begin
    if from_bt in btAllJSInteger then
      begin
      // integer to integer -> value
      Result:=ConvertExpression(Param,AContext);
      Result:=ConvertIntToInt(Result,from_bt,to_bt,El,AContext);
      exit;
      end
    else if from_bt in btAllJSBooleans then
      begin
      // boolean to integer -> value?1:0
      Result:=ConvertExpression(Param,AContext);
      // Note: convert Param first in case it raises an exception
      CondExpr:=TJSConditionalExpression(CreateElement(TJSConditionalExpression,El));
      CondExpr.A:=Result;
      if to_bt=btCurrency then
        CondExpr.B:=CreateLiteralNumber(El,10000)
      else
        CondExpr.B:=CreateLiteralNumber(El,1);
      CondExpr.C:=CreateLiteralNumber(El,0);
      Result:=CondExpr;
      exit;
      end
    else if from_bt in btAllJSChars then
      begin
      // char to integer
      Result:=ConvertExpression(Param,AContext);
      Result:=ConvertCharToInt(Result,El,AContext);
      Result:=ConvertIntToInt(Result,btWord,to_bt,El,AContext);
      exit;
      end
    else if from_bt=btContext then
      begin
      if FromTypeEl.ClassType=TPasEnumType then
        begin
        // e.g. longint(TEnum) -> value
        Result:=ConvertExpression(Param,AContext);
        if to_bt=btCurrency then
          // value*10000
          Result:=CreateMulNumber(Param,Result,10000);
        exit;
        end;
      end
    else if IsParamPas2JSBaseType then
      begin
      if JSBaseType=pbtJSValue then
        begin
        // convert jsvalue to integer -> rtl.trunc(value)
        Result:=ConvertExpression(Param,AContext);
        // Note: convert Param first in case it raises an exception
        if to_bt=btCurrency then
          // jsvalue to currency -> rtl.trunc(value*10000)
          Result:=CreateMulNumber(Param,Result,10000);
        Result:=CreateTruncFloor(El,Result,true);
        exit;
        end;
      end
    else if (to_bt=btCurrency) and (from_bt in btAllJSFloats) then
      begin
      // currency(double)  ->  double*10000
      Result:=ConvertExpression(Param,AContext);
      Result:=CreateMulNumber(Param,Result,10000);
      exit;
      end;
    end
  else if to_bt in btAllJSBooleans then
    begin
    if from_bt in btAllJSBooleans then
      begin
      // boolean to boolean -> value
      Result:=ConvertExpression(Param,AContext);
      exit;
      end
    else if from_bt in btAllJSInteger then
      begin
      // integer to boolean -> value!=0
      Result:=ConvertExpression(Param,AContext);
      // Note: convert Param first in case it raises an exception
      NotEqual:=TJSEqualityExpressionNE(CreateElement(TJSEqualityExpressionNE,El));
      NotEqual.A:=Result;
      NotEqual.B:=CreateLiteralNumber(El,0);
      Result:=NotEqual;
      exit;
      end
    else if IsParamPas2JSBaseType then
      begin
      if JSBaseType=pbtJSValue then
        begin
        // convert jsvalue to boolean -> !(value==false)
        Result:=ConvertExpression(Param,AContext);
        // Note: convert Param first in case it raises an exception
        NotExpr:=TJSUnaryNotExpression(CreateElement(TJSUnaryNotExpression,El));
        NotExpr.A:=TJSEqualityExpressionEQ(CreateElement(TJSEqualityExpressionEQ,El));
        TJSEqualityExpressionEQ(NotExpr.A).A:=Result;
        TJSEqualityExpressionEQ(NotExpr.A).B:=CreateLiteralBoolean(El,false);
        Result:=NotExpr;
        exit;
        end;
      end;
    end
  else if to_bt in btAllJSFloats then
    begin
    if from_bt in (btAllJSFloats+btAllJSInteger) then
      begin
      // int to double -> value
      Result:=ConvertExpression(Param,AContext);
      if ParamResolved.BaseType=btCurrency then
        // currency to double  ->  value/10000
        Result:=CreateDivideNumber(El,Result,10000);
      exit;
      end
    else if IsParamPas2JSBaseType then
      begin
      if JSBaseType=pbtJSValue then
        begin
        // convert jsvalue to double -> rtl.getNumber(value)
        Result:=ConvertExpression(Param,AContext);
        // Note: convert Param first in case it raises an exception
        Call:=CreateCallExpression(El);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnGetNumber)]);
        Call.AddArg(Result);
        Result:=Call;
        exit;
        end;
      end;
    end
  else if to_bt in btAllJSStrings then
    begin
    if from_bt in btAllJSStringAndChars then
      begin
      // string or char to string -> value
      Result:=ConvertExpression(Param,AContext);
      exit;
      end
    else if from_bt=btPointer then
      begin
      // string(aPointer) -> value
      Result:=ConvertExpression(Param,AContext);
      exit;
      end
    else if IsParamPas2JSBaseType then
      begin
      if JSBaseType=pbtJSValue then
        begin
        // convert jsvalue to string -> ""+value
        Result:=ConvertExpression(Param,AContext);
        // Note: convert value first in case it raises an exception
        AddExpr:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,El));
        AddExpr.A:=CreateLiteralString(El,'');
        AddExpr.B:=Result;
        Result:=AddExpr;
        exit;
        end;
      end;
    end
  else if to_bt in [btChar,btWideChar] then
    begin
    if from_bt in [btChar,btWideChar] then
      begin
      // char to char
      Result:=ConvertExpression(Param,AContext);
      exit;
      end
    else if (from_bt in btAllJSInteger)
        or ((from_bt=btContext)
          and (FromTypeEl.ClassType=TPasEnumType))
        then
      begin
      // Note: convert value first in case it raises an exception
      Result:=ConvertExpression(Param,AContext);
      if IsLiteralInteger(Result,Int)
          and (Int>=0) and (Int<=$ffff) then
        begin
        FreeAndNil(Result);
        Result:=CreateLiteralJSString(Param,WideChar(Int));
        end
      else
        begin
        // char(integer) -> String.fromCharCode(integer)
        Result:=CreateCallFromCharCode(Result,El);
        end;
      exit;
      end
    else if (from_bt in (btArrayRangeTypes+[btRange]))
        or (IsParamPas2JSBaseType and (JSBaseType=pbtJSValue)) then
      begin
      // convert value to char -> rtl.getChar(value)
      // Note: convert value first in case it raises an exception
      Result:=ConvertExpression(Param,AContext);
      if IsLiteralInteger(Result,Int) then
        begin
        if (Int>=0) and (Int<=$ffff) then
          begin
          FreeAndNil(Result);
          Result:=CreateLiteralJSString(Param,WideChar(Int));
          end
        else
          begin
          // char(integer) -> String.fromCharCode(integer)
          Result:=CreateCallFromCharCode(Result,El);
          end;
        end
      else
        begin
        // convert value to char -> rtl.getChar(value)
        Call:=CreateCallExpression(El);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnGetChar)]);
        Call.AddArg(Result);
        Result:=Call;
        end;
      exit;
      end;
    end
  else if to_bt=btPointer then
    begin
    if IsParamPas2JSBaseType then
      begin
      if JSBaseType=pbtJSValue then
        begin
        // convert jsvalue to pointer -> value
        Result:=ConvertExpression(Param,AContext);
        exit;
        end;
      end
    else if from_bt in btAllJSStrings then
      begin
      // pointer(aString) -> value
      Result:=ConvertExpression(Param,AContext);
      exit;
      end
    else if from_bt=btContext then
      begin
      // convert user type/value to pointer -> value
      Result:=ConvertExpression(Param,AContext);
      exit;
      end;
    end
  else if (to_bt=btCustom) and (ToBaseTypeData is TResElDataPas2JSBaseType) then
    begin
    JSBaseType:=TResElDataPas2JSBaseType(ToBaseTypeData).JSBaseType;
    if JSBaseType=pbtJSValue then
      begin
      // type cast to jsvalue
      Result:=ConvertExpression(Param,AContext);
      exit;
      end;
    end;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertTypeCastToBaseType BaseTypeData=',aResolver.BaseTypeNames[to_bt],' ParamResolved=',GetResolverResultDbg(ParamResolved));
  {$ENDIF}
  RaiseNotSupported(El,AContext,20170325161150);
end;

function TPasToJSConverter.ConvertArrayOrSetLiteral(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  Call: TJSCallExpression;
  ArgContext: TConvertContext;

  procedure AddArg(Expr: TPasExpr);
  begin
    Call.AddArg(CreateSetLiteralElement(Expr,ArgContext));
  end;

var
  i: Integer;
  ArgEl: TPasExpr;
  aResolver: TPas2JSResolver;
  ArrayType: TPasArrayType;
begin
  if El.Kind<>pekSet then
    RaiseInconsistency(20170209112737,El);
  if AContext.Access<>caRead then
    DoError(20170209112926,nCantWriteSetLiteral,sCantWriteSetLiteral,[],El);
  aResolver:=AContext.Resolver;

  if aResolver<>nil then
    begin
    ArrayType:=aResolver.IsArrayExpr(El);
    if ArrayType<>nil then
      begin
      // array literal
      Result:=CreateArrayInit(ArrayType,El,El,AContext);
      exit;
      end;
    end;

  if length(El.Params)=0 then
    Result:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El))
  else
    begin
    Result:=nil;
    ArgContext:=AContext.GetNonDotContext;
    Call:=CreateCallExpression(El);
    try
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnSet_Create)]);
      for i:=0 to length(El.Params)-1 do
        begin
        ArgEl:=El.Params[i];
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.ConvertSetLiteral ',i,' El.Params[i]=',GetObjName(ArgEl));
        {$ENDIF}
        if (ArgEl.ClassType=TBinaryExpr) and (TBinaryExpr(ArgEl).Kind=pekRange) then
          begin
          // range -> add three parameters: null,left,right
          Call.AddArg(CreateLiteralNull(ArgEl));
          AddArg(TBinaryExpr(ArgEl).left);
          AddArg(TBinaryExpr(ArgEl).right);
          end
        else
          AddArg(ArgEl);
        end;
      Result:=Call;
    finally
      if Result=nil then
        Call.Free;
    end;
    end;
end;

function TPasToJSConverter.ConvertBuiltIn_Length(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  Arg: TJSElement;
  Param, RangeEl: TPasExpr;
  ParamResolved: TPasResolverResult;
  Ranges: TPasExprArray;
  Call: TJSCallExpression;
  RgLen: TMaxPrecInt;
begin
  Result:=nil;
  Param:=El.Params[0];
  AContext.Resolver.ComputeElement(Param,ParamResolved,[]);
  if ParamResolved.BaseType=btContext then
    begin
    if ParamResolved.LoTypeEl is TPasArrayType then
      begin
      Ranges:=TPasArrayType(ParamResolved.LoTypeEl).Ranges;
      if length(Ranges)>0 then
        begin
        // static array -> number literal
        if length(Ranges)>1 then
          RaiseNotSupported(El,AContext,20170223131042);
        RangeEl:=Ranges[0];
        RgLen:=AContext.Resolver.GetRangeLength(RangeEl);
        Result:=CreateLiteralNumber(El,RgLen);
        exit;
        end
      else
        begin
        // dynamic array -> rtl.length(array)
        Result:=ConvertExpression(El.Params[0],AContext);
        // Note: convert param first, it may raise an exception
        Call:=CreateCallExpression(El);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Length)]);
        Call.AddArg(Result);
        Result:=Call;
        exit;
        end;
      end;
    end;

  // default: Param.length
  Arg:=ConvertExpression(Param,AContext);
  Result:=CreateDotNameExpr(El,Arg,'length');
end;

function TPasToJSConverter.ConvertBuiltIn_SetLength(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// convert "SetLength(a,Len)" to "a = rtl.arraySetLength(a,Len)"
var
  Param0, Range: TPasExpr;
  ResolvedParam0, RangeResolved: TPasResolverResult;
  ArrayType: TPasArrayType;
  Call: TJSCallExpression;
  ValInit: TJSElement;
  AssignContext: TAssignContext;
  ElType, TypeEl: TPasType;
  i: Integer;
  aResolver: TPas2JSResolver;
  DimSize: TMaxPrecInt;
  StaticDims: TObjectList;
  Lit: TJSLiteral;
begin
  Result:=nil;
  Param0:=El.Params[0];
  if AContext.Access<>caRead then
    RaiseInconsistency(20170213213621,El);
  aResolver:=AContext.Resolver;
  aResolver.ComputeElement(Param0,ResolvedParam0,[rcNoImplicitProc]);
  {$IFDEF VerbosePasResolver}
  writeln('TPasToJSConverter.ConvertBuiltInSetLength ',GetResolverResultDbg(ResolvedParam0));
  {$ENDIF}
  TypeEl:=ResolvedParam0.LoTypeEl;
  if TypeEl is TPasArrayType then
    begin
    // SetLength(AnArray,dim1,dim2,...)
    ArrayType:=TPasArrayType(TypeEl);
    {$IFDEF VerbosePasResolver}
    writeln('TPasToJSConverter.ConvertBuiltInSetLength array');
    {$ENDIF}

    // ->  AnArray = rtl.setArrayLength(AnArray,defaultvalue,dim1,dim2,...)
    AssignContext:=TAssignContext.Create(El,nil,AContext);
    StaticDims:=nil;
    try
      aResolver.ComputeElement(Param0,AssignContext.LeftResolved,[rcNoImplicitProc]);
      AssignContext.RightResolved:=ResolvedParam0;

      // create right side
      // rtl.setArrayLength()
      Call:=CreateCallExpression(El);
      AssignContext.RightSide:=Call;
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_SetLength)]);
      // 1st param: AnArray
      Call.AddArg(ConvertExpression(Param0,AContext));
      // 2nd param: default value
      for i:=3 to length(El.Params) do
        begin
        ElType:=aResolver.ResolveAliasType(aResolver.GetArrayElType(ArrayType));
        ArrayType:=ElType as TPasArrayType;
        end;
      ElType:=aResolver.ResolveAliasType(aResolver.GetArrayElType(ArrayType));
      while (ElType.ClassType=TPasArrayType) and (length(TPasArrayType(ElType).Ranges)>0) do
        begin
        // array of static array, Note: setlength reallocs static arrays
        ArrayType:=ElType as TPasArrayType;
        for i:=0 to length(ArrayType.Ranges)-1 do
          begin
          Range:=ArrayType.Ranges[i];
          // compute size of this dimension
          DimSize:=aResolver.GetRangeLength(Range);
          if DimSize=0 then
            begin
            aResolver.ComputeElement(Range,RangeResolved,[rcConstant]);
            RaiseNotSupported(Range,AContext,20190614171520,GetResolverResultDbg(RangeResolved));
            end;
          Lit:=CreateLiteralNumber(El,DimSize);
          if StaticDims=nil then
            StaticDims:=TObjectList.Create(true);
          StaticDims.Add(Lit);
          end;
        ElType:=aResolver.ResolveAliasType(aResolver.GetArrayElType(ArrayType));
        end;
      if ElType.ClassType=TPasRecordType then
        ValInit:=CreateReferencePathExpr(ElType,AContext)
      else
        ValInit:=CreateValInit(ElType,nil,Param0,AContext);
      Call.AddArg(ValInit);
      // add params: dim1, dim2, ...
      for i:=1 to length(El.Params)-1 do
        Call.AddArg(ConvertExpression(El.Params[i],AContext));
      if StaticDims<>nil then
        begin
        Call.AddArg(CreateLiteralJSString(El,'s'));
        for i:=0 to StaticDims.Count-1 do
          Call.AddArg(TJSElement(StaticDims[i]));
        StaticDims.OwnsObjects:=false;
        end;

      // create left side:  array =
      Result:=CreateAssignStatement(Param0,AssignContext);
    finally
      AssignContext.RightSide.Free;
      AssignContext.Free;
      StaticDims.Free;
    end;
    end
  else if ResolvedParam0.BaseType=btString then
    begin
    // convert "SetLength(astring,NewLen);" to "astring = rtl.strSetLength(astring,NewLen);"
    {$IFDEF VerbosePasResolver}
    writeln('TPasToJSConverter.ConvertBuiltInSetLength string');
    {$ENDIF}
    AssignContext:=TAssignContext.Create(El,nil,AContext);
    try
      aResolver.ComputeElement(Param0,AssignContext.LeftResolved,[rcNoImplicitProc]);
      AssignContext.RightResolved:=AssignContext.LeftResolved;

      // create right side  rtl.strSetLength(aString,NewLen)
      Call:=CreateCallExpression(El);
      AssignContext.RightSide:=Call;
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnStringSetLength)]);
      Call.AddArg(ConvertExpression(Param0,AContext));
      Call.AddArg(ConvertExpression(El.Params[1],AContext));

      Result:=CreateAssignStatement(Param0,AssignContext);
    finally
      AssignContext.RightSide.Free;
      AssignContext.Free;
    end;
    end
  else
    RaiseNotSupported(El.Value,AContext,20170130141026,'setlength '+GetResolverResultDbg(ResolvedParam0));
end;

function TPasToJSConverter.ConvertBuiltIn_ExcludeInclude(El: TParamsExpr;
  AContext: TConvertContext; IsInclude: boolean): TJSElement;
// convert "Include(aSet,Enum)" to "aSet=rtl.includeSet(aSet,Enum)"
var
  Call: TJSCallExpression;
  Param0: TPasExpr;
  AssignContext: TAssignContext;
  FunName: String;
begin
  Result:=nil;
  Param0:=El.Params[0];
  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    AContext.Resolver.ComputeElement(Param0,AssignContext.LeftResolved,[rcNoImplicitProc]);
    AssignContext.RightResolved:=AssignContext.LeftResolved;

    // create right side  rtl.includeSet(aSet,Enum)
    Call:=CreateCallExpression(El);
    AssignContext.RightSide:=Call;
    if IsInclude then
      FunName:=GetBIName(pbifnSet_Include)
    else
      FunName:=GetBIName(pbifnSet_Exclude);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),FunName]);
    Call.AddArg(ConvertExpression(Param0,AContext));
    Call.AddArg(ConvertExpression(El.Params[1],AContext));

    Result:=CreateAssignStatement(Param0,AssignContext);
  finally
    AssignContext.RightSide.Free;
    AssignContext.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltInContinue(El: TPasExpr;
  AContext: TConvertContext): TJSElement;
begin
  if AContext=nil then;
  Result:=TJSContinueStatement(CreateElement(TJSContinueStatement,El));
end;

function TPasToJSConverter.ConvertBuiltInBreak(El: TPasExpr;
  AContext: TConvertContext): TJSElement;
begin
  if AContext=nil then;
  Result:=TJSBreakStatement(CreateElement(TJSBreakStatement,El));
end;

function TPasToJSConverter.ConvertBuiltIn_Exit(El: TPasExpr;
  AContext: TConvertContext): TJSElement;
// convert "exit;" -> in a function: "return result;"  in a procedure: "return;"
// convert "exit(param);" -> "return param;"
var
  ParentEl: TPasElement;
  ImplProcScope: TPas2JSProcedureScope;
  ResultVarName: String;
  FuncContext: TFunctionContext;
  AssignSt: TJSSimpleAssignStatement;
  St: TJSStatementList;
  ImplProc, DeclProc: TPasProcedure;
  ImplTry: TPasImplTry;
  ResultIsRead: Boolean;
  ResultEl: TPasResultElement;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertBuiltIn_Exit ',GetObjName(El));
  {$ENDIF}
  ParentEl:=El.Parent;
  while (ParentEl<>nil) and not (ParentEl is TPasProcedure) do
    ParentEl:=ParentEl.Parent;
  // ParentEl can be nil, when exit is in program begin block
  ImplProc:=TPasProcedure(ParentEl);
  ResultVarName:='';
  if ImplProc<>nil then
    begin
    ImplProcScope:=ImplProc.CustomData as TPas2JSProcedureScope;
    if ImplProc.ProcType is TPasFunctionType then
      begin
      ResultVarName:=ImplProcScope.ResultVarName; // ResultVarName needs ImplProc
      if ResultVarName='' then
        ResultVarName:=ResolverResultVar;
      end;
    end;
  Result:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
  if (El is TParamsExpr) and (length(TParamsExpr(El).Params)>0) then
    begin
    // with parameter, e.g. "exit(param);"
    ResultIsRead:=false;
    if (ResultVarName<>'') then
      begin
      DeclProc:=ImplProcScope.DeclarationProc;
      if DeclProc=nil then
        DeclProc:=ImplProc; // Note: references refer to ResultEl of DeclProc
      ResultEl:=TPasFunctionType(DeclProc.ProcType).ResultEl;
      ParentEl:=El.Parent;
      while (ParentEl<>ImplProc) do
        begin
        if ParentEl is TPasImplTry then
          begin
          ImplTry:=TPasImplTry(ParentEl);
          if ImplTry.FinallyExcept is TPasImplTryFinally then
            begin
            if AContext.Resolver.ImplBlockReadsDecl(ImplTry.FinallyExcept,ResultEl) then
              begin
              ResultIsRead:=true;
              break;
              end;
            end;
          end;
        ParentEl:=ParentEl.Parent;
        end;
      end;

    if ResultIsRead then
      begin
      // create "Result = param; return Result;"
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AssignSt.LHS:=CreatePrimitiveDotExpr(ResultVarName,El);
      AssignSt.Expr:=ConvertExpression(TParamsExpr(El).Params[0],AContext);
      TJSReturnStatement(Result).Expr:=CreatePrimitiveDotExpr(ResultVarName,El);
      St:=TJSStatementList(CreateElement(TJSStatementList,El));
      St.A:=AssignSt;
      St.B:=Result;
      Result:=St;
      end
    else
      begin
      // create "return param;"
      TJSReturnStatement(Result).Expr:=ConvertExpression(TParamsExpr(El).Params[0],AContext);
      end;
    end
  else
    begin
    // without parameter
    if (ResultVarName<>'') then
      begin
      // in a function, "return Result;"
      TJSReturnStatement(Result).Expr:=CreatePrimitiveDotExpr(ResultVarName,El);
      end
    else
      ; // in a procedure, "return;" which means "return undefined;"
    end;

  FuncContext:=AContext.GetFunctionContext;
  if (FuncContext<>nil) and FuncContext.ResultNeedsIntfRelease then
    begin
    // add "$ok = true;"
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
    AssignSt.LHS:=CreatePrimitiveDotExpr(GetBIName(pbivnProcOk),El);
    AssignSt.Expr:=CreateLiteralBoolean(El,true);
    St:=TJSStatementList(CreateElement(TJSStatementList,El));
    St.A:=AssignSt;
    St.B:=Result;
    Result:=St;
    end;
end;

function TPasToJSConverter.ConvertBuiltIn_IncDec(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
{ inc(a) or inc(a,b)
 if a is a variable:
   convert inc(a,b) to a+=b
 if a is a var/out arg:
   convert inc(a,b) to a.set(a.get+b)
 if a is a property
   Getter: field, procedure
 if a is an indexed-property
   Getter: field, procedure
 if a is a property with index-specifier
   Getter: field, procedure
}
var
  AssignSt: TJSAssignStatement;
  Expr, SrcEl: TPasExpr;
  ExprResolved: TPasResolverResult;
  ExprArg: TPasArgument;
  LHS, ValueJS: TJSElement;
  Call: TJSCallExpression;
  IsInc: Boolean;
  AddJS: TJSAdditiveExpression;
  AssignContext: TAssignContext;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  Expr:=aResolver.GetRightMostExpr(El.Value);
  if not (Expr is TPrimitiveExpr) then
    RaiseNotSupported(Expr,AContext,20200620113218);
  IsInc:=CompareText(TPrimitiveExpr(Expr).Value,'inc')=0;
  Expr:=El.Params[0];
  aResolver.ComputeElement(Expr,ExprResolved,[]);

  // convert value
  if length(El.Params)=1 then
    ValueJS:=CreateLiteralNumber(El,1)
  else
    ValueJS:=ConvertExpression(El.Params[1],AContext);
  SrcEl:=El.Value;

  // check target variable
  AssignSt:=nil;
  Call:=nil;
  AssignContext:=nil;
  LHS:=nil;
  try
    if ExprResolved.IdentEl is TPasArgument then
      begin
      ExprArg:=TPasArgument(ExprResolved.IdentEl);
      if ExprArg.Access in [argVar,argOut] then
        begin
        // target variable is a reference
        // -> convert inc(ref,b)  to  ref.set(ref.get()+b)
        Call:=CreateCallExpression(SrcEl);
        // create "ref.set"
        Call.Expr:=CreateDotNameExpr(SrcEl,
          CreateIdentifierExpr(ExprResolved.IdentEl,AContext),
          TempRefObjSetterName);
        // create "+"
        if IsInc then
          AddJS:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,SrcEl))
        else
          AddJS:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,SrcEl));
        Call.AddArg(AddJS);
        // create "ref.get()"
        AddJS.A:=TJSCallExpression(CreateElement(TJSCallExpression,SrcEl));
        TJSCallExpression(AddJS.A).Expr:=CreateDotNameExpr(SrcEl,
          CreateIdentifierExpr(ExprResolved.IdentEl,AContext),
          TJSString(TempRefObjGetterName));
        // add "b"
        AddJS.B:=ValueJS;
        ValueJS:=nil;

        Result:=Call;
        exit;
        end;
      end
    else if ExprResolved.IdentEl is TPasProperty then
      begin
      RaiseNotSupported(Expr,AContext,20170501151316);
      end;

    // inc(a,b)  ->  a = a+b  or setter(getter()+b)
    AssignContext:=TAssignContext.Create(Expr,nil,AContext);
    aResolver.ComputeElement(Expr,AssignContext.LeftResolved,[rcNoImplicitProc]);
    SetResolverValueExpr(AssignContext.RightResolved,
      AssignContext.LeftResolved.BaseType,AssignContext.LeftResolved.LoTypeEl,
      AssignContext.LeftResolved.HiTypeEl,Expr,[rrfReadable]);
    AssignContext.RightSide:=ValueJS;
    ValueJS:=nil;
    LHS:=ConvertExpression(Expr,AssignContext);

    if AssignContext.Call<>nil then
      begin
      // left side is a Setter -> RightSide was already inserted as parameter
      RaiseNotSupported(El,AContext,20181101154351);
      end
    else
      begin
      // left side is a variable
      if AssignContext.RightSide=nil then
        RaiseInconsistency(20180622211919,El);
      end;

    // convert inc(avar,b)  to  a+=b
    if IsInc then
      AssignSt:=TJSAddEqAssignStatement(CreateElement(TJSAddEqAssignStatement,SrcEl))
    else
      AssignSt:=TJSSubEqAssignStatement(CreateElement(TJSSubEqAssignStatement,SrcEl));

    AssignSt.LHS:=LHS;
    LHS:=nil;
    AssignSt.Expr:=AssignContext.RightSide;
    AssignContext.RightSide:=nil;
    Result:=AssignSt;
  finally
    ValueJS.Free;
    if Result=nil then
      begin
      AssignSt.Free;
      Call.Free;
      LHS.Free;
      end;
    if AssignContext<>nil then
      begin
      AssignContext.RightSide.Free;
      AssignContext.Free;
      end;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_Assigned(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  NE: TJSEqualityExpressionNE;
  Param: TPasExpr;
  ParamResolved: TPasResolverResult;
  C: TClass;
  GT: TJSRelationalExpressionGT;
  Call: TJSCallExpression;
begin
  Result:=nil;
  if AContext.Resolver=nil then
    RaiseInconsistency(20170210105235,El);
  Param:=El.Params[0];
  AContext.Resolver.ComputeElement(Param,ParamResolved,[rcNoImplicitProcType]);
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertBuiltInAssigned ParamResolved=',GetResolverResultDbg(ParamResolved));
  {$ENDIF}
  if ParamResolved.BaseType=btPointer then
    begin
    // convert Assigned(value)  ->  value!=null
    Result:=ConvertExpression(Param,AContext);
    // Note: convert Param first, it may raise an exception
    NE:=TJSEqualityExpressionNE(CreateElement(TJSEqualityExpressionNE,El));
    NE.A:=Result;
    NE.B:=CreateLiteralNull(El);
    Result:=NE;
    end
  else if ParamResolved.BaseType=btContext then
    begin
    C:=ParamResolved.LoTypeEl.ClassType;
    if (C=TPasClassType)
        or (C=TPasClassOfType)
        or C.InheritsFrom(TPasProcedureType) then
      begin
      // convert Assigned(value)  ->  value!=null
      Result:=ConvertExpression(Param,AContext);
      // Note: convert Param first, it may raise an exception
      NE:=TJSEqualityExpressionNE(CreateElement(TJSEqualityExpressionNE,El));
      NE.A:=Result;
      NE.B:=CreateLiteralNull(El);
      Result:=NE;
      end
    else if C=TPasArrayType then
      begin
      // convert Assigned(value)  ->  rtl.length(value)>0
      Result:=ConvertExpression(Param,AContext);
      // Note: convert Param first, it may raise an exception
      GT:=TJSRelationalExpressionGT(CreateElement(TJSRelationalExpressionGT,El));
      Call:=CreateCallExpression(El);
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Length)]);
      Call.AddArg(Result);
      GT.A:=Call;
      GT.B:=CreateLiteralNumber(El,0);
      Result:=GT;
      end
    else
      RaiseNotSupported(El,AContext,20170328124606);
    end;
end;

function TPasToJSConverter.ConvertBuiltIn_Chr(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  ParamResolved: TPasResolverResult;
  Param: TPasExpr;
begin
  Result:=nil;
  if AContext.Resolver=nil then
    RaiseInconsistency(20170325185847,El);
  Param:=El.Params[0];
  AContext.Resolver.ComputeElement(Param,ParamResolved,[]);
  if ParamResolved.BaseType in btAllJSInteger then
    begin
    // chr(integer) -> String.fromCharCode(integer)
    Result:=ConvertExpression(Param,AContext);
    // Note: convert Param first, as it might raise an exception
    Result:=CreateCallFromCharCode(Result,El);
    exit;
    end;
  DoError(20170325185906,nXExpectedButYFound,sXExpectedButYFound,['integer',
    AContext.Resolver.GetResolverResultDescription(ParamResolved)],Param);
end;

function TPasToJSConverter.ConvertBuiltIn_Ord(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;

  function CheckOrdConstant(aResolver: TPas2JSResolver; Param: TPasExpr): TJSElement;
  var
    ParamValue, OrdValue: TResEvalValue;
  begin
    Result:=nil;
    OrdValue:=nil;
    ParamValue:=aResolver.Eval(Param,[]);
    try
      if ParamValue<>nil then
        begin
        OrdValue:=aResolver.ExprEvaluator.OrdValue(ParamValue,El);
        if OrdValue<>nil then
          begin
          // ord(constant) -> constant
          Result:=ConvertConstValue(OrdValue,AContext,El);
          exit;
          end;
        end;
    finally
      ReleaseEvalValue(ParamValue);
      ReleaseEvalValue(OrdValue);
    end;
  end;

var
  ParamResolved, SubParamResolved: TPasResolverResult;
  Param, SubParam: TPasExpr;
  Call: TJSCallExpression;
  SubParams: TParamsExpr;
  SubParamJS: TJSElement;
  Minus: TJSAdditiveExpressionMinus;
  Add: TJSAdditiveExpressionPlus;
  aResolver: TPas2JSResolver;
  bt: TResolverBaseType;
  C: TClass;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  if aResolver=nil then
    RaiseInconsistency(20170210105235,El);
  Param:=El.Params[0];
  aResolver.ComputeElement(Param,ParamResolved,[]);
  bt:=ParamResolved.BaseType;
  if bt=btRange then
    bt:=ParamResolved.SubType;
  if bt in [btChar,btWideChar] then
    begin
    if Param is TParamsExpr then
      begin
      SubParams:=TParamsExpr(Param);
      if SubParams.Kind=pekArrayParams then
        begin
        // e.g. ord(something[index])
        SubParam:=SubParams.Value;
        AContext.Resolver.ComputeElement(SubParam,SubParamResolved,[]);
        if SubParamResolved.BaseType in btAllJSStrings then
          begin
          // e.g. ord(aString[index]) -> aString.charCodeAt(index-1)
          SubParamJS:=ConvertExpression(SubParam,AContext);
          // Note: convert SubParam first, as it might raise an exception
          Call:=nil;
          try
            Call:=CreateCallExpression(El);
            Call.Expr:=CreateDotNameExpr(El,SubParamJS,'charCodeAt');
            Minus:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,Param));
            Call.AddArg(Minus);
            if length(SubParams.Params)<>1 then
              RaiseInconsistency(20170405231706,El);
            Minus.A:=ConvertExpression(SubParams.Params[0],AContext);
            Minus.B:=CreateLiteralNumber(Param,1);
            Result:=Call;
          finally
            if Result=nil then
              Call.Free;
          end;
          exit;
          end;
        end;
      end
    else
      begin
      Result:=CheckOrdConstant(aResolver,Param);
      if Result<>nil then exit;
      end;
    // ord(aChar) -> aChar.charCodeAt()
    Result:=ConvertExpression(Param,AContext);
    // Note: convert Param first, as it might raise an exception
    Result:=CreateCallCharCodeAt(Result,0,El);
    exit;
    end
  else if bt in btAllJSBooleans then
    begin
    // ord(bool)
    Result:=CheckOrdConstant(aResolver,Param);
    if Result<>nil then exit;
    // ord(bool) ->  bool+0
    Result:=ConvertExpression(Param,AContext);
    // Note: convert Param first, as it might raise an exception
    Add:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,El));
    Add.A:=Result;
    Add.B:=CreateLiteralNumber(El,0);
    Result:=Add;
    exit;
    end
  else if bt in btAllJSInteger then
    begin
    // ord(integer)
    Result:=CheckOrdConstant(aResolver,Param);
    if Result<>nil then exit;
    // ord(integer) ->  integer
    Result:=ConvertExpression(Param,AContext);
    exit;
    end
  else if bt=btContext then
    begin
    C:=ParamResolved.LoTypeEl.ClassType;
    if (C=TPasEnumType) or (C=TPasRangeType) then
      begin
      // ord(enum) -> enum
      Result:=ConvertExpression(Param,AContext);
      exit;
      end;
    end;
  DoError(20170210105339,nXExpectedButYFound,sXExpectedButYFound,['enum',
    AContext.Resolver.GetResolverResultDescription(ParamResolved)],Param);
end;

function TPasToJSConverter.ConvertBuiltIn_LowHigh(El: TParamsExpr;
  AContext: TConvertContext; IsLow: boolean): TJSElement;
// low(enumtype) -> first enumvalue
// high(enumtype) -> last enumvalue
// low(set var) -> first enumvalue
// high(set var) -> last enumvalue
// low(settype) -> first enumvalue
// high(settype) -> last enumvalue
// low(array var) -> first index
// high(dynamic array) -> array.length-1
// high(static array) -> last index

  procedure CreateEnumValue(TypeEl: TPasEnumType);
  var
    EnumValue: TPasEnumValue;
  begin
    if IsLow then
      EnumValue:=TPasEnumValue(TypeEl.Values[0])
    else
      EnumValue:=TPasEnumValue(TypeEl.Values[TypeEl.Values.Count-1]);
    Result:=CreateReferencePathExpr(EnumValue,AContext);
  end;

var
  Param: TPasExpr;
  aResolver: TPas2JSResolver;
  ResolvedEl: TPasResolverResult;
  TypeEl: TPasType;
  Ranges: TPasExprArray;
  Value: TResEvalValue;
  Call: TJSCallExpression;
  MinusExpr: TJSAdditiveExpressionMinus;
  MinVal, MaxVal: TMaxPrecInt;
  bt: TResolverBaseType;
begin
  Result:=nil;
  if AContext.Resolver=nil then
    RaiseInconsistency(20170210120659,El);
  Param:=El.Params[0];
  aResolver:=AContext.Resolver;
  aResolver.ComputeElement(Param,ResolvedEl,[]);
  bt:=ResolvedEl.BaseType;
  if bt=btRange then
    bt:=ResolvedEl.SubType;
  case bt of
    btContext:
      begin
      TypeEl:=ResolvedEl.LoTypeEl;
      if TypeEl.ClassType=TPasRangeType then
        begin
        if IsLow then
          Result:=ConvertElement(TPasRangeType(TypeEl).RangeExpr.left,AContext)
        else
          Result:=ConvertElement(TPasRangeType(TypeEl).RangeExpr.right,AContext);
        exit;
        end
      else if TypeEl.ClassType=TPasEnumType then
        begin
        CreateEnumValue(TPasEnumType(TypeEl));
        exit;
        end
      else if (TypeEl.ClassType=TPasSetType) then
        begin
        if TPasSetType(TypeEl).EnumType<>nil then
          begin
          TypeEl:=TPasSetType(TypeEl).EnumType;
          CreateEnumValue(TPasEnumType(TypeEl));
          exit;
          end;
        end
      else if TypeEl.ClassType=TPasArrayType then
        begin
        Ranges:=TPasArrayType(TypeEl).Ranges;
        if IsLow then
          begin
          // low(arr)
          if length(Ranges)=0 then
            begin
            // dynamic array starts at 0
            Result:=CreateLiteralNumber(El,0);
            exit;
            end
          else
            begin
            // static array
            Value:=AContext.Resolver.EvalRangeLimit(Ranges[0],[refConst],true,El);
            if Value=nil then
              RaiseNotSupported(El,AContext,20170910160817);
            try
              Result:=ConvertConstValue(Value,AContext,Param);
            finally
              ReleaseEvalValue(Value);
            end;
            exit;
            end;
          end
        else
          begin
          // high(arr)
          if length(Ranges)=0 then
            begin
            // dynamic array -> rtl.length(Param)-1
            Result:=ConvertExpression(Param,AContext);
            // Note: convert Param first, it may raise an exception
            Call:=CreateCallExpression(El);
            Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Length)]);
            Call.AddArg(Result);
            MinusExpr:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,El));
            MinusExpr.A:=Call;
            MinusExpr.B:=CreateLiteralNumber(El,1);
            Result:=MinusExpr;
            exit;
            end
          else
            begin
            // static array
            Value:=AContext.Resolver.EvalRangeLimit(Ranges[0],[refConst],false,El);
            if Value=nil then
              RaiseNotSupported(El,AContext,20170910161555);
            try
              Result:=ConvertConstValue(Value,AContext,Param);
            finally
              ReleaseEvalValue(Value);
            end;
            exit;
            end;
          end;
        end;
      end;
    btBoolean,btByteBool,btWordBool,btLongBool:
      begin
      if IsLow then
        Result:=CreateLiteralBoolean(El,LowJSBoolean)
      else
        Result:=CreateLiteralBoolean(El,HighJSBoolean);
      exit;
      end;
    btChar,
    btWideChar:
      begin
      if IsLow then
        Result:=CreateLiteralJSString(El,#0)
      else
        Result:=CreateLiteralJSString(El,#$ffff);
      exit;
      end;
    btByte..btIntMax:
      begin
      TypeEl:=ResolvedEl.LoTypeEl;
      if TypeEl.ClassType=TPasUnresolvedSymbolRef then
        begin
        if TypeEl.CustomData is TResElDataBaseType then
          begin
          AContext.Resolver.GetIntegerRange(ResolvedEl.BaseType,MinVal,MaxVal);
          if IsLow then
            Result:=CreateLiteralNumber(El,MinVal)
          else
            Result:=CreateLiteralNumber(El,MaxVal);
          exit;
          end;
        end
      else if TypeEl.ClassType=TPasRangeType then
        begin
        Value:=AContext.Resolver.EvalRangeLimit(TPasRangeType(TypeEl).RangeExpr,
                                                [refConst],IsLow,El);
        try
          case Value.Kind of
          revkInt:
            Result:=CreateLiteralNumber(El,TResEvalInt(Value).Int);
          revkUInt:
            Result:=CreateLiteralNumber(El,TResEvalUInt(Value).UInt);
          else
            RaiseNotSupported(El,AContext,20170925214317);
          end;
          exit;
        finally
          ReleaseEvalValue(Value);
        end;
        end;
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertBuiltIn_LowHigh ',GetResolverResultDbg(ResolvedEl));
      {$ENDIF}
      RaiseNotSupported(El,AContext,20170925214351);
      end;
    btSet,btArrayOrSet:
      begin
      TypeEl:=ResolvedEl.LoTypeEl;
      if TypeEl.ClassType=TPasEnumType then
        begin
        CreateEnumValue(TPasEnumType(TypeEl));
        exit;
        end;
      end;
  end;
  DoError(20170210110717,nXExpectedButYFound,sXExpectedButYFound,['enum or array',
    AContext.Resolver.GetResolverResultDescription(ResolvedEl)],Param);
end;

function TPasToJSConverter.ConvertBuiltIn_PredSucc(El: TParamsExpr;
  AContext: TConvertContext; IsPred: boolean): TJSElement;
// pred(enumvalue) -> enumvalue-1
// succ(enumvalue) -> enumvalue+1
var
  ResolvedEl: TPasResolverResult;
  TypeEl: TPasType;

  procedure EnumExpected(Id: TMaxPrecInt);
  begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertBuiltIn_PredSucc ',ResolvedEl.BaseType,' ',ResolvedEl.SubType,' ',GetObjName(TypeEl));
    {$ENDIF}
    DoError(Id,nXExpectedButYFound,sXExpectedButYFound,['enum',
      AContext.Resolver.GetResolverResultDescription(ResolvedEl)],El.Params[0]);
  end;

  procedure CreateAdd(Param: TPasExpr);
  var
    V: TJSElement;
    Expr: TJSAdditiveExpression;
  begin
    V:=ConvertExpression(Param,AContext);
    if IsPred then
      // pred(int) -> Param-1
      Expr:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,El))
    else
      // succ(int) -> Param+1
      Expr:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,El));
    Expr.A:=V;
    Expr.B:=CreateLiteralNumber(El,1);
    ConvertBuiltIn_PredSucc:=Expr;
  end;

  procedure CreateSwitchBool;
  begin
    if IsPred then
      // pred(bool) -> false
      ConvertBuiltIn_PredSucc:=CreateLiteralBoolean(El,false)
    else
      // succ(bool) -> true
      ConvertBuiltIn_PredSucc:=CreateLiteralBoolean(El,true);
  end;

  procedure CreateCharPredSucc(Param: TPasExpr);
  var
    V: TJSElement;
    Call: TJSCallExpression;
    Expr: TJSAdditiveExpression;
  begin
    V:=ConvertExpression(Param,AContext);
    // V.charCodeAt()
    Call:=CreateCallCharCodeAt(V,0,El);
    if IsPred then
      // pred(V) -> V.charCodeAt-1
      Expr:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,El))
    else
      // succ(V) -> V.charCodeAt+1
      Expr:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,El));
    Expr.A:=Call;
    Expr.B:=CreateLiteralNumber(El,1);
    // String.fromCharCode(V.charCodeAt+1)
    Call:=CreateCallFromCharCode(Expr,El);
    ConvertBuiltIn_PredSucc:=Call;
  end;

var
  Param: TPasExpr;
  Value: TResEvalValue;
begin
  Result:=nil;
  if AContext.Resolver=nil then
    RaiseInconsistency(20170210120648,El);
  Param:=El.Params[0];
  AContext.Resolver.ComputeElement(Param,ResolvedEl,[]);
  TypeEl:=ResolvedEl.LoTypeEl;
  if ResolvedEl.BaseType in btAllJSInteger then
    begin
    CreateAdd(Param);
    exit;
    end
  else if ResolvedEl.BaseType in btAllJSBooleans then
    begin
    CreateSwitchBool;
    exit;
    end
  else if ResolvedEl.BaseType in btAllJSChars then
    begin
    CreateCharPredSucc(Param);
    exit;
    end
  else if ResolvedEl.BaseType=btContext then
    begin
    if TypeEl.ClassType=TPasEnumType then
      begin
      CreateAdd(Param);
      exit;
      end
    else
      EnumExpected(20180424115902);
    end
  else if ResolvedEl.BaseType=btRange then
    begin
    if ResolvedEl.SubType in btAllJSInteger then
      begin
      CreateAdd(Param);
      exit;
      end
    else if ResolvedEl.SubType in btAllJSBooleans then
      begin
      CreateAdd(Param);
      exit;
      end
    else if ResolvedEl.SubType=btContext then
      begin
      if TypeEl.ClassType=TPasRangeType then
        begin
        Value:=AContext.Resolver.EvalTypeRange(TypeEl,[refConst]);
        if Value<>nil then
          try
            case Value.Kind of
            revkRangeInt:
              case TResEvalRangeInt(Value).ElKind of
              revskEnum, revskInt:
                begin
                CreateAdd(Param);
                exit;
                end;
              revskChar:
                EnumExpected(20180424115736);
              revskBool:
                begin
                CreateSwitchBool;
                exit;
                end;
              else
                EnumExpected(20180424115959);
              end;
            revkRangeUInt:
              begin
              CreateAdd(Param);
              exit;
              end;
            else
              EnumExpected(20180424115757);
            end;
          finally
            ReleaseEvalValue(Value);
          end;
        end
      else
        EnumExpected(20180424115934);
      end;
    end;
  EnumExpected(20170210120039);
end;

function TPasToJSConverter.ConvertBuiltIn_StrProc(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// convert 'str(value,aString)' to 'aString = <string>'
// for the conversion see ConvertBuiltInStrParam
var
  AssignContext: TAssignContext;
  StrVar: TPasExpr;
  TypeEl: TPasType;
begin
  Result:=nil;
  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    StrVar:=El.Params[1];
    AContext.Resolver.ComputeElement(StrVar,AssignContext.LeftResolved,[rcNoImplicitProc]);

    // create right side
    AssignContext.RightSide:=ConvertBuiltInStrParam(El.Params[0],AContext,false,true);
    TypeEl:=AContext.Resolver.BaseTypes[btString];
    SetResolverValueExpr(AssignContext.RightResolved,btString,
      TypeEl,TypeEl,El,[rrfReadable]);

    // create 'StrVar = rightside'
    Result:=CreateAssignStatement(StrVar,AssignContext);
  finally
    AssignContext.RightSide.Free;
    AssignContext.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_StrFunc(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// convert 'str(boolean)' to '""+boolean'
// convert 'str(integer)' to '""+integer'
// convert 'str(float)' to '""+float'
// convert 'str(float:width)' to rtl.spaceLeft('""+float,width)'
// convert 'str(float:width:precision)' to 'rtl.spaceLeft(float.toFixed(precision),width)'
var
  i: Integer;
  Param: TPasExpr;
  Sum, Add: TJSElement;
  AddEl: TJSAdditiveExpressionPlus;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertBuiltInStrFunc Count=',length(El.Params));
  {$ENDIF}
  Result:=nil;
  Sum:=nil;
  Add:=nil;
  try
    for i:=0 to length(El.Params)-1 do
      begin
      Param:=El.Params[i];
      Add:=ConvertBuiltInStrParam(Param,AContext,true,i=0);
      if Sum=nil then
        Sum:=Add
      else
        begin
        AddEl:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,Param));
        AddEl.A:=Sum;
        AddEl.B:=Add;
        Sum:=AddEl;
        end;
      Add:=nil;
      end;
    Result:=Sum;
  finally
    Add.Free;
    if Result=nil then
      Sum.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltInStrParam(El: TPasExpr;
  AContext: TConvertContext; IsStrFunc, IsFirst: boolean): TJSElement;
var
  Add: TJSElement;

  procedure PrependStrLit;
  var
    PlusEl: TJSAdditiveExpressionPlus;
  begin
    PlusEl:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,El));
    PlusEl.A:=CreateLiteralString(El,'');
    PlusEl.B:=Add;
    Add:=PlusEl;
  end;

var
  aResolver: TPas2JSResolver;
  ResolvedEl, ElTypeResolved: TPasResolverResult;
  NeedStrLit: Boolean;
  Call: TJSCallExpression;
  Bracket: TJSBracketMemberExpression;
  Arg: TJSElement;
  bt: TResolverBaseType;
  TypeEl: TPasType;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  aResolver.ComputeElement(El,ResolvedEl,[]);
  Add:=nil;
  Call:=nil;
  Bracket:=nil;
  try
    NeedStrLit:=false;
    bt:=ResolvedEl.BaseType;
    if bt=btRange then
      bt:=ResolvedEl.SubType;
    if bt in (btAllJSBooleans+btAllJSInteger-[btCurrency]) then
      begin
      NeedStrLit:=true;
      Add:=ConvertExpression(El,AContext);
      end
    else if bt in (btAllJSFloats+[btCurrency]) then
      begin
      // convert to rtl.floatToStr(El,width,precision)
      Call:=CreateCallExpression(El);
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnFloatToStr)]);
      Arg:=ConvertExpression(El,AContext);
      if ResolvedEl.BaseType=btCurrency then
        Arg:=CreateDivideNumber(El,Arg,10000);
      Call.AddArg(Arg);
      if El.format1<>nil then
        Call.AddArg(ConvertExpression(El.format1,AContext));
      if El.format2<>nil then
        Call.AddArg(ConvertExpression(El.format2,AContext));
      Result:=Call;
      Call:=nil;
      exit;
      end
    else if IsStrFunc and (bt in btAllJSStringAndChars) then
      Add:=ConvertExpression(El,AContext)
    else if bt=btContext then
      begin
      TypeEl:=ResolvedEl.LoTypeEl;
      if TypeEl.ClassType=TPasRangeType then
        begin
        aResolver.ComputeElement(TPasRangeType(TypeEl).RangeExpr.left,ElTypeResolved,[rcConstant]);
        TypeEl:=ElTypeResolved.LoTypeEl;
        end;
      if TypeEl.ClassType=TPasEnumType then
        begin
        // create enumtype[enumvalue]
        Bracket:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
        Bracket.MExpr:=CreateReferencePathExpr(TPasEnumType(TypeEl),AContext);
        Bracket.Name:=ConvertExpression(El,AContext);
        Add:=Bracket;
        Bracket:=nil;
        end
      else
        RaiseNotSupported(El,AContext,20170320123827);
      end
    else
      RaiseNotSupported(El,AContext,20170320093001);

    if El.format1<>nil then
      begin
      // width -> leading spaces
      if NeedStrLit then
        PrependStrLit;
      // create 'rtl.spaceLeft(add,width)'
      Call:=CreateCallExpression(El);
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnSpaceLeft)]);
      Call.AddArg(Add);
      Add:=nil;
      Call.AddArg(ConvertExpression(El.format1,AContext));
      Add:=Call;
      Call:=nil;
      end
    else if IsFirst and NeedStrLit then
      PrependStrLit;
    Result:=Add;
  finally
    Call.Free;
    Bracket.Free;
    if Result=nil then
      Add.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_WriteStr(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// convert 'writestr(aString,v:width,p)' to 'aString = <string of v> + (<string of p>+"")'
// for the conversion see ConvertBuiltInStrParam
var
  AssignContext: TAssignContext;
  StrVar: TPasExpr;
  TypeEl: TPasType;
  JS: TJSElement;
  AddJS: TJSAdditiveExpressionPlus;
  i: Integer;
begin
  Result:=nil;
  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    StrVar:=El.Params[0];
    AContext.Resolver.ComputeElement(StrVar,AssignContext.LeftResolved,[rcNoImplicitProc]);

    // create right side
    for i:=1 to length(El.Params)-1 do
      begin
      JS:=ConvertBuiltInStrParam(El.Params[i],AContext,false,true);
      if AssignContext.RightSide=nil then
        AssignContext.RightSide:=JS
      else
        begin
        AddJS:=TJSAdditiveExpressionPlus(CreateElement(TJSAdditiveExpressionPlus,El));
        AddJS.A:=AssignContext.RightSide;
        AssignContext.RightSide:=AddJS;
        AddJS.B:=JS;
        end;
      end;

    TypeEl:=AContext.Resolver.BaseTypes[btString];
    SetResolverValueExpr(AssignContext.RightResolved,btString,
      TypeEl,TypeEl,El,[rrfReadable]);

    // create 'StrVar = rightside'
    Result:=CreateAssignStatement(StrVar,AssignContext);
  finally
    AssignContext.RightSide.Free;
    AssignContext.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_Val(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// val(const s: string; out value: valuetype; out Code: integertype)
// for enum it is converted to
//   value = rtl.valEnum(s,enumType,function(c){ Code=c; })
var
  aResolver: TPas2JSResolver;
  AssignContext: TAssignContext;
  ValueExpr, CodeExpr: TPasExpr;
  Call: TJSCallExpression;
  Params: TPasExprArray;
  EnumType: TPasEnumType;
  Fun: TJSFunctionDeclarationStatement;
  ExprResolved, ElTypeResolved: TPasResolverResult;
  ExprArg: TPasArgument;
  AssignSt: TJSSimpleAssignStatement;
  SetterArgName: String;
  ArgJS, SetExpr: TJSElement;
  bt: TResolverBaseType;
  LoTypeEl: TPasType;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  Params:=El.Params;
  Call:=nil;
  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    //
    ValueExpr:=Params[1];
    aResolver.ComputeElement(ValueExpr,AssignContext.LeftResolved,[rcNoImplicitProc]);

    // rtl.valEnum()
    Call:=CreateCallExpression(El);
    AssignContext.RightSide:=Call;
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnValEnum)]);
    // add arg string
    Call.AddArg(ConvertExpression(Params[0],AContext));
    // add arg enumtype
    bt:=AssignContext.LeftResolved.BaseType;
    if bt=btRange then
      bt:=AssignContext.LeftResolved.SubType;
    if bt=btContext then
      begin
      LoTypeEl:=AssignContext.LeftResolved.LoTypeEl;
      if LoTypeEl.ClassType=TPasRangeType then
        begin
        aResolver.ComputeElement(TPasRangeType(LoTypeEl).RangeExpr.left,ElTypeResolved,[rcConstant]);
        LoTypeEl:=ElTypeResolved.LoTypeEl;
        end;
      if LoTypeEl.ClassType=TPasEnumType then
        begin
        EnumType:=TPasEnumType(LoTypeEl);
        Call.AddArg(CreateReferencePathExpr(EnumType,AContext));
        end else
          RaiseNotSupported(Params[1],AContext,20181214145226,GetResolverResultDbg(AssignContext.LeftResolved));
      end
    else
      RaiseNotSupported(Params[1],AContext,20181214145125,GetResolverResultDbg(AssignContext.LeftResolved));
    // add arg setter for Code
    CodeExpr:=Params[2];
    AContext.Resolver.ComputeElement(CodeExpr,ExprResolved,[rcNoImplicitProc]);
    ArgJS:=nil;
    if ExprResolved.IdentEl is TPasArgument then
      begin
      ExprArg:=TPasArgument(ExprResolved.IdentEl);
      if ExprArg.Access in [argVar,argOut] then
        begin
        // add arg setter for Code: Code.set
        ArgJS:=CreateDotNameExpr(CodeExpr,
          CreateIdentifierExpr(ExprResolved.IdentEl,AContext),
          TempRefObjSetterName);
        Call.AddArg(ArgJS);
        end;
      end;
    if ArgJS=nil then
      begin
      // add arg setter for Code: function(v){ Code=v; }
      if (ExprResolved.IdentEl=nil) or (ExprResolved.IdentEl is TPasProperty) then
        RaiseNotSupported(CodeExpr,AContext,20181214154031,'property');
      Fun:=CreateFunctionSt(CodeExpr);
      ArgJS:=Fun;
      Call.AddArg(ArgJS);
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,CodeExpr));
      Fun.AFunction.Body.A:=AssignSt;
      SetExpr:=ConvertExpression(CodeExpr,AContext);
      AssignSt.LHS:=SetExpr;
      SetterArgName:=TempRefObjSetterArgName;
      FindAvailableLocalName(SetterArgName,SetExpr);
      Fun.AFunction.Params.Add(SetterArgName);
      AssignSt.Expr:=CreatePrimitiveDotExpr(SetterArgName,CodeExpr);
      end;

    // create 'ValueVar = rightside'
    Result:=CreateAssignStatement(ValueExpr,AssignContext);
  finally
    if TAssignContext<>nil then
      begin
      AssignContext.RightSide.Free;
      AssignContext.Free;
      end;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_LoHi(El: TParamsExpr;
  AContext: TConvertContext; IsLoFunc: Boolean): TJSElement;
var
  ResolvedParam: TPasResolverResult;
  Param: TPasExpr;
  Mask: LongWord;
  Shift, Digits: Integer;
  ShiftEx: TJSShiftExpression;
  AndEx: TJSBitwiseAndExpression;
begin
  Result := nil;
  if AContext.Resolver=nil then
    RaiseInconsistency(20190129102200,El);
  Param := El.Params[0];
  AContext.Resolver.ComputeElement(Param,ResolvedParam,[]);
  if not (ResolvedParam.BaseType in btAllInteger) then
    DoError(20190129121100,nXExpectedButYFound,sXExpectedButYFound,['integer type',
      AContext.Resolver.GetResolverResultDescription(ResolvedParam)],Param);
  Shift := AContext.Resolver.GetShiftAndMaskForLoHiFunc(ResolvedParam.BaseType,IsLoFunc,Mask);
  Result := ConvertExpression(Param,AContext);
  // Note: convert Param first, as it might raise an exception
  if Shift > 0 then
    begin
    if Shift=32 then
      begin
      // JS bitwise operations work only 32bit -> use division for bigger shifts
      Result:=CreateTruncFloor(El,CreateDivideNumber(El,Result,$100000000),false);
      end
    else
      begin
      ShiftEx := TJSRShiftExpression(CreateElement(TJSRShiftExpression,El));
      ShiftEx.A := Result;
      ShiftEx.B := CreateLiteralNumber(El, Shift);
      Result := ShiftEx;
      end;
    end;
  case Mask of
    $FF: Digits := 2;
    $FFFF: Digits := 4;
    $FFFFFFFF: Digits := 8;
    else { $F } Digits := 1;
  end;
  if Digits<8 then
    begin
    // & Mask
    AndEx := TJSBitwiseAndExpression(CreateElement(TJSBitwiseAndExpression,El));
    AndEx.A := Result;
    AndEx.B := CreateLiteralHexNumber(El,Mask,Digits);
    Result := AndEx;
    end
  else
    begin
    // mask to longword ->   >>> 0
    ShiftEx:=TJSURShiftExpression(CreateElement(TJSURShiftExpression,El));
    ShiftEx.A:=Result;
    ShiftEx.B:=CreateLiteralNumber(El,0);
    Result:=ShiftEx;
    end;
end;

function TPasToJSConverter.ConvertBuiltIn_ConcatArray(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// concat(array1, array2)
var
  Params: TPasExprArray;
  ParamResolved: TPasResolverResult;
  Param0, Param: TPasExpr;
  ArrayType: TPasArrayType;
  i: Integer;
  Call: TJSCallExpression;
  JS: TJSElement;
begin
  Result:=nil;
  Params:=El.Params;
  if length(Params)<1 then
    RaiseInconsistency(20170331000332,El);
  Param0:=El.Params[0];
  if length(Params)=1 then
    begin
    // concat(array1)  ->  array1
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertBuiltInConcatArray Count=',length(El.Params));
    {$ENDIF}
    Result:=CreateArrayRef(El,ConvertExpression(Param0,AContext));
    end
  else
    begin
    // concat(array1,array2,...)
    Call:=nil;
    AContext.Resolver.ComputeElement(Param0,ParamResolved,[]);
    if ParamResolved.LoTypeEl is TPasArrayType then
      begin
      ArrayType:=TPasArrayType(ParamResolved.LoTypeEl);
      Call:=CreateArrayConcat(ArrayType,El,AContext);
      end
    else if ParamResolved.BaseType=btArrayLit then
      begin
      ParamResolved.BaseType:=ParamResolved.SubType;
      ParamResolved.SubType:=btNone;
      Call:=CreateArrayConcat(ParamResolved,El,AContext);
      end;
    if Call=nil then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertBuiltIn_ConcatArray Param0Resolved=',GetResolverResultDbg(ParamResolved));
      {$ENDIF}
      RaiseNotSupported(Param0,AContext,20170331000846);
      end;
    try
      for i:=0 to length(Params)-1 do
        begin
        Param:=Params[i];
        JS:=ConvertExpression(Param,AContext);
        JS:=CreateArrayEl(Param,JS,AContext);
        Call.AddArg(JS);
        end;
      Result:=Call;
    finally
      if Result=nil then
        Call.Free;
    end;
    end;
end;

function TPasToJSConverter.ConvertBuiltIn_ConcatString(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  Params: TPasExprArray;
  A: TJSElement;
  Call: TJSCallExpression;
  i: Integer;
begin
  Params:=El.Params;
  if Length(Params)=1 then
    // concat(a) -> a
    Result:=ConvertExpression(Params[0],AContext)
  else
    begin
    // concat(a,b,c) -> a.concat(b,c)
    Result:=nil;
    A:=ConvertExpression(Params[0],AContext); // beware: might fail
    Call:=CreateCallExpression(El);
    try
      Call.Expr:=CreateDotNameExpr(Params[0],A,'concat');
      for i:=1 to length(Params)-1 do
        Call.AddArg(ConvertExpression(Params[i],AContext));
      Result:=Call;
    finally
      if Result=nil then
        Call.Free;
    end;
    end;
end;

function TPasToJSConverter.ConvertBuiltIn_CopyArray(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// convert  copy(Arr,Start,Count)
//   ->  rtl.arrayCopy(type,Arr,Start,Count)
var
  Param: TPasExpr;
  ParamResolved, ElTypeResolved: TPasResolverResult;
  C: TClass;
  TypeParam: TJSElement;
  Call: TJSCallExpression;
  ArrayType: TPasArrayType;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  Call:=nil;
  try
    Param:=El.Params[0];
    aResolver.ComputeElement(El,ParamResolved,[]);
    if (ParamResolved.BaseType=btContext)
        and (ParamResolved.LoTypeEl.ClassType=TPasArrayType) then
      begin
      ArrayType:=TPasArrayType(ParamResolved.LoTypeEl);
      aResolver.ComputeElement(aResolver.GetArrayElType(ArrayType),ElTypeResolved,[rcType]);
      end
    else if ParamResolved.BaseType=btArrayLit then
      begin
      ElTypeResolved:=ParamResolved;
      ElTypeResolved.BaseType:=ElTypeResolved.SubType;
      ElTypeResolved.SubType:=btNone;
      end;
    // rtl.arrayCopy(type,src,start,count)
    TypeParam:=nil;
    if ElTypeResolved.BaseType=btContext then
      begin
      C:=ElTypeResolved.LoTypeEl.ClassType;
      if C=TPasRecordType then
        // copy array of record
        TypeParam:=CreateReferencePathExpr(TPasRecordType(ElTypeResolved.LoTypeEl),AContext);
      end
    else if ElTypeResolved.BaseType=btSet then
      // copy array of set
      TypeParam:=CreateLiteralString(El,GetBIName(pbifnSet_Reference));
    if TypeParam=nil then
      TypeParam:=CreateLiteralNumber(El,0);
    Call:=CreateCallExpression(El);
    // rtl.arrayCopy
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Copy)]);
    // param: type
    Call.AddArg(TypeParam);
    // param: src
    Call.AddArg(ConvertExpression(Param,AContext));
    // param: start
    if length(El.Params)=1 then
      Call.AddArg(CreateLiteralNumber(El,0))
    else
      Call.AddArg(ConvertExpression(El.Params[1],AContext));
    // param: count
    if length(El.Params)>=3 then
      Call.AddArg(ConvertExpression(El.Params[2],AContext));
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;

  if El=nil then ;
  if AContext=nil then;
end;

function TPasToJSConverter.ConvertBuiltIn_InsertArray(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// procedure insert(item,var AnArray,const position)
// ->  AnArray=rtl.arrayInsert(item,AnArray,position);
var
  Call: TJSCallExpression;
  AssignSt: TJSSimpleAssignStatement;
begin
  Result:=nil;
  AssignSt:=nil;
  try
    // AnArray=
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
    AssignSt.LHS:=ConvertExpression(El.Params[1],AContext);
    Call:=CreateCallExpression(El);
    AssignSt.Expr:=Call;
    // rtl.arrayInsert
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Insert)]);
    // param: item
    Call.AddArg(ConvertExpression(El.Params[0],AContext));
    // param: AnArray
    Call.AddArg(ConvertExpression(El.Params[1],AContext));
    // param: position
    Call.AddArg(ConvertExpression(El.Params[2],AContext));
    Result:=AssignSt;
  finally
    if Result=nil then
      AssignSt.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_DeleteArray(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// proc delete(var array,const start,count)
// ->  array.splice(start,count)
var
  ArrEl: TJSElement;
  Call: TJSCallExpression;
begin
  Result:=nil;
  Call:=nil;
  try
    Call:=CreateCallExpression(El);
    ArrEl:=ConvertExpression(El.Params[0],AContext);
    Call.Expr:=CreateDotNameExpr(El,ArrEl,'splice');
    Call.AddArg(ConvertExpression(El.Params[1],AContext));
    Call.AddArg(ConvertExpression(El.Params[2],AContext));
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_TypeInfo(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  ParamResolved: TPasResolverResult;
  Param: TPasExpr;
  ResultEl: TPasResultElement;
  TypeEl: TPasType;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  Param:=El.Params[0];
  aResolver:=AContext.Resolver;
  aResolver.ComputeElement(Param,ParamResolved,[rcNoImplicitProc]);
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertBuiltIn_TypeInfo ',GetResolverResultDbg(ParamResolved));
  {$ENDIF}
  if (ParamResolved.BaseType=btProc) and (ParamResolved.IdentEl is TPasFunction) then
    begin
    // typeinfo(function) -> typeinfo(resulttype)
    ResultEl:=TPasFunction(ParamResolved.IdentEl).FuncType.ResultEl;
    aResolver.ComputeResultElement(ResultEl,ParamResolved,[]);
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertBuiltIn_TypeInfo FuncResult=',GetResolverResultDbg(ParamResolved));
    {$ENDIF}
    Include(ParamResolved.Flags,rrfReadable);
    ParamResolved.IdentEl:=ResultEl;
    end;
  TypeEl:=ResolveSimpleAliasType(ParamResolved.HiTypeEl);
  if TypeEl=nil then
    RaiseNotSupported(El,AContext,20170413001544)
  else if ParamResolved.IdentEl is TPasType then
    Result:=CreateTypeInfoRef(TPasType(ParamResolved.IdentEl),AContext,Param)
  else if (rrfReadable in ParamResolved.Flags)
      and ((TypeEl.ClassType=TPasClassType)
        or (TypeEl.ClassType=TPasClassOfType))
      and ((ParamResolved.IdentEl is TPasVariable)
        or (ParamResolved.IdentEl.ClassType=TPasArgument)
        or (ParamResolved.IdentEl.ClassType=TPasResultElement)) then
    begin
    // typeinfo(classinstance) -> classinstance.$rtti
    // typeinfo(classof) -> classof.$rtti
    Result:=ConvertExpression(Param,AContext);
    Result:=CreateDotNameExpr(El,Result,TJSString(GetBIName(pbivnRTTI)));
    end
  else
    Result:=CreateTypeInfoRef(TypeEl,AContext,Param);
end;

function TPasToJSConverter.ConvertBuiltIn_GetTypeKind(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  aResolver: TPas2JSResolver;
  Value: TResEvalValue;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  aResolver.BI_GetTypeKind_OnEval(aResolver.BuiltInProcs[bfGetTypeKind],El,[refConst],Value);
  try
    if not (Value is TResEvalEnum) then
      RaiseNotSupported(El,AContext,20200826222729,GetObjName(Value));
    Result:=CreateLiteralNumber(El,TResEvalEnum(Value).Index);
  finally
    ReleaseEvalValue(Value);
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_Assert(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// throw pas.SysUtils.EAssertionFailed.$create("Create");
// throw pas.SysUtils.EAssertionFailed.$create("Create$1",["text"]);
// throw "text"
var
  IfSt: TJSIfStatement;
  ThrowSt: TJSThrowStatement;
  ModScope: TPasModuleScope;
  aConstructor: TPasConstructor;
  Ref: TResolvedReference;
  ArrLit: TJSArrayLiteral;
  Call: TJSCallExpression;
  FunName: String;
  PosEl: TPasExpr;
begin
  Result:=nil;

  // check if assertions are enabled
  if not (bsAssertions in AContext.ScannerBoolSwitches) then
    exit;

  Ref:=nil;
  IfSt:=TJSIfStatement(CreateElement(TJSIfStatement,El));
  try
    PosEl:=El.Params[0];
    IfSt.Cond:=CreateUnaryNot(ConvertExpression(PosEl,AContext),PosEl);

    ThrowSt:=TJSThrowStatement(CreateElement(TJSThrowStatement,PosEl));
    IfSt.BTrue:=ThrowSt;

    // using sysutils.EAssertionFailed if available
    aConstructor:=nil;
    if El.CustomData is TResolvedReference then
      begin
      Ref:=TResolvedReference(El.CustomData);
      if Ref.Declaration is TPasConstructor then
        aConstructor:=TPasConstructor(Ref.Declaration);
      Ref:=nil;
      end;
    //writeln('TPasToJSConverter.ConvertBuiltIn_Assert ',GetObjName(aConstructor));
    if aConstructor<>nil then
      begin
      Ref:=TResolvedReference.Create;
      ModScope:=El.GetModule.CustomData as TPasModuleScope;
      Ref.Declaration:=ModScope.AssertClass;
      // pas.sysutils.EAssertionFailed
      FunName:=CreateReferencePath(ModScope.AssertClass,AContext,rpkPathAndName,true,Ref);
      // append .$create('Create')
      FunName:=FunName+'.'+GetBIName(pbifnClassInstanceNew);
      Call:=CreateCallExpression(PosEl);
      Call.Expr:=CreatePrimitiveDotExpr(FunName,PosEl);
      // parameter: "Create"
      Call.AddArg(CreateLiteralString(PosEl,TransformElToJSName(aConstructor,AContext)));
      ThrowSt.A:=Call;
      if length(El.Params)>1 then
        begin
        // add [msg]
        ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El.Params[1]));
        Call.AddArg(ArrLit);
        ArrLit.AddElement(ConvertExpression(El.Params[1],AContext));
        end;
      end;
    if ThrowSt.A=nil then
      begin
      // fallback: throw msg
      if length(El.Params)>1 then
        ThrowSt.A:=ConvertExpression(El.Params[1],AContext)
      else
        ThrowSt.A:=CreateLiteralJSString(El.Params[0],'assert failed');
      end;
    Result:=IfSt;
  finally
    Ref.Free;
    if Result=nil then
      IfSt.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_New(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// new(p)  ->  p=new TRecord();
var
  Param0: TPasExpr;
  ParamResolved: TPasResolverResult;
  AssignContext: TAssignContext;
  TypeEl, SubTypeEl: TPasType;
  aResolveR: TPas2JSResolver;
  RecType: TPasRecordType;
begin
  Result:=nil;
  Param0:=El.Params[0];
  aResolveR:=AContext.Resolver;
  aResolveR.ComputeElement(Param0,ParamResolved,[]);
  RecType:=nil;
  if ParamResolved.BaseType=btContext then
    begin
    TypeEl:=ParamResolved.LoTypeEl;
    if TypeEl.ClassType=TPasPointerType then
      begin
      SubTypeEl:=aResolveR.ResolveAliasType(TPasPointerType(TypeEl).DestType);
      if SubTypeEl.ClassType=TPasRecordType then
        RecType:=TPasRecordType(SubTypeEl);
      end;
    end;
  if RecType=nil then
    DoError(20180425011901,nIncompatibleTypesGotExpected,sIncompatibleTypesGotExpected,
      [aResolveR.GetResolverResultDescription(ParamResolved,true),'pointer of record'],Param0);

  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    aResolveR.ComputeElement(Param0,AssignContext.LeftResolved,[rcNoImplicitProc]);
    AssignContext.RightResolved:=AssignContext.LeftResolved;

    // create right side  new TRecord()
    AssignContext.RightSide:=CreateRecordCallNew(El,RecType,AContext);

    Result:=CreateAssignStatement(Param0,AssignContext);
  finally
    AssignContext.RightSide.Free;
    AssignContext.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_Dispose(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
// dispose(p)
// if p is writable set to null
var
  Param0: TPasExpr;
  aResolveR: TPas2JSResolver;
  ParamResolved: TPasResolverResult;
  TypeEl, SubTypeEl: TPasType;
  RecType: TPasRecordType;
  AssignContext: TAssignContext;
begin
  Result:=nil;
  Param0:=El.Params[0];
  aResolveR:=AContext.Resolver;
  aResolveR.ComputeElement(Param0,ParamResolved,[]);
  RecType:=nil;
  if ParamResolved.BaseType=btContext then
    begin
    TypeEl:=ParamResolved.LoTypeEl;
    if TypeEl.ClassType=TPasPointerType then
      begin
      SubTypeEl:=aResolveR.ResolveAliasType(TPasPointerType(TypeEl).DestType);
      if SubTypeEl.ClassType=TPasRecordType then
        RecType:=TPasRecordType(SubTypeEl);
      end;
    end;
  if RecType=nil then
    DoError(20180425012910,nIncompatibleTypesGotExpected,sIncompatibleTypesGotExpected,
      [aResolveR.GetResolverResultDescription(ParamResolved,true),'pointer of record'],Param0);

  if not (rrfWritable in ParamResolved.Flags) then
    // Param0 is no writable
    exit(nil);

  // Param0 is writable -> set to null
  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    aResolveR.ComputeElement(Param0,AssignContext.LeftResolved,[rcNoImplicitProc]);
    AssignContext.RightResolved:=AssignContext.LeftResolved;

    // create right side:  null
    AssignContext.RightSide:=CreateLiteralNull(El);

    Result:=CreateAssignStatement(Param0,AssignContext);
  finally
    AssignContext.RightSide.Free;
    AssignContext.Free;
  end;
end;

function TPasToJSConverter.ConvertBuiltIn_Default(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;

  procedure CreateEnumValue(TypeEl: TPasEnumType);
  var
    EnumValue: TPasEnumValue;
  begin
    EnumValue:=TPasEnumValue(TypeEl.Values[0]);
    Result:=CreateReferencePathExpr(EnumValue,AContext);
  end;

var
  ResolvedEl: TPasResolverResult;
  Param: TPasExpr;
  TypeEl: TPasType;
  Value: TResEvalValue;
  MinVal, MaxVal: TMaxPrecInt;
  C: TClass;
begin
  Result:=nil;
  if AContext.Resolver=nil then
    RaiseInconsistency(20180501011029,El);
  Param:=El.Params[0];
  AContext.Resolver.ComputeElement(Param,ResolvedEl,[]);
  case ResolvedEl.BaseType of
  btBoolean,btByteBool,btWordBool,btLongBool:
    begin
    Result:=CreateLiteralBoolean(El,LowJSBoolean);
    exit;
    end;
  btChar,
  btWideChar:
    begin
    Result:=CreateLiteralJSString(El,#0);
    exit;
    end;
  btString,btUnicodeString:
    begin
    Result:=CreateLiteralJSString(El,'');
    exit;
    end;
  btByte..btIntMax:
    begin
    TypeEl:=ResolvedEl.LoTypeEl;
    if TypeEl.ClassType=TPasUnresolvedSymbolRef then
      begin
      if TypeEl.CustomData is TResElDataBaseType then
        begin
        AContext.Resolver.GetIntegerRange(ResolvedEl.BaseType,MinVal,MaxVal);
        Result:=CreateLiteralNumber(El,MinVal);
        exit;
        end;
      end
    else if TypeEl.ClassType=TPasRangeType then
      begin
      Value:=AContext.Resolver.EvalRangeLimit(TPasRangeType(TypeEl).RangeExpr,
                                              [refConst],true,El);
      try
        case Value.Kind of
        revkInt:
          Result:=CreateLiteralNumber(El,TResEvalInt(Value).Int);
        revkUInt:
          Result:=CreateLiteralNumber(El,TResEvalUInt(Value).UInt);
        else
          RaiseNotSupported(El,AContext,20180501011646);
        end;
        exit;
      finally
        ReleaseEvalValue(Value);
      end;
      end;
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertBuiltIn_Default ',GetResolverResultDbg(ResolvedEl));
    {$ENDIF}
    RaiseNotSupported(El,AContext,20180501011649);
    end;
  btSingle,btDouble:
    begin
    Result:=CreateLiteralNumber(El,0);
    TJSLiteral(Result).Value.CustomValue:='0.0';
    exit;
    end;
  btCurrency:
    begin
    Result:=CreateLiteralNumber(El,0);
    exit;
    end;
  btContext:
    begin
    TypeEl:=ResolvedEl.LoTypeEl;
    C:=TypeEl.ClassType;
    if C=TPasEnumType then
      begin
      CreateEnumValue(TPasEnumType(TypeEl));
      exit;
      end
    else if C=TPasSetType then
      begin
      Result:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
      exit;
      end
    else if C=TPasArrayType then
      begin
      Result:=CreateArrayInit(TPasArrayType(TypeEl),nil,El,AContext);
      exit;
      end
    else if C=TPasRecordType then
      begin
      Result:=CreateRecordInit(TPasRecordType(TypeEl),nil,El,AContext);
      exit;
      end
    else if C=TPasRangeType then
      // a custom range without initial value -> use first value
      begin
      Value:=AContext.Resolver.Eval(TPasRangeType(TypeEl).RangeExpr.left,[refConst]);
      try
        Result:=ConvertConstValue(Value,AContext,El);
      finally
        ReleaseEvalValue(Value);
      end;
      end
    else if (C=TPasClassType) or (C=TPasPointerType) then
      begin
      Result:=CreateLiteralNull(El);
      exit;
      end;
    end;
  btRange:
    begin
    if ResolvedEl.LoTypeEl is TPasRangeType then
      begin
      Value:=AContext.Resolver.Eval(TPasRangeType(ResolvedEl.LoTypeEl).RangeExpr.left,[refConst]);
      try
        Result:=ConvertConstValue(Value,AContext,El);
      finally
        ReleaseEvalValue(Value);
      end;
      exit;
      end;
    end;
  btSet:
    begin
    Result:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
    exit;
    end;
  end;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertBuiltIn_Default ',GetResolverResultDbg(ResolvedEl));
  {$ENDIF}
  DoError(20180501011723,nXExpectedButYFound,sXExpectedButYFound,['record',
    AContext.Resolver.GetResolverResultDescription(ResolvedEl)],Param);
end;

function TPasToJSConverter.ConvertBuiltIn_Debugger(El: TPasExpr;
  AContext: TConvertContext): TJSElement;
begin
  Result:=CreateLiteralCustomValue(El,'debugger');
  if AContext=nil then ;
end;

function TPasToJSConverter.ConvertBuiltIn_AWait(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  Param: TPasExpr;
  JS: TJSElement;
  AWaitJS: TJSAwaitExpression;
begin
  if length(El.Params)=1 then
    Param:=El.Params[0]
  else if length(El.Params)=2 then
    Param:=El.Params[1]
  else
    RaiseNotSupported(El,AContext,20200519233919);
  JS:=ConvertExpression(Param,AContext);
  AWaitJS:=TJSAwaitExpression(CreateElement(TJSAwaitExpression,El));
  AWaitJS.A:=JS;
  Result:=AWaitJS;
end;

function TPasToJSConverter.ConvertRecordValues(El: TRecordValues;
  AContext: TConvertContext): TJSElement;
var
  aResolver: TPas2JSResolver;
  Vars: TFPList;
  RecType: TPasRecordType;
  Ref: TResolvedReference;
  ResolvedEl: TPasResolverResult;
  ObjLit: TJSObjectLiteral;
  i: Integer;
  RecFields: TRecordValuesItemArray;
  Field: PRecordValuesItem;
  Member: TPasElement;
  PasVar: TPasVariable;
  ok: Boolean;
  ObjLitEl: TJSObjectLiteralElement;
  Call: TJSCallExpression;
  CurName: String;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  Vars:=TFPList.Create;
  ok:=false;
  try
    RecType:=nil;
    if aResolver<>nil then
      begin
      // with resolver: TRecord.$clone({...})
      aResolver.ComputeElement(El,ResolvedEl,[]);
      if (ResolvedEl.BaseType<>btContext)
          or (ResolvedEl.LoTypeEl.ClassType<>TPasRecordType) then
        RaiseNotSupported(El,AContext,20180429210932);
      RecType:=TPasRecordType(ResolvedEl.LoTypeEl);

      Call:=CreateRecordCallClone(El,RecType,nil,AContext);
      Result:=Call;
      ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
      Call.AddArg(ObjLit);
      end
    else
      begin
      // without resolver: {...}
      ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
      Result:=ObjLit;;
      end;

    RecFields:=El.Fields;
    for i:=0 to length(RecFields)-1 do
      begin
      Field:=@RecFields[i];
      Ref:=Field^.NameExp.CustomData as TResolvedReference;
      PasVar:=Ref.Declaration as TPasVariable;
      Vars.Add(PasVar);
      ObjLitEl:=ObjLit.Elements.AddElement;
      CurName:=TransformElToJSName(PasVar,AContext);
      if CurName[1]='[' then
        begin
        if CurName[length(CurName)]=']' then
          CurName:=copy(CurName,2,length(CurName)-2)
        else
          CurName:=copy(CurName,2,length(CurName)-1);
        end;
      ObjLitEl.Name:=TJSString(CurName);
      ObjLitEl.Expr:=CreateValInit(PasVar.VarType,Field^.ValueExp,Field^.NameExp,AContext);
      end;
    // add missing fields
    if RecType<>nil then
      for i:=0 to RecType.Members.Count-1 do
        begin
        Member:=TPasElement(RecType.Members[i]);
        if Member.ClassType<>TPasVariable then continue;
        PasVar:=TPasVariable(Member);
        if [vmClass,vmStatic]*PasVar.VarModifiers<>[] then continue;
        if Vars.IndexOf(PasVar)>=0 then continue;
        if not IsElementUsed(PasVar) then continue;
        // missing instance field in constant -> add default value
        ObjLitEl:=ObjLit.Elements.AddElement;
        ObjLitEl.Name:=TJSString(TransformElToJSName(PasVar,AContext));
        ObjLitEl.Expr:=CreateValInit(PasVar.VarType,PasVar.Expr,PasVar,AContext);
        end;
    ok:=true;
  finally
    Vars.Free;
    if not ok then
      Result.Free;
  end;
end;

function TPasToJSConverter.ConvertArrayValues(El: TArrayValues;
  AContext: TConvertContext): TJSElement;
Var
  ArrLit :  TJSArrayLiteral;
  I : Integer;
begin
  ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
  For I:=0 to Length(El.Values)-1 do
    begin
    ArrLit.AddElement(ConvertExpression(El.Values[i],AContext));
    end;
  Result:=ArrLit;
end;

function TPasToJSConverter.ConvertExpression(El: TPasExpr;
  AContext: TConvertContext): TJSElement;
var
  C: TClass;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertExpression El=',GetObjName(El),' Context=',GetObjName(AContext));
  {$ENDIF}
  Result:=Nil;
  C:=El.ClassType;
  if C=TUnaryExpr then
    Result:=ConvertUnaryExpression(TUnaryExpr(El),AContext)
  else if C=TBinaryExpr then
    Result:=ConvertBinaryExpression(TBinaryExpr(El),AContext)
  else if C=TPrimitiveExpr then
    Result:=ConvertPrimitiveExpression(TPrimitiveExpr(El),AContext)
  else if C=TBoolConstExpr then
    Result:=ConvertBoolConstExpression(TBoolConstExpr(El),AContext)
  else if C=TNilExpr then
    Result:=ConvertNilExpr(TNilExpr(El),AContext)
  else if C=TInheritedExpr then
    Result:=ConvertInheritedExpr(TInheritedExpr(El),AContext)
  else if C=TParamsExpr then
    Result:=ConvertParamsExpr(TParamsExpr(El),AContext)
  else if C=TProcedureExpr then
    Result:=ConvertProcedure(TProcedureExpr(El).Proc,AContext)
  else if C=TRecordValues then
    Result:=ConvertRecordValues(TRecordValues(El),AContext)
  else if C=TArrayValues then
    Result:=ConvertArrayValues(TArrayValues(El),AContext)
  else if C=TInlineSpecializeExpr then
    Result:=ConvertInlineSpecializeExpr(TInlineSpecializeExpr(El),AContext)
  else
    RaiseNotSupported(El,AContext,20161024191314);
end;

function TPasToJSConverter.CreatePrimitiveDotExpr(Path: string;
  PosEl: TPasElement): TJSElement;
var
  p: Integer;
  DotExpr: TJSDotMemberExpression;
  Ident: TJSPrimaryExpressionIdent;
begin
  if Path='' then
    RaiseInconsistency(20170402230134,PosEl);
  p:=PosLast('.',Path);
  if p>0 then
    begin
    if PosEl<>nil then
      DotExpr:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,PosEl))
    else
      DotExpr:=TJSDotMemberExpression.Create(0,0);
    DotExpr.Name:=TJSString(copy(Path,p+1,length(Path))); // do not lowercase
    DotExpr.MExpr:=CreatePrimitiveDotExpr(LeftStr(Path,p-1),PosEl);
    Result:=DotExpr;
    end
  else
    begin
    if PosEl<>nil then
      Ident:=TJSPrimaryExpressionIdent(CreateElement(TJSPrimaryExpressionIdent,PosEl))
    else
      Ident:=TJSPrimaryExpressionIdent.Create(0,0);
    Ident.Name:=TJSString(Path); // do not lowercase
    Result:=Ident;
    end;
end;

function TPasToJSConverter.CreateTypeDecl(El: TPasType;
  AContext: TConvertContext): TJSElement;

var
  C: TClass;
  GlobalCtx: TConvertContext;
begin
  Result:=Nil;
  GlobalCtx:=AContext;
  if El.Parent is TProcedureBody then
    GlobalCtx:=AContext.GetGlobalFunc;

  C:=El.ClassType;
  if C=TPasClassType then
    Result := ConvertClassType(TPasClassType(El), GlobalCtx)
  else if (C=TPasClassOfType) then
    Result := ConvertClassOfType(TPasClassOfType(El), GlobalCtx)
  else if C=TPasRecordType then
    Result := ConvertRecordType(TPasRecordType(El), GlobalCtx)
  else if C=TPasEnumType then
    Result := ConvertEnumType(TPasEnumType(El), GlobalCtx)
  else if (C=TPasSetType) then
    Result := ConvertSetType(TPasSetType(El), GlobalCtx)
  else if (C=TPasRangeType) then
    Result:=ConvertRangeType(TPasRangeType(El),GlobalCtx)
  else if (C=TPasAliasType) then
  else if (C=TPasTypeAliasType) then
    Result:=ConvertTypeAliasType(TPasTypeAliasType(El),GlobalCtx)
  else if (C=TPasPointerType) then
    Result:=ConvertPointerType(TPasPointerType(El),GlobalCtx)
  else if (C=TPasProcedureType)
       or (C=TPasFunctionType) then
    Result:=ConvertProcedureType(TPasProcedureType(El),GlobalCtx)
  else if (C=TPasArrayType) then
    Result:=ConvertArrayType(TPasArrayType(El),GlobalCtx)
  else if (C=TPasSpecializeType) then
    // specialize type is converted at the generic type
  else
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.CreateTypeDecl El=',GetObjName(El));
    {$ENDIF}
    RaiseNotSupported(El,AContext,20170208144053);
    end;
end;

function TPasToJSConverter.CreateVarDecl(El: TPasVariable;
  AContext: TConvertContext): TJSElement;

Var
  C : TJSElement;
  V : TJSVariableStatement;
  AssignSt: TJSSimpleAssignStatement;
  Obj: TJSObjectLiteral;
  ObjLit: TJSObjectLiteralElement;
begin
  Result:=nil;
  if El.AbsoluteExpr<>nil then
    exit; // absolute: do not add a declaration
  if vmExternal in El.VarModifiers then
    exit; // external: do not add a declaration
  if AContext is TObjectContext then
    begin
    // create 'A: initvalue'
    Obj:=TObjectContext(AContext).JSElement as TJSObjectLiteral;
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TJSString(TransformElToJSName(El,AContext));
    ObjLit.Expr:=CreateVarInit(El,AContext);
    end
  else if AContext.IsGlobal or (El.Parent is TPasMembersType) then
    begin
    // create 'this.A=initvalue'
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
    Result:=AssignSt;
    AssignSt.LHS:=CreateSubDeclNameExpr(El,AContext);
    AssignSt.Expr:=CreateVarInit(El,AContext);
    end
  else
    begin
    // create 'var A=initvalue'
    C:=ConvertVariable(El,AContext);
    if C=nil then
      RaiseInconsistency(20180501114300,El);
    V:=TJSVariableStatement(CreateElement(TJSVariableStatement,El));
    V.A:=C;
    Result:=V;
    end;
end;

function TPasToJSConverter.CreateSwitchStatement(El: TPasImplCaseOf;
  AContext: TConvertContext): TJSElement;
var
  SwitchEl: TJSSwitchStatement;
  JSCaseEl: TJSCaseElement;
  SubEl: TPasImplElement;
  St: TPasImplCaseStatement;
  ok: Boolean;
  i, j: Integer;
  BreakSt: TJSBreakStatement;
  BodySt: TJSElement;
  StList: TJSStatementList;
  Expr: TPasExpr;
begin
  Result:=nil;
  SwitchEl:=TJSSwitchStatement(CreateElement(TJSSwitchStatement,El));
  ok:=false;
  try
    SwitchEl.Cond:=ConvertExpression(El.CaseExpr,AContext);
    for i:=0 to El.Elements.Count-1 do
      begin
      SubEl:=TPasImplElement(El.Elements[i]);
      if not (SubEl is TPasImplCaseStatement) then
        continue;
      St:=TPasImplCaseStatement(SubEl);
      JSCaseEl:=nil;
      for j:=0 to St.Expressions.Count-1 do
        begin
        Expr:=TPasExpr(St.Expressions[j]);
        JSCaseEl:=SwitchEl.Cases.AddCase;
        JSCaseEl.Expr:=ConvertExpression(Expr,AContext);
        end;
      BodySt:=nil;
      if St.Body<>nil then
        BodySt:=ConvertElement(St.Body,AContext);
      // add break
      BreakSt:=TJSBreakStatement(CreateElement(TJSBreakStatement,St));
      if BodySt=nil then
        // no Pascal statement -> add only one 'break;'
        BodySt:=BreakSt
      else
        begin
        if (BodySt is TJSStatementList) then
          begin
          // list of statements -> append 'break;' to end
          StList:=TJSStatementList(BodySt);
          AddToStatementList(TJSStatementList(BodySt),StList,BreakSt,St);
          end
        else
          begin
          // single statement -> create list of old and 'break;'
          StList:=TJSStatementList(CreateElement(TJSStatementList,St));
          StList.A:=BodySt;
          StList.B:=BreakSt;
          BodySt:=StList;
          end;
        end;
      JSCaseEl.Body:=BodySt;
      end;
    if El.ElseBranch<>nil then
      begin
      JSCaseEl:=SwitchEl.Cases.AddCase;
      JSCaseEl.Body:=ConvertImplBlockElements(El.ElseBranch,AContext,false);
      SwitchEl.TheDefault:=JSCaseEl;
      end;
    ok:=true;
  finally
    if not ok then
      SwitchEl.Free;
  end;
  Result:=SwitchEl;
end;

function TPasToJSConverter.ConvertDeclarations(El: TPasDeclarations;
  AContext: TConvertContext): TJSElement;
Var
  SLFirst, SLLast: TJSStatementList;
  IsProcBody, IsFunction, IsAssembler, IsConstructor, HasResult: boolean;
  PasProc: TPasProcedure;
  ProcScope: TPasProcedureScope;
  ProcBody: TPasImplBlock;
  ResultEl: TPasResultElement;
  ResultVarName: String;
  ResStrVarEl: TJSVarDeclaration;
  ResStrVarElAdd: boolean;

  Procedure Add(NewEl: TJSElement; PosEl: TPasElement);
  begin
    if AContext is TObjectContext then
      begin
      // NewEl is already added
      end
    else if AContext.IsGlobal and (AContext.JSElement is TJSSourceElements) then
      AddToSourceElements(TJSSourceElements(AContext.JSElement),NewEl)
    else
      begin
      AddToStatementList(SLFirst,SLLast,NewEl,PosEl);
      ConvertDeclarations:=SLFirst;
      end;
  end;

  Procedure AddFunctionResultInit;
  var
    Proc: TPasProcedure;
    FunType: TPasFunctionType;
    VarSt: TJSVariableStatement;
    ImplScope: TPas2JSProcedureScope;
  begin
    Proc:=El.Parent as TPasProcedure;
    FunType:=Proc.ProcType as TPasFunctionType;
    ResultEl:=FunType.ResultEl;
    ImplScope:=Proc.CustomData as TPas2JSProcedureScope;
    if (ResultEl=nil) or (ResultEl.ResultType=nil) then
      begin
      Proc:=ImplScope.DeclarationProc;
      FunType:=Proc.ProcType as TPasFunctionType;
      ResultEl:=FunType.ResultEl;
      end;
    if ImplScope.ResultVarName<>'' then
      ResultVarName:=ImplScope.ResultVarName
    else
      ResultVarName:=ResolverResultVar;

    // add 'var result=initvalue'
    VarSt:=CreateVarStatement(ResultVarName,
      CreateValInit(ResultEl.ResultType,nil,ResultEl,aContext),ResultEl);
    Add(VarSt,ResultEl);
    Result:=SLFirst;
  end;

  Procedure AddFunctionResultReturn;
  var
    RetSt: TJSReturnStatement;
  begin
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,ResultEl));
    RetSt.Expr:=CreatePrimitiveDotExpr(ResultVarName,ResultEl);
    Add(RetSt,ResultEl);
  end;

  Procedure AddReturnThis;
  var
    RetSt: TJSReturnStatement;
    HelperForType: TPasType;
    Call: TJSCallExpression;
    Proc: TPasProcedure;
    aResolver: TPas2JSResolver;
    ClassOrRec: TPasMembersType;
  begin
    // "return this"
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
    RetSt.Expr:=TJSPrimaryExpressionThis(CreateElement(TJSPrimaryExpressionThis,El));
    aResolver:=AContext.Resolver;
    if aResolver<>nil then
      begin
      Proc:=TPasProcedure(El.Parent);
      ProcScope:=Proc.CustomData as TPas2JSProcedureScope;
      ClassOrRec:=ProcScope.ClassRecScope.Element as TPasMembersType;
      if (ClassOrRec.ClassType=TPasClassType)
          and (TPasClassType(ClassOrRec).HelperForType<>nil) then
        begin
        HelperForType:=AContext.Resolver.ResolveAliasType(TPasClassType(ClassOrRec).HelperForType);
        if HelperForType is TPasMembersType then
          // helper constructor for class or record -> "this" is the class/record
        else
          begin
          // helper constructor for a simpletype -> "this" is a reference
          // -> return this.get()
          Call:=CreateCallExpression(El);
          Call.Expr:=CreateDotExpression(El,RetSt.Expr,
            CreatePrimitiveDotExpr(TempRefObjGetterName,El));
          RetSt.Expr:=Call;
          end;
        end;
      end;
    Add(RetSt,El);
  end;

  procedure AddResourceString(ResStr: TPasResString);
  // $mod.$resourcestrings = {
  //  name1 : { org: "value" },
  //  name2 : { org: "value" },
  //  ...
  //  }
  var
    Value: TResEvalValue;
    ObjLit: TJSObjectLiteral;
    Lit: TJSObjectLiteralElement;
    RootContext: TRootContext;
  begin
    // first convert expression, it might fail
    Value:=AContext.Resolver.Eval(ResStr.Expr,[refConst]);
    //writeln('AddResourceString ',GetObjName(ResStr),' Value=',Value.AsDebugString);
    // create table
    if (ResStrVarEl=nil) and (El.ClassType=TImplementationSection) then
      begin
      RootContext:=AContext.GetRootContext as TRootContext;
      ResStrVarEl:=RootContext.ResourceStrings;
      end;
    if ResStrVarEl=nil then
      begin
      ResStrVarEl:=TJSVarDeclaration(CreateElement(TJSVarDeclaration,El));
      ResStrVarEl.Name:=GetBIName(pbivnModule)+'.'+GetBIName(pbivnResourceStrings);
      ResStrVarElAdd:=true;
      ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
      ResStrVarEl.Init:=ObjLit;
      RootContext:=TRootContext(AContext.GetContextOfType(TRootContext));
      RootContext.ResourceStrings:=ResStrVarEl;
      end;
    // add element:  name : { ... }
    Lit:=TJSObjectLiteral(ResStrVarEl.Init).Elements.AddElement;
    Lit.Name:=TJSString(TransformElToJSName(ResStr,AContext));
    ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,ResStr));
    Lit.Expr:=ObjLit;
    // add sub element: org: value
    Lit:=ObjLit.Elements.AddElement;
    Lit.Name:=TJSString(GetBIName(pbivnResourceStringOrig));
    Lit.Expr:=ConvertConstValue(Value,AContext,ResStr);
    ReleaseEvalValue(Value);
  end;

  procedure AddResultInterfacRelease(FuncContext: TFunctionContext);
  var
    AssignSt: TJSSimpleAssignStatement;
    IfSt: TJSIfStatement;
    VarSt: TJSVariableStatement;
    Call: TJSCallExpression;
  begin
    AddInterfaceReleases(FuncContext,ProcBody);
    if FuncContext.ResultNeedsIntfRelease then
      begin
      // add in front of try "var $ok=false;"
      VarSt:=CreateVarStatement(GetBIName(pbivnProcOk),CreateLiteralBoolean(ProcBody,false),ProcBody);
      AddInFrontOfFunctionTry(VarSt,ProcBody,FuncContext);
      // add in front of finally "$ok=true;"
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,ProcBody));
      AddToStatementList(FuncContext.TrySt.Block as TJSStatementList,AssignSt,ProcBody);
      AssignSt.LHS:=CreatePrimitiveDotExpr(GetBIName(pbivnProcOk),ProcBody);
      AssignSt.Expr:=CreateLiteralBoolean(ProcBody,true);
      // add finally: "if(!$ok) rtl._Release(Result);"
      IfSt:=TJSIfStatement(CreateElement(TJSIfStatement,ProcBody));
      AddFunctionFinallySt(IfSt,ProcBody,FuncContext);
      // !$ok
      IfSt.Cond:=CreateUnaryNot(
          CreatePrimitiveDotExpr(GetBIName(pbivnProcOk),ProcBody),ProcBody);
      // rtl._Release(Result)
      Call:=CreateCallExpression(ProcBody);
      IfSt.BTrue:=Call;
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntf_Release)]);
      Call.AddArg(CreatePrimitiveDotExpr(ResultVarName,ProcBody));
      end;
  end;

  procedure InitForwards(Decls: TFPList; SectionContext: TSectionContext);
  var
    i: Integer;
    P: TPasElement;
    C: TClass;
    Proc: TPasProcedure;
    aResolver: TPas2JSResolver;
  begin
    aResolver:=AContext.Resolver;
    For i:=0 to Decls.Count-1 do
      begin
      P:=TPasElement(Decls[i]);
      if not IsElementUsed(P) then continue;
      C:=P.ClassType;
      if (C=TPasClassType) or (C=TPasRecordType) or (C=TPasEnumType) then
        begin
        if (C=TPasClassType) then
          begin
          if TPasClassType(P).IsForward then
            continue;
          if not aResolver.IsFullySpecialized(TPasClassType(P)) then
            continue;
          end
        else if C=TPasRecordType then
          begin
          if not aResolver.IsFullySpecialized(TPasRecordType(P)) then
            continue;
          end;
        // add var $lt = null;
        CreateGlobalAliasNull(P,pbivnLocalTypeRef,SectionContext);
        if (C=TPasClassType) or (C=TPasRecordType) then
          InitForwards(TPasMembersType(P).Members,SectionContext);
        end
      else if C.InheritsFrom(TPasProcedure) then
        begin
        Proc:=TPasProcedure(P);
        if Proc.IsForward or Proc.IsAbstract or Proc.IsExternal then
          continue;
        if TPas2JSProcedureScope(Proc.CustomData).SpecializedFromItem=nil then
          continue;
        if not aResolver.IsFullySpecialized(Proc) then
          continue; // skip non specialized generics
        // specialized proc: add var $lp = null;
        CreateGlobalAliasNull(P,pbivnLocalProcRef,SectionContext);
        end;
      end;
  end;

  procedure InitSection(Section: TPasSection);
  var
    SectionScope: TPas2JSSectionScope;
    SectionCtx: TSectionContext;
    Src: TJSSourceElements;
    ImplSect: TImplementationSection;
  begin
    SectionScope:=Section.CustomData as TPas2JSSectionScope;
    AContext.ScannerBoolSwitches:=SectionScope.BoolSwitches;
    AContext.ScannerModeSwitches:=SectionScope.ModeSwitches;

    if not (AContext is TSectionContext) then
      RaiseNotSupported(Section,AContext,20200606142828,GetObjName(AContext));
    SectionCtx:=TSectionContext(AContext);
    Src:=SectionCtx.JSElement as TJSSourceElements;
    SectionCtx.HeaderIndex:=Src.Statements.Count;

    // add local vars for forward declarations
    if (coShortRefGlobals in Options)
        and (Section.ClassType<>TImplementationSection) then
      begin
      InitForwards(Section.Declarations,TSectionContext(AContext));
      if Section is TInterfaceSection then
        begin
        ImplSect:=TPasModule(Section.Parent).ImplementationSection;
        if ImplSect<>nil then
          InitForwards(ImplSect.Declarations,TSectionContext(AContext));
        end;
      end;
  end;

var
  E, BodySt: TJSElement;
  I : Integer;
  P: TPasElement;
  C: TClass;
begin
  Result:=nil;
  {
    TPasDeclarations = class(TPasElement)
    TPasSection = class(TPasDeclarations)
    TInterfaceSection = class(TPasSection)
    TImplementationSection = class(TPasSection)
    TProgramSection = class(TImplementationSection)
    TLibrarySection = class(TImplementationSection)
    TProcedureBody = class(TPasDeclarations)
  }

  IsProcBody:=(El is TProcedureBody) and (TProcedureBody(El).Body<>nil);
  IsFunction:=IsProcBody and (TPasProcedure(El.Parent).ProcType is TPasFunctionType);
  IsAssembler:=IsProcBody and (TProcedureBody(El).Body is TPasImplAsmStatement);
  IsConstructor:=IsProcBody and (El.Parent.ClassType=TPasConstructor);
  HasResult:=IsFunction and not IsAssembler;

  if (AContext.Resolver<>nil) and (El is TPasSection) then
    InitSection(TPasSection(El));

  SLFirst:=nil;
  SLLast:=nil;
  ResultEl:=nil;
  ResultVarName:='';
  ResStrVarEl:=nil;
  ResStrVarElAdd:=false;
  try

    if HasResult then
      AddFunctionResultInit;

    For I:=0 to El.Declarations.Count-1 do
      begin
      P:=TPasElement(El.Declarations[i]);
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertDeclarations El[',i,']=',GetObjName(P));
      {$ENDIF}
      if not IsElementUsed(P) then continue;

      E:=Nil;
      C:=P.ClassType;
      if C=TPasConst then
        E:=ConvertConst(TPasConst(P),aContext) // can be nil
      else if C=TPasVariable then
        E:=CreateVarDecl(TPasVariable(P),aContext) // can be nil
      else if C.InheritsFrom(TPasType) then
        E:=CreateTypeDecl(TPasType(P),aContext) // can be nil
      else if C.InheritsFrom(TPasProcedure) then
        begin
        PasProc:=TPasProcedure(P);
        if PasProc.IsForward then continue; // JavaScript does not need the forward
        ProcScope:=TPasProcedureScope(PasProc.CustomData);
        if (ProcScope.DeclarationProc<>nil)
            and (not ProcScope.DeclarationProc.IsForward) then
          continue; // this proc was already converted in interface or class
        if ProcScope.DeclarationProc<>nil then
          PasProc:=ProcScope.DeclarationProc;
        E:=ConvertProcedure(PasProc,aContext);
        end
      else if C=TPasResString then
        begin
        if not (El is TPasSection) then
          RaiseNotSupported(P,AContext,20171004185348);
        AddResourceString(TPasResString(P));
        continue;
        end
      else if C=TPasAttributes then
        continue
      else if C=TPasExportSymbol then
        continue
      else
        RaiseNotSupported(P as TPasElement,AContext,20161024191434);
      Add(E,P);
      end;

    if IsProcBody then
      begin
      ProcBody:=TProcedureBody(El).Body;
      if (ProcBody.Elements.Count>0) or IsAssembler then
        begin
        // convert body  (creates a TJSStatementList)
        BodySt:=ConvertElement(ProcBody,aContext);

        if AContext is TFunctionContext then
          begin
          TFunctionContext(AContext).BodySt:=BodySt;
          // if needed add try..finally for COM interfaces
          AddResultInterfacRelease(TFunctionContext(AContext));
          BodySt:=TFunctionContext(AContext).BodySt;
          end;

        Add(BodySt,ProcBody);
        end;
      end;

    if HasResult then
      AddFunctionResultReturn
    else if IsConstructor then
      AddReturnThis;

    if ResStrVarEl<>nil then
      begin
      if ResStrVarElAdd then
        Add(ResStrVarEl,El);
      ResStrVarEl:=nil;
      end;
  finally
    ResStrVarEl.Free;
  end;
end;

function TPasToJSConverter.ConvertClassType(El: TPasClassType;
  AContext: TConvertContext): TJSElement;
(*
  type
    TMyClass = class(Ancestor)
      i: longint;
    end;

  rtl.createClass(this,"TMyClass",Ancestor,function(){
    this.i = 0;
  });
*)
var
  IsTObject, AncestorIsExternal: boolean;

  function IsMemberNeeded(aMember: TPasElement): boolean;
  begin
    if IsElementUsed(aMember) then exit(true);
    if IsTObject then
      begin
      if aMember.ClassType=TPasProcedure then
        begin
        if (CompareText(aMember.Name,'AfterConstruction')=0)
            or (CompareText(aMember.Name,'BeforeDestruction')=0) then
          exit(true);
        end;
      end;
    Result:=false;
  end;

  procedure AddInterfaceProcNames(Call: TJSCallExpression);
  var
    Arr: TJSArrayLiteral;
    i: Integer;
    Member: TPasElement;
  begin
    Arr:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
    Call.AddArg(Arr);
    for i:=0 to El.Members.Count-1 do
      begin
      Member:=TPasElement(El.Members[i]);
      if not (Member is TPasProcedure) then continue;
      if not IsMemberNeeded(Member) then continue;
      if (Member.ClassType=TPasClassConstructor)
          or (Member.ClassType=TPasClassDestructor) then
        continue;
      Arr.AddElement(CreateLiteralString(Member,TransformElToJSName(Member,AContext)));
      end;
  end;

var
  aResolver: TPas2JSResolver;
  DelaySrc: TJSSourceElements;
  DelayFuncContext: TFunctionContext;
  Call: TJSCallExpression;
  FunDecl: TJSFunctionDeclarationStatement;
  Src: TJSSourceElements;
  ArgEx: TJSLiteral;
  FuncContext: TFunctionContext;
  i: Integer;
  NewEl: TJSElement;
  P: TPasElement;
  Scope: TPas2JSClassScope;
  Ancestor: TPasType;
  AncestorPath, OwnerName, DestructorName, FnName, IntfKind, JSName: String;
  C: TClass;
  AssignSt: TJSSimpleAssignStatement;
  NeedInitFunction, HasConstructor, IsJSFunction, NeedClassExt,
    SpecializeDelay, NeedTypeInfo: Boolean;
  Proc: TPasProcedure;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;

  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertClassType START ',GetObjName(El));
  {$ENDIF}
  if not (El.ObjKind in [okClass,okInterface,okClassHelper,okRecordHelper,okTypeHelper]) then
    RaiseNotSupported(El,AContext,20170927183645);
  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231004355);
  if not aResolver.IsFullySpecialized(El) then exit;
  if El.IsForward then
    exit(ConvertClassForwardType(El,AContext))
  else if El.IsExternal then
    exit(ConvertExtClassType(El,AContext));

  IsTObject:=false;
  if El.CustomData is TPas2JSClassScope then
    begin
    Scope:=TPas2JSClassScope(El.CustomData);
    if Scope.AncestorScope<>nil then
      Ancestor:=Scope.AncestorScope.Element as TPasType
    else
      begin
      Ancestor:=nil;
      IsTObject:=(El.ObjKind=okClass) and SameText(El.Name,'TObject');
      end;
    FreeAndNil(Scope.MsgIntToProc);
    FreeAndNil(Scope.MsgStrToProc);
    SpecializeDelay:=SpecializeNeedsDelay(El,AContext);
    end
  else
    begin
    Scope:=nil;
    IsTObject:=(El.AncestorType=nil) and (El.ObjKind=okClass) and SameText(El.Name,'TObject');
    Ancestor:=El.AncestorType;
    SpecializeDelay:=false;
    end;

  // create call 'rtl.createClass(' or 'rtl.createInterface('
  FuncContext:=nil;
  DelaySrc:=nil;
  DelayFuncContext:=nil;
  Call:=CreateCallExpression(El);
  try
    AncestorIsExternal:=(Ancestor is TPasClassType) and TPasClassType(Ancestor).IsExternal;
    IsJSFunction:=aResolver.IsExternalClass_Name(El,'Function');

    NeedClassExt:=AncestorIsExternal or IsJSFunction;
    if NeedClassExt and (El.ObjKind<>okClass) then
      RaiseNotSupported(El,AContext,20200627083750);

    if El.ObjKind=okInterface then
      FnName:=GetBIName(pbifnIntfCreate)
    else if El.ObjKind in okAllHelpers then
      FnName:=GetBIName(pbifnCreateHelper)
    else if NeedClassExt then
      FnName:=GetBIName(pbifnCreateClassExt)
    else
      FnName:=GetBIName(pbifnCreateClass);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),FnName]);

    // add parameter: owner. For top level class, the module is the owner.
    if (El.Parent=nil)
        or ((El.Parent is TPasSection)
          and (El.Parent.ClassType<>TImplementationSection)) then
      OwnerName:=AContext.GetLocalName(El.GetModule,[cvkGlobal])
    else
      OwnerName:=AContext.GetLocalName(El.Parent,[cvkGlobal]);
    if OwnerName='' then
      OwnerName:='this';
    Call.AddArg(CreatePrimitiveDotExpr(OwnerName,El));

    // add parameter: string constant '"classname"'
    JSName:=TransformElToJSName(El,AContext);
    ArgEx:=CreateLiteralString(El,JSName);
    Call.AddArg(ArgEx);

    if El.ObjKind=okInterface then
      begin
      // add parameter: string constant guid
      Call.AddArg(CreateLiteralString(El,uppercase(Scope.GUID)));

      // add parameter: array of function names
      AddInterfaceProcNames(Call);
      end;

    // add parameter: ancestor
    if Ancestor=nil then
      AncestorPath:='null'
    else if AncestorIsExternal then
      AncestorPath:=TPasClassType(Ancestor).ExternalName
    else
      AncestorPath:=CreateReferencePath(Ancestor,AContext,rpkPathAndName);
    Call.AddArg(CreatePrimitiveDotExpr(AncestorPath,El));

    // for external class: add name of NewInstance function
    if NeedClassExt then
      begin
      if Scope.NewInstanceFunction<>nil then
        Call.AddArg(CreateLiteralString(
          Scope.NewInstanceFunction,Scope.NewInstanceFunction.Name))
      else
        Call.AddArg(CreateLiteralString(El,''));
      end;

    NeedInitFunction:=true;
    NeedTypeInfo:=(pcsfPublished in Scope.Flags) or HasTypeInfo(El,AContext);
    IntfKind:='';
    if El.ObjKind=okInterface then
      begin
      if (Scope.AncestorScope=nil) and (not (coNoTypeInfo in Options)) then
        case El.InterfaceType of
        citCom: IntfKind:='com';
        citCorba: ; // default
        else
          RaiseNotSupported(El,AContext,20180405093512);
        end;
      NeedInitFunction:=NeedTypeInfo or (IntfKind<>'') or (coShortRefGlobals in Options);
      end;

    if NeedInitFunction then
      begin
      // add parameter: class initialize function 'function(){...}'
      FunDecl:=CreateFunctionSt(El,true,true);
      Call.AddArg(FunDecl);
      Src:=TJSSourceElements(FunDecl.AFunction.Body.A);

      // create context
      FuncContext:=TFunctionContext.Create(El,Src,AContext);
      FuncContext.IsGlobal:=true;
      FuncContext.ThisVar.Element:=El;
      FuncContext.ThisVar.Kind:=cvkGlobal;

      if coShortRefGlobals in Options then
        begin
        // $lt = this;
        JSName:=AContext.GetLocalName(El,[cvkGlobal]);
        if JSName='' then
          RaiseNotSupported(El,AContext,20200926232402);
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
        AssignSt.LHS:=CreatePrimitiveDotExpr(JSName,El);
        AssignSt.Expr:=CreatePrimitiveDotExpr('this',El);
        AddToSourceElements(Src,AssignSt);
        end;

      if IntfKind<>'' then
        begin
        // add this.$kind="com";
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
        AssignSt.LHS:=CreatePrimitiveDotExpr('this.'+GetBIName(pbivnIntfKind),El);
        AssignSt.Expr:=CreateLiteralString(El,IntfKind);
        AddToSourceElements(Src,AssignSt);
        end;

      // add class members: types and class vars
      if SpecializeDelay then
        DelayFuncContext:=CreateDelayedInitMembersFunction(El,Src,FuncContext,DelaySrc);
      if El.ObjKind in ([okClass]+okAllHelpers) then
        begin
        For i:=0 to El.Members.Count-1 do
          begin
          P:=TPasElement(El.Members[i]);
          //writeln('TPasToJSConverter.ConvertClassType class vars El[',i,']=',GetObjName(P));
          if not IsMemberNeeded(P) then continue;
          C:=P.ClassType;
          NewEl:=nil;
          if C=TPasVariable then
            begin
            if ClassVarModifiersType*TPasVariable(P).VarModifiers*[vmClass, vmStatic]<>[] then
              begin
              NewEl:=CreateVarDecl(TPasVariable(P),FuncContext); // can be nil
              if NewEl=nil then continue;
              end
            else
              continue;
            end
          else if C=TPasConst then
            NewEl:=ConvertConst(TPasConst(P),FuncContext)
          else if C=TPasProperty then
            NewEl:=ConvertProperty(TPasProperty(P),FuncContext)
          else if C.InheritsFrom(TPasType) then
            NewEl:=CreateTypeDecl(TPasType(P),FuncContext)
          else if C.InheritsFrom(TPasProcedure) then
            continue
          else if C=TPasMethodResolution then
            continue
          else if C=TPasAttributes then
            continue
          else
            RaiseNotSupported(P,FuncContext,20161221233338);
          if NewEl<>nil then
            begin
            if SpecializeDelay and not (P is TPasProcedure) then
              AddToSourceElements(DelaySrc,NewEl)
            else
              AddToSourceElements(Src,NewEl);
            end;
          end;
        end;

      if El.ObjKind in [okClass] then
        begin
        // instance initialization function
        AddClassConDestructorFunction(El,Src,FuncContext,IsTObject,Ancestor,mfInit);
        // instance finalization function
        AddClassConDestructorFunction(El,Src,FuncContext,IsTObject,Ancestor,mfFinalize);
        end;

      if El.ObjKind in ([okClass]+okAllHelpers) then
        begin
        HasConstructor:=false;
        // add method implementations
        For i:=0 to El.Members.Count-1 do
          begin
          P:=TPasElement(El.Members[i]);
          //writeln('TPasToJSConverter.ConvertClassType methods El[',i,']=',GetObjName(P));
          if not IsMemberNeeded(P) then continue;
          NewEl:=nil;
          C:=P.ClassType;
          if not (P is TPasProcedure) then continue;
          Proc:=TPasProcedure(P);
          if IsTObject and (C=TPasDestructor) then
            begin
            DestructorName:=TransformElToJSName(P,AContext);
            if DestructorName<>'Destroy' then
              begin
              // add 'rtl.tObjectDestroy="destroy";'
              AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,P));
              AssignSt.LHS:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbivnTObjectDestroy)]);
              AssignSt.Expr:=CreateLiteralString(P,DestructorName);
              AddToSourceElements(Src,AssignSt);
              end;
            end
          else if C=TPasConstructor then
            HasConstructor:=true
          else if (C=TPasClassConstructor)
              or (C=TPasClassDestructor) then
            begin
            AddGlobalClassMethod(AContext,Proc);
            continue;
            end
          else if (Proc.MessageExpr<>nil) and (aResolver<>nil) then
            aResolver.AddMessageIdToClassScope(Proc,false);
          NewEl:=ConvertProcedure(Proc,FuncContext);
          if NewEl=nil then
            continue; // e.g. abstract or external proc
          AddToSourceElements(Src,NewEl);
          end;
        if HasConstructor and (El.HelperForType<>nil) then
          AddHelperConstructor(El,Src,FuncContext);
        end;

      if aResolver<>nil then
        begin
        // add interfaces
        if (El.ObjKind=okClass) then
          AddClassSupportedInterfaces(El,Src,FuncContext);
        AddClassMessageIds(El,Src,FuncContext,pbivnMessageInt);
        AddClassMessageIds(El,Src,FuncContext,pbivnMessageStr);
        if NeedTypeInfo then
          begin
          // add RTTI init function
          if SpecializeDelay then
            AddClassRTTI(El,DelaySrc,DelayFuncContext)
          else
            AddClassRTTI(El,Src,FuncContext);
          end;
        end;

      end;// end of init function

    // for specialization: add RTTI name
    if ((Scope.JSName<>'') and (Scope.JSName<>El.Name))
        or (El.Parent is TPasMembersType) then
      begin
      Call.AddArg(CreateLiteralString(El,GetTypeInfoName(El,AContext,El)));
      end;

    Result:=Call;
  finally
    FuncContext.Free;
    if Result<>Call then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertClassForwardType(El: TPasClassType;
  AContext: TConvertContext): TJSElement;
// module.$rtti.$Class("classname");
var
  Ref: TResolvedReference;
  aClass: TPasClassType;
  Creator: String;
  ObjLit: TJSObjectLiteral;
begin
  Result:=nil;
  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231004420);
  if (El.GenericTemplateTypes<>nil) and (El.GenericTemplateTypes.Count>0) then
    exit;
  if (AContext.Resolver=nil) or not (El.CustomData is TResolvedReference) then
    exit;
  Ref:=TResolvedReference(El.CustomData);
  aClass:=Ref.Declaration as TPasClassType;
  if IsClassRTTICreatedBefore(aClass,El,AContext) then
    exit; // there is a class-of in front, which already created the class RTTI

  if not HasTypeInfo(aClass,AContext) then exit;

  // module.$rtti.$Class("classname");
  Creator:=GetClassBIName(aClass,AContext);
  Result:=CreateRTTINewType(aClass,Creator,true,AContext,ObjLit);
  if ObjLit<>nil then
    RaiseInconsistency(20170412093427,El);
end;

function TPasToJSConverter.ConvertClassOfType(El: TPasClassOfType;
  AContext: TConvertContext): TJSElement;
// create
// module.$rtti.$ClassRef("typename",{
//    instancetype: module.$rtti["classname"])
//  }
// if class is defined later add a forward define for the class
var
  ObjLit: TJSObjectLiteral;
  Prop: TJSObjectLiteralElement;
  Call: TJSCallExpression;
  ok: Boolean;
  List: TJSStatementList;
  DestType: TPasClassType;
begin
  Result:=nil;
  if not HasTypeInfo(El,AContext) then exit;
  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231004435);

  ok:=false;
  Call:=CreateRTTINewType(El,GetBIName(pbifnRTTINewClassRef),false,AContext,ObjLit);
  Result:=Call;
  try
    Prop:=ObjLit.Elements.AddElement;
    Prop.Name:=TJSString(GetBIName(pbivnRTTIClassRef_InstanceType));
    DestType:=AContext.Resolver.ResolveAliasType(El.DestType) as TPasClassType;
    Prop.Expr:=CreateTypeInfoRef(DestType,AContext,El);

    if IsClassRTTICreatedBefore(DestType,El,AContext) then
      // there is a forward class in front, which already created the class RTTI
    else
      begin
      // class rtti must be forward registered
      if not (AContext is TFunctionContext) then
        RaiseNotSupported(El,AContext,20170412102916);
      // prepend   module.$rtti.$Class("classname");
      Call:=CreateRTTINewType(DestType,GetClassBIName(DestType,AContext),true,
                              AContext,ObjLit);
      if ObjLit<>nil then
        RaiseInconsistency(20170412102654,El);
      List:=TJSStatementList(CreateElement(TJSStatementList,El));
      List.A:=Call;
      List.B:=Result;
      Result:=List;
      end;
    ok:=true;
  finally
    if not ok then
      FreeAndNil(Result);
  end;
end;

function TPasToJSConverter.ConvertExtClassType(El: TPasClassType;
  AContext: TConvertContext): TJSElement;
//   module.$rtti.$ExtClass("TJSObject",{
//     ancestor: ancestortypeinfo,
//     jsclass: "Object"
//   });
var
  TIObj: TJSObjectLiteral;
  Call: TJSCallExpression;
  TIProp: TJSObjectLiteralElement;
  ClassScope: TPas2JSClassScope;
  AncestorType: TPasClassType;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  if not El.IsExternal then
    RaiseNotSupported(El,AContext,20191027183236);

  aResolver:=AContext.Resolver;
  if not aResolver.IsFullySpecialized(El) then
    exit;

  if not HasTypeInfo(El,AContext) then
    exit;
  // create typeinfo
  if not (AContext is TFunctionContext) then
    RaiseNotSupported(El,AContext,20191027182023,'typeinfo');
  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20191027182019);

  ClassScope:=El.CustomData as TPas2JSClassScope;
  if ClassScope.AncestorScope<>nil then
    AncestorType:=ClassScope.AncestorScope.Element as TPasClassType
  else
    AncestorType:=nil;

  Call:=nil;
  try
    // module.$rtti.$ExtClass("TMyClass",{...});
    Call:=CreateRTTINewType(El,GetBIName(pbifnRTTINewExtClass),false,AContext,TIObj);
    if AncestorType<>nil then
      begin
      // add  ancestor: ancestortypeinfo
      TIProp:=TIObj.Elements.AddElement;
      TIProp.Name:=TJSString(GetBIName(pbivnRTTIExtClass_Ancestor));
      TIProp.Expr:=CreateTypeInfoRef(AncestorType,AContext,El);
      end;
    // add  jsclass: "extname"
    TIProp:=TIObj.Elements.AddElement;
    TIProp.Name:=TJSString(GetBIName(pbivnRTTIExtClass_JSClass));
    TIProp.Expr:=CreateLiteralString(El,TPasClassType(El).ExternalName);
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertEnumType(El: TPasEnumType;
  AContext: TConvertContext): TJSElement;
// TMyEnum = (red, green)
// convert to
//   this.TMyEnum = {
//     "0":"red",
//     "red":0,
//     "0":"green",
//     "green":0,
//   };
//   module.$rtti.$Enum("TMyEnum",{
//     enumtype: this.TMyEnum,
//     minvalue: 0,
//     maxvalue: 1
//   });
//  coShortRefGlobals:
//  $lt = this.TMyEnum ...
var
  ObjectContect: TObjectContext;
  i: Integer;
  EnumValue: TPasEnumValue;
  ParentObj, Obj, TIObj: TJSObjectLiteral;
  ObjLit, TIProp: TJSObjectLiteralElement;
  AssignSt: TJSSimpleAssignStatement;
  JSName: string;
  Call: TJSCallExpression;
  List: TJSStatementList;
  ok: Boolean;
  OrdType: TOrdType;
  Src: TJSSourceElements;
  ProcScope: TPas2JSProcedureScope;
  VarSt: TJSVariableStatement;
  SectionContext: TSectionContext;
begin
  Result:=nil;
  for i:=0 to El.Values.Count-1 do
    begin
    EnumValue:=TPasEnumValue(El.Values[i]);
    if EnumValue.Value<>nil then
      RaiseNotSupported(EnumValue.Value,AContext,20170208145221,'enum constant');
    end;

  ok:=false;
  ObjectContect:=nil;
  Src:=nil;
  Call:=nil;
  VarSt:=nil;
  ProcScope:=nil;
  try
    Obj:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
    if AContext is TObjectContext then
      begin
      // add 'TypeName: {}'
      ParentObj:=TObjectContext(AContext).JSElement as TJSObjectLiteral;
      ObjLit:=ParentObj.Elements.AddElement;
      ObjLit.Name:=TJSString(TransformElToJSName(El,AContext));
      ObjLit.Expr:=Obj;
      Result:=Obj;
      end
    else if El.Parent is TProcedureBody then
      begin
      // add 'var TypeName = {}'
      JSName:=TransformElToJSName(El,AContext);
      VarSt:=CreateVarStatement(JSName,Obj,El);
      if AContext.JSElement is TJSSourceElements then
        begin
        Src:=TJSSourceElements(AContext.JSElement);
        AddToSourceElements(Src,VarSt); // keep Result=nil
        if AContext is TFunctionContext then
          TFunctionContext(AContext).AddLocalVar(JSName,El,cvkGlobal,false);
        end
      else
        Result:=VarSt;
      end
    else
      begin
      // add 'this.TypeName = {}'
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AssignSt.LHS:=CreateSubDeclNameExpr(El,AContext);
      AssignSt.Expr:=Obj;
      Result:=AssignSt;
      if coShortRefGlobals in Options then
        begin
        SectionContext:=TSectionContext(AContext.GetMainSectionContext);
        JSName:=SectionContext.GetLocalName(El,[cvkGlobal]);
        if JSName='' then
          RaiseNotSupported(El,AContext,20200926232620);
        if coStoreImplJS in Options then
          StoreImplJSLocal(El,AContext);
        // $lt = this.TypeName = {}
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
        AssignSt.LHS:=CreatePrimitiveDotExpr(JSName,El);
        AssignSt.Expr:=Result;
        Result:=AssignSt;
        end;
      end;

    ObjectContect:=TObjectContext.Create(El,Obj,AContext);
    for i:=0 to El.Values.Count-1 do
      begin
      EnumValue:=TPasEnumValue(El.Values[i]);
      JSName:=TransformElToJSName(EnumValue,AContext);
      // add "0":"value"
      ObjLit:=Obj.Elements.AddElement;
      ObjLit.Name:=TJSString(IntToStr(i));
      ObjLit.Expr:=CreateLiteralJSString(El,TJSString(JSName));
      // add value:0
      ObjLit:=Obj.Elements.AddElement;
      ObjLit.Name:=TJSString(JSName);
      ObjLit.Expr:=CreateLiteralNumber(El,i);
      end;

    if Src<>nil then
      begin
      // store precompiled enum type in proc
      ProcScope:=GetImplJSProcScope(El,Src,AContext);
      if ProcScope<>nil then
        ProcScope.AddGlobalJS(CreatePrecompiledJS(VarSt));
      end;

    if HasTypeInfo(El,AContext) then
      begin
      // create typeinfo
      if not (AContext is TFunctionContext) then
        RaiseNotSupported(El,AContext,20170411210045,'typeinfo');
      OrdType:=GetOrdType(0,TMaxPrecInt(El.Values.Count)-1,El);
      // module.$rtti.$TIEnum("TMyEnum",{...});
      Call:=CreateRTTINewType(El,GetBIName(pbifnRTTINewEnum),false,AContext,TIObj);
      // add  minvalue: number
      TIProp:=TIObj.Elements.AddElement;
      TIProp.Name:=TJSString(GetBIName(pbivnRTTIInt_MinValue));
      TIProp.Expr:=CreateLiteralNumber(El,0);
      // add  maxvalue: number
      TIProp:=TIObj.Elements.AddElement;
      TIProp.Name:=TJSString(GetBIName(pbivnRTTIInt_MaxValue));
      TIProp.Expr:=CreateLiteralNumber(El,El.Values.Count-1);
      // add  ordtype: number
      TIProp:=TIObj.Elements.AddElement;
      TIProp.Name:=TJSString(GetBIName(pbivnRTTIInt_OrdType));
      TIProp.Expr:=CreateLiteralNumber(El,ord(OrdType));
      // add  enumtype: this.TypeName
      TIProp:=TIObj.Elements.AddElement;
      TIProp.Name:=TJSString(GetBIName(pbivnRTTIEnum_EnumType));
      TIProp.Expr:=CreateSubDeclNameExpr(El,AContext);

      if Src<>nil then
        begin
        // add to source elements
        AddToSourceElements(Src,Call);
        if ProcScope<>nil then
          ProcScope.AddGlobalJS(CreatePrecompiledJS(Call));
        end
      else if Result=nil then
        RaiseNotSupported(El,AContext,20190101130432)
      else
        begin
        // create statement list
        List:=TJSStatementList(CreateElement(TJSStatementList,El));
        List.A:=Result;
        Result:=List;
        List.B:=Call;
        end;
      Call:=nil;
      end;

    ok:=true;
  finally
    Call.Free;
    ObjectContect.Free;
    if not ok then
      FreeAndNil(Result);
  end;
end;

function TPasToJSConverter.ConvertSetType(El: TPasSetType;
  AContext: TConvertContext): TJSElement;
// create
//   module.$rtti.$Set("name",{
//       comptype: module.$rtti["enumtype"]
//     })
var
  Obj: TJSObjectLiteral;
  Call: TJSCallExpression;
  Prop: TJSObjectLiteralElement;
begin
  Result:=nil;
  if El.IsPacked then
    DoError(20170222231613,nPasElementNotSupported,sPasElementNotSupported,
      ['packed'],El);
  if not HasTypeInfo(El,AContext) then exit;

  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231112029);

  // module.$rtti.$Set("name",{...})
  Call:=CreateRTTINewType(El,GetBIName(pbifnRTTINewSet),false,AContext,Obj);
  try
    // "comptype: ref"
    Prop:=Obj.Elements.AddElement;
    Prop.Name:=TJSString(GetBIName(pbivnRTTISet_CompType));
    Prop.Expr:=CreateTypeInfoRef(El.EnumType,AContext,El);
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertRangeType(El: TPasRangeType;
  AContext: TConvertContext): TJSElement;
// create
//   module.$rtti.$Int("name",{
//       minvalue: <number>,
//       maxvalue: <number>,
//       ordtype: <number>
//     })
var
  TIObj: TJSObjectLiteral;
  Call: TJSCallExpression;
  MinVal, MaxVal: TResEvalValue;
  MinInt, MaxInt: TMaxPrecInt;
  OrdType: TOrdType;
  TIProp: TJSObjectLiteralElement;
  fn: TPas2JSBuiltInName;
begin
  Result:=nil;
  if not HasTypeInfo(El,AContext) then exit;

  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231112029);

  // module.$rtti.$Int("name",{...})
  MinVal:=nil;
  MaxVal:=nil;
  Call:=nil;
  try
    MinVal:=AContext.Resolver.EvalRangeLimit(El.RangeExpr,[refConst],true,El);
    MaxVal:=AContext.Resolver.EvalRangeLimit(El.RangeExpr,[refConst],false,El);
    if MinVal.Kind=revkInt then
      begin
      fn:=pbifnRTTINewInt;
      MinInt:=TResEvalInt(MinVal).Int;
      MaxInt:=TResEvalInt(MaxVal).Int;
      end
    else if MinVal.Kind=revkEnum then
      begin
      fn:=pbifnRTTINewEnum;
      MinInt:=TResEvalEnum(MinVal).Index;
      MaxInt:=TResEvalEnum(MaxVal).Index;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertRangeType type: ',MinVal.AsDebugString,'..',MaxVal.AsDebugString);
      {$ENDIF}
      RaiseNotSupported(El,AContext,20170925201628);
      end;
    OrdType:=GetOrdType(MinInt,MaxInt,El);
    Call:=CreateRTTINewType(El,GetBIName(fn),false,AContext,TIObj);
    // add  minvalue: number
    TIProp:=TIObj.Elements.AddElement;
    TIProp.Name:=TJSString(GetBIName(pbivnRTTIInt_MinValue));
    TIProp.Expr:=CreateLiteralNumber(El,MinInt);
    // add  maxvalue: number
    TIProp:=TIObj.Elements.AddElement;
    TIProp.Name:=TJSString(GetBIName(pbivnRTTIInt_MaxValue));
    TIProp.Expr:=CreateLiteralNumber(El,MaxInt);
    // add  ordtype: number
    TIProp:=TIObj.Elements.AddElement;
    TIProp.Name:=TJSString(GetBIName(pbivnRTTIInt_OrdType));
    TIProp.Expr:=CreateLiteralNumber(El,ord(OrdType));
    if MinVal.Kind=revkEnum then
      begin
      // add  enumtype: this.TypeName
      TIProp:=TIObj.Elements.AddElement;
      TIProp.Name:=TJSString(GetBIName(pbivnRTTIEnum_EnumType));
      TIProp.Expr:=CreateSubDeclPasNameExpr(El,TResEvalEnum(MinVal).ElType.Name,AContext); // use Pascal name
      end;
    Result:=Call;
  finally
    ReleaseEvalValue(MinVal);
    ReleaseEvalValue(MaxVal);
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertTypeAliasType(El: TPasTypeAliasType;
  AContext: TConvertContext): TJSElement;
// create
//   module.$rtti.$inherited(name,desttype,{});
var
  Obj: TJSObjectLiteral;
begin
  Result:=nil;
  if not HasTypeInfo(El,AContext) then exit;

  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231112029);

  Result:=CreateRTTINewType(El,GetBIName(pbifnRTTIInherited),false,AContext,Obj);
end;

function TPasToJSConverter.ConvertPointerType(El: TPasPointerType;
  AContext: TConvertContext): TJSElement;
// create
//   module.$rtti.$Pointer("name",{
//       reftype: module.$rtti["reftype"]
//     })
var
  Obj: TJSObjectLiteral;
  Call: TJSCallExpression;
  Prop: TJSObjectLiteralElement;
begin
  Result:=nil;
  if not HasTypeInfo(El,AContext) then exit;

  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231112029);

  // module.$rtti.$Pointer("name",{...})
  Call:=CreateRTTINewType(El,GetBIName(pbifnRTTIInherited),false,AContext,Obj);
  try
    // "comptype: ref"
    Prop:=Obj.Elements.AddElement;
    Prop.Name:=TJSString(GetBIName(pbivnRTTISet_CompType));
    Prop.Expr:=CreateTypeInfoRef(El.DestType,AContext,El);
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertProcedureType(El: TPasProcedureType;
  AContext: TConvertContext): TJSElement;
// create
// "reference to":
//   module.$rtti.$RefToProcVar("longname",{
//       procsig: rtl.newTIProcSignature([[arg1name,arg1type,arg1flags],[arg2name...],...],resulttype,flags)
//     })
// "of object":
//   module.$rtti.$MethodVar("longname",{
//       procsig: rtl.newTIProcSignature([[arg1name,arg1type,arg1flags],[arg2name...],...],resulttype,flags),
//       methodkind: 1
//     })
// "normal":
//   module.$rtti.$ProcVar("longname",{
//       procsig: rtl.newTIProcSignature([[arg1name,arg1type,arg1flags],[arg2name...],...],resulttype,flags)
//     })
// delayed specialization:
//   module.$rtti.$MethodVar("longname",{
//       init: function()}{ this.procsig = rtl.newTIProcSignature([[arg1name,arg1type,arg1flags],[arg2name...],...],resulttype,flags)},
//       methodkind: 1
//     })
var
  Call, InnerCall: TJSCallExpression;
  FunName: String;
  ResultEl: TPasResultElement;
  ResultTypeInfo: TJSElement;
  Flags: Integer;
  MethodKind: TMethodKind;
  Obj: TJSObjectLiteral;
  Prop: TJSObjectLiteralElement;
  aResolver: TPas2JSResolver;
  Scope: TPas2JSProcTypeScope;
  SpecializeDelay: Boolean;
  FuncSt: TJSFunctionDeclarationStatement;
  AssignSt: TJSSimpleAssignStatement;
  FuncContext: TFunctionContext;
  CurContext: TConvertContext;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  if not aResolver.IsFullySpecialized(El) then exit;
  if El.IsNested then
    DoError(20170222231636,nPasElementNotSupported,sPasElementNotSupported,
      ['is nested'],El);
  if not (El.CallingConvention in [ccDefault,ccSafeCall]) then
    DoError(20170222231532,nPasElementNotSupported,sPasElementNotSupported,
        ['calling convention '+cCallingConventions[El.CallingConvention]],El);
  if not HasTypeInfo(El,AContext) then
    exit; // no RTTI needed

  if El.Parent is TProcedureBody then
    RaiseNotSupported(El,AContext,20181231112029);

  Scope:=El.CustomData as TPas2JSProcTypeScope;
  SpecializeDelay:=(Scope<>nil) and SpecializeNeedsDelay(El,AContext);

  // module.$rtti.$ProcVar("name",function(){})
  if El.IsReferenceTo then
    FunName:=GetBIName(pbifnRTTINewRefToProcVar)
  else if El.IsOfObject then
    FunName:=GetBIName(pbifnRTTINewMethodVar)
  else
    FunName:=GetBIName(pbifnRTTINewProcVar);
  Call:=CreateRTTINewType(El,FunName,false,AContext,Obj);
  FuncContext:=nil;
  try
    // add "procsig: rtl.newTIProcSignature()"
    Prop:=Obj.Elements.AddElement;
    InnerCall:=CreateCallExpression(El);

    if SpecializeDelay then
      begin
      Prop.Name:=TJSString(GetBIName(pbivnRTTIProc_InitSpec));
      // init: function(){ this.procsig = rtl.newTIProcSignature(...) }
      FuncSt:=CreateFunctionSt(El);
      Prop.Expr:=FuncSt;
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AssignSt.LHS:=CreatePrimitiveDotExpr('this.'+GetBIName(pbivnRTTIProcVar_ProcSig),El);
      AssignSt.Expr:=InnerCall;
      FuncSt.AFunction.Body.A:=AssignSt;
      FuncContext:=TFunctionContext.Create(El,AssignSt,AContext);
      CurContext:=FuncContext;
      end
    else
      begin
      CurContext:=AContext;
      Prop.Name:=TJSString(GetBIName(pbivnRTTIProcVar_ProcSig));
      Prop.Expr:=InnerCall;
      end;

    InnerCall.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnRTTINewProcSig)]);
    // add array of arguments
    InnerCall.AddArg(CreateRTTIArgList(El,El.Args,CurContext));
    // add resulttype as typeinfo reference
    if El is TPasFunctionType then
      begin
      ResultEl:=TPasFunctionType(El).ResultEl;
      ResultTypeInfo:=CreateTypeInfoRef(ResultEl.ResultType,CurContext,ResultEl);
      if ResultTypeInfo<>nil then
        InnerCall.AddArg(ResultTypeInfo);
      end;
    // add procedure flags
    Flags:=0;
    if ptmVarargs in El.Modifiers then
      inc(Flags,pfVarargs);
    if ptmAsync in El.Modifiers then
      inc(Flags,pfAsync);
    if El.CallingConvention=ccSafeCall then
      inc(Flags,pfSafeCall);
    if Flags>0 then
      begin
      if not (El is TPasFunctionType) then
        InnerCall.AddArg(CreateLiteralNull(El));
      InnerCall.AddArg(CreateLiteralNumber(El,Flags));
      end;

    if El.IsOfObject then
      begin
      // add "methodkind: number;"
      Prop:=Obj.Elements.AddElement;
      Prop.Name:=TJSString(GetBIName(pbivnRTTIMethodKind));
      if El.ClassType=TPasProcedureType then
        MethodKind:=mkProcedure
      else if El.ClassType=TPasFunctionType then
        MethodKind:=mkFunction
      else
        RaiseNotSupported(El,AContext,20170411180848);
      Prop.Expr:=CreateLiteralNumber(El,ord(MethodKind));
      end;

    Result:=Call;
  finally
    FuncContext.Free;
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.ConvertArrayType(El: TPasArrayType;
  AContext: TConvertContext): TJSElement;
// Static array of static array need clone function:
//  this.TStaticArray$clone = function(a){
//    var r = [];
//    for (var i=0; i<*High(a)*; i++) r.push(a[i].slice(0));
//    return r;
//  };
//
// Published array types need:
//  module.$rtti.$StaticArray("name",{
//    dims: [dimsize1,dimsize2,...],
//    eltype: module.$rtti["ElTypeName"]
//  };
//  module.$rtti.$DynArray("name",{
//    eltype: module.$rtti["ElTypeName"]
//  };
//
const
  CloneArrName = 'a';
  CloneResultName = 'r';
  CloneRunName = 'i';
var
  ProcScope: TPas2JSProcedureScope;
  Src: TJSSourceElements;

  procedure StorePrecompiledJS(JS: TJSElement);
  begin
    // store precompiled enum type in proc
    if ProcScope=nil then
      ProcScope:=GetImplJSProcScope(El,Src,AContext);
    if ProcScope<>nil then
      ProcScope.AddGlobalJS(CreatePrecompiledJS(JS));
  end;

var
  aResolver: TPas2JSResolver;
  AssignSt: TJSSimpleAssignStatement;
  ArrName: String;
  Index: Integer;
  ElTypeLo: TPasType;
  RangeEl: TPasExpr;
  Call: TJSCallExpression;
  RangeEnd: TMaxPrecInt;
  List: TJSStatementList;
  Func: TJSFunctionDeclarationStatement;
  BodySrc: TJSSourceElements;
  VarSt: TJSVariableStatement;
  ForLoop: TJSForStatement;
  ExprLT: TJSRelationalExpressionLT;
  PlusPlus: TJSUnaryPostPlusPlusExpression;
  BracketEx: TJSBracketMemberExpression;
  ArraySt, CloneEl: TJSElement;
  ReturnSt: TJSReturnStatement;
  FuncContext: TFunctionContext;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  if not aResolver.IsFullySpecialized(El) then exit;
  if El.PackMode<>pmNone then
    DoError(20170222231648,nPasElementNotSupported,sPasElementNotSupported,
       ['packed'],El);
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertArrayType ',GetObjName(El));
  {$ENDIF}

  ProcScope:=nil;
  Src:=nil;
  if AContext.JSElement is TJSSourceElements then
    Src:=TJSSourceElements(AContext.JSElement);

  if aResolver.HasStaticArrayCloneFunc(El) then
    begin
    // For example: type TArr = array[1..2] of array[1..2] of longint;
    //  this.TStaticArray$clone = function(a){
    //    var r = [];
    //    for (var i=0; i<*High(a)*; i++) r.push(a[i].slice(0));
    //    return r;
    //  };
    BracketEx:=nil;
    AssignSt:=nil;
    Func:=nil;
    FuncContext:=nil;
    try
      Index:=0;
      RangeEl:=El.Ranges[Index];
      // function(a){...
      Func:=CreateFunctionSt(El,true,true);
      Func.AFunction.Params.Add(CloneArrName);
      BodySrc:=Func.AFunction.Body.A as TJSSourceElements;
      FuncContext:=TFunctionContext.Create(El,BodySrc,AContext);
      FuncContext.IsGlobal:=true;
      // var r = [];
      VarSt:=CreateVarStatement(CloneResultName,TJSArrayLiteral(CreateElement(TJSArrayLiteral,El)),El);
      AddToSourceElements(BodySrc,VarSt);
      // for (
      ForLoop:=TJSForStatement(CreateElement(TJSForStatement,El));
      AddToSourceElements(BodySrc,ForLoop);
      // var i=0;
      ForLoop.Init:=CreateVarStatement(CloneRunName,CreateLiteralNumber(El,0),El);
      // i<high(a)
      ExprLT:=TJSRelationalExpressionLT(CreateElement(TJSRelationalExpressionLT,El));
      ForLoop.Cond:=ExprLT;
      ExprLT.A:=CreatePrimitiveDotExpr(CloneRunName,El);
      RangeEnd:=aResolver.GetRangeLength(RangeEl);
      ExprLT.B:=CreateLiteralNumber(RangeEl,RangeEnd);
      // i++
      PlusPlus:=TJSUnaryPostPlusPlusExpression(CreateElement(TJSUnaryPostPlusPlusExpression,El));
      ForLoop.Incr:=PlusPlus;
      PlusPlus.A:=CreatePrimitiveDotExpr(CloneRunName,El);
      // r.push(...
      Call:=CreateCallExpression(El);
      ForLoop.Body:=Call;
      Call.Expr:=CreatePrimitiveDotExpr(CloneResultName+'.push',El);
      // a[i]
      BracketEx:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
      BracketEx.MExpr:=CreatePrimitiveDotExpr(CloneArrName,El);
      BracketEx.Name:=CreatePrimitiveDotExpr(CloneRunName,El);
      // clone a[i]
      ElTypeLo:=aResolver.ResolveAliasType(El.ElType);
      CloneEl:=nil;
      if ElTypeLo is TPasArrayType then
        begin
        if length(TPasArrayType(ElTypeLo).Ranges)=0 then
          RaiseNotSupported(El,FuncContext,20180218223414,GetObjName(ElTypeLo));
        CloneEl:=CreateCloneStaticArray(El,TPasArrayType(ElTypeLo),BracketEx,FuncContext);
        end
      else if ElTypeLo is TPasRecordType then
        CloneEl:=CreateRecordCallClone(El,TPasRecordType(ElTypeLo),BracketEx,FuncContext)
      else if ElTypeLo is TPasSetType then
        CloneEl:=CreateReferencedSet(El,BracketEx)
      else
        RaiseNotSupported(El,FuncContext,20180218223618,GetObjName(ElTypeLo));
      Call.AddArg(CloneEl);
      BracketEx:=nil;
      // return r;
      ReturnSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
      AddToSourceElements(BodySrc,ReturnSt);
      ReturnSt.Expr:=CreatePrimitiveDotExpr(CloneResultName,El);

      ArrName:=GetOverloadName(El,AContext)+GetBIName(pbifnArray_Static_Clone);
      if El.Parent is TProcedureBody then
        begin
        // local array type (elevated to global)
        // -> add 'var TypeName = function(){}'
        ArraySt:=CreateVarStatement(ArrName,Func,El);
        end
      else
        begin
        // global array type
        // -> add 'this.TypeName = function(){}'
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
        ArraySt:=AssignSt;
        AssignSt.LHS:=CreateSubDeclPasNameExpr(El,ArrName,AContext);
        AssignSt.Expr:=Func;
        end;
      Func:=nil;

      if Src<>nil then
        AddToSourceElements(Src,ArraySt)
      else
        Result:=ArraySt;

      // store precompiled array type in proc
      StorePrecompiledJS(ArraySt);

      ArraySt:=nil;
    finally
      BracketEx.Free;
      Func.Free;
      ArraySt.Free;
      FuncContext.Free;
    end;
    end;

  if (not (AContext.PasElement is TPasMembersType)) // rtti of members is added separate
      and HasTypeInfo(El,AContext) then
    begin
    // writeln('TPasToJSConverter.ConvertArrayType ',GetObjPath(El),' ',GetObjPath(AContext.PasElement));
    Call:=nil;
    try
      Call:=CreateRTTIAnonymousArray(El,AContext);

      if Src<>nil then
        begin
        AddToSourceElements(Src,Call);
        // store precompiled rtti call in proc
        StorePrecompiledJS(Call);
        end
      else if Result=nil then
        Result:=Call
      else
        begin
        List:=TJSStatementList(CreateElement(TJSStatementList,El));
        List.A:=Result;
        List.B:=Call;
        Result:=List;
        end;
      Call:=nil;
    finally
      Call.Free;
    end;
    end;
end;

function TPasToJSConverter.GetOrdType(MinValue, MaxValue: TMaxPrecInt;
  ErrorEl: TPasElement): TOrdType;
var
  V: TMaxPrecInt;
begin
  if MinValue<0 then
    begin
    if MaxValue<-(MinValue+1) then
      V:=-(MinValue+1)
    else
      V:=MaxValue;
    if V<$8f then
      Result:=otSByte
    else if V<$8fff then
      Result:=otSWord
    else if V<$8fffffff then
      Result:=otSLong
    else if V<=MaxSafeIntDouble then
      Result:=otSIntDouble
    else
      DoError(20170925200802,nRangeCheckError,sRangeCheckError,[],ErrorEl);
    end
  else
    begin
    if MaxValue<$ff then
      Result:=otUByte
    else if MaxValue<$ffff then
      Result:=otUWord
    else if MaxValue<$ffffffff then
      Result:=otULong
    else if MaxValue<=MaxSafeIntDouble then
      Result:=otUIntDouble
    else
      DoError(20170925201002,nRangeCheckError,sRangeCheckError,[],ErrorEl);
    end;
end;

{$IFDEF EnableForLoopRunnerCheck}
procedure TPasToJSConverter.ForLoop_OnProcBodyElement(El: TPasElement;
  arg: pointer);
// Called by ConvertForStatement on each element of the current proc body
// Check each element that lies behind the loop if it is reads the LoopVar
var
  Data: PForLoopFindData absolute arg;
begin
  if El.HasParent(Data^.ForLoop) then
    Data^.FoundLoop:=true
  else if Data^.FoundLoop and (not Data^.LoopVarWrite) and (not Data^.LoopVarRead) then
    begin
    // El comes after loop and LoopVar was not yet accessed
    if (El.CustomData is TResolvedReference)
        and (TResolvedReference(El.CustomData).Declaration=Data^.LoopVar) then
      begin
        // El refers the LoopVar
        // ToDo: check write only access
        Data^.LoopVarRead:=true;
      end;
    end;
end;
{$ENDIF}

procedure TPasToJSConverter.SetUseEnumNumbers(const AValue: boolean);
begin
  if AValue then
    Include(FOptions,coEnumNumbers)
  else
    Exclude(FOptions,coEnumNumbers);
end;

procedure TPasToJSConverter.SetUseLowerCase(const AValue: boolean);
begin
  if AValue then
    Include(FOptions,coLowerCase)
  else
    Exclude(FOptions,coLowerCase);
end;

procedure TPasToJSConverter.SetUseSwitchStatement(const AValue: boolean);
begin
  if AValue then
    Include(FOptions,coSwitchStatement)
  else
    Exclude(FOptions,coSwitchStatement);
end;

function TPasToJSConverter.OnCreateReferencePathExpr(El: TPasElement;
  AContext: TConvertContext; CreateRefPathData: Pointer): TJSElement;
var
  Data: PCreateRefPathData absolute CreateRefPathData;
begin
  Result:=CreateReferencePathExpr(Data^.El,AContext,Data^.Full,Data^.Ref);
  if El=nil then ;
end;

constructor TPasToJSConverter.Create;
begin
  FOptions:=DefaultPasToJSOptions;
end;

destructor TPasToJSConverter.Destroy;
begin
  Globals:=nil;
  inherited Destroy;
end;

function TPasToJSConverter.ConvertProcedure(El: TPasProcedure;
  AContext: TConvertContext): TJSElement;
var
  BodyJS: TJSFunctionBody;
  FirstSt, LastSt: TJSStatementList;

  procedure AddBodyStatement(Add: TJSElement; Src: TPasElement);
  begin
    AddToStatementList(FirstSt,LastSt,Add,Src);
    BodyJS.A:=FirstSt;
  end;

  procedure AddRangeCheckType(Arg: TPasArgument; aType: TPasType;
    AContext: TConvertContext);
  var
    GetExpr: TJSElement;
  begin
    GetExpr:=CreateArgumentAccess(Arg,AContext,Arg);
    AddBodyStatement(CreateRangeCheckCall_TypeRange(aType,GetExpr,AContext,Arg),Arg);
  end;

Var
  FS : TJSFunctionDeclarationStatement;
  FD : TJSFuncDef;
  n, i, Line, Col:Integer;
  AssignSt, AssignSt2: TJSSimpleAssignStatement;
  FuncContext, ConstContext: TFunctionContext;
  ProcScope, ImplProcScope: TPas2JSProcedureScope;
  Arg, SelfArg: TPasArgument;
  SelfSt: TJSVariableStatement;
  ImplProc: TPasProcedure;
  BodyPas: TProcedureBody;
  PosEl, ThisPas: TPasElement;
  Call: TJSCallExpression;
  ClassPath, aName: String;
  ArgResolved: TPasResolverResult;
  Lit: TJSLiteral;
  ConstSrcElems: TJSSourceElements;
  ArgTypeEl, HelperForType: TPasType;
  aResolver: TPas2JSResolver;
  IsClassConDestructor: Boolean;
  ThisKind: TCtxVarKind;
  ImplJS: TPas2JSPrecompiledJS;
begin
  Result:=nil;

  if El.IsAbstract then exit;
  if El.IsExternal then exit;

  ProcScope:=TPas2JSProcedureScope(El.CustomData);
  if ProcScope.DeclarationProc<>nil then
    exit;
  IsClassConDestructor:=(El.ClassType=TPasClassConstructor)
                     or (El.ClassType=TPasClassDestructor);
  aResolver:=AContext.Resolver;
  if not aResolver.IsFullySpecialized(El) then exit;

  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertProcedure "',El.Name,'" Overload="',ProcScope.OverloadName,'" ',El.Parent.ClassName);
  {$ENDIF}

  ImplProc:=El;
  if ProcScope.ImplProc<>nil then
    ImplProc:=ProcScope.ImplProc;
  ImplProcScope:=TPas2JSProcedureScope(ImplProc.CustomData);

  ImplJS:=ImplProcScope.ImplJS;
  if ImplJS<>nil then
    begin
    // using precompiled code
    TPasResolver.UnmangleSourceLineNumber(El.SourceLinenumber,Line,Col);
    if ImplJS.GlobalJS<>nil then
      begin
      ConstContext:=AContext.GetGlobalFunc;
      if not (ConstContext.JSElement is TJSSourceElements) then
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.ConvertProcedure ConstContext=',GetObjName(ConstContext),' JSElement=',GetObjName(ConstContext.JSElement));
        {$ENDIF}
        RaiseNotSupported(El,AContext,20180228231008);
        end;
      ConstSrcElems:=TJSSourceElements(ConstContext.JSElement);
      for i:=0 to ImplJS.GlobalJS.Count-1 do
        begin
        // precompiled global var or type
        Lit:=TJSLiteral.Create(Line,Col,El.SourceFilename);
        Lit.Value.CustomValue:=StrToJSString(ImplJS.GlobalJS[i]);
        AddToSourceElements(ConstSrcElems,Lit);
        end;
      end;

    if coShortRefGlobals in Options then
      CreateGlobalAlias_List(ImplJS.ShortRefs,AContext);
    // precompiled body
    Lit:=TJSLiteral.Create(Line,Col,El.SourceFilename);
    Lit.Value.CustomValue:=StrToJSString(ImplJS.BodyJS);
    Result:=Lit;
    exit;
    end
  else if (coStoreImplJS in Options) and (aResolver<>nil) then
    begin
    if aResolver.ProcCanBePrecompiled(El) then
      begin
      ImplJS:=TPas2JSPrecompiledJS.Create;
      ImplProcScope.ImplJS:=ImplJS;
      end;
    end;

  AssignSt:=nil;
  if AContext.IsGlobal then
    begin
    // add 'this.FuncName = ...'
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,ImplProc));
    Result:=AssignSt;
    AssignSt.LHS:=CreateSubDeclNameExpr(El,AContext,ImplProc);

    if (coShortRefGlobals in Options) then
      begin
      aName:=AContext.GetLocalName(El,[cvkGlobal]);
      if aName<>'' then
        begin
        // this.FuncName = $lp = ...;
        AssignSt2:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,ImplProc));
        AssignSt.Expr:=AssignSt2;
        AssignSt:=AssignSt2;
        AssignSt.LHS:=CreatePrimitiveDotExpr(aName,El);
        end;
      end;
    end;

  FS:=CreateFunctionSt(ImplProc,ImplProc.Body<>nil);
  FD:=FS.AFunction;
  FD.IsAsync:=El.IsAsync or ImplProc.IsAsync;
  if AssignSt<>nil then
    AssignSt.Expr:=FS
  else
    begin
    // local/nested or anonymous function
    Result:=FS;
    if (El.Name<>'') and not IsClassConDestructor then
      FD.Name:=TJSString(TransformElToJSName(El,AContext));
    end;

  for n := 0 to El.ProcType.Args.Count - 1 do
    begin
    Arg:=TPasArgument(El.ProcType.Args[n]);
    FD.Params.Add(TransformElToJSName(Arg,AContext));
    end;

  BodyPas:=ImplProc.Body;
  if BodyPas<>nil then
    begin
    PosEl:=BodyPas;
    if PosEl=nil then
      PosEl:=ImplProc;
    BodyJS:=FD.Body;
    FuncContext:=TFunctionContext.Create(ImplProc,FD.Body,AContext);
    try
      FuncContext.ScannerBoolSwitches:=ImplProcScope.BoolSwitches;
      FirstSt:=nil;
      LastSt:=nil;

      if ProcScope.ClassRecScope<>nil then
        begin
        ThisKind:=cvkNone;
        SelfArg:=nil;
        // method or class method
        //writeln('TPasToJSConverter.ConvertProcedure El=',GetObjPath(El),' IsStatic=',El.IsStatic);
        if not AContext.IsGlobal then
          begin
          // nested sub procedure  ->  no 'this'
          ThisPas:=nil;
          end
        else if El.IsStatic or IsClassConDestructor then
          ThisPas:=nil
        else
          begin
          ThisPas:=ProcScope.ClassRecScope.Element;
          if aResolver.IsHelper(ThisPas) then
            begin
            // helper method
            HelperForType:=aResolver.ResolveAliasType(TPasClassType(ThisPas).HelperForType);
            ThisPas:=HelperForType;
            if HelperForType is TPasMembersType then
              begin
              // 'this' in a class/record helper method is the class (instance)
              ThisKind:=cvkInstance;
              end
            else
              begin
              // 'this' in a type helper is a temporary getter/setter JS object
              ThisKind:=cvkHelperTemp;
              end;
            end
          else if aResolver.IsClassMethod(El) then
            ThisKind:=cvkCurType
          else
            ThisKind:=cvkInstance;
          if ProcScope.ImplProc<>nil then
            SelfArg:=TPas2JSProcedureScope(ProcScope.ImplProc.CustomData).SelfArg
          else
            SelfArg:=ProcScope.SelfArg;
          //writeln('TPasToJSConverter.ConvertProcedure El=',GetObjPath(El),' SelfArg=',GetObjPath(SelfArg));
          end;
        FuncContext.ThisVar.Element:=ThisPas;
        FuncContext.ThisVar.Kind:=ThisKind;

        if ThisPas<>nil then
          begin

          if (bsObjectChecks in FuncContext.ScannerBoolSwitches)
              and (ThisKind in [cvkGlobal,cvkInstance,cvkCurType]) then
            begin
            // rtl.checkMethodCall(this,<class>)
            Call:=CreateCallExpression(PosEl);
            AddBodyStatement(Call,PosEl);
            Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),
                                            GetBIName(pbifnCheckMethodCall)]);
            Call.AddArg(CreatePrimitiveDotExpr('this',PosEl));
            ClassPath:=CreateReferencePath(ProcScope.ClassRecScope.Element,AContext,rpkPathAndName);
            Call.AddArg(CreatePrimitiveDotExpr(ClassPath,PosEl));
            end;

          //writeln('TPasToJSConverter.ConvertProcedure El=',GetObjPath(El),' SelfArg=',GetObjPath(SelfArg),' ThisPas=',GetObjPath(ThisPas));
          if (ImplProc.Body.Functions.Count>0)
              or aResolver.HasAnonymousFunctions(ImplProc.Body.Body) then
            begin
            // has nested procs -> add "var $Self = this;"
            FuncContext.AddLocalVar(GetBIName(pbivnSelf),ThisPas,ThisKind,false);
            SelfSt:=CreateVarStatement(GetBIName(pbivnSelf),
                              CreatePrimitiveDotExpr('this',ImplProc),ImplProc);
            AddBodyStatement(SelfSt,PosEl);
            if (SelfArg<>nil) and (ThisPas<>SelfArg) then
              begin
              // add alias (two PasElements for "this")
              FuncContext.AddLocalVar(GetBIName(pbivnSelf),SelfArg,ThisKind,false);
              end;
            end
          else if (SelfArg<>nil) and (ThisPas<>SelfArg) then
            begin
            // add alias (two PasElements for "this")
            FuncContext.AddLocalVar('this',SelfArg,ThisKind,false);
            end;
          end;
        end;

      if (bsRangeChecks in ImplProcScope.BoolSwitches) and (aResolver<>nil) then
        for i:=0 to El.ProcType.Args.Count-1 do
          begin
          Arg:=TPasArgument(El.ProcType.Args[i]);
          if Arg.ArgType=nil then continue;
          aResolver.ComputeElement(Arg,ArgResolved,[rcType]);
          ArgTypeEl:=ArgResolved.LoTypeEl;
          if ArgTypeEl=nil then continue;
          if ArgResolved.BaseType in btAllJSRangeCheckTypes then
            AddRangeCheckType(Arg,ArgTypeEl,FuncContext)
          else if ArgResolved.BaseType=btContext then
            begin
            if ArgTypeEl.ClassType=TPasEnumType then
              AddRangeCheckType(Arg,ArgTypeEl,FuncContext);
            end
          else if ArgResolved.BaseType=btRange then
            begin
            if ArgResolved.SubType in btAllJSRangeCheckTypes then
              AddRangeCheckType(Arg,ArgTypeEl,FuncContext)
            else if ArgResolved.SubType=btContext then
              AddRangeCheckType(Arg,ArgTypeEl,FuncContext)
            else
              begin
              {$IFDEF VerbosePas2JS}
              writeln('TPasToJSConverter.ConvertProcedure ',GetResolverResultDbg(ArgResolved));
              RaiseNotSupported(Arg,AContext,20180424120701);
              {$ENDIF}
              end;
            end;
          end;
      {$IFDEF VerbosePas2JS}
      //FuncContext.WriteStack;
      {$ENDIF}
      if BodyPas<>nil then
        AddBodyStatement(ConvertDeclarations(BodyPas,FuncContext),BodyPas);
    finally
      FuncContext.Free;
    end;
    end;

  if ImplJS<>nil then
    begin
    ImplJS.BodyJS:=CreatePrecompiledJS(Result);
    ImplJS.EmptyJS:=BodyPas.Body=nil;
    end;
end;

function TPasToJSConverter.ConvertBeginEndStatement(El: TPasImplBeginBlock;
  AContext: TConvertContext; NilIfEmpty: boolean): TJSElement;

begin
  Result:=ConvertImplBlockElements(El,AContext,NilIfEmpty);
end;

function TPasToJSConverter.ConvertImplBlockElements(El: TPasImplBlock;
  AContext: TConvertContext; NilIfEmpty: boolean): TJSElement;
var
  First, Last: TJSStatementList;
  I : Integer;
  PasImpl: TPasImplElement;
  JSImpl : TJSElement;
begin
  if Not (Assigned(El.Elements) and (El.Elements.Count>0)) then
    begin
    if NilIfEmpty then
      Result:=nil
    else
      Result:=TJSEmptyBlockStatement(CreateElement(TJSEmptyBlockStatement,El));
    end
  else
    begin
    Result:=nil;
    try
      First:=nil;
      Last:=nil;
      //writeln('TPasToJSConverter.ConvertImplBlockElements START El.Elements.Count=',El.Elements.Count);
      For I:=0 to El.Elements.Count-1 do
        begin
        PasImpl:=TPasImplElement(El.Elements[i]);
        JSImpl:=ConvertElement(PasImpl,AContext);
        if JSImpl=nil then
          continue; // e.g. "inherited;" when there is no ancestor proc
        //writeln('TPasToJSConverter.ConvertImplBlockElements ',i,' ',JSImpl.ClassName);
        AddToStatementList(First,Last,JSImpl,PasImpl);
        end;
      Result:=First;
    finally
      if Result=nil then
        First.Free;
    end;
    end;
end;

function TPasToJSConverter.ConvertInitializationSection(El: TPasModule;
  AContext: TConvertContext): TJSElement;
var
  FDS: TJSFunctionDeclarationStatement;
  FuncContext: TFunctionContext;
  PosEl: TPasElement;

  function CreateBody: TJSFunctionBody;
  var
    FuncDef: TJSFuncDef;
  begin
    FuncDef:=FDS.AFunction;
    Result:=FuncDef.Body;
    if Result=nil then
      begin
      Result:=TJSFunctionBody(CreateElement(TJSFunctionBody,PosEl));
      FuncDef.Body:=Result;
      Result.A:=TJSSourceElements(CreateElement(TJSSourceElements, PosEl));
      end;
    if FuncContext=nil then
      FuncContext:=TFunctionContext.Create(PosEl,Result,AContext);
  end;

var
  FunName, S: String;
  IsMain, NeedRTLCheckVersion: Boolean;
  AssignSt: TJSSimpleAssignStatement;
  Body: TJSFunctionBody;
  Scope: TPas2JSInitialFinalizationScope;
  Line, Col: integer;
  Lit: TJSLiteral;
  Section: TInitializationSection;
  RootContext: TRootContext;
  ImplJS: TPas2JSPrecompiledJS;
begin
  // create: '$mod.$init=function(){}'
  Result:=nil;

  Section:=El.InitializationSection;
  if Section<>nil then
    begin
    PosEl:=Section;
    Scope:=TPas2JSInitialFinalizationScope(Section.CustomData);
    end
  else
    begin
    PosEl:=El;
    Scope:=nil;
    end;

  if El.ClassType=TPasProgram then
    begin
    IsMain:=true;
    FunName:=GetBIName(pbifnProgramMain)
    end
  else if El.ClassType=TPasLibrary then
    begin
    IsMain:=true;
    FunName:=GetBIName(pbifnLibraryMain)
    end
  else
    begin
    IsMain:=false;
    FunName:=GetBIName(pbifnUnitInit);
    end;
  NeedRTLCheckVersion:=IsMain and (coRTLVersionCheckMain in Options);

  RootContext:=AContext.GetRootContext as TRootContext;
  FuncContext:=nil;
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
  try
    // $mod.$init =
    AssignSt.LHS:=CreateMemberExpression([GetBIName(pbivnModule),FunName]);
    // = function(){...}
    FDS:=CreateFunctionSt(PosEl,false);
    AssignSt.Expr:=FDS;
    Body:=FDS.AFunction.Body;

    // first convert main/initialization statements
    if Section<>nil then
      begin
      ImplJS:=Scope.ImplJS;
      if ImplJS<>nil then
        begin
        S:=TrimRight(ImplJS.BodyJS);
        if S<>'' then
          begin
          Body:=CreateBody;
          // use precompiled JS
          TPasResolver.UnmangleSourceLineNumber(El.SourceLinenumber,Line,Col);
          Lit:=TJSLiteral.Create(Line,Col,El.SourceFilename);
          Lit.Value.CustomValue:=StrToJSString(S);
          Body.A:=Lit;
          if coShortRefGlobals in Options then
            CreateGlobalAlias_List(ImplJS.ShortRefs,AContext);
          end;
        end
      else
        begin
        if (coStoreImplJS in Options) and (AContext.Resolver<>nil) then
          begin
          ImplJS:=TPas2JSPrecompiledJS.Create;
          Scope.ImplJS:=ImplJS;
          end;
        if Section.Elements.Count>0 then
          begin
          Body:=CreateBody;
          // Note: although the rtl sets 'this' as the module, the function can
          //   simply refer to $mod, so no need to set ThisPas here
          Body.A:=ConvertImplBlockElements(Section,FuncContext,false);
          FuncContext.BodySt:=Body.A;

          AddInterfaceReleases(FuncContext,PosEl);
          Body.A:=FuncContext.BodySt;

          // store precompiled JS
          if ImplJS<>nil then
            begin
            ImplJS.BodyJS:=TrimRight(CreatePrecompiledJS(Body.A));
            ImplJS.EmptyJS:=ImplJS.BodyJS=''; // store the information, that there is an empty initialization section
            end;
          end
        else if ImplJS<>nil then
          ImplJS.EmptyJS:=true; // store the information, that there is an empty initialization section
        end
      end;

    if length(RootContext.GlobalClassMethods)>0 then
      begin
      // prepend class constructors (which one depends on WPO)
      Body:=CreateBody;
      AddClassConstructors(FuncContext,El);
      Body.A:=FuncContext.BodySt;
      end;

    if NeedRTLCheckVersion then
      begin
      // prepend rtl.versionCheck
      Body:=CreateBody;
      AddRTLVersionCheck(FuncContext,El);
      Body.A:=FuncContext.BodySt;
      end;

    Result:=AssignSt;
  finally
    FuncContext.Free;
    if Result=nil then
      AssignSt.Free;
  end;
end;

function TPasToJSConverter.ConvertFinalizationSection(El: TFinalizationSection;
  AContext: TConvertContext): TJSElement;
begin
  Result:=nil;
  RaiseNotSupported(El,AContext,20161024192519);
end;

function TPasToJSConverter.ConvertTryStatement(El: TPasImplTry;
  AContext: TConvertContext): TJSElement;
Var
  T : TJSTryStatement;
  ExceptBlock: TPasImplTryHandler;
  i: Integer;
  ExceptOn: TPasImplExceptOn;
  IfSt, Last: TJSIfStatement;

begin
  Result:=nil;
  T:=nil;
  try
    if El.FinallyExcept is TPasImplTryFinally then
      begin
      T:=TJSTryFinallyStatement(CreateElement(TJSTryFinallyStatement,El));
      T.Block:=ConvertImplBlockElements(El,AContext,true);
      T.BFinally:=ConvertImplBlockElements(El.FinallyExcept,AContext,true);
      end
    else
      begin
      T:=TJSTryCatchStatement(CreateElement(TJSTryCatchStatement,El));
      T.Block:=ConvertImplBlockElements(El,AContext,true);
      // always set the catch except object, needed by nodejs
      T.Ident:=TJSString(GetBIName(pbivnExceptObject));
      ExceptBlock:=El.FinallyExcept;
      if (ExceptBlock.Elements.Count>0)
          and (TPasImplElement(ExceptBlock.Elements[0]) is TPasImplExceptOn) then
        begin
        Last:=nil;
        for i:=0 to ExceptBlock.Elements.Count-1 do
          begin
          ExceptOn:=TObject(ExceptBlock.Elements[i]) as TPasImplExceptOn;
          IfSt:=ConvertExceptOn(ExceptOn,AContext) as TJSIfStatement;
          if Last=nil then
            T.BCatch:=IfSt
          else
            Last.BFalse:=IfSt;
          Last:=IfSt;
          end;
        if El.ElseBranch<>nil then
          Last.BFalse:=ConvertImplBlockElements(El.ElseBranch,AContext,true)
        else
          begin
          // default else: throw exceptobject
          Last.BFalse:=TJSThrowStatement(CreateElement(TJSThrowStatement,El));
          TJSThrowStatement(Last.BFalse).A:=
            CreatePrimitiveDotExpr(GetBIName(pbivnExceptObject),El);
          end;
        end
      else
        begin
        if El.ElseBranch<>nil then
          RaiseNotSupported(El.ElseBranch,AContext,20170205003014);
        T.BCatch:=ConvertImplBlockElements(ExceptBlock,AContext,true);
        end;
      end;
    Result:=T;
  finally
    if Result=nil then
      T.Free;
  end;
end;

function TPasToJSConverter.ConvertCaseOfStatement(El: TPasImplCaseOf;
  AContext: TConvertContext): TJSElement;
var
  SubEl: TPasImplElement;
  St: TPasImplCaseStatement;
  ok, IsCaseOfString: Boolean;
  i, j: Integer;
  JSExpr: TJSElement;
  StList: TJSStatementList;
  Expr: TPasExpr;
  IfSt, LastIfSt: TJSIfStatement;
  TmpVar: TFCLocalIdentifier;
  VarDecl: TJSVarDeclaration;
  VarSt: TJSVariableStatement;
  JSOrExpr: TJSLogicalOrExpression;
  JSAndExpr: TJSLogicalAndExpression;
  JSLEExpr: TJSRelationalExpressionLE;
  JSGEExpr: TJSRelationalExpressionGE;
  JSEQExpr: TJSEqualityExpressionSEQ;
  aResolver: TPas2JSResolver;
  CaseResolved: TPasResolverResult;
  FuncCtx: TFunctionContext;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;

  IsCaseOfString:=false;
  if aResolver<>nil then
    begin
    aResolver.ComputeElement(El.CaseExpr,CaseResolved,[]);
    if CaseResolved.BaseType in btAllStrings then
      IsCaseOfString:=true;
    end;

  if UseSwitchStatement then
    begin
    // convert to switch statement
    // switch does not support ranges -> check
    ok:=true;
    for i:=0 to El.Elements.Count-1 do
      begin
      SubEl:=TPasImplElement(El.Elements[i]);
      if not (SubEl is TPasImplCaseStatement) then
        continue;
      St:=TPasImplCaseStatement(SubEl);
      for j:=0 to St.Expressions.Count-1 do
        begin
        Expr:=TPasExpr(St.Expressions[j]);
        if (Expr is TBinaryExpr) and (TBinaryExpr(Expr).Kind=pekRange) then
          begin
          ok:=false;
          break;
          end;
        end;
      if not ok then break;
      end;
    if ok then
      begin
      Result:=CreateSwitchStatement(El,AContext);
      exit;
      end;
    end;

  // convert to if statements
  StList:=TJSStatementList(CreateElement(TJSStatementList,El));
  ok:=false;
  try
    // create var $tmp1=CaseExpr;
    FuncCtx:=AContext.GetFunctionContext;
    if FuncCtx=nil then
      RaiseNotSupported(El,AContext,20200608132048);
    TmpVar:=FuncCtx.AddLocalVar('$tmp',El.CaseExpr,cvkNone,true);
    VarSt:=TJSVariableStatement(CreateElement(TJSVariableStatement,El.CaseExpr));
    StList.A:=VarSt;
    VarDecl:=TJSVarDeclaration(CreateElement(TJSVarDeclaration,El.CaseExpr));
    VarSt.A:=VarDecl;
    VarDecl.Name:=TmpVar.Name;
    VarDecl.Init:=ConvertExpression(El.CaseExpr,AContext);

    LastIfSt:=nil;
    for i:=0 to El.Elements.Count-1 do
      begin
      SubEl:=TPasImplElement(El.Elements[i]);
      if SubEl is TPasImplCaseStatement then
        begin
        St:=TPasImplCaseStatement(SubEl);
        // create for example "if (tmp==expr) || ((tmp>=expr) && (tmp<=expr)){}"
        IfSt:=TJSIfStatement(CreateElement(TJSIfStatement,SubEl));
        if LastIfSt=nil then
          StList.B:=IfSt
        else
          LastIfSt.BFalse:=IfSt;
        LastIfSt:=IfSt;

        for j:=0 to St.Expressions.Count-1 do
          begin
          Expr:=TPasExpr(St.Expressions[j]);
          if (Expr is TBinaryExpr) and (TBinaryExpr(Expr).Kind=pekRange) then
            begin
            // range -> create "(tmp>=left) && (tmp<=right)"
            // create "() && ()"
            JSAndExpr:=TJSLogicalAndExpression(CreateElement(TJSLogicalAndExpression,Expr));
            JSExpr:=JSAndExpr;
            // create "tmp>=left"
            JSGEExpr:=TJSRelationalExpressionGE(CreateElement(TJSRelationalExpressionGE,Expr));
            JSAndExpr.A:=JSGEExpr;
            JSGEExpr.A:=CreatePrimitiveDotExpr(TmpVar.Name,El.CaseExpr);
            JSGEExpr.B:=ConvertExpression(TBinaryExpr(Expr).left,AContext);
            // create "tmp<=right"
            JSLEExpr:=TJSRelationalExpressionLE(CreateElement(TJSRelationalExpressionLE,Expr));
            JSAndExpr.B:=JSLEExpr;
            JSLEExpr.A:=CreatePrimitiveDotExpr(TmpVar.Name,El.CaseExpr);
            JSLEExpr.B:=ConvertExpression(TBinaryExpr(Expr).right,AContext);
            if IsCaseOfString then
              begin
              // case of string, range  ->  "(tmp.length===1) &&"
              JSEQExpr:=TJSEqualityExpressionSEQ(CreateElement(TJSEqualityExpressionSEQ,Expr));
              JSEQExpr.A:=CreateDotNameExpr(Expr,
                            CreatePrimitiveDotExpr(TmpVar.Name,El.CaseExpr),
                            'length');
              JSEQExpr.B:=CreateLiteralNumber(Expr,1);
              JSAndExpr:=TJSLogicalAndExpression(CreateElement(TJSLogicalAndExpression,Expr));
              JSAndExpr.A:=JSEQExpr;
              JSAndExpr.B:=JSExpr;
              JSExpr:=JSAndExpr;
              end;
            end
          else
            begin
            // value -> create (tmp===Expr)
            JSEQExpr:=TJSEqualityExpressionSEQ(CreateElement(TJSEqualityExpressionSEQ,Expr));
            JSExpr:=JSEQExpr;
            JSEQExpr.A:=CreatePrimitiveDotExpr(TmpVar.Name,El.CaseExpr);
            JSEQExpr.B:=ConvertExpression(Expr,AContext);
            end;
          if IfSt.Cond=nil then
            // first expression
            IfSt.Cond:=JSExpr
          else
            begin
            // multi expression -> append with OR
            JSOrExpr:=TJSLogicalOrExpression(CreateElement(TJSLogicalOrExpression,St));
            JSOrExpr.A:=IfSt.Cond;
            JSOrExpr.B:=JSExpr;
            IfSt.Cond:=JSOrExpr;
            end;
          end;
        // convert statement
        if St.Body<>nil then
          IfSt.BTrue:=ConvertElement(St.Body,AContext)
        else
          IfSt.BTrue:=TJSEmptyStatement(CreateElement(TJSEmptyStatement,St));
        end
      else if SubEl is TPasImplCaseElse then
        begin
        // Pascal 'else' or 'otherwise' -> create JS "else{}"
        if LastIfSt=nil then
          RaiseNotSupported(SubEl,AContext,20161128120802,'case-of needs at least one case');
        LastIfSt.BFalse:=ConvertImplBlockElements(El.ElseBranch,AContext,true);
        end
      else
        RaiseNotSupported(SubEl,AContext,20161128113055);
      end;

    ok:=true;
  finally
    if not ok then
      StList.Free;
  end;
  Result:=StList;
end;

function TPasToJSConverter.ConvertAsmStatement(El: TPasImplAsmStatement;
  AContext: TConvertContext): TJSElement;
var
  s: String;
  L: TJSLiteral;
  AsmLines: TStrings;
  Line, Col, StartLine: integer;
  Statements: TJSStatementList;
begin
  if AContext=nil then ;
  AsmLines:=El.Tokens;
  s:=Trim(AsmLines.Text);
  if (s<>'') and (s[length(s)]=';') then
    Delete(s,length(s),1);
  if s='' then
    Result:=TJSEmptyStatement(CreateElement(TJSEmptyStatement,El))
  else begin
    StartLine:=0;
    while (StartLine<AsmLines.Count) and (Trim(AsmLines[StartLine])='') do
      inc(StartLine);
    TPasResolver.UnmangleSourceLineNumber(El.SourceLinenumber,Line,Col);
    if StartLine>0 then
      Col:=1;
    L:=TJSLiteral.Create(Line+StartLine,Col,El.SourceFilename);
    L.Value.CustomValue:=TJSString(s);
    Result:=L;
    if Pos(';',s)>0 then
      begin
      // multi statement JS
      // for example "if e then asm a;b end;"
      //       ->     if (e){ a;b }
      Statements:=TJSStatementList.Create(L.Line,L.Column,L.Source);
      Statements.A:=L;
      Result:=Statements;
      end;
  end;
end;

function TPasToJSConverter.ConvertConstValue(Value: TResEvalValue;
  AContext: TConvertContext; El: TPasElement): TJSElement;
var
  Ranges: TResEvalSet.TItems;
  Range: TResEvalSet.TItem;
  Call: TJSCallExpression;
  i: Integer;
begin
  Result:=nil;
  if Value=nil then
    RaiseNotSupported(El,AContext,20170910211948);
  case Value.Kind of
  revkNil:
    Result:=CreateLiteralNull(El);
  revkBool:
    Result:=CreateLiteralBoolean(El,TResEvalBool(Value).B);
  revkInt:
    Result:=CreateLiteralNumber(El,TResEvalInt(Value).Int);
  revkUInt:
    Result:=CreateLiteralNumber(El,TResEvalUInt(Value).UInt);
  revkFloat:
    Result:=CreateLiteralNumber(El,TResEvalFloat(Value).FloatValue);
  {$IFDEF FPC_HAS_CPSTRING}
  revkString:
    Result:=CreateLiteralString(El,TResEvalString(Value).S);
  {$ENDIF}
  revkUnicodeString:
    Result:=CreateLiteralJSString(El,TResEvalUTF16(Value).S);
  revkEnum:
    Result:=CreateReferencePathExpr(TResEvalEnum(Value).GetEnumValue,AContext);
  revkSetOfInt:
    if Value.IdentEl is TPasExpr then
      Result:=ConvertExpression(TPasExpr(Value.IdentEl),AContext)
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertConstValue Value=',Value.AsDebugString,' IdentEl=',GetObjName(Value.IdentEl));
      {$ENDIF}
      // rtl.createSet()
      Call:=CreateCallExpression(El);
      try
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnSet_Create)]);
        Ranges:=TResEvalSet(Value).Ranges;
        for i:=0 to length(Ranges)-1 do
          begin
          Range:=Ranges[i];
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.ConvertConstValue SetLiteral ',i,' ',Range.RangeStart,'..',Range.RangeEnd);
          {$ENDIF}
          if Range.RangeStart=Range.RangeEnd then
            begin
            // add one integer
            Call.AddArg(CreateLiteralNumber(El,Range.RangeStart));
            end
          else
            begin
            // range -> add three parameters: null,left,right
            Call.AddArg(CreateLiteralNull(El));
            Call.AddArg(CreateLiteralNumber(El,Range.RangeStart));
            Call.AddArg(CreateLiteralNumber(El,Range.RangeEnd));
            end;
          end;
        Result:=Call;
      finally
        if Result=nil then
          Call.Free;
      end;
      end
  else
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertConstValue Value=',Value.AsDebugString);
    {$ENDIF}
    RaiseNotSupported(El,AContext,20170910211951);
  end;
end;

function TPasToJSConverter.CreateImplementationSection(El: TPasModule;
  IntfContext: TInterfaceSectionContext): TJSFunctionDeclarationStatement;
var
  Src: TJSSourceElements;
  ImplContext: TSectionContext;
  ImplDecl, JS: TJSElement;
  FunDecl: TJSFunctionDeclarationStatement;
  i: Integer;
begin
  Result:=nil;

  // create function(){}
  FunDecl:=CreateFunctionSt(El.ImplementationSection,true,true);
  Src:=TJSSourceElements(FunDecl.AFunction.Body.A);
  IntfContext.ImplSrcElements:=Src;

  // create section context (a function)
  ImplContext:=TSectionContext.Create(El.ImplementationSection,Src,IntfContext);
  try
    IntfContext.ImplContext:=ImplContext;
    // ToDo: IntfContext.ThisPas:=El;
    // ToDo: IntfContext.ThisKind:=cvkGlobal;

    // add pending impl header statements
    if IntfContext.ImplHeaderStatements<>nil then
      begin
      for i:=0 to IntfContext.ImplHeaderStatements.Count-1 do
        begin
        JS:=TJSElement(IntfContext.ImplHeaderStatements[i]);
        ImplContext.AddHeaderStatement(JS);
        IntfContext.ImplHeaderStatements[i]:=nil;
        end;
      FreeAndNil(IntfContext.ImplHeaderStatements);
      end;

    // create implementation declarations
    if El.ImplementationSection<>nil then
      begin
      ImplDecl:=ConvertDeclarations(El.ImplementationSection,ImplContext);
      if ImplDecl<>nil then
        RaiseInconsistency(20170910175032,El); // elements should have been added directly
      end;
    IntfContext.ImplHeaderIndex:=ImplContext.HeaderIndex;
    Result:=FunDecl;
  finally
    IntfContext.ImplContext:=nil;
    ImplContext.Free;
    if Result=nil then
      begin
      FunDecl.Free;
      IntfContext.ImplSrcElements:=nil;
      end;
  end;
end;

procedure TPasToJSConverter.CreateInitSection(El: TPasModule;
  Src: TJSSourceElements; AContext: TConvertContext);
var
  RootContext: TRootContext;
begin
  RootContext:=AContext.GetRootContext as TRootContext;
  // add initialization section
  if Assigned(El.InitializationSection)
      or (length(RootContext.GlobalClassMethods)>0) then
    AddToSourceElements(Src,ConvertInitializationSection(El,AContext));
  // finalization: not supported
  if Assigned(El.FinalizationSection) then
    raise Exception.Create('TPasToJSConverter.ConvertInitializationSection: finalization section is not supported');
end;

procedure TPasToJSConverter.AddHeaderStatement(JS: TJSElement;
  PosEl: TPasElement; aContext: TConvertContext);
var
  SectionCtx: TSectionContext;
begin
  if JS=nil then exit;
  SectionCtx:=TSectionContext(aContext.GetMainSectionContext);
  if SectionCtx=nil then
    RaiseNotSupported(PosEl,aContext,20200606142555);
  SectionCtx.AddHeaderStatement(JS);
end;

procedure TPasToJSConverter.AddImplHeaderStatement(JS: TJSElement;
  PosEl: TPasElement; aContext: TConvertContext);
var
  IntfSec: TInterfaceSectionContext;
begin
  if JS=nil then exit;
  IntfSec:=TInterfaceSectionContext(aContext.GetContextOfType(TInterfaceSectionContext));
  if IntfSec=nil then
    RaiseNotSupported(PosEl,aContext,20200606142555);
  IntfSec.AddImplHeaderStatement(JS);
end;

function TPasToJSConverter.AddDelayedInits(El: TPasModule;
  Src: TJSSourceElements; AContext: TConvertContext): boolean;
var
  aResolver: TPas2JSResolver;
  Hub: TPas2JSResolverHub;
  i: Integer;
  JS: TJSElement;
  AssignSt: TJSSimpleAssignStatement;
  FunDecl: TJSFunctionDeclarationStatement;
  ImplSrc: TJSSourceElements;
begin
  Result:=false;
  aResolver:=AContext.Resolver;
  if aResolver=nil then exit;
  if El=nil then ;
  Hub:=aResolver.Hub as TPas2JSResolverHub;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.AddDelayedInits Hub.JSDelaySpecializeCount=',Hub.JSDelaySpecializeCount);
  {$ENDIF}
  ImplSrc:=nil;
  for i:=0 to Hub.JSDelaySpecializeCount-1 do
    begin
    JS:=CreateDelaySpecializeInit(Hub.JSDelaySpecializes[i],AContext);
    if JS=nil then continue;
    if ImplSrc=nil then
      begin
      // create  "$mod.$implcode = function(){ }"
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AddToSourceElements(Src,AssignSt);
      AssignSt.LHS:=CreateMemberExpression([GetBIName(pbivnModule),GetBIName(pbivnImplCode)]);
      // create function(){}
      FunDecl:=CreateFunctionSt(El,true,true);
      AssignSt.Expr:=FunDecl;
      ImplSrc:=TJSSourceElements(FunDecl.AFunction.Body.A);
      end;
    AddToSourceElements(ImplSrc,JS);
    Result:=true;
    end;
end;

function TPasToJSConverter.CreateDelaySpecializeInit(El: TPasGenericType;
  AContext: TConvertContext): TJSElement;
var
  C: TClass;
  Path: String;
  Call: TJSCallExpression;
  DotExpr: TJSDotMemberExpression;
  AssignSt: TJSSimpleAssignStatement;
  Arr: TPasArrayType;
  ElTypeHi, ElTypeLo: TPasType;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  if not IsElementUsed(El) then exit;
  if not AContext.Resolver.IsFullySpecialized(El) then
    RaiseNotSupported(El,AContext,20201202145045,'not fully specialized, probably a bug in the analyzer');
  if not SpecializeNeedsDelay(El,AContext) then exit;
  C:=El.ClassType;
  if (C=TPasRecordType)
      or (C=TPasClassType) then
    begin
    if (C=TPasClassType) and TPasClassType(El).IsExternal then exit;
    // pas.unitname.recordtype.$initSpec();
    Path:=CreateReferencePath(El,AContext,rpkPathAndName)+'.'+GetBIName(pbifnClassInitSpecialize);
    Call:=CreateCallExpression(El);
    Call.Expr:=CreatePrimitiveDotExpr(Path,El);
    Result:=Call;
    end
  else if (C=TPasProcedureType) or (C=TPasFunctionType) then
    begin
    if not HasTypeInfo(El,AContext) then
      exit; // no RTTI needed
    // pas.unitname.$rtti.TProcF.init();
    DotExpr:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,El));
    DotExpr.MExpr:=CreateTypeInfoRef(El,AContext,El);
    DotExpr.Name:=TJSString(GetBIName(pbivnRTTIProc_InitSpec));
    Call:=CreateCallExpression(El);
    Call.Expr:=DotExpr;
    Result:=Call;
    end
  else if (C=TPasArrayType) then
    begin
    if not HasTypeInfo(El,AContext) then
      exit; // no RTTI needed
    // pas.unitname.$rtti.TArr.eltype=$mod.$rtti.TBird;
    aResolver:=AContext.Resolver;
    Arr:=TPasArrayType(El);
    ElTypeHi:=aResolver.ResolveAliasType(Arr.ElType,false);
    ElTypeLo:=aResolver.ResolveAliasType(ElTypeHi);
    if length(Arr.Ranges)>0 then
      begin
      // static array
      while ElTypeLo.ClassType=TPasArrayType do
        begin
        Arr:=TPasArrayType(ElTypeLo);
        if length(Arr.Ranges)=0 then
          RaiseNotSupported(Arr,AContext,20200902155418,'static array of anonymous array');
        ElTypeHi:=aResolver.ResolveAliasType(Arr.ElType,false);
        ElTypeLo:=aResolver.ResolveAliasType(ElTypeHi);
        end;
      end;
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
    AssignSt.LHS:=CreateDotNameExpr(El,CreateTypeInfoRef(El,AContext,El),
                                   TJSString(GetBIName(pbivnRTTIArray_ElType)));
    AssignSt.Expr:=CreateTypeInfoRef(ElTypeHi,AContext,El);
    Result:=AssignSt;
    end
  else
    RaiseNotSupported(El,AContext,20200831115251);
end;

function TPasToJSConverter.CreateReferencedSet(El: TPasElement; SetExpr: TJSElement
  ): TJSElement;
var
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(El);
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnSet_Reference)]);
  Call.AddArg(SetExpr);
  Result:=Call;
end;

function TPasToJSConverter.CreateRecordInit(aRecord: TPasRecordType;
  Expr: TPasExpr; El: TPasElement; AContext: TConvertContext): TJSElement;
// without Expr: recordtype.$new()
// with Expr: recordtype.$clone(expr)
var
  aResolver: TPas2JSResolver;
  ObjLit: TJSObjectLiteral;
  GUID: TGuid;
begin
  Result:=nil;
  if Expr<>nil then
    begin
    aResolver:=AContext.Resolver;
    if aResolver<>nil then
      begin
      if aResolver.GetAssignGUIDString(aRecord,Expr,GUID) then
        begin
        // TGuid.$clone({ D1:...})
        ObjLit:=CreateGUIDObjLit(aRecord,GUID,El,AContext);
        Result:=CreateRecordCallClone(El,aRecord,ObjLit,AContext);
        exit;
        end;
      end;
    if Expr is TRecordValues then
      // TRecord.$clone({...})
      Result:=ConvertRecordValues(TRecordValues(Expr),AContext);
    if Result=nil then
      RaiseNotSupported(Expr,AContext,20161024192747);
    end
  else
    begin
    // TRecord.$new()
    Result:=CreateRecordCallNew(El,aRecord,AContext);
    end;
end;

function TPasToJSConverter.CreateRecordCallNew(PosEl: TPasElement;
  RecTypeEl: TPasRecordType; AContext: TConvertContext): TJSCallExpression;
// create "RecordType.$new()"
var
  Expr: TJSElement;
  Call: TJSCallExpression;
begin
  Expr:=CreateReferencePathExpr(RecTypeEl,AContext);
  Call:=CreateCallExpression(PosEl);
  Call.Expr:=CreateDotNameExpr(PosEl,Expr,
                      TJSString(GetBIName(pbifnRecordNew)));
  Result:=Call;
end;

function TPasToJSConverter.CreateRecordCallClone(PosEl: TPasElement;
  RecTypeEl: TPasRecordType; RecordExpr: TJSElement; AContext: TConvertContext
  ): TJSCallExpression;
// create "RecordType.$clone(RecordExpr)
var
  Expr, CallExpr: TJSElement;
  DotExpr: TJSDotMemberExpression;
  Call: TJSCallExpression;
begin
  Expr:=CreateReferencePathExpr(RecTypeEl,AContext);
  if RecordExpr is TJSCallExpression then
    begin
    CallExpr:=TJSCallExpression(RecordExpr).Expr;
    if CallExpr is TJSDotMemberExpression then
      begin
      DotExpr:=TJSDotMemberExpression(CallExpr);
      if JSEquals(Expr,DotExpr.MExpr) then
        begin
        if (DotExpr.Name=TJSString(GetBIName(pbifnRecordNew)))
            or (DotExpr.Name=TJSString(GetBIName(pbifnRecordClone))) then
          begin
          // RecordExpr is already a RecordType.$new() or .$clone(...) -> skip clone
          Expr.Free;
          exit(TJSCallExpression(RecordExpr));
          end;
        end;
      // Note: rtl.getIntfGUIDR returns a cached version, which must be cloned
      end;
    end;

  Call:=CreateCallExpression(PosEl);
  Call.Expr:=CreateDotNameExpr(PosEl,Expr,
                                        TJSString(GetBIName(pbifnRecordClone)));
  Result:=Call;
  if RecordExpr<>nil then
    Call.AddArg(RecordExpr);
end;

function TPasToJSConverter.CreateRecordFunctionNew(El: TPasRecordType;
  AContext: TConvertContext; Fields: TFPList): TJSElement;
// this.$new = function(){
//   var r = Object.create(this);
//   r.aSet = {};
//   return r;
// }
const
  LocalVarName = 'r';
var
  AssignSt, CurAssignSt: TJSSimpleAssignStatement;
  FDS: TJSFunctionDeclarationStatement;
  FD: TJSFuncDef;
  RetSt: TJSReturnStatement;
  i: Integer;
  PasVar: TPasVariable;
  Call: TJSCallExpression;
  VarSt: TJSVariableStatement;
  Src: TJSSourceElements;
  VarName: String;
begin
  Result:=nil;
  if Fields.Count=0 then exit;

  // add "this.$new ="
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
  try
    AssignSt.LHS:=CreateMemberExpression(['this',GetBIName(pbifnRecordNew)]);
    // add "function(){"
    FDS:=CreateFunctionSt(El);
    AssignSt.Expr:=FDS;
    FD:=FDS.AFunction;
    Src:=TJSSourceElements(CreateElement(TJSSourceElements,El));
    FD.Body.A:=Src;

    // add "var r = Object.create(this);"
    Call:=CreateCallExpression(El);
    Call.Expr:=CreateMemberExpression(['Object','create']);
    Call.AddArg(CreatePrimitiveDotExpr('this',El));
    //Call.AddArg(CreatePrimitiveDotExpr('this.'+GetBIName(pbivnPtrRecord),El));
    VarSt:=CreateVarStatement(LocalVarName,Call,El);
    AddToSourceElements(Src,VarSt);

    // add "r.fieldname = initvalue;"
    for i:=0 to Fields.Count-1 do
      begin
      PasVar:=TPasVariable(Fields[i]);
      CurAssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      VarName:=TransformElToJSName(PasVar,AContext);
      CurAssignSt.LHS:=CreateMemberExpression([LocalVarName,VarName]);
      CurAssignSt.Expr:=CreateVarInit(PasVar,AContext);
      AddToSourceElements(Src,CurAssignSt);
      end;

    // add "return r;"
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
    AddToSourceElements(Src,RetSt);
    RetSt.Expr:=CreatePrimitiveDotExpr(LocalVarName,El);

    Result:=AssignSt;
  finally
    if Result=nil then
      AssignSt.Free;
  end;
end;

function TPasToJSConverter.CreateRecordFunctionEqual(El: TPasRecordType;
  AContext: TConvertContext; Fields: TFPList): TJSElement;
// this.$eq = function(b){
//   return (this.member1 == b.member1);
// };
const
  EqualParamName = 'b';
var
  LastAndExpr: TJSLogicalAndExpression;

  procedure Add_AndExpr_ToReturnSt(RetSt: TJSReturnStatement;
    PasVar: TPasVariable; Expr: TJSElement);
  var
    AndExpr: TJSLogicalAndExpression;
  begin
    if RetSt.Expr=nil then
      RetSt.Expr:=Expr
    else
      begin
      AndExpr:=TJSLogicalAndExpression(CreateElement(TJSLogicalAndExpression,PasVar));
      if LastAndExpr=nil then
        begin
        AndExpr.A:=RetSt.Expr;
        RetSt.Expr:=AndExpr;
        end
      else
        begin
        AndExpr.A:=LastAndExpr.B;
        LastAndExpr.B:=AndExpr;
        end;
      AndExpr.B:=Expr;
      LastAndExpr:=AndExpr;
      end;
  end;

var
  AssignSt: TJSSimpleAssignStatement;
  FD: TJSFuncDef;
  RetSt: TJSReturnStatement;
  i: Integer;
  PasVar: TPasVariable;
  FDS: TJSFunctionDeclarationStatement;
  EqExpr: TJSEqualityExpressionSEQ;
  VarType: TPasType;
  Call: TJSCallExpression;
  VarName: String;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  // add "this.$eq ="
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
  try
    AssignSt.LHS:=CreateMemberExpression(['this',GetBIName(pbifnRecordEqual)]);
    // add "function(b){"
    FDS:=CreateFunctionSt(El);
    AssignSt.Expr:=FDS;
    FD:=FDS.AFunction;
    FD.Params.Add(EqualParamName);
    // add "return "
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
    FD.Body.A:=RetSt;
    LastAndExpr:=nil;
    for i:=0 to Fields.Count-1 do
      begin
      PasVar:=TPasVariable(Fields[i]);
      // "this.member = b.member;"
      VarType:=PasVar.VarType;
      if aResolver<>nil then
        VarType:=aResolver.ResolveAliasType(VarType);
      VarName:=TransformElToJSName(PasVar,aContext);
      if VarType.ClassType=TPasRecordType then
        begin
        // record
        // add "this.member.$eq(b.member)"
        Call:=CreateCallExpression(PasVar);
        Add_AndExpr_ToReturnSt(RetSt,PasVar,Call);
        Call.Expr:=CreateMemberExpression(['this',VarName,GetBIName(pbifnRecordEqual)]);
        Call.AddArg(CreateMemberExpression([EqualParamName,VarName]));
        end
      else if VarType.ClassType=TPasSetType then
        begin
        // set
        // add "rtl.eqSet(this.member,b.member)"
        Call:=CreateCallExpression(PasVar);
        Add_AndExpr_ToReturnSt(RetSt,PasVar,Call);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnSet_Equal)]);
        Call.AddArg(CreateMemberExpression(['this',VarName]));
        Call.AddArg(CreateMemberExpression([EqualParamName,VarName]));
        end
      else if VarType is TPasProcedureType then
        begin
        // proc type
        // add "rtl.eqCallback(this.member,b.member)"
        Call:=CreateCallExpression(PasVar);
        Add_AndExpr_ToReturnSt(RetSt,PasVar,Call);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnProcType_Equal)]);
        Call.AddArg(CreateMemberExpression(['this',VarName]));
        Call.AddArg(CreateMemberExpression([EqualParamName,VarName]));
        end
      else if (VarType.ClassType=TPasArrayType)
          and (length(TPasArrayType(VarType).Ranges)>0) then
        begin
        // static array
        // add "rtl.arrayEq(this.member,b.member)"
        Call:=CreateCallExpression(PasVar);
        Add_AndExpr_ToReturnSt(RetSt,PasVar,Call);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Equal)]);
        Call.AddArg(CreateMemberExpression(['this',VarName]));
        Call.AddArg(CreateMemberExpression([EqualParamName,VarName]));
        end
      else
        begin
        // default: use strict equal "==="
        EqExpr:=TJSEqualityExpressionSEQ(CreateElement(TJSEqualityExpressionSEQ,PasVar));
        Add_AndExpr_ToReturnSt(RetSt,PasVar,EqExpr);
        EqExpr.A:=CreateMemberExpression(['this',VarName]);
        EqExpr.B:=CreateMemberExpression([EqualParamName,VarName]);
        end;
      end;
    if RetSt.Expr=nil then
      RetSt.Expr:=CreateLiteralBoolean(El,true); // no fields, "return true;"

    Result:=AssignSt;
  finally
    if Result=nil then
      AssignSt.Free;
  end;
end;

function TPasToJSConverter.CreateRecordFunctionAssign(El: TPasRecordType;
  AContext: TConvertContext; Fields: TFPList): TJSElement;
const
  SrcParamName = 's';
var
  AssignSt, VarAssignSt: TJSSimpleAssignStatement;
  FDS: TJSFunctionDeclarationStatement;
  FD: TJSFuncDef;
  Src: TJSSourceElements;
  i: Integer;
  PasVar: TPasVariable;
  VarName: String;
  aResolver: TPas2JSResolver;
  PasVarType: TPasType;
  RetSt: TJSReturnStatement;
  PasVarClass: TClass;
  Call: TJSCallExpression;
  SrcExpr: TJSElement;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;

  // add "this.$assign ="
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
  try
    AssignSt.LHS:=CreateMemberExpression(['this',GetBIName(pbifnRecordAssign)]);
    // add "function(s){"
    FDS:=CreateFunctionSt(El);
    AssignSt.Expr:=FDS;
    FD:=FDS.AFunction;
    FD.Params.Add(SrcParamName);
    Src:=TJSSourceElements(CreateElement(TJSSourceElements,El));
    FD.Body.A:=Src;

    PasVarType:=nil;
    PasVarClass:=nil;
    for i:=0 to Fields.Count-1 do
      begin
      PasVar:=TPasVariable(Fields[i]);
      VarName:=TransformElToJSName(PasVar,AContext);
      SrcExpr:=CreateMemberExpression([SrcParamName,VarName]);
      if aResolver<>nil then
        begin
        PasVarType:=aResolver.ResolveAliasType(PasVar.VarType);
        PasVarClass:=PasVarType.ClassType;
        if PasVarClass=TPasRecordType then
          begin
          // assign sub record "this.A.$assign(s.A);"
          Call:=CreateCallExpression(PasVar);
          AddToSourceElements(Src,Call);
          Call.Expr:=CreateMemberExpression(['this',VarName,GetBIName(pbifnRecordAssign)]);
          Call.AddArg(SrcExpr);
          continue;
          end;
        end;
      // create "this.A = s.A;"
      VarAssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PasVar));
      AddToSourceElements(Src,VarAssignSt);
      VarAssignSt.LHS:=CreateSubDeclNameExpr(PasVar,aContext);
      VarAssignSt.Expr:=SrcExpr;
      if PasVarClass=TPasArrayType then
        begin
        if length(TPasArrayType(PasVarType).Ranges)>0 then
          begin
          // clone sub static array
          VarAssignSt.Expr:=CreateCloneStaticArray(PasVar,TPasArrayType(PasVarType),
                                              SrcExpr,aContext);
          end
        else
          // reference dynamic array
          VarAssignSt.Expr:=CreateArrayRef(PasVar,SrcExpr);
        end
      else if PasVarClass=TPasSetType then
        begin
        // clone sub set
        VarAssignSt.Expr:=CreateReferencedSet(PasVar,SrcExpr);
        end;
      end;

    // add "return this;"
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
    AddToSourceElements(Src,RetSt);
    RetSt.Expr:=CreatePrimitiveDotExpr('this',El);

    Result:=AssignSt;
  finally
    if Result=nil then
      AssignSt.Free;
  end;
end;

procedure TPasToJSConverter.CreateRecordRTTI(El: TPasRecordType;
  Src: TJSSourceElements; FuncContext: TFunctionContext;
  MembersSrc: TJSSourceElements; MembersFuncContext: TFunctionContext);
var
  ObjLit: TJSObjectLiteral;
  Call: TJSCallExpression;
  HasRTTIMembers: Boolean;
begin
  Call:=nil;
  try
    // module.$rtti.$Record("typename",{});
    Call:=CreateRTTINewType(El,GetBIName(pbifnRTTINewRecord),false,FuncContext,ObjLit);
    if ObjLit=nil then
      RaiseInconsistency(20190105141430,El);

    HasRTTIMembers:=CreateRTTIMembers(El,Src,FuncContext,MembersSrc,MembersFuncContext,Call,false);
    if not HasRTTIMembers then
      begin
      // no published members, add "module.$rtti.$Record..."
      if Src=MembersSrc then
        AddToSourceElements(Src,Call)
      else
        Src.Statements.InsertNode(0).Node:=Call;
      end;

    Call:=nil;
  finally
      Call.Free;
  end;
end;

function TPasToJSConverter.CreateDelayedInitMembersFunction(PosEl: TPasElement;
  Src: TJSSourceElements; FuncContext: TFunctionContext; out
  DelaySrc: TJSSourceElements): TFunctionContext;
var
  AssignSt: TJSSimpleAssignStatement;
  FunDecl: TJSFunctionDeclarationStatement;
begin
  // this.$initSpec = function(){ DelaySrc }
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
  AddToSourceElements(Src,AssignSt);
  AssignSt.LHS:=CreatePrimitiveDotExpr('this.'+GetBIName(pbifnClassInitSpecialize),PosEl);
  FunDecl:=CreateFunctionSt(PosEl,true,true);
  AssignSt.Expr:=FunDecl;
  DelaySrc:=TJSSourceElements(FunDecl.AFunction.Body.A);
  Result:=TFunctionContext.Create(PosEl,DelaySrc,FuncContext);
  Result.IsGlobal:=true;
  Result.ThisVar.Element:=PosEl;
  Result.ThisVar.Kind:=cvkCurType;
end;

function TPasToJSConverter.CreateArrayConcat(
  ElTypeResolved: TPasResolverResult; PosEl: TPasElement;
  AContext: TConvertContext): TJSCallExpression;
var
  Call: TJSCallExpression;
begin
  Result:=nil;
  Call:=CreateCallExpression(PosEl);
  try
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.CreateArrayConcat ElType=',GetResolverResultDbg(ElTypeResolved));
    {$ENDIF}
    if ElTypeResolved.BaseType=btContext then
      begin
      if ElTypeResolved.LoTypeEl.ClassType=TPasRecordType then
        begin
        // record: rtl.arrayConcat(RecordType,array1,array2,...)
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Concat)]);
        Call.AddArg(CreateReferencePathExpr(ElTypeResolved.LoTypeEl,AContext));
        end;
      end
    else if ElTypeResolved.BaseType=btSet then
      begin
      // set: rtl.arrayConcat("refSet",array1,array2,...)
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Concat)]);
      Call.AddArg(CreateLiteralString(PosEl,GetBIName(pbifnSet_Reference)));
      end;
    if Call.Expr=nil then
      begin
      // default: rtl.arrayConcatN(array1,array2,...)
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_ConcatN)]);
      end;
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.CreateArrayConcat(ArrayType: TPasArrayType;
  PosEl: TPasElement; AContext: TConvertContext): TJSCallExpression;
var
  ElTypeResolved: TPasResolverResult;
  aResolver: TPas2JSResolver;
begin
  if length(ArrayType.Ranges)>1 then
    RaiseNotSupported(PosEl,AContext,20170331001021);
  aResolver:=AContext.Resolver;
  aResolver.ComputeElement(aResolver.GetArrayElType(ArrayType),ElTypeResolved,[rcType]);
  Result:=CreateArrayConcat(ElTypeResolved,PosEl,AContext);
end;

function TPasToJSConverter.CreateArrayInit(ArrayType: TPasArrayType;
  Expr: TPasExpr; El: TPasElement; AContext: TConvertContext): TJSElement;

  function IsAdd(AnExpr: TPasExpr): Boolean;
  begin
    Result:=(AnExpr.ClassType=TBinaryExpr) and (AnExpr.OpCode=eopAdd);
  end;

  function ConvertArrayExpr(CurArrType: TPasArrayType; RgIndex: integer;
    CurExpr: TPasExpr): TJSElement;
  var
    NextArrType: TPasArrayType;
    NextRgIndex: integer;
    IsLastRange: boolean;

    function ConvertSubExpr(SubExpr: TPasExpr): TJSElement;
    begin
      if IsLastRange then
        Result:=ConvertExpression(SubExpr,AContext)
      else
        Result:=ConvertArrayExpr(NextArrType,NextRgIndex,SubExpr);
    end;

    function ConvertSubValues(ExprArray: TPasExprArray): TJSArrayLiteral;
    var
      i: Integer;
      JS: TJSElement;
      Param: TPasExpr;
    begin
      Result:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
      for i:=0 to length(ExprArray)-1 do
        begin
        Param:=ExprArray[i];
        JS:=ConvertSubExpr(Param);
        JS:=CreateArrayEl(Param,JS,AContext);
        Result.Elements.AddElement.Expr:=JS;
        end;
    end;

    procedure TraverseAdd(Bin: TBinaryExpr; ConcatCall: TJSCallExpression);
    // A+B -> A,B
    // (A+B)+C -> A,B,C
    begin
      if IsAdd(Bin.left) then
        TraverseAdd(TBinaryExpr(Bin.left),ConcatCall)
      else
        ConcatCall.AddArg(ConvertArrayExpr(NextArrType,NextRgIndex,Bin.left));
      if IsAdd(Bin.right) then
        TraverseAdd(TBinaryExpr(Bin.right),ConcatCall)
      else
        ConcatCall.AddArg(ConvertArrayExpr(NextArrType,NextRgIndex,Bin.right));
    end;

  var
    ElTypeResolved: TPasResolverResult;
    Call: TJSCallExpression;
    aResolver: TPas2JSResolver;
  begin
    Result:=nil;
    IsLastRange:=false;
    NextArrType:=CurArrType;
    NextRgIndex:=RgIndex+1;
    if RgIndex>=length(CurArrType.Ranges)-1 then
      begin
      aResolver:=AContext.Resolver;
      aResolver.ComputeElement(aResolver.GetArrayElType(CurArrType),ElTypeResolved,[rcType]);
      if (ElTypeResolved.BaseType=btContext)
          and (ElTypeResolved.LoTypeEl.ClassType=TPasArrayType) then
        begin
        NextArrType:=TPasArrayType(ElTypeResolved.LoTypeEl);
        NextRgIndex:=0;
        end
      else
        IsLastRange:=true;
      end;
    if CurExpr.ClassType=TArrayValues then
      begin
      // (...,...)
      Result:=ConvertSubValues(TArrayValues(CurExpr).Values);
      exit;
      end
    else if (CurExpr.ClassType=TParamsExpr) and (TParamsExpr(CurExpr).Kind=pekSet) then
      begin
      // [...,...]
      Result:=ConvertSubValues(TParamsExpr(CurExpr).Params);
      exit;
      end
    else if IsAdd(CurExpr) then
      begin
      // A+B  ->  rtl.arrayConcat(null,A,B)
      Call:=CreateArrayConcat(ArrayType,CurExpr,AContext);
      try
        TraverseAdd(TBinaryExpr(CurExpr),Call);
        Result:=Call;
      finally
        if Result=nil then
          Call.Free;
      end;
      exit;
      end;
    // use default, e.g. a.b  or  c[...]  or  copy(...)
    Result:=ConvertExpression(CurExpr,AContext);
  end;

  function ConvertExprToVarRec(CurExpr: TPasExpr): TJSElement;
  // convert [true,Int] to  system.varrecs(1,true,0,Int)
  var
    aResolver: TPas2JSResolver;
    Param: TPasExpr;
    ParamResolved: TPasResolverResult;

    procedure RaiseWrongTypeInArrayConstructor(id: TMaxPrecInt);
    begin
      aResolver.RaiseMsg(id,nWrongTypeXInArrayConstructor,sWrongTypeXInArrayConstructor,
        [aResolver.GetResolverResultDescription(ParamResolved)],Param);
    end;

  var
    Params: TParamsExpr;
    ModScope: TPas2JSModuleScope;
    Call: TJSCallExpression;
    i, VType: Integer;
    LoTypeEl: TPasType;
    ParamsArr: TPasExprArray;
  begin
    Result:=nil;
    aResolver:=AContext.Resolver;
    if IsAdd(CurExpr) then
      aResolver.RaiseMsg(20190215222435,nXExpectedButYFound,sXExpectedButYFound,
        ['array of const',GetElementTypeName(CurExpr)],CurExpr);
    if (not (CurExpr is TParamsExpr)) or (TParamsExpr(CurExpr).Kind<>pekSet) then
      begin
      // e.g. Format(args)
      Result:=ConvertExpression(CurExpr,AContext);
      exit;
      end;
    Params:=TParamsExpr(CurExpr);
    ParamsArr:=Params.Params;
    if length(ParamsArr)=0 then
      begin
      // e.g. Format([])
      Result:=CreateElement(TJSArrayLiteral,Params);
      exit;
      end;

    ModScope:=NoNil(aResolver.RootElement.CustomData) as TPas2JSModuleScope;
    if ModScope.SystemVarRecs=nil then
      RaiseNotSupported(Params,AContext,20190215215148);
    Call:=CreateCallExpression(Params);
    try
      Call.Expr:=CreateReferencePathExpr(ModScope.SystemVarRecs,AContext);
      for i:=0 to length(ParamsArr)-1 do
        begin
        Param:=ParamsArr[i];
        aResolver.ComputeElement(Param,ParamResolved,[]);
        if not (rrfReadable in ParamResolved.Flags) then
          begin
          if (ParamResolved.BaseType=btContext)
              and (ParamResolved.IdentEl is TPasClassType)
              and (TPasClassType(ParamResolved.IdentEl).ObjKind=okClass) then
            VType:=pas2js_vtClass
          else
            RaiseWrongTypeInArrayConstructor(20190215221549);
          end
        else if ParamResolved.BaseType in [btByte,btShortInt,btWord,btSmallInt,btLongint] then
          VType:=pas2js_vtInteger
        else if ParamResolved.BaseType in [btLongWord,btUIntDouble,btIntDouble] then
          VType:=pas2js_vtNativeInt
        else if ParamResolved.BaseType in btAllJSBooleans then
          VType:=pas2js_vtBoolean
        else if ParamResolved.BaseType in btAllJSFloats then
          VType:=pas2js_vtExtended
        else if ParamResolved.BaseType in btAllJSChars then
          VType:=pas2js_vtWideChar
        else if ParamResolved.BaseType in btAllJSStrings then
          VType:=pas2js_vtUnicodeString
        else if ParamResolved.BaseType in [btNil,btPointer] then
          VType:=pas2js_vtPointer
        else if ParamResolved.BaseType=btCurrency then
          VType:=pas2js_vtCurrency
        else if ParamResolved.BaseType=btContext then
          begin
          LoTypeEl:=ParamResolved.LoTypeEl;
          if LoTypeEl.ClassType=TPasClassType then
            case TPasClassType(LoTypeEl).ObjKind of
            okClass: VType:=pas2js_vtObject;
            okInterface: VType:=pas2js_vtInterface;
            else
              RaiseWrongTypeInArrayConstructor(20190215221106);
            end
          else if LoTypeEl.ClassType=TPasClassOfType then
            VType:=pas2js_vtClass
          else
            RaiseWrongTypeInArrayConstructor(20190215221122);
          end
        else if (ParamResolved.BaseType=btCustom)
            and aResolver.IsJSBaseType(ParamResolved,pbtJSValue) then
          VType:=pas2js_vtJSValue
        else
          RaiseWrongTypeInArrayConstructor(20190215221457);
        Call.AddArg(CreateLiteralNumber(Param,VType));
        Call.AddArg(ConvertExpression(Param,AContext));
        end;
      Result:=Call;
    finally
      if Result=nil then
        Call.Free;
    end;
  end;

var
  Call: TJSCallExpression;
  ArrLit: TJSArrayLiteral;
  i, DimSize: Integer;
  RangeResolved, ElTypeResolved, ExprResolved: TPasResolverResult;
  Range: TPasExpr;
  Lit: TJSLiteral;
  CurArrayType: TPasArrayType;
  DefaultValue: TJSElement;
  US: TJSString;
  DimLits: TObjectList;
  aResolver: TPas2JSResolver;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.CreateArrayInit ',GetObjName(ArrayType),' ',ArrayType.ParentPath,' Expr=',GetObjName(Expr));
  {$ENDIF}
  aResolver:=AContext.Resolver;
  if Assigned(Expr) then
    begin
    // init array with expression
    if aResolver=nil then
      DoError(20161024192739,nInitializedArraysNotSupported,sInitializedArraysNotSupported,[],ArrayType);
    aResolver.ComputeElement(Expr,ExprResolved,[]);
    if (ExprResolved.BaseType in [btArrayOrSet,btArrayLit])
        or ((ExprResolved.BaseType=btContext)
          and (ExprResolved.LoTypeEl.ClassType=TPasArrayType)) then
      begin
      if ArrayType.ElType=nil then
        Result:=ConvertExprToVarRec(Expr)
      else
        Result:=ConvertArrayExpr(ArrayType,0,Expr);
      end
    else if ExprResolved.BaseType in btAllStringAndChars then
      begin
      US:=StrToJSString(aResolver.ComputeConstString(Expr,false,true));
      ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,Expr));
      Result:=ArrLit;
      for i:=1 to length(US) do
        ArrLit.Elements.AddElement.Expr:=CreateLiteralJSString(Expr,US[i]);
      end
    else if ExprResolved.BaseType=btNil then
      begin
      Result:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,Expr));
      end
    else
      RaiseNotSupported(Expr,AContext,20170223133034);
    end
  else if length(ArrayType.Ranges)=0 then
    begin
    // empty dynamic array: []
    Result:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
    end
  else
    begin
    // static array
    // create "rtl.arraySetLength(null,defaultvalue,dim1,dim2,...)"
    if aResolver=nil then
      RaiseNotSupported(El,AContext,20170223113050,'');
    Result:=nil;
    DimLits:=TObjectList.Create(true);
    try
      Call:=CreateCallExpression(El);
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_SetLength)]);
      // add parameter null
      Call.AddArg(CreateLiteralNull(El));

      // create parameters dim1,dim2,...
      CurArrayType:=ArrayType;
      while true do
        begin
        for i:=0 to length(CurArrayType.Ranges)-1 do
          begin
          Range:=CurArrayType.Ranges[i];
          // compute size of this dimension
          DimSize:=aResolver.GetRangeLength(Range);
          if DimSize=0 then
            begin
            aResolver.ComputeElement(Range,RangeResolved,[rcConstant]);
            RaiseNotSupported(Range,AContext,20170223113318,GetResolverResultDbg(RangeResolved));
            end;
          Lit:=CreateLiteralNumber(El,DimSize);
          DimLits.Add(Lit);
          end;
        aResolver.ComputeElement(aResolver.GetArrayElType(CurArrayType),ElTypeResolved,[rcType]);
        if (ElTypeResolved.LoTypeEl is TPasArrayType) then
          begin
          CurArrayType:=TPasArrayType(ElTypeResolved.LoTypeEl);
          if length(CurArrayType.Ranges)>0 then
            begin
            // nested static array
            continue;
            end;
          end;
        break;
        end;

      // add parameter defaultvalue
      if ElTypeResolved.LoTypeEl is TPasRecordType then
        begin
        // array of record -> push the type reference
        DefaultValue:=CreateReferencePathExpr(ElTypeResolved.LoTypeEl,AContext);
        end
      else
        DefaultValue:=CreateValInit(ElTypeResolved.LoTypeEl,nil,El,AContext);
      Call.AddArg(DefaultValue);

      // add parameters dim1,dim2,...
      for i:=0 to DimLits.Count-1 do
        Call.AddArg(TJSElement(DimLits[i]));
      DimLits.OwnsObjects:=false;
      DimLits.Clear;

      Result:=Call;
    finally
      DimLits.Free;
      if Result=nil then
        Call.Free;
    end;
    end;
  if Result=nil then
    RaiseInconsistency(20180617233317,Expr);
end;

function TPasToJSConverter.CreateArrayRef(El: TPasElement; ArrayExpr: TJSElement
  ): TJSElement;
var
  Call: TJSCallExpression;
begin
  if ArrayExpr is TJSArrayLiteral then
    exit(ArrayExpr);
  Call:=CreateCallExpression(El);
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Reference)]);
  Call.AddArg(ArrayExpr);
  Result:=Call;
end;

function TPasToJSConverter.CreateCmpArrayWithNil(El: TPasElement;
  JSArray: TJSElement; OpCode: TExprOpCode): TJSElement;
// convert "array = nil" to "rtl.length(array) > 0"
// convert "array <> nil" to "rtl.length(array) === 0"
var
  Call: TJSCallExpression;
  BinExpr: TJSBinaryExpression;
begin
  if not (OpCode in [eopEqual,eopNotEqual]) then
    RaiseInconsistency(20170401184819,El);
  Call:=CreateCallExpression(El);
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnArray_Length)]);
  Call.AddArg(JSArray);
  if OpCode=eopEqual then
    BinExpr:=TJSEqualityExpressionSEQ(CreateElement(TJSEqualityExpressionSEQ,El))
  else
    BinExpr:=TJSRelationalExpressionGT(CreateElement(TJSRelationalExpressionGT,El));
  BinExpr.A:=Call;
  BinExpr.B:=CreateLiteralNumber(El,0);
  Result:=BinExpr;
end;

function TPasToJSConverter.CreateCloneStaticArray(El: TPasElement;
  ArrTypeEl: TPasArrayType; ArrayExpr: TJSElement; AContext: TConvertContext
  ): TJSElement;
var
  Call: TJSCallExpression;
  Path: String;
  FuncContext: TFunctionContext;
  DotExpr: TJSDotMemberExpression;
  i: TMaxPrecInt;
  JSExpr: TJSElement;
begin
  if ArrayExpr is TJSArrayLiteral then
    exit(ArrayExpr);

  if AContext.Resolver.HasStaticArrayCloneFunc(ArrTypeEl) then
    begin
    // TArrayType$clone(ArrayExpr);
    if ArrTypeEl.Name='' then
      RaiseNotSupported(El,AContext,20180218230407,'copy anonymous multi dim static array');
    if length(ArrTypeEl.Ranges)>1 then
      RaiseNotSupported(El,AContext,20180218231700,'copy multi dim static array');
    FuncContext:=AContext.GetFunctionContext;
    Path:=CreateReferencePath(ArrTypeEl,FuncContext,rpkPathAndName)
          +GetBIName(pbifnArray_Static_Clone);
    Call:=CreateCallExpression(El);
    Call.Expr:=CreatePrimitiveDotExpr(Path,El);
    Call.AddArg(ArrayExpr);
    Result:=Call;
    end
  else
    begin
    // ArrayExpr.slice(0)
    if ArrayExpr is TJSCallExpression then
      begin
      Call:=TJSCallExpression(ArrayExpr);
      if Call.Expr is TJSDotMemberExpression then
        begin
        DotExpr:=TJSDotMemberExpression(Call.Expr);
        if (DotExpr.Name='slice') and (Call.Args<>nil)
            and (Call.Args.Elements.Count=1) then
          begin
          JSExpr:=Call.Args.Elements[0].Expr;
          if IsLiteralInteger(JSExpr,i) and (i=0) then
            exit(Call); // is already ".slice(0)"
          end;
        end;
      end;

    Call:=CreateCallExpression(El);
    Call.Expr:=CreateDotNameExpr(El,ArrayExpr,'slice');
    Call.AddArg(CreateLiteralNumber(El,0));
    Result:=Call;
    end;
end;

procedure TPasToJSConverter.AddClassConDestructorFunction(El: TPasClassType;
  Src: TJSSourceElements; ClassContext: TConvertContext; IsTObject: boolean;
  Ancestor: TPasType; Kind: TMemberFunc);
const
  MemberFuncName: array[TMemberFunc] of string = (
    '$init',
    '$final'
    );
var
  AncestorIsExternal: boolean;

  function IsMemberNeeded(aMember: TPasElement): boolean;
  begin
    if IsElementUsed(aMember) then exit(true);
    if IsTObject then
      begin
      if aMember.ClassType=TPasProcedure then
        begin
        if (CompareText(aMember.Name,'AfterConstruction')=0)
            or (CompareText(aMember.Name,'BeforeDestruction')=0) then
          exit(true);
        end;
      end;
    Result:=false;
  end;

  procedure AddCallAncestorMemberFunction(ClassContext: TConvertContext;
    Ancestor: TPasType; Src: TJSSourceElements; Kind: TMemberFunc);
  var
    Call: TJSCallExpression;
    AncestorPath: String;
  begin
    if (Ancestor=nil) or AncestorIsExternal then
      exit;
    Call:=CreateCallExpression(El);
    AncestorPath:=CreateReferencePath(Ancestor,ClassContext,rpkPathAndName);
    Call.Expr:=CreatePrimitiveDotExpr(AncestorPath+'.'+MemberFuncName[Kind]+'.call',El);
    Call.AddArg(CreatePrimitiveDotExpr('this',El));
    AddToSourceElements(Src,Call);
  end;

// add instance initialization function:
//   this.$init = function(){
//     ancestor.$init();
//     ... init variables ...
//   }
// or add instance finalization function:
//   this.$final = function(){
//     ... clear references ...
//     ancestor.$final();
//   }
var
  FuncVD: TJSVarDeclaration;
  New_Src: TJSSourceElements;
  New_FuncContext: TFunctionContext;
  I: Integer;
  P: TPasElement;
  NewEl: TJSElement;
  Func: TJSFunctionDeclarationStatement;
  VarType: TPasType;
  AssignSt: TJSSimpleAssignStatement;
begin
  // add instance members
  AncestorIsExternal:=(Ancestor is TPasClassType) and TPasClassType(Ancestor).IsExternal;
  New_Src:=TJSSourceElements(CreateElement(TJSSourceElements, El));
  New_FuncContext:=TFunctionContext.Create(El,New_Src,ClassContext);
  try
    New_FuncContext.ThisVar.Element:=El;
    New_FuncContext.ThisVar.Kind:=cvkCurType;
    New_FuncContext.IsGlobal:=false;

    // add class members
    For I:=0 to El.Members.Count-1 do
      begin
      P:=TPasElement(El.Members[i]);
      if not IsMemberNeeded(P) then continue;
      NewEl:=nil;
      if (P.ClassType=TPasVariable)
          and (ClassVarModifiersType*TPasVariable(P).VarModifiers=[]) then
        begin
        if Kind=mfInit then
        begin
          // mfInit: init var
          NewEl:=CreateVarDecl(TPasVariable(P),New_FuncContext); // can be nil
        end
        else
          begin
          // mfFinalize: clear reference
          if vmExternal in TPasVariable(P).VarModifiers then continue;
          VarType:=ClassContext.Resolver.ResolveAliasType(TPasVariable(P).VarType);
          if (VarType.ClassType=TPasRecordType)
              or (VarType.ClassType=TPasClassType)
              or (VarType.ClassType=TPasClassOfType)
              or (VarType.ClassType=TPasSetType)
              or (VarType.ClassType=TPasProcedureType)
              or (VarType.ClassType=TPasFunctionType)
              or (VarType.ClassType=TPasArrayType) then
            begin
            // add 'this.FieldName = undefined;'
            AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
            NewEl:=AssignSt;
            AssignSt.LHS:=CreateSubDeclNameExpr(P,New_FuncContext);
            AssignSt.Expr:=CreateLiteralUndefined(El);
            end;
          end;
        end;
      if NewEl=nil then continue;
      if (Kind=mfInit) and (New_Src.Statements.Count=0) then
        // add call ancestor.$init.call(this)
        AddCallAncestorMemberFunction(ClassContext,Ancestor,New_Src,Kind);
      AddToSourceElements(New_Src,NewEl);
      end;
    if (Kind=mfFinalize) and (New_Src.Statements.Count>0) then
      // call ancestor.$final.call(this)
      AddCallAncestorMemberFunction(ClassContext,Ancestor,New_Src,Kind);
    if (Ancestor<>nil) and (not AncestorIsExternal)
        and (New_Src.Statements.Count=0) then
      exit; // descendent does not need $init/$final

    FuncVD:=TJSVarDeclaration(CreateElement(TJSVarDeclaration,El));
    AddToSourceElements(Src,FuncVD);
    FuncVD.Name:='this.'+MemberFuncName[Kind];
    Func:=CreateFunctionSt(El);
    FuncVD.Init:=Func;
    Func.AFunction.Body.A:=New_Src;
    New_Src:=nil;
  finally
    New_Src.Free;
    New_FuncContext.Free;
  end;
end;

procedure TPasToJSConverter.AddClassRTTI(El: TPasClassType;
  Src: TJSSourceElements; FuncContext: TFunctionContext);
var
  HasRTTIMembers, NeedLocalVar: Boolean;
  RTTIExpr, AttrJS: TJSElement;
  Attr: TPasExprArray;
  AssignSt: TJSAssignStatement;
  ClassScope: TPas2JSClassScope;
  Creator: String;
  ObjLit: TJSObjectLiteral;
  Call: TJSCallExpression;
begin
  AttrJS:=nil;
  RTTIExpr:=nil;
  try
    ClassScope:=El.CustomData as TPas2JSClassScope;
    if (ClassScope.SpecializedFromItem<>nil)
        and not (coNoTypeInfo in Options)
        and FuncContext.Resolver.HasTypeInfo(El) then
      begin
      // specialized class -> init RTTI
      // add header:  module.$rtti.$Class("classname");
      Creator:=GetClassBIName(El,FuncContext);
      Call:=CreateRTTINewType(El,Creator,true,FuncContext,ObjLit);
      if ObjLit<>nil then
        RaiseInconsistency(20200606134834,El);
      AddHeaderStatement(Call,El,FuncContext);
      end;

    // this.$rtti
    RTTIExpr:=CreateMemberExpression(['this',GetBIName(pbivnRTTI)]);
    Attr:=FuncContext.Resolver.GetAttributeCallsEl(El);
    AttrJS:=CreateRTTIAttributes(Attr,El,FuncContext);
    NeedLocalVar:=(AttrJS<>nil);

    HasRTTIMembers:=CreateRTTIMembers(El,Src,FuncContext,Src,FuncContext,RTTIExpr,NeedLocalVar);
    if HasRTTIMembers then
      RTTIExpr:=nil;

    if AttrJS<>nil then
      begin
      // $r.attr = [];
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AddToSourceElements(Src,AssignSt);
      AssignSt.LHS:=CreateMemberExpression([GetBIName(pbivnRTTILocal),GetBIName(pbivnRTTITypeAttributes)]);
      AssignSt.Expr:=AttrJS;
      AttrJS:=nil;
      end;
  finally
    AttrJS.Free;
    RTTIExpr.Free;
  end;
end;

procedure TPasToJSConverter.AddClassConstructors(FuncContext: TFunctionContext;
  PosEl: TPasElement);
var
  i: Integer;
  Proc: TPasProcedure;
  First, Last: TJSStatementList;
  St: TJSElement;
  Call: TJSCallExpression;
  Bracket: TJSUnaryBracketsExpression;
  RootContext: TRootContext;
begin
  RootContext:=TRootContext(FuncContext.GetRootContext);
  First:=nil;
  Last:=nil;
  try
    for i:=0 to length(RootContext.GlobalClassMethods)-1 do
      begin
      Proc:=RootContext.GlobalClassMethods[i];
      St:=ConvertProcedure(Proc,FuncContext);
      // create direct call  ( function(){} )();
      Bracket:=TJSUnaryBracketsExpression(CreateElement(TJSUnaryBracketsExpression,PosEl));
      Bracket.A:=St;
      Call:=CreateCallExpression(PosEl);
      Call.Expr:=Bracket;
      AddToStatementList(First,Last,Call,PosEl);
      end;
    PrependToStatementList(FuncContext.BodySt,First,PosEl);
    First:=nil;
  finally
    First.Free;
  end;
end;

procedure TPasToJSConverter.AddClassMessageIds(El: TPasClassType;
  Src: TJSSourceElements; FuncContext: TFunctionContext;
  pbivn: TPas2JSBuiltInName);
// $msgint = { id1:"proc1name", id2: "proc2name" ... }
var
  Scope: TPas2JSClassScope;
  List: TMessageIdToProc_List;
  i: Integer;
  AssignSt: TJSSimpleAssignStatement;
  ObjLit: TJSObjectLiteral;
  LitEl: TJSObjectLiteralElement;
  Proc: TPasProcedure;
begin
  Scope:=TPas2JSClassScope(El.CustomData);
  case pbivn of
  pbivnMessageInt: List:=Scope.MsgIntToProc;
  pbivnMessageStr: List:=Scope.MsgStrToProc;
  else
    RaiseNotSupported(El,FuncContext,20190304001209,GetBIName(pbivn));
  end;
  if (List=nil) or (List.Count=0) then exit;

  // this.$msgint = {}
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
  AddToSourceElements(Src,AssignSt);
  AssignSt.LHS:=CreateMemberExpression(['this',GetBIName(pbivn)]);
  ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
  AssignSt.Expr:=ObjLit;

  for i:=0 to List.Count-1 do
    begin
    LitEl:=ObjLit.Elements.AddElement;
    LitEl.Name:=TJSString(List[i]);
    Proc:=TPasProcedure(List.Objects[i]);
    LitEl.Expr:=CreateLiteralJSString(Proc,TJSString(TransformElToJSName(Proc,FuncContext)));
    end;
end;

function TPasToJSConverter.CreateCallback(Expr: TPasExpr;
  ResolvedEl: TPasResolverResult; aSafeCall: boolean; AContext: TConvertContext
  ): TJSElement;
// El is a reference to a proc
// if aSafeCall then create  "rtl.createSafeCallback(Target,func)"
// for a proc or nested proc simply use the function
// for a method create  "rtl.createCallback(Target,func)"

  function NeedAppendClass(El: TPasElement): boolean;
  var
    TargetResolved: TPasResolverResult;
  begin
    AContext.Resolver.ComputeElement(El,TargetResolved,[]);
    if (TargetResolved.IdentEl is TPasClassType)
        or (TargetResolved.LoTypeEl is TPasClassOfType) then
      // left side is a class
      Result:=false
    else
      Result:=true;
  end;

var
  Call: TJSCallExpression;
  TargetJS: TJSElement;
  FunName, TargetName: String;
  Proc: TPasProcedure;
  IsHelper, NeedClass: Boolean;
  Bin: TBinaryExpr;
  aResolver: TPas2JSResolver;
  OldAccess: TCtxAccess;
  PosEl: TPasExpr;
  Ref: TResolvedReference;
  WithExprScope: TPas2JSWithExprScope;
  SelfScope: TPasProcedureScope;
begin
  Result:=nil;
  if not (ResolvedEl.IdentEl is TPasProcedure) then
    RaiseInconsistency(20170215140756,Expr);
  aResolver:=AContext.Resolver;

  Proc:=TPasProcedure(ResolvedEl.IdentEl);
  if not aResolver.ProcHasSelf(Proc) then
    begin
    // not an "of object" method -> simply use the function
    Result:=CreateReferencePathExpr(Proc,AContext);
    if aSafeCall then
      Result:=CreateSafeCallback(Expr,Result,AContext);
    exit;
    end;
  IsHelper:=aResolver.IsHelperMethod(Proc);
  NeedClass:=aResolver.IsClassMethod(Proc) and not aResolver.MethodIsStatic(Proc);

  if Expr is TInlineSpecializeExpr then
    Expr:=TInlineSpecializeExpr(Expr).NameExpr;

  // an of-object method -> create "rtl.createCallback(Target,func)"
  TargetJS:=nil;
  Call:=nil;
  try
    if Expr is TBinaryExpr then
      begin
      // e.g. "target.func"
      Bin:=TBinaryExpr(Expr);
      if Bin.OpCode<>eopSubIdent then
        RaiseNotSupported(Expr,AContext,20190205230811);
      OldAccess:=AContext.Access;
      AContext.Access:=caRead;
      TargetJS:=ConvertExpression(Bin.left,AContext);
      AContext.Access:=OldAccess;
      if NeedClass then
        NeedClass:=NeedAppendClass(Bin.left);
      PosEl:=Bin.right;
      end
    else if aResolver.IsNameExpr(Expr) then
      begin
      // e.g. "func"
      PosEl:=Expr;
      if not (Expr.CustomData is TResolvedReference) then
        RaiseNotSupported(Expr,AContext,20190205230915);
      Ref:=TResolvedReference(Expr.CustomData);
      WithExprScope:=Ref.WithExprScope as TPas2JSWithExprScope;
      if WithExprScope<>nil then
        begin
        // e.g. "with target do f:=@func"
        TargetName:=WithExprScope.WithVarName;
        if (TargetName='') and IsHelper then
          RaiseNotSupported(PosEl,AContext,20190209092355);
        if NeedClass then
          NeedClass:=NeedAppendClass(WithExprScope.Expr);
        end
      else
        begin
        // inside method e.g. "func"  or "fly(@func)"
        SelfScope:=aResolver.GetSelfScope(Expr);
        if SelfScope=nil then
          RaiseNotSupported(PosEl,AContext,20190205230919);
        if SelfScope.SelfArg<>nil then
          TargetName:=GetLocalName(SelfScope.SelfArg,cvkAll,AContext)
        else if SelfScope.ClassRecScope<>nil then
          begin
          TargetName:=CreateReferencePath(SelfScope.ClassRecScope.Element,
                                                       AContext,rpkPathAndName);
          NeedClass:=false;
          end
        else
          RaiseNotSupported(PosEl,AContext,20190206104558,GetObjName(Proc));
        if TargetName='' then
          TargetName:='this';
        if NeedClass then
          NeedClass:=NeedAppendClass(SelfScope.SelfArg);
        end;
      TargetJS:=CreatePrimitiveDotExpr(TargetName,PosEl);
      end
    else
      RaiseNotSupported(Expr,AContext,20190205230924);

    if NeedClass then
      // append '.$class'
      TargetJS:=CreateDotExpression(Expr,TargetJS,
                        CreatePrimitiveDotExpr(GetBIName(pbivnPtrClass),PosEl));

    Call:=CreateCallExpression(Expr);
    // "rtl.createCallback"
    if aSafeCall then
      TargetName:=GetBIName(pbifnProcType_CreateSafe)
    else
      TargetName:=GetBIName(pbifnProcType_Create);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),TargetName]);
    // add target
    Call.AddArg(TargetJS);
    TargetJS:=nil;

    // add function name as parameter
    if IsHelper then
      // create  rtl.createCallback(target, THelperType.FunName)
      Call.AddArg(CreateReferencePathExpr(Proc,AContext))
    else
      begin
      // create  rtl.createCallback(target, "FunName")
      if (coShortRefGlobals in Options)
          and (TPas2JSProcedureScope(Proc.CustomData).SpecializedFromItem<>nil) then
        begin
        FunName:=CreateStaticProcPath(Proc,AContext);
        Call.AddArg(CreatePrimitiveDotExpr(FunName,Expr));
        end
      else
        begin
        FunName:=TransformElToJSName(Proc,AContext);
        Call.AddArg(CreateLiteralString(Expr,FunName));
        end;
      end;

    Result:=Call;
  finally
    if Result=nil then
      begin
      TargetJS.Free;
      Call.Free;
      end;
  end;
end;

function TPasToJSConverter.CreateSafeCallback(Expr: TPasExpr; JS: TJSElement;
  AContext: TConvertContext): TJSElement;
var
  Call: TJSCallExpression;
  DotExpr: TJSDotMemberExpression;
  Prim: TJSPrimaryExpressionIdent;
begin
  Result:=JS;
  if AContext=nil then ;
  if JS is TJSCallExpression then
    begin
    Call:=TJSCallExpression(JS);
    if Call.Expr is TJSDotMemberExpression then
      begin
      DotExpr:=TJSDotMemberExpression(Call.Expr);
      if DotExpr.MExpr is TJSPrimaryExpressionIdent then
        begin
        Prim:=TJSPrimaryExpressionIdent(DotExpr.MExpr);
        if Prim.Name=TJSString(GetBIName(pbivnRTL)) then
          begin
          if DotExpr.Name=TJSString(GetBIName(pbifnProcType_Create)) then
            // rtl.createCallback - > rtl.createSafeCallback
            DotExpr.Name:=TJSString(GetBIName(pbifnProcType_CreateSafe));
          end;
        end;
      end;
    // Note: if the call is not a rtl.createCallback then there is no SafeCall
    // e.g.  aSafeCall:=Btn1.GetOnClick();
    end
  else
    begin
    // enclose JS in rtl.createSafeCallback()
    Call:=CreateCallExpression(Expr);
    Result:=Call;
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnProcType_CreateSafe)]);
    if JS is TJSDotMemberExpression then
      begin
      // convert "a.fn"  to "rtl.createSafeCallback(a,fn)"
      DotExpr:=TJSDotMemberExpression(JS);
      Call.AddArg(DotExpr.MExpr);
      DotExpr.MExpr:=nil;
      Call.AddArg(CreateLiteralJSString(Expr,DotExpr.Name));
      JS.Free;
      end
    else
      begin
      // convert "JS"  to  "rtl.createSafeCallback(null,JS)"
      Call.AddArg(CreateLiteralNull(Expr));
      Call.AddArg(JS);
      end;
    end;
end;

function TPasToJSConverter.CreateExternalBracketAccessorCall(El: TParamsExpr;
  AContext: TConvertContext): TJSElement;
var
  Ref: TResolvedReference;
  ArgContext: TConvertContext;
  ok: Boolean;
  AssignSt: TJSSimpleAssignStatement;
  IndexJS: TJSElement;
  WithData: TPas2JSWithExprScope;
  Path: String;
  BracketJS: TJSBracketMemberExpression;
begin
  Result:=nil;
  if length(El.Params)<1 then
    RaiseInconsistency(20180511151259,El);
  if not (El.Value.CustomData is TResolvedReference) then
    RaiseInconsistency(20180511144445,El);
  Ref:=TResolvedReference(El.Value.CustomData);
  ArgContext:=AContext.GetNonDotContext;
  ok:=false;
  try
    // First convert index, because it may raise an exception
    IndexJS:=ConvertExpression(El.Params[0],ArgContext);

    if Ref.WithExprScope<>nil then
      begin
      // with path do GetItems(astring) -> withtmp1[astring]
      WithData:=Ref.WithExprScope as TPas2JSWithExprScope;
      Path:=WithData.WithVarName;
      if Path='' then
        RaiseNotSupported(El,AContext,20190209092417);
      end
    else
      begin
      // GetItems(astring) -> this[astring]
      Path:='this';
      end;
    BracketJS:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
    Result:=BracketJS;
    BracketJS.Name:=IndexJS;
    BracketJS.MExpr:=CreatePrimitiveDotExpr(Path,El);

    if length(El.Params)>1 then
      begin
      // SetItems(astring,value) -> this[astring]:=value
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AssignSt.LHS:=Result;
      Result:=AssignSt;
      AssignSt.Expr:=ConvertExpression(El.Params[1],ArgContext); // may raise an exception
      end;

    if length(El.Params)>2 then
      DoError(20180511144047,nCantCallExtBracketAccessor,sCantCallExtBracketAccessor,[],El);
    ok:=true;
  finally
    if not ok then Result.Free;
  end;
end;

function TPasToJSConverter.CreateAssignStatement(LeftEl: TPasExpr;
  AssignContext: TAssignContext): TJSElement;
var
  LHS: TJSElement;
  AssignSt: TJSSimpleAssignStatement;
begin
  Result:=nil;
  LHS:=ConvertExpression(LeftEl,AssignContext);
  if AssignContext.Call<>nil then
    begin
    // has a setter -> right side was already added as parameter
    if AssignContext.RightSide<>nil then
      begin
      LHS.Free;
      RaiseInconsistency(20170207215447,LeftEl);
      end;
    Result:=LHS;
    end
  else
    begin
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,AssignContext.PasElement));
    AssignSt.LHS:=LHS;
    AssignSt.Expr:=AssignContext.RightSide;
    AssignContext.RightSide:=nil;
    Result:=AssignSt;
    end;
end;

function TPasToJSConverter.CreateGetEnumeratorLoop(El: TPasImplForLoop;
  AContext: TConvertContext): TJSElement;
//  for Item in List do
// convert to
//  var $in=List.GetEnumerator();
//  try{
//    while ($in.MoveNext()){
//      Item=$in.getCurrent;
//      // code
//    }
//  } finally {
//    $in=rtl.freeLoc($in);
//  };
var
  PosEl: TPasElement;
  CurInVar: TFCLocalIdentifier;

  function CreateInName: TJSElement;
  var
    Ident: TJSPrimaryExpressionIdent;
  begin
    Ident:=TJSPrimaryExpressionIdent(CreateElement(TJSPrimaryExpressionIdent,PosEl));
    Ident.Name:=TJSString(CurInVar.Name); // do not lowercase
    Result:=Ident;
  end;

var
  aResolver: TPas2JSResolver;
  ForScope: TPasForLoopScope;
  Statements: TJSStatementList;
  VarSt: TJSVariableStatement;
  FuncContext: TFunctionContext;
  List, GetCurrent, J: TJSElement;
  Call: TJSCallExpression;
  TrySt: TJSTryFinallyStatement;
  WhileSt: TJSWhileStatement;
  AssignSt: TJSSimpleAssignStatement;
  GetEnumeratorFunc, MoveNextFunc: TPasFunction;
  CurrentProp: TPasProperty;
  DotContext: TDotContext;
  ResolvedEl: TPasResolverResult;
  EnumeratorTypeEl: TPasType;
  NeedTryFinally, NeedIntfRef: Boolean;
begin
  aResolver:=AContext.Resolver;
  ForScope:=TPasForLoopScope(El.CustomData);
  NeedTryFinally:=true;
  NeedIntfRef:=false;

  // find function GetEnumerator
  GetEnumeratorFunc:=ForScope.GetEnumerator;
  if (GetEnumeratorFunc=nil) then
    RaiseNotSupported(El,AContext,20171225104212);
  if GetEnumeratorFunc.ClassType<>TPasFunction then
    RaiseNotSupported(El,AContext,20171225104237);
  aResolver.ComputeResultElement(GetEnumeratorFunc.FuncType.ResultEl,ResolvedEl,[rcCall]);
  EnumeratorTypeEl:=ResolvedEl.LoTypeEl;

  if EnumeratorTypeEl is TPasClassType then
    begin
    case TPasClassType(EnumeratorTypeEl).ObjKind of
    okClass,okClassHelper,okRecordHelper,okTypeHelper: ;
    okInterface:
      case TPasClassType(EnumeratorTypeEl).InterfaceType of
      citCom: NeedIntfRef:=true;
      citCorba: NeedTryFinally:=false;
      else
        RaiseNotSupported(El.VariableName,AContext,20180328192842);
      end;
    else
      RaiseNotSupported(El.VariableName,AContext,20180328192452);
    end;
    end;

  // find function MoveNext
  MoveNextFunc:=ForScope.MoveNext;
  if (MoveNextFunc=nil) then
    RaiseNotSupported(El,AContext,20171225104249);
  if MoveNextFunc.ClassType<>TPasFunction then
    RaiseNotSupported(El,AContext,20171225104256);
  if MoveNextFunc.Parent.ClassType<>TPasClassType then
    RaiseNotSupported(El,AContext,20190208153949);
  if TPasClassType(MoveNextFunc.Parent).HelperForType<>nil then
    RaiseNotSupported(El,AContext,20190208155015);
  // find property Current
  CurrentProp:=ForScope.Current;
  if (CurrentProp=nil) then
    RaiseNotSupported(El,AContext,20171225104306);
  if CurrentProp.ClassType<>TPasProperty then
    RaiseNotSupported(El,AContext,20171225104316);
  if CurrentProp.Parent.ClassType<>TPasClassType then
    RaiseNotSupported(El,AContext,20190208154003);

  // get function context
  FuncContext:=AContext.GetFunctionContext;

  PosEl:=El;
  Statements:=TJSStatementList(CreateElement(TJSStatementList,PosEl));
  DotContext:=nil;
  try
    // var...
    VarSt:=TJSVariableStatement(CreateElement(TJSVariableStatement,PosEl));
    Statements.A:=VarSt;
    // List
    List:=ConvertExpression(El.StartExpr,AContext); // beware: might fail
    PosEl:=El.StartExpr;
    // List.GetEnumerator()
    if aResolver.IsHelperMethod(GetEnumeratorFunc) then
      Call:=CreateCallHelperMethod(GetEnumeratorFunc,El.StartExpr,AContext,true)
    else
      begin
      Call:=TJSCallExpression(CreateElement(TJSCallExpression,PosEl));
      Call.Expr:=CreateDotExpression(PosEl,List,
                         CreateIdentifierExpr(GetEnumeratorFunc,AContext),true);
      end;
    // var $in=
    CurInVar:=FuncContext.AddLocalVar(GetBIName(pbivnLoopIn),El.VariableName,cvkNone,true);
    VarSt.A:=CreateVarDecl(CurInVar.Name,Call,PosEl);

    PosEl:=El.VariableName;
    TrySt:=nil;
    if NeedTryFinally then
      begin
      // try()
      TrySt:=TJSTryFinallyStatement(CreateElement(TJSTryFinallyStatement,PosEl));
      Statements.B:=TrySt;
      end;

    // while ()
    WhileSt:=TJSWhileStatement(CreateElement(TJSWhileStatement,PosEl));
    if TrySt<>nil then
      TrySt.Block:=WhileSt
    else
      Statements.B:=WhileSt;
    // $in.MoveNext()
    Call:=TJSCallExpression(CreateElement(TJSCallExpression,PosEl));
    WhileSt.Cond:=Call;
    Call.Expr:=CreateDotExpression(PosEl,CreateInName,
                                   CreateIdentifierExpr(MoveNextFunc,AContext));

    // read property "Current"
    // Item=$in.GetCurrent();  or Item=$in.FCurrent;
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
    WhileSt.Body:=AssignSt;
    AssignSt.LHS:=ConvertExpression(El.VariableName,AContext); // beware: might fail

    DotContext:=TDotContext.Create(El.StartExpr,nil,AContext);
    try
      GetCurrent:=CreatePropertyGet(CurrentProp,nil,DotContext,PosEl); // beware: might fail
      if DotContext.JS<>nil then
        RaiseNotSupported(El,AContext,20180509134302,GetObjName(DotContext.JS));
    finally
      FreeAndNil(DotContext);
    end;
    AssignSt.Expr:=CreateDotExpression(PosEl,CreateInName,GetCurrent,true);

    // add body
    if El.Body<>nil then
      begin
      J:=ConvertElement(El.Body,AContext); // beware: might fail
      if J<>nil then
        begin
        List:=TJSStatementList(CreateElement(TJSStatementList,PosEl));
        TJSStatementList(List).A:=WhileSt.Body;
        TJSStatementList(List).B:=J;
        WhileSt.Body:=List;
        end;
      end;

    PosEl:=El.StartExpr;
    if TrySt<>nil then
      begin
      // finally{ $in=rtl.freeLoc($in) }
      if NeedIntfRef then
        begin
        Call:=CreateCallExpression(PosEl);
        TrySt.BFinally:=Call;
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntf_Release)]);
        Call.AddArg(CreateInName);
        end
      else
        TrySt.BFinally:=CreateCallRTLFreeLoc(CreateInName,CreateInName,PosEl);
      end;

    Result:=Statements;
  finally
    DotContext.Free;
    if Result=nil then
      Statements.Free;
  end;
end;

function TPasToJSConverter.CreateCallRTLFreeLoc(Setter, Getter: TJSElement;
  Src: TPasElement): TJSElement;
// create "Setter=rtl.freeLoc(Getter)"
var
  Call: TJSCallExpression;
  AssignSt: TJSSimpleAssignStatement;
begin
  Call:=CreateCallExpression(Src);
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnFreeLocalVar)]);
  Call.Args.AddElement(Getter);
  AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,Src));
  AssignSt.LHS:=Setter;
  AssignSt.Expr:=Call;
  Result:=AssignSt;
end;

function TPasToJSConverter.CreatePropertyGet(Prop: TPasProperty;
  Expr: TPasExpr; AContext: TConvertContext; PosEl: TPasElement): TJSElement;
var
  aResolver: TPas2JSResolver;
  Decl: TPasElement;
  Call: TJSCallExpression;
  Name: String;
  Ref: TResolvedReference;
begin
  aResolver:=AContext.Resolver;
  Decl:=aResolver.GetPasPropertyGetter(Prop);
  if (Expr<>nil) and (Expr.CustomData is TResolvedReference) then
    Ref:=TResolvedReference(Expr.CustomData)
  else
    Ref:=nil;
  if Decl is TPasFunction then
    begin
    // call function
    if aResolver.IsHelperMethod(Decl) then
      begin
      if (Expr=nil) then
        // implicit property read, e.g. enumerator property Current
        RaiseNotSupported(PosEl,AContext,20190208111355,GetObjName(Prop));
      Result:=CreateCallHelperMethod(TPasProcedure(Decl),Expr,AContext);
      exit;
      end;
    Call:=CreateCallExpression(PosEl);
    try
      Call.Expr:=CreateReferencePathExpr(Decl,AContext,false,Ref);
      Result:=AppendPropertyReadArgs(Call,Prop,AContext,PosEl);
    finally
      if Result=nil then
        Call.Free;
    end;
    end
  else
    begin
    // read field
    Name:=CreateReferencePath(Decl,AContext,rpkPathAndName,false,Ref);
    Result:=CreatePrimitiveDotExpr(Name,PosEl);
    end;
end;

function TPasToJSConverter.AppendPropertyAssignArgs(Call: TJSCallExpression;
  Prop: TPasProperty; AssignContext: TAssignContext; PosEl: TPasElement
  ): TJSCallExpression;
var
  aResolver: TPas2JSResolver;
  IndexExpr: TPasExpr;
  Value: TResEvalValue;
begin
  AssignContext.Call:=Call;
  AssignContext.PropertyEl:=Prop;
  aResolver:=AssignContext.Resolver;
  IndexExpr:=aResolver.GetPasPropertyIndex(Prop);
  if IndexExpr<>nil then
    begin
    Value:=aResolver.Eval(IndexExpr,[refConst]);
    try
      Call.AddArg(ConvertConstValue(Value,AssignContext,PosEl));
    finally
      ReleaseEvalValue(Value);
    end;
    end;
  Call.AddArg(AssignContext.RightSide);
  AssignContext.RightSide:=nil;
  Result:=Call;
end;

function TPasToJSConverter.AppendPropertyReadArgs(Call: TJSCallExpression;
  Prop: TPasProperty; aContext: TConvertContext; PosEl: TPasElement
  ): TJSCallExpression;
var
  aResolver: TPas2JSResolver;
  IndexExpr: TPasExpr;
  Value: TResEvalValue;
  TypeEl: TPasType;
begin
  aResolver:=aContext.Resolver;
  IndexExpr:=aResolver.GetPasPropertyIndex(Prop);
  if IndexExpr<>nil then
    begin
    Value:=aResolver.Eval(IndexExpr,[refConst]);
    try
      Call.AddArg(ConvertConstValue(Value,AContext.GetFunctionContext,PosEl));
    finally
      ReleaseEvalValue(Value);
    end;
    end;
  TypeEl:=aResolver.GetPasPropertyType(Prop);
  if aResolver.IsInterfaceType(TypeEl,citCom) then
    Call:=CreateIntfRef(Call,AContext,PosEl);
  Result:=Call;
end;

function TPasToJSConverter.CreateDotSplit(El: TPasElement; Expr: TJSElement
  ): TJSElement;
var
  DotExpr: TJSDotMemberExpression;
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(El);
  DotExpr:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,El));
  Call.Expr:=DotExpr;
  DotExpr.MExpr:=Expr;
  DotExpr.Name:='split';
  Call.AddArg(CreateLiteralJSString(El,''));
  Result:=Call;
end;

function TPasToJSConverter.CreatePrecompiledJS(El: TJSElement): string;
var
  aWriter: TBufferWriter;
  aJSWriter: TJSWriter;
begin
  aJSWriter:=nil;
  aWriter:=TBufferWriter.Create(1000);
  try
    aJSWriter:=TJSWriter.Create(aWriter);
    aJSWriter.Options:=DefaultJSWriterOptions;
    aJSWriter.IndentSize:=2;
    aJSWriter.SkipCurlyBrackets:=true;
    aJSWriter.Writer.LineBreak:=#10;
    aJSWriter.WriteJS(El);
    Result:=aWriter.AsString;
  finally
    aJSWriter.Free;
    aWriter.Free;
  end;
end;

function TPasToJSConverter.CreateRaisePropReadOnly(PosEl: TPasElement
  ): TJSElement;
var
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(PosEl);
  Result:=Call;
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnRaiseException)]);
  Call.AddArg(CreateLiteralJSString(PosEl,'EPropReadOnly'));
end;

procedure TPasToJSConverter.AddRTLVersionCheck(FuncContext: TFunctionContext;
  PosEl: TPasElement);
var
  Call: TJSCallExpression;
begin
  // rtl.checkVersion(RTLVersion)
  Call:=CreateCallExpression(PosEl);
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnCheckVersion)]);
  Call.AddArg(CreateLiteralNumber(PosEl,FGlobals.RTLVersion));
  PrependToStatementList(FuncContext.BodySt,Call,PosEl);
end;

function TPasToJSConverter.CreateTypeInfoRef(El: TPasType;
  AContext: TConvertContext; ErrorEl: TPasElement): TJSElement;
var
  aName, aModName: String;
  aModule: TPasModule;
  Bracket: TJSBracketMemberExpression;
begin
  El:=ResolveSimpleAliasType(El);
  if El is TPasSpecializeType then
    El:=TPasSpecializeTypeData(El.CustomData).SpecializedType;
  aName:=GetTypeInfoName(El,AContext,ErrorEl);
  if aName=GetBIName(pbivnRTTILocal) then
    Result:=CreatePrimitiveDotExpr(aName,El)
  else if LeftStr(aName,length(GetBIName(pbivnRTL))+1)=GetBIName(pbivnRTL)+'.' then
    Result:=CreatePrimitiveDotExpr(aName,El)
  else
    begin
    aModule:=El.GetModule;
    aModName:=TransformModuleName(aModule,true,AContext);
    Bracket:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
    Bracket.MExpr:=CreateMemberExpression([aModName,GetBIName(pbivnRTTI)]);
    Bracket.Name:=CreateLiteralString(El,aName);
    Result:=Bracket;
    end;
end;

function TPasToJSConverter.CreateRTTIArgList(Parent: TPasElement;
  Args: TFPList; AContext: TConvertContext): TJSElement;
var
  Params: TJSArrayLiteral;
  i: Integer;
begin
  Result:=nil;
  if Args.Count=0 then
    Result:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,Parent))
  else
    try
      Params:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,Parent));
      for i:=0 to Args.Count-1 do
        AddRTTIArgument(TPasArgument(Args[i]),Params,AContext);
      Result:=Params;
    finally
      if Result=nil then
        Params.Free;
    end;
end;

procedure TPasToJSConverter.AddRTTIArgument(Arg: TPasArgument;
  TargetParams: TJSArrayLiteral; AContext: TConvertContext);
var
  Param: TJSArrayLiteral;
  ArgName: String;
  Flags: Integer;
  ArrType: TPasArrayType;
  aResolver: TPas2JSResolver;
begin
  aResolver:=AContext.Resolver;
  // for each param add  "["argname",argtype,flags]"  Note: flags only if >0
  Param:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,Arg));
  TargetParams.Elements.AddElement.Expr:=Param;
  // add "argname"
  ArgName:=TransformToJSName(Arg,Arg.Name,true,AContext); // use Pascal name
  Param.Elements.AddElement.Expr:=CreateLiteralString(Arg,ArgName);
  Flags:=0;
  // add "argtype"
  if Arg.ArgType=nil then
    // untyped
    Param.Elements.AddElement.Expr:=CreateLiteralNull(Arg)
  else if (Arg.ArgType.Name='') and (Arg.ArgType.ClassType=TPasArrayType) then
    begin
    // open array param
    inc(Flags,pfArray);
    ArrType:=TPasArrayType(Arg.ArgType);
    Param.Elements.AddElement.Expr:=
              CreateTypeInfoRef(aResolver.GetArrayElType(ArrType),AContext,Arg);
    end
  else
    Param.Elements.AddElement.Expr:=CreateTypeInfoRef(Arg.ArgType,AContext,Arg);
  // add flags
  case Arg.Access of
    argDefault: ;
    argConst,argConstRef: inc(Flags,pfConst);
    argVar: inc(Flags,pfVar);
    argOut: inc(Flags,pfOut);
  else
    RaiseNotSupported(Arg,AContext,20170409192127,AccessNames[Arg.Access]);
  end;
  if Flags>0 then
    Param.Elements.AddElement.Expr:=CreateLiteralNumber(Arg,Flags);
end;

function TPasToJSConverter.GetClassBIName(El: TPasClassType;
  AContext: TConvertContext): string;
begin
  case El.ObjKind of
  okClass:
    if El.IsExternal then
      Result:=GetBIName(pbifnRTTINewExtClass)
    else
      Result:=GetBIName(pbifnRTTINewClass);
  okInterface:
    Result:=GetBIName(pbifnRTTINewInterface);
  else
    RaiseNotSupported(El,AContext,20190128102749);
  end;
end;

function TPasToJSConverter.CreateRTTINewType(El: TPasType;
  const CallFuncName: string; IsForward: boolean; AContext: TConvertContext;
  out ObjLit: TJSObjectLiteral): TJSCallExpression;
// module.$rtti.$Something("name",{})
var
  RttiPath, TypeName: String;
  Call: TJSCallExpression;
  aModule: TPasModule;
  aResolver: TPas2JSResolver;
  Attr: TPasExprArray;
  AttrJS: TJSElement;
  ObjLitEl: TJSObjectLiteralElement;
begin
  Result:=nil;
  ObjLit:=nil;

  aResolver:=AContext.Resolver;
  // get module path
  aModule:=El.GetModule;
  if aModule=nil then
    RaiseInconsistency(20170418115552,El);
  RttiPath:=TransformModuleName(aModule,true,AContext);

  Call:=CreateCallExpression(El);
  try
    // module.$rtti.$Something
    Call.Expr:=CreateMemberExpression([RttiPath,GetBIName(pbivnRTTI),CallFuncName]);
    // add param "typename"
    TypeName:=GetTypeInfoName(El,AContext,El,true);
    Call.AddArg(CreateLiteralString(El,TypeName));
    if El is TPasTypeAliasType then
      begin
      // add desttype
      Call.AddArg(CreateTypeInfoRef(TPasTypeAliasType(El).DestType,AContext,El));
      end;
    if not IsForward then
      begin
      // add {}
      ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
      Call.AddArg(ObjLit);

      Attr:=aResolver.GetAttributeCallsEl(El);
      AttrJS:=CreateRTTIAttributes(Attr,El,AContext);
      if AttrJS<>nil then
        begin
        // attr: [...]
        ObjLitEl:=ObjLit.Elements.AddElement;
        ObjLitEl.Name:=TJSString(GetBIName(pbivnRTTITypeAttributes));
        ObjLitEl.Expr:=AttrJS;
        end;
      end;

    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.CreateRTTIAttributes(const Attr: TPasExprArray;
  PosEl: TPasElement; aContext: TConvertContext): TJSElement;
// create [Attr1Class,'Attr1ProcName',[Attr1Params],...]
var
  AttrArrayLit, ParamsArrayLit: TJSArrayLiteral;
  i, j: Integer;
  Expr, ParamExpr: TPasExpr;
  aResolver: TPas2JSResolver;
  Ref: TResolvedReference;
  AttrClass, ConstrParent: TPasClassType;
  aConstructor: TPasConstructor;
  aName: String;
  Params: TPasExprArray;
  Value: TResEvalValue;
  JSExpr: TJSElement;
begin
  Result:=nil;
  aResolver:=aContext.Resolver;
  AttrArrayLit:=nil;
  try
    for i:=0 to length(Attr)-1 do
      begin
      Expr:=Attr[i];
      if Expr is TParamsExpr then
        Expr:=TParamsExpr(Expr).Value;
      if (Expr is TBinaryExpr) and (TBinaryExpr(Expr).OpCode=eopSubIdent) then
        Expr:=TBinaryExpr(Expr).right;
      if not aResolver.IsNameExpr(Expr) then
        RaiseNotSupported(Expr,aContext,20190222182742,GetObjName(Expr));
      // attribute class
      Ref:=Expr.CustomData as TResolvedReference;
      if Ref=nil then
        // unknown attribute -> silently skip (delphi 10.3 compatible)
        continue;
      AttrClass:=Ref.Declaration as TPasClassType;
      if AttrClass.IsAbstract then
        continue; // silently skip abstract class (Delphi 10.3 compatible)
      // attribute constructor name as string
      if not (Ref.Context is TResolvedRefCtxAttrProc) then
        RaiseNotSupported(Expr,aContext,20190223085831,GetObjName(Expr));
      aConstructor:=TResolvedRefCtxAttrProc(Ref.Context).Proc;
      if aConstructor.IsAbstract then
        continue; // silently skip abstract method (Delphi 10.3 compatible)
      ConstrParent:=aConstructor.Parent as TPasClassType;
      if ConstrParent.HelperForType<>nil then
        aResolver.RaiseMsg(20190223220134,nXExpectedButYFound,sXExpectedButYFound,
          ['class method','helper method'],Expr);
      aName:=TransformElToJSName(aConstructor,aContext);

      if AttrArrayLit=nil then
        AttrArrayLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,PosEl));

      // add class reference  pas.system.TCustomAttribute
      AttrArrayLit.AddElement(CreateReferencePathExpr(AttrClass,aContext));
      // add constructor name 'Create$1'
      AttrArrayLit.AddElement(CreateLiteralString(PosEl,aName));
      // add attribute params as [] if needed
      ParamsArrayLit:=nil;
      Expr:=Attr[i];
      if Expr is TParamsExpr then
        begin
        Params:=TParamsExpr(Expr).Params;
        for j:=0 to length(Params)-1 do
          begin
          ParamExpr:=Params[j];
          Value:=aResolver.Eval(ParamExpr,[]);
          if Value<>nil then
            try
              JSExpr:=ConvertConstValue(Value,aContext,PosEl);
            finally
              ReleaseEvalValue(Value);
            end
          else
            JSExpr:=ConvertExpression(ParamExpr,aContext);
          if ParamsArrayLit=nil then
            begin
            ParamsArrayLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,PosEl));
            AttrArrayLit.AddElement(ParamsArrayLit);
            end;
          ParamsArrayLit.AddElement(JSExpr);
          end;
        end;
      end;
    Result:=AttrArrayLit;
  finally
    if Result=nil then
      AttrArrayLit.Free;
  end;
end;

function TPasToJSConverter.CreateRTTIMemberField(Members: TFPList;
  Index: integer; AContext: TConvertContext): TJSElement;
// create $r.addField("varname",typeinfo);
// create $r.addField("varname",typeinfo,options);
var
  V: TPasVariable;
  Call: TJSCallExpression;
  OptionsEl: TJSObjectLiteral;

  procedure AddOption(const aName: String; JS: TJSElement);
  var
    ObjLit: TJSObjectLiteralElement;
  begin
    if JS=nil then exit;
    if OptionsEl=nil then
      begin
      OptionsEl:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,V));
      Call.AddArg(OptionsEl);
      end;
    ObjLit:=OptionsEl.Elements.AddElement;
    ObjLit.Name:=TJSString(aName);
    ObjLit.Expr:=JS;
  end;

  function VarTypeInfoAlreadyCreated(VarType: TPasType): boolean;
  var
    i: Integer;
    PrevMember: TPasElement;
  begin
    i:=Index-1;
    while (i>=0) do
      begin
      PrevMember:=TPasElement(Members[i]);
      if (PrevMember is TPasVariable) and (TPasVariable(PrevMember).VarType=VarType)
          and IsElementUsed(PrevMember) then
        exit(true);
      dec(i);
      end;
    Result:=false;
  end;

var
  JSTypeInfo: TJSElement;
  aName: String;
  aResolver: TPas2JSResolver;
  Attr: TPasExprArray;
  VarType: TPasType;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  V:=TPasVariable(Members[Index]);
  VarType:=V.VarType;
  if (VarType<>nil) and (VarType.Name='') then
    begin
    if not VarTypeInfoAlreadyCreated(VarType) then
      CreateRTTIAnonymous(VarType,AContext); // only needed by precompiled files from 2.0.0
    end;

  JSTypeInfo:=CreateTypeInfoRef(VarType,AContext,V);
  OptionsEl:=nil;

  // Note: create JSTypeInfo first, it may raise an exception
  Call:=CreateCallExpression(V);
  try
    // $r.addField
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTTILocal),GetBIName(pbifnRTTIAddField)]);
    // param "varname"
    aName:=TransformElToJSName(V,AContext);
    Call.AddArg(CreateLiteralString(V,aName));
    // param typeinfo
    Call.AddArg(JSTypeInfo);

    // param options if needed as {}
    // option: attributes
    Attr:=aResolver.GetAttributeCalls(Members,Index);
    if length(Attr)>0 then
      AddOption(GetBIName(pbivnRTTIMemberAttributes),
                CreateRTTIAttributes(Attr,V,AContext));

    Result:=Call;
    Call:=nil;
  finally
    Call.Free;
  end;
end;

function TPasToJSConverter.CreateRTTIMemberMethod(Members: TFPList;
  Index: integer; AContext: TConvertContext): TJSElement;
// create $r.addMethod("funcname",methodkind,params,resulttype,options)
var
  Proc: TPasProcedure;
  OptionsEl: TJSObjectLiteral;
  ResultTypeInfo: TJSElement;
  Call: TJSCallExpression;
  Flags: Integer;

  procedure AddOption(const aName: String; JS: TJSElement);
  var
    ObjLit: TJSObjectLiteralElement;
  begin
    if JS=nil then exit;
    if OptionsEl=nil then
      begin
      OptionsEl:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,Proc));
      Call.AddArg(OptionsEl);
      end;
    ObjLit:=OptionsEl.Elements.AddElement;
    ObjLit.Name:=TJSString(aName);
    ObjLit.Expr:=JS;
  end;

var
  FunName: String;
  C: TClass;
  MethodKind: Integer;
  ResultEl: TPasResultElement;
  ProcScope, OverriddenProcScope: TPasProcedureScope;
  OverriddenClass: TPasClassType;
  aResolver: TPas2JSResolver;
  Attr: TPasExprArray;
begin
  Result:=nil;
  Proc:=TPasProcedure(Members[Index]);
  aResolver:=AContext.Resolver;
  if Proc.IsOverride then
    begin
    ProcScope:=Proc.CustomData as TPasProcedureScope;
    if ProcScope.OverriddenProc.Visibility=visPublished then
      begin
      // overridden proc is published as well
      OverriddenProcScope:=ProcScope.OverriddenProc.CustomData as TPasProcedureScope;
      OverriddenClass:=OverriddenProcScope.ClassRecScope.Element as TPasClassType;
      if HasTypeInfo(OverriddenClass,AContext) then
        exit; // overridden proc was already published in ancestor
      end;
    end;
  if (Proc.ClassType=TPasClassConstructor)
      or (Proc.ClassType=TPasClassDestructor) then
    exit; // no RTTI for class constructor

  OptionsEl:=nil;
  ResultTypeInfo:=nil;
  try
    // $r.addMethod
    Call:=CreateCallExpression(Proc);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTTILocal),GetBIName(pbifnRTTIAddMethod)]);

    // param "funname"
    FunName:=TransformElToJSName(Proc,AContext);
    Call.AddArg(CreateLiteralString(Proc,FunName));

    // param methodkind as number
    C:=Proc.ClassType;
    if C=TPasProcedure then
      MethodKind:=ord(mkProcedure)
    else if C=TPasFunction then
      MethodKind:=ord(mkFunction)
    else if C=TPasConstructor then
      MethodKind:=ord(mkConstructor)
    else if C=TPasDestructor then
      MethodKind:=ord(mkDestructor)
    else if C=TPasClassProcedure then
      MethodKind:=ord(mkClassProcedure)
    else if C=TPasClassFunction then
      MethodKind:=ord(mkClassFunction)
    else
      RaiseNotSupported(Proc,AContext,20170409190242);
    Call.AddArg(CreateLiteralNumber(Proc,MethodKind));

    // param params as []
    Call.AddArg(CreateRTTIArgList(Proc,Proc.ProcType.Args,AContext));

    // optional params:
    ResultTypeInfo:=nil;
    Flags:=0;
    if Proc.IsStatic then
      inc(Flags,pfStatic);
    if ptmVarargs in Proc.ProcType.Modifiers then
      inc(Flags,pfVarargs);
    if ptmAsync in Proc.ProcType.Modifiers then
      inc(Flags,pfAsync);
    if Proc.IsExternal then
      inc(Flags,pfExternal);
    Attr:=aResolver.GetAttributeCalls(Members,Index);

    // param resulttype as typeinfo reference
    if C.InheritsFrom(TPasFunction) then
      begin
      ResultEl:=TPasFunction(Proc).FuncType.ResultEl;
      ResultTypeInfo:=CreateTypeInfoRef(ResultEl.ResultType,AContext,ResultEl);
      if ResultTypeInfo<>nil then
        Call.AddArg(ResultTypeInfo);
      end;
    if (ResultTypeInfo=nil) and ((Flags>0) or (length(Attr)>0)) then
      Call.AddArg(CreateLiteralNull(Proc));

    // flags if needed
    if (Flags>0) or (length(Attr)>0) then
      Call.AddArg(CreateLiteralNumber(Proc,Flags));

    // param options if needed as {}
    if length(Attr)>0 then
      AddOption(GetBIName(pbivnRTTIMemberAttributes),
                CreateRTTIAttributes(Attr,Proc,AContext));

    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.CreateRTTIMemberProperty(Members: TFPList;
  Index: integer; AContext: TConvertContext): TJSElement;
// create  $r.addProperty("propname",flags,proptype,"getter","setter",{options})
var
  Prop: TPasProperty;
  Call: TJSCallExpression;
  OptionsEl: TJSObjectLiteral;

  function GetAccessorName(Decl: TPasElement): String;
  begin
    Result:=TransformElToJSName(Decl,AContext);
  end;

  procedure AddOption(const aName: String; JS: TJSElement);
  var
    ObjLit: TJSObjectLiteralElement;
  begin
    if JS=nil then exit;
    if OptionsEl=nil then
      begin
      OptionsEl:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,Prop));
      Call.AddArg(OptionsEl);
      end;
    ObjLit:=OptionsEl.Elements.AddElement;
    ObjLit.Name:=TJSString(aName);
    ObjLit.Expr:=JS;
  end;

var
  PropName: String;
  Flags: Integer;
  GetterPas, SetterPas, DeclEl: TPasElement;
  ResultTypeInfo, DefValue: TJSElement;
  VarType: TPasType;
  StoredExpr, IndexExpr, DefaultExpr: TPasExpr;
  StoredResolved, VarTypeResolved: TPasResolverResult;
  StoredValue, PasValue, IndexValue: TResEvalValue;
  aResolver: TPas2JSResolver;
  Attr: TPasExprArray;
begin
  Result:=nil;
  Prop:=TPasProperty(Members[Index]);
  aResolver:=AContext.Resolver;
  OptionsEl:=nil;
  try
    // $r.addProperty
    Call:=CreateCallExpression(Prop);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTTILocal),GetBIName(pbifnRTTIAddProperty)]);

    // param "propname"
    PropName:=TransformToJSName(Prop,Prop.Name,false,AContext); // use Pascal name
    Call.AddArg(CreateLiteralString(Prop,PropName));

    // add flags
    Flags:=0;
    GetterPas:=aResolver.GetPasPropertyGetter(Prop);
    if GetterPas is TPasProcedure then
      inc(Flags,pfGetFunction);
    SetterPas:=aResolver.GetPasPropertySetter(Prop);
    if SetterPas is TPasProcedure then
      inc(Flags,pfSetProcedure);
    StoredExpr:=aResolver.GetPasPropertyStoredExpr(Prop);
    IndexExpr:=aResolver.GetPasPropertyIndex(Prop);
    if IndexExpr<>nil then
      inc(Flags,pfHasIndex);
    DefaultExpr:=aResolver.GetPasPropertyDefaultExpr(Prop);
    if StoredExpr<>nil then
      begin
      aResolver.ComputeElement(StoredExpr,StoredResolved,[rcNoImplicitProc]);
      if StoredResolved.IdentEl is TPasProcedure then
        // stored <function>
        inc(Flags,pfStoredFunction)
      else
        begin
        if (StoredResolved.BaseType=btBoolean) and (StoredResolved.ExprEl<>nil) then
          begin
          // could be a const boolean
          // -> try evaluating const boolean
          StoredValue:=aResolver.Eval(StoredExpr,[]);
          if StoredValue<>nil then
            try
              // stored <const bool>
              if StoredValue.Kind<>revkBool then
                RaiseInconsistency(20170924082845,Prop);
              StoredExpr:=nil;
              if TResEvalBool(StoredValue).B then
                inc(Flags,pfStoredTrue)
              else
                inc(Flags,pfStoredFalse);
            finally
              ReleaseEvalValue(StoredValue);
            end;
          end;
        if StoredExpr<>nil then
          // stored <field>
          inc(Flags,pfStoredField);
        end;
      end;
    Call.AddArg(CreateLiteralNumber(Prop,Flags));

    // add type
    VarType:=aResolver.GetPasPropertyType(Prop);
    aResolver.ComputeElement(VarType,VarTypeResolved,[rcType]);
    ResultTypeInfo:=CreateTypeInfoRef(VarType,AContext,Prop);
    if ResultTypeInfo<>nil then
      Call.AddArg(ResultTypeInfo)
    else
      Call.AddArg(CreateLiteralNull(Prop));

    // add "getter"
    if GetterPas=nil then
      Call.AddArg(CreateLiteralString(Prop,''))
    else
      Call.AddArg(CreateLiteralString(Prop,GetAccessorName(GetterPas)));

    // add "setter"
    if SetterPas=nil then
      Call.AddArg(CreateLiteralString(Prop,''))
    else
      Call.AddArg(CreateLiteralString(Prop,GetAccessorName(SetterPas)));

    // add option "index"
    IndexExpr:=aResolver.GetPasPropertyIndex(Prop);
    if IndexExpr<>nil then
      begin
      IndexValue:=aResolver.Eval(IndexExpr,[refConst]);
      try
        AddOption(GetBIName(pbivnRTTIPropIndex),
          ConvertConstValue(IndexValue,AContext,Prop));
      finally
        ReleaseEvalValue(IndexValue);
      end;
      end;

    // add option "stored"
    if StoredExpr<>nil then
      begin
      DeclEl:=(StoredExpr.CustomData as TResolvedReference).Declaration;
      AddOption(GetBIName(pbivnRTTIPropStored),
        CreateLiteralString(Prop,GetAccessorName(DeclEl)));
      end;

    // add option "defaultvalue"
    if DefaultExpr<>nil then
      begin
      PasValue:=aResolver.Eval(DefaultExpr,[refConst],false);
      try
        DefValue:=nil;
        if VarTypeResolved.BaseType in [btSet,btArrayOrSet] then
          DefValue:=CreateValInit(VarType,DefaultExpr,DefaultExpr,AContext);
        if DefValue=nil then
          DefValue:=ConvertConstValue(PasValue,AContext,Prop);
        AddOption(GetBIName(pbivnRTTIPropDefault),DefValue);
      finally
        ReleaseEvalValue(PasValue);
      end;
      end;

    // add option "attr"
    Attr:=aResolver.GetAttributeCalls(Members,Index);
    if length(Attr)>0 then
      AddOption(GetBIName(pbivnRTTIMemberAttributes),
        CreateRTTIAttributes(Attr,Prop,AContext));

    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

procedure TPasToJSConverter.CreateRTTIAnonymous(El: TPasType;
  AContext: TConvertContext);
// if El has any anonymous types, create the RTTI
var
  C: TClass;
  JS: TJSElement;
  GlobalCtx: TFunctionContext;
  Src: TJSSourceElements;
begin
  if El.Name<>'' then
    RaiseNotSupported(El,AContext,20170905162324,'inconsistency');

  GlobalCtx:=AContext.GetGlobalFunc;
  if GlobalCtx=nil then
    RaiseNotSupported(El,AContext,20181229130835);
  if not (GlobalCtx.JSElement is TJSSourceElements) then
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.CreateRTTIAnonymous GlobalCtx=',GetObjName(GlobalCtx),' JSElement=',GetObjName(GlobalCtx.JSElement));
    {$ENDIF}
    RaiseNotSupported(El,AContext,20181229130926);
    end;
  Src:=TJSSourceElements(GlobalCtx.JSElement);
  C:=El.ClassType;
  if C=TPasArrayType then
    begin
    JS:=ConvertArrayType(TPasArrayType(El),AContext);
    AddToSourceElements(Src,JS);
    end;
end;

function TPasToJSConverter.CreateRTTIAnonymousArray(El: TPasArrayType;
  AContext: TConvertContext): TJSCallExpression;
var
  Scope: TPas2JSArrayScope;
  SpecializeDelay: Boolean;
  CallName: String;
  Call: TJSCallExpression;
  Obj: TJSObjectLiteral;
  aResolver: TPas2JSResolver;
  ElTypeHi, ElTypeLo: TPasType;
  Prop: TJSObjectLiteralElement;
  ArrLit: TJSArrayLiteral;
  Arr: TPasArrayType;
  Index: Integer;
  RangeEl: TPasExpr;
  RgLen: TMaxPrecInt;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;

  Scope:=El.CustomData as TPas2JSArrayScope;
  SpecializeDelay:=(Scope<>nil) and (SpecializeNeedsDelay(El,AContext));

  // module.$rtti.$DynArray("name",{...})
  if length(El.Ranges)>0 then
    CallName:=GetBIName(pbifnRTTINewStaticArray)
  else
    CallName:=GetBIName(pbifnRTTINewDynArray);
  Call:=CreateRTTINewType(El,CallName,false,AContext,Obj);
  try
    ElTypeHi:=aResolver.ResolveAliasType(El.ElType,false);
    ElTypeLo:=aResolver.ResolveAliasType(ElTypeHi);
    if length(El.Ranges)>0 then
      begin
      // static array
      // dims: [dimsize1,dimsize2,...]
      Prop:=Obj.Elements.AddElement;
      Prop.Name:=TJSString(GetBIName(pbivnRTTIArray_Dims));
      ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
      Prop.Expr:=ArrLit;
      Arr:=El;
      Index:=0;
      repeat
        RangeEl:=Arr.Ranges[Index];
        RgLen:=aResolver.GetRangeLength(RangeEl);
        ArrLit.AddElement(CreateLiteralNumber(RangeEl,RgLen));
        inc(Index);
        if Index=length(Arr.Ranges) then
          begin
          if ElTypeLo.ClassType<>TPasArrayType then
            break;
          Arr:=TPasArrayType(ElTypeLo);
          if length(Arr.Ranges)=0 then
            RaiseNotSupported(Arr,AContext,20170411222315,'static array of anonymous array');
          ElTypeHi:=aResolver.ResolveAliasType(Arr.ElType,false);
          ElTypeLo:=aResolver.ResolveAliasType(ElTypeHi);
          Index:=0;
          end;
      until false;
      end;
    // eltype: ref
    if not SpecializeDelay then
      begin
      Prop:=Obj.Elements.AddElement;
      Prop.Name:=TJSString(GetBIName(pbivnRTTIArray_ElType));
      Prop.Expr:=CreateTypeInfoRef(ElTypeHi,AContext,El);
      end;
    Result:=Call;
  finally
    if Result=nil then
      Call.Free;
  end;
end;

function TPasToJSConverter.CreateRTTIMembers(El: TPasMembersType;
  Src: TJSSourceElements; FuncContext: TFunctionContext;
  MembersSrc: TJSSourceElements; MembersFuncContext: TFunctionContext;
  RTTIExpr: TJSElement; NeedLocalVar: boolean): boolean;
type
  TMemberType = (
    mtClass,
    mtInterface,
    mtRecord
    );

  procedure CreateLocalvar;
  var
    VarSt: TJSVariableStatement;
  begin
    if Result then exit;
    // add "var $r = module.$rtti.$Record..."
    Result:=true;
    VarSt:=CreateVarStatement(GetBIName(pbivnRTTILocal),RTTIExpr,El);
    if Src=MembersSrc then
      AddToSourceElements(Src,VarSt)
    else
      Src.Statements.InsertNode(0).Node:=VarSt;
  end;

var
  mt: TMemberType;
  i: integer;
  P: TPasElement;
  C: TClass;
  NewEl: TJSElement;
  Members: TFPList;
  aResolver: TPas2JSResolver;
begin
  Result:=false;
  aResolver:=FuncContext.Resolver;
  if El.ClassType=TPasRecordType then
    mt:=mtRecord
  else if El.ClassType=TPasClassType then
    case TPasClassType(El).ObjKind of
    okInterface: mt:=mtInterface;
    else mt:=mtClass;
    end
  else
    RaiseNotSupported(El,FuncContext,20190223211808,GetObjName(El));

  // add $r to local vars, to avoid name clashes and for nicer debugging
  FuncContext.AddLocalJSVar(GetBIName(pbivnRTTILocal),false);

  if NeedLocalVar then
    CreateLocalvar;

  Members:=El.Members;
  For i:=0 to Members.Count-1 do
    begin
    NewEl:=nil;
    P:=TPasElement(Members[i]);
    C:=P.ClassType;
    //writeln('TPasToJSConverter.CreateRTTIMembers ',GetObjPath(P));
    if C.InheritsFrom(TPasType) and HasTypeInfo(TPasType(P),MembersFuncContext) then
      begin
      // published subtype
      if aResolver.IsAnonymousElType(TPasType(P)) then
        begin
        // published anonymous eltype
        if C.InheritsFrom(TPasArrayType) then
          NewEl:=CreateRTTIAnonymousArray(TPasArrayType(P),MembersFuncContext);
        end;
      end
    else
      begin
      // check visibility
      case mt of
      mtClass:
        if P.Visibility<>visPublished then continue;
      mtInterface: ; // all members of an interface are published
      mtRecord:
        // a published record publishes all non private members
        if P.Visibility in [visPrivate,visStrictPrivate] then
          continue
        else if P.ClassType=TPasConst then
          continue;
      end;
      if not IsElementUsed(P) then continue;

      if C=TPasVariable then
        NewEl:=CreateRTTIMemberField(Members,i,MembersFuncContext)
      else if C.InheritsFrom(TPasProcedure) then
        begin
        if aResolver.GetProcTemplateTypes(TPasProcedure(P))<>nil then
          continue; // parametrized functions cannot be published
        NewEl:=CreateRTTIMemberMethod(Members,i,MembersFuncContext);
        end
      else if C=TPasProperty then
        NewEl:=CreateRTTIMemberProperty(Members,i,MembersFuncContext)
      else if C.InheritsFrom(TPasType)
          or (C=TPasAttributes) then
      else
        DoError(20190105142236,nSymbolCannotBePublished,sSymbolCannotBePublished,[],P);
      end;
    if NewEl=nil then
      continue; // e.g. abstract or external proc
    // add RTTI element
    if not Result then
      CreateLocalvar;
    AddToSourceElements(MembersSrc,NewEl);
    end;
end;

procedure TPasToJSConverter.AddIntfDelegations(ClassEl: TPasElement;
  Prop: TPasProperty; FinishedGUIDs: TStringList; ObjLit: TJSObjectLiteral;
  aContext: TFunctionContext);
var
  i: Integer;
  Expr: TPasExpr;
  ResolvedEl: TPasResolverResult;
  OrigIntfType, OrigPropType, PropType: TPasType;
  IntfType: TPasClassType;
  LitEl: TJSObjectLiteralElement;
  Scope: TPas2JSClassScope;
  FunSt: TJSFunctionDeclarationStatement;
  aResolver: TPas2JSResolver;
  GetterJS: TJSElement;
  RetSt: TJSReturnStatement;
  Call: TJSCallExpression;
  FunName: String;
  FuncContext: TFunctionContext;
begin
  aResolver:=aContext.Resolver;
  GetterJS:=nil;
  FuncContext:=nil;
  try
    for i:=0 to length(Prop.Implements)-1 do
      begin
      Expr:=Prop.Implements[i];
      aResolver.ComputeElement(Expr,ResolvedEl,[rcNoImplicitProc]);
      if not (ResolvedEl.IdentEl is TPasType) then
        RaiseInconsistency(20180327183019,Expr);
      // mark interface as finished
      OrigIntfType:=TPasType(ResolvedEl.IdentEl);
      IntfType:=aResolver.ResolveAliasType(OrigIntfType) as TPasClassType;
      Scope:=IntfType.CustomData as TPas2JSClassScope;
      if Scope.GUID='' then
        RaiseInconsistency(20180327184912,Expr);
      if FinishedGUIDs.IndexOf(Scope.GUID)>=0 then
        continue;
      FinishedGUIDs.Add(Scope.GUID);

      // "guid" : function(){ return ...}
      LitEl:=ObjLit.Elements.AddElement;
      LitEl.Name:=TJSString(Scope.GUID);
      FunSt:=CreateFunctionSt(ClassEl,true,false);
      LitEl.Expr:=FunSt;
      RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,Prop));
      FunSt.AFunction.Body.A:=RetSt;

      // check property type
      OrigPropType:=aResolver.GetPasPropertyType(Prop);
      aResolver.ComputeElement(OrigPropType,ResolvedEl,[rcType]);
      if not (ResolvedEl.IdentEl is TPasType) then
        RaiseInconsistency(20180327190201,Prop);
      PropType:=aResolver.ResolveAliasType(TPasType(ResolvedEl.IdentEl));
      if not (PropType is TPasClassType) then
        RaiseInconsistency(20180327190442,Prop);

      if FuncContext<>nil then
        FreeAndNil(FuncContext);
      FuncContext:=TFunctionContext.Create(Prop,RetSt,AContext);
      FuncContext.ThisVar.Element:=ClassEl;
      FuncContext.ThisVar.Kind:=cvkInstance;

      // check property getter
      if aResolver.GetPasPropertyArgs(Prop).Count>0 then
        RaiseNotSupported(Prop,aContext,20180327191159);
      GetterJS:=CreatePropertyGet(Prop,nil,FuncContext,Prop);

      case TPasClassType(PropType).ObjKind of
      okClass:
        begin
        // delegate to class instance
        case TPasClassType(IntfType).InterfaceType of
        citCom:
          // 'guid': function(){ return rtl.queryIntfT(this.FField,IntfType); }
          // 'guid': function(){ return rtl.queryIntfT(this.GetObj(),IntfType); }
          FunName:=GetBIName(pbifnIntfQueryIntfT);
        citCorba:
          // 'guid': function(){ return rtl.getIntfT(this.FField,IntfType); }
          // 'guid': function(){ return rtl.getIntfT(this.GetObj(),IntfType); }
          FunName:=GetBIName(pbifnIntfGetIntfT);
        else
          RaiseNotSupported(Prop,aContext,20180406085319,InterfaceTypeNames[TPasClassType(IntfType).InterfaceType]);
        end;
        Call:=CreateCallExpression(Prop);
        RetSt.Expr:=Call;
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),FunName]);
        Call.AddArg(GetterJS);
        GetterJS:=nil;
        Call.AddArg(CreateReferencePathExpr(IntfType,FuncContext));
        end;
      okInterface:
        begin
        // delegate to interface
        case TPasClassType(IntfType).InterfaceType of
        citCom:
          begin
          if IsInterfaceRef(GetterJS) then
            // 'guid': function(){ return this.GetIntf(); },
            GetterJS:=RemoveIntfRef(TJSCallExpression(GetterJS),FuncContext)
          else
            begin
            // 'guid': function(){ return rtl._AddRef(this.FField); },
            Call:=CreateCallExpression(Prop);
            Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntf_AddRef)]);
            Call.AddArg(GetterJS);
            GetterJS:=Call;
            end;
          end;
        citCorba:
          begin
          // 'guid': function(){ return this.FField; },
          // 'guid': function(){ return this.GetIntf(); },
          end;
        else
          RaiseNotSupported(Prop,FuncContext,20180406085053,InterfaceTypeNames[TPasClassType(IntfType).InterfaceType]);
        end;
        RetSt.Expr:=GetterJS;
        GetterJS:=nil;
        end;
      else
        RaiseNotSupported(Prop,FuncContext,20180327190538,ObjKindNames[TPasClassType(PropType).ObjKind]);
      end;

      end;
  finally
    FuncContext.Free;
    GetterJS.Free;
  end;
end;

function TPasToJSConverter.CreateGUIDObjLit(aTGUIDRecord: TPasRecordType;
  const GUID: TGUID; PosEl: TPasElement; AContext: TConvertContext
  ): TJSObjectLiteral;
var
  i: integer;
  Members: TFPList;

  function GetMember(const aName: string): TPasElement;
  begin
    while i<Members.Count do
      begin
      Result:=TPasElement(Members[i]);
      inc(i);
      if (Result is TPasVariable) then
        if SameText(Result.Name,aName) then
          exit
        else
          RaiseInconsistency(20180415094721,PosEl);
      end;
    RaiseInconsistency(20210306223031,PosEl);
  end;

var
  PropEl: TJSObjectLiteralElement;
  MemberEl: TPasElement;
  ArrLit: TJSArrayLiteral;
begin
  Members:=aTGUIDRecord.Members;
  Result:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,PosEl));
  i:=0;

  // D1: 0x12345678
  MemberEl:=GetMember('D1');
  PropEl:=Result.Elements.AddElement;
  PropEl.Name:=TJSString(TransformElToJSName(MemberEl,AContext));
  PropEl.Expr:=CreateLiteralHexNumber(PosEl,GUID.D1,8);
  // D2: 0x1234
  MemberEl:=GetMember('D2');
  PropEl:=Result.Elements.AddElement;
  PropEl.Name:=TJSString(TransformElToJSName(MemberEl,AContext));
  PropEl.Expr:=CreateLiteralHexNumber(PosEl,GUID.D2,4);
  // D3: 0x1234
  MemberEl:=GetMember('D3');
  PropEl:=Result.Elements.AddElement;
  PropEl.Name:=TJSString(TransformElToJSName(MemberEl,AContext));
  PropEl.Expr:=CreateLiteralHexNumber(PosEl,GUID.D3,4);
  // D4: [0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12]
  MemberEl:=GetMember('D4');
  PropEl:=Result.Elements.AddElement;
  PropEl.Name:=TJSString(TransformElToJSName(MemberEl,AContext));
  ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,PosEl));
  PropEl.Expr:=ArrLit;
  for i:=0 to 7 do
    ArrLit.AddElement(CreateLiteralHexNumber(PosEl,GUID.D4[i],2));
end;

function TPasToJSConverter.CreateAssignComIntfVar(
  const LeftResolved: TPasResolverResult; var LHS, RHS: TJSElement;
  AContext: TConvertContext; PosEl: TPasElement): TJSElement;

  procedure AddProcRelease(Proc: TPasProcedure; SubEl: TPasElement);
  var
    FuncContext: TFunctionContext;
  begin
    FuncContext:=AContext.GetFuncContextOfPasElement(Proc);
    if FuncContext<>nil then
      begin
      if SubEl is TPasResultElement then
        FuncContext.ResultNeedsIntfRelease:=true
      else
        FuncContext.Add_InterfaceRelease(SubEl);
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      AContext.WriteStack;
      {$ENDIF}
      RaiseInconsistency(20180401164150,PosEl);
      end;
  end;

var
  Call: TJSCallExpression;
  AssignSt: TJSSimpleAssignStatement;
  Prim: TJSPrimaryExpressionIdent;
  IdentEl: TPasElement;
  Proc: TPasProcedure;
  ok, SkipAddRef: Boolean;
begin
  Result:=nil;
  ok:=false;
  try
    SkipAddRef:=false;
    if IsInterfaceRef(RHS) then
      begin
      // simplify: $ir.ref(id,expr)  ->  expr
      RHS:=RemoveIntfRef(TJSCallExpression(RHS),AContext);
      SkipAddRef:=true;
      end;

    Call:=CreateCallExpression(PosEl);
    Result:=Call;
    if LHS is TJSDotMemberExpression then
      begin
      // path.name = RHS  ->  rtl.setIntfP(path,"IntfVar",RHS})
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfSetIntfP)]);
      Call.AddArg(TJSDotMemberExpression(LHS).MExpr);
      TJSDotMemberExpression(LHS).MExpr:=nil;
      Call.AddArg(CreateLiteralJSString(PosEl,TJSDotMemberExpression(LHS).Name));
      FreeAndNil(LHS);
      Call.AddArg(RHS);
      RHS:=nil;
      if SkipAddRef then
        Call.AddArg(CreateLiteralBoolean(PosEl,true));
      end
    else if LHS is TJSBracketMemberExpression then
      begin
      // path[index] = RHS  ->  rtl.setIntfP(path,index,RHS})
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfSetIntfP)]);
      Call.AddArg(TJSBracketMemberExpression(LHS).MExpr);
      TJSBracketMemberExpression(LHS).MExpr:=nil;
      Call.AddArg(TJSBracketMemberExpression(LHS).Name);
      FreeAndNil(LHS);
      Call.AddArg(RHS);
      RHS:=nil;
      if SkipAddRef then
        Call.AddArg(CreateLiteralBoolean(PosEl,true));
      end
    else if LHS is TJSPrimaryExpressionIdent then
      begin
      // name = RHS  -> name = rtl.setIntfL(name,RHS)
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfSetIntfL)]);
      // add parameter name
      Prim:=TJSPrimaryExpressionIdent(CreateElement(TJSPrimaryExpressionIdent,PosEl));
      Prim.Name:=TJSPrimaryExpressionIdent(LHS).Name;
      Call.AddArg(Prim);
      // add parameter RHS
      Call.AddArg(RHS);
      RHS:=nil;
      if SkipAddRef then
        Call.AddArg(CreateLiteralBoolean(PosEl,true));
      // name = ...
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
      AssignSt.LHS:=LHS;
      LHS:=nil;
      AssignSt.Expr:=Call;
      Result:=AssignSt;
      end
    else
      RaiseNotSupported(PosEl,AContext,20180401105030,GetObjName(LHS));

    IdentEl:=LeftResolved.IdentEl;
    if (IdentEl<>nil) then
      begin
      if (IdentEl.ClassType=TPasVariable) and (IdentEl.Parent is TProcedureBody) then
        begin
        // local variable
        Proc:=TPasProcedure(IdentEl.Parent.Parent);
        AddProcRelease(Proc,IdentEl);
        end
      else if (IdentEl.ClassType=TPasArgument)
          and (IdentEl.Parent is TPasProcedureType)
          and (IdentEl.Parent.Parent is TPasProcedure) then
        begin
        // argument
        Proc:=TPasProcedure(IdentEl.Parent.Parent);
        AddProcRelease(Proc,IdentEl);
        end
      else if IdentEl.ClassType=TPasResultElement then
        begin
        // Result variable
        Proc:=TPasFunction(TPasFunctionType(IdentEl.Parent).Parent);
        AddProcRelease(Proc,IdentEl);
        end;
      end;

    ok:=true;
  finally
    if not ok then Result.Free;
  end;
end;

function TPasToJSConverter.IsInterfaceRef(Expr: TJSElement): boolean;
var
  Call: TJSCallExpression;
  DotExpr: TJSDotMemberExpression;
begin
  Result:=false;
  if Expr=nil then exit;
  if Expr.ClassType<>TJSCallExpression then exit;
  Call:=TJSCallExpression(Expr);
  if Call.Expr.ClassType<>TJSDotMemberExpression then exit;
  DotExpr:=TJSDotMemberExpression(Call.Expr);
  Result:=(DotExpr.Name=TJSString(GetBIName(pbifnIntfExprRefsAdd)))
      and (DotExpr.MExpr is TJSPrimaryExpressionIdent)
      and (TJSPrimaryExpressionIdent(DotExpr.MExpr).Name=TJSString(GetBIName(pbivnIntfExprRefs)));
end;

function TPasToJSConverter.CreateIntfRef(Expr: TJSElement;
  aContext: TConvertContext; PosEl: TPasElement): TJSCallExpression;
// enclose Expr
// ->  $ir.ref(id,Expr)
var
  FuncContext: TFunctionContext;
  Call: TJSCallExpression;
begin
  FuncContext:=aContext.GetFunctionContext;
  if FuncContext=nil then
    RaiseNotSupported(PosEl,aContext,20180402183859);
  if IsInterfaceRef(Expr) then
    exit(TJSCallExpression(Expr));
  inc(FuncContext.IntfExprReleaseCount);
  Call:=CreateCallExpression(PosEl);
  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnIntfExprRefs)+'.'+GetBIName(pbifnIntfExprRefsAdd),PosEl);
  Call.AddArg(CreateLiteralNumber(PosEl,FuncContext.IntfExprReleaseCount));
  Call.AddArg(Expr);
  Result:=Call;
end;

function TPasToJSConverter.RemoveIntfRef(Call: TJSCallExpression;
  AContext: TConvertContext): TJSElement;
var
  Lit: TJSArrayLiteralElement;
  LitValue: TJSValue;
  FuncContext: TFunctionContext;
begin
  Lit:=Call.Args.Elements[1];
  Result:=Lit.Expr;
  Lit.Expr:=nil;

  // check if $ir is still needed
  Lit:=Call.Args.Elements[0];
  if (Lit.Expr is TJSLiteral) then
    begin
    LitValue:=TJSLiteral(Lit.Expr).Value;
    FuncContext:=AContext.GetFunctionContext;
    if (FuncContext<>nil)
        and (FuncContext.IntfExprReleaseCount=LitValue.AsNumber) then
      dec(FuncContext.IntfExprReleaseCount);
    end;

  Call.Free;
end;

procedure TPasToJSConverter.CreateFunctionTryFinally(
  FuncContext: TFunctionContext);
begin
  if FuncContext.TrySt<>nil then exit;
  FuncContext.TrySt:=TJSTryFinallyStatement(CreateElement(TJSTryFinallyStatement,FuncContext.PasElement));
  FuncContext.TrySt.Block:=FuncContext.BodySt;
  FuncContext.BodySt:=FuncContext.TrySt;
end;

procedure TPasToJSConverter.AddFunctionFinallySt(NewEl: TJSElement;
  PosEl: TPasElement; FuncContext: TFunctionContext);
begin
  CreateFunctionTryFinally(FuncContext);
  AddToStatementList(FuncContext.FinallyFirst,FuncContext.FinallyLast,NewEl,PosEl);
  FuncContext.TrySt.BFinally:=FuncContext.FinallyFirst;
end;

procedure TPasToJSConverter.AddFunctionFinallyRelease(SubEl: TPasElement;
  FuncContext: TFunctionContext);
// add to finally: rtl._Release(IntfVar)
var
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(SubEl);
  AddFunctionFinallySt(Call,SubEl,FuncContext);
  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntf_Release)]);
  Call.AddArg(CreateReferencePathExpr(SubEl,FuncContext));
end;

procedure TPasToJSConverter.AddInFrontOfFunctionTry(NewEl: TJSElement;
  PosEl: TPasElement; FuncContext: TFunctionContext);
var
  St, OldSt: TJSStatementList;
begin
  CreateFunctionTryFinally(FuncContext);
  if FuncContext.BodySt=FuncContext.TrySt then
    begin
    St:=TJSStatementList(CreateElement(TJSStatementList,PosEl));
    St.A:=NewEl;
    St.B:=FuncContext.TrySt;
    FuncContext.BodySt:=St;
    end
  else if FuncContext.BodySt is TJSStatementList then
    begin
    OldSt:=TJSStatementList(FuncContext.BodySt);
    while OldSt.B is TJSStatementList do
      OldSt:=TJSStatementList(OldSt.B);
    St:=TJSStatementList(CreateElement(TJSStatementList,PosEl));
    St.A:=NewEl;
    St.B:=OldSt.B;
    OldSt.B:=St;
    end
  else
    RaiseInconsistency(20180402103144,PosEl);
end;

procedure TPasToJSConverter.AddInterfaceReleases(FuncContext: TFunctionContext;
  PosEl: TPasElement);
var
  i: Integer;
  P: TPasElement;
  Call: TJSCallExpression;
  VarSt: TJSVariableStatement;
begin
  if FuncContext.IntfExprReleaseCount>0 then
    begin
    // add in front of try..finally "var $ir = rtl.createIntfRefs();"
    Call:=CreateCallExpression(PosEl);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfExprRefsCreate)]);
    VarSt:=CreateVarStatement(GetBIName(pbivnIntfExprRefs),Call,PosEl);
    AddInFrontOfFunctionTry(VarSt,PosEl,FuncContext);
    // add in finally: "$ir.free();"
    Call:=CreateCallExpression(PosEl);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnIntfExprRefs),GetBIName(pbifnIntfExprRefsFree)]);
    AddFunctionFinallySt(Call,PosEl,FuncContext);
    end;

  if FuncContext.IntfElReleases<>nil then
    for i:=0 to FuncContext.IntfElReleases.Count-1 do
      begin
      // enclose body in try..finally and add release statement
      P:=TPasElement(FuncContext.IntfElReleases[i]);
      if P.ClassType=TPasVariable then
        AddFunctionFinallyRelease(P,FuncContext)
      else if P.ClassType=TPasArgument then
        begin
        // add in front of try..finally "rtl._AddRef(arg);"
        Call:=CreateCallExpression(P);
        AddInFrontOfFunctionTry(Call,PosEl,FuncContext);
        Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntf_AddRef)]);
        Call.AddArg(CreateReferencePathExpr(P,FuncContext));
        // add in finally: "rtl._Release(arg);"
        AddFunctionFinallyRelease(P,FuncContext);
        end
      else
        RaiseInconsistency(20180401165742,P);
      end;
end;

procedure TPasToJSConverter.AddClassSupportedInterfaces(El: TPasClassType;
  Src: TJSSourceElements; FuncContext: TFunctionContext);

  function IsClassInterfaceNeeded(aMember: TPasElement): boolean;
  var
    SpecData: TPasSpecializeTypeData;
  begin
    if aMember is TPasSpecializeType then
      begin
      SpecData:=aMember.CustomData as TPasSpecializeTypeData;
      aMember:=SpecData.SpecializedType;
      end;
    if IsElementUsed(aMember) then exit(true);
    Result:=false;
  end;

  procedure AddMapProcs(Map: TPasClassIntfMap; Call: TJSCallExpression;
    var ObjLit: TJSObjectLiteral; FuncContext: TConvertContext);
  var
    i: Integer;
    MapItem: TObject;
    Proc, IntfProc: TPasProcedure;
    ProcName, IntfProcName: String;
    Intf: TPasClassType;
    Lit: TJSObjectLiteralElement;
  begin
    Intf:=Map.Intf;
    if Map.Procs<>nil then
      for i:=0 to Map.Procs.Count-1 do
        begin
        MapItem:=TObject(Map.Procs[i]);
        if not (MapItem is TPasProcedure) then continue;
        Proc:=TPasProcedure(MapItem);
        ProcName:=TransformElToJSName(Proc,FuncContext);
        IntfProc:=TObject(Intf.Members[i]) as TPasProcedure;
        IntfProcName:=TransformElToJSName(IntfProc,FuncContext);
        if IntfProcName=ProcName then continue;
        if ObjLit=nil then
          begin
          ObjLit:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
          Call.AddArg(ObjLit);
          end;
        Lit:=ObjLit.Elements.AddElement;
        Lit.Name:=TJSString(IntfProcName);
        Lit.Expr:=CreateLiteralString(El,ProcName);
        end;
    if Map.AncestorMap<>nil then
      AddMapProcs(Map.AncestorMap,Call,ObjLit,FuncContext);
  end;

var
  Call: TJSCallExpression;
  ObjLit: TJSObjectLiteral;
  i: Integer;
  Scope, CurScope: TPas2JSClassScope;
  o: TObject;
  IntfMaps: TJSSimpleAssignStatement;
  MapsObj: TJSObjectLiteral;
  Map: TPasClassIntfMap;
  FinishedGUIDs: TStringList;
  Intf: TPasType;
  CurEl: TPasClassType;
  NeedIntfMap, HasInterfaces: Boolean;
begin
  HasInterfaces:=false;
  NeedIntfMap:=false;
  Scope:=TPas2JSClassScope(El.CustomData);
  repeat
    if Scope.Interfaces<>nil then
      begin
      for i:=0 to Scope.Interfaces.Count-1 do
        begin
        CurEl:=TPasClassType(Scope.Element);
        if not IsClassInterfaceNeeded(TPasElement(CurEl.Interfaces[i])) then continue;
        HasInterfaces:=true;
        o:=TObject(Scope.Interfaces[i]);
        if o is TPasProperty then
          // interface delegation -> needs  $intfmaps={}
          NeedIntfMap:=true;
        end;
      end;
    Scope:=TPas2JSClassScope(Scope.AncestorScope);
  until Scope=nil;
  if not HasInterfaces then exit;

  IntfMaps:=nil;
  FinishedGUIDs:=TStringList.Create;
  try
    ObjLit:=nil;
    Scope:=TPas2JSClassScope(El.CustomData);
    repeat
      if Scope.Interfaces<>nil then
        begin
        for i:=0 to Scope.Interfaces.Count-1 do
          begin
          CurEl:=TPasClassType(Scope.Element);
          if not IsClassInterfaceNeeded(TPasElement(CurEl.Interfaces[i])) then continue;
          if NeedIntfMap then
            begin
            // add "this.$intfmaps = {};"
            IntfMaps:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
            AddToSourceElements(Src,IntfMaps);
            IntfMaps.LHS:=CreatePrimitiveDotExpr('this.'+GetBIName(pbivnIntfMaps),El);
            MapsObj:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));
            IntfMaps.Expr:=MapsObj;
            NeedIntfMap:=false;
            end;

          o:=TObject(Scope.Interfaces[i]);
          if o is TPasClassIntfMap then
            begin
            // add rtl.addIntf(this,intftype,{ intfprocname: "procname", ...});
            Map:=TPasClassIntfMap(o);
            Intf:=Map.Intf;
            CurScope:=TPas2JSClassScope(Intf.CustomData);
            if FinishedGUIDs.IndexOf(CurScope.GUID)>=0 then continue;
            FinishedGUIDs.Add(CurScope.GUID);
            Call:=CreateCallExpression(El);
            AddToSourceElements(Src,Call);
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfAddMap),El);
            Call.AddArg(CreatePrimitiveDotExpr('this',El));
            Call.AddArg(CreateReferencePathExpr(Map.Intf,FuncContext));
            AddMapProcs(Map,Call,ObjLit,FuncContext);
            end
          else if o is TPasProperty then
            AddIntfDelegations(El,TPasProperty(o),FinishedGUIDs,MapsObj,FuncContext)
          else
            RaiseNotSupported(El,FuncContext,20180326234026,GetObjName(o));
          end;
        end;
      Scope:=TPas2JSClassScope(Scope.AncestorScope);
    until Scope=nil;
  finally
    FinishedGUIDs.Free;
  end;
end;

function TPasToJSConverter.CreateCallHelperMethod(Proc: TPasProcedure;
  Expr: TPasExpr; AContext: TConvertContext; Implicit: boolean
  ): TJSCallExpression;
var
  Left: TPasExpr;
  WithExprScope: TPas2JSWithExprScope;
  SelfScope: TPasProcedureScope;

  function ConvertLeftExpr: TJSElement;
  var
    OldAccess: TCtxAccess;
    Path: String;
  begin
    if WithExprScope<>nil then
      Result:=CreatePrimitiveDotExpr(WithExprScope.WithVarName,Expr)
    else if SelfScope<>nil then
      begin
      Path:=GetLocalName(SelfScope.SelfArg,cvkAll,AContext);
      Result:=CreatePrimitiveDotExpr(Path,Expr);
      end
    else if Left=nil then
      begin
      RaiseNotSupported(Expr,AContext,20190205172904);
      Result:=nil;
      end
    else
      begin
      OldAccess:=AContext.Access;
      AContext.Access:=caRead;
      Result:=ConvertExpression(Left,AContext);
      AContext.Access:=OldAccess;
      end;
  end;

  function CreateRefObj(PosEl: TPasElement; PathExpr: TJSElement;
    GetExpr, SetExpr: TJSElement; SetterArgName: string;
    const LeftResolved: TPasResolverResult): TJSObjectLiteral;

    function CreateRgCheck(aType: TPasType): TJSElement;
    begin
      Result:=CreateRangeCheckCall_TypeRange(aType,
        CreatePrimitiveDotExpr(SetterArgName,PosEl),AContext,PosEl);
    end;

  var
    Obj: TJSObjectLiteral;
    ObjLit: TJSObjectLiteralElement;
    FuncSt: TJSFunctionDeclarationStatement;
    RetSt: TJSReturnStatement;
    TypeEl: TPasType;
    RgCheck: TJSElement;
    List: TJSStatementList;
  begin
    RgCheck:=nil;
    if (SetExpr is TJSSimpleAssignStatement)
        and (SetterArgName<>'')
        and (bsRangeChecks in AContext.ScannerBoolSwitches) then
      begin
      TypeEl:=LeftResolved.LoTypeEl;
      if TypeEl<>nil then
        begin
        if LeftResolved.BaseType in btAllJSRangeCheckTypes then
          RgCheck:=CreateRgCheck(TypeEl)
        else if LeftResolved.BaseType=btContext then
          begin
          if TypeEl.ClassType=TPasEnumType then
            RgCheck:=CreateRgCheck(TypeEl);
          end
        else if LeftResolved.BaseType=btRange then
          begin
          if LeftResolved.SubType in btAllJSRangeCheckTypes then
            RgCheck:=CreateRgCheck(TypeEl)
          else if LeftResolved.SubType=btContext then
            RgCheck:=CreateRgCheck(TypeEl)
          else
            begin
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.CreateCallHelperMethod ',GetResolverResultDbg(LeftResolved));
            RaiseNotSupported(PosEl,AContext,20190220011900);
            {$ENDIF}
            end;
          end;
        end;
      end;

    Obj:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,PosEl));
    Result:=Obj;

    if PathExpr<>nil then
      begin
      // add "p:path"
      ObjLit:=Obj.Elements.AddElement;
      ObjLit.Name:=TJSString(TempRefGetPathName);
      ObjLit.Expr:=PathExpr;
      end;

    // add "get: function(){return Left}"
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TempRefObjGetterName;
    FuncSt:=CreateFunctionSt(PosEl);
    ObjLit.Expr:=FuncSt;
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,PosEl));
    FuncSt.AFunction.Body.A:=RetSt;
    RetSt.Expr:=GetExpr;

    // add "set: function(v){Left=v}"
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TempRefObjSetterName;
    FuncSt:=CreateFunctionSt(PosEl);
    ObjLit.Expr:=FuncSt;
    if SetterArgName<>'' then
      FuncSt.AFunction.Params.Add(SetterArgName);
    if RgCheck<>nil then
      begin
      List:=TJSStatementList(CreateElement(TJSStatementList,PosEl));
      List.A:=RgCheck;
      List.B:=SetExpr;
      SetExpr:=List;
      end;
    FuncSt.AFunction.Body.A:=SetExpr;
  end;

  function ConvertImplicitLeftIdentifier(PosEl: TPasElement;
    const LeftResolved: TPasResolverResult): TJSElement;
  var
    GetExpr, SetExpr, RHS: TJSElement;
    SetterArgName: string;
    AssignSt: TJSSimpleAssignStatement;
    Arg: TPasArgument;
    TypeEl: TPasType;
    IsCOMIntf: Boolean;
  begin
    // implicit Left (e.g. "with Left do proc", or "Proc")

    if LeftResolved.IdentEl is TPasArgument then
      begin
      Arg:=TPasArgument(LeftResolved.IdentEl);
      if Arg.Access in [argVar,argOut] then
        begin
        // implicit Left is already a reference
        Result:=CreatePrimitiveDotExpr(TransformArgName(Arg,AContext),PosEl);
        exit;
        end;
      end;

    // ->  {get: function(){return GetExpr},set:function(v){SetExpr}}

    // GetExpr  "ImplicitLeft"
    GetExpr:=ConvertLeftExpr;

    if rrfWritable in LeftResolved.Flags then
      begin
      // SetExpr  "ImplicitLeft = v"
      TypeEl:=LeftResolved.LoTypeEl;
      IsCOMIntf:=(TypeEl is TPasClassType)
             and (TPasClassType(TypeEl).ObjKind=okInterface)
             and (TPasClassType(TypeEl).InterfaceType=citCom);
      SetExpr:=ConvertLeftExpr;
      SetterArgName:=TempRefObjSetterArgName;
      FindAvailableLocalName(SetterArgName,SetExpr);
      RHS:=CreatePrimitiveDotExpr(SetterArgName,PosEl);
      if IsCOMIntf then
        begin
        // create   rtl.setIntfP(path,"IntfVar",v)
        SetExpr:=CreateAssignComIntfVar(LeftResolved,SetExpr,RHS,AContext,PosEl);
        end
      else
        begin
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
        AssignSt.LHS:=SetExpr;
        AssignSt.Expr:=RHS;
        SetExpr:=AssignSt;
        end;
      end
    else
      begin
      // SetExpr  rtl.raiseE("EPropReadOnly")
      SetterArgName:='';
      SetExpr:=CreateRaisePropReadOnly(PosEl);
      end;

    Result:=CreateRefObj(PosEl,nil,GetExpr,SetExpr,SetterArgName,LeftResolved);
  end;

  function CreatePropertyReference(PosEl: TPasElement;
    const LeftResolved: TPasResolverResult): TJSElement;
  var
    Prop: TPasProperty;
    OldAccess: TCtxAccess;
    GetExpr, SetExpr, LeftJS, PathExpr, RHS: TJSElement;
    DotExpr: TJSDotMemberExpression;
    AssignSt: TJSSimpleAssignStatement;
    SetterArgName, aName: String;
    TypeEl: TPasType;
    IsCOMIntf: Boolean;
  begin
    // explicit Left is property
    // path.Prop.Proc or Prop.Proc
    Prop:=TPasProperty(LeftResolved.IdentEl);

    OldAccess:=AContext.Access;
    AContext.Access:=caRead;
    LeftJS:=ConvertExpression(Left,AContext);
    AContext.Access:=OldAccess;
    {$IFDEF VerbosePas2JS}
    writeln('CreatePropertyReference LeftJS=',GetObjName(LeftJS));
    {$ENDIF}

    TypeEl:=LeftResolved.LoTypeEl;
    IsCOMIntf:=(TypeEl is TPasClassType)
           and (TPasClassType(TypeEl).ObjKind=okInterface)
           and (TPasClassType(TypeEl).InterfaceType=citCom);

    PathExpr:=nil;
    SetterArgName:='';
    if LeftJS=nil then
      DoError(20190211105946,nNoMemberIsProvidedToAccessProperty,sNoMemberIsProvidedToAccessProperty,[],PosEl)
    else if LeftJS.ClassType=TJSLiteral then
      begin
      // getter is a const
      // convert to {get:function(){return value},set:function(v){ error }}
      SetExpr:=CreateRaisePropReadOnly(PosEl);
      GetExpr:=LeftJS;
      end
    else if LeftJS.ClassType=TJSDotMemberExpression then
      begin
      // getter is a field
      // convert to {p:path,get:function(){return this.p.field},set:function(v){ this.p.field=v }}
      DotExpr:=TJSDotMemberExpression(LeftJS);
      PathExpr:=DotExpr.MExpr;
      DotExpr.MExpr:=nil;
      aName:=String(DotExpr.Name);
      DotExpr.Free;
      GetExpr:=CreateMemberExpression(['this',TempRefGetPathName,aName]);
      SetterArgName:=TempRefObjSetterArgName;
      RHS:=CreatePrimitiveDotExpr(SetterArgName,PosEl);
      if vmClass in Prop.VarModifiers then
        // assign class field -> always use class path
        SetExpr:=CreateDotExpression(PosEl,
           CreateReferencePathExpr(Prop.Parent,AContext),
           CreatePrimitiveDotExpr(aName,PosEl))
      else
        SetExpr:=CreateMemberExpression(['this',TempRefGetPathName,aName]);
      if IsCOMIntf then
        begin
        // create   rtl.setIntfP(path,"IntfVar",v)
        SetExpr:=CreateAssignComIntfVar(LeftResolved,SetExpr,RHS,AContext,PosEl);
        end
      else
        begin
        // create  SetExpr=v
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
        AssignSt.LHS:=SetExpr;
        SetExpr:=AssignSt;
        AssignSt.Expr:=RHS;
        end;
      end
    else if LeftJS.ClassType=TJSCallExpression then
      begin
      // getter is a function
      // convert to {p:FuncResult(),get:function(){return this.p},set:function(v){ this.p=v }}
      PathExpr:=TJSCallExpression(LeftJS);
      GetExpr:=CreateMemberExpression(['this',TempRefGetPathName]);
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
      AssignSt.LHS:=CreateMemberExpression(['this',TempRefGetPathName]);
      SetExpr:=AssignSt;
      SetterArgName:=TempRefObjSetterArgName;
      AssignSt.Expr:=CreatePrimitiveDotExpr(SetterArgName,PosEl);
      end
    else
      RaiseNotSupported(PosEl,AContext,20190210193605,GetObjName(LeftJS));

    Result:=CreateRefObj(PosEl,PathExpr,GetExpr,SetExpr,SetterArgName,LeftResolved);
  end;

  function CreateReference(PosEl: TPasElement;
    const LeftResolved: TPasResolverResult): TJSElement;
  var
    ProcScope: TPas2JSProcedureScope;
  begin
    if Left=nil then
      Result:=ConvertImplicitLeftIdentifier(PosEl,LeftResolved)
    else if LeftResolved.IdentEl is TPasProperty then
      Result:=CreatePropertyReference(PosEl,LeftResolved)
    else
      begin
      ProcScope:=Proc.CustomData as TPas2JSProcedureScope;
      if ProcScope.ImplProc<>nil then
        ProcScope:=ProcScope.ImplProc.CustomData as TPas2JSProcedureScope;
      if ProcScope.SelfArg=nil then
        begin
        {$IFDEF VerbosePas2JS}
        writeln('CreateReference Proc=',GetObjPath(Proc),' Left=',GetObjPath(Left),' LeftResolved=',GetResolverResultDbg(LeftResolved),' ProcScope.DeclarationProc=',GetObjPath(ProcScope.DeclarationProc));
        {$ENDIF}
        RaiseNotSupported(PosEl,AContext,20190209214906,GetObjName(Proc));
        end;
      Result:=CreateProcCallArgRef(Left,LeftResolved,ProcScope.SelfArg,AContext);
      end;
  end;

var
  aResolver: TPas2JSResolver;
  LoTypeEl: TPasType;
  Bin: TBinaryExpr;
  LeftResolved: TPasResolverResult;
  SelfJS: TJSElement;
  PosEl, NameExpr: TPasExpr;
  ProcPath: String;
  Call: TJSCallExpression;
  IdentEl: TPasElement;
  IsStatic, IsConstructorNormalCall: Boolean;
  Ref: TResolvedReference;
  ProcType: TPasProcedureType;
  ParamsExpr: TParamsExpr;
  ArgElements : TJSArrayLiteralElements;
  ArrLit: TJSArrayLiteral;
  Prop: TPasProperty;
  C: TClass;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.CreateCallHelperMethod Proc=',GetObjName(Proc),' Expr=',GetObjName(Expr),' Implicit=',Implicit);
  {$ENDIF}
  Result:=nil;
  aResolver:=AContext.Resolver;
  //Helper:=Proc.Parent as TPasClassType;
  //HelperForType:=aResolver.ResolveAliasType(Helper.HelperForType);
  IsStatic:=aResolver.MethodIsStatic(Proc);
  WithExprScope:=nil;
  SelfScope:=nil;
  PosEl:=Expr;
  Ref:=nil;
  Prop:=nil;
  Left:=nil;
  SelfJS:=nil;
  Call:=nil;
  ArgElements:=nil;
  try
    if Implicit then
      begin
      Left:=Expr;
      PosEl:=Expr;
      aResolver.ComputeElement(Left,LeftResolved,[]);
      end
    else
      begin
      NameExpr:=Expr;
      if NameExpr is TInlineSpecializeExpr then
        NameExpr:=TInlineSpecializeExpr(NameExpr).NameExpr;
      if NameExpr is TBinaryExpr then
        begin
        // e.g. "path.proc(args)" or "path.proc"
        Bin:=TBinaryExpr(NameExpr);
        if Bin.OpCode<>eopSubIdent then
          RaiseNotSupported(NameExpr,AContext,20190201163152);
        Left:=Bin.left;
        aResolver.ComputeElement(Left,LeftResolved,[]);
        PosEl:=Bin.right;
        if PosEl.CustomData is TResolvedReference then
          Ref:=TResolvedReference(PosEl.CustomData);
        end
      else if aResolver.IsNameExpr(NameExpr) then
        begin
        // e.g. "proc(args)"
        PosEl:=NameExpr;
        if not (NameExpr.CustomData is TResolvedReference) then
          RaiseNotSupported(NameExpr,AContext,20190201163210);
        Ref:=TResolvedReference(NameExpr.CustomData);
        WithExprScope:=Ref.WithExprScope as TPas2JSWithExprScope;
        if WithExprScope<>nil then
          begin
          // e.g. "with left do proc()"
          // -> Left is the WithVarName
          aResolver.ComputeElement(WithExprScope.Expr,LeftResolved,[]);
          end
        else
          begin
          // inside helper method, no explicit left expression
          if IsStatic then
            LeftResolved:=default(TPasResolverResult)
          else
            begin
            SelfScope:=aResolver.GetSelfScope(NameExpr);
            if SelfScope=nil then
              RaiseNotSupported(PosEl,AContext,20190205171529);
            if SelfScope.SelfArg=nil then
              RaiseNotSupported(PosEl,AContext,20190205171902,GetObjName(SelfScope.Element));
            aResolver.ComputeElement(SelfScope.SelfArg,LeftResolved,[]);
            end;
          end;
        end
      else if NameExpr is TParamsExpr then
        begin
        // implicit call, e.g. default property  a[]
        PosEl:=NameExpr;
        if not (NameExpr.CustomData is TResolvedReference) then
          RaiseNotSupported(NameExpr,AContext,20190208105144);
        Ref:=TResolvedReference(PosEl.CustomData);
        if Ref.Declaration.ClassType<>TPasProperty then
          RaiseNotSupported(NameExpr,AContext,20190208105222);
        Left:=TParamsExpr(NameExpr).Value;
        aResolver.ComputeElement(Left,LeftResolved,[]);
        end
      else
        begin
        RaiseNotSupported(NameExpr,AContext,20190201163210);
        LeftResolved:=default(TPasResolverResult);
        end;
      end;

    LoTypeEl:=LeftResolved.LoTypeEl;
    IdentEl:=LeftResolved.IdentEl;
    IsConstructorNormalCall:=false;
    if Ref<>nil then
      begin
      IsConstructorNormalCall:=(Proc.ClassType=TPasConstructor)
                             and not (rrfNewInstance in Ref.Flags);
      if Ref.Declaration.ClassType=TPasProperty then
        Prop:=TPasProperty(Ref.Declaration);
      end;
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.CreateCallHelperMethod IsStatic=',IsStatic,' IsConstructorNormalCall=',IsConstructorNormalCall,' Ref=',GetObjName(Ref),' Left=',GetObjName(Left),' ',GetResolverResultDbg(LeftResolved));
    {$ENDIF}

    if IsStatic then
      begin
      // call static helper method ->  HelperType.HelperCall(args?)
      if (Proc.ClassType<>TPasClassFunction)
          and (Proc.ClassType<>TPasClassProcedure) then
        RaiseNotSupported(PosEl,AContext,20190206151034,GetObjName(Proc));
      end
    else if (Proc.ClassType=TPasClassFunction) or (Proc.ClassType=TPasClassProcedure) then
      begin
      // call non static helper class method
      // Note: only allowed for class helpers because "this" must be the class type
      if LoTypeEl=nil then
        RaiseNotSupported(PosEl,AContext,20190201163453,GetResolverResultDbg(LeftResolved));
      if (IdentEl is TPasClassType) then
        begin
        // ClassType.HelperCall -> HelperType.HelperCall.call(ClassType,args?)
        if TPasClassType(LeftResolved.IdentEl).IsExternal then
          RaiseNotSupported(PosEl,AContext,20190201165636);
        SelfJS:=CreateReferencePathExpr(LeftResolved.IdentEl,AContext);
        end
      else if (LoTypeEl.ClassType=TPasClassType) and (rrfReadable in LeftResolved.Flags) then
        begin
        // ClassInstance.HelperCall -> HelperType.HelperCall.call(ClassInstance.$class,args?)
        if TPasClassType(LeftResolved.LoTypeEl).IsExternal then
          RaiseNotSupported(PosEl,AContext,20190201165656);
        SelfJS:=ConvertLeftExpr;
        SelfJS:=CreateDotExpression(PosEl,SelfJS,
                        CreatePrimitiveDotExpr(GetBIName(pbivnPtrClass),PosEl));
        end
      else if (LoTypeEl.ClassType=TPasClassOfType) and (rrfReadable in LeftResolved.Flags) then
        begin
        // ClassOfVar.HelperCall -> HelperType.HelperCall.call(ClassOfVar,args?)
        SelfJS:=ConvertLeftExpr;
        end
      else
        RaiseNotSupported(PosEl,AContext,20190201162601,GetResolverResultDbg(LeftResolved));
      end
    else if (Proc.ClassType=TPasFunction) or (Proc.ClassType=TPasProcedure)
        or IsConstructorNormalCall then
      begin
      // normal method, neither static nor class method
      if IdentEl is TPasType then
        RaiseNotSupported(PosEl,AContext,20190201170843);
      if (LoTypeEl is TPasClassType) and (rrfReadable in LeftResolved.Flags)
          and (TPasClassType(LoTypeEl).ObjKind=okClass) then
        begin
        // ClassInstance.HelperCall -> HelperType.HelperCall.call(ClassInstance,args?)
        SelfJS:=ConvertLeftExpr;
        end
      else if (LoTypeEl is TPasRecordType) and (rrfReadable in LeftResolved.Flags) then
        begin
        // RecordInstance.HelperCall -> HelperType.HelperCall.call(RecordInstance,args?)
        SelfJS:=ConvertLeftExpr;
        end
      else if IdentEl<>nil then
        begin
        C:=IdentEl.ClassType;
        if (C=TPasArgument)
            or (C=TPasVariable)
            or (C=TPasConst)
            or (C=TPasProperty)
            or (C=TPasResultElement)
            or (C=TPasEnumValue)
            or (C=TPasClassType) then
          begin
          // Left.HelperCall -> HelperType.HelperCall.call({get,set},args?)
          SelfJS:=CreateReference(PosEl,LeftResolved);
          end
        else
          RaiseNotSupported(PosEl,AContext,20190209224904,GetResolverResultDbg(LeftResolved));
        end
      else if (LeftResolved.ExprEl<>nil) and (rrfReadable in LeftResolved.Flags) then
        begin
        // LeftExpr.HelperCall -> HelperType.HelperCall.call({get,set},args?)
        SelfJS:=CreateReference(PosEl,LeftResolved);
        end
      else
        begin
        // Literal.HelperCall -> HelperType.HelperCall.call({p: Literal,get,set},args?)
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateCallHelperMethod Left=',GetObjName(Left),' LeftResolved=',GetResolverResultDbg(LeftResolved));
        {$ENDIF}
        RaiseNotSupported(PosEl,AContext,20190131211753);
        end;
      end
    else if Proc.ClassType=TPasConstructor then
      begin
      if Ref=nil then
        RaiseNotSupported(PosEl,AContext,20190206151234);
      if not (rrfNewInstance in Ref.Flags) then
        RaiseNotSupported(PosEl,AContext,20190206151901);
      // new instance
      if (LoTypeEl<>nil)
          and ((LoTypeEl.ClassType=TPasClassType)
            or (LoTypeEl.ClassType=TPasClassOfType)) then
        begin
        // aClassVarOrType.HelperCall(args)
        //  -> aClassVarOrType.$create(HelperType.HelperCall,[args])
        if (LoTypeEl.ClassType=TPasClassType) and (TPasClassType(LoTypeEl).ObjKind<>okClass) then
          RaiseNotSupported(PosEl,AContext,20190302154215,GetElementTypeName(LoTypeEl));
        Call:=CreateCallExpression(PosEl);
        SelfJS:=ConvertLeftExpr;
        Call.Expr:=CreateDotExpression(PosEl,SelfJS,
                CreatePrimitiveDotExpr(GetBIName(pbifnClassInstanceNew),PosEl));
        SelfJS:=nil;
        Call.AddArg(CreateReferencePathExpr(Proc,AContext));
        end
      else
        begin
        // record, simpletype  ->  HelperType.$new('HelperCall',[args])
        Call:=CreateCallExpression(PosEl);
        ProcPath:=CreateReferencePath(Proc.Parent,AContext,rpkPathAndName)+'.'+GetBIName(pbifnHelperNew);
        Call.Expr:=CreatePrimitiveDotExpr(ProcPath,PosEl);
        ProcPath:=TransformElToJSName(Proc,AContext);
        Call.AddArg(CreateLiteralString(PosEl,ProcPath));
        end;
      ArrLit:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,PosEl));
      Call.AddArg(ArrLit);
      ArgElements:=ArrLit.Elements;
      end
    else
      RaiseNotSupported(PosEl,AContext,20190201162609,GetObjName(Proc));

    if Call=nil then
      begin
      if (SelfJS=nil) and not IsStatic then
        RaiseNotSupported(PosEl,AContext,20190203171010,GetResolverResultDbg(LeftResolved));

      // create HelperType.HelperCall.call(SelfJS)
      Call:=CreateCallExpression(Expr);
      if (coShortRefGlobals in Options)
          and (TPas2JSProcedureScope(Proc.CustomData).SpecializedFromItem<>nil) then
        ProcPath:=CreateGlobalElPath(Proc,AContext)
      else
        ProcPath:=CreateReferencePath(Proc,AContext,rpkPathAndName);
      if not IsStatic then
        ProcPath:=ProcPath+'.call';
      Call.Expr:=CreatePrimitiveDotExpr(ProcPath,Expr);
      if SelfJS<>nil then
        begin
        Call.AddArg(SelfJS);
        SelfJS:=nil;
        end;
      ArgElements:=Call.Args.Elements;
      end;

    if Prop<>nil then
      begin
      if aResolver.GetPasPropertyArgs(Prop).Count>0 then
        begin
        // arguments are passed by ConvertParamsExpr
        Result:=Call;
        Call:=nil;
        exit;
        end;
      case AContext.Access of
      caAssign:
        begin
        // call property setter, e.g. left.prop:=RightSide
        // -> HelperType.HelperSetter.call(SelfJS,RightSide)
        // append index and RightSide
        Result:=AppendPropertyAssignArgs(Call,Prop,TAssignContext(AContext),PosEl);
        Call:=nil;
        exit;
        end;
      caRead:
        begin
        Result:=AppendPropertyReadArgs(Call,Prop,aContext,PosEl);
        Call:=nil;
        exit;
        end;
      else
        RaiseNotSupported(PosEl,AContext,20190207122708);
      end;
      end;

    // append args
    ProcType:=Proc.ProcType;
    if (Expr.Parent is TParamsExpr) and (TParamsExpr(Expr.Parent).Value=Expr) then
      ParamsExpr:=TParamsExpr(Expr.Parent)
    else
      ParamsExpr:=nil;
    CreateProcedureCallArgs(ArgElements,ParamsExpr,ProcType,AContext);

    if (ProcType is TPasFunctionType)
        and aResolver.IsInterfaceType(
          TPasFunctionType(ProcType).ResultEl.ResultType,citCom)
    then
      // need interface reference: $ir.ref(id,fnname())
      Call:=CreateIntfRef(Call,AContext,PosEl);

    Result:=Call;
    Call:=nil;
  finally
    Call.Free;
    SelfJS.Free;
  end;
end;

procedure TPasToJSConverter.AddHelperConstructor(El: TPasClassType;
  Src: TJSSourceElements; AContext: TConvertContext);
const
  FunName = 'fn';
  ArgsName = 'args';
  ValueName = 'p';
var
  aResolver: TPas2JSResolver;
  HelperForType: TPasType;
  AssignSt: TJSSimpleAssignStatement;
  Func, FuncSt: TJSFunctionDeclarationStatement;
  New_Src: TJSSourceElements;
  Call: TJSCallExpression;
  DotExpr: TJSDotMemberExpression;
  BracketExpr: TJSBracketMemberExpression;
  New_FuncContext: TFunctionContext;
  SelfJS: TJSElement;
  ReturnSt, RetSt: TJSReturnStatement;
  Obj: TJSObjectLiteral;
  ObjLit: TJSObjectLiteralElement;
  SetterArgName: Char;
begin
  if El.HelperForType=nil then exit;
  aResolver:=AContext.Resolver;
  HelperForType:=aResolver.ResolveAliasType(El.HelperForType);
  if HelperForType.ClassType=TPasClassType then
    exit; // a class helper does not need a special sub function

  New_Src:=TJSSourceElements(CreateElement(TJSSourceElements, El));
  New_FuncContext:=TFunctionContext.Create(El,New_Src,AContext);
  try
    New_FuncContext.ThisVar.Element:=El;
    New_FuncContext.ThisVar.Kind:=cvkCurType;
    New_FuncContext.IsGlobal:=true;

    // Note: a newinstance call looks like this: THelper.$new("NewHlp", [3]);
    // The $new function:
    // this.$new = function(fnname,args){
    // record:
    //   return this[fnname].apply(TRecType.$new(),args);
    // other:
    //   return this[fnname].apply({p:SelfJS,get,set},args);
    // }
    ReturnSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
    AddToSourceElements(New_Src,ReturnSt);
    Call:=CreateCallExpression(El);
    ReturnSt.Expr:=Call;
    DotExpr:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,El));
    Call.Expr:=DotExpr;
    BracketExpr:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
    DotExpr.MExpr:=BracketExpr;
    DotExpr.Name:='apply';
    BracketExpr.MExpr:=CreatePrimitiveDotExpr('this',El);
    BracketExpr.Name:=CreatePrimitiveDotExpr(FunName,El);
    SelfJS:=CreateValInit(HelperForType,nil,El,New_FuncContext);
    if HelperForType.ClassType=TPasRecordType then
      // pass new record directly
    else
      begin
      // pass new value as reference
      Obj:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));

      // add "p: SelfJS"
      ObjLit:=Obj.Elements.AddElement;
      ObjLit.Name:=TJSString(ValueName);
      ObjLit.Expr:=SelfJS;
      SelfJS:=Obj;

      // add "get: function(){return this.p}"
      ObjLit:=Obj.Elements.AddElement;
      ObjLit.Name:=TempRefObjGetterName;
      FuncSt:=CreateFunctionSt(El);
      ObjLit.Expr:=FuncSt;
      RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
      FuncSt.AFunction.Body.A:=RetSt;
      RetSt.Expr:=CreateMemberExpression(['this',ValueName]);

      // add "set: function(v){this.p=v}"
      ObjLit:=Obj.Elements.AddElement;
      ObjLit.Name:=TempRefObjSetterName;
      FuncSt:=CreateFunctionSt(El);
      ObjLit.Expr:=FuncSt;
      SetterArgName:=TempRefObjSetterArgName;
      FuncSt.AFunction.Params.Add(SetterArgName);
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      FuncSt.AFunction.Body.A:=AssignSt;
      AssignSt.LHS:=CreateMemberExpression(['this',ValueName]);
      AssignSt.Expr:=CreatePrimitiveDotExpr(SetterArgName,El);
      end;

    Call.AddArg(SelfJS);
    Call.AddArg(CreatePrimitiveDotExpr(ArgsName,El));
    // this.$new = function(fnname,args){
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
    AddToSourceElements(Src,AssignSt);
    AssignSt.LHS:=CreatePrimitiveDotExpr('this.'+GetBIName(pbifnHelperNew),El);
    Func:=CreateFunctionSt(El);
    AssignSt.Expr:=Func;
    Func.AFunction.Params.Add(FunName);
    Func.AFunction.Params.Add(ArgsName);
    Func.AFunction.Body.A:=New_Src;
    New_Src:=nil;
  finally
    New_Src.Free;
    New_FuncContext.Free;
  end;
end;

function TPasToJSConverter.ConvertImplBlock(El: TPasImplBlock;
  AContext: TConvertContext): TJSElement;
begin
  //writeln('TPasToJSConverter.ConvertImplBlock ');
  Result:=Nil;
  if (El is TPasImplStatement) then
    Result:=ConvertStatement(TPasImplStatement(El),AContext)
  else if (El.ClassType=TPasImplIfElse) then
    Result:=ConvertIfStatement(TPasImplIfElse(El),AContext)
  else if (El.ClassType=TPasImplRepeatUntil) then
    Result:=ConvertRepeatStatement(TPasImplRepeatUntil(El),AContext)
  else if (El.ClassType=TPasImplBeginBlock) then
    Result:=ConvertBeginEndStatement(TPasImplBeginBlock(El),AContext,true)
  else if (El.ClassType=TInitializationSection) then
    Result:=ConvertInitializationSection(TPasModule(El.Parent),AContext)
  else if (El.ClassType=TFinalizationSection) then
    Result:=ConvertFinalizationSection(TFinalizationSection(El),AContext)
  else if (El.ClassType=TPasImplTry) then
    Result:=ConvertTryStatement(TPasImplTry(El),AContext)
  else if (El.ClassType=TPasImplCaseOf) then
    Result:=ConvertCaseOfStatement(TPasImplCaseOf(El),AContext)
  else
    RaiseNotSupported(El,AContext,20161024192156);
end;

function TPasToJSConverter.ConvertImplCommand(El: TPasImplCommand;
  AContext: TConvertContext): TJSElement;
begin
  if El.Command<>'' then
    RaiseNotSupported(El,AContext,20181013224809,El.Command);
  if not (El.Parent is TPasImplIfElse) then
    RaiseNotSupported(El,AContext,20181013224929,GetObjName(El.Parent));
  Result:=nil;
end;

function TPasToJSConverter.ConvertPackage(El: TPasPackage;
  AContext: TConvertContext): TJSElement;

begin
  RaiseNotSupported(El,AContext,20161024192555);
  Result:=Nil;
  // ToDo TPasPackage = class(TPasElement)
end;

function TPasToJSConverter.ConvertResString(El: TPasResString;
  AContext: TConvertContext): TJSElement;

begin
  RaiseNotSupported(El,AContext,20161024192604);
  Result:=Nil;
  // ToDo: TPasResString
end;

function TPasToJSConverter.ConvertVariable(El: TPasVariable;
  AContext: TConvertContext): TJSElement;

Var
  V : TJSVarDeclaration;
  vm: TVariableModifier;
begin
  for vm in TVariableModifier do
    if (vm in El.VarModifiers) and (not (vm in [vmClass,vmExternal])) then
      RaiseNotSupported(El,AContext,20170208141622,'modifier '+VariableModifierNames[vm]);
  if El.LibraryName<>nil then
    RaiseNotSupported(El,AContext,20170208141844,'library name');
  if El.AbsoluteExpr<>nil then
    RaiseNotSupported(El,AContext,20170208141926,'absolute');

  V:=TJSVarDeclaration(CreateElement(TJSVarDeclaration,El));
  V.Name:=TransformElToJSName(El,AContext);
  V.Init:=CreateVarInit(El,AContext);
  Result:=V;
end;

function TPasToJSConverter.ConvertProperty(El: TPasProperty;
  AContext: TConvertContext): TJSElement;
begin
  Result:=Nil;
  if El.DispIDExpr<>nil then
    RaiseNotSupported(El.DispIDExpr,AContext,20170215103029,'property dispid expression');
  // does not need any declaration. Access is redirected to getter/setter.
  // RTTI is created in CreateRTTIMemberProperty
end;

function TPasToJSConverter.ConvertExportSymbol(El: TPasExportSymbol;
  AContext: TConvertContext): TJSElement;

begin
  RaiseNotSupported(El,AContext,20161024192650);
  Result:=Nil;
  // ToDo: TPasExportSymbol
end;

function TPasToJSConverter.ConvertLabels(El: TPasLabels;
  AContext: TConvertContext): TJSElement;

begin
  RaiseNotSupported(El,AContext,20161024192701);
  Result:=Nil;
  // ToDo: TPasLabels = class(TPasImplElement)
end;

function TPasToJSConverter.ConvertRaiseStatement(El: TPasImplRaise;
  AContext: TConvertContext): TJSElement;

Var
  E : TJSElement;
  T : TJSThrowStatement;

begin
  if El.ExceptObject<>Nil then
    E:=ConvertExpression(El.ExceptObject,AContext)
  else
    E:=CreatePrimitiveDotExpr(GetBIName(pbivnExceptObject),El);
  T:=TJSThrowStatement(CreateElement(TJSThrowStatement,El));
  T.A:=E;
  Result:=T;
end;

function TPasToJSConverter.ConvertAssignStatement(El: TPasImplAssign;
  AContext: TConvertContext): TJSElement;
var
  lRightIsTemp, lRightIsTempValid: boolean;
  lLeftIsConstSetter, lLeftIsConstSetterValid: boolean;

  procedure NotSupported(AssignContext: TAssignContext; id: TMaxPrecInt);
  begin
    {$IFDEF VerbosePas2JS}
    writeln('NotSupported Left=',GetResolverResultDbg(AssignContext.LeftResolved),
      '  Op=',AssignKindNames[El.Kind],
      '  Right=',GetResolverResultDbg(AssignContext.RightResolved));
    {$ENDIF}
    RaiseNotSupported(El,AContext,id,
      GetResolverResultDbg(AssignContext.LeftResolved)+AssignKindNames[El.Kind]
        +GetResolverResultDbg(AssignContext.RightResolved));
  end;

  function RightIsTemporaryVar: boolean;
  // returns true if right side is a temporary variable, e.g. a function result
  begin
    if not lRightIsTempValid then
      begin
      lRightIsTempValid:=true;
      lRightIsTemp:=IsExprTemporaryVar(El.right);
      end;
    Result:=lRightIsTemp;
  end;

  function LeftIsConstSetter: boolean;
  // returns true if left side is a property setter with const argument
  begin
    if not lLeftIsConstSetterValid then
      begin
      lLeftIsConstSetterValid:=true;
      lLeftIsConstSetter:=IsExprPropertySetterConst(El.left,AContext);
      end;
    Result:=lLeftIsConstSetter
  end;

  function CreateRangeCheck(AssignSt: TJSElement;
    MinVal, MaxVal: TMaxPrecInt; RTLFunc: TPas2JSBuiltInName): TJSElement;
  var
    Call: TJSCallExpression;
  begin
    Call:=CreateCallExpression(El);
    Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(RTLFunc),El);
    if AssignSt.ClassType=TJSSimpleAssignStatement then
      begin
      // LHS:=rtl.rc(RHS,min,max)    check before assign
      Result:=AssignSt;
      Call.AddArg(TJSSimpleAssignStatement(AssignSt).Expr);
      TJSSimpleAssignStatement(AssignSt).Expr:=Call;
      end
    else
      begin
      // rtl.rc(LHS+=RHS,min,max)    check after assign
      Call.AddArg(AssignSt);
      Result:=Call;
      end;
    Call.AddArg(CreateLiteralNumber(El.right,MinVal));
    Call.AddArg(CreateLiteralNumber(El.right,MaxVal));
  end;

  function ApplyRangeCheck_Type(AssignSt: TJSElement; aType: TPasType): TJSElement;
  var
    Value: TResEvalValue;
  begin
    Result:=AssignSt;
    Value:=AContext.Resolver.EvalTypeRange(aType,[refConst]);
    if Value=nil then
      RaiseNotSupported(El,AContext,20180424110758,'range checking '+GetObjName(aType));
    try
      case Value.Kind of
      revkRangeInt:
        case TResEvalRangeInt(Value).ElKind of
          revskEnum, revskInt:
            Result:=CreateRangeCheck(AssignSt,TResEvalRangeInt(Value).RangeStart,
              TResEvalRangeInt(Value).RangeEnd,pbifnRangeCheckInt);
          revskChar:
            Result:=CreateRangeCheck(AssignSt,TResEvalRangeInt(Value).RangeStart,
              TResEvalRangeInt(Value).RangeEnd,pbifnRangeCheckChar);
          revskBool: ; // maybe check for type?
        else
          RaiseNotSupported(El,AContext,20190220003746,'range checking '+Value.AsDebugString);
        end;
      else
        RaiseNotSupported(El,AContext,20180424111037,'range checking '+Value.AsDebugString);
      end;
    finally
      ReleaseEvalValue(Value);
    end;
  end;

Var
  LHS: TJSElement;
  T: TJSAssignStatement;
  AssignContext: TAssignContext;
  Flags: TPasResolverComputeFlags;
  LeftIsProcType: Boolean;
  Call: TJSCallExpression;
  MinVal, MaxVal: TMaxPrecInt;
  LeftTypeEl, RightTypeEl: TPasType;
  aResolver: TPas2JSResolver;
  ObjLit: TJSObjectLiteral;
  GUID: TGUID;
begin
  Result:=nil;
  LHS:=nil;
  aResolver:=AContext.Resolver;
  lLeftIsConstSetterValid:=false;
  lRightIsTempValid:=false;
  AssignContext:=TAssignContext.Create(El,nil,AContext);
  try
    if aResolver<>nil then
      begin
      aResolver.ComputeElement(El.left,AssignContext.LeftResolved,[rcNoImplicitProc]);
      Flags:=[];
      LeftIsProcType:=aResolver.IsProcedureType(AssignContext.LeftResolved,false);
      if LeftIsProcType then
        begin
        if msDelphi in AContext.CurrentModeSwitches then
          Include(Flags,rcNoImplicitProc)
        else
          Include(Flags,rcNoImplicitProcType);
        end;
      aResolver.ComputeElement(El.right,AssignContext.RightResolved,Flags);
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.ConvertAssignStatement Left={',GetResolverResultDbg(AssignContext.LeftResolved),'} Right={',GetResolverResultDbg(AssignContext.RightResolved),'}');
      {$ENDIF}
      if LeftIsProcType and (msDelphi in AContext.CurrentModeSwitches)
          and (AssignContext.RightResolved.BaseType=btProc)
          and (AssignContext.RightResolved.IdentEl is TPasProcedure) then
        begin
        // Delphi allows assigning a proc without @: proctype:=proc
        LeftTypeEl:=AssignContext.LeftResolved.LoTypeEl;
        AssignContext.RightSide:=CreateCallback(El.right,
             AssignContext.RightResolved,
             TPasProcedureType(LeftTypeEl).CallingConvention=ccSafeCall,
             AContext);
        end
      else if AssignContext.RightResolved.BaseType=btNil then
        begin
        if aResolver.IsArrayType(AssignContext.LeftResolved) then
          begin
          // array:=nil -> array:=[]
          AssignContext.RightSide:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El.right));
          end;
        end
      else if AssignContext.LeftResolved.BaseType=btContext then
        begin
        LeftTypeEl:=AssignContext.LeftResolved.LoTypeEl;
        if (LeftTypeEl.ClassType=TPasRecordType)
            and (AssignContext.RightResolved.BaseType in btAllStrings) then
          begin
          if aResolver.GetAssignGUIDString(TPasRecordType(LeftTypeEl),El.right,GUID) then
            begin
            // guidvar:='{...}';  ->  convert string to GUID object { D1:x12345678, D2:0x1234,...}
            // Note: the "guidvar.$assign()" is done by left side
            ObjLit:=CreateGUIDObjLit(TPasRecordType(LeftTypeEl),GUID,El,AContext);
            AssignContext.RightSide:=ObjLit;
            end
          else
            RaiseNotSupported(El,AContext,20180415101516);
          end;
        if (LeftTypeEl.ClassType=TPasArrayType) and (El.Kind<>akDefault) then
          aResolver.RaiseMsg(20201028212754,nIllegalQualifier,sIllegalQualifier,[AssignKindNames[El.Kind]],El);
        end;
      end;
    if AssignContext.RightSide=nil then
      AssignContext.RightSide:=ConvertExpression(El.right,AContext);
    if (AssignContext.RightResolved.BaseType in [btSet,btArrayOrSet])
        and (AssignContext.RightResolved.IdentEl<>nil) then
      begin
      // right side is a set variable -> create reference
      {$IFDEF VerbosePas2JS}
      //writeln('TPasToJSConverter.ConvertAssignStatement SET variable Right={',GetResolverResultDbg(AssignContext.RightResolved),'} AssignContext.RightResolved.IdentEl=',GetObjName(AssignContext.RightResolved.IdentEl));
      {$ENDIF}
      // create  rtl.refSet(right)
      AssignContext.RightSide:=CreateReferencedSet(El.right,AssignContext.RightSide);
      end
    else if AssignContext.LeftResolved.BaseType=btCurrency then
      begin
      if AssignContext.RightResolved.BaseType=btCurrency then
        // currency := currency
      else if AssignContext.RightResolved.BaseType in btAllJSFloats then
        begin
        // currency := double  ->  currency := rtl.trunc(double*10000)
        AssignContext.RightSide:=CreateMulNumber(El,AssignContext.RightSide,10000);
        AssignContext.RightSide:=CreateTruncFloor(El,AssignContext.RightSide,true);
        end
      else if AssignContext.RightResolved.BaseType in btAllJSInteger then
        begin
        // currency := integer  ->  currency := double*10000
        AssignContext.RightSide:=CreateMulNumber(El,AssignContext.RightSide,10000);
        end
      else
        RaiseNotSupported(El,AContext,20181016094542,GetResolverResultDbg(AssignContext.RightResolved));
      end
    else if AssignContext.RightResolved.BaseType=btCurrency then
      begin
      // noncurrency := currency
      // e.g. double := currency  ->  double := currency/10000
      AssignContext.RightSide:=CreateDivideNumber(El,AssignContext.RightSide,10000);
      end
    else if AssignContext.RightResolved.BaseType in btAllStringAndChars then
      begin
      if AssignContext.LeftResolved.BaseType=btContext then
        begin
        if AssignContext.LeftResolved.LoTypeEl is TPasArrayType then
          begin
          // AnArray:=aString  -> AnArray:=aString.split("")
          AssignContext.RightSide:=CreateDotSplit(El.right,AssignContext.RightSide);
          end;
        end;
      end
    else if AssignContext.RightResolved.BaseType=btContext then
      begin
      RightTypeEl:=AssignContext.RightResolved.LoTypeEl;
      if RightTypeEl.ClassType=TPasArrayType then
        begin
        if length(TPasArrayType(RightTypeEl).Ranges)>0 then
          begin
          // right side is a static array -> clone
          if (not RightIsTemporaryVar)
              and (not LeftIsConstSetter) then
            begin
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.ConvertAssignStatement STATIC ARRAY variable Right={',GetResolverResultDbg(AssignContext.RightResolved),'} AssignContext.RightResolved.IdentEl=',GetObjName(AssignContext.RightResolved.IdentEl));
            {$ENDIF}
            AssignContext.RightSide:=CreateCloneStaticArray(El.right,
                   TPasArrayType(RightTypeEl),AssignContext.RightSide,AContext);
            end;
          end
        else if RightTypeEl.Parent.ClassType=TPasArgument then
         // right side is open array
        else
          begin
          // right side is dynamic array
          if (AssignContext.LeftResolved.BaseType=btContext)
              and (AssignContext.LeftResolved.LoTypeEl is TPasArrayType) then
            begin
            if El.Kind<>akDefault then
              aResolver.RaiseMsg(20201028213335,nIllegalQualifier,sIllegalQualifier,[AssignKindNames[El.Kind]],El);
            if (not RightIsTemporaryVar)
                and (not LeftIsConstSetter) then
              begin
              // DynArrayA := DynArrayB  ->  DynArrayA = rtl.arrayRef(DynArrayB)
              AssignContext.RightSide:=CreateArrayRef(El.right,AssignContext.RightSide);
              end;
            end;
          end;
        end
      else if RightTypeEl.ClassType=TPasClassType then
        begin
        if AssignContext.LeftResolved.BaseType in btAllStrings then
          begin
          if TPasClassType(RightTypeEl).ObjKind=okInterface then
            begin
            // aString:=IntfTypeOrVar  ->  intfTypeOrVar.$guid
            AssignContext.RightSide:=CreateDotNameExpr(El,
              AssignContext.RightSide,TJSString(GetBIName(pbivnIntfGUID)));
            end;
          end
        else if AssignContext.LeftResolved.BaseType=btContext then
          begin
          LeftTypeEl:=AssignContext.LeftResolved.LoTypeEl;
          if LeftTypeEl.ClassType=TPasRecordType then
            begin
            if (TPasClassType(RightTypeEl).ObjKind=okInterface)
                and SameText(LeftTypeEl.Name,'TGUID') then
              begin
              // GUIDRecord:=IntfTypeOrVar  ->  rtl.getIntfGUIDR(IntfTypeOrVar)
              // Note: the GUIDRecord.$assign() is created by the left side
              Call:=CreateCallExpression(El);
              Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGetGUIDR),El);
              Call.AddArg(AssignContext.RightSide);
              AssignContext.RightSide:=Call;
              end
            else
              RaiseNotSupported(El,AContext,20180413194856);
            end
          else if LeftTypeEl.ClassType=TPasClassType then
            case TPasClassType(LeftTypeEl).ObjKind of
            okClass:
              case TPasClassType(RightTypeEl).ObjKind of
              okClass: ; // ClassInstVar:=ClassInstVar
              else
                NotSupported(AssignContext,20180327202735);
              end;
            okInterface:
              case TPasClassType(RightTypeEl).ObjKind of
              okClass:
                begin
                // IntfVar:=ClassInstVar
                if TPasClassType(RightTypeEl).IsExternal then
                  RaiseNotSupported(El.right,AContext,20180327210004,'external class instance');
                if AssignContext.LeftResolved.LoTypeEl=nil then
                  RaiseNotSupported(El.right,AContext,20180327204021);
                Call:=CreateCallExpression(El.right);
                case TPasClassType(LeftTypeEl).InterfaceType of
                // COM: $ir.ref(id,rtl.queryIntfT(ClassInstVar,IntfVarType))
                citCom:
                  begin
                  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfQueryIntfT)]);
                  Call.AddArg(AssignContext.RightSide);
                  AssignContext.RightSide:=Call;
                  Call.AddArg(CreateReferencePathExpr(AssignContext.LeftResolved.LoTypeEl,
                    AContext));
                  Call:=CreateIntfRef(Call,AContext,El);
                  AssignContext.RightSide:=Call;
                  end;
                // CORBA: rtl.getIntfT(ClassInstVar,IntfVarType)
                citCorba:
                  begin
                  Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfGetIntfT)]);
                  Call.AddArg(AssignContext.RightSide);
                  AssignContext.RightSide:=Call;
                  Call.AddArg(CreateReferencePathExpr(AssignContext.LeftResolved.LoTypeEl,
                    AContext));
                  end;
                else RaiseNotSupported(El,AContext,20180401225931,InterfaceTypeNames[TPasClassType(RightTypeEl).InterfaceType]);
                end;
                end;
              okInterface: ;// IntfVar:=IntfVar
              else
                NotSupported(AssignContext,20180327203326);
              end;
            else
              NotSupported(AssignContext,20180327203334);
            end;
          end;
        end
      else if RightTypeEl.ClassType=TPasRecordType then
        begin
        // right side is a record
        if AssignContext.LeftResolved.BaseType in btAllStrings then
          begin
          if aResolver.IsTGUID(TPasRecordType(RightTypeEl)) then
            begin
            // aString:=GUIDVar  ->  rtl.guidrToStr(GUIDVar)
            Call:=CreateCallExpression(El);
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGuidRToStr),El);
            Call.AddArg(AssignContext.RightSide);
            AssignContext.RightSide:=Call;
            end;
          end;
        end
      else if RightTypeEl is TPasProcedureType then
        begin
        LeftTypeEl:=AssignContext.LeftResolved.LoTypeEl;
        if (LeftTypeEl is TPasProcedureType)
            and (TPasProcedureType(AssignContext.LeftResolved.LoTypeEl).CallingConvention=ccSafeCall)
            and (El.right is TUnaryExpr)
            and (TUnaryExpr(El.right).OpCode=eopAddress) then
          begin
          // aSafeCall:=@Proc
          AssignContext.RightSide:=CreateSafeCallback(El.right,AssignContext.RightSide,AContext);
          end;
        end;
      end;
    // convert left side
    LHS:=ConvertExpression(El.left,AssignContext);

    if AssignContext.Call<>nil then
      begin
      // left side is a Setter -> RightSide was already inserted as parameter
      if AssignContext.RightSide<>nil then
        RaiseInconsistency(20170207215544,El.left);
      Result:=LHS;
      end
    else
      begin
      // left side is a variable
      if AssignContext.RightSide=nil then
        RaiseInconsistency(20180622211919,El);

      LeftTypeEl:=AssignContext.LeftResolved.LoTypeEl;
      if AssignContext.LeftResolved.BaseType=btContext then
        begin
        if (LeftTypeEl is TPasClassType)
            and (TPasClassType(LeftTypeEl).ObjKind=okInterface)
            and (TPasClassType(LeftTypeEl).InterfaceType=citCom) then
          begin
          // left side is a COM interface variable
          Result:=CreateAssignComIntfVar(AssignContext.LeftResolved,
                                  LHS,AssignContext.RightSide,AssignContext,El);
          if Result<>nil then exit;
          end;
        end;

      // create normal assign statement
      case El.Kind of
        akDefault: T:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
        akAdd: T:=TJSAddEqAssignStatement(CreateElement(TJSAddEqAssignStatement,El));
        akMinus: T:=TJSSubEqAssignStatement(CreateElement(TJSSubEqAssignStatement,El));
        akMul: T:=TJSMulEqAssignStatement(CreateElement(TJSMulEqAssignStatement,El));
        akDivision: T:=TJSDivEqAssignStatement(CreateElement(TJSDivEqAssignStatement,El));
        else RaiseNotSupported(El,AContext,20161107221807);
      end;
      T.Expr:=AssignContext.RightSide;
      AssignContext.RightSide:=nil;
      T.LHS:=LHS;
      Result:=T;
      LHS:=nil;

      if (bsRangeChecks in AContext.ScannerBoolSwitches)
          and not (T.Expr is TJSLiteral) then
        begin
        if AssignContext.LeftResolved.BaseType in btAllJSInteger then
          begin
          if LeftTypeEl is TPasUnresolvedSymbolRef then
            begin
            if not aResolver.GetIntegerRange(AssignContext.LeftResolved.BaseType,MinVal,MaxVal) then
              RaiseNotSupported(El.left,AContext,20180119154120);
            Result:=CreateRangeCheck(Result,MinVal,MaxVal,pbifnRangeCheckInt);
            end
          else if LeftTypeEl.ClassType=TPasRangeType then
            Result:=ApplyRangeCheck_Type(Result,LeftTypeEl);
          end
        else if AssignContext.LeftResolved.BaseType in btAllJSChars then
          Result:=ApplyRangeCheck_Type(Result,LeftTypeEl)
        else if AssignContext.LeftResolved.BaseType=btContext then
          begin
          if LeftTypeEl.ClassType=TPasEnumType then
            Result:=ApplyRangeCheck_Type(Result,LeftTypeEl);
          end
        else if AssignContext.LeftResolved.BaseType=btRange then
          begin
          if AssignContext.LeftResolved.SubType in btAllJSRangeCheckTypes then
            Result:=ApplyRangeCheck_Type(Result,LeftTypeEl)
          else if AssignContext.LeftResolved.SubType=btContext then
            Result:=ApplyRangeCheck_Type(Result,LeftTypeEl)
          else
            begin
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.ConvertAssignStatement ',GetResolverResultDbg(AssignContext.LeftResolved));
            RaiseNotSupported(El,AContext,20180424121201);
            {$ENDIF}
            end;
          end;
        end;
      end;
  finally
    if Result=nil then
      LHS.Free;
    AssignContext.RightSide.Free;
    AssignContext.Free;
  end;
end;

function TPasToJSConverter.ConvertIfStatement(El: TPasImplIfElse;
  AContext: TConvertContext): TJSElement;
Var
  C, BThen, BElse: TJSElement;
  T: TJSIfStatement;
begin
  Result:=nil;
  if AContext=nil then ;
  C:=Nil;
  BThen:=Nil;
  BElse:=Nil;
  try
    C:=ConvertExpression(El.ConditionExpr,AContext);
    if Assigned(El.IfBranch) then
      BThen:=ConvertElement(El.IfBranch,AContext);
    if Assigned(El.ElseBranch) then
      BElse:=ConvertElement(El.ElseBranch,AContext);
    T:=TJSIfStatement(CreateElement(TJSIfStatement,El));
    T.Cond:=C;
    T.BTrue:=BThen;
    T.BFalse:=BElse;
    Result:=T;
  finally
    if Result=nil then
      begin
      FreeAndNil(C);
      FreeAndNil(BThen);
      FreeAndNil(BElse);
      end;
  end;
end;

function TPasToJSConverter.ConvertWhileStatement(El: TPasImplWhileDo;
  AContext: TConvertContext): TJSElement;

Var
  C : TJSElement;
  B : TJSElement;
  W : TJSWhileStatement;
  ok: Boolean;
begin
  Result:=Nil;
  C:=Nil;
  B:=Nil;
  ok:=false;
  try
    C:=ConvertExpression(El.ConditionExpr,AContext);
    if Assigned(El.Body) then
      B:=ConvertElement(El.Body,AContext)
    else
      B:=TJSEmptyBlockStatement(CreateElement(TJSEmptyBlockStatement,El));
    ok:=true;
  finally
    if not ok then
      begin
      FreeAndNil(B);
      FreeAndNil(C);
      end;
  end;
  W:=TJSWhileStatement(CreateElement(TJSWhileStatement,El));
  W.Cond:=C;
  W.Body:=B;
  Result:=W;
end;

function TPasToJSConverter.ConvertRepeatStatement(El: TPasImplRepeatUntil;
  AContext: TConvertContext): TJSElement;
// do{implblock}while(!untilcondition);
var
  C : TJSElement;
  W : TJSDoWhileStatement;
  B : TJSElement;
begin
  Result:=Nil;
  C:=Nil;
  B:=Nil;
  try
    C:=ConvertExpression(El.ConditionExpr,AContext);
    if C is TJSUnaryNotExpression then
      begin
      // Note: do..while(condition) checks for truthiness, same as the ! operator
      // therefore  do..while(!!expr)  is the same as  do..while(expr)
      B:=C;
      C:=TJSUnaryNotExpression(B).A;
      TJSUnaryNotExpression(B).A:=nil;
      B.Free;
      B:=nil;
      end
    else
      C:=CreateUnaryNot(C,El.ConditionExpr);
    B:=ConvertImplBlockElements(El,AContext,false);
    W:=TJSDoWhileStatement(CreateElement(TJSDoWhileStatement,El));
    W.Cond:=C;
    W.Body:=B;
    Result:=W;
  finally
    if Result=nil then
      begin
      FreeAndNil(B);
      FreeAndNil(C);
      end;
  end;
end;

function TPasToJSConverter.ConvertForStatement(El: TPasImplForLoop;
  AContext: TConvertContext): TJSElement;
// Creates the following code:
//  for (var $loop1 = <startexpr>, $loopend = <endexpr>; $loop<=$loopend; $loop++){
//    VariableName = $loop;
//    ...Body...
//  }
//
// For compatibility:
//  LoopVar can be a varname or programname.varname
//  The StartExpr must be executed exactly once at beginning.
//  The EndExpr must be executed exactly once at beginning.
//  If the loop is not executed the Variable is not set, aka keeps its old value.
//  After the loop the variable has the last value.
type
  TInKind = (
    ikNone,
    ikEnum,
    ikBool,
    ikChar,
    ikString,
    ikArray,
    ikSetInt,
    ikSetBool,
    ikSetChar,
    ikSetString
  );
var
  aResolver: TPas2JSResolver;

  function ConvExpr(Expr: TPasExpr): TJSElement; overload;
  var
    ResolvedEl: TPasResolverResult;
  begin
    Result:=ConvertExpression(Expr,AContext);
    if Result is TJSLiteral then
      case TJSLiteral(Result).Value.ValueType of
      jstBoolean:
        // convert bool literal to int
        TJSLiteral(Result).Value.AsNumber:=ord(TJSLiteral(Result).Value.AsBoolean);
      jstNumber:
        exit;
      jstString:
        begin
        // convert char literal to int
        ConvertCharLiteralToInt(TJSLiteral(Result),Expr,AContext);
        exit;
        end;
      else
        Result.Free;
        RaiseNotSupported(Expr,AContext,20171112021222);
      end
    else if aResolver<>nil then
      begin
      aResolver.ComputeElement(Expr,ResolvedEl,[]);
      if (ResolvedEl.BaseType in btAllChars)
          or ((ResolvedEl.BaseType=btRange) and (ResolvedEl.SubType in btAllChars)) then
        begin
        // convert char variable to int: append  .charCodeAt()
        Result:=CreateCallCharCodeAt(Result,0,Expr);
        end
      else if (ResolvedEl.BaseType in btAllJSBooleans)
          or ((ResolvedEl.BaseType=btRange) and (ResolvedEl.SubType in btAllJSBooleans)) then
        begin
        // convert bool variable to int: +expr
        Result:=CreateUnaryPlus(Result,Expr);
        end;
      end;
  end;

  function GetOrd(Value: TResEvalValue; ErrorEl: TPasElement): TMaxPrecInt; overload;
  var
    OrdValue: TResEvalValue;
  begin
    if Value=nil then
      exit(0);
    OrdValue:=aResolver.ExprEvaluator.OrdValue(Value,ErrorEl);
    case OrdValue.Kind of
    revkInt: Result:=TResEvalInt(OrdValue).Int;
    else
      RaiseNotSupported(ErrorEl,AContext,20171112133917);
    end;
    if Value<>OrdValue then
      ReleaseEvalValue(OrdValue);
  end;

  function GetEnumValue(EnumType: TPasEnumType; Int: TMaxPrecInt): TResEvalValue; overload;
  begin
    if (coEnumNumbers in Options) or (Int<0) or (Int>=EnumType.Values.Count) then
      Result:=TResEvalInt.CreateValue(Int)
    else
      Result:=TResEvalEnum.CreateValue(Int,TObject(EnumType.Values[Int]) as TPasEnumValue);
  end;

var
  FuncContext: TFunctionContext;
  VarResolved, InResolved: TPasResolverResult;
  StartValue, EndValue, InValue: TResEvalValue;
  StartInt, EndInt: TMaxPrecInt;
  HasLoopVar, HasEndVar, HasInVar: Boolean;
  InKind: TInKind;
  ForScope: TPasForLoopScope;

  function InitWithResolver: boolean;
  var
    EnumType: TPasEnumType;
    TypeEl: TPasType;
    ArgResolved, LengthResolved, PropResultResolved: TPasResolverResult;
  begin
    Result:=true;
    aResolver.ComputeElement(El.VariableName,VarResolved,[rcNoImplicitProc]);
    if (not (VarResolved.IdentEl is TPasVariable))
        and not (VarResolved.IdentEl is TPasResultElement) then
      DoError(20170213214404,nXExpectedButYFound,sXExpectedButYFound,['var',
        aResolver.GetResolverResultDescription(VarResolved)],El.VariableName);

    case El.LoopType of
    ltNormal,ltDown:
      begin
      StartValue:=aResolver.Eval(El.StartExpr,[],false);
      StartInt:=GetOrd(StartValue,El.StartExpr);
      EndValue:=aResolver.Eval(El.EndExpr,[],false);
      EndInt:=GetOrd(EndValue,El.EndExpr);
      end;
    ltIn:
      begin
      if ForScope.GetEnumerator<>nil then
        begin
        ConvertForStatement:=CreateGetEnumeratorLoop(El,AContext);
        exit(false);
        end;

      aResolver.ComputeElement(El.StartExpr,InResolved,[]);
      HasInVar:=true;
      InValue:=aResolver.Eval(El.StartExpr,[],false);
      if InValue=nil then
        begin
        if InResolved.IdentEl is TPasType then
          begin
          TypeEl:=aResolver.ResolveAliasType(TPasType(InResolved.IdentEl));
          if TypeEl is TPasArrayType then
            begin
            if length(TPasArrayType(TypeEl).Ranges)=1 then
              InValue:=aResolver.Eval(TPasArrayType(TypeEl).Ranges[0],[refConst]);
            end
          else if TypeEl is TPasSetType then
            InValue:=aResolver.EvalTypeRange(TPasSetType(TypeEl).EnumType,[refConst]);
          end;
        end;
      if InValue<>nil then
        begin
        // for <var> in <constant> do
        case InValue.Kind of
        {$IFDEF FPC_HAS_CPSTRING}
        revkString,
        {$ENDIF}
        revkUnicodeString:
          begin
          // example:
          //  for c in 'foo' do ;
          // -> for (var $l1 = 0, $li2 = 'foo'; $l1<=2; $l1++) c = $li2.charAt($l1);
          InKind:=ikString;
          StartInt:=0;
          {$IFDEF FPC_HAS_CPSTRING}
          if InValue.Kind=revkString then
            EndInt:=TMaxPrecInt(length(UTF8Decode(TResEvalString(InValue).S)))-1
          else
          {$ENDIF}
            EndInt:=TMaxPrecInt(length(TResEvalUTF16(InValue).S))-1;
          ReleaseEvalValue(InValue);
          end;
        revkRangeInt,revkSetOfInt:
          begin
          if InValue.Kind=revkSetOfInt then
            begin
            if length(TResEvalSet(InValue).Ranges)=0 then
              exit(false);
            if length(TResEvalSet(InValue).Ranges)>1 then
              begin
              // set, non continuous range
              case TResEvalSet(InValue).ElKind of
              revskEnum,revskInt: InKind:=ikSetInt;
              revskChar: InKind:=ikSetChar;
              revskBool: InKind:=ikSetBool;
              end;
              HasInVar:=false;
              HasLoopVar:=InKind<>ikSetInt;
              HasEndVar:=false;
              exit;
              end;
            end;
          StartInt:=TResEvalRangeInt(InValue).RangeStart;
          EndInt:=TResEvalRangeInt(InValue).RangeEnd;
          HasInVar:=false;
          HasEndVar:=false;
          case TResEvalRangeInt(InValue).ElKind of
          revskEnum:
            if coEnumNumbers in Options then
              InKind:=ikNone
            else
              begin
              InKind:=ikEnum;
              EnumType:=TPasEnumType(TResEvalRangeInt(InValue).ElType);
              StartValue:=GetEnumValue(EnumType,StartInt);
              EndValue:=GetEnumValue(EnumType,EndInt);
              end;
          revskInt:
            InKind:=ikNone;
          revskChar:
            InKind:=ikChar;
          revskBool:
            InKind:=ikBool;
          else
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.ConvertForStatement ',GetObjName(El.StartExpr),' InValue=',InValue.AsDebugString);
            {$ENDIF}
            RaiseNotSupported(El.StartExpr,AContext,20171113023419);
          end;
          end
        else
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.ConvertForStatement ',GetObjName(El.StartExpr),' InValue=',InValue.AsDebugString);
          {$ENDIF}
          RaiseNotSupported(El.StartExpr,AContext,20171112161527);
        end;
        end
      else if rrfReadable in InResolved.Flags then
        begin
        // for v in <variable> do
        if InResolved.BaseType in btAllStrings then
          begin
          InKind:=ikString;
          StartInt:=0;
          end
        else if InResolved.BaseType=btCustom then
          begin
          if aResolver.IsJSBaseType(InResolved,pbtJSValue) then
            begin
            // for v in jsvalue do
            InKind:=ikSetString;
            HasInVar:=false;
            HasLoopVar:=false;
            HasEndVar:=false;
            exit;
            end;
          end
        else if InResolved.BaseType=btContext then
          begin
          TypeEl:=InResolved.LoTypeEl;
          if TypeEl.ClassType=TPasArrayType then
            begin
            if length(TPasArrayType(TypeEl).Ranges)<=1 then
              begin
              InKind:=ikArray;
              StartInt:=0;
              end
            else
              begin
              {$IFDEF VerbosePas2JS}
              writeln('TPasToJSConverter.ConvertForStatement.InitWithResolver ResolvedIn=',GetResolverResultDbg(InResolved),' length(Ranges)=',length(TPasArrayType(TypeEl).Ranges));
              {$ENDIF}
              RaiseNotSupported(El.StartExpr,AContext,20171220010147);
              end;
            end
          else if (TypeEl.ClassType=TPasClassType) and TPasClassType(TypeEl).IsExternal then
            begin
            if aResolver.IsForInExtArray(El,VarResolved,InResolved,
                ArgResolved,LengthResolved,PropResultResolved) then
              begin
              // for v in JSArray do
              InKind:=ikArray;
              StartInt:=0;
              end
            else
              begin
              // for v in jsobject do  ->  for(v in jsobject){ }
              InKind:=ikSetString;
              HasInVar:=false;
              HasLoopVar:=false;
              HasEndVar:=false;
              exit;
              end;
            end
          else
            begin
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.ConvertForStatement.InitWithResolver El.StartExpr=',GetObjName(El.StartExpr),' ResolvedIn=',GetResolverResultDbg(InResolved));
            {$ENDIF}
            RaiseNotSupported(El.StartExpr,AContext,20171113012226);
            end;
          end
        else if InResolved.BaseType in [btSet,btArrayOrSet] then
          begin
          if InResolved.SubType in btAllJSBooleans then
            InKind:=ikSetBool
          else if InResolved.SubType in btAllChars then
            InKind:=ikSetChar
          else
            InKind:=ikSetInt;
          HasInVar:=false;
          HasLoopVar:=true;
          HasEndVar:=false;
          exit;
          end
        else
          begin
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.ConvertForStatement.InitWithResolver ResolvedIn=',GetResolverResultDbg(InResolved));
          {$ENDIF}
          RaiseNotSupported(El.StartExpr,AContext,20171220221747);
          end;
        end
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.ConvertForStatement.InitWithResolver ResolvedIn=',GetResolverResultDbg(InResolved));
        {$ENDIF}
        RaiseNotSupported(El.StartExpr,AContext,20171112195629);
        end;
      end;
    end;

    if EndValue<>nil then
      begin
      HasEndVar:=false;
      if (StartValue<>nil) then
        begin
        if StartInt<=EndInt then
          begin
          // loop is always executed
          if StartValue.Kind in [revkInt,revkUInt,revkEnum] then
            HasLoopVar:=false; // variable can be used as runner
          end
        else
          begin
          // loop is never executed
          if coEliminateDeadCode in Options then exit;
          end;
        end;
      end;
  end;

  function CreateStrictNotEqual0(Left: TJSElement; PosEl: TPasElement): TJSElement;
  var
    SNE: TJSEqualityExpressionSNE;
  begin
    SNE:=TJSEqualityExpressionSNE(CreateElement(TJSEqualityExpressionSNE,PosEl));
    SNE.A:=Left;
    SNE.B:=CreateLiteralNumber(PosEl,0);
    Result:=SNE;
  end;

Var
  ForSt : TJSBodyStatement;
  List: TJSStatementList;
  SimpleAss : TJSSimpleAssignStatement;
  Incr: TJSUNaryExpression;
  BinExp : TJSBinaryExpression;
  VarStat: TJSVariableStatement;
  CurLoopVarName, CurEndVarName, CurInVarName: String;
  PosEl: TPasElement;
  Statements, V: TJSElement;
  Call: TJSCallExpression;
  Br: TJSBracketMemberExpression;
begin
  Result:=Nil;
  if AContext.Access<>caRead then
    RaiseInconsistency(20170213213740,El);
  aResolver:=AContext.Resolver;
  ForScope:=El.CustomData as TPasForLoopScope; // can be nil!
  case El.LoopType of
  ltNormal,ltDown: ;
  ltIn:
    if aResolver=nil then
      RaiseNotSupported(El,AContext,20171112160707);
  else
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.ConvertForStatement LoopType=',El.LoopType);
    {$ENDIF}
    RaiseNotSupported(El,AContext,20171110141937);
  end;

  // get function context
  FuncContext:=AContext.GetFunctionContext;

  StartValue:=nil;
  StartInt:=0;
  EndValue:=nil;
  EndInt:=0;
  InValue:=nil;
  InKind:=ikNone;
  Statements:=nil;
  try
    HasLoopVar:=true;
    HasEndVar:=true;
    HasInVar:=false;
    if (aResolver<>nil) and not InitWithResolver then
      exit;

    // create unique var names $l, $end, $in
    if FuncContext=nil then
      begin
      CurInVarName:='$in';
      CurLoopVarName:='$l';
      CurEndVarName:='$end';
      end
    else
      begin
      if HasInVar then
        CurInVarName:=FuncContext.AddLocalJSVar(GetBIName(pbivnLoopIn),true).Name
      else
        CurInVarName:='';
      if HasLoopVar then
        CurLoopVarName:=FuncContext.AddLocalJSVar(GetBIName(pbivnLoop),true).Name
      else
        CurLoopVarName:='';
      if HasEndVar then
        CurEndVarName:=FuncContext.AddLocalJSVar(GetBIName(pbivnLoopEnd),true).Name
      else
        CurEndVarName:='';
      end;

    // add "for()"
    if InKind in [ikSetInt,ikSetBool,ikSetChar,ikSetString] then
      ForSt:=TJSForInStatement(CreateElement(TJSForInStatement,El))
    else
      ForSt:=TJSForStatement(CreateElement(TJSForStatement,El));
    Statements:=ForSt;
    PosEl:=El;

    // add in front of for():  variable=<startexpr>
    if (not HasLoopVar) and (HasEndVar or HasInVar) then
      begin
      // for example:
      //   i=<startexpr>;
      //   for (var $end = <endexpr>; $i<$end; $i++)...
      List:=TJSStatementList(CreateElement(TJSStatementList,El));
      SimpleAss:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El.VariableName));
      List.A:=SimpleAss;
      List.B:=Statements;
      Statements:=List;
      SimpleAss.LHS:=ConvertExpression(El.VariableName,AContext);
      if StartValue<>nil then
        SimpleAss.Expr:=CreateLiteralNumber(El.StartExpr,StartInt)
      else
        SimpleAss.Expr:=ConvertExpression(El.StartExpr,AContext);
      PosEl:=El.StartExpr;
      end;

    if ForSt.ClassType=TJSForInStatement then
      begin
      if HasLoopVar then
        begin
        // add for("var $l" in <startexpr>)
        VarStat:=TJSVariableStatement(CreateElement(TJSVariableStatement,PosEl));
        VarStat.A:=CreatePrimitiveDotExpr(CurLoopVarName,PosEl);
        TJSForInStatement(ForSt).LHS:=VarStat;
        end
      else
        // add for("<varname>" in <startexpr>)
        TJSForInStatement(ForSt).LHS:=ConvertExpression(El.VariableName,AContext);
      // add for(<varname> in "<startexpr>")
      TJSForInStatement(ForSt).List:=ConvertExpression(El.StartExpr,AContext);
      end
    else if HasLoopVar or HasEndVar or HasInVar then
      begin
      // add "for(var ..."
      VarStat:=TJSVariableStatement(CreateElement(TJSVariableStatement,El));
      TJSForStatement(ForSt).Init:=VarStat;
      if HasInVar then
        begin
        // add "$in=<InExpr>"
        PosEl:=El.StartExpr;
        if (InValue<>nil) and (InValue.Kind<>revkSetOfInt) then
          V:=ConvertConstValue(InValue,AContext,PosEl)
        else
          V:=ConvertExpression(El.StartExpr,AContext);
        V:=CreateVarDecl(CurInVarName,V,PosEl);
        AddToVarStatement(VarStat,V,PosEl);
        end;
      if HasLoopVar then
        begin
        // add "$l=<StartExpr>"
        PosEl:=El.StartExpr;
        if StartValue<>nil then
          V:=CreateLiteralNumber(PosEl,StartInt)
        else if El.LoopType=ltIn then
          V:=CreateLiteralNumber(PosEl,StartInt)
        else
          V:=ConvExpr(El.StartExpr);
        V:=CreateVarDecl(CurLoopVarName,V,PosEl);
        AddToVarStatement(VarStat,V,PosEl);
        end;
      if HasEndVar then
        begin
        // add "$end=<EndExpr>"
        PosEl:=El.EndExpr;
        if PosEl=nil then
          PosEl:=El.StartExpr;
        if EndValue<>nil then
          V:=CreateLiteralNumber(PosEl,EndInt)
        else if El.LoopType=ltIn then
          case InKind of
          ikEnum,ikBool,ikChar:
            V:=CreateLiteralNumber(PosEl,EndInt);
          ikString:
            begin
            // add "$in.length-1"
            V:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,PosEl));
            TJSAdditiveExpressionMinus(V).A:=CreatePrimitiveDotExpr(CurInVarName+'.length',PosEl);
            TJSAdditiveExpressionMinus(V).B:=CreateLiteralNumber(PosEl,1);
            end;
          ikArray:
            begin
            // add "rtl.length($in)-1"
            Call:=CreateCallExpression(PosEl);
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnArray_Length),PosEl);
            Call.AddArg(CreatePrimitiveDotExpr(CurInVarName,PosEl));
            V:=TJSAdditiveExpressionMinus(CreateElement(TJSAdditiveExpressionMinus,PosEl));
            TJSAdditiveExpressionMinus(V).A:=Call;
            TJSAdditiveExpressionMinus(V).B:=CreateLiteralNumber(PosEl,1);
            end;
          else
            RaiseNotSupported(El.StartExpr,AContext,20171113015445);
          end
        else
          V:=ConvExpr(El.EndExpr);
        V:=CreateVarDecl(CurEndVarName,V,PosEl);
        AddToVarStatement(VarStat,V,PosEl);
        end;
      end
    else
      begin
      // No new vars. For example:
      //   for (VariableName = <startexpr>; VariableName <= <EndExpr>; VariableName++)
      SimpleAss:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El.VariableName));
      TJSForStatement(ForSt).Init:=SimpleAss;
      SimpleAss.LHS:=ConvertExpression(El.VariableName,AContext);
      if StartValue<>nil then
        SimpleAss.Expr:=CreateLiteralNumber(El.StartExpr,StartInt)
      else
        SimpleAss.Expr:=ConvertExpression(El.StartExpr,AContext);
      PosEl:=El.StartExpr;
      end;

    if ForSt.ClassType=TJSForStatement then
      begin
      // add "$l<=$end"
      if (El.EndExpr<>nil) then
        PosEl:=El.EndExpr;
      if El.Down then
        BinExp:=TJSRelationalExpressionGE(CreateElement(TJSRelationalExpressionGE,PosEl))
      else
        BinExp:=TJSRelationalExpressionLE(CreateElement(TJSRelationalExpressionLE,PosEl));
      TJSForStatement(ForSt).Cond:=BinExp;
      if HasLoopVar then
        BinExp.A:=CreatePrimitiveDotExpr(CurLoopVarName,PosEl)
      else
        BinExp.A:=ConvertExpression(El.VariableName,AContext);
      if HasEndVar then
        BinExp.B:=CreatePrimitiveDotExpr(CurEndVarName,PosEl)
      else
        BinExp.B:=CreateLiteralNumber(PosEl,EndInt);

      // add "$l++"
      if El.Down then
        Incr:=TJSUnaryPostMinusMinusExpression(CreateElement(TJSUnaryPostMinusMinusExpression,PosEl))
      else
        Incr:=TJSUnaryPostPlusPlusExpression(CreateElement(TJSUnaryPostPlusPlusExpression,PosEl));
      TJSForStatement(ForSt).Incr:=Incr;
      if HasLoopVar then
        Incr.A:=CreatePrimitiveDotExpr(CurLoopVarName,PosEl)
      else
        Incr.A:=ConvertExpression(El.VariableName,AContext);
      end;

    // add  "VariableName:=$l;"
    if HasLoopVar then
      begin
      PosEl:=El.Body;
      if PosEl=nil then
        PosEl:=El;
      PosEl:=El.VariableName;
      SimpleAss:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,PosEl));
      ForSt.Body:=SimpleAss;
      SimpleAss.LHS:=ConvertExpression(El.VariableName,AContext);
      SimpleAss.Expr:=CreatePrimitiveDotExpr(CurLoopVarName,PosEl);
      if aResolver<>nil then
        begin
        if InKind<>ikNone then
          case InKind of
          ikEnum,ikSetInt:
            if ForSt.ClassType=TJSForInStatement then
              // $in=+$l
              SimpleAss.Expr:=CreateUnaryPlus(SimpleAss.Expr,PosEl);
          ikBool,ikSetBool:
            // $in!==0;
            SimpleAss.Expr:=CreateStrictNotEqual0(SimpleAss.Expr,PosEl);
          ikChar,ikSetChar:
            // String.fromCharCode($l)
            SimpleAss.Expr:=CreateCallFromCharCode(SimpleAss.Expr,PosEl);
          ikString:
            begin
            // $in.charAt($l)
            Call:=CreateCallExpression(PosEl);
            Call.Expr:=CreateDotNameExpr(PosEl,
              CreatePrimitiveDotExpr(CurInVarName,El.StartExpr),
              'charAt');
            Call.AddArg(SimpleAss.Expr);
            SimpleAss.Expr:=Call;
            end;
          ikArray:
            begin
            // $in[$l]
            Br:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,PosEl));
            Br.MExpr:=CreatePrimitiveDotExpr(CurInVarName,El.StartExpr);
            Br.Name:=SimpleAss.Expr;
            SimpleAss.Expr:=Br;
            end;
          else
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.ConvertForStatement InKind=',InKind);
            {$ENDIF}
            RaiseNotSupported(El.StartExpr,AContext,20171113002550);
          end
        else if (VarResolved.BaseType in btAllChars)
            or ((VarResolved.BaseType=btRange) and (VarResolved.SubType in btAllChars)) then
          begin
          // convert int to char
          SimpleAss.Expr:=CreateCallFromCharCode(SimpleAss.Expr,PosEl);
          end
        else if (VarResolved.BaseType in btAllJSBooleans)
            or ((VarResolved.BaseType=btRange) and (VarResolved.SubType in btAllJSBooleans)) then
          begin
          // convert int to bool  ->  $l!=0
          SimpleAss.Expr:=CreateStrictNotEqual0(SimpleAss.Expr,PosEl);
          end
        end;
      end;

    // add body
    if El.Body<>nil then
      begin
      V:=ConvertElement(El.Body,AContext);
      if ForSt.Body=nil then
        ForSt.Body:=V
      else
        begin
        List:=TJSStatementList(CreateElement(TJSStatementList,El.Body));
        List.A:=ForSt.Body;
        List.B:=V;
        ForSt.Body:=List;
        end;
      end;
    Result:=Statements;
  finally
    ReleaseEvalValue(StartValue);
    ReleaseEvalValue(EndValue);
    ReleaseEvalValue(InValue);
    if Result=nil then
      Statements.Free;
  end;
end;

function TPasToJSConverter.ConvertSimpleStatement(El: TPasImplSimple;
  AContext: TConvertContext): TJSElement;

Var
  E : TJSElement;
  C: TClass;

begin
  E:=ConvertExpression(EL.Expr,AContext);
  if E=nil then
    exit(nil); // e.g. "inherited;" without ancestor proc
  C:=E.ClassType;
  if (C=TJSExpressionStatement)
      or (C=TJSStatementList) then
    Result:=E
  else
    begin
    Result:=TJSExpressionStatement(CreateElement(TJSExpressionStatement,El));
    TJSExpressionStatement(Result).A:=E;
    end;
end;

function TPasToJSConverter.ConvertWithStatement(El: TPasImplWithDo;
  AContext: TConvertContext): TJSElement;
Var
  aResolver: TPas2JSResolver;
  FuncContext: TFunctionContext;
  WithScope: TPasWithScope;
  WithExprScope: TPas2JSWithExprScope;
  PasExpr: TPasExpr;
  ResolvedEl: TPasResolverResult;
  B,E , Expr: TJSElement;
  W,W2 : TJSWithStatement;
  I : Integer;
  ok: Boolean;
  V: TJSVariableStatement;
  FirstSt, LastSt: TJSStatementList;
  TypeEl: TPasType;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  if aResolver<>nil then
    begin
    // with Resolver:
    // Insert for each expression a local var. Example:
    //   with aPoint do X:=3;
    // convert to
    //   var $with1 = aPoint;
    //   $with1.X = 3;
    FuncContext:=TFunctionContext(AContext.GetContextOfType(TFunctionContext));
    if FuncContext=nil then
      RaiseInconsistency(20170212003759,El);
    FirstSt:=nil;
    LastSt:=nil;
    try
      WithScope:=El.CustomData as TPasWithScope;
      for i:=0 to El.Expressions.Count-1 do
        begin
        PasExpr:=TPasExpr(El.Expressions[i]);
        aResolver.ComputeElement(PasExpr,ResolvedEl,[]);
        if ResolvedEl.IdentEl is TPasType then
          begin
          TypeEl:=ResolvedEl.LoTypeEl;
          if (TypeEl.ClassType=TPasClassType)
              or (TypeEl.ClassType=TPasRecordType)
              or (TypeEl.ClassType=TPasEnumType) then
            // have JS object -> ok
          else
            begin
            // e.g. "with byte do"  allowed with type helpers
            continue;
            end;
          end;
        Expr:=ConvertExpression(PasExpr,AContext);

        WithExprScope:=WithScope.ExpressionScopes[i] as TPas2JSWithExprScope;
        if (Expr is TJSPrimaryExpressionIdent)
            and IsValidJSIdentifier(TJSPrimaryExpressionIdent(Expr).Name) then
          begin
          // expression is already a local variable
          WithExprScope.WithVarName:=String(TJSPrimaryExpressionIdent(Expr).Name);
          Expr.Free;
          end
        else if Expr is TJSPrimaryExpressionThis then
          begin
          // expression is 'this'
          WithExprScope.WithVarName:='this';
          Expr.Free;
          end
        else
          begin
          // create unique local var name
          WithExprScope.WithVarName:=FuncContext.AddLocalJSVar(GetBIName(pbivnWith),true).Name;
          // create local "var $with1 = expr;"
          V:=CreateVarStatement(WithExprScope.WithVarName,Expr,PasExpr);
          AddToStatementList(FirstSt,LastSt,V,PasExpr);
          end;
        end;
      // convert with body
      if Assigned(El.Body) then
        begin
        B:=ConvertElement(El.Body,AContext);
        AddToStatementList(FirstSt,LastSt,B,El.Body);
        end;
      Result:=FirstSt;
    finally
      if Result=nil then
        FreeAndNil(FirstSt);
    end;
    end
  else
    begin
    // without Resolver use as fallback the JavaScript with(){}
    W:=Nil;
    if Assigned(El.Body) then
      B:=ConvertElement(El.Body,AContext)
    else
      B:=TJSEmptyBlockStatement(CreateElement(TJSEmptyBlockStatement,El));
    ok:=false;
    try
      For I:=0 to El.Expressions.Count-1 do
        begin
        PasExpr:=TPasExpr(El.Expressions[i]);
        E:=ConvertExpression(PasExpr,AContext);
        W2:=TJSWithStatement(CreateElement(TJSWithStatement,PasExpr));
        if Not Assigned(Result) then // result is the first
          Result:=W2;
        if Assigned(W) then // Chain
          W.B:=W2;
        W:=W2; // W is the last
        W.A:=E;
        end;
      ok:=true;
    finally
      if not ok then
        begin
        FreeAndNil(E);
        FreeAndNil(Result);
        end;
    end;
    W.B:=B;
    end;
end;

function TPasToJSConverter.IsElementUsed(El: TPasElement): boolean;
begin
  if Assigned(OnIsElementUsed) then
    Result:=OnIsElementUsed(Self,El)
  else
    Result:=true;
end;

function TPasToJSConverter.IsSystemUnit(aModule: TPasModule): boolean;
begin
  Result:=(CompareText(aModule.Name,'system')=0) and (aModule.ClassType=TPasModule);
end;

function TPasToJSConverter.HasTypeInfo(El: TPasType; AContext: TConvertContext
  ): boolean;
begin
  Result:=false;
  if coNoTypeInfo in Options then exit;
  if AContext.Resolver=nil then exit;
  if not AContext.Resolver.HasTypeInfo(El) then exit;
  if Assigned(OnIsTypeInfoUsed) and not OnIsTypeInfoUsed(Self,El) then exit;
  Result:=true;
end;

function TPasToJSConverter.IsClassRTTICreatedBefore(aClass: TPasClassType;
  Before: TPasElement; AConText: TConvertContext): boolean;
var
  Decls: TPasDeclarations;
  i: Integer;
  List: TFPList;
  C: TClass;
  aParent, Decl: TPasElement;
begin
  Result:=false;
  aParent:=aClass.Parent;
  if aParent<>Before.Parent then
    exit(true);
  if not aParent.InheritsFrom(TPasDeclarations) then
    RaiseInconsistency(20170412101457,aClass);
  Decls:=TPasDeclarations(aParent);
  List:=Decls.Declarations;
  for i:=0 to List.Count-1 do
    begin
    Decl:=TPasElement(List[i]);
    if Decl=Before then exit;
    if Decl=aClass then exit(true);
    C:=Decl.ClassType;
    if C=TPasClassType then
      begin
      if TPasClassType(Decl).IsForward and (Decl.CustomData is TResolvedReference)
          and (TResolvedReference(Decl.CustomData).Declaration=aClass) then
        exit(true);
      end
    else if C=TPasClassOfType then
      begin
      if AConText.Resolver.ResolveAliasType(TPasClassOfType(Decl).DestType)=aClass then
        exit(true);
      end;
    end;
end;

function TPasToJSConverter.IsExprTemporaryVar(Expr: TPasExpr): boolean;
var
  Params: TParamsExpr;
  Ref: TResolvedReference;
  C: TClass;
begin
  if Expr.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(Expr.CustomData);
    if [rrfNewInstance,rrfImplicitCallWithoutParams]*Ref.Flags<>[] then
      exit(true);
    end;

  C:=Expr.ClassType;
  if C=TParamsExpr then
    begin
    Params:=TParamsExpr(Expr);
    if Params.Kind=pekFuncParams then
      exit(true);
    end
  else if C.InheritsFrom(TBinaryExpr) then
    exit(true);

  Result:=false;
end;

function TPasToJSConverter.IsExprPropertySetterConst(Expr: TPasExpr;
  AContext: TConvertContext): boolean;
var
  Bin: TBinaryExpr;
  Ref: TResolvedReference;
  Prop: TPasProperty;
  Setter, Arg: TPasElement;
  Args: TFPList;
begin
  if Expr is TBinaryExpr then
    begin
    Bin:=TBinaryExpr(Expr);
    if Bin.OpCode=eopSubIdent then
      Expr:=Bin.right;
    end;
  if Expr.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(Expr.CustomData);
    if Ref.Declaration is TPasProperty then
      begin
      Prop:=TPasProperty(Ref.Declaration);
      Setter:=AContext.Resolver.GetPasPropertySetter(Prop);
      if Setter is TPasProcedure then
        begin
        Args:=TPasProcedure(Setter).ProcType.Args;
        if Args.Count>0 then
          begin
          Arg:=TPasElement(Args[Args.Count-1]);
          if (Arg is TPasArgument) and (TPasArgument(Arg).Access in [argConst,argConstRef]) then
            exit(true);
          end;
        end;
      end;
    end;
  Result:=false;
end;

procedure TPasToJSConverter.FindAvailableLocalName(var aName: string;
  JSExpr: TJSElement);
var
  StartJSName, JSName: TJSString;
  n: integer;
  Changed: boolean;

  procedure Next;
  var
    ch: WideChar;
  begin
    Changed:=true;
    // name clash -> change JSName
    if (n=0) and (length(JSName)=1) then
      begin
      // single letter -> choose next single letter
      ch:=JSName[1];
      case ch of
      'a'..'x': JSName:=succ(ch);
      'z': JSName:='a';
      end;
      if JSName=StartJSName then
        begin
        n:=1;
        JSName:=StartJSName+TJSString(IntToStr(n));
        end;
      end
    else
      begin
      inc(n);
      JSName:=StartJSName+TJSString(IntToStr(n));
      end;
  end;

  procedure Find(El: TJSElement);
  var
    C: TClass;
    Call: TJSCallExpression;
    i: Integer;
  begin
    if El=nil then exit;
    C:=El.ClassType;
    if C=TJSPrimaryExpressionIdent then
      begin
      if TJSPrimaryExpressionIdent(El).Name=JSName then
        Next;
      end
    else if C.InheritsFrom(TJSMemberExpression) then
      begin
      Find(TJSMemberExpression(El).MExpr);
      if C=TJSBracketMemberExpression then
        Find(TJSBracketMemberExpression(El).Name)
      else if C=TJSNewMemberExpression then
        with TJSNewMemberExpression(El).Args.Elements do
          for i:=0 to Count-1 do
            Find(Elements[i].Expr);
      end
    else if C=TJSCallExpression then
      begin
      Call:=TJSCallExpression(El);
      Find(Call.Expr);
      if Call.Args<>nil then
        with Call.Args.Elements do
          for i:=0 to Count-1 do
            Find(Elements[i].Expr);
      end
    else if C.InheritsFrom(TJSUnary) then
      Find(TJSUnary(El).A)
    else if C.InheritsFrom(TJSBinary) then
      begin
      Find(TJSBinary(El).A);
      Find(TJSBinary(El).B);
      end
    else if C=TJSArrayLiteral then
      begin
      with TJSArrayLiteral(El).Elements do
        for i:=0 to Count-1 do
          Find(Elements[i].Expr);
      end
    else if C=TJSConditionalExpression then
      begin
      Find(TJSConditionalExpression(El).A);
      Find(TJSConditionalExpression(El).B);
      Find(TJSConditionalExpression(El).C);
      end
    else if C.InheritsFrom(TJSAssignStatement) then
      begin
      Find(TJSAssignStatement(El).LHS);
      Find(TJSAssignStatement(El).Expr);
      end
    else if C=TJSVarDeclaration then
      Find(TJSVarDeclaration(El).Init)
    else if C=TJSObjectLiteral then
      begin
      with TJSObjectLiteral(El).Elements do
        for i:=0 to Count-1 do
          Find(Elements[i].Expr);
      end
    else if C=TJSIfStatement then
      begin
      Find(TJSIfStatement(El).Cond);
      Find(TJSIfStatement(El).BTrue);
      Find(TJSIfStatement(El).BFalse);
      end
    else if C.InheritsFrom(TJSBodyStatement) then
      begin
      Find(TJSBodyStatement(El).Body);
      if C.InheritsFrom(TJSCondLoopStatement) then
        begin
        Find(TJSCondLoopStatement(El).Cond);
        if C=TJSForStatement then
          begin
          Find(TJSForStatement(El).Init);
          Find(TJSForStatement(El).Incr);
          end;
        end
      else if C=TJSForInStatement then
        begin
        Find(TJSForInStatement(El).LHS);
        Find(TJSForInStatement(El).List);
        end;
      end
    else if C=TJSSwitchStatement then
      begin
      Find(TJSSwitchStatement(El).Cond);
      with TJSSwitchStatement(El).Cases do
        for i:=0 to Count-1 do
          with Cases[i] do
            begin
            Find(Expr);
            Find(Body);
            end;
      if TJSSwitchStatement(El).TheDefault<>nil then
        with TJSSwitchStatement(El).TheDefault do
          begin
          Find(Expr);
          Find(Body);
          end;
      end;
  end;

begin
  if JSExpr=nil then exit;
  StartJSName:=TJSString(aName);
  JSName:=StartJSName;
  n:=0;
  Changed:=false;
  Find(JSExpr);
  if not Changed then exit;
  repeat
    Changed:=false;
    Find(JSExpr);
  until not changed;
  aName:=JSStringToString(JSName);
end;

function TPasToJSConverter.GetImplJSProcScope(El: TPasElement;
  Src: TJSSourceElements; AContext: TConvertContext): TPas2JSProcedureScope;
begin
  if (Src=nil) or not (coStoreImplJS in Options) or (AContext.Resolver=nil) then
    exit(nil);
  Result:=AContext.Resolver.GetTopLvlProcScope(El);
end;

function TPasToJSConverter.SpecializeNeedsDelay(El: TPasGenericType;
  AContext: TConvertContext): boolean;
var
  SpecItem: TPRSpecializedItem;
  C: TClass;
  Members: TFPList;
  ChildEl: TPasElement;
  PasVar: TPasVariable;
  aResolver: TPas2JSResolver;
  PasVarType: TPasType;
  IsRecord, NeedInitFunction: Boolean;
  aClass: TPasClassType;
  ClassScope: TPas2JSClassScope;
  IntfKind: String;
  i: Integer;
begin
  Result:=false;
  aResolver:=AContext.Resolver;
  if aResolver=nil then exit;
  if not (El.CustomData is TPasGenericScope) then exit;
  SpecItem:=TPasGenericScope(El.CustomData).SpecializedFromItem;
  if aResolver.SpecializeParamsNeedDelay(SpecItem)=nil then
    exit; // params are declared in front of generic -> no need to delay

  if HasTypeInfo(El,AContext) then
    exit(true); // RTTI -> delay needed

  C:=El.ClassType;
  if El.InheritsFrom(TPasMembersType) then
    begin
    IsRecord:=C=TPasRecordType;

    if C=TPasClassType then
      begin
      aClass:=TPasClassType(El);
      ClassScope:=TPas2JSClassScope(El.CustomData);
      if aClass.ObjKind=okInterface then
        begin
        IntfKind:='';
        if (ClassScope.AncestorScope=nil) and (not (coNoTypeInfo in Options)) then
          case aClass.InterfaceType of
          citCom: IntfKind:='com';
          citCorba: ; // default
          else
            RaiseNotSupported(El,AContext,20200905132130);
          end;
        NeedInitFunction:=(pcsfPublished in ClassScope.Flags) or (IntfKind<>'');
        if not NeedInitFunction then
          exit; // interface without init function -> no need to delay
        end;
      end;

    Members:=TPasMembersType(El).Members;
    for i:=0 to Members.Count-1 do
      begin
      ChildEl:=TPasElement(Members[i]);
      if not IsElementUsed(ChildEl) then continue;
      if ChildEl is TPasVariable then
        begin
        PasVar:=TPasVariable(ChildEl);
        if ChildEl.ClassType=TPasConst then
        else if ChildEl.ClassType=TPasVariable then
          begin
          if (not IsRecord) and (PasVar.VarModifiers*[vmClass, vmStatic]=[]) then
            continue; // class field -> no delay needed
          end
        else
          continue;
        PasVarType:=aResolver.ResolveAliasType(PasVar.VarType);
        if (PasVarType.ClassType=TPasRecordType) then
          exit(true) // global record -> needs delay (Eventually: check if it uses one of the after params)
        else if (PasVarType.ClassType=TPasArrayType) and (length(TPasArrayType(PasVarType).Ranges)>0) then
          exit(true); // global static array -> needs delay (Eventually: check if it uses one of the after params)
        end;
      end;
    end;
end;

function TPasToJSConverter.CreateUnary(Members: array of string; E: TJSElement): TJSUnary;
var
  unary: TJSUnary;
  asi: TJSSimpleAssignStatement;
begin
  unary := TJSUnary.Create(0, 0, '');
  asi := TJSSimpleAssignStatement.Create(0, 0, '');
  unary.A := asi;
  asi.Expr := E;
  asi.LHS := CreateMemberExpression(Members);
  Result := unary;
end;

function TPasToJSConverter.CreateUnaryPlus(Expr: TJSElement; El: TPasElement
  ): TJSUnaryPlusExpression;
begin
  Result:=TJSUnaryPlusExpression(CreateElement(TJSUnaryPlusExpression,El));
  Result.A:=Expr;
end;

function TPasToJSConverter.CreateMemberExpression(Members: array of string
  ): TJSElement;
// Examples:
//   foo   ->  foo
//   foo,bar  -> foo.bar
//   foo,[1]  ->  foo[1]
var
  Prim: TJSPrimaryExpressionIdent;
  MExpr, LastMExpr: TJSMemberExpression;
  k: integer;
  CurName: String;
begin
  if Length(Members) < 1 then
    DoError(20161024192715,'internal error: member expression needs at least one element');
  LastMExpr := nil;
  for k:=High(Members) downto Low(Members)+1 do
  begin
    CurName:=Members[k];
    if CurName='' then
      DoError(20190124114806,'internal error: member expression needs name');
    if CurName[1]='[' then
      begin
      if CurName[length(CurName)]=']' then
        CurName:=copy(CurName,2,length(CurName)-2)
      else
        CurName:=copy(CurName,2,length(CurName)-1);
      MExpr := TJSBracketMemberExpression.Create(0,0,'');
      Prim := TJSPrimaryExpressionIdent.Create(0, 0, '');
      Prim.Name:=TJSString(CurName);
      TJSBracketMemberExpression(MExpr).Name := Prim;
      end
    else
      begin
      MExpr := TJSDotMemberExpression.Create(0, 0, '');
      TJSDotMemberExpression(MExpr).Name := TJSString(CurName);
      end;
    if LastMExpr=nil then
      Result := MExpr
    else
      LastMExpr.MExpr := MExpr;
    LastMExpr := MExpr;
  end;
  Prim := TJSPrimaryExpressionIdent.Create(0, 0, '');
  Prim.Name := TJSString(Members[Low(Members)]);
  if LastMExpr=nil then
    Result:=Prim
  else
    LastMExpr.MExpr := Prim;
end;

function TPasToJSConverter.CreateCallExpression(El: TPasElement
  ): TJSCallExpression;
begin
  Result:=TJSCallExpression(CreateElement(TJSCallExpression,El));
  Result.Args:=TJSArguments(CreateElement(TJSArguments,El));
end;

function TPasToJSConverter.CreateCallCharCodeAt(Arg: TJSElement;
  aNumber: integer; El: TPasElement): TJSCallExpression;
begin
  Result:=CreateCallExpression(El);
  Result.Expr:=CreateDotNameExpr(El,Arg,'charCodeAt');
  if aNumber<>0 then
    Result.Args.AddElement(CreateLiteralNumber(El,aNumber));
end;

function TPasToJSConverter.CreateCallFromCharCode(Arg: TJSElement;
  El: TPasElement): TJSCallExpression;
begin
  Result:=CreateCallExpression(El);
  Result.Expr:=CreateMemberExpression(['String','fromCharCode']);
  Result.AddArg(Arg);
end;

function TPasToJSConverter.CreateUsesList(UsesSection: TPasSection;
  AContext: TConvertContext): TJSArrayLiteral;
var
  ArgArray: TJSArrayLiteral;
  i: Integer;
  anUnitName: String;
  ArgEx: TJSLiteral;
  UsesClause: TPasUsesClause;
  aModule: TPasModule;
begin
  UsesClause:=UsesSection.UsesClause;
  ArgArray:=TJSArrayLiteral.Create(0,0);
  for i:=0 to length(UsesClause)-1 do
    begin
    aModule:=UsesClause[i].Module as TPasModule;
    if (not IsElementUsed(aModule)) and not IsSystemUnit(aModule) then
      continue;
    anUnitName := TransformModuleName(aModule,false,AContext);
    ArgEx := CreateLiteralString(UsesSection,anUnitName);
    ArgArray.Elements.AddElement.Expr := ArgEx;
    end;
  Result:=ArgArray;
end;

procedure TPasToJSConverter.AddToStatementList(var First,
  Last: TJSStatementList; Add: TJSElement; Src: TPasElement);
var
  SL2: TJSStatementList;
begin
  if Add=nil then exit;
  if Add is TJSStatementList then
    begin
    // add list
    if TJSStatementList(Add).A=nil then
      begin
      // empty list -> skip
      if TJSStatementList(Add).B<>nil then
        raise Exception.Create('internal error: AddToStatementList add list A=nil, B<>nil, B='+TJSStatementList(Add).B.ClassName);
      FreeAndNil(Add);
      end
    else if Last=nil then
      begin
      // our list is not yet started -> simply take the extra list
      Last:=TJSStatementList(Add);
      First:=Last;
      end
    else
      begin
      // merge lists (append)
      if Last.B<>nil then
        begin
        // add a nil to the end of chain
        SL2:=TJSStatementList(CreateElement(TJSStatementList,Src));
        SL2.A:=Last.B;
        Last.B:=SL2;
        Last:=SL2;
        // Last.B is now nil
        end;
      Last.B:=Add;
      while Last.B is TJSStatementList do
        Last:=TJSStatementList(Last.B);
      end;
    end
  else
    begin
    if Last=nil then
      begin
      // start list
      Last:=TJSStatementList(CreateElement(TJSStatementList,Src));
      First:=Last;
      Last.A:=Add;
      end
    else if Last.B=nil then
      // second element
      Last.B:=Add
    else
      begin
      // add to chain
      while Last.B is TJSStatementList do
        Last:=TJSStatementList(Last.B);
      SL2:=TJSStatementList(CreateElement(TJSStatementList,Src));
      SL2.A:=Last.B;
      Last.B:=SL2;
      Last:=SL2;
      Last.B:=Add;
      end;
    end;
end;

procedure TPasToJSConverter.AddToStatementList(St: TJSStatementList;
  Add: TJSElement; Src: TPasElement);
var
  First, Last: TJSStatementList;
begin
  First:=St;
  Last:=St;
  while Last.B is TJSStatementList do
    Last:=TJSStatementList(Last.B);
  AddToStatementList(First,Last,Add,Src);
end;

procedure TPasToJSConverter.PrependToStatementList(var St: TJSElement;
  Add: TJSElement; PosEl: TPasElement);
var
  NewSt: TJSStatementList;
begin
  if St=nil then
    St:=Add
  else if St is TJSEmptyBlockStatement then
    begin
    St.Free;
    St:=Add;
    end
  else if St is TJSStatementList then
    begin
    NewSt:=TJSStatementList(CreateElement(TJSStatementList,PosEl));
    NewSt.A:=Add;
    NewSt.B:=St;
    St:=NewSt;
    end
  else
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.PrependToStatementList St=',GetObjName(St));
    {$ENDIF}
    RaiseNotSupported(PosEl,nil,20181002154026,GetObjName(St));
    end;
end;

procedure TPasToJSConverter.AddToVarStatement(VarStat: TJSVariableStatement;
  Add: TJSElement; Src: TPasElement);
var
  List: TJSVariableDeclarationList;
begin
  if VarStat.A=nil then
    VarStat.A:=Add
  else
    begin
    List:=TJSVariableDeclarationList(CreateElement(TJSVariableDeclarationList,Src));
    List.A:=VarStat.A;
    List.B:=Add;
    VarStat.A:=List;
    end;
end;

function TPasToJSConverter.CreateValInit(PasType: TPasType; Expr: TPasExpr;
  El: TPasElement; AContext: TConvertContext): TJSElement;
var
  T: TPasType;
  Lit: TJSLiteral;
  bt: TResolverBaseType;
  JSBaseType: TPas2jsBaseType;
  C: TClass;
  aResolver: TPas2JSResolver;
  Value: TResEvalValue;
begin
  T:=PasType;
  aResolver:=AContext.Resolver;
  if aResolver<>nil then
    T:=aResolver.ResolveAliasType(T);

  //writeln('START TPasToJSConverter.CreateValInit PasType=',GetObjName(PasType),' El=',GetObjName(El),' T=',GetObjName(T),' Expr=',GetObjName(Expr));
  if T=nil then
    begin
    // untyped var/const
    if Expr=nil then
      begin
      if aResolver=nil then
        exit(CreateLiteralUndefined(El));
      RaiseInconsistency(20170415185745,El);
      end;
    Result:=ConvertExpression(Expr,AContext);
    if Result=nil then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.CreateValInit PasType=',GetObjName(PasType),' El=',GetObjName(El),' T=',GetObjName(T),' Expr=',GetObjName(Expr));
      {$ENDIF}
      RaiseNotSupported(Expr,AContext,20170415185927);
      end;
    exit;
    end;

  C:=T.ClassType;
  if C=TPasArrayType then
    Result:=CreateArrayInit(TPasArrayType(T),Expr,El,AContext)
  else if C=TPasRecordType then
    Result:=CreateRecordInit(TPasRecordType(T),Expr,El,AContext)
  else if Assigned(Expr) then
    // if there is an expression then simply convert it
    Result:=ConvertExpression(Expr,AContext)
  else if C=TPasSetType then
    // a "set" without initial value
    Result:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El))
  else if (C=TPasRangeType) and (aResolver<>nil) then
    begin
    // a custom range without initial value
    // -> for FPC/Delphi compatibility use 0 even if it is out of range
    Value:=AContext.Resolver.Eval(TPasRangeType(T).RangeExpr.left,[refConst]);
    try
      case Value.Kind of
      revkInt,revkUInt: Result:=CreateLiteralNumber(El,0);
      else
        Result:=ConvertConstValue(Value,AContext,El);
      end;
    finally
      ReleaseEvalValue(Value);
    end;
    end
  else
    begin
    // always init with a default value to create a typed variable (faster and more readable)
    Lit:=TJSLiteral(CreateElement(TJSLiteral,El));
    Result:=Lit;
    if (C=TPasPointerType)
        or (C=TPasClassType)
        or (C=TPasClassOfType)
        or (C=TPasProcedureType)
        or (C=TPasFunctionType) then
      Lit.Value.IsNull:=true
    else if C=TPasStringType then
      Lit.Value.AsString:=''
    else if C=TPasEnumType then
      Lit.Value.AsNumber:=0
    else if C=TPasUnresolvedSymbolRef then
      begin
      if T.CustomData is TResElDataBaseType then
        begin
        bt:=TResElDataBaseType(T.CustomData).BaseType;
        if bt in btAllJSInteger then
          Lit.Value.AsNumber:=0
        else if bt in btAllJSFloats then
          Lit.Value.CustomValue:='0.0'
        else if bt in btAllJSStringAndChars then
          Lit.Value.AsString:=''
        else if bt in btAllJSBooleans then
          Lit.Value.AsBoolean:=false
        else if bt in [btNil,btPointer,btProc] then
          Lit.Value.IsNull:=true
        else if (bt=btCustom) and (T.CustomData is TResElDataPas2JSBaseType) then
          begin
          JSBaseType:=TResElDataPas2JSBaseType(T.CustomData).JSBaseType;
          if JSBaseType=pbtJSValue then
            Lit.Value.IsUndefined:=true;
          end
        else
          begin
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.CreateVarInit unknown PasType T=',GetObjName(T),' basetype=',aResolver.BaseTypeNames[bt]);
          {$ENDIF}
          RaiseNotSupported(PasType,AContext,20170208162121);
          end;
        end
      else if aResolver<>nil then
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateValInit PasType=',GetObjName(PasType),' El=',GetObjName(El),' T=',GetObjName(T),' Expr=',GetObjName(Expr));
        {$ENDIF}
        RaiseNotSupported(El,AContext,20170415190259);
        end
      else if (CompareText(T.Name,'longint')=0)
           or (CompareText(T.Name,'int64')=0)
           or (CompareText(T.Name,'real')=0)
           or (CompareText(T.Name,'double')=0)
           or (CompareText(T.Name,'single')=0) then
        Lit.Value.AsNumber:=0.0
      else if (CompareText(T.Name,'boolean')=0) then
        Lit.Value.AsBoolean:=false
      else if (CompareText(T.Name,'string')=0)
           or (CompareText(T.Name,'char')=0)
      then
        Lit.Value.AsString:=''
      else
        begin
        Lit.Value.IsUndefined:=true;
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateVarInit unknown PasType class=',T.ClassName,' name=',T.Name);
        {$ENDIF}
        end;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.CreateValInit unknown PasType ',GetObjName(T));
      {$ENDIF}
      RaiseNotSupported(PasType,AContext,20170208161506);
      end;
    end;
  if Result=nil then
    begin
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.CreateValInit PasType=',GetObjName(PasType),' El=',GetObjName(El),' T=',GetObjName(T),' Expr=',GetObjName(Expr));
    {$ENDIF}
    RaiseNotSupported(El,AContext,20170415190103);
    end;
end;

function TPasToJSConverter.CreateVarInit(El: TPasVariable;
  AContext: TConvertContext): TJSElement;
begin
  Result:=CreateValInit(El.VarType,El.Expr,El,AContext);
end;

function TPasToJSConverter.CreateVarStatement(const aName: String;
  Init: TJSElement; El: TPasElement): TJSVariableStatement;
// create "var aname = init"
begin
  Result:=TJSVariableStatement(CreateElement(TJSVariableStatement,El));
  Result.A:=CreateVarDecl(aName,Init,El);
end;

function TPasToJSConverter.CreateVarDecl(const aName: String; Init: TJSElement;
  El: TPasElement): TJSVarDeclaration;
begin
  Result:=TJSVarDeclaration(CreateElement(TJSVarDeclaration,El));
  Result.Name:=aName;
  Result.Init:=Init;
end;

function TPasToJSConverter.CreateLiteralNumber(El: TPasElement;
  const n: TJSNumber): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.AsNumber:=n;
end;

function TPasToJSConverter.CreateLiteralHexNumber(El: TPasElement;
  const n: TMaxPrecInt; Digits: byte): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.AsNumber:=n;
  Result.Value.CustomValue:=TJSString('0x'+HexStr(n,Digits));
end;

function TPasToJSConverter.CreateLiteralString(El: TPasElement; const s: string
  ): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.AsString:=TJSString(s);
end;

function TPasToJSConverter.CreateLiteralJSString(El: TPasElement;
  const s: TJSString): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.AsString:=s;
end;

function TPasToJSConverter.CreateLiteralBoolean(El: TPasElement; b: boolean
  ): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.AsBoolean:=b;
end;

function TPasToJSConverter.CreateLiteralNull(El: TPasElement): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.IsNull:=true;
end;

function TPasToJSConverter.CreateLiteralUndefined(El: TPasElement): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.IsUndefined:=true;
end;

function TPasToJSConverter.CreateLiteralCustomValue(El: TPasElement;
  const s: TJSString): TJSLiteral;
begin
  Result:=TJSLiteral(CreateElement(TJSLiteral,El));
  Result.Value.CustomValue:=s;
end;

function TPasToJSConverter.CreateSetLiteralElement(Expr: TPasExpr;
  AContext: TConvertContext): TJSElement;
var
  LitVal: TJSValue;
  NewEl: TJSElement;
  WS: TJSString;
  ExprResolved: TPasResolverResult;
  Call: TJSCallExpression;
  DotExpr: TJSDotMemberExpression;
  aResolver: TPas2JSResolver;
  bt: TResolverBaseType;
  C: TClass;
begin
  Result:=ConvertExpression(Expr,AContext);
  if Result=nil then
    RaiseNotSupported(Expr,AContext,20170415192209);
  if Result.ClassType=TJSLiteral then
    begin
    // argument is a literal  -> convert to number
    LitVal:=TJSLiteral(Result).Value;
    case LitVal.ValueType of
      jstBoolean:
        begin
        if LitVal.AsBoolean=LowJSBoolean then
          NewEl:=CreateLiteralNumber(Expr,0)
        else
          NewEl:=CreateLiteralNumber(Expr,1);
        Result.Free;
        exit(NewEl);
        end;
      jstNumber:
        exit;
      jstString:
        begin
        WS:=LitVal.AsString;
        Result.Free;
        if length(WS)<>1 then
          DoError(20170415193254,nXExpectedButYFound,sXExpectedButYFound,['char','string'],Expr);
        Result:=CreateLiteralNumber(Expr,ord(WS[1]));
        exit;
        end;
    else
      RaiseNotSupported(Expr,AContext,20170415205955);
    end;
    end
  else if Result.ClassType=TJSCallExpression then
    begin
    Call:=TJSCallExpression(Result);
    if (Call.Expr is TJSDotMemberExpression) then
      begin
      DotExpr:=TJSDotMemberExpression(Call.Expr);
      if DotExpr.Name='charCodeAt' then
        exit;
      if DotExpr.Name='charAt' then
        begin
        DotExpr.Name:='charCodeAt';
        exit;
        end;
      end;
    end;

  aResolver:=AContext.Resolver;
  if aResolver<>nil then
    begin
    aResolver.ComputeElement(Expr,ExprResolved,[]);
    bt:=ExprResolved.BaseType;
    if bt=btRange then
      bt:=ExprResolved.SubType;
    if bt in btAllJSStringAndChars then
      begin
      // aChar -> aChar.charCodeAt()
      Result:=CreateCallCharCodeAt(Result,0,Expr);
      end
    else if bt in btAllJSInteger then
      begin
      // ok
      end
    else if bt=btContext then
      begin
      C:=ExprResolved.LoTypeEl.ClassType;
      if (C=TPasEnumType) or (C=TPasRangeType) then
        // ok
      else
        RaiseNotSupported(Expr,AContext,20170415191933);
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.CreateSetLiteralElement ',GetResolverResultDbg(ExprResolved));
      {$ENDIF}
      RaiseNotSupported(Expr,AContext,20170415191822);
      end;
    end;
end;

function TPasToJSConverter.CreateUnaryNot(El: TJSElement; Src: TPasElement
  ): TJSUnaryNotExpression;
begin
  Result:=TJSUnaryNotExpression(CreateElement(TJSUnaryNotExpression,Src));
  Result.A:=El;
end;

procedure TPasToJSConverter.ConvertCharLiteralToInt(Lit: TJSLiteral;
  ErrorEl: TPasElement; AContext: TConvertContext);
var
  JS: TJSString;
begin
  if Lit.Value.ValueType<>jstString then
    RaiseInconsistency(20171112020856,ErrorEl);
  if Lit.Value.CustomValue<>'' then
    JS:=Lit.Value.CustomValue
  else
    JS:=Lit.Value.AsString;
  if length(JS)<>1 then
    RaiseNotSupported(ErrorEl,AContext,20171112021003);
  Lit.Value.AsNumber:=ord(JS[1]);
end;

function TPasToJSConverter.ClonePrimaryExpression(El: TJSPrimaryExpression;
  Src: TPasElement): TJSPrimaryExpression;
begin
  Result:=TJSPrimaryExpression(CreateElement(TJSElementClass(El.ClassType),Src));
  if Result.ClassType=TJSPrimaryExpressionIdent then
    TJSPrimaryExpressionIdent(Result).Name:=TJSPrimaryExpressionIdent(El).Name;
end;

function TPasToJSConverter.CreateMulNumber(El: TPasElement; JS: TJSElement;
  n: TMaxPrecInt): TJSElement;
// create JS*n
var
  Mul: TJSMultiplicativeExpressionMul;
  Value: TJSValue;
begin
  if JS is TJSLiteral then
    begin
    Value:=TJSLiteral(JS).Value;
    case Value.ValueType of
      jstUNDEFINED:
        begin
        // undefined * number  ->  NaN
        Value.AsNumber:=NaN;
        exit(JS);
        end;
      jstNull:
        begin
        // null*number -> 0
        Value.AsNumber:=0;
        exit(JS);
        end;
      jstBoolean:
        begin
        // true is 1, false is 0
        if Value.AsBoolean then
          Value.AsNumber:=n
        else
          Value.AsNumber:=0;
        exit(JS);
        end;
      jstNumber:
        if IsNan(Value.AsNumber) or IsInfinite(Value.AsNumber) then
        else
          begin
          Value.AsNumber:=Value.AsNumber*n;
          exit(JS);
          end;
    end;
    end;
  Mul:=TJSMultiplicativeExpressionMul(CreateElement(TJSMultiplicativeExpressionMul,El));
  Result:=Mul;
  Mul.A:=JS;
  Mul.B:=CreateLiteralNumber(El,n);
end;

function TPasToJSConverter.CreateDivideNumber(El: TPasElement; JS: TJSElement;
  n: TMaxPrecInt): TJSElement;
// create JS/n
var
  Mul: TJSMultiplicativeExpressionDiv;
  Value: TJSValue;
begin
  if (n<>0) and (JS is TJSLiteral) then
    begin
    Value:=TJSLiteral(JS).Value;
    case Value.ValueType of
      jstUNDEFINED:
        begin
        // undefined / number  ->  NaN
        Value.AsNumber:=NaN;
        exit(JS);
        end;
      jstNull:
        begin
        // null / number -> 0
        Value.AsNumber:=0;
        exit(JS);
        end;
      jstBoolean:
        begin
        // true is 1, false is 0
        if Value.AsBoolean then
          Value.AsNumber:=1/n
        else
          Value.AsNumber:=0;
        exit(JS);
        end;
      jstNumber:
        if IsNan(Value.AsNumber) or IsInfinite(Value.AsNumber) then
        else
          begin
          Value.AsNumber:=Value.AsNumber / n;
          exit(JS);
          end;
    end;
    end;
  Mul:=TJSMultiplicativeExpressionDiv(CreateElement(TJSMultiplicativeExpressionDiv,El));
  Result:=Mul;
  Mul.A:=JS;
  Mul.B:=CreateLiteralNumber(El,n);
end;

function TPasToJSConverter.CreateTruncFloor(El: TPasElement; JS: TJSElement;
  FloorAndCeil: boolean): TJSElement;
// create Math.floor(JS)
var
  Value: TJSValue;
  Call: TJSCallExpression;
begin
  if JS is TJSLiteral then
    begin
    Value:=TJSLiteral(JS).Value;
    case Value.ValueType of
      jstUNDEFINED:
        begin
        // Math.floor(undefined)  ->  NaN
        Value.AsNumber:=NaN;
        exit(JS);
        end;
      jstNull:
        begin
        // Math.floor(null) -> 0
        Value.AsNumber:=0;
        exit(JS);
        end;
      jstBoolean:
        begin
        // true is 1, false is 0
        if Value.AsBoolean then
          Value.AsNumber:=1
        else
          Value.AsNumber:=0;
        exit(JS);
        end;
      jstNumber:
        begin
        if IsNan(Value.AsNumber) or IsInfinite(Value.AsNumber) then
          exit(JS);
        if FloorAndCeil then
          Value.AsNumber:=Trunc(Value.AsNumber)
        else
          Value.AsNumber:=Floor(Value.AsNumber);
        exit(JS);
        end;
    end;
    end;
  Call:=CreateCallExpression(El);
  Result:=Call;
  if FloorAndCeil then
    Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnTrunc),El)
  else
    Call.Expr:=CreatePrimitiveDotExpr('Math.floor',El);
  Call.AddArg(JS);
end;

function TPasToJSConverter.CreateDotNameExpr(PosEl: TPasElement;
  MExpr: TJSElement; const aName: TJSString): TJSDotMemberExpression;
begin
  Result:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,PosEl));
  Result.MExpr:=MExpr;
  Result.Name:=aName;
end;

function TPasToJSConverter.CreateDotExpression(aParent: TPasElement; Left,
  Right: TJSElement; CheckRightIntfRef: boolean): TJSElement;
var
  Dot: TJSDotMemberExpression;
  RightParent, Expr: TJSElement;
  ok: Boolean;
  Call: TJSCallExpression;
begin
  Result:=nil;
  if Left=nil then
    RaiseInconsistency(20170201140827,aParent);
  if Right=nil then
    RaiseInconsistency(20170211192018,aParent);

  if CheckRightIntfRef and IsInterfaceRef(Right) then
    begin
    // right was an implicit call
    // convert "$ir.ref(id,Expr)"  ->  $ir.ref(id,Left.Expr)
    Call:=TJSCallExpression(Right);
    Expr:=Call.Args.Elements[1].Expr;
    Call.Args.Elements[1].Expr:=CreateDotExpression(aParent,Left,Expr);
    Result:=Call;
    exit;
    end;

  ok:=false;
  try
    // create a TJSDotMemberExpression of Left and the left-most identifier of Right
    // Left becomes the new left-most element of Right.
    Result:=Right;
    RightParent:=nil;
    repeat
      if (Right.ClassType=TJSCallExpression) then
        begin
        RightParent:=Right;
        Right:=TJSCallExpression(Right).Expr;
        if Right=nil then
          begin
          // left-most is nil -> insert Left
          TJSCallExpression(RightParent).Expr:=Left;
          break;
          end;
        end
      else if (Right.ClassType=TJSBracketMemberExpression) then
        begin
        RightParent:=Right;
        Right:=TJSBracketMemberExpression(Right).MExpr;
        if Right=nil then
          begin
          // left-most is nil -> insert Left
          TJSBracketMemberExpression(RightParent).MExpr:=Left;
          break;
          end;
        end
      else if (Right.ClassType=TJSDotMemberExpression) then
        begin
        RightParent:=Right;
        Right:=TJSDotMemberExpression(Right).MExpr;
        if Right=nil then
          begin
          // left-most is nil -> insert Left
          TJSDotMemberExpression(RightParent).MExpr:=Left;
          break;
          end;
        end
      else if (Right.ClassType=TJSPrimaryExpressionIdent) then
        begin
        // left-most identifier found
        // -> replace it
        Dot := TJSDotMemberExpression(CreateElement(TJSDotMemberExpression, aParent));
        if Result=Right then
          Result:=Dot
        else if RightParent is TJSBracketMemberExpression then
          TJSBracketMemberExpression(RightParent).MExpr:=Dot
        else if RightParent is TJSCallExpression then
          TJSCallExpression(RightParent).Expr:=Dot
        else if RightParent is TJSDotMemberExpression then
          TJSDotMemberExpression(RightParent).MExpr:=Dot
        else
          begin
          Dot.Free;
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.CreateDotExpression Right=',GetObjName(Right),' RightParent=',GetObjName(RightParent),' Result=',GetObjName(Result));
          {$ENDIF}
          RaiseInconsistency(20170129141307,aParent);
          end;
        Dot.MExpr := Left;
        Dot.Name := TJSPrimaryExpressionIdent(Right).Name;
        FreeAndNil(Right);
        break;
        end
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('CreateDotExpression Right=',Right.ClassName);
        {$ENDIF}
        DoError(20161024191240,nMemberExprMustBeIdentifier,sMemberExprMustBeIdentifier,[],aParent);
        end;
    until false;

    ok:=true;
  finally
    if not ok then
      begin
      Left.Free;
      FreeAndNil(Result);
      end;
  end;
end;

function TPasToJSConverter.CreateOverflowCheckCall(GetExpr: TJSElement;
  PosEl: TPasElement): TJSCallExpression;
var
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(PosEl);
  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnOverflowCheckInt),PosEl);
  Call.AddArg(GetExpr);
  Result:=Call;
end;

function TPasToJSConverter.CreateRangeCheckCall(GetExpr: TJSElement; MinVal,
  MaxVal: TMaxPrecInt; RTLFunc: TPas2JSBuiltInName; PosEl: TPasElement
  ): TJSCallExpression;
var
  Call: TJSCallExpression;
begin
  Call:=CreateCallExpression(PosEl);
  Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(RTLFunc),PosEl);
  Call.AddArg(GetExpr);
  Call.AddArg(CreateLiteralNumber(PosEl,MinVal));
  Call.AddArg(CreateLiteralNumber(PosEl,MaxVal));
  Result:=Call;
end;

function TPasToJSConverter.CreateRangeCheckCall_TypeRange(aType: TPasType;
  GetExpr: TJSElement; AContext: TConvertContext; PosEl: TPasElement
  ): TJSCallExpression;
var
  Value: TResEvalValue;
begin
  Result:=nil;
  Value:=AContext.Resolver.EvalTypeRange(aType,[refConst]);
  try
    if Value=nil then
      RaiseNotSupported(PosEl,AContext,20180424111936,'range checking '+GetObjName(aType));
    case Value.Kind of
    revkRangeInt:
      case TResEvalRangeInt(Value).ElKind of
        revskEnum, revskInt:
          Result:=CreateRangeCheckCall(GetExpr,TResEvalRangeInt(Value).RangeStart,
            TResEvalRangeInt(Value).RangeEnd,pbifnRangeCheckInt,PosEl);
        revskChar:
          Result:=CreateRangeCheckCall(GetExpr,TResEvalRangeInt(Value).RangeStart,
            TResEvalRangeInt(Value).RangeEnd,pbifnRangeCheckChar,PosEl);
        revskBool: ; // range check not needed
      else
        RaiseNotSupported(PosEl,AContext,20190220002007,'range checking '+Value.AsDebugString);
      end;
    else
      RaiseNotSupported(PosEl,AContext,20180424112010,'range checking '+Value.AsDebugString);
    end;
  finally
    ReleaseEvalValue(Value);
    if Result=nil then
      GetExpr.Free;
  end;
end;

function TPasToJSConverter.CreateReferencePath(El: TPasElement;
  AContext: TConvertContext; Kind: TRefPathKind; Full: boolean;
  Ref: TResolvedReference): string;
{ Notes:
 - local var, argument or result variable, even higher lvl does not need a reference path
   local vars are also argument, result var, result variable
 - with context uses the local $withnnn var
 - auto created local var
 otherwise use absolute path
}
var
  aResolver: TPas2JSResolver;

  function IsLocalVar: boolean;
  begin
    Result:=false;
    if El.ClassType=TPasArgument then
      exit(true);
    if El.ClassType=TPasResultElement then
      exit(true);
    if aResolver=nil then
      exit(true);
    if El.Parent=nil then
      RaiseNotSupported(El,AContext,20170203121306,GetObjName(El));
    if El.Parent.ClassType=TPasImplExceptOn then
      exit(true);
    if not (El.Parent is TProcedureBody) then exit;
    Result:=true;
  end;

  procedure Prepend(var aPath: string; Prefix: string);
  begin
    if (aPath<>'') and (aPath[1]<>'[') then
      aPath:='.'+aPath;
    aPath:=Prefix+aPath;
  end;

  procedure PrependClassOrRecName(var Path: string; ClassOrRec: TPasMembersType);
  begin
    if (ClassOrRec.ClassType=TPasClassType) and TPasClassType(ClassOrRec).IsExternal then
      Prepend(Path,TPasClassType(ClassOrRec).ExternalName)
    else
      Prepend(Path,CreateGlobalTypePath(ClassOrRec,AContext));
  end;

  function NeedsWithExpr: boolean;
  var
    Parent: TPasElement;
  begin
    if (Ref=nil) or (Ref.WithExprScope=nil) then exit(false);
    Parent:=El.Parent;
    if (Parent.ClassType=TPasClassType)
        and (TPasClassType(Parent).HelperForType<>nil) then
      begin
      // e.g. with Obj do HelperMethod
      if aResolver.IsHelperForMember(El) then
        // e.g. with Obj do HelperExternalMethod  -> Obj.HelperCall
      else
        // e.g. with Obj do HelperMethod  -> THelper.HelperCall
        exit(false);
      end;
    Result:=true;
  end;

  function ProcSelfIsInstance(Proc: TPasElement): boolean;
  var
    C: TClass;
  begin
    if Proc=nil then exit(false);
    C:=Proc.ClassType;
    Result:=(C=TPasFunction) or (C=TPasProcedure) or (C=TPasConstructor) or (C=TPasDestructor);
  end;

  procedure Append_GetClass(Member: TPasElement);
  var
    P: TPasElement;
  begin
    P:=Member.Parent;
    if P=nil then
      RaiseNotSupported(Member,AContext,20191018125004);
    if P.ClassType=TPasClassType then
      begin
      if TPasClassType(P).IsExternal then
        exit;
      if Result<>'' then
        Result:=Result+'.'+GetBIName(pbivnPtrClass)
      else
        Result:=GetBIName(pbivnPtrClass);
      end
    else if P.ClassType=TPasRecordType then
      begin
      if Result<>'' then
        Result:=Result+'.'+GetBIName(pbivnPtrRecord)
      else
        Result:=GetBIName(pbivnPtrRecord);
      end
    else
      RaiseNotSupported(Member,AContext,20190106110525);
  end;

  function GetAbsoluteAlias: string;
  var
    AbsolResolved: TPasResolverResult;
  begin
    aResolver.ComputeElement(TPasVariable(El).AbsoluteExpr,AbsolResolved,[rcNoImplicitProc]);
    Result:=CreateReferencePath(AbsolResolved.IdentEl,AContext,Kind,Full,Ref);
  end;

  function ImplToDecl(El: TPasElement): TPasElement;
  var
    ProcScope: TPasProcedureScope;
  begin
    Result:=El;
    if El.CustomData is TPasProcedureScope then
      begin
      // proc: always use the declaration, not the body
      ProcScope:=TPasProcedureScope(El.CustomData);
      if ProcScope.DeclarationProc<>nil then
        Result:=ProcScope.DeclarationProc;
      end;
  end;

  function IsA(SrcType, DstType: TPasType): boolean;
  var
    C: TClass;
  begin
    while SrcType<>nil do
      begin
      if SrcType=DstType then exit(true);
      C:=SrcType.ClassType;
      if C=TPasClassType then
        SrcType:=TPas2JSClassScope(SrcType.CustomData).DirectAncestor
      else if (C=TPasAliasType)
          or (C=TPasTypeAliasType) then
        SrcType:=TPasAliasType(SrcType).DestType
      else if C=TPasSpecializeType then
        begin
        if SrcType.CustomData is TPasSpecializeTypeData then
          SrcType:=TPasSpecializeTypeData(SrcType.CustomData).SpecializedType
        else
          RaiseInconsistency(20191027172642,SrcType);
        end
      else
        exit(false);
      end;
    Result:=false;
  end;

  function ShortRefGlobal: boolean;
  var
    ElClass: TClass;
    Proc: TPasProcedure;
  begin
    ElClass:=El.ClassType;
    if ElClass.InheritsFrom(TPasType) then
      begin
      if El.Parent.ClassType=TProcedureBody then
        exit(false);
      CreateReferencePath:=CreateGlobalTypePath(TPasType(El),AContext);
      exit(true);
      end
    else if ElClass.InheritsFrom(TPasProcedure) then
      begin
      Proc:=TPasProcedure(El);
      if ProcCanHaveShortRef(Proc) then
        begin
        if aResolver.ProcHasSelf(Proc) then
          begin
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.CreateReferencePath El=',GetObjName(El),' Parent=',GetObjName(El.Parent),' Kind=',Kind,' Context=',GetObjName(AContext),' SelfContext=',GetObjName(AContext.GetSelfContext));
          {$ENDIF}
          aResolver.RaiseNotYetImplemented(20201030233511,El);
          end;
        CreateReferencePath:=CreateStaticProcPath(Proc,AContext);
        exit(true);
        end;
      end
    else if (ElClass=TPasEnumValue) then
      begin
      CreateReferencePath:=CreateGlobalElPath(El,AContext);
      exit(true);
      end;
    Result:=false;
  end;

var
  FoundModule: TPasModule;
  ParentEl, CurEl: TPasElement;
  Dot: TDotContext;
  WithData: TPas2JSWithExprScope;
  ShortName: String;
  SelfContext: TFunctionContext;
  ElClass: TClass;
  IsClassRec: Boolean;
  VarKinds: TCtxVarKinds;
  Proc: TPasProcedure;
begin
  Result:='';
  {$IFDEF VerbosePas2JS}
  //writeln('TPasToJSConverter.CreateReferencePath START El=',GetObjName(El),' Parent=',GetObjName(El.Parent),' Context=',GetObjName(AContext),' SelfContext=',GetObjName(AContext.GetSelfContext));
  //AContext.WriteStack;
  {$ENDIF}
  aResolver:=AContext.Resolver;
  if (El is TPasType) and (AContext<>nil) then
    El:=aResolver.ResolveAliasType(TPasType(El));

  ElClass:=El.ClassType;
  if ElClass.InheritsFrom(TPasVariable) and (TPasVariable(El).AbsoluteExpr<>nil)
      and (AContext.Resolver<>nil) then
    exit(GetAbsoluteAlias);

  if AContext is TDotContext then
    begin
    Dot:=TDotContext(AContext);
    if aResolver<>nil then
      begin
      if ElClass.InheritsFrom(TPasVariable) then
        begin
        //writeln('TPasToJSConverter.CreateReferencePath Left=',GetResolverResultDbg(Dot.LeftResolved),' Right=class var ',GetObjName(El));
        if ([vmClass,vmStatic]*ClassVarModifiersType*TPasVariable(El).VarModifiers<>[])
            and (Dot.Access=caAssign)
            and aResolver.ResolvedElIsClassOrRecordInstance(Dot.LeftResolved) then
          begin
          // writing a class var or class const
          Append_GetClass(El);
          end;
        end
      else if aResolver.IsMethod_SelfIsClass(El)
          and aResolver.ResolvedElIsClassOrRecordInstance(Dot.LeftResolved) then
        // accessing a class method from an object, 'this' must be the class/record
        Append_GetClass(El);
      end;
    end
  else if IsLocalVar then
    begin
    // El is local var -> does not need path
    end
  else if ElClass.InheritsFrom(TPasProcedure)
      and (TPasProcedure(El).LibrarySymbolName<>nil)
      and not (El.Parent is TPasMembersType) then
    begin
    // an external global function -> use the literal
    if Kind=rpkPathAndName then
      Result:=ComputeConstString(TPasProcedure(El).LibrarySymbolName,AContext,true)
    else
      Result:='';
    exit;
    end
  else if ElClass.InheritsFrom(TPasVariable) and (TPasVariable(El).ExportName<>nil)
      and not (El.Parent is TPasMembersType) then
    begin
    // an external global var -> use the literal
    if Kind=rpkPathAndName then
      Result:=ComputeConstString(TPasVariable(El).ExportName,AContext,true)
    else
      Result:='';
    exit;
    end
  else if (ElClass=TPasClassType) and TPasClassType(El).IsExternal then
    begin
    // an external class -> use the literal
    Result:=TPasClassType(El).ExternalName;
    exit;
    end
  else if NeedsWithExpr then
    begin
    // using local WITH var
    WithData:=Ref.WithExprScope as TPas2JSWithExprScope;
    if WithData.WithVarName='' then
      RaiseNotSupported(WithData.Expr,AContext,20190209092506,GetObjName(El));
    Prepend(Result,WithData.WithVarName);
    if not (wesfOnlyTypeMembers in WithData.Flags)
        and aResolver.IsMethod_SelfIsClass(El) then
      begin
      // with Obj do NonStaticClassMethod -> append .$class
      Append_GetClass(El);
      end;
    end
  else
    begin
    // neither Dot nor With context, nor local, nor external,
    // -> translate a Pascal identifier to the JS path
    //  Examples: this.name, $Self.name, this.sub.name, globalpath.name
    if El.Parent=nil then
      RaiseNotSupported(El,AContext,20170201172141,GetObjName(El));

    if (coShortRefGlobals in Options) and (Kind=rpkPathAndName) then
      begin
      if ShortRefGlobal then exit;
      end;

    El:=ImplToDecl(El);

    CurEl:=El;
    repeat
      ParentEl:=CurEl.Parent;
      if ParentEl=nil then break;
      if ParentEl is TProcedureBody then break;
      ParentEl:=ImplToDecl(ParentEl);

      IsClassRec:=(ParentEl.ClassType=TPasClassType)
               or (ParentEl.ClassType=TPasRecordType);

      if IsClassRec then
        begin
        // Not in a Pascal dotscope and accessing a class member.
        // Possible results: this.v, module.path.path.v, this.$class.v, $Self.v
        //    In nested proc 'this' can have another name, e.g. '$Self'
        if (ParentEl.ClassType=TPasClassType)
            and (TPasClassType(ParentEl).HelperForType<>nil) then
          begin
          if (El=CurEl)
              and aResolver.IsHelperForMember(CurEl) then
            begin
            // external helper proc/var -> redirect to helper-for-type
            ParentEl:=aResolver.ResolveAliasType(TPasClassType(ParentEl).HelperForType);
            IsClassRec:=(ParentEl.ClassType=TPasClassType)
                     or (ParentEl.ClassType=TPasRecordType);
            if not IsClassRec then
              RaiseNotSupported(El,AContext,20190926091356);
            end
          else
            begin
            // helper members cannot be accessed via "this"
            PrependClassOrRecName(Result,TPasMembersType(ParentEl));
            break;
            end;
          end;

        if Full then
          begin
          PrependClassOrRecName(Result,TPasMembersType(ParentEl));
          break;
          end;

        if El is TPasVariable then
          begin
          if TPasVariable(El).VarModifiers*[vmClass, vmStatic]<>[] then
            VarKinds:=[cvkGlobal,cvkCurType,cvkInstance]
          else
            VarKinds:=[cvkInstance];
          end
        else if El is TPasProcedure then
          begin
          Proc:=TPasProcedure(El);
          if ProcSelfIsInstance(Proc) then
            VarKinds:=[cvkCurType,cvkInstance]
          else
            VarKinds:=[cvkGlobal,cvkCurType,cvkInstance];
          end
        else
          VarKinds:=[cvkGlobal,cvkCurType,cvkInstance];
        if VarKinds<>[cvkGlobal] then
          begin
          // Pascal uses implicit Self -> use "this" if available
          SelfContext:=AContext.GetSelfContext;
          if (SelfContext<>nil)
              and IsA(TPasType(SelfContext.ThisVar.Element),TPasMembersType(ParentEl)) then
            begin
            ShortName:=GetLocalName(SelfContext.ThisVar.Element,VarKinds,AContext);
            if ShortName='' then
              begin
              if not (cvkGlobal in VarKinds) then
                begin
                {$IFDEF VerbosePas2JS}
                {AllowWriteln}
                AContext.WriteStack;
                writeln('TPasToJSConverter.CreateReferencePath SelfContext.ThisVar=',GetObjPath(SelfContext.ThisVar.Element),' El=',GetObjPath(El));
                {AllowWriteln-}
                {$ENDIF}
                RaiseNotSupported(El,AContext,20200920214421);
                end;
              // e.g. inside a static function inside a record accessing a class var of the record
              PrependClassOrRecName(Result,TPasMembersType(ParentEl));
              break;
              end;
            if ProcSelfIsInstance(SelfContext.PasElement) then
              begin
              // inside a method -> Self is a class instance
              if aResolver.IsMethod_SelfIsClass(El) then
                Append_GetClass(El); // accessing a class function -> this.$class.procname
              end;
            Prepend(Result,ShortName);
            break;
            end;
          end;

        ShortName:=GetLocalName(ParentEl,VarKinds,AContext);
        //writeln('TPasToJSConverter.CreateReferencePath NOT USING SELF ',GetObjPath(El),' ShortName=',ShortName);
        if ShortName<>'' then
          begin
          Prepend(Result,ShortName);
          break;
          end
        else if (ParentEl.ClassType=TPasClassType) and TPasClassType(ParentEl).IsExternal then
          begin
          Prepend(Result,TPasClassType(ParentEl).ExternalName);
          break;
          end
        else if coShortRefGlobals in Options then
          begin
          PrependClassOrRecName(Result,TPasMembersType(ParentEl));
          break;
          end
        else
          begin
          ShortName:=TransformElToJSName(ParentEl,AContext);
          Prepend(Result,ShortName);
          end;
        end
      else
        begin
        // check if ParentEl has a JS var
        ShortName:=GetLocalName(ParentEl,[cvkGlobal],AContext);
        if (ShortName<>'') then
          begin
          Prepend(Result,ShortName);
          break;
          end
        else if ParentEl.ClassType=TImplementationSection then
          begin
          // element is in an implementation section (not program/library section)
          // in other unit -> use pas.unitname.$impl
          FoundModule:=ParentEl.GetModule;
          if FoundModule=nil then
            RaiseInconsistency(20161024192755,El);
          Prepend(Result,TransformModuleName(FoundModule,true,AContext)
             +'.'+GetBIName(pbivnImplementation));
          break;
          end
        else if ParentEl is TPasModule then
          begin
          // element is in an unit interface or program/library section
          Prepend(Result,TransformModuleName(TPasModule(ParentEl),true,AContext));
          break;
          end
        else if ParentEl.ClassType=TPasEnumType then
          begin
          Prepend(Result,ParentEl.Name);
          end;
        end;
      CurEl:=ParentEl;
      until false;
    end;

  case Kind of
  rpkPathWithDot:
    if Result<>'' then Result:=Result+'.';
  rpkPathAndName:
    begin
    if (coShortRefGlobals in Options) then
      if ShortRefGlobal then exit;
    ShortName:=TransformElToJSName(El,AContext);
    if Result='' then
      Result:=ShortName
    else if (ShortName<>'') and (ShortName[1] in ['[','(']) then
      Result:=Result+ShortName
    else
      Result:=Result+'.'+ShortName;
    end;
  end;
end;

function TPasToJSConverter.CreateReferencePathExpr(El: TPasElement;
  AContext: TConvertContext; Full: boolean; Ref: TResolvedReference
  ): TJSElement;
var
  Name: String;
  Src: TPasElement;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.CreateReferencePathExpr El="',GetObjName(El),'" El.Parent=',GetObjName(El.Parent),' ',GetObjName(AContext));
  {$ENDIF}
  Name:=CreateReferencePath(El,AContext,rpkPathAndName,Full,Ref);
  if Ref<>nil then
    Src:=Ref.Element
  else
    Src:=nil;
  Result:=CreatePrimitiveDotExpr(Name,Src);
end;

function TPasToJSConverter.CreateGlobalTypePath(El: TPasType;
  AContext: TConvertContext): string;
var
  aType: TPasType;
begin
  aType:=AContext.Resolver.ResolveAliasType(El);
  Result:=CreateGlobalElPath(aType,AContext);
end;

function TPasToJSConverter.CreateStaticProcPath(El: TPasProcedure;
  AContext: TConvertContext): string;
begin
  if El.IsAbstract or El.IsExternal then
    RaiseNotSupported(El,AContext,20201101185117)
  else if El.IsStatic
      or (El.Parent is TPasSection)
      or (TPas2JSProcedureScope(El.CustomData).SpecializedFromItem<>nil) then
    Result:=CreateGlobalElPath(El,AContext)
  else
    RaiseNotSupported(El,AContext,20200925104007);
end;

function TPasToJSConverter.CreateGlobalElPath(El: TPasElement;
  AContext: TConvertContext): string;
var
  ShortRefGlobals: Boolean;
  Parent: TPasElement;
  CurModule: TPasModule;
  ElClass: TClass;
  aResolver: TPas2JSResolver;
begin
  aResolver:=AContext.Resolver;
  Result:=AContext.GetLocalName(El,[cvkGlobal]);
  if Result<>'' then
    begin
    // already exists
    if coStoreImplJS in Options then
      StoreImplJSLocal(El,AContext);
    exit;
    end;
  ShortRefGlobals:=coShortRefGlobals in Options;

  Parent:=El.Parent;
  if Parent<>nil then
    begin
    Result:=AContext.GetLocalName(Parent,[cvkGlobal]);
    if Result='' then
      begin
      ElClass:=Parent.ClassType;
      if ElClass.InheritsFrom(TPasType) then
        Result:=CreateGlobalElPath(Parent,AContext)
      else if ElClass.InheritsFrom(TPasSection) then
        begin
        // element is in foreign unit -> use pas.unitname
        CurModule:=Parent.GetModule;
        Result:=TransformModuleName(CurModule,true,AContext);
        if (Parent.ClassType=TImplementationSection)
            and (CurModule<>AContext.GetRootContext.PasElement.GetModule) then
          begin
          // element is in foreign implementation section (not program/library section)
          // -> use pas.unitname.$impl
          Result:=Result+'.'+GetBIName(pbivnImplementation);
          end;
        end
      else if ElClass.InheritsFrom(TPasModule) then
        Result:=TransformModuleName(TPasModule(Parent),true,AContext)
      else
        RaiseNotSupported(El,AContext,20200609230526,GetObjPath(El));
      end
    else
      begin
      // parent has local var
      if (coStoreImplJS in Options) and (aResolver.GetParentProcBody(Parent)=nil) then
        StoreImplJSLocal(Parent,AContext);
      end;
    Result:=Result+'.'+TransformElToJSName(El,AContext);
    end
  else
    begin
    if El is TPasModule then
      begin
      Result:=TransformModuleName(TPasModule(El),true,AContext);
      exit; // already created a shortrefglobal
      end
    else
      RaiseNotSupported(El,AContext,20201010221704,GetObjPath(El));
    end;
  if ShortRefGlobals then
    Result:=CreateGlobalAliasForeign(El,Result,AContext);
end;

function TPasToJSConverter.GetLocalName(El: TPasElement;
  const Filter: TCtxVarKinds; AContext: TConvertContext): string;
begin
  if coStoreImplJS in Options then
    begin
    if cvkGlobal in Filter then
      begin
      Result:=AContext.GetLocalName(El,[cvkGlobal]);
      if Result<>'' then
        begin
        StoreImplJSLocal(El,AContext);
        exit;
        end
      else if Filter=[cvkGlobal] then
        exit('');
      end;
    end;
  Result:=AContext.GetLocalName(El,Filter);
end;

function TPasToJSConverter.ProcCanHaveShortRef(Proc: TPasProcedure): boolean;
var
  C: TClass;
begin
  // can not:
  if Proc.IsExternal or Proc.IsVirtual then
    exit(false);
  C:=Proc.Parent.ClassType;
  if C=TProcedureBody then
    exit(false);

  // can:
  if C.InheritsFrom(TPasSection) then
    exit(true);
  if Proc.IsStatic then
    exit(true);
  if TPas2JSProcedureScope(Proc.CustomData).SpecializedFromItem<>nil then
    exit(true);

  Result:=false;
end;

procedure TPasToJSConverter.StoreImplJSLocal(El: TPasElement;
  AContext: TConvertContext);
var
  Ctx: TConvertContext;
  CurEl: TPasElement;
  Data: TObject;
  ImplJS: TPas2JSPrecompiledJS;
begin
  Ctx:=AContext;
  while Ctx<>nil do
    begin
    CurEl:=Ctx.PasElement;
    if CurEl<>nil then
      begin
      Data:=CurEl.CustomData;
      if Data is TPas2JSProcedureScope then
        begin
        ImplJS:=TPas2JSProcedureScope(Data).ImplJS;
        if ImplJS<>nil then
          ImplJS.AddShortRef(El);
        end
      else if Data is TPas2JSInitialFinalizationScope then
        begin
        ImplJS:=TPas2JSInitialFinalizationScope(Data).ImplJS;
        if ImplJS<>nil then
          ImplJS.AddShortRef(El);
        end;
      end;
    Ctx:=Ctx.Parent;
    end;
end;

procedure TPasToJSConverter.StoreImplJSLocals(ModScope: TPas2JSModuleScope;
  IntfContext: TSectionContext);
var
  i, StoredIndex: Integer;
  CtxVar: TFCLocalIdentifier;
  StoredVar: TPas2JSStoredLocalVar;
  CurName: String;
begin
  ModScope.ClearStoreJSLocalVars;
  SetLength(ModScope.StoreJSLocalVars,length(IntfContext.LocalVars));
  StoredIndex:=0;
  for i:=0 to length(IntfContext.LocalVars)-1 do
    begin
    CtxVar:=IntfContext.LocalVars[i];
    if (CtxVar.Element=nil) or (CtxVar.Kind<>cvkGlobal) then
      continue;
    if CtxVar.Element.Parent is TProcedureBody then
      continue;
    CurName:=CtxVar.Name;
    if (CurName='') or (CurName='this')
        or (CurName=GetBIName(pbivnModule))
        or (CurName=GetBIName(pbivnImplementation))
      then continue;
    StoredVar:=TPas2JSStoredLocalVar.Create;
    StoredVar.Name:=CurName;
    StoredVar.Element:=CtxVar.Element;
    ModScope.StoreJSLocalVars[StoredIndex]:=StoredVar;
    inc(StoredIndex);
    end;
  SetLength(ModScope.StoreJSLocalVars,StoredIndex);
end;

procedure TPasToJSConverter.RestoreImplJSLocals(ModScope: TPas2JSModuleScope;
  IntfContext: TSectionContext);
begin
  IntfContext.PrecompiledVars:=ModScope.StoreJSLocalVars;
end;

procedure TPasToJSConverter.CreateProcedureCall(var Call: TJSCallExpression;
  Args: TParamsExpr; TargetProc: TPasProcedureType; AContext: TConvertContext);
// create a call, adding call by reference and default values
begin
  if Call=nil then
    Call:=TJSCallExpression(CreateElement(TJSCallExpression,Args));
  if ((Args=nil) or (length(Args.Params)=0))
      and ((TargetProc=nil) or (TargetProc.Args.Count=0)) then
    exit;
  if Call.Args=nil then
    Call.Args:=TJSArguments(CreateElement(TJSArguments,Args));
  CreateProcedureCallArgs(Call.Args.Elements,Args,TargetProc,AContext);
end;

procedure TPasToJSConverter.CreateProcedureCallArgs(
  Elements: TJSArrayLiteralElements; Args: TParamsExpr;
  TargetProc: TPasProcedureType; AContext: TConvertContext);
// Add call arguments. Handle call by reference and default values
var
  ArgContext: TConvertContext;
  i: Integer;
  Arg: TJSElement;
  TargetArgs: TFPList;
  TargetArg: TPasArgument;
  OldAccess: TCtxAccess;
begin
  // get context
  ArgContext:=AContext.GetNonDotContext;
  i:=0;
  OldAccess:=ArgContext.Access;
  if TargetProc<>nil then
    TargetArgs:=TargetProc.Args
  else
    TargetArgs:=nil;
  // add params
  if Args<>nil then
    while i<length(Args.Params) do
      begin
      if (TargetArgs<>nil) and (i<TargetArgs.Count) then
        TargetArg:=TPasArgument(TargetArgs[i])
      else
        TargetArg:=nil;
      Arg:=CreateProcCallArg(Args.Params[i],TargetArg,ArgContext);
      Elements.AddElement.Expr:=Arg;
      inc(i);
      end;
  // fill up default values
  if TargetProc<>nil then
    begin
    while i<TargetArgs.Count do
      begin
      TargetArg:=TPasArgument(TargetArgs[i]);
      if TargetArg.ValueExpr=nil then
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateProcedureCallArgs missing default value: i=',i,' TargetProc=',GetObjPath(TargetProc),' Args=',GetObjPath(Args));
        {$ENDIF}
        if Args=nil then
          RaiseNotSupported(TargetProc,AContext,20201028203457)
        else
          RaiseNotSupported(Args,AContext,20170201193601);
        end;
      AContext.Access:=caRead;
      Arg:=ConvertExpression(TargetArg.ValueExpr,ArgContext);
      Elements.AddElement.Expr:=Arg;
      inc(i);
      end;
    end;
  ArgContext.Access:=OldAccess;
end;

function TPasToJSConverter.CreateProcCallArg(El: TPasExpr;
  TargetArg: TPasArgument; AContext: TConvertContext): TJSElement;
var
  ExprIsTemp, ExprIsTempValid: boolean;
  ExprResolved, ArgResolved: TPasResolverResult;

  function ExprIsTemporaryVar: boolean;
  // returns true if Expr is a temporary variable, e.g. a function result
  begin
    if not ExprIsTempValid then
      begin
      ExprIsTempValid:=true;
      ExprIsTemp:=IsExprTemporaryVar(El);
      end;
    Result:=ExprIsTemp;
  end;

var
  ExprFlags: TPasResolverComputeFlags;
  IsRecord, NeedVar, ArgTypeIsArray: Boolean;
  ArgTypeEl, ExprTypeEl: TPasType;
  Call: TJSCallExpression;
  aResolver: TPas2JSResolver;
begin
  Result:=nil;
  if TargetArg=nil then
    begin
    // simple conversion
    AContext.Access:=caRead;
    Result:=ConvertExpression(El,AContext);
    exit;
    end;

  if not (TargetArg.Access in [argDefault,argVar,argOut,argConst,argConstRef]) then
    DoError(20170213220927,nPasElementNotSupported,sPasElementNotSupported,
            [AccessNames[TargetArg.Access]],El);
  aResolver:=AContext.Resolver;

  aResolver.ComputeElement(TargetArg,ArgResolved,[]);
  ArgTypeEl:=ArgResolved.LoTypeEl;
  IsRecord:=ArgTypeEl is TPasRecordType;
  ArgTypeIsArray:=ArgTypeEl is TPasArrayType;
  NeedVar:=(TargetArg.Access in [argVar,argOut]) and not IsRecord;
  ExprFlags:=[];
  if NeedVar then
    Include(ExprFlags,rcNoImplicitProc)
  else if aResolver.IsProcedureType(ArgResolved,true) then
    Include(ExprFlags,rcNoImplicitProcType);

  aResolver.ComputeElement(El,ExprResolved,ExprFlags);
  ExprIsTempValid:=false;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.CreateProcCallArg Arg=',GetResolverResultDbg(ArgResolved),' Expr=',GetResolverResultDbg(ExprResolved));
  {$ENDIF}

  if (TargetArg.ArgType=nil) and (ExprResolved.LoTypeEl is TPasRecordType) then
    NeedVar:=false; // pass aRecord to UntypedArg -> no reference needed

  // consider TargetArg access
  if NeedVar then
    Result:=CreateProcCallArgRef(El,ExprResolved,TargetArg,AContext)
  else
    begin
    // pass as default, const or constref
    AContext.Access:=caRead;

    if ArgTypeIsArray then
      begin
      // array as argument
      if ExprResolved.BaseType=btNil then
        begin
        // nil to array ->  pass []
        Result:=TJSArrayLiteral(CreateElement(TJSArrayLiteral,El));
        exit;
        end
      else if ExprResolved.BaseType in btAllStringAndChars then
        begin
        // pass string to an array
        Result:=ConvertExpression(El,AContext);
        Result:=CreateDotSplit(El,Result);
        end
      else
        Result:=CreateArrayInit(TPasArrayType(ArgTypeEl),El,El,AContext);
      end
    else if ExprResolved.BaseType=btProc then
      begin
      if (ArgTypeEl is TPasProcedureType)
          and (msDelphi in AContext.CurrentModeSwitches)
          and (ExprResolved.IdentEl is TPasProcedure) then
        begin
        // Delphi allows passing a proc address without @
        Result:=CreateCallback(El,ExprResolved,
             TPasProcedureType(ArgTypeEl).CallingConvention=ccSafeCall,
             AContext);
        end;
      end;

    if Result=nil then
      Result:=ConvertExpression(El,AContext);

    if (ExprResolved.BaseType=btSet) and (ExprResolved.IdentEl<>nil) then
      begin
      // pass a set variable
      if TargetArg.Access=argDefault then
        begin
        // pass set with argDefault  -> create reference   rtl.refSet(right)
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateProcCallArg create reference of SET variable Right={',GetResolverResultDbg(ExprResolved),'} AssignContext.RightResolved.IdentEl=',GetObjName(ExprResolved.IdentEl));
        {$ENDIF}
        Result:=CreateReferencedSet(El,Result);
        end;
      end
    else if ArgResolved.BaseType=btCurrency then
      begin
      if ExprResolved.BaseType<>btCurrency then
        begin
        // pass double to currency  ->  *10000
        Result:=CreateMulNumber(El,Result,10000);
        end;
      end
    else if ExprResolved.BaseType=btCurrency then
      begin
      if ArgResolved.BaseType<>btCurrency then
        begin
        // pass currency to noncurrency
        // e.g. pass currency to double  -> /10000
        Result:=CreateDivideNumber(El,Result,10000);
        end;
      end
    else if ExprResolved.BaseType in btAllStrings then
      begin
      if ArgTypeEl=nil then
        // string to untyped
      else if ArgTypeEl.ClassType=TPasRecordType then
        begin
        if aResolver.IsTGUID(TPasRecordType(ArgTypeEl)) then
          begin
          // pass aString to TGuid  ->  rtl.strToGUIDR(aString)
          Call:=CreateCallExpression(El);
          Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfStrToGUIDR),El);
          Call.AddArg(Result);
          Result:=Call;
          end;
        end;
      end
    else if ExprResolved.BaseType=btContext then
      begin
      ExprTypeEl:=ExprResolved.LoTypeEl;
      if (ExprTypeEl.ClassType=TPasArrayType) then
        begin
        if length(TPasArrayType(ExprTypeEl).Ranges)>0 then
          begin
          if (TargetArg.Access=argDefault)
              and not ExprIsTemporaryVar then
            begin
            // pass static array with argDefault  -> clone
            Result:=CreateCloneStaticArray(El,TPasArrayType(ExprTypeEl),Result,AContext);
            end;
          end
        else
          begin
          // pass dyn or open array
          if (TargetArg.Access=argDefault)
              and (ArgResolved.BaseType=btContext)
              and (ArgResolved.LoTypeEl is TPasArrayType)
              and not (ArgResolved.LoTypeEl.Parent is TPasArgument)
              and not ExprIsTemporaryVar then
            begin
            // pass dyn array to argDefault array  -> reference
            Result:=CreateArrayRef(El,Result);
            end;
          end;
        end
      else if ExprTypeEl.ClassType=TPasClassType then
        begin
        if ArgTypeEl=nil then
          // class to untyped
        else if ArgResolved.BaseType in btAllStrings then
          begin
          if TPasClassType(ExprTypeEl).ObjKind=okInterface then
            begin
            // pass IntfVarOrType to string  ->  IntfVarOrType.$guid
            Result:=CreateDotNameExpr(El,Result,
                                           TJSString(GetBIName(pbivnIntfGUID)));
            end;
          end
        else if ArgTypeEl.ClassType=TPasRecordType then
          begin
          if (TPasClassType(ExprTypeEl).ObjKind=okInterface)
              and aResolver.IsTGUID(TPasRecordType(ArgTypeEl)) then
            begin
            // pass IntfTypeOrVar to GUIDRecord  ->  rtl.getIntfGUIDR(IntfTypeOrVar)
            Call:=CreateCallExpression(El);
            Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGetGUIDR),El);
            Call.AddArg(Result);
            Result:=Call;
            if TargetArg.Access=argDefault then
              begin
              // pass record with argDefault ->  "TGuid.$clone(RightRecord)"
              {$IFDEF VerbosePas2JS}
              writeln('TPasToJSConverter.CreateProcCallArg clone RECORD TGuid variable Right={',GetResolverResultDbg(ExprResolved),'} AssignContext.RightResolved.IdentEl=',GetObjName(ExprResolved.IdentEl));
              {$ENDIF}
              Result:=CreateRecordCallClone(El,TPasRecordType(ArgTypeEl),Result,AContext);
              end;
            end
          else
            RaiseNotSupported(El,AContext,20180410160008);
          end
        else if ArgTypeEl.ClassType=TPasClassType then
          case TPasClassType(ExprTypeEl).ObjKind of
          okClass:
            case TPasClassType(ArgTypeEl).ObjKind of
            okClass: ; // pass ClassInstVar to ClassType
            okInterface:
              begin
              // pass ClassInstVar to IntfType
              Call:=CreateCallExpression(El);
              case TPasClassType(ArgTypeEl).InterfaceType of
              citCom:
                begin
                // COM:  $ir.ref(id,rtl.queryIntfT(Expr,IntfType))
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfQueryIntfT)]);
                Call.AddArg(Result);
                Result:=Call;
                Call.AddArg(CreateReferencePathExpr(ArgTypeEl,AContext));
                Call:=CreateIntfRef(Call,AContext,El);
                Result:=Call;
                end;
              citCorba:
                begin
                // CORBA:  rtl.getIntfT(Expr,IntfType)
                Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIntfGetIntfT)]);
                Call.AddArg(Result);
                Result:=Call;
                Call.AddArg(CreateReferencePathExpr(ArgTypeEl,AContext));
                end;
              else
                RaiseNotSupported(El,AContext,20180401230251,InterfaceTypeNames[TPasClassType(ArgTypeEl).InterfaceType]);
              end;
              end
            else
              RaiseNotSupported(El,AContext,20180328134244,ObjKindNames[TPasClassType(ArgTypeEl).ObjKind]);
            end;
          okInterface:
            case TPasClassType(ExprTypeEl).ObjKind of
            okInterface: ; // pass IntfVar to IntfType
            else
              RaiseNotSupported(El,AContext,20180328134305,ObjKindNames[TPasClassType(ArgTypeEl).ObjKind]);
            end;
          else
            RaiseNotSupported(El,AContext,20180328134146,ObjKindNames[TPasClassType(ExprTypeEl).ObjKind]);
          end;
        end
      else if ExprTypeEl.ClassType=TPasRecordType then
        begin
        // right side is a record
        if (ArgResolved.BaseType in btAllStrings)
            and aResolver.IsTGUID(TPasRecordType(ExprTypeEl)) then
          begin
          // pass GuidVar to string  ->  rtl.guidrToStr(GuidVar)
          Call:=CreateCallExpression(El);
          Call.Expr:=CreatePrimitiveDotExpr(GetBIName(pbivnRTL)+'.'+GetBIName(pbifnIntfGuidRToStr),El);
          Call.AddArg(Result);
          Result:=Call;
          exit;
          end;
        if TargetArg.Access=argDefault then
          begin
          // pass record with argDefault ->  "RightRecord.$clone(RightRecord)"
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.CreateProcCallArg clone RECORD variable Right={',GetResolverResultDbg(ExprResolved),'} AssignContext.RightResolved.IdentEl=',GetObjName(ExprResolved.IdentEl));
          {$ENDIF}
          Result:=CreateRecordCallClone(El,TPasRecordType(ExprTypeEl),Result,AContext);
          end;
        end
      else if (ExprResolved.LoTypeEl is TPasProcedureType)
          and (ArgResolved.LoTypeEl is TPasProcedureType)
          and (TPasProcedureType(ArgResolved.LoTypeEl).CallingConvention=ccSafeCall) then
        begin
        // pass proc to SafeCall proc type
        Result:=CreateSafeCallback(El,Result,AContext);
        end;
      end;
    end;
end;

function TPasToJSConverter.CreateProcCallArgRef(El: TPasExpr;
  ResolvedEl: TPasResolverResult; TargetArg: TPasArgument;
  AContext: TConvertContext): TJSElement;
var
  Obj: TJSObjectLiteral;

  procedure AddVar(const aName: string; var Expr: TJSElement);
  var
    ObjLit: TJSObjectLiteralElement;
  begin
    if Expr=nil then exit;
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TJSString(aName);
    ObjLit.Expr:=Expr;
    Expr:=nil;
  end;

  function IfReadOnlyCreateRaiseE(const ParamContext: TParamContext): TJSElement;
  begin
    if not (rrfWritable in ResolvedEl.Flags) then
      begin
      FreeAndNil(ParamContext.Setter);
      ParamContext.Setter:=CreateRaisePropReadOnly(El);
      end;
    Result:=ParamContext.Setter;
  end;

  function CreateRgCheck(const SetterArgName: string): TJSElement;

    function CreateRgCheckSt(aType: TPasType): TJSElement;
    begin
      Result:=CreateRangeCheckCall_TypeRange(aType,
        CreatePrimitiveDotExpr(SetterArgName,El),AContext,El);
    end;

  var
    ArgResolved: TPasResolverResult;
    TypeEl: TPasType;
  begin
    Result:=nil;
    if TargetArg.ArgType=nil then exit;
    AContext.Resolver.ComputeElement(TargetArg,ArgResolved,[]);
    TypeEl:=ArgResolved.LoTypeEl;
    if TypeEl=nil then exit;
    if ArgResolved.BaseType in btAllJSRangeCheckTypes then
      Result:=CreateRgCheckSt(TypeEl)
    else if ArgResolved.BaseType=btContext then
      begin
      if TypeEl.ClassType=TPasEnumType then
        Result:=CreateRgCheckSt(TypeEl);
      end
    else if ArgResolved.BaseType=btRange then
      begin
      if ArgResolved.SubType in btAllJSRangeCheckTypes then
        Result:=CreateRgCheckSt(TypeEl)
      else if ArgResolved.SubType=btContext then
        Result:=CreateRgCheckSt(TypeEl)
      else
        begin
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateProcCallArgRef ',GetResolverResultDbg(ArgResolved));
        RaiseNotSupported(El,AContext,20190220014806);
        {$ENDIF}
        end;
      end;
  end;

var
  ParamContext: TParamContext;
  FullGetter, GetPathExpr, SetPathExpr, GetExpr, SetExpr, ParamExpr,
    RHS, RgCheck: TJSElement;
  AssignSt: TJSSimpleAssignStatement;
  ObjLit: TJSObjectLiteralElement;
  FuncSt: TJSFunctionDeclarationStatement;
  RetSt: TJSReturnStatement;
  GetDotPos, SetDotPos: Integer;
  GetPath, SetPath: String;
  BracketExpr: TJSBracketMemberExpression;
  DotExpr: TJSDotMemberExpression;
  SetterArgName: String;
  TypeEl: TPasType;
  FuncContext: TFunctionContext;
  IsCOMIntf, HasCustomSetter: Boolean;
  Call: TJSCallExpression;
  StList: TJSStatementList;
begin
  // pass reference -> create a temporary JS object with a getter and setter
  Obj:=nil;
  FullGetter:=nil;
  ParamContext:=TParamContext.Create(El,nil,AContext);
  GetPathExpr:=nil;
  SetPathExpr:=nil;
  GetExpr:=nil;
  SetExpr:=nil;
  SetterArgName:=TempRefObjSetterArgName;
  RgCheck:=nil;
  try
    // create FullGetter and setter
    ParamContext.Access:=caByReference;
    ParamContext.Arg:=TargetArg;
    ParamContext.Expr:=El;
    ParamContext.ResolvedExpr:=ResolvedEl;
    FullGetter:=ConvertExpression(El,ParamContext);
    // FullGetter is now a full JS expression to retrieve the value.
    if ParamContext.ReusingReference then
      begin
      // result is already a reference
      Result:=FullGetter;
      exit;
      end;

    // if ParamContext.Getter is set then
    // ParamContext.Getter is the last part of the FullGetter
    // FullSetter is created from FullGetter by replacing the Getter with the Setter
    {$IFDEF VerbosePas2JS}
    writeln('TPasToJSConverter.CreateProcCallArgRef VAR El=',GetObjName(El),' FullGetter=',GetObjName(FullGetter),' Setter=',GetObjName(ParamContext.Setter),' ',GetResolverResultDbg(ResolvedEl));
    {$ENDIF}

    // create "{p:path,get:function(){return this.p.Getter},set:function(v){this.p.Setter(v);}}"
    Obj:=TJSObjectLiteral(CreateElement(TJSObjectLiteral,El));

    if FullGetter.ClassType=TJSPrimaryExpressionIdent then
      begin
      // create "{get:function(){return FullGetter;},set:function(v){FullGetter=v;}}"
      SetExpr:=IfReadOnlyCreateRaiseE(ParamContext);
      HasCustomSetter:=SetExpr<>nil;

      GetPath:=String(TJSPrimaryExpressionIdent(FullGetter).Name);
      GetDotPos:=PosLast('.',GetPath);
      if GetDotPos>0 then
        begin
        // e.g. path1.path2.readvar
        // create
        //    GetPathExpr: path1.path2
        //    GetExpr:     this.p.readvar
        //    SetExpr:     this.p.readvar
        // Will create "{p:GetPathExpr, get:function(){return GetExpr;},
        //                              set:function(v){SetExpr = v;}}"
        GetPathExpr:=CreatePrimitiveDotExpr(LeftStr(GetPath,GetDotPos-1),El);
        GetExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName+'.'+copy(GetPath,GetDotPos+1),El);
        if SetExpr=nil then
          SetExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName+'.'+copy(GetPath,GetDotPos+1),El);
        end
      else
        begin
        // local var
        GetExpr:=FullGetter;
        FullGetter:=nil;
        if SetExpr=nil then
          SetExpr:=CreatePrimitiveDotExpr(GetPath,El);
        end;

      if HasCustomSetter then
        begin
        // custom Setter
        ParamContext.Setter:=nil;
        if SetExpr.ClassType=TJSPrimaryExpressionIdent then
          begin
          SetPath:=String(TJSPrimaryExpressionIdent(SetExpr).Name);
          SetDotPos:=PosLast('.',SetPath);
          FreeAndNil(SetExpr);
          if LeftStr(GetPath,GetDotPos)=LeftStr(SetPath,SetDotPos) then
            begin
            // use GetPathExpr for setter
            SetExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName+'.'+copy(SetPath,GetDotPos+1),El);
            end
          else
            begin
            // setter needs its own SetPathExpr
            SetPathExpr:=CreatePrimitiveDotExpr(LeftStr(SetPath,SetDotPos-1),El);
            SetExpr:=CreatePrimitiveDotExpr('this.'+TempRefSetPathName+'.'+copy(SetPath,GetDotPos+1),El);
            end;
          end;
        end;
      end
    else if FullGetter.ClassType=TJSDotMemberExpression then
      begin
      if ParamContext.Setter<>nil then
        RaiseNotSupported(El,AContext,20170214231900);
      // convert  this.r.i  to
      // {p:this.r,
      //  get:function{return this.p.i;},
      //  set:function(v){this.p.i=v;}
      // }
      // GetPathExpr:  this.r
      // GetExpr:  this.p.i
      // SetExpr:  this.p.i
      DotExpr:=TJSDotMemberExpression(FullGetter);
      GetPathExpr:=DotExpr.MExpr;
      DotExpr.MExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName,El);
      GetExpr:=DotExpr;
      FullGetter:=nil;
      if (rrfWritable in ResolvedEl.Flags) then
        SetExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName+'.'+String(DotExpr.Name),El)
      else
        SetExpr:=IfReadOnlyCreateRaiseE(ParamContext);
      end
    else if FullGetter.ClassType=TJSBracketMemberExpression then
      begin
      if ParamContext.Setter<>nil then
        RaiseNotSupported(El,AContext,20170214215150);
      // convert  path.arr[ParamExpr]  to
      // {a:ParamExpr,
      //  p:path.arr,
      //  get:function{return this.p[this.a];},
      //  set:function(v){this.p[this.a]=v;}
      // }
      BracketExpr:=TJSBracketMemberExpression(FullGetter);
      ParamExpr:=BracketExpr.Name;

      // create "a:ParamExpr"
      AddVar(TempRefParamName,ParamExpr);

      // create GetPathExpr "this.arr"
      GetPathExpr:=BracketExpr.MExpr;

      // GetExpr  "this.p[this.a]"
      BracketExpr.MExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName,El);
      BracketExpr.Name:=CreatePrimitiveDotExpr('this.'+TempRefParamName,El);
      GetExpr:=BracketExpr;
      FullGetter:=nil;

      // SetExpr  "this.p[this.a]"
      BracketExpr:=TJSBracketMemberExpression(CreateElement(TJSBracketMemberExpression,El));
      SetExpr:=BracketExpr;
      BracketExpr.MExpr:=CreatePrimitiveDotExpr('this.'+TempRefGetPathName,El);
      BracketExpr.Name:=CreatePrimitiveDotExpr('this.'+TempRefParamName,El);
      end
    else if FullGetter.ClassType=TJSCallExpression then
      begin
      if ParamContext.Setter<>nil then
        RaiseNotSupported(El,AContext,20190210094430);
      // convert  func()  to
      // {a:func(),
      //  get:function{return this.a;},
      //  set:function(v){this.a=v;}
      // }

      // create "p:FullGetter"
      AddVar(TempRefParamName,FullGetter);
      FullGetter:=nil;

      // GetExpr  "this.a"
      GetExpr:=CreatePrimitiveDotExpr('this.'+TempRefParamName,El);

      // SetExpr  "this.a"
      SetExpr:=CreatePrimitiveDotExpr('this.'+TempRefParamName,El);
      end
    else if FullGetter.ClassType=TJSLiteral then
      begin
      // getter is a const value
      GetExpr:=FullGetter;
      FullGetter:=nil;
      SetExpr:=IfReadOnlyCreateRaiseE(ParamContext);
      ParamContext.Setter:=nil;
      // ToDo: break down SetExpr into path and property
      end
    else
      begin
      // getter is the result of an operation

      // create "p:FullGetter"
      AddVar(TempRefParamName,FullGetter);
      FullGetter:=nil;

      // GetExpr  "this.a"
      GetExpr:=CreatePrimitiveDotExpr('this.'+TempRefParamName,El);

      // SetExpr  "raise EPropReadOnly"
      SetExpr:=CreateRaisePropReadOnly(El);
      end;

    {$IFDEF VerbosePas2JS}
    //writeln('TPasToJSConverter.CreateProcCallArgRef GetExpr=',GetObjName(GetExpr),' SetExpr=',GetObjName(SetExpr),' SetterArgName=',SetterArgName);
    {$ENDIF}

    if (SetExpr.ClassType=TJSPrimaryExpressionIdent)
        or (SetExpr.ClassType=TJSDotMemberExpression)
        or (SetExpr.ClassType=TJSBracketMemberExpression) then
      begin
      // create setter
      FindAvailableLocalName(SetterArgName,SetExpr);
      RHS:=CreatePrimitiveDotExpr(SetterArgName,El);
      TypeEl:=ResolvedEl.LoTypeEl;
      IsCOMIntf:=(TypeEl is TPasClassType)
          and (TPasClassType(TypeEl).ObjKind=okInterface)
          and (TPasClassType(TypeEl).InterfaceType=citCom);
      if IsCOMIntf and (TargetArg.ArgType<>nil) then
        begin
        // create   rtl.setIntfP(path,"IntfVar",v)
        SetExpr:=CreateAssignComIntfVar(ResolvedEl,SetExpr,RHS,AContext,El);
        end
      else if (TypeEl is TPasRecordType) then
        begin
        // create  SetExpr.$assign(v)
        Call:=CreateCallExpression(El);
        Call.Expr:=CreateDotNameExpr(El,SetExpr,
                                     TJSString(GetBIName(pbifnRecordAssign)));
        Call.AddArg(RHS);
        SetExpr:=Call;
        end
      else
        begin
        // create   SetExpr = v;
        AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
        AssignSt.LHS:=SetExpr;
        AssignSt.Expr:=RHS;
        SetExpr:=AssignSt;
        if IsCOMIntf and (TargetArg.ArgType=nil) then
          begin
          // IntfVar is passed to an untyped parameter
          // This must not call AddRef, but the IntfVar must still be
          // released at the end of the function
          FuncContext:=AContext.GetFunctionContext;
          if ResolvedEl.IdentEl is TPasResultElement then
            FuncContext.ResultNeedsIntfRelease:=true
          else
            FuncContext.Add_InterfaceRelease(ResolvedEl.IdentEl);
          end
        else if (SetExpr is TJSSimpleAssignStatement)
            and (SetterArgName<>'')
            and (bsRangeChecks in AContext.ScannerBoolSwitches) then
          RgCheck:=CreateRgCheck(SetterArgName);
        end;
      end
    else if (SetExpr.ClassType=TJSCallExpression) then
      // has already the form  Func(v)
    else
      RaiseInconsistency(20170213225940,El);

    {$IFDEF VerbosePas2JS}
    //writeln('TPasToJSConverter.CreateProcCallArgRef created full SetExpr=',GetObjName(SetExpr),' SetterArgName=',SetterArgName);
    {$ENDIF}

    // add   p:GetPathExpr
    AddVar(TempRefGetPathName,GetPathExpr);

    // add   get:function(){ return GetExpr; }
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TempRefObjGetterName;
    FuncSt:=CreateFunctionSt(El);
    ObjLit.Expr:=FuncSt;
    RetSt:=TJSReturnStatement(CreateElement(TJSReturnStatement,El));
    FuncSt.AFunction.Body.A:=RetSt;
    RetSt.Expr:=GetExpr;
    GetExpr:=nil;

    // add   s:SetPathExpr
    AddVar(TempRefSetPathName,SetPathExpr);

    // add   set:function(v){ SetExpr }
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TempRefObjSetterName;
    FuncSt:=CreateFunctionSt(El);
    ObjLit.Expr:=FuncSt;
    if SetterArgName<>'' then
      FuncSt.AFunction.Params.Add(SetterArgName);
    if RgCheck<>nil then
      begin
      StList:=TJSStatementList(CreateElement(TJSStatementList,El));
      StList.A:=RgCheck;
      StList.B:=SetExpr;
      SetExpr:=StList;
      end;
    FuncSt.AFunction.Body.A:=SetExpr;
    SetExpr:=nil;

    Result:=Obj;
  finally
    if Result=nil then
      begin
      GetPathExpr.Free;
      SetPathExpr.Free;
      GetExpr.Free;
      SetExpr.Free;
      Obj.Free;
      ParamContext.Setter.Free;
      FullGetter.Free;
      end;
    ParamContext.Free;
  end;
end;

function TPasToJSConverter.CreateArrayEl(El: TPasExpr; JS: TJSElement;
  AContext: TConvertContext): TJSElement;
// call this function for every element of an array literal
// e.g. [aSet,aStaticArray]
var
  ResolvedEl: TPasResolverResult;
  ArrayType: TPasArrayType;
  TypeEl: TPasType;
begin
  Result:=JS;
  AContext.Resolver.ComputeElement(El,ResolvedEl,[rcNoImplicitProcType]);
  if ResolvedEl.IdentEl<>nil then
    begin
    // pass a variable
    if ResolvedEl.BaseType=btSet then
      begin
      // pass a set variable  -> create reference   rtl.refSet(Expr)
      Result:=CreateReferencedSet(El,Result);
      end
    else if ResolvedEl.BaseType=btContext then
      begin
      TypeEl:=ResolvedEl.LoTypeEl;
      if TypeEl.ClassType=TPasArrayType then
        begin
        ArrayType:=TPasArrayType(TypeEl);
        if length(ArrayType.Ranges)>0 then
          // pass static array variable  ->  clone
          Result:=CreateCloneStaticArray(El,ArrayType,Result,AContext);
        end
      else if TypeEl.ClassType=TPasRecordType then
        begin
        // pass record variable ->  clone "new RightRecordType(RightRecord)"
        Result:=CreateRecordCallClone(El,TPasRecordType(TypeEl),Result,AContext);
        end;
      end;
    end;
end;

function TPasToJSConverter.CreateArgumentAccess(Arg: TPasArgument;
  AContext: TConvertContext; PosEl: TPasElement): TJSElement;
var
  ArgName: String;

  function CreateSetter(const SetterName: string; AssignContext: TAssignContext): TJSElement;
  var
    Call: TJSCallExpression;
  begin
    Call:=CreateCallExpression(PosEl);
    AssignContext.Call:=Call;
    Call.Expr:=CreateDotNameExpr(PosEl,
                  CreatePrimitiveDotExpr(ArgName,PosEl),
                  TJSString(SetterName));
    Call.AddArg(AssignContext.RightSide);
    AssignContext.RightSide:=nil;
    Result:=Call;
  end;

var
  TypeEl: TPasType;
  IsRecord: boolean;
  Call: TJSCallExpression;
  AssignContext: TAssignContext;
  ParamContext: TParamContext;
begin
  ArgName:=TransformArgName(Arg,AContext);

  TypeEl:=AContext.Resolver.ResolveAliasType(Arg.ArgType);
  IsRecord:=TypeEl is TPasRecordType;

  if AContext.Access=caAssign then
    begin
    AssignContext:=AContext.AccessContext as TAssignContext;
    if IsRecord then
      begin
      // aRecordArg:=right  ->  "aRecordArg.$assign(right)"
      if AssignContext.Call<>nil then
        RaiseNotSupported(Arg,AContext,20190105174026);
      Result:=CreateSetter(GetBIName(pbifnRecordAssign),AssignContext);
      exit;
      end
    else if (Arg.ArgType=nil)
        and (AssignContext.RightResolved.LoTypeEl is TPasRecordType)
        and (rrfReadable in AssignContext.RightResolved.Flags) then
      begin
      // UntypedArg:=aRecordVar  ->  "UntypedArg.$assign(right)"
      // Note: records are passed directly to Untyped parameters
      if AssignContext.Call<>nil then
        RaiseNotSupported(Arg,AContext,20190311140048);
      Result:=CreateSetter(GetBIName(pbifnRecordAssign),AssignContext);
      exit;
      end;
    end
  else if IsRecord and (AContext is TParamContext) then
    begin
    ParamContext:=TParamContext(AContext);
    if ParamContext.ResolvedExpr.BaseType=btUntyped then
      begin
      // pass aRecordVar to UntypedArg -> pass aRecordVar directly, no temp ref object
      Result:=CreatePrimitiveDotExpr(ArgName,PosEl);
      exit;
      end;
    end;

  if (Arg.Access in [argVar,argOut]) and not IsRecord then
    begin
    // Arg is a reference object
    case AContext.Access of
      caRead:
        begin
        // create arg.get()
        Call:=CreateCallExpression(PosEl);
        Call.Expr:=CreateDotNameExpr(PosEl,
          CreatePrimitiveDotExpr(ArgName,PosEl),
          TempRefObjGetterName);
        Result:=Call;
        exit;
        end;
      caAssign:
        begin
        // create arg.set(RHS)
        AssignContext:=AContext.AccessContext as TAssignContext;
        if AssignContext.Call<>nil then
          RaiseNotSupported(Arg,AContext,20170214120606);
        Result:=CreateSetter(TempRefObjSetterName,AssignContext);
        exit;
        end;
      caByReference:
        begin
        // simply pass the reference
        ParamContext:=AContext.AccessContext as TParamContext;
        ParamContext.ReusingReference:=true;
        Result:=CreatePrimitiveDotExpr(ArgName,PosEl);
        exit;
        end;
      else
        RaiseNotSupported(Arg,AContext,20170214120739);
    end;
    end;
  Result:=CreatePrimitiveDotExpr(ArgName,PosEl);
end;

function TPasToJSConverter.ConvertExceptOn(El: TPasImplExceptOn;
  AContext: TConvertContext): TJSElement;
// convert "on T do ;" to "if(T.isPrototypeOf(exceptObject)){}"
// convert "on E:T do ;" to "if(T.isPrototypeOf(exceptObject)){ var E=exceptObject; }"
// convert "on TExternal do ;" to "if(rtl.isExt(exceptObject,TExternal)){}"

Var
  IfSt : TJSIfStatement;
  ListFirst , ListLast: TJSStatementList;
  DotExpr: TJSDotMemberExpression;
  Call: TJSCallExpression;
  V: TJSVariableStatement;
  aResolver: TPas2JSResolver;
  aType: TPasType;
  IsExternal: Boolean;
begin
  Result:=nil;
  aResolver:=AContext.Resolver;
  aType:=aResolver.ResolveAliasType(El.TypeEl);
  IsExternal:=(aType is TPasClassType) and TPasClassType(aType).IsExternal;

  // create "if()"
  IfSt:=TJSIfStatement(CreateElement(TJSIfStatement,El));
  try
    if IsExternal then
      begin
      // create rtl.isExt(exceptObject,T)
      Call:=CreateCallExpression(El);
      Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnIsExt)]);
      Call.AddArg(CreatePrimitiveDotExpr(GetBIName(pbivnExceptObject),El));
      Call.AddArg(CreateReferencePathExpr(El.TypeEl,AContext));
      end
    else
      begin
      // create "T.isPrototypeOf"
      DotExpr:=TJSDotMemberExpression(CreateElement(TJSDotMemberExpression,El));
      DotExpr.MExpr:=CreateReferencePathExpr(El.TypeEl,AContext);
      DotExpr.Name:='isPrototypeOf';
      // create "T.isPrototypeOf(exceptObject)"
      Call:=CreateCallExpression(El);
      Call.Expr:=DotExpr;
      Call.AddArg(CreatePrimitiveDotExpr(GetBIName(pbivnExceptObject),El));
      end;
    IfSt.Cond:=Call;

    if El.VarEl<>nil then
      begin
      // add "var E=exceptObject;"
      ListFirst:=TJSStatementList(CreateElement(TJSStatementList,El.Body));
      ListLast:=ListFirst;
      IfSt.BTrue:=ListFirst;
      V:=CreateVarStatement(TransformElToJSName(El.VarEl,AContext),
        CreatePrimitiveDotExpr(GetBIName(pbivnExceptObject),El),El);
      ListFirst.A:=V;
      // add statements
      if El.Body<>nil then
        AddToStatementList(ListFirst,ListLast,ConvertElement(El.Body,AContext),El);
      end
    else if El.Body<>nil then
      // add statements
      IfSt.BTrue:=ConvertElement(El.Body,AContext);

    Result:=IfSt;
  finally
    if Result=nil then
      IfSt.Free;
  end;
end;

function TPasToJSConverter.ConvertStatement(El: TPasImplStatement;
  AContext: TConvertContext): TJSElement;

begin
  Result:=Nil;
  if (El is TPasImplRaise) then
    Result:=ConvertRaiseStatement(TPasImplRaise(El),AContext)
  else if (El is TPasImplAssign) then
    Result:=ConvertAssignStatement(TPasImplAssign(El),AContext)
  else if (El is TPasImplWhileDo) then
    Result:=ConvertWhileStatement(TPasImplWhileDo(El),AContext)
  else if (El is TPasImplSimple) then
    Result:=ConvertSimpleStatement(TPasImplSimple(El),AContext)
  else if (El is TPasImplWithDo) then
    Result:=ConvertWithStatement(TPasImplWithDo(El),AContext)
  else if (El is TPasImplExceptOn) then
    Result:=ConvertExceptOn(TPasImplExceptOn(El),AContext)
  else if (El is TPasImplForLoop) then
    Result:=ConvertForStatement(TPasImplForLoop(El),AContext)
  else if (El is TPasImplAsmStatement) then
    Result:=ConvertAsmStatement(TPasImplAsmStatement(El),AContext)
  else
    RaiseNotSupported(El,AContext,20161024192759);
{
  TPasImplCaseStatement = class(TPasImplStatement)
}
end;

function TPasToJSConverter.ConvertConst(El: TPasConst; AContext: TConvertContext
  ): TJSElement;
// Important: returns nil if const was added to higher context
Var
  AssignSt: TJSSimpleAssignStatement;
  Obj: TJSObjectLiteral;
  ObjLit: TJSObjectLiteralElement;
  GlobalCtx: TFunctionContext;
  C: TJSElement;
  V: TJSVariableStatement;
  Src: TJSSourceElements;
  Proc: TPasProcedure;
  ProcScope: TPas2JSProcedureScope;
begin
  Result:=nil;
  if El.AbsoluteExpr<>nil then
    exit; // absolute: do not add a declaration
  if vmExternal in El.VarModifiers then
    exit; // external: do not add a declaration
  if not AContext.IsGlobal then
    begin
    // local const are stored in interface/implementation
    GlobalCtx:=AContext.GetGlobalFunc;
    if not (GlobalCtx.JSElement is TJSSourceElements) then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.CreateConstDecl GlobalCtx=',GetObjName(GlobalCtx),' JSElement=',GetObjName(GlobalCtx.JSElement));
      {$ENDIF}
      RaiseNotSupported(El,AContext,20170220153216);
      end;
    Src:=TJSSourceElements(GlobalCtx.JSElement);
    C:=ConvertVariable(El,AContext);
    if C=nil then
      RaiseInconsistency(20180501114422,El);
    V:=TJSVariableStatement(CreateElement(TJSVariableStatement,El));
    V.A:=C;
    AddToSourceElements(Src,V);

    if (coStoreImplJS in Options) and (AContext.Resolver<>nil) then
      begin
      Proc:=AContext.Resolver.GetTopLvlProc(AContext.PasElement);
      if Proc<>nil then
        begin
        ProcScope:=TPas2JSProcedureScope(Proc.CustomData);
        ProcScope.AddGlobalJS(CreatePrecompiledJS(V));
        end;
      end;
    end
  else if AContext is TObjectContext then
    begin
    // create 'A: initvalue'
    Obj:=TObjectContext(AContext).JSElement as TJSObjectLiteral;
    ObjLit:=Obj.Elements.AddElement;
    ObjLit.Name:=TJSString(TransformElToJSName(El,AContext));
    ObjLit.Expr:=CreateVarInit(El,AContext);
    end
  else
    begin
    // create 'this.A=initvalue'
    AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
    Result:=AssignSt;
    AssignSt.LHS:=CreateSubDeclNameExpr(El,AContext);
    AssignSt.Expr:=CreateVarInit(El,AContext);
    end;
end;

function TPasToJSConverter.ConvertLabelMark(El: TPasImplLabelMark;
  AContext: TConvertContext): TJSElement;

begin
  RaiseNotSupported(El,AContext,20161024192857);
  Result:=Nil;
  // ToDo:   TPasImplLabelMark = class(TPasImplLabelMark) then
end;

function TPasToJSConverter.ConvertElement(El: TPasElement;
  AContext: TConvertContext): TJSElement;
var
  C: TClass;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertElement El=',GetObjName(El),' Context=',GetObjName(AContext));
  {$ENDIF}
  if El=nil then
    begin
    Result:=nil;
    RaiseInconsistency(20161024190203,El);
    end;
  C:=El.ClassType;
  if C=TPasConst then
    Result:=ConvertConst(TPasConst(El),AContext)
  else if C=TPasProperty then
    Result:=ConvertProperty(TPasProperty(El),AContext)
  else if C=TPasVariable then
    Result:=ConvertVariable(TPasVariable(El),AContext)
  else if C=TPasResString then
    Result:=ConvertResString(TPasResString(El),AContext)
  else if C=TPasExportSymbol then
    Result:=ConvertExportSymbol(TPasExportSymbol(El),AContext)
  else if C=TPasLabels then
    Result:=ConvertLabels(TPasLabels(El),AContext)
  else if C=TPasImplLabelMark then
    Result:=ConvertLabelMark(TPasImplLabelMark(El),AContext)
  else if C.InheritsFrom(TPasExpr) then
    Result:=ConvertExpression(TPasExpr(El),AContext)
  else if C.InheritsFrom(TPasDeclarations) then
    Result:=ConvertDeclarations(TPasDeclarations(El),AContext)
  else if C.InheritsFrom(TPasProcedure) then
    Result:=ConvertProcedure(TPasProcedure(El),AContext)
  else if C.InheritsFrom(TPasImplBlock) then
    Result:=ConvertImplBlock(TPasImplBlock(El),AContext)
  else if C=TPasImplCommand then
    Result:=ConvertImplCommand(TPasImplCommand(El),AContext)
  else if C.InheritsFrom(TPasModule)  then
    Result:=ConvertModule(TPasModule(El),AContext)
  else if C=TPasPackage then
    Result:=ConvertPackage(TPasPackage(El),AContext)
  else
    begin
    Result:=nil;
    RaiseNotSupported(El, AContext, 20161024190449);
    end;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertElement END ',GetObjName(El));
  {$ENDIF}
end;

function TPasToJSConverter.ConvertRecordType(El: TPasRecordType;
  AContext: TConvertContext): TJSElement;
var
  aResolver: TPas2JSResolver;
  DelaySrc: TJSSourceElements;
  DelayFuncContext: TFunctionContext;
  Call: TJSCallExpression;
  JSParentName, JSName: String;
  FunDecl: TJSFunctionDeclarationStatement;
  Src: TJSSourceElements;
  FuncContext: TFunctionContext;
  i: Integer;
  P: TPasElement;
  C: TClass;
  NewEl: TJSElement;
  PasVar: TPasVariable;
  PasVarType: TPasType;
  NewFields, Vars, Methods: TFPList;
  ok, IsComplex, SpecializeDelay: Boolean;
  VarSt: TJSVariableStatement;
  AssignSt: TJSSimpleAssignStatement;
begin
  Result:=nil;
  if El.Name='' then
    RaiseNotSupported(El,AContext,20190105101258,'anonymous record');
  aResolver:=AContext.Resolver;
  if not aResolver.IsFullySpecialized(El) then exit;
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.ConvertRecordType ',GetObjName(El));
  {$ENDIF}
  FuncContext:=nil;
  NewFields:=nil;
  Vars:=nil;
  Methods:=nil;
  DelaySrc:=nil;
  DelayFuncContext:=nil;
  ok:=false;
  try
    SpecializeDelay:=SpecializeNeedsDelay(El,AContext);

    // rtl.recNewT()
    Call:=CreateCallExpression(El);
    Call.Expr:=CreateMemberExpression([GetBIName(pbivnRTL),GetBIName(pbifnRecordCreateType)]);

    // types are stored in interface/implementation
    if El.Parent is TProcedureBody then
      begin
      // local record type
      if not (AContext.JSElement is TJSSourceElements) then
        RaiseNotSupported(El,AContext,20190105104054);
      // local record type elevated to global scope
      Src:=TJSSourceElements(AContext.JSElement);
      JSName:=TransformElToJSName(El,AContext);
      VarSt:=CreateVarStatement(JSName,Call,El);
      AddToSourceElements(Src,VarSt); // keep Result=nil
      // add parameter: parent = null
      Call.AddArg(CreateLiteralNull(El));
      // add parameter: typename string = ''
      Call.AddArg(CreateLiteralString(El,''));
      end
    else
      begin
      Result:=Call;

      // add parameter: JS parent. For top level record, the module is the JS parent.
      if (El.Parent=nil)
          or ((El.Parent is TPasSection)
            and (El.Parent.ClassType<>TImplementationSection)) then
        JSParentName:=GetLocalName(El.GetModule,[cvkGlobal],AContext)
      else
        JSParentName:=GetLocalName(El.Parent,[cvkGlobal],AContext);
      if JSParentName='' then
        JSParentName:='this';
      Call.AddArg(CreatePrimitiveDotExpr(JSParentName,El));

      // add parameter: typename: string
      Call.AddArg(CreateLiteralString(El,TransformElToJSName(El,AContext)));
      end;

    // add parameter: initialize function 'function(){...}'
    FunDecl:=CreateFunctionSt(El,true,true);
    Call.AddArg(FunDecl);
    Src:=TJSSourceElements(FunDecl.AFunction.Body.A);

    // create context
    FuncContext:=TFunctionContext.Create(El,Src,AContext);
    FuncContext.IsGlobal:=true;
    FuncContext.ThisVar.Element:=El;
    FuncContext.ThisVar.Kind:=cvkGlobal;

    if (coShortRefGlobals in Options) and not (El.Parent is TProcedureBody) then
      begin
      // $lt = this;
      JSName:=AContext.GetLocalName(El,[cvkGlobal]);
      if JSName='' then
        RaiseNotSupported(El,AContext,20200926235501);
      if coStoreImplJS in Options then
        StoreImplJSLocal(El,AContext);
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AssignSt.LHS:=CreatePrimitiveDotExpr(JSName,El);
      AssignSt.Expr:=CreatePrimitiveDotExpr('this',El);
      AddToSourceElements(Src,AssignSt);
      end;

    // init fields
    NewFields:=TFPList.Create;
    Vars:=TFPList.Create;
    Methods:=TFPList.Create;
    IsComplex:=false;
    if SpecializeDelay then
      DelayFuncContext:=CreateDelayedInitMembersFunction(El,Src,FuncContext,DelaySrc);
    for i:=0 to El.Members.Count-1 do
      begin
      P:=TPasElement(El.Members[i]);
      //writeln('TPasToJSConverter.ConvertRecordType simple fields El[',i,']=',GetObjName(P));
      if not IsElementUsed(P) then continue;
      C:=P.ClassType;
      NewEl:=nil;
      if C=TPasVariable then
        begin
        PasVar:=TPasVariable(P);
        if PasVar.VarModifiers*[vmClass, vmStatic]<>[] then
          IsComplex:=true
        else if aResolver<>nil then
          begin
          Vars.Add(PasVar);
          PasVarType:=aResolver.ResolveAliasType(PasVar.VarType);
          if PasVarType.ClassType=TPasArrayType then
            begin
            // sub array
            NewFields.Add(PasVar);
            continue;
            end
          else if PasVarType.ClassType=TPasRecordType then
            begin
            // sub record
            NewFields.Add(PasVar);
            continue;
            end
          else if PasVarType.ClassType=TPasSetType then
            begin
            // sub set
            NewFields.Add(PasVar);
            continue;
            end
          else
            begin
            // simple vars are initialized in the record type, no need to initialize them for each instance
            end;
          end;
        NewEl:=CreateVarDecl(PasVar,FuncContext); // can be nil
        end
      else if C=TPasConst then
        begin
        NewEl:=ConvertConst(TPasConst(P),FuncContext);
        IsComplex:=true;
        end
      else if C=TPasProperty then
        NewEl:=ConvertProperty(TPasProperty(P),FuncContext)
      else if C.InheritsFrom(TPasType) then
        begin
        NewEl:=CreateTypeDecl(TPasType(P),FuncContext);
        if (C=TPasRecordType) or (C=TPasClassType) then
          IsComplex:=true;
        end
      else if C.InheritsFrom(TPasProcedure) then
        begin
        if (C=TPasClassConstructor)
           or (C=TPasClassDestructor) then
          AddGlobalClassMethod(FuncContext,TPasProcedure(P))
        else
          Methods.Add(P);
        end
      else if C=TPasAttributes then
      else
        RaiseNotSupported(P,FuncContext,20190105105436);
      if NewEl<>nil then
        begin
        if SpecializeDelay and not (P is TPasProcedure) then
          AddToSourceElements(DelaySrc,NewEl)
        else
          AddToSourceElements(Src,NewEl);
        end;
      end;
    if IsComplex then
      Call.AddArg(CreateLiteralBoolean(El,true)); // needs $record

    // add $new function if needed
    if NewFields.Count>0 then
      AddToSourceElements(Src,CreateRecordFunctionNew(El,FuncContext,NewFields));
    // add $eq function
    AddToSourceElements(Src,CreateRecordFunctionEqual(El,FuncContext,Vars));
    // add $assign function
    AddToSourceElements(Src,CreateRecordFunctionAssign(El,FuncContext,Vars));
    // add methods
    for i:=0 to Methods.Count-1 do
      begin
      P:=TPasProcedure(Methods[i]);
      NewEl:=ConvertProcedure(TPasProcedure(P),FuncContext);
      AddToSourceElements(Src,NewEl);
      end;

    // add RTTI init function
    if (aResolver<>nil) and HasTypeInfo(El,FuncContext) then
      begin
      if SpecializeDelay then
        CreateRecordRTTI(El,Src,FuncContext,DelaySrc,DelayFuncContext)
      else
        CreateRecordRTTI(El,Src,FuncContext,Src,FuncContext);
      end;

    ok:=true;
  finally
    NewFields.Free;
    Vars.Free;
    Methods.Free;
    DelayFuncContext.Free;
    FuncContext.Free;
    if not ok then
      FreeAndNil(Result);
  end;
end;

procedure TPasToJSConverter.DoError(Id: TMaxPrecInt; const Msg: String);
var
  E: EPas2JS;
begin
  E:=EPas2JS.Create(Msg);
  E.Id:=Id;
  E.MsgType:=mtError;
  Raise E;
end;

procedure TPasToJSConverter.DoError(Id: TMaxPrecInt; const Msg: String;
  const Args: array of {$IFDEF pas2js}jsvalue{$ELSE}const{$ENDIF});
var
  E: EPas2JS;
begin
  E:=EPas2JS.CreateFmt(Msg,Args);
  E.Id:=Id;
  E.MsgType:=mtError;
  Raise E;
end;

procedure TPasToJSConverter.DoError(Id: TMaxPrecInt; MsgNumber: integer;
  const MsgPattern: string;
  const Args: array of {$IFDEF pas2js}jsvalue{$ELSE}const{$ENDIF};
  El: TPasElement);
var
  E: EPas2JS;
begin
  E:=EPas2JS.CreateFmt(MsgPattern,Args);
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.DoError ',id,' ',GetElementDbgPath(El),':',El.ClassName,' Msg="',E.Message,'"');
  {$ENDIF}
  E.PasElement:=El;
  E.MsgNumber:=MsgNumber;
  E.Id:=Id;
  E.MsgType:=mtError;
  CreateMsgArgs(E.Args,Args);
  raise E;
end;

procedure TPasToJSConverter.RaiseNotSupported(El: TPasElement;
  AContext: TConvertContext; Id: TMaxPrecInt; const Msg: string);
var
  E: EPas2JS;
begin
  {$IFDEF VerbosePas2JS}
  writeln('TPasToJSConverter.RaiseNotSupported ',id,' ',GetElementDbgPath(El),':',El.ClassName,' Msg="',Msg,'"');
  {$ENDIF}
  if AContext=nil then ;
  E:=EPas2JS.CreateFmt(sPasElementNotSupported,[GetObjName(El)+' ['+IntToStr(Id)+']']);
  if Msg<>'' then
    E.Message:=E.Message+': '+Msg;
  E.PasElement:=El;
  E.MsgNumber:=nPasElementNotSupported;
  SetLength(E.Args,1);
  E.Args[0]:=El.ClassName;
  E.Id:=Id;
  E.MsgType:=mtError;
  raise E;
end;

procedure TPasToJSConverter.RaiseIdentifierNotFound(Identifier: string;
  El: TPasElement; Id: TMaxPrecInt);
var
  E: EPas2JS;
begin
  E:=EPas2JS.CreateFmt(sIdentifierNotFound,[Identifier]);
  E.PasElement:=El;
  E.MsgNumber:=nIdentifierNotFound;
  SetLength(E.Args,1);
  E.Args[0]:=Identifier;
  E.Id:=Id;
  E.MsgType:=mtError;
  raise E;
end;

procedure TPasToJSConverter.RaiseInconsistency(Id: TMaxPrecInt; El: TPasElement);
var
  s: String;
begin
  s:='TPasToJSConverter.RaiseInconsistency['+IntToStr(Id)+']: you found a bug';
  if El<>nil then
    begin
    s:=s+GetElementDbgPath(El);
    if El.Name<>'' then
      s:=s+El.Name
    else
      s:=s+GetElementTypeName(El);
    s:=s+' at '+TPas2JSResolver.GetDbgSourcePosStr(El);
    end;
  raise Exception.Create(s);
end;

function TPasToJSConverter.TransformToJSName(ErrorEl: TPasElement;
  const AName: String; CheckGlobal: boolean; AContext: TConvertContext): String;
// CheckGlobal: check name clashes with global identifiers too
var
  i: Integer;
  c: Char;
begin
  if AContext=nil then ;
  if Pos('.',AName)>0 then
    RaiseInconsistency(20170203164711,ErrorEl);
  if UseLowerCase then
    Result:=LowerCase(AName)
  else
    Result:=AName;
  if not IsReservedWord(Result,CheckGlobal) then
    exit;
  for i:=1 to length(Result) do
    begin
    c:=Result[i];
    case c of
    'a'..'z','A'..'Z':
      begin
      Result[i]:=chr(ord(c) xor 32);
      if not IsReservedWord(Result,CheckGlobal) then
        exit;
      end;
    end;
    end;
  RaiseNotSupported(ErrorEl,AContext,20170203131832);
end;

function TPasToJSConverter.TransformElToJSName(El: TPasElement;
  AContext: TConvertContext): String;
var
  aType: TPasType;
begin
  if (El is TPasProcedure) and (TPasProcedure(El).LibrarySymbolName<>nil) then
    Result:=ComputeConstString(TPasProcedure(El).LibrarySymbolName,AContext,true)
  else if (El is TPasVariable) and (TPasVariable(El).ExportName<>nil) then
    Result:=ComputeConstString(TPasVariable(El).ExportName,AContext,true)
  else if (El is TPasType) then
    begin
    if AContext.Resolver<>nil then
      aType:=AContext.Resolver.ResolveAliasType(TPasType(El))
    else
      aType:=TPasType(El);
    if (aType.ClassType=TPasClassType) and TPasClassType(aType).IsExternal then
      Result:=TPasClassType(aType).ExternalName
    else
      Result:=TransformToJSName(El,GetOverloadName(aType,AContext),
                                CanClashWithGlobal(aType),AContext);
    end
  else
    Result:=TransformToJSName(El,GetOverloadName(El,AContext),
                                  CanClashWithGlobal(El),AContext);
end;

function TPasToJSConverter.TransformModuleName(El: TPasModule;
  AddModulesPrefix: boolean; AContext: TConvertContext): String;
var
  p, StartP: Integer;
  aName, Part: String;
begin
  if AddModulesPrefix then
    begin
    Result:=GetLocalName(El,[cvkGlobal],AContext);
    if Result<>'' then
      exit;
    end;
  if El.ClassType=TPasProgram then
    Result:=GetBIName(pbivnProgram)
  else if El.ClassType=TPasLibrary then
    Result:=GetBIName(pbivnLibrary)
  else
    begin
    Result:='';
    aName:=El.Name;
    p:=1;
    while p<=length(aName) do
      begin
      StartP:=p;
      while (p<=length(aName)) and (aName[p]<>'.') do inc(p);
      Part:=copy(aName,StartP,p-StartP);
      Part:=TransformToJSName(El,Part,false,AContext);
      if Result<>'' then Result:=Result+'.';
      Result:=Result+Part;
      inc(p);
      end;
    end;
  if AddModulesPrefix then
    begin
    if Pos('.',Result)>0 then
      Result:=GetBIName(pbivnModules)+'["'+Result+'"]'
    else
      Result:=GetBIName(pbivnModules)+'.'+Result;

    if coShortRefGlobals in Options then
      Result:=CreateGlobalAliasForeign(El,Result,AContext);
    end;
end;

function TPasToJSConverter.IsReservedWord(const aName: string;
  CheckGlobal: boolean): boolean;
var
  l, r, m, cmp: Integer;
begin
  Result:=true;
  if aName=GetBIName(pbivnModules) then exit;
  if aName=GetBIName(pbivnRTL) then exit;

  // search default list
  l:=low(JSReservedWords);
  r:=high(JSReservedWords);
  while l<=r do
    begin
    m:=(l+r) div 2;
    cmp:=CompareStr(aName,JSReservedWords[m]);
    //writeln('TPasToJSConverter.IsReservedWord Name="',aName,'" l=',l,' r=',r,' m=',m,' JSReservedWords[m]=',JSReservedWords[m],' cmp=',cmp);
    if cmp>0 then
      l:=m+1
    else if cmp<0 then
      r:=m-1
    else
      exit;
    end;

  // search user list
  l:=0;
  r:=length(FReservedWords)-1;
  while l<=r do
    begin
    m:=(l+r) div 2;
    cmp:=CompareStr(aName,FReservedWords[m]);
    //writeln('TPasToJSConverter.IsReservedWord Name="',aName,'" l=',l,' r=',r,' m=',m,' FReservedWords[m]=',FReservedWords[m],' cmp=',cmp);
    if cmp>0 then
      l:=m+1
    else if cmp<0 then
      r:=m-1
    else
      exit;
    end;

  if CheckGlobal then
    begin
    // search default global list
    l:=low(JSReservedGlobalWords);
    r:=high(JSReservedGlobalWords);
    while l<=r do
      begin
      m:=(l+r) div 2;
      cmp:=CompareStr(aName,JSReservedGlobalWords[m]);
      //writeln('TPasToJSConverter.IsReservedWord Name="',aName,'" l=',l,' r=',r,' m=',m,' JSReservedGlobalWords[m]=',JSReservedGlobalWords[m],' cmp=',cmp);
      if cmp>0 then
        l:=m+1
      else if cmp<0 then
        r:=m-1
      else
        exit;
      end;
    end;

  Result:=false;
end;

function TPasToJSConverter.GetTypeInfoName(El: TPasType;
  AContext: TConvertContext; ErrorEl: TPasElement; Full: boolean): String;
var
  C: TClass;
  bt: TResolverBaseType;
  jbt: TPas2jsBaseType;
  CurEl: TPasElement;
  aName: String;
begin
  Result:='';
  El:=ResolveSimpleAliasType(El);
  if El=nil then
    RaiseInconsistency(20170409172756,El);
  C:=El.ClassType;

  if C=TPasSpecializeType then
    begin
    if not (El.CustomData is TPasSpecializeTypeData) then
      RaiseInconsistency(20200220113319,El);
    El:=TPasSpecializeTypeData(El.CustomData).SpecializedType;
    C:=El.ClassType;
    end;

  if (El=AContext.PasElement) and not Full then
    begin
    // referring to itself
    if El is TPasMembersType then
      begin
      // use this
      Result:=GetBIName(pbivnRTTILocal);
      exit;
      end
    else
      RaiseNotSupported(ErrorEl,AContext,20170905150746,'cannot typeinfo itself');
    end;
  if C=TPasUnresolvedSymbolRef then
    begin
    if El.Name='' then
      DoError(20170905150752,nTypeXCannotBePublished,sTypeXCannotBePublished,
        ['typeinfo of anonymous '+El.ElementTypeName],ErrorEl);
    if El.CustomData is TResElDataBaseType then
      begin
      bt:=TResElDataBaseType(El.CustomData).BaseType;
      case bt of
      btWideChar: bt:=btChar;
      btUnicodeString: bt:=btString;
      btCurrency: bt:=btIntDouble;
      end;

      case bt of
      btShortInt,btByte,
      btSmallInt,btWord,
      btLongint,btLongWord,
      btIntDouble,btUIntDouble,
      btString,btChar,
      btDouble,
      btBoolean,
      btPointer:
        begin
        // create rtl.basename
        Result:=GetBIName(pbivnRTL)+'.'+lowercase(AContext.Resolver.BaseTypeNames[bt]);
        exit;
        end;
      btCustom:
        if El.CustomData is TResElDataPas2JSBaseType then
          begin
          jbt:=TResElDataPas2JSBaseType(El.CustomData).JSBaseType;
          case jbt of
          pbtJSValue:
            begin
            // create rtl.basename
            Result:=GetBIName(pbivnRTL)+'.'+lowercase(Pas2jsBaseTypeNames[jbt]);
            exit;
            end;
          else
            {$IFDEF VerbosePas2JS}
            writeln('TPasToJSConverter.CreateTypeInfoRef [20170905150833] El=',GetObjName(El),' El.CustomData=',GetObjName(El.CustomData),' jbt=',Pas2jsBaseTypeNames[jbt]);
            {$ENDIF}
          end;
          end
        else
          begin
          {$IFDEF VerbosePas2JS}
          writeln('TPasToJSConverter.CreateTypeInfoRef [20170905150840] El=',GetObjName(El),' El.CustomData=',GetObjName(El.CustomData),' bt=',AContext.Resolver.BaseTypeNames[bt]);
          {$ENDIF}
          end
      else
        {$IFDEF VerbosePas2JS}
        writeln('TPasToJSConverter.CreateTypeInfoRef [20170905150842] El=',GetObjName(El),' El.CustomData=',GetObjName(El.CustomData),' bt=',AContext.Resolver.BaseTypeNames[bt]);
        {$ENDIF}
      end;
      end
    else
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.CreateTypeInfoRef [20170905150844] El=',GetObjName(El),' El.CustomData=',GetObjName(El.CustomData));
      {$ENDIF}
      end;
    end
  else if (C=TPasEnumType)
      or (C=TPasSetType)
      or (C=TPasClassType)
      or (C=TPasClassOfType)
      or (C=TPasArrayType)
      or (C=TPasProcedureType)
      or (C=TPasFunctionType)
      or (C=TPasPointerType)
      or (C=TPasTypeAliasType)
      or (C=TPasRecordType)
      or (C=TPasRangeType)
      then
    begin
    // user type  ->  module.$rtti["pascalname"]
    // Notes:
    // a nested type gets the parent types prepended: classnameA.ElName
    // an anonymous type gets for each level '$a' prepended
    // an anonymous type of a variable/argument gets the variable name prepended
    CurEl:=ResolveSimpleAliasType(TPasType(El));
    repeat
      if CurEl.Name<>'' then
        begin
        // RTTI uses Pascal name
        Result:=CurEl.Name+Result;
        end
      else
        begin
        // anonymous type -> prepend '$a'
        // for example:
        //   "var AnArray: array of array of char;" becomes AnArray$a$a
        Result:=GetBIName(pbitnAnonymousPostfix)+Result;
        end;
      CurEl:=CurEl.Parent;
      if CurEl=nil then
        break;
      C:=CurEl.ClassType;
      if (C=TPasClassType)
          or (C=TPasRecordType) then
        // nested
        Result:='.'+Result
      else if C.InheritsFrom(TPasType)
          or (C=TPasVariable)
          or (C=TPasConst)
          or (C=TPasArgument)
          or (C=TPasProperty) then
        begin
        // for example: var a: array of longint;
        end
      else
        break;
    until false;

    if CurEl is TPasSection then
      exit;
    end;
  aName:=El.Name;
  if aName='' then aName:=El.ClassName;
  DoError(20170409173329,nTypeXCannotBePublished,sTypeXCannotBePublished,
    [aName],ErrorEl);
end;

function TPasToJSConverter.TransformArgName(Arg: TPasArgument;
  AContext: TConvertContext): string;
begin
  Result:=Arg.Name;
  if (CompareText(Result,'Self')=0) and (Arg.Parent is TPasProcedure) then
    begin
    // hidden self argument
    Result:=AContext.GetLocalName(Arg,cvkAll);
    if Result='' then
      begin
      {$IFDEF VerbosePas2JS}
      writeln('TPasToJSConverter.TransformArgName Arg=',GetObjPath(Arg));
      AContext.WriteStack;
      {$ENDIF}
      RaiseNotSupported(Arg,AContext,20190205190114,GetObjName(Arg.Parent));
      end;
    end
  else
    Result:=TransformToJSName(Arg,Result,true,AContext);
end;

function TPasToJSConverter.CreateGlobalAliasForeign(El: TPasElement; JSPath: string;
  AContext: TConvertContext): string;
var
  ElModule, MyModule: TPasModule;
  aResolver: TPas2JSResolver;
  SectionContext: TSectionContext;
  FuncContext: TFunctionContext;
  Expr: TJSElement;
  V: TJSVariableStatement;
  AssignSt: TJSSimpleAssignStatement;
  ElClass: TClass;
begin
  Result:=JSPath;
  if El is TPasUnresolvedSymbolRef then
    exit; // built-in element

  ElModule:=El.GetModule;
  aResolver:=AContext.Resolver;
  MyModule:=aResolver.RootElement;
  if ElModule=MyModule then
    begin
    // El is in this module
    exit;
    end
  else
    begin
    // El is from another unit
    SectionContext:=TSectionContext(AContext.GetMainSectionContext);

    FuncContext:=AContext.GetFunctionContext;
    ElClass:=El.ClassType;
    if ElClass.InheritsFrom(TPasType) then
      Result:=GetBIName(pbivnLocalTypeRef)
    else if ElClass.InheritsFrom(TPasProcedure) then
      Result:=GetBIName(pbivnLocalProcRef)
    else if ElClass=TPasEnumValue then
      Result:=GetBIName(pbivnLocalTypeRef)
    else if ElClass.InheritsFrom(TPasModule) then
      Result:=GetBIName(pbivnLocalModuleRef)
    else
      RaiseNotSupported(El,AContext,20200608160225);
    Result:=FuncContext.CreateLocalIdentifier(Result,El,cvkGlobal);
    SectionContext.AddLocalVar(Result,El,cvkGlobal,true);
    if coStoreImplJS in Options then
      StoreImplJSLocal(El,AContext);

    if aResolver.ImplementationUsesUnit(ElModule) then
      begin
      // insert var $lm = null;
      Expr:=CreateLiteralNull(El);
      V:=CreateVarStatement(Result,Expr,El);
      AddHeaderStatement(V,El,SectionContext);
      // insert impl  $lm = JSPath;
      AssignSt:=TJSSimpleAssignStatement(CreateElement(TJSSimpleAssignStatement,El));
      AssignSt.LHS:=CreatePrimitiveDotExpr(Result,El);
      AssignSt.Expr:=CreatePrimitiveDotExpr(JSPath,El);
      AddImplHeaderStatement(AssignSt,El,AContext);
      end
    else
      begin
      // insert var $lm = JSPath;
      Expr:=CreatePrimitiveDotExpr(JSPath,El);
      V:=CreateVarStatement(Result,Expr,El);
      AddHeaderStatement(V,El,SectionContext);
      end;
    end;
end;

function TPasToJSConverter.CreateGlobalAliasNull(El: TPasElement;
  Prefix: TPas2JSBuiltInName; SectionContext: TSectionContext
  ): TFCLocalIdentifier;
var
  V: TJSVariableStatement;
begin
  // insert var $lt = null;
  Result:=SectionContext.AddLocalVar(GetBIName(Prefix),El,cvkGlobal,true);
  V:=CreateVarStatement(Result.Name,CreateLiteralNull(El),El);
  AddHeaderStatement(V,El,SectionContext);
end;

procedure TPasToJSConverter.CreateGlobalAlias_List(ElRefList: TFPList;
  AContext: TConvertContext);
var
  i: Integer;
  SectionContext: TSectionContext;
  El: TPasElement;
begin
  if ElRefList=nil then exit;
  if ElRefList.Count=0 then exit;
  SectionContext:=TSectionContext(AContext.GetMainSectionContext);
  for i:=0 to ElRefList.Count-1 do
    begin
    El:=TPasElement(ElRefList[i]);
    // Note: they are all needed by precompiled code, do not check ElNeedsGlobalAlias
    CreateGlobalElPath(El,SectionContext);
    end;
end;

function TPasToJSConverter.ElNeedsGlobalAlias(El: TPasElement): boolean;
var
  C: TClass;
begin
  Result:=false;
  if El=nil then exit;
  if not (coShortRefGlobals in Options) then
    exit;
  C:=El.ClassType;
  if El.CustomData is TResElDataBuiltInSymbol then
    exit(false)
  else if C.InheritsFrom(TPasType) then
    exit(true)
  else if C.InheritsFrom(TPasProcedure) then
    exit(ProcCanHaveShortRef(TPasProcedure(El)))
  else if C=TPasEnumValue then
    begin
    if not (coEnumNumbers in Options) then
      exit(true);
    end
  else if C.InheritsFrom(TPasModule) then
    exit(true);
end;

function TPasToJSConverter.ConvertPasElement(El: TPasElement;
  Resolver: TPas2JSResolver): TJSElement;
var
  aContext: TRootContext;
  Scanner: TPas2jsPasScanner;
begin
  if FGlobals=nil then
    FGlobals:=TPasToJSConverterGlobals.Create(Self);
  if (Resolver<>nil)
      and (Resolver.CurrentParser<>nil)
      and (Resolver.CurrentParser.Scanner is TPas2jsPasScanner) then
    begin
    Scanner:=TPas2jsPasScanner(Resolver.CurrentParser.Scanner);
    Options:=Options+Scanner.GlobalConvOptsEnabled-Scanner.GlobalConvOptsDisabled;
    end;
  aContext:=TRootContext.Create(El,nil,nil);
  try
    aContext.Resolver:=Resolver;
    if (El.ClassType=TPasImplBeginBlock) then
      Result:=ConvertBeginEndStatement(TPasImplBeginBlock(El),AContext,false)
    else
      Result:=ConvertElement(El,aContext);
  finally
    FreeAndNil(aContext);
  end;
end;

end.

