{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by the Free Pascal development team

    This unit provides the definition of the NSObject root class

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit objcbase;

{$ifdef FPC_HAS_FEATURE_OBJECTIVEC1}

{$modeswitch objectivec1}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$packrecords c}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

interface

uses
  ctypes;

type
  NSString = objcclass external;
  NSInvocation = objcclass external;
  NSMethodSignature = objcclass external;
  NSCoder = objcclass external;

{ needed by NSZone.h below }
{$if defined(cpu64) or defined(cpuarm) or defined(win32)}
  NSInteger = clong;
  NSUInteger = culong;
{$else}
  NSInteger = cint;
  NSUInteger = cuint;
{$endif}


{ NSZone.h, has to be here because NSObject.h imports NSZone.inc }

{ Types }
type
  NSZone = record
  end;
  PNSZone = ^NSZone;
  NSZonePtr = PNSZone;

{ Constants }

const
  NSScannedOption = 1 shl 0;
  NSCollectorDisabledOption = 1 shl 1;

  { Functions }
  function NSDefaultMallocZone: NSZonePtr; cdecl; external;
  function NSCreateZone(startSize: NSUInteger; granularity: NSUInteger; canFree: Boolean): NSZonePtr; cdecl; external;
  procedure NSRecycleZone(zone_: NSZonePtr); cdecl; external;
  procedure NSSetZoneName(zone_: NSZonePtr; name: NSString); cdecl; external;
  function NSZoneName(zone_: NSZonePtr): NSString; cdecl; external;
  function NSZoneFromPointer(ptr: Pointer): NSZonePtr; cdecl; external;
  function NSZoneMalloc(zone_: NSZonePtr; size: NSUInteger): Pointer; cdecl; external;
  function NSZoneCalloc(zone_: NSZonePtr; numElems: NSUInteger; byteSize: NSUInteger): Pointer; cdecl; external;
  function NSZoneRealloc(zone_: NSZonePtr; ptr: Pointer; size: NSUInteger): Pointer; cdecl; external;
  procedure NSZoneFree(zone_: NSZonePtr; ptr: Pointer); cdecl; external;
  function NSAllocateCollectable(size: NSUInteger; options: NSUInteger): Pointer; cdecl; external;
  function NSReallocateCollectable(ptr: Pointer; size: NSUInteger; options: NSUInteger): Pointer; cdecl; external;
  function NSPageSize: NSUInteger; cdecl; external;
  function NSLogPageSize: NSUInteger; cdecl; external;
  function NSRoundUpToMultipleOfPageSize(bytes: NSUInteger): NSUInteger; cdecl; external;
  function NSRoundDownToMultipleOfPageSize(bytes: NSUInteger): NSUInteger; cdecl; external;
  function NSAllocateMemoryPages(bytes: NSUInteger): Pointer; cdecl; external;
  procedure NSDeallocateMemoryPages(ptr: Pointer; bytes: NSUInteger); cdecl; external;
  procedure NSCopyMemoryPages(source: Pointer; dest: Pointer; bytes: NSUInteger); cdecl; external;
  function NSRealMemoryAvailable: NSUInteger; cdecl; external;

type
  Protocol = objcclass external
  end;

  NSObjectProtocol = objcprotocol external name 'NSObject'
    function isEqual(obj: id): boolean; message 'isEqual:';
    function hash: NSUInteger; message 'hash';

    function superclass: pobjc_class; message 'superclass';
    function _class: pobjc_class; message 'class';
    { "self" is both a hidden parameter to each method, and a method of
      NSObject and thereby of each subclass as well
    }
    function self: id;  message 'self';
    function zone: PNSZone;  message 'zone';

    function performSelector(aSelector: SEL): id; message 'performSelector:';
    function performSelector_withObject(aSelector: SEL; obj: id): id; message 'performSelector:withObject:';
    function performSelector_withObject_withObject(aSelector: SEL; obj1, obj2: id): id; message 'performSelector:withObject:withObject:';

    function isProxy: boolean; message 'isProxy';

    function isKindOfClass(aClass: pobjc_class): boolean; message 'isKindOfClass:';
    function isMemberOfClass(aClass: pobjc_class): boolean; message 'isMemberOfClass:';
    function conformsToProtocol(aProtocol: Protocol): boolean; message 'conformsToProtocol:';

    function respondsToSelector(aSelector: SEL): boolean; message 'respondsToSelector:';

    function retain: id; message 'retain';
    procedure release;  message 'release'; { oneway }
    function autorelease: id; message 'autorelease';
    function retainCount: NSUInteger; message 'retainCount';

    function description: NSString; message 'description';
  end;


  NSObject = objcclass external (NSObjectProtocol)
   strict protected
    isa: pobjc_class;
   public
    { NSObjectProtocol -- the message names are copied from the protocol
      definition by the compiler, but you can still repeat them if you want }
    function isEqual(obj: id): boolean;
    function isEqual_(obj: id): boolean; message 'isEqual:';
    function hash: NSUInteger;

    function superclass: pobjc_class;
    function _class: pobjc_class;
    { "self" is both a hidden parameter to each method, and a method of
      NSObject and thereby of each subclass as well
    }
    function self: id;
    function zone: PNSZone;

    function performSelector(aSelector: SEL): id;
    function performSelector_(aSelector: SEL): id; message 'performSelector:';
    function performSelector_withObject(aSelector: SEL; obj: id): id;
    function performSelector_withObject_(aSelector: SEL; obj: id): id; message 'performSelector:withObject:';
    function performSelector_withObject_withObject(aSelector: SEL; obj1, obj2: id): id;
    function performSelector_withObject_withObject_(aSelector: SEL; obj1, obj2: id): id; message 'performSelector:withObject:withObject:';

    function isProxy: boolean;

    function isKindOfClass(aClass: pobjc_class): boolean;
    function isKindOfClass_(aClass: pobjc_class): boolean; message 'isKindOfClass:';
    function isMemberOfClass(aClass: pobjc_class): boolean;
    function isMemberOfClass_(aClass: pobjc_class): boolean; message 'isMemberOfClass:';
    function conformsToProtocol(aProtocol: Protocol): boolean;
    function conformsToProtocol_(aProtocol: Protocol): boolean; message 'conformsToProtocol:';

    function respondsToSelector(aSelector: SEL): boolean;
    function respondsToSelector_(aSelector: SEL): boolean; message 'respondsToSelector:';

    function retain: id;
    procedure release; { oneway }
    function autorelease: id;
    function retainCount: NSUInteger;

    function description: NSString;

    { NSObject class }
    { "class" prefix to method name to avoid name collision with NSObjectProtocol }
    class function classIsEqual(obj: id): boolean; message 'isEqual:';
    class function classIsEqual_(obj: id): boolean; message 'isEqual:';
    { "class" prefix to method name to avoid name collision with NSObjectProtocol }
    class function classHash: cuint; message 'hash';

    { NSObject methods }

    class procedure load; message 'load';

    class procedure initialize; message 'initialize';
    function init: id; message 'init';

    class function new: id; message 'new';
    class function allocWithZone(_zone: PNSZone): id; message 'allocWithZone:';
    class function allocWithZone_(_zone: PNSZone): id; message 'allocWithZone:';
    class function alloc: id; message 'alloc';
    procedure dealloc; message 'dealloc';

    { if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_4 }
    procedure finalize; message 'finalize';
    { endif }

    function copy: id; message 'copy';
    function mutableCopy: id; message 'mutableCopy';

    class function classCopyWithZone(_zone: NSZonePtr): id; message 'copyWithZone:';
    class function classCopyWithZone_(_zone: NSZonePtr): id; message 'copyWithZone:';
    class function classMutableCopyWithZone(_zone: NSZonePtr): id; message 'mutableCopyWithZone:';
    class function classMutableCopyWithZone_(_zone: NSZonePtr): id; message 'mutableCopyWithZone:';

    { "class" prefix to method name to avoid name collision with NSObjectProtocol }
    class function classSuperclass: pobjc_class; message 'superclass';
    { "class" prefix to method name to avoid name collision with NSObjectProtocol }
    class function classClass: pobjc_class; message 'class';
    class procedure poseAsClass(aClass: pobjc_class); message 'poseAsClass:';
    class procedure poseAsClass_(aClass: pobjc_class); message 'poseAsClass:';
    class function instancesRespondToSelector(aSelector: SEL): boolean; message 'instancesRespondToSelector:';
    class function instancesRespondToSelector_(aSelector: SEL): boolean; message 'instancesRespondToSelector:';
    { "class" prefix to method name to avoid name collision with NSObjectProtocol }
    class function classConformsToProtocol(aProtocol: Protocol): boolean; message 'conformsToProtocol:';
    class function classConformsToProtocol_(aProtocol: Protocol): boolean; message 'conformsToProtocol:';
    function methodForSelector(aSelector: SEL): IMP; message 'methodForSelector:';
    function methodForSelector_(aSelector: SEL): IMP; message 'methodForSelector:';
    class function instanceMethodForSelector(aSelector: SEL): IMP; message 'instanceMethodForSelector:';
    class function instanceMethodForSelector_(aSelector: SEL): IMP; message 'instanceMethodForSelector:';
    procedure doesNotRecognizeSelector(aSelector: SEL); message 'doesNotRecognizeSelector:';
    procedure doesNotRecognizeSelector_(aSelector: SEL); message 'doesNotRecognizeSelector:';
    procedure forwardInvocation(anInvocation: NSInvocation); message 'forwardInvocation:';
    procedure forwardInvocation_(anInvocation: NSInvocation); message 'forwardInvocation:';
    function methodSignatureForSelector(aSelector: SEL): NSMethodSignature; message 'methodSignatureForSelector:';
    function methodSignatureForSelector_(aSelector: SEL): NSMethodSignature; message 'methodSignatureForSelector:';

    // can't be classDescription, becaue there's a method in another
    // class that's also called classDescription
    class function _classDescription: NSString; message 'description';

  end;


  NSCoderMethods = objccategory external (NSObject)
    class function version: cint; message 'version';
    class procedure setVersion(aVersion: cint); message 'setVersion:';
    function classForCoder: pobjc_class; message 'classForCoder';
    function replacementObjectForCoder(aCoder: NSCoder): id; message 'replacementObjectForCoder:';
    function awakeAfterUsingCoder(aDecoder: NSCoder): id; message 'awakeAfterUsingCoder:';
  end;


  NSCopyingProtocol = objcprotocol external name 'NSCopying'
    function copyWithZone(zone_: NSZonePtr): id; message 'copyWithZone:';
  end;


{ NSMutableCopying Protocol }
  NSMutableCopyingProtocol = objcprotocol external name 'NSMutableCopying'
    function mutableCopyWithZone(zone_: NSZonePtr): id; message 'mutableCopyWithZone:';
  end;
  
{ NSCoding Protocol }
  NSCodingProtocol = objcprotocol external name 'NSCoding'
    procedure encodeWithCoder(aCoder: NSCoder); message 'encodeWithCoder:';
    function initWithCoder(aDecoder: NSCoder): id; message 'initWithCoder:';
  end;
  
{ NSDiscardableContent Protocol }
  NSDiscardableContentProtocol = objcprotocol external name 'NSDiscardableContent'
    function beginContentAccess: Boolean; message 'beginContentAccess';
    procedure endContentAccess; message 'endContentAccess';
    procedure discardContentIfPossible; message 'discardContentIfPossible';
    function isContentDiscarded: Boolean; message 'isContentDiscarded';
  end;

implementation

{$else }

interface

implementation

{$endif}

end.


