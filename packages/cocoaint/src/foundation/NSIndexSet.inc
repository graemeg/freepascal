{ Parsed from Foundation.framework NSIndexSet.h }

{$ifdef TYPES}
{$ifndef NSINDEXSET_PAS_T}
{$define NSINDEXSET_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSINDEXSET_PAS_R}
{$define NSINDEXSET_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSINDEXSET_PAS_F}
{$define NSINDEXSET_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSINDEXSET_PAS_S}
{$define NSINDEXSET_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSIndexSet = objcclass;
  NSIndexSetPointer = ^NSIndexSet;
  NSIndexSetPtr = NSIndexSetPointer;
  NSMutableIndexSet = objcclass;
  NSMutableIndexSetPointer = ^NSMutableIndexSet;
  NSMutableIndexSetPtr = NSMutableIndexSetPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSINDEXSET_PAS_C}
{$define NSINDEXSET_PAS_C}

{ NSIndexSet }
  NSIndexSet = objcclass external (NSObject, NSCopyingProtocol, NSMutableCopyingProtocol, NSCodingProtocol)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __indexSetFlags: record
      case byte of
       0: (_anonbitfield_NSIndexSet0: NSUInteger);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __indexSetFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSIndexSet0: NSUInteger);
       1: (
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
        _isEmpty: 0..1;
        _hasSingleRange: 0..1;
        _cacheValid: 0..1;
        _reservedArrayBinderController: 0..((1 shl 29)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
      );
      end;
    _internal: record
      case byte of
        0: (
    __singleRange: record
        _range: NSRange;
      end;
           );
	1: (
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    __singleRange: record
        _range: NSRange;
      end;
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
    __multipleRanges: record
        _data: Pointer; {garbage collector: __strong }
        _reserved: Pointer;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           );
    end;
=======
    __internal: record
      end;
>>>>>>> graemeg/cpstrnew
=======
    __internal: record
      end;
>>>>>>> graemeg/cpstrnew
=======
    __internal: record
      end;
>>>>>>> graemeg/cpstrnew
=======
    __internal: record
      end;
>>>>>>> origin/cpstrnew
    
  public
    class function indexSet: id; message 'indexSet';
    class function indexSetWithIndex(value: NSUInteger): id; message 'indexSetWithIndex:';
    class function indexSetWithIndexesInRange(range: NSRange): id; message 'indexSetWithIndexesInRange:';
    function init: id; message 'init';
    function initWithIndex(value: NSUInteger): id; message 'initWithIndex:';
    function initWithIndexesInRange(range: NSRange): id; message 'initWithIndexesInRange:';
    function initWithIndexSet(indexSet_: NSIndexSet): id; message 'initWithIndexSet:';
    function isEqualToIndexSet(indexSet_: NSIndexSet): Boolean; message 'isEqualToIndexSet:';
    function count: NSUInteger; message 'count';
    function firstIndex: NSUInteger; message 'firstIndex';
    function lastIndex: NSUInteger; message 'lastIndex';
    function indexGreaterThanIndex(value: NSUInteger): NSUInteger; message 'indexGreaterThanIndex:';
    function indexLessThanIndex(value: NSUInteger): NSUInteger; message 'indexLessThanIndex:';
    function indexGreaterThanOrEqualToIndex(value: NSUInteger): NSUInteger; message 'indexGreaterThanOrEqualToIndex:';
    function indexLessThanOrEqualToIndex(value: NSUInteger): NSUInteger; message 'indexLessThanOrEqualToIndex:';
    function getIndexes_maxCount_inIndexRange(indexBuffer: NSUIntegerPtr; bufferSize: NSUInteger; range: NSRangePointer): NSUInteger; message 'getIndexes:maxCount:inIndexRange:';
    function countOfIndexesInRange(range: NSRange): NSUInteger; message 'countOfIndexesInRange:';
    function containsIndex(value: NSUInteger): Boolean; message 'containsIndex:';
    function containsIndexesInRange(range: NSRange): Boolean; message 'containsIndexesInRange:';
    function containsIndexes(indexSet_: NSIndexSet): Boolean; message 'containsIndexes:';
    function intersectsIndexesInRange(range: NSRange): Boolean; message 'intersectsIndexesInRange:';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    function mutableCopyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSMutableIndexSet }
  NSMutableIndexSet = objcclass external (NSIndexSet)
  private
    _reserved: Pointer;
    
  public
    procedure addIndexes(indexSet_: NSIndexSet); message 'addIndexes:';
    procedure removeIndexes(indexSet_: NSIndexSet); message 'removeIndexes:';
    procedure removeAllIndexes; message 'removeAllIndexes';
    procedure addIndex(value: NSUInteger); message 'addIndex:';
    procedure removeIndex(value: NSUInteger); message 'removeIndex:';
    procedure addIndexesInRange(range: NSRange); message 'addIndexesInRange:';
    procedure removeIndexesInRange(range: NSRange); message 'removeIndexesInRange:';
    procedure shiftIndexesStartingAtIndex_by(index: NSUInteger; delta: NSInteger); message 'shiftIndexesStartingAtIndex:by:';
  end;

{$endif}
{$endif}
