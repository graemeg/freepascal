{ Parsed from Foundation.framework NSCache.h }

{$ifdef TYPES}
{$ifndef NSCACHE_PAS_T}
{$define NSCACHE_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCACHE_PAS_R}
{$define NSCACHE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCACHE_PAS_F}
{$define NSCACHE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCACHE_PAS_S}
{$define NSCACHE_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSCacheDelegateProtocol = objcprotocol;
  NSCache = objcclass;
  NSCachePointer = ^NSCache;
  NSCachePtr = NSCachePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCACHE_PAS_C}
{$define NSCACHE_PAS_C}

{ NSCache }
  NSCache = objcclass external (NSObject)
  private
    _delegate: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _private: array[0..(5)-1] of Pointer;
=======
        _private: array[0..4] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..4] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..4] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _private: array[0..4] of Pointer;
>>>>>>> origin/cpstrnew
    _reserved: Pointer;
    
  public
    procedure setName(n: NSString); message 'setName:';
    function name: NSString; message 'name';
    procedure setDelegate(d: id); message 'setDelegate:';
    function delegate: id; message 'delegate';
    function objectForKey(key: id): id; message 'objectForKey:';
    procedure setObject_forKey(obj: id; key: id); message 'setObject:forKey:';
    procedure setObject_forKey_cost(obj: id; key: id; g: NSUInteger); message 'setObject:forKey:cost:';
    procedure removeObjectForKey(key: id); message 'removeObjectForKey:';
    procedure removeAllObjects; message 'removeAllObjects';
    procedure setTotalCostLimit(lim: NSUInteger); message 'setTotalCostLimit:';
    function totalCostLimit: NSUInteger; message 'totalCostLimit';
    procedure setCountLimit(lim: NSUInteger); message 'setCountLimit:';
    function countLimit: NSUInteger; message 'countLimit';
    function evictsObjectsWithDiscardedContent: Boolean; message 'evictsObjectsWithDiscardedContent';
    procedure setEvictsObjectsWithDiscardedContent(b: Boolean); message 'setEvictsObjectsWithDiscardedContent:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSCACHE_PAS_P}
{$define NSCACHE_PAS_P}
  
{ NSCacheDelegate Protocol }
  NSCacheDelegateProtocol = objcprotocol external name 'NSCacheDelegate'
  optional
    procedure cache_willEvictObject(cache: NSCache; obj: id); message 'cache:willEvictObject:';
  end;
{$endif}
{$endif}
