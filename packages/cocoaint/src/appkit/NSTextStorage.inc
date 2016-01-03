{ Parsed from Appkit.framework NSTextStorage.h }

{$ifdef TYPES}
{$ifndef NSTEXTSTORAGE_PAS_T}
{$define NSTEXTSTORAGE_PAS_T}

{ Constants }

const
  NSTextStorageEditedAttributes = 1;
  NSTextStorageEditedCharacters = 2;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTEXTSTORAGE_PAS_R}
{$define NSTEXTSTORAGE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTEXTSTORAGE_PAS_F}
{$define NSTEXTSTORAGE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTEXTSTORAGE_PAS_S}
{$define NSTEXTSTORAGE_PAS_S}

{ External string constants }
var
  NSTextStorageWillProcessEditingNotification: NSString; cvar; external;
  NSTextStorageDidProcessEditingNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTextStorageDelegateProtocol = objcprotocol;
  NSTextStorage = objcclass;
  NSTextStoragePointer = ^NSTextStorage;
  NSTextStoragePtr = NSTextStoragePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTEXTSTORAGE_PAS_C}
{$define NSTEXTSTORAGE_PAS_C}

{ NSTextStorage }
  NSTextStorage = objcclass external (NSMutableAttributedString)
  private
    _editedRange: NSRange;
    _editedDelta: NSInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSTextStorage2: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTextStorage2: cuint);
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
        _editedMask: 0..((1 shl 8)-1);
        _anonbitfield_NSTextStorage0: 0..((1 shl 8)-1);
        disabled: 0..((1 shl 16)-1);
        {$ifdef cpu64}
        _anonbitfield_NSTextStorage1: 0..((1 shl 32)-1);
        {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    _layoutManagers: NSMutableArray;
    _sideData: id;
    
  public
    procedure addLayoutManager(obj: NSLayoutManager); message 'addLayoutManager:';
    procedure removeLayoutManager(obj: NSLayoutManager); message 'removeLayoutManager:';
    function layoutManagers: NSArray; message 'layoutManagers';
    procedure edited_range_changeInLength(editedMask_: NSUInteger; range: NSRange; delta: NSInteger); message 'edited:range:changeInLength:';
    procedure processEditing; message 'processEditing';
    procedure invalidateAttributesInRange(range: NSRange); message 'invalidateAttributesInRange:';
    procedure ensureAttributesAreFixedInRange(range: NSRange); message 'ensureAttributesAreFixedInRange:';
    function fixesAttributesLazily: Boolean; message 'fixesAttributesLazily';
    function editedMask: NSUInteger; message 'editedMask';
    function editedRange: NSRange; message 'editedRange';
    function changeInLength: NSInteger; message 'changeInLength';
    procedure setDelegate(delegate_: NSTextStorageDelegateProtocol); message 'setDelegate:';
    function delegate: NSTextStorageDelegateProtocol; message 'delegate';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSTEXTSTORAGE_PAS_P}
{$define NSTEXTSTORAGE_PAS_P}
  
{ NSTextStorageDelegate Protocol }
  NSTextStorageDelegateProtocol = objcprotocol external name 'NSTextStorageDelegate'
  optional
    procedure textStorageWillProcessEditing(notification: NSNotification); message 'textStorageWillProcessEditing:';
    procedure textStorageDidProcessEditing(notification: NSNotification); message 'textStorageDidProcessEditing:';
  end;
{$endif}
{$endif}
