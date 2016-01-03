{ Parsed from Appkit.framework NSRuleEditor.h }

{$ifdef TYPES}
{$ifndef NSRULEEDITOR_PAS_T}
{$define NSRULEEDITOR_PAS_T}

{ Constants }

const
  NSRuleEditorNestingModeSingle = 0;
  NSRuleEditorNestingModeList = 1;
  NSRuleEditorNestingModeCompound = 2;
  NSRuleEditorNestingModeSimple = 3;

const
  NSRuleEditorRowTypeSimple = 0;
  NSRuleEditorRowTypeCompound = 1;

{ Types }
type
  NSRuleEditorNestingMode = NSUInteger;
  NSRuleEditorNestingModePtr = ^NSRuleEditorNestingMode;
  NSRuleEditorRowType = NSUInteger;
  NSRuleEditorRowTypePtr = ^NSRuleEditorRowType;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSRULEEDITOR_PAS_R}
{$define NSRULEEDITOR_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSRULEEDITOR_PAS_F}
{$define NSRULEEDITOR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSRULEEDITOR_PAS_S}
{$define NSRULEEDITOR_PAS_S}

{ External string constants }
var
  NSRuleEditorPredicateLeftExpression: NSString; cvar; external;
  NSRuleEditorPredicateRightExpression: NSString; cvar; external;
  NSRuleEditorPredicateComparisonModifier: NSString; cvar; external;
  NSRuleEditorPredicateOptions: NSString; cvar; external;
  NSRuleEditorPredicateOperatorType: NSString; cvar; external;
  NSRuleEditorPredicateCustomSelector: NSString; cvar; external;
  NSRuleEditorPredicateCompoundType: NSString; cvar; external;
  NSRuleEditorRowsDidChangeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSRuleEditorDelegateProtocol = objcprotocol;
  NSRuleEditor = objcclass;
  NSRuleEditorPointer = ^NSRuleEditor;
  NSRuleEditorPtr = NSRuleEditorPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSRULEEDITOR_PAS_C}
{$define NSRULEEDITOR_PAS_C}

{ NSRuleEditor }
  NSRuleEditor = objcclass external (NSControl)
  private
    _ruleDataSource: id;
    _ruleDelegate: id;
    _draggingRows: NSIndexSet;
    _rowCache: NSMutableArray;
    _slicesHolder: NSView;
    _slices: NSMutableArray;
    _sliceHeight: CGFloat;
    _alignmentGridWidth: CGFloat;
    _subviewIndexOfDropLine: NSInteger;
    _dropLineView: id;
    _currentAnimation: NSViewAnimation;
    _frameTimer: NSTimer;
    _stringsFileName: NSString;
    _standardLocalizer: id;
    _headerLocalizer: id;
    _predicate: NSPredicate;
    _nestingMode: NSInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __ruleEditorFlags: record
      case byte of
       0: (_anonbitfield_NSRuleEditor0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __ruleEditorFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSRuleEditor0: cuint);
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
        elideUpdating: 0..1;
        lastAlternateKeyValue: 0..1;
        extendedDelegateCalls: 0..1;
        editable: 0..1;
        settingSize: 0..1;
        suppressKeyDown: 0..1;
        dropWasSuccessful: 0..1;
        delegateWantsValidation: 0..1;
        disallowEmpty: 0..1;
        lastDrewWithFRAppearance: 0..1;
        allowsEmptyCompoundRows: 0..1;
        dropChangedRowCount: 0..1;
        reserved: 0..((1 shl 20)-1);
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
    _typeKeyPath: NSString;
    _itemsKeyPath: NSString;
    _valuesKeyPath: NSString;
    _subrowsArrayKeyPath: NSString;
    _rowClass: Pobjc_class;
    _boundArrayOwner: id;
    _boundArrayKeyPath: NSString;
    _ruleReserved1: id;
    _lastRow: NSInteger;
    _ruleReserved2: id;
    
  public
    procedure setDelegate(delegate_: NSRuleEditorDelegateProtocol); message 'setDelegate:';
    function delegate: NSRuleEditorDelegateProtocol; message 'delegate';
    procedure setFormattingStringsFilename(stringsFilename: NSString); message 'setFormattingStringsFilename:';
    function formattingStringsFilename: NSString; message 'formattingStringsFilename';
    procedure setFormattingDictionary(dictionary: NSDictionary); message 'setFormattingDictionary:';
    function formattingDictionary: NSDictionary; message 'formattingDictionary';
    procedure reloadCriteria; message 'reloadCriteria';
    procedure setNestingMode(mode: NSRuleEditorNestingMode); message 'setNestingMode:';
    function nestingMode: NSRuleEditorNestingMode; message 'nestingMode';
    procedure setRowHeight(height: CGFloat); message 'setRowHeight:';
    function rowHeight: CGFloat; message 'rowHeight';
    procedure setEditable(editable: Boolean); message 'setEditable:';
    function isEditable: Boolean; message 'isEditable';
    procedure setCanRemoveAllRows(val: Boolean); message 'setCanRemoveAllRows:';
    function canRemoveAllRows: Boolean; message 'canRemoveAllRows';
    function predicate: NSPredicate; message 'predicate';
    procedure reloadPredicate; message 'reloadPredicate';
    function predicateForRow(row: NSInteger): NSPredicate; message 'predicateForRow:';
    function numberOfRows: NSInteger; message 'numberOfRows';
    function subrowIndexesForRow(rowIndex: NSInteger): NSIndexSet; message 'subrowIndexesForRow:';
    function criteriaForRow(row: NSInteger): NSArray; message 'criteriaForRow:';
    function displayValuesForRow(row: NSInteger): NSArray; message 'displayValuesForRow:';
    function rowForDisplayValue(displayValue: id): NSInteger; message 'rowForDisplayValue:';
    function rowTypeForRow(rowIndex: NSInteger): NSRuleEditorRowType; message 'rowTypeForRow:';
    function parentRowForRow(rowIndex: NSInteger): NSInteger; message 'parentRowForRow:';
    procedure addRow(sender: id); message 'addRow:';
    procedure insertRowAtIndex_withType_asSubrowOfRow_animate(rowIndex: NSInteger; rowType: NSRuleEditorRowType; parentRow: NSInteger; shouldAnimate: Boolean); message 'insertRowAtIndex:withType:asSubrowOfRow:animate:';
    procedure setCriteria_andDisplayValues_forRowAtIndex(criteria: NSArray; values: NSArray; rowIndex: NSInteger); message 'setCriteria:andDisplayValues:forRowAtIndex:';
    procedure removeRowAtIndex(rowIndex: NSInteger); message 'removeRowAtIndex:';
    procedure removeRowsAtIndexes_includeSubrows(rowIndexes: NSIndexSet; includeSubrows: Boolean); message 'removeRowsAtIndexes:includeSubrows:';
    function selectedRowIndexes: NSIndexSet; message 'selectedRowIndexes';
    procedure selectRowIndexes_byExtendingSelection(indexes: NSIndexSet; extend: Boolean); message 'selectRowIndexes:byExtendingSelection:';
    procedure setRowClass(rowClass_: Pobjc_class); message 'setRowClass:';
    function rowClass: Pobjc_class; message 'rowClass';
    procedure setRowTypeKeyPath(keyPath: NSString); message 'setRowTypeKeyPath:';
    function rowTypeKeyPath: NSString; message 'rowTypeKeyPath';
    procedure setSubrowsKeyPath(keyPath: NSString); message 'setSubrowsKeyPath:';
    function subrowsKeyPath: NSString; message 'subrowsKeyPath';
    procedure setCriteriaKeyPath(keyPath: NSString); message 'setCriteriaKeyPath:';
    function criteriaKeyPath: NSString; message 'criteriaKeyPath';
    procedure setDisplayValuesKeyPath(keyPath: NSString); message 'setDisplayValuesKeyPath:';
    function displayValuesKeyPath: NSString; message 'displayValuesKeyPath';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSRULEEDITOR_PAS_P}
{$define NSRULEEDITOR_PAS_P}
  
{ NSRuleEditorDelegate Protocol }
  NSRuleEditorDelegateProtocol = objcprotocol external name 'NSRuleEditorDelegate'
  required
    function ruleEditor_numberOfChildrenForCriterion_withRowType(editor: NSRuleEditor; criterion: id; rowType: NSRuleEditorRowType): NSInteger; message 'ruleEditor:numberOfChildrenForCriterion:withRowType:';
    function ruleEditor_child_forCriterion_withRowType(editor: NSRuleEditor; index: NSInteger; criterion: id; rowType: NSRuleEditorRowType): id; message 'ruleEditor:child:forCriterion:withRowType:';
    function ruleEditor_displayValueForCriterion_inRow(editor: NSRuleEditor; criterion: id; row: NSInteger): id; message 'ruleEditor:displayValueForCriterion:inRow:';
  optional
    function ruleEditor_predicatePartsForCriterion_withDisplayValue_inRow(editor: NSRuleEditor; criterion: id; value: id; row: NSInteger): NSDictionary; message 'ruleEditor:predicatePartsForCriterion:withDisplayValue:inRow:';
    procedure ruleEditorRowsDidChange(notification: NSNotification); message 'ruleEditorRowsDidChange:';
  end;
{$endif}
{$endif}
