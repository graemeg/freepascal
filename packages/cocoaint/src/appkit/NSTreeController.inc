{ Parsed from Appkit.framework NSTreeController.h }

{$ifdef TYPES}
{$ifndef NSTREECONTROLLER_PAS_T}
{$define NSTREECONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTREECONTROLLER_PAS_R}
{$define NSTREECONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTREECONTROLLER_PAS_F}
{$define NSTREECONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTREECONTROLLER_PAS_S}
{$define NSTREECONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTreeController = objcclass;
  NSTreeControllerPointer = ^NSTreeController;
  NSTreeControllerPtr = NSTreeControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTREECONTROLLER_PAS_C}
{$define NSTREECONTROLLER_PAS_C}

{ NSTreeController }
  NSTreeController = objcclass external (NSObjectController)
  private
    _treeControllerReserved1: id;
    _modelObservingKeyPaths: NSArray;
    _treeStructureObservers: id;
    _arrangedObjects: id;
    _rootNode: id;
    _selectionIndexPaths: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __treeControllerFlags: record
      case byte of
       0: (_anonbitfield_NSTreeController0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __treeControllerFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTreeController0: cuint);
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
        _avoidsEmptySelection: 0..1;
        _preservesSelection: 0..1;
        _selectsInsertedObjects: 0..1;
        _explicitlyCannotInsert: 0..1;
        _explicitlyCannotInsertChild: 0..1;
        _explicitlyCannotAddChild: 0..1;
        _alwaysUsesMultipleValuesMarker: 0..1;
        _observingThroughArrangedObjects: 0..1;
        _mutatingNodes: 0..1;
        _performingFetch: 0..1;
        _skipSortingAfterFetch: 0..1;
        _usesIdenticalComparisonOfModelObjects: 0..1;
        _reservedTreeController: 0..((1 shl 20)-1);
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
    _selectedObjects: NSArray;
    _childrenKeyPath: NSString;
    _countKeyPath: NSString;
    _leafKeyPath: NSString;
    _sortDescriptors: NSArray;
    
  public
    procedure rearrangeObjects; message 'rearrangeObjects';
    function arrangedObjects: id; message 'arrangedObjects';
    procedure setChildrenKeyPath(keyPath: NSString); message 'setChildrenKeyPath:';
    function childrenKeyPath: NSString; message 'childrenKeyPath';
    procedure setCountKeyPath(keyPath: NSString); message 'setCountKeyPath:';
    function countKeyPath: NSString; message 'countKeyPath';
    procedure setLeafKeyPath(keyPath: NSString); message 'setLeafKeyPath:';
    function leafKeyPath: NSString; message 'leafKeyPath';
    procedure setSortDescriptors(sortDescriptors_: NSArray); message 'setSortDescriptors:';
    function sortDescriptors: NSArray; message 'sortDescriptors';
    function content: id; message 'content';
    procedure setContent(content_: id); message 'setContent:';
    procedure add(sender: id); message 'add:';
    procedure remove(sender: id); message 'remove:';
    procedure addChild(sender: id); message 'addChild:';
    procedure insert(sender: id); message 'insert:';
    procedure insertChild(sender: id); message 'insertChild:';
    function canInsert: Boolean; message 'canInsert';
    function canInsertChild: Boolean; message 'canInsertChild';
    function canAddChild: Boolean; message 'canAddChild';
    procedure insertObject_atArrangedObjectIndexPath(object_: id; indexPath: NSIndexPath); message 'insertObject:atArrangedObjectIndexPath:';
    procedure insertObjects_atArrangedObjectIndexPaths(objects: NSArray; indexPaths: NSArray); message 'insertObjects:atArrangedObjectIndexPaths:';
    procedure removeObjectAtArrangedObjectIndexPath(indexPath: NSIndexPath); message 'removeObjectAtArrangedObjectIndexPath:';
    procedure removeObjectsAtArrangedObjectIndexPaths(indexPaths: NSArray); message 'removeObjectsAtArrangedObjectIndexPaths:';
    procedure setAvoidsEmptySelection(flag: Boolean); message 'setAvoidsEmptySelection:';
    function avoidsEmptySelection: Boolean; message 'avoidsEmptySelection';
    procedure setPreservesSelection(flag: Boolean); message 'setPreservesSelection:';
    function preservesSelection: Boolean; message 'preservesSelection';
    procedure setSelectsInsertedObjects(flag: Boolean); message 'setSelectsInsertedObjects:';
    function selectsInsertedObjects: Boolean; message 'selectsInsertedObjects';
    procedure setAlwaysUsesMultipleValuesMarker(flag: Boolean); message 'setAlwaysUsesMultipleValuesMarker:';
    function alwaysUsesMultipleValuesMarker: Boolean; message 'alwaysUsesMultipleValuesMarker';
    function selectedObjects: NSArray; message 'selectedObjects';
    function setSelectionIndexPaths(indexPaths: NSArray): Boolean; message 'setSelectionIndexPaths:';
    function selectionIndexPaths: NSArray; message 'selectionIndexPaths';
    function setSelectionIndexPath(indexPath: NSIndexPath): Boolean; message 'setSelectionIndexPath:';
    function selectionIndexPath: NSIndexPath; message 'selectionIndexPath';
    function addSelectionIndexPaths(indexPaths: NSArray): Boolean; message 'addSelectionIndexPaths:';
    function removeSelectionIndexPaths(indexPaths: NSArray): Boolean; message 'removeSelectionIndexPaths:';
    function selectedNodes: NSArray; message 'selectedNodes';
    procedure moveNode_toIndexPath(node: NSTreeNode; indexPath: NSIndexPath); message 'moveNode:toIndexPath:';
    procedure moveNodes_toIndexPath(nodes: NSArray; startingIndexPath: NSIndexPath); message 'moveNodes:toIndexPath:';
    function childrenKeyPathForNode(node: NSTreeNode): NSString; message 'childrenKeyPathForNode:';
    function countKeyPathForNode(node: NSTreeNode): NSString; message 'countKeyPathForNode:';
    function leafKeyPathForNode(node: NSTreeNode): NSString; message 'leafKeyPathForNode:';
  end;

{$endif}
{$endif}
