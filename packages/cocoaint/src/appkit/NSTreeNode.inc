{ Parsed from Appkit.framework NSTreeNode.h }

{$ifdef TYPES}
{$ifndef NSTREENODE_PAS_T}
{$define NSTREENODE_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTREENODE_PAS_R}
{$define NSTREENODE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTREENODE_PAS_F}
{$define NSTREENODE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTREENODE_PAS_S}
{$define NSTREENODE_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTreeNode = objcclass;
  NSTreeNodePointer = ^NSTreeNode;
  NSTreeNodePtr = NSTreeNodePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTREENODE_PAS_C}
{$define NSTREENODE_PAS_C}

{ NSTreeNode }
  NSTreeNode = objcclass external (NSObject)
  private
    _childNodesProxy: id;
    _representedObject: id;
    _observationInfo: Pointer;
    _reserved2: id;
    _childNodes: NSMutableArray;
    _parentNode: NSTreeNode;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __NSTreeNodeFlags: record
      case byte of
       0: (_anonbitfield_NSTreeNode0: cuint);
       1: (data: bitpacked record
        ignoreObserving: 0..1;
        reserved: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __NSTreeNodeFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTreeNode0: cuint);
       1: (
        ignoreObserving: 0..1;
        reserved: 0..((1 shl 31)-1);
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
      );
      end;
    
  public
    class function treeNodeWithRepresentedObject(modelObject: id): id; message 'treeNodeWithRepresentedObject:';
    function initWithRepresentedObject(modelObject: id): id; message 'initWithRepresentedObject:';
    function representedObject: id; message 'representedObject';
    function indexPath: NSIndexPath; message 'indexPath';
    function isLeaf: Boolean; message 'isLeaf';
    function childNodes: NSArray; message 'childNodes';
    function mutableChildNodes: NSMutableArray; message 'mutableChildNodes';
    function descendantNodeAtIndexPath(indexPath_: NSIndexPath): NSTreeNode; message 'descendantNodeAtIndexPath:';
    function parentNode: NSTreeNode; message 'parentNode';
    procedure sortWithSortDescriptors_recursively(sortDescriptors: NSArray; recursively: Boolean); message 'sortWithSortDescriptors:recursively:';
  end;

{$endif}
{$endif}
