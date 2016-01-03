{ Parsed from Foundation.framework NSXMLDTD.h }

{$ifdef TYPES}
{$ifndef NSXMLDTD_PAS_T}
{$define NSXMLDTD_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSXMLDTD_PAS_R}
{$define NSXMLDTD_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSXMLDTD_PAS_F}
{$define NSXMLDTD_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSXMLDTD_PAS_S}
{$define NSXMLDTD_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSXMLDTD = objcclass;
  NSXMLDTDPointer = ^NSXMLDTD;
  NSXMLDTDPtr = NSXMLDTDPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSXMLDTD_PAS_C}
{$define NSXMLDTD_PAS_C}

{ NSXMLDTD }
  NSXMLDTD = objcclass external (NSXMLNode)
  private
    _name: NSString;
    _publicID: NSString;
    _systemID: NSString;
    _children: NSArray;
    _childrenHaveMutated: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _padding3: array[0..(3)-1] of cuint8;
=======
        _padding3: array[0..2] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding3: array[0..2] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding3: array[0..2] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding3: array[0..2] of cuint8;
>>>>>>> origin/cpstrnew
    _entities: NSMutableDictionary;
    _elements: NSMutableDictionary;
    _notations: NSMutableDictionary;
    _attributes: NSMutableDictionary;
    _original: NSString;
    _modified: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _padding2: array[0..(3)-1] of cuint8;
=======
        _padding2: array[0..2] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding2: array[0..2] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding2: array[0..2] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding2: array[0..2] of cuint8;
>>>>>>> origin/cpstrnew
    
  public
    function initWithContentsOfURL_options_error(url: NSURL; mask: NSUInteger; error: NSErrorPointer): id; message 'initWithContentsOfURL:options:error:';
    function initWithData_options_error(data: NSData; mask: NSUInteger; error: NSErrorPointer): id; message 'initWithData:options:error:';
    procedure setPublicID(publicID_: NSString); message 'setPublicID:';
    function publicID: NSString; message 'publicID';
    procedure setSystemID(systemID_: NSString); message 'setSystemID:';
    function systemID: NSString; message 'systemID';
    procedure insertChild_atIndex(child: NSXMLNode; index_: NSUInteger); message 'insertChild:atIndex:';
    procedure insertChildren_atIndex(children_: NSArray; index_: NSUInteger); message 'insertChildren:atIndex:';
    procedure removeChildAtIndex(index_: NSUInteger); message 'removeChildAtIndex:';
    procedure setChildren(children_: NSArray); message 'setChildren:';
    procedure addChild(child: NSXMLNode); message 'addChild:';
    procedure replaceChildAtIndex_withNode(index_: NSUInteger; node: NSXMLNode); message 'replaceChildAtIndex:withNode:';
    function entityDeclarationForName(name_: NSString): NSXMLDTDNode; message 'entityDeclarationForName:';
    function notationDeclarationForName(name_: NSString): NSXMLDTDNode; message 'notationDeclarationForName:';
    function elementDeclarationForName(name_: NSString): NSXMLDTDNode; message 'elementDeclarationForName:';
    function attributeDeclarationForName_elementName(name_: NSString; elementName: NSString): NSXMLDTDNode; message 'attributeDeclarationForName:elementName:';
    class function predefinedEntityDeclarationForName(name_: NSString): NSXMLDTDNode; message 'predefinedEntityDeclarationForName:';
  end;

{$endif}
{$endif}
