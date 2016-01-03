{ Parsed from Foundation.framework NSXMLElement.h }

{$ifdef TYPES}
{$ifndef NSXMLELEMENT_PAS_T}
{$define NSXMLELEMENT_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSXMLELEMENT_PAS_R}
{$define NSXMLELEMENT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSXMLELEMENT_PAS_F}
{$define NSXMLELEMENT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSXMLELEMENT_PAS_S}
{$define NSXMLELEMENT_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSXMLElement = objcclass;
  NSXMLElementPointer = ^NSXMLElement;
  NSXMLElementPtr = NSXMLElementPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSXMLELEMENT_PAS_C}
{$define NSXMLELEMENT_PAS_C}

{ NSXMLElement }
  NSXMLElement = objcclass external (NSXMLNode)
  private
    _name: NSString;
    _attributes: NSMutableArray;
    _namespaces: NSMutableArray;
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
    _URI: NSString;
    _prefixIndex: NSInteger;
    
  public
    function initWithName(name_: NSString): id; message 'initWithName:';
    function initWithName_URI(name_: NSString; URI_: NSString): id; message 'initWithName:URI:';
    function initWithName_stringValue(name_: NSString; string_: NSString): id; message 'initWithName:stringValue:';
    function initWithXMLString_error(string_: NSString; error: NSErrorPointer): id; message 'initWithXMLString:error:';
    function elementsForName(name_: NSString): NSArray; message 'elementsForName:';
    function elementsForLocalName_URI(localName_: NSString; URI_: NSString): NSArray; message 'elementsForLocalName:URI:';
    procedure addAttribute(attribute: NSXMLNode); message 'addAttribute:';
    procedure removeAttributeForName(name_: NSString); message 'removeAttributeForName:';
    procedure setAttributes(attributes_: NSArray); message 'setAttributes:';
    procedure setAttributesAsDictionary(attributes_: NSDictionary); message 'setAttributesAsDictionary:';
    function attributes: NSArray; message 'attributes';
    function attributeForName(name_: NSString): NSXMLNode; message 'attributeForName:';
    function attributeForLocalName_URI(localName_: NSString; URI_: NSString): NSXMLNode; message 'attributeForLocalName:URI:';
    procedure addNamespace(aNamespace: NSXMLNode); message 'addNamespace:';
    procedure removeNamespaceForPrefix(name_: NSString); message 'removeNamespaceForPrefix:';
    procedure setNamespaces(namespaces_: NSArray); message 'setNamespaces:';
    function namespaces: NSArray; message 'namespaces';
    function namespaceForPrefix(name_: NSString): NSXMLNode; message 'namespaceForPrefix:';
    function resolveNamespaceForName(name_: NSString): NSXMLNode; message 'resolveNamespaceForName:';
    function resolvePrefixForNamespaceURI(namespaceURI: NSString): NSString; message 'resolvePrefixForNamespaceURI:';
    procedure insertChild_atIndex(child: NSXMLNode; index_: NSUInteger); message 'insertChild:atIndex:';
    procedure insertChildren_atIndex(children_: NSArray; index_: NSUInteger); message 'insertChildren:atIndex:';
    procedure removeChildAtIndex(index_: NSUInteger); message 'removeChildAtIndex:';
    procedure setChildren(children_: NSArray); message 'setChildren:';
    procedure addChild(child: NSXMLNode); message 'addChild:';
    procedure replaceChildAtIndex_withNode(index_: NSUInteger; node: NSXMLNode); message 'replaceChildAtIndex:withNode:';
    procedure normalizeAdjacentTextNodesPreservingCDATA(preserve: Boolean); message 'normalizeAdjacentTextNodesPreservingCDATA:';
  end;

{$endif}
{$endif}
