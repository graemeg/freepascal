{ Parsed from Foundation.framework NSXMLDocument.h }

{$ifdef TYPES}
{$ifndef NSXMLDOCUMENT_PAS_T}
{$define NSXMLDOCUMENT_PAS_T}

{ Constants }

const
  NSXMLDocumentXMLKind = 0;
  NSXMLDocumentXHTMLKind = 1;
  NSXMLDocumentHTMLKind = 2;
  NSXMLDocumentTextKind = 3;

{ Types }
type
  NSXMLDocumentContentKind = NSUInteger;
  NSXMLDocumentContentKindPtr = ^NSXMLDocumentContentKind;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSXMLDOCUMENT_PAS_R}
{$define NSXMLDOCUMENT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSXMLDOCUMENT_PAS_F}
{$define NSXMLDOCUMENT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSXMLDOCUMENT_PAS_S}
{$define NSXMLDOCUMENT_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSXMLDocument = objcclass;
  NSXMLDocumentPointer = ^NSXMLDocument;
  NSXMLDocumentPtr = NSXMLDocumentPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSXMLDOCUMENT_PAS_C}
{$define NSXMLDOCUMENT_PAS_C}

{ NSXMLDocument }
  NSXMLDocument = objcclass external (NSXMLNode)
  private
    _encoding: NSString;
    _version: NSString;
    _docType: NSXMLDTD;
    _children: NSArray;
    _childrenHaveMutated: Boolean;
    _standalone: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _padding3: array[0..(2)-1] of cuint8;
=======
        _padding3: array[0..1] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding3: array[0..1] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding3: array[0..1] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _padding3: array[0..1] of cuint8;
>>>>>>> origin/cpstrnew
    _rootElement: NSXMLElement;
    _URI: NSString;
    _MIMEType: NSString;
    _fidelityMask: NSUInteger;
    _contentKind: NSXMLDocumentContentKind;
    
  public
    function initWithXMLString_options_error(string_: NSString; mask: NSUInteger; error: NSErrorPointer): id; message 'initWithXMLString:options:error:';
    function initWithContentsOfURL_options_error(url: NSURL; mask: NSUInteger; error: NSErrorPointer): id; message 'initWithContentsOfURL:options:error:';
    function initWithData_options_error(data: NSData; mask: NSUInteger; error: NSErrorPointer): id; message 'initWithData:options:error:';
    function initWithRootElement(element: NSXMLElement): id; message 'initWithRootElement:';
    class function replacementClassForClass(cls: Pobjc_class): Pobjc_class; message 'replacementClassForClass:';
    procedure setCharacterEncoding(encoding: NSString); message 'setCharacterEncoding:';
    function characterEncoding: NSString; message 'characterEncoding';
    procedure setVersion(version_: NSString); message 'setVersion:';
    function version: NSString; message 'version';
    procedure setStandalone(standalone: Boolean); message 'setStandalone:';
    function isStandalone: Boolean; message 'isStandalone';
    procedure setDocumentContentKind(kind_: NSXMLDocumentContentKind); message 'setDocumentContentKind:';
    function documentContentKind: NSXMLDocumentContentKind; message 'documentContentKind';
    procedure setMIMEType(MIMEType_: NSString); message 'setMIMEType:';
    function MIMEType: NSString; message 'MIMEType';
    procedure setDTD(documentTypeDeclaration: NSXMLDTD); message 'setDTD:';
    function DTD: NSXMLDTD; message 'DTD';
    procedure setRootElement(root: NSXMLNode); message 'setRootElement:';
    function rootElement: NSXMLElement; message 'rootElement';
    procedure insertChild_atIndex(child: NSXMLNode; index_: NSUInteger); message 'insertChild:atIndex:';
    procedure insertChildren_atIndex(children_: NSArray; index_: NSUInteger); message 'insertChildren:atIndex:';
    procedure removeChildAtIndex(index_: NSUInteger); message 'removeChildAtIndex:';
    procedure setChildren(children_: NSArray); message 'setChildren:';
    procedure addChild(child: NSXMLNode); message 'addChild:';
    procedure replaceChildAtIndex_withNode(index_: NSUInteger; node: NSXMLNode); message 'replaceChildAtIndex:withNode:';
    function XMLData: NSData; message 'XMLData';
    function XMLDataWithOptions(options: NSUInteger): NSData; message 'XMLDataWithOptions:';
    function objectByApplyingXSLT_arguments_error(xslt: NSData; arguments: NSDictionary; error: NSErrorPointer): id; message 'objectByApplyingXSLT:arguments:error:';
    function objectByApplyingXSLTString_arguments_error(xslt: NSString; arguments: NSDictionary; error: NSErrorPointer): id; message 'objectByApplyingXSLTString:arguments:error:';
    function objectByApplyingXSLTAtURL_arguments_error(xsltURL: NSURL; argument: NSDictionary; error: NSErrorPointer): id; message 'objectByApplyingXSLTAtURL:arguments:error:';
    function validateAndReturnError(error: NSErrorPointer): Boolean; message 'validateAndReturnError:';
  end;

{$endif}
{$endif}
