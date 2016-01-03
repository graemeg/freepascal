(*
 * Summary: the core parser module
 * Description: Interfaces, constants and types related to the XML parser
 *
 * Copy: See Copyright for the status of this software.
 *
 * Author: Daniel Veillard
 *)

{$IFDEF POINTER}
  xmlParserInputPtr = ^xmlParserInput;
  xmlParserInputPtrPtr = ^xmlParserInputPtr;
  xmlParserNodeInfoPtr = ^xmlParserNodeInfo;
  xmlParserNodeInfoSeqPtr = ^xmlParserNodeInfoSeq;
  xmlParserCtxtPtr = ^xmlParserCtxt;
  xmlSAXLocatorPtr = ^xmlSAXLocator;
  xmlSAXHandlerPtr = ^xmlSAXHandler;
  xmlSAXHandlerV1Ptr = ^xmlSAXHandlerV1;
{$ENDIF}

{$IFDEF CONST}
(**
 * XML_DEFAULT_VERSION:
 *
 * The default version of XML used: 1.0
 *)
  XML_DEFAULT_VERSION = '1.0';
{$ENDIF}

{$IFDEF TYPE}
(**
 * xmlParserInput:
 *
 * An xmlParserInput is an input flow for the XML processor.
 * Each entity parsed is associated an xmlParserInput (except the
 * few predefined ones). This is the case both for internal entities
 * - in which case the flow is already completely in memory - or
 * external entities - in which case we use the buf structure for
 * progressive reading and I18N conversions to the internal UTF-8 format.
 *)

(**
 * xmlParserInputDeallocate:
 * @str:  the string to deallocate
 *
 * Callback for freeing some parser input allocations.
 *)
  xmlParserInputDeallocate = procedure(str: xmlCharPtr); EXTDECL;

  xmlParserInput = record
    (* Input buffer *)
    buf         : xmlParserInputBufferPtr;      (* UTF-8 encoded buffer *)

    filename    : pchar;             (* The file analyzed, if any *)
    directory   : pchar;            (* the directory/base of the file *)
    base        : xmlCharPtr;              (* Base of the array to parse *)
    cur         : xmlCharPtr;               (* Current char being parsed *)
    _end        : xmlCharPtr;               (* end of the array to parse *)
    length      : cint;                       (* length if known *)
    line        : cint;                         (* Current line *)
    col         : cint;                          (* Current column *)
    (*
     * NOTE: consumed is only tested for equality in the parser code,
     *       so even if there is an overflow this should not give troubles
     *       for parsing very large instances.
     *)
    consumed    : culong;           (* How many xmlChars already consumed *)
    free        : xmlParserInputDeallocate;    (* function to deallocate the base *)
    encoding    : xmlCharPtr;          (* the encoding string for entity *)
    version     : xmlCharPtr;           (* the version string for entity *)
    standalone  : cint;                   (* Was that entity marked standalone *)
    id          : cint;                           (* an unique identifier for the entity *)
  end;

(**
 * xmlParserNodeInfo:
 *
 * The parser can be asked to collect Node informations, i.e. at what
 * place in the file they were detected.
 * NOTE: This is off by default and not very well tested.
 *)
  xmlParserNodeInfo = record
    node        : xmlNodePtr;
  (* Position & line # that text that created the node begins & ends on *)
    begin_pos   : culong;
    begin_line  : culong;
    end_pos     : culong;
    end_line    : culong;
  end;

  xmlParserNodeInfoSeq = record
    maximum     : culong;
    length      : culong;
    buffer      : xmlParserNodeInfoPtr;
  end;

(**
 * xmlParserInputState:
 *
 * The parser is now working also as a state based parser.
 * The recursive one use the state info for entities processing.
 *)
  xmlParserInputState = (
    XML_PARSER_EOF = -1,	(* nothing is to be parsed *)
    XML_PARSER_START = 0,	(* nothing has been parsed *)
    XML_PARSER_MISC,		(* Misc* before int subset *)
    XML_PARSER_PI,		(* Within a processing instruction *)
    XML_PARSER_DTD,		(* within some DTD content *)
    XML_PARSER_PROLOG,		(* Misc* after internal subset *)
    XML_PARSER_COMMENT,		(* within a comment *)
    XML_PARSER_START_TAG,	(* within a start tag *)
    XML_PARSER_CONTENT,		(* within the content *)
    XML_PARSER_CDATA_SECTION,	(* within a CDATA section *)
    XML_PARSER_END_TAG,		(* within a closing tag *)
    XML_PARSER_ENTITY_DECL,	(* within an entity declaration *)
    XML_PARSER_ENTITY_VALUE,	(* within an entity value in a decl *)
    XML_PARSER_ATTRIBUTE_VALUE,	(* within an attribute value *)
    XML_PARSER_SYSTEM_LITERAL,	(* within a SYSTEM value *)
    XML_PARSER_EPILOG, 		(* the Misc* after the last end tag *)
    XML_PARSER_IGNORE,		(* within an IGNORED section *)
    XML_PARSER_PUBLIC_LITERAL 	(* within a PUBLIC value *)
  );
{$ENDIF}

{$IFDEF CONST}
(**
 * XML_DETECT_IDS:
 *
 * Bit in the loadsubset context field to tell to do ID/REFs lookups.
 * Use it to initialize xmlLoadExtDtdDefaultValue.
 *)
  XML_DETECT_IDS = 2;

(**
 * XML_COMPLETE_ATTRS:
 *
 * Bit in the loadsubset context field to tell to do complete the
 * elements attributes lists with the ones defaulted from the DTDs.
 * Use it to initialize xmlLoadExtDtdDefaultValue.
 *)
  XML_COMPLETE_ATTRS = 4;

(**
 * XML_SKIP_IDS:
 *
 * Bit in the loadsubset context field to tell to not do ID/REFs registration.
 * Used to initialize xmlLoadExtDtdDefaultValue in some special cases.
 *)
  XML_SKIP_IDS = 8;
{$ENDIF}

{$IFDEF TYPE}
(**
 * xmlParserMode:
 *
 * A parser can operate in various modes
 *)
  xmlParserMode = (
    XML_PARSE_UNKNOWN = 0,
    XML_PARSE_DOM = 1,
    XML_PARSE_SAX = 2,
    XML_PARSE_PUSH_DOM = 3,
    XML_PARSE_PUSH_SAX = 4,
    XML_PARSE_READER = 5
  );

(**
 * xmlParserCtxt:
 *
 * The parser context.
 * NOTE This doesn't completely define the parser state, the (current ?)
 *      design of the parser uses recursive function calls since this allow
 *      and easy mapping from the production rules of the specification
 *      to the actual code. The drawback is that the actual function call
 *      also reflect the parser state. However most of the parsing routines
 *      takes as the only argument the parser context pointer, so migrating
 *      to a state based parser for progressive parsing shouldn't be too hard.
 *)
  xmlParserCtxt = record
    sax               : xmlSAXHandlerPtr;       (* The SAX handler *)
    userData          : pointer;        (* For SAX interface only, used by DOM build *)
    myDoc             : xmlDocPtr;        (* the document being built *)
    wellFormed        : cint;        (* is the document well formed *)
    replaceEntities   : cint;        (* shall we replace entities ? *)
    version           : xmlCharPtr;        (* the XML version string *)
    encoding          : xmlCharPtr;        (* the declared encoding, if any *)
    standalone        : cint;        (* standalone document *)
    html              : cint;        (* an HTML(1)/Docbook(2) document *)

    (* Input stream stack *)
    input             : xmlParserInputPtr;         (* Current input stream *)
    inputNr           : cint;       (* Number of current input streams *)
    inputMax          : cint;      (* Max number of input streams *)
    inputTab          : xmlParserInputPtrPtr;      (* stack of inputs *)

    (* Node analysis stack only used for DOM building *)
    node              : xmlNodePtr;          (* Current parsed Node *)
    nodeNr            : cint;        (* Depth of the parsing stack *)
    nodeMax           : cint;       (* Max depth of the parsing stack *)
    nodeTab           : xmlNodePtrPtr;       (* array of nodes *)

    record_info       : cint;                  (* Whether node info should be kept *)
    node_seq          : xmlParserNodeInfoSeq;    (* info about each node parsed *)

    errNo             : cint;                        (* error code *)

    hasExternalSubset : cint;        (* reference and external subset *)
    hasPErefs         : cint;        (* the internal subset has PE refs *)
    external          : cint;        (* are we parsing an external entity *)

    valid             : cint;        (* is the document valid *)
    validate          : cint;        (* shall we try to validate ? *)
    vctxt             : xmlValidCtxt;        (* The validity context *)

    instate           : xmlParserInputState;      (* current type of input *)
    token             : cint;        (* next char look-ahead *)

    directory         : pchar;        (* the data directory *)

    (* Node name stack *)
    name              : xmlCharPtr;          (* Current parsed Node *)
    nameNr            : cint;        (* Depth of the parsing stack *)
    nameMax           : cint;       (* Max depth of the parsing stack *)
    nameTab           : xmlCharPtrPtr;       (* array of nodes *)

    nbChars           : culong;       (* number of xmlChar processed *)
    checkIndex        : culong;       (* used by progressive parsing lookup *)
    keepBlanks        : cint;       (* ugly but ... *)
    disableSAX        : cint;       (* SAX callbacks are disabled *)
    inSubset          : cint;       (* Parsing is in int 1/ext 2 subset *)
    intSubName        : xmlCharPtr;    (* name of subset *)
    extSubURI         : xmlCharPtr;     (* URI of external subset *)
    extSubSystem      : xmlCharPtr;  (* SYSTEM ID of external subset *)

    (* xml:space values *)
    space             : pcint;         (* Should the parser preserve spaces *)
    spaceNr           : cint;       (* Depth of the parsing stack *)
    spaceMax          : cint;      (* Max depth of the parsing stack *)
    spaceTab          : pcint;      (* array of space infos *)

    depth             : cint;         (* to prevent entity substitution loops *)
    entity            : xmlParserInputPtr;        (* used to check entities boundaries *)
    charset           : cint;       (* encoding of the in-memory content
				         actually an xmlCharEncoding *)
    nodelen           : cint;       (* Those two fields are there to *)
    nodemem           : cint;       (* Speed up large node parsing *)
    pedantic          : cint;      (* signal pedantic warnings *)
    _private          : pointer;      (* For user data, libxml won't touch it *)

    loadsubset        : cint;    (* should the external subset be loaded *)
    linenumbers       : cint;   (* set line number in element content *)
    catalogs          : pointer;      (* document's own catalog *)
    recovery          : cint;      (* run in recovery mode *)
    progressive       : cint;   (* is this a progressive parsing *)
    dict              : xmlDictPtr;          (* dictionnary for the parser *)
    atts              : xmlCharPtrPtr;          (* array for the attributes callbacks *)
    maxatts           : cint;       (* the size of the array *)
    docdict           : cint;       (* use strings from dict to build tree *)

    (*
     * pre-interned strings
     *)
    str_xml           : xmlCharPtr;
    str_xmlns         : xmlCharPtr;
    str_xml_ns        : xmlCharPtr;

    (*
     * Everything below is used only by the new SAX mode
     *)
    sax2              : cint;          (* operating in the new SAX mode *)
    nsNr              : cint;          (* the number of inherited namespaces *)
    nsMax             : cint;         (* the size of the arrays *)
    nsTab             : xmlCharPtrPtr;         (* the array of prefix/namespace name *)
    attallocs         : pcint;     (* which attribute were allocated *)
    pushTab           : ppointer;       (* array of data for push *)
    attsDefault       : xmlHashTablePtr;   (* defaulted attributes if any *)
    attsSpecial       : xmlHashTablePtr;   (* non-CDATA attributes if any *)
    nsWellFormed      : cint;  (* is the document XML Nanespace okay *)
    options           : cint;       (* Extra options *)

    (*
     * Those fields are needed only for treaming parsing so far
     *)
    dictNames         : cint;    (* Use dictionary names for the tree *)
    freeElemsNr       : cint;  (* number of freed element nodes *)
    freeElems         : xmlNodePtr;    (* List of freed element nodes *)
    freeAttrsNr       : cint;  (* number of freed attributes nodes *)
    freeAttrs         : xmlAttrPtr;    (* List of freed attributes nodes *)

    (*
     * the complete error informations for the last error.
     *)
    lastError         : xmlError;
    parseMode         : xmlParserMode;    (* the parser mode *)
  end;

(**
 * xmlSAXLocator:
 *
 * A SAX Locator.
 *)
  getPublicIdFunc = function(ctx: pointer): xmlCharPtr; EXTDECL;
  getSystemIdFunc = function(ctx: pointer): xmlCharPtr; EXTDECL;
  getLineNumberFunc = function(ctx: pointer): cint; EXTDECL;
  getColumnNumberFunc = function(ctx: pointer): cint; EXTDECL;

  xmlSAXLocator = record
    getPublicId     : getPublicIdFunc;
    getSystemId     : getSystemIdFunc;
    getLineNumber   : getLineNumberFunc;
    getColumnNumber : getColumnNumberFunc;
  end;

(**
 * xmlSAXHandler:
 *
 * A SAX handler is bunch of callbacks called by the parser when processing
 * of the input generate data or structure informations.
 *)

(**
 * resolveEntitySAXFunc:
 * @ctx:  the user data (XML parser context)
 * @publicId: The public ID of the entity
 * @systemId: The system ID of the entity
 *
 * Callback:
 * The entity loader, to control the loading of external entities,
 * the application can either:
 *    - override this resolveEntity() callback in the SAX block
 *    - or better use the xmlSetExternalEntityLoader() function to
 *      set up it's own entity resolution routine
 *
 * Returns the xmlParserInputPtr if inlined or NULL for DOM behaviour.
 *)
  resolveEntitySAXFunc = function(ctx: pointer; publicID, systemID: xmlCharPtr): xmlParserInputPtr; EXTDECL;

(**
 * internalSubsetSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  the root element name
 * @ExternalID:  the external ID
 * @SystemID:  the SYSTEM ID (e.g. filename or URL)
 *
 * Callback on internal subset declaration.
 *)
  internalSubsetSAXFunc = procedure(ctx: pointer; name, ExternalID, SystemID: xmlCharPtr); EXTDECL;

(**
 * externalSubsetSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  the root element name
 * @ExternalID:  the external ID
 * @SystemID:  the SYSTEM ID (e.g. filename or URL)
 *
 * Callback on external subset declaration.
 *)
  externalSubsetSAXFunc = procedure(ctx: pointer; name, ExternalID, SystemID: xmlCharPtr); EXTDECL;

(**
 * getEntitySAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name: The entity name
 *
 * Get an entity by name.
 *
 * Returns the xmlEntityPtr if found.
 *)
  getEntitySAXFunc = function(ctx: pointer; name: xmlCharPtr): xmlEntityPtr; EXTDECL;

(**
 * getParameterEntitySAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name: The entity name
 *
 * Get a parameter entity by name.
 *
 * Returns the xmlEntityPtr if found.
 *)
  getParameterEntitySAXFunc = function(ctx: pointer; name: xmlCharPtr): xmlEntityPtr; EXTDECL;

(**
 * entityDeclSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  the entity name
 * @type:  the entity type
 * @publicId: The public ID of the entity
 * @systemId: The system ID of the entity
 * @content: the entity value (without processing).
 *
 * An entity definition has been parsed.
 *)
  entityDeclSAXFunc = procedure(ctx: pointer; name: xmlCharPtr; _type: cint; publicId, systemId, content: xmlCharPtr); EXTDECL;

(**
 * notationDeclSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name: The name of the notation
 * @publicId: The public ID of the entity
 * @systemId: The system ID of the entity
 *
 * What to do when a notation declaration has been parsed.
 *)
  notationDeclSAXFunc = procedure(ctx: pointer; name, publicId, systemId: xmlCharPtr); EXTDECL;

(**
 * attributeDeclSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @elem:  the name of the element
 * @fullname:  the attribute name
 * @type:  the attribute type
 * @def:  the type of default value
 * @defaultValue: the attribute default value
 * @tree:  the tree of enumerated value set
 *
 * An attribute definition has been parsed.
 *)
  attributeDeclSAXFunc = procedure(ctx: pointer; elem, fullname: xmlCharPtr; _type, def: cint; defaultValue: xmlCharPtr; tree: xmlEnumerationPtr); EXTDECL;

(**
 * elementDeclSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  the element name
 * @type:  the element type
 * @content: the element value tree
 *
 * An element definition has been parsed.
 *)
  elementDeclSAXFunc = procedure(ctx: pointer; name: xmlCharPtr; _type: cint; content: xmlElementContentPtr); EXTDECL;

(**
 * unparsedEntityDeclSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name: The name of the entity
 * @publicId: The public ID of the entity
 * @systemId: The system ID of the entity
 * @notationName: the name of the notation
 *
 * What to do when an unparsed entity declaration is parsed.
 *)
  unparsedEntityDeclSAXFunc = procedure(ctx: pointer; name, publicId, systemId, notationName: xmlCharPtr); EXTDECL;

(**
 * setDocumentLocatorSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @loc: A SAX Locator
 *
 * Receive the document locator at startup, actually xmlDefaultSAXLocator.
 * Everything is available on the context, so this is useless in our case.
 *)
  setDocumentLocatorSAXFunc = procedure(ctx: pointer; loc: xmlSAXLocatorPtr); EXTDECL;

(**
 * startDocumentSAXFunc:
 * @ctx:  the user data (XML parser context)
 *
 * Called when the document start being processed.
 *)
  startDocumentSAXFunc = procedure(ctx: pointer); EXTDECL;

(**
 * endDocumentSAXFunc:
 * @ctx:  the user data (XML parser context)
 *
 * Called when the document end has been detected.
 *)
  endDocumentSAXFunc = procedure(ctx: pointer); EXTDECL;

(**
 * startElementSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  The element name, including namespace prefix
 * @atts:  An array of name/value attributes pairs, NULL terminated
 *
 * Called when an opening tag has been processed.
 *)
  startElementSAXFunc = procedure(ctx: pointer; name: xmlCharPtr; atts: xmlCharPtrPtr); EXTDECL;

(**
 * endElementSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  The element name
 *
 * Called when the end of an element has been detected.
 *)
  endElementSAXFunc = procedure(ctx: pointer; name: xmlCharPtr); EXTDECL;

(**
 * attributeSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  The attribute name, including namespace prefix
 * @value:  The attribute value
 *
 * Handle an attribute that has been read by the parser.
 * The default handling is to convert the attribute into an
 * DOM subtree and past it in a new xmlAttr element added to
 * the element.
 *)
  attributeSAXFunc = procedure(ctx: pointer; name, value: xmlCharPtr); EXTDECL;

(**
 * referenceSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @name:  The entity name
 *
 * Called when an entity reference is detected.
 *)
  referenceSAXFunc = procedure(ctx: pointer; name: xmlCharPtr); EXTDECL;

(**
 * charactersSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @ch:  a xmlChar string
 * @len: the number of xmlChar
 *
 * Receiving some chars from the parser.
 *)
  charactersSAXFunc = procedure(ctx: pointer; ch: xmlCharPtr; len: cint); EXTDECL;

(**
 * ignorableWhitespaceSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @ch:  a xmlChar string
 * @len: the number of xmlChar
 *
 * Receiving some ignorable whitespaces from the parser.
 * UNUSED: by default the DOM building will use characters.
 *)
  ignorableWhitespaceSAXFunc = procedure(ctx: pointer; ch: xmlCharPtr; len: cint); EXTDECL;

(**
 * processingInstructionSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @target:  the target name
 * @data: the PI data's
 *
 * A processing instruction has been parsed.
 *)
  processingInstructionSAXFunc = procedure(ctx: pointer; target, data: xmlCharPtr); EXTDECL;

(**
 * commentSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @value:  the comment content
 *
 * A comment has been parsed.
 *)
  commentSAXFunc = procedure(ctx: pointer; value: xmlCharPtr); EXTDECL;

(**
 * cdataBlockSAXFunc:
 * @ctx:  the user data (XML parser context)
 * @value:  The pcdata content
 * @len:  the block length
 *
 * Called when a pcdata block has been parsed.
 *)
  cdataBlockSAXFunc = procedure(ctx: pointer; value: xmlCharPtr; len: cint); EXTDECL;

(**
 * warningSAXFunc:
 * @ctx:  an XML parser context
 * @msg:  the message to display/transmit
 * @...:  extra parameters for the message display
 *
 * Display and format a warning messages, callback.
 *)
  warningSAXFunc = procedure(ctx: pointer; msg: pchar); cdecl; varargs;

(**
 * errorSAXFunc:
 * @ctx:  an XML parser context
 * @msg:  the message to display/transmit
 * @...:  extra parameters for the message display
 *
 * Display and format an error messages, callback.
 *)
  errorSAXFunc = procedure(ctx: pointer; msg: pchar); cdecl; varargs;

(**
 * fatalErrorSAXFunc:
 * @ctx:  an XML parser context
 * @msg:  the message to display/transmit
 * @...:  extra parameters for the message display
 *
 * Display and format fatal error messages, callback.
 * Note: so far fatalError() SAX callbacks are not used, error()
 *       get all the callbacks for errors.
 *)
  fatalErrorSAXFunc = procedure(ctx: pointer; msg: pchar); cdecl; varargs;

(**
 * isStandaloneSAXFunc:
 * @ctx:  the user data (XML parser context)
 *
 * Is this document tagged standalone?
 *
 * Returns 1 if true
 *)
  isStandaloneSAXFunc = function(ctx: pointer): cint; EXTDECL;

(**
 * hasInternalSubsetSAXFunc:
 * @ctx:  the user data (XML parser context)
 *
 * Does this document has an internal subset.
 *
 * Returns 1 if true
 *)
  hasInternalSubsetSAXFunc = function(ctx: pointer): cint; EXTDECL;

(**
 * hasExternalSubsetSAXFunc:
 * @ctx:  the user data (XML parser context)
 *
 * Does this document has an external subset?
 *
 * Returns 1 if true
 *)
  hasExternalSubsetSAXFunc = function(ctx: pointer): cint; EXTDECL;
{$ENDIF}

(************************************************************************
 *									*
 *			The SAX version 2 API extensions		*
 *									*
 ************************************************************************)
(**
 * XML_SAX2_MAGIC:
 *
 * Special constant found in SAX2 blocks initialized fields
 *)
{$IFDEF CONST}
  XML_SAX2_MAGIC = $DEEDBEAF;
{$ENDIF}

{$IFDEF TYPE}
(**
 * startElementNsSAX2Func:
 * @ctx:  the user data (XML parser context)
 * @localname:  the local name of the element
 * @prefix:  the element namespace prefix if available
 * @URI:  the element namespace name if available
 * @nb_namespaces:  number of namespace definitions on that node
 * @namespaces:  pointer to the array of prefix/URI pairs namespace definitions
 * @nb_attributes:  the number of attributes on that node
 * @nb_defaulted:  the number of defaulted attributes. The defaulted
 *                  ones are at the end of the array
 * @attributes:  pointer to the array of (localname/prefix/URI/value/end)
 *               attribute values.
 *
 * SAX2 callback when an element start has been detected by the parser.
 * It provides the namespace informations for the element, as well as
 * the new namespace declarations on the element.
 *)
  startElementNsSAX2Func = procedure(ctx: pointer; localname, prefix, URI: xmlCharPtr; nb_namespaces: cint;
    namespaces: xmlCharPtrPtr; nb_attributes, nb_defaulted: cint; attributes: xmlCharPtrPtr); EXTDECL;

(**
 * endElementNsSAX2Func:
 * @ctx:  the user data (XML parser context)
 * @localname:  the local name of the element
 * @prefix:  the element namespace prefix if available
 * @URI:  the element namespace name if available
 *
 * SAX2 callback when an element end has been detected by the parser.
 * It provides the namespace informations for the element.
 *)
  endElementNsSAX2Func = procedure(ctx: pointer; localname, prefix, URI: xmlCharPtr); EXTDECL;

  xmlSAXHandler = record
    internalSubset: internalSubsetSAXFunc;
    isStandalone: isStandaloneSAXFunc;
    hasInternalSubset: hasInternalSubsetSAXFunc;
    hasExternalSubset: hasExternalSubsetSAXFunc;
    resolveEntity: resolveEntitySAXFunc;
    getEntity: getEntitySAXFunc;
    entityDecl: entityDeclSAXFunc;
    notationDecl: notationDeclSAXFunc;
    attributeDecl: attributeDeclSAXFunc;
    elementDecl: elementDeclSAXFunc;
    unparsedEntityDecl: unparsedEntityDeclSAXFunc;
    setDocumentLocator: setDocumentLocatorSAXFunc;
    startDocument: startDocumentSAXFunc;
    endDocument: endDocumentSAXFunc;
    startElement: startElementSAXFunc;
    endElement: endElementSAXFunc;
    reference: referenceSAXFunc;
    characters: charactersSAXFunc;
    ignorableWhitespace: ignorableWhitespaceSAXFunc;
    processingInstruction: processingInstructionSAXFunc;
    comment: commentSAXFunc;
    warning: warningSAXFunc;
    error: errorSAXFunc;
    fatalError: fatalErrorSAXFunc; (* unused error() get all the errors *)
    getParameterEntity: getParameterEntitySAXFunc;
    cdataBlock: cdataBlockSAXFunc;
    externalSubset: externalSubsetSAXFunc;
    initialized: cuint;
    (* The following fields are extensions available only on version 2 *)
    _private: pointer;
    startElementNs: startElementNsSAX2Func;
    endElementNs: endElementNsSAX2Func;
    serror: xmlStructuredErrorFunc;
  end;

(*
 * SAX Version 1
 *)
  xmlSAXHandlerV1 = record
    internalSubset: internalSubsetSAXFunc;
    isStandalone: isStandaloneSAXFunc;
    hasInternalSubset: hasInternalSubsetSAXFunc;
    hasExternalSubset: hasExternalSubsetSAXFunc;
    resolveEntity: resolveEntitySAXFunc;
    getEntity: getEntitySAXFunc;
    entityDecl: entityDeclSAXFunc;
    notationDecl: notationDeclSAXFunc;
    attributeDecl: attributeDeclSAXFunc;
    elementDecl: elementDeclSAXFunc;
    unparsedEntityDecl: unparsedEntityDeclSAXFunc;
    setDocumentLocator: setDocumentLocatorSAXFunc;
    startDocument: startDocumentSAXFunc;
    endDocument: endDocumentSAXFunc;
    startElement: startElementSAXFunc;
    endElement: endElementSAXFunc;
    reference: referenceSAXFunc;
    characters: charactersSAXFunc;
    ignorableWhitespace: ignorableWhitespaceSAXFunc;
    processingInstruction: processingInstructionSAXFunc;
    comment: commentSAXFunc;
    warning: warningSAXFunc;
    error: errorSAXFunc;
    fatalError: fatalErrorSAXFunc; (* unused error() get all the errors *)
    getParameterEntity: getParameterEntitySAXFunc;
    cdataBlock: cdataBlockSAXFunc;
    externalSubset: externalSubsetSAXFunc;
    initialized: cuint;
  end;


(**
 * xmlExternalEntityLoader:
 * @URL: The System ID of the resource requested
 * @ID: The Public ID of the resource requested
 * @context: the XML parser context
 *
 * External entity loaders types.
 *
 * Returns the entity input parser.
 *)
  xmlExternalEntityLoader = function(URL, ID: pchar; context: xmlParserCtxtPtr): xmlParserInputPtr; EXTDECL;

(**
 * xmlParserOption:
 *
 * This is the set of XML parser options that can be passed down
 * to the xmlReadDoc() and similar calls.
 *)
  xmlParserOption = type cint;
{$ENDIF}
{$IFDEF CONST}
  XML_PARSE_RECOVER = (1 shl 0); (* recover on errors *)
  XML_PARSE_NOENT = (1 shl 1); (* substitute entities *)
  XML_PARSE_DTDLOAD = (1 shl 2); (* load the external subset *)
  XML_PARSE_DTDATTR = (1 shl 3); (* default DTD attributes *)
  XML_PARSE_DTDVALID  = (1 shl 4); (* validate with the DTD *)
  XML_PARSE_NOERROR = (1 shl 5); (* suppress error reports *)
  XML_PARSE_NOWARNING = (1 shl 6); (* suppress warning reports *)
  XML_PARSE_PEDANTIC  = (1 shl 7); (* pedantic error reporting *)
  XML_PARSE_NOBLANKS  = (1 shl 8); (* remove blank nodes *)
  XML_PARSE_SAX1  = (1 shl 9); (* use the SAX1 interface internally *)
  XML_PARSE_XINCLUDE  = (1 shl 10);(* Implement XInclude substitition  *)
  XML_PARSE_NONET = (1 shl 11);(* Forbid network access *)
  XML_PARSE_NODICT  = (1 shl 12);(* Do not reuse the context dictionnary *)
  XML_PARSE_NSCLEAN = (1 shl 13);(* remove redundant namespaces declarations *)
  XML_PARSE_NOCDATA = (1 shl 14);(* merge CDATA as text nodes *)
  XML_PARSE_NOXINCNODE = (1 shl 15);(* do not generate XINCLUDE START/END nodes *)
  XML_PARSE_COMPACT   = (1 shl 16); (* compact small text nodes); no modification of
                                   the tree allowed afterwards (will possibly
           crash if you try to modify the tree) *)
{$ENDIF}

{$IFDEF TYPE}
  xmlFeature = (
    XML_WITH_THREAD = 1,
    XML_WITH_TREE = 2,
    XML_WITH_OUTPUT = 3,
    XML_WITH_PUSH = 4,
    XML_WITH_READER = 5,
    XML_WITH_PATTERN = 6,
    XML_WITH_WRITER = 7,
    XML_WITH_SAX1 = 8,
    XML_WITH_FTP = 9,
    XML_WITH_HTTP = 10,
    XML_WITH_VALID = 11,
    XML_WITH_HTML = 12,
    XML_WITH_LEGACY = 13,
    XML_WITH_C14N = 14,
    XML_WITH_CATALOG = 15,
    XML_WITH_XPATH = 16,
    XML_WITH_XPTR = 17,
    XML_WITH_XINCLUDE = 18,
    XML_WITH_ICONV = 19,
    XML_WITH_ISO8859X = 20,
    XML_WITH_UNICODE = 21,
    XML_WITH_REGEXP = 22,
    XML_WITH_AUTOMATA = 23,
    XML_WITH_EXPR = 24,
    XML_WITH_SCHEMAS = 25,
    XML_WITH_SCHEMATRON = 26,
    XML_WITH_MODULES = 27,
    XML_WITH_DEBUG = 28,
    XML_WITH_DEBUG_MEM = 29,
    XML_WITH_DEBUG_RUN = 30,
    XML_WITH_ZLIB = 31,
    XML_WITH_NONE = 99999 (* just to be sure of allocation size *)
  );

{$ENDIF}

{$IFDEF FUNCTION}
(*
 * Init/Cleanup
 *)
procedure xmlInitParser; EXTDECL; external xml2lib;
procedure xmlCleanupParser; EXTDECL; external xml2lib;

(*
 * Input functions
 *)
function xmlParserInputRead(_in: xmlParserInputPtr; len: cint): cint; EXTDECL; external xml2lib;
function xmlParserInputGrow(_in: xmlParserInputPtr; len: cint): cint; EXTDECL; external xml2lib;

(*
 * Basic parsing Interfaces
 *)
{$IFDEF LIBXML_SAX1_ENABLED}
function xmlParseDoc(cur: xmlCharPtr): xmlDocPtr; EXTDECL; external xml2lib;
function xmlParseFile(filename: pchar): xmlDocPtr; EXTDECL; external xml2lib;
function xmlParseMemory(buffer: pchar; size: cint): xmlDocPtr; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_SAX1_ENABLED *)
function xmlSubstituteEntitiesDefault(val: cint): cint; EXTDECL; external xml2lib;
function xmlKeepBlanksDefault(val: cint): cint; EXTDECL; external xml2lib;
procedure xmlStopParser(ctxt: xmlParserCtxtPtr); EXTDECL; external xml2lib;
function xmlPedanticParserDefault(val: cint): cint; EXTDECL; external xml2lib;
function xmlLineNumbersDefault(val: cint): cint; EXTDECL; external xml2lib;

{$IFDEF LIBXML_SAX1_ENABLED}
(*
 * Recovery mode
 *)
function xmlRecoverDoc(cur: xmlCharPtr): xmlDocPtr; EXTDECL; external xml2lib;
function xmlRecoverMemory(buffer: pchar; size: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlRecoverFile(filename: pchar): xmlDocPtr; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_SAX1_ENABLED *)

(*
 * Less common routines and SAX interfaces
 *)
function xmlParseDocument(ctxt: xmlParserCtxtPtr): cint; EXTDECL; external xml2lib;
function xmlParseExtParsedEnt(ctxt: xmlParserCtxtPtr): cint; EXTDECL; external xml2lib;
{$IFDEF LIBXML_SAX1_ENABLED}
function xmlSAXUserParseFile(sax: xmlSAXHandlerPtr; user_data: pointer; filename: pchar): cint; EXTDECL; external xml2lib;
function xmlSAXUserParseMemory(sax: xmlSAXHandlerPtr; user_data: pointer; buffer: pchar; size: cint): cint; EXTDECL; external xml2lib;
function xmlSAXParseDoc(sax: xmlSAXHandlerPtr; cur: xmlCharPtr; recovery: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlSAXParseMemory(sax: xmlSAXHandlerPtr; buffer: pchar; size: cint; recovery: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlSAXParseMemoryWithData(sax: xmlSAXHandlerPtr; buffer: pchar; size: cint; recovery: cint; data: pointer): xmlDocPtr; EXTDECL; external xml2lib;
function xmlSAXParseFile(sax: xmlSAXHandlerPtr; filename: pchar; recovery: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlSAXParseFileWithData(sax: xmlSAXHandlerPtr; filename: pchar; recovery: cint; data: pointer): xmlDocPtr; EXTDECL; external xml2lib;
function xmlSAXParseEntity(sax: xmlSAXHandlerPtr; filename: pchar): xmlDocPtr; EXTDECL; external xml2lib;
function xmlParseEntity(filename: pchar): xmlDocPtr; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_SAX1_ENABLED *)

{$IFDEF LIBXML_VALID_ENABLED}
function xmlSAXParseDTD(sax: xmlSAXHandlerPtr; ExternalID, SystemID: xmlCharPtr): xmlDtdPtr; EXTDECL; external xml2lib;
function xmlParseDTD(ExternalID, SystemID: xmlCharPtr): xmlDtdPtr; EXTDECL; external xml2lib;
function xmlIOParseDTD(sax: xmlSAXHandlerPtr; input: xmlParserInputBufferPtr; enc: xmlCharEncoding): xmlDtdPtr; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_VALID_ENABLE *)
{$IFDEF LIBXML_SAX1_ENABLED}
function xmlParseBalancedChunkMemory(doc: xmlDocPtr; sax: xmlSAXHandlerPtr; user_data: pointer; depth: cint; _string: xmlCharPtr; lst: xmlNodePtrPtr): cint; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_SAX1_ENABLED *)
function xmlParseInNodeContext(node: xmlNodePtr; data: pchar; datalen, options: cint; lst: xmlNodePtrPtr): xmlParserErrors; EXTDECL; external xml2lib;
{$IFDEF LIBXML_SAX1_ENABLED}
function xmlParseBalancedChunkMemoryRecover(doc: xmlDocPtr; sax: xmlSAXHandlerPtr; user_data: pointer;
  depth: cint; _string: xmlCharPtr; lst: xmlNodePtrPtr; recover: cint): cint; EXTDECL; external xml2lib;
function xmlParseExternalEntity(doc: xmlDocPtr; sax: xmlSAXHandlerPtr; user_data: pointer;
  depth: cint; URL, ID: xmlCharPtr; lst: xmlNodePtrPtr): cint; EXTDECL; external xml2lib;

{$ENDIF} (* LIBXML_SAX1_ENABLED *)
function xmlParseCtxtExternalEntity(ctx: xmlParserCtxtPtr; URL, ID: xmlCharPtr; lst: xmlNodePtrPtr): cint; EXTDECL; external xml2lib;

(*
 * Parser contexts handling.
 *)
function xmlNewParserCtxt: xmlParserCtxtPtr; EXTDECL; external xml2lib;
function xmlInitParserCtxt(ctxt: xmlParserCtxtPtr): cint; EXTDECL; external xml2lib;
procedure xmlClearParserCtxt(ctxt: xmlParserCtxtPtr); EXTDECL; external xml2lib;
procedure xmlFreeParserCtxt(ctxt: xmlParserCtxtPtr); EXTDECL; external xml2lib;
{$IFDEF LIBXML_SAX1_ENABLED}
procedure xmlSetupParserForBuffer(ctxt: xmlParserCtxtPtr; buffer: xmlCharPtr; filename: pchar); EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_SAX1_ENABLED *)
function xmlCreateDocParserCtxt(cur: xmlCharPtr): xmlParserCtxtPtr; EXTDECL; external xml2lib;

{$IFDEF LIBXML_LEGACY_ENABLED}
(*
 * Reading/setting optional parsing features.
 *)
function xmlGetFeaturesList(var len: cint; var result: pchar): cint; EXTDECL; external xml2lib;
function xmlGetFeature(ctxt: xmlParserCtxtPtr; name: pchar; result: pointer): cint; EXTDECL; external xml2lib;
function xmlSetFeature(ctxt: xmlParserCtxtPtr; name: pchar; value: pointer): cint; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_LEGACY_ENABLED *)

{$IFDEF LIBXML_PUSH_ENABLED}
(*
 * Interfaces for the Push mode.
 *)
function xmlCreatePushParserCtxt(sax: xmlSAXHandlerPtr; user_data: pointer; chunk: pchar; size: cint; filename: pchar): xmlParserCtxtPtr; EXTDECL; external xml2lib;
function xmlParseChunk(ctxt: xmlParserCtxtPtr; chunk: pchar; size, terminate: cint): cint; EXTDECL; external xml2lib;
{$ENDIF} (* LIBXML_PUSH_ENABLED *)

(*
 * Special I/O mode.
 *)
function xmlCreateIOParserCtxt(sax: xmlSAXHandlerPtr; user_data: pointer; ioread: xmlInputReadCallback;
  ioclose: xmlInputCloseCallback; ioctx: pointer; enc: xmlCharEncoding): xmlParserCtxtPtr; EXTDECL; external xml2lib;
function xmlNewIOInputStream(ctxt: xmlParserCtxtPtr; input: xmlParserInputBufferPtr; enc: xmlCharEncoding): xmlParserInputPtr; EXTDECL; external xml2lib;

(*
 * Node infos.
 *)
function xmlParserFindNodeInfo(ctxt: xmlParserCtxtPtr; node: xmlNodePtr): xmlParserNodeInfoPtr; EXTDECL; external xml2lib;
procedure xmlInitNodeInfoSeq(seq: xmlParserNodeInfoSeqPtr); EXTDECL; external xml2lib;
procedure xmlClearNodeInfoSeq(seq: xmlParserNodeInfoSeqPtr); EXTDECL; external xml2lib;
function xmlParserFindNodeInfoIndex(seq: xmlParserNodeInfoSeqPtr; node: xmlNodePtr): culong; EXTDECL; external xml2lib;
procedure xmlParserAddNodeInfo(ctxt: xmlParserCtxtPtr; info: xmlParserNodeInfoPtr); EXTDECL; external xml2lib;

(*
 * External entities handling actually implemented in xmlIO.
 *)
procedure xmlSetExternalEntityLoader(f: xmlExternalEntityLoader); EXTDECL; external xml2lib;
function xmlGetExternalEntityLoader(): xmlExternalEntityLoader; EXTDECL; external xml2lib;
function xmlLoadExternalEntity(URL, ID: pchar; ctxt: xmlParserCtxtPtr): xmlParserInputPtr; EXTDECL; external xml2lib;

(*
 * Index lookup, actually implemented in the encoding module
 *)
function xmlByteConsumed(ctxt: xmlParserCtxtPtr): culong; EXTDECL; external xml2lib;

(*
 * New set of simpler/more flexible APIs
 *)
procedure xmlCtxtReset(ctxt: xmlParserCtxtPtr); EXTDECL; external xml2lib;
function xmlCtxtResetPush(ctxt: xmlParserCtxtPtr; chunk: pchar; size: cint; filename, encoding: pchar): cint; EXTDECL; external xml2lib;
function xmlCtxtUseOptions(ctxt: xmlParserCtxtPtr; options: cint): cint; EXTDECL; external xml2lib;
function xmlReadDoc(cur: xmlCharPtr; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlReadFile(filename, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlReadMemory(buffer: pchar; size: cint; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlReadFd(fd: cint; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlReadIO(ioread: xmlInputReadCallback; ioclose: xmlInputCloseCallback; ioctx: pointer; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlCtxtReadDoc(ctxt: xmlParserCtxtPtr; cur: xmlCharPtr; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlCtxtReadFile(ctxt: xmlParserCtxtPtr; filename, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlCtxtReadMemory(ctxt: xmlParserCtxtPtr; buffer: pchar; size: cint; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlCtxtReadFd(ctxt: xmlParserCtxtPtr; fd: cint; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;
function xmlCtxtReadIO(ctxt: xmlParserCtxtPtr; ioread: xmlInputReadCallback; ioclose: xmlInputCloseCallback; ioctx: pchar; URL, encoding: pchar; options: cint): xmlDocPtr; EXTDECL; external xml2lib;

(*
 * Library wide options
 *)
(**
 * xmlFeature:
 *
 * Used to examine the existance of features that can be enabled
 * or disabled at compile-time.
 * They used to be called XML_FEATURE_xxx but this clashed with Expat
 *)
function xmlHasFeature(feature: xmlFeature): cint; EXTDECL; external xml2lib;
{$ENDIF}

