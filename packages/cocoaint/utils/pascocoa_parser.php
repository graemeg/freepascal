<?php
function ReadTextFile ($path) {
	if ($handle = fopen($path, "r")) {
		return fread($handle, 400 * 1024);
		fclose($handle);
	}
}
	
function str_replace_word ($needle, $replacement, $haystack) {
    $pattern = "/\b$needle\b/";
    $haystack = preg_replace($pattern, $replacement, $haystack);
    return $haystack;
}

function istr_replace_word ($needle, $replacement, $haystack) {
    $pattern = "/\b$needle\b/i";
    $haystack = preg_replace($pattern, $replacement, $haystack);
    return $haystack;
}

define("CAST_HANDLE", true);
define("DONT_CAST_HANDLE", false);

define("ACCESS_HANDLE_DIRECT", 1);
define("ACCESS_HANDLE_FUNCTION", 2);

define("REGISTER_SEL", true);
define("DONT_REGISTER_SEL", false);

define("USE_HANDLE", true);
define("DONT_USE_HANDLE", false);
	
class TPasCocoaParser {

	// Frameworks to parse
	var $frameworks = array(	"foundation" => array(	"root" => "/foundation/Foundation.inc", 
															"bridge" => "/bridgesupport/foundation.xml",
															"headers" => "/System/Library/Frameworks/Foundation.framework/Headers",
															"include_pattern" => "{[$]+include (NS.*).inc}",
															"header_pattern" => "^NS(.*)\.h",
															"enabled" => false,
														),
	
								"appkit" => array(		"root" => "/appkit/AppKit.inc", 
														"bridge" => "/bridgesupport/appkit.xml",
														"headers" => "/System/Library/Frameworks/AppKit.framework/Headers",
														"include_pattern" => "{[$]+include (NS.*).inc}",
														"header_pattern" => "^NS(.*)\.h",
														"enabled" => false,
														),
								
								"uikit" => array(		"root" => "/uikit/UIKit.inc", 
														"bridge" => "/bridgesupport/appkit.xml",
														//"headers" => "/Users/ryanjoseph/Desktop/iphone/UIKit.framework/Headers",
														"headers" => "/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/iPhoneOS2.2.1.sdk/System/Library/Frameworks/UIKit.framework/Headers",
														"include_pattern" => "{[$]+include (UI.*).inc}",
														"header_pattern" => "^UI(.*)\.h",
														"enabled" => false,
														),
								
								"webkit" => array(		"root" => "/webkit/WebKit.inc", 
														"bridge" => "/bridgesupport/webkit.xml",
														"headers" => "/System/Library/Frameworks/WebKit.framework/Headers",
														"include_pattern" => "{[$]+include (.*).inc}",
														"header_pattern" => "^(.*)\.h",
														"enabled" => false,
														),
														
								"coredata" => array(	"root" => "/coredata/CoreData.inc", 
														"bridge" => "/bridgesupport/coredata.xml",
														"headers" => "/Developer/SDKs/MacOSX10.5.sdk/System/Library/Frameworks/CoreData.framework/Headers",
														"include_pattern" => "{[$]+include (.*).inc}",
														"header_pattern" => "^(.*)\.h",
														"enabled" => false,
														),
								);
	
	var $maximum_method_length = 111; 	// WE GET THIS ERROR: NSDelegatesAll.pas(6109,296) Error: Identifier not found "NSDelegateController_NSLayoutManagerDelegate_layoutManager_shouldUseTemporaryAttributes_forDrawingToScreen_atCharacterIndex_effectiveRange"
	
	var $output;									// current output file handle                        
	var $root;										// root for printing/locating resources              
	var $out;										// directory to print                                
	var $show;										// print output to screen instead of file            
	var $framework;									// current framework being parsed                    
	var $current_class;								// reference to current class structure being parsed 
	var $current_header;							// reference to current header structure being parsed
	var $method_count;								// count of all methods parsed                       
	var $class_count;								// count of all classes parsed                       
	var $warning_count;								// count of parser warnings                          
	var $parser_skipping;							// the parser is skipping lines in the current file  
	var $inside_macro_block;						// the parser is inside a macro block, no nesting!
	var $instance_var_scope;						// current scope of the instance variable parser
	var $cocoa_classes = array(); 					// array of all NS*** classes                        
	var $cocoa_categories = array(); 				// array of all NS*** categories	                 
	var $dump = array();							// Convert Pascal classes
	var $delegate_methods = array();				// Delegate methods and types from GEN_BRIDGE_METADATA XML data
	var $delegate_method_names = array();			// Delegate method name array
	var $type_encodings = array();					// Master listing of type encodings for each method in the frameworks
	
	/**
	 * PARSER OPTIONS
	 */
	var $objc_id = "NSObjectRef";							// Default type for generic objects
	var $objc_id_real = "NSObjectRef";						// The real type of generic objects (id)
	var $objc_id_base = "Pointer";							// The base type for all "Ref" types
	var $sel_string = "SELString";							// The selector string type which registers selectors internally
	var $protocol_suffix = "Protocol";						// All protocols append this suffix 
	var $pointer_type_suffx = "Ref";						// NS*** pointers in parameter lists are suffixed with this
	var $class_pointer_suffix = "Pointer";					// Pointers to NS*** classes are suffxed with this
	var $register_selectors = true;							// Register selectors automatically inside the wrappers
	var $show_added_messages = false;						// show messages when methods are added to a class
	var $show_class_hierarchy = false;
	var $objects_are_wrappers = false;						// Treat all objects (id) like wrappers. i.e aObject.Handle;
	var $replace_hinted_params = false;						// replace comment hints with hinted type - (void * /* CMProfileRef */)colorSyncProfile;
	var $master_delegate_class = "NSDelegateController";	// Name of the master delegate class
	var $master_delegate_file = "NSDelegatesAll";			// Name of the master delegate file (no extension)
	var $trailing_underscore = false;						// Append the trailing underscore for last parameter
	var $record_keyword = "record";							// The keyword used for printing records
	var $bitpacked_record_keyword = "bitpacked record";		// The keyword used for printing bitpacked records
	var $string_macro = "NSString";
	
	// Pascal keywords to protect
	var $reserved_keywords = array(	"const", "object", "string", "array", "var", "set", "interface", "classname", "unit",
									"self", "type", "raise", "property", "to", "for", "with", "function", "procedure", "result",
									"pointer", "create", "new", "dispose", "label", "packed", "record", "char", "class",
									);
									
	// FPC methods that can't be overloaded
	var $reserved_methods = array("className");
	
	// Types which can not be altered by reserved keywords
	var $reserved_types = array("Pointer");
	
	// Objective-c types to convert
	var $replace_types = array(	"id"=>"NSObjectRef", "void"=>"Pointer", "BOOL"=>"LongBool", "long"=>"LongInt", "int"=>"Integer",
								"unsigned long"=>"UInt32", "unsigned short"=>"UInt8", "void *"=>"Pointer", "unsigned int"=>"UInt16",
								"NSUInteger"=>"UInt32", "NSInteger"=>"SInt32", "Class"=>"Pobjc_class", "uint"=>"UInt16", 
								"uint8_t"=>"UInt8", "signed int"=>"Integer", "const char"=>"PChar", "const void"=>"Pointer",
								"const uint8_t"=>"Pointer", "unsigned"=>"UInt8", "int32_t"=>"SInt32", "float"=>"Float32",
								"unsigned long long"=>"UInt64", "int64_t"=>"SInt64", "uint32_t"=>"UInt32", "uint16_t"=>"UInt16",
								"unsigned char"=>"char", "short"=>"SInt8", "double"=>"Float64", "long long"=>"SInt64",
								
								// macros
								"IBAction"=>"void",
															
								// special pointers
								"const id *"=>"NSObjectArrayOfObjectsPtr", "Protocol *"=>"ObjcProtocol", "NSObject *"=>"NSObjectRef",
								"const char *"=>"PChar", "const void *"=>"Pointer", "unsigned char *"=>"Pointer", "char *"=>"Pointer",
								"unsigned *"=>"Pointer",
								);						
	
	// These "types" are hints to the Objective-C garbage collector
	var $garbage_collector_hints = array("__strong", "__weak");

	var $null_macros = array("IBOutlet", "IBAction");
	
	// External NSString macros. These should be moved into the frameworks array
	var $external_string_macros = "APPKIT_EXTERN|FOUNDATION_EXPORT|UIKIT_EXTERN|COREDATA_EXTERN";

	// Types which have known pointers declared in the headers
	var $pointer_types = array(	// MacOSAll types
								"CGFloat"=>"psingle", "UInt32"=>"UInt32Ptr", "SInt32"=>"SInt32Ptr",
								
								// Cocoa types
								"BOOL"=>"pboolean",
								
								"unsigned char"=>"pchar", 
								"unsigned short"=>"pcushort", "unsigned int"=>"pcuint",	"uint"=>"pcuint", "signed int"=>"pcint",
								"float"=>"psingle", "double"=>"pdouble", "long long"=>"pclonglong","long"=>"pclong", "unsigned long"=>"pculong", 
								"int"=>"pinteger","uint8_t"=>"pbyte","unsigned"=>"pbyte","unsigned long long"=>"pculonglong"
								);

	var $objc_object_array = "id; objParams: array of const"; // Type which represents a dynamic array of Objective-c objects (id)
	
	// Symbols to ignore
	var $ignore_symbol = array("NSApp");
	
	// Categories to ignore
	var $ignore_categories = array("NSCoderMethods", "NSDeprecatedKeyValueCoding", "NSDeprecated");
	
	// Methods to ignore
	var $ignore_methods = array(	"retain", "release", "retainCount", "copyWithZone", "mutableCopyWithZone",
									"allocWithZone", "alloc", "copy", "mutableCopy", "self_", "autorelease", "awakeFromNib",
									"observationInfo",
								);	
	
	// default protected keywords by class/category
	// these may be useful if super classes were not parsed before
	var $default_protected = array(	"*"=>array("description", "classDescription"),
									"NSDeprecated"=>array("accessoryView"),
									"NSToolbarSupport"=>array("toolbar"),
									"DOMNode"=>array("version"),
									"WebView"=>array("frame"),
									"DOMImplementation"=>array("version"),
									);
	
	// Send methods that have a custom design
	var $custom_send_methods = array(	"NSArray.arrayWithObjects", "NSArray.initWithObjects", 
										"NSDictionary.dictionaryWithObjectsAndKeys", "NSDictionary.initWithObjectsAndKeys", 
										"NSSet.setWithObjects", "NSSet.initWithObjects", 
										);
	
	var $toll_free_bridge = array(	
									"NSString"=>"CFStringRef", "NSArray"=>"CFArrayRef", "NSMutableString"=>"CFMutableStringRef",
									"NSData"=>"CFDataRef", "NSDictionary"=>"CFDictionaryRef", "NSSet"=>"CFSetRef", 
									"NSMutableArray"=>"CFMutableArrayRef", "NSMutableCharacterSetRef"=>"CFMutableCharacterSetRef",
									"NSMutableData"=>"CFMutableDataRef", "NSMutableDictionary"=>"CFMutableDictionaryRef",
									"NSMutableSet"=>"CFMutableSetRef",  "NSNumber"=>"CFNumberRef", "NSURL"=>"CFURLRef", 
									"NSError"=>"CFErrorRef", 
									
									/* The Cocoa versions of these API's are much better and not very common, should we replace them?
									"NSOutputStream"=>"CFWriteStreamRef", "NSPasteboard"=>"PasteboardRef"
									"NSInputStream"=>"CFReadStreamRef", "NSTimer"=>"CFRunLoopTimerRef",
									"NSTimeZone"=>"CFTimeZoneRef", "NSCharacterSet"=>"CFCharacterSetRef",
									"NSDate"=>"CFDateRef",
									*/
									);	
	
	// types that use objc_msgSend_stret on all architectures
	var $struct_types = array(	"NSRect", "NSDecimal", "NSFastEnumerationState", "NSMapTableValueCallBacks",
								"NSHashTableCallBacks", "NSMapTableKeyCallBacks", 
								
								// TEMPORARY PARSER HACKS
								"aeDesc_",
								);
						
	// structs that use objc_msgSend or objc_msgSend_stret depending on architecture
	// "On Mac OS X Intel/i386, records whose size is 1, 2, 4 or 8 bytes are returned using registers"
	var $struct_register_types = array(	"NSRange", "NSPoint", "NSSize", "NSAffineTransformStruct" );
														
	// Functions that return types will invoke objc_msgSend_fpret
	// NOTE: we must also know the exact type so we can make the proper function pointer
	var $float_types = array(	// C-types
								"float", "long double", "double", 
								
								// Cocoa types
								"NSTimeInterval",
								
								// Pascal types
								"Float64", "Float32",
								
								// CarbonTypes
								"CGFloat",
								);
	
	var $skip_blocks = array(	"^#if __LP64__ \|\| NS_BUILD_32_LIKE_64"=>"^#(else|endif)+",
								);
								
	var $macro_blocks = array(	"^#if (__LP64__)"=>"\$ifdef cpu64",
								"^#if ![[:space:]]*(__LP64__)"=>"\$ifndef cpu64",
								"^#ifdef __BIG_ENDIAN__"=>"\$ifdef fpc_big_endian",
								//"^#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_[0-9]+_[0-9]+"=>"*",
								);
								
	// these categories should be placed into NSObject
	var $base_categories = array(		"NSArchiverCallback", "NSClassDescriptionPrimitives", "NSCoderMethods", "NSComparisonMethods", 
										"NSDelayedPerforming", "NSDeprecatedKeyValueCoding", "NSDeprecatedKeyValueObservingCustomization",
										"NSDistributedObjects", "NSErrorRecoveryAttempting", "NSKeyValueCoding", "NSPlaceholders",
										"NSKeyValueObserverRegistration", "NSKeyValueObserving", "NSKeyValueObservingCustomization", 
										"NSKeyedArchiverObjectSubstitution", "NSKeyedUnarchiverObjectSubstitution", "NSDeprecatedMethods", 
										"NSScriptKeyValueCoding", "NSThreadPerformAdditions", "NSServicesRequests", "NSKeyValueBindingCreation",
										"NSAccessibility", "NSAccessibilityAdditions",
										);
										
										// These really don't feel like they should be in NSObject, removing until we know where to put them.
										// Maybe a new class? NSCategories...
										
										// "NSURLClient", "NSScripting", "NSScriptClassDescription", "NSScriptObjectSpecifiers", 
										// "NSScriptingComparisonMethods", "NSFontManagerResponderMethod", "NSMenuValidation",
										// "NSColorPanelResponderMethod", "NSFontPanelValidationAdditions", "NSToolbarItemValidation",
										// "NSDictionaryControllerKeyValuePair", "NSEditor", 
	/**
	 * COMMON REGULAR EXPRESIONS
	 */
	var $regex_objc_method_params = "^(-|\+)[[:space:]]*\((.*)\)[[:space:]]*([a-zA-Z0-9]+):(.*);";
	var $regex_objc_method_no_params = "^(-|\+)[[:space:]]*\((.*)\)[[:space:]]*([a-zA-Z0-9]+);";
	var $regex_objc_category = "^@interface ([a-zA-Z]+)[[:space:]]*\(([a-zA-Z]+)\)";
	var $regex_objc_class = "^@interface ([a-zA-Z]+)[[:space:]]*:[[:space:]]*([a-zA-Z]+)[[:space:]]*(<(.*)>)*";
	var $regex_objc_class_no_super = "^@interface ([a-zA-Z]+)[[:space:]]*<(.*)>[[:space:]]*";
	var $regex_objc_protocol = "^@protocol ([a-zA-Z]+)";
	var $regex_procedure_type = "^[[:space:]]*(void|IBAction)+[[:space:]]*$";
	var $regex_scope_compiler_directive = "^\s*@(private|protected|public)(;*)+\s*$";
	var $regex_objc_property = "^@property\((.*)\)[[:space:]]*(.*);";
	
	/**
	 * TEMPLATES
	 */
	
	// Template for implemented function
	var $template_implemented_function = "function [CLASS].implemented_[NAME][PARAMS_HEADER]: [RETURN];
begin
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    Result :=  [RETURN](super_[NAME][PARAMS_BODY_WRAPPER]);
  {\$else}
    Result :=  [RETURN](super_[NAME][PARAMS_BODY]);
  {\$endif}
end;";

	// Template for implemented procedure
	var $template_implemented_procedure = "procedure [CLASS].implemented_[NAME][PARAMS_HEADER];
begin
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    super_[NAME][PARAMS_BODY_WRAPPER];
  {\$else}
    super_[NAME][PARAMS_BODY];
  {\$endif}
end;";
	
	// Template for constructor
	var $template_constructor = "constructor [CLASS][NAME][PARAMS_HEADER];
type
  TmsgSendWrapper = function (param1: id; param2: SEL[PARAMS_PROC]): id; cdecl;
var
  vmethod: TmsgSendWrapper;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));

  RegisterSubClass;
  allocbuf := objc_msgSend(ClassID, SEL_alloc, []);
  vmethod := TmsgSendWrapper(@objc_msgSend);
  Handle := vmethod(allocbuf, SEL_[SELNAME][PARAMS_LIST_WRAPPER]);
  retainCount := 1;
  AssignSelf(Handle);
  AddMethods;
  BindMethods;
end;";	

	// Template for constructor that does not allocate memory
	var $template_constructor_no_alloc = "constructor [CLASS][NAME][PARAMS_HEADER];
type
  TmsgSendWrapper = function (param1: id; param2: SEL[PARAMS_PROC]): id; cdecl;
var
  vmethod: TmsgSendWrapper;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));

  RegisterSubClass;
  vmethod := TmsgSendWrapper(@objc_msgSend);
  Handle := vmethod(ClassID, SEL_[SELNAME][PARAMS_LIST_WRAPPER]);
  AutoReleaseObject;
  AssignSelf(Handle);
  AddMethods; 
  BindMethods;
end;";	

	// Template for function to send Objective-c message that returns an auto-generated/memory managed wrapper object.
	var $template_function_make_wrapper = "function [CLASS][NAME][PARAMS_HEADER]: [RETURN];
type
  TmsgSendWrapper = function (param1: id; param2: SEL[PARAMS_PROC]): [RETURN]; cdecl;
var
  vmethod: TmsgSendWrapper;
  wrapper: [RETURN];
begin
  vmethod := TmsgSendWrapper(@[MSG_SEND]);
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));

  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]));
  {\$else}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]));
  {\$endif}

  {\$ifdef NSOBJECT_AUTO_WRAPPER}
   wrapper := [RETURN](GetWrapper(Result));
   if wrapper = nil then
     Result := [RETURN]([RETURN].CreateWithHandle(Pobjc_object(Result)).deferObject)
   else
     Result := wrapper;
  {\$endif}
end;";

	var $template_function_make_wrapper_no_params = "function [CLASS][NAME]: [RETURN];
var
  wrapper: [RETURN];
begin
  if SEL_[SELNAME] = nil then
	SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
	
  Result := [RETURN]([MSG_SEND]([OBJC_OBJECT], SEL_[SELNAME], []));
 
 {\$ifdef NSOBJECT_AUTO_WRAPPER}
  wrapper := [RETURN](GetWrapper(Result));
  if wrapper = nil then
    Result := [RETURN]([RETURN].CreateWithHandle(Pobjc_object(Result)).deferObject)
  else
    Result := wrapper;
 {\$endif}
end;";

	// Template for protocol function to send Objective-c message that returns an auto-generated/memory managed wrapper object.
	var $template_protocol_make_wrapper = "function [CLASS][NAME][PARAMS_HEADER]: [RETURN];
type
  TmsgSendWrapper = function (param1: id; param2: SEL[PARAMS_PROC]): [RETURN]; cdecl;
var
  vmethod: TmsgSendWrapper;
begin
  vmethod := TmsgSendWrapper(@[MSG_SEND]);
  if SEL_[SELNAME] = nil then
	SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
	
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]));
  {\$else}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]));
  {\$endif}

  {\$ifdef NSOBJECT_AUTO_WRAPPER}
   Result := [RETURN]([RETURN].CreateWithHandle(Pobjc_object(Result)).deferObject);
  {\$endif}
end;";

	var $template_protocol_make_wrapper_no_params = "function [CLASS][NAME]: [RETURN];
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  Result := [RETURN]([MSG_SEND]([OBJC_OBJECT], SEL_[SELNAME], []));

 {\$ifdef NSOBJECT_AUTO_WRAPPER}
   Result := [RETURN]([RETURN].CreateWithHandle(Pobjc_object(Result)).deferObject);
 {\$endif}
end;";

	// Template for function to send Objective-c message
	var $template_function_objc_send = "function [CLASS][NAME][PARAMS_HEADER]: [RETURN];
type
  TmsgSendWrapper = function (param1: [TARGET_TYPE]; param2: SEL[PARAMS_PROC]): [RETURN]; cdecl;
var
  vmethod: TmsgSendWrapper;
  super: objc_super;
begin
  vmethod := TmsgSendWrapper(@[MSG_SEND]);
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
	
  [GET_SUPER_CLASS]
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]));
  {\$else}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]));
  {\$endif}
end;";

	// Template for function to send Objective-c message (no params)
	var $template_function_objc_send_no_params = "function [CLASS][NAME]: [RETURN];
var
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  [GET_SUPER_CLASS]
  Result := [RETURN]([MSG_SEND]([OBJC_OBJECT], SEL_[SELNAME], []));
end;";
	
	// Template for function to send Objective-c message which returns a struct
	var $template_function_objc_send_struct = "function [CLASS][NAME][PARAMS_HEADER]: [RETURN];
type
  TmsgSendWrapper = function (param1: [TARGET_TYPE]; param2: SEL[PARAMS_PROC]): [RETURN]; cdecl;
var
  vmethod: TmsgSendWrapper;
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  vmethod := TmsgSendWrapper(@[MSG_SEND]);
  [GET_SUPER_CLASS]
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]));
  {\$else}
    Result := [RETURN](vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]));
  {\$endif}
end;";

	// Template for function to send Objective-c message which returns a struct
	var $template_function_objc_send_struct_cpu = "function [CLASS][NAME][PARAMS_HEADER]: [RETURN];
type
  TmsgSendWrapper_reg = function (param1: [TARGET_TYPE]; param2: SEL[PARAMS_PROC]): [RETURN]; cdecl;
  TmsgSendWrapper_stret = function (param1: [TARGET_TYPE]; param2: SEL[PARAMS_PROC]): [RETURN]; cdecl;
var
  vmethod_reg: TmsgSendWrapper_reg;
  vmethod_stret: TmsgSendWrapper_stret;
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  [GET_SUPER_CLASS]
  {\$ifdef CPUi386}
    vmethod_reg := TmsgSendWrapper_reg(@[MSG_SEND_REGISTER]);
	{\$ifdef NSOBJECT_AUTO_WRAPPER}
      Result := vmethod_reg([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]);
    {\$else}
      Result := vmethod_reg([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]);
    {\$endif}
  {\$else}
    vmethod_stret := TmsgSendWrapper_stret(@[MSG_SEND_STRET]);
    {\$ifdef NSOBJECT_AUTO_WRAPPER}
      Result := [RETURN](vmethod_stret([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]));
    {\$else}
      Result := [RETURN](vmethod_stret([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]));
    {\$endif}
  {\$endif}
end;";

	// Template for function to send Objective-c message (no params) which returns a struct
	var $template_function_objc_send_no_params_struct = "function [CLASS][NAME]: [RETURN];
var
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  [GET_SUPER_CLASS]
  [MSG_SEND](@Result, [OBJC_OBJECT], SEL_[SELNAME], []);
end;";

	// Template for function to send Objective-c message (no params) which returns CPU dependent struct
	var $template_function_objc_send_no_params_struct_cpu = "function [CLASS][NAME]: [RETURN];
type
  TmsgSendWrapper = function (param1: [TARGET_TYPE]; param2: SEL): [RETURN]; cdecl;
var
  vmethod: TmsgSendWrapper;
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  [GET_SUPER_CLASS]
  {\$ifdef CPUi386}
    vmethod := TmsgSendWrapper(@[MSG_SEND_REGISTER]);
    Result := vmethod([OBJC_OBJECT], SEL_[SELNAME]                       );
  {\$else}
    [MSG_SEND_STRET](@Result, [OBJC_OBJECT], SEL_[SELNAME], []);
  {\$endif}
end;";

	// Template for procedure to send Objective-c message
	var $template_procedure_objc_send = "procedure [CLASS][NAME][PARAMS_HEADER];
type
  TmsgSendWrapper = procedure (param1: [TARGET_TYPE]; param2: SEL[PARAMS_PROC]); cdecl;
var
  vmethod: TmsgSendWrapper;
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  vmethod := TmsgSendWrapper(@[MSG_SEND]);
  [GET_SUPER_CLASS]
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST_WRAPPER]);
  {\$else}
    vmethod([OBJC_OBJECT], SEL_[SELNAME][PARAMS_LIST]);
  {\$endif}
end;";

	// Template for procedure to send Objective-c message
	var $template_procedure_objc_send_no_params = "procedure [CLASS][NAME];
var
  super: objc_super;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  [GET_SUPER_CLASS]
  [MSG_SEND]([OBJC_OBJECT], SEL_[SELNAME], []);
end;";

	// Template for procedure to call implemented class method. This is the procedure which the Objectice-c methods are sent to
	var $template_procedure_objc_wrapper = "procedure [CLASS]_[NAME](_self: id; _cmd: SEL[PARAMS_HEADER]); cdecl;
var
  this: [CLASS];
  [VARIABLES]
begin
  this := [CLASS]([CLASS].GetSelf(_self));
  if this <> nil then
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    begin
      [WRAPPERS_CREATE]
      this.implemented_[NAME][PARAMS_LIST_WRAPPER];
      [WRAPPERS_RELEASE]
    end;
  {\$else}
    this.implemented_[NAME][PARAMS_LIST];
  {\$endif}
end;";

	// Template for procedure to call implemented class method. This is the procedure which the Objectice-c methods are sent to
	var $template_function_objc_wrapper = "function [CLASS]_[NAME](_self: id; _cmd: SEL[PARAMS_HEADER]): [RETURN]; cdecl;
var
  this: [CLASS];
  [VARIABLES]
begin
  this := [CLASS]([CLASS].GetSelf(_self));
  if this <> nil then
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    begin
      [WRAPPERS_CREATE]
      Result := [RETURN](this.implemented_[NAME][PARAMS_LIST_WRAPPER]);
      [WRAPPERS_RELEASE]
    end;
  {\$else}
    Result := [RETURN](this.implemented_[NAME][PARAMS_LIST]);
  {\$endif}
end;";


	// Template for method to override Objective-c method
	var $template_method_override = "procedure [CLASS].override_[NAME];
begin
  AddMethod('[OBJC_METHOD]', '[TYPE_ENCODING]', Pointer(@[CLASS]_[NAME]));
end;";

	// Template for method to override Objective-c method
	var $template_method_override_DEPRECATED = "procedure [CLASS].override_[NAME];
begin
  OverrideMethod('[OBJC_METHOD]', Pointer(@[CLASS]_[NAME]));
end;";

	// Template for method to add method to Objective-c runtime
	var $template_method_add_runtime = "procedure [CLASS].add_[NAME];
begin
  AddMethod('[OBJC_METHOD]', '[TYPES]', Pointer(@[CLASS]_[NAME]));
end;";

	// Template for implemented delegate procedure
	var $template_procedure_delegate = "procedure [CLASS].[NAME][PARAMS];
begin
end;";

	// Template for implemented delegate procedure
	var $template_function_delegate = "function [CLASS].[NAME][PARAMS]: [RETURN];
begin
end;";
	
	
	// Template for implemented delegate procedure
	var $template_procedure_delegate_objc = "procedure [CLASS]_[NAME][PARAMS_HEADER]; cdecl;
var 
  this: [CLASS];
  [VARIABLES]
begin
  this := [CLASS]([CLASS].GetSelf(_self));
  if this <> nil then
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    begin
      [WRAPPERS_CREATE]
      this.[NAME][PARAMS_LIST_WRAPPER];
      [WRAPPERS_RELEASE]
    end;
  {\$else}
    this.[NAME][PARAMS_LIST];
  {\$endif}
end;";

	// Template for implemented delegate procedure
	var $template_function_delegate_objc = "function [CLASS]_[NAME][PARAMS_HEADER]: [RETURN]; cdecl;
var
  this: [CLASS];
  [VARIABLES]
begin
  this := [CLASS]([CLASS].GetSelf(_self));
  if this <> nil then
  {\$ifdef NSOBJECT_AUTO_WRAPPER}
    begin
      [WRAPPERS_CREATE]
      Result := [RETURN](this.[NAME][PARAMS_LIST_WRAPPER]);
      [WRAPPERS_RELEASE]
    end;
  {\$else}
    Result := [RETURN](this.[NAME][PARAMS_LIST]);
  {\$endif}
end;";

	// Template for create override in delegate class
	var $template_delegate_create = "constructor [CLASS].Create;
begin
  CreateClassDefinition(ClassName, 'NSObject');

  ClassID := objc_getClass('[CLASS]');
  allocbuf := objc_msgSend(ClassId, SEL_alloc, []);
  Handle := objc_msgSend(allocbuf, SEL_init, []);
  retainCount := 1;

  { Adds custom methods, if any }
  AddMethods;
  BindMethods;

  { Assign our wrapper instance }
  if Handle <> nil then
    AssignSelf(Handle);
end;";

	// Template for constructor
	var $template_constructor_constarray = "constructor [CLASS][NAME][PARAMS_HEADER];
type
  TmsgSendWrapper = function (param1: id; param2: SEL; param3: [CFTYPE]): id; cdecl;
var
  vmethod: TmsgSendWrapper;
  paramList: [CFTYPE];
  i: integer;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  RegisterSubClass;
  [ALLOC_PARAM_LIST]
  allocbuf := objc_msgSend(ClassID, SEL_alloc, []);
  vmethod := TmsgSendWrapper(@objc_msgSend);
  Handle := vmethod(allocbuf, SEL_[SELNAME], paramList);
  retainCount := 1;
  AssignSelf(Handle);
  AddMethods;
  BindMethods;
  CFRelease(paramList);
end;";	

	// Template for constructor that does not allocate memory
	var $template_constructor_constarray_no_alloc = "constructor [CLASS][NAME][PARAMS_HEADER];
type
  TmsgSendWrapper = function (param1: id; param2: SEL; param3: [CFTYPE]): id; cdecl;
var
  vmethod: TmsgSendWrapper;
  paramList: [CFTYPE];
  i: integer;
begin
  if SEL_[SELNAME] = nil then
    SEL_[SELNAME] := sel_registerName(PChar('[OBJC_METHOD]'));
  RegisterSubClass;
  [ALLOC_PARAM_LIST]
  vmethod := TmsgSendWrapper(@objc_msgSend);
  Handle := vmethod(ClassID, SEL_[SELNAME], paramList);
  AutoReleaseObject;
  AssignSelf(Handle);
  AddMethods; 
  BindMethods;
  CFRelease(paramList);
end;";	

	// template to create param list for NSDictionary methods
	var $template_dictionary_param_list = "paramList := CFDictionaryCreateMutable(nil, 0, @kCFTypeDictionaryKeyCallBacks, @kCFTypeDictionaryValueCallBacks);
  i := High(firstObject);
  while i > 0 do
    begin
      CFDictionaryAddValue(paramList, firstObject[i].VPointer, firstObject[i - 1].VPointer);
      i := i - 2;
    end;";

	// template to create param list for NSArray methods
	var $template_array_param_list = "paramList := CFArrayCreateMutable(nil, 0, @kCFTypeArrayCallBacks);
  for i := 0 to High(firstObj) do
    CFArrayAppendValue(paramList, firstObj[i].VPointer);";
	
	// template to create param list for NSSet methods
	var $template_set_param_list = "paramList := CFSetCreateMutable(nil, 0, @kCFTypeSetCallBacks);
  for i := 0 to High(firstObj) do
    CFSetAddValue(paramList, firstObj[i].VPointer);";
	
	/**
	 * UTILITIES
	 */
	
	// Skips blocks in the current file being parsed
	function SkipBlock ($line) {
		
		if ($line != "") {
			foreach ($this->skip_blocks as $key => $value) {
				if (@ereg($key, $line)) $this->parser_skipping = true;
				if (@ereg($value, $line)) $this->parser_skipping = false;
			}
		}
		
		return $this->parser_skipping;
	}
	
	function IsKeywordReserved($keyword) {
		$keyword = strtolower($keyword);
		if (in_array($keyword, $this->reserved_keywords)) return true;
	}
	
	// Replace type with pointer equivalent
	function ReplacePointerType ($type) {
		
		$type = "Pointer {".$type."}";
		/*
		foreach ($this->pointer_types as $objc_type => $replace_type) {
			if ($objc_type == $type) {
				$type = $replace_type;
				break;
			}
		}
		*/
		return $type;
	}
	
	// Makes a struct field into an inline array (or returns field un-changed)
	function MakeFieldInlineArray ($io_field, $line, $name, $type) {

		if (eregi("\[([0-9]+)\];", $line, $array_size)) {
			$length = (int)$array_size[1] - 1;
			if ($length > 0) {
				$io_field = "    $name: array[0..$length] of $type;";
			}
		}
		
		return $io_field;
	}

	// Makes a type bitpacked (or returns type un-changed)
	function MakeFieldBitPacked ($ioType, $field, &$bitpacked) {
		$bitpacked = false;
		
		if (eregi(":([0-9]+);$", $field, $bitpack)) {
			$length = (int)$bitpack[1];
			if ($length > 1) {
				$ioType = "0..((1 shl $length)-1)";
			} else {
				$ioType = "0..$length";
			}
			
			$bitpacked = true;
		}
		
		return $ioType;
	}

	// Replace objc type with preferred type
	function ReplaceObjcType ($type) {
		
		foreach ($this->replace_types as $objc_type => $replace_type) {
			if ($objc_type == $type) {
				$type = $replace_type;
				break;
			}
		}
		
		return $type;
	}
	
	// Exchanges the preferred objc type with the real type
	function SwapObjcTypeWithReal ($type) {
		if ($type == $this->objc_id) $type = $this->objc_id_real;
		
		return $type;
	}
	
	// Replace garbage collector hints
	function ReplaceGarbageCollectorHints ($string, &$io_hint) {
		$io_hint = false;
		
		foreach ($this->garbage_collector_hints as $hint) {
			$out_string = str_ireplace($hint, "", $string);
			if ($out_string != $string) {
				$io_hint = $hint;
				$string = $out_string;
			}
		}
		
		return $string;
	}
	
	
	// Replace type of reference parameter with pointer
	function ReplaceReferenceParameterType ($type) {
		foreach ($this->pointer_types as $key => $value) {
			if ($key == $type) {
				$found = true;
				$type = $value;
				break;
			}
		}
		
		if (!$found) $type = $type."Pointer";
		
		return $type;
	}

	// Replace NS*** "toll free bridge" types with CoreFoundation type
	function ReplaceTollFreeBridgeType ($type) {
		foreach ($this->toll_free_bridge as $objc_type => $replace_type) {
			if ($objc_type == $type) {
				$type = istr_replace_word($type, $replace_type, $type);
				break;
			}
		}
		
		return $type;
	}
	
	// Replace all NS*** classes in a string with the preffered generic type $this->objc_id
	function ReplaceNSTypes ($string) {
		foreach ($this->cocoa_classes as $class) {
			$string = istr_replace_word($class, $this->objc_id, $string);
		}
		return $string;
	}
	
	// Replace all NS*** classes in a string with id
	function ReplaceNSTypesWithReal ($string) {
		foreach ($this->cocoa_classes as $class) {
			$string = istr_replace_word($class, $this->objc_id_real, $string);
		}
		return $string;
	}
	
	// Replace all NS*** classes in a string with their reference equivalent (i.e NSStringRef = id)
	function ReplaceNSTypesWithRef ($string) {
		foreach ($this->cocoa_classes as $class) {
			$string = istr_replace_word($class, $class."Ref", $string);
		}
		return $string;
	}
	
	// Copies the name from an Objective-C method definition
	function CopyObjcMethodName ($method) {
		
		// cut out comments first
		$method = eregi_replace("(/\*.*\*/)", "", $method);
		$method = eregi_replace("//.*$", "", $method);
		$method = trim($method, " 	");
		
		$params = explode(":", $method);
		$name = "";
		
		if (count($params) > 1) {
			foreach ($params as $value) {
				$value = trim($value, " 	");
				if (eregi("([a-zA-Z0-9_]+)$", $value, $captures)) $name .= $captures[1].":";
			}
		} else {
			if (eregi("([a-zA-Z0-9_]+)[[:space:]]*(;)*$", $method, $captures)) $name = $captures[1];
		}
		
		return $name;
	}
	
	// Converts a function pointer to Pascal function
	function ConvertFunctionPointerToPascal ($result, $param_string) {
		
		if ($result != "") {
			$params = explode(",", $param_string);
			$function = "function (";
			$count = 0;
			
			foreach ($params as $param) {
				$count ++;
				$param = trim($param, " ");
				$param = $this->ReplaceObjcType($param);
				$param = $this->SwapObjcTypeWithReal($param);
				$param = trim($param, "*");
				
				$function .= "param$count: $param; ";
			}
			
			$function = rtrim($function, "; ");
			$function .= "): $result; cdecl;";
		}
		
		//print("$function\n");
		return $function;
	}	
	
	// Converts a C parameter string to Pascal
	function ConvertCParamsPascal ($string) {
		
		$params = explode(",", $string);
		$param_string = "";
		
		foreach ($params as $param) {
			
			$param = istr_replace_word("const", "", $param);
			$param = trim($param, " ");
			
			$pair = explode(" ", $param);
			$name = $pair[1];
			$type = $pair[0];
				
			$type = $this->ReplaceObjcType($type);
			$type = $this->SwapObjcTypeWithReal($type);
			$type = $this->ReplaceTollFreeBridgeType($type);
			$type = $this->ReplaceNSTypesWithRef($type);
			
			if (($name[0] == "*") && ($name[1] == "*")) {
				$name = trim($name, "*");
				$type = $this->ReplacePointerType($type);
			} elseif ($name[0] == "*") {
				$name = trim($name, "*");
				
				$name = $name."Pointer";
				//$name = "var $name";
				
				$name = trim($name, " ");
			} else {
				$name = trim($name, "*");
			}
			
			// Remove array brackets (NSUInteger[])p
			if (eregi("\[[0-9]*\]", $name)) {
				$name = "$name";
				$type = "Pointer {array of $type}";
				$name = eregi_replace("\[[0-9]*\]", "", $name);
			}
  			
			if ($this->IsKeywordReserved($name)) $name .= "_";
			
			// multiple parameters
			if ($type == "...") {
				$param_string .= "varargs: array of const";
				break;
			}
			
			$param_string .= "$name: $type; ";
		}
		
		$param_string = trim($param_string, "; ");
		return $param_string;
	}
	
	// Remove OS X versions macros from a line
	// NOTE: These should be re-inlcuded in Pascal
	function RemoveOSVersionMacros ($line) {
		$line = eregi_replace("[[:space:]]*AVAILABLE_MAC_OS_X_VERSION_[0-9]+_[0-9]+_AND_LATER[[:space:]]*", "", $line);
		return $line;
	}
	
	// Removes all comments from a line
	function RemoveComments ($line) {
		// remove single-line comments
		$line = eregi_replace("[[:space:]]+//(.*)", "", $line);
		
		// remove multi-line comments /* ... */
		$line = eregi_replace("/\*.*\*/", "", $line);
		
		return $line;
	}
	
	// Performs additional formatting on Objective-c type i.e. (out NSObject **)
	function FormatObjcType ($type, &$modifiers) {
		$modifiers = "";
		
		// toss out all const identifiers
		$type = istr_replace_word("const", "", $type);
		
		// replace inout paramaters
		$type = istr_replace_word("inout", "", $type);
		$type = istr_replace_word("out", "", $type);
		$type_clean = trim($type, "* ");
		
		// Replace types before cleaning
		$type = $this->ReplaceObjcType($type);
		
		// Remove protocol which type conforms to (id <NSURLHandleClient>)
		$type = eregi_replace("<.*>", "", $type);
		
		// Remove array brackets (NSUInteger[])p
		$type = eregi_replace("\[[0-9]*\]", "", $type);
		
		// var params to non-object types (NSRange *)
		if (ereg("([a-zA-Z0-9_]+)[[:space:]]*[*]+$", $type, $captures)) { 
			if ((!in_array($captures[1], $this->cocoa_classes)) && ($captures[1] != "id")) {
				$type = $this->ReplaceReferenceParameterType($type_clean);	//"$type_clean$this->pointer_type_suffx";
				//$modifiers = "var ";
			}
		} 
		
		// Handle NS*** pointers (NSError **)
		if (ereg("(NS[a-zA-Z0-9_]+)[[:space:]]*\*\*$", $type, $captures)) { 
			if (in_array($captures[1], $this->cocoa_classes)) {
				$type = "$type_clean$this->class_pointer_suffix";
				//$modifiers = "var ";
			}
		}
		
		// clean the type
		$type = trim($type, "* ");
		
		//print("$type\n");
		return $type;
	}
	
	// Performs additional formatting on Objective-c parameter types		
	function FormatObjcParams ($string) {
		$params = explode(":", $string);
		$string = "";
		
		if (count($params) > 0) {
			foreach ($params as $value) {
				if (ereg("\((.*)\)", $value, $captures)) {
					$new_value = $this->ReplaceObjcType($captures[1]);
					
					if ($new_value != $captures[1]) $value = ereg_replace("\((.*)\)", "($new_value)", $value);
					
					$string .= ":$value";
				}
			}
		}
		
		$string = ltrim($string, ":");
		//print("$string\n");
		return $string;
	}
	
	// Converts an Objective-c parameter string to Pascal		
	function ConvertObjcParamsToPascal ($string, $protected_keywords, &$variable_arguments) {
		$params = explode(":", $string);
		$list = array();
		$list["pairs"] = array();
		$param_list = array();
		$variable_arguments = false;
		
		if (count($params) > 0) {
			//print_r($params);
			foreach ($params as $value) {
				$value = trim($value);
				$valid = false;
				$modifiers = "";
				
				// function pointer (callback)
				if (eregi("\(([a-zA-Z0-9_]+)[[:space:]]\((.*)\)\((.*)\)\)([a-zA-Z0-9_]+)", $value, $captures)) {
					$name = $captures[4];
					
					$type = $this->current_header["name_clean"].ucwords($name);
					
					// attempt to build a function pointer from the parameter and append the class type
					if ($this->current_header) {
						$function_pointer = $this->ConvertFunctionPointerToPascal($captures[1], $captures[3]);
						
						if (!@in_array($function_pointer, $this->current_header["types"]["callbacks"])) {
							$count = 0;
							while (@array_key_exists($type, $this->current_header["types"]["callbacks"])) {
								$count ++;
								$type = "$type$count";
							}
							
							// append the new type to the the current class
							$this->current_header["types"]["callbacks"][$type] = $function_pointer;  
						} else {
							// Use the name of the existing callback of matching type
							$type = array_search($function_pointer, $this->current_header["types"]["callbacks"]);
						}
					}
					
					$valid = true;
				} elseif (eregi("\(([a-zA-Z_]+).*\)([a-zA-Z_]+).*\.\.\.", $value, $captures)) { // variable arguments
					$name = $captures[2];
					$type = $captures[1];
					$variable_arguments = true;
					$valid = true;
				} elseif (eregi("\((.*)\)[[:space:]]*([a-zA-Z_]+)", $value, $captures)) { // standard parameter
					
					// pointers params to non-object types (NSRange *)
					if (ereg("[a-zA-Z0-9_]+Ptr$", $captures[2])) { 
						$captures[1] = trim($captures[1], "* ");
						$captures[1] = $this->ReplaceObjcType($captures[1]);

						$type = $captures[1].$this->class_pointer_suffix;//$this->ReplacePointerType($captures[1]);
						$name = $captures[2];
					} else {
						$type = $this->FormatObjcType($captures[1], $modifiers);
						$name = $captures[2];
					}
					
					
					$valid = true;
				}
				
				if ($valid) {
					
					// protect reserved keywords
					if ($this->IsKeywordReserved($name)) $name .= "_";
					
					if (!in_array($type, $this->reserved_types)) {
						if ($this->IsKeywordReserved($type)) $type .= "_";
					}

					if (@in_array($name, $protected_keywords)) $name .= "_";
					if (@in_array($type, $protected_keywords)) $type .= "_";
					
					// replace objc types
					$type = $this->ReplaceObjcType($type);
					$type = $this->ReplaceTollFreeBridgeType($type);
					
					// make sure we label duplicate params, which are allowed in Objective-C
					while (in_array($name, $param_list)) {
						$count ++;
						$name = "$name$count";
					}
					
					// id is always a wrapper
					if (($this->objects_are_wrappers) && ($type == $this->objc_id)) {
						$name_list = "$type(GetHandle($name))";
					} else {
						$name_list = $name;
					}
					
					// add modifiers to the name if there are any
					$name_with_modifiers = $modifiers.$name;

					// create pair array
					$pair["name"] = $name;
					$pair["type"] = $type;
					
					// append list
					$list["pairs"][] = $pair;
					$list["string_with_modifiers"] .= "$name_with_modifiers: $type; ";
					$list["string"] .= "$name: $type; ";
					$list["list"] .= "$name_list, ";
					$param_list[] = $name;
				}
			}
		}
		
		// clean up the strings
		$list["string"] = trim($list["string"], "; ");
		$list["string_with_modifiers"] = trim($list["string_with_modifiers"], "; ");
		$list["list"] = trim($list["list"], ", ");
		
		return $list;
	}

	// Converts an Objective-c method name to Pascal
	function ConvertObjcMethodName ($method) {
		$params = explode(":", $method);
		$name = "";
		
		if (count($params) > 1) {
			foreach ($params as $value) {
				if (eregi("([a-zA-Z0-9]+)$", $value, $captures)) $name .= $captures[1]."_";
			}
		} else {
			if (eregi("([a-zA-Z0-9]+)(;)*$", $params[0], $captures)) $name .= $captures[1]."_";
		}
		
		// clean it up
		if (!$this->trailing_underscore) $name = trim($name, "_");
		
		$name = $this->ReplaceObjcType($name);
		
		return $name;
	}	
	
	// Converts an Objective-C method to Pascal format 
	function ConvertObjcMethodToPascal ($class, $source, $parts, $protected_keywords, $has_params) {
		
		// replace "hinted" params comment with hinted type
		if ($this->replace_hinted_params) {
			
			// param string
			if (eregi("(/\*[[:space:]]*(.*)[[:space:]]*\*/)", $parts[4], $captures)) {
				// ??? change the parameter to the hinted type
				//$parts[4] = eregi_replace("(/\*.*\*/)", $captures[2], $parts[4]);
				//$parts[4] = trim($parts[4], " ");
			}

			// return type
			if (eregi("(/\*[[:space:]]*(.*)[[:space:]]*\*/)", $parts[2], $captures)) $parts[2] = $captures[2];

			//print_r($parts);

		} else { // remmove comments from params and return type
			$parts[4] = eregi_replace("(/\*.*\*/)", "", $parts[4]);
			$parts[4] = trim($parts[4], " ");

			$parts[2] = eregi_replace("(/\*.*\*/)", "", $parts[2]);
			$parts[2] = trim($parts[2], " ");
		}
		
		$return_type_clean = $parts[2];
		
		// perform preformatting before attempting to protect keywords
		$parts[2] = $this->FormatObjcType($parts[2], $modifiers);
		$parts[4] = $this->FormatObjcParams($parts[4]);
		
		// protect keywords in the parameter and return type
		if (count($protected_keywords) > 0) {
			foreach ($protected_keywords as $keyword) {
				$parts[4] = istr_replace_word($keyword, $keyword."_", $parts[4]);
				$parts[2] = istr_replace_word($keyword, $keyword."_", $parts[2]);
			}
		}
		
		if ($has_params) {
			$name = $this->ConvertObjcMethodName($source);
			
			// merge default protected keywords for the class/category
			if ($this->default_protected["*"]) $protected_keywords = array_merge($this->default_protected["*"], $protected_keywords);
			if ($this->default_protected[$class]) $protected_keywords = array_merge($this->default_protected[$class], $protected_keywords);
			
			$param_array = $this->ConvertObjcParamsToPascal($parts[4], $protected_keywords, $variable_arguments);
			$params = "(".$param_array["string"].")";
			$params_with_modifiers = "(".$param_array["string_with_modifiers"].")";
		} else {
			$params = "";
			$params_with_modifiers = "";
			$name = $parts[3];
			$param_array = null;
		}
		
		//print("$params_with_modifiers\n");
		
		// protect method name from keywords
		if ($this->IsKeywordReserved($name)) $name .= "_";
		
		// clean return type
		$return_type = trim($parts[2], "* ");
		$return_type = $this->ReplaceObjcType($return_type);
		$return_type = $this->ReplaceTollFreeBridgeType($return_type);
		
		$virtual = "";
		$class_prefix = "";
		
		// determine the type based on return value
		if (ereg($this->regex_procedure_type, $return_type_clean)) {
			$kind = "procedure";
		} else {
			$kind = "function";
			
			// make sure Objective-c objects that are returned from fuctions are not NSObject (and thus auto-wrapped)
			if ($return_type == $this->objc_id) $return_type = $this->objc_id_real;
			
			// method name starts with "init or alloc"
			if ((ereg("^(init|alloc)+[^ialization]", $name)) && ($parts[1] == "-")) {
				$struct["alloc"] = true;
				$kind = "constructor";
				$virtual = " virtual;";
			}

			// Class methods with the words: With, By or From in the name
			if ((ereg("^([a-zA-Z]+)(With|By|From)+", $name, $captures)) && ($parts[1] == "+")) $kind = "constructor";
			
			// Class methods which return "id" are constructors
			if (($parts[1] == "+") && ($return_type == $this->objc_id_real)) $kind = "constructor";
			
			// method result is the class name
			if ($return_type == $class) $kind = "constructor";
		}
		
		// determine if this is a class method
		if (($kind != "constructor") && ($parts[1] == "+")) $class_prefix = "class ";
		
		// Determine if the method needs a particular modifier
		// ??? THIS IS NOT COMPILING???
		//if (ereg($this->objc_object_array, $params)) $modifier = " cdecl;";
		
		// Replace SEL with the string equivalent
		if ($this->register_selectors) {
			$params_with_modifiers = str_replace_word("SEL", $this->sel_string, $params_with_modifiers);
		}
		
		// make method templates
		if ($kind != "function") {
			$method = "$class_prefix$kind $name$params_with_modifiers;$modifier$virtual";
			$method_template = "[KIND] [PREFIX]$name"."[PARAMS];$modifier";
		} else {
			$method = $class_prefix."function $name$params_with_modifiers: $return_type;$modifier$virtual";
			$method_template = "[KIND] [PREFIX]$name"."[PARAMS]: [RETURN];$modifier";
			$method_template_function = "function [PREFIX]$name"."[PARAMS]: [RETURN];$modifier";
		}
		
		$method_template_procedure = "procedure [PREFIX]$name"."[PARAMS];$modifier";
		$method_template_function = "function [PREFIX]$name"."[PARAMS]: [RETURN];$modifier";
		
		// ??? DEBUGGING
		//print("$method\n");
				
		// build structure
		$struct["def"] = $method;
		$struct["template"] = $method_template;
		$struct["template_function"] = $method_template_function;
		$struct["template_procedure"] = $method_template_procedure;
		$struct["objc_method"] = $this->CopyObjcMethodName($source);
		$struct["class_prefix"] = $class_prefix;
		//$struct["def_objc"] = eregi("(.*);", $source, $captures[1]);
		if ($return_type == "void") $return_type = "";
		$struct["return"] = $return_type;
		if (in_array($return_type, $this->cocoa_classes)) $struct["returns_wrapper"] = true;
		$struct["param_string_clean"] = trim($params, "()");
		$struct["param_string_clean_with_modifiers"] = trim($params_with_modifiers, "()");
		$struct["param_string"] = $params;
		$struct["param_string_with_modifiers"] = $params_with_modifiers;
		$struct["param_array"] = $param_array["pairs"];
		$struct["param_list"] = $param_array["list"];
		$struct["class"] = $class;
		$struct["name"] = $name;
		$struct["kind"] = $kind;
		
		if ($struct["param_array"] != null) $struct["has_params"] = true;
		
		// determine if the method can be overriden
		// (!eregi("^(set|get|is)+", $name))
		if ($kind != "constructor") $struct["can_override"] = true;
		
		/*
			TEMPORARY! we don't know how to handle super methods that have have floating point values
		*/	
		if (in_array($struct["return"], $this->float_types)) {
			$struct["can_override"] = false;
			print("	# WARNING: method $name can't override because the return type is float\n");
			$this->warning_count ++;
		}
		
		// FPC bug work around
		if (strlen($name) > $this->maximum_method_length) {
			$struct["can_override"] = false;
			print("	# WARNING: method $name can't override because the name is too long\n");
			$this->warning_count ++;
		}
			
		return $struct;
	}
	
	// Print string to output file
	function PrintOutput ($indent, $string) {
		for ($i=0; $i < $indent; $i++) { 
			$indent_string .= "  ";
		}
		
		if (($this->output) && (!$this->show)) fwrite($this->output, "$indent_string$string\n");
		
		if ($this->show) print("$indent_string$string\n");
	}
	
	// Returns the message sending template for a method structure
	function GetMsgSendTemplate ($method, $super) {
		if ($method["kind"] == "function") {
			if ($method["has_params"]) {
				$template = $this->template_function_objc_send;
			} else {
				$template = $this->template_function_objc_send_no_params;
			}
		} else {
			if ($method["has_params"]) {
				$template = $this->template_procedure_objc_send;
			} else {
				$template = $this->template_procedure_objc_send_no_params;
			}
		}
		
		// method returns a NS*** class wrapper. Now, super methods can't return wrappers
		if (!$super) {
			if (($method["kind"] == "function") && (in_array($method["return"], $this->cocoa_classes))) {
				if ($method["has_params"]) {
					$template = $this->template_function_make_wrapper;
				} else {
					$template = $this->template_function_make_wrapper_no_params;
				}
			}
		}
		
		// method returns a struct
		if (($method["kind"] == "function") && (in_array($method["return"], $this->struct_types))) {
			if ($method["has_params"]) {
				$template = $this->template_function_objc_send_struct;
			} else {
				$template = $this->template_function_objc_send_no_params_struct;
			}
		}
		
		// method returns an architecture dependent struct
		if (($method["kind"] == "function") && (in_array($method["return"], $this->struct_register_types))) {
			if ($method["has_params"]) {
				$template = $this->template_function_objc_send_struct_cpu;
			} else {
				$template = $this->template_function_objc_send_no_params_struct_cpu;
			}
		}
		
		// method is a constructor
		if ($method["kind"] == "constructor") {
			$template = $this->template_constructor_no_alloc;
			if ($method["alloc"]) $template = $this->template_constructor;
		}
		
		return $template;
	}
	
	// Returns a class hierarchy array 
	function GetClassHierarchy ($class, &$hierarchy) {
		if (!$hierarchy) $hierarchy = array();
		$hierarchy[] = $class["name"];
		
		if ($class["super_class"]) {
			$hierarchy[] = $this->GetClassHierarchy($class["super_class"], $hierarchy);
		} else {
			$hierarchy[] = "NSObject";
		}
		
		return $class["name"];
	}
	
	// returns if a keyword is protected in a class hierarchy
	function IsKeywordProtected ($keyword, $in_class) {
		$keywords = $this->GetClassHierarchy($in_class, $hierarchy);
		
		foreach ($hierarchy as $key) {
			if (@in_array($keyword, $keywords)) { //$this->dump["master"][$key]["protected_keywords"]
				return true;
			}
		}
	}
	
	// Returns all protected keywords in a class hierarchy
	function GetProtectedKeywords ($in_class) {
		$this->GetClassHierarchy($in_class, $hierarchy);
		$keywords = array();
		
		foreach ($hierarchy as $class) {
			if ($this->dump["master"][$class]["protected_keywords"]) {
				foreach ($this->dump["master"][$class]["protected_keywords"] as $keyword) $keywords[] = $keyword;
			}
		}
		
		return $keywords;
	}
	
	
	// Returns header a category should be moved to
	function FindCategoryHeader ($category) {
		
		foreach ($this->dump as $name => $header) {
			if ((@array_key_exists($category, $header["classes"])) && ($category != "NSObject")) {
				return $name;
			}
		}
	}
	
	// Adds a method structure to a class and performs checks for overloaded methods
	function AddMethodToClass (&$method, &$class) {
		
		// ignore methods
		if (in_array($method["name"], $this->ignore_methods)) return false;
		
		if (@!in_array($method["name"], $class["declared_methods"])) {
			
			$class["all"][$method["name"]] = $method;
			$class["protected_keywords"][] = $method["name"];
			$class["declared_methods"][] = $method["name"];
			$this->dump["all_methods"][$class["name"]][] = $method["objc_method"];
			
			if ($this->show_added_messages) print("	@ Added ".$method["name"]." to ".$class["name"]."\n");

			$this->method_count ++;
			return true;
		} else {
			print("	! ".$method["def"]." already exists in ".$class["name"]." defined as ".$class["all"][$method["name"]]["def"]."\n");
		}
	}
	
	// Adds a typedef to the header and handles organization to prevent order conflicts
	function AddTypeDef (&$header, $typedef) {
		$header["types"]["typedef"][] = $typedef;
	}
		
	// Returns a paramater list string with options to modify
	function MakeParamList ($param_array, $use_handle, $cast_handle, $direct, $register_selector) {
		$params = "";
		foreach ($param_array as $pair) {
			
			// register selector parameters
			if (($register_selector) && ($pair["type"] == "SEL")) {
				$params .= "sel_registerName(".$pair["name"]."), ";
				continue;
			}
			
			// use the object handle for NSObject descendants
			if ($use_handle) {
				if (in_array($pair["type"], $this->cocoa_classes)) {

					// cast the param to the original class type
					if ($cast_handle) {
						if ($direct == ACCESS_HANDLE_DIRECT) {
							$params .= $pair["type"]."(".$pair["name"].".Handle), ";
						} else {
							$params .= $pair["type"]."(GetHandle(".$pair["name"].")), ";
						}
					} else {
						if ($direct == ACCESS_HANDLE_DIRECT) {
							$params .= $pair["name"].".Handle, ";
						} else {
							$params .= "GetHandle(".$pair["name"]."), ";
						}
					}

				} else {
					if (($this->objects_are_wrappers) && ($pair["type"] == $this->objc_id)) {	// id is always a wrapper
						if ($direct == ACCESS_HANDLE_DIRECT) {
							$params .= $pair["type"]."(".$pair["name"].".Handle), ";
						} else {
							$params .= $pair["type"]."(GetHandle(".$pair["name"].")), ";
						}
					} else {
						$params .= $pair["name"].", ";
					}
				}
			} else { // append without modification
				$params .= $pair["name"].", ";
			}
		}
		
		return trim($params, ", ");
	}
	
	// Returns a list of paramameter variables with NS*** class types cast to "id" or the original class
	function MakeObjcTypeParamList ($param_array, $objc_type) {
		$params = "";
		foreach ($param_array as $pair) {
			if (in_array($pair["type"], $this->cocoa_classes)) {
				if ($objc_type) {
					$params .= "$this->objc_id(".$pair["name"]."), ";
				} else {
					$params .= $pair["type"]."(".$pair["name"]."), ";
				}
			} else {
				$params .= $pair["name"].", ";
			}
		}
		return trim($params, ", ");
	}
	
	/**
	 * PRINTING METHODS
	 */
			
	// Prints implemented methods
	function PrintImplementedMethods ($class) {
		
		// print implemented methods
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Implemented methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				if ($method["kind"] == "function") {
					$template = $this->template_implemented_function;
				} else {
					$template = $this->template_implemented_procedure;
				}
				
				$template = str_replace("[CLASS]", $class["name"], $template);
				$template = str_replace("[NAME]", $method["name"], $template);
				
				$method["return"] = $this->ReplaceNSTypesWithRef($method["return"]);
				$template = str_replace("[RETURN]", $method["return"], $template);
				$template = str_replace("[PARAMS_HEADER]", $method["param_string_with_modifiers"], $template);
				
				// build parameter list
				if ($method["has_params"]) {
					
					// auto-generate wrappers
					$params = "(";
					$params .= $this->MakeParamList($method["param_array"], USE_HANDLE, CAST_HANDLE, ACCESS_HANDLE_FUNCTION, DONT_REGISTER_SEL);
					$params .= ")";
					$template = str_replace("[PARAMS_BODY_WRAPPER]", $params, $template);
					
					// standard params
					$params = "(";
					$params .= $this->MakeObjcTypeParamList($method["param_array"], true);
					$params .= ")";
					$template = str_replace("[PARAMS_BODY]", $params, $template);
					
					
				} else {
					$template = str_replace("[PARAMS_BODY]", "", $template);
					$template = str_replace("[PARAMS_BODY_WRAPPER]", "", $template);
				}

				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}
		}
		
	}
				
	// Prints Objective-C wrapper procedures
	function PrintObjcWrapperProcedures ($class) {
		
		// print implemented methods
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Objective-c wrapper procedures }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				if ($method["kind"] == "function") {
					$template = $this->template_function_objc_wrapper;
				} else {
					$template = $this->template_procedure_objc_wrapper;
				}
				
				$template = str_replace("[CLASS]", $class["name"], $template);
				$template = str_replace("[NAME]", $method["name"], $template);
				$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
				
				$method["return"] = $this->ReplaceNSTypesWithRef($method["return"]);
				$template = str_replace("[RETURN]", $method["return"], $template);
				
				if ($method["has_params"]) {
					$method["param_string_clean"] = $this->ReplaceNSTypes($method["param_string_clean"]);
					
					// Make sure we always the id type in objc wrappers
					$params_header = $this->ReplaceNSTypesWithRef($method["param_string_clean_with_modifiers"]);
					$template = str_replace("[PARAMS_HEADER]", "; $params_header", $template);
					
					// auto-generate wrappers
					$wrappers_variables = "";
					$wrappers_create = "";
					$wrappers_release = "";
					$variable_list = "";
					
					foreach ($method["param_array"] as $pair) {
						if (in_array($pair["type"], $this->cocoa_classes)) {
							
							$wrappers_variables .= "object_".$pair["name"].": ".$pair["type"]."\n;";
							$wrappers_create .= "object_".$pair["name"]." := ".$pair["type"].".CreateWithHandle(".$pair["name"].");\n";
							$wrappers_release .= "object_".$pair["name"].".release;\n";
							$variable_list .= "object_".$pair["name"].", ";
						} else {
							$variable_list .= $pair["name"].", ";
						}
					}
					$variable_list = trim($variable_list, ", ");
					
					$template = str_replace("[VARIABLES]", $wrappers_variables, $template);
					$template = str_replace("[WRAPPERS_CREATE]", $wrappers_create, $template);
					$template = str_replace("[WRAPPERS_RELEASE]", $wrappers_release, $template);
					
					$template = str_replace("[PARAMS_LIST_WRAPPER]", "($variable_list)", $template);
					
					$params = $this->MakeObjcTypeParamList($method["param_array"], false);
					$template = str_replace("[PARAMS_LIST]", "($params)", $template);
					
				} else {
					$template = str_replace("[PARAMS_HEADER]", "", $template);
					$template = str_replace("[PARAMS_LIST]", "", $template);
					$template = str_replace("[PARAMS_LIST_WRAPPER]", "", $template);
					$template = str_replace("[VARIABLES]", "", $template);
					$template = str_replace("[WRAPPERS_CREATE]", "", $template);
					$template = str_replace("[WRAPPERS_RELEASE]", "", $template);
				}
				
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}
		}
	}
	
	// Prints send message objects with a custom implementation
	function PrintCustomSendMessageMethods ($class, $method) {
		
		// NSArray
		if ($class["name"] == "NSArray") {
			if ($method["name"] == "arrayWithObjects") $template = $this->template_constructor_constarray_no_alloc; 
			if ($method["name"] == "initWithObjects") $template = $this->template_constructor_constarray; 
			
			$template = str_replace("[CFTYPE]", "CFArrayRef", $template);
			$template = str_replace("[ALLOC_PARAM_LIST]", $this->template_array_param_list, $template);
			
			if ($method["name"] == "arrayWithObjects") $template = str_replace("[OBJC_METHOD]", "arrayWithArray:", $template);
			if ($method["name"] == "initWithObjects") $template = str_replace("[OBJC_METHOD]", "initWithArray:", $template);
		}
		
		// NSDictionary
		if ($class["name"] == "NSDictionary") {
			if ($method["name"] == "dictionaryWithObjectsAndKeys") $template = $this->template_constructor_constarray_no_alloc; 
			if ($method["name"] == "initWithObjectsAndKeys") $template = $this->template_constructor_constarray; 
			
			$template = str_replace("[CFTYPE]", "CFDictionaryRef", $template);
			$template = str_replace("[ALLOC_PARAM_LIST]", $this->template_dictionary_param_list, $template);
			
			if ($method["name"] == "dictionaryWithObjectsAndKeys") $template = str_replace("[OBJC_METHOD]", "dictionaryWithDictionary:", $template);
			if ($method["name"] == "initWithObjectsAndKeys") $template = str_replace("[OBJC_METHOD]", "initWithDictionary:", $template);
		}
		
		// NSSet
		if ($class["name"] == "NSSet") {
			if ($method["name"] == "setWithObjects") $template = $this->template_constructor_constarray_no_alloc; 
			if ($method["name"] == "initWithObjects") $template = $this->template_constructor_constarray; 
			
			$template = str_replace("[CFTYPE]", "CFSetRef", $template);
			$template = str_replace("[ALLOC_PARAM_LIST]", $this->template_set_param_list, $template);
			
			if ($method["name"] == "setWithObjects") $template = str_replace("[OBJC_METHOD]", "setWithSet:", $template);
			if ($method["name"] == "initWithObjects") $template = str_replace("[OBJC_METHOD]", "initWithSet:", $template);
		}
		
		$template = str_replace("[PARAMS_HEADER]", $method["param_string_with_modifiers"], $template);			
		$template = str_replace("[CLASS]", $class["name"].".", $template);
		$template = str_replace("[NAME]", $method["name"], $template);
		$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, $template);
	}
				
	// Prints send message objects
	function PrintSendMessageMethods ($class) {

		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Objective-c send message methods }");
		foreach ($class["all"] as $method) {
			
			// handle custom methods
			if (in_array($class["name"].".".$method["name"], $this->custom_send_methods)) {
				$this->PrintCustomSendMessageMethods($class, $method);
				continue;
			}
			
			$template = $this->GetMsgSendTemplate($method, false);
			
			$template = $method["class_prefix"].$template;
			
			$template = str_replace("[CLASS]", $class["name"].".", $template);
			$template = str_replace("[NAME]", $method["name"], $template);
			$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
			$template = str_replace("[RETURN]", $method["return"], $template);
			
			// Replace SEL with the string equivalent so it can be registered inside the wrapper
			if ($this->register_selectors) {
				$params_header = str_replace_word("SEL", $this->sel_string, $method["param_string_with_modifiers"]);
				$register = REGISTER_SEL;
			} else {
				$params_header = $method["param_string_with_modifiers"];
				$register = DONT_REGISTER_SEL;
			}
			
			$template = str_replace("[PARAMS_HEADER]", $params_header, $template);			
			
			if ($method["has_params"]) {
				$template = str_replace("[PARAMS_PROC]", "; ".$method["param_string_clean_with_modifiers"], $template);
				
				$params_wrapper = $this->MakeParamList($method["param_array"], USE_HANDLE, CAST_HANDLE, ACCESS_HANDLE_FUNCTION, $register);			
				$params_list = $this->MakeParamList($method["param_array"], DONT_USE_HANDLE, DONT_CAST_HANDLE, ACCESS_HANDLE_FUNCTION, $register);	
				
				$template = str_replace("[PARAMS_LIST_WRAPPER]", ", ".$params_wrapper, $template);
				$template = str_replace("[PARAMS_LIST]", ", ".$params_list, $template);
			} else {
				$template = str_replace("[PARAMS_PROC]", "", $template);
				$template = str_replace("[PARAMS_LIST]", "", $template);
				$template = str_replace("[PARAMS_LIST_WRAPPER]", "", $template);
			}
			
			$template = str_replace("[TARGET_TYPE]", $this->objc_id_real, $template);
			$template = str_replace("[OBJC_METHOD]", $method["objc_method"], $template);
			$template = str_replace("[GET_SUPER_CLASS]", "", $template);
			
			// decide reference to objc object by method type
			if ($method["class_prefix"] == "") {
				$template = str_replace("[OBJC_OBJECT]", "Handle", $template);
			} else {
				$template = str_replace("[OBJC_OBJECT]", "getClass", $template);
			}
			
			$template = str_replace("[MSG_SEND_STRET]", "objc_msgSend_stret", $template);
			$template = str_replace("[MSG_SEND_REGISTER]", "objc_msgSend", $template);
			
			if (in_array($method["return"], $this->struct_types)) {		// structure
				$template = str_replace("[MSG_SEND]", "objc_msgSend_stret", $template);
			} elseif (in_array($method["return"], $this->float_types)) {		// floating point
				$template = str_replace("[MSG_SEND]", "objc_msgSend_fpret", $template);
			} else { // simple type
				$template = str_replace("[MSG_SEND]", "objc_msgSend", $template);
			}
			
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, $template);
		}
	}
	
	// Prints override methods
	function PrintOverrideMethods ($class) {

		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Override methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				$template = $this->template_method_override;
				
				$template = str_replace("[CLASS]", $class["name"], $template);
				$template = str_replace("[NAME]", $method["name"], $template);
				$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
				$template = str_replace("[OBJC_METHOD]", $method["objc_method"], $template);
				$template = str_replace("[TYPE_ENCODING]", $this->type_encodings[$class["name"]][$method["objc_method"]], $template);
				
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}
		}
	}
	
	// Prints implemented methods that contain sending code
	function PrintImplementedSuperMethods ($class) {
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Implemented methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				$template = $this->GetMsgSendTemplate($method, true);
				
				$template = str_replace("[CLASS]", $class["name"].".", $template);
				$template = str_replace("[NAME]", "implemented_".$method["name"], $template);
				$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
				$method["return"] = $this->ReplaceNSTypesWithRef($method["return"]);
				$template = str_replace("[RETURN]", $method["return"], $template);

				// Replace SEL with the string equivalent so it can be registered inside the wrapper
				if ($this->register_selectors) {
					$params_header = str_replace_word("SEL", $this->sel_string, $method["param_string_with_modifiers"]);
					$register = REGISTER_SEL;
				} else {
					$params_header = $method["param_string_with_modifiers"];
					$register = DONT_REGISTER_SEL;
				}
				$template = str_replace("[PARAMS_HEADER]", $params_header, $template);			

				if ($method["has_params"]) {
					$template = str_replace("[PARAMS_PROC]", "; ".$method["param_string_clean_with_modifiers"], $template);

					$params_wrapper = $this->MakeParamList($method["param_array"], USE_HANDLE, CAST_HANDLE, ACCESS_HANDLE_FUNCTION, $register);			
					$params_list = $this->MakeParamList($method["param_array"], DONT_USE_HANDLE, DONT_CAST_HANDLE, ACCESS_HANDLE_FUNCTION, $register);	

					$template = str_replace("[PARAMS_LIST_WRAPPER]", ", ".$params_wrapper, $template);
					$template = str_replace("[PARAMS_LIST]", ", ".$params_list, $template);
				} else {
					$template = str_replace("[PARAMS_PROC]", "", $template);
					$template = str_replace("[PARAMS_LIST]", "", $template);
				}
				
				$template = str_replace("[TARGET_TYPE]", "Pobjc_super", $template);
				$template = str_replace("[OBJC_METHOD]", $method["objc_method"], $template);
				$template = str_replace("[OBJC_OBJECT]", "@super", $template);
				$template = str_replace("[GET_SUPER_CLASS]", "super := getSuperClass;", $template);
				
				$template = str_replace("[MSG_SEND_STRET]", "objc_msgSendSuper_stret", $template);
				$template = str_replace("[MSG_SEND_REGISTER]", "objc_msgSendSuper", $template);
				
				if (in_array($method["return"], $this->struct_types)) {
					$template = str_replace("[MSG_SEND]", "objc_msgSendSuper_stret", $template);
				} else {
					$template = str_replace("[MSG_SEND]", "objc_msgSendSuper", $template);
				}
				
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}
		}
	}
		
	// Prints super methods
	function PrintSuperMethods ($class) {
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Super methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				$template = $this->GetMsgSendTemplate($method, true);
				
				$template = str_replace("[CLASS]", $class["name"].".", $template);
				$template = str_replace("[NAME]", "super_".$method["name"], $template);
				$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
				
				$method["return"] = $this->ReplaceNSTypesWithRef($method["return"]);
				$template = str_replace("[RETURN]", $method["return"], $template);
				
				//$method["param_string"] = $this->ReplaceNSTypesWithReal($method["param_string"]);
				//$method["param_string_clean"] = $this->ReplaceNSTypesWithReal($method["param_string_clean"]);
				
				$template = str_replace("[PARAMS_HEADER]", $method["param_string_with_modifiers"], $template);
				$template = str_replace("[PARAMS_PROC]", "; ".$method["param_string_clean"], $template);
				$template = str_replace("[PARAMS_LIST_WRAPPER]", ", ".$method["param_list"], $template);
				$template = str_replace("[PARAMS_LIST]", ", ".$method["param_list"], $template);
				$template = str_replace("[TARGET_TYPE]", "Pobjc_super", $template);
				$template = str_replace("[OBJC_METHOD]", $method["objc_method"], $template);
				$template = str_replace("[OBJC_OBJECT]", "@super", $template);
				$template = str_replace("[GET_SUPER_CLASS]", "", $template);
				
				if (in_array($method["return"], $this->struct_types)) {
					$template = str_replace("[MSG_SEND]", "objc_msgSendSuper_stret", $template);
				} else {
					$template = str_replace("[MSG_SEND]", "objc_msgSendSuper", $template);
				}
				
				
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}
		}
	}
	
	function PrintProtocolDeclaration ($protocol, $method) {
		$template = $method["template"];
		$template = str_replace("[PREFIX]", $protocol["name"]."_", $template);
		
		if ($method["param_array"] == 0) {
			$param = "sourceObject: NSObjectRef";
		} else {
			$param = "sourceObject: NSObjectRef; ";
		}
		
		$template = str_replace("[PARAMS]", "($param".$method["param_string_clean_with_modifiers"].")", $template);
		$template = str_replace("[KIND]", $method["kind"], $template);
		$template = str_replace("[RETURN]", $method["return"], $template);

		$this->PrintOutput(0, $template);
	}
	
	// Prints all the protocols in the header
	function PrintHeaderProtocols ($header, $implemented) {
		
		if (!$header["protocols"]) return;
		
		foreach ($header["protocols"] as $protocol) {
			
			if ($implemented) {
				
				if (!$protocol["methods"]) continue;
				
				foreach ($protocol["methods"] as $name => $method) {
					if ($method["kind"] != "constructor") {
						//$this->PrintProtocolDeclaration($protocol, $method);
						
						$template = $this->GetMsgSendTemplate($method, false);
						
						// choose the protocol version
						if ($template == $this->template_function_make_wrapper) $template = $this->template_protocol_make_wrapper;
						if ($template == $this->template_function_make_wrapper_no_params) $template = $this->template_protocol_make_wrapper_no_params;
						
						$template = str_replace("[CLASS]", $protocol["name"]."_", $template);
						if ($method["param_array"] == 0) {
							// add header params token to accommodate our extra parameter
							$template = str_replace("[NAME]", $method["name"]."[PARAMS_HEADER]", $template);
						} else {
							$template = str_replace("[NAME]", $method["name"], $template);
						}
						$template = str_replace("[SELNAME]", str_replace(":", "_", $method["objc_method"]), $template);
						$template = str_replace("[RETURN]", $method["return"], $template);

						if ($method["param_array"] == 0) {
							$source_param = "sourceObject: NSObjectRef";
						} else {
							$source_param = "sourceObject: NSObjectRef; ";
						}

						// Replace SEL with the string equivalent so it can be registered inside the wrapper
						if ($this->register_selectors) {
							$params_header = str_replace_word("SEL", $this->sel_string, $method["param_string_clean_with_modifiers"]);
							$register = REGISTER_SEL;
						} else {
							$params_header = $method["param_string_clean_with_modifiers"];
							$register = DONT_REGISTER_SEL;
						}

						$template = str_replace("[PARAMS_HEADER]", "($source_param$params_header)", $template);			

						if ($method["has_params"]) {
							$template = str_replace("[PARAMS_PROC]", "; ".$method["param_string_clean"], $template);

							$params_wrapper = $this->MakeParamList($method["param_array"], USE_HANDLE, CAST_HANDLE, ACCESS_HANDLE_DIRECT, $register);			
							$params_list = $this->MakeParamList($method["param_array"], DONT_USE_HANDLE, DONT_CAST_HANDLE, ACCESS_HANDLE_DIRECT, $register);	

							$template = str_replace("[PARAMS_LIST_WRAPPER]", ", ".$params_wrapper, $template);
							$template = str_replace("[PARAMS_LIST]", ", ".$params_list, $template);
						} else {
							$template = str_replace("[PARAMS_PROC]", "", $template);
							$template = str_replace("[PARAMS_LIST]", "", $template);
						}

						$template = str_replace("[TARGET_TYPE]", $this->objc_id_real, $template);
						$template = str_replace("[OBJC_METHOD]", $method["objc_method"], $template);
						$template = str_replace("[GET_SUPER_CLASS]", "", $template);
						
						$template = str_replace("[MSG_SEND_STRET]", "objc_msgSend_stret", $template);
						$template = str_replace("[MSG_SEND_REGISTER]", "objc_msgSend", $template);

						// use the source object as the the target
						$template = str_replace("[OBJC_OBJECT]", "sourceObject", $template);

						if (in_array($method["return"], $this->struct_types)) {		// structure
							$template = str_replace("[MSG_SEND]", "objc_msgSend_stret", $template);
						} elseif (in_array($method["return"], $this->float_types)) {		// floating point
							$template = str_replace("[MSG_SEND]", "objc_msgSend_fpret", $template);
						} else { // simple type
							$template = str_replace("[MSG_SEND]", "objc_msgSend", $template);
						}

						$this->PrintOutput(0, "");
						$this->PrintOutput(0, $template);
						
						
					}
				}
				
			} else {
				if ($protocol["methods"]) {
					$this->PrintOutput(0, "");
					$this->PrintOutput(0, "{ Protocol: ".$protocol["name"]." }");

					foreach ($protocol["methods"] as $name => $method) {
						if ($method["kind"] != "constructor") {
							$this->PrintProtocolDeclaration($protocol, $method);
						}
					}
				}
			}
		}
	}
	
	function PrintSelectorVariables ($class) {
		
		// class has no methods, bail!
		if (!$class["methods"]) return;
		
		$this->PrintOutput(0,"var");
		
		foreach ($class["all"] as $method) {
			$sel_name = str_replace(":", "_", $method["objc_method"]);
			$this->PrintOutput(1, "SEL_$sel_name: SEL;");
		}
		
	}
	
	// Prints a classes implementation in Pascal format to a file handle
	function PrintClassImplementation ($class) {
		
		// class has no methods, bail!
		if (!$class["methods"]) return;
		
		$name = $class["name"];
		
		$this->PrintOutput(0,"");
		$this->PrintOutput(0, "{ Selectors for $name }");
		$this->PrintSelectorVariables($class);
		
		$this->PrintOutput(0,"");
		$this->PrintOutput(0, "{ Implementation for $name }");
		
		// Global accessor object
		$this->PrintOutput(0,"");
		$this->PrintOutput(0,"var");
		$this->PrintOutput(1, "__".$class["name"].": ".$class["name"].";");
		
		// getClass method
		$this->PrintOutput(0,"");
		$this->PrintOutput(0,"class function $name.getClass: $this->objc_id_real;");
		$this->PrintOutput(0,"begin");
		$this->PrintOutput(1,"Result := objc_getClass('$name');");
		$this->PrintOutput(0,"end;");
		
		// withObject static accessor
		$this->PrintOutput(0,"");
		$this->PrintOutput(0, "class function $name.withObject (inObject: Pointer): $name;");
		$this->PrintOutput(0,"begin");
		$this->PrintOutput(1,"if __$name = nil then");
		$this->PrintOutput(2,"__$name := $name.Create;");
		$this->PrintOutput(1,"__$name.Handle := inObject;");
		$this->PrintOutput(1,"result := __$name;");
		$this->PrintOutput(0,"end;");
		
		$this->PrintImplementedSuperMethods($class);
		// DEPRECTAED IN FAVOR OF IMPLEMENTED SUPER METHODS
		//$this->PrintImplementedMethods($class);
		//$this->PrintSuperMethods($class);
		$this->PrintObjcWrapperProcedures($class);
		$this->PrintOverrideMethods($class);
		$this->PrintSendMessageMethods($class);
	}
	
	// Prints a calls in Pascal format to a file handle
	public function PrintClass ($class) {
		
		// class has no methods, bail!
		if (!$class["methods"]) return;
		
		// the delegate class is the super class of NSObject
		if ($class["name"] == "NSObject") $class["super"] = $this->master_delegate_class;
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ ".$class["name"]." }");
		$this->PrintOutput(1, $class["name"]." = class(".$class["super"].")");
		$this->PrintOutput(1, "public");
	   
		// getClass override
		$this->PrintOutput(2, "class function getClass: $this->objc_id_real; override;");
		
		// static wrapper accessor
		$class_name = $class["name"];
		$this->PrintOutput(2, "class function withObject (inObject: Pointer): $class_name;");
		
		// print class-level methods
		$this->PrintOutput(0, "");
		$this->PrintOutput(2, "{ Class Methods }");
		foreach ($class["methods"] as $method) {
			$this->PrintOutput(2, $method["def"]);
		}
	
		// print category-level methods
		if (count($class["categories"]) > 0) {
			foreach ($class["categories"] as $name => $category) {
				$this->PrintOutput(0, "");
				$this->PrintOutput(2, "{ Category: $name }");
				
				if ($category["methods"]) {
					foreach ($category["methods"] as $method) {
						$this->PrintOutput(2, $method["def"]);
					}
				}	
			}
		}
		
		// print implemented methods
		$this->PrintOutput(0, "");
		$this->PrintOutput(1, "protected");
		$this->PrintOutput(2, "{ Implemented Methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				$template = $method["template"];
				$template = str_replace("[PREFIX]", "implemented_", $template);
				$template = str_replace("[PARAMS]", $method["param_string_with_modifiers"], $template);
				$template = str_replace("[KIND]", $method["kind"], $template);
				
				// implemented methods always return id instead of wrappers
				$method["return"] = $this->ReplaceNSTypesWithRef($method["return"]);
				
				$template = str_replace("[RETURN]", $method["return"], $template);
				
				$this->PrintOutput(2, $template." virtual;");
			}
		}
		
		// print override methods
		$this->PrintOutput(0, "");
		$this->PrintOutput(2, "{ Override Methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				$template = $method["template_procedure"];
				$template = str_replace("[PREFIX]", "override_", $template);
				$template = str_replace("[PARAMS]", "", $template);
				$template = str_replace("[RETURN]", "", $template);
				
				$this->PrintOutput(2, $template);
			}
		}
		
		// print super methods
		/* DEPRECTAED IN FAVOR OF IMPLEMENTED SUPER METHODS
		$this->PrintOutput(0, "");
		$this->PrintOutput(2, "{ Super Methods }");
		foreach ($class["all"] as $method) {
			if ($method["can_override"]) {
			
				$template = $method["template"];
				$template = str_replace("[PREFIX]", "super_", $template);
				$template = str_replace("[PARAMS]", $method["param_string_with_modifiers"], $template);
				$template = str_replace("[KIND]", $method["kind"], $template);
				$template = str_replace("[RETURN]", $this->ReplaceNSTypesWithRef($method["return"]), $template);	
				
				$this->PrintOutput(2, $template);
			}
		}
		*/
		$this->PrintOutput(1, "end;");
	}
		
	function PrintDelegateReference ($valid_categories) {
		
		ksort($this->delegate_methods);
		
		$this->PrintOutput(0, "unit $this->master_delegate_file;");
		$this->PrintOutput(0, "interface");
		$this->PrintOutput(0, "uses");
		$this->PrintOutput(1, "ctypes, objc, MacOSAll");

		$this->PrintOutput(0, "type");
		$this->PrintOutput(1, "$this->master_delegate_class = class");
		$this->PrintOutput(1, "public");
		
		// implemented methods
		foreach ($this->delegate_methods as $category => $selectors) {
			if (in_array($category, $this->ignore_categories)) continue;
			
			// make sure the category is valid
			$valid = false;
			foreach ($valid_categories as $pattern) {
				if (eregi($pattern, $category)) {
					$valid = true;
					break;
				}
			}
			if (!$valid) continue;
			
			$this->PrintOutput(2, "");
			$this->PrintOutput(2, "{ $category }");
			
			foreach ($selectors as $selector) {
				
				// FPC long name bug work-around
				if (strlen($selector["name_pascal"]) > $this->maximum_method_length) continue;
				
				if ($selector["kind"] == "procedure") {
					$this->PrintOutput(2, $selector["kind"]." ".$selector["name_pascal"].$selector["param_string"].";");
				} else {
					$this->PrintOutput(2, $selector["kind"]." ".$selector["name_pascal"].$selector["param_string"].": ".$selector["method"]["return"].";");
				}
				
			}
		}
		
		$this->PrintOutput(1, "end;");
	}
		
		
	function PrintDelegateClass ($valid_categories) {
		
		ksort($this->delegate_methods);
		
		$this->PrintOutput(0, "{\$ifdef FORWARD}");
		$this->PrintOutput(1, "$this->master_delegate_class = class;");
		$this->PrintOutput(0, "{\$endif}");
	
		$this->PrintOutput(0, "{\$ifdef CLASSES}");
		$macro = strtoupper($this->master_delegate_class);
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_C}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_C}");
		
		$this->PrintOutput(1, "$this->master_delegate_class = class(NSObjectCore)");
		$this->PrintOutput(1, "public");
		
		//$this->PrintOutput(2, "constructor Create; override;");
		
		// implemented methods
		foreach ($this->delegate_methods as $category => $selectors) {
			if (in_array($category, $this->ignore_categories)) continue;
			
			// make sure the category is valid
			$valid = false;
			foreach ($valid_categories as $pattern) {
				if (eregi($pattern, $category)) {
					$valid = true;
					break;
				}
			}
			if (!$valid) continue;
			
			$this->PrintOutput(2, "");
			$this->PrintOutput(2, "{ $category }");
			
			foreach ($selectors as $selector) {
				
				// FPC long name bug work-around
				if (strlen($selector["name_pascal"]) > $this->maximum_method_length) continue;
				
				if ($selector["kind"] == "procedure") {
					$this->PrintOutput(2, $selector["kind"]." ".$selector["name_pascal"].$selector["param_string"]."; virtual;");
				} else {
					$this->PrintOutput(2, $selector["kind"]." ".$selector["name_pascal"].$selector["param_string"].": ".$selector["method"]["return"]."; virtual;");
				}
				
			}
		}
		
		// add methods
		$this->PrintOutput(2, "");
		$this->PrintOutput(2, "{ Adding methods }");
		foreach ($this->delegate_methods as $category => $selectors) {
			if (in_array($category, $this->ignore_categories)) continue;
			
			// make sure the category is valid
			$valid = false;
			foreach ($valid_categories as $pattern) {
				if (eregi($pattern, $category)) {
					$valid = true;
					break;
				}
			}
			if (!$valid) continue;
			
			foreach ($selectors as $selector) {
				// FPC long name bug work-around
				if (strlen("add_".$selector["name_pascal"]) > $this->maximum_method_length) continue;
				
				$this->PrintOutput(2, "procedure add_".$selector["name_pascal"].";");
			}
		}
		
		$this->PrintOutput(1, "end;");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$ifdef IMPLEMENTATION}");		
		
		// create constructor method
		/*
		$this->PrintOutput(0, "");
		$template = str_replace("[CLASS]", $this->master_delegate_class, $this->template_delegate_create);
		$this->PrintOutput(0, $template);
		*/
				
		// print implemented methods
		foreach ($this->delegate_methods as $category => $selectors) {
			if (in_array($category, $this->ignore_categories)) continue;
			
			// make sure the category is valid
			$valid = false;
			foreach ($valid_categories as $pattern) {
				if (eregi($pattern, $category)) {
					$valid = true;
					break;
				}
			}
			if (!$valid) continue;
			
			// place-holder methods
			foreach ($selectors as $selector) {

				// FPC long name bug work-around
				if (strlen($selector["name_pascal"]) > $this->maximum_method_length) continue;

				if ($selector["kind"] == "procedure") {
					$this->PrintOutput(0, $selector["kind"]." ".$this->master_delegate_class.".".$selector["name_pascal"].$selector["param_string"].";");
				} else {
					$this->PrintOutput(0, $selector["kind"]." ".$this->master_delegate_class.".".$selector["name_pascal"].$selector["param_string"].": ".$selector["method"]["return"].";");
				}

				$this->PrintOutput(0, "begin");
				$this->PrintOutput(0, "end;");
				$this->PrintOutput(0, "");
			}			
			
			// objc wrappers
			foreach ($selectors as $selector) {
				
				// FPC long name bug work-around
				if (strlen($selector["name_pascal"]) > $this->maximum_method_length) continue;

				if ($selector["kind"] == "function") {
					$template = $this->template_function_delegate_objc;
				} else {
					$template = $this->template_procedure_delegate_objc;
				}
				
				$template = str_replace("[CLASS]", $this->master_delegate_class, $template);
				$template = str_replace("[NAME]", $selector["name_pascal"], $template);
				
				$selector["method"]["return"] = $this->ReplaceNSTypes($selector["method"]["return"]);
				$template = str_replace("[RETURN]", $selector["method"]["return"], $template);
				
				if ($selector["method"]["has_params"]) {
					$selector["method"]["param_string_clean"] = $this->ReplaceNSTypesWithRef($selector["method"]["param_string_clean"]);
					$template = str_replace("[PARAMS_HEADER]", " (_self: $this->objc_id_real; _cmd: SEL; ".$selector["method"]["param_string_clean"].")", $template);
					
					// auto-generate wrappers
					$wrappers_variables = "";
					$wrappers_create = "";
					$wrappers_release = "";
					$variable_list = "";
					
					foreach ($selector["method"]["param_array"] as $pair) {
						if (in_array($pair["type"], $this->cocoa_classes)) {
							
							$wrappers_variables .= "object_".$pair["name"].": ".$pair["type"].";\n";
							$wrappers_create .= "object_".$pair["name"]." := ".$pair["type"].".CreateWithHandle(".$pair["name"].");\n";
							$wrappers_release .= "object_".$pair["name"].".release;\n";
							$variable_list .= "object_".$pair["name"].", ";
						} else {
							$variable_list .= $pair["name"].", ";
						}
					}
					$variable_list = trim($variable_list, ", ");
					
					$template = str_replace("[VARIABLES]", $wrappers_variables, $template);
					$template = str_replace("[WRAPPERS_CREATE]", $wrappers_create, $template);
					$template = str_replace("[WRAPPERS_RELEASE]", $wrappers_release, $template);
					
					$template = str_replace("[PARAMS_LIST_WRAPPER]", "($variable_list)", $template);
					
					$params = $this->MakeObjcTypeParamList($selector["method"]["param_array"], false);
					$template = str_replace("[PARAMS_LIST]", "($params)", $template);
					
				} else {
					$template = str_replace("[PARAMS_HEADER]", "(_self: $this->objc_id_real; _cmd: SEL)", $template);
					$template = str_replace("[PARAMS_LIST]", "", $template);
					$template = str_replace("[PARAMS_LIST_WRAPPER]", "", $template);
					$template = str_replace("[VARIABLES]", "", $template);
					$template = str_replace("[WRAPPERS_CREATE]", "", $template);
					$template = str_replace("[WRAPPERS_RELEASE]", "", $template);
				}
				
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}
			
			// add methods
			foreach ($selectors as $selector) {
				
				// FPC long name bug work-around
				if (strlen($selector["name_pascal"]) > $this->maximum_method_length) continue;
				
				$template = $this->template_method_add_runtime;
				
				$template = str_replace("[CLASS]", $this->master_delegate_class, $template);
				$template = str_replace("[NAME]", $selector["name_pascal"], $template);
				$template = str_replace("[TYPES]", $selector["types"], $template);
				$template = str_replace("[OBJC_METHOD]", $selector["name"], $template);
				
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, $template);
			}			
		}
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		
		print("* Printed delegate class to "."$this->root$this->out/foundation/$this->master_delegate_file.inc\n");
	}
	
	// Prints all externally defined symbols
	function PrintExternalSymbols ($header) {
		if (!$this->dump[$header["name"]]["types"]) return;
		foreach ($this->dump[$header["name"]]["types"] as $key => $type_array) {
			
			// External string constants
			if ($key == "string_constant") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ External string constants }");
				$this->PrintOutput(0, "var");
				
				foreach ($type_array as $type) $this->PrintOutput(1, $type);
			}
				
			if ($key == "external_symbol") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ External symbols }");
				$this->PrintOutput(0, "var");
				
				foreach ($type_array as $type) $this->PrintOutput(1, $type);
			}
		}
	}
	
	// Prints all types in the header
	function PrintTypes ($header) {
		if (!$this->dump[$header["name"]]["types"]) return;
		
		foreach ($this->dump[$header["name"]]["types"] as $key => $type_array) {	
			
			// External defines
			if ($key == "defines") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Defines }");
				$this->PrintOutput(0, "const");
				
				foreach ($type_array as $type) $this->PrintOutput(1, $type);
			}
				
			// External CFString constants
			/*
			if ($key == "string_constant") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ External string constants }");
				$this->PrintOutput(0, "var");
				
				foreach ($type_array as $type) $this->PrintOutput(1, $type);
			}
			*/	
							
			// Named Enumerations
			/*
			if ($key == "named_enums") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Sets }");
				foreach ($type_array as $type) {
					$this->PrintOutput(0, "");
					$this->PrintOutput(0, "type");
					$this->PrintOutput(1, $type);
				}
			}
			*/
			
			// Enumerations
			if ($key == "enums") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Constants }");
				foreach ($type_array as $block) {
					$this->PrintOutput(0, "");
					$this->PrintOutput(0, "const");
					foreach ($block as $type) $this->PrintOutput(1, $type);
				}
			}
			
			// Typedefs		
			if (($key == "typedef") || ($key == "named_enums")) {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Types }");
				$this->PrintOutput(0, "type");

				foreach ($type_array as $type) $this->PrintOutput(1, $type);
			}
			
			// CallBacks
			if ($key == "callbacks") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Callbacks }");
				$this->PrintOutput(0, "type");

				foreach ($type_array as $name => $type) $this->PrintOutput(1, "$name = $type");
			}
			
		}
	}
	
	// Prints all records in the header
	function PrintRecords ($header) {
		if (!$this->dump[$header["name"]]["types"]) return;
		
		foreach ($this->dump[$header["name"]]["types"] as $key => $type_array) {			
			// Structures
			if ($key == "structs") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Records }");

				foreach ($type_array as $type) {
					$this->PrintOutput(0, "type");
					$this->PrintOutput(1, $type);
				}
			}			
		}
	}
	
	// Prints all callbacks in the header
	function PrintCallBacks ($header) {
		if (!$this->dump[$header["name"]]["types"]) return;
		
		foreach ($this->dump[$header["name"]]["types"] as $key => $type_array) {	
			if ($key == "callbacks") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Callbacks }");
				$this->PrintOutput(0, "type");

				foreach ($type_array as $name => $type) $this->PrintOutput(1, "$name = $type");
			}
		}
	}
	
	// Prints all external functions in the header
	function PrintFunctions ($header) {
		if (!$this->dump[$header["name"]]["types"]) return;
		
		foreach ($this->dump[$header["name"]]["types"] as $key => $type_array) {	
			if ($key == "functions") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Functions }");
				
				foreach ($type_array as $type) $this->PrintOutput(0, $type);
			}
		}
	}
		
	// Prints all classes from the header in reference format (not for compiling)
	function PrintHeaderReference ($header, $path) {
		
		$this->output = fopen($path, "w+");
		
		//$this->PrintOutput(0, "{ ".ucfirst($header["framework"]).".framework ".$header["name"]." }");
		$this->PrintOutput(0, "unit ".$header["name_clean"].";");
		$this->PrintOutput(0, "interface");
		$this->PrintOutput(0, "uses");
		$this->PrintOutput(1, "ctypes, objc, MacOSAll;");
		
		if ($header["classes"]) {
			foreach ($header["classes"] as $class) {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "type");
				$this->PrintOutput(1, $class["name"]."Ref = ".$this->objc_id_real.";");
				$this->PrintOutput(1, $class["name"]."Pointer = Pointer;");
			}
		}
		
		// types
		$this->PrintTypes($header);
		$this->PrintRecords($header);
		$this->PrintFunctions($header);
		$this->PrintExternalSymbols($header);
		
		if ($header["classes"]) {
			
			foreach ($header["classes"] as $class) {
				if (in_array($class["name"], $this->cocoa_classes)) {
					$this->PrintOutput(0, "");
					$this->PrintOutput(0, "type");
					
					$this->PrintOutput(1, $class["name"]." = object(".$class["super"].")");

					// print class-level methods
					if (count($class["methods"]) > 0) {
						$this->PrintOutput(0, "");
						foreach ($class["methods"] as $method) {
							$this->PrintOutput(2, $method["def"]);
						}
					}

					// print category-level methods
					if (count($class["categories"]) > 0) {
						foreach ($class["categories"] as $name => $category) {
							$this->PrintOutput(0, "");
							$this->PrintOutput(2, "{ Category: $name }");

							if ($category["methods"]) {
								foreach ($category["methods"] as $method) {
									$this->PrintOutput(2, $method["def"]);
								}
							}	
						}
					}
					
					$this->PrintOutput(1, "end;");
				}
			}
		}
		
		// print procedural protocols
		if ($header["protocols"]) {
			foreach ($header["protocols"] as $protocol) {
				if ($protocol["methods"]) {
					$this->PrintOutput(0, "");
					$this->PrintOutput(0, "{ Protocol: ".$protocol["name"]." }");

					foreach ($protocol["methods"] as $name => $method) {
						if ($method["kind"] != "constructor") {
							$this->PrintProtocolDeclaration($protocol, $method);
						}
					}
				}
			}
		}
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "implementation");
		$this->PrintOutput(0, "end.");
	}
		
	// Prints all classes from the header
	public function PrintHeader ($header) {
		global $version;
		
		$this->output = fopen($header["path"], "w+");
		
		$this->PrintOutput(0, "{ Parsed from ".ucfirst($header["framework"]).".framework ".$header["name"]." }");
		
		$date = date("D M j G:i:s T Y");
		$this->PrintOutput(0, "{ Version $version - $date }");
		$this->PrintOutput(0, "");
		
		$macro = strtoupper(substr($header["name"], 0, (strripos($header["name"], "."))));
		
		if ($header["classes"]) {
			$this->PrintOutput(0, "{\$ifdef HEADER}");
			$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_H}");
			$this->PrintOutput(0, "{\$define $macro"."_PAS_H}");
		
		
				foreach ($header["classes"] as $class) {
					$this->PrintOutput(0, "type");

					// Make a id "reference" to each class which is an object but reveals the name of the class
					if ($class["name"]."Ref" == $this->objc_id_real) {
						$ref = $this->objc_id_base;	// replace duplicates with the "base id"
					} else {
						$ref = $this->objc_id_real;
					}
					
					$this->PrintOutput(1, $class["name"]."Ref = ".$ref.";");

					// Make a pointer to each class
					$this->PrintOutput(1, $class["name"]."Pointer = Pointer;");
				}
		
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef TYPES}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_T}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_T}");
		$this->PrintTypes($header);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef RECORDS}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_R}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_R}");
		$this->PrintRecords($header);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef FUNCTIONS}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_F}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_F}");
		$this->PrintFunctions($header);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef CALLBACKS}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_F}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_F}");
		$this->PrintCallBacks($header);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef EXTERNAL_SYMBOLS}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_T}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_T}");
		$this->PrintExternalSymbols($header);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		if ($header["classes"]) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$ifdef FORWARD}");
		
			foreach ($header["classes"] as $class) {
				// if the class contains methods make a forward declaration, otherwise a dummy class to NSObject
				if (count($class["all"]) > 0) {
					$this->PrintOutput(1, $class["name"]." = class;");
				} else {
					$this->PrintOutput(1, $class["name"]." = NSObject;");
				}
			}

			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
		}
	
		if ($header["classes"]) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$ifdef CLASSES}");
			$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_C}");
			$this->PrintOutput(0, "{\$define $macro"."_PAS_C}");
		
			foreach ($header["classes"] as $class) {
				if (in_array($class["name"], $this->cocoa_classes)) {
					$this->PrintClass($class);
					//print("	- Printed class ".$class["name"]."\n");
				}
			}
	
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef PROTOCOLS}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_P}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_P}");
		$this->PrintHeaderProtocols($header, false);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef IMPLEMENTATION}");
	
		$this->PrintHeaderProtocols($header, true);
		
		if ($header["classes"]) {
			foreach ($header["classes"] as $class) {
				if (in_array($class["name"], $this->cocoa_classes)) {
					$this->PrintClassImplementation($class);
				}
			}
		}
		
		$this->PrintOutput(0, "{\$endif}");
	}

	// Prints all headers parsed
	function PrintAllHeaders ($output_path, $ignore_output, $only_files, $print_header_references) {
			
			foreach ($this->dump as $file => $header) {
				if (eregi("^[a-zA-Z]+\.h", $file)) {
					
					// ignore these files
					if (@in_array($header["path_partial"], $ignore_output)) continue;
					
					// only parse these files
					if ((@count($only_files) > 0) && (@!in_array($header["name"], $only_files))) continue;
					
					$name_clean = substr($file, 0, (strripos($file, ".")));	
					
					// assign output path
					if ($output_path != "") $header["path"] = $output_path."/".$name_clean.".inc";
					
					$this->PrintHeader($header);
					
					if ($print_header_references) $this->PrintHeaderReference($header, $this->root.$this->out."/reference/".$name_clean.".pas");
					
					print("* Printed $name_clean.h to ".$header["path"]."\n");
				}
			}
	}	
	
	/**
	 * PARSING METHODS
	 */
	
	// Insert macro blocks to replace c-style blocks
	function InsertMacroBlocks ($line, &$in_macro_block) {
		
		// only insert if we are in a block already.
		// NOTE: this does not handle nesting!
		if ($in_macro_block) {
			
			// macro else statment
			if (eregi("#else", $line)) {
				return "{\$else}";
			}

			// macro endif statment
			if (eregi("#endif", $line)) {
				$in_macro_block = false;
				return "{\$endif}";
			}
		}
		
		foreach ($this->macro_blocks as $key => $value) {
			if (eregi($key, $line, $captures)) {
				$in_macro_block = true;
				
				// replace the c-macro with a Pascal version
				if ($value == "*") {
					$captures[0] = trim($captures[0], "#");
					return "{\$".$captures[0]."}";
				} else {
					return "{".$value."}";
				}
			}
		}
	}

	function ParseInstanceVariables ($line, &$struct) {
		$field = null;
		$field_bitpacked = false;
		//print("$line\n");
			
		// insert macros
		if ($macro = $this->InsertMacroBlocks($line, $this->inside_macro_block)) {
			if ($struct["valid"]) {
				$struct["fields"][] = $macro;
				return null;
			} else {
				return $macro;
			}
			
		}
		// got struct
		if (eregi("^[[:space:]]*struct.*{", $line)) {
			$struct["valid"] = true;
			return null;
		} 
		
		if (eregi("^[[:space:]]*}[[:space:]]*([a-zA-Z_0-9]+);", $line, $captures)) {
			$struct["name"] = "_".trim($captures[1], " 	");
			//print_r($struct);
			return "struct";
		}
		
		// set field prefix to protect scope
		if (!$struct["valid"]) $field_prefix = "_";
		
		// remove null-defined macros: 
		$line = str_ireplace($this->null_macros, "", $line);
		
		// replace garbage collector hints in the field	
		$line = $this->ReplaceGarbageCollectorHints($line, $garbage_collector_hint);
				
		if (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*	 ]+).*;", $line, $captures)) { // double-word single
			
			$name = trim($captures[3], "* 	");
			$name = str_replace(" ", "", $name);
			$name = str_replace("	", "", $name);

			if (eregi("^[[:space:]]*struct", $captures[1])) {
				$type = $captures[2];
			} else {
				$type = $captures[1]." ".$captures[2];
			}
			
			$type = $this->ReplaceObjcType($type);
			$type = $this->SwapObjcTypeWithReal($type);
			$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);
			if ($this->IsKeywordReserved($name)) $name .= "_";
			if ($captures[3][0] == "*") $this->ReplacePointerType($type);

			$field = "$field_prefix$name: $type;";
			$field = $this->MakeFieldInlineArray($field, $line, $name, $type);
			$field = eregi_replace("<.*>", "", $field);
			
		} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_* 	]+)(.*);", $line, $captures)) { // double-word type list
			$name = trim($captures[2], "* 	");
			$name = str_replace(" ", "", $name);
			$name = str_replace("	", "", $name);
			
			$type = $this->ReplaceObjcType($captures[1]);
			$type = $this->SwapObjcTypeWithReal($type);
			$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);
			if ($this->IsKeywordReserved($name)) $name .= "_";
			if ($captures[2][0] == "*") $this->ReplacePointerType($type);
			
			$field = "$field_prefix$name: $type;";
			$field = $this->MakeFieldInlineArray($field, $line, $name, $type);
			$field = eregi_replace("<.*>", "", $field);
			
		} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_*]+)(.*);", $line, $captures)) { // single word type list
			$name = trim($captures[2], "* 	");
			$name = str_replace(" ", "", $name);
			$name = str_replace("	", "", $name);

			$type = $this->ReplaceObjcType($captures[1]);
			$type = $this->SwapObjcTypeWithReal($type);
			$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);
			if ($this->IsKeywordReserved($name)) $name .= "_";
			if ($captures[2][0] == "*") $this->ReplacePointerType($type);
			$type = trim($type, "*");

			$field = "$field_prefix$name: $type;";
			$field = $this->MakeFieldInlineArray($field, $line, $name, $type);
			$field = eregi_replace("<.*>", "", $field);
		}
		
		// mark the field as having a garbage collector field
		if ($garbage_collector_hint) $field = "$field {garbage collector: $garbage_collector_hint }";
		
		// return field
		if ($struct["valid"]) {
			if ($field_bitpacked) $struct["bitpacked"] = true;
			$struct["fields"][] = $field;
		} else {
			return $field;
		}
	}
	
	// Parses a struct field into a list
	function ParseStructList ($input, $name, $type) {
		$field = "";
		
		$list = explode(",", $input);
		if (count($list) > 1) {
			$field = "    ";
			foreach ($list as $key) {
				$key = trim($key, " ");
				$field .= "$key, ";
			}
			
			$field = rtrim($field, ", ");
			$field .= ": $type;\n";
		} else {
			$field = "    $name: $type;\n";
		}
		
		return $field;
	}
	
	// Parse external symbols, enums and typedef's from the header
	function ParseHeaderTypes ($file) {
			$contents = ReadTextFile($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));	
			$field_bitpacked = false;
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
				
				// skip blocks
				if ($this->SkipBlock($line)) continue;
				
				// garbage collector hints
				$line = $this->ReplaceGarbageCollectorHints($line, $garbage_collector_hint);
					
				// remove macros	
				$line = $this->RemoveOSVersionMacros($line);
				
				// remove comments
				$line = $this->RemoveComments($line);
				$line = trim($line, " ");
									
				if ($got_struct) {
					
					// insert macros
					if ($macro = $this->InsertMacroBlocks($line, $this->inside_macro_block)) $struct_fields .= "$macro\n";
					
					// collect fields
					if (eregi("^[[:space:]]*([a-zA-Z0-9_*]+)[[:space:]]*[*]*\((.*)\)\((.*)\);", $line, $captures)) { // function pointer (callback)
						//continue;
						$name = trim($captures[2], "*");
						$result = $this->ReplaceNSTypesWithReal($captures[1]);
						$result = $this->ReplaceObjcType($result);
						$result = ": ".$this->SwapObjcTypeWithReal($result);
						if ($this->IsKeywordReserved($name)) $name .= "_";
						
						if ($captures[1] == "void") {
							$kind = "procedure";
							$result = "";
						} else {
							$kind = "function";
						}
						
						// ??? convert params to Pascal
						//$method = $this->ConvertFunctionPointerToPascal($result, $captures[3]);
						//$params = $method["param_string_clean"];
						$params = "context: Pointer {bad params!!}";
						
						$struct_fields .= "    $name: $kind ($params)$result; cdecl;\n";
						//print("$name: $kind ($params)$result; cdecl;\n");
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_* 	]+)(.*);", $line, $captures)) { // double-word single
						$name = trim($captures[3], "* ");
						$name = str_replace(" ", "", $name);
						$name = str_replace("	", "", $name);
						$type = $captures[1]." ".$captures[2];
						$type = $this->ReplaceObjcType($type);
						$type = $this->SwapObjcTypeWithReal($type);
						$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);
						if ($this->IsKeywordReserved($name)) $name .= "_";
						if ($captures[3][0] == "*") $this->ReplacePointerType($type);
						
						//$struct_fields .= "    $name: $type;\n";
						$struct_fields .= $this->ParseStructList($captures[3]+$captures[4], $name, $type);
						
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_* 	]+)(.*);", $line, $captures)) { // double-word type list
						$name = trim($captures[2], "* ");
						$name = str_replace(" ", "", $name);
						$name = str_replace("	", "", $name);
						$type = $this->ReplaceObjcType($captures[1]);
						$type = $this->SwapObjcTypeWithReal($type);
						$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);
						if ($this->IsKeywordReserved($name)) $name .= "_";
						if ($captures[2][0] == "*") $this->ReplacePointerType($type);
						
						$struct_fields .= $this->ParseStructList("$captures[2]$captures[3]", $name, $type);

					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_* 	]+)(.*);", $line, $captures)) { // single word type list
						$name = trim($captures[2], "* ");
						$captures[1] = str_replace(" ", "", $captures[1]);
						$captures[1] = str_replace("	", "", $captures[1]);
						$type = $this->ReplaceObjcType($captures[1]);
						$type = $this->SwapObjcTypeWithReal($type);
						$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);
						if ($this->IsKeywordReserved($name)) $name .= "_";
						if ($captures[2][0] == "*") $this->ReplacePointerType($type);
						
						//$struct_fields .= "    $name: $type;\n";
						$struct_fields .= $this->ParseStructList($captures[2], $name, $type);
					}

					
					// got end of struct
					if (ereg("^}[[:space:]]*([a-zA-Z_0-9]+);", $line, $captures)) {
						
						if ($struct_name == "") {
							$struct_name = $captures[1];
							$make_pointer = true;
						} else {
							$struct_type = $captures[1];
						}
						
						if ($field_bitpacked) {
							$struct = "$struct_name = $this->bitpacked_record_keyword\n";
						} else {
							$struct = "$struct_name = $this->record_keyword\n";
						}
						
						$struct .= $struct_fields;
						$struct .= "  end;\n";
						if (($struct_type) && ($struct_name != $struct_type)) {
							$struct .= "$struct_type = $struct_name;\n";
							// SEE NOTE BELOW
							//$struct .= $struct_type."Pointer = ^$struct_type;\n";
							$make_pointer = false;
						}
						
						// make an extra pointer for us since Pascal may need it
						// NOTE: remove this until we can protect against duplicate types
						//if ($make_pointer) $struct .= $struct_name."Pointer = ^$struct_name;\n";
						
						$this->dump[$file_name]["types"]["structs"][] = $struct;
						$this->dump["global_structs"][] = $struct_name;
						$got_struct = false;
						$field_bitpacked = false;
					}
				}
				
				
				// got struct
				if (ereg("^typedef struct(.*){", $line, $captures)) {
					
					$struct_name = trim($captures[1], " ");
					$struct_type = null;
					$struct_fields = "";
					
					$make_pointer = false;
					$got_struct = true;
				}
				
				// integer #define
				if (ereg("#define[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([0-9.]+)", $line, $captures)) {
					$this->dump[$file_name]["types"]["defines"][] = $captures[1]." = ".$captures[2].";";
				}
				
				// parse enum fields
				if (($got_enum) || ($got_named_enum)) {
					//print($line."\n");
					
					// insert macros
					//if ($macro = $this->InsertMacroBlocks($line, $this->inside_macro_block)) $this->dump[$file_name]["types"]["enums"][$block_count][] = $macro;

					if (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*([a-zA-Z_]+)[,]*[[:space:]]*$", $line, $captures)) { // string value
						$captures[2] = trim($captures[2], ", ");
						$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";";
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*([0-9-]+)[,]*[[:space:]]*$", $line, $captures)) { // integer value
						$captures[2] = trim($captures[2], ", ");
						$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";";
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*([0-9]+[xX]+[a-fA-F0-9]+)", $line, $captures)) { // hexadecimal value
						$captures[2] = trim($captures[2], ", ");
						$captures[2] = eregi_replace("^0x", "$", $captures[2]);
						$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";";
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*([0-9]+[[:space:]]*<<[[:space:]]*[0-9]+)", $line, $captures)) { // << shl value, no ()
						$captures[2] = str_replace("<<", " shl ", $captures[2]);
						$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";";
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*\(([0-9]+[[:space:]]*<<[[:space:]]*[0-9]+)\)", $line, $captures)) { // << shl value
						$captures[2] = trim($captures[2], ", ");
						$captures[2] = str_replace("<<", " shl ", $captures[2]);
						$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";";
					} elseif (ereg("^[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*[,}]*[[:space:]]*$", $line, $captures)) { // non-value
						
						// omit lines which started nested structures.
						// bad practice but the single-line regex parser can't handle them
						if (!eregi("[=|]+", $line)) {
							$captures[1] = trim($captures[1], ", ");
							$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$auto_increment.";";
							$auto_increment ++;
						}
					}
									
					// found the end
					if (ereg("^};", $line)) $got_enum = false;
				}
				
				// ==== got enum ===
				if (ereg("^enum {", $line)) {
					$got_enum = true;
					$block_count ++;
					$auto_increment = 0;
				}
				
				// terminate named enum
				if ($got_named_enum) {
					if (ereg("^}[[:space:]]*([a-zA-Z0-9_]+);", $line, $captures)) {
						$got_named_enum = false;
						
						$named_enum = trim($named_enum, ", \n");
						
						$this->dump[$file_name]["types"]["named_enums"][] = "$captures[1] = culong;";
						$this->dump["global_types"][$captures[1]] = $captures[1];
					}
				}
				
				// ==== got named enum ===
				if (ereg("^typedef enum {", $line)) {
					$got_named_enum = true;
					$named_enum = "";
					$auto_increment = 0;
					$block_count ++;
				}
				
				// ==== external string constant ===
				if (eregi("^($this->external_string_macros)+[[:space:]]+NSString[[:space:]]+\*[[:space:]]*(const)*[[:space:]]*([a-zA-Z_]+);", $line, $captures)) {
					$name = $captures[3];
					
					if (in_array($name, $this->ignore_symbol)) continue;
					
					$this->dump[$file_name]["types"]["string_constant"][] = "$name: $this->string_macro; external name '_$name';";
				}
				
				// ==== external symbol ===
				if (eregi("^($this->external_string_macros)+[[:space:]]+([a-zA-Z_ ]+)[[:space:]]+([a-zA-Z_]+);", $line, $captures)) {
					$name = $captures[3];
					$type = $captures[2];

					// ignore symbols
					if (in_array($name, $this->ignore_symbol)) continue;
					
					$type = istr_replace_word("const", "", $type);
					$type = trim($type, " ");
					
					$this->dump[$file_name]["types"]["external_symbol"][] = "$name: $type; external name '_$name';";
				}
				
				
				// ==== external procedures ===
				if (ereg("^($this->external_string_macros)+[[:space:]]+(.*)[[:space:]]+(\*)*([a-zA-Z0-9_]+)\((.*)\)", $line, $captures)) {
					
					$result = $this->ConvertReturnType($captures[2]);
					$name = $captures[4];
					$params = "";
					$captures[2] = trim($captures[2], " 	");
					$captures[5] = trim($captures[5], " 	");

					// ignore symbols
					if (in_array($name, $this->ignore_symbol)) continue;
					
					if ($captures[5] != "void") $params = "(".$this->ConvertCParamsPascal($captures[5]).")";
					
					if ($captures[2] == "void") {
						$this->dump[$file_name]["types"]["functions"][] = "procedure $name$params; cdecl; external name '$name';";
					} else {
						$this->dump[$file_name]["types"]["functions"][] = "function $name$params: $result; cdecl; external name '$name';";
					}
				}
				
				// ==== got typedef ===
				if (ereg("^typedef[[:space:]]+struct[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_]+);", $line, $captures)) { // defined struct type
					$real_type = $captures[1];
					$struct_type = $captures[1];
					$new_type = $captures[2];
					
					$this->AddTypeDef($this->dump[$file_name], "$struct_type = Pointer;");
					
					$struct_type = $this->ReplaceObjcType($struct_type);
					$struct_type = $this->SwapObjcTypeWithReal($struct_type);
					$this->AddTypeDef($this->dump[$file_name], "$new_type = $struct_type;");
					
					$this->dump["global_types"][$struct_type] = "Pointer";
					$this->dump["global_types"][$new_type] = $real_type;
				} elseif (ereg("^typedef[[:space:]]+struct[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*]+);", $line, $captures)) { // pointer to struct
					$real_type = $captures[1];
					$clean_name = trim($captures[2], "*");
					$pointer_type = $captures[1];
					
					// ??? maybe check to see if this type exists like NSRect *NSRect is NSRectPointer which exists
					$pointer_type = "Pointer";
						
					//$captures[2] = $this->FormatObjcType($captures[2], $modifiers);
					//$this->dump[$file_name]["types"]["typedef"][] = "$pointer_type = Pointer;";
					$this->AddTypeDef($this->dump[$file_name], "$clean_name = $pointer_type;");
					
					
					//$this->dump["global_types"][$pointer_type] = "Pointer";
					$this->dump["global_types"][$clean_name] = $real_type;
				} elseif (ereg("^typedef[[:space:]]+(const)*[[:space:]]*struct[[:space:]]+([a-zA-Z0-9_*]+)[[:space:]]+([a-zA-Z0-9_]+);", $line, $captures)) { // struct type (complex)
					$real_type = $captures[1];
					
					$captures[2] = $this->FormatObjcType($captures[2], $modifiers);
					$this->AddTypeDef($this->dump[$file_name], $captures[3]." = ".$captures[2].";");
					
					$this->dump["global_types"][$captures[3]] = $real_type;
				} elseif (ereg("^typedef[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*]+);", $line, $captures)) { // single-word type
					$real_type = $captures[1];
					
					// type is a pointer
					if ($captures[2][0] == "*") {
						$captures[2] = trim($captures[2], "*");
						$captures[1] = $this->ReplaceObjcType($captures[1]);
						$captures[1] = $this->SwapObjcTypeWithReal($captures[1]);
						$this->AddTypeDef($this->dump[$file_name], $captures[2]." = ^".$captures[1].";");
						
						$this->dump["global_types"][$captures[2]] = $real_type;
					} else {
						$captures[2] = trim($captures[2], "*");
						$captures[1] = $this->ReplaceObjcType($captures[1]);
						$captures[1] = $this->SwapObjcTypeWithReal($captures[1]);
						$this->AddTypeDef($this->dump[$file_name],$captures[2]." = ".$captures[1].";");

						$this->dump["global_types"][$captures[2]] = $real_type;
					}
				} elseif (ereg("^typedef[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*]+);", $line, $captures)) { // double-word type
					$real_type = $captures[1];
					
					$captures[3] = trim($captures[3], "*");
					$long_type = $captures[1]." ".$captures[2];
					$long_type = $this->ReplaceObjcType($long_type);
					$long_type = $this->SwapObjcTypeWithReal($long_type);
					$this->AddTypeDef($this->dump[$file_name], $captures[3]." = $long_type;");
					
					$this->dump["global_types"][$captures[3]] = $real_type;
				}
			}
			
		//print_r($this->dump[$file_name]["types"]);
	}	
	
	// Parse all protocols in a header
	function ParseHeaderProtocols ($file) {
			$contents = ReadTextFile($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
							
				// parse protocol
				if ($got_protocol) {
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// found property
					if (eregi($this->regex_objc_property, $line, $captures)) {
							$property = $this->ParseClassProperty($current_protocol, $captures);
							
							if ($property["setter"]) {
								$this->current_header["protocols"][$current_protocol]["methods"][$method["objc_method"]] = $property["setter"];
							}
							
							if ($property["getter"]) {
								$this->current_header["protocols"][$current_protocol]["methods"][$method["objc_method"]] = $property["getter"];
							}
							
							continue;
					}
					
					// found method
					$method = null;
					if (eregi($this->regex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_protocol, $line, $captures, array(), true);	
					} elseif (eregi($this->regex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_protocol, $line, $captures, array(), false);
					}

					// append to classes
					if (($method) && (!in_array($current_protocol, $this->ignore_categories)) && (!in_array($method["name"], $this->ignore_methods)) ) {
						$this->current_header["protocols"][$current_protocol]["methods"][$method["objc_method"]] = $method;
					}
					
					// found the end
					if (ereg("^@end", $line)) $got_protocol = false;
				}
				
				// got protocol
				if ((eregi($this->regex_objc_protocol, $line, $captures)) && (!eregi(".*;$", $line))) {
						$got_protocol = true;
						$current_protocol = $captures[1];
						print("+ Protocol $current_protocol\n");
						$this->current_header["protocols"][$current_protocol]["name"] = $captures[1];
				}
			}
			
		//print_r($this->current_class);
	}
	
	
	// Parse all categories in a header
	function ParseHeaderCategories ($file) {
			$contents = ReadTextFile($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
							
				// parse category
				if ($got_category) {
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// found property
					if (eregi($this->regex_objc_property, $line, $captures)) {
							$property = $this->ParseClassProperty($current_category, $captures);
							
							if ($property["setter"]) {
								if ($this->AddMethodToClass($property["setter"], $this->current_class)) {
									$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["methods"][] = $property["setter"];
								}
							}
							
							if ($property["getter"]) {
								if ($this->AddMethodToClass($property["getter"], $this->current_class)) {
									$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["methods"][] = $property["getter"];
								}
							}

							continue;
					}
					
					// found method
					$method = null;
					if (eregi($this->regex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, $this->GetProtectedKeywords($this->current_class), true);						
					} elseif (eregi($this->regex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, $this->GetProtectedKeywords($this->current_class), false);	
					}
					
					// append to classes
					if (($method) && (!in_array($method["name"], $this->ignore_categories))) {
						if ($current_class) {
							if ($this->AddMethodToClass($method, $this->current_class)) {
								$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["methods"][] = $method;
							}
						} else {
							
							// add base categories to NSObject
							if (in_array($current_category, $this->base_categories)) {
								if ($this->AddMethodToClass($method, $this->dump["NSObject.h"]["classes"]["NSObject"])) {
									$this->dump["NSObject.h"]["classes"]["NSObject"]["categories"][$current_category]["methods"][] = $method;
								}
							}
							
							$this->dump["categories"][$current_category]["methods"][$method["objc_method"]] = $method;
						}
					}
					
					// found the end
					if (ereg("^@end", $line)) $got_category = false;
				}
				
				// got category
				if (eregi($this->regex_objc_category, $line, $captures)) {
					
					// ??? if the current header is NSObject, then we DO want to accept these categories, they are NOT delegates this time...
					
					// append category to it's super class
					$category_owner = $this->FindCategoryHeader($captures[1]);
					if (($category_owner) && ($captures[1] != "NSObject")) {
						$got_category = true;
						$current_category = $captures[2];
						$current_class = $captures[1];
						$this->current_class = &$this->dump[$category_owner]["classes"][$current_class];
						
						$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["name"] = $captures[2];
						$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["super"] = $captures[1];
						
						print("	-> Category $current_category belongs to $current_class in $category_owner\n");
					} else {
						
						if ($captures[1] == "NSObject") {
							print("	+ Category ".$captures[2]."->".$captures[1]." belongs to NSObject\n");
							$got_category = true;
						} else {
							$this->warning_count ++;
							print("# WARNING: Category ".$captures[2]." (".$captures[1].") has no header\n");
							$got_category = false;
						}
						
						$current_category = $captures[2];
						$current_class = null;

						$this->dump["categories"][$current_category]["name"] = $captures[2];
						$this->dump["categories"][$current_category]["super"] = $captures[1];
					}
					
				}
			}
			
		//print_r($this->current_class);
	}
	
	// Parse all "pre-defined" category methods in a header
	function PreparseCategoryMethods ($file) {
			$contents = ReadTextFile($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
							
				// parse category
				if ($got_category) {
					
					// found method
					$method = null;
					if (eregi($this->regex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, array(), true);						
					} elseif (eregi($this->regex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, array(), false);	
					}
					
					// append to classes
					if (($method) && ($current_class)) {
						$this->dump[$category_owner]["category_methods"][] = $method["name"];
						//print($method["name"]."\n");
					}
					
					// found the end
					if (ereg("^@end", $line)) $got_category = false;
				}
				
				// got category
				if (eregi($this->regex_objc_category, $line, $captures)) {
					$category_owner = $this->FindCategoryHeader($captures[1]);
					if ($category_owner) {
						$got_category = true;
						$current_category = $captures[2];
						$current_class = $captures[1];
					} else {
						$current_class = null;
					}
				}
			}
			
		return $this->dump[$category_owner]["category_methods"];
	}
	
	// Preparses a class for protected keywords
	function PreparseClass ($lines, $line_count) {
		$protected_keywords = array();
		
		for ($i=$line_count; $i < count($lines); $i++) { 
			$line = $lines[$i - 1];
			
			// found method
			if (eregi($this->regex_objc_method_params, $line, $captures)) {
				$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $protected_keywords, true);						
				$this->current_class["protected_keywords"][] = $method["name"];
			} elseif (eregi($this->regex_objc_method_no_params, $line, $captures)) {
				$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $protected_keywords, false);
				$this->current_class["protected_keywords"][] = $method["name"];
			}
			
			// class ended
			if (ereg("^@end", $line)) return $protected_keywords;
		}
	}
	
	// Gets the preferred property name from attributes
	function GetPropertyName ($kind, $params, &$name) {
		foreach ($params as $value) {
			$pair = explode("=", $value);
			
			if ($pair[0] == $kind) {
				$name = $pair[1];
				return true;
				break;
			}
		}
	}
	
	// Convert a method return type to Pascal
	function ConvertReturnType ($type) {
		$type = trim($type, " ");
		$type = $this->ReplaceObjcType($type);

		// if the type was not converted remove the * and process further
		$type = trim($type, "* ");
		$type = $this->ReplaceObjcType($type);

		// format the return type again to make sure it's clean
		$type = $this->FormatObjcType($type, $null_modifier);
		
		return $type;
	}
	
	// Parse a property into accessor methods
	function ParseClassProperty ($class, $parts) {
		$property["parameters"] = explode(",", $parts[1]);
		$method = array();
		
		// property name
		if (eregi("([a-zA-Z0-9]+)$", $parts[2], $captures)) $property["name"] = ucwords($captures[1]);
		
		// property type
		$type = istr_replace_word($captures[1], "", $parts[2]);
		$type = $this->ConvertReturnType($type);
		
		// setter
		if (!in_array("readonly", $property["parameters"])) {
			$method["setter"] = array();
			
			$name = $property["name"];
			if (!$this->GetPropertyName("setter", $property["parameters"], $name)) {
				$name = "set$name";
			}
			
			// protect method name from keywords
			if ($this->IsKeywordReserved($name)) $name .= "_";
			
			$method["setter"]["def"] = "procedure $name (newValue: $type);";
			$method["setter"]["objc_method"] = "$name:";
			$method["setter"]["class"] = $class;
			$method["setter"]["name"] = $name;
			$method["setter"]["kind"] = "procedure";
		}
		
		// getter
		$method["getter"] = array();
		
		$name = $property["name"];
		if (!$this->GetPropertyName("getter", $property["parameters"], $name)) {
			$name = strtolower(substr($name, 0, 1)) . substr($name, 1);
		}
		
		// protect method name from keywords
		if ($this->IsKeywordReserved($name)) $name .= "_";
		
		$method["getter"]["def"] = "function $name: $type;";
		$method["getter"]["objc_method"] = $name;
		$method["getter"]["class"] = $class;
		$method["getter"]["name"] = $name;
		$method["getter"]["kind"] = "function";
		
		return $method;
	}
	
	// Main entry to parse a header
	function ParseHeaderClasses ($file) {
			$contents = ReadTextFile($file);
			
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			$line_count = 0;
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
				$line_count++;
				
				// remove external class macros
				$line = eregi_replace("^[A-Z0-9]+_EXTERN_CLASS[[:space:]]+", "", $line);
				
				// parse instance vars
				if ($got_instance_vars) {
					
					// scope compiler directive
					if (eregi($this->regex_scope_compiler_directive, $line, $captures)) {
						$this->instance_var_scope = $captures[1];
						continue;
					}
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// parse instance variables
					$result = $this->ParseInstanceVariables($line, $struct);
					
					// parse structures
					if ($result == "struct") {
						//print_r($struct);
						//$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": $current"."_".$struct["name"].";";
						$this->dump[$file_name]["classes"][$current]["ivars_structs"][] = $struct;
						
						// print inline-record type
						if ($struct["bitpacked"]) {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": ".$this->bitpacked_record_keyword;
						} else {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": ".$this->record_keyword;
						}
						
						
						// print fields
						if ($struct["fields"]) {
							foreach ($struct["fields"] as $field) $this->dump[$file_name]["classes"][$current]["ivars"][] = "    ".$field;
						}
						$this->dump[$file_name]["classes"][$current]["ivars"][] = "  end;";
						
						$struct = null;
					} elseif($result != null) {
						//print($result);
						$this->dump[$file_name]["classes"][$current]["ivars"][] = $result;
					}
					
					// instance var section terminated.
					if (eregi("^\s*}\s*$", $line)) {
						$struct = null;
						$got_instance_vars = false;
						$this->instance_var_scope = null;
					}
					
				} elseif ($got_class) { // parse the class
					
					// the instance variable section started after the class line and no other ivar's were parsed yet
					if (!$this->dump[$file_name]["classes"][$current]["ivars"]) {
						if (eregi("{\s*$", $line)) {
							$got_instance_vars = true;
							continue;
						}
					}
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// found property
					if (eregi($this->regex_objc_property, $line, $captures)) {
							$property = $this->ParseClassProperty($current, $captures);
							
							if ($property["setter"]) {
								if ($this->AddMethodToClass($property["setter"], $this->dump[$file_name]["classes"][$current])) {
									$this->dump[$file_name]["classes"][$current]["methods"][] = $property["setter"];
								}
							}
							
							if ($property["getter"]) {
								if ($this->AddMethodToClass($property["getter"], $this->dump[$file_name]["classes"][$current])) {
									$this->dump[$file_name]["classes"][$current]["methods"][] = $property["getter"];
								}
							}
							
							continue;
					}
					
					// found method
					if (eregi($this->regex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $this->GetProtectedKeywords($this->current_class), true);						
						if ($this->AddMethodToClass($method, $this->dump[$file_name]["classes"][$current])) {
							$this->dump[$file_name]["classes"][$current]["methods"][] = $method;
						}
						
					} elseif (eregi($this->regex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $this->GetProtectedKeywords($this->current_class), false);
						if ($this->AddMethodToClass($method, $this->dump[$file_name]["classes"][$current])) {
							$this->dump[$file_name]["classes"][$current]["methods"][] = $method;
						}
					}
					
					// found the end
					if (ereg("^@end", $line)) $got_class = false;
				}
				
				// ==== got class ====
				if ((eregi($this->regex_objc_class, $line, $captures)) || (eregi($this->regex_objc_class_no_super, $line, $captures))) {
					$current = $captures[1];
					$got_class = true;
					
					// check for instance variable section
					if (eregi("{\s*$", $line)) $got_instance_vars = true;
					
					// get the protocol which the class conforms to
					if (eregi($this->regex_objc_class, $line, $captures)) {
						if ($captures[3]) $this->dump[$file_name]["classes"][$current]["conforms"] = $captures[3];
					} else {
						if ($captures[2]) $this->dump[$file_name]["classes"][$current]["conforms"] = $captures[2];
					}
					
					// clean up the conforms string
					if ($this->dump[$file_name]["classes"][$current]["conforms"]) {
						$conform_protocols = explode(",", $this->dump[$file_name]["classes"][$current]["conforms"]);
						
						foreach ($conform_protocols as $protocol) {
							$protocol = trim($protocol, "<> ");
							$protocol_clean .= $protocol."$this->protocol_suffix, ";
						}
						
						$protocol_clean = trim($protocol_clean, ", ");
						$this->dump[$file_name]["classes"][$current]["conforms"] = $protocol_clean;
						
						$protocol_clean = "";
					}
					
					$this->dump[$file_name]["classes"][$current]["name"] = $captures[1];
					$this->dump[$file_name]["classes"][$current]["super"] = $captures[2];
					$this->dump[$file_name]["classes"][$current]["super_class"] = &$this->dump["master"][$captures[2]];
					$this->dump[$file_name]["classes"][$current]["file_name"] = $file_name;
					$this->dump[$file_name]["classes"][$current]["file_clean"] = substr($file_name, 0, (strripos($file_name, ".")));
					$this->dump[$file_name]["classes"][$current]["protected_keywords"] = array();
					$this->dump[$file_name]["classes"][$current]["declared_methods"] = array();
					$this->dump[$file_name]["category_methods"] = array();
					
					$this->current_class = &$this->dump[$file_name]["classes"][$current];
					
					// append master class listing
					$this->dump["master"][$current] = &$this->dump[$file_name]["classes"][$current];
					
					// preparse for protected keywords
					$this->PreparseClass($lines, $line_count);
					
					// preparse for category methods that may present naming conflicts
					$category_methods = $this->PreparseCategoryMethods($file);
					
					// add category methods to protected keywords
					if ($category_methods) $this->current_class["protected_keywords"] = array_merge($this->current_class["protected_keywords"], $category_methods);
					
					// print class hierarchy
					if ($this->show_class_hierarchy) {
						$this->GetClassHierarchy($this->current_class, $hierarchy);
						$hierarchy_string = "";
						foreach ($hierarchy as $value) {
							$hierarchy_string .= "$value->";
						}
						$hierarchy_string = trim($hierarchy_string, "->");
						print("	- $current: $hierarchy_string\n");
					}
					
					$this->class_count ++;
					//print_r($this->dump[$file_name]["classes"][$current]);
				}
				
			}
		
		//print_r($this->dump[$file_name]["classes"][$current]);
	}		
	
	// Parse categories which depend on another header
	function ParseHeaderDependents ($file) {
		$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));

		$this->ParseHeaderCategories($file);
			
		print("+ Parsed $file_name for dependents\n");
	}
	
	
	// Main entry to parse a header
	function ParseHeader ($file) {
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			$name_clean = substr($file_name, 0, (strripos($file_name, ".")));	
			
			// get framework we're parsing from
			if (eregi("/([a-zA-Z]+)\.framework/", $file, $captures)) $this->framework = strtolower($captures[1]);
			
			// get the output path
			$this->dump[$file_name]["path"] = "$this->root$this->out/$this->framework/$name_clean.inc";
			$this->dump[$file_name]["path_partial"] = "$this->framework/$name_clean.inc";
			$this->dump[$file_name]["framework"] = $this->framework;
			$this->dump[$file_name]["name"] = $file_name;
			$this->dump[$file_name]["name_clean"] = $name_clean;
			$this->current_header = &$this->dump[$file_name];
			
			$this->ParseHeaderProtocols($file);
			$this->ParseHeaderClasses($file);
			$this->ParseHeaderTypes($file);
			
			print("+ Parsed $file_name\n");
	}
			
	// Parse all AppKit and Foundation framework headers
	function ParseCocoaFrameworks ($ignore_files, $parse_only) {
		
		foreach ($this->frameworks as $framework_name => $framework_info) {
			
			// framework is disabled
			if ($framework_info["enabled"] != 1) continue;

			if ($this->out != "/") {
				$path = $this->root.$this->out."/".$framework_info["root"];
			} else {
				$path = $this->root.$framework_info["root"];
			}
				
			$contents = ReadTextFile($path);
			$lines = explode("\n", $contents);
			
			foreach ($lines as $line) {
				if (eregi($framework_info["include_pattern"], $line, $captures)) {
					$header = $captures[1].".h";
					$path = $framework_info["headers"]."/$header";
					
					// main header
					if ($parse_only) {
						if (@in_array($header, $parse_only)) $this->ParseHeader($path);
					} elseif (@!in_array($header, $ignore_files)) {
						 $this->ParseHeader($path);
					}

					// header dependents
					if ($parse_only) {
						if (@in_array($header, $parse_only)) $this->ParseHeaderDependents($path);
					} elseif (@!in_array($header, $ignore_files)) {
						 $this->ParseHeaderDependents($path);
					}
				}
			}
			
		}

		// diagnostics
		print("\n• Parsed $this->method_count methods in $this->class_count classes.\n\n");
		
		if ($this->warning_count > 0) print("• $this->warning_count warnings were encountered.\n\n");
	}
		
	// Parse headers in a system framework
	function ParseFramework ($ignore_files, $parse_only) {
	}
		
	// Parses XML file generated by GEN_BRIDGE_METADATA -f /System/Library/Frameworks/AppKit.framework/
	function ParseBridgeSupportXML ($file, $categories) {
		$contents = ReadTextFile($file);
			
		$lines = explode("\n", $contents);
		foreach ($lines as $line) {
			
			if ($got_informal_protocol) {
				
				if (eregi("<method type='(.*)' selector='(.*)'/>", $line, $captures)) {
					
					$set["name"] = $captures[2];
					$set["name_pascal"] = str_replace(":", "_", $set["name"]);
					$set["name_pascal"] = rtrim($set["name_pascal"], "_");
					$set["types"] = $captures[1];
					$set["param_string"] = $categories[$informal_protocol]["methods"][$captures[2]]["param_string"];
					$set["method"] = &$categories[$informal_protocol]["methods"][$captures[2]];
					
					if ($captures[1][0] == "v") {
						$set["kind"] = "procedure";
					} else {
						$set["kind"] = "function";
					}
					
					// add the selector if the name isn't reserved for Pascal
					if ((!in_array($set["name_pascal"], $this->reserved_keywords)) && (!in_array($set["name_pascal"], $this->reserved_methods))) {
						$this->delegate_methods[$informal_protocol][] = $set;
						$this->delegate_method_names[] = $set["name_pascal"];
					}
				}
				
				// end tag
				if ($line == "</informal_protocol>") $got_informal_protocol = false;
			}
			
			// got informal_protocol
			if (eregi("<informal_protocol name='(.*)'>", $line, $captures)) {
				$informal_protocol = $captures[1];
				//print("\"$informal_protocol\", ");
				$got_informal_protocol = true;
			}
		}
		
		print("+ Parsed bridge support XMl file at $file\n");
		//print_r($this->delegate_methods);
	}		
		
	// Parse all classes/categories (non-delegate) from the header
	function ParseAllHeaderClasses ($file) {
		$contents = ReadTextFile($file);
			
		$lines = explode("\n", $contents);
		foreach ($lines as $line) {
			
			// remove external class macros
			$line = eregi_replace("^[A-Z0-9]+_EXTERN_CLASS[[:space:]]+", "", $line);
			
			// classes
			if (eregi($this->regex_objc_class, $line, $captures)) $this->cocoa_classes[] = $captures[1];
			if (eregi($this->regex_objc_class_no_super, $line, $captures)) $this->cocoa_classes[] = $captures[1];
			
			// categories
			if (eregi($this->regex_objc_category, $line, $captures)) {
				$this->cocoa_categories[] = $captures[1];
			}
		}
	}
		
	// Build array of all known Cocoa classes in frameworks	
	function BuildCocoaClasses () {
	
		foreach ($this->frameworks as $framework_name => $framework_info) {
			
			// framework is disabled
			if ($framework_info["enabled"] != 1) continue;
			
			$handle = opendir($framework_info["headers"]);
			while (($file = readdir($handle)) !== false) {
				if (eregi($framework_info["header_pattern"], $file)) {
					$this->ParseAllHeaderClasses($framework_info["headers"]."/$file");
				}
			}
			closedir($handle);
		}
	}		

	function ProcessFile ($file, $print) {
		$this->ParseHeader($file);
		$this->ParseHeaderDependents($file);

		if ($print) $this->PrintAllHeaders("", null, null, false);
	}

	function ParseDelegateClasses () {
		
		foreach ($this->frameworks as $framework_name => $framework_info) {
			
			// framework is disabled
			if ($framework_info["enabled"] != 1) continue;
			
			$this->ParseBridgeSupportXML("$this->root".$framework_info["bridge"], $this->dump["categories"]);
		}

		// These are expressions which match valid class names or the names themself
		$delegate_categories = array(	"(Delegation|Delegate|Notification|DataSource|Handler)+", 
										"NSDraggingDestination", "NSDistantObjectRequestMethods", "NSDraggingSource",
										"NSEditorRegistration", "NSFileManagerFileOperationAdditions", "NSPasteboardOwner",
										);

		$this->output = fopen("$this->root$this->out/foundation/$this->master_delegate_file.inc", "w+");
		$this->PrintDelegateClass($delegate_categories);
		fclose($this->output);
		
		$this->output = fopen("$this->root$this->out/$this->master_delegate_file.pas", "w+");
		$this->PrintDelegateReference($delegate_categories);
		fclose($this->output);
	}
	
	function LoadTypeEncodings ($name) {
		$contents = ReadTextFile("$this->root/$name");
		
		$lines = explode("\n", $contents);
		foreach ($lines as $line) {
			$row = explode("|", $line);
			
			$this->type_encodings[$row[0]][$row[1]] = $row[2];
		}
		
		//print_r($this->type_encodings);
	}
	
	// Prints out code to generate type encodings with GenerateTypeEncodings.p
	// Paste the output of the function into GenerateTypeEncodings.p, run the program and save the output into a text file
	// which is loaded into this script.
	function PrintTypeEncodingGlue () {
		$count = 0;
		$block = true;
		$block_count = 1;
		$limit = 2000;
		
		foreach ($this->dump["all_methods"] as $class => $method) {
			foreach ($method as $name) {
				
				if ($count == 0) {
					print("\n");
					print("procedure PrintGlue$block_count;\n");
					print("begin\n");
					
					$block_count ++;
				}
				
				$count ++;
				
				print("aMethod := class_getInstanceMethod(objc_getClass('$class'), sel_registerName(PChar('$name')));\n");
				print("if aMethod <> nil then\n");
				print("writeln('$class|$name|', method_getTypeEncoding(aMethod));\n");
				
				if ($count == $limit) {
					print("end;\n");
					$count = 0;
				}
			}
		}
		
		if ($count < $limit) {
			print("end;\n");
			$block_count --;
		}
		
		print("\n========= IMPLEMENTATION =========\n");
		for ($i=1; $i < $block_count + 1; $i++) { 
			print("PrintGlue$i;\n");
		}
	}
		
	function __construct ($directory, $out_directory, $frameworks, $show)  {
		$this->root = $directory;
		$this->out = $out_directory;
		$this->show = $show;
		if ($frameworks) {
			foreach ($frameworks as $name) {
				$this->frameworks[$name]["enabled"] = true;
			}
		}
		$this->BuildCocoaClasses();
		//$this->LoadTypeEncodings("TypeEncodingsAll.txt");
	}
}
?>