<?php

class TObjPParser extends TPasCocoaParser {
	
	var $objc_id = "id";							// Default type for generic objects
	var $objc_id_real = "id";						// The real type of generic objects (id)
	var $sel_string = "SEL";						
	var $trailing_underscore = true;
	var $print_header_references = false;
	
	// ignore these classes when testing for ivar size
	var $ignore_class_ivar_comparison = array(	"NSNibOutletConnector", "NSNibConnector", "NSNibControlConnector", "NSPredicateEditorRowTemplate", "NSSegmentedCell",
												"NSSimpleHorizontalTypesetter", "NSInvocation", "NSPointerFunctions", "NSConstantString");
	
	var $reserved_keywords = array(	"const", "object", "string", "array", "var", "set", "interface", "classname", "unit",
									"self", "type", "raise", "property", "to", "for", "with", "function", "procedure", "result",
									"pointer", "create", "new", "dispose", "label", "packed", "record", "char", "class", "implementation",
									
									// identifiers from NSObject
									"zone", 
									);

	var $replace_types = array(	"void"=>"Pointer", "BOOL"=>"Boolean", "long"=>"clong", "int"=>"cint",
								"unsigned long"=>"culong", "unsigned short"=>"cushort", "void *"=>"Pointer", "unsigned int"=>"cuint",
								"Class"=>"Pobjc_class", "uint"=>"cuint",
								"uint8_t"=>"byte", "signed int"=>"cint", "const char"=>"char", "const void"=>"Pointer",
								"const uint8_t"=>"byte", "unsigned"=>"cuint", "int32_t"=>"longint", "float"=>"single",
								"unsigned long long"=>"culonglong", "int64_t"=>"clonglong", "uint32_t"=>"cardinal", "uint16_t"=>"word",
								"unsigned char"=>"char", "short"=>"cshort", "double"=>"double", "long long"=>"clonglong",
								
								// ??? new in instance var parser: (add to main section eventually)
								"signed char"=>"char", "uint64_t"=>"qword", 
								
								// work-arounds - the type replacement needs regex to handle with spaces I guess
								"void*"=>"Pointer",
								
								// macros
								"IBAction"=>"void", "IBOutlet"=>"",
								
								// special pointers
								"const id *"=>"NSObjectArrayOfObjectsPtr", "Protocol *"=>"Protocol", "NSObject *"=>"NSObject",
								"const char *"=>"PChar", "const void *"=>"Pointer", "unsigned char *"=>"Pointer", "char *"=>"PChar",
								"unsigned *"=>"Pointer", "unichar *"=>"PWideChar", "const unichar *"=>"PWideChar", 
								);
		
	// These methods require that the last parameter append a trailing underscore (if $trailing_underscore is on)
	var $trailing_underscore_methods = array("- (void)copy:(id)sender;", "- (void)setNeedsDisplay:(BOOL)flag;");

	// We use direct Cocoa classes now always
	var $toll_free_bridge = array();
	
	var $ignore_methods = array("observationInfo");	

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
		if ($this->trailing_underscore) {
		 if (!in_array($method, $this->trailing_underscore_methods)) $name = trim($name, "_");	
		}
		
		$name = $this->ReplaceObjcType($name);
		
		return $name;
	}	

	// We use direct objc classes now so we don't need to replace them with references like in PasCocoa
	function ReplaceNSTypesWithRef ($string) {
		return $string;
	}

	// Converts an Objective-C method to Pascal format 
	function ConvertObjcMethodToPascal ($class, $source, $parts, $protected_keywords, $has_params) {
		
		// remove deprecated macros from method source
		$source = eregi_replace("[[:space:]]*DEPRECATED_IN_MAC_OS_X_VERSION_[0-9]+_[0-9]+_AND_LATER", "", $source);
		
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
			$variable_arguments = false;
		}

		// protect method name from keywords
		if ($this->IsKeywordReserved($name)) $name .= "_";

		// replace objc type
		$return_type = $this->ConvertReturnType($return_type_clean);

		$virtual = "";
		$class_prefix = "";

		// determine the type based on return value
		if (ereg($this->regex_procedure_type, $return_type_clean)) {
			$kind = "procedure";
		} else {
			$kind = "function";
		}

		// determine if this is a class method
		if ($parts[1] == "+") {
			$class_prefix = "class ";
		
			// These methods probably return the an allocated instance of the class, a typical convenience method.
			// ??? Ack! $class may be the category or protocol name
			//if ($return_type == $this->objc_id) $return_type = $class; 
		}

		// Replace SEL with the string equivalent
		if ($this->register_selectors) {
			$params_with_modifiers = str_replace_word("SEL", $this->sel_string, $params_with_modifiers);
		}

		// make method templates
		if ($kind != "function") {
			if ($variable_arguments) $modifier .= " varargs;";
		
			$method = "$class_prefix$kind $name$params_with_modifiers;$modifier$virtual";
			$method_template = "[KIND] [PREFIX]$name"."[PARAMS];$modifier";
		} else {
			if ($variable_arguments) $return_type = "$return_type; varargs";

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

		// FPC bug work around
		if (strlen($name) > $this->maximum_method_length) {
			$struct["can_override"] = false;
			print("	# WARNING: method $name can't override because the name is too long\n");
			$this->warning_count ++;
		}

		return $struct;
	}
	
	function InsertPatches ($header) {
		$path = "$this->root/patches/".$header["name_clean"].".patch";
		if ($handle = @fopen($path, "r")) {
			$text = ReadTextFile($path);
			$this->PrintOutput(0, $text);
			fclose($handle);
		}
	}
	
	function HeaderContainsPatch ($header) {
		if ($handle = @fopen("$this->root/patches/".$header["name_clean"].".patch", "r")) {
			fclose($handle);
			return true;
		}
	}
	
	// Prints all classes from the header in Objective-P FPC format
	function PrintHeader ($header) {
		global $version;
		
		$this->output = fopen($header["path"], "w+");

		$this->PrintOutput(0, "{ Parsed from ".ucfirst($header["framework"]).".framework ".$header["name"]." }");

		$date = date("D M j G:i:s T Y");
		$this->PrintOutput(0, "{ Version $version - $date }");
		$this->PrintOutput(0, "");

		$macro = strtoupper(substr($header["name"], 0, (strripos($header["name"], "."))));
		
		/*
		if ($header["classes"]) {
			$this->PrintOutput(0, "{\$ifdef HEADER}");
			$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_H}");
			$this->PrintOutput(0, "{\$define $macro"."_PAS_H}");
			$this->PrintOutput(0, "type");
			
			foreach ($header["classes"] as $class) {
				
				// Make a pointer to each class
				$this->PrintOutput(1, $class["name"]."Pointer = Pointer;");
			}
		
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}
		*/
		
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
		
		// Records from types
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
		$this->PrintOutput(0, "{\$ifdef EXTERNAL_SYMBOLS}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_S}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_S}");
		$this->PrintExternalSymbols($header);
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$endif}");
		$this->PrintOutput(0, "{\$endif}");
		
		// insert user patches
		if ($this->HeaderContainsPatch($header)) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$ifdef USER_PATCHES}");
			//$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_PATCH}");
			//$this->PrintOutput(0, "{\$define $macro"."_PAS_PATCH}");
			$this->InsertPatches($header);
			$this->PrintOutput(0, "");
			//$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}

		if (($header["classes"]) || ($header["protocols"])) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$ifdef FORWARD}");

			if ($header["protocols"]) {
				foreach ($header["protocols"] as $protocol) $this->PrintOutput(1, $protocol["name"]."$this->protocol_suffix = objcprotocol;");
			}
			
			if ($header["classes"]) {
				foreach ($header["classes"] as $class) {
					$this->PrintOutput(1, $class["name"]." = objcclass;");
					$this->PrintOutput(1, $class["name"]."Pointer = ^".$class["name"].";");
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
				//if (in_array($class["name"], $this->cocoa_classes))
				$this->PrintClass($class);
			}

			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}


		if ($header["protocols"]) {
			$this->PrintOutput(0, "{\$ifdef PROTOCOLS}");
			$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_P}");
			$this->PrintOutput(0, "{\$define $macro"."_PAS_P}");

			
			foreach ($header["protocols"] as $protocol) {
				$this->PrintOutput(1, "");
				$this->PrintOutput(0, "{ ".$protocol["name"]." Protocol }");
				$this->PrintOutput(1, $protocol["name"]."$this->protocol_suffix = objcprotocol");

				// print methods
				if ($protocol["methods"]) {
					foreach ($protocol["methods"] as $name => $method) {
						$this->PrintOutput(2, $method["def"]." message '".$method["objc_method"]."';");
					}
				}
				
				$this->PrintOutput(1, "end; external name '".$protocol["name"]."';");		
			}
			
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}
	}
	
	function PrintClass ($class) {

		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ ".$class["name"]." }");
		//print_r($class["methods"]);
		
		// print super class or protocol which the class conforms to
		if ($class["conforms"]) {
			$this->PrintOutput(1, $class["name"]." = objcclass(".$class["super"].", ".$class["conforms"].")");
		} elseif ($class["super"]) {
			$this->PrintOutput(1, $class["name"]." = objcclass(".$class["super"].")");
		}

		// print instance variables
		if ($class["ivars"]) {
			$this->PrintOutput(1, "private");
			foreach ($class["ivars"] as $ivar) {
				$this->PrintOutput(2, $ivar);
			}
		}

		// print alloc method for the class
		$this->PrintOutput(2, "");
		$this->PrintOutput(1, "public");
		$this->PrintOutput(2, "class function alloc: ".$class["name"]."; message 'alloc';");
		
		// print class-level methods
		if ($class["methods"]) {
			$this->PrintOutput(0, "");
			foreach ($class["methods"] as $method) {
				$this->PrintOutput(2, $method["def"]." message '".$method["objc_method"]."';");
			}
		}

		// print category-level methods
		if (count($class["categories"]) > 0) {
			foreach ($class["categories"] as $name => $category) {
				$this->PrintOutput(0, "");
				$this->PrintOutput(2, "{ Category: $name }");

				if ($category["methods"]) {
					foreach ($category["methods"] as $method) {
						$this->PrintOutput(2, $method["def"]." message '".$method["objc_method"]."';");
					}
				}	
			}
		}

		$this->PrintOutput(1, "end; external;");
	}

	function PrintDelegateReference ($valid_categories) {
		global $version;

		$date = date("D M j G:i:s T Y");
		$this->PrintOutput(0, "{ Version $version - $date }");
		$this->PrintOutput(0, "");

		ksort($this->delegate_methods);
		
		$this->PrintOutput(0, "unit $this->master_delegate_file;");
		$this->PrintOutput(0, "interface");
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ Copy and paste these delegate methods into your real classes. }");
		
		
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
			
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "type");
			$this->PrintOutput(1, "$category = objccategory (NSObject)");
			//$this->PrintOutput(1, "public");
			
			foreach ($selectors as $selector) {
				
				// FPC long name bug work-around
				if (strlen($selector["name_pascal"]) > $this->maximum_method_length) continue;
				
				if ($selector["kind"] == "procedure") {
					$this->PrintOutput(2, $selector["kind"]." ".$selector["name_pascal"].$selector["param_string"].";"." message '".$selector["name"]."';");
				} else {
					$this->PrintOutput(2, $selector["kind"]." ".$selector["name_pascal"].$selector["param_string"].": ".$selector["method"]["return"].";"." message '".$selector["name"]."';");
				}
			}
			
			$this->PrintOutput(1, "end;");
		}
		
	}

	function PrintIvarSizeComparison ($path) {
		$count = 0;
		$block = true;
		$block_count = 1;
		$limit = 2000;
		
		$handle = fopen($path, "w+");
		if (!$handle) die("Bad path to size comparison output program!");
		
		fwrite($handle, "{\$mode objfpc}\n");
		fwrite($handle, "{\$modeswitch objectivec1}\n");

		fwrite($handle, "program IvarSize;\n");
		fwrite($handle, "uses\n");
		fwrite($handle, " objp,objcrtl,objcrtlmacosx;\n");
		
		// print derived classes
		foreach ($this->cocoa_classes as $class) {
			if (in_array($class, $this->ignore_class_ivar_comparison)) continue;
			if ($previous == $class) continue;
			
			fwrite($handle, "type\n");
			fwrite($handle, " TDerived$class = objcclass ($class)\n");
			fwrite($handle, " extrabyte: byte;\n");
			fwrite($handle, "end;\n");
			
			$previous = $class;
		}
		
		// print procedures
		foreach ($this->cocoa_classes as $class) {
			if (in_array($class, $this->ignore_class_ivar_comparison)) continue;
			if ($previous == $class) continue;
			
			if ($count == 0) {
				fwrite($handle, "\n");
				fwrite($handle, "procedure PrintGlue$block_count;\n");
				fwrite($handle, "begin\n");
				
				$block_count ++;
			}
			
			$count ++;
			
		 	fwrite($handle, " if class_getInstanceSize(TDerived$class) <> (class_getInstanceSize($class)+1) then\n");
		    fwrite($handle, " writeln('size of $class is wrong: ',class_getInstanceSize(TDerived$class),' <> ',class_getInstanceSize($class)+1);\n");
			
			if ($count == $limit) {
				fwrite($handle, "end;\n");
				$count = 0;
			}
			
			$previous = $class;
		}
		
		if ($count < $limit) {
			fwrite($handle, "end;\n");
			$block_count --;
		}
		
		fwrite($handle, "begin\n");
		for ($i=1; $i < $block_count + 1; $i++) { 
			fwrite($handle, " PrintGlue$i;\n");
		}
		fwrite($handle, "end.\n");
	}

}
?>
