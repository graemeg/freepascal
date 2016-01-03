<?php

// Includes
require_once("utilities.php");
require_once("objp_base.php");
require_once("docset.php");

// Constants
define("CAST_HANDLE", true);
define("DONT_CAST_HANDLE", false);

define("ACCESS_HANDLE_DIRECT", 1);
define("ACCESS_HANDLE_FUNCTION", 2);

define("REGISTER_SEL", true);
define("DONT_REGISTER_SEL", false);

define("USE_HANDLE", true);
define("DONT_USE_HANDLE", false);
	
class ObjectivePParser extends ObjectivePParserBase {

	/**
	 * UTILITIES
	 */
	
	// Skips blocks in the current file being parsed
	function SkipBlock ($line) {
		
		if ($line != "") {
			foreach ($this->skip_blocks as $key => $value) {
				if (@ereg($key, $line)) {
					$this->parser_skipping = true;
				}
				if (@ereg($value, $line)) {
					$this->parser_skipping = false;
				}
			}
		}
		
		return $this->parser_skipping;
	}
	
	function IsKeywordReserved($keyword) {
		$keyword = strtolower($keyword);
		if (in_array($keyword, $this->reserved_keywords)) return true;
		if (($this->current_class != null) &&
		    ($this->dump["master"][$this->current_class["name"]] ["field_names"] != null) &&
		    in_array($keyword, $this->dump["master"][$this->current_class["name"]] ["field_names"])) return true;
	}
		
	// Extracts the name from an Objective-C method definition
	function ExtractObjcMethodName ($method) {
		
		// cut out comments first
		$method = eregi_replace("(/\*.*\*/)", "", $method);
		$method = eregi_replace("//.*$", "", $method);
		$method = trim($method, " 	");
		
		$params = explode(":", $method);
		$name = "";
		
		if (count($params) > 1) {
			foreach ($params as $value) {
				$value = trim($value, " 	");
				
				if (eregi("([a-zA-Z0-9_]+)[[:space:]]*$", $value, $captures)) $name .= $captures[1].":";
				
				/*
				// paremeter with no label (or first parameter)
				if (eregi("\([a-zA-Z0-9_]+\)[[:space:]]*([a-zA-Z0-9]+)[[:space:]]*$", $value, $captures)) {
					if ($name != "") {
						$name .= ":";
					} else {
						$name .= $captures[1].":";
					}
					
					continue;
				}
				
				// parameter with label
				if (eregi("\([a-zA-Z0-9_]+\)[[:space:]]*[a-zA-Z0-9_]+[[:space:]]+([a-zA-Z0-9]+)$", $value, $captures)) $name .= $captures[1].":";
				*/
			}
		} else {
			if (eregi("([a-zA-Z0-9_]+)[[:space:]]*(;)*$", $method, $captures)) $name = $captures[1];
		}
		
		return $name;
	}


	function MaybeRenameMethod(&$name, $isclassmethod) {
	  if ($isclassmethod) {
			foreach ($this->replace_class_methods as $org_name => $replace_name) {
				if ($name == $org_name) {
					$name = $replace_name;
					break;
				}
			}
	  } else {
			foreach ($this->replace_instance_methods as $org_name => $replace_name) {
				if ($name == $org_name) {
					$name = $replace_name;
					break;
				}
			}
	  }
	}


	// in case of a non-function pointer and non-pointer type,
	// split a C field expression (e.g. "long afield" or
	// "long int :32") into its type and field name. Necessary
	// to recognise name that "long int :32" means "a field
	// without a name whose type/alignment is "long int" and that is
	// bitpacked
	function ExtractCFieldSimpleTypeAndSingleName($param) {
//		print("Converting field $param\n");
	
		// remove all "const" occurrences to simplify matching
		$clean_param = str_replace_word("const", "", $param);

		$got_type = false;
		// assume not a pointer type
		if (preg_match("!^\s*(unsigned|signed)\b!", $clean_param, $signedunsigned)) {
			$got_type = true;
			$clean_param = preg_replace("!^\s*(unsigned|signed)\b!", "", $clean_param);
		}
		if (preg_match("!^\s*(char|short|int|long\s+long\s+int|long\s+int|long\s+long|long)\b!", $clean_param, $inttype)) {
			$got_type = true;
			$clean_param = preg_replace("!^\s*(char|short|int|long\s+long\s+int|long\s+int|long\s+long|long)\b!", "", $clean_param);
		}
		if ($got_type) {
			// remove bitfield and array, are encoded later
			$result["name"] = trim(preg_replace("!(?::.*)?(?:\[[^]]*\])?!","",$clean_param));
			$result["type"] = trim(preg_replace("!\s+!", " ", $signedunsigned[0] . " " . $inttype[0]));
		} else {
			// remove "struct"
			$clean_param = str_replace_word("struct", "", $clean_param);
			// has to be "type fieldname"
			preg_match("!^\s*(\w+)\b\s*(\w+)(?:\[[0-9]*\])?\s*!", $clean_param, $matches);
			$result["name"] = $matches[2]; // can be empty
			$result["type"] = $matches[1];
		}
//		print("field conversion result: ".$result["name"].": ".$result["type"]."\n");
		return $result;
	}
	
	function ExtractCParaNameAndType($param) {
//		print("Converting $param\n");
	
		// remove all "const" occurrences to simplify matching
		$clean_param = str_replace_word("const", "", $param);
		// remove all spaces between multiple pointer modifiers
		$clean_param = trim(preg_replace("![*]\s+[*]!", "**", $clean_param));
		// 1) varargs
		if ($clean_param == "...") {
			$result["name"] = "";
			$result["pointermods"] = "";
			$result["type"] = "...";
		// 2) type is a function pointer (does not yet match arrays of function
		//    pointers!)
		} elseif (preg_match($this->pregex_function_pointer_c_paratype, $clean_param, $matches)) {
			$result["name"] = $matches[3];
			// not correct; simply assume no "pointer to function pointer" types are
			// used
			$result["pointermods"] = "";
			$funcptrtype = $this->ParseFunctionDeclaration($matches[1], $matches[2], "", $matches[4], false, "");
			$result["type"] = $this->AddCallback($matches[3], $funcptrtype);
		// 3) any other pointer type (including arrays)
		} elseif (preg_match("!^([^*]*)([*]+)\s*(\w+(?:\[[0-9]*\])?)?\s*$!", $clean_param, $matches)){
			$result["name"] = $matches[3]; // can be empty
			$result["pointermods"] = $matches[2];
			$result["type"] = trim($matches[1]);
		// 4) basic C type (including arrays)
		} else {
			// definitely not a pointer type
			$result["pointermods"] = "";
			$got_type = false;
			if (preg_match("!^\s*(unsigned|signed)!", $clean_param, $signedunsigned)) {
				$got_type = true;
				$clean_param = preg_replace("!^\s*(unsigned|signed)!", "", $clean_param);
			}
			if (preg_match("!^\s*(char|short|int|long\s+long\s+int|long\s+int|long\s+long|long)!", $clean_param, $inttype)) {
				$got_type = true;
				$clean_param = preg_replace("!^\s*(char|short|int|long\s+long\s+int|long\s+int|long\s+long|long)!", "", $clean_param);
			}
			if ($got_type) {
				$result["name"] = trim($clean_param);
				$result["type"] = trim($signedunsigned[0] . " " . $inttype[0]);
			} else {
				// remove "struct"
				$clean_param = str_replace_word("struct", "", $clean_param);
				// has to be "type paraname", or just "type"
				preg_match("!^\s*(\w+)\b\s*(\w+(?:\[[0-9]*\])?)?\s*!", $clean_param, $matches);
				$result["name"] = $matches[2]; // can be empty
				$result["type"] = $matches[1];
			}
		}
//		print("param conversion result: ".$result["name"].": ".$result["pointermods"].$result["type"]."\n");
		return $result;
	}

	// Returns the parameter modifier string for a callback parameter type
	function GetCallbackParameterModifiers ($type, $name) {
		$modifiers = "";
		$type = trim($type, "*");
		
		// if the name starts with * it's a pointer
		// don't process framework classes since they're always pointers
		if (ereg("^[[:space:]]*\*", $name)) { 
			if (!in_array($type, $this->cocoa_classes)) $modifiers = "var ";
		} 
		
		// double ** are always var
		if (ereg("^[[:space:]]*\*\*", $name)) { 
			$modifiers = "var ";
		} 

		return $modifiers;
	}
						
	// Print string to output file
	function PrintOutput ($indent, $string) {
		for ($i=0; $i < $indent; $i++) { 
			$indent_string .= "  ";
		}
		
		if (($this->output) && (!$this->show)) fwrite($this->output, "$indent_string$string\n");
		
		if ($this->show) print("$indent_string$string\n");
	}
		
	// Returns a class hierarchy array 
	function GetClassHierarchy ($class, &$hierarchy) {
		if (!$hierarchy) $hierarchy = array();
		$hierarchy[] = $class["name"];
		
		if ($class["super_class"]) {
			$hierarchy[] = $this->GetClassHierarchy($class["super_class"], $hierarchy);
		}
		
		return $class["name"];
	}
	
	// Returns all protected keywords in a class hierarchy
	function GetProtectedKeywords ($in_class) {
		$this->GetClassHierarchy($in_class, $hierarchy);
		$keywords = array();
		
		foreach ($hierarchy as $class) {
			if ($this->dump["master"][$class]["protected_keywords"]) {
				foreach ($this->dump["master"][$class]["protected_keywords"] as $keyword) $keywords[] = strtolower($keyword);
			}
		}
		
		return $keywords;
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

	// Checks if a line has a property and returns the parts
	function LineHasProperty ($line, &$parts) {
		if (eregi($this->regex_objc_property_attributes, $line, $captures)) {
			$parts = $captures;
			//print_r($parts);
			return true;
		} elseif (eregi($this->regex_objc_property, $line, $captures)) {
			$parts = $captures;
			//print_r($parts);
			return true;
		}
	}
	
	// Returns header a category should be moved to
	function FindCategoryHeader ($category) {
		
		foreach ($this->dump as $name => $header) {
			if ((@array_key_exists($category, $header["classes"])) && ($category != "NSObject")) {
				return $name;
			}
		}
	}
	
	// Checks if $class (array) contains $method (array)
	function ClassContainsMethod ($class, $method) {
		if ($class["methods"]) {
			foreach ($class["methods"] as $key) {
				if ($key["name"] == $method["name"]) return true;
			}
		}
	}
	
	function GetAnonBitFieldName() {
		$name = "_anonbitfield_".$this->current_header["name_clean"].$this->current_header["anonbitfields"];
		$this->current_header["anonbitfields"]++;
		return $name;
	}
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	// create a variant normal record with a first anonymous field of type
	// first_bitpacked_type so that the compiler may correctly align it
	// the actual bitpacked record is embedded inside
	function BitPackedForceAlignment($first_bitpacked_type, $firstindent, $otherindents) {
		$result = $firstindent . "case byte of\n" . $otherindents ."0: (" . $this->GetAnonBitFieldName() . ": $first_bitpacked_type);\n" . $otherindents . "1: (data: bitpacked record";
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	// create a variant record with a first anonymous field of type
	// first_bitpacked_type so that the compiler may correctly align it
	// (it does not do that currently, but this at least provides a
	//  hint to the programmer)
	function BitPackedForceAlignment($first_bitpacked_type, $firstindent, $otherindents) {
		$result = $firstindent . "case byte of\n" . $otherindents ."0: (" . $this->GetAnonBitFieldName() . ": $first_bitpacked_type);\n" . $otherindents . "1: (";
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
		return $result;
	}
	
	function EncodePointerModifiers($type, $pointertype) {
		if ($pointertype[0] == "*") $type = $this->ReplacePointerType($type);
		if ($pointertype[1] == "*") {
			if ($type == "Pointer") {
				$type = "PPointer";
			} elseif ($type == "PChar") {
				$type = "PPChar";
			} elseif (in_array($type, $this->cocoa_classes)) {
				$type = $type.$this->class_pointer_suffix;
			} else {
				// to match MacOSAll types
				$type = $type."Ptr";
			}
		}
		return $type;
	}
			
	/**
	 * DOCSETS UTILITIES
	 */
	
	function FindDocumentationForMethod ($class, $name) {
		if ($this->docset) {
			$doc = $this->docset[$class][$name];
			
			if ($doc) return "{ $doc }";
		}
	}
	
	function FindDocumentationForType ($name) {
		if ($this->docset) {
			foreach ($this->docset as $class) {
				foreach ($class as $type => $text) {
					if ($type == $name) return "{ $text }";
				}
			}
		}
	}

	/**
	 * ADDING LANGUAGE STRUCTURE UTILITIES
	 */
	
	// Adds a method structure to a class and performs checks for overloaded methods
	function AddMethodToClass (&$method, &$class) {
		
		// ignore methods
		if (in_array($method["name"], $this->ignore_methods)) return false;
		
		// add comment to the method
		$method["comment"] = $this->InsertCurrentComment();

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
		
		//$this->AppendCurrentMacro($header["types"]["typedef"]);
		$this->AppendCurrentComment($header["types"]["typedef"]);
		if ($this->comment_terminated) $header["types"]["typedef"][] = $this->InsertCurrentComment();
		
		$header["types"]["typedef"][] = $typedef;
	}
		
	// adds a function callback type with name $name, and returns
	// the name of the callback type
	function AddCallback($name, $function_pointer) {
		if ($this->current_header) {
			$type = $this->current_header["name_clean"].ucwords($name);
	
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
			return $type;
		}
		return "invalid_callback_type_because_no_current_header";
	}
	
	/**
	 * OBJC FORMATTING UTILITIES
	 */
	
	// Performs additional formatting on Objective-c type i.e. (out NSObject **)
	function FormatObjcType ($type, &$modifiers) {
		$modifiers = "";
		
		// toss out all const identifiers
		$type = istr_replace_word("const", "", $type);
		
		// replace inout paramaters
		$type = istr_replace_word("inout", "", $type);
		$type = istr_replace_word("out", "", $type);
		
		// Translate protocol which type conforms to (id <NSURLHandleClient>)
		$type = preg_replace("!id\s*<([^,>]*)>!", "$1Protocol", $type);
		// Remove other protocol types
		$type = trim(eregi_replace("<[^>]*>", "", $type));
		
		// Replace types before cleaning
		$type = $this->ReplaceObjcType($type);
		
		// Remove array brackets (NSUInteger[])p
		$type = trim(eregi_replace("\[[0-9]*\]", "", $type));
		
		// var params to non-object types (NSRange *)
		if (ereg("([a-zA-Z0-9_]+)[[:space:]]*[*]+$", $type, $captures)) { 
			if (!in_array($captures[1], $this->cocoa_classes)) {
				$type = $this->ReplaceReferenceParameterType(trim($type,"* "));
				//$modifiers = "var ";
			}
		} 
		
		// Handle NS*** pointers (NSError **)
		if (ereg("(NS[a-zA-Z0-9_]+)[[:space:]]*\*\*$", $type, $captures)) { 
			if (in_array($captures[1], $this->cocoa_classes)) {
				$type = trim($type,"* ")."$this->class_pointer_suffix";
			}
		}
		
		// clean the type
		$type = trim($type, "* 	");
		
		//print("$type\n");
		return $type;
	}
	
	// Performs additional formatting on Objective-c parameter types		
	function FormatObjcParams ($string, &$variable_arguments) {
		$params = explode(":", $string);
		$string = "";
		$variable_arguments = false;
		// print_r($params);
		
		if (count($params) > 0) {
			foreach ($params as $value) {
				
				// parameter is varargs
				if (eregi("(.*),[[:space:]]*\.\.\.", $value)) {
					$string .= ":(id)$this->varargs_param_name";
					$variable_arguments = true;
					//print("$value\n");
					continue;
				}
				
				if (preg_match("!^[^(]*$!",$value)) {
					// parameter without type -> id
					$value = "(id)".$value;
				}

				// normal parameter
				if (ereg("\((.*)\)", $value, $captures)) {
					$new_value = $this->ReplaceObjcType($captures[1]);
					
					if ($new_value != $captures[1]) $value = ereg_replace("\((.*)\)", "($new_value)", $value);
					
					$string .= ":$value";
					continue;
				}
				
			}
		}
		$string = ltrim($string, ":");
		return $string;
	}

	/**
	 * SYNTAX MAKING UTILITIES
	 */	
		
	// Makes a paramater list string with options to modify
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
	
	// Makes a list of paramameter variables with NS*** class types cast to "id" or the original class
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
	
	// Makes a struct field into an inline array (or returns field un-changed)
	function MakeFieldInlineArray ($io_field, $line, $name, $type) {

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		if (eregi("\[([^]]+)\];", $line, $array_size)) {
			if ($array_size[1] == "")
				$io_field = "$name: array[0..0] of $type; { dynamically expanding, 0 elements in C }";
			else if ($array_size[1] == "0")
				$io_field = "$name: record end; { array of 0 elements in C, does not allocate space }";
			else
				// array_size[1] may be a symbolic constant rather than a number, so don't calculate in php
				$io_field = "$name: array[0..($array_size[1])-1] of $type;";
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		if (eregi("\[([0-9]+)\];", $line, $array_size)) {
			$length = (int)$array_size[1] - 1;
			if ($length > 0) {
				$io_field = "    $name: array[0..$length] of $type;";
			}
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
		}
		
		return $io_field;
	}

	// Makes a type bitpacked (or returns type un-changed)
	function MakeFieldBitPacked ($ioType, $field, &$bitpacked) {
		$bitpacked = false;
		
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		if (preg_match("!:([0-9]+)\s*(?:__attribute__\(\([^)]*\)\))?\s*;\s*$!", $field, $bitpack)) {
=======
		if (preg_match("!:([0-9]+)\s*(?:__attribute__\(\([^)]*\)\))?\s*;$!", $field, $bitpack)) {
>>>>>>> graemeg/cpstrnew
=======
		if (preg_match("!:([0-9]+)\s*(?:__attribute__\(\([^)]*\)\))?\s*;$!", $field, $bitpack)) {
>>>>>>> graemeg/cpstrnew
=======
		if (preg_match("!:([0-9]+)\s*(?:__attribute__\(\([^)]*\)\))?\s*;$!", $field, $bitpack)) {
>>>>>>> graemeg/cpstrnew
=======
		if (preg_match("!:([0-9]+)\s*(?:__attribute__\(\([^)]*\)\))?\s*;$!", $field, $bitpack)) {
>>>>>>> origin/cpstrnew
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
	
	/**
	 * REPLACEMENT UTILITIES
	 */
	
	// Replace type with pointer equivalent
	function ReplacePointerType ($type) {
		$found = false;
		
		// all classes are implicit pointer types
		if (in_array($type, $this->cocoa_classes)) return $type;
		
		// function pointer types are sometimes also implicit
		// pointers
		if (in_array($type, $this->implicit_function_pointer_types)) return $type;

		// PPointer = ^Pointer
		if ($type == "Pointer") return "PPointer";
		if ($type == "PChar") return "PPChar";
		
		// use preferred pointer type
		foreach ($this->pointer_types as $objc_type => $replace_type) {
			if ($objc_type == $type) {
				$found = true;
				$type = $replace_type;
				break;
			}
		}
		
		//$type = "Pointer {".$type."}";
		// use generic pointer type
		if (!$found)
			$type = $type."Ptr";
		
		return $type;
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
	
	// Replace remote message passing modifiers
	function ReplaceRemoteMessagingModifiers ($string, &$io_modifier) {
		$io_hint = false;

		foreach ($this->remote_messaging_modifiers as $modifier) {
			$out_string = preg_replace("!\b".$modifier."\b!", "", $string);
			if ($out_string != $string) {
				$io_modifier = $modifier;
				$string = $out_string;
			}
		}
			
		return trim($string);
	}
	
	// Replace type of reference parameter with pointer
	function ReplaceReferenceParameterType ($type) {
		$type = $this->ReplaceObjcType($type);
		return $this->ReplacePointerType($type);
	}
	
	// Replace a framework class with generic id and comment hint
	function ReplaceFrameworkClassWithID ($string) {
		foreach ($this->cocoa_classes as $class) {
			$string = istr_replace_word($class, $this->objc_id, $string);
		}
		return $string;
	}
				

	/**
	 * MACRO UTILITIES
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

	// Appends the current macro to a recipient and resets it's reference
	function AppendCurrentMacro (&$recipient) {
		if ($this->in_macro_block) {
			$this->in_macro_block = false;
			
			$recipient[] = $this->macro_block;
			//if (is_array($recipient)) $recipient[] = $this->macro_block;
			//if (is_string($recipient)) $recipient .= $this->macro_block;
		}
	}
	
	// Returns the current macro and resets it's reference
	function InsertCurrentMacro () {
		if ($this->in_macro_block) {
			$this->in_macro_block = false;
			return $this->macro_block;
		} else {
			return null;
		}
	}

	// Build the current macro block
	function BuildMacroBlocks ($line) {
		
		// only insert if we are in a block already.
		// NOTE: this does not handle nesting!
		if ($this->in_macro_block) {
			
			// macro else statment
			if (eregi("#else", $line)) {
				$this->macro_block = "{\$else}";
			}

			// macro endif statment
			if (eregi("#endif", $line)) {
				$this->in_macro_block = false;
				$this->macro_block = "{\$endif}";
			}
		}
		
		foreach ($this->macro_blocks as $key => $value) {
			
			if (eregi($key, $line, $captures)) {
				$this->in_macro_block = true;
				
				// replace the c-macro with a Pascal version
				if ($value == "*") {
					$captures[0] = trim($captures[0], "#");
					$this->macro_block = "{\$".$captures[0]."}";
				} else {
					$this->macro_block = "{".$value."}";
				}
			}
		}
	}

	// Remove OS X versions macros from a line
	// NOTE: These should be re-inlcuded in Pascal
	function RemoveVersionMacros ($line, &$deprecatedmods) {
		if (preg_match($this->pregex_deprecated_macro,$line)) {
			$deprecatedmods = DeprecatedMacroToDirective($line);
		} else {
			$deprecatedmods = "";
		}
		foreach ($this->version_macros as $macro) {
			$line = eregi_replace($macro, "", $line);
		}
		return $line;
	}
				
	/**
	 * CONVERTING UTILITIES
	 */
	
	// Converts an Objective-c parameter string to Pascal		
	function ConvertObjcParamsToPascal ($string, $protected_keywords) {
		$params = explode(":", $string);
		$list = array();
		$list["pairs"] = array();
		$param_list = array();
		//print_r($params);
		//print("$string\n");

		if (count($params) > 0) {

			foreach ($params as $value) {
				$value = trim($value);
				$valid = false;
				$modifiers = "";

				$value = $this->ReplaceRemoteMessagingModifiers($value, $null);

				// function pointer (callback)
				if (preg_match($this->pregex_function_pointer_objc_paratype, $value, $captures)) {
					$name = $captures[5];

					$function_pointer = $this->ParseFunctionDeclaration($captures[1], $captures[2], "", $captures[4], false, "");
					$type = $this->AddCallback($name, $function_pointer);
					$valid = true;
				} elseif ((eregi("\(([a-zA-Z_]+).*\)([a-zA-Z_]+).*\.\.\.", $value, $captures)) || (eregi("(.*),[[:space:]]*\.\.\.", $value, $captures))) { // variable arguments
					$name = $captures[2];
					$type = $captures[1];
					$valid = true;
				} elseif (eregi("\((.*)\)[[:space:]]*([a-zA-Z_]+)", $value, $captures)) { // standard parameter

					$captures[1] = trim($captures[1]);
					$type = $this->FormatObjcType($captures[1], $modifiers);
					$name = $captures[2];

					$valid = true;
				}

				if ($valid) {

					// protect reserved keywords
					if ($this->IsKeywordReserved($name)) $name .= "_";

					if (!in_array($type, $this->reserved_types)) {
						if ($this->IsKeywordReserved($type)) $type .= "_";
					}

					// can't have "boolean: boolean;" parameters
					if (preg_match("!\b$name\b!i",$type)) $name .= "_";

//					print("para name: \"$name\"\n");
//					print_r("protected: \"$protected_keywords\"\n");
					while (@in_array(strtolower($name), $protected_keywords)) $name .= "_";

					// case-insensitive in_array:
//					if( preg_match("/\b$name\b/i", join(" ", array_values($protected_keywords))) ) $name .= "_";
//					if (@in_array($type, $protected_keywords)) $type .= "_";

					// replace objc types
					$type = $this->ReplaceObjcType($type);

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
	
	// Converts an Objective-C method to Pascal format 
	function ConvertObjcMethodToPascal ($class, $source, $parts, $protected_keywords, $has_params, $deprecatedmods) {
		
		//print("$source\n");
		//print_r($parts);
		
		// replace "hinted" params comment with hinted type
		if ($this->replace_hinted_params) {

			// param string
			if (eregi("(/\*[[:space:]]*(.*)[[:space:]]*\*/)", $parts[4], $captures)) {
				// ??? change the parameter to the hinted type
				//$parts[4] = eregi_replace("(/\*.*\*/)", $captures[2], $parts[4]);
				//$parts[4] = trim($parts[4], " ");
			}

			// return type
			if (eregi("(/\*[[:space:]]*(.*)[[:space:]]*\*/)", $parts[2], $captures)) $parts[2] = $this->ReplaceRemoteMessagingModifiers($captures[2], $null);

			//print_r($parts);

		} else { // remove comments from params and return type
			$parts[4] = eregi_replace("(/\*.*\*/)", "", $parts[4]);
			$parts[4] = trim($parts[4]);

			$parts[2] = eregi_replace("(/\*.*\*/)", "", $parts[2]);
			$parts[2] = $this->ReplaceRemoteMessagingModifiers($parts[2], $null);
		}
	
		$return_type_clean = $parts[2];
		$return_type_pointers = preg_replace("![^*]+!e", "", $return_type_clean);
		$return_type_clean = trim($return_type_clean,"* 	");

		// perform preformatting before attempting to protect keywords
		$parts[2] = $this->FormatObjcType($parts[2], $modifiers);
		$parts[4] = $this->FormatObjcParams($parts[4], $variable_arguments);
		//print($parts[4]."\n");
		
		if ($has_params) {
			$name = $this->ConvertObjcMethodName($source);

			// merge default protected keywords for the class/category
			if ($this->default_protected["*"]) $protected_keywords = array_merge($this->default_protected["*"], $protected_keywords);
			if ($this->default_protected[$class]) $protected_keywords = array_merge($this->default_protected[$class], $protected_keywords);
			
			
			$param_array = $this->ConvertObjcParamsToPascal($parts[4], $protected_keywords);
			$params = "(".$param_array["string"].")";
			$params_with_modifiers = "(".$param_array["string_with_modifiers"].")";
			
		} else {
			$params = "";
			$params_with_modifiers = "";
			// no parameters -> definitely no underscore normally, but there are some
			// conflicts...
			$name = $parts[3];
			// clean it up
			if ($this->trailing_underscore) {
				if (in_array($source, $this->trailing_underscore_methods)) $name = $name . "_";	
			}
			$param_array = null;
			$variable_arguments = false;
		}
		
		// rename method if required
		$this->MaybeRenameMethod($name,$parts[1]=="+");

		// protect method name from keywords
		if ($this->IsKeywordReserved($name)) $name .= "_";
		
		// replace objc type
		$return_type = $this->ConvertReturnType($return_type_clean,$return_type_pointers);

		$virtual = "";
		$class_prefix = "";

		// determine the type based on return value
		if (ereg($this->regex_procedure_type, $return_type_clean.$return_type_pointers)) {
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

	  // detect blocks (not yet supported)
		$has_blocks = strpos("$return_type $params_with_modifiers","^");

		// make method templates
		if ($kind != "function") {
			if ($variable_arguments) $modifier .= " varargs;";
		
			$method = "$class_prefix$kind $name$params_with_modifiers;$modifier$virtual";
			$method_template = "[KIND] [PREFIX]$name"."[PARAMS];$modifier";
		} else {
			if ($variable_arguments) $return_type = "$return_type; varargs";

			$method = $class_prefix."function $name$params_with_modifiers: $return_type;$modifier$virtual";
			$method_template = "[KIND] [PREFIX]$name"."[PARAMS]: [RETURN];$modifier";
		}

		$method_template_procedure = "procedure [PREFIX]$name"."[PARAMS];$modifier";
		$method_template_function = "function [PREFIX]$name"."[PARAMS]: [RETURN];$modifier";

		// build structure
		$struct["def"] = $method;
		$struct["template"] = $method_template;
		$struct["template_function"] = $method_template_function;
		$struct["template_procedure"] = $method_template_procedure;
		$struct["objc_method"] = $this->ExtractObjcMethodName($source);
		$struct["class_prefix"] = $class_prefix;
		if ($deprecatedmods != "") $struct["deprecated"] = $deprecatedmods.";";
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
		if ($has_blocks === false) $struct["blocks_disable_comment"] = "";
		else $struct["blocks_disable_comment"] = "// ";

		if ($struct["param_array"] != null) $struct["has_params"] = true;

		// FPC bug work around
		if (strlen($name) > $this->maximum_method_length) {
			$struct["can_override"] = false;
			print(" # WARNING: method $name can't override because the name is too long\n");
			$this->warning_count ++;
		}

		return $struct;
	}
	
	// Converts a C parameter string to Pascal
	function ConvertCParamsPascal ($string) {

//		print("params: $string\n");
		if ((trim($string) == "void")  || (trim($string) == "")) return "";

		$params = explode(",", $string);
		$count = 0;
		$param_string = "";

		foreach ($params as $param) {

			$name_type = $this->ExtractCParaNameAndType($param);
			$type = $name_type["type"];
			$name = $name_type["name"];
			$pointertype = $name_type["pointermods"];

			// create name if none is specified
			if ($name == "") {
				$count ++;
				$name = "param$count";
			}

			// remove const keywords
			$type = str_replace_word("const", "", $type);
			$type = $this->ReplaceObjcType($type);

			// Remove array brackets (NSUInteger[])p
			if (eregi("\[[0-9]*\]", $name)) {
				$orgtype = $this->EncodePointerModifiers($type,$pointertype);
				$pointertype .= "*";
				$type = $this->EncodePointerModifiers($type,$pointertype)." {array of $orgtype}";
				$name = eregi_replace("\[[0-9]*\]", "", $name);
			} else {
				$type = $this->EncodePointerModifiers($type,$pointertype);
			}
			$modifier = "";

			if ($this->IsKeywordReserved($name)) $name .= "_";

			// multiple parameters
			if ($type == "...") {
				$param_string .= "varargs: array of const";
				break;
			}

			$param_string .= "$modifier$name: $type; ";
		}

		$param_string = trim($param_string, "; ");
		//print("$param_string\n");
		return $param_string;
	}
	
	// Converts an Objective-c method name to Pascal
	function ConvertObjcMethodName ($method) {
		$params = explode(":", $method);
		$name = "";
		$count = 0;
		
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
	
	// Convert a method return type to Pascal
	function ConvertReturnType ($type, $pointertype) {
		$type = $this->ReplaceGarbageCollectorHints($type, $null);
		$type = $this->ReplaceRemoteMessagingModifiers($type, $null);

		// format the return type to make sure it's clean
		$type = $this->FormatObjcType($type, $null_modifier);
		// translate type to Pascal
		$type = $this->ReplaceObjcType($type);
		// incorportate pointer modifiers
		$type = $this->EncodePointerModifiers($type,$pointertype);

		return $type;
	}
	
	/**
	 * USER PATCHES
	 */
	function InsertPatches ($header) {
		$path = "$this->root/patches/".$header["name_clean"].".patch";
		if ($handle = @fopen($path, "r")) {
			$text = file_get_contents($path);
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

	/**
	 * COMMENTS
	 */
	
	// Trim a comment string
	function TrimComment ($comment, $preserve_line_endings) {
		
		// trim line breaks
		if (!$preserve_line_endings) $comment = trim($comment, "\n");
		
		// remove remains of C comments
		$comment = eregi_replace("^[!*/ 	]+", "", $comment);
		$comment = eregi_replace("[!*/ 	]+$", "", $comment);
		
		// remove all Pascal comment characters to prevent nesting
		$comment = str_replace(array("{", "}"), "", $comment);
		
		//print("$comment\n");	
		return $comment;

	}
	
	// Builds comments from $line spaning multiple lines or at the end of lines
	// Returns a terminated comment string or a fragment in $fragment for futher processing
	function BuildComment ($line, $file) {
		
		// set the current comment header being parsed
		$this->comment_header = $file;
		
		// comment parsing is off, bail!
		if (!$this->parse_comments) return;
		
		// single-line comment at start of line
		if (eregi("^[[:space:]]*//(.*)", $line, $captures)) {
			//print("$captures[1]\n");
			// ignore comment
			foreach ($this->ignore_comments as $pattern) {
				if (eregi($pattern, $captures[1])) return;
			}
			
			$this->comment_terminated = $this->TrimComment($captures[1], false);
			return;
		} elseif (eregi("[[:space:]]+//(.*)", $line, $captures)) { // single-line comments at end of line
			//print("$captures[1]\n");
			// ignore comment
			foreach ($this->ignore_comments as $pattern) {
				if (eregi($pattern, $captures[1])) return;
			}
			
			$this->comment_eol = $captures[1];
			return;
		}
		
		// multi-line comments terminated
		if (eregi("/\*(.*)\*/", $line, $captures)) {
			//print("$captures[1]\n");
			
			// ignore comment
			foreach ($this->ignore_comments as $pattern) {
				if (eregi($pattern, $captures[1])) return;
			}
			
			$this->comment_terminated = $this->TrimComment($captures[1], false);
			return;
		}
		
		// terminate comment fragment 
		if ($this->comment_fragment_open) {
			if (eregi("(.*)\*/", $line, $captures)) {
				
				// append fragment
				$comment = $this->TrimComment($captures[1], false);
				if ($comment) $this->comment_fragment .= $comment;
				
				// closed comment block
				if (!$captures[1]) {
					$this->comment_block_closed = true;
				}
				
				// set terminated comment with fragment
				$this->comment_terminated = $this->comment_fragment;
				
				// add extra line break for comment blocks
				if (($this->comment_block_open) && ($this->comment_block_closed)) {
					// ??? the printing will cut this out if we add line endings!
					$this->comment_terminated = "$this->comment_fragment";
				}
				
				$this->comment_fragment = null;
				$this->comment_fragment_open = false;
				
				$this->comment_block_open = false;
				$this->comment_block_closed = false;
				
				return;
			} else {
				// build the fragment
				$comment = $this->TrimComment($line, true);

				// ignore comment and stop building fragment
				foreach ($this->ignore_comments as $pattern) {
					if (eregi($pattern, $comment)) {
						$this->comment_fragment = null;
						$this->comment_fragment_open = false;

						$this->comment_block_open = false;
						$this->comment_block_closed = false;
						
						return;
					}
				}
				
				if (($this->comment_fragment_previous != $line) && ($comment)) $this->comment_fragment .= $comment."\n";
				
				$this->comment_fragment_previous = $line;
				return;
			}
		}
		
		// start comment fragment
		if (eregi("/\*(.*)", $line, $captures)) {
			
			$this->comment_terminated = null;

			// ignore comment
			foreach ($this->ignore_comments as $pattern) {
				if (eregi($pattern, $captures[1])) return;
			}
			
			$this->comment_fragment_open = true;
			$this->comment_fragment = "";
			$this->comment_block_open = true;
			$this->comment_block_closed = false;
			
			// prevent against empty comments
			if ((!$captures[1]) || ($captures[1] == "\n")) {
				$this->comment_block_open = true;
				return;
			}
			
			// append the line if valid
			$comment = $this->TrimComment($captures[1], false);
			
			if ($comment) $this->comment_fragment .= $comment;
			
			return;
		}
	}
	
	// Resets current comment references
	function ResetComment () {
		$this->comment_fragment = null;
		$this->comment_eol = null;
		$this->comment_terminated = null;
		$this->comment_block_open = false;
		$this->comment_block_closed = false;
	}
	
	// Checks/sets if a comment is a duplicate in the file
	function CheckDuplicateComment ($comment) {
		if ((@!in_array($comment, $this->dump[$this->comment_header]["comments"])) && ($comment)) {
			$this->dump[$this->comment_header]["comments"][] = $comment;
			return true;
		} else {
			return false;
		}
		
	}
	
	// Appends the eol comment to the output and clears the reference
	function AppendEOLComment () {
		if ($this->comment_eol) {
			$comment = "  // ".$this->TrimComment($this->comment_eol, false);
			$this->comment_eol = "";
			return $comment;
		} else {
			return null;
		}
	}
	
	// Inserts the recently terminated comment to the output on a new line and clears the reference
	function InsertCurrentComment () {
		if (($this->comment_terminated) && ($this->CheckDuplicateComment($this->comment_terminated))) {
			
			if ($this->comment_terminated != $this->comment_previous) $comment = "$this->comment_break{".$this->comment_padding_left.$this->TrimComment($this->comment_terminated, false).$this->comment_padding_right."}";
			
			$this->comment_previous = $this->comment_terminated;
			$this->comment_terminated = "";
			
			return $comment;
		} else {
			return null;
		}
	}
	
	// Appends the recently terminated comment to a recipient and clears the reference
	function AppendCurrentComment (&$recipient) {
		if (($this->comment_terminated) && ($this->CheckDuplicateComment($this->comment_terminated)) && (gettype($recipient) == "array")) {
			
			if ($this->comment_terminated != $this->comment_previous) $comment = "$this->comment_break{".$this->comment_padding_left.$this->TrimComment($this->comment_terminated, false).$this->comment_padding_right."}";
			
			$recipient[] = $comment;
			//if (is_array($recipient)) $recipient[] = $comment;
			//if (is_string($recipient)) $recipient .= $comment;
			
			$this->comment_previous = $this->comment_terminated;
			$this->comment_terminated = "";
		}
	}
	
	// Removes all comments from a line
	function RemoveComments ($line) {
		// remove single-line comments
		$line = eregi_replace("[[:space:]]+//(.*)", "", $line);
		
		// remove multi-line comments /* ... */
		$line = eregi_replace("/\*.*\*/", "", $line);
		
		return $line;
	}
	
	/**
	 * PRINTING METHODS
	 */
	
	function PrintClass ($class) {

		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ ".$class["name"]." }");
		
		if ($class["comment"]) $this->PrintOutput(0, $class["comment"]);
		//print_r($class["methods"]);
		
		// print super class or protocol which the class conforms to
		if ($class["adopts"]) {
			if ($class["super"]) {
				$this->PrintOutput(1, $class["name"]." = objcclass external (".$class["super"].", ".$class["adopts"].")");
			}	else {
				$this->PrintOutput(1, $class["name"]." = objcclass external (".$class["adopts"].")");
			}
		} elseif ($class["super"]) {
			$this->PrintOutput(1, $class["name"]." = objcclass external (".$class["super"].")");
		} else {
			$this->PrintOutput(1, $class["name"]." = objcclass external");
		}

		// print instance variables
		if ($class["ivars"]) {
			$this->PrintOutput(1, "private");
			foreach ($class["ivars"] as $ivar) {
				$this->PrintOutput(2, $ivar);
			}
		}

		// print methods (public)
		$this->PrintOutput(2, "");
		$this->PrintOutput(1, "public");
		
		// print class-level methods
		if ($class["methods"]) {
			foreach ($class["methods"] as $method) {
				if ($method["comment"]) $this->PrintOutput(2, $method["comment"]);
				if ($method["documentation"]) $this->PrintOutput(2, $method["documentation"]);
				$this->PrintOutput(2, $method["blocks_disable_comment"].$method["def"]." message '".$method["objc_method"]."';".$method["deprecated"]);
			}
		}
		
		// print adopted protocol methods
		if (count($class["protocols"]) > 0) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(2, "{ Adopted Protocols }");
			//print_r($this->dump["protocols"]);

			foreach ($class["protocols"] as $name) {
				if ($this->dump["protocols"][$name]) {
					foreach ($this->dump["protocols"][$name] as $method) {
						if (!$this->ClassContainsMethod($class, $method)) $this->PrintOutput(2, $method["blocks_disable_comment"].$method["def"]);
					}
				}
			}
		}

		$this->PrintOutput(1, "end;");
	}
	
	function PrintCategory ($class, $category) {

		// declare real category if external
		if ($category["external"]) {
			$new_name = " name '".$category["external_name"]."'";
		}
		
		$category_name = $category["name"].$this->category_suffix;
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{ $category_name }");
		if ($category["comment"]) $this->PrintOutput(0, $category["comment"]);
		
		// print super class or protocol which the class conforms to
		$this->PrintOutput(1, "$category_name = objccategory external$new_name (".$category["super"].")");
		
		// print methods
		if ($category["methods"]) {
			foreach ($category["methods"] as $method) {
				if ($method["comment"]) $this->PrintOutput(2, $method["comment"]);
				$this->PrintOutput(2, $method["blocks_disable_comment"].$method["def"]." message '".$method["objc_method"]."';".$method["deprecated"]);
			}
		} 

		$this->PrintOutput(1, "end;");
	}
						
	function PrintHeader ($header) {
		global $version;
		//print_r($header);
		//print_r($this->dump["categories"]);
		
		// open the output file if we not printing to terminal
		if (!$this->show) {
			if ($this->merge_headers) {
				$this->output = fopen($header["path_merge"], "w+");
			} else {
				$this->output = fopen($header["path"], "w+");
			}
		}

		$this->PrintOutput(0, "{ Parsed from ".ucfirst($header["framework"]).".framework ".$header["name"]." }");

		//$date = @date("D M j G:i:s T Y");
		//$this->PrintOutput(0, "{ Version: $version - $date }");
		//$this->PrintOutput(0, "");

		$macro = strtoupper(substr($header["name"], 0, (strripos($header["name"], "."))));
		
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "{\$ifdef TYPES}");
		$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_T}");
		$this->PrintOutput(0, "{\$define $macro"."_PAS_T}");
		$this->PrintTypes($header, false);
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

		// print class/protocol forward declarations
		if (($header["classes"]) || ($header["protocols"])) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$ifdef FORWARD}");

			if ($header["protocols"]) {
				foreach ($header["protocols"] as $protocol) $this->PrintOutput(1, $protocol["name"]."$this->protocol_suffix = objcprotocol;");
			}
			
			if ($header["classes"]) {
				foreach ($header["classes"] as $class) {
					if ($class["name"]) {
						$this->PrintOutput(1, $class["name"]." = objcclass;");
						$this->PrintOutput(1, $class["name"].$this->class_pointer_suffix." = ^".$class["name"].";");
						// for consistency also offer Ptr-name variant
						$this->PrintOutput(1, $class["name"]."Ptr = ".$class["name"].$this->class_pointer_suffix.";");
					}
				}
			}
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
		}

		// print classes
		if ($header["classes"]) {
			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$ifdef CLASSES}");
			$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_C}");
			$this->PrintOutput(0, "{\$define $macro"."_PAS_C}");

			foreach ($header["classes"] as $class) {
				if ($class["name"]) $this->PrintClass($class);
			}

			if (count($header["categories"]) > 0) {
				foreach ($header["categories"] as $category) {
					$this->PrintCategory($class, $category);
				}
			}

			$this->PrintOutput(0, "");
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}

		// print protocols
		if ($header["protocols"]) {
			$this->PrintOutput(0, "{\$ifdef PROTOCOLS}");
			$this->PrintOutput(0, "{\$ifndef $macro"."_PAS_P}");
			$this->PrintOutput(0, "{\$define $macro"."_PAS_P}");

			foreach ($header["protocols"] as $protocol) {
				$this->PrintOutput(1, "");
				$this->PrintOutput(0, "{ ".$protocol["name"]." Protocol }");
				if ($protocol["comment"]) $this->PrintOutput(0, $protocol["comment"]);
				$this->PrintOutput(1, $protocol["name"]."$this->protocol_suffix = objcprotocol external name '".$protocol["name"]."'");

				// print methods
				if ($protocol["methods"]) {
				  $section="";
					foreach ($protocol["methods"] as $name => $method) {
						// print the required/optional section
						if ($method["section"] != $section) {
							$section = $method["section"];
							$this->PrintOutput(1, $section);
						}
						if ($method["comment"]) $this->PrintOutput(2, $method["comment"]);
						$this->PrintOutput(2, $method["blocks_disable_comment"].$method["def"]." message '".$method["objc_method"]."';".$method["deprecated"]);
					}
				}
				
				$this->PrintOutput(1, "end;");		
			}
			
			$this->PrintOutput(0, "{\$endif}");
			$this->PrintOutput(0, "{\$endif}");
		}
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
			$section_printed = false;

			// External defines
			if ($key == "defines") {
				
				foreach ($type_array as $type) {
					if (!$section_printed) {
						$this->PrintOutput(0, "");
						$this->PrintOutput(0, "{ Defines }");
						$this->PrintOutput(0, "const");
						$section_printed = true;
					}
					$this->PrintOutput(1, $type);
				}
			}
							
			// Enumerations
			if ($key == "enums") {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "{ Constants }");
				foreach ($type_array as $block) {
					$section_printed = false;
					
					foreach ($block as $type) {
						if (!$section_printed) {
							$this->PrintOutput(0, "");
							$this->PrintOutput(0, "const");
							$section_printed = true;
						}

						$this->PrintOutput(1, $type);
					}
				}
			}
			
			// Typedefs		
			if (($key == "typedef") || ($key == "named_enums")) {
				foreach ($type_array as $type) {
					if (!$section_printed) {
						$this->PrintOutput(0, "");
						$this->PrintOutput(0, "{ Types }");
						$this->PrintOutput(0, "type");
						$section_printed = true;
					}
					
					$this->PrintOutput(1, $type);
				}
			}

			// CallBacks
			if ($key == "callbacks") {
				foreach ($type_array as $name => $type) {
					if (!$section_printed) {
						$this->PrintOutput(0, "");
						$this->PrintOutput(0, "{ Callbacks }");
						$this->PrintOutput(0, "type");
						$section_printed = true;
					}
					
					$this->PrintOutput(1, "$name = $type");
				}
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
		$this->PrintOutput(1, "ctypes, objc, MacOSAll, AnonClassDefinitions;");
		
		if ($header["classes"]) {
			foreach ($header["classes"] as $class) {
				$this->PrintOutput(0, "");
				$this->PrintOutput(0, "type");
				$this->PrintOutput(1, $class["name"]."Ref = ".$this->objc_id.";");
				$this->PrintOutput(1, $class["name"]."Ptr = Pointer;");
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
							$this->PrintOutput(2, $method["blocks_disable_comment"].$method["def"]);
						}
					}

					// print category-level methods
					if (count($class["categories"]) > 0) {
						foreach ($class["categories"] as $name => $category) {
							$this->PrintOutput(0, "");
							$this->PrintOutput(2, "{ Category: $name }");

							if ($category["methods"]) {
								foreach ($category["methods"] as $method) {
									$this->PrintOutput(2, $method["blocks_disable_comment"].$method["def"]);
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
		
	// Merges two headers by difference
	function MergeHeader ($header) {
		/*
		diff QLPreviewPanel_ORIGINAL.inc QLPreviewPanel_UPDATED.inc > diff.patch
		patch QLPreviewPanel_ORIGINAL.inc -i diff.patch -o output.txt
		
		patch QLPreviewPanel.inc -i header.patch -o QLPreviewPanel.inc.merged
		
		also add -section="types,classes" which only prints into the sections and copies text from
		the previous version if not specified
		
		*/
		
		// set the paths to use
		$path = $header["path"];
		$patch = "$this->root$this->out/$this->framework/header.patch";
		$merged = $header["path_merge"];
		
		// create a patch using diff
		exec("/usr/bin/diff \"$path\" \"$merged\" > \"$patch\"");
		
		// parse the diff file by mode
		$lines = file($patch);
		$content = "";
		$section_lines = null;
		$section_part = null;
		$count = 0;
		$section_id = 0;
		//print_r($lines);
		
		if ($lines) {
			foreach ($lines as $line) {
				$count++;
				
				// remove line endings to aid regex
				$line = trim($line, "\n");
				
				// parse section
				if (($section_lines) && (!eregi("^[0-9]+", $line))) {
					
					// append line to section
					$section_lines[] = "$line\n";
					
					// the section id is adding
					if (eregi("^>", $line)) $section_part[$section_id] = true;
					if (eregi("^<", $line)) $section_part[$section_id] = 0;

					// section is changing type
					if ($line == "---") {
						$section_id++;
						continue;
					}
				}
				
				// line is a new section or eof
				if ((eregi("^[0-9]+", $line)) || ($count == count($lines))) {
					
					// append the section to the content
					// we only accept sections where the first part (before ---) contains additions ">"
					//print_r($section_part);
					if (($section_part[0]) && (!$section_part[1])) $content .= implode($section_lines);

					// init the new section
					$section_lines = array();
					$section_lines[] = "$line\n";
					$section_id = 0;
					$section_part = array(null, null);
					
					continue;
				}
				
			}
			
			//print($content);
			
			// write the parsed patch back to the file
			if ($content) {
				file_put_contents($patch, $content);
			} else {
				$content = null;
			}
		}
		
		// patch the header to the merged file
		if ($content) {
			exec("/usr/bin/patch \"$path\" -i \"$patch\" -o \"$merged\" ");

			// swap the contents of the merged file with the original
			file_put_contents($path, file_get_contents($merged));
		}
		
		// delete the temporary files
		unlink($patch);
		unlink($merged);
	}

  function PrintGlobalClassInfo($all_classes, $defined_classes, $anon_classes) {
		// add all classes as anonymous external classes to a separate unit.
		// They will be overridden by the actual definitions in the translated
		// headers part of the main unit, but this way they can appear as record
		// field types and as callback parameters

		// open the output file if we not printing to terminal
		if (!$this->show) {
			$this->output = fopen("$this->root$this->out/AnonClassDefinitions".ucfirst($this->framework).".pas", "w+");
		}
		
		$this->PrintOutput(0, "{ Parsed from ".ucfirst($this->framework)." }");

		$date = @date("D M j G:i:s T Y");
		
		$this->PrintOutput(0, "");
		// allows parameter names conflicting with field names
		$this->PrintOutput(0, "{\$mode delphi}");
		$this->PrintOutput(0, "{\$modeswitch objectivec1}");
		// enables "external" after the semi-colon
		$this->PrintOutput(0, "{\$modeswitch cvar}");
		$this->PrintOutput(0, "");

		$this->PrintOutPut(0,"unit AnonClassDefinitions".ucfirst($this->framework).";");
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "interface");
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "type");

    	foreach ($all_classes as $class) {
    		$this->PrintOutput(1, $class." = objcclass external;");
    	}
			
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "implementation");
		$this->PrintOutput(0, "");
		$this->PrintOutput(0, "end.");


		// Now all anonymous external classes that have no real definition to an
		// include file that is added to the main unit. This way it is possible
		// to declare variables of these types in user programs without having to
		// include the unit above will all anonymous classes (should not be used)

		// open the output file if we not printing to terminal
		if (!$this->show) {
			$this->output = fopen("$this->root$this->out/$this->framework/AnonIncludeClassDefinitions".ucfirst($this->framework).".inc", "w+");
		}
		
		$this->PrintOutput(0, "{ Parsed from ".ucfirst($this->framework)." }");

		$date = @date("D M j G:i:s T Y");
		

		// add all classes as anonymous external classes. They will be overridden
		// by the actual definitions in the translated headers, but this way they
		// can appear as record field types and as callback parameters
		$first = true;
    foreach ($anon_classes as $class) {
    	if (!in_array($class,$defined_classes)) {
    		if ($first) {
    			$this->PrintOutput(0, "type");
    			$first = false;
    		}
				$this->PrintOutput(1, $class." = objcclass external;");
			}
		}
  }

	// Prints all headers parsed
	function PrintAllHeaders ($output_path, $ignore_output, $only_files, $print_header_references) {
			
			//print("• Printing ".count($this->dump)." headers...\n");
			
			foreach ($this->dump as $file => $header) {
				
				// the framework is set to not print, ignore the header
				if (!$this->frameworks[$header["framework"]]["print"]) continue;

				if (eregi("^[a-zA-Z0-9]+\.h", $file)) {
					
					// ignore these files
					if (@in_array($header["path_partial"], $ignore_output)) continue;
					
					// only parse these files
					if ((@count($only_files) > 0) && (@!in_array($header["name"], $only_files))) continue;
					
					$name_clean = substr($file, 0, (strripos($file, ".")));	
					
					// assign output path
					if ($output_path != "") {
						$header["path"] = $output_path."/".$name_clean.".inc";
					}
					
					// print the header
					$this->PrintHeader($header);
					
					// merge the headers
					if (($this->merge_headers) && (!$this->show)) {
						$this->MergeHeader($header);
					}
					
					if ($print_header_references) $this->PrintHeaderReference($header, $this->root.$this->out."/reference/".$name_clean.".pas");
					
					if (!$this->show) print("* Printed $name_clean.h to ".$header["path"]."\n");
				}
			}
			
			// print global stuff (for Obj-P: a unit containing all classes as
			// anonymous external classes, so they can be used before they are
			// declared)
			$this->PrintGlobalClassInfo($this->cocoa_classes, $this->defined_cocoa_classes, $this->anon_cocoa_classes);
	}	
	
	/**
	 * PRE-PARSING METHODS
	 */
	
	// Parse all "pre-defined" category methods in a header
	function PreparseCategoryMethods ($file) {
			$contents = file_get_contents($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
							
				// skip blocks
				if ($this->SkipBlock($line)) continue;

				// parse category
				if ($got_category) {
					
					// build method fragment
					if ($method_fragment) $method_fragment .= " ".trim($line, " 	");
					
					// found method fragment termination
					if (($method_fragment) && (preg_match($this->pregex_objc_method_terminate, $line))) {
						$line = $method_fragment;
						$method_fragment = null;
					}
					
					// found method
					$method = null;
					if (preg_match($this->pregex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, array(), true, "");						
					} elseif (preg_match($this->pregex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, array(), false, "");	
					} elseif (preg_match($this->pregex_objc_method_partial, $line, $captures)) {
						$method_fragment = $line;
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
			
			// skip blocks
			if ($this->SkipBlock($line)) continue;

			// build method fragment
			if ($method_fragment) $method_fragment .= " ".trim($line, " 	");
			
			// found method fragment termination
			if (($method_fragment) && (preg_match($this->pregex_objc_method_terminate, $line))) {
				$line = $method_fragment;
				$method_fragment = null;
			}
			
			// found method
			if (preg_match($this->pregex_objc_method_params, $line, $captures)) {
				$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $protected_keywords, true, "");
				$this->current_class["protected_keywords"][] = strtolower($method["name"]);
			} elseif (preg_match($this->pregex_objc_method_no_params, $line, $captures)) {
				$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $protected_keywords, false, "");
				$this->current_class["protected_keywords"][] = strtolower($method["name"]);
			} elseif (preg_match($this->pregex_objc_method_partial, $line, $captures)) {
				$method_fragment = $line;
			}
			
			// class ended
			if (ereg("^@end", $line)) return $protected_keywords;
		}
	}
	
	/**
	 * PARSING METHODS
	 */
	
	function ParseNormalField($line, $protected_keywords, &$field_bitpacked, &$bitpacked_real_type, &$this_class_field_names) {
		if (preg_match("!^\s*([^-{}*@+,<;:]*struct[^-*@+,<;:]+|[^-{}*@+,<;:]+)\b\s*(?:<([^>]*)>)?\s*([*]*)\s*(\w+)((?:\s*,\s*[^:;[]+)*)?\s*(:[0-9]+)?\s*(\[.*\])?\s*(?:__attribute__\(\(([^)]*)\)\))?\s*;!", $line, $captures)) { // regular field
					// captures[1]: type name (may be multiple words, and may not be complete
					//              in case of an anonymous bitfield without a name)
					//              Curly braces are only allowed in case of structs
					// captures[2]: in case of id<protocol list>, the protocol list, empty
					//              otherwise
					// captures[3]: all pointer modifiers (*, **, ...), but only "*" and "**"
					//              currently handled
					// captures[4]: the field name (may be last part of type in case of
					//              anonymous bit field, e.g. "unsigned long : 32")
					// captures[5]: in case of multiple fields, the other field names (note
					//              that things will go wrong if some of the fields in the
					//							list are arrays/bitfields, that is only supported for
					//							a single field)
					// captures[6]: bitfield specification if any, empty otherwise
					// captures[7]:	array specification if any, empty otherwise
					// captures[8]: attributes if any, empty otherwise


					if ($captures[3] != "") { // pointer type -> ok (cannot be wrongly interpreted)
						$type = trim($captures[1]);
						$pointertype = $captures[3];
						$name = trim($captures[4]);
		//				print("pointer field: $name: $pointertype $type\n");
					} else {
						// make sure we properly interpret stuff like "unsigned int :32"
		//				print("regular: type = \"$captures[1]\", name = $captures[3]\n");
						$pair = $this->ExtractCFieldSimpleTypeAndSingleName($captures[1]." ".$captures[4]);
						$type = $pair["type"];
						$name = $pair["name"];
						if ($name == "") $name = $this->GetAnonBitFieldName(); 
		//				print("regular field: \"$name\": $type\n");
					}
		//			print("field \"$name\": \"$type\", attr: $captures[8]\n");

					// if we have id <protocollist> and there's only one protocol in the
					// in the list, we can replace id with the protocol
					if (($type == "id") && ($captures[2] != "") && !strstr(",", $captures[2])) {
		//				print("id<protocol>: $type -> $captures[2]Protocol\n");
						$type = $captures[2]."Protocol";
					}
					// in case of "type field1, field2, field3", ", field2, field3" gets
					// stored in othernames
					$othernames = $captures[5];
					$field_prefix = "";

					// Multiple Objective-C fields cannot have the same name, but they
					// are case-insensitive and some only differ in case (only "reserved"
					// fields until now, but that can change)
					while (in_array(strtolower("$field_prefix$name"),$protected_keywords)) $field_prefix.="_";
					if ($this_class_field_names != null) {
						while (in_array(strtolower("$field_prefix$name"),$this_class_field_names)) $field_prefix.="_";
					}
					if ($this->IsKeywordReserved($field_prefix.$name)) $field_prefix .= "_";
					// protect the name of these fields, in case there are methods with
					// the same name
					$this_class_field_names[] = strtolower($field_prefix.$name);
					if ($othernames != "") {
						$other_lower_case_names = preg_split("/\s*,\s*/",$othernames,-1,PREG_SPLIT_NO_EMPTY);
						array_walk($other_lower_case_names,strtolowerref);
						// should actually also check these for conflicts and add underscores if necessary
					  $this_class_field_names = array_merge ($this_class_field_names, $other_lower_case_names);
					}

		/*
					// we should also add prefixes to the other names if required, but my
					// php-fu is too weak to replace the elements in original string
					// efficiently, and this problem does not occur in the supported headers
					foreach (explode($othernames, ",") as $othername) {
						while (in_array(strtolower("$field_prefix$othername"),$protected_keywords)) $field_prefix.="_";
						if ($this->IsKeywordReserved($field_prefix.$othernamename)) $field_prefix .= "_";
					}
		*/
					// remove "struct" from the type
					$type = preg_replace("!(\b)struct\b!","\1",$type);

					// clean/convert type			
					$type = $this->ReplaceObjcType($type);
					$bitpacked_real_type = $type;
					$type = $this->MakeFieldBitPacked($type, $line, $field_bitpacked);

					// add pointer modifiers
					$type = $this->EncodePointerModifiers($type,$pointertype);

					$field = "$field_prefix$name$othernames: $type";
					if ($captures[8] && strstr("deprecated",$captures[8])) $field .= " deprecated";
					$field .= ";";
					$field = $this->MakeFieldInlineArray($field, $line, $name, $type);
					$field = eregi_replace("<.*>", "", $field);
					return $field;
		}
			return "";
	}

	function ParseInstanceVariables ($line, &$struct, $protected_keywords, &$this_class_field_names) {
		$field = null;
		$field_bitpacked = false;
//		print("$line\n");

		// insert macros
		if ($macro = $this->InsertMacroBlocks($line, $this->inside_macro_block)) {
			if ($struct["valid"]) {
				$struct["fields"][] = $macro;
				return null;
			} else {
				return $macro;
			}
		}

		// got inline struct, probably a reference to a private struct
		if (eregi("[[:space:]]*struct[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+\*([a-zA-Z0-9_]+)", $line, $captures)) {
			if ($struct["valid"]) {
				// ??? These don't compile as inline records which I thought they did...
				//$struct["fields"][] = "$captures[1] = record end;";
				//$struct["fields"][] = "$captures[1]Pointer = ^$captures[1];";
				$struct["fields"][] = "$captures[2]: Pointer;";
				$struct["ignore"][] = "$captures[2]";
				return null;
			} else {
				//$fields = array();
				//$fields[] = "$captures[1] = record end;";
				//$fields[] = "$captures[1]Ptr = ^$captures[1];";
				$field = "_$captures[2]: Pointer;";
				return $field;
			}
		} 

		// got struct
		if (eregi("^[[:space:]]*struct.*{", $line)) {
			$struct["valid"] = true;
			$struct["ispsuedostruct"] = false;
			$struct["fieldnames"] = array();
			return null;
		}

		// create an anonymous struct in case we have bitpacked fields without a
		// surrounding struct
		if (!$struct["valid"] && preg_match("!.*:[0-9]+\s*;!", $line)) {
			$struct["valid"] = true;
			$struct["ispsuedostruct"] = true;
			$struct["fieldnames"] = array();
		}

		// end of bunch of bitfields -> go to normal mode again
		if ($struct["ispsuedostruct"] && !preg_match("!.*:[0-9]+\s*;!", $line)) {
			$struct["name"] = "_anoninternstruct_".$this->current_header["name_clean"].$this->current_header["anoninternstrucs"];
			$this->current_header["anoninternstrucs"]++;
			$struct["isfinished"] = true;
			// make sure the current field isn't added anymore
			$struct["valid"] = false;
		}

		// end of struct
		if (eregi("^[[:space:]]*}[[:space:]]*([a-zA-Z_0-9]+);", $line, $captures)) {
			$struct["name"] = "_".trim($captures[1], " 	");
			//print_r($struct);
			$struct["isfinished"] = true;
			return "struct";
		}

		// set field prefix to protect scope
		if (!$struct["valid"]) $field_prefix = "_";

		// remove null-defined macros: 
		$line = str_ireplace($this->null_macros, "", $line);

		// replace garbage collector hints in the field	
		$line = $this->ReplaceGarbageCollectorHints($line, $garbage_collector_hint);

		if (preg_match($this->pregex_function_pointer, $line, $captures)) { // function pointer
//			print("function pointer: $line\n");
			$field =$this->ParseFunctionDeclaration($captures[1], $captures[2], $captures[3], $captures[4], false, "");
		} else {
		  if (!$struct["valid"])
		    $field = $this->ParseNormalField($line,$protected_keywords,$field_bitpacked,$bitpacked_real_type,$this_class_field_names);
		  else
		  	// don't register the names of fields of embedded structs and field names of the current
		  	// class, but do track them for the current struct as there may be conflicts due to
		  	// Pascal's case-insensitivity
		    $field = $this->ParseNormalField($line,$protected_keywords,$field_bitpacked,$bitpacked_real_type,$struct["fieldnames"]);
		}

		// mark the field as having a garbage collector field
		if ($garbage_collector_hint) $field = "$field {garbage collector: $garbage_collector_hint }";

		// return field
		if ($struct["valid"]) {
			if (!$struct["bitpacked"]) $struct["bitpacked_first_type"] = $bitpacked_real_type;
			if ($field_bitpacked) $struct["bitpacked"] = true;
			$struct["fields"][] = $field;
		} else {
			return $field;
		}
	}
	
	// Parses a struct field into a list
	function ParseStructList ($line, $input, $name, $type) {
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
			$field = "    $name: $type;";
			$field = $this->MakeFieldInlineArray($field, $line, $name, $type)."\n";
		}
		
		return $field;
	}
	
	// Parses $line into a function declaration string. Handles both
	// function pointers (isexternfunc = false) and external functions
	// (isexternfunc = true)
	function ParseFunctionDeclaration($rettypestr, $retpointertypestr, $funcname, $parastr, $isexternfunc, $deprecatedmods) {
	  
	  if ($deprecatedmods != "") $deprecatedmods .= ";";
		if ($this->IsKeywordReserved($funcname)) $funcname .= "_";
	  
	  $rettype = trim(str_replace_word("const","",$rettypestr));
		$rettype = $this->ReplaceObjcType($rettype);
		$rettype = $this->EncodePointerModifiers($rettype,$retpointertypestr);
		$params = $this->ConvertCParamsPascal($parastr);

		if ($rettype == "void")
			$result = "procedure ";
		else
			$result = "function ";
	  
		// if no name specified, the caller will add it
		if ($funcname != "") {
			if (!$isexternfunc)
				$result = "$funcname: " . $result;
			else
				$result .= $funcname;
		}

		if ($params != "") $params = "(" . $params . ")";
		if ($rettype == "void")
			$result .= $params . "; cdecl;";
		else
			$result .= $params . ": " . $rettype . "; cdecl;";
	  
	  if ($isexternfunc)
	  	$result .= " external;$deprecatedmods";
	  else
	  	$result .= "\n";
	  
	  return $result;
	}
	
	// Parses $line into the combined $struct_fields string
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	function ParseStructFields ($line, $protected_keywords, &$struct_fields, &$any_field_parsed, &$found_any_bitpacked, &$all_bitpacked, &$first_bitpacked_type) {
=======
	function ParseStructFields ($line, $protected_keywords, &$struct_fields, &$found_any_bitpacked, &$all_bitpacked, &$first_bitpacked_type) {
>>>>>>> graemeg/cpstrnew
=======
	function ParseStructFields ($line, $protected_keywords, &$struct_fields, &$found_any_bitpacked, &$all_bitpacked, &$first_bitpacked_type) {
>>>>>>> graemeg/cpstrnew
=======
	function ParseStructFields ($line, $protected_keywords, &$struct_fields, &$found_any_bitpacked, &$all_bitpacked, &$first_bitpacked_type) {
>>>>>>> graemeg/cpstrnew
=======
	function ParseStructFields ($line, $protected_keywords, &$struct_fields, &$found_any_bitpacked, &$all_bitpacked, &$first_bitpacked_type) {
>>>>>>> origin/cpstrnew
		if (preg_match($this->pregex_function_pointer, $line, $captures)) {
			$struct_fields .= "    " . $this->ParseFunctionDeclaration($captures[1], $captures[2], $captures[3], $captures[4], false, "");
			$all_bitpacked = false;
		} else {
			// better: keep for entire struct, so we can escape conflicting names due to
			// case-insensitivity
		  $tempnewprotected = array();
			$new_field = $this->ParseNormalField($line,$protected_keywords,$field_bitpacked,$bitpacked_real_type,$tempnewprotected);
//			print("field: '$new_field', bitpacked: $field_bitpacked, any: $found_any_bitpacked, all: $all_bitpacked\n");
			if ($new_field != "") {
				$found_any_bitpacked |= $field_bitpacked;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
				if (!$any_field_parsed) {
					$all_bitpacked = $field_bitpacked;
					$first_bitpacked_type = $bitpacked_real_type;
					$any_field_parsed=true;
=======
				if ($struct_fields == "") {
					$all_bitpacked = $field_bitpacked;
					$first_bitpacked_type = $bitpacked_real_type;
>>>>>>> graemeg/cpstrnew
=======
				if ($struct_fields == "") {
					$all_bitpacked = $field_bitpacked;
					$first_bitpacked_type = $bitpacked_real_type;
>>>>>>> graemeg/cpstrnew
=======
				if ($struct_fields == "") {
					$all_bitpacked = $field_bitpacked;
					$first_bitpacked_type = $bitpacked_real_type;
>>>>>>> graemeg/cpstrnew
=======
				if ($struct_fields == "") {
					$all_bitpacked = $field_bitpacked;
					$first_bitpacked_type = $bitpacked_real_type;
>>>>>>> origin/cpstrnew
				}
				else $all_bitpacked &= $field_bitpacked;
				$struct_fields .= "    " . $new_field . $this->AppendEOLComment() . "\n";
			}
//			print("  after: any: $found_any_bitpacked, all: $all_bitpacked\n");
		}
	}
	
	// Parse a single enum field
	function ParseEnumFields ($line, $file_name, &$block_count, &$auto_increment) {
		
		// insert macros
		//if ($macro = $this->InsertMacroBlocks($line, $this->inside_macro_block)) $this->dump[$file_name]["types"]["enums"][$block_count][] = $macro;

		if (ereg("^[[:space:]]*[,]*[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*[(]*([a-zA-Z_]+)[)]*[,]*[[:space:]]*$", $line, $captures)) { // string value
			$captures[2] = trim($captures[2], ", ");
			$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";".$this->AppendEOLComment();
		} elseif (ereg("^[[:space:]]*[,]*[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*[(]*([0-9-]+)[)]*[,]*[[:space:]]*$", $line, $captures)) { // integer value
			$captures[2] = trim($captures[2], ", ");
			$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";".$this->AppendEOLComment();
			$auto_increment = $captures[2] + 1;
		} elseif (ereg("^[[:space:]]*[,]*[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*[(]*([0-9]+[xX]+[a-fA-F0-9]+)[)]*", $line, $captures)) { // hexadecimal value
			$captures[2] = trim($captures[2], ", ");
			$auto_increment = $captures[2] + 1;
			$captures[2] = eregi_replace("^0x", "$", $captures[2]);
			$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";".$this->AppendEOLComment();
		} elseif (ereg("^[[:space:]]*[,]*[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*([a-zA-Z0-9]+[[:space:]]*<<[[:space:]]*[a-zA-Z0-9]+)", $line, $captures)) { // << shl value, no ()
			$captures[2] = ereg_replace("[[:space:]]?<<[[:space:]]?", " shl ", $captures[2]);
			
			// remove integer type hints
			$captures[2] = ereg_replace("([0-9]+)[UL]+([[:space:]]+)shl([[:space:]])", "\\1\\2shl\\3", $captures[2]);
			$captures[2] = ereg_replace("([[:space:]])shl([[:space:]]+)([0-9]+)[UL]+", "\\1shl\\2\\3", $captures[2]);

			$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";".$this->AppendEOLComment();
			$operands = preg_split("/\s*shl\s*/", $captures[2]);
			$auto_increment = ($operands[0] << $operands[1]) + 1;
			
		} elseif (ereg("^[[:space:]]*[,]*[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*=[[:space:]]*\(([a-zA-Z0-9]+[[:space:]]*<<[[:space:]]*[a-zA-Z0-9]+)\)", $line, $captures)) { // << shl value
			$captures[2] = trim($captures[2], ", ");
			$captures[2] = ereg_replace("[[:space:]]?<<[[:space:]]?", " shl ", $captures[2]);
			
			// remove integer type hints
			$captures[2] = ereg_replace("([0-9]+)[UL]+([[:space:]]+)shl([[:space:]])", "\\1\\2shl\\3", $captures[2]);
			$captures[2] = ereg_replace("([[:space:]])shl([[:space:]]+)([0-9]+)[UL]+", "\\1shl\\2\\3", $captures[2]);

			$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$captures[2].";".$this->AppendEOLComment();
			
			$operands = preg_split("/\s*shl\s*/", $captures[2]);
			$auto_increment = ($operands[0] << $operands[1]) + 1;
		} elseif (ereg("^[[:space:]]*[,]*[[:space:]]*([a-zA-Z0-9_]+)[[:space:]]*[,}]*[[:space:]]*$", $line, $captures)) { // non-value
			
			// omit lines which started nested structures.
			// bad practice but the single-line regex parser can't handle them
			if (!eregi("[=|]+", $line)) {
				$captures[1] = trim($captures[1], ", ");
				$this->dump[$file_name]["types"]["enums"][$block_count][] = $captures[1]." = ".$auto_increment.";";
				$auto_increment ++;
			}
		}
	}
		
	// Parse external symbols, enums and typedef's from the header
	function ParseHeaderTypes ($file) {
			$contents = file_get_contents($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			$any_field_parsed = false;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
			$any_field_bitpacked = false;
			$all_fields_bitpacked = false;
			
			// reset comments from previous parsing sections
			$this->ResetComment();
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
				
				// skip blocks
				if ($this->SkipBlock($line)) continue;
				
				// ignore lines
				if (in_array($line, $this->ignore_lines)) continue;
				
				// build comments
				$this->BuildComment($line, $file_name);
				
				// build macro blocks
				$this->BuildMacroBlocks($line);
				
				// garbage collector hints
				$line = $this->ReplaceGarbageCollectorHints($line, $garbage_collector_hint);
					
				// remove macros	
				$line = $this->RemoveVersionMacros($line, $deprecatedmods);
				
				// remove comments
				$line = $this->RemoveComments($line);
				$line = trim($line, " ");
									
				if ($got_struct) {
					
					// insert macros
					if ($macro = $this->InsertMacroBlocks($line, $this->inside_macro_block)) $struct_fields .= "$macro\n";
					
					// collect fields
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					$this->ParseStructFields($line, array(), $struct_fields, $any_field_parsed, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
=======
					$this->ParseStructFields($line, array(), $struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> graemeg/cpstrnew
=======
					$this->ParseStructFields($line, array(), $struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> graemeg/cpstrnew
=======
					$this->ParseStructFields($line, array(), $struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> graemeg/cpstrnew
=======
					$this->ParseStructFields($line, array(), $struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> origin/cpstrnew
					
					// got end of struct
					if (ereg("^}[[:space:]]*([a-zA-Z_0-9]+);", $line, $captures)) {
						
						if ($struct_name == "") {
							$struct_name = $captures[1];
						} else {
							$struct_type = $captures[1];
						}
						
						// ignore this struct
						if (in_array($struct_name, $this->ignore_types)) continue;
						
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						$struct = "$struct_comment$struct_name = $this->record_keyword\n";
						if ($any_field_bitpacked) {
							$struct .= $this->BitPackedForceAlignment($first_bitpacked_type, "    ", "    ") . "\n";
							$struct_fields = str_replace("    ","          ",$struct_fields);
							$struct_fields .= "         end;\n       );\n";
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
						if ($all_fields_bitpacked) {
							$struct = "$struct_comment$struct_name = $this->bitpacked_record_keyword\n";
							$struct .= $this->BitPackedForceAlignment($first_bitpacked_type, "    ", "    ") . "\n";
							$struct_fields = str_replace("    ","          ",$struct_fields);
							$struct_fields .= "       );\n";
						} else {
							$struct = "$struct_comment$struct_name = $this->record_keyword\n";
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
						}
						
						$struct .= $struct_fields;
						$struct .= "  end$deprecatedmods;\n";
						if (($struct_type) && ($struct_name != $struct_type)) {
							$struct .= "$struct_type = $struct_name;\n";
						}
						// pointer type
						$struct .= $struct_name."Ptr = ^".$struct_name.";\n";
						
						$this->dump[$file_name]["types"]["structs"][] = $struct;
						$this->dump["global_structs"][] = $struct_name;
						$got_struct = false;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						$any_field_parsed = false;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
						$any_field_bitpacked = false;
						$all_fields_bitpacked = false;
					}
				}
				
				// got single-line struct
				if (ereg("^typedef[[:space:]]+struct[[:space:]]+{(.*)}[[:space:]]+([a-zA-Z0-9_]+)", $line, $captures)) {
					$struct_name = trim($captures[2], " ");
					if (!in_array($struct_name, $this->ignore_types)) {
						

						// break the struct into lines
						$single_struct_fields = "";
						$fields = explode(";", $captures[1]);
						$comment = $this->InsertCurrentComment();
						$this->ResetComment();
						
						// parse each line
						foreach ($fields as $field) {
							$field = trim($field);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
							$this->ParseStructFields($field.";", array(), $single_struct_fields, $any_field_parsed, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
=======
							$this->ParseStructFields($field.";", array(), $single_struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> graemeg/cpstrnew
=======
							$this->ParseStructFields($field.";", array(), $single_struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> graemeg/cpstrnew
=======
							$this->ParseStructFields($field.";", array(), $single_struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> graemeg/cpstrnew
=======
							$this->ParseStructFields($field.";", array(), $single_struct_fields, $any_field_bitpacked, $all_fields_bitpacked, $first_bitpacked_type);
>>>>>>> origin/cpstrnew
						}
						
						// merge the fields into the definition
						$struct = "$comment\n"."$struct_name = ";
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						$struct .= "$this->record_keyword\n";
						if ($any_field_bitpacked) {
							$struct .= $this->BitPackedForceAlignment($first_bitpacked_type, "    ", "    ") . "\n";
							$single_struct_fields = str_replace("    ","          ",$single_struct_fields);
							$single_struct_fields .= "         end;\n       );\n";
						}	else ;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
						if ($all_fields_bitpacked) {
							$struct .= "$this->bitpacked_record_keyword\n";
							$struct .= $this->BitPackedForceAlignment($first_bitpacked_type, "    ", "    ") . "\n";
							$single_struct_fields = str_replace("    ","          ",$single_struct_fields);
							$single_struct_fields .= "       );\n";
						}	else $struct .= "$this->record_keyword\n";
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
						$struct .= $single_struct_fields;
						$struct .= "  end$deprecatedmods;\n";
						// pointer type
						$struct .= $struct_name."Ptr = ^".$struct_name.";\n";
						
						$this->dump[$file_name]["types"]["structs"][] = $struct;
						$this->dump["global_structs"][] = $struct_name;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						$any_field_parsed = false;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
						$any_field_bitpacked = false;
						$all_fields_bitpacked = false;
						//print("$single_struct_fields\n");
					}
				// got begin of struct
				} elseif (ereg("^typedef struct(.*){", $line, $captures)) {
					$struct_name = trim($captures[1], " ");
					if (!in_array($struct_name, $this->ignore_types)) {
						$struct_type = null;
						$struct_fields = "";
						$struct_comment = $this->InsertCurrentComment();
						$this->ResetComment();
						if ($struct_comment != "") $struct_comment = "$struct_comment\n";
						$got_struct = true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						print("Parsing struct $struct_name\n");
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
					}
				}

				// got function pointer type
				if (preg_match($this->pregex_function_pointer_typedef, $line, $captures)) {
					
					$typestr = $this->ParseFunctionDeclaration($captures[1], $captures[2], "", $captures[6], false, $deprecatedmods);
					$functypename = $captures[7];
					if ($functypename == "") $functypename = $captures[5];
					if ($functypename == "") $functypename = $captures[4];
					$this->dump[$file_name]["types"]["callbacks"][$functypename] = $typestr;
					// record if it is a function type instead of a function pointer type
					if ($captures[3] == "") $this->implicit_function_pointer_types[] = $functypename;

					continue;
				}
				
				// #defines
				$got_define = false;
				if (ereg("#[[:space:]]*define[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+\(\(.*\)(.*)\)", $line, $captures)) { // named constant with type case
					$got_define = true;
				} elseif (ereg("#[[:space:]]*define[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+[(]*([0-9.-]+)[)]*", $line, $captures)) { //integer
					$got_define = true;
				}
				
				if ($got_define) {
					$define_name = $captures[1];
					if (!in_array($define_name, $this->ignore_types)) {
						$define_name = $this->ReplaceObjcType($define_name);
						//if ($this->comment_terminated) $this->dump[$file_name]["types"]["defines"][] = $this->InsertCurrentComment();
						$this->AppendCurrentComment($this->dump[$file_name]["types"]["defines"]);
						$this->dump[$file_name]["types"]["defines"][] = $define_name." = ".$captures[2].";".$this->AppendEOLComment();
					} else {
						$this->ResetComment();
					}
				}
								
				// parse enum fields
				if (($got_enum) || ($got_named_enum)) {
					// print($line.", auto_inc = $auto_increment\n");
					
					$this->ParseEnumFields($line, $file_name, &$block_count, &$auto_increment);

					// found the end
					if (ereg("^};", $line)) $got_enum = false;
				}
				
				// ==== got inline named enum ===
				if (ereg("^[[:space:]]*enum[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]*{(.*)};", $line, $captures)) {
					//print("$line\n");
					
					$enum_name = trim($captures[1], " ");
					if (!in_array($enum_name, $this->ignore_types)) {
						
						$block_count ++;
						$auto_increment = 0;

						// break the enum into lines
						$fields = explode(",", $captures[2]);
						//$this->AppendCurrentMacro($this->dump[$file_name]["types"]["enums"][$block_count]);
						$this->AppendCurrentComment($this->dump[$file_name]["types"]["enums"][$block_count]);
						if ($this->comment_terminated) $this->dump[$file_name]["types"]["enums"][$block_count][] = $this->InsertCurrentComment();
						$this->ResetComment();
						
						// parse each line
						foreach ($fields as $field) {
							$field = trim($field, " ");
							$this->ParseEnumFields($field.",", $file_name, &$block_count, &$auto_increment);
						}
					}
					
					continue;
				}

				// ==== got inline enum ===
				if (ereg("^[[:space:]]*enum[[:space:]]*{(.*)};", $line, $captures)) {
					//print("$line\n");
					
					$block_count ++;
					$auto_increment = 0;

					// break the enum into lines
					$fields = explode(",", $captures[1]);
					$this->AppendCurrentComment($this->dump[$file_name]["types"]["enums"][$block_count]);
					//if ($this->comment_terminated) $this->dump[$file_name]["types"]["enums"][$block_count][] = $this->InsertCurrentComment();
					$this->ResetComment();
					
					// parse each line
					foreach ($fields as $field) {
						$field = trim($field, " ");
						$this->ParseEnumFields($field.",", $file_name, &$block_count, &$auto_increment);
					}
					
					continue;
				}
				
				// ==== got enum ===
				if (ereg("^enum", $line)) {
					$got_enum = true;
					$block_count ++;
					$auto_increment = 0;
					$this->AppendCurrentComment($this->dump[$file_name]["types"]["enums"][$block_count]);
					//if ($this->comment_terminated) $this->dump[$file_name]["types"]["enums"][$block_count][] = $this->InsertCurrentComment();
				}
				
				// terminate named enum
				if ($got_named_enum) {
					if (ereg("^}[[:space:]]*([a-zA-Z0-9_]+);", $line, $captures)) {
						$got_named_enum = false;
						
						$named_enum = trim($named_enum, ", \n");
						$name = $captures[1];
						
						if (!in_array($name, $this->ignore_types)) {
							$this->dump[$file_name]["types"]["named_enums"][] = "$name = culong;";
							$this->dump["global_types"][$name] = $name;
						}
					}
				}

				// ==== got named enum ===
				if (ereg("^typedef enum {", $line)) {
					$got_named_enum = true;
					$named_enum = "";
					$auto_increment = 0;
					$block_count ++;
					$this->AppendCurrentComment($this->dump[$file_name]["types"]["named_enums"][$block_count]);
					//if ($this->comment_terminated) $this->dump[$file_name]["types"]["named_enums"][] = $this->InsertCurrentComment();
				}
				
				// ==== external functions ===
				// doesn't work when $this->external_string_macros is added to
				// the string at initialisation time, because it can still change
				// later (while loading frameworks.xml)
				if (preg_match("!^(?:$this->external_string_macros)+".$this->pregex_external_function_end, $line, $captures)) {
				
					// ignore symbols
					if (in_array($captures[3], $this->ignore_types)) continue;

					$typestr = $this->ParseFunctionDeclaration($captures[1], $captures[2], $captures[3], $captures[4], true, $deprecatedmods);
					
					$this->dump[$file_name]["types"]["functions"][] = $typestr;
					continue;
				}
				
				// ==== external string constant ===
				if (eregi("^($this->external_string_macros)+[[:space:]]+NSString[[:space:]]*\*[[:space:]]*(const)*[[:space:]]*([a-zA-Z0-9_]+)", $line, $captures)) {
					$name = $captures[3];
					
					if (in_array($name, $this->ignore_types)) continue;
					
					// insert comments
					$this->AppendCurrentComment($this->dump[$file_name]["types"]["string_constant"]);
					//if ($this->comment_terminated) $this->dump[$file_name]["types"]["string_constant"][] = $this->InsertCurrentComment();
					
					$this->dump[$file_name]["types"]["string_constant"][] = "$name: $this->string_macro$deprecatedmods; cvar; external;";
				}
				
				// ==== external symbol ===
				if (eregi("^($this->external_string_macros)+[[:space:]]+([a-zA-Z0-9_ ]+)[[:space:]]+([a-zA-Z0-9_]+)", $line, $captures)) {
					$name = $captures[3];
					$type = $captures[2];
					
					// ignore symbols
					if (in_array($name, $this->ignore_types)) continue;
					
					$type = istr_replace_word("const", "", $type);
					$type = $this->ReplaceObjcType(trim($type, " "));
					
					
					$this->AppendCurrentComment($this->dump[$file_name]["types"]["external_symbol"]);
					//if ($this->comment_terminated) $this->dump[$file_name]["types"]["external_symbol"][] = $this->InsertCurrentComment();
					$this->dump[$file_name]["types"]["external_symbol"][] = "$name: $type$deprecatedmods; cvar; external;";
				}
								
				// ==== got typedef ===
				if (ereg("^typedef[[:space:]]+struct[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_]+);", $line, $captures)) { // defined struct type
					$real_type = $captures[1];
					$struct_type = $captures[1];
					$new_type = $captures[2];
					
					// ignore types
					if (in_array($struct_type, $this->ignore_types)) continue;
					if (in_array($new_type, $this->ignore_types)) continue;
					
					$this->AddTypeDef($this->dump[$file_name], "$struct_type = record end$deprecatedmods;");
					
					$struct_type = $this->ReplaceObjcType($struct_type);
					if ($new_type != $struct_type) {
						$this->AddTypeDef($this->dump[$file_name], "$new_type = $struct_type$deprecatedmods;");
						$this->dump["global_types"][$new_type] = $real_type;
					}
					$this->opaque_structs[] = $struct_type;
					
					// also add pointer type to the opaque struct
					$this->AddTypeDef($this->dump[$file_name], $new_type."Ptr = ^$new_type$deprecatedmods;");
					
					$this->dump["global_types"][$struct_type] = "record end";
					$this->dump["global_types"][$new_type."Ptr"] = "^".$new_type;
				} elseif (ereg("^typedef[[:space:]]+struct[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*]+);", $line, $captures)) { // pointer to struct
					$real_type = $captures[1];
					$clean_name = trim($captures[2], "*");
					$pointer_type = $captures[1];
					
					// ignore types
					if (in_array($clean_name, $this->ignore_types)) continue;
					
					$pointer_type = "Pointer";
					$this->AddTypeDef($this->dump[$file_name], "$clean_name = $pointer_type$deprecatedmods;");
					
					$this->dump["global_types"][$clean_name] = $real_type;

					// also add pointer type
					$this->AddTypeDef($this->dump[$file_name], $clean_name."Ptr = ^$clean_name$deprecatedmods;");
					$this->dump["global_types"][$clean_name."Ptr"] = "^".$clean_name;

				} elseif (ereg("^typedef[[:space:]]+(const)*[[:space:]]*struct[[:space:]]+([a-zA-Z0-9_*]+)[[:space:]]+([a-zA-Z0-9_]+);", $line, $captures)) { // struct type (complex)
					$real_type = $captures[1];
					$typedef_name = $captures[3];
					
					// ignore types
					if (in_array($typedef_name, $this->ignore_types)) continue;
					
					$captures[2] = $this->FormatObjcType($captures[2], $modifiers);
					$this->AddTypeDef($this->dump[$file_name], $typedef_name." = ".$captures[2].$deprecatedmods.";");
					
					$this->dump["global_types"][$typedef_name] = $real_type;

					// also add pointer type
					$this->AddTypeDef($this->dump[$file_name], $typedef_name."Ptr = ^$typedef_name$deprecatedmods;");
					$this->dump["global_types"][$typedef_name."Ptr"] = "^".$typedef_name;
				} elseif (ereg("^typedef[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*]+);", $line, $captures)) { // single-word type
					$real_type = $captures[1];
					
					// type is a pointer
					if ($captures[2][0] == "*") {
						$captures[2] = trim($captures[2], "*");
						$captures[1] = $this->ReplaceObjcType($captures[1]);
						
						// ignore types
						if (in_array($captures[2], $this->ignore_types)) continue;
						
						$this->AddTypeDef($this->dump[$file_name], $captures[2]." = ^".$captures[1]."$deprecatedmods;");
						
						$this->dump["global_types"][$captures[2]] = $real_type;

					} else {
						$captures[2] = trim($captures[2], "*");
						$captures[1] = $this->ReplaceObjcType($captures[1]);
						
						// ignore types
						if (in_array($captures[2], $this->ignore_types)) continue;
												
						$this->AddTypeDef($this->dump[$file_name],$captures[2]." = ".$captures[1]."$deprecatedmods;");
					}
					// also add pointer type
					$this->AddTypeDef($this->dump[$file_name], $captures[2]."Ptr = ^$captures[2]$deprecatedmods;");
					$this->dump["global_types"][$captures[2]."Ptr"] = "^".$captures[2];
				} elseif (ereg("^typedef[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_]+)[[:space:]]+([a-zA-Z0-9_*]+);", $line, $captures)) { // double-word type
					$real_type = $captures[1];
					
					$typedef_name = trim($captures[3], "*");
					$long_type = $captures[1]." ".$captures[2];
					$long_type = $this->ReplaceObjcType($long_type);
					
					// ignore types
					if (in_array($captures[2], $this->ignore_types)) continue;
					
					$this->AddTypeDef($this->dump[$file_name], $typedef_name." = $long_type$deprecatedmods;");
					
					$this->dump["global_types"][$typedef_name] = $real_type;

					// also add pointer type
					$this->AddTypeDef($this->dump[$file_name], $typedef_name."Ptr = ^$typedef_name$deprecatedmods;");
					$this->dump["global_types"][$typedef_name."Ptr"] = "^".$typedef_name;
				}
			}
			
		//print_r($this->dump[$file_name]["types"]);
	}	
		
	// Parse all protocols in a header
	function ParseHeaderProtocols ($file) {
			$contents = file_get_contents($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			$section = null;
			
			// reset comments from previous parsing sections
			$this->ResetComment();
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
							
				// skip blocks
				if ($this->SkipBlock($line)) continue;
						
				// ignore lines
				if (in_array($line, $this->ignore_lines)) continue;
										
				// remove macros	
				$line = $this->RemoveVersionMacros($line, $deprecatedmods);
															
				// build comments
				if (!$got_got_protocol) $this->BuildComment($line, $file_name);
							
				// parse protocol
				if ($got_protocol) {
					
					// build comments
					$this->BuildComment($line, $file_name);
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// found @optional/@required section
					if (eregi("^[[:space:]]*@(optional|required)+", $line, $captures)) {
						$section = $captures[1];
					}
					
					// found property
					if ($this->LineHasProperty($line, $captures)) {
							$properties = $this->ParseClassProperty($current_protocol, $captures, $deprecatedmods);
							
							foreach ($properties as $property) {
								
								if ($property["setter"]) {
									$property["setter"]["comment"] = $this->InsertCurrentComment();
									$property["setter"]["section"] = $section;
									$property["setter"]["documentation"] = $this->FindDocumentationForMethod($current_protocol, $property["setter"]["property"]);
									
									$this->current_header["protocols"][$current_protocol]["methods"][$property["setter"]["objc_method"]] = $property["setter"];
							
									// append to master list of protocols
									$this->dump["protocols"][$current_protocol][] = $property["setter"];
								}
							
								if ($property["getter"]) {
									$property["getter"]["comment"] = $this->InsertCurrentComment();
									$property["getter"]["section"] = $section;
									$property["getter"]["documentation"] = $this->FindDocumentationForMethod($current_protocol, $property["getter"]["property"]);

									$this->current_header["protocols"][$current_protocol]["methods"][$property["getter"]["objc_method"]] = $property["getter"];
							
									// append to master list of protocols
									$this->dump["protocols"][$current_protocol][] = $property["getter"];
								}
							}
						
							continue;
					}
					
					// found method
					$method = null;
					if (preg_match($this->pregex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_protocol, $line, $captures, array(), true, $deprecatedmods);	
					} elseif (preg_match($this->pregex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_protocol, $line, $captures, array(), false, $deprecatedmods);
					} elseif (preg_match($this->pregex_objc_method_partial, $line, $captures)) {
						$method_fragment = $line;
					}

					// append to classes
					if (($method)  && (!in_array($method["name"], $this->ignore_methods)) ) {
						
						// add comment to the method
						$method["comment"] = $this->InsertCurrentComment();
						
						// add optional/required section to the method
						$method["section"] = $section;

						// add documentation for method
						$method["documentation"] = $this->FindDocumentationForMethod($current_protocol, $method["objc_method"]);
						
						$this->current_header["protocols"][$current_protocol]["methods"][$method["objc_method"]] = $method;
						
						// append to master list of protocols
						$this->dump["protocols"][$current_protocol][] = $method;
					}
					
					// found the end
					if (ereg("^@end", $line)) {
						$this->ResetComment();
						$got_protocol = false;
					}
				}
				
				// got protocol
				if ((eregi($this->regex_objc_protocol, $line, $captures)) && (!eregi(".*;$", $line))) {
						$got_protocol = true;
						$current_protocol = $captures[1];
						
						print("+ Protocol $current_protocol\n");
						
						if ($this->comment_terminated) $this->current_header["protocols"][$current_protocol]["comment"] = $this->InsertCurrentComment();
						$this->current_header["protocols"][$current_protocol]["name"] = $captures[1];
				}
			}
			
		//print_r($this->current_class);
	}
	
	// Parse all categories in a header
	function ParseHeaderCategories ($file) {
			$contents = file_get_contents($file);
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			
			// reset comments from previous parsing sections
			$this->ResetComment();
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {

				// skip blocks
				if ($this->SkipBlock($line)) continue;

				// ignore lines
				if (in_array($line, $this->ignore_lines)) continue;

				// remove macros	
				$line = $this->RemoveVersionMacros($line, $deprecatedmods);

				// build comments
				if (!$got_category) $this->BuildComment($line, $file_name);
				
				// parse category
				if ($got_category) {
					
					// build comments
					$this->BuildComment($line, $file_name);
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// found property
					if ($this->LineHasProperty($line, $captures)) {
							$properties = $this->ParseClassProperty($current_category, $captures, $deprecatedmods);

							if (!in_array($current_category, $this->ignore_categories)) {
								foreach ($properties as $property) {
									if ($property["setter"]) {
										if ($this->AddMethodToClass($property["setter"], $this->current_class)) {
											$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["methods"][] = $property["setter"];
											$this->dump[$category_owner]["categories"][$current_category]["methods"][] = $property["setter"];
										}
									}
								
									if ($property["getter"]) {
										if ($this->AddMethodToClass($property["getter"], $this->current_class)) {
											$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["methods"][] = $property["getter"];
											$this->dump[$category_owner]["categories"][$current_category]["methods"][] = $property["getter"];
										}
									}
								}
						  }
							continue;
					}
					
					// build method fragment
					if ($method_fragment) $method_fragment .= " ".trim($line);
					
					// found method fragment termination
					if (($method_fragment) && (preg_match($this->pregex_objc_method_terminate, $line))) {
						$line = $method_fragment;
						$method_fragment = null;
					}
					
					// found method
					$method = null;
					if (preg_match($this->pregex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, $this->GetProtectedKeywords($this->current_class), true, $deprecatedmods);						
					} elseif (preg_match($this->pregex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current_category, $line, $captures, $this->GetProtectedKeywords($this->current_class), false, $deprecatedmods);	
					} elseif (preg_match($this->pregex_objc_method_partial, $line, $captures)) {
						$method_fragment = $line;
					}
					
					// append to classes
					if (($method) && !in_array($current_category, $this->ignore_categories)) {
						if ($current_class) {
							if ($this->AddMethodToClass($method, $this->current_class)) {
								$this->dump[$category_owner]["classes"][$current_class]["categories"][$current_category]["methods"][] = $method;
								$this->dump[$category_owner]["categories"][$current_category]["methods"][] = $method;
							}
						}
						
					}
					
					// found the end
					if (ereg("^@end", $line)) {
						$got_category = false;
						continue;
					}
				}
				
				// got category
				if (eregi($this->regex_objc_category, $line, $captures)) {
					
					$got_category = true;
					$category_owner = $file_name;
					$category_name = $captures[2];
					$current_class = $captures[1];
					$current_category = $category_name;

					if (!in_array($current_category, $this->ignore_categories)) {
						// Protect category names against duplicate identifiers by appending the class it extends to the name
						if ((count($this->dump["categories"][$current_category]) > 0) || (in_array($category_name, $this->cocoa_classes))) {
							$current_category = $category_name."_".$current_class;
							$this->dump[$file_name]["categories"][$current_category]["external"] = true;
						} else {
							$this->dump[$file_name]["categories"][$current_category]["external"] = false;
						}
						
						$this->current_class = &$this->dump[$category_owner]["classes"][$current_class];
						
						// insert into headers category array
						$this->dump[$file_name]["categories"][$current_category]["name"] = $current_category;
						$this->dump[$file_name]["categories"][$current_category]["super"] = $current_class;
						$this->dump[$file_name]["categories"][$current_category]["comment"] = $this->InsertCurrentComment();
						$this->dump[$file_name]["categories"][$current_category]["external_name"] = $category_name;
	
						// append to master list of categories
						$this->dump["categories"][$category_name][] = $current_class;
						
					}
				}
			}
			
		//print_r($this->current_class);
	}
			
	// Parse a property into accessor methods
	function ParseClassProperty ($class, $parts, $deprecatedmods) {
		
		$method = array();
		//print_r($parts);

		// property has attributes
		if (count($parts) == 5) {
			//$property["parameters"] = explode(",", $parts[1]);
			$property["parameters"] = preg_split("/\s*,\s*/", $parts[1]);
			$attributes = $parts[1];
			$type = $parts[2];
			$pointertype = $parts[3];
			$content = $parts[4];
		} else {
			$property["parameters"] = array();
			$type = $parts[1];
			$pointertype = $parts[2];
			$content = $parts[3];
		}
		
		// unspecified type -> id
		if ($type == "") $type = "id";
		
		// get property list
		$list = explode(",", $content);
		if (count($list) > 1) {
			$property_list = array();
			foreach ($list as $key) {
				// clean the name and remove the return type
				$property_list[] = trim($key);
			}
			//print_r($property_list);
		} else {
			$property_list = array($content);
		}
		
		$methods = array();
		
		foreach ($property_list as $property_name) {
			
			// property name
			if (eregi("([a-zA-Z0-9_]+)[[:space:]]*$", $property_name, $captures)) {
				$property["name"] = ucwords($captures[1]);
				$property["name_raw"] = $captures[1];
			}

			// property type
			$type = $this->ConvertReturnType($type,$pointertype);

			// prepare for appending
			if ($deprecatedmods != "") $deprecatedmods .= ";";

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
				$method["setter"]["deprecated"] = $deprecatedmods;
				$method["setter"]["property"] = $property["name_raw"];
				//$method["setter"]["comment"] = $this->InsertCurrentComment();
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
			$method["getter"]["deprecated"] = $deprecatedmods;
			$method["getter"]["property"] = $property["name_raw"];
			//$method["getter"]["comment"] = $this->InsertCurrentComment();
			
			// append to array of methods
			$methods[] = $method;
			
		}
		
		//print_r($methods);
		return $methods;
	}
		
	// Parse header classes and methods
	function ParseHeaderClasses ($file) {
			$contents = file_get_contents($file);
			
			$file_name = substr($file, (strripos($file, "/")) + 1, strlen($file));
			$line_count = 0;
			
			// reset comments from previous parsing sections
			$this->ResetComment();
			
			$lines = explode("\n", $contents);
			foreach ($lines as $line) {
				$line_count++;
				
				// skip blocks
				if ($this->SkipBlock($line)) continue;
						
				// ignore lines
				if (in_array($line, $this->ignore_lines)) continue;
						
				// remove macros	
				$line = $this->RemoveVersionMacros($line, $deprecatedmods);
												
				// remove external class macros
				$line = eregi_replace("^[A-Z0-9]+_EXTERN_CLASS[[:space:]]+", "", $line);
				
				// build comments
				if (!$got_class) $this->BuildComment($line, $file_name);
				
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
					$result = $this->ParseInstanceVariables($line, $struct, $this->GetProtectedKeywords($this->current_class),$this->dump["master"][$this->current_class["name"]]["field_names"]);
					
					// parse structures
					if ($struct["isfinished"]) {
						//print_r($struct);
						//$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": $current"."_".$struct["name"].";";
						$this->dump[$file_name]["classes"][$current]["ivars_structs"][] = $struct;
						
						// print inline-record type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": ".$this->record_keyword;
						if ($struct["bitpacked"]) {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $this->BitPackedForceAlignment($struct["bitpacked_first_type"], "  ", "       ");
						}
						
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
						if ($struct["bitpacked"]) {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": ".$this->bitpacked_record_keyword;
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $this->BitPackedForceAlignment($struct["bitpacked_first_type"], "  ", "       ");
						} else {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $struct["name"].": ".$this->record_keyword;
						}
						
						
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
						// print fields
						if ($struct["fields"]) {
							foreach ($struct["fields"] as $field) $this->dump[$file_name]["classes"][$current]["ivars"][] = "    ".$field;
						}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						if ($struct["bitpacked"]) {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = "   end;";
							$this->dump[$file_name]["classes"][$current]["ivars"][] = "  );";
						}
=======
						if ($struct["bitpacked"]) $this->dump[$file_name]["classes"][$current]["ivars"][] = "  );";
>>>>>>> graemeg/cpstrnew
=======
						if ($struct["bitpacked"]) $this->dump[$file_name]["classes"][$current]["ivars"][] = "  );";
>>>>>>> graemeg/cpstrnew
=======
						if ($struct["bitpacked"]) $this->dump[$file_name]["classes"][$current]["ivars"][] = "  );";
>>>>>>> graemeg/cpstrnew
=======
						if ($struct["bitpacked"]) $this->dump[$file_name]["classes"][$current]["ivars"][] = "  );";
>>>>>>> origin/cpstrnew
						$this->dump[$file_name]["classes"][$current]["ivars"][] = "  end;";
						
						$struct = null;
					}

					if(($result != null) && ($result != "struct")) {
						//print($result);
						
						// add a single string or an array of fields to the ivars array
						if (count($result) <= 1) {
							$this->dump[$file_name]["classes"][$current]["ivars"][] = $result;
						} else {
							foreach ($result as $field) {
								$this->dump[$file_name]["classes"][$current]["ivars"][] = $field;
							}
						}
					}
					
					// instance var section terminated.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					if (preg_match("!^\s*}\s*[;]*$!", $line)) {
=======
					if (eregi("^\s*}\s*[;]*$", $line)) {
>>>>>>> graemeg/cpstrnew
=======
					if (eregi("^\s*}\s*[;]*$", $line)) {
>>>>>>> graemeg/cpstrnew
=======
					if (eregi("^\s*}\s*[;]*$", $line)) {
>>>>>>> graemeg/cpstrnew
=======
					if (eregi("^\s*}\s*[;]*$", $line)) {
>>>>>>> origin/cpstrnew
						$struct = null;
						$got_instance_vars = false;
						$this->instance_var_scope = null;
					}
					
				} elseif ($got_class) { // parse the class
					
					// the instance variable section started after the class line and no other ivar's were parsed yet
					if (!$this->dump[$file_name]["classes"][$current]["ivars"]) {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
						if (preg_match("!{\s*$!", $line)) {
=======
						if (eregi("{\s*$", $line)) {
>>>>>>> graemeg/cpstrnew
=======
						if (eregi("{\s*$", $line)) {
>>>>>>> graemeg/cpstrnew
=======
						if (eregi("{\s*$", $line)) {
>>>>>>> graemeg/cpstrnew
=======
						if (eregi("{\s*$", $line)) {
>>>>>>> origin/cpstrnew
							$got_instance_vars = true;
							continue;
						}
					}
					
					// build comments
					$this->BuildComment($line, $file_name);
					
					// remove comments
					$line = $this->RemoveComments($line);
					
					// found property
					if ($this->LineHasProperty($line, $captures)) {
							$properties = $this->ParseClassProperty($current, $captures, $deprecatedmods);
							foreach ($properties as $property) {
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
							}
							
							continue;
					}
					
					// build method fragment
					if ($method_fragment) $method_fragment .= " ".trim($line, " 	");
					
					// found method fragment termination
					if (($method_fragment) && (preg_match($this->pregex_objc_method_terminate, $line))) {
						$line = $method_fragment;
						$method_fragment = null;
					}
					
					// found method
					if (preg_match($this->pregex_objc_method_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $this->GetProtectedKeywords($this->current_class), true, $deprecatedmods);						
						
						// add documentation for method
						$method["documentation"] = $this->FindDocumentationForMethod($current, $method["objc_method"]);
						
						if ($this->AddMethodToClass($method, $this->dump[$file_name]["classes"][$current])) {
							//if ($this->comment_terminated) $method["comment"] = $this->InsertCurrentComment();
							$this->dump[$file_name]["classes"][$current]["methods"][] = $method;
						}
						
					} elseif (preg_match($this->pregex_objc_method_no_params, $line, $captures)) {
						$method = $this->ConvertObjcMethodToPascal($current, $line, $captures, $this->GetProtectedKeywords($this->current_class), false, $deprecatedmods);
						
						// add documentation for method
						$method["documentation"] = $this->FindDocumentationForMethod($current, $method["objc_method"]);

						if ($this->AddMethodToClass($method, $this->dump[$file_name]["classes"][$current])) {
							//if ($this->comment_terminated) $method["comment"] = $this->InsertCurrentComment();
							$this->dump[$file_name]["classes"][$current]["methods"][] = $method;
						}
					} elseif (preg_match($this->pregex_objc_method_partial, $line, $captures)) {
						$method_fragment = $line;
					}
									
					// found the end
					if (ereg("^@end", $line)) {
						$got_class = false;
						$this->ResetComment();
					}
				}

				// ==== got class ====
				if ((eregi($this->regex_objc_class, $line, $captures)) || (eregi($this->regex_objc_class_no_super, $line, $captures))) {
					$current = $captures[1];
					$got_class = true;
					$has_superclass = true;
					
					// check for instance variable section
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					if (preg_match("!{\s*$!", $line)) $got_instance_vars = true;
=======
					if (eregi("{\s*$", $line)) $got_instance_vars = true;
>>>>>>> graemeg/cpstrnew
=======
					if (eregi("{\s*$", $line)) $got_instance_vars = true;
>>>>>>> graemeg/cpstrnew
=======
					if (eregi("{\s*$", $line)) $got_instance_vars = true;
>>>>>>> graemeg/cpstrnew
=======
					if (eregi("{\s*$", $line)) $got_instance_vars = true;
>>>>>>> origin/cpstrnew
					
					// get the protocols which the class adopts
					if (eregi($this->regex_objc_class, $line, $captures)) {
						if ($captures[3]) $this->dump[$file_name]["classes"][$current]["adopts"] = $captures[3];
					} else {
						if ($captures[2]) $this->dump[$file_name]["classes"][$current]["adopts"] = $captures[2];
						$has_superclass=false;
					}
					
					// clean up the conforms string
					if ($this->dump[$file_name]["classes"][$current]["adopts"]) {
						$conform_protocols = explode(",", $this->dump[$file_name]["classes"][$current]["adopts"]);
						$protocol_list = array();
						
						foreach ($conform_protocols as $protocol) {
							$protocol = trim($protocol, "<> ");
							$protocol_clean .= $protocol."$this->protocol_suffix, ";
							$protocol_list[] = $protocol;
						}
						
						$protocol_clean = trim($protocol_clean, ", ");
						$this->dump[$file_name]["classes"][$current]["adopts"] = $protocol_clean;
						$this->dump[$file_name]["classes"][$current]["protocols"] = $protocol_list;

						$protocol_clean = "";
					}
					
					$this->dump[$file_name]["classes"][$current]["name"] = $captures[1];
					if ($has_superclass) {
						$this->dump[$file_name]["classes"][$current]["super"] = $captures[2];
						$this->dump[$file_name]["classes"][$current]["super_class"] = &$this->dump["master"][$captures[2]];
					}
					$this->dump[$file_name]["classes"][$current]["file_name"] = $file_name;
					$this->dump[$file_name]["classes"][$current]["file_clean"] = substr($file_name, 0, (strripos($file_name, ".")));
					$this->dump[$file_name]["classes"][$current]["protected_keywords"] = array();
					$this->dump[$file_name]["classes"][$current]["declared_methods"] = array();
					$this->dump[$file_name]["classes"][$current]["comment"] = $this->InsertCurrentComment();
					
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
			
			// get the framework we're parsing from if it was specified during a batch parse
			if ((!$this->framework) && (eregi("/([a-zA-Z]+)\.framework", $file, $captures))) $this->framework = strtolower($captures[1]);
			
			// set the dump for the header
			$this->dump[$file_name]["path"] = "$this->root$this->out/$this->framework/$name_clean.inc";
			$this->dump[$file_name]["path_partial"] = "$this->framework/$name_clean.inc";
			$this->dump[$file_name]["path_merge"] = $this->dump[$file_name]["path"].".merge";
			$this->dump[$file_name]["framework"] = $this->framework;
			$this->dump[$file_name]["name"] = $file_name;
			$this->dump[$file_name]["name_clean"] = $name_clean;
			$this->dump[$file_name]["anoninternstrucs"] = 0;
			$this->dump[$file_name]["anonbitfields"] = 0;
			$this->current_header = &$this->dump[$file_name];
			
			// parse each section of the header
			$this->ParseHeaderTypes($file);
			$this->ParseHeaderProtocols($file);
			$this->ParseHeaderClasses($file);
			
			print("+ Parsed $file_name\n");
	}
	
	// Parses the docset at $path for the current framework
	function ParseFrameworkDocset ($path) {
		$name = basename($path);

		$parser = new DocSetParser($path);
		if ($parser->parse_directory($this->docset_paths[$name])) {
			$this->docset = $parser->methods;
			print("+ Parsed documentation for $name.\n");
		}
		
	}		
			
	// Parse all headers assigned in $this->frameworks
	function ParseAllFrameworks ($ignore_files, $parse_only) {
		
		foreach ($this->frameworks as $framework_name => $framework_info) {
			
			// framework is disabled
			if ($framework_info["enabled"] != 1) continue;
			
			// set the current framework being parsed
			$this->framework = $framework_name;

			// get the root file path
			if ($this->out != "/") {
				$path = $this->root.$this->out."/".$framework_info["root"];
			} else {
				$path = $this->root.$framework_info["root"];
			}
			
			// Parse the framework docset
			if ($this->parse_docsets) $this->ParseFrameworkDocset($framework_info["docset"]);
			
			// Load the header if found
			if (file_exists($path)) {
				$contents = file_get_contents($path);
				$lines = explode("\n", $contents);

				foreach ($lines as $line) {
					
					$header = null;
					$path = null;

					// parse the header path from the {$include} macro
					if (eregi($framework_info["include_pattern"], $line, $captures)) {
						$header = $captures[1].".h";
						$path = $framework_info["headers"]."/$header";
					}

					// parse the header path from {-parse} directive
					if (eregi("^\{-parse[[:space:]]+(.*)[[:space:]]*\}", $line, $captures)) {
						$header = $captures[1];
						$path = $framework_info["headers"]."/$header";
					}
					
					// parse the header if valid
					if (file_exists($path)) {
						
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
						
					} elseif ($header) {
						print("*** The header $path could not be found. ***\n");
						$this->warning_count++;
					}
				}
			} else {
				die("FATAL: The master include \"$path\" is missing.\n");
			}
		}
		
		// diagnostics
		print("\n• Parsed $this->method_count methods in $this->class_count classes.\n\n");
		
		if ($this->warning_count > 0) print("• $this->warning_count warnings were encountered.\n\n");
	}
		
	/**
	 * MAIN METHODS
	 */
						
	// Parse all classes/categories (non-delegate) from the header
	function CollectHeaderClasses ($file) {
		
		// can't find the header, bail
		if (!file_exists($file)) return;
			
		$contents = file_get_contents($file);
			
		$lines = explode("\n", $contents);
		foreach ($lines as $line) {
			
			// remove external class macros
			$line = eregi_replace("^[A-Z0-9]+_EXTERN_CLASS[[:space:]]+", "", $line);
			// remove version macro's (some can appear before a class)
			$line = $this->RemoveVersionMacros($line, $dummy_deprecated_mods);
			
			// classes
			if (eregi($this->regex_objc_class, $line, $captures)) {
				$this->defined_cocoa_classes[] = $captures[1];
				// may already have been parsed as an anonymous class
				if (!in_array($captures[1], $this->cocoa_classes))
				  $this->cocoa_classes[] = $captures[1];
			}
			if (eregi($this->regex_objc_class_no_super, $line, $captures)) {
				$this->defined_cocoa_classes[] = $captures[1];
				// may already have been parsed as an anonymous class
				if (!in_array($captures[1], $this->cocoa_classes))
				  $this->cocoa_classes[] = $captures[1];
			}
			// anonymous classes ===
			if (eregi($this->regex_objc_anon_class, $line, $captures)) {
				$anon_classes = explode(",", $captures[1]);
				foreach ($anon_classes as $anon_class) {
					$anon_class=trim($anon_class);
					// may already have been parsed as a regular class
					if (!in_array($anon_class, $this->cocoa_classes)) {
						$this->cocoa_classes[] = $anon_class;
						if (!in_array($anon_class, $this->anon_cocoa_classes)) {
							$this->anon_cocoa_classes[] = $anon_class;
						}
					}
				}
			}
				
			// categories
			if (eregi($this->regex_objc_category, $line, $captures)) {
				$this->cocoa_categories[] = $captures[1];
			}
		}
	}
		
	// Build array of all known classes in frameworks	
	function BuildFrameworkClasses () {
	
		foreach ($this->frameworks as $framework_name => $framework_info) {
			
			// framework is disabled
			if ($framework_info["enabled"] != 1) continue;
			
			if ($handle = @opendir($framework_info["headers"])) {
				while (($file = readdir($handle)) !== false) {
					if (eregi($framework_info["header_pattern"], $file)) {
						$this->CollectHeaderClasses($framework_info["headers"]."/$file");
					}
				}
				closedir($handle);
			} else {
				die("FATAL: The framework \"$framework_name\" can not be located at ".$framework_info["headers"]);
			}
			
		}
	}		

	// Process a single and print output
	function ProcessFile ($file, $print) {
		
		// set the current framework to null so it's parsed from the framework
		$this->framework = null;
		
		$this->ParseHeader($file);
		$this->ParseHeaderDependents($file);

		if ($print) $this->PrintAllHeaders("", null, null, false);
	}
	
	// Loads parser settings from the XML file
	function LoadFrameworksXML ($framework_path) {
		$xml = new SimpleXMLElement(file_get_contents("frameworks.xml"));
		
		foreach ($xml as $framework) {
				$this->frameworks[(string) $framework->name]["root"] = (string) $framework->root;
				$this->frameworks[(string) $framework->name]["headers"] = (string) $framework->headers;
				if ($framework_path != "")
					$this->frameworks[(string) $framework->name]["headers"] = preg_replace("!^.*/System/Library/Frameworks!", $framework_path, $this->frameworks[(string) $framework->name]["headers"]);
				$this->frameworks[(string) $framework->name]["include_pattern"] = (string) $framework->include_pattern;
				$this->frameworks[(string) $framework->name]["header_pattern"] = (string) $framework->header_pattern;
				$this->frameworks[(string) $framework->name]["external_macro"] = (string) $framework->external_macro;
				$this->frameworks[(string) $framework->name]["ignore_types"] = (string) $framework->ignore_types;
				$this->frameworks[(string) $framework->name]["ignore_methods"] = (string) $framework->ignore_methods;
				$this->frameworks[(string) $framework->name]["replace_types"] = $framework->replace_types;
				$this->frameworks[(string) $framework->name]["ignore_lines"] = $framework->ignore_lines;
				$this->frameworks[(string) $framework->name]["ignore_comments"] = $framework->ignore_comments;
				$this->frameworks[(string) $framework->name]["docset"] = (string)$framework->docset;
				$this->frameworks[(string) $framework->name]["enabled"] = false;
				$this->frameworks[(string) $framework->name]["print"] = true;
		}
		
	}

	function __construct ($directory, $out_directory, $frameworks, $frameworks_path, $show)  {
		$this->root = $directory;
		$this->out = $out_directory;
		$this->show = $show;
		
		// load defined frameworks from xml
		$this->LoadFrameworksXML($frameworks_path);
		
		// enable frameworks requested by the command line options
		if ($frameworks) {
			foreach ($frameworks as $name) {
				$name_clean = trim($name, "^ ");
				$this->frameworks[$name_clean]["enabled"] = true;
				
				// apply options from framework definition
				if ($this->frameworks[$name_clean]["external_macro"]) $this->external_string_macros .= "|".$this->frameworks[$name_clean]["external_macro"];
				
				if ($this->frameworks[$name_clean]["ignore_types"]) $this->ignore_types = array_merge($this->ignore_types, explode(",", $this->frameworks[$name_clean]["ignore_types"]));
				if ($this->frameworks[$name_clean]["ignore_methods"]) $this->ignore_methods = array_merge($this->ignore_methods, explode(",", $this->frameworks[$name_clean]["ignore_methods"]));

				if ($this->frameworks[$name_clean]["ignore_lines"]) {
					foreach ($this->frameworks[$name_clean]["ignore_lines"]->line as $line) {
						if (!in_array($line, $this->ignore_lines)) $this->ignore_lines[] = (string)$line;
					}
				}

				if ($this->frameworks[$name_clean]["ignore_comments"]) {
					foreach ($this->frameworks[$name_clean]["ignore_comments"]->line as $line) {
						if (!in_array($line, $this->ignore_comments)) $this->ignore_comments[] = (string)$line;
					}
				}

				if ($this->frameworks[$name_clean]["replace_types"]) {
					foreach ($this->frameworks[$name_clean]["replace_types"]->type as $type) {
						$pair = explode("=", (string)$type);
						$this->replace_types[$pair[0]] = $pair[1];
					}
				}
				
				// print mode
				if ($name[0] == "^") $this->frameworks[$name_clean]["print"] = false;
			}
		}
		
		//print_r($this->ignore_comments);
		//print_r($this->ignore_lines);
		//print_r($this->frameworks);
		//exit;
		$this->BuildFrameworkClasses();
	}

}

?>