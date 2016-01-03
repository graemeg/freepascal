unit aspell;

{ * This file is header translation of The New Aspell
  * Copyright (C) 2001-2002 by Kevin Atkinson under the GNU LGPL
  * license version 2.0 or 2.1.  You should have received a copy of the
  * LGPL license along with this library if you did not you can find it
  * at http://www.gnu.org/.                                              * }

{ * Translation to pascal (c) 2008 by Aleš Katona. * }

{$PACKRECORDS C}

interface

uses
  cTypes;

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF UNIX}
  const libaspell = 'aspell';
{$ELSE} 
 {$IFDEF WINDOWS}
  const libaspell = 'aspell-15.dll';
 {$ELSE} 
  {$MESSAGE ERROR Target not supported'}
 {$ENDIF}
{$ENDIF}

  {$i aspelltypes.inc}
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF Linux}
  const libaspell = '/usr/lib/libaspell.so';
{$ENDIF}

{$IFDEF FreeBSD}
  const libaspell = '/usr/local/lib/libaspell.so';
{$ENDIF}

{$IFDEF darwin}
 const libaspell = '/opt/local/lib/libaspell.dylib';
{$ENDIF}

{$IFDEF windows}
 const libaspell = 'aspell-%s.dll';
{$ENDIF}

{$IFDEF BeOS}
 const libaspell = '/boot/home/config/lib/libaspell.so';
{$ENDIF}

{$IFDEF Solaris}
 const libaspell = '/opt/csw/lib/libpspell.so.15';
{$ENDIF}

{$IFDEF SkyOS}
 {$DEFINE STATIC_ASPELL}
 {$LINKLIB aspell}
 const libaspell = 'aspell';
{$ENDIF}

    type
      PAspellCanHaveError  = Pointer;
      PAspellConfig  = Pointer;
      PAspellDictInfoEnumeration  = Pointer;
      PAspellDictInfoList  = Pointer;
      PAspellDocumentChecker  = Pointer;
      PAspellFilter  = Pointer;
      PAspellKeyInfoEnumeration  = Pointer;
      PAspellModuleInfoEnumeration  = Pointer;
      PAspellModuleInfoList  = Pointer;
      PAspellMutableContainer  = Pointer;
      PAspellSpeller  = Pointer;
      PAspellStringEnumeration  = Pointer;
      PAspellStringList  = Pointer;
      PAspellStringMap  = Pointer;
      PAspellStringPairEnumeration  = Pointer;
      PAspellWordList  = Pointer;

  {****************************** type id ****************************** }

   type
     PAspellTypeId = ^AspellTypeId;
     AspellTypeId = record
         case longint of
            0 : ( num : cuint );
            1 : ( str : array[0..3] of char );
         end;
         
    {****************************** key info ****************************** }

       PAspellKeyInfoType = ^AspellKeyInfoType;
       AspellKeyInfoType = (AspellKeyInfoString,AspellKeyInfoInt,
         AspellKeyInfoBool,AspellKeyInfoList
         );

    { A brief description of the key or NULL if internal value.  }

       PAspellKeyInfo = ^AspellKeyInfo;
       AspellKeyInfo = record
            name : pchar;
            _type : AspellKeyInfoType;
            def : pchar;
            desc : pchar;
            flags : cint;
            other_data : cint;
         end;

    {****************************** error ****************************** }
  
       PAspellErrorInfo = ^AspellErrorInfo;
       AspellErrorInfo = record
            isa : PAspellErrorInfo;
            mesg : pchar;
            num_parms : cuint;
            parms : array[0..2] of pchar;
         end;
         
       PAspellError = ^AspellError;
       AspellError = record
            mesg : pchar;
            err : PAspellErrorInfo;
         end;
         
    {****************************** token ****************************** }

       PAspellToken = ^AspellToken;
       AspellToken = record
            offset : cuint;
            len : cuint;
         end;

    {*************************** module/dict *************************** }

       PAspellModuleInfo = ^AspellModuleInfo;
       AspellModuleInfo = record
            name : pchar;
            order_num : double;
            lib_dir : pchar;
            dict_dirs : PAspellStringList;
            dict_exts : PAspellStringList;
         end;

    { The Name to identify this dictionary by.  }

    { The language code to identify this dictionary.
       * A two letter UPPER-CASE ISO 639 language code
       * and an optional two letter ISO 3166 country
       * code after a dash or underscore.  }

    { Any extra information to distinguish this
       * variety of dictionary from other dictionaries
       * which may have the same language and size.  }

    { A two char digit code describing the size of
       * the dictionary: 10=tiny, 20=really small,
       * 30=small, 40=med-small, 50=med, 60=med-large,
       * 70=large, 80=huge, 90=insane.  Please check
       * the README in aspell-lang-200?????.tar.bz2 or
       * see SCOWL (http://wordlist.sourceforge.net)
       * for an example of how these sizes are used.  }

       PAspellDictInfo = ^AspellDictInfo;
       AspellDictInfo = record
            name : pchar;
            code : pchar;
            jargon : pchar;
            size : cint;
            size_str : pchar;
            module : PAspellModuleInfo;
         end;

  {**************************** string pair **************************** }

       PAspellStringPair = ^AspellStringPair;
       AspellStringPair = record
            first : pchar;
            second : pchar;
         end;
         
{$IFDEF STATIC_ASPELL}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  {************************* mutable container ************************* }

  function aspell_mutable_container_add(ths:PAspellMutableContainer; to_add:pchar):cint;cdecl;external libaspell name 'aspell_mutable_container_add';

  function aspell_mutable_container_remove(ths:PAspellMutableContainer; to_rem:pchar):cint;cdecl;external libaspell name 'aspell_mutable_container_remove';

  procedure aspell_mutable_container_clear(ths:PAspellMutableContainer);cdecl;external libaspell name 'aspell_mutable_container_clear';

  function aspell_mutable_container_to_mutable_container(ths:PAspellMutableContainer):PAspellMutableContainer; cdecl;external libaspell name 'aspell_mutable_container_to_mutable_container';

    {******************************* config ******************************* }

    function aspell_key_info_enumeration_at_end(ths:PAspellKeyInfoEnumeration):cint;cdecl;external libaspell name 'aspell_key_info_enumeration_at_end';

    function aspell_key_info_enumeration_next(ths:PAspellKeyInfoEnumeration):PAspellKeyInfo;cdecl;external libaspell name 'aspell_key_info_enumeration_next';

    procedure delete_aspell_key_info_enumeration(ths:PAspellKeyInfoEnumeration);cdecl;external libaspell name 'delete_aspell_key_info_enumeration';

    function aspell_key_info_enumeration_clone(ths:PAspellKeyInfoEnumeration):PAspellKeyInfoEnumeration; cdecl;external libaspell name 'aspell_key_info_enumeration_clone';

    procedure aspell_key_info_enumeration_assign(ths:PAspellKeyInfoEnumeration; other:PAspellKeyInfoEnumeration);cdecl;external libaspell name 'aspell_key_info_enumeration_assign';

    function new_aspell_config():PAspellConfig;cdecl;external libaspell name 'new_aspell_config';

    procedure delete_aspell_config(ths:PAspellConfig);cdecl;external libaspell name 'delete_aspell_config';

    function aspell_config_clone(ths:PAspellConfig):PAspellConfig;cdecl;external libaspell name 'aspell_config_clone';

    procedure aspell_config_assign(ths:PAspellConfig; other:PAspellConfig);cdecl;external libaspell name 'aspell_config_assign';

    function aspell_config_error_number(ths:PAspellConfig):cuint;cdecl;external libaspell name 'aspell_config_error_number';

    function aspell_config_error_message(ths:PAspellConfig):pchar;cdecl;external libaspell name 'aspell_config_error_message';

    function aspell_config_error(ths:PAspellConfig):PAspellError;cdecl;external libaspell name 'aspell_config_error';

    { Sets extra keys which this config class should
     * accept. begin and end are expected to point to
     * the beginning and ending of an array of Aspell
     * Key Info.  }

    procedure aspell_config_set_extra(ths:PAspellConfig; b:PAspellKeyInfo; e:PAspellKeyInfo);cdecl;external libaspell name 'aspell_config_set_extra';

    { Returns the KeyInfo object for the
     * corresponding key or returns NULL and sets
     * error_num to PERROR_UNKNOWN_KEY if the key is
     * not valid. The pointer returned is valid for
     * the lifetime of the object.  }

    function aspell_config_keyinfo(ths:PAspellConfig; key:pchar):PAspellKeyInfo;cdecl;external libaspell name 'aspell_config_keyinfo';

    { Returns a newly allocated enumeration of all
     * the possible objects this config class uses.  }

    function aspell_config_possible_elements(ths:PAspellConfig; include_extra:cint):PAspellKeyInfoEnumeration;cdecl;external libaspell name 'aspell_config_possible_elements';

    { Returns the default value for given key which
     * may involve substituting variables, thus it is
     * not the same as keyinfo(key)->def returns NULL
     * and sets error_num to PERROR_UNKNOWN_KEY if
     * the key is not valid. Uses the temporary
     * string.  }

    function aspell_config_get_default(ths:PAspellConfig; key:pchar):pchar;cdecl;external libaspell name 'aspell_config_get_default';

    { Returns a newly allocated enumeration of all
     * the key/value pairs. This DOES not include ones
     * which are set to their default values.  }

    function aspell_config_elements(ths:PAspellConfig):PAspellStringPairEnumeration;cdecl;external libaspell name 'aspell_config_elements';

    { Inserts an item, if the item already exists it
     * will be replaced. Returns TRUE if it succeeded
     * or FALSE on error. If the key is not valid it
     * sets error_num to PERROR_UNKNOWN_KEY, if the
     * value is not valid it will set error_num to
     * PERROR_BAD_VALUE, if the value can not be
     * changed it sets error_num to
     * PERROR_CANT_CHANGE_VALUE, and if the value is
     * a list and you are trying to set its directory,
     * it sets error_num to PERROR_LIST_SET  }

    function aspell_config_replace(ths:PAspellConfig; key:pchar; value:pchar):cint;cdecl;external libaspell name 'aspell_config_replace';

    { Remove a key and returns TRUE if it exists
     * otherwise return FALSE. This effectively sets
     * the key to its default value. Calling replace
     * with a value of "<default>" will also call
     * remove. If the key does not exist then it sets
     * error_num to 0 or PERROR_NOT, if the key is
     * not valid then it sets error_num to
     * PERROR_UNKNOWN_KEY, if the value can not be
     * changed then it sets error_num to
     * PERROR_CANT_CHANGE_VALUE  }

    function aspell_config_remove(ths:PAspellConfig; key:pchar):cint;cdecl;external libaspell name 'aspell_config_remove';

    function aspell_config_have(ths:PAspellConfig; key:pchar):cint;cdecl;external libaspell name 'aspell_config_have';

    { Returns NULL on error.  }

    function aspell_config_retrieve(ths:PAspellConfig; key:pchar):pchar;cdecl;external libaspell name 'aspell_config_retrieve';

    function aspell_config_retrieve_list(ths:PAspellConfig; key:pchar; lst:PAspellMutableContainer):cint;cdecl;external libaspell name 'aspell_config_retrieve_list';

    { In "ths" Aspell configuration, search for a
     * character string matching "key" string.
     * If "key" is found then return 1 else return 0.
     * If error encountered, then return -1.  }

    function aspell_config_retrieve_bool(ths:PAspellConfig; key:pchar):cint;cdecl;external libaspell name 'aspell_config_retrieve_bool';

    { In "ths" Aspell configuration, search for an
     * integer value matching "key" string.
     * Return -1 on error.  }

    function aspell_config_retrieve_int(ths:PAspellConfig; key:pchar):cint;cdecl;external libaspell name 'aspell_config_retrieve_int';

    {******************************* error ******************************* }

    function aspell_error_is_a(ths:PAspellError; e:PAspellErrorInfo):cint;cdecl;external libaspell name 'aspell_error_is_a';

    {*************************** can have error *************************** }

    function aspell_error_number(ths:PAspellCanHaveError):cuint;cdecl;external libaspell name 'aspell_error_number';

    function aspell_error_message(ths:PAspellCanHaveError):pchar;cdecl;external libaspell name 'aspell_error_message';

    function aspell_error(ths:PAspellCanHaveError):PAspellError;cdecl;external libaspell name 'aspell_error';

    procedure delete_aspell_can_have_error(ths:PAspellCanHaveError);cdecl;external libaspell name 'delete_aspell_can_have_error';

    {******************************* errors ******************************* }

    // ignored

    {****************************** speller ****************************** }

    function new_aspell_speller(config:PAspellConfig):PAspellCanHaveError;cdecl;external libaspell name 'new_aspell_speller';

    function to_aspell_speller(obj:PAspellCanHaveError):PAspellSpeller;cdecl;external libaspell name 'to_aspell_speller';

    procedure delete_aspell_speller(ths:PAspellSpeller);cdecl;external libaspell name 'delete_aspell_speller';

    function aspell_speller_error_number(ths:PAspellSpeller):cuint;cdecl;external libaspell name 'aspell_speller_error_number';

    function aspell_speller_error_message(ths:PAspellSpeller):pchar;cdecl;external libaspell name 'aspell_speller_error_message';

    function aspell_speller_error(ths:PAspellSpeller):PAspellError;cdecl;external libaspell name 'aspell_speller_error';

    function aspell_speller_config(ths:PAspellSpeller):PAspellConfig;cdecl;external libaspell name 'aspell_speller_config';
    { Returns 0 if it is not in the dictionary,
     * 1 if it is, or -1 on error.  }

    function aspell_speller_check(ths:PAspellSpeller; word:pchar; word_size:cint):cint;cdecl;external libaspell name 'aspell_speller_check';

    { Add this word to your own personal word list.  }

    function aspell_speller_add_to_personal(ths:PAspellSpeller; word:pchar; word_size:cint):cint;cdecl;external libaspell name 'aspell_speller_add_to_personal';

    { Add this word to the current spelling session.  }

    function aspell_speller_add_to_session(ths:PAspellSpeller; word:pchar; word_size:cint):cint;cdecl;external libaspell name 'aspell_speller_add_to_session';

    { This is your own personal word list file plus
     * any extra words added during this session to
     * your own personal word list.  }

    function aspell_speller_personal_word_list(ths:PAspellSpeller):PAspellWordList;cdecl;external libaspell name 'aspell_speller_personal_word_list';

    { This is a list of words added to this session
     * that are not in the main word list or in your
     * own personal list but are considered valid for
     * this spelling session.  }

    function aspell_speller_session_word_list(ths:PAspellSpeller):PAspellWordList;cdecl;external libaspell name 'aspell_speller_session_word_list';

    { This is the main list of words used during this
     * spelling session.  }

    function aspell_speller_main_word_list(ths:PAspellSpeller):PAspellWordList;cdecl;external libaspell name 'aspell_speller_main_word_list';

    function aspell_speller_save_all_word_lists(ths:PAspellSpeller):cint;cdecl;external libaspell name 'aspell_speller_save_all_word_lists';

    function aspell_speller_clear_session(ths:PAspellSpeller):cint;cdecl;external libaspell name 'aspell_speller_clear_session';

    { Return NULL on error.
     * The word list returned by suggest is only
     * valid until the next call to suggest.  }

    function aspell_speller_suggest(ths:PAspellSpeller; word:pchar; word_size:cint):PAspellWordList;cdecl;external libaspell name 'aspell_speller_suggest';

    function aspell_speller_store_replacement(ths:PAspellSpeller; mis:pchar; mis_size:cint; cor:pchar; cor_size:cint):cint;cdecl;external libaspell name 'aspell_speller_store_replacement';

    {******************************* filter ******************************* }

    procedure delete_aspell_filter(ths:PAspellFilter);cdecl;external libaspell name 'delete_aspell_filter';

    function aspell_filter_error_number(ths:PAspellFilter):cuint;cdecl;external libaspell name 'aspell_filter_error_number';

    function aspell_filter_error_message(ths:PAspellFilter):pchar;cdecl;external libaspell name 'aspell_filter_error_message';

    function aspell_filter_error(ths:PAspellFilter):PAspellError;cdecl;external libaspell name 'aspell_filter_error';

    function to_aspell_filter(obj:PAspellCanHaveError):PAspellFilter;cdecl;external libaspell name 'to_aspell_filter';

    {************************** document checker ************************** }

    procedure delete_aspell_document_checker(ths:PAspellDocumentChecker);cdecl;external libaspell name 'delete_aspell_document_checker';

    function aspell_document_checker_error_number(ths:PAspellDocumentChecker):cuint;cdecl;external libaspell name 'aspell_document_checker_error_number';

    function aspell_document_checker_error_message(ths:PAspellDocumentChecker):pchar;cdecl;external libaspell name 'aspell_document_checker_error_message';

    function aspell_document_checker_error(ths:PAspellDocumentChecker):PAspellError;cdecl;external libaspell name 'aspell_document_checker_error';

    { Creates a new document checker.
     * The speller class is expected to last until
     * this class is destroyed.
     * If config is given it will be used to override
     * any relevent options set by this speller class.
     * The config class is not once this function is done.
     * If filter is given then it will take ownership of
     * the filter class and use it to do the filtering.
     * You are expected to free the checker when done.  }

    function new_aspell_document_checker(speller: PAspellSpeller): PAspellCanHaveError;cdecl;external libaspell name 'new_aspell_document_checker';

    function to_aspell_document_checker(obj:PAspellCanHaveError):PAspellDocumentChecker;cdecl;external libaspell name 'to_aspell_document_checker';

    { Reset the internal state of the filter.
     * Should be called whenever a new document is
     * being filtered.  }
    procedure aspell_document_checker_reset(ths:PAspellDocumentChecker);cdecl;external libaspell name 'aspell_document_checker_reset';

    { Process a string.
     * The string passed in should only be split on
     * white space characters.  Furthermore, between
     * calls to reset, each string should be passed
     * in exactly once and in the order they appeared
     * in the document.  Passing in strings out of
     * order, skipping strings or passing them in
     * more than once may lead to undefined results.  }

    procedure aspell_document_checker_process(ths:PAspellDocumentChecker; str:pchar; size:cint);cdecl;external libaspell name 'aspell_document_checker_process';

    { Returns the next misspelled word in the
     * processed string.  If there are no more
     * misspelled words, then token.word will be
     * NULL and token.size will be 0  }

    // internal hacky version to go around a bug regarding struct results/cdecl
<<<<<<< HEAD
<<<<<<< HEAD
    function aspell_document_checker_next_misspelling(ths:PAspellDocumentChecker):AspellToken;cdecl;external libaspell name 'aspell_document_checker_next_misspelling';
=======
    function __aspell_document_checker_next_misspelling(ths:PAspellDocumentChecker):{$IFDEF CPU64}{$IFDEF LINUX}QWord{$ELSE}AspellToken{$ENDIF}{$ELSE}AspellToken{$ENDIF};cdecl;external libaspell name 'aspell_document_checker_next_misspelling';
>>>>>>> graemeg/fixes_2_2
=======
    function __aspell_document_checker_next_misspelling(ths:PAspellDocumentChecker):{$IFDEF CPU64}{$IFDEF LINUX}QWord{$ELSE}AspellToken{$ENDIF}{$ELSE}AspellToken{$ENDIF};cdecl;external libaspell name 'aspell_document_checker_next_misspelling';
>>>>>>> origin/fixes_2_2

    { Returns the underlying filter class.  }

    function aspell_document_checker_filter(ths:PAspellDocumentChecker):PAspellFilter;cdecl;external libaspell name 'aspell_document_checker_filter';

    {***************************** word list ***************************** }

    function aspell_word_list_empty(ths:PAspellWordList):cint;cdecl;external libaspell name 'aspell_word_list_empty';

    function aspell_word_list_size(ths:PAspellWordList):cuint;cdecl;external libaspell name 'aspell_word_list_size';

    function aspell_word_list_elements(ths:PAspellWordList):PAspellStringEnumeration;cdecl;external libaspell name 'aspell_word_list_elements';

    {************************* string enumeration ************************* }

    procedure delete_aspell_string_enumeration(ths:PAspellStringEnumeration);cdecl;external libaspell name 'delete_aspell_string_enumeration';

    function aspell_string_enumeration_clone(ths:PAspellStringEnumeration):PAspellStringEnumeration;cdecl;external libaspell name 'aspell_string_enumeration_clone';

    procedure aspell_string_enumeration_assign(ths:PAspellStringEnumeration; other:PAspellStringEnumeration);cdecl;external libaspell name 'aspell_string_enumeration_assign';

    function aspell_string_enumeration_at_end(ths:PAspellStringEnumeration):cint;cdecl;external libaspell name 'aspell_string_enumeration_at_end';

    function aspell_string_enumeration_next(ths:PAspellStringEnumeration):pchar;cdecl;external libaspell name 'aspell_string_enumeration_next';

    {******************************** info ******************************** }

    function get_aspell_module_info_list(config:PAspellConfig):PAspellModuleInfoList;cdecl;external libaspell name 'get_aspell_module_info_list';

    function aspell_module_info_list_empty(ths:PAspellModuleInfoList):cint;cdecl;external libaspell name 'aspell_module_info_list_empty';

    function aspell_module_info_list_size(ths:PAspellModuleInfoList):cuint;cdecl;external libaspell name 'aspell_module_info_list_size';

    function aspell_module_info_list_elements(ths:PAspellModuleInfoList):PAspellModuleInfoEnumeration;cdecl;external libaspell name 'aspell_module_info_list_elements';

    function get_aspell_dict_info_list(config:PAspellConfig):PAspellDictInfoList;cdecl;external libaspell name 'get_aspell_dict_info_list';

    function aspell_dict_info_list_empty(ths:PAspellDictInfoList):cint;cdecl;external libaspell name 'aspell_dict_info_list_empty';

    function aspell_dict_info_list_size(ths:PAspellDictInfoList):cuint;cdecl;external libaspell name 'aspell_dict_info_list_size';

    function aspell_dict_info_list_elements(ths:PAspellDictInfoList):PAspellDictInfoEnumeration;cdecl;external libaspell name 'aspell_dict_info_list_elements';

    function aspell_module_info_enumeration_at_end(ths:PAspellModuleInfoEnumeration):cint;cdecl;external libaspell name 'aspell_module_info_enumeration_at_end';

    function aspell_module_info_enumeration_next(ths:PAspellModuleInfoEnumeration):PAspellModuleInfo;cdecl;external libaspell name 'aspell_module_info_enumeration_next';

    procedure delete_aspell_module_info_enumeration(ths:PAspellModuleInfoEnumeration);cdecl;external libaspell name 'delete_aspell_module_info_enumeration';

    function aspell_module_info_enumeration_clone(ths:PAspellModuleInfoEnumeration):PAspellModuleInfoEnumeration;cdecl;external libaspell name 'aspell_module_info_enumeration_clone';

    procedure aspell_module_info_enumeration_assign(ths:PAspellModuleInfoEnumeration; other:PAspellModuleInfoEnumeration);cdecl;external libaspell name 'aspell_module_info_enumeration_assign';

    function aspell_dict_info_enumeration_at_end(ths:PAspellDictInfoEnumeration):cint;cdecl;external libaspell name 'aspell_dict_info_enumeration_at_end';

    function aspell_dict_info_enumeration_next(ths:PAspellDictInfoEnumeration):PAspellDictInfo;cdecl;external libaspell name 'aspell_dict_info_enumeration_next';

    procedure delete_aspell_dict_info_enumeration(ths:PAspellDictInfoEnumeration);cdecl;external libaspell name 'delete_aspell_dict_info_enumeration';

    function aspell_dict_info_enumeration_clone(ths:PAspellDictInfoEnumeration):PAspellDictInfoEnumeration;cdecl;external libaspell name 'aspell_dict_info_enumeration_clone';

    procedure aspell_dict_info_enumeration_assign(ths:PAspellDictInfoEnumeration; other:PAspellDictInfoEnumeration);cdecl;external libaspell name 'aspell_dict_info_enumeration_assign';

    {**************************** string list **************************** }

    function new_aspell_string_list():PAspellStringList;cdecl;external libaspell name 'new_aspell_string_list';

    function aspell_string_list_empty(ths:PAspellStringList):cint;cdecl;external libaspell name 'aspell_string_list_empty';

    function aspell_string_list_size(ths:PAspellStringList):cuint;cdecl;external libaspell name 'aspell_string_list_size';

    function aspell_string_list_elements(ths:PAspellStringList):PAspellStringEnumeration;cdecl;external libaspell name 'aspell_string_list_elements';

    function aspell_string_list_add(ths:PAspellStringList; to_add:pchar):cint;cdecl;external libaspell name 'aspell_string_list_add';

    function aspell_string_list_remove(ths:PAspellStringList; to_rem:pchar):cint;cdecl;external libaspell name 'aspell_string_list_remove';

    procedure aspell_string_list_clear(ths:PAspellStringList);cdecl;external libaspell name 'aspell_string_list_clear';

    function aspell_string_list_to_mutable_container(ths:PAspellStringList):PAspellMutableContainer;cdecl;external libaspell name 'aspell_string_list_to_mutable_container';

    procedure delete_aspell_string_list(ths:PAspellStringList);cdecl;external libaspell name 'delete_aspell_string_list';

    function aspell_string_list_clone(ths:PAspellStringList):PAspellStringList;cdecl;external libaspell name 'aspell_string_list_clone';

    procedure aspell_string_list_assign(ths:PAspellStringList; other:PAspellStringList);cdecl;external libaspell name 'aspell_string_list_assign';

    {***************************** string map ***************************** }

    function new_aspell_string_map():PAspellStringMap;cdecl;external libaspell name 'new_aspell_string_map';

    function aspell_string_map_add(ths:PAspellStringMap; to_add:pchar):cint;cdecl;external libaspell name 'aspell_string_map_add';

    function aspell_string_map_remove(ths:PAspellStringMap; to_rem:pchar):cint;cdecl;external libaspell name 'aspell_string_map_remove';

    procedure aspell_string_map_clear(ths:PAspellStringMap);cdecl;external libaspell name 'aspell_string_map_clear';

    function aspell_string_map_to_mutable_container(ths:PAspellStringMap):PAspellMutableContainer;cdecl;external libaspell name 'aspell_string_map_to_mutable_container';

    procedure delete_aspell_string_map(ths:PAspellStringMap);cdecl;external libaspell name 'delete_aspell_string_map';

    function aspell_string_map_clone(ths:PAspellStringMap):PAspellStringMap;cdecl;external libaspell name 'aspell_string_map_clone';

    procedure aspell_string_map_assign(ths:PAspellStringMap; other:PAspellStringMap);cdecl;external libaspell name 'aspell_string_map_assign';

    function aspell_string_map_empty(ths:PAspellStringMap):cint;cdecl;external libaspell name 'aspell_string_map_empty';

    function aspell_string_map_size(ths:PAspellStringMap):cuint;cdecl;external libaspell name 'aspell_string_map_size';

    function aspell_string_map_elements(ths:PAspellStringMap):PAspellStringPairEnumeration;cdecl;external libaspell name 'aspell_string_map_elements';

    { Insert a new element.
     * Will NOT overwrite an existing entry.
     * Returns FALSE if the element already exists.  }

    function aspell_string_map_insert(ths:PAspellStringMap; key:pchar; value:pchar):cint;cdecl;external libaspell name 'aspell_string_map_insert';

    { Insert a new element.
     * Will overwrite an existing entry.
     * Always returns TRUE.  }

    function aspell_string_map_replace(ths:PAspellStringMap; key:pchar; value:pchar):cint;cdecl;external libaspell name 'aspell_string_map_replace';

    { Looks up an element and returns the value.
     * Returns NULL if the element does not exist.
     * Returns an empty string if the element exists
     * but has a NULL value.  }

    function aspell_string_map_lookup(ths:PAspellStringMap; key:pchar):pchar;cdecl;external libaspell name 'aspell_string_map_lookup';

    {********************** string pair enumeration ********************** }

    function aspell_string_pair_enumeration_at_end(ths:PAspellStringPairEnumeration):cint;cdecl;external libaspell name 'aspell_string_pair_enumeration_at_end';

    function aspell_string_pair_enumeration_next(ths:PAspellStringPairEnumeration):AspellStringPair;cdecl;external libaspell name 'aspell_string_pair_enumeration_next';

    procedure delete_aspell_string_pair_enumeration(ths:PAspellStringPairEnumeration);cdecl;external libaspell name 'delete_aspell_string_pair_enumeration';

    function aspell_string_pair_enumeration_clone(ths:PAspellStringPairEnumeration):PAspellStringPairEnumeration;cdecl;external libaspell name 'aspell_string_pair_enumeration_clone';

    procedure aspell_string_pair_enumeration_assign(ths:PAspellStringPairEnumeration; other:PAspellStringPairEnumeration);cdecl;external libaspell name 'aspell_string_pair_enumeration_assign';

    {******************************* cache ******************************* }
    { Reset the global cache(s) so that cache queries will
     * create a new object. If existing objects are still in
     * use they are not deleted. If which is NULL then all
     * caches will be reset. Current caches are "encode",
     * "decode", "dictionary", "language", and "keyboard".  }

    function aspell_reset_cache(which:pchar):cint;cdecl;external libaspell name 'aspell_reset_cache';
    
<<<<<<< HEAD
<<<<<<< HEAD
  function aspell_init(const libn: ansistring): Boolean;
  function aspell_loaded: Boolean;

implementation

=======
=======
>>>>>>> origin/fixes_2_2
{$ELSE}

  {************************* mutable container ************************* }
var
  aspell_mutable_container_add: function(ths:PAspellMutableContainer; to_add:pchar):cint;cdecl;

  aspell_mutable_container_remove: function(ths:PAspellMutableContainer; to_rem:pchar):cint;cdecl;

  aspell_mutable_container_clear: procedure(ths:PAspellMutableContainer);cdecl;

  aspell_mutable_container_to_mutable_container: function(ths:PAspellMutableContainer):PAspellMutableContainer;cdecl;

      {******************************* config ******************************* }

  aspell_key_info_enumeration_at_end: function(ths:PAspellKeyInfoEnumeration):cint;cdecl;

  aspell_key_info_enumeration_next: function(ths:PAspellKeyInfoEnumeration):PAspellKeyInfo;cdecl;

  delete_aspell_key_info_enumeration: procedure(ths:PAspellKeyInfoEnumeration);cdecl;

  aspell_key_info_enumeration_clone: function(ths:PAspellKeyInfoEnumeration):PAspellKeyInfoEnumeration;cdecl;

  aspell_key_info_enumeration_assign: procedure(ths:PAspellKeyInfoEnumeration; other:PAspellKeyInfoEnumeration);cdecl;

  new_aspell_config: function():PAspellConfig;cdecl;

  delete_aspell_config: procedure(ths:PAspellConfig);cdecl;

  aspell_config_clone: function(ths:PAspellConfig):PAspellConfig;cdecl;

  aspell_config_assign: procedure(ths:PAspellConfig; other:PAspellConfig);cdecl;

  aspell_config_error_number: function(ths:PAspellConfig):cuint;cdecl;

  aspell_config_error_message: function(ths:PAspellConfig):pchar;cdecl;

  aspell_config_error: function(ths:PAspellConfig):PAspellError;cdecl;

      { Sets extra keys which this config class should
       * accept. begin and end are expected to point to
       * the beginning and ending of an array of Aspell
       * Key Info.  }

  aspell_config_set_extra: procedure(ths:PAspellConfig; b:PAspellKeyInfo; e:PAspellKeyInfo);cdecl;

      { Returns the KeyInfo object for the
       * corresponding key or returns NULL and sets
       * error_num to PERROR_UNKNOWN_KEY if the key is
       * not valid. The pointer returned is valid for
       * the lifetime of the object.  }

  aspell_config_keyinfo: function(ths:PAspellConfig; key:pchar):PAspellKeyInfo;cdecl;

      { Returns a newly allocated enumeration of all
       * the possible objects this config class uses.  }

  aspell_config_possible_elements: function(ths:PAspellConfig; include_extra:cint):PAspellKeyInfoEnumeration;cdecl;

      { Returns the default value for given key which
       * may involve substituting variables, thus it is
       * not the same as keyinfo(key)->def returns NULL
       * and sets error_num to PERROR_UNKNOWN_KEY if
       * the key is not valid. Uses the temporary
       * string.  }

  aspell_config_get_default: function(ths:PAspellConfig; key:pchar):pchar;cdecl;

      { Returns a newly allocated enumeration of all
       * the key/value pairs. This DOES not include ones
       * which are set to their default values.  }

  aspell_config_elements: function(ths:PAspellConfig):PAspellStringPairEnumeration;cdecl;

      { Inserts an item, if the item already exists it
       * will be replaced. Returns TRUE if it succeeded
       * or FALSE on error. If the key is not valid it
       * sets error_num to PERROR_UNKNOWN_KEY, if the
       * value is not valid it will set error_num to
       * PERROR_BAD_VALUE, if the value can not be
       * changed it sets error_num to
       * PERROR_CANT_CHANGE_VALUE, and if the value is
       * a list and you are trying to set its directory,
       * it sets error_num to PERROR_LIST_SET  }

  aspell_config_replace: function(ths:PAspellConfig; key:pchar; value:pchar):cint;cdecl;

      { Remove a key and returns TRUE if it exists
       * otherwise return FALSE. This effectively sets
       * the key to its default value. Calling replace
       * with a value of "<default>" will also call
       * remove. If the key does not exist then it sets
       * error_num to 0 or PERROR_NOT, if the key is
       * not valid then it sets error_num to
       * PERROR_UNKNOWN_KEY, if the value can not be
       * changed then it sets error_num to
       * PERROR_CANT_CHANGE_VALUE  }

  aspell_config_remove: function(ths:PAspellConfig; key:pchar):cint;cdecl;

  aspell_config_have: function(ths:PAspellConfig; key:pchar):cint;cdecl;

      { Returns NULL on error.  }

  aspell_config_retrieve: function(ths:PAspellConfig; key:pchar):pchar;cdecl;

  aspell_config_retrieve_list: function(ths:PAspellConfig; key:pchar; lst:PAspellMutableContainer):cint;cdecl;

      { In "ths" Aspell configuration, search for a
       * character string matching "key" string.
       * If "key" is found then return 1 else return 0.
       * If error encountered, then return -1.  }

  aspell_config_retrieve_bool: function(ths:PAspellConfig; key:pchar):cint;cdecl;

      { In "ths" Aspell configuration, search for an
       * integer value matching "key" string.
       * Return -1 on error.  }

  aspell_config_retrieve_int: function(ths:PAspellConfig; key:pchar):cint;cdecl;

      {******************************* error ******************************* }

  aspell_error_is_a: function(ths:PAspellError; e:PAspellErrorInfo):cint;cdecl;

      {*************************** can have error *************************** }

  aspell_error_number: function(ths:PAspellCanHaveError):cuint;cdecl;

  aspell_error_message: function(ths:PAspellCanHaveError):pchar;cdecl;

  aspell_error: function(ths:PAspellCanHaveError):PAspellError;cdecl;

  delete_aspell_can_have_error: procedure(ths:PAspellCanHaveError);cdecl;

      {******************************* errors ******************************* }

      // ignored

      {****************************** speller ****************************** }

  new_aspell_speller: function(config:PAspellConfig):PAspellCanHaveError;cdecl;

  to_aspell_speller: function(obj:PAspellCanHaveError):PAspellSpeller;cdecl;

  delete_aspell_speller: procedure(ths:PAspellSpeller);cdecl;

  aspell_speller_error_number: function(ths:PAspellSpeller):cuint;cdecl;

  aspell_speller_error_message: function(ths:PAspellSpeller):pchar;cdecl;

  aspell_speller_error: function(ths:PAspellSpeller):PAspellError;cdecl;

  aspell_speller_config: function(ths:PAspellSpeller):PAspellConfig;cdecl;
      { Returns 0 if it is not in the dictionary,
       * 1 if it is, or -1 on error.  }

  aspell_speller_check: function(ths:PAspellSpeller; word:pchar; word_size:cint):cint;cdecl;

      { Add this word to your own personal word list.  }

  aspell_speller_add_to_personal: function(ths:PAspellSpeller; word:pchar; word_size:cint):cint;cdecl;

      { Add this word to the current spelling session.  }

  aspell_speller_add_to_session: function(ths:PAspellSpeller; word:pchar; word_size:cint):cint;cdecl;

      { This is your own personal word list file plus
       * any extra words added during this session to
       * your own personal word list.  }

  aspell_speller_personal_word_list: function(ths:PAspellSpeller):PAspellWordList;cdecl;

      { This is a list of words added to this session
       * that are not in the main word list or in your
       * own personal list but are considered valid for
       * this spelling session.  }

  aspell_speller_session_word_list: function(ths:PAspellSpeller):PAspellWordList;cdecl;

      { This is the main list of words used during this
       * spelling session.  }

  aspell_speller_main_word_list: function(ths:PAspellSpeller):PAspellWordList;cdecl;

  aspell_speller_save_all_word_lists: function(ths:PAspellSpeller):cint;cdecl;

  aspell_speller_clear_session: function(ths:PAspellSpeller):cint;cdecl;

      { Return NULL on error.
       * The word list returned by suggest is only
       * valid until the next call to suggest.  }

  aspell_speller_suggest: function(ths:PAspellSpeller; word:pchar; word_size:cint):PAspellWordList;cdecl;

  aspell_speller_store_replacement: function(ths:PAspellSpeller; mis:pchar; mis_size:cint; cor:pchar; cor_size:cint):cint;cdecl;

      {******************************* filter ******************************* }

  delete_aspell_filter: procedure(ths:PAspellFilter);cdecl;

  aspell_filter_error_number: function(ths:PAspellFilter):cuint;cdecl;

  aspell_filter_error_message: function(ths:PAspellFilter):pchar;cdecl;

  aspell_filter_error: function(ths:PAspellFilter):PAspellError;cdecl;

  to_aspell_filter: function(obj:PAspellCanHaveError):PAspellFilter;cdecl;

      {************************** document checker ************************** }

  delete_aspell_document_checker: procedure(ths:PAspellDocumentChecker);cdecl;

  aspell_document_checker_error_number: function(ths:PAspellDocumentChecker):cuint;cdecl;

  aspell_document_checker_error_message: function(ths:PAspellDocumentChecker):pchar;cdecl;

  aspell_document_checker_error: function(ths:PAspellDocumentChecker):PAspellError;cdecl;

      { Creates a new document checker.
       * The speller class is expected to last until
       * this class is destroyed.
       * If config is given it will be used to override
       * any relevent options set by this speller class.
  is: function done.cdecl;
       * If filter is given then it will take ownership of
       * the filter class and use it to do the filtering.
       * You are expected to free the checker when done.  }

  new_aspell_document_checker: function(speller: PAspellSpeller): PAspellCanHaveError;cdecl;

  to_aspell_document_checker: function(obj:PAspellCanHaveError):PAspellDocumentChecker;cdecl;

      { Reset the internal state of the filter.
       * Should be called whenever a new document is
       * being filtered.  }
  aspell_document_checker_reset: procedure(ths:PAspellDocumentChecker);cdecl;

      { Process a string.
       * The string passed in should only be split on
       * white space characters.  Furthermore, between
       * calls to reset, each string should be passed
       * in exactly once and in the order they appeared
       * in the document.  Passing in strings out of
       * order, skipping strings or passing them in
       * more than once may lead to undefined results.  }

  aspell_document_checker_process: procedure(ths:PAspellDocumentChecker; str:pchar; size:cint);cdecl;

      { Returns the next misspelled word in the
       * processed string.  If there are no more
       * misspelled words, then token.word will be
       * NULL and token.size will be 0  }

  // hack around struct/cdecl problem
  __aspell_document_checker_next_misspelling: function(ths:PAspellDocumentChecker):{$IFDEF CPU64}{$IFDEF LINUX}QWord{$ELSE}AspellToken{$ENDIF}{$ELSE}AspellToken{$ENDIF};cdecl;

      { Returns the underlying filter class.  }

  aspell_document_checker_filter: function(ths:PAspellDocumentChecker):PAspellFilter;cdecl;

      {***************************** word list ***************************** }

  aspell_word_list_empty: function(ths:PAspellWordList):cint;cdecl;

  aspell_word_list_size: function(ths:PAspellWordList):cuint;cdecl;

  aspell_word_list_elements: function(ths:PAspellWordList):PAspellStringEnumeration;cdecl;

      {************************* string enumeration ************************* }

  delete_aspell_string_enumeration: procedure(ths:PAspellStringEnumeration);cdecl;

  aspell_string_enumeration_clone: function(ths:PAspellStringEnumeration):PAspellStringEnumeration;cdecl;

  aspell_string_enumeration_assign: procedure(ths:PAspellStringEnumeration; other:PAspellStringEnumeration);cdecl;

  aspell_string_enumeration_at_end: function(ths:PAspellStringEnumeration):cint;cdecl;

  aspell_string_enumeration_next: function(ths:PAspellStringEnumeration):pchar;cdecl;

      {******************************** info ******************************** }

  get_aspell_module_info_list: function(config:PAspellConfig):PAspellModuleInfoList;cdecl;

  aspell_module_info_list_empty: function(ths:PAspellModuleInfoList):cint;cdecl;

  aspell_module_info_list_size: function(ths:PAspellModuleInfoList):cuint;cdecl;

  aspell_module_info_list_elements: function(ths:PAspellModuleInfoList):PAspellModuleInfoEnumeration;cdecl;

  get_aspell_dict_info_list: function(config:PAspellConfig):PAspellDictInfoList;cdecl;

  aspell_dict_info_list_empty: function(ths:PAspellDictInfoList):cint;cdecl;

  aspell_dict_info_list_size: function(ths:PAspellDictInfoList):cuint;cdecl;

  aspell_dict_info_list_elements: function(ths:PAspellDictInfoList):PAspellDictInfoEnumeration;cdecl;

  aspell_module_info_enumeration_at_end: function(ths:PAspellModuleInfoEnumeration):cint;cdecl;

  aspell_module_info_enumeration_next: function(ths:PAspellModuleInfoEnumeration):PAspellModuleInfo;cdecl;

  delete_aspell_module_info_enumeration: procedure(ths:PAspellModuleInfoEnumeration);cdecl;

  aspell_module_info_enumeration_clone: function(ths:PAspellModuleInfoEnumeration):PAspellModuleInfoEnumeration;cdecl;

  aspell_module_info_enumeration_assign: procedure(ths:PAspellModuleInfoEnumeration; other:PAspellModuleInfoEnumeration);cdecl;

  aspell_dict_info_enumeration_at_end: function(ths:PAspellDictInfoEnumeration):cint;cdecl;

  aspell_dict_info_enumeration_next: function(ths:PAspellDictInfoEnumeration):PAspellDictInfo;cdecl;

  delete_aspell_dict_info_enumeration: procedure(ths:PAspellDictInfoEnumeration);cdecl;

  aspell_dict_info_enumeration_clone: function(ths:PAspellDictInfoEnumeration):PAspellDictInfoEnumeration;cdecl;

  aspell_dict_info_enumeration_assign: procedure(ths:PAspellDictInfoEnumeration; other:PAspellDictInfoEnumeration);cdecl;

      {**************************** string list **************************** }

  new_aspell_string_list: function():PAspellStringList;cdecl;

  aspell_string_list_empty: function(ths:PAspellStringList):cint;cdecl;

  aspell_string_list_size: function(ths:PAspellStringList):cuint;cdecl;

  aspell_string_list_elements: function(ths:PAspellStringList):PAspellStringEnumeration;cdecl;

  aspell_string_list_add: function(ths:PAspellStringList; to_add:pchar):cint;cdecl;

  aspell_string_list_remove: function(ths:PAspellStringList; to_rem:pchar):cint;cdecl;

  aspell_string_list_clear: procedure(ths:PAspellStringList);cdecl;

  aspell_string_list_to_mutable_container: function(ths:PAspellStringList):PAspellMutableContainer;cdecl;

  delete_aspell_string_list: procedure(ths:PAspellStringList);cdecl;

  aspell_string_list_clone: function(ths:PAspellStringList):PAspellStringList;cdecl;

  aspell_string_list_assign: procedure(ths:PAspellStringList; other:PAspellStringList);cdecl;

      {***************************** string map ***************************** }

  new_aspell_string_map: function():PAspellStringMap;cdecl;

  aspell_string_map_add: function(ths:PAspellStringMap; to_add:pchar):cint;cdecl;

  aspell_string_map_remove: function(ths:PAspellStringMap; to_rem:pchar):cint;cdecl;

  aspell_string_map_clear: procedure(ths:PAspellStringMap);cdecl;

  aspell_string_map_to_mutable_container: function(ths:PAspellStringMap):PAspellMutableContainer;cdecl;

  delete_aspell_string_map: procedure(ths:PAspellStringMap);cdecl;

  aspell_string_map_clone: function(ths:PAspellStringMap):PAspellStringMap;cdecl;

  aspell_string_map_assign: procedure(ths:PAspellStringMap; other:PAspellStringMap);cdecl;

  aspell_string_map_empty: function(ths:PAspellStringMap):cint;cdecl;

  aspell_string_map_size: function(ths:PAspellStringMap):cuint;cdecl;

  aspell_string_map_elements: function(ths:PAspellStringMap):PAspellStringPairEnumeration;cdecl;

      { Insert a new element.
       * Will NOT overwrite an existing entry.
       * Returns FALSE if the element already exists.  }

  aspell_string_map_insert: function(ths:PAspellStringMap; key:pchar; value:pchar):cint;cdecl;

      { Insert a new element.
       * Will overwrite an existing entry.
       * Always returns TRUE.  }

  aspell_string_map_replace: function(ths:PAspellStringMap; key:pchar; value:pchar):cint;cdecl;

      { Looks up an element and returns the value.
       * Returns NULL if the element does not exist.
       * Returns an empty string if the element exists
       * but has a NULL value.  }

  aspell_string_map_lookup: function(ths:PAspellStringMap; key:pchar):pchar;cdecl;

      {********************** string pair enumeration ********************** }

  aspell_string_pair_enumeration_at_end: function(ths:PAspellStringPairEnumeration):cint;cdecl;

  aspell_string_pair_enumeration_next: function(ths:PAspellStringPairEnumeration):AspellStringPair;cdecl;

  delete_aspell_string_pair_enumeration: procedure(ths:PAspellStringPairEnumeration);cdecl;

  aspell_string_pair_enumeration_clone: function(ths:PAspellStringPairEnumeration):PAspellStringPairEnumeration;cdecl;

  aspell_string_pair_enumeration_assign: procedure(ths:PAspellStringPairEnumeration; other:PAspellStringPairEnumeration);cdecl;

      {******************************* cache ******************************* }
      { Reset the global cache(s) so that cache queries will
       * create a new object. If existing objects are still in
       * use they are not deleted. If which is NULL then all
       * caches will be reset. Current caches are "encode",
       * "decode", "dictionary", "language", and "keyboard".  }

  aspell_reset_cache: function(which:pchar):cint;cdecl;
  
{$ENDIF}

  function aspell_init(const libn: ansistring): Boolean;
  function aspell_loaded: Boolean;
  function aspell_document_checker_next_misspelling(ths:PAspellDocumentChecker):AspellToken;

implementation

{$IFDEF STATIC_ASPELL}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function aspell_init(const libn: ansistring): Boolean;
begin
  aspell_init := True;
end;

function aspell_loaded: Boolean;
begin
  aspell_loaded := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
function aspell_document_checker_next_misspelling(ths: PAspellDocumentChecker
  ): AspellToken;
begin
  // yup...
  aspell_document_checker_next_misspelling := AspellToken(__aspell_document_checker_next_misspelling(ths));
end;

{$ELSE} // dynamic

uses
  {$IFDEF WINDOWS}
  Windows, SysUtils, Classes,
  {$ENDIF}
  dynlibs;

var
  LibHandle: TLibHandle = 0;

{$IFDEF WINDOWS}
function RegistryQueryValue (name,sub:shortstring):shortstring;
const 
  maxkeysize=255;
var
  buf:string [maxkeysize];
  bufsize:longint;
  buftype:longint;
  res:longint;
  key,rkey:hkey;
  p,sp:pchar;

begin
  RegistryQueryValue:='';
  name:=name+#0; p:=@name[1];
  if sub='' then sp:=nil else begin sub:=sub+#0; sp:=@sub[1]; end;
  bufsize:=maxkeysize;
  buftype:=REG_SZ;
  key:=HKEY_LOCAL_MACHINE;
  res:=RegOpenKeyEx (key,p,0,KEY_QUERY_VALUE,rkey);
  if res<>ERROR_SUCCESS then exit;
  res:=RegQueryValueEx (rkey,sp,nil,@buftype,@buf[1],@bufsize);
  if res<>ERROR_SUCCESS then exit;
  buf[0]:=chr(bufsize-1);
  RegCloseKey (rkey);
  RegistryQueryValue:=buf;
end;
{$ENDIF}

function aspell_init(const libn: ansistring): Boolean;
var
  libname: ansistring;
  {$IFDEF WINDOWS}
  bversion, path: ansistring;
  version: dword;
  {$ENDIF}
  {$ifdef linux}
  i: Integer;
  s: string;
  {$endif}
begin
  aspell_init := True;
  libname := libn;
  
  {$IFDEF windows}
  bversion := RegistryQueryValue('SOFTWARE\Aspell','AspellVersion');
  if Length(bversion) >= 4 then begin
    move(bversion[1], version, 4);
    path := RegistryQueryValue('SOFTWARE\Aspell','Path');
    // will work if they passed %s, won't bork if they passed absolute
    libname := path + PathDelim + StringReplace(libn, '%s', IntToStr(Version), [rfReplaceAll]);
  end;
  {$ENDIF}

  LibHandle := LoadLibrary(libname);
  {$ifdef darwin}
  if LibHandle = 0 then begin
    libname := '/sw/lib/libaspell.dylib';
    LibHandle := LoadLibrary(libname);
  end;
  {$endif}

  {$ifdef linux}
  if LibHandle = 0 then begin
    for i := 15 to 30 do begin // TODO: make sure to up this when required
      str(i, s);
      libname := libn + '.' + s;
      LibHandle := LoadLibrary(libname);
      if LibHandle <> 0 then
        Break;
    end;
  end;
  {$endif}

  if LibHandle = 0 then
    Exit(False);

  aspell_mutable_container_add := nil;
  Pointer(aspell_mutable_container_add) := GetProcedureAddress(LibHandle, 'aspell_mutable_container_add');
  if not Assigned(aspell_mutable_container_add) then Exit(False);

  aspell_mutable_container_remove := nil;
  Pointer(aspell_mutable_container_remove) := GetProcedureAddress(LibHandle, 'aspell_mutable_container_remove');
  if not Assigned(aspell_mutable_container_remove) then Exit(False);

  aspell_mutable_container_clear := nil;
  Pointer(aspell_mutable_container_clear) := GetProcedureAddress(LibHandle, 'aspell_mutable_container_clear');
  if not Assigned(aspell_mutable_container_clear) then Exit(False);

  aspell_mutable_container_to_mutable_container := nil;
  Pointer(aspell_mutable_container_to_mutable_container) := GetProcedureAddress(LibHandle, 'aspell_mutable_container_to_mutable_container');
  if not Assigned(aspell_mutable_container_to_mutable_container) then Exit(False);

  aspell_key_info_enumeration_at_end := nil;
  Pointer(aspell_key_info_enumeration_at_end) := GetProcedureAddress(LibHandle, 'aspell_key_info_enumeration_at_end');
  if not Assigned(aspell_key_info_enumeration_at_end) then Exit(False);

  aspell_key_info_enumeration_next := nil;
  Pointer(aspell_key_info_enumeration_next) := GetProcedureAddress(LibHandle, 'aspell_key_info_enumeration_next');
  if not Assigned(aspell_key_info_enumeration_next) then Exit(False);

  delete_aspell_key_info_enumeration := nil;
  Pointer(delete_aspell_key_info_enumeration) := GetProcedureAddress(LibHandle, 'delete_aspell_key_info_enumeration');
  if not Assigned(delete_aspell_key_info_enumeration) then Exit(False);

  aspell_key_info_enumeration_clone := nil;
  Pointer(aspell_key_info_enumeration_clone) := GetProcedureAddress(LibHandle, 'aspell_key_info_enumeration_clone');
  if not Assigned(aspell_key_info_enumeration_clone) then Exit(False);

  aspell_key_info_enumeration_assign := nil;
  Pointer(aspell_key_info_enumeration_assign) := GetProcedureAddress(LibHandle, 'aspell_key_info_enumeration_assign');
  if not Assigned(aspell_key_info_enumeration_assign) then Exit(False);

  new_aspell_config := nil;
  Pointer(new_aspell_config) := GetProcedureAddress(LibHandle, 'new_aspell_config');
  if not Assigned(new_aspell_config) then Exit(False);

  delete_aspell_config := nil;
  Pointer(delete_aspell_config) := GetProcedureAddress(LibHandle, 'delete_aspell_config');
  if not Assigned(delete_aspell_config) then Exit(False);

  aspell_config_clone := nil;
  Pointer(aspell_config_clone) := GetProcedureAddress(LibHandle, 'aspell_config_clone');
  if not Assigned(aspell_config_clone) then Exit(False);

  aspell_config_assign := nil;
  Pointer(aspell_config_assign) := GetProcedureAddress(LibHandle, 'aspell_config_assign');
  if not Assigned(aspell_config_assign) then Exit(False);

  aspell_config_error_number := nil;
  Pointer(aspell_config_error_number) := GetProcedureAddress(LibHandle, 'aspell_config_error_number');
  if not Assigned(aspell_config_error_number) then Exit(False);

  aspell_config_error_message := nil;
  Pointer(aspell_config_error_message) := GetProcedureAddress(LibHandle, 'aspell_config_error_message');
  if not Assigned(aspell_config_error_message) then Exit(False);

  aspell_config_error := nil;
  Pointer(aspell_config_error) := GetProcedureAddress(LibHandle, 'aspell_config_error');
  if not Assigned(aspell_config_error) then Exit(False);

  aspell_config_set_extra := nil;
  Pointer(aspell_config_set_extra) := GetProcedureAddress(LibHandle, 'aspell_config_set_extra');
  if not Assigned(aspell_config_set_extra) then Exit(False);

  aspell_config_keyinfo := nil;
  Pointer(aspell_config_keyinfo) := GetProcedureAddress(LibHandle, 'aspell_config_keyinfo');
  if not Assigned(aspell_config_keyinfo) then Exit(False);

  aspell_config_possible_elements := nil;
  Pointer(aspell_config_possible_elements) := GetProcedureAddress(LibHandle, 'aspell_config_possible_elements');
  if not Assigned(aspell_config_possible_elements) then Exit(False);

  aspell_config_get_default := nil;
  Pointer(aspell_config_get_default) := GetProcedureAddress(LibHandle, 'aspell_config_get_default');
  if not Assigned(aspell_config_get_default) then Exit(False);

  aspell_config_elements := nil;
  Pointer(aspell_config_elements) := GetProcedureAddress(LibHandle, 'aspell_config_elements');
  if not Assigned(aspell_config_elements) then Exit(False);

  aspell_config_replace := nil;
  Pointer(aspell_config_replace) := GetProcedureAddress(LibHandle, 'aspell_config_replace');
  if not Assigned(aspell_config_replace) then Exit(False);

  aspell_config_remove := nil;
  Pointer(aspell_config_remove) := GetProcedureAddress(LibHandle, 'aspell_config_remove');
  if not Assigned(aspell_config_remove) then Exit(False);

  aspell_config_have := nil;
  Pointer(aspell_config_have) := GetProcedureAddress(LibHandle, 'aspell_config_have');
  if not Assigned(aspell_config_have) then Exit(False);

  aspell_config_retrieve := nil;
  Pointer(aspell_config_retrieve) := GetProcedureAddress(LibHandle, 'aspell_config_retrieve');
  if not Assigned(aspell_config_retrieve) then Exit(False);

  aspell_config_retrieve_list := nil;
  Pointer(aspell_config_retrieve_list) := GetProcedureAddress(LibHandle, 'aspell_config_retrieve_list');
  if not Assigned(aspell_config_retrieve_list) then Exit(False);

  aspell_config_retrieve_bool := nil;
  Pointer(aspell_config_retrieve_bool) := GetProcedureAddress(LibHandle, 'aspell_config_retrieve_bool');
  if not Assigned(aspell_config_retrieve_bool) then Exit(False);

  aspell_config_retrieve_int := nil;
  Pointer(aspell_config_retrieve_int) := GetProcedureAddress(LibHandle, 'aspell_config_retrieve_int');
  if not Assigned(aspell_config_retrieve_int) then Exit(False);

  aspell_error_is_a := nil;
  Pointer(aspell_error_is_a) := GetProcedureAddress(LibHandle, 'aspell_error_is_a');
  if not Assigned(aspell_error_is_a) then Exit(False);

  aspell_error_number := nil;
  Pointer(aspell_error_number) := GetProcedureAddress(LibHandle, 'aspell_error_number');
  if not Assigned(aspell_error_number) then Exit(False);

  aspell_error_message := nil;
  Pointer(aspell_error_message) := GetProcedureAddress(LibHandle, 'aspell_error_message');
  if not Assigned(aspell_error_message) then Exit(False);

  aspell_error := nil;
  Pointer(aspell_error) := GetProcedureAddress(LibHandle, 'aspell_error');
  if not Assigned(aspell_error) then Exit(False);

  delete_aspell_can_have_error := nil;
  Pointer(delete_aspell_can_have_error) := GetProcedureAddress(LibHandle, 'delete_aspell_can_have_error');
  if not Assigned(delete_aspell_can_have_error) then Exit(False);

  new_aspell_speller := nil;
  Pointer(new_aspell_speller) := GetProcedureAddress(LibHandle, 'new_aspell_speller');
  if not Assigned(new_aspell_speller) then Exit(False);

  to_aspell_speller := nil;
  Pointer(to_aspell_speller) := GetProcedureAddress(LibHandle, 'to_aspell_speller');
  if not Assigned(to_aspell_speller) then Exit(False);

  delete_aspell_speller := nil;
  Pointer(delete_aspell_speller) := GetProcedureAddress(LibHandle, 'delete_aspell_speller');
  if not Assigned(delete_aspell_speller) then Exit(False);

  aspell_speller_error_number := nil;
  Pointer(aspell_speller_error_number) := GetProcedureAddress(LibHandle, 'aspell_speller_error_number');
  if not Assigned(aspell_speller_error_number) then Exit(False);

  aspell_speller_error_message := nil;
  Pointer(aspell_speller_error_message) := GetProcedureAddress(LibHandle, 'aspell_speller_error_message');
  if not Assigned(aspell_speller_error_message) then Exit(False);

  aspell_speller_error := nil;
  Pointer(aspell_speller_error) := GetProcedureAddress(LibHandle, 'aspell_speller_error');
  if not Assigned(aspell_speller_error) then Exit(False);

  aspell_speller_config := nil;
  Pointer(aspell_speller_config) := GetProcedureAddress(LibHandle, 'aspell_speller_config');
  if not Assigned(aspell_speller_config) then Exit(False);

  aspell_speller_check := nil;
  Pointer(aspell_speller_check) := GetProcedureAddress(LibHandle, 'aspell_speller_check');
  if not Assigned(aspell_speller_check) then Exit(False);

  aspell_speller_add_to_personal := nil;
  Pointer(aspell_speller_add_to_personal) := GetProcedureAddress(LibHandle, 'aspell_speller_add_to_personal');
  if not Assigned(aspell_speller_add_to_personal) then Exit(False);

  aspell_speller_add_to_session := nil;
  Pointer(aspell_speller_add_to_session) := GetProcedureAddress(LibHandle, 'aspell_speller_add_to_session');
  if not Assigned(aspell_speller_add_to_session) then Exit(False);

  aspell_speller_personal_word_list := nil;
  Pointer(aspell_speller_personal_word_list) := GetProcedureAddress(LibHandle, 'aspell_speller_personal_word_list');
  if not Assigned(aspell_speller_personal_word_list) then Exit(False);

  aspell_speller_session_word_list := nil;
  Pointer(aspell_speller_session_word_list) := GetProcedureAddress(LibHandle, 'aspell_speller_session_word_list');
  if not Assigned(aspell_speller_session_word_list) then Exit(False);

  aspell_speller_main_word_list := nil;
  Pointer(aspell_speller_main_word_list) := GetProcedureAddress(LibHandle, 'aspell_speller_main_word_list');
  if not Assigned(aspell_speller_main_word_list) then Exit(False);

  aspell_speller_save_all_word_lists := nil;
  Pointer(aspell_speller_save_all_word_lists) := GetProcedureAddress(LibHandle, 'aspell_speller_save_all_word_lists');
  if not Assigned(aspell_speller_save_all_word_lists) then Exit(False);

  aspell_speller_clear_session := nil;
  Pointer(aspell_speller_clear_session) := GetProcedureAddress(LibHandle, 'aspell_speller_clear_session');
  if not Assigned(aspell_speller_clear_session) then Exit(False);

  aspell_speller_suggest := nil;
  Pointer(aspell_speller_suggest) := GetProcedureAddress(LibHandle, 'aspell_speller_suggest');
  if not Assigned(aspell_speller_suggest) then Exit(False);

  aspell_speller_store_replacement := nil;
  Pointer(aspell_speller_store_replacement) := GetProcedureAddress(LibHandle, 'aspell_speller_store_replacement');
  if not Assigned(aspell_speller_store_replacement) then Exit(False);

  delete_aspell_filter := nil;
  Pointer(delete_aspell_filter) := GetProcedureAddress(LibHandle, 'delete_aspell_filter');
  if not Assigned(delete_aspell_filter) then Exit(False);

  aspell_filter_error_number := nil;
  Pointer(aspell_filter_error_number) := GetProcedureAddress(LibHandle, 'aspell_filter_error_number');
  if not Assigned(aspell_filter_error_number) then Exit(False);

  aspell_filter_error_message := nil;
  Pointer(aspell_filter_error_message) := GetProcedureAddress(LibHandle, 'aspell_filter_error_message');
  if not Assigned(aspell_filter_error_message) then Exit(False);

  aspell_filter_error := nil;
  Pointer(aspell_filter_error) := GetProcedureAddress(LibHandle, 'aspell_filter_error');
  if not Assigned(aspell_filter_error) then Exit(False);

  to_aspell_filter := nil;
  Pointer(to_aspell_filter) := GetProcedureAddress(LibHandle, 'to_aspell_filter');
  if not Assigned(to_aspell_filter) then Exit(False);

  delete_aspell_document_checker := nil;
  Pointer(delete_aspell_document_checker) := GetProcedureAddress(LibHandle, 'delete_aspell_document_checker');
  if not Assigned(delete_aspell_document_checker) then Exit(False);

  aspell_document_checker_error_number := nil;
  Pointer(aspell_document_checker_error_number) := GetProcedureAddress(LibHandle, 'aspell_document_checker_error_number');
  if not Assigned(aspell_document_checker_error_number) then Exit(False);

  aspell_document_checker_error_message := nil;
  Pointer(aspell_document_checker_error_message) := GetProcedureAddress(LibHandle, 'aspell_document_checker_error_message');
  if not Assigned(aspell_document_checker_error_message) then Exit(False);

  aspell_document_checker_error := nil;
  Pointer(aspell_document_checker_error) := GetProcedureAddress(LibHandle, 'aspell_document_checker_error');
  if not Assigned(aspell_document_checker_error) then Exit(False);

  new_aspell_document_checker := nil;
  Pointer(new_aspell_document_checker) := GetProcedureAddress(LibHandle, 'new_aspell_document_checker');
  if not Assigned(new_aspell_document_checker) then Exit(False);

  to_aspell_document_checker := nil;
  Pointer(to_aspell_document_checker) := GetProcedureAddress(LibHandle, 'to_aspell_document_checker');
  if not Assigned(to_aspell_document_checker) then Exit(False);

  aspell_document_checker_reset := nil;
  Pointer(aspell_document_checker_reset) := GetProcedureAddress(LibHandle, 'aspell_document_checker_reset');
  if not Assigned(aspell_document_checker_reset) then Exit(False);

  aspell_document_checker_process := nil;
  Pointer(aspell_document_checker_process) := GetProcedureAddress(LibHandle, 'aspell_document_checker_process');
  if not Assigned(aspell_document_checker_process) then Exit(False);

  __aspell_document_checker_next_misspelling := nil;
  Pointer(__aspell_document_checker_next_misspelling) := GetProcedureAddress(LibHandle, 'aspell_document_checker_next_misspelling');
  if not Assigned(__aspell_document_checker_next_misspelling) then Exit(False);

  aspell_document_checker_filter := nil;
  Pointer(aspell_document_checker_filter) := GetProcedureAddress(LibHandle, 'aspell_document_checker_filter');
  if not Assigned(aspell_document_checker_filter) then Exit(False);

  aspell_word_list_empty := nil;
  Pointer(aspell_word_list_empty) := GetProcedureAddress(LibHandle, 'aspell_word_list_empty');
  if not Assigned(aspell_word_list_empty) then Exit(False);

  aspell_word_list_size := nil;
  Pointer(aspell_word_list_size) := GetProcedureAddress(LibHandle, 'aspell_word_list_size');
  if not Assigned(aspell_word_list_size) then Exit(False);

  aspell_word_list_elements := nil;
  Pointer(aspell_word_list_elements) := GetProcedureAddress(LibHandle, 'aspell_word_list_elements');
  if not Assigned(aspell_word_list_elements) then Exit(False);

  delete_aspell_string_enumeration := nil;
  Pointer(delete_aspell_string_enumeration) := GetProcedureAddress(LibHandle, 'delete_aspell_string_enumeration');
  if not Assigned(delete_aspell_string_enumeration) then Exit(False);

  aspell_string_enumeration_clone := nil;
  Pointer(aspell_string_enumeration_clone) := GetProcedureAddress(LibHandle, 'aspell_string_enumeration_clone');
  if not Assigned(aspell_string_enumeration_clone) then Exit(False);

  aspell_string_enumeration_assign := nil;
  Pointer(aspell_string_enumeration_assign) := GetProcedureAddress(LibHandle, 'aspell_string_enumeration_assign');
  if not Assigned(aspell_string_enumeration_assign) then Exit(False);

  aspell_string_enumeration_at_end := nil;
  Pointer(aspell_string_enumeration_at_end) := GetProcedureAddress(LibHandle, 'aspell_string_enumeration_at_end');
  if not Assigned(aspell_string_enumeration_at_end) then Exit(False);

  aspell_string_enumeration_next := nil;
  Pointer(aspell_string_enumeration_next) := GetProcedureAddress(LibHandle, 'aspell_string_enumeration_next');
  if not Assigned(aspell_string_enumeration_next) then Exit(False);

  get_aspell_module_info_list := nil;
  Pointer(get_aspell_module_info_list) := GetProcedureAddress(LibHandle, 'get_aspell_module_info_list');
  if not Assigned(get_aspell_module_info_list) then Exit(False);

  aspell_module_info_list_empty := nil;
  Pointer(aspell_module_info_list_empty) := GetProcedureAddress(LibHandle, 'aspell_module_info_list_empty');
  if not Assigned(aspell_module_info_list_empty) then Exit(False);

  aspell_module_info_list_size := nil;
  Pointer(aspell_module_info_list_size) := GetProcedureAddress(LibHandle, 'aspell_module_info_list_size');
  if not Assigned(aspell_module_info_list_size) then Exit(False);

  aspell_module_info_list_elements := nil;
  Pointer(aspell_module_info_list_elements) := GetProcedureAddress(LibHandle, 'aspell_module_info_list_elements');
  if not Assigned(aspell_module_info_list_elements) then Exit(False);

  get_aspell_dict_info_list := nil;
  Pointer(get_aspell_dict_info_list) := GetProcedureAddress(LibHandle, 'get_aspell_dict_info_list');
  if not Assigned(get_aspell_dict_info_list) then Exit(False);

  aspell_dict_info_list_empty := nil;
  Pointer(aspell_dict_info_list_empty) := GetProcedureAddress(LibHandle, 'aspell_dict_info_list_empty');
  if not Assigned(aspell_dict_info_list_empty) then Exit(False);

  aspell_dict_info_list_size := nil;
  Pointer(aspell_dict_info_list_size) := GetProcedureAddress(LibHandle, 'aspell_dict_info_list_size');
  if not Assigned(aspell_dict_info_list_size) then Exit(False);

  aspell_dict_info_list_elements := nil;
  Pointer(aspell_dict_info_list_elements) := GetProcedureAddress(LibHandle, 'aspell_dict_info_list_elements');
  if not Assigned(aspell_dict_info_list_elements) then Exit(False);

  aspell_module_info_enumeration_at_end := nil;
  Pointer(aspell_module_info_enumeration_at_end) := GetProcedureAddress(LibHandle, 'aspell_module_info_enumeration_at_end');
  if not Assigned(aspell_module_info_enumeration_at_end) then Exit(False);

  aspell_module_info_enumeration_next := nil;
  Pointer(aspell_module_info_enumeration_next) := GetProcedureAddress(LibHandle, 'aspell_module_info_enumeration_next');
  if not Assigned(aspell_module_info_enumeration_next) then Exit(False);

  delete_aspell_module_info_enumeration := nil;
  Pointer(delete_aspell_module_info_enumeration) := GetProcedureAddress(LibHandle, 'delete_aspell_module_info_enumeration');
  if not Assigned(delete_aspell_module_info_enumeration) then Exit(False);

  aspell_module_info_enumeration_clone := nil;
  Pointer(aspell_module_info_enumeration_clone) := GetProcedureAddress(LibHandle, 'aspell_module_info_enumeration_clone');
  if not Assigned(aspell_module_info_enumeration_clone) then Exit(False);

  aspell_module_info_enumeration_assign := nil;
  Pointer(aspell_module_info_enumeration_assign) := GetProcedureAddress(LibHandle, 'aspell_module_info_enumeration_assign');
  if not Assigned(aspell_module_info_enumeration_assign) then Exit(False);

  aspell_dict_info_enumeration_at_end := nil;
  Pointer(aspell_dict_info_enumeration_at_end) := GetProcedureAddress(LibHandle, 'aspell_dict_info_enumeration_at_end');
  if not Assigned(aspell_dict_info_enumeration_at_end) then Exit(False);

  aspell_dict_info_enumeration_next := nil;
  Pointer(aspell_dict_info_enumeration_next) := GetProcedureAddress(LibHandle, 'aspell_dict_info_enumeration_next');
  if not Assigned(aspell_dict_info_enumeration_next) then Exit(False);

  delete_aspell_dict_info_enumeration := nil;
  Pointer(delete_aspell_dict_info_enumeration) := GetProcedureAddress(LibHandle, 'delete_aspell_dict_info_enumeration');
  if not Assigned(delete_aspell_dict_info_enumeration) then Exit(False);

  aspell_dict_info_enumeration_clone := nil;
  Pointer(aspell_dict_info_enumeration_clone) := GetProcedureAddress(LibHandle, 'aspell_dict_info_enumeration_clone');
  if not Assigned(aspell_dict_info_enumeration_clone) then Exit(False);

  aspell_dict_info_enumeration_assign := nil;
  Pointer(aspell_dict_info_enumeration_assign) := GetProcedureAddress(LibHandle, 'aspell_dict_info_enumeration_assign');
  if not Assigned(aspell_dict_info_enumeration_assign) then Exit(False);

  new_aspell_string_list := nil;
  Pointer(new_aspell_string_list) := GetProcedureAddress(LibHandle, 'new_aspell_string_list');
  if not Assigned(new_aspell_string_list) then Exit(False);

  aspell_string_list_empty := nil;
  Pointer(aspell_string_list_empty) := GetProcedureAddress(LibHandle, 'aspell_string_list_empty');
  if not Assigned(aspell_string_list_empty) then Exit(False);

  aspell_string_list_size := nil;
  Pointer(aspell_string_list_size) := GetProcedureAddress(LibHandle, 'aspell_string_list_size');
  if not Assigned(aspell_string_list_size) then Exit(False);

  aspell_string_list_elements := nil;
  Pointer(aspell_string_list_elements) := GetProcedureAddress(LibHandle, 'aspell_string_list_elements');
  if not Assigned(aspell_string_list_elements) then Exit(False);

  aspell_string_list_add := nil;
  Pointer(aspell_string_list_add) := GetProcedureAddress(LibHandle, 'aspell_string_list_add');
  if not Assigned(aspell_string_list_add) then Exit(False);

  aspell_string_list_remove := nil;
  Pointer(aspell_string_list_remove) := GetProcedureAddress(LibHandle, 'aspell_string_list_remove');
  if not Assigned(aspell_string_list_remove) then Exit(False);

  aspell_string_list_clear := nil;
  Pointer(aspell_string_list_clear) := GetProcedureAddress(LibHandle, 'aspell_string_list_clear');
  if not Assigned(aspell_string_list_clear) then Exit(False);

  aspell_string_list_to_mutable_container := nil;
  Pointer(aspell_string_list_to_mutable_container) := GetProcedureAddress(LibHandle, 'aspell_string_list_to_mutable_container');
  if not Assigned(aspell_string_list_to_mutable_container) then Exit(False);

  delete_aspell_string_list := nil;
  Pointer(delete_aspell_string_list) := GetProcedureAddress(LibHandle, 'delete_aspell_string_list');
  if not Assigned(delete_aspell_string_list) then Exit(False);

  aspell_string_list_clone := nil;
  Pointer(aspell_string_list_clone) := GetProcedureAddress(LibHandle, 'aspell_string_list_clone');
  if not Assigned(aspell_string_list_clone) then Exit(False);

  aspell_string_list_assign := nil;
  Pointer(aspell_string_list_assign) := GetProcedureAddress(LibHandle, 'aspell_string_list_assign');
  if not Assigned(aspell_string_list_assign) then Exit(False);

  new_aspell_string_map := nil;
  Pointer(new_aspell_string_map) := GetProcedureAddress(LibHandle, 'new_aspell_string_map');
  if not Assigned(new_aspell_string_map) then Exit(False);

  aspell_string_map_add := nil;
  Pointer(aspell_string_map_add) := GetProcedureAddress(LibHandle, 'aspell_string_map_add');
  if not Assigned(aspell_string_map_add) then Exit(False);

  aspell_string_map_remove := nil;
  Pointer(aspell_string_map_remove) := GetProcedureAddress(LibHandle, 'aspell_string_map_remove');
  if not Assigned(aspell_string_map_remove) then Exit(False);

  aspell_string_map_clear := nil;
  Pointer(aspell_string_map_clear) := GetProcedureAddress(LibHandle, 'aspell_string_map_clear');
  if not Assigned(aspell_string_map_clear) then Exit(False);

  aspell_string_map_to_mutable_container := nil;
  Pointer(aspell_string_map_to_mutable_container) := GetProcedureAddress(LibHandle, 'aspell_string_map_to_mutable_container');
  if not Assigned(aspell_string_map_to_mutable_container) then Exit(False);

  delete_aspell_string_map := nil;
  Pointer(delete_aspell_string_map) := GetProcedureAddress(LibHandle, 'delete_aspell_string_map');
  if not Assigned(delete_aspell_string_map) then Exit(False);

  aspell_string_map_clone := nil;
  Pointer(aspell_string_map_clone) := GetProcedureAddress(LibHandle, 'aspell_string_map_clone');
  if not Assigned(aspell_string_map_clone) then Exit(False);

  aspell_string_map_assign := nil;
  Pointer(aspell_string_map_assign) := GetProcedureAddress(LibHandle, 'aspell_string_map_assign');
  if not Assigned(aspell_string_map_assign) then Exit(False);

  aspell_string_map_empty := nil;
  Pointer(aspell_string_map_empty) := GetProcedureAddress(LibHandle, 'aspell_string_map_empty');
  if not Assigned(aspell_string_map_empty) then Exit(False);

  aspell_string_map_size := nil;
  Pointer(aspell_string_map_size) := GetProcedureAddress(LibHandle, 'aspell_string_map_size');
  if not Assigned(aspell_string_map_size) then Exit(False);

  aspell_string_map_elements := nil;
  Pointer(aspell_string_map_elements) := GetProcedureAddress(LibHandle, 'aspell_string_map_elements');
  if not Assigned(aspell_string_map_elements) then Exit(False);

  aspell_string_map_insert := nil;
  Pointer(aspell_string_map_insert) := GetProcedureAddress(LibHandle, 'aspell_string_map_insert');
  if not Assigned(aspell_string_map_insert) then Exit(False);

  aspell_string_map_replace := nil;
  Pointer(aspell_string_map_replace) := GetProcedureAddress(LibHandle, 'aspell_string_map_replace');
  if not Assigned(aspell_string_map_replace) then Exit(False);

  aspell_string_map_lookup := nil;
  Pointer(aspell_string_map_lookup) := GetProcedureAddress(LibHandle, 'aspell_string_map_lookup');
  if not Assigned(aspell_string_map_lookup) then Exit(False);

  aspell_string_pair_enumeration_at_end := nil;
  Pointer(aspell_string_pair_enumeration_at_end) := GetProcedureAddress(LibHandle, 'aspell_string_pair_enumeration_at_end');
  if not Assigned(aspell_string_pair_enumeration_at_end) then Exit(False);

  aspell_string_pair_enumeration_next := nil;
  Pointer(aspell_string_pair_enumeration_next) := GetProcedureAddress(LibHandle, 'aspell_string_pair_enumeration_next');
  if not Assigned(aspell_string_pair_enumeration_next) then Exit(False);

  delete_aspell_string_pair_enumeration := nil;
  Pointer(delete_aspell_string_pair_enumeration) := GetProcedureAddress(LibHandle, 'delete_aspell_string_pair_enumeration');
  if not Assigned(delete_aspell_string_pair_enumeration) then Exit(False);

  aspell_string_pair_enumeration_clone := nil;
  Pointer(aspell_string_pair_enumeration_clone) := GetProcedureAddress(LibHandle, 'aspell_string_pair_enumeration_clone');
  if not Assigned(aspell_string_pair_enumeration_clone) then Exit(False);

  aspell_string_pair_enumeration_assign := nil;
  Pointer(aspell_string_pair_enumeration_assign) := GetProcedureAddress(LibHandle, 'aspell_string_pair_enumeration_assign');
  if not Assigned(aspell_string_pair_enumeration_assign) then Exit(False);

  aspell_reset_cache := nil;
  Pointer(aspell_reset_cache) := GetProcedureAddress(LibHandle, 'aspell_reset_cache');
  if not Assigned(aspell_reset_cache) then Exit(False);
end;

function aspell_loaded: Boolean;
begin
  aspell_loaded := LibHandle <> 0;
end;

function aspell_document_checker_next_misspelling(ths: PAspellDocumentChecker
  ): AspellToken;
begin
  // yup...
  aspell_document_checker_next_misspelling := AspellToken(__aspell_document_checker_next_misspelling(ths));
end;

initialization
  aspell_init(libaspell);

finalization
  if LibHandle <> 0 then
    UnloadLibrary(LibHandle);
    
{$ENDIF}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
