Type
<<<<<<< HEAD
<<<<<<< HEAD
  HKEY = THandle;
  PHKEY = ^HKEY;
  
{$ifdef windows}

{ Direct mapping to constants in Windows unit }

Const
  HKEY_CLASSES_ROOT     = Windows.HKEY_CLASSES_ROOT;
  HKEY_CURRENT_USER     = Windows.HKEY_CURRENT_USER;
  HKEY_LOCAL_MACHINE    = Windows.HKEY_LOCAL_MACHINE;
  HKEY_USERS            = Windows.HKEY_USERS;
  HKEY_PERFORMANCE_DATA = Windows.HKEY_PERFORMANCE_DATA;
  HKEY_CURRENT_CONFIG   = Windows.HKEY_CURRENT_CONFIG;
  HKEY_DYN_DATA         = Windows.HKEY_DYN_DATA;

  KEY_ALL_ACCESS         = Windows.KEY_ALL_ACCESS;
  KEY_CREATE_LINK        = Windows.KEY_CREATE_LINK;
  KEY_CREATE_SUB_KEY     = Windows.KEY_CREATE_SUB_KEY;
  KEY_ENUMERATE_SUB_KEYS = Windows.KEY_ENUMERATE_SUB_KEYS;
  KEY_EXECUTE            = Windows.KEY_EXECUTE;
  KEY_NOTIFY             = Windows.KEY_NOTIFY;
  KEY_QUERY_VALUE        = Windows.KEY_QUERY_VALUE;
  KEY_READ               = Windows.KEY_READ;
  KEY_SET_VALUE          = Windows.KEY_SET_VALUE;
  KEY_WRITE              = Windows.KEY_WRITE;
{$ifndef WinCE}
  KEY_WOW64_64KEY        = Windows.KEY_WOW64_64KEY;
  KEY_WOW64_32KEY        = Windows.KEY_WOW64_32KEY;
{$endif WinCE}
{$else}

Const
=======
=======
>>>>>>> origin/fixes_2_2
  LPDWORD = ^DWord;
  LPVOID  = Pointer;
  WINBOOL = LongBool;
  LPCSTR  = PChar;
  LPSTR   = Pchar;
  LONG    = LongInt;
  LPBYTE  = ^Byte;

  ACCESS_MASK = DWORD;
       REGSAM = ACCESS_MASK;

  SECURITY_ATTRIBUTES = record
    nLength : DWORD;
    lpSecurityDescriptor : LPVOID;
    bInheritHandle : WINBOOL;
  end;
  LPSECURITY_ATTRIBUTES = ^SECURITY_ATTRIBUTES;


  HKEY = THandle;
  PHKEY = ^HKEY;



Const
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  HKEY_CLASSES_ROOT     = HKEY($80000000);
  HKEY_CURRENT_USER     = HKEY($80000001);
  HKEY_LOCAL_MACHINE    = HKEY($80000002);
  HKEY_USERS            = HKEY($80000003);
  HKEY_PERFORMANCE_DATA = HKEY($80000004);
  HKEY_CURRENT_CONFIG   = HKEY($80000005);
  HKEY_DYN_DATA         = HKEY($80000006);

  KEY_ALL_ACCESS         = $F003F;
  KEY_CREATE_LINK        = 32;
  KEY_CREATE_SUB_KEY     = 4;
  KEY_ENUMERATE_SUB_KEYS = 8;
  KEY_EXECUTE            = $20019;
  KEY_NOTIFY             = 16;
  KEY_QUERY_VALUE        = 1;
  KEY_READ               = $20019;
  KEY_SET_VALUE          = 2;
  KEY_WRITE              = $20006;
<<<<<<< HEAD
  KEY_WOW64_64KEY        = $100;
  KEY_WOW64_32KEY        = $200;
  
{$endif windows}
=======

  REG_BINARY                     = 3;
  REG_DWORD                      = 4;
  REG_DWORD_LITTLE_ENDIAN        = 4;
  REG_DWORD_BIG_ENDIAN           = 5;
  REG_EXPAND_SZ                  = 2;
  REG_FULL_RESOURCE_DESCRIPTOR   = 9;
  REG_LINK                       = 6;
  REG_MULTI_SZ                   = 7;
  REG_NONE                       = 0;
  REG_RESOURCE_LIST              = 8;
  REG_RESOURCE_REQUIREMENTS_LIST = 10;
  REG_SZ                         = 1;

  REG_OPTION_VOLATILE            = 1;
  REG_OPTION_NON_VOLATILE        = 0;
  REG_CREATED_NEW_KEY            = 1;
  REG_OPENED_EXISTING_KEY        = 2;

  ERROR_SUCCESS = 0;


>>>>>>> graemeg/fixes_2_2
