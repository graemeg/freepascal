unit pwd;

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  initc,unixtype,baseunix,ctypes;
=======
  initc,baseunix,ctypes;
>>>>>>> graemeg/fixes_2_2
=======
  initc,baseunix,ctypes;
>>>>>>> origin/fixes_2_2

{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}

const
      External_library= clib;  {Setup as you need}

  const
     _PATH_PWD = '/etc';     
     _PATH_PASSWD = '/etc/passwd';     
     _PASSWD = 'passwd';     
     _PATH_MASTERPASSWD = '/etc/master.passwd';     
{$ifdef Darwin}
     _PATH_MASTERPASSWD_LOCK = '/etc/ptmp';
{$endif}
     _MASTERPASSWD = 'master.passwd';     

     _PATH_MP_DB = '/etc/pwd.db';     
     _MP_DB = 'pwd.db';     
     _PATH_SMP_DB = '/etc/spwd.db';     
     _SMP_DB = 'spwd.db';     
     _PATH_PWD_MKDB = '/usr/sbin/pwd_mkdb';     


{$ifdef BSD}
  _PW_VERSION_MASK = #$F0;     

  function _PW_VERSIONED(x,v : longint) : cuchar; inline; 


  const
     _PW_KEYBYNAME    = #$31;          { stored by name  } 
     _PW_KEYBYNUM     = #$32;        { stored by entry in the "file"}
     _PW_KEYBYUID     = #$33;        { stored by uid  }
   {$ifdef FreeBSD}
     _PW_KEYYPENABLED = #$34;        { YP is enabled  }
     _PW_KEYYPBYNUM   = #$35;        { special +@netgroup entries  }
   {$endif}

  { The database also contains a key to indicate the format version of
   * the entries therein.  There may be other, older versioned entries
   * as well. }

const  
   {$ifdef FreeBSD}
    _PWD_VERSION_KEY	 = #$FF+'VERSION';
    _PWD_CURRENT_VERSION = #$4;
   {$endif}
    _PASSWORD_EFMT1      = '_';      	{ extended encryption format  } 
    _PASSWORD_LEN        = 128;         { max length, not counting NULL  }
  {$ifdef Darwin}
    _PASSWORD_NOUID         = $01;    (* flag for no specified uid. *)
    _PASSWORD_NOGID         = $02;    (* flag for no specified gid. *)
    _PASSWORD_NOCHG         = $04;    (* flag for no specified change. *)
    _PASSWORD_NOEXP          =$08;    (* flag for no specified expire. *)
    _PASSWORD_WARNDAYS      = 14;     (* days to warn about expiry *)
    _PASSWORD_CHGNOW        = -1;     (* special day to force password
                                         * change at next login *)
  {$endif}
 
{$endif}

type
{ Darwin uses __darwin_time_t, but that is an alias for time_t }
     PPasswd  = ^TPasswd;
     PPPasswd = ^PPasswd;
     Passwd   = record
            pw_name    : pchar;        { user name  }
            pw_passwd  : pchar;	{ encrypted password  }
            pw_uid     : Tuid;		{ user uid  }
            pw_gid     : Tgid;		{ user gid  }
            {$ifdef bsd}
            pw_change  : Ttime platform;         { password change time  }
            pw_class   : pchar platform;        { user access class  }
            {$endif}
            pw_gecos   : pchar;        { Honeywell login info  }
            pw_dir     : pchar;        { home directory  }
            pw_shell   : pchar;        { default shell  }
            {$ifdef bsd}
            pw_expire  : Ttime platform;         { account expiration  }
            {$ifdef FreeBSD}
            pw_fields  : cint platform;          { internal: fields filled in  }
            {$endif}
            {$endif}
         end;
     TPasswd  = Passwd;


{$ifdef FreeBSD}
const
        _PWF_NAME	 = 	1;
        _PWF_PASSWD	 = 	2;
        _PWF_UID	 = 	4;
        _PWF_GID	 = 	8;
        _PWF_CHANGE      =    $10;
        _PWF_CLASS	 =    $20;
        _PWF_GECOS	 =    $40;
        _PWF_DIR	 =    $80;
        _PWF_SHELL	 =   $100;
        _PWF_EXPIRE	 =   $200;


        _PWF_SOURCE 	 = $3000;       
        _PWF_FILES 	 = $1000;       
        _PWF_NIS 	 = $2000;       
        _PWF_HESIOD 	 = $3000;       
{$endif}

function  fpgetpwnam (name:pchar):PPasswd; cdecl;external External_library name 'getpwnam';
function  fpgetpwuid (id:tuid):PPasswd;cdecl;external External_library name 'getpwuid';

procedure fpendpwent;cdecl;external External_library name 'endpwent';
function  fpgetpwent:ppasswd;cdecl;external External_library name 'getpwent';
procedure fpsetpwent;cdecl;external External_library name 'setpwent';

function  fpgetpwnam_r (namepara1:pchar; pwd:Ppasswd; buffer:pchar; bufsize:size_t; pwresult:PPpasswd):cint;cdecl;external External_library name 'getpwnam_r';
function  fpgetpwuid_r (uid:uid_t; pwd:Ppasswd; buffer:pchar; buffersize:size_t; pwresult:PPpasswd):cint;cdecl;external External_library name 'getpwuid_r';
{$ifndef Darwin}
function  fpgetpwent_r (pwd:Ppasswd; buffer:pchar; buffersize:size_t; pwresult:PPpasswd):cint;cdecl;external External_library name 'getpwent_r';
{$endif}

implementation

{$ifdef BSD}
function _PW_VERSIONED (x,v : longint) : cuchar; inline; 

begin
 _PW_VERSIONED:=  (x and $CF) or  (v shl 4);
end;
{$endif}

end.
