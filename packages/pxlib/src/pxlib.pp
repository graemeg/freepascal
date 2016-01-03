{$IFDEF FPC}
{$mode objfpc}
{$h+}
{$PACKRECORDS C}
{$ENDIF}
unit pxlib;
interface

uses
{$ifdef unix}
  unixtype,
{$endif}  
  ctypes;

{ Automatically converted by H2Pas 1.0.0 from pxlib.h
  The following command line parameters were used:
  pxlib.h -D -l pxlib -o pxlib.pp -P -u pxlib -C -p
  MVC : Heavily edited after generation. C is a mess :( }

const
{$ifdef windows}
  pxlibraryname='pxlib.dll';
{$else}
  pxlibraryname='libpx.so.0'; { Default name }
{$endif}  

const

  px_true = 1;     
  px_false = 0;     
 
  { Error codes  }
 
  PX_MemoryError = 1;     
  PX_IOError = 2;     
  PX_RuntimeError = 3;     
  PX_Warning = 100;     
  
  { IO Stream types  }
 
  pxfIOFile   = 1;      { pxfIOGsf is defined as 2 in paradox-gsf.h  }
  pxfIOStream = 3;   
  
  { Field types  }
  
  pxfAlpha       = $01;
  pxfDate        = $02;
  pxfShort       = $03;
  pxfLong        = $04;
  pxfCurrency    = $05;
  pxfNumber      = $06;
  pxfLogical     = $09;
  pxfMemoBLOb    = $0C;
  pxfBLOb        = $0D;
  pxfFmtMemoBLOb = $0E;     
  pxfOLE         = $0F;
  pxfGraphic     = $10;
  pxfTime        = $14;
  pxfTimestamp   = $15;
  pxfAutoInc     = $16;
  pxfBCD         = $17;
  pxfBytes       = $18;
  pxfNumTypes    = $18;
  
  { File types  }
  pxfFileTypIndexDB = 0;     
  pxfFileTypPrimIndex = 1;     
  pxfFileTypNonIndexDB = 2;     
  pxfFileTypNonIncSecIndex = 3;     
  pxfFileTypSecIndex = 4;     
  pxfFileTypIncSecIndex = 5;     
  pxfFileTypNonIncSecIndexG = 6;     
  pxfFileTypSecIndexG = 7;     
  pxfFileTypIncSecIndexG = 8;     
 
  { File modes  }
  pxfFileRead  = $1;     
  pxfFileWrite = $2;     

Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;
  Pcchar    = pchar;

{$ifndef unix}
  size_t    = Integer;
{$endif}

  PFILE  = ^FILE;
  iconv_t = pointer;
  
  Ppxstream_t = ^pxstream_t;
  Ppxfield_t = ^pxfield_t;
  ppxval_t = ^pxval_t;
  Ppxhead_t = ^pxhead_t;
  Ppxdoc_t = ^pxdoc_t;
  ppxblockcache_t = ^pxblockcache_t;     
  ppxmbblockinfo_t = ^pxmbblockinfo_t;
  Ppxblob_t = ^pxblob_t;
  Pmbhead_t = ^mbhead_t;
  Ppxdatablockinfo_t = ^pxdatablockinfo_t;
  PPpxval_t = ^ppxval_t;
  Ppcchar = ^pcchar;
      
  Ppx_field = ^px_field;
  px_field = record
    px_fname : pcchar;
    px_ftype : cchar;
    px_flen : cint;
    px_fdc : cint;
  end;
  pxfield_t = px_field;  

  Ppx_val = ^px_val;
  px_val = record
    isnull : cchar;
    _type : cint;
    value : record
    case longint of
      0 : ( lval : clong );
      1 : ( dval : double );
      2 : ( str : record
            val : pcchar;
            len : cint;
            end );
    end;
  end;
  pxval_t = px_val;  

  Ppx_head = ^px_head;
  px_head = record
    px_tablename : pcchar;
    px_recordsize : cint;
    px_filetype : cchar;
    px_fileversion : cint;
    px_numrecords : cint;
    px_theonumrecords : cint;
    px_numfields : cint;
    px_maxtablesize : cint;
    px_headersize : cint;
    px_fileblocks : cuint;
    px_firstblock : cuint;
    px_lastblock : cuint;
    px_indexfieldnumber : cint;
    px_indexroot : cint;
    px_numindexlevels : cint;
    px_writeprotected : cint;
    px_doscodepage : cint;
    px_primarykeyfields : cint;
    px_modifiedflags1 : cchar;
    px_modifiedflags2 : cchar;
    px_sortorder : cchar;
    px_autoinc : cint;
    px_fileupdatetime : cint;
    px_refintegrity : cchar;
    px_fields : Ppx_field;
    px_encryption : culong;
  end;
  pxhead_t = px_head;

  Ppx_stream = ^px_stream;
  px_stream = record
    _type : cint;
    mode : cint;
    close : cint;
    s : record
        case longint of
           0 : ( fp : PFILE );
           1 : ( stream : pointer );
        end;
    read : function (p:Ppxdoc_t; stream:Ppxstream_t; numbytes:size_t; buffer:pointer):size_t;cdecl;
    seek : function (p:Ppxdoc_t; stream:Ppxstream_t; offset:clong; whence:cint):cint; cdecl;
    tell : function (p:Ppxdoc_t; stream:Ppxstream_t):clong; cdecl;
    write : function (p:Ppxdoc_t; stream:Ppxstream_t; numbytes:size_t; buffer:pointer):size_t;cdecl;
  end;
  pxstream_t = px_stream;  

  Ppx_doc = ^px_doc;
  px_doc = record
    px_stream : Ppxstream_t;
    px_name : pcchar;
    px_close_fp : cint;
    px_head : Ppxhead_t;
    px_data : pointer;
    px_datalen : cint;
    px_indexdata : pointer;
    px_indexdatalen : cint;
    px_pindex : Ppxdoc_t;
    px_blob : Ppxblob_t;
    last_position : cint;
    warnings : cint;
    writeproc : function (p:Ppxdoc_t; data:pointer; size:size_t):size_t; cdecl;
    errorhandler : procedure (p:Ppxdoc_t; level:cint; msg:pcchar; data:pointer); cdecl;
    errorhandler_user_data : pointer; 
    malloc : function (p:Ppxdoc_t; size:size_t; caller:pcchar):pointer; cdecl; 
    calloc : function (p:Ppxdoc_t; size:size_t; caller:pcchar):pointer; cdecl;
    realloc : function (p:Ppxdoc_t; mem:pointer; size:size_t; caller:pcchar):pointer; cdecl;
    free : procedure (p:Ppxdoc_t; mem:pointer); cdecl;
    read : function (p:Ppxdoc_t; stream:Ppxstream_t; numbytes:size_t; buffer:pointer):size_t; cdecl;
    seek : function (p:Ppxdoc_t; stream:Ppxstream_t; offset:clong; whence:cint):cint; cdecl;
    tell : function (p:Ppxdoc_t; stream:Ppxstream_t):clong; cdecl;
    write : function (p:Ppxdoc_t; stream:Ppxstream_t; numbytes:size_t; buffer:pointer):size_t; cdecl;
    targetencoding : pcchar;
    inputencoding : pcchar;
    out_iconvcd : iconv_t;
    in_iconvcd : iconv_t;
    curblocknr : clong;
    curblockdirty : cint;
    curblock : pcuchar;
  end;
  pxdoc_t = px_doc;

  px_blockcache = record
    start : clong;
    size : size_t;
    data : pcuchar;
  end;
  pxblockcache_t = px_blockcache;  

  Ppx_mbblockinfo = ^px_mbblockinfo;
  px_mbblockinfo = record
    number : cint;
    _type : cchar;
    numblobs : cchar;
    numblocks : cint;
    allocspace : cint;
  end;
  pxmbblockinfo_t = px_mbblockinfo;

  px_blob = record
    mb_name : pcchar;
    pxdoc : Ppxdoc_t;
    mb_stream : Ppxstream_t;
    mb_head : Pmbhead_t;
    used_datablocks : cint;
    subblockoffset : cint;
    subblockinneroffset : cint;
    subblockfree : cint;
    subblockblobcount : cint;
    read : function (p:Ppxblob_t; stream:Ppxstream_t; numbytes:size_t; buffer:pointer):size_t; cdecl;
    seek : function (p:Ppxblob_t; stream:Ppxstream_t; offset:clong; whence:cint):cint; cdecl;
    tell : function (p:Ppxblob_t; stream:Ppxstream_t):clong; cdecl;
    write : function (p:Ppxblob_t; stream:Ppxstream_t; numbytes:size_t; buffer:pointer):size_t; cdecl;
    blockcache : pxblockcache_t;
    blocklist : Ppxmbblockinfo_t;
    blocklistlen : cint;
  end;
  pxblob_t = px_blob;  

  Pmb_head = ^mb_head;
  mb_head = record
    modcount : cint;
  end;
  mbhead_t = mb_head;

  Ppx_datablockinfo = ^px_datablockinfo;
  px_datablockinfo = record
    blockpos : clong;
    recordpos : clong;
    size : cint;
    recno : cint;
    numrecords : cint;
    prev : cint;
    next : cint;
    number : cint;
  end;
  pxdatablockinfo_t = px_datablockinfo;

  Ppx_pindex = ^px_pindex;
  px_pindex = record
    data : pcchar;
    blocknumber : cint;
    numrecords : cint;
    dummy : cint;
    myblocknumber : cint;
    level : cint;
  end;
  pxpindex_t = px_pindex;

  TPXErrorhandler = procedure (p:Ppxdoc_t; _type:cint; msg:pcchar; data:pointer); cdecl;
  TPXAllocHandler = function (p:Ppxdoc_t; size:size_t; caller:pcchar):pointer; cdecl;
  TPXReAllocHandler = function (p:Ppxdoc_t; mem:pointer; size:size_t; caller:pcchar):pointer;cdecl;
  TPXFreeHandler = procedure (p:Ppxdoc_t; mem:pointer); cdecl;

var
  PX_get_majorversion : function:cint;cdecl;
  PX_get_minorversion : function:cint;cdecl;
  PX_get_subminorversion : function:cint;cdecl;
  PX_has_recode_support : function:cint;cdecl;
  PX_has_gsf_support : function:cint;cdecl;
  PX_is_bigendian : function:cint;cdecl;
  PX_get_builddate : function:pcchar;cdecl;
  PX_boot : procedure;cdecl;
  PX_shutdown : procedure;cdecl;
  PX_new3 : function(errorhandler: TPXErrorHandler; 
                     allocproc: TPXAllocHandler;
                     reallocproc: TPXReallocHandler; 
                     freeproc : TPXFreeHandler;
                     errorhandler_user_data : pointer) : Ppxdoc_t; cdecl;
  PX_new2 : function(errorhandler: TPXErrorHandler;
                     allocproc: TPXAllocHandler;
                     reallocproc: TPXReallocHandler;
                     freeproc : TPXFreeHandler) : Ppxdoc_t; cdecl;
  PX_new : function:Ppxdoc_t;cdecl;
  PX_open_fp : function(pxdoc:Ppxdoc_t; fp:PFILE):cint;cdecl;
  PX_open_file : function(pxdoc:Ppxdoc_t; filename:pcchar):cint;cdecl;
  PX_create_file : function(pxdoc:Ppxdoc_t; pxf:Ppxfield_t; numfields:cint; filename:pcchar; _type:cint):cint;cdecl;
  PX_create_fp : function(pxdoc:Ppxdoc_t; pxf:Ppxfield_t; numfields:cint; fp:PFILE; _type:cint):cint;cdecl;
  PX_get_opaque : function(pxdoc:Ppxdoc_t):pointer;cdecl;
  PX_write_primary_index : function(pxdoc:Ppxdoc_t; pxindex:Ppxdoc_t):cint;cdecl;
  PX_read_primary_index : function(pindex:Ppxdoc_t):cint;cdecl;
  PX_add_primary_index : function(pxdoc:Ppxdoc_t; pindex:Ppxdoc_t):cint;cdecl;
  PX_get_record : function(pxdoc:Ppxdoc_t; recno:cint; data:pcchar):pcchar;cdecl;
  PX_get_record2 : function(pxdoc:Ppxdoc_t; recno:cint; data:pcchar; deleted:pcint; pxdbinfo:Ppxdatablockinfo_t):pcchar;cdecl;
  PX_put_recordn : function(pxdoc:Ppxdoc_t; data:pcchar; recpos:cint):cint;cdecl;
  PX_put_record : function(pxdoc:Ppxdoc_t; data:pcchar):cint;cdecl;
  PX_insert_record : function(pxdoc:Ppxdoc_t; dataptr:PPpxval_t):cint;cdecl;
  PX_update_record : function(pxdoc:Ppxdoc_t; dataptr:PPpxval_t; recno:cint):cint;cdecl;
  PX_delete_record : function(pxdoc:Ppxdoc_t; recno:cint):cint;cdecl;
  PX_retrieve_record : function(pxdoc:Ppxdoc_t; recno:cint):PPpxval_t;cdecl;
  PX_close : procedure(pxdoc:Ppxdoc_t);cdecl;
  PX_delete : procedure(pxdoc:Ppxdoc_t);cdecl;
  PX_pack : function(pxdoc:Ppxdoc_t):cint;cdecl;
  PX_get_fields : function(pxdoc:Ppxdoc_t):Ppxfield_t;cdecl;
  PX_get_field : function(pxdoc:Ppxdoc_t; i:cint):Ppxfield_t;cdecl;
  PX_get_num_fields : function(pxdoc:Ppxdoc_t):cint;cdecl;
  PX_get_num_records : function(pxdoc:Ppxdoc_t):cint;cdecl;
  PX_get_recordsize : function(pxdoc:Ppxdoc_t):cint;cdecl;
  PX_set_parameter : function(pxdoc:Ppxdoc_t; name:pcchar; value:pcchar):cint;cdecl;
  PX_get_parameter : function(pxdoc:Ppxdoc_t; name:pcchar; value:Ppcchar):cint;cdecl;
  PX_set_value : function(pxdoc:Ppxdoc_t; name:pcchar; value:double):cint;cdecl;
  PX_get_value : function(pxdoc:Ppxdoc_t; name:pcchar; value:Pdouble):cint;cdecl;
  PX_set_targetencoding : function(pxdoc:Ppxdoc_t; encoding:pcchar):cint;cdecl;
  PX_set_inputencoding : function(pxdoc:Ppxdoc_t; encoding:pcchar):cint;cdecl;
  PX_set_tablename : function(pxdoc:Ppxdoc_t; tablename:pcchar):cint;cdecl;
  PX_set_blob_file : function(pxdoc:Ppxdoc_t; filename:pcchar):cint;cdecl;
  PX_set_blob_fp : function(pxdoc:Ppxdoc_t; fp:PFILE):cint;cdecl;
  PX_has_blob_file : function(pxdoc:Ppxdoc_t):cint;cdecl;
  PX_new_blob : function(pxdoc:Ppxdoc_t):Ppxblob_t;cdecl;
  PX_open_blob_fp : function(pxdoc:Ppxblob_t; fp:PFILE):cint;cdecl;
  PX_open_blob_file : function(pxdoc:Ppxblob_t; filename:pcchar):cint;cdecl;
  PX_create_blob_fp : function(pxdoc:Ppxblob_t; fp:PFILE):cint;cdecl;
  PX_create_blob_file : function(pxblob:Ppxblob_t; filename:pcchar):cint;cdecl;
  PX_close_blob : procedure(pxdoc:Ppxblob_t);cdecl;
  PX_delete_blob : procedure(pxblob:Ppxblob_t);cdecl;
  PX_read_blobdata : function(pxblob:Ppxblob_t; data:pcchar; len:cint; _mod:pcint; blobsize:pcint):pcchar;cdecl;
  PX_read_graphicdata : function(pxblob:Ppxblob_t; data:pcchar; len:cint; _mod:pcint; blobsize:pcint):pcchar;cdecl;
  PX_read_grahicdata : function(pxblob:Ppxblob_t; data:pcchar; len:cint; _mod:pcint; blobsize:pcint):pcchar;cdecl;
  PX_get_data_alpha : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:Ppcchar):cint;cdecl;
  PX_get_data_bytes : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:Ppcchar):cint;cdecl;
  PX_get_data_double : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:Pdouble):cint;cdecl;
  PX_get_data_long : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pclong):cint;cdecl;
<<<<<<< HEAD
<<<<<<< HEAD
  PX_get_data_short : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcsshort):cint;cdecl;
=======
  PX_get_data_short : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcsint):cint;cdecl;
>>>>>>> graemeg/fixes_2_2
=======
  PX_get_data_short : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcsint):cint;cdecl;
>>>>>>> origin/fixes_2_2
  PX_get_data_byte : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar):cint;cdecl;
  PX_get_data_bcd : function(pxdoc:Ppxdoc_t; data:pcuchar; len:cint; value:Ppcchar):cint;cdecl;
  PX_get_data_blob : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; _mod:pcint; blobsize:pcint; 
    value:Ppcchar):cint;cdecl;
  PX_get_data_graphic : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; _mod:pcint; blobsize:pcint; 
    value:Ppcchar):cint;cdecl;
  PX_put_data_alpha : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar);cdecl;
  PX_put_data_bytes : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar);cdecl;
  PX_put_data_double : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:double);cdecl;
  PX_put_data_long : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:cint);cdecl;
<<<<<<< HEAD
<<<<<<< HEAD
  PX_put_data_short : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:csshort);cdecl;
  PX_put_data_byte : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:cchar);cdecl;
  PX_put_data_bcd : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar);cdecl;
  PX_put_data_blob : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar; valuelen:cint):cint;cdecl;
{$ifndef windows}
  PX_SdnToGregorian : procedure(sdn:clong; pYear:pcint; pMonth:pcint; pDay:pcint);cdecl;
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
  PX_put_data_short : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:csint);cdecl;
  PX_put_data_byte : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:cchar);cdecl;
  PX_put_data_bcd : procedure(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar);cdecl;
  PX_put_data_blob : function(pxdoc:Ppxdoc_t; data:pcchar; len:cint; value:pcchar; valuelen:cint):cint;cdecl;
  PX_SdnToGregorian : procedure(sdn:clong; pYear:pcint; pMonth:pcint; pDay:pcint);cdecl;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  PX_GregorianToSdn : function(year:cint; month:cint; day:cint):clong;cdecl;
  PX_make_time : function(pxdoc:Ppxdoc_t; hour:cint; minute:cint; second:cint):Ppxval_t;cdecl;
  PX_make_date : function(pxdoc:Ppxdoc_t; year:cint; month:cint; day:cint):Ppxval_t;cdecl;
  PX_make_timestamp : function(pxdoc:Ppxdoc_t; year:cint; month:cint; day:cint; hour:cint; 
    minute:cint; second:cint):Ppxval_t;cdecl;
  PX_timestamp2string : function(pxdoc:Ppxdoc_t; value:double; format:pcchar):pcchar;cdecl;
  PX_time2string : function(pxdoc:Ppxdoc_t; value:clong; format:pcchar):pcchar;cdecl;
  PX_date2string : function(pxdoc:Ppxdoc_t; value:clong; format:pcchar):pcchar;cdecl;
  PX_strdup : function(pxdoc:Ppxdoc_t; str:pcchar):pcchar;cdecl;

procedure Freepxlib;
Procedure Loadpxlib(lib : String);

implementation

uses
  SysUtils, dynlibs;
  
var
  hlib : tlibhandle;

Procedure Freepxlib;

begin
  if (HLib<>NilHandle) then
    begin
    FreeLibrary(hlib);
    hlib:=Nilhandle;
    end;
  PX_get_majorversion:=nil;
  PX_get_minorversion:=nil;
  PX_get_subminorversion:=nil;
  PX_has_recode_support:=nil;
  PX_has_gsf_support:=nil;
  PX_is_bigendian:=nil;
  PX_get_builddate:=nil;
  PX_boot:=nil;
  PX_shutdown:=nil;
  PX_new3:=nil;
  PX_new2:=nil;
  PX_new:=nil;
  PX_open_fp:=nil;
  PX_open_file:=nil;
  PX_create_file:=nil;
  PX_create_fp:=nil;
  PX_get_opaque:=nil;
  PX_write_primary_index:=nil;
  PX_read_primary_index:=nil;
  PX_add_primary_index:=nil;
  PX_get_record:=nil;
  PX_get_record2:=nil;
  PX_put_recordn:=nil;
  PX_put_record:=nil;
  PX_insert_record:=nil;
  PX_update_record:=nil;
  PX_delete_record:=nil;
  PX_retrieve_record:=nil;
  PX_close:=nil;
  PX_delete:=nil;
  PX_pack:=nil;
  PX_get_fields:=nil;
  PX_get_field:=nil;
  PX_get_num_fields:=nil;
  PX_get_num_records:=nil;
  PX_get_recordsize:=nil;
  PX_set_parameter:=nil;
  PX_get_parameter:=nil;
  PX_set_value:=nil;
  PX_get_value:=nil;
  PX_set_targetencoding:=nil;
  PX_set_inputencoding:=nil;
  PX_set_tablename:=nil;
  PX_set_blob_file:=nil;
  PX_set_blob_fp:=nil;
  PX_has_blob_file:=nil;
  PX_new_blob:=nil;
  PX_open_blob_fp:=nil;
  PX_open_blob_file:=nil;
  PX_create_blob_fp:=nil;
  PX_create_blob_file:=nil;
  PX_close_blob:=nil;
  PX_delete_blob:=nil;
  PX_read_blobdata:=nil;
  PX_read_graphicdata:=nil;
  PX_read_grahicdata:=nil;
  PX_get_data_alpha:=nil;
  PX_get_data_bytes:=nil;
  PX_get_data_double:=nil;
  PX_get_data_long:=nil;
  PX_get_data_short:=nil;
  PX_get_data_byte:=nil;
  PX_get_data_bcd:=nil;
  PX_get_data_blob:=nil;
  PX_get_data_graphic:=nil;
  PX_put_data_alpha:=nil;
  PX_put_data_bytes:=nil;
  PX_put_data_double:=nil;
  PX_put_data_long:=nil;
  PX_put_data_short:=nil;
  PX_put_data_byte:=nil;
  PX_put_data_bcd:=nil;
  PX_put_data_blob:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef windows}
  PX_SdnToGregorian:=nil;
{$endif}  
=======
  PX_SdnToGregorian:=nil;
>>>>>>> graemeg/fixes_2_2
=======
  PX_SdnToGregorian:=nil;
>>>>>>> origin/fixes_2_2
  PX_GregorianToSdn:=nil;
  PX_make_time:=nil;
  PX_make_date:=nil;
  PX_make_timestamp:=nil;
  PX_timestamp2string:=nil;
  PX_time2string:=nil;
  PX_date2string:=nil;
  PX_strdup:=nil;
end;

Procedure Loadpxlib(lib : String);

begin
  Freepxlib;
  hlib:=LoadLibrary(Pchar(lib));
  if hlib=0 then
    raise Exception.Create(format('Could not load library: %s',[lib]));
  pointer(PX_get_majorversion):=GetProcAddress(hlib,'PX_get_majorversion');
  pointer(PX_get_minorversion):=GetProcAddress(hlib,'PX_get_minorversion');
  pointer(PX_get_subminorversion):=GetProcAddress(hlib,'PX_get_subminorversion');
  pointer(PX_has_recode_support):=GetProcAddress(hlib,'PX_has_recode_support');
  pointer(PX_has_gsf_support):=GetProcAddress(hlib,'PX_has_gsf_support');
  pointer(PX_is_bigendian):=GetProcAddress(hlib,'PX_is_bigendian');
  pointer(PX_get_builddate):=GetProcAddress(hlib,'PX_get_builddate');
  pointer(PX_boot):=GetProcAddress(hlib,'PX_boot');
  pointer(PX_shutdown):=GetProcAddress(hlib,'PX_shutdown');
  pointer(PX_new3):=GetProcAddress(hlib,'PX_new3');
  pointer(PX_new2):=GetProcAddress(hlib,'PX_new2');
  pointer(PX_new):=GetProcAddress(hlib,'PX_new');
  pointer(PX_open_fp):=GetProcAddress(hlib,'PX_open_fp');
  pointer(PX_open_file):=GetProcAddress(hlib,'PX_open_file');
  pointer(PX_create_file):=GetProcAddress(hlib,'PX_create_file');
  pointer(PX_create_fp):=GetProcAddress(hlib,'PX_create_fp');
  pointer(PX_get_opaque):=GetProcAddress(hlib,'PX_get_opaque');
  pointer(PX_write_primary_index):=GetProcAddress(hlib,'PX_write_primary_index');
  pointer(PX_read_primary_index):=GetProcAddress(hlib,'PX_read_primary_index');
  pointer(PX_add_primary_index):=GetProcAddress(hlib,'PX_add_primary_index');
  pointer(PX_get_record):=GetProcAddress(hlib,'PX_get_record');
  pointer(PX_get_record2):=GetProcAddress(hlib,'PX_get_record2');
  pointer(PX_put_recordn):=GetProcAddress(hlib,'PX_put_recordn');
  pointer(PX_put_record):=GetProcAddress(hlib,'PX_put_record');
  pointer(PX_insert_record):=GetProcAddress(hlib,'PX_insert_record');
  pointer(PX_update_record):=GetProcAddress(hlib,'PX_update_record');
  pointer(PX_delete_record):=GetProcAddress(hlib,'PX_delete_record');
  pointer(PX_retrieve_record):=GetProcAddress(hlib,'PX_retrieve_record');
  pointer(PX_close):=GetProcAddress(hlib,'PX_close');
  pointer(PX_delete):=GetProcAddress(hlib,'PX_delete');
  pointer(PX_pack):=GetProcAddress(hlib,'PX_pack');
  pointer(PX_get_fields):=GetProcAddress(hlib,'PX_get_fields');
  pointer(PX_get_field):=GetProcAddress(hlib,'PX_get_field');
  pointer(PX_get_num_fields):=GetProcAddress(hlib,'PX_get_num_fields');
  pointer(PX_get_num_records):=GetProcAddress(hlib,'PX_get_num_records');
  pointer(PX_get_recordsize):=GetProcAddress(hlib,'PX_get_recordsize');
  pointer(PX_set_parameter):=GetProcAddress(hlib,'PX_set_parameter');
  pointer(PX_get_parameter):=GetProcAddress(hlib,'PX_get_parameter');
  pointer(PX_set_value):=GetProcAddress(hlib,'PX_set_value');
  pointer(PX_get_value):=GetProcAddress(hlib,'PX_get_value');
  pointer(PX_set_targetencoding):=GetProcAddress(hlib,'PX_set_targetencoding');
  pointer(PX_set_inputencoding):=GetProcAddress(hlib,'PX_set_inputencoding');
  pointer(PX_set_tablename):=GetProcAddress(hlib,'PX_set_tablename');
  pointer(PX_set_blob_file):=GetProcAddress(hlib,'PX_set_blob_file');
  pointer(PX_set_blob_fp):=GetProcAddress(hlib,'PX_set_blob_fp');
  pointer(PX_has_blob_file):=GetProcAddress(hlib,'PX_has_blob_file');
  pointer(PX_new_blob):=GetProcAddress(hlib,'PX_new_blob');
  pointer(PX_open_blob_fp):=GetProcAddress(hlib,'PX_open_blob_fp');
  pointer(PX_open_blob_file):=GetProcAddress(hlib,'PX_open_blob_file');
  pointer(PX_create_blob_fp):=GetProcAddress(hlib,'PX_create_blob_fp');
  pointer(PX_create_blob_file):=GetProcAddress(hlib,'PX_create_blob_file');
  pointer(PX_close_blob):=GetProcAddress(hlib,'PX_close_blob');
  pointer(PX_delete_blob):=GetProcAddress(hlib,'PX_delete_blob');
  pointer(PX_read_blobdata):=GetProcAddress(hlib,'PX_read_blobdata');
  pointer(PX_read_graphicdata):=GetProcAddress(hlib,'PX_read_graphicdata');
  pointer(PX_read_grahicdata):=GetProcAddress(hlib,'PX_read_grahicdata');
  pointer(PX_get_data_alpha):=GetProcAddress(hlib,'PX_get_data_alpha');
  pointer(PX_get_data_bytes):=GetProcAddress(hlib,'PX_get_data_bytes');
  pointer(PX_get_data_double):=GetProcAddress(hlib,'PX_get_data_double');
  pointer(PX_get_data_long):=GetProcAddress(hlib,'PX_get_data_long');
  pointer(PX_get_data_short):=GetProcAddress(hlib,'PX_get_data_short');
  pointer(PX_get_data_byte):=GetProcAddress(hlib,'PX_get_data_byte');
  pointer(PX_get_data_bcd):=GetProcAddress(hlib,'PX_get_data_bcd');
  pointer(PX_get_data_blob):=GetProcAddress(hlib,'PX_get_data_blob');
  pointer(PX_get_data_graphic):=GetProcAddress(hlib,'PX_get_data_graphic');
  pointer(PX_put_data_alpha):=GetProcAddress(hlib,'PX_put_data_alpha');
  pointer(PX_put_data_bytes):=GetProcAddress(hlib,'PX_put_data_bytes');
  pointer(PX_put_data_double):=GetProcAddress(hlib,'PX_put_data_double');
  pointer(PX_put_data_long):=GetProcAddress(hlib,'PX_put_data_long');
  pointer(PX_put_data_short):=GetProcAddress(hlib,'PX_put_data_short');
  pointer(PX_put_data_byte):=GetProcAddress(hlib,'PX_put_data_byte');
  pointer(PX_put_data_bcd):=GetProcAddress(hlib,'PX_put_data_bcd');
  pointer(PX_put_data_blob):=GetProcAddress(hlib,'PX_put_data_blob');
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef windows}
  pointer(PX_SdnToGregorian):=GetProcAddress(hlib,'PX_SdnToGregorian');
{$endif windows}  
=======
  pointer(PX_SdnToGregorian):=GetProcAddress(hlib,'PX_SdnToGregorian');
>>>>>>> graemeg/fixes_2_2
=======
  pointer(PX_SdnToGregorian):=GetProcAddress(hlib,'PX_SdnToGregorian');
>>>>>>> origin/fixes_2_2
  pointer(PX_GregorianToSdn):=GetProcAddress(hlib,'PX_GregorianToSdn');
  pointer(PX_make_time):=GetProcAddress(hlib,'PX_make_time');
  pointer(PX_make_date):=GetProcAddress(hlib,'PX_make_date');
  pointer(PX_make_timestamp):=GetProcAddress(hlib,'PX_make_timestamp');
  pointer(PX_timestamp2string):=GetProcAddress(hlib,'PX_timestamp2string');
  pointer(PX_time2string):=GetProcAddress(hlib,'PX_time2string');
  pointer(PX_date2string):=GetProcAddress(hlib,'PX_date2string');
  pointer(PX_strdup):=GetProcAddress(hlib,'PX_strdup');
end;


finalization
  Freepxlib;
end.
