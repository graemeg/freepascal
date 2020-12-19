%{
(*
Vorspann
 ****************************************************************************)

unit rcparser;

{$I rcparserfn.inc}

%}

%token _ILLEGAL
%token _NUMBER _QUOTEDSTR _QUOTEDSTRL
%token _STR_StringFileInfo _STR_VarFileInfo _STR_Translation
%token _BEGIN _END _ID _NSWPSTR
%token _LANGUAGE _CHARACTERISTICS _VERSION _MOVEABLE _FIXED _PURE _IMPURE _PRELOAD _LOADONCALL _DISCARDABLE
%token _BITMAP _CURSOR _ICON _STRINGTABLE _VERSIONINFO
%token _ANICURSOR _ANIICON _DLGINCLUDE _DLGINIT _HTML _MANIFEST _MESSAGETABLE _PLUGPLAY _RCDATA _VXD
%token _FILEVERSION _PRODUCTVERSION _FILEFLAGSMASK _FILEFLAGS _FILEOS _FILETYPE _FILESUBTYPE _BLOCK _VALUE
%token _ACCELERATORS _DIALOG _DIALOGEX _MENU _MENUEX

%type <rcnumtype> numpos numexpr numeral
%type <rcstrtype> ident_string long_string non_whitespace_string long_stringfn
%type <TResourceDesc> resid rcdataid
%type <TMemoryStream> raw_data raw_item
%type <TFileStream> filename_string

%left '|'
%left '^'
%left '&'
%left '+' '-'
%left '*' '/' '%'
%right '~' _NUMNEG

%%

rcfile
    : /* empty */
    | rcfile defnstatement
    ;

defnstatement
    : resourcedef
    | languagedef
    ;

resourcedef
    : res_stringtable
    | res_bitmap
    | res_cursor
    | res_icon
    | res_version
    | res_rcdata
    ;

res_bitmap
    : resid _BITMAP { create_resource($1, RT_BITMAP); } suboptions filename_string            { TBitmapResource(aktresource).SetCustomBitmapDataStream($5); }

res_cursor
    : resid _CURSOR { create_resource($1, RT_CURSOR); } suboptions filename_string            { TGroupCursorResource(aktresource).SetCustomItemDataStream($5); }

res_icon
    : resid _ICON { create_resource($1, RT_ICON); } suboptions filename_string                { TGroupIconResource(aktresource).SetCustomItemDataStream($5); }

res_version
    : resid _VERSIONINFO { create_resource($1, RT_VERSION); } version_fixed _BEGIN version_blocks _END

version_fixed
    : /* empty */
    | version_fixed _FILEVERSION numeral ',' numeral ',' numeral ',' numeral                  { TVersionResource(aktresource).FixedInfo.FileVersion:= make_version($3.v, $5.v, $7.v, $9.v); }
    | version_fixed _PRODUCTVERSION numeral ',' numeral ',' numeral ',' numeral               { TVersionResource(aktresource).FixedInfo.ProductVersion:= make_version($3.v, $5.v, $7.v, $9.v); }
    | version_fixed _FILEFLAGSMASK numpos                                                     { TVersionResource(aktresource).FixedInfo.FileFlagsMask:= $3.v; }
    | version_fixed _FILEFLAGS numpos                                                         { TVersionResource(aktresource).FixedInfo.FileFlags:= $3.v; }
    | version_fixed _FILEOS numpos                                                            { TVersionResource(aktresource).FixedInfo.FileOS:= $3.v; }
    | version_fixed _FILETYPE numpos                                                          { TVersionResource(aktresource).FixedInfo.FileType:= $3.v; }
    | version_fixed _FILESUBTYPE numpos                                                       { TVersionResource(aktresource).FixedInfo.FileSubType:= $3.v; }
    ;

version_blocks
    : /* empty */
    | version_blocks _BLOCK _STR_StringFileInfo _BEGIN ver_strings_lang _END
    | version_blocks _BLOCK _STR_VarFileInfo _BEGIN ver_translation_data _END
    ;

ver_strings_lang
    : /* empty */
    | ver_strings_lang _BLOCK long_string _BEGIN                                              { version_string_tab_begin($3.v^); }
                                          ver_strings_data _END
    ;

ver_strings_data
    : /* empty */
    | ver_strings_data _VALUE long_string ',' long_string                                     { version_string_tab_add($3.v^, $5.v^); }
    ;

ver_translation_data
    : _VALUE _STR_Translation ',' ver_translation_pair
    ;

ver_translation_pair
    : numeral ',' numeral                                                                     { version_var_translation_add($1.v, $3.v); }
    | ver_translation_pair ',' numeral ',' numeral                                            { version_var_translation_add($3.v, $5.v); }
    ;

res_rcdata
    : resid rcdataid { create_resource($1, $2); } suboptions filename_string                  { aktresource.SetCustomRawDataStream($5); }
    | resid rcdataid { create_resource($1, $2); } suboptions _BEGIN raw_data _END             { aktresource.SetCustomRawDataStream($6); }
    ;

res_stringtable
    : _STRINGTABLE { stringtable_begin(); } suboptions _BEGIN stringtable_data _END { stringtable_end(); }

stringtable_data
    : /* empty */
    | stringtable_data stringtable_entry
    ;

stringtable_entry
    : numeral ',' long_string                      { stringtable_add($1.v, $3.v^); }
    | numeral long_string                          { stringtable_add($1.v, $2.v^); }
    ;

rcdataid
    : _ANICURSOR                                   { $$:= TResourceDesc.Create(RT_ANICURSOR); }
    | _ANIICON                                     { $$:= TResourceDesc.Create(RT_ANIICON); }
    | _DLGINCLUDE                                  { $$:= TResourceDesc.Create(RT_DLGINCLUDE); }
    | _DLGINIT                                     { $$:= TResourceDesc.Create(RT_DLGINIT); }
    | _HTML                                        { $$:= TResourceDesc.Create(23); }
    | _MANIFEST                                    { $$:= TResourceDesc.Create(RT_MANIFEST); }
    | _MESSAGETABLE                                { $$:= TResourceDesc.Create(RT_MESSAGETABLE); }
    | _PLUGPLAY                                    { $$:= TResourceDesc.Create(RT_PLUGPLAY); }
    | _RCDATA                                      { $$:= TResourceDesc.Create(RT_RCDATA); }
    | _VXD                                         { $$:= TResourceDesc.Create(RT_VXD); }
    | resid
    ;

resid
    : numpos                                       { $$:= TResourceDesc.Create($1.v); }
    | ident_string                                 { $$:= TResourceDesc.Create($1.v^); }
    ;

suboptions
    : /* empty */
    | suboptions suboption
    ;

suboption
    : _LANGUAGE numpos ',' numpos                  { change_lang_id(MakeLangID($2.v, $4.v)); }
    | _CHARACTERISTICS numpos                      { aktresource.Characteristics:= $2.v; }
    | _VERSION numpos                              { aktresource.Version:= $2.v; }
    | _MOVEABLE                                    { aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_MOVEABLE; }
    | _FIXED                                       { aktresource.MemoryFlags:= aktresource.MemoryFlags and not MF_MOVEABLE; }
    | _PURE                                        { aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_PURE; }
    | _IMPURE                                      { aktresource.MemoryFlags:= aktresource.MemoryFlags and not MF_PURE; }
    | _PRELOAD                                     { aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_PRELOAD; }
    | _LOADONCALL                                  { aktresource.MemoryFlags:= aktresource.MemoryFlags and not MF_PRELOAD; }
    | _DISCARDABLE                                 { aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_DISCARDABLE; }
    ;

languagedef
    : _LANGUAGE numpos ',' numpos                  { language:= MakeLangID($2.v, $4.v); }

numpos
    : numexpr
    ;

numeral
    : _NUMBER                                      { $$:= str_to_num(yytext); }
    ;

numexpr
    : numeral
    | '(' numexpr ')'                              { $$:= $2; }
    | '~' numexpr %prec '~'                        { $$.v:= not $2.v; $$.long:= $2.long; }
    | '-' numexpr %prec _NUMNEG                    { $$.v:= -$2.v; $$.long:= $2.long; }
    | numexpr '*' numexpr                          { $$.v:= $1.v * $3.v; $$.long:= $1.long or $3.long; }
    | numexpr '/' numexpr                          { $$.v:= $1.v div Max(1, $3.v); $$.long:= $1.long or $3.long; }
    | numexpr '%' numexpr                          { $$.v:= $1.v mod Max(1, $3.v); $$.long:= $1.long or $3.long; }
    | numexpr '+' numexpr                          { $$.v:= $1.v + $3.v; $$.long:= $1.long or $3.long; }
    | numexpr '-' numexpr                          { $$.v:= $1.v - $3.v; $$.long:= $1.long or $3.long; }
    | numexpr '&' numexpr                          { $$.v:= $1.v and $3.v; $$.long:= $1.long or $3.long; }
    | numexpr '^' numexpr                          { $$.v:= $1.v xor $3.v; $$.long:= $1.long or $3.long; }
    | numexpr '|' numexpr                          { $$.v:= $1.v or $3.v; $$.long:= $1.long or $3.long; }
    ;

ident_string
    : _ID                                          { string_new($$, yytext, opt_code_page); }
    | long_string
    ;

non_whitespace_string
    : _NSWPSTR                                     { string_new($$, yytext, opt_code_page); }
    ;

filename_string
    : long_stringfn                                { $$:= TFileStream.Create($1.v^, fmOpenRead or fmShareDenyWrite); }
    | non_whitespace_string                        { $$:= TFileStream.Create($1.v^, fmOpenRead or fmShareDenyWrite); }
    ;

long_stringfn
    : _QUOTEDSTR                                   { string_new_uni($$, @strbuf[0], strbuflen, opt_code_page, false); }
    | _QUOTEDSTRL                                  { string_new_uni($$, @strbuf[0], strbuflen, CP_UTF16, false); }
    | _STR_StringFileInfo                          { string_new($$, yytext, opt_code_page); }
    | _STR_VarFileInfo                             { string_new($$, yytext, opt_code_page); }
    | _STR_Translation                             { string_new($$, yytext, opt_code_page); }
    ;

long_string
    : _QUOTEDSTR                                   { string_new_uni($$, @strbuf[0], strbuflen, opt_code_page, true); }
    | _QUOTEDSTRL                                  { string_new_uni($$, @strbuf[0], strbuflen, CP_UTF16, true); }
    | _STR_StringFileInfo                          { string_new($$, yytext, opt_code_page); }
    | _STR_VarFileInfo                             { string_new($$, yytext, opt_code_page); }
    | _STR_Translation                             { string_new($$, yytext, opt_code_page); }
    ;

raw_data
    :                                              { $$:= TMemoryStream.Create; }
      raw_item
    | raw_data ',' { $$:= $1; } raw_item
    ;

raw_item
    : /* empty */
      {
        $$:= $<TMemoryStream>0;
      }
    | long_string
      {
        $$:= $<TMemoryStream>0;
        raw_write_string($$, $1);
      }
    | numeral
      {
        $$:= $<TMemoryStream>0;
        raw_write_int($$, $1);
      }
    ;

%%

{$I rclex.inc}
begin
  bufptr:= 0;
  lexlib.get_char:= @rc_get_char;
  lexlib.unget_char:= @rc_unget_char;
end.

