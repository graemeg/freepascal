%{
var
  kwtmp: integer;

const
  KeywordDefs: array [0..33] of TIdentMapEntry = (
    // attribs
    (Value: _LANGUAGE;             Name: 'LANGUAGE'),
    (Value: _CHARACTERISTICS;      Name: 'CHARACTERISTICS'),
    (Value: _VERSION;              Name: 'VERSION'),
    (Value: _MOVEABLE;             Name: 'MOVEABLE'),
    (Value: _FIXED;                Name: 'FIXED'),
    (Value: _PURE;                 Name: 'PURE'),
    (Value: _IMPURE;               Name: 'IMPURE'),
    (Value: _PRELOAD;              Name: 'PRELOAD'),
    (Value: _LOADONCALL;           Name: 'LOADONCALL'),
    (Value: _DISCARDABLE;          Name: 'DISCARDABLE'),
    // resource names
    (Value: _ANICURSOR;            Name: 'ANICURSOR'),
    (Value: _ANIICON;              Name: 'ANIICON'),
    (Value: _BITMAP;               Name: 'BITMAP'),
    (Value: _CURSOR;               Name: 'CURSOR'),
    (Value: _DLGINCLUDE;           Name: 'DLGINCLUDE'),
    (Value: _DLGINIT;              Name: 'DLGINIT'),
    (Value: _HTML;                 Name: 'HTML'),
    (Value: _ICON;                 Name: 'ICON'),
    (Value: _MANIFEST;             Name: 'MANIFEST'),
    (Value: _MESSAGETABLE;         Name: 'MESSAGETABLE'),
    (Value: _PLUGPLAY;             Name: 'PLUGPLAY'),
    (Value: _STRINGTABLE;          Name: 'STRINGTABLE'),
    (Value: _RCDATA;               Name: 'RCDATA'),
    (Value: _VERSIONINFO;          Name: 'VERSIONINFO'),
    (Value: _VXD;                  Name: 'VXD'),
    // file version fields names
    (Value: _FILEVERSION;          Name: 'FILEVERSION'),
    (Value: _PRODUCTVERSION;       Name: 'PRODUCTVERSION'),
    (Value: _FILEFLAGSMASK;        Name: 'FILEFLAGSMASK'),
    (Value: _FILEFLAGS;            Name: 'FILEFLAGS'),
    (Value: _FILEOS;               Name: 'FILEOS'),
    (Value: _FILETYPE;             Name: 'FILETYPE'),
    (Value: _FILESUBTYPE;          Name: 'FILESUBTYPE'),
    (Value: _BLOCK;                Name: 'BLOCK'),
    (Value: _VALUE;                Name: 'VALUE')
  );

function KeywordToInt(k: string; out kw: integer): boolean;
var
  i: integer;
begin
  Result:= False;
  for i:= low(KeywordDefs) to high(KeywordDefs) do begin
    if k = KeywordDefs[i].Name then begin
      kw:= KeywordDefs[i].Value;
      Exit(True);
    end;
  end;
end;

%}
%x INCOMLINE INCOMMENT INSTRING INSTRINGL

O [0-7]
D [0-9]
H [0-9a-fA-F]
IDENT [a-zA-Z_]([a-zA-Z0-9_])*
NSWPSTR ([a-zA-Z0-9_\:\.\\])*

%%

"//"                    start(INCOMLINE);
<INCOMLINE>\n           begin start(0); unget_char(nl); end;
<INCOMLINE>.            yymore;

"/*"                    start(INCOMMENT);
<INCOMMENT>.            ;
<INCOMMENT>"*/"         start(0);
<INCOMMENT>\0           return(_ILLEGAL);

{D}+L?                  return(_NUMBER);
0x{H}+L?                return(_NUMBER);
0o{O}+L?                return(_NUMBER);
L\"                               begin start(INSTRINGL); strbuf_begin(); end;
\"                                begin start(INSTRING); strbuf_begin(); end;
<INSTRING,INSTRINGL>\"\"          strbuf_append('"');
<INSTRING>\"                      begin
                                    start(0);
                                    return(_QUOTEDSTR);
                                  end;
<INSTRINGL>\"                     begin
                                    start(0);
                                    return(_QUOTEDSTRL);
                                  end;
<INSTRING,INSTRINGL>\\\n          ;
<INSTRING,INSTRINGL>\n            return(_ILLEGAL);
<INSTRING,INSTRINGL>.             strbuf_append(yytext);
\"StringFileInfo\"      begin yytext:= 'StringFileInfo'; return(_STR_StringFileInfo); end;
\"VarFileInfo\"         begin yytext:= 'VarFileInfo'; return(_STR_VarFileInfo); end;
\"Translation\"         begin yytext:= 'Translation'; return(_STR_Translation); end;

BEGIN|{                 return(_BEGIN);
END|}                   return(_END);
{IDENT}                 begin
                          if ypreproc.isdefine(yytext) then begin
                            unget_char(' ');
                            unget_string(ypreproc.getdefine(yytext));
                          end else
                            if KeywordToInt(yytext, kwtmp) then
                              return(kwtmp)
                            else
                              return(_ID);
                         end;
{NSWPSTR}         return(_NSWPSTR);
[ \t\n\f]               ;
[,()|^&+-*/%~]          returnc(yytext[1]);
.                       return(_ILLEGAL);
%%

// end.





