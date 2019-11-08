{
  Translation of the libxml2 headers for FreePascal
  Copyright (C) 2008 by Ivo Steinmann
}

unit xml2;

{$mode objfpc}
{$H+}
{$macro on}

{$ALIGN 8}
{$MINENUMSIZE 4}

interface

uses
  dynlibs,
  ctypes;

const
{$IF Defined(WINDOWS)}
  xml2lib = 'libxml2.'+sharedsuffix;
  {$DEFINE EXTDECL := cdecl}
  {$DEFINE NO_EXTERNAL_VARS}
{$ELSEIF Defined(UNIX)}
  xml2lib = 'libxml2.'+sharedsuffix;
  {$DEFINE EXTDECL := cdecl}
{$ELSE}
  {$MESSAGE ERROR 'Platform not supported right now'}
{$IFEND}

{$i xml2.inc}

implementation

{$IFDEF NO_EXTERNAL_VARS}
function GetxmlMalloc: xmlMallocFunc; inline;
begin
  Result := varxmlMalloc^;
end;

procedure SetxmlMalloc(AValue: xmlMallocFunc); inline;
begin
  varxmlMalloc^ := AValue;
end;

function GetxmlMallocAtomic: xmlMallocFunc; inline;
begin
  Result := varxmlMallocAtomic^;
end;

procedure SetxmlMallocAtomic(AValue: xmlMallocFunc); inline;
begin
  varxmlMallocAtomic^ := AValue;
end;

function GetxmlRealloc: xmlReallocFunc; inline;
begin
  Result := varxmlRealloc^;
end;

procedure SetxmlRealloc(AValue: xmlReallocFunc); inline;
begin
  varxmlRealloc^ := AValue;
end;

function GetxmlFree: xmlFreeFunc; inline;
begin
  Result := varxmlFree^;
end;

procedure SetxmlFree(AValue: xmlFreeFunc); inline;
begin
  varxmlFree^ := AValue;
end;

function GetxmlMemStrdup: xmlStrdupFunc; inline;
begin
  Result := varxmlMemStrdup^;
end;

procedure SetxmlMemStrdup(AValue: xmlStrdupFunc); inline;
begin
  varxmlMemStrdup^ := AValue;
end;
{$ENDIF}

procedure fpcxmlFree(mem: pointer); EXTDECL;
begin
  FreeMem(mem);
end;

function fpcxmlMalloc(size: csize_t): pointer; EXTDECL;
begin
  GetMem(Result, size);
end;

function fpcxmlRealloc(mem: pointer; size: csize_t): pointer; EXTDECL;
begin
  Result := mem;
  ReallocMem(Result, size);
end;

function fpcxmlStrdup(str: pchar): pchar; EXTDECL;
var
  L: SizeInt;
begin
  L := Length(str) + 1;
  Getmem(Result, L);
  if Result <> nil then
    Move(str^, Result^, L);
end;

procedure fpcxmlStructuredErrorHandler(userData: pointer; error: xmlErrorPtr); EXTDECL;
begin
  writeln('struct error');
end;


(*
 * macros from xmlversion.inc
 *)

procedure LIBXML_TEST_VERSION;
begin
  xmlCheckVersion(LIBXML_VERSION);
end;


(*
 * macros from xmlversion.inc
 *)


(*
 * macros from chvalid.inc
 *)

function xmlIsBaseChar_ch(c: cint): cbool;
begin
  Result :=
    ((c >= $41) and (c <= $5A)) or
    ((c >= $61) and (c <= $7A)) or
    ((c >= $C0) and (c <= $D6)) or
    ((c >= $D8) and (c <= $F6)) or
     (c >= $F8);
end;

function xmlIsBaseCharQ(c: cint): cbool;
begin
  if c < $100 then
    Result := xmlIsBaseChar_ch(c)
  else
    Result := xmlCharInRange(c, __xmlIsBaseCharGroup);
end;

function xmlIsBlank_ch(c: cint): cbool;
begin
  Result := (c = $20) or ((c >= $9) and (c <= $A)) or (c = $D);
end;

function xmlIsBlankQ(c: cint): cbool;
begin
  if c < $100 then
    Result := xmlIsBaseChar_ch(c)
  else
    Result := false;
end;

function xmlIsChar_ch(c: cint): cbool;
begin
  Result := ((c >= $9) and (c <= $A)) or (c = $D) or (c >= $20);
end;

function xmlIsCharQ(c: cint): cbool;
begin
  if c < $100 then
    Result := xmlIsChar_ch(c)
  else
    Result :=
          ((c >= $000100) and (c <= $00D7FF)) or
          ((c >= $00E000) and (c <= $00FFFD)) or
          ((c >= $010000) and (c <= $10FFFF));
end;

function xmlIsCombiningQ(c: cint): cbool;
begin
    if c < $100 then
    Result := false
  else
    Result := xmlCharInRange(c, __xmlIsCombiningGroup);
end;

function xmlIsDigit_ch(c: cint): cbool;
begin
  Result := (c >= $30) and (c <= $39);
end;

function xmlIsDigitQ(c: cint): cbool;
begin
  if c < $100 then
    Result := xmlIsDigit_ch(c)
  else
    Result := xmlCharInRange(c, __xmlIsDigitGroup);
end;

function xmlIsExtender_ch(c: cint): cbool;
begin
  Result := c = $B7;
end;

function xmlIsExtenderQ(c: cint): cbool;
begin
  if c < $100 then
    Result := xmlIsExtender_ch(c)
  else
    Result := xmlCharInRange(c, __xmlIsExtenderGroup);
end;

function xmlIsIdeographicQ(c: cint): cbool;
begin
  if c < $100 then
    Result := false
  else
    Result :=
      ((c >= $4E00) and (c <= $9FA5)) or
       (c  = $3007) or
      ((c >= $3021) and (c <= $3029));
end;

function xmlIsPubidChar_ch(c: cint): cbool;
begin
  if (c >= 0) and (c <= 255) then
    Result := __xmlIsPubidChar_tab^[c]
  else
    Result := false;
end;

function xmlIsPubidCharQ(c: cint): cbool;
begin
  if c < $100 then
    Result := xmlIsPubidChar_ch(c)
  else
    Result := false;
end;


(*
 * macros from HTMLparser.inc
 *)

function htmlDefaultSubelement(elt: htmlElemDescPtr): pchar;
begin
  Result := elt^.defaultsubelt;
end;

function htmlElementAllowedHereDesc(parent: htmlElemDescPtr; elt: htmlElemDescPtr): cint;
begin
  Result := htmlElementAllowedHere(parent, xmlCharPtr(elt^.name));
end;

function htmlRequiredAttrs(elt: htmlElemDescPtr): ppchar;
begin
  Result := elt^.attrs_req;
end;


(*
 * macros from tree.inc
 *)

function XML_GET_CONTENT(n: pointer): xmlCharPtr;
begin
  if xmlNodePtr(n)^._type = XML_ELEMENT_NODE then
    Result := nil
  else
    Result := xmlNodePtr(n)^.content;
end;


(*
 * macros from xpath.inc
 *)

function xmlXPathNodeSetGetLength(ns: xmlNodeSetPtr): cint;
begin
  if assigned(ns) then
    Result := ns^.nodeNr
  else
    Result := 0;
end;

function xmlXPathNodeSetItem(ns: xmlNodeSetPtr; index: cint): xmlNodePtr;
begin
  if assigned(ns) and (index >= 0) and (index < ns^.nodeNr) then
    Result := ns^.nodeTab[index]
  else
    Result := nil;
end;

function xmlXPathNodeSetIsEmpty(ns: xmlNodeSetPtr): boolean;
begin
  Result := not assigned(ns) or (ns^.nodeNr = 0) or (ns^.nodeTab = nil);
end;

{$IFDEF NO_EXTERNAL_VARS}
procedure LoadExternalVariables;
var
  libHandle: THandle;
begin
  libHandle := LoadLibrary(xml2lib);
  if libHandle <> 0 then
  begin
  { xmlregexp.inc }
   {__emptyExp := xmlExpNodePtrPtr(GetProcAddress(libHandle, 'emptyExp'));
    __forbiddenExp := xmlExpNodePtrPtr(GetProcAddress(libHandle, 'forbiddenExp'));}

  { paserInternals.inc }
    //__xmlParserMaxDepth := PCardinal(GetProcAddress(libHandle, 'xmlParserMaxDepth'));
   
  {  }
   {xmlStringComment := PChar(GetProcAddress(libHandle, 'xmlStringComment'));
    xmlStringText := PChar(GetProcAddress(libHandle, 'xmlStringText'));
    xmlStringTextNoenc := PChar(GetProcAddress(libHandle, 'xmlStringTextNoenc'));}

  { chvalid.inc }
    __xmlIsBaseCharGroup := xmlChRangeGroupPtr(GetProcAddress(libHandle, 'xmlIsBaseCharGroup'));
    __xmlIsCharGroup := xmlChRangeGroupPtr(GetProcAddress(libHandle, 'xmlIsCharGroup'));
    __xmlIsCombiningGroup := xmlChRangeGroupPtr(GetProcAddress(libHandle, 'xmlIsCombiningGroup'));
    __xmlIsDigitGroup := xmlChRangeGroupPtr(GetProcAddress(libHandle, 'xmlIsDigitGroup'));
    __xmlIsExtenderGroup := xmlChRangeGroupPtr(GetProcAddress(libHandle, 'xmlIsExtenderGroup'));
    __xmlIsIdeographicGroup := xmlChRangeGroupPtr(GetProcAddress(libHandle, 'xmlIsIdeographicGroup'));
    __xmlIsPubidChar_tab := GetProcAddress(libHandle, 'xmlIsPubidChar_tab');
    
  { globals.inc }
    varxmlMalloc := PxmlMallocFunc(GetProcAddress(libHandle, 'xmlMalloc'));
    varxmlMallocAtomic := PxmlMallocFunc(GetProcAddress(libHandle, 'xmlMallocAtomic'));
    varxmlRealloc := PxmlReallocFunc(GetProcAddress(libHandle, 'xmlRealloc'));
    varxmlFree := PxmlFreeFunc(GetProcAddress(libHandle, 'xmlFree'));
    varxmlMemStrdup := PxmlStrdupFunc(GetProcAddress(libHandle, 'xmlMemStrdup'));
    
  { xpath.inc }
   {__xmlXPathNAN := PDouble(GetProcAddress(libHandle, 'xmlXPathNAN'));
    __xmlXPathNINF := PDouble(GetProcAddress(libHandle, 'xmlXPathNINF'));
    __xmlXPathPINF := PDouble(GetProcAddress(libHandle, 'xmlXPathPINF'));}
    
    FreeLibrary(libHandle);
  end;
end;
{$ENDIF}

initialization
{$IFDEF NO_EXTERNAL_VARS}
  LoadExternalVariables;
{$ENDIF}

(*
 * overloading the memory functions
 *)
  xmlMemSetup(@fpcxmlFree, @fpcxmlMalloc, @fpcxmlRealloc, @fpcxmlStrdup);

(*
 * this initialize the library and check potential ABI mismatches
 * between the version it was compiled for and the actual shared
 * library used.
 *)
  LIBXML_TEST_VERSION;

(*
 * overloading the error functions
 *)
  //xmlSetGenericErrorFunc(nil, @fpcxmlGenericErrorHandler);
  //xmlSetStructuredErrorFunc(nil, @fpcxmlStructuredErrorHandler);

finalization
(*
 * Cleanup function for the XML library.
 *)
  xmlCleanupParser();

(*
 * this is to debug memory for regression tests
 *)
  //xmlMemoryDump();

end.
