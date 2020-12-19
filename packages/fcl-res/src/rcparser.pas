
(* Yacc parser template (TP Yacc V3.0), V1.2 6-17-91 AG *)

(* global definitions: *)

(*
Vorspann
 ****************************************************************************)

unit rcparser;

{$I rcparserfn.inc}

const _ILLEGAL = 257;
const _NUMBER = 258;
const _QUOTEDSTR = 259;
const _QUOTEDSTRL = 260;
const _STR_StringFileInfo = 261;
const _STR_VarFileInfo = 262;
const _STR_Translation = 263;
const _BEGIN = 264;
const _END = 265;
const _ID = 266;
const _NSWPSTR = 267;
const _LANGUAGE = 268;
const _CHARACTERISTICS = 269;
const _VERSION = 270;
const _MOVEABLE = 271;
const _FIXED = 272;
const _PURE = 273;
const _IMPURE = 274;
const _PRELOAD = 275;
const _LOADONCALL = 276;
const _DISCARDABLE = 277;
const _BITMAP = 278;
const _CURSOR = 279;
const _ICON = 280;
const _STRINGTABLE = 281;
const _VERSIONINFO = 282;
const _ANICURSOR = 283;
const _ANIICON = 284;
const _DLGINCLUDE = 285;
const _DLGINIT = 286;
const _HTML = 287;
const _MANIFEST = 288;
const _MESSAGETABLE = 289;
const _PLUGPLAY = 290;
const _RCDATA = 291;
const _VXD = 292;
const _FILEVERSION = 293;
const _PRODUCTVERSION = 294;
const _FILEFLAGSMASK = 295;
const _FILEFLAGS = 296;
const _FILEOS = 297;
const _FILETYPE = 298;
const _FILESUBTYPE = 299;
const _BLOCK = 300;
const _VALUE = 301;
const _ACCELERATORS = 302;
const _DIALOG = 303;
const _DIALOGEX = 304;
const _MENU = 305;
const _MENUEX = 306;
const _NUMNEG = 307;

type YYSType = record case Integer of
                 1 : ( yyTFileStream : TFileStream );
                 2 : ( yyTMemoryStream : TMemoryStream );
                 3 : ( yyTResourceDesc : TResourceDesc );
                 4 : ( yyrcnumtype : rcnumtype );
                 5 : ( yyrcstrtype : rcstrtype );
               end(*YYSType*);

var yylval : YYSType;

function yylex : Integer; forward;

function yyparse : Integer;

var yystate, yysp, yyn : Integer;
    yys : array [1..yymaxdepth] of Integer;
    yyv : array [1..yymaxdepth] of YYSType;
    yyval : YYSType;

procedure yyaction ( yyruleno : Integer );
  (* local definitions: *)
begin
  (* actions: *)
  case yyruleno of
   1 : begin
       end;
   2 : begin
         yyval := yyv[yysp-1];
       end;
   3 : begin
         yyval := yyv[yysp-0];
       end;
   4 : begin
         yyval := yyv[yysp-0];
       end;
   5 : begin
         yyval := yyv[yysp-0];
       end;
   6 : begin
         yyval := yyv[yysp-0];
       end;
   7 : begin
         yyval := yyv[yysp-0];
       end;
   8 : begin
         yyval := yyv[yysp-0];
       end;
   9 : begin
         yyval := yyv[yysp-0];
       end;
  10 : begin
         yyval := yyv[yysp-0];
       end;
  11 : begin
         create_resource(yyv[yysp-1].yyTResourceDesc, RT_BITMAP); 
       end;
  12 : begin
         TBitmapResource(aktresource).SetCustomBitmapDataStream(yyv[yysp-0].yyTFileStream); 
       end;
  13 : begin
         create_resource(yyv[yysp-1].yyTResourceDesc, RT_CURSOR); 
       end;
  14 : begin
         TGroupCursorResource(aktresource).SetCustomItemDataStream(yyv[yysp-0].yyTFileStream); 
       end;
  15 : begin
         create_resource(yyv[yysp-1].yyTResourceDesc, RT_ICON); 
       end;
  16 : begin
         TGroupIconResource(aktresource).SetCustomItemDataStream(yyv[yysp-0].yyTFileStream); 
       end;
  17 : begin
         create_resource(yyv[yysp-1].yyTResourceDesc, RT_VERSION); 
       end;
  18 : begin
         yyval := yyv[yysp-6];
       end;
  19 : begin
       end;
  20 : begin
         TVersionResource(aktresource).FixedInfo.FileVersion:= make_version(yyv[yysp-6].yyrcnumtype.v, yyv[yysp-4].yyrcnumtype.v, yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcnumtype.v); 
       end;
  21 : begin
         TVersionResource(aktresource).FixedInfo.ProductVersion:= make_version(yyv[yysp-6].yyrcnumtype.v, yyv[yysp-4].yyrcnumtype.v, yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcnumtype.v); 
       end;
  22 : begin
         TVersionResource(aktresource).FixedInfo.FileFlagsMask:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  23 : begin
         TVersionResource(aktresource).FixedInfo.FileFlags:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  24 : begin
         TVersionResource(aktresource).FixedInfo.FileOS:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  25 : begin
         TVersionResource(aktresource).FixedInfo.FileType:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  26 : begin
         TVersionResource(aktresource).FixedInfo.FileSubType:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  27 : begin
       end;
  28 : begin
         yyval := yyv[yysp-5];
       end;
  29 : begin
         yyval := yyv[yysp-5];
       end;
  30 : begin
       end;
  31 : begin
         version_string_tab_begin(yyv[yysp-1].yyrcstrtype.v^); 
       end;
  32 : begin
         yyval := yyv[yysp-6];
       end;
  33 : begin
       end;
  34 : begin
         version_string_tab_add(yyv[yysp-2].yyrcstrtype.v^, yyv[yysp-0].yyrcstrtype.v^); 
       end;
  35 : begin
         yyval := yyv[yysp-3];
       end;
  36 : begin
         version_var_translation_add(yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcnumtype.v); 
       end;
  37 : begin
         version_var_translation_add(yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcnumtype.v); 
       end;
  38 : begin
         create_resource(yyv[yysp-1].yyTResourceDesc, yyv[yysp-0].yyTResourceDesc); 
       end;
  39 : begin
         aktresource.SetCustomRawDataStream(yyv[yysp-0].yyTFileStream); 
       end;
  40 : begin
         create_resource(yyv[yysp-1].yyTResourceDesc, yyv[yysp-0].yyTResourceDesc); 
       end;
  41 : begin
         aktresource.SetCustomRawDataStream(yyv[yysp-1].yyTMemoryStream); 
       end;
  42 : begin
         stringtable_begin(); 
       end;
  43 : begin
         stringtable_end(); 
       end;
  44 : begin
       end;
  45 : begin
         yyval := yyv[yysp-1];
       end;
  46 : begin
         stringtable_add(yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcstrtype.v^); 
       end;
  47 : begin
         stringtable_add(yyv[yysp-1].yyrcnumtype.v, yyv[yysp-0].yyrcstrtype.v^); 
       end;
  48 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_ANICURSOR); 
       end;
  49 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_ANIICON); 
       end;
  50 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_DLGINCLUDE); 
       end;
  51 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_DLGINIT); 
       end;
  52 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(23); 
       end;
  53 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_MANIFEST); 
       end;
  54 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_MESSAGETABLE); 
       end;
  55 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_PLUGPLAY); 
       end;
  56 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_RCDATA); 
       end;
  57 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(RT_VXD); 
       end;
  58 : begin
         yyval := yyv[yysp-0];
       end;
  59 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(yyv[yysp-0].yyrcnumtype.v); 
       end;
  60 : begin
         yyval.yyTResourceDesc:= TResourceDesc.Create(yyv[yysp-0].yyrcstrtype.v^); 
       end;
  61 : begin
       end;
  62 : begin
         yyval := yyv[yysp-1];
       end;
  63 : begin
         change_lang_id(MakeLangID(yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcnumtype.v)); 
       end;
  64 : begin
         aktresource.Characteristics:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  65 : begin
         aktresource.Version:= yyv[yysp-0].yyrcnumtype.v; 
       end;
  66 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_MOVEABLE; 
       end;
  67 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags and not MF_MOVEABLE; 
       end;
  68 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_PURE; 
       end;
  69 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags and not MF_PURE; 
       end;
  70 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_PRELOAD; 
       end;
  71 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags and not MF_PRELOAD; 
       end;
  72 : begin
         aktresource.MemoryFlags:= aktresource.MemoryFlags or MF_DISCARDABLE; 
       end;
  73 : begin
         language:= MakeLangID(yyv[yysp-2].yyrcnumtype.v, yyv[yysp-0].yyrcnumtype.v); 
       end;
  74 : begin
         yyval := yyv[yysp-0];
       end;
  75 : begin
         yyval.yyrcnumtype:= str_to_num(yytext); 
       end;
  76 : begin
         yyval := yyv[yysp-0];
       end;
  77 : begin
         yyval.yyrcnumtype:= yyv[yysp-1].yyrcnumtype; 
       end;
  78 : begin
         yyval.yyrcnumtype.v:= not yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-0].yyrcnumtype.long; 
       end;
  79 : begin
         yyval.yyrcnumtype.v:= -yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-0].yyrcnumtype.long; 
       end;
  80 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v * yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  81 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v div Max(1, yyv[yysp-0].yyrcnumtype.v); yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  82 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v mod Max(1, yyv[yysp-0].yyrcnumtype.v); yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  83 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v + yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  84 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v - yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  85 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v and yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  86 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v xor yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  87 : begin
         yyval.yyrcnumtype.v:= yyv[yysp-2].yyrcnumtype.v or yyv[yysp-0].yyrcnumtype.v; yyval.yyrcnumtype.long:= yyv[yysp-2].yyrcnumtype.long or yyv[yysp-0].yyrcnumtype.long; 
       end;
  88 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
  89 : begin
         yyval := yyv[yysp-0];
       end;
  90 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
  91 : begin
         yyval.yyTFileStream:= TFileStream.Create(yyv[yysp-0].yyrcstrtype.v^, fmOpenRead or fmShareDenyWrite); 
       end;
  92 : begin
         yyval.yyTFileStream:= TFileStream.Create(yyv[yysp-0].yyrcstrtype.v^, fmOpenRead or fmShareDenyWrite); 
       end;
  93 : begin
         string_new_uni(yyval.yyrcstrtype, @strbuf[0], strbuflen, opt_code_page, false); 
       end;
  94 : begin
         string_new_uni(yyval.yyrcstrtype, @strbuf[0], strbuflen, CP_UTF16, false); 
       end;
  95 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
  96 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
  97 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
  98 : begin
         string_new_uni(yyval.yyrcstrtype, @strbuf[0], strbuflen, opt_code_page, true); 
       end;
  99 : begin
         string_new_uni(yyval.yyrcstrtype, @strbuf[0], strbuflen, CP_UTF16, true); 
       end;
 100 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
 101 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
 102 : begin
         string_new(yyval.yyrcstrtype, yytext, opt_code_page); 
       end;
 103 : begin
         yyval.yyTMemoryStream:= TMemoryStream.Create; 
       end;
 104 : begin
         yyval := yyv[yysp-1];
       end;
 105 : begin
         yyval.yyTMemoryStream:= yyv[yysp-1].yyTMemoryStream; 
       end;
 106 : begin
         yyval := yyv[yysp-3];
       end;
 107 : begin
         
         yyval.yyTMemoryStream:= yyv[yysp-0].yyTMemoryStream;
         
       end;
 108 : begin
         
         yyval.yyTMemoryStream:= yyv[yysp-1].yyTMemoryStream;
         raw_write_string(yyval.yyTMemoryStream, yyv[yysp-0].yyrcstrtype);
         
       end;
 109 : begin
         
         yyval.yyTMemoryStream:= yyv[yysp-1].yyTMemoryStream;
         raw_write_int(yyval.yyTMemoryStream, yyv[yysp-0].yyrcnumtype);
         
       end;
  end;
end(*yyaction*);

(* parse table: *)

type YYARec = record
                sym, act : Integer;
              end;
     YYRRec = record
                len, sym : Integer;
              end;

const

yynacts   = 654;
yyngotos  = 145;
yynstates = 187;
yynrules  = 109;

yya : array [1..yynacts] of YYARec = (
{ 0: }
{ 1: }
  ( sym: 0; act: 0 ),
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
  ( sym: 266; act: 26 ),
  ( sym: 268; act: 27 ),
  ( sym: 281; act: 28 ),
{ 2: }
{ 3: }
{ 4: }
{ 5: }
{ 6: }
{ 7: }
{ 8: }
{ 9: }
{ 10: }
{ 11: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
  ( sym: 266; act: 26 ),
  ( sym: 278; act: 31 ),
  ( sym: 279; act: 32 ),
  ( sym: 280; act: 33 ),
  ( sym: 282; act: 34 ),
  ( sym: 283; act: 35 ),
  ( sym: 284; act: 36 ),
  ( sym: 285; act: 37 ),
  ( sym: 286; act: 38 ),
  ( sym: 287; act: 39 ),
  ( sym: 288; act: 40 ),
  ( sym: 289; act: 41 ),
  ( sym: 290; act: 42 ),
  ( sym: 291; act: 43 ),
  ( sym: 292; act: 44 ),
{ 12: }
{ 13: }
{ 14: }
{ 15: }
  ( sym: 37; act: 45 ),
  ( sym: 38; act: 46 ),
  ( sym: 42; act: 47 ),
  ( sym: 43; act: 48 ),
  ( sym: 45; act: 49 ),
  ( sym: 47; act: 50 ),
  ( sym: 94; act: 51 ),
  ( sym: 124; act: 52 ),
  ( sym: 0; act: -74 ),
  ( sym: 40; act: -74 ),
  ( sym: 44; act: -74 ),
  ( sym: 126; act: -74 ),
  ( sym: 258; act: -74 ),
  ( sym: 259; act: -74 ),
  ( sym: 260; act: -74 ),
  ( sym: 261; act: -74 ),
  ( sym: 262; act: -74 ),
  ( sym: 263; act: -74 ),
  ( sym: 264; act: -74 ),
  ( sym: 266; act: -74 ),
  ( sym: 267; act: -74 ),
  ( sym: 268; act: -74 ),
  ( sym: 269; act: -74 ),
  ( sym: 270; act: -74 ),
  ( sym: 271; act: -74 ),
  ( sym: 272; act: -74 ),
  ( sym: 273; act: -74 ),
  ( sym: 274; act: -74 ),
  ( sym: 275; act: -74 ),
  ( sym: 276; act: -74 ),
  ( sym: 277; act: -74 ),
  ( sym: 278; act: -74 ),
  ( sym: 279; act: -74 ),
  ( sym: 280; act: -74 ),
  ( sym: 281; act: -74 ),
  ( sym: 282; act: -74 ),
  ( sym: 283; act: -74 ),
  ( sym: 284; act: -74 ),
  ( sym: 285; act: -74 ),
  ( sym: 286; act: -74 ),
  ( sym: 287; act: -74 ),
  ( sym: 288; act: -74 ),
  ( sym: 289; act: -74 ),
  ( sym: 290; act: -74 ),
  ( sym: 291; act: -74 ),
  ( sym: 292; act: -74 ),
  ( sym: 293; act: -74 ),
  ( sym: 294; act: -74 ),
  ( sym: 295; act: -74 ),
  ( sym: 296; act: -74 ),
  ( sym: 297; act: -74 ),
  ( sym: 298; act: -74 ),
  ( sym: 299; act: -74 ),
{ 16: }
{ 17: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 18: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 19: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 20: }
{ 21: }
{ 22: }
{ 23: }
{ 24: }
{ 25: }
{ 26: }
{ 27: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 28: }
{ 29: }
  ( sym: 259; act: -38 ),
  ( sym: 260; act: -38 ),
  ( sym: 261; act: -38 ),
  ( sym: 262; act: -38 ),
  ( sym: 263; act: -38 ),
  ( sym: 267; act: -38 ),
  ( sym: 268; act: -38 ),
  ( sym: 269; act: -38 ),
  ( sym: 270; act: -38 ),
  ( sym: 271; act: -38 ),
  ( sym: 272; act: -38 ),
  ( sym: 273; act: -38 ),
  ( sym: 274; act: -38 ),
  ( sym: 275; act: -38 ),
  ( sym: 276; act: -38 ),
  ( sym: 277; act: -38 ),
  ( sym: 264; act: -40 ),
{ 30: }
{ 31: }
{ 32: }
{ 33: }
{ 34: }
{ 35: }
{ 36: }
{ 37: }
{ 38: }
{ 39: }
{ 40: }
{ 41: }
{ 42: }
{ 43: }
{ 44: }
{ 45: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 46: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 47: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 48: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 49: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 50: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 51: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 52: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 53: }
  ( sym: 37; act: 45 ),
  ( sym: 38; act: 46 ),
  ( sym: 41; act: 72 ),
  ( sym: 42; act: 47 ),
  ( sym: 43; act: 48 ),
  ( sym: 45; act: 49 ),
  ( sym: 47; act: 50 ),
  ( sym: 94; act: 51 ),
  ( sym: 124; act: 52 ),
{ 54: }
{ 55: }
{ 56: }
  ( sym: 44; act: 73 ),
{ 57: }
{ 58: }
{ 59: }
{ 60: }
{ 61: }
{ 62: }
{ 63: }
{ 64: }
{ 65: }
  ( sym: 37; act: 45 ),
  ( sym: 42; act: 47 ),
  ( sym: 43; act: 48 ),
  ( sym: 45; act: 49 ),
  ( sym: 47; act: 50 ),
  ( sym: 0; act: -85 ),
  ( sym: 38; act: -85 ),
  ( sym: 40; act: -85 ),
  ( sym: 41; act: -85 ),
  ( sym: 44; act: -85 ),
  ( sym: 94; act: -85 ),
  ( sym: 124; act: -85 ),
  ( sym: 126; act: -85 ),
  ( sym: 258; act: -85 ),
  ( sym: 259; act: -85 ),
  ( sym: 260; act: -85 ),
  ( sym: 261; act: -85 ),
  ( sym: 262; act: -85 ),
  ( sym: 263; act: -85 ),
  ( sym: 264; act: -85 ),
  ( sym: 266; act: -85 ),
  ( sym: 267; act: -85 ),
  ( sym: 268; act: -85 ),
  ( sym: 269; act: -85 ),
  ( sym: 270; act: -85 ),
  ( sym: 271; act: -85 ),
  ( sym: 272; act: -85 ),
  ( sym: 273; act: -85 ),
  ( sym: 274; act: -85 ),
  ( sym: 275; act: -85 ),
  ( sym: 276; act: -85 ),
  ( sym: 277; act: -85 ),
  ( sym: 278; act: -85 ),
  ( sym: 279; act: -85 ),
  ( sym: 280; act: -85 ),
  ( sym: 281; act: -85 ),
  ( sym: 282; act: -85 ),
  ( sym: 283; act: -85 ),
  ( sym: 284; act: -85 ),
  ( sym: 285; act: -85 ),
  ( sym: 286; act: -85 ),
  ( sym: 287; act: -85 ),
  ( sym: 288; act: -85 ),
  ( sym: 289; act: -85 ),
  ( sym: 290; act: -85 ),
  ( sym: 291; act: -85 ),
  ( sym: 292; act: -85 ),
  ( sym: 293; act: -85 ),
  ( sym: 294; act: -85 ),
  ( sym: 295; act: -85 ),
  ( sym: 296; act: -85 ),
  ( sym: 297; act: -85 ),
  ( sym: 298; act: -85 ),
  ( sym: 299; act: -85 ),
{ 66: }
{ 67: }
  ( sym: 37; act: 45 ),
  ( sym: 42; act: 47 ),
  ( sym: 47; act: 50 ),
  ( sym: 0; act: -83 ),
  ( sym: 38; act: -83 ),
  ( sym: 40; act: -83 ),
  ( sym: 41; act: -83 ),
  ( sym: 43; act: -83 ),
  ( sym: 44; act: -83 ),
  ( sym: 45; act: -83 ),
  ( sym: 94; act: -83 ),
  ( sym: 124; act: -83 ),
  ( sym: 126; act: -83 ),
  ( sym: 258; act: -83 ),
  ( sym: 259; act: -83 ),
  ( sym: 260; act: -83 ),
  ( sym: 261; act: -83 ),
  ( sym: 262; act: -83 ),
  ( sym: 263; act: -83 ),
  ( sym: 264; act: -83 ),
  ( sym: 266; act: -83 ),
  ( sym: 267; act: -83 ),
  ( sym: 268; act: -83 ),
  ( sym: 269; act: -83 ),
  ( sym: 270; act: -83 ),
  ( sym: 271; act: -83 ),
  ( sym: 272; act: -83 ),
  ( sym: 273; act: -83 ),
  ( sym: 274; act: -83 ),
  ( sym: 275; act: -83 ),
  ( sym: 276; act: -83 ),
  ( sym: 277; act: -83 ),
  ( sym: 278; act: -83 ),
  ( sym: 279; act: -83 ),
  ( sym: 280; act: -83 ),
  ( sym: 281; act: -83 ),
  ( sym: 282; act: -83 ),
  ( sym: 283; act: -83 ),
  ( sym: 284; act: -83 ),
  ( sym: 285; act: -83 ),
  ( sym: 286; act: -83 ),
  ( sym: 287; act: -83 ),
  ( sym: 288; act: -83 ),
  ( sym: 289; act: -83 ),
  ( sym: 290; act: -83 ),
  ( sym: 291; act: -83 ),
  ( sym: 292; act: -83 ),
  ( sym: 293; act: -83 ),
  ( sym: 294; act: -83 ),
  ( sym: 295; act: -83 ),
  ( sym: 296; act: -83 ),
  ( sym: 297; act: -83 ),
  ( sym: 298; act: -83 ),
  ( sym: 299; act: -83 ),
{ 68: }
  ( sym: 37; act: 45 ),
  ( sym: 42; act: 47 ),
  ( sym: 47; act: 50 ),
  ( sym: 0; act: -84 ),
  ( sym: 38; act: -84 ),
  ( sym: 40; act: -84 ),
  ( sym: 41; act: -84 ),
  ( sym: 43; act: -84 ),
  ( sym: 44; act: -84 ),
  ( sym: 45; act: -84 ),
  ( sym: 94; act: -84 ),
  ( sym: 124; act: -84 ),
  ( sym: 126; act: -84 ),
  ( sym: 258; act: -84 ),
  ( sym: 259; act: -84 ),
  ( sym: 260; act: -84 ),
  ( sym: 261; act: -84 ),
  ( sym: 262; act: -84 ),
  ( sym: 263; act: -84 ),
  ( sym: 264; act: -84 ),
  ( sym: 266; act: -84 ),
  ( sym: 267; act: -84 ),
  ( sym: 268; act: -84 ),
  ( sym: 269; act: -84 ),
  ( sym: 270; act: -84 ),
  ( sym: 271; act: -84 ),
  ( sym: 272; act: -84 ),
  ( sym: 273; act: -84 ),
  ( sym: 274; act: -84 ),
  ( sym: 275; act: -84 ),
  ( sym: 276; act: -84 ),
  ( sym: 277; act: -84 ),
  ( sym: 278; act: -84 ),
  ( sym: 279; act: -84 ),
  ( sym: 280; act: -84 ),
  ( sym: 281; act: -84 ),
  ( sym: 282; act: -84 ),
  ( sym: 283; act: -84 ),
  ( sym: 284; act: -84 ),
  ( sym: 285; act: -84 ),
  ( sym: 286; act: -84 ),
  ( sym: 287; act: -84 ),
  ( sym: 288; act: -84 ),
  ( sym: 289; act: -84 ),
  ( sym: 290; act: -84 ),
  ( sym: 291; act: -84 ),
  ( sym: 292; act: -84 ),
  ( sym: 293; act: -84 ),
  ( sym: 294; act: -84 ),
  ( sym: 295; act: -84 ),
  ( sym: 296; act: -84 ),
  ( sym: 297; act: -84 ),
  ( sym: 298; act: -84 ),
  ( sym: 299; act: -84 ),
{ 69: }
{ 70: }
  ( sym: 37; act: 45 ),
  ( sym: 38; act: 46 ),
  ( sym: 42; act: 47 ),
  ( sym: 43; act: 48 ),
  ( sym: 45; act: 49 ),
  ( sym: 47; act: 50 ),
  ( sym: 0; act: -86 ),
  ( sym: 40; act: -86 ),
  ( sym: 41; act: -86 ),
  ( sym: 44; act: -86 ),
  ( sym: 94; act: -86 ),
  ( sym: 124; act: -86 ),
  ( sym: 126; act: -86 ),
  ( sym: 258; act: -86 ),
  ( sym: 259; act: -86 ),
  ( sym: 260; act: -86 ),
  ( sym: 261; act: -86 ),
  ( sym: 262; act: -86 ),
  ( sym: 263; act: -86 ),
  ( sym: 264; act: -86 ),
  ( sym: 266; act: -86 ),
  ( sym: 267; act: -86 ),
  ( sym: 268; act: -86 ),
  ( sym: 269; act: -86 ),
  ( sym: 270; act: -86 ),
  ( sym: 271; act: -86 ),
  ( sym: 272; act: -86 ),
  ( sym: 273; act: -86 ),
  ( sym: 274; act: -86 ),
  ( sym: 275; act: -86 ),
  ( sym: 276; act: -86 ),
  ( sym: 277; act: -86 ),
  ( sym: 278; act: -86 ),
  ( sym: 279; act: -86 ),
  ( sym: 280; act: -86 ),
  ( sym: 281; act: -86 ),
  ( sym: 282; act: -86 ),
  ( sym: 283; act: -86 ),
  ( sym: 284; act: -86 ),
  ( sym: 285; act: -86 ),
  ( sym: 286; act: -86 ),
  ( sym: 287; act: -86 ),
  ( sym: 288; act: -86 ),
  ( sym: 289; act: -86 ),
  ( sym: 290; act: -86 ),
  ( sym: 291; act: -86 ),
  ( sym: 292; act: -86 ),
  ( sym: 293; act: -86 ),
  ( sym: 294; act: -86 ),
  ( sym: 295; act: -86 ),
  ( sym: 296; act: -86 ),
  ( sym: 297; act: -86 ),
  ( sym: 298; act: -86 ),
  ( sym: 299; act: -86 ),
{ 71: }
  ( sym: 37; act: 45 ),
  ( sym: 38; act: 46 ),
  ( sym: 42; act: 47 ),
  ( sym: 43; act: 48 ),
  ( sym: 45; act: 49 ),
  ( sym: 47; act: 50 ),
  ( sym: 94; act: 51 ),
  ( sym: 0; act: -87 ),
  ( sym: 40; act: -87 ),
  ( sym: 41; act: -87 ),
  ( sym: 44; act: -87 ),
  ( sym: 124; act: -87 ),
  ( sym: 126; act: -87 ),
  ( sym: 258; act: -87 ),
  ( sym: 259; act: -87 ),
  ( sym: 260; act: -87 ),
  ( sym: 261; act: -87 ),
  ( sym: 262; act: -87 ),
  ( sym: 263; act: -87 ),
  ( sym: 264; act: -87 ),
  ( sym: 266; act: -87 ),
  ( sym: 267; act: -87 ),
  ( sym: 268; act: -87 ),
  ( sym: 269; act: -87 ),
  ( sym: 270; act: -87 ),
  ( sym: 271; act: -87 ),
  ( sym: 272; act: -87 ),
  ( sym: 273; act: -87 ),
  ( sym: 274; act: -87 ),
  ( sym: 275; act: -87 ),
  ( sym: 276; act: -87 ),
  ( sym: 277; act: -87 ),
  ( sym: 278; act: -87 ),
  ( sym: 279; act: -87 ),
  ( sym: 280; act: -87 ),
  ( sym: 281; act: -87 ),
  ( sym: 282; act: -87 ),
  ( sym: 283; act: -87 ),
  ( sym: 284; act: -87 ),
  ( sym: 285; act: -87 ),
  ( sym: 286; act: -87 ),
  ( sym: 287; act: -87 ),
  ( sym: 288; act: -87 ),
  ( sym: 289; act: -87 ),
  ( sym: 290; act: -87 ),
  ( sym: 291; act: -87 ),
  ( sym: 292; act: -87 ),
  ( sym: 293; act: -87 ),
  ( sym: 294; act: -87 ),
  ( sym: 295; act: -87 ),
  ( sym: 296; act: -87 ),
  ( sym: 297; act: -87 ),
  ( sym: 298; act: -87 ),
  ( sym: 299; act: -87 ),
{ 72: }
{ 73: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 74: }
  ( sym: 264; act: 83 ),
  ( sym: 268; act: 84 ),
  ( sym: 269; act: 85 ),
  ( sym: 270; act: 86 ),
  ( sym: 271; act: 87 ),
  ( sym: 272; act: 88 ),
  ( sym: 273; act: 89 ),
  ( sym: 274; act: 90 ),
  ( sym: 275; act: 91 ),
  ( sym: 276; act: 92 ),
  ( sym: 277; act: 93 ),
{ 75: }
  ( sym: 264; act: 94 ),
  ( sym: 268; act: 84 ),
  ( sym: 269; act: 85 ),
  ( sym: 270; act: 86 ),
  ( sym: 271; act: 87 ),
  ( sym: 272; act: 88 ),
  ( sym: 273; act: 89 ),
  ( sym: 274; act: 90 ),
  ( sym: 275; act: 91 ),
  ( sym: 276; act: 92 ),
  ( sym: 277; act: 93 ),
{ 76: }
  ( sym: 259; act: 98 ),
  ( sym: 260; act: 99 ),
  ( sym: 261; act: 100 ),
  ( sym: 262; act: 101 ),
  ( sym: 263; act: 102 ),
  ( sym: 267; act: 103 ),
  ( sym: 268; act: 84 ),
  ( sym: 269; act: 85 ),
  ( sym: 270; act: 86 ),
  ( sym: 271; act: 87 ),
  ( sym: 272; act: 88 ),
  ( sym: 273; act: 89 ),
  ( sym: 274; act: 90 ),
  ( sym: 275; act: 91 ),
  ( sym: 276; act: 92 ),
  ( sym: 277; act: 93 ),
{ 77: }
  ( sym: 259; act: 98 ),
  ( sym: 260; act: 99 ),
  ( sym: 261; act: 100 ),
  ( sym: 262; act: 101 ),
  ( sym: 263; act: 102 ),
  ( sym: 267; act: 103 ),
  ( sym: 268; act: 84 ),
  ( sym: 269; act: 85 ),
  ( sym: 270; act: 86 ),
  ( sym: 271; act: 87 ),
  ( sym: 272; act: 88 ),
  ( sym: 273; act: 89 ),
  ( sym: 274; act: 90 ),
  ( sym: 275; act: 91 ),
  ( sym: 276; act: 92 ),
  ( sym: 277; act: 93 ),
{ 78: }
  ( sym: 259; act: 98 ),
  ( sym: 260; act: 99 ),
  ( sym: 261; act: 100 ),
  ( sym: 262; act: 101 ),
  ( sym: 263; act: 102 ),
  ( sym: 267; act: 103 ),
  ( sym: 268; act: 84 ),
  ( sym: 269; act: 85 ),
  ( sym: 270; act: 86 ),
  ( sym: 271; act: 87 ),
  ( sym: 272; act: 88 ),
  ( sym: 273; act: 89 ),
  ( sym: 274; act: 90 ),
  ( sym: 275; act: 91 ),
  ( sym: 276; act: 92 ),
  ( sym: 277; act: 93 ),
{ 79: }
  ( sym: 259; act: 98 ),
  ( sym: 260; act: 99 ),
  ( sym: 261; act: 100 ),
  ( sym: 262; act: 101 ),
  ( sym: 263; act: 102 ),
  ( sym: 267; act: 103 ),
  ( sym: 268; act: 84 ),
  ( sym: 269; act: 85 ),
  ( sym: 270; act: 86 ),
  ( sym: 271; act: 87 ),
  ( sym: 272; act: 88 ),
  ( sym: 273; act: 89 ),
  ( sym: 274; act: 90 ),
  ( sym: 275; act: 91 ),
  ( sym: 276; act: 92 ),
  ( sym: 277; act: 93 ),
{ 80: }
  ( sym: 264; act: 107 ),
  ( sym: 293; act: 108 ),
  ( sym: 294; act: 109 ),
  ( sym: 295; act: 110 ),
  ( sym: 296; act: 111 ),
  ( sym: 297; act: 112 ),
  ( sym: 298; act: 113 ),
  ( sym: 299; act: 114 ),
{ 81: }
{ 82: }
{ 83: }
{ 84: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 85: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 86: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 87: }
{ 88: }
{ 89: }
{ 90: }
{ 91: }
{ 92: }
{ 93: }
{ 94: }
{ 95: }
{ 96: }
{ 97: }
{ 98: }
{ 99: }
{ 100: }
{ 101: }
{ 102: }
{ 103: }
{ 104: }
{ 105: }
{ 106: }
{ 107: }
{ 108: }
  ( sym: 258; act: 20 ),
{ 109: }
  ( sym: 258; act: 20 ),
{ 110: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 111: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 112: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 113: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 114: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 115: }
  ( sym: 258; act: 20 ),
  ( sym: 265; act: 131 ),
{ 116: }
  ( sym: 44; act: 132 ),
{ 117: }
{ 118: }
{ 119: }
  ( sym: 258; act: 20 ),
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
  ( sym: 44; act: -107 ),
  ( sym: 265; act: -107 ),
{ 120: }
  ( sym: 44; act: 136 ),
  ( sym: 265; act: 137 ),
{ 121: }
  ( sym: 265; act: 138 ),
  ( sym: 300; act: 139 ),
{ 122: }
  ( sym: 44; act: 140 ),
{ 123: }
  ( sym: 44; act: 141 ),
{ 124: }
{ 125: }
{ 126: }
{ 127: }
{ 128: }
{ 129: }
{ 130: }
  ( sym: 44; act: 143 ),
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
{ 131: }
{ 132: }
  ( sym: 40; act: 17 ),
  ( sym: 45; act: 18 ),
  ( sym: 126; act: 19 ),
  ( sym: 258; act: 20 ),
{ 133: }
{ 134: }
{ 135: }
{ 136: }
{ 137: }
{ 138: }
{ 139: }
  ( sym: 261; act: 146 ),
  ( sym: 262; act: 147 ),
{ 140: }
  ( sym: 258; act: 20 ),
{ 141: }
  ( sym: 258; act: 20 ),
{ 142: }
{ 143: }
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
{ 144: }
{ 145: }
  ( sym: 258; act: 20 ),
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
  ( sym: 44; act: -107 ),
  ( sym: 265; act: -107 ),
{ 146: }
  ( sym: 264; act: 152 ),
{ 147: }
  ( sym: 264; act: 153 ),
{ 148: }
  ( sym: 44; act: 154 ),
{ 149: }
  ( sym: 44; act: 155 ),
{ 150: }
{ 151: }
{ 152: }
{ 153: }
  ( sym: 301; act: 158 ),
{ 154: }
  ( sym: 258; act: 20 ),
{ 155: }
  ( sym: 258; act: 20 ),
{ 156: }
  ( sym: 265; act: 161 ),
  ( sym: 300; act: 162 ),
{ 157: }
  ( sym: 265; act: 163 ),
{ 158: }
  ( sym: 263; act: 164 ),
{ 159: }
  ( sym: 44; act: 165 ),
{ 160: }
  ( sym: 44; act: 166 ),
{ 161: }
{ 162: }
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
{ 163: }
{ 164: }
  ( sym: 44; act: 168 ),
{ 165: }
  ( sym: 258; act: 20 ),
{ 166: }
  ( sym: 258; act: 20 ),
{ 167: }
  ( sym: 264; act: 171 ),
{ 168: }
  ( sym: 258; act: 20 ),
{ 169: }
{ 170: }
{ 171: }
{ 172: }
  ( sym: 44; act: 175 ),
  ( sym: 265; act: -35 ),
{ 173: }
  ( sym: 44; act: 176 ),
{ 174: }
{ 175: }
  ( sym: 258; act: 20 ),
{ 176: }
  ( sym: 258; act: 20 ),
{ 177: }
  ( sym: 265; act: 180 ),
  ( sym: 301; act: 181 ),
{ 178: }
  ( sym: 44; act: 182 ),
{ 179: }
{ 180: }
{ 181: }
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 ),
{ 182: }
  ( sym: 258; act: 20 ),
{ 183: }
  ( sym: 44; act: 185 ),
{ 184: }
{ 185: }
  ( sym: 259; act: 21 ),
  ( sym: 260; act: 22 ),
  ( sym: 261; act: 23 ),
  ( sym: 262; act: 24 ),
  ( sym: 263; act: 25 )
{ 186: }
);

yyg : array [1..yyngotos] of YYARec = (
{ 0: }
  ( sym: -14; act: 1 ),
{ 1: }
  ( sym: -23; act: 2 ),
  ( sym: -22; act: 3 ),
  ( sym: -21; act: 4 ),
  ( sym: -20; act: 5 ),
  ( sym: -19; act: 6 ),
  ( sym: -18; act: 7 ),
  ( sym: -17; act: 8 ),
  ( sym: -16; act: 9 ),
  ( sym: -15; act: 10 ),
  ( sym: -9; act: 11 ),
  ( sym: -6; act: 12 ),
  ( sym: -5; act: 13 ),
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 16 ),
{ 2: }
{ 3: }
{ 4: }
{ 5: }
{ 6: }
{ 7: }
{ 8: }
{ 9: }
{ 10: }
{ 11: }
  ( sym: -10; act: 29 ),
  ( sym: -9; act: 30 ),
  ( sym: -6; act: 12 ),
  ( sym: -5; act: 13 ),
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 16 ),
{ 12: }
{ 13: }
{ 14: }
{ 15: }
{ 16: }
{ 17: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 53 ),
{ 18: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 54 ),
{ 19: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 55 ),
{ 20: }
{ 21: }
{ 22: }
{ 23: }
{ 24: }
{ 25: }
{ 26: }
{ 27: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 56 ),
{ 28: }
  ( sym: -38; act: 57 ),
{ 29: }
  ( sym: -37; act: 58 ),
  ( sym: -36; act: 59 ),
{ 30: }
{ 31: }
  ( sym: -25; act: 60 ),
{ 32: }
  ( sym: -26; act: 61 ),
{ 33: }
  ( sym: -27; act: 62 ),
{ 34: }
  ( sym: -29; act: 63 ),
{ 35: }
{ 36: }
{ 37: }
{ 38: }
{ 39: }
{ 40: }
{ 41: }
{ 42: }
{ 43: }
{ 44: }
{ 45: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 64 ),
{ 46: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 65 ),
{ 47: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 66 ),
{ 48: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 67 ),
{ 49: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 68 ),
{ 50: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 69 ),
{ 51: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 70 ),
{ 52: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 71 ),
{ 53: }
{ 54: }
{ 55: }
{ 56: }
{ 57: }
  ( sym: -24; act: 74 ),
{ 58: }
  ( sym: -24; act: 75 ),
{ 59: }
  ( sym: -24; act: 76 ),
{ 60: }
  ( sym: -24; act: 77 ),
{ 61: }
  ( sym: -24; act: 78 ),
{ 62: }
  ( sym: -24; act: 79 ),
{ 63: }
  ( sym: -28; act: 80 ),
{ 64: }
{ 65: }
{ 66: }
{ 67: }
{ 68: }
{ 69: }
{ 70: }
{ 71: }
{ 72: }
{ 73: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 81 ),
{ 74: }
  ( sym: -41; act: 82 ),
{ 75: }
  ( sym: -41; act: 82 ),
{ 76: }
  ( sym: -41; act: 82 ),
  ( sym: -13; act: 95 ),
  ( sym: -8; act: 96 ),
  ( sym: -7; act: 97 ),
{ 77: }
  ( sym: -41; act: 82 ),
  ( sym: -13; act: 104 ),
  ( sym: -8; act: 96 ),
  ( sym: -7; act: 97 ),
{ 78: }
  ( sym: -41; act: 82 ),
  ( sym: -13; act: 105 ),
  ( sym: -8; act: 96 ),
  ( sym: -7; act: 97 ),
{ 79: }
  ( sym: -41; act: 82 ),
  ( sym: -13; act: 106 ),
  ( sym: -8; act: 96 ),
  ( sym: -7; act: 97 ),
{ 80: }
{ 81: }
{ 82: }
{ 83: }
  ( sym: -39; act: 115 ),
{ 84: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 116 ),
{ 85: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 117 ),
{ 86: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 118 ),
{ 87: }
{ 88: }
{ 89: }
{ 90: }
{ 91: }
{ 92: }
{ 93: }
{ 94: }
  ( sym: -42; act: 119 ),
  ( sym: -11; act: 120 ),
{ 95: }
{ 96: }
{ 97: }
{ 98: }
{ 99: }
{ 100: }
{ 101: }
{ 102: }
{ 103: }
{ 104: }
{ 105: }
{ 106: }
{ 107: }
  ( sym: -30; act: 121 ),
{ 108: }
  ( sym: -4; act: 122 ),
{ 109: }
  ( sym: -4; act: 123 ),
{ 110: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 124 ),
{ 111: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 125 ),
{ 112: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 126 ),
{ 113: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 127 ),
{ 114: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 128 ),
{ 115: }
  ( sym: -40; act: 129 ),
  ( sym: -4; act: 130 ),
{ 116: }
{ 117: }
{ 118: }
{ 119: }
  ( sym: -12; act: 133 ),
  ( sym: -6; act: 134 ),
  ( sym: -4; act: 135 ),
{ 120: }
{ 121: }
{ 122: }
{ 123: }
{ 124: }
{ 125: }
{ 126: }
{ 127: }
{ 128: }
{ 129: }
{ 130: }
  ( sym: -6; act: 142 ),
{ 131: }
{ 132: }
  ( sym: -4; act: 14 ),
  ( sym: -3; act: 15 ),
  ( sym: -2; act: 144 ),
{ 133: }
{ 134: }
{ 135: }
{ 136: }
  ( sym: -43; act: 145 ),
{ 137: }
{ 138: }
{ 139: }
{ 140: }
  ( sym: -4; act: 148 ),
{ 141: }
  ( sym: -4; act: 149 ),
{ 142: }
{ 143: }
  ( sym: -6; act: 150 ),
{ 144: }
{ 145: }
  ( sym: -12; act: 151 ),
  ( sym: -6; act: 134 ),
  ( sym: -4; act: 135 ),
{ 146: }
{ 147: }
{ 148: }
{ 149: }
{ 150: }
{ 151: }
{ 152: }
  ( sym: -31; act: 156 ),
{ 153: }
  ( sym: -32; act: 157 ),
{ 154: }
  ( sym: -4; act: 159 ),
{ 155: }
  ( sym: -4; act: 160 ),
{ 156: }
{ 157: }
{ 158: }
{ 159: }
{ 160: }
{ 161: }
{ 162: }
  ( sym: -6; act: 167 ),
{ 163: }
{ 164: }
{ 165: }
  ( sym: -4; act: 169 ),
{ 166: }
  ( sym: -4; act: 170 ),
{ 167: }
{ 168: }
  ( sym: -35; act: 172 ),
  ( sym: -4; act: 173 ),
{ 169: }
{ 170: }
{ 171: }
  ( sym: -34; act: 174 ),
{ 172: }
{ 173: }
{ 174: }
  ( sym: -33; act: 177 ),
{ 175: }
  ( sym: -4; act: 178 ),
{ 176: }
  ( sym: -4; act: 179 ),
{ 177: }
{ 178: }
{ 179: }
{ 180: }
{ 181: }
  ( sym: -6; act: 183 ),
{ 182: }
  ( sym: -4; act: 184 ),
{ 183: }
{ 184: }
{ 185: }
  ( sym: -6; act: 186 )
{ 186: }
);

yyd : array [0..yynstates-1] of Integer = (
{ 0: } -1,
{ 1: } 0,
{ 2: } -10,
{ 3: } -9,
{ 4: } -8,
{ 5: } -7,
{ 6: } -6,
{ 7: } -5,
{ 8: } -4,
{ 9: } -3,
{ 10: } -2,
{ 11: } 0,
{ 12: } -89,
{ 13: } -60,
{ 14: } -76,
{ 15: } 0,
{ 16: } -59,
{ 17: } 0,
{ 18: } 0,
{ 19: } 0,
{ 20: } -75,
{ 21: } -98,
{ 22: } -99,
{ 23: } -100,
{ 24: } -101,
{ 25: } -102,
{ 26: } -88,
{ 27: } 0,
{ 28: } -42,
{ 29: } 0,
{ 30: } -58,
{ 31: } -11,
{ 32: } -13,
{ 33: } -15,
{ 34: } -17,
{ 35: } -48,
{ 36: } -49,
{ 37: } -50,
{ 38: } -51,
{ 39: } -52,
{ 40: } -53,
{ 41: } -54,
{ 42: } -55,
{ 43: } -56,
{ 44: } -57,
{ 45: } 0,
{ 46: } 0,
{ 47: } 0,
{ 48: } 0,
{ 49: } 0,
{ 50: } 0,
{ 51: } 0,
{ 52: } 0,
{ 53: } 0,
{ 54: } -79,
{ 55: } -78,
{ 56: } 0,
{ 57: } -61,
{ 58: } -61,
{ 59: } -61,
{ 60: } -61,
{ 61: } -61,
{ 62: } -61,
{ 63: } -19,
{ 64: } -82,
{ 65: } 0,
{ 66: } -80,
{ 67: } 0,
{ 68: } 0,
{ 69: } -81,
{ 70: } 0,
{ 71: } 0,
{ 72: } -77,
{ 73: } 0,
{ 74: } 0,
{ 75: } 0,
{ 76: } 0,
{ 77: } 0,
{ 78: } 0,
{ 79: } 0,
{ 80: } 0,
{ 81: } -73,
{ 82: } -62,
{ 83: } -44,
{ 84: } 0,
{ 85: } 0,
{ 86: } 0,
{ 87: } -66,
{ 88: } -67,
{ 89: } -68,
{ 90: } -69,
{ 91: } -70,
{ 92: } -71,
{ 93: } -72,
{ 94: } -103,
{ 95: } -39,
{ 96: } -91,
{ 97: } -92,
{ 98: } -93,
{ 99: } -94,
{ 100: } -95,
{ 101: } -96,
{ 102: } -97,
{ 103: } -90,
{ 104: } -12,
{ 105: } -14,
{ 106: } -16,
{ 107: } -27,
{ 108: } 0,
{ 109: } 0,
{ 110: } 0,
{ 111: } 0,
{ 112: } 0,
{ 113: } 0,
{ 114: } 0,
{ 115: } 0,
{ 116: } 0,
{ 117: } -64,
{ 118: } -65,
{ 119: } 0,
{ 120: } 0,
{ 121: } 0,
{ 122: } 0,
{ 123: } 0,
{ 124: } -22,
{ 125: } -23,
{ 126: } -24,
{ 127: } -25,
{ 128: } -26,
{ 129: } -45,
{ 130: } 0,
{ 131: } -43,
{ 132: } 0,
{ 133: } -104,
{ 134: } -108,
{ 135: } -109,
{ 136: } -105,
{ 137: } -41,
{ 138: } -18,
{ 139: } 0,
{ 140: } 0,
{ 141: } 0,
{ 142: } -47,
{ 143: } 0,
{ 144: } -63,
{ 145: } 0,
{ 146: } 0,
{ 147: } 0,
{ 148: } 0,
{ 149: } 0,
{ 150: } -46,
{ 151: } -106,
{ 152: } -30,
{ 153: } 0,
{ 154: } 0,
{ 155: } 0,
{ 156: } 0,
{ 157: } 0,
{ 158: } 0,
{ 159: } 0,
{ 160: } 0,
{ 161: } -28,
{ 162: } 0,
{ 163: } -29,
{ 164: } 0,
{ 165: } 0,
{ 166: } 0,
{ 167: } 0,
{ 168: } 0,
{ 169: } -20,
{ 170: } -21,
{ 171: } -31,
{ 172: } 0,
{ 173: } 0,
{ 174: } -33,
{ 175: } 0,
{ 176: } 0,
{ 177: } 0,
{ 178: } 0,
{ 179: } -36,
{ 180: } -32,
{ 181: } 0,
{ 182: } 0,
{ 183: } 0,
{ 184: } -37,
{ 185: } 0,
{ 186: } -34
);

yyal : array [0..yynstates-1] of Integer = (
{ 0: } 1,
{ 1: } 1,
{ 2: } 14,
{ 3: } 14,
{ 4: } 14,
{ 5: } 14,
{ 6: } 14,
{ 7: } 14,
{ 8: } 14,
{ 9: } 14,
{ 10: } 14,
{ 11: } 14,
{ 12: } 38,
{ 13: } 38,
{ 14: } 38,
{ 15: } 38,
{ 16: } 91,
{ 17: } 91,
{ 18: } 95,
{ 19: } 99,
{ 20: } 103,
{ 21: } 103,
{ 22: } 103,
{ 23: } 103,
{ 24: } 103,
{ 25: } 103,
{ 26: } 103,
{ 27: } 103,
{ 28: } 107,
{ 29: } 107,
{ 30: } 124,
{ 31: } 124,
{ 32: } 124,
{ 33: } 124,
{ 34: } 124,
{ 35: } 124,
{ 36: } 124,
{ 37: } 124,
{ 38: } 124,
{ 39: } 124,
{ 40: } 124,
{ 41: } 124,
{ 42: } 124,
{ 43: } 124,
{ 44: } 124,
{ 45: } 124,
{ 46: } 128,
{ 47: } 132,
{ 48: } 136,
{ 49: } 140,
{ 50: } 144,
{ 51: } 148,
{ 52: } 152,
{ 53: } 156,
{ 54: } 165,
{ 55: } 165,
{ 56: } 165,
{ 57: } 166,
{ 58: } 166,
{ 59: } 166,
{ 60: } 166,
{ 61: } 166,
{ 62: } 166,
{ 63: } 166,
{ 64: } 166,
{ 65: } 166,
{ 66: } 220,
{ 67: } 220,
{ 68: } 274,
{ 69: } 328,
{ 70: } 328,
{ 71: } 382,
{ 72: } 436,
{ 73: } 436,
{ 74: } 440,
{ 75: } 451,
{ 76: } 462,
{ 77: } 478,
{ 78: } 494,
{ 79: } 510,
{ 80: } 526,
{ 81: } 534,
{ 82: } 534,
{ 83: } 534,
{ 84: } 534,
{ 85: } 538,
{ 86: } 542,
{ 87: } 546,
{ 88: } 546,
{ 89: } 546,
{ 90: } 546,
{ 91: } 546,
{ 92: } 546,
{ 93: } 546,
{ 94: } 546,
{ 95: } 546,
{ 96: } 546,
{ 97: } 546,
{ 98: } 546,
{ 99: } 546,
{ 100: } 546,
{ 101: } 546,
{ 102: } 546,
{ 103: } 546,
{ 104: } 546,
{ 105: } 546,
{ 106: } 546,
{ 107: } 546,
{ 108: } 546,
{ 109: } 547,
{ 110: } 548,
{ 111: } 552,
{ 112: } 556,
{ 113: } 560,
{ 114: } 564,
{ 115: } 568,
{ 116: } 570,
{ 117: } 571,
{ 118: } 571,
{ 119: } 571,
{ 120: } 579,
{ 121: } 581,
{ 122: } 583,
{ 123: } 584,
{ 124: } 585,
{ 125: } 585,
{ 126: } 585,
{ 127: } 585,
{ 128: } 585,
{ 129: } 585,
{ 130: } 585,
{ 131: } 591,
{ 132: } 591,
{ 133: } 595,
{ 134: } 595,
{ 135: } 595,
{ 136: } 595,
{ 137: } 595,
{ 138: } 595,
{ 139: } 595,
{ 140: } 597,
{ 141: } 598,
{ 142: } 599,
{ 143: } 599,
{ 144: } 604,
{ 145: } 604,
{ 146: } 612,
{ 147: } 613,
{ 148: } 614,
{ 149: } 615,
{ 150: } 616,
{ 151: } 616,
{ 152: } 616,
{ 153: } 616,
{ 154: } 617,
{ 155: } 618,
{ 156: } 619,
{ 157: } 621,
{ 158: } 622,
{ 159: } 623,
{ 160: } 624,
{ 161: } 625,
{ 162: } 625,
{ 163: } 630,
{ 164: } 630,
{ 165: } 631,
{ 166: } 632,
{ 167: } 633,
{ 168: } 634,
{ 169: } 635,
{ 170: } 635,
{ 171: } 635,
{ 172: } 635,
{ 173: } 637,
{ 174: } 638,
{ 175: } 638,
{ 176: } 639,
{ 177: } 640,
{ 178: } 642,
{ 179: } 643,
{ 180: } 643,
{ 181: } 643,
{ 182: } 648,
{ 183: } 649,
{ 184: } 650,
{ 185: } 650,
{ 186: } 655
);

yyah : array [0..yynstates-1] of Integer = (
{ 0: } 0,
{ 1: } 13,
{ 2: } 13,
{ 3: } 13,
{ 4: } 13,
{ 5: } 13,
{ 6: } 13,
{ 7: } 13,
{ 8: } 13,
{ 9: } 13,
{ 10: } 13,
{ 11: } 37,
{ 12: } 37,
{ 13: } 37,
{ 14: } 37,
{ 15: } 90,
{ 16: } 90,
{ 17: } 94,
{ 18: } 98,
{ 19: } 102,
{ 20: } 102,
{ 21: } 102,
{ 22: } 102,
{ 23: } 102,
{ 24: } 102,
{ 25: } 102,
{ 26: } 102,
{ 27: } 106,
{ 28: } 106,
{ 29: } 123,
{ 30: } 123,
{ 31: } 123,
{ 32: } 123,
{ 33: } 123,
{ 34: } 123,
{ 35: } 123,
{ 36: } 123,
{ 37: } 123,
{ 38: } 123,
{ 39: } 123,
{ 40: } 123,
{ 41: } 123,
{ 42: } 123,
{ 43: } 123,
{ 44: } 123,
{ 45: } 127,
{ 46: } 131,
{ 47: } 135,
{ 48: } 139,
{ 49: } 143,
{ 50: } 147,
{ 51: } 151,
{ 52: } 155,
{ 53: } 164,
{ 54: } 164,
{ 55: } 164,
{ 56: } 165,
{ 57: } 165,
{ 58: } 165,
{ 59: } 165,
{ 60: } 165,
{ 61: } 165,
{ 62: } 165,
{ 63: } 165,
{ 64: } 165,
{ 65: } 219,
{ 66: } 219,
{ 67: } 273,
{ 68: } 327,
{ 69: } 327,
{ 70: } 381,
{ 71: } 435,
{ 72: } 435,
{ 73: } 439,
{ 74: } 450,
{ 75: } 461,
{ 76: } 477,
{ 77: } 493,
{ 78: } 509,
{ 79: } 525,
{ 80: } 533,
{ 81: } 533,
{ 82: } 533,
{ 83: } 533,
{ 84: } 537,
{ 85: } 541,
{ 86: } 545,
{ 87: } 545,
{ 88: } 545,
{ 89: } 545,
{ 90: } 545,
{ 91: } 545,
{ 92: } 545,
{ 93: } 545,
{ 94: } 545,
{ 95: } 545,
{ 96: } 545,
{ 97: } 545,
{ 98: } 545,
{ 99: } 545,
{ 100: } 545,
{ 101: } 545,
{ 102: } 545,
{ 103: } 545,
{ 104: } 545,
{ 105: } 545,
{ 106: } 545,
{ 107: } 545,
{ 108: } 546,
{ 109: } 547,
{ 110: } 551,
{ 111: } 555,
{ 112: } 559,
{ 113: } 563,
{ 114: } 567,
{ 115: } 569,
{ 116: } 570,
{ 117: } 570,
{ 118: } 570,
{ 119: } 578,
{ 120: } 580,
{ 121: } 582,
{ 122: } 583,
{ 123: } 584,
{ 124: } 584,
{ 125: } 584,
{ 126: } 584,
{ 127: } 584,
{ 128: } 584,
{ 129: } 584,
{ 130: } 590,
{ 131: } 590,
{ 132: } 594,
{ 133: } 594,
{ 134: } 594,
{ 135: } 594,
{ 136: } 594,
{ 137: } 594,
{ 138: } 594,
{ 139: } 596,
{ 140: } 597,
{ 141: } 598,
{ 142: } 598,
{ 143: } 603,
{ 144: } 603,
{ 145: } 611,
{ 146: } 612,
{ 147: } 613,
{ 148: } 614,
{ 149: } 615,
{ 150: } 615,
{ 151: } 615,
{ 152: } 615,
{ 153: } 616,
{ 154: } 617,
{ 155: } 618,
{ 156: } 620,
{ 157: } 621,
{ 158: } 622,
{ 159: } 623,
{ 160: } 624,
{ 161: } 624,
{ 162: } 629,
{ 163: } 629,
{ 164: } 630,
{ 165: } 631,
{ 166: } 632,
{ 167: } 633,
{ 168: } 634,
{ 169: } 634,
{ 170: } 634,
{ 171: } 634,
{ 172: } 636,
{ 173: } 637,
{ 174: } 637,
{ 175: } 638,
{ 176: } 639,
{ 177: } 641,
{ 178: } 642,
{ 179: } 642,
{ 180: } 642,
{ 181: } 647,
{ 182: } 648,
{ 183: } 649,
{ 184: } 649,
{ 185: } 654,
{ 186: } 654
);

yygl : array [0..yynstates-1] of Integer = (
{ 0: } 1,
{ 1: } 2,
{ 2: } 17,
{ 3: } 17,
{ 4: } 17,
{ 5: } 17,
{ 6: } 17,
{ 7: } 17,
{ 8: } 17,
{ 9: } 17,
{ 10: } 17,
{ 11: } 17,
{ 12: } 24,
{ 13: } 24,
{ 14: } 24,
{ 15: } 24,
{ 16: } 24,
{ 17: } 24,
{ 18: } 26,
{ 19: } 28,
{ 20: } 30,
{ 21: } 30,
{ 22: } 30,
{ 23: } 30,
{ 24: } 30,
{ 25: } 30,
{ 26: } 30,
{ 27: } 30,
{ 28: } 33,
{ 29: } 34,
{ 30: } 36,
{ 31: } 36,
{ 32: } 37,
{ 33: } 38,
{ 34: } 39,
{ 35: } 40,
{ 36: } 40,
{ 37: } 40,
{ 38: } 40,
{ 39: } 40,
{ 40: } 40,
{ 41: } 40,
{ 42: } 40,
{ 43: } 40,
{ 44: } 40,
{ 45: } 40,
{ 46: } 42,
{ 47: } 44,
{ 48: } 46,
{ 49: } 48,
{ 50: } 50,
{ 51: } 52,
{ 52: } 54,
{ 53: } 56,
{ 54: } 56,
{ 55: } 56,
{ 56: } 56,
{ 57: } 56,
{ 58: } 57,
{ 59: } 58,
{ 60: } 59,
{ 61: } 60,
{ 62: } 61,
{ 63: } 62,
{ 64: } 63,
{ 65: } 63,
{ 66: } 63,
{ 67: } 63,
{ 68: } 63,
{ 69: } 63,
{ 70: } 63,
{ 71: } 63,
{ 72: } 63,
{ 73: } 63,
{ 74: } 66,
{ 75: } 67,
{ 76: } 68,
{ 77: } 72,
{ 78: } 76,
{ 79: } 80,
{ 80: } 84,
{ 81: } 84,
{ 82: } 84,
{ 83: } 84,
{ 84: } 85,
{ 85: } 88,
{ 86: } 91,
{ 87: } 94,
{ 88: } 94,
{ 89: } 94,
{ 90: } 94,
{ 91: } 94,
{ 92: } 94,
{ 93: } 94,
{ 94: } 94,
{ 95: } 96,
{ 96: } 96,
{ 97: } 96,
{ 98: } 96,
{ 99: } 96,
{ 100: } 96,
{ 101: } 96,
{ 102: } 96,
{ 103: } 96,
{ 104: } 96,
{ 105: } 96,
{ 106: } 96,
{ 107: } 96,
{ 108: } 97,
{ 109: } 98,
{ 110: } 99,
{ 111: } 102,
{ 112: } 105,
{ 113: } 108,
{ 114: } 111,
{ 115: } 114,
{ 116: } 116,
{ 117: } 116,
{ 118: } 116,
{ 119: } 116,
{ 120: } 119,
{ 121: } 119,
{ 122: } 119,
{ 123: } 119,
{ 124: } 119,
{ 125: } 119,
{ 126: } 119,
{ 127: } 119,
{ 128: } 119,
{ 129: } 119,
{ 130: } 119,
{ 131: } 120,
{ 132: } 120,
{ 133: } 123,
{ 134: } 123,
{ 135: } 123,
{ 136: } 123,
{ 137: } 124,
{ 138: } 124,
{ 139: } 124,
{ 140: } 124,
{ 141: } 125,
{ 142: } 126,
{ 143: } 126,
{ 144: } 127,
{ 145: } 127,
{ 146: } 130,
{ 147: } 130,
{ 148: } 130,
{ 149: } 130,
{ 150: } 130,
{ 151: } 130,
{ 152: } 130,
{ 153: } 131,
{ 154: } 132,
{ 155: } 133,
{ 156: } 134,
{ 157: } 134,
{ 158: } 134,
{ 159: } 134,
{ 160: } 134,
{ 161: } 134,
{ 162: } 134,
{ 163: } 135,
{ 164: } 135,
{ 165: } 135,
{ 166: } 136,
{ 167: } 137,
{ 168: } 137,
{ 169: } 139,
{ 170: } 139,
{ 171: } 139,
{ 172: } 140,
{ 173: } 140,
{ 174: } 140,
{ 175: } 141,
{ 176: } 142,
{ 177: } 143,
{ 178: } 143,
{ 179: } 143,
{ 180: } 143,
{ 181: } 143,
{ 182: } 144,
{ 183: } 145,
{ 184: } 145,
{ 185: } 145,
{ 186: } 146
);

yygh : array [0..yynstates-1] of Integer = (
{ 0: } 1,
{ 1: } 16,
{ 2: } 16,
{ 3: } 16,
{ 4: } 16,
{ 5: } 16,
{ 6: } 16,
{ 7: } 16,
{ 8: } 16,
{ 9: } 16,
{ 10: } 16,
{ 11: } 23,
{ 12: } 23,
{ 13: } 23,
{ 14: } 23,
{ 15: } 23,
{ 16: } 23,
{ 17: } 25,
{ 18: } 27,
{ 19: } 29,
{ 20: } 29,
{ 21: } 29,
{ 22: } 29,
{ 23: } 29,
{ 24: } 29,
{ 25: } 29,
{ 26: } 29,
{ 27: } 32,
{ 28: } 33,
{ 29: } 35,
{ 30: } 35,
{ 31: } 36,
{ 32: } 37,
{ 33: } 38,
{ 34: } 39,
{ 35: } 39,
{ 36: } 39,
{ 37: } 39,
{ 38: } 39,
{ 39: } 39,
{ 40: } 39,
{ 41: } 39,
{ 42: } 39,
{ 43: } 39,
{ 44: } 39,
{ 45: } 41,
{ 46: } 43,
{ 47: } 45,
{ 48: } 47,
{ 49: } 49,
{ 50: } 51,
{ 51: } 53,
{ 52: } 55,
{ 53: } 55,
{ 54: } 55,
{ 55: } 55,
{ 56: } 55,
{ 57: } 56,
{ 58: } 57,
{ 59: } 58,
{ 60: } 59,
{ 61: } 60,
{ 62: } 61,
{ 63: } 62,
{ 64: } 62,
{ 65: } 62,
{ 66: } 62,
{ 67: } 62,
{ 68: } 62,
{ 69: } 62,
{ 70: } 62,
{ 71: } 62,
{ 72: } 62,
{ 73: } 65,
{ 74: } 66,
{ 75: } 67,
{ 76: } 71,
{ 77: } 75,
{ 78: } 79,
{ 79: } 83,
{ 80: } 83,
{ 81: } 83,
{ 82: } 83,
{ 83: } 84,
{ 84: } 87,
{ 85: } 90,
{ 86: } 93,
{ 87: } 93,
{ 88: } 93,
{ 89: } 93,
{ 90: } 93,
{ 91: } 93,
{ 92: } 93,
{ 93: } 93,
{ 94: } 95,
{ 95: } 95,
{ 96: } 95,
{ 97: } 95,
{ 98: } 95,
{ 99: } 95,
{ 100: } 95,
{ 101: } 95,
{ 102: } 95,
{ 103: } 95,
{ 104: } 95,
{ 105: } 95,
{ 106: } 95,
{ 107: } 96,
{ 108: } 97,
{ 109: } 98,
{ 110: } 101,
{ 111: } 104,
{ 112: } 107,
{ 113: } 110,
{ 114: } 113,
{ 115: } 115,
{ 116: } 115,
{ 117: } 115,
{ 118: } 115,
{ 119: } 118,
{ 120: } 118,
{ 121: } 118,
{ 122: } 118,
{ 123: } 118,
{ 124: } 118,
{ 125: } 118,
{ 126: } 118,
{ 127: } 118,
{ 128: } 118,
{ 129: } 118,
{ 130: } 119,
{ 131: } 119,
{ 132: } 122,
{ 133: } 122,
{ 134: } 122,
{ 135: } 122,
{ 136: } 123,
{ 137: } 123,
{ 138: } 123,
{ 139: } 123,
{ 140: } 124,
{ 141: } 125,
{ 142: } 125,
{ 143: } 126,
{ 144: } 126,
{ 145: } 129,
{ 146: } 129,
{ 147: } 129,
{ 148: } 129,
{ 149: } 129,
{ 150: } 129,
{ 151: } 129,
{ 152: } 130,
{ 153: } 131,
{ 154: } 132,
{ 155: } 133,
{ 156: } 133,
{ 157: } 133,
{ 158: } 133,
{ 159: } 133,
{ 160: } 133,
{ 161: } 133,
{ 162: } 134,
{ 163: } 134,
{ 164: } 134,
{ 165: } 135,
{ 166: } 136,
{ 167: } 136,
{ 168: } 138,
{ 169: } 138,
{ 170: } 138,
{ 171: } 139,
{ 172: } 139,
{ 173: } 139,
{ 174: } 140,
{ 175: } 141,
{ 176: } 142,
{ 177: } 142,
{ 178: } 142,
{ 179: } 142,
{ 180: } 142,
{ 181: } 143,
{ 182: } 144,
{ 183: } 144,
{ 184: } 144,
{ 185: } 145,
{ 186: } 145
);

yyr : array [1..yynrules] of YYRRec = (
{ 1: } ( len: 0; sym: -14 ),
{ 2: } ( len: 2; sym: -14 ),
{ 3: } ( len: 1; sym: -15 ),
{ 4: } ( len: 1; sym: -15 ),
{ 5: } ( len: 1; sym: -16 ),
{ 6: } ( len: 1; sym: -16 ),
{ 7: } ( len: 1; sym: -16 ),
{ 8: } ( len: 1; sym: -16 ),
{ 9: } ( len: 1; sym: -16 ),
{ 10: } ( len: 1; sym: -16 ),
{ 11: } ( len: 0; sym: -25 ),
{ 12: } ( len: 5; sym: -19 ),
{ 13: } ( len: 0; sym: -26 ),
{ 14: } ( len: 5; sym: -20 ),
{ 15: } ( len: 0; sym: -27 ),
{ 16: } ( len: 5; sym: -21 ),
{ 17: } ( len: 0; sym: -29 ),
{ 18: } ( len: 7; sym: -22 ),
{ 19: } ( len: 0; sym: -28 ),
{ 20: } ( len: 9; sym: -28 ),
{ 21: } ( len: 9; sym: -28 ),
{ 22: } ( len: 3; sym: -28 ),
{ 23: } ( len: 3; sym: -28 ),
{ 24: } ( len: 3; sym: -28 ),
{ 25: } ( len: 3; sym: -28 ),
{ 26: } ( len: 3; sym: -28 ),
{ 27: } ( len: 0; sym: -30 ),
{ 28: } ( len: 6; sym: -30 ),
{ 29: } ( len: 6; sym: -30 ),
{ 30: } ( len: 0; sym: -31 ),
{ 31: } ( len: 0; sym: -34 ),
{ 32: } ( len: 7; sym: -31 ),
{ 33: } ( len: 0; sym: -33 ),
{ 34: } ( len: 5; sym: -33 ),
{ 35: } ( len: 4; sym: -32 ),
{ 36: } ( len: 3; sym: -35 ),
{ 37: } ( len: 5; sym: -35 ),
{ 38: } ( len: 0; sym: -36 ),
{ 39: } ( len: 5; sym: -23 ),
{ 40: } ( len: 0; sym: -37 ),
{ 41: } ( len: 7; sym: -23 ),
{ 42: } ( len: 0; sym: -38 ),
{ 43: } ( len: 6; sym: -18 ),
{ 44: } ( len: 0; sym: -39 ),
{ 45: } ( len: 2; sym: -39 ),
{ 46: } ( len: 3; sym: -40 ),
{ 47: } ( len: 2; sym: -40 ),
{ 48: } ( len: 1; sym: -10 ),
{ 49: } ( len: 1; sym: -10 ),
{ 50: } ( len: 1; sym: -10 ),
{ 51: } ( len: 1; sym: -10 ),
{ 52: } ( len: 1; sym: -10 ),
{ 53: } ( len: 1; sym: -10 ),
{ 54: } ( len: 1; sym: -10 ),
{ 55: } ( len: 1; sym: -10 ),
{ 56: } ( len: 1; sym: -10 ),
{ 57: } ( len: 1; sym: -10 ),
{ 58: } ( len: 1; sym: -10 ),
{ 59: } ( len: 1; sym: -9 ),
{ 60: } ( len: 1; sym: -9 ),
{ 61: } ( len: 0; sym: -24 ),
{ 62: } ( len: 2; sym: -24 ),
{ 63: } ( len: 4; sym: -41 ),
{ 64: } ( len: 2; sym: -41 ),
{ 65: } ( len: 2; sym: -41 ),
{ 66: } ( len: 1; sym: -41 ),
{ 67: } ( len: 1; sym: -41 ),
{ 68: } ( len: 1; sym: -41 ),
{ 69: } ( len: 1; sym: -41 ),
{ 70: } ( len: 1; sym: -41 ),
{ 71: } ( len: 1; sym: -41 ),
{ 72: } ( len: 1; sym: -41 ),
{ 73: } ( len: 4; sym: -17 ),
{ 74: } ( len: 1; sym: -2 ),
{ 75: } ( len: 1; sym: -4 ),
{ 76: } ( len: 1; sym: -3 ),
{ 77: } ( len: 3; sym: -3 ),
{ 78: } ( len: 2; sym: -3 ),
{ 79: } ( len: 2; sym: -3 ),
{ 80: } ( len: 3; sym: -3 ),
{ 81: } ( len: 3; sym: -3 ),
{ 82: } ( len: 3; sym: -3 ),
{ 83: } ( len: 3; sym: -3 ),
{ 84: } ( len: 3; sym: -3 ),
{ 85: } ( len: 3; sym: -3 ),
{ 86: } ( len: 3; sym: -3 ),
{ 87: } ( len: 3; sym: -3 ),
{ 88: } ( len: 1; sym: -5 ),
{ 89: } ( len: 1; sym: -5 ),
{ 90: } ( len: 1; sym: -7 ),
{ 91: } ( len: 1; sym: -13 ),
{ 92: } ( len: 1; sym: -13 ),
{ 93: } ( len: 1; sym: -8 ),
{ 94: } ( len: 1; sym: -8 ),
{ 95: } ( len: 1; sym: -8 ),
{ 96: } ( len: 1; sym: -8 ),
{ 97: } ( len: 1; sym: -8 ),
{ 98: } ( len: 1; sym: -6 ),
{ 99: } ( len: 1; sym: -6 ),
{ 100: } ( len: 1; sym: -6 ),
{ 101: } ( len: 1; sym: -6 ),
{ 102: } ( len: 1; sym: -6 ),
{ 103: } ( len: 0; sym: -42 ),
{ 104: } ( len: 2; sym: -11 ),
{ 105: } ( len: 0; sym: -43 ),
{ 106: } ( len: 4; sym: -11 ),
{ 107: } ( len: 0; sym: -12 ),
{ 108: } ( len: 1; sym: -12 ),
{ 109: } ( len: 1; sym: -12 )
);


const _error = 256; (* error token *)

function yyact(state, sym : Integer; var act : Integer) : Boolean;
  (* search action table *)
  var k : Integer;
  begin
    k := yyal[state];
    while (k<=yyah[state]) and (yya[k].sym<>sym) do inc(k);
    if k>yyah[state] then
      yyact := false
    else
      begin
        act := yya[k].act;
        yyact := true;
      end;
  end(*yyact*);

function yygoto(state, sym : Integer; var nstate : Integer) : Boolean;
  (* search goto table *)
  var k : Integer;
  begin
    k := yygl[state];
    while (k<=yygh[state]) and (yyg[k].sym<>sym) do inc(k);
    if k>yygh[state] then
      yygoto := false
    else
      begin
        nstate := yyg[k].act;
        yygoto := true;
      end;
  end(*yygoto*);

label parse, next, error, errlab, shift, reduce, accept, abort;

begin(*yyparse*)

  (* initialize: *)

  yystate := 0; yychar := -1; yynerrs := 0; yyerrflag := 0; yysp := 0;

{$ifdef yydebug}
  yydebug := true;
{$else}
  yydebug := false;
{$endif}

parse:

  (* push state and value: *)

  inc(yysp);
  if yysp>yymaxdepth then
    begin
      yyerror('yyparse stack overflow');
      goto abort;
    end;
  yys[yysp] := yystate; yyv[yysp] := yyval;

next:

  if (yyd[yystate]=0) and (yychar=-1) then
    (* get next symbol *)
    begin
      yychar := yylex; if yychar<0 then yychar := 0;
    end;

  if yydebug then writeln('state ', yystate, ', char ', yychar);

  (* determine parse action: *)

  yyn := yyd[yystate];
  if yyn<>0 then goto reduce; (* simple state *)

  (* no default action; search parse table *)

  if not yyact(yystate, yychar, yyn) then goto error
  else if yyn>0 then                      goto shift
  else if yyn<0 then                      goto reduce
  else                                    goto accept;

error:

  (* error; start error recovery: *)

  if yyerrflag=0 then yyerror('syntax error');

errlab:

  if yyerrflag=0 then inc(yynerrs);     (* new error *)

  if yyerrflag<=2 then                  (* incomplete recovery; try again *)
    begin
      yyerrflag := 3;
      (* uncover a state with shift action on error token *)
      while (yysp>0) and not ( yyact(yys[yysp], _error, yyn) and
                               (yyn>0) ) do
        begin
          if yydebug then
            if yysp>1 then
              writeln('error recovery pops state ', yys[yysp], ', uncovers ',
                      yys[yysp-1])
            else
              writeln('error recovery fails ... abort');
          dec(yysp);
        end;
      if yysp=0 then goto abort; (* parser has fallen from stack; abort *)
      yystate := yyn;            (* simulate shift on error *)
      goto parse;
    end
  else                                  (* no shift yet; discard symbol *)
    begin
      if yydebug then writeln('error recovery discards char ', yychar);
      if yychar=0 then goto abort; (* end of input; abort *)
      yychar := -1; goto next;     (* clear lookahead char and try again *)
    end;

shift:

  (* go to new state, clear lookahead character: *)

  yystate := yyn; yychar := -1; yyval := yylval;
  if yyerrflag>0 then dec(yyerrflag);

  goto parse;

reduce:

  (* execute action, pop rule from stack, and go to next state: *)

  if yydebug then writeln('reduce ', -yyn);

  yyflag := yyfnone; yyaction(-yyn);
  dec(yysp, yyr[-yyn].len);
  if yygoto(yys[yysp], yyr[-yyn].sym, yyn) then yystate := yyn;

  (* handle action calls to yyaccept, yyabort and yyerror: *)

  case yyflag of
    yyfaccept : goto accept;
    yyfabort  : goto abort;
    yyferror  : goto errlab;
  end;

  goto parse;

accept:

  yyparse := 0; exit;

abort:

  yyparse := 1; exit;

end(*yyparse*);


{$I rclex.inc}
begin
  bufptr:= 0;
  lexlib.get_char:= @rc_get_char;
  lexlib.unget_char:= @rc_unget_char;
end.
