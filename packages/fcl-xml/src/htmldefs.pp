{
    $Id: htmldefs.pp,v 1.2 2006/01/03 23:33:23 lukvdl Exp $
    This file is part of the Free Component Library

    HTML definitions and utility functions
    Copyright (c) 2000-2002 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


unit HTMLDefs;

{$MODE objfpc}
{$H+}

interface

type

  THTMLCData = string;
  THTMLID = string;
  THTMLName = string;
  THTMLIDRef = string;
  THTMLIDRefs = string;
  THTMLNumber = longint;

  THTMLText = THTMLCData;
  THTMLCharsets = THTMLCData;
  THTMLContentTypes = THTMLCData;
  THTMLURI = string;
  THTMLCharacter = char;

  THTMLDir = (dirEmpty,dirLeftToRight,dirRightToLeft);
  THTMLalign = (alEmpty,alleft,alcenter,alright,aljustify,alchar);
  THTMLvalign = (vaEmpty,vatop,vamiddle,vabottom,vabaseline);
  THTMLframe = (frEmpty,frvoid,frabove,frbelow,frhsides,frvsides,frlefthandsise,frrighthandside,frbox,frborder);
  THTMLrules = (ruEmpty,runone,rugroups,rurows,rucols,ruall);
  THTMLvaluetype = (vtEmpty,vtdata,vtref,vtobject);
  THTMLshape = (shEmpty,shdefault,shrect,shcircle,shpoly);
  THTMLinputtype = (itEmpty,ittext,itpassword,itcheckbox,itradio,itsubmit,itreset,itfile,ithidden,itimage,itbutton);
  THTMLbuttontype = (btEmpty,btsubmit,btreset,btbutton);

  THTMLColor = (
      clHTMLBlack, clHTMLSilver, clHTMLGray,   clHTMLWhite, clHTMLMaroon,
    //  #000000      #C0C0C0       #808080       #FFFFFF      #800000
      clHTMLRed,   clHTMLPurple, clHTMLFuchsia,clHTMLGreen, clHTMLLime, clHTMLOlive,
    //  #FF0000      #800080       #FF00FF       #008000      #00FF00     #808000
      clHTMLYellow,clHTMLNavy,   clHTMLBlue,   clHTMLTeal,  clHTMLAqua
    //  #FFFF00      #000080       #0000FF       #008080      #00FFFF
      );

  THTMLAttributeTag = (
<<<<<<< HEAD
<<<<<<< HEAD
      atabbr, atalink, atacceptcharset, ataccept, ataccesskey, ataction, atalign, atalt, atarchive,
      ataxis, atbackground, atbgcolor, atborder, atcellpadding, atcellspacing, atchar, atcharoff, atcharset,
      atchecked, atcite, atclass, atclassid, atclear, atcode, atcodebase, atcodetype, atcolor, atcols,
      atcolspan, atcompact, atcontent, atcoords, atdata, atdatetime, atdeclare,atdefer,
      atdir, atdisabled, atenctype, atface, atfor, atframe, atframeborder, atheaders,
      atheight, athref, athreflang, athspace, athttpequiv, atid, atismap, atlabel, atlang, atlink,
      atlongdesc, atmarginheight, atmarginwidth, atmaxlength, atmedia, atmethod,
      atmultiple, atname, atnohref, atnoresize, atnoshade, atnowrap, atobject, atonblur, atonchange, atonclick,
      atondblclick, atonfocus, atonkeydown, atonkeypress, atonkeyup, atonload,
      atonmousedown, atonmousemove, atonmouseout, atonmouseover, atonmouseup,
      atonreset, atonselect, atonsubmit, atonunload, atprofile, atprompt, atreadonly,
      atrel, atrev, atrows, atrowspan, atrules, atscheme, atscope, atscrolling,
      atselected, atshape, atsize, atspan, atsrc, atstandby, atstart, atstyle, atsummary,
      attabindex, attarget, attext, attitle, attype, atusemap, atvalign, atvalue,
      atvaluetype, atversion, atvlink, atvspace, atwidth
=======
=======
>>>>>>> origin/fixes_2_2
      atabbr, atacceptcharset, ataccept, ataccesskey, ataction, atalign, atalt, atarchive,
      ataxis, atborder, atcellpadding, atcellspacing, atchar, atcharoff, atcharset,
      atchecked, atcite, atclass, atclassid, atcodebase, atcodetype, atcols,
      atcolspan, atcontent, atcoords, atdata, atdatetime, atdeclare,atdefer,
      atdir, atdisabled, atenctype, atfor, atframe, atframeborder, atheaders,
      atheight, athref, athreflang, athttpequiv, atid, atismap, atlabel, atlang,
      atlongdesc, atmarginheight, atmarginwidth, atmaxlength, atmedia, atmethod,
      atmultiple, atname, atnohref, atnoresize, atonblur, atonchange, atonclick,
      atondblclick, atonfocus, atonkeydown, atonkeypress, atonkeyup, atonload,
      atonmousedown, atonmousemove, atonmouseout, atonmouseover, atonmouseup,
      atonreset, atonselect, atonsubmit, atonunload, atprofile, atreadonly,
      atrel, atrev, atrows, atrowspan, atrules, atscheme, atscope, atscrolling,
      atselected, atshape, atsize, atspan, atsrc, atstandby, atstyle, atsummary,
      attabindex, attarget, attitle, attype, atusemap, atvalign, atvalue,
      atvaluetype, atwidth
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      );
  THTMLAttributeSet = set of THTMLAttributeTag;

  THTMLElementTag = (
      eta, etabbr, etacronym, etaddress, etapplet, etarea, etb, etbase,
      etbasefont, etbdo, etbig, etblockquote, etbody, etbr, etbutton,
      etcaption, etcenter, etcite, etcode, etcol, etcolgroup, etdd, etdel,
      etdfn, etdir, etdiv, etdl, etdt, etem, etfieldset, etfont, etform,
      etframe, etframeset, eth1, eth2, eth3, eth4, eth5, eth6, ethead, ethr,
      ethtml, eti, etiframe, etimg, etinput, etins, etisindex, etkbd, etlabel,
      etlegend, etli, etlink, etmap, etmenu, etmeta, etnoframes, etnoscript,
      etobject, etol, etoptgroup, etoption, etp, etparam, etpre, etq, ets,
      etsamp, etscript, etselect, etsmall, etspan, etstrike, etstrong,
      etstyle, etsub, etsup, ettable, ettbody, ettd, ettextarea, ettfoot,
      etth, etthead, ettitle, ettr, ettt, etu, etul, etvar,
      etText, etUnknown
      );
  THTMLElementTagSet = set of THTMLElementTag;

  THTMLElementFlag = (
    efSubelementContent,                // may have subelements
    efPCDATAContent,                    // may have PCDATA content
    efPreserveWhitespace,               // preserve all whitespace
    efDeprecated,                       // can be dropped in future versions
<<<<<<< HEAD
<<<<<<< HEAD
    efNoChecks,                         // Checks (attributes,subtags,...) can only be implemented in descendants
    efEndTagOptional
  );
=======
    efNoChecks                          // Checks (attributes,subtags,...) can only be implemented in descendants
    );
>>>>>>> graemeg/fixes_2_2
=======
    efNoChecks                          // Checks (attributes,subtags,...) can only be implemented in descendants
    );
>>>>>>> origin/fixes_2_2
  THTMLElementFlags = set of THTMLElementFlag;

  PHTMLElementProps = ^THTMLElementProps;
  THTMLElementProps = record
    Name: String;
    Flags: THTMLElementFlags;
    Attributes: THTMLAttributeSet;
  end;


const

  BooleanAttributes = [atchecked,atdeclare,atdefer,atdisabled,atnohref,atnoresize,
                    atmultiple,atreadonly,atselected];

<<<<<<< HEAD
<<<<<<< HEAD
  DeprecatedAttributes = [atalink, atbackground, atbgcolor, atclear, atcode, atcolor,
    atcompact, atface, athspace, atlink, atnoshade, atnowrap, atobject, atprompt,
    atstart, attext, atvlink, atversion, atvspace];

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  efSubcontent = [efSubelementContent, efPCDATAContent];

  atsi18n = [atlang, atdir];
  atscoreattrs = [atid,atclass,atstyle,attitle];
  atsevents = [atonclick,atondblclick,atonmousedown,atonmouseup,atonmouseover,
               atonmousemove,atonmouseout,atonkeypress,atonkeydown,atonkeyup];
  atsattrs = atsevents + atscoreattrs + atsi18n;
  atscellhalign = [atalign, atchar, atcharoff];

{  etsStructured := [];
  etsDivisions := [];
  etsLists := [];
  etsLinks := [];
  etsObjects := [etImg, etObject, etApplet, etMap, etArea];
  etsForms := [etForm];

  etsText = etsStructured + etsDivisions + etsLists + etsLinks + etsObjects +
          etsForms +
          etTable + etText + etScript + ;      }

  HTMLElementProps: array[THTMLElementTag] of THTMLElementProps = (
    (Name: 'a';         Flags: efSubcontent;
     Attributes: atsattrs+[atcharset,attype,atname,athref,athreflang,atrel,atrev,
                 ataccesskey,atshape,atcoords,attabindex,atonfocus,atonblur]),

    (Name: 'abbr';      Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'acronym';   Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'address';   Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'applet';    Flags: efSubcontent+[efDeprecated];
     Attributes: atscoreattrs+[atcodebase,atarchive,atalt,atname,atwidth,atheight]),

    (Name: 'area';      Flags: [];
     Attributes: atsattrs+[atshape,atcoords,athref,atnohref,atalt,attabindex,
     ataccesskey,atonfocus,atonblur]),

    (Name: 'b';         Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'base';      Flags: []; Attributes: [athref]),

    (Name: 'basefont';  Flags: [efDeprecated]; Attributes: [atid]),

    (Name: 'bdo';       Flags: efSubcontent; Attributes: atscoreattrs+[atlang,atdir]),

    (Name: 'big';       Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'blockquote';Flags: [efSubelementContent]; Attributes: atsattrs+[atcite]),

    (Name: 'body';      Flags: [efSubelementContent];
     Attributes: atsAttrs+[atonload, atonunload]),

    (Name: 'br';        Flags: []; Attributes: atscoreattrs),

    (Name: 'button';    Flags: efSubcontent;
     Attributes: atsattrs+[atname,atvalue,attype,atdisabled,attabindex,
                 ataccesskey,atonfocus,atonblur]),

    (Name: 'caption';   Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'center';    Flags: [efSubelementContent,efDeprecated]; Attributes: []),

    (Name: 'cite';      Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'code';      Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'col';       Flags: [];
     Attributes: atsattrs+atscellhalign+[atvalign,atspan,atwidth]),

<<<<<<< HEAD
<<<<<<< HEAD
    (Name: 'colgroup';  Flags: [efSubelementContent, efEndTagOptional];
     Attributes: atsattrs+atscellhalign+[atvalign,atspan,atwidth]),

    (Name: 'dd';        Flags: efSubcontent+[efEndTagOptional]; Attributes: atsattrs),
=======
=======
>>>>>>> origin/fixes_2_2
    (Name: 'colgroup';  Flags: [efSubelementContent];
     Attributes: atsattrs+atscellhalign+[atvalign,atspan,atwidth]),

    (Name: 'dd';        Flags: efSubcontent; Attributes: atsattrs),
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    (Name: 'del';       Flags: [efSubelementContent]; Attributes: atsattrs+[atcite,atdatetime]),

    (Name: 'dfn';       Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'dir';       Flags: [efSubelementContent,efDeprecated]; Attributes: atsattrs),

<<<<<<< HEAD
<<<<<<< HEAD
    (Name: 'div';       Flags: efSubContent; Attributes: atsattrs),

    (Name: 'dl';        Flags: [efSubelementContent]; Attributes: atsattrs),

    (Name: 'dt';        Flags: [efPCDataContent, efEndTagOptional]; Attributes: atsattrs),
=======
=======
>>>>>>> origin/fixes_2_2
    (Name: 'div';       Flags: [efSubelementContent]; Attributes: atsattrs),

    (Name: 'dl';        Flags: [efSubelementContent]; Attributes: atsattrs),

    (Name: 'dt';        Flags: [efPCDataContent]; Attributes: atsattrs),
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    (Name: 'em';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'fieldset';  Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'font';      Flags: efSubcontent+[efDeprecated]; Attributes: atscoreattrs+atsi18n),

    (Name: 'form';      Flags: [efSubelementContent];
     Attributes: atsattrs+[ataction,atmethod,atenctype,atonsubmit,atonreset,atacceptcharset]),

    (Name: 'frame';     Flags: [];
     Attributes: atscoreattrs+[atlongdesc,atname,atsrc,atframeborder,
                 atmarginwidth,atmarginheight,atnoresize,atscrolling]),

    (Name: 'frameset';  Flags: efSubcontent;
     Attributes: atsCoreattrs+[atrows,atcols,atonload,atonunload]),

    (Name: 'h1';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'h2';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'h3';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'h4';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'h5';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'h6';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'head';      Flags: [efSubelementContent]; Attributes: atsi18n+[atprofile]),

    (Name: 'hr';        Flags: []; Attributes: atscoreattrs+atsevents),

    (Name: 'html';      Flags: [efSubelementContent]; Attributes: atsi18n),

    (Name: 'i';         Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'iframe';    Flags: [efSubelementContent];
     Attributes: atscoreattrs+[atlongdesc,atname,atsrc,atframeborder,atmarginwidth,
                 atmarginheight,atscrolling,atalign,atheight,atwidth]),

    (Name: 'img';       Flags: [];
     Attributes: atsattrs+[atsrc,atalt,atlongdesc,atheight,atwidth,atusemap,atismap]),

    (Name: 'input';     Flags: [];
     Attributes: atsattrs+[attype,atname,atvalue,atchecked,atdisabled,
                 atreadonly,atsize,atmaxlength,atsrc,atalt,atusemap,attabindex,
                 ataccesskey,atonfocus,atonblur,atonselect,atonchange,ataccept]),

    (Name: 'ins';       Flags: [efSubelementContent]; Attributes: atsattrs+[atcite,atdatetime]),

    (Name: 'isindex';   Flags: [efDeprecated]; Attributes: atscoreattrs+atsi18n),

    (Name: 'kbd';       Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'label';     Flags: efSubcontent;
     Attributes: atsattrs+[atfor,ataccesskey,atonfocus,atonblur]),

    (Name: 'legend';    Flags: efSubcontent; Attributes: atsattrs+[ataccesskey]),

<<<<<<< HEAD
<<<<<<< HEAD
    (Name: 'li';        Flags: efSubcontent+[efEndTagOptional]; Attributes: atsattrs),
=======
    (Name: 'li';        Flags: efSubcontent; Attributes: atsattrs),
>>>>>>> graemeg/fixes_2_2
=======
    (Name: 'li';        Flags: efSubcontent; Attributes: atsattrs),
>>>>>>> origin/fixes_2_2

    (Name: 'link';      Flags: [];
     Attributes: atsattrs+[atcharset,athref,athreflang,attype,atrel,atrev,atmedia]),

    (Name: 'map';       Flags: [efSubelementContent]; Attributes: atsattrs+[atname]),

    (Name: 'menu';      Flags: [efSubelementContent,efDeprecated]; Attributes: atsattrs),

    (Name: 'meta';      Flags: []; Attributes: atsi18n+[athttpequiv,atname,atcontent,atscheme]),

    (Name: 'noframes';  Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'noscript';  Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'object';    Flags: efSubcontent;
     Attributes: atsattrs+[atdeclare,atclassid,atcodebase,atdata,attype,atcodetype,
                 atarchive,atstandby,atheight,atwidth,atusemap,atname,attabindex]),

    (Name: 'ol';        Flags: [efSubelementContent]; Attributes: atsattrs),

    (Name: 'optgroup';  Flags: efSubcontent; Attributes: atsattrs+[atdisabled,atlabel]),

<<<<<<< HEAD
<<<<<<< HEAD
    (Name: 'option';    Flags: efSubcontent+[efEndTagOptional];
     Attributes: atsattrs+[atselected,atdisabled,atlabel,atvalue]),

    (Name: 'p';         Flags: efSubcontent+[efEndTagOptional]; Attributes: atsattrs),
=======
=======
>>>>>>> origin/fixes_2_2
    (Name: 'option';    Flags: efSubcontent;
     Attributes: atsattrs+[atselected,atdisabled,atlabel,atvalue]),

    (Name: 'p';         Flags: efSubcontent; Attributes: atsattrs),
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    (Name: 'param';     Flags: []; Attributes: [atid,atname,atvalue,atvaluetype,attype]),

    (Name: 'pre';       Flags: efSubcontent + [efPreserveWhitespace]; Attributes: atsattrs),

    (Name: 'q';         Flags: efSubcontent; Attributes: atsattrs+[atcite]),

    (Name: 's';         Flags: efSubcontent+[efDeprecated]; Attributes: atsattrs),

    (Name: 'samp';      Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'script';    Flags: [efPCDATAContent]; Attributes: [atcharset,attype,atsrc,atdefer]),

    (Name: 'select';    Flags: [efSubelementContent];
     Attributes: atsattrs+[atname,atsize,atmultiple,atdisabled,attabindex,atonfocus,
                 atonblur,atonchange]),

    (Name: 'small';     Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'span';      Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'strike';    Flags: efSubcontent+[efDeprecated]; Attributes: atsattrs),

    (Name: 'strong';    Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'style';     Flags: [efPCDATAContent];
     Attributes: atsi18n+[attype,atmedia,attitle]),

    (Name: 'sub';       Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'sup';       Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'table';     Flags: [efSubelementContent];
     Attributes: atsattrs+[atsummary,atwidth,atborder,atframe,atrules,atcellspacing,atcellpadding]),

    (Name: 'tbody';     Flags: [efSubelementContent]; Attributes: atsattrs+atscellhalign+[atvalign]),

<<<<<<< HEAD
<<<<<<< HEAD
    (Name: 'td';        Flags: efSubcontent+[efEndTagOptional];
=======
    (Name: 'td';        Flags: efSubcontent;
>>>>>>> graemeg/fixes_2_2
=======
    (Name: 'td';        Flags: efSubcontent;
>>>>>>> origin/fixes_2_2
     Attributes: atsattrs+atscellhalign+[atvalign,atabbr,ataxis,atheaders,atscope,atrowspan,atcolspan]),

    (Name: 'textarea';  Flags: [efPCDATAContent];
     Attributes: atsattrs+[atname,atrows,atcols,atdisabled,atreadonly,attabindex,
                 ataccesskey,atonfocus,atonblur,atonselect,atonchange]),

<<<<<<< HEAD
<<<<<<< HEAD
    (Name: 'tfoot';     Flags: [efSubelementContent,efEndTagOptional]; Attributes: atsattrs+atscellhalign+[atvalign]),

    (Name: 'th';        Flags: efSubcontent+[efEndTagOptional];
     Attributes: atsattrs+atscellhalign+[atvalign,atabbr,ataxis,atheaders,atscope,atrowspan,atcolspan]),

    (Name: 'thead';     Flags: [efSubelementContent, efEndTagOptional]; Attributes: atsattrs+atscellhalign+[atvalign]),

    (Name: 'title';     Flags: efSubcontent; Attributes: atsi18n),

    (Name: 'tr';        Flags: [efSubelementContent, efEndTagOptional];
=======
=======
>>>>>>> origin/fixes_2_2
    (Name: 'tfoot';     Flags: [efSubelementContent]; Attributes: atsattrs+atscellhalign+[atvalign]),

    (Name: 'th';        Flags: efSubcontent;
     Attributes: atsattrs+atscellhalign+[atvalign,atabbr,ataxis,atheaders,atscope,atrowspan,atcolspan]),

    (Name: 'thead';     Flags: [efSubelementContent]; Attributes: atsattrs+atscellhalign+[atvalign]),

    (Name: 'title';     Flags: efSubcontent; Attributes: atsi18n),

    (Name: 'tr';        Flags: [efSubelementContent];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     Attributes: atsattrs+atscellhalign+[atvalign]),

    (Name: 'tt';        Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'u';         Flags: efSubcontent+[efDeprecated]; Attributes: atsattrs),

    (Name: 'ul';        Flags: [efSubelementContent]; Attributes: atsattrs),

    (Name: 'var';       Flags: efSubcontent; Attributes: atsattrs),

    (Name: 'text';      Flags: efSubcontent; Attributes: []),

    (Name: 'unknown';   Flags: efSubcontent+[efNoChecks]; Attributes: [])

    );

<<<<<<< HEAD
<<<<<<< HEAD
  HTMLAttributeTag : array [THTMLAttributeTag] of String = (
      'abbr', 'alink', 'accept-charset', 'accept', 'accesskey', 'action', 'align', 'alt', 'archive',
      'axis', 'background', 'bgcolor', 'border', 'cellpadding', 'cellspacing', 'char', 'charoff', 'charset',
      'checked', 'cite', 'class', 'classid', 'clear', 'code', 'codebase', 'codetype', 'color', 'cols',
      'colspan', 'compact', 'content', 'coords', 'data', 'datetime', 'declare', 'defer',
      'dir', 'disabled', 'enctype', 'face', 'for', 'frame', 'frameborder', 'headers',
      'height', 'href', 'hreflang', 'hspace', 'http-equiv', 'id', 'ismap', 'label', 'lang', 'link',
      'longdesc', 'marginheight', 'marginwidth', 'maxlength', 'media', 'method',
      'multiple', 'name', 'nohref', 'noresize', 'noshade', 'nowrap', 'object', 'onblur', 'onchange', 'onclick',
      'ondblclick', 'onfocus', 'onkeydown', 'onkeypress', 'onkeyup', 'onload',
      'onmousedown', 'onmousemove', 'onmouseout', 'onmouseover', 'onmouseup',
      'onreset', 'onselect', 'onsubmit', 'onunload', 'profile', 'prompt', 'readonly',
      'rel', 'rev', 'rows', 'rowspan', 'rules', 'scheme', 'scope', 'scrolling',
      'selected', 'shape', 'size', 'span', 'src', 'standby', 'start', 'style', 'summary',
      'tabindex', 'target', 'text', 'title', 'type', 'usemap', 'valign', 'value',
      'valuetype', 'version', 'vlink', 'vspace', 'width');
=======
=======
>>>>>>> origin/fixes_2_2
  HTMLAttributeTag : array [THTMLAttributeTag] of string = (
      'abbr', 'accept-charset', 'accept', 'accesskey', 'action', 'align', 'alt', 'archive',
      'axis', 'border', 'cellpadding', 'cellspacing', 'char', 'charoff', 'charset',
      'checked', 'cite', 'class', 'classid', 'codebase', 'codetype', 'cols',
      'colspan', 'content', 'coords', 'data', 'datetime', 'declare', 'defer',
      'dir', 'disabled', 'enctype', 'for', 'frame', 'frameborder', 'headers',
      'height', 'href', 'hreflang', 'http-equiv', 'id', 'ismap', 'label', 'lang',
      'longdesc', 'marginheight', 'marginwidth', 'maxlength', 'media', 'method',
      'multiple', 'name', 'nohref', 'noresize', 'onblur', 'onchange', 'onclick',
      'ondblclick', 'onfocus', 'onkeydown', 'onkeypress', 'onkeyup', 'onload',
      'onmousedown', 'onmousemove', 'onmouseout', 'onmouseover', 'onmouseup',
      'onreset', 'onselect', 'onsubmit', 'onunload', 'profile', 'readonly',
      'rel', 'rev', 'rows', 'rowspan', 'rules', 'scheme', 'scope', 'scrolling',
      'selected', 'shape', 'size', 'span', 'src', 'standby', 'style', 'summary',
      'tabindex', 'target', 'title', 'type', 'usemap', 'valign', 'value',
      'valuetype', 'width');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  HTMLColor : array [THTMLColor] of string =
    ('Black', 'Silver', 'Gray', 'White', 'Maroon', 'Red', 'Purple', 'Fuchsia',
     'Green', 'Lime', 'Olive', 'Yellow', 'Navy', 'Blue', 'Teal', 'Aqua');
  HTMLDir : array [THTMLDir] of string = ('','LTR','RTL');
  HTMLAlign : array [THTMLalign] of string = ('','left','center','right','justify','char');
  HTMLvalign : array [THTMLvalign] of string = ('','top','middle','bottom','baseline');
  HTMLframe : array [THTMLframe] of string =
      ('','void','above','below','hsides','vsides','lhs','rhs','box','border');
  HTMLrules : array [THTMLrules] of string = ('','none','groups','rows','cols','all');
  HTMLvaluetype : array [THTMLvaluetype] of string = ('','data','ref','object');
  HTMLshape : array [THTMLshape] of string = ('','default','rect','circle','poly');
  HTMLinputtype : array [THTMLinputtype] of string = ('','text','password','checkbox',
      'radio','submit','reset','file','hidden','image','button');
  HTMLbuttontype : array [THTMLbuttontype] of string = ('','submit','reset','button');

<<<<<<< HEAD
<<<<<<< HEAD
function ResolveHTMLEntityReference(const Name: WideString;
  var Entity: WideChar): Boolean;

function IsAutoClose(NewTag, OldTag: THTMLElementTag): Boolean;


implementation

uses SysUtils;

{ Define which elements auto-close other elements, modelled after libxml2.
  This is an array of variable-length lists, each terminated by etUnknown.
  Indices to first element of each list are provided by AutoCloseIndex array,
  which *must* be updated after any change. }
const
  AutoCloseTab: array[0..277] of THTMLElementTag = (

  etform,       etform, etp, ethr, eth1, eth2, eth3, eth4, eth5, eth6,
                etdl, etul, etol, etmenu, etdir, etaddress, etpre,
                ethead, etUnknown,
  ethead,       etp, etUnknown,
  ettitle,      etp, etUnknown,
  etbody,       ethead, etstyle, etlink, ettitle, etp, etUnknown,
  etframeset,   ethead, etstyle, etlink, ettitle, etp, etUnknown,
  etli,         etp, eth1, eth2, eth3, eth4, eth5, eth6, etdl, etaddress,
                etpre, ethead, etli, etUnknown,
  ethr,         etp, ethead, etUnknown,
  eth1,         etp, ethead, etUnknown,
  eth2,         etp, ethead, etUnknown,
  eth3,         etp, ethead, etUnknown,
  eth4,         etp, ethead, etUnknown,
  eth5,         etp, ethead, etUnknown,
  eth6,         etp, ethead, etUnknown,
  etdir,        etp, ethead, etUnknown,
  etaddress,    etp, ethead, etul, etUnknown,
  etpre,        etp, ethead, etul, etUnknown,
  etblockquote, etp, ethead, etUnknown,
  etdl,         etp, etdt, etmenu, etdir, etaddress, etpre,
                ethead, etUnknown,
  etdt,         etp, etmenu, etdir, etaddress, etpre,
                ethead, etdd, etUnknown,
  etdd,         etp, etmenu, etdir, etaddress, etpre,
                ethead, etdt, etUnknown,
  etul,         etp, ethead, etol, etmenu, etdir, etaddress, etpre, etUnknown,
  etol,         etp, ethead, etul, etUnknown,
  etmenu,       etp, ethead, etul, etUnknown,
  etp,          etp, ethead, eth1, eth2, eth3, eth4, eth5, eth6, etUnknown,
  etdiv,        etp, ethead, etUnknown,
  etnoscript,   etp, ethead, etUnknown,
  etcenter,     etfont, etb, eti, etp, ethead, etUnknown,
  eta,          eta, etUnknown,
  etcaption,    etp, etUnknown,
  etcolgroup,   etcaption, etcolgroup, etcol, etp, etUnknown,
  etcol,        etcaption, etcol, etp, etUnknown,
  ettable,      etp, ethead, eth1, eth2, eth3, eth4, eth5, eth6, etpre,
                eta, etUnknown,
  etth,         etth, ettd, etp, etspan, etfont, eta, etb, eti, etu, etUnknown,
  ettd,         etth, ettd, etp, etspan, etfont, eta, etb, eti, etu, etUnknown,
  ettr,         etth, ettd, ettr, etcaption, etcol, etcolgroup, etp, etUnknown,
  etthead,      etcaption, etcol, etcolgroup, etUnknown,
  ettfoot,      etth, ettd, ettr, etcaption, etcol, etcolgroup, etthead,
                ettbody, etp, etUnknown,
  ettbody,      etth, ettd, ettr, etcaption, etcol, etcolgroup, etthead,
                ettfoot, ettbody, etp, etUnknown,
  etoptgroup,   etoption, etUnknown,
  etoption,     etoption, etUnknown,
  etfieldset,   etlegend, etp, ethead, eth1, eth2, eth3, eth4, eth5, eth6,
                etpre, eta, etUnknown,
  etUnknown);

  AutoCloseIndex: array[0..40] of Integer = (
    0, 19, 22, 25, 32, 39, 53, 57, 61, 65, 69,
    73, 77, 81, 85, 90, 95, 99, 108, 117, 126,
    135, 140, 145, 155, 159, 163, 170, 173, 176,
    182, 187, 199, 210, 221, 230, 235, 246, 258,
    261, 264
  );

{ HTML entities, each preceded with its code. There is a separate list for
  each entity length, and each list is sorted by character codes.
  The sole purpose of using AnsiString here is staying compatible with Delphi 7,
  which is totally broken with respect to handling wide literals.
}

  ent_2 =
    #3#$9C  + 'Mu'+
    #3#$9D  + 'Nu'+
    #3#$A0  + 'Pi'+
    #3#$9E  + 'Xi'+
    #$22#$65+ 'ge'+
    #0#62   + 'gt'+
    #$22#$64+ 'le'+
    #0#60   + 'lt'+
    #3#$BC  + 'mu'+
    #$22#$60+ 'ne'+
    #$22#$0B+ 'ni'+
    #3#$BD  + 'nu'+
    #$22#$28+ 'or'+
    #3#$C0  + 'pi'+
    #3#$BE  + 'xi';

  ent_3 =
    #3#$A7  + 'Chi'+
    #0#208  + 'ETH'+
    #3#$97  + 'Eta'+
    #3#$A6  + 'Phi'+
    #3#$A8  + 'Psi'+
    #3#$A1  + 'Rho'+
    #3#$A4  + 'Tau'+
    #0#38   + 'amp'+
    #$22#$27+ 'and'+
    #$22#$20+ 'ang'+
    #$22#$29+ 'cap'+
    #3#$C7  + 'chi'+
    #$22#$2A+ 'cup'+
    #0#176  + 'deg'+
    #3#$B7  + 'eta'+
    #0#240  + 'eth'+
    #$22#$2B+ 'int'+
    #$25#$CA+ 'loz'+
    #$20#$0E+ 'lrm'+
    #0#172  + 'not'+
    #3#$C6  + 'phi'+
    #3#$D6  + 'piv'+
    #3#$C8  + 'psi'+
    #0#174  + 'reg'+
    #3#$C1  + 'rho'+
    #$20#$0F+ 'rlm'+
    #0#173  + 'shy'+
    #$22#$3C+ 'sim'+
    #$22#$82+ 'sub'+
    #$22#$11+ 'sum'+
    #$22#$83+ 'sup'+
    #3#$C4  + 'tau'+
    #0#168  + 'uml'+
    #0#165  + 'yen'+
    #$20#$0D+ 'zwj';

  ent_4 =
    #0#196  + 'Auml'+
    #3#$92  + 'Beta'+
    #0#203  + 'Euml'+
    #3#$99  + 'Iota'+
    #0#207  + 'Iuml'+
    #0#214  + 'Ouml'+
    #0#220  + 'Uuml'+
    #1#$78  + 'Yuml'+
    #3#$96  + 'Zeta'+

    #0#228  + 'auml'+
    #3#$B2  + 'beta'+
    #$20#$22+ 'bull'+
    #0#162  + 'cent'+
    #2#$C6  + 'circ'+
    #$22#$45+ 'cong'+
    #0#169  + 'copy'+
    #$21#$D3+ 'dArr'+
    #$21#$93+ 'darr'+
    #$20#$03+ 'emsp'+
    #$20#$02+ 'ensp'+
    #0#235  + 'euml'+
    #$20#$AC+ 'euro'+
    #1#$92  + 'fnof'+
    #$21#$D4+ 'hArr'+
    #$21#$94+ 'harr'+
    #3#$B9  + 'iota'+
    #$22#$08+ 'isin'+
    #0#239  + 'iuml'+
    #$21#$D0+ 'lArr'+
    #$23#$29+ 'lang'+
    #$21#$90+ 'larr'+
    #0#175  + 'macr'+
    #0#160  + 'nbsp'+
    #$22#$84+ 'nsub'+
    #0#170  + 'ordf'+
    #0#186  + 'ordm'+
    #0#246  + 'ouml'+
    #0#182  + 'para'+
    #$22#$02+ 'part'+
    #$22#$A5+ 'perp'+
    #$22#$0F+ 'prod'+
    #$22#$1D+ 'prop'+
    #0#34   + 'quot'+
    #$21#$D2+ 'rArr'+
    #$23#$2A+ 'rang'+
    #$21#$92+ 'rarr'+
    #$21#$1C+ 'real'+
    #$22#$C5+ 'sdot'+
    #0#167  + 'sect'+
    #$22#$86+ 'sube'+
    #0#185  + 'sup1'+
    #0#178  + 'sup2'+
    #0#179  + 'sup3'+
    #$22#$87+ 'supe'+
    #$21#$D1+ 'uArr'+
    #$21#$91+ 'uarr'+
    #0#252  + 'uuml'+
    #0#255  + 'yuml'+
    #3#$B6  + 'zeta'+
    #$20#$0C+ 'zwnj';

  ent_5 =
    #0#198  + 'AElig'+
    #0#194  + 'Acirc'+
    #3#$91  + 'Alpha'+
    #0#197  + 'Aring'+
    #3#$94  + 'Delta'+
    #0#202  + 'Ecirc'+
    #3#$93  + 'Gamma'+
    #0#206  + 'Icirc'+
    #3#$9A  + 'Kappa'+
    #1#$52  + 'OElig'+
    #0#212  + 'Ocirc'+
    #3#$A9  + 'Omega'+
    #$20#$33+ 'Prime'+
    #3#$A3  + 'Sigma'+
    #0#222  + 'THORN'+
    #3#$98  + 'Theta'+
    #0#219  + 'Ucirc'+

    #0#226  + 'acirc'+
    #0#180  + 'acute'+
    #0#230  + 'aelig'+
    #3#$B1  + 'alpha'+
    #0#229  + 'aring'+
    #$22#$48+ 'asymp'+
    #$20#$1E+ 'bdquo'+
    #0#184  + 'cedil'+
    #$26#$63+ 'clubs'+
    #$21#$B5+ 'crarr'+
    #3#$B4  + 'delta'+
    #$26#$66+ 'diams'+
    #0#234  + 'ecirc'+
    #$22#$05+ 'empty'+
    #$22#$61+ 'equiv'+
    #$22#$03+ 'exist'+
    #$20#$44+ 'frasl'+
    #3#$B3  + 'gamma'+
    #0#238  + 'icirc'+
    #0#161  + 'iexcl'+
    #$21#$11+ 'image'+
    #$22#$1E+ 'infin'+
    #3#$BA  + 'kappa'+
    #0#171  + 'laquo'+
    #$23#$08+ 'lceil'+
    #$20#$1C+ 'ldquo'+
    #$20#$18+ 'lsquo'+
    #$20#$14+ 'mdash'+
    #0#181  + 'micro'+
    #$22#$12+ 'minus'+
    #$22#$07+ 'nabla'+
    #$20#$13+ 'ndash'+
    #$22#$09+ 'notin'+
    #0#244  + 'ocirc'+
    #1#$53  + 'oelig'+
    #$20#$3E+ 'oline'+
    #3#$C9  + 'omega'+
    #$22#$95+ 'oplus'+
    #0#163  + 'pound'+
    #$20#$32+ 'prime'+
    #$22#$1A+ 'radic'+
    #0#187  + 'raquo'+
    #$23#$09+ 'rceil'+
    #$20#$1D+ 'rdquo'+
    #$20#$19+ 'rsquo'+
    #$20#$1A+ 'sbquo'+
    #3#$C3  + 'sigma'+
    #0#223  + 'szlig'+
    #3#$B8  + 'theta'+
    #0#254  + 'thorn'+
    #2#$DC  + 'tilde'+
    #0#215  + 'times'+
    #$21#$22+ 'trade'+
    #0#251  + 'ucirc'+
    #3#$D2  + 'upsih';

  ent_6 =
    #0#193  + 'Aacute'+
    #0#192  + 'Agrave'+
    #0#195  + 'Atilde'+
    #0#199  + 'Ccedil'+
    #$20#$21+ 'Dagger'+
    #0#201  + 'Eacute'+
    #0#200  + 'Egrave'+
    #0#205  + 'Iacute'+
    #0#204  + 'Igrave'+
    #3#$9B  + 'Lambda'+
    #0#209  + 'Ntilde'+
    #0#211  + 'Oacute'+
    #0#210  + 'Ograve'+
    #0#216  + 'Oslash'+
    #0#213  + 'Otilde'+
    #1#$60  + 'Scaron'+
    #0#218  + 'Uacute'+
    #0#217  + 'Ugrave'+
    #0#221  + 'Yacute'+

    #0#225  + 'aacute'+
    #0#224  + 'agrave'+
    #0#227  + 'atilde'+
    #0#166  + 'brvbar'+
    #0#231  + 'ccedil'+
    #0#164  + 'curren'+
    #$20#$20+ 'dagger'+
    #0#247  + 'divide'+
    #0#233  + 'eacute'+
    #0#232  + 'egrave'+
    #$22#$00+ 'forall'+
    #0#189  + 'frac12'+
    #0#188  + 'frac14'+
    #0#190  + 'frac34'+
    #$26#$65+ 'hearts'+
    #$20#$26+ 'hellip'+
    #0#237  + 'iacute'+
    #0#236  + 'igrave'+
    #0#191  + 'iquest'+
    #3#$BB  + 'lambda'+
    #$23#$0A+ 'lfloor'+
    #$22#$17+ 'lowast'+
    #$20#$39+ 'lsaquo'+
    #0#183  + 'middot'+
    #0#241  + 'ntilde'+
    #0#243  + 'oacute'+
    #0#242  + 'ograve'+
    #0#248  + 'oslash'+
    #0#245  + 'otilde'+
    #$22#$97+ 'otimes'+
    #$20#$30+ 'permil'+
    #0#177  + 'plusmn'+
    #$23#$0B+ 'rfloor'+
    #$20#$3A+ 'rsaquo'+
    #1#$61  + 'scaron'+
    #3#$C2  + 'sigmaf'+
    #$26#$60+ 'spades'+
    #$22#$34+ 'there4'+
    #$20#$09+ 'thinsp'+
    #0#250  + 'uacute'+
    #0#249  + 'ugrave'+
    #$21#$18+ 'weierp'+
    #0#253  + 'yacute';

  ent_7 =
    #3#$95  + 'Epsilon'+
    #3#$9F  + 'Omicron'+
    #3#$A5  + 'Upsilon'+
    #$21#$35+ 'alefsym'+
    #3#$B5  + 'epsilon'+
    #3#$BF  + 'omicron'+
    #3#$C5  + 'upsilon';

  ent_8 =
    #3#$D1  + 'thetasym';

  strs: array[2..8] of string = (
    ent_2, ent_3, ent_4, ent_5, ent_6, ent_7, ent_8
  );

function BSearch(P: PWideChar; Len: Integer; const data: string): WideChar;
var
  L, H, mid, J, C: Integer;
begin
  Result := #0;
  L := 0;
  H := (Length(data)+1) div (Len+2);
  while L <= H do
  begin
    mid := L + ((H - L) shr 1);
    J := 0;
    repeat
      C := ord(P[J]) - ord(data[mid*(Len+2)+3+J]);
      Inc(J);
    until (C <> 0) or (J >= Len);
    if C > 0 then L := mid + 1 else
    begin
      H := mid - 1;
      if C = 0 then
      begin
        Result := WideChar((ord(data[mid*(Len+2)+1]) shl 8) or ord(data[mid*(Len+2)+2]));
        Exit;
      end;
    end;
  end;
end;

{
  Remaining issues:
  1) UTF-16 surrogate pairs
  2) HTML accepts uppercase 'X' for hex notation, but XML does not.
  3) 'apos' is used in xml/xhtml, but not in HTML 4.01
}

function ResolveHTMLEntityReference(const Name: WideString;
  var Entity: WideChar): Boolean;
var
  i, L: Integer;
  value: Integer;
begin
  L := Length(Name);
  if (L > 1) and (Name[1] = '#') then
  begin
    value := 0;
    if (Name[2] = 'x') or (Name[2] = 'X') then
    begin
      i := 3;
      while i <= L do
      begin
        case Name[i] of
          '0'..'9': Value := Value * 16 + Ord(Name[i]) - Ord('0');
          'a'..'f': Value := Value * 16 + Ord(Name[i]) - (Ord('a') - 10);
          'A'..'F': Value := Value * 16 + Ord(Name[i]) - (Ord('A') - 10);
        else
          Break;
        end;
        Inc(i);
      end;
    end
    else
    begin
      i := 2;
      while i <= L do
      begin
        case Name[i] of
          '0'..'9': Value := Value * 10 + Ord(Name[i]) - Ord('0');
        else
          Break;
        end;
        Inc(i);
      end;
    end;
    Result := (i = L+1);
    if Result then
      Entity := WideChar(Value);
  end
  else
  begin
    case L of
      2..8: Entity := BSearch(PWideChar(Name), L, strs[L]);
    else
      Entity := #0;
    end;
    Result := (Entity <> #0);
  end;
end;

function IsAutoClose(NewTag, OldTag: THTMLElementTag): Boolean;
var
  i, j: Integer;
begin
  Result := False;
  for i := 0 to high(AutoCloseIndex) do
    if NewTag = AutoCloseTab[AutoCloseIndex[i]] then
    begin
      j := AutoCloseIndex[i]+1;
      while AutoCloseTab[j] <> etUnknown do
      begin
        if AutoCloseTab[j] = OldTag then
        begin
          Result := True;
          Exit;
        end;
        Inc(j);
      end;
      Exit;
    end;
=======
=======
>>>>>>> origin/fixes_2_2

  // ISO8859-1 mapping:
  HTMLEntities: array[#160..#255] of String = (
    // 160-191
    'nbsp', 'iexcl', 'cent', 'pound', 'curren', 'yen', 'brvbar', 'sect',
    'uml', 'copy', 'ordf', 'laquo', 'not', 'shy', 'reg', 'macr',
    'deg', 'plusmn', 'sup2', 'sup3', 'acute', 'micro', 'para', 'middot',
    'cedil', 'sup1', 'ordm', 'raquo', 'frac14', 'frac12', 'frac34', 'iquest',
    // 192-223
    'Agrave', 'Aacute', 'Acirc', 'Atilde', 'Auml', 'Aring', 'AElig', 'Ccedil',
    'Egrave', 'Eacute', 'Ecirc', 'Euml', 'Igrave', 'Iacute', 'Icirc', 'Iuml',
    'ETH', 'Ntilde', 'Ograve', 'Oacute', 'Ocirc', 'Otilde', 'Ouml', 'times',
    'Oslash', 'Ugrave', 'Uacute', 'Ucirc', 'Uuml', 'Yacute', 'THORN', 'szlig',
    // 224-255
    'agrave', 'aacute', 'acirc', 'atilde', 'auml', 'aring', 'aelig', 'ccedil',
    'egrave', 'eacute', 'ecirc', 'euml', 'igrave', 'iacute', 'icirc', 'iuml',
    'eth', 'ntilde', 'ograve', 'oacute', 'ocirc', 'otilde', 'ouml', 'divide',
    'oslash', 'ugrave', 'uacute', 'ucirc', 'uuml', 'yacute', 'thorn', 'yuml');


  UnicodeHTMLEntities: array[0..141] of String = (
    'Alpha',    // #913
    'Beta',     // #914
    'Gamma',    // #915
    'Delta',    // #916
    'Epsilon',  // #917
    'Zeta',     // #918
    'Eta',      // #919
    'Theta',    // #920
    'Iota',     // #921
    'Kappa',    // #922
    'Lambda',   // #923
    'Mu',       // #924
    'Nu',       // #925
    'Xi',       // #926
    'Omicron',  // #927
    'Pi',       // #928
    'Rho',      // #929
    'Sigma',    // #931
    'Tau',      // #932
    'Upsilon',  // #933
    'Phi',      // #934
    'Chi',      // #935
    'Psi',      // #936
    'Omega',    // #937
    'alpha',    // #945
    'beta',     // #946
    'gamma',    // #947
    'delta',    // #948
    'epsilon',  // #949
    'zeta',     // #950
    'eta',      // #951
    'theta',    // #952
    'iota',     // #953
    'kappa',    // #954
    'lambda',   // #955
    'mu',       // #956
    'nu',       // #957
    'xi',       // #958
    'omicron',  // #959
    'pi',       // #960
    'rho',      // #961
    'sigmaf',   // #962
    'sigma',    // #963
    'tau',      // #964
    'upsilon',  // #965
    'phi',      // #966
    'chi',      // #967
    'psi',      // #968
    'omega',    // #969
    'thetasym', // #977
    'upsih',    // #978
    'piv',      // #982
    'ensp',     // #8194
    'emsp',     // #8195
    'thinsp',   // #8201
    'zwnj',     // #8204
    'zwj',      // #8205
    'lrm',      // #8206
    'rlm',      // #8207
    'ndash',    // #8211
    'mdash',    // #8212
    'lsquo',    // #8216
    'rsquo',    // #8217
    'sbquo',    // #8218
    'ldquo',    // #8220
    'rdquo',    // #8221
    'bdquo',    // #8222
    'dagger',   // #8224
    'Dagger',   // #8225
    'bull',     // #8226
    'hellip',   // #8230
    'permil',   // #8240
    'prime',    // #8242
    'lsaquo',   // #8249
    'rsaquo',   // #8250
    'oline',    // #8254
    'frasl',    // #8260
    'image',    // #8465
    'weierp',   // #8472
    'real',     // #8476
    'trade',    // #8482
    'alefsym',  // #8501
    'larr',     // #8592
    'uarr',     // #8593
    'rarr',     // #8594
    'darr',     // #8595
    'harr',     // #8596
    'crarr',    // #8629
    'lArr',     // #8656
    'uArr',     // #8657
    'rArr',     // #8658
    'dArr',     // #8659
    'hArr',     // #8660
    'forall',   // #8704
    'part',     // #8706
    'exist',    // #8707
    'empty',    // #8709
    'nabla',    // #8711
    'isin',     // #8712
    'notin',    // #8713
    'ni',       // #8715
    'prod',     // #8719
    'sum',      // #8721
    'minus',    // #8722
    'lowast',   // #8727
    'radic',    // #8730
    'prop',     // #8733
    'infin',    // #8734
    'ang',      // #8736
    'and',      // #8743
    'or',       // #8744
    'cap',      // #8745
    'cup',      // #8746
    'int',      // #8747
    'there4',   // #8756
    'sim',      // #8764
    'cong',     // #8773
    'asymp',    // #8776
    'ne',       // #8800
    'equiv',    // #8801
    'le',       // #8804
    'ge',       // #8805
    'sub',      // #8834
    'sup',      // #8835
    'nsub',     // #8836
    'sube',     // #8838
    'supe',     // #8839
    'oplus',    // #8853
    'otimes',   // #8855
    'perp',     // #8869
    'sdot',     // #8901
    'lceil',    // #8968
    'rceil',    // #8969
    'lfloor',   // #8970
    'rfloor',   // #8971
    'lang',     // #9001
    'rang',     // #9002
    'loz',      // #9674
    'spades',   // #9824
    'clubs',    // #9827
    'hearts',   // #9829
    'diams'     // #9830
  );



function ResolveHTMLEntityReference(const Name: String;
  var Entity: Char): Boolean;



implementation

uses SysUtils;

function ResolveHTMLEntityReference(const Name: String;
  var Entity: Char): Boolean;
var
  Ent: Char;
  i: Integer;
begin
  if Name = 'quot' then
  begin
    Entity := '"';
    Result := True;
  end else if Name = 'apos' then
  begin
    Entity := '''';
    Result := True;
  end else if Name = 'amp' then
  begin
    Entity := '&';
    Result := True;
  end else if Name = 'lt' then
  begin
    Entity := '<';
    Result := True;
  end else if Name = 'gt' then
  begin
    Entity := '>';
    Result := True;
  end else if (Length(Name) > 0) and (Name[1] = '#') then
  begin
    for i := 2 to Length(Name) do
      if (Name[i] < '0') or (Name[i] > '9') then
        break;
    if i > 2 then
    begin
      Entity := Chr(StrToInt(Copy(Name, 2, i - 1)));
      Result := True;
    end else
      Result := False;
  end else
  begin
    for Ent := Low(HTMLEntities) to High(HTMLEntities) do
      if HTMLEntities[Ent] = Name then
      begin
        Entity := Ent;
        Result := True;
        exit;
      end;
    Result := False;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

end.
