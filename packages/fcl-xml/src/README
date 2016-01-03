Free Pascal XML units
=====================

DOM
---
Implements most of the DOM level 1 specification and supports some of the
DOM level 2 extensions.


XMLRead
-------
Provides an XML reader, which can read XML data from a file or stream.
The parser can read files encoded in UTF-8, UTF-16 (both endianness),
and ISO-8859-1. It supports DTD validation.
Regarding entity references: The pre-defined entities "lt", "gt", "amp", "apos"
and "quot", and internal entities declared in DTD, are replaced by their
defined values during reading. Ability to resolve external entities is
currently limited to the file system.
Regarding whitespace handling: By default, whitespace directly after the beginning of a
tag is discarded, and sections of the XML file which contain only whitespace and
no other text content are discarded as well. However, whitespace-preserving
mode can be enabled by setting TDOMParser.Options.PreserveWhitespace property to
True.


XMLWrite
--------
Writes a DOM structure as XML data into a file or stream. It can deal both with
XML files and XML fragments.
At the moment it supports only the UTF-8 output endcoding.
Please note that the writer replaces some characters by entity references
automatically:
For normal text nodes, the following replacements will be done:
'<' => '&lt;'
'>' => '&gt;'
'&' => '&amp;'
For attribute values, additionally '"' gets replaced by '&quot;', and characters
#9, #10 and #13 are escaped using numerical references. Single apostrophes (')
don't need to get converted, as values are already written using "" quotes.
The XML reader (in xmlread.pp) will convert these entity references back to
their original characters.


XPath
-----
Just a XPath implementation. Should be fairly completed, but there hasn't been
further development recently.


HTMLDefs
--------
Contains basic HTML declarations.


HTMLElements
------------
Implements a DOM for HTML content. Contains a TDOMElement descendent for
all valid HTML 4.1 tags.

THtmlCustomElement:
  Basis for all HTML tag elements.
THTMLDocument:
  TDOMDocument descendent
THTMLIDElement:
  element representing <ID> tag 

All tags are in tagsintf.inc.


HTMLWriter
----------
Implements a verified HTML producer.

THTMLwriter:
  This is a class which allows to write certified correct HTML.
  It works using the DOM for HTML.
  It also has forms support.

Writing HTML is done as follows:

  StartBold;
  Write('This text is bold');
  EndBold;
or
  Bold('This text is bold');

But the following is also possible
  Bold(Center('Bold centered text'));

Open tags will be closed automatically.

wtagsintf.inc contains all possible tags.
