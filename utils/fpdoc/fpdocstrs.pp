unit fpdocstrs;

{$mode objfpc}{$H+}

interface

resourcestring
  // Output strings
  SDocPackageTitle           = 'Reference for package ''%s''';
  SDocPackageMenuTitle       = 'Package ''%s''';
  SDocPackageLinkTitle       = 'Package';
  SDocPrograms               = 'Programs';
  SDocUnits                  = 'Units';
  SDocUnitTitle              = 'Reference for unit ''%s''';
  SDocUnitMenuTitle          = 'Unit ''%s''';
  SDocInheritanceHierarchy   = 'Inheritance Hierarchy';
  SDocInterfaceSection       = 'Interface section';
  SDocImplementationSection  = 'Implementation section';
  SDocUsedUnits              = 'Used units';
  SDocUsedUnitsByUnitXY      = 'Used units by unit ''%s''';
  SDocConstsTypesVars        = 'Constants, types and variables';
  SDocResStrings             = 'Resource strings';
  SDocTypes                  = 'Types';
  SDocType                   = 'Type';
  SDocConstants              = 'Constants';
  SDocConstant               = 'Constant';
  SDocClasses                = 'Classes';
  SDocClass                  = 'Class';
  SDocProceduresAndFunctions = 'Procedures and functions';
  SDocProcedureOrFunction    = 'Procedure/function';
  SDocVariables              = 'Variables';
  SDocVariable               = 'Variable';
  SDocIdentifierIndex        = 'Index';
  SDocPackageClassHierarchy  = 'Class hierarchy';
  SDocModuleIndex            = 'Index of all identifiers in unit ''%s''';
  SDocPackageIndex           = 'Index of all identifiers in package ''%s''';
  SDocUnitOverview           = 'Overview of unit ''%s''';
  SDocOverview               = 'Overview';
  SDocSearch                 = 'Search';
  SDocDeclaration            = 'Declaration';
  SDocDescription            = 'Description';
  SDocErrors                 = 'Errors';
  SDocVersion                = 'Version info';
  SDocSeeAlso                = 'See also';
  SDocExample                = 'Example';
  SDocArguments              = 'Arguments';
  SDocFunctionResult         = 'Function result';
  SDocRemark                 = 'Remark:   ';
  SDocMethodOverview         = 'Method overview';
  SDocPropertyOverview       = 'Property overview';
  SDocEventOverview          = 'Event overview';
  SDocInterfacesOverview     = 'Interfaces overview';
  SDocInterface              = 'Interfaces';
  SDocPage                   = 'Page';
  SDocMember                 = 'Member';
  SDocMembers                = 'Members';
  SDocField                  = 'Field';
  SDocMethod                 = 'Method';
  SDocProperty               = 'Property';
  SDocAccess                 = 'Access';
  SDocInheritance            = 'Inheritance';
  SDocProperties             = 'Properties';
  SDocMethods                = 'Methods';
  SDocEvents                 = 'Events';
  SDocByName                 = 'by Name';
  SDocByInheritance          = 'By inheritance';
  SDocValue                  = 'Value';
  SDocExplanation            = 'Explanation';
  SDocProcedure              = 'Procedure';
  SDocValuesForEnum          = 'Enumeration values for type %s';
  SDocSourcePosition         = 'Source position: %s line %d';
  SDocSynopsis               = 'Synopsis';
  SDocVisibility             = 'Visibility';
  SDocOpaque                 = 'Opaque type';
  SDocDateGenerated          = 'Documentation generated on: %s';
  // The next line requires leading/trailing space due to XML comment layout:
  SDocGeneratedByComment     = ' Generated using FPDoc - (c) 2000-2021 FPC contributors and Sebastian Guenther, sg@freepascal.org ';
  SDocNotes                  = 'Notes';
  SDocName                   = 'Name';
  SDocType_s                 = 'Type(s)';
  SDocTopic                  = 'Topic';
  SDocNoneAVailable          = 'No members available';

  // Topics
  SDocRelatedTopics = 'Related topics';
  SDocUp            = 'Up';
  SDocNext          = 'Next';
  SDocPrevious      = 'Previous';

  // Various backend constants
  SDocChapter    = 'Chapter';
  SDocSection    = 'Section';
  SDocSubSection = 'Subsection';
  SDocTable      = 'Table';
  SDocListing    = 'Listing';

  // Man page usage
  SManUsageManSection         = 'Use ASection as the man page section';
  SManUsageNoUnitPrefix       = 'Do not prefix man pages with unit name.';
  SManUsageWriterDescr        = 'UNIX man page output.';
  SManUsagePackageDescription = 'Use descr as the description of man pages';

  // HTML usage
  SHTMLUsageFooter = 'Append xhtml (@filename reads from file) as footer to html page';
  SHTMLUsageNavigator = 'Append xhtml (@filename reads from file) in navigator bar';
  SHTMLUsageHeader = 'Append xhtml (@filename reads from file) as header to html page below navigation bar';
  SHTMLUsageFooterDate = 'Append footer with date. fmt is Optional format for FormatDateTime';
  SHTMLUsageCharset = 'Set the HTML character set';
  SHTMLHtmlSearch = 'Add search page with given name to the menu bar';
  SHTMLIndexColcount = 'Use N columns in the identifier index pages';
  SHTMLImageUrl = 'Prefix image URLs with url';
  SHTMLDisableMenuBrackets = 'Disable ''['' and '']'' characters around menu items at the top of the page. Useful for custom css';

  // CHM usage
  SCHMUsageTOC     = 'Use [File] as the table of contents. Usually a .hhc file.';
  SCHMUsageIndex   = 'Use [File] as the index. Usually a .hhk file.';
  SCHMUsageDefPage = 'Set the "Home" page relative to where it lives in the chm. i.e. "/index.html"';
  SCHMUsageOtrFiles= 'A txt file containing a list of files to be added relative to the working directory.';
  SCHMUsageCSSFile = 'Filename of a .css file to be included in the chm.';
  SCHMUsageAutoTOC = 'Automatically generate a Table of Contents. Ignores --toc-file';
  SCHMUsageAutoIDX = 'Automatically generate an Index. Ignores --index-file';
  SCHMUsageMakeSearch = 'Automatically generate a Search Index from filenames that match *.htm*';
  SCHMUsageChmTitle= 'Title of the chm. Defaults to the value from --package';

  // MarkDown usage
  SMDUsageFooter = 'Append markdown (@filename reads from file) as footer to every markdown page';
  SMDUsageHeader = 'Prepend markdown (@filename reads from file) as header to every markdown page';
  SMDIndexColcount = 'Use N columns in the identifier index pages';
  SMDImageUrl = 'Prefix image URLs with url';
  SMDTheme = 'Use name as theme name';
  SMDNavigation = 'Use scheme for navigation tree, here scheme is one of:';
  SMDNavSubtree = '    UnitSubTree : put all units in a sub tree of a Units node';
  SMDNavTree =    '    UnitTree : put every units as a node on the same level as packages node';

  SXMLUsageFlatStructure  = 'Use a flat output structure of XML files and directories';
  SXMLUsageSource  = 'Include source file and line info in generated XML';

  // Linear usage
  SLinearUsageDupLinkedDocsP1 = 'Duplicate linked element documentation in';
  SLinearUsageDupLinkedDocsP2 = 'descendant classes.';

  STitle           = 'FPDoc - Free Pascal Documentation Tool';
  SVersion         = 'Version %s [%s]';
  SCopyright1      = '(c) 2000 - 2003 Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org';
  SCopyright2      = '(c) 2005 - 2021 various FPC contributors';

  SCmdLineHelp     = 'Usage: %s [options]';
  SUsageOption008  = '--base-descr-dir=DIR prefix all description files with this directory';
  SUsageOption009  = '--base-input-dir=DIR prefix all input files with this directory';
  SUsageOption010  = '--content         Create content file for package cross-references';
  SUsageOption020  = '--cputarget=value Set the target CPU for the scanner.';
  SUsageOption030  = '--descr=file      use file as description file, e.g.: ';
  SUsageOption035  = '                  --descr=c:\WIP\myzipperdoc.xml';
  SUsageOption040  = '                  This option is allowed more than once';
  SUsageOption050  = '--descr-dir=Dir   Add All XML files in Dir to list of description files';
  SUsageOption060  = '--format=fmt      Select output format.';
  SUsageOption070  = '--help            Show this help.';
  SUsageOption080  = '--hide-protected  Do not show protected methods in overview';
  SUsageOption090  = '--import=file     Import content file for package cross-references';
  SUsageOption100  = '--input=cmd       use cmd as input for the parser, e.g.:';
  SUsageOption110  = '           --input=C:\fpc\packages\paszlib\src\zipper.pp';
  SUsageOption120  = '                  At least one input option is required.';
  SUsageOption130  = '--input-dir=Dir   Add All *.pp and *.pas files in Dir to list of input files';
  SUsageOption140  = '--lang=lng        Select output language.';
  SUsageOption145  = '--macro=name=value Define a macro to preprocess the project file with.';
  SUsageOption150  = '--ostarget=value  Set the target OS for the scanner.';
  SUsageOption160  = '--output=name     use name as the output name.';
  SUsageOption170  = '                  Each backend interprets this as needed.';
  SUsageOption180  = '--package=name    Set the package name for which to create output,';
  SUsageOption190  = '                  e.g. --package=fcl';
  SUsageOption200  = '--project=file    Use file as project file';
  SUsageOption210  = '--show-private    Show private methods.';
  SUsageOption211  = '--fallback-seealso-links';
  SUsageOption212  = '                  Simplify seealso links by exluding last link level';
  SUsageOption215  = '--stop-on-parser-error';
  SUsageOption215A = '                  Stop when a parser error occurs. Default is to ignore parser errors.';
  SUsageOption220  = '--warn-no-node    Warn if no documentation node was found.';
  SUsageOption221  = '--warn-documentation-empty    Warn if documentation is empty.';
  SUsageOption222  = '--warn-xct        Warn if an external class could not be resolved.';
  SUsageOption223  = '--info-used-file  Output the file path of an implicitly processed file.';
  SUsageOption230  = '--mo-dir=dir      Set directory where language files reside to dir';
  SUsageOption240  = '--parse-impl      (Experimental) try to parse implementation too';
  SUsageOption250  = '--dont-trim       Do not trim XML contents. Useful for preserving';
  SUsageOption260  = '                  formatting inside e.g <pre> tags';
  SUsageOption270  = '--write-project=file';
  SUsageOption280  = '                  Do not write documentation, create project file instead';
  SUsageOption290  = '--verbose         Write more information on the screen';
  SUsageOption300  = '--dry-run         Only parse sources and XML, do not create output';
  SUsageOption310  = '--write-project=file';
  SUsageOption320  = '                  Write all command-line options to a project file';
  SUsageSubNames   = 'Use the file subnames instead the indexes as postfixes';
  SUsageOnlyPages  = 'Only write pages in LIST, LIST is comma-separated list of filenames or @filename where the named file contains 1 file per line.';

  SUsageFormats        = 'The following output formats are supported by this fpdoc:';
  SUsageBackendHelp    = 'Specify an output format, combined with --help to get more help for this backend.';
  SUsageFormatSpecific = 'Output format "%s" supports the following options:';
  SCmdLineErrInvalidMacro     = 'Macro needs to be in the form name=value';

  SCmdLineInvalidOption       = 'Ignoring unknown option "%s"';
  SCmdLineInvalidFormat       = 'Invalid format "%s" specified';
  SCmdLineOutputOptionMissing = 'Need an output filename, please specify one with --output=<filename>';
  SWritingPages               = 'Writing %d pages...';
  SNeedPackageName            = 'No package name specified. Please specify one using the --package option.';
  SAvailablePackages          = 'Available packages: ';
  SDone                       = 'Done.';
  SErrCouldNotCreateOutputDir = 'Could not create output directory "%s"';
  SErrCouldNotCreateFile      = 'Could not create file "%s": %s';
  SSeeURL                     = '(See %s)';      // For linear text writers.
  SParsingUsedUnit            = 'Parsing used unit "%s" with commandLine "%s"';

  SErrFileWriting = 'An error occurred during writing of file "%s": %s';

  SErrInvalidShortDescr = 'Invalid short description';
  SErrInvalidDescr = 'Invalid description (illegal XML element: "%s")';
  SErrInvalidParaContent = 'Invalid paragraph content';
  SErrInvalidElementInList = 'Invalid element in list - only "li" allowed';
  SErrInvalidListContent = 'Invalid list content';
  SErrInvalidRemarkContent = 'Invalid <remark> content (illegal XML element: "%s")';
  SErrListIsEmpty = 'List is empty - need at least one "li" element';
  SErrInvalidDefinitionTermContent = 'Invalid content in definition term';
  SErrDefinitionEntryMissing = 'Definition entry after definition term is missing';
  SErrInvalidBorderValue = 'Invalid "border" value for %s';
  SErrInvalidTableContent = 'Invalid table content';
  SErrTableRowEmpty = 'Table row is empty (no "td" elements found)';
  SErrInvalidContentBeforeSectionTitle = 'Invalid content before section title';
  SErrSectionTitleExpected = 'Section title ("title" element) expected';

  SErrDescrTagUnknown = 'Warning: Unknown tag "%s" in description';
  SErrUnknownEntityReference = 'Warning: Unknown entity reference "&%s;" found';
  SErrUnknownLinkID = 'Warning: Target ID of <link> in unit "%s", element "%s", is unknown: "%s"';
  SErrUnknownPrintShortID = 'Warning: Target ID of <printshort> is unknown: "%s"';
  SErrUnknownLink = 'Could not resolve link to "%s"';
  SErralreadyRegistered = 'Class for output format "%s" already registered';
  SErrUnknownWriterClass = 'Unknown output format "%s"';

  SErrCannotChangeIndentSizeWhenIndented = 'Cannot change indent size while text is indented.';
  SErrIndentMismatch = 'Indent mismatch: trying to undent when current indent too small';
  SErrNotInList = 'Not in list';
  SErrPopListStack = 'Pop list stack list type mismatch';
  SErrMinListStack = 'Min list stack reached';
  SErrMaxListStack = 'Max list stack reached';
  SErrMinIndentStack = 'Min indent stack reached';
  SErrMaxIndentStack = 'Max indent stack reached';

  // doc xml
  SErrInvalidRootNode = 'Invalid options root node: Got "%s", expected "docproject"';
  SErrNoPackagesNode = 'No "packages" node found in docproject';
  SErrNoInputFile = 'unit tag without file attribute found';
  SErrNoDescrFile = 'description tag without file attribute';
  SErrNoImportFile = 'Import tag without file attribute';
  SErrNoImportPrefix = 'Import tag without prefix attribute';

implementation

end.

