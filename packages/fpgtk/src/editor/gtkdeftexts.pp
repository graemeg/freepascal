{$mode objfpc}{$h+}
unit GtkDefTexts;

interface

resourcestring

// SettingsRec
  sOptions = 'Options';
  sOk = '  Ok  ';
  sCancel = 'Cancel';
  sExtention = 'Extention';
  sSaveonclose = '_Save on close';
  sFileFormat = 'File format';
  sMRUcount = 'MRU count';
  sComponentBin = 'Component Binary';
  sHintCompBin = 'Streaming of the component';
  sComponentText = 'Component Text';
  sHintCompText = 'Converting the streamed object to text (Delphi compatible)';
  sHomeText = 'Private format';
  sHintHomeText = 'Text format not compatible with streaming or Delphi';
  sProgressWindow = 'S_how progress';

// ProgWin
  ProgressWinTitle = 'Progres generation unit';

// GtkEditor
  sEditorTitle = 'Pascal GTK editor';
  sComponent = 'Component';
  sObject = 'Object';
  sDefinition = 'Definition';
  sParameters = 'Parameters';
  sRead = 'Read';
  sWrite = 'Write';
  sUnitName = 'Unit name';
  sUseslist = 'Uses list';
  sGtkPrefix = 'Gtk prefix';
  sName = 'Name';
  sInherits = 'Inherits from';
  sGtkName = 'Gtk name';
  sCreateObject = 'Create object';
  sWithPointer = 'With pointer';
  sCreateParams = 'Create params';
  sGtkFunctionName = 'Gtk func name';
  sType = 'Type';
  sTypes = 'Types';
  sPascalType = 'Pascal type';
  sSection = 'Section';
  // section types
  sPrivate = 'Private';
  sProtected = 'Protected';
  sPublic = 'Public';
  sPublished = 'Published';
  // Property types
  sField = 'Field';
  sProperty = 'Property';
  sFunction = 'Function';
  sProcedure = 'Procedure';
  sSignal = 'Signal';
  sHelperproc = 'Helperproc';
  sHelperFunc = 'HelperFunc';
  sSignalType = 'SignalType';
  sDeclarations = 'Declarations';
  sTypeDecl = 'TypeDecl';
  sConstructor = 'Constructor';
  sDestructor = 'Destructor';
  sInitialization = 'Initialization';
  sFinalization = 'Finalization';
  sCode = 'Code';
  // function Types
  sOverride = 'Override';
  sVirtual = 'Virtual';
  sDynamic = 'Dynamic';
  sAbstract = 'Abstract';
  sCDecl = 'CDecl';
  sOverload = 'Overload';
  sReintroduce = 'Reintroduce';
  // Parameter types
  sNothing = 'Nothing';
  sVar = 'Var';
  sConst = 'Const';
  // Property read types
  sGtkFunction = 'Gtk function';
  sObjectField = 'Object field';
  sObjectFunction = 'Object function';
  sNotImplemented= 'Not implemented';
  sGtkMacro = 'Gtk macro';
  sExistingFunc = 'Existing function';
  // Property write types (extra)
  sGtkProcedure= 'Gtk Procedure';
  sObjectProcedure= 'Object Procedure';
  sExistingProc = 'Existing procedure';
  // Other
  sConvert = 'Convert';
  // Menu
  smFile = '_File';
  smFileNew = '_New';
  smFileOpen = '_Open';
  smFileReopen = '_Reopen';
  smFileSave = '_Save';
  smFileSaveAs = 'Save _as';
  smFileExit = 'E_xit';
  smTools = '_Tools';
  smToolsGenerate = '_Generate';
  smToolsOptions = '_Options';
  smHelp = '_Help';
  smHelpAbout = '_About';
  smHelpInfo = '_Info';
  smEdit = '_Edit';
  smEditObject = '_Object';
  smEditProperty = '_Property';
  smEditParameter = 'P_arameter';

// About
  sAboutTitle = 'About';
  sAbout1 = 'Editor to generate FPGTK unit,';
  sAbout2 = 'Or similar units.';
  sAboutVersion  = 'Version: 1.0';
  sAboutDesigner = 'Designer: Luk Vandelaer';

// Help
  sInfoTitle = 'Help';
  sInfoMessage = 'Not yet implemented (searching for FPDoc and help display component)';

// ButtonRow
  sNew = 'New';
  smAdd = '_Add';
  smCopy = '_Copy';
  smDelete = '_Delete';
  smUp = 'Move _Up';
  smDown = 'Move D_own';
  smFirst = '_First';
  smPrevious = '_Previous';
  smNext = '_Next';
  smLast = '_Last';

implementation

end.
