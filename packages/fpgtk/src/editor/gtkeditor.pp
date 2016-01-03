{$mode objfpc}{$h+}
unit GtkEditor;

interface
{__$define debug}
uses sysutils, classes,
     glib, gdk, gtk, FPGtk, FPgtkExt,
     buttonrow, ObjectDef, SettingsRec;

type

  TGtkEditorWindow = class (TFPgtkWindow)
  private
    FSettings : TSettingsRec;
  { widgets in the window }
    // visual arrangement
    LObjects, LProperties, LParams : TFPgtkScrollCList;
    BrObjects, BrProperties, BrParams : TButtonRow;
    PObject, PProperty, PParam : TFPgtkHPaned;
    FDefinition, FObject, FProperty : TFPgtkFrame;
    FParam : TFPgtkBox;
    // definition
    DUnitName, DGtkPrefix, DUsesList : TFPgtkentry;
    // object
    OName, OInherit, OGtkName, OGtkFuncName, OCreateParams : TFPgtkentry;
    OCreateObject, OWithPointer : TFPgtkToggleButton;
    // property
    PType : TFPgtkOptionMenu; // Or TFPgtkCombo
    Bladen : TFPgtkNotebook;
    BDefinition, BParameter, BRead, BWrite, BFunction, BCode : TFPgtkWidget;
      // definition
      PName, PPascalType, PGtkName : TFPgtkEntry;
      PSection : TFPgtkOptionMenu; // Or TFPgtkCombo
      // parameter
      ParamName, ParamPascalType : TFPgtkentry;
      ParamType : TFPgtkOptionmenu; // Or TFPgtkCombo
      ParamConvert : TFPgtkToggleButton;
      // read
      PRType : TFPgtkOptionMenu;
      PRGtkName : TFPgtkEntry;
      PRCode : TFPgtkScrollText;
      PRConvert : TFPgtkToggleButton;
      // write
      PWType : TFPgtkOptionMenu;
      PWGtkName : TFPgtkEntry;
      PWCode : TFPgtkScrollText;
      PWConvert : TFPgtkToggleButton;
      // function
      POverride, PVirtual, PDynamic,
      PAbstract, PCDecl, POverload,
      PReintroduce : TFPgtkTogglebutton;
      PFCode : TFPgtkScrollText;
      // Code
      PCode : TFPgtkScrollText;
  { CollectionItems that are currently shown }
    ciObject : TObjectItem;
    ciProperty : TPropertyItem;
    ciParameter : TParameterItem;
  { Defs Property }
    FDefs : TObjectDefs;
  { entry saving procedures/signals }
    // Definition
    procedure ChangedDUsesList (Sender:TFPgtkObject; data:pointer);
    procedure ChangedDUnitName (Sender:TFPgtkObject; data:pointer);
    procedure ChangedDGtkPrefix (Sender:TFPgtkObject; data:pointer);
    // Object
    procedure ChangedOName (Sender:TFPgtkObject; data:pointer);
    procedure ChangedOInherit (Sender:TFPgtkObject; data:pointer);
    procedure ChangedOGtkName (Sender:TFPgtkObject; data:pointer);
    procedure ChangedOGtkFuncName (Sender:TFPgtkObject; data:pointer);
    procedure ChangedOCreateParams (Sender:TFPgtkObject; data:pointer);
    procedure ChangedOCreateObject (Sender:TFPgtkObject; data:pointer);
    procedure ChangedOWithPointer (Sender:TFPgtkObject; data:pointer);
    // Property
    procedure ChangedPType (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPName (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPPascalType (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPGtkName (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPSection (Sender:TFPgtkObject; data:pointer);
      // parameter
      procedure ChangedParamName (Sender:TFPgtkObject; data:pointer);
      procedure ChangedParamPascalType (Sender:TFPgtkObject; data:pointer);
      procedure ChangedParamType (Sender:TFPgtkObject; data:pointer);
      procedure ChangedParamConvert (Sender:TFPgtkObject; data:pointer);
      // read
      procedure ChangedPRType (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPRConvert (Sender:TFPgtkObject; data:pointer);
      // write
      procedure ChangedPWType (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPWGtkName (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPWCode (Sender:TFPgtkObject; data:pointer);
      procedure ChangedPWConvert (Sender:TFPgtkObject; data:pointer);
      // function
      procedure ChangedPFuncType (Sender:TFPgtkObject; data:pointer);
      // Code
      procedure ChangedPCode (Sender:TFPgtkObject; data:pointer);
  { Showing procedures }
    RefreshingParam, RefreshingProperty, RefreshingObject, RefreshingDefinition : boolean;
    PropPixs : array [0..13] of PGdkPixmap;
    PropMasks : array [0..13] of PGdkBitmap;
    procedure CreatePixMaps;
    procedure PropertyIcon (Item:TCollectionItem; var Pixmap:PGdkPixMap; var mask:PGdkBitmap);
    procedure EnDisablePages (pt : TPropType);
    procedure RefreshParam (Selected : TCollectionItem; NeedFocus:boolean);
    procedure RefreshProperty (Selected : TCollectionItem; NeedFocus:boolean);
    procedure RefreshObject (Selected : TCollectionItem; NeedFocus:boolean);
    procedure RefreshDefinition;
    procedure ObjectDisplayChanged;
    procedure PropertyDisplayChanged;
    procedure ParamDisplayChanged;
    procedure ComposeWindow;
  { File and menu handling }
    FFileName : string;
    HasAFile : boolean;
    FReopenList : TStrings;
    MenuEditObject, MenuEditProperty, MenuEditParameter,
    MenuFileReopen : TFPgtkMenuItem;
    AccelGroup : integer;
    procedure NewFilename (NewName : string);
    procedure BuildReopenList;
    procedure DataRead (filename : string);
    procedure DataWrite (filename : string);
    procedure Generate;
  { Menu signals }
    procedure FileNew (Sender : TFPgtkObject; data : pointer);
    procedure FileOpen (Sender : TFPgtkObject; data : pointer);
    procedure FileSave (Sender : TFPgtkObject; data : pointer);
    procedure FileSaveAs (Sender : TFPgtkObject; data : pointer);
    procedure FileExit (Sender : TFPgtkObject; data : pointer);
    procedure ToolsGenerate (Sender : TFPgtkObject; data : pointer);
    procedure ToolsOptions (Sender : TFPgtkObject; data : pointer);
    procedure HelpInfo (Sender : TFPgtkObject; data : pointer);
    procedure HelpAbout (Sender : TFPgtkObject; data : pointer);
    procedure ToolbarReopen (Sender : TFpgtkObject; data : pointer);
  { Dialog Procedures }
    procedure DialogSetFilename (Sender:TFPgtkWindow;
              aDialogResult:pointer; Action:integer; initiator:TFPgtkObject);
    procedure FileReopen (Sender : TFPgtkObject; data : pointer);
  { Settings procedures }
    procedure ReadSettings;
    procedure WriteSettings (Sender : TFPgtkObject; data : pointer);
  public
    constructor create;
    destructor Destroy; override;
  end;

implementation

uses XPMs, GtkDefTexts, inifiles, ProgWin;

Type
  TRightLabel = class (TFPgtkLabel)
  public
    constructor create(aText : string);
  end;

constructor TRightLabel.Create (aText : string);
begin
  inherited create (aText);
  XAlign := 1;
end;

{ TGtkEditorWindow }

{ *** Creation of window *** }

const
  gtk_all = gtk_fill + gtk_expand + gtk_Shrink;
  gtk_NoExp = gtk_fill + gtk_shrink;

constructor TGtkEditorWindow.Create;
begin
  inherited Create (Gtk_Window_TopLevel);
  SetUSize (800, 500);
  MenuFileReopen := nil;
  MenuEditObject := nil;
  MenuEditProperty := nil;
  MenuEditParameter := nil;
  Title := sEditorTitle;
  ciObject := nil;
  ciProperty := nil;
  ciParameter := nil;
  FDefs := nil;
  CreatePixmaps;
  ComposeWindow;
  FReopenList := TStringList.Create;
  ReadSettings;
  ConnectDestroy (@WriteSettings, @FSettings);
end;

destructor TGtkEditorWindow.Destroy;
begin
  FReopenList.Free;
end;

procedure TGtkEditorWindow.ComposeWindow;

var b, b1 : TFPgtkBox;
    t : TFPgtkTable;
    m : TFPgtkMenuBar;
    mlist : TFPgtkItemGroup;
    F : TFPgtkFrame;
    tb : TFPgtkToolbar;
    //pm : TFPgtkPixmap;
    but : TFPgtkButton;
    AG : PGtkAccelGroup;

begin

  AccelGroup := AccelGroupNew;
  AG := AccelGroups[AccelGroup];

  b := TFPgtkVBox.Create;
  Add (b);

  //writeln ('------->> Menu');
  MenuEditObject := NewSubMenu (smEditObject, '', '', nil, []);
  MenuEditProperty := NewSubMenu (smEditProperty, '', '', nil, []);
  MenuEditParameter := NewSubMenu (smEditParameter, '', '', nil, []);
  MenuFileReopen := NewSubMenu (smFileReopen, '', '', nil, []);
  m := NewMenuBar ([
         NewSubMenu (smFile, '',  '', MakeAccelKeyDef(AG,Gdk_F,[amAlt]), [
           NewMenuItem (smFileNew, '', '', MakeAccelKeyDef(AG,Gdk_N,[amControl]), @FileNew, nil),
           NewMenuItem (smFileOpen, '', '', MakeAccelKeyDef(AG,Gdk_L,[amControl]), @FileOpen, nil),
           MenuFileReopen,
           NewMenuItem (smFileSave, '', '', MakeAccelKeyDef(AG,Gdk_S,[amControl]), @FileSave, nil),
           NewMenuItem (smFileSaveAs, '', '', MakeAccelKeyDef(AG,Gdk_A,[amControl]), @FileSaveAs, nil),
           NewLine,
           NewMenuItem (smFileExit, '', '', MakeAccelKeyDef(AG,Gdk_W,[amControl]), @FileExit, nil)]),
         NewSubMenu (smEdit, '', '', MakeAccelKeyDef(AG,Gdk_E,[amAlt]), [
           MenuEditObject, MenuEditProperty, MenuEditParameter]),
         NewSubMenu (smTools, '', '', MakeAccelKeyDef(AG,Gdk_T,[amAlt]), [
           NewMenuItem (smToolsGenerate, '', '', MakeAccelKeyDef(AG,Gdk_G,[amControl]), @ToolsGenerate, nil),
           NewMenuItem (smToolsOptions, '', '', MakeAccelKeyDef(AG,Gdk_O,[amControl]), @ToolsOptions, nil)]),
         NewSubMenu (smHelp, '', '', MakeAccelKeyDef(AG,Gdk_H,[amAlt]), [
           NewMenuItem (smHelpInfo, '', '', MakeAccelKeyDef(AG,Gdk_I,[amControl]), @HelpInfo, nil),
           NewMenuItem (smHelpAbout, '', '', MakeAccelKeyDef(AG,Gdk_B,[amControl]), @HelpAbout, nil)])
         ]);
  b.PackStart (m, false, false, 0);

  //writeln ('------->> Toolbar');

  tb := TFPgtkToolbar.Create;
  b.PackStart (tb, false, false, 0);
  b.Packstart (TFPgtkHSeparator.Create, false, false, 0);

  with tb do
    begin
    ButtonRelief := Gtk_Relief_None;
    AppendSpace;
    AppendItem ('', RemoveUnderscore(smFileNew), '', XPMFileNew, @FileNew, nil);
    AppendSpace;
    AppendItem ('', RemoveUnderscore(smFileOpen), '', XPMFileOpen, @FileOpen, nil);
    but := TFPgtkButton.Create;
    with but do
      begin
      ConnectClicked (@ToolbarReopen, nil);
      Add (TFPgtkArrow.Create(GTK_Arrow_Down, GTK_Shadow_Out));
      CanFocus := False;
      ReliefStyle := GTK_RELIEF_NONE;
      SetUsize (15, 22);
      end;
    AppendWidget (but, RemoveUnderscore(smFileReopen), '');
    AppendSpace;
    AppendItem ('', RemoveUnderscore(smFileSave), '', XPMFileSave, @FileSave, nil);
    AppendSpace;
    AppendItem ('', RemoveUnderscore(smToolsGenerate), '', XPMGenerate, @ToolsGenerate, nil);
    end;

  //writeln ('------->> Panels and lists');

  FDefinition := TFPgtkFrame.Create;
  FDefinition.Text := sComponent;
  b.PackStart (FDefinition, false, true, 0);

  PObject := TFPgtkHPaned.Create;
  b.packStart (PObject, true, true, 0);

  LObjects := TFPgtkScrollCList.Create (1);
  LObjects.SetUsize (120,40);
  b1 := TFPgtkVBox.Create;
  BrObjects := TButtonRow.Create;
  BrObjects.Title := 'Objects buttonrow';
  b1.PackEnd (BrObjects, false, false, 0);
  b1.PackEnd (LObjects);
  PObject.Add1 (b1);

  b := TFPgtkVBox.create;
  PObject.Add2 (b);

  FObject := TFPgtkFrame.Create;
  FObject.Text := SObject;
  b.PackStart (FObject, false, true, 0);

  PProperty := TFPgtkHPaned.Create;
  B.PackStart (PProperty, true, true, 0);

  LProperties := TFpgtkScrollCList.Create (2);
  LProperties.SetUSize (180,30);
  b1 := TFPgtkVBox.Create;
  BrProperties := TButtonRow.Create;
  BrProperties.Title := 'Properties buttonrow';
  b1.PackEnd (BrProperties, false, false, 0);
  b1.PackEnd (LProperties);
  PProperty.Add1 (b1);

  FProperty := TFPgtkFrame.Create;
  FProperty.Text := SProperty;
  PProperty.Add2 (FProperty);

  PProperty.ComputePosition (40, 20, 20);
  PObject.ComputePosition (40, 20, 20);

  //writeln ('------->> Definition');

  t := TFPgtkTable.create (6,1);
  t.ColSpacings := 3;

  t.attach (TFPgtkLabel.Create(sUnitName), 0,1, 0,1, gtk_NoExp, gtk_fill, 3,0);
  DUnitName := TFPgtkEntry.create;
  DUnitName.ConnectChanged (@ChangedDUnitName, nil);
  t.attach (DUnitName, 1,2, 0,1, gtk_all, gtk_fill, 0,0);

  t.attach (TFPgtkLabel.Create (sGtkPrefix), 2,3, 0,1, gtk_NoExp, gtk_fill, 3,0);
  DGtkPrefix := TFPgtkentry.create;
  DGtkPrefix.ConnectChanged (@ChangedDGtkPrefix, nil);
  t.attach (DGtkPrefix, 3,4, 0,1, gtk_all, gtk_fill, 0,0);

  t.attach (TFPgtkLabel.Create(sUsesList), 4,5, 0,1, gtk_NoExp, gtk_fill, 3,0);
  DUsesList := TFPgtkEntry.create;
  DUsesList.ConnectChanged (@ChangedDUsesList, nil);
  t.attach (DUsesList, 5,6, 0,1, gtk_all, gtk_fill, 0,0);

  FDefinition.Add (t);

  //writeln ('------->> Object');

  t := tFPgtkTable.Create (5,3);
  t.ColSpacings := 3;
  FObject.Add (t);

  t.attach (TRightLabel.create(sName), 0,1, 0,1, gtk_noExp, gtk_fill, 3,0);
  OName := TFPgtkentry.create;
  OName.ConnectChanged (@ChangedOName, nil);
  t.attach (OName, 1,2, 0,1);
  t.attach (TRightLabel.create(sInherits), 0,1, 1,2, gtk_noExp, gtk_fill, 3,0);
  OInherit := TFPgtkentry.create;
  OInherit.ConnectChanged (@ChangedOInherit, nil);
  t.attach (OInherit, 1,2, 1,2);
  t.attach (TRightLabel.create(sGtkName), 0,1, 2,3, gtk_noExp, gtk_fill, 3,0);
  OGtkName := TFPgtkentry.create;
  OGtkName.ConnectChanged (@ChangedOGtkName, nil);
  t.attach (OGtkName, 1,2, 2,3);
  t.SetOneColSpacing (1,7);
  OCreateObject := TFPgtkCheckedButton.createWithLabel (sCreateObject);
  OCreateObject.ConnectClicked (@ChangedOCreateObject, nil);
  t.attach (OCreateObject, 3,4, 0,1);
  OWithPointer := TFPgtkCheckedButton.createWithLabel (sWithPointer);
  OWithPointer.ConnectClicked (@ChangedOWithPointer, nil);
  t.attach (OWithPointer, 4,5, 0,1);
  t.attach (TRightLabel.create(sCreateParams), 2,3, 1,2, gtk_noExp, gtk_fill, 3,0);
  OCreateParams := TFPgtkentry.create;
  OCreateParams.ConnectChanged (@ChangedOCreateParams, nil);
  t.attach (OCreateParams, 3,5, 1,2, gtk_all, gtk_fill, 0,0);
  t.attach (TRightLabel.create(sGtkFunctionName), 2,3, 2,3, gtk_noExp, gtk_fill, 3,0);
  OGtkFuncName := TFPgtkentry.create;
  OGtkFuncName.ConnectChanged (@ChangedOGtkFuncName, nil);
  t.attach (OGtkFuncName, 3,5, 2,3, gtk_all, gtk_fill, 0,0);

  //writeln ('------->> Property');

  mlist := TFPgtkItemGroup.Create (TFPgtkMenuItem);
  b := TFPgtkVBox.Create;
  FProperty.Add (b);

  b1 := TFPgtkHBox.Create;
  b.PackStart (b1, false, true, 0);
  b1.PackStart (TFPgtkLabel.Create(sType), false, true, 3);
  mlist.FillFromArray ([sField, sProperty,sFunction,sProcedure,sSignal,sHelperproc,
                        sHelperFunc,sSignalType,sDeclarations,sTypeDecl,sConstructor,
                        sDestructor, sInitialization, sFinalization]);
  PType := TFPgtkOptionMenu.Create;
  with PType do
    begin
    menu := TFPgtkMenu.Create;
    setUsize (70, 26);
    AppendMenuItemGroup (menu, mlist, @ChangedPType, nil);
    end;
  mlist.Clear;
  b1.PackStart (PType, true, true,0);
  bladen := TFPgtkNotebook.Create;
//  bladen.Homogenous := True;
//  bladen.Scrollable := false;
  b.PackStart (bladen, true, true, 0);

  // defintion
  //writeln ('------->> Property Definition');
    t := TFPgtkTable.Create (2, 4);
    t.attach (TRightLabel.Create(sName), 0,1, 0,1, gtk_noExp, gtk_fill, 3,0);
    PName := TFPgtkEntry.create;
    PName.ConnectChanged (@ChangedPName, nil);
    t.attach (PName, 1,2, 0,1);
    t.attach (TRightLabel.Create(sPascalType), 0,1, 1,2, gtk_noExp, gtk_fill, 3,0);
    PPascalType := TFPgtkEntry.create;
    PPascalType.ConnectChanged (@ChangedPPascalType, nil);
    t.attach (PPascalType, 1,2, 1,2);
    t.attach (TRightLabel.Create(sSection), 0,1, 2,3, gtk_noExp, gtk_fill, 3,0);
    PSection := TFPgtkOptionMenu.create;
    mlist.FillFromArray ([sPrivate, sProtected, sPublic, sPublished]);
    with PSection do
      begin
      menu := TFPgtkMenu.Create;
      AppendMenuItemGroup (menu, mlist, @ChangedPSection, nil);
      setUsize (60,26);
      end;
    mlist.Clear;
    t.attach (PSection, 1,2, 2,3);
    t.attach (TRightLabel.Create(sGtkName), 0,1, 3,4, gtk_noExp, gtk_fill, 3,0);
    PGtkName := TFPgtkEntry.create;
    PGtkName.ConnectChanged (@ChangedPGtkName, nil);
    t.attach (PGtkName, 1,2, 3,4);
    b1 := TFPgtkVBox.Create;
    b1.Packstart (t, false, false, 0);
    BDefinition := b1;
    bladen.AppendPage (b1, TFPgtkLabel.Create(sDefinition));

  // parameter
  //writeln ('------->> Property Parameter');
    PParam := TFPgtkHPaned.Create;
    bladen.AppendPage (PParam, TFPgtkLabel.Create(sParameters));
    BParameter := PParam;
    LParams := TFPgtkScrollCList.Create(1);
    LParams.setusize (120,30);
    b1 := TFPgtkVBox.Create;
    BrParams := TButtonRow.Create;
    BrParams.Title := 'Parameters buttonrow';
    b1.PackEnd (BrParams, false, false, 0);
    b1.PackEnd (LParams);
    PParam.Add1 (b1);
    FParam := TFPgtkVBox.Create;
    PParam.Add2 (FParam);
    t := TFPgtkTable.Create (3,4);
    FParam.Packstart (t, false, false, 0);
    t.attach (TRightLabel.Create(sName), 0,1, 0,1, gtk_noExp, gtk_fill, 3,0);
    ParamName := TFPgtkentry.Create;
    ParamName.ConnectChanged (@ChangedParamName, nil);
    t.attach (ParamName, 1,3, 0,1);
    t.attach (TRightLabel.Create(sPascalType), 0,1, 1,2, gtk_noExp, gtk_fill, 3,0);
    ParamPascalType := TFPgtkentry.Create;
    ParamPascalType.ConnectChanged (@ChangedParamPascalType, nil);
    t.attach (ParamPascalType, 1,3, 1,2);
    t.attach (TRightLabel.Create(sType), 0,1, 2,3, gtk_noExp, gtk_fill, 3,0);
    ParamType := TFPgtkOptionmenu.Create;
    t.attach (ParamType, 1,3, 2,3);
    with ParamType do
      begin
      Menu := TFPgtkMenu.Create;
      mList.FillFromArray ([sNothing, sVar, sConst]);
      AppendMenuItemGroup (menu, mlist, @ChangedParamType, nil);
      setusize (50, 26);
      end;
    mlist.Clear;
    ParamConvert := TFPgtkCheckedButton.CreateWithLabel(sConvert);
    ParamConvert.ConnectClicked (@ChangedParamConvert, nil);
    t.attach (ParamConvert, 1,2, 3,4, gtk_noExp, gtk_Fill, 0,0);

  // Read
  //writeln ('------->> Property Read');
    t := TFPgtkTable.Create (3,3);
    bladen.AppendPage (t, TFPgtkLabel.Create(sRead));
    BRead := t;
    t.Attach (TRightLabel.Create(sType), 0,1, 0,1, gtk_noExp, gtk_fill, 3,0);
    PRtype := TFPgtkOptionMenu.Create;
    with PRType do
      begin
      Menu := TFPgtkMenu.Create;
      mlist.FillFromArray ([sGtkFunction, sObjectField, sObjectFunction,SField, sFunction,
                        sNotImplemented, sGtkMacro, sExistingFunc]);
      AppendMenuItemGroup (Menu, mlist, @ChangedPRType, nil);
      SetUsize (60,26);
      end;
    mlist.Clear;
    t.attach (PRtype, 1,2, 0,1, gtk_all, gtk_noExp, 0,0);
    PRConvert := TFPgtkCheckedButton.CreateWithLabel (sConvert);
    PRConvert.ConnectClicked (@ChangedPRConvert, nil);
    PRConvert.TheLabel.XAlign := 0.0;
    t.attach (PRConvert, 2,3, 0,1, gtk_all, gtk_noExp, 0,0);
    t.Attach (TRightLabel.Create(sGtkName), 0,1, 1,2, gtk_noExp, gtk_fill, 3,0);
    PRGtkName := TFPgtkEntry.Create;
    PRGtkName.ConnectChanged (@ChangedPGtkName, nil);
    t.attach (PRGtkName, 1,3, 1,2, gtk_all, gtk_noExp, 0,0);
    t.Attach (TRightLabel.Create(sCode), 0,1, 2,3, gtk_noExp, gtk_fill, 3,0);
    PRCode := TFPgtkScrollText.Create;
    PRCode.TheText.ConnectChanged (@ChangedPCode, nil);
    t.attach (PRCode, 1,3, 2,3);

  // Write
  //writeln ('------->> Property Write');
    t := TFPgtkTable.Create (3,3);
    BWrite := t;
    bladen.AppendPage (t, TFPgtkLabel.Create(sWrite));
    t.Attach (TRightLabel.Create(sType), 0,1, 0,1, gtk_noExp, gtk_fill, 3,0);
    PWtype := TFPgtkOptionMenu.Create;
    with PWType do
      begin
      Menu := TFPgtkMenu.Create;
      mlist.FillFromArray ([sGtkProcedure, sObjectField, sObjectProcedure,SField, sProcedure,
                          sNotImplemented, sGtkMacro, sExistingProc]);
      AppendMenuItemGroup (Menu, mlist, @ChangedPWType, nil);
      SetUsize (60,26);
      end;
    mlist.Clear;
    t.attach (PWtype, 1,2, 0,1, gtk_all, gtk_noExp, 0,0);
    PWConvert := TFPgtkCheckedButton.CreateWithLabel (sConvert);
    PWConvert.ConnectClicked (@ChangedPWConvert, nil);
    PWConvert.TheLabel.XAlign := 0.0;
    t.attach (PWConvert, 2,3, 0,1, gtk_all, gtk_noExp, 0,0);
    t.Attach (TRightLabel.Create(sGtkName), 0,1, 1,2, gtk_noExp, gtk_fill, 3,0);
    PWGtkName := TFPgtkEntry.Create;
    PWGtkName.ConnectChanged (@ChangedPWGtkName, nil);
    t.attach (PWGtkName, 1,3, 1,2, gtk_all, gtk_noExp, 0,0);
    t.Attach (TRightLabel.Create(sCode), 0,1, 2,3, gtk_noExp, gtk_fill, 3,0);
    PWCode := TFPgtkScrollText.Create;
    PWCode.TheText.ConnectChanged (@ChangedPWCode, nil);
    t.attach (PWCode, 1,3, 2,3);

  // Function
  //writeln ('------->> Property Function');
    t := TFPgtkTable.Create (2,2);
    BFunction := t;
    bladen.AppendPage (t, TFPgtkLabel.Create(sFunction));

    t.Attach (TFPgtkLabel.Create(sCode), 1,2, 0,1, gtk_NoExp, gtk_NoExp, 7,0);
    PFCode := TFPgtkScrollText.Create;
    PFCode.TheText.ConnectChanged (@ChangedPCode, nil);
    t.Attach (PFCode, 1,2, 1,2, gtk_all, gtk_all, 0,0);
    f := TFPgtkFrame.Create;
    f.Border := 3;
    f.Text := sTypes;
    t.Attach (f, 0,1, 0,2, gtk_NoExp, gtk_NoExp, 0,0);

    b1 := TFPgtkVBox.Create;
    f.Add (b1);
    b1.border := 2;
    POverride := TFPgtkCheckedButton.CreateWithLabel (sOverride);
    POverride.ConnectClicked (@ChangedPFuncType, inttopointer(0));
    b1.PackStart (POverride, false, false, 0);
    PVirtual := TFPgtkCheckedButton.CreateWithLabel (sVirtual);
    PVirtual.ConnectClicked (@ChangedPFuncType, inttopointer(1));
    b1.PackStart (PVirtual, false, false, 0);
    PDynamic := TFPgtkCheckedButton.CreateWithLabel (sDynamic);
    PDynamic.ConnectClicked (@ChangedPFuncType, inttopointer(2));
    b1.PackStart (PDynamic, false, false, 0);
    PAbstract := TFPgtkCheckedButton.CreateWithLabel (sAbstract);
    PAbstract.ConnectClicked (@ChangedPFuncType, inttopointer(3));
    b1.PackStart (PAbstract, false, false, 0);
    PCDecl := TFPgtkCheckedButton.CreateWithLabel (sCDecl);
    PCDecl.ConnectClicked (@ChangedPFuncType, inttopointer(4));
    b1.PackStart (PCDecl, false, false, 0);
    POverload := TFPgtkCheckedButton.CreateWithLabel (sOverload);
    POverload.ConnectClicked (@ChangedPFuncType, inttopointer(5));
    b1.PackStart (POverload, false, false, 0);
    PReintroduce := TFPgtkCheckedButton.CreateWithLabel (sReintroduce);
    PReintroduce.ConnectClicked (@ChangedPFuncType, inttopointer(6));
    b1.PackStart (PReintroduce, false, false, 0);

  // Code
  //writeln ('------->> Property Code');
    PCode := TFPgtkScrollText.Create;
    BCode := PCode;
    PCode.TheText.ConnectChanged (@ChangedPCode, nil);
    bladen.AppendPage (PCode, TFPgtkLabel.Create(sCode));

  // Configuring buttonrows
  //writeln ('------->> Configure Buttonrows');
  BrParams.Configure (LParams.CList, nil, @RefreshParam,
                      MenuEditParameter.SubMenu,
                      AG, Gdk_Mod1_Mask+Gdk_Shift_Mask);
  BrProperties.Configure (LProperties.Clist, @PropertyIcon, @RefreshProperty,
                          MenuEditProperty.SubMenu,
                          AG, Gdk_Control_Mask+Gdk_Shift_Mask);
  BrObjects.Configure (LObjects.CList, nil, @RefreshObject,
                       MenuEditObject.SubMenu, AG, 0);

  //writeln ('------->> Einde ComposeWindow');

end;

{ *** Procedures to show parts in the window (when selecting items) *** }

{ Showing procedures }

procedure TGtkEditorWindow.RefreshDefinition;
begin
  RefreshingDefinition := True;
  try
    if assigned (FDefs) then
      with FDefs do
        begin
        DUnitName.text := UnitName;
        DGtkPrefix.Text := GtkPrefix;
        DUsesList.Text := UsesList;
        BrObjects.ChangeCollection (Definition);
        end
    else
      begin
      DUnitName.text := '';
      DGtkPrefix.Text := '';
      DUsesList.Text := '';
      BrObjects.ChangeCollection (nil);
      end;
  finally
    RefreshingDefinition := False;
  end;
end;

procedure TGtkEditorWindow.RefreshObject (Selected : TCollectionItem; NeedFocus:boolean);
begin
  RefreshingObject := True;
  try
    ciObject := TObjectItem(Selected);
    if assigned (ciObject) then
      with ciObject do
        begin
        FObject.Sensitive := true;
        OName.Text := Name;
        OInherit.Text := Inherit;
        OGtkName.Text := GtkName;
        OGtkFuncName.Text := GtkFuncName;
        OCreateParams.Text := CreateParams;
        OCreateObject.Active := CreateObject;
        OWithPointer.Active := WithPointer;
        BrProperties.ChangeCollection (Props);
        end
    else
      begin
      FObject.Sensitive := false;
      OName.Clear;
      OInherit.Clear;
      OGtkName.Clear;
      OGtkFuncName.Clear;
      OCreateParams.Clear;
      OCreateObject.Active := False;
      OWithPointer.Active := False;
      BrProperties.ChangeCollection (nil);
      end;
    if NeedFocus then
      with OName do
        begin
        SelectRegion (0, -1);
        GrabFocus;
        end;
  finally
    RefreshingObject := false;
  end;
end;

procedure TGtkEditorWindow.CreatePixmaps;

  procedure GdkPixmap (Data : array of string; var pm : PGdkPixmap; var bm : PGdkBitmap);
  var ppdata : ppgchar;
  begin
    ppdata := ArrayToPPgchar(Data);
    pm := gdk_pixmap_colormap_create_from_xpm_d (nil, Colormap, @bm, nil, ppdata);
    freemem (ppdata, sizeof (pchar) * (high(data)-low(data)+1));
  end;

begin
  GdkPixmap (XPMPropField, PropPixs[0], PropMasks[0]);
  GdkPixmap (XPMPropProperty, PropPixs[1], PropMasks[1]);
  GdkPixmap (XPMPropFunction, PropPixs[2], PropMasks[2]);
  GdkPixmap (XPMPropProcedure, PropPixs[3], PropMasks[3]);
  GdkPixmap (XPMPropSignal, PropPixs[4], PropMasks[4]);
  GdkPixmap (XPMPropHelperProc, PropPixs[5], PropMasks[5]);
  GdkPixmap (XPMPropHelperFunc, PropPixs[6], PropMasks[6]);
  GdkPixmap (XPMPropSignalType, PropPixs[7], PropMasks[7]);
  GdkPixmap (XPMPropDeclar, PropPixs[8], PropMasks[8]);
  GdkPixmap (XPMPropTypeDecl, PropPixs[9], PropMasks[9]);
  GdkPixmap (XPMPropConstr, PropPixs[10], PropMasks[10]);
  GdkPixmap (XPMPropDestr, PropPixs[11], PropMasks[11]);
  GdkPixmap (XPMPropInitial, PropPixs[12], PropMasks[12]);
  GdkPixmap (XPMPropFinal, PropPixs[13], PropMasks[13]);
end;

procedure TGtkEditorWindow.PropertyIcon (Item:TCollectionItem; var Pixmap:PGdkPixMap; var mask:PGdkBitmap);
var r : integer;
begin
  r := ord((Item as TPropertyItem).propType);
  Pixmap := PropPixs[r];
  Mask := PropMasks[r];
end;

procedure TGtkEditorWindow.EnDisablePages (pt : TPropType);
begin
  BDefinition.Visible := pt in [ptField,ptProperty,ptFunction,ptProcedure,ptSignal,
               ptHelperProc,ptHelperFunc,ptSignalType,ptDeclarations,ptTypeDecl,
               ptConstructor,ptDestructor];
  BParameter.visible := pt in [ptProperty,ptFunction,ptProcedure,
               ptHelperProc,ptHelperFunc,ptSignalType,ptConstructor,ptDestructor];
  BFunction.Visible := pt in [ptFunction,ptProcedure,ptHelperProc,ptHelperFunc,
               ptConstructor,ptDestructor];
  BRead.Visible := pt in [ptProperty];
  BWrite.Visible := pt in [ptProperty];
  BCode.visible := pt in [ptDeclarations,ptTypeDecl,ptInitialization,ptFinalization];
end;

procedure TGtkEditorWindow.RefreshProperty (Selected : TCollectionItem; NeedFocus:boolean);
var r : byte;
    s : string;
begin
  RefreshingProperty := True;
  try
    ciProperty := selected as TPropertyItem;
    if assigned (ciProperty) then
      with ciProperty do
        begin
        s := Code.Text;
        FProperty.Sensitive := true;
        r := ord (PropType);
        PType.SetHistory (r);
        EnDisablePages (PropType);
        // definition
        PName.Text := Name;
        PPascalType.Text := PascalType;
        PGtkName.Text := GtkName;
        PSection.SetHistory (Ord(Section));
        // read
        PRType.SetHistory (Ord(ReadFuncType));
        PRGtkName.Text := GtkName;
        PRCode.Text := s;
        PRConvert.Active := ReadConvert;
        // write
        PWType.SetHistory (Ord(WriteProcType));
        PWGtkName.Text := WriteGtkName;
        PWCode.Text := WriteCode.Text;
        PWConvert.Active := WriteConvert;
        // function
        POverride.active := ptOverride in ProcTypes;
        PVirtual.active := ptVirtual in ProcTypes;
        PDynamic.active := ptDynamic in ProcTypes;
        PAbstract.active := ptAbstract in ProcTypes;
        PCDecl.active := ptCDecl in ProcTypes;
        POverload.active := ptOverload in ProcTypes;
        PReintroduce.active := ptReintroduce in ProcTypes;
        PFCode.Text := s;
        // Code
        PCode.Text := s;
        // Parameters
        BrParams.ChangeCollection (Parameters);
        end
    else
      begin
      FProperty.Sensitive := false;
      PType.Clear;
      // definition
      PName.Clear;
      PPascalType.Clear;
      PGtkName.Clear;
      PSection.Clear;
      // read
      PRType.Clear;
      PRGtkName.Clear;
      PRCode.Clear;
      PRConvert.Active := false;
      // write
      PWType.Clear;
      PWGtkName.Clear;
      PWCode.Clear;
      PWConvert.Active := False;
      // function
      POverride.active := False;
      PVirtual.active := False;
      PDynamic.active := False;
      PAbstract.active := False;
      PCDecl.active := False;
      POverload.active := False;
      PReintroduce.active := False;
      PFCode.Clear;
      // Code
      PCode.Clear;
      // Parameters
      BrParams.ChangeCollection (nil);
      end;
  finally
    RefreshingProperty := false;
  end;
  if NeedFocus then
    with PName do
      begin
      Bladen.PageIndex := 0;
      SelectRegion (0, -1);
      GrabFocus;
      end;
end;

procedure TGtkEditorWindow.RefreshParam (Selected : TCollectionItem; NeedFocus:boolean);
begin
  RefreshingParam := True;
  try
    ciParameter := Selected as TParameterItem;
    if assigned (ciParameter) then
      with ciParameter do
        begin
        FParam.Sensitive := true;
        ParamName.text := name;
        ParamPascalType.text := PascalType;
        self.ParamType.SetHistory (Ord(ParamType));
        ParamConvert.Active := Convert;
        end
    else
      begin
      FParam.Sensitive := false;
      ParamName.Clear;
      ParamPascalType.Clear;
      self.ParamType.Clear;
      ParamConvert.Active := False;
      end;
    if NeedFocus then
      with ParamName do
        begin
        selectRegion (0, -1);
        GrabFocus;
        end;
  finally
    RefreshingParam := False;
  end;
end;

procedure TGtkEditorWindow.ObjectDisplayChanged;
begin
  with BrObjects do
    if (SelectedRow >= 0) and assigned(ciObject) then
      LObjects.Clist.CellText[SelectedRow,0] := ciObject.DisplayName;
end;

procedure TGtkEditorWindow.PropertyDisplayChanged;
var r : integer;
begin
  with BrProperties do
    if (SelectedRow >= 0) and assigned(ciProperty) then
      begin
      LProperties.CList.CellText[SelectedRow,1] := ciProperty.DisplayName;
      r := ord(ciProperty.PropType);
      LProperties.CList.SetPixMap (SelectedRow, 0, PropPixs[r], PropMasks[r]);
      end;
end;

procedure TGtkEditorWindow.ParamDisplayChanged;
begin
  with BrParams do
    if (SelectedRow >= 0) and assigned(ciParameter) then
      LParams.CList.CellText[SelectedRow,0] := ciParameter.DisplayName;
end;

{ entry saving procedures/signals }

// Definition

procedure TGtkEditorWindow.ChangedDUsesList (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingDefinition then Exit;
  if assigned (FDefs) then
    FDefs.UsesList := DUsesList.Text;
end;

procedure TGtkEditorWindow.ChangedDUnitName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingDefinition then Exit;
  if assigned (FDefs) then
    FDefs.UnitName := DUnitName.Text;
end;

procedure TGtkEditorWindow.ChangedDGtkPrefix (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingDefinition then Exit;
  if assigned (FDefs) then
    FDefs.GtkPrefix := DGtkPrefix.Text;
end;

// Object

procedure TGtkEditorWindow.ChangedOName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    begin
    ciObject.Name := OName.Text;
    ObjectDisplayChanged;
    end;
end;

procedure TGtkEditorWindow.ChangedOInherit (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    ciObject.Inherit := OInherit.Text;
end;

procedure TGtkEditorWindow.ChangedOGtkName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    ciObject.GtkName := OGtkName.Text;
end;

procedure TGtkEditorWindow.ChangedOGtkFuncName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    ciObject.GtkFuncName := OGtkFuncName.Text;
end;

procedure TGtkEditorWindow.ChangedOCreateParams (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    ciObject.CreateParams := OCreateParams.Text;
end;

procedure TGtkEditorWindow.ChangedOCreateObject (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    ciObject.CreateObject := OCreateObject.active;
end;

procedure TGtkEditorWindow.ChangedOWithPointer (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingObject then Exit;
  if assigned(ciObject) then
    ciObject.WithPointer := OWithpointer.active;
end;

// Property

procedure TGtkEditorWindow.ChangedPType (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    begin
    ciProperty.PropType := TPropType(Pointertoint(data));
    PropertyDisplayChanged;
    EnDisablePages (ciProperty.PropType);
    end;
end;

procedure TGtkEditorWindow.ChangedPName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    begin
    ciProperty.Name := PName.Text;
    PropertyDisplayChanged;
    end;
end;

procedure TGtkEditorWindow.ChangedPPascalType (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    ciProperty.PascalType := PPascalType.Text;
end;

procedure TGtkEditorWindow.ChangedPGtkName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    ciProperty.GtkName := (Sender as TFpgtkEntry).Text;
end;

procedure TGtkEditorWindow.ChangedPSection (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    begin
    ciProperty.Section := TInterfaceSection(Pointertoint(data));
    PropertyDisplayChanged;
    end;
end;

procedure TGtkEditorWindow.ChangedPCode (Sender:TFPgtkObject; data:pointer);
var s : string;
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    s := (Sender as TFPgtkText).Text;
  ciProperty.Code.Text := s;
end;

// read

procedure TGtkEditorWindow.ChangedPRType (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    ciProperty.ReadfuncType := TPropFuncType(Pointertoint(data));
end;

procedure TGtkEditorWindow.ChangedPRConvert (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    ciProperty.ReadConvert := PRConvert.Active;
end;

// write

procedure TGtkEditorWindow.ChangedPWType (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    begin
    ciProperty.WriteProcType := TPropFuncType(Pointertoint(data));
    end;
end;

procedure TGtkEditorWindow.ChangedPWGtkName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    ciProperty.WriteGtkName := PWGtkName.Text;
end;

procedure TGtkEditorWindow.ChangedPWCode (Sender:TFPgtkObject; data:pointer);
var s : string;
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    begin
    s := PWCode.Text;
    ciProperty.WriteCode.Text := s;
    end;
end;

procedure TGtkEditorWindow.ChangedPWConvert (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    ciProperty.WriteConvert := PWConvert.active;
end;

// function

procedure TGtkEditorWindow.ChangedPFuncType (Sender:TFPgtkObject; data:pointer);
var pt : TProcType;
begin
  if RefreshingProperty then Exit;
  if assigned(ciProperty) then
    with ciProperty do
      begin
      pt := TProcType(pointertoint(data));
      if (Sender as TFPgtkToggleButton).Active then
        ProcTypes := Proctypes + [pt]
      else
        ProcTypes := Proctypes - [pt];
      end;
end;

// parameter

procedure TGtkEditorWindow.ChangedParamName (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingParam then Exit;
  if assigned (ciParameter) then
    begin
    ciParameter.Name := ParamName.Text;
    ParamDisplayChanged;
    end;
end;

procedure TGtkEditorWindow.ChangedParamPascalType (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingParam then Exit;
  if assigned (ciParameter) then
    ciParameter.PascalType := ParamPascalType.Text;
end;

procedure TGtkEditorWindow.ChangedParamType (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingParam then Exit;
  if assigned (ciParameter) then
    ciParameter.ParamType := TParamType (Pointertoint(data));
end;

procedure TGtkEditorWindow.ChangedParamConvert (Sender:TFPgtkObject; data:pointer);
begin
  if RefreshingParam then Exit;
  if assigned (ciParameter) then
    ciParameter.Convert := ParamConvert.Active;
end;

{ *** Menu procedures *** }

procedure TGtkEditorWindow.BuildReopenList;
var r : integer;
    mi : TFPgtkMenuItem;
begin
  with FReopenList do
    begin
    while (count > FSettings.MRUCount) do
      Delete (0);
    with MenuFileReopen do
      begin
      if assigned(SubMenu) then
        SubMenu.Free;
      SubMenu := TFPgtkMenu.Create;
      with (submenu as TFPgtkMenu) do
        for r := FReopenList.count-1 downto 0 do
          begin
          mi := NewMenuItem (FReopenList[r], '', '', nil, @FileReopen, mi);
          Append (mi);
          end;
      end;
    end;
end;

procedure TGtkEditorWindow.NewFilename (NewName : string);
var r : integer;
begin
  if NewName = '' then
    Title := sEditorTitle
  else
    Title := sEditorTitle + ' - ' + NewName;
  FFilename := NewName;
  with FReopenList do
    begin
    r := IndexOf (NewName);
    if r >= 0 then
      Delete (r);
    Add (NewName);
    BuildReopenList;
    end;
end;

procedure TGtkEditorWindow.DataWrite (filename : string);
var
  BinStream : TMemoryStream;
  StrStream : TFileStream;
  l : TStrings;
begin
  StrStream := TFileStream.Create(filename, fmCreate);
  try
    case FSettings.FileFormat of
    ffComponentText :
      begin
      BinStream := TMemoryStream.Create;
      try
        BinStream.WriteComponent(FDefs);
        BinStream.Seek(0, soFromBeginning);
        ObjectBinaryToText(BinStream, StrStream);
      finally
        BinStream.Free
      end;
      end;
    ffComonentBin :
      StrStream.WriteComponent(FDefs);
    ffHomeText :
      begin
      l := TStringList.Create;
      try
        l.capacity := FDefs.definition.count * 5;
        FDefs.Save (l);
        l.SaveToStream (StrStream);
      finally
        l.Free;
      end;
      end;
    end;
    HasAFile := True;
    NewFilename (filename);
  finally
    StrStream.Free;
  end;
end;

procedure TGtkEditorWindow.DataRead (filename : string);
var FStream : TFileStream;
    MStream : TMemoryStream;
    s : string[6];
    l : TStrings;
begin
  if fileExists (filename) then
    begin
    FStream := TFileStream.Create(filename, fmOpenRead);
    try
      FStream.Readbuffer (s[1],6);
      s[0] := #6;
      FStream.Seek (0, soFromBeginning);
      if not assigned (FDefs) then
        FDefs := TObjectDefs.Create (nil);
      if s = 'object' then
        begin
        MStream := TMemoryStream.Create;
        try
          ObjectTextToBinary(FStream, MStream);
          MStream.Seek(0, soFromBeginning);
          MStream.ReadComponent(FDefs);
        finally
          MStream.Free;
        end;
        end
      else if s = 'defini' then
        begin
        l := TStringList.Create;
        try
          l.LoadFromStream (FStream);
          FDefs.Load (l);
        finally
          l.Free;
        end;
        end
      else
        FStream.ReadComponent (FDefs);
    finally
      FStream.Free;
    end;
    HasAFile := True;
    newFileName (filename);
    end
  else
    ShowMessage ('Error', 'Can''t find file "'+filename+'"');
end;

procedure TGtkEditorWindow.Generate;
var TheFile : TStringList;
    Prog : TProgressWindow;
begin
  TheFile := TStringList.Create;
  try
    TheFile.beginUpdate;
    if FSettings.ShowProgress then
      begin
      Prog := TProgressWindow.Create;
      try
        Prog.Show;
        FDefs.Write (TheFile, @(Prog.StepIt), @(Prog.SetMax));
        Prog.Hide;
      finally
        Prog.Free;
      end;
      end
    else
      FDefs.Write (TheFile, nil, nil);
    TheFile.EndUpdate;
    Thefile.SaveToFile (FDefs.UnitName+'.'+FSettings.Extention);
  finally
    TheFile.Free;
  end;
end;

{ Menu signals }
procedure TGtkEditorWindow.FileNew (Sender : TFPgtkObject; data : pointer);
begin
  FDefs.Free;
  FDefs := TObjectDefs.Create (nil);
  FFileName := '';
  RefreshDefinition;
end;

procedure TGtkEditorWindow.DialogSetFilename (Sender:TFPgtkWindow;
              aDialogResult:pointer; Action:integer; initiator:TFPgtkObject);
begin
  if Action = drOk then
    FFilename := (Sender as TFPgtkFileSelection).Filename;
end;

procedure TGtkEditorWindow.FileOpen (Sender : TFPgtkObject; data : pointer);
var fs : TFPgtkFileSelection;
begin
  fs := TFPgtkFileSelection.Create (gtk_window_dialog);
  with fs do
    begin
    Title := 'Select file to open';
    OKButton.ConnectClicked (@(fs.CloseWithResult), inttopointer(drOk));
    CancelButton.ConnectClicked (@(fs.CloseWindow), nil);
    Filename := FFilename;
    if execute (nil, nil, @DialogSetFilename) = drOk then
      begin
      DataRead (FFilename);
      RefreshDefinition;
      end;
    end;
end;

procedure TGtkEditorWindow.FileReopen (Sender : TFPgtkObject; data : pointer);
begin
  with (Sender as TFPgtkMenuItem) do
    begin
    DataRead (Text);
    RefreshDefinition;
    end;
end;

procedure TGtkEditorWindow.ToolbarReopen (Sender : TFpgtkObject; data : pointer);
begin
  (MenuFileReopen.submenu as TFPgtkMenu).Popup (0);
end;

procedure TGtkEditorWindow.FileSave (Sender : TFPgtkObject; data : pointer);
begin
  if FFilename = '' then
    FileSaveAs (Sender, data)
  else
    DataWrite (FFilename);
end;

procedure TGtkEditorWindow.FileSaveAs (Sender : TFPgtkObject; data : pointer);
var fs : TFPgtkFileSelection;
begin
  fs := TFPgtkFileSelection.Create (gtk_window_dialog);
  with fs do
    begin
    Title := 'Give filename to save';
    OKButton.ConnectClicked (@(fs.CloseWithResult), inttopointer(drOk));
    CancelButton.ConnectClicked (@(fs.CloseWindow), nil);
    Filename := FFilename;
    if execute (nil, nil, @DialogSetFilename) = drOk then
      DataWrite (FFilename);
    end;
end;

procedure TGtkEditorWindow.FileExit (Sender : TFPgtkObject; data : pointer);
begin
  If FSettings.SaveOnClose then
    FileSave (Sender, data);
  Close;
end;

procedure TGtkEditorWindow.ToolsGenerate (Sender : TFPgtkObject; data : pointer);
begin
  Generate;
end;

procedure TGtkEditorWindow.ToolsOptions (Sender : TFPgtkObject; data : pointer);
begin
  with TSettingsDialog.Create do
    Execute (nil, @FSettings, nil);
end;

procedure TGtkEditorWindow.HelpInfo (Sender : TFPgtkObject; data : pointer);
begin
  ShowMessage (sInfoTitle, sInfoMessage);
end;

procedure TGtkEditorWindow.HelpAbout (Sender : TFPgtkObject; data : pointer);
var d : TFPgtkDialog;
    b : TFPgtkButton;
    box : TFPgtkBox;
    AG : integer;
begin
  d := TFPgtkDialog.Create;
  with d do
    begin
    title := sAboutTitle;
    box := TFPgtkVBox.Create;
    with Box do
      begin
      border := 15;
      PackStart (TFPgtkLabel.Create (sAbout1));
      PackStart (TFPgtkLabel.Create (sAbout2));
      PackStart (TFPgtkHSeparator.Create, false, false, 8);
      PackStart (TFPgtkLabel.Create (sAboutVersion));
      PackStart (TFPgtkLabel.Create (sAboutDesigner));
      end;
    vbox.Packstart (box, true, false, 0);
    with ActionArea do
      begin
      b := TFPgtkButton.CreateWithLabel (sOk);
      b.ConnectClicked (@(d.CloseWindow), nil);
      Packstart (b);
      end;
    AG := AccelGroupNew;
    AcceleratorAdd (AG, b, sgClicked, gdk_Return, 0, TGTKAccelFlags(0));
    AcceleratorAdd (AG, b, sgClicked, gdk_Cancel, 0, TGTKAccelFlags(0));
    Execute (nil, nil, nil);
    end;
end;

const
  secSettings = 'Settings';
  keySaveOnExit = 'SaveOnExit';
  keyFileFormat = 'TextFormat';
  keyMRUCount = 'MRUCount';
  keyExtention = 'Extention';
  keyProgressWindow = 'ShowProgress';
  secMRU = 'Last open files';
  keyFile = 'File';
  keyCount = 'Count';

procedure TGtkEditorWindow.ReadSettings;
var c, r : integer;
    s : string;
begin
  with FSettings do
  with TMemInifile.Create(ChangeFileExt(paramstr(0), '.ini')) do
    try
      saveonclose := readbool (SecSettings, keySaveOnExit, true);
      Fileformat := TFileFormat(readinteger (secSettings, keyFileFormat, 2));
      Extention := readstring (secSettings, keyExtention, '.pp');
      MRUCount := readinteger (secSettings, keyMRUCount, 5);
      ShowProgress := readbool (SecSettings, keyProgressWindow, true);
      FReopenList.capacity := MRUCount;
      c := ReadInteger (secMRU, keyCount, 0);
      for r := 0 to c-1 do
        begin
        s := readstring(secMRU, keyFile+inttostr(r), '');
        if s <> '' then
          FReopenlist.Add (s);
        end;
      BuildReopenList;
    finally
      free;
    end;
end;

procedure TGtkEditorWindow.WriteSettings (Sender : TFPgtkObject; data : pointer);
var r : integer;
begin
  with PSettingsRec(data)^, TMemInifile.Create (ChangeFileExt(paramstr(0), '.ini')) do
    try
      writebool (SecSettings, keySaveOnExit, saveonclose);
      writeinteger (secSettings, keyFileFormat, Ord(FileFormat));
      writestring (secSettings, keyExtention, Extention);
      writeinteger (secSettings, keyMRUCount, MRUCount);
      writebool (SecSettings, keyProgressWindow, ShowProgress);
      writeinteger (secMRU, keyCount, FReopenlist.count);
      UpdateFile;
      for r := 0 to FReopenList.count-1 do
        writestring (secMRU, keyFile+inttostr(r), FReopenList[r]);
    finally
      free;
    end;
end;

end.
