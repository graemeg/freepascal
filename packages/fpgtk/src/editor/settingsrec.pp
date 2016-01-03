{$mode objfpc}{$h+}
unit SettingsRec;

{$define UseLog}

interface

uses FPgtk, FPgtkExt;

type

  TFileFormat = (ffComonentBin, ffComponentText, ffHomeText);

  PSettingsRec = ^TSettingsRec;
  TSettingsRec = record
    SaveOnClose : boolean;
    FileFormat : TFileFormat;
    Extention : string;
    MRUCount : integer;
    ShowProgress : boolean;
  end;

  TSettingsDialog = class (TFPgtkDialog)
  private
    FCBSaveOnClose : TFPgtkToggleButton;
    FEFileFormat : TFPgtkOptionMenu;
    FMenuFileFormat : TFPgtkMenu;
    FEExtention : TFPgtkCombo;
    FEMRUCount : TFPgtkSpinButton;
    FCBProgressWindow : TFPgtkToggleButton;
    procedure BuildDialog;
  protected
    procedure DoDialogResult (Action:integer; Sender:TFPgtkObject); override;
    procedure DoDialogInit (InitData:pointer); override;
  public
    Constructor Create;
  end;

procedure Log (s : string); overload;
procedure Log (fmt : string; params : array of const); overload;
procedure Log (indent:integer; s:string);
procedure Log (indent:integer; fmt:string; params:array of const);

implementation

uses GtkDefTexts, gdk, gtk, sysutils;

constructor TSettingsDialog.Create;
begin
  inherited;
  Title := sOptions;
  BuildDialog;
end;

procedure TSettingsDialog.BuildDialog;
var but : TFPgtkButton;
    b, box : TFPgtkBox;
    AG : integer;
    AGroup : PGtkAccelGroup;
begin
  // Action Area
  AG := AccelGroupNew;
  Agroup := AccelGroups[AG];

  Box := ActionArea;

  but := TFPgtkButton.CreateWithLabel (sOk, AGroup);
  box.PackEnd (but, false, false, 0);
  with but do
    begin
    Candefault := true;
    ConnectClicked (@CloseWithResult, inttopointer(drOk));
    GrabDefault;
    end;
  AcceleratorAdd (AG, but, sgClicked, GDK_Return, 0, GTK_ACCEL_VISIBLE);

  but := TFPgtkButton.CreateWithLabel (sCancel, AGroup);
  box.PackEnd (but, false, false, 0);
  with but do
    begin
    CanDefault := true;
    ConnectClicked (@CloseWithResult, inttopointer(drCancel));
    end;
  AcceleratorAdd (AG, but, sgClicked, GDK_Escape, 0, gtk_accel_visible);

  // Setting controls
  box := vbox;
  border := 15;

  FEFileFormat := TFPgtkOptionMenu.Create;
  FMenuFileFormat := NewMenu ('', [
         NewMenuItem (sComponentBin, sHintCompBin, '', nil, nil, nil),
         NewMenuItem (sComponentText, sHintCompText, '', nil, nil, nil),
         NewMenuItem (sHomeText, sHintHomeText, '', nil, nil, nil)
         ]);
  FEFileFormat.Menu := FMenuFileFormat;
  b := TFPgtkHBox.Create;
  b.PackStart (TFPgtkLabel.Create (sFileFormat), false, false, 0);
  b.PackStart (FEFileFormat, False, False, 10);
  box.PackStart (b, false, false, 0);

  b := TFPgtkHBox.Create;
  box.PackStart (b, true, true, 0);

  FCBSaveOnClose := TFPgtkCheckedButton.CreateWithLabel(sSaveonclose,AGroup);
  b.PackStart (FCBSaveOnClose, False, False, 10);

  FCBProgressWindow := TFPgtkCheckedButton.CreateWithLabel(sProgressWindow, AGroup);
  b.PackStart (FCBProgressWindow, False, False, 10);

  FEExtention := TFPgtkCombo.Create;
  with FEExtention do
    begin
    SetValueInList (false, false);
    List.Add (TFPgtkListItem.CreateWithLabel('pp'));
    List.Add (TFPgtkListItem.CreateWithLabel('pas'));
    end;
  b := TFPgtkHBox.Create;
  b.PackStart (TFPgtkLabel.Create (sExtention), false, false, 0);
  b.PackStart (FEExtention, false, false, 5);
  box.PackStart (b, false, false, 10);

  FEMRUCount := TFPgtkSpinButton.Create;
  with FEMRUCount do
    begin
    Configure (nil, 1.0, 0);
    SnapToTicks := True;
    Numeric := True;
    Wrap := False;
    Adjustment.configure (1.0, 10.0, 5.0, 1.0, 3.0, 1.0);
    end;
  b := TFPgtkHBox.Create;
  b.PackStart (TFPgtkLabel.Create (sMRUcount), false, false, 0);
  b.PackStart (FEMRUCount, false, false, 5);
  box.PackStart (b, false, false, 10);

end;

procedure TSettingsDialog.DoDialogResult (Action:integer; Sender:TFPgtkObject);
begin
  if Action = drOk then
    with PSettingsRec(DialogResult)^ do
      begin
      SaveOnClose := FCBSaveOnClose.Active;
      FileFormat := TFileFormat(FMenuFileFormat.ActiveIndex);
      Extention := FEExtention.Entry.Text;
      MRUCount := FEMRUCount.AsInteger;
      ShowProgress := FCBProgressWindow.Active;
      end;
  inherited;
end;

procedure TSettingsDialog.DoDialogInit (InitData:pointer);
begin
  with PSettingsRec(InitData)^ do
    begin
    FCBSaveOnClose.Active := SaveOnClose;
    FEFileFormat.SetHistory (ord(FileFormat));
    FEExtention.Entry.Text := Extention;
    FEMRUCount.AsInteger := MRUCount;
    FCBProgressWindow.Active := ShowProgress;
    end;
  inherited;
end;

procedure Log (s : string);
begin
  {$ifdef UseLog}
  writeln (s);
  {$endif}
end;

procedure Log (fmt : string; params : array of const);
begin
  Log (format (fmt, params));
end;

procedure Log (indent:integer; fmt:string; params:array of const);
begin
  Log (stringofchar(' ',indent) + format(fmt, params));
end;

procedure Log (indent:integer; s:string);
begin
  Log (stringofchar(' ',indent) + s);
end;

end.
