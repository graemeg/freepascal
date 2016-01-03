<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{%delfiles=dump.bin}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Source provided for Free Pascal Bug Report 14708 }
{ Submitted by "Anton Kavalenka" on  2009-11-11 }
{ e-mail:  }
program tw14709;

{$mode delphi}{$H+}
{$apptype console}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Classes,Sysutils
  { you can add units after this };

var ok : boolean = true;

type
  TIntComponent=class(TComponent)
  public
    procedure DoChange(Sender:TObject);
  end;

  TTestComponent=class(TComponent)
  private
    fButton,
    fEdit:TIntComponent;
    fOnChangeButton,fOnChangeEdit:TNotifyEvent;
    fStr:string;
  public
    constructor Create(AnOwner:TComponent);override;
    procedure GetChildren(Proc: TGetChildProc; Root: TComponent);override;
    procedure Change;
  published
    property Str:string read fStr write fStr;
    property OnChangeButton:TNotifyEvent read fOnChangeButton write fOnChangeButton;
    property OnChangeEdit:TNotifyEvent read fOnChangeEdit write fOnChangeEdit;
  end;

procedure TIntComponent.DoChange(Sender:TObject);
begin
  writeln(Self.className+' reports that '+Sender.ClassName+' changed');
end;

constructor TTestComponent.Create(AnOwner:TComponent);
begin
  inherited Create(AnOwner);
  fStr:='Test string';
  fButton:=TIntComponent.Create(Self);
  fOnChangeButton:=fButton.DoChange;

  fEdit:=TIntComponent.Create(Self);
  fOnChangeEdit:=fEdit.DoChange;
end;

procedure TTestComponent.GetChildren(Proc: TGetChildProc; Root: TComponent);
var
  i:integer;
begin
 { for i:=0 to Componentcount-1 do
  Proc(Components[i]);
  }
end;

procedure TTestComponent.Change;
begin
  writeln(format('OnChangeButton code=%x data=%x',
    [ptruint(TMethod(fOnChangeButton).Code),
     ptruint(TMethod(fOnChangeButton).Data)]));
  writeln(format('OnChangeEdit code=%x data=%x',
    [ptruint(TMethod(fOnChangeEdit).Code),
     ptruint(TMethod(fOnChangeEdit).Data)]));

  if Assigned(OnChangeButton) then
    OnChangeButton(Self)
  else
    begin

      writeln('OnChangeButton handler is clear');
      ok:=false;
    end;
  if Assigned(OnChangeEdit) then
    OnChangeEdit(Self)
  else
    begin

      writeln('OnChangeEdit handler is clear');
      ok:=false;
    end;
end;

var
  tc:TTestComponent;
  ms,os,f:TStream;
begin
  RegisterClasses([TTestComponent,TIntComponent]);
  tc:=TTestComponent.Create(nil);
  writeln('Testing....');
  tc.Change;
  ms:=TmemoryStream.Create;
  ms.WriteComponent(tc);
  writeln('Cleanup...');
  tc.free;
  ms.Position:=0;

  writeln('Dumping streamed object as text:');
  
  f:=TFileStream.Create('dump.bin',fmCreate);
  f.CopyFrom(ms,ms.size);
  f.free;
  
  
  ms.Position:=0;
  os:=TMemoryStream.Create;
  ObjectBinaryToText(ms,os);
  os.Position:=0;
  repeat
    write(char(os.ReadByte));
  until os.Position>=os.Size;
  writeln();

  ms.Position:=0;
  tc:=TTestComponent(ms.ReadComponent(nil));
  writeln('Just read, testing ...');
  tc.Change;

  tc.free;
  ms.free;
  os.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if not ok then
=======
  if not ok then 
>>>>>>> graemeg/cpstrnew
=======
  if not ok then 
>>>>>>> graemeg/cpstrnew
=======
  if not ok then 
>>>>>>> graemeg/cpstrnew
=======
  if not ok then 
>>>>>>> origin/cpstrnew
    halt(1);
end.

