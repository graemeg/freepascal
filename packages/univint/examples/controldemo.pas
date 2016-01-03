{
 controldemo.pas

 *****************************************************************************
 *                                                                           *
 *  This demonstration program is public domain, which means no copyright,   *
 * but also no warranty!                                                     *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                     *
 *                                                                           *
 *****************************************************************************

 This application will create a window with two buttons in it

 When you click the button 'Hello Button',
  it will show or hide (alternating with each click) a text on the window

 When you click the button 'Show Dialog', it will show a modal message dialog

 Author: Felipe Monteiro de Carvalho

 Contributors: Ingemar Ragnemalm
}
program controldemo;

{$mode delphi}

uses
<<<<<<< HEAD
<<<<<<< HEAD
 SysUtils, MacOSAll, MacPas;
=======
 SysUtils, FPCMacOSAll, MacPas;
>>>>>>> graemeg/fixes_2_2
=======
 SysUtils, FPCMacOSAll, MacPas;
>>>>>>> origin/fixes_2_2

var
  mainWindow: WindowRef;
  contentView: HIViewRef;
  button1, button2: ControlRef;
  staticText: ControlRef;
  showTextFlag: Boolean = false;

const
  kButtonHello = 'HELO';
  kButtonMessage = 'MSGE';

{ implementation of the functions }

{ Functions to easely generate carbon structures }

<<<<<<< HEAD
<<<<<<< HEAD
function GetQDRect(Left, Top, Width, Height: Integer): MacOSAll.Rect;
=======
function GetQDRect(Left, Top, Width, Height: Integer): FPCMacOSAll.Rect;
>>>>>>> graemeg/fixes_2_2
=======
function GetQDRect(Left, Top, Width, Height: Integer): FPCMacOSAll.Rect;
>>>>>>> origin/fixes_2_2
begin
  result.Left := Left;
  result.Top := Top;
  result.Right := Left + Width;
  result.Bottom := Top + Height;
end;

{ Shows a message box }

procedure DoShowMessage(ATitle, AMsg: string);
var
  outItemHit: SInt16;
  err: OSErr;
begin
  err := StandardAlert(kAlertNoteAlert, ATitle, AMsg, nil, outItemHit);
end;

{ Event handling routines }

{  Here we alternate the visibility status of the static text
  with each button click }
function ButtonHelloPressed: OSStatus;
begin
  result := 0;

  showTextFlag := not showTextFlag;

  if showTextFlag then HIViewSetVisible(staticText, True)
  else HIViewSetVisible(staticText, False);
end;

function ButtonMessagePressed: OSStatus;
begin
  result := 0;

  DoShowMessage('Standard message dialog', 'This dialog is modal');
end;

{ Message handling function }

function WindowCommandHandler(nextHandler: EventHandlerCallRef; theEvent: EventRef; userDataPtr: UnivPtr): OSStatus; cdecl;
var
  status: OSStatus;
  ignoreresult: OSStatus;
  aCommand: HICommand;
begin
  status := eventNotHandledErr;

  ignoreresult := GetEventParameter(theEvent, kEventParamDirectObject,
   typeHICommand, nil, sizeof(aCommand), nil, @aCommand);

  if aCommand.commandID = FOUR_CHAR_CODE(kButtonHello) then status := ButtonHelloPressed()
  else if aCommand.commandID = FOUR_CHAR_CODE(kButtonMessage) then status := ButtonMessagePressed();

  result := status;
end;

{ Initialization and finalization routines }

procedure Initialize;
var
  status, ignoreResult: OSStatus;
  cmdEvent: EventTypeSpec;
  eventHandler: EventHandlerUPP;
  fontStyle: ControlFontStyleRec;
<<<<<<< HEAD
<<<<<<< HEAD
  psn: ProcessSerialNumber;
begin
  psn.highLongOfPSN:=0;
  psn.lowLongOfPSN:=kCurrentProcess;
  TransformProcessType( psn, kProcessTransformToForegroundApplication );
  setFrontProcess( psn );
=======
begin
>>>>>>> graemeg/fixes_2_2
=======
begin
>>>>>>> origin/fixes_2_2
  status := CreateNewWindow(kDocumentWindowClass,
   (kWindowStandardDocumentAttributes or kWindowStandardHandlerAttribute
    or kWindowCompositingAttribute),
   GetQDRect(100, 100, 350, 350), mainWindow);

  if (status <> noErr) or (mainWindow = nil) then
  begin
    DoShowMessage('Error', 'CreateNewWindow failed');
    Exit;
  end;

  ignoreResult := SetWindowTitleWithCFString(mainWindow, CFSTRP('Carbon FPC Controls Demo'));

  ignoreResult := HIViewFindByID(HIViewGetRoot(mainWindow), kHIViewWindowContentID, contentView);

  { Add events }

  cmdEvent.eventClass := kEventClassCommand;
  cmdEvent.eventKind := kEventCommandProcess;
  eventHandler := NewEventHandlerUPP(@WindowCommandHandler);
  ignoreResult := InstallEventHandler(GetWindowEventTarget(mainWindow),
   eventHandler, 1, @cmdEvent, nil, nil);

  { Creates the hello button }

  ignoreResult := CreatePushButtonControl(nil, GetQDRect(50, 200, 100, 50),
   CFSTRP('Hello Button'), button1);

  ignoreResult := HIViewAddSubview(contentView, button1);
  ignoreResult := SetControlCommandID(button1, FOUR_CHAR_CODE(kButtonHello));
  ignoreResult := HIViewSetVisible(button1, TRUE);

  { Creates the message button }

  ignoreResult := CreatePushButtonControl(nil, GetQDRect(200, 200, 100, 50),
   CFSTRP('Show Dialog'), button2);

  ignoreResult := HIViewAddSubview(contentView, button2);
  ignoreResult := SetControlCommandID(button2, FOUR_CHAR_CODE(kButtonMessage));
  ignoreResult := HIViewSetVisible(button2, TRUE);

  { Creates the text control }

  fontStyle.flags := kControlUseJustMask or kControlUseSizeMask;
  fontStyle.just := teCenter;
  fontStyle.size := 30;

  ignoreResult := CreateStaticTextControl(mainWindow,
   GetQDRect(0, 50, 350, 50), nil, @fontStyle, staticText);

  ignoreResult := HIViewAddSubview(contentView, staticText);
  ignoreResult := HIViewSetVisible(staticText, FALSE);

  HIViewSetText(staticText, CFSTRP('Hello Controls!'));

  { Shows the window }

  ShowWindow(mainWindow);
end;

procedure DoCloseWindow(theWind: WindowRef);
var
  theEvent: EventRef;
begin
  CreateEvent(nil, kEventClassWindow, kEventWindowClose, GetCurrentEventTime, kEventAttributeNone, theEvent);
  SetEventParameter(theEvent, kEventParamDirectObject, typeWindowRef, sizeof(WindowRef), theWind);
  SendEventToEventTarget(theEvent, GetWindowEventTarget(theWind));
end;

{ Closes all windows, so they have time to save any user data (none in this case) }

procedure Finalize;
begin
  DoCloseWindow(mainWindow);
end;

{ Main program section }

begin
  Initialize();

  RunApplicationEventLoop();

  Finalize();
end.

