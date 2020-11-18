{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    Options menu entries

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

procedure TIDEApp.SetSwitchesMode;
var R,R2: TRect;
    D: PCenterDialog;
    RB: PRadioButtons;
    i : TSwitchMode;
    SwitchesCount : integer;
    LastItem: PSItem;
    L: longint;
begin
  SwitchesCount:=ord(high(TSwitchMode))-ord(low(TSwitchMode))+1;
  R.Assign(0,0,36,4+SwitchesCount);
  New(D, Init(R, dialog_switchesmode));
  with D^ do
  begin
    HelpCtx:=hcSwitchesMode;
    GetExtent(R);
    R.Grow(-3,-1);
    Inc(R.A.Y);
    R2.Copy(R);
    Inc(R2.A.Y);
    R2.B.Y:=R2.A.Y+SwitchesCount;
    LastItem:=nil;
    for I:=high(TSwitchMode) downto low(TSwitchMode) do
      LastItem:=NewSItem(SwitchesModeName[I], LastItem);
    New(RB, Init(R2, LastItem));
    L:=ord(SwitchesMode);
    { ^^^ this is necessary, since TRadioButtons.GetData() reads a full
      longint and by just specifying the SwitchesMode var (only 1 bytes),
      the three bytes located next to it in the memory will determine the
      three most significant bytes of the longint. And if they aren't all
      zero, then we will select some items outside the actual ones... }
    RB^.SetData(L);
    Insert(RB);
    R2.Copy(R);
    R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, static_switchesmode_switchesmode, RB)));
  end;
  InsertButtons(D);
  RB^.Select;
  if Desktop^.ExecView(D)=cmOK then
   SwitchesMode:=TSwitchMode(RB^.Value);
  Dispose(D, Done);
  UpdateMode;
  UpdateTarget;
end;

procedure TIDEApp.DoCompilerSwitch;
var R,R2,R3,TabR,TabIR: TRect;
    D: PCenterDialog;
    CB1,CB2,CB3,CB4,CB5: PCheckBoxes;
    RB1,RB1b,RB2,RB4,RB5,RB6: PRadioButtons;
    Items: PSItem;
    IL: PEditorInputLine;
    IL2: PEditorInputLine;
    Count : integer;
    I,L: longint;
    Tab: PTab;
    Label11,Label12,
    Label21,Label22,Label23,Label23b,
    Label31,Label41,
    Label51,Label52,Label53: PLabel;
begin
  R.Assign(0,0,76,31);
  New(D, Init(R, dialog_compilerswitches));
  with D^ do
  begin
    HelpCtx:=hcCompilerNoAltX;
    GetExtent(R);
    R.Grow(-2,-1);
    Dec(R.B.Y,2);
    TabR.Copy(R);
    Dec(TabR.B.Y,2);
    TabIR.Copy(R);
    Inc(TabIR.A.Y,2);
    TabIR.Grow(0,-1);

    { --- Sheet 1 --- }
    Count:=SyntaxSwitches^.ItemCount;
    R.Copy(TabIR);
    R2.Copy(R);
    R2.B.X:=(R2.A.X+(R2.B.X-R2.A.X) div 2)-3;{ R2.B.X:=R2.B.X-4;}
{    R2.B.Y:=R2.A.Y+((Count+1) div 2);}
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(SyntaxSwitches^.ItemName(I), Items);
    New(CB1, Init(R2, Items));
    for I:=0 to Count-1 do
      if SyntaxSwitches^.GetBooleanItem(I) then
        CB1^.Press(I);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label11, Init(R2, label_compiler_syntaxswitches, CB1));

    Count:=CompilerModeSwitches^.ItemCount;
    R.Copy(TabIR);
    R2.Copy(R);
    R2.A.X:=(R2.A.X+(R2.B.X-R2.A.X) div 2)-2;
    R2.B.X:=R2.B.X-3;
{    R2.B.Y:=R2.A.Y+((Count+1) div 2);}
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(CompilerModeSwitches^.ItemName(I), Items);
    New(RB2, Init(R2, Items));
    L:=CompilerModeSwitches^.GetCurrSel;
    RB2^.SetData(L);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label12, Init(R2, label_compiler_mode, RB2));

    { --- Sheet 2 --- }
    Count:=CodegenSwitches^.ItemCount;
    R2.Copy(TabIR);
    R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2-2;
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(CodegenSwitches^.ItemName(I), Items);
    New(CB3, Init(R2, Items));
    for I:=0 to Count-1 do
      if CodegenSwitches^.GetBooleanItem(I) then
        CB3^.Press(I);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label21, Init(R2, label_compiler_codegeneration, CB3));

    Count:=OptimizationSwitches^.ItemCount;
    R2.Copy(TabIR);
    R2.A.X:=R2.B.X-(R2.B.X-R2.A.X) div 2;
    Dec(R2.B.X,4);
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(OptimizationSwitches^.ItemName(I), Items);
    New(CB2, Init(R2, Items));
    for I:=0 to Count-1 do
      if OptimizationSwitches^.GetBooleanItem(I) then
        CB2^.Press(I);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label22, Init(R2, label_compiler_optimizations, CB2));

    { --- Sheet 3 --- }
    Count:=ProcessorOptimizationSwitches^.ItemCount;
    R.Copy(TabIR);
    R2.Copy(R);
    R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2;
    Dec(R2.B.X,4);
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(ProcessorOptimizationSwitches^.ItemName(I), Items);
    New(RB1, Init(R2, Items));
    L:=ProcessorOptimizationSwitches^.GetCurrSel;
    RB1^.SetData(L);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label23, Init(R2, label_compiler_opt_targetprocessor, RB1));

    Count:=ProcessorCodeGenerationSwitches^.ItemCount;
    R.Copy(TabIR);
    R2.Copy(R);
    R2.A.X:=R2.A.X+(R2.B.X-R2.A.X) div 2;
    R2.B.X:=R2.B.X-3;
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(ProcessorCodeGenerationSwitches^.ItemName(I), Items);
    New(RB1b, Init(R2, Items));
    L:=ProcessorCodeGenerationSwitches^.GetCurrSel;
    RB1b^.SetData(L);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label23b, Init(R2, label_compiler_codegen_targetprocessor, RB1b));

    { --- Sheet 4 --- }
    Count:=VerboseSwitches^.ItemCount;
    R.Copy(TabIR);
    R2.Copy(R);
    R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2;
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(VerboseSwitches^.ItemName(I), Items);
    New(CB4, Init(R2, Items));
    for I:=0 to Count-1 do
      if VerboseSwitches^.GetBooleanItem(I) then
        CB4^.Press(I);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label31, Init(R2, label_compiler_verboseswitches, CB4));

    { --- Sheet 5 --- }
    Count:=BrowserSwitches^.ItemCount;
    R.Copy(TabIR);
    R2.Copy(R);
    R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2;
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(BrowserSwitches^.ItemName(I), Items);
    New(RB4, Init(R2, Items));
    L:=BrowserSwitches^.GetCurrSel;
    RB4^.SetData(L);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label41, Init(R2, label_compiler_browser, RB4));

    { --- Sheet 6 --- }
    Count:=AsmReaderSwitches^.ItemCount;
    R2.Copy(TabIR);
    R2.B.X:=R2.B.X-(R2.B.X-R2.A.X) div 2;
    Dec(R2.B.X,4);
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(AsmReaderSwitches^.ItemName(I), Items);
    New(RB5, Init(R2, Items));
    L:=AsmReaderSwitches^.GetCurrSel;
    RB5^.SetData(L);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label51, Init(R2, label_compiler_assemblerreader, RB5));

    R2.Copy(TabIR);
    R2.B.X:=R2.B.X-(R2.B.X-R2.A.X) div 2;
    Dec(R2.B.X,4);
    R2.A.Y:=R2.A.Y+Count+2;
    Count:=AsmInfoSwitches^.ItemCount;
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(AsmInfoSwitches^.ItemName(I), Items);
    New(CB5, Init(R2, Items));
    for I:=0 to Count-1 do
      if AsmInfoSwitches^.GetBooleanItem(I) then
        CB5^.Press(I);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label52, Init(R2, label_compiler_assemblerinfo, CB5));

    Count:=AsmOutputSwitches^.ItemCount;
    R2.Copy(TabIR);
    R2.A.X:=R2.B.X-(R2.B.X-R2.A.X) div 2;
    Dec(R2.B.X,4);
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    UpdateAsmOutputSwitches;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(AsmOutputSwitches^.ItemName(I), Items);
    New(RB6, Init(R2, Items));
    L:=AsmOutputSwitches^.GetCurrSel;
    RB6^.SetData(L);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    New(Label53, Init(R2, label_compiler_assembleroutput, RB6));

    { create tabs }
    New(Tab, Init(TabR,
      NewTabDef(page_compiler_syntax,CB1,
        NewTabItem(Label11,
        NewTabItem(CB1,
        NewTabItem(Label12,
        NewTabItem(RB2,
        nil)))),
      NewTabDef(page_compiler_codegeneration,CB3,
        NewTabItem(Label21,
        NewTabItem(CB3,
        NewTabItem(Label22,
        NewTabItem(CB2,
        nil)))),
      NewTabDef('~P~rocessor',RB1,
        NewTabItem(Label23,
        NewTabItem(RB1,
        NewTabItem(Label23b,
        NewTabItem(RB1b,
        nil)))),
      NewTabDef(page_compiler_verbose,CB4,
        NewTabItem(Label31,
        NewTabItem(CB4,
        nil)),
      NewTabDef(page_compiler_browser,RB4,
        NewTabItem(Label41,
        NewTabItem(RB4,
        nil)),
      NewTabDef(page_compiler_assembler,CB1,
        NewTabItem(Label51,
        NewTabItem(RB5,
        NewTabItem(Label52,
        NewTabItem(CB5,
        NewTabItem(Label53,
        NewTabItem(RB6,
        nil)))))),
      nil))))))));
    Tab^.GrowMode:=0;
    Insert(Tab);

    { conditionnals }
    R2.Copy(TabR); R2.A.Y:=R2.B.Y+1; R2.B.Y:=R2.A.Y+1; Dec(R2.B.X,4);
    New(IL, Init(R2, 255));
    IL^.Data^:=ConditionalSwitches^.GetStringItem(0);
    Insert(IL);
    R3.Copy(R2); R3.A.X:=R2.B.X+1; R3.B.X:=R3.A.X+3;
    Insert(New(PHistory, Init(R3, IL, hidConditionalDefines)));
    R2.Move(0,-1);
    Insert(New(PLabel, Init(R2,ConditionalSwitches^.ItemName(0), IL)));

    {custom }
    inc(R2.A.Y,3); inc(R2.B.Y,3);
    New(IL2, Init(R2, 255));
    IL2^.Data^:=CustomArg[SwitchesMode];
    Insert(IL2);
    R3.Copy(R2); R3.A.X:=R2.B.X+1; R3.B.X:=R3.A.X+3;
    Insert(New(PHistory, Init(R3, IL2, hidCompilerArgs)));
    R2.Move(0,-1);
    Insert(New(PLabel, Init(R2,label_debugger_compilerargs, IL2)));
  end;
  InsertButtons(D);
  if Desktop^.ExecView(D)=cmOK then
  begin
    for I:=0 to SyntaxSwitches^.ItemCount-1 do
      SyntaxSwitches^.SetBooleanItem(I,CB1^.Mark(I));
    CompilerModeSwitches^.SetCurrSel(RB2^.Value);
    for I:=0 to CodeGenSwitches^.ItemCount-1 do
      CodegenSwitches^.SetBooleanItem(I,CB3^.Mark(I));
    for I:=0 to OptimizationSwitches^.ItemCount-1 do
      OptimizationSwitches^.SetBooleanItem(I,CB2^.Mark(I));
    for I:=0 to VerboseSwitches^.ItemCount-1 do
      VerboseSwitches^.SetBooleanItem(I,CB4^.Mark(I));
    ProcessorOptimizationSwitches^.SetCurrSel(RB1^.Value);
    ProcessorCodeGenerationSwitches^.SetCurrSel(RB1b^.Value);
    AsmReaderSwitches^.SetCurrSel(RB5^.Value);
    for I:=0 to AsmInfoSwitches^.ItemCount-1 do
      AsmInfoSwitches^.SetBooleanItem(I,CB5^.Mark(I));
    AsmOutputSwitches^.SetCurrSel(RB6^.Value);
    BrowserSwitches^.SetCurrSel(RB4^.Value);
    ConditionalSwitches^.SetStringItem(0,IL^.Data^);
    CustomArg[SwitchesMode]:=IL2^.Data^;
  end;
  Dispose(D, Done);
end;

procedure TIDEApp.MemorySizes;
var R,R2,R3: TRect;
    D: PCenterDialog;
    ILs: array[0..10] of PIntegerLine;
    I: integer;
begin
  R.Assign(0,0,40,2+MemorySwitches^.ItemCount*2);
  New(D, Init(R, dialog_memorysizes));
  with D^ do
  begin
    HelpCtx:=hcmemorysizes;
    GetExtent(R); R.Grow(-3,-1);
    R2.Copy(R); Inc(R2.A.Y); R2.B.Y:=R2.A.Y+1;
    for I:=0 to MemorySwitches^.ItemCount-1 do
     begin
       R3.Copy(R2); R3.A.X:=21; R3.B.X:=R3.A.X+10;
       New(ILs[I], Init(R3, MinMemSize, MaxMemSize));
       ILs[I]^.Data^:=IntToStr(MemorySwitches^.GetLongintItem(I));
       Insert(ILs[I]);
       R3.Copy(R2); R3.B.X:=21;
       Insert(New(PLabel, Init(R3, MemorySwitches^.ItemName(I), ILs[I])));
       R2.Move(0,2);
     end;

{    R3.Copy(R2); R3.A.X:=21; R3.B.X:=R3.A.X+10;
    New(IL2, Init(R3, MinHeapSize, MaxHeapSize));
    IL2^.Data^:=IntToStr(GetHeapSize);
    Insert(IL2);
    R3.Copy(R2); R3.B.X:=21;
    Insert(New(PLabel, Init(R3, 'Local ~h~eap size', IL2)));}
  end;
  InsertButtons(D);
  ILs[0]^.Select;
  if Desktop^.ExecView(D)=cmOK then
  begin
    for I:=0 to MemorySwitches^.ItemCount-1 do
     begin
       MemorySwitches^.SetLongintItem(I,StrToInt(ILs[I]^.Data^));
     end;
{    SetStackSize(StrToInt(IL1^.Data^));
    SetHeapSize(StrToInt(IL2^.Data^));}
  end;
  Dispose(D, Done);
end;

procedure TIDEApp.DoLinkerSwitch;
var R,R2: TRect;
    D: PCenterDialog;
    RB2: PRadioButtons;
    CB,CB2 : PCheckBoxes;
    Count,I: longint;
    Items: PSItem;
    L: longint;
begin
  R.Assign(0,0,66,8);
  New(D, Init(R, dialog_linker));
  with D^ do
  begin
    HelpCtx:=hclinker;
    GetExtent(R); R.Grow(-3,-1);
    Count:=LinkAfterSwitches^.ItemCount;
    R2.Copy(R);
    R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2 - 1;
    Inc(R2.A.Y,1);
    R2.B.Y:=R2.A.Y+Count;
    Items:=nil;
    for I:=Count-1 downto 0 do
      Items:=NewSItem(LinkAfterSwitches^.ItemName(I), Items);
    New(CB, Init(R2, Items));
    for I:=0 to Count-1 do
      if LinkAfterSwitches^.GetBooleanItem(I) then
        CB^.Press(I);
    Insert(CB);
    Dec(R2.A.Y);
    R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_compiler_linkafter, CB)));
    R2.Copy(R);
    R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2 - 1;
    Inc(R2.A.Y,Count+2);
    R2.B.Y:=R2.A.Y+1;
    Items:=NewSItem(OtherLinkerSwitches^.ItemName(1), Nil);
    New(CB2, Init(R2, Items));
    if OtherLinkerSwitches^.GetBooleanItem(1) then
      CB2^.Press(0);
    Insert(CB2);
    R2.Copy(R); Inc(R2.A.Y);
    R2.B.Y:=R2.A.Y+LibLinkerSwitches^.ItemCount;
    R2.A.X:=R2.B.X-(R2.B.X-R2.A.X) div 2+1;
    Items:=nil;
    for I:=LibLinkerSwitches^.ItemCount-1 downto 0 do
      Items:=NewSItem(LibLinkerSwitches^.ItemName(I), Items);
    New(RB2, Init(R2, Items));
    L:=LibLinkerSwitches^.GetCurrSel;
    RB2^.SetData(L);
    Insert(RB2);

    R2.Copy(R); R2.B.Y:=R2.A.Y+1; R2.A.X:=R2.B.X-(R2.B.X-R2.A.X) div 2+1;
    Insert(New(PLabel, Init(R2, label_linker_preferredlibtype, RB2)));
  end;
  InsertButtons(D);
  RB2^.Select;
  if Desktop^.ExecView(D)=cmOK then
  begin
{    SetEXEFormat(TEXEFormat(RB1^.Value+1));}
    LibLinkerSwitches^.SetCurrSel(RB2^.Value);
    OtherLinkerSwitches^.SetBooleanItem(1,CB2^.Mark(0));
    if LibLinkerSwitches^.GetCurrSelParam='X' then
     if CompareText(TargetSwitches^.GetCurrSelParam,'Windows')=0 then
      if CompareText(AsmOutputSwitches^.GetCurrSelParam,'pecoff')<>0 then
        if ConfirmBox(
          FormatStrStr3(msg_xmustbesettoyforz_doyouwanttochangethis,
           label_compiler_assembleroutput,'pecoff','smartlinking'),nil,false)=cmYes then
             AsmOutputSwitches^.SetCurrSelParam('pecoff');
    for I:=0 to LinkAfterSwitches^.ItemCount-1 do
      LinkAfterSwitches^.SetBooleanItem(I,CB^.Mark(I));
  end;
  Dispose(D, Done);
end;

procedure TIDEApp.DoDebuggerSwitch;

var R,R2,R3: TRect;
    D: PCenterDialog;
    RB,RB2 : PRadioButtons;
    CBStrip: PCheckBoxes;
{$ifdef Windows}
    CB2: PCheckBoxes;
{$endif Windows}
{$ifdef Unix}
    IL: PEditorInputLine;
{$endif Unix}
    L,I: longint;
    Items: PSItem;
const
{$ifdef Windows}
  OtherFieldLines = 3;
{$else not Windows}
{$ifdef Unix}
  OtherFieldLines = 3;
{$else not Unix}
  OtherFieldLines = 0;
{$endif Unix}
{$endif Windows}
begin
  R.Assign(0,0,60,2+DebugInfoSwitches^.ItemCount+1+2
      +ProfileInfoSwitches^.ItemCount+2+OtherFieldLines);
  New(D, Init(R, dialog_debugger));
  with D^ do
  begin
    HelpCtx:=hcdebugger;
    { Strip all }
    GetExtent(R); R.Grow(-3,-1);
    R2.Copy(R); Inc(R2.A.Y,2); R2.B.Y:=R2.A.Y+1;
    Items:=NewSItem(OtherLinkerSwitches^.ItemName(0), Nil);
    New(CBStrip, Init(R2, Items));
    if OtherLinkerSwitches^.GetBooleanItem(0) then
      CBStrip^.Press(0);
    Insert(CBStrip);

    GetExtent(R); R.Grow(-3,-1);
    R2.Copy(R); Inc(R2.A.Y,3); R2.B.Y:=R2.A.Y+DebugInfoSwitches^.ItemCount;
    Items:=nil;
    for I:=DebugInfoSwitches^.ItemCount-1 downto 0 do
      Items:=NewSItem(DebugInfoSwitches^.ItemName(I), Items);
    New(RB, Init(R2, Items));
    L:=DebugInfoSwitches^.GetCurrSel;
    RB^.SetData(L);
    Insert(RB);
    R2.Copy(R); Inc(R2.A.Y); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_debugger_debuginfo, RB)));

    R2.Copy(R); Inc(R2.A.Y,3+DebugInfoSwitches^.ItemCount+2); R2.B.Y:=R2.A.Y+ProfileInfoSwitches^.ItemCount;
    Items:=nil;
    for I:=ProfileInfoSwitches^.ItemCount-1 downto 0 do
      Items:=NewSItem(ProfileInfoSwitches^.ItemName(I), Items);
    New(RB2, Init(R2, Items));
    L:=ProfileInfoSwitches^.GetCurrSel;
    RB2^.SetData(L);
    Insert(RB2);
    R2.Move(0,-1); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_debugger_profileswitches, RB2)));

{$ifdef Windows}
    R2.Move(0,ProfileInfoSwitches^.ItemCount+3);
    New(CB2,Init(R2,NewSItem(label_debugger_useanotherconsole, nil)));
    Insert(CB2);
    if DebuggeeTTY<>'' then
      L:=1
    else
      L:=0;
    CB2^.SetData(L);
{$ifdef GDB_WINDOWS_ALWAYS_USE_ANOTHER_CONSOLE}
    { EnableMask type is longint, avoid range check error here }
    CB2^.EnableMask := CB2^.EnableMask and longint($7ffffffe);
{$endif GDB_WINDOWS_ALWAYS_USE_ANOTHER_CONSOLE}
    R2.Move(0,-1);
    Insert(New(PLabel, Init(R2,label_debugger_redirection, CB2)));
{$endif Windows}
{$ifdef Unix}
    R2.Move(0,ProfileInfoSwitches^.ItemCount+3);
    New(IL, Init(R2, 255));
    IL^.Data^:=DebuggeeTTY;
    Insert(IL);
    R2.Move(0,-1);
    Insert(New(PLabel, Init(R2,label_debugger_useanothertty, IL)));
{$endif Unix}
  end;
  InsertButtons(D);
  RB^.Select;
  if Desktop^.ExecView(D)=cmOK then
  begin
    DebugInfoSwitches^.SetCurrSel(RB^.Value);
    ProfileInfoSwitches^.SetCurrSel(RB2^.Value);
    OtherLinkerSwitches^.SetBooleanItem(0,CBStrip^.Mark(0));
{$ifdef Windows}
    if CB2^.value<>0 then
      DebuggeeTTY:='on'
    else
      DebuggeeTTY:='';
{$endif Windows}
{$ifdef Unix}
    DebuggeeTTY:=IL^.Data^;
{$endif Unix}
  end;
  Dispose(D, Done);
end;


{$ifdef SUPPORT_REMOTE}
procedure TIDEApp.DoRemote;

var R,R2: TRect;
    IL1,IL2,IL3,IL4,IL5,IL6,IL7,IL8: PEditorInputLine;
    IL9,IL10,IL11 : PEditorInputLine;
    D: PCenterDialog;
const
  FieldLines = 11;
begin
{
     RemoteMachine : string = '';
     RemotePort : string = '2345';
     RemoteConfig : string = '';
     RemoteIdent : string = '';
     RemoteDir : string = '';
     RemoteSendCommand : string = 'scp $CONFIG $IDENT $LOCALFILE $REMOTEMACHINE:$REMOTEDIR';

}
  R.Assign(0,0,60,2+2*FieldLines+2);
  New(D, Init(R, dialog_remote));
  with D^ do
    begin
      HelpCtx:=hcremotedialog;
      GetExtent(R); R.Grow(-3,-1);
      R2.Copy(R);
      R2.B.Y:=R2.A.Y+1;
      { remote machine name }
      R2.Move(0,3);
      New(IL1, Init(R2, 255));
      IL1^.Data^:=RemoteMachine;
      Insert(IL1);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_machine, IL1)));
      { remote machine port }
      R2.Move(0,3);
      New(IL2, Init(R2, 255));
      IL2^.Data^:=RemotePort;
      Insert(IL2);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_port, IL2)));
      { remote machine dir }
      R2.Move(0,3);
      New(IL3, Init(R2, 255));
      IL3^.Data^:=RemoteDir;
      Insert(IL3);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_dir, IL3)));
      { remote machine config }
      R2.Move(0,3);
      New(IL4, Init(R2, 255));
      IL4^.Data^:=RemoteConfig;
      Insert(IL4);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_config, IL4)));
      { remote machine ident }
      R2.Move(0,3);
      New(IL5, Init(R2, 255));
      IL5^.Data^:=RemoteIdent;
      Insert(IL5);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_ident, IL5)));
      { remote machine send command  }
      R2.Move(0,3);
      New(IL6, Init(R2, 255));
      IL6^.Data^:=RemoteSendCommand;
      Insert(IL6);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_send_command, IL6)));
      { remote machine exec command  }
      R2.Move(0,3);
      New(IL7, Init(R2, 255));
      IL7^.Data^:=RemoteExecCommand;
      Insert(IL7);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_exec_command, IL7)));
      { remote machine exec command using ssh  }
      R2.Move(0,3);
      New(IL8, Init(R2, 255));
      IL8^.Data^:=RemoteSshExecCommand;
      Insert(IL8);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_ssh_exec_command, IL8)));
      { Remote shell executable }
      R2.Move(0,3);
      New(IL9, Init(R2, 255));
      IL9^.Data^:=RemoteShell;
      Insert(IL9);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_shell, IL9)));
      { Remote copy executable }
      R2.Move(0,3);
      New(IL10, Init(R2, 255));
      IL10^.Data^:=RemoteCopy;
      Insert(IL10);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_copy, IL10)));
      R2.Move(0,3);
      { Remote gdbserver }
      New(IL11, Init(R2, 255));
      IL11^.Data^:=RemoteGdbServer;
      Insert(IL11);
      R2.Move(0,-1);
      Insert(New(PLabel, Init(R2,label_remote_gdbserver, IL11)));
    end;
  InsertButtons(D);
  if Desktop^.ExecView(D)=cmOK then
  begin
    RemoteMachine:=IL1^.Data^;
    RemotePort:=IL2^.Data^;
    RemoteDir:=IL3^.Data^;
    RemoteConfig:=IL4^.Data^;
    RemoteIdent:=IL5^.Data^;
    RemoteSendCommand:=IL6^.Data^;
    RemoteExecCommand:=IL7^.Data^;
    RemoteSshExecCommand:=IL8^.Data^;
    RemoteShell:=IL9^.Data^;
    RemoteCopy:=IL10^.Data^;
    RemoteGdbServer:=IL11^.Data^;
  end;
  Dispose(D, Done);
end;
{$endif SUPPORT_REMOTE}



procedure TIDEApp.directories;

{Shows a window where the user can configure the directories the compilerproc
 will search files or output files to.}

var tab:Ptab;
    tabR,R,R2:Trect;
    D:PCenterDialog;
    s,misc_string:string;
    E_units,E_includes,E_libraries,E_objects,e:Pfpmemo;
    L_units,L_includes,L_libraries,L_objects:Plabel;
    c:PunsortedStringCollection;
    count,i,j:integer;
    IL:array[0..11] of PEditorInputLine;
    misc_items:PTabItem;
    misc_tabfocus:Pview;
    newmisc_items,
    temp : PTabItem;
const LW=25;

begin
  R.assign(0,0,screenwidth*64 div 80,18);
  new(D,init(R,dialog_directories));
  if d^.size.x<72 then
    misc_string:='~M~isc.'
  else
    misc_string:='~M~iscellaneous';

  {Create editors.}
  R.assign(1,4,d^.size.x-4,d^.size.y-4);
  new(E_units,init(R,nil,nil,nil));
  R.assign(1,3,d^.size.x-4,4);
  new(L_units,init(R,'Unit ~d~irectories:',E_units));

  R.assign(1,4,d^.size.x-4,d^.size.y-4);
  new(E_includes,init(R,nil,nil,nil));
  R.assign(1,3,d^.size.x-4,4);
  new(L_includes,init(R,'Include ~d~irectories:',E_includes));

  R.assign(1,4,d^.size.x-4,d^.size.y-4);
  new(E_libraries,init(R,nil,nil,nil));
  R.assign(1,3,d^.size.x-4,4);
  new(L_libraries,init(R,'Library ~d~irectories:',E_libraries));

  R.assign(1,4,d^.size.x-4,d^.size.y-4);
  new(E_objects,init(R,nil,nil,nil));
  R.assign(1,3,d^.size.x-4,4);
  new(L_objects,init(R,'Object file ~d~irectories:',E_objects));

  {The switches that are put into the editors are of type multistring.
   We add multistrings to the editor. Other inputboxes are created on
   demand on the "Miscellaneous" tab.}
  R.assign(1,4,d^.size.x-5,5);
  count:=DirectorySwitches^.ItemCount;
  misc_items:=nil;
  misc_tabfocus:=nil;
  for i:=0 to count-1 do
    begin
      if directorySwitches^.GetItemTyp(i)=ot_MultiString then
        begin
          case directorySwitches^.itemParam(i)[3] of
            'u':
              e:=E_units;
            'i':
              e:=E_includes;
            'l':
              e:=E_libraries;
            'o':
              e:=E_objects;
            else
              messagebox('Internal error: Unknown switch.',nil,mfOkButton);
          end;
          e^.setcontent(directorySwitches^.getMultiStringItem(i));
          e^.addline(''); {Empty line so user can scroll below existing dirs.}
          IL[i]:=nil;
        end
      else
        begin
          R2.copy(R);
          R2.A.X:=LW;
          new(IL[i],init(R2,255));
          IL[i]^.data^:=DirectorySwitches^.GetStringItem(i);
          misc_items:=newTabItem(IL[i],misc_items);
          if misc_tabfocus=nil then
            misc_tabfocus:=IL[i];
          R2.copy(R);
          R2.B.X:=LW;
          misc_items:=newTabItem(
                        new(Plabel,init(R2,
                                        DirectorySwitches^.ItemName(i),
                                        IL[i])),
                        misc_items);
          R.move(0,2);
        end;
    end;

  { revert items for correct tab order }
  newmisc_items:=nil;
  while assigned(misc_items) do
    begin
      { get element }
      temp:=misc_items;
      misc_items:=temp^.next;
      { put element }
      temp^.next:=newmisc_items;
      newmisc_items:=temp;
    end;
  misc_items:=newmisc_items;

  {Create some tabs in the window.}
  tabR.assign(1,1,d^.size.x-2,d^.size.y-1);
  new(tab,init(tabR,
    newtabdef('~U~nits',e_units,
      NewTabItem(L_units,
      NewTabItem(E_units,
      nil)),
    NewTabDef('~I~nclude files',E_includes,
      NewTabItem(L_includes,
      NewTabItem(E_includes,
      nil)),
    NewTabDef('~L~ibraries',E_libraries,
      NewTabItem(L_libraries,
      NewTabItem(E_libraries,
      nil)),
    NewTabDef('~O~bject files',E_objects,
      NewTabItem(L_objects,
      NewTabItem(E_objects,
      nil)),
    NewTabDef(misc_string,misc_tabfocus,
      misc_items,
    nil)))))
  ));

  tab^.growmode:=0;
  d^.insert(tab);
  insertbuttons(D);
  if desktop^.execview(D)=cmOK then
    begin
      {Move the data from the window back into the switches.}
      for i:=0 to count-1 do
        if directorySwitches^.GetItemTyp(i)=ot_MultiString then
          begin
            case directorySwitches^.itemParam(i)[3] of
              'u':
                e:=E_units;
              'i':
                e:=E_includes;
              'l':
                e:=E_libraries;
              'o':
                e:=E_objects;
              else
                messagebox('Internal error: Unknown switch.',nil,mfOkButton);
            end;
            c:=directorySwitches^.getMultiStringItem(i);
            c^.freeall;
            for j:=0 to e^.getlinecount-1 do
              begin
                s:=e^.getlinetext(j);
                {Strip string.}
                while (length(s)>0) and (s[length(s)]=' ') do
                  dec(s[0]);
                while (length(s)>0) and (s[1]=' ') do
                  system.delete(s,1,1);
                if s<>'' then
                  c^.insert(newstr(s));
              end;
          end
        else
          begin
            s:=IL[i]^.data^;
            {Strip string.}
            while (length(s)>0) and (s[length(s)]=' ') do
              dec(s[0]);
            while (length(s)>0) and (s[1]=' ') do
              system.delete(s,1,1);
            DirectorySwitches^.SetStringItem(i,s);
          end;
    end;
  dispose(D,done);
end;

procedure TIDEApp.Tools;
var
  D : PToolsDialog;
begin
  D:=New(PToolsDialog, Init);
  ExecuteDialog(D,nil);
end;

(*procedure TIDEApp.Preferences;
var R,R2: TRect;
    D: PCenterDialog;
    RB1 : PRadioButtons;
    CountModes : integer;
    hp : pvideomodelist;
    items : PSItem;
    videomode : tvideomode;
    i,modevalue : longint;


  function ToStr(l : longint) : string;

    var
       s : string;

    begin
       str(l,s);
       ToStr:=s;
    end;

  const
     color2str : array[false..true] of string = ('in b/w','in color');

begin
  GetVideoMode(videomode);
  CountModes:=0;
  i:=0;
  modevalue:=0;
  R.Assign(0,0,64,18);
  New(D, Init(R, 'Preferences'));
  with D^ do
  begin
     hp:=video.modes;
     items:=nil;
     r2.assign(2,3,24,17);
     while assigned(hp) do
       begin
          items:=NewSItem(ToStr(hp^.col)+'x'+ToStr(hp^.row)+' '+color2str[hp^.color],items);
          if (hp^.col=videomode.col) and (hp^.row=videomode.row) and
            (hp^.color=videomode.color) then
            modevalue:=i;
          inc(CountModes);
          { we can't display an infinite number of modes }
          if CountModes>=r.b.y-r.a.y+1 then
            break;
          inc(i);
          hp:=hp^.next;
       end;
     modevalue:=CountModes-modevalue-1;
     new(rb1,init(r2,items));
     insert(rb1);
     rb1^.value:=modevalue;

     r2.move(0,-1);r2.b.y:=r2.a.y+1;
     insert(new(plabel,init(r2,'~V~ideo mode',rb1)));
  end;
  InsertButtons(D);
  if Desktop^.ExecView(D)=cmOK then
   begin
      { change video mode ? }
      if rb1^.value<>modevalue then
        begin
           modevalue:=CountModes-rb1^.value-1;
           hp:=video.modes;
           for i:=1 to modevalue do
             hp:=hp^.next;
           videomode.col:=hp^.col;
           videomode.row:=hp^.row;
           videomode.color:=hp^.color;
           SetScreenVideoMode(videomode);
        end;
   end;
  Dispose(D, Done);
end;*)

type
     PVideoModeCollection = ^TVideoModeCollection;
     TVideoModeCollection = object(TSortedCollection)
       function  Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
       procedure FreeItem(Item: Pointer); virtual;
     end;

function TVideoModeCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
var R: Sw_integer;
    K1: PVideoMode absolute Key1;
    K2: PVideoMode absolute Key2;
begin
  if K1^.Col<K2^.Col then R:=-1 else
  if K1^.Col>K2^.Col then R:= 1 else
  if K1^.Row<K2^.Row then R:=-1 else
  if K1^.Row>K2^.Row then R:= 1 else
  if (K1^.Color=false) and (K2^.Color=true ) then R:=-1 else
  if (K1^.Color=true ) and (K2^.Color=false) then R:= 1 else
  R:=0;
  Compare:=R;
end;

procedure TVideoModeCollection.FreeItem(Item: Pointer);
begin
  FreeMem(Item,sizeof(TVideoMode));
end;

procedure TIDEApp.Preferences;
var R,R2: TRect;
    D: PCenterDialog;
    C: PVideoModeCollection;
    VMLB: PVideoModeListBox;
    OldScreenMode,VM: TVideoMode;
    ScreenModeInfo : array[1..3] of longint;
    CurVP,VP: PVideoMode;
    RB1: PPlainRadioButtons;
    CB1,CB2: PPlainCheckBoxes;
    CurIdx: integer;
    i : word;
begin
  New(C, Init(10,50));
  CurVP:=nil;
  for i:=0 to GetVideoModeCount-1 do
   begin
      GetVideoModeData(i,VM);
      GetMem(VP,sizeof(TVideoMode));
      Move(VM,VP^,sizeof(TVideoMode));
      C^.Insert(VP);
      if (VM.Row=ScreenMode.Row) and (VM.Col=ScreenMode.Col) and
         (VM.Color=ScreenMode.Color) then
       CurVP:=VP;
   end;
  R.Assign(0,0,64,15);
  New(D, Init(R, dialog_preferences));
  with D^ do
  begin
    HelpCtx:=hcpreferences;
    GetExtent(R); R.Grow(-2,-2);
    R.B.X:=R.A.X+(R.B.X-R.A.X) div 2 - 1;

    R.B.Y:=R.A.Y+3;
    R2.Copy(R); R2.Grow(-1,-1);
    New(VMLB, Init(R2, Min(4,C^.Count), C));
    if CurVP=nil then CurIdx:=-1 else
      CurIdx:=C^.IndexOf(CurVP);
    if CurIdx<>-1 then
      VMLB^.FocusItem(CurIdx);
    Insert(New(PGroupView, Init(R, label_preferences_videomode, VMLB)));
    Insert(VMLB);

    R.Move(0,R.B.Y-R.A.Y{+1}); R.B.Y:=R.A.Y+4;
    R2.Copy(R); R2.Grow(-1,-1);
    New(RB1, Init(R2,
      NewSItem(label_preferences_currentdirectory,
      NewSItem(label_preferences_configdirectory,
      nil))));
    RB1^.Press(DesktopLocation);
    Insert(New(PGroupView, Init(R, label_preferences_desktopfile, RB1)));
    Insert(RB1);

    R.Move(0,R.B.Y-R.A.Y{+1}); R.B.Y:=R.A.Y+5;
    R2.Copy(R); R2.Grow(-1,-1);
    New(CB1, Init(R2,
      NewSItem(label_preferences_editorfiles,
      NewSItem(label_preferences_environment,
      NewSItem(label_preferences_desktop,
      nil)))));
    CB1^.Value:=AutoSaveOptions;
    Insert(New(PGroupView, Init(R, label_preferences_autosave, CB1)));
    Insert(CB1);

    GetExtent(R); R.Grow(-2,-2);
    R.A.X:=R.B.X-(R.B.X-R.A.X) div 2 + 1;
    R.B.Y:=R.A.Y+7;

    R.Move(0,R.B.Y-R.A.Y{+1}); R.B.Y:=R.A.Y+5;
    R2.Copy(R); R2.Grow(-1,-1);
    New(CB2, Init(R2,
      NewSItem(label_preferences_autotracksource,
      NewSItem(label_preferences_closeongotosource,
      NewSItem(label_preferences_changedironopen,
      nil)))));
    CB2^.Value:=MiscOptions;
    Insert(New(PGroupView, Init(R, label_preferences_options, CB2)));
    Insert(CB2);

  end;
  InsertButtons(D);
  if Desktop^.ExecView(D)=cmOK then
   begin
     if (C^.count>0) then
       begin
         with PVideoMode(C^.At(VMLB^.Focused))^ do
           begin
             VM.Col:=Col;
             VM.Row:=Row;
             VM.Color:=Color;
           end;
         if (VM.Col<>ScreenMode.Col) or (VM.Row<>ScreenMode.Row) or (VM.Color<>ScreenMode.Color) then
           Begin
             OldScreenMode:=ScreenMode;
             SetScreenVideoMode(VM);
             if (VM.Col<>ScreenMode.Col) or (VM.Row<>ScreenMode.Row) or (VM.Color<>ScreenMode.Color) then
               begin
                 SetScreenVideoMode(OldScreenMode);
                 ScreenModeInfo[1]:=VM.col;
                 ScreenModeInfo[2]:=VM.row;
                 ScreenModeInfo[3]:=byte(VM.color);

                 ErrorBox(msg_cantsetscreenmode,@ScreenModeInfo);
               end
             else
               if TimedMessageBox (msg_confirmnewscreenmode, nil,
                              mfConfirmation or mfOKCancel, 15) = cmCancel then
                begin
                 SetScreenVideoMode (OldScreenMode);
                 ScreenModeInfo [1] := VM.Col;
                 ScreenModeInfo [2] := VM.Row;
                 ScreenModeInfo [3] := byte (VM.Color);
                end;
           End;
       end;
     AutoSaveOptions:=CB1^.Value;
     MiscOptions:=CB2^.Value;
     DesktopLocation:=RB1^.Value;
   end;
  Dispose(D, Done);
  Dispose(C, Done);
end;

procedure TIDEApp.EditorOptions(Editor: PEditor);
var D: PCenterDialog;
    R,R2,R3: TRect;
    CB: PCheckBoxes;
    ILTab,ILIdent: PIntegerLine;
    ExtIL,TabExtIL: PEditorInputLine;
    TabSize,IndentSize: Integer;
    EFlags,EFValue: Longint;
    Title: string;
begin
  if Editor=nil then
    begin
      TabSize:=DefaultTabSize; EFlags:=DefaultCodeEditorFlags;
      IndentSize:=DefaultIndentSize;
      Title:=dialog_defaulteditoroptions;
    end
  else
    begin
      TabSize:=Editor^.GetTabSize; EFlags:=Editor^.GetFlags;
      IndentSize:=Editor^.GetIndentSize;
      Title:=dialog_editoroptions;
    end;

  EFValue:=0;
  if (EFlags and efBackupFiles       )<>0 then EFValue:=EFValue or (1 shl  0);
  if (EFlags and efInsertMode        )<>0 then EFValue:=EFValue or (1 shl  1);
  if (EFlags and efAutoIndent        )<>0 then EFValue:=EFValue or (1 shl  2);
  if (EFlags and efUseTabCharacters  )<>0 then EFValue:=EFValue or (1 shl  3);
  if (EFlags and efBackSpaceUnindents)<>0 then EFValue:=EFValue or (1 shl  4);
  if (EFlags and efPersistentBlocks  )<>0 then EFValue:=EFValue or (1 shl  5);
  if (EFlags and efSyntaxHighlight   )<>0 then EFValue:=EFValue or (1 shl  6);
  if (EFlags and efBlockInsCursor    )<>0 then EFValue:=EFValue or (1 shl  7);
  if (EFlags and efVerticalBlocks    )<>0 then EFValue:=EFValue or (1 shl  8);
  if (EFlags and efHighlightColumn   )<>0 then EFValue:=EFValue or (1 shl  9);
  if (EFlags and efHighlightRow      )<>0 then EFValue:=EFValue or (1 shl 10);
  if (EFlags and efAutoBrackets      )<>0 then EFValue:=EFValue or (1 shl 11);
  if (EFlags and efKeepTrailingSpaces)<>0 then EFValue:=EFValue or (1 shl 12);
  if (EFlags and efCodeComplete      )<>0 then EFValue:=EFValue or (1 shl 13);
  if (EFlags and efFolds             )<>0 then EFValue:=EFValue or (1 shl 14);

  R.Assign(0,0,66,20);
  New(D, Init(R, Title));
  with D^ do
  begin
    HelpCtx:=hcEditor;
    GetExtent(R); R.Grow(-2,-2); R.B.Y:=R.A.Y+9;
    R2.Copy(R); Inc(R2.A.Y);
    New(CB, Init(R2,
      NewSItem(label_editor_backupfiles,
      NewSItem(label_editor_insertmode,
      NewSItem(label_editor_autoindentmode,
      NewSItem(label_editor_usetabcharacters,
      NewSItem(label_editor_backspaceunindents,
      NewSItem(label_editor_persistentblocks,
      NewSItem(label_editor_syntaxhighlight,
      NewSItem(label_editor_blockinsertcursor,
      NewSItem(label_editor_verticalblocks,
      NewSItem(label_editor_highlightcolumn,
      NewSItem(label_editor_highlightrow,
      NewSItem(label_editor_autoclosingbrackets,
      NewSItem(label_editor_keeptrailingspaces,
      NewSItem(label_editor_codecomplete,
      NewSItem(label_editor_folds,
      nil)))))))))))))))));
    CB^.Value:=EFValue;
    Insert(CB);
    R2.Move(0,-1); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_editor_editoroptions, CB)));

    R.Move(0,(R.B.Y-R.A.Y)+1); R.B.Y:=R.A.Y+1;
    R2.Copy(R); Inc(R2.A.Y); R2.B.Y:=R2.A.Y;
    R3.Copy(R); Inc(R3.A.X,10); R3.B.X:=R3.A.X+5;
    New(ILTab, Init(R3, 0,100));
    ILTab^.Data^:=IntToStr(TabSize);
    Insert(ILTab);
    R3.Copy(R); R3.B.X:=R3.A.X+10;
    Insert(New(PLabel, Init(R3, label_editor_tabsize, ILTab)));
    R3.Copy(R); Inc(R3.A.X,40); R3.B.X:=R3.A.X+5;
    New(ILIdent, Init(R3, 0,100));
    ILIdent^.Data^:=IntToStr(IndentSize);
    Insert(ILIdent);
    R3.Copy(R); Inc(R3.A.X,28); R3.B.X:=R3.A.X+12;
    Insert(New(PLabel, Init(R3, label_editor_IndentSize, ILIdent)));

    R.Move(0,(R.B.Y-R.A.Y)+1); R.B.Y:=R.A.Y+2;
    R2.Copy(R); Inc(R2.A.Y);
    New(ExtIL, Init(R2, 128));
    ExtIL^.SetData(HighlightExts);
    Insert(ExtIL);
    R2.Move(0,-1);
    Insert(New(PLabel, Init(R2, label_editor_highlightextensions, ExtIL)));

    R.Move(0,(R.B.Y-R.A.Y)+1); R.B.Y:=R.A.Y+2;
    R2.Copy(R); Inc(R2.A.Y);
    New(TabExtIL, Init(R2, 128));
    TabExtIL^.SetData(TabsPattern);
    Insert(TabExtIL);
    R2.Move(0,-1);
    Insert(New(PLabel, Init(R2, label_editor_filepatternsneedingtabs, TabExtIL)));
  end;
  InsertButtons(D);
  CB^.Select;
  if Desktop^.ExecView(D)=cmOK then
  begin
    EFlags:=0;
    if (CB^.Value and (1 shl  0))<>0 then EFlags:=EFlags or efBackupFiles;
    if (CB^.Value and (1 shl  1))<>0 then EFlags:=EFlags or efInsertMode;
    if (CB^.Value and (1 shl  2))<>0 then EFlags:=EFlags or efAutoIndent;
    if (CB^.Value and (1 shl  3))<>0 then EFlags:=EFlags or efUseTabCharacters;
    if (CB^.Value and (1 shl  4))<>0 then EFlags:=EFlags or efBackSpaceUnindents;
    if (CB^.Value and (1 shl  5))<>0 then EFlags:=EFlags or efPersistentBlocks;
    if (CB^.Value and (1 shl  6))<>0 then EFlags:=EFlags or efSyntaxHighlight;
    if (CB^.Value and (1 shl  7))<>0 then EFlags:=EFlags or efBlockInsCursor;
    if (CB^.Value and (1 shl  8))<>0 then EFlags:=EFlags or efVerticalBlocks;
    if (CB^.Value and (1 shl  9))<>0 then EFlags:=EFlags or efHighlightColumn;
    if (CB^.Value and (1 shl 10))<>0 then EFlags:=EFlags or efHighlightRow;
    if (CB^.Value and (1 shl 11))<>0 then EFlags:=EFlags or efAutoBrackets;
    if (CB^.Value and (1 shl 12))<>0 then EFlags:=EFlags or efKeepTrailingSpaces;
    if (CB^.Value and (1 shl 13))<>0 then EFlags:=EFlags or efCodeComplete;
    if (CB^.Value and (1 shl 14))<>0 then EFlags:=EFlags or efFolds;
    TabSize:=StrToInt(ILTab^.Data^);
    IndentSize:=StrToInt(ILIdent^.Data^);
    if Editor=nil then
       begin
         DefaultTabSize:=TabSize;
         DefaultIndentSize:=IndentSize;
         DefaultCodeEditorFlags:=EFlags;
       end
    else
       begin
         Editor^.SetIndentSize(IndentSize);
         Editor^.SetTabSize(TabSize);
         Editor^.SetFlags(EFlags);
       end;
    ExtIL^.GetData(HighlightExts);
    TabExtIL^.GetData(TabsPattern);
  end;
  Dispose(D, Done);
end;

procedure TIDEApp.CodeComplete;
var
  D : PCodeCompleteDialog;
begin
  D:=New(PCodeCompleteDialog, Init);
  ExecuteDialog(D,nil);
end;

procedure TIDEApp.CodeTemplates;
begin
  ExecuteDialog(New(PCodeTemplatesDialog, Init(false,'')),nil);
end;

procedure TIDEApp.BrowserOptions(Browser: PBrowserWindow);
var D: PCenterDialog;
    R,R2,R3 : TRect;
    TitleS: string;
    CB1,CB2: PCheckBoxes;
    RB1,RB2: PRadioButtons;
begin
  if Browser=nil then
    TitleS:=dialog_browseroptions
  else
    TitleS:=dialog_localbrowseroptions;
  R.Assign(0,0,56,15);
  New(D, Init(R, TitleS));
  with D^ do
  begin
    HelpCtx:=hcBrowser;
    GetExtent(R); R.Grow(-2,-2);
    R.B.Y:=R.A.Y+1+3; R2.Copy(R); Inc(R2.A.Y);
    New(CB1, Init(R2,
      NewSItem(RExpand(label_browser_labels,21+2),
      NewSItem(label_browser_constants,
      NewSItem(label_browser_types,
      NewSItem(label_browser_variables,
      NewSItem(label_browser_procedures,
      NewSItem(label_browser_inherited,
      nil)))))))
    );
    Insert(CB1);
    R2.Move(0,-1); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_browser_symbols, CB1)));

    R.Move(0,R.B.Y-R.A.Y+1);
    R.B.Y:=R.A.Y+1+2; R2.Copy(R);
    R3.Copy(R2); R3.B.X:=R3.A.X+(R3.B.X-R3.A.X) div 2-1; Inc(R3.A.Y);
    New(RB1, Init(R3,
      NewSItem(label_browser_newbrowser,
      NewSItem(label_browser_currentbrowser,
      nil)))
    );
    Insert(RB1);
    R3.Move(0,-1); R3.B.Y:=R3.A.Y+1;
    Insert(New(PLabel, Init(R3, label_browser_subbrowsing, RB1)));
    R3.Copy(R2); R3.A.X:=R3.B.X-(R3.B.X-R3.A.X) div 2+1; Inc(R3.A.Y);
    New(RB2, Init(R3,
      NewSItem(label_browser_scope,
      NewSItem(label_browser_reference,
      nil)))
    );
    Insert(RB2);
    R3.Move(0,-1); R3.B.Y:=R3.A.Y+1;
    Insert(New(PLabel, Init(R3, label_browser_preferredpane, RB2)));

    R.Move(0,R.B.Y-R.A.Y+1);
    R.B.Y:=R.A.Y+1+1; R2.Copy(R); Inc(R2.A.Y);
    New(CB2, Init(R2,
      NewSItem(RExpand(label_browser_qualifiedsymbols,21+2),
      NewSItem(label_browser_sortsymbols,
      nil)))
    );
    Insert(CB2);
    R2.Move(0,-1); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_browser_display, CB2)));
  end;
  InsertButtons(D);
  CB1^.Select;
  if Desktop^.ExecView(D)=cmOK then
  begin
  end;
  Dispose(D, Done);
end;

procedure TIDEApp.Startup;
begin
  NotImplemented;
end;

procedure TIDEApp.DesktopOptions;
var R: TRect;
    D: PCenterDialog;
    CB: PCheckBoxes;
begin
  R.Assign(0,0,40,12);
  New(D, Init(R, dialog_desktoppreferences));
  with D^ do
  begin
    HelpCtx:=hcDesktopOptions;
    GetExtent(R); R.Grow(-2,-2); Inc(R.A.Y); R.B.Y:=R.A.Y+8;
    New(CB, Init(R,
      NewSItem(label_desktop_historylists,
      NewSItem(label_desktop_clipboard,
      NewSItem(label_desktop_watches,
      NewSItem(label_desktop_breakpoints,
      NewSItem(label_desktop_openwindow,
      NewSItem(label_desktop_symbolinfo,
      NewSItem(label_desktop_codecompletewords,
      NewSItem(label_desktop_codetemplates,
      nil))))))))));
    CB^.Value:=DesktopFileFlags;
    Insert(CB);
    R.Move(0,-1); R.B.Y:=R.A.Y+1;
    Insert(New(PLabel, Init(R, label_desktop_preservedacrosssessions, CB)));
  end;
  InsertButtons(D);
  CB^.Select;
  if Desktop^.ExecView(D)=cmOK then
    begin
      DesktopFileFlags:=CB^.Value;
    end;
  Dispose(D, Done);
end;

procedure TIDEApp.Mouse;
var R,R2: TRect;
    D: PCenterDialog;
    SB: PScrollBar;
    CB: PCheckBoxes;
    RB1,RB2,RBKB: PRadioButtons;
begin
  R.Assign(0,0,62,19);
  New(D, Init(R, dialog_mouseoptions));
  with D^ do
  begin
    HelpCtx:=hcMouse;
    GetExtent(R); R.Grow(-3,-2); inc(R.A.Y); R.B.Y:=R.A.Y+2;
    New(RBkb, Init(R,
      NewSItem('~C~UA-91 convention (Shift+Del,Ctrl+Ins,Shift+Ins)',
      NewSItem('~M~icrosoft convention (Ctrl+X,Ctrl+C,Ctrl+V)',
      nil))));
    RBkb^.Press(integer(EditKeys));
    Insert(RBkb);
    R.move(0,-1); R.B.Y:=R.A.Y+1;
    Insert(New(PLabel, Init(R, '~K~eys for cut, copy and paste:', RBkb)));

    R.Move(0,(R.B.Y-R.A.Y)+3); R.B.Y:=R.A.Y+3;
    R2.Copy(R); Inc(R2.A.Y,2); R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2 -1;
    New(SB, Init(R2)); SB^.GrowMode:=0; SB^.Options:=SB^.Options or ofSelectable;
    SB^.SetParams(DoubleDelay,1,20,1,1);
    Insert(SB);
    R2.Move(0,-1);
    Insert(New(PStaticText, Init(R2, label_mouse_speedbar)));
    R2.Move(-1,-1);
    Insert(New(PLabel, Init(R2, label_mouse_doubleclickspeed, SB)));

    R2.Copy(R); Inc(R2.A.Y,2); R2.A.X:=R2.B.X-(R2.B.X-R2.A.X) div 2 +1;
    New(CB, Init(R2, NewSItem(label_mouse_reversebuttons, nil) ));
    if MouseReverse then CB^.Press(0);
    Insert(CB);

    R.Move(0,(R.B.Y-R.A.Y)+1); R.B.Y:=R.A.Y+8;
    R2.Copy(R); Inc(R2.A.Y); R2.B.X:=R2.A.X+(R2.B.X-R2.A.X) div 2 -1;
    New(RB1, Init(R2,
      NewSItem(label_mouse_act_nothing,
      NewSItem(label_mouse_act_topicsearch,
      NewSItem(label_mouse_act_gotocursor,
      NewSItem(label_mouse_act_breakpoint,
      NewSItem(label_mouse_act_evaluate,
      NewSItem(label_mouse_act_addwatch,
      NewSItem(label_mouse_act_browsesymbol,
      nil)))))))));
    RB1^.Press(CtrlMouseAction);
    Insert(RB1);
    R2.Move(0,-1); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2, label_mouse_crtlrightmousebuttonaction, RB1)));

    R2.Copy(R); Inc(R2.A.Y); R2.A.X:=R2.B.X-(R2.B.X-R2.A.X) div 2 +1;
    New(RB2, Init(R2,
      NewSItem(label_mouse_act_nothing,
      NewSItem(label_mouse_act_topicsearch,
      NewSItem(label_mouse_act_gotocursor,
      NewSItem(label_mouse_act_breakpoint,
      NewSItem(label_mouse_act_evaluate,
      NewSItem(label_mouse_act_addwatch,
      NewSItem(label_mouse_act_browsesymbol,
      nil)))))))));
    RB2^.Press(AltMouseAction);
    Insert(RB2);
    R2.Move(0,-1); R2.B.Y:=R2.A.Y+1;
    Insert(New(PLabel, Init(R2,  label_mouse_altrightmousebuttonaction, RB2)));
  end;
  InsertButtons(D);

  RBkb^.Select;
  if Desktop^.ExecView(D)=cmOK then
  begin
    MouseReverse:=CB^.Mark(0);
    DoubleDelay:=SB^.Value;
    CtrlMouseAction:=RB1^.Value;
    AltMouseAction:=RB2^.Value;
    if Tedit_key_modes(RBkb^.value)<>EditKeys then
      begin
        EditKeys:=Tedit_key_modes(RBkb^.value);
        reload_menubar;
      end;
  end;
  Dispose(D, Done);
end;

procedure TIDEApp.Colors;
{$Ifdef COLORSEL}
var D: PColorDialog;
begin
  New(D, Init(AppPalette,
    ColorGroup(label_colors_grp_browser,
      ColorItem(label_colors_framepassive   , 215,
      ColorItem(label_colors_frameactive    , 216,
      ColorItem(label_colors_frameicon      , 217,
      ColorItem(label_colors_scrollbarpage  , 218,
      ColorItem(label_colors_scrollbaricons , 219,
      ColorItem(label_colors_normaltext     , 220,
      ColorItem(label_colors_selectedtext   , 221,
      ColorItem(label_colors_activeitem     , 222,
      ColorItem(label_colors_inactiveitem   , 223,
      ColorItem(label_colors_focuseditem    , 224,
      ColorItem(label_colors_selecteditem   , 225,
      ColorItem(label_colors_divider        , 226,
      nil)))))))))))),
    ColorGroup(label_colors_grp_clock,
      ColorItem(label_colors_clockview      , 227,
      nil),
    ColorGroup(label_colors_grp_desktop, DesktopColorItems(nil),
    ColorGroup(label_colors_grp_dialogs, DialogColorItems(dpGrayDialog,nil),
    ColorGroup(label_colors_grp_editor,
      ColorItem(label_colors_framepassive   , 167,
      ColorItem(label_colors_frameactive    , 168,
      ColorItem(label_colors_frameicon      , 169,
      ColorItem(label_colors_scrollbarpage  , 170,
      ColorItem(label_colors_scrollbaricons , 171,
      ColorItem(label_colors_normaltext     , 199,
      ColorItem(label_colors_selectedtext   , 208,
      ColorItem(label_colors_highlighcolumn , 209,
      ColorItem(label_colors_highlightrow   , 210,
      ColorItem(label_colors_errormessages  , 214,
      nil)))))))))),
    ColorGroup(label_colors_grp_help,
      ColorItem(label_colors_framepassive   , 128,
      ColorItem(label_colors_frameactive    , 129,
      ColorItem(label_colors_frameicon      , 130,
      ColorItem(label_colors_scrollbarpage  , 131,
      ColorItem(label_colors_scrollbaricons , 132,
      ColorItem(label_colors_helptext       , 160,
      ColorItem(label_colors_helplinks      , 161,
      ColorItem(label_colors_selectedlink   , 162,
      ColorItem(label_colors_selectedtext   , 163,
      ColorItem(label_colors_html_heading1  , 229,
      ColorItem(label_colors_html_heading2  , 230,
      ColorItem(label_colors_html_heading3  , 231,
      ColorItem(label_colors_html_heading4  , 232,
      ColorItem(label_colors_html_heading5  , 233,
      ColorItem(label_colors_html_heading6  , 234,
      nil))))))))))))))),
    ColorGroup(label_colors_grp_menus,   MenuColorItems(nil),
    ColorGroup(label_colors_grp_syntax,
      ColorItem(label_colors_whitespace      , 200,
      ColorItem(label_colors_comments        , 201,
      ColorItem(label_colors_reservedwords   , 202,
      ColorItem(label_colors_identifiers     , 203,
      ColorItem(label_colors_strings         , 204,
      ColorItem(label_colors_numbers         , 205,
      ColorItem(label_colors_hexnumbers      , 212,
      ColorItem(label_colors_assembler       , 206,
      ColorItem(label_colors_symbols         , 207,
      ColorItem(label_colors_directives      , 211,
      ColorItem(label_colors_tabs            , 213,
      nil))))))))))),
    nil))))))))));
  D^.HelpCtx:=hcColors;
  if ExecuteDialog(D, @AppPalette)=cmOK then
    begin
      DoneMemory;
      Message(Application,evBroadcast,cmUpdate,nil);
      ReDraw;
      UpdateScreen(true);
    end;
end;
{$else COLORSEL}
begin
end;
{$endif COLORSEL}

procedure TIDEApp.OpenINI;
var D: PFileDialog;
    FileName: string;
begin
  New(D, Init('*'+ExtOf(INIFileName),dialog_openoptions,dialog_ini_filename,fdOpenButton,hidOpenIniFile));
  D^.HelpCtx:=hcOpenIni;
  if Desktop^.ExecView(D)<>cmCancel then
    begin
      D^.GetFileName(FileName);
      if ExistsFile(FileName)=false then ErrorBox(msg_cantopenconfigfile,nil) else
        begin
          IniFileName:=FileName;
          ReadINIFile;
          Message(Application,evBroadcast,cmUpdate,nil);
        end;
    end;
  Dispose(D, Done);
end;

procedure TIDEApp.SaveINI;
begin
  if WriteINIFile(false)=false then
    ErrorBox(msg_errorsavingconfigfile,nil);
end;

procedure TIDEApp.SaveAsINI;
var D: PFileDialog;
    FileName: string;
    CanWrite: boolean;
begin
  New(D, Init('*'+ExtOf(INIFileName),dialog_saveoptions,dialog_ini_filename,fdOkButton,hidSaveIniFile));
  D^.HelpCtx:=hcSaveAsINI;
  if Desktop^.ExecView(D)<>cmCancel then
    begin
      D^.GetFileName(FileName);
      CanWrite:=(ExistsFile(FileName)=false);
      if CanWrite=false then
        CanWrite:=ConfirmBox(FormatStrStr(msg_filealreadyexistsoverwrite,SmartPath(FileName)),nil,false)=cmYes;
      if CanWrite then
        begin
          IniFileName:=FileName;
          if WriteINIFile(true)=false then
            ErrorBox(msg_errorsavingconfigfile,nil);
          Message(Application,evBroadcast,cmUpdate,nil);
        end;
    end;
  Dispose(D, Done);
end;

